#!/usr/bin/env python3
"""无头「跑图」驱动 + 体检 + 落盘：给 mode:=mapping 的建图跑一条脚本化覆盖路线。

设计要点（都是踩过的坑，写下来省得重踩）：
  · **MultiThreadedExecutor**：`/tf` 有 ~80 msg/s，单线程 `spin_once` 采样会永远在处理积压，
    "latest" 拿到十几秒前的旧值 ⇒ 会得出"LIO 滞后 13 s / 估计位移是真值的 2.6 倍"这种**假结论**
    （docs/worlds.md §5.1 的测量陷阱）。本工具用 MultiThreadedExecutor + 独立回调组，
    并且直接用 **/odom 话题**（不是 TF lookup）算 LIO 位姿 ⇒ 从根上避开那类饥饿。
  · **照抄真值做对齐**：仿真里有 `/odom_ground_truth`（planar_move，**world 系**）。
    LIO 的 `/odom` 是**出生点相对系**，两者差一个常量刚体变换 ⇒ 用开头静止 2 s 的平均值
    估出这个变换，再把 LIO 轨迹搬到 world 系算 ATE。这样报出来的是 **LIO 相对真值的误差**，
    而不是"两个坐标系不一样"这种废话。
  · **明确停**：Gazebo 的 planar_move 插件**没有命令超时**，停发就保持最后速度 ⇒
    本工具按 20 Hz 持续发 `/cmd_vel_chassis`，收尾必发 10 次零 Twist。
  · 60 s 一次 `ros2 topic hz` 之类的外部命令**不用**：直接在节点里数消息算频率。

用法（栈要先跑起来：ros2 launch ... mode:=mapping lio:=small_point_lio mapper:=slam_toolbox）：
  python3 tools/scripts/mapping/coverage_drive.py \
      --route .tmp_cache/spl2026/route.json \
      --out-json .tmp_cache/spl2026/drive.json \
      --save-2d src/rm_nav_bringup/map/RMUC2026_spl \
      --save-3d src/rm_nav_bringup/PCD/RMUC2026_spl.pcd \
      --spl-pcd-in-tree src/rm_localization/small_point_lio/pcd/scan.pcd \
      --dump-registered .tmp_cache/spl2026/registered_voxels.npz \
      --speed 0.30 --timeout 600

  --dry-run：只跟踪不发（打印路线/统计），用于核对参数。
退出码：0=正常跑完并（如要求）落盘成功；1=链路/落盘失败（原因在 stdout 与 JSON 里）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
MAP_Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def sh(cmd, timeout=60, cwd=None):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd)
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except Exception as e:  # noqa: BLE001
        return 124, 'ERR %s' % e


# --------------------------------------------------------------------- node
class DriveNode(Node):
    def __init__(self, args):
        super().__init__('coverage_drive')
        self.args = args
        self.lock = threading.Lock()
        self.grp = ReentrantCallbackGroup()
        self.lio = None          # (sim_t, x, y, z, yaw)
        self.gt = None
        self.sim_now = None
        self.wall_of_sim = None
        self.map_stat = None
        self.map_stat_first = None
        self.map_series = []
        self.lio_hist = []
        self.gt_hist = []
        self.clock_hist = []
        self.hz = {}
        self.reg_keys = []
        self.cloud_msg_count = 0
        self.cloud_tf_err = 0
        self.twist = Twist()

        self.create_subscription(Odometry, '/odom', self.on_lio, BE, callback_group=self.grp)
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_gt, BE,
                                 callback_group=self.grp)
        self.create_subscription(Clock, '/clock', self.on_clock, BE, callback_group=self.grp)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_Q,
                                 callback_group=self.grp)
        if args.dump_registered:
            self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, BE,
                                     callback_group=self.grp)
        self.pub = self.create_publisher(Twist, '/cmd_vel_chassis', 10)
        self._t0 = time.time()
        self._hz_count = {}

    # ---- callbacks
    def _bump(self, topic):
        c = self._hz_count.setdefault(topic, [0, time.time()])
        c[0] += 1

    def on_lio(self, m):
        with self.lock:
            self.lio = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                        yaw_of(m.pose.pose.orientation))
            self.lio_hist.append(self.lio)
        self._bump('/odom')

    def on_gt(self, m):
        with self.lock:
            self.gt = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                       yaw_of(m.pose.pose.orientation))
            self.gt_hist.append(self.gt)
        self._bump('/odom_ground_truth')

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        w = time.time()
        with self.lock:
            self.sim_now = t
            self.wall_of_sim = w
            self.clock_hist.append((t, w))
        self._bump('/clock')

    def on_map(self, m):
        d = np.asarray(m.data, dtype=np.int8)
        st = {'sim': self.sim_now,
              'w': m.info.width, 'h': m.info.height, 'res': m.info.resolution,
              'origin': [round(m.info.origin.position.x, 3), round(m.info.origin.position.y, 3)],
              'known': int((d >= 0).sum()), 'occ': int((d >= 65).sum()),
              'free': int(((d >= 0) & (d < 65)).sum())}
        st['known_m2'] = round(st['known'] * m.info.resolution ** 2, 1)
        with self.lock:
            self.map_stat = st
            if self.map_stat_first is None:
                self.map_stat_first = dict(st)
            self.map_series.append(st)
        self._bump('/map')

    def on_cloud(self, m):
        self.cloud_msg_count += 1
        try:
            from sensor_msgs_py import point_cloud2 as pc2
            pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:  # noqa: BLE001
            self.cloud_tf_err += 1
            return
        if pts is None or len(pts) == 0:
            return
        k = np.floor(np.asarray(pts, dtype=np.float64) / self.args.reg_voxel).astype(np.int32)
        self.reg_keys.append(k)
        self._bump('/cloud_registered')

    # ---- helpers
    def snapshot_lio(self):
        with self.lock:
            return self.lio

    def snapshot_gt(self):
        with self.lock:
            return self.gt

    def publish(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def stop(self, n=10):
        for _ in range(n):
            self.publish(0.0, 0.0)
            time.sleep(0.05)

    def measure_hz(self):
        """按最近 N s 的消息计数算频率。"""
        with self.lock:
            counts = {k: v[0] for k, v in self._hz_count.items()}
            t0 = min(v[1] for v in self._hz_count.values()) if self._hz_count else time.time()
        dur = max(time.time() - t0, 1e-6)
        return {k: round(c / dur, 2) for k, c in counts.items()}, round(dur, 1)

    def rtf(self):
        with self.lock:
            h = list(self.clock_hist)
        if len(h) < 3:
            return None
        return round((h[-1][0] - h[0][0]) / max(h[-1][1] - h[0][1], 1e-6), 3)


# --------------------------------------------------------------- 轨迹分析
def analyze(lio_hist, gt_hist):
    """把 LIO 轨迹搬到 world 系后与真值比：初始偏移 / 漂移 / ATE / 位移比。"""
    if len(lio_hist) < 5 or len(gt_hist) < 5:
        return {'error': 'samples too few', 'n_lio': len(lio_hist), 'n_gt': len(gt_hist)}
    L = np.asarray(lio_hist, dtype=float)   # t,x,y,z,yaw
    G = np.asarray(gt_hist, dtype=float)
    # 用最近邻把 GT 配到 LIO 时间戳（两条都是仿真钟）
    idx = np.searchsorted(G[:, 0], L[:, 0])
    idx = np.clip(idx, 1, len(G) - 1)
    left = np.abs(G[idx - 1, 0] - L[:, 0]) < np.abs(G[idx, 0] - L[:, 0])
    j = np.where(left, idx - 1, idx)
    dt = np.abs(G[j, 0] - L[:, 0])
    ok = dt < 0.15
    L, Gj, dt = L[ok], G[j][ok], dt[ok]
    if len(L) < 5:
        return {'error': 'no time-matched pairs', 'n_lio': len(lio_hist), 'n_gt': len(gt_hist)}
    # 初始刚体（用开头 10% 且至少 5 个样本估）：R = Rz(gt_yaw - lio_yaw)，t = gt_xy - R*lio_xy
    k = max(5, int(0.1 * len(L)))
    dyaw = np.unwrap(Gj[:k, 4]) - np.unwrap(L[:k, 4])
    yaw0 = float(np.arctan2(np.sin(dyaw).mean(), np.cos(dyaw).mean()))
    c, s = math.cos(yaw0), math.sin(yaw0)
    R = np.array([[c, -s], [s, c]])
    t0 = (Gj[:k, 1:3] - L[:k, 1:3] @ R.T).mean(axis=0)
    est = L[:, 1:3] @ R.T + t0
    err = np.linalg.norm(est - Gj[:, 1:3], axis=1)
    # 轨迹长度
    def plen(a):
        return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1))) if len(a) > 1 else 0.0
    len_lio = plen(est)
    len_gt = plen(Gj[:, 1:3])
    z_err = L[:, 3] - Gj[:, 3]
    return {
        'n_pairs': int(len(L)),
        'pair_dt_mean_s': round(float(dt.mean()), 4),
        'pair_dt_max_s': round(float(dt.max()), 4),
        'align': {'yaw_offset_rad': round(yaw0, 4),
                  'translation': [round(float(t0[0]), 4), round(float(t0[1]), 4)]},
        'ate_xy_rmse_m': round(float(np.sqrt((err ** 2).mean())), 4),
        'ate_xy_mean_m': round(float(err.mean()), 4),
        'ate_xy_max_m': round(float(err.max()), 4),
        # "漂移" = 后半段 ATE 均值 - 前半段（扣掉常量坐标系偏差后仍在长的是真漂移）
        'ate_head_mean_m': round(float(err[:len(err) // 2].mean()), 4),
        'ate_tail_mean_m': round(float(err[len(err) // 2:].mean()), 4),
        'traj_len_lio_m': round(len_lio, 3),
        'traj_len_gt_m': round(len_gt, 3),
        'traj_len_ratio': round(len_lio / len_gt, 4) if len_gt > 1e-9 else None,
        'disp_ratio_final': round(float(np.linalg.norm(est[-1] - est[0]) /
                                        max(np.linalg.norm(Gj[-1, 1:3] - Gj[0, 1:3]), 1e-9)), 4),
        'z_err_mean_m': round(float(z_err.mean()), 4),
        'z_err_std_m': round(float(z_err.std()), 4),
    }


# --------------------------------------------------------------- 路径跟踪
class PathFollower:
    """折线路径的纯追踪（pure pursuit）跟随器。

    为什么不用"朝当前航点直冲 + 原地转向"（第一版的做法，实测踩坑）：
      · 该算法在仿真里会让车频繁**原地/低速大角速度**转，而本槽位的 LIO 对"帧内旋转"最敏感
        —— livox 插件把所有点的 offset_time 置 0（一帧 126 ms 的运动没被补偿，见
        config/mid360_sim_tuned.yaml 头部①），旋转越快、点云被拉得越歪，位姿越抖、地图越发虚。
      · 实测证据：run1 用"直冲+原地转"跑，LIO 轨迹长度 40.7 m vs 真值 27.1 m（**1.50×**）、
        ATE max 1.61 m，存下来的 2D 图墙面糊成 1 m 宽带。
    纯追踪则总是朝**前方 lookahead 米处**的一个"胡萝卜点"走 ⇒ 转弯是大半径圆弧、速度不掉 0，
    实测把 1.50× 降到 1.1× 量级（见 docs/mapping_small_point_lio.md 的 A/B）。
    """

    def __init__(self, pts, lookahead):
        self.pts = [(float(a), float(b)) for a, b in pts]
        self.look = float(lookahead)
        self.seg = [0.0]
        for k in range(1, len(self.pts)):
            self.seg.append(self.seg[-1] + math.hypot(self.pts[k][0] - self.pts[k - 1][0],
                                                      self.pts[k][1] - self.pts[k - 1][1]))
        self.total = self.seg[-1]
        self.i = 0
        self.s_prev = 0.0

    def project(self, x, y, heading=None):
        """把 (x,y) 投影到折线上 → 弧长 s。

        ⚠️ 两条约束都是踩坑加的：
          · **只从当前段往后搜**（不能往前/全局搜）：NN 序的路线会反复经过出生点附近，
            全局最近点会跳到"后面某条经过这里的 lane"上 ⇒ 索引瞬移、胡萝卜跑到 4 m 外
            （v1 实测：车没动，索引已经到 wp6）。
          · **单调不回退**：测距噪声会让投影在段边界来回跳 ⇒ 索引抖动。
        做法：从当前段起找第一个"距离 < 0.4 m"的段（那就是所在段）；都没有才取最小距离那个。
        """
        # ⚠️ 第四个坑：self.i 的语义是"**下一个**要去的航点序号" ⇒ 车当前所在的段是 i-1，
        #    搜索必须从 i-1 起。从 i 起搜（v3 的写法）会让"当前段"永远差一段：车走在第 1 段上、
        #    却拿第 2 段去量距离 ⇒ 距离恒 > 1 m ⇒ s 卡死在 s_prev、胡萝卜停在 0.9 m 处不动
        #    （实测：车到 y=3.2 了胡萝卜还在 y=1.58）。
        lo = max(0, self.i - 1)
        hi = min(len(self.pts) - 2, self.i + 3)
        best = (1e18, self.s_prev)
        hit = None
        for k in range(lo, hi + 1):
            ax, ay = self.pts[k]
            bx, by = self.pts[k + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(x - px, y - py)
            s = self.seg[k] + t * math.hypot(dx, dy)
            if d < 0.4:
                # ⚠️ 第五个坑：折返路线（1.5 m 的掉头）会让"上行/下行"两条 lane 相距只有
                #    0.05~0.1 m，局部投影**本质歧义**（run2 实测：车在掉头处来回摆、看着像卡住）。
                #    判据用**航向**：候选段的"远端"若在车头前方（(end-pos)·heading > 0）才是
                #    "车正在走的那一段"；两条近平行 lane 里只有一个满足。
                endp = (bx, by) if t < 0.5 or True else (ax, ay)
                if heading is not None:
                    dot = math.cos(heading) * (bx - x) + math.sin(heading) * (by - y)
                else:
                    dot = 0.0
                if hit is None or dot > hit[0]:
                    hit = (dot, s)
            if d < best[0]:
                best = (d, s)
        if hit is not None and hit[0] >= -0.2:
            s = max(hit[1], self.s_prev)
            self.s_prev = s
            return s
        if hit is not None and best[0] > 1.0:
            s = max(hit[1], self.s_prev)      # 航向也不明确、且离所有段都远 ⇒ 用命中段的 s
            self.s_prev = s
            return s
        # ⚠️ 第三个坑：所有候选段都远（>1 m）时**绝不能**取"最小距离那个段"的弧长 ——
        #    NN 序路线会折返，远处某条 lane 的端点可能"最小"，于是索引瞬移、胡萝卜飞到几米外
        #    （v2 实测：车还在 s=0.95，索引已经跳到 6、s 跳到 13）。此时保持 s 不动最安全。
        if best[0] > 1.0:
            return self.s_prev
        s = max(best[1], self.s_prev)         # 单调不回退
        self.s_prev = s
        return s

    def at(self, s):
        s = max(0.0, min(self.total, s))
        k = 0
        while k < len(self.seg) - 2 and self.seg[k + 1] < s:
            k += 1
        L = self.seg[k + 1] - self.seg[k]
        t = 0.0 if L < 1e-9 else (s - self.seg[k]) / L
        ax, ay = self.pts[k]
        bx, by = self.pts[k + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)

    def update(self, x, y, heading=None):
        """→ (carrot_x, carrot_y, 已通过的航点数 i, 是否走完)"""
        s = self.project(x, y, heading)
        while self.i < len(self.pts) - 1 and s > self.seg[self.i]:
            self.i += 1
        cx, cy = self.at(s + self.look)
        return cx, cy, self.i, (s >= self.total - 0.15)


def pair_series(lio_hist, gt_hist, hz=2.0):
    """按**仿真时间**把 LIO 与真值配成对，抽稀到 hz 条/s —— 诊断"哪一段开始飘"用。"""
    if len(lio_hist) < 5 or len(gt_hist) < 5:
        return []
    L = np.asarray(lio_hist, dtype=float)
    G = np.asarray(gt_hist, dtype=float)
    idx = np.clip(np.searchsorted(G[:, 0], L[:, 0]), 1, len(G) - 1)
    left = np.abs(G[idx - 1, 0] - L[:, 0]) < np.abs(G[idx, 0] - L[:, 0])
    j = np.where(left, idx - 1, idx)
    ok = np.abs(G[j, 0] - L[:, 0]) < 0.15
    L, Gj = L[ok], G[j][ok]
    out, last = [], -1e9
    for a, b in zip(L, Gj):
        if a[0] - last < 1.0 / hz:
            continue
        last = a[0]
        out.append([round(float(a[0]), 2),
                    round(float(a[1]), 3), round(float(a[2]), 3), round(float(a[4]), 4),
                    round(float(b[1]), 3), round(float(b[2]), 3), round(float(b[4]), 4),
                    round(float(math.hypot(a[1] - b[1], a[2] - b[2])), 4)])
    return out


# ------------------------------------------------------------------ 落盘
def save_2d(prefix, log):
    rc, out = sh(['ros2', 'topic', 'info', '/map', '-v'], timeout=30)
    pubs = [ln.strip() for ln in out.splitlines() if 'Publisher count' in ln or 'Node name' in ln]
    log('[save2d] /map 拓扑: %s' % ' | '.join(pubs[:12]))
    # ⚠️ map_saver_cli 的 CLI：-t/-f/--occ/--free/--fmt/--mode，**--ros-args 必须放最后**
    #    （阈值与既有资产对齐：free 0.25 / occ 0.65，见 map/RMUC2026.yaml 的生成口径）
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix, '-t', '/map',
           '--free', '0.25', '--occ', '0.65',
           '--ros-args', '-p', 'save_map_timeout:=60.0']
    rc, out = sh(cmd, timeout=180)
    log('[save2d] rc=%d %s' % (rc, out.strip()[-400:]))
    res = {'rc': rc, 'cmd': ' '.join(cmd), 'out': out.strip()[-800:],
           'publishers': pubs, 'files': {}}
    for ext in ('.pgm', '.yaml'):
        p = prefix + ext
        res['files'][ext] = {'path': p, 'exists': os.path.isfile(p),
                             'bytes': os.path.getsize(p) if os.path.isfile(p) else None}
    return res


def save_3d(dest, in_tree, log, wait_s=120):
    """调 small_point_lio 的 /map_save（Trigger），等 ROOT_DIR/pcd/scan.pcd 落地，再搬到 dest。

    ⚠️ 源码事实（small_point_lio_node.cpp:37-51）：服务**立刻**回 success=true，
    真正的 write_pcd 在**分离线程**里做 ⇒ 必须轮询文件"出现且大小稳定"，不能只看服务返回。
    """
    t_call = time.time()
    # ⚠️ 顺序很重要：**先删旧文件、再发服务**。write_pcd 跑在分离线程里，服务几乎立刻回
    #    success=true（small_point_lio_node.cpp:34-50）—— 若等服务返回后再删，就会把那条线程
    #    已经 fopen 出来、正在写的文件 unlink 掉（fd 还有效但目录项没了）⇒ 表现是
    #    "日志里 save pcd success，但文件哪儿都找不到"（本工具第一版就踩了这个坑，实测复现）。
    if os.path.isfile(in_tree):
        os.unlink(in_tree)
    rc, out = sh(['ros2', 'service', 'call', '/map_save', 'std_srvs/srv/Trigger', '{}'], timeout=60)
    log('[save3d] /map_save rc=%d out=%s' % (rc, out.strip()[-300:]))
    res = {'service_rc': rc, 'service_out': out.strip()[-400:],
           'in_tree': in_tree, 'dest': dest}
    deadline = time.time() + wait_s
    last = None
    stable = 0
    while time.time() < deadline:
        if os.path.isfile(in_tree):
            sz = os.path.getsize(in_tree)
            if sz == last and sz > 0:
                stable += 1
                if stable >= 3:
                    break
            else:
                stable = 0
            last = sz
        time.sleep(0.5)
    if not os.path.isfile(in_tree) or not last:
        res['ok'] = False
        res['error'] = '等 %.0f s 没等到 %s（服务返回 success 不代表写完了）' % (wait_s, in_tree)
        log('[save3d] ❌ %s' % res['error'])
        return res
    res['in_tree_bytes'] = os.path.getsize(in_tree)
    res['in_tree_mtime_after_call'] = os.path.getmtime(in_tree) >= t_call - 1
    os.makedirs(os.path.dirname(os.path.abspath(dest)) or '.', exist_ok=True)
    sh(['cp', in_tree, dest], timeout=60)
    res['dest_bytes'] = os.path.getsize(dest) if os.path.isfile(dest) else None
    res['ok'] = bool(res['dest_bytes'])
    # 源码树里的那份删掉（包内 *.pcd 已在 .gitignore，但别留垃圾）
    try:
        os.unlink(in_tree)
        res['in_tree_removed'] = True
    except OSError as e:
        res['in_tree_removed'] = 'FAILED %s' % e
    log('[save3d] %s → %s（%s B；源码树副本已删=%s）'
        % (in_tree, dest, res['dest_bytes'], res.get('in_tree_removed')))
    return res


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='建图跑图：脚本化覆盖 + 体检 + 落盘')
    ap.add_argument('--route', required=True, help='coverage_route.py 生成的 route.json')
    ap.add_argument('--out-json', help='体检结果 JSON 输出路径')
    ap.add_argument('--speed', type=float, default=0.30, help='最大线速度（m/s）')
    ap.add_argument('--spin-rate', type=float, default=0.7, help='原地旋转角速度（rad/s）')
    ap.add_argument('--yaw-gain', type=float, default=1.2, help='航向 P 增益（w = gain*herr）')
    ap.add_argument('--yaw-rate-max', type=float, default=0.8, help='角速度上限（rad/s）')
    ap.add_argument('--rot-in-place-rad', type=float, default=2.2,
                    help='|航向误差| 超过它就只原地转（纯追踪下很少触发）')
    ap.add_argument('--lookahead', type=float, default=0.8,
                    help='纯追踪前视距离（m）：越大越平滑、越小越贴线')
    ap.add_argument('--turn-slow', type=float, default=0.45,
                    help='转弯降速系数：v *= 1 - turn_slow*|herr|/pi')
    ap.add_argument('--pose-source', choices=['lio', 'gt'], default='lio',
                    help='用谁做闭环控制：lio=自己的 /odom（真实用法）| gt=/odom_ground_truth'
                         '（仿真专用：让"跑图"不被 LIO 抖动带偏，LIO 质量另外用 ATE 单独报）')
    ap.add_argument('--spin-every', type=int, default=7, help='--spin-all-turns 时每过几个航点转一次')
    ap.add_argument('--stuck-max', type=int, default=3, help='同一处连续卡几次就跳过')
    ap.add_argument('--reach-tol', type=float, default=0.30, help='到点判定（m）')
    ap.add_argument('--spin-at', default='', help='在这些"第几个航点"处做原地旋转，逗号分隔（从 0 计）')
    ap.add_argument('--spin-turns', type=float, default=1.0, help='每次原地旋转圈数')
    ap.add_argument('--final-spin', type=float, default=0.0,
                    help='路线跑完后原地转几圈（>0 才转；放在最后以免把地图转糊）')
    ap.add_argument('--spin-all-turns', type=float, default=0.0,
                    help='>0 时每到一个航点都先转这么多圈（0=不做）')
    ap.add_argument('--warmup', type=float, default=6.0, help='发车前静止等待（s，让 LIO 收敛+对齐用）')
    ap.add_argument('--timeout', type=float, default=900.0, help='整程墙钟上限（s）')
    ap.add_argument('--stuck-sec', type=float, default=6.0, help='多久没进展算卡住')
    ap.add_argument('--save-2d', help='map_saver_cli 的前缀（不含扩展名）')
    ap.add_argument('--save-3d', help='PCD 目标路径（/map_save 产物会搬到这里）')
    ap.add_argument('--spl-pcd-in-tree', default='src/rm_localization/small_point_lio/pcd/scan.pcd',
                    help='small_point_lio 硬编码的落盘路径（ROOT_DIR/pcd/scan.pcd）')
    ap.add_argument('--dump-registered', help='把 /cloud_registered 体素化后存 npz（A/B 证据用）')
    ap.add_argument('--reg-voxel', type=float, default=0.05, help='--dump-registered 的体素边长')
    ap.add_argument('--dry-run', action='store_true', help='只打印，不发车不落盘')
    args = ap.parse_args()

    route = json.load(open(args.route))
    wps = route['waypoints']
    print('[drv] route: %d waypoints, planned %.1f m, frame=%s, clearance=%.2f'
          % (len(wps), route['stats']['path_len_m'], route.get('frame'), route.get('clearance')))

    rclpy.init()
    node = DriveNode(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    report = {'route_stats': route['stats'], 'route_file': os.path.abspath(args.route),
              'args': {k: v for k, v in vars(args).items()}, 'warnings': [], 'events': []}
    warn = report['warnings'].append

    # ---- 等链路
    t_start = time.time()
    last_warn = 0
    while time.time() - t_start < 120:
        lio, gt = node.snapshot_lio(), node.snapshot_gt()
        if lio and gt:
            break
        if time.time() - last_warn > 10:
            last_warn = time.time()
            print('[drv] 等 /odom=%s /odom_ground_truth=%s ...' % (bool(lio), bool(gt)))
        time.sleep(0.5)
    lio, gt = node.snapshot_lio(), node.snapshot_gt()
    if not lio:
        print('[drv] ❌ 没有 /odom（LIO 没出数）⇒ 直接收尾')
        warn('no /odom')
    if not gt:
        warn('no /odom_ground_truth（无真值，只能报 LIO 自身轨迹）')
    report['link_ready'] = {'odom': bool(lio), 'ground_truth': bool(gt),
                            'wait_s': round(time.time() - t_start, 1)}

    # ---- 静止对齐段：LIO 出生点 vs 真值出生点
    print('[drv] 静止 %.1f s（LIO 收敛 + 记录初始偏移）' % args.warmup)
    time.sleep(max(0.0, args.warmup))
    lio0, gt0 = node.snapshot_lio(), node.snapshot_gt()
    report['start'] = {
        'lio': [round(v, 4) for v in lio0] if lio0 else None,
        'gt': [round(v, 4) for v in gt0] if gt0 else None}
    if lio0 and gt0:
        report['start']['lio_minus_gt_in_spawn_frame_offset'] = [
            round(lio0[1] - gt0[1], 4), round(lio0[2] - gt0[2], 4)]
    # ⚠️ --pose-source gt 时，/odom_ground_truth 是 **world 系**（planar_move 发 model 的
    #    世界位姿），而路线是 **spawn(map) 系**（先验图/出生点相对系）⇒ 必须把世界系搬到
    #    spawn 系，否则控制器会朝"世界系里的那个点"开（实测：车一路往西跑，胡萝卜在 4 m 外
    #    够不着）。用静止段的第一个 GT 样本做平移 + 初始 yaw。
    gt_origin = [None]

    def pose_now():
        if args.pose_source == 'gt':
            sm = node.snapshot_gt()
            if sm is None or gt_origin[0] is None:
                return None
            ox, oy, oyaw = gt_origin[0]
            dx, dy = sm[1] - ox, sm[2] - oy
            c, sn = math.cos(-oyaw), math.sin(-oyaw)
            return (sm[0], c * dx - sn * dy, sn * dx + c * dy, sm[3], wrap(sm[4] - oyaw))
        return node.snapshot_lio()

    if args.pose_source == 'gt' and gt0 is not None:
        gt_origin[0] = (gt0[1], gt0[2], gt0[4])
        print('[drv] GT→spawn 系对齐：origin=(%.4f, %.4f) yaw=%.4f'
              % (gt0[1], gt0[2], gt0[4]))
    report['hz_warmup'] = node.measure_hz()
    report['rtf_warmup'] = node.rtf()
    print('[drv] 起点: lio=%s gt=%s rtf=%s' %
          (report['start']['lio'], report['start']['gt'], report['rtf_warmup']))

    # ---- 覆盖路线跟踪
    spin_at = set()
    if args.spin_at:
        for tok in args.spin_at.split(','):
            tok = tok.strip()
            if tok:
                spin_at.add(int(tok))
    if not spin_at and len(wps) >= 6 and not args.spin_all_turns:
        spin_at = {2, len(wps) // 2, len(wps) - 2}   # 默认三处原地旋转
    spin_at = {i for i in spin_at if 0 <= i < len(wps)}
    print('[drv] 原地旋转航点: %s（每次 %.2f 圈 @ %.2f rad/s）'
          % (sorted(spin_at), args.spin_turns, args.spin_rate))

    follow = PathFollower(wps, args.lookahead)
    t_drive0_sim = node.sim_now
    t_drive0_wall = time.time()
    state = 'goto'
    spin_target = 0.0
    spin_done = 0.0
    yaw_prev = None
    last_motion = (None, None, None)      # 上一次"有运动"的 (x, y, yaw)
    stuck_since = time.time()
    stuck_count = 0
    spins = 0
    skipped = []
    reached = 0
    cmd = (0.0, 0.0)
    odo_len = 0.0
    last_pos = None
    traj = []          # [wall_t, sim_t, x, y, yaw, v_cmd, w_cmd] 每 0.2 s（诊断用）
    n_motion_checks = 0
    n_motion_fail = 0

    def begin_spin(turns):
        nonlocal state, spin_target, spin_done
        state = 'spin'
        spin_target = abs(turns) * 2 * math.pi
        spin_done = 0.0

    while time.time() - t_drive0_wall < args.timeout:
        src = pose_now()
        if src is None:
            time.sleep(0.05)
            continue
        t_sim, x, y, _, yaw = src
        if last_pos is not None:
            odo_len += math.hypot(x - last_pos[0], y - last_pos[1])
        last_pos = (x, y)
        w_t = time.time() - t_drive0_wall
        if not traj or (w_t - traj[-1][0]) > 0.2:
            traj.append([round(w_t, 2), round(t_sim or 0.0, 2), round(x, 3), round(y, 3),
                         round(yaw, 4), round(cmd[0], 3), round(cmd[1], 3)])
        # 卡住检测：以"**任何**运动"为准（原地旋转也算进展）。
        # ⚠️ 第一版只看"到目标的距离变小" ⇒ 原地转向时被误判；第二版把阈值设成"每 50 ms
        #    3 cm / 0.05 rad" ⇒ 慢速原地转（<1 rad/s）又会被误判。这里改成"与上一次检测点比"，
        #    并且只有在真的连续 --stuck-sec 秒**一点都没动**时才处理。
        mx, my, myaw = last_motion
        n_motion_checks += 1
        if mx is None or math.hypot(x - mx, y - my) > 0.03 or abs(wrap(yaw - myaw)) > 0.03:
            last_motion = (x, y, yaw)
            stuck_since = time.time()
        elif time.time() - stuck_since > args.stuck_sec:
            n_motion_fail += 1

        cx, cy, i, done = follow.update(x, y, yaw)
        if i > reached:
            for k in range(reached, i):
                report['events'].append({'t': round(w_t, 1), 'event': 'wp_reached', 'wp': k,
                                         'odo_len': round(odo_len, 2)})
            print('[drv] 通过 wp%d/%d（胡萝卜 (%5.2f,%5.2f)）轨迹长 %.1f m'
                  % (i, len(wps), cx, cy, odo_len))
            reached = i
            stuck_count = 0
            stuck_since = time.time()
            if state != 'spin' and args.spin_all_turns > 0 and (i % args.spin_every) == 0:
                begin_spin(args.spin_all_turns)
        if done and state != 'spin':
            print('[drv] 路线走完（%.1f m / 计划 %.1f m）' % (odo_len, follow.total))
            break

        dx, dy = cx - x, cy - y
        dist = math.hypot(dx, dy)
        if state == 'spin':
            if yaw_prev is not None:
                spin_done += abs(wrap(yaw - yaw_prev))
            cmd = (0.0, args.spin_rate)
            if spin_done >= spin_target:
                state = 'goto'
                spins += 1
                print('[drv] 原地旋转 #%d 完成（%.2f rad）@ wp%d' % (spins, spin_done, i))
                report['events'].append({'t': round(w_t, 1), 'event': 'spin_done', 'wp': i,
                                         'rad': round(spin_done, 2)})
        else:
            herr = wrap(math.atan2(dy, dx) - yaw)
            # 纯追踪：朝 lookahead 处的"胡萝卜点"走；转弯是大圆弧、速度不掉 0。
            # 只在"目标几乎在正后方"（|herr|>2.2 rad）时才原地转，那种情况直冲会画大圈。
            if abs(herr) > args.rot_in_place_rad:
                cmd = (0.0, max(-args.yaw_rate_max, min(args.yaw_rate_max, args.yaw_gain * herr)))
            else:
                slow = 1.0 - args.turn_slow * min(1.0, abs(herr) / math.pi)
                v = max(0.05, min(args.speed, args.speed * max(0.35, min(1.0, dist / 0.6)))) * slow
                cmd = (v, max(-args.yaw_rate_max, min(args.yaw_rate_max, args.yaw_gain * herr)))
            if time.time() - stuck_since > args.stuck_sec:
                stuck_count += 1
                print('[drv] ⚠️ %.1f s 没有任何运动（胡萝卜 (%5.2f,%5.2f) dist=%.2f，第 %d 次）⇒ 倒退+转向'
                      % (args.stuck_sec, cx, cy, dist, stuck_count))
                report['events'].append({'t': round(w_t, 1), 'event': 'stuck', 'wp': i,
                                         'dist': round(dist, 3)})
                if stuck_count >= args.stuck_max:
                    skipped.append(i)
                    warn('wp%d 连续卡 %d 次已跳过' % (i, args.stuck_max))
                    follow.i = min(follow.i + 1, len(wps) - 1)
                    reached = follow.i
                    stuck_count = 0
                    stuck_since = time.time()
                    continue
                for _ in range(12):
                    node.publish(-0.14, 0.0)
                    time.sleep(0.05)
                for _ in range(16):
                    node.publish(0.0, args.yaw_rate_max if herr > 0 else -args.yaw_rate_max)
                    time.sleep(0.05)
                stuck_since = time.time()
                continue
        node.publish(*cmd)
        yaw_prev = yaw
        time.sleep(0.05)

    # ---- 收尾原地旋转（默认 0）：放在**路线跑完之后**，这样"原地转"不会把地图转糊
    if args.final_spin > 0:
        print('[drv] 收尾原地旋转 %.2f 圈 @ %.2f rad/s' % (args.final_spin, args.spin_rate))
        spun, yaw_prev2, t_spin0 = 0.0, None, time.time()
        while spun < args.final_spin * 2 * math.pi and time.time() - t_spin0 < 90:
            src = pose_now()
            if src is None:
                time.sleep(0.05)
                continue
            yaw = src[4]
            if yaw_prev2 is not None:
                spun += abs(wrap(yaw - yaw_prev2))
            yaw_prev2 = yaw
            node.publish(0.0, args.spin_rate)
            time.sleep(0.05)
        node.stop(10)
        report['events'].append({'t': round(time.time() - t_drive0_wall, 1),
                                 'event': 'final_spin', 'rad': round(spun, 2)})
        print('[drv] 收尾旋转完成 %.2f rad' % spun)

    node.stop(20)
    t_drive1_wall = time.time()
    t_drive1_sim = node.sim_now
    print('[drv] 行驶结束：通过航点 %d/%d，跳过 %d，原地旋转 %d 次，轨迹长 %.1f m，墙钟 %.0f s'
          % (reached, len(wps), len(skipped), spins, odo_len, t_drive1_wall - t_drive0_wall))
    print('[drv] 运动检测：检查 %d 次，判定"完全没动" %d 次'
          % (n_motion_checks, n_motion_fail))

    # ---- 收尾测量（等地图/真值更新）
    time.sleep(3.0)
    node.stop(5)
    report['drive'] = {
        'waypoints_total': len(wps), 'waypoints_reached': reached,
        'waypoints_skipped': skipped, 'spins': spins,
        'odo_path_len_m': round(odo_len, 2),
        'planned_path_len_m': route['stats']['path_len_m'],
        'wall_s': round(t_drive1_wall - t_drive0_wall, 1),
        'sim_s': round((t_drive1_sim - t_drive0_sim), 1) if (t_drive1_sim and t_drive0_sim) else None,
        'finished_all_waypoints': bool(reached >= len(wps) - 1),
        'pose_source': args.pose_source,
        'motion_checks': n_motion_checks, 'motion_checks_failed': n_motion_fail,
    }
    report['hz_drive'] = node.measure_hz()
    report['rtf_drive'] = node.rtf()
    report['maps'] = {
        'first': node.map_stat_first, 'last': node.map_stat,
        'n_updates': len(node.map_series),
        'series': node.map_series[:1] + node.map_series[::max(1, len(node.map_series) // 12)][1:],
    }
    with node.lock:
        lio_hist = list(node.lio_hist)
        gt_hist = list(node.gt_hist)
    report['lio_hist_n'] = len(lio_hist)
    report['gt_hist_n'] = len(gt_hist)
    report['traj_cmd'] = traj
    # 真值轨迹（同样 0.2 s 抽稀）——用于"车到底走了哪"的复核
    gt_traj = []
    for s in gt_hist:
        if not gt_traj or (s[0] - gt_traj[-1][0]) > 0.2:
            gt_traj.append([round(s[0], 2), round(s[1], 3), round(s[2], 3), round(s[4], 4)])
    report['traj_gt'] = gt_traj
    report['lio_vs_truth'] = analyze(lio_hist, gt_hist)
    report['pair_series'] = pair_series(lio_hist, gt_hist)
    report['cloud_registered_msgs'] = node.cloud_msg_count
    if node.cloud_msg_count == 0 and args.dump_registered:
        warn('/cloud_registered 一帧都没收到（订阅者数 0 时节点不发布，见 small_point_lio_node.cpp:102）')

    # ---- 落盘
    if args.save_2d or args.save_3d:
        node.stop(3)
    report['saves'] = {}
    if args.save_2d:
        report['saves']['map_2d'] = save_2d(args.save_2d, print)
    if args.save_3d:
        report['saves']['pcd_3d'] = save_3d(args.save_3d, args.spl_pcd_in_tree, print)
    node.stop(5)

    if args.dump_registered and node.reg_keys:
        keys = np.concatenate(node.reg_keys, axis=0)
        uniq = np.unique(keys.view([('a', '<i4'), ('b', '<i4'), ('c', '<i4')]).ravel())
        arr = uniq.view(np.int32).reshape(-1, 3)
        os.makedirs(os.path.dirname(os.path.abspath(args.dump_registered)), exist_ok=True)
        np.savez_compressed(args.dump_registered, voxel=np.asarray(arr, dtype=np.int32),
                            leaf=np.array([args.reg_voxel]))
        report['registered_dump'] = {'file': args.dump_registered, 'voxel': args.reg_voxel,
                                     'n_voxels': int(len(arr)),
                                     'bbox_voxel_m': [round(float(arr[:, k].min()) * args.reg_voxel, 3)
                                                      for k in range(3)] +
                                                     [round(float(arr[:, k].max()) * args.reg_voxel, 3)
                                                      for k in range(3)]}
        print('[drv] /cloud_registered 体素 %.2f m：%d 个体素 → %s'
              % (args.reg_voxel, len(arr), args.dump_registered))

    print('[drv] SUMMARY ' + json.dumps({
        'rtf_warmup': report.get('rtf_warmup'), 'rtf_drive': report.get('rtf_drive'),
        'hz_drive': report.get('hz_drive'), 'drive': report['drive'],
        'map_first': report['maps']['first'], 'map_last': report['maps']['last'],
        'lio_vs_truth': report['lio_vs_truth']}, ensure_ascii=False))
    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, 'w') as f:
            json.dump(report, f, indent=1, default=str)
        print('[drv] 写出 %s' % args.out_json)

    ex.shutdown()
    rclpy.shutdown()
    ok = True
    for k, v in report['saves'].items():
        if not v.get('ok', v.get('rc') == 0):
            ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
