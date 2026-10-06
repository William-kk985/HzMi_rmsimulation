#!/usr/bin/env python3
"""LIO 漂移诊断：**分段脚本化路线**驱动 + 同步高频采样（CSV/JSON），回答"跑一小会儿里程计就飘"。

为什么要有这个工具（而不是直接跑 coverage_drive.py）：
  · coverage_drive.py 回答的是"覆盖路线跑完没有、图存下来没有"；它**不分段**，
    所以没法把误差归因到"直线 / 慢转 / 快转 / 原路返回 / 窄道"这几种运动模式上。
  · 本工具把路线切成**带标签的段**（`A` 直线、`Ta` 过渡原地转、`D` 原路返回、
    `B` 慢转、`C` 快转、`E` 窄沟/坡道口），每段起止都写进事件表（仿真时间 + 墙钟），
    分析脚本按事件切片 ⇒ 每一段都能单独报 ATE / 每米漂移。
  · 同时**逐帧落盘**一条 50 Hz 的时间轴（CSV），把 LIO、真值、两条 TF 边、
    融合位姿、指令 Twist、`/scan` 体检量、RTF 放在同一张表上 ⇒ 后面所有相关性
    都是"同一时间轴上的数"，不是事后拼的。

⚠️ 采样饥饿陷阱（`docs/worlds.md` §5.1，本仓踩过的坑，本工具从根上避开）：
   单线程 `rclpy.spin_once` + "取最新值"在 ~80 Hz 的 `/tf` 洪水下会被饿死 ⇒
   拿到十几秒前的旧值 ⇒ 会得出"LIO 滞后 13 s / 估计位移是真值的 2.6 倍"这种**假结论**。
   本工具：① `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup`；
   ② 回调里**只做 O(1) 赋值**（不做 TF lookup、不做文件写、不做点云解析）；
   ③ 落盘由独立的 50 Hz 定时器做，并且**每条记录都带上"这条数据有多旧"**
      （`lio_age_sim` / `lio_age_wall` / `tf_*_age`）⇒ 万一还是旧了，CSV 里看得出来，
      分析脚本可以据此把可疑行标出来，而不是默默给出错误结论。

隔离/跑法（必须和 launch 塞进**同一次 bash 调用**，见 tools/scripts/mapping/run_mapping_headless.sh）：
  HOME=/tmp/gzhome-<tag>  ROS_DOMAIN_ID=<非默认>  GAZEBO_MASTER_URI=http://127.0.0.1:<port>  unset DISPLAY

用法：
  python3 tools/scripts/diag/segment_drive.py --dry-run                  # 只看分段计划
  python3 tools/scripts/diag/segment_drive.py --dry-run --spec my.json   # 看自定义分段
  python3 tools/scripts/diag/segment_drive.py \
      --out-csv .tmp_cache/diag/spl.csv --out-json .tmp_cache/diag/spl.json \
      --pose-source gt --warmup 10 --timeout 600

  退出码：0=跑完（哪怕有 jam，jam 事实记在 JSON 里）；1=链路起不来（没有 /odom 或没有真值）。

闭环用谁做反馈：`--pose-source gt`（默认）用 `/odom_ground_truth` 闭环 ⇒ 车实际走的
几何形状是**确定的**，四段之间可以互相比；LIO 的质量另外用 ATE 单独算。
`--pose-source lio` 才是"真实用法"（LIO 闭环），会引入正反馈（LIO 飘 ⇒ 车跑歪 ⇒ 更难比），
本诊断默认不用。

⚠️ 仿真插件的硬约束：`libgazebo_ros_planar_move.so` **没有命令超时**，停发就保持最后速度。
本工具的指令发布定时器（`--cmd-hz`，默认 **100 Hz**）**独立于控制循环**一直在发，
收尾还会连发 20 次零 Twist，并且任何异常路径都走 `finally` 里的收尾。

⚠️ 第二个坑（首跑实测）：**本仓仿真底盘对角速度的"权威"远低于指令** ——
   `cmd_w=0.6 rad/s` 就地旋转 → 真值实测只有 **0.0214 rad/s（1/28）**；`cmd_w=0.9` 且
   v=0.09 m/s → 0.098 rad/s（1/9.2）。同一次跑里 LIO 与真值的偏航**逐帧一致**，
   所以不是"真值在骗人"，是底盘真的没转那么快。⇒ 所有旋转段都用 **GT 偏航率闭环**
   （`cmd = target + kp*(target - measured)`，上限 `--spin-cmd-cap`），
   这样分段描述里的"慢转 0.35 / 快转 1.5 rad/s"指的是**实际发生的角速度**。
   `S` 段（`spin_sweep`）专门把"指令→实际"这条传递曲线量出来。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid

# 与建图链同一套 QoS：LIO/真值/scan 都是 sensor 类（BEST_EFFORT）
BE = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
MAP_Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

PAIRS = (('odom', 'base_link'), ('map', 'odom'), ('map', 'base_link'))


# ---------------------------------------------------------------- 数学小工具
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ---------------------------------------------------------------- 默认分段计划
def default_spec():
    """默认路线：一条能"分开四种候选原因"的路线。

    几何全部来自先验图 `map/RMUC2026.pgm` 的**出生点连通可行驶域**
    （`coverage_route.py` 的 clearance=0.35 m 口径，离线量过，见 docs/lio_drift_diagnosis.md §2）：
      · 大厅：y∈[-3.0,+2.25] 时 x 从西边界(-5.9~-6.6)一直通到 +3.3 ⇒ 一条 8.5 m 的东西直线；
      · 东北是 1.4 m 宽的南北通道（x∈[-0.1,1.3]，y 到 +6.75）；
      · 西南是一条**斜向窄沟**（两侧 0.25~0.45 m 竖沿），y 从 -3.75 到 -6.8、中心线 x 从 -6.0 斜到 -8.1，
        沟底就是 23° 坡的坡脚 —— 这是"平行墙几何退化 + 靠近坡道"的那一段。
    段命名 `T*` = 过渡/换向（本身也是要量的对象：180° 原地转到底带来多少误差）。
    """
    return {
        'frame': 'map (spawn-relative)',
        'note': 'RMUC2026 / 出生点相对系；所有坐标单位 m',
        # ⚠️ 段序是**实验设计的一部分**，不是随便排的（首跑踩过）：
        #   `C`（快转 ±1.5 rad/s）与 `S`（角速度扫频，最高 ~1.9 rad/s）会**不可逆地把 LIO 带坏**
        #   （首跑实测：S 段 25 仿真秒的强转之后，LIO 偏航差 0.81 rad=46°，位置差 0.44 m，
        #   而车一步没平移；此后位置误差按 0.78 m/m 增长）。
        #   ⇒ 破坏性段必须放**最后**，否则前面所有"干净段"的结论都作废。
        #   顺序 = 直线(a) → 原路返回(d) → 慢转(b) → 快转(c) → 窄沟/坡道口(e) → 扫频(S)。
        'legs': [
            # ---- 就位（不算进主表）：只有 0.6 rad/s 量级的温和对齐，不污染 LIO
            {'kind': 'goto', 'label': 'prep1', 'wps': [[-5.5, 1.5]], 'speed': 0.30,
             'group': 'prep', 'desc': '就位到西端 (-5.5, 1.5)'},
            {'kind': 'spin', 'label': 'Tp', 'rate': 0.60, 'target_rad': None,
             'align_yaw': 0.0, 'group': 'align', 'desc': '对齐到东向'},
            # ---- (a) 直线 8.5 m @0.30 m/s ≈ 28 仿真秒
            {'kind': 'goto', 'label': 'A', 'wps': [[3.0, 1.5]], 'speed': 0.30,
             'group': 'straight', 'desc': '(a) 宽区直线向东 8.5 m @0.30 m/s'},
            {'kind': 'spin', 'label': 'Ta', 'rate': 0.60, 'target_rad': math.pi,
             'group': 'turn180', 'desc': '180° 原地转（换向，实测 ~0.32 rad/s）'},
            # ---- (d) 原路返回 8.5 m（与 A 同一条线 ⇒ 回环机会）
            {'kind': 'goto', 'label': 'D', 'wps': [[-5.5, 1.5]], 'speed': 0.30,
             'group': 'return', 'desc': '(d) 沿 A 的同一条线原路返回 8.5 m（回环机会）'},
            {'kind': 'spin', 'label': 'Td', 'rate': 0.60, 'target_rad': math.pi,
             'group': 'turn180', 'desc': '180° 原地转（换向）'},
            {'kind': 'goto', 'label': 'F', 'wps': [[0.5, 1.5]], 'speed': 0.30,
             'group': 'straight', 'desc': '直线向东 6.0 m（慢转之前）'},
            # ---- (b) 慢转：期望 0.35 rad/s、每 5 仿真秒换向，累计 5.25 rad
            {'kind': 'spin_alt', 'label': 'B', 'rate': 0.35, 'period': 5.0,
             'target_rad': 5.25, 'group': 'slow_turn',
             'desc': '(b) 慢转 ±0.35 rad/s（每 5 仿真秒换向，累计 5.25 rad）'},
            {'kind': 'spin', 'label': 'Tg', 'rate': 0.60, 'target_rad': None,
             'align_yaw': math.pi, 'group': 'align', 'desc': '对齐到西向'},
            {'kind': 'goto', 'label': 'G', 'wps': [[-5.5, 1.5]], 'speed': 0.30,
             'group': 'straight', 'desc': '直线向西 6.0 m（慢转之后，看恢复）'},
            # ---- (c) 快转：期望 1.5 rad/s、每 1.5 仿真秒换向，累计 22.5 rad
            #      ⚠️ 这一段会不可逆地带坏 LIO ⇒ 放在所有"干净段"之后
            {'kind': 'spin', 'label': 'T0', 'rate': 0.60, 'target_rad': None,
             'align_yaw': 0.0, 'group': 'align', 'desc': '对齐到东向'},
            {'kind': 'spin_alt', 'label': 'C', 'rate': 1.50, 'period': 1.5,
             'target_rad': 22.5, 'group': 'fast_turn',
             'desc': '(c) 快转 ±1.5 rad/s（每 1.5 仿真秒换向，累计 22.5 rad；'
                     '对应 docs/mapping_small_point_lio.md §5.1 run1 的已知坏工况）'},
            {'kind': 'spin', 'label': 'T1', 'rate': 0.60, 'target_rad': None,
             'align_yaw': 0.0, 'group': 'align', 'desc': '快转后对齐到东向'},
            {'kind': 'goto', 'label': 'H', 'wps': [[-1.0, 1.5]], 'speed': 0.30,
             'group': 'straight', 'desc': '直线向东 4.5 m（快转之后，看恢复/不恢复）'},
            # ---- (e) 窄沟（两侧 0.25~0.45 m 竖沿、宽 ~1.4 m）下行到 23° 坡坡脚
            {'kind': 'spin', 'label': 'T2', 'rate': 0.60, 'target_rad': None,
             'align_yaw': -2.554, 'group': 'align', 'desc': '对齐到沟口方位'},
            {'kind': 'goto', 'label': 'E',
             'wps': [[-5.3, -1.0], [-5.9, -3.3], [-6.5, -4.5], [-7.0, -5.25],
                     [-7.55, -6.0], [-8.2, -6.8]],
             'speed': 0.22, 'lookahead': 0.45, 'stuck_reverse': True,
             'group': 'narrow', 'desc': '(e) 斜向窄沟下行到 23° 坡坡脚'},
            # ---- (e2) 走廊试探（已知会被卡住，带自动退出）
            {'kind': 'goto', 'label': 'E2',
             'wps': [[-8.6, -7.4], [-9.4, -7.8], [-10.0, -7.9]],
             'speed': 0.16, 'lookahead': 0.40, 'stuck_reverse': True,
             'group': 'ramp_probe',
             'desc': '(e2) 23° 坡 + 1.05 m 走廊试探（已知扰动段；卡住就退出并记录）'},
            # ---- 角速度指令→实际 传递曲线（**放最后**：它是破坏性的）
            {'kind': 'spin_sweep', 'label': 'S', 'rates': [0.3, 0.6, 1.2, 2.4, 4.8],
             'dur': 5.0, 'group': 'sweep',
             'desc': '角速度指令→实际 传递曲线（5 档 × 5 仿真秒；破坏性，故放最后）'},
        ],
    }


# ---------------------------------------------------------------- 折线跟随
class Follow:
    """折线纯追踪（简化版：段内投影 + 单调不回退）。

    与 coverage_drive.PathFollower 的差别：这里路线**不自交、不折返**（每条腿 1~6 个点、
    形状简单），所以不需要那套"航向消歧 / 全局最小距离会瞬移"的复杂防御；
    但仍然保留"单调不回退"（投影噪声会让索引在段边界来回跳）。
    """

    def __init__(self, pts, lookahead, reach_tol=0.25):
        self.pts = [(float(a), float(b)) for a, b in pts]
        self.look = float(lookahead)
        self.reach_tol = float(reach_tol)
        self.seg = [0.0]
        for k in range(1, len(self.pts)):
            self.seg.append(self.seg[-1] + math.hypot(self.pts[k][0] - self.pts[k - 1][0],
                                                      self.pts[k][1] - self.pts[k - 1][1]))
        self.total = self.seg[-1]
        self.i = 0
        self.s_prev = 0.0
        # ⚠️ 单航点腿（total=0）必须单独处理：`s >= total - 0.05` 会**一开始就成立**
        #    ⇒ 车一步都不走就"到达"。默认路线里 prep1/A/D/F/G 全是单航点腿，踩了就是废跑。
        self.single = self.total < 1e-6

    def project(self, x, y):
        lo, hi = max(0, self.i - 1), min(len(self.pts) - 2, self.i + 1)
        best = (1e18, self.s_prev)
        for k in range(lo, hi + 1):
            ax, ay = self.pts[k]
            bx, by = self.pts[k + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d < best[0]:
                best = (d, self.seg[k] + t * math.sqrt(L2))
        s = max(best[1], self.s_prev)
        self.s_prev = s
        return s, best[0]

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

    def advance(self, x, y):
        if self.single:
            cx, cy = self.pts[0]
            d = math.hypot(cx - x, cy - y)
            return cx, cy, 0.0, d, (d < self.reach_tol)
        s, d = self.project(x, y)
        while self.i < len(self.pts) - 1 and s > self.seg[self.i]:
            self.i += 1
        cx, cy = self.at(s + self.look)
        return cx, cy, s, d, (s >= self.total - 0.05)


# --------------------------------------------------------------------- 节点
class DriveNode(Node):
    def __init__(self, args):
        super().__init__('segment_drive')
        self.args = args
        self.lock = threading.Lock()
        self.grp = ReentrantCallbackGroup()
        self.sim_now = None
        self.wall_of_sim = None
        self.lio = None            # (sim_t, x, y, z, yaw, vx, vy, vz, wz)
        self.gt = None             # (sim_t, x, y, z, yaw)
        self.tf = {}               # (parent,child) -> (sim_t, x, y, z, yaw)
        self.scan = None           # (sim_t, n, nfin, fwd_min, min_r)
        self.map_known_m2 = None
        self.cmd = (0.0, 0.0)
        self.clock_hist = []       # 最近 400 条 (sim, wall)
        self.n = {'odom': 0, 'gt': 0, 'scan': 0, 'tf': 0, 'clock': 0, 'map': 0}
        self.stamps = {'lio': None, 'gt': None, 'scan': None, 'tf': {}, 'map': None}

        self.create_subscription(Odometry, '/odom', self.on_lio, BE, callback_group=self.grp)
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_gt, BE,
                                 callback_group=self.grp)
        self.create_subscription(Clock, '/clock', self.on_clock, BE, callback_group=self.grp)
        self.create_subscription(LaserScan, '/scan', self.on_scan, BE, callback_group=self.grp)
        self.create_subscription(TFMessage, '/tf', self.on_tf, BE, callback_group=self.grp)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_Q,
                                 callback_group=self.grp)
        self.pub = self.create_publisher(Twist, '/cmd_vel_chassis', 10)
        # 指令发布定时器：**独立于控制循环**，保证"停发就保持最后速度"这个坑不会踩
        self.create_timer(1.0 / max(1.0, args.cmd_hz), self._tick_cmd, callback_group=self.grp)

    # ---- 回调：只做 O(1) 赋值（不做 lookup / 不写文件）
    def on_lio(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        with self.lock:
            self.lio = (t, p.x, p.y, p.z, yaw_of(q),
                        tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z)
            self.stamps['lio'] = time.time()
            self.n['odom'] += 1

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.gt = (t, p.x, p.y, p.z, yaw_of(q))
            self.stamps['gt'] = time.time()
            self.n['gt'] += 1

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        w = time.time()
        with self.lock:
            self.sim_now = t
            self.wall_of_sim = w
            self.clock_hist.append((t, w))
            if len(self.clock_hist) > 400:
                del self.clock_hist[:-400]
            self.n['clock'] += 1

    def on_scan(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rng = m.ranges
        n = len(rng)
        rmin, rmax = m.range_min, m.range_max
        nfin = 0
        best = float('inf')
        # 前向扇区 ±20°（用 angle_min/increment 算索引，不假设 0 在中间）
        lo = m.angle_min - 0.3490658503988659      # -20°
        hi = m.angle_min + 0.3490658503988659
        fwd = float('inf')
        for k in range(n):
            r = rng[k]
            if not (r == r) or r <= 0.0 or r < rmin or r > rmax:
                continue
            nfin += 1
            if r < best:
                best = r
            a = m.angle_min + k * m.angle_increment
            if lo <= a <= hi and r < fwd:
                fwd = r
        with self.lock:
            self.scan = (t, n, nfin, (None if fwd == float('inf') else fwd),
                         (None if best == float('inf') else best))
            self.stamps['scan'] = time.time()
            self.n['scan'] += 1

    def on_tf(self, m):
        with self.lock:
            for tr in m.transforms:
                key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
                if key in PAIRS:
                    t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                    old = self.tf.get(key)
                    if old is None or t >= old[0]:
                        p, q = tr.transform.translation, tr.transform.rotation
                        self.tf[key] = (t, p.x, p.y, p.z, yaw_of(q))
                        self.stamps['tf'][key] = time.time()
            self.n['tf'] += 1

    def on_map(self, m):
        with self.lock:
            self.map_known_m2 = float((m.info.width * m.info.height) * 0.0 +
                                      m.info.resolution ** 2 *
                                      sum(1 for v in m.data if v >= 0))
            self.stamps['map'] = time.time()
            self.n['map'] += 1

    # ---- 指令
    def _tick_cmd(self):
        t = Twist()
        with self.lock:
            t.linear.x, t.angular.z = self.cmd
        self.pub.publish(t)

    def set_cmd(self, v, w):
        with self.lock:
            self.cmd = (float(v), float(w))

    def stop(self, n=20):
        self.set_cmd(0.0, 0.0)
        for _ in range(n):
            self.pub.publish(Twist())
            time.sleep(0.05)

    # ---- 快照
    def snap(self):
        with self.lock:
            return dict(lio=self.lio, gt=self.gt, tf=dict(self.tf), scan=self.scan,
                        sim=self.sim_now, wall=self.wall_of_sim, cmd=self.cmd,
                        n=dict(self.n), stamps=dict(self.stamps),
                        map_known_m2=self.map_known_m2)

    def rtf(self, win_s=3.0):
        with self.lock:
            h = list(self.clock_hist)
        if len(h) < 3:
            return None
        t1 = h[-1][0]
        sel = [p for p in h if t1 - p[0] <= win_s]
        if len(sel) < 3:
            sel = h[-3:]
        ds = sel[-1][0] - sel[0][0]
        dw = sel[-1][1] - sel[0][1]
        return round(ds / dw, 3) if dw > 1e-9 else None


# --------------------------------------------------------------------- 采样器
class Sampler:
    """50 Hz 时间轴采样：把"最新值"拍成一行 CSV。

    每一行都带 `*_age_sim` / `*_age_wall`（这条数据比"现在"旧多少）——
    这是对"采样饥饿"的**结构化防御**：真被饿死了，CSV 里 age 会涨，
    分析脚本会把 age 超阈值的行标出来，而不是拿旧值当新值下结论。
    """
    COLS = ['wall_t', 'sim_t', 'rtf',
            'lio_t', 'lio_age_sim', 'lio_age_wall', 'lio_x', 'lio_y', 'lio_z', 'lio_yaw',
            'lio_vx', 'lio_vy', 'lio_vz', 'lio_wz',
            'gt_t', 'gt_age_sim', 'gt_x', 'gt_y', 'gt_z', 'gt_yaw',
            'tfo_t', 'tfo_age_sim', 'tfo_x', 'tfo_y', 'tfo_z', 'tfo_yaw',
            'tfm_t', 'tfm_age_sim', 'tfm_x', 'tfm_y', 'tfm_z', 'tfm_yaw',
            'fus_t', 'fus_x', 'fus_y', 'fus_z', 'fus_yaw',
            'tfb_t', 'tfb_age_sim', 'tfb_x', 'tfb_y', 'tfb_z', 'tfb_yaw',
            'cmd_v', 'cmd_w',
            'scan_t', 'scan_age_sim', 'scan_n', 'scan_nfin', 'scan_fwd_min', 'scan_min',
            'map_known_m2',
            'n_odom', 'n_gt', 'n_scan', 'n_tf', 'n_clock', 'n_map']

    def __init__(self, node, path, hz):
        self.node = node
        self.path = path
        self.hz = hz
        self.rows = []
        self.buf = []
        self.t0 = time.time()
        self.sim0 = None
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
        self.f = open(path, 'w')
        self.f.write(','.join(self.COLS) + '\n')
        self.timer = node.create_timer(1.0 / hz, self.tick, callback_group=node.grp)

    def _age(self, t, sim_now):
        if t is None or sim_now is None:
            return None
        return round(sim_now - t, 4)

    def tick(self):
        s = self.node.snap()
        sim = s['sim']
        if self.sim0 is None and sim is not None:
            self.sim0 = sim
        lio, gt, tf, scan = s['lio'], s['gt'], s['tf'], s['scan']
        tfo = tf.get(('odom', 'base_link'))
        tfm = tf.get(('map', 'odom'))
        tfb = tf.get(('map', 'base_link'))
        # 手动复合 map→base_link = (map→odom) ∘ (odom→base_link)
        # （不依赖 tf2 buffer 做时间旅行：两条边都是"各自最新"，时间戳一起报出来，
        #   分析脚本可以按 |tfm_t - tfo_t| 判断这一行能不能用）
        fus = None
        if tfm is not None and tfo is not None:
            c, sn = math.cos(tfm[4]), math.sin(tfm[4])
            fus = (min(tfm[0], tfo[0]),
                   tfm[1] + c * tfo[1] - sn * tfo[2],
                   tfm[2] + sn * tfo[1] + c * tfo[2],
                   tfm[3] + tfo[3],
                   wrap(tfm[4] + tfo[4]))
        row = {
            'wall_t': round(time.time() - self.t0, 4),
            'sim_t': sim,
            'rtf': self.node.rtf(),
            'lio_t': lio[0] if lio else None,
            'lio_age_sim': self._age(lio[0] if lio else None, sim),
            'lio_age_wall': (round(time.time() - s['stamps']['lio'], 4)
                             if s['stamps'].get('lio') else None),
            'lio_x': lio[1] if lio else None, 'lio_y': lio[2] if lio else None,
            'lio_z': lio[3] if lio else None, 'lio_yaw': lio[4] if lio else None,
            'lio_vx': lio[5] if lio else None, 'lio_vy': lio[6] if lio else None,
            'lio_vz': lio[7] if lio else None, 'lio_wz': lio[8] if lio else None,
            'gt_t': gt[0] if gt else None,
            'gt_age_sim': self._age(gt[0] if gt else None, sim),
            'gt_x': gt[1] if gt else None, 'gt_y': gt[2] if gt else None,
            'gt_z': gt[3] if gt else None, 'gt_yaw': gt[4] if gt else None,
            'cmd_v': s['cmd'][0], 'cmd_w': s['cmd'][1],
            'scan_t': scan[0] if scan else None,
            'scan_age_sim': self._age(scan[0] if scan else None, sim),
            'scan_n': scan[1] if scan else None, 'scan_nfin': scan[2] if scan else None,
            'scan_fwd_min': scan[3] if scan else None, 'scan_min': scan[4] if scan else None,
            'map_known_m2': s['map_known_m2'],
        }
        for pre, v in (('tfo', tfo), ('tfm', tfm), ('tfb', tfb)):
            row[pre + '_t'] = v[0] if v else None
            row[pre + '_age_sim'] = self._age(v[0] if v else None, sim)
            row[pre + '_x'] = v[1] if v else None
            row[pre + '_y'] = v[2] if v else None
            row[pre + '_z'] = v[3] if v else None
            row[pre + '_yaw'] = v[4] if v else None
        if fus:
            row.update({'fus_t': fus[0], 'fus_x': fus[1], 'fus_y': fus[2],
                        'fus_z': fus[3], 'fus_yaw': fus[4]})
        for k, v in s['n'].items():
            row['n_' + k] = v
        self.buf.append(row)
        if len(self.buf) >= int(self.hz):       # 每 ~1 s 落一次盘
            self.flush()

    def flush(self):
        if not self.buf:
            return
        out = []
        for r in self.buf:
            out.append(','.join('' if r.get(c) is None else
                                (('%.4f' % r[c]) if isinstance(r[c], float) else str(r[c]))
                                for c in self.COLS))
        self.f.write('\n'.join(out) + '\n')
        self.f.flush()
        self.rows.extend(self.buf)
        self.buf = []

    def close(self):
        self.flush()
        try:
            self.f.close()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------- 主流程
def run(args):
    spec = json.load(open(args.spec)) if args.spec else default_spec()
    legs = spec['legs']
    if args.only:
        want = set(x.strip() for x in args.only.split(',') if x.strip())
        legs = [l for l in legs if l['label'] in want]
    if args.dry_run:
        print('[dry] frame=%s  legs=%d' % (spec.get('frame'), len(legs)))
        tot = 0.0
        cur = [0.0, 0.0]        # 出生点 (0,0)，map 系
        for l in legs:
            if l['kind'] == 'goto':
                pts = [list(cur)] + [list(p) for p in l['wps']]
                d = sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                        for k in range(len(pts) - 1))
                tot += d
                cur = list(l['wps'][-1])
                print('  %-6s goto   %5.2f m  v=%.2f  → (%.2f, %.2f)  %s'
                      % (l['label'], d, l['speed'], cur[0], cur[1], l.get('desc', '')))
            elif l['kind'] == 'spin_sweep':
                print('  %-6s %-10s rates=%s ×%.1f 仿真秒  %s'
                      % (l['label'], l['kind'], l.get('rates'), l.get('dur', 5.0),
                         l.get('desc', '')))
            else:
                print('  %-6s %-8s rate=%.2f target=%s  %s'
                      % (l['label'], l['kind'], l['rate'], l.get('target_rad'),
                         l.get('desc', '')))
        print('[dry] 直线总长 %.1f m；按 0.30 m/s ≈ %.0f s + 原地转' % (tot, tot / 0.30))
        return 0

    rclpy.init()
    node = DriveNode(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    report = {'args': {k: v for k, v in vars(args).items()}, 'spec': spec,
              'events': [], 'warnings': [], 'link': {}}
    ev = report['events'].append
    warn = report['warnings'].append

    def now():
        s = node.snap()
        return s['sim'], time.time()

    sampler = None
    ok = 1
    try:
        # ---- 等链路
        t_wait0 = time.time()
        last = 0.0
        while time.time() - t_wait0 < args.timeout:
            s = node.snap()
            if s['lio'] and s['gt'] and s['scan'] and ('odom', 'base_link') in s['tf']:
                break
            if time.time() - last > 10:
                last = time.time()
                print('[seg] 等链路 lio=%s gt=%s scan=%s tf(odom→base_link)=%s map→odom=%s'
                      % (bool(s['lio']), bool(s['gt']), bool(s['scan']),
                         ('odom', 'base_link') in s['tf'], ('map', 'odom') in s['tf']))
            time.sleep(0.5)
        s = node.snap()
        report['link'] = {
            'lio': bool(s['lio']), 'gt': bool(s['gt']), 'scan': bool(s['scan']),
            'tf_odom_base': ('odom', 'base_link') in s['tf'],
            'tf_map_odom': ('map', 'odom') in s['tf'],
            'waited_s': round(time.time() - t_wait0, 1),
        }
        if not s['lio']:
            warn('没有 /odom：LIO 没出数')
            raise SystemExit(1)
        if not s['gt']:
            warn('没有 /odom_ground_truth：没有真值，无法算 ATE')
            raise SystemExit(1)
        if not s['scan']:
            warn('没有 /scan：无法做束数/最近回波诊断（继续跑）')

        # 采样器从"链路就绪"就开始记，静止窗口也在里面（对齐要用）
        sampler = Sampler(node, args.out_csv, args.sample_hz)
        print('[seg] 采样 → %s @ %.0f Hz' % (args.out_csv, args.sample_hz))
        ev({'event': 'link_ready', 'wall': time.time(), 'sim': node.snap()['sim']})

        # ---- 静止窗口（对齐基准 + 让 LIO 收敛）
        print('[seg] 静止 %.1f s（LIO 收敛 + 对齐基准）' % args.warmup)
        t0 = time.time()
        while time.time() - t0 < args.warmup:
            node.set_cmd(0.0, 0.0)
            time.sleep(0.1)
        node.stop(10)
        sim_w, wall_w = now()
        ev({'event': 'warmup_end', 'sim': sim_w, 'wall': wall_w})
        s = node.snap()
        # ★ 脏数据护栏：出生点必须落在期望的 world 位姿上。
        #   动机（实测踩过）：若上一轮的 gzserver 没死干净，新 launch 的 gzserver 会
        #   `process has died (exit code 255)`，`spawn_entity` 报 `Entity [robot] already exists`
        #   并直接退出 —— 于是**量到的是上一轮那台车**（它会停在上一轮路线中断的地方，
        #   本轮首帧真值就是 (11.14, 4.71, yaw=3.05) 而不是 (10.925, 2.525, 0)）。
        #   这种"整跑都建在别人的车上"的脏数据必须当场拦掉，不能进报告。
        if s['gt'] is not None and args.expect_spawn:
            ex = args.expect_spawn
            dx = s['gt'][1] - ex[0]
            dy = s['gt'][2] - ex[1]
            dyaw = wrap(s['gt'][4] - ex[2])
            report['spawn_check'] = {'expected': ex[:3], 'measured': [s['gt'][1], s['gt'][2],
                                                                    s['gt'][4]],
                                     'dxy_m': round(math.hypot(dx, dy), 4),
                                     'dyaw_rad': round(dyaw, 4), 'tol_m': ex[3]}
            if math.hypot(dx, dy) > ex[3] or abs(dyaw) > 0.35:
                msg = ('出生点与期望差 %.3f m / %.1f°（期望 (%.3f, %.3f, %.3f)，实测 '
                       '(%.3f, %.3f, %.3f)）—— 极可能是上一轮的 gzserver 没死干净、'
                       'spawn_entity 撞上 `Entity [robot] already exists`，'
                       '本跑量到的是上一台车。已中止。'
                       % (math.hypot(dx, dy), math.degrees(dyaw), ex[0], ex[1], ex[2],
                          s['gt'][1], s['gt'][2], s['gt'][4]))
                warn(msg)
                print('[seg] ❌ ' + msg)
                raise SystemExit(1)
            print('[seg] ✅ 出生点核对通过：Δxy=%.3f m Δyaw=%.2f°'
                  % (math.hypot(dx, dy), math.degrees(dyaw)))
        report['warmup'] = {'sim': sim_w, 'wall': wall_w,
                            'lio': s['lio'], 'gt': s['gt'],
                            'tf_map_odom': s['tf'].get(('map', 'odom')),
                            'tf_odom_base': s['tf'].get(('odom', 'base_link')),
                            'rtf': node.rtf(), 'scan': s['scan']}
        print('[seg] 起点 lio=%s' % (s['lio'],))
        print('[seg] 起点 gt =%s' % (s['gt'],))

        # ---- 逐腿执行
        gt_origin = [None]
        if args.pose_source == 'gt' and s['gt'] is not None:
            gt_origin[0] = (s['gt'][1], s['gt'][2], s['gt'][4])
            print('[seg] GT→spawn 对齐 origin=(%.4f, %.4f) yaw=%.4f' % gt_origin[0])

        def pose():
            ss = node.snap()
            if args.pose_source == 'gt':
                if ss['gt'] is None or gt_origin[0] is None:
                    return None
                ox, oy, oyaw = gt_origin[0]
                dx, dy = ss['gt'][1] - ox, ss['gt'][2] - oy
                c, sn = math.cos(-oyaw), math.sin(-oyaw)
                return (ss['gt'][0], c * dx - sn * dy, sn * dx + c * dy, wrap(ss['gt'][4] - oyaw))
            if ss['lio'] is None:
                return None
            return (ss['lio'][0], ss['lio'][1], ss['lio'][2], ss['lio'][4])

        t_drive0 = time.time()
        for leg in legs:
            if time.time() - t_drive0 > args.timeout:
                warn('整体超时，剩余腿未执行')
                break
            label, kind = leg['label'], leg['kind']
            ev({'event': 'seg_start', 'label': label, 'kind': kind,
                'sim': node.snap()['sim'], 'wall': time.time(),
                'cmd_w': node.snap()['cmd'][1]})
            t_seg0 = time.time()
            print('[seg] ▶ %-6s %-9s %s' % (label, kind, leg.get('desc', '')))

            if kind in ('spin', 'spin_alt', 'spin_sweep'):
                # ================= 旋转类：**对角速度闭环** =================
                # ⚠️ 为什么不能"发多少就以为转多少"：本仓仿真底盘对 yaw 的实际权威**远低于指令**
                #    （首跑实测：cmd_w=0.6 就地 → 实际 0.0214 rad/s，比例 1/28；cmd_w=0.9 且
                #     v=0.09 m/s → 实际 0.098 rad/s，比例 1/9.2。LIO 与真值**一致**，
                #     所以不是"真值在骗人"，是底盘真的没转那么快）。
                #    ⇒ 一律用 GT 偏航率做 P 闭环（cmd = target + kp*(target - measured)），
                #      指令上限 --spin-cmd-cap（默认 6.0，对应实车小陀螺 spin_speed=5.0 的量级）。
                #    这样报出来的"慢转 0.35 / 快转 1.5 rad/s"才是**实际发生的角速度**。
                def rate_of(hist, now_sim, win=0.6):
                    """从 (sim_t, yaw) 历史里量最近 win 仿真秒的平均偏航率。"""
                    if len(hist) < 3:
                        return None
                    t1 = hist[-1][0]
                    sel = [h for h in hist if t1 - h[0] <= win]
                    if len(sel) < 3:
                        return None
                    dt = sel[-1][0] - sel[0][0]
                    if dt <= 1e-6:
                        return None
                    dy = 0.0
                    for k in range(1, len(sel)):
                        dy += wrap(sel[k][1] - sel[k - 1][1])
                    return dy / dt

                rate = float(leg.get('rate', 0.6))   # align 分支用
                hist = []
                acc = 0.0
                prev = None
                sim_prev = None
                cmd_w_sum = 0.0
                sim_sum = 0.0
                tgt = leg.get('target_rad')
                period = float(leg.get('period', 0.0))
                sign = 1.0
                sim_flip = None
                sweep = list(leg.get('rates') or [])
                sweep_dur = float(leg.get('dur', 5.0))
                sweep_i = 0
                sweep_t0 = None
                sweep_cmd_sum = 0.0
                sweep_w0 = time.time()
                _last_w = time.time()
                node.set_cmd(0.0, 0.0)
                while True:
                    p = pose()
                    if p is None:
                        time.sleep(0.03)
                        continue
                    tnow, x, y, yaw = p
                    if prev is not None and sim_prev is not None and tnow > sim_prev:
                        acc += abs(wrap(yaw - prev))
                        sim_sum += (tnow - sim_prev)
                    prev, sim_prev = yaw, tnow
                    hist.append((tnow, yaw))
                    if len(hist) > 40:
                        del hist[:-40]

                    if kind == 'spin_sweep':
                        if sweep_t0 is None:
                            sweep_t0 = tnow
                        if sweep_i >= len(sweep):
                            break
                        if tnow - sweep_t0 >= sweep_dur:
                            r_meas = rate_of(hist, tnow)
                            _dtw = max(time.time() - sweep_w0, 1e-6)
                            ev({'event': 'sweep_step', 'label': label,
                                'rate_target': sweep[sweep_i],
                                'rate_meas': None if r_meas is None else round(r_meas, 4),
                                'cmd_w_mean': round(sweep_cmd_sum / _dtw, 3),
                                'sim': round(tnow, 2)})
                            sweep_cmd_sum = 0.0
                            sweep_w0 = time.time()
                            print('[seg]   · 指令 |ω|=%.2f → 实测 %s rad/s（同向比例 %s）'
                                  % (sweep[sweep_i],
                                     'n/a' if r_meas is None else '%+.4f' % r_meas,
                                     'n/a' if not r_meas else
                                     '%.2f' % (abs(r_meas) / sweep[sweep_i])))
                            sweep_i += 1
                            sweep_t0 = tnow
                            sign = -sign
                        if sweep_i >= len(sweep):
                            break
                        rt = sweep[sweep_i]
                        # 每档换一次方向，避免"同向一直转"把线缆/边界拉满（也顺便测双向对称性）
                        sgn = 1.0 if (sweep_i % 2 == 0) else -1.0
                        rm = rate_of(hist, tnow)
                        cmd = sgn * rt + (0.0 if rm is None else args.spin_kp * (sgn * rt - rm))
                        cmd = max(-args.spin_cmd_cap, min(args.spin_cmd_cap, cmd))
                        node.set_cmd(0.0, cmd)
                        _w = time.time()
                        cmd_w_sum += abs(cmd) * max(0.0, _w - _last_w)
                        sweep_cmd_sum += abs(cmd) * max(0.0, _w - _last_w)
                        _last_w = _w
                        if time.time() - t_seg0 > args.leg_max_wall:
                            warn('%s 超过 --leg-max-wall，提前收' % label)
                            break
                        time.sleep(0.03)
                        continue

                    if kind == 'spin' and leg.get('align_yaw') is not None:
                        e = wrap(leg['align_yaw'] - yaw)
                        if abs(e) < 0.05:
                            break
                        # ⚠️ 不能写成 cmd = clamp(1.5*|e|, ±rate)：本仓底盘在小角速度指令上
                        #    几乎是死区（实测 cmd=0.6 ⇒ 0.02 rad/s，见 --help 里的传递曲线），
                        #    于是"对齐"会 60 s 都转不到位（首跑实测 0.83 rad / 46 仿真秒）。
                        #    正确做法：**外环** 角度误差 → 期望角速度，**内环** 期望角速度 → 指令（P 闭环）。
                        rt = max(-1.5, min(1.5, args.align_kp * e))
                        rm2 = rate_of(hist, tnow)
                        cmd = rt + (0.0 if rm2 is None else args.spin_kp * (rt - rm2))
                        cmd = max(-args.spin_cmd_cap, min(args.spin_cmd_cap, cmd))
                    else:
                        rt = float(leg['rate'])
                        rm = rate_of(hist, tnow)
                        cmd = sign * rt + (0.0 if rm is None else args.spin_kp * (sign * rt - rm))
                        cmd = max(-args.spin_cmd_cap, min(args.spin_cmd_cap, cmd))
                        if kind == 'spin_alt' and period > 0:
                            if sim_flip is None:
                                sim_flip = tnow
                            elif tnow - sim_flip >= period:
                                sign = -sign
                                sim_flip = tnow
                    node.set_cmd(0.0, cmd)
                    _w = time.time()
                    cmd_w_sum += abs(cmd) * max(0.0, _w - _last_w)
                    _last_w = _w
                    if tgt is not None and acc >= tgt:
                        break
                    if time.time() - t_seg0 > args.leg_max_wall:
                        warn('%s 超过 --leg-max-wall，提前收' % label)
                        break
                    time.sleep(0.03)
                node.stop(3)
                rm_all = (acc / sim_sum) if sim_sum > 1e-6 else None
                ev({'event': 'seg_end', 'label': label, 'kind': kind,
                    'sim': node.snap()['sim'], 'wall': time.time(),
                    'acc_rad': round(acc, 3), 'sim_s': round(sim_sum, 2),
                    'rate_achieved': None if rm_all is None else round(rm_all, 4),
                    'rate_target': leg.get('rate'),
                    # 指令均值：按墙钟加权后再乘 RTF，折算到"每仿真秒"，与 rate_achieved 同口径
                    'cmd_w_mean': round(cmd_w_sum / max(time.time() - t_seg0, 1e-6)
                                        * (sim_sum / max(time.time() - t_seg0, 1e-6)), 3),
                    'wall_s': round(time.time() - t_seg0, 2)})
                print('[seg] ■ %-6s 累计转 %.2f rad / %.1f 仿真秒 = 实测 %.3f rad/s'
                      '（指令均值 %.3f），墙钟 %.1f s'
                      % (label, acc, sim_sum, -1 if rm_all is None else rm_all,
                         cmd_w_sum / max(sim_sum, 1e-6), time.time() - t_seg0))
                continue

            # ---- goto
            fl = Follow(leg['wps'], float(leg.get('lookahead', args.lookahead)))
            speed = float(leg['speed'])
            stuck_since = time.time()
            last_move = None
            reverses = 0
            jammed = False
            n_motion = n_stuck = 0
            while True:
                p = pose()
                if p is None:
                    time.sleep(0.05)
                    continue
                _, x, y, yaw = p
                n_motion += 1
                if last_move is None or math.hypot(x - last_move[0], y - last_move[1]) > 0.03 \
                        or abs(wrap(yaw - last_move[2])) > 0.03:
                    last_move = (x, y, yaw)
                    stuck_since = time.time()
                elif time.time() - stuck_since > args.stuck_sec:
                    n_stuck += 1
                    if not leg.get('stuck_reverse'):
                        warn('%s 卡住但没开 stuck_reverse' % label)
                        break
                    ss = node.snap()
                    ev({'event': 'jam', 'label': label, 'sim': ss['sim'], 'wall': time.time(),
                        'pose': [round(x, 3), round(y, 3), round(yaw, 4)],
                        'cmd': [round(ss['cmd'][0], 3), round(ss['cmd'][1], 3)],
                        'scan_nfin': ss['scan'][2] if ss['scan'] else None,
                        'scan_fwd_min': ss['scan'][3] if ss['scan'] else None,
                        'scan_min': ss['scan'][4] if ss['scan'] else None,
                        'lio': ss['lio'], 'gt': ss['gt'],
                        'reverse_no': reverses + 1})
                    print('[seg] ⚠️ %s 卡住 @ (%.2f, %.2f) 前向最近回波=%s m 有限束=%s ⇒ 倒退'
                          % (label, x, y,
                             ss['scan'][3] if ss['scan'] else None,
                             ss['scan'][2] if ss['scan'] else None))
                    reverses += 1
                    if reverses >= args.stuck_max:
                        jammed = True
                        warn('%s 连续卡 %d 次，放弃这一段' % (label, args.stuck_max))
                        break
                    node.set_cmd(-0.12, 0.0)
                    time.sleep(1.2)
                    node.set_cmd(0.0, 0.35)
                    time.sleep(0.9)
                    node.stop(3)
                    stuck_since = time.time()
                    last_move = None
                    fl = Follow(leg['wps'], float(leg.get('lookahead', args.lookahead)))
                    continue
                cx, cy, s_arc, dproj, done = fl.advance(x, y)
                if done:
                    break
                herr = wrap(math.atan2(cy - y, cx - x) - yaw)
                # 航向 P + 限幅；直线段 |herr| 很小 ⇒ w≈0（不会把"直线"变成"扭着走"）
                w = max(-args.yaw_rate_max, min(args.yaw_rate_max, args.yaw_gain * herr))
                v = speed * max(0.25, min(1.0, 1.0 - 0.7 * min(1.0, abs(herr) / 1.0)))
                node.set_cmd(v, w)
                if time.time() - t_seg0 > args.leg_max_wall:
                    warn('%s 超过 --leg-max-wall，提前收' % label)
                    break
                time.sleep(0.05)
            node.stop(3)
            ev({'event': 'seg_end', 'label': label, 'kind': kind,
                'sim': node.snap()['sim'], 'wall': time.time(),
                'wall_s': round(time.time() - t_seg0, 2),
                'jammed': jammed, 'n_reverse': reverses,
                'n_motion_checks': n_motion, 'n_stuck': n_stuck})
            print('[seg] ■ %-6s 完成（卡住 %d 次%s），墙钟 %.1f s'
                  % (label, n_stuck, '，已放弃' if jammed else '', time.time() - t_seg0))

        sim_e, wall_e = now()
        ev({'event': 'drive_end', 'sim': sim_e, 'wall': wall_e})
        node.stop(20)
        time.sleep(2.0)
        node.stop(10)
        s = node.snap()
        report['drive'] = {'sim_s': (round(sim_e - sim_w, 1)
                                     if (sim_e is not None and sim_w is not None) else None),
                           'wall_s': round(wall_e - wall_w, 1),
                           'rtf': node.rtf(),
                           'n_msgs': s['n'],
                           'map_known_m2': s['map_known_m2']}
        report['rtf_series'] = None
        print('[seg] 行驶结束：仿真 %.1f s / 墙钟 %.1f s，RTF≈%s，消息计数=%s'
              % (report['drive']['sim_s'] or -1, report['drive']['wall_s'],
                 report['drive']['rtf'], s['n']))
        ok = 0
    finally:
        try:
            node.stop(20)
        except Exception:  # noqa: BLE001
            pass
        if sampler is not None:
            sampler.close()
            report['csv'] = os.path.abspath(args.out_csv)
            report['csv_rows'] = len(sampler.rows)
            # 采样健康度：age 超阈值的行数（"采样饥饿"的结构化判据）
            for key in ('lio', 'gt', 'scan'):
                col = {'lio': 'lio_age_sim', 'gt': 'gt_age_sim',
                       'scan': 'scan_age_sim'}[key]
                vals = [r[col] for r in sampler.rows if r.get(col) is not None]
                if vals:
                    report.setdefault('age_check', {})[key] = {
                        'n': len(vals), 'max_s': round(max(vals), 4),
                        'p99_s': round(sorted(vals)[int(0.99 * (len(vals) - 1))], 4),
                        'over_0.5s': sum(1 for v in vals if v > 0.5)}
        report['events'].append({'event': 'script_end', 'wall': time.time(),
                                 'sim': node.snap()['sim']})
        if args.out_json:
            os.makedirs(os.path.dirname(os.path.abspath(args.out_json)) or '.', exist_ok=True)
            with open(args.out_json, 'w') as f:
                json.dump(report, f, indent=1, default=str)
            print('[seg] 写出 %s' % args.out_json)
        try:
            ex.shutdown()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return ok


def main():
    ap = argparse.ArgumentParser(
        description='LIO 漂移诊断：分段脚本化路线 + 50 Hz 同步采样（MultiThreadedExecutor）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='默认分段计划见 default_spec()（--dry-run 可打印）；用 --spec 传 JSON 覆盖。')
    ap.add_argument('--spec', help='分段计划 JSON（缺省用内置 default_spec()）')
    ap.add_argument('--only', help='只跑这些 label（逗号分隔，调试用）')
    ap.add_argument('--out-csv', default='.tmp_cache/diag/segment_drive.csv',
                    help='50 Hz 时间轴 CSV 路径')
    ap.add_argument('--out-json', default='.tmp_cache/diag/segment_drive.json',
                    help='事件/元数据 JSON 路径')
    ap.add_argument('--pose-source', choices=['gt', 'lio'], default='gt',
                    help='闭环反馈源：gt=仿真真值（几何确定，默认）| lio=自己的 /odom（真实用法）')
    ap.add_argument('--warmup', type=float, default=10.0, help='起步前静止秒数（对齐基准）')
    ap.add_argument('--expect-spawn', nargs=4, type=float, default=[10.925, 2.525, 0.0, 0.35],
                    metavar=('X', 'Y', 'YAW', 'TOL'),
                    help='world 系期望出生点（默认 RMUC2026 的 (10.925, 2.525, 0)，容差 0.35 m）。'
                         '对不上就退出 1 —— 挡住"上一轮 gzserver 没死干净 ⇒ 量到上一台车"的脏数据。'
                         '传 --expect-spawn 0 0 0 999 可关掉')
    ap.add_argument('--timeout', type=float, default=600.0, help='整程墙钟上限（s）')
    ap.add_argument('--leg-max-wall', type=float, default=60.0, help='单腿墙钟上限（s）')
    ap.add_argument('--stuck-sec', type=float, default=4.0, help='多久没动算卡住（s）')
    ap.add_argument('--stuck-max', type=int, default=2, help='同一段最多倒退重试几次')
    ap.add_argument('--speed-default', type=float, default=0.30, help='（保留）默认线速度')
    ap.add_argument('--lookahead', type=float, default=0.70, help='纯追踪前视距离（m）')
    ap.add_argument('--yaw-gain', type=float, default=1.2, help='航向 P 增益')
    ap.add_argument('--yaw-rate-max', type=float, default=3.0,
                    help='航向控制角速度上限（**不是物理角速度**：仿真底盘实际只能做到指令的 ~1/9，'
                         '所以这个上限要放到 3.0 才有实际权威）')
    ap.add_argument('--spin-cmd-cap', type=float, default=6.0,
                    help='旋转类的角速度指令上限（rad/s）。对应实车「小陀螺」语义 spin_speed=5.0 的量级')
    ap.add_argument('--align-kp', type=float, default=2.0,
                    help='对齐外环增益：期望角速度 = clamp(align_kp*角度误差, ±1.5 rad/s)')
    ap.add_argument('--spin-kp', type=float, default=6.0,
                    help='角速度闭环 P 增益（cmd = target + kp*(target - measured)）')
    ap.add_argument('--cmd-hz', type=float, default=100.0, help='指令发布频率（独立定时器）')
    ap.add_argument('--sample-hz', type=float, default=50.0, help='时间轴采样频率')
    ap.add_argument('--dry-run', action='store_true', help='只打印分段计划')
    args = ap.parse_args()
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
