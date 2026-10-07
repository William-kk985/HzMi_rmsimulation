#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tilt_mount_probe.py —— 「斜装雷达：射线/帧/外参/外方位」取证探针（2026-10-09）。

与 tools/scripts/regress/robot11_mount_probe.py（别的主题的**点账本**探针）的分工：
那个探针回答"每级裁了多少点、代价图里有什么"；**本探针回答几何/帧问题**，为
docs/tilted_lidar_fidelity.md §I 服务。它只订阅，不发布任何东西（不改任何契约）。

量什么（每一项都落进 probe.json，点云另存 npz 供离线复算）：
  1. `/livox/lidar/pointcloud`（raw，frame_id=livox_frame）整帧：RANSAC 地面平面（法向/夹角/
     平面高）＋**按方位 30° 扇区的点数/仰角包络/地面点最小水平半径**（H3：一侧盲区）＋
     四象限点数。
  2. `/cloud_registered`（LIO 发，frame_id=odom）整帧：同一套平面/仰角统计（H1：odom 里地面斜不斜）。
  3. **同一朵 raw 云用 LIO 自己的 TF（odom←livox_frame）搬到 odom** 再拟合平面 —— 这是
     "LIO 自己以为的 odom 帧里的地面"，用来把"odom 帧斜了"与"发布时被多转了一次"分开。
  4. 刚体配准假设检验（raw → registered）：H_a（多了一次 T(base_link←livox_frame) 的旋转）
     vs H_b（没有旋转，只有平移）的残差对比 + 一般 Kabsch（最近邻配对）复原的旋转角/轴。
  5. `/livox/imu` 窗口统计：imu 系里的重力方向（斜装 ⇒ imu 自己的 z 不再指向天）＋
     用 TF 把它搬到 base_link / odom 后的重力方向（H1：odom 帧相对重力斜多少）。
  6. 真值 `/odom_ground_truth`（Gazebo 世界位姿）＋ TF（odom←base_link）⇒ **odom 帧相对
     真实重力的倾角**（不依赖 LIO 内部任何假设）。
  7. `/scan`：有限波束总数、逐扇区有限波束数、最近波束；并用 TF 把每条波束搬到 odom，
     统计 z 分布 ⇒ 直接量 obstacle_layer 的 min/max_obstacle_height(0/2.0) 会丢掉多少波束。
  8. `/local_costmap/costmap`：lethal/inscribed/free/unknown + 车心到最近 lethal 的距离 +
     **lethal 格的方位分布**（以车为原点、在车体系里）+ ★ 2026-10-09 新增：**车半径圆内**的
     格数/≥99 格数/free 格数（"车是不是一开始就被膨胀团包住"的判据）。
  9. ★ 2026-10-09 新增（为 docs/tilted_lidar_fidelity.md §J 的 A/B 服务）：
     · `/segmentation/ground` 每帧点数（与 obstacle 一起看"分割有没有塌"）；
     · `/scan` 的**距离带**直方图（0.05–0.1 / 0.1–0.3 / … / 4–7 m）；
     · **RTF**（记录窗内 Δ仿真钟/Δ墙钟，与 /clock 同口径）；
     · `--drive`：记录窗内注入**固定动作**（直行 10 s vx=0.30 ↔ 原地转 10 s wz=0.60 交替），
       窗口末尾报"odom 位移 − 真值位移"漂移与 yaw 差（本仓惯例，见 §H.5）；
     · `--goal-forward H`（+`--goal-wait`）：窗口末尾沿车头方向发一次 `/goal_pose`，
       报 pre/post 位姿、真值位移、`/cmd_vel` 非零条数、`/plan` 条数（"目标被接受但车不动"的判据）。
     ⚠️ `--drive` / `--goal-forward` 的注入**都在 settle 之后**（本沙箱里在 spawn+插件加载期
     注入会让 gzserver segfault，见 §H.6）；不带这两个开关时探针仍然**只订阅**。

用法：
  python3 tools/scripts/tiltmount/tilt_mount_probe.py --out <dir>/probe.json \
      --npz <dir>/clouds.npz --duration 45 --settle 25 --frames 3
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from collections import Counter, defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
import tf2_ros

try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_CUSTOM = True
except Exception:                                            # pragma: no cover
    CustomMsg = None
    HAVE_CUSTOM = False

QOS_BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)
QOS_RE = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST)

DATATYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
            5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
SECTOR_NAMES = ['%d..%d' % (a, a + 30) for a in range(-180, 180, 30)]


# --------------------------------------------------------------------------- 几何工具
def pct(a, q):
    a = np.asarray(a, dtype=np.float64)
    return float(np.percentile(a, q)) if a.size else float('nan')


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_rpy_deg(R):
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                                    # pragma: no cover
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def tf_msg_to_RT(tr):
    t = tr.transform.translation
    q = tr.transform.rotation
    return quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z], dtype=np.float64)


def pc2_to_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    buf = buf[:n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for k in ('x', 'y', 'z'):
        f = fields[k]
        dt = DATATYPE.get(f.datatype)
        if dt is None:
            return np.zeros((0, 3))
        cols.append(buf[:, f.offset:f.offset + np.dtype(dt).itemsize].copy()
                    .view(dt).reshape(-1).astype(np.float64))
    P = np.stack(cols, axis=1)
    good = np.isfinite(P).all(axis=1)
    return P[good]


def fit_plane_ransac(P, iters=300, tol=0.02, seed=0):
    """返回最大平面的 (n, d, inliers, resid_p50/p95)。平面方程 n·p + d = 0，n 取 z>0。"""
    if len(P) < 20:
        return None
    rng = np.random.default_rng(seed)
    best = (0, None, None)
    N = len(P)
    for _ in range(iters):
        idx = rng.choice(N, 3, replace=False)
        p0, p1, p2 = P[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = -float(n @ p0)
        r = np.abs(P @ n + d)
        cnt = int((r < tol).sum())
        if cnt > best[0]:
            best = (cnt, n.copy(), d)
    if best[1] is None:
        return None
    n, d = best[1], best[2]
    r = P @ n + d
    inl = np.abs(r) < tol
    # 用内点做一次最小二乘精修
    A = P[inl]
    if len(A) >= 3:
        c = A.mean(axis=0)
        u, s, vt = np.linalg.svd(A - c)
        n = vt[2]
        d = -float(n @ c)
        r = P @ n + d
        inl = np.abs(r) < tol
    if n[2] < 0:
        n, d = -n, -d
    res = np.abs(P[inl] @ n + d)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(n[2]))))))
    del r
    return {'n': [float(v) for v in n], 'd': float(d),
            'n_inliers': int(inl.sum()), 'frac_inliers': float(inl.sum()) / N,
            'ang_from_frame_z_deg': ang,
            'sensor_height_m': float(abs(d)),
            'resid_p50': pct(res, 50), 'resid_p95': pct(res, 95)}


def kabsch(A, B):
    """求 R,t 使 R·A + t ≈ B（A/B 同长、已配对）。"""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    u, s, vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    R = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    t = cb - R @ ca
    return R, t


def rot_angle_axis_deg(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.degrees(math.acos(c))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    ax = ax / n if n > 1e-9 else np.array([0.0, 0.0, 0.0])
    return ang, [float(v) for v in ax]


def sector_stats(P, plane=None, nsec=12, ground_tol=0.03):
    """按方位（x 前、y 左，REP-103）分扇区的点数/仰角包络/地面点最小水平半径。"""
    if len(P) == 0:
        return []
    r3 = np.linalg.norm(P, axis=1)
    good = r3 > 1e-6
    P, r3 = P[good], r3[good]
    az = np.degrees(np.arctan2(P[:, 1], P[:, 0]))
    elev = np.degrees(np.arcsin(np.clip(P[:, 2] / r3, -1, 1)))
    rxy = np.hypot(P[:, 0], P[:, 1])
    isg = np.zeros(len(P), dtype=bool)
    if plane is not None:
        n = np.array(plane['n'])
        d = plane['d']
        isg = np.abs(P @ n + d) < ground_tol
    out = []
    width = 360.0 / nsec
    for k in range(nsec):
        lo = -180.0 + k * width
        hi = lo + width
        m = (az >= lo) & (az < hi)
        mg = m & isg
        out.append({
            'sector_deg': [lo, hi],
            'n': int(m.sum()),
            'n_ground': int(mg.sum()),
            'elev_min_deg': pct(elev[m], 0) if m.any() else None,
            'elev_max_deg': pct(elev[m], 100) if m.any() else None,
            'elev_p05_deg': pct(elev[m], 5) if m.any() else None,
            'rxy_min_all_m': float(rxy[m].min()) if m.any() else None,
            'rxy_min_ground_m': float(rxy[mg].min()) if mg.any() else None,
        })
    return out


def ground_depression_deg(plane):
    """无帧假设的判据：地面最近环半径 r_min 与传感器离地高度 h ⇒ 最陡的下俯角。"""
    return None if plane is None else plane


def quadrants(stats):
    """front/back/left/right（扇区下标 0 = -180..-150 …）。"""
    q = {'front(|az|<=45)': [0, 0], 'left(45..135)': [0, 0],
         'back(|az|>135)': [0, 0], 'right(-135..-45)': [0, 0]}
    for i, s in enumerate(stats):
        lo = s['sector_deg'][0]
        for name, (a, b) in (('front(|az|<=45)', (-45, 45)),
                             ('left(45..135)', (45, 135)),
                             ('back(|az|>135)', (135, 180)),
                             ('right(-135..-45)', (-180, -45))):
            if a <= lo < b:
                q[name][0] += s['n']
                q[name][1] += s['n_ground']
    return {k: {'n': v[0], 'n_ground': v[1]} for k, v in q.items()}


def cloud_stats(tag, P, frame_id, stamp, plane=None, elev_src=None):
    d = {'tag': tag, 'frame_id': frame_id, 'stamp': stamp, 'n': int(len(P))}
    if len(P) == 0:
        return d
    pl = fit_plane_ransac(P) if plane is None else plane
    d['plane'] = pl
    d['sectors'] = sector_stats(P, pl)
    d['quadrants'] = quadrants(d['sectors'])
    # 仰角谱（相对本帧自己的 z；本帧 z 是不是重力方向由 plane 的 ang_from_frame_z_deg 说明）
    r3 = np.linalg.norm(P, axis=1)
    elev = np.degrees(np.arcsin(np.clip(P[:, 2] / np.maximum(r3, 1e-9), -1, 1)))
    d['elev_pct'] = {str(q): pct(elev, q) for q in (0, 1, 5, 25, 50, 75, 95, 99, 100)}
    d['n_below_horizontal_plane'] = int((P[:, 2] < -0.02).sum())
    d['frac_below_horizontal_plane'] = float((P[:, 2] < -0.02).mean())
    # 地面环几何（无帧假设）：最近地面点的水平半径 + 由它反推的最陡下俯角
    if pl is not None:
        n = np.array(pl['n'])
        d_ = pl['d']
        isg = np.abs(P @ n + d_) < 0.03
        if isg.any():
            rxy = np.hypot(P[isg, 0], P[isg, 1])
            azg = np.degrees(np.arctan2(P[isg, 1], P[isg, 0]))
            h = abs(pl['d'])
            d['ground_ring'] = {
                'h_m': h,
                'rxy_min_m': float(rxy.min()),
                'depression_max_deg': math.degrees(math.atan2(h, max(float(rxy.min()), 1e-6))),
                'rxy_p05_m': pct(rxy, 5), 'rxy_p50_m': pct(rxy, 50),
            }
            near = np.argsort(rxy)[:200]
            hist, edges = np.histogram(azg[near], bins=np.arange(-180, 181, 30))
            d['ground_ring']['nearest200_az_hist'] = {SECTOR_NAMES[i]: int(hist[i])
                                                      for i in range(len(hist))}
            d['ground_ring']['nearest200_az_p50'] = pct(azg[near], 50)
    return d


# --------------------------------------------------------------------------- 探针
class TiltProbe(Node):
    def __init__(self, args):
        super().__init__('tilt_mount_probe')
        self.args = args
        self.lock = threading.Lock()
        self.frames = Counter()
        self.pts = defaultdict(list)
        self.frame_ids = defaultdict(Counter)
        self.clouds = {}                      # 'raw'/'registered' → [ (stamp, frame_id, P) ]
        self.scan_frames = []
        self.imu = []                         # (t, ax,ay,az, gx,gy,gz)
        self.imu_frame_id = Counter()
        self.odom = defaultdict(list)         # topic → [(t, x,y,z, qx,qy,qz,qw)]
        self.clock = None
        self.clock_t0 = None
        self.costmap = {}
        self.tf = {}
        self.dumps = defaultdict(int)
        # ★ 2026-10-09：A/B 需要的额外量（--drive / --goal-forward）
        self.cmdvel_abs = []                  # /cmd_vel 的 |vx|+|wz|（nav2 侧；恢复行为也在这里）
        self.clock_marks = []                 # (wall, clock) —— 记录窗首尾 ⇒ RTF
        self.goal_pub = None
        self.vel_pub = None
        self.goal_result = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Clock, '/clock', self.on_clock, QOS_BE)
        if HAVE_CUSTOM:
            self.create_subscription(CustomMsg, '/livox/lidar', self.on_custom, QOS_BE)
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud',
                                 lambda m: self.on_cloud('raw', m), QOS_BE)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 lambda m: self.on_cloud('registered', m), QOS_BE)
        self.create_subscription(PointCloud2, '/segmentation/obstacle',
                                 lambda m: self.on_cloud('obstacle', m), QOS_BE)
        # ★ 2026-10-09：地面那一半也要（A/B 里看"分割有没有塌"；只数点数，不落点）
        self.create_subscription(PointCloud2, '/segmentation/ground',
                                 lambda m: self.on_cloud('ground', m), QOS_BE)
        # ★ 2026-10-09：nav2 侧证据（--goal-forward 时用来判"目标被接受但车不动"）
        self.create_subscription(Twist, '/cmd_vel', self.on_cmdvel, QOS_RE)
        self.create_subscription(Path, '/plan', self.on_plan, QOS_RE)
        self.create_subscription(PointCloud2, '/global_costmap/voxel_grid',
                                 lambda m: self.on_cloud('voxel_grid', m), QOS_BE)
        self.create_subscription(LaserScan, '/scan', self.on_scan, QOS_BE)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, QOS_BE)
        for t in ('/odom', '/odom_ground_truth'):
            self.create_subscription(Odometry, t,
                                     lambda m, t=t: self.on_odom(t, m), QOS_RE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: self.on_costmap('local', m), QOS_BE)

    # ---------------- 回调
    def on_clock(self, m):
        with self.lock:
            self.clock = m.clock.sec + m.clock.nanosec * 1e-9
            if self.clock_t0 is None:
                self.clock_t0 = (self.clock, time.time())

    def on_custom(self, m):
        with self.lock:
            self.frames['custom'] += 1
            self.pts['custom'].append(int(m.point_num))
            self.frame_ids['custom'][m.header.frame_id] += 1

    def on_cloud(self, tag, m):
        n = m.width * m.height
        with self.lock:
            self.frames[tag] += 1
            self.pts[tag].append(n)
            self.frame_ids[tag][m.header.frame_id] += 1
            want = self.dumps[tag] < self.args.frames
            if want:
                self.dumps[tag] += 1
        if not want:
            return
        P = pc2_to_xyz(m)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.clouds.setdefault(tag, []).append((st, m.header.frame_id, P))

    def on_scan(self, m):
        with self.lock:
            self.frames['scan'] += 1
            self.frame_ids['scan'][m.header.frame_id] += 1
            if len(self.scan_frames) < max(3, self.args.frames):
                self.scan_frames.append({
                    'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    'frame_id': m.header.frame_id,
                    'angle_min': m.angle_min, 'angle_max': m.angle_max,
                    'angle_increment': m.angle_increment,
                    'range_min': m.range_min, 'range_max': m.range_max,
                    'ranges': list(m.ranges)})

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.frames['imu'] += 1
            self.imu_frame_id[m.header.frame_id] += 1
            self.imu.append((t, m.linear_acceleration.x, m.linear_acceleration.y,
                             m.linear_acceleration.z, m.angular_velocity.x,
                             m.angular_velocity.y, m.angular_velocity.z))

    def on_odom(self, topic, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.frames[topic] += 1
            self.odom[topic].append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w,
                                     m.header.frame_id, m.child_frame_id))

    def on_costmap(self, tag, m):
        with self.lock:
            self.frames['costmap_' + tag] += 1
            data = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            self.costmap[tag] = {
                'frame_id': m.header.frame_id,
                'resolution': m.info.resolution,
                'origin': [m.info.origin.position.x, m.info.origin.position.y,
                           m.info.origin.position.z],
                'width': m.info.width, 'height': m.info.height,
                'counts': {str(v): int((data == v).sum()) for v in
                           (100, 99, 0, -1)},
                'n_lethal': int((data == 100).sum()),
                'n_inscribed': int((data == 99).sum()),
                'n_free': int((data == 0).sum()),
                'n_unknown': int(((data < 0)).sum()),
                '_data': data}

    def on_cmdvel(self, m):
        with self.lock:
            self.frames['cmd_vel'] += 1
            self.cmdvel_abs.append(abs(m.linear.x) + abs(m.angular.z))

    def on_plan(self, m):
        with self.lock:
            self.frames['plan'] += 1
            self.pts['plan'].append(len(m.poses))

    def drive_step(self, t_in_window, period=20.0, straight_s=10.0):
        """★ 2026-10-09：`--drive` 的固定动作（与 run_robot11_mount_probe.sh **逐字相同**的协议）。

        直行 10 s（vx=0.30）↔ 原地转 10 s（wz=0.60）交替，注入点 = `/cmd_vel_chassis`
        （mecanum 插件订阅的那个；`fake_vel_transform` 是事件驱动转发，不会持续发零覆盖它）。
        """
        if self.vel_pub is None:
            self.vel_pub = self.create_publisher(Twist, '/cmd_vel_chassis', 10)
        m = Twist()
        if (t_in_window % period) < straight_s:
            m.linear.x = 0.30
        else:
            m.angular.z = 0.60
        self.vel_pub.publish(m)

    def send_goal(self, x, y):
        """发一次 `/goal_pose`（map 系；只为取证，不改任何参数/配置）。"""
        if self.goal_pub is None:
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        for _ in range(5):
            self.goal_pub.publish(p)
            time.sleep(0.2)

    def robot_map_pose(self):
        """车在 map 系里的 (x, y, yaw)（拿不到就 None）。"""
        tr = self._lookup('map', 'base_link')
        if tr is None:
            return None
        R, t = tf_msg_to_RT(tr)
        return (float(t[0]), float(t[1]), math.radians(R_to_rpy_deg(R)[2]))

    # ---------------- TF
    def _lookup(self, target, source, stamp=None):
        tries = []
        if stamp is not None:
            tries.append(stamp)
            tries.append(stamp + 0.1)
            tries.append(stamp - 0.1)
        tries.append(None)
        for t in tries:
            try:
                if t is None:
                    tr = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                else:
                    tr = self.tf_buffer.lookup_transform(
                        target, source, rclpy.time.Time(seconds=int(t),
                                                        nanoseconds=int((t % 1) * 1e9)),
                        timeout=rclpy.duration.Duration(seconds=0.4))
                return tr
            except Exception:
                continue
        return None

    def snapshot_tf(self):
        pairs = [('base_link', 'livox_frame'), ('base_link', 'imu_link'),
                 ('imu_link', 'livox_frame'), ('odom', 'base_link'),
                 ('odom', 'livox_frame'), ('odom', 'imu_link'),
                 ('map', 'odom'), ('map', 'base_link')]
        for a, b in pairs:
            tr = self._lookup(a, b)
            if tr is None:
                self.tf['%s<- %s' % (a, b)] = None
                continue
            R, t = tf_msg_to_RT(tr)
            self.tf['%s<- %s' % (a, b)] = {
                'xyz': [float(v) for v in t], 'rpy_deg': R_to_rpy_deg(R),
                'quat_xyzw': [float(tr.transform.rotation.x), float(tr.transform.rotation.y),
                              float(tr.transform.rotation.z), float(tr.transform.rotation.w)],
                'R': [[float(v) for v in row] for row in R]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--npz', default='')
    ap.add_argument('--duration', type=float, default=45.0)
    ap.add_argument('--settle', type=float, default=0.0)
    ap.add_argument('--frames', type=int, default=3)
    ap.add_argument('--variant', default='')
    # ★ 2026-10-09：A/B 用的三个开关（都不带 = 探针仍然**只订阅、不发布**）
    ap.add_argument('--drive', action='store_true',
                    help='记录窗内注入固定动作（直行 10 s vx=0.30 ↔ 原地转 10 s wz=0.60），'
                         '末尾报"odom 位移 − 真值位移"漂移（口径同 docs/tilted_lidar_fidelity.md §H.5）')
    ap.add_argument('--goal-forward', type=float, default=None,
                    help='窗口末尾沿车头方向发一次 /goal_pose（map 系），观察 --goal-wait 秒')
    ap.add_argument('--goal-wait', type=float, default=30.0)
    ap.add_argument('--robot-radius', type=float, default=0.3565,
                    help='local costmap "车半径圆内"统计用的半径（robot11 = 0.3565）')
    args = ap.parse_args()

    rclpy.init()
    node = TiltProbe(args)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    if args.settle > 0:
        print('[probe] settle %.0f s …' % args.settle, flush=True)
        time.sleep(args.settle)
    t0 = time.time()
    print('[probe] 记录窗口 %.0f s …%s' % (args.duration,
          '（--drive：直行/原地转交替，注入点 /cmd_vel_chassis）' if args.drive else ''),
          flush=True)
    with node.lock:
        node.clock_marks.append((time.time(), node.clock if node.clock is not None else 0.0))
    while time.time() - t0 < args.duration:
        if args.drive:
            node.drive_step(time.time() - t0)
        time.sleep(0.5)
        with node.lock:
            nraw = node.frames.get('raw', 0)
            nreg = node.frames.get('registered', 0)
            nimu = node.frames.get('imu', 0)
        print('[probe] t=%.0f raw=%d reg=%d imu=%d' % (time.time() - t0, nraw, nreg, nimu),
              flush=True)
    with node.lock:
        node.clock_marks.append((time.time(), node.clock if node.clock is not None else 0.0))
    if args.drive:
        try:
            node.vel_pub.publish(Twist())          # 收尾一发零速（与 regress 工具同款）
        except Exception:
            pass

    # ★ 2026-10-09：`--goal-forward` = 窗口末尾发一次 /goal_pose（在 settle+窗口之后 ⇒ 不会
    #   落在 spawn/插件加载窗口里，见 §H.6 的沙箱崩溃教训）
    if args.goal_forward is not None:
        pose = node.robot_map_pose()
        if pose is None:
            print('[probe] **拿不到 map→base_link ⇒ 不发目标**', flush=True)
        else:
            gx = pose[0] + args.goal_forward * math.cos(pose[2])
            gy = pose[1] + args.goal_forward * math.sin(pose[2])
            print('[probe] 车在 map 系 (%.3f, %.3f, %.1f°)，目标 = 车头前 %.2f m → (%.3f, %.3f)'
                  % (pose[0], pose[1], math.degrees(pose[2]), args.goal_forward, gx, gy), flush=True)
            with node.lock:
                plan0 = node.frames.get('plan', 0)
                cmd0 = len(node.cmdvel_abs)
                gt0 = list(node.odom.get('/odom_ground_truth', []))
            node.send_goal(gx, gy)
            time.sleep(args.goal_wait)
            post = node.robot_map_pose()
            with node.lock:
                plan1 = node.frames.get('plan', 0)
                cmd1 = len(node.cmdvel_abs)
                gt1 = list(node.odom.get('/odom_ground_truth', []))
            def _xy(seq):
                a = np.array([r[:8] for r in seq], dtype=np.float64) if seq else None
                return (None if a is None else
                        (float(a[-1, 1]), float(a[-1, 2]), R_to_rpy_deg(quat_to_R(*a[-1, 4:8]))[2]))
            p0, p1 = _xy(gt0), _xy(gt1)
            node.goal_result = {
                'goal': [round(gx, 4), round(gy, 4)],
                'goal_forward_m': args.goal_forward, 'goal_wait_s': args.goal_wait,
                'pre_map_pose': None if pose is None else [round(v, 4) for v in pose],
                'post_map_pose': None if post is None else [round(v, 4) for v in post],
                'gt_pre': None if p0 is None else [round(v, 4) for v in p0],
                'gt_post': None if p1 is None else [round(v, 4) for v in p1],
                'gt_moved_m': (None if (p0 is None or p1 is None) else
                               round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 5)),
                'gt_yaw_delta_deg': (None if (p0 is None or p1 is None) else
                                     round(math.degrees(p1[2] - p0[2]), 3)),
                'plan_frames_during_wait': int(plan1 - plan0),
                'cmd_vel_n_during_wait': int(cmd1 - cmd0),
                'dist_to_goal_after_m': (None if post is None else
                                         round(math.hypot(post[0] - gx, post[1] - gy), 4))}
            print('[probe] 目标后：%s' % node.goal_result, flush=True)

    node.snapshot_tf()

    out = {'variant': args.variant, 'frames': dict(node.frames),
           'frame_ids': {k: dict(v) for k, v in node.frame_ids.items()},
           'pts_median': {k: pct(v, 50) for k, v in node.pts.items() if v},
           'tf': node.tf}
    # ★ 2026-10-09：RTF（记录窗口径：Δ仿真钟 / Δ墙钟；与 /clock 同一条时间轴）
    with node.lock:
        marks = list(node.clock_marks)
    if len(marks) >= 2:
        dc = marks[-1][1] - marks[0][1]
        dw = max(1e-6, marks[-1][0] - marks[0][0])
        out['rtf'] = {'sim_seconds': round(dc, 3), 'wall_seconds': round(dw, 3),
                      'rtf': round(dc / dw, 4)}

    # ---- 点云统计（raw / registered / obstacle / voxel_grid 的**最后一帧**，再对 raw 全部帧）
    cloud_stats_out = {}
    npz = {}
    with node.lock:
        clouds = {k: list(v) for k, v in node.clouds.items()}
    for tag, lst in clouds.items():
        if not lst:
            continue
        stats = []
        for st, fid, P in lst:
            s = cloud_stats(tag, P, fid, st)
            stats.append(s)
            npz['%s_%d' % (tag, len(npz))] = P
        cloud_stats_out[tag] = stats

    # ---- 用 LIO 自己的 TF 把 raw 云搬到 odom / base_link 再拟合（分开"odom 斜"与"发布多转"）
    if clouds.get('raw'):
        st, fid, P = clouds['raw'][-1]
        moved = {}
        for tgt in ('odom', 'base_link'):
            tr = node._lookup(tgt, fid, stamp=st)
            if tr is None:
                moved[tgt] = None
                continue
            R, t = tf_msg_to_RT(tr)
            Q = (R @ P.T).T + t
            moved[tgt] = cloud_stats('raw_via_TF_' + tgt, Q,
                                     '%s(经 TF %s<-%s)' % (fid, tgt, fid), st)
            npz['raw_via_%s' % tgt] = Q
        cloud_stats_out['raw_via_tf'] = moved

    # ---- 假设检验：raw → registered 是不是"多转了一个 T(base_link<-livox_frame)"
    hyp = {}
    if clouds.get('raw') and clouds.get('registered'):
        st_r, fid_r, P_r = clouds['raw'][-1]
        # 取时间最接近的一帧 registered
        st_g, fid_g, P_g = min(clouds['registered'], key=lambda z: abs(z[0] - st_r))
        tr_bl = node._lookup('base_link', fid_r, stamp=st_r)
        R_bl, t_bl = (tf_msg_to_RT(tr_bl) if tr_bl is not None
                      else (np.eye(3), np.zeros(3)))
        # H_a: p_pub ≈ R_bl·p_raw + t
        t_a = (P_g.mean(axis=0) - (R_bl @ P_r.mean(axis=0)))
        # 残差按"每点最近邻"太重，这里用同帧同序不可靠 ⇒ 用**点数中位数一致**的粗口径：
        # 对 H_a/H_b 都算"把 raw 按假设变换后再拟合地面平面"的角度（与 registered 的地面角对比）
        def plane_angle(Q):
            pl = fit_plane_ransac(Q)
            return None if pl is None else pl['ang_from_frame_z_deg']
        hyp = {
            'stamps': {'raw': st_r, 'registered': st_g, 'dt_s': abs(st_g - st_r)},
            'T_base_link_from_livox': {'rpy_deg': R_to_rpy_deg(R_bl),
                                       'xyz': [float(v) for v in t_bl]},
            'H_a_rot_extra': {
                'desc': 'p_pub ≈ R(base<-livox)·p_raw + t',
                'rot_deg': rot_angle_axis_deg(R_bl)[0],
                'rot_axis': rot_angle_axis_deg(R_bl)[1],
                't_fit': [float(v) for v in t_a],
                'raw_plane_ang_deg': plane_angle(P_r),
                'rot_then_plane_ang_deg': plane_angle((R_bl @ P_r.T).T + t_a),
                'published_plane_ang_deg': plane_angle(P_g)},
            'H_b_no_rot': {
                'desc': 'p_pub ≈ p_raw + t',
                't_fit': [float(v) for v in (P_g.mean(axis=0) - P_r.mean(axis=0))],
                'plane_ang_deg': plane_angle(P_r)},
        }
        # 一般刚体配准（最近邻配对 + Kabsch，两轮）—— 与上面的假设无关地复原旋转
        try:
            from scipy.spatial import cKDTree
            A, B = P_r, P_g
            R = np.eye(3)
            t = B.mean(axis=0) - A.mean(axis=0)
            for it in range(3):
                tree = cKDTree(B)
                d, idx = tree.query((R @ A.T).T + t, k=1)
                m = d < 0.25
                if m.sum() < 50:
                    break
                R, t = kabsch(A[m], B[idx[m]])
            ang, ax = rot_angle_axis_deg(R)
            resid = np.linalg.norm((R @ A.T).T + t - B[np.clip(idx, 0, len(B) - 1)], axis=1)
            hyp['kabsch_raw_to_published'] = {
                'n_pairs': int(m.sum()), 'angle_deg': ang, 'axis': ax,
                't': [float(v) for v in t], 'rpy_deg': R_to_rpy_deg(R),
                'resid_p50_m': pct(resid[m], 50), 'resid_p95_m': pct(resid[m], 95)}
            # 两档 raw 互配（H2 的本地云配准）留给离线 compare 脚本
        except Exception as e:                                # pragma: no cover
            hyp['kabsch_raw_to_published'] = {'error': str(e)}
    out['clouds'] = cloud_stats_out
    out['hypotheses'] = hyp

    # ---- IMU
    with node.lock:
        imu = np.array([[r[0], r[1], r[2], r[3], r[4], r[5], r[6]] for r in node.imu],
                       dtype=np.float64)
        imu_fid = dict(node.imu_frame_id)
        odom = {k: list(v) for k, v in node.odom.items()}
    if len(imu):
        a = imu[:, 1:4]
        g = imu[:, 4:7]
        am = a.mean(axis=0)
        gd = am / (np.linalg.norm(am) or 1.0)
        out['imu'] = {
            'n': int(len(imu)), 'frame_id': imu_fid,
            'acc_mean': [float(v) for v in am],
            'acc_std': [float(v) for v in a.std(axis=0)],
            'acc_norm_mean': float(np.linalg.norm(am)),
            'gyro_mean': [float(v) for v in g.mean(axis=0)],
            'gyro_absmax': float(np.abs(g).max()),
            'gravity_dir_imu_deg_from_z': math.degrees(
                math.acos(max(-1.0, min(1.0, abs(float(gd[2])))))),
        }
        # 重力方向搬到 base_link / odom（用 TF）
        for tgt in ('base_link', 'odom'):
            tr = node._lookup(tgt, 'imu_link')
            if tr is None:
                continue
            R, _ = tf_msg_to_RT(tr)
            gv = R @ (-gd)                               # 单位向量，指向"上"（-a 方向）
            out['imu']['gravity_up_in_%s' % tgt] = {
                'vec': [float(v) for v in gv],
                'deg_from_%s_z' % tgt: math.degrees(
                    math.acos(max(-1.0, min(1.0, abs(float(gv[2])))))),
                'rpy_deg': R_to_rpy_deg(R)}
    # ---- 真值 + LIO TF ⇒ odom 帧相对真实重力的倾角（不依赖 LIO 内部假设）
    gt = odom.get('/odom_ground_truth') or []
    if gt:
        arr = np.array([r[:8] for r in gt], dtype=np.float64)
        q = arr[-1, 4:8]
        R_gt = quat_to_R(*q)                             # R(world<-base_link)（Gazebo 世界）
        tr = node._lookup('odom', 'base_link')
        R_ob = tf_msg_to_RT(tr)[0] if tr is not None else None
        d = {'n': int(len(arr)), 'frame_id': gt[-1][8], 'child_frame_id': gt[-1][9],
             'pos_mean': [float(v) for v in arr[:, 1:4].mean(axis=0)],
             'base_rpy_in_world_deg': R_to_rpy_deg(R_gt)}
        if R_ob is not None:
            # 重力（世界 -z）在 odom 里的方向 = R(odom<-base)·R(base<-world)·(0,0,-1)
            g_odom = R_ob @ (R_gt.T @ np.array([0.0, 0.0, -1.0]))
            d['gravity_down_in_odom'] = [float(v) for v in g_odom]
            d['odom_tilt_from_gravity_deg'] = math.degrees(
                math.acos(max(-1.0, min(1.0, abs(float(-g_odom[2]))))))
            d['R_odom_from_base_rpy_deg'] = R_to_rpy_deg(R_ob)
            d['R_gt_R_from_base_deg'] = R_to_rpy_deg(R_gt)
        out['ground_truth'] = d
    for topic in ('/odom', '/odom_ground_truth'):
        seq = odom.get(topic) or []
        if not seq:
            continue
        arr = np.array([r[:8] for r in seq], dtype=np.float64)
        rpys = np.array([R_to_rpy_deg(quat_to_R(*q)) for q in arr[:, 4:8]])
        out.setdefault('odom_series', {})[topic] = {
            'n': int(len(arr)), 't_first': float(arr[0, 0]), 't_last': float(arr[-1, 0]),
            'rpy_first_deg': [round(float(v), 3) for v in rpys[0]],
            'rpy_last_deg': [round(float(v), 3) for v in rpys[-1]],
            'rpy_span_deg': [round(float(v), 3) for v in (rpys.max(axis=0) - rpys.min(axis=0))],
            'yaw_deg_series_t0_t1': [round(float(rpys[0, 2]), 3), round(float(rpys[-1, 2]), 3)],
            'pos_span_m': [round(float(v), 4) for v in (arr[:, 1:4].max(axis=0) - arr[:, 1:4].min(axis=0))]}
    if odom.get('/odom'):
        arr = np.array([r[:8] for r in odom['/odom']], dtype=np.float64)
        q = arr[-1, 4:8]
        out['lio_odometry'] = {
            'n': int(len(arr)), 'frame_id': odom['/odom'][-1][8],
            'child_frame_id': odom['/odom'][-1][9],
            'pos_mean': [float(v) for v in arr[:, 1:4].mean(axis=0)],
            'rpy_deg_mean_of_last': R_to_rpy_deg(quat_to_R(*q))}

    # ---- ★ 2026-10-09：`--drive` 的动作/漂移 + `/cmd_vel` 证据（"目标被接受但车不动"也看这里）
    with node.lock:
        cmd_abs = list(node.cmdvel_abs)
        plan_pts = list(node.pts.get('plan', []))
    out['cmd_vel'] = {
        'n': len(cmd_abs),
        'max_abs_vx_plus_wz': round(max(cmd_abs), 4) if cmd_abs else 0.0,
        'nonzero': int(sum(1 for v in cmd_abs if v > 1e-3))}
    out['plan'] = {'n_frames': len(plan_pts),
                   'pts_median': pct(plan_pts, 50) if plan_pts else None}
    if args.drive and odom.get('/odom') and odom.get('/odom_ground_truth'):
        def _trip(seq):
            a = np.array([r[:8] for r in seq], dtype=np.float64)
            d = math.hypot(a[-1, 1] - a[0, 1], a[-1, 2] - a[0, 2])
            p = float(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2]))))
            y0 = R_to_rpy_deg(quat_to_R(*a[0, 4:8]))[2]
            y1 = R_to_rpy_deg(quat_to_R(*a[-1, 4:8]))[2]
            return d, p, y1, y0
        do_, po_, yo1, yo0 = _trip(odom['/odom'])
        dg_, pg_, yg1, yg0 = _trip(odom['/odom_ground_truth'])
        a = np.array([r[:8] for r in odom['/odom']], dtype=np.float64)
        b = np.array([r[:8] for r in odom['/odom_ground_truth']], dtype=np.float64)
        dx = (a[-1, 1] - a[0, 1]) - (b[-1, 1] - b[0, 1])
        dy = (a[-1, 2] - a[0, 2]) - (b[-1, 2] - b[0, 2])
        out['drive'] = {
            'protocol': '直行 10 s (vx=0.30) ↔ 原地转 10 s (wz=0.60) 交替 → /cmd_vel_chassis',
            'odom_displacement_m': round(do_, 5), 'odom_path_m': round(po_, 5),
            'gt_displacement_m': round(dg_, 5), 'gt_path_m': round(pg_, 5),
            'gt_yaw_start_end_deg': [round(yg0, 3), round(yg1, 3)],
            'drift_vs_truth_m': round(math.hypot(dx, dy), 5),
            'drift_vs_truth_yaw_deg': round((yo1 - yo0) - (yg1 - yg0), 4)}

    # ---- /scan：有限波束 + 逐扇区 + 搬到 odom 后的 z 分布（obstacle_layer 高度带）
    sc = None
    if node.scan_frames:
        f = max(node.scan_frames, key=lambda z: int(np.isfinite(z['ranges']).sum()))
        r = np.array(f['ranges'], dtype=np.float64)
        ang = f['angle_min'] + np.arange(len(r)) * f['angle_increment']
        fin = np.isfinite(r)
        az = np.degrees(ang)
        sect = []
        for k in range(12):
            lo = -180.0 + k * 30
            hi = lo + 30
            m = fin & (az >= lo) & (az < hi)
            sect.append({'sector_deg': [lo, hi], 'n_finite': int(m.sum()),
                         'r_min_m': float(r[m].min()) if m.any() else None,
                         'r_p50_m': pct(r[m], 50) if m.any() else None})
        sc = {'frame_id': f['frame_id'], 'n_total': int(len(r)), 'n_finite': int(fin.sum()),
              'angle_min': f['angle_min'], 'angle_increment': f['angle_increment'],
              'ranges': [float(v) for v in r],
              'r_min_m': float(r[fin].min()) if fin.any() else None,
              'r_p50_m': pct(r[fin], 50), 'sectors': sect}
        q = {'front(|az|<=45)': 0, 'left(45..135)': 0, 'back(|az|>135)': 0,
             'right(-135..-45)': 0}
        for s in sect:
            lo = s['sector_deg'][0]
            for name, (a, b) in (('front(|az|<=45)', (-45, 45)), ('left(45..135)', (45, 135)),
                                 ('back(|az|>135)', (135, 180)),
                                 ('right(-135..-45)', (-180, -45))):
                if a <= lo < b:
                    q[name] += s['n_finite']
        sc['quadrants_n_finite'] = q
        # ★ 2026-10-09：**距离带**直方图（与 tools/scripts/regress 探针的分带口径一致 ⇒ 可并排读）
        BANDS = [(0.00, 0.05), (0.05, 0.10), (0.10, 0.15), (0.15, 0.20), (0.20, 0.30),
                 (0.30, 0.50), (0.50, 1.00), (1.00, 2.00), (2.00, 4.00), (4.00, 7.00),
                 (7.00, 1e9)]
        rf = r[fin]
        sc['range_bands_per_frame'] = {
            ('%.2f-%.2f' % (lo, hi) if hi < 1e8 else '>7.00'): int(((rf >= lo) & (rf < hi)).sum())
            for lo, hi in BANDS}
        # 逐波束搬到 odom：z 分布（obstacle_layer 的 min/max_obstacle_height = 0/2.0 在 odom 里量）
        tr = node._lookup('odom', f['frame_id'])
        if tr is not None:
            R, t = tf_msg_to_RT(tr)
            rr = r[fin]
            aa = ang[fin]
            pts = np.stack([rr * np.cos(aa), rr * np.sin(aa), np.zeros_like(rr)], axis=1)
            Q = (R @ pts.T).T + t
            # 相对"车"的高度 = odom z 减去车心（base_link 原点）的 odom z
            z_rel = Q[:, 2] - t[2]
            z_odom = Q[:, 2]
            lo = np.degrees(np.arctan2(Q[:, 1], Q[:, 0]))
            dropped = (z_odom < 0.0) | (z_odom > 2.0)
            h, _ = np.histogram(lo[dropped], bins=np.arange(-180, 181, 30))
            sc['beam_z_odom_absolute'] = {
                'min': float(z_odom.min()), 'p05': pct(z_odom, 5), 'p50': pct(z_odom, 50),
                'p95': pct(z_odom, 95), 'max': float(z_odom.max()),
                'n_below_0': int((z_odom < 0.0).sum()),
                'n_above_2': int((z_odom > 2.0).sum()),
                'frac_outside_0_2': float(dropped.mean()),
                'dropped_az_hist': {SECTOR_NAMES[i]: int(h[i]) for i in range(len(h))}}
            sc['beam_z_in_odom_rel_to_robot'] = {
                'p01': pct(z_rel, 1), 'p05': pct(z_rel, 5), 'p50': pct(z_rel, 50),
                'p95': pct(z_rel, 95), 'p99': pct(z_rel, 99),
                'min': float(z_rel.min()), 'max': float(z_rel.max()),
                'frac_below_0': float((z_rel < 0).mean()),
                'frac_above_2': float((z_rel > 2.0).mean()),
                'frac_outside_0_2': float(((z_rel < 0) | (z_rel > 2.0)).mean())}
    out['scan'] = sc

    # ---- 代价图：lethal 格的方位分布（车体系）
    if 'local' in node.costmap:
        cm = dict(node.costmap['local'])
        data = cm.pop('_data')
        res, ox, oy = cm['resolution'], cm['origin'][0], cm['origin'][1]
        ys, xs = np.nonzero(data == 100)
        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res
        tr = node._lookup('odom', 'base_link')
        if tr is not None and len(wx):
            R, t = tf_msg_to_RT(tr)
            rel = np.stack([wx - t[0], wy - t[1]], axis=1)
            rel_b = rel @ R[:2, :2]                     # odom → base_link（只取平面部分）
            az = np.degrees(np.arctan2(rel_b[:, 1], rel_b[:, 0]))
            rr = np.linalg.norm(rel_b, axis=1)
            hist, _ = np.histogram(az, bins=np.arange(-180, 181, 30))
            cm['lethal_n'] = int(len(rr))
            cm['lethal_r_min_m'] = float(rr.min())
            cm['lethal_r_p50_m'] = pct(rr, 50)
            cm['lethal_az_hist_robot_frame'] = {SECTOR_NAMES[i]: int(hist[i])
                                                for i in range(len(hist))}
            cm['lethal_quadrants'] = {
                'front': int(((np.abs(az) <= 45)).sum()),
                'left': int(((az > 45) & (az <= 135)).sum()),
                'back': int((np.abs(az) > 135).sum()),
                'right': int(((az > -135) & (az <= -45)).sum())}
        # ★ 2026-10-09：**车半径圆内**的格（"车一开始就在膨胀团里"的判据，见 §D.2 的口径）。
        #   车心 = TF(costmap_frame ← base_link) 的平移 —— 与 lethal 的方位统计同一个变换。
        try:
            trc = node._lookup(cm['frame_id'], 'base_link')
            if trc is not None:
                rc = tf_msg_to_RT(trc)
                cx, cy = float(rc[1][0]), float(rc[1][1])
                yy, xx = np.nonzero(np.ones_like(data, dtype=bool))
                wx2 = ox + (xx + 0.5) * res
                wy2 = oy + (yy + 0.5) * res
                rr = np.hypot(wx2 - cx, wy2 - cy)
                m_in = rr <= float(node.args.robot_radius)
                vals = data[yy[m_in], xx[m_in]]
                cm['robot_center_cell'] = int(data[int((cy - oy) / res), int((cx - ox) / res)])
                cm['robot_center_xy_in_map'] = [round(cx, 4), round(cy, 4)]
                cm['circle_cells'] = int(m_in.sum())
                cm['circle_n_ge99'] = int((vals >= 99).sum())
                cm['circle_n_lethal'] = int((vals == 100).sum())
                cm['circle_n_free'] = int((vals == 0).sum())
                cm['circle_n_unknown'] = int((vals < 0).sum())
        except Exception as e:                                # pragma: no cover
            cm['circle_error'] = str(e)
        out['costmap_local'] = cm

    out['goal'] = node.goal_result
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=1, ensure_ascii=False, default=str)
    if 'local' in node.costmap and 'local_costmap' not in npz:
        npz['local_costmap'] = node.costmap['local']['_data'].astype(np.int8)
    if args.npz and npz:
        np.savez_compressed(args.npz, **npz)
    print('[probe] 写出 %s' % args.out, flush=True)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
