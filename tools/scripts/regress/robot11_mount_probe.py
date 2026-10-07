#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11_mount_probe.py —— 「雷达安装方式（帧正/帧斜）」的逐帧取证探针（2026-10-08）。

用途：回答三件事，全部只要**实测数字**，不做推断：
  A. 点云到底表达在哪个坐标系里 —— 直接量 `/livox/lidar/pointcloud` 与 `/cloud_registered` 的
     `header.frame_id` + **地面点的平面拟合法向**（法向与 link z 的夹角 0° ⇒ 帧是重力对齐的；
     ≈30° ⇒ 点云表达在**斜的传感器系**里），并同时给出"到水平面 z=−h"与"到斜平面 n·p=−h"
     两种残差分布 —— 两个模型的数字放在一起，哪个模型对不上就一眼看得出来。
  B. 每帧点的**逐级账本**：/livox/lidar（CustomMsg）→ /livox/lidar/pointcloud → /cloud_registered
     → /segmentation/ground|obstacle → /scan，每一级的点数 + 近场/远场/低于地面/超量程的细账。
  D. 代价图归因：local/global costmap 的 lethal(100)/inscribed(99)/inflated(1..98)/free/unknown
     计数 + **以车为中心的距离剖面** + 车所在格与"车半径圆内"的代价 ⇒ 判定"车是不是一开始
     就在一个 inscribed 团里"。整张 local 栅格可落盘（npz/CSV）供离线复算。

只订阅、不发布除 /cmd_vel_chassis 零速以外的东西（--goal 时才发一次 NavigateToPose）。
不改任何节点的参数、不动 TF、不动 /segmentation/* 契约。

用法：
  python3 tools/scripts/regress/robot11_mount_probe.py --out .tmp_x/probe.json \
      --dump-dir .tmp_x/frames --duration 60 [--goal 0.8 0.0] [--grid-dump .tmp_x/local_costmap.npz]
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
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
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
QOS_TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def rot_deg_from_quat(x, y, z, w):
    """四元数 → 欧拉角（度），与 tf2 的 RPY 同序（先 roll 后 pitch 再 yaw）。"""
    roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return [roll, pitch, yaw]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def fit_plane_ransac(P, iters=200, tol=0.02, seed=0):
    """极简 RANSAC 平面拟合（拿最大平面 —— 在这台车上就是地面）。返回 (n, d, inlier_frac)。"""
    rng = np.random.default_rng(seed)
    if len(P) < 50:
        return None
    best = (None, None, -1)
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
        dist = np.abs(P @ n + d)
        frac = float((dist < tol).mean())
        if frac > best[2]:
            best = (n, d, frac)
    n, d, frac = best
    if n is None:
        return None
    # 用内点做一次最小二乘精修
    dist = np.abs(P @ n + d)
    inl = P[dist < tol]
    if len(inl) >= 50:
        c = inl.mean(axis=0)
        u, s, vt = np.linalg.svd(inl - c)
        n2 = vt[2]
        d2 = -float(n2 @ c)
        if (np.abs(P @ n2 + d2) < tol).mean() >= frac:
            n, d = n2, d2
    if n[2] < 0:          # 统一朝上（z 分量为正），便于读
        n, d = -n, -d
    return n, d, float((np.abs(P @ n + d) < tol).mean())


class MountProbe(Node):
    def __init__(self, args):
        super().__init__('robot11_mount_probe')
        self.args = args
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.frames = Counter()
        self.pts = defaultdict(list)                # 话题 → 每帧点数
        self.cmdvel_abs = []
        self._R_cache = None
        self.frame_ids = defaultdict(Counter)       # 话题 → frame_id 计数
        self.detail = defaultdict(list)             # 话题 → 逐帧细账（只存详采的那几帧）
        self.scan_frames = []
        self.clock = None
        self.clock_t0 = None
        self.odom = defaultdict(list)               # /odom /odom_ground_truth → (t, x, y, yaw)
        self.tf_snap = {}
        self.costmap = {}                           # 话题 → dict(grid, info, robot_cell, ...)
        self.dumps = defaultdict(int)
        self.dump_dir = args.dump_dir
        if self.dump_dir:
            os.makedirs(self.dump_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Clock, '/clock', self.on_clock, QOS_BE)
        if HAVE_CUSTOM:
            self.create_subscription(CustomMsg, '/livox/lidar', self.on_custom, QOS_BE)
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud',
                                 lambda m: self.on_cloud('raw', m), QOS_BE)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 lambda m: self.on_cloud('registered', m), QOS_BE)
        self.create_subscription(PointCloud2, '/segmentation/ground',
                                 lambda m: self.on_cloud('ground', m), QOS_BE)
        self.create_subscription(PointCloud2, '/segmentation/obstacle',
                                 lambda m: self.on_cloud('obstacle', m), QOS_BE)
        self.create_subscription(LaserScan, '/scan', self.on_scan, QOS_BE)
        for t in ('/odom', '/odom_ground_truth'):
            self.create_subscription(Odometry, t,
                                     lambda m, t=t: self.on_odom(t, m), QOS_RE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: self.on_costmap('local', m), QOS_BE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self.on_costmap('global', m), QOS_BE)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap_raw',
                                 lambda m: self.on_costmap('local_raw', m), QOS_BE)
        # ★ 2026-10-08：global_costmap 的 STVL 体素图（publish_voxel_map: true）——
        #   体素中心 = **真的被标成障碍的三维点** ⇒ 用来把"哪些点进了代价图"归因到源头
        #   （自击点 / 地面点 / 真障碍），而不是只从代价图的格值反推。
        self.create_subscription(PointCloud2, '/global_costmap/voxel_grid',
                                 lambda m: self.on_cloud('voxel_grid', m), QOS_BE)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_chassis', 10)
        self.goal_pub = None              # --goal 时才建（不需要就不碰 nav2 的契约）
        # nav2 侧行为取证（只数消息，不改任何东西）：规划器有没有出路径、控制器有没有发力
        self.create_subscription(Path, '/plan', self.on_plan, QOS_RE)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmdvel, QOS_RE)

    def on_plan(self, m):
        with self.lock:
            self.frames['plan'] += 1
            self.pts['plan'].append(len(m.poses))

    def on_cmdvel(self, m):
        with self.lock:
            self.frames['cmd_vel'] += 1
            self.pts['cmd_vel'].append(1)
            self.cmdvel_abs.append(abs(m.linear.x) + abs(m.angular.z))

    # ---------------- 回调 ----------------
    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        with self.lock:
            if self.clock_t0 is None:
                self.clock_t0 = (t, time.time())
            self.clock = t

    def on_odom(self, topic, m):
        p = m.pose.pose
        yaw = rot_deg_from_quat(p.orientation.x, p.orientation.y, p.orientation.z,
                                p.orientation.w)[2]
        with self.lock:
            self.odom[topic].append((time.time(), p.position.x, p.position.y, yaw))

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
        try:
            arr = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
                m, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float64)
        except Exception as e:                                # pragma: no cover
            arr = np.zeros((0, 3))
            sys.stderr.write('[probe] 读 %s 点云失败: %s\n' % (tag, e))
        d = self._cloud_stats(arr, m.header.frame_id)
        d['frame_id'] = m.header.frame_id
        d['stamp'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.detail[tag].append(d)
        if self.dump_dir and len(arr):
            np.savetxt(os.path.join(self.dump_dir, '%s_%02d.csv' % (tag, self.dumps[tag])),
                       arr, delimiter=',', fmt='%.5f', header='x,y,z', comments='')

    def on_scan(self, m):
        r = np.array(m.ranges, dtype=np.float64)
        fin = r[np.isfinite(r)]
        band = {}
        if len(fin):
            edges = [0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0, 2.0, 4.0, 7.0, 1e9]
            for lo, hi in zip(edges[:-1], edges[1:]):
                band['%.2f-%.2f' % (lo, hi)] = int(((fin >= lo) & (fin < hi)).sum())
        with self.lock:
            self.frames['scan'] += 1
            self.pts['scan'].append(int(len(fin)))
            self.frame_ids['scan'][m.header.frame_id] += 1
            n = self.dumps['scan']
            want = n < self.args.frames
            if want:
                self.dumps['scan'] += 1
            self.scan_frames.append({
                'n_total': int(len(r)), 'n_finite': int(len(fin)), 'band': band,
                'min': float(fin.min()) if len(fin) else None,
                'max': float(fin.max()) if len(fin) else None,
                'range_min': m.range_min, 'range_max': m.range_max,
                'angle_min': m.angle_min, 'angle_max': m.angle_max,
                'angle_increment': m.angle_increment})
        if want and self.dump_dir:
            # 落盘"方位 + 距离"（供离线把它映射到代价图的格上做归因）
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            np.savetxt(os.path.join(self.dump_dir, 'scan_%02d.csv' % (n + 1)),
                       np.stack([ang, r], axis=1), delimiter=',', fmt='%.6f',
                       header='angle_rad,range', comments='')

    def on_costmap(self, tag, m):
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        yaw = rot_deg_from_quat(info.origin.orientation.x, info.origin.orientation.y,
                                info.origin.orientation.z, info.origin.orientation.w)[2]
        ent = {'frame_id': m.header.frame_id, 'nx': info.width, 'ny': info.height,
               'res': info.resolution, 'origin': [info.origin.position.x,
                                                  info.origin.position.y,
                                                  info.origin.position.z],
               'origin_yaw_deg': yaw, 'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9}
        vals, cnts = np.unique(grid, return_counts=True)
        ent['value_hist'] = {int(v): int(c) for v, c in zip(vals, cnts)}
        ent['n_lethal_100'] = int((grid == 100).sum())
        ent['n_inscribed_99'] = int((grid == 99).sum())
        ent['n_inflated_1_98'] = int(((grid >= 1) & (grid <= 98)).sum())
        ent['n_free_0'] = int((grid == 0).sum())
        ent['n_unknown_-1'] = int((grid == -1).sum())
        ent['n_ge99'] = int((grid >= 99).sum())
        # 车在代价图里的位置（局部图 = odom 系；全局图 = map 系）
        base = 'base_link'
        try:
            tr = self.tf_buffer.lookup_transform(m.header.frame_id, base,
                                                 rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            tr = None
        if tr is not None:
            bx = tr.transform.translation.x
            by = tr.transform.translation.y
            cx = int(math.floor((bx - info.origin.position.x) / info.resolution))
            cy = int(math.floor((by - info.origin.position.y) / info.resolution))
            ent['robot_xy_in_frame'] = [bx, by]
            ent['robot_cell'] = [cx, cy]
            ent['robot_cell_inside'] = bool(0 <= cx < info.width and 0 <= cy < info.height)
            if ent['robot_cell_inside']:
                ent['robot_cell_cost'] = int(grid[cy, cx])
                # 以车为中心的距离剖面（米 → 该环内的最大代价 / 非空格数）
                xs = (np.arange(info.width) + 0.5) * info.resolution + info.origin.position.x
                ys = (np.arange(info.height) + 0.5) * info.resolution + info.origin.position.y
                DX = xs[None, :] - bx
                DY = ys[:, None] - by
                D = np.sqrt(DX * DX + DY * DY)
                prof = []
                edges = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.3565, 0.4, 0.5,
                         0.6, 0.7, 0.75, 0.8, 1.0, 1.5, 2.0]
                for lo, hi in zip(edges[:-1], edges[1:]):
                    msk = (D >= lo) & (D < hi)
                    if not msk.any():
                        continue
                    g = grid[msk]
                    prof.append({'r': [lo, hi], 'cells': int(msk.sum()),
                                 'n_ge99': int((g >= 99).sum()),
                                 'n_inflated': int(((g >= 1) & (g <= 98)).sum()),
                                 'max_cost': int(g.max()), 'mean_cost': round(float(g.mean()), 2)})
                ent['radial_profile'] = prof
                ent['min_dist_to_ge99'] = float(D[grid >= 99].min()) if (grid >= 99).any() else None
                ent['min_dist_to_lethal'] = float(D[grid == 100].min()) if (grid == 100).any() else None
                # 车半径圆（0.3565 m）内的格数与其中非 free 的比例
                msk = D <= 0.3565
                ent['disc_cells'] = int(msk.sum())
                ent['disc_ge99'] = int((grid[msk] >= 99).sum())
                ent['disc_free'] = int((grid[msk] == 0).sum())
        with self.lock:
            self.costmap[tag] = ent
            if self.args.grid_dump and tag == 'local':
                np.savez_compressed(self.args.grid_dump, grid=grid, res=info.resolution,
                                    ox=info.origin.position.x, oy=info.origin.position.y,
                                    frame=m.header.frame_id)

    # ---------------- 细账 ----------------
    def _mount_R(self):
        """base_link←livox_frame 的旋转（实测 TF；2 s 缓存）——决定"点云该按哪个平面解释"。"""
        now = time.time()
        if self._R_cache is not None and now - self._R_cache[0] < 2.0:
            return self._R_cache[1]
        R = None
        try:
            tr = self.tf_buffer.lookup_transform('base_link', 'livox_frame', rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=0.2))
            q = tr.transform.rotation
            R = quat_to_R(q.x, q.y, q.z, q.w)
        except Exception:
            R = None
        self._R_cache = (now, R)
        return R

    def _cloud_stats(self, P, frame_id=''):
        d = {'n': int(len(P))}
        if not len(P):
            return d
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        rxy = np.hypot(x, y)
        r = np.linalg.norm(P, axis=1)
        elev = np.degrees(np.arctan2(-z, rxy))
        d.update({
            'z_min': float(z.min()), 'z_p01': pct(z, 1), 'z_p50': pct(z, 50),
            'z_p99': pct(z, 99), 'z_max': float(z.max()),
            'rxy_p05': pct(rxy, 5), 'rxy_p50': pct(rxy, 50), 'rxy_p95': pct(rxy, 95),
            'r_p50': pct(r, 50),
            'elev_p01': pct(elev, 1), 'elev_p50': pct(elev, 50), 'elev_p99': pct(elev, 99),
            'elev_min': float(elev.min()), 'elev_max': float(elev.max()),
            'n_r_lt_005': int((r < 0.05).sum()), 'n_r_lt_012': int((r < 0.12).sum()),
            'n_r_lt_03': int((r < 0.3).sum()), 'n_r_lt_05': int((r < 0.5).sum()),
            'n_r_gt_4': int((r > 4.0).sum()), 'n_r_gt_10': int((r > 10.0).sum()),
            'n_rxy_lt_012': int((rxy < 0.12).sum()), 'n_rxy_lt_05': int((rxy < 0.5).sum()),
        })
        # 平面拟合（地面）：用 r>0.35 m 的点，避免近场自击把平面带歪
        far = P[rxy > 0.35]
        pf = fit_plane_ransac(far, iters=300, tol=0.02)
        if pf:
            n, dd, frac = pf
            d['plane_n'] = [round(float(v), 5) for v in n]
            d['plane_d'] = round(float(dd), 5)
            d['plane_inlier_frac'] = round(frac, 4)
            d['plane_tilt_vs_link_z_deg'] = round(math.degrees(
                math.acos(min(1.0, max(-1.0, abs(float(n[2])))))), 3)
        # ★ 2026-10-08：两种"地面平面"模型下的残差（这才是"帧正/帧斜"的判据）：
        #   ① 模型 LEVEL（帧重力对齐）：地面 = link 系的水平面 z = −h
        #   ② 模型 TILT （点云表达在斜的传感器系）：地面 = 法向 n_up = Rᵀ·ẑ 的平面 n·p = −h
        #   h = 雷达离地高度（livox_frame 0.2595；odom 系里地面在 z = −0.102499）。
        h = 0.2595 if frame_id == 'livox_frame' else (0.102499 if frame_id == 'odom' else None)
        if h is not None:
            rl = z + h
            d['resid_level_p05/p50/p95'] = [round(pct(rl, 5), 4), round(pct(rl, 50), 4),
                                            round(pct(rl, 95), 4)]
            d['n_below_level_plane'] = int((rl < -0.02).sum())
            R = self._mount_R() if frame_id == 'livox_frame' else np.eye(3)
            if R is not None:
                nup = R.T @ np.array([0.0, 0.0, 1.0])       # 世界"上"在点云自己的帧里
                rt = P @ nup + h
                d['tilt_plane_n_up'] = [round(float(v), 5) for v in nup]
                d['resid_tilt_p05/p50/p95'] = [round(pct(rt, 5), 4), round(pct(rt, 50), 4),
                                               round(pct(rt, 95), 4)]
                d['n_below_tilt_plane'] = int((rt < -0.02).sum())
        return d

    # ---------------- 报告 ----------------
    def snapshot_tf(self):
        out = {}
        for parent, child in [('base_link', 'livox_frame'), ('base_link', 'imu_link'),
                              ('base_link', 'l12')]:
            try:
                tr = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(),
                                                     timeout=rclpy.duration.Duration(seconds=0.3))
            except Exception as e:
                out['%s->%s' % (parent, child)] = {'error': str(e)[:80]}
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            out['%s->%s' % (parent, child)] = {
                'xyz': [round(t.x, 6), round(t.y, 6), round(t.z, 6)],
                'rpy_deg': [round(v, 4) for v in rot_deg_from_quat(q.x, q.y, q.z, q.w)],
                'frame_id': tr.header.frame_id, 'child_frame_id': tr.child_frame_id}
        with self.lock:
            self.tf_snap = out

    def vel(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.vel_pub.publish(m)

    def robot_map_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            sys.stderr.write('[probe] 拿不到 map→base_link: %s\n' % str(e)[:100])
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        return t.x, t.y, math.radians(rot_deg_from_quat(q.x, q.y, q.z, q.w)[2])

    def send_goal(self, x, y):
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

    def report(self):
        with self.lock:
            frames = dict(self.frames)
            pts = {k: list(v) for k, v in self.pts.items()}
            ids = {k: dict(v) for k, v in self.frame_ids.items()}
            detail = {k: list(v) for k, v in self.detail.items()}
            scan = list(self.scan_frames)
            odom = {k: list(v) for k, v in self.odom.items()}
            cm = dict(self.costmap)
            clock = self.clock
            ct0 = self.clock_t0
        out = {'wall_s': round(time.time() - self.t0, 2), 'frames': frames,
               'frame_ids': ids, 'tf': self.tf_snap, 'costmap': cm}
        out['points_per_frame'] = {
            k: {'n_frames': len(v), 'median': float(np.median(v)) if v else None,
                'min': int(min(v)) if v else None, 'max': int(max(v)) if v else None}
            for k, v in pts.items()}
        out['points_per_frame_all'] = pts
        out['cloud_detail'] = detail
        if scan:
            agg = Counter()
            nf_median = float(np.median([s['n_finite'] for s in scan]))
            for s in scan:
                for k, v in s['band'].items():
                    agg[k] += v / max(1, len(scan))
            out['scan'] = {
                'n_frames': len(scan),
                'finite_beams_median': nf_median,
                'finite_beams_all': [s['n_finite'] for s in scan],
                'band_per_frame': {k: round(v, 2) for k, v in agg.items()},
                'band_per_frame_frac': {k: round(v / max(1.0, nf_median), 4)
                                        for k, v in agg.items()},
                'nearest': min([s['min'] for s in scan if s['min'] is not None], default=None),
                'farthest': max([s['max'] for s in scan if s['max'] is not None], default=None),
                'range_min': scan[-1]['range_min'], 'range_max': scan[-1]['range_max'],
                'n_beams_total': scan[-1]['n_total'],
            }
        if clock is not None and ct0 is not None:
            out['rtf'] = round((clock - ct0[0]) / max(1e-6, time.time() - ct0[1]), 4)
            out['sim_seconds'] = round(clock, 2)
        for topic, rows in odom.items():
            if len(rows) < 2:
                out.setdefault('odom', {})[topic] = {'n': len(rows)}
                continue
            xs = np.array([r[1] for r in rows])
            ys = np.array([r[2] for r in rows])
            yaws = np.array([r[3] for r in rows])
            out.setdefault('odom', {})[topic] = {
                'n': len(rows),
                'start': [round(float(xs[0]), 4), round(float(ys[0]), 4), round(float(yaws[0]), 3)],
                'end': [round(float(xs[-1]), 4), round(float(ys[-1]), 4), round(float(yaws[-1]), 3)],
                'displacement_m': round(float(math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])), 5),
                'path_m': round(float(np.sum(np.hypot(np.diff(xs), np.diff(ys)))), 5)}
        if '/odom' in odom and '/odom_ground_truth' in odom:
            a, b = odom['/odom'], odom['/odom_ground_truth']
            # ★ 口径 = **位移差**（本仓惯例，见 docs/robot_models.md §11.5 注）：
            #   odom 的起点是 (0,0)，真值起点是出生点 ⇒ 绝对差混着出生点偏移，不能当漂移。
            dx = (a[-1][1] - a[0][1]) - (b[-1][1] - b[0][1])
            dy = (a[-1][2] - a[0][2]) - (b[-1][2] - b[0][2])
            out['drift_vs_truth_m'] = round(float(math.hypot(dx, dy)), 5)
            out['drift_vs_truth_yaw_deg'] = round(
                float((a[-1][3] - a[0][3]) - (b[-1][3] - b[0][3])), 4)
        with self.lock:
            if self.cmdvel_abs:
                out['cmd_vel'] = {'n': len(self.cmdvel_abs),
                                  'max_abs': round(float(max(self.cmdvel_abs)), 4),
                                  'nonzero': int(sum(1 for v in self.cmdvel_abs if v > 1e-3))}
            else:
                out['cmd_vel'] = {'n': 0}
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--duration', type=float, default=60.0, help='采样窗口（墙钟秒）')
    ap.add_argument('--settle', type=float, default=0.0, help='先等这么多秒再开始记（默认 0）')
    ap.add_argument('--frames', type=int, default=3, help='每类点云落盘/细算的帧数')
    ap.add_argument('--dump-dir', default='')
    ap.add_argument('--grid-dump', default='')
    ap.add_argument('--goal', nargs=2, type=float, default=None,
                    help='窗口末尾发一次 /goal_pose（x y，map 系）并观察 30 s —— 只为取证，不改参数')
    ap.add_argument('--goal-forward', type=float, default=None,
                    help='窗口末尾发一次 /goal_pose = 当前 map 位姿沿车头前进这么多米，观察 30 s')
    ap.add_argument('--goal-wait', type=float, default=30.0)
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = MountProbe(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    if a.settle > 0:
        time.sleep(a.settle)
    t_end = time.time() + a.duration
    goal_xy = None
    try:
        while time.time() < t_end:
            time.sleep(0.5)
        node.snapshot_tf()
        if a.goal_forward is not None:
            pose = node.robot_map_pose()
            if pose:
                goal_xy = (pose[0] + a.goal_forward * math.cos(pose[2]),
                           pose[1] + a.goal_forward * math.sin(pose[2]))
                sys.stderr.write('[probe] 车在 map 系 (%.3f, %.3f, %.1f°)，目标 = 车头前 %.2f m '
                                 '→ (%.3f, %.3f)\n' % (pose[0], pose[1],
                                                       math.degrees(pose[2]), a.goal_forward,
                                                       goal_xy[0], goal_xy[1]))
        elif a.goal:
            goal_xy = tuple(a.goal)
        if goal_xy:
            pre = node.robot_map_pose()
            sys.stderr.write('[probe] 发 /goal_pose %.3f %.3f（只为取证，不改任何参数）\n' % goal_xy)
            node.send_goal(*goal_xy)
            time.sleep(a.goal_wait)
            post = node.robot_map_pose()
            sys.stderr.write('[probe] 目标后车在 map 系 %s\n' % (str(post),))
            with node.lock:
                node.goal_result = {'goal': list(goal_xy), 'pre': pre, 'post': post}
    finally:
        rep = node.report()
        rep['goal'] = getattr(node, 'goal_result', None)
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
        sys.stderr.write('[probe] 写 %s\n' % a.out)
        try:
            node.vel(0.0, 0.0)
        except Exception:
            pass
        # ★ 2026-10-08：收尾用 os._exit —— 之前 ex.shutdown()/destroy_node() 在
        #   MultiThreadedExecutor 还在 spin 时会 terminate（实测 exit 134/SIGABRT，
        #   虽然 probe.json 已经写完，但 rc 会变成 134 干扰调用方判定）。
        #   本探针只读、只落盘，JSON 已 flush+close ⇒ 直接退出是安全的。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
