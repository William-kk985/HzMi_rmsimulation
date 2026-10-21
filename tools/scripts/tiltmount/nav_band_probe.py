#!/usr/bin/env python3
"""nav_band_probe.py —— `mode:=nav` 下「/scan 高度带 + 局部代价图」取证探针（只订阅，不发布）。

回答的问题（见 `docs/tilted_lidar_fidelity.md` §J.5 / §K.1）：
  · `/scan` 是**二维平盘**，nav2 的 `obstacle_layer` 先 `projectLaser`（z 强行置 0）再用 TF
    搬到**代价图帧**（local `global_frame: odom`）⇒ **每条波束在代价图帧里的 z = 那一帧
    `scan` 帧原点的 odom z**，带子 `[min_obstacle_height(默认 0.0), max_obstacle_height(2.0)]`
    量的是这个常量。odom 一旦重力对齐，这个 z 会变 ⇒ 可能整盘掉出带子（"代价图变空"）。
  · 同时把局部代价图按 nav2 原始代价（254=LETHAL / 253=INSCRIBED / 0=free / 255=NO_INFO）
    数出来：lethal / free / unknown、车那格、到最近 lethal 的距离、车前方窗口内的 lethal 数。

口径：首帧 `/cloud_registered` 之后 settle 秒（仿真）起，持续 window 秒（仿真）。
每帧 `/scan` 记一次 z 统计；每条 `/local_costmap/costmap_raw` 记一次栅格统计。
输出一个 JSON（含逐帧/逐样本明细 + summary）。

用法：
  python3 nav_band_probe.py --out band.json --settle 20 --window 20 --wall-timeout 400
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2

RAW_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
LETHAL, INSCRIBED, NO_INFO = 254, 253, 255
Z_LO, Z_HI = 0.0, 2.0          # obstacle_layer.scan 的有效高度带（min 默认 0.0 / max 2.0）
AHEAD_X, AHEAD_Y = 2.0, 0.6    # 「车前方窗口」：x∈[0,2.0] m、|y|≤0.6 m（机器人系）


class Grid:
    def __init__(self, msg):
        self.res = float(msg.metadata.resolution)
        self.ox = float(msg.metadata.origin.position.x)
        self.oy = float(msg.metadata.origin.position.y)
        self.sx = int(msg.metadata.size_x)
        self.sy = int(msg.metadata.size_y)
        self.frame = msg.header.frame_id
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(self.sy, self.sx)
        self.arr = a
        self.n_lethal = int((a == LETHAL).sum())
        self.n_inscribed = int((a == INSCRIBED).sum())
        self.n_free = int((a == 0).sum())
        self.n_unknown = int((a == NO_INFO).sum())
        self.n_cells = int(self.sx * self.sy)
        self.lethal_xy = None
        if self.n_lethal:
            r, c = np.nonzero(a == LETHAL)
            self.lethal_xy = np.stack([self.ox + (c + 0.5) * self.res,
                                       self.oy + (r + 0.5) * self.res], axis=1)

    def cell_of(self, x, y):
        c = int(math.floor((x - self.ox) / self.res))
        r = int(math.floor((y - self.oy) / self.res))
        if not (0 <= c < self.sx and 0 <= r < self.sy):
            return None
        return r, c

    def dist_to_nearest_lethal(self, x, y):
        if self.lethal_xy is None:
            return None
        d = np.hypot(self.lethal_xy[:, 0] - x, self.lethal_xy[:, 1] - y)
        return float(d.min())


class Band(Node):
    def __init__(self, settle, window, frames, costmap_topic, class_topic, global_topic='',
                 scan_frame='odom_level'):
        super().__init__('nav_band_probe')
        # ★ 必须开 use_sim_time：TF 查询用的是 msg 里的**仿真**时间戳，节点若用系统时钟，
        #   每一次 lookup 都会 "extrapolation into the past"（第一版就踩了：124/124 全失败）。
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.settle, self.window = settle, window
        self.base_frame = frames
        self.costmap_topic = costmap_topic
        self.class_topic = class_topic
        self.global_topic = global_topic
        # ★ 2026-10-21（契约 §4.11）：扫描面 z / 在带必须量在**代价图自己的帧**里。
        #   改动前这里硬编码 `odom`；默认档局部代价图已经是 `odom_level` ⇒ 传 `--scan-frame odom`
        #   才是改动前的口径（回滚档 / 历史 A/B 复现用）。
        self.scan_frame = scan_frame
        self.sim = None
        self.t_first_cloud = None
        self.t_first_scan = None
        self.t_first_grid = None
        self.phase = 'wait'
        self.win_start_sim = None
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.scans = []
        self.grids = []
        self.grids_global = []
        self.class_grids = []
        self.n_scan_rx = self.n_grid_rx = self.n_class_rx = self.n_ggrid_rx = 0
        self.scan_frame_ids = set()
        self.tf_fail = 0
        self.tf_latest_fallback = 0
        self.tf_fail_total = 0
        q = qos_profile_sensor_data
        self.create_subscription(Clock, '/clock', self.on_clock, q)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, q)
        self.create_subscription(LaserScan, '/scan', self.on_scan, q)
        self.create_subscription(Costmap, costmap_topic, self.on_grid, RAW_QOS)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        if class_topic:
            self.create_subscription(Costmap, class_topic, self.on_class_grid, RAW_QOS)
        if global_topic:
            self.create_subscription(Costmap, global_topic, self.on_global_grid, RAW_QOS)
        self.odom_xy = []

    def on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cloud(self, m):
        if self.t_first_cloud is None:
            self.t_first_cloud = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_odom(self, m):
        if self.phase == 'window':
            p = m.pose.pose.position
            qq = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                             1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z))
            self.odom_xy.append([p.x, p.y, p.z, yaw, m.child_frame_id])

    def _lookup(self, target, source, stamp=None):
        """**非阻塞** TF 查询（timeout=0）：单线程 executor 里一旦阻塞等 TF，
        TF 订阅本身就再也进不来 ⇒ 第一版 124/124 全失败（自锁死）。
        静态场景（车不动）先按 msg 戳查，查不到就退回"最新一帧"（差值 <1 mm）。"""
        if stamp is not None:
            try:
                return self.buf.lookup_transform(target, source, Time.from_msg(stamp))
            except Exception:
                pass
            self.tf_latest_fallback += 1
        try:
            return self.buf.lookup_transform(target, source, Time())
        except Exception:
            self.tf_fail_total += 1
            return None

    def on_scan(self, m):
        if self.t_first_scan is None:
            self.t_first_scan = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.phase != 'window':
            return
        self.n_scan_rx += 1
        self.scan_frame_ids.add(m.header.frame_id)
        r = np.asarray(m.ranges, dtype=np.float64)
        if r.size == 0:
            return
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        n_finite = int(ok.sum())
        tf = self._lookup(self.scan_frame, m.header.frame_id, m.header.stamp)
        rec = {'n_beams': int(r.size), 'n_finite': n_finite,
               'range_min': float(m.range_min), 'range_max': float(m.range_max)}
        if tf is None:
            self.tf_fail += 1
            rec['tf_ok'] = False
        else:
            rec['tf_ok'] = True
            t = np.array([tf.transform.translation.x, tf.transform.translation.y,
                          tf.transform.translation.z])
            qq = tf.transform.rotation
            # 旋转矩阵（xyzw）
            x, y, z, w = qq.x, qq.y, qq.z, qq.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            inc = np.asarray(m.angle_increment, dtype=np.float64)
            ang = m.angle_min + inc * np.arange(r.size)
            pts = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]),
                            np.zeros(n_finite)], axis=1)
            world = pts @ R.T + t
            zz = world[:, 2]
            rec.update({
                'z_mean': float(zz.mean()), 'z_min': float(zz.min()),
                'z_max': float(zz.max()), 'z_p50': float(np.median(zz)),
                'z_sensor_origin': float(t[2]),
                'n_in_band': int(((zz >= Z_LO) & (zz <= Z_HI)).sum()),
                'n_below': int((zz < Z_LO).sum()), 'n_above': int((zz > Z_HI).sum()),
                'band': [Z_LO, Z_HI],
            })
        self.scans.append(rec)

    def _grid_stats(self, g, stamp):
        tf = self._lookup(self.scan_frame, self.base_frame)   # 最新一帧：车静止，与 msg 戳差 <1 mm
        rec = {'frame': g.frame, 'res': g.res, 'sx': g.sx, 'sy': g.sy,
               'n_cells': g.n_cells, 'n_lethal': g.n_lethal,
               'n_inscribed': g.n_inscribed, 'n_free': g.n_free,
               'n_unknown': g.n_unknown}
        if tf is None:
            rec['tf_ok'] = False
            return rec
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        qq = tf.transform.rotation
        yaw = math.atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                         1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z))
        rec.update({'tf_ok': True, 'robot_x': float(rx), 'robot_y': float(ry),
                    'robot_z': float(tf.transform.translation.z), 'robot_yaw': float(yaw),
                    'n_lethal_win': int(g.n_lethal), 'n_free_win': int(g.n_free)})
        rc = g.cell_of(rx, ry)
        rec['robot_cell'] = None if rc is None else int(g.arr[rc[0], rc[1]])
        rec['robot_cell_xy'] = None if rc is None else [int(rc[1]), int(rc[0])]
        d = g.dist_to_nearest_lethal(rx, ry)
        rec['nearest_lethal_m'] = d
        if g.lethal_xy is not None:
            # 车前方窗口（机器人系）：x∈[0,2.0]、|y|≤0.6
            dx = g.lethal_xy[:, 0] - rx
            dy = g.lethal_xy[:, 1] - ry
            c, s = math.cos(-yaw), math.sin(-yaw)
            bx = c * dx - s * dy
            by = s * dx + c * dy
            m = (bx >= 0.0) & (bx <= AHEAD_X) & (np.abs(by) <= AHEAD_Y)
            rec['ahead_window'] = [0.0, AHEAD_X, AHEAD_Y]
            rec['n_lethal_ahead'] = int(m.sum())
            m1 = np.hypot(dx, dy) <= 1.0
            rec['n_lethal_within_1m'] = int(m1.sum())
        else:
            rec['n_lethal_ahead'] = 0
            rec['n_lethal_within_1m'] = 0
        return rec

    def on_grid(self, m):
        if self.t_first_grid is None:
            self.t_first_grid = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.phase != 'window':
            return
        self.n_grid_rx += 1
        try:
            g = Grid(m)
        except Exception as e:
            self.grids.append({'error': repr(e)})
            return
        self.grids.append(self._grid_stats(g, m.header.stamp))

    def on_global_grid(self, m):
        if self.phase != 'window':
            return
        self.n_ggrid_rx += 1
        try:
            g = Grid(m)
        except Exception as e:
            self.grids_global.append({'error': repr(e)})
            return
        self.grids_global.append(self._grid_stats(g, m.header.stamp))

    def on_class_grid(self, m):
        if self.phase != 'window':
            return
        self.n_class_rx += 1
        try:
            g = Grid(m)
        except Exception:
            return
        self.class_grids.append({'frame': g.frame, 'n_lethal': g.n_lethal,
                                 'n_inscribed': g.n_inscribed, 'n_free': g.n_free,
                                 'n_unknown': g.n_unknown, 'n_cells': g.n_cells})

    def step(self):
        if self.sim is None or self.t_first_cloud is None:
            return False
        if self.phase == 'wait':
            ready = (self.t_first_scan is not None and self.t_first_grid is not None)
            if ready and self.sim - self.t_first_cloud >= self.settle:
                self.phase = 'window'
                self.win_start_sim = self.sim
            return False
        if self.phase == 'window' and self.sim - self.win_start_sim >= self.window:
            self.phase = 'done'
            return True
        return False


def med(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    a = np.asarray(vals, dtype=np.float64)
    return {'n': int(a.size), 'median': float(np.median(a)),
            'min': float(a.min()), 'max': float(a.max()), 'mean': float(a.mean())}


def summarize(arr, key):
    return med([x.get(key) for x in arr])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--settle', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=20.0)
    ap.add_argument('--wall-timeout', type=float, default=400.0)
    ap.add_argument('--base-frame', default='base_link_fake')
    # ★ 2026-10-21（契约 §4.11）：量扫描面 z 的帧 = **局部代价图的 global_frame**。
    #   默认档 odom_level；回滚档 `--scan-frame odom`（= 本工具改动前的硬编码行为）。
    ap.add_argument('--scan-frame', default='odom_level')
    ap.add_argument('--costmap-topic', default='/local_costmap/costmap_raw')
    ap.add_argument('--class-topic', default='/local_costmap/costmap')
    ap.add_argument('--global-topic', default='/global_costmap/costmap_raw',
                    help='同时量全局图（默认开；给空串关掉）')
    args = ap.parse_args()
    rclpy.init()
    node = Band(args.settle, args.window, args.base_frame,
                args.costmap_topic, args.class_topic, args.global_topic,
                scan_frame=args.scan_frame)
    t0 = time.time()
    done = False
    try:
        while time.time() - t0 < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.step():
                done = True
                break
    finally:
        grids = node.grids
        out = {'ok': done, 'settle_s': args.settle, 'window_s_sim': args.window,
               'phase_at_end': node.phase,
               'costmap_topic': args.costmap_topic,
               'base_frame': args.base_frame,
               'scan_frame': args.scan_frame,
               'band': [Z_LO, Z_HI], 'ahead_window': [0.0, AHEAD_X, AHEAD_Y],
               'n_scan_rx': node.n_scan_rx, 'n_grid_rx': node.n_grid_rx,
               'n_class_rx': node.n_class_rx, 'tf_fail_scan': node.tf_fail,
               'tf_latest_fallback': node.tf_latest_fallback,
               'tf_fail_total': node.tf_fail_total,
               'scan_frame_ids': sorted(node.scan_frame_ids),
               'scan': {
                   'n_finite': summarize(node.scans, 'n_finite'),
                   'n_beams': summarize(node.scans, 'n_beams'),
                   'z_mean': summarize(node.scans, 'z_mean'),
                   'z_min': summarize(node.scans, 'z_min'),
                   'z_max': summarize(node.scans, 'z_max'),
                   'z_p50': summarize(node.scans, 'z_p50'),
                   'z_sensor_origin': summarize(node.scans, 'z_sensor_origin'),
                   'n_in_band': summarize(node.scans, 'n_in_band'),
                   'n_below': summarize(node.scans, 'n_below'),
                   'n_above': summarize(node.scans, 'n_above'),
                   'per_frame': node.scans[-5:],
               },
               'local_costmap': {
                   'n_lethal': summarize(grids, 'n_lethal'),
                   'n_inscribed': summarize(grids, 'n_inscribed'),
                   'n_free': summarize(grids, 'n_free'),
                   'n_unknown': summarize(grids, 'n_unknown'),
                   'n_cells': summarize(grids, 'n_cells'),
                   'robot_cell': summarize(grids, 'robot_cell'),
                   'nearest_lethal_m': summarize(grids, 'nearest_lethal_m'),
                   'n_lethal_ahead': summarize(grids, 'n_lethal_ahead'),
                   'n_lethal_within_1m': summarize(grids, 'n_lethal_within_1m'),
                   'robot_z': summarize(grids, 'robot_z'),
                   'last': grids[-1] if grids else None,
               },
               'local_costmap_class': node.class_grids[-1] if node.class_grids else None,
               'n_ggrid_rx': node.n_ggrid_rx,
               'global_costmap': {
                   'n_lethal': summarize(node.grids_global, 'n_lethal'),
                   'n_inscribed': summarize(node.grids_global, 'n_inscribed'),
                   'n_free': summarize(node.grids_global, 'n_free'),
                   'n_unknown': summarize(node.grids_global, 'n_unknown'),
                   'n_cells': summarize(node.grids_global, 'n_cells'),
                   'robot_cell': summarize(node.grids_global, 'robot_cell'),
                   'nearest_lethal_m': summarize(node.grids_global, 'nearest_lethal_m'),
                   'n_lethal_ahead': summarize(node.grids_global, 'n_lethal_ahead'),
                   'n_lethal_within_1m': summarize(node.grids_global, 'n_lethal_within_1m'),
                   'robot_z': summarize(node.grids_global, 'robot_z'),
                   'last': node.grids_global[-1] if node.grids_global else None,
               },
               'odom_last': node.odom_xy[-1] if node.odom_xy else None,
               }
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: out[k] for k in ('ok', 'phase_at_end', 'n_scan_rx', 'n_grid_rx',
                                          'scan', 'local_costmap')},
                     indent=1, ensure_ascii=False))
    return 0 if done else 2


if __name__ == '__main__':
    raise SystemExit(main())
