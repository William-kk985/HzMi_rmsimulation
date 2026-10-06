#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑图时同时间轴记录"跳变 / 打滑 / 卡死 / /scan 内容"的旁路探针（离线分析用）。

给 `tools/scripts/mapping/coverage_drive.py` 当**并行进程**用：drive 负责开车+落盘，
本探针只订阅、只记录，一次跑完把同一时间轴上的这些量交出来：

| 记录 | 话题/来源 | 用途（对应的问题） |
|---|---|---|
| `/odom`（LIO）位姿 + z/roll/pitch | `/odom` | LIO 有没有 z 漂/俯仰漂（"图歪了"的几何来源） |
| `/odom_ground_truth` 位姿 + z | `/odom_ground_truth` | 底盘**物理**有没有被顶起来/弹跳（真值 z 突变） |
| `map→odom`（slam_toolbox） | `/tf` | SLAM 位姿单步跳变 = 图被"掰"的那一下 |
| `/scan` 全量 ranges + frame_id | `/scan` | 坡道段有没有冒出一道**横穿走廊的假墙**；帧号是什么 |
| `/cmd_vel_chassis` | `/cmd_vel_chassis` | "发了速度但不动" ⇒ 物理卡死（不是控制没发） |
| `/map` 已知/占用格数 | `/map` | 边建边长 + 存图前的面积 |

**为什么不用 `ros2 topic echo` 拼**：本仓踩过 `/tf` 80 Hz 饿死单线程 `spin_once` 的坑
（docs/worlds.md §5.1）⇒ 这里用 `MultiThreadedExecutor` + 独立回调组，
并且位姿一律**从话题**取（不 lookup TF），从根上避开那类饥饿。

用法（栈已经在跑）：
    python3 tools/scripts/mapping/ramp_section_probe.py --out /tmp/ramp/probe \
        --duration 300 --sector-deg 60 --sector-at 0
退出码：0 正常；2 一个话题都没收到。
"""
from __future__ import annotations

import argparse
import json
import math
import os
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
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

BE = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
MAP_Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def rpy_of(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class Probe(Node):
    def __init__(self, a):
        super().__init__('ramp_section_probe')
        self.a = a
        self.lock = threading.RLock()   # RLock：summarize() 会在持锁时调用 find_stuck()（也加锁）
        self.grp = ReentrantCallbackGroup()
        self.t = {'odom': [], 'gt': [], 'mapodom': [], 'scan': [], 'cmd': [], 'cloud': []}
        self.scan_ranges = []      # list[list[float32]]
        self.scan_meta = []        # list[(stamp, frame_id)]
        self.map_hist = []
        self.n_scan = 0
        # ⚠️ **故意不订阅** `/livox/lidar/pointcloud` 与 `/map`：8 Hz 的大点云回调会把
        #    executor 的线程占满，实测会让"到点收尾"那条 timer 回调迟迟拿不到锁 ⇒ 落盘卡死。
        #    点云频率/地图增长 coverage_drive 已经记了，这里不重复。
        for name, typ, qos in (('/odom', Odometry, BE),
                               ('/odom_ground_truth', Odometry, BE),
                               ('/scan', LaserScan, BE),
                               ('/cmd_vel_chassis', Twist, BE)):
            self.create_subscription(typ, name,
                                     lambda m, n=name: self.on_msg(n, m), qos,
                                     callback_group=self.grp)
        self.create_subscription(TFMessage, '/tf', self.on_tf, QoSProfile(depth=200),
                                 callback_group=self.grp)
        self.get_logger().info('ramp probe 就绪：记录 /odom /odom_ground_truth /scan /tf /cmd_vel_chassis /map')

    # ------------------------------------------------------------------ helpers
    def pose(self, m: Odometry):
        p = m.pose.pose.position
        r, pi, y = rpy_of(m.pose.pose.orientation)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return [t, p.x, p.y, p.z, r, pi, y]

    def on_msg(self, name, m):
        with self.lock:
            if name == '/odom':
                self.t['odom'].append(self.pose(m))
            elif name == '/odom_ground_truth':
                self.t['gt'].append(self.pose(m))
            elif name == '/cmd_vel_chassis':
                self.t['cmd'].append([m.linear.x, m.linear.y, m.angular.z])
            elif name == '/livox/lidar/pointcloud':
                self.t['cloud'].append([m.width * m.height])
            elif name == '/scan':
                self.n_scan += 1
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                rng = np.asarray(m.ranges, dtype=np.float32)
                finite = np.isfinite(rng) & (rng > m.range_min) & (rng < m.range_max)
                self.scan_meta.append([t, m.header.frame_id, m.angle_min,
                                       m.angle_increment, len(rng),
                                       int(finite.sum()),
                                       float(rng[finite].min()) if finite.any() else -1.0])
                self.scan_ranges.append(rng)
            elif name == '/map':
                d = np.asarray(m.data, dtype=np.int8)
                self.map_hist.append([len(d), int((d == 0).sum()),
                                      int((d >= 65).sum()), int((d < 0).sum())])

    def on_tf(self, msg: TFMessage):
        with self.lock:
            for tr in msg.transforms:
                if tr.header.frame_id == self.a.parent_frame and tr.child_frame_id == self.a.child_frame:
                    t = tr.transform.translation
                    r, pi, y = rpy_of(tr.transform.rotation)
                    self.t['mapodom'].append(
                        [tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                         t.x, t.y, t.z, r, pi, y])

    # ------------------------------------------------------------------ report
    def summarize(self):
        out = {'counts': {}, 'rate_hz': {}}
        with self.lock:
            for k, v in self.t.items():
                out['counts'][k] = len(v)
            if len(self.t['odom']) > 1:
                a = np.asarray(self.t['odom'])
                gt = np.asarray(self.t['gt']) if self.t['gt'] else None
                span = a[-1, 0] - a[0, 0]
                out['rate_hz']['odom'] = round(len(a) / span, 2) if span > 0 else None
                if gt is not None and len(gt) > 1:
                    out['rate_hz']['gt'] = round(len(gt) / (gt[-1, 0] - gt[0, 0]), 2)
                    # 按时间配对（最近邻）
                    idx = np.searchsorted(gt[:, 0], a[:, 0]).clip(1, len(gt) - 1)
                    pair = gt[idx]
                    e = a[:, 1:4] - pair[:, 1:4]
                    out['lio_vs_gt'] = {
                        'n_pairs': int(len(a)),
                        'z_err_mean': round(float(e[:, 2].mean()), 4),
                        'z_err_std': round(float(e[:, 2].std()), 4),
                        'z_err_ptp': round(float(np.ptp(e[:, 2])), 4),
                        'xy_rmse': round(float(np.sqrt((e[:, 0] ** 2 + e[:, 1] ** 2).mean())), 4),
                    }
                out['lio_z'] = {'min': round(float(a[:, 3].min()), 4),
                                'max': round(float(a[:, 3].max()), 4),
                                'ptp': round(float(np.ptp(a[:, 3])), 4),
                                'std': round(float(a[:, 3].std()), 4)}
                out['lio_pitch_deg'] = {'min': round(float(np.degrees(a[:, 5].min())), 2),
                                        'max': round(float(np.degrees(a[:, 5].max())), 2),
                                        'ptp': round(float(np.degrees(np.ptp(a[:, 5]))), 2)}
                out['lio_roll_deg'] = {'min': round(float(np.degrees(a[:, 4].min())), 2),
                                       'max': round(float(np.degrees(a[:, 4].max())), 2),
                                       'ptp': round(float(np.degrees(np.ptp(a[:, 4]))), 2)}
            if self.t['mapodom']:
                mo = np.asarray(self.t['mapodom'])
                d = np.hypot(np.diff(mo[:, 1]), np.diff(mo[:, 2]))
                dy = np.abs(np.diff(np.unwrap(mo[:, 6])))
                out['map_odom'] = {
                    'n': int(len(mo)),
                    'x_range': [round(float(mo[:, 1].min()), 3), round(float(mo[:, 1].max()), 3)],
                    'y_range': [round(float(mo[:, 2].min()), 3), round(float(mo[:, 2].max()), 3)],
                    'z_range': [round(float(mo[:, 3].min()), 3), round(float(mo[:, 3].max()), 3)],
                    'step_xy_p99': round(float(np.percentile(d, 99)), 4) if len(d) else None,
                    'step_xy_max': round(float(d.max()), 4) if len(d) else None,
                    'step_yaw_p99_deg': round(float(np.degrees(np.percentile(dy, 99))), 3) if len(dy) else None,
                    'step_yaw_max_deg': round(float(np.degrees(dy.max())), 3) if len(dy) else None,
                    'n_step_gt_0.20m': int((d > 0.20).sum()),
                    'n_step_gt_5deg': int((np.degrees(dy) > 5.0).sum()),
                }
            if self.scan_meta:
                sm = np.asarray([[r[0], r[5], r[6]] for r in self.scan_meta], dtype=np.float64)
                out['scan'] = {
                    'n': int(len(sm)),
                    'frame_ids': sorted({r[1] for r in self.scan_meta}),
                    'n_beams': int(self.scan_meta[0][4]),
                    'finite_beams_mean': round(float(sm[:, 1].mean()), 1),
                    'min_range_mean': round(float(sm[:, 2].mean()), 3),
                    'min_range_min': round(float(sm[:, 2].min()), 3),
                    'sample_period_p99': round(float(np.percentile(np.diff(sm[:, 0]), 99)), 4)
                    if len(sm) > 1 else None,
                }
            if self.t['cmd']:
                c = np.asarray(self.t['cmd'])
                out['cmd'] = {'n': int(len(c)),
                              'max_vx': round(float(np.abs(c[:, 0]).max()), 3),
                              'max_wz': round(float(np.abs(c[:, 2]).max()), 3)}
            if self.map_hist:
                out['map'] = {'n_updates': len(self.map_hist),
                              'last_cells': self.map_hist[-1][0],
                              'last_free': self.map_hist[-1][1],
                              'last_occupied': self.map_hist[-1][2],
                              'last_unknown': self.map_hist[-1][3]}
        # ⚠️ 这一行**必须在 with self.lock 之外**：find_stuck() 自己也加锁，
        #    普通 Lock 下就是自死锁（实测症状：看门线程打印 "flush: summarize ..." 后永久卡住，
        #    JSON/NPZ 全部丢失）。双保险：锁也换成 RLock。
        out['stuck_windows'] = self.find_stuck()
        return out

    def find_stuck(self, window=3.0, eps=0.03):
        """发着速度但 LIO 位移 ~0 的窗口（物理卡死的判据）。"""
        res = []
        with self.lock:
            a = np.asarray(self.t['odom']) if self.t['odom'] else None
            c = np.asarray(self.t['cmd']) if self.t['cmd'] else None
        if a is None or len(a) < 10:
            return res
        i = 0
        while i < len(a) - 1:
            j = np.searchsorted(a[:, 0], a[i, 0] + window)
            if j >= len(a):
                break
            # 窗口内净位移
            step = np.hypot(a[i + 1:j + 1, 1] - a[i, 1], a[i + 1:j + 1, 2] - a[i, 2])
            moved = float(np.nansum(step)) if len(step) else 0.0
            if moved < eps:
                res.append({'t0': round(float(a[i, 0]), 2), 't1': round(float(a[j - 1, 0]), 2),
                            'path_len': round(moved, 4),
                            'pose': [round(float(a[i, 1]), 2), round(float(a[i, 2]), 2)],
                            'cmd_seen': bool(c is not None and len(c)),
                            'cmd_max_abs': round(float(np.abs(c).max()), 3) if c is not None and len(c) else None})
                i = j
            else:
                i += 1
        return res


def flush(node, args, t0, rc=0):
    """把已记录的数据落盘 + 打印摘要。**必须能从 timer 回调里直接调用**。"""
    summ = node.summarize()
    summ['wall_seconds'] = round(time.time() - t0, 1)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    with open(args.out + '.json', 'w') as f:
        json.dump(summ, f, ensure_ascii=False, indent=1)
    arrs = {}
    for k, v in node.t.items():
        if v:
            arrs[k] = np.asarray(v, dtype=np.float64)
    if node.scan_ranges:
        L = max(len(r) for r in node.scan_ranges)
        M = np.full((len(node.scan_ranges), L), np.nan, dtype=np.float32)
        for i, r in enumerate(node.scan_ranges):
            M[i, :len(r)] = r
        arrs['scan_ranges'] = M
        arrs['scan_meta'] = np.asarray(node.scan_meta, dtype=object)
    np.savez(args.out + '.npz', **arrs)
    print('PROBEJSON ' + json.dumps({k: v for k, v in summ.items() if k != 'stuck_windows'},
                                    ensure_ascii=False), flush=True)
    print('PROBESTUCK ' + json.dumps(summ['stuck_windows'][:20], ensure_ascii=False), flush=True)
    if summ['counts'].get('odom', 0) == 0:
        rc = 2
    return rc


def main():
    ap = argparse.ArgumentParser(description='跑图旁路探针（跳变/打滑/卡死//scan 内容）')
    ap.add_argument('--out', required=True, help='输出前缀（写 <out>.json 与 <out>.npz）')
    ap.add_argument('--duration', type=float, default=0.0, help='记录秒数（0=直到 Ctrl-C）')
    ap.add_argument('--parent-frame', default='map')
    ap.add_argument('--child-frame', default='odom')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Probe(args)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    t0 = time.time()

    # 到点收尾**必须由一个不参与 executor 的纯 Python 线程做**。踩过两次坑：
    #   ① `while rclpy.ok(): ex.spin_once()` —— 回调积压时 spin_once 迟迟不返回，脚本卡死；
    #   ② 在 timer 回调里 `rclpy.shutdown()` 或直接落盘 —— `ex.spin()` 不会醒，
    #      或者在 executor 线程里抢锁/做几十 MB I/O ⇒ 落盘卡住，JSON/NPZ 全丢。
    # 看门线程 + 逐阶段 print + 外面 GNU timeout 兜底 = 这条链路现在不会静默失败。
    def guardian():
        deadline = t0 + (float(args.duration) if args.duration > 0 else 1e9)
        while time.time() < deadline:
            time.sleep(0.1)
        node.get_logger().info('记录 %.1f s 到点，看门线程开始落盘' % args.duration)
        rc = 0
        try:
            print('[probe] flush: summarize ...', flush=True)
            rc = flush(node, args, t0)
            print('[probe] flush: done rc=%s' % rc, flush=True)
        except Exception:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            rc = 1
        os._exit(rc)

    if args.duration > 0:
        threading.Thread(target=guardian, daemon=True).start()
    try:
        ex.spin()
    except Exception as e:  # noqa: BLE001
        print('[probe] spin 结束：%r' % (e,), flush=True)
    rc = flush(node, args, t0)
    try:
        ex.shutdown(); node.destroy_node(); rclpy.shutdown()
    except Exception:
        pass
    return rc


if __name__ == '__main__':
    sys.exit(main())
