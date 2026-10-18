#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uc_procstats.py —— 「统一输出契约」效率测量的取数器（只订阅 + 只读 /proc，不发任何东西）。

为什么单独一个脚本（而不是改 .tmp_gravtest/grav_probe.py）：那个探针负责**精度**（地面倾角/
TF rpy/点云），本脚本负责**开销**（CPU/RSS/速率/RTF）。两者同时跑、各读各的，互不干扰。

量什么（窗口口径，见 docs/frame_contract_v3_measurements.md）：
  · **窗口**：从「首帧 /cloud_registered 之后 settle 秒」起，持续 window 秒**仿真时间**；
  · 进程 CPU：`/proc/<pid>/stat` 的 utime+stime（字段 14/15，单位 = USER_HZ）在窗口**两端**求差
      - `cpu_s`：窗口内消耗的 CPU 秒（单核口径）
      - `%CPU` = cpu_s / 窗口墙钟秒 × 100（与 top 单核口径一致）
      - `ms/frame` = cpu_s × 1000 / 窗口内 /cloud_registered 帧数（= LIO 处理的雷达帧数）
      - 归一化到仿真秒：`cpu_per_sim_s` = cpu_s / 窗口仿真秒
  · RSS：`VmHWM`（峰值常驻，kB → MB）；`VmRSS` 同时取一份
  · 速率：`/cloud_registered`、`/odom`、`/scan`、`/clock` 在窗口内的条数 / 窗口仿真秒
  · RTF：窗口内仿真秒 / 窗口墙钟秒
  · 初始化耗时：首个 `/clock`（≈ 世界开跑）→ 首个 `/odom`（LIO 开始出数）的**仿真**秒与**墙钟**秒
  · 全程 CPU：进程自启动到窗口结束的累计 CPU（含初始化/200 帧门），除以「窗口结束时的仿真秒」
    与「总帧数」⇒ 另一套口径（把初始化也算进去），与窗口口径并列报告。

用法（由 run_uc.sh 起，见那里）：
  uc_procstats.py --out proc.json --settle 20 --window 20 --pgid <launch pgid> \
      --watch small_point_lio,gzserver,lio_tf_adapter --launch-epoch 1699999999.5
"""
import argparse
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2

CLK_TCK = os.sysconf('SC_CLK_TCK')


def stat_cpu_s(pid):
    """utime+stime（秒）。comm 里可能有空格 ⇒ 从最后一个 ')' 之后切。"""
    try:
        s = open('/proc/%d/stat' % pid).read()
    except OSError:
        return None
    rest = s[s.rfind(')') + 1:].split()
    # rest[0] = state（字段 3）⇒ 字段 14 (utime) = rest[11]，字段 15 (stime) = rest[12]
    return (int(rest[11]) + int(rest[12])) / float(CLK_TCK)


def stat_rss(pid):
    """(VmRSS_MB, VmHWM_MB)；读不到返回 (None, None)。"""
    rss = hwm = None
    try:
        for line in open('/proc/%d/status' % pid):
            if line.startswith('VmRSS:'):
                rss = float(line.split()[1]) / 1024.0
            elif line.startswith('VmHWM:'):
                hwm = float(line.split()[1]) / 1024.0
    except OSError:
        pass
    return rss, hwm


def group_pids(pgid):
    """进程组里的 (pid, comm)。comm 会被内核截到 15 字符 ⇒ 调用方按前缀匹配。"""
    out = []
    try:
        txt = subprocess.run(['ps', '-o', 'pid=,comm=', '-g', str(pgid)],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return out
    for line in txt.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].isdigit():
            out.append((int(parts[0]), parts[1]))
    return out


def pick(pids, prefix):
    for pid, comm in pids:
        if comm.startswith(prefix):
            return pid, comm
    return None, None


class Stats(Node):
    def __init__(self, settle, window, pgid, watch, launch_epoch):
        super().__init__('uc_procstats')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.settle = settle
        self.window = window
        self.pgid = pgid
        self.watch = watch
        self.launch_epoch = launch_epoch
        self.sim = None
        self.first_clock = None
        self.first_odom = None
        self.cloud_stamps = []
        self.n_odom = 0
        self.n_cloud = 0
        self.n_scan = 0
        self.n_clock = 0
        self.t_first_cloud_sim = None
        self.odom_frame_ids = set()
        self.odom_child_ids = set()
        self.cloud_frame_ids = set()
        self.scan_frames = set()
        self.phase = 'wait_cloud'
        self.win = {}
        self.win_start = None
        self.win_end = None
        q = qos_profile_sensor_data
        self.create_subscription(Clock, '/clock', self.on_clock, q)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, q)
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self.on_raw, q)
        self.create_subscription(Odometry, '/odom', self.on_odom, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, q)

    # ---------- 采集 ----------
    def on_clock(self, m):
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        self.sim = sim
        self.n_clock += 1
        if self.first_clock is None:
            self.first_clock = (sim, time.time())

    def on_cloud(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.n_cloud += 1
        self.cloud_frame_ids.add(m.header.frame_id)
        if self.t_first_cloud_sim is None:
            self.t_first_cloud_sim = t
        if self.phase in ('window',):
            self.cloud_stamps.append(t)

    def on_raw(self, m):
        pass

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.n_odom += 1
        self.odom_frame_ids.add(m.header.frame_id)
        self.odom_child_ids.add(m.child_frame_id)
        if self.first_odom is None:
            self.first_odom = (t, time.time())

    def on_scan(self, m):
        self.n_scan += 1
        self.scan_frames.add(m.header.frame_id)

    # ---------- 状态机 ----------
    def snapshot(self):
        pids = group_pids(self.pgid)
        snap = {'wall': time.time(), 'sim': self.sim,
                'n_cloud': self.n_cloud, 'n_odom': self.n_odom, 'n_scan': self.n_scan,
                'procs': {}}
        for prefix in self.watch:
            pid, comm = pick(pids, prefix)
            if pid is None:
                snap['procs'][prefix] = None
                continue
            rss, hwm = stat_rss(pid)
            snap['procs'][prefix] = {'pid': pid, 'comm': comm,
                                     'cpu_s': stat_cpu_s(pid),
                                     'rss_mb': rss, 'hwm_mb': hwm}
        return snap

    def step(self):
        """每轮 spin 调一次；返回 True 表示可以收工。"""
        if self.sim is None or self.t_first_cloud_sim is None:
            return False
        if self.phase == 'wait_cloud':
            if self.sim - self.t_first_cloud_sim >= self.settle:
                self.win_start = self.snapshot()
                self.phase = 'window'
                self.cloud_stamps = []
            return False
        if self.phase == 'window':
            if self.sim - self.win_start['sim'] >= self.window:
                self.win_end = self.snapshot()
                self.phase = 'done'
                return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--settle', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=20.0)
    ap.add_argument('--pgid', type=int, required=True)
    ap.add_argument('--watch', default='small_point_lio,gzserver,lio_tf_adapter')
    ap.add_argument('--launch-epoch', type=float, default=0.0)
    ap.add_argument('--wall-timeout', type=float, default=320.0)
    args = ap.parse_args()

    watch = [w.strip() for w in args.watch.split(',') if w.strip()]
    rclpy.init()
    node = Stats(args.settle, args.window, args.pgid, watch, args.launch_epoch)
    t0 = time.time()
    done = False
    try:
        while time.time() - t0 < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.step():
                done = True
                break
    finally:
        out = {'ok': done, 'settle_s': args.settle, 'window_s_sim': args.window,
               'watch': watch, 'pgid': args.pgid,
               'phase_at_end': node.phase}
        out['first_clock'] = None if node.first_clock is None else {
            'sim': node.first_clock[0],
            'wall_rel': node.first_clock[1] - args.launch_epoch if args.launch_epoch else None}
        out['first_odom'] = None if node.first_odom is None else {
            'sim': node.first_odom[0],
            'wall_rel': node.first_odom[1] - args.launch_epoch if args.launch_epoch else None}
        if node.first_clock and node.first_odom:
            out['init'] = {
                'sim_s_from_clock0': node.first_odom[0] - node.first_clock[0],
                'wall_s_from_launch': (node.first_odom[1] - args.launch_epoch
                                       if args.launch_epoch else None)}
        if node.win_start and node.win_end:
            a, b = node.win_start, node.win_end
            sim_s = b['sim'] - a['sim']
            wall_s = b['wall'] - a['wall']
            out['window'] = {
                'sim_s': sim_s, 'wall_s': wall_s,
                'rtf': (sim_s / wall_s) if wall_s > 0 else None,
                'n_cloud': b['n_cloud'] - a['n_cloud'],
                'n_odom': b['n_odom'] - a['n_odom'],
                'n_scan': b['n_scan'] - a['n_scan'],
                'cloud_hz': (b['n_cloud'] - a['n_cloud']) / sim_s if sim_s > 0 else None,
                'odom_hz': (b['n_odom'] - a['n_odom']) / sim_s if sim_s > 0 else None,
                'scan_hz': (b['n_scan'] - a['n_scan']) / sim_s if sim_s > 0 else None,
                'procs': {},
            }
            # 「全程」口径的分母：自首个 /clock 起的**已流逝仿真秒**（不是绝对仿真时钟）
            sim_elapsed = (b['sim'] - node.first_clock[0]) if node.first_clock else b['sim']
            out['window']['sim_elapsed_since_clock0'] = sim_elapsed
            for prefix in watch:
                pa, pb = a['procs'].get(prefix), b['procs'].get(prefix)
                if not pa or not pb or pa['cpu_s'] is None or pb['cpu_s'] is None:
                    out['window']['procs'][prefix] = None
                    continue
                cpu = pb['cpu_s'] - pa['cpu_s']
                frames = b['n_cloud'] - a['n_cloud']
                out['window']['procs'][prefix] = {
                    'pid': pb['pid'], 'comm': pb['comm'],
                    'cpu_s': cpu,
                    'cpu_pct': (cpu / wall_s * 100.0) if wall_s > 0 else None,
                    'cpu_per_sim_s': (cpu / sim_s) if sim_s > 0 else None,
                    'ms_per_frame': (cpu * 1000.0 / frames) if frames > 0 else None,
                    'hwm_mb': pb['hwm_mb'], 'rss_mb': pb['rss_mb'],
                    'whole_run_cpu_s': pb['cpu_s'],
                    'whole_run_cpu_per_sim_s': (pb['cpu_s'] / sim_elapsed) if sim_elapsed > 0 else None,
                    'whole_run_ms_per_frame': (pb['cpu_s'] * 1000.0 / b['n_cloud']
                                               if b['n_cloud'] else None),
                }
        out['frame_ids'] = {'odom': sorted(node.odom_frame_ids),
                            'odom_child': sorted(node.odom_child_ids),
                            'cloud': sorted(node.cloud_frame_ids),
                            'scan': sorted(node.scan_frames)}
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: out.get(k) for k in ('ok', 'init', 'first_odom', 'window')},
                     indent=1, ensure_ascii=False))
    return 0 if done else 2


if __name__ == '__main__':
    sys.exit(main())
