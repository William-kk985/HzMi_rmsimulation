#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""slope_speed_probe.py —— 「前瞻限速 / 可通行性判据」的**逐帧**验收探针（2026-10-07 Phase 4）。

为什么要单独一个（`robot_model_probe.py` 不订阅这两个私有话题）：
  缺陷 ② 的判据是**判据自己吐出来的连续量**，不是点云统计：
    · `/ground_segmentation/slope_speed_stats`（`SlopeSpeedDecision` + 走廊逐格
      `[d, slope, dtan, step]`）⇒ 直接看"车**静止**时有没有假台阶/假坡度、限速有没有被钉在地板"；
    · `/ground_segmentation/traversability_stats`（`FrameStats`）⇒ `self_masked`
      = 本帧被**自击掩膜**剔出建格的点数（掩膜真的在跑的证据）。
  `[slope_speed]` 日志行只在"上限变化 >0.03 或每 2 s"时打 ⇒ 用它统计会**丢掉中间帧**，
  所以这里按帧采（10 Hz 全收）。

用法（与 launch 同一次 bash 调用里跑；隔离约定同 run_robot_model_probe.sh）：
    python3 tools/scripts/regress/slope_speed_probe.py --out x.json \
        [--duration 12] [--drive-seconds 6] [--drive-speed 0.3] [--mask-yaml <path>]
输出：一行 JSON（含 limit / max_step / max_dtan / dmin / near-step 的分布 + 逐帧明细）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def _stats(xs):
    if not xs:
        return None
    xs = sorted(xs)
    def q(p):
        return xs[min(len(xs) - 1, int(p * (len(xs) - 1)))]
    return {'n': len(xs), 'min': round(xs[0], 4), 'p05': round(q(0.05), 4),
            'p50': round(statistics.median(xs), 4), 'p95': round(q(0.95), 4),
            'max': round(xs[-1], 4)}


class Probe(Node):
    def __init__(self, args):
        super().__init__('slope_speed_probe')
        self.args = args
        self.lock = threading.Lock()
        self.frames = []
        self.trav = []
        self.cloud_r = []
        self.cloud_masked = 0
        self.cloud_n = 0
        self.mask_boxes, self.mask_radius, self.mask_zmin = self._load_mask(args.mask_yaml)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, args.speed_stats_topic, self.on_speed, qos)
        self.create_subscription(String, args.trav_stats_topic, self.on_trav, qos)
        self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos)
        self.cmd = self.create_publisher(Twist, args.cmd_topic, 10)

    @staticmethod
    def _load_mask(path):
        boxes, radius, zmin = [], 0.0, -1e9
        if not path or not os.path.isfile(path):
            return boxes, radius, zmin
        txt = open(path).read()
        if 'self_mask_enable: true' not in txt:
            return boxes, 0.0, zmin
        for line in txt.splitlines():
            s = line.split('#')[0].strip()
            if s.startswith('self_mask_radius_m:'):
                radius = float(s.split(':', 1)[1])
            elif s.startswith('self_mask_z_min_m:'):
                zmin = float(s.split(':', 1)[1])
        i = txt.index('self_mask_boxes:')
        j = txt.index(']', i)
        vals = [float(v) for v in
                txt[i + len('self_mask_boxes:'):j].strip().strip('[').split(',') if v.strip()]
        boxes = [vals[k:k + 6] for k in range(0, len(vals), 6)]
        return boxes, radius, zmin

    def on_speed(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.frames.append((time.time(), d))

    def on_trav(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.trav.append(d)

    def on_cloud(self, msg):
        n = 0
        near = 0
        masked = 0
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            n += 1
            if math.hypot(x, y) < 0.12:
                near += 1
            if self.mask_radius > 0 and z >= self.mask_zmin and \
                    (x * x + y * y) <= self.mask_radius ** 2:
                masked += 1
                continue
            for b in self.mask_boxes:
                if b[0] <= x <= b[3] and b[1] <= y <= b[4] and b[2] <= z <= b[5]:
                    masked += 1
                    break
        with self.lock:
            self.cloud_n += n
            self.cloud_r.append(near / max(n, 1))
            self.cloud_masked += masked

    def drive(self, vx, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            m = Twist()
            m.linear.x = vx
            self.cmd.publish(m)
            time.sleep(0.02)
        self.cmd.publish(Twist())

    def report(self):
        with self.lock:
            fr = list(self.frames)
            tv = list(self.trav)
            cr = list(self.cloud_r)
            cn, cm = self.cloud_n, self.cloud_masked
        out = {'n_speed_frames': len(fr), 'n_trav_frames': len(tv),
               'n_cloud_frames': len(cr),
               'cloud_points': cn,
               'cloud_selfhit_frac_lt012': round(statistics.median(cr), 4) if cr else None,
               'cloud_masked_frac': round(cm / cn, 4) if cn else None,
               'mask': {'boxes': len(self.mask_boxes), 'radius_m': self.mask_radius,
                        'z_min_m': round(self.mask_zmin, 4) if self.mask_zmin > -1e8 else None}}
        if tv:
            out['self_masked_per_frame'] = _stats([t.get('self_masked', 0) for t in tv])
            out['trav'] = {k: _stats([t.get(k, 0) for t in tv]) for k in
                           ('points', 'ground_in', 'ground_out', 'obstacle_out',
                            'step_edge', 'corridor_cells', 'corridor_max_step_m')}
        if not fr:
            return out
        lim = [d['limit'] for _, d in fr]
        mx_step = [d['max_step_m'] for _, d in fr]
        mx_dtan = [d['max_dtan_deg'] for _, d in fr]
        dmin = [d['dmin'] for _, d in fr]
        out['limit'] = _stats(lim)
        out['limit_at_floor'] = {'count': sum(1 for x in lim if x < 0.605),
                                 'frac': round(sum(1 for x in lim if x < 0.605) / len(lim), 4),
                                 'floor_mps': None}
        out['corridor_max_step_m'] = _stats(mx_step)
        out['corridor_max_dtan_deg'] = _stats(mx_dtan)
        out['data_min_d'] = _stats(dmin)
        out['why'] = {}
        for _, d in fr:
            out['why'][d['why']] = out['why'].get(d['why'], 0) + 1
        # 近场格（d<0.30 m）的台阶/坡度 —— 缺陷 ② 的直接判据
        near_step, near_slope, n_near = [], [], 0
        for _, d in fr:
            ns = [c[3] for c in d.get('profile', []) if c[0] < 0.30]
            nl = [c[1] for c in d.get('profile', []) if c[0] < 0.30]
            if ns:
                n_near += 1
                near_step.append(max(ns))
                near_slope.append(max(nl))
        out['near_field_d_lt_0.30m'] = {
            'frames_with_near_cells': n_near,
            'max_step_m': _stats(near_step), 'max_slope_deg': _stats(near_slope)}
        # 特征距离：限速是不是被"最近的那个格"主导
        out['feature_d'] = _stats([d['d'] for _, d in fr])
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='')
    ap.add_argument('--duration', type=float, default=12.0)
    ap.add_argument('--ready-timeout', type=float, default=120.0)
    ap.add_argument('--drive-seconds', type=float, default=0.0)
    ap.add_argument('--drive-speed', type=float, default=0.3)
    ap.add_argument('--settle', type=float, default=2.0)
    ap.add_argument('--mask-yaml', default=os.path.join(
        os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')),
        'src', 'rm_nav_bringup', 'config', 'traversability_self_mask_robot11.yaml'))
    ap.add_argument('--speed-stats-topic', default='/ground_segmentation/slope_speed_stats')
    ap.add_argument('--trav-stats-topic', default='/ground_segmentation/traversability_stats')
    ap.add_argument('--cloud-topic', default='/livox/lidar/pointcloud')
    ap.add_argument('--cmd-topic', default='/cmd_vel_chassis')
    a = ap.parse_args()

    rclpy.init()
    node = Probe(a)
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        t0 = time.time()
        while time.time() - t0 < a.ready_timeout:
            with node.lock:
                if node.frames and node.cloud_r:
                    break
            time.sleep(0.2)
        time.sleep(a.settle)
        with node.lock:
            node.frames.clear()
            node.trav.clear()
        print('[slope_speed_probe] 静止窗口 %.1fs …' % a.duration, file=sys.stderr)
        time.sleep(a.duration)
        res = {'stationary': node.report()}
        if a.drive_seconds > 0:
            with node.lock:
                node.frames.clear()
                node.trav.clear()
            print('[slope_speed_probe] 直行 %.1fs @ %.2f m/s …'
                  % (a.drive_seconds, a.drive_speed), file=sys.stderr)
            node.drive(a.drive_speed, a.drive_seconds)
            time.sleep(a.settle)
            res['driving'] = node.report()
    finally:
        try:
            node.cmd.publish(Twist())
        except Exception:
            pass
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    txt = json.dumps(res, ensure_ascii=False, indent=1)
    print('=== SLOPE_SPEED_PROBE_JSON_BEGIN ===')
    print(txt)
    print('=== SLOPE_SPEED_PROBE_JSON_END ===')
    if a.out:
        with open(a.out, 'w') as f:
            f.write(txt)


if __name__ == '__main__':
    main()
