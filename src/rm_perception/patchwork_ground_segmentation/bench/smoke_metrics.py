#!/usr/bin/env python3
"""地面分割槽位整栈 smoke 的**只读**指标采集器（不改任何话题/参数）。

采集（写成一个 JSON）：
  · 各话题频率：/livox/lidar/pointcloud、/segmentation/obstacle、/scan、/clock
  · /scan 内容：有限回波比例、距离分位、**近距离（<1.5 m）有限回波占比**（"假墙"的客观代理：
    地面被误判成障碍时，机器人周围会出现一圈短距回波）
  · /map（最后一帧）：占用/空闲/未知格数
  · RTF：(仿真钟推进)/(墙钟推进)
  · CPU：按进程名采样 /proc/<pid>/stat 的 utime+stime，给出**平均核占用**

用法：python3 smoke_metrics.py --out res.json --seconds 90 --proc ground_segmentation_node ...
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=5)


class Metrics(Node):
    def __init__(self, procs):
        super().__init__('ground_slot_smoke_metrics')
        self.procs = procs
        self.n = {'pcloud': 0, 'obstacle': 0, 'ground': 0, 'scan': 0}
        self.t0 = None
        self.t_last = {}
        self.rates = {}
        self.scan_stats = []
        self.sim_t0 = None
        self.sim_t_last = None
        self.wall_t0 = time.time()
        self.last_map = None
        self.map_msgs = 0
        self.cpu_samples = []

        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud',
                                 lambda m: self._tick('pcloud'), SENSOR_QOS)
        self.create_subscription(PointCloud2, '/segmentation/obstacle',
                                 lambda m: self._tick('obstacle'), SENSOR_QOS)
        self.create_subscription(PointCloud2, '/segmentation/ground',
                                 lambda m: self._tick('ground'), SENSOR_QOS)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SENSOR_QOS)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        # /clock 是 BEST_EFFORT（gzserver 的 QoS）；用 RELIABLE 订阅会直接不兼容、收不到任何消息
        self.create_subscription(Clock, '/clock', self.on_clock, SENSOR_QOS)
        self.create_timer(1.0, self.sample_cpu)

    def _tick(self, k):
        self.n[k] += 1

    def on_scan(self, msg):
        self.n['scan'] += 1
        rng = [r for r in msg.ranges if r == r and r != float('inf') and r > 0.0]
        if not rng:
            return
        rng.sort()
        near = sum(1 for r in rng if r < 1.5)
        self.scan_stats.append({
            'n_finite': len(rng),
            'n_total': len(msg.ranges),
            'r_min': rng[0],
            'r_p50': rng[len(rng) // 2],
            'r_p95': rng[int(len(rng) * 0.95)],
            'n_near_1p5': near,
        })

    def on_map(self, msg):
        self.last_map = msg
        self.map_msgs += 1

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_t0 is None:
            self.sim_t0 = t
        self.sim_t_last = t

    def sample_cpu(self):
        now = time.time()
        for k, v in list(self.n.items()):
            prev_t, prev_n = self.t_last.get(k, (self.wall_t0, 0))
            if now - prev_t >= 1.0:
                self.rates.setdefault(k, []).append((v - prev_n) / (now - prev_t))
                self.t_last[k] = (now, v)
        s = {}
        for name in self.procs:
            tot = 0.0
            for pid in os.listdir('/proc'):
                if not pid.isdigit():
                    continue
                try:
                    with open('/proc/%s/cmdline' % pid, 'rb') as f:
                        cmd = f.read().decode('utf-8', 'replace')
                    if name not in cmd:
                        continue
                    with open('/proc/%s/stat' % pid) as f:
                        parts = f.read().split()
                    tot += (int(parts[13]) + int(parts[14])) / os.sysconf('SC_CLK_TCK')
                except (OSError, IndexError, ValueError):
                    continue
            s[name] = tot
        self.cpu_samples.append((now, s))

    def report(self):
        wall = time.time() - self.wall_t0
        rtf = ((self.sim_t_last - self.sim_t0) / wall) if (self.sim_t0 is not None
                                                          and self.sim_t_last) else None
        out = {
            'wall_sec': wall,
            'rtf': rtf,
            'counts': self.n,
            'hz': {k: (sum(v) / len(v) if v else 0.0) for k, v in self.rates.items()},
            'scan': {},
            'map': {},
            'cpu_cores': {},
        }
        if self.scan_stats:
            n = len(self.scan_stats)
            out['scan'] = {
                'frames': n,
                'n_finite_mean': sum(s['n_finite'] for s in self.scan_stats) / n,
                'n_total_mean': sum(s['n_total'] for s in self.scan_stats) / n,
                'finite_frac_mean': sum(s['n_finite'] / max(1, s['n_total'])
                                        for s in self.scan_stats) / n,
                'near_1p5_mean': sum(s['n_near_1p5'] for s in self.scan_stats) / n,
                'near_1p5_frac_of_finite_mean':
                    sum(s['n_near_1p5'] / max(1, s['n_finite']) for s in self.scan_stats) / n,
                'r_min_mean': sum(s['r_min'] for s in self.scan_stats) / n,
                'r_p50_mean': sum(s['r_p50'] for s in self.scan_stats) / n,
                'r_p95_mean': sum(s['r_p95'] for s in self.scan_stats) / n,
            }
        if self.last_map is not None:
            d = self.last_map.data
            out['map'] = {
                'msgs': self.map_msgs,
                'width': self.last_map.info.width,
                'height': self.last_map.info.height,
                'resolution': self.last_map.info.resolution,
                'occupied': sum(1 for v in d if v >= 65),
                'free': sum(1 for v in d if 0 <= v < 65),
                'unknown': sum(1 for v in d if v < 0),
                'known': sum(1 for v in d if v >= 0),
                'total': len(d),
            }
        if len(self.cpu_samples) >= 2:
            t0, s0 = self.cpu_samples[0]
            t1, s1 = self.cpu_samples[-1]
            dt = t1 - t0
            for name in self.procs:
                if dt > 0:
                    out['cpu_cores'][name] = (s1.get(name, 0.0) - s0.get(name, 0.0)) / dt
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--proc', action='append', default=[])
    args = ap.parse_args()

    rclpy.init()
    node = Metrics(args.proc)
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rep = node.report()
    with open(args.out, 'w') as f:
        json.dump(rep, f, indent=2)
    print(json.dumps(rep, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
