#!/usr/bin/env python3
"""cloud_z_profile.py —— 判定「stvl 近处看不到 / 墙碎块」到底是
   (A) 高度阈值把近处回波裁掉了，还是 (B) stvl 内部（清除/体素/射程参数）。

做法：订阅 stvl 的**输入** `/segmentation/obstacle`（以及 `/scan` 作对照），
按「到机器人的水平距离」分桶，统计每桶的点数与 z 的最小/中位/最大。
z 已折算到 map 系：z_map ≈ z_livox + sensor_height（默认 0.226，见 --sensor-height），
因为 stvl 的 min_obstacle_height/max_obstacle_height 就是在该系下判的。

判读（关键在"近距桶 vs 远距桶"的对比）：
  · 近距桶(<1m)里 z_map ≥ 0.2 的点**几乎没有**，而远距桶里有 ⇒ (A) 成立：
    固定仰角射线 + 距离 ⇒ 近处只打到墙的下半截，被高度阈值裁掉（改 min_obstacle_height 可救）。
  · 近距桶里明明有大量 z_map ≥ 0.2 的点，而 stvl 仍不显示 ⇒ (B)：问题在 stvl 的
    clearing / raytrace_*_range / voxel_decay / mark_threshold（清掉了，或没标）。

用法（先贴墙站 20s，再退到 3m 外站 20s，对比两次输出）：
  python3 tools/scripts/diag/cloud_z_profile.py --duration 20
  python3 tools/scripts/diag/cloud_z_profile.py --duration 20 --sensor-height 0.226
"""
import argparse
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
BUCKETS = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 10.0), (10.0, 1e9)]


def iter_xyz(msg: PointCloud2, stride=1):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in ('x', 'y', 'z')):
        return
    step, data = msg.point_step, msg.data
    for i in range(0, msg.width * msg.height, stride):
        b = i * step
        try:
            yield (struct.unpack_from('<f', data, b + off['x'])[0],
                   struct.unpack_from('<f', data, b + off['y'])[0],
                   struct.unpack_from('<f', data, b + off['z'])[0])
        except struct.error:
            return


def bucket_of(d):
    for lo, hi in BUCKETS:
        if lo <= d < hi:
            return (lo, hi)
    return None


class Prof(Node):
    def __init__(self, h):
        super().__init__("cloud_z_profile")
        self.h = h
        self.cloud = {b: {"n": 0, "zmin": 1e9, "zmax": -1e9, "zs": [], "hi": 0} for b in BUCKETS}
        self.cloud_frames, self.cloud_n = set(), 0
        self.scan = {b: {"finite": 0, "total": 0} for b in BUCKETS}
        self.create_subscription(PointCloud2, "/segmentation/obstacle", self.on_cloud, BEST)
        self.create_subscription(LaserScan, "/scan", self.on_scan, BEST)

    def on_cloud(self, m):
        self.cloud_n += 1
        self.cloud_frames.add(m.header.frame_id)
        for x, y, z in iter_xyz(m, stride=2):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            b = bucket_of(math.hypot(x, y))
            if b is None:
                continue
            zm = z + self.h                      # 折算到 map 系（地面 z=0）
            e = self.cloud[b]
            e["n"] += 1
            e["zmin"] = min(e["zmin"], zm)
            e["zmax"] = max(e["zmax"], zm)
            e["hi"] += 1 if zm >= 0.2 else 0     # 会被 "min_obstacle_height: 0.2" 留下的点
            if len(e["zs"]) < 4000:
                e["zs"].append(zm)

    def on_scan(self, m):
        inc = m.angle_increment
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * inc
            d = math.hypot(r * math.cos(a), r * math.sin(a)) if math.isfinite(r) else None
            b = bucket_of(d) if d is not None else None
            if b is None:
                continue
            self.scan[b]["total"] += 1
            if math.isfinite(r) and r > 0:
                self.scan[b]["finite"] += 1

    def table(self):
        print(f"\n  stvl 输入=/segmentation/obstacle  帧数={self.cloud_n}  frame={sorted(self.cloud_frames)}  "
              f"z=livox_frame+{self.h}")
        print(f"  {'距离桶(m)':>12} {'点数':>8} {'z_map最小':>10} {'z_map中位':>10} {'z_map最大':>10} "
              f"{'z>=0.2占比':>11} {'/scan 有回波占比':>16}")
        first_break = None
        for b in BUCKETS:
            c = self.cloud[b]
            if c["n"] == 0:
                print(f"  {f'{b[0]}-{b[1] if b[1]<1e8 else \"inf\"}':>12} {'0':>8} {'--':>10} {'--':>10} {'--':>10} {'--':>11} "
                      f"{self._scan_pct(b):>16}")
                continue
            zs = sorted(c["zs"])
            med = zs[len(zs) // 2]
            pct = 100.0 * c["hi"] / max(1, c["n"])
            print(f"  {f'{b[0]}-{b[1] if b[1]<1e8 else \"inf\"}':>12} {c['n']:>8} {c['zmin']:>10.3f} {med:>10.3f} "
                  f"{c['zmax']:>10.3f} {pct:>10.1f}% {self._scan_pct(b):>16}")
            if first_break is None and pct < 5.0 and c["n"] > 50:
                first_break = b
        # 自动判读
        near = [b for b in BUCKETS if b[0] < 1.0]
        n_near = sum(self.cloud[b]["n"] for b in near)
        hi_near = sum(self.cloud[b]["hi"] for b in near)
        ratio = (100.0 * hi_near / n_near) if n_near else -1
        print(f"\n  【判读】近距桶(<1m) 点数={n_near}  其中 z_map>=0.2 占比={ratio:.1f}%")
        if ratio >= 0 and ratio < 5:
            print("   ⇒ 支持 (A)：近处的回波几乎都在 0.2m 以下 ⇒ 被高度阈值裁掉（min_obstacle_height 已改 0.0，重启后应改善）")
        elif ratio > 20:
            print("   ⇒ 支持 (B)：输入里近处**有**足够高的点 ⇒ stvl 仍看不到 ⇒ 查 clearing / raytrace_*_range / voxel_decay / mark_threshold")
        else:
            print("   ⇒ 中间地带：把两次测量（贴墙 vs 远站）对比着看，重点看'首个 z>=0.2 占比<5% 的距离桶'")
        if first_break:
            print(f"   · 第一个「几乎不含 z>=0.2 点」的距离桶：{first_break[0]}~{first_break[1]} m")

    def _scan_pct(self, b):
        s = self.scan[b]
        return f"{100.0*s['finite']/s['total']:.1f}%" if s["total"] else "--"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--sensor-height", type=float, default=0.226)
    a = ap.parse_args()
    rclpy.init()
    n = Prof(a.sensor_height)
    print(f"[profile] 采样 {a.duration:.0f}s —— 请保持机器人与被测墙的相对位置不变")
    t0 = time.time()
    try:
        while time.time() - t0 < a.duration:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    n.table()
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
