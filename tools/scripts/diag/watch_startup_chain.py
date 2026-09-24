#!/usr/bin/env python3
"""启动链路看门狗：每秒打印链路上每一环的最新戳与速率，直到全部就绪或超时。

链路顺序（左→右应依次出现并持续前进）：
  /clock → 点云 → IMU → /scan → /odom → TF(odom→base_link) → TF(map→odom) → costmap footprint
任何一环显示 '--' 或计数不再增加，它就是当前卡点；全绿之后再发目标点才有意义。

QoS 已按各话题实际设置好（这是本仓库反复踩的坑：echo/hz 默认 RELIABLE 收不到 best effort 话题）：
  /livox/lidar/pointcloud、/livox/imu、/scan   -> BEST_EFFORT
  /clock                                        -> BEST_EFFORT（gzserver 这么发的）
  /odom、/tf、/local_costmap/published_footprint -> RELIABLE

用法: python3 tools/scripts/diag/watch_startup_chain.py [--duration 120]
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
EDGES = ("odom->base_link", "map->odom", "base_link->base_link_fake")


class Watch(Node):
    def __init__(self):
        super().__init__("watch_startup_chain")
        self.stamp = {}
        self.cnt = {}
        self.tf = {}

        def mk(name, count=False):
            def cb(m):
                s = (m.clock.sec + m.clock.nanosec * 1e-9) if count else \
                    (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                self.stamp[name] = s
                self.cnt[name] = self.cnt.get(name, 0) + 1
            return cb

        self.create_subscription(Clock, "/clock", mk("clock", True), BEST)
        self.create_subscription(PointCloud2, "/livox/lidar/pointcloud", mk("pcloud"), BEST)
        self.create_subscription(Imu, "/livox/imu", mk("imu"), BEST)
        self.create_subscription(LaserScan, "/scan", mk("scan"), BEST)
        self.create_subscription(Odometry, "/odom", mk("odom"), RELI)
        self.create_subscription(PolygonStamped, "/local_costmap/published_footprint", mk("footprint"), RELI)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI)

    def on_tf(self, msg):
        for t in msg.transforms:
            k = f"{t.header.frame_id.lstrip('/')}->{t.child_frame_id.lstrip('/')}"
            if k in EDGES:
                self.tf[k] = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0)
    a = ap.parse_args()

    rclpy.init()
    n = Watch()
    t0, prev = time.time(), {}
    print(f"[watch] 观察 {a.duration:.0f}s；'--' = 至今没收到。请现在启动（或已启动的）仿真栈。", flush=True)
    try:
        while time.time() - t0 < a.duration:
            end = time.time() + 1.0
            while time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.02)
            d = {k: n.cnt.get(k, 0) - prev.get(k, 0) for k in
                 ("pcloud", "imu", "scan", "odom", "footprint")}
            prev = {k: n.cnt.get(k, 0) for k in d}
            f = lambda v: "--" if v is None else f"{v:.2f}"          # noqa: E731
            row = (f"t={time.time()-t0:5.0f}s clock={f(n.stamp.get('clock')):>8}"
                   f" | pcloud={f(n.stamp.get('pcloud')):>8}({d['pcloud']:>2}Hz)"
                   f" | imu={f(n.stamp.get('imu')):>8}({d['imu']:>3}Hz)"
                   f" | scan={f(n.stamp.get('scan')):>8}({d['scan']:>2}Hz)"
                   f" | odom={f(n.stamp.get('odom')):>8}({d['odom']:>2}Hz)")
            row2 = ("          TF: " + " | ".join(f"{k}={f(n.tf.get(k))}" for k in EDGES)
                    + f" | footprint={f(n.stamp.get('footprint'))}({d['footprint']:>2}Hz)")
            print(row)
            print(row2, flush=True)
    finally:
        print("\n[watch] 判读：从左往右找第一个 '--' 或 0Hz 的环节，那就是当前卡点。")
        print("        全绿且 footprint 戳持续前进 ⇒ 再发目标点（RViz）才有意义。")
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
