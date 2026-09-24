#!/usr/bin/env python3
"""同一时刻对比 /clock 与各 TF 边的最新戳，判断是否存在 >cache_time(10s) 的偏差。

判据：若 |clock - tf_stamp| 持续 > 10 s，则 tf2 的 TF_OLD_DATA 剪枝会把所有新样本判为
"过去的数据"而丢弃 —— 这正是 costmap 的 tf 缓冲区"只进 0.2 秒就永久停"的可疑机制。
用墙钟订阅（不做时钟同步），只读 header.stamp，因此不影响被测系统。
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

# /tf 上真正被直接发布的边（map->base_link_fake 是 tf2 现场合成的，不会出现在 /tf 里）
WATCH = {("odom", "base_link"), ("base_link", "base_link_fake"), ("map", "odom")}


class Skew(Node):
    def __init__(self):
        super().__init__("measure_tf_clock_skew")
        self.clock = None
        self.stamps = {}
        self.msgs = {}   # 话题 -> 最新 header.stamp（对照 TF 数据区间用）
        # 注意：gzserver 的 /clock 是 BEST_EFFORT 发的，用默认 RELIABLE 会收不到（QoS 不兼容）
        clock_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        # 点云是 SensorDataQoS(best effort)；/odom 用默认 RELIABLE（与 ros2 topic echo 默认一致）
        self.create_subscription(PointCloud2, "/livox/lidar/pointcloud", lambda m: self.on_msg("pointcloud", m),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(Odometry, "/odom", lambda m: self.on_msg("odom", m), 20)

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_msg(self, name, msg):
        self.msgs[name] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_tf(self, msg):
        for t in msg.transforms:
            key = (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
            if key in WATCH:
                self.stamps[key] = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

    def report(self):
        if self.clock is None:
            print("  (还没收到 /clock)")
            return
        parts = []
        for key in sorted(WATCH):
            v = self.stamps.get(key)
            parts.append(f"{key[0]}->{key[1]}: {'--' if v is None else f'{v:.3f} (Δ{v - self.clock:+.3f})'}")
        print(f"clock={self.clock:.3f} | " + " | ".join(parts))
        m = []
        for k in ("pointcloud", "odom"):
            v = self.msgs.get(k)
            m.append(f"{k}: {'--' if v is None else f'{v:.3f} (Δ{v - self.clock:+.3f})'}")
        print("        消息戳  " + " | ".join(m))


def main():
    rclpy.init()
    n = Skew()
    try:
        for _ in range(8):
            # 在 1 秒内密集排空队列，避免 KEEP_LAST(100)+43Hz 把样本挤掉
            t_end = time.time() + 1.0
            while time.time() < t_end:
                rclpy.spin_once(n, timeout_sec=0.05)
            n.report()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
