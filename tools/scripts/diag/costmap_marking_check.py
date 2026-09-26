#!/usr/bin/env python3
"""costmap_marking_check.py —— 量化「缺口」：近处的墙点在**全局 costmap 里到底有没有被标记**。

做法：同时订阅
  · /segmentation/obstacle（stvl 的输入，BEST_EFFORT，frame=livox_frame）
  · /global_costmap/costmap_raw（RELIABLE+VOLATILE，frame=map）
用 TF 把点云变换到 map 系，逐点查它落在 costmap 的哪个格子、该格子的代价值是多少，
按「到机器人的水平距离」分桶统计：
  点数 | 已标记占比(cost>=100) | 致命/内切占比(cost>=253) | 平均cost

判读（近距桶 <1.5m）：
  · 已标记占比很低(<30%) ⇒ **缺口确认在 costmap 侧**：stvl 没标（高度带/参数）或被清/衰减
  · 已标记占比很高        ⇒ costmap 里有标记，"缺口"是显示层或其它层造成（不是 stvl 丢）
配套：先用 cloud_z_profile.py 确认输入侧的 z 分布（已做：近距 z_map<0.2 ⇒ 老参数会全丢）。

用法（贴近那面墙站 20s；再退远 20s 对比）：
  python3 tools/scripts/diag/costmap_marking_check.py --duration 20
"""
import argparse
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
# nav2 的 costmap/costmap_raw 是 **transient_local(latched)** 发布的；再挂一份 transient_local 订阅，
# 两种 durability 各匹配一次，谁收到用谁（同话题两个订阅是允许的）。
LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
BUCKETS = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 10.0)]


def iter_xyz(msg, stride=2):
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


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    return ((1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
            2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
            2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz)


class Chk(Node):
    def __init__(self, topic="/global_costmap/costmap_raw"):
        super().__init__("costmap_marking_check")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.grid = None
        self.stat = {b: {"n": 0, "mk": 0, "lethal": 0, "cost": 0} for b in BUCKETS}
        self.frames, self.clouds, self.no_tf = set(), 0, 0
        self.create_subscription(PointCloud2, "/segmentation/obstacle", self.on_cloud, BEST)
        # ★ 2026-09-26 修正：原来请求 RELIABLE，而话题可能是 BEST_EFFORT 发布的 ⇒ 一点都收不到
        #   （本项目第 N 次踩同一个坑：/scan、/clock 也是）。DDS 的 RxO 规则下：
        #   「请求 BEST_EFFORT」对 RELIABLE / BEST_EFFORT 两种写者**都兼容** ⇒ 统一用它，
        #   同时把写者的真实 QoS 打印出来，避免以后再猜。
        try:
            infos = self.get_publishers_info_by_topic(topic)
            if infos:
                q = infos[0].qos_profile
                print("[check] %s 写者 QoS: reliability=%s durability=%s depth=%s"
                      % (topic, q.reliability, q.durability, q.depth), flush=True)
            else:
                print("[check] %s 暂无写者（图发现可能滞后，仍继续订阅）" % topic, flush=True)
        except Exception as e:               # noqa: BLE001
            print("[check] 查询写者 QoS 失败:", e, flush=True)
        self.create_subscription(Costmap, topic, self.on_grid, BEST)

    def on_grid(self, m):
        self.grid = m

    def on_cloud(self, m):
        self.clouds += 1
        self.frames.add(m.header.frame_id)
        if self.grid is None:
            return
        try:
            tf = self.buf.lookup_transform(self.grid.header.frame_id, m.header.frame_id, Time())
        except Exception:
            self.no_tf += 1
            return
        t = tf.transform.translation
        q = (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w)
        # nav2_msgs/msg/Costmap：元数据在 metadata（size_x/size_y/origin），代价是 uint8 0..255
        # （0=free, 1..252=inflated, 253=inscribed, 254=lethal, 255=NO_INFORMATION）
        md = self.grid.metadata
        res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
        w, h, data = md.size_x, md.size_y, self.grid.data
        for x, y, z in iter_xyz(m):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            rx, ry, _ = quat_rotate(q, (x, y, z))
            mx, my = rx + t.x, ry + t.y
            b = None
            for lo, hi in BUCKETS:
                if lo <= math.hypot(x, y) < hi:
                    b = (lo, hi); break
            if b is None:
                continue
            self.stat[b]["n"] += 1
            cx, cy = int((mx - ox) / res), int((my - oy) / res)
            if 0 <= cx < w and 0 <= cy < h:
                c = int(data[cy * w + cx])
                if c == 255:                      # NO_INFORMATION：既非障碍也非自由
                    self.stat[b]["unk"] = self.stat[b].get("unk", 0) + 1
                else:
                    if c >= 100:
                        self.stat[b]["mk"] += 1
                    if c >= 253:
                        self.stat[b]["lethal"] += 1
                    self.stat[b]["cost"] += c

    def table(self):
        g = self.grid
        print(f"\n  点云帧={self.clouds} frame={sorted(self.frames)}  未取到TF={self.no_tf}")
        if g is None:
            print("  ⚠️ 没收到 /global_costmap/costmap_raw —— 全局 costmap 可能没启动（或话题不同）"); return
        md = g.metadata
        print(f"  costmap: {md.size_x}x{md.size_y} @ {md.resolution:.3f} m  frame={g.header.frame_id}  "
              f"type=nav2_msgs/Costmap")
        print(f"  {'距离桶(m)':>12} {'点数':>8} {'已标记(cost>=100)':>18} {'致命/内切(>=253)':>17} {'平均cost':>10}")
        for b in BUCKETS:
            e = self.stat[b]
            if e["n"] == 0:
                print(f"  {('%g-%g' % b):>12} {0:>8} {'--':>18} {'--':>17} {'--':>10}"); continue
            print(f"  {('%g-%g' % b):>12} {e['n']:>8} "
                  f"{('%.1f%%' % (100.0*e['mk']/e['n'])):>18} "
                  f"{('%.1f%%' % (100.0*e['lethal']/e['n'])):>17} "
                  f"{(e['cost']/e['n']):>10.1f}")
        near = [b for b in BUCKETS if b[0] < 1.5]
        n = sum(self.stat[b]["n"] for b in near)
        mk = sum(self.stat[b]["mk"] for b in near)
        if n:
            pct = 100.0 * mk / n
            print(f"\n  【判读】近距(<1.5m) 点数={n}  已标记占比={pct:.1f}%")
            if pct < 30:
                print("   ⇒ 缺口在 costmap 侧：输入有点（见 cloud_z_profile）、但格子里没标上")
                print("     先确认 stvl 参数：min_obstacle_height / clearing / raytrace_*_range / voxel_decay")
            elif pct < 70:
                print("   ⇒ 部分标记：典型的'碎块'——时有时无，查 voxel_decay / observation_persistence / clearing")
            else:
                print("   ⇒ costmap 里有标记：'缺口'更可能是显示层（RViz 显示的是哪张图/哪个 frame）")
        else:
            print("\n  【判读】近距桶没有点：先跑 cloud_z_profile.py 看输入侧")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--topic", default="/global_costmap/costmap_raw",
                    help="代价地图话题；也可指向 /global_costmap/costmap、/local_costmap/costmap_raw 等")
    a = ap.parse_args()
    rclpy.init()
    n = Chk(a.topic)
    print(f"[check] 采样 {a.duration:.0f}s，代价地图话题={a.topic} —— 请保持机器人与被测墙的相对位置不变")
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
