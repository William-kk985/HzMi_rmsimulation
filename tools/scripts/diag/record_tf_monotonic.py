#!/usr/bin/env python3
"""录一段 /tf（+ /clock），按「到达顺序」检查每条边的戳是否单调。

判据（对应候选①：fake_vel_transform 戳非单调 ⇒ tf2 按 TF_OLD_DATA 丢弃 ⇒ costmap 位姿冻在启动那一刻）：
  · running_max  = 按到达顺序处理完后，tf2 缓冲区里会持有的「最新样本」戳
  · drops        = 到达时戳 < 此前最大值 的条数（这些会被 tf2 的 TF_OLD_DATA 丢掉）
  · 若 running_max 明显落后于 clock ⇒ 该边在 tf2 里就是冻的（候选①成立）
  · 若 running_max 基本跟得上 clock ⇒ 候选①不成立（缓冲区会拿到新鲜样本）

用法：
  python3 tools/scripts/diag/record_tf_monotonic.py [--wait 900] [--duration 30]
先等 /tf + /clock 出现（最多 --wait 秒），再录 --duration 秒，然后打印分析。
原始数据存 .tmp_bags/tf_monotonic_<ts>.jsonl。
"""
import argparse
import json
import os
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

WATCH_HINT = ("odom", "base_link", "base_link_fake", "map", "livox_frame", "imu_link")


class Rec(Node):
    def __init__(self):
        super().__init__("record_tf_monotonic")
        self.clock = None
        self.clock_n = 0
        self.saw_tf = False
        self.rows = []                      # (arrival_wall, parent, child, stamp)
        cq = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, "/clock", self.on_clock, cq)
        tq = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tq)

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_n += 1

    def on_tf(self, msg):
        self.saw_tf = True
        wall = time.time()
        for t in msg.transforms:
            self.rows.append((wall,
                              t.header.frame_id.lstrip("/"),
                              t.child_frame_id.lstrip("/"),
                              t.header.stamp.sec + t.header.stamp.nanosec * 1e-9))


def spin_for(node, seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)


def analyse(rows, clock_at_end, clock_n):
    per = defaultdict(list)
    for wall, p, c, st in rows:
        per[(p, c)].append(st)
    print(f"\n共 {len(rows)} 条变换，{len(per)} 条边；clock(末)={clock_at_end}，/clock 消息数={clock_n}")
    print(f"{'边':<38}{'条数':>6}{'Hz':>7}{'drops':>7}{'dup':>5}{'最大回退s':>10}"
          f"{'running_max':>13}{'最后戳':>12}{'max-clock':>11}")
    lines = []
    for (p, c), stamps in per.items():
        n = len(stamps)
        rate = n / max(1e-6, sum([0]) + (rows[-1][0] - rows[0][0])) if n else 0.0
        run_max, drops, dup, max_back = None, 0, 0, 0.0
        for s in stamps:
            if run_max is None:
                run_max = s
            else:
                if s > run_max:
                    run_max = s
                else:
                    drops += 1
                    if s == run_max:
                        dup += 1
                    else:
                        max_back = max(max_back, run_max - s)
        delta = (run_max - clock_at_end) if (run_max is not None and clock_at_end) else float("nan")
        lines.append((p, c, n, rate, drops, dup, max_back, run_max, stamps[-1], delta))
    # 关心的边优先，其余按条数
    def key(r):
        p, c = r[0], r[1]
        star = 0 if (c == "base_link_fake" or (p, c) in (("odom", "base_link"), ("map", "odom"))) else 1
        return (star, -r[2])
    for p, c, n, rate, drops, dup, max_back, run_max, last, delta in sorted(lines, key=key):
        mark = ""
        if c == "base_link_fake":
            mark = "  ★候选①的目标边"
        print(f"{p+' -> '+c:<38}{n:>6}{rate:>7.1f}{drops:>7}{dup:>5}{max_back:>10.3f}"
              f"{run_max:>13.3f}{last:>12.3f}{delta:>11.3f}{mark}")
    print("\n判读：drops>0 且 running_max 明显落后 clock ⇒ tf2 会丢新样本、缓冲区停在旧戳（候选①成立）；")
    print("      drops≈0 且 running_max≈clock ⇒ 该边在 tf2 里是新鲜的，候选①不成立。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=900.0, help="等 /tf 出现的最长秒数")
    ap.add_argument("--duration", type=float, default=30.0, help="录制秒数")
    ap.add_argument("--outdir", default=".tmp_bags")
    a = ap.parse_args()

    rclpy.init()
    n = Rec()
    os.makedirs(a.outdir, exist_ok=True)
    out = os.path.join(a.outdir, f"tf_monotonic_{int(time.time())}.jsonl")

    print(f"[wait] 等 /tf（最多 {a.wait:.0f}s）… 请现在启动仿真/nav 栈", flush=True)
    t0 = time.time()
    while time.time() - t0 < a.wait:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.saw_tf and n.clock_n > 0:
            break
        if int(time.time() - t0) % 15 == 0:
            print(f"[wait] {time.time()-t0:.0f}s … saw_tf={n.saw_tf} clock_msgs={n.clock_n}", flush=True)
    if not n.saw_tf:
        print("[wait] 超时：始终没有 /tf。请启动栈后重跑。", flush=True)
        n.destroy_node(); rclpy.shutdown(); sys.exit(2)

    print(f"[rec] 开始录制 {a.duration:.0f}s（clock≈{n.clock}）", flush=True)
    spin_for(n, a.duration)
    clock_end, clock_n = n.clock, n.clock_n

    with open(out, "w") as f:
        for wall, p, c, st in n.rows:
            f.write(json.dumps({"wall": wall, "parent": p, "child": c, "stamp": st}) + "\n")
    print(f"[rec] 原始数据 -> {out}", flush=True)

    if not n.rows:
        print("[rec] 一条 /tf 都没收到（QoS/图未起？）", flush=True)
    else:
        analyse(n.rows, clock_end, clock_n)

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
