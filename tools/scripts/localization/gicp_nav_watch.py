#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gicp 定位「发散 / 抖动」A/B 的记录仪 + 发目标驱动器（一条命令，跑在已起的栈上）。

它回答的问题（docs/gicp_divergence_and_jitter.md 的 A/B 表就由它产出）：
  ① `map→odom`（GICP 的修正量）**逐次更新**的步长有多大（平移/yaw 的 p50/p90/p99/max）——
     这就是"发散"的直接证据；`~/fitness_score` 好不好看是另一回事；
  ② 融合位姿 `map→base_link`（= map→odom ∘ odom→base_link）的**逐条 TF 抖动**（相邻差分的 std/p95）——
     这是 nav2 真正看到的量，也是用户抱怨的"抖"；
  ③ 目标到达后 `/cmd_vel*` 还有没有输出、车（/odom_ground_truth）=**真的**在动吗（真值抖动）；
  ④ 与真值的一致性：`map→base_link` 与 `/odom_ground_truth` 的偏差（定位误差代理量）；
  ⑤ gicp 节点 [status] 行的采纳/拒绝计数（从 launch 日志里 grep，见 gicp_ab_report.py）。

**只读仪器**：除了"发一个 navigate_to_pose 目标"和"收尾时发零速"之外，本脚本不改变任何东西；
它不发布 TF、不写任何参数文件。零速收尾是必须的（否则 MPPI 的最后一个指令会留在底盘上，
下一次跑之前车会一直动 —— 这也是本仓库其它 A/B 脚本的既有做法）。

用法（**必须与 launch 塞进同一次 bash 调用**，见 run_gicp_nav_ab.sh 的头注）：
    python3 tools/scripts/localization/gicp_nav_watch.py \
        --tag base --out .tmp_gicp_ab/out/base.jsonl \
        --goal -12.64 -0.31 --settle 8 --post-goal-sec 90 --summary .tmp_gicp_ab/out/base.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

CHAIN = ("map->odom", "odom->base_link", "base_link->base_link_fake")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Watch(Node):
    def __init__(self, a):
        super().__init__("gicp_nav_watch")
        self.a = a
        self.fh = open(a.out, "w")
        self.t0 = time.time()
        self.clock = None
        self.clock0 = None
        self.wall0 = time.time()
        self.cnt = {}
        self.tf = {}          # key -> (stamp_sim, (x,y,z), (qx,qy,qz,qw))
        self.tf_seen = set()
        self.fp = 0
        self.gt = None        # 最新 /odom_ground_truth
        self.cmd = {"nav": (0.0, 0.0, 0.0, 0.0), "smooth": (0.0, 0.0), "chassis": (0.0, 0.0)}
        self.health = {"fitness": None, "converged": None}
        self.last_row_clock = None
        self.last_mo_stamp = None
        self.n_rows = 0
        self.goal_events = []   # (wall, sim, kind, payload)

        self.create_subscription(Clock, "/clock", self.on_clock, BEST)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, RELI)
        self.create_subscription(Odometry, "/odom", self.on_odom, RELI)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd("smooth"), RELI)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_cmd("nav"), RELI)
        self.create_subscription(Twist, "/cmd_vel_chassis", self.on_cmd("chassis"), RELI)
        self.create_subscription(Float64, "/gicp_registration/fitness_score", self.on_fit, RELI)
        self.create_subscription(Bool, "/gicp_registration/converged", self.on_conv, RELI)
        for t, q in (("/livox/lidar/pointcloud", BEST), ("/livox/imu", BEST), ("/scan", BEST)):
            self.create_subscription(PointCloud2 if "point" in t else (
                Imu if "imu" in t else LaserScan), t, self.on_any(t), q)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_chassis", RELI)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---------------- 回调 ----------------
    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock0 is None:
            self.clock0, self.wall0 = self.clock, time.time()
        self.maybe_row()

    def on_any(self, name):
        def f(_m):
            self.cnt[name] = self.cnt.get(name, 0) + 1
        return f

    def on_tf(self, m):
        for t in m.transforms:
            k = "%s->%s" % (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
            if k not in CHAIN:
                continue
            s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tr, q = t.transform.translation, t.transform.rotation
            self.tf[k] = (s, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
            self.tf_seen.add(k)
            if k == "map->odom":
                self.cnt["map->odom"] = self.cnt.get("map->odom", 0) + 1
        self.maybe_row()
        # footprint 的活性由 /local_costmap/published_footprint 另计（这里用 TF 变化代理）

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.twist.twist
        self.gt = (p.x, p.y, p.z, yaw_of((q.x, q.y, q.z, q.w)),
                   t.linear.x, t.linear.y, t.angular.z)
        self.cnt["gt"] = self.cnt.get("gt", 0) + 1

    def on_odom(self, m):
        self.cnt["odom"] = self.cnt.get("odom", 0) + 1

    def on_cmd(self, name):
        def f(m):
            self.cmd[name] = (m.linear.x, m.linear.y, m.angular.z)
            self.cnt["cmd_" + name] = self.cnt.get("cmd_" + name, 0) + 1
        return f

    def on_fit(self, m):
        self.health["fitness"] = m.data
        self.cnt["fitness"] = self.cnt.get("fitness", 0) + 1

    def on_conv(self, m):
        self.health["converged"] = bool(m.data)

    # ---------------- 记录 ----------------
    def maybe_row(self, force=False):
        """采样策略（两个都必须满足"能看见"）：

        · **map→odom 每次更新必录**（新戳就写一行）—— GICP 的采纳帧是"发散"的最小单位，
          漏掉它就没法算"逐次更新步长"；
        · 其余按仿真时钟限频 **≥50 Hz**（`--row-dt`）—— 融合位姿/TF 抖动的采样率。

        注：行里的值是"当时收到的最新一条"，与 nav2 通过 tf2 查到的量同一个口径（不做插值）。
        """
        if self.clock is None:
            return
        mo_stamp = None if self.tf.get("map->odom") is None else self.tf["map->odom"][0]
        new_update = (mo_stamp is not None and mo_stamp != self.last_mo_stamp)
        if not force and not new_update:
            if self.last_row_clock is not None and self.clock - self.last_row_clock < self.a.row_dt:
                return
        self.last_row_clock = self.clock
        if mo_stamp is not None:
            self.last_mo_stamp = mo_stamp
        mo = self.tf.get("map->odom")
        ob = self.tf.get("odom->base_link")
        bf = self.tf.get("base_link->base_link_fake")
        row = {
            "w": round(time.time() - self.t0, 4),
            "t": round(self.clock, 5),
            "mo": None if mo is None else [round(v, 6) for v in mo[1]] + [round(yaw_of(mo[2]), 6), round(mo[0], 5)],
            "ob": None if ob is None else [round(v, 6) for v in ob[1]] + [round(yaw_of(ob[2]), 6), round(ob[0], 5)],
            "bf": None if bf is None else [round(v, 6) for v in bf[1]] + [round(yaw_of(bf[2]), 6)],
            "gt": None if self.gt is None else [round(v, 5) for v in self.gt],
            "cmd": [round(self.cmd["smooth"][0], 4), round(self.cmd["smooth"][1], 4),
                    round(self.cmd["nav"][0], 4), round(self.cmd["nav"][1], 4),
                    round(self.cmd["chassis"][0], 4), round(self.cmd["chassis"][1], 4)],
            "upd": bool(new_update),
            "fit": self.health["fitness"],
            "conv": self.health["converged"],
        }
        self.fh.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.n_rows += 1

    # ---------------- 工具 ----------------
    def drain(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if pred():
                return True
        return False

    def fused(self):
        """map→base_link（平移 + yaw，平面复合）：缺任一段返回 None。"""
        mo, ob = self.tf.get("map->odom"), self.tf.get("odom->base_link")
        if mo is None or ob is None:
            return None
        x0, y0, y0aw = mo[1][0], mo[1][1], yaw_of(mo[2])
        x1, y1, y1aw = ob[1][0], ob[1][1], yaw_of(ob[2])
        c, s = math.cos(y0aw), math.sin(y0aw)
        return (x0 + c * x1 - s * y1, y0 + s * x1 + c * y1, mo[1][2] + ob[1][2], wrap(y0aw + y1aw))

    def zero_twist(self, sec=2.0):
        t = Twist()
        end = time.time() + sec
        while time.time() < end:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)

    def ev(self, kind, payload=None):
        self.goal_events.append({"w": round(time.time() - self.t0, 3),
                                 "t": None if self.clock is None else round(self.clock, 3),
                                 "kind": kind, "payload": payload})
        print("  [ev %7.3f] %s %s" % (time.time() - self.t0, kind, payload if payload else ""), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="run")
    ap.add_argument("--out", required=True, help="JSONL 输出路径")
    ap.add_argument("--summary", default="", help="汇总 JSON 输出路径")
    ap.add_argument("--goal", nargs=2, type=float, default=None)
    ap.add_argument("--yaw", default="auto", help="auto = 当前朝向（与 RViz 拖箭头一致）或弧度值")
    ap.add_argument("--settle", type=float, default=8.0, help="发目标前静置多少秒（仿真时间近似）")
    ap.add_argument("--ready-timeout", type=float, default=240.0)
    ap.add_argument("--goal-timeout", type=float, default=180.0)
    ap.add_argument("--post-goal-sec", type=float, default=90.0, help="目标结束后继续录多久（**墙钟**秒）")
    ap.add_argument("--post-stop-sec", type=float, default=8.0, help="零速收尾后再录多久")
    ap.add_argument("--row-dt", type=float, default=0.02,
                    help="非更新行的最小仿真时间间隔（秒；默认 0.02 = 50 Hz 采样）")
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    rclpy.init()
    n = Watch(a)
    fails = []

    # ---------- 0 等链路 ----------
    print("[watch] 等链路（最多 %.0fs）：点云/imu/scan/odom/TF(odom→base_link, map→odom)/真值…"
          % a.ready_timeout, flush=True)
    t0 = time.time()
    while time.time() - t0 < a.ready_timeout:
        n.drain(2.0)
        ok = (n.cnt.get("/livox/lidar/pointcloud", 0) > 0 and n.cnt.get("/livox/imu", 0) > 0
              and n.cnt.get("/scan", 0) > 0 and n.cnt.get("gt", 0) > 0
              and {"odom->base_link", "map->odom"} <= n.tf_seen)
        print("  [ready %5.1fs] pcloud=%d imu=%d scan=%d gt=%d TF=%s map→odom=%d fitness=%s"
              % (time.time() - t0, n.cnt.get("/livox/lidar/pointcloud", 0), n.cnt.get("/livox/imu", 0),
                 n.cnt.get("/scan", 0), n.cnt.get("gt", 0), ",".join(sorted(n.tf_seen)),
                 n.cnt.get("map->odom", 0), n.health["fitness"]), flush=True)
        if ok and n.cnt.get("map->odom", 0) > 0:
            break
    else:
        fails.append("链路未就绪")
    rtf = None
    if n.clock0 is not None:
        rtf = (n.clock - n.clock0) / max(1e-6, time.time() - n.wall0)
    print("[watch] 链路就绪用时 %.1fs（墙钟），RTF≈%s" % (time.time() - t0, None if rtf is None else round(rtf, 2)),
          flush=True)

    # ---------- 1 静置（让 GICP 收敛 + 记录静止基线） ----------
    if a.settle > 0:
        print("[watch] 静置 %.1fs（墙钟；记录静止基线）…" % a.settle, flush=True)
        n.drain(a.settle)
    n.ev("baseline", {"map_odom": n.tf.get("map->odom"), "fused": n.fused(), "gt": n.gt})

    # ---------- 2 发目标 ----------
    res = {"status": None, "secs": None, "d_min": None, "recoveries": 0}
    if a.goal is not None and not fails:
        fu = n.fused()
        if a.yaw == "auto":
            yaw = 0.0 if fu is None else fu[3]
        else:
            yaw = float(a.yaw)
        print("[watch] 发目标 (%.2f, %.2f) yaw=%.3f rad" % (a.goal[0], a.goal[1], yaw), flush=True)
        if not n.ac.wait_for_server(timeout_sec=30.0):
            fails.append("/navigate_to_pose 不可用")
        else:
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"
            g.pose.pose.position.x, g.pose.pose.position.y = a.goal
            g.pose.pose.orientation.z = math.sin(yaw / 2.0)
            g.pose.pose.orientation.w = math.cos(yaw / 2.0)
            t_send = time.time()

            def fb(m):
                fb_ = m.feedback
                res["d_min"] = (fb_.distance_remaining if res["d_min"] is None
                                else min(res["d_min"], fb_.distance_remaining))
                res["recoveries"] = max(res["recoveries"], fb_.number_of_recoveries)

            send = n.ac.send_goal_async(g, feedback_callback=fb)
            n.spin_until(lambda: send.done(), 20.0)
            gh = send.result() if send.done() else None
            if gh is None or not gh.accepted:
                fails.append("目标被拒")
                n.ev("goal_rejected")
            else:
                n.ev("goal_sent", {"goal": a.goal, "yaw": round(yaw, 4)})
                rr = gh.get_result_async()
                done = n.spin_until(lambda: rr.done(), a.goal_timeout)
                status = None if not done else rr.result().status
                res.update(status=status, secs=round(time.time() - t_send, 2), wall=round(time.time() - n.t0, 2),
                           sim=None if n.clock is None else round(n.clock, 3))
                names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
                print("[watch] 目标结束 status=%s（%s）用时=%.1fs 墙钟 d_min=%s recoveries=%d"
                      % (status, names.get(status, "?"), time.time() - t_send, res["d_min"], res["recoveries"]),
                      flush=True)
                n.ev("goal_result", {"status": status, "name": names.get(status, "?"),
                                     "secs_wall": res["secs"], "d_min": res["d_min"],
                                     "recoveries": res["recoveries"]})

    # ---------- 3 目标后继续录（**用户抱怨的"四处抖动"就在这一段**） ----------
    if a.post_goal_sec > 0:
        print("[watch] 目标后继续录 %.0fs（墙钟）…" % a.post_goal_sec, flush=True)
        n.drain(a.post_goal_sec)
        n.ev("post_goal_window_end")

    # ---------- 4 零速收尾（不改变任何结论，只为"下一个跑之前车是停的"） ----------
    if a.post_stop_sec > 0:
        print("[watch] 发零速 %.0fs 并再录 %.0fs…" % (2.0, a.post_stop_sec), flush=True)
        n.zero_twist(2.0)
        n.drain(a.post_stop_sec)
        n.ev("zero_twist_done")

    # ---------- 5 落盘 ----------
    n.drain(0.2)
    n.fh.flush()
    n.fh.close()
    summ = {"tag": a.tag, "fails": fails, "rows": n.n_rows, "cnt": n.cnt,
            "tf_seen": sorted(n.tf_seen), "rtf": rtf, "goal": a.goal, "result": res,
            "events": n.goal_events}
    if a.summary:
        with open(a.summary, "w") as f:
            json.dump(summ, f, ensure_ascii=False, indent=2)
    print("[watch] 记录 %d 行 → %s；汇总 → %s" % (n.n_rows, a.out, a.summary or "(未要)"), flush=True)
    print("[watch] fails=%s" % fails, flush=True)
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0 if not fails else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[watch] 中断")
        sys.exit(2)
