#!/usr/bin/env python3
"""P0 回归：一条命令回答"这套槽位组合能不能正常走完一次导航，断点在哪"。

它做五件事（都不改算法/参数，只订阅 + 发一个目标点）：
  0) 等链路就绪：/livox/lidar/pointcloud、/livox/imu、/scan、/odom、TF(odom->base_link, map->odom)、
     costmap footprint —— 每项都要出现且频率达标；否则直接给出「第一个断点」。
  1) 发一个固定目标（默认 -1.0, 2.0，可用 --goal 指定；--skip-goal 只做链路自检）。
  2) 四跳命令链断言：/cmd_vel_nav → /cmd_vel → /cmd_vel_chassis，并用 /odom_ground_truth（仿真真值）
     确认车真的动了。**这条判据就是当初抓到 fake_vel_transform 丢角速度的那个判据。**
  3) 导航结果断言：distance_remaining 递减并到达；**拒绝"假到达"**——若结果 <2 s 就 SUCCEEDED
     而距离残余还 >0.5 m ⇒ FAIL（这正是我们花一天追的那个 bug 的特征）。
  4) 输出 PASS/FAIL + 首个断点 + 关键数值，并写 .tmp_bags/regress_<ts>.json（可直接抄进
     docs/algorithm_matrix.md §四 实测状态表）。

用法（栈要先跑起来；本脚本会等它）：
  python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
  python3 tools/scripts/regress/nav_smoke_regression.py --skip-goal          # 只体检链路
  python3 tools/scripts/regress/nav_smoke_regression.py --ready-timeout 240 --goal-timeout 180

判据阈值集中在下面 TH 字典里，可按需调（例如换了 world / 起始点）。
"""
import argparse
import json
import os
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

# 阈值（判据集中在此，便于按 world/起始点调整）
TH = {
    "clock_rtf_min": 0.35,        # /clock 相对墙钟的实时因子下限
    "hz_pcloud_min": 4.0,         # 名义 10 Hz；RTF≈0.7 时约 7 Hz
    "hz_imu_min": 40.0,           # 名义 100 Hz
    "hz_scan_min": 4.0,
    "hz_odom_min": 4.0,
    "tf_age_max": 3.0,            # TF 最新戳落后 /clock 的上限（秒）
    "footprint_min_distinct": 3,  # footprint 戳至少要变过几次（证明位姿在更新）
    "fake_success_secs": 2.0,     # 结果快于此且距离没减 ⇒ 判"假到达"
    "fake_success_dist": 0.5,     # 距离残余大于此值 ⇒ 没真到
    "arrive_tol": 0.35,           # 判定"到了"的距离阈值（goal checker 0.25 + 余量）
    "mv_eps": 1e-3,               # 命令链非零判定
}


class Regress(Node):
    def __init__(self, spin_speed):
        super().__init__("nav_smoke_regression")
        self.spin_speed = spin_speed
        self.clock = None
        self.clock_wall = None
        self.clock0 = None
        self.wall0 = time.time()
        self.stamp, self.cnt = {}, {}
        self.tf, self.tf_seen = {}, set()
        self.fp_stamps = []
        self.cmd = {"nav": (0.0, 0.0), "smooth": (0.0, 0.0), "chassis": (0.0, 0.0)}
        self.gt_twist_max = 0.0
        self.gt_pose0 = None
        self.gt_pose_delta = 0.0
        self.d_min = None
        self.recoveries = 0

        def cb(name, qos, stamp_attr=None):
            def f(m):
                s = getattr(m, stamp_attr) if stamp_attr else m.header.stamp
                v = s.sec + s.nanosec * 1e-9
                self.stamp[name] = v
                self.cnt[name] = self.cnt.get(name, 0) + 1
            return f

        self.create_subscription(Clock, "/clock", self.on_clock, BEST)
        self.create_subscription(PointCloud2, "/livox/lidar/pointcloud", cb("pcloud", BEST), BEST)
        self.create_subscription(Imu, "/livox/imu", cb("imu", BEST), BEST)
        self.create_subscription(LaserScan, "/scan", cb("scan", BEST), BEST)
        self.create_subscription(Odometry, "/odom", cb("odom", RELI), RELI)
        self.create_subscription(PolygonStamped, "/local_costmap/published_footprint",
                                 self.on_fp, RELI)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, RELI)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, RELI)
        self.create_subscription(Twist, "/cmd_vel", self.on_smooth, RELI)
        self.create_subscription(Twist, "/cmd_vel_chassis", self.on_chassis, RELI)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---- 回调 ----
    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock_wall = time.time()
        if self.clock0 is None:
            self.clock0, self.wall0 = self.clock, time.time()

    def on_fp(self, m):
        self.stamp["footprint"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.cnt["footprint"] = self.cnt.get("footprint", 0) + 1
        self.fp_stamps.append(self.stamp["footprint"])

    def on_gt(self, m):
        p = m.pose.pose.position
        if self.gt_pose0 is None:
            self.gt_pose0 = (p.x, p.y)
        else:
            self.gt_pose_delta = max(self.gt_pose_delta,
                                     abs(p.x - self.gt_pose0[0]) + abs(p.y - self.gt_pose0[1]))
        t = m.twist.twist
        self.gt_twist_max = max(self.gt_twist_max, abs(t.linear.x), abs(t.linear.y), abs(t.angular.z))

    def on_tf(self, m):
        for t in m.transforms:
            k = f"{t.header.frame_id.lstrip('/')}->{t.child_frame_id.lstrip('/')}"
            if k in ("odom->base_link", "map->odom", "base_link->base_link_fake"):
                self.tf[k] = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self.tf_seen.add(k)

    def on_nav(self, m):
        self.cmd["nav"] = (max(self.cmd["nav"][0], abs(m.linear.x)), max(self.cmd["nav"][1], abs(m.angular.z)))

    def on_smooth(self, m):
        self.cmd["smooth"] = (max(self.cmd["smooth"][0], abs(m.linear.x)), max(self.cmd["smooth"][1], abs(m.angular.z)))

    def on_chassis(self, m):
        self.cmd["chassis"] = (max(self.cmd["chassis"][0], abs(m.linear.x)), max(self.cmd["chassis"][1], abs(m.angular.z)))

    # ---- 工具 ----
    def drain(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def hz(self, name, secs=3.0):
        c0 = self.cnt.get(name, 0)
        self.drain(secs)
        return (self.cnt.get(name, 0) - c0) / secs

    def rtf(self):
        if self.clock is None or self.clock is None:
            return None
        if self.clock0 is None:
            return None
        dt_wall = max(1e-6, time.time() - self.wall0)
        return (self.clock - self.clock0) / dt_wall


def param(node_name, name):
    try:
        out = subprocess.run(["ros2", "param", "get", node_name, name],
                             capture_output=True, text=True, timeout=8).stdout
        return float(out.strip().split()[-1])
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, default=[-1.0, 2.0])
    ap.add_argument("--ready-timeout", type=float, default=240.0)
    ap.add_argument("--goal-timeout", type=float, default=180.0)
    ap.add_argument("--skip-goal", action="store_true")
    ap.add_argument("--outdir", default=".tmp_bags")
    a = ap.parse_args()

    rclpy.init()
    n = Regress(None)
    fails, notes = [], []

    def hp():
        print("  [%5.1fs] clock=%s | pcloud=%s | imu=%s | scan=%s | odom=%s | fp=%s | TF=%s"
              % (time.time() - n.wall0,
                 "--" if n.clock is None else f"{n.clock:.2f}",
                 n.cnt.get("pcloud", 0), n.cnt.get("imu", 0), n.cnt.get("scan", 0),
                 n.cnt.get("odom", 0), n.cnt.get("footprint", 0),
                 ",".join(sorted(n.tf_seen)) or "--"), flush=True)

    # ---------------- 阶段 0：链路就绪 ----------------
    print("[0] 等链路就绪（最多 %.0fs）… 请确认栈已启动" % a.ready_timeout, flush=True)
    t0 = time.time()
    while time.time() - t0 < a.ready_timeout:
        n.drain(2.0)
        hp()
        if (n.cnt.get("pcloud", 0) and n.cnt.get("imu", 0) and n.cnt.get("scan", 0)
                and n.cnt.get("odom", 0) and n.cnt.get("footprint", 0)
                and {"odom->base_link", "map->odom"} <= n.tf_seen):
            break
    else:
        missing = [k for k in ("pcloud", "imu", "scan", "odom", "footprint") if not n.cnt.get(k)]
        if not {"odom->base_link", "map->odom"} <= n.tf_seen:
            missing += ["TF:" + x for x in ("odom->base_link", "map->odom") if x not in n.tf_seen]
        fails.append("链路未就绪，缺: " + ", ".join(missing))

    rtf = n.rtf()
    hz = {k: n.hz(k) for k in ("pcloud", "imu", "scan", "odom")}
    fp_distinct = len(set(n.fp_stamps))
    if rtf is not None and rtf < TH["clock_rtf_min"]:
        fails.append(f"/clock RTF={rtf:.2f} < {TH['clock_rtf_min']}（仿真没在正常推进）")
    for k, mn in (("pcloud", TH["hz_pcloud_min"]), ("imu", TH["hz_imu_min"]),
                  ("scan", TH["hz_scan_min"]), ("odom", TH["hz_odom_min"])):
        if hz[k] < mn:
            fails.append(f"{k} 频率 {hz[k]:.1f}Hz < {mn}Hz")
    if fp_distinct < TH["footprint_min_distinct"]:
        fails.append(f"costmap footprint 戳只变过 {fp_distinct} 次（位姿未持续更新）")
    age = None if not n.tf else max((n.clock - v) for v in n.tf.values())
    if age is not None and age > TH["tf_age_max"]:
        fails.append(f"TF 最新戳落后 /clock {age:.2f}s > {TH['tf_age_max']}s")
    print(f"[0] RTF={rtf if rtf is None else round(rtf,2)} hz={ {k: round(v,1) for k,v in hz.items()} } "
          f"fp_distinct={fp_distinct} tf_age={None if age is None else round(age,2)}", flush=True)

    # ---------------- 阶段 1：发目标 ----------------
    if not a.skip_goal and not fails:
        print(f"[1] 发目标 ({a.goal[0]:.2f}, {a.goal[1]:.2f}) in map …", flush=True)
        if not n.ac.wait_for_server(timeout_sec=20.0):
            fails.append("/navigate_to_pose 动作服务不可用")
        else:
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"      # stamp 留 0 = 取最新（避免墙钟/仿真钟差异）
            g.pose.pose.position.x, g.pose.pose.position.y = a.goal
            g.pose.pose.orientation.w = 1.0
            t_send = time.time()

            def fb(m):
                n.d_min = m.feedback.distance_remaining if n.d_min is None else min(n.d_min, m.feedback.distance_remaining)
                n.recoveries = max(n.recoveries, m.feedback.number_of_recoveries)

            send = n.ac.send_goal_async(g, feedback_callback=fb)
            n.spin_until(lambda: send.done(), 15.0)
            gh = send.result() if send.done() else None
            if gh is None or not gh.accepted:
                fails.append("目标被拒绝（accepted=False）")
            else:
                res = gh.get_result_async()
                n.spin_until(lambda: res.done(), a.goal_timeout)
                n.drain(1.0)
                dt = time.time() - t_send
                if not res.done():
                    fails.append(f"目标超时 {a.goal_timeout:.0f}s（残余距离 "
                                 f"{'?' if n.d_min is None else round(n.d_min,2)}m）")
                else:
                    st = res.result().status
                    names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
                    notes.append(f"结果 status={names.get(st, st)} 用时={dt:.1f}s "
                                 f"d_min={None if n.d_min is None else round(n.d_min,3)}m "
                                 f"recoveries={n.recoveries}")
                    if st == 4 and dt < TH["fake_success_secs"] and (n.d_min or 0) > TH["fake_success_dist"]:
                        fails.append(f"**假到达**：{dt:.1f}s 就 SUCCEEDED 而残余 {n.d_min:.2f}m"
                                     f"（上游 nav2 丢弃 transformPose 失败的典型特征）")
                    elif st == 4 and (n.d_min is None or n.d_min > TH["arrive_tol"]):
                        fails.append(f"报 SUCCEEDED 但残余距离 {n.d_min}m > 容差 {TH['arrive_tol']}m")
                    elif st != 4:
                        fails.append(f"导航未成功：status={names.get(st, st)}"
                                     f"（若为 ABORTED 且 recoveries>0，多为 progress_checker 判 Failed to make progress）")

            # ---- 阶段 2：四跳命令链 + 真值 ----
            if n.spin_speed is None:
                n.spin_speed = param("/fake_vel_transform", "spin_speed")
            nav_l, nav_a = n.cmd["nav"]
            ch_l, ch_a = n.cmd["chassis"]
            notes.append(f"命令链 max|v|,|w|: nav=({nav_l:.2f},{nav_a:.2f}) "
                         f"smooth=({n.cmd['smooth'][0]:.2f},{n.cmd['smooth'][1]:.2f}) "
                         f"chassis=({ch_l:.2f},{ch_a:.2f}) spin_speed={n.spin_speed}")
            notes.append(f"真值: twist_max={n.gt_twist_max:.3f} pose_delta={n.gt_pose_delta:.3f}m")
            if nav_a > TH["mv_eps"] or nav_l > TH["mv_eps"]:
                if ch_l < TH["mv_eps"] and ch_a < TH["mv_eps"]:
                    fails.append("**命令链断**：nav 有输出但 /cmd_vel_chassis 恒 0"
                                 "（fake_vel_transform 或更下游）")
                elif n.spin_speed == 0.0 and nav_a > TH["mv_eps"] and ch_a < TH["mv_eps"]:
                    fails.append("命令链断：spin_speed=0 时 /cmd_vel_chassis 的角速度应等于 nav 的")
                if n.gt_twist_max < TH["mv_eps"] and n.gt_pose_delta < 0.02:
                    fails.append("真值没动：指令下去了但车没动（底盘/物理侧）")

    # ---------------- 汇总 ----------------
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    ok = not fails
    os.makedirs(a.outdir, exist_ok=True)
    out = os.path.join(a.outdir, f"regress_{int(time.time())}.json")
    snap = {"pass": ok, "fails": fails, "notes": notes, "rtf": rtf, "hz": hz,
            "fp_distinct": fp_distinct, "tf_age": age, "cmd": n.cmd,
            "spin_speed": n.spin_speed, "d_min": n.d_min,
            "gt_twist_max": n.gt_twist_max, "gt_pose_delta": n.gt_pose_delta,
            "goal": a.goal, "skip_goal": a.skip_goal}
    with open(out, "w") as f:
        json.dump(snap, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 72)
    print("P0 回归：" + ("✅ PASS" if ok else "❌ FAIL"))
    for x in notes:
        print("  · " + x)
    for x in fails:
        print("  ✗ " + x)
    print(f"  首个断点：{fails[0] if fails else '无'}")
    print(f"  快照 -> {out}（可抄进 docs/algorithm_matrix.md §四）")
    print("=" * 72)
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[中断] 未完成，结论无效")
