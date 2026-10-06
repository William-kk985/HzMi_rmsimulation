#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""「没撞击却飘里程计」的**只读**时间轴记录仪（50 Hz 状态行 + 逐消息事件行）。

它专治一个问题：用户报告"车开得好好的、没撞、LIO 里程计自己跑了"。
要判定这件事，必须把下面这些量**放在同一条时间轴**上（缺任何一个都会变成猜）：

  ① `/odom`（LIO）**逐帧**位置/偏航 → 单步位移 `step` 与隐含速度 `step/dt`（数值爆掉的直接证据）；
  ② `/odom_ground_truth`（Gazebo 真值）→ 与之对比才知道"是不是真的飘了"（对齐在离线做）；
  ③ 指令角速度：`/cmd_vel`(smoother 输出) / `/cmd_vel_nav`(MPPI) / `/cmd_vel_chassis`(底盘级)
     —— 候选①「高角速度甩转」的**自变量**；
  ④ `/scan`：有限束数、**前向 ±20° 最近回波**、**后向 ±20° 最近回波**、全向最小
     —— 候选②「慢速贴墙/被挡但 IMU 无辜」的直接观测（车尾后面的坡脚只有后向窗口看得见）；
  ⑤ `/livox/imu`：|a| / 分量 / |ω| —— 区分「有撞击（IMU 尖峰）」与「没撞击」；
  ⑥ 定位健康：`~/fitness_score`、`~/converged`、`map→odom` **逐次更新步长**（发散幅度）；
  ⑦ 限速器：`/speed_limit`（nav2 SpeedLimit，0=不限速）+ `~/slope_speed_stats`（限速决策 JSON）；
  ⑧ 恢复行为：`/navigate_to_pose/_action/feedback` 的 `number_of_recoveries`
     （行为**类型** Spin/BackUp/Wait 由 runner 从 launch.log 的 `Running <plugin>` 行取，带墙钟戳）。

⚠️ 采样饥饿陷阱（本仓踩过，见 docs/worlds.md §5.1 / docs/lio_drift_diagnosis.md §3.2）：
  单线程 `rclpy.spin_once()` 在 ~80 Hz 的 `/tf` 洪水下会被饿死 ⇒ 拿到十几秒前的旧值
  ⇒ 会得出"LIO 位移是真值的 2.6 倍"这种**假结论**。本工具从根上避开：
    · `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup`；
    · 回调里只做 O(1) 赋值 + 往内存队列 append（**不做 TF lookup、不解析点云、不写文件**）；
    · 落盘由独立 50 Hz 定时器做（只读"最新值"快照 + 排空事件队列）；
    · 每条记录都带 `*_age_sim` / `*_age_wall`（这条数据比"现在"旧多少）——真被饿死看得出来。

**纯只读**：本节点不发布任何话题、不发 TF、不写参数文件、不发目标、最后也不发零速
（发目标与零速收尾由 `nav_clearance_probe.py` 负责，见 run_nav_lio_timeline.sh）。

产物（两份 JSONL，键都是短名）：
  --out       50 Hz 状态行：每次一行"当时最新的"快照（含 counter 与 age）
  --events    逐消息事件行：`odom`/`gt`（含单步 step 与速度）/`scan`/`imu`（仅 |a|>阈值）/`mo`/`sl`/`conv`/`recov`

用法（必须与 launch 塞进同一次 bash 调用，见 run_nav_lio_timeline.sh 头注）：
    python3 tools/scripts/diag/nav_lio_timeline.py --tag t1 \
        --out .tmp_lio_noimpact/out/t1/rows.jsonl \
        --events .tmp_lio_noimpact/out/t1/events.jsonl --hz 50
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String
from tf2_msgs.msg import TFMessage

try:                                     # nav2 msgs（栈里必然有，缺了也不致命）
    from nav2_msgs.msg import SpeedLimit
except Exception:                        # pragma: no cover
    SpeedLimit = None
try:                                     # 直接订 action 的 feedback 话题（**不发目标**）
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
except Exception:                        # pragma: no cover
    NavigateToPose_FeedbackMessage = None

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
LAT1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

CHAIN = ("map->odom", "odom->base_link", "base_link->base_link_fake", "odom->base_link_fake")
FWD_DEG = 20.0          # 「前向」窗口半角
BACK_DEG = 20.0         # 「后向」窗口半角
IMU_EVT = 25.0          # |a| 超过它才单独记一条事件行（m/s²）
QMAX = 30000            # 事件队列上限（超了丢最旧并计数，绝不让内存/落盘拖慢回调）


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def now():
    return time.time()


def stamp_of(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Timeline(Node):
    """只读记录仪：回调 O(1) 存值 + append，定时器落盘。"""

    def __init__(self, a):
        super().__init__("nav_lio_timeline")
        self.a = a
        self.grp = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.t0 = now()
        self.wall0 = now()
        self.clock = None
        self.clock0 = None
        self.cnt = collections.Counter()
        self.drops = collections.Counter()

        # ---- 最新值（回调只写这些） ----
        self.lio = None            # (stamp, x, y, z, yaw, vx, vy, wz, wall)
        self.gt = None             # 同上
        self.tf = {}               # key -> (stamp, (x,y,z), (qx,qy,qz,qw), wall)
        self.tf_seen = set()
        self.cmd = {}              # name -> (vx, vy, wz, wall)
        self.scan = None           # (stamp, n, nfin, fwd_min, back_min, amin, wall)
        self.imu = None            # (stamp, |a|, ax, ay, az, |w|, wall)
        self.imu_win = 0.0         # 本采样窗口内 |a| 峰值（峰值保持，落盘后清零）
        self.imu_win_t = None
        self.fit = None
        self.conv = None
        self.sl = None             # (stamp, limit, wall)
        self.slope = None          # (stamp, json_str, wall)
        self.fb = {"recov": 0, "dist": None, "selapsed": None, "wall": None}
        self.mo_stamp_wall = None
        self.mo_age_max = 0.0

        # ---- 事件队列（定时器排空） ----
        self.q = collections.deque()
        self.fh_rows = open(a.out, "w")
        self.fh_ev = open(a.events, "w") if a.events else None

        g = self.grp
        self.create_subscription(Clock, "/clock", self.on_clock, BEST, callback_group=g)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI, callback_group=g)
        self.create_subscription(Odometry, "/odom", self.on_lio, RELI, callback_group=g)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, RELI, callback_group=g)
        for t in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_chassis"):
            self.create_subscription(Twist, t, self.on_cmd(t), RELI, callback_group=g)
        self.create_subscription(LaserScan, "/scan", self.on_scan, BEST, callback_group=g)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, BEST, callback_group=g)
        self.create_subscription(Float64, "/gicp_registration/fitness_score",
                                 self.on_fit, RELI, callback_group=g)
        self.create_subscription(Bool, "/gicp_registration/converged", self.on_conv, RELI,
                                 callback_group=g)
        if SpeedLimit is not None:
            self.create_subscription(SpeedLimit, "/speed_limit", self.on_sl, RELI,
                                     callback_group=g)
        self.create_subscription(String, "/ground_segmentation/slope_speed_stats",
                                 self.on_slope, BEST, callback_group=g)
        if NavigateToPose_FeedbackMessage is not None:
            self.create_subscription(NavigateToPose_FeedbackMessage,
                                     "/navigate_to_pose/_action/feedback", self.on_fb, BEST,
                                     callback_group=g)
        self.timer = self.create_timer(1.0 / max(1.0, a.hz), self.tick, callback_group=g)

    # ------------------------------------------------------------------ 回调
    def _push(self, rec):
        q = self.q
        if len(q) >= QMAX:
            self.drops["q_full"] += 1
            try:
                q.popleft()
            except IndexError:
                pass
        q.append(rec)

    def on_clock(self, m):
        c = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock = c
        if self.clock0 is None:
            self.clock0, self.wall0 = c, now()
        self.cnt["clock"] += 1

    def on_tf(self, m):
        for t in m.transforms:
            k = "%s->%s" % (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
            if k not in CHAIN:
                continue
            tr, q = t.transform.translation, t.transform.rotation
            self.tf[k] = (stamp_of(t.header), (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w), now())
            self.tf_seen.add(k)
            if k == "map->odom":
                self.mo_stamp_wall = now()
                self.cnt["map->odom"] += 1
        self.cnt["tf"] += 1

    def on_lio(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist
        st = stamp_of(m.header)
        self.lio = (st, p.x, p.y, p.z, yaw_of((q.x, q.y, q.z, q.w)),
                    v.linear.x, v.linear.y, v.angular.z, now())
        self.cnt["odom"] += 1
        self._push({"k": "odom", "t": round(st, 5), "w": round(now() - self.t0, 4),
                    "x": round(p.x, 4), "y": round(p.y, 4), "yaw": round(self.lio[4], 5),
                    "vx": round(v.linear.x, 4), "vy": round(v.linear.y, 4),
                    "wz": round(v.angular.z, 4)})

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist
        st = stamp_of(m.header)
        self.gt = (st, p.x, p.y, p.z, yaw_of((q.x, q.y, q.z, q.w)),
                   v.linear.x, v.linear.y, v.angular.z, now())
        self.cnt["gt"] += 1
        self._push({"k": "gt", "t": round(st, 5), "w": round(now() - self.t0, 4),
                    "x": round(p.x, 4), "y": round(p.y, 4), "yaw": round(self.gt[4], 5),
                    "vx": round(v.linear.x, 4), "vy": round(v.linear.y, 4),
                    "wz": round(v.angular.z, 4)})

    def on_cmd(self, topic):
        name = topic.strip("/").replace("cmd_vel", "cmd").strip("_") or "cmd"

        def f(m):
            self.cmd[name] = (m.linear.x, m.linear.y, m.angular.z, now())
            self.cnt["cmd:" + topic] += 1
        return f

    def on_scan(self, m):
        n = len(m.ranges)
        nfin = 0
        fwd = back = allmin = None
        amin = 0.0
        inc = m.angle_increment
        lo = m.angle_min
        fwd_half = math.radians(FWD_DEG)
        back_half = math.radians(BACK_DEG)
        for i, r in enumerate(m.ranges):
            if not (r == r) or r <= 0.0 or math.isinf(r):     # NaN/0/inf 都跳过（不引 numpy）
                continue
            nfin += 1
            a = lo + i * inc
            if allmin is None or r < allmin:
                allmin, amin = r, a
            aa = wrap(a)
            if -fwd_half <= aa <= fwd_half:
                fwd = r if fwd is None else min(fwd, r)
            if abs(aa) >= math.pi - back_half:
                back = r if back is None else min(back, r)
        st = stamp_of(m.header)
        self.scan = (st, n, nfin, fwd, back, amin, now())
        self.cnt["scan"] += 1
        self._push({"k": "scan", "t": round(st, 5), "w": round(now() - self.t0, 4),
                    "n": n, "nfin": nfin,
                    "fwd": None if fwd is None else round(fwd, 3),
                    "back": None if back is None else round(back, 3),
                    "min": None if allmin is None else round(allmin, 3),
                    "min_deg": round(math.degrees(amin), 1)})

    def on_imu(self, m):
        a = m.linear_acceleration
        w = m.angular_velocity
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        wmag = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        st = stamp_of(m.header)
        self.imu = (st, mag, a.x, a.y, a.z, wmag, now())
        self.cnt["imu"] += 1
        if mag > self.imu_win:
            self.imu_win, self.imu_win_t = mag, st
        if mag >= IMU_EVT:
            fus = self.fused()
            self._push({"k": "imu", "t": round(st, 5), "w": round(now() - self.t0, 4),
                        "a": round(mag, 2), "ax": round(a.x, 2), "ay": round(a.y, 2),
                        "az": round(a.z, 2), "w": round(wmag, 3),
                        "fx": None if fus is None else round(fus[0], 3),
                        "fy": None if fus is None else round(fus[1], 3),
                        "vx": None if self.gt is None else round(self.gt[5], 3)})

    def on_fit(self, m):
        self.fit = m.data
        self.cnt["fitness"] += 1

    def on_conv(self, m):
        v = bool(m.data)
        if self.conv is None or v != self.conv:
            self._push({"k": "conv", "t": round(self.clock or 0.0, 3),
                        "w": round(now() - self.t0, 4), "v": v})
        self.conv = v
        self.cnt["converged"] += 1

    def on_sl(self, m):
        lim = float(getattr(m, "speed_limit", 0.0))
        pct = bool(getattr(m, "percentage", False))
        if self.sl is None or abs(lim - self.sl[1]) > 1e-3:
            self._push({"k": "sl", "t": round(self.clock or 0.0, 3),
                        "w": round(now() - self.t0, 4), "limit": round(lim, 3),
                        "pct": pct})
        self.sl = (self.clock, lim, now())
        self.cnt["speed_limit"] += 1

    def on_slope(self, m):
        self.slope = (self.clock, m.data, now())
        self.cnt["slope_stats"] += 1

    def on_fb(self, m):
        fb = m.feedback
        nr = int(getattr(fb, "number_of_recoveries", 0))
        if nr != self.fb["recov"]:
            self._push({"k": "recov", "t": round(self.clock or 0.0, 3),
                        "w": round(now() - self.t0, 4), "n": nr,
                        "dist": round(float(getattr(fb, "distance_remaining", -1.0)), 3)})
        self.fb = {"recov": nr, "dist": float(getattr(fb, "distance_remaining", -1.0)),
                   "selapsed": float(getattr(fb, "navigation_time", None).sec
                                     if getattr(fb, "navigation_time", None) else -1.0),
                   "wall": now()}
        self.cnt["fb"] += 1

    # ------------------------------------------------------------------ 复合位姿
    def pose_of(self, key):
        v = self.tf.get(key)
        if v is None:
            return None
        return (v[1][0], v[1][1], yaw_of(v[2]))

    def fused(self):
        mo, ob = self.pose_of("map->odom"), self.pose_of("odom->base_link")
        if mo is None or ob is None:
            return None
        c, s = math.cos(mo[2]), math.sin(mo[2])
        return (mo[0] + c * ob[0] - s * ob[1], mo[1] + s * ob[0] + c * ob[1],
                wrap(mo[2] + ob[2]))

    def age(self, t):
        if t is None or self.clock is None:
            return None
        return round(self.clock - t, 4)

    def rtf(self):
        if self.clock is None or self.clock0 is None:
            return None
        dw = now() - self.wall0
        return round((self.clock - self.clock0) / dw, 3) if dw > 1e-6 else None

    # ------------------------------------------------------------------ 落盘
    def tick(self):
        try:
            self._tick()
        except Exception as ex:                       # noqa: BLE001 单行异常不毁整次跑
            self.cnt["tick_err"] += 1
            if self.cnt["tick_err"] <= 3:
                import traceback
                traceback.print_exc()
            self.cnt["tick_err_msg:" + type(ex).__name__] += 1

    def _tick(self):
        w = now() - self.t0
        # 排空事件队列（带前一条 odom/gt 的"单步"计算）
        if self.fh_ev is not None:
            with self.lock:
                evs = list(self.q)
                self.q.clear()
            prev = getattr(self, "_prev", None) or {}
            for e in evs:
                k = e["k"]
                if k in ("odom", "gt"):
                    p = prev.get(k)
                    if p is not None:
                        dt = e["t"] - p[0]
                        st = math.hypot(e["x"] - p[1], e["y"] - p[2])
                        e["dt"] = round(dt, 4)
                        e["step"] = round(st, 4)
                        e["spdyaw"] = round(wrap(e["yaw"] - p[3]), 5)
                        if dt > 1e-4:
                            e["spd"] = round(st / dt, 3)
                    prev[k] = (e["t"], e["x"], e["y"], e["yaw"])
                self.fh_ev.write(json.dumps(e, ensure_ascii=False) + "\n")
            self._prev = prev
        else:
            with self.lock:
                self.q.clear()

        # 状态行
        row = {"w": round(w, 4), "t": None if self.clock is None else round(self.clock, 4),
               "rtf": self.rtf()}
        lio = self.lio
        if lio:
            row["lio"] = [round(lio[1], 4), round(lio[2], 4), round(lio[4], 5),
                          round(lio[5], 4), round(lio[6], 4), round(lio[7], 4)]
            row["lio_age_sim"] = self.age(lio[0])
            row["lio_age_wall"] = round(now() - lio[8], 4)
        gt = self.gt
        if gt:
            row["gt"] = [round(gt[1], 4), round(gt[2], 4), round(gt[4], 5),
                         round(gt[5], 4), round(gt[6], 4), round(gt[7], 4)]
            row["gt_age_sim"] = self.age(gt[0])
        mo = self.tf.get("map->odom")
        if mo:
            row["mo"] = [round(mo[1][0], 4), round(mo[1][1], 4), round(yaw_of(mo[2]), 5)]
            row["mo_age_sim"] = self.age(mo[0])
            row["mo_age_wall"] = round(now() - mo[3], 3)
        ob = self.tf.get("odom->base_link")
        if ob:
            row["ob"] = [round(ob[1][0], 4), round(ob[1][1], 4), round(yaw_of(ob[2]), 5)]
            row["ob_age_sim"] = self.age(ob[0])
        fus = self.fused()
        if fus:
            row["fus"] = [round(fus[0], 4), round(fus[1], 4), round(fus[2], 5)]
        cmd = {}
        for k, v in self.cmd.items():
            cmd[k] = [round(v[0], 3), round(v[1], 3), round(v[2], 3)]
        if cmd:
            row["cmd"] = cmd
        sc = self.scan
        if sc:
            row["scan"] = [sc[1], sc[2], None if sc[3] is None else round(sc[3], 3),
                           None if sc[4] is None else round(sc[4], 3)]
            row["scan_age_sim"] = self.age(sc[0])
        im = self.imu
        if im:
            row["imu_a"] = round(im[1], 2)
            row["imu_w"] = round(im[5], 3)
            row["imu_age_sim"] = self.age(im[0])
        if self.imu_win > 0:
            row["imu_a_peak"] = round(self.imu_win, 2)
            row["imu_a_peak_t"] = None if self.imu_win_t is None else round(self.imu_win_t, 3)
            self.imu_win, self.imu_win_t = 0.0, None
        if self.fit is not None:
            row["fit"] = round(float(self.fit), 5)
        if self.conv is not None:
            row["conv"] = self.conv
        if self.sl is not None:
            row["sl"] = round(self.sl[1], 3)
        if self.slope is not None:
            row["slope_age_wall"] = round(now() - self.slope[2], 3)
            try:                                     # 只留决策那几个键，别把逐格数组写进来
                d = json.loads(self.slope[1])
                row["sl_dec"] = {k: d[k] for k in
                                 ("limit", "why", "d", "dtan", "slope", "step", "v_req", "dir")
                                 if k in d}
            except Exception:                        # noqa: BLE001
                pass
        row["recov"] = self.fb["recov"]
        if self.fb["dist"] is not None:
            row["dist_rem"] = round(self.fb["dist"], 3)
        if self.mo_stamp_wall is not None:
            age = now() - self.mo_stamp_wall
            self.mo_age_max = max(self.mo_age_max, age)
            row["mo_pub_age"] = round(age, 3)
        row["cnt"] = dict(self.cnt)
        if self.drops:
            row["drops"] = dict(self.drops)
        self.fh_rows.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.cnt["rows"] += 1

    def summary(self):
        return {"tag": self.a.tag, "rows": self.cnt.get("rows", 0), "cnt": dict(self.cnt),
                "drops": dict(self.drops), "tf_seen": sorted(self.tf_seen),
                "mo_age_max_s": round(self.mo_age_max, 3),
                "t_wall0": self.t0, "clock0": self.clock0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="run")
    ap.add_argument("--out", required=True, help="50 Hz 状态行 JSONL")
    ap.add_argument("--events", default="", help="逐消息事件行 JSONL")
    ap.add_argument("--summary", default="", help="汇总 JSON")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--run-sec", type=float, default=0.0,
                    help=">0 ⇒ 最多录这么久（墙钟秒）后自行退出；0 = 一直录到 SIGINT/SIGTERM")
    ap.add_argument("--ready-timeout", type=float, default=300.0)
    a = ap.parse_args()
    for p in (a.out, a.events, a.summary):
        if p:
            os.makedirs(os.path.dirname(os.path.abspath(p)) or ".", exist_ok=True)

    rclpy.init()
    n = Timeline(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    print("[tl] 记录开始（hz=%.0f）→ %s" % (a.hz, a.out), flush=True)

    t0 = now()
    while now() - t0 < a.ready_timeout and n.lio is None:
        time.sleep(0.5)
    print("[tl] /odom 就绪用时 %.1fs（tf=%s）" % (now() - t0, ",".join(sorted(n.tf_seen))),
          flush=True)

    try:
        if a.run_sec > 0:
            end = now() + a.run_sec
            while now() < end:
                time.sleep(0.5)
        else:
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    n.fh_rows.flush()
    if n.fh_ev:
        n.fh_ev.flush()
    s = n.summary()
    if a.summary:
        with open(a.summary, "w") as f:
            json.dump(s, f, ensure_ascii=False, indent=2)
    print("[tl] 结束：rows=%d cnt=%s drops=%s mo_age_max=%.3fs"
          % (s["rows"], s["cnt"], s["drops"], s["mo_age_max_s"]), flush=True)
    n.fh_rows.close()
    if n.fh_ev:
        n.fh_ev.close()
    try:
        ex.shutdown()
    except Exception:                            # noqa: BLE001
        pass
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:                            # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[tl] 中断")
        sys.exit(2)
