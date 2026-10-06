#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""「贴膨胀边走 + 撞进去 + 定位跟着坏」的实测记录仪（只读仪器）。

回答 docs/path_clearance_and_contact.md 需要的四个量：

  ① **执行路径的余量（clearance）**：机器人中心到「致命格(cost 254)」/「内切带格(cost>=253)」/
     「膨胀带格(cost>0)」的**欧氏距离**（米），从**同期发布的两张代价地图**上现算
     （global: map 系 + static/inflation；local: odom 系 + scan/inflation）。
     减去 robot_radius(0.22) ⇒ 足迹边缘余量；<=0 = 足迹压进内切带。
     · 两个位姿各算一份：**融合位姿**（= nav2 真正看到的 map→odom∘odom→base_link_fake）与
       **真值位姿对齐到 map**（冻结开机时的 map→odom 对齐，/odom_ground_truth 直接给 odom 系真值）
       —— 前者含定位误差，后者不含；两者之差就是"定位误差把车放到哪去了"。
  ② **接触 / 被推 / 打滑**：底盘是 `gazebo_ros_planar_move`（**运动学**插件：它每个周期直接设模型
     速度），所以"真值位移 ≠ 指令位移"只可能来自物理引擎的接触修正 ⇒ 残差就是接触/被推的直接量。
     逐 0.1 s 算 `|Δgt - R(yaw)·cmd·dt|`，分两类事件：blocked（被挡住）/ pushed（没指令却在动）。
     另有独立佐证：`/scan` 最小距离（真实世界的墙到雷达的距离，与地图无关）。
  ③ **定位事件时间轴**：`map→odom` 的**发布停顿**（GICP 判失效后停发 ⇒ TF 变陈旧）、
     `~/fitness_score`、`~/converged`；launch 日志里的拒帧/失效行由 runner 打时间戳后另存。
  ④ **规划质量**：`/plan` 的最小余量（同一张 global 代价地图上的 EDT）、路径长度、与
     nav2 的 action feedback（剩余距离 / recovery 次数 / 导航耗时）。

输出：`--out` JSONL 逐样本（10 Hz）+ `--summary` JSON（事件、分布、结局）。

用法（**必须与 launch 塞进同一次 bash 调用**，见 run_nav_clearance_ab.sh 头注）：
    python3 tools/scripts/regress/nav_clearance_probe.py \
        --tag base --out .tmp_clear/out/base/samples.jsonl --summary .tmp_clear/out/base/summary.json \
        --goal -12.64 -0.31 --settle 8 --max-drive-sec 90 --post-goal-sec 20
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage

try:
    from scipy import ndimage as _ndi
except Exception:  # pragma: no cover
    _ndi = None

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

CHAIN = ("map->odom", "odom->base_link", "base_link->base_link_fake",
         "odom->base_link_fake", "map->base_link_fake")

LETHAL = 254          # nav2_costmap_2d::LETHAL_OBSTACLE
INSCRIBED = 253       # nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE（= 内切带，半径 robot_radius）
NO_INFO = 255         # nav2_costmap_2d::NO_INFORMATION


def canon_costs(a):
    """把 /{global,local}_costmap/costmap 的 OccupancyGrid 值还原成 nav2 原始代价 0..255。

    ⚠️ 这个话题**不是原始代价**：`Costmap2DPublisher::prepareGrid()` 过一张 translation table
    （costmap_2d_publisher.cpp:87-99）把 253→**99**、254→**100**、255→**-1**、1..252→1..98。
    直接按 253/254 判定 ⇒ **永远判不出致命格**（本探针第一版踩过：d_lethal 恒等于"到数组边界的距离"，
    26 m 这种离谱值就是这么来的）。这里按"出现 -1 或 max∈[99,100] 且 max<=100"识别编码并还原。
    """
    a = a.astype(np.int16)
    looks_translated = bool((a < 0).any()) or (99 <= int(a.max()) <= 100)
    if not looks_translated:
        return a
    out = a.copy()
    out[a < 0] = NO_INFO
    out[a == 100] = LETHAL
    out[a == 99] = INSCRIBED
    return out


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(a, b):
    """(x,y,yaw) ∘ (x,y,yaw)"""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1], a[1] + sa * b[0] + ca * b[1], wrap(a[2] + b[2]))


def inv(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-(ca * a[0] + sa * a[1]), -(-sa * a[0] + ca * a[1]), wrap(-a[2]))


class Grid:
    """一张代价地图 + 三张距离场（米）。距离 = 到最近"满足条件"的格的欧氏距离。"""

    __slots__ = ("res", "ox", "oy", "w", "h", "frame", "stamp", "arr",
                 "d_lethal", "d_insc", "d_infl", "wall")

    def __init__(self, msg: OccupancyGrid):
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.w = msg.info.width
        self.h = msg.info.height
        self.frame = msg.header.frame_id
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.wall = time.time()
        a = canon_costs(np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w))
        self.arr = a
        if _ndi is None:
            self.d_lethal = self.d_insc = self.d_infl = None
            return
        # EDT：给出每个格到最近 True 格的距离（单位=格），×res ⇒ 米。
        # 未知格(255) **不算障碍**（nav2 的 allow_unknown/track_unknown_space 语义：不是"墙"）
        self.d_lethal = _ndi.distance_transform_edt((a != LETHAL) & (a != NO_INFO)) * self.res
        self.d_insc = _ndi.distance_transform_edt((a < INSCRIBED) & (a != NO_INFO)) * self.res
        self.d_infl = _ndi.distance_transform_edt((a <= 0) | (a == NO_INFO)) * self.res

    def cell(self, x, y):
        c = int((x - self.ox) / self.res)
        r = int((y - self.oy) / self.res)
        if not (0 <= c < self.w and 0 <= r < self.h):
            return None
        return r, c

    def probe(self, x, y):
        """(d_lethal, d_insc, d_infl, cost) —— 米/原始代价；出界返回 None。"""
        rc = self.cell(x, y)
        if rc is None:
            return None
        r, c = rc
        if self.d_lethal is None:
            return None
        return (float(self.d_lethal[r, c]), float(self.d_insc[r, c]),
                float(self.d_infl[r, c]), int(self.arr[r, c]))

    def min_over(self, pts, field):
        """一组 map 系点上的最小距离（用于 /plan 的最小余量）。"""
        if self.d_lethal is None or not pts:
            return None
        f = {"lethal": self.d_lethal, "insc": self.d_insc, "infl": self.d_infl}[field]
        best = None
        for x, y in pts:
            rc = self.cell(x, y)
            if rc is None:
                continue
            v = float(f[rc[0], rc[1]])
            best = v if best is None else min(best, v)
        return best


class Probe(Node):
    def __init__(self, a):
        super().__init__("nav_clearance_probe")
        self.a = a
        self.fh = open(a.out, "w")
        self.t_wall0 = time.time()
        self.clock = None
        self.clock0 = None
        self.cnt = {}
        self.tf = {}          # key -> (stamp_sim, (x,y,z), (qx,qy,qz,qw))
        self.mo_stamp = None  # map->odom 的最新消息时间（墙钟）
        self.mo_age_max = 0.0
        self.gt = None        # (stamp, (x,y,yaw), (vx,vy,wz))
        self.ob_prev = None
        self.mo_prev = None
        self.gt_step = None
        self.cmd = {"nav": (0.0, 0.0, 0.0, 0.0), "chassis": (0.0, 0.0, 0.0, 0.0)}
        self.cmd_prev = None
        self.gg = None        # global Grid
        self.lg = None        # local Grid
        self.scan = None      # (stamp, rmin, amin_deg)
        self.imu = None       # (stamp, |a|, wz)
        self.health = {"fitness": None, "converged": None}
        self.plan = None
        self.plan_t = None
        self.plan_metrics = {}
        self.n_rows = 0
        self.t0_align = None  # 冻结对齐：map->odom 的启动均值
        self._align_buf = []
        self.align = None
        self.events = []      # 时间轴事件（goal/recovery/lost/..）
        self.fb = {"recov": 0, "dist": None, "nav_time": None}
        self.goal_state = None
        self.rows = []        # 逐样本（内存里也留一份，便于算分布）
        self.contact = []     # 事件
        self.blocked = []
        self.pushed = []
        self.mo_stale_runs = []
        self.travel_gt = 0.0
        self.travel_cmd = 0.0

        self.create_subscription(Clock, "/clock", self.on_clock, BEST)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_g, LATCHED)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_l, LATCHED)
        self.create_subscription(Path, "/plan", self.on_plan, RELI)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, RELI)
        self.create_subscription(LaserScan, "/scan", self.on_scan, BEST)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, BEST)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd("nav"), RELI)
        self.create_subscription(Twist, "/cmd_vel_chassis", self.on_cmd("chassis"), RELI)
        self.create_subscription(Float64, "/gicp_registration/fitness_score",
                                 lambda m: self.health.__setitem__("fitness", m.data), RELI)
        self.create_subscription(Bool, "/gicp_registration/converged",
                                 lambda m: self.health.__setitem__("converged", m.data), LATCHED)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_chassis", RELI)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------------ 回调
    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock0 is None:
            self.clock0 = self.clock

    def on_tf(self, m):
        for t in m.transforms:
            k = "%s->%s" % (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
            if k not in CHAIN:
                continue
            s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            tr, q = t.transform.translation, t.transform.rotation
            self.tf[k] = (s, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
            if k == "map->odom":
                self.mo_stamp = time.time()
                self.cnt["map->odom"] = self.cnt.get("map->odom", 0) + 1

    def on_g(self, m):
        try:
            self.gg = Grid(m)
            self.cnt["global_costmap"] = self.cnt.get("global_costmap", 0) + 1
        except Exception as ex:      # noqa: BLE001
            self.cnt["global_costmap_err"] = str(ex)

    def on_l(self, m):
        try:
            self.lg = Grid(m)
            self.cnt["local_costmap"] = self.cnt.get("local_costmap", 0) + 1
        except Exception as ex:      # noqa: BLE001
            self.cnt["local_costmap_err"] = str(ex)

    def on_plan(self, m):
        self.plan = m
        self.plan_t = time.time()
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        L = 0.0
        for i in range(1, len(pts)):
            L += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        self.plan_metrics = {"n": len(pts), "len": L, "wall": self.plan_t}
        if self.gg is not None:
            self.plan_metrics["min_lethal"] = self.gg.min_over(pts, "lethal")
            self.plan_metrics["min_insc"] = self.gg.min_over(pts, "insc")
        self.events.append({"wall": self.plan_t, "sim": self.clock, "kind": "plan",
                            "len": round(L, 3),
                            "min_lethal": _r(self.plan_metrics.get("min_lethal")),
                            "min_insc": _r(self.plan_metrics.get("min_insc"))})

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pose = (p.x, p.y, yaw_of((q.x, q.y, q.z, q.w)))
        # ---- 接触/被推/打滑：真值位移 vs 指令位移 ----
        # 底盘是 golang_ros_planar_move（strings 实证：Model::SetLinearVel/SetAngularVel，
        # **速度控制**、不是 SetWorldPose 的瞬移）⇒ 位移与指令不符只可能来自物理接触修正。
        # 速度平滑器限幅 4 m/s² ⇒ 位置滞后上界 0.5·4·dt² = 0.02 m（dt=0.1 s）≪ 0.05 m 门限。
        if self.gt is not None:
            st0, p0, _ = self.gt
            dt = st - st0
            if 0.02 < dt < 0.5:
                dx, dy = pose[0] - p0[0], pose[1] - p0[1]
                step = math.hypot(dx, dy)
                cvx, cvy = (self.cmd_prev[0], self.cmd_prev[1]) if self.cmd_prev else (0.0, 0.0)
                ca, sa = math.cos(p0[2]), math.sin(p0[2])
                ex, ey = (ca * cvx - sa * cvy) * dt, (sa * cvx + ca * cvy) * dt
                exp = math.hypot(ex, ey)
                res = math.hypot(dx - ex, dy - ey)
                self.travel_gt += step
                self.travel_cmd += exp
                self.gt_step = {"st": st, "step": step, "exp": exp, "res": res,
                                "x": pose[0], "y": pose[1], "dt": dt,
                                "cmd": (cvx, cvy), "yaw0": p0[2], "dx": dx, "dy": dy}
                self.cmd_prev = self.cmd["chassis"]
                # 事件判定：被挡住（位移远小于指令）/ 没指令却在动（被推）
                if exp > 0.05 and step < 0.35 * exp and res > 0.05:
                    self._push(self.blocked, self.gt_step, res, "blocked")
                elif exp <= 0.02 and step > 0.025 and res > 0.025:
                    self._push(self.pushed, self.gt_step, res, "pushed")
        self.gt = (st, pose, (v.linear.x, v.linear.y, v.angular.z))

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        r = r[np.isfinite(r) & (r > 0.0)]
        if r.size == 0:
            return
        i = int(np.argmin(r))
        self.scan = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     float(r[i]), math.degrees(m.angle_min + i * m.angle_increment))

    def on_imu(self, m):
        a = m.linear_acceleration
        self.imu = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z), m.angular_velocity.z)

    def on_cmd(self, key):
        def f(m):
            self.cmd[key] = (m.linear.x, m.linear.y, m.angular.z, time.time())
        return f

    # ------------------------------------------------------------------ 位姿
    def pose_of(self, key):
        v = self.tf.get(key)
        if v is None:
            return None
        return (v[1][0], v[1][1], yaw_of(v[2]))

    def fused(self):
        """map 系 nav2 看到的位姿（map->base_link_fake，按段组合）。"""
        p = self.pose_of("map->base_link_fake")
        if p is not None:
            return p
        mo = self.pose_of("map->odom")
        ob = self.odom_base_fake()
        if mo is None or ob is None:
            return None
        return compose(mo, ob)

    def odom_base_fake(self):
        """odom 系 nav2 的 robot_base_frame（base_link_fake）位姿。

        本仓库 TF 链是**逐段**的（map→odom 归 GICP、odom→base_link 归 LIO、
        base_link→base_link_fake 归 fake_vel_transform），没有一条 "odom->base_link_fake"
        的直接边 ⇒ 这里按段组合（spin_speed:=0.0 时最后一段是单位变换）。
        """
        p = self.pose_of("odom->base_link_fake")
        if p is not None:
            return p
        ob = self.pose_of("odom->base_link")
        if ob is None:
            return None
        sp = self.pose_of("base_link->base_link_fake")
        return ob if sp is None else compose(ob, sp)

    # ------------------------------------------------------------------ 采样
    def tick(self):
        try:
            self._tick()
        except Exception as ex:      # noqa: BLE001  —— 单行异常不允许毁掉整次跑
            self.cnt["tick_err"] = "%s: %s" % (type(ex).__name__, ex)
            if self.cnt.get("tick_err_n", 0) < 5:
                self.cnt["tick_err_n"] = self.cnt.get("tick_err_n", 0) + 1
                import traceback
                traceback.print_exc()

    def _tick(self):
        if self.clock is None or self.gg is None:
            return
        now_w = time.time()
        fuse = self.fused()
        ob = self.odom_base_fake()
        gt = self.gt
        # 真值进 map 系的常量对齐：**实测发现 /odom_ground_truth 给的是 Gazebo 世界系位姿**
        #   （planar_move 直接抄 model->WorldPose()），而 map 系 = 出生点相对系
        #   ⇒ 两者差一个常量刚体变换（本场地 = 出生点世界坐标 ≈ (10.93, 2.52)）。
        #   冻结方式：静置前 3 s 里取 median( 融合位姿 ∘ inv(真值) )（此时真值位姿=出生点位姿、
        #   融合位姿按 initial_pose 就是 map 系出生点位姿）⇒ 之后 gt_map = align ∘ gt。
        if self.align is None and fuse is not None and gt is not None \
                and self.clock - (self.clock0 or 0) < 5.0:
            self._align_buf.append(compose(fuse, inv(gt[1])))
            if len(self._align_buf) >= 20:
                self.align = (float(np.median([p[0] for p in self._align_buf])),
                              float(np.median([p[1] for p in self._align_buf])),
                              float(np.median([p[2] for p in self._align_buf])))
        if self.align is None and fuse is not None and gt is not None \
                and self.clock - (self.clock0 or 0) >= 5.0:
            self.align = compose(fuse, inv(gt[1]))
        gt_map = None
        if gt is not None and self.align is not None:
            gt_map = compose(self.align, gt[1])

        row = {"t": round(self.clock, 3), "wall": round(now_w - self.t_wall0, 3),
               "wall_abs": round(now_w, 3)}
        if fuse:
            row["fx"], row["fy"], row["fyaw"] = (round(fuse[0], 4), round(fuse[1], 4),
                                                 round(fuse[2], 5))
        mo = self.pose_of("map->odom")
        if mo is not None:
            row["mox"], row["moy"], row["moyaw"] = (round(mo[0], 4), round(mo[1], 4),
                                                    round(mo[2], 5))
            if self.mo_prev is not None:
                row["mo_step"] = round(math.hypot(mo[0] - self.mo_prev[0],
                                                  mo[1] - self.mo_prev[1]), 4)
            self.mo_prev = mo
        if ob is not None:
            row["ox"], row["oy"], row["oyaw"] = (round(ob[0], 4), round(ob[1], 4),
                                                 round(ob[2], 5))
            if self.ob_prev is not None:
                row["lio_step"] = round(math.hypot(ob[0] - self.ob_prev[0],
                                                   ob[1] - self.ob_prev[1]), 4)
            self.ob_prev = ob
        if gt:
            row["gx"], row["gy"], row["gyaw"] = (round(gt[1][0], 4), round(gt[1][1], 4),
                                                 round(gt[1][2], 5))
            row["gvx"], row["gvy"], row["gwz"] = (round(gt[2][0], 3), round(gt[2][1], 3),
                                                  round(gt[2][2], 3))
        if gt_map:
            row["mx"], row["my"] = round(gt_map[0], 4), round(gt_map[1], 4)
            if fuse:
                row["loc_err"] = round(math.hypot(fuse[0] - gt_map[0], fuse[1] - gt_map[1]), 4)
        if fuse and self.gg is not None:
            p = self.gg.probe(fuse[0], fuse[1])
            if p:
                row["g_lethal_f"], row["g_insc_f"], row["g_infl_f"], row["g_cost_f"] = (
                    round(p[0], 4), round(p[1], 4), round(p[2], 4), p[3])
        if gt_map and self.gg is not None:
            p = self.gg.probe(gt_map[0], gt_map[1])
            if p:
                row["g_lethal_t"], row["g_insc_t"], row["g_infl_t"], row["g_cost_t"] = (
                    round(p[0], 4), round(p[1], 4), round(p[2], 4), p[3])
        if self.scan is not None:
            row["scan_min"] = round(self.scan[1], 3)
            row["scan_min_deg"] = round(self.scan[2], 1)
            if gt_map and self.gg is not None and isinstance(row.get("g_lethal_t"), float):
                # 地图乐观度 = 地图说有多远 − 雷达说有多远（>0 ⇒ 地图偏乐观/错位）
                row["map_opt"] = round(row["g_lethal_t"] - self.scan[1], 3)
        if ob and self.lg is not None:
            p = self.lg.probe(ob[0], ob[1])
            if p:
                row["l_lethal"], row["l_insc"], row["l_infl"], row["l_cost"] = (
                    round(p[0], 4), round(p[1], 4), round(p[2], 4), p[3])
        cv = self.cmd["chassis"]
        row["cvx"], row["cvy"], row["cwz"] = round(cv[0], 3), round(cv[1], 3), round(cv[2], 3)
        nv = self.cmd["nav"]
        row["nvx"], row["nvy"], row["nwz"] = round(nv[0], 3), round(nv[1], 3), round(nv[2], 3)
        if self.imu:
            row["imu_a"] = round(self.imu[1], 3)
        row["fit"] = self.health["fitness"]
        row["conv"] = self.health["converged"]
        # map→odom 发布停顿（判失效的直接可见信号）
        if self.mo_stamp is not None:
            age = now_w - self.mo_stamp
            row["mo_age"] = round(age, 3)
            self.mo_age_max = max(self.mo_age_max, age)
        row["recov"] = self.fb["recov"]
        row["dist_rem"] = _r(self.fb["dist"])
        if self.plan_t is not None:
            row["plan_age"] = round(now_w - self.plan_t, 2)
            row["plan_min_lethal"] = _r(self.plan_metrics.get("min_lethal"))
            row["plan_min_insc"] = _r(self.plan_metrics.get("min_insc"))

        # ---- 接触/被推（在 on_gt 里算，这里只抄最近一次的残差） ----
        if self.gt_step is not None and (self.clock - self.gt_step["st"]) < 0.5:
            row["step"] = round(self.gt_step["step"], 4)
            row["exp"] = round(self.gt_step["exp"], 4)
            row["res"] = round(self.gt_step["res"], 4)
        # ---- map→odom 停顿事件 ----
        if row.get("mo_age", 0.0) > 0.5:
            if not self.mo_stale_runs or (row["t"] - self.mo_stale_runs[-1]["t1"]) > 0.5:
                self.mo_stale_runs.append({"t0": row["t"], "t1": row["t"],
                                           "wall0": row["wall"], "age": row["mo_age"]})
            else:
                self.mo_stale_runs[-1]["t1"] = row["t"]
                self.mo_stale_runs[-1]["age"] = max(self.mo_stale_runs[-1]["age"], row["mo_age"])

        self.rows.append(row)
        self.fh.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.n_rows += 1
        if self.n_rows % 10 == 0:
            self.fh.flush()

    def _push(self, lst, g, res, kind):
        row = {"t": g["st"], "x": round(g["x"], 3), "y": round(g["y"], 3)}
        if lst and (row["t"] - lst[-1]["t1"]) <= 0.25:
            e = lst[-1]
            e["t1"] = row["t"]
            e["n"] += 1
            e["res_max"] = max(e["res_max"], round(res, 4))
            e["x"], e["y"] = row["x"], row["y"]
        else:
            lst.append({"kind": kind, "t0": row["t"], "t1": row["t"], "n": 1,
                        "res_max": round(res, 4), "x": row["x"], "y": row["y"],
                        "step": round(g["step"], 4), "exp": round(g["exp"], 4)})

    # ------------------------------------------------------------------ 结局
    def goal_feedback(self, msg):
        fb = msg.feedback
        self.fb["recov"] = fb.number_of_recoveries
        self.fb["dist"] = fb.distance_remaining
        self.fb["nav_time"] = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9

    def send_zero(self):
        try:
            for _ in range(5):
                self.cmd_pub.publish(Twist())
                rclpy.spin_once(self, timeout_sec=0.02)
        except Exception:      # noqa: BLE001
            pass


def _r(v):
    return None if v is None else round(v, 4)


def pct(vals, q):
    v = np.asarray(vals, dtype=float)
    if v.size == 0:
        return None
    return float(np.percentile(v, q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--summary", default="")
    ap.add_argument("--goal", nargs=2, type=float, default=None)
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--ready-timeout", type=float, default=300.0)
    ap.add_argument("--max-drive-sec", type=float, default=90.0)
    ap.add_argument("--post-goal-sec", type=float, default=20.0)
    ap.add_argument("--robot-radius", type=float, default=0.22)
    a, _ = ap.parse_known_args()

    rclpy.init()
    n = Probe(a)
    print("[probe] 等待 TF 链 + 代价地图 …", flush=True)
    t0 = time.time()
    while time.time() - t0 < a.ready_timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        ok = (n.pose_of("map->odom") is not None and n.odom_base_fake() is not None
              and n.gg is not None and n.gt is not None)
        if ok and (n.clock or 0) > 0 and time.time() - t0 > 4.0:
            break
    print("[probe] 就绪 %.1fs：map->odom=%s costmap=%s gt=%s"
          % (time.time() - t0, n.pose_of("map->odom") is not None, n.gg is not None,
             n.gt is not None), flush=True)

    # settle（静置，同时冻结对齐）
    t1 = time.time()
    while time.time() - t1 < a.settle:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.events.append({"wall": time.time() - n.t_wall0, "sim": n.clock, "kind": "settle_done",
                     "align": n.align})

    if a.goal is not None:
        print("[probe] 等 action server …", flush=True)
        if not n.ac.wait_for_server(timeout_sec=min(60.0, a.ready_timeout)):
            print("[probe] ❌ navigate_to_pose action server 没出现", flush=True)
        else:
            from geometry_msgs.msg import PoseStamped
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"
            g.pose.header.stamp = n.get_clock().now().to_msg()
            g.pose.pose.position.x = a.goal[0]
            g.pose.pose.position.y = a.goal[1]
            g.pose.pose.orientation.w = 1.0
            gh = n.ac.send_goal_async(g, feedback_callback=n.goal_feedback)
            t2 = time.time()
            while not gh.done() and time.time() - t2 < 30:
                rclpy.spin_once(n, timeout_sec=0.1)
            res = gh.result()
            n.events.append({"wall": time.time() - n.t_wall0, "sim": n.clock, "kind": "goal_sent",
                             "goal": list(a.goal)})
            if res is None or not res.accepted:
                n.events.append({"wall": time.time() - n.t_wall0, "sim": n.clock,
                                 "kind": "goal_rejected"})
                print("[probe] ❌ 目标被拒", flush=True)
            else:
                print("[probe] 目标已接受，开始行驶（上限 %.0f s）" % a.max_drive_sec, flush=True)
                handle = res
                res_fut = handle.get_result_async()   # 终态时完成
                t3 = time.time()
                while time.time() - t3 < a.max_drive_sec and not res_fut.done():
                    rclpy.spin_once(n, timeout_sec=0.05)
                if res_fut.done():
                    st = int(res_fut.result().status)
                    n.goal_state = st
                    n.events.append({"wall": time.time() - n.t_wall0, "sim": n.clock,
                                     "kind": "goal_result", "status": st})
                    print("[probe] 终态 status=%d（4=SUCCEEDED,5=CANCELED,6=ABORTED）" % st,
                          flush=True)
                else:
                    n.events.append({"wall": time.time() - n.t_wall0, "sim": n.clock,
                                     "kind": "drive_timeout"})
                    print("[probe] ⏱ 行驶超时（未达终态）", flush=True)

    # post：继续记录但不再驱动（可观察"撞完/到点之后"的定位行为）
    t5 = time.time()
    while time.time() - t5 < a.post_goal_sec:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.send_zero()
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    n.fh.flush()
    n.fh.close()

    # ------------------------------------------------------------ 汇总
    R = a.robot_radius
    rows = n.rows

    def col(k):
        return [r[k] for r in rows if isinstance(r.get(k), (int, float))]

    summary = {"tag": a.tag, "goal": a.goal, "robot_radius": R,
               "t_wall0": n.t_wall0,
               "n_rows": len(rows), "clock_span": (rows[-1]["t"] - rows[0]["t"]) if rows else 0,
               "align_map_odom": n.align, "counts": n.cnt,
               "goal_status": n.goal_state, "fb": n.fb,
               "travel_gt": round(n.travel_gt, 3), "travel_cmd": round(n.travel_cmd, 3),
               "events": n.events[-40:],
               "blocked": n.blocked, "pushed": n.pushed,
               "mo_stale_runs": n.mo_stale_runs, "mo_age_max": round(n.mo_age_max, 3),
               "plan_last": {k: _r(v) if isinstance(v, float) else v
                             for k, v in n.plan_metrics.items()},
               "plans": [e for e in n.events if e["kind"] == "plan"][:200],
               }
    # 移动段（>0.05 m/s）才计入"执行路径余量"
    mv = [r for r in rows if isinstance(r.get("step"), (int, float)) and r["step"] > 0.005]
    if not mv:
        mv = [r for r in rows if math.hypot(r.get("gvx", 0), r.get("gvy", 0)) > 0.05]
    summary["moving_rows"] = len(mv)
    for tag, arr, keys in (("all", rows, ("g_lethal_f", "g_insc_f", "g_lethal_t", "g_insc_t",
                                          "l_lethal", "l_insc", "scan_min", "map_opt",
                                          "loc_err", "plan_min_lethal", "plan_min_insc")),
                           ("moving", mv, ("g_lethal_f", "g_insc_f", "g_lethal_t", "g_insc_t",
                                           "l_lethal", "l_insc", "scan_min", "map_opt",
                                           "loc_err", "plan_min_lethal", "plan_min_insc"))):
        d = {}
        for k in keys:
            v = [r[k] for r in arr if isinstance(r.get(k), (int, float))]
            if not v:
                continue
            d[k] = {"p50": round(pct(v, 50), 4), "p05": round(pct(v, 5), 4),
                    "p01": round(pct(v, 1), 4), "min": round(min(v), 4),
                    "max": round(max(v), 4), "n": len(v)}
            if k in ("g_lethal_f", "g_lethal_t", "l_lethal"):
                m = [x - R for x in v]
                d[k + "_margin"] = {"p50": round(pct(m, 50), 4), "p05": round(pct(m, 5), 4),
                                    "p01": round(pct(m, 1), 4), "min": round(min(m), 4),
                                    "frac_le0": round(float(np.mean(np.asarray(m) <= 0.0)), 4)}
        summary["dist_" + tag] = d
    # 内切带占用时间比例（足迹压进内切带）
    for tag, arr in (("all", rows), ("moving", mv)):
        for k in ("g_lethal_f", "g_lethal_t", "l_lethal"):
            v = [r[k] for r in arr if isinstance(r.get(k), (int, float))]
            if v:
                summary.setdefault("occ_" + tag, {})[k + "_lt_R"] = round(
                    float(np.mean(np.asarray(v) < R)), 4)
                summary["occ_" + tag][k + "_lt_R-0.05"] = round(
                    float(np.mean(np.asarray(v) < R - 0.05)), 4)
        for k in ("scan_min",):
            v = [r[k] for r in arr if isinstance(r.get(k), (int, float))]
            if v:
                summary.setdefault("occ_" + tag, {})[k + "_lt_0.30"] = round(
                    float(np.mean(np.asarray(v) < 0.30)), 4)
                summary["occ_" + tag][k + "_lt_0.22"] = round(
                    float(np.mean(np.asarray(v) < 0.22)), 4)
                summary["occ_" + tag][k + "_lt_0.16"] = round(
                    float(np.mean(np.asarray(v) < 0.16)), 4)
    summary["contact_events"] = len(n.blocked) + len(n.pushed)
    summary["imu_amax"] = round(max(col("imu_a")), 3) if col("imu_a") else None
    # 定位误差（融合位姿 vs 真值）
    le = col("loc_err")
    if le:
        summary["loc_err"] = {"p50": round(pct(le, 50), 4), "p95": round(pct(le, 95), 4),
                              "max": round(max(le), 4)}
    for k in ("lio_step", "mo_step", "res"):
        v = np.asarray(col(k), dtype=float)
        if v.size:
            summary["step_" + k] = {"p50": round(pct(v, 50), 5), "p95": round(pct(v, 95), 5),
                                    "p99": round(pct(v, 99), 5), "max": round(float(v.max()), 5)}
    vx = np.asarray(col("mox"), dtype=float)
    vy = np.asarray(col("moy"), dtype=float)
    if vx.size:
        summary["map_odom_xy_range"] = {"x_min": round(float(vx.min()), 4),
                                        "x_max": round(float(vx.max()), 4),
                                        "y_min": round(float(vy.min()), 4),
                                        "y_max": round(float(vy.max()), 4)}
    for k in ("fx", "fy", "fyaw", "mx", "my"):
        v = np.asarray(col(k), dtype=float)
        if v.size:
            summary["span_" + k] = {"min": round(float(v.min()), 3),
                                    "max": round(float(v.max()), 3)}
    summary["fitness"] = {"p50": pct(col("fit"), 50), "p95": pct(col("fit"), 95),
                          "max": max(col("fit")) if col("fit") else None}
    if a.summary:
        try:
            with open(a.summary, "w") as f:
                json.dump(summary, f, ensure_ascii=False, indent=1)
        except Exception as ex:      # noqa: BLE001
            print("[probe] ⚠️ summary 写盘失败：%s" % ex, flush=True)
            with open(a.summary, "w") as f:
                json.dump({"tag": a.tag, "error": str(ex), "n_rows": len(rows)}, f)
    print(json.dumps({k: summary[k] for k in
                      ("tag", "n_rows", "goal_status", "travel_gt", "travel_cmd",
                       "contact_events", "mo_age_max", "moving_rows", "plans")
                      if k in summary}, ensure_ascii=False)[:800], flush=True)
    for k in ("dist_moving", "occ_moving", "dist_all", "occ_all"):
        if k in summary:
            print("[probe] %s = %s" % (k, json.dumps(summary[k], ensure_ascii=False)), flush=True)
    print("[probe] blocked=%d pushed=%d mo_stale=%d" % (len(n.blocked), len(n.pushed),
                                                        len(n.mo_stale_runs)), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
