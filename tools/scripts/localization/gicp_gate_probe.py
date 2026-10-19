#!/usr/bin/env python3
"""gicp_gate_probe.py —— `localization:=gicp` 的**运动一致性门限**逐帧取证探针（只订阅，不发布）。

回答的问题（见 `docs/frame_contract_v3_measurements.md` 里「GICP 首帧门死锁」那一节）：
  · 每帧的**创新**（本帧 GICP 结果相对里程计预测位姿的偏离）、**dt**、**门限**、**是否被采纳**、
    **连续拒绝帧数** —— 这些量在节点日志里是**限频**的（未采纳 WARN 1 s 一条、采纳帧的 DEBUG
    默认不打印）⇒ "逐帧"证据只能靠 `/gicp_registration/gate` 这条话题（同包 publishGateMetrics）。
  · `map→odom` 到底还在不在发、以多高速率发、**唯一戳**多少（= 位姿真正更新的速率；
    节点 50 Hz 重复广播同一条 TF，条数 ≠ 更新率）。
  · 采纳帧率 / `~/pose` 发布率 / `~/fitness_score` 量级。

话题（全部只读；字段契约见 `gicp_registration.cpp` 的 `publishGateMetrics`）：
  /gicp_registration/gate            std_msgs/Float64MultiArray（每帧一条）
  /gicp_registration/pose            geometry_msgs/PoseWithCovarianceStamped
  /gicp_registration/fitness_score   std_msgs/Float64（transient_local）
  /gicp_registration/converged       std_msgs/Bool（transient_local）
  /tf                                tf2_msgs/TFMessage（只数 map→odom 条数与唯一戳）
  /clock                             rosgraph_msgs/Clock（窗口用**仿真**秒，与 settle/window 同口径）

口径：首个 `/clock` 起 settle 秒（仿真）之后持续 window 秒 = 分析窗口；窗口外也留"整跑"统计
（首帧门死锁发生在启动后 ~6 s，**在窗口之前** ⇒ 两个口径都要看）。

用法：
  python3 tools/scripts/localization/gicp_gate_probe.py --out .tmp_gate/<tag>/gate.json \
      --settle 20 --window 20 --wall-timeout 400
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64, Float64MultiArray
from tf2_msgs.msg import TFMessage

# /gicp_registration/gate 的字段顺序（= 节点的 publishGateMetrics 契约，改顺序 = 破坏可复现性）
GATE_FIELDS = [
    'pub_t',          # 0 发布时刻（节点时钟 = 仿真秒）
    'innov_xy',       # 1 创新·平移 (m)
    'innov_yaw_deg',  # 2 创新·yaw (°)
    'allow_xy',       # 3 门限·平移 (m) = clamp(rate·dt, min, max)
    'allow_yaw_deg',  # 4 门限·yaw (°)
    'dt',             # 5 门限用的 dt (s)
    'score',          # 6 fitness score (m²)
    'accepted',       # 7 1/0
    'converged',      # 8 1/0
    'src_points',     # 9 下采样后源点数
    'align_ms',       # 10 align 耗时 (ms)
    'streak',         # 11 本帧处理后的连续拒绝帧数
    'flags',          # 12 bit0 跳过创新门限 / bit1 重锚 / bit2 失效态探针
]
FLAG_SKIP_INNOV, FLAG_REANCHOR, FLAG_LOST_PROBE = 1, 2, 4

TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST)
# /clock 的发布端（Gazebo / ros_gz）是 **BEST_EFFORT**：用默认 RELIABLE 订阅会
# "offering incompatible QoS. No messages will be received"（2026-10-21 实测踩到）⇒
# 拿不到仿真时间，窗口就退化成墙钟。这里显式 BEST_EFFORT。
CLOCK_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)


def pct(vals, q):
    """百分位（nearest-rank，不插值）：空列表 ⇒ None。"""
    if not vals:
        return None
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round((q / 100.0) * (len(s) - 1)))))
    return s[k]


class GateProbe(Node):
    def __init__(self, settle, window, wall_timeout):
        super().__init__('gicp_gate_probe')
        self.settle = settle
        self.window = window
        self.wall_timeout = wall_timeout
        self.t_wall0 = time.monotonic()
        self.sim_t = None            # 最近一次 /clock（仿真秒）
        self.t0 = None               # 分析锚点 = 首个 /clock
        self.gate = []               # [(t_rel, [f0..f12])]
        self.pose_t = []             # ~/pose 到达时刻（仿真秒）
        self.fitness = []
        self.converged = []
        self.tf_map_odom = []        # 每条 map→odom 的到达时刻（仿真秒）
        self.tf_map_odom_stamps = []  # 每条 map→odom 的 header.stamp（唯一戳用）
        self.tf_other_n = 0

        self.create_subscription(Float64MultiArray, '/gicp_registration/gate',
                                 self._on_gate, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/gicp_registration/pose',
                                 self._on_pose, 10)
        self.create_subscription(Float64, '/gicp_registration/fitness_score',
                                 self._on_fitness, TL)
        self.create_subscription(Bool, '/gicp_registration/converged',
                                 self._on_converged, TL)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 200)
        self.create_subscription(Clock, '/clock', self._on_clock, CLOCK_QOS)

    # ---------------- 回调 ----------------
    def _on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_t = t
        if self.t0 is None:
            self.t0 = t

    def _now_rel(self):
        """相对 t0 的时刻：优先仿真秒，没有 /clock 就退回墙钟。"""
        if self.sim_t is not None and self.t0 is not None:
            return self.sim_t - self.t0
        return time.monotonic() - self.t_wall0

    def _on_gate(self, msg):
        d = list(msg.data)
        if len(d) < len(GATE_FIELDS):        # 兼容旧档（没有 flags 字段）
            d = d + [0.0] * (len(GATE_FIELDS) - len(d))
        self.gate.append((self._now_rel(), d))

    def _on_pose(self, msg):
        self.pose_t.append(self._now_rel())

    def _on_fitness(self, msg):
        self.fitness.append(float(msg.data))

    def _on_converged(self, msg):
        self.converged.append(bool(msg.data))

    def _on_tf(self, msg):
        t = self._now_rel()
        for tr in msg.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                self.tf_map_odom.append(t)
                self.tf_map_odom_stamps.append(
                    tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
            else:
                self.tf_other_n += 1

    # ---------------- 收尾判据 ----------------
    def done(self):
        if time.monotonic() - self.t_wall0 >= self.wall_timeout:
            return True
        if self.t0 is None:
            # 拿不到 /clock（时钟未起/QoS 不兼容）⇒ 用墙钟兜底：等够窗口 + 一点余量就收，
            # 免得探针一直挂到 wall-timeout（此时 t 轴是墙钟，stats_window 的窗口也是墙钟）。
            return (time.monotonic() - self.t_wall0) >= self.settle + self.window + 5.0
        return self._now_rel() >= self.settle + self.window + 2.0

    # ---------------- 统计 ----------------
    def _slice(self, t_lo, t_hi):
        return [d for (t, d) in self.gate if t_lo <= t < t_hi]

    def _stats(self, rows):
        n = len(rows)
        acc = [r for r in rows if r[7] >= 0.5]
        inn_all = [r[1] for r in rows]
        inn_acc = [r[1] for r in acc]
        allow = [r[3] for r in rows]
        dt = [r[5] for r in rows]
        score = [r[6] for r in rows if r[6] == r[6]]  # 去 nan
        return {
            'n_frames': n,
            'n_accepted': len(acc),
            'adoption_rate': (len(acc) / n) if n else None,
            'innov_xy_p50': pct(inn_all, 50), 'innov_xy_p95': pct(inn_all, 95),
            'innov_xy_max': (max(inn_all) if inn_all else None),
            'innov_xy_accepted_p50': pct(inn_acc, 50),
            'innov_xy_accepted_p95': pct(inn_acc, 95),
            'innov_xy_accepted_max': (max(inn_acc) if inn_acc else None),
            'innov_yaw_deg_max': (max([r[2] for r in rows]) if rows else None),
            'allow_xy_min': (min(allow) if allow else None),
            'allow_xy_p50': pct(allow, 50), 'allow_xy_max': (max(allow) if allow else None),
            'dt_p50': pct(dt, 50), 'dt_min': (min(dt) if dt else None),
            'dt_max': (max(dt) if dt else None),
            'n_dt_zero': sum(1 for v in dt if v <= 0.0),
            'score_p50': pct(score, 50), 'score_p95': pct(score, 95),
            'score_max': (max(score) if score else None),
            'streak_max': (max([r[11] for r in rows]) if rows else None),
            'n_flag_skip_innov': sum(1 for r in rows if int(r[12]) & FLAG_SKIP_INNOV),
            'n_flag_reanchor': sum(1 for r in rows if int(r[12]) & FLAG_REANCHOR),
            'n_flag_lost_probe': sum(1 for r in rows if int(r[12]) & FLAG_LOST_PROBE),
            'src_points_p50': pct([r[9] for r in rows], 50),
            'align_ms_p50': pct([r[10] for r in rows], 50),
        }

    def _per_sec(self, times):
        out = {}
        for t in times:
            k = int(t)
            out[str(k)] = out.get(str(k), 0) + 1
        return out

    def report(self):
        w_lo = (self.t0 or 0.0) + self.settle
        w_hi = w_lo + self.window
        rows_win = self._slice(self.settle, self.settle + self.window)
        rows_all = [d for (_t, d) in self.gate]
        tf_span = (self.tf_map_odom[-1] - self.tf_map_odom[0]) if len(self.tf_map_odom) > 1 else 0.0
        tf_uniq = sorted(set(round(s, 6) for s in self.tf_map_odom_stamps))
        gate_t = [t for (t, _d) in self.gate]
        out = {
            'ok': len(self.gate) > 0,
            'probe': 'gicp_gate_probe',
            'clock': 'sim' if self.sim_t is not None else 'wall',
            't0': self.t0,
            't_end_rel': self._now_rel(),
            'window_rel': [self.settle, self.settle + self.window],
            'window_abs': [w_lo, w_hi],
            'gate_fields': GATE_FIELDS,
            'n_gate': len(self.gate),
            'gate_rel_span': ([gate_t[0], gate_t[-1]] if gate_t else None),
            'n_pose': len(self.pose_t),
            'pose_rel_span': ([self.pose_t[0], self.pose_t[-1]] if self.pose_t else None),
            'n_fitness': len(self.fitness),
            'fitness_p50': pct(self.fitness, 50),
            'fitness_p95': pct(self.fitness, 95),
            'fitness_max': (max(self.fitness) if self.fitness else None),
            'n_converged_msgs': len(self.converged),
            'n_converged_true': sum(1 for c in self.converged if c),
            'tf_map_odom_n': len(self.tf_map_odom),
            'tf_map_odom_rel_span': ([self.tf_map_odom[0], self.tf_map_odom[-1]]
                                     if self.tf_map_odom else None),
            'tf_map_odom_rate_hz': (len(self.tf_map_odom) / tf_span if tf_span > 0 else None),
            'tf_map_odom_unique_stamps': len(tf_uniq),
            'tf_map_odom_unique_rate_hz': (len(tf_uniq) / tf_span if tf_span > 0 else None),
            'tf_map_odom_stamp_span': ([tf_uniq[0], tf_uniq[-1]] if tf_uniq else None),
            'tf_map_odom_per_sec': self._per_sec(self.tf_map_odom),
            'tf_other_n': self.tf_other_n,
            'pose_per_sec': self._per_sec(self.pose_t),
            'gate_per_sec': self._per_sec(gate_t),
            'stats_all': self._stats(rows_all),
            'stats_window': self._stats(rows_win),
            'first10': [dict(zip(GATE_FIELDS, r)) for r in rows_all[:10]],
            'gate': [[round(t, 4)] + [round(v, 6) for v in d] for (t, d) in self.gate],
        }
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--settle', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=20.0)
    ap.add_argument('--wall-timeout', type=float, default=400.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = GateProbe(args.settle, args.window, args.wall_timeout)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        out = node.report()
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)
        print(json.dumps({k: out[k] for k in (
            'ok', 'n_gate', 'n_pose', 'tf_map_odom_n', 'tf_map_odom_unique_stamps',
            'stats_all', 'stats_window')}, indent=1))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
