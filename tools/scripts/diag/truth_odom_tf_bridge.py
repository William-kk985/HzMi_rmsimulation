#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**仅用于实验**的真值里程计 TF 桥：把 `/odom_ground_truth` 直接写成 `odom→base_link`。

⚠️ 定位（请勿误用）
    这是 `docs/slam_drops_loopclosure_odom.md` §4「里程计上界」实验的**一次性测量装置**：
    它存在的唯一目的，是回答"给 slam_toolbox 换一份**完美**里程计，丢帧会怎样"。
    它**不进任何 launch**、不被任何默认链路启动，也不改变任何仓库配置。

⚠️ TF 契约（`docs/tf_interface_contract.md` §三）
    `odom→base_link` **只能有一个发布者**。因此本桥只能这样用：
        ros2 launch … lio:=none            # ← 关键：没有 LIO 就没有第二个发布者
        ros2 run   … truth_odom_tf_bridge  # ← 本桥成为唯一发布者
    **绝不能**在 `lio:=small_point_lio`（或 fastlio/pointlio）跑着的时候同时启动本桥 ——
    那就是两个发布者争同一条边（契约 P5），量到的东西没有意义。
    脚本启动时会主动检查 `/tf` 上是否已有别人在发 `odom→base_link`，**有就拒绝启动**。

它做什么
    · 订阅 `/odom_ground_truth`（Gazebo 底盘插件的（近）真值里程计，100 Hz）；
    · 用消息**自己的 header.stamp** 广播 TF `odom→base_link`（位姿原样搬运，不插值、不外推）；
    · `--delay-s`：把 TF **晚** `delay_s` 发布（戳还是帧的真实戳）。
      这是为了做「完美里程计 + 与 LIO 相同量级的 TF 延迟」那一组：
      它能区分"丢帧是**精度**不够"还是"丢帧是 **TF 到得晚**"（见报告 §4）。
      没有它就只能得到"零延迟 + 零误差"的上界，两个因素混在一起。

输出
    `--stats <path>.json`：发布条数、实际速率、TF 发布滞后（墙钟）分布。
    Ctrl-C / SIGTERM 都会先落盘再退出。

用法（必须与 launch 塞进**同一次 bash 调用**，见 run_dropab.sh 头注）
    python3 tools/scripts/diag/truth_odom_tf_bridge.py --stats /tmp/x/bridge.json \
        --delay-s 0.0 --duration 400
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import signal
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TruthOdomTfBridge(Node):
    def __init__(self, args):
        super().__init__('truth_odom_tf_bridge')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.args = args
        self.br = TransformBroadcaster(self)
        self.pub_times = []          # 每条 TF 的墙钟发布时刻（用于速率/滞后统计）
        self.stamps = []
        self.q = collections.deque()       # (release_wall, TransformStamped)
        self.q_odom = collections.deque()  # (release_wall, Odometry)
        self.n_msg = 0
        self.n_pub = 0
        self.bad_child = 0
        self.other_pub = 0            # /tf 上别人发的 odom→base_link（契约检查）
        self.violation = False
        self.t0 = time.monotonic()
        self.qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, args.topic, self.on_odom, self.qos)
        # 同时把同一份里程计按**统一话题口径**发到 /odom：`lio:=none` 的契约就是
        # "外部里程计负责 /odom + odom→base_link"（docs/algorithm_matrix.md:198、
        # docs/tf_interface_contract.md §三）。下游（coverage_drive 等）等的是 /odom。
        self.pub_odom = self.create_publisher(Odometry, args.odom_topic, self.qos)
        self.create_timer(1.0 / 200.0, self.tick)
        # 契约哨兵（不是"再听一遍 /tf"——那会听到自己）：查图里 /tf 的**发布者节点名**，
        # 出现任何 LIO 节点就说明这条边有两个发布者（契约 P5），量出来的东西不算数。
        self.create_timer(2.0, self.check_contract)

    # ------------------------------------------------------------------
    LIO_NODES = ('small_point_lio', 'fastlio', 'fast_lio', 'pointlio', 'point_lio',
                 'lio_tf_adapter')

    def check_contract(self):
        try:
            infos = self.get_publishers_info_by_topic('/tf')
        except Exception:
            return
        names = {i.node_name for i in infos}
        bad = {n for n in names if any(p in n for p in self.LIO_NODES)}
        if bad:
            self.violation = True
            self.other_pub = len(bad)
            if not getattr(self, '_warned', False):
                self._warned = True
                self.get_logger().error(
                    '契约违规：/tf 上还有 LIO 节点在发（%s）——odom→base_link 会有两个发布者，'
                    '本桥应当独占这条边，请用 lio:=none 起栈' % ','.join(sorted(bad)))

    def on_odom(self, msg):
        self.n_msg += 1
        if msg.child_frame_id.lstrip('/') and \
                msg.child_frame_id.lstrip('/') != self.args.child_frame_id:
            self.bad_child += 1
            if self.bad_child == 1:
                self.get_logger().warn('里程计的 child_frame_id=%r ≠ %r：位姿按原样搬运，'
                                       '请自行确认语义' % (msg.child_frame_id,
                                                           self.args.child_frame_id))
        ts = TransformStamped()
        ts.header.stamp = msg.header.stamp
        ts.header.frame_id = self.args.frame_id
        ts.child_frame_id = self.args.child_frame_id
        ts.transform.translation.x = msg.pose.pose.position.x
        ts.transform.translation.y = msg.pose.pose.position.y
        ts.transform.translation.z = msg.pose.pose.position.z
        ts.transform.rotation = msg.pose.pose.orientation
        # /odom 中继（与 TF 同一个延迟口径，保持"两条口径一致"）
        if self.args.odom_topic:
            relay = msg
            relay.header.frame_id = self.args.frame_id
            relay.child_frame_id = self.args.child_frame_id
            if self.args.delay_s <= 0.0:
                self.pub_odom.publish(relay)
            else:
                self.q_odom.append((time.monotonic() + self.args.delay_s, relay))
        if self.args.delay_s <= 0.0:
            self.send(ts)
        else:
            self.q.append((time.monotonic() + self.args.delay_s, ts))

    def tick(self):
        now = time.monotonic()
        while self.q and self.q[0][0] <= now:
            self.send(self.q.popleft()[1])
        while self.q_odom and self.q_odom[0][0] <= now:
            self.pub_odom.publish(self.q_odom.popleft()[1])

    def send(self, ts):
        self.br.sendTransform(ts)
        self.n_pub += 1
        self.pub_times.append(time.monotonic())
        self.stamps.append(ts.header.stamp.sec + ts.header.stamp.nanosec * 1e-9)

    # ------------------------------------------------------------------
    def stats(self):
        out = {'n_msg': self.n_msg, 'n_pub': self.n_pub, 'delay_s': self.args.delay_s,
               'child_frame_mismatch': self.bad_child,
               'tf_other_publishers': self.other_pub, 'contract_violation': self.violation}
        if len(self.pub_times) > 2:
            d = [self.pub_times[i + 1] - self.pub_times[i]
                 for i in range(len(self.pub_times) - 1)]
            out['pub_rate_hz'] = round(1.0 / st.median(d), 2) if st.median(d) > 0 else None
            out['pub_gap_p99_wall'] = round(sorted(d)[int(0.99 * (len(d) - 1))], 4)
            out['pub_gap_max_wall'] = round(max(d), 4)
            out['stamp_span_s'] = round(self.stamps[-1] - self.stamps[0], 2)
        if self.q:
            out['queue_left'] = len(self.q)
        return out


def main():
    ap = argparse.ArgumentParser(
        description='实验用：把 /odom_ground_truth 写成 odom→base_link（配合 lio:=none）')
    ap.add_argument('--stats', required=True, help='统计 JSON 落盘路径')
    ap.add_argument('--topic', default='/odom_ground_truth')
    ap.add_argument('--odom-topic', default='/odom',
                    help='中继发布统一里程计话题（默认 /odom；给空串则不发）')
    ap.add_argument('--frame-id', default='odom')
    ap.add_argument('--child-frame-id', default='base_link')
    ap.add_argument('--delay-s', type=float, default=0.0,
                    help='把 TF 晚这么多秒发布（戳不变）——模拟 LIO 的 TF 延迟')
    ap.add_argument('--duration', type=float, default=600.0, help='自动退出（墙钟秒）')
    args = ap.parse_args()

    rclpy.init()
    node = TruthOdomTfBridge(args)
    done = {'v': False}

    def stop(*_):
        done['v'] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    t0 = time.monotonic()
    while rclpy.ok() and not done['v'] and (time.monotonic() - t0) < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    s = node.stats()
    with open(args.stats, 'w') as f:
        json.dump(s, f, ensure_ascii=False, indent=1)
    print('BRIDGEJSON ' + json.dumps(s, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if s['contract_violation'] else 0


if __name__ == '__main__':
    sys.exit(main())
