#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map→odom 跳变哨兵（alert-only 的"急停哨"）。

## 为什么需要它

建图/定位跑飞时，`slam_toolbox`（或任何 map→odom 发布者）会在**一次更新**里把
`map→odom` 挪动一大截（扫描匹配收敛到错误解 / 回环闭合 / 退化方向上的解跳）。
现象是"**图是慢慢变歪的，但你发现的时候已经歪了**"——因为没有任何东西在"单步跳变"
这个尺度上报警。本节点就是那个报警器。

## 它做什么 / 不做什么（**重要**）

做：
  · 订阅 `/tf`，跟踪 `map→odom`（父/子帧可配）；
  · 每次收到新值，算**相对上一次更新**的增量 `Δxy / Δz / Δyaw / Δpitch / Δroll`；
  · 任一增量超过阈值 ⇒ `WARNING` 级日志（带全部数字与累计跳变次数）；
  · 跳变状态以 **latched**（TRANSIENT_LOCAL + RELIABLE）的 `std_msgs/Bool`
    发到 `~/jumped`：一旦跳过一次就保持 `true`，直到 `/reset` 服务被调用；
  · 全程统计每步增量的 p50/p90/p99/max，退出时（或收到 `/report` 服务）打印
    —— **这个分布就是"正常运动"的实测依据**，用来定阈值（见下）。

不做（**故意**）：
  · **不重新发布 `map→odom`**。原因有两条，任何一条都足以否掉"拒绝传播跳变"这条路：
    ① 本仓的契约是 **`map→odom` 只有一个发布者**（`docs/tf_interface_contract.md`；
       建图模式下 = slam_toolbox，导航+空 localization 模式 = 静态桥）。
       本节点一旦往 `/tf` 上写 `map→odom`，就成了**第二个发布者**：`tf2` 对同一
       (parent, child) 的两个发布者不做仲裁，消费者拿到的是**两路交错的值**
       （50 Hz 的 slam_toolbox 更新 + 本节点的"最后一次好值"）⇒ 位姿会来回抖，
       比跳变本身更糟。
    ② 更要命的是"拒绝传播"在 TF 树里**没有语义**：`tf2` 的 `Buffer` 是按时间缓存
       每一帧的，你把旧值**重新盖上去**也删不掉 slam_toolbox 已经广播出去的那一帧
       （`setTransform` 只覆盖同时间戳，而新时间戳的错值仍在写）⇒ 想"拦"就得变成
       唯一发布者（即自己实现一个包装层），那是另一个量级的改动（见本文末"若要真拦截"）。
  · 因此：**alert-only**。它保证"跳变发生的那一瞬间"你就知道（并把状态挂在一个
    latched 话题上给别的节点/脚本消费），而不是等图糊了才回头查日志。

## 阈值怎么定（**不许拍脑袋**）

默认值 `--threshold-trans 0.20 --threshold-rot-deg 8.0` 来自本仓 `world:=RMUC2026` +
`lio:=small_point_lio` 建图跑的**实测分布**（`tools/scripts/mapping/ramp_section_probe.py`
/ 本节点的 `--report-file` 输出，见 `docs/mapping_2d_from_cloud.md` §2）：
正常（无卡死、无退化）一段 150 s 的跑图里，单步增量 p99 ≈ 0.05 m / 1.5°，
max ≈ 0.13 m / 4°；而实测"歪掉"那一次的单步跳变是 0.4~1.2 m / 10~25°。
⇒ 0.20 m / 8° 落在两者中间（≈ p99 的 4 倍、异常值的 1/2），
既能抓住真跳变，又不会把正常扫描匹配的修正误报成跳变。
换平台请**先跑一次 `--report-file` 再定阈值**，不要沿用这里的数字。

## 用法

    # 单跑（另开一个终端，与建图/导航栈并行）
    python3 tools/scripts/diag/map_odom_jump_gate.py --threshold-trans 0.20

    # 只看统计、不报警（定阈值用）
    python3 tools/scripts/diag/map_odom_jump_gate.py --report-file /tmp/gate.json --threshold-trans 99

    # 消费 latched 状态：
    ros2 topic echo /map_odom_jump_gate/jumped --qos-durability transient_local

退出码：默认 0；`--exit-on-jump` 时一旦跳变就 exit 7（给脚本当"急停"用）。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_of(q):
    """→ (roll, pitch, yaw)，与 tf2 的 RPY 约定一致。"""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch, yaw_of(q)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


class JumpGate(Node):
    def __init__(self, a):
        super().__init__('map_odom_jump_gate')
        self.a = a
        self.lock = threading.Lock()
        self.grp = ReentrantCallbackGroup()
        self.last = None            # (x, y, z, roll, pitch, yaw)
        self.last_stamp = None
        self.n = 0
        self.n_jump = 0
        self.jumps = []             # 每次跳变的明细
        self.hist = {'trans': [], 'yaw': [], 'rot': [], 'dt': []}
        self.jumped = False

        self.sub = self.create_subscription(TFMessage, a.tf_topic, self.on_tf,
                                            QoSProfile(depth=100), callback_group=self.grp)
        self.pub = self.create_publisher(Bool, '~/jumped', LATCHED)
        self.create_service(Trigger, '~/reset', self.on_reset, callback_group=self.grp)
        self.create_service(Trigger, '~/report', self.on_report, callback_group=self.grp)
        self.timer = self.create_timer(1.0, self.publish_state, callback_group=self.grp)
        self.get_logger().info(
            'jump gate 就绪： watching %s -> %s on %s；阈值 trans=%.3f m  rot=%.2f deg；'
            'alert-only（**不**发布 map→odom，见文件头说明）'
            % (a.parent_frame, a.child_frame, a.tf_topic, a.threshold_trans,
               a.threshold_rot_deg))

    # ------------------------------------------------------------------ callbacks
    def on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            if (tr.header.frame_id != self.a.parent_frame
                    or tr.child_frame_id != self.a.child_frame):
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            roll, pitch, yaw = rpy_of(q)
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            with self.lock:
                cur = (t.x, t.y, t.z, roll, pitch, yaw)
                if self.last is not None:
                    dx = cur[0] - self.last[0]
                    dy = cur[1] - self.last[1]
                    dz = cur[2] - self.last[2]
                    dtrans = math.hypot(dx, dy)
                    dyaw = abs(wrap(cur[5] - self.last[5]))
                    droll = abs(wrap(cur[3] - self.last[3]))
                    dpitch = abs(wrap(cur[4] - self.last[4]))
                    drot = max(dyaw, droll, dpitch)
                    dt = (stamp - self.last_stamp) if self.last_stamp else 0.0
                    self.n += 1
                    self.hist['trans'].append(dtrans)
                    self.hist['yaw'].append(dyaw)
                    self.hist['rot'].append(drot)
                    self.hist['dt'].append(dt)
                    bad_t = dtrans > self.a.threshold_trans
                    bad_r = math.degrees(drot) > self.a.threshold_rot_deg
                    if (bad_t or bad_r) and self.n > self.a.warmup_samples:
                        self.n_jump += 1
                        self.jumped = True
                        rec = {'n': self.n, 'sim_t': round(stamp, 3), 'dt': round(dt, 4),
                               'dxy': round(dtrans, 4), 'dz': round(dz, 4),
                               'dyaw_deg': round(math.degrees(dyaw), 2),
                               'drot_deg': round(math.degrees(drot), 2),
                               'from': [round(v, 4) for v in self.last],
                               'to': [round(v, 4) for v in cur],
                               'reason': ('trans' if bad_t else '') + ('+rot' if bad_r else '')}
                        if len(self.jumps) < 500:
                            self.jumps.append(rec)
                        self.get_logger().warning(
                            '★ map→odom 单步跳变 #%d（第 %d 次更新）：Δxy=%.3f m (阈 %.3f) '
                            'Δz=%.3f m Δyaw=%.2f° Δmax-rot=%.2f° (阈 %.2f°) Δt=%.3f s | '
                            '(%+.3f,%+.3f,%.3f) → (%+.3f,%+.3f,%.3f)'
                            % (self.n_jump, self.n, dtrans, self.a.threshold_trans, dz,
                               math.degrees(dyaw), math.degrees(drot),
                               self.a.threshold_rot_deg, dt,
                               self.last[0], self.last[1], self.last[2],
                               cur[0], cur[1], cur[2]))
                        if self.a.exit_on_jump:
                            self.flush_report()
                            self.get_logger().error('--exit-on-jump ⇒ 退出码 7')
                            rclpy.shutdown()
                            return
                self.last = cur
                self.last_stamp = stamp

    def on_reset(self, _req, res):
        with self.lock:
            self.jumped = False
            self.n_jump = 0
            self.jumps = []
        self.publish_state()
        res.success = True
        res.message = 'cleared'
        return res

    def on_report(self, _req, res):
        txt = self.flush_report()
        res.success = True
        res.message = txt[:1000]
        return res

    # ------------------------------------------------------------------ output
    def publish_state(self):
        m = Bool()
        with self.lock:
            m.data = bool(self.jumped)
        self.pub.publish(m)

    def stats(self):
        with self.lock:
            h = {k: list(v) for k, v in self.hist.items()}
            return {
                'updates': self.n,
                'jumps': self.n_jump,
                'trans_m': {'p50': pct(h['trans'], 50), 'p90': pct(h['trans'], 90),
                            'p99': pct(h['trans'], 99),
                            'max': max(h['trans']) if h['trans'] else float('nan')},
                'rot_deg': {'p50': math.degrees(pct(h['rot'], 50)),
                            'p90': math.degrees(pct(h['rot'], 90)),
                            'p99': math.degrees(pct(h['rot'], 99)),
                            'max': math.degrees(max(h['rot'])) if h['rot'] else float('nan')},
                'yaw_deg': {'p99': math.degrees(pct(h['yaw'], 99)),
                            'max': math.degrees(max(h['yaw'])) if h['yaw'] else float('nan')},
                'dt_s': {'p50': pct(h['dt'], 50), 'p99': pct(h['dt'], 99)},
                'threshold_trans_m': self.a.threshold_trans,
                'threshold_rot_deg': self.a.threshold_rot_deg,
                'jump_details': self.jumps,
            }

    def flush_report(self):
        st = self.stats()
        txt = ('JUMPGATE ' + json.dumps(
            {k: v for k, v in st.items() if k != 'jump_details'}, ensure_ascii=False))
        print(txt, flush=True)
        if self.a.report_file:
            with open(self.a.report_file, 'w') as f:
                json.dump(st, f, ensure_ascii=False, indent=1)
        return txt


def main():
    ap = argparse.ArgumentParser(description='map→odom 单步跳变哨兵（alert-only，见文件头）')
    ap.add_argument('--tf-topic', default='/tf')
    ap.add_argument('--parent-frame', default='map')
    ap.add_argument('--child-frame', default='odom')
    ap.add_argument('--threshold-trans', type=float, default=0.20,
                    help='单步平移跳变阈值（m）。默认 0.20 来自 RMUC2026+small_point_lio 实测（文件头）')
    ap.add_argument('--threshold-rot-deg', type=float, default=8.0,
                    help='单步旋转跳变阈值（deg，取 roll/pitch/yaw 三者最大）')
    ap.add_argument('--warmup-samples', type=int, default=20,
                    help='前 N 次更新不判（启动瞬间 map→odom 从 0 跳到初值）')
    ap.add_argument('--report-file', help='把增量分布 + 跳变明细写成 JSON')
    ap.add_argument('--exit-on-jump', action='store_true', help='跳变即 exit 7')
    ap.add_argument('--duration', type=float, default=0.0, help='秒；>0 时到点自动收尾退出')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = JumpGate(args)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    # 到点硬收尾（同 ramp_section_probe 的坑：`while ok(): spin_once()` 会被积压回调拖住）
    if args.duration > 0:
        def _stop():
            node.flush_report()
            import os as _os, sys as _sys
            _sys.stdout.flush(); _sys.stderr.flush()
            _os._exit(0)
        node.create_timer(float(args.duration), _stop, callback_group=node.grp)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    node.flush_report()
    try:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
