#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_goal_forensics.py —— 「目标导航为什么不动」归因探针（2026-10-09）。

与 `tilt_mount_probe.py` 的分工：那个探针量**几何/帧**（点云/平面/TF/代价图圆），
本探针只量**一次导航尝试的因果链**，为 docs/tilted_lidar_fidelity.md §L 服务：

  · **有没有规划**：`/plan` 每一条的位姿数/长度/首末位姿/末位姿到目标的距离；
  · **规划器看到的世界**：`/global_costmap/costmap` 的周期性快照（落 grids.npz），
    并沿**每条 plan** 采样代价（unknown/lethal/inscribed/free 计数）；
  · **目标格本身是什么**：目标点（map 系）在全局代价图里的值 + 半径 0.5 m 内的分类计数
    + 到最近 lethal 格的距离；
  · **局部图覆盖**：目标在不在 `/local_costmap/costmap` 的窗口里；
  · **控制器有没有接手**：`/follow_path/_action/status` 与 `/navigate_to_pose/_action/status`
    的**状态机时间线**（ACCEPTED/EXECUTING/SUCCEEDED/ABORTED…）；
  · **恢复次数**：`/navigate_to_pose/_action/feedback` 的 `number_of_recoveries`
    + `/behavior_tree_log` 的**逐节点状态跳变**（BT 里到底进了哪个恢复节点）；
  · **控制器在想什么**：`/follow_path/_action/feedback` 的 `distance_to_goal`/`speed` 时间线；
  · **真正发出去的速度**：`/cmd_vel`、`/cmd_vel_chassis`（vx/wz）时间线；
  · **车到底动没动**：`map→base_link` TF 与 `/odom_ground_truth` 的位姿时间线；
  · **spawn 自转**：记录窗口起点 vs 发目标时的真值 yaw（"自转结束了没有"）。

只发布一样东西：一次 `/goal_pose`（`--goal-forward H` / `--goal X Y`），其余全是订阅。
口径：`--settle` + `--duration` 之后发目标（与 `tilt_mount_probe.py --goal-forward` 同款，
避开 spawn/插件加载窗口，见 §H.6），随后观察 `--goal-wait` 秒。

用法：
  python3 tools/scripts/tiltmount/nav_goal_forensics.py --out <dir>/forensics.json \
      --grids <dir>/grids.npz --settle 30 --duration 30 --goal-forward 2.0 --goal-wait 45
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from collections import Counter

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.msg import BehaviorTreeLog
# ★ action 的 **feedback 话题消息类型**在 rclpy 里只在私有模块里生成（`FollowPath.FeedbackMessage`
#   不存在）——实测：`AttributeError: type object 'FollowPath' has no attribute 'FeedbackMessage'`。
from nav2_msgs.action._follow_path import FollowPath_FeedbackMessage as FOLLOW_PATH_FB
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage as NAV_TO_POSE_FB
import tf2_ros

QOS_RE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST)
# nav2 的代价图发布者 = transient_local + reliable（costmap_2d_publisher.cpp:68）
QOS_MAP = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

STATUS_NAME = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
               4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def cost_class(v):
    """OccupancyGrid 值 → 类名（nav2：-1/255 unknown、0 free、99 inscribed、100 lethal）。"""
    if v < 0 or v == 255:
        return 'unknown'
    if v >= 100:
        return 'lethal'
    if v >= 99:
        return 'inscribed'
    if v == 0:
        return 'free'
    return 'inflated'


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Grid(object):
    """一张 OccupancyGrid + 坐标查询（数据行 0 = 图的最下方，与 nav_msgs 一致）。"""

    def __init__(self, msg):
        self.frame_id = msg.header.frame_id
        self.res = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.oyaw = yaw_from_quat(msg.info.origin.orientation.x, msg.info.origin.orientation.y,
                                  msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.a = np.array(msg.data, dtype=np.int16).reshape(self.h, self.w)

    def metrics(self):
        c = Counter(cost_class(int(v)) for v in self.a.ravel())
        n = float(max(1, self.a.size))
        return {'frame_id': self.frame_id, 'res': self.res, 'w': self.w, 'h': self.h,
                'origin': [round(self.ox, 4), round(self.oy, 4),
                           round(math.degrees(self.oyaw), 3)],
                'counts': {k: int(v) for k, v in c.items()},
                'frac': {k: round(v / n, 4) for k, v in c.items()},
                'stamp': round(self.stamp, 3)}

    def xy_to_cell(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        col = int(math.floor(lx / self.res))
        row = int(math.floor(ly / self.res))
        if 0 <= row < self.h and 0 <= col < self.w:
            return row, col
        return None

    def value(self, x, y):
        rc = self.xy_to_cell(x, y)
        return None if rc is None else int(self.a[rc[0], rc[1]])

    def window(self, x, y, radius):
        counts = Counter()
        c = int(math.ceil(radius / self.res))
        rc = self.xy_to_cell(x, y)
        if rc is None:
            return {'center_cell': None, 'n': 0, 'counts': {}}
        r0, c0 = rc
        sub = self.a[max(0, r0 - c):r0 + c + 1, max(0, c0 - c):c0 + c + 1]
        for v in sub.ravel():
            counts[cost_class(int(v))] += 1
        return {'center_cell': [r0, c0], 'n': int(sub.size),
                'counts': {k: int(v) for k, v in counts.items()}}

    def nearest_lethal(self, x, y, max_r=4.0):
        rc = self.xy_to_cell(x, y)
        if rc is None:
            return None
        r0, c0 = rc
        c = int(math.ceil(max_r / self.res))
        ra, rb = max(0, r0 - c), min(self.h, r0 + c + 1)
        ca, cb = max(0, c0 - c), min(self.w, c0 + c + 1)
        sub = self.a[ra:rb, ca:cb]
        m = sub >= 99
        if not m.any():
            return None
        rr, cc = np.nonzero(m)
        d = np.hypot(rr + ra - r0, cc + ca - c0) * self.res
        return round(float(d.min()), 4)


class Forensics(Node):
    def __init__(self, args):
        super().__init__('nav_goal_forensics')
        self.args = args
        self.lock = threading.Lock()
        self.t0_wall = time.time()
        self.counts = Counter()
        self.events = []
        self.timeline = []

        self.global_grids = {}
        self.global_msg = None
        self.local_grid = None
        self.plans = []
        self.plan_paths = {}
        self.nav_status_seen = set()
        self.fp_status_seen = set()
        self.nav_status = []
        self.fp_status = []
        self.cmd_vel = []
        self.truth = []
        self.odo = []
        self.nav_fb = []
        self.fp_fb = []
        self.bt = []

        self.create_subscription(Path, '/plan', self.on_plan, QOS_RE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self.on_grid('global', m), QOS_MAP)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: self.on_grid('local', m), QOS_MAP)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_truth, 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.on_cmd(m, 'cmd_vel'), 50)
        self.create_subscription(Twist, '/cmd_vel_chassis',
                                 lambda m: self.on_cmd(m, 'cmd_vel_chassis'), 50)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 lambda m: self.on_status(m, 'navigate_to_pose'), QOS_RE)
        self.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                 lambda m: self.on_status(m, 'follow_path'), QOS_RE)
        self.create_subscription(NAV_TO_POSE_FB, '/navigate_to_pose/_action/feedback',
                                 self.on_nav_fb, QOS_RE)
        self.create_subscription(FOLLOW_PATH_FB, '/follow_path/_action/feedback',
                                 self.on_fp_fb, QOS_RE)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, QOS_RE)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_pub = None
        self.goal_xy = None

    # ---------------- 回调 ----------------
    def t(self):
        return time.time() - self.t0_wall

    def on_plan(self, m):
        poses = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        L = float(sum(math.hypot(poses[i + 1][0] - poses[i][0], poses[i + 1][1] - poses[i][1])
                      for i in range(len(poses) - 1))) if len(poses) > 1 else 0.0
        with self.lock:
            self.counts['plan'] += 1
            idx = len(self.plans)
            self.plans.append({'i': idx, 't': round(self.t(), 2),
                               'frame_id': m.header.frame_id,
                               'n_poses': len(poses), 'length_m': round(L, 4),
                               'first': None if not poses else [round(poses[0][0], 4),
                                                                round(poses[0][1], 4)],
                               'last': None if not poses else [round(poses[-1][0], 4),
                                                               round(poses[-1][1], 4)],
                               'dt_to_goal': (None if (not poses or self.goal_xy is None) else
                                              round(math.hypot(poses[-1][0] - self.goal_xy[0],
                                                               poses[-1][1] - self.goal_xy[1]), 4))})
            if idx < 12:
                self.plan_paths[idx] = poses

    def on_grid(self, tag, m):
        g = Grid(m)
        with self.lock:
            self.counts[tag + '_grid'] += 1
            if tag == 'global':
                self.global_msg = m
                keys = sorted(self.global_grids.keys())
                if keys and g.stamp - self.global_grids[keys[-1]].stamp < 2.0:
                    return
                self.global_grids['t%.1f' % self.t()] = g
            else:
                self.local_grid = g

    def on_odom(self, m):
        with self.lock:
            self.counts['odom'] += 1
            self.odo.append((self.t(), m.pose.pose.position.x, m.pose.pose.position.y,
                             yaw_from_quat(m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                           m.pose.pose.orientation.z, m.pose.pose.orientation.w)))

    def on_truth(self, m):
        with self.lock:
            self.counts['truth'] += 1
            self.truth.append((self.t(), m.pose.pose.position.x, m.pose.pose.position.y,
                               yaw_from_quat(m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                             m.pose.pose.orientation.z, m.pose.pose.orientation.w)))

    def on_cmd(self, m, tag):
        with self.lock:
            self.counts[tag] += 1
            self.cmd_vel.append((self.t(), tag, round(m.linear.x, 4), round(m.angular.z, 4)))

    def on_status(self, msg, who):
        store = self.nav_status if who == 'navigate_to_pose' else self.fp_status
        seen = self.nav_status_seen if who == 'navigate_to_pose' else self.fp_status_seen
        for st in msg.status_list:
            key = (bytes(st.goal_info.goal_id.uuid), int(st.status))
            if key in seen:
                continue
            seen.add(key)
            rec = {'t': round(self.t(), 2), 'who': who,
                   'status': STATUS_NAME.get(int(st.status), int(st.status)),
                   'uuid_tail': bytes(st.goal_info.goal_id.uuid)[-4:].hex()}
            with self.lock:
                self.events.append((round(self.t(), 2), who + '_status', rec))
                store.append(rec)

    def on_nav_fb(self, m):
        with self.lock:
            self.counts['nav_fb'] += 1
            self.nav_fb.append((round(self.t(), 2),
                                round(float(m.feedback.distance_remaining), 3),
                                int(m.feedback.number_of_recoveries),
                                round(float(m.feedback.navigation_time.sec)
                                      + m.feedback.navigation_time.nanosec * 1e-9, 2)))

    def on_fp_fb(self, m):
        with self.lock:
            self.counts['fp_fb'] += 1
            self.fp_fb.append((round(self.t(), 2), round(float(m.feedback.distance_to_goal), 3),
                               round(float(m.feedback.speed), 3)))

    def on_bt(self, m):
        with self.lock:
            self.counts['bt_log'] += 1
            for e in m.event_log:
                self.bt.append((round(self.t(), 2), e.node_name, e.previous_status,
                                e.current_status))

    # ---------------- 位姿/变换 ----------------
    def map_pose(self, timeout=0.3):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def tf_xy_in(self, frame, x, y, timeout=0.3):
        if frame in ('', 'map'):
            return (x, y)
        try:
            tr = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x + c * x - s * y, t.y + s * x + c * y)

    def send_goal(self, x, y):
        if self.goal_pub is None:
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        for _ in range(5):
            self.goal_pub.publish(p)
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--grids', default='')
    ap.add_argument('--settle', type=float, default=30.0)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--goal-forward', type=float, default=None)
    ap.add_argument('--goal', nargs=2, type=float, default=None)
    ap.add_argument('--goal-wait', type=float, default=45.0)
    ap.add_argument('--push', type=float, default=None,
                    help='★ 不经 nav2：直接给 /cmd_vel_chassis 发 vx=该值（±）持续 --push-time 秒，'
                         '量"纯物理能走多远"（把"控制器/代价图"从因果链里摘掉）')
    ap.add_argument('--push-wz', type=float, default=None,
                    help='★ 与 --push 同款但发角速度（纯自转）——量"底盘到底转不转"')
    ap.add_argument('--push-time', type=float, default=12.0)
    ap.add_argument('--push-rate', type=float, default=20.0)
    a, _ = ap.parse_known_args()

    rclpy.init()
    node = Forensics(a)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def log(msg):
        print('[fg] %s' % msg, flush=True)

    log('settle %.0f s …' % a.settle)
    time.sleep(a.settle)
    t_rec = node.t()
    truth0 = node.truth[-1] if node.truth else None
    log('记录窗口 %.0f s（起点真值 %s）' % (a.duration, None if truth0 is None else
                                       [round(v, 4) for v in truth0[1:]]))
    while node.t() - t_rec < a.duration:
        time.sleep(0.5)

    pre_map = node.map_pose()
    pre_truth = node.truth[-1] if node.truth else None
    goal_xy = None
    if a.goal_forward is not None and pre_map is not None:
        goal_xy = (pre_map[0] + a.goal_forward * math.cos(pre_map[2]),
                   pre_map[1] + a.goal_forward * math.sin(pre_map[2]))
        log('车在 map 系 (%.3f, %.3f, %.1f°)，目标 = 车头前 %.2f m → (%.3f, %.3f)'
            % (pre_map[0], pre_map[1], math.degrees(pre_map[2]), a.goal_forward,
               goal_xy[0], goal_xy[1]))
    elif a.goal is not None:
        goal_xy = tuple(a.goal)
        log('显式目标 (%.3f, %.3f)（车在 map 系 %s）' % (goal_xy[0], goal_xy[1], pre_map))
    else:
        log('** 没发目标（goal-forward 拿不到位姿 或 没给 --goal）')

    def sample():
        rec = {'t': round(node.t(), 2)}
        p = node.map_pose()
        rec['map_pose'] = None if p is None else [round(v, 4) for v in p]
        with node.lock:
            if node.truth:
                rec['truth'] = [round(v, 4) for v in node.truth[-1][1:]]
            if node.odo:
                rec['odom'] = [round(v, 4) for v in node.odo[-1][1:]]
            if node.plans:
                rec['last_plan'] = node.plans[-1]['i']
            if node.nav_fb:
                rec['nav_fb'] = node.nav_fb[-1]
            if node.fp_fb:
                rec['fp_fb'] = node.fp_fb[-1]
            if node.cmd_vel:
                rec['cmd'] = node.cmd_vel[-1][1:]
            if node.local_grid is not None:
                rec['local_grid_stamp'] = round(node.local_grid.stamp, 2)
                if goal_xy is not None:
                    rec['goal_in_local'] = node.local_grid.xy_to_cell(*goal_xy) is not None
        node.timeline.append(rec)

    t_send = None
    if goal_xy is not None:
        node.goal_xy = goal_xy
        log('发 /goal_pose %.3f %.3f' % goal_xy)
        node.send_goal(*goal_xy)
        t_send = node.t()
        with node.lock:
            node.events.append((round(t_send, 2), 'goal_sent',
                                {'goal': [round(goal_xy[0], 4), round(goal_xy[1], 4)],
                                 'pre_map_pose': None if pre_map is None else
                                 [round(v, 4) for v in pre_map],
                                 'pre_truth': None if pre_truth is None else
                                 [round(v, 4) for v in pre_truth[1:]],
                                 'settle_s': a.settle, 'duration_s': a.duration}))
            node.global_grids['t_goal'] = (Grid(node.global_msg)
                                           if node.global_msg is not None else None)
            if node.global_grids['t_goal'] is None:
                del node.global_grids['t_goal']
    wait_end = time.time() + (a.goal_wait if goal_xy is not None else 0.0)
    sample()
    nxt = time.time() + 0.5
    while time.time() < wait_end:
        if time.time() >= nxt:
            sample()
            nxt = time.time() + 0.5
        time.sleep(0.1)
    sample()

    # ★ 纯物理推：绕过 nav2（控制器/代价图/BT 全不参与），只给底盘插件发固定 vx
    push = None
    if a.push is not None or a.push_wz is not None:
        vx_push = 0.0 if a.push is None else float(a.push)
        wz_push = 0.0 if a.push_wz is None else float(a.push_wz)
        pub = node.create_publisher(Twist, '/cmd_vel_chassis', 10)
        t0 = time.time()
        truth_start = node.truth[-1] if node.truth else None
        log('★ push vx=%.3f wz=%.3f 持续 %.1f s（绕过 nav2）' % (vx_push, wz_push, a.push_time))
        last_move_t = 0.0
        prev = None
        odom_traj = []
        while time.time() - t0 < a.push_time:
            tw = Twist()
            tw.linear.x = vx_push
            tw.angular.z = wz_push
            pub.publish(tw)
            if node.truth:
                cur = node.truth[-1]
                if prev is not None and math.hypot(cur[1] - prev[1], cur[2] - prev[2]) > 0.002:
                    last_move_t = time.time() - t0
                prev = cur
            if node.odo:
                o = node.odo[-1]
                odom_traj.append([round(o[0], 2), round(o[1], 4), round(o[2], 4),
                                  round(math.degrees(o[3]), 3)])
            time.sleep(1.0 / max(1.0, a.push_rate))
        tw = Twist()
        pub.publish(tw)
        time.sleep(0.5)
        truth_end = node.truth[-1] if node.truth else None
        traj = []
        with node.lock:
            for (tt, x, y, yw) in node.truth:
                if tt >= node.t() - (a.push_time + 1.0):
                    traj.append([round(tt, 2), round(x, 4), round(y, 4), round(math.degrees(yw), 3)])
        d = None
        if truth_start and truth_end:
            d = round(math.hypot(truth_end[1] - truth_start[1],
                                 truth_end[2] - truth_start[2]), 4)
        unw = 0.0
        prev_y = None
        for (_t2, _x2, _y2, _yw2) in (traj if traj else []):
            if prev_y is not None:
                unw += (math.radians(_yw2) - prev_y + math.pi) % (2 * math.pi) - math.pi
            prev_y = math.radians(_yw2)
        unw_o = 0.0
        prev_oy = None
        for (_t3, _x3, _y3, _yw3) in odom_traj:
            if prev_oy is not None:
                unw_o += (math.radians(_yw3) - prev_oy + math.pi) % (2 * math.pi) - math.pi
            prev_oy = math.radians(_yw3)
        push = {'vx': vx_push, 'wz': wz_push, 'push_time_s': a.push_time,
                'd_yaw_deg': round(math.degrees(unw), 2),
                'd_yaw_lio_deg': round(math.degrees(unw_o), 2), 'traj_lio': odom_traj,
                'truth_start': None if truth_start is None else
                [round(v, 4) for v in truth_start[1:]],
                'truth_end': None if truth_end is None else [round(v, 4) for v in truth_end[1:]],
                'disp_m': d, 'requested_m': round(abs(vx_push) * a.push_time, 3),
                'requested_yaw_deg': round(math.degrees(abs(wz_push) * a.push_time), 1),
                't_last_motion_s': round(last_move_t, 2), 'traj': traj}
        log('★ push 结果：位移 %s m（请求 %s m），Δyaw 真值 %s° / LIO %s°（请求 %s°），'
            '最后一次 >2 mm 的运动在 %.2f s' %
            (d, push['requested_m'], push['d_yaw_deg'], push['d_yaw_lio_deg'],
             push['requested_yaw_deg'], last_move_t))

    with node.lock:
        node.global_grids['t_end'] = (Grid(node.global_msg)
                                      if node.global_msg is not None else None)
        if node.global_grids['t_end'] is None:
            del node.global_grids['t_end']

    post_map = node.map_pose()
    with node.lock:
        truth = list(node.truth)
        plans = list(node.plans)
        events = list(node.events)
        timeline = list(node.timeline)
        nav_fb = list(node.nav_fb)
        fp_fb = list(node.fp_fb)
        bt = list(node.bt)
        cmd = list(node.cmd_vel)
        grids = {k: v for k, v in node.global_grids.items() if v is not None}
        local_grid = node.local_grid
        counts = dict(node.counts)
        plan_paths = dict(node.plan_paths)
    post_truth = truth[-1] if truth else None

    def dydisp(p0, p1):
        if not p0 or not p1:
            return None
        return {'dist_m': round(math.hypot(p1[1] - p0[1], p1[2] - p0[2]), 5),
                'dyaw_deg': round(math.degrees(p1[3] - p0[3]), 3)}

    out = {'counts': counts, 'settle_s': a.settle, 'duration_s': a.duration,
           'goal_wait_s': a.goal_wait,
           'goal': None if goal_xy is None else [round(goal_xy[0], 4), round(goal_xy[1], 4)],
           'pre_map_pose': None if pre_map is None else [round(v, 4) for v in pre_map],
           'post_map_pose': None if post_map is None else [round(v, 4) for v in post_map],
           'pre_truth': None if pre_truth is None else [round(v, 4) for v in pre_truth[1:]],
           'post_truth': None if post_truth is None else [round(v, 4) for v in post_truth[1:]],
           'truth_disp_during_goal': dydisp(pre_truth, post_truth),
           'truth_disp_record_window': dydisp(truth0, pre_truth),
           'map_disp_during_goal_m': (None if (pre_map is None or post_map is None) else
                                      round(math.hypot(post_map[0] - pre_map[0],
                                                       post_map[1] - pre_map[1]), 4)),
           'dist_to_goal_after_map_m': (None if (post_map is None or goal_xy is None) else
                                        round(math.hypot(post_map[0] - goal_xy[0],
                                                         post_map[1] - goal_xy[1]), 4)),
           'dist_to_goal_after_truth_m': (None if (post_truth is None or goal_xy is None) else
                                          round(math.hypot(post_truth[1] - goal_xy[0],
                                                           post_truth[2] - goal_xy[1]), 4)),
           'plans': plans, 'events': events, 'timeline': timeline,
           'nav_fb': nav_fb, 'fp_fb': fp_fb, 'bt_log': bt, 'cmd_vel': cmd,
           'cmd_vel_stats': {},
           'global_grids': {k: g.metrics() for k, g in grids.items()},
           'local_grid': None if local_grid is None else local_grid.metrics(),
           't_send_rel': t_send, 'push': push}

    for tag in ('cmd_vel', 'cmd_vel_chassis'):
        rows = [c for c in cmd if c[1] == tag]
        rows_after = [c for c in rows if t_send is not None and c[0] >= t_send]
        out['cmd_vel_stats'][tag] = {
            'n': len(rows), 'n_after_goal': len(rows_after),
            'max_abs_vx': round(max([abs(c[2]) for c in rows], default=0.0), 4),
            'max_abs_wz': round(max([abs(c[3]) for c in rows], default=0.0), 4),
            'max_abs_vx_after_goal': round(max([abs(c[2]) for c in rows_after], default=0.0), 4),
            'max_abs_wz_after_goal': round(max([abs(c[3]) for c in rows_after], default=0.0), 4),
            'n_vx_gt_001_after_goal': sum(1 for c in rows_after if abs(c[2]) > 0.01),
        }

    grid_analysis = {}
    if grids:
        keys = sorted(grids.keys(), key=lambda k: grids[k].stamp)
        g_last = grids[keys[-1]]
        ga = {'snapshots': keys, 'last': keys[-1], 'last_frame': g_last.frame_id}
        if goal_xy is not None:
            per = {}
            for k in keys:
                g = grids[k]
                p = node.tf_xy_in(g.frame_id, goal_xy[0], goal_xy[1])
                if p is None:
                    per[k] = {'note': 'TF 搬不过去', 'frame': g.frame_id}
                    continue
                v = g.value(p[0], p[1])
                per[k] = {'frame': g.frame_id, 'value': v,
                          'cls': None if v is None else cost_class(v),
                          'window_0.5m': g.window(p[0], p[1], 0.5),
                          'nearest_lethal_m': g.nearest_lethal(p[0], p[1], 4.0)}
            ga['goal_cell'] = per
        plans_cost = []
        for i, pts in sorted(plan_paths.items()):
            vals = []
            for (x, y) in pts[::max(1, len(pts) // 60)]:
                p = node.tf_xy_in(g_last.frame_id, x, y)
                if p is None:
                    continue
                v = g_last.value(p[0], p[1])
                vals.append(-1 if v is None else v)
            c = Counter(cost_class(v) for v in vals)
            plans_cost.append({'plan_i': i, 'n_sampled': len(vals),
                               'classes': {k: int(v) for k, v in c.items()}})
        ga['plan_cost_last_grid'] = plans_cost
        for nm, p in (('robot_pre', pre_map), ('robot_post', post_map)):
            if p is None:
                continue
            q = node.tf_xy_in(g_last.frame_id, p[0], p[1])
            if q is None:
                continue
            ga[nm] = {'frame': g_last.frame_id, 'value': g_last.value(q[0], q[1]),
                      'cls': None if g_last.value(q[0], q[1]) is None
                      else cost_class(g_last.value(q[0], q[1])),
                      'window_0.5m': g_last.window(q[0], q[1], 0.5)}
        grid_analysis = ga          # ★ 2026-10-09 修：原来这里忘了把 ga 交回去，JSON 里恒为 {}
    out['grid_analysis'] = grid_analysis

    out['bt_n_entries'] = len(bt)
    out['bt_nodes_transitions'] = sorted({(n, c) for (_t, n, _p, c) in bt})
    out['recoveries_max'] = max([r[2] for r in nav_fb], default=None)
    out['nav_fb_tail'] = nav_fb[-5:]
    out['fp_fb_tail'] = fp_fb[-5:]
    out['timeline_tail'] = timeline[-6:]

    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    if a.grids:
        np.savez_compressed(a.grids, **{('global_' + k): g.a for k, g in grids.items()},
                            **({'local': local_grid.a} if local_grid is not None else {}),
                            **{('plan_%d' % i): np.array(p, dtype=np.float64)
                               for i, p in plan_paths.items() if p})
    log('写 %s（plans=%d, bt=%d, cmd=%d）' % (a.out, len(plans), len(bt), len(cmd)))
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
