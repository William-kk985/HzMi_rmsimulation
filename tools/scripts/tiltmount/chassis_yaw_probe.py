#!/usr/bin/env python3
"""chassis_yaw_probe.py —— 台架探针：只发 /cmd_vel_chassis，只读真值/关节角，回答「底盘转不转」。

订阅：/odom_ground_truth（planar_move 发的是 **模型 WorldPose**，逐条真值）、/joint_states（可选）
发布：/cmd_vel_chassis（固定 vx/wz，持续 --push-wall 墙钟秒）

输出 JSON（stdout 或 --out）：
  · yaw_deg / d_yaw_deg                真值 yaw 与窗内净转（解缠绕）
  · requested_wall_yaw_deg             按**墙钟**请求（与 §L 口径一致）
  · requested_sim_yaw_deg              按**仿真时间**请求（真正的物理请求）
  · exec_ratio_wall / exec_ratio_sim
  · yaw_rate_sim_deg_s                 窗内实测平均角速度（°/s 仿真时间）
  · drift_deg_s                        push 前 settle 窗内的自由漂移速率
  · wz_actual_deg_s                    /odom_ground_truth.twist.angular.z 的窗内中位
  · joints / wheel_spin_deg_s / steer_deg  关节角读数（若有 /joint_states）
"""
import argparse
import json
import os
import math
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def unwrap(seq):
    out = []
    acc = 0.0
    prev = None
    for v in seq:
        if prev is not None:
            acc += (v - prev + math.pi) % (2 * math.pi) - math.pi
        prev = v
        out.append(acc)
    return out


class Probe(Node):
    def __init__(self):
        super().__init__('wz_bench_probe')
        self.lock = threading.Lock()
        self.truth = []          # (t_sim, x, y, yaw_deg, wz_deg_s)
        self.joints = []         # (t_sim, {name: pos_deg})
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_odom, 50)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel_chassis', 10)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        with self.lock:
            self.truth.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                               yaw, math.degrees(m.twist.twist.angular.z),
                               m.pose.pose.position.z, math.degrees(roll), math.degrees(pitch)))

    def on_js(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = {n: math.degrees(p) for n, p in zip(m.name, m.position)}
        with self.lock:
            self.joints.append((t, d))

    def sim_now(self):
        with self.lock:
            return self.truth[-1][0] if self.truth else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wz', type=float, default=1.0)
    ap.add_argument('--vx', type=float, default=0.0)
    ap.add_argument('--settle', type=float, default=8.0, help='push 前等待的**仿真**秒')
    ap.add_argument('--push-wall', type=float, default=12.0, help='push 持续**墙钟**秒（与 §L 同口径）')
    ap.add_argument('--post', type=float, default=2.0, help='push 后再记录的仿真秒')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    th.start()

    t_wall0 = time.time()
    while n.sim_now() is None and time.time() - t_wall0 < 120:
        time.sleep(0.2)
    if n.sim_now() is None:
        print('NO /odom_ground_truth', file=sys.stderr)
        return 2
    t_sim0 = n.sim_now()
    # 等 settle（仿真秒）
    while n.sim_now() - t_sim0 < a.settle:
        time.sleep(0.05)
        if time.time() - t_wall0 > 300:
            print('settle timeout', file=sys.stderr)
            return 3
    t_push0 = n.sim_now()
    wall0 = time.time()
    rate = 100.0
    while time.time() - wall0 < a.push_wall:
        tw = Twist()
        tw.linear.x = a.vx
        tw.angular.z = a.wz
        n.pub.publish(tw)
        time.sleep(1.0 / rate)
    t_push1 = n.sim_now()
    wall1 = time.time()
    tw = Twist()
    n.pub.publish(tw)
    while n.sim_now() - t_push1 < a.post:
        time.sleep(0.05)
    t_end = n.sim_now()

    with n.lock:
        truth = list(n.truth)
        joints = list(n.joints)
    win = [r for r in truth if t_push0 <= r[0] <= t_push1]
    pre = [r for r in truth if t_sim0 <= r[0] < t_push0]
    post = [r for r in truth if r[0] > t_push1]
    out = {'wz_cmd': a.wz, 'vx_cmd': a.vx,
           'settle_sim_s': round(t_push0 - t_sim0, 3),
           'push_sim_s': round(t_push1 - t_push0, 3),
           'push_wall_s': round(wall1 - wall0, 3),
           'rtf_push': round((t_push1 - t_push0) / max(1e-9, wall1 - wall0), 3)}
    if len(win) >= 3:
        ys = unwrap([r[3] for r in win])
        d_yaw = math.degrees(ys[-1] - ys[0])
        out.update({
            'd_yaw_deg': round(d_yaw, 3),
            'requested_wall_yaw_deg': round(math.degrees(abs(a.wz) * a.push_wall), 1),
            'requested_sim_yaw_deg': round(math.degrees(abs(a.wz) * (t_push1 - t_push0)), 1),
            'exec_ratio_wall': round(abs(d_yaw) / max(1e-9, math.degrees(abs(a.wz) * a.push_wall)), 5),
            'exec_ratio_sim': round(abs(d_yaw) / max(1e-9, math.degrees(abs(a.wz) * (t_push1 - t_push0))), 5),
            'yaw_rate_sim_deg_s': round(d_yaw / max(1e-9, t_push1 - t_push0), 3),
            'wz_actual_deg_s_median': round(statistics.median([r[4] for r in win]), 3),
            'disp_m': round(math.hypot(win[-1][1] - win[0][1], win[-1][2] - win[0][2]), 4),
            'yaw_traj': [[round(r[0], 3), round(r[3], 3)] for r in win],
            'z_start': round(win[0][5], 4), 'z_end': round(win[-1][5], 4),
            'roll_start': round(win[0][6], 3), 'roll_end': round(win[-1][6], 3),
            'pitch_start': round(win[0][7], 3), 'pitch_end': round(win[-1][7], 3),
        })
    if len(pre) >= 3:
        step = max(1, len(pre) // 30)
        out['pre_traj'] = [[round(r[0], 2), round(r[3], 3)] for r in pre[::step]]
        pys = unwrap([r[3] for r in pre])
        out['drift_deg_s'] = round(math.degrees(pys[-1] - pys[0]) / max(1e-9, pre[-1][0] - pre[0][0]), 4)
        out['drift_total_deg'] = round(math.degrees(pys[-1] - pys[0]), 3)
    if joints:
        # 轮子转速（j6..j9）与转向角（j2..j5）：窗内首末差 / 时间
        jwin = [j for j in joints if t_push0 <= j[0] <= t_push1]
        if len(jwin) >= 3:
            dt = jwin[-1][0] - jwin[0][0]
            spin = {}
            for k in ('j6', 'j7', 'j8', 'j9'):
                if k in jwin[0][1]:
                    d = jwin[-1][1][k] - jwin[0][1][k]
                    spin[k] = {'delta_deg': round(d, 2),
                               'rate_deg_s': round(d / max(1e-9, dt), 2),
                               'expect_roll_deg_s_if_no_slip': None}
            steer = {k: {'start': round(jwin[0][1].get(k, float('nan')), 2),
                         'end': round(jwin[-1][1].get(k, float('nan')), 2)}
                     for k in ('j2', 'j3', 'j4', 'j5')}
            out['wheel_spin'] = spin
            out['steer'] = steer
    out['n_odom'] = len(truth)
    out['n_joint'] = len(joints)
    txt = json.dumps(out, ensure_ascii=False)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(txt)
    print(txt)
    sys.stdout.flush()
    os._exit(0)      # rclpy 的 spin 线程在 shutdown 时会 terminate（实测 abort），直接退


if __name__ == '__main__':
    sys.exit(main())
