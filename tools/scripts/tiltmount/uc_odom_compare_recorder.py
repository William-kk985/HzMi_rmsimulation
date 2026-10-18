#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_compare_recorder.py —— 把 /Odometry、/cloud_registered、odom→base_link TF 逐条写成
**逐位可 diff 的文本**（float 用 repr = 17 位有效数字；点云用 md5(data)）。

用途：同一份 bag 输入流分别喂给 before/after 两个二进制，两次输出 `diff` 应当**完全为空**。
（= docs/tilted_lidar_fidelity.md §O.2 的"同一输入流重放逐位 0 差异"口径。）
"""
import argparse
import hashlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


class Rec(Node):
    def __init__(self, path, seconds, want_tf_edge):
        super().__init__('odom_compare_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        self.f = open(path, 'w')
        self.seconds = seconds
        self.want = want_tf_edge
        self.n_odom = 0
        self.n_cloud = 0
        self.n_tf = 0
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 1000)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 1000)
        self.create_timer(0.2, self.tick)
        self.t0 = self.get_clock().now()

    def tick(self):
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 > self.seconds:
            raise SystemExit(0)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.f.write('ODOM %r %r %r %r %r %r %r %r %s %s\n' % (
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            m.header.frame_id, m.child_frame_id))
        self.n_odom += 1

    def on_cloud(self, m):
        h = hashlib.md5(bytes(m.data)).hexdigest()
        self.f.write('CLOUD %r %s %d %s\n' % (
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            m.header.frame_id, m.width * m.height, h))
        self.n_cloud += 1

    def on_tf(self, m):
        for tr in m.transforms:
            key = '%s=>%s' % (tr.header.frame_id, tr.child_frame_id)
            if self.want and key != self.want:
                continue
            t, r = tr.transform.translation, tr.transform.rotation
            self.f.write('TF %r %s %r %r %r %r %r %r %r\n' % (
                tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, key,
                t.x, t.y, t.z, r.x, r.y, r.z, r.w))
            self.n_tf += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--tf-edge', default='odom=>base_link')
    a = ap.parse_args()
    rclpy.init()
    n = Rec(a.out, a.seconds, a.tf_edge)
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    n.f.close()
    print('recorded odom=%d cloud=%d tf(%s)=%d → %s' % (
        n.n_odom, n.n_cloud, a.tf_edge, n.n_tf, a.out))


if __name__ == '__main__':
    main()
