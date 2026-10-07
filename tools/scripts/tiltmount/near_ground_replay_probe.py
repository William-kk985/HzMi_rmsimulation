#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""near_ground_replay_probe.py —— 把**一帧真点云**回放给**活的**地面分割节点，量近地剔除

为什么需要一个"回放台"（而不是一直跑 Gazebo）：近地剔除（`obstacle_near_ground_m`）是
**在点云这一级**改标签（`/segmentation/obstacle` → 改判 ground），所以判据必须落在
"**同一帧**、只换参数"的干净 A/B 上 —— Gazebo 每次跑的地面峰、出生位姿、LIO 初始化都略有不同
（本仓已知的跑间散布），拿两次独立跑比"标签数"会被噪声吃掉。

做法（与 `tools/scripts/regress/linefit_replay_probe.py` 同一套最小台子，只多了近地剔除）：
  ① 起一个真的 `ground_segmentation_node`（**由本脚本的同一条 bash 调用负责**，参数由调用方给）；
  ② 本脚本发一帧 `PointCloud2`（`frame_id=livox_frame`）到 `/livox/lidar/pointcloud`，
     并用**静态 TF** 给出 `base_link←livox_frame`（rpy 由命令行给，默认 `0 0 0`）；
  ③ 订阅 `/segmentation/{ground,obstacle}`，报**中位点数**与**逐帧配对**（ground+obstacle）。
  ⇒ 对同一帧点云，`obstacle_near_ground_m` 取 0.0 / 0.05 就是**唯一变量**，点数的差就是
     "被判成贴地、改判成 ground"的点数。

用法（**一次 bash 调用**跑完，自带隔离：HOME/ROS_DOMAIN_ID/unset DISPLAY）：
  F=.tmp_tiltmount/k7_geo/dump/raw_0.csv
  for ng in 0.0 0.05; do
    ros2 run linefit_ground_segmentation_ros ground_segmentation_node --ros-args \\
      -p use_sim_time:=false -p sensor_height:=0.2595 -p gravity_aligned_frame:="" \\
      -p r_min:=0.2 -p max_dist_to_line:=0.05 -p obstacle_near_ground_m:=$ng &
    python3 tools/scripts/tiltmount/near_ground_replay_probe.py "$F" "0 0 0" 0.2595 $ng
    kill %1
  done

输出：一行 JSON（`--json`）+ 人读摘要。
"""

import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frame')
    ap.add_argument('rpy', nargs='?', default='0 0 0')
    ap.add_argument('sensor_height', nargs='?', type=float, default=0.2595)
    ap.add_argument('near_ground_m', nargs='?', type=float, default=0.0)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()

    rpy = [math.radians(float(v)) for v in args.rpy.split()]
    pts = np.loadtxt(args.frame, delimiter=',', skiprows=1).astype(np.float32)
    rclpy.init()
    node = Node('replay_near_ground')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
    br = tf2_ros.StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'livox_frame'
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy
    t.transform.translation.x = 0.000561701465058485
    t.transform.translation.y = 0.130915824456595
    t.transform.translation.z = 0.15702816968305
    br.sendTransform(t)

    pub = node.create_publisher(PointCloud2, '/livox/lidar/pointcloud', QOS)
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    from std_msgs.msg import Header
    hdr = Header()
    hdr.frame_id = 'livox_frame'
    msg = point_cloud2.create_cloud(hdr, fields, pts)
    stats = {'ground': [], 'obstacle': [], 'step_edge': []}

    def mk(k):
        def cb(m):
            stats[k].append(int(m.width * m.height))
        return cb
    node.create_subscription(PointCloud2, '/segmentation/ground', mk('ground'), QOS)
    node.create_subscription(PointCloud2, '/segmentation/obstacle', mk('obstacle'), QOS)
    node.create_subscription(PointCloud2, '/segmentation/step_edge', mk('step_edge'), QOS)
    stop = False

    def spin():
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    th = threading.Thread(target=spin, daemon=True)
    th.start()
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1.0)
    stop = True
    time.sleep(0.3)
    rclpy.shutdown()

    def med(a):
        return float(np.median(a)) if a else None
    n_pub = len(stats['ground']) + len(stats['obstacle']) and max(
        len(stats['ground']), len(stats['obstacle'])) or 0
    out = {
        'frame': args.frame, 'rpy_deg': [round(math.degrees(v), 3) for v in rpy],
        'sensor_height': args.sensor_height, 'near_ground_m': args.near_ground_m,
        'n_input_points': int(len(pts)),
        'frames': {k: len(v) for k, v in stats.items()},
        'ground_median': med(stats['ground']),
        'obstacle_median': med(stats['obstacle']),
        'step_edge_median': med(stats['step_edge']),
        'sum_median': (med(stats['ground']) or 0) + (med(stats['obstacle']) or 0),
    }
    if args.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        print('回放 %(frame)s  rpy=%(rpy_deg)s  sensor_height=%(sensor_height).4f  '
              'near_ground_m=%(near_ground_m).2f' % out)
        print('  输入 %(n_input_points)d 点 | ground 中位 %(ground_median)s + obstacle 中位 '
              '%(obstacle_median)s = %(sum_median)s | step_edge 中位 %(step_edge_median)s | '
              '帧数 %(frames)s' % out)


if __name__ == '__main__':
    main()
