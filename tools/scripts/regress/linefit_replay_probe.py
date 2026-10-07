#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""linefit_replay_probe.py —— 把一帧点云回放给**活的** linefit 节点（无 Gazebo），量它判出多少地面点。

为什么需要（docs/robot_models.md §11.4）：`gravity_aligned_frame` 那条分支曾经因为
`Eigen::Affine3d tf;` 默认构造不清零，把点云塌到原点 ⇒ `/segmentation/ground` 恒 0 点。
"离线复算"（linefit_offline_probe.py）只能复算**算法**，复算不了**节点里的这段预处理**
⇒ 需要一个"只有 TF + 一帧点云 + 真节点"的最小回放台，用来：
  · 回归：默认路径（gravity_aligned_frame=""）的点数**修前修后一致**；
  · 验证：gravity_aligned_frame=base_link 修后能出正常地面点（修前恒 0）。

用法（**一次 bash 调用**跑完，自带隔离：HOME/ROS_DOMAIN_ID/unset DISPLAY）：
  # ① 起节点（与 bringup 同款参数文件）
  ros2 run linefit_ground_segmentation_ros ground_segmentation_node --ros-args -p use_sim_time:=false \
      --params-file install/linefit_ground_segmentation_ros/share/linefit_ground_segmentation_ros/config/segmentation_sim.yaml &
  # ② 回放一帧
  python3 tools/scripts/regress/linefit_replay_probe.py <frame.csv> "<base_link→livox rpy, 度>" <sensor_height>

它做三件事：发静态 TF base_link→livox_frame（给定 rpy）、以 10 Hz 发 PointCloud2(frame_id=livox_frame)、
订阅 `/segmentation/{ground,obstacle}` 统计点数（**BEST_EFFORT**，与节点发布端匹配）。
"""
import sys, math, time, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

def main():
    frame, rpy_s, sh = sys.argv[1], sys.argv[2], float(sys.argv[3])
    rpy = [math.radians(float(v)) for v in rpy_s.split()]
    pts = np.loadtxt(frame, delimiter=',', skiprows=1).astype(np.float32)
    rclpy.init()
    node = Node('replay_linefit')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
    br = tf2_ros.StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'; t.child_frame_id = 'livox_frame'
    cr, sr = math.cos(rpy[0]/2), math.sin(rpy[0]/2)
    cp, sp = math.cos(rpy[1]/2), math.sin(rpy[1]/2)
    cy, sy = math.cos(rpy[2]/2), math.sin(rpy[2]/2)
    t.transform.rotation.w = cr*cp*cy + sr*sp*sy
    t.transform.rotation.x = sr*cp*cy - cr*sp*sy
    t.transform.rotation.y = cr*sp*cy + sr*cp*sy
    t.transform.rotation.z = cr*cp*sy - sr*sp*cy
    t.transform.translation.x = 0.000561701465058485
    t.transform.translation.y = 0.130915824456595
    t.transform.translation.z = 0.15702816968305
    br.sendTransform(t)
    pub = node.create_publisher(PointCloud2, '/livox/lidar/pointcloud', QOS)
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    hdr = __import__('std_msgs.msg', fromlist=['Header']).Header()
    hdr.frame_id = 'livox_frame'; hdr.stamp = node.get_clock().now().to_msg()
    msg = point_cloud2.create_cloud(hdr, fields, pts)
    stats = {'ground': [], 'obstacle': []}
    def mk(k):
        def cb(m):
            stats[k].append(m.width * m.height)
        return cb
    subq = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, '/segmentation/ground', mk('ground'), subq)
    node.create_subscription(PointCloud2, '/segmentation/obstacle', mk('obstacle'), subq)
    stop = False
    def spin():
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    th = threading.Thread(target=spin, daemon=True); th.start()
    t0 = time.time()
    while time.time() - t0 < 12.0:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.1)
    time.sleep(1.0)
    stop = True; time.sleep(0.3)
    g = sorted(stats['ground']); o = sorted(stats['obstacle'])
    print('回放 %s  rpy=%s  sensor_height=%.4f  点数=%d' % (frame, rpy_s, sh, len(pts)))
    print('  /segmentation/ground 帧数=%d 中位点数=%s  (前几帧 %s)' % (len(g), g[len(g)//2] if g else None, g[:5]))
    print('  /segmentation/obstacle 帧数=%d 中位点数=%s' % (len(o), o[len(o)//2] if o else None))
    node.destroy_node(); rclpy.shutdown()

main()
