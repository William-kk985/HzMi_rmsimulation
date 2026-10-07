#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_model_probe.py —— 「换机器人模型」时一次量齐所有会影响的量（只读探针）。

用途（docs/robot_models.md §5 的实测数字就是它跑出来的）：
  · 机器人几何/传感器落点：TF 里 base_link→{livox_frame,imu_link,lidar_link,turret_*,wheel_*} 在不在、数值多少；
  · 点云：/livox/lidar/pointcloud 的频率、每帧点数、**传感器系 z 直方图的地面峰**（= linefit sensor_height
    的标定方法）、近距自击点（r<0.3 m）个数；
  · IMU：/livox/imu 频率；
  · 分割：/segmentation/obstacle 与 /segmentation/ground 的每帧点数（linefit 有没有在干活）；
  · /scan：帧数、**有效波束数**、最近回波、最近回波方位、inf 占比（是否"看不到东西"）；
  · 里程计：/odom（LIO）与 /odom_ground_truth（planar_move 真值）的路径长度、终点位置/偏航差（漂移）；
  · RTF：/clock 的仿真时间推进 / 墙钟。
可选：顺便按 `--drive-*` 发一段 /cmd_vel_chassis（结束**一定**补一段零速），
再按 `--yaw-*` 发一段原地偏航（Phase 3 起：用来分别量"直线"和"偏航"两段相对真值的漂移）。

只订阅 + 只发 /cmd_vel_chassis（该话题的唯一订阅者是 Gazebo 的 planar_move 插件），
不发 TF、不发 /segmentation/*、不发 /map ⇒ 不动本仓任何契约。
"""
import argparse
import json
import math
import sys
import threading
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros

# 与发布者兼容的最宽松 QoS：BEST_EFFORT + KEEP_LAST（pub 是 RELIABLE 时也兼容）
QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rpy_of(q):
    """完整 roll/pitch/yaw（度）—— 用来判"车是不是翻了/被顶起来了"。"""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return [round(math.degrees(roll), 2), round(math.degrees(pitch), 2),
            round(math.degrees(yaw_of(q)), 2)]


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Probe(Node):
    def __init__(self, args):
        super().__init__('robot_model_probe')
        self.args = args
        self.lock = threading.Lock()

        # ---- 计数器 / 统计（全部在锁里改）----
        self.n = Counter()
        self.cloud_points = []          # 每帧点数
        self.cloud_z_hist = Counter()   # 传感器系 z 直方图（0.01 m 桶）
        self.cloud_z = []               # 采样下来的原始 z（限幅采样，防内存）
        self.cloud_r_near = []          # 近距点（r<0.5）的水平半径
        # ★ Phase 3：自击比例的**精确**计数（云点总数 + 各阈值内的点数；不采样、不截断）
        self.cloud_r_stats = Counter()  # {'total':n, 'lt0.05':n, 'lt0.12':n, 'lt0.2':n, 'lt0.3':n}
        self.ground_r_hist = Counter()  # /segmentation/ground 的**水平距离**直方图（0.5 m 桶）
        self.scan_beams = []            # 每帧有效波束数
        self.scan_nearest = []          # 每帧最近有效回波
        self.scan_inf_ratio = []
        self.scan_last = None
        self.scan_bands = {}
        self.imu_stamps = []
        self.cloud_stamps = []
        self.scan_stamps = []
        self.odom_last = None
        self.gt_last = None
        self.odom_path = 0.0
        self.gt_path = 0.0
        self._odom_prev = None
        self._gt_prev = None
        self.obstacle_points = []
        self.ground_points = []
        self.clock_first = None
        self.clock_last = None
        self.wall_first = None
        self.wall_last = None
        self.tf_frames = set()

        self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, QOS)
        self.create_subscription(PointCloud2, args.obstacle_topic, self.on_obstacle, QOS)
        self.create_subscription(PointCloud2, args.ground_topic, self.on_ground, QOS)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, QOS)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, QOS)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, QOS)
        self.create_subscription(Odometry, args.gt_topic, self.on_gt, QOS)
        self.create_subscription(Clock, '/clock', self.on_clock, QOS)
        self.create_subscription(
            __import__('tf2_msgs.msg', fromlist=['TFMessage']).TFMessage, '/tf', self.on_tf, QOS)
        self.create_subscription(
            __import__('tf2_msgs.msg', fromlist=['TFMessage']).TFMessage, '/tf_static', self.on_tf,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.dump_dir = args.dump_cloud
        self.dump_frames = 0
        self.dump_scan_dir = args.dump_scan
        self.dump_scans = 0
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ---------------- 回调 ----------------
    def on_cloud(self, msg):
        with self.lock:
            self.n['cloud'] += 1
            n = msg.width * msg.height
            self.cloud_points.append(n)
            self.cloud_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            keep = 0
            rows = []
            for i, p in enumerate(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                rows.append((x, y, z))
                if len(self.cloud_z) < 200000:
                    self.cloud_z.append(z)
                self.cloud_z_hist[round(z / 0.01)] += 1
                r = math.hypot(x, y)
                self.cloud_r_stats['total'] += 1
                for thr, key in ((0.05, 'lt0.05'), (0.12, 'lt0.12'), (0.2, 'lt0.2'), (0.3, 'lt0.3')):
                    if r < thr:
                        self.cloud_r_stats[key] += 1
                if r < 0.5 and len(self.cloud_r_near) < 20000:
                    self.cloud_r_near.append(round(r, 3))
                if r < 1.5:
                    self.cloud_zmin_r15 = min(getattr(self, 'cloud_zmin_r15', 9e9), z)
                if r < 3.0:
                    self.cloud_zmin_r30 = min(getattr(self, 'cloud_zmin_r30', 9e9), z)
                keep += 1
            if self.dump_dir and self.dump_frames < 3:
                self.dump_frames += 1
                import os as _os
                _os.makedirs(self.dump_dir, exist_ok=True)
                with open(_os.path.join(self.dump_dir, 'cloud_frame_%02d.csv' % self.dump_frames),
                          'w') as f:
                    f.write('x,y,z\n')
                    for x, y, z in rows:
                        f.write('%.4f,%.4f,%.4f\n' % (x, y, z))

    def on_obstacle(self, msg):
        with self.lock:
            self.n['obstacle'] += 1
            self.obstacle_points.append(msg.width * msg.height)

    def on_ground(self, msg):
        with self.lock:
            self.n['ground'] += 1
            self.ground_points.append(msg.width * msg.height)
            # ★ Phase 3：地面点的**距离分布** —— 用来量 `speed_limit_lookahead_m` 是否有效
            #   （hzmirm 的教训：雷达高、下视窄 ⇒ 前瞻距离内根本没有地面点）。
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                rr = math.hypot(float(p[0]), float(p[1]))
                self.ground_r_hist[round(rr / 0.5) * 0.5] += 1

    def on_imu(self, msg):
        with self.lock:
            self.n['imu'] += 1
            self.imu_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def on_scan(self, msg):
        with self.lock:
            self.n['scan'] += 1
            self.scan_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            rng = msg.ranges
            finite = [(i, r) for i, r in enumerate(rng)
                      if math.isfinite(r) and r > 0.0]
            self.scan_beams.append(len(finite))
            self.scan_inf_ratio.append(1.0 - len(finite) / max(1, len(rng)))
            for lo, hi, key in ((0.0, 1.0, 'lt1'), (1.0, 2.0, 'b1_2'), (2.0, 4.0, 'b2_4'),
                                (4.0, 7.0, 'b4_7'), (7.0, 1e9, 'gt7')):
                self.scan_bands[key] = self.scan_bands.get(key, 0) + sum(
                    1 for _, r in finite if lo <= r < hi)
            if self.dump_scan_dir and self.dump_scans < 3:
                self.dump_scans += 1
                import os as _os
                _os.makedirs(self.dump_scan_dir, exist_ok=True)
                with open(_os.path.join(self.dump_scan_dir, 'scan_%02d.csv' % self.dump_scans),
                          'w') as f:
                    f.write('i,angle_deg,range\n')
                    for i, r in enumerate(msg.ranges):
                        f.write('%d,%.3f,%s\n' % (i, math.degrees(msg.angle_min + i * msg.angle_increment),
                                                  ('%.3f' % r) if math.isfinite(r) else 'inf'))
            if finite:
                i, r = min(finite, key=lambda t: t[1])
                ang = msg.angle_min + i * msg.angle_increment
                self.scan_nearest.append((round(r, 3), round(math.degrees(ang), 1)))
                self.scan_last = {'range': round(r, 3),
                                  'angle_deg': round(math.degrees(ang), 1),
                                  'n_finite': len(finite), 'n_beams': len(rng),
                                  'frame_id': msg.header.frame_id}

    def on_odom(self, msg):
        with self.lock:
            self.n['odom'] += 1
            p = msg.pose.pose.position
            self.odom_last = {'x': p.x, 'y': p.y, 'z': p.z,
                              'yaw': yaw_of(msg.pose.pose.orientation),
                              'rpy_deg': rpy_of(msg.pose.pose.orientation)}
            rp = rpy_of(msg.pose.pose.orientation)
            self.odom_tilt_max = max(getattr(self, 'odom_tilt_max', 0.0),
                                     abs(rp[0]), abs(rp[1]))
            if self._odom_prev is not None:
                self.odom_path += math.hypot(p.x - self._odom_prev[0], p.y - self._odom_prev[1])
            self._odom_prev = (p.x, p.y)

    def on_gt(self, msg):
        with self.lock:
            self.n['gt'] += 1
            p = msg.pose.pose.position
            self.gt_last = {'x': p.x, 'y': p.y, 'z': p.z,
                            'yaw': yaw_of(msg.pose.pose.orientation),
                            'rpy_deg': rpy_of(msg.pose.pose.orientation)}
            rp = rpy_of(msg.pose.pose.orientation)
            self.gt_tilt_max = max(getattr(self, 'gt_tilt_max', 0.0), abs(rp[0]), abs(rp[1]))
            if self._gt_prev is not None:
                self.gt_path += math.hypot(p.x - self._gt_prev[0], p.y - self._gt_prev[1])
            self._gt_prev = (p.x, p.y)

    def on_clock(self, msg):
        with self.lock:
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.clock_first is None:
                self.clock_first = t
                self.wall_first = time.time()
            self.clock_last = t
            self.wall_last = time.time()

    def on_tf(self, msg):
        with self.lock:
            for tr in msg.transforms:
                self.tf_frames.add((tr.header.frame_id, tr.child_frame_id))

    # ---------------- 工具 ----------------
    def pose_snapshot(self):
        """取一份 (odom, gt) 位姿快照（相位漂移用）。

        ⚠️ 口径（Phase 3 更正）：**漂移 = 位移之差**，不是 odom 与 gt 的绝对位置之差。
        原因：两条轨迹的原点不同 —— `/odom` 的原点是 LIO 初始化处（≈出生点）、
        `/odom_ground_truth` 的原点就是世界原点 ⇒ 绝对差里混着"出生点偏移"（默认模型
        在 RMUL2026 上是 5.45 m！）。Phase 2 的表里那一列就是这个量，别当漂移读。
        """
        with self.lock:
            o = dict(self.odom_last) if self.odom_last else None
            g = dict(self.gt_last) if self.gt_last else None
        return o, g

    @staticmethod
    def phase_drift(p0, p1):
        """两段快照之间的相对漂移：|Δxy_odom − Δxy_gt| 与偏航差（度）。"""
        o0, g0 = p0
        o1, g1 = p1
        if not (o0 and g0 and o1 and g1):
            return None
        dox, doy = o1['x'] - o0['x'], o1['y'] - o0['y']
        dgx, dgy = g1['x'] - g0['x'], g1['y'] - g0['y']
        dyaw_o = wrap(o1['yaw'] - o0['yaw'])
        dyaw_g = wrap(g1['yaw'] - g0['yaw'])
        return {'d_odom_xy': [round(dox, 3), round(doy, 3)],
                'd_gt_xy': [round(dgx, 3), round(dgy, 3)],
                'drift_xy': round(math.hypot(dox - dgx, doy - dgy), 4),
                'drift_yaw_deg': round(math.degrees(wrap(dyaw_o - dyaw_g)), 3),
                'd_odom_yaw_deg': round(math.degrees(dyaw_o), 2),
                'd_gt_yaw_deg': round(math.degrees(dyaw_g), 2)}

    def rate(self, stamps):
        if len(stamps) < 3:
            return 0.0
        span = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / span if span > 1e-6 else 0.0

    def lookup(self, parent, child):
        try:
            tr = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=1.0))
            t = tr.transform.translation
            r = tr.transform.rotation
            return {'ok': True, 'xyz': [round(t.x, 4), round(t.y, 4), round(t.z, 4)],
                    'rpy_deg': [round(math.degrees(v), 2) for v in
                                (math.atan2(2 * (r.w * r.x + r.y * r.z), 1 - 2 * (r.x ** 2 + r.y ** 2)),
                                 math.asin(max(-1, min(1, 2 * (r.w * r.y - r.z * r.x)))),
                                 math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y ** 2 + r.z ** 2)))]}
        except Exception as e:                                    # noqa: BLE001
            return {'ok': False, 'error': str(e)[:120]}

    # ---------------- 主流程 ----------------
    def run(self):
        a = self.args
        self.get_logger().info('等待就绪（cloud/imu/scan）…')
        t0 = time.time()
        while time.time() - t0 < a.ready_timeout:
            with self.lock:
                ok = self.n['cloud'] > 3 and self.n['imu'] > 50
                if a.require_scan:
                    ok = ok and self.n['scan'] > 3
                if a.require_odom:
                    ok = ok and self.n['odom'] > 5
            if ok:
                break
            time.sleep(0.5)
        with self.lock:
            ready = {'cloud': self.n['cloud'], 'imu': self.n['imu'], 'scan': self.n['scan'],
                     'odom': self.n['odom'], 'obstacle': self.n['obstacle']}
        self.get_logger().info('就绪计数 = %s（等了 %.1fs）' % (ready, time.time() - t0))

        # 清一次统计，让窗口干净
        with self.lock:
            self.cloud_points.clear(); self.cloud_z.clear(); self.cloud_z_hist.clear()
            self.cloud_r_near.clear(); self.scan_beams.clear(); self.scan_nearest.clear()
            self.scan_inf_ratio.clear(); self.imu_stamps.clear(); self.cloud_stamps.clear()
            self.scan_bands.clear()
            self.scan_stamps.clear(); self.obstacle_points.clear(); self.ground_points.clear()
            self.odom_path = 0.0; self.gt_path = 0.0
            self._odom_prev = None; self._gt_prev = None
            self.n = Counter()
            self.wall_start = time.time()

        # 静止窗口
        self.get_logger().info('静止窗口 %.1fs …' % a.duration)
        time.sleep(a.duration)

        ph0 = self.pose_snapshot()
        drove = 0.0
        if a.drive_seconds > 0:
            self.get_logger().info('发 /cmd_vel_chassis 前进 %.2f m/s × %.1fs …'
                                   % (a.drive_speed, a.drive_seconds))
            tw = Twist()
            tw.linear.x = a.drive_speed
            tw.angular.z = a.drive_turn
            t_end = time.time() + a.drive_seconds
            while time.time() < t_end:
                self.cmd_pub.publish(tw)
                time.sleep(1.0 / 20.0)
            drove = time.time() - (t_end - a.drive_seconds)
            self.get_logger().info('补零速（契约要求）…')
            stop = Twist()
            for _ in range(20):
                self.cmd_pub.publish(stop)
                time.sleep(0.05)
            time.sleep(a.settle_after_drive)
        ph1 = self.pose_snapshot()

        yawed = 0.0
        if a.yaw_seconds > 0:
            self.get_logger().info('发 /cmd_vel_chassis 原地偏航 %.2f rad/s × %.1fs …'
                                   % (a.yaw_rate, a.yaw_seconds))
            tw = Twist()
            tw.angular.z = a.yaw_rate
            t_end = time.time() + a.yaw_seconds
            while time.time() < t_end:
                self.cmd_pub.publish(tw)
                time.sleep(1.0 / 20.0)
            yawed = time.time() - (t_end - a.yaw_seconds)
            self.get_logger().info('补零速（契约要求）…')
            stop = Twist()
            for _ in range(20):
                self.cmd_pub.publish(stop)
                time.sleep(0.05)
            time.sleep(a.settle_after_drive)
        ph2 = self.pose_snapshot()

        return self.report(drove, ph0, ph1, ph2, yawed)

    def report(self, drove, ph0=None, ph1=None, ph2=None, yawed=0.0):
        with self.lock:
            n = dict(self.n)
            cps = list(self.cloud_points)
            op = list(self.obstacle_points)
            gp = list(self.ground_points)
            zb = Counter(self.cloud_z_hist)
            zs = list(self.cloud_z)
            near = list(self.cloud_r_near)
            beams = list(self.scan_beams)
            infr = list(self.scan_inf_ratio)
            odom_last, gt_last = self.odom_last, self.gt_last
            odom_path, gt_path = self.odom_path, self.gt_path
            imu_st, cl_st, sc_st = list(self.imu_stamps), list(self.cloud_stamps), list(self.scan_stamps)
            wall = time.time() - getattr(self, 'wall_start', time.time())
            c_first, c_last = self.clock_first, self.clock_last
            w_first, w_last = self.wall_first, self.wall_last
            tf_frames = sorted(self.tf_frames)
        frames = {'base_link': 'base_link', 'livox_frame': 'livox_frame', 'imu_link': 'imu_link',
                  'lidar_link': 'lidar_link', 'turret_head': 'turret_head',
                  'turret_base': 'turret_base', 'wheel_fl': 'wheel_fl', 'wheel_rr': 'wheel_rr',
                  'barrel': 'barrel', 'base_link_fake': 'base_link_fake'}
        tfs = {k: self.lookup('base_link', v) for k, v in frames.items()}

        def med(x):
            x = sorted(x)
            return x[len(x) // 2] if x else None

        # 地面峰 = z 直方图里最负的显著峰（|z|<2 m 范围）
        peak_z, peak_cnt = None, 0
        for k, c in zb.items():
            z = k * 0.01
            if -2.0 < z < -0.02 and c > peak_cnt:
                peak_z, peak_cnt = z, c
        hist_top = sorted(((round(k * 0.01, 2), c) for k, c in zb.items()),
                          key=lambda t: -t[1])[:6]

        out = {
            'wall_seconds': round(wall, 2),
            'rtf': round((c_last - c_first) / (w_last - w_first), 3) if (c_first is not None and w_last > w_first) else None,
            'counts': n,
            'rates_hz': {'cloud': round(self.rate(cl_st), 2), 'imu': round(self.rate(imu_st), 2),
                         'scan': round(self.rate(sc_st), 2)},
            'cloud': {
                'points_per_frame': {'min': min(cps) if cps else None, 'median': med(cps),
                                     'max': max(cps) if cps else None, 'frames': len(cps)},
                'z_min': round(min(zs), 3) if zs else None,
                'z_max': round(max(zs), 3) if zs else None,
                'ground_peak_z': peak_z,
                'ground_peak_points': peak_cnt,
                'z_hist_top6': hist_top,
                'near_points_r_lt_0.3': sum(1 for r in near if r < 0.3),
                'near_points_r_lt_0.5': len(near),
                'near_min_r': min(near) if near else None,
                'z_min_r_lt_1.5': (None if getattr(self, 'cloud_zmin_r15', 9e9) > 8e9
                                   else round(self.cloud_zmin_r15, 3)),
                'z_min_r_lt_3.0': (None if getattr(self, 'cloud_zmin_r30', 9e9) > 8e9
                                   else round(self.cloud_zmin_r30, 3)),
            },
            'obstacle_points_per_frame': {'median': med(op), 'frames': len(op)},
            'ground_points_per_frame': {'median': med(gp), 'frames': len(gp)},
            'scan': {
                'frames': len(beams),
                'beams_finite_median': med(beams),
                'beams_finite_min': min(beams) if beams else None,
                'beams_finite_max': max(beams) if beams else None,
                'inf_ratio_median': round(med(infr), 4) if infr else None,
                'nearest_ranges': sorted(set(r for r, _ in self.scan_nearest))[:8],
                'last': self.scan_last,
                'band_beams_total': dict(self.scan_bands),
            },
            'odom': {'last': odom_last, 'path_len': round(odom_path, 3)},
            'ground_truth': {'last': gt_last, 'path_len': round(gt_path, 3)},
            'tf': tfs,
            'tf_frames_seen': ['%s->%s' % f for f in tf_frames],
            'drive_seconds': round(drove, 2),
            'yaw_seconds': round(yawed, 2),
            # ★ Phase 3：自击比例（**云点**口径，与 docs §10.6 的 75.5% 同一口径）
            'cloud_r_stats': dict(self.cloud_r_stats),
            'cloud_selfhit_frac': {
                'lt0.05': round(self.cloud_r_stats['lt0.05'] / max(1, self.cloud_r_stats['total']), 4),
                'lt0.12': round(self.cloud_r_stats['lt0.12'] / max(1, self.cloud_r_stats['total']), 4),
                'lt0.2': round(self.cloud_r_stats['lt0.2'] / max(1, self.cloud_r_stats['total']), 4),
            },
            # ★ Phase 3：地面点的距离分布（0.5 m 桶 → 累计点数）——speed_limit_lookahead_m 的判据
            'ground_r_hist': sorted((round(float(k), 1), v) for k, v in self.ground_r_hist.items()),
            # ★ Phase 3：相位漂移（**位移之差**口径）—— 直线段与偏航段分开报
            'phase_drift': {
                'straight': self.phase_drift(ph0, ph1) if ph0 and ph1 else None,
                'yaw': self.phase_drift(ph1, ph2) if ph1 and ph2 else None,
                'total': self.phase_drift(ph0, ph2) if ph0 and ph2 else None,
            },
            'tilt_max_deg': {'odom_roll_pitch': round(getattr(self, 'odom_tilt_max', 0.0), 2),
                             'gt_roll_pitch': round(getattr(self, 'gt_tilt_max', 0.0), 2)},
        }
        if odom_last and gt_last:
            out['drift'] = {
                'delta_xy': [round(odom_last['x'] - gt_last['x'], 3),
                             round(odom_last['y'] - gt_last['y'], 3)],
                'abs_xy': round(math.hypot(odom_last['x'] - gt_last['x'],
                                           odom_last['y'] - gt_last['y']), 3),
                'delta_yaw_deg': round(math.degrees(wrap(odom_last['yaw'] - gt_last['yaw'])), 2),
                'z_odom': round(odom_last['z'], 3), 'z_gt': round(gt_last['z'], 3),
                'path_len_ratio': (round(odom_path / gt_path, 3) if gt_path > 0.05 else None),
            }
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=20.0, help='静止统计窗口（秒）')
    ap.add_argument('--settle-after-drive', type=float, default=3.0)
    ap.add_argument('--ready-timeout', type=float, default=90.0)
    ap.add_argument('--drive-seconds', type=float, default=0.0)
    ap.add_argument('--drive-speed', type=float, default=0.2)
    ap.add_argument('--drive-turn', type=float, default=0.0)
    ap.add_argument('--yaw-seconds', type=float, default=0.0,
                    help='直线段之后再发一段**原地偏航**（Phase 3：分别量直线/偏航的漂移）')
    ap.add_argument('--yaw-rate', type=float, default=0.6, help='原地偏航角速度 (rad/s)')
    ap.add_argument('--require-scan', action='store_true', default=True)
    ap.add_argument('--require-odom', action='store_true', default=True)
    ap.add_argument('--cloud-topic', default='/livox/lidar/pointcloud')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--obstacle-topic', default='/segmentation/obstacle')
    ap.add_argument('--ground-topic', default='/segmentation/ground')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--gt-topic', default='/odom_ground_truth')
    ap.add_argument('--cmd-topic', default='/cmd_vel_chassis')
    ap.add_argument('--out', default='')
    ap.add_argument('--dump-cloud', default='', help='把前 3 帧点云 (x,y,z) 落到这个目录，便于离线看几何')
    ap.add_argument('--dump-scan', default='', help='把前 3 帧 /scan (angle,range) 落到这个目录')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Probe(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        res = node.run()
        print('=== ROBOT_MODEL_PROBE_JSON_BEGIN ===')
        print(json.dumps(res, ensure_ascii=False, indent=1, default=str))
        print('=== ROBOT_MODEL_PROBE_JSON_END ===')
        if args.out:
            with open(args.out, 'w') as f:
                json.dump(res, f, ensure_ascii=False, indent=1, default=str)
    finally:
        # 无论如何都补一次零速（防止探针异常退出把车留在动着的状态）
        try:
            stop = Twist()
            for _ in range(10):
                node.cmd_pub.publish(stop)
                time.sleep(0.05)
        except Exception:                                         # noqa: BLE001
            pass
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
