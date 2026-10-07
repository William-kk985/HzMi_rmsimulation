#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""costmap_input_dump.py —— 采一帧「近场归因」需要的全部输入（2026-10-09）

为什么需要：`obstacle_layer.scan` 的高度带是**在代价图帧（odom）里**量的，而
`/scan` 是一张**二维平盘**（`projectLaser` 把 z 强行置 0）⇒ 每条波束在 odom 里的 z
**恒等于**"那一帧 `livox_frame` 原点的 odom z"，与它实际打到的三维点**无关**。
⇒ "地面点为什么能进代价图"这件事**不能**只看 `/scan`；必须同时拿到：

  · `/livox/lidar/pointcloud`（raw，`livox_frame`）—— 每个点的真实三维位置（`range·axis`）
  · `/segmentation/{ground,obstacle}` —— linefit + 判据之后谁被当成障碍

本脚本**只订阅**（不发任何东西），把同一时刻的这几朵云 + TF 落成 CSV/NPZ，
供离线脚本用**与 C++ 同一条公式**的局部地面重算 `dz = z − g(x,y)`。

用法（隔离无头，一次 bash 调用跑完）：
  python3 tools/scripts/tiltmount/costmap_input_dump.py --out-dir .tmp_tiltmount/<tag>/dump \
      --settle 25 --duration 30 --frames 6

输出：`<out-dir>/{meta.json,raw_<i>.csv,ground_<i>.csv,obstacle_<i>.csv,scan_<i>.csv}`
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
import tf2_ros

QOS_BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST)
DATATYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
            5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def pc2_to_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for k in ('x', 'y', 'z'):
        f = fields[k]
        dt = DATATYPE.get(f.datatype)
        cols.append(buf[:, f.offset:f.offset + np.dtype(dt).itemsize].copy()
                    .view(dt).reshape(-1).astype(np.float64))
    return np.stack(cols, axis=1)


def tf_RT(tr):
    t = tr.transform.translation
    q = tr.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z], dtype=np.float64)


class Dumper(Node):
    def __init__(self, args):
        super().__init__('costmap_input_dump')
        # ★ 必须显式声明 use_sim_time（否则 TF 查询用墙钟、TF 数据是仿真钟 ⇒ 全部查不到）
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                       value=bool(args.use_sim_time))])
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        # ⚠️ humble 的 TransformListener 需要**显式传 node**（新版 rclpy 才是单参数）
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.n = {'raw': 0, 'ground': 0, 'obstacle': 0, 'scan': 0}
        self.done = {k: 0 for k in self.n}
        self.meta = {'raw': [], 'ground': [], 'obstacle': [], 'scan': []}
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud',
                                 lambda m: self.on_cloud('raw', m), QOS_BE)
        self.create_subscription(PointCloud2, '/segmentation/ground',
                                 lambda m: self.on_cloud('ground', m), QOS_BE)
        self.create_subscription(PointCloud2, '/segmentation/obstacle',
                                 lambda m: self.on_cloud('obstacle', m), QOS_BE)
        self.create_subscription(LaserScan, '/scan', self.on_scan, QOS_BE)

    def _tf(self, parent, child, stamp):
        """取 parent←child 的 TF。★ 先取最新一帧（`Time()`），失败再按 stamp 查。

        ⚠️ 本仓 TF 树：`map→odom→base_link`（LIO 发）+ `base_link→livox_frame`
        （robot_state_publisher 发）。实测（本沙箱、两次独立复现）：
          · `lookup_transform('odom','livox_frame', <点云 stamp>)` **一定失败** ——
            点云的 `stamp` 比 TF 缓存里最新的一条**新一点**（传感器回调与 TF 发布不同步）
            ⇒ tf2 报 "chain 不连通"；
          · `lookup_transform(<任意对>, Time())`（= 最新）**成功**。
        ⚠️ 另一个必踩的坑：**节点必须先声明 `use_sim_time=true`**。TF 的时间戳是仿真钟
        （本仓 `/clock` 已跑 ~640 s），而 rclpy 默认 `use_sim_time=false` ⇒ 节点自己
        发 `Time()` 请求时用的是**墙钟**（≈0）⇒ tf2 认为"你问的是过去 640 s 之前的位姿"，
        直接失败（这就是上面那次"odom 全查不到"的真因）。
        代价：用最新 TF 而不是"与该帧严格同时刻的 TF" ⇒ 车在动时有一帧的滞后
        （dump 是静止协议，报告里记录 `robot_z_in_odom` 供核对）。
        """
        for use_latest in (True, False):
            try:
                ts = rclpy.time.Time() if use_latest else stamp
                return self.tf_buffer.lookup_transform(
                    parent, child, ts, timeout=rclpy.duration.Duration(seconds=0.3))
            except Exception:
                continue
        return None

    def on_cloud(self, tag, m):
        if self.done[tag] >= self.args.frames:
            return
        i = self.done[tag]
        self.done[tag] += 1
        P = pc2_to_xyz(m)
        stamp = m.header.stamp
        np.savetxt(os.path.join(self.args.out_dir, '%s_%d.csv' % (tag, i)), P,
                   delimiter=',', fmt='%.6f', header='x,y,z', comments='')
        rec = {'i': i, 'n': int(len(P)), 'frame_id': m.header.frame_id,
               'stamp': stamp.sec + stamp.nanosec * 1e-9}
        R_o = t_o = R_b = t_b = None
        tr = self._tf('odom', 'base_link', stamp)
        if tr is not None:
            R_o, t_o = tf_RT(tr)
            rec['T_odom_from_base_link'] = {
                'R': [[round(float(v), 9) for v in row] for row in R_o],
                't': [round(float(v), 6) for v in t_o]}
        tr = self._tf('base_link', m.header.frame_id, stamp)
        if tr is not None:
            R_b, t_b = tf_RT(tr)
            rec['T_base_link_from_%s' % m.header.frame_id] = {
                'R': [[round(float(v), 9) for v in row] for row in R_b],
                't': [round(float(v), 6) for v in t_b]}
        # ★ 合成：odom ← 点云帧 = (odom←base_link)·(base_link←点云帧)
        if R_o is not None and R_b is not None:
            rec['T_odom_from_%s' % m.header.frame_id] = {
                'R': [[round(float(v), 9) for v in row] for row in (R_o @ R_b)],
                't': [round(float(v), 6) for v in (R_o @ t_b + t_o)]}
        self.meta[tag].append(rec)
        print('[dump] %s#%d n=%d frame_id=%s' % (tag, i, len(P), m.header.frame_id), flush=True)

    def on_scan(self, m):
        if self.done['scan'] >= self.args.frames:
            return
        i = self.done['scan']
        self.done['scan'] += 1
        stamp = m.header.stamp
        ang = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        r = np.asarray(m.ranges, dtype=np.float64)
        np.savetxt(os.path.join(self.args.out_dir, 'scan_%d.csv' % i),
                   np.stack([ang, r], axis=1), delimiter=',', fmt='%.6f',
                   header='angle_rad,range', comments='')
        rec = {'i': i, 'frame_id': m.header.frame_id, 'n': int(len(r)),
               'range_min': m.range_min, 'range_max': m.range_max,
               'angle_min': m.angle_min, 'angle_increment': m.angle_increment,
               'stamp': stamp.sec + stamp.nanosec * 1e-9}
        # ★ 关键：把每条波束用 TF 搬到 odom / base_link（二维平盘 z=0 + 变换）
        fin = np.isfinite(r)
        pts = np.stack([r[fin] * np.cos(ang[fin]), r[fin] * np.sin(ang[fin]),
                        np.zeros(int(fin.sum()))], axis=1)
        tr_o = self._tf('odom', 'base_link', stamp)
        tr_b = self._tf('base_link', m.header.frame_id, stamp)
        if tr_b is not None:
            R, t = tf_RT(tr_b)
            Q = (R @ pts.T).T + t
            rec['beam_base_link'] = {
                'z_min': float(Q[:, 2].min()), 'z_p05': float(np.percentile(Q[:, 2], 5)),
                'z_p50': float(np.percentile(Q[:, 2], 50)),
                'z_p95': float(np.percentile(Q[:, 2], 95)),
                'z_max': float(Q[:, 2].max()), 'n': int(fin.sum()),
                'T_base_link_from_%s' % m.header.frame_id: {
                    'R': [[round(float(v), 9) for v in row] for row in R],
                    't': [round(float(v), 6) for v in t]}}
        if tr_b is not None and tr_o is not None:
            Ro, to = tf_RT(tr_o)
            Rb, tb = tf_RT(tr_b)
            R, t = Ro @ Rb, Ro @ tb + to
            Q = (R @ pts.T).T + t
            rec['beam_odom'] = {
                'z_min': float(Q[:, 2].min()), 'z_p05': float(np.percentile(Q[:, 2], 5)),
                'z_p50': float(np.percentile(Q[:, 2], 50)),
                'z_p95': float(np.percentile(Q[:, 2], 95)),
                'z_max': float(Q[:, 2].max()), 'n': int(fin.sum()),
                'robot_z_in_odom': float(to[2]),
                'T_odom_from_%s' % m.header.frame_id: {
                    'R': [[round(float(v), 9) for v in row] for row in R],
                    't': [round(float(v), 6) for v in t]}}
        self.meta['scan'].append(rec)
        print('[dump] scan#%d n_finite=%d' % (i, int(np.isfinite(r).sum())), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--settle', type=float, default=25.0)
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--frames', type=int, default=6)
    ap.add_argument('--use-sim-time', type=int, default=1,
                    help='节点自己的 use_sim_time（默认 1；TF 的时间戳是仿真钟 ⇒ 必须一致）')
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Dumper(args)
    t0 = time.time()
    print('[dump] settle %.0f s …' % args.settle, flush=True)
    while time.time() - t0 < args.settle:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('[dump] 记录窗口 %.0f s …' % args.duration, flush=True)
    t1 = time.time()
    while time.time() - t1 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    # 最后再给 TF 一点时间补齐
    t2 = time.time()
    while time.time() - t2 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.meta['counts'] = node.done
    with open(os.path.join(args.out_dir, 'meta.json'), 'w') as f:
        json.dump(node.meta, f, indent=1, ensure_ascii=False)
    print('[dump] 写出 %s/meta.json counts=%s' % (args.out_dir, node.done), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
