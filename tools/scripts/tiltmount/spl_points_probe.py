#!/usr/bin/env python3
"""spl_points_probe.py —— 「每帧有多少点进 EKF」的取证探针（**只订阅，不发布**）。

为什么需要它（源码级口径，2026-10-21 复核过）：
  · `small_point_lio`（`src/rm_localization/small_point_lio/src/small_point_lio/preprocess.cpp:26-48`
    与 `small_point_lio.cpp:269-306`）：
      - `point_deque`  = 通过 `i % point_filter_num`、`min_distance/max_distance`、
                         可选 `space_downsample`（体素）之后的点；**每个点触发一次
                         `kf.update_point()`** ⇒ 这就是「进 EKF 的点」。
      - `dense_point_deque` = 时间戳合法的**全部原始点**，只用于拼 `/cloud_registered`。
     ⇒ `/cloud_registered` 的点数（约 11.8k）**不是** EKF 的输入点数，两者的差别正是过滤。
  · `point_lio`（`preprocess.cpp:582-590` + `laserMapping.cpp:450-457`）走同一套算术
    （`i % point_filter_num` → `blind` 球 → 可选 `filter_size_surf` 体素）；它的
    `/cloud_registered`（`feats_down_world`）恰好就是 EKF 输入云，所以那边的发布点数可直读。

本探针量的是**输入侧**：每帧 `/livox/lidar/pointcloud` 的原始点数，以及按上述算式复算出的
「进 EKF 点数」——对 `point_filter_num ∈ {1,2,3,10}` × `leaf ∈ {none,0.1,0.2,0.3,0.5}`
逐组合给出（体素用「每个体素留一个点（质心）」的上游实现 ⇒ 输出点数 = 唯一体素数，
`src/util/voxelgrid_sampling.cpp:93-130`）。`min_distance` / `blind` 取 0.5 m（两份配置都是 0.5）。

窗口口径与 `uc_procstats.py` 完全一致：首帧 `/cloud_registered` 之后 settle 秒（仿真）起，
持续 window 秒（仿真）。输出 JSON；只写一个文件，不碰栈。

用法：
  python3 spl_points_probe.py --out points.json --settle 20 --window 20 --wall-timeout 300
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

DATATYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
            5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
PFNS = (1, 2, 3)
LEAVES = ()   # 体素口径**不在运行期算**（避免探针自己抢 CPU）：用 clouds.npz 离线复算
MIN_DIST = 0.5


def pc2_to_xyz(msg):
    """PointCloud2 → (N,3) float64（只取 x/y/z；与 grav_probe.py 同一实现）。"""
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
        if dt is None:
            return np.zeros((0, 3))
        col = np.frombuffer(np.ascontiguousarray(buf[:, f.offset:f.offset + np.dtype(dt).itemsize]),
                            dtype=dt).astype(np.float64)
        cols.append(col)
    return np.stack(cols, axis=1)


class Points(Node):
    def __init__(self, settle, window):
        super().__init__('spl_points_probe')
        self.settle = settle
        self.window = window
        self.sim = None
        self.t_first_cloud_sim = None
        self.phase = 'wait'
        self.win_start_sim = None
        self.raw_counts = []          # 每帧原始点数
        self.reg_counts = []          # 每帧 /cloud_registered 点数
        self.ekf = {f'pfn{p}': [] for p in PFNS}
        self.ekf_vox = {f'pfn{p}_leaf{leaf}': [] for p in PFNS for leaf in LEAVES}
        self.n_raw_msg = 0
        self.raw_frame_ids = set()
        self.reg_frame_ids = set()
        q = qos_profile_sensor_data
        self.create_subscription(Clock, '/clock', self.on_clock, q)
        self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self.on_raw, q)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_reg, q)

    def on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def on_reg(self, m):
        if self.phase == 'wait':
            if self.t_first_cloud_sim is None:
                self.t_first_cloud_sim = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            return
        if self.phase == 'window':
            self.reg_counts.append(int(m.width * m.height))
            self.reg_frame_ids.add(m.header.frame_id)

    def on_raw(self, m):
        if self.phase != 'window':
            return
        xyz = pc2_to_xyz(m)
        n = xyz.shape[0]
        if n == 0:
            return
        self.n_raw_msg += 1
        self.raw_frame_ids.add(m.header.frame_id)
        self.raw_counts.append(n)
        r = np.linalg.norm(xyz, axis=1)
        keep = (r > MIN_DIST) & (r < 1000.0)
        base = xyz[keep]
        for p in PFNS:
            sel = base[::p]
            self.ekf[f'pfn{p}'].append(int(sel.shape[0]))
            for leaf in LEAVES:
                if sel.shape[0] == 0:
                    self.ekf_vox[f'pfn{p}_leaf{leaf}'].append(0)
                    continue
                keys = np.floor(sel / leaf).astype(np.int64)
                self.ekf_vox[f'pfn{p}_leaf{leaf}'].append(int(np.unique(keys, axis=0).shape[0]))

    def step(self):
        if self.sim is None or self.t_first_cloud_sim is None:
            return False
        if self.phase == 'wait':
            if self.sim - self.t_first_cloud_sim >= self.settle:
                self.phase = 'window'
                self.win_start_sim = self.sim
            return False
        if self.phase == 'window' and self.sim - self.win_start_sim >= self.window:
            self.phase = 'done'
            return True
        return False


def stat(v):
    if not v:
        return None
    a = np.asarray(v, dtype=np.float64)
    return {'n': int(a.size), 'median': float(np.median(a)),
            'min': float(a.min()), 'max': float(a.max()), 'mean': float(a.mean())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--settle', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=20.0)
    ap.add_argument('--wall-timeout', type=float, default=300.0)
    args = ap.parse_args()
    rclpy.init()
    node = Points(args.settle, args.window)
    t0 = time.time()
    done = False
    try:
        while time.time() - t0 < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.step():
                done = True
                break
    finally:
        out = {'ok': done, 'settle_s': args.settle, 'window_s_sim': args.window,
               'phase_at_end': node.phase, 'n_raw_msg': node.n_raw_msg,
               'raw_frame_ids': sorted(node.raw_frame_ids),
               'reg_frame_ids': sorted(node.reg_frame_ids),
               'min_distance_m': MIN_DIST,
               'note': 'EKF 点数 = i%pfn==0 且 0.5<|p|<1000，再按体素去重（每体素 1 点）',
               'raw_points_per_frame': stat(node.raw_counts),
               'cloud_registered_points_per_frame': stat(node.reg_counts),
               'ekf_points_per_frame': {k: stat(v) for k, v in node.ekf.items()},
               'ekf_points_per_frame_voxel': {k: stat(v) for k, v in node.ekf_vox.items()},
               }
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: out[k] for k in ('ok', 'phase_at_end', 'n_raw_msg',
                                          'raw_points_per_frame',
                                          'cloud_registered_points_per_frame',
                                          'ekf_points_per_frame')},
                     indent=1, ensure_ascii=False))
    return 0 if done else 2


if __name__ == '__main__':
    raise SystemExit(main())
