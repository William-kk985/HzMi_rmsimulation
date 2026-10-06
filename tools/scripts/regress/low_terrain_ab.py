#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""low_terrain_ab.py —— 「低矮地形（坡脚/台阶）在不在感知链里」的实测装置。

回答三个缺陷各自的**可引用数字**（口径与 docs/ground_segmentation_slots.md §10 对齐）：

  ① 地面分割槽位（linefit vs patchwork）：同一批**真帧**上，坡面/台阶被删掉多少。
  ② `pointcloud_to_laserscan` 的高度带（max_height 0.1 vs 1.0）：0.4~1.0 m 的几何到底有没有
     进过 `/scan`（同一批帧、同一朵 obstacle 云，只换 z 过滤 ⇒ 干净的 A/B）。
  ③ 新增的「坡度/台阶」判据（`traversability_enable`）：map(−3.6, 3.1) 那个坡脚/台阶有没有
     变成障碍点，以及可行驶坡面有没有被误判。

工作方式（三条子命令）：

  replay  —— **离线 A/B**：`ros2 bag play` 一袋点云 → 起地面分割节点 + **两个** p2l 实例
             （max_height 0.1 / 1.0）→ 本脚本订阅 `/segmentation/{ground,obstacle,step_edge}`、
             `/scan_mh010`、`/scan_mh100`、`~/traversability_stats`，逐帧算指标。
             不需要 Gazebo、不需要真值传感器 ⇒ 同帧可比、可复跑。
  live    —— 在一整套**跑着的栈**上只读采样（供 run_low_terrain_capture.sh 在同一次 bash 调用里用）。
  drive   —— 按航点发 /cmd_vel（供 capture 脚本把车开到坡脚附近）。

真值（"那里的场地几何到底多高"）来自 `RMUC2026.stl`，用 `field_mesh_vs_map.height_grid`
栅格化成 0.05 m 高度图（README 里那套毫米/z 抬升换算），再与**map 系**里的感知点逐帧对账。

输出：`--out` JSON（逐帧 + 汇总）。退出码 0 = 正常。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(WS, 'tools', 'scripts', 'regress'))

SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

# 关注区域（map 系；与 docs/path_clearance_and_contact.md §2.2 同一批数字）
BOXES = {
    'collision': (-3.6, 3.1),     # 用户那次撞击点
    'goal': (-12.64, -0.31),      # 用户那次目标
    'spawn': (0.0, 0.0),          # 出生点（对照：这里不该有假障碍）
    'ramp_top': (-6.0, 0.5),      # 场地中央台面附近（对照：可行驶区）
}
BOX_HALF = 0.6

# 高度带（**离地**，m）：p2l 的 z 过滤在雷达系，离地 = z + 0.226（livox 安装高度）
LIVOX_Z = 0.226
BANDS = [(0.05, 0.10), (0.10, 0.20), (0.20, 0.30), (0.30, 0.40),
         (0.40, 0.70), (0.70, 1.00), (1.00, 1.50)]


# --------------------------------------------------------------------- 点云工具
def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N,3) float32（只取 x/y/z；本链路点类型固定为 PointXYZ）。"""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(off):
        return np.zeros((0, 3), np.float32)
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.zeros((0, 3), np.float32)
    out = np.zeros((n, 3), np.float32)
    for i, ax in enumerate('xyz'):
        out[:, i] = np.ndarray(shape=(n,), dtype=np.float32, buffer=msg.data,
                               offset=off[ax], strides=(msg.point_step,))
    return out


def tf_xyz(T, pts):
    """(4,4) 变换 × (N,3) 点。"""
    if len(pts) == 0:
        return pts.reshape(0, 3)
    return (pts @ T[:3, :3].T) + T[:3, 3]


def mat_of(msg):
    q = msg.transform.rotation
    t = msg.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


# --------------------------------------------------------------------- 真值高度图
def load_truth(world='RMUC2026', cell=0.05):
    from field_mesh_vs_map import read_stl, load_map, height_grid
    stl = os.path.join(WS, 'src', 'rm_simulation', 'hzmi_rm_simulation', 'world',
                       '%s_world' % world, 'meshes', '%s.stl' % world)
    map_yaml = os.path.join(WS, 'src', 'rm_nav_bringup', 'map', '%s.yaml' % world)
    V = read_stl(stl)
    m, img, occ, res, ox, oy = load_map(map_yaml)
    rel = height_grid(V, occ.shape, res, ox, oy, 10.925, 2.524, 0.001, 1.6413436, 0.0, cell=cell)
    return {'rel': rel, 'occ': occ, 'res': res, 'ox': ox, 'oy': oy,
            'shape': occ.shape, 'stl': stl, 'map': map_yaml}


def truth_cells(truth, cx, cy, half, min_h):
    """窗口内真值高度 > min_h 的格中心（map 系）。"""
    rel, res, ox, oy, (H, W) = (truth['rel'], truth['res'], truth['ox'], truth['oy'],
                                truth['shape'])
    c0 = max(0, int((cx - half - ox) / res)); c1 = min(W, int((cx + half - ox) / res) + 1)
    r0 = max(0, H - 1 - int((cy + half - oy) / res)); r1 = min(H, H - int((cy - half - oy) / res))
    sub = rel[r0:r1, c0:c1]
    rr, cc = np.where(sub > min_h)
    if len(rr) == 0:
        return np.zeros((0, 2))
    xs = ox + (c0 + cc + 0.5) * res
    ys = oy + (H - 1 - (r0 + rr) + 0.5) * res
    return np.stack([xs, ys], axis=1)


def nearest_dist(pts_xy, ref_xy):
    if len(pts_xy) == 0 or len(ref_xy) == 0:
        return None
    d = np.sqrt(((pts_xy[:, None, :] - ref_xy[None, :, :]) ** 2).sum(-1))
    return float(d.min())


# --------------------------------------------------------------------- 采样器
class Sampler(Node):
    def __init__(self, tag, tf=True, scan_topics=('/scan',), pointcloud_topic='/livox/lidar/pointcloud'):
        super().__init__('low_terrain_ab_%s' % tag)
        self.tag = tag
        self.rows = []
        self.truth = None
        self.step_ref = {}
        self.tf_buffer = Buffer() if tf else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if tf else None
        self.cloud = None
        self.cloud_frame = ''
        self.cloud_stamp = None
        self.seg = {}       # topic → (xyz, stamp)
        self.scans = {}     # topic → (ranges, angle_min, angle_inc, stamp)
        self.stats = []     # traversability_stats 解析结果
        self.timing = []
        self.create_subscription(PointCloud2, pointcloud_topic, self.on_cloud, SENSOR)
        for t in ('/segmentation/ground', '/segmentation/obstacle', '/segmentation/step_edge'):
            self.create_subscription(PointCloud2, t, lambda m, t=t: self.on_seg(t, m), SENSOR)
        for t in scan_topics:
            self.create_subscription(LaserScan, t, lambda m, t=t: self.on_scan(t, m), SENSOR)
        self.create_subscription(String, '/ground_segmentation/traversability_stats',
                                 self.on_stats, 10)
        self.create_subscription(Float64, '/ground_segmentation/segmentation_time_ms',
                                 self.on_time, 10)
        self.create_timer(0.2, self.tick)     # 5 Hz 采样（点云 8-10 Hz ⇒ 覆盖大部分帧）

    # ---- 回调
    def on_cloud(self, msg):
        self.cloud = pc2_to_xyz(msg)
        self.cloud_frame = msg.header.frame_id
        self.cloud_stamp = msg.header.stamp

    def on_seg(self, topic, msg):
        self.seg[topic] = (pc2_to_xyz(msg), msg.header.stamp)

    def on_scan(self, topic, msg):
        self.scans[topic] = (np.asarray(msg.ranges, np.float32), msg.angle_min, msg.angle_increment)

    def on_stats(self, msg):
        try:
            self.stats.append(json.loads(msg.data))
        except Exception:
            pass

    def on_time(self, msg):
        self.timing.append(float(msg.data))

    # ---- 采样一拍
    def tick(self):
        if self.cloud is None or '/segmentation/obstacle' not in self.seg:
            return
        T = None
        if self.tf_buffer is not None:
            try:
                tr = self.tf_buffer.lookup_transform('map', self.cloud_frame or 'livox_frame',
                                                     rclpy.time.Time())
                T = mat_of(tr)
            except Exception:
                T = None
        row = {'t': time.time(), 'have_tf': T is not None}
        if T is not None:
            row['pose_x'] = float(T[0, 3])
            row['pose_y'] = float(T[1, 3])
        cloud = self.cloud
        row['n_input'] = int(len(cloud))
        for name, key in (('ground', '/segmentation/ground'),
                          ('obstacle', '/segmentation/obstacle'),
                          ('step_edge', '/segmentation/step_edge')):
            arr = self.seg.get(key, (np.zeros((0, 3), np.float32), None))[0]
            row['n_' + name] = int(len(arr))
            if T is not None and len(arr):
                m = tf_xyz(T, arr)
                for bname, (bx, by) in BOXES.items():
                    inb = ((np.abs(m[:, 0] - bx) <= BOX_HALF) &
                           (np.abs(m[:, 1] - by) <= BOX_HALF))
                    row['%s_%s' % (name, bname)] = int(inb.sum())
                    if bname == 'collision' and inb.any():
                        row['%s_collision_zmax' % name] = float(m[inb, 2].max())
                        row['%s_collision_zp95' % name] = float(np.percentile(m[inb, 2], 95))
                # 与"真值台阶格"的距离（map 系 xy）——台阶=真值高度 > 0.12 m 的格
                if 'collision' in self.step_ref and len(self.step_ref['collision']):
                    row['d_step_%s' % name] = nearest_dist(
                        m[:, :2], self.step_ref['collision'])
            else:
                for bname in BOXES:
                    row['%s_%s' % (name, bname)] = 0
        # ★ 缺陷 ② 的直接量：max_height 0.1 → 1.0 **多出来**多少条回波
        #   （旧带下该方位是 inf ⇒ 下游 inf_is_valid 会把它当"10 m 处有回波"沿射线清成 free）
        r010 = self.scans.get('/scan_mh010')
        r100 = self.scans.get('/scan_mh100')
        if r010 is not None and r100 is not None:
            a, b = np.isfinite(r010[0]), np.isfinite(r100[0])
            row['scan_added_by_mh100'] = int((b & ~a).sum())
            row['scan_lost_by_mh100'] = int((a & ~b).sum())
            n = min(len(r010[0]), len(r100[0]))
            if n:
                both = a[:n] & b[:n]
                row['scan_range_delta_p50'] = (float(np.percentile(
                    r010[0][:n][both] - r100[0][:n][both], 50)) if both.any() else 0.0)
        for topic, (rng, amin, ainc) in self.scans.items():
            fin = np.isfinite(rng)
            key = 'scan' if topic == '/scan' else topic.strip('/').replace('/', '_')
            row['%s_finite' % key] = int(fin.sum())
            row['%s_min' % key] = float(rng[fin].min()) if fin.any() else None
            row['%s_lt1' % key] = int((rng[fin] < 1.0).sum())
            if T is not None and fin.any():
                ang = amin + ainc * np.arange(len(rng))
                pts = np.stack([rng[fin] * np.cos(ang[fin]), rng[fin] * np.sin(ang[fin]),
                                np.zeros(fin.sum())], 1)
                m = tf_xyz(T, pts)
                for bname, (bx, by) in BOXES.items():
                    row['%s_%s' % (key, bname)] = int(((np.abs(m[:, 0] - bx) <= BOX_HALF) &
                                                       (np.abs(m[:, 1] - by) <= BOX_HALF)).sum())
                if 'collision' in self.step_ref and len(self.step_ref['collision']):
                    row['d_step_%s' % key] = nearest_dist(m[:, :2], self.step_ref['collision'])
        # p2l 高度带的**解析**覆盖：在 obstacle 云上按"离地高度带"统计方位 bin 覆盖
        arr = self.seg.get('/segmentation/obstacle', (np.zeros((0, 3), np.float32), None))[0]
        row['bands'] = band_coverage(arr)
        self.rows.append(row)

    def set_truth(self, truth):
        self.truth = truth
        for bname, (bx, by) in BOXES.items():
            self.step_ref[bname] = truth_cells(truth, bx, by, BOX_HALF, 0.12)


def band_coverage(cloud):
    """按"离地高度带"算方位 bin 覆盖（p2l 口径）。

    p2l 的 z 过滤在**雷达系**：保留 min_height < z < max_height。离地高度 h = z + 0.226
    ⇒ 高度带 [h0,h1) 的几何"能不能进 /scan" ⟺ z ∈ (h0-0.226, h1-0.226) 与过滤带相交。
    这里返回每个高度带的 (bins_total, bins_covered_by_obstacle, bins_passing_mh010,
    bins_passing_mh100)：后两个 = "若 max_height 取该值，这个高度带还有几个方位 bin 有回波"。
    """
    out = {}
    if len(cloud) == 0:
        return out
    h = cloud[:, 2] + LIVOX_Z                     # 离地高度（雷达系 z + 安装高）
    az = np.round(np.arctan2(cloud[:, 1], cloud[:, 0]) / 0.0043).astype(np.int64)
    for (h0, h1) in BANDS:
        sel = (h >= h0) & (h < h1)
        key = '%.2f_%.2f' % (h0, h1)
        if not sel.any():
            out[key] = {'pts': 0, 'bins': 0, 'bins_mh010': 0, 'bins_mh100': 0}
            continue
        b = np.unique(az[sel])
        z = cloud[sel, 2]
        p010 = np.unique(az[sel & (cloud[:, 2] > -1.0) & (cloud[:, 2] < 0.1)])
        p100 = np.unique(az[sel & (cloud[:, 2] > -1.0) & (cloud[:, 2] < 1.0)])
        out[key] = {'pts': int(sel.sum()), 'bins': int(len(b)),
                    'bins_mh010': int(len(p010)), 'bins_mh100': int(len(p100))}
    return out


def summarize(rows, extra=None):
    s = {'frames': len(rows)}
    if not rows:
        return s

    def stats(key):
        v = np.array([r[key] for r in rows if isinstance(r.get(key), (int, float))], float)
        if v.size == 0:
            return None
        return {'p50': float(np.percentile(v, 50)), 'p95': float(np.percentile(v, 95)),
                'min': float(v.min()), 'max': float(v.max()), 'mean': float(v.mean())}

    for k in ('n_input', 'n_ground', 'n_obstacle', 'n_step_edge',
              'obstacle_collision', 'step_edge_collision', 'ground_collision',
              'obstacle_spawn', 'step_edge_spawn', 'obstacle_goal',
              'obstacle_collision_zmax', 'step_edge_collision_zmax',
              'd_step_obstacle', 'd_step_step_edge', 'scan_min', 'scan_finite',
              'scan_mh010_finite', 'scan_mh100_finite', 'scan_mh010_collision',
              'scan_mh100_collision', 'd_step_scan', 'd_step_scan_mh010',
              'd_step_scan_mh100'):
        st = stats(k)
        if st is not None:
            s[k] = st
    # 高度带的方位覆盖（逐帧求和后再比）
    band_acc = defaultdict(lambda: defaultdict(int))
    for r in rows:
        for band, d in (r.get('bands') or {}).items():
            for k2, v2 in d.items():
                band_acc[band][k2] += v2
    s['bands_total'] = {b: dict(d) for b, d in sorted(band_acc.items())}
    if extra:
        s.update(extra)
    return s


# --------------------------------------------------------------------- replay
def run_replay(args):
    passes = {
        'lf_off': ('linefit_ground_segmentation_ros', 'ground_segmentation_node', False),
        'lf_on': ('linefit_ground_segmentation_ros', 'ground_segmentation_node', True),
        'pw_off': ('patchwork_ground_segmentation', 'patchwork_ground_segmentation_node', False),
        'pw_on': ('patchwork_ground_segmentation', 'patchwork_ground_segmentation_node', True),
    }
    want = args.passes.split(',')
    report = {}
    for name in want:
        pkg, exe, trav = passes[name]
        rep = run_one_pass(args, name, pkg, exe, trav)
        report[name] = rep
    with open(args.out, 'w') as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print('[ab] → %s' % args.out)
    for name, rep in report.items():
        s = rep.get('summary', {})
        print('[ab] %-7s frames=%-4s step_edge_collision(p50/max)=%s/%s  d_step(step_edge)=%s '
              'obstacle_spawn(max)=%s' % (
                  name, s.get('frames'),
                  (s.get('step_edge_collision') or {}).get('p50'),
                  (s.get('step_edge_collision') or {}).get('max'),
                  (s.get('d_step_step_edge') or {}).get('min'),
                  (s.get('obstacle_spawn') or {}).get('max')))
    return 0


def run_one_pass(args, name, pkg, exe, trav):
    print('[ab] ===== pass %s（%s, traversability=%s）=====' % (name, exe, trav))
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(args.domain)
    procs = []

    def spawn(cmd):
        # start_new_session ⇒ 单独进程组：收尾时整组杀，绝不留孤儿节点（否则下一轮 A/B 会
        # 出现**两个** /segmentation/* 发布者，采到混合数据 ⇒ 白跑）。
        p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                             start_new_session=True)
        procs.append(p)
        return p

    crit = os.path.join(WS, 'src', 'rm_nav_bringup', 'config', 'traversability_criteria.yaml')
    base = os.path.join(WS, 'install', pkg, 'share', pkg, 'config')
    if pkg == 'linefit_ground_segmentation_ros':
        params = os.path.join(base, 'segmentation_sim.yaml')
    else:
        params = os.path.join(base, 'ground_segmentation_sim.yaml')
    p2l_params = os.path.join(WS, 'install', 'pointcloud_to_laserscan', 'share',
                              'pointcloud_to_laserscan', 'config', 'laserscan_params.yaml')
    try:
        if args.live:
            # live：只读采样跑着的栈 —— **绝不**另起 ground/p2l（那会出现两个
            # /segmentation/* 发布者，直接违反单发布者契约）。
            return sample_seconds(args, name, scan_topics=('/scan',))
        spawn(['ros2', 'run', pkg, exe, '--ros-args',
               '--params-file', params, '--params-file', crit,
               '-p', 'use_sim_time:=false',
               '-p', 'traversability_enable:=%s' % ('true' if trav else 'false')])
        for tag, mh in (('mh010', 0.1), ('mh100', 1.0)):
            spawn(['ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
                   '--ros-args', '--params-file', p2l_params,
                   '-r', '__node:=p2l_%s' % tag,
                   '-r', 'cloud_in:=/segmentation/obstacle',
                   '-r', 'scan:=/scan_%s' % tag,
                   '-p', 'use_sim_time:=false', '-p', 'max_height:=%s' % mh])
        time.sleep(float(args.start_delay))
        # ⚠ 只播点云 + TF：包里还录着 /scan 与 /segmentation/*，若一起播就会与本进程起的
        #   节点**抢同一个人话题**（两个发布者）⇒ 采样到混合数据、A/B 失效。
        play = spawn(['ros2', 'bag', 'play', args.bag, '--rate', str(args.rate),
                      '--topics', '/livox/lidar/pointcloud', '/tf', '/tf_static'])
        rep = sample_seconds(args, name, scan_topics=('/scan_mh010', '/scan_mh100'),
                             stop_when=play)
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
        time.sleep(1.0)
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1.0)
    return rep


def sample_seconds(args, name, scan_topics, stop_when=None):
    truth = load_truth(args.world)
    node = Sampler(name, tf=not args.no_tf, scan_topics=scan_topics)
    node.set_truth(truth)
    t0 = time.time()
    while time.time() - t0 < float(args.seconds):
        rclpy.spin_once(node, timeout_sec=0.1)
        if stop_when is not None and stop_when.poll() is not None:
            break
    rows = node.rows
    truth_info = {'collision_step_cells_gt_0.12m':
                  int(len(node.step_ref.get('collision', []))),
                  'spawn_step_cells_gt_0.12m': int(len(node.step_ref.get('spawn', [])))}
    node.destroy_node()
    return {'summary': summarize(rows, {'truth': truth_info,
                                        'stats_frames': len(node.stats),
                                        'classify_ms': _num_stats(node.stats, 'classify_ms'),
                                        'segmentation_ms': _num_stats(
                                            [{'v': v} for v in node.timing], 'v')}),
            'rows': rows}


def _num_stats(stats, key):
    v = np.array([s[key] for s in stats if isinstance(s.get(key), (int, float))], float)
    if v.size == 0:
        return None
    return {'n': int(v.size), 'p50': float(np.percentile(v, 50)),
            'p95': float(np.percentile(v, 95)), 'max': float(v.max())}


# --------------------------------------------------------------------- drive
WAYPOINTS = [
    (-1.0, 0.6, 0.35),      # x, y, 到达容差
    (-2.6, 2.2, 0.35),
    (-3.3, 2.9, 0.30),      # 从东北方向接近坡脚
    (-3.6, 3.1, 0.25),      # 用户那次撞击点
]


def run_drive(args):
    node = Node('low_terrain_drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    buf = Buffer()
    TransformListener(buf, node)
    wps = WAYPOINTS if not args.waypoints else [
        tuple(float(v) for v in w.split(',')) for w in args.waypoints.split(';')]
    t_end = time.time() + float(args.drive_seconds)
    i = 0
    print('[drive] 航点 %s' % (wps,))
    try:
        while rclpy.ok() and time.time() < t_end and i < len(wps):
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = tr.transform.translation.x
                y = tr.transform.translation.y
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            except Exception:
                pub.publish(Twist())
                continue
            gx, gy, tol = wps[i]
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < tol:
                print('[drive] 到达航点 %d (%.2f, %.2f) 实际 (%.2f, %.2f)' % (i, gx, gy, x, y))
                i += 1
                continue
            yaw_err = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                                 math.cos(math.atan2(dy, dx) - yaw))
            cmd = Twist()
            cmd.angular.z = max(-0.8, min(0.8, 1.6 * yaw_err))
            cmd.linear.x = 0.0 if abs(yaw_err) > 0.6 else min(args.speed, 0.7 * dist)
            pub.publish(cmd)
        # 停车 + 原地慢转一圈（把坡脚扫全）
        for _ in range(int(args.spin_seconds / 0.1)):
            cmd = Twist()
            cmd.angular.z = args.spin_rate
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(20):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.1)
        print('[drive] 完成：到达航点 %d/%d' % (i, len(wps)))
    finally:
        node.destroy_node()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('replay', 'live', 'drive'):
        p = sub.add_parser(name)
        p.add_argument('--world', default='RMUC2026')
        p.add_argument('--out', default=None)
        p.add_argument('--domain', type=int, default=161)
        p.add_argument('--seconds', type=float, default=60.0)
        p.add_argument('--rate', type=float, default=1.0)
        p.add_argument('--start-delay', type=float, default=4.0)
        p.add_argument('--no-tf', action='store_true')
        p.add_argument('--bag', default=None)
        p.add_argument('--passes', default='lf_off,pw_off,lf_on,pw_on')
        p.add_argument('--live', action='store_true')
        p.add_argument('--speed', type=float, default=0.30)
        p.add_argument('--drive-seconds', type=float, default=120.0)
        p.add_argument('--spin-seconds', type=float, default=12.0)
        p.add_argument('--spin-rate', type=float, default=0.6)
        p.add_argument('--waypoints', default='')
    a = ap.parse_args()
    a.live = (a.cmd == 'live')      # 子命令名就是模式名（--live 只是给 replay 用的旧开关）
    rclpy.init()
    try:
        if a.cmd == 'drive':
            return run_drive(a)
        if a.cmd == 'live':
            rep = {'live': run_one_pass(a, 'live', 'linefit_ground_segmentation_ros',
                                        'ground_segmentation_node', True)}
        else:
            if not a.bag:
                raise SystemExit('replay 需要 --bag')
            return run_replay(a)
    finally:
        rclpy.shutdown()
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
        print('[ab] → %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
