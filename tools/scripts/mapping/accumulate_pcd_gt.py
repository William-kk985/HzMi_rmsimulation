#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""accumulate_pcd_gt.py —— 从**建图 bag** 离线累积 3D 先验（PCD），位姿可来自三种源。

为什么需要它（2026-10-29，measurements §45）

`cloud_accumulator` 是**在线**节点：它逐帧查 `map←livox_frame` 的 TF 再累积。
建图跑的 `map→odom` 一旦坏掉（§43/§45），累积出来的 PCD 就会把同一段围栏按不同
时刻的位姿叠成**多层鬼影**——这不是累积器的 bug，是"输入位姿坏"的后果。
本脚本把同一件事搬到离线，并且**允许把位姿换成另一套**，于是：

  · `--pose-source gt`    ：用 `/odom_ground_truth`（仿真真值）+ 出生点定义 map 帧
                            ⇒ **干净上界**（GT 累积图，`PCD/<world>_gt.pcd`）；
  · `--pose-source trace` ：用一次"重放跑出来的 `map→odom` 记录"（jsonl）
                            ⇒ **可复现的 SLAM 图**（`PCD/<world>.pcd`）；
  · `--pose-source tf`    ：用 bag 里录下的 `map→odom`
                            ⇒ 复现历史资产（验收/回归对照用）。

清洗口径与在线累积器**逐条对齐**（同一份阈值、同一套计数语义）：
  ① 距离闸 `--min-range`（默认 0.30 m，与 `cloud_accumulator` 的默认一致）；
  ② 高度带 `[z_band_min, z_band_max]`（默认 −0.5 / +1.8 m），**相对本帧 base_link 在 map 系的高度**
     （`z_ref_frame=base_link` 的语义；本场地 base z≈0，故与绝对口径同值）；
  ③ 帧级运动闸 `--max-frame-step/--max-speed/--max-frame-yaw-deg/--max-yaw-rate-dps`
     （默认 0.50 m / 1.50 m/s / 25° / 90°/s，与在线节点同值）；
  ④ 体素：0.10 m 质心合并（与在线节点同一口径）。

输出（全部路径都会打印）：
  · `--out`     8 字段 binary PCD（与 `PCD/RMUC2026.pcd` / `PCD/RMUC2027.pcd` 同构）；
  · `--poses-out` 逐帧 `T_map_base`（jsonl）—— 建 SC DB / 复算残差用的**同一套位姿**；
  · `--json`    体检数字（点数/bbox/地面 z/各闸计数/位姿源）；
  · `--meta`    可选：用 `rm_utils/map_asset_guard.py` 的 `write_manifest` 写 sidecar。

用法（仓库根，source 过 ROS）：
  python3 tools/scripts/mapping/accumulate_pcd_gt.py \
      --bag third_party/eval_data/capture_map2027/bag/raw --pose-source gt \
      --spawn 5.90 2.72 0.132 0.0 \
      --out src/rm_nav_bringup/PCD/RMUC2027_gt.pcd \
      --poses-out /tmp/rmuc2027_gt_poses.jsonl
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))


def _repo_root():
    cur = os.path.abspath(os.getcwd())
    while True:
        if os.path.isfile(os.path.join(cur, 'third_party', 'eval', 'extract_frames.py')):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return os.path.abspath(os.getcwd())
        cur = parent


def _load_pcd_stats():
    path = os.path.join(_HERE, 'pcd_stats.py')
    spec = importlib.util.spec_from_file_location('pcd_stats_mod', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ------------------------------------------------------------------ 四元数/矩阵
def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(t, q):
    m = np.eye(4)
    m[:3, :3] = quat_to_R(q)
    m[:3, 3] = (float(t[0]), float(t[1]), float(t[2]))
    return m


def yaw_of(m):
    return math.atan2(float(m[1, 0]), float(m[0, 0]))


def apply_tf(pts, m):
    return pts @ m[:3, :3].T + m[:3, 3]


def voxel_centroid(pts, leaf):
    """0.10 m 体素质心合并（与 cloud_accumulator 的 _merge_voxels 同口径）。"""
    if len(pts) == 0:
        return np.zeros((0, 3))
    keys = np.floor(pts / leaf).astype(np.int64)
    _, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    out = np.zeros((len(cnt), 3), dtype=np.float64)
    for d in range(3):
        out[:, d] = np.bincount(inv, weights=pts[:, d], minlength=len(cnt))
    return out / cnt[:, None]


# ------------------------------------------------------------------ bag 读取
def read_bag(path):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    from tf2_msgs.msg import TFMessage

    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id='sqlite3'), ConverterOptions('', ''))
    tf, tf_static, gt, clouds = [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in ('/tf', '/tf_static'):
            m = deserialize_message(data, TFMessage)
            for tr in m.transforms:
                row = (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                       tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'),
                       (tr.transform.translation.x, tr.transform.translation.y,
                        tr.transform.translation.z),
                       (tr.transform.rotation.x, tr.transform.rotation.y,
                        tr.transform.rotation.z, tr.transform.rotation.w))
                (tf_static if topic == '/tf_static' else tf).append(row)
        elif topic == '/odom_ground_truth':
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                       (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        elif topic == '/cloud_registered':
            m = deserialize_message(data, PointCloud2)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            pts = np.asarray(pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True),
                             dtype=np.float64)
            if pts.ndim != 2:
                pts = pts.reshape(-1, 3)
            clouds.append((st, m.header.frame_id.lstrip('/'), pts))
    return tf, tf_static, gt, clouds


class EdgeTable:
    """单个 TF 边的时间序列（按戳最近邻查询，超时返回 None）。"""

    def __init__(self, rows, parent, child):
        sel = sorted([(r[0], T_of(r[3], r[4])) for r in rows
                      if r[1] == parent and r[2] == child], key=lambda x: x[0])
        self.stamps = np.array([s for s, _ in sel], dtype=np.float64)
        self.mats = [m for _, m in sel]

    def __len__(self):
        return len(self.stamps)

    def at(self, t, tol=0.25):
        if len(self) == 0:
            return None
        i = int(np.argmin(np.abs(self.stamps - t)))
        if abs(self.stamps[i] - t) > tol:
            return None
        return self.mats[i]


def main(argv=None):
    ap = argparse.ArgumentParser(description='离线累积 3D 先验（GT / SLAM / 历史 TF 三种位姿源）')
    ap.add_argument('--bag', required=True)
    ap.add_argument('--pose-source', choices=['gt', 'trace', 'tf'], default='gt')
    ap.add_argument('--spawn', nargs=4, type=float, metavar=('X', 'Y', 'Z', 'YAW'),
                    default=[5.90, 2.72, 0.132, 0.0],
                    help='出生点的 world 位姿（map 帧 = 出生点相对系；Z = 出生时 base_link 的 world z）')
    ap.add_argument('--trace', default='', help='--pose-source trace 的 jsonl（map→odom 记录）')
    ap.add_argument('--trace-frame', choices=['odom', 'odom_level'], default='odom',
                    help='trace 里 map→X 的 X（odom_level = 重力对齐系；离线诊断用）')
    ap.add_argument('--world', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--poses-out', default='')
    ap.add_argument('--json', default='')
    ap.add_argument('--meta', default='', help='写 sidecar 的基名（如 .../PCD/RMUC2027）')
    ap.add_argument('--voxel', type=float, default=0.10)
    ap.add_argument('--min-range', type=float, default=0.30)
    ap.add_argument('--max-range', type=float, default=0.0)
    ap.add_argument('--z-band', nargs=2, type=float, default=[-0.5, 1.8])
    ap.add_argument('--max-frame-step', type=float, default=0.50)
    ap.add_argument('--max-speed', type=float, default=1.50)
    ap.add_argument('--max-frame-yaw-deg', type=float, default=25.0)
    ap.add_argument('--max-yaw-rate-dps', type=float, default=90.0)
    ap.add_argument('--jump-gate-dt-max', type=float, default=0.50)
    ap.add_argument('--anchor-first', action='store_true',
                    help='把 map 原点锚到**第一帧的 base_link 位姿**（identity）：'
                         'map 系 = 出生点相对系的严格实现；SLAM 图用它去掉 LIO 起点的常值偏置')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--cloud-frame', default='odom')
    ap.add_argument('--level-frame', default='odom_level')
    args = ap.parse_args(argv)

    pcd_stats = _load_pcd_stats()
    print('[accum] bag=%s pose_source=%s' % (args.bag, args.pose_source))
    tf, tf_static, gt, clouds = read_bag(args.bag)
    print('[accum] tf=%d tf_static=%d gt=%d clouds=%d' % (len(tf), len(tf_static), len(gt), len(clouds)))
    if not clouds:
        raise SystemExit('[accum] bag 里没有 /cloud_registered')

    # 静态边：base_link→livox_frame、odom→odom_level
    stat = {(r[1], r[2]): T_of(r[3], r[4]) for r in tf_static}
    T_bl = stat.get((args.base_frame, 'livox_frame'), np.eye(4))
    T_oy = stat.get(('odom', args.level_frame), np.eye(4))
    print('[accum] T_odom_level=%s  T_base_livox t=%s' %
          (np.round(T_oy[:3, 3], 4).tolist(), np.round(T_bl[:3, 3], 4).tolist()))

    e_ol = EdgeTable(tf, args.level_frame, args.base_frame)      # odom_level→base_link
    e_map_odom = EdgeTable(tf, 'map', 'odom') if args.pose_source == 'tf' else None
    trace = []
    if args.pose_source == 'trace':
        if not args.trace:
            raise SystemExit('[accum] --pose-source trace 需要 --trace')
        for line in open(args.trace):
            d = json.loads(line)
            if d.get('child') not in (None, args.trace_frame):
                continue
            m = np.eye(4)
            c, s = math.cos(math.radians(d['yaw'])), math.sin(math.radians(d['yaw']))
            m[:2, :2] = [[c, -s], [s, c]]
            m[:3, 3] = (d['x'], d['y'], d.get('z', 0.0))
            trace.append((d['t'], m))
        trace.sort(key=lambda x: x[0])
        print('[accum] trace: %d samples map→odom' % len(trace))

    # map←world（出生点定义）
    sx, sy, sz, syaw = args.spawn
    R = np.array([[math.cos(-syaw), -math.sin(-syaw), 0.0],
                  [math.sin(-syaw), math.cos(-syaw), 0.0], [0.0, 0.0, 1.0]])
    T_map_world = np.eye(4)
    T_map_world[:3, :3] = R
    T_map_world[:3, 3] = -R @ np.array([sx, sy, sz])
    print('[accum] T_map_world t=%s yaw=%.2f°' %
          (np.round(T_map_world[:3, 3], 3).tolist(), math.degrees(-syaw)))

    gt_sorted = sorted(gt, key=lambda x: x[0])
    gt_stamps = np.array([g[0] for g in gt_sorted])

    def gt_at(t, tol=0.25):
        if len(gt_sorted) == 0:
            return None
        i = int(np.argmin(np.abs(gt_stamps - t)))
        if abs(gt_stamps[i] - t) > tol:
            return None
        return T_of(gt_sorted[i][1], gt_sorted[i][2])

    def T_map_odom(t):
        if args.pose_source == 'tf':
            return e_map_odom.at(t)
        if args.pose_source == 'trace':
            ts = np.array([s for s, _ in trace])
            i = int(np.argmin(np.abs(ts - t)))
            if abs(ts[i] - t) > 0.10:
                return None
            return trace[i][1]
        # gt
        Tg = gt_at(t)
        Tol = e_ol.at(t)
        if Tg is None or Tol is None:
            return None
        T_odom_base = T_oy @ Tol
        return T_map_world @ Tg @ np.linalg.inv(T_odom_base)

    # ---- 预扫：第一帧有效位姿（--anchor-first 的锚）
    T_anchor = np.eye(4)
    if args.anchor_first:
        for st, frame_id, pts in clouds:
            Tmo0 = T_map_odom(st)
            Tol0 = e_ol.at(st)
            if Tmo0 is not None and Tol0 is not None:
                Tb = Tmo0 @ (T_oy @ Tol0)
                yaw_b = yaw_of(Tb)
                c, s_ = math.cos(-yaw_b), math.sin(-yaw_b)
                T_anchor = np.eye(4)
                T_anchor[:2, :2] = [[c, -s_], [s_, c]]
                T_anchor[:2, 3] = -T_anchor[:2, :2] @ Tb[:2, 3]
                print('[accum] anchor-first: 第一帧 st=%.3f 的 map→base 的水平位姿 (%.4f, %.4f, %.3f°) 锚为 0'
                      % (st, float(Tb[0, 3]), float(Tb[1, 3]), math.degrees(yaw_b)))
                break
    n_tf = 0
    frames_ok = 0
    dropped = {'range': 0, 'height': 0, 'jump': 0, 'tf': 0, 'empty': 0}
    pts_all = []
    poses = []
    prev = None
    for st, frame_id, pts in clouds:
        if len(pts) == 0:
            dropped['empty'] += 1
            continue
        Tmo = T_map_odom(st)
        Tol = e_ol.at(st)
        if Tmo is None or Tol is None:
            dropped['tf'] += 1
            n_tf += 1
            continue
        Tmo = T_anchor @ Tmo
        T_mo = Tmo @ np.linalg.inv(T_oy) if args.trace_frame == 'odom_level' else Tmo
        T_map_base = T_mo @ T_oy @ Tol
        # 运动闸
        if prev is not None:
            dt = st - prev[0]
            step = float(np.linalg.norm(T_map_base[:3, 3] - prev[1][:3, 3]))
            dyaw = abs(math.degrees((yaw_of(T_map_base) - yaw_of(prev[1]) + math.pi)
                                    % (2 * math.pi) - math.pi))
            bad = False
            if dt <= args.jump_gate_dt_max:
                if args.max_frame_step > 0 and step > args.max_frame_step:
                    bad = True
                if dyaw > args.max_frame_yaw_deg:
                    bad = True
            if dt > 1e-6:
                if args.max_speed > 0 and step / dt > args.max_speed:
                    bad = True
                if dyaw / dt > args.max_yaw_rate_dps:
                    bad = True
            if bad:
                dropped['jump'] += 1
                prev = (st, T_map_base)
                continue
        prev = (st, T_map_base)
        # 点云 → map：map←cloud = map←odom ∘ odom←base ∘ base←cloud
        if frame_id == args.cloud_frame:
            T_map_cloud = T_mo
        elif frame_id == 'livox_frame':
            T_map_cloud = T_mo @ (T_oy @ Tol) @ T_bl
        else:
            raise SystemExit('[accum] 不支持的点云 frame_id=%r（支持 %r / livox_frame）'
                             % (frame_id, args.cloud_frame))
        p = pts
        r = np.linalg.norm(p, axis=1)
        keep = r >= args.min_range
        if args.max_range > 0:
            keep &= r <= args.max_range
        dropped['range'] += int((~keep).sum())
        p = p[keep]
        if len(p) == 0:
            dropped['empty'] += 1
            continue
        pm = apply_tf(p, T_map_cloud)
        z_ref = float(T_map_base[2, 3])
        keep = (pm[:, 2] - z_ref >= args.z_band[0]) & (pm[:, 2] - z_ref <= args.z_band[1])
        dropped['height'] += int((~keep).sum())
        pm = pm[keep]
        if len(pm) == 0:
            dropped['empty'] += 1
            continue
        pts_all.append(pm)
        frames_ok += 1
        poses.append({'stamp': float(st), 'frame': frame_id,
                      'T_map_base': [float(v) for v in T_map_base.reshape(-1)],
                      'T_map_cloud': [float(v) for v in T_map_cloud.reshape(-1)]})

    if not pts_all:
        raise SystemExit('[accum] 没有可累积的帧 %s' % dropped)
    raw = np.vstack(pts_all)
    print('[accum] frames_ok=%d  raw pts=%d  dropped=%s' % (frames_ok, len(raw), dropped))
    final = voxel_centroid(raw, args.voxel)
    print('[accum] voxel(%.2f m) → %d pts  bbox=%s' %
          (args.voxel, len(final), np.round(np.stack([final.min(0), final.max(0)]), 2).tolist()))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
    pcd_stats.write_pcd_xyz(args.out, final, legacy8=True)
    print('[accum] wrote %s（%.2f MB）' % (args.out, os.path.getsize(args.out) / 1e6))

    if args.poses_out:
        with open(args.poses_out, 'w') as f:
            for row in poses:
                f.write(json.dumps(row) + '\n')
        print('[accum] poses → %s（%d 帧）' % (args.poses_out, len(poses)))

    st = pcd_stats.stats(final, args.out)
    rep = {'bag': os.path.abspath(args.bag), 'pose_source': args.pose_source,
           'spawn': args.spawn, 'voxel': args.voxel, 'frames_ok': frames_ok,
           'frames_tf_failed': n_tf, 'dropped': dropped, 'points_raw': int(len(raw)),
           'points': int(len(final)), 'bbox': [[float(v) for v in final.min(0)],
                                               [float(v) for v in final.max(0)]],
           'stats': st}
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(rep, f, indent=2)
        print('[accum] json → %s' % args.json)
    if args.meta:
        guard_mod = os.path.join(_repo_root(), 'src', 'rm_utils', 'rm_utils', 'map_asset_guard.py')
        spec = importlib.util.spec_from_file_location('rm_map_asset_guard', guard_mod)
        g = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(g)
        guard = {'pose_source': args.pose_source, 'accumulated_points': int(len(raw)),
                 'voxel_size': args.voxel,
                 'bbox': rep['bbox'], 'health_warning': False,
                 'dropped_points_height': dropped['height'],
                 'dropped_points_range': dropped['range'],
                 'skipped_frames_jump': dropped['jump'],
                 'skipped_frames_tf_fallback': 0,
                 'tf_frames_ok': frames_ok, 'tf_frames_failed': n_tf,
                 'source': 'tools/scripts/mapping/accumulate_pcd_gt.py'}
        g.write_manifest(args.meta, os.path.basename(args.meta) or args.world,
                         g.KIND_PCD, args.world or os.path.basename(args.meta),
                         [args.out], spawn={'x': sx, 'y': sy, 'z': sz, 'yaw': syaw},
                         extra=guard)
        print('[accum] meta → %s.meta.yaml' % args.meta)
    return 0


if __name__ == '__main__':
    sys.exit(main())
