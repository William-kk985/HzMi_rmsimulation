#!/usr/bin/env python3
"""build_sc_db —— 离线建 Scan Context keyframe DB（`PCD/<world>_sc_db.npz`）。

输入是**建图跑的 bag**（`mode:=mapping` 录的 `bag/raw`）：只要
  · `/cloud_registered`（odom 系点云，10 Hz）
  · `/tf` + `/tf_static`（至少含 map→odom→odom_level→base_link→livox_frame）
就够。**不需要真值、不需要 PCD/map 资产、不需要 ROS 图**（只读 sqlite + TF 链）。

为什么用 bag 而不是 PCD：
  · 描述子必须来自**与查询同源**的传感器视角（max-height 图），从 PCD 栅格重采样会引入
    "库看的是地面以上薄壳、查询看的是雷达一圈"的同源性问题（docs/scan_context_samesource_gate_report.md）；
  · keyframe 位姿直接取**同一跑**的 `T_map_base`（map→odom 来自 slam_toolbox 的实时 TF），
    ⇒ 与这份跑保存下来的 `PCD/<world>.pcd` / `map/<world>.pgm` **同一 map 帧**，不靠真值对齐。

用法（在仓库根、source 过 ROS 的 shell 里）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 run scan_context_relocalizer build_sc_db -- \\
      --bag third_party/eval_data/capture_q2027b/bag/raw \\
      --world RMUC2027 --out src/rm_nav_bringup/PCD/RMUC2027_sc_db.npz

  # 只保留建图跑路线上的稀疏 keyframe；排除评估帧窗口（做 held-out 时需要）
  ... --min-move 0.35 --exclude-stamps third_party/eval_data/cap2027/frames/index.json

依赖：numpy + 本仓库 `third_party/eval/{eval_lib,extract_frames}.py`（只读复用，不自造 bag 解析）。
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys

import numpy as np

from . import db as scdb
from . import scan_context as SC

FORMAT_VERSION = scdb.FORMAT_VERSION


def _repo_root():
    """找仓库根：优先当前工作目录的祖先（`ros2 run` 从仓库根跑），其次本文件的祖先。

    为什么不能简单数 `..`：`--symlink-install` 对 ament_python 的 .py 是**复制**到
    `build/<pkg>/<pkg>/`（不是符号链接），`__file__` 不在 src 下（实测踩过）。
    """
    starts = [os.getcwd(), os.path.dirname(os.path.abspath(__file__))]
    for start in starts:
        cur = os.path.abspath(start)
        while True:
            if os.path.isfile(os.path.join(cur, 'third_party', 'eval', 'extract_frames.py')):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return os.path.abspath(os.getcwd())


def _load_eval(eval_dir):
    eval_dir = os.path.abspath(eval_dir)
    if not os.path.isfile(os.path.join(eval_dir, 'extract_frames.py')):
        raise SystemExit(
            '找不到 third_party/eval/extract_frames.py（--eval-dir=%s）\n'
            '  本脚本复用仓库的 bag 只读解析；请从仓库根运行，或显式给 --eval-dir' % eval_dir)
    if eval_dir not in sys.path:
        sys.path.insert(0, eval_dir)
    import eval_lib as el  # noqa: E402
    from extract_frames import open_db, topic_map, read_msgs, chain  # noqa: E402
    return el, open_db, topic_map, read_msgs, chain


def _yaw_of(T):
    return math.atan2(float(T[1, 0]), float(T[0, 0]))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def build(args):
    el, open_db, topic_map, read_msgs, chain = _load_eval(args.eval_dir)
    from sensor_msgs_py import point_cloud2 as pc2
    bag = args.bag
    con = open_db(bag)
    tm = topic_map(con)
    for need in ('/cloud_registered', '/tf', '/tf_static'):
        if need not in tm:
            raise SystemExit('bag 缺话题 %s（有：%s）' % (need, sorted(tm)))
    tfs = read_msgs(con, *tm['/tf'])
    tss = read_msgs(con, *tm['/tf_static'])
    clouds = read_msgs(con, *tm['/cloud_registered'])
    print('[build_sc_db] bag=%s  tf=%d tf_static=%d clouds=%d'
          % (bag, len(tfs), len(tss), len(clouds)))

    edges = {}
    for _, m in tss:
        for tr in m.transforms:
            edges[(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))] = \
                el.T_of_pose(tr.transform)

    bbox = None
    if args.pcd:
        pcd = el.read_pcd(args.pcd)
        if len(pcd) == 0:
            raise SystemExit('--pcd 为空：%s' % args.pcd)
        bbox = (pcd[:, :2].min(axis=0) - args.map_margin,
                pcd[:, :2].max(axis=0) + args.map_margin)
        print('[build_sc_db] 位姿合理性 bbox（PCD + %.2f m）：x[%.2f, %.2f] y[%.2f, %.2f]'
              % (args.map_margin, bbox[0][0], bbox[1][0], bbox[0][1], bbox[1][1]))

    ext_poses = []
    if args.poses:
        for line in open(args.poses):
            row = json.loads(line)
            ext_poses.append((float(row['stamp']), np.array(row['T_map_base'], dtype=np.float64).reshape(4, 4)))
        ext_poses.sort(key=lambda x: x[0])
        if not ext_poses:
            raise SystemExit('--poses 为空：%s' % args.poses)
        print('[build_sc_db] 外部位姿 %d 条（%s）：T_map_base 用它覆盖 bag 的 map→odom 链'
              % (len(ext_poses), args.poses))

    def external_pose(ts):
        best = min(ext_poses, key=lambda x: abs(x[0] - ts))
        if abs(best[0] - ts) > 0.11:
            return None
        return best[1]

    exclude = np.zeros(0, dtype=np.float64)
    if args.exclude_stamps:
        idx = json.load(open(args.exclude_stamps))['frames']
        exclude = np.array([f['stamp'] for f in idx], dtype=np.float64)
        print('[build_sc_db] 排除 %d 个评估帧 ±%.2f s' % (len(exclude), args.exclude_window))

    # ---- 逐帧：推进 TF（累积最新）→ T_map_base / T_level_base / T_level_cloud → 描述子 ----
    tf_i = 0
    entries = []
    t_first = float(clouds[0][0]) if clouds else 0.0
    skipped = {'tf': 0, 'points': 0, 'excluded': 0, 'pose': 0}
    for ts, cmsg in clouds:
        if args.max_time > 0 and (ts - t_first) > args.max_time:
            skipped['time'] = skipped.get('time', 0) + 1
            continue
        while tf_i < len(tfs) and tfs[tf_i][0] <= ts + 1e-6:
            for tr in tfs[tf_i][1].transforms:
                edges[(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))] = \
                    el.T_of_pose(tr.transform)
            tf_i += 1
        if exclude.size:
            if float(np.min(np.abs(exclude - ts))) < args.exclude_window:
                skipped['excluded'] += 1
                continue
        try:
            T_map_base = chain(edges, args.map_frame, args.base_frame)
            T_level_base = chain(edges, args.level_frame, args.base_frame)
            cf = cmsg.header.frame_id.lstrip('/') or args.level_frame
            T_level_cloud = np.eye(4) if cf == args.level_frame else \
                chain(edges, args.level_frame, cf)
        except KeyError:
            skipped['tf'] += 1
            continue
        if ext_poses:
            # 外部位姿按**点云 header stamp**（仿真时间）对齐；ts 是 bag 接收时间（epoch），两者不同轴
            cstamp = cmsg.header.stamp.sec + cmsg.header.stamp.nanosec * 1e-9
            T_map_base = external_pose(cstamp)
        if T_map_base is None or T_level_base is None or T_level_cloud is None:
            skipped['tf'] += 1
            continue
        if not np.all(np.isfinite(T_map_base)) or not np.all(np.isfinite(T_level_cloud)):
            skipped['tf'] += 1
            continue
        # 位姿合理性：TF 链在 slam_toolbox 发散/回环跳变时会给出 1e3 m 量级的 map→base
        # （§41 的 q2027b 跑在 ~115 s 后就是这样）。不拦掉会毒化整库。
        xy = T_map_base[:2, 3]
        if (float(np.linalg.norm(xy)) > args.max_xy
                or abs(float(T_map_base[2, 3])) > args.max_z
                or (bbox is not None and (np.any(xy < bbox[0]) or np.any(xy > bbox[1])))):
            skipped['pose'] += 1
            continue
        pts = np.asarray(pc2.read_points_numpy(cmsg, field_names=('x', 'y', 'z'),
                                               skip_nans=True), dtype=np.float64)
        if pts.ndim != 2 or len(pts) < args.min_points:
            skipped['points'] += 1
            continue
        p_level = pts @ T_level_cloud[:3, :3].T + T_level_cloud[:3, 3]
        q = p_level - T_level_base[:3, 3]
        desc = SC.descriptor(q, (0.0, 0.0), 0.0)
        if int(np.count_nonzero(desc)) < 10:
            skipped['points'] += 1
            continue
        entries.append({'stamp': float(ts), 'T': T_map_base, 'Tlb': T_level_base, 'desc': desc})

    if not entries:
        raise SystemExit('没有可用的帧（skipped=%s）' % skipped)
    print('[build_sc_db] 可用帧 %d，跳过 %s' % (len(entries), skipped))

    # ---- keyframe 抽取：按 map 系位移（默认 0.35 m）----
    kept = [entries[0]]
    for e in entries[1:]:
        d = float(np.linalg.norm(e['T'][:2, 3] - kept[-1]['T'][:2, 3]))
        dyaw = abs(_wrap(_yaw_of(e['T']) - _yaw_of(kept[-1]['T'])))
        if d >= args.min_move or (d >= args.min_move_stationary
                                  and dyaw >= math.radians(args.min_yaw_deg)):
            kept.append(e)
    if kept[-1] is not entries[-1]:
        kept.append(entries[-1])
    if len(kept) > args.max_keyframes:
        sel = np.linspace(0, len(kept) - 1, args.max_keyframes).round().astype(int)
        kept = [kept[i] for i in sel]
    print('[build_sc_db] keyframes %d（rule: move>=%.2f m 或 move>=%.2f m 且 yaw>=%.0f°）'
          % (len(kept), args.min_move, args.min_move_stationary, args.min_yaw_deg))

    descs = np.stack([k['desc'] for k in kept]).astype(np.float32)
    poses = np.stack([k['T'] for k in kept]).astype(np.float64)
    # level 帧在 map 系里的 yaw：T_map_level = T_map_base * inv(T_level_base)
    level_yaws = np.array([_yaw_of(k['T'] @ np.linalg.inv(k['Tlb'])) for k in kept],
                          dtype=np.float64)
    stamps = np.array([k['stamp'] for k in kept], dtype=np.float64)
    ring = np.stack([SC.ring_key(d) for d in descs]).astype(np.float32)

    meta = {
        'format_version': FORMAT_VERSION,
        'world': args.world,
        'map_frame': args.map_frame,
        'level_frame': args.level_frame,
        'base_frame': args.base_frame,
        'source_bag': os.path.abspath(bag),
        'created_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'command': ' '.join([os.path.basename(sys.argv[0])] + sys.argv[1:]),
        'n_scans': len(entries),
        'n_skipped': skipped,
        'keyframe_rule': 'move>=%.2f m or (move>=%.2f m and yaw>=%.0f deg); last frame forced'
                         % (args.min_move, args.min_move_stationary, args.min_yaw_deg),
        'exclude_stamps': args.exclude_stamps,
        'exclude_window_sec': args.exclude_window,
        'num_rings': SC.NUM_RINGS,
        'num_sectors': SC.NUM_SECTORS,
        'max_radius': SC.MAX_RADIUS,
        'height_max': SC.HEIGHT_MAX,
        'descriptor_origin': 'base_link origin, gravity-aligned (level_frame); '
                             'q = T_level_cloud*p - t_level_base',
        'level_yaws': 'keyframe 时刻 T_map_level 的 yaw（rad）；跨会话换算 heading 用，见 db.py',
        'pose_source': ('external jsonl: %s' % os.path.abspath(args.poses)) if args.poses
                        else 'bag map→odom TF',
        'sc_impl': 'third_party/eval/sc.py（复制为 scan_context_relocalizer/scan_context.py）',
        'notes': 'poses 是 map 系 T_map_base；map 系 = 该建图跑保存的 PCD/map 的同一帧',
    }
    db = scdb.ScanContextDB(descs, poses, ring, stamps, meta, level_yaws=level_yaws)
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    db.save(out)

    xy = poses[:, :2, 3]
    print('[build_sc_db] wrote %s（%.2f MB）' % (out, os.path.getsize(out) / 1e6))
    print('[build_sc_db] keyframe xy bbox x[%.2f, %.2f] y[%.2f, %.2f]'
          % (xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max()))
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1) if len(xy) > 1 else np.array([0.0])
    print('[build_sc_db] keyframe spacing median/max = %.2f / %.2f m'
          % (float(np.median(step)), float(step.max())))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='建 Scan Context keyframe DB（离线，读建图 bag）')
    ap.add_argument('--bag', required=True, help='建图跑 rosbag2 目录（含 raw_*.db3）')
    ap.add_argument('--out', required=True, help='输出 .npz（建议 PCD/<world>_sc_db.npz）')
    ap.add_argument('--world', default='', help='world 名（只进 meta，用于核对）')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--level-frame', default='odom_level',
                    help='重力对齐帧（默认 odom_level；没有这条静态边就建不了）')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--min-points', type=int, default=500)
    ap.add_argument('--min-move', type=float, default=0.35, help='keyframe 最小位移（m）')
    ap.add_argument('--min-move-stationary', type=float, default=0.10,
                    help='原地转时的最小位移（m，配合 --min-yaw-deg）')
    ap.add_argument('--min-yaw-deg', type=float, default=15.0)
    ap.add_argument('--max-keyframes', type=int, default=600)
    ap.add_argument('--pcd', default='',
                    help='可选：PCD/<world>.pcd；用它的 XY bbox（+margin）拦掉 TF 发散的帧')
    ap.add_argument('--map-margin', type=float, default=2.0, help='--pcd bbox 外扩容差（m）')
    ap.add_argument('--max-xy', type=float, default=60.0, help='keyframe map 系 |xy| 上限（m）')
    ap.add_argument('--max-z', type=float, default=2.0, help='keyframe map 系 |z| 上限（m）')
    ap.add_argument('--max-time', type=float, default=0.0,
                    help='可选：只用 bag 开始后这么多秒内的帧（<=0 = 全用）。'
                         '作用：建图跑中途 slam 发散时截断（§41 的 q2027b 在 ~115 s 后发散）')
    ap.add_argument('--poses', default='',
                    help='外部位姿 jsonl（accumulate_pcd_gt.py --poses-out）：逐帧覆盖 T_map_base。'
                         '描述子仍从 bag 的 level 帧算（与位姿源无关）；用于"用干净位姿重建同一份扫描库"')
    ap.add_argument('--exclude-stamps', default='', help='index.json（frames[].stamp）：排除这些查询 ±窗口')
    ap.add_argument('--exclude-window', type=float, default=0.6)
    ap.add_argument('--eval-dir', default=os.path.join(_repo_root(), 'third_party', 'eval'),
                    help='third_party/eval（复用 eval_lib/extract_frames）')
    args = ap.parse_args(argv)
    build(args)


if __name__ == '__main__':
    main()
