#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_db_pcd_same_frame.py —— **"SC DB 与 PCD 同一帧"** 的硬断言（fail loud）。

为什么需要它（2026-10-29，measurements §45）

§42.7#5 承认：上一轮的"GT"是**建图跑自己的 DB keyframe 位姿**，不是外部真值；而且
DB 与 PCD 都来自那次 `map→odom` 坏掉的跑 ⇒ 两者"静态同帧"，但**时间一致性坏了**
（同一段围栏被叠成多层）。只看位姿数值是查不出这件事的——必须回到**扫描**：

本脚本对 DB 的**每一个 keyframe** 做两件事：

 ① **同一帧断言（几何）**：把该 keyframe 的扫描按 `T_map_base` 搬到 map 系，
    与磁盘上的 PCD 做最近邻；`median / p90 / frac<tol` 三个数必须在门限内。
    位姿与 PCD 只要差一个常量平移/旋转，这里就会**立刻**变大（fail loud，退出码 1）。
 ② **描述子原点约定断言**：用 bag 的 TF 复算
    `q = T_level←cloud·p − t_level←base`，再重算 20×60 描述子，与 npz 里存的那一列**逐元素比较**。
    它证明"描述子的原点 = base_link（重力对齐 level 帧）"，而不是 map 原点或雷达原点；
    同时证明本脚本与建库用的是**同一条链**。
 ③ 附带自检：`poses[:,2,3] ≈ 0`（map 是水平系、base_link 在出生高度上）、
    `meta.map_frame/level_frame/base_frame` 与参数一致、`source_bag` 存在。

用法（仓库根）：
  python3 tools/scripts/mapping/check_db_pcd_same_frame.py \
      --db src/rm_nav_bringup/PCD/RMUC2027_sc_db.npz \
      --pcd src/rm_nav_bringup/PCD/RMUC2027.pcd \
      --bag third_party/eval_data/capture_map2027/bag/raw \
      --max-median 0.05 --max-p90 0.10 --min-frac 0.90
"""
from __future__ import annotations

import argparse
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


def _load(name, path):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--bag', default='', help='建库用的 bag（重算描述子/取扫描）；默认用 meta.source_bag')
    ap.add_argument('--tol', type=float, default=0.10, help='单点"贴近 PCD"的门限（m）；'
                    'PCD 是 0.10 m 体素（质心）⇒ 0.05 m 门限的物理上限只有 ~75%')
    ap.add_argument('--max-median', type=float, default=0.05, help='最近邻中位门限（m）')
    ap.add_argument('--max-p90', type=float, default=0.12, help='最近邻 p90 门限（m）')
    ap.add_argument('--min-frac', type=float, default=0.90, help='frac(d<tol) 下限（0.10 m 体素 + 转弯扫描畸变 ⇒ 0.90 是实测量级；旧资产是 0.72-0.75）')
    ap.add_argument('--max-abs-z', type=float, default=0.02, help='keyframe base z 的允许偏差（m）')
    ap.add_argument('--json', default='')
    a = ap.parse_args(argv)

    repo = _repo_root()
    sys.path.insert(0, os.path.join(repo, 'src', 'rm_localization', 'scan_context_relocalizer'))
    from scan_context_relocalizer import db as scdb
    from scan_context_relocalizer import scan_context as SC

    pcd_stats = _load('pcd_stats_mod', os.path.join(repo, 'tools', 'scripts', 'mapping', 'pcd_stats.py'))
    from scipy.spatial import cKDTree

    db = scdb.ScanContextDB.load(a.db)
    meta = db.meta
    print('[sameframe] db=%s  keyframes=%d  meta.pose_source=%s'
          % (a.db, len(db.poses), meta.get('pose_source', '?')))
    bag = a.bag or meta.get('source_bag') or ''
    if not bag or not os.path.isdir(bag):
        raise SystemExit('[sameframe] ❌ 找不到 bag（--bag 或 meta.source_bag）：%r' % bag)

    pcd, _h = pcd_stats.read_pcd_xyz(a.pcd)
    pcd = np.asarray(pcd, dtype=np.float64)
    if len(pcd) == 0:
        raise SystemExit('[sameframe] ❌ PCD 为空：%s' % a.pcd)
    tree = cKDTree(pcd)
    print('[sameframe] pcd=%s  %d pts  bbox=%s' %
          (a.pcd, len(pcd), np.round(np.stack([pcd.min(0), pcd.max(0)]), 2).tolist()))

    # ---- bag 读取（复用 third_party/eval 的只读解析，与 build_sc_db 同一口径）
    ed = os.path.join(repo, 'third_party', 'eval')
    sys.path.insert(0, ed)
    import eval_lib as el                    # noqa: E402
    from extract_frames import open_db, topic_map, read_msgs, chain  # noqa: E402
    from sensor_msgs_py import point_cloud2 as pc2
    con = open_db(bag)
    tm = topic_map(con)
    for need in ('/cloud_registered', '/tf', '/tf_static'):
        if need not in tm:
            raise SystemExit('[sameframe] ❌ bag 缺话题 %s' % need)
    tfs = read_msgs(con, *tm['/tf'])
    tss = read_msgs(con, *tm['/tf_static'])
    clouds = read_msgs(con, *tm['/cloud_registered'])
    cstamps = np.array([float(c[0]) for c in clouds])
    print('[sameframe] bag clouds=%d  tf=%d  tf_static=%d' % (len(clouds), len(tfs), len(tss)))

    # 静态边 + 逐帧推进（与 build_sc_db 完全相同的写法）
    edges = {}
    for _, m in tss:
        for tr in m.transforms:
            edges[(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))] = \
                el.T_of_pose(tr.transform)
    fr = meta.get('map_frame', 'map')
    lf = meta.get('level_frame', 'odom_level')
    bf = meta.get('base_frame', 'base_link')
    for k, v in (('map_frame', fr), ('level_frame', lf), ('base_frame', bf)):
        got = {'map_frame': meta.get('map_frame'), 'level_frame': meta.get('level_frame'),
               'base_frame': meta.get('base_frame')}[k]
        if got != v:
            raise SystemExit('[sameframe] ❌ meta.%s=%r 与约定 %r 不符' % (k, got, v))

    fails = []
    rows = []
    tf_i = 0
    for i in range(len(db.poses)):
        st = float(db.stamps[i])
        while tf_i < len(tfs) and tfs[tf_i][0] <= st + 1e-6:
            for tr in tfs[tf_i][1].transforms:
                edges[(tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))] = \
                    el.T_of_pose(tr.transform)
            tf_i += 1
        j = int(np.argmin(np.abs(cstamps - st)))
        dt = abs(cstamps[j] - st)
        cmsg = clouds[j][1]
        cf = cmsg.header.frame_id.lstrip('/') or lf
        try:
            Tlb = chain(edges, lf, bf)
            Tlc = np.eye(4) if cf == lf else chain(edges, lf, cf)
        except KeyError as e:
            fails.append('keyframe %d（stamp %.3f）：bag 里缺 TF 链 %s' % (i, st, e))
            continue
        pts = np.asarray(pc2.read_points_numpy(cmsg, field_names=('x', 'y', 'z'), skip_nans=True),
                         dtype=np.float64).reshape(-1, 3)
        q = pts @ Tlc[:3, :3].T + Tlc[:3, 3] - Tlb[:3, 3]
        desc = SC.descriptor(q, (0.0, 0.0), 0.0)
        same = np.array_equal(desc.astype(np.float32), db.descriptors[i].astype(np.float32))
        # 几何：扫描（frame_id 可能是 odom）→ map：T_map_odom = T_map_base·T_base_odom
        T = db.poses[i]
        T_odom_base = chain(edges, 'odom', bf) if cf == 'odom' else None
        if cf == 'odom':
            T_map_cloud = T @ np.linalg.inv(T_odom_base)
        elif cf == lf:
            T_map_cloud = T @ np.linalg.inv(Tlb)
        else:
            T_map_cloud = T @ np.linalg.inv(Tlb) @ Tlc
        # 只测"本来会进图"的那部分点（与累积器同一套清洗）：距离闸 + 相对 base 的高度带
        r = np.linalg.norm(pts, axis=1)
        in_band = r >= 0.30
        zmap = T_map_cloud[:3, :3] @ pts.T + T_map_cloud[:3, 3:4]
        in_band &= (zmap[2] - float(T[2, 3]) >= -0.5) & (zmap[2] - float(T[2, 3]) <= 1.8)
        pts = pts[in_band]
        pm = pts @ T_map_cloud[:3, :3].T + T_map_cloud[:3, 3]
        d, _ = tree.query(pm)
        row = {'i': i, 'stamp': st, 'dt_cloud': dt, 'npts': int(len(pts)),
               'nn_median': float(np.median(d)), 'nn_p90': float(np.percentile(d, 90)),
               'nn_p99': float(np.percentile(d, 99)), 'n_used': int(len(pts)),
               'frac_lt_tol': float((d < a.tol).mean()), 'desc_equal': bool(same),
               'z': float(T[2, 3]), 'xy': [float(T[0, 3]), float(T[1, 3])],
               'cloud_frame': cf}
        rows.append(row)
        bad = []
        if not same:
            bad.append('描述子不一致')
        if row['nn_median'] > a.max_median:
            bad.append('NN 中位 %.3f > %.3f' % (row['nn_median'], a.max_median))
        if row['nn_p90'] > a.max_p90:
            bad.append('NN p90 %.3f > %.3f' % (row['nn_p90'], a.max_p90))
        if row['frac_lt_tol'] < a.min_frac:
            bad.append('frac<%.2f = %.2f < %.2f' % (a.tol, row['frac_lt_tol'], a.min_frac))
        if abs(T[2, 3]) > a.max_abs_z:
            bad.append('base z = %.3f（|z|>%.3f，map 不是出生高度系？）' % (T[2, 3], a.max_abs_z))
        if bad:
            fails.append('keyframe %d（stamp %.3f）：%s' % (i, st, '；'.join(bad)))
        print('[sameframe] kf%02d st=%.2f n=%-5d nn med %.3f p90 %.3f frac %.2f  desc=%s  xy=(%.2f,%.2f) z=%+.3f %s'
              % (i, st, len(pts), row['nn_median'], row['nn_p90'], row['frac_lt_tol'],
                 'OK' if same else 'MISMATCH', T[0, 3], T[1, 3], T[2, 3],
                 '❌ ' + '；'.join(bad) if bad else ''))

    rep = {'db': a.db, 'pcd': a.pcd, 'bag': bag, 'tol': a.tol,
           'max_median': a.max_median, 'max_p90': a.max_p90, 'min_frac': a.min_frac,
           'n_keyframes': len(db.poses), 'rows': rows, 'fails': fails,
           'summary': {'nn_median_worst': max([r['nn_median'] for r in rows], default=None),
                       'nn_p90_worst': max([r['nn_p90'] for r in rows], default=None),
                       'desc_all_equal': all(r['desc_equal'] for r in rows)}}
    if a.json:
        json.dump(rep, open(a.json, 'w'), indent=2)
        print('[sameframe] json → %s' % a.json)
    if fails:
        print('\n[sameframe] ❌ FAIL —— DB 与 PCD **不是同一帧**（或描述子原点约定不符）')
        for f in fails:
            print('  · ' + f)
        print('  修法：用同一份位姿重跑 build_sc_db（--poses）与 accumulate_pcd_gt.py（--pose-source 同一源）；'
              '不要混用两次跑的资产。')
        return 1
    print('\n[sameframe] ✅ PASS —— %d/%d keyframe：扫描按 T_map_base 落在 PCD 上（中位最差 %.3f m）、'
          '描述子与 bag 复算逐元素相同。' %
          (len(rows), len(db.poses), rep['summary']['nn_median_worst']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
