#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ground_leak_probe.py —— 「近场地面漏进 obstacle」的离线归因（2026-10-09）

读 `tilt_mount_probe.py` 落的 `<dir>/clouds.npz`，对 `/segmentation/obstacle` 那朵云
**逐点**算三件事（全部可复算，不需要 Gazebo）：

  ① `dz = z − g(x,y)`，其中 `g` = **与 C++ 侧同一条公式**的局部地面
     （`rm_ground_traversability/low_terrain_classifier.hpp` 的 `cellOf()`：
      0.20 m 粗格 → 4×4 个 0.05 m 细格 → 各细格"最低点" → **p05**（取秩 floor(pct/100·(n−1))））；
  ② 该点在代价图帧（odom）里的 z（用探针同一次跑采到的 TF，若给了 --probe）；
  ③ 被各种"离地间隙闸"剔掉之后，还剩多少点、这些点落在哪些距离带/方位带
     —— 这是 `obstacle_min_ground_clearance_m` 那一级的设计判据。

用法：
  python3 tools/scripts/tiltmount/ground_leak_probe.py \
      --dir .tmp_tiltmount/p5_base_plugin [--probe probe.json] [--out leak.json] \
      [--clearances 0.00,0.02,0.05,0.08,0.10,0.15]

输出（stdout + --out）：每个云的
  · n / n_ground_plane(|z+h|<0.03) / dz 的分位 / dz<=gate 的点数（= 会被闸掉的）
  · 被闸掉的点的 (水平距离带, 方位带) 直方图 —— 判"闸掉的是近场地面还是真障碍"
"""

import argparse
import json
import math
import os
import sys

import numpy as np

# 与 traversability_criteria.yaml 同源（粗格/细格/分位）
GROUND_CELL = 0.20
FINE_CELL = 0.05
GROUND_PCT = 5.0
GROUND_MIN_POINTS = 2


def pct(a, q):
    a = np.asarray(a, dtype=np.float64)
    return float(np.percentile(a, q)) if a.size else float('nan')


def local_ground(x, y, z):
    """与 C++ `LowTerrainClassifier::cellOf()` 同一条公式的局部地面高度（逐点返回）。"""
    k = int(round(GROUND_CELL / FINE_CELL))
    ix = np.floor(x / FINE_CELL).astype(np.int64)
    iy = np.floor(y / FINE_CELL).astype(np.int64)
    key = (ix.astype(np.int64) << 32) | (iy.astype(np.int64) & 0xffffffff)
    order = np.argsort(key, kind='stable')
    ks = key[order]
    zs = z[order]
    # 每个细格的最低点
    uniq, start = np.unique(ks, return_index=True)
    end = np.append(start[1:], len(ks))
    lows = np.array([zs[s:e].min() for s, e in zip(start, end)])
    npts = end - start
    fux = (uniq >> 32).astype(np.int64)
    fuy = ((uniq & 0xffffffff).astype(np.int64))
    fuy = np.where(fuy >= 2 ** 31, fuy - 2 ** 32, fuy)
    # 细格 → 粗格
    cx = np.floor_divide(fux, k)
    cy = np.floor_divide(fuy, k)
    ckey = (cx << 32) | (cy & 0xffffffff)
    o2 = np.argsort(ckey, kind='stable')
    cks = ckey[o2]
    clows = lows[o2]
    cn = npts[o2]
    cuniq, cstart = np.unique(cks, return_index=True)
    cend = np.append(cstart[1:], len(cks))
    ground = {}
    for key_c, s, e in zip(cuniq, cstart, cend):
        lo = np.sort(clows[s:e])
        n = int(cn[s:e].sum())
        if lo.size == 0 or n < max(1, GROUND_MIN_POINTS):
            ground[int(key_c)] = np.inf
            continue
        cnt = lo.size
        target = int(math.floor(GROUND_PCT / 100.0 * (cnt - 1)))
        ground[int(key_c)] = float(lo[min(target, cnt - 1)])
    # 逐点取它自己的粗格地面
    pt_ckey = (np.floor(x / GROUND_CELL).astype(np.int64) << 32) | \
              (np.floor(y / GROUND_CELL).astype(np.int64) & 0xffffffff)
    g = np.array([ground.get(int(kk), np.inf) for kk in pt_ckey], dtype=np.float64)
    return g


def load_probe(dirname, probe=None):
    p = probe or os.path.join(dirname, 'probe.json')
    if not os.path.isfile(p):
        return None
    with open(p) as f:
        return json.load(f)


def describe(P, name, h_sensor, clearances, frame='livox_frame'):
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    rxy = np.hypot(x, y)
    az = np.degrees(np.arctan2(y, x))
    g = local_ground(x, y, z)
    dz = z - g
    finite = np.isfinite(dz)
    on_plane = np.abs(z + h_sensor) < 0.03
    d = {
        'name': name, 'n': int(len(P)),
        'z_p01': pct(z, 1), 'z_p50': pct(z, 50), 'z_p99': pct(z, 99),
        'dz_p01': pct(dz[finite], 1), 'dz_p05': pct(dz[finite], 5),
        'dz_p50': pct(dz[finite], 50), 'dz_p95': pct(dz[finite], 95),
        'n_ground_plane_+-0.03': int(on_plane.sum()),
        'n_no_ground_cell': int((~finite).sum()),
        'rxy_p50': pct(rxy, 50),
        'by_clearance': {},
    }
    bands = [(0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.75),
             (0.75, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 1e9)]
    sec = [(-180, -120), (-120, -60), (-60, 0), (0, 60), (60, 120), (120, 180)]
    for c in clearances:
        m = finite & (dz <= c)
        hb = {}
        for lo, hi in bands:
            hb['%.2f-%.2f' % (lo, hi)] = int((m & (rxy >= lo) & (rxy < hi)).sum())
        hs = {}
        for lo, hi in sec:
            hs['%d..%d' % (lo, hi)] = int((m & (az >= lo) & (az < hi)).sum())
        d['by_clearance']['%.3f' % c] = {
            'n_dropped': int(m.sum()), 'frac': round(float(m.mean()), 4),
            'n_dropped_on_plane': int((m & on_plane).sum()),
            'n_kept': int((~m & finite).sum()),
            'dropped_rxy_bands': hb, 'dropped_az_bands': hs,
            'kept_min_rxy': float(rxy[~m & finite].min()) if (~m & finite).any() else None}
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--probe', default=None)
    ap.add_argument('--npz', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--clearances', default='0.00,0.02,0.05,0.08,0.10,0.15')
    args = ap.parse_args()

    npz_path = args.npz or os.path.join(args.dir, 'clouds.npz')
    if not os.path.isfile(npz_path):
        print('没有 %s' % npz_path, file=sys.stderr)
        return 2
    z = np.load(npz_path)
    pr = load_probe(args.dir, args.probe)
    h_sensor = 0.2595
    if pr:
        # 传感器离地高度：优先用探针里 TF 的 base_link 高度差（若在）
        try:
            h_sensor = float(pr['tf']['base_link->livox_frame']['xyz'][2]) + 0.102499
        except Exception:
            pass
    cl = [float(v) for v in args.clearances.split(',')]
    out = {'npz': npz_path, 'keys': sorted(z.files), 'h_sensor_used': h_sensor, 'clouds': {}}
    for k in sorted(z.files):
        P = z[k]
        if P.ndim != 2 or P.shape[1] != 3:
            continue
        tag = k.rsplit('_', 1)[0]
        if tag not in ('obstacle', 'ground', 'raw', 'registered'):
            continue
        out['clouds'].setdefault(tag, []).append(describe(P, k, h_sensor, cl))
    with open(args.out, 'w') if args.out else sys.stdout as fh:
        json.dump(out, fh, indent=1, ensure_ascii=False)
        fh.write('\n')
    # 人读摘要
    for tag, lst in out['clouds'].items():
        for d in lst:
            print('== %-28s n=%-6d  z p50=%.4f  dz p05/p50/p95=%.4f/%.4f/%.4f  '
                  'on_plane(±0.03)=%d  nocell=%d'
                  % (d['name'], d['n'], d['z_p50'], d['dz_p05'], d['dz_p50'], d['dz_p95'],
                     d['n_ground_plane_+-0.03'], d['n_no_ground_cell']))
            for c, v in d['by_clearance'].items():
                print('     gate dz<=%s → drop %4d (%.1f%%, of which on-plane %d), keep %4d, '
                      'kept min rxy=%.3f'
                      % (c, v['n_dropped'], 100 * v['frac'], v['n_dropped_on_plane'],
                         v['n_kept'], v['kept_min_rxy'] if v['kept_min_rxy'] is not None else -1))
                if c in ('0.050', '0.100'):
                    print('        被剔点的水平距离带 %s' % v['dropped_rxy_bands'])
                    print('        被剔点的方位带     %s' % v['dropped_az_bands'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
