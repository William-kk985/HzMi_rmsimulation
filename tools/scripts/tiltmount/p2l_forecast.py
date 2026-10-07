#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p2l_forecast.py —— 用**实测的一帧**预测 `/scan` 与 `obstacle_layer` 的标记（离线，可复算）

为什么需要（docs/tilted_lidar_fidelity.md §K）：`/scan` 那条链路上，"地面点能不能变成
lethal 格"由三件事共同决定，而它们分散在三个地方、**任何一次实跑只能看到最后那张图**：

  ① `pointcloud_to_laserscan`：z 带（`min/max_height`，在**点云自带帧**里）、`range_min`、
     角度分辨率 ⇒ 每个角度 bin 保留**最近**的点 ⇒ `/scan.ranges`；
  ② nav2 `obstacle_layer`：`projectLaser`（**z 强行置 0**）+ TF 搬到代价图帧（odom）⇒
     每条波束在那个帧里的 z = **那一帧原点平移的 z**（与它实际打到的三维点无关！）⇒
     `min/max_obstacle_height` 在这条链路上只是一条常量闸；
  ③ `obstacle_min_range / obstacle_max_range`。

本脚本把 ①②③ **逐字复算**（输入 = `costmap_input_dump.py` 落的 CSV），于是可以纯离线地回答：
  · "如果近地剔除开着，`/scan` 的哪几条波束会消失"（改的是**点云**这一级）；
  · "如果高度带改成 [lo,hi]，会丢多少波束"（改的是**代价图**这一级）；
  · "车半径内还有多少格会被标成 lethal"（把保留下来的波束画到 0.02 m 栅格上数）。

用法：
  python3 tools/scripts/tiltmount/p2l_forecast.py --dump .tmp_tiltmount/<tag>/dump \
      [--band-lo 0.0 --band-hi 2.0] [--near-ground 0.05] [--near-ground 0.0] ... \
      [--out forecast.json]

数值口径与 C++ 对齐：
  · 局部地面 g(x,y) = 0.20 m 粗格内各 0.05 m 细格"最低点"的 p05（取秩 floor(pct/100*(n-1))），
    用 **ground ∪ obstacle 两朵云合起来**建格（与 ground_segmentation_node.cc 同一朵 cloud）；
  · p2l 的 z 带在**点云自带帧**里量、`range = hypot(x,y)`、每 bin 取最近；
  · 代价图标记：`pz ∈ [band_lo, band_hi]`、`dist_cells ∈ [min_range_cells, max_range_cells]`
    （cellDistance = ceil(r/res)），落在 0.02 m 栅格上的格标 lethal。
"""

import argparse
import json
import math
import os
import sys

import numpy as np


def pct(a, q):
    a = np.asarray(a, dtype=np.float64)
    return float(np.percentile(a, q)) if a.size else float('nan')


# --------------------------------------------------------------------------- 局部地面（= C++ 同一条公式）
def local_ground_xy(P, cell=0.20, res=0.05, pctile=5.0, minp=2):
    """返回 dict: 粗格 key(int) → 地面高度；key 的编码与 C++ keyOf() 同构。"""
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    ix = np.floor(x / res).astype(np.int64)
    iy = np.floor(y / res).astype(np.int64)
    key = (ix << 32) | (iy & 0xffffffff)
    order = np.argsort(key, kind='stable')
    ks, zs = key[order], z[order]
    u, st = np.unique(ks, return_index=True)
    en = np.append(st[1:], len(ks))
    lows = np.array([zs[s:e].min() for s, e in zip(st, en)])
    npts = en - st
    k = int(round(cell / res))
    cx = np.floor_divide((u >> 32).astype(np.int64), k)
    cy = np.floor_divide((u & 0xffffffff).astype(np.int64), k)
    ck = (cx << 32) | (cy & 0xffffffff)
    o2 = np.argsort(ck, kind='stable')
    cks, clows, cn = ck[o2], lows[o2], npts[o2]
    cu, cs = np.unique(cks, return_index=True)
    ce = np.append(cs[1:], len(cks))
    out = {}
    for key_c, s, e in zip(cu, cs, ce):
        lo = np.sort(clows[s:e])
        n = int(cn[s:e].sum())
        if lo.size == 0 or n < max(1, minp):
            out[int(key_c)] = np.inf
            continue
        t = int(math.floor(pctile / 100.0 * (lo.size - 1)))
        out[int(key_c)] = float(lo[min(t, lo.size - 1)])
    return out


def ground_at(ground, P, cell=0.20):
    cx = np.floor(P[:, 0] / cell).astype(np.int64)
    cy = np.floor(P[:, 1] / cell).astype(np.int64)
    ck = (cx << 32) | (cy & 0xffffffff)
    return np.array([ground.get(int(k), np.inf) for k in ck], dtype=np.float64)


# --------------------------------------------------------------------------- p2l（逐字复算）
def p2l_forecast(P, frame_z_band, range_min, range_max, angle_min, angle_max, inc):
    n_bins = int(math.ceil((angle_max - angle_min) / inc))
    res = np.full(n_bins, np.inf)
    z = P[:, 2]
    keep = (z <= frame_z_band[1]) & (z >= frame_z_band[0])
    r = np.hypot(P[:, 0], P[:, 1])
    keep &= (r >= range_min) & (r <= range_max)
    ang = np.arctan2(P[keep, 1], P[keep, 0])
    rr = r[keep]
    ok = (ang >= angle_min) & (ang <= angle_max)
    idx = ((ang[ok] - angle_min) / inc).astype(np.int64)
    np.minimum.at(res, idx, rr[ok])
    return res


# --------------------------------------------------------------------------- 代价图标记（逐字复算）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dump', required=True)
    ap.add_argument('--band-lo', type=float, default=0.0)
    ap.add_argument('--band-hi', type=float, default=2.0)
    ap.add_argument('--near-ground', type=float, nargs='*', default=[0.0, 0.05, 0.10],
                    help='要试的"近地剔除"阈值（0 = 关）')
    ap.add_argument('--res', type=float, default=0.02)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.dump, 'meta.json')))

    def load(tag, i):
        p = os.path.join(args.dump, '%s_%d.csv' % (tag, i))
        return np.loadtxt(p, delimiter=',', skiprows=1) if os.path.isfile(p) else None

    out = {'dump': args.dump, 'band': [args.band_lo, args.band_hi], 'frames': []}
    for i in range(len(meta['obstacle'])):
        O = load('obstacle', i)
        G = load('ground', i)
        raw = load('raw', i)
        if O is None or G is None or raw is None:
            continue
        # ★ 口径：**同一帧的 raw 云**是唯一自洽的输入（ground/obstacle 两朵云是**异步**采到的，
        #   帧号不同 ⇒ 不能拼）。用 raw 复算"近地剔除关 / 开"两版 p2l 会输出什么：
        #     · "关"版候选 = raw 里 `dz > 0` 的点（= 只有真值地面不算障碍的那一版，与今天
        #       实测的 `/scan` 对账，见 act_vs_recon）；
        #     · "开"版候选 = 再剔掉 `dz ≤ thr` 的点。
        A = raw
        ground = local_ground_xy(A)
        gA = ground_at(ground, A)
        dzA = A[:, 2] - gA
        fin = np.isfinite(dzA)
        rec = {'i': i,
               'n_raw': int(len(raw)), 'n_ground': int(len(G)), 'n_obstacle': int(len(O)),
               'n_raw_dz_gt_0': int((fin & (dzA > 0.0)).sum()),
               'dz_raw_p05': pct(dzA[fin], 5), 'dz_raw_p50': pct(dzA[fin], 50),
               'dz_raw_p95': pct(dzA[fin], 95),
               'n_raw_no_ground_cell': int((~fin).sum())}

        f = meta['obstacle'][i]
        fid = f['frame_id']
        Tkey = 'T_odom_from_%s' % fid
        R = np.array(f[Tkey]['R']) if Tkey in f else None
        t = np.array(f[Tkey]['t']) if Tkey in f else None

        sc = meta['scan'][i] if i < len(meta['scan']) else None
        if sc is None:
            continue
        am, inc = sc['angle_min'], sc['angle_increment']
        ax = am + inc * (sc['n'] - 1)   # 探针只落了 angle_min/increment ⇒ 用 n 反推上界
        rmin, rmax = sc['range_min'], sc['range_max']
        band = (-1.0, 1.0)
        s_now = p2l_forecast(A[fin & (dzA > 0.0)], band, rmin, rmax, am, ax, inc)
        rec['n_scan_now'] = int(np.isfinite(s_now).sum())
        s_act = np.loadtxt(os.path.join(args.dump, 'scan_%d.csv' % i), delimiter=',',
                           skiprows=1)[:, 1] if os.path.isfile(
            os.path.join(args.dump, 'scan_%d.csv' % i)) else None
        if s_act is not None and len(s_act) == len(s_now):
            fn_a, fn_n = np.isfinite(s_act), np.isfinite(s_now)
            rec['act_vs_recon'] = {
                'n_act': int(fn_a.sum()), 'n_recon': int(fn_n.sum()),
                'n_agree': int((fn_a & fn_n).sum()),
                'n_only_act': int((fn_a & ~fn_n).sum()),
                'n_only_recon': int((~fn_a & fn_n).sum()),
                'range_absdiff_p50': pct(np.abs(s_act[fn_a & fn_n] - s_now[fn_a & fn_n]), 50),
                'range_absdiff_p95': pct(np.abs(s_act[fn_a & fn_n] - s_now[fn_a & fn_n]), 95)}
        rec['near_ground'] = {}
        for thr in args.near_ground:
            keep = fin & (dzA > max(0.0, thr))
            s_new = p2l_forecast(A[keep], band, rmin, rmax, am, ax, inc)
            fn, fnn = np.isfinite(s_now), np.isfinite(s_new)
            ang = am + np.arange(len(s_now)) * inc
            rec['near_ground']['%.2f' % thr] = {
                'n_kept_points': int(keep.sum()),
                'n_scan': int(fnn.sum()),
                'n_scan_lost': int((fn & ~fnn).sum()),
                'n_scan_gained': int((~fn & fnn).sum()),
                'lost_ranges': [round(float(v), 4) for v in s_now[fn & ~fnn][:40]],
                'lost_az_deg': [round(float(math.degrees(a)), 1)
                                for a in ang[fn & ~fnn][:40]],
                'kept_ranges_p50': pct(s_new[fnn], 50)}
        if R is None:
            out['frames'].append(rec)
            continue

        def cells(scan):
            fn = np.isfinite(scan)
            ang = am + np.arange(len(scan)) * inc
            pts = np.stack([scan[fn] * np.cos(ang[fn]), scan[fn] * np.sin(ang[fn]),
                            np.zeros(int(fn.sum()))], axis=1)
            Q = (R @ pts.T).T + t
            z = Q[:, 2]
            m = (z >= args.band_lo) & (z <= args.band_hi)
            key = set()
            for x, y, ok in zip(Q[:, 0], Q[:, 1], m):
                if not ok:
                    continue
                key.add((int(math.floor(x / args.res)), int(math.floor(y / args.res))))
            return key, int(m.sum()), int((~m).sum()), \
                float(z.min()) if len(z) else float('nan'), \
                float(z.max()) if len(z) else float('nan')
        k_now, ok_now, drop_now, zmin, zmax = cells(s_now)
        rec['mark_now'] = {'n_marked_beams': ok_now, 'n_beams_outside_band': drop_now,
                           'n_cells': len(k_now), 'z_min': zmin, 'z_max': zmax,
                           'robot_z_in_odom': float(t[2])}
        rec['mark_near_ground'] = {}
        for thr in args.near_ground:
            if thr <= 0.0:
                continue
            keep = fin & (dzA > thr)
            s_new = p2l_forecast(A[keep], band, rmin, rmax, am, ax, inc)
            k_new, ok_new, drop_new, zmin2, zmax2 = cells(s_new)
            rec['mark_near_ground']['%.2f' % thr] = {
                'n_marked_beams': ok_new, 'n_cells': len(k_new),
                'cells_removed': len(k_now - k_new), 'cells_added': len(k_new - k_now),
                'n_marked_beams_removed': ok_now - ok_new}
        out['frames'].append(rec)

    # 汇总（按帧中位数）
    def med(path):
        vals = []
        for r in out['frames']:
            cur = r
            for k in path:
                cur = cur.get(k) if isinstance(cur, dict) else None
                if cur is None:
                    break
            if isinstance(cur, (int, float)):
                vals.append(cur)
        return pct(vals, 50) if vals else None

    out['summary'] = {
        'n_frames': len(out['frames']),
        'n_scan_now_median': med(['n_scan_now']),
        'n_cells_now_median': med(['mark_now', 'n_cells']),
        'beams_outside_band_now_median': med(['mark_now', 'n_beams_outside_band']),
    }
    for thr in args.near_ground:
        if thr <= 0.0:
            continue
        out['summary']['scan_lost_%.2f_median' % thr] = med(['near_ground', '%.2f' % thr,
                                                             'n_scan_lost'])
        out['summary']['cells_removed_%.2f_median' % thr] = med(['mark_near_ground',
                                                                 '%.2f' % thr, 'cells_removed'])
    txt = json.dumps(out, indent=1, ensure_ascii=False)
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(txt + '\n')
    print(txt)
    return 0


if __name__ == '__main__':
    sys.exit(main())
