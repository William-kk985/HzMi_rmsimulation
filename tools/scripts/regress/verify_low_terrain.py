#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_low_terrain.py —— **重跑建图之后**的验收装置（离线，不需要 Gazebo）。

回答「低矮地形（坡脚/台阶/坡道）在先验图里到底有没有被表示出来」以及
「原来那个会撞的目标点，现在规划出来的路还穿不穿过它」。

口径（**与实时链路同一套阈值**，见 src/rm_nav_bringup/config/traversability_criteria.yaml）：
  · 真值 = `RMUC2026.stl` → 每 0.05 m 格"最高可站立面"高度（field_mesh_vs_map.height_grid）
  · "台阶/边沿格" = 该格高度 − 周围 3×3 格最低高度 > --edge-threshold（默认 0.12 m）
  · "可行驶斜面格" = 真值高度 ∈ [0.02, 0.15) 且局部坡度 ≤ 22°（本车能爬 23°）
  · "被表示" = 该格 0.10 m（2 格）内**有占用格**

用法（**先看旧图，再看重跑图**，两条命令同一口径）：

    # ① 旧先验图（用户现在那份）
    python3 tools/scripts/regress/verify_low_terrain.py \
        --map src/rm_nav_bringup/map/RMUC2026.yaml --label old \
        --json .tmp_lowterrain/verify_old.json --png .tmp_lowterrain/verify_old.png

    # ② 重跑建图 + pcd_to_nav2_map 出来的新图（不覆盖用户资产）
    python3 tools/scripts/regress/verify_low_terrain.py \
        --map .tmp_lowterrain/prior/RMUC2026.yaml --label new \
        --json .tmp_lowterrain/verify_new.json

    # ③ 实时链路（"坡脚在不在 /scan / obstacle 里"）：录一袋包再跑 A/B 装置
    #    （见 run_low_terrain_capture.sh / low_terrain_ab.py；本条是**建图之后**的图侧验收）

退出码 0 = 全部检查跑完（**不等于**通过）；打印里 PASS/FAIL 逐条给。
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
from field_mesh_vs_map import read_stl, load_map, height_grid  # noqa: E402


def truth_grid(shape, res, ox, oy, world='RMUC2026', spawn=(10.925, 2.524)):
    """真值高度图，**按待验收那张图的窗口/res** 算（新图窗口可能和旧图不一样）。"""
    stl = os.path.join(WS, 'src', 'rm_simulation', 'hzmi_rm_simulation', 'world',
                       '%s_world' % world, 'meshes', '%s.stl' % world)
    V = read_stl(stl)
    return height_grid(V, shape, res, ox, oy, spawn[0], spawn[1], 0.001, 1.6413436, 0.0,
                       cell=res)


def neighbours_min(h, k=1):
    """3x3（或 (2k+1)²）窗口最小值，nan → +inf。"""
    out = np.full_like(h, np.inf)
    for dx in range(-k, k + 1):
        for dy in range(-k, k + 1):
            out = np.fmin(out, np.roll(np.roll(h, dx, 0), dy, 1))
    return out


def local_slope(h, res):
    f = np.where(np.isfinite(h), h, np.nan)
    f = np.where(np.isfinite(f), f, 0.0)
    gy, gx = np.gradient(f, res)
    s = np.degrees(np.arctan(np.hypot(gx, gy)))
    s[~np.isfinite(h)] = np.nan
    return s


def window(shape, ox, oy, cx, cy, half, res):
    """返回 (切片, r0, c0)。⚠ 切片是**子图坐标**，算 map 系坐标时必须加回 r0/c0。"""
    H, W = shape
    c0 = max(0, int((cx - half - ox) / res)); c1 = min(W, int((cx + half - ox) / res) + 1)
    r0 = max(0, H - 1 - int((cy + half - oy) / res)); r1 = min(H, H - int((cy - half - oy) / res))
    return np.s_[r0:r1, c0:c1], r0, c0


def dilate(mask, k=2):
    out = mask.copy()
    for dx in range(-k, k + 1):
        for dy in range(-k, k + 1):
            out |= np.roll(np.roll(mask, dx, 0), dy, 1)
    return out


# ------------------------------------------------------------------ 静态图路径
def shortest_path(free, res, start_xy, goal_xy, ox, oy, clearance=None, edt=None):
    """在 free 格上做 8 邻接 Dijkstra。clearance（m）给了就只走 edt*res >= clearance 的格。"""
    H, W = free.shape
    ok = free.copy()
    if clearance is not None and edt is not None:
        ok &= (edt * res) >= clearance

    def nearest(x, y):
        c = int((x - ox) / res); r = H - 1 - int((y - oy) / res)
        for rad in range(0, 40):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W and ok[rr, cc]:
                        return rr, cc
        return None

    s = nearest(*start_xy); g = nearest(*goal_xy)
    if s is None or g is None:
        return None
    dist = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    SQ2 = math.sqrt(2.0)
    while pq:
        d, cur = heapq.heappop(pq)
        if cur == g:
            break
        if d > dist.get(cur, 1e18):
            continue
        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                rr, cc = r + dr, c + dc
                if not (0 <= rr < H and 0 <= cc < W) or not ok[rr, cc]:
                    continue
                nd = d + (SQ2 if dr and dc else 1.0) * res
                if nd < dist.get((rr, cc), 1e18):
                    dist[(rr, cc)] = nd
                    prev[(rr, cc)] = cur
                    heapq.heappush(pq, (nd, (rr, cc)))
    if g not in dist:
        return None
    path = [g]
    while path[-1] != s:
        path.append(prev[path[-1]])
    path.reverse()
    xs = np.array([ox + (c + 0.5) * res for _, c in path])
    ys = np.array([oy + (H - 1 - r + 0.5) * res for r, _ in path])
    return {'length_m': float(dist[g]), 'n_cells': len(path), 'x': xs, 'y': ys}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='待验收的 2D 先验图 yaml')
    ap.add_argument('--label', default='map')
    ap.add_argument('--world', default='RMUC2026')
    ap.add_argument('--collision', nargs=2, type=float, default=[-3.6, 3.1])
    ap.add_argument('--goal', nargs=2, type=float, default=[-12.64, -0.31])
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0])
    ap.add_argument('--box-half', type=float, default=0.6)
    ap.add_argument('--edge-threshold', type=float, default=0.12,
                    help='真值"台阶/边沿"判据（m，默认 0.12）')
    ap.add_argument('--represent-radius', type=float, default=0.10,
                    help='"被表示"= 该格这个半径内有占用格（m，默认 0.10 = 2 格）')
    ap.add_argument('--json', default=None)
    ap.add_argument('--png', default=None)
    a = ap.parse_args()

    m, img, occ, res, ox, oy = load_map(a.map)
    H, W = occ.shape
    known = img != 205
    free = ~occ & known
    rel = truth_grid(occ.shape, res, ox, oy, a.world)
    has = rel > -90
    hmin = neighbours_min(np.where(has, rel, np.inf), 1)
    edge = has & np.isfinite(hmin) & ((rel - hmin) > a.edge_threshold)
    slope = local_slope(np.where(has, rel, np.nan), res)
    drivable = has & (rel >= 0.02) & (rel < 0.15) & np.isfinite(slope) & (slope <= 22.0)

    rep = {'label': a.label, 'map': os.path.abspath(a.map), 'resolution': res,
           'origin': [ox, oy], 'shape': [H, W],
           'cells': {'occupied': int(occ.sum()), 'free': int(free.sum()),
                     'unknown': int((img == 205).sum())},
           'truth': {'cells_with_data': int(has.sum()),
                     'edge_cells': int(edge.sum()),
                     'drivable_slope_cells': int(drivable.sum())}}

    # ① 台阶/边沿有没有被表示（全场 + 撞击点那个窗口）
    near_occ = dilate(occ, k=int(round(a.represent_radius / res)))
    rep['edge_coverage'] = {
        'field_pct': round(100.0 * float((edge & near_occ).sum()) / max(1, int(edge.sum())), 2),
        'edge_cells': int(edge.sum()),
        'edge_covered': int((edge & near_occ).sum())}
    bx, by = a.collision
    win, wr0, wc0 = window(occ.shape, ox, oy, bx, by, a.box_half, res)
    e_w = edge[win]; o_w = occ[win]; n_w = near_occ[win]; h_w = rel[win]; f_w = free[win]
    rep['collision_box'] = {
        'center': [bx, by], 'half': a.box_half,
        'truth_edge_cells': int(e_w.sum()),
        'truth_edge_covered': int((e_w & n_w).sum()),
        'occupied_cells': int(o_w.sum()), 'free_cells': int(f_w.sum()),
        'truth_max_height_m': round(float(h_w[h_w > -90].max()), 3) if (h_w > -90).any() else None}
    if e_w.sum():
        rr, cc = np.where(e_w)
        rc, rr2 = np.where(o_w)
        H0 = H
        rep['collision_box']['truth_edge_x_range'] = [
            round(ox + (wc0 + cc.min() + 0.5) * res, 2), round(ox + (wc0 + cc.max() + 0.5) * res, 2)]
        rep['collision_box']['truth_edge_y_range'] = [
            round(oy + (H0 - 1 - (wr0 + rr.max()) + 0.5) * res, 2),
            round(oy + (H0 - 1 - (wr0 + rr.min()) + 0.5) * res, 2)]
        if len(rr2):
            rep['collision_box']['occupied_x_range'] = [
                round(ox + (wc0 + rr2.min() + 0.5) * res, 2),
                round(ox + (wc0 + rr2.max() + 0.5) * res, 2)]
            rep['collision_box']['occupied_y_range'] = [
                round(oy + (H0 - 1 - (wr0 + rc.max()) + 0.5) * res, 2),
                round(oy + (H0 - 1 - (wr0 + rc.min()) + 0.5) * res, 2)]
        if len(rr2):
            xs = ox + (cc + 0.5) * res; ys = oy + (H - 1 - (np.where(e_w)[0]) + 0.5) * res
            ox_ = ox + (rr2 + 0.5) * res; oy_ = oy + (H - 1 - rc + 0.5) * res
            d = np.sqrt((xs[:, None] - ox_[None, :]) ** 2 + (ys[:, None] - oy_[None, :]) ** 2)
            rep['collision_box']['nearest_occupied_to_edge_m'] = round(float(d.min()), 3)

    # ② 可行驶斜面有没有被误标成占用
    rep['drivable'] = {
        'cells': int(drivable.sum()),
        'free_pct': round(100.0 * float((drivable & free).sum()) / max(1, int(drivable.sum())), 2),
        'occupied_pct': round(100.0 * float((drivable & occ).sum()) / max(1, int(drivable.sum())), 2),
        'unknown_pct': round(100.0 * float((drivable & ~known).sum()) / max(1, int(drivable.sum())), 2)}
    # 对照口径：图上自由但真值 > 0.10 m（**注意**：平台/台面的内部在两张图里都合法地是 free，
    # 只有边沿必须占用 ⇒ 这个数只当"背景量"，判据看 edge_coverage）
    rep['free_but_high'] = {'%.2f' % t: int((free & (rel > t)).sum())
                            for t in (0.10, 0.20, 0.30)}

    # ③ 路径：原来会撞的目标，现在规划出来的路会不会穿进撞击点
    try:
        from scipy import ndimage as ndi
        edt = ndi.distance_transform_edt(~occ)
    except Exception as e:  # noqa: BLE001
        edt = None
        print('[verify] ⚠ 没有 scipy ⇒ 跳过余量约束与路径检查：%s' % e)
    rep['paths'] = {}
    if edt is not None:
        truth_at = lambda x, y: float(rel[int(H - 1 - (y - oy) / res), int((x - ox) / res)])
        for clr in (0.0, 0.205, 0.25, 0.30):
            p = shortest_path(free, res, a.start, a.goal, ox, oy,
                              clearance=(clr if clr > 0 else None), edt=edt)
            if p is None:
                rep['paths']['%.3f' % clr] = {'reachable': False}
                continue
            dcoll = np.sqrt((p['x'] - bx) ** 2 + (p['y'] - by) ** 2)
            hh = np.array([truth_at(x, y) for x, y in zip(p['x'], p['y'])])
            # 路径每格的"局部台阶高差" = 该格真值高度 − 3×3 邻域最低真值高度
            rise = []
            for x, y in zip(p['x'], p['y']):
                r_, c_ = H - 1 - int((y - oy) / res), int((x - ox) / res)
                w = rel[max(0, r_ - 1):r_ + 2, max(0, c_ - 1):c_ + 2]
                w = w[w > -90]
                rise.append(float(rel[r_, c_] - w.min()) if w.size else 0.0)
            rise = np.array(rise)
            rep['paths']['%.3f' % clr] = {
                'reachable': True, 'length_m': round(p['length_m'], 2),
                'min_dist_to_collision_m': round(float(dcoll.min()), 3),
                'cells_inside_collision_box': int((dcoll <= a.box_half * math.sqrt(2)).sum()),
                'path_min_truth_height_m': round(float(np.nanmin(hh)), 3),
                'path_max_truth_height_m': round(float(np.nanmax(hh)), 3),
                'path_max_local_step_m': round(float(np.nanmax(rise)), 3),
                'path_pct_with_local_step_gt_edge': round(
                    100.0 * float(np.mean(rise > a.edge_threshold)), 2),
                'x': [round(float(v), 3) for v in p['x']], 'y': [round(float(v), 3) for v in p['y']]}

    # ---- 打印 ----
    print('== verify_low_terrain [%s] %s' % (a.label, a.map))
    print('  栅格 %dx%d @ %.3f m  origin (%.2f, %.2f) | 占用 %d / 自由 %d / 未知 %d'
          % (W, H, res, ox, oy, rep['cells']['occupied'], rep['cells']['free'],
             rep['cells']['unknown']))
    print('  真值：有数据 %d 格；台阶/边沿(>%.2f m) %d 格；可行驶斜面(≤22°) %d 格'
          % (rep['truth']['cells_with_data'], a.edge_threshold,
             rep['truth']['edge_cells'], rep['truth']['drivable_slope_cells']))
    ec = rep['edge_coverage']
    print('  【全场台阶/边沿被表示】%.2f%% (%d/%d)  —— 判据：%.0f%% 以上'
          % (ec['field_pct'], ec['edge_covered'], ec['edge_cells'], 80))
    cb = rep['collision_box']
    print('  【撞击点 map(%.2f,%.2f) ±%.2f m】真值边沿 %d 格，其中被表示 %d 格；占用 %d 格；'
          '真值最高 %.3f m' % (bx, by, a.box_half, cb['truth_edge_cells'],
                            cb['truth_edge_covered'], cb['occupied_cells'],
                            cb['truth_max_height_m'] if cb['truth_max_height_m'] is not None else float('nan')))
    if 'truth_edge_x_range' in cb:
        print('      真值边沿范围 x=%s y=%s；图上的占用格范围 x=%s y=%s（对不上的那一段就是"图上看不见"）'
              % (cb['truth_edge_x_range'], cb['truth_edge_y_range'],
                 cb.get('occupied_x_range'), cb.get('occupied_y_range')))
    if 'nearest_occupied_to_edge_m' in cb:
        print('      最近的占用格到"真值边沿格"的距离 = %.3f m（≤%.2f m 才算表示出来）'
              % (cb['nearest_occupied_to_edge_m'], a.represent_radius))
    dr = rep['drivable']
    print('  【可行驶斜面不许被标占用】free %.2f%% / occupied %.2f%% / unknown %.2f%%（判据：occupied ≤ 5%%）'
          % (dr['free_pct'], dr['occupied_pct'], dr['unknown_pct']))
    print('  【背景量】图上判自由而真值 >0.10/0.20/0.30 m 的格 = %d / %d / %d（台面内部合法为 free，只看边沿）'
          % (rep['free_but_high']['0.10'], rep['free_but_high']['0.20'],
             rep['free_but_high']['0.30']))
    for k, v in rep['paths'].items():
        if not v.get('reachable'):
            print('  【路径 余量≥%s m】❌ 规划不出来（起点或目标不可达）' % k)
        else:
            print('  【路径 余量≥%s m】长 %.2f m；到撞击点最近 %.3f m；落入撞击窗 %d 格；'
                  '路径上真值高度 %.3f~%.3f m；沿途最大局部台阶 %.3f m（>%.2f m 的格占 %.1f%%）'
                  % (k, v['length_m'], v['min_dist_to_collision_m'],
                     v['cells_inside_collision_box'], v['path_min_truth_height_m'],
                     v['path_max_truth_height_m'], v['path_max_local_step_m'],
                     a.edge_threshold, v['path_pct_with_local_step_gt_edge']))

    # ---- 结论（粗判，给人看的）----
    ok = (ec['field_pct'] >= 80.0 and (cb['truth_edge_cells'] == 0 or
                                       cb['truth_edge_covered'] > 0) and dr['occupied_pct'] <= 5.0)
    print('  ==> %s' % ('✅ 台阶/边沿已表示且可行驶斜面未被误标' if ok else
                        '⚠️ 未达标：看上面三行（边沿覆盖率 / 撞击点 / 可行驶斜面被标占用）'))
    rep['verdict_ok'] = bool(ok)

    if a.json:
        with open(a.json, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
        print('  → %s' % a.json)
    if a.png:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(16, 7))
            ext = [ox, ox + W * res, oy, oy + H * res]
            ax[0].imshow(np.where(occ, 0, np.where(known, 254, 205)).T, origin='lower',
                         extent=ext, cmap='gray_r', vmin=0, vmax=255)
            ax[0].set_title('%s: prior map (0=occ)' % a.label)
            ax[1].imshow(np.where(has, rel, np.nan).T, origin='lower', extent=ext,
                         cmap='terrain', vmin=-0.1, vmax=0.6)
            ax[1].contour(np.where(edge, 1.0, np.nan).T, levels=[0.5], colors='r',
                          extent=ext, linewidths=0.5)
            ax[1].set_title('%s: STL truth height + red = step edge' % a.label)
            for k, v in rep['paths'].items():
                if v.get('reachable'):
                    ax[0].plot(v['x'], v['y'], lw=1.0, label='clr>=%s' % k)
            ax[0].legend(fontsize=7)
            ax[0].plot([bx], [by], 'r*', ms=12)
            plt.tight_layout()
            plt.savefig(a.png, dpi=110)
            print('  → %s' % a.png)
        except Exception as e:  # noqa: BLE001
            print('  [warn] PNG 没画成：%s' % e)
    return 0


if __name__ == '__main__':
    sys.exit(main())
