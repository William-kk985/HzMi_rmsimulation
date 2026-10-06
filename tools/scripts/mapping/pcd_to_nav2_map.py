#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcd_to_nav2_map.py —— 3D 点云 ⇒ nav2 能直接吃的 2D 栅格图（pgm + yaml）。

## 为什么要有这个工具（用户的决定）

原来的 2D 先验图是"**从 `/scan` 累积**"来的，而 `/scan` 是
`linefit 地面分割 → pointcloud_to_laserscan（传感器系窄高度带）`的产物：
**坡度够缓的斜坡会被地面分割判成地面 ⇒ 从 `/scan` 里彻底消失**，
于是"3D 点云里看得见的坡道，在 2D 图上是空白的"（甚至被当成可自由通行）。
本工具把 2D 图改成**直接从 3D 点云投影**：2D 与 3D 先验同源，
坡道按"相对**局部地面**的高度"判定 ⇒ **坡度小于阈值的可行驶斜面保持 FREE**。

## 判据（全部可配）

对每个 0.05 m 栅格：

```
局部地面 g(x,y)   = 以 (x,y) 为中心、边长 --ground-cell 的粗格里 z 的 --ground-percentile 分位
                    （默认：0.20 m 粗格 / p05）——"我脚下这块地大概多高"
                    ⇒ 竖直墙面格子的"地面"自然是它**自己最低的那些点**（≈ 楼板），
                      而不是墙面更高处 ⇒ 墙不会被自己的点"抬"成地面。
                    ⇒ 一段 24° 的坡：一个 0.2 m 粗格内的高度跨度只有 0.2*tan24° = 0.09 m。
占 用 ⟺ ① z_max(格内) − g > --height-threshold（默认 0.15 m，与 STL 管线同口径）
        或 ② 局部表面坡度 > --slope-limit（默认 25°）且 z_max − g > --slope-min-height
           （默认 0.05 m）—— 防"又陡又高"的台阶沿被算成缓坡
空 闲 ⟺ 有数据且不满足上面两条
未 知 ⟺ 该格没有数据、且局部地面也估计不出来（离任何数据都超过 --ground-fill-cells 粗格）
```

**为什么 ② 需要"且 z_max−g > slope-min-height"**：只看坡度会把**坡道本身**判成障碍
（坡道 24° 就超过很多"安全坡度"阈值）。加了这个高度闸，24° 的坡（每 0.05 m 只升 0.02 m）
不会被误判，而竖直台阶沿（0.05 m 内升 0.2 m）会被判成占用。

**空格填充**：点云通常是 0.10 m 体素下采样过的，而栅格是 0.05 m ⇒ 必然隔格空。
默认 `--fill-empty ground`：**能估出局部地面的空格算 FREE**（否则图会变成"虚线"，
nav2 会把它当未知区挡路）；`--fill-empty unknown` 是严格口径（"没数据就是未知"）。
两种口径的面积都会打印出来。占用格会按 `--occ-dilate-cells`（默认 1 格 = 0.05 m）
向**空格**膨胀一次，保证薄墙不断线（不会覆盖已经判成 FREE 的格）。

## 坐标约定（**和本仓既有资产对齐，不改任何约定**）

* 输入点云的 xy 已经是 **map 系 = 出生点相对系**（本仓 `PCD/<world>.pcd` 与
  `PCD/<world>_spl.pcd` 都是这个口径，两份的 bbox 直接可比）。
  若你的点云是 **world 系**（场地几何系），用
  `--world-to-map-shift -10.925 -2.525`（RMUC2026 的出生点）搬过来。
* 输出 yaml 的 `origin` = 栅格左下角在 **map 系**里的坐标
  ⇒ `amcl initial_pose=(0,0)`、gicp `initial_pose=[0,0,0]` 这些约定**不用改**
  （它们说的是"出生点在 map 系里的坐标"，与本工具选的栅格窗口无关）。
* 像素值沿用本仓既有图（`map_saver_cli`）的口径：`0=占用 / 205=未知 / 254=空闲`，
  `mode: trinary`、`free_thresh 0.25`、`occupied_thresh 0.65`。

## 判据的**单一真源**（★ 2026-10-07）

占用/空闲的四个阈值不再写在本脚本里，而是读
`src/rm_nav_bringup/config/traversability_criteria.yaml` —— **同一份文件**也被
`bringup_sim.launch.py` 当参数文件喂给两个地面分割节点（`ground:=linefit` / `ground:=patchwork`），
经由 `rm_ground_traversability/low_terrain_classifier.hpp` 在**实时链路**上执行同一套判据。

```
YAML 键                     → 本脚本的 CLI                → 实时链路（地面分割节点）
step_height_threshold       --height-threshold            step_height_threshold
drivable_slope_deg          --slope-limit                 drivable_slope_deg
slope_min_height            --slope-min-height            slope_min_height
ground_cell_m               --ground-cell                 ground_cell_m
fine_cell_m                 --resolution                  fine_cell_m
ground_percentile           --ground-percentile           ground_percentile
ground_min_points           --ground-min-points           ground_min_points
```

⇒ **实时代价图与先验图口径一致**（这正是缺陷 ③ 的修法：以前只有离线这一侧有坡度/台阶判据）。
改阈值只改那份 YAML；CLI 仍可临时覆盖（A/B 用）。一致性由
`tools/scripts/regress/check_traversability_criteria.py` 断言。

## 用法

    # 最常用：把真实建图点云投成 nav2 图（窗口自动取数据 bbox + 0.5 m 外扩）
    python3 tools/scripts/mapping/pcd_to_nav2_map.py \
        --pcd src/rm_nav_bringup/PCD/RMUC2026_spl.pcd \
        --out src/rm_nav_bringup/map/RMUC2026_cloud \
        --png .tmp_cache/hzmap2d/RMUC2026_cloud.png \
        --report-box ramp -7.9 -8.4 -3.6 -3.6 --report-box corridor -9.0 -8.4 -3.6 -7.2 \
        --json .tmp_cache/hzmap2d/RMUC2026_cloud.json

    # 只算不写（看数字）
    python3 tools/scripts/mapping/pcd_to_nav2_map.py --pcd x.pcd --out x --dry-run

`--help` 有全部参数；`--seed` 固定（`--max-points` 抽样时用）。退出码 0 正常。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pcd_stats import read_pcd_xyz  # noqa: E402  （同目录，复用 PCD 读取）

FREE_PX = 254      # 与 map_saver_cli 同口径
OCC_PX = 0
UNK_PX = 205

RMUC2026_SPAWN = (10.925, 2.525)   # world 系出生点；--world-to-map-shift 的默认建议值

# 判据真源（与 bringup_sim.launch.py / rm_ground_traversability 共用同一份文件）
CRITERIA_REL = os.path.join('src', 'rm_nav_bringup', 'config', 'traversability_criteria.yaml')
# YAML 键 → argparse 目标名（单一真源的映射表；两处实现共用同一批键名）
CRITERIA_KEYS = {
    'step_height_threshold': 'height_threshold',
    'drivable_slope_deg': 'slope_limit',
    'slope_min_height': 'slope_min_height',
    'ground_cell_m': 'ground_cell',
    'fine_cell_m': 'resolution',
    'ground_percentile': 'ground_percentile',
    'ground_min_points': 'ground_min_points',
}
# 读不到文件时的兜底（**必须**与 YAML / C++ Criteria 默认值逐键相同，见 check_traversability_criteria.py）
CRITERIA_FALLBACK = {
    'step_height_threshold': 0.15, 'drivable_slope_deg': 25.0, 'slope_min_height': 0.05,
    'ground_cell_m': 0.20, 'fine_cell_m': 0.05, 'ground_percentile': 5.0,
    'ground_min_points': 2,
}


def load_criteria(path, repo_root=None):
    """读判据 YAML（`/**:` 段下的 ros__parameters）⇒ (dict, 诊断字符串)。

    只认 CRITERIA_KEYS 里的键；缺键用 CRITERIA_FALLBACK 补齐并把缺的键报出来。
    故意**不**依赖 pyyaml（本脚本历史上零第三方依赖，只要 numpy）：用一个只认
    "缩进 + 键: 值" 的极小解析器 —— 判据文件的结构是扁平的，够用且可解释。
    """
    src = path
    if src is None:
        root = repo_root or os.getcwd()
        cand = [os.path.join(root, CRITERIA_REL)]
        here = os.path.dirname(os.path.abspath(__file__))
        cand.append(os.path.abspath(os.path.join(here, '..', '..', '..', CRITERIA_REL)))
        cand.append(os.path.abspath(os.path.join(here, '..', '..', '..', '..', CRITERIA_REL)))
        src = next((c for c in cand if os.path.isfile(c)), None)
    out = dict(CRITERIA_FALLBACK)
    if src is None or not os.path.isfile(src):
        return out, 'fallback(找不到 %s)' % CRITERIA_REL
    seen = set()
    with open(src) as f:
        for line in f:
            line = line.split('#')[0].rstrip()
            if not line or line.lstrip().startswith('/**') or 'ros__parameters' in line:
                continue
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            k = k.strip()
            if k not in CRITERIA_KEYS:
                continue
            v = v.strip()
            try:
                out[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
            except ValueError:
                continue
            seen.add(k)
    missing = sorted(set(CRITERIA_KEYS) - seen)
    diag = os.path.abspath(src) + ('(缺键，已用兜底：%s)' % ','.join(missing) if missing else '')
    return out, diag


# ------------------------------------------------------------------ 栅格工具
def _cell_index(xy, origin, res, shape):
    ij = np.floor((xy - origin) / res).astype(np.int64)
    ok = ((ij[:, 0] >= 0) & (ij[:, 1] >= 0)
          & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1]))
    return ij, ok


def _per_cell_percentile(z, a, b, shape, pct):
    """每格 z 的 pct 分位（无数据的格 = nan）。排序 + 格内秩，纯 numpy。"""
    n = len(z)
    lin = a * shape[1] + b
    order = np.lexsort((z, lin))
    l2 = lin[order]
    uniq, start = np.unique(l2, return_index=True)
    cnt = np.diff(np.append(start, n))
    rank = np.arange(n) - np.repeat(start, cnt)
    cnt_rep = np.repeat(cnt, cnt)
    # ★ 每格**恰好取一个**点：秩 == floor(pct/100*(cnt-1))。
    #   （别写成 `frac >= pct` 再散写：fancy-index 赋值是"最后一个赢"，
    #     那样拿到的是该格 z 的**最大值**而不是分位数 —— 这个坑踩过一次。）
    target = np.floor(pct / 100.0 * np.maximum(cnt_rep - 1, 0)).astype(np.int64)
    keep = (rank == target)
    out = np.full(shape, np.nan, dtype=np.float64)
    out[a[order][keep], b[order][keep]] = z[order][keep]
    counts = np.zeros(shape, dtype=np.int64)
    np.add.at(counts, (a, b), 1)
    return out, counts


def _nearest_fill(g, max_cells):
    """把已有值的格向外做切比雪夫距离 <= max_cells 的最近邻填充（保留原值）。"""
    filled = np.isfinite(g)
    if not filled.any():
        return g
    out = g.copy()
    if max_cells <= 0:
        return out
    # 迭代膨胀：每轮把"已知"的值复制到未知的 8 邻域
    cur = np.isfinite(out)
    for _ in range(int(max_cells)):
        new = cur.copy()
        vals = out.copy()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                sh = np.roll(np.roll(out, dx, axis=0), dy, axis=1)
                cv = np.roll(np.roll(cur, dx, axis=0), dy, axis=1)
                take = (~new) & cv
                vals[take] = sh[take]
                new |= take
        out, cur = vals, new
    return out


def _median3(g):
    """3x3 中值（nan 忽略；全 nan 保留 nan）。"""
    stack = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            stack.append(np.roll(np.roll(g, dx, axis=0), dy, axis=1))
    st = np.stack(stack, axis=0)
    with np.errstate(all='ignore'):
        return np.nanmedian(st, axis=0)


# ------------------------------------------------------------------ 主流程
def build(args):
    xyz, hdr = read_pcd_xyz(args.pcd)
    rep = {'pcd': os.path.abspath(args.pcd), 'pcd_fields': hdr.get('fields'),
           'points_read': int(len(xyz))}
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite].astype(np.float64)
    rep['points_finite'] = int(len(xyz))

    if args.world_to_map_shift != (0.0, 0.0):
        xyz[:, 0] -= args.world_to_map_shift[0]
        xyz[:, 1] -= args.world_to_map_shift[1]
        rep['world_to_map_shift'] = list(args.world_to_map_shift)
    if args.z_offset:
        xyz[:, 2] += args.z_offset

    if args.max_points and len(xyz) > args.max_points:
        rng = np.random.default_rng(args.seed)
        sel = rng.choice(len(xyz), size=args.max_points, replace=False)
        xyz = xyz[np.sort(sel)]
        rep['subsampled_to'] = int(len(xyz))

    # ---- 窗口
    if args.bbox:
        x0, y0, x1, y1 = args.bbox
    else:
        lo = np.percentile(xyz[:, :2], args.robust_bbox_percentile, axis=0)
        hi = np.percentile(xyz[:, :2], 100.0 - args.robust_bbox_percentile, axis=0)
        pad = args.pad
        x0 = float(np.floor((lo[0] - pad) / args.resolution) * args.resolution)
        y0 = float(np.floor((lo[1] - pad) / args.resolution) * args.resolution)
        x1 = float(np.ceil((hi[0] + pad) / args.resolution) * args.resolution)
        y1 = float(np.ceil((hi[1] + pad) / args.resolution) * args.resolution)
    res = args.resolution
    W = int(round((x1 - x0) / res))
    H = int(round((y1 - y0) / res))
    if W <= 0 or H <= 0:
        raise SystemExit('空窗口')
    rep['grid'] = {'width': W, 'height': H, 'resolution': res,
                   'origin': [round(float(x0), 4), round(float(y0), 4), 0.0],
                   'bbox': [round(float(x0), 4), round(float(y0), 4),
                            round(float(x1), 4), round(float(y1), 4)]}

    inside = ((xyz[:, 0] >= x0) & (xyz[:, 0] < x0 + W * res)
              & (xyz[:, 1] >= y0) & (xyz[:, 1] < y0 + H * res))
    rep['points_outside_window'] = int((~inside).sum())
    P = xyz[inside]
    if len(P) == 0:
        raise SystemExit('窗口里一个点都没有（--bbox 给错了？）')

    # ---- 细栅格：每格 z 的 min/max/点数
    ij, _ = _cell_index(P[:, :2], np.array([x0, y0]), res, (W, H))
    zmin = np.full((W, H), np.inf)
    zmax = np.full((W, H), -np.inf)
    np.minimum.at(zmin, (ij[:, 0], ij[:, 1]), P[:, 2])
    np.maximum.at(zmax, (ij[:, 0], ij[:, 1]), P[:, 2])
    cnt = np.zeros((W, H), dtype=np.int64)
    np.add.at(cnt, (ij[:, 0], ij[:, 1]), 1)
    has = cnt > 0
    zmin[~has] = np.nan
    zmax[~has] = np.nan

    # ---- 点级法向 ⇒ "可行驶面"点 ⇒ 粗格局部地面
    #
    # ★ 为什么要走"法向"这一步（第一版踩的坑，有实测数字）：
    #   直接把"粗格内 z 的 p05"当局部地面时，**墙自己会被当成地面**——
    #   墙脚那些点的 p05 就是墙的最低处，于是 dz = 墙顶 − 墙脚 ≈ 0 ⇒ 墙消失。
    #   实测：`/scan` 累积图里 1493 个占用格，有 1418 个在这张点云图上被判成"空闲"。
    #   修法：先给每个点估局部法向（k 近邻 PCA），**只有法向接近竖直（坡度 ≤ --slope-limit）
    #   的点才有资格当"可行驶面"**；局部地面只从这些点里取。于是
    #     · 坡道：面上每个点都合格 ⇒ 地面 = 坡面本身 ⇒ 缓坡保持 FREE（这正是我们要的）
    #     · 竖墙：墙面的点不合格 ⇒ 该粗格借**邻近候选格里最低的那个地面** ⇒ dz = 墙高 ⇒ 占用
    #     · 台阶/边沿：同理，边沿两侧的高度差被算成 dz ⇒ 占用
    try:
        from scipy.spatial import cKDTree
        from scipy import ndimage as _ndi
    except Exception as e:  # noqa: BLE001
        raise SystemExit('需要 scipy（法向估计 + 最小值滤波）：%s' % e)
    _gm = getattr(args, 'ground_mode', 'lowest')
    if _gm == 'lowest':
        # lowest 档不需要点法向（局部地面走"细格最低点"，坡度走栅格梯度）⇒ 整段 KD-tree 省掉。
        walk = np.zeros(len(P), dtype=bool)
        rep['normals'] = {'k': 0, 'walkable_points': -1, 'walkable_pct': -1.0,
                          'note': 'ground_mode=lowest ⇒ 不做点法向 PCA'}
    else:
        kq = int(min(args.normal_k + 1, len(P)))
        tree = cKDTree(P[:, :3])
        _, nidx = tree.query(P[:, :3], k=kq, workers=-1)
        nb = P[nidx]
        ctr = nb - nb.mean(axis=1, keepdims=True)
        cov = np.einsum('nki,nkj->nij', ctr, ctr, optimize=True) / max(kq, 1)
        _, vec = np.linalg.eigh(cov)
        nrm = vec[:, :, 0]
        nrm = np.where(nrm[:, 2:3] < 0, -nrm, nrm)
        pt_slope = np.degrees(np.arccos(np.clip(np.abs(nrm[:, 2]), 0.0, 1.0)))
        walk = pt_slope <= args.slope_limit
        rep['normals'] = {'k': kq, 'walkable_points': int(walk.sum()),
                          'walkable_pct': round(100.0 * walk.mean(), 1),
                          'pt_slope_deg': {'p50': round(float(np.percentile(pt_slope, 50)), 1),
                                           'p90': round(float(np.percentile(pt_slope, 90)), 1)}}

    gc = args.ground_cell if args.ground_cell > 0 else res * 4
    og = np.array([x0, y0])
    Wc = int(math.ceil(W * res / gc))
    Hc = int(math.ceil(H * res / gc))
    jc, _ = _cell_index(P[:, :2], og, gc, (Wc, Hc))
    g_cand = np.full((Wc, Hc), np.nan)
    gmode = getattr(args, 'ground_mode', 'lowest')
    if gmode == 'lowest':
        # ★ 2026-10-07（与实时判据统一）：局部地面 = 粗格内**各 0.05 m 细格最低点**的 p05。
        #   与 rm_ground_traversability/low_terrain_classifier.hpp 的 groundOf() **逐条同构**
        #   （细格取最低点 ⇒ 竖直墙面格不会把自己抬成地面；不做法向闸）。
        #   为什么需要这一档：`normal` 档用"点法向 ≤ slope-limit"筛可行驶面，
        #   而**从没被开上去过的台面/坡面**在雷达里是掠射，k 近邻法向不可靠 ⇒ 合格点判不出来
        #   ⇒ 那里的"地面"会借到旁边的低处 ⇒ 整片 0.2~0.3 m 台面被判成"高出 0.2~0.3 m"的障碍，
        #   连目标点自己都变成占用格。实测（RMUC2026_lt 的 86 594 点 PCD）：
        #   normal 档 122.6 m² 占用、可行驶斜面里 30.3% 被标占用；lowest 档见 --json 的 cells。
        ij_f, _ = _cell_index(P[:, :2], np.array([x0, y0]), res, (W, H))
        zmin_map = np.full((W, H), np.inf)
        np.minimum.at(zmin_map, (ij_f[:, 0], ij_f[:, 1]), P[:, 2])
        has_f = np.isfinite(zmin_map)
        blk = int(round(gc / res)) if gc >= res else 1
        # 每个粗格 = blk×blk 个细格；取这些细格最低点的 p05（与实时实现同一条取秩公式）
        rr, cc = np.where(has_f)
        br = rr // blk
        bc = cc // blk
        order = np.lexsort((zmin_map[rr, cc], br * (Wc + 1) + bc))
        lin = (br * (Wc + 1) + bc)[order]
        zz = zmin_map[rr, cc][order]
        uniq, start = np.unique(lin, return_index=True)
        cnt = np.diff(np.append(start, len(lin)))
        rank = np.arange(len(lin)) - np.repeat(start, cnt)
        target = np.floor(args.ground_percentile / 100.0 *
                          np.maximum(np.repeat(cnt, cnt) - 1, 0)).astype(np.int64)
        keep = rank == target
        b_un = lin[keep] // (Wc + 1)
        c_un = lin[keep] % (Wc + 1)
        z_un = zz[keep]
        ok_pts = np.repeat(cnt, cnt)[keep] >= args.ground_min_points
        g_cand[b_un[ok_pts], c_un[ok_pts]] = z_un[ok_pts]
        rep['normals']['ground_mode'] = 'lowest(fine-cell minima p%d)' % args.ground_percentile
    else:
        Pw = P[walk]
        jw, _ = _cell_index(Pw[:, :2], og, gc, (Wc, Hc))
        if len(Pw):
            g_cand, wcnt = _per_cell_percentile(Pw[:, 2], jw[:, 0], jw[:, 1], (Wc, Hc),
                                                args.ground_percentile)
            g_cand[wcnt < args.ground_min_points] = np.nan
        rep['normals']['ground_mode'] = 'normal(可行驶面点的 p%d)' % args.ground_percentile
    finite_cand = np.isfinite(g_cand)
    # 借地面 = 半径 r 内候选格里**最低**的那个（min 滤波）⇒ 台阶沿一定借到低的那侧
    r = int(args.ground_borrow_cells)
    filled = np.where(finite_cand, g_cand, np.inf)
    if r > 0:
        filled = _ndi.minimum_filter(filled, size=2 * r + 1, mode='nearest')
    filled[~np.isfinite(filled)] = np.nan
    g_coarse = np.where(finite_cand, g_cand, filled)
    rep['ground'] = {'cell_m': gc, 'percentile': args.ground_percentile,
                     'min_points_per_cell': args.ground_min_points,
                     'borrow_cells': r,
                     'coarse_cells': int(Wc * Hc),
                     'coarse_cells_with_walkable_data': int(finite_cand.sum()),
                     'coarse_cells_borrowed': int((~finite_cand & np.isfinite(g_coarse)).sum()),
                     'coarse_cells_no_ground': int((~np.isfinite(g_coarse)).sum())}
    g_cov_any = np.isfinite(g_coarse)
    g_coarse = _nearest_fill(g_coarse, args.ground_fill_cells)
    if args.ground_median:
        g_coarse = _median3(g_coarse)
    rep['ground']['coarse_cells_after_fill'] = int(np.isfinite(g_coarse).sum())

    # 粗格地面 → 细格（最近邻放大）
    fi = np.clip((np.arange(W) * res) // gc, 0, Wc - 1).astype(np.int64)
    fj = np.clip((np.arange(H) * res) // gc, 0, Hc - 1).astype(np.int64)
    g_fine = g_coarse[np.ix_(fi, fj)]

    # ---- 细格"走行面"坡度（用 z_min 组成的面；空格用局部地面补）
    surf = np.where(has, zmin, g_fine)
    surf = np.where(np.isfinite(surf), surf, np.nan)
    if args.surface_median:
        surf = _median3(surf)
    surf_f = np.where(np.isfinite(surf), surf, 0.0)
    gy, gx = np.gradient(surf_f, res)
    with np.errstate(all='ignore'):
        slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    slope[~np.isfinite(surf)] = np.nan

    # ---- 判定
    dz = zmax - g_fine
    occ_h = has & (dz > args.height_threshold)
    occ_s = has & (slope > args.slope_limit) & (dz > args.slope_min_height)
    occupied = occ_h | occ_s
    free = has & ~occupied

    # 占用去斑：孤立/近孤立的占用格基本是点云噪声（0.10 m 体素下采样 + LIO 残差），
    # 留着会在空闲区里造出"假障碍"。线状墙的每个格沿墙方向有 2 个邻居 ⇒ 采 2 不伤墙。
    despeckled = 0
    if args.occ_min_neighbors > 0:
        nb = np.zeros(occupied.shape, dtype=np.int64)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nb += np.roll(np.roll(occupied, dx, axis=0), dy, axis=1).astype(np.int64)
        keep = occupied & (nb >= args.occ_min_neighbors)
        despeckled = int((occupied & ~keep).sum())
        occupied = keep

    # 占用向"空格"膨胀（薄墙补线；不覆盖 FREE）
    dil = int(args.occ_dilate_cells)
    if dil > 0:
        occ_d = occupied.copy()
        cur = occupied.copy()
        for _ in range(dil):
            nxt = cur.copy()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nxt |= np.roll(np.roll(cur, dx, axis=0), dy, axis=1)
            cur = nxt
        occupied = occ_d | (cur & ~free)
        free = has & ~occupied

    # 空格：能估出局部地面 ⇒ FREE（默认）或 UNKNOWN（严格口径）
    empty = ~has
    ground_known = empty & np.isfinite(g_fine)
    if args.fill_empty == 'ground':
        free = free | ground_known
    unknown = empty & ~free

    if args.border == 'occupied':
        unknown = np.zeros_like(unknown)   # 把"未知"并入占用（当围墙）
        occupied = occupied | unknown

    # ---- 统计
    cell_a = res * res
    rep['cells'] = {
        'free': int(free.sum()), 'occupied': int(occupied.sum()),
        'unknown': int(unknown.sum()),
        'free_m2': round(float(free.sum() * cell_a), 2),
        'occupied_m2': round(float(occupied.sum() * cell_a), 2),
        'unknown_m2': round(float(unknown.sum() * cell_a), 2),
        'occupied_from_height': int(occ_h.sum()),
        'occupied_from_slope': int((occ_s & ~occ_h).sum()),
        'empty_filled_from_ground': int(ground_known.sum()),
        'occupied_despeckled': int(despeckled),
    }
    rep['params'] = {
        'ground_mode': getattr(args, 'ground_mode', 'lowest'),
        'resolution': res, 'height_threshold': args.height_threshold,
        'slope_limit_deg': args.slope_limit, 'slope_min_height': args.slope_min_height,
        'fill_empty': args.fill_empty, 'border': args.border,
        'occ_dilate_cells': dil, 'occ_min_neighbors': args.occ_min_neighbors,
        'seed': args.seed,
        # ★ 判据来源（单一真源）：与实时链路（rm_ground_traversability）共用同一份 YAML
        'criteria_source': getattr(args, 'criteria_diag', ''),
        'criteria_file': os.path.abspath(args.criteria_file) if args.criteria_file else None,
    }

    # ---- 坡道段分类（--report-box；以及自动找"缓坡区"）
    rep['report_boxes'] = []
    for name, bx0, by0, bx1, by1 in args.report_box:
        i0 = max(0, int(math.floor((bx0 - x0) / res)))
        i1 = min(W, int(math.ceil((bx1 - x0) / res)))
        j0 = max(0, int(math.floor((by0 - y0) / res)))
        j1 = min(H, int(math.ceil((by1 - y0) / res)))
        sub = np.s_[i0:i1, j0:j1]
        f, o, u = int(free[sub].sum()), int(occupied[sub].sum()), int(unknown[sub].sum())
        sl = slope[sub]
        z = zmax[sub]
        zz = z[np.isfinite(z)]
        rep['report_boxes'].append({
            'name': name, 'bbox': [bx0, by0, bx1, by1],
            'cells': f + o + u, 'free': f, 'occupied': o, 'unknown': u,
            'free_m2': round(f * cell_a, 3), 'occupied_m2': round(o * cell_a, 3),
            'unknown_m2': round(u * cell_a, 3),
            'free_pct': round(100.0 * f / max(f + o + u, 1), 1),
            'slope_deg_median': round(float(np.nanmedian(sl)), 1) if np.isfinite(sl).any() else None,
            'slope_deg_p90': round(float(np.nanpercentile(sl, 90)), 1) if np.isfinite(sl).any() else None,
            'z_min': round(float(zz.min()), 3) if zz.size else None,
            'z_max': round(float(zz.max()), 3) if zz.size else None,
        })

    # 自动：坡度在 [5°, slope_limit] 的"可行驶斜面"总面积（坡道的量化指标）
    drivable_slope = np.isfinite(slope) & (slope > 5.0) & (slope <= args.slope_limit) & has
    rep['drivable_slope'] = {'cells': int(drivable_slope.sum()),
                             'area_m2': round(float(drivable_slope.sum() * cell_a), 2),
                             'of_which_free': int((drivable_slope & free).sum()),
                             'of_which_occupied': int((drivable_slope & occupied).sum())}
    # 自动：又高又陡 ⇒ 台阶/墙
    step = has & (dz > args.height_threshold)
    rep['steps'] = {'cells': int(step.sum()), 'area_m2': round(float(step.sum() * cell_a), 2)}

    img = np.full((W, H), UNK_PX, dtype=np.uint8)   # 内部一律 (W, H) = (x, y)，写出时再转
    img[free] = FREE_PX
    img[occupied] = OCC_PX
    return img, rep, (free, occupied, unknown, slope, dz)


def write_pgm_yaml(img, out_prefix, res, origin, name=None):
    W, H = img.shape
    pgm = out_prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(np.flipud(img.T).tobytes())    # pgm 行序 = 从上到下 ⇒ 图上边 = y 最大
    yml = out_prefix + '.yaml'
    with open(yml, 'w') as f:
        f.write('image: %s\n' % os.path.basename(pgm if not name else name + '.pgm'))
        f.write('mode: trinary\n')
        f.write('resolution: %g\n' % res)
        f.write('origin: [%g, %g, 0]\n' % (round(origin[0], 6), round(origin[1], 6)))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm, yml


def write_png(img, path, res, origin, boxes=()):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print('[warn] 没有 matplotlib，跳过 PNG：%s' % e)
        return None
    W, H = img.shape
    ext = [origin[0], origin[0] + W * res, origin[1], origin[1] + H * res]
    fig, ax = plt.subplots(figsize=(max(6, W * res / 3.0), max(4, H * res / 3.0)), dpi=120)
    ax.imshow(img.T, origin='lower', extent=ext, cmap='gray_r', vmin=0, vmax=255,
              interpolation='nearest')
    for (nm, bx0, by0, bx1, by1) in boxes:
        ax.add_patch(plt.Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fill=False,
                                   edgecolor='red', lw=1.2))
        ax.text(bx0, by1, nm, color='red', fontsize=8, va='bottom')
    ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    ax.set_title('pcd_to_nav2_map: 0=occupied 205=unknown 254=free')
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(path)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(
        description='3D 点云 ⇒ nav2 2D 栅格图（pgm+yaml）：按"相对局部地面高度"判占用，缓坡保持 FREE',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('## 用法')[-1])
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out', required=True, help='输出前缀（写 <out>.pgm 与 <out>.yaml）')
    ap.add_argument('--criteria-file', default=None,
                    help='判据真源 YAML（默认 src/rm_nav_bringup/config/traversability_criteria.yaml）；'
                         '--height-threshold / --slope-limit / --slope-min-height / --ground-cell / '
                         '--resolution / --ground-percentile / --ground-min-points 的**默认值**来自它')
    # 先解析 --criteria-file，再把它填进下面这些键的默认值（单一真源；CLI 仍可临时覆盖）
    _pre, _ = ap.parse_known_args()
    _crit, _crit_diag = load_criteria(_pre.criteria_file)
    ap.set_defaults(criteria_diag=_crit_diag)
    ap.add_argument('--resolution', type=float, default=_crit['fine_cell_m'])
    ap.add_argument('--height-threshold', type=float, default=_crit['step_height_threshold'],
                    help='相对局部地面的高度超过它 = 占用（m，默认来自判据真源 YAML）')
    ap.add_argument('--slope-limit', type=float, default=_crit['drivable_slope_deg'],
                    help='可行驶坡度上限（deg，默认来自判据真源 YAML）；超过它的"又陡又高"面算占用')
    ap.add_argument('--slope-min-height', type=float, default=_crit['slope_min_height'],
                    help='坡度判据的高度闸（m，默认来自真源 YAML）：低于它不因坡度判占用（保住坡道本身）')
    ap.add_argument('--ground-cell', type=float, default=_crit['ground_cell_m'],
                    help='估计"局部地面"的粗格边长（m，默认来自真源 YAML）；0 = 4*resolution')
    ap.add_argument('--ground-percentile', type=float, default=_crit['ground_percentile'],
                    help='粗格内取 z 的哪个分位当局部地面（默认 p05，来自真源 YAML）')
    ap.add_argument('--ground-min-points', type=int, default=_crit['ground_min_points'],
                    help='粗格至少几个点才认为能估出地面（默认来自真源 YAML）')
    ap.add_argument('--ground-mode', choices=['lowest', 'normal'], default='lowest',
                    help='局部地面怎么估：lowest（默认）= 粗格内各 0.05 m 细格"最低点"的 p05，'
                         '与实时判据 rm_ground_traversability 同构；normal = 只用"法向 ≤ slope-limit"的'
                         '可行驶面点（老口径，在"没被开上去过的台面/掠射坡面"上会把地面借低、整片判占用）')
    ap.add_argument('--normal-k', type=int, default=8,
                    help='点法向 PCA 的近邻数（默认 8）；法向决定"哪些点算可行驶面"')
    ap.add_argument('--ground-borrow-cells', type=int, default=1,
                    help='"借地面"的搜索半径（粗格数，默认 10 = 2 m @0.2 m 格）；借到的是半径内**最低**的候选地面')
    ap.add_argument('--ground-fill-cells', type=int, default=3,
                    help='地面外推的最大粗格距离（默认 3 格 = 0.6 m）；超出 = 未知')
    ap.add_argument('--ground-median', type=int, default=1, help='粗格地面 3x3 中值滤波（0/1）')
    ap.add_argument('--surface-median', type=int, default=1, help='走行面 3x3 中值滤波（0/1）')
    ap.add_argument('--occ-min-neighbors', type=int, default=1,
                    help='占用去斑：占用格的 8 邻域里至少要有这么多占用格才保留（0=不去斑，默认 2）')
    ap.add_argument('--occ-dilate-cells', type=int, default=1,
                    help='占用格向外膨胀的格数（只进"空格"，默认 1）')
    ap.add_argument('--fill-empty', choices=['ground', 'unknown'], default='ground',
                    help='空格口径：ground=能估出地面就算 free（默认，避免虚线图）；unknown=严格"没数据就未知"')
    ap.add_argument('--border', choices=['unknown', 'occupied'], default='unknown',
                    help='窗口内"无数据"格的口径（默认 unknown；STL 管线那种"外沿当围墙"选 occupied）')
    ap.add_argument('--bbox', nargs=4, type=float, metavar=('X0', 'Y0', 'X1', 'Y1'),
                    help='显式窗口（map 系，m）；不给则用点云 bbox + --pad')
    ap.add_argument('--pad', type=float, default=0.5, help='自动窗口的外扩（m，默认 0.5）')
    ap.add_argument('--robust-bbox-percentile', type=float, default=0.2,
                    help='自动窗口前先按这个百分位砍掉离群点（默认 0.2%%，两侧）')
    ap.add_argument('--world-to-map-shift', nargs=2, type=float, default=(0.0, 0.0),
                    metavar=('DX', 'DY'),
                    help='把 world 系点云搬到 map 系要减掉的出生点坐标（RMUC2026 = %.3f %.3f）'
                         % RMUC2026_SPAWN)
    ap.add_argument('--z-offset', type=float, default=0.0)
    ap.add_argument('--max-points', type=int, default=0, help='随机下采样上限（0=不采样）')
    ap.add_argument('--seed', type=int, default=0, help='--max-points 的随机种子（默认 0，固定）')
    ap.add_argument('--report-box', nargs=5, action='append', default=[],
                    metavar=('NAME', 'X0', 'Y0', 'X1', 'Y1'),
                    help='指定区域单独统计（可重复），例如 --report-box ramp -7.9 -8.4 -3.6 -3.6')
    ap.add_argument('--png', help='额外画一张俯视图 PNG')
    ap.add_argument('--json', help='把 manifest 写到这个 JSON')
    ap.add_argument('--dry-run', action='store_true', help='只算不写 pgm/yaml')
    args = ap.parse_args()
    args.report_box = [(b[0], float(b[1]), float(b[2]), float(b[3]), float(b[4]))
                       for b in args.report_box]

    img, rep, _ = build(args)
    if not args.dry_run:
        pgm, yml = write_pgm_yaml(img, args.out, args.resolution, rep['grid']['origin'])
        rep['wrote'] = {'pgm': os.path.abspath(pgm), 'yaml': os.path.abspath(yml),
                        'pgm_bytes': os.path.getsize(pgm)}
    if args.png:
        p = write_png(img, args.png, args.resolution, rep['grid']['origin'],
                      boxes=[(b['name'], b['bbox'][0], b['bbox'][1], b['bbox'][2], b['bbox'][3])
                             for b in rep['report_boxes']])
        rep['png'] = p
    print('PCD2MAP ' + json.dumps(rep, ensure_ascii=False))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
