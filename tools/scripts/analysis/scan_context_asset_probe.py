#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_context_asset_probe.py —— A1 资产可行性探针（对应 docs/scan_context_plan.md §D.1）。

**只读**：只用 numpy 解析 PCD（二进制/ascii），不写任何文件（`--json`/`--render-png` 也只允许落在
仓库的 `.tmp_cache/` 或 `.tmp_research/` 下）。不依赖 ROS / open3d / matplotlib / scipy。

回答两个问题（在写任何检索器之前必须先回答）：

  A1.2/A1.3  先验 PCD 里到底有没有**连成线的墙**？
      · 2D 投影（高度带内点 → xy 格占用）后的：占用格数、墙长估计（格数法 / 细化骨架法）、
        连通域数（4/8 邻域）、从质心做角度扫掠的**最大缺口**、包围盒周长（场地周长代理）、
        以及"闭合性"（从格图边界洪泛能否漏进内部）。
  A1.4       2D 极坐标距离描述子**能不能区分地点**？
      · 在地图上取若干**空旷且分散**的查询位姿，用同一份 PCD 当"扫描"射线投射生成
        360°/r_max/`--bins` 的距离描述子，量：
          (a) **自距离**：同一地点做小扰动（±0.5 格 xy、±2° yaw）重建描述子后的距离分布；
          (b) **互距离**：不同地点之间的描述子距离分布（含/不含 yaw 圆周搜索两版）；
          给出两者的 min/中位/max、分离裕度、是否存在干净阈值（含最佳阈值与准确率）；
          以及 **yaw 鲁棒性**：同一地点纯 yaw 偏移 0…`--yaw-scan-max` 度时的距离曲线
          （⇒ 粗 yaw 搜索要几步）。

判据（来自 plan §D.1，实测后按数据重定）：
  · A1.3 墙长 ≥ 场地周长的一半（场地 12.45×8.65 m ⇒ 周长 ≈42.2 m ⇒ **≥ ~21 m**）；
  · A1.4 自距离 ≪ 互距离（建议中位比 **≥ 5×**），且存在干净分离阈值。

用法：
  # 主资产 + 对照组（一条命令跑两份 PCD）
  python3 tools/scripts/analysis/scan_context_asset_probe.py \\
      --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd src/rm_nav_bringup/PCD/RMUL.pcd

  # 只跑一份、改参数、落 JSON 到 .tmp_cache
  python3 tools/scripts/analysis/scan_context_asset_probe.py \\
      --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd --bins 360 --height-band 0.20,0.35 \\
      --json .tmp_cache/sc_probe.json --render-png .tmp_cache/sc_probe.png

注意：`--height-band` 的上下界都是**地面以上**的高度（自动地面 = z 的 5% 分位，与 plan §A.0 同法），
`inf` 可写 `inf`/`1e9`/`-`（见 `--help`）。
"""

import argparse
import json
import math
import os
import struct
import sys
import time
import zlib
from collections import deque

import numpy as np

# --------------------------------------------------------------------------------------
# 常量 / 路径
# --------------------------------------------------------------------------------------
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
TMP_ALLOWED = (os.path.join(REPO_ROOT, ".tmp_cache"), os.path.join(REPO_ROOT, ".tmp_research"))
INF = float("inf")


def log(msg):
    print(msg, flush=True)


def die(msg, code=2):
    print(f"scan_context_asset_probe: ERROR: {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


# --------------------------------------------------------------------------------------
# PCD 读取（numpy only）
# --------------------------------------------------------------------------------------
_NP_TYPE = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "i1", ("I", 2): "<i2",
            ("I", 4): "<i4", ("I", 8): "<i8", ("U", 1): "u1", ("U", 2): "<u2",
            ("U", 4): "<u4", ("U", 8): "<u8"}


def read_pcd(path):
    """读 PCD，返回 (fields, xyz(float64 Nx3), extra(dict name->array))。只支持 ascii/binary。"""
    if not os.path.isfile(path):
        die(f"PCD not found: {path}")
    with open(path, "rb") as fh:
        header_lines = []
        while True:
            raw = fh.readline()
            if not raw:
                die(f"{path}: header 里没有 DATA 行")
            line = raw.decode("ascii", errors="replace").strip()
            header_lines.append(line)
            if line.upper().startswith("DATA"):
                break
        data_off = fh.tell()

    fields, sizes, types, counts = [], [], [], []
    n_points, data_kind = None, None
    for line in header_lines:
        tok = line.split()
        if not tok:
            continue
        key = tok[0].upper()
        if key == "FIELDS":
            fields = tok[1:]
        elif key == "SIZE":
            sizes = [int(t) for t in tok[1:]]
        elif key == "TYPE":
            types = [t.upper() for t in tok[1:]]
        elif key == "COUNT":
            counts = [int(t) for t in tok[1:]]
        elif key == "POINTS":
            n_points = int(tok[1])
        elif key == "WIDTH":
            if n_points is None:
                n_points = int(tok[1])
        elif key == "DATA":
            data_kind = tok[1].lower() if len(tok) > 1 else "ascii"

    if not fields or len(fields) != len(sizes) or len(fields) != len(types):
        die(f"{path}: FIELDS/SIZE/TYPE 不一致")
    if not counts:
        counts = [1] * len(fields)
    if data_kind == "binary_compressed":
        die(f"{path}: DATA binary_compressed 暂不支持（本仓资产都是 binary）")

    if data_kind == "ascii":
        arr = np.loadtxt(path, dtype=np.float64, skiprows=len(header_lines), ndmin=2)
        if arr.shape[1] < len(fields):
            die(f"{path}: ascii 列数 {arr.shape[1]} < FIELDS {len(fields)}")
        cols = {name: arr[:, i] for i, name in enumerate(fields)}
    else:
        names, formats, offsets, off = [], [], [], 0
        for name, sz, ty, cnt in zip(fields, sizes, types, counts):
            dt = _NP_TYPE.get((ty, sz))
            if dt is None:
                die(f"{path}: 不支持的字段类型 TYPE={ty} SIZE={sz} ({name})")
            names.append(name)
            formats.append(dt if cnt == 1 else (dt, (cnt,)))
            offsets.append(off)
            off += sz * cnt
        point_step = off
        dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})
        size = os.path.getsize(path) - data_off
        if n_points is None:
            n_points = size // point_step if point_step else 0
        avail = size // point_step if point_step else 0
        if avail < n_points:
            log(f"  [!] {path}: 头里 POINTS={n_points} 但文件只够 {avail} 点，按 {avail} 读")
            n_points = avail
        with open(path, "rb") as fh:
            fh.seek(data_off)
            buf = fh.read(n_points * point_step)
        rec = np.frombuffer(buf, dtype=dtype, count=n_points)
        cols = {name: np.asarray(rec[name], dtype=np.float64) for name in fields}

    for need in ("x", "y", "z"):
        if need not in cols:
            die(f"{path}: 缺少字段 {need}（有 {list(cols)}）")
    xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1)
    finite = np.isfinite(xyz).all(axis=1)
    n_bad = int((~finite).sum())
    xyz = xyz[finite]
    extra = {k: v[finite] for k, v in cols.items() if k not in ("x", "y", "z")}
    return {"fields": fields, "n_points_header": n_points, "n_bad": n_bad, "xyz": xyz, "extra": extra}


# --------------------------------------------------------------------------------------
# 基础几何 / 格图工具
# --------------------------------------------------------------------------------------
def grid_of(xy, cell):
    """xy(N,2) → (idx(N,2) 为 (行=y, 列=x) 的整数格号, origin(2,) 为 (x0,y0), shape=(ny,nx))。

    ⚠️ 统一约定：格图第一维 = y（行），第二维 = x（列）—— 与 raycast/PNG/ASCII 的 [gy, gx] 索引一致。
    """
    lo = xy.min(axis=0)
    origin = lo - 0.5 * cell  # 让第 0 格中心落在 lo 上
    ixy = np.floor((xy - origin) / cell).astype(np.int64)   # (N,2) = (ix, iy)
    idx = ixy[:, ::-1].copy()                               # (N,2) = (iy, ix)
    shape = idx.max(axis=0) + 1                             # (ny, nx)
    return idx, origin, shape


def rc_to_xy(rc, origin, cell):
    """(行=y, 列=x) 格号 → xy 世界坐标（格中心）。"""
    rc = np.atleast_2d(np.asarray(rc, dtype=np.float64))
    return origin[None, :] + (rc[:, ::-1] + 0.5) * cell


def occ_from_idx(idx, shape):
    occ = np.zeros((int(shape[0]), int(shape[1])), dtype=bool)
    occ[idx[:, 0], idx[:, 1]] = True
    return occ


def pad_grid(occ, margin_cells):
    return np.pad(occ, int(margin_cells), mode="constant", constant_values=False)


def label_components(occ, connectivity=8):
    """纯 numpy/python BFS 连通域标记。返回 (n_comp, labels(int32), sizes(list, 降序))."""
    n0, n1 = occ.shape
    labels = np.zeros((n0, n1), dtype=np.int32)
    if connectivity == 8:
        offs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    else:
        offs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    cells = np.argwhere(occ)
    n_comp = 0
    sizes = []
    for start in cells:
        r0, c0 = int(start[0]), int(start[1])
        if labels[r0, c0]:
            continue
        n_comp += 1
        size = 0
        q = deque([(r0, c0)])
        labels[r0, c0] = n_comp
        while q:
            r, c = q.popleft()
            size += 1
            for dr, dc in offs:
                rr, cc = r + dr, c + dc
                if 0 <= rr < n0 and 0 <= cc < n1 and occ[rr, cc] and not labels[rr, cc]:
                    labels[rr, cc] = n_comp
                    q.append((rr, cc))
        sizes.append(size)
    order = np.argsort(sizes)[::-1]
    relabel = np.zeros(n_comp + 1, dtype=np.int32)
    for new_id, old_id in enumerate(order, start=1):
        relabel[old_id + 1] = new_id
    return n_comp, relabel[labels], [int(sizes[i]) for i in order]


def zhang_suen(occ, max_iter=200):
    """Zhang-Suen 细化（numpy 向量化）。返回骨架 bool 图。"""
    img = occ.copy()
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1, mode="constant", constant_values=False)
            p2 = p[0:-2, 1:-1]
            p3 = p[0:-2, 2:]
            p4 = p[1:-1, 2:]
            p5 = p[2:, 2:]
            p6 = p[2:, 1:-1]
            p7 = p[2:, 0:-2]
            p8 = p[1:-1, 0:-2]
            p9 = p[0:-2, 0:-2]
            nb = [p2, p3, p4, p5, p6, p7, p8, p9]
            b = np.zeros(img.shape, dtype=np.int16)
            for q in nb:
                b += q.astype(np.int16)
            seq = nb + [p2]
            a = np.zeros(img.shape, dtype=np.int16)
            for i in range(8):
                a += ((~seq[i]) & seq[i + 1]).astype(np.int16)
            if step == 0:
                cond = img & (b >= 2) & (b <= 6) & (a == 1) & ~(p2 & p4 & p6) & ~(p4 & p6 & p8)
            else:
                cond = img & (b >= 2) & (b <= 6) & (a == 1) & ~(p2 & p4 & p8) & ~(p2 & p6 & p8)
            if cond.any():
                img[cond] = False
                changed = True
        if not changed:
            break
    return img


def neighbor_stats(binary):
    """返回 (n_orth, n_diag, n_total) 每个前景格的邻居计数（8 邻域，不含自身）。"""
    p = np.pad(binary, 1, mode="constant", constant_values=False).astype(np.int16)
    orth = (p[0:-2, 1:-1] + p[2:, 1:-1] + p[1:-1, 0:-2] + p[1:-1, 2:])
    diag = (p[0:-2, 0:-2] + p[0:-2, 2:] + p[2:, 0:-2] + p[2:, 2:])
    return orth, diag, orth + diag


def geodesic_clearance(occ, cap_cells):
    """自由格到最近墙格的 4 邻域测地距离（单位：格），wall=0，cap 之外截断。"""
    dist = np.full(occ.shape, -1, dtype=np.int32)
    dist[occ] = 0
    frontier = occ.copy()
    d = 0
    while frontier.any() and d < int(cap_cells):
        d += 1
        new = np.zeros_like(frontier)
        new[1:, :] |= frontier[:-1, :]
        new[:-1, :] |= frontier[1:, :]
        new[:, 1:] |= frontier[:, :-1]
        new[:, :-1] |= frontier[:, 1:]
        new &= (dist < 0)
        if not new.any():
            break
        dist[new] = d
        frontier = new
    return dist


def raycast(occ, origin, cell, pose_xy, angles, max_radius, step_frac=0.5):
    """从 pose 沿 angles(rad, world 系) 射线投射。

    返回 (ranges, hits)：ranges 为首次命中距离（未命中 = max_radius），hits bool。
    采样步长 = cell*step_frac；越界视为自由（= 无回波）。
    """
    n_steps = max(1, int(math.ceil(max_radius / (cell * step_frac))))
    t = (np.arange(n_steps, dtype=np.float64) + 0.5) * (cell * step_frac)
    ca, sa = np.cos(angles), np.sin(angles)
    xs = pose_xy[0] + ca[:, None] * t[None, :]
    ys = pose_xy[1] + sa[:, None] * t[None, :]
    gx = np.floor((xs - origin[0]) / cell).astype(np.int64)
    gy = np.floor((ys - origin[1]) / cell).astype(np.int64)
    inside = (gx >= 0) & (gx < occ.shape[1]) & (gy >= 0) & (gy < occ.shape[0])
    np.clip(gx, 0, occ.shape[1] - 1, out=gx)
    np.clip(gy, 0, occ.shape[0] - 1, out=gy)
    hit = occ[gy, gx] & inside
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    ranges = np.where(any_hit, t[first], max_radius)
    return ranges, any_hit


def build_descriptor(occ_pad, origin_pad, cell, pose_xy, pose_yaw, bins, max_radius, step_frac):
    ang = (np.arange(bins, dtype=np.float64) + 0.5) * (2.0 * math.pi / bins) + pose_yaw
    ranges, hits = raycast(occ_pad, origin_pad, cell, pose_xy, ang, max_radius, step_frac)
    return ranges, hits


def desc_dist(d1, d2):
    """描述子距离：逐 bin 距离差的均值（米）。未命中已钳到 max_radius。"""
    return float(np.mean(np.abs(d1 - d2)))


def yaw_search_dist(D, bins, max_radius, bins_per_shift):
    """D(n,bins) → best(n,n)：互距离，min over 圆周移位（左移 = query 与 db 的 yaw 差）。"""
    n = D.shape[0]
    best = np.full((n, n), np.inf)
    for s in range(0, bins, bins_per_shift):
        rolled = np.roll(D, -s, axis=1)
        d = np.abs(D[:, None, :] - rolled[None, :, :]).mean(axis=2)
        best = np.minimum(best, d)
    return best


def dist_stats(v):
    v = np.asarray(v, dtype=np.float64).ravel()
    if v.size == 0:
        return {"n": 0}
    return {"n": int(v.size), "min": float(v.min()), "p25": float(np.percentile(v, 25)),
            "median": float(np.median(v)), "p75": float(np.percentile(v, 75)), "max": float(v.max()),
            "mean": float(v.mean())}


def best_threshold(self_v, cross_v):
    """在 self(小=同地点) / cross(大=异地点) 上找最佳阈值，返回 (thr, acc, tpr, tnr, margin)."""
    s = np.asarray(self_v).ravel()
    c = np.asarray(cross_v).ravel()
    cand = np.unique(np.concatenate([s, c]))
    best = (float("nan"), -1.0, 0.0, 0.0)
    for thr in cand:
        tpr = float((c > thr).mean()) if c.size else 0.0
        tnr = float((s <= thr).mean()) if s.size else 0.0
        acc = 0.5 * (tpr + tnr)
        if acc > best[1]:
            best = (float(thr), acc, tpr, tnr)
    return {"threshold": best[0], "balanced_accuracy": best[1], "tpr_cross_above": best[2],
            "tnr_self_below": best[3], "margin_min_cross_minus_max_self": float(c.min() - s.max())}


# --------------------------------------------------------------------------------------
# 测量：墙线连续性
# --------------------------------------------------------------------------------------
def cell_orientation_stats(pts_xyz, idx, shape, occ, zrel=None, min_pts=4):
    """逐占用格做 PCA，看法向与竖直方向的夹角（0° = 水平面/地面样，90° = 竖直面/墙样）。

    这是"0.15 m 格里的点到底是墙还是地面/坡道"的判据：2D 距离描述子只关心墙，
    如果带内的"墙线"有相当比例是水平面（LIO z 漂移/坡道/地面起伏被高度带切出来），
    那么先验库与"去地面后的 /scan"就不同源。

    另外按邻域把格分成 密集(8 邻域≥7 满)/线状(≤2 邻域)/其余 三类分别统计 ——
    "面状填充区"如果倾向接近水平 ⇒ 那是被高度带切出来的地面/坡道，不是墙。
    """
    cid = idx[:, 0] * shape[1] + idx[:, 1]
    order = np.argsort(cid, kind="stable")
    cid_s = cid[order]
    pts = pts_xyz[order]
    if cid_s.size == 0:
        return {"cells_pca": 0}
    bounds = np.flatnonzero(np.diff(cid_s)) + 1
    groups = np.split(pts, bounds)
    _, _, tot = neighbor_stats(occ)
    cls = np.where(tot >= 7, 0, np.where(tot <= 2, 1, 2))          # 0=密集 1=线状 2=其余
    uc = np.unique(cid_s)
    rows, cols = uc // shape[1], uc % shape[1]
    cls_of_group = cls[rows, cols]
    # 每格的 zrel 中位数（与 groups 同序）
    z_groups = np.split(zrel[order], bounds) if zrel is not None else None

    tilts, gcls, zmeds = [], [], []
    for gi, g in enumerate(groups):
        if g.shape[0] < min_pts:
            continue
        q = g - g.mean(axis=0)
        cov = (q.T @ q) / g.shape[0]
        w, v = np.linalg.eigh(cov)
        n = v[:, 0]
        tilts.append(math.degrees(math.acos(min(1.0, abs(float(n[2]))))))
        gcls.append(int(cls_of_group[gi]))
        if z_groups is not None:
            zmeds.append(float(np.median(z_groups[gi])))
    t = np.asarray(tilts)
    gcls = np.asarray(gcls)
    zmeds = np.asarray(zmeds)
    if t.size == 0:
        return {"cells_pca": 0, "min_points_per_cell": min_pts}
    cnt = np.diff(np.concatenate([[0], bounds, [cid_s.size]]))
    out = {"cells_pca": int(t.size), "min_points_per_cell": min_pts,
           "tilt_deg_median": float(np.median(t)), "tilt_deg_p25": float(np.percentile(t, 25)),
           "tilt_deg_p75": float(np.percentile(t, 75)),
           "frac_wall_like_gt60deg": float((t > 60).mean()),
           "frac_ground_like_lt30deg": float((t < 30).mean()),
           "points_per_cell_p25": float(np.percentile(cnt, 25)),
           "points_per_cell_median": float(np.median(cnt)),
           "points_per_cell_p95": float(np.percentile(cnt, 95))}
    for cid_, name in ((0, "dense_cells"), (1, "thin_cells"), (2, "other_cells")):
        sel = gcls == cid_
        if not sel.any():
            continue
        d = {"cells": int(sel.sum()), "tilt_deg_median": float(np.median(t[sel])),
             "frac_wall_like_gt60deg": float((t[sel] > 60).mean()),
             "frac_ground_like_lt30deg": float((t[sel] < 30).mean())}
        if zmeds.size:
            d["z_rel_median_of_cell_medians"] = float(np.median(zmeds[sel]))
        out[name] = d
    return out


def dilate8(mask, n):
    m = mask.copy()
    for _ in range(int(n)):
        p = np.pad(m, 1, mode="constant", constant_values=False)
        m = (m | p[0:-2, 0:-2] | p[0:-2, 1:-1] | p[0:-2, 2:] | p[1:-1, 0:-2]
             | p[1:-1, 2:] | p[2:, 0:-2] | p[2:, 1:-1] | p[2:, 2:])
    return m


def map_overlap_metric(occ, origin, cell, pgm, res, map_origin, tol):
    """占用格 vs cartographer 栅格图（pgm）的重合度（A1.2 的"墙线是否与 pgm 重合"）。

    pgm 里 <128 视为占用（本仓 map/*.pgm 是 trinary/negate=0）。**格占用 = 格覆盖范围内只要
    有一个占用像素**（积分图 max-pooling；按格中心单点采样会漏掉 0.05 m 的细墙）。tol 为容差
    （米），按格数取整后做 8 邻域膨胀再判交。
    """
    h, w = occ.shape
    ph, pw = pgm.shape
    occ_px = (pgm < 128).astype(np.int32)
    I = np.pad(occ_px.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    xs = origin[0] + (np.arange(w) + 0.5) * cell
    ys = origin[1] + (np.arange(h) + 0.5) * cell
    c0 = np.clip(np.floor((xs - 0.5 * cell - map_origin[0]) / res).astype(np.int64), 0, pw - 1)
    c1 = np.clip(np.floor((xs + 0.5 * cell - map_origin[0]) / res).astype(np.int64), 0, pw - 1)
    top = map_origin[1] + ph * res                       # pgm 行号随 y 增大而减小
    r0 = np.clip(np.floor((top - (ys + 0.5 * cell)) / res).astype(np.int64), 0, ph - 1)
    r1 = np.clip(np.floor((top - (ys - 0.5 * cell)) / res).astype(np.int64), 0, ph - 1)
    in_c = (xs - 0.5 * cell >= map_origin[0]) & (xs + 0.5 * cell <= map_origin[0] + pw * res)
    in_r = (ys - 0.5 * cell >= map_origin[1]) & (ys + 0.5 * cell <= top)
    inside = in_r[:, None] & in_c[None, :]
    R0, C0 = r0[:, None], c0[None, :]
    R1, C1 = r1[:, None], c1[None, :]
    s = I[R1 + 1, C1 + 1] - I[R0, C1 + 1] - I[R1 + 1, C0] + I[R0, C0]
    pgm_occ = (s > 0) & inside
    n = int(round(tol / cell))
    occ_near = occ & dilate8(pgm_occ, n)
    pgm_near = pgm_occ & dilate8(occ, n)
    return {"tol_m": tol, "tol_cells": n, "effective_tol_m": n * cell,
            "cells_inside_pgm": int(inside.sum()),
            "pgm_occupied_cells_in_grid": int(pgm_occ.sum()),
            "frac_occ_cells_near_pgm_wall": float(occ_near.sum() / max(1, int(occ.sum()))),
            "frac_pgm_wall_cells_near_occ": float(pgm_near.sum() / max(1, int(pgm_occ.sum()))),
            "occ_cells_far_from_pgm_wall": int((occ & ~dilate8(pgm_occ, n)).sum())}


def measure_wall_structure(occ, cell, close_pad):
    """2D 占用格的连续性指标。返回 (dict, skel, lab8, enclosed(紧格图内), center_rc)。"""
    out = {}
    n_occ = int(occ.sum())
    out["occupied_cells"] = n_occ
    out["grid_shape_rc"] = [int(occ.shape[0]), int(occ.shape[1])]
    out["occupied_fraction_of_bbox"] = float(n_occ / occ.size)
    out["wall_length_cells_x_cell"] = float(n_occ * cell)

    orth, diag, tot = neighbor_stats(occ)
    thin = occ & (tot <= 2)
    out["thin_cells_le2_neighbors"] = int(thin.sum())
    out["wall_length_from_thin_cells"] = float(int(thin.sum()) * cell)
    # 轮廓法：占用格中"有自由 4 邻居"的格 = 轮廓格。1 格厚的线 ⇒ 轮廓格 ≈ 2×线长 ⇒ 线长 ≈ 轮廓格/2
    free = ~occ
    pfree = np.pad(free, 1, mode="constant", constant_values=True)
    outline = occ & (pfree[0:-2, 1:-1] | pfree[2:, 1:-1] | pfree[1:-1, 0:-2] | pfree[1:-1, 2:])
    out["outline_cells"] = int(outline.sum())
    out["wall_length_from_outline_half"] = 0.5 * float(int(outline.sum())) * cell

    skel = zhang_suen(occ)
    so, sd, st = neighbor_stats(skel)
    n_sk = int(skel.sum())
    # 只在骨架格上累加邻居数（否则会把背景格的邻居也加进来 ⇒ 长度爆炸）
    length = 0.5 * float((so[skel] * cell + sd[skel] * cell * math.sqrt(2.0)).sum())
    out["skeleton_cells"] = n_sk
    out["wall_length_skeleton_m"] = length
    out["skeleton_over_occupied_ratio"] = float(n_sk / n_occ) if n_occ else 0.0
    out["skeleton_endpoints"] = int((skel & (st == 1)).sum())
    out["skeleton_junctions"] = int((skel & (st >= 3)).sum())

    n4, lab4, sz4 = label_components(occ, 4)
    n8, lab8, sz8 = label_components(occ, 8)
    out["components_4conn"] = n4
    out["components_8conn"] = n8
    out["component_sizes_8conn_top5"] = sz8[:5]
    out["largest_component_cells_8conn"] = sz8[0] if sz8 else 0
    out["largest_component_frac_8conn"] = float(sz8[0] / n_occ) if n_occ else 0.0

    # 闭合性：把格图外扩 close_pad 格当"外面"，从外圈对自由格洪泛；漏不进紧格图内部的自由格
    # = 被墙线围住（证明轮廓闭合）。紧 bbox 会让"外面"退化 ⇒ 必须补外圈，否则指标虚高。
    occ_c = pad_grid(occ, close_pad)
    free = ~occ_c
    seen = np.zeros_like(free)
    q = deque()
    for r in range(occ_c.shape[0]):
        for c in (0, occ_c.shape[1] - 1):
            if free[r, c] and not seen[r, c]:
                seen[r, c] = True
                q.append((r, c))
    for c in range(occ_c.shape[1]):
        for r in (0, occ_c.shape[0] - 1):
            if free[r, c] and not seen[r, c]:
                seen[r, c] = True
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < occ_c.shape[0] and 0 <= cc < occ_c.shape[1] and free[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                q.append((rr, cc))
    enclosed = (free & ~seen)[close_pad:close_pad + occ.shape[0],
                              close_pad:close_pad + occ.shape[1]]
    free_tight = ~occ
    out["closure_pad_cells"] = int(close_pad)
    out["enclosed_free_cells"] = int(enclosed.sum())
    out["enclosed_free_area_m2"] = float(enclosed.sum() * cell * cell)
    out["free_cells_tight"] = int(free_tight.sum())
    out["enclosed_ratio_of_free"] = float(enclosed.sum() / max(1, int(free_tight.sum())))
    out["leaked_free_cells"] = int((free_tight & ~enclosed).sum())

    # 场中心：最大的"被围住"自由连通域的中心（= 场地内部），扫掠从这里打
    center_rc = None
    if enclosed.any():
        n_ce, lab_ce, sz_ce = label_components(enclosed, 4)
        out["enclosed_components_4conn"] = n_ce
        out["enclosed_largest_component_cells"] = sz_ce[0]
        biggest = np.argwhere(lab_ce == 1).astype(np.float64)
        cen = biggest.mean(axis=0)
        d = np.linalg.norm(biggest - cen[None, :], axis=1)
        center_rc = biggest[int(np.argmin(d))]
    if center_rc is None:
        n4o, lab4o, sz4o = label_components(free_tight, 4)
        if sz4o:
            biggest = np.argwhere(lab4o == 1).astype(np.float64)
            center_rc = biggest[int(np.argmin(np.linalg.norm(biggest - biggest.mean(axis=0)[None, :],
                                                             axis=1)))]
    return out, skel, lab8, enclosed, center_rc


def angular_sweep(occ_pad, origin_pad, cell, centroid_xy, max_radius, n_ang, step_frac):
    ang = (np.arange(n_ang, dtype=np.float64) + 0.5) * (2.0 * math.pi / n_ang)
    ranges, hits = raycast(occ_pad, origin_pad, cell, centroid_xy, ang, max_radius, step_frac)
    res = {"n_angles": int(n_ang), "hit_angles": int(hits.sum()), "gap_angles": int((~hits).sum()),
           "gap_fraction": float((~hits).mean())}
    if hits.any():
        res["hit_radius_min"] = float(ranges[hits].min())
        res["hit_radius_median"] = float(np.median(ranges[hits]))
        res["hit_radius_max"] = float(ranges[hits].max())
    # 命中角度的连续段数（= 轮廓被切成几段）
    h = hits.astype(np.int8)
    if h.all():
        res["angular_hit_runs"] = 1
        res["max_gap_deg"] = 0.0
    elif not h.any():
        res["angular_hit_runs"] = 0
        res["max_gap_deg"] = 360.0
    else:
        d = np.diff(np.concatenate([h, h[:1]]))
        starts = int((d == 1).sum())
        res["angular_hit_runs"] = starts
        # 最大连续 gap 段（循环）
        idx = np.where(h == 0)[0]
        runs, cur = [], 1
        for i in range(1, len(idx)):
            if idx[i] == idx[i - 1] + 1:
                cur += 1
            else:
                runs.append(cur)
                cur = 1
        runs.append(cur)
        if h[0] == 0 and h[-1] == 0 and len(runs) > 1:
            runs[0] += runs[-1]
            runs = runs[:-1]
        max_run = max(runs) if runs else 0
        res["max_gap_deg"] = float(max_run * 360.0 / n_ang)
    r_ref = res.get("hit_radius_median", max_radius)
    g = math.radians(res["max_gap_deg"])
    res["max_gap_arc_at_median_radius_m"] = float(r_ref * g)
    res["max_gap_chord_at_median_radius_m"] = float(2.0 * r_ref * math.sin(g / 2.0))
    return res


# --------------------------------------------------------------------------------------
# 查询位姿选取
# --------------------------------------------------------------------------------------
def pick_poses(occ, dist_cells, n_poses, clearance_cells, seed):
    """优先"内部（洪泛漏不进）+ 离墙 ≥ clearance"的自由格，用最远点采样铺开。"""
    # enclosed 在这里重算一次，避免依赖调用顺序
    free = ~occ
    seen = np.zeros_like(free)
    q = deque()
    for r in range(occ.shape[0]):
        for c in (0, occ.shape[1] - 1):
            if free[r, c] and not seen[r, c]:
                seen[r, c] = True
                q.append((r, c))
    for c in range(occ.shape[1]):
        for r in (0, occ.shape[0] - 1):
            if free[r, c] and not seen[r, c]:
                seen[r, c] = True
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < occ.shape[0] and 0 <= cc < occ.shape[1] and free[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                q.append((rr, cc))
    enclosed = free & ~seen
    base = dist_cells >= int(clearance_cells)
    cand = base & enclosed
    src = "enclosed+clearance"
    if int(cand.sum()) < n_poses:
        cand = base
        src = "clearance_only(未闭合/内部格不足)"
    if int(cand.sum()) < n_poses:
        cand = free & (dist_cells >= max(1, int(clearance_cells // 2)))
        src = "clearance/2_only"
    if int(cand.sum()) == 0:
        cand = free
        src = "any_free"
    pts = np.argwhere(cand).astype(np.float64)
    # 起点：到占用质心最远的候选；随后最远点采样
    rng = np.random.default_rng(seed)
    occ_cells = np.argwhere(occ).astype(np.float64)
    cen = occ_cells.mean(axis=0)
    d0 = np.linalg.norm(pts - cen[None, :], axis=1)
    sel = [int(np.argmax(d0))]
    dmin = np.linalg.norm(pts - pts[sel[0]][None, :], axis=1)
    while len(sel) < min(n_poses, len(pts)):
        nxt = int(np.argmax(dmin))
        if dmin[nxt] <= 0:
            break
        sel.append(nxt)
        dmin = np.minimum(dmin, np.linalg.norm(pts - pts[nxt][None, :], axis=1))
    if len(sel) < n_poses and len(pts) > len(sel):
        rest = np.setdiff1d(np.arange(len(pts)), np.array(sel))
        extra = rng.choice(rest, size=min(n_poses - len(sel), len(rest)), replace=False)
        sel.extend(int(i) for i in extra)
    return pts[sel], src, int(cand.sum()), int(enclosed.sum())


# --------------------------------------------------------------------------------------
# 主流程（单份 PCD）
# --------------------------------------------------------------------------------------
def probe_one(args, path):
    t0 = time.time()
    label = args.label or os.path.splitext(os.path.basename(path))[0]
    log("=" * 100)
    log(f"[{label}] {path}")
    log("=" * 100)

    pcd = read_pcd(path)
    xyz = pcd["xyz"]
    res = {"label": label, "pcd": os.path.abspath(path), "n_points": int(xyz.shape[0]),
           "n_points_dropped_nonfinite": pcd["n_bad"], "fields": pcd["fields"]}
    log(f"  点数 = {xyz.shape[0]}（丢弃非有限 {pcd['n_bad']}）, 字段 = {' '.join(pcd['fields'])}")
    log(f"  bbox x[{xyz[:,0].min():.3f},{xyz[:,0].max():.3f}] "
        f"y[{xyz[:,1].min():.3f},{xyz[:,1].max():.3f}] z[{xyz[:,2].min():.3f},{xyz[:,2].max():.3f}] "
        f"z 跨度 {xyz[:,2].max()-xyz[:,2].min():.3f} m")
    res["bbox"] = {"x": [float(xyz[:, 0].min()), float(xyz[:, 0].max())],
                   "y": [float(xyz[:, 1].min()), float(xyz[:, 1].max())],
                   "z": [float(xyz[:, 2].min()), float(xyz[:, 2].max())]}
    res["z_span"] = float(xyz[:, 2].max() - xyz[:, 2].min())

    # 非几何字段是否全 0（强度/法向/曲率）
    zero_fields = {}
    for k, v in pcd["extra"].items():
        vv = np.asarray(v, dtype=np.float64)
        zero_fields[k] = {"min": float(vv.min()), "max": float(vv.max()),
                          "all_zero": bool(vv.min() == 0.0 and vv.max() == 0.0)}
    res["extra_fields"] = zero_fields
    for k, v in zero_fields.items():
        if v["all_zero"]:
            log(f"  [!] 字段 {k} 全 0（min=max=0）")

    # 地面
    if args.floor_z == "auto":
        floor = float(np.percentile(xyz[:, 2], 5.0))
        floor_src = "auto(5% 分位)"
    else:
        floor = float(args.floor_z)
        floor_src = "manual"
    res["floor_z"] = floor
    res["floor_z_source"] = floor_src
    log(f"  地面 z = {floor:.4f} m（{floor_src}）")

    zrel = xyz[:, 2] - floor
    band_lo, band_hi = args.height_band
    in_band = (zrel >= band_lo) & (zrel < band_hi)
    n_band = int(in_band.sum())
    log(f"  高度带 [{band_lo:g}, {band_hi:g}) m（地面以上）点数 = {n_band}")
    res["height_band_above_floor"] = [band_lo, band_hi]
    res["points_in_band"] = n_band

    if n_band < 10:
        log(f"  [!] 带内点太少（{n_band}），后面指标无意义 —— 直接判定不可用")
        res["usable"] = False
        res["wall"] = {"occupied_cells": 0}
        res["elapsed_s"] = time.time() - t0
        return res

    xy_band = xyz[in_band][:, :2]
    idx, origin, shape = grid_of(xy_band, args.cell)
    occ = occ_from_idx(idx, shape)
    log(f"  2D 投影格图（cell={args.cell:g} m）：{shape[1]}×{shape[0]} 格, "
        f"占用 {int(occ.sum())} 格, 覆盖率 {occ.mean()*100:.2f}%")
    res["cell_size"] = args.cell
    res["grid"] = {"origin_xy": [float(origin[0]), float(origin[1])],
                   "shape_rc": [int(shape[0]), int(shape[1])]}
    res["_occ"] = occ          # 仅供本进程内 ASCII/PNG 用，写 JSON 前会被剥掉
    res["_origin"] = origin

    # ---- 包围盒 / 场地周长代理 -------------------------------------------------
    lo, hi = xy_band.min(axis=0), xy_band.max(axis=0)
    wh = hi - lo
    perim = 2.0 * float(wh[0] + wh[1])
    res["xy_bbox_size_m"] = [float(wh[0]), float(wh[1])]
    res["bbox_perimeter_m"] = perim
    res["wall_length_criterion_half_bbox_perimeter_m"] = 0.5 * perim
    log(f"  xy 包围盒 {wh[0]:.2f} × {wh[1]:.2f} m ⇒ 周长代理 {perim:.2f} m "
        f"（判据：墙长 ≥ 一半 = {0.5*perim:.2f} m）")

    wall, skel, lab8, enclosed, center_rc = measure_wall_structure(occ, args.cell, args.close_pad)
    orient = cell_orientation_stats(xyz[in_band][:, :3], idx, shape, occ, zrel[in_band])
    wall["orientation"] = orient
    res["wall"] = wall
    log(f"  ── 墙线连续性 ──")
    log(f"    占用格 = {wall['occupied_cells']}（占 bbox {wall['occupied_fraction_of_bbox']*100:.1f}%"
        f"；格数×cell = 墙长上界 {wall['wall_length_cells_x_cell']:.2f} m）")
    if orient.get("cells_pca"):
        log(f"    逐格 PCA 倾向（0°=水平面/地面样，90°=竖直面/墙样；{orient['cells_pca']} 格可用）："
            f"中位 {orient['tilt_deg_median']:.1f}°，墙样(>60°) {orient['frac_wall_like_gt60deg']*100:.1f}%，"
            f"地面样(<30°) {orient['frac_ground_like_lt30deg']*100:.1f}%；"
            f"点/格 中位 {orient['points_per_cell_median']:.0f}"
            f"（p25 {orient['points_per_cell_p25']:.0f} / p95 {orient['points_per_cell_p95']:.0f}）")
        for key, name in (("dense_cells", "面状密集格"), ("thin_cells", "线状格"), ("other_cells", "其余")):
            d = orient.get(key)
            if d:
                log(f"      · {name}: {d['cells']:5d} 格  倾角中位 {d['tilt_deg_median']:5.1f}°  "
                    f"墙样 {d['frac_wall_like_gt60deg']*100:5.1f}%  地面样 "
                    f"{d['frac_ground_like_lt30deg']*100:5.1f}%"
                    + (f"  z_rel 中位 {d['z_rel_median_of_cell_medians']:.3f} m"
                       if "z_rel_median_of_cell_medians" in d else ""))
    else:
        log(f"    逐格 PCA：可用格不足（每格 ≥{orient.get('min_points_per_cell', 4)} 点的格太少）"
            f" —— 该高度带里**没有面状结构**")
    log(f"    墙长估计（三种口径）：格数×cell 上界 {wall['wall_length_cells_x_cell']:.1f} m / "
        f"轮廓格/2 {wall['wall_length_from_outline_half']:.1f} m（{wall['outline_cells']} 轮廓格）/ "
        f"骨架 {wall['wall_length_skeleton_m']:.1f} m")
    log(f"    细化骨架：{wall['skeleton_cells']} 格（骨架/占用 = {wall['skeleton_over_occupied_ratio']:.2f}）"
        f" ⇒ 墙长估计 **{wall['wall_length_skeleton_m']:.2f} m**"
        f"（端点 {wall['skeleton_endpoints']} 个 / 分叉 {wall['skeleton_junctions']} 个）")
    log(f"    连通域：4 邻域 {wall['components_4conn']} 个 / 8 邻域 {wall['components_8conn']} 个，"
        f"最大域 {wall['largest_component_cells_8conn']} 格"
        f"（占 {wall['largest_component_frac_8conn']*100:.1f}%）")
    log(f"    闭合性（外扩 {wall['closure_pad_cells']} 格后从外圈洪泛）：内部自由格 "
        f"{wall['enclosed_free_cells']} 格 = {wall['enclosed_free_area_m2']:.2f} m²"
        f"（占紧格图自由区 {wall['enclosed_ratio_of_free']*100:.1f}%，漏进去的 {wall['leaked_free_cells']} 格；"
        f"最大内部连通域 {wall.get('enclosed_largest_component_cells', 0)} 格）")

    # ---- 高度带敏感性（给"该用哪个高度带"提供依据）-----------------------------
    edges = [(0.05, 0.10), (0.10, 0.20), (0.20, 0.35), (0.35, 0.50), (0.50, 1.00), (1.00, INF)]
    sweep_rows = []
    for lo_, hi_ in edges:
        m = (zrel >= lo_) & (zrel < hi_)
        n_pt = int(m.sum())
        if n_pt:
            ix, _, sh = grid_of(xyz[m][:, :2], args.cell)
            n_cell = int(len(np.unique(ix[:, 0] * sh[1] + ix[:, 1])))
        else:
            n_cell = 0
        sweep_rows.append({"band_above_floor": [lo_, hi_], "points": n_pt, "cells": n_cell})
    res["height_band_sweep"] = sweep_rows
    log(f"  ── 高度带敏感性（cell={args.cell:g} m）──")
    for row in sweep_rows:
        log(f"    [{row['band_above_floor'][0]:.2f},{row['band_above_floor'][1]:>4}) m: "
            f"{row['points']:8d} 点 → {row['cells']:6d} 格")

    # 描述子用的带 margin 格图（margin = max_radius，越界=自由）
    margin = int(math.ceil(args.max_radius / args.cell)) + 1
    occ_pad = pad_grid(occ, margin)
    origin_pad = origin - margin * args.cell
    n_occ_pad = int(occ_pad.sum())

    # ---- 角度扫掠（最大缺口）-------------------------------------------------
    #  扫掠原点必须落在**场地内部的开阔处**：占用格质心常常落在墙/矮障碍的密集带里
    #  （实测 RMUL2026 质心到最近占用格只有 0.04 m ⇒ 所有射线立刻命中，缺口指标退化）。
    occ_centroid_rc = np.argwhere(occ).mean(axis=0)          # (行=y, 列=x)
    occ_centroid_xy = rc_to_xy(occ_centroid_rc, origin, args.cell)[0]
    if center_rc is not None:
        center_src = "largest_enclosed_free_component_center"
    else:
        center_rc = occ_centroid_rc
        center_src = "occupied_centroid(没有内部自由区 ⇒ 场地未闭合)"
    center_xy = rc_to_xy(center_rc, origin, args.cell)[0]
    res["sweep_center_xy"] = [float(center_xy[0]), float(center_xy[1])]
    res["sweep_center_source"] = center_src
    res["occupied_centroid_xy"] = [float(occ_centroid_xy[0]), float(occ_centroid_xy[1])]
    res["bbox_center_xy"] = [float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)]
    sweep = angular_sweep(occ_pad, origin_pad, args.cell, center_xy, args.max_radius,
                          args.n_ang_sweep, args.step_frac)
    sweep_occ = angular_sweep(occ_pad, origin_pad, args.cell, occ_centroid_xy, args.max_radius,
                              args.n_ang_sweep, args.step_frac)
    res["angular_sweep"] = sweep
    res["angular_sweep_from_occupied_centroid"] = sweep_occ
    log(f"    角度扫掠（原点 = {center_src} ({center_xy[0]:.2f},{center_xy[1]:.2f})；"
        f"{args.n_ang_sweep} 条射线 / r≤{args.max_radius:g} m）：命中 {sweep['hit_angles']} 条，"
        f"缺口 {sweep['gap_angles']} 条（{sweep['gap_fraction']*100:.1f}%），"
        f"命中半径 min/中位/max = {sweep.get('hit_radius_min', float('nan')):.2f}/"
        f"{sweep.get('hit_radius_median', float('nan')):.2f}/"
        f"{sweep.get('hit_radius_max', float('nan')):.2f} m")
    log(f"    **最大角度缺口 = {sweep['max_gap_deg']:.2f}°** ⇒ 弧长 ≈ "
        f"{sweep['max_gap_arc_at_median_radius_m']:.2f} m（弦长 ≈ "
        f"{sweep['max_gap_chord_at_median_radius_m']:.2f} m）；命中角连续段 = {sweep['angular_hit_runs']}")
    log(f"    （对照：从占用格质心 ({occ_centroid_xy[0]:.2f},{occ_centroid_xy[1]:.2f}) 扫掠 "
        f"→ 命中半径中位 {sweep_occ.get('hit_radius_median', float('nan')):.2f} m，"
        f"缺口 {sweep_occ['gap_fraction']*100:.1f}% —— 质心落在结构带内时该指标退化）")

    # ---- 查询位姿 -------------------------------------------------------------
    clearance_cells = max(1, int(round(args.pose_clearance / args.cell)))
    dist_cells = geodesic_clearance(occ, clearance_cells + 2)
    poses_rc, pose_src, n_cand, n_encl = pick_poses(occ, dist_cells, args.n_poses,
                                                    clearance_cells, args.seed)
    poses_xy = rc_to_xy(poses_rc, origin, args.cell)
    res["pose_selection"] = {"clearance_m": args.pose_clearance, "source": pose_src,
                             "candidates": n_cand, "n_poses": int(len(poses_xy)),
                             "clearance_cells": clearance_cells}
    log(f"  ── 查询位姿（{len(poses_xy)} 个，来源「{pose_src}」，候选 {n_cand} 格）──")
    pose_rows = []
    for i, p in enumerate(poses_xy):
        log(f"    #{i:02d} ({p[0]:7.3f}, {p[1]:7.3f})  xy 与 bbox 中心 "
            f"{np.linalg.norm(p - np.array([(lo[0]+hi[0])/2, (lo[1]+hi[1])/2])):.2f} m")
        pose_rows.append([float(p[0]), float(p[1])])
    res["poses_xy"] = pose_rows

    # ---- 描述子 ---------------------------------------------------------------
    bins = args.bins
    bins_per_shift = max(1, int(round(math.radians(args.yaw_search_step) / (2 * math.pi) * bins)))
    log(f"  ── 2D 极坐标距离描述子（{bins} bins = {360.0/bins:.3f}°/bin, r_max={args.max_radius:g} m,"
        f" 采样步长 {args.cell*args.step_frac:.3f} m）──")
    D, hitfrac = [], []
    for p in poses_xy:
        r, h = build_descriptor(occ_pad, origin_pad, args.cell, p, 0.0, bins, args.max_radius,
                                args.step_frac)
        D.append(r)
        hitfrac.append(float(h.mean()))
    D = np.asarray(D)
    res["descriptor"] = {"bins": bins, "deg_per_bin": 360.0 / bins, "max_radius": args.max_radius,
                         "step_m": args.cell * args.step_frac,
                         "hit_fraction_per_pose": [float(x) for x in hitfrac],
                         "hit_fraction_median": float(np.median(hitfrac))}
    log(f"    每姿势命中 bin 占比：中位 {np.median(hitfrac)*100:.1f}% "
        f"(min {min(hitfrac)*100:.1f}%, max {max(hitfrac)*100:.1f}%)")

    # 自距离：小扰动（±0.5 格 xy、±self_yaw_deg yaw）**重建**描述子
    rng = np.random.default_rng(args.seed + 1)
    half = 0.5 * args.cell
    yaw_pert = math.radians(args.self_yaw_deg)
    self_raw = np.zeros((len(poses_xy), args.self_perturbations))
    self_desc = np.zeros((len(poses_xy), args.self_perturbations, bins))
    for i, (x, y) in enumerate(poses_xy):
        for k in range(args.self_perturbations):
            dx = float(rng.uniform(-half, half))
            dy = float(rng.uniform(-half, half))
            dyaw = float(rng.uniform(-yaw_pert, yaw_pert))
            r, _ = build_descriptor(occ_pad, origin_pad, args.cell, (x + dx, y + dy), dyaw, bins,
                                    args.max_radius, args.step_frac)
            self_desc[i, k] = r
            self_raw[i, k] = desc_dist(D[i], r)
    res["self_distance_raw"] = dist_stats(self_raw)

    # 互距离：s=0（不搜 yaw）与 min-over-圆周移位（搜 yaw，= 真实检索时的判别力，更保守）
    cross_zero = np.abs(D[:, None, :] - D[None, :, :]).mean(axis=2)
    cross_best = yaw_search_dist(D, bins, args.max_radius, bins_per_shift)
    iu = np.triu_indices(len(poses_xy), k=1)
    cross_v = cross_best[iu]
    # 同一地点、扰动重建之后再搜 yaw：自距离的"最好情形"
    self_best = np.zeros_like(self_raw)
    for s in range(0, bins, bins_per_shift):
        shifted = np.roll(D, -s, axis=1)                      # (n, bins)
        d = np.abs(self_desc - shifted[:, None, :]).mean(axis=2)   # (n, k)
        self_best = np.minimum(self_best, d) if s else d
    res["self_distance_yawsearched"] = dist_stats(self_best)
    res["cross_distance_yawsearched"] = dist_stats(cross_v)
    res["cross_distance_zero_yaw"] = dist_stats(cross_zero[iu])
    sep = best_threshold(self_raw.ravel(), cross_v)
    sep_y = best_threshold(self_best.ravel(), cross_v)
    res["separation_raw_vs_cross"] = sep
    res["separation_yawsearched"] = sep_y
    r_raw, r_y = res["self_distance_raw"], res["self_distance_yawsearched"]
    c = res["cross_distance_yawsearched"]
    c0 = res["cross_distance_zero_yaw"]
    ratio = (c["median"] / r_raw["median"]) if r_raw["median"] > 0 else float("inf")
    ratio_y = (c["median"] / r_y["median"]) if r_y["median"] > 0 else float("inf")
    log(f"  ── 描述子区分性 ──")
    log(f"    自距离(小扰动重建, 不搜 yaw)：min {r_raw['min']:.4f} / 中位 {r_raw['median']:.4f} / "
        f"max {r_raw['max']:.4f} m")
    log(f"    自距离(扰动重建 + yaw 搜索)  ：min {r_y['min']:.4f} / 中位 {r_y['median']:.4f} / "
        f"max {r_y['max']:.4f} m")
    log(f"    互距离(异地点, Δyaw=0)       ：min {c0['min']:.4f} / 中位 {c0['median']:.4f} / "
        f"max {c0['max']:.4f} m")
    log(f"    互距离(异地点, yaw 搜索后)   ：min {c['min']:.4f} / 中位 {c['median']:.4f} / "
        f"max {c['max']:.4f} m   (n={c['n']})")
    log(f"    ⇒ 中位比 互/自 = {ratio:.2f}×（不搜 yaw）/ {ratio_y:.2f}×（搜 yaw）（判据 ≥ 5×）")
    log(f"    ⇒ 分离裕度 min(互) − max(自) = {c['min']-r_raw['max']:+.4f} m（不搜 yaw）/ "
        f"{c['min']-r_y['max']:+.4f} m（搜 yaw）")
    log(f"    ⇒ 最佳阈值 {sep['threshold']:.4f} m, 平衡准确率 {sep['balanced_accuracy']*100:.1f}% "
        f"(TPR {sep['tpr_cross_above']*100:.1f}% / TNR {sep['tnr_self_below']*100:.1f}%)")

    # 互距离 vs 位姿间距：近邻（< grid 分辨率）单列
    dxy = np.linalg.norm(poses_xy[:, None, :] - poses_xy[None, :, :], axis=2)
    for tag, thr_xy in (("within_1.0m", 1.0), (f"far_ge_{args.cross_far_m:g}m", args.cross_far_m)):
        m = (dxy[iu] <= thr_xy) if tag.startswith("within") else (dxy[iu] >= thr_xy)
        res[f"cross_{tag}"] = dist_stats(cross_v[m])
        if cross_v[m].size:
            log(f"    互距离(位姿间距 {tag})：min {cross_v[m].min():.4f} / 中位 "
                f"{np.median(cross_v[m]):.4f} / max {cross_v[m].max():.4f} m (n={int(m.sum())})")
    # 检索意义上的分界：位姿相距 ≥ cross_far 的互距离 vs 自距离（"同一格内"的邻居对不算真歧义）
    m_far = dxy[iu] >= args.cross_far_m
    if cross_v[m_far].size:
        sep_far = best_threshold(self_best.ravel(), cross_v[m_far])
        res["separation_yawsearched_far"] = sep_far
        res["margin_yawsearched_far_min_cross_minus_max_self"] = float(
            cross_v[m_far].min() - self_best.max())
        log(f"    ⇒ 只算相距 ≥{args.cross_far_m:g} m 的互距离：min {cross_v[m_far].min():.4f} m，"
            f"分离裕度(搜 yaw) {cross_v[m_far].min()-self_best.max():+.4f} m，"
            f"最佳阈值 {sep_far['threshold']:.4f} m 平衡准确率 {sep_far['balanced_accuracy']*100:.1f}%"
            f" (TPR {sep_far['tpr_cross_above']*100:.1f}% / TNR {sep_far['tnr_self_below']*100:.1f}%)")

    # 最易混的几对
    order = np.argsort(cross_v)
    worst = []
    for j in order[:5]:
        a, b = int(iu[0][j]), int(iu[1][j])
        worst.append({"pose_a": a, "pose_b": b, "dist": float(cross_v[j]),
                      "xy_sep_m": float(dxy[a, b])})
        log(f"    [!] 最易混对 #{a}-#{b}: d={cross_v[j]:.4f} m, 位姿间距 {dxy[a,b]:.2f} m")
    res["most_confusable_pairs"] = worst

    # yaw 鲁棒性曲线
    offs = np.arange(0.0, args.yaw_scan_max + 0.5 * args.yaw_scan_step, args.yaw_scan_step)
    curve = {float(o): [] for o in offs}
    for i in range(len(poses_xy)):
        for o in offs:
            s = int(round(math.radians(o) / (2 * math.pi) * bins)) % bins
            curve[float(o)].append(desc_dist(D[i], np.roll(D[i], s)))
    yaw_rows = []
    log(f"  ── yaw 鲁棒性（同一地点纯 yaw 偏移，描述子圆周移位）──")
    for o in offs:
        v = np.asarray(curve[float(o)])
        yaw_rows.append({"dyaw_deg": float(o), "median": float(np.median(v)),
                         "min": float(v.min()), "max": float(v.max())})
    for row in yaw_rows:
        if abs(row["dyaw_deg"] % 5.0) < 1e-9 or row["dyaw_deg"] == 0.0:
            log(f"    Δyaw {row['dyaw_deg']:5.1f}° : 中位 {row['median']:.4f} m "
                f"(min {row['min']:.4f}, max {row['max']:.4f})")
    res["yaw_robustness"] = yaw_rows
    # 距离超过"自距离 max"时的最小 Δyaw（⇒ 粗 yaw 搜索步长上界）
    thr = max(r_raw["max"], 1e-9)
    crossed = [row["dyaw_deg"] for row in yaw_rows if row["median"] > thr]
    res["yaw_offset_exceeding_self_max_deg"] = float(min(crossed)) if crossed else None
    if crossed:
        log(f"    ⇒ 中位距离首次超过「自距离 max = {thr:.4f} m」的 Δyaw = {min(crossed):.1f}°"
            f" ⇒ 粗 yaw 搜索步长应 ≤ 该值（否则真值会落在两档之间）")

    # ---- 可选：与 cartographer 栅格图（pgm）的重合度 --------------------------
    if args.map_png:
        try:
            pgm_img, _, _ = read_pgm(args.map_png)
            ov = map_overlap_metric(occ, origin, args.cell, pgm_img,
                                    args.map_resolution or 0.05,
                                    args.map_origin or [0.0, 0.0], args.map_tol)
            res["map_overlap"] = ov
            log(f"  ── 与 {os.path.basename(args.map_png)} 的重合度（容差 {ov['effective_tol_m']:.2f} m"
                f" = {ov['tol_cells']} 格）──")
            log(f"    占用格中 {ov['frac_occ_cells_near_pgm_wall']*100:.1f}% 落在 pgm 墙附近"
                f"（离 pgm 墙 >{ov['effective_tol_m']:.2f} m 的 {ov['occ_cells_far_from_pgm_wall']} 格）；"
                f"pgm 墙像素中 {ov['frac_pgm_wall_cells_near_occ']*100:.1f}% 附近有占用格"
                f"（格图内 pgm 占用 {ov['pgm_occupied_cells_in_grid']} 格）")
        except SystemExit:
            raise
        except Exception as exc:      # pragma: no cover - 诊断用，不因叠加失败而中断
            log(f"  [!] 与 pgm 的重合度计算失败：{exc}")

    # ---- 可选：渲染 PNG（写 .tmp_cache/.tmp_research 才允许）-------------------
    if args.render_png:
        out = _check_tmp_path(args.render_png)
        render_png(out, occ, origin, args.cell, poses_xy, args.map_png, args.map_resolution,
                   args.map_origin)
        res["render_png"] = out
        log(f"  [i] 已渲染 {out}")

    res["usable"] = True
    res["elapsed_s"] = time.time() - t0
    log(f"  （本份耗时 {res['elapsed_s']:.1f} s）")
    return res


# --------------------------------------------------------------------------------------
# 极简 PNG 写出（zlib + struct，无 matplotlib）
# --------------------------------------------------------------------------------------
def write_png_rgb(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as fh:
        fh.write(png)


def read_pgm(path):
    """读 P5 PGM（只读我们自己的 map/*.pgm）。返回 (img uint8, w, h)。"""
    with open(path, "rb") as fh:
        data = fh.read()
    # 手工解析 header（可能有注释）
    tok, i = [], 0
    while len(tok) < 4 and i < len(data):
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tok.append(data[i:j])
        i = j
    if tok[0] != b"P5":
        die(f"{path}: 只支持 P5 PGM（实际 {tok[0]!r}）")
    w, h = int(tok[1]), int(tok[2])
    i += 1  # 单个空白
    img = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i).reshape(h, w)
    return img, w, h


def render_png(path, occ, origin, cell, poses_xy, map_png=None, map_resolution=None,
               map_origin=None):
    """占用格（红） + 查询位姿（绿）+ 可选 pgm 背景（灰）。"""
    h, w = occ.shape
    rgb = np.full((h, w, 3), 255, dtype=np.uint8)
    if map_png and os.path.isfile(map_png):
        img, pw, ph = read_pgm(map_png)
        res = map_resolution or 0.05
        mo = map_origin or [0.0, 0.0]
        # pgm 行 0 = 地图 y 最大
        for r in range(h):
            yy = origin[1] + (r + 0.5) * cell
            pr = int((mo[1] + ph * res - yy) / res)
            if not (0 <= pr < ph):
                continue
            xx = origin[0] + (np.arange(w) + 0.5) * cell
            pc = ((xx - mo[0]) / res).astype(np.int64)
            ok = (pc >= 0) & (pc < pw)
            val = np.full(w, 255, dtype=np.uint8)
            val[ok] = img[pr, pc[ok]]
            rgb[r, :, 0] = val
            rgb[r, :, 1] = val
            rgb[r, :, 2] = val
    rgb[occ] = (220, 40, 40)
    for p in poses_xy:
        c = int((p[0] - origin[0]) / cell)
        r = int((p[1] - origin[1]) / cell)
        if 0 <= r < h and 0 <= c < w:
            rgb[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = (0, 160, 0)
    # 图像上下翻转（格图 r 向上 = 世界 y 向上；PNG 行向下）
    write_png_rgb(path, rgb[::-1])


def ascii_map(occ, poses_xy, origin, cell, width):
    h, w = occ.shape
    step = max(1, int(math.ceil(w / float(width))))
    rows = []
    for r in range(h - 1, -1, -step):
        line = []
        for c in range(0, w, step):
            blk = occ[max(0, r - step + 1):r + 1, c:c + step]
            line.append("#" if blk.any() else ".")
        rows.append("".join(line))
    # 位姿叠加
    for p in poses_xy:
        c = int((p[0] - origin[0]) / cell) // step
        r = int((p[1] - origin[1]) / cell) // step
        row_i = (h - 1 - int((p[1] - origin[1]) / cell)) // step
        if 0 <= row_i < len(rows) and 0 <= c < len(rows[row_i]):
            rows[row_i] = rows[row_i][:c] + "o" + rows[row_i][c + 1:]
    return rows


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------
def parse_band(s):
    """'0.20,inf' / '0.2:0.5' / '0.2' → (lo, hi)（地面以上高度，米）。"""
    t = s.replace(":", ",").split(",")
    lo = float(t[0])
    hi = float(t[1]) if len(t) > 1 else INF
    if lo >= hi:
        die(f"--height-band 下界 {lo} 不小于上界 {hi}")
    return lo, hi


def _check_tmp_path(path):
    ap = os.path.abspath(path)
    for root in TMP_ALLOWED:
        if ap == root or ap.startswith(root + os.sep):
            os.makedirs(os.path.dirname(ap), exist_ok=True)
            return ap
    die(f"拒绝写到 {ap}：只允许落在 {TMP_ALLOWED[0]}/ 或 {TMP_ALLOWED[1]}/ 下（本探针不改仓库文件）")


def build_parser():
    p = argparse.ArgumentParser(
        prog="scan_context_asset_probe.py",
        description="A1 资产可行性探针（plan §D.1）：先验 PCD 的墙线连续性 + 2D 极坐标距离描述子的"
                    "区分性。只读、numpy-only、不依赖 ROS。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例：
  # 主资产 + 对照（RMUL 有 14.4 m 竖直结构）
  python3 tools/scripts/analysis/scan_context_asset_probe.py \\
      --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd src/rm_nav_bringup/PCD/RMUL.pcd

  # 只跑主资产，改 bins/高度带，落 JSON + PNG 到 .tmp_cache
  python3 tools/scripts/analysis/scan_context_asset_probe.py \\
      --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd --bins 360 --height-band 0.20,0.35 \\
      --json .tmp_cache/sc_probe.json \\
      --render-png .tmp_cache/sc_probe_RMUL2026.png \\
      --map-png src/rm_nav_bringup/map/RMUL2026.pgm --map-resolution 0.05 --map-origin -2.2,-3.15

判读：墙长估计 vs 包围盒周长的一半；中位(互距离)/中位(自距离) ≥ 5 ⇒ 描述子可用；
最大角度缺口大 ⇒ 轮廓不闭合（射线会从缺口漏出去 ⇒ 检索退化）。
""")
    p.add_argument("--pcd", nargs="+", required=True,
                   help="一个或多个 PCD 路径（第一个通常是主资产，其余当对照）")
    p.add_argument("--floor-z", default="auto",
                   help="地面 z（米，PCD 自身坐标系）；'auto' = z 的 5%% 分位（与 plan §A.0 同法）。默认 auto")
    p.add_argument("--height-band", default="0.20,inf", type=parse_band,
                   help="地面以上的高度带 LO,HI（米，半开区间）；'inf' 可写 inf/1e9。默认 0.20,inf")
    p.add_argument("--cell", type=float, default=0.15, help="2D 格尺寸（米）。默认 0.15")
    p.add_argument("--max-radius", type=float, default=10.0,
                   help="描述子/射线最大半径（米），与 p2l 的 range_max 对齐。默认 10.0")
    p.add_argument("--bins", type=int, default=1440,
                   help="描述子角向 bin 数（360°/bins；1440 ⇒ 0.25° 与 /scan 的 0.0043 rad 同量级）。默认 1440")
    p.add_argument("--step-frac", type=float, default=0.5,
                   help="射线采样步长 = cell*step_frac（越小越不容易穿缝，越慢）。默认 0.5")
    p.add_argument("--n-poses", type=int, default=20, help="查询位姿数。默认 20")
    p.add_argument("--pose-clearance", type=float, default=1.5,
                   help="查询位姿到最近墙的最小 4 邻域测地距离（米）。默认 1.5")
    p.add_argument("--self-perturbations", type=int, default=8,
                   help="每个位姿的小扰动重建次数（自距离样本数）。默认 8")
    p.add_argument("--self-yaw-deg", type=float, default=2.0,
                   help="自距离扰动的 yaw 幅度（±度）。默认 2.0（= ±0.5 格 xy ⇒ plan §D.1 A1.4）")
    p.add_argument("--yaw-search-step", type=float, default=1.0,
                   help="检索时的粗 yaw 搜索步长（度）。默认 1.0")
    p.add_argument("--yaw-scan-max", type=float, default=30.0,
                   help="yaw 鲁棒性曲线扫到多少度。默认 30")
    p.add_argument("--yaw-scan-step", type=float, default=1.0, help="yaw 曲线步长（度）。默认 1.0")
    p.add_argument("--n-ang-sweep", type=int, default=3600,
                   help="角度扫掠的射线数（最大缺口分辨率 = 360/N 度）。默认 3600")
    p.add_argument("--cross-far-m", type=float, default=2.0,
                   help="判定'是不同地点'的最小位姿间距（米）：比它近的对视为同一格/近邻，不计入互距离判据。默认 2.0")
    p.add_argument("--close-pad", type=int, default=10,
                   help="闭合性检验时把格图外扩多少格当'外面'（紧 bbox 会让'外面'退化）。默认 10")
    p.add_argument("--seed", type=int, default=0, help="RNG 种子。默认 0")
    p.add_argument("--json", default=None,
                   help="把结果写给一个 JSON（必须落在 .tmp_cache/ 或 .tmp_research/ 下）")
    p.add_argument("--render-png", default=None,
                   help="渲染顶视图 PNG（红=占用格，绿=查询位姿；必须落在 .tmp_cache/.tmp_research 下）")
    p.add_argument("--map-png", default=None, help="可选：叠加的 PGM 背景（如 map/RMUL2026.pgm）")
    p.add_argument("--map-resolution", type=float, default=None, help="--map-png 的分辨率（米/像素）")
    p.add_argument("--map-origin", default=None, help="--map-png 的 origin x,y（逗号分隔）")
    p.add_argument("--map-tol", type=float, default=0.20,
                   help="与 pgm 比重合度时的容差（米）。默认 0.20")
    p.add_argument("--label", default=None, help="结果里的名字（默认取 PCD 文件名）")
    p.add_argument("--ascii-map", action="store_true",
                   help="在 stdout 打印占用格 ASCII 图（'#'=占用，'o'=查询位姿）")
    p.add_argument("--ascii-width", type=int, default=100, help="ASCII 图宽度（列）。默认 100")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.map_origin:
        t = args.map_origin.replace(":", ",").split(",")
        args.map_origin = [float(t[0]), float(t[1])]
    results = []
    for path in args.pcd:
        r = probe_one(args, path)
        if args.ascii_map and r.get("usable"):
            occ_ = r["_occ"]
            origin_ = r["_origin"]
            log(f"  ── ASCII 顶视图（cell={args.cell:g} m, '#'=占用, 'o'=查询位姿, 上=+y, 右=+x）──")
            for line in ascii_map(occ_, np.asarray(r["poses_xy"]), origin_, args.cell,
                                  args.ascii_width):
                log("    " + line)
        results.append(r)

    if args.json:
        out = _check_tmp_path(args.json)
        clean = [{k: v for k, v in r.items() if not k.startswith("_")} for r in results]
        with open(out, "w", encoding="utf-8") as fh:
            json.dump({"args": {k: (list(v) if isinstance(v, tuple) else v)
                                for k, v in vars(args).items() if k != "pcd"},
                       "pcds": args.pcd, "results": clean}, fh, ensure_ascii=False, indent=2,
                      default=lambda o: None)
        log(f"[i] JSON 已写入 {out}")

    # 结论行
    log("=" * 100)
    log("结论速览（判据见 docs/scan_context_plan.md §D.1）")
    log("=" * 100)
    for r in results:
        if not r.get("usable"):
            log(f"  {r['label']:12s} 带内点不足 ⇒ **no-go**")
            continue
        w = r["wall"]
        ratio = (r["cross_distance_yawsearched"]["median"] / r["self_distance_raw"]["median"]
                 if r["self_distance_raw"]["median"] > 0 else float("inf"))
        wall_ok = w["wall_length_skeleton_m"] >= r["wall_length_criterion_half_bbox_perimeter_m"]
        desc_ok = ratio >= 5.0
        log(f"  {r['label']:12s} 墙长(骨架) {w['wall_length_skeleton_m']:6.2f} m "
            f"(判据 ≥ {r['wall_length_criterion_half_bbox_perimeter_m']:.1f} m) "
            f"{'PASS' if wall_ok else 'FAIL'} | 连通域(8) {w['components_8conn']:4d} | "
            f"最大缺口 {r['angular_sweep']['max_gap_deg']:6.2f}° | "
            f"互/自中位比 {ratio:5.2f}× {'PASS' if desc_ok else 'FAIL'} ⇒ "
            f"{'GO' if (wall_ok and desc_ok) else 'NO-GO'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
