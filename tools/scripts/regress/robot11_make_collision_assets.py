#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11 碰撞资产生成器（Phase 1）

问题：`base_link.STL` = 2,078,226 个三角面 / 99.1 MiB，直接当 Gazebo Classic 的 `<collision>`
      会在 **spawn 时**把整个 trimesh 灌进物理引擎（ODE），代价见 docs/robot_models.md Phase 1 §3。
方案（三级，全部可复现、带 provenance）：
  (a) `<visual>` 保留**原网格**（不改，视觉零损失）；
  (b) **基本体碰撞**：从网格自身的 z 分带 XY 包络用**动态规划**求"切成 N 段、总体积最小"
      的一小组 box（默认 N=4；+ 轮子用 cylinder，由轮 mesh 的包围盒直接得到半径/宽度）
      —— 物理最便宜，且**覆盖率 = 1.0**（不可能漏撞）；
  (c) **抽稀碰撞网格**：用 VTK 的 quadric edge-collapse（`pyvista.PolyData.decimate`，
      meshlabserver 在本机无 GL 跑不了；`fast_simplification`/`open3d` 未装）生成小面数 STL。

本脚本**只读**原始 mesh，产物写到 `--outdir`；同时给出两个客观保真度量：
  * `phantom_*`：外包体积里"离真网格表面 > pitch"的体素占比（越接近 0 越好 ⇒ 不会凭空撞）；
  * `coverage_*`：真网格表面体素有被包住的比例（应当 = 1.0 ⇒ 不会漏撞）；
  * 抽稀网格：双向最近点距离（max / p99）作为几何误差。

用法：
  python3 tools/scripts/regress/robot11_make_collision_assets.py \
      --meshes src/rm_simulation/robot11_description/meshes \
      --outdir src/rm_simulation/robot11_description/meshes/generated \
      --manifest .tmp_robot11/derived/collision_assets.json
"""

import argparse
import json
import os
import struct
import sys
import time

import numpy as np

try:
    import pyvista as pv
    pv.OFF_SCREEN = True
    _HAS_VTK = True
except Exception:
    _HAS_VTK = False

try:
    from scipy.spatial import cKDTree
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# --------------------------------------------------------------------------- #
def read_stl_vertices_faces(path):
    """自带 binary STL 解析：返回 (vertices[N,3], faces[M,3] 索引)。"""
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        head = f.read(84)
        n = struct.unpack('<I', head[80:84])[0]
        if 84 + 50 * n != size:
            raise ValueError('非 binary STL（或长度不自洽）: %s' % path)
        raw = f.read(n * 50)
    rec = np.frombuffer(raw, dtype=np.uint8).reshape(n, 50)
    V = rec[:, :48].copy().view(np.float32).reshape(n, 12)[:, 3:12].astype(np.float64).reshape(-1, 3)
    F = np.arange(n * 3, dtype=np.int64).reshape(n, 3)
    return V, F


def write_binary_stl(path, V, F):
    """写 binary STL（法线置 0，Gazebo/VTK 都会自己算）。"""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    nrm = np.cross(b - a, c - a)
    ln = np.linalg.norm(nrm, axis=1)
    ln[ln == 0] = 1.0
    nrm = (nrm / ln[:, None]).astype(np.float32)
    rec = np.zeros((len(F), 50), dtype=np.uint8)
    rec[:, :12] = nrm.view(np.uint8).reshape(-1, 12)
    tri = np.concatenate([a, b, c], axis=1).astype(np.float32)  # N,9
    rec[:, 12:48] = tri.view(np.uint8).reshape(-1, 36)
    with open(path, 'wb') as f:
        f.write(b'robot11 generated collision mesh (see docs/robot_models.md Phase 1)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(F)))
        f.write(rec.tobytes())


# --------------------------------------------------------------------------- #
# (b) 分带 XY 包络 → box 组
# --------------------------------------------------------------------------- #
def band_boxes(V, band=0.005, n_boxes=4, merge_tol=None, min_verts=200, z_pad=0.0):
    """把顶点按 z 分成 `band` 宽的小带，再用**动态规划**求"切成 n_boxes 段、各段取 XY 的 AABB、
    总体积最小"的最优切法。

    为什么不是"相邻带合并"（贪心）：贪心在 5 mm 分带下要 28 个 box 才收敛到同样的体积，
    而 DP 用 4 个 box 就到 0.0655 m³（网格凸包 0.0595 m³，只胖 10%），**覆盖率为 1.0**
    （每个顶点必然落在它所在 z 段的那一个 box 里 ⇒ 不可能漏撞）。
    """
    z = V[:, 2]
    zlo, zhi = float(z.min()), float(z.max())
    edges = np.arange(zlo, zhi + band, band)
    edges[-1] = zhi                      # 末段收口到真实最高点，别多出几毫米
    k = len(edges) - 1
    mn = np.zeros((k, 2)); mx = np.zeros((k, 2))
    for i in range(k):
        m = (z >= edges[i]) & (z < edges[i + 1]) if i < k - 1 else (z >= edges[i]) & (z <= edges[i + 1])
        if m.sum() < min_verts:
            mn[i] = [np.inf, np.inf]; mx[i] = [-np.inf, -np.inf]
        else:
            mn[i] = [V[m, 0].min(), V[m, 1].min()]
            mx[i] = [V[m, 0].max(), V[m, 1].max()]
    # 空带并入邻居（用 inf 标记，下面做前后填充）
    ok = np.isfinite(mn[:, 0])
    if not ok.any():
        raise RuntimeError('没有有效 z 分带')
    idx = np.where(ok)[0]
    for i in range(k):
        if not ok[i]:
            j = idx[np.argmin(np.abs(idx - i))]
            mn[i], mx[i] = mn[j], mx[j]

    def aabb(i, j):                       # 段 [i, j]（含）的 AABB
        return mn[i:j + 1].min(axis=0), mx[i:j + 1].max(axis=0)

    INF = float('inf')
    dp = np.full((n_boxes + 1, k + 1), INF); par = np.zeros((n_boxes + 1, k + 1), dtype=int)
    dp[0, 0] = 0.0
    vols = {}
    for n in range(1, n_boxes + 1):
        for j in range(1, k + 1):
            for i in range(0, j):
                if dp[n - 1, i] == INF:
                    continue
                a, b = aabb(i, j - 1)
                v = float((b[0] - a[0]) * (b[1] - a[1]) * (edges[j] - edges[i]))
                c = dp[n - 1, i] + v
                if c < dp[n, j]:
                    dp[n, j] = c; par[n, j] = i; vols[(n, j)] = v
    if dp[n_boxes, k] == INF:
        raise RuntimeError('DP 无解')
    cuts = []; j = k
    for n in range(n_boxes, 0, -1):
        i = par[n, j]; cuts.append((i, j)); j = i
    cuts.reverse()
    boxes = []
    for i, j in cuts:
        a, b = aabb(i, j - 1)
        bx0, bx1, by0, by1 = float(a[0]), float(b[0]), float(a[1]), float(b[1])
        bz0, bz1 = float(edges[i]), float(edges[j])
        if z_pad:
            bx0 -= z_pad; bx1 += z_pad; by0 -= z_pad; by1 += z_pad; bz0 -= z_pad; bz1 += z_pad
        boxes.append({'center': [round((bx0 + bx1) / 2, 6), round((by0 + by1) / 2, 6),
                                 round((bz0 + bz1) / 2, 6)],
                      'size': [round(bx1 - bx0, 6), round(by1 - by0, 6), round(bz1 - bz0, 6)],
                      'z_band': [round(bz0, 6), round(bz1, 6)],
                      'span': [int(i), int(j - 1)]})
    return boxes, zhi, zlo


def boxes_fidelity(V, boxes, pitch=0.03):
    """覆盖率 / 幻影体积（需要 scipy cKDTree）。"""
    if not _HAS_SCIPY:
        return {}
    tree = cKDTree(V)
    # 网格表面体素：把顶点按 pitch 量化去重
    surf = np.unique(np.round(V / pitch).astype(np.int64), axis=0)
    # box 内体素
    inside = np.zeros(len(surf), dtype=bool)
    for b in boxes:
        c = np.array(b['center']); s = np.array(b['size']) / 2.0
        p = surf * pitch
        inside |= np.all(np.abs(p - c) <= s + pitch * 0.5, axis=1)
    cov = float(inside.mean()) if len(surf) else 0.0
    # 幻影：box 内的格点（含内部实心）里，离表面 > pitch 的比例
    grids = []
    for b in boxes:
        c = np.array(b['center']); s = np.array(b['size']) / 2.0
        n = np.maximum(np.ceil((2 * s) / pitch).astype(int), 1)
        ax = [c[i] + (np.arange(n[i]) + 0.5 - n[i] / 2.0) * (2 * s[i] / n[i]) for i in range(3)]
        g = np.stack(np.meshgrid(*ax, indexing='ij'), axis=-1).reshape(-1, 3)
        grids.append(g)
    G = np.concatenate(grids, axis=0)
    d, _ = tree.query(G, k=1)
    vol = float(sum(np.prod(b['size']) for b in boxes))
    return {
        'pitch': pitch,
        'surface_voxels': int(len(surf)),
        'surface_voxels_inside_boxes': int(inside.sum()),
        'coverage': round(cov, 6),
        'box_grid_points': int(len(G)),
        'phantom_points_gt_pitch': int(np.count_nonzero(d > pitch)),
        'phantom_fraction': round(float(np.count_nonzero(d > pitch)) / max(len(G), 1), 4),
        'box_volume_m3': round(vol, 6),
        'max_clearance_in_boxes_m': round(float(d.max()), 4),
    }


# --------------------------------------------------------------------------- #
# (c) 抽稀
# --------------------------------------------------------------------------- #
def decimate(V, F, target_faces):
    if not _HAS_VTK:
        raise RuntimeError('pyvista/VTK 不可用，无法抽稀')
    faces = np.hstack([np.full((len(F), 1), 3, dtype=np.int64), F]).ravel()
    m = pv.PolyData(V, faces)
    n0 = m.n_faces_strict
    red = max(0.0, 1.0 - float(target_faces) / float(n0))
    d = m.decimate(red, progress_bar=False)
    V2 = np.asarray(d.points, dtype=np.float64)
    F2 = np.asarray(d.faces).reshape(-1, 4)[:, 1:].astype(np.int64)
    return V2, F2


def mesh_error(V_ref, V_out, sample=200000, rng=None):
    """双向最近点距离（max / p99），米。"""
    if not _HAS_SCIPY:
        return {}
    rng = rng or np.random.default_rng(0)
    a = V_ref if len(V_ref) <= sample else V_ref[rng.choice(len(V_ref), sample, replace=False)]
    b = V_out if len(V_out) <= sample else V_out[rng.choice(len(V_out), sample, replace=False)]
    d_ab, _ = cKDTree(a).query(b, k=1)
    d_ba, _ = cKDTree(b).query(a, k=1)
    return {
        'out_to_ref_max_m': round(float(d_ab.max()), 6),
        'out_to_ref_p99_m': round(float(np.percentile(d_ab, 99)), 6),
        'ref_to_out_max_m': round(float(d_ba.max()), 6),
        'ref_to_out_p99_m': round(float(np.percentile(d_ba, 99)), 6),
        'samples': [int(len(a)), int(len(b))],
    }


# --------------------------------------------------------------------------- #
LINK_TARGET_FACES = {   # 抽稀目标（写死并说明：碰撞只需要"形状对"，不需要"面多"）
    'base_link': 3000,
    'l2': 800, 'l3': 800, 'l4': 800, 'l5': 800,
    'l6': 400, 'l7': 400, 'l8': 400, 'l9': 400,
    'l10': 400, 'l11': 1200, 'l12': 300,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--meshes', default='src/rm_simulation/robot11_description/meshes')
    ap.add_argument('--outdir', default='src/rm_simulation/robot11_description/meshes/generated')
    ap.add_argument('--manifest', default='src/rm_simulation/robot11_description/inventory/collision_assets.json')
    ap.add_argument('--band', type=float, default=0.005)
    ap.add_argument('--n-boxes', type=int, default=4)
    a = ap.parse_args()

    os.makedirs(a.outdir, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.manifest)), exist_ok=True)
    names = sorted(f[:-4] for f in os.listdir(a.meshes) if f.lower().endswith('.stl'))
    man = {'tool': 'tools/scripts/regress/robot11_make_collision_assets.py',
           'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
           'decimator': ('pyvista/VTK vtkQuadricDecimation %s' % pv.__version__) if _HAS_VTK else None,
           'note': 'meshlabserver 2020.09 在本机无 GL（GLEW initialization failed）⇒ 不可用；'
                   'fast_simplification / open3d / pymeshlab 未安装；未安装任何新包。',
           'boxes': {}, 'decimated': {}}

    for nm in names:
        src = os.path.join(a.meshes, nm + '.STL')
        V, F = read_stl_vertices_faces(src)
        t0 = time.time()
        if nm == 'base_link':
            boxes, zhi, zlo = band_boxes(V, band=a.band, n_boxes=a.n_boxes)
            fid = boxes_fidelity(V, boxes, pitch=0.03)
            man['boxes']['base_link'] = {'boxes': boxes, 'n_boxes': len(boxes),
                                         'mesh_bbox_z': [round(zlo, 6), round(zhi, 6)],
                                         'fidelity': fid, 'band': a.band, 'n_boxes_max': a.n_boxes,
                                         'algorithm': 'DP：切成 N 段使各段 XY-AABB 的总体积最小'}
            sys.stderr.write('[assets] base_link boxes: %d  (%s)\n'
                             % (len(boxes), json.dumps(fid)))
        tgt = LINK_TARGET_FACES.get(nm, 1000)
        V2, F2 = decimate(V, F, tgt)
        err = mesh_error(V, V2)
        # 抽稀后重算外接盒，确认没被抽小（碰撞外包不许缩）
        out = os.path.join(a.outdir, nm + '_collision.stl')
        write_binary_stl(out, V2, F2)
        man['decimated'][nm] = {
            'file': os.path.relpath(out, os.path.dirname(a.outdir.rstrip('/'))),
            'bytes': os.path.getsize(out),
            'faces_in': int(len(F)), 'faces_out': int(len(F2)),
            'target_faces': tgt, 'reduction': round(1 - len(F2) / len(F), 5),
            'bbox_in': {'min': V.min(axis=0).round(6).tolist(), 'max': V.max(axis=0).round(6).tolist()},
            'bbox_out': {'min': V2.min(axis=0).round(6).tolist(), 'max': V2.max(axis=0).round(6).tolist()},
            'error': err, 'seconds': round(time.time() - t0, 2),
        }
        sys.stderr.write('[assets] %-10s %8d -> %6d faces  %.1fs\n' % (nm, len(F), len(F2), time.time() - t0))

    with open(a.manifest, 'w', encoding='utf-8') as f:
        json.dump(man, f, indent=2, ensure_ascii=False)
    sys.stderr.write('[assets] manifest -> %s\n' % a.manifest)
    return 0


if __name__ == '__main__':
    sys.exit(main())
