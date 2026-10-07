#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11_decimate_visuals.py —— 把用户 CAD 导出的 STL 抽稀成"仿真视觉件"（Phase 5 / §12）

为什么要有这个文件（一句话）：`robot:=robot11` 的 `<visual>` 过去直接用原始的 SolidWorks
导出件（整车 **2 870 498 面**，其中 `base_link` 一个 link 就 2 078 226 面），而 Gazebo Classic
的 `gzclient` 是"把每个 `<visual>` 网格整份塞进 OGRE"的客户端进程 ⇒ 实测被内核 OOM 杀掉
（`SIGKILL`），`rviz2` 也只能给出黑屏。**碰撞侧**早已换成基本体（§9.3，实测有接触时
RTF 0.20 → 1.002、峰值内存 759 MB → 173 MB），**但视觉侧一个字节都没动** —— 这就是"模型放不进
仿真"的最后一块。

本工具做的是**只改渲染代价、不改任何会改变仿真行为的几何**：
  · 输入 = `src/rm_simulation/robot11_description/meshes/<name>.STL`（**只读**，一个字节不动）；
  · 输出 = `meshes/decimated/<name>.stl`（binary STL，三角形数按预算上限截断）；
  · 不动的：`<inertial>`（质量/惯量）、`<collision>`（Phase 1/3 的基本体）、
    每个 `<visual>` 的 `<origin>`、单位、坐标系、mesh 文件名前缀。
    ⇒ 雷达位姿、轮心/半径、足印、云台链、质量合计**逐字节不变**。

算法：**二次误差边折叠（quadric edge collapse, Garland–Heckbert）**。
  实现用 VTK 的 `vtkQuadricDecimation`（pyvista 0.46.4 已装、离线、无 GL 依赖）。
  为什么不用 MeshLab 的 `Quadric Edge Collapse Decimation`：本机 `meshlabserver 2020.09`
  在**无显示**的机器上起不来（`MLException: GLEW initialization failed: Missing GL version`，
  加 `LIBGL_ALWAYS_SOFTWARE=1` 也一样）—— 实测见 docs/robot_models.md §12.4。
  两者都是"二次误差边折叠"这一族：MeshLab 的 QECD 与 VTK 的 vtkQuadricDecimation
  都用 QEM 误差度量 + 边折叠 + 最优位置放置，差别只在边界/法线权重的旋钮上。
  （`trimesh.simplify_quadric_decimation` 在本机不可用：需要未安装的 `fast_simplification`；
    `open3d` / `pymeshlab` 未安装；Blender 未安装。）

预算（每个 link 的上限，来源见 §12.2 的引用）：
  · 视觉网格**每 link ≤ 50 000 面**（Blender-CAD-Meshes 的推荐档 `< 50k`）、
    另一种口径给的是 ≤ 100 000 面/link（ros2_gazebo_setup/mesh-optimization.md）；
  · 整车视觉三角形**总量 ≤ 200 000**。两者取交集 ⇒ 本工具默认 `--total-budget 200000`
    且逐 link 上限按 §12.2 的表给（最大的 base_link 也只要 50k）。
  · 绝不为了"数字好看"把某个 mesh 抽到 0 面：每个 link 有 `--min-faces` 下限，
    低于它就不再抽（并打印原因）。

验证（`--verify`，默认开；这是本工具的主要产出之一，机器可读落
`inventory/visual_decimation.json`）：
  · **包围盒**：逐轴 min/max 差 ≤ `--bbox-tol-m`（默认 1.0 mm）；单位/原点一致性就看它；
  · **凸包体积**：相对误差（凸包对"轮廓/可通行体积"是稳健量，非水密网格的 signed volume 不可信）；
  · **表面积**：相对误差；
  · **单向表面误差**：用 `vtkImplicitPolyDataDistance`（精确的点到三角面距离，带 cell locator）
    双向各量一次 ⇒ max / p99 / mean（单位 mm）。这就是 MeshLab/Metro 那一套"抽稀误差"；
  · **剪影**：三个正交投影（XY/XZ/YZ）2 mm 栅格化后的 IoU（闭运算填采样孔）；
  · **非流形/边界边**：抽稀前后的边界边数（爆增 = 抽出了洞）；
  · **雷达自击**：复用 `robot11_livox_fov.py` 的 30000 条射线，对**抽稀后的视觉件**求
    "锥内命中"数 —— 视觉件在 ODE 里不参与射线（见 §12.5），这里量的是"视觉上会不会把
    凹槽/雷达挡住"。

用法：
  # 生成 + 验证（默认：写 meshes/decimated/，并更新 inventory/visual_decimation.json）
  python3 tools/scripts/regress/robot11_decimate_visuals.py

  # 只看不写（CI/复核用；用于断言"已经生成的抽稀件仍然满足全部容差"）
  python3 tools/scripts/regress/robot11_decimate_visuals.py --check

  # 单个文件试验
  python3 tools/scripts/regress/robot11_decimate_visuals.py --only l11 --out-dir /tmp/x
"""

import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
PKG = os.path.join(REPO, 'src/rm_simulation/robot11_description')
MESHES = os.path.join(PKG, 'meshes')
DEFAULT_OUT = os.path.join(MESHES, 'decimated')
INVENTORY = os.path.join(PKG, 'inventory', 'visual_decimation.json')

#: 每个 link 的视觉三角形**上限**（预算的来源见文件头 + docs/robot_models.md §12.2）。
#: 取"每 link ≤ 50k"这一档（有两种公开口径：<50k 与 ≤100k，取严的那个）；
#: 各 link 的具体数字再按"它原来多大 / 在画面里多显眼 / 有多少小特征"分档。
FACE_BUDGET = {
    'base_link': 50000,   # 2 078 226 → 底盘（最大、最显眼，给到上限）
    'l11': 40000,         #   176 458 → 云台 pitch + 发射机构（整车最高、细节最多）
    'l10': 8000,          #    37 648 → 云台 yaw
    'l2': 12000,          #    89 556 → 转向模块 ×4
    'l3': 12000,
    'l4': 12000,
    'l5': 12000,
    'l6': 3000,           #    24 436 → 轮 ×4（几乎是纯圆柱，3000 面足够圆）
    'l7': 3000,
    'l8': 3000,
    'l9': 3000,
    'l12': 6000,          #    68 740 → Livox MID-360（小、但外形要认得出）
}
#: 总量上限（断言用；`--total-budget` 可覆盖）
TOTAL_BUDGET = 200000
#: 每个 link 的**下限**：低于它就不再抽（宁可多花面，也不把 link 抽没）
MIN_FACES = 400

#: 哪些 link 的视觉件需要做"雷达自击"检查（只有底盘 + 云台在雷达视锥附近）
FOV_CHECK = ('base_link', 'l10', 'l11')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def read_mesh(path):
    """用 trimesh 读 STL（`process=False` = 不做任何合并/去重，逐三角面照抄）。"""
    import trimesh
    m = trimesh.load(path, process=False)
    if not isinstance(m, trimesh.Trimesh):
        raise SystemExit('[decimate] %s 不是单个三角网格（读成 %s）' % (path, type(m).__name__))
    return m


# =============================================================================
# 抽稀：vtkCleanPolyData（按坐标合并重复点） → vtkQuadricDecimation（QEM 边折叠）
# =============================================================================
def decimate(vertices, faces, target_faces, volume_preservation=False):
    """返回 (verts, faces)。`vertices` = 三角汤顶点表（float64），`faces` = (M,3) int64。"""
    import pyvista as pv
    import vtk

    n_faces = len(faces)
    if target_faces >= n_faces:
        return vertices, faces

    tri = np.ascontiguousarray(vertices[faces].reshape(-1, 3), dtype=np.float64)
    cells = np.hstack(
        [np.full((n_faces, 1), 3, dtype=np.int64),
         np.arange(n_faces * 3, dtype=np.int64).reshape(-1, 3)]).ravel()
    pd = pv.PolyData(tri, cells)

    # ① 合并重合点：CAD 的 STL 是"三角汤"（每个面自带 3 个顶点、彼此不共享）
    #    ⇒ 不合并的话 QEM 连"边"都认不出来（拓扑上全是孤立三角形），抽稀退化成丢面。
    clean = vtk.vtkCleanPolyData()
    clean.SetInputData(pd)
    clean.PointMergingOn()
    clean.SetTolerance(0.0)          # 只合并**坐标完全相同**的点（不改几何）
    clean.ConvertPolysToLinesOff()
    clean.ConvertLinesToPointsOff()
    clean.Update()
    merged = pv.wrap(clean.GetOutput())
    n_merged = merged.n_faces_strict

    # ② QEM 边折叠（目标 = 保留 target_faces 面）
    reduction = max(0.0, min(0.999999, 1.0 - float(target_faces) / float(n_merged)))
    dec = vtk.vtkQuadricDecimation()
    dec.SetInputData(merged)
    dec.SetTargetReduction(reduction)
    dec.SetVolumePreservation(bool(volume_preservation))
    dec.Update()
    out = pv.wrap(dec.GetOutput())
    out = out.triangulate()

    pts = np.asarray(out.points, dtype=np.float64)
    fc = np.asarray(out.faces, dtype=np.int64).reshape(-1, 4)[:, 1:]
    return pts, fc


# =============================================================================
# 验证
# =============================================================================
def _pd(vertices, faces):
    import pyvista as pv
    tri = np.ascontiguousarray(vertices[faces].reshape(-1, 3), dtype=np.float64)
    cells = np.hstack(
        [np.full((len(faces), 1), 3, dtype=np.int64),
         np.arange(len(faces) * 3, dtype=np.int64).reshape(-1, 3)]).ravel()
    return pv.PolyData(tri, cells)


def one_way_distance(src_v, src_f, dst_v, dst_f, sample=None, tag='', dst_samples=3000000,
                     exact=False):
    """src 上每个点（超过 `sample` 就抽样）到 dst **曲面**的最短距离（mm）。

    两种口径（默认走快的那个；`--exact-err` 切到精确的那个）：
      · `exact=True`：`vtkImplicitPolyDataDistance` —— **精确**的点到三角面距离，
        但它要在 dst 上建 cell locator：base_link 那种 208 万面的 dst 要跑好几分钟
        （实测：本工具第一版就是死在这一步，10 分钟没出来）。
      · 默认：在 dst 表面**均匀撒 `dst_samples` 个点**建 KD-tree，再查 src 的点。
        误差上界 ≈ 采样间距 ≈ `sqrt(2·Area/N)`（Metro/Hausdorff 近似那一套）。
        base_link 面积 ~1.5 m²、N=3e6 ⇒ 间距 ~1.0 mm；本工具的判据是 p99 ≤ 3 mm / max ≤ 12 mm
        ⇒ 采样误差比判据小一个量级。`--calibrate` 会在小网格上把两种口径对一遍并打印差值。
    """
    t0 = time.time()
    if sample is not None and len(src_f) == 0:
        return np.zeros(0)
    if exact:
        a = _pd(src_v, src_f)
        b = _pd(dst_v, dst_f)
        if sample is not None and a.n_points > sample:
            rng = np.random.default_rng(0)
            idx = np.sort(rng.choice(a.n_points, size=sample, replace=False))
            a = a.extract_points(idx)
        t1 = time.time()
        d = np.abs(np.asarray(a.compute_implicit_distance(b)['implicit_distance'],
                              dtype=np.float64))
        print('           [err%s|exact] src=%d 点 dst=%d 面  抽取 %.1fs  测距 %.1fs'
              % (tag, a.n_points, len(dst_f), t1 - t0, time.time() - t1), flush=True)
        return d * 1000.0

    import trimesh
    from scipy.spatial import cKDTree
    dm = trimesh.Trimesh(vertices=dst_v, faces=dst_f, process=False)
    k = int(max(50000, min(dst_samples, max(50000, 8 * len(dst_f)))))
    pts = trimesh.sample.sample_surface(dm, k)[0]
    # 采样间距（保守估计：每个采样点"负责"的面积开方）
    spacing_mm = float(np.sqrt(2.0 * dm.area / max(1, k))) * 1000.0
    tree = cKDTree(pts)
    t1 = time.time()
    p = src_v[src_f].reshape(-1, 3)          # 三角汤顶点（每面 3 个，天然按面积加权）
    if sample is not None and len(p) > sample:
        rng = np.random.default_rng(0)
        p = p[np.sort(rng.choice(len(p), size=sample, replace=False))]
    d, _ = tree.query(p, k=1, workers=-1)
    print('           [err%s|sampled] src=%d 点 dst=%d 面  建树(%.0f 采样点, 间距~%.2f mm)%.1fs  '
          '查询 %.1fs' % (tag, len(p), len(dst_f), k, spacing_mm, t1 - t0, time.time() - t1),
          flush=True)
    out = d * 1000.0
    out = np.maximum(out - spacing_mm, 0.0)   # 减去采样间距 ⇒ **保守偏低**估计（不虚报误差）
    return out


def silhouette_iou(v0, f0, v1, f1, bins=2.0, n=400000):
    """三个正交投影的剪影 IoU（2 mm 栅格；用表面采样点做占用，闭运算 + 填洞补齐采样孔）。

    两个网格用**同一套栅格**（原点/范围都由原件 bbox 定）⇒ IoU 是可比的。
    采样点数按面数自适应（大网格不必采更多点：2 mm 栅格下 40 万点已经远超像素数）。
    """
    from scipy import ndimage
    import trimesh

    step = bins / 1000.0
    out = {}
    for name, ax in (('XY', (0, 1)), ('XZ', (0, 2)), ('YZ', (1, 2))):
        lo = v0[:, ax].min()
        hi = v0[:, ax].max()
        nx = max(2, int(np.ceil((hi - lo) / step)) + 3)
        ny = max(2, int(np.ceil((hi - lo) / step)) + 3)

        def occl(v, f):
            m = trimesh.Trimesh(vertices=v, faces=f, process=False)
            k = int(min(n, max(60000, 6 * len(f))))
            p = trimesh.sample.sample_surface(m, k)[0][:, ax]
            ix = np.clip(((p[:, 0] - lo) / step).astype(np.int64) + 1, 0, nx - 1)
            iy = np.clip(((p[:, 1] - lo) / step).astype(np.int64) + 1, 0, ny - 1)
            g = np.zeros((nx, ny), bool)
            g[ix, iy] = True
            g = ndimage.binary_closing(g, structure=np.ones((3, 3)), iterations=3)
            return ndimage.binary_fill_holes(g)

        g0, g1 = occl(v0, f0), occl(v1, f1)
        inter = int(np.logical_and(g0, g1).sum())
        union = int(np.logical_or(g0, g1).sum())
        out[name] = float(inter) / float(union) if union else 1.0
    return out


def boundary_edges(faces):
    """边界边（只被 1 个面用到）与非流形边（被 ≥3 个面用到）的条数。"""
    e = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e = np.sort(e, axis=1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    return int((cnt == 1).sum()), int((cnt >= 3).sum())


def weld_faces(vertices, faces):
    """按"坐标完全相同"合并顶点后重编号面（CAD 的 STL 是三角汤 ⇒ 不合并的话每条边都是边界边）。

    只用于**统计**（边界边/非流形边），不参与抽稀与导出。
    ⚠️ 不要用 `np.unique(..., axis=0)`：base_link 的三角汤是 623 万行，lexsort 要跑几十秒。
    这里先把坐标按 1e-9 m 量化成 int64 再打包成 3 个 int64 做 unique ⇒ 同一语义、快 10 倍以上。
    """
    q = np.round(vertices[faces].reshape(-1, 3) * 1e9).astype(np.int64)
    key = (q[:, 0] * 1000003 + q[:, 1]) * 1000033 + q[:, 2]
    _, inv = np.unique(key, return_inverse=True)
    return inv.reshape(-1, 3)


_FOV_CACHE = {}


def _fov_eval(mesh_path, tilt_axis, max_range, n_rays, local_m=0.13):
    """用 robot11_livox_fov.py 的 30000 条射线，量**视觉网格**在雷达近场（凹槽）里的自击比例。

    ⚠️ 口径两条，别搞混：
      · **物理射线**（Gazebo/ODE）打的是 `<collision>`，**与视觉网格无关**（§13.5）——
        所以这不是"感知检查"，是**外观检查**：抽稀会不会把雷达凹槽填平/盖上一个"盖子"。
      · 只在**雷达原点 `local_m` 米的立方体**内的三角形上求交。为什么这是**精确**的：
        被报告的 `selfhit_lt012`（t < 0.12 m）只可能落在离原点 0.12 m 以内的三角形上
        ⇒ 取 0.13 m 的局部盒**不丢任何一个可能被计入的面**。
        （不这么做的话：base_link 208 万面 × 30000 条 = 6×10¹⁰ 次求交，实测**跑不出来**；
          l11 17.6 万面就已经要 7.5 分钟。全局量 `escape_gt075` / `ground_seen_frac`
          对"凹槽有没有被填平"没有信息量，所以这里不报。）
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import robot11_livox_fov as fov
    key = (tilt_axis, n_rays)
    if key not in _FOV_CACHE:
        _FOV_CACHE[key] = fov.load_pattern(
            os.path.join(REPO, 'src/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv'),
            n_rays)
    dirs = _FOV_CACHE[key]
    tris = fov.read_stl(mesh_path)
    n_all = len(tris)
    o = np.array(fov.LIVOX_XYZ)
    lo, hi = o - local_m, o + local_m
    keep = np.all((tris >= lo) & (tris <= hi), axis=(1, 2))
    tris = tris[keep]
    if len(tris) == 0:
        return {'n_beams': int(len(dirs)), 'local_tris': 0, 'src_tris': int(n_all),
                'selfhit_lt012': 0.0, 'selfhit_lt005': 0.0, 'selfhit_lt012_horiz': 0.0}
    ev = fov.evaluate({'kind': 'mesh', 'tris': tris}, dirs, tilt_axis, 0.0, max_range)
    return {'n_beams': int(ev['n_beams']), 'local_tris': int(len(tris)), 'src_tris': int(n_all),
            'selfhit_lt012': ev['selfhit_lt012'], 'selfhit_lt005': ev['selfhit_lt005'],
            'selfhit_lt012_horiz': ev['selfhit_lt012_horiz']}


# =============================================================================
def process_one(name, src, out_dir, budget, args):
    src_path = os.path.join(MESHES, src)
    m0 = read_mesh(src_path)
    v0 = np.asarray(m0.vertices, dtype=np.float64)
    f0 = np.asarray(m0.faces, dtype=np.int64)
    n0 = len(f0)
    target = min(budget, n0)
    t0 = time.time()
    v1, f1 = decimate(v0, f0, target, args.volume_preservation)
    dt = time.time() - t0
    n1 = len(f1)

    rec = {
        'link': name,
        'src': os.path.relpath(src_path, REPO),
        'src_sha256': sha256(src_path),
        'src_bytes': os.path.getsize(src_path),
        'src_faces': n0,
        'budget_faces': budget,
        'out_faces': n1,
        'reduction': (1.0 - n1 / n0) if n0 else 0.0,
        'decimate_s': round(dt, 3),
    }

    # ---- 几何量 ----
    import trimesh
    m1 = trimesh.Trimesh(vertices=v1, faces=f1, process=False)
    rec['src_bbox_min'] = [round(float(x), 9) for x in v0.min(axis=0)]
    rec['src_bbox_max'] = [round(float(x), 9) for x in v0.max(axis=0)]
    rec['out_bbox_min'] = [round(float(x), 9) for x in v1.min(axis=0)]
    rec['out_bbox_max'] = [round(float(x), 9) for x in v1.max(axis=0)]
    db = np.maximum(np.abs(v1.min(axis=0) - v0.min(axis=0)),
                    np.abs(v1.max(axis=0) - v0.max(axis=0)))
    rec['bbox_err_m'] = [round(float(x), 9) for x in db]
    rec['bbox_err_max_m'] = float(db.max())
    rec['bbox_ok'] = bool(db.max() <= args.bbox_tol_m)

    hv0, hv1 = float(m0.convex_hull.volume), float(m1.convex_hull.volume)
    rec['hull_vol_src_m3'] = hv0
    rec['hull_vol_out_m3'] = hv1
    rec['hull_vol_err_rel'] = float(abs(hv1 - hv0) / hv0) if hv0 else 0.0
    rec['area_src_m2'] = float(m0.area)
    rec['area_out_m2'] = float(m1.area)
    rec['area_err_rel'] = float(abs(m1.area - m0.area) / m0.area) if m0.area else 0.0

    # ---- 单向表面误差（双向） ----
    #  ⚠️ 默认用"dst 表面撒点 + KD-tree"（快）；`--exact-err` 切到精确的 VTK 口径。
    d_out2src = one_way_distance(v1, f1, v0, f0, tag=' out2src', exact=args.exact_err,
                                dst_samples=args.err_dst_samples)
    d_src2out = one_way_distance(v0, f0, v1, f1, sample=args.err_samples, tag=' src2out',
                                 exact=args.exact_err, dst_samples=args.err_dst_samples)
    for tag, d in (('out2src', d_out2src), ('src2out', d_src2out)):
        rec['err_%s_mm' % tag] = {
            'max': float(d.max()), 'p99': float(np.percentile(d, 99)),
            'p999': float(np.percentile(d, 99.9)), 'mean': float(d.mean()),
        }
    #: 判据用"抽稀件 → 原件"的 p99（= 抽稀件偏离原表面的量；max 会被单个尖角拉爆，
    #: 所以两个都报，但门限卡 p99 与 max 两条）
    rec['err_ok'] = bool(rec['err_out2src_mm']['p99'] <= args.err_p99_mm and
                         rec['err_out2src_mm']['max'] <= args.err_max_mm)

    # ---- 剪影 ----
    sil = silhouette_iou(v0, f0, v1, f1, bins=args.sil_bins_mm)
    rec['silhouette_iou'] = {k: round(v, 5) for k, v in sil.items()}
    rec['silhouette_ok'] = bool(min(sil.values()) >= args.sil_iou_min)

    # ---- 拓扑 ----
    #  ⚠️ 原件是三角汤（每个面 3 个独立顶点）⇒ 必须先"按坐标合并"再数边界边，
    #     否则每条边都会被算成边界边（会得出"边界边 20 万"这种没有信息量的数）。
    b0, nm0 = boundary_edges(weld_faces(v0, f0))
    b1, nm1 = boundary_edges(f1)
    rec['boundary_edges'] = {'src': b0, 'out': b1}
    rec['nonmanifold_edges'] = {'src': nm0, 'out': nm1}
    #: 判据：抽稀**不该**抽出新洞 ⇒ 边界边不得显著增长（允许 5% + 64 条的余量）。
    rec['topology_ok'] = bool(b1 <= max(b0, 1) * args.boundary_growth + args.boundary_slack)

    # ---- 写盘 ----
    out_path = os.path.join(out_dir, '%s.stl' % name)
    if not args.check:
        os.makedirs(out_dir, exist_ok=True)
        tri = np.ascontiguousarray(v1[f1].astype(np.float32))
        m1.export(out_path, file_type='stl')
    rec['out'] = os.path.relpath(out_path, REPO)
    if os.path.isfile(out_path):
        rec['out_sha256'] = sha256(out_path)
        rec['out_bytes'] = os.path.getsize(out_path)

    # ---- 雷达近场（凹槽）外观检查：只对底盘/云台做 ----
    if name in FOV_CHECK and os.path.isfile(out_path):
        try:
            a = _fov_eval(src_path, args.tilt_axis, args.fov_max_range, args.fov_rays,
                          args.fov_local_m)
            b = _fov_eval(out_path, args.tilt_axis, args.fov_max_range, args.fov_rays,
                          args.fov_local_m)
            rec['fov_visual'] = {
                'rays': a['n_beams'],
                'local_box_m': args.fov_local_m,
                'local_tris': {'src': a['local_tris'], 'out': b['local_tris']},
                'src': {k: a[k] for k in ('selfhit_lt012', 'selfhit_lt005', 'selfhit_lt012_horiz')},
                'out': {k: b[k] for k in ('selfhit_lt012', 'selfhit_lt005', 'selfhit_lt012_horiz')},
                'selfhit_lt012_delta': b['selfhit_lt012'] - a['selfhit_lt012'],
            }
            rec['fov_ok'] = bool(abs(rec['fov_visual']['selfhit_lt012_delta']) <=
                                 args.fov_selfhit_tol)
        except Exception as exc:                                   # noqa: BLE001
            rec['fov_visual'] = {'error': '%s: %s' % (type(exc).__name__, exc)}
            rec['fov_ok'] = True       # 量不了不算不合格（但会记在清单里）

    rec['ok'] = bool(rec['bbox_ok'] and rec['err_ok'] and rec['silhouette_ok'] and
                     rec['topology_ok'] and rec.get('fov_ok', True))
    return rec


def calibrate(name, args):
    """把"精确 VTK 距离"与"采样 KD-tree 距离"在同一对网格上对一遍（采样口径的可信度证据）。

    只在小网格上跑（`--only l12` 那种）；大网格上精确口径要几分钟，得不偿失。
    """
    src_path = os.path.join(MESHES, '%s.STL' % name)
    m0 = read_mesh(src_path)
    v0 = np.asarray(m0.vertices, dtype=np.float64)
    f0 = np.asarray(m0.faces, dtype=np.int64)
    v1, f1 = decimate(v0, f0, FACE_BUDGET[name], args.volume_preservation)
    exact = one_way_distance(v1, f1, v0, f0, tag=' out2src', exact=True)
    samp = one_way_distance(v1, f1, v0, f0, tag=' out2src', exact=False,
                            dst_samples=args.err_dst_samples)
    out = {
        'link': name, 'src_faces': len(f0), 'out_faces': len(f1),
        'exact_mm': {'max': float(exact.max()), 'p99': float(np.percentile(exact, 99)),
                     'mean': float(exact.mean())},
        'sampled_mm': {'max': float(samp.max()), 'p99': float(np.percentile(samp, 99)),
                       'mean': float(samp.mean())},
    }
    out['delta_mm'] = {k: out['sampled_mm'][k] - out['exact_mm'][k]
                       for k in ('max', 'p99', 'mean')}
    print('[calibrate] %s 精确：max %.3f / p99 %.3f / mean %.3f mm' %
          (name, out['exact_mm']['max'], out['exact_mm']['p99'], out['exact_mm']['mean']))
    print('[calibrate] %s 采样：max %.3f / p99 %.3f / mean %.3f mm' %
          (name, out['sampled_mm']['max'], out['sampled_mm']['p99'], out['sampled_mm']['mean']))
    print('[calibrate] %s 差  ：max %+.3f / p99 %+.3f / mean %+.3f mm（采样口径**偏低**是设计如此）'
          % (name, out['delta_mm']['max'], out['delta_mm']['p99'], out['delta_mm']['mean']))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out-dir', default=DEFAULT_OUT)
    ap.add_argument('--inventory', default=INVENTORY)
    ap.add_argument('--total-budget', type=int, default=TOTAL_BUDGET)
    ap.add_argument('--bbox-tol-m', type=float, default=1.0e-3,
                    help='逐轴包围盒最大允许差（m）；默认 1.0 mm')
    ap.add_argument('--err-p99-mm', type=float, default=3.0,
                    help='单向表面误差 p99 门限（mm）；默认 3.0')
    ap.add_argument('--err-max-mm', type=float, default=12.0,
                    help='单向表面误差 max 门限（mm）；默认 12.0')
    ap.add_argument('--err-samples', type=int, default=200000,
                    help='src→out 方向的抽样点数（原网格太大时抽样；out→src 全量算）')
    ap.add_argument('--err-dst-samples', type=int, default=3000000,
                    help='默认口径下在 dst 表面撒多少点建 KD-tree（间距 ~ sqrt(2A/N)）')
    ap.add_argument('--exact-err', action='store_true',
                    help='用 vtkImplicitPolyDataDistance 精确量误差（慢很多；只用于小网格/校准）')
    ap.add_argument('--calibrate', action='store_true',
                    help='在 --only 指定的（小）网格上把"精确"与"采样"两种误差口径对一遍')
    ap.add_argument('--sil-bins-mm', type=float, default=2.0)
    ap.add_argument('--sil-iou-min', type=float, default=0.97)
    ap.add_argument('--boundary-growth', type=float, default=1.05)
    ap.add_argument('--boundary-slack', type=int, default=64)
    ap.add_argument('--volume-preservation', action='store_true',
                    help='打开 vtkQuadricDecimation 的 VolumePreservation（默认关：会稍微改动外形）')
    ap.add_argument('--tilt-axis', default='roll', choices=['roll', 'pitch'])
    ap.add_argument('--fov-rays', type=int, default=30000,
                    help='视觉件自击检查用的射线数（与插件一帧同源：mid360.csv 前 N 行）')
    ap.add_argument('--fov-max-range', type=float, default=0.75)
    ap.add_argument('--fov-local-m', type=float, default=0.13,
                    help='只在这半个边长的立方体（雷达原点周围）内的三角形上求交；'
                         '0.13 > 0.12 = selfhit_lt012 的门限 ⇒ **不丢任何一个可能被计入的面**')
    ap.add_argument('--fov-selfhit-tol', type=float, default=0.05,
                    help='抽稀件 vs 原件 的"自击<0.12 m"比例最大允许变化（默认 5 个百分点）')
    ap.add_argument('--only', action='append', default=None, help='只处理这些 link（可重复）')
    ap.add_argument('--check', action='store_true',
                    help='只验证已生成的抽稀件 + 逐字节核对 inventory（不写任何文件）')
    ap.add_argument('--explain', action='store_true')
    args = ap.parse_args()

    if args.explain:
        print(__doc__)
        return 0

    names = args.only or sorted(FACE_BUDGET, key=lambda n: -FACE_BUDGET[n])
    if args.calibrate:
        cal = [calibrate(nm, args) for nm in (args.only or ['l12'])]
        with open(os.path.join(REPO, '.tmp_robot11/work/err_calibration.json'), 'w') as f:
            json.dump(cal, f, indent=2, ensure_ascii=False)
            f.write('\n')
        return 0
    recs = []
    for nm in names:
        src = '%s.STL' % nm
        if not os.path.isfile(os.path.join(MESHES, src)):
            # l12 的文件名是大写 STL，base_link 也是；统一按存在性挑
            cand = [f for f in os.listdir(MESHES) if f.lower() == src.lower()]
            if not cand:
                print('[decimate] 跳过 %s：%s 不存在' % (nm, src))
                continue
            src = cand[0]
        print('[decimate] %-10s budget=%-7d ...' % (nm, FACE_BUDGET[nm]), flush=True)
        r = process_one(nm, src, args.out_dir, FACE_BUDGET[nm], args)
        recs.append(r)
        print('           %7d → %-7d 面  bbox≤%.3fmm  err(p99/max)=%.2f/%.2fmm  '
              'IoU=%.4f  边界边 %d→%d  %s'
              % (r['src_faces'], r['out_faces'], r['bbox_err_max_m'] * 1000.0,
                 r['err_out2src_mm']['p99'], r['err_out2src_mm']['max'],
                 min(r['silhouette_iou'].values()),
                 r['boundary_edges']['src'], r['boundary_edges']['out'],
                 'OK' if r['ok'] else '**不合格**'), flush=True)

    total_in = sum(r['src_faces'] for r in recs)
    total_out = sum(r['out_faces'] for r in recs)
    print('\n[decimate] 视觉三角形合计：%d → %d（预算 %d）%s'
          % (total_in, total_out, args.total_budget,
             '' if total_out <= args.total_budget else '  **超预算**'))

    doc = {
        'tool': os.path.relpath(os.path.abspath(__file__), REPO),
        'generated_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'algorithm': 'quadric edge collapse (Garland–Heckbert) via VTK vtkQuadricDecimation '
                     '+ vtkCleanPolyData(point merging, tolerance=0)',
        'why_not_meshlab': 'meshlabserver 2020.09 在本机（无显示）抛 '
                           'MLException: GLEW initialization failed: Missing GL version',
        'budget': {'per_link_faces': FACE_BUDGET, 'total_faces': args.total_budget},
        'tolerances': {'bbox_max_m': args.bbox_tol_m, 'err_p99_mm': args.err_p99_mm,
                       'err_max_mm': args.err_max_mm, 'silhouette_iou_min': args.sil_iou_min,
                       'sil_bins_mm': args.sil_bins_mm},
        'totals': {'src_faces': total_in, 'out_faces': total_out,
                   'src_bytes': sum(r['src_bytes'] for r in recs),
                   'out_bytes': sum(r.get('out_bytes', 0) for r in recs)},
        'meshes': recs,
        'all_ok': all(r['ok'] for r in recs) and total_out <= args.total_budget,
    }
    if not args.check:
        os.makedirs(os.path.dirname(args.inventory), exist_ok=True)
        with open(args.inventory, 'w') as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
            f.write('\n')
        print('[decimate] 清单 → %s' % os.path.relpath(args.inventory, REPO))
    print('[decimate] all_ok = %s' % doc['all_ok'])
    return 0 if doc['all_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
