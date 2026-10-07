#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11_livox_fov.py —— 「这台车的雷达看得见什么」的**离线**量测 + 凹槽开口（Phase 3）

为什么要有这个工具（一句话）：Gazebo 里量一次要 2~4 分钟，而"雷达被自己的车挡了多少"
完全可以离线算准 —— 先离线把 A/B 方案筛到 1~2 个，再上 Gazebo 复核（Phase 3 的实测纪律）。

三种"车体几何"表示（互相对照，同一套射线、同一套判据）：
  · ``mesh``    —— 抽稀碰撞网格（``meshes/generated/base_link_collision.stl``，3000 面；
                   到原网格单向误差 max 26 mm / p99 18 mm，见 docs/robot_models.md §9.3）
                   ⇒ **表面**碰撞：能穿过上游 CAD 里凹槽/支架的开口（Phase 2 里"spawn 干净
                   但 100 s 无传感器数据"的那条路，Phase 3 查清了原因，见 §11）
  · ``boxes``   —— 现行 ``<collision>``：4 个 DP box（实心包络，把凹槽填实了）
  · ``carved``  —— Phase 3 的 A 方案：DP box **减去"雷达视锥"** 后再分解成 box
                   （= 「一组摆好位置的 box，使 30° 斜置雷达的视锥畅通」，见 ``--emit-carved``）

射线：**与 Gazebo 插件同源** —— 直接读 ``livox_laser_simulation_RO2/scan_mode/mid360.csv``
的 (Azimuth, Zenith)，按插件 ``livox_points_plugin.cpp`` 的构造 ``axis = q(0, zenith, azimuth)·x̂``
→ 传感器系仰角 = 90° − Zenith ∈ [−7.2°, +52.2°]（用 Phase 2 dump 的点云实测复核过：
实测仰角 [−8.4°, +52.1°]）。

判据（与 Gazebo 探针 ``robot_model_probe.py`` 同口径，便于逐项对照）：
  · ``自击 < 0.12 m``  —— 探针报的 "r<0.12 的点占 76%" 就是这个口径（传感器系水平半径），
     这里报**射线比例**（Gazebo 报的是点数比例；两者在自击为主时接近，见 --explain）
  · ``看见地面``       —— 射线在打到任何车体几何之前先穿过地面平面（body z = −0.102499，
                         由轮 mesh 最低点实测，见 §9.4）
  · ``逃逸``           —— 0.75 m 内什么车体几何都没打到

用法：
  # 三种几何并排对照（A/B 决策用）
  python3 tools/scripts/regress/robot11_livox_fov.py --compare

  # 生成 A 方案的 box 清单 → inventory/livox_fov_boxes.json（生成器 --body-collision carved 读它）
  python3 tools/scripts/regress/robot11_livox_fov.py --emit-carved

  # 试 B 方案（抬起雷达）：把雷达原点在 body 系里抬高 0.06 m
  python3 tools/scripts/regress/robot11_livox_fov.py --compare --livox-raise-m 0.06

只读：除 ``--emit-carved``（写一个 JSON 到包内 inventory/）之外不写任何文件。
"""

import argparse
import datetime
import hashlib
import json
import math
import os
import struct
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
PKG = os.path.join(REPO, 'src/rm_simulation/robot11_description')

#: 雷达原点（body 系）——上游 `body_to_livox` 的 xyz（逐字，见 urdf/upstream/robot11.urdf）
LIVOX_XYZ = (0.000561701465058485, 0.130915824456595, 0.15702816968305)
#: 30° 倾角的两个候选（上游两份材料矛盾，Phase 2 实测的是"最朝下的方位"；用户 2026-10-07 确认为 roll）
TILT_RPY = {'roll': (-0.523598775598293, 0.0, 0.0),
            'pitch': (0.0, -0.523598775598293, 0.0)}
#: 地面平面（body 系，实测：四个轮 mesh 最低点；见 docs/robot_models.md §9.4）
GROUND_Z = -0.102499
#: mid360 宏声明的垂直 FOV（本仓既有取值，仅用于 --cone 的口径说明）
FOV_ELEV_DEG = (-7.22, 55.22)


# --------------------------------------------------------------------------
# 几何：旋转、射线求交
# --------------------------------------------------------------------------
def rot_rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def read_stl(path):
    """自带 binary/ascii STL 解析（不装新包；与 tools/scripts/regress/robot11_mesh_inventory.py 同款）。"""
    with open(path, 'rb') as f:
        head = f.read(84)
        if len(head) < 84:
            raise ValueError('%s 太短' % path)
        n = struct.unpack('<I', head[80:84])[0]
        if 84 + 50 * n == os.path.getsize(path):
            data = np.fromfile(f, dtype=np.uint8).reshape(n, 50)
            tri = data[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)
            return tri
    import trimesh
    m = trimesh.load(path, process=False)
    return np.asarray(m.vertices, dtype=np.float64)[np.asarray(m.faces)]


def ray_boxes(origins, dirs, boxes):
    """射线 vs 一组 AABB（body 系）。返回每条第**一次打到盒子表面**的 t（无交 = inf）。

    ⚠️ 口径必须与 Gazebo/ODE 一致（否则离线预测与实测对不上，Phase 3 的第一版就踩了）：
    ODE 的 box 是**表面**（不是实心介质）⇒ 射线**起点在盒子内部**时，第一次"撞到"的是
    **出射面**（t_max），不是 0。Phase 2 实测"75.5% 的点 r<0.12 m"正是这个现象：
    雷达原点落在第 4 个 DP box **内部**（z=0.157 < box 顶 0.213），于是每条射线都先在
    0.03~0.2 m 处打到这个盒子的内壁。
    """
    lo = np.array([b['center'] for b in boxes]) - np.array([b['size'] for b in boxes]) / 2
    hi = np.array([b['center'] for b in boxes]) + np.array([b['size'] for b in boxes]) / 2
    t = np.full(len(origins), np.inf)
    for k in range(len(boxes)):
        o, d = origins, dirs
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo[k] - o) / d
            t2 = (hi[k] - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        inside = (tmin < 0.0) & (tmax > 0.0)
        entry = np.where(inside, tmax, np.maximum(tmin, 0.0))
        hit = inside | ((tmax >= np.maximum(tmin, 0.0)) & (tmax > 0.0))
        t = np.where(hit & (entry < t), entry, t)
    return t


def ray_mesh(origins, dirs, tris, chunk=40, max_t=2000.0):
    """射线 vs 三角网格（Möller–Trumbore，按三角形分块）。返回每条第**一次**命中的 t。"""
    t = np.full(len(origins), np.inf)
    v0 = tris[:, 0]
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    for i in range(0, len(tris), chunk):
        v0c, e1c, e2c = v0[i:i + chunk], e1[i:i + chunk], e2[i:i + chunk]
        pv = np.cross(dirs[:, None, :], e2c[None, :, :])
        det = np.einsum('ikj,kj->ik', pv, e1c)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tv = origins[:, None, :] - v0c[None, :, :]
        u = np.einsum('ikj,ikj->ik', tv, pv) * inv
        qv = np.cross(tv, e1c[None, :, :])
        v = np.einsum('ikj,ikj->ik', dirs[:, None, :], qv) * inv
        tt = np.einsum('ikj,ikj->ik', e2c[None, :, :], qv) * inv
        m = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (tt > 1e-6) & (tt < max_t)
        tt = np.where(m, tt, np.inf)
        t = np.minimum(t, tt.min(axis=1))
    return t


# --------------------------------------------------------------------------
# 雷达视锥（凹槽开口）
# --------------------------------------------------------------------------
def cone_axis(tilt_axis):
    """视锥的**轴**：radar 自身 +z 在 body 系里的方向（仰角 −7.22..+55.22 的那条带绕它张开）。"""
    R = rot_rpy(*TILT_RPY[tilt_axis])
    return R @ np.array([0.0, 0.0, 1.0])


def carve_mask(pts, axis, half_angle_deg, center):
    """pts（N,3）中落在"以 axis 为轴、半角 half_angle_deg 的锥"内的 True。"""
    d = pts - center
    n = np.linalg.norm(d, axis=1)
    ok = n > 1e-9
    cosang = np.zeros(len(pts))
    cosang[ok] = (d[ok] / n[ok, None]) @ axis
    return cosang >= math.cos(math.radians(half_angle_deg))


def decompose_boxes(occ, lo, res, min_run=1):
    """把占据体素（bool, (nx,ny,nz)）贪心分解成 box 列表（先按 z 分层做 2D 矩形覆盖，再跨层合并）。

    返回 [{center, size, n_voxels}]，尺寸用体素格点数（调用方换算成米）。
    """
    nx, ny, nz = occ.shape
    slabs = []
    for k in range(nz):
        sl = occ[:, :, k]
        rects = []
        free = sl.copy()
        for i in range(nx):
            j = 0
            while j < ny:
                if not free[i, j]:
                    j += 1
                    continue
                # 向右伸
                i2 = i
                while i2 + 1 < nx and free[i2 + 1, j]:
                    i2 += 1
                # 向下伸（同一 x 范围）
                j2 = j
                while j2 + 1 < ny and free[i:i2 + 1, j2 + 1].all():
                    j2 += 1
                free[i:i2 + 1, j:j2 + 1] = False
                if (i2 - i + 1) * (j2 - j + 1) >= min_run:
                    rects.append((i, j, i2, j2))
                j += 1
        slabs.append(rects)
    # 跨层合并：同一 (i,j,i2,j2) 的连续层合成一段
    boxes = []
    open_map = {}
    for k, rects in enumerate(slabs):
        cur = set(rects)
        for key in list(open_map.keys()):
            if key not in cur:
                i, j, i2, j2, k0 = open_map.pop(key)
                boxes.append({'i0': i, 'j0': j, 'i1': i2, 'j1': j2, 'k0': k0, 'k1': k - 1})
        for key in cur:
            if key not in open_map:
                open_map[key] = (key[0], key[1], key[2], key[3], k)
    for key, (i, j, i2, j2, k0) in open_map.items():
        boxes.append({'i0': i, 'j0': j, 'i1': i2, 'j1': j2, 'k0': k0, 'k1': nz - 1})
    res = np.asarray(res, dtype=float)
    if res.ndim == 0:
        res = np.repeat(res, 3)
    out = []
    for b in boxes:
        c = [lo[0] + (b['i0'] + b['i1'] + 1) / 2.0 * res[0],
             lo[1] + (b['j0'] + b['j1'] + 1) / 2.0 * res[1],
             lo[2] + (b['k0'] + b['k1'] + 1) / 2.0 * res[2]]
        s = [(b['i1'] - b['i0'] + 1) * res[0], (b['j1'] - b['j0'] + 1) * res[1],
             (b['k1'] - b['k0'] + 1) * res[2]]
        out.append({'center': [round(v, 6) for v in c], 'size': [round(v, 6) for v in s],
                    'n_voxels': (b['i1'] - b['i0'] + 1) * (b['j1'] - b['j0'] + 1) * (b['k1'] - b['k0'] + 1)})
    out.sort(key=lambda b: (-b['n_voxels'], b['center'][2]))
    return merge_boxes(out, float(np.max(res)))


def merge_boxes(boxes, res, tol=1e-9):
    """把"共面且另两维完全对齐"的 box 合并（体素网格上的精确合并）⇒ box 数降一个量级。

    Gazebo 的 ODE 每个 box 都很便宜，但**没必要**给 100 个盒子：合并后语义完全等价
    （仍是同一组轴对齐盒子的并集），只是数目少 ⇒ 生成物和文档都可读。
    """
    boxes = [dict(b) for b in boxes]
    changed = True
    while changed:
        changed = False
        for axis in (0, 1, 2):
            other = [k for k in (0, 1, 2) if k != axis]
            boxes.sort(key=lambda b: (round(b['center'][other[0]], 6),
                                      round(b['center'][other[1]], 6),
                                      round(b['center'][axis], 6)))
            i = 0
            while i < len(boxes):
                j = i + 1
                while j < len(boxes):
                    A, B = boxes[i], boxes[j]
                    same = all(abs(A['center'][k] - B['center'][k]) < 1e-6 and
                               abs(A['size'][k] - B['size'][k]) < 1e-6 for k in other)
                    if same:
                        a_lo = A['center'][axis] - A['size'][axis] / 2
                        a_hi = A['center'][axis] + A['size'][axis] / 2
                        b_lo = B['center'][axis] - B['size'][axis] / 2
                        b_hi = B['center'][axis] + B['size'][axis] / 2
                        if abs(a_hi - b_lo) < res / 2 + tol or abs(b_hi - a_lo) < res / 2 + tol:
                            lo = min(a_lo, b_lo)
                            hi = max(a_hi, b_hi)
                            C = {'center': list(A['center']), 'size': list(A['size']),
                                 'n_voxels': A.get('n_voxels', 0) + B.get('n_voxels', 0)}
                            C['center'][axis] = (lo + hi) / 2
                            C['size'][axis] = hi - lo
                            boxes[i] = C
                            boxes.pop(j)
                            changed = True
                            continue
                    j += 1
                i += 1
    for b in boxes:
        b['center'] = [round(v, 6) for v in b['center']]
        b['size'] = [round(v, 6) for v in b['size']]
    boxes.sort(key=lambda b: (-b['size'][0] * b['size'][1] * b['size'][2], b['center'][2]))
    return boxes


def carve_boxes(dp_boxes, tilt_axis, cone_r, margin_deg, voxel,
                livox_raise_m=0.0, inner_r=0.05):
    """A 方案：DP box 减去"雷达视锥"→ 体素 → box 分解。

    视锥定义（**这就是"凹槽开口"的形式化**）：
      顶点 = 雷达原点（可被 --livox-raise-m 抬高）；轴 = 雷达自身 +z（30° 斜）；半角 =
      (90° + |FOV 下界| + margin) —— 即"把雷达 FOV 那一条带（−7.22°…+55.22°）整个包住"
      所需的锥（对 360° 方位雷达 = 一个半角 ≈ 100° 的锥）；半径截断 cone_r。
      ⇒ 锥内 cone_r 以内**没有任何碰撞体** ⇒ FOV 里每一条射线都能出去。
    """
    lo = np.array(dp_boxes[0]['center']) - np.array(dp_boxes[0]['size']) / 2
    hi = np.array(dp_boxes[0]['center']) + np.array(dp_boxes[0]['size']) / 2
    for b in dp_boxes[1:]:
        lo = np.minimum(lo, np.array(b['center']) - np.array(b['size']) / 2)
        hi = np.maximum(hi, np.array(b['center']) + np.array(b['size']) / 2)
    # ⚠️ 网格**逐轴对齐包络**（每轴体素数 = round(尺寸/体素)，体素尺寸 = 尺寸/体素数）：
    #    否则 ceil() 会多出一格 ⇒ 车顶凭空长高 or 被裁掉（Phase 3 第一版实测：z 顶到 0.2221）。
    n = np.maximum(np.round((hi - lo) / voxel).astype(int), 1)
    res = (hi - lo) / n
    occ = np.zeros(tuple(n), dtype=bool)
    for b in dp_boxes:
        c = np.array(b['center'])
        s = np.array(b['size'])
        i0 = np.maximum(np.round((c - s / 2 - lo) / res).astype(int), 0)
        i1 = np.minimum(np.round((c + s / 2 - lo) / res).astype(int), n)
        occ[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] = True
    # 锥内挖空
    idx = np.stack(np.meshgrid(*[np.arange(k) for k in n], indexing='ij'), axis=-1)
    pts = lo + (idx + 0.5) * res
    center = np.array(LIVOX_XYZ) + np.array([0.0, 0.0, livox_raise_m])
    axis = cone_axis(tilt_axis)
    half = 90.0 + abs(FOV_ELEV_DEG[0]) + margin_deg     # ≈ 100.2°
    # **保守**判定：体素的 8 个角点 + 中心，只要有一个落在锥内就整格挖掉
    # （否则"格中心在锥外、格子却压在锥上"的体素会留下 ⇒ 锥内的射线仍可能被打到；
    #  这是"锥内保证无碰撞"这条不变量的实现细节，--voxel-m 越粗越需要它）
    inside = np.zeros(tuple(n), dtype=bool)
    for dx in (-0.5, 0.0, 0.5):
        for dy in (-0.5, 0.0, 0.5):
            for dz in (-0.5, 0.0, 0.5):
                q = pts + np.array([dx, dy, dz]) * res
                inside |= carve_mask(q.reshape(-1, 3), axis, half, center).reshape(tuple(n))
    dist = np.linalg.norm(pts - center, axis=-1)
    inside &= (dist <= cone_r + float(np.max(res)))
    # 顶点邻域**必须**也挖空：锥的顶点处"方向"是退化的（体素中心相对顶点的方向是任意的），
    # 不额外挖就会留下"包住雷达原点的那一格" ⇒ 每条射线在 ~6 mm 处打到自己（Phase 3 第一版实测）。
    inside |= (dist <= inner_r)
    occ &= ~inside
    # 把 box **裁回原始包络**：体素网格按 ceil() 铺，最后一格可能越出车体外形（实测 z 顶会到
    # 0.2221 > 0.213）⇒ 不裁就等于"车顶凭空高了 9 mm"，会污染与墙/低矮结构的接触几何。
    boxes = decompose_boxes(occ, lo, res)
    return boxes, {'half_angle_deg': half, 'cone_r': cone_r,
                                             'inner_clearance_r': inner_r,
                                             'voxel_xyz_m': [round(float(v), 6) for v in res],
                                             'center': list(center), 'axis': list(np.round(axis, 6)),
                                             'voxel_m': voxel, 'margin_deg': margin_deg}


# --------------------------------------------------------------------------
# 云台（l10/l11）的**细碰撞盒**：把"单个包围盒"换成"网格推出的几个盒子"
# --------------------------------------------------------------------------
# 为什么需要（Phase 3 实测）：云台的包围盒（list 里的 l10 0.147x0.138x0.071、
# l11 0.133x0.172x0.335）**离雷达只有 0.098~0.106 m**，而真网格最近的表面在
# 0.116~0.139 m；盒子又是**实心**的 ⇒ Gazebo 实测 34.7% 的射线打在盒面上（0.11~0.17 m），
# 而**用真网格求交只有 0.3%** 落在 0.12 m 以内 —— 也就是说"自击"绝大部分是**碰撞近似**造成的，
# 不是真几何。云台又正好在雷达的 360° 视场里（离得最近的部件），所以这一步直接影响
# "自击占不占主导"这条验收。
TURRET_LINKS = ('l10', 'l11')


def fk_poses(urdf_path, root='body'):
    """上游 URDF 的零位正运动学（返回 {link: (t, R)}，body 系）。"""
    import xml.etree.ElementTree as ET
    root_el = ET.parse(urdf_path).getroot()
    pose = {root: (np.zeros(3), np.eye(3))}
    joints = root_el.findall('joint')
    changed = True
    while changed:
        changed = False
        for j in joints:
            par, ch = j.find('parent').get('link'), j.find('child').get('link')
            if par in pose and ch not in pose:
                o = j.find('origin')
                xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0')).split()])
                rpy = [float(v) for v in (o.get('rpy', '0 0 0')).split()]
                pose[ch] = (pose[par][0] + pose[par][1] @ xyz, pose[par][1] @ rot_rpy(*rpy))
                changed = True
    return pose


def mesh_to_boxes(mesh_path, voxel=0.015, min_voxels=8, pad=0.005):
    """把 mesh 变成"实心近似"的 box 集合（体素 + 闭运算 + 填孔 + 贪心分解）。

    ⚠️ 诚实说明：原始 STL **没有一个水密**（§9.2）⇒ 不存在严格的"内部/外部"。
    这里用 3x3x3 闭运算 + 逐体素填洞得到"实心近似"，再分解成盒子 —— 它的**外表面**与网格
    单向误差 ~1 个体素（15 mm），体积远小于包围盒（实测 l10 0.00064 vs 0.00187 m³）。
    """
    from scipy import ndimage
    T = read_stl(mesh_path)
    V = T.reshape(-1, 3)
    lo = V.min(axis=0) - pad
    hi = V.max(axis=0) + pad
    n = np.maximum(np.round((hi - lo) / voxel).astype(int), 1)
    res = (hi - lo) / n
    occ = np.zeros(tuple(n), dtype=bool)
    idx = np.clip(np.floor((V - lo) / res).astype(int), 0, n - 1)
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3, 3)))
    occ = ndimage.binary_fill_holes(occ)
    boxes = [b for b in decompose_boxes(occ, lo, res) if b['n_voxels'] >= min_voxels]
    return boxes, lo, hi


def emit_turret_boxes(mesh_dir, urdf, out_path, voxel, min_voxels):
    import datetime as _dt
    pose = fk_poses(urdf)
    livox = np.array(LIVOX_XYZ)
    out = {'tool': 'tools/scripts/regress/robot11_livox_fov.py (--emit-turret-boxes)',
           'generated_at': _dt.datetime.now().isoformat(timespec='seconds'),
           'note': ('Phase 3：云台 l10/l11 的碰撞由"单个包围盒"换成"网格推出的 N 个 box"。'
                    '理由：包围盒离雷达只有 0.098~0.106 m 且是实心 ⇒ 实测 34.7% 的射线打在它面上；'
                    '真网格最近 0.116~0.139 m。这属于**仿真侧适配**（偏离上游的 collision=mesh），'
                    '逐条登记在 docs/robot_models.md §11。'),
           'params': {'voxel_m': voxel, 'min_voxels': min_voxels,
                      'method': 'voxel + binary_closing(3x3x3) + binary_fill_holes + 贪心 box 分解'},
           'links': {}}
    for nm in TURRET_LINKS:
        mp = os.path.join(mesh_dir, '%s.STL' % nm)
        boxes, lo, hi = mesh_to_boxes(mp, voxel, min_voxels)
        t, R = pose[nm]
        dmin = min(np.linalg.norm(np.maximum(
            np.abs(livox - (R @ np.array(b['center']) + t)) - np.array(b['size']) / 2, 0))
            for b in boxes)
        bb_c = (hi + lo) / 2
        bb_s = hi - lo
        d_bbox = np.linalg.norm(np.maximum(np.abs(livox - (R @ bb_c + t)) - bb_s / 2, 0))
        out['links'][nm] = {
            'mesh': os.path.relpath(mp, REPO),
            'mesh_sha256': hashlib.sha256(open(mp, 'rb').read()).hexdigest(),
            'n_boxes': len(boxes), 'boxes': boxes,
            'bbox_center': [round(float(v), 6) for v in bb_c],
            'bbox_size': [round(float(v), 6) for v in bb_s],
            'nearest_face_to_lidar_m': round(float(dmin), 4),
            'nearest_face_if_single_bbox_m': round(float(d_bbox), 4),
        }
        print('[turret] %s: %d box，离雷达最近面 %.4f m（单包围盒 %.4f m）'
              % (nm, len(boxes), dmin, d_bbox))
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print('[turret] 写出 %s' % os.path.relpath(out_path, REPO))


# --------------------------------------------------------------------------
# 射线统计
# --------------------------------------------------------------------------
def load_pattern(path, n_rows):
    import csv
    az, ze = [], []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for i, row in enumerate(r):
            if i >= n_rows:
                break
            az.append(float(row[1]))
            ze.append(float(row[2]))
    az = np.radians(np.array(az))
    el = np.radians(90.0 - np.array(ze))        # 插件构造下传感器系仰角 = 90° − Zenith
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1)
    return d


def evaluate(model, dirs_sensor, tilt_axis, livox_raise_m=0.0, max_range=0.75):
    """把传感器系射线转到 body 系，量：自击/逃逸/看见地面。"""
    R = rot_rpy(*TILT_RPY[tilt_axis])
    d_body = dirs_sensor @ R.T
    origin = np.array(LIVOX_XYZ) + np.array([0.0, 0.0, livox_raise_m])
    o = np.repeat(origin[None, :], len(d_body), axis=0)
    if model['kind'] == 'mesh':
        t = ray_mesh(o, d_body, model['tris'], max_t=max_range)
    else:
        t = ray_boxes(o, d_body, model['boxes'])
        t = np.where(t > max_range, np.inf, t)
    dz = d_body[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t_ground = np.where(dz < -1e-9, (origin[2] - GROUND_Z) / (-dz), np.inf)
    dz_g = np.where(np.abs(dz) > 1e-9, dz, 1e-9)
    horiz = t * np.hypot(d_body[:, 0], d_body[:, 1])
    return {
        'n_beams': int(len(t)),
        'selfhit_lt012': float(np.mean(t < 0.12)),
        'selfhit_lt005': float(np.mean(t < 0.05)),
        'selfhit_lt012_horiz': float(np.mean(horiz < 0.12)),
        'escape_gt075': float(np.mean(~np.isfinite(t))),
        'hit_median': float(np.median(t[np.isfinite(t)])) if np.isfinite(t).any() else None,
        'ground_seen_frac': float(np.mean(t > t_ground)),
        'ground_range_median': float(np.median(t_ground[t > t_ground])) if (t > t_ground).any() else None,
        'ground_seen_beams': int(np.sum(t > t_ground)),
        '_t': t, '_t_ground': t_ground, '_d': d_body,
    }


def fmt_table(rows, cols):
    w = [max(len(str(r[i])) for r in [cols] + rows) for i in range(len(cols))]
    out = ['  '.join(str(c).ljust(w[i]) for i, c in enumerate(cols))]
    out.append('  '.join('-' * w[i] for i in range(len(cols))))
    for r in rows:
        out.append('  '.join(('%.4f' % v if isinstance(v, float) else str(v)).ljust(w[i])
                             for i, v in enumerate(r)))
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mesh', default=os.path.join(PKG, 'meshes/generated/base_link_collision.stl'))
    ap.add_argument('--assets', default=os.path.join(PKG, 'inventory/collision_assets.json'))
    ap.add_argument('--fov-boxes', default=os.path.join(PKG, 'inventory/livox_fov_boxes.json'))
    ap.add_argument('--pattern', default=os.path.join(
        REPO, 'src/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv'))
    ap.add_argument('--pattern-rows', type=int, default=30000,
                    help='用 CSV 的前 N 行当"一帧射线"（插件 samples=30000 ⇒ 与 Gazebo 一帧同源）')
    ap.add_argument('--tilt-axis', default='roll', choices=['roll', 'pitch'])
    ap.add_argument('--livox-raise-m', type=float, default=0.0,
                    help='B 方案：把雷达原点在 body 系里抬高这么多（默认 0 = 上游几何）')
    ap.add_argument('--voxel-m', type=float, default=0.012)
    ap.add_argument('--cone-r-m', type=float, default=0.5)
    ap.add_argument('--cone-margin-deg', type=float, default=3.0)
    ap.add_argument('--inner-clearance-m', type=float, default=0.05,
                    help='雷达原点周围的无碰撞小球半径（锥顶点处方向退化，必须显式挖空）')
    ap.add_argument('--max-range', type=float, default=0.75)
    ap.add_argument('--compare', action='store_true', help='mesh / boxes / carved 三种几何并排')
    ap.add_argument('--emit-carved', action='store_true',
                    help='算 A 方案的 box 并写 inventory/livox_fov_boxes.json')
    ap.add_argument('--explain', action='store_true', help='打印口径说明')
    ap.add_argument('--emit-turret-boxes', action='store_true',
                    help='生成云台 l10/l11 的细碰撞盒 → inventory/turret_boxes.json')
    ap.add_argument('--turret-boxes', default=os.path.join(PKG, 'inventory/turret_boxes.json'))
    ap.add_argument('--turret-voxel-m', type=float, default=0.015)
    ap.add_argument('--mesh-dir', default=os.path.join(PKG, 'meshes'))
    ap.add_argument('--urdf', default='src/rm_nav_bringup/urdf/upstream/robot11.urdf')
    a = ap.parse_args()

    if a.explain:
        print(__doc__)
        return 0

    if a.emit_turret_boxes:
        emit_turret_boxes(a.mesh_dir, os.path.join(REPO, a.urdf),
                          os.path.join(REPO, a.turret_boxes), a.turret_voxel_m, 8)
        return 0

    dp = json.load(open(os.path.join(REPO, a.assets) if not os.path.isabs(a.assets) else a.assets))
    dp_boxes = dp['boxes']['base_link']['boxes']
    dirs = load_pattern(a.pattern, a.pattern_rows)
    print('[fov] 射线 %d 条（%s 前 %d 行）；仰角 %.2f°..%.2f°；倾角轴 %s；雷达抬高 %.3f m'
          % (len(dirs), os.path.basename(a.pattern), a.pattern_rows,
             math.degrees(math.asin(dirs[:, 2].min())), math.degrees(math.asin(dirs[:, 2].max())),
             a.tilt_axis, a.livox_raise_m))

    carved_boxes, carve_info = carve_boxes(dp_boxes, a.tilt_axis, a.cone_r_m,
                                           a.cone_margin_deg, a.voxel_m, a.livox_raise_m,
                                           a.inner_clearance_m)
    print('[fov] A 方案 box 数 = %d（体素 %.0f mm，锥半角 %.1f°，锥半径 %.2f m，共 %d 体素）'
          % (len(carved_boxes), a.voxel_m * 1000, carve_info['half_angle_deg'], a.cone_r_m,
             sum(b['n_voxels'] for b in carved_boxes)))

    models = []
    if not os.path.isfile(a.mesh):
        raise SystemExit('[fov] 缺抽稀碰撞网格 %s（跑 robot11_make_collision_assets.py）' % a.mesh)
    models.append(('mesh(抽稀表面)', {'kind': 'mesh', 'tris': read_stl(a.mesh)}))
    models.append(('boxes(现值)', {'kind': 'boxes', 'boxes': dp_boxes}))
    if a.compare or a.emit_carved:
        models.append(('carved(A)', {'kind': 'boxes', 'boxes': carved_boxes}))
    if os.path.isfile(a.fov_boxes) and not a.compare:
        j = json.load(open(a.fov_boxes))
        models.append(('carved(已提交)', {'kind': 'boxes', 'boxes': j['boxes']}))

    rows = []
    detail = {}
    for name, m in models:
        r = evaluate(m, dirs, a.tilt_axis, a.livox_raise_m, a.max_range)
        detail[name] = {k: v for k, v in r.items() if not k.startswith('_')}
        rows.append([name, r['n_beams'], r['selfhit_lt005'], r['selfhit_lt012'],
                     r['escape_gt075'], r['ground_seen_frac'], r['ground_seen_beams'],
                     r['ground_range_median'] if r['ground_range_median'] else -1.0,
                     r['hit_median'] if r['hit_median'] else -1.0])
    print()
    print(fmt_table(rows, ['几何', '射线数', '自击<0.05', '自击<0.12', '逃逸>0.75m',
                           '看见地面', '地面射线数', '地面中位距离', '命中中位距离']))

    if a.emit_carved:
        out = {
            'tool': 'tools/scripts/regress/robot11_livox_fov.py',
            'generated_at': datetime.datetime.now().isoformat(timespec='seconds'),
            'note': ('A 方案（Phase 3）：`body` 的碰撞 = DP box 减去"雷达视锥"后重新分解成 box。'
                     '这是**仿真侧适配**（上游 CAD 把雷达的支架/凹槽画成了把雷达包住的实体），'
                     '偏离量 = 锥内 cone_r 以内无碰撞 —— 逐条见 docs/robot_models.md §11。'),
            'inputs': {
                'mesh': os.path.relpath(a.mesh, REPO),
                'mesh_sha256': hashlib.sha256(open(a.mesh, 'rb').read()).hexdigest(),
                'dp_boxes': os.path.relpath(os.path.join(REPO, a.assets), REPO),
                'pattern': os.path.relpath(a.pattern, REPO),
                'pattern_rows': a.pattern_rows,
            },
            'params': {'tilt_axis': a.tilt_axis, 'livox_raise_m': a.livox_raise_m,
                       'fov_elev_deg': list(FOV_ELEV_DEG), **carve_info},
            'n_boxes': len(carved_boxes),
            'boxes': carved_boxes,
            'verification': detail,
        }
        p = os.path.join(REPO, a.fov_boxes)
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('\n[fov] 写出 %s（%d box）' % (os.path.relpath(p, REPO), len(carved_boxes)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
