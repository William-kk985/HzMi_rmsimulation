#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11 几何量测（Phase 1 §4 / Phase 2 §3）

从**用户 URDF 的关节表** + **mesh 实测包围盒/顶点**推：
  · 零位正运动学（所有关节角 = 0）→ 每个 link 的 mesh 在 body 系下的**精确**包围盒
  · 地面平面（由四个轮 mesh 的最低点定）→ 雷达离地高度、底盘离地间隙
  · 足印（XY 凸包）→ 内切半径（nav2 `robot_radius` 下界）/ 外接半径（上界）
  · 轮距/轴距（j2..j5 的 origin）
  · 质量/惯量合计
  · mesh 包围盒 ↔ 关节值的**一致性检查**（不一致就报出来）
  · 雷达安装倾角的两个候选（roll / pitch）各自的世界俯仰范围与"最近地面环"半径
    （注意：MID-360 方位 360° ⇒ **幅值相同、轴不同**的两个候选在"世界仰角集合"上完全等价，
      只有**方位相位**能区分 ⇒ 判据是"最近地面环出现在 body 的哪个方位"）

用法：
  python3 tools/scripts/regress/robot11_geometry.py \
      --urdf .tmp_urdf2/robot11.urdf --meshes src/rm_simulation/robot11_description/meshes \
      --json .tmp_robot11/derived/robot11_geometry.json --md .tmp_robot11/derived/robot11_geometry.md
"""

import argparse
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    from scipy.spatial import ConvexHull
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T_of(origin):
    T = np.eye(4)
    if origin is None:
        return T
    T[:3, :3] = rpy_to_R(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
    T[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    return T


def read_stl_vertices(path):
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        head = f.read(84)
        n = struct.unpack('<I', head[80:84])[0]
        if 84 + 50 * n != size:
            raise ValueError('非 binary STL: %s' % path)
        raw = f.read(n * 50)
    rec = np.frombuffer(raw, dtype=np.uint8).reshape(n, 50)
    return rec[:, :48].copy().view(np.float32).reshape(n, 12)[:, 3:12].astype(np.float64).reshape(-1, 3)


def grid_filter(P, cell=0.003):
    """体素抽稀但保住凸包极值：每格留下 max-x/min-x/max-y/min-y/max-r 五类点。"""
    if len(P) == 0:
        return P
    key = np.floor(P[:, :2] / cell).astype(np.int64)
    r = np.hypot(P[:, 0], P[:, 1])
    order = np.lexsort((r, P[:, 1], P[:, 0]))
    k = key[order]
    P = P[order]
    _, start = np.unique(k, axis=0, return_index=True)
    idx = set()
    for col, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
        o = np.lexsort((P[:, col] * sign, k[:, 0], k[:, 1]))
        kk = k[o]
        _, st = np.unique(kk, axis=0, return_index=True)
        idx.update(o[st].tolist())
    ro = np.argsort(-r)
    kr = key[ro]
    _, st = np.unique(kr, axis=0, return_index=True)
    idx.update(ro[st].tolist())
    return P[sorted(idx)]


def hull_metrics(P, cell=0.003):
    """返回 (内切半径, 外接半径, 点数, 凸包面积)。内切 = 原点(0,0) 到最近凸包边的距离。"""
    Q = grid_filter(P, cell)[:, :2]          # ⚠️ 必须只喂 XY：喂 3 列会求成 3D 凸包（体积/平面全错）
    if not _HAS_SCIPY or len(Q) < 3:
        return None
    try:
        h = ConvexHull(Q)
    except Exception:
        return None
    eq = h.equations                            # [A | b]，A·x + b <= 0，A 已归一化
    A, b = eq[:, :2], eq[:, 2]
    nrm = np.linalg.norm(A, axis=1)
    inside = b < 0
    r_in = float(np.min(-b[inside] / nrm[inside])) if inside.any() else None
    r_out = float(np.max(np.hypot(Q[:, 0], Q[:, 1])))
    # 鞋带公式算面积（不依赖 scipy 对 2D hull `.volume` 语义的假设）
    V = Q[h.vertices]
    area = 0.5 * abs(float(np.dot(V[:, 0], np.roll(V[:, 1], -1)) - np.dot(V[:, 1], np.roll(V[:, 0], -1))))
    return {'inscribed_radius_m': round(r_in, 6) if r_in is not None else None,
            'circumscribed_radius_m': round(r_out, 6),
            'signed_area_m2': round(area, 6),
            'points_used': int(len(Q)), 'filter_cell_m': cell,
            'n_hull_vertices': int(len(h.vertices))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default='.tmp_urdf2/robot11.urdf')
    ap.add_argument('--meshes', default='src/rm_simulation/robot11_description/meshes')
    ap.add_argument('--json', default='src/rm_simulation/robot11_description/inventory/geometry.json')
    ap.add_argument('--md', default='src/rm_simulation/robot11_description/inventory/geometry.md')
    a = ap.parse_args()

    root = ET.parse(a.urdf).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    child_of = {j.find('child').get('link'): j for j in joints}

    # ---- 零位 FK ----
    world = {}
    changed = True
    world[root.find('link').get('name') if root.find('link') is not None else None] = np.eye(4)
    # 根 = 不是任何 joint 的 child 的 link
    children = {j.find('child').get('link') for j in joints}
    roots = [n for n in links if n not in children]
    for r in roots:
        world[r] = np.eye(4)
    for _ in range(len(joints) + 2):
        for j in joints:
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            if p in world and c not in world:
                world[c] = world[p] @ T_of(j.find('origin'))

    out = {'urdf': os.path.abspath(a.urdf), 'robot': root.get('name'),
           'root_links': roots, 'links': {}, 'joints': {}}

    # ---- 每个 link 的 mesh 顶点 → body 系 ----
    all_pts, drive_pts = [], []
    DRIVE = {'body'} | {'l%d' % i for i in range(2, 10)}
    for nm, l in links.items():
        mesh = l.find('visual/geometry/mesh') if l.find('visual') is not None else None
        if mesh is None:
            continue
        fn = os.path.basename(mesh.get('filename'))
        p = os.path.join(a.meshes, fn)
        if not os.path.exists(p):
            out['links'][nm] = {'mesh': fn, 'missing': True}
            continue
        V = read_stl_vertices(p)
        T = world.get(nm, np.eye(4))
        W = V @ T[:3, :3].T + T[:3, 3]
        all_pts.append(W)
        if nm in DRIVE:
            drive_pts.append(W)
        ine = l.find('inertial')
        out['links'][nm] = {
            'mesh': fn, 'triangles': int(len(V)),
            'T_body': [[round(x, 9) for x in row] for row in T],
            'bbox_body_m': {'min': W.min(axis=0).round(6).tolist(),
                            'max': W.max(axis=0).round(6).tolist(),
                            'size': (W.max(axis=0) - W.min(axis=0)).round(6).tolist()},
            'mesh_bbox_origin_m': {'min': V.min(axis=0).round(6).tolist(),
                                   'max': V.max(axis=0).round(6).tolist()},
            'mass_kg': float(ine.find('mass').get('value')) if ine is not None else None,
        }

    ALL = np.concatenate(all_pts, axis=0)
    DRV = np.concatenate(drive_pts, axis=0)

    # ---- 地面：四个轮 mesh 的最低点 ----
    wheel_names = [n for n in ('l6', 'l7', 'l8', 'l9') if n in out['links']]
    wz = {n: out['links'][n]['bbox_body_m']['min'][2] for n in wheel_names}
    z_ground = min(wz.values())
    out['ground'] = {
        'wheel_min_z_body_m': {k: round(v, 6) for k, v in wz.items()},
        'z_ground_in_body_m': round(z_ground, 6),
        'how': '四个轮 link 的 mesh 顶点在 body 系下的最小 z（轮 mesh 原点在轮轴上，'
               'j2..j5 的 origin z=0.05735 = 轮心高度）⇒ 地面 = body z=%.6f' % z_ground,
        'wheel_radius_from_mesh_m': round(max(out['links'][n]['mesh_bbox_origin_m']['max'][1] for n in wheel_names), 6),
    }

    # ---- 雷达 ----
    lv = 'livox_frame'
    Tv = world[lv]
    out['lidar'] = {
        'link': lv,
        'origin_in_body_m': [round(x, 6) for x in Tv[:3, 3]],
        'origin_in_body_exact': [float(x) for x in Tv[:3, 3]],
        'z_above_ground_m': round(float(Tv[2, 3] - z_ground), 6),
        'mesh_bbox_body_m': out['links'][lv]['bbox_body_m'],
        'tilt_rpy_file': [float(v) for v in child_of[lv].find('origin').get('rpy').split()],
    }
    # 候选：roll vs pitch（-0.523598775598293 rad = -30°）
    ang = -0.523598775598293
    fov_lo, fov_hi = -7.0, 52.0
    cand = {}
    for nm, rpy in (('roll(Rx)', (ang, 0, 0)), ('pitch(Ry)', (0, ang, 0))):
        R = rpy_to_R(*rpy)
        # 世界仰角范围：sin(el) = d_z, 对 e∈[fov_lo,fov_hi]、ψ∈[0,2π) 取极值
        el = np.radians(np.linspace(fov_lo, fov_hi, 400))
        psi = np.linspace(0, 2 * np.pi, 721)
        E, P = np.meshgrid(el, psi, indexing='ij')
        d = np.stack([np.cos(E) * np.cos(P), np.cos(E) * np.sin(P), np.sin(E)], axis=-1)
        dz = d @ R[2, :]
        lo, hi = float(np.degrees(np.arcsin(dz.min()))), float(np.degrees(np.arcsin(dz.max())))
        # 最"朝下"的方位（body 系 atan2(y,x)）
        i, j = np.unravel_index(np.argmin(dz), dz.shape)
        dw = R @ np.array([np.cos(el[i]) * np.cos(psi[j]), np.cos(el[i]) * np.sin(psi[j]), np.sin(el[i])])
        az = math.degrees(math.atan2(dw[1], dw[0]))
        h = float(Tv[2, 3] - z_ground)
        cand[nm] = {
            'rpy': rpy,
            'world_elev_range_deg': [round(lo, 3), round(hi, 3)],
            'most_downward_body_azimuth_deg': round(az, 2),
            'nearest_ground_ring_radius_m': round(h / math.tan(math.radians(-lo)), 4) if lo < 0 else None,
            'blind_cone_half_angle_deg': round(90 + lo, 3),
        }
    out['lidar']['tilt_candidates'] = cand
    out['lidar']['note'] = ('MID-360 方位 360° ⇒ roll/pitch 两个候选的**世界仰角集合完全相同**'
                            '（只差一个绕 z 的旋转）⇒ 仰角/地面环半径**不能**区分它们；'
                            '唯一判据是"最朝下的方位"在 body 系里指向哪里（-y / -x）。')

    # ---- 足印 ----
    # 极坐标轮廓：每 15° 一个扇区，取该扇区内点的最大半径（供 nav2 robot_radius/inflation 参考）
    def polar(P, nb=24):
        r = np.hypot(P[:, 0], P[:, 1])
        az = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360.0
        b = np.minimum((az / (360.0 / nb)).astype(int), nb - 1)
        prof = []
        for i in range(nb):
            m = b == i
            prof.append({'az_center_deg': round(i * 360.0 / nb + 180.0 / nb, 1),
                         'max_r_m': round(float(r[m].max()), 4) if m.any() else None})
        return prof

    out['footprint_polar_drive'] = polar(DRV, 24)
    out['footprint'] = {
        'drive_group(body+l2..l9)': hull_metrics(DRV),
        'all_links': hull_metrics(ALL),
    }
    ext_all = {'min': ALL.min(axis=0).round(6).tolist(), 'max': ALL.max(axis=0).round(6).tolist()}
    ext_all['size'] = [round(ext_all['max'][i] - ext_all['min'][i], 6) for i in range(3)]
    out['envelope_all_m'] = ext_all
    ext_drv = {'min': DRV.min(axis=0).round(6).tolist(), 'max': DRV.max(axis=0).round(6).tolist()}
    ext_drv['size'] = [round(ext_drv['max'][i] - ext_drv['min'][i], 6) for i in range(3)]
    out['envelope_drive_m'] = ext_drv
    out['height_above_ground_m'] = round(ext_all['max'][2] - z_ground, 6)
    out['drive_height_above_ground_m'] = round(ext_drv['max'][2] - z_ground, 6)
    out['ground_clearance_m'] = round(ext_drv['min'][2] - z_ground, 6)

    # ---- 关节表 ----
    for j in joints:
        nm = j.get('name')
        o = j.find('origin')
        out['joints'][nm] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
            'xyz': [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0],
            'rpy': [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0],
            'axis': [float(v) for v in j.find('axis').get('xyz').split()] if j.find('axis') is not None else None,
            'child_origin_in_body_m': [round(x, 6) for x in world[j.find('child').get('link')][:3, 3]],
        }

    # ---- 轴距/轮距 ----
    steer = [out['joints']['j%d' % i]['xyz'] for i in (2, 3, 4, 5)]
    xs = sorted({round(p[0], 6) for p in steer})
    ys = sorted({round(p[1], 6) for p in steer})
    out['wheel_layout'] = {
        'steer_joint_origins': steer,
        'track_y_m': round(ys[-1] - ys[0], 6),
        'wheelbase_x_m': round(xs[-1] - xs[0], 6),
        'half_track_m': round((ys[-1] - ys[0]) / 2, 6),
        'half_wheelbase_m': round((xs[-1] - xs[0]) / 2, 6),
        'steer_joint_z_m': steer[0][2],
    }

    # ---- 质量 ----
    tot = sum(v['mass_kg'] for v in out['links'].values() if v.get('mass_kg'))
    out['mass'] = {'total_kg': round(tot, 6),
                   'by_link': {k: v.get('mass_kg') for k, v in out['links'].items()}}

    # ---- 一致性检查 ----
    checks = []
    bd = out['links']['body']['bbox_body_m']
    checks.append(('base_link mesh 顶面 z ↔ j10(云台) origin z',
                   bd['max'][2], out['joints']['j10']['xyz'][2], abs(bd['max'][2] - out['joints']['j10']['xyz'][2])))
    checks.append(('轮 mesh 半径 ↔ j2..j5 origin z（地面应在 body z≈0）',
                   0.0, round(wz[wheel_names[0]], 6), abs(wz[wheel_names[0]])))
    checks.append(('l12 mesh 底面 z ↔ livox_frame 原点（mesh 原点=安装面）',
                   out['links'][lv]['mesh_bbox_origin_m']['min'][2], 0.0,
                   abs(out['links'][lv]['mesh_bbox_origin_m']['min'][2])))
    checks.append(('l12 mesh 高度 ↔ MID-360 高度（0.0601 m）',
                   out['links'][lv]['mesh_bbox_origin_m']['max'][2], 0.0601,
                   abs(out['links'][lv]['mesh_bbox_origin_m']['max'][2] - 0.0601)))
    out['consistency_checks'] = [{'what': w, 'mesh_side': m, 'joint_side': j, 'abs_diff': round(d, 6)}
                                 for w, m, j, d in checks]

    os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
    with open(a.json, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)

    # ---- Markdown ----
    L = ['# robot11 几何量测（自动生成）\n',
         '来源：`%s`（关节表）+ `%s`（mesh 顶点，自带 binary STL 解析）\n' % (a.urdf, a.meshes),
         '零位（所有关节角 = 0）正运动学；所有数值单位 m。\n',
         '## 1. 逐 link（mesh 顶点在 body 系下的精确包围盒）\n',
         '| link | mesh | 三角面 | 质量 kg | min xyz | max xyz | 尺寸 |',
         '|---|---|---|---|---|---|---|']
    for nm, v in out['links'].items():
        if v.get('missing'):
            L.append('| `%s` | %s | — | — | — | — | **缺失** |' % (nm, v['mesh']))
            continue
        b = v['bbox_body_m']
        L.append('| `%s` | `%s` | %d | %s | %s | %s | %s × %s × %s |' % (
            nm, v['mesh'], v['triangles'], v.get('mass_kg'),
            b['min'], b['max'], *['%.4f' % x for x in b['size']]))
    L += ['\n## 2. 地面 / 雷达 / 高度\n',
          '| 量 | 值 | 怎么来的 |', '|---|---|---|',
          '| **地面平面** | body 系 `z = %.6f` | 四个轮 mesh 在 body 系下的最低 z（轮 mesh 原点在轮轴上）|' % z_ground,
          '| 雷达原点（body 系） | `%s` | `body_to_livox` 的 xyz |' % out['lidar']['origin_in_body_m'],
          '| **雷达离地** | **%.4f m** | 雷达原点 z − 地面 z |' % out['lidar']['z_above_ground_m'],
          '| 整车高度（含云台/发射机构） | %.4f m | 所有 mesh 顶点的 max z − 地面 |' % out['height_above_ground_m'],
          '| 底盘组高度 | %.4f m | body+l2..l9 |' % out['drive_height_above_ground_m'],
          '| 离地间隙 | %.4f m | 底盘组最低点 − 地面 |' % out['ground_clearance_m'],
          '\n## 3. 足印（XY 凸包，3 mm 体素抽稀后求）\n',
          '| 组 | 内切半径 (m) | 外接半径 (m) | 凸包面积 (m²) |', '|---|---|---|---|']
    for k, v in out['footprint'].items():
        if v:
            L.append('| %s | **%s** | **%s** | %s |' % (k, v['inscribed_radius_m'], v['circumscribed_radius_m'], v['signed_area_m2']))
    L += ['\n### 3.1 极坐标轮廓（底盘组，每 15° 的最大半径）\n',
          '| 方位扇区中心 (°) | 最大半径 (m) |', '|---|---|']
    for p_ in out['footprint_polar_drive']:
        L.append('| %s | %s |' % (p_['az_center_deg'], p_['max_r_m']))
    L += ['\n## 4. 轮距 / 轴距 / 质量\n',
          '| 量 | 值 |', '|---|---|',
          '| 轴距（x） | %.5f m |' % out['wheel_layout']['wheelbase_x_m'],
          '| 轮距（y） | %.5f m |' % out['wheel_layout']['track_y_m'],
          '| 转向关节 z | %.5f m |' % out['wheel_layout']['steer_joint_z_m'],
          '| 质量合计 | %.4f kg |' % out['mass']['total_kg'],
          '\n## 5. 雷达倾角候选（roll vs pitch）\n',
          '| 候选 | rpy | 世界仰角范围 | 最朝下的 body 方位 | 最近地面环半径 |', '|---|---|---|---|---|']
    for k, v in cand.items():
        L.append('| %s | %s | %s°…%s° | **%s°** | %s m |' % (
            k, v['rpy'], v['world_elev_range_deg'][0], v['world_elev_range_deg'][1],
            v['most_downward_body_azimuth_deg'], v['nearest_ground_ring_radius_m']))
    L += ['', '> %s' % out['lidar']['note'],
          '\n## 6. 一致性检查（mesh 实测 ↔ 关节值）\n',
          '| 检查 | mesh 侧 | 关节侧 | 差 |', '|---|---|---|---|']
    for c in out['consistency_checks']:
        L.append('| %s | %.6f | %.6f | %.6f |' % (c['what'], c['mesh_side'], c['joint_side'], c['abs_diff']))
    with open(a.md, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')
    sys.stderr.write('[geom] -> %s / %s\n' % (a.json, a.md))
    print(json.dumps({k: out[k] for k in ('ground', 'wheel_layout', 'mass', 'height_above_ground_m',
                                          'ground_clearance_m', 'footprint')}, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
