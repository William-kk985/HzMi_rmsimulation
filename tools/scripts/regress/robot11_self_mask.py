#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11_self_mask.py —— `robot:=robot11` 的「自击掩膜」（self-hit mask）生成 + 离线验证。

## 这个掩膜是什么、为什么必须有

`robot:=robot11` 的雷达（MID-360）装在**底盘凹槽里**，360° 视场里最近的部件是**云台**
（`l10`/`l11`，最近表面离雷达 0.116~0.139 m）。Gazebo 实测：**28.6% 的云点落在 r<0.12 m**，
100% 归因于云台（Phase 3，docs/robot_models.md §11.2）。

这些近场自击点**不是障碍**，但它们落在 `rm_ground_traversability` 的前瞻走廊粗格里
（走廊从 d=0 开始），会把"格内最高点 − 局部地面"抬到 **0.16 m** ⇒ 限速器判成台阶
⇒ 车**停着不动**时 `[slope_speed] limit=0.60 m/s`（速度表地板）被钉死（Phase 4 实测）。

## 掩膜的几何定义（**不是**拍脑袋的半径圆）

> 掩膜 = **机器人自己那 12 个 link 的 `<collision>` 几何**在 `livox_frame` 下的并集。

为什么这条定义是**充分且必要**的：

* **充分**：Gazebo 的 ray sensor 只与 `<collision>` 求交（ODE）。任何"自击"点必然落在
  某个 collision 体的**表面**上 ⇒ 必然落在该体的 AABB 内。`livox_frame` 相对 `base_link`
  的 rpy = **0 0 0**（Phase 3：点云在源头重力对齐），所以 AABB 变换时不必担心旋转误差。
* **必要（不误伤）**：真障碍/地面点不可能在机器人自己的 collision 体内部（那已经是碰撞态）。
  唯一要小心的是**地面本身**：轮子/底盘的 collision 下沿**正好在地面平面**上
  （`base_link` 的 DP box 底 = z −0.047944，轮子 mesh 最低点 = 地面 z −0.102499）
  ⇒ 掩膜把 `z_min` **抬到地面以上 `--ground-clearance-m`**（默认 0.03 m）
  ⇒ **地面点（z ≈ −sensor_height）永远不可能被掩掉**，而被掩掉的只有"离地 ≥3 cm 的自身点"
  （一个离地 3 cm 的自身点在 0.20 m 粗格里最多贡献 3 cm 抬升 < `step_deadband` 0.06 m）。

## 运行期怎么用（槽位作用域，默认模型逐字节不变）

* `src/rm_nav_bringup/config/traversability_self_mask.yaml`            → `self_mask_enable: false`（默认槽位）
* `src/rm_nav_bringup/config/traversability_self_mask_robot11.yaml`    → `self_mask_enable: true` + 本脚本生成的点表
  （launch 按 `robot` 槽位选文件，机制与 linefit 的 `segmentation_sim_<slot>.yaml` **完全同款**）

## 用法

    python3 tools/scripts/regress/robot11_self_mask.py --emit        # 重写两份 YAML（robot11 那份）
    python3 tools/scripts/regress/robot11_self_mask.py --verify      # 离线验证（射线 vs 网格/盒子）
    python3 tools/scripts/regress/robot11_self_mask.py --dump-fk     # 打印零位 FK（与运行期 TF 对照）
    python3 tools/scripts/regress/robot11_self_mask.py --check       # 断言 YAML == 由模型重算的结果
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)

from robot11_livox_fov import read_stl, rot_rpy  # noqa: E402  （同目录、可复用）

XACRO = os.path.join(WS, 'src', 'rm_nav_bringup', 'urdf', 'sentry_robot_robot11_sim.xacro')
MESH_DIR = os.path.join(WS, 'src', 'rm_simulation', 'robot11_description', 'meshes')
GEN_DIR = os.path.join(MESH_DIR, 'generated')
PATTERN = os.path.join(WS, 'src', 'rm_simulation', 'livox_laser_simulation_RO2',
                       'scan_mode', 'mid360.csv')
OUT_ROBOT11 = os.path.join(WS, 'src', 'rm_nav_bringup', 'config',
                           'traversability_self_mask_robot11.yaml')
OUT_DEFAULT = os.path.join(WS, 'src', 'rm_nav_bringup', 'config',
                           'traversability_self_mask.yaml')

#: `livox_frame` 在 base_link 下的原点（xacro `body_to_livox`；raise=0）＝掩膜的坐标原点
LIVOX_XYZ = np.array([0.000561701465058485, 0.130915824456595, 0.15702816968305])
#: 地面平面在 base_link 下的 z（四轮 mesh 最低点实测，见 docs/robot_models.md §9.4）
GROUND_Z = -0.102499
#: 30° 安装倾角（roll 形式，= launch 默认 livox_tilt_axis:=roll）
TILT_RPY = (-0.523598775598293, 0.0, 0.0)
#: 雷达离地高度（实测；= 地面在 livox 系下的 z 的相反数）
SENSOR_H = 0.2595
#: 最低那条射线的**世界**下俯角（= 安装倾角 30° + FOV 下沿 7.22°；绕 roll 转 ⇒ 两角相加）
MIN_ELEV_DEG = 30.0 + 7.22
#: **几何盲半径**（m）：低于它**不可能**有地面回波（= SENSOR_H / tan(最低俯角)）
BLIND_R = SENSOR_H / math.tan(math.radians(MIN_ELEV_DEG))
#: 近场死区半径相对盲半径留的余量（m）
NEAR_FIELD_MARGIN_M = 0.10


def near_field_radius():
    return round(BLIND_R - NEAR_FIELD_MARGIN_M, 4)


# --------------------------------------------------------------------------
# 模型 → collision AABB（link 系）→ livox_frame
# --------------------------------------------------------------------------
def _f(x, d=0.0):
    return float(x) if x is not None else d


def run_xacro():
    cmd = ['xacro', XACRO, 'xyz:=0 0 0', 'rpy:=0 0 0',
           'livox_tilt_rpy:=%f %f %f' % TILT_RPY, 'livox_raise_m:=0.0']
    out = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return out.stdout


def aabb_of_geometry(geo, link_dir):
    """`<geometry>` → link 系下的 AABB（lo, hi）。mesh 用 STL 的 bbox（本模型碰撞全是 box/cylinder）。"""
    kind = list(geo)[0].tag
    if kind == 'box':
        s = np.array([float(v) for v in geo.find('box').get('size').split()])
        return -s / 2, s / 2
    if kind == 'cylinder':
        c = geo.find('cylinder')
        r, l = float(c.get('radius')), float(c.get('length'))
        return np.array([-r, -r, -l / 2]), np.array([r, r, l / 2])
    if kind == 'sphere':
        r = float(geo.find('sphere').get('radius'))
        return np.array([-r, -r, -r]), np.array([r, r, r])
    if kind == 'mesh':
        fn = geo.find('mesh').get('filename')
        rel = fn.split('package://robot11/', 1)[-1]
        path = os.path.join(os.path.dirname(MESH_DIR), rel)
        if not os.path.isfile(path):                       # 点云用抽稀件不在时退回原 mesh
            path = os.path.join(MESH_DIR, os.path.basename(rel))
        V = read_stl(path).reshape(-1, 3)
        return V.min(axis=0), V.max(axis=0)
    raise ValueError('未知 geometry: %s' % kind)


def collision_aabbs(urdf_xml):
    """返回 [(link, lo_link, hi_link, n_shapes)]（零位 FK；本模型只有 box/cylinder）。"""
    root = ET.fromstring(urdf_xml)
    # ---- FK（零位；j2..j11 都是 continuous，零位 = 单位旋转）----
    joints = []
    for j in root.findall('joint'):
        o = j.find('origin')
        xyz = np.array([_f(v) for v in (o.get('xyz') or '0 0 0').split()]) if o is not None \
            else np.zeros(3)
        rpy = [_f(v) for v in ((o.get('rpy') or '0 0 0').split() if o is not None else [])] \
            if o is not None else [0.0, 0.0, 0.0]
        joints.append({'name': j.get('name'), 'type': j.get('type'),
                       'parent': j.find('parent').get('link'),
                       'child': j.find('child').get('link'),
                       'xyz': xyz, 'R': rot_rpy(*rpy)})
    pose = {'base_link': (np.zeros(3), np.eye(3))}
    for _ in range(len(joints) + 2):
        for j in joints:
            if j['parent'] in pose and j['child'] not in pose:
                tp, Rp = pose[j['parent']]
                pose[j['child']] = (tp + Rp @ j['xyz'], Rp @ j['R'])
    out = []
    for link in root.findall('link'):
        name = link.get('name')
        if name not in pose:
            continue
        cols = link.findall('collision')
        if not cols:
            continue
        t, R = pose[name]
        for c in cols:
            o = c.find('origin')
            xyz = np.array([_f(v) for v in (o.get('xyz') or '0 0 0').split()]) if o is not None \
                else np.zeros(3)
            rpy = [_f(v) for v in ((o.get('rpy') or '0 0 0').split())] if o is not None \
                else [0.0, 0.0, 0.0]
            lo, hi = aabb_of_geometry(c.find('geometry'), None)
            # collision 自身系下的 8 个角 → link 系 → base_link 系
            corners = np.array([[lo[0] if i & 1 else hi[0],
                                 lo[1] if i & 2 else hi[1],
                                 lo[2] if i & 4 else hi[2]] for i in range(8)])
            corners = (rot_rpy(*rpy) @ corners.T).T + xyz
            corners = (R @ corners.T).T + t
            out.append((name, corners.min(axis=0), corners.max(axis=0)))
    return pose, out


def to_livox(lo, hi):
    """base_link 系 AABB → livox_frame（纯平移，rpy=0 ⇒ 无旋转误差）。"""
    return lo - LIVOX_XYZ, hi - LIVOX_XYZ


def build_mask(tolerance_m=0.005, ground_clearance_m=0.03, merge=True):
    xml = run_xacro()
    pose, aabbs = collision_aabbs(xml)
    boxes = []
    for name, lo, hi in aabbs:
        l2, h2 = to_livox(lo - tolerance_m, hi + tolerance_m)
        boxes.append({'link': name,
                      'lo': [round(float(v), 5) for v in l2],
                      'hi': [round(float(v), 5) for v in h2]})
    if merge:
        boxes = drop_contained(boxes)
    # 地面保护：z_min 抬到地面以上（**在任何合并之后**做，保证单调）
    z_floor = GROUND_Z - LIVOX_XYZ[2] + ground_clearance_m
    kept = []
    for b in boxes:
        if b['hi'][2] <= z_floor:
            continue                                  # 整块都在地面保护带以下 ⇒ 不需要掩
        b = dict(b)
        b['lo'] = [b['lo'][0], b['lo'][1], round(max(b['lo'][2], z_floor), 5)]
        kept.append(b)
    return kept, z_floor, pose


def drop_contained(boxes):
    """去掉被别的盒子**完全包含**的盒子（掩膜是并集 ⇒ 不改变语义，只是短一点）。"""
    keep = []
    for i, b in enumerate(boxes):
        l1, h1 = np.array(b['lo']), np.array(b['hi'])
        contained = False
        for j, c in enumerate(boxes):
            if i == j:
                continue
            l2, h2 = np.array(c['lo']), np.array(c['hi'])
            if np.all(l2 <= l1 + 1e-9) and np.all(h2 >= h1 - 1e-9):
                if b['link'] == c['link'] or np.any(l2 < l1 - 1e-9) or np.any(h2 > h1 + 1e-9):
                    contained = True
                    break
        if not contained:
            keep.append(b)
    return keep


# --------------------------------------------------------------------------
# 离线验证：射线 vs 真几何（网格 / 碰撞盒），看掩膜覆盖率
# --------------------------------------------------------------------------
def pattern_dirs():
    """MID-360 的 30000 条采样方向（`livox_frame`，**已含 30° 安装倾角**）。

    口径与插件一致（`livox_points_plugin.cpp`）：传感器系方向由 CSV 的 Azimuth/Zenith
    构造（与本目录 robot11_livox_fov.py 同款），再乘 `offset.Rot()`（= `<tilt_rpy>`）
    转到父 link 系 ⇒ 射线真的斜 30°，但点表达在重力对齐的 `livox_frame` 里。
    """
    from robot11_livox_fov import load_pattern
    v = load_pattern(PATTERN, 30000)
    R = rot_rpy(*TILT_RPY)
    return v @ R.T


def in_mask(pts, boxes, radius=None, z_min=None):
    """pts（livox_frame）是否落在掩膜并集里。返回 bool 数组。

    `radius`/`z_min` = (b) 近场死区；`boxes` = (a) 自身 collision 包络。
    """
    m = np.zeros(len(pts), dtype=bool)
    if radius is not None and radius > 0 and len(pts):
        zok = np.ones(len(pts), dtype=bool) if z_min is None else (pts[:, 2] >= z_min)
        m |= zok & (np.hypot(pts[:, 0], pts[:, 1]) <= radius)
    for b in boxes:
        lo, hi = np.array(b['lo']), np.array(b['hi'])
        m |= np.all((pts >= lo) & (pts <= hi), axis=1)
    return m


def verify(tol=0.005, clearance=0.03, n_ray=None, json_out=None):
    boxes, z_floor, pose = build_mask(tol, clearance)
    dirs = pattern_dirs()
    if n_ray:
        dirs = dirs[:n_ray]
    origins = np.zeros_like(dirs)
    r_near = near_field_radius()
    res = {'tolerance_m': tol, 'ground_clearance_m': clearance,
           'z_floor_m': round(float(z_floor), 5), 'n_boxes': len(boxes),
           'near_field_radius_m': r_near, 'blind_radius_m': round(BLIND_R, 4),
           'sensor_height_m': SENSOR_H, 'min_elev_deg': MIN_ELEV_DEG,
           'n_rays': int(len(dirs))}

    # ---- ① 每条 collision 几何被掩膜覆盖了多少（**这就是 Gazebo 射线唯一能打到的东西**）----
    #   Gazebo 的 ray sensor 走 ODE、只与 `<collision>` 求交 ⇒ "自击点必在 collision 体表面上"
    #   ⇒ 掩膜（= collision 的 AABB 并集）**按构造**覆盖它。这里把"按构造"变成可复核的数字：
    #   逐个 collision 体报"被 z 下限裁掉多少"（这是**故意的**地面保护，不是漏掩）。
    xml = run_xacro()
    _, aabbs = collision_aabbs(xml)
    clipped, worst = 0, 0.0
    by_link = {}
    for name, lo, hi in aabbs:
        l2, h2 = to_livox(lo - tol, hi + tol)
        d = max(0.0, z_floor - l2[2])
        by_link[name] = by_link.get(name, 0) + 1
        if d > 0:
            clipped += 1
            worst = max(worst, d)
    res['collision_cover'] = {
        'n_shapes': len(aabbs), 'by_link': by_link,
        'z_clipped_shapes': clipped, 'z_clip_max_m': round(float(worst), 5),
        'note': ('掩膜 = collision 的 AABB 并集（按构造 ⊇ 每个 collision 体）。'
                 'z_clipped = 被"地面保护"下限裁掉的形状数（裁掉的部分只在地面以上 %.3f m 以内）'
                 % clearance)}

    # ---- ② 地面环：地面点会不会被误掩（**必须 0**）----
    gz = GROUND_Z - LIVOX_XYZ[2]                   # 地面在 livox_frame 的 z
    down = np.where(dirs[:, 2] < -1e-6)[0]
    t_ground = gz / dirs[down, 2]
    ok = t_ground > 0.15
    gp = dirs[down][ok] * t_ground[ok, None]
    gmask = in_mask(gp, boxes, r_near, z_floor)
    res['ground_ring'] = {'n_points': int(len(gp)),
                          'r_min': round(float(np.linalg.norm(gp[:, :2], axis=1).min()), 4),
                          'masked': int(gmask.sum()),
                          'masked_frac': round(float(gmask.mean()), 6) if len(gp) else None,
                          'note': '地面点（射线打到 z=地面平面）被掩膜误掉的个数；必须 = 0'}

    # ---- ③（参考信息）射线 vs **真 mesh 表面** ----
    #   ⚠️ 这一项**不是**判据，故意留作"诚实说明"：真 mesh 上还有一批 <0.75 m 的命中**不在掩膜里**，
    #   因为 Phase 3 的 A 方案**故意把雷达视锥里的 body 材质从碰撞里挖掉**了
    #   （否则实心包络把雷达包住 ⇒ 75.5% 自击、看不见地面，见 §11.2）。
    #   Gazebo 的射线打不到这些材质（ODE 只认 collision）⇒ 它们不会变成自击点。
    #   证据：这些"漏掩"的点落在 **pre-carve 的 DP 包络**（collision_assets.json 的 4 个 box）里。
    mesh_hits, per_link = [], {}
    from robot11_livox_fov import ray_mesh
    for link in ('base_link', 'l2', 'l3', 'l4', 'l5', 'l6', 'l7', 'l8', 'l9',
                 'l10', 'l11'):
        path = os.path.join(GEN_DIR, '%s_collision.stl' % link)
        t, R = pose.get(link, (None, None))
        if t is None or not os.path.isfile(path):
            continue
        tris_l = (R @ read_stl(path).reshape(-1, 3).T).T.reshape(-1, 3, 3) + (t - LIVOX_XYZ)
        tt = ray_mesh(origins, dirs, tris_l, chunk=64, max_t=1.5)
        sel = np.isfinite(tt) & (tt < 0.75)
        if sel.any():
            pts = dirs[sel] * tt[sel, None]
            mesh_hits.append(pts)
            per_link[link] = int(sel.sum())
    allpts = np.vstack(mesh_hits) if mesh_hits else np.zeros((0, 3))
    inside = in_mask(allpts, boxes, r_near, z_floor)
    info = {'n_hits_lt075': int(len(allpts)),
            'masked': int(inside.sum()),
            'masked_frac': round(float(inside.mean()), 6) if len(allpts) else None,
            'per_link': per_link,
            'note': '参考信息，不是判据：真 mesh 上被 A 方案**故意挖掉**（碰撞里没有）的材质不算自击'}
    if len(allpts) and (~inside).any():
        miss = allpts[~inside]
        dp = dp_envelope_boxes()
        info['miss_n'] = int((~inside).sum())
        info['miss_in_precarve_dp_envelope_frac'] = round(float(in_mask(miss, dp).mean()), 6)
        info['miss_max_r'] = round(float(np.linalg.norm(miss, axis=1).max()), 4)
        info['miss_p50_r'] = round(float(np.median(np.linalg.norm(miss, axis=1))), 4)
    res['mesh_reference'] = info
    if json_out:
        with open(json_out, 'w') as f:
            json.dump(res, f, indent=1, ensure_ascii=False)
    return res


def dp_envelope_boxes():
    """Phase 2 的 **pre-carve** body 包络（4 个 DP box，body 系）→ livox_frame 的 AABB 列表。

    用来证明"真 mesh 上未被掩膜覆盖的命中点"落在这 4 个 box 里 = A 方案故意挖掉的材质。
    """
    p = os.path.join(WS, 'src', 'rm_simulation', 'robot11_description', 'inventory',
                     'collision_assets.json')
    d = json.load(open(p))
    out = []
    for b in d['boxes']['base_link']['boxes']:
        c = np.array(b['center'])
        s = np.array(b['size'])
        lo, hi = to_livox(c - s / 2, c + s / 2)
        out.append({'lo': list(lo), 'hi': list(hi)})
    return out


# --------------------------------------------------------------------------
# 写 YAML
# --------------------------------------------------------------------------
def yaml_boxes(boxes):
    vals = []
    for b in boxes:
        vals += ['%.5f' % b['lo'][0], '%.5f' % b['lo'][1], '%.5f' % b['lo'][2],
                 '%.5f' % b['hi'][0], '%.5f' % b['hi'][1], '%.5f' % b['hi'][2]]
    out, line = [], '    self_mask_boxes: ['
    for v in vals:
        if len(line) + len(v) + 2 > 98:
            out.append(line)
            line = '      '
        line += v + ', '
    line = line.rstrip().rstrip(',') + ']'
    out.append(line)
    return '\n'.join(out)


def emit(tol=0.005, clearance=0.03):
    boxes, z_floor, pose = build_mask(tol, clearance)
    n_links = len({b['link'] for b in boxes})
    r_near = near_field_radius()
    head = '''# =============================================================================
# traversability_self_mask_robot11.yaml —— `robot:=robot11` 的**自击掩膜**（2026-10-07 Phase 4 新增）
#
# 是什么：`rm_ground_traversability`（两个地面分割节点共用的那层）在**建格之前**把掩膜里的点剔掉
#   ⇒ 这些点既不参与"局部地面高度"、也不参与"格内最高点 − 地面"（台阶残差），更不进前瞻走廊。
#   **只影响判据/限速**：点云与 `/segmentation/*` 的标签一个都不变（近场回波是物理真实的）。
#
# 为什么（Phase 4 基线实测）：本模型雷达装在底盘凹槽里，360° 视场里最近的是**云台** l10/l11。
#   Gazebo 实测 **26.9~28.1%% 的点在 r ≤ 0.09 m**，它们落在前瞻走廊粗格 d=0 里
#   ⇒ 台阶残差被抬到 **0.076~0.095 m**（> step_deadband 0.06）⇒ 限速器判成台阶
#   ⇒ **车停着也被压到速度表地板 0.60 m/s**（基线 129 条 [slope_speed] 里 9 条在地板、
#      12 条的特征距离 = 0.00 m，见 docs/robot_models.md §12）。
#
# 掩膜 = 两部分并集：
#   (a) 自身 collision 包络：本模型 12 个 link 的 `<collision>` 在 `livox_frame` 下的 AABB
#       （%d 个盒；m 为单位、原点 = 雷达原点；`livox_frame` 相对 base_link 的 rpy = 0
#        ⇒ 变换是纯平移、无旋转误差）。依据：Gazebo 的 ray sensor 只与 `<collision>` 求交。
#   (b) 近场死区：`r_xy ≤ %.4f m 且 z ≥ %.4f m`。为什么 (a) 不够（诚实更正）：
#       本槽位近场那 27%% 的点**不在任何 collision 几何上**（逐点验证：113 个盒只覆盖 3 个）。
#       它们是**射线插件把点重建成 `range·axis`** 的系统内移：射线实际从 `minDist·axis`
#       （= 0.1 m）出发，点却按 `range·axis` 发布 ⇒ 每个点朝传感器方向内移 0.1 m。
#       独立证据（与掩膜无关）：地面点高度随距离单调变化（r 0.25–0.35 → z −0.208；
#       r 2–4 → z −0.253；几何真值 −0.2595），正是"沿射线内移 0.1 m"应有的样子。
#       那个插件属共享代码（改它会改变**所有模型**的点云）⇒ 本主题只在判据层剔掉这团近场。
#       `radius` 的上界是**几何硬约束**：下俯 30°、离地 0.2595 m ⇒ 最低射线的地面交点在
#       **%.4f m**，即 r 小于它时**不可能有地面回波**；取 %.4f = 盲半径 − 0.10 m 留余量。
#
# 地面保护（两处）：每个盒子的 z 下限被抬到"地面以上 %.2f m"（= livox 系 z ≥ %.4f）；
#   近场死区也带同一个 z 下限 ⇒ **地面点（z ≈ −0.2595）永远不可能被掩掉**。
#   离线验证（--verify）：地面环被掩 **0 点**；在线验证：3 帧实测里被掩的地面带点也是 0。
#
# 生成/验证：`python3 tools/scripts/regress/robot11_self_mask.py --emit|--verify|--check`
#   · --verify 用 MID-360 的 30000 条射线打真几何，报"地面环被掩点数"（必须 0）等；
#   · --check 断言本文件 == 由模型重算的结果（模型改了而掩膜没跟着改 ⇒ 报错）。
#
# 作用域：**只有 robot:=robot11 会读到本文件**（launch 按 robot 槽位选文件，机制同 linefit
#   `segmentation_sim_<slot>.yaml`）。默认槽位读的是 `traversability_self_mask.yaml`
#   （`self_mask_enable: false`）⇒ 默认模型逐字节行为不变。
# 回退：把 `self_mask_enable` 改成 false（或把 launch 的这一路撤掉）⇒ 完全回到旧行为。
# 关联文档：docs/robot_models.md §12。
# =============================================================================
ground_segmentation:
  ros__parameters:
    # 掩膜总开关。false ⇒ `buildGrid()` 完全不走这段代码（默认槽位就是这条路径）。
    self_mask_enable: true
    # (b) 近场死区：半径（m，livox 系 XY）+ z 下限（m，livox 系）。0 / 不设 = 关掉这一部分。
    self_mask_radius_m: %.4f
    self_mask_z_min_m: %.5f
    # (a) 自身 collision 包络。每个盒子 6 个数：xmin,ymin,zmin,xmax,ymax,zmax（**livox_frame**，米）。
    # n_boxes=%d（来自 %d 个 link 的 collision；已被"去掉被包含者"精简过）
''' % (len(boxes), r_near, z_floor, BLIND_R, r_near, clearance, z_floor, r_near, z_floor,
       len(boxes), n_links)
    body = yaml_boxes(boxes)
    txt = head + body + '\n'
    with open(OUT_ROBOT11, 'w') as f:
        f.write(txt)
    t2 = '''# =============================================================================
# traversability_self_mask.yaml —— 自击掩膜的**默认（关闭）**参数文件
#
# 为什么存在这份文件：launch 用与 linefit 的 `segmentation_sim_<slot>.yaml` **同款的槽位机制**
#   给地面分割节点选参数文件（`_RobotSlotFile`），需要一个"默认路径"。
#   默认 = **关**（`self_mask_enable: false`）⇒ 与 C++ 里 `SelfMask::enable{false}` 的兜底默认一致，
#   也与本判据引入掩膜之前的行为**逐字节相同**。
# 只有 `robot:=robot11` 会切到 `traversability_self_mask_robot11.yaml`（= 开 + 点表）。
# 生成/验证：python3 tools/scripts/regress/robot11_self_mask.py --emit|--check
# 关联文档：docs/robot_models.md §12。
# =============================================================================
ground_segmentation:
  ros__parameters:
    # false ⇒ `LowTerrainClassifier::buildGrid()` 完全不走掩膜分支（旧的建格行为）。
    self_mask_enable: false
'''
    with open(OUT_DEFAULT, 'w') as f:
        f.write(t2)
    return boxes, z_floor


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--emit', action='store_true', help='重写两份 YAML')
    ap.add_argument('--verify', action='store_true', help='离线验证掩膜覆盖率')
    ap.add_argument('--check', action='store_true', help='断言 YAML == 由模型重算的结果')
    ap.add_argument('--dump-fk', action='store_true', help='打印零位 FK（与运行期 TF 对照）')
    ap.add_argument('--tolerance-m', type=float, default=0.005)
    ap.add_argument('--ground-clearance-m', type=float, default=0.03)
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    if a.dump_fk:
        pose, _ = collision_aabbs(run_xacro())
        for k, (t, R) in sorted(pose.items()):
            tl = t - LIVOX_XYZ
            print('%-12s livox_xyz=[%9.6f %9.6f %9.6f]' % (k, tl[0], tl[1], tl[2]))
        return 0
    if a.emit:
        boxes, zf = emit(a.tolerance_m, a.ground_clearance_m)
        print('[emit] %s : %d boxes + 近场死区 r<=%.4f m, z_floor=%.5f（几何盲半径 %.4f m）'
              % (OUT_ROBOT11, len(boxes), near_field_radius(), zf, BLIND_R))
        print('[emit] %s : self_mask_enable=false' % OUT_DEFAULT)
    if a.verify:
        res = verify(a.tolerance_m, a.ground_clearance_m, json_out=a.json or None)
        print(json.dumps(res, indent=1, ensure_ascii=False))
    if a.check:
        boxes, zf, _pose = build_mask(a.tolerance_m, a.ground_clearance_m)
        want = yaml_boxes(boxes)
        have = open(OUT_ROBOT11).read()
        r_near = near_field_radius()
        for key, val in (('self_mask_radius_m', '%.4f' % r_near),
                         ('self_mask_z_min_m', '%.5f' % zf)):
            if '%s: %s' % (key, val) not in have:
                print('[check] ❌ %s 里 %s 应为 %s（跑 --emit）' % (OUT_ROBOT11, key, val))
                return 1
        if want not in have:
            print('[check] ❌ %s 与由模型重算的掩膜不一致（模型改了？跑 --emit）' % OUT_ROBOT11)
            return 1
        if 'self_mask_enable: false' not in open(OUT_DEFAULT).read():
            print('[check] ❌ %s 必须 self_mask_enable: false' % OUT_DEFAULT)
            return 1
        print('[check] ✅ 掩膜与模型一致（%d boxes + 近场死区 r<=%.4f m，z_floor=%.5f；'
              '几何盲半径 %.4f m）' % (len(boxes), r_near, zf, BLIND_R))
    return 0


if __name__ == '__main__':
    sys.exit(main())
