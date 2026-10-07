#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tilt_mount_compare.py —— 两个安装档（plugin / urdf）的**离线对照**（2026-10-09）。

输入 = run_tilt_mount_probe.sh 的两份产物（probe.json + clouds.npz）。做四件事：

  H2（本地云配准）：把 A 档与 B 档的 `/livox/lidar/pointcloud`（frame_id=livox_frame）**刚体配准**
      ——"帧真的斜了"的正确实现应当给出 R⁻¹ ≈ 30°（同一世界景物在斜系里看到的是转过的）；
      实测若 ≈0° 且残差 ~mm ⇒ 两档的点云坐标**逐点相同** ⇒ 斜的只有"帧/label"，不是"数据"。
  H1（odom 帧斜不斜）：把 raw 云用两档各自的 **LIO TF（odom←livox_frame）** 搬到 odom 再拟合地面；
      再给出 /cloud_registered 在同一次跑里的地面倾角 ⇒ 二者之差就是"发布环节多转的那一下"。
      并用真值 `/odom_ground_truth` + TF(odom←base_link) 独立算 odom 帧相对真实重力的倾角。
  约束假设残差：对每一档，分别按 H_a（p_pub ≈ R(base←livox)·p_raw + t）与 H_b（p_pub ≈ p_raw + t）
      构造残差（最近邻配对），谁小谁成立 —— 这是"30° 是发布时转的还是 odom 帧本来就斜"的判据。
  H3（一侧盲区）：raw 云按方位 30° 扇区的点数/仰角包络/地面环半径 + 理论 FOV 下界（−7.22°…+55.22°
      的锥形视场经过安装倾角后的**解析包络**）。
  另外：自击掩膜条件在 raw 云上的命中数、p2l 高度带在**本帧坐标**里的命中数 —— 用来判"感知参数
      在 urdf 档还成不成立"。

用法：
  python3 tools/scripts/tiltmount/tilt_mount_compare.py \
      --a .tmp_tiltmount/tm_plugin --b .tmp_tiltmount/tm_urdf --out .tmp_tiltmount/compare.json
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FOV_LOW_DEG, FOV_HIGH_DEG = -7.22, 55.22          # 本仓 mid360.xacro 的垂直视场（sensor 系）
TILT_DEG = -30.0                                   # 安装倾角（roll，弧度 -0.523598775598293）
SELF_MASK_R = 0.2416                               # config/traversability_self_mask_robot11.yaml
SELF_MASK_Z = -0.2295
SECTOR_NAMES = ['%d..%d' % (a, a + 30) for a in range(-180, 180, 30)]
P2L_MIN_H, P2L_MAX_H = -1.0, 1.0                   # pointcloud_to_laserscan/config/laserscan_params.yaml


def load(d):
    with open(os.path.join(d, 'probe.json')) as fh:
        j = json.load(fh)
    z = np.load(os.path.join(d, 'clouds.npz'))
    return j, z


def rot_x(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def kabsch(A, B):
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    u, s, vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    R = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return R, cb - R @ ca


def angle_axis(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.degrees(math.acos(c))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    return ang, (ax / n if n > 1e-9 else np.zeros(3))


def register(A, B, tol=0.25, rounds=3):
    """最近邻配对 + Kabsch 迭代（A→B）。返回 R,t,残差,配对数。"""
    from scipy.spatial import cKDTree
    R, t = np.eye(3), B.mean(axis=0) - A.mean(axis=0)
    idx = None
    m = None
    for _ in range(rounds):
        tree = cKDTree(B)
        d, idx = tree.query((R @ A.T).T + t, k=1)
        m = d < tol
        if m.sum() < 50:
            break
        R, t = kabsch(A[m], B[idx[m]])
    d, idx = cKDTree(B).query((R @ A.T).T + t, k=1)
    m = d < tol
    resid = d[m]
    return {'R': R, 't': t, 'n_pairs': int(m.sum()),
            'angle_deg': angle_axis(R)[0], 'axis': angle_axis(R)[1],
            'resid_p50_m': float(np.percentile(resid, 50)) if resid.size else None,
            'resid_p95_m': float(np.percentile(resid, 95)) if resid.size else None,
            'resid_max_m': float(resid.max()) if resid.size else None}


def constrained_resid(A, B, R_fix, tol=0.25):
    """p_B ≈ R_fix·p_A + t（t 用均值差；再用最近邻看残差）。"""
    from scipy.spatial import cKDTree
    t = B.mean(axis=0) - (R_fix @ A.mean(axis=0))
    Q = (R_fix @ A.T).T + t
    d, _ = cKDTree(B).query(Q, k=1)
    m = d < tol
    return {'t_fit': [float(v) for v in t], 'n_pairs': int(m.sum()),
            'resid_p50_m': float(np.percentile(d[m], 50)) if m.any() else None,
            'resid_p95_m': float(np.percentile(d[m], 95)) if m.any() else None,
            'frac_within_5cm': float((d < 0.05).mean()),
            'frac_within_10cm': float((d < 0.10).mean())}


def analytic_lower_envelope():
    """锥形视场（绕 z_s 的 e_s ∈ [−7.22,+55.22]）经过 R_x(TILT) 后，逐方位的**理论最低仰角**。"""
    th = math.radians(TILT_DEG)
    zs = np.array([0.0, -math.sin(th), math.cos(th)])       # 传感器 z 轴在世界（=云坐标）里
    out = []
    for lo in range(-180, 180, 30):
        phis = np.radians(np.linspace(lo, lo + 30, 13))
        best = None
        # d·zs = A sin e + B cos e，A = zs_z, B = zs_y·sinφ
        for phi in phis:
            A, B = zs[2], zs[1] * math.sin(phi)
            Rm = math.hypot(A, B)
            delta = math.atan2(B, A)
            s = math.sin(math.radians(FOV_LOW_DEG)) / Rm
            if abs(s) > 1.0:
                continue
            e = math.asin(s) - delta
            best = e if best is None else min(best, e)
        out.append({'sector_deg': [lo, lo + 30],
                    'fov_lower_env_deg': None if best is None else math.degrees(best)})
    return out


def sec_table(sectors, env):
    rows = []
    for s, e in zip(sectors, env):
        rows.append({'sector': '%d..%d' % (s['sector_deg'][0], s['sector_deg'][1]),
                     'n': s['n'], 'n_ground': s['n_ground'],
                     'elev_min': s['elev_min_deg'], 'elev_max': s['elev_max_deg'],
                     'rxy_min_ground': s['rxy_min_ground_m'],
                     'fov_lower_env': e['fov_lower_env_deg']})
    return rows



def newest(Z, key):
    ks = []
    for k in Z.files:
        if not k.startswith(key + '_'):
            continue
        suf = k.rsplit('_', 1)[1]
        if suf.isdigit():
            ks.append((int(suf), k))
    return Z[sorted(ks)[-1][1]] if ks else None


def plane_angle(P):
    """地面平面法向与本帧 z 的夹角（RANSAC 最大平面）。"""
    from tilt_mount_probe import fit_plane_ransac
    pl = fit_plane_ransac(P)
    return None if pl is None else pl['ang_from_frame_z_deg']


def scan_z_from_json(J):
    """用 /scan 里存的波束 + TF(odom<-livox_frame) 算每条波束在 odom 里的绝对 z
    （nav2 obstacle_layer 的 min/max_obstacle_height 就是在这一层量的）。"""
    s = J.get('scan')
    if not s:
        return None
    tf = (J.get('tf') or {}).get('odom<- %s' % s['frame_id'])
    if not tf:
        return None
    R = np.array(tf['R'], dtype=np.float64)
    t = np.array(tf['xyz'], dtype=np.float64)
    if 'ranges' not in s:
        return None
    r = np.array(s['ranges'], dtype=np.float64)
    fin = np.isfinite(r)
    ang = s['angle_min'] + np.arange(len(r)) * s['angle_increment']
    pts = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)[fin]
    Q = (R @ pts.T).T + t
    z = Q[:, 2]
    az = np.degrees(np.arctan2(Q[:, 1], Q[:, 0]))
    drop = (z < 0.0) | (z > 2.0)
    h, _ = np.histogram(az[drop], bins=np.arange(-180, 181, 30))
    return {'n_beams': int(fin.sum()), 'z_odom_min': float(z.min()), 'z_odom_p50': float(np.percentile(z, 50)),
            'z_odom_max': float(z.max()), 'n_below_0': int((z < 0).sum()),
            'n_above_2': int((z > 2).sum()), 'frac_dropped_by_band': float(drop.mean()),
            'dropped_az_hist': {SECTOR_NAMES[i]: int(h[i]) for i in range(len(h))},
            'note': 'band = obstacle_layer min/max_obstacle_height 0.0/2.0（odom 绝对高度）'}


def inverse_transform_test(J, Z):
    """把 /cloud_registered 按 R(base_link<-livox_frame)⁻¹ 反变换回去再拟合地面。
    预测（机制 = 发布环节多转了一次）：反变换后地面变水平；若"odom 帧本来就斜"则不成立。"""
    reg = newest(Z, 'registered')
    if reg is None:
        return None
    bl = (J.get('tf') or {}).get('base_link<- livox_frame')
    if not bl:
        return None
    R = np.array(bl['R'], dtype=np.float64)
    t = np.array(bl['xyz'], dtype=np.float64)
    Q = (R.T @ (reg - t).T).T
    n = np.array(J['clouds']['registered'][-1]['plane']['n'])
    return {'published_plane_ang_deg': J['clouds']['registered'][-1]['plane']['ang_from_frame_z_deg'],
            'after_inverse_plane_ang_deg': plane_angle(Q),
            'R_rpy_deg': bl['rpy_deg'], 'R_rot_deg': angle_axis(R)[0],
            'note': '反变换 = 抵消 T(base_link<-livox_frame)（small_point_lio_node.cpp 对已在 odom 的点又乘了一次）'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', required=True, help='plugin 档目录')
    ap.add_argument('--b', required=True, help='urdf 档目录')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    JA, ZA = load(args.a)
    JB, ZB = load(args.b)
    rep = {}

    # ---- 取"最新一帧"的 raw / registered
    raw_a, raw_b = newest(ZA, 'raw'), newest(ZB, 'raw')
    reg_a, reg_b = newest(ZA, 'registered'), newest(ZB, 'registered')
    rep['n_points'] = {'raw_a': None if raw_a is None else int(len(raw_a)),
                       'raw_b': None if raw_b is None else int(len(raw_b)),
                       'reg_a': None if reg_a is None else int(len(reg_a)),
                       'reg_b': None if reg_b is None else int(len(reg_b))}

    # ---- H2：两档 raw 云（livox_frame）刚体配准
    if raw_a is not None and raw_b is not None:
        r = register(raw_a, raw_b)
        rep['H2_raw_local_registration'] = {
            'n_pairs': r['n_pairs'], 'angle_deg': r['angle_deg'], 'axis': r['axis'],
            't': [float(v) for v in r['t']],
            'resid_p50_m': r['resid_p50_m'], 'resid_p95_m': r['resid_p95_m'],
            'resid_max_m': r['resid_max_m'],
            'comment': '≈0° + mm 残差 ⇒ 两档点云坐标逐点相同（倾角没进数据）'}
        # 直接逐点比（同长度时按索引比，否则最近邻的最大偏差）
        if len(raw_a) == len(raw_b):
            d = np.linalg.norm(raw_a - raw_b, axis=1)
            rep['H2_raw_local_registration']['per_index_max_diff_m'] = float(d.max())
            rep['H2_raw_local_registration']['per_index_frac_lt_5mm'] = float((d < 0.005).mean())
        else:
            from scipy.spatial import cKDTree
            d, _ = cKDTree(raw_b).query(raw_a, k=1)
            rep['H2_raw_local_registration']['nn_max_diff_m'] = float(d.max())
    # ---- 两档 /cloud_registered 配准（预测：≈30° 绕 x）
    if reg_a is not None and reg_b is not None:
        r = register(reg_a, reg_b)
        rep['registered_cross_variant_registration'] = {
            'n_pairs': r['n_pairs'], 'angle_deg': r['angle_deg'], 'axis': r['axis'],
            't': [float(v) for v in r['t']], 'resid_p50_m': r['resid_p50_m'],
            'resid_p95_m': r['resid_p95_m']}

    # ---- 约束假设残差（每档各自：raw → registered）
    hyp = {}
    for name, J, Z in (('plugin', JA, ZA), ('urdf', JB, ZB)):
        ra, rg = newest(Z, 'raw'), newest(Z, 'registered')
        if ra is None or rg is None:
            continue
        bl = J.get('tf', {}).get('base_link<- livox_frame')
        R_bl = np.array(bl['R']) if bl else np.eye(3)
        hyp[name] = {
            'T_base_link_from_livox_rpy_deg': bl['rpy_deg'] if bl else None,
            'T_base_link_from_livox_xyz': bl['xyz'] if bl else None,
            'H_a_extra_rot': constrained_resid(ra, rg, R_bl),
            'H_b_no_rot': constrained_resid(ra, rg, np.eye(3)),
            'free_registration': {k: v for k, v in register(ra, rg).items() if k not in ('R',)},
        }
        hyp[name]['H_a_extra_rot']['rot_deg'] = angle_axis(R_bl)[0]
        hyp[name]['H_a_extra_rot']['rot_axis'] = angle_axis(R_bl)[1]
    rep['hypothesis_residuals'] = hyp

    # ---- odom 帧里的地面（raw 经 LIO 自己的 TF 搬到 odom）
    for name, J in (('plugin', JA), ('urdf', JB)):
        v = (J.get('clouds') or {}).get('raw_via_tf') or {}
        o = v.get('odom') if isinstance(v, dict) else None
        rep.setdefault('odom_frame_ground', {})[name] = {
            'raw_via_tf_odom_plane_ang_deg': None if not o else
            o['plane']['ang_from_frame_z_deg'],
            'raw_via_tf_odom_plane_n': None if not o else o['plane']['n'],
            'raw_via_tf_odom_plane_d': None if not o else o['plane']['d'],
            'published_registered_plane_ang_deg': (
                (J['clouds']['registered'][-1]['plane'] or {}).get('ang_from_frame_z_deg')
                if J.get('clouds', {}).get('registered') else None),
            'odom_tilt_from_gravity_deg': (J.get('ground_truth') or {}).get(
                'odom_tilt_from_gravity_deg'),
            'R_odom_from_base_rpy_deg': (J.get('ground_truth') or {}).get(
                'R_odom_from_base_rpy_deg'),
            'checker': 'odom 里的地面（raw 经 TF）水平 ⇒ 是发布环节多转了一次，不是 odom 帧斜',
        }

    # ---- IMU / 外参
    rep['imu'] = {n: J.get('imu') for n, J in (('plugin', JA), ('urdf', JB))}
    rep['extrinsic_check'] = {}
    for n, J in (('plugin', JA), ('urdf', JB)):
        t = J.get('tf', {})
        rep['extrinsic_check'][n] = {
            'T_imu_link_from_livox_frame': t.get('imu_link<- livox_frame'),
            'config_extrinsic_T': [0.0, 0.0, 0.05],
            'config_extrinsic_R': 'identity'}

    # ---- H3：扇区表 + 解析 FOV 包络
    env = analytic_lower_envelope()
    rep['H3_sectors'] = {}
    for n, J in (('plugin', JA), ('urdf', JB)):
        raw = (J.get('clouds') or {}).get('raw') or []
        if not raw:
            continue
        f = raw[-1]
        rep['H3_sectors'][n] = {
            'n': f['n'], 'plane': f['plane'], 'ground_ring': f.get('ground_ring'),
            'quadrants': f['quadrants'], 'rows': sec_table(f['sectors'], env),
            'n_below_horizontal_plane': f['n_below_horizontal_plane'],
            'elev_pct': f['elev_pct']}
        # 自击掩膜 / p2l 高度带在**本帧坐标**里的命中（判"感知参数还成不成立"）
        ra = newest(ZA if n == 'plugin' else ZB, 'raw')
        if ra is not None:
            rxy = np.hypot(ra[:, 0], ra[:, 1])
            rep['H3_sectors'][n]['self_mask_hits'] = int(
                ((rxy <= SELF_MASK_R) & (ra[:, 2] >= SELF_MASK_Z)).sum())
            rep['H3_sectors'][n]['self_mask_frac'] = float(
                ((rxy <= SELF_MASK_R) & (ra[:, 2] >= SELF_MASK_Z)).mean())
            rep['H3_sectors'][n]['in_p2l_band_frac'] = float(
                ((ra[:, 2] >= P2L_MIN_H) & (ra[:, 2] <= P2L_MAX_H)).mean())
    rep['H3_analytic_fov_envelope'] = env

    # ---- 反变换检验（机制判据）+ scan 的 odom 绝对高度带
    rep['inverse_transform_test'] = {n: inverse_transform_test(J, Z)
                                     for n, J, Z in (('plugin', JA, ZA), ('urdf', JB, ZB))}
    rep['scan_z_band'] = {n: scan_z_from_json(J) for n, J in (('plugin', JA), ('urdf', JB))}

    # ---- scan / costmap
    rep['scan'] = {n: {k: v for k, v in (J.get('scan') or {}).items() if k != 'ranges'}
                   for n, J in (('plugin', JA), ('urdf', JB))}
    rep['costmap'] = {n: J.get('costmap_local') for n, J in (('plugin', JA), ('urdf', JB))}

    txt = json.dumps(rep, indent=1, ensure_ascii=False, default=str)
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(txt)
        print('[compare] 写出 %s' % args.out)
    print(txt)


if __name__ == '__main__':
    main()
