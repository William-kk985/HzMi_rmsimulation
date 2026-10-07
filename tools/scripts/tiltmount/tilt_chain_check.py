#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tilt_chain_check.py —— "斜装正确链"的**逐级坐标系自检**（2026-10-09，为 §J 服务）。

回答三件事（都只用 `run_tilt_mount_probe.sh` 的产物，不需要 Gazebo）：

  ① **点云 ↔ 它自称的帧**：把 `/livox/lidar/pointcloud` 的最大平面（地面）法向
     · 与**自己的 `frame_id`** 的 z 比（自洽 ⇒ ≈1°；`urdf` 档的旧状态是"数据水平、帧斜 30°"⇒ 仍是 1°，
       所以这一条**必须**配合下一条看）；
     · 再用 TF `base_link ← frame_id` 的旋转搬到 `base_link` 后比（**正确链** ⇒ ≈1°；
       `plugin`/`urdf` 两档因为数据本来就在水平系里，也是 ≈1°）。
     判据：**"帧真的斜了"**（`sensor` 档）时，第 1 条应当 ≈**30°**、第 2 条 ≈1° —— 这才叫"数据在传感器系、
     与帧自洽"。§I.2 的三分法（物理系／数据实际表达的系／`frame_id`）在这一条上被量化。
  ② **自击掩膜要不要重烘**：掩膜在 linefit 内部作用在 `cloud_proc` 上（= 点云被
     `gravity_aligned_frame` **只旋转**后的坐标）。这里复刻同一步：把原始云按
     `TF(base_link←livox_frame)` 的旋转转过去，再数 `r_xy <= SELF_MASK_R and z >= SELF_MASK_Z`
     的占比 ⇒ 应当与 `plugin` 档基线（≈29.2%）一致 ⇒ **不用重烘**。
  ③ **p2l 高度带**：同一份旋转后坐标里 `min_height(-1.0) <= z <= max_height(1.0)` 的占比。
     （`sensor` 档下 p2l 会用 `target_frame: base_link` 的**完整** TF —— 含平移 —— 这里给的是"旋转 + 带子"
     的量级判据。）

用法：
  python3 tools/scripts/tiltmount/tilt_chain_check.py --dir plugin:.tmp_tiltmount/j2_plugin \
      --dir urdf:.tmp_tiltmount/j2_urdf --dir sensor:.tmp_tiltmount/j2_sensor
"""
import argparse
import json
import math
import os

import numpy as np

SELF_MASK_R = 0.2416          # config/traversability_self_mask_robot11.yaml
SELF_MASK_Z = -0.2295
P2L_MIN_H, P2L_MAX_H = -1.0, 1.0


def rpy_to_R(rpy_deg):
    r, p, y = [math.radians(a) for a in rpy_deg]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def fit_plane(P, iters=400, tol=0.02, seed=0):
    if len(P) < 20:
        return None
    rng = np.random.default_rng(seed)
    best = (0, None, None)
    for _ in range(iters):
        i = rng.choice(len(P), 3, replace=False)
        p0, p1, p2 = P[i]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = -float(n @ p0)
        cnt = int((np.abs(P @ n + d) < tol).sum())
        if cnt > best[0]:
            best = (cnt, n.copy(), d)
    if best[1] is None:
        return None
    n, d = best[1], best[2]
    inl = np.abs(P @ n + d) < tol
    A = P[inl]
    if len(A) >= 3:
        c = A.mean(axis=0)
        _, _, vt = np.linalg.svd(A - c)
        n = vt[2]
        d = -float(n @ c)
    if n[2] < 0:
        n, d = -n, -d
    return {'n': n, 'd': d, 'ang_from_z_deg': math.degrees(
        math.acos(max(-1.0, min(1.0, abs(float(n[2]))))))}


def tf_from_json(d):
    R = np.array(d['R'], dtype=float)
    t = np.array(d['xyz'], dtype=float)
    return R, t


def nn_resid(A, B):
    from scipy.spatial import cKDTree
    d, _ = cKDTree(B).query(A, k=1)
    return (float(np.percentile(d, 50)), float(np.percentile(d, 95)), float(d.max()))


def bug2_check(J, z):
    """bug ② 的**判决性残差**（任何安装档都能跑）：

    用本次跑的 TF 反解出 LIO 自己的状态 `T_ol = T_ob · T_bl⁻¹`（`T_ob` = 发布的 odom→base_link、
    `T_bl` = base_link←livox_frame），再对同一帧的原始云分别按两种发布假设预测，
    与实际发布的 `/cloud_registered` 做最近邻残差比较：
      H_fixed（**修好后的代码**）：p_pub = T_ol · p_raw
      H_old  （2026-10-09 之前）  ：p_pub = T_bl⁻¹ · (T_ol · p_raw)
    残差小者 = 当前代码在做的事；两档的差 = |t|（默认档 0.2044 m）⇒ 判据很硬。
    """
    try:
        raw_k = sorted([k for k in z.files if k.startswith('raw_')])[0]
        reg_k = sorted([k for k in z.files if k.startswith('registered_')])[0]
    except Exception:
        return None
    P, Q = z[raw_k], z[reg_k]
    tf = J.get('tf') or {}
    if not (tf.get('odom<- base_link') and tf.get('base_link<- livox_frame')):
        return None
    Rob, tob = tf_from_json(tf['odom<- base_link'])
    # ⚠️ 探针的 'base_link<- livox_frame' = lookupTransform(base_link, livox_frame) = T(base←livox)，
    #    而代码里的 `tf_base_link_to_lidar_frame` = lookupTransform(livox_frame, base_link) = 它的逆。
    R_blc, t_blc = tf_from_json(tf['base_link<- livox_frame'])   # T(base←livox)（探针口径）
    # T_ob = T_ol·T(livox←base) ⇒ T_ol = T_ob·T(base←livox)：R_ol = R_ob·R_blc，t_ol = t_ob + R_ob·t_blc
    Rol = Rob @ R_blc
    tol = tob + Rob @ t_blc
    # ⚠️ 出点公式里还有 LIO 配置的 extrinsic_T = (0, 0, 0.05)（lidar→imu，见 mid360_sim_tuned.yaml）
    ext = np.array([0.0, 0.0, 0.05])
    fixed = (Rol @ (P + ext).T).T + tol
    # 旧代码：把已经在 odom 的点再乘一次 T(base_link←livox_frame) = T_bl⁻¹
    Rbl_inv = R_blc            # 旧代码里乘的是 T(base←livox) = 探针口径的这条
    tbl_inv = t_blc
    oldp = (Rbl_inv @ fixed.T).T + tbl_inv
    # 判据用**地面平面**（点数最多的那个平面）而不是最近邻：两朵云是**不同帧**（落盘时机不同、
    # 车在 settle 后仍有微小晃动）⇒ 最近邻残差底噪 ~0.2 m；而平面拟合有 ~4800 个内点、robust。
    def plane(Qq):
        f = fit_plane(Qq)
        return None if f is None else (f['ang_from_z_deg'], f['d'], f['n'])
    return {'n_raw': int(len(P)), 'n_pub': int(len(Q)),
            'T_ol_rpy_deg': [round(math.degrees(a), 3) for a in _rpy(Rol)],
            'published_plane_ang_deg': round(plane(Q)[0], 4),
            'H_fixed_plane_ang_deg': round(plane(fixed)[0], 4),
            'H_old_plane_ang_deg': round(plane(oldp)[0], 4),
            'note': '只比**平面法向角**：平移里含"有意保留的旧值"⇒ d 不可比（见 §J.5）'}


def _rpy(R):
    sy = max(-1.0, min(1.0, -R[2, 0]))
    pitch = math.asin(sy)
    if abs(sy) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', action='append', default=[], metavar='LABEL:DIR')
    args = ap.parse_args()

    print('| 档 | raw 点数 | 地面法向 vs **自己 frame_id** z | 经 TF 转到 base_link 后 | frame_id | TF base_link←livox rpy | 自击掩膜命中（旋转后） | p2l 高度带内（旋转后） |')
    print('|---|---|---|---|---|---|---|---|')
    for spec in args.dir:
        label, _, d = spec.partition(':')
        with open(os.path.join(d, 'probe.json'), encoding='utf-8') as fh:
            J = json.load(fh)
        z = np.load(os.path.join(d, 'clouds.npz'))
        raw_keys = sorted([k for k in z.files if k.startswith('raw_')])
        P = z[raw_keys[0]] if raw_keys else None
        bl = (J.get('tf') or {}).get('base_link<- livox_frame') or {}
        pl_own = fit_plane(P) if P is not None else None
        pl_base = None
        mask_frac = band_frac = float('nan')
        if P is not None and bl:
            R = rpy_to_R(bl['rpy_deg'])
            Q = (R @ P.T).T
            pl_base = fit_plane(Q)
            rxy = np.hypot(Q[:, 0], Q[:, 1])
            mask_frac = float(((rxy <= SELF_MASK_R) & (Q[:, 2] >= SELF_MASK_Z)).mean())
            band_frac = float(((Q[:, 2] >= P2L_MIN_H) & (Q[:, 2] <= P2L_MAX_H)).mean())
        fids = (J.get('frame_ids') or {}).get('raw') or {}
        fid = max(fids, key=fids.get) if fids else '—'
        print('| %s | %d | %.3f° | %s | `%s` | %s | %.3f%% | %.4f |' % (
            label, 0 if P is None else len(P),
            pl_own['ang_from_z_deg'] if pl_own else float('nan'),
            ('%.3f°' % pl_base['ang_from_z_deg']) if pl_base else '—',
            fid, bl.get('rpy_deg'), 100 * mask_frac, band_frac))
        b2 = bug2_check(J, z)
        if b2:
            print('    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：'
                  '实测发布 = %.3f° ；H_fixed(现在的代码) = %.3f° ；H_old(旧代码) = %.3f°'
                  '（反解 T_ol(=LIO 状态) rpy = %s；两条假设的平移差 = |t| = 0.2044 m）' % (
                      b2['published_plane_ang_deg'], b2['H_fixed_plane_ang_deg'],
                      b2['H_old_plane_ang_deg'], b2['T_ol_rpy_deg']))


if __name__ == '__main__':
    main()
