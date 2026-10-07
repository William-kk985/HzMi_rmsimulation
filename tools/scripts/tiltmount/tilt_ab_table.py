#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tilt_ab_table.py —— 安装档 A/B 表 + "改动前/后"逐项 diff（2026-10-09）。

给 docs/tilted_lidar_fidelity.md §J 用：把任意几份 `run_tilt_mount_probe.sh` 的产物
（`probe.json`）排成一张表，并支持把两份（同一安装档、改动前 vs 改动后）逐项 diff。

用法：
  # 三方 A/B 表（markdown）
  python3 tools/scripts/tiltmount/tilt_ab_table.py \
      --col plugin:.tmp_tiltmount/j1_plugin \
      --col urdf:.tmp_tiltmount/j1_urdf \
      --col sensor:.tmp_tiltmount/j1_sensor [--col "plugin(before):.tmp_tiltmount/tm_plugin"] \
      --out .tmp_tiltmount/ab_table.md
  # 同一档的"改动前 vs 改动后"逐项 diff（默认档回归证据）
  python3 tools/scripts/tiltmount/tilt_ab_table.py --diff before:.tmp_tiltmount/tm_plugin \
      after:.tmp_tiltmount/j1_plugin --json .tmp_tiltmount/plugin_before_after.json

口径说明（都直接取探针里的量，不在这里重新定义）：
  · `raw_ground_ang_deg` = `/livox/lidar/pointcloud` 最大平面（地面）法向与**它自己的 frame_id z**
    的夹角；`raw_ground_ang_vs_base_deg` = 同一平面的法向经 TF(base_link←frame_id) 旋转后与 base_link
    z 的夹角（"账对不对"的判据：帧与数据自洽时它应当很小）。
  · `registered_ground_ang_deg` = `/cloud_registered`（frame_id=odom）里地面的倾角；
    `registered_via_inv_TF_ang_deg` = 把它按同一条 TF 反变换回去后的倾角（bug ② 的判据）。
  · `scan_z_outside_0_2_frac` = `/scan` 逐波束搬到 odom 后落在 `obstacle_layer`
    `min/max_obstacle_height=0/2.0` 之外的比例（"被高度带丢掉"的验收数，urdf 档旧值 71.4%）。
  · `circle_*` = local costmap 里"车半径圆内"的格数/≥99/free（`--robot-radius`，robot11 = 0.3565）。
"""
import argparse
import json
import math
import os
import sys

import numpy as np


def load(d):
    with open(os.path.join(d, 'probe.json'), encoding='utf-8') as fh:
        return json.load(fh)


def _plane(J, tag, i=-1):
    try:
        return (J['clouds'][tag][i].get('plane') or {})
    except Exception:
        return {}


def _first(d, *path, default=None):
    cur = d
    for p in path:
        if isinstance(cur, dict) and p in cur:
            cur = cur[p]
        elif isinstance(cur, list) and isinstance(p, int) and -len(cur) <= p < len(cur):
            cur = cur[p]
        else:
            return default
    return cur


def rot_apply(rpy_deg, v):
    """rpy(ZYX, 度) 旋转矩阵作用到向量（与探针同一个 R_to_rpy_deg 约定）。"""
    r, p, y = [math.radians(a) for a in rpy_deg]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    return R @ np.asarray(v, dtype=float)


def metrics(J):
    """从一份 probe.json 里抽出 A/B 表要用的量（缺的填 None，不猜）。"""
    m = {}
    tf = J.get('tf') or {}
    bl = tf.get('base_link<- livox_frame') or {}
    ob = tf.get('odom<- base_link') or {}
    m['variant'] = J.get('variant')
    m['tf_livox_rpy_deg'] = bl.get('rpy_deg')
    m['tf_odom_base_rpy_deg'] = ob.get('rpy_deg')
    m['tf_odom_base_xyz'] = ob.get('xyz')
    raw = _plane(J, 'raw')
    def _dom(d):
        return max(d, key=d.get) if isinstance(d, dict) and d else d
    m['raw_frame_id'] = _dom(_first(J, 'frame_ids', 'raw', default=None))
    m['raw_ground_ang_deg'] = raw.get('ang_from_frame_z_deg')
    m['raw_ground_height_m'] = raw.get('sensor_height_m')
    m['raw_ground_n'] = raw.get('n')
    # 把 raw 地面法向经 TF(base_link←frame_id) 旋到 base_link 系里再量夹角 = "账对不对"
    try:
        n = np.array(raw['n'], dtype=float)
        nb = rot_apply(bl['rpy_deg'], n)
        m['raw_ground_ang_vs_base_deg'] = math.degrees(
            math.acos(max(-1.0, min(1.0, abs(float(nb[2]))))))
    except Exception:
        m['raw_ground_ang_vs_base_deg'] = None
    reg = _plane(J, 'registered')
    m['registered_frame_id'] = _dom(_first(J, 'frame_ids', 'registered', default=None))
    m['registered_ground_ang_deg'] = reg.get('ang_from_frame_z_deg')
    m['registered_below_horizontal_n'] = _first(J, 'clouds', 'registered', 0,
                                                'n_below_horizontal_plane', default=None)
    m['registered_n'] = reg.get('n')
    m['odom_tilt_from_gravity_deg'] = _first(J, 'ground_truth', 'odom_tilt_from_gravity_deg')
    m['base_rpy_in_world_deg'] = _first(J, 'ground_truth', 'base_rpy_in_world_deg')
    m['odom_rpy_span_deg'] = _first(J, 'odom_series', '/odom', 'rpy_span_deg')
    m['pts'] = J.get('pts_median') or {}
    m['scan_n_finite'] = _first(J, 'scan', 'n_finite')
    m['scan_frame_id'] = _first(J, 'scan', 'frame_id')
    m['scan_bands'] = _first(J, 'scan', 'range_bands_per_frame')
    m['scan_z_outside_0_2_frac'] = _first(J, 'scan', 'beam_z_odom_absolute',
                                          'frac_outside_0_2')
    m['scan_z_min'] = _first(J, 'scan', 'beam_z_odom_absolute', 'min')
    m['scan_z_max'] = _first(J, 'scan', 'beam_z_odom_absolute', 'max')
    cm = J.get('costmap_local') or {}
    m['costmap_frame_id'] = cm.get('frame_id')
    m['cm_lethal'] = cm.get('n_lethal')
    m['cm_inscribed'] = cm.get('n_inscribed')
    m['cm_free'] = cm.get('n_free')
    m['cm_center_cell'] = cm.get('robot_center_cell')
    m['cm_lethal_r_min_m'] = cm.get('lethal_r_min_m')
    m['cm_circle_cells'] = cm.get('circle_cells')
    m['cm_circle_ge99'] = cm.get('circle_n_ge99')
    m['cm_circle_free'] = cm.get('circle_n_free')
    m['cm_lethal_quadrants'] = cm.get('lethal_quadrants')
    m['rtf'] = _first(J, 'rtf', 'rtf')
    m['rtf_sim_s'] = _first(J, 'rtf', 'sim_seconds')
    m['cmd_vel'] = J.get('cmd_vel')
    m['plan'] = J.get('plan')
    m['drive'] = J.get('drive')
    m['goal'] = J.get('goal')
    return m


def fmt(v, nd=3):
    if v is None:
        return '—'
    if isinstance(v, float):
        if math.isnan(v):
            return 'NaN'
        return ('%.' + str(nd) + 'f') % v
    if isinstance(v, (list, tuple)):
        return '[' + ', '.join(fmt(x, nd) for x in v) + ']'
    if isinstance(v, dict):
        return '{' + ', '.join('%s=%s' % (k, fmt(x, nd)) for k, x in v.items()) + '}'
    return str(v)


ROWS = [
    ('安装档 variant', 'variant'),
    ('TF base_link→livox_frame rpy(度)', 'tf_livox_rpy_deg'),
    ('TF odom→base_link rpy(度)', 'tf_odom_base_rpy_deg'),
    ('TF odom→base_link xyz(m)', 'tf_odom_base_xyz'),
    ('原始云 frame_id', 'raw_frame_id'),
    ('原始云地面法向 vs **自己的 frame_id** z(度)', 'raw_ground_ang_deg'),
    ('原始云地面法向 vs **base_link** z(度)（账对不对）', 'raw_ground_ang_vs_base_deg'),
    ('原始云地面平面高(m)', 'raw_ground_height_m'),
    ('/cloud_registered frame_id', 'registered_frame_id'),
    ('/cloud_registered 地面倾角（odom 里，度）', 'registered_ground_ang_deg'),
    ('/cloud_registered 低于水平面点数', 'registered_below_horizontal_n'),
    ('odom 帧相对真实重力倾角(度)', 'odom_tilt_from_gravity_deg'),
    ('/odom 窗口内 rpy 跨度(度)', 'odom_rpy_span_deg'),
    ('/scan frame_id', 'scan_frame_id'),
    ('/scan 有限波束/帧', 'scan_n_finite'),
    ('/scan 波束在 odom 里超出 [0,2] m 的比例', 'scan_z_outside_0_2_frac'),
    ('/scan 波束 z(odom) min / max (m)', None),          # 特殊：两列合一
    ('local costmap frame_id', 'costmap_frame_id'),
    ('local costmap lethal / inscribed / free', None),   # 特殊
    ('车那格的值', 'cm_center_cell'),
    ('到最近 lethal 格距离(m)', 'cm_lethal_r_min_m'),
    ('车半径圆内格数 / ≥99 / free', None),                # 特殊
    ('lethal 方位（前后左右）', 'cm_lethal_quadrants'),
    ('RTF（记录窗）', 'rtf'),
    ('/cmd_vel 条数 / 非零', 'cmd_vel'),
    ('/plan 帧数 / 每条位姿数(中位)', 'plan'),
    ('--drive 漂移 vs 真值(m) / yaw(度)', None),          # 特殊
    ('--goal 真值位移(m) / 目标后残余(m)', None),          # 特殊
]

POINTS = [('原始云', 'raw'), ('/scan 输入(obstacle)', 'obstacle'),
          ('/segmentation/ground', 'ground'), ('/cloud_registered', 'registered'),
          ('voxel_grid', 'voxel_grid')]


def special_row(name, ms):
    if name.startswith('/scan 波束 z'):
        return [('%s / %s' % (fmt(m['scan_z_min']), fmt(m['scan_z_max']))) for m in ms]
    if name.startswith('local costmap lethal / inscribed'):
        return ['%s / %s / %s' % (fmt(m['cm_lethal'], 0), fmt(m['cm_inscribed'], 0),
                                  fmt(m['cm_free'], 0)) for m in ms]
    if name.startswith('车半径圆内'):
        return ['%s / %s / %s' % (fmt(m['cm_circle_cells'], 0), fmt(m['cm_circle_ge99'], 0),
                                  fmt(m['cm_circle_free'], 0)) for m in ms]
    if name.startswith('--drive'):
        return [('—' if not m['drive'] else '%s / %s' % (fmt(m['drive']['drift_vs_truth_m'], 4),
                fmt(m['drive']['drift_vs_truth_yaw_deg'], 3))) for m in ms]
    if name.startswith('--goal'):
        return [('—' if not m['goal'] else '%s / %s' % (fmt(m['goal'].get('gt_moved_m'), 4),
                fmt(m['goal'].get('dist_to_goal_after_m'), 4))) for m in ms]
    return None


def table(cols):
    labels = [c[0] for c in cols]
    ms = [metrics(load(c[1])) for c in cols]
    lines = ['| 量 | ' + ' | '.join(labels) + ' |',
             '|---|' + '---|' * len(labels)]
    for name, key in ROWS:
        sp = special_row(name, ms)
        if sp is not None:
            lines.append('| %s | %s |' % (name, ' | '.join(sp)))
        else:
            lines.append('| %s | %s |' % (name, ' | '.join(fmt(m.get(key)) for m in ms)))
    # 每帧点数
    lines.append('| 每帧点数（中位） | ' + ' | '.join(
        ', '.join('%s=%s' % (lab, fmt(m['pts'].get(k), 1)) for lab, k in POINTS)
        for m in ms) + ' |')
    return '\n'.join(lines), ms


# ------------------------------------------------------------------ 逐项 diff
DIFF_KEYS = [
    ('TF base_link→livox_frame rpy(度)', lambda m: m['tf_livox_rpy_deg']),
    ('TF odom→base_link rpy(度)', lambda m: m['tf_odom_base_rpy_deg']),
    ('TF odom→base_link xyz(m)', lambda m: m['tf_odom_base_xyz']),
    ('原始云地面倾角 vs 自己帧(度)', lambda m: m['raw_ground_ang_deg']),
    ('原始云地面平面高(m)', lambda m: m['raw_ground_height_m']),
    ('原始云点数(中位)', lambda m: m['pts'].get('raw')),
    ('/cloud_registered 点数(中位)', lambda m: m['pts'].get('registered')),
    ('/cloud_registered 地面倾角(度)', lambda m: m['registered_ground_ang_deg']),
    ('/cloud_registered 低于水平面点数', lambda m: m['registered_below_horizontal_n']),
    ('/segmentation/ground 点数(中位)', lambda m: m['pts'].get('ground')),
    ('/segmentation/obstacle 点数(中位)', lambda m: m['pts'].get('obstacle')),
    ('/scan 有限波束', lambda m: m['scan_n_finite']),
    ('/scan 超出高度带比例', lambda m: m['scan_z_outside_0_2_frac']),
    ('local costmap lethal', lambda m: m['cm_lethal']),
    ('local costmap inscribed', lambda m: m['cm_inscribed']),
    ('车那格的值', lambda m: m['cm_center_cell']),
    ('车半径圆内 ≥99 格', lambda m: m['cm_circle_ge99']),
    ('车半径圆内 free 格', lambda m: m['cm_circle_free']),
    ('到最近 lethal 格(m)', lambda m: m['cm_lethal_r_min_m']),
    ('RTF', lambda m: m['rtf']),
]


def diff_table(a_label, a, b_label, b, tol=1e-9):
    ma, mb = metrics(a), metrics(b)
    lines = ['| 量 | %s（改动前） | %s（改动后） | 差 |' % (a_label, b_label),
             '|---|---|---|---|']
    out = {}
    for name, get in DIFF_KEYS:
        va, vb = get(ma), get(mb)
        d = None
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            d = vb - va
        lines.append('| %s | %s | %s | %s |' % (name, fmt(va), fmt(vb), fmt(d, 6)))
        out[name] = {'before': va, 'after': vb, 'delta': d}
    lines.insert(0, '（差 = 改动后 − 改动前；`—` = 该跑没采到这个量）')
    return '\n'.join(lines), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--col', action='append', default=[],
                    help='label:dir（可多次；每份必须是 run_tilt_mount_probe.sh 的输出目录）')
    ap.add_argument('--diff', nargs=2, default=None, metavar=('LABEL:DIR', 'LABEL:DIR'),
                    help='同一安装档的"改动前 vs 改动后"逐项 diff')
    ap.add_argument('--out', default='')
    ap.add_argument('--json', default='')
    args = ap.parse_args()

    def parse_col(s):
        lab, _, d = s.partition(':')
        if not d:
            raise SystemExit('--col 需要 label:dir 形式，收到 %r' % s)
        return lab, d

    rep = {}
    if args.col:
        cols = [parse_col(c) for c in args.col]
        txt, ms = table(cols)
        print('### A/B 表\n')
        print(txt)
        rep['table'] = {c[0]: m for c, m in zip(cols, ms)}
    if args.diff:
        (la, da), (lb, db) = [parse_col(x) for x in args.diff]
        txt, out = diff_table(la, load(da), lb, load(db))
        print('\n### 改动前/后逐项 diff\n')
        print(txt)
        rep['diff'] = out
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps(rep, ensure_ascii=False, indent=1, default=str))
        print('\n[ab] 写出 %s' % args.out)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=1, default=str)
        print('[ab] 写出 %s' % args.json)


if __name__ == '__main__':
    main()
