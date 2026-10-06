#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_slope_speed_table.py —— 断言「前瞻限速」的**单一真源 + 物理自洽 + 与 nav2 对得上**。

为什么需要它（三件事，任何一件漂了都可能"以为限速了其实没限 / 限错速"）：

  ① **参数一致**：src/rm_nav_bringup/config/traversability_criteria.yaml 的 `speed_limit_*`
     必须与 C++ `struct SpeedLimitCriteria`
     （src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/
       slope_speed_limit.hpp）的字面默认值逐键相同
     —— 后者是"节点没拿到参数文件"时的兜底，与判据那套（check_traversability_criteria.py）
     同一个思路。
  ② **物理自洽**：两张表（坡度 deg→m/s、台阶 m→m/s）的每个结点反算
         a_peak = v · tanθ / Δt_eff,   Δt_eff = anchor_v·anchor_tan/anchor_peak
     必须 ≤ peak_target，**例外**是"已经落到地板值"的结点（地板 = 机构上还愿意走的最低速，
     到地板就承认模型解不出来，由实测兜）。同时断言两表单调（x 升序、v 不升）。
  ③ **与 nav2 两个真源对得上**：
         speed_limit_vx_max      == min(controller_server 的 FollowPath.vx_max,
                                        velocity_smoother.max_velocity[0])
         speed_limit_brake_mps2  ≤ |velocity_smoother.max_decel[0]|
     nav2 参数默认从 `nav2_params_sim_controller_mppi.yaml` 与 `nav2_params_sim_base.yaml` 读
     （`--mppi-file` / `--base-file` 可覆盖）。

用法：
  python3 tools/scripts/regress/check_slope_speed_table.py [--json]
退出码 0 = 全部通过；1 = 有漂移（打印哪一项、哪一侧、两个值）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
YAML = os.path.join(WS, 'src', 'rm_nav_bringup', 'config', 'traversability_criteria.yaml')
HPP = os.path.join(WS, 'src', 'rm_perception', 'rm_ground_traversability', 'include',
                   'rm_ground_traversability', 'slope_speed_limit.hpp')
MPPI_YAML = os.path.join(WS, 'src', 'rm_navigation', 'rm_navigation', 'params',
                         'nav2_params_sim_controller_mppi.yaml')
BASE_YAML = os.path.join(WS, 'src', 'rm_navigation', 'rm_navigation', 'params',
                         'nav2_params_sim_base.yaml')

# YAML 键 → C++ SpeedLimitCriteria 成员名（只有 enable 去掉前缀，其余同名）
SCALAR_KEYS = {
    'speed_limit_enable': 'enable',
    'speed_limit_vx_max': 'vx_max',
    'speed_limit_lookahead_m': 'lookahead_m',
    'speed_limit_corridor_half_width_m': 'corridor_half_width_m',
    'speed_limit_corridor_spread_deg': 'corridor_spread_deg',
    'speed_limit_slope_baseline_m': 'slope_baseline_m',
    'speed_limit_slope_change_deadband_deg': 'slope_change_deadband_deg',
    'speed_limit_step_deadband_m': 'step_deadband_m',
    'speed_limit_step_ignore_above_m': 'step_ignore_above_m',
    'speed_limit_brake_mps2': 'brake_mps2',
    'speed_limit_release_mps2': 'release_mps2',
    'speed_limit_hold_decay_factor': 'hold_decay_factor',
    'speed_limit_min_support_cells': 'min_support_cells',
    'speed_limit_rear_lookahead_m': 'rear_lookahead_m',
    'speed_limit_direction_aware': 'direction_aware',
    'speed_limit_direction_min_vx': 'direction_min_vx',
    'speed_limit_direction_timeout_s': 'direction_timeout_s',
    'speed_limit_floor_mps': 'floor_mps',
    'speed_limit_anchor_v_mps': 'anchor_v_mps',
    'speed_limit_anchor_tan_slope': 'anchor_tan_slope',
    'speed_limit_anchor_peak_mps2': 'anchor_peak_mps2',
    'speed_limit_peak_target_mps2': 'peak_target_mps2',
}
ARRAY_KEYS = {
    'speed_limit_slope_knots_deg': 'slope_knots_deg',
    'speed_limit_slope_knots_mps': 'slope_knots_mps',
    'speed_limit_step_knots_m': 'step_knots_m',
    'speed_limit_step_knots_mps': 'step_knots_mps',
}


def _num(text):
    t = text.strip().rstrip(',').strip()
    if t in ('true', 'True'):
        return True
    if t in ('false', 'False'):
        return False
    return float(t)


def read_yaml(path=YAML):
    """只认扁平 '键: 值'（本文件结构就是扁平的；与另两个校验脚本同一套极小解析器）。"""
    out, arrays = {}, {}
    for line in open(path, encoding='utf-8'):
        line = line.split('#')[0].rstrip()
        if not line or line.lstrip().startswith('/**') or 'ros__parameters' in line:
            continue
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        k, v = k.strip(), v.strip()
        if k in SCALAR_KEYS:
            out[k] = _num(v)
        elif k in ARRAY_KEYS:
            if not (v.startswith('[') and v.endswith(']')):
                raise SystemExit('❌ %s 的 %s 不是 [..] 列表：%r' % (path, k, v))
            arrays[k] = [float(x) for x in v[1:-1].split(',') if x.strip()]
    return out, arrays


def read_cpp(path=HPP):
    src = open(path, encoding='utf-8').read()
    body = src[src.index('struct SpeedLimitCriteria'):src.index('/// 标定出来的等效接触时间')]
    out, arrays = {}, {}
    for k, member in SCALAR_KEYS.items():
        m = re.search(r'\b' + re.escape(member) + r'\s*\{([^}]*)\}', body)
        if m:
            out[k] = _num(m.group(1))
    for k, member in ARRAY_KEYS.items():
        m = re.search(r'\b' + re.escape(member) + r'\s*\{([^}]*)\}', body)
        if m:
            arrays[k] = [float(x) for x in m.group(1).split(',') if x.strip()]
    return out, arrays


def _yaml_scalar_in(path, key):
    for line in open(path, encoding='utf-8'):
        line = line.split('#')[0].rstrip()
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        if k.strip() == key:
            try:
                return float(v.strip())
            except ValueError:
                continue
    return None


def _yaml_list_in(path, key):
    for line in open(path, encoding='utf-8'):
        line = line.split('#')[0].rstrip()
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        if k.strip() == key:
            v = v.strip()
            if v.startswith('[') and v.endswith(']'):
                return [float(x) for x in v[1:-1].split(',') if x.strip()]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--mppi-file', default=MPPI_YAML)
    ap.add_argument('--base-file', default=BASE_YAML)
    a = ap.parse_args()

    y, ya = read_yaml()
    c, ca = read_cpp()
    problems = []

    # ---- ① 参数一致 ----
    for k in list(SCALAR_KEYS) + list(ARRAY_KEYS):
        is_scalar = k in SCALAR_KEYS
        yd, cd = (y, c) if is_scalar else (ya, ca)
        if k not in yd or k not in cd:
            problems.append({'check': 'param', 'key': k, 'problem': '缺键',
                             'values': {'yaml': yd.get(k), 'cpp': cd.get(k)}})
            continue
        if is_scalar:
            yv, cv = yd[k], cd[k]
            same = (bool(yv) == bool(cv)) if isinstance(yv, bool) or isinstance(cv, bool) \
                else abs(float(yv) - float(cv)) < 1e-9
        else:
            yv, cv = yd[k], cd[k]
            same = len(yv) == len(cv) and all(abs(p - q) < 1e-9 for p, q in zip(yv, cv))
        if not same:
            problems.append({'check': 'param', 'key': k, 'problem': '取值不一致',
                             'values': {'yaml': yv, 'cpp': cv}})

    # ---- ② 物理自洽 + 单调 ----
    dt_eff = y['speed_limit_anchor_v_mps'] * y['speed_limit_anchor_tan_slope'] / \
        y['speed_limit_anchor_peak_mps2']
    target = y['speed_limit_peak_target_mps2']
    floor = y['speed_limit_floor_mps']
    rows = []
    for deg, v in zip(ya['speed_limit_slope_knots_deg'], ya['speed_limit_slope_knots_mps']):
        peak = v * math.tan(math.radians(deg)) / dt_eff
        rows.append({'kind': 'slope', 'x': deg, 'v': v, 'peak': round(peak, 1),
                     'at_floor': v <= floor + 1e-9, 'ok': v <= floor + 1e-9 or peak <= target + 1e-9})
    grid = y['speed_limit_slope_baseline_m'] / 2.0
    for s, v in zip(ya['speed_limit_step_knots_m'], ya['speed_limit_step_knots_mps']):
        peak = v * (s / grid) / dt_eff
        rows.append({'kind': 'step', 'x': s, 'v': v, 'peak': round(peak, 1),
                     'at_floor': v <= floor + 1e-9, 'ok': v <= floor + 1e-9 or peak <= target + 1e-9})
    for r in rows:
        if not r['ok']:
            problems.append({'check': 'physics', 'key': '%s@%s' % (r['kind'], r['x']),
                             'problem': '预期峰值 %.1f m/s² > target %.1f 且未到地板 %.2f m/s'
                                        % (r['peak'], target, floor),
                             'values': {'v': r['v']}})
    for kx, ky, name in (('speed_limit_slope_knots_deg', 'speed_limit_slope_knots_mps', '坡度表'),
                         ('speed_limit_step_knots_m', 'speed_limit_step_knots_mps', '台阶表')):
        xs, ys = ya[kx], ya[ky]
        if len(xs) != len(ys):
            problems.append({'check': 'monotone', 'key': kx, 'problem': '两数组长度不同',
                             'values': {'x': len(xs), 'y': len(ys)}})
            continue
        for i in range(1, len(xs)):
            if xs[i] <= xs[i - 1] or ys[i] > ys[i - 1] + 1e-12:
                problems.append({'check': 'monotone', 'key': '%s[%d]' % (name, i),
                                 'problem': '必须按 x 递增、v 单调不增',
                                 'values': {'x': xs[i - 1:i + 1], 'v': ys[i - 1:i + 1]}})

    # ---- ③ 与 nav2 两个真源对得上 ----
    mppi_vx = _yaml_scalar_in(a.mppi_file, 'vx_max')
    sm_max = _yaml_list_in(a.base_file, 'max_velocity')
    sm_dec = _yaml_list_in(a.base_file, 'max_decel')
    if mppi_vx is None or sm_max is None:
        problems.append({'check': 'nav2', 'key': 'vx_max / max_velocity',
                         'problem': '读不到 nav2 参数文件',
                         'values': {'mppi': a.mppi_file, 'base': a.base_file}})
    else:
        eff = min(mppi_vx, sm_max[0])
        if abs(eff - y['speed_limit_vx_max']) > 1e-9:
            problems.append({'check': 'nav2', 'key': 'speed_limit_vx_max',
                             'problem': '必须 == min(MPPI vx_max, velocity_smoother.max_velocity[0])',
                             'values': {'yaml': y['speed_limit_vx_max'], 'expected': eff,
                                        'mppi_vx_max': mppi_vx,
                                        'smoother_max_velocity0': sm_max[0]}})
    if sm_dec is not None and y['speed_limit_brake_mps2'] > abs(sm_dec[0]) + 1e-9:
        problems.append({'check': 'nav2', 'key': 'speed_limit_brake_mps2',
                         'problem': '刹车距离界比 velocity_smoother.max_decel 还乐观',
                         'values': {'yaml': y['speed_limit_brake_mps2'],
                                    'smoother_max_decel0': sm_dec[0]}})

    rep = {'yaml': y, 'yaml_arrays': ya, 'cpp': c, 'cpp_arrays': ca,
           'dt_eff_ms': round(dt_eff * 1e3, 3), 'rows': rows, 'problems': problems,
           'files': {'yaml': YAML, 'hpp': HPP, 'mppi': a.mppi_file, 'base': a.base_file}}
    if a.json:
        print(json.dumps(rep, ensure_ascii=False, indent=1))
    else:
        print('[slope_speed] 真源 %s' % os.path.relpath(YAML, WS))
        print('[slope_speed] Δt_eff = %.3f ms（锚点 v=%.2f m/s、tanθ=%.3f ⇒ %.0f m/s² 反算）'
              % (dt_eff * 1e3, y['speed_limit_anchor_v_mps'], y['speed_limit_anchor_tan_slope'],
                 y['speed_limit_anchor_peak_mps2']))
        for k in SCALAR_KEYS:
            bad = any(p['key'] == k and p['check'] == 'param' for p in problems)
            print('  %-36s yaml=%-8s cpp=%-8s %s'
                  % (k, y.get(k), c.get(k), '❌' if bad else 'OK'))
        for k in ARRAY_KEYS:
            bad = any(p['key'] == k and p['check'] == 'param' for p in problems)
            print('  %-36s yaml=%s' % (k, ya.get(k)))
            print('  %-36s cpp =%s %s' % ('', ca.get(k), '❌' if bad else 'OK'))
        print('[slope_speed] 表 ↔ 物理模型（target ≤ %.0f m/s²；到地板 %.2f m/s 的结点豁免）：'
              % (target, floor))
        for r in rows:
            tail = '（= 地板，模型解 %.2f m/s）' % (target * dt_eff / max(
                math.tan(math.radians(r['x'])) if r['kind'] == 'slope' else (r['x'] / grid), 1e-9),) \
                if r['at_floor'] else ''
            print('  %-5s %-7s v=%.2f m/s  预测峰值=%6.1f m/s²  %s %s'
                  % (r['kind'], r['x'], r['v'], r['peak'], 'OK' if r['ok'] else '❌', tail))
        if problems:
            print('\n[slope_speed] ❌ 有 %d 处问题：%s'
                  % (len(problems), json.dumps(problems, ensure_ascii=False)))
        else:
            print('\n[slope_speed] ✅ 参数一致 + 表与物理模型自洽 + 与 nav2 vx_max/max_decel 对得上')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
