#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_traversability_criteria.py —— 断言「可通行性判据」的**单一真源**没有漂移。

三处必须逐键相同（任何一处改了另两处没改 ⇒ 实时代价图与先验图口径不一致，正是缺陷 ③）：

  ① src/rm_nav_bringup/config/traversability_criteria.yaml        ← **真源**（launch 喂给两个节点）
  ② src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/
     low_terrain_classifier.hpp 的 `struct Criteria` 字面默认值（没拿到参数文件时的兜底）
  ③ tools/scripts/mapping/pcd_to_nav2_map.py 的 CRITERIA_FALLBACK（找不到 YAML 时的兜底）

用法：python3 tools/scripts/regress/check_traversability_criteria.py [--json]
退出码 0 = 一致；1 = 有漂移（打印哪个键、哪一侧、两个值）。
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
YAML = os.path.join(WS, 'src', 'rm_nav_bringup', 'config', 'traversability_criteria.yaml')
HPP = os.path.join(WS, 'src', 'rm_perception', 'rm_ground_traversability', 'include',
                   'rm_ground_traversability', 'low_terrain_classifier.hpp')
PY = os.path.join(WS, 'tools', 'scripts', 'mapping', 'pcd_to_nav2_map.py')

# YAML 键 → C++ Criteria 成员名（同名，除 enable/publish_* 之外一一对应）
KEYS = ['step_height_threshold', 'drivable_slope_deg', 'slope_min_height',
        'ground_cell_m', 'fine_cell_m', 'ground_percentile', 'ground_min_points']


def read_yaml_keys(path=YAML):
    out = {}
    for line in open(path):
        line = line.split('#')[0].rstrip()
        if ':' not in line or line.lstrip().startswith('/**'):
            continue
        k, v = line.split(':', 1)
        k, v = k.strip(), v.strip()
        if k in KEYS:
            out[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
    return out


def read_cpp_defaults(path=HPP):
    src = open(path).read()
    body = src[src.index('struct Criteria'):src.index('/// 粗格/细格比')]
    out = {}
    for k in KEYS:
        m = re.search(r'\b' + re.escape(k) + r'\s*\{([^}]*)\}', body)
        if not m:
            continue
        v = m.group(1).strip()
        out[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
    return out


def read_py_fallback(path=PY):
    src = open(path).read()
    body = src[src.index('CRITERIA_FALLBACK'):src.index('def load_criteria')]
    out = {}
    for k in KEYS:
        m = re.search(r"'" + re.escape(k) + r"'\s*:\s*([0-9.eE+-]+)", body)
        if m:
            v = m.group(1)
            out[k] = float(v) if ('.' in v or 'e' in v.lower()) else int(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    y = read_yaml_keys()
    c = read_cpp_defaults()
    p = read_py_fallback()
    problems = []
    for k in KEYS:
        vals = {'yaml': y.get(k), 'cpp': c.get(k), 'py': p.get(k)}
        if None in vals.values():
            problems.append({'key': k, 'problem': '缺键', 'values': vals})
            continue
        if not (abs(float(vals['yaml']) - float(vals['cpp'])) < 1e-9 and
                abs(float(vals['yaml']) - float(vals['py'])) < 1e-9):
            problems.append({'key': k, 'problem': '取值不一致', 'values': vals})
    rep = {'yaml': y, 'cpp_defaults': c, 'py_fallback': p, 'problems': problems,
           'files': {'yaml': YAML, 'hpp': HPP, 'py': PY}}
    if a.json:
        print(json.dumps(rep, ensure_ascii=False, indent=1))
    else:
        print('[criteria] 真源 %s' % os.path.relpath(YAML, WS))
        for k in KEYS:
            print('  %-24s yaml=%-8s cpp=%-8s py=%-8s %s' % (
                k, y.get(k), c.get(k), p.get(k),
                'OK' if not any(x['key'] == k for x in problems) else '❌'))
        if problems:
            print('\n[criteria] ❌ 有 %d 处漂移：%s' % (len(problems), json.dumps(problems, ensure_ascii=False)))
        else:
            print('\n[criteria] ✅ 三处逐键一致（实时链路与离线先验图同一套阈值）')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
