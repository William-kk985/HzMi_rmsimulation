#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""analyze_slope_speed_ab.py —— 从 `run_nav_clearance_ab.sh` 的产物里抽出"前瞻限速 A/B"要看的那几个量。

输入 = 一次跑的产物目录（含 samples.jsonl / summary.json / launch.log）。
输出（默认人读；--json 给机器读）：
  · IMU：全程峰值、>30 / >50 的样本数、逐个"撞击簇"（时间/位置/当时车速/峰值）
  · 定位：`loc_err` max、`lio_step`/`mo_step` max、`map→odom` 停发（mo_stale_runs）
  · 限速：`[slope_speed]` 行数、limit 分布、**不限速时间占比**、why 分布、
          **实测车速 vs 上限的越界量**（`cvx` 是否真的被压在上限之内 —— 验证 nav2 侧真的生效）
  · 结局：goal_status / travel / 驱动时长；余量（借 summary.json 的 dist_moving）
用法：
  python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_on [--json]
  python3 tools/scripts/regress/analyze_slope_speed_ab.py --compare .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import statistics
import sys

RE_LIMIT = re.compile(
    r'\[(\d+\.\d+)\].*\[slope_speed\] limit=([\d.]+) m/s \((\d+)% of ([\d.]+)\)(.*?)\| why=(\S+) '
    r'd=([\d.]+) m.*v_req=([\d.]+)')
RE_CLUSTER = re.compile(r'连续|撞击')


def load_run(d):
    rows = [json.loads(l) for l in open(os.path.join(d, 'samples.jsonl'), encoding='utf-8')]
    summary = json.load(open(os.path.join(d, 'summary.json'), encoding='utf-8'))
    limits = []
    log = os.path.join(d, 'launch.log')
    if os.path.isfile(log):
        for line in open(log, encoding='utf-8', errors='ignore'):
            m = RE_LIMIT.search(line)
            if m:
                limits.append({'wall': float(m.group(1)), 'limit': float(m.group(2)),
                               'pct': int(m.group(3)), 'vx_max': float(m.group(4)),
                               'unlimited': '（不限速）' in m.group(5),
                               'why': m.group(6), 'd': float(m.group(7)),
                               'v_req': float(m.group(8))})
    return rows, summary, limits


def imu_clusters(rows, thr=30.0):
    out, cur = [], None
    for r in rows:
        a = r.get('imu_a', 0.0)
        if a > thr:
            if cur is None:
                cur = {'t0': r['t'], 't1': r['t'], 'n': 1, 'peak': a, 'x': r.get('mx'),
                       'y': r.get('my'), 'gvx': r.get('gvx'), 'cvx': r.get('cvx')}
            else:
                cur['t1'] = r['t']
                cur['n'] += 1
                if a > cur['peak']:
                    cur.update(peak=a, x=r.get('mx'), y=r.get('my'), gvx=r.get('gvx'),
                               cvx=r.get('cvx'))
        elif cur is not None:
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    return out


def join_limit(rows, limits, tol=0.6):
    """把 [slope_speed] 行按 wall_abs 贴到样本上；返回 (配对样本数, 越界统计)。"""
    if not limits:
        return 0, None
    lw = [x['wall'] for x in limits]
    paired, over = 0, []
    for r in rows:
        w = r.get('wall_abs')
        if w is None or r.get('cvx') is None:
            continue
        i = bisect.bisect_left(lw, w)
        cand = [j for j in (i - 1, i) if 0 <= j < len(limits)]
        if not cand:
            continue
        j = min(cand, key=lambda j: abs(limits[j]['wall'] - w))
        if abs(limits[j]['wall'] - w) > tol:
            continue
        paired += 1
        if not limits[j]['unlimited']:
            over.append(abs(r['cvx']) - limits[j]['limit'])
    if not over:
        return paired, None
    over.sort()
    return paired, {'p50': round(over[len(over) // 2], 3), 'p95': round(over[int(len(over) * 0.95)], 3),
                    'max': round(over[-1], 3),
                    'frac_le0': round(sum(1 for x in over if x <= 0.02) / len(over), 3)}


def report(d, js=False):
    rows, summary, limits = load_run(d)
    imu = [r.get('imu_a', 0.0) for r in rows]
    clusters = imu_clusters(rows)
    lm = [x['limit'] for x in limits]
    unlim = sum(1 for x in limits if x['unlimited'])
    paired, over = join_limit(rows, limits)
    moving = [r for r in rows if abs(r.get('gvx', 0.0)) > 0.05]
    sp = sorted(abs(r['gvx']) for r in moving)
    g = summary.get('goal') or [-12.64, -0.31]
    dgoal = [math.hypot(r['mx'] - g[0], r['my'] - g[1]) for r in rows if r.get('mx') is not None]
    out = {
        'dir': d, 'tag': summary.get('tag'), 'n_rows': len(rows),
        'imu': {'max': round(max(imu), 1) if imu else None,
                'n_gt30': sum(1 for x in imu if x > 30), 'n_gt50': sum(1 for x in imu if x > 50),
                'clusters': clusters},
        'loc': {'loc_err_max': summary.get('loc_err', {}).get('max'),
                'lio_step_max': summary.get('step_lio_step', {}).get('max'),
                'mo_step_max': summary.get('step_mo_step', {}).get('max'),
                'mo_stale_runs': len(summary.get('mo_stale_runs', [])),
                'mo_age_max': summary.get('mo_age_max')},
        'governor': {'n_lines': len(limits),
                     'limit_p50': round(statistics.median(lm), 2) if lm else None,
                     'limit_min': round(min(lm), 2) if lm else None,
                     'limit_max': round(max(lm), 2) if lm else None,
                     'frac_unlimited': round(unlim / len(limits), 3) if limits else None,
                     'why': {w: sum(1 for x in limits if x['why'] == w) for w in
                             sorted({x['why'] for x in limits})},
                     'paired_samples': paired, 'applied_over_limit': over},
        'goal': {'status': summary.get('goal_status'), 'travel_gt': round(summary.get('travel_gt', 0), 2),
                 'travel_cmd': round(summary.get('travel_cmd', 0), 2),
                 'contact_events': summary.get('contact_events'),
                 'blocked': len(summary.get('blocked', [])),
                 'closest_to_goal': round(min(dgoal), 2) if dgoal else None},
        'speed': {'n_moving': len(moving),
                  'p50': round(sp[len(sp) // 2], 2) if sp else None,
                  'p90': round(sp[int(len(sp) * 0.9)], 2) if sp else None,
                  'max': round(sp[-1], 2) if sp else None,
                  'sec_below_0p6': round(0.1 * sum(1 for r in moving if abs(r['gvx']) < 0.6), 1)},
        'clearance': summary.get('dist_moving', {}).get('scan_min'),
        'rtf_note': summary.get('counts'),
    }
    if js:
        print(json.dumps(out, ensure_ascii=False, indent=1))
        return out
    print('########## %s（%s） ##########' % (d, out['tag']))
    print('IMU: max=%.1f  >30:%d  >50:%d' % (out['imu']['max'], out['imu']['n_gt30'],
                                             out['imu']['n_gt50']))
    for c in clusters:
        print('  撞击簇 t=%.1f~%.1f n=%d peak=%.1f @map(%.2f,%.2f) 车速=%.2f 指令=%.2f'
              % (c['t0'], c['t1'], c['n'], c['peak'], c['x'] or 0, c['y'] or 0,
                 c['gvx'] or 0, c['cvx'] or 0))
    print('定位: loc_err_max=%s  lio_step_max=%s  mo_step_max=%s  map→odom 停发=%d 次'
          % (out['loc']['loc_err_max'], out['loc']['lio_step_max'], out['loc']['mo_step_max'],
             out['loc']['mo_stale_runs']))
    if limits:
        print('限速: 行数=%d  limit p50=%.2f min=%.2f max=%.2f  不限速占比=%.0f%%  why=%s'
              % (len(limits), out['governor']['limit_p50'], out['governor']['limit_min'],
                 out['governor']['limit_max'], 100 * out['governor']['frac_unlimited'],
                 out['governor']['why']))
        if over:
            print('      实测 |cvx| 与上限之差: p50=%+.2f p95=%+.2f max=%+.2f（≤0.02 视为"压住了"的占比 %.0f%%，配对 %d 样本）'
                  % (over['p50'], over['p95'], over['max'], 100 * over['frac_le0'], paired))
        else:
            print('      （无"已限速"样本可与 cvx 配对）')
    else:
        print('限速: 本次没有 [slope_speed] 行（governor 关闭 或 没装）')
    print('结局: status=%s travel=%.2f m contact=%d blocked=%d 到目标最近=%.2f m'
          % (out['goal']['status'], out['goal']['travel_gt'], out['goal']['contact_events'],
             out['goal']['blocked'], out['goal']['closest_to_goal']))
    print('速度: 运动样本=%d |v| p50=%.2f p90=%.2f max=%.2f，|v|<0.6 时长≈%.1f s'
          % (out['speed']['n_moving'], out['speed']['p50'], out['speed']['p90'],
             out['speed']['max'], out['speed']['sec_below_0p6']))
    sm = out['clearance'] or {}
    print('余量: /scan 最小距离 p50=%s p05=%s min=%s' % (sm.get('p50'), sm.get('p05'), sm.get('min')))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    outs = [report(d, a.json) for d in a.dirs]
    if not a.json and len(outs) == 2:
        x, y = outs
        print('\n########## 对照 ##########')
        print('IMU 峰值        : %-10s → %-10s' % (x['imu']['max'], y['imu']['max']))
        print('IMU>30 样本数   : %-10s → %-10s' % (x['imu']['n_gt30'], y['imu']['n_gt30']))
        cm = (x['imu']['clusters'] or [{}])[0]
        cn = (y['imu']['clusters'] or [{}])[0]
        print('首个撞击簇峰值  : %-10s → %-10s' % (cm.get('peak'), cn.get('peak')))
        print('loc_err max     : %-10s → %-10s' % (x['loc']['loc_err_max'], y['loc']['loc_err_max']))
        print('lio_step max    : %-10s → %-10s' % (x['loc']['lio_step_max'], y['loc']['lio_step_max']))
        print('goal status     : %-10s → %-10s' % (x['goal']['status'], y['goal']['status']))
        print('travel (m)      : %-10s → %-10s' % (x['goal']['travel_gt'], y['goal']['travel_gt']))
        print('|v| p50 / max   : %s/%s → %s/%s' % (x['speed']['p50'], x['speed']['max'],
                                                    y['speed']['p50'], y['speed']['max']))
        print('|v|<0.6 时长(s) : %-10s → %-10s' % (x['speed']['sec_below_0p6'],
                                                    y['speed']['sec_below_0p6']))
        print('contact/blocked : %s/%s → %s/%s' % (x['goal']['contact_events'], x['goal']['blocked'],
                                                    y['goal']['contact_events'], y['goal']['blocked']))
        print('不限速占比      : %-10s → %-10s' % (x['governor']['frac_unlimited'],
                                                    y['governor']['frac_unlimited']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
