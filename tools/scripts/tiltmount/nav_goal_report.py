#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_goal_report.py —— 把 `nav_goal_forensics.py` 的 forensics.json 读成一张判决表。

用法：
  python3 tools/scripts/tiltmount/nav_goal_report.py .tmp_tiltmount/n1_* [--full]
"""
import argparse
import json
import os
import sys
from collections import Counter


def fmt(v, n=3):
    if v is None:
        return '—'
    if isinstance(v, float):
        return ('%.' + str(n) + 'f') % v
    return str(v)


def load(p):
    if os.path.isdir(p):
        p = os.path.join(p, 'forensics.json')
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def report(path, full=False):
    d = load(path)
    name = os.path.basename(os.path.dirname(path if path.endswith('.json') else
                                           os.path.join(path, 'x')))
    print('=' * 100)
    print('### %s   settle=%s duration=%s goal_wait=%s' %
          (name, d.get('settle_s'), d.get('duration_s'), d.get('goal_wait_s')))
    print('  goal(map系)=%s  pre_map=%s  post_map=%s' %
          (d.get('goal'), d.get('pre_map_pose'), d.get('post_map_pose')))
    print('  pre_truth=%s post_truth=%s' % (d.get('pre_truth'), d.get('post_truth')))
    td = d.get('truth_disp_during_goal') or {}
    print('  **真值位移(发目标后)=%s m, 真值 Δyaw=%s°**；map 位移=%s m；'
          '到目标(map)=%s m；到目标(真值)=%s m' %
          (fmt(td.get('dist_m')), fmt(td.get('dyaw_deg')),
           fmt(d.get('map_disp_during_goal_m')), fmt(d.get('dist_to_goal_after_map_m')),
           fmt(d.get('dist_to_goal_after_truth_m'))))
    trec = d.get('truth_disp_record_window') or {}
    print('  记录窗内真值位移=%s m / Δyaw=%s°（spawn 自转是否停了看这里）' %
          (fmt(trec.get('dist_m')), fmt(trec.get('dyaw_deg'))))
    print('  counts=%s' % json.dumps(d.get('counts', {}), ensure_ascii=False))

    plans = d.get('plans') or []
    print('  --- /plan：%d 条 ---' % len(plans))
    for p in plans[:10]:
        print('    #%-2d t=%-6s n_poses=%-4s len=%-7s first=%s last=%s 末点到目标=%s' %
              (p['i'], p['t'], p['n_poses'], p['len' if False else 'length_m'],
               p.get('first'), p.get('last'), p.get('dt_to_goal')))
    if len(plans) > 10:
        print('    … 共 %d 条；最后一条 t=%s len=%s 末点到目标=%s' %
              (len(plans), plans[-1]['t'], plans[-1]['length_m'], plans[-1]['dt_to_goal']))

    print('  --- action 状态时间线（去重后）---')
    for e in (d.get('events') or [])[:40]:
        if e[1] == 'goal_sent':
            print('    t=%-6s goal_sent %s' % (e[0], json.dumps(e[2], ensure_ascii=False)))
        else:
            print('    t=%-6s %-18s %s' % (e[0], e[1], e[2].get('status')))

    nav = d.get('nav_fb') or []
    if nav:
        rec = [n[2] for n in nav]
        print('  --- NavigateToPose feedback：n=%d，distance_remaining %s→%s，'
              '**number_of_recoveries max=%s** ---' %
              (len(nav), fmt(nav[0][1]), fmt(nav[-1][1]), max(rec)))
    fp = d.get('fp_fb') or []
    if fp:
        print('  --- FollowPath feedback：n=%d，distance_to_goal %s→%s，speed max=%s ---' %
              (len(fp), fmt(fp[0][1]), fmt(fp[-1][1]),
               fmt(max([f[2] for f in fp], default=0.0))))
    else:
        print('  --- FollowPath feedback：**一条都没有**（控制器没接手）---')

    bt = d.get('bt_log') or []
    kinds = Counter((n, c) for (_t, n, _p, c) in bt)
    rec_nodes = [(n, c, k) for (n, c), k in kinds.items()
                 if any(s in n for s in ('Recovery', 'Clear', 'Spin', 'BackUp', 'Wait',
                                         'GoalUpdated', 'RateController'))]
    print('  --- BT log：%d 条；恢复/控制类节点跳变 %d 种 ---' % (len(bt), len(rec_nodes)))
    for n, c, k in sorted(rec_nodes, key=lambda x: -x[2])[:14]:
        print('      %-40s → %-8s x%d' % (n, c, k))

    cs = d.get('cmd_vel_stats') or {}
    for k, v in cs.items():
        print('  --- %s: n=%s after_goal=%s max|vx|=%s max|wz|=%s '
              '(after: vx=%s wz=%s, n_vx>0.01=%s) ---' %
              (k, v['n'], v['n_after_goal'], v['max_abs_vx'], v['max_abs_wz'],
               v['max_abs_vx_after_goal'], v['max_abs_wz_after_goal'],
               v['n_vx_gt_001_after_goal']))

    lg = d.get('local_grid')
    if lg:
        print('  --- 局部代价图(frame=%s %dx%d): %s ---' %
              (lg['frame_id'], lg['w'], lg['h'], json.dumps(lg['counts'], ensure_ascii=False)))

    ga = d.get('grid_analysis') or {}
    print('  --- 全局代价图快照：%s（最后一张 frame=%s）---' %
          (ga.get('snapshots'), ga.get('last_frame')))
    for k, v in (ga.get('goal_cell') or {}).items():
        if 'value' in v:
            print('    [%s] 目标格 value=%-5s cls=%-9s 窗口0.5m=%s 最近≥99=%s m' %
                  (k, v['value'], v['cls'], json.dumps(v['window_0.5m'].get('counts'),
                                                        ensure_ascii=False),
                   fmt(v.get('nearest_lethal_m'))))
        else:
            print('    [%s] %s' % (k, v))
    for k in ('robot_pre', 'robot_post'):
        if k in ga:
            print('    [%s] 车心格 value=%s cls=%s 窗口0.5m=%s' %
                  (k, ga[k]['value'], ga[k].get('cls'),
                   json.dumps(ga[k]['window_0.5m'].get('counts'), ensure_ascii=False)))
    for pc in (ga.get('plan_cost_last_grid') or []):
        print('    [plan#%s] 沿路采样 %s 点：%s' %
              (pc['plan_i'], pc['n_sampled'], json.dumps(pc['classes'], ensure_ascii=False)))
    if full:
        print('  --- timeline（尾 12）---')
        for r in (d.get('timeline') or [])[-12:]:
            print('    %s' % json.dumps(r, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--full', action='store_true')
    a = ap.parse_args()
    for p in a.paths:
        try:
            report(p, a.full)
        except Exception as e:
            print('!! %s: %s' % (p, e))


if __name__ == '__main__':
    main()
