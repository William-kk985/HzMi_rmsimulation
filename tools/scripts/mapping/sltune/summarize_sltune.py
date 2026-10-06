#!/usr/bin/env python3
"""把 A/B/C/D 的分析 JSON 汇成一张对照表（markdown 行），给 §3 直接贴。"""
import argparse
import json
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='.tmp_cache/sltune')
    ap.add_argument('--tags', default='A2,B,C,D')
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    rows = []
    for tag in args.tags.split(','):
        p = os.path.join(args.base, tag, 'analysis.json')
        if not os.path.isfile(p):
            print('缺 %s' % p)
            continue
        d = json.load(open(p))
        jumps = d.get('corr_jumps') or []
        lc = [j for j in jumps if j.get('near_cyc_up')]
        cs = d.get('cyc') or []
        g = cs[-1] if cs else {}
        g = {'cyc': g.get('cyc'), 'V': g.get('V'), 'E': g.get('E')}
        rows.append({
            'tag': tag,
            'err_map_mean': d['err_map_mean'], 'err_map_max': d['err_map_max'],
            'err_map_end': d['err_map_end'],
            'err_lio_mean': d['err_lio_mean'], 'err_lio_max': d['err_lio_max'],
            'n_jumps': len(jumps),
            'max_jump': round(max([j['d'] for j in jumps]), 3) if jumps else 0.0,
            'n_lc': len(lc),
            'lc_times': [j['t'] for j in lc],
            'lc_max': round(max([j['d'] for j in lc]), 3) if lc else 0.0,
            'cyc_end': g.get('cyc'), 'V_end': g.get('V'), 'E_end': g.get('E'),
            'ret': d.get('ret'), 'rtf': d.get('rtf_mean'),
            'cpu': d.get('cpu_mean_pct'), 'gt_path': d.get('gt_path_m'),
            'far': d.get('gt_max_dist_from_start'),
        })
    print('| 跑 | gt 轨迹长 | 离起点最远 | err_map 均值 | err_map 最大 | 末值 | err_lio 均值/最大 | '
          '环秩末值(V/E) | map→odom 跳变数 | 最大跳变 | 疑似回环次数(最大) | RTF |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        print('| **%s** | %.1f m | %.2f m | %.3f | %.3f | %.3f | %.3f / %.3f | %s (%s/%s) | %d | %.3f m | %d (%.3f m) | %s |'
              % (r['tag'], r['gt_path'], r['far'], r['err_map_mean'], r['err_map_max'],
                 r['err_map_end'], r['err_lio_mean'], r['err_lio_max'],
                 r['cyc_end'], r['V_end'], r['E_end'], r['n_jumps'], r['max_jump'],
                 r['n_lc'], r['lc_max'], r['rtf']))
    for r in rows:
        print('  %s: 回到起点最后一次=%s；疑似回环时刻=%s；CPU=%s'
              % (r['tag'], r['ret'], r['lc_times'], r['cpu']))
    if args.json_out:
        json.dump(rows, open(args.json_out, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
