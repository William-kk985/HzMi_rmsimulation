#!/usr/bin/env python3
"""spl_ab_table.py —— 把 `.tmp_ab/<tag>/` 的取证结果汇总成「逐跑 + 每臂中位数(区间)」表。

只读文件，不碰栈。臂（arm）由 meta.txt 的 `variant` 与 `launch_args` 里的
`gravity_align_mode` 推出：`before+off` / `after+off` / `after+hybrid`（没有该参数时记 `off`）。

用法：
  python3 tools/scripts/tiltmount/spl_ab_table.py --root .tmp_ab ab_v0_r1 ab_v0_r2 ab_v1_r1 ...
  python3 tools/scripts/tiltmount/spl_ab_table.py --root .tmp_ab --prefix ab_        # 自动找

输出：① 逐跑表（markdown）；② 每臂 median[min–max]；③ 臂间差值（相对 before+off）。
"""
import argparse
import glob
import json
import os
import re
import statistics as st


def read_meta(d):
    m = {}
    f = os.path.join(d, 'meta.txt')
    if os.path.isfile(f):
        for line in open(f, encoding='utf-8', errors='replace'):
            if '=' in line:
                k, v = line.rstrip('\n').split('=', 1)
                m[k] = v
    return m


def load(d):
    r = {'tag': os.path.basename(d.rstrip('/')), 'dir': d}
    r['meta'] = read_meta(d)
    args = r['meta'].get('launch_args', '')
    mm = re.search(r'gravity_align_mode:=(\w+)', args)
    r['align'] = mm.group(1) if mm else 'off'
    r['variant'] = r['meta'].get('variant', '?')
    r['arm'] = '%s+%s' % (r['variant'], r['align'])
    for name in ('proc', 'probe', 'points', 'navband'):
        f = os.path.join(d, name + '.json')
        r[name] = json.load(open(f)) if os.path.isfile(f) else None
    p = r['proc'] or {}
    w = p.get('window') or {}
    procs = w.get('procs') or {}
    lio = procs.get('small_point_lio') or procs.get('pointlio') or {}
    ada = procs.get('lio_tf_adapter') or {}
    r['ms_frame'] = lio.get('ms_per_frame')
    r['cpu_pct'] = lio.get('cpu_pct')
    r['whole_ms'] = lio.get('whole_run_ms_per_frame')
    r['hwm'] = lio.get('hwm_mb')
    r['rtf'] = w.get('rtf')
    r['cloud_hz'] = w.get('cloud_hz')
    r['odom_hz'] = w.get('odom_hz')
    r['n_cloud'] = w.get('n_cloud')
    r['adapter_cpu'] = ada.get('cpu_pct')
    r['adapter_ms'] = ada.get('ms_per_frame')
    r['init_sim'] = (p.get('init') or {}).get('sim_s_from_clock0')
    r['init_wall'] = (p.get('init') or {}).get('wall_s_from_launch')
    gz = procs.get('gzserver') or {}
    r['gz_cpu'] = gz.get('cpu_pct')
    r['cpu_per_sim_s'] = lio.get('cpu_per_sim_s')
    r['lio_over_gz'] = (lio.get('cpu_pct') / gz.get('cpu_pct')
                        if (lio.get('cpu_pct') and gz.get('cpu_pct')) else None)
    mm = re.search(r'_r(\d+)$', r['tag'])
    r['cycle'] = int(mm.group(1)) if mm else None
    g = r['probe'] or {}
    r['residual'] = g.get('residual_deg')
    r['odom_rpy'] = (g.get('odom') or {}).get('rpy_deg')
    r['odom_xyz'] = (g.get('odom') or {}).get('xyz')
    r['scan_finite'] = (g.get('scan') or {}).get('n_finite')
    r['scan_span'] = (g.get('scan') or {}).get('range_span')
    r['odom_vs_gt'] = g.get('odom_vs_gt_xy_m')
    pp = r['points'] or {}
    r['raw_pts'] = (pp.get('raw_points_per_frame') or {}).get('median')
    r['reg_pts'] = (pp.get('cloud_registered_points_per_frame') or {}).get('median')
    r['ekf_pts'] = ((pp.get('ekf_points_per_frame') or {}).get('pfn1') or {}).get('median')
    nb = r['navband'] or {}
    r['band'] = nb
    return r


def fmt(v, nd=2):
    if v is None:
        return '—'
    if isinstance(v, (int,)):
        return str(v)
    if isinstance(v, float):
        return ('%.*f' % (nd, v))
    return str(v)


def agg(vals):
    vals = [v for v in vals if isinstance(v, (int, float))]
    if not vals:
        return '—'
    if len(vals) == 1:
        return fmt(vals[0])
    return '%s [%s–%s]' % (fmt(st.median(vals)), fmt(min(vals)), fmt(max(vals)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='.tmp_ab')
    ap.add_argument('--prefix', default=None, help='自动收集 <root>/<prefix>*')
    ap.add_argument('tags', nargs='*')
    args = ap.parse_args()
    tags = list(args.tags)
    if args.prefix:
        tags += sorted(os.path.basename(p) for p in glob.glob(os.path.join(args.root, args.prefix + '*'))
                       if os.path.isdir(p))
    rows = [load(os.path.join(args.root, t)) for t in tags]
    rows = [r for r in rows if r['proc'] or r['probe'] or r['navband']]
    if not rows:
        print('(no data)')
        return 1

    print('### 逐跑\n')
    hdr = ['tag', 'variant', 'align', 'ms/帧', '%CPU', 'CPU/sim s', 'LIO/gz', '全程 ms/帧',
           'VmHWM MB', 'RTF', '/cloud Hz', 'init 仿真s/墙钟s', 'adapter %CPU', '残余倾角°',
           'odom rpy° (roll)', 'odom z m', 'EKF 点/帧', 'raw 点/帧', '/scan 有限']
    print('| ' + ' | '.join(hdr) + ' |')
    print('|' + '---|' * len(hdr))
    for r in rows:
        cells = [r['tag'], r['variant'], r['align'],
                 fmt(r['ms_frame']), fmt(r['cpu_pct']), fmt(r['cpu_per_sim_s'], 4),
                 fmt(r['lio_over_gz'], 4), fmt(r['whole_ms']), fmt(r['hwm']),
                 fmt(r['rtf'], 3), fmt(r['cloud_hz']),
                 '%s/%s' % (fmt(r['init_sim']), fmt(r['init_wall'])),
                 fmt(r['adapter_cpu']), fmt(r['residual']),
                 fmt((r['odom_rpy'] or [None])[0]),
                 fmt((r['odom_xyz'] or [None, None, None])[2]),
                 fmt(r['ekf_pts'], 0), fmt(r['raw_pts'], 0), fmt(r['scan_finite'], 0)]
        print('| ' + ' | '.join(cells) + ' |')

    arms = []
    for r in rows:
        if r['arm'] not in arms:
            arms.append(r['arm'])
    print('\n### 每臂（median[min–max]）\n')
    hdr = ['臂', 'n', 'ms/帧', '%CPU', 'CPU/sim s', 'LIO/gz', '全程 ms/帧', 'VmHWM MB', 'RTF',
           'adapter %CPU', '残余倾角°', 'EKF 点/帧', '/scan 有限']
    print('| ' + ' | '.join(hdr) + ' |')
    print('|' + '---|' * len(hdr))
    for a in arms:
        sub = [r for r in rows if r['arm'] == a]
        cells = [a, str(len(sub)), agg([r['ms_frame'] for r in sub]),
                 agg([r['cpu_pct'] for r in sub]), agg([r['cpu_per_sim_s'] for r in sub]),
                 agg([r['lio_over_gz'] for r in sub]), agg([r['whole_ms'] for r in sub]),
                 agg([r['hwm'] for r in sub]), agg([r['rtf'] for r in sub]),
                 agg([r['adapter_cpu'] for r in sub]), agg([r['residual'] for r in sub]),
                 agg([r['ekf_pts'] for r in sub]), agg([r['scan_finite'] for r in sub])]
        print('| ' + ' | '.join(cells) + ' |')

    base = next((a for a in arms if a.startswith('before')), arms[0])
    bsub = [r for r in rows if r['arm'] == base]
    # 逐 cycle 配对（同一轮里 before 与 after 相邻 ⇒ 抵消机器漂移）
    print('\n### 逐 cycle 配对（同 cycle 内相减）\n')
    print('| cycle | arm | ms/帧 | Δ vs ' + base + ' |')
    print('|---|---|---|---|')
    by_cycle = {}
    for r in rows:
        if r['cycle'] is None:
            continue
        by_cycle.setdefault(r['cycle'], {})[r['arm']] = r['ms_frame']
    for cyc in sorted(by_cycle):
        b = by_cycle[cyc].get(base)
        for a in arms:
            v = by_cycle[cyc].get(a)
            d = (v - b) if (v is not None and b is not None) else None
            print('| %d | %s | %s | %s |' % (cyc, a, fmt(v), ('%+.2f' % d) if d is not None else '—'))

    print('\n### 相对 `%s` 的差（中位数）\n' % base)
    hdr = ['臂', 'Δms/帧', 'Δ%CPU', 'ΔCPU/sim s', 'ΔVmHWM MB']
    print('| ' + ' | '.join(hdr) + ' |')
    print('|' + '---|' * len(hdr))
    bms = [r['ms_frame'] for r in bsub if r['ms_frame'] is not None]
    bpc = [r['cpu_pct'] for r in bsub if r['cpu_pct'] is not None]
    bcs = [r['cpu_per_sim_s'] for r in bsub if r['cpu_per_sim_s'] is not None]
    bhw = [r['hwm'] for r in bsub if r['hwm'] is not None]
    for a in arms:
        sub = [r for r in rows if r['arm'] == a]
        ms = [r['ms_frame'] for r in sub if r['ms_frame'] is not None]
        pc = [r['cpu_pct'] for r in sub if r['cpu_pct'] is not None]
        cs = [r['cpu_per_sim_s'] for r in sub if r['cpu_per_sim_s'] is not None]
        hw = [r['hwm'] for r in sub if r['hwm'] is not None]
        if not (ms and bms):
            continue
        cells = [a, '%+.2f' % (st.median(ms) - st.median(bms)),
                 ('%+.2f' % (st.median(pc) - st.median(bpc))) if (pc and bpc) else '—',
                 ('%+.4f' % (st.median(cs) - st.median(bcs))) if (cs and bcs) else '—',
                 ('%+.2f' % (st.median(hw) - st.median(bhw))) if (hw and bhw) else '—']
        print('| ' + ' | '.join(cells) + ' |')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
