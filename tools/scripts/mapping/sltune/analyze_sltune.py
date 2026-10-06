#!/usr/bin/env python3
"""分析 rec.jsonl（v2）：以「真值」为基准的融合误差 / LIO 误差 / 是否发生图校正 / 回环证据。

关键指标（全部在同一坐标系：map 系 = 出生点相对系；/odom_ground_truth 是世界系，
用第一帧减出生点得到 map 系真值）：
  err_map(t) = ||map→base_link(t) − 真值(t)||   ← 融合估计误差（建图/导航真正在意的量）
  err_lio(t) = ||odom→base_link(t) − 真值(t)||  ← 纯 LIO 误差（对照组）
  返回起点误差 = 机器人真值最后一次回到起点附近时的 err_map（"回到同一地点，估计说自己在哪"）
  校正事件 = map→odom 一步跳变 > 3 cm（slam_toolbox 只有在回环被接受时才会跑 Ceres 优化）
  环秩 cyc = E − V + 1（位姿图里"成环的边"数；顺序链应恒为 0）
"""
import argparse
import json
import math
import statistics as st


def load(path):
    poses, graphs, maps, sysl = [], [], [], []
    for line in open(path):
        try:
            d = json.loads(line)
        except Exception:
            continue
        k = d.get('k')
        if k == 'pose':
            poses.append(d)
        elif k == 'graph':
            graphs.append(d)
        elif k == 'map':
            maps.append(d)
        elif k == 'sys':
            sysl.append(d)
    return poses, graphs, maps, sysl


def yawdiff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rec', required=True)
    ap.add_argument('--tag', default='')
    ap.add_argument('--near', type=float, default=0.60, help='「回到起点」判定半径（m）')
    ap.add_argument('--json')
    args = ap.parse_args()

    poses, graphs, maps, sysl = load(args.rec)
    rows = [p for p in poses if 'gt' in p and 'map' in p and 'odom' in p]
    if len(rows) < 20:
        print('❌ 可用样本太少：%d' % len(rows))
        return 1
    t0 = rows[0]['tw']
    # 出生点（世界系）取前几秒中位数
    pool = [p for p in rows if p['tw'] - t0 <= 6.0] or rows[:10]
    org = [st.median([p['gt'][i] for p in pool]) for i in range(2)]

    def gtm(p):
        return [p['gt'][0] - org[0], p['gt'][1] - org[1], p['gt'][2]]

    def emap(p):
        return math.dist(p['map'][:2], gtm(p)[:2])

    def elio(p):
        return math.dist(p['odom'][:2], gtm(p)[:2])

    def corr(p):
        """map→odom 位置分量（融合相对 LIO 的修正量）"""
        return [p['map'][0] - p['odom'][0], p['map'][1] - p['odom'][1]]

    for p in rows:
        p['_em'] = emap(p)
        p['_el'] = elio(p)
        p['_gt'] = gtm(p)
        p['_corr'] = corr(p)

    near = [p for p in rows if math.dist(p['_gt'][:2], (0, 0)) < args.near]
    ret = near[-1] if near else None
    worst = max(rows, key=lambda p: p['_em'])
    res = {
        'tag': args.tag, 'n': len(rows), 'spawn_world': org,
        'err_map_mean': round(st.mean([p['_em'] for p in rows]), 3),
        'err_map_max': round(worst['_em'], 3),
        'err_map_max_t': round(worst['tw'] - t0, 1),
        'err_map_end': round(rows[-1]['_em'], 3),
        'err_lio_mean': round(st.mean([p['_el'] for p in rows]), 3),
        'err_lio_max': round(max(p['_el'] for p in rows), 3),
        'err_lio_end': round(rows[-1]['_el'], 3),
        'ret': None, 'corr_jumps': [], 'cyc': [],
        'gt_path_m': round(sum(math.dist(a['_gt'][:2], b['_gt'][:2])
                               for a, b in zip(rows, rows[1:])), 2),
        'gt_max_dist_from_start': round(max(math.dist(p['_gt'][:2], (0, 0)) for p in rows), 2),
    }
    if ret is not None:
        # 第一次离开前 2 s 内的基准（近似 t=0 的融合位姿）
        res['ret'] = {'t': round(ret['tw'] - t0, 1), 'err_map': round(ret['_em'], 3),
                      'err_lio': round(ret['_el'], 3),
                      'gt_from_start': round(math.dist(ret['_gt'][:2], (0, 0)), 3)}
    # map→odom 跳变（校正事件）
    for a, b in zip(rows, rows[1:]):
        d = math.dist(a['_corr'], b['_corr'])
        if d > 0.03:
            res['corr_jumps'].append({'t': round(b['tw'] - t0, 1), 'd': round(d, 3),
                                      'err_map_before': round(a['_em'], 3),
                                      'err_map_after': round(b['_em'], 3)})
    for g in graphs:
        res['cyc'].append({'t': round(g['tw'] - t0, 1), 'V': g['V'], 'E': g['E'],
                           'cyc': g['cyc'], 'maxedge': g['maxedge']})
    # 环秩增加的时刻（= 图里多了跨时间约束）
    cyc_up = []
    for a, b in zip(res['cyc'], res['cyc'][1:]):
        if b['cyc'] > a['cyc']:
            cyc_up.append(b['t'])
    # 回环（TryCloseLoop 被接受）的判据：map→odom 跳变 且 附近 ±8 s 内环秩上升
    for j in res['corr_jumps']:
        j['near_cyc_up'] = any(abs(j['t'] - t) <= 8.0 for t in cyc_up)
    res['cyc_up_times'] = cyc_up
    # 打印每 2 s 一行
    print('== %s ==  gt_path=%.1f m  离起点最远=%.2f m  样本=%d' % (
        args.tag, res['gt_path_m'], res['gt_max_dist_from_start'], len(rows)))
    print('   t(s)  gt_dist  err_map  err_lio   map-odom修正(x,y)      cyc')
    last = -99
    for p in rows:
        if p['tw'] - last < 2.0:
            continue
        last = p['tw']
        g = None
        for gg in graphs:
            if gg['tw'] <= p['tw']:
                g = gg
        print('  %6.1f %7.2f %8.3f %8.3f   (%6.3f,%6.3f)  %4s' % (
            p['tw'] - t0, math.dist(p['_gt'][:2], (0, 0)), p['_em'], p['_el'],
            p['_corr'][0], p['_corr'][1], g['cyc'] if g else '-'))
    print('  ↳ err_map: mean=%.3f max=%.3f @%.0fs end=%.3f | err_lio: mean=%.3f max=%.3f end=%.3f' % (
        res['err_map_mean'], res['err_map_max'], res['err_map_max_t'], res['err_map_end'],
        res['err_lio_mean'], res['err_lio_max'], res['err_lio_end']))
    print('  ↳ 回到起点(<%.2f m)最后一次：%s' % (args.near, res['ret']))
    print('  ↳ 环秩上升时刻：%s' % (res['cyc_up_times'] or '无'))
    print('  ↳ map→odom 跳变（>3cm）：%s' % (res['corr_jumps'] or '无'))
    lc = [j for j in res['corr_jumps'] if j.get('near_cyc_up')]
    print('  ↳ 疑似回环（跳变 + 环秩上升同时发生）：%s' % (lc or '无'))
    print('  ↳ 环秩 cyc 序列（变化点）：%s' % (
        [(c['t'], c['V'], c['E'], c['cyc'], c['maxedge']) for c in res['cyc']] or '无'))
    rtf = [s['rtf'] for s in sysl if s.get('rtf')]
    cpu = {}
    for s in sysl:
        for k, v in (s.get('cpu') or {}).items():
            cpu.setdefault(k, []).append(v)
    res['rtf_mean'] = round(st.mean(rtf), 3) if rtf else None
    res['cpu_mean_pct'] = {k: round(st.mean(v), 1) for k, v in sorted(cpu.items())}
    res['map_first_last'] = [({'occ': m['occ'], 'known': m['known']} if m else None)
                             for m in (maps[0] if maps else None, maps[-1] if maps else None)]
    print('  ↳ RTF=%s CPU%%=%s 地图(首/末)=%s' % (res['rtf_mean'], res['cpu_mean_pct'],
                                                res['map_first_last']))
    if args.json:
        json.dump(res, open(args.json, 'w'), ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
