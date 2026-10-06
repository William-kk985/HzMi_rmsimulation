#!/usr/bin/env python3
"""把 A/B/C/D 四跑的 err_map(t) / 环秩 cyc(t) 画成一张对照图（docs/img/）。"""
import argparse
import json
import math
import os
import statistics as st

os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl-sltune')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

COL = {'A2': 'tab:blue', 'B': 'tab:red', 'C': 'tab:green', 'D': 'tab:orange'}
LBL = {'A2': 'A2 baseline (chain=10, d~0.80m)', 'B': 'B chain=4 (d~0.80m)',
       'C': 'C denser nodes (d~0.31m)', 'D': 'D chain4+radius4+loose thr (d~0.80m)'}


def load(path):
    poses, graphs = [], []
    for line in open(path):
        try:
            d = json.loads(line)
        except Exception:
            continue
        if d.get('k') == 'pose' and 'gt' in d and 'map' in d:
            poses.append(d)
        elif d.get('k') == 'graph':
            graphs.append(d)
    return poses, graphs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='.tmp_cache/sltune')
    ap.add_argument('--tags', default='A2,B,C,D')
    ap.add_argument('--out', default='docs/img/slam_toolbox_tuning_ab.png')
    args = ap.parse_args()

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for tag in args.tags.split(','):
        p = os.path.join(args.base, tag, 'rec.jsonl')
        if not os.path.isfile(p):
            print('缺 %s' % p)
            continue
        poses, graphs = load(p)
        if not poses:
            continue
        t0 = poses[0]['tw']
        pool = [q for q in poses if q['tw'] - t0 <= 6.0] or poses[:10]
        ox = st.median([q['gt'][0] for q in pool])
        oy = st.median([q['gt'][1] for q in pool])
        T = [q['tw'] - t0 for q in poses]
        E = [math.dist(q['map'][:2], (q['gt'][0] - ox, q['gt'][1] - oy)) for q in poses]
        L = [math.dist(q['odom'][:2], (q['gt'][0] - ox, q['gt'][1] - oy))
             for q in poses if 'odom' in q]
        axes[0].plot(T, E, color=COL.get(tag, 'k'), lw=1.6, label=LBL.get(tag, tag))
        if L:
            axes[0].plot(T[:len(L)], L, color=COL.get(tag, 'k'), lw=0.8, ls=':', alpha=0.6)
        if graphs:
            axes[1].step([g['tw'] - t0 for g in graphs], [g['cyc'] for g in graphs],
                         where='post', color=COL.get(tag, 'k'), lw=1.6,
                         label=LBL.get(tag, tag))
    axes[0].set_ylabel('err_map = ||map->base_link - ground truth||  [m]')
    axes[0].set_title('slam_toolbox mapping: fused pose error (solid) vs raw LIO error (dotted)')
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel('pose-graph cycle rank  E - V + 1')
    axes[1].set_xlabel('wall time [s] (recorder start = 0)')
    axes[1].set_title('cycle rank > 0 = extra cross-time constraint; only with a map->odom jump is it an accepted loop closure')
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print('写出 %s' % args.out)


if __name__ == '__main__':
    main()
