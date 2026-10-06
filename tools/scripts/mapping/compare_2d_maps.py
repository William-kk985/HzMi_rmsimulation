#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compare_2d_maps.py —— 两张 nav2 栅格图（pgm+yaml）的 A/B：逐格对照 + 区域计数 + 对比图。

用来回答"**同一块地方，两张图分别是什么**"这种问题（本仓的用例：`/scan` 累积图 vs
点云投影图，看坡道/台阶那一带）。两个图**分辨率、窗口、原点都可以不同** ——
本工具把它们重采样到共同的栅格上再比（最近邻；只用于对比，不用于导航）。

用法：
    python3 tools/scripts/mapping/compare_2d_maps.py \
      --a src/rm_nav_bringup/map/RMUC2026_spl --b src/rm_nav_bringup/map/RMUC2026_cloud \
      --label-a "scan-accumulated" --label-b "cloud-projected" \
      --box ramp -7.9 -8.4 -3.6 -3.6 --box corridor -9.2 -8.4 -3.6 -7.2 \
      --png .tmp_cache/2d_ab/compare.png --json .tmp_cache/2d_ab/compare.json
退出码 0。像素口径：0=占用 / 205=未知 / 254=空闲（本仓 map_saver 口径）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

FREE, UNK, OCC = 254, 205, 0


def load_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError('%s 不是二值 PGM（P5），是 %r' % (path, magic))
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = (int(t) for t in line.split())
        maxv = int(f.readline())
        if maxv != 255:
            raise ValueError('%s maxval=%d（只支持 255）' % (path, maxv))
        d = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return d, w, h


def load_yaml_min(path):
    """只认本仓这几行（不引入 yaml 依赖）。"""
    o = {'resolution': None, 'origin': None}
    for ln in open(path):
        ln = ln.strip()
        if ln.startswith('resolution:'):
            o['resolution'] = float(ln.split(':', 1)[1])
        elif ln.startswith('origin:'):
            o['origin'] = [float(t) for t in ln.split(':', 1)[1].strip().strip('[]').split(',')]
    return o


def read_map(prefix):
    d, w, h = load_pgm(prefix + '.pgm')
    y = load_yaml_min(prefix + '.yaml')
    res = y['resolution']
    ox, oy = y['origin'][0], y['origin'][1]
    # pgm 第 0 行 = 图的最上面 = y 最大
    return {'data': d, 'w': w, 'h': h, 'res': res, 'ox': ox, 'oy': oy,
            'name': os.path.basename(prefix)}


def classify(a):
    """→ 布尔掩码 (free, unknown, occupied)，形状 = pgm (行, 列)。"""
    d = a['data']
    return d >= 250, (d > 50) & (d < 250), d <= 50


def sample(a, x, y):
    """把 map 系坐标 (x, y) 投到该图的格值（不在图内 → None）。"""
    i = int(np.floor((x - a['ox']) / a['res']))
    j = int(np.floor((a['h'] - 1) - (y - a['oy']) / a['res']))
    if i < 0 or j < 0 or i >= a['w'] or j >= a['h']:
        return None
    return int(a['data'][j, i])


def counts_in_box(a, box, res=0.05):
    x0, y0, x1, y1 = box
    xs = np.arange(x0 + res / 2, x1, res)
    ys = np.arange(y0 + res / 2, y1, res)
    n = f = u = o = outside = 0
    for y in ys:
        for x in xs:
            v = sample(a, x, y)
            n += 1
            if v is None:
                outside += 1
            elif v >= 250:
                f += 1
            elif v > 50:
                u += 1
            else:
                o += 1
    return {'box': [round(v, 3) for v in box], 'cells': n, 'free': f, 'unknown': u,
            'occupied': o, 'outside_map': outside,
            'free_m2': round(f * res * res, 3), 'unknown_m2': round(u * res * res, 3),
            'occupied_m2': round(o * res * res, 3),
            'outside_map_m2': round(outside * res * res, 3)}


def render(A, B, out, boxes, la, lb, pad=0.5, res=0.05):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print('[warn] 没有 matplotlib，跳过 PNG：%s' % e)
        return None
    x0 = min(A['ox'], B['ox']); y0 = min(A['oy'], B['oy'])
    x1 = max(A['ox'] + A['w'] * A['res'], B['ox'] + B['w'] * B['res'])
    y1 = max(A['oy'] + A['h'] * A['res'], B['oy'] + B['h'] * B['res'])
    W = int((x1 - x0) / res); H = int((y1 - y0) / res)
    ext = [x0, x1, y0, y1]

    def to_common(a):
        g = np.full((H, W), UNK, dtype=np.uint8)
        for j in range(H):
            for i in range(W):
                g[j, i] = UNK
        # 向量化：把共同格中心投到 a
        cx = x0 + (np.arange(W) + 0.5) * res
        cy = y0 + (np.arange(H) + 0.5) * res
        II = np.floor((cx - a['ox']) / a['res']).astype(int)
        JJ = (a['h'] - 1) - np.floor((cy - a['oy']) / a['res']).astype(int)
        okx = (II >= 0) & (II < a['w']); oky = (JJ >= 0) & (JJ < a['h'])
        sub = a['data'][np.ix_(JJ[oky], II[okx])]
        g[np.ix_(np.where(oky)[0], np.where(okx)[0])] = sub
        return g

    ga, gb = to_common(A), to_common(B)
    fig, axs = plt.subplots(1, 3, figsize=(24, 8), dpi=100)
    for ax, g, title in ((axs[0], ga, 'A: %s' % la), (axs[1], gb, 'B: %s' % lb)):
        ax.imshow(g, origin='upper', extent=ext, cmap='gray_r', vmin=0, vmax=255,
                  interpolation='nearest')
        ax.set_title(title); ax.set_aspect('equal')
    diff = np.zeros((H, W, 3), dtype=np.uint8)
    diff[...] = 255
    fa, ua, oa = ga >= 250, (ga > 50) & (ga < 250), ga <= 50
    fb, ub, ob = gb >= 250, (gb > 50) & (gb < 250), gb <= 50
    diff[fa & fb] = (200, 200, 200)          # 都是 free = 灰
    diff[oa & ob] = (0, 0, 0)                # 都是占用 = 黑
    diff[ua & ub] = (150, 150, 255)          # 都是未知 = 淡蓝
    diff[oa & fb] = (255, 0, 0)              # A 占用 / B 空闲 = 红
    diff[ob & fa] = (0, 160, 0)              # B 占用 / A 空闲 = 绿
    diff[(fa | oa) & ub] = (255, 200, 0)     # B 未知 / A 有判定 = 橙
    diff[(fb | ob) & ua] = (255, 0, 255)     # A 未知 / B 有判定 = 品红
    axs[2].imshow(diff, origin='upper', extent=ext, interpolation='nearest')
    axs[2].set_title('diff: gray=both free, black=both occ, red=A occ/B free,\n'
                     'green=B occ/A free, orange=B unknown, magenta=A unknown, blue=both unk')
    axs[2].set_aspect('equal')
    for ax in axs:
        for (nm, bx0, by0, bx1, by1) in boxes:
            ax.add_patch(plt.Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fill=False,
                                       edgecolor='lime', lw=1.4))
            ax.text(bx0, by1, nm, color='lime', fontsize=8, va='bottom')
        ax.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(out); plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description='两张 nav2 栅格图 A/B（区域计数 + 对比图）')
    ap.add_argument('--a', required=True, help='A 图前缀（不含扩展名）')
    ap.add_argument('--b', required=True, help='B 图前缀')
    ap.add_argument('--label-a', default='A')
    ap.add_argument('--label-b', default='B')
    ap.add_argument('--box', nargs=5, action='append', default=[], metavar=('NAME', 'X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--png')
    ap.add_argument('--json')
    ap.add_argument('--res', type=float, default=0.05, help='计数用的采样格边长（默认 0.05）')
    args = ap.parse_args()
    A, B = read_map(args.a), read_map(args.b)
    rep = {'a': {'prefix': args.a, 'w': A['w'], 'h': A['h'], 'res': A['res'],
                 'origin': [A['ox'], A['oy']], 'label': args.label_a},
           'b': {'prefix': args.b, 'w': B['w'], 'h': B['h'], 'res': B['res'],
                 'origin': [B['ox'], B['oy']], 'label': args.label_b}}
    for a, k in ((A, 'a'), (B, 'b')):
        f, u, o = classify(a)
        rep[k]['cells'] = {'free': int(f.sum()), 'unknown': int(u.sum()), 'occupied': int(o.sum()),
                           'free_m2': round(float(f.sum()) * a['res'] ** 2, 2),
                           'unknown_m2': round(float(u.sum()) * a['res'] ** 2, 2),
                           'occupied_m2': round(float(o.sum()) * a['res'] ** 2, 2)}
    rep['boxes'] = []
    for b in args.box:
        nm, x0, y0, x1, y1 = b[0], float(b[1]), float(b[2]), float(b[3]), float(b[4])
        rep['boxes'].append({'name': nm,
                             'a': counts_in_box(A, (x0, y0, x1, y1), args.res),
                             'b': counts_in_box(B, (x0, y0, x1, y1), args.res)})
    if args.png:
        rep['png'] = render(A, B, args.png,
                            [(b['name'], b['a']['box'][0], b['a']['box'][1],
                              b['a']['box'][2], b['a']['box'][3]) for b in rep['boxes']],
                            args.label_a, args.label_b, res=args.res)
    for bx in rep['boxes']:
        a_, b_ = bx['a'], bx['b']
        print('[box] %-22s A: free %5d / occ %4d / unk %5d / 图外 %5d | '
              'B: free %5d / occ %4d / unk %5d / 图外 %5d'
              % (bx['name'], a_['free'], a_['occupied'], a_['unknown'], a_['outside_map'],
                 b_['free'], b_['occupied'], b_['unknown'], b_['outside_map']))
    print('CMP2D ' + json.dumps(rep, ensure_ascii=False))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
