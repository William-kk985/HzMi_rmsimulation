#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_prior_ab.py —— **两张 2D 先验候选图的对照**（离线，不需要 ROS / Gazebo）。

它回答 `verify_low_terrain.py` 单图口径**看不出来**的三件事：

  ① 两张图的窗口往往不同（例：`568x318 origin(-24.8,-8.92)` vs `577x326 origin(-25.2,-9.09)`）
     ⇒ 全场地百分比的分母不一样，**不能直接比**。这里把两张图都裁到**公共物理窗口**再算。
  ② 「可行驶斜面被标占用」这个数只看总量，分不清
       · **台阶/边沿旁边的保守膨胀**（安全方向，应该鼓励），与
       · **坡面中段凭空多出来的假障碍**（真回归）。
     这里按"离最近真值边沿格的距离"分桶（≤0.10 m / >0.20 m）。
  ③ 「换图之后哪里变了」：在重叠窗口里逐格做旧值×新值的混淆矩阵，并对翻转格按
     "是否落在真值边沿 0.10 m 内"分类 —— 一眼看出哪些翻转是**修正**、哪些是**新引入的假墙**。

真值口径与 `verify_low_terrain.py` **完全同一套**（`RMUC2026.stl` → 0.05 m"最高可站立面"高度图；
台阶/边沿 = 比 3×3 邻域最低高 > `--edge-threshold`）——两边结论可以直接互相引用。

用法（先看两张图的单图验收，再用本工具做对照）：

    python3 tools/scripts/regress/verify_low_terrain.py --map <A>.yaml --label A
    python3 tools/scripts/regress/verify_low_terrain.py --map <B>.yaml --label B
    python3 tools/scripts/mapping/map_prior_ab.py <A>.yaml <B>.yaml --label-a A --label-b B

    # 换图流程（含回滚）见 docs/map_assets.md §4；当前默认对见该文档 §0

退出码 0 = 跑完（**不等于**结论"谁更好"）；两边的数字都打印，谁好在"结论表"里给。
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
_REGRESS = os.path.join(WS, 'tools', 'scripts', 'regress')
sys.path.insert(0, _REGRESS)
from field_mesh_vs_map import load_map                                  # noqa: E402
from verify_low_terrain import truth_grid, local_slope, neighbours_min, dilate  # noqa: E402

EDGE_THRESHOLD = 0.12      # 与 verify_low_terrain.py 默认值一致
DRIVABLE_SLOPE_DEG = 22.0  # 与 traversability_criteria.yaml 的 drivable_slope_deg 一致（本车能爬 23°）


def grid_of(yaml_path: str, world: str = 'RMUC2026') -> dict:
    """读一张 nav2 先验图 + 按它自己的窗口/res 栅格化真值 ⇒ 一组布尔掩码。"""
    _, img, occ, res, ox, oy = load_map(yaml_path)
    H, W = occ.shape
    known = img != 205
    free = ~occ & known
    rel = truth_grid(occ.shape, res, ox, oy, world)
    has = rel > -90
    hmin = neighbours_min(np.where(has, rel, np.inf), 1)
    edge = has & np.isfinite(hmin) & ((rel - hmin) > EDGE_THRESHOLD)
    slope = local_slope(np.where(has, rel, np.nan), res)
    drivable = has & (rel >= 0.02) & (rel < 0.15) & np.isfinite(slope) & (slope <= DRIVABLE_SLOPE_DEG)
    return dict(yaml=yaml_path, occ=occ, free=free, known=known, rel=rel, has=has,
                edge=edge, drivable=drivable, res=res, ox=ox, oy=oy, shape=(H, W), world=world)


def crop(g: dict, x0: float, y0: float, x1: float, y1: float) -> dict:
    """按**米**裁剪（不是按格）⇒ 两张不同分辨率/原点的图裁到同一块物理区域。

    ⚠ 裁剪后必须**同步更新** `ox/oy/shape`：`flips()` 要用它们把格心映射到另一张图上，
    元数据不跟着裁就会整块错位（这是本工具第一版踩过的坑）。
    """
    H, W = g['shape']
    c0 = max(0, int(round((x0 - g['ox']) / g['res']))); c1 = min(W, int(round((x1 - g['ox']) / g['res'])))
    r1 = H - max(0, int(round((y0 - g['oy']) / g['res'])))
    r0 = H - min(H, int(round((y1 - g['oy']) / g['res'])))
    if c1 <= c0 or r1 <= r0:
        raise SystemExit('[ab] 公共窗口为空：检查两张图的 origin/窗口')
    out = {k: (v[r0:r1, c0:c1] if isinstance(v, np.ndarray) else v) for k, v in g.items()}
    out['shape'] = (r1 - r0, c1 - c0)
    out['ox'] = g['ox'] + c0 * g['res']
    out['oy'] = g['oy'] + (H - r1) * g['res']
    return out


def stats(g: dict, tag: str, quiet: bool = False) -> dict:
    occ, free, known = g['occ'], g['free'], g['known']
    edge, drivable = g['edge'], g['drivable']
    near2, near4 = dilate(occ, k=2), dilate(occ, k=4)          # 占用格 0.10 / 0.20 m 邻域
    edge2, edge4 = dilate(edge, k=2), dilate(edge, k=4)        # 真值边沿 0.10 / 0.20 m 邻域
    d_occ = drivable & occ
    d = int(drivable.sum()); e = int(edge.sum())
    out = dict(
        edge_cells=e, edge_covered=int((edge & near2).sum()),
        edge_cov=100.0 * (edge & near2).sum() / max(1, e),
        drv_cells=d, drv_occ=100.0 * d_occ.sum() / max(1, d),
        drv_occ_near=100.0 * (d_occ & edge2).sum() / max(1, d),
        drv_occ_far=100.0 * (d_occ & ~edge4).sum() / max(1, d),
        occ=int(occ.sum()), free=int(free.sum()), unknown=int((~known).sum()),
        edge_occ_self=int((edge & occ).sum()),
        shape=(g['shape'][1], g['shape'][0]), origin=(g['ox'], g['oy']), res=g['res'])
    if not quiet:
        print('-- %s  %s' % (tag, os.path.basename(g['yaml'])))
        print('   窗口 %dx%d @ %.3f  origin (%.2f, %.2f) | 占用 %d / 自由 %d / 未知 %d'
              % (out['shape'][0], out['shape'][1], g['res'], g['ox'], g['oy'],
                 out['occ'], out['free'], out['unknown']))
        print('   真值：有数据 %d | 台阶/边沿(>%.2f m) %d | 可行驶斜面(≤%.0f°) %d'
              % (int(g['has'].sum()), EDGE_THRESHOLD, e, DRIVABLE_SLOPE_DEG, d))
        print('   全场边沿被表示(0.10 m 内有占用格) = %.2f%% (%d/%d)'
              % (out['edge_cov'], out['edge_covered'], e))
        print('   可行驶斜面被标占用 = %.2f%% (%d/%d)'
              % (out['drv_occ'], int(d_occ.sum()), d))
        print('      其中 离最近真值边沿 ≤0.10 m（安全方向的保守膨胀）= %.2f%%' % out['drv_occ_near'])
        print('      其中 离最近真值边沿 >0.20 m（坡面中段的真假障碍）= %.2f%%' % out['drv_occ_far'])
    return out


def flips(ga: dict, gb: dict) -> dict:
    """重叠窗口内逐格混淆矩阵 + 翻转格按"是否落在真值边沿 0.10 m 内"分类。

    只对**同一 res** 的两张图做（本仓资产都是 0.05 m）；逐格比较时把 A 的格心映射到 B 的格。
    两张图的原点不是整格对齐（例：差 0.4 / 0.17 m）⇒ 真值的栅格化会差半格，
    因此"真值边沿格被占用"这一项**两个基准都算**（以 A 的栅格化 / 以 B 的栅格化），取一致的结论。
    """
    if abs(ga['res'] - gb['res']) > 1e-9:
        print('[ab] ⚠ 两张图分辨率不同（%.3f vs %.3f）⇒ 跳过逐格比较' % (ga['res'], gb['res']))
        return {}
    A, B = ga['occ'], gb['occ']
    Ha, Wa = A.shape; Hb, Wb = B.shape
    edge2b = dilate(gb['edge'], k=2)
    buckets = {}
    rrs, ccs, rbs, cbs = [], [], [], []
    for r in range(Ha):
        y = ga['oy'] + (Ha - 1 - r + 0.5) * ga['res']
        rb = Hb - 1 - int(round((y - gb['oy']) / gb['res']))
        for c in range(Wa):
            x = ga['ox'] + (c + 0.5) * ga['res']
            cb = int(round((x - gb['ox']) / gb['res']))
            if not (0 <= rb < Hb and 0 <= cb < Wb):
                continue
            va = 0 if A[r, c] else (205 if not ga['known'][r, c] else 254)
            vb = 0 if B[rb, cb] else (205 if not gb['known'][rb, cb] else 254)
            b = buckets.setdefault((va, vb), [0, 0, 0])
            b[0] += 1
            if edge2b[rb, cb]:
                b[1] += 1
            if gb['edge'][rb, cb]:
                b[2] += 1
            rrs.append(r); ccs.append(c); rbs.append(rb); cbs.append(cb)
    tot = sum(v[0] for v in buckets.values())
    print('\n=== 重叠窗口逐格（共 %d 格；0=占用 205=未知 254=自由）===' % tot)
    names = {(254, 0): '旧free→新occ', (0, 254): '旧occ→新free', (0, 0): '两图都occ',
             (254, 254): '两图都free', (205, 254): '旧unk→新free', (254, 205): '旧free→新unk',
             (205, 0): '旧unk→新occ', (0, 205): '旧occ→新unk'}
    for k in sorted(buckets, key=lambda k: -buckets[k][0]):
        n, ne2, ne = buckets[k]
        print('   %-14s %8d  %5.2f%%  | 落在新图真值边沿 0.10 m 内 %5.1f%% | 恰是真值边沿格 %5.1f%%'
              % (names.get(k, '%s→%s' % k), n, 100.0 * n / tot, 100.0 * ne2 / n, 100.0 * ne / n))
    A_, B_ = ga['occ'], gb['occ']
    rrs = np.array(rrs); ccs = np.array(ccs); rbs = np.array(rbs); cbs = np.array(cbs)
    out = dict(overlap=tot)
    for tag, emask in (('以 A 的栅格化', ga['edge'][rrs, ccs]), ('以 B 的栅格化', gb['edge'][rbs, cbs])):
        e = int(emask.sum())
        oa = int((emask & A_[rrs, ccs]).sum())      # A_ = occ（True = 占用）
        ob = int((emask & B_[rbs, cbs]).sum())
        print('   %s 的真值边沿格 %d：A 占用 %d (%.1f%%)，B 占用 %d (%.1f%%)'
              % (tag, e, oa, 100.0 * oa / max(1, e), ob, 100.0 * ob / max(1, e)))
        out[tag] = (e, oa, ob)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('a', help='候选 A 的 yaml（通常 = 当前默认 map/<world>.yaml）')
    ap.add_argument('b', help='候选 B 的 yaml')
    ap.add_argument('--label-a', default='A')
    ap.add_argument('--label-b', default='B')
    ap.add_argument('--world', default='RMUC2026')
    a = ap.parse_args()

    ga = grid_of(a.a, a.world); gb = grid_of(a.b, a.world)
    print('=== 各自窗口（原样）===')
    sa = stats(ga, a.label_a); sb = stats(gb, a.label_b)

    def ext(g):
        H, W = g['shape']
        return (g['ox'], g['oy'], g['ox'] + W * g['res'], g['oy'] + H * g['res'])
    ax0, ay0, ax1, ay1 = ext(ga); bx0, by0, bx1, by1 = ext(gb)
    x0, y0, x1, y1 = max(ax0, bx0), max(ay0, by0), min(ax1, bx1), min(ay1, by1)
    print('\n=== 公共物理窗口 x∈[%.2f,%.2f] y∈[%.2f,%.2f]（分母一致，可比）==='
          % (x0, x1, y0, y1))
    ca, cb = crop(ga, x0, y0, x1, y1), crop(gb, x0, y0, x1, y1)
    ta = stats(ca, a.label_a + '(crop)'); tb = stats(cb, a.label_b + '(crop)')
    flips(ca, cb)

    print('\n=== 结论表（公共窗口；"谁好"只对判据明确的指标给）===')
    rows = [('全场边沿被表示 %', 'edge_cov', True),
            ('可行驶斜面被标占用 %', 'drv_occ', False),
            ('  └ 离边沿>0.20m %（真假障碍）', 'drv_occ_far', False),
            ('  └ 离边沿≤0.10m %（保守膨胀）', 'drv_occ_near', None),
            ('未知格数', 'unknown', False),
            ('自由格数', 'free', True),
            ('占用格数', 'occ', None),
            ('真值边沿格本身被占用（各自栅格化）', 'edge_occ_self', True)]
    for name, key, better in rows:
        va, vb = ta[key], tb[key]
        win = ''
        if better is not None and abs(va - vb) > 1e-9:
            win = a.label_a if (va > vb) == better else a.label_b
        print('  %-32s %-14.4g %-14.4g %s' % (name, va, vb, win or '='))
    print('\n提醒：这个场地的 2D 先验有**固有上限**（全场边沿被表示都到不了 80%，见 '
          'docs/traversability_plan.md）；本工具只回答"两张候选谁更好"，不回答"够不够好"。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
