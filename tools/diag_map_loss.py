#!/usr/bin/env python3
"""区分"墙格子消失"的两种完全不同的去向 —— 这决定了该怎么修。

把 bag 里 /map 整段读进来，按世界格跟踪每个格子：
  · 曾占据（某帧 >=65）的格子，到最后**去哪了**：
      - 仍 >=65           → 留住
      - 落到 0~30 (free)  → **被清除**（cartographer 的 miss 写把它擦成空地）⇒ 调 hit/miss/窗口
      - 落到 31~64 (mid)  → 只是证据不足、变淡 ⇒ 调命中率/子图窗口
      - 变成 -1 (unknown)→ **没有任何子图再覆盖它**（子图被移动/被替换/被裁剪）⇒ 调子图与位姿图，不是清除！
  · 以及"曾占据的格子中仍占据的比例"随时间的曲线：平的就说明是"局部闪烁"，一路下滑说明是**累积性丢失**。

为什么必须分这两种：unknown 不是清除，`hit_probability`/`miss_probability` 完全管不到它；
而"运行久了就没了"这种**累积性**症状，最像的就是子图（submap）搬家/换班导致覆盖丢失。
用法: python3 tools/diag_map_loss.py --bag .tmp_bags/ret4 [--occ 65]
"""
import argparse
import glob
import math
import sqlite3

import numpy as np

RES = 0.05
ORIGIN = (-8.0, -8.0)
SIZE = (700, 700)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--occ', type=int, default=65)
    ap.add_argument('--free', type=int, default=30)
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path = args.bag if args.bag.endswith('.db3') else sorted(glob.glob(args.bag + '/*.db3'))[0]
    db = sqlite3.connect(path)
    tid = {n: i for i, n in db.execute('select id, name from topics')}['/map']
    T = get_message('nav_msgs/msg/OccupancyGrid')

    N = SIZE[0] * SIZE[1]
    ever = np.zeros(N, bool)          # 曾经 >=occ
    last = np.full(N, -2, np.int8)    # 最后一次出现时的值（-2 = 该格从未出现在任何快照里）
    curve = []
    n_snap = 0
    rows = db.execute('select data from messages where topic_id=? order by id', (tid,)).fetchall()
    keep_hist = []
    for raw in rows:
        m = deserialize_message(bytes(raw[0]), T)
        d = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        o = m.info.origin.position
        iy, ix = np.meshgrid(np.arange(m.info.height), np.arange(m.info.width), indexing='ij')
        wx = o.x + (ix + 0.5) * m.info.resolution
        wy = o.y + (iy + 0.5) * m.info.resolution
        gx = np.floor((wx - ORIGIN[0]) / RES).astype(np.int64)
        gy = np.floor((wy - ORIGIN[1]) / RES).astype(np.int64)
        ok = (gx >= 0) & (gx < SIZE[0]) & (gy >= 0) & (gy < SIZE[1])
        flat = (gy * SIZE[0] + gx)[ok]
        val = d[ok]
        last[flat] = val                                  # 后出现的快照覆盖前面的（与画图顺序一致）
        o_mask = val >= args.occ
        if o_mask.any():
            ever[flat[o_mask]] = True
        n_snap += 1
        if ever.any():
            present = last[ever] != -2
            still = (last[ever] >= args.occ) & present
            keep_hist.append((n_snap, int(ever.sum()), 100.0 * still.sum() / max(int(present.sum()), 1)))

    n_ever = int(ever.sum())
    lv = last[ever]
    kept = int((lv >= args.occ).sum())
    free = int(((lv >= 0) & (lv <= args.free)).sum())
    mid = int(((lv > args.free) & (lv < args.occ)).sum())
    unk = int((lv == -1).sum())
    gone = int((lv == -2).sum())
    print(f'[map-loss] {args.bag}  快照 {n_snap} 帧；曾占据（>={args.occ}）格子 {n_ever}')
    print(f'  末态仍占据 {kept} ({100.0 * kept / n_ever:.1f}%)')
    print(f'  ── 消失的去向（这是关键）:')
    print(f'     被清成自由 0~{args.free}   : {free:6d}  {100.0 * free / n_ever:5.1f}%   ← 清除（hit/miss/窗口 能管）')
    print(f'     淡成中间 {args.free + 1}~{args.occ - 1} : {mid:6d}  {100.0 * mid / n_ever:5.1f}%   ← 证据不足（命中率/窗口）')
    print(f'     变成 unknown -1     : {unk:6d}  {100.0 * unk / n_ever:5.1f}%   ← **没有任何子图再覆盖它**（子图搬家/换班/裁剪）')
    print(f'     最后快照里根本不存在 : {gone:6d}  {100.0 * gone / n_ever:5.1f}%   ← 地图范围收缩（同样不是清除）')
    print('  ── 曾占据格子中"当帧仍占据"的比例随时间：')
    step = max(1, len(keep_hist) // 12)
    for i, (n, ne, pct) in enumerate(keep_hist):
        if i % step == 0 or i == len(keep_hist) - 1:
            print(f'     快照#{n:4d}  曾占据 {ne:5d}  仍占据 {pct:5.1f}%')
    # 抖动：同一个格子 occupied->非occupied 的翻转次数（区分"平"与"一路下滑"）
    print('  判读：右边那列如果**一路下滑** = 累积性丢失（看 unknown/不存在 两项）；'
          '如果**基本平** = 只是闪烁（看 清除/中间 两项）。')


if __name__ == '__main__':
    main()
