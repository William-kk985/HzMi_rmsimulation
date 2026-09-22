#!/usr/bin/env python3
"""量化"到底是谁把墙格子擦掉的"。

背景（tools/replay_scan_grid.py 已证实）：本链路没有任何"无回波→假想空地"的写入，
`missing_data_ray_length` 空转；擦除只可能来自**打到东西的光束**（起点→命中点）沿途的 miss 写。
于是只剩一个问题：**为什么这些射线的途经格子会压在墙自己的格子上？**

本工具对真 /map 里的墙格子做抽样，逐周期找出"只清不命"的那些射线，把每条射线的
**命中点比该格子远多少** 统计出来，用来区分两种完全不同的机理：
  · 远 0~10cm  ：格子被"同一条墙上的邻居点"的射线擦到（栅格化/斜入射的必然结果）
  · 远 10cm~1m ：格子被"更远处同一条墙的另一段"的射线擦到（斜视/掠射）
  · 远 >1m     ：那一帧该方位**根本没看见墙**（回波丢了/打到别的东西）⇒ 观测不连续
用法: python3 tools/diag_wall_passes.py --bag .tmp_bags/ret4 [--cycles 250] [--cells 120]
"""
import argparse
import importlib.util
import math
import numpy as np


def load_module(path='tools/replay_scan_grid.py'):
    spec = importlib.util.spec_from_file_location('rsg', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='.tmp_bags/ret4')
    ap.add_argument('--cycles', type=int, default=250, help='抽查多少帧（均匀抽样）')
    ap.add_argument('--cells', type=int, default=120, help='每类抽查多少墙格子')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    m = load_module()
    rng = np.random.default_rng(args.seed)

    cur = m.open_bag(args.bag)
    tids = m.topic_ids(cur)
    t_odom, xy, yaw, bl = m.load_poses(cur, tids, 'tf')
    scans = m.load_scans(cur, tids['/scan'], 1)

    # 真 /map 的末态墙格子（世界格 int）
    d = np.load('.tmp_cache/ret4_raw.npz')
    cand = d['cand']
    ever, last = m.real_map_label(args.bag, cand)
    gi = cand[last] if last.sum() else cand
    cy, cx = np.divmod(gi, m.GRID_SIZE[0])
    wall_xy = np.stack([cx * m.RES + m.GRID_ORIGIN[0] + m.RES / 2,
                        cy * m.RES + m.GRID_ORIGIN[1] + m.RES / 2], 1)
    pick = rng.choice(len(wall_xy), size=min(args.cells, len(wall_xy)), replace=False)
    wall_xy = wall_xy[pick]
    # 每个墙格子的"方向 + 距离"（相对当前雷达位置算，逐周期更新）
    step = max(1, len(scans) // args.cycles)
    beyond, angle_off, n_pass, n_hit, n_cyc = [], [], 0, 0, 0
    for si in range(0, len(scans), step):
        s = scans[si]
        pose = m.pose_at(t_odom, xy, yaw, s['t'])
        if pose is None:
            continue
        n_cyc += 1
        pts = m.voxel_filter(s['pts'], m.RES)
        if len(pts) == 0:
            continue
        o = pose[0] + np.array([bl[0], bl[1]])
        wp = m.world_pts(pts, pose) if False else m.world_pts(pts + np.array([bl[0], bl[1]]), pose)
        # 墙格子：距离 + 方位
        dv = wall_xy - o
        dist = np.hypot(dv[:, 0], dv[:, 1])
        ang = np.arctan2(dv[:, 1], dv[:, 0])
        inrange = (dist > 0.3) & (dist < 10.0)
        # 命中点：距离 + 方位
        hd = np.hypot(wp[:, 0] - o[0], wp[:, 1] - o[1])
        ha = np.arctan2(wp[:, 1] - o[1], wp[:, 0] - o[0])
        order = np.argsort(ha)
        ha_s, hd_s = ha[order], hd[order]
        # 对每个墙格子：找同方位(±0.5 格角宽)的回波里最近的那条 → 判定 hit / pass / 未观测
        halfw = np.arctan2(m.RES * 0.5, np.maximum(dist, 0.1))
        lo = np.searchsorted(ha_s, ang - halfw)
        hi = np.searchsorted(ha_s, ang + halfw, side='right')
        for k in range(len(wall_xy)):
            if not inrange[k]:
                continue
            a, b_ = lo[k], hi[k]
            if b_ <= a:
                continue                     # 该方位这一帧没有回波 ⇒ "扫不到"（不改值，但也没有命中）
            seg = hd_s[a:b_]
            near = seg.min()
            if near <= dist[k] + m.RES * 0.5:
                n_hit += 1                    # 命中（或命中点就在本格）
            else:
                n_pass += 1                   # 该方位有回波，但比本格子远 ⇒ 射线压过本格
                beyond.append(near - dist[k])
                angle_off.append(abs(np.degrees(halfw[k])))
    beyond = np.array(beyond)
    print(f'[diag-wall] 抽查 {n_cyc} 帧 × {len(wall_xy)} 个真 /map 墙格子：'
          f'命中 {n_hit} 次，被"更远回波"压过 {n_pass} 次（比 {n_pass / max(n_hit, 1):.2f}:1）')
    if len(beyond) == 0:
        return
    print(f'  压过的回波比本格子远多少：中位 {np.median(beyond) * 100:.1f}cm  '
          f'P10 {np.percentile(beyond, 10) * 100:.1f}cm  P90 {np.percentile(beyond, 90) * 100:.1f}cm')
    edges = [0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 1e9]
    names = ['0~5cm(同墙邻格)', '5~10cm(同墙邻格)', '10~20cm', '20~50cm(同墙另一段)',
             '0.5~1m(掠射更远段)', '1~2m(该方位没看见墙)', '>2m(打到别的东西)']
    tot = len(beyond)
    for e0, e1, nm in zip(edges[:-1], edges[1:], names):
        c = int(((beyond >= e0) & (beyond < e1)).sum())
        print(f'   {nm:22s} {c:7d}  {100.0 * c / tot:5.1f}%')


if __name__ == '__main__':
    main()
