#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_quality_45.py —— §45 的地图质量闸：bbox/覆盖 + **时间一致性** + **围栏厚度/层数**。

为什么要有它：§43 说 PCD 的围栏被"叠成多层"（结构 2D 残差 0.160 m），但那是**对一个位姿**
的最近邻残差；本脚本回答的是**资产级**的两个问题：

 ① **时间一致性**（"同一朵扫描在不同时刻落点一样吗"）：对每一帧 i，用它自己的位姿把点搬到 map，
    与"**排除 i ± exclude 窗口**后累积的图"做最近邻；中位/p90 越小 ⇒ 图越不自相矛盾。
    （这就是"鬼影"的定量定义；GT 图应 ≈ 扫描噪声，坏图会到 0.2 m 量级。）
 ② **围栏厚度/层数**：围栏在 map 系是 4 个竖直平面（x=±…、y=±…）。取每面墙 ±band 内的点，
    沿**法向**做直方图（1 cm 桶）：单层墙 = 一个窄峰；鬼影 = 多个峰/一个被拉宽的峰。
    另给"在围栏 3 根横杆高度带里的点"的 z 直方图（层数 = 独立峰个数）。

用法：
  python3 tools/scripts/mapping/map_quality_45.py --pcd PCD/RMUC2027.pcd \
      --bag third_party/eval_data/capture_map2027/bag/raw --poses .tmp_45/gt/poses_gt.jsonl
  # --poses 是 accumulate_pcd_gt.py 的 --poses-out（逐帧 T_map_base），时间一致性用它；
  # 不给就只能做静态 bbox/厚度。
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))


def _pcd_stats():
    path = os.path.join(_HERE, 'pcd_stats.py')
    spec = importlib.util.spec_from_file_location('pcd_stats_mod', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def read_clouds(bag):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != '/cloud_registered':
            continue
        m = deserialize_message(data, PointCloud2)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pts = np.asarray(pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True),
                         dtype=np.float64)
        out.append((st, pts.reshape(-1, 3)))
    return sorted(out, key=lambda x: x[0])


def voxel(pts, leaf=0.10):
    """质心体素（与累积器同口径）。"""
    if len(pts) == 0:
        return np.zeros((0, 3))
    keys = np.floor(pts / leaf).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    out = np.zeros((len(uniq), 3))
    cnt = np.bincount(inv, minlength=len(uniq))
    for d in range(3):
        out[:, d] = np.bincount(inv, weights=pts[:, d], minlength=len(uniq))
    return out / cnt[:, None]


def temporal_residual(poses, clouds, leaf=0.05, exclude=8.0, max_frames=40):
    """leave-time-out 最近邻：每帧对它 ±exclude 秒之外的累积图。返回逐帧 (t, med, p90)。"""
    from scipy.spatial import cKDTree
    Ts = {}
    for row in poses:
        key = round(row['stamp'], 6)
        Ts[key] = np.array(row.get('T_map_cloud') or row['T_map_base']).reshape(4, 4)
    world = []
    for st, pts in clouds:
        key = min(Ts, key=lambda k: abs(k - st))
        if abs(key - st) > 0.11:
            continue
        T = Ts[key]
        p = pts @ T[:3, :3].T + T[:3, 3]
        world.append((st, p))
    sel = list(range(0, len(world), max(1, len(world) // max_frames)))
    res = []
    for i in sel:
        st, p = world[i]
        far = np.vstack([q for st2, q in world if abs(st2 - st) > exclude])
        if len(far) < 1000:
            continue
        base = voxel(far, leaf)
        tree = cKDTree(base)
        d, _ = tree.query(p)
        res.append((st, float(np.median(d)), float(np.percentile(d, 90))))
    return np.array(res) if res else np.zeros((0, 3))


def fence_thickness(xyz, planes, band=1.2):
    """对每面围墙，取 ±band 内的点，沿法向做 1 cm 直方图；返回峰位、FWHM、多峰。"""
    out = []
    for name, axis, pos in planes:
        oth = 0 if axis != 0 else 1
        sel = np.abs(xyz[:, axis] - pos) < band
        if sel.sum() < 50:
            out.append({'plane': name, 'n': int(sel.sum()), 'thickness': None})
            continue
        v = xyz[sel, axis]
        # 以 1 cm 桶做直方图，找主峰 ±0.3 m 内的 FWHM
        bins = np.arange(pos - band, pos + band + 0.01, 0.01)
        h, edges = np.histogram(v, bins=bins)
        k = int(np.argmax(h))
        peak = 0.5 * (edges[k] + edges[k + 1])
        half = h[k] / 2.0
        lo = k
        while lo > 0 and h[lo - 1] > half:
            lo -= 1
        hi = k
        while hi < len(h) - 1 and h[hi + 1] > half:
            hi += 1
        fwhm = edges[hi + 1] - edges[lo]
        # 多峰：主峰之外、高度 > 25% 主峰且相距 > 10 cm 的局部峰个数
        loc = [j for j in range(1, len(h) - 1) if h[j] >= h[j - 1] and h[j] >= h[j + 1] and h[j] > 0.25 * h[k]]
        far = [j for j in loc if abs(0.5 * (edges[j] + edges[j + 1]) - peak) > 0.10]
        out.append({'plane': name, 'n': int(sel.sum()), 'peak_offset_cm': float((peak - pos) * 100),
                    'thickness_fwhm_m': float(fwhm), 'n_peaks': int(len(loc)),
                    'n_far_peaks': int(len(far))})
    return out


def rail_layers(xyz, planes, band=1.5, zlo=0.0, zhi=1.3):
    """围栏横杆高度带内的 z 直方图（2 cm 桶），报告峰个数与每个峰的点数。"""
    m = np.zeros(len(xyz), dtype=bool)
    for name, axis, pos in planes:
        m |= np.abs(xyz[:, axis] - pos) < band
    z = xyz[m, 2]
    z = z[(z >= zlo) & (z <= zhi)]
    if len(z) < 100:
        return {'n': int(len(z)), 'layers': []}
    edges = np.arange(zlo, zhi + 0.02, 0.02)
    h, edges = np.histogram(z, bins=edges)
    peaks = [j for j in range(1, len(h) - 1) if h[j] >= h[j - 1] and h[j] >= h[j + 1]
             and h[j] > max(5, 0.15 * h.max())]
    return {'n': int(len(z)), 'layers': [
        {'z': float(0.5 * (edges[j] + edges[j + 1])), 'n': int(h[j])} for j in peaks]}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--bag', default='')
    ap.add_argument('--poses', default='')
    ap.add_argument('--planes', nargs='+', default=['x:8.08', 'x:-19.88', 'y:6.40', 'y:-8.55'],
                    help='围栏平面 name:coord，name 以 x/y 开头决定法向')
    ap.add_argument('--exclude', type=float, default=8.0)
    ap.add_argument('--json', default='')
    ap.add_argument('--label', default='')
    a = ap.parse_args(argv)
    ps = _pcd_stats()
    xyz, _hdr = ps.read_pcd_xyz(a.pcd)
    xyz = np.asarray(xyz, dtype=np.float64)
    rep = {'label': a.label or os.path.basename(a.pcd), 'pcd': a.pcd, 'points': int(len(xyz)),
           'bbox': [[float(v) for v in xyz.min(0)], [float(v) for v in xyz.max(0)]],
           'span': [float(v) for v in (xyz.max(0) - xyz.min(0))]}
    print('[q] %s  %d pts  span=%s' % (rep['label'], len(xyz), np.round(rep['span'], 2).tolist()))
    planes = []
    for s in a.planes:
        k, _, v = s.partition(':')
        planes.append((s, 0 if k.startswith('x') else 1, float(v)))
    rep['fence'] = fence_thickness(xyz, planes)
    for f in rep['fence']:
        print('[q] fence %-10s n=%-6s peak=%+.1f cm  FWHM=%s m  peaks=%s/%s' % (
            f['plane'], f['n'], f.get('peak_offset_cm', float('nan')),
            ('%.3f' % f['thickness_fwhm_m']) if f.get('thickness_fwhm_m') else '—',
            f.get('n_peaks'), f.get('n_far_peaks')))
    rep['rails'] = rail_layers(xyz, planes)
    print('[q] rail z-layers: %s' % [(round(l['z'], 2), l['n']) for l in rep['rails']['layers']])
    if a.bag and a.poses:
        clouds = read_clouds(a.bag)
        poses = [json.loads(l) for l in open(a.poses)]
        tr = temporal_residual(poses, clouds, exclude=a.exclude)
        if len(tr):
            rep['temporal'] = {'n': int(len(tr)), 'median': float(np.median(tr[:, 1])),
                               'p90': float(np.percentile(tr[:, 2], 90)),
                               'max': float(tr[:, 2].max()),
                               'frac_med_lt_5cm': float((tr[:, 1] < 0.05).mean())}
            print('[q] temporal residual (leave-time-out NN): median %.3f  p90 %.3f  max %.3f  '
                  'frames<5cm %.0f%%' % (rep['temporal']['median'], rep['temporal']['p90'],
                                         rep['temporal']['max'],
                                         100 * rep['temporal']['frac_med_lt_5cm']))
            rep['temporal_per_frame'] = tr.tolist()
    if a.json:
        json.dump(rep, open(a.json, 'w'), indent=2)
        print('[q] json → %s' % a.json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
