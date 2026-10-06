#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcd_bbox_health.py —— 3D 先验（PCD）落盘后的**独立体检**：bbox / z 跨度 / 告警。

2026-10-06 事故后新增：`map_archive.sh save` 在写完之后用它再量一次（与 cloud_accumulator 节点
自己的写前体检**互相独立**：节点在另一个终端的日志里，而这条会出现在"存档"这一步的输出里）。

判定（阈值与节点同口径，可用参数改）：
  z 跨度 > `--z-span-warn`（默认 3.0 m）⇒ 响亮 WARNING。
  实测依据：健康累积云 z 跨度 1.96~2.02 m（PCD/RMUC2026_mapped.pcd 1,450,064 点全场地 /
  用户实测 z[-0.25,1.77]）；被 LIO 退化污染的两朵云分别是 27.41 m（PCD/RMUC2026_cont.pcd）
  与 8.00 m（合成测试云）⇒ 3.0 m 落在两侧都 ≥1.5 倍间隔的位置。

退出码：0 = 体检通过（或文件读不了 ⇒ 只提示，不挡流程）；3 = **发现不合理**（调用方据此打 WARNING）。
用法：python3 tools/scripts/mapping/pcd_bbox_health.py <pcd> [--z-span-warn 3.0] [--xy-span-warn 40.0] [--json]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys


def _load_reader(ws):
    """复用 cloud_accumulator 自己的 PCD 读取器（x/y/z，binary/ascii 都支持）。"""
    p = os.path.join(ws, 'src', 'rm_perception', 'cloud_accumulator', 'cloud_accumulator', 'pcd_io.py')
    if not os.path.isfile(p):
        raise RuntimeError('找不到 %s' % p)
    spec = importlib.util.spec_from_file_location('pio', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.read_pcd_xyz


def main(argv=None):
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    ap = argparse.ArgumentParser(description='PCD bbox/z 跨度体检（3D 先验落盘后用）')
    ap.add_argument('pcd')
    ap.add_argument('--z-span-warn', type=float, default=3.0)
    ap.add_argument('--xy-span-warn', type=float, default=40.0)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    if not os.path.isfile(a.pcd):
        print('[体检] 没有这个文件：%s' % a.pcd)
        return 0
    try:
        import numpy as np
        read = _load_reader(ws)
        xyz = read(a.pcd)
    except Exception as e:
        print('[体检] 跳过（%s）' % e)
        return 0
    if len(xyz) == 0:
        print('[体检] %s：0 点' % a.pcd)
        return 0
    mn, mx = xyz.min(axis=0), xyz.max(axis=0)
    zspan = float(mx[2] - mn[2])
    xspan, yspan = float(mx[0] - mn[0]), float(mx[1] - mn[1])
    bad = []
    if a.z_span_warn > 0 and zspan > a.z_span_warn:
        bad.append('z 跨度 %.2f m > %.2f m' % (zspan, a.z_span_warn))
    if a.xy_span_warn > 0 and max(xspan, yspan) > a.xy_span_warn:
        bad.append('XY 跨度 %.2f×%.2f m 超过 %.2f m' % (xspan, yspan, a.xy_span_warn))
    info = {'pcd': a.pcd, 'points': int(len(xyz)), 'bbox_min': [float(v) for v in mn],
            'bbox_max': [float(v) for v in mx], 'z_span': zspan, 'xy_span': [xspan, yspan],
            'z_span_warn': a.z_span_warn, 'bad': bad}
    if a.json:
        print(json.dumps(info, ensure_ascii=False))
    else:
        print('  [体检] %s' % a.pcd)
        print('  [体检] %d 点  bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]  z 跨度=%.2f m  XY 跨度=%.2f m'
              % (len(xyz), mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], zspan, max(xspan, yspan)))
    if bad:
        name = os.path.basename(a.pcd)[:-4]
        print('  ⚠️⚠️ WARNING：这份 3D 先验不合理 —— %s' % '；'.join(bad))
        print('        （健康累积云实测 z 跨度 1.96~2.02 m）⇒ 很可能混进了 LIO 退化/错位姿累积的点；'
              '它已被写出（不静默拒绝），但**别拿它当前验**：')
        print('        ① 回滚到上一代：tools/scripts/mapping/map_archive.sh restore --name %s' % name)
        print('        ② 改名另存：ros2 param set /cloud_accumulator map_name %s_bad 后再 save' % name)
        print('        ③ 看更多细节：python3 tools/scripts/mapping/pcd_stats.py %s' % a.pcd)
        return 3
    return 0


if __name__ == '__main__':
    sys.exit(main())
