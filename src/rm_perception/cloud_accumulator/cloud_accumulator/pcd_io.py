#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCD 读写（只关心 x/y/z，写出去的是本仓资产口径的 8 字段 binary）。

为什么要自己写这 20 行、而不是依赖 pcl_ros / open3d：
  · 本节点是 **纯 Python + numpy**（见 package.xml 的依赖列表），装到任何一台跑得动
    仿真/RViz 的机器上都不用再编 PCL；
  · v1 只需要"读 xyz / 写 xyz"，字段口径必须与既有资产 PCD/RMUC2026.pcd 等**逐字段同构**
    （8 字段 float32 binary：x y z intensity normal_x normal_y normal_z curvature，
     见 tools/scripts/mapping/pcd_stats.py 顶部说明），这样 PointXYZ / PointXYZI / 8 字段全读的
     老消费者都不用改。
  · 读：ascii / binary 都支持；`binary_compressed`（LZF）**明确报错并给替代命令**，
    而不是悄悄读出乱码（既有资产里没有这种，但用户拿别的工具生成的 PCD 可能是）。
"""
from __future__ import annotations

import numpy as np

HEADER_KEYS = ('VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT',
               'VIEWPOINT', 'POINTS', 'DATA')
LEGACY8 = ('x y z intensity normal_x normal_y normal_z curvature').split()
NP_TYPE = {'F': {4: '<f4', 8: '<f8'}, 'U': {1: '<u1', 2: '<u2', 4: '<u4', 8: '<u8'},
           'I': {1: '<i1', 2: '<i2', 4: '<i4', 8: '<i8'}}


def read_header(f):
    hdr = {'fields': [], 'size': [], 'type': [], 'count': []}
    raw_lines = []
    while True:
        raw = f.readline()
        if not raw:
            raise ValueError('PCD 头部意外结束（没有 DATA 行）')
        raw_lines.append(raw)
        line = raw.decode('ascii', 'replace').strip()
        if not line or line.startswith('#'):
            continue
        key, _, val = line.partition(' ')
        key = key.upper()
        toks = val.split()
        if key == 'FIELDS':
            hdr['fields'] = toks
        elif key == 'SIZE':
            hdr['size'] = [int(t) for t in toks]
        elif key == 'TYPE':
            hdr['type'] = toks
        elif key == 'COUNT':
            hdr['count'] = [int(t) for t in toks]
        elif key == 'WIDTH':
            hdr['width'] = int(toks[0])
        elif key == 'HEIGHT':
            hdr['height'] = int(toks[0])
        elif key == 'POINTS':
            hdr['points'] = int(toks[0])
        elif key == 'DATA':
            hdr['data'] = toks[0].lower() if toks else 'ascii'
            break
    if not hdr['count']:
        hdr['count'] = [1] * len(hdr['fields'])
    if not hdr['size']:
        hdr['size'] = [4] * len(hdr['fields'])
    if not hdr['type']:
        hdr['type'] = ['F'] * len(hdr['fields'])
    hdr['n'] = hdr.get('points') or (hdr.get('width', 0) * hdr.get('height', 0))
    return hdr


def read_pcd_xyz(path, max_points=None):
    """读 PCD 的 x/y/z ⇒ (N,3) float64。"""
    with open(path, 'rb') as f:
        hdr = read_header(f)
        fields = [x.lower() for x in hdr['fields']]
        for need in ('x', 'y', 'z'):
            if need not in fields:
                raise ValueError('%s 里没有 %s 字段（FIELDS=%s）' % (path, need, hdr['fields']))
        ix = [fields.index(k) for k in ('x', 'y', 'z')]
        n = int(hdr['n'])
        if hdr['data'] == 'ascii':
            arr = np.loadtxt(f, dtype=np.float64, ndmin=2)
            xyz = arr[:, ix] if arr.size else np.zeros((0, 3))
        elif hdr['data'] == 'binary':
            dt = np.dtype([(fn, NP_TYPE[t][s]) for fn, t, s in
                           zip(hdr['fields'], hdr['type'], hdr['size'])])
            buf = f.read(dt.itemsize * n)
            arr = np.frombuffer(buf, dtype=dt, count=n)
            # dtype 的字段名用**文件里原始的**拼写（FIELDS 行），索引必须按原名取
            names = [hdr['fields'][i] for i in ix]
            xyz = np.stack([arr[nm].astype(np.float64) for nm in names], axis=1)
        elif hdr['data'] == 'binary_compressed':
            raise ValueError(
                '%s 是 binary_compressed（LZF）压缩 PCD，本节点不读它。\n'
                '  先转成 binary（顺带体素下采样）：\n'
                '      python3 tools/scripts/mapping/pcd_stats.py %s --voxel 0.10 --out /tmp/conv.pcd\n'
                '  再用 /tmp/conv.pcd 走 load。' % (path, path))
        else:
            raise ValueError('%s 的 DATA 段类型不认识：%r' % (path, hdr['data']))
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    bad = ~np.isfinite(xyz).all(axis=1)
    if bad.any():
        xyz = xyz[~bad]
    if max_points and len(xyz) > max_points:
        xyz = xyz[:max_points]
    return xyz


def write_pcd_binary(path, xyz, legacy8=True):
    """写 8 字段（或 3 字段）binary PCD；列顺序与既有资产一致。"""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = len(xyz)
    if legacy8:
        arr = np.zeros((n, 8), dtype=np.float32)
        arr[:, 0:3] = xyz
        fields, size, typ = LEGACY8, [4] * 8, ['F'] * 8
    else:
        arr = xyz
        fields, size, typ = ['x', 'y', 'z'], [4, 4, 4], ['F', 'F', 'F']
    with open(path, 'wb') as f:
        f.write(b'# .PCD v0.7 - Point Cloud Data file format\n')
        f.write(('VERSION 0.7\n').encode())
        f.write(('FIELDS %s\n' % ' '.join(fields)).encode())
        f.write(('SIZE %s\n' % ' '.join(str(s) for s in size)).encode())
        f.write(('TYPE %s\n' % ' '.join(typ)).encode())
        f.write(('COUNT %s\n' % ' '.join(['1'] * len(fields))).encode())
        f.write(('WIDTH %d\nHEIGHT 1\n' % n).encode())
        f.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(('POINTS %d\n' % n).encode())
        f.write(b'DATA binary\n')
        f.write(np.ascontiguousarray(arr).tobytes())
    return n
