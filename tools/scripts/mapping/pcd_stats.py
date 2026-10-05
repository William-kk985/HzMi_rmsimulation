#!/usr/bin/env python3
"""PCD 体检 / 体素下采样 / 转成"本仓资产布局"的小工具（离线，不需要 ROS）。

为什么需要它：
  · small_point_lio 的 /map_save 写出来的是 **0.02 m 体素均值**的原始图
    （点数可达百万级、几十 MB）⇒ 直接提交进仓库不合适，也不符合既有资产
    （PCD/RMUC2026.pcd 是 26,618 点 / 0.85 MB）的性格；
  · 同时需要一份**可核对的数字**（点数 / bbox / 体素密度 / 地面 z）写进文档。

子命令（一个工具三种用法）：
  1) 体检：
       python3 tools/scripts/mapping/pcd_stats.py PCD/RMUC2026_spl.pcd
     输出 JSON：点数、bbox、每轴 min/max、体素密度（0.1/0.2/0.5 m）、
     地面 z（z 直方图众数附近的中位数）、字段列表、文件大小。
  2) 下采样 + 转换：
       python3 tools/scripts/mapping/pcd_stats.py RAW.pcd --voxel 0.10 --out PCD/RMUC2026_spl.pcd
     · 体素下采样 = 每体素取**质心**（与 small_point_lio 内部的 PointcloudMapping 同语义）；
     · --out 默认写 **8 字段**布局（x y z intensity normal_x normal_y normal_z curvature，
         全部 float32，DATA binary）—— 与 PCD/RMUC2026.pcd / PCD/RMUL2026.pcd **逐字段同构**，
         这样任何按旧资产写的消费者（PointXYZ / PointXYZI / 8 字段全读）都不用改。
  3) 对比两份 PCD（A/B 用）：
       python3 tools/scripts/mapping/pcd_stats.py A.pcd --compare B.pcd
     打印两者的 bbox 差与质心差（用于验证"某条导出路线是不是差一个常量平移"）。

支持 PCD 数据段：binary / ascii / binary_compressed（读取只取 x/y/z，其余字段一律忽略）。
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys

import numpy as np


# ------------------------------------------------------------------ 读取
def _parse_header(f):
    header = {'fields': [], 'size': [], 'type': [], 'count': []}
    while True:
        raw = f.readline()
        if not raw:
            raise ValueError('PCD 头部意外结束（没有 DATA 行）')
        line = raw.decode('ascii', 'replace').strip()
        if not line or line.startswith('#'):
            continue
        key, _, val = line.partition(' ')
        key = key.upper()
        toks = val.split()
        if key == 'FIELDS':
            header['fields'] = toks
        elif key == 'SIZE':
            header['size'] = [int(t) for t in toks]
        elif key == 'TYPE':
            header['type'] = toks
        elif key == 'COUNT':
            header['count'] = [int(t) for t in toks]
        elif key == 'WIDTH':
            header['width'] = int(toks[0])
        elif key == 'HEIGHT':
            header['height'] = int(toks[0])
        elif key == 'POINTS':
            header['points'] = int(toks[0])
        elif key == 'DATA':
            header['data'] = toks[0].lower()
            break
    if not header['count']:
        header['count'] = [1] * len(header['fields'])
    if not header['size']:
        header['size'] = [4] * len(header['fields'])
    if not header['type']:
        header['type'] = ['F'] * len(header['fields'])
    return header


_NP_TYPE = {('F', 4): '<f4', ('F', 8): '<f8',
            ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4',
            ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4'}


def read_pcd_xyz(path):
    """→ (xyz float32 (N,3), header dict)。只取 x/y/z 三个字段。"""
    with open(path, 'rb') as f:
        h = _parse_header(f)
        n = h.get('points') or (h.get('width', 0) * h.get('height', 0))
        names = h['fields']
        strides = [h['size'][i] * h['count'][i] for i in range(len(names))]
        offsets, off = {}, 0
        for i, nm in enumerate(names):
            offsets[nm] = (off, h['size'][i], h['type'][i])
            off += strides[i]
        point_step = off
        data_kind = h['data']
        if data_kind == 'binary':
            buf = f.read(n * point_step)
            arr = np.frombuffer(buf, dtype=np.uint8, count=len(buf)).reshape(-1, point_step)
            xyz = np.empty((arr.shape[0], 3), dtype=np.float32)
            for k, nm in enumerate(('x', 'y', 'z')):
                o, sz, tp = offsets[nm]
                npdt = _NP_TYPE[(tp.upper(), sz)]
                xyz[:, k] = arr[:, o:o + sz].copy().view(npdt).ravel().astype(np.float32)
                if tp.upper() == 'F' and sz == 8:
                    xyz[:, k] = xyz[:, k]  # float64 → float32 已在上一步 astype 完成
        elif data_kind == 'ascii':
            txt = f.read().decode('ascii', 'replace')
            rows = np.array([[float(t) for t in ln.split()] for ln in txt.splitlines() if ln.strip()],
                            dtype=np.float64)
            if rows.size == 0:
                xyz = np.zeros((0, 3), dtype=np.float32)
            else:
                idx = [names.index(c) for c in ('x', 'y', 'z')]
                xyz = rows[:, idx].astype(np.float32)
        elif data_kind == 'binary_compressed':
            csize, usize = struct.unpack('<II', f.read(8))
            comp = f.read(csize)
            raw = _lzf_decompress(comp, usize)
            arr = np.frombuffer(raw, dtype=np.uint8)
            # binary_compressed 是"按字段分块"存的：先 x 的 n 个值、再 y、再 z …
            npts = n
            xyz = np.empty((npts, 3), dtype=np.float32)
            for k, nm in enumerate(('x', 'y', 'z')):
                o, sz, tp = offsets[nm]
                npdt = _NP_TYPE[(tp.upper(), sz)]
                seg = arr[k * npts * sz:(k + 1) * npts * sz]
                xyz[:, k] = seg.copy().view(npdt).astype(np.float32)
        else:
            raise ValueError('不支持的 DATA 类型: %s' % data_kind)
    return xyz, h


def _lzf_decompress(data, out_len):
    """纯 Python 版 liblzf 解压（只用于 binary_compressed，罕见路径）。"""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:
            out += data[i:i + ctrl + 1]
            i += ctrl + 1
        else:
            length = ctrl >> 5
            if length == 7:
                length += data[i]
                i += 1
            ref = len(out) - ((ctrl & 0x1f) << 8) - data[i] - 1
            i += 1
            if ref < 0:
                raise ValueError('lzf 参考位置越界')
            for _ in range(length + 2):
                out.append(out[ref])
                ref += 1
    if len(out) != out_len:
        raise ValueError('lzf 解压长度不符：%d != %d' % (len(out), out_len))
    return bytes(out)


# ------------------------------------------------------------------ 写出
def write_pcd_xyz(path, xyz, legacy8=True):
    """写 PCD；legacy8=True 时写成本仓资产同构的 8 字段布局（binary）。"""
    xyz = np.asarray(xyz, dtype=np.float32)
    n = xyz.shape[0]
    if legacy8:
        buf = np.zeros((n, 8), dtype=np.float32)
        buf[:, :3] = xyz
    else:
        buf = xyz
    fields = ('x y z intensity normal_x normal_y normal_z curvature' if legacy8 else 'x y z')
    nf = buf.shape[1]
    with open(path, 'wb') as f:
        f.write(b'# .PCD v0.7 - Point Cloud Data file format\n')
        f.write(b'VERSION 0.7\n')
        f.write(('FIELDS %s\n' % fields).encode())
        f.write(('SIZE %s\n' % ' '.join(['4'] * nf)).encode())
        f.write(('TYPE %s\n' % ' '.join(['F'] * nf)).encode())
        f.write(('COUNT %s\n' % ' '.join(['1'] * nf)).encode())
        f.write(('WIDTH %d\n' % n).encode())
        f.write(b'HEIGHT 1\n')
        f.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(('POINTS %d\n' % n).encode())
        f.write(b'DATA binary\n')
        f.write(buf.tobytes())


# ------------------------------------------------------------------ 统计
def voxel_downsample(xyz, leaf):
    """每体素取质心（与 small_point_lio 的 PointcloudMapping 同语义）。"""
    if len(xyz) == 0:
        return xyz
    key = np.floor(xyz.astype(np.float64) / leaf).astype(np.int64)
    # 三轴整数键 → 单键（用 void view 做 unique）
    kv = np.ascontiguousarray(key).view([('a', '<i8'), ('b', '<i8'), ('c', '<i8')]).ravel()
    uniq, inv = np.unique(kv, return_inverse=True)
    sums = np.zeros((len(uniq), 3), dtype=np.float64)
    np.add.at(sums, inv, xyz.astype(np.float64))
    cnt = np.bincount(inv, minlength=len(uniq)).astype(np.float64)[:, None]
    return (sums / cnt).astype(np.float32)


def stats(xyz, path=None, leafs=(0.10, 0.20, 0.50)):
    out = {'file': path, 'points': int(len(xyz))}
    if path and os.path.isfile(path):
        out['file_bytes'] = os.path.getsize(path)
        out['file_mb'] = round(os.path.getsize(path) / 1e6, 2)
    if len(xyz) == 0:
        return out
    finite = np.isfinite(xyz).all(axis=1)
    out['nonfinite_points'] = int((~finite).sum())
    xyz = xyz[finite]
    mn, mx = xyz.min(axis=0), xyz.max(axis=0)
    out['bbox'] = {'min': [round(float(v), 3) for v in mn],
                   'max': [round(float(v), 3) for v in mx],
                   'size': [round(float(v), 3) for v in (mx - mn)]}
    out['centroid'] = [round(float(v), 4) for v in xyz.mean(axis=0)]
    out['voxels'] = {}
    for leaf in leafs:
        k = np.floor(xyz.astype(np.float64) / leaf).astype(np.int64)
        nvox = len(np.unique(k.view([('a', '<i8'), ('b', '<i8'), ('c', '<i8')]).ravel()))
        out['voxels']['%g' % leaf] = {
            'n_voxels': int(nvox),
            'pts_per_voxel': round(len(xyz) / max(nvox, 1), 2),
            'voxels_per_m2_xy': None,
        }
    # 地面 z：xy 在起点附近 4x4 m 的点的 z 中位数（最贴近"地面"的估计）
    near = xyz[(np.abs(xyz[:, 0]) < 2) & (np.abs(xyz[:, 1]) < 2)]
    if len(near) > 100:
        out['z_near_origin'] = {
            'p01': round(float(np.percentile(near[:, 2], 1)), 3),
            'p05': round(float(np.percentile(near[:, 2], 5)), 3),
            'p50': round(float(np.percentile(near[:, 2], 50)), 3),
            'p95': round(float(np.percentile(near[:, 2], 95)), 3),
            'p99': round(float(np.percentile(near[:, 2], 99)), 3),
        }
        # 地面 z = 近地面点的众数（0.02 m 分桶）
        h, edges = np.histogram(near[:, 2], bins=60)
        out['z_near_origin']['mode_bin'] = round(float(edges[int(np.argmax(h))]), 3)
    # xy 占用面积（0.10 m 栅格），用来算"点云覆盖了多少地面/墙面"
    kxy = np.floor(xyz[:, :2].astype(np.float64) / 0.10).astype(np.int64)
    out['xy_footprint_m2'] = round(len(np.unique(
        kxy.view([('a', '<i8'), ('b', '<i8')]).ravel())) * 0.01, 2)
    return out


def main():
    ap = argparse.ArgumentParser(description='PCD 体检 / 下采样 / 对比')
    ap.add_argument('pcd')
    ap.add_argument('--out', help='下采样后写到这个路径（默认 8 字段布局）')
    ap.add_argument('--voxel', type=float, default=0.0, help='下采样体素边长（m）；0=不下采样')
    ap.add_argument('--xyz-only', action='store_true', help='--out 写 3 字段而不是 8 字段')
    ap.add_argument('--compare', help='与另一份 PCD 对比（bbox/质心差）')
    ap.add_argument('--json', help='把统计写到这个 JSON 文件')
    args = ap.parse_args()

    xyz, h = read_pcd_xyz(args.pcd)
    st = stats(xyz, args.pcd)
    st['fields'] = h['fields']
    st['data'] = h.get('data')
    print('PCDJSON ' + json.dumps(st, ensure_ascii=False))
    if args.compare:
        other, _ = read_pcd_xyz(args.compare)
        so = stats(other, args.compare)
        d = {}
        if len(xyz) and len(other):
            d['bbox_min_diff'] = [round(float(a - b), 3) for a, b in
                                  zip(st['bbox']['min'], so['bbox']['min'])]
            d['bbox_max_diff'] = [round(float(a - b), 3) for a, b in
                                  zip(st['bbox']['max'], so['bbox']['max'])]
            d['centroid_diff'] = [round(float(a - b), 4) for a, b in
                                  zip(st['centroid'], so['centroid'])]
            d['points_ratio'] = round(st['points'] / max(so['points'], 1), 3)
        print('CMPJSON ' + json.dumps({'a': args.pcd, 'b': args.compare,
                                       'a_stats': st, 'b_stats': so, 'diff': d},
                                      ensure_ascii=False))
    if args.out:
        out_xyz = voxel_downsample(xyz, args.voxel) if args.voxel > 0 else xyz
        write_pcd_xyz(args.out, out_xyz, legacy8=not args.xyz_only)
        st2 = stats(out_xyz, args.out)
        print('OUTJSON ' + json.dumps({'out': args.out, 'voxel': args.voxel, **st2},
                                      ensure_ascii=False))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(st, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
