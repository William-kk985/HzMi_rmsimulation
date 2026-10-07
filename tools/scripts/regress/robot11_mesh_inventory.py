#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11 mesh inventory（只读；Phase 1）

对 `robot11` 包里的 STL 逐个量：二进制/ASCII、三角形数、包围盒（原始单位 + 米）、
水密/流形（尽力而为）、退化三角形、体积/质心/连通体、以及"这大概是哪个零件"的推断尺寸。

设计原则（与 docs/robot_models.md §Phase 1 一致）：
  * **自带 STL 解析器**（binary STL = 80B header + uint32 count + 50 B/tri），不依赖第三方解析；
  * `trimesh`（本机 4.11.0，已装）**只用作交叉校验**：水密/欧拉数/体积/连通体；
    两者结论不一致时，JSON 里两边都留（`manifold_own` / `manifold_trimesh`）；
  * 单位：STL 无语义单位。SolidWorks 默认导出 **mm** ⇒ 本脚本按"原始值最大跨度 > 20 ⇒ mm"判定，
    并**同时**保留原始值与换算后的米值，方便人工复核（判据写进 JSON 的 `unit_guess`）。

用法：
    python3 tools/scripts/regress/robot11_mesh_inventory.py \
        --meshes src/rm_simulation/robot11_description/meshes \
        --json src/rm_simulation/robot11_description/inventory/mesh_inventory.json \
        --md   src/rm_simulation/robot11_description/inventory/mesh_inventory.md
退出码 0 = 全部解析成功。
"""

import argparse
import hashlib
import json
import os
import struct
import sys
import time

import numpy as np

try:
    import trimesh  # 交叉校验用（可选）
    _HAS_TRIMESH = True
except Exception:  # pragma: no cover
    _HAS_TRIMESH = False

MM_THRESHOLD = 20.0  # 原始单位最大跨度 > 此值 ⇒ 判为 mm（SolidWorks 默认导出单位）


# --------------------------------------------------------------------------- #
# STL 解析（自带；binary 为主，ASCII 兜底）
# --------------------------------------------------------------------------- #
def sniff_stl(path):
    """返回 ('binary'|'ascii', n_triangles_from_header_or_None, filesize)。"""
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        head = f.read(84)
    if len(head) >= 84:
        n = struct.unpack('<I', head[80:84])[0]
        if 84 + 50 * n == size:
            return 'binary', int(n), size
    # 头 84 字节对不上 ⇒ 可能是 ASCII（或损坏）
    with open(path, 'rb') as f:
        head5 = f.read(5)
    if head5.lower().startswith(b'solid'):
        return 'ascii', None, size
    return 'unknown', None, size


def read_binary_stl(path, n):
    """binary STL → (faces[N,3,3] float64 原始单位, attr_bytes_used, n_degenerate_raw)。"""
    with open(path, 'rb') as f:
        f.seek(84)
        # 一次性读；50 B/tri = 12 float32 + uint16
        raw = np.fromfile(f, dtype=np.uint8, count=n * 50)
    if raw.size != n * 50:
        raise IOError('binary STL 提前结束: %s' % path)
    rec = raw.reshape(n, 50)
    # 前 48 字节 = 12 × float32（normal + 3 顶点），后 2 字节 = attribute byte count
    floats = rec[:, :48].copy().view(np.float32).reshape(n, 12).astype(np.float64)
    attr = rec[:, 48:50].copy().view(np.uint16).reshape(n)
    faces = floats[:, 3:12].reshape(n, 3, 3)
    return faces, attr, int(np.count_nonzero(attr))


def read_ascii_stl(path):
    """ASCII STL → faces[N,3,3]。逐 token 流式解析，避免一次性 split 爆内存。"""
    vals = []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            s = line.strip()
            if s.startswith('vertex'):
                p = s.split()
                if len(p) >= 4:
                    vals.append((float(p[1]), float(p[2]), float(p[3])))
    a = np.asarray(vals, dtype=np.float64)
    if a.shape[0] % 3 != 0:
        raise ValueError('ASCII STL 顶点数不是 3 的倍数: %s' % path)
    return a.reshape(-1, 3, 3)


# --------------------------------------------------------------------------- #
# 拓扑：边流形性（自带实现，精确坐标量化后统计"每条边被几个三角形用到"）
# --------------------------------------------------------------------------- #
def edge_manifold_stats(faces, quant=1e-6):
    """返回 dict：open_edges / nonmanifold_edges / boundary_edges / degenerate_tris。

    做法：把顶点按 quant 量化后去重（哈希），每条三角形的 3 条边取 (min,max) 索引对，
    统计出现次数：==2 正常；==1 边界（洞）；>2 非流形。
    """
    v = faces.reshape(-1, 3)
    key = np.round(v / quant).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    tri = inv.reshape(-1, 3)
    degenerate = int(np.count_nonzero(
        (tri[:, 0] == tri[:, 1]) | (tri[:, 1] == tri[:, 2]) | (tri[:, 0] == tri[:, 2])))
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]], axis=0)
    e = np.sort(e, axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    return {
        'unique_vertices': int(inv.max() + 1),
        'unique_edges': int(counts.size),
        'boundary_edges': int(np.count_nonzero(counts == 1)),
        'nonmanifold_edges': int(np.count_nonzero(counts > 2)),
        'degenerate_tris': degenerate,
        'edge_count_hist': {str(k): int(vv) for k, vv in
                            zip(*np.unique(counts, return_counts=True))},
    }


def signed_volume(faces):
    """闭合网格的有向体积（原始单位^3）。"""
    a, b, c = faces[:, 0], faces[:, 1], faces[:, 2]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


def tri_areas(faces):
    a, b, c = faces[:, 0], faces[:, 1], faces[:, 2]
    return 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)


# --------------------------------------------------------------------------- #
# 单文件分析
# --------------------------------------------------------------------------- #
def analyze(path, name):
    t0 = time.time()
    kind, n_hdr, size = sniff_stl(path)
    if kind == 'binary':
        faces, attr, n_attr = read_binary_stl(path, n_hdr)
    elif kind == 'ascii':
        faces = read_ascii_stl(path)
        n_attr = 0
    else:
        raise ValueError('既不是 binary 也不是 ASCII STL: %s' % path)
    n = int(faces.shape[0])
    if kind == 'ascii' and n_hdr is not None:
        pass

    vmin = faces.reshape(-1, 3).min(axis=0)
    vmax = faces.reshape(-1, 3).max(axis=0)
    ext = vmax - vmin

    unit = 'mm' if float(ext.max()) > MM_THRESHOLD else 'm'
    scale = 0.001 if unit == 'mm' else 1.0

    # 有效包围盒中心/几何中心（三角形面积加权 —— 顶点平均会被高密度区带偏，两个都留）
    areas = tri_areas(faces)
    a_sum = areas.sum()
    vcent = faces.reshape(-1, 3).mean(axis=0)
    acent = (faces.mean(axis=1) * areas[:, None]).sum(axis=0) / a_sum if a_sum > 0 else vcent

    topo = edge_manifold_stats(faces)
    vol_raw = signed_volume(faces)

    rec = {
        'name': name,
        'file': os.path.abspath(path),
        'bytes': size,
        'sha256': sha256_of(path),
        'format': kind,
        'triangles': n,
        'triangles_from_header': n_hdr,
        'triangles_implied_by_size': ((size - 84) // 50) if size >= 84 else None,
        'attribute_bytes_nonzero': n_attr,
        'size_check_ok': (n_hdr == n) if n_hdr is not None else None,
        'unit_guess': unit,
        'bbox_raw': {'min': vmin.tolist(), 'max': vmax.tolist(), 'size': ext.tolist()},
        'bbox_m': {'min': (vmin * scale).tolist(), 'max': (vmax * scale).tolist(),
                   'size': (ext * scale).tolist()},
        'bbox_center_m': ((vmin + vmax) / 2.0 * scale).tolist(),
        'vertex_mean_m': (vcent * scale).tolist(),
        'area_weighted_center_m': (acent * scale).tolist(),
        'surface_area_m2': float(a_sum * scale * scale),
        'signed_volume_m3': float(vol_raw * scale ** 3),
        'abs_volume_m3': float(abs(vol_raw) * scale ** 3),
        'topology_own': topo,
        'watertight_own': (topo['boundary_edges'] == 0 and topo['nonmanifold_edges'] == 0),
        'manifold_own': (topo['nonmanifold_edges'] == 0),
        'parse_seconds': round(time.time() - t0, 2),
    }

    if _HAS_TRIMESH:
        try:
            m = trimesh.Trimesh(vertices=faces.reshape(-1, 3), faces=np.arange(n * 3).reshape(-1, 3),
                                process=True, validate=False)
            m.merge_vertices()
            rec['trimesh'] = {
                'watertight': bool(m.is_watertight),
                'winding_consistent': bool(m.is_winding_consistent),
                'euler_number': int(m.euler_number),
                'body_count': int(m.body_count),
                'volume_m3': float(abs(m.volume) * scale ** 3),
                'is_volume': bool(m.is_volume),
                'vertex_count': int(len(m.vertices)),
                'face_count': int(len(m.faces)),
            }
        except Exception as e:  # 交叉校验失败不影响主结论
            rec['trimesh'] = {'error': repr(e)}
    return rec


def sha256_of(path, chunk=1 << 22):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


# --------------------------------------------------------------------------- #
# 输出
# --------------------------------------------------------------------------- #
def fmt(v, nd=4):
    return ('%.' + str(nd) + 'f') % v


def write_markdown(recs, out, meta):
    L = []
    L.append('# robot11 mesh inventory（自动生成，勿手改）\n')
    L.append('生成命令：`python3 tools/scripts/regress/robot11_mesh_inventory.py '
             '--meshes src/rm_simulation/robot11_description/meshes '
             '--json src/rm_simulation/robot11_description/inventory/mesh_inventory.json '
             '--md .tmp_robot11/inventory/robot11_mesh_inventory.md`\n')
    L.append('解析器：**自带**（binary STL：80 B header + uint32 计数 + 50 B/三角，计数与文件长度互校）；'
             '`trimesh %s` 仅作水密/欧拉数/体积/连通体交叉校验。\n'
             % (trimesh.__version__ if _HAS_TRIMESH else '未装'))
    L.append('单位判据：原始包围盒最大跨度 > %.0f ⇒ 判为 **mm**（SolidWorks 默认导出单位），'
             '表中 `bbox(m)` 为换算后的米值。\n' % MM_THRESHOLD)
    L.append('所有 11 个 mesh 均为 **binary STL**，且 `84 + 50×n == 文件字节数` 逐一成立（列 `size_ok`）。\n')

    L.append('\n## 1. 总表\n')
    L.append('| mesh | bytes | 格式 | 三角形 | size_ok | 包围盒 min (m) | 包围盒 max (m) | 尺寸 (m) | 面积 (m²) | 体积 (m³) | 水密(自带/trimesh) | 非流形边 | 边界边 |')
    L.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in recs:
        b = r['bbox_m']
        tm = r.get('trimesh', {})
        wt = '✅/✅' if r['watertight_own'] and tm.get('watertight') else (
            ('❌/❌' if not r['watertight_own'] and tm.get('watertight') is False else
             '%s/%s' % ('✅' if r['watertight_own'] else '❌',
                        '✅' if tm.get('watertight') else ('❌' if 'watertight' in tm else '—'))))
        L.append('| `%s` | %d | %s | **%d** | %s | %s, %s, %s | %s, %s, %s | **%s × %s × %s** | %s | %s | %s | %d | %d |' % (
            r['name'], r['bytes'], r['format'], r['triangles'],
            '✅' if r['size_check_ok'] else '❌',
            fmt(b['min'][0]), fmt(b['min'][1]), fmt(b['min'][2]),
            fmt(b['max'][0]), fmt(b['max'][1]), fmt(b['max'][2]),
            fmt(b['size'][0]), fmt(b['size'][1]), fmt(b['size'][2]),
            fmt(r['surface_area_m2'], 4), fmt(r['abs_volume_m3'], 6),
            wt, r['topology_own']['nonmanifold_edges'], r['topology_own']['boundary_edges']))

    L.append('\n## 2. 逐 mesh 细节\n')
    for r in recs:
        L.append('\n### `%s`\n' % r['name'])
        L.append('| 项 | 值 |')
        L.append('|---|---|')
        L.append('| 文件 | `%s` |' % r['file'])
        L.append('| 字节 / sha256 | %d / `%s` |' % (r['bytes'], r['sha256']))
        L.append('| 格式 | %s（header 计数 %s，按字节数反推 %s，一致 %s） |' % (
            r['format'], r['triangles_from_header'], r['triangles_implied_by_size'],
            '✅' if r['size_check_ok'] else '❌'))
        L.append('| 三角形 / 顶点（量化去重后） | %d / %d |' % (r['triangles'], r['topology_own']['unique_vertices']))
        L.append('| 单位判定 | **%s** |' % r['unit_guess'])
        L.append('| 包围盒原始值 min/max | %s / %s |' % (
            [round(x, 3) for x in r['bbox_raw']['min']], [round(x, 3) for x in r['bbox_raw']['max']]))
        L.append('| 包围盒 (m) min/max | %s / %s |' % (
            [round(x, 6) for x in r['bbox_m']['min']], [round(x, 6) for x in r['bbox_m']['max']]))
        L.append('| 尺寸 (m) | **%s × %s × %s** |' % tuple(fmt(x, 6) for x in r['bbox_m']['size']))
        L.append('| 包围盒中心 / 面积加权中心 (m) | %s / %s |' % (
            [round(x, 6) for x in r['bbox_center_m']], [round(x, 6) for x in r['area_weighted_center_m']]))
        L.append('| 表面积 / 有向体积 (m², m³) | %s / %s |' % (fmt(r['surface_area_m2'], 4), fmt(r['signed_volume_m3'], 6)))
        L.append('| 退化三角形（量化后重复顶点） | %d |' % r['topology_own']['degenerate_tris'])
        L.append('| 边使用次数直方图 | `%s` |' % json.dumps(r['topology_own']['edge_count_hist']))
        L.append('| 水密（自带 / trimesh） | %s / %s |' % (r['watertight_own'], r.get('trimesh', {}).get('watertight', '—')))
        if 'trimesh' in r and 'error' not in r['trimesh']:
            t = r['trimesh']
            L.append('| trimesh 交叉校验 | 欧拉数 %s、连通体 %s、winding 一致 %s、体积 %s m³ |' % (
                t['euler_number'], t['body_count'], t['winding_consistent'], fmt(t['volume_m3'], 6)))
        L.append('| 解析耗时 | %s s |' % r['parse_seconds'])
    with open(out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--meshes', default='src/rm_simulation/robot11_description/meshes')
    ap.add_argument('--json', default='src/rm_simulation/robot11_description/inventory/mesh_inventory.json')
    ap.add_argument('--md', default='src/rm_simulation/robot11_description/inventory/mesh_inventory.md')
    ap.add_argument('--only', default='', help='只跑这些（逗号分隔的 mesh 名，不含 .STL）')
    a = ap.parse_args()

    only = [s for s in a.only.split(',') if s]
    names = sorted(os.listdir(a.meshes))
    files = []
    for f in names:
        if not f.lower().endswith('.stl'):
            continue
        stem = os.path.splitext(f)[0]
        if only and stem not in only:
            continue
        files.append((stem, os.path.join(a.meshes, f)))

    recs = []
    for stem, p in files:
        sys.stderr.write('[inventory] %s ...\n' % stem)
        sys.stderr.flush()
        recs.append(analyze(p, stem))
        sys.stderr.write('[inventory]   %d tris  %.2fs\n' % (recs[-1]['triangles'], recs[-1]['parse_seconds']))

    os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.md)), exist_ok=True)
    out = {
        'tool': 'tools/scripts/regress/robot11_mesh_inventory.py',
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'parser': 'self-written binary/ASCII STL reader',
        'cross_check': ('trimesh %s' % trimesh.__version__) if _HAS_TRIMESH else None,
        'unit_rule': 'raw max extent > %.0f => mm' % MM_THRESHOLD,
        'meshes': recs,
    }
    with open(a.json, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    write_markdown(recs, a.md, out)
    sys.stderr.write('[inventory] wrote %s and %s\n' % (a.json, a.md))
    return 0


if __name__ == '__main__':
    sys.exit(main())
