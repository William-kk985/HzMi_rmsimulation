#!/usr/bin/env python3
"""为「建图跑图」生成一条**保证不撞、且不跨半场**的覆盖路线（离线，不需要 ROS/Gazebo）。

为什么需要它（而不是手写一串坐标）：RMUC2026 是一张 29x16 m 的全场，
两个半场之间最窄只有 **0.45 m**（docs/worlds.md §6），而车直径 0.44 m
⇒ 手写坐标极容易在"过场"那条缝上把车卡住/顶墙。本工具把"哪里能走"
直接从**先验 2D 栅格图**里算出来（同一张图就是导航时用的那张），
于是路线的合法性由几何保证：所有航点与连线都在
`clearance >= --clearance` 的**同一个连通域**里。

算法（每一步都可在输出里核对）：
  1. 读 map_server 语义的 .pgm/.yaml（trinary：occ>occupied_thresh=占用、
     occ<free_thresh=空闲、其余未知）⇒ free 掩码；
  2. `distance_transform_edt(free) * resolution >= clearance` ⇒ **可行驶掩码**
     （clearance 默认 0.26 = 车半径 0.22 + 跟踪余量 0.04）；
  3. 取包含起点（默认 map 系 (0,0) = 出生点）的**连通域** —— 这一步就是
     "不跨半场"的硬保证（0.45 m 的缝在 0.26 m clearance 下不连通）；
  4. 在域内按 `--lane-spacing` 生成"割草机"式栅格航点，逐条 lane 排序；
  5. 相邻航点之间用 **A\*** 在可行驶掩码上求最短路径（8 邻域、禁止贴角穿越），
     Douglas-Peucker 抽稀 ⇒ 输出折线航点表；
  6. 打印域面积 / 路线总长 / 航点数，可选 --png 出一张俯视图核对。

用法：
  python3 tools/scripts/mapping/coverage_route.py \
      --map-yaml src/rm_nav_bringup/map/RMUC2026.yaml \
      --out .tmp_cache/spl2026/route.json --lane-spacing 2.0 --png .tmp_cache/spl2026/route.png

  只想看统计不写文件：加 --dry-run
  （⚠️ world:=RMUC2026 的先验图当前可能被其它跑图流程临时移走；被移走时用
    `--map-yaml` 指到副本，或先 `git checkout -- src/rm_nav_bringup/map/RMUC2026.pgm`）

输出 JSON 结构（coverage_drive.py 直接读它）：
  {"frame": "map", "resolution": 0.05, "clearance": 0.26, "start": [x, y],
   "waypoints": [[x, y], ...],           # 待跟踪的折线（*不是* 全部栅格点）
   "goals": [[x, y], ...],               # 割草机原始目标点（供核对覆盖率）
   "stats": {"area_m2":..., "path_len_m":..., "n_waypoints":...,
             "bbox": [minx,miny,maxx,maxy], "spawn_half_area_m2":...}}
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys

import numpy as np

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover
    print('需要 scipy（距离变换/连通域）: pip3 install scipy', file=sys.stderr)
    raise


# ---------------------------------------------------------------- 栅格图读取
def read_pgm(path: str) -> np.ndarray:
    """读 P5/P2 PGM → uint8 数组（shape = (h, w)，第 0 行 = 图像顶 = 最大 y）。"""
    with open(path, 'rb') as f:
        raw = f.read()
    fields, pos = [], 0
    magic = None
    while len(fields) < 3:  # magic + width + height + maxval
        # 跳过空白与 # 注释
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while pos < len(raw) and raw[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        tok = raw[start:pos].decode()
        if magic is None:
            magic = tok
        else:
            fields.append(int(tok))
    pos += 1  # 单个空白分隔符
    w, h, maxval = fields[0], fields[1], fields[2]
    if magic == 'P5':
        data = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=pos)
    else:  # P2 ascii
        data = np.array(raw[pos:].split(), dtype=np.uint16)[:w * h].astype(np.uint8)
    if maxval != 255:
        data = (data.astype(np.float64) * 255.0 / maxval).astype(np.uint8)
    return data.reshape(h, w)


def read_map_yaml(path: str) -> dict:
    """极简 YAML 读取（只认 map_server 那几个键，不引入 pyyaml 依赖问题）。"""
    out = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line or ':' not in line:
                continue
            k, v = line.split(':', 1)
            v = v.strip()
            if k.strip() == 'origin':
                out['origin'] = [float(x) for x in v.strip('[]').split(',')]
            elif k.strip() in ('resolution', 'occupied_thresh', 'free_thresh', 'negate'):
                out[k.strip()] = float(v)
            else:
                out[k.strip()] = v.strip('"\'')
    return out


def load_occupancy(yaml_path: str):
    """→ (free_mask, meta)。trinary 语义与 nav2_map_server 一致。"""
    meta = read_map_yaml(yaml_path)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        cand = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        if not os.path.isfile(cand):  # 允许 image 名与 yaml 名不一致
            cand = img_path
        img_path = cand
    img = read_pgm(img_path)
    occ = (255.0 - img.astype(np.float64)) / 255.0
    if meta.get('negate', 0):
        occ = 1.0 - occ
    free = occ < meta.get('free_thresh', 0.25)
    occd = occ > meta.get('occupied_thresh', 0.65)
    return free, occd, meta, img_path


# ------------------------------------------------------- map 系 ↔ 栅格坐标
def xy_to_rc(x, y, meta, shape):
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    col = int(math.floor((x - ox) / res))
    row = shape[0] - 1 - int(math.floor((y - oy) / res))
    return row, col


def rc_to_xy(row, col, meta, shape):
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    return ox + (col + 0.5) * res, oy + (shape[0] - 1 - row + 0.5) * res


# ------------------------------------------------------------------- A*
def astar(mask: np.ndarray, start: tuple, goal: tuple):
    """8 邻域 A*；mask=True 可走。返回 [(r,c)...] 或 None。禁止"贴角穿越"。"""
    h, w = mask.shape
    if not mask[start] or not mask[goal]:
        return None
    if start == goal:
        return [start]
    SQ2 = math.sqrt(2.0)

    def heur(a, b):
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dr + dc) + (SQ2 - 2.0) * min(dr, dc)

    openq = [(heur(start, goal), 0.0, start)]
    gscore = {start: 0.0}
    came = {}
    closed = set()
    nbr = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
           (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2))
    while openq:
        _, g, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        r, c = cur
        for dr, dc, cost in nbr:
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= h or nc >= w or not mask[nr, nc]:
                continue
            if dr and dc:  # 禁止穿角：两侧正交格都必须是可走的
                if not mask[r + dr, c] or not mask[r, c + dc]:
                    continue
            ng = g + cost
            nxt = (nr, nc)
            if ng < gscore.get(nxt, 1e18):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(openq, (ng + heur(nxt, goal), ng, nxt))
    return None


def rdp(points, eps):
    """Douglas-Peucker 抽稀（点已经是 xy 米制）。"""
    if len(points) < 3:
        return list(points)
    pts = np.asarray(points, dtype=float)
    keep = np.zeros(len(pts), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L = np.hypot(*ab)
        seg = pts[i + 1:j]
        if L < 1e-9:
            d = np.hypot(*(seg - a).T)
        else:
            d = np.abs(np.cross(ab, seg - a)) / L
        k = int(np.argmax(d))
        if d[k] > eps:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return [tuple(p) for p in pts[keep]]


# --------------------------------------------------------------------- main
def build_route(args):
    free, occ, meta, img_path = load_occupancy(args.map_yaml)
    res = meta['resolution']
    shape = free.shape
    # 可行驶掩码：距最近障碍/未知 >= clearance
    dist = ndimage.distance_transform_edt(free) * res
    mask = dist >= args.clearance
    # 起点所在连通域（"不跨半场"的硬保证）
    lab, n = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    r0, c0 = xy_to_rc(args.start[0], args.start[1], meta, shape)
    if not (0 <= r0 < shape[0] and 0 <= c0 < shape[1]):
        raise SystemExit('起点 %s 不在图内（图 %dx%d origin=%s res=%s）'
                         % (args.start, shape[1], shape[0], meta['origin'], res))
    # 起点本身可能落在 mask 外（离障碍太近）⇒ 找最近的可行驶格
    if not mask[r0, c0]:
        idx = np.argwhere(mask)
        if len(idx) == 0:
            raise SystemExit('可行驶掩码是空的（clearance 太大？）')
        d = np.hypot(idx[:, 0] - r0, idx[:, 1] - c0)
        r0, c0 = idx[int(np.argmin(d))]
        print('[route] ⚠️ 起点不在可行驶区，改用最近可行驶格 %s (%.2f, %.2f)'
              % ((r0, c0), *rc_to_xy(r0, c0, meta, shape)))
    comp = lab == lab[r0, c0]
    area = float(comp.sum()) * res * res
    print('[route] 图 %s' % img_path)
    print('[route] 图 %dx%d @%.3f m origin=(%.3f, %.3f)' % (shape[1], shape[0], res,
                                                            meta['origin'][0], meta['origin'][1]))
    print('[route] free=%.1f m²  可行驶(clearance>=%.2f m)=%.1f m²  '
          '起点连通域=%.1f m²（这就是本次覆盖的上限）'
          % (free.sum() * res * res, args.clearance, mask.sum() * res * res, area))
    if args.report_only:
        return {'area_m2': area}

    # ---- 割草机航点：沿 x 分 lane，每 lane 内在 y 上按 step 布点
    rows, cols = np.nonzero(comp)
    rr0, rr1, cc0, cc1 = rows.min(), rows.max(), cols.min(), cols.max()
    step = max(1, int(round(args.goal_step / res)))
    lane = max(1, int(round(args.lane_spacing / res)))
    goals = []
    lane_no = 0
    for c in range(cc0, cc1 + 1, lane):
        sel = comp[:, c]
        rws = np.nonzero(sel)[0]
        if len(rws) == 0:
            continue
        # 该列上连续的可行驶段（一段一条，避免把被障碍切开的列连成一条）
        breaks = np.nonzero(np.diff(rws) > 1)[0]
        segs = np.split(rws, breaks + 1)
        order = segs if lane_no % 2 == 0 else segs[::-1]
        for seg in order:
            for r in seg[::step]:
                goals.append((int(r), int(c)))
        lane_no += 1
    print('[route] 割草机栅格目标 %d 个（lane 间距 %.2f m，lane 内步长 %.2f m）'
          % (len(goals), args.lane_spacing, args.goal_step))

    # ---- 目标排序：nn = 贪心最近邻（对斜长/非凸区域比严格 lane 序短得多）
    if args.order == 'nn':
        rem = list(goals)
        cur = (r0, c0)
        ordered = []
        while rem:
            k = min(range(len(rem)),
                    key=lambda i: (rem[i][0] - cur[0]) ** 2 + (rem[i][1] - cur[1]) ** 2)
            cur = rem.pop(k)
            ordered.append(cur)
        goals = ordered
        print('[route] 目标按贪心最近邻重排（--order nn）')

    # ---- 相邻目标之间 A* 连成合法折线
    poly = [(r0, c0)]
    cur = (r0, c0)
    legs = []
    failed = 0
    for g in goals:
        if g == cur:
            continue
        p = astar(comp, cur, g)
        if p is None:
            failed += 1
            continue
        legs.append(len(p))
        poly.extend(p[1:])
        cur = g
    print('[route] A* 连接：成功 %d 段 / 失败 %d 段（失败=同一连通域内不应发生）'
          % (len(legs), failed))
    xy = [rc_to_xy(r, c, meta, shape) for r, c in poly]
    xy = rdp(xy, args.simplify)
    # 航点间距太密对控制器没意义，再按最小间距抽一次（保留首尾）
    minsep = args.min_waypoint_sep
    out = [xy[0]]
    for p in xy[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= minsep:
            out.append(p)
    out.append(xy[-1])
    path_len = float(sum(math.hypot(out[i + 1][0] - out[i][0], out[i + 1][1] - out[i][1])
                         for i in range(len(out) - 1)))
    goal_xy = [rc_to_xy(r, c, meta, shape) for r, c in goals]
    bbox = [min(p[0] for p in goal_xy), min(p[1] for p in goal_xy),
            max(p[0] for p in goal_xy), max(p[1] for p in goal_xy)]
    print('[route] 折线航点 %d 个（抽稀 eps=%.2f m，最小间距 %.2f m）  路线总长 %.1f m'
          % (len(out), args.simplify, minsep, path_len))
    print('[route] 航点 bbox x[%.2f, %.2f] y[%.2f, %.2f]  ⇒ 与起点(0,0)的距离上限 %.1f m'
          % (bbox[0], bbox[2], bbox[1], bbox[3],
             max(math.hypot(p[0], p[1]) for p in goal_xy)))
    if args.speed > 0:
        print('[route] 按 %.2f m/s 估计纯行驶 %.0f s（不含转向/原地旋转）'
              % (args.speed, path_len / args.speed))

    route = {
        'frame': 'map',
        'map_yaml': os.path.abspath(args.map_yaml),
        'resolution': res,
        'origin': meta['origin'],
        'clearance': args.clearance,
        'start': [rc_to_xy(r0, c0, meta, shape)[0], rc_to_xy(r0, c0, meta, shape)[1]],
        'waypoints': [[round(x, 3), round(y, 3)] for x, y in out],
        'goals': [[round(x, 3), round(y, 3)] for x, y in goal_xy],
        'stats': {
            'area_m2': round(area, 2),
            'reachable_mask_m2': round(float(mask.sum()) * res * res, 2),
            'free_m2': round(float(free.sum()) * res * res, 2),
            'path_len_m': round(path_len, 2),
            'n_waypoints': len(out),
            'n_goals': len(goal_xy),
            'bbox': [round(v, 3) for v in bbox],
            'est_drive_s': round(path_len / args.speed, 1) if args.speed > 0 else None,
        },
    }
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w') as f:
            json.dump(route, f, indent=1)
        print('[route] 写出 %s' % args.out)
    if args.png:
        save_png(img_path, meta, comp, out, args.png, args.clearance)
    return route


def save_png(img_path, meta, comp, waypoints, path, clearance):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    img = read_pgm(img_path)
    h, w = img.shape
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    ext = [ox, ox + w * res, oy, oy + h * res]
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.imshow(img, cmap='gray', origin='upper', extent=ext, vmin=0, vmax=255)
    overlay = np.zeros((h, w, 4))
    overlay[comp] = [0.1, 0.6, 1.0, 0.35]
    ax.imshow(overlay, origin='upper', extent=ext)
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]
    ax.plot(xs, ys, '-', color='red', lw=1.2, label='route %.1f m' % sum(
        math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1)))
    ax.plot(xs, ys, 'o', color='red', ms=2)
    ax.plot([0], [0], 'g*', ms=18, label='spawn (0,0)')
    ax.set_title('coverage route (clearance %.2f m, drivable component %.1f m2, %.1f m path)'
                 % (clearance, float(comp.sum()) * res * res,
                    sum(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
                        for i in range(len(xs) - 1))))
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print('[route] 俯视图 %s' % path)


def main():
    ap = argparse.ArgumentParser(description='从先验 2D 栅格图生成"不跨半场"的覆盖路线')
    ap.add_argument('--map-yaml', required=True, help='map_server 风格的 .yaml（同目录找 .pgm）')
    ap.add_argument('--out', help='路线 JSON 输出路径')
    ap.add_argument('--png', help='可选：俯视图 PNG 输出路径')
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0],
                    help='起点（map 系，默认 0 0 = 出生点）')
    ap.add_argument('--clearance', type=float, default=0.26,
                    help='可行驶所需的"离最近障碍"距离（m）；车半径 0.22 + 余量')
    ap.add_argument('--lane-spacing', type=float, default=2.0, help='割草机 lane 间距（m）')
    ap.add_argument('--goal-step', type=float, default=1.0, help='lane 内目标点步长（m）')
    ap.add_argument('--simplify', type=float, default=0.12, help='Douglas-Peucker 容差（m）')
    ap.add_argument('--min-waypoint-sep', type=float, default=0.5, help='输出航点最小间距（m）')
    ap.add_argument('--speed', type=float, default=0.4, help='仅用于估算耗时（m/s）')
    ap.add_argument('--order', choices=['nn', 'lawnmower'], default='nn',
                    help='目标点排序：nn=贪心最近邻（默认，路径更短）| lawnmower=严格 lane 序')
    ap.add_argument('--report-only', action='store_true', help='只打印面积统计，不规划路线')
    ap.add_argument('--dry-run', action='store_true', help='规划但不写文件')
    args = ap.parse_args()
    if args.dry_run:
        args.out = None
        args.png = None
    r = build_route(args)
    return 0 if r else 1


if __name__ == '__main__':
    sys.exit(main())
