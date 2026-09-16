#!/usr/bin/env python3
"""栅格地图可达性/目标点检查工具（仿真测试选点用）。

为什么需要它：nav2 的全局规划会把「障碍 + 机器人内切半径(robot_radius)膨胀带」
视为不可通行。若地图上有一道细墙（甚至只有 1 像素宽的虚线），膨胀后可能把整条
走廊封死 → `planner_server: failed to generate a valid path` → BT 反复跑恢复行为
（表现为 /cmd_vel 只有 BackUp 的 -0.05 和 Spin）。选目标点前先跑一遍本工具即可避免。

用法:
  # 1) 检查某个目标点从起点是否可达（最常用）
  ./check_map_reachable.py --map src/rm_nav_bringup/map/RMUL2026.yaml --start 4.3 3.35 --goal 6.0 3.4

  # 2) 让工具推荐一批「可达且离墙够远」的目标点（按距离分档）
  ./check_map_reachable.py --map src/rm_nav_bringup/map/RMUL2026.yaml --start 4.3 3.35

  # 3) 换机器人半径 / 想看的距离范围
  ./check_map_reachable.py --map ... --start ... --robot-radius 0.3 --min-dist 1.0 --max-dist 5.0

判读要点:
  * 「不可达」= 该点与起点不在同一连通域 → nav2 一定规划失败，别用它测导航；
  * 连通域被切成多块，通常说明地图上有细墙/虚线把场地分隔（地图资产问题）；
  * 本工具对像素的 occupancy 判定与 nav2 map_io 的 trinary 模式一致。
"""

import argparse
import collections
import math
import os
import sys


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # 解析 P5/P2 头（magic / width height / maxval）
    fields = []
    i = 0
    while len(fields) < 4:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and not data[j : j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    magic = fields[0]
    width, height, maxval = int(fields[1]), int(fields[2]), int(fields[3])
    i += 1  # 跳过一个空白
    if magic == b"P5":
        pixels = list(data[i : i + width * height])
    elif magic == b"P2":
        pixels = [int(t) for t in data[i:].split()[: width * height]]
    else:
        raise ValueError("只支持 P5/P2 格式，当前为 %r" % magic)
    if maxval != 255:  # 归一化到 0~255
        pixels = [round(p * 255.0 / maxval) for p in pixels]
    return width, height, pixels


def main():
    ap = argparse.ArgumentParser(description="栅格地图可达性/目标点检查")
    ap.add_argument("--map", required=True, help="map yaml 路径（如 src/rm_nav_bringup/map/RMUL2026.yaml）")
    ap.add_argument("--start", nargs=2, type=float, required=True, metavar=("X", "Y"),
                    help="机器人起点（map 系，米）。AMCL 初值给对时应等于 AMCL 估计位姿")
    ap.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="要检查是否可达的目标点")
    ap.add_argument("--robot-radius", type=float, default=0.2,
                    help="nav2 global_costmap 的 robot_radius（内切半径，默认 0.2）")
    ap.add_argument("--clearance", type=float, default=0.5, help="推荐目标点时要求的离墙距离（默认 0.5m）")
    ap.add_argument("--min-dist", type=float, default=1.0, help="推荐目标点的最近距离（默认 1.0m）")
    ap.add_argument("--max-dist", type=float, default=4.5, help="推荐目标点的最远距离（默认 4.5m）")
    ap.add_argument("--ascii", action="store_true", help="额外打印可达区域示意图")
    a = ap.parse_args()

    # ---- 读 map yaml ----
    try:
        import yaml
    except ImportError:
        sys.exit("需要 python3-yaml：sudo apt install python3-yaml")
    with open(a.map) as f:
        meta = yaml.safe_load(f)
    img = meta["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(a.map)), img)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.25))
    W, H, px = load_pgm(img)
    print("地图 %s\n  图像 %s  %dx%d  resolution=%.3f  origin=(%.2f, %.2f)  negate=%d  occ_th=%.2f free_th=%.2f"
          % (a.map, os.path.basename(img), W, H, res, ox, oy, negate, occ_th, free_th))

    # occupancy 判定：与 nav2 map_io 的 trinary 模式一致
    #   occ=(255-v)/255（negate=0）；> occ_th 障碍；< free_th 自由；否则未知(=可通行)
    def is_occ(v):
        o = (v if negate else 255 - v) / 255.0
        return o > occ_th

    occupied = [[is_occ(px[r * W + c]) for c in range(W)] for r in range(H)]

    def to_rc(x, y):
        c = int((x - ox) / res)
        r = (H - 1) - int((y - oy) / res)
        return r, c

    def to_xy(r, c):
        return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

    R = int(round(a.robot_radius / res))
    blocked = [[False] * W for _ in range(H)]
    n_occ = 0
    for r in range(H):
        for c in range(W):
            if occupied[r][c]:
                n_occ += 1
                for dr in range(-R, R + 1):
                    for dc in range(-R, R + 1):
                        if dr * dr + dc * dc <= R * R:
                            rr, cc = r + dr, c + dc
                            if 0 <= rr < H and 0 <= cc < W:
                                blocked[rr][cc] = True
    print("  障碍格 %d，按 robot_radius=%.2fm（%d 格）膨胀后不可通行 %d 格"
          % (n_occ, a.robot_radius, R, sum(map(sum, blocked))))

    # ---- 连通域（四连通；比 nav2 的八连通更保守，保守判定"不可达"更安全）----
    label = [[-1] * W for _ in range(H)]
    comp_size = []
    for r0 in range(H):
        for c0 in range(W):
            if blocked[r0][c0] or label[r0][c0] >= 0:
                continue
            cid = len(comp_size)
            n = 0
            dq = collections.deque([(r0, c0)])
            label[r0][c0] = cid
            while dq:
                r, c = dq.popleft()
                n += 1
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W and not blocked[rr][cc] and label[rr][cc] < 0:
                        label[rr][cc] = cid
                        dq.append((rr, cc))
            comp_size.append(n)
    comp_size_sorted = sorted(range(len(comp_size)), key=lambda i: -comp_size[i])
    sr, sc = to_rc(*a.start)
    if not (0 <= sr < H and 0 <= sc < W):
        sys.exit("起点 (%.2f, %.2f) 在地图范围外！" % tuple(a.start))
    if blocked[sr][sc]:
        print("  ⚠️ 起点本身落在「障碍/膨胀带」里 → nav2 会报 start is in lethal space，必须换初值或修地图")
        sid = -1
    else:
        sid = label[sr][sc]
        print("  起点 (%.2f, %.2f) 所在连通域 #%d：%d 格（占全部可通行 %d 格的 %.1f%%）"
              % (a.start[0], a.start[1], sid, comp_size[sid],
                 sum(comp_size), 100.0 * comp_size[sid] / max(1, sum(comp_size))))
    print("  连通域共 %d 块，最大几块：" % len(comp_size)
          + ", ".join("#%d=%d格" % (i, comp_size[i]) for i in comp_size_sorted[:5]))
    if len(comp_size) > 1:
        print("  ⚠️ 地图被切成多块：如果目标点不在起点这块，nav2 必然 `failed to generate a valid path`。"
              "地图上细墙/虚线 1 像素宽也足够切断走廊。")

    # ---- 指定目标点检查 ----
    if a.goal:
        gr, gc = to_rc(*a.goal)
        if not (0 <= gr < H and 0 <= gc < W):
            print("\n目标 (%.2f, %.2f): 在地图范围外" % tuple(a.goal))
        elif blocked[gr][gc]:
            print("\n目标 (%.2f, %.2f): ❌ 落在障碍/膨胀带内（planner 会认为 goal 不可达）" % tuple(a.goal))
        elif sid >= 0 and label[gr][gc] == sid:
            print("\n目标 (%.2f, %.2f): ✅ 与起点同一连通域（%.2f m）"
                  % (a.goal[0], a.goal[1], math.dist(a.start, a.goal)))
        else:
            print("\n目标 (%.2f, %.2f): ❌ 与起点**不同连通域**（%.2f m）→ nav2 一定规划失败"
                  % (a.goal[0], a.goal[1], math.dist(a.start, a.goal)))

    # ---- 推荐可达目标点 ----
    if sid >= 0:
        def clearance(x, y):
            r0, c0 = to_rc(x, y)
            k = int(math.ceil(1.0 / res))
            best = 1.0
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    rr, cc = r0 + dr, c0 + dc
                    if 0 <= rr < H and 0 <= cc < W and occupied[rr][cc]:
                        best = min(best, math.hypot(dr, dc) * res)
            return best

        cands = []
        for r in range(H):
            for c in range(W):
                if label[r][c] != sid:
                    continue
                x, y = to_xy(r, c)
                d = math.dist((x, y), a.start)
                if a.min_dist <= d <= a.max_dist and clearance(x, y) >= a.clearance:
                    cands.append((round(d, 2), round(x, 2), round(y, 2), round(clearance(x, y), 2)))
        print("\n★ 可达候选目标点（同一连通域、离墙≥%.2fm、距离 %.1f~%.1f m）"
              % (a.clearance, a.min_dist, a.max_dist))
        if not cands:
            print("   无 → 可放宽 --clearance 或检查起点是否被膨胀带围住")
        picked, bins = [], {}
        for d, x, y, cl in cands:
            b = int(d / 0.7)  # 每 0.7m 一档，每档取离墙最远的一个，避免刷屏
            if b not in bins or cl > bins[b][3]:
                bins[b] = (d, x, y, cl)
        for d, x, y, cl in [bins[b] for b in sorted(bins)][:8]:
            print("   (%5.2f, %5.2f)  距起点 %.2f m  离最近墙 %.2f m" % (x, y, d, cl))
        if len(bins) > 8:
            print("   ...（共 %d 个距离档，已截断）" % len(bins))

    # ---- 示意图 ----
    if a.ascii:
        step = 3
        print("\n可达区域（R=起点, #=可达, o=障碍/膨胀带, 空=其它连通域）:")
        for r in range(0, H, step):
            line = ""
            for c in range(0, W, step):
                ch = "#" if (sid >= 0 and label[r][c] == sid) else ("o" if blocked[r][c] else " ")
                x, y = to_xy(r, c)
                if abs(x - a.start[0]) < res * 2.5 and abs(y - a.start[1]) < res * 2.5:
                    ch = "R"
                if a.goal and abs(x - a.goal[0]) < res * 2.5 and abs(y - a.goal[1]) < res * 2.5:
                    ch = "G"
                line += ch
            print("   " + line)


if __name__ == "__main__":
    main()
