#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场地网格（STL） vs 2D 先验图 vs 执行轨迹：把"地图看不见的低矮几何"量化出来。

**为什么需要它**（docs/path_clearance_and_contact.md §2.2）：
  2D 图是"3D 点云切一条高度带 + 去地面"得到的 ⇒ **0.06~0.30 m 的低矮结构/坡道会被当地面删掉**。
  于是规划器/控制器对它们完全盲，而车体（轮 r=0.06 m、底盘底面离地 0.06 m）会实撞上去。
  本脚本把仿真场地网格（毫米单位、z 抬升 1.6413436 m，见 world 文件头注）栅格化成"每格最高几何高度"，
  再和 2D 图占用 / `/scan` / 真值轨迹逐点对账，给出**可引用的数字**。

用法：
    # ① 全局对账（静态图判自由但场地高度 > 阈值的格有多少、在哪）
    python3 tools/scripts/regress/field_mesh_vs_map.py --world RMUC2026 \\
        --stl src/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/meshes/RMUC2026.stl \\
        --map src/rm_nav_bringup/map/RMUC2026.yaml --cell 0.05

    # ② 带上执行轨迹（probe 的 samples.jsonl）：逐点的场地高度 + 撞击时刻附近的高度分布
    python3 tools/scripts/regress/field_mesh_vs_map.py --world RMUC2026 \\
        --traj .tmp_clear/out/base2/samples.jsonl --impact-imu 30

约定：
  · STL 默认二进制、毫米；`--stl-scale 0.001`、`--stl-lift 1.6413436`（world 文件头注给的变换）。
  · "自由" = 不是占用格（`occupied_thresh` 按 map yaml，trinary）。
  · 只用**水平/倾斜**三角形填高度（竖直面的 XY 投影面积为 0，会被跳过）⇒
    本脚本给的是"可站立面高度"，**不是**"障碍高度"；竖直墙面请用 2D 图本身。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct

import numpy as np


def read_stl(path: str) -> np.ndarray:
    """二进制 STL → (n,3,3) float32（单位与文件一致，本场地是毫米）。"""
    raw = open(path, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    if len(raw) != 84 + n * 50:
        raise SystemExit("不是二进制 STL（长度对不上）：%s" % path)
    arr = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return arr[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def load_map(map_yaml: str):
    import yaml
    from PIL import Image
    m = yaml.safe_load(open(map_yaml))
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), m["image"])))
    res = m["resolution"]
    ox, oy, _ = m["origin"]
    occ = img.astype(float) < 255 * (1 - m["occupied_thresh"])
    return m, img, occ, res, float(ox), float(oy)


def height_grid(V: np.ndarray, occ_shape, res, ox, oy, spawn_x, spawn_y,
                scale=0.001, lift=1.6413436, floor=0.0, cell=None):
    """把网格栅格化成"每格最高**可站立面**高度（相对地面）"。

    V：STL 三角形（原始单位）；返回 (rel_height, abs_height)。
    只在三角形 XY 投影有面积时填（竖直面跳过）。
    """
    H, W = occ_shape
    grid_res = cell or res
    hh = np.full((H, W), -99.0, np.float32)
    V = V * scale
    for i in range(len(V)):
        x = V[i, :, 0] - spawn_x
        y = V[i, :, 1] - spawn_y
        z = V[i, :, 2] + lift
        x0, x1, y0, y1 = x.min(), x.max(), y.min(), y.max()
        d = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(d) < 1e-12:
            continue
        c0 = int((x0 - ox) / grid_res)
        c1 = int((x1 - ox) / grid_res)
        r0 = H - 1 - int((y1 - oy) / grid_res)
        r1 = H - 1 - int((y0 - oy) / grid_res)
        c0, c1 = max(0, c0), min(W - 1, c1)
        r0, r1 = max(0, r0), min(H - 1, r1)
        if r1 < r0 or c1 < c0:
            continue
        cc, rr = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
        px = ox + (cc + 0.5) * grid_res
        # ⚠️ OccupancyGrid 的行号从**下**往上数（y 递增），不是图像的行号 ⇒ 必须 H-1-r 折回来。
        #    这里写错过一次：栅格图整体上下镜像，"轨迹处高度恒为 0"就是这么来的。
        py = oy + (H - 1 - rr + 0.5) * grid_res
        a = ((y[1] - y[2]) * (px - x[2]) + (x[2] - x[1]) * (py - y[2])) / d
        b = ((y[2] - y[0]) * (px - x[2]) + (x[0] - x[2]) * (py - y[2])) / d
        c = 1 - a - b
        inside = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
        if not inside.any():
            continue
        zz = a * z[0] + b * z[1] + c * z[2]
        sub = hh[r0:r1 + 1, c0:c1 + 1]
        np.maximum(sub, np.where(inside, zz - floor, -99.0).astype(np.float32), out=sub)
    return hh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="RMUC2026")
    ap.add_argument("--stl", default=None,
                    help="默认 src/rm_simulation/hzmi_rm_simulation/world/<world>_world/meshes/<world>.stl")
    ap.add_argument("--map", default=None, help="默认 src/rm_nav_bringup/map/<world>.yaml")
    ap.add_argument("--traj", default=None, help="nav_clearance_probe.py 的 samples.jsonl")
    ap.add_argument("--cell", type=float, default=0.05, help="高度图栅格（默认=图分辨率）")
    ap.add_argument("--stl-scale", type=float, default=0.001)
    ap.add_argument("--stl-lift", type=float, default=1.6413436)
    ap.add_argument("--floor", type=float, default=0.0, help="地面绝对高度（抬升后）")
    ap.add_argument("--spawn", default="10.925,2.524",
                    help="出生点世界坐标（map 原点对应的世界 XY；实测 /odom_ground_truth 静置值）")
    ap.add_argument("--impact-imu", type=float, default=30.0)
    ap.add_argument("--out", default=None, help="把高度图存成 .npy（可选）")
    a = ap.parse_args()

    stl = a.stl or ("src/rm_simulation/hzmi_rm_simulation/world/%s_world/meshes/%s.stl"
                    % (a.world, a.world))
    map_yaml = a.map or ("src/rm_nav_bringup/map/%s.yaml" % a.world)
    sx, sy = [float(v) for v in a.spawn.split(",")]

    V = read_stl(stl)
    m, img, occ, res, ox, oy = load_map(map_yaml)
    H, W = occ.shape
    print("[mesh] %s：三角形 %d，bbox(mm) x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]"
          % (stl, len(V), V[:, :, 0].min(), V[:, :, 0].max(), V[:, :, 1].min(),
             V[:, :, 1].max(), V[:, :, 2].min(), V[:, :, 2].max()))
    print("[map ] %s：%dx%d @ %.3f m，origin=(%.2f, %.2f)，占用格 %d"
          % (map_yaml, W, H, res, ox, oy, int(occ.sum())))
    rel = height_grid(V, occ.shape, res, ox, oy, sx, sy, a.stl_scale, a.stl_lift, a.floor,
                      cell=a.cell)
    if a.out:
        np.save(a.out, rel)
        print("[out ] 高度图 → %s" % a.out)
    free = ~occ
    for t in (0.06, 0.10, 0.15, 0.20, 0.275, 0.30, 0.40):
        s = free & (rel > t)
        print("  [对账] 静态图判『自由』而场地高度 > %.3f m：%6d 格（占自由区 %.2f%%）"
              % (t, int(s.sum()), 100.0 * s.sum() / max(1, free.sum())))

    if not a.traj:
        return
    rows = [json.loads(l) for l in open(a.traj) if l.strip()]
    t0 = rows[0].get("t", 0.0)

    def cell_of(x, y):
        c = int((x - ox) / a.cell)
        r = H - 1 - int((y - oy) / a.cell)
        return (r, c) if (0 <= c < W and 0 <= r < H) else None

    mov = [r for r in rows if isinstance(r.get("step"), (int, float)) and r["step"] > 0.005]
    hv, occv, clr = [], [], []
    for r in mov:
        if not isinstance(r.get("mx"), float):
            continue
        rc = cell_of(r["mx"], r["my"])
        if rc is None:
            continue
        hv.append(float(rel[rc]))
        occv.append(bool(occ[rc]))
        if isinstance(r.get("g_lethal_t"), float):
            clr.append(r["g_lethal_t"])
    hv = np.asarray(hv)
    if hv.size:
        print("\n[轨迹] %s：移动样本 %d（其中 %d 个落在地图『自由』格上）"
              % (os.path.basename(a.traj), hv.size, int(np.sum(~np.asarray(occv)))))
        print("  轨迹处场地高度：p50=%.3f p95=%.3f max=%.3f；>0.06 占 %.1f%%、>0.10 占 %.1f%%、>0.20 占 %.1f%%"
              % (np.percentile(hv, 50), np.percentile(hv, 95), hv.max(),
                 100 * float((hv > 0.06).mean()), 100 * float((hv > 0.10).mean()),
                 100 * float((hv > 0.20).mean())))
        if clr:
            print("  同期静态图余量：p50=%.3f min=%.3f（>%.2f ⇒ 图上完全看不出那里有东西）"
                  % (np.percentile(clr, 50), min(clr), 0.10))
    # 撞击时刻邻域的高度分布（撞击 = IMU 模长超过阈值）
    imp = [r for r in rows if isinstance(r.get("imu_a"), (int, float)) and r["imu_a"] > a.impact_imu]
    print("\n[撞击] IMU > %.0f m/s² 的样本 %d 个" % (a.impact_imu, len(imp)))
    for r in imp[:6]:
        rc = cell_of(r["mx"], r["my"]) if isinstance(r.get("mx"), float) else None
        if rc is None:
            continue
        r0, c0 = max(0, rc[0] - 6), max(0, rc[1] - 6)
        win = rel[r0:rc[0] + 7, c0:rc[1] + 7]
        win = win[win > -90]
        print("  t=%6.1f map=(%6.2f,%6.2f) IMU=%6.1f | 该处场地高度 %.3f | 周围 0.6 m 内高度 p50=%.3f p95=%.3f max=%.3f | 地图占用=%s"
              % (r["t"] - t0, r["mx"], r["my"], r["imu_a"], float(rel[rc]),
                 float(np.percentile(win, 50)) if win.size else float("nan"),
                 float(np.percentile(win, 95)) if win.size else float("nan"),
                 float(win.max()) if win.size else float("nan"), bool(occ[rc])))


if __name__ == "__main__":
    main()
