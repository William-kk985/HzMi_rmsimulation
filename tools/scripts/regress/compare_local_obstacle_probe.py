#!/usr/bin/env python3
"""把 `local_obstacle_probe.py` 的采样汇总成一张 A/B 对照表（+ 可选：把栅格快照画成 ASCII）。

不重启任何东西、不改配置：只读探针写出的 `.probe.jsonl`（`--dump-grid` 的 `.grid.npz` 可选）。

用法：
  python3 tools/scripts/regress/compare_local_obstacle_probe.py \\
      scan=.tmp_cache/stvl_local/scan.probe.jsonl \\
      stvl=.tmp_cache/stvl_local/stvl.probe.jsonl \\
      scan_wall=.tmp_cache/stvl_local/scan_wall.probe.jsonl \\
      stvl_wall=.tmp_cache/stvl_local/stvl_wall.probe.jsonl

  # 额外把"离障碍最近那一刻"的局部栅格画成 ASCII（看墙上的标记是连续线还是点状）
  python3 tools/scripts/regress/compare_local_obstacle_probe.py \\
      scan_wall=.tmp_cache/stvl_local/scan_wall.probe.jsonl \\
      stvl_wall=.tmp_cache/stvl_local/stvl_wall.probe.jsonl \\
      --grid scan_wall=.tmp_cache/stvl_local/scan_wall.grid.npz \\
      --grid stvl_wall=.tmp_cache/stvl_local/stvl_wall.grid.npz

列的含义（全部来自"只订阅"的探针，不改栈的行为）：
  lc_/gc_ 前缀 = local/global costmap；`lethal` = 栅格值 254（nav2 LETHAL_OBSTACLE，未膨胀）；
  `*_00_05 / *_05_10` = 机器人中心 0~0.5 m / 0.5~1.0 m 环带内的真障碍格数；
  `min_r` = 到最近真障碍的平面距离（米）；`behind` = 车后 120°~180° 扇区 0.3~1.5 m；
  `scan_n_valid` = /scan 有效回波数；`cloud_n*` = /segmentation/obstacle 点数（含落在 p2l
  高度带 z∈[-1.0,0.1] 内/外的拆分）。

退出码：0=正常；1=一个文件都没读到。
"""
from __future__ import annotations

import json
import sys

import numpy as np


def load(path):
    rows, summary = [], {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            if "summary" in d:
                summary = d["summary"]
            else:
                rows.append(d)
    return rows, summary


def col(rows, key):
    v = [r[key] for r in rows if isinstance(r.get(key), (int, float))]
    return np.asarray(v, dtype=float) if v else np.asarray([])


def q(v, p):
    return None if v.size == 0 else float(np.percentile(v, p))


def stats(rows, summary):
    out = {"n": len(rows), "span_s": None}
    if rows:
        t = [r["t_wall"] for r in rows if "t_wall" in r]
        out["span_s"] = round(max(t) - min(t), 1) if t else None
    for key, label in (("lc_lethal_00_05", "lc#00-05"), ("lc_lethal_05_10", "lc#05-10"),
                       ("lc_lethal_behind_03_15", "lc#behind"), ("lc_lethal_n_1m", "lc#1m"),
                       ("gc_lethal_n_1m", "gc#1m"), ("gc_lethal_00_05", "gc#00-05")):
        v = col(rows, key)
        out[label + "_p50"] = q(v, 50)
        out[label + "_max"] = None if v.size == 0 else float(v.max())
    for key, label in (("lc_lethal_min_r", "lc_min_r"), ("gc_lethal_min_r", "gc_min_r"),
                       ("scan_min_d_robot", "scan_min_d"), ("cloud_min_d_robot", "cloud_min_d")):
        v = col(rows, key)
        out[label + "_min"] = None if v.size == 0 else float(v.min())
        out[label + "_p10"] = q(v, 10)
        out[label + "_p50"] = q(v, 50)
    v = col(rows, "scan_n_05")
    out["scan#00-05_max"] = None if v.size == 0 else float(v.max())
    v = col(rows, "cloud_n_05")
    out["cloud#00-05_max"] = None if v.size == 0 else float(v.max())
    v = col(rows, "cloud_n_05_out_scanband")
    out["cloud#00-05_outband_max"] = None if v.size == 0 else float(v.max())
    v = col(rows, "scan_n_valid")
    out["scan_n_valid_p50"] = q(v, 50)
    v = col(rows, "cloud_n")
    out["cloud_n_p50"] = q(v, 50)
    out["msgs"] = summary.get("msgs")
    out["sample_failures"] = summary.get("sample_failures")
    return out


ROWS = [
    ("lc#00-05 (0~0.5m 环带真障碍格, p50/max)", "lc#00-05_p50", "lc#00-05_max"),
    ("lc#05-10 (0.5~1.0m, p50/max)", "lc#05-10_p50", "lc#05-10_max"),
    ("lc#behind (车后扇区, p50/max)", "lc#behind_p50", "lc#behind_max"),
    ("lc_min_r 到最近障碍 (min/p10/p50)", "lc_min_r_min", "lc_min_r_p10", "lc_min_r_p50"),
    ("gc#1m (全局对照, p50/max)", "gc#1m_p50", "gc#1m_max"),
    ("scan 有效回波 (p50)", "scan_n_valid_p50"),
    ("cloud 点数 (p50)", "cloud_n_p50"),
    ("scan 0~0.5m 回波 (max)", "scan#00-05_max"),
    ("cloud 0~0.5m 点 (max)", "cloud#00-05_max"),
    ("cloud 0~0.5m 且**在 p2l 高度带外** (max)", "cloud#00-05_outband_max"),
]


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return ("%.3f" % v).rstrip("0").rstrip(".") if abs(v) < 1000 else "%.0f" % v
    return str(v)


def ascii_grid(npz_path, half=1.1, step=2):
    d = np.load(npz_path)
    key = "best_grid" if "best_grid" in d else "last_grid"
    pose = d["best_pose"] if key == "best_grid" else d["last_pose"]
    g = d[key]
    res = 0.02  # local_costmap resolution（params 里写死；栅格本身不自带分辨率）
    rx, ry = float(pose[0]), float(pose[1])
    n = int(half / res)
    # 栅格原点：机器人位姿是 odom 系，栅格索引需按栅格自身的原点换算。
    # costmap_raw 的 metadata.origin 没被存进 npz ⇒ 用"机器人所在格"反推：
    # 取栅格内所有 lethal 格的重心-机器人 的相对形状即可（下面按整幅找最近 lethal 行/列）。
    ys, xs = np.nonzero((g >= 254) & (g < 255))
    if xs.size == 0:
        print("   （%s：没有任何 254 真障碍格）" % npz_path)
        return
    print("   %s（%s，sim 时刻 %s）：真障碍格 %d 个"
          % (npz_path, key, fmt(float(d.get("best_stamp", d.get("last_stamp", 0)))),
             int(xs.size)))
    # 找到离机器人（栅格中心）最近的一行/列，沿该方向打印 lethal 分布 —— 看"点状"还是"线状"
    gy, gx = g.shape
    cy, cx = gy // 2, gx // 2          # rolling window ⇒ 机器人在正中心
    print("   ASCII（机器人=R，'#'=真障碍254，'+'=内切/膨胀 253，'.'=空，每字符 %.2f m）："
          % (res * step))
    for yy in range(cy + n, cy - n - 1, -step):
        line = ""
        for xx in range(cx - n, cx + n + 1, step):
            if not (0 <= yy < gy and 0 <= xx < gx):
                line += " "
                continue
            v = g[yy, xx]
            line += "R" if abs(yy - cy) < step and abs(xx - cx) < step else (
                "#" if v >= 254 and v < 255 else ("+" if v >= 253 else "."))
        print("   " + line)


def main():
    args, grids = [], {}
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--grid":
            k, v = argv[i + 1].split("=", 1)
            grids[k] = v
            i += 2
            continue
        if not a.startswith("--"):
            args.append(a)
        i += 1
    if not args:
        print(__doc__)
        return 1
    runs = {}
    for a in args:
        tag, path = a.split("=", 1)
        runs[tag] = load(path)
    tags = list(runs)
    print("| 指标 | " + " | ".join("`%s`" % t for t in tags) + " |")
    print("|---|" + "---|" * len(tags))
    print("| 采样数 / 时长(s) | " + " | ".join(
        "%s / %s" % (len(runs[t][0]), stats(*runs[t])["span_s"]) for t in tags) + " |")
    for label, *keys in ROWS:
        cells = []
        for t in tags:
            st = stats(*runs[t])
            cells.append(" / ".join(fmt(st.get(k)) for k in keys))
        print("| %s | %s |" % (label, " | ".join(cells)))
    print("| 探针收到的消息 | " + " | ".join(
        str(stats(*runs[t])["msgs"]) for t in tags) + " |")
    for t in tags:
        if t in grids:
            print()
            print("### %s 的栅格快照" % t)
            ascii_grid(grids[t])
    return 0


if __name__ == "__main__":
    sys.exit(main())
