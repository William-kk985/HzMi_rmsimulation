#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 nav_clearance_probe.py 的产物读成"报告可直接引用"的数字 + 因果时间轴。

用法：
    python3 tools/scripts/regress/analyze_clearance.py .tmp_clear/out/base [more dirs...]
    python3 tools/scripts/regress/analyze_clearance.py --table .tmp_clear/out/*   # A/B 汇总表

输出（stdout）：
    · 余量分布（全局/局部代价地图、融合位姿/真值位姿；p50/p05/p01/min；足迹边缘余量）
    · 接触/被推事件（时间、位置、残差）与"贴墙时间占比"
    · 定位事件（map→odom 发布停顿段、定位误差、fitness）
    · 规划质量（每次 /plan 的长度与最小余量）
    · 因果时间轴（把上面几条按仿真时刻排序）
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np

R = 0.22          # robot_radius：足迹边缘余量 = 到致命格的距离 − R


def load(d):
    rows = []
    p = os.path.join(d, "samples.jsonl")
    if not os.path.exists(p):
        return rows, None, ""
    with open(p) as f:
        for ln in f:
            try:
                rows.append(json.loads(ln))
            except Exception:      # noqa: BLE001
                pass
    s = json.load(open(os.path.join(d, "summary.json"))) if os.path.exists(
        os.path.join(d, "summary.json")) else None
    g = open(os.path.join(d, "gate.txt")).read() if os.path.exists(
        os.path.join(d, "gate.txt")) else ""
    return rows, s, g


def col(rows, k):
    return np.asarray([r[k] for r in rows if isinstance(r.get(k), (int, float))], dtype=float)


def dist(name, v):
    if v.size == 0:
        return "n=0"
    return ("p50=%.3f p05=%.3f p01=%.3f min=%.3f max=%.3f n=%d"
            % (np.percentile(v, 50), np.percentile(v, 5), np.percentile(v, 1), v.min(), v.max(),
               v.size))


def moving_of(rows, thr=0.005):
    """移动段 = 真值**实际位移** > thr 米/样本（0.1 s）⇒ >0.05 m/s。

    ⚠️ 不用 /odom_ground_truth 的 twist：planar_move 发的 twist 是**指令值**（不是实测速度），
    被墙挡住时它照样报 1 m/s ⇒ 会把"贴着墙推"的样本误判成移动。
    """
    out = [r for r in rows if isinstance(r.get("step"), (int, float)) and r["step"] > thr]
    if not out:      # 没有 step（旧数据）时退回 twist
        return [r for r in rows if math.hypot(r.get("gvx", 0.0), r.get("gvy", 0.0)) > 0.05]
    return out


def gate_events(g):
    """gate.txt 里带绝对墙钟时间戳的 GICP 事件。"""
    ev = []
    for ln in g.splitlines():
        m = re.match(r"^\[[\w\-\.]+\]\s*\[(\d+\.\d+)\]\s*\[(\w+)\]\s*\[([\w\./]+)\]:\s*(.*)$", ln)
        if not m:
            continue
        ts, sev, node, msg = float(m.group(1)), m.group(2), m.group(3), m.group(4)
        kind = None
        if "GICP 本帧未被采纳" in msg:
            kind = "reject"
        elif "连续" in msg and "帧被拒绝" in msg:
            kind = "lost"
        elif "[status]" in msg and "定位失效" in msg:
            kind = "lost_status"
        elif "[status]" in msg:
            kind = "status"
        if kind:
            ev.append({"ts": ts, "sev": sev, "kind": kind, "msg": msg[:220]})
    return ev


def report(d, verbose=True):
    rows, s, g = load(d)
    if not rows or s is None:
        print("== %s：无数据" % d)
        return None
    mv = moving_of(rows)
    t0 = rows[0]["t"]
    tgt = s.get("t_wall0")
    print("=" * 100)
    print("## %s  样本 %d 行（移动段 %d）| 仿真时长 %.1fs | 目标 %s | 结局 status=%s | "
          "recovery=%s | 真值行程 %.2f m（指令 %.2f m）"
          % (os.path.basename(d.rstrip('/')), len(rows), len(mv), rows[-1]["t"] - t0,
             s.get("goal"), s.get("goal_status"), (s.get("fb") or {}).get("recov"),
             s.get("travel_gt", 0), s.get("travel_cmd", 0)))

    def line(label, key, sub="", margin=False):
        v = col(mv if sub != "all" else rows, key)
        if v.size == 0:
            print("  %-34s n=0" % label)
            return
        extra = ("  边缘余量: " + dist("", v - R) + "  frac<=0=%.3f"
                 % float(np.mean(v - R <= 0.0))) if margin else ""
        print("  %-34s %s%s" % (label, dist("", v), extra))

    print("-- 余量（移动段，米；到最近格的距离）")
    line("global→致命格(真值位姿)", "g_lethal_t", margin=True)
    line("global→致命格(融合位姿)", "g_lethal_f", margin=True)
    line("global→内切带(真值位姿)", "g_insc_t")
    line("global→膨胀带(真值位姿)", "g_infl_t")
    line("local→致命格(融合位姿)", "l_lethal", margin=True)
    line("local→内切带(融合位姿)", "l_insc")
    line("/scan 最小距离(真实世界)", "scan_min")
    line("地图乐观度(地图−雷达)", "map_opt")
    print("  贴墙时间占比（移动段）：")
    for k, thr in (("g_lethal_t", R), ("g_lethal_t", R - 0.05), ("g_lethal_t", 0.16),
                   ("l_lethal", R), ("scan_min", 0.30), ("scan_min", 0.22), ("scan_min", 0.16)):
        v = col(mv, k)
        if v.size:
            print("    %-14s < %.2f m : %.1f%%" % (k, thr, 100.0 * float(np.mean(v < thr))))
    mv_le = col(mv, "loc_err")
    if mv_le.size:
        print("-- 定位误差（融合位姿 vs 真值，移动段）: " + dist("", mv_le))
    print("-- 规划（每次 /plan）")
    pl = s.get("plans", [])
    if pl:
        ml = np.asarray([p["min_lethal"] for p in pl if isinstance(p.get("min_lethal"), float)])
        mi = np.asarray([p["min_insc"] for p in pl if isinstance(p.get("min_insc"), float)])
        ln = np.asarray([p["len"] for p in pl], dtype=float)
        print("  计划 %d 条 | 长度 p50=%.2f m | 最小到致命格 %s | 最小到内切带 %s"
              % (len(pl), np.median(ln), dist("", ml) if ml.size else "n=0",
                 dist("", mi) if mi.size else "n=0"))
        if ml.size:
            print("  计划余量(减 R): min=%.3f  frac<=0=%.3f" % (ml.min() - R,
                                                             float(np.mean(ml - R <= 0))))
    print("-- 接触 / 被推（真值位移 vs 指令位移残差，planar_move 是运动学插件）")
    for kind in ("blocked", "pushed"):
        ev = s.get(kind, [])
        print("  %s: %d 次" % (kind, len(ev)))
        for e in ev[:14]:
            print("    t=%.1f..%.1f s  res_max=%.3f m  n=%d  位置(m)=(%s,%s)"
                  % (e["t0"] - t0, e["t1"] - t0, e["res_max"], e["n"], e.get("x"), e.get("y")))
    print("-- 定位事件")
    print("  map→odom 发布停顿段（>0.5 s）: %d 段，最大陈旧 %.2f s"
          % (len(s.get("mo_stale_runs", [])), s.get("mo_age_max", 0)))
    for r in s.get("mo_stale_runs", [])[:8]:
        print("    t=%.1f..%.1f s（陈旧 %.2f s）" % (r["t0"] - t0, r["t1"] - t0, r["age"]))
    if s.get("loc_err"):
        print("  全程定位误差: " + json.dumps(s["loc_err"]))
    if s.get("fitness"):
        print("  fitness: " + json.dumps(s["fitness"]))
    ge = gate_events(g)
    cnt = {}
    for e in ge:
        cnt[e["kind"]] = cnt.get(e["kind"], 0) + 1
    print("  GICP 日志事件计数: %s" % cnt)
    lost = [e for e in ge if e["kind"] in ("lost", "lost_status")]
    if lost and tgt:
        print("  首次「判失效」墙钟=%.1f（相对探针起点 %.1f s）" % (lost[0]["ts"],
                                                              lost[0]["ts"] - tgt))
    rej = [e for e in ge if e["kind"] == "reject"]
    if rej and tgt:
        print("  首次「未被采纳」墙钟=%.1f（相对 %.1f s）" % (rej[0]["ts"], rej[0]["ts"] - tgt))
    if verbose:
        print("-- 因果时间轴（仿真时刻；接触/推挤、map→odom 停顿、定位误差>0.3、recovery 变化）")
        tl = []
        for kind in ("blocked", "pushed"):
            for e in s.get(kind, []):
                tl.append((e["t0"] - t0, "%s res=%.2f @(%.1f,%.1f)"
                           % (kind, e["res_max"], e.get("x") or 0, e.get("y") or 0)))
        for r in s.get("mo_stale_runs", []):
            tl.append((r["t0"] - t0, "map→odom 停顿开始（age %.2f）" % r["age"]))
        prev_rec = -1
        for r in rows:
            if isinstance(r.get("recov"), int) and r["recov"] != prev_rec:
                prev_rec = r["recov"]
                if prev_rec:
                    tl.append((r["t"] - t0, "recovery #%d" % prev_rec))
            if isinstance(r.get("loc_err"), float) and r["loc_err"] > 0.3:
                if not any(abs(tl[-1][0] - (r["t"] - t0)) < 3.0 for _ in [0]) if tl else True:
                    pass
        le_big = [r for r in rows if isinstance(r.get("loc_err"), float) and r["loc_err"] > 0.3]
        if le_big:
            tl.append((le_big[0]["t"] - t0, "定位误差首次 >0.3 m (%.2f)" % le_big[0]["loc_err"]))
        if tgt:
            for e in ge:
                if e["kind"] in ("reject", "lost", "lost_status"):
                    tl.append((e["ts"] - tgt - (rows[0]["wall_abs"] - tgt) + 0.0,
                               "GICP %s" % e["kind"]))
        for e in s.get("events", []):
            if e["kind"] in ("goal_sent", "goal_result", "drive_timeout"):
                tl.append((e["wall"] - rows[0]["wall"], "%s %s"
                           % (e["kind"], {k: v for k, v in e.items() if k not in ("wall", "sim", "kind")})))
        for t, txt in sorted(tl)[:40]:
            print("   %7.1f s  %s" % (t, txt))
    return s


def table(dirs):
    print("| tag | 结局 | 行程/指令 | 计划余量min | g_clear(pt)p50/p05/min | 边缘frac<=0 | "
          "scan<0.22 | 接触(block/push) | aod停顿 | recov | locerr p95 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for d in dirs:
        rows, s, g = load(d)
        if not rows or s is None:
            continue
        mv = moving_of(rows)
        t0 = rows[0]["t"]
        gl = col(mv, "g_lethal_t")
        sc = col(mv, "scan_min")
        pl = s.get("plans", [])
        ml = [p["min_lethal"] for p in pl if isinstance(p.get("min_lethal"), float)]
        le = col(mv, "loc_err")
        print("| %s | %s | %.1f/%.1f | %s | %s | %s | %s | %d/%d | %d | %s | %s |" % (
            s.get("tag"), s.get("goal_status"), s.get("travel_gt", 0), s.get("travel_cmd", 0),
            ("%.2f" % min(ml)) if ml else "-",
            ("%.2f/%.2f/%.2f" % (np.percentile(gl, 50), np.percentile(gl, 5), gl.min()))
            if gl.size else "-",
            ("%.2f" % float(np.mean(gl < R))) if gl.size else "-",
            ("%.1f%%" % (100 * float(np.mean(sc < 0.22)))) if sc.size else "-",
            len(s.get("blocked", [])), len(s.get("pushed", [])),
            len(s.get("mo_stale_runs", [])), (s.get("fb") or {}).get("recov"),
            ("%.2f" % np.percentile(le, 95)) if le.size else "-"))


def plot(d, out=None, mapdir="src/rm_nav_bringup/map", world="RMUC2026"):
    """把执行轨迹画在静态地图上，颜色 = 到最近致命格的余量（红=贴墙）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    import yaml
    rows, s, g = load(d)
    if not rows:
        print("== %s：无数据" % d)
        return
    my = yaml.safe_load(open(os.path.join(mapdir, world + ".yaml")))
    img = np.array(Image.open(os.path.join(mapdir, my["image"])))
    res = my["resolution"]; ox, oy, _ = my["origin"]
    H, W = img.shape
    plt.figure(figsize=(13, 7.5), dpi=110)
    plt.imshow(img, cmap="gray", origin="upper",
               extent=[ox, ox + W * res, oy, oy + H * res])
    xs = [r.get("mx") for r in rows]
    ys = [r.get("my") for r in rows]
    cs = [r.get("g_lethal_t") for r in rows]
    ok = [(x, y, c) for x, y, c in zip(xs, ys, cs) if isinstance(c, float) and x is not None]
    if ok:
        x2, y2, c2 = zip(*ok)
        sc = plt.scatter(x2, y2, c=c2, cmap="jet", s=9, vmin=0.15, vmax=0.9, zorder=3)
        plt.colorbar(sc, label="clearance to nearest lethal cell (m)")
    fx = [r.get("fx") for r in rows]
    fy = [r.get("fy") for r in rows]
    plt.plot([x for x in fx if x is not None], [y for y in fy if y is not None], "b-", lw=1.0,
             alpha=0.8, label="fused pose (map frame, GICP)")
    for ev, c, lab in ((s.get("blocked", []), "red", "blocked"), (s.get("pushed", []), "orange", "pushed")):
        for e in ev:
            plt.plot(e["x"], e["y"], "o", color=c, ms=9, mfc="none", mew=2,
                     label=lab if lab not in plt.gca().get_legend_handles_labels()[1] else None)
    plt.plot(rows[0].get("mx"), rows[0].get("my"), "gs", ms=11, label="start")
    if s.get("goal"):
        plt.plot(s["goal"][0], s["goal"][1], "k*", ms=16, label="goal")
    plt.legend(loc="upper right", fontsize=9)
    plt.title("%s：执行轨迹余量（块=%s，目标 %s，结局 %s）"
              % (s.get("tag"), os.path.basename(d.rstrip('/')), s.get("goal"), s.get("goal_status")))
    plt.xlabel("map x (m)"); plt.ylabel("map y (m)"); plt.tight_layout()
    out = out or os.path.join(d, "traj_clearance.png")
    plt.savefig(out)
    print("[plot] %s" % out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    dirs = []
    for d in a.dirs:
        dirs.extend(sorted(glob.glob(d)) if any(c in d for c in "*?[") else [d])
    if a.table:
        table(dirs)
        return
    for d in dirs:
        if a.plot:
            try:
                plot(d)
            except Exception as ex:      # noqa: BLE001
                print("[plot] 失败：%s" % ex)
        report(d, verbose=not a.quiet)


if __name__ == "__main__":
    main()
