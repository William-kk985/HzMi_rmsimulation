#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gicp A/B 的**离线**指标计算 + 画图（只读 watch.jsonl / watch.json / status.txt）。

一次跑的全部判据（docs/gicp_divergence_and_jitter.md §4 的表就是它算出来的）：

  A. `map→odom`（GICP 的修正量）
     · **逐次更新步长**（只在 map→odom 真的换了一条新戳时算）：平移/yaw 的 p50/p90/p99/max
       —— "发散"的最小单位；fitness 好看但这一步很大 = 修正量在自传播。
     · 累计修正路程 Σ|Δ| 与净位移 |end-start|（净/路程比 ≈ 1 说明单调跑偏，≈0 说明来回来抖）。
     · 相对"发目标前基线"的最大偏离（xy、z）。
  B. 融合位姿 `map→base_link`（= map→odom ∘ odom→base_link，nav2 真正看到的量）
     · 相邻采样差分的平移/yaw 分位数（抖动）—— 采样 ≥50 Hz。
  C. 真值（/odom_ground_truth）：目标到达后车**真的**在动吗
     · 位置/朝向的峰峰值与逐样本差分 p95（"四处抖动"的物理证据）。
  D. `/cmd_vel`（平滑前的 nav 指令）：目标后的活跃度（非零占比、均值、角速度换向次数）。
  E. 一致性：融合位姿相对真值的**相对漂移**（用起点对齐，避开 map 系与真值系的未知常量偏移）
     · p50/p95/max；以及偏离 1.0 m 的首个时刻。
  F. 发散是否发生 / 是否被抓住 / 在哪一步（按"逐次更新步长 > 阈值"的首帧）。
  G. gicp 健康信号：fitness 的 p50/p95/max、~/converged 掉 false 的次数、[status] 行里的采纳计数。

用法：
    python3 tools/scripts/localization/gicp_ab_report.py --run a_baseline=.tmp_gicp_ab/out/a_baseline \
        --run b_gate=.tmp_gicp_ab/out/b_gate --outdir .tmp_gicp_ab/report [--plot]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

import numpy as np

MISSING = None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pct(v, q):
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], dtype=float)
    return float("nan") if v.size == 0 else float(np.percentile(v, q))


def stats(v, name, out, unit=""):
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], dtype=float)
    if v.size == 0:
        out[name + "_n"] = 0
        return
    out[name + "_n"] = int(v.size)
    out[name + "_p50"] = round(float(np.percentile(v, 50)), 5)
    out[name + "_p90"] = round(float(np.percentile(v, 90)), 5)
    out[name + "_p95"] = round(float(np.percentile(v, 95)), 5)
    out[name + "_p99"] = round(float(np.percentile(v, 99)), 5)
    out[name + "_max"] = round(float(v.max()), 5)
    out[name + "_mean"] = round(float(v.mean()), 5)
    out[name + "_unit"] = unit


def load_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def phase_of(rows, events):
    """把每一行归到 phase：base / drive / post / stop（用事件里的墙钟时刻切）。"""
    ev = {e["kind"]: e["w"] for e in events}
    t_sent = ev.get("goal_sent")
    t_res = ev.get("goal_result")
    t_pend = ev.get("post_goal_window_end")
    t_zero = ev.get("zero_twist_done")
    ph = []
    # 无目标跑（--no-goal 静止基线）：没有 goal_sent/goal_result ⇒ 用 baseline 事件当分界，
    # 之后整段算 "post"（口径与"目标后窗口"一致：车静止、只看漂移与抖动）。
    t_base = ev.get("baseline")
    for r in rows:
        w = r["w"]
        if t_sent is None and t_base is not None:
            ph.append("post" if w >= t_base else "base")
            continue
        if t_sent is not None and w < t_sent:
            ph.append("base")
        elif t_res is not None and (t_sent is None or w >= t_sent) and w < t_res:
            ph.append("drive")
        elif t_pend is not None and t_res is not None and t_res <= w < t_pend:
            ph.append("post")
        elif t_zero is not None:
            ph.append("stop")
        elif t_res is not None and w >= t_res:
            ph.append("post")
        else:
            ph.append("base")
    return ph


def analyze(tag, run_dir):
    rows = load_rows(os.path.join(run_dir, "watch.jsonl"))
    summ = {}
    sp = os.path.join(run_dir, "watch.json")
    if os.path.exists(sp):
        summ = json.load(open(sp))
    events = summ.get("events", [])
    ph = phase_of(rows, events)
    out = {"tag": tag, "dir": run_dir, "rows": len(rows),
           "goal_result": summ.get("result", {}), "fails": summ.get("fails", []),
           "rtf": summ.get("rtf"), "cnt": summ.get("cnt", {}), "events": events}

    # ---------------- A. map→odom 逐次更新步长 ----------------
    upd_i = [i for i in range(len(rows)) if rows[i].get("upd") and rows[i]["mo"] is not None]
    dt_upd, dxy, dyaw = [], [], []
    for a, b in zip(upd_i, upd_i[1:]):
        ra, rb = rows[a], rows[b]
        dt_upd.append(rb["t"] - ra["t"])
        dxy.append(math.hypot(rb["mo"][0] - ra["mo"][0], rb["mo"][1] - ra["mo"][1]))
        dyaw.append(abs(wrap(rb["mo"][3] - ra["mo"][3])))
    out["upd_n"] = len(upd_i)
    stats(dxy, "mo_step_xy", out, "m")
    stats([math.degrees(x) for x in dyaw], "mo_step_yaw", out, "deg")
    stats(dt_upd, "mo_upd_dt", out, "s")
    if upd_i:
        out["mo_upd_hz"] = round(len(upd_i) / max(1e-6, rows[-1]["t"] - rows[0]["t"]), 3)
    # **统一口径**：把 map→odom 重采样到固定 10 Hz 再算步长 —— 未平滑时它等于"每次采纳的步长"，
    # 平滑后（发布值 50 Hz 连续变化）也要能直接对照 ⇒ A/B 表里用这一组。
    for p in ("base", "drive", "post"):
        ii = [i for i in range(len(rows)) if ph[i] == p and rows[i]["mo"] is not None]
        if len(ii) < 3:
            continue
        t = np.array([rows[i]["t"] for i in ii])
        xy = np.array([[rows[i]["mo"][0], rows[i]["mo"][1]] for i in ii])
        yaw = np.array([rows[i]["mo"][3] for i in ii])
        grid = np.arange(t[0], t[-1], 0.1)
        if grid.size < 3:
            continue
        gx = np.interp(grid, t, xy[:, 0]); gy = np.interp(grid, t, xy[:, 1])
        gyaw = np.interp(grid, t, np.unwrap(yaw))
        st = np.hypot(np.diff(gx), np.diff(gy))
        sy = np.abs(np.degrees(np.diff(gyaw)))
        out["mo_step10_%s_p50" % p] = round(float(np.percentile(st, 50)), 5)
        out["mo_step10_%s_p95" % p] = round(float(np.percentile(st, 95)), 5)
        out["mo_step10_%s_max" % p] = round(float(st.max()), 5)
        out["mo_step10_%s_yaw_p95" % p] = round(float(np.percentile(sy, 95)), 4)
        out["mo_step10_%s_yaw_max" % p] = round(float(sy.max()), 4)
        out["mo_step10_%s_sum" % p] = round(float(st.sum()), 4)
        out["mo_net_%s" % p] = round(float(math.hypot(gx[-1] - gx[0], gy[-1] - gy[0])), 4)

    # 按 phase 的更新统计
    for p in ("base", "drive", "post"):
        idx = [i for i in upd_i if ph[i] == p]
        s = []
        for a, b in zip(idx, idx[1:]):
            s.append(math.hypot(rows[b]["mo"][0] - rows[a]["mo"][0], rows[b]["mo"][1] - rows[a]["mo"][1]))
        if s:
            out["mo_step_xy_%s_p95" % p] = round(pct(s, 95), 5)
            out["mo_step_xy_%s_max" % p] = round(float(np.max(s)), 5)
            out["mo_step_xy_%s_sum" % p] = round(float(np.sum(s)), 4)
            out["mo_upd_%s_n" % p] = len(idx)
    # 逐次更新 > 阈值的首帧（发散被抓/发生的时刻）——阈值 = 0.08 m / 2°
    thr_xy, thr_yaw = 0.08, 2.0
    div = None
    for a, b in zip(upd_i, upd_i[1:]):
        ra, rb = rows[a], rows[b]
        d = math.hypot(rb["mo"][0] - ra["mo"][0], rb["mo"][1] - ra["mo"][1])
        dy = math.degrees(abs(wrap(rb["mo"][3] - ra["mo"][3])))
        if d > thr_xy or dy > thr_yaw:
            div = {"sim_t": rb["t"], "wall_s": rb["w"], "phase": ph[b], "dxy": round(d, 4),
                   "dyaw_deg": round(dy, 3), "step_index": upd_i.index(b),
                   "n_upd_before": upd_i.index(b), "mo_before": ra["mo"], "mo_after": rb["mo"],
                   "fit_after": rb.get("fit")}
            break
    out["first_big_step"] = div
    # 相对基线的最大偏离
    base = rows[upd_i[0]]["mo"] if upd_i else None
    if base is not None:
        dev_xy = [math.hypot(r["mo"][0] - base[0], r["mo"][1] - base[1])
                  for r in rows if r["mo"] is not None]
        dev_z = [abs(r["mo"][2] - base[2]) for r in rows if r["mo"] is not None]
        dev_yaw = [abs(math.degrees(wrap(r["mo"][3] - base[3]))) for r in rows if r["mo"] is not None]
        stats(dev_xy, "mo_dev_xy", out, "m")
        stats(dev_z, "mo_dev_z", out, "m")
        stats(dev_yaw, "mo_dev_yaw", out, "deg")
        out["mo_start"] = [round(x, 4) for x in base]
        last = [r for r in rows if r["mo"] is not None][-1]["mo"]
        out["mo_end"] = [round(x, 4) for x in last]
        # 首次偏离 > 1 m 的时刻
        for r in rows:
            if r["mo"] is not None and math.hypot(r["mo"][0] - base[0], r["mo"][1] - base[1]) > 1.0:
                out["first_dev_gt_1m"] = {"sim_t": r["t"], "wall_s": r["w"], "phase": ph[rows.index(r)],
                                          "shift_xy": round(math.hypot(r["mo"][0] - base[0],
                                                                       r["mo"][1] - base[1]), 3),
                                          "mo": r["mo"], "fit": r.get("fit")}
                break

    # ---------------- B. 融合位姿抖动 ----------------
    fused = []
    for r in rows:
        if r["mo"] is None or r["ob"] is None:
            fused.append(None)
            continue
        x0, y0, yaw0 = r["mo"][0], r["mo"][1], r["mo"][3]
        x1, y1, yaw1 = r["ob"][0], r["ob"][1], r["ob"][3]
        c, s = math.cos(yaw0), math.sin(yaw0)
        fused.append((x0 + c * x1 - s * y1, y0 + s * x1 + c * y1, r["mo"][2] + r["ob"][2], wrap(yaw0 + yaw1)))
    for p in ("drive", "post"):
        d, dy = [], []
        prev = None
        for i, f in enumerate(fused):
            if f is None or ph[i] != p:
                prev = None
                continue
            if prev is not None and rows[i]["t"] > rows[i - 1]["t"]:
                d.append(math.hypot(f[0] - prev[0], f[1] - prev[1]))
                dy.append(abs(math.degrees(wrap(f[3] - prev[3]))))
            prev = f
        stats(d, "fused_step_xy_%s" % p, out, "m")
        stats(dy, "fused_step_yaw_%s" % p, out, "deg")
        # 高频抖动：对"位姿-时间"序列做二阶差分（速度的变化 = 抖动），按采样间隔归一
        if len(d) > 4:
            v = np.array(d) / max(1e-6, 0.02)
            out["fused_step_xy_%s_std" % p] = round(float(np.std(v)), 5)

    # ---------------- C. 真值抖动（目标后车真的在动？） ----------------
    for p in ("base", "drive", "post", "stop"):
        gi = [i for i in range(len(rows)) if rows[i]["gt"] is not None and ph[i] == p]
        if len(gi) < 3:
            continue
        g = np.array([[rows[i]["gt"][0], rows[i]["gt"][1], rows[i]["gt"][3]] for i in gi])
        out["gt_%s_p2p_xy" % p] = round(float(math.hypot(g[:, 0].max() - g[:, 0].min(),
                                                         g[:, 1].max() - g[:, 1].min())), 4)
        out["gt_%s_p2p_yaw_deg" % p] = round(float(np.degrees(g[:, 2].max() - g[:, 2].min())), 3)
        dg = np.hypot(np.diff(g[:, 0]), np.diff(g[:, 1]))
        out["gt_%s_pathlen" % p] = round(float(dg.sum()), 3)
        if len(dg):
            out["gt_%s_step_p95" % p] = round(float(np.percentile(dg, 95)), 5)
        tw = np.array([[rows[i]["gt"][4], rows[i]["gt"][5], rows[i]["gt"][6]] for i in gi])
        out["gt_%s_vmax" % p] = round(float(np.abs(tw[:, 0]).max()), 4)
        out["gt_%s_wmax" % p] = round(float(np.abs(tw[:, 2]).max()), 4)

    # ---------------- D. /cmd_vel 活跃度 ----------------
    for p in ("drive", "post"):
        ci = [i for i in range(len(rows)) if ph[i] == p]
        if not ci:
            continue
        v = np.array([abs(rows[i]["cmd"][0]) for i in ci])
        w = np.array([rows[i]["cmd"][1] for i in ci])
        nav_w = np.array([rows[i]["cmd"][3] for i in ci])
        ch_w = np.array([rows[i]["cmd"][5] for i in ci])
        act = ((v > 0.02) | (np.abs(w) > 0.02)).mean()
        flip = int(np.sum(np.abs(np.diff(np.sign(w))) > 0))
        out["cmd_%s_active_frac" % p] = round(float(act), 4)
        out["cmd_%s_v_mean" % p] = round(float(v.mean()), 4)
        out["cmd_%s_v_max" % p] = round(float(v.max()), 4)
        out["cmd_%s_w_absmean" % p] = round(float(np.abs(w).mean()), 4)
        out["cmd_%s_w_flips" % p] = flip
        out["cmd_%s_navw_absmean" % p] = round(float(np.abs(nav_w).mean()), 4)
        out["cmd_%s_chassis_w_absmax" % p] = round(float(np.abs(ch_w).max()), 3)

    # ---------------- E. 与真值的相对漂移 ----------------
    f0 = next((f for f in fused if f is not None), None)
    g0 = next((r["gt"] for r in rows if r["gt"] is not None), None)
    if f0 is not None and g0 is not None:
        for p in ("drive", "post"):
            e = []
            for i, f in enumerate(fused):
                if f is None or ph[i] != p or rows[i]["gt"] is None:
                    continue
                g = rows[i]["gt"]
                e.append(math.hypot((f[0] - f0[0]) - (g[0] - g0[0]), (f[1] - f0[1]) - (g[1] - g0[1])))
            stats(e, "drift_vs_gt_%s" % p, out, "m")
            if e:
                idx = [k for k, x in enumerate(e) if x > 1.0]
                if idx:
                    out["drift_vs_gt_%s_first_gt1m" % p] = True

    # ---------------- F. 健康信号 ----------------
    for p in ("base", "drive", "post"):
        fi = [rows[i]["fit"] for i in range(len(rows)) if ph[i] == p and rows[i]["fit"] is not None]
        stats(fi, "fitness_%s" % p, out, "m2")
        ci = [rows[i]["conv"] for i in range(len(rows)) if ph[i] == p and rows[i]["conv"] is not None]
        if ci:
            out["converged_true_frac_%s" % p] = round(float(np.mean([1.0 if x else 0.0 for x in ci])), 5)

    # ---------------- G. [status] 行里的采纳计数 ----------------
    st = os.path.join(run_dir, "status.txt")
    acc = []
    if os.path.exists(st):
        txt = open(st, errors="ignore").read()
        for m in re.finditer(r"\[status\][^\n]*", txt):
            line = m.group(0)
            mm = re.search(r"采纳 (\d+)/(\d+) 帧", line)
            if mm:
                acc.append((int(mm.group(1)), int(mm.group(2))))
        out["status_lines"] = len(acc)
        if acc:
            out["accept_first"] = acc[0]
            out["accept_last"] = acc[-1]
            out["accept_rate_overall"] = round(acc[-1][0] / max(1, acc[-1][1]), 4)
        out["reject_warn_lines"] = len(re.findall(r"未被采纳", txt))
        out["lost_lines"] = len(re.findall(r"定位已失效", txt))
        out["no_tf_lines"] = len(re.findall(r"不发 TF|尚无 map→odom", txt))
        out["divergence_lines"] = len(re.findall(r"发散|拒绝|门限|gate", txt))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True, metavar="TAG=DIR")
    ap.add_argument("--outdir", default=".tmp_gicp_ab/report")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    runs = []
    for spec in a.run:
        tag, d = spec.split("=", 1)
        runs.append((tag, d))
    res = []
    for tag, d in runs:
        if not os.path.exists(os.path.join(d, "watch.jsonl")):
            print("[report] ⚠️ 跳过 %s（没有 watch.jsonl）" % tag, file=sys.stderr)
            continue
        r = analyze(tag, d)
        res.append(r)
        with open(os.path.join(a.outdir, "%s.metrics.json" % tag), "w") as f:
            json.dump(r, f, ensure_ascii=False, indent=2)

    # ---- 控制台表 ----
    cols = [("map→odom @10Hz 步长 行驶 p95/max (m)", "mo_step10_drive_p95", "mo_step10_drive_max", 4),
            ("map→odom @10Hz 步长 目标后 p95/max (m)", "mo_step10_post_p95", "mo_step10_post_max", 4),
            ("map→odom @10Hz 累计路程 行驶/目标后 (m)", "mo_step10_drive_sum", "mo_step10_post_sum", 3),
            ("map→odom 逐次步长 xy p95/max (m)", "mo_step_xy_p95", "mo_step_xy_max", 4),
            ("map→odom 逐次 yaw p95/max (°)", "mo_step_yaw_p95", "mo_step_yaw_max", 3),
            ("更新率 (Hz)", "mo_upd_hz", None, 2),
            ("map→odom 相对基线最大偏离 xy/z (m)", "mo_dev_xy_max", "mo_dev_z_max", 3),
            ("map→odom 终点 (x,y,z,yaw)", None, None, None),
            ("fused 抖动 drive xy p95/max (m)", "fused_step_xy_drive_p95", "fused_step_xy_drive_max", 4),
            ("fused 抖动 post  xy p95/max (m)", "fused_step_xy_post_p95", "fused_step_xy_post_max", 4),
            ("fused 抖动 post  yaw p95/max (°)", "fused_step_yaw_post_p95", "fused_step_yaw_post_max", 3),
            ("真值 post 峰峰值 xy/yaw", "gt_post_p2p_xy", "gt_post_p2p_yaw_deg", 3),
            ("真值 post 路程 (m)", "gt_post_pathlen", None, 3),
            ("cmd_vel post 活跃占比", "cmd_post_active_frac", None, 3),
            ("cmd_vel post |w| 均值 / 换向", "cmd_post_w_absmean", "cmd_post_w_flips", 3),
            ("漂移(vs真值) post p95/max (m)", "drift_vs_gt_post_p95", "drift_vs_gt_post_max", 3),
            ("fitness post p95/max (m²)", "fitness_post_p95", "fitness_post_max", 5)]
    if all(r.get("goal_result", {}).get("status") is None for r in res):
        print("  （无目标/静止跑：下面用 post 段 = 静置后的整段记录）")
    for name, k1, k2, nd in cols:
        cells = []
        for r in res:
            if k1 is None and k2 is None:
                cells.append(str(r.get("mo_end", "--")))
            else:
                v1 = r.get(k1, "--")
                v2 = "" if k2 is None else "/%s" % r.get(k2, "--")
                if isinstance(v1, float):
                    v1 = ("%%.%df" % nd) % v1
                if isinstance(v2, str) and len(v2) > 1 and k2 is not None and isinstance(r.get(k2), float):
                    v2 = ("/%%.%df" % nd) % r[k2]
                cells.append("%s%s" % (v1, v2))
        print("  %-38s | %s" % (name, " | ".join("%-22s" % c for c in cells)))

    with open(os.path.join(a.outdir, "summary.json"), "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print("[report] %d 跑 → %s/summary.json" % (len(res), a.outdir))

    # ---- 画图 ----
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for tag, d in runs:
            p = os.path.join(d, "watch.jsonl")
            if not os.path.exists(p):
                continue
            rows = load_rows(p)
            summ = json.load(open(os.path.join(d, "watch.json"))) if os.path.exists(
                os.path.join(d, "watch.json")) else {}
            ev = {e["kind"]: e["w"] for e in summ.get("events", [])}
            t = np.array([r["t"] for r in rows])
            mo = np.array([r["mo"][:4] if r["mo"] else [np.nan] * 4 for r in rows])
            gt = np.array([r["gt"][:3] if r["gt"] else [np.nan] * 3 for r in rows])
            fit = np.array([r["fit"] if r["fit"] is not None else np.nan for r in rows])
            fig, ax = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
            ax[0][0].plot(t, mo[:, 0], label="map→odom x")
            ax[0][0].plot(t, mo[:, 1], label="map→odom y")
            ax[0][0].plot(t, mo[:, 2], label="map→odom z")
            ax[0][0].set_ylabel("m"); ax[0][0].legend(); ax[0][0].set_title("map→odom（GICP 修正量）")
            ax[0][1].plot(t, np.degrees(mo[:, 3]), color="tab:red")
            ax[0][1].set_ylabel("deg"); ax[0][1].set_title("map→odom yaw")
            ax[1][0].plot(t, gt[:, 0], label="gt x"); ax[1][0].plot(t, gt[:, 1], label="gt y")
            ax[1][0].set_ylabel("m"); ax[1][0].legend(); ax[1][0].set_title("/odom_ground_truth（真值）")
            ax[1][1].plot(t, np.degrees(gt[:, 2]), color="tab:green")
            ax[1][1].set_ylabel("deg"); ax[1][1].set_title("真值 yaw")
            ax[2][0].plot(t, np.array([abs(r["cmd"][0]) for r in rows]), label="|cmd_vel.linear.x|")
            ax[2][0].plot(t, np.array([abs(r["cmd"][1]) for r in rows]), label="|cmd_vel.angular.z|")
            ax[2][0].set_ylabel("SI"); ax[2][0].legend(); ax[2][0].set_title("/cmd_vel")
            ax[2][1].plot(t, fit, color="tab:purple")
            ax[2][1].set_ylabel("m²"); ax[2][1].set_title("~/fitness_score")
            for k, v in ev.items():
                for row in ax:
                    for x in row:
                        x.axvline(v, color="k", ls=":", lw=0.7)
            for x in ax[-1]:
                x.set_xlabel("sim t (s)")
            fig.suptitle("gicp A/B 跑 %s（虚线 = 事件：目标发出/结果/录制结束）" % tag)
            fig.tight_layout()
            fig.savefig(os.path.join(a.outdir, "%s_traj.png" % tag), dpi=110)
            print("[report] 图 → %s/%s_traj.png" % (a.outdir, tag))
        # A/B 对照图：map→odom xy 轨迹
        if len(runs) > 1:
            fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
            for tag, d in runs:
                p = os.path.join(d, "watch.jsonl")
                if not os.path.exists(p):
                    continue
                rows = [r for r in load_rows(p) if r["mo"]]
                if not rows:
                    continue
                mo = np.array([r["mo"][:2] for r in rows])
                t = np.array([r["t"] for r in rows])
                # 逐帧更新点（新戳）才是"修正量真正动了"的时刻：图上把它标出来，摊平的段是 50 Hz 重复
                ax[0].plot(mo[:, 0], mo[:, 1], label=tag, lw=1.0)
                ax[1].plot(t, np.hypot(mo[:, 0] - mo[0, 0], mo[:, 1] - mo[0, 1]), label=tag, lw=1.0)
            ax[0].set_xlabel("map→odom x (m)"); ax[0].set_ylabel("map→odom y (m)")
            ax[0].set_title("map→odom 轨迹（A/B 对照）"); ax[0].legend()
            ax[1].set_xlabel("sim t (s)"); ax[1].set_ylabel("|map→odom - start| (m)")
            ax[1].set_title("相对起点的偏离"); ax[1].legend()
            fig.tight_layout()
            fig.savefig(os.path.join(a.outdir, "ab_map_odom.png"), dpi=110)
            print("[report] 图 → %s/ab_map_odom.png" % a.outdir)
            # 静止跑（无目标）再加一张**放大**图：修正量抖动是 mm/cm 级，放在上面那张里看不见
            def _static(d):
                sp = os.path.join(d, "watch.json")
                if not os.path.exists(sp):
                    return False
                ev = {e["kind"] for e in json.load(open(sp)).get("events", [])}
                return "goal_sent" not in ev
            if all(_static(d) for _, d in runs):
                fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
                for tag, d in runs:
                    p2 = os.path.join(d, "watch.jsonl")
                    if not os.path.exists(p2):
                        continue
                    rows2 = [r for r in load_rows(p2) if r["mo"]]
                    if not rows2:
                        continue
                    mo2 = np.array([r["mo"][:2] for r in rows2])
                    t2 = np.array([r["t"] for r in rows2])
                    ax[0].plot(mo2[:, 0], mo2[:, 1], label=tag, lw=0.8)
                    ax[1].plot(t2, np.hypot(mo2[:, 0] - mo2[:, 0].mean(),
                                            mo2[:, 1] - mo2[:, 1].mean()), label=tag, lw=0.8)
                lim = max(0.02, max(abs(float(np.concatenate([ax[0].get_xlim(), ax[0].get_ylim()])))
                                    for _ in [0]) * 1.05) if False else None
                ax[0].set_title("map→odom 抖动（放大：静止 150 s，逐帧更新点）")
                ax[0].set_xlabel("map→odom x (m)"); ax[0].set_ylabel("map→odom y (m)")
                ax[0].legend(); ax[0].grid(alpha=0.3)
                ax[1].set_title("|map→odom − 均值|（时间轴）")
                ax[1].set_xlabel("sim t (s)"); ax[1].set_ylabel("m"); ax[1].legend()
                ax[1].grid(alpha=0.3)
                ax[0].set_xlim(float(np.mean([r["mo"][0] for _, d in runs
                                              for r in load_rows(os.path.join(d, "watch.jsonl"))
                                              if r["mo"]])) - 0.03,
                               float(np.mean([r["mo"][0] for _, d in runs
                                              for r in load_rows(os.path.join(d, "watch.jsonl"))
                                              if r["mo"]])) + 0.03)
                ax[0].set_ylim(-0.03, 0.03)
                fig.tight_layout()
                fig.savefig(os.path.join(a.outdir, "ab_static_zoom.png"), dpi=110)
                print("[report] 图 → %s/ab_static_zoom.png" % a.outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
