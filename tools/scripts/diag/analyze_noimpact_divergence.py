#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""「没撞击却飘里程计」的**归因**分析器 + 限速 A/B 指标表（读 run_nav_lio_timeline.sh 的产物）。

两件事：
  ① 归因（`--timeline DIR`）：把一次跑的 50 Hz 时间轴切成"发散事件"，对**每个事件**
     给出因果排序与候选判定（①高角速度甩转 / ②慢速贴墙被挡 / ③滤波自身数值爆掉 / ④几何退化）。
     判定的**依据信号**（都在报告里逐条列出，不靠感觉）：
       · LIO 相对真值的误差 `lio_err`（用**起步静止窗口冻结**的一个常量刚体变换对齐 —— 全程沿用，
         绝不逐段重对齐，见 docs/lio_drift_diagnosis.md §3.3）；
       · `/odom` **逐帧**单步 `step` 与隐含速度 `spd`（数值爆掉的直接证据；50 Hz 行与逐消息事件都有）；
       · 指令 ω（三路：smoother / MPPI / 底盘）与**实测** ω（真值）—— ①的自变量；
       · IMU |a|（含重力 ⇒ 静止 ≈9.8）—— 区分"有撞击"与"没撞击"；
       · `/scan` 前向/后向最近回波 —— ②的直接观测；
       · 接触/被挡（底盘是运动学插件 ⇒ 真值位移 ≠ 指令位移只可能来自接触修正）；
       · 定位健康（`converged` / `fitness` / `map→odom` 停发）—— 谁先坏。
     分类规则（写在 `classify()` 里，改规则只改那一处）：
       · 事件前 4 s 内有 IMU>30 尖峰 ⇒ **②/撞击触发**（"没撞击"是错觉：撞击在几秒前）；
       · 前 4 s 内有 blocked/pushed，且 IMU 平（<15）⇒ **②慢速贴墙/被挡**；
       · 前 1 s 内指令或实测 |ω| 落在 0.8~1.9 rad/s 带内 ⇒ **①甩转**（并报告实测传递比）；
       · 以上都没有，且**车静止（真值 |v|<0.1）而 LIO 隐含速度 >1 m/s** ⇒ **③滤波数值爆掉**；
       · 事件窗口内前后向回波都 <0.8 m（平行墙/夹缝）⇒ 记 **④几何退化**（作为并因，不单独定罪）。
  ② A/B 表（多个 DIR）：结局/墙钟/里程、|v| p50/p90/max、|v|<0.6 时长、IMU 峰与撞击簇、
     LIO 发散（有/无、最大单帧步长、`lio_err` max）、GICP 事件、接触/被挡、恢复行为（次数+类型）、
     余量统计、RTF、限速器行为。

用法：
  python3 tools/scripts/diag/analyze_noimpact_divergence.py --timeline .tmp_lio_noimpact/out/nb1
  python3 tools/scripts/diag/analyze_noimpact_divergence.py .tmp_clear/out/v3base_validate .tmp_lio_noimpact/out/nb1
  python3 tools/scripts/diag/analyze_noimpact_divergence.py --compare A B C --json out.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import statistics
import sys

DIV_THR = 0.50          # ｜lio_err｜ 超过它 = 起一段"误差段"（米；0.3~0.5 是正常 LIO 噪声量级）
SEVERE_DIV = 1.00       # ｜lio_err｜ 超过它才算"真发散"（米）——报告里的 有/无 用这个门槛
IMU_IMPACT = 30.0       # IMU |a| 超过它 = 撞击/重磕（静止基线 ≈9.8）
IMU_CALM = 15.0         # 低于它 = 这一帧没有撞击
YAW_LO, YAW_HI = 0.8, 1.9   # 仓库实测的"甩转危险带"（rad/s，实测值）
STEP_THR = 0.15         # 单帧位移超过它 = 可疑（≈1.5 m/s @10 Hz / 15 m/s @100 Hz）
TGAP = 4.0              # 事件前回看窗口（秒，仿真时间）

RE_LIMIT = re.compile(r'\[slope_speed\] limit=([\d.]+) m/s \((\d+)% of ([\d.]+)\)(.*?)\| why=(\S+)')
# 日志行长这样（launch 里带 [节点名-序号] 前缀，用 search 而不是 match，免得被前缀空格坑）：
#   [behavior_server-16] [1791311101.355760673] [INFO] [behavior_server]: Running spin
#   [gicp_registration-9] [1791311097.881559134] [WARN] [gicp_registration]: GICP 本帧未被采纳（…）
RE_BEHAV = re.compile(r'\[(\d{10}\.\d+)\]\s*(?:\[[A-Z]+\]\s*)?\[behavior_server\]:\s*'
                      r'(Running|Canceling|Completed)\s+(\S+)')
RE_GATE = re.compile(r'\[(\d{10}\.\d+)\]\s*(?:\[[A-Z]+\]\s*)?\[gicp_registration[^\]]*\]:\s*(.*)$')
RE_TS = re.compile(r'\[(\d{10}\.\d+)\]')


# --------------------------------------------------------------------- 工具
# 指令 ω 的键名（两套仪器历史命名不同；都认）
CMDK = {"chassis": ("chassis", "cmd_chassis"), "nav": ("nav", "cmd_nav"),
        "smooth": ("smooth", "cmd")}


def cmd_of(e, which="chassis"):
    """取该行最新指令 [vx, vy, wz]（键名兼容两套仪器）；没有则 None。"""
    c = e.get("cmd") or {}
    for k in CMDK[which]:
        v = c.get(k)
        if v:
            return v
    return None


def cmdw(e, which="chassis"):
    """取该行最新指令的 |ω|（rad/s）；键名兼容两套仪器。"""
    v = cmd_of(e, which)
    return abs(v[2]) if v else 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def pct(v, q):
    v = [x for x in v if x is not None]
    if not v:
        return None
    v = sorted(v)
    i = min(len(v) - 1, max(0, int(round((q / 100.0) * (len(v) - 1)))))
    return v[i]


def rd(x, n=3):
    return None if x is None else round(x, n)


def load_jsonl(path):
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:                       # noqa: BLE001
                pass
    return out


def load_log(path):
    if not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8", errors="ignore") as f:
        return f.readlines()


# --------------------------------------------------------------------- 单跑
class Run:
    def __init__(self, d):
        self.d = d
        self.tag = os.path.basename(d.rstrip("/"))
        self.rows = load_jsonl(os.path.join(d, "rows.jsonl"))
        self.ev = load_jsonl(os.path.join(d, "events.jsonl"))
        self.samples = load_jsonl(os.path.join(d, "samples.jsonl"))
        self.summary = {}
        p = os.path.join(d, "summary.json")
        if os.path.isfile(p):
            self.summary = json.load(open(p, encoding="utf-8"))
        self.tlsum = {}
        p = os.path.join(d, "tl.summary.json")
        if os.path.isfile(p):
            self.tlsum = json.load(open(p, encoding="utf-8"))
        self.log = load_log(os.path.join(d, "launch.log"))
        self.behav = []          # (epoch, kind, plugin)
        self.gate = []           # (epoch, text)
        self.limits = []         # 限速行
        for line in self.log:
            m = RE_BEHAV.search(line)
            if m:
                self.behav.append((float(m.group(1)), m.group(2), m.group(3)))
                continue
            m = RE_GATE.search(line)
            if m:
                self.gate.append((float(m.group(1)), m.group(2).strip()))
                continue
            m = RE_LIMIT.search(line)
            if m:
                ts = RE_TS.search(line)
                self.limits.append({"wall": float(ts.group(1)) if ts else None,
                                    "limit": float(m.group(1)), "pct": int(m.group(2)),
                                    "unlimited": "（不限速）" in m.group(4), "why": m.group(5)})
        # ---- 日志（墙钟）→ 仿真时间：用 samples.jsonl 的 (wall_abs, t) 线性插值 ----
        self.wmap = [(r["wall_abs"], r["t"]) for r in self.samples
                     if isinstance(r.get("wall_abs"), (int, float)) and r.get("t") is not None]
        self.behav_sim = []          # (sim_t, kind, plugin)
        for ep, kind, plug in self.behav:
            st = self.sim_of(ep)
            if st is not None:
                self.behav_sim.append((st, kind, plug))
        self.gate_sim = [(self.sim_of(ep), txt) for ep, txt in self.gate]
        self.gate_sim = [(t, x) for t, x in self.gate_sim if t is not None]
        # gt / lio 时间轴（优先 50 Hz rows，退化到 10 Hz samples）
        self.align = None
        self.tl = self._build_timeline()

    def sim_of(self, epoch):
        """墙钟 epoch → 仿真时间（用记录仪的 (wall_abs, sim) 对线性插值）。"""
        m = self.wmap
        if not m or epoch is None:
            return None
        if epoch <= m[0][0]:
            return m[0][1]
        if epoch >= m[-1][0]:
            return m[-1][1]
        lo, hi = 0, len(m) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if m[mid][0] <= epoch:
                lo = mid
            else:
                hi = mid
        x0, y0 = m[lo]
        x1, y1 = m[hi]
        if x1 <= x0:
            return y0
        return y0 + (y1 - y0) * (epoch - x0) / (x1 - x0)

    # ---------------- 时间轴 ----------------
    def _build_timeline(self):
        tl = []

        def keep(e):
            # ⚠ 同一仿真时刻可能有多行（50 Hz 落盘定时器比仿真钟快）⇒ 去重，
            #   否则"时长"类指标会被 dt=0 的行数垮掉（本仓踩过：|v|<0.6 时长算成 0 s）。
            if tl and e["t"] is not None and tl[-1]["t"] is not None and e["t"] == tl[-1]["t"]:
                # ⚠ 同一个仿真时刻会有多行（仿真钟 ~10 Hz、落盘 50 Hz）⇒ 去重时**必须把 IMU 峰值
                #   并起来**，否则尖峰落在被丢掉的那几行上、撞击簇被少数倍（本仓踩过：15 个 >30 只剩 5 个）。
                pim, eim = tl[-1].get("im"), e.get("im")
                if pim is not None and eim is not None:
                    e["im"] = max(pim, eim)
                elif pim is not None:
                    e["im"] = pim
                tl[-1] = e
                return
            tl.append(e)

        if self.rows:
            for r in self.rows:
                if r.get("lio") is None or r.get("gt") is None:
                    continue
                keep({"t": r["t"], "w": r["w"], "rtf": r.get("rtf"),
                           "l": r["lio"][:3], "g": r["gt"][:3], "gv": r["gt"][3:],
                           # ⚠ 用**峰值保持**（每 20 ms 窗口内的 |a| 峰值）而不是"落盘那一刻的瞬时值"：
                      #   200 Hz 的 IMU 用 50 Hz 采样会**漏掉尖峰**（实测 f2：瞬时 69.4 vs 峰值保持 192.2）
                      "im": (r.get("imu_a_peak") if r.get("imu_a_peak") is not None
                             else r.get("imu_a")), "impk": r.get("imu_a_peak"),
                           "sc": r.get("scan"), "cmd": r.get("cmd"),
                           "mo": r.get("mo"), "fit": r.get("fit"), "conv": r.get("conv"),
                           "sl": r.get("sl"), "recov": r.get("recov"),
                           "mo_age": r.get("mo_age_sim"), "mo_pub_age": r.get("mo_pub_age")})
        elif self.samples:
            for r in self.samples:
                if r.get("ox") is None or r.get("gx") is None:
                    continue
                keep({"t": r["t"], "w": r.get("wall"), "rtf": None,
                           "l": [r["ox"], r["oy"], r.get("oyaw", 0.0)],
                           "g": [r["gx"], r["gy"], r.get("gyaw", 0.0)],
                           "gv": [r.get("gvx", 0.0), r.get("gvy", 0.0), r.get("gwz", 0.0)],
                           "im": r.get("imu_a"), "impk": None,
                           "sc": [None, None, None, r.get("scan_min")],
                           "cmd": {"nav": [r.get("nvx", 0.0), r.get("nvy", 0.0), r.get("nwz", 0.0)],
                                   "chassis": [r.get("cvx", 0.0), r.get("cvy", 0.0),
                                               r.get("cwz", 0.0)]},
                           "mo": [r.get("mox"), r.get("moy"), r.get("moyaw")],
                           "fit": r.get("fit"), "conv": r.get("conv"), "sl": None,
                           "recov": r.get("recov")})
        # 冻结对齐（起步静止窗口：真值静止 + 已有 LIO）
        buf = []
        for e in tl[:400]:
            gv = e["gv"]
            if math.hypot(gv[0], gv[1]) < 0.05 and abs(gv[2]) < 0.05:
                buf.append(compose(e["l"], inv(e["g"])))
            if len(buf) >= 30:
                break
        if buf:
            self.align = (statistics.median([b[0] for b in buf]),
                          statistics.median([b[1] for b in buf]),
                          statistics.median([wrap(b[2]) for b in buf]))
        for e in tl:
            if self.align is None:
                continue
            t = compose(self.align, e["g"])
            e["err"] = math.hypot(e["l"][0] - t[0], e["l"][1] - t[1])
            e["erryaw"] = wrap(e["l"][2] - t[2])
            e["spd"] = math.hypot(e["gv"][0], e["gv"][1])
            e["wz_gt"] = e["gv"][2]
        return tl

    # ---------------- 逐帧事件（有 --events 才有） ----------------
    def frames(self):
        out = []
        for e in self.ev:
            if e.get("k") == "odom" and e.get("step") is not None:
                out.append(e)
        return out

    def first_over(self, key, thr, t_lo=None, t_hi=None):
        for e in self.tl:
            if t_lo is not None and e["t"] < t_lo:
                continue
            if t_hi is not None and e["t"] > t_hi:
                break
            v = e.get(key)
            if v is None:
                continue
            if (isinstance(v, (int, float)) and v > thr) or (isinstance(v, list) and max(v) > thr):
                return e["t"]
        return None

    # ---------------- 发散事件 ----------------
    def episodes(self, hold=1.0):
        eps, cur = [], None
        for e in self.tl:
            err = e.get("err")
            if err is None:
                continue
            if err > DIV_THR:
                if cur is None:
                    cur = {"t0": e["t"], "t1": e["t"], "max": err, "n": 1}
                else:
                    cur["t1"] = e["t"]
                    cur["max"] = max(cur["max"], err)
                    cur["n"] += 1
            elif cur is not None and e["t"] - cur["t1"] > hold:
                eps.append(cur)
                cur = None
        if cur is not None:
            eps.append(cur)
        for c in eps:
            c["dur"] = round(c["t1"] - c["t0"], 2)
        return eps

    # ---------------- 归因 ----------------
    def classify(self, ep):
        """给一个发散事件定罪：返回 (candidates, 判据 dict)。规则见模块 docstring。"""
        w0, w1 = ep["t0"] - TGAP, ep["t0"]
        win = [e for e in self.tl if w0 <= e["t"] <= ep["t1"] + 0.5]
        pre = [e for e in self.tl if w0 <= e["t"] <= w1]
        sig = {}
        # 撞击
        imp = [(e["t"], e["im"]) for e in pre if e.get("im") is not None and e["im"] > IMU_IMPACT]
        sig["impact_t"] = imp[0][0] if imp else None
        sig["impact_peak_pre"] = round(max([x[1] for x in imp]), 1) if imp else None
        inwin = [e["im"] for e in win if e.get("im") is not None]
        sig["imu_peak_win"] = round(max(inwin), 1) if inwin else None
        sig["imu_min_win"] = round(min(inwin), 1) if inwin else None
        # 甩转（指令 ω：三路里取最大绝对值）
        sig["cmd_wz_max_pre"] = round(max([max(cmdw(e, "chassis"), cmdw(e, "nav")) for e in pre]
                                          or [0.0]), 2)
        sig["cmd_wz_max_win"] = round(max([max(cmdw(e, "chassis"), cmdw(e, "nav")) for e in win]
                                          or [0.0]), 2)
        sig["gt_wz_max_pre"] = round(max([abs(e.get("wz_gt") or 0.0) for e in pre] or [0.0]), 2)
        sig["gt_wz_max_win"] = round(max([abs(e.get("wz_gt") or 0.0) for e in win] or [0.0]), 2)
        # ① 只认「原地甩转」：|ω| 落在危险带内**且**几乎不前进（车在转圈/被 recovery spin 甩）
        #    （正常转弯时 |ω| 也会进带，那不是本仓库复现出来的失效模式 —— 见 docs/lio_drift_diagnosis.md §5.1）
        inband = [e for e in pre if YAW_LO <= abs(e.get("wz_gt") or 0.0) <= YAW_HI
                  and (e.get("spd") or 0.0) < 0.3]
        sig["yaw_band_pre"] = round(sum(1 for e in pre if
                                        YAW_LO <= abs(e.get("wz_gt") or 0.0) <= YAW_HI)
                                    * (pre[1]["t"] - pre[0]["t"] if len(pre) > 1 else 0.02), 2)
        sig["yaw_band_first"] = inband[0]["t"] if inband else None
        # 贴墙/被挡（雷达前/后向最近回波 + 接触事件）
        def scmin(e, i):
            sc = e.get("sc")
            if not sc:
                return None
            v = sc[i] if len(sc) > i else None
            return v
        fwd = [x for x in (scmin(e, 2) for e in win) if x is not None]
        back = [x for x in (scmin(e, 3) for e in win) if x is not None]
        sig["scan_fwd_min_win"] = rd(min(fwd)) if fwd else None
        sig["scan_back_min_win"] = rd(min(back)) if back else None
        sig["scan_min_win"] = rd(min([x for x in (scmin(e, 3) for e in win)
                                      if x is not None] or [None])) if win else None
        sig["scan_fwd_p50_win"] = rd(pct(fwd, 50)) if fwd else None
        sig["scan_back_p50_win"] = rd(pct(back, 50)) if back else None
        b = self.summary.get("blocked", []) or []
        p = self.summary.get("pushed", []) or []
        sig["contact_pre"] = [(x["kind"], x["t0"], x["t1"], x.get("res_max"))
                              for x in (b + p) if w0 <= x["t0"] <= ep["t0"] + 0.5]
        sig["contact_win"] = [(x["kind"], x["t0"], x["t1"], x.get("res_max"))
                              for x in (b + p) if x["t1"] >= w0 and x["t0"] <= ep["t1"]]
        # 静止却飞（③ 的判据）
        still = [e for e in win if e.get("spd") is not None and e["spd"] < 0.1]
        fly = [(e["t"], e.get("err")) for e in still]
        lio_spd = []
        prev = None
        for e in win:
            if prev is not None and e["t"] > prev["t"]:
                dt = e["t"] - prev["t"]
                if dt > 1e-3:
                    lio_spd.append((math.hypot(e["l"][0] - prev["l"][0], e["l"][1] - prev["l"][1]) / dt,
                                    e["t"]))
            prev = e
        sig["still_frames"] = len(still)
        sig["lio_spd_max_while_still"] = rd(max([s for s, _ in lio_spd] or [0.0]), 2)
        sig["runaway"] = bool(sig["still_frames"] > 20 and (sig["lio_spd_max_while_still"] or 0) > 1.0)
        # 定位健康
        sig["conv_false_t"] = next((e["t"] for e in self.tl if e.get("conv") is False), None)
        sig["fit_nan_t"] = next((e["t"] for e in self.tl
                                 if isinstance(e.get("fit"), float) and math.isnan(e["fit"])), None)
        ms = self.summary.get("mo_stale_runs") or []
        sig["mo_stale"] = [(x.get("t0"), x.get("t1")) for x in ms]
        # 恢复行为
        rec = [e.get("recov") for e in self.tl if e.get("recov") is not None]
        sig["recov_at_t0"] = None
        for e in self.tl:
            if e["t"] >= ep["t0"] and e.get("recov") is not None:
                sig["recov_at_t0"] = e["recov"]
                break
        sig["recov_max"] = max(rec) if rec else None
        # 恢复行为（日志墙钟 → 仿真时间）：事件前 4 s 内有没有跑过 Spin/BackUp/Wait
        bs = [(t, k, p) for t, k, p in self.behav_sim if w0 <= t <= ep["t0"] + 0.5]
        sig["behav_pre"] = ["%s %s@%.1f" % (k, p, t) for t, k, p in bs]
        sig["spin_behav_pre"] = round(min([ep["t0"] - t for t, k, p in bs
                                           if p == "spin" and k == "Running"] or [0.0]), 1) or None
        # 逐帧单步
        fr = [f for f in self.frames() if w0 <= f["t"] <= ep["t1"] + 0.5]
        sig["frame_step_max"] = rd(max([f.get("step", 0.0) for f in fr] or [0.0]), 3)
        sig["frame_spd_max"] = rd(max([f.get("spd", 0.0) for f in fr] or [0.0]), 2)
        sig["frame_yawjump_max"] = rd(max([abs(f.get("spdyaw", 0.0)) for f in fr] or [0.0]), 4)
        sig["frame_n"] = len(fr)

        # ---- 定罪 ----
        cands = []
        if sig["impact_t"] is not None and (sig["impact_t"] <= sig["cmd_wz_max_pre"] or True):
            cands.append("②撞击后失锁（撞击在 %.1f s，IMU 峰 %.1f）"
                         % (sig["impact_t"], sig["impact_peak_pre"]))
        if sig["contact_pre"]:
            cands.append("②慢速贴墙/被挡（接触事件 %d 个，IMU 平=%s）"
                         % (len(sig["contact_pre"]),
                            sig["imu_peak_win"] is not None and sig["imu_peak_win"] < IMU_CALM))
        if sig["yaw_band_first"] is not None:
            cands.append("①原地甩转（实测 |ω| 落在 %.1f~%.1f 且 |v|<0.3，最早 %.1f s；指令峰 %.2f）"
                         % (YAW_LO, YAW_HI, sig["yaw_band_first"], sig["cmd_wz_max_pre"]))
        if sig["spin_behav_pre"]:
            cands.append("①恢复行为 Spin 在事件前 %s s 跑过" % sig["spin_behav_pre"])
        if sig["runaway"] and not sig["contact_pre"] and sig["impact_t"] is None:
            cands.append("③滤波数值爆掉（车静止 %d 帧，LIO 隐含 %.1f m/s）"
                         % (sig["still_frames"], sig["lio_spd_max_while_still"] or 0.0))
        if sig["scan_fwd_min_win"] is not None and sig["scan_back_min_win"] is not None \
                and sig["scan_fwd_min_win"] < 0.8 and sig["scan_back_min_win"] < 0.8:
            cands.append("④几何退化窗口（前 %.2f m / 后 %.2f m 同时 <0.8 m）"
                         % (sig["scan_fwd_min_win"], sig["scan_back_min_win"]))
        if not cands:
            cands.append("③无外部触发（既无撞击/接触，也无甩转）—— 只能归到滤波自身")
        return cands, sig


# --------------------------------------------------------------------- 报表
def run_metrics(r: Run):
    m = {"tag": r.tag, "dir": r.d}
    s = r.summary
    m["goal_status"] = s.get("goal_status")
    m["recov_fb"] = (s.get("fb") or {}).get("recov")
    evs = s.get("events") or []
    gs = next((e for e in evs if e.get("kind") == "goal_sent"), None)
    ge = next((e for e in evs if e.get("kind") in ("goal_result", "drive_timeout")), None)
    m["drive_wall_s"] = round(ge["wall"] - gs["wall"], 1) if (gs and ge) else None
    m["travel_gt"] = s.get("travel_gt")
    m["plan_len0"] = (s.get("plans") or [{}])[0].get("len") if s.get("plans") else None
    m["contact_events"] = s.get("contact_events")
    m["blocked"] = len(s.get("blocked") or [])
    m["pushed"] = len(s.get("pushed") or [])
    m["mo_stale"] = len(s.get("mo_stale_runs") or [])
    m["mo_age_max"] = s.get("mo_age_max")
    t0 = gs["sim"] if gs else None
    t1 = (ge.get("sim") if ge and ge.get("sim") is not None else None)
    if t1 is None:                      # 没有终态事件（例如被 --max-drive-sec 截断）⇒ 用最后一个 plan 之后留白
        t1 = r.tl[-1]["t"] if r.tl else None
    m["t0_sim"], m["t1_sim"] = rd(t0, 2), rd(t1, 2)
    # 速度/IMU/发散：**只统计目标有效的那一段**（[goal_sent, 终态]），
    #   否则 post-goal 的静止会把 |v|<0.6 时长灌水（本仓踩过：26.5 s 的跑算出 22.7 s）
    tl = [e for e in r.tl if (t0 is None or e["t"] >= t0) and (t1 is None or e["t"] <= t1)]
    v = [e["spd"] for e in tl if e.get("spd") is not None]
    m["v_p50"], m["v_p90"], m["v_max"] = rd(pct(v, 50), 2), rd(pct(v, 90), 2), rd(max(v), 2) if v else None
    if tl:
        dts = [tl[i]["t"] - tl[i - 1]["t"] for i in range(1, len(tl))]
        dt = statistics.median(dts) if dts else 0.1
        m["t_lt_0.6_s"] = round(sum(dt for e in tl if (e.get("spd") or 0) < 0.6), 1)
        m["t_moving_s"] = round(sum(dt for e in tl if (e.get("spd") or 0) > 0.05), 1)
    im = [e["im"] for e in tl if e.get("im") is not None]
    m["imu_max"] = rd(max(im), 1) if im else None
    m["imu_gt30"] = sum(1 for x in im if x > 30)
    m["imu_gt50"] = sum(1 for x in im if x > 50)
    er = [e["err"] for e in r.tl if e.get("err") is not None]
    m["lio_err_max"] = rd(max(er), 2) if er else None
    fr = r.frames()
    m["frame_step_max"] = rd(max([f.get("step", 0.0) for f in fr] or [0.0]), 3)
    m["frame_spd_max"] = rd(max([f.get("spd", 0.0) for f in fr] or [0.0]), 2)
    m["diverged"] = bool(m["lio_err_max"] and m["lio_err_max"] > SEVERE_DIV)
    fits = [e["fit"] for e in r.tl if isinstance(e.get("fit"), float)]
    good = [f for f in fits if not math.isnan(f)]
    m["fit_p50"] = rd(pct(good, 50), 4) if good else None
    m["fit_nan"] = sum(1 for f in fits if math.isnan(f))
    m["conv_false_n"] = sum(1 for e in r.tl if e.get("conv") is False)
    m["gate_reject"] = sum(1 for _, txt in r.gate if "未采纳" in txt or "拒绝" in txt)
    m["gate_lost"] = sum(1 for _, txt in r.gate if "失效" in txt)
    m["behav"] = {}
    for _, kind, plug in r.behav:
        if kind == "Running":
            m["behav"][plug] = m["behav"].get(plug, 0) + 1
    rtf = [e["rtf"] for e in r.tl if e.get("rtf") is not None]
    m["rtf_p50"] = rd(pct(rtf, 50), 2) if rtf else None
    if r.limits:
        lim = [x["limit"] for x in r.limits]
        m["sl_n"] = len(lim)
        m["sl_p50"] = rd(pct(lim, 50))
        m["sl_min"] = rd(min(lim))
        m["sl_unlimited_frac"] = rd(sum(1 for x in r.limits if x["unlimited"]) / len(r.limits), 3)
    dm = (s.get("dist_moving") or {})
    m["scan_min_p50"] = (dm.get("scan_min") or {}).get("p50")
    m["scan_min_p05"] = (dm.get("scan_min") or {}).get("p05")
    m["g_lethal_margin_p05"] = (dm.get("g_lethal_f_margin") or {}).get("p05")
    m["frames_n"] = len(fr)
    m["rows_n"] = len(r.rows) or len(r.samples)
    return m


def print_metrics(m):
    def f(k, unit=""):
        v = m.get(k)
        return "—" if v is None else ("%s%s" % (v, unit))
    print("  %-14s 结局=%s 墙钟驾驶=%ss 里程=%sm (plan %.1fm) 恢复=%s %s"
          % (m["tag"], {4: "到点", 5: "取消", 6: "中止", None: "未达终态"}.get(m["goal_status"],
                                                                        m["goal_status"]),
             f("drive_wall_s"), f("travel_gt"), m.get("plan_len0") or 0.0,
             f("recov_fb"), json.dumps(m["behav"], ensure_ascii=False)))
    print("                  |v| p50/p90/max=%s/%s/%s  |v|<0.6 %ss  运动 %ss  RTF=%s"
          % (f("v_p50"), f("v_p90"), f("v_max"), f("t_lt_0.6_s"), f("t_moving_s"), f("rtf_p50")))
    print("                  IMU max=%s >30:%s >50:%s   发散=%s lio_err_max=%sm 单帧步长=%sm (%.1f m/s)"
          % (f("imu_max"), m["imu_gt30"], m["imu_gt50"], m["diverged"], f("lio_err_max"),
             f("frame_step_max"), m.get("frame_spd_max") or 0.0))
    print("                  GICP: 拒=%s 失效=%s fit_p50=%s fit_nan=%s conv_false=%s mo_stale=%s(age %ss)"
          % (m["gate_reject"], m["gate_lost"], f("fit_p50"), m["fit_nan"], m["conv_false_n"],
             m["mo_stale"], f("mo_age_max")))
    print("                  接触=%s blocked=%s pushed=%s  余量 scan_min p50/p05=%s/%s g_margin_p05=%s"
          % (m["contact_events"], m["blocked"], m["pushed"], f("scan_min_p50"), f("scan_min_p05"),
             f("g_lethal_margin_p05")))
    if m.get("sl_n"):
        print("                  限速器：%d 行 limit p50=%s min=%s 不限速占比=%s"
              % (m["sl_n"], f("sl_p50"), f("sl_min"), m.get("sl_unlimited_frac")))


def print_timeline(r: Run, eps):
    print("#" * 100)
    print("## 归因：%s（dir=%s）" % (r.tag, r.d))
    print("   数据源：%s；对齐 T=lio∘gt⁻¹（起步静止窗口冻结）= %s"
          % ("50 Hz rows + 逐帧 events" if r.rows else "10 Hz samples（**无 50 Hz 时间轴**）",
             None if r.align is None else [round(x, 3) for x in r.align]))
    if r.tlsum:
        print("   采样健康：rows=%s cnt=%s drops=%s mo_age_max=%ss"
              % (r.tlsum.get("rows"), json.dumps(r.tlsum.get("cnt", {}), ensure_ascii=False),
                 r.tlsum.get("drops"), r.tlsum.get("mo_age_max_s")))
    if not eps:
        print("   ⇒ **本次跑没有超过 %.2f m 的 LIO 误差段**（lio_err_max=%s；"
              "真发散门槛 %.2f m）"
              % (DIV_THR, rd(max([e["err"] for e in r.tl if e.get("err") is not None] or [0]), 3),
                 SEVERE_DIV))
    for i, ep in enumerate(eps, 1):
        cands, sig = r.classify(ep)
        print("\n### 事件 %d：t=%.1f→%.1f s（%.1f s）max lio_err=**%.2f m**" % (i, ep["t0"], ep["t1"],
                                                                          ep["dur"], ep["max"]))
        print("  判据（全部实测）：")
        for k in ("impact_t", "impact_peak_pre", "imu_peak_win", "imu_min_win",
                  "cmd_wz_max_pre", "cmd_wz_max_win", "gt_wz_max_pre", "gt_wz_max_win",
                  "yaw_band_first", "yaw_band_pre", "scan_fwd_min_win", "scan_fwd_p50_win",
                  "scan_back_min_win", "scan_back_p50_win", "still_frames",
                  "lio_spd_max_while_still", "conv_false_t", "fit_nan_t",
                  "recov_at_t0", "recov_max", "frame_step_max", "frame_spd_max",
                  "frame_yawjump_max"):
            print("    · %-24s = %s" % (k, sig.get(k)))
        print("    · contact_pre              = %s" % (sig.get("contact_pre"),))
        print("    · behav_pre                = %s" % (sig.get("behav_pre"),))
        print("    · mo_stale                 = %s" % (sig.get("mo_stale"),))
        print("  ⇒ 判定：%s" % (" ＋ ".join(cands)))
    # 关键事件排序（全局）
    marks = []
    for e in r.tl:
        if e.get("im") is not None and e["im"] > IMU_IMPACT:
            marks.append((e["t"], "IMU %.1f" % e["im"]))
    if r.frames():
        for f in r.frames():
            if f.get("step", 0) > 0.5:
                marks.append((f["t"], "LIO 单帧 %.2f m (%.1f m/s)" % (f["step"], f.get("spd") or 0)))
                break
    for x in (r.summary.get("mo_stale_runs") or []):
        marks.append((x.get("t0"), "map→odom 停发"))
    for _, kind, plug in r.behav_sim:
        marks.append((None, "行为 %s %s" % (kind, plug)))
    for t, kind, plug in r.behav_sim:
        marks.append((t, "行为 %s %s" % (kind, plug)))
    marks = [m for m in marks if m[0] is not None]
    marks.sort()
    if marks:
        print("\n  事件序（sim 时间，前 25 条）：")
        for t, txt in marks[:25]:
            print("    %8.2f  %s" % (t, txt))


def print_window(r: Run, a, b, dt=0.2):
    """把 [a,b]（仿真秒）这段按 dt 打成人读时间轴 —— 报告 §1 的那张表就是它。

    列：t / lio_err / LIO 隐含速度（相邻采样）/ 真值 |v| 与 ω / 三路指令 ω / IMU |a| /
        雷达 前向·后向·全向最近回波 / map→odom 发布年龄 / fitness / converged / 恢复次数。
    另外插入逐帧 LIO 跳变（>%.2f m）与日志事件（恢复行为 / 拒帧）。""" % STEP_THR
    print("\n  时间轴 [%.1f, %.1f] s（%s）" % (a, b, r.tag))
    print("  %7s %8s %8s %6s %6s | %6s %6s %6s | %6s %6s %6s %6s | %6s %6s %6s | %6s %6s %5s %3s"
          % ("t", "lio_err", "lio_spd", "gt|v|", "gt_wz", "c_chs", "c_nav", "c_sm",
             "imu", "fwd", "back", "min", "sl", "mo_age", "mo_pub", "fit", "conv", "rec", "n"))
    tl = [e for e in r.tl if a <= e["t"] <= b]
    fr = [f for f in r.frames() if a <= f["t"] <= b]
    marks = []
    gate_seen = {}
    for t, kind, plug in r.behav_sim:
        if a <= t <= b:
            marks.append((t, "★行为 %s %s" % (kind, plug)))
    for t, txt in r.gate_sim:
        if not (a <= t <= b):
            continue
        key = txt[:40]
        if key in gate_seen and t - gate_seen[key] < 3.0:   # 同一句反复打 ⇒ 只留第一条
            continue
        gate_seen[key] = t
        marks.append((t, "gate: %s" % txt[:70]))
    for f in fr:
        if f.get("step", 0.0) > STEP_THR:
            marks.append((f["t"], "⚡LIO 单帧 %.3f m / %.2f m/s / Δyaw %+.3f"
                          % (f["step"], f.get("spd") or 0.0, f.get("spdyaw") or 0.0)))
    tnext, inext = a, 0
    prev = None
    for e in tl:
        while inext < len(marks) and marks[inext][0] <= e["t"]:
            print("  %7.2f   %s" % (marks[inext][0], marks[inext][1]))
            inext += 1
        if e["t"] < tnext:
            prev = e
            continue
        tnext = e["t"] + dt
        cch, cna, csm = cmdw(e, "chassis"), cmdw(e, "nav"), cmdw(e, "smooth")
        sc = e.get("sc") or [None, None, None, None]
        lspd = None
        if prev is not None and e["t"] > prev["t"]:
            lspd = math.hypot(e["l"][0] - prev["l"][0], e["l"][1] - prev["l"][1]) / (e["t"] - prev["t"])
        fit = e.get("fit")
        print("  %7.2f %8.3f %8s %6.2f %6.2f | %6.2f %6.2f %6s | %6s %6s %6s %6s | %6s %6s %6s | "
              "%6s %5s %3s %5s"
              % (e["t"], e.get("err") or 0.0, "—" if lspd is None else "%.2f" % lspd,
                 e.get("spd") or 0.0, e.get("wz_gt") or 0.0, cch, cna,
                 "%.2f" % csm,
                 "—" if e.get("im") is None else "%.1f" % e["im"],
                 "—" if sc[2] is None else "%.2f" % sc[2],
                 "—" if sc[3] is None else "%.2f" % sc[3],
                 "—" if not sc or sc[3] is None else "%.2f" % min([x for x in sc[2:] if x is not None] or [0]),
                 "—" if e.get("sl") is None else "%.2f" % e["sl"],
                 "—" if e.get("mo_age") is None else "%.2f" % e["mo_age"],
                 "—" if e.get("mo_pub_age") is None else "%.2f" % e["mo_pub_age"],
                 "—" if fit is None else ("nan" if isinstance(fit, float) and math.isnan(fit)
                                          else "%.3f" % fit),
                 e.get("conv"), e.get("recov"), len(fr)))
        prev = e
    while inext < len(marks):
        print("  %7.2f   %s" % marks[inext])
        inext += 1


def yaw_transfer(r: Run, cmd_key="chassis"):
    """实测「指令 ω → 实测 ω」传递：分箱中位数（这是"能不能靠限 wz 来防甩转"的地基）。

    只取**时戳对齐**的样本（同一行里的指令与真值，10/50 Hz 行都算），分箱 = 指令 |ω| 的区间。
    平地直行时指令 ω≈0 ⇒ 会落在第 0 箱（那里只有噪声，别拿去算比例）。
    """
    bins = [(0.0, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.5)]
    acc = {b: [] for b in bins}
    for e in r.tl:
        c = cmd_of(e, cmd_key)
        if not c or e.get("wz_gt") is None:
            continue
        cw = abs(c[2])
        gw = abs(e["wz_gt"])
        for b in bins:
            if b[0] <= cw < b[1]:
                acc[b].append((cw, gw, e.get("spd") or 0.0))
                break
    print("\n  指令 ω → 实测 ω（%s；同 10/50 Hz 行内对齐，分箱中位数）" % cmd_key)
    print("    %-14s %6s %8s %8s %8s %8s" % ("指令|ω| 箱", "n", "cmd p50", "实测 p50", "比值 p50", "原地占比"))
    for b in bins:
        v = acc[b]
        if len(v) < 5:
            continue
        c50 = pct([x[0] for x in v], 50)
        g50 = pct([x[1] for x in v], 50)
        rat = pct([x[1] / x[0] for x in v if x[0] > 1e-3], 50)
        inplace = sum(1 for x in v if x[2] < 0.3) / len(v)
        print("    %-14s %6d %8.2f %8.2f %8s %8.2f"
              % ("[%.1f,%.1f)" % b, len(v), c50, g50, "—" if rat is None else "%.2f" % rat,
                 inplace))
    # 符号一致率（指令与实测转向同号）——"底盘到底听不听话"的判据
    sgn = [(cmd_of(e, cmd_key)[2], e["wz_gt"]) for e in r.tl
           if cmd_of(e, cmd_key) and abs(cmd_of(e, cmd_key)[2]) > 0.3 and e.get("wz_gt") is not None
           and abs(e["wz_gt"]) > 0.1]
    if sgn:
        agree = sum(1 for a, b in sgn if a * b > 0) / len(sgn)
        print("    |cmd|>0.3 且 |实测|>0.1 的样本 n=%d，同号率=%.2f"
              % (len(sgn), agree))


def print_phases(r: Run, win=4.0):
    """把一次跑切成「里程碑」并按**先后顺序**给出每个里程碑之前的现场（报告的因果排序就是它）。

    里程碑（谁先出现就是谁先动的手）：
      · IMU 第一次 > %.0f（真撞击）
      · GICP 第一帧未被采纳 / converged=False / map→odom 停发（定位链先坏？）
      · LIO 误差第一次 > %.2f m（里程计开始飘）
      · LIO **自持跑飞**：车静止（真值 |v|<0.2）而 LIO 隐含速度 >2 m/s 连续 ≥3 采样
      · 第一次恢复行为（Running spin/backup/wait）
    """
    tls = r.tl
    ms = []

    def add(name, t):
        if t is not None:
            ms.append((t, name))

    add("IMU |a|>%.0f（撞击）" % IMU_IMPACT,
        next((e["t"] for e in tls if (e.get("im") or 0) > IMU_IMPACT), None))
    add("GICP 首帧未被采纳", next((t for t, x in r.gate_sim if "未被采纳" in x), None))
    add("GICP converged=False", next((e["t"] for e in tls if e.get("conv") is False), None))
    add("LIO 误差 >%.2f m" % DIV_THR,
        next((e["t"] for e in tls if (e.get("err") or 0) > DIV_THR), None))
    # 「LIO 越界」：LIO 的隐含速度**首次持续**超过底盘物理上限 2.0 m/s
    #   （用"持续"而不是"单帧"：单帧毛刺不算；用底盘上限而不是固定值：2.0 = min(MPPI 2.5, smoother 2.0)）
    speeds, prev = [], None
    for e in tls:
        if prev is not None and e["t"] > prev["t"]:
            speeds.append((e["t"], math.hypot(e["l"][0] - prev["l"][0], e["l"][1] - prev["l"][1])
                           / (e["t"] - prev["t"]), e.get("spd")))
        prev = e
    for i in range(len(speeds) - 9):
        w = speeds[i:i + 10]
        if all(x[1] > 2.0 for x in w):
            add("★LIO 速度持续 >2 m/s（底盘上限）", w[0][0])
            break
    for t, k, p in r.behav_sim:
        if k == "Running":
            add("恢复行为 %s" % p, t)
            break
    add("map→odom 停发", (r.summary.get("mo_stale_runs") or [{}])[0].get("t0"))
    ms.sort()
    print("\n  里程碑（按时间）与各自前 %.0f s 的现场：" % win)
    print("    %7s %-24s %7s %7s %7s %7s %7s %7s  %s"
          % ("t(sim)", "里程碑", "imuP", "cmdωP", "gtωP", "fwd", "back", "LIO v", "接触/行为"))
    for t, name in ms:
        pre = [e for e in tls if t - win <= e["t"] <= t]
        im = max([e.get("im") or 0 for e in pre] or [0])
        cw = max([cmdw(e, "chassis") for e in pre] or [0])
        gw = max([abs(e.get("wz_gt") or 0) for e in pre] or [0])
        fwd = [e["sc"][2] for e in pre if e.get("sc") and e["sc"][2] is not None]
        bak = [e["sc"][3] for e in pre if e.get("sc") and e["sc"][3] is not None]
        lv, prev = [], None
        for e in pre:
            if prev is not None and e["t"] > prev["t"]:
                lv.append(math.hypot(e["l"][0] - prev["l"][0], e["l"][1] - prev["l"][1])
                          / (e["t"] - prev["t"]))
            prev = e
        ct = [x["kind"] for x in (r.summary.get("blocked", []) + r.summary.get("pushed", []))
              if t - win <= x["t0"] <= t]
        bh = [p for tt, k, p in r.behav_sim if t - win <= tt <= t and k == "Running"]
        print("    %7.2f %-24s %7.1f %7.2f %7.2f %7s %7s %7.2f  %s"
              % (t, name, im, cw, gw,
                 "—" if not fwd else "%.2f" % min(fwd), "—" if not bak else "%.2f" % min(bak),
                 max(lv) if lv else 0.0,
                 ",".join(ct) + (" " if ct and bh else "") + ",".join(sorted(set(bh)))))
    return ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="*", help="跑次产物目录（可多个，出对比表）")
    ap.add_argument("--timeline", default="", help="对这一次跑做归因（可重复用逗号分隔）")
    ap.add_argument("--window", default="", help="打印时间轴窗口 'A,B'（仿真秒）")
    ap.add_argument("--window-dir", default="", help="窗口用在哪个跑次目录（默认 = 第一个 --timeline）")
    ap.add_argument("--window-dt", type=float, default=0.2)
    ap.add_argument("--json", default="", help="把指标表写 JSON")
    ap.add_argument("--yaw-transfer", action="store_true", help="额外打印「指令 ω→实测 ω」传递表")
    ap.add_argument("--glob", default="", help="按通配符收集跑次目录")
    a = ap.parse_args()
    dirs = list(a.dirs)
    if a.glob:
        dirs += sorted(glob.glob(a.glob))
    tl_dirs = [x for x in a.timeline.split(",") if x] or dirs
    runs = [Run(d) for d in dirs]
    if runs:
        print("=" * 100)
        print("## 限速 A/B 指标表")
        for r in runs:
            print_metrics(run_metrics(r))
    for d in tl_dirs:
        r = next((x for x in runs if x.d == d), None) or Run(d)
        print_timeline(r, r.episodes())
        if a.yaw_transfer:
            yaw_transfer(r)
        print_phases(r)
    if a.window:
        d = a.window_dir or (tl_dirs[0] if tl_dirs else None)
        if d:
            lo, hi = [float(x) for x in a.window.split(",")]
            print_window(next((x for x in runs if x.d == d), None) or Run(d), lo, hi, a.window_dt)
    if a.json:
        out = {r.tag: run_metrics(r) for r in runs}
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        print("[an] 指标 → %s" % a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
