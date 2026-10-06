#!/usr/bin/env python3
"""把 `segment_drive.py` 的 50 Hz 时间轴 + 事件表算成"可归因的数字"。

回答的问题（对应 docs/lio_drift_diagnosis.md 的结论表）：
  1. LIO（`/odom` 与 `odom→base_link`）相对**仿真真值**的误差：整程 / 每段 RMSE、max、
     段内相对位移误差（RPE）与"每米漂移"；融合位姿 `map→base_link`（= `map→odom` ∘ `odom→base_link`）
     同样一套 ⇒ 两条曲线放一起，"融合是不是被 LIO 带飘"一眼能看出来。
  2. 单步跳变（>0.2 m / >10° 一步）在**仿真时间轴**上的位置，以及那一刻的
     `/scan` 有限束数 / 前向最近回波 / 指令角速度 / RTF ⇒ 把跳变和"卡死/退化/快转"对上。
  3. 逐秒窗口的"误差增量"与三类候选原因的相关性（束数塌陷 / 角速度尖峰 / RTF 掉）。
  4. 融合位姿是不是**有界**（slam 在纠正）还是**跟着 LIO 一起飘**。

坐标系与对齐（这是所有数字的地基，必须说清楚）：
  · `/odom_ground_truth` 在 **world 系**（planar_move 发的是模型世界位姿）；
  · LIO 的 `/odom`、`odom→base_link` 在 LIO 的**出生点相对系**（原点/朝向任意）；
  · slam_toolbox 的 `map` 系 = LIO 出生点相对系（`map→odom` 从单位阵起）。
  ⇒ 三个序列与真值之间各差一个**常量刚体变换**。本工具用**起步前静止窗口**（事件
    `link_ready`→`warmup_end`）估这一个变换，然后**全程沿用**——
    绝不能"每段重新对齐"，那会把漂移当坐标系差减掉，是自欺欺人。

用法：
  python3 tools/scripts/diag/lio_drift_analyze.py \
      --csv .tmp_cache/diag/spl.csv --events .tmp_cache/diag/spl.json \
      --out-json .tmp_cache/diag/spl.analysis.json \
      --out-dir docs/img --tag lio_drift_spl

  --no-figs：只出数字（CI/快速回归用）。
  退出码：0=算完；2=输入不可用（缺列/没有可配对样本）。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

NAN = float('nan')
# 判定阈值（都写在这里，方便复现时逐条改）
JUMP_POS = 0.20          # 单步位移跳变阈值 m
JUMP_YAW = math.radians(10.0)   # 单步偏航跳变阈值 rad
# ⚠️ 必须**同时**看"这一步的速率"（自测+首跑踩过）：快转段实测 1.87 rad/s，10 Hz 采样下
#    每一步天然就是 10.7° —— 只看"单步 >10°"会把**正常快转**判成跳变（首跑误报 149 次）。
#    这里的速率门取在"底盘实测能力之上"：线速度 >1.5 m/s、角速度 >3.5 rad/s 才算异常。
JUMP_V = 1.5             # m/s
JUMP_W = 3.5             # rad/s
PAIR_DT = 0.15           # LIO↔真值 时间配对容差 s
AGE_MAX = 0.5            # 采样"陈旧"判据：这条数据比现在旧多少算不可信 s


# ------------------------------------------------------------------ 读 CSV
def read_csv(path):
    with open(path, newline='') as f:
        rd = csv.DictReader(f)
        cols = rd.fieldnames
        rows = list(rd)
    out = {c: np.full(len(rows), NAN) for c in cols}
    for i, r in enumerate(rows):
        for c in cols:
            v = r.get(c, '')
            if v is None or v == '':
                continue
            try:
                out[c][i] = float(v)
            except ValueError:
                pass
    return out, len(rows)


def dedupe(arr_t, *arrs, dt_eps=1e-6):
    """按时间戳去重（CSV 是 50 Hz 采样，而 LIO/真值只有 10 Hz ⇒ 同一个值会重复 5 行）。"""
    ok = np.isfinite(arr_t)
    t = arr_t[ok]
    others = [a[ok] for a in arrs]
    if len(t) == 0:
        return (t, *others)
    keep = np.ones(len(t), dtype=bool)
    keep[1:] = np.diff(t) > dt_eps
    return (t[keep], *[a[keep] for a in others])


def pair_to_gt(t_src, x_src, y_src, yaw_src, t_gt, x_gt, y_gt, yaw_gt, tol=PAIR_DT):
    """把源序列配到真值上：时间范围取交集 + **线性插值**取真值。

    为什么用插值而不是"最近邻取一条"（自测踩过的坑）：真值只有 10 Hz，而快转段
    ω=1.5 rad/s ⇒ 最近邻在 ±0.05 s 上的偏航差可达 **±4.3°**，会被当成"LIO 偏航误差"
    报出来 —— 那是配对伪影，不是漂移。位置同理（0.3 m/s ⇒ 1.5 cm）。
    返回 (t, sx, sy, syaw, gx, gy, gyaw, dt_nn)；`dt_nn` 只用于报告配对质量。
    """
    if len(t_gt) < 2 or len(t_src) == 0:
        return None
    finite = np.isfinite(t_src)
    inrange = finite & (t_src >= t_gt[0] - tol) & (t_src <= t_gt[-1] + tol)
    if inrange.sum() < 5:
        return None
    ts = t_src[inrange]
    idx = np.clip(np.searchsorted(t_gt, ts), 1, len(t_gt) - 1)
    left = np.abs(t_gt[idx - 1] - ts) < np.abs(t_gt[idx] - ts)
    j = np.where(left, idx - 1, idx)
    dt = np.abs(t_gt[j] - ts)
    gxi = np.interp(ts, t_gt, x_gt)
    gyi = np.interp(ts, t_gt, y_gt)
    gyawi = np.interp(ts, t_gt, np.unwrap(yaw_gt))
    return (ts, x_src[inrange], y_src[inrange], yaw_src[inrange],
            gxi, gyi, wrap_arr(gyawi), dt)


def align_from(src, gt, k):
    """用前 k 个配对样本估常量刚体 (yaw0, t0)：gt ≈ Rz(yaw0)·src + t0。"""
    dy = np.unwrap(gt[2][:k]) - np.unwrap(src[2][:k])
    yaw0 = float(np.arctan2(np.sin(dy).mean(), np.cos(dy).mean()))
    c, s = math.cos(yaw0), math.sin(yaw0)
    R = np.array([[c, -s], [s, c]])
    P = np.stack([src[0], src[1]], axis=1)
    G = np.stack([gt[0], gt[1]], axis=1)
    t0 = (G[:k] - P[:k] @ R.T).mean(axis=0)
    return yaw0, t0, R


def apply_align(src, R, t0):
    P = np.stack([src[0], src[1]], axis=1)
    return P @ R.T + t0


def plen(pts):
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def wrap_arr(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ------------------------------------------------------------------ 分段切片
def segments_from_events(events, gmap=None):
    """把 seg_start/seg_end 配成段；返回 [{label, group, sim0, sim1, ...}]。"""
    gmap = gmap or {}
    open_seg = {}
    segs = []
    for e in events:
        k = e.get('event')
        if k == 'seg_start':
            open_seg[e['label']] = e
        elif k == 'seg_end':
            st = open_seg.pop(e['label'], None)
            if st is None:
                continue
            segs.append({'label': e['label'], 'kind': e.get('kind'),
                         'group': gmap.get(e['label']),
                         'sim0': float(st['sim']) if st.get('sim') is not None else None,
                         'sim1': float(e['sim']) if e.get('sim') is not None else None,
                         'wall_s': e.get('wall_s'), 'jammed': e.get('jammed'),
                         'n_stuck': e.get('n_stuck'), 'acc_rad': e.get('acc_rad')})
    return segs


def analyze_series(name, t, x, y, yaw, gt, R, t0, segs, yaw0=0.0):
    """给一条序列算整程 + 每段 + 单步跳变。

    ⚠️ 偏航也必须一起对齐：est = Rz(yaw0)·src + t0 ⇒ 对齐后的偏航 = src_yaw + yaw0。
       忘了这一步会得到"偏航误差恒等于坐标系差"这种废数（自测踩过）。
    """
    est = apply_align((x, y), R, t0)
    yaw = wrap_arr(yaw + yaw0)
    err = np.linalg.norm(est - np.stack([gt[0], gt[1]], axis=1), axis=1)
    res = {
        'name': name, 'n_pairs': int(len(t)),
        'pair_dt_mean_s': round(float(gt[3].mean()), 4),
        'pair_dt_max_s': round(float(gt[3].max()), 4),
        'ate_rmse_m': round(float(np.sqrt((err ** 2).mean())), 4),
        'ate_mean_m': round(float(err.mean()), 4),
        'ate_max_m': round(float(err.max()), 4),
        'ate_max_sim_s': round(float(t[int(np.argmax(err))]), 2),
        'ate_final_m': round(float(err[-1]), 4),
        'yaw_err_rmse_deg': round(float(np.degrees(
            np.sqrt((wrap_arr(yaw - gt[2]) ** 2).mean()))), 3),
        'yaw_err_mean_deg': round(float(np.degrees(
            np.arctan2(np.sin(yaw - gt[2]).mean(), np.cos(yaw - gt[2]).mean()))), 3),
        'traj_len_m': round(plen(est), 3),
    }
    res['segments'] = []
    for s in segs:
        if s['sim0'] is None or s['sim1'] is None:
            continue
        m = (t >= s['sim0']) & (t <= s['sim1'])
        n = int(m.sum())
        if n < 3:
            res['segments'].append({'label': s['label'], 'n_pairs': n,
                                    'note': '样本太少'})
            continue
        e = err[m]
        g = np.stack([gt[0][m], gt[1][m]], axis=1)
        p = est[m]
        dgt = plen(g)
        dlio = plen(p)
        # RPE：段内相对位移误差（扣掉段起点，只看"这一段自己走了多少"）
        rel_err = float(np.linalg.norm((p[-1] - p[0]) - (g[-1] - g[0])))
        rpe_m = float(np.linalg.norm(
            (p - p[0]) - (g - g[0]), axis=1).mean()) if n > 2 else NAN
        yaw_e = wrap_arr(yaw[m] - gt[2][m])
        res['segments'].append({
            'label': s['label'], 'group': s.get('group'), 'kind': s['kind'],
            'sim_s': round(float(t[m][-1] - t[m][0]), 2),
            'n_pairs': n,
            'gt_dist_m': round(dgt, 3), 'est_dist_m': round(dlio, 3),
            'len_ratio': round(dlio / dgt, 4) if dgt > 1e-6 else None,
            'ate_rmse_m': round(float(np.sqrt((e ** 2).mean())), 4),
            'ate_mean_m': round(float(e.mean()), 4),
            'ate_max_m': round(float(e.max()), 4),
            'err_start_m': round(float(e[:max(1, n // 10)].mean()), 4),
            'err_end_m': round(float(e[-max(1, n // 10):].mean()), 4),
            'err_growth_m': round(float(e[-max(1, n // 10):].mean() -
                                        e[:max(1, n // 10)].mean()), 4),
            'rpe_mean_m': round(rpe_m, 4) if rpe_m == rpe_m else None,
            'rpe_final_m': round(rel_err, 4),
            'drift_m_per_m': round(rel_err / dgt, 5) if dgt > 0.05 else None,
            'yaw_err_end_deg': round(float(np.degrees(
                np.arctan2(np.sin(yaw_e[-max(1, n // 10):]).mean(),
                           np.cos(yaw_e[-max(1, n // 10):]).mean()))), 3),
            'yaw_err_rmse_deg': round(float(np.degrees(
                np.sqrt((yaw_e ** 2).mean()))), 3),
            'jammed': s.get('jammed'), 'n_stuck': s.get('n_stuck'),
            'wall_s': s.get('wall_s'),
        })
    # 单步跳变（在去重后的原始序列上；CSV 的 50 Hz 重复不算）
    jumps = []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.6:
            continue
        dp = math.hypot(est[i][0] - est[i - 1][0], est[i][1] - est[i - 1][1])
        dyaw = abs(float(wrap_arr(np.array([yaw[i] - yaw[i - 1]]))[0]))
        v_imp = dp / dt
        w_imp = dyaw / dt
        if (dp > JUMP_POS and v_imp > JUMP_V) or (dyaw > JUMP_YAW and w_imp > JUMP_W):
            jumps.append({'sim_s': round(float(t[i]), 3), 'dt_s': round(float(dt), 3),
                          'dpos_m': round(dp, 4), 'dyaw_deg': round(math.degrees(dyaw), 3),
                          'v_imp_m_s': round(v_imp, 3), 'w_imp_rad_s': round(w_imp, 3),
                          'err_before_m': round(float(err[i - 1]), 4),
                          'err_after_m': round(float(err[i]), 4)})
    res['jumps'] = jumps
    return res, est, err


# ------------------------------------------------------------------ 相关性
def segment_diag(segs, cols, gt_t, x_gt, y_gt, cmd_t, cmd_w, rtf_t, rtf_v,
                 scan_t, nfin, fwd, motion_eps=0.02):
    """每段的"当时传感器/执行器/负载"体检：束数、前向最近回波、指令角速度、RTF、
    以及"发了速度但不动"的时间占比（卡死代理）。

    为什么必须和 ATE 放同一张表：ATE 只有配上"那一刻传感器看见了什么"才能做归因
    （否则"卡死/退化/快转"三个候选永远分不开）。
    """
    out = []
    for sg in segs:
        if sg.get('sim0') is None or sg.get('sim1') is None:
            continue
        a, b = sg['sim0'], sg['sim1']
        m = (gt_t >= a) & (gt_t <= b)
        row = {'label': sg['label'], 'group': sg.get('group'), 'kind': sg.get('kind'),
               'sim_s': round(b - a, 2)}
        if m.sum() >= 2:
            gx, gy = x_gt[m], y_gt[m]
            d = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))])
            row['gt_dist_m'] = round(float(d[-1]), 3)
            dt = np.diff(gt_t[m])
            sp = np.diff(d) / np.maximum(dt, 1e-6)
            row['gt_speed_mean'] = round(float(d[-1] / max(gt_t[m][-1] - gt_t[m][0], 1e-6)), 3)
        mm = (scan_t >= a) & (scan_t <= b)
        if mm.sum() >= 2:
            row['nfin_mean'] = int(np.nanmean(nfin[mm]))
            row['nfin_min'] = int(np.nanmin(nfin[mm]))
            f = fwd[mm]
            f = f[np.isfinite(f)]
            if len(f):
                row['fwd_p10'] = round(float(np.percentile(f, 10)), 3)
                row['fwd_p50'] = round(float(np.percentile(f, 50)), 3)
                row['fwd_lt1_5_frac'] = round(float((f < 1.5).mean()), 3)
        mc = (cmd_t >= a) & (cmd_t <= b)
        if mc.sum() >= 2:
            row['abs_cmd_w_mean'] = round(float(np.nanmean(np.abs(cmd_w[mc]))), 3)
        mr = (rtf_t >= a) & (rtf_t <= b)
        if mr.sum() >= 2:
            row['rtf_mean'] = round(float(np.nanmean(rtf_v[mr])), 3)
        out.append(row)
    return out


def window_corr(t, err, dist_gt, t_full, diag, win=1.0):
    """把误差增量与诊断量按 1 s 窗口聚合，返回相关性 + "坏窗口 vs 中位窗口"的对比。"""
    if len(t) < 10:
        return None
    t0, t1 = float(t[0]), float(t[-1])
    edges = np.arange(t0, t1 + win, win)
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (t >= a) & (t < b)
        if m.sum() < 2:
            continue
        e = err[m]
        de = float(e[-1] - e[0])
        # 该窗口内的行走距离
        dd = float(dist_gt[m][-1] - dist_gt[m][0]) if dist_gt is not None else NAN
        row = {'t': float(a), 'd_err': de, 'd_dist': dd,
               'rate': (de / dd) if dd and dd > 1e-3 else NAN}
        for k, (td, vd) in diag.items():
            mm = (td >= a) & (td < b) & np.isfinite(vd)
            row[k] = float(np.nanmean(vd[mm])) if mm.sum() else NAN
        row['n'] = int(m.sum())
        rows.append(row)
    if len(rows) < 8:
        return None
    de = np.array([r['d_err'] for r in rows])
    out = {'n_windows': len(rows), 'win_s': win,
           'd_err_median_m': round(float(np.median(de)), 4),
           'd_err_p90_m': round(float(np.percentile(de, 90)), 4),
           'd_err_max_m': round(float(de.max()), 4),
           'corr': {}, 'bad_vs_median': {}}
    # 只对"车在动"的窗口算相关性（停着不动时误差也会小，会污染相关）
    moving = np.array([r['d_dist'] for r in rows])
    sel = np.isfinite(moving) & (moving > 0.05)
    thresh = float(np.percentile(de[sel], 80)) if sel.sum() > 5 else float(np.percentile(de, 80))
    bad = sel & (de >= thresh)
    good = sel & ~bad
    for k in diag:
        v = np.array([r.get(k, NAN) for r in rows])
        ok = sel & np.isfinite(v)
        r = None
        if ok.sum() > 5 and np.std(v[ok]) > 1e-9 and np.std(de[ok]) > 1e-9:
            r = float(np.corrcoef(de[ok], v[ok])[0, 1])
        out['corr'][k] = None if r is None else round(r, 3)
        bm = np.nanmean(v[bad]) if bad.sum() else NAN
        gm = np.nanmean(v[good]) if good.sum() else NAN
        out['bad_vs_median'][k] = {
            'bad_mean': None if bm != bm else round(float(bm), 4),
            'other_mean': None if gm != gm else round(float(gm), 4),
            'n_bad': int(bad.sum()), 'n_other': int(good.sum()),
        }
    out['windows'] = rows
    return out


# ------------------------------------------------------------------ 画图
def make_figs(tag, out_dir, common, series, events, segs, diag_series, dist_gt, jumps_cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    # 中文字体：本机有 Noto Sans CJK / WenQuanYi Micro Hei / AR PL UMing CN。
    # 不设的话所有中文都渲染成方框 —— 图就白出了。
    matplotlib.rcParams['font.sans-serif'] = [
        'Noto Sans CJK JP', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei',
        'AR PL UMing CN', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    os.makedirs(out_dir, exist_ok=True)
    paths = []
    colors = {'lio': '#1f77b4', 'fused': '#d62728', 'tfb': '#2ca02c'}
    labels = {'lio': 'LIO /odom (对齐后)', 'fused': 'MAP→BASE_LINK 融合 (对齐后)',
              'tfb': 'odom→base_link TF (对齐后)'}
    # 段底色
    band = {'straight': '#d9edf7', 'return': '#dff0d8', 'slow_turn': '#fcf8e3',
            'fast_turn': '#f2dede', 'turn180': '#eeeeee', 'align': '#eeeeee',
            'prep': '#ffffff', 'narrow': '#e8e0f0', 'ramp_probe': '#f5d0d0'}
    gt = common['gt_t']

    # ---------------- Fig 1: error vs time / vs distance
    fig, axes = plt.subplots(2, 1, figsize=(15, 10))
    ax = axes[0]
    for s in segs:
        if s['sim0'] is None:
            continue
        c = band.get(s.get('group') or '', '#ffffff')
        ax.axvspan(s['sim0'], s['sim1'], color=c, zorder=0)
        ax.text((s['sim0'] + s['sim1']) / 2, 0.98, s['label'], ha='center', va='top',
                transform=ax.get_xaxis_transform(), fontsize=8, color='#333333')
    # 画序固定：tfb 最底、fused 中间、lio 最上 —— 这样三条都看得见
    # （`/odom` 与 `odom→base_link` 的 ATE 本来就应当逐位相同，重合是"装配一致"的证据，
    #   不能因为它被盖住就以为少画了一条）
    for name in ('tfb', 'fused', 'lio'):
        if name not in common['series']:
            continue
        ax.plot(common['series'][name]['t'], common['series'][name]['err'],
                '-' if name != 'tfb' else '--',
                lw=1.6 if name != 'tfb' else 1.1,
                color=colors.get(name), label=labels.get(name, name),
                zorder={'tfb': 2, 'fused': 3, 'lio': 4}[name])
    for j in series.get('__jumps__', []):
        ax.axvline(j['sim_s'], color='red', ls=':', lw=0.9, alpha=0.8)
    for e in events:
        if e.get('event') == 'jam':
            ax.axvline(float(e['sim']), color='black', ls='--', lw=1.2)
            ax.annotate('JAM', (float(e['sim']), ax.get_ylim()[1] * 0.6),
                        fontsize=8, color='black', rotation=90)
    ax.set_xlabel('仿真时间 (s)')
    ax.set_ylabel('位置误差 vs 真值 (m)')
    ax.set_title('%s —— 误差随时间（底纹=路线分段；红点线=单步跳变 >%.2f m 且 >%.1f m/s；'
                 '黑虚线=卡死）' % (tag, JUMP_POS, JUMP_V))
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)
    ax2 = ax.twinx()
    ax2.plot(gt, dist_gt, color='#888888', lw=1.0, alpha=0.7)
    ax2.set_ylabel('累计真值里程 (m)', color='#888888')

    ax = axes[1]
    for s in segs:
        if s['sim0'] is None:
            continue
        m = (gt >= s['sim0']) & (gt <= s['sim1'])
        if m.sum() < 2:
            continue
        c = band.get(s.get('group') or '', '#ffffff')
        ax.axvspan(dist_gt[m][0], dist_gt[m][-1], color=c, zorder=0)
        ax.text((dist_gt[m][0] + dist_gt[m][-1]) / 2, 0.98, s['label'], ha='center',
                va='top', transform=ax.get_xaxis_transform(), fontsize=8, color='#333333')
    for name in ('tfb', 'fused', 'lio'):
        if name not in common['series']:
            continue
        d = common['series'][name]
        gg = np.interp(d['t'], gt, dist_gt)
        ax.plot(gg, d['err'], '-' if name != 'tfb' else '--',
                lw=1.6 if name != 'tfb' else 1.1, color=colors.get(name),
                label=labels.get(name, name),
                zorder={'tfb': 2, 'fused': 3, 'lio': 4}[name])
    ax.set_xlabel('累计真值里程 (m)')
    ax.set_ylabel('位置误差 vs 真值 (m)')
    ax.set_title('误差随里程（"每米漂多少"看这条的斜率）')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    p = os.path.join(out_dir, '%s_error_vs_time_distance.png' % tag)
    fig.savefig(p, dpi=110)
    plt.close(fig)
    paths.append(p)

    # ---------------- Fig 2: diagnostics
    fig, axes = plt.subplots(4, 1, figsize=(15, 12), sharex=True)
    ax = axes[0]
    for s in segs:
        if s['sim0'] is not None:
            ax.axvspan(s['sim0'], s['sim1'], color=band.get(s.get('group') or '', '#ffffff'))
    ax.plot(diag_series['t'], diag_series['nfin'], '-', lw=0.8, color='#333')
    ax.set_ylabel('/scan 有限束数')
    ax.set_title('%s —— 诊断量（同一时间轴；灰底纹=分段）' % tag)
    ax.grid(alpha=0.3)
    ax = axes[1]
    for s in segs:
        if s['sim0'] is not None:
            ax.axvspan(s['sim0'], s['sim1'], color=band.get(s.get('group') or '', '#ffffff'))
    ax.plot(diag_series['t'], diag_series['fwd_min'], '.', ms=1.5, color='#d62728')
    ax.set_yscale('log')
    ax.set_ylabel('/scan 前向(±20°)最近回波 (m)')
    ax.grid(alpha=0.3)
    ax = axes[2]
    for s in segs:
        if s['sim0'] is not None:
            ax.axvspan(s['sim0'], s['sim1'], color=band.get(s.get('group') or '', '#ffffff'))
    ax.plot(diag_series['t'], diag_series['cmd_w'], '-', lw=0.7, color='#1f77b4',
            label='指令 ω')
    ax.plot(diag_series['t'], diag_series['lio_wz'], '-', lw=0.7, color='#ff7f0e',
            alpha=0.7, label='LIO 报告 ω')
    ax.set_ylabel('角速度 (rad/s)')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax = axes[3]
    for s in segs:
        if s['sim0'] is not None:
            ax.axvspan(s['sim0'], s['sim1'], color=band.get(s.get('group') or '', '#ffffff'))
    ax.plot(diag_series['t'], diag_series['rtf'], '-', lw=0.8, color='#2ca02c')
    ax.set_ylabel('RTF（3 s 窗）')
    ax.set_xlabel('仿真时间 (s)')
    ax.grid(alpha=0.3)
    # 跳变 stem
    axj = axes[0].twinx()
    for j in series.get('__jumps__', []):
        axj.plot([j['sim_s']], [j['dpos_m']], 'v', color='red', ms=4)
    axj.set_ylabel('单步 Δpos (m)', color='red')
    fig.tight_layout()
    p = os.path.join(out_dir, '%s_diagnostics.png' % tag)
    fig.savefig(p, dpi=110)
    plt.close(fig)
    paths.append(p)

    # ---------------- Fig 3: XY 轨迹
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(common['gt_xy'][:, 0], common['gt_xy'][:, 1], '-', color='#333333', lw=1.8,
            label='真值')
    for name in ('lio', 'fused', 'tfb'):
        if name not in common['xy']:
            continue
        ax.plot(common['xy'][name][:, 0], common['xy'][name][:, 1], '--', lw=1.2,
                color=colors.get(name), label=labels.get(name, name))
    for e in events:
        if e.get('event') == 'jam':
            ax.plot([float(e['pose'][0])], [float(e['pose'][1])], 'kx', ms=10)
    ax.plot([0], [0], 'g*', ms=16, label='出生点 (0,0)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    ax.set_title('%s —— XY 轨迹（对齐后）' % tag)
    ax.legend(fontsize=9)
    fig.tight_layout()
    p = os.path.join(out_dir, '%s_traj_xy.png' % tag)
    fig.savefig(p, dpi=110)
    plt.close(fig)
    paths.append(p)
    return paths


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='segment_drive.py 的分析：分段 ATE / 跳变 / 相关性')
    ap.add_argument('--csv', required=True, help='segment_drive.py 的 --out-csv')
    ap.add_argument('--events', required=True, help='segment_drive.py 的 --out-json')
    ap.add_argument('--out-json', help='分析结果 JSON 输出路径')
    ap.add_argument('--out-dir', default='docs/img', help='PNG 输出目录')
    ap.add_argument('--tag', default='lio_drift', help='图名前缀')
    ap.add_argument('--no-figs', action='store_true', help='不画图，只出数字')
    ap.add_argument('--align-samples', type=int, default=40,
                    help='用静止窗口的头多少对样本估对齐（默认 40 ≈ 4 s @10 Hz）')
    args = ap.parse_args()

    cols, nrows = read_csv(args.csv)
    meta = json.load(open(args.events))
    events = meta.get('events', [])
    gmap = {l['label']: l.get('group')
            for l in meta.get('spec', {}).get('legs', [])}
    segs_ev = segments_from_events(events, gmap)
    print('[ana] CSV %s：%d 行；段数=%d' % (args.csv, nrows, len(segs_ev)))

    # ---- 静止窗口
    w0 = next((float(e['sim']) for e in events
               if e.get('event') == 'link_ready' and e.get('sim') is not None), None)
    w1 = next((float(e['sim']) for e in events
               if e.get('event') == 'warmup_end' and e.get('sim') is not None), None)
    if w0 is None or w1 is None:
        w0 = float(np.nanmin(cols['sim_t']))
        w1 = w0 + 5.0
        print('[ana] ⚠️ 事件表里没有 link_ready/warmup_end，用 sim 区间 [%.1f, %.1f] 当静止窗'
              % (w0, w1))

    sim = cols['sim_t']
    # 采样健康度（"采样饥饿"结构化判据）
    health = {}
    for k, c in (('lio', 'lio_age_sim'), ('gt', 'gt_age_sim'),
                 ('scan', 'scan_age_sim'), ('tf_odom_base', 'tfo_age_sim'),
                 ('tf_map_odom', 'tfm_age_sim')):
        v = cols.get(c)
        if v is None:
            continue
        v = v[np.isfinite(v)]
        if len(v) == 0:
            continue
        health[k] = {'n': int(len(v)), 'mean_s': round(float(v.mean()), 4),
                     'p99_s': round(float(np.percentile(v, 99)), 4),
                     'max_s': round(float(v.max()), 4),
                     'over_%ss' % AGE_MAX: int((v > AGE_MAX).sum())}
    print('[ana] 采样健康度（数据有多旧）: %s' % json.dumps(health, ensure_ascii=False))

    # ---- 去重后的各序列
    t_lio, x_lio, y_lio, z_lio, yaw_lio, vx, vy, wz = dedupe(
        cols['lio_t'], cols['lio_x'], cols['lio_y'], cols['lio_z'], cols['lio_yaw'],
        cols['lio_vx'], cols['lio_vy'], cols['lio_wz'])
    t_gt, x_gt, y_gt, z_gt, yaw_gt = dedupe(
        cols['gt_t'], cols['gt_x'], cols['gt_y'], cols['gt_z'], cols['gt_yaw'])
    print('[ana] 去重后：/odom %d 条，/odom_ground_truth %d 条' % (len(t_lio), len(t_gt)))
    if len(t_lio) < 20 or len(t_gt) < 20:
        print('[ana] ❌ 样本太少')
        return 2

    # 真值累计里程（按去重后的真值时间轴）
    dist_gt = np.zeros(len(t_gt))
    if len(t_gt) > 1:
        dist_gt[1:] = np.cumsum(np.linalg.norm(
            np.diff(np.stack([x_gt, y_gt], axis=1), axis=0), axis=1))
    # RTF 等也放到真值时间轴
    rtf_full = np.interp(t_gt, sim[np.isfinite(sim)], cols['rtf'][np.isfinite(sim)]
                         if np.isfinite(cols['rtf']).any() else np.zeros(len(sim)))

    # ---- 静止窗口内的对齐
    inw = (t_lio >= w0) & (t_lio <= w1)
    if inw.sum() < 10:
        print('[ana] ⚠️ 静止窗内 /odom 样本只有 %d 条，放宽到前 5 s' % inw.sum())
        inw = t_lio <= (t_lio[0] + 5.0)
    p_lio0 = pair_to_gt(t_lio[inw], x_lio[inw], y_lio[inw], yaw_lio[inw],
                        t_gt, x_gt, y_gt, yaw_gt)
    if p_lio0 is None:
        print('[ana] ❌ 静止窗内配不上真值')
        return 2
    k = min(args.align_samples, len(p_lio0[0]) // 2)
    k = max(k, 5)
    src0 = (p_lio0[1], p_lio0[2], p_lio0[3])
    gt0 = (p_lio0[4], p_lio0[5], p_lio0[6])
    yaw_lio_off, t_lio_off, R_lio = align_from(src0, gt0, k)
    print('[ana] LIO 对齐（用静止窗 %d 对样本）：yaw_off=%.2f°  t=(%.3f, %.3f)'
          % (k, math.degrees(yaw_lio_off), t_lio_off[0], t_lio_off[1]))

    series = {}
    # LIO
    p = pair_to_gt(t_lio, x_lio, y_lio, yaw_lio, t_gt, x_gt, y_gt, yaw_gt)
    lio_gt = (p[4], p[5], p[6], p[7])
    res_lio, est_lio, err_lio = analyze_series(
        'lio', p[0], p[1], p[2], p[3], lio_gt, R_lio, t_lio_off,
        segs_ev, yaw_lio_off)
    series['lio'] = {'t': p[0], 'err': err_lio, 'xy': est_lio, 'yaw': p[3],
                     'raw': (p[0], p[1], p[2], p[3])}

    # odom→base_link TF（与 /odom 应当一致；不一致就是装配问题）
    align = {}
    if np.isfinite(cols['tfo_t']).sum() > 20:
        t_tfo, x_tfo, y_tfo, yaw_tfo = dedupe(cols['tfo_t'], cols['tfo_x'],
                                              cols['tfo_y'], cols['tfo_yaw'])
        p2 = pair_to_gt(t_tfo, x_tfo, y_tfo, yaw_tfo, t_gt, x_gt, y_gt, yaw_gt)
        if p2 is not None:
            # 用与 /odom 相同的静止窗口估对齐
            m0 = (p2[0] >= w0) & (p2[0] <= w1)
            if m0.sum() >= 5:
                yy, tt, RR = align_from((p2[1][m0], p2[2][m0], p2[3][m0]),
                                        (p2[4][m0], p2[5][m0], p2[6][m0]),
                                        min(k, m0.sum()))
                res_tfb, est_tfb, err_tfb = analyze_series(
                    'tfb', p2[0], p2[1], p2[2], p2[3],
                    (p2[4], p2[5], p2[6], p2[7]), RR, tt, [], yy)
                series['tfb'] = {'t': p2[0], 'err': err_tfb, 'xy': est_tfb, 'yaw': p2[3]}
                align['tfb'] = {'yaw_off_deg': round(math.degrees(yy), 3),
                                't': [round(float(tt[0]), 4), round(float(tt[1]), 4)]}

    # 融合 map→base_link
    res_fused = None
    if np.isfinite(cols['fus_x']).sum() > 20:
        # 融合位姿是"各自最新"的两条 TF 复合出来的：记下此刻两条边的时间差，太旧的丢掉
        fm = np.isfinite(cols['fus_x']) & np.isfinite(cols['tfm_t']) & \
            np.isfinite(cols['tfo_t']) & np.isfinite(sim) & \
            (np.abs(cols['tfm_t'] - cols['tfo_t']) < 1.0)
        t_fus = sim[fm]
        x_fus = cols['fus_x'][fm]
        y_fus = cols['fus_y'][fm]
        yaw_fus = cols['fus_yaw'][fm]
        # ⚠️ 融合位姿是**分段常数**（map→odom 只在 slam 处理完一帧时变，≈2 Hz；
        #    odom→base_link ≈10 Hz 变）。50 Hz 重复采样会把"同一个状态"记几十遍：
        #    ① 让 ATE 的统计量被"停留时间"加权（慢速/静止时段的权重被放大）；
        #    ② 让单步跳变检测看不出来（跳变被稀释）。
        #    ⇒ 按**复合位姿的取值**去重，只保留"状态真的变了"的那些行。
        #    （不能按 (tfm_t, tfo_t) 去重：slam_toolbox 以 50 Hz **重发**同一个 map→odom，
        #      时间戳一直在走、值不变 —— 自测里那样去重一条都压不掉。）
        key = np.stack([x_fus, y_fus, yaw_fus], axis=1)
        if len(key) > 1:
            kf = np.ones(len(key), dtype=bool)
            kf[1:] = np.any(np.abs(np.diff(key, axis=0)) > 1e-9, axis=1)
            t_fus, x_fus, y_fus, yaw_fus = (t_fus[kf], x_fus[kf], y_fus[kf], yaw_fus[kf])
        print('[ana] 融合位姿：50 Hz 行 %d → 状态去重后 %d 条' % (int(fm.sum()), len(t_fus)))
        p3 = pair_to_gt(t_fus, x_fus, y_fus, yaw_fus, t_gt, x_gt, y_gt, yaw_gt)
        if p3 is not None:
            m0 = (p3[0] >= w0) & (p3[0] <= w1)
            kk = max(5, min(k, int(m0.sum())))
            yy, tt, RR = align_from((p3[1][m0], p3[2][m0], p3[3][m0]),
                                    (p3[4][m0], p3[5][m0], p3[6][m0]), kk)
            res_fused, est_fus, err_fus = analyze_series(
                'fused', p3[0], p3[1], p3[2], p3[3],
                (p3[4], p3[5], p3[6], p3[7]), RR, tt,
                segs_ev, yy)
            series['fused'] = {'t': p3[0], 'err': err_fus, 'xy': est_fus, 'yaw': p3[3]}
            align['fused'] = {'yaw_off_deg': round(math.degrees(yy), 3),
                              't': [round(float(tt[0]), 4), round(float(tt[1]), 4)],
                              'kind': 'map→odom ∘ odom→base_link（手动复合）'}
    if res_fused is None:
        print('[ana] ⚠️ 没有可用的融合位姿（map→odom 没来？）')

    # ---- 相关性（逐秒窗口的误差增量 vs 诊断量）
    diag = {}
    tscan, nfin, fwd = dedupe(cols['scan_t'], cols['scan_nfin'], cols['scan_fwd_min'])
    if len(tscan) > 5:
        diag['scan_nfin'] = (tscan, nfin)
        diag['scan_fwd_min'] = (tscan, fwd)
    tcmd = sim[np.isfinite(sim)]
    diag['abs_cmd_w'] = (tcmd, np.abs(cols['cmd_w'][np.isfinite(sim)]))
    diag['rtf'] = (tcmd, cols['rtf'][np.isfinite(sim)])
    # 误差按真值时间轴插值，便于和诊断量对齐
    err_lio_on_gt = np.interp(t_gt, series['lio']['t'], series['lio']['err'])
    dist_lio = dist_gt
    corr = window_corr(t_gt, err_lio_on_gt, dist_lio, t_gt, diag, win=1.0)
    corr_fused = None
    if 'fused' in series:
        err_f_on_gt = np.interp(t_gt, series['fused']['t'], series['fused']['err'])
        corr_fused = window_corr(t_gt, err_f_on_gt, dist_gt, t_gt, diag, win=1.0)

    # ---- 融合是否有界
    bounded = None
    if res_fused and res_lio:
        n = min(len(series['lio']['err']), len(series['fused']['err']))
        a = max(1, n // 3)
        bounded = {
            'lio_err_head_m': round(float(series['lio']['err'][:a].mean()), 4),
            'lio_err_tail_m': round(float(series['lio']['err'][-a:].mean()), 4),
            'fused_err_head_m': round(float(series['fused']['err'][:a].mean()), 4),
            'fused_err_tail_m': round(float(series['fused']['err'][-a:].mean()), 4),
            'lio_err_max_m': res_lio['ate_max_m'], 'fused_err_max_m': res_fused['ate_max_m'],
            'ratio_max_fused_over_lio': round(res_fused['ate_max_m'] /
                                              max(res_lio['ate_max_m'], 1e-9), 3),
            'ratio_tail_fused_over_lio': round(float(series['fused']['err'][-a:].mean()) /
                                               max(float(series['lio']['err'][-a:].mean()), 1e-9), 3),
            'fused_final_m': res_fused['ate_final_m'],
        }

    # ---- 汇总打印
    def dump_table(res, title):
        print('\n=== %s ===' % title)
        print('  整程: n=%d  ATE RMSE=%.4f m  mean=%.4f  max=%.4f m@%.1fs  final=%.4f m  '
              'yaw RMSE=%.3f°  LIO 里程=%.2f m'
              % (res['n_pairs'], res['ate_rmse_m'], res['ate_mean_m'], res['ate_max_m'],
                 res['ate_max_sim_s'], res['ate_final_m'], res['yaw_err_rmse_deg'],
                 res['traj_len_m']))
        print('  %-6s %-10s %6s %7s %7s %7s %8s %9s %9s %8s %s'
              % ('seg', 'group', 'T(s)', 'gt(m)', 'est(m)', 'RMSE', 'max', 'err_grow',
                 'RPE_fin', 'm/m', '备注'))
        for s in res['segments']:
            if 'note' in s:
                print('  %-6s %s' % (s['label'], s['note']))
                continue
            print('  %-6s %-10s %6.1f %7.2f %7.2f %7.4f %8.4f %9.4f %9.4f %8s %s'
                  % (s['label'], s.get('group'), s['sim_s'], s['gt_dist_m'],
                     s['est_dist_m'], s['ate_rmse_m'], s['ate_max_m'],
                     s['err_growth_m'], s['rpe_final_m'],
                     ('%.4f' % s['drift_m_per_m']) if s['drift_m_per_m'] is not None else '-',
                     ('JAM' if s.get('jammed') else '')))
        if res['jumps']:
            print('  单步跳变 %d 次（>%.2f m 或 >10°）：' % (len(res['jumps']), JUMP_POS))
            for j in res['jumps'][:20]:
                print('    t=%.2f s  Δpos=%.3f m  Δyaw=%.2f°  err %.3f→%.3f'
                      % (j['sim_s'], j['dpos_m'], j['dyaw_deg'],
                         j['err_before_m'], j['err_after_m']))
        else:
            print('  单步跳变：无')

    dump_table(res_lio, 'LIO /odom（对齐后 vs 真值）')
    if res_fused:
        dump_table(res_fused, '融合 map→base_link（对齐后 vs 真值）')
    if 'tfb' in series:
        print('\n=== odom→base_link TF ===  整程 ATE RMSE=%.4f m  max=%.4f m  '
              '（与 /odom 的差 = 装配一致性检查）'
              % (res_tfb['ate_rmse_m'], res_tfb['ate_max_m']))
    if bounded:
        print('\n=== 融合是否有界 ===\n  %s' % json.dumps(bounded, ensure_ascii=False))
    if corr:
        print('\n=== 逐秒窗口误差增量 vs 诊断量（只看"在动"的窗口）===')
        print('  窗口数=%d  中位 Δerr=%.4f m  p90=%.4f  max=%.4f'
              % (corr['n_windows'], corr['d_err_median_m'], corr['d_err_p90_m'],
                 corr['d_err_max_m']))
        for k in corr['corr']:
            bv = corr['bad_vs_median'][k]
            print('  %-14s r=%6s   坏窗口均值=%8s  其余窗口均值=%8s  (n=%d/%d)'
                  % (k, corr['corr'][k], bv['bad_mean'], bv['other_mean'],
                     bv['n_bad'], bv['n_other']))

    # ---- 画图
    figs = []
    if not args.no_figs:
        segs_plot = segs_ev
        jumps_all = list(res_lio['jumps'])
        if res_fused:
            jumps_all += res_fused['jumps']
        common = {'gt_t': t_gt, 'gt_xy': np.stack([x_gt, y_gt], axis=1),
                  'series': {k: {'t': v['t'], 'err': v['err']} for k, v in series.items()},
                  'xy': {k: v['xy'] for k, v in series.items()}}
        diag_series = {'t': tscan, 'nfin': nfin, 'fwd_min': fwd,
                       'cmd_w': np.interp(tscan, tcmd, cols['cmd_w'][np.isfinite(sim)]),
                       'lio_wz': np.interp(tscan, t_lio, wz) if len(t_lio) else tscan * 0,
                       'rtf': np.interp(tscan, tcmd, cols['rtf'][np.isfinite(sim)])}
        try:
            figs = make_figs(args.tag, args.out_dir, common,
                             dict(series, __jumps__=jumps_all),
                             events, segs_plot, diag_series, dist_gt, {})
            for p in figs:
                print('[ana] 图 → %s' % p)
        except Exception as e:  # noqa: BLE001
            print('[ana] ⚠️ 画图失败（数字仍然有效）：%r' % e)

    seg_diag = segment_diag(segs_ev, cols, t_gt, x_gt, y_gt, tcmd,
                            cols['cmd_w'][np.isfinite(sim)], tcmd,
                            cols['rtf'][np.isfinite(sim)], tscan, nfin, fwd)
    print('\n=== 每段的当时体检（传感器/执行器/负载）===')
    print('  %-6s %-10s %6s %7s %8s %9s %8s %8s %7s %8s'
          % ('seg', 'group', 'T(s)', 'gt(m)', '束数均值', '束数最小', '前向p10',
             '前向p50', '<1.5m占比', '|cmdω|'))
    for r in seg_diag:
        if r['label'] in ('S',):
            pass
        print('  %-6s %-10s %6.1f %7s %8s %9s %8s %8s %8s %8s'
              % (r['label'], r.get('group'), r['sim_s'],
                 r.get('gt_dist_m', '-'), r.get('nfin_mean', '-'),
                 r.get('nfin_min', '-'), r.get('fwd_p10', '-'),
                 r.get('fwd_p50', '-'), r.get('fwd_lt1_5_frac', '-'),
                 r.get('abs_cmd_w_mean', '-')))

    out = {
        'segment_diagnostics': seg_diag,
        'csv': os.path.abspath(args.csv), 'events': os.path.abspath(args.events),
        'n_csv_rows': nrows, 'sampling_health': health,
        'align': {'lio': {'yaw_off_deg': round(math.degrees(yaw_lio_off), 3),
                          't': [round(float(t_lio_off[0]), 4), round(float(t_lio_off[1]), 4)],
                          'n_samples': k, 'window_sim': [w0, w1]},
                  **align},
        'thresholds': {'jump_pos_m': JUMP_POS, 'jump_yaw_deg': math.degrees(JUMP_YAW),
                       'jump_v_m_s': JUMP_V, 'jump_w_rad_s': JUMP_W,
                       'pair_dt_s': PAIR_DT, 'age_max_s': AGE_MAX},
        'lio': res_lio, 'fused': res_fused,
        'tfb': res_tfb if 'tfb' in series else None,
        'bounded': bounded, 'corr_lio': corr, 'corr_fused': corr_fused,
        'figs': figs, 'legs': meta.get('spec', {}).get('legs'),
        'drive': meta.get('drive'), 'warnings': meta.get('warnings'),
        'link': meta.get('link'),
    }
    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)) or '.', exist_ok=True)
        with open(args.out_json, 'w') as f:
            json.dump(out, f, indent=1, ensure_ascii=False, default=str)
        print('[ana] 写出 %s' % args.out_json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
