#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线归因：把「丢帧 / 回环(Ceres) / TF 等待 / CPU / RTF」放到**同一条仿真时间轴**上。

输入 = `run_dropab.sh` 一次跑出来的四份证据（全部只读）：
  · `<tag>.timing.json` / `<tag>.timing.npz`   scan_tf_timing_probe.py（时间戳链实测）
  · `<tag>.rec.jsonl`                          rec_sltune.py（位姿图 V/E、各进程 CPU、RTF）
  · `launch.log`                               整栈 stdout/stderr（**Ceres 的 glog 行在这里**）
  · `<tag>.ssl.log`                            slam_toolbox 节点日志（丢帧行，干净、不含 rviz）
  · `<tag>.drive.json`                         coverage_drive.py（路线/速度/卡死）

三条**关键事实**（源码级，见 docs/slam_drops_loopclosure_odom.md §1/§2），本脚本据此设计：
  1. `ceres::Solve` 在整个 slam_toolbox 里**只有**一条调用路径：
     `MapperGraph::TryCloseLoop()` → `CorrectPoses()` → `solver_->Compute()`
     （`lib/karto_sdk/src/Mapper.cpp:1549,2012-2017`；`slam_toolbox_common.cpp:778` 那处只在
     反序列化时用）。⇒ **每一条 `preprocessor.cc:62` 行 = 一次被接受的回环**，
     它的时间戳就是"图优化开始"的时刻。这就是"回环事件"的直接、带时刻的代理量。
  2. 丢帧行里 `at time X` 的 X 是**被顶掉的那条扫描**的 `header.stamp`（= 仿真时间）
     ⇒ 丢帧可以与逐帧的 TF 等待、CPU、回环**逐帧对齐**，不需要靠墙钟换算。
  3. `QueueFull` 的判据（tf2 语义）：该扫描到达时 TF 还查不到，且**等待时间 > 下一条扫描的
     到达间隔**（队列深度 1）。本脚本对每一帧都算这个判据，并和真实丢帧行对账。

用法：
  python3 tools/scripts/mapping/sltune/analyze_dropab.py --arm a_dense=.tmp_cache/dropab/a_dense \
      [--arm b_coarse=... ] [--json out.json] [--md out.md] [--png docs/img/xxx.png]
"""
from __future__ import annotations

import argparse
import datetime as _dt
import glob
import json
import math
import os
import re
import statistics as st
import sys
import time

import numpy as np

CERES_RE = re.compile(r'preprocessor\.cc:62')
DROP_RE = re.compile(r"queue is full")
DROP_TIME_RE = re.compile(r'at time ([0-9.]+)')
GLOG_RE = re.compile(r'\bW[0-9]{4} ([0-9:.]+)\s')
LEAD_RE = re.compile(r'^\s*([0-9]{9,}\.[0-9]+)')
EPOCH_RE = re.compile(r'^\s*([0-9]{9,}\.[0-9]+)')


# --------------------------------------------------------------------------- 小工具
def load_npz(path):
    try:
        z = np.load(path, allow_pickle=True)
        return {k: z[k] for k in z.files}
    except Exception as e:
        print('[warn] npz 打不开 %s: %r' % (path, e), file=sys.stderr)
        return {}


def load_jsonl(path, key):
    out = []
    try:
        with open(path) as f:
            for line in f:
                try:
                    d = json.loads(line)
                except Exception:
                    continue
                if d.get('k') == key:
                    out.append(d)
    except Exception:
        pass
    return out


def pct(x, q):
    x = np.asarray([v for v in np.asarray(x, dtype=float) if np.isfinite(v)])
    return float(np.percentile(x, q)) if x.size else float('nan')


def stats(x):
    x = np.asarray([v for v in np.asarray(x, dtype=float) if np.isfinite(v)])
    if x.size == 0:
        return None
    return {'n': int(x.size), 'p50': round(pct(x, 50), 4), 'p90': round(pct(x, 90), 4),
            'p99': round(pct(x, 99), 4), 'max': round(float(x.max()), 4),
            'mean': round(float(x.mean()), 4)}


def interp_sim_of_wall(clocks, wall):
    """clocks: (K,2) = (sim, wall_monotonic)。墙钟 → 仿真时间（分段线性）。"""
    if clocks is None or len(clocks) < 3:
        return None
    c = np.asarray(clocks, dtype=float)
    o = np.argsort(c[:, 1])
    return lambda w: float(np.interp(w, c[o, 1], c[o, 0]))


# --------------------------------------------------------------------------- 一次跑
def parse_ceres(launch_log, rt_minus_mono, sim_of_wall, t_scan_first_sim):
    """返回 [(sim_time, wall_epoch, raw)]：每条 = 一次被接受的回环（图优化开始）。"""
    out = []
    if not os.path.exists(launch_log):
        return out
    with open(launch_log, errors='replace') as f:
        for line in f:
            if not CERES_RE.search(line):
                continue
            m = LEAD_RE.match(line)          # `ros2 launch` 给每行的墙钟前缀（epoch）
            if m:
                w = float(m.group(1))
            else:
                g = GLOG_RE.search(line)      # glog 自带的 HH:MM:SS.micro（当天）
                if not g:
                    continue
                hh, mm, ss = g.group(1).split(':')
                today = _dt.datetime.fromtimestamp(time.time()).date()
                t = _dt.datetime.combine(today, _dt.time(int(hh), int(mm))) + \
                    _dt.timedelta(seconds=float(ss))
                w = t.timestamp()
            mono = w - rt_minus_mono
            s = sim_of_wall(mono) if sim_of_wall else None
            out.append({'wall_epoch': round(w, 4),
                        'sim': (round(s, 3) if s is not None else None),
                        'raw': line.strip()[:160]})
    # 去重（同一 Solve 只有一行，但防御性去重：1 ms 内相同视为同一条）
    ded = []
    for e in sorted(out, key=lambda d: d['wall_epoch']):
        if ded and e['wall_epoch'] - ded[-1]['wall_epoch'] < 1e-3:
            continue
        ded.append(e)
    if t_scan_first_sim is not None:
        for e in ded:
            e['before_first_scan'] = bool(e['sim'] is not None and e['sim'] < t_scan_first_sim)
    return ded


def parse_drops(ssl_log, launch_log):
    """丢帧的**扫描戳**（仿真时间）。优先用节点日志（不含 rviz），退回整栈日志。"""
    for p in (ssl_log, launch_log):
        if not os.path.exists(p):
            continue
        got = []
        with open(p, errors='replace') as f:
            for line in f:
                if DROP_RE.search(line):
                    m = DROP_TIME_RE.search(line)
                    if m:
                        got.append(float(m.group(1)))
        if got:
            return sorted(got), p
    return [], None


def arm_report(name, d, rt_minus_mono):
    r = {'arm': name, 'dir': os.path.abspath(d)}
    npz = load_npz(os.path.join(d, '%s.timing.npz' % name))
    tj = {}
    try:
        tj = json.load(open(os.path.join(d, '%s.timing.json' % name)))
    except Exception:
        pass
    rec = load_jsonl(os.path.join(d, '%s.rec.jsonl' % name), 'sys')
    graphs = load_jsonl(os.path.join(d, '%s.rec.jsonl' % name), 'graph')
    poses = load_jsonl(os.path.join(d, '%s.rec.jsonl' % name), 'pose')
    maps = load_jsonl(os.path.join(d, '%s.rec.jsonl' % name), 'map')
    try:
        drive = json.load(open(os.path.join(d, '%s.drive.json' % name)))
    except Exception:
        drive = {}
    try:
        params = open(os.path.join(d, 'params.txt'), errors='replace').read()
    except Exception:
        params = ''
    try:
        cfgdiff = open(os.path.join(d, 'cfg_diff.txt'), errors='replace').read()
    except Exception:
        cfgdiff = ''

    clocks = npz.get('clocks')
    sim_of_wall = interp_sim_of_wall(clocks, None)
    stamps = np.asarray(npz.get('stamps', []), dtype=float)
    sim_arr = np.asarray(npz.get('sim_arr', []), dtype=float)
    wall_arr = np.asarray(npz.get('wall_arr', []), dtype=float)
    tf_ok = np.asarray(npz.get('tf_ok', []), dtype=bool)
    wait_sim = np.asarray(npz.get('wait_sim', []), dtype=float)
    wait_wall = np.asarray(npz.get('wait_wall', []), dtype=float)
    tf_ob = np.asarray(npz.get('tf_ob', np.zeros((0, 3))), dtype=float)
    tf_mo = np.asarray(npz.get('tf_mo', np.zeros((0, 3))), dtype=float) \
        if 'tf_mo' in npz else np.zeros((0, 3))

    t_scan_first = float(stamps.min()) if stamps.size else None
    drops, drop_src = parse_drops(os.path.join(d, '%s.ssl.log' % name),
                                 os.path.join(d, 'launch.log'))
    ceres = parse_ceres(os.path.join(d, 'launch.log'), rt_minus_mono, sim_of_wall, t_scan_first)
    ceres = [e for e in ceres if not e.get('before_first_scan')]

    # ---- 每帧：QueueFull 判据（等待 > 下一条扫描的到达间隔，墙钟口径更接近 tf2 的真实语义）
    gap_wall = np.diff(wall_arr, append=np.nan) if wall_arr.size else np.zeros(0)
    if gap_wall.size:
        gap_wall[-1] = np.nan
    gap_sim = np.diff(sim_arr, append=np.nan) if sim_arr.size else np.zeros(0)
    pred = np.zeros(stamps.size, dtype=bool)
    for i in range(max(stamps.size - 1, 0)):
        if not tf_ok[i] and np.isfinite(wait_wall[i]):
            pred[i] = (wait_wall[i] > gap_wall[i]) or (wait_sim[i] > gap_sim[i])
    wait_eff = np.where(np.isfinite(wait_wall), wait_wall, 0.0)

    # ---- 丢帧 → 逐帧对齐
    drop_rows = []
    if stamps.size:
        for s in drops:
            j = int(np.argmin(np.abs(stamps - s)))
            hit = abs(stamps[j] - s) < 0.03
            row = {'scan_stamp': s, 'matched_scan': bool(hit)}
            if hit:
                row.update({'tf_ok_at_arrival': bool(tf_ok[j]),
                            'wait_wall_ms': (round(1000 * float(wait_wall[j]), 2)
                                             if np.isfinite(wait_wall[j]) else None),
                            'wait_sim_ms': (round(1000 * float(wait_sim[j]), 2)
                                            if np.isfinite(wait_sim[j]) else None),
                            'gap_wall_ms': (round(1000 * float(gap_wall[j]), 2)
                                            if np.isfinite(gap_wall[j]) else None),
                            'pred_queuefull': bool(pred[j]),
                            'sim_arr': round(float(sim_arr[j]), 3)})
                # 最近的 Ceres（回环）事件
                if ceres:
                    cs = [e['sim'] for e in ceres if e['sim'] is not None]
                    if cs:
                        k = int(np.argmin([abs(c - s) for c in cs]))
                        row['nearest_ceres_dt_s'] = round(cs[k] - s, 3)
            drop_rows.append(row)

    # ---- 单帧处理时延：/pose（addScan 成功才发）到达 − 同戳 /scan 到达
    #      这是"executor 被占住多久"的**直接**量：回环 + Ceres 都算在里面。
    pose_arr = np.asarray(npz['pose'][:, 1], dtype=float) if npz.get('pose') is not None \
        and np.asarray(npz['pose']).size else np.zeros(0)
    pose_st = np.asarray(npz['pose'][:, 0], dtype=float) if pose_arr.size else np.zeros(0)
    proc_lat = np.zeros(0)
    if pose_st.size and stamps.size:
        lat = []
        for ps, pa in zip(pose_st, pose_arr):
            j = int(np.argmin(np.abs(stamps - ps)))
            if abs(stamps[j] - ps) < 0.03 and np.isfinite(wall_arr[j]):
                lat.append(pa - wall_arr[j])
        proc_lat = np.asarray(lat, dtype=float)
    r['proc_latency_ms'] = stats(1000 * proc_lat)

    # ---- TF 到达滞后（LIO→slam_toolbox 那条边的"新鲜度"）：仿真域
    tf_lag = (tf_ob[:, 2] - tf_ob[:, 0]) if tf_ob.size else np.zeros(0)
    tf_stamp = tf_ob[:, 0] if tf_ob.size else np.zeros(0)

    # 有回环窗口 vs 无回环窗口的对比（±1.5 s 窗口）
    ceres_sim = np.array([e['sim'] for e in ceres if e['sim'] is not None], dtype=float)
    in_win = np.zeros(tf_stamp.size, dtype=bool)
    for c in ceres_sim:
        in_win |= np.abs(tf_stamp - c) <= 1.5
    scan_in_win = np.zeros(stamps.size, dtype=bool)
    scan_win_drop = np.zeros(stamps.size, dtype=bool)
    for c in ceres_sim:
        scan_in_win |= np.abs(stamps - c) <= 1.5
        scan_win_drop |= np.abs(stamps - c) <= 3.0

    r['counts'] = {
        'scan_frames': int(stamps.size),
        'drops': len(drops), 'drop_src': drop_src,
        'ceres_loop_closures': len(ceres),
        'pose_processed': (tj.get('pose_processed') or {}).get('n'),
        'pose_rate_per_sim_s': (tj.get('pose_processed') or {}).get('rate_per_sim_s'),
        'rec_pose_samples': len(poses),
        'graph_samples': len(graphs), 'map_samples': len(maps),
        'tf_odom_base': int(tf_ob.shape[0]),
    }
    span = float(stamps.max() - stamps.min()) if stamps.size else float('nan')
    r['sim_span_s'] = round(span, 2)
    r['drop_rate_per_sim_s'] = round(len(drops) / span, 4) if span and span > 0 else None
    r['drop_pct_of_scan'] = round(100.0 * len(drops) / max(stamps.size, 1), 3)
    r['loop_closure_rate_per_sim_s'] = (round(len(ceres_sim) / span, 4)
                                        if span and span > 0 else None)

    r['tf_wait_ms'] = {'all': stats(1000 * wait_eff),
                       'blocked_only': stats(1000 * wait_wall[np.isfinite(wait_wall)])}
    r['scan_blocked_at_arrival_pct'] = round(100.0 * float((~tf_ok).sum()) / max(stamps.size, 1), 2)
    r['predict_queuefull'] = {'n': int(pred.sum()),
                              'pct_of_scan': round(100.0 * float(pred.sum()) / max(stamps.size, 1), 3)}
    r['scan_gap_wall_ms'] = stats(1000 * gap_wall)
    r['tf_arrival_lag_sim_ms'] = stats(1000 * tf_lag)
    r['rtf'] = {'probe_p50': (tj.get('rtf') or {}).get('rtf_p50'),
                'probe_p10': (tj.get('rtf') or {}).get('rtf_p10'),
                'rec_p50': round(pct([s.get('rtf') for s in rec if s.get('rtf')], 50), 3),
                'rec_p10': round(pct([s.get('rtf') for s in rec if s.get('rtf')], 10), 3)}
    r['cpu_pct_p50'] = {k: round(pct([s['cpu'].get(k, 0.0) for s in rec if s.get('cpu')], 50), 1)
                        for k in ('gzserver', 'slam_toolbox', 'small_point_lio',
                                  'pointcloud_to_laserscan', 'ground_segmentation')}
    r['cpu_pct_max'] = {k: round(pct([s['cpu'].get(k, 0.0) for s in rec if s.get('cpu')], 100), 1)
                        for k in ('gzserver', 'slam_toolbox', 'small_point_lio')}
    r['cpu_total_pct'] = {'p50': round(pct([s.get('cpu_total_pct') for s in rec
                                            if s.get('cpu_total_pct')], 50), 1),
                          'p90': round(pct([s.get('cpu_total_pct') for s in rec
                                            if s.get('cpu_total_pct')], 90), 1),
                          'max': round(pct([s.get('cpu_total_pct') for s in rec
                                            if s.get('cpu_total_pct')], 100), 1)}

    # ---- 位姿图 / 环秩
    if graphs:
        g0, g1 = graphs[0], graphs[-1]
        r['pose_graph'] = {'V_first': g0.get('V'), 'E_first': g0.get('E'),
                           'cyc_first': g0.get('cyc'),
                           'V_last': g1.get('V'), 'E_last': g1.get('E'),
                           'cyc_last': g1.get('cyc'),
                           'cyc_max': max(g.get('cyc', 0) for g in graphs),
                           'sp_mean_last': g1.get('sp_mean')}
    # ---- /map 增长
    if maps:
        r['map'] = {'n_pub': len(maps), 'occ_first': maps[0].get('occ'),
                    'occ_last': maps[-1].get('occ'), 'known_last': maps[-1].get('known'),
                    'w': maps[-1].get('w'), 'h': maps[-1].get('h')}
    # ---- 精度（融合 vs 真值）
    rows = [p for p in poses if 'gt' in p and 'map' in p]
    if rows:
        # ⚠️ 口径：/odom_ground_truth 是**世界系**，map→base_link 是**出生点相对系**
        #    （slam_toolbox 的 map 原点 = 起步位姿）⇒ 必须各自减去自己的第一帧，
        #    否则得到的是出生点在世界里的偏移（实测 11.2 m = |(10.925, 2.525)|）。
        m0, g0 = rows[0]['map'], rows[0]['gt']
        e = [math.dist((p['map'][0] - m0[0], p['map'][1] - m0[1]),
                       (p['gt'][0] - g0[0], p['gt'][1] - g0[1])) for p in rows]
        el = [math.dist((p['odom'][0], p['odom'][1]),
                        (p['gt'][0] - g0[0] + m0[0], p['gt'][1] - g0[1] + m0[1]))
              for p in rows if 'odom' in p]
        r['err_map_m'] = {'mean': round(st.mean(e), 4), 'max': round(max(e), 4),
                          'last': round(e[-1], 4), 'n': len(e)}
        if el:
            r['err_odom_m'] = {'mean': round(st.mean(el), 4), 'max': round(max(el), 4)}
    # ---- 驱动/路线
    if drive:
        r['drive'] = {k: drive.get(k) for k in
                      ('finished_all_waypoints', 'waypoints_reached', 'waypoints_total',
                       'sim_duration_s', 'wall_duration_s', 'stuck_events')
                      if k in drive}
    r['params'] = {ln.split(' = ')[0]: ln.split(' = ')[-1]
                   for ln in params.splitlines() if ' = ' in ln}
    r['cfg_diff_lines'] = [ln for ln in cfgdiff.splitlines() if ln.startswith(('+', '-'))
                           and not ln.startswith(('+++', '---'))]

    # ---- 相关系数（§2 的核心数字）
    drop_stamps = np.array(drops, dtype=float)
    corr = {}
    if drop_stamps.size and ceres_sim.size:
        # 每个丢帧到最近回环的 Δ（秒）
        d = [min(abs(c - s) for c in ceres_sim) for s in drop_stamps]
        corr['drop_to_nearest_ceres_s'] = stats(d)
        corr['drops_within_2s_of_loop_closure'] = int(sum(1 for x in d if x <= 2.0))
        corr['drops_within_5s_of_loop_closure'] = int(sum(1 for x in d if x <= 5.0))
        # 随机基线：把丢帧时刻在整段里均匀重采样 200 次，看"±2 s 内有回环"的比例期望
        lo, hi = float(stamps.min()), float(stamps.max())
        rng = np.random.default_rng(0)
        base = []
        for _ in range(200):
            rs = rng.uniform(lo, hi, size=drop_stamps.size)
            base.append(np.mean([min(abs(c - s) for c in ceres_sim) <= 2.0 for s in rs]))
        corr['random_baseline_within_2s'] = round(float(np.mean(base)), 3)
        corr['observed_frac_within_2s'] = round(
            corr['drops_within_2s_of_loop_closure'] / max(drop_stamps.size, 1), 3)
    if drop_stamps.size:
        corr['drops_predicted_by_queuefull_rule'] = int(
            sum(1 for row in drop_rows if row.get('pred_queuefull')))
        corr['drops_with_tf_ok_at_arrival'] = int(
            sum(1 for row in drop_rows if row.get('tf_ok_at_arrival')))
        corr['drops_matched_to_probe_scan'] = int(
            sum(1 for row in drop_rows if row.get('matched_scan')))
        ww = [row['wait_wall_ms'] for row in drop_rows if row.get('wait_wall_ms') is not None]
        corr['dropped_scan_wait_wall_ms'] = stats(ww)
        gw = [row['gap_wall_ms'] for row in drop_rows if row.get('gap_wall_ms') is not None]
        corr['dropped_scan_next_gap_wall_ms'] = stats(gw)
    # 回环窗口内的 CPU / TF 滞后 vs 窗口外
    if ceres_sim.size and rec:
        cpu_ts = np.array([s.get('ts') if s.get('ts') is not None else np.nan for s in rec],
                          dtype=float)
        cpu_ssl = np.array([s['cpu'].get('slam_toolbox', 0.0) for s in rec], dtype=float)
        cpu_tot = np.array([s.get('cpu_total_pct') or 0.0 for s in rec], dtype=float)
        m = np.zeros(cpu_ts.size, dtype=bool)
        for c in ceres_sim:
            m |= np.abs(cpu_ts - c) <= 1.5
        corr['cpu_slam_toolbox_p50_in_loop_win'] = round(pct(cpu_ssl[m], 50), 1) if m.any() else None
        corr['cpu_slam_toolbox_p50_outside'] = round(pct(cpu_ssl[~m], 50), 1) if (~m).any() else None
        corr['cpu_total_p50_in_loop_win'] = round(pct(cpu_tot[m], 50), 1) if m.any() else None
        corr['cpu_total_p50_outside'] = round(pct(cpu_tot[~m], 50), 1) if (~m).any() else None
        corr['tf_lag_p50_in_loop_win_ms'] = (round(1000 * pct(tf_lag[in_win], 50), 2)
                                             if in_win.any() else None)
        corr['tf_lag_p50_outside_ms'] = (round(1000 * pct(tf_lag[~in_win], 50), 2)
                                         if (~in_win).any() else None)
        corr['scan_blocked_pct_in_loop_win'] = (round(100.0 * float((~tf_ok[scan_in_win]).mean()), 2)
                                               if scan_in_win.any() else None)
        corr['scan_blocked_pct_outside'] = (round(100.0 * float((~tf_ok[~scan_in_win]).mean()), 2)
                                           if (~scan_in_win).any() else None)
    if ceres_sim.size and proc_lat.size:
        pl_ts = []
        for ps in pose_st:
            j = int(np.argmin(np.abs(stamps - ps)))
            pl_ts.append(stamps[j] if abs(stamps[j] - ps) < 0.03 else ps)
        pl_ts = np.asarray(pl_ts, dtype=float)
        mm = np.zeros(pl_ts.size, dtype=bool)
        for c in ceres_sim:
            mm |= np.abs(pl_ts - c) <= 1.5
        corr['proc_latency_p50_in_loop_win_ms'] = (round(1000 * pct(proc_lat[mm], 50), 2)
                                                   if mm.any() else None)
        corr['proc_latency_p50_outside_ms'] = (round(1000 * pct(proc_lat[~mm], 50), 2)
                                               if (~mm).any() else None)
        corr['proc_latency_max_ms'] = round(1000 * float(proc_lat.max()), 2)
    r['correlation'] = corr
    r['drops_detail'] = drop_rows
    r['ceres_events'] = ceres
    r['series'] = {'stamps': stamps, 'sim_arr': sim_arr, 'wall_arr': wall_arr,
                   'tf_ok': tf_ok, 'wait_wall': wait_wall, 'wait_sim': wait_sim,
                   'pred': pred, 'tf_stamp': tf_stamp, 'tf_lag': tf_lag,
                   'tf_mo': tf_mo, 'clocks': clocks,
                   'rec_ts': np.array([s.get('ts') if s.get('ts') is not None else np.nan
                                       for s in rec], dtype=float),
                   'rec_cpu_ssl': np.array([s['cpu'].get('slam_toolbox', 0.0) for s in rec]),
                   'rec_cpu_total': np.array([s.get('cpu_total_pct') or np.nan for s in rec]),
                   'rec_rtf': np.array([s.get('rtf') if s.get('rtf') is not None else np.nan
                                        for s in rec], dtype=float),
                   'ceres_sim': ceres_sim, 'drops': drop_stamps}
    return r


# --------------------------------------------------------------------------- 作图
def plot(arms, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print('[warn] 画图跳过：%r' % e, file=sys.stderr)
        return
    n = len(arms)
    fig, axes = plt.subplots(n, 1, figsize=(13, 2.9 * n), sharex=True, squeeze=False)
    for k, r in enumerate(arms):
        ax = axes[k][0]
        s = r['series']
        st_, w = s['stamps'], 1000 * np.where(np.isfinite(s['wait_wall']), s['wait_wall'], 0.0)
        ax.plot(st_, w, lw=0.7, color='tab:blue',
                label='per-scan TF wait (ms, wall clock; 0 = not blocked)')
        ax.axhline(130, color='tab:gray', ls='--', lw=0.8,
                   label='next /scan arrival interval ~130 ms (drop threshold)')
        for c in s['ceres_sim']:
            ax.axvline(c, color='tab:red', lw=1.0, alpha=0.55)
        ax.scatter(s['drops'], np.full(s['drops'].size, -14), marker='v', s=26,
                   color='black', zorder=5, label='logged drop (log "at time X")')
        d = s['stamps'][s['pred']]
        ax.scatter(d, np.full(d.size, -26), marker='x', s=18, color='tab:orange',
                   label='predicted drop (tf2 rule: wait > next gap)')
        ax2 = ax.twinx()
        if s['rec_ts'].size:
            ax2.plot(s['rec_ts'], s['rec_cpu_ssl'] / 100.0, lw=0.8, color='tab:green',
                     alpha=0.8, label='slam_toolbox CPU (cores)')
            ax2.plot(s['rec_ts'], s['rec_cpu_total'] / 100.0, lw=0.8, color='tab:purple',
                     alpha=0.5, label='total machine CPU (cores)')
        ax2.set_ylabel('CPU (cores)', fontsize=8)
        ax2.set_ylim(0, max(4.0, np.nanmax(s['rec_cpu_total']) / 100.0 * 1.1
                            if s['rec_cpu_total'].size and np.isfinite(s['rec_cpu_total']).any()
                            else 4.0))
        ax.set_title('%s: drops=%d  loop-closures(Ceres)=%d  RTF p50=%s' %
                     (r['arm'], r['counts']['drops'], r['counts']['ceres_loop_closures'],
                      r['rtf'].get('rec_p50')), fontsize=9)
        ax.set_ylabel('TF wait (ms)')
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=6.5, loc='upper left', ncol=2)
    axes[-1][0].set_xlabel('sim time (s)')
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    print('[png] %s' % path)


def main():
    ap = argparse.ArgumentParser(description='丢帧 ↔ 回环(Ceres) ↔ TF 等待 ↔ CPU 的联合归因')
    ap.add_argument('--arm', action='append', required=True,
                    help='name=dir（可多次）；dir 里应含 run_dropab.sh 的产物')
    ap.add_argument('--json', help='汇总 JSON 落盘')
    ap.add_argument('--md', help='可读报告落盘')
    ap.add_argument('--png', help='时间轴图落盘（docs/img/…）')
    args = ap.parse_args()
    rt_minus_mono = time.time() - time.monotonic()   # 墙钟(epoch) ↔ 单调钟 的常量偏移
    arms = []
    for spec in args.arm:
        if '=' not in spec:
            sys.exit('--arm 要写成 name=dir')
        name, d = spec.split('=', 1)
        r = arm_report(name, d, rt_minus_mono)
        arms.append(r)
    if args.png:
        plot(arms, args.png)

    # ---- 汇总表（A/B 那一张）
    keys = ['drops', 'drop_rate_per_sim_s', 'drop_pct_of_scan', 'ceres_loop_closures',
            'loop_closure_rate_per_sim_s', 'sim_span_s', 'scan_frames', 'pose_processed']
    hdr = ['arm'] + keys + ['cyc_last', 'rtf_p50', 'rtf_p10', 'ssl_cpu_p50', 'ssl_cpu_max',
                            'lio_cpu_p50', 'cpu_total_p50', 'wait_p90_ms', 'blocked_pct']
    lines = ['| ' + ' | '.join(hdr) + ' |', '|' + '---|' * len(hdr)]
    for r in arms:
        row = [r['arm']] + [str(r['counts'].get(k, '')) if k in r['counts']
                            else str(r.get(k, '')) for k in keys]
        row += [str((r.get('pose_graph') or {}).get('cyc_last', '')),
                str(r['rtf'].get('rec_p50', '')), str(r['rtf'].get('rec_p10', '')),
                str(r['cpu_pct_p50'].get('slam_toolbox', '')),
                str(r['cpu_pct_max'].get('slam_toolbox', '')),
                str(r['cpu_pct_p50'].get('small_point_lio', '')),
                str(r['cpu_total_pct'].get('p50', '')),
                str((r['tf_wait_ms']['blocked_only'] or {}).get('p90', '')),
                str(r['scan_blocked_at_arrival_pct'])]
        lines.append('| ' + ' | '.join(row) + ' |')
    table = '\n'.join(lines)
    print(table)

    out = {'generated': _dt.datetime.now().isoformat(timespec='seconds'),
           'arms': [{k: v for k, v in r.items() if k != 'series'} for r in arms],
           'table_md': table}
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('[json] %s' % args.json)
    if args.md:
        with open(args.md, 'w') as f:
            f.write(table + '\n')
        print('[md] %s' % args.md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
