#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`scan_tf_timing_probe.py` 的离线复算器：**逐纳秒**比较 LIO 的 TF 戳与 `/scan` 的戳。

⚠️ 定位（请勿误用）：一次性/离线分析工具，不是运行时监控、不被任何 launch 启动。
   输入是探针落盘的 `<out>.npz`（`scan_tf_timing_probe.py --out <out>` 的产物）。

为什么需要它（本工具存在的唯一理由）：
  `/scan` 的戳来自仿真插件（`rclcpp::Time` 整数纳秒，精确），而 odom→base_link 的 TF 戳来自
  LIO 内部的 **float64 秒 → (sec, nanosec)** 转换。若那个转换是**截断**的，浮点误差会让
  约 40% 的帧算成 X.99999999x ⇒ nanosec 少 1 ⇒ TF 戳比同一帧 `/scan` 戳**低 1 纳秒**。
  tf2 的 MessageFilter 判据是"缓存里必须有 stamp >= 请求戳的样本"，低 1 ns 的同帧样本不合格
  ⇒ 等下一帧 TF（~0.1 s）⇒ `scan_queue_size=1` 时该帧被 `QueueFull` 顶掉。
  ⇒ 这个 1 ns 的差必须能被**直接量出来**，而不是从丢帧日志里反推。

它做什么：
  1. 从 npz 里取 `/scan` 的戳序列与 odom→base_link 的 TF 戳序列；
  2. 用**整数纳秒**重建（`round(sec*1e9)`，double 在 ~1e3 s 处的分辨率 ≈0.1 ns ⇒ 无歧义）；
  3. 对每条 `/scan` 找时间上最近的 TF 样本（同帧；窗口默认 ±5 ms），算 Δ = TF戳 − scan戳；
  4. 输出 Δ 的分布：`Δ=0`（同纳秒）、`Δ=−1 ns`（截断 bug 的指纹）、`Δ<0`、`Δ≥0` 的比例，
     以及 |Δ|≤10 ns 的完整直方图。

判据（怎么读结果）：
  · `Δ=−1 ns` 占 ~40%、`Δ=0` 占 ~60%  ⇒ **截断 bug 存在**（已实测：small_point_lio 修复前）；
  · `Δ=−1 ns` = 0、`Δ=0` ~95%+      ⇒ 该 LIO 的 TF 戳与插件戳同纳秒（修复后 / 本来就是四舍五入）；
  · `Δ<0` 且量级是 ms 级            ⇒ 不是 1 ns 截断，而是"LIO 用帧首/帧尾/IMU 时刻盖戳"的真实偏移，
                                    需要看 `--hist-ms` 的大尺度直方图与代码，不能当 1 ns 处理。

用法：
  python3 tools/scripts/diag/stamp_delta_ns_report.py \
      --npz .tmp_cache/slamq/q05/q05.timing --json /tmp/q05.delta.json
  # 一次算多个（对照用）：
  python3 tools/scripts/diag/stamp_delta_ns_report.py --npz a.timing b.timing
"""
import argparse
import json
import sys

import numpy as np


def ns_of(x):
    """float64 秒 → 整数纳秒（round；1e3 s 处 double 分辨率 ~0.1 ns，足够）。"""
    return np.rint(np.asarray(x, dtype=float) * 1e9).astype(np.int64)


def load(npz_path):
    d = np.load(npz_path)
    if 'scan_ns' in d.files and 'tf_ob_ns' in d.files and d['tf_ob_ns'].size:
        scan = d['scan_ns'].astype(np.int64)
        tf = d['tf_ob_ns'].astype(np.int64)
        src = 'int-fields'
    else:
        scan = ns_of(d['stamps'])
        tf = ns_of(d['tf_ob'][:, 0]) if d['tf_ob'].size else np.zeros(0, np.int64)
        src = 'float-derived'
    return scan, tf, src


def analyze(npz_path, win_ns, hist_ms):
    scan, tf, src = load(npz_path)
    out = {'npz': npz_path, 'ns_source': src,
           'n_scan': int(scan.size), 'n_tf': int(tf.size)}
    if scan.size == 0 or tf.size == 0:
        out['error'] = 'empty scan or tf sequence'
        return out
    tf_sorted = np.sort(tf)
    # 每条 scan 找最近的 TF 样本（在窗口内）
    idx = np.searchsorted(tf_sorted, scan)
    cand = []
    for off in (-1, 0):
        j = np.clip(idx + off, 0, tf_sorted.size - 1)
        cand.append(tf_sorted[j] - scan)
    cand = np.vstack(cand)
    delta = np.where(np.abs(cand[0]) <= np.abs(cand[1]), cand[0], cand[1])
    near = np.abs(delta) <= win_ns
    d = delta[near]
    n = int(d.size)
    out['match_window_ms'] = win_ns / 1e6
    out['n_matched'] = n
    out['n_unmatched'] = int(scan.size - n)
    if n == 0:
        out['error'] = 'no TF sample within match window'
        return out

    def pct(k):
        return round(100.0 * float(k) / n, 3)

    out['delta_ns'] = {
        'exact_0ns_pct': pct((d == 0).sum()),
        'minus_1ns_pct': pct((d == -1).sum()),
        'plus_1ns_pct': pct((d == 1).sum()),
        'lt_0_pct': pct((d < 0).sum()),
        'ge_0_pct': pct((d >= 0).sum()),
        'p50': int(np.percentile(d, 50)), 'min': int(d.min()), 'max': int(d.max()),
    }
    # |Δ| ≤ 10 ns 的完整直方图（截断 bug 的指纹区）
    vals, cnts = np.unique(d[np.abs(d) <= 10], return_counts=True)
    out['hist_abs_le_10ns'] = {int(v): int(c) for v, c in zip(vals, cnts)}
    # 大尺度直方图（看是不是"帧首 vs 帧尾"的真实偏移，而不是 1 ns）
    edges = np.arange(-hist_ms, hist_ms + 1e-9, max(hist_ms / 20.0, 0.1)) * 1e6
    h, _ = np.histogram(d, bins=edges)
    out['hist_ms_bins'] = [[round(float(edges[i]) / 1e6, 3),
                            round(float(edges[i + 1]) / 1e6, 3), int(h[i])]
                           for i in range(len(h)) if h[i] > 0]
    # TF 戳序列自身的帧周期（判断"等下一帧"要等多久）
    if tf_sorted.size > 2:
        dt = np.diff(tf_sorted)
        dt = dt[dt > 0]
        if dt.size:
            out['tf_period_ms'] = {
                'p50': round(float(np.percentile(dt, 50)) / 1e6, 3),
                'p90': round(float(np.percentile(dt, 90)) / 1e6, 3)}
    # 结论行（只做最保守的判定，避免把 ms 级真实偏移也叫"截断 bug"）
    if out['delta_ns']['minus_1ns_pct'] > 10.0 and out['delta_ns']['exact_0ns_pct'] > 10.0:
        out['verdict'] = 'TRUNCATION-FINGERPRINT（Δ=−1 ns 占比 %.1f%% / Δ=0 占比 %.1f%%）' % (
            out['delta_ns']['minus_1ns_pct'], out['delta_ns']['exact_0ns_pct'])
    elif out['delta_ns']['exact_0ns_pct'] >= 90.0:
        out['verdict'] = 'OK（同纳秒 %.1f%%，无 −1 ns 指纹）' % out['delta_ns']['exact_0ns_pct']
    else:
        out['verdict'] = 'OTHER（Δ=0 %.1f%%，需看 hist_ms_bins）' % out['delta_ns']['exact_0ns_pct']
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--npz', nargs='+', required=True,
                    help='探针输出前缀（不带 .npz）或 .npz 路径，可给多个')
    ap.add_argument('--match-window-ms', type=float, default=5.0,
                    help='同帧匹配窗口（毫秒，默认 5）')
    ap.add_argument('--hist-ms', type=float, default=200.0,
                    help='大尺度直方图范围（毫秒，默认 ±200）')
    ap.add_argument('--json', default='', help='把结果写成 JSON（可选）')
    args = ap.parse_args()

    results = []
    for p in args.npz:
        path = p if p.endswith('.npz') else p + '.npz'
        try:
            r = analyze(path, args.match_window_ms * 1e6, args.hist_ms)
        except Exception as e:  # 单份失败不影响其它
            r = {'npz': path, 'error': repr(e)}
        results.append(r)
        if 'error' in r:
            print('[delta-ns] %s ❌ %s' % (path, r['error']))
            continue
        d = r['delta_ns']
        print('[delta-ns] %-46s scan=%d tf=%d matched=%d | Δ=0 %.1f%% | Δ=−1ns %.1f%% | '
              'Δ<0 %.1f%% | tf周期p50=%s ms | %s' % (
                  path, r['n_scan'], r['n_tf'], r['n_matched'], d['exact_0ns_pct'],
                  d['minus_1ns_pct'], d['lt_0_pct'],
                  r.get('tf_period_ms', {}).get('p50', '?'), r['verdict']))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results if len(results) > 1 else results[0], f,
                      ensure_ascii=False, indent=1)
        print('[delta-ns] JSON → %s' % args.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
