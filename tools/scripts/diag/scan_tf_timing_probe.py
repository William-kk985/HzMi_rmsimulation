#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`/scan` 时间戳链 + tf2 MessageFilter 等待时间 —— **一次性测量仪器**。

⚠️ 定位（请勿误用）：这是**离线/一次性测量工具**，不是运行时监控、不是哨兵、不被任何
   launch 启动、不进默认链路。它存在的唯一目的：把「slam_toolbox 的 tf2 MessageFilter
   为什么必须等 TF、等多久、等多久才会被队列深度 1 顶掉」变成可复现的数字，
   用于 `docs/slam_toolbox_scan_drops.md` 的根因判定。

测什么（仿真钟 = `/clock`，墙钟 = `time.monotonic()`，两把尺子都记）：

  1. **`/scan` 到达延迟**：到达时刻 − `header.stamp`
     = 插件 → linefit → pointcloud_to_laserscan → DDS 的端到端延迟（"扫描发得晚不晚"）
  2. **到达那一刻能不能查到 `odom→<scan frame>`**：复刻 `tf2_ros::MessageFilter` 的判据
     （target = odom_frame，source = scan 的 `frame_id`，时间 = `header.stamp`）
  3. **查不到时等了多久**（250 Hz 墙钟轮询 tf2 Buffer，直到可查）
     ⇒ 与"下一条 `/scan` 的到达间隔"比较：等待 > 间隔 的那些帧，
        就是 `scan_queue_size=1` 下会被 `QueueFull` 顶掉的那些（tf2 MessageFilter 语义）
  4. **`/tf` 到达延迟 + `odom→base_link` 的 stamp 粒度/相位**
     （LIO 的 TF stamp 比 `/scan` 的 stamp 是超前还是滞后、超前/滞后多少）
  5. **slam_toolbox 自己发的 `/pose`**：源码 `slam_toolbox_common.cpp:655`
     `publishPose()` 只在 `addScan` 成功（`processed==true`）时调用一次
     ⇒ 不需要改 slam_toolbox 源码，就能拿到"**哪些 `/scan` 真的被拿去匹配了**"（stamp 与该帧 `header.stamp` 相同）
  6. **`/map` 每次发布的格子数**（建图增长曲线：occupied / free / unknown）
  7. ★ 2026-10-06 新增：**逐纳秒 Δ = TF戳 − /scan戳**（整数域，不用 double）。
     `Δ=−1 ns` 大量出现就是"float64 秒 → 纳秒用截断"的指纹（见
     docs/timestamp_construction_audit.md）；同口径的离线复算器（读本探针的 npz）：
     `tools/scripts/diag/stamp_delta_ns_report.py`

输出：`--out <prefix>.json`（汇总数字）+ `<prefix>.npz`（原始序列，供离线复算）。

用法（必须与跑图在同一次 bash 调用里，见 `tools/scripts/mapping/mapping_ab_run.sh` 头注）：
  python3 tools/scripts/diag/scan_tf_timing_probe.py --out /tmp/x/probe --duration 240
"""
import argparse
import json
import threading
import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def qos(depth, reliable=True, transient=False):
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
    )


class ScanTfTimingProbe(Node):
    def __init__(self, args):
        super().__init__('scan_tf_timing_probe')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.args = args
        self.lock = threading.Lock()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=args.tf_cache))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # ---- 原始序列（stamp 一律用消息 header.stamp；到达时刻两把尺子都记）----
        self.scan = []          # (stamp, wall_arr, sim_arr, tf_ok_at_arrival)
        self.pending = OrderedDict()   # stamp -> dict(t_wall, t_sim, idx)
        self.tf_ob = []         # odom->base_link: (stamp, wall_arr, sim_arr)
        self.tf_mo = []         # map->odom:      (stamp, wall_arr, sim_arr)
        self.pose = []          # slam_toolbox /pose (processed scans): (stamp, wall_arr, sim_arr)
        self.maps = []          # (sim, occ, free, unk, w, h, res)
        self.clocks = []        # (sim, wall) from /clock
        self.tf_frames = {}     # 「父→子」计数
        # ---- 整数纳秒序列（★ 2026-10-06 新增：量 1 ns 级戳差专用）----
        #   为什么另存一份整数：`stamp = sec + nanosec*1e-9` 这个 double 在 ~1e3 s 处
        #   分辨率只有 ~0.1 ns，虽然够用，但"截断 bug"的指纹恰好就是 1 ns；
        #   直接从消息的 sec/nanosec 整数域取，彻底不引入浮点。
        #   与 self.scan / self.tf_ob 一一对应（只 append，顺序一致）。
        #   离线复算器：tools/scripts/diag/stamp_delta_ns_report.py
        self.scan_ns = []       # /scan header.stamp 的整数纳秒
        self.tf_ob_ns = []      # odom->base_link 的整数纳秒

        cb = ReentrantCallbackGroup()
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos(500), callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.on_pose,
                                 qos(200), callback_group=cb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 qos(1, transient=True), callback_group=cb)
        # /clock 的发布端是 BEST_EFFORT（rclpy TimeSource 也用 BEST_EFFORT，见
        # rclpy/time_source.py:70）——用 RELIABLE 订阅会收到 QoS 不兼容警告且**一帧都收不到**。
        self.create_subscription(ClockMsg, '/clock', self.on_clock,
                                 qos(20, reliable=False), callback_group=cb)

        # 墙钟 250 Hz 轮询：pending 帧的 TF 什么时候才可查（不受 excutor 饥饿影响）
        self._stop = threading.Event()
        self._poller = threading.Thread(target=self._poll_loop, daemon=True)
        self._poller.start()

    # ------------------------------------------------------------------ 时间基
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 回调
    def on_clock(self, msg):
        s = msg.clock.sec + msg.clock.nanosec * 1e-9
        w = time.monotonic()
        with self.lock:
            self.clocks.append((s, w))

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamp_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
        w = time.monotonic()
        s = self.sim_now()
        frame = msg.header.frame_id.lstrip('/')
        ok = self._can(frame, stamp)
        with self.lock:
            idx = len(self.scan)
            self.scan.append((stamp, w, s, 1 if ok else 0))
            self.scan_ns.append(stamp_ns)
            if not ok:
                self.pending[stamp] = dict(t_wall=w, t_sim=s, idx=idx, frame=frame)
        if self.args.verbose:
            print(f'[probe] scan stamp={stamp:.3f} sim_arr={s:.3f} tf_ok={ok} '
                  f'lat={s - stamp:+.3f}s', flush=True)

    def _can(self, frame, stamp):
        try:
            return bool(self.tf_buffer.can_transform(
                self.args.odom_frame, frame, Time(seconds=stamp),
                timeout=Duration(seconds=0.0)))
        except Exception:
            return False

    def on_tf(self, msg):
        w = time.monotonic()
        s = self.sim_now()
        with self.lock:
            for tr in msg.transforms:
                key = f'{tr.header.frame_id.lstrip("/")}->{tr.child_frame_id.lstrip("/")}'
                self.tf_frames[key] = self.tf_frames.get(key, 0) + 1
                st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                st_ns = int(tr.header.stamp.sec) * 10**9 + int(tr.header.stamp.nanosec)
                if tr.child_frame_id.lstrip('/') == self.args.base_frame and \
                        tr.header.frame_id.lstrip('/') == self.args.odom_frame:
                    self.tf_ob.append((st, w, s))
                    self.tf_ob_ns.append(st_ns)
                if tr.child_frame_id.lstrip('/') == self.args.odom_frame and \
                        tr.header.frame_id.lstrip('/') == self.args.map_frame:
                    self.tf_mo.append((st, w, s))

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.pose.append((stamp, time.monotonic(), self.sim_now()))

    def on_map(self, msg):
        n = len(msg.data)
        occ = free = unk = 0
        if n:
            a = np.frombuffer(bytes(msg.data), dtype=np.int8)
            occ = int((a > 50).sum())
            free = int(((a >= 0) & (a <= 50)).sum())
            unk = int((a < 0).sum())
        with self.lock:
            self.maps.append((self.sim_now(), occ, free, unk,
                              msg.info.width, msg.info.height, msg.info.resolution))

    # ------------------------------------------------------------------ 轮询线程
    def _poll_loop(self):
        while not self._stop.is_set():
            if self.pending:
                with self.lock:
                    items = list(self.pending.items())
                for stamp, d in items:
                    if self._can(d['frame'], stamp):
                        with self.lock:
                            cur = self.pending.pop(stamp, None)
                        if cur is not None:
                            w = time.monotonic()
                            s = self.sim_now()
                            # ⚠️ 第 4 位必须保持 0（= 到达时查不到 TF）。写 1 会把
                            #    "到达时就能查" 的统计弄反（q01b 就踩过这个坑）。
                            self.scan[cur['idx']] = (stamp, cur['t_wall'], cur['t_sim'],
                                                     0, w, s)
            time.sleep(1.0 / 250.0)

    def stop(self):
        self._stop.set()
        self._poller.join(timeout=2.0)

    # ------------------------------------------------------------------ 汇总
    def summary(self):
        with self.lock:
            scan = list(self.scan)
            pend = dict(self.pending)
            tf_ob = list(self.tf_ob)
            tf_mo = list(self.tf_mo)
            pose = list(self.pose)
            maps = list(self.maps)
            clocks = list(self.clocks)
            frames = dict(self.tf_frames)

        out = {'counts': {'scan': len(scan), 'tf_odom_base': len(tf_ob),
                          'tf_map_odom': len(tf_mo), 'pose_processed': len(pose),
                          'map_pub': len(maps), 'clock': len(clocks),
                          'pending_left': len(pend)},
               'tf_frames': frames}
        if not scan:
            return out

        stamps = np.array([r[0] for r in scan])
        wall_arr = np.array([r[1] for r in scan])
        sim_arr = np.array([r[2] for r in scan])
        tf_ok = np.array([r[3] for r in scan], dtype=bool)
        wait_wall = np.full(len(scan), np.nan)
        wait_sim = np.full(len(scan), np.nan)
        avail_wall = np.full(len(scan), np.nan)
        for i, r in enumerate(scan):
            if len(r) >= 6:
                avail_wall[i] = r[4]
                wait_wall[i] = r[4] - r[1]
                wait_sim[i] = r[5] - r[2]

        # ---- RTF / 时钟对齐：sim <- wall 插值 ----
        cl = np.array(clocks) if len(clocks) else np.zeros((0, 2))
        out['rtf'] = self._rtf_stats(cl)
        sim_per_wall = out['rtf'].get('sim_per_wall_median', 1.0)

        gap_sim = np.diff(sim_arr, prepend=sim_arr[0])
        gap_sim[0] = np.median(gap_sim[1:]) if len(gap_sim) > 1 else 0.1

        # ---- (1) /scan 到达延迟 ----
        lat_sim = sim_arr - stamps
        out['scan_arrival_latency_sim'] = self._stats(lat_sim)
        out['scan_arrival_latency_wall'] = self._stats(lat_sim / max(sim_per_wall, 1e-9))
        out['scan_period_sim'] = self._stats(gap_sim)

        # ---- (2)(3) MessageFilter 判据 ----
        n_block = int((~tf_ok).sum())
        out['tf_available_at_arrival'] = {
            'n_ok': int(tf_ok.sum()), 'n_blocked': n_block,
            'frac_blocked_pct': round(100.0 * n_block / len(scan), 3)}
        if n_block:
            ws = wait_sim[~tf_ok]
            ws = ws[np.isfinite(ws)]
            out['wait_sim_when_blocked'] = self._stats(ws)
            ww = wait_wall[~tf_ok]
            ww = ww[np.isfinite(ww)]
            out['wait_wall_when_blocked'] = self._stats(ww)
            # QueueFull 预测：等待 > 下一条扫描的到达间隔 ⇒ 被顶掉（队列深度 1 的语义）
            pred = np.zeros(len(scan), dtype=bool)
            for i in range(len(scan) - 1):
                if not tf_ok[i] and np.isfinite(wait_sim[i]):
                    pred[i] = wait_sim[i] > gap_sim[i + 1]
            out['predict_queuefull'] = {
                'n_pred': int(pred.sum()),
                'frac_of_scan_pct': round(100.0 * pred.sum() / len(scan), 3),
                'pred_stamps_head': [round(float(s), 3) for s in stamps[pred][:10]]}
        else:
            out['predict_queuefull'] = {'n_pred': 0, 'frac_of_scan_pct': 0.0,
                                        'pred_stamps_head': []}

        # ---- (4) LIO TF 粒度/相位 ----
        if len(tf_ob):
            tob = np.array(tf_ob)
            tst = tob[:, 0]
            arr = tob[:, 1]
            sarr = tob[:, 2]
            out['tf_odom_base'] = {
                'n': int(len(tst)),
                'stamp_period_sim': self._stats(np.diff(tst)),
                'arrival_lag_sim': self._stats(sarr - tst),
                'stamp_min': round(float(tst.min()), 3),
                'stamp_max': round(float(tst.max()), 3),
            }
            # 每条 /scan 到达时，odom→base_link 的"最新 stamp 余量"（>0 = TF 领先扫描戳）
            head = []
            for i in range(len(scan)):
                j = np.searchsorted(arr, wall_arr[i], side='right') - 1
                if j >= 0:
                    head.append(tst[j] - stamps[i])
            head = np.array(head)
            out['tf_headroom_at_scan_arrival'] = self._stats(head)
            out['tf_headroom_frac_negative_pct'] = round(
                100.0 * float((head < 0).sum()) / max(len(head), 1), 3)
        if len(tf_mo):
            tmo = np.array(tf_mo)
            out['tf_map_odom'] = {
                'n': int(len(tmo)),
                'stamp_period_sim': self._stats(np.diff(tmo[:, 0])),
                'arrival_lag_sim': self._stats(tmo[:, 2] - tmo[:, 0])}

        # ---- (4b) ★ 2026-10-06 新增：逐纳秒 Δ = TF戳 − /scan戳（截断 bug 的指纹）----
        #   整数域比较（sec/nanosec 直接拼），不用 double。口径与离线复算器
        #   tools/scripts/diag/stamp_delta_ns_report.py 完全一致（同窗口、同"最近样本"规则）。
        #   读法：Δ=−1 ns 大量出现 = float64→ns 用**截断**的指纹；Δ=0 高 = 同纳秒有 TF 样本。
        with self.lock:
            s_ns = np.asarray(self.scan_ns, dtype=np.int64)
            t_ns = np.asarray(self.tf_ob_ns, dtype=np.int64)
        if s_ns.size and t_ns.size:
            t_sorted = np.sort(t_ns)
            j = np.clip(np.searchsorted(t_sorted, s_ns), 0, t_sorted.size - 1)
            cand = np.vstack([t_sorted[j] - s_ns,
                              t_sorted[np.clip(j - 1, 0, t_sorted.size - 1)] - s_ns])
            delta = np.where(np.abs(cand[0]) <= np.abs(cand[1]), cand[0], cand[1])
            near = np.abs(delta) <= int(5e6)      # 同帧窗口 ±5 ms
            d = delta[near]
            if d.size:
                vals, cnts = np.unique(d[np.abs(d) <= 10], return_counts=True)
                out['stamp_delta_ns_tf_minus_scan'] = {
                    'n_matched': int(d.size),
                    'exact_0ns_pct': round(100.0 * float((d == 0).sum()) / d.size, 3),
                    'minus_1ns_pct': round(100.0 * float((d == -1).sum()) / d.size, 3),
                    'lt_0_pct': round(100.0 * float((d < 0).sum()) / d.size, 3),
                    'hist_abs_le_10ns': {str(int(v)): int(c) for v, c in zip(vals, cnts)}}

        # ---- (5) 被真正处理的扫描 ----
        if len(pose):
            p = np.array(pose)
            pst = np.sort(p[:, 0])
            out['pose_processed'] = {
                'n': int(len(pst)),
                'rate_per_sim_s': round(len(pst) / max(stamps[-1] - stamps[0], 1e-9), 3),
                'inter_interval_sim': self._stats(np.diff(pst)),
                'stamps': [round(float(x), 3) for x in pst],
            }
            # 丢帧 vs "本来会被处理的那一帧"：最近的处理帧距离
            out['pose_gap_stats_sim'] = self._stats(np.diff(pst))
            # 哪些 /scan 从未到过回调（= 被过滤器顶掉的），与 pose 的时间关系
            out['pose_stamps_head'] = [round(float(x), 3) for x in pst[:10]]
        # ---- (6) /map 增长 ----
        if maps:
            m = np.array([(a, b, c, d) for a, b, c, d, *_ in maps], dtype=float)
            out['map_growth'] = {
                'n_pub': len(maps),
                'sim_first': round(float(maps[0][0]), 2),
                'sim_last': round(float(maps[-1][0]), 2),
                'occ_first': int(m[0, 0]), 'occ_last': int(m[-1, 0]),
                'free_first': int(m[0, 1]), 'free_last': int(m[-1, 1]),
                'unk_first': int(m[0, 2]), 'unk_last': int(m[-1, 2]),
                'occ_max': int(m[:, 0].max()), 'free_max': int(m[:, 1].max()),
                'cells_known_last': int(m[-1, 0] + m[-1, 1]),
                'size': [int(maps[-1][4]), int(maps[-1][5])],
                'res': float(maps[-1][6]),
                'series': [[round(float(a), 2), int(b), int(c), int(d)]
                           for a, b, c, d, *_ in maps],
            }
        out['_raw'] = dict(stamps=stamps, wall_arr=wall_arr, sim_arr=sim_arr,
                           tf_ok=tf_ok, wait_sim=wait_sim, wait_wall=wait_wall,
                           tf_ob=(np.array(tf_ob) if len(tf_ob) else np.zeros((0, 3))),
                           pose=(np.array(pose) if len(pose) else np.zeros((0, 3))),
                           clocks=cl, maps=np.array(maps) if maps else np.zeros((0, 7)),
                           # ★ 整数纳秒（1 ns 级戳差专用；离线复算器优先用这两条）
                           scan_ns=(np.asarray(self.scan_ns, dtype=np.int64)
                                    if self.scan_ns else np.zeros(0, np.int64)),
                           tf_ob_ns=(np.asarray(self.tf_ob_ns, dtype=np.int64)
                                     if self.tf_ob_ns else np.zeros(0, np.int64)))
        return out

    @staticmethod
    def _stats(x):
        x = np.asarray(x, dtype=float)
        x = x[np.isfinite(x)]
        if x.size == 0:
            return None
        return {'n': int(x.size), 'min': round(float(x.min()), 4),
                'p50': round(float(np.percentile(x, 50)), 4),
                'p90': round(float(np.percentile(x, 90)), 4),
                'p99': round(float(np.percentile(x, 99)), 4),
                'max': round(float(x.max()), 4),
                'mean': round(float(x.mean()), 4)}

    @staticmethod
    def _rtf_stats(clocks):
        if len(clocks) < 3:
            return {'n': int(len(clocks))}
        sim = clocks[:, 0]
        wall = clocks[:, 1]
        ds = np.diff(sim)
        dw = np.diff(wall)
        ok = (dw > 1e-6) & (ds >= 0)
        rtf = ds[ok] / dw[ok]
        rtf = rtf[np.isfinite(rtf)]
        if rtf.size == 0:
            return {'n': 0}
        return {'n': int(rtf.size),
                'rtf_p50': round(float(np.percentile(rtf, 50)), 4),
                'rtf_p10': round(float(np.percentile(rtf, 10)), 4),
                'rtf_p90': round(float(np.percentile(rtf, 90)), 4),
                'sim_per_wall_median': float(np.percentile(rtf, 50))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='输出前缀（写 <out>.json / <out>.npz）')
    ap.add_argument('--duration', type=float, default=240.0, help='采集时长（仿真秒）')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--scan-frame', default='livox_frame')
    ap.add_argument('--tf-cache', type=float, default=30.0)
    ap.add_argument('--verbose', action='store_true')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ScanTfTimingProbe(args)
    # 多线程 executor：/tf 很密，单线程会被 /scan 的处理饿到
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - t0) < args.duration:
            ex.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.stop()
    try:
        s = node.summary()
    except Exception as e:  # 汇总崩了也必须把原始序列落盘（否则整跑白跑）
        import traceback
        traceback.print_exc()
        s = {'summary_error': repr(e)}
        with node.lock:
            s['_raw'] = dict(
                stamps=np.array([r[0] for r in node.scan]),
                wall_arr=np.array([r[1] for r in node.scan]),
                sim_arr=np.array([r[2] for r in node.scan]),
                tf_ok=np.array([r[3] for r in node.scan], dtype=bool),
                wait_sim=np.full(len(node.scan), np.nan),
                wait_wall=np.full(len(node.scan), np.nan),
                tf_ob=np.array(node.tf_ob) if node.tf_ob else np.zeros((0, 3)),
                pose=np.array(node.pose) if node.pose else np.zeros((0, 3)),
                clocks=np.array(node.clocks) if node.clocks else np.zeros((0, 2)),
                maps=np.array(node.maps) if node.maps else np.zeros((0, 7)))
    raw = s.pop('_raw')
    np.savez_compressed(args.out + '.npz', **raw)
    with open(args.out + '.json', 'w') as f:
        json.dump(s, f, ensure_ascii=False, indent=1)
    print('PROBETIMINGJSON ' + json.dumps(
        {k: v for k, v in s.items() if k not in ('tf_frames',)}, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
