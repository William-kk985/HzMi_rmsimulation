#!/usr/bin/env python3
# =============================================================================
# gz_livox_probe.py —— `sim_backend:=gz` 的契约/性能探针（只订阅，不发布）
# =============================================================================
# 一个进程里同时订阅两路（双实例的验收就是"两路同时有人收，且互相比对"）：
#   /livox/lidar            livox_ros_driver2/CustomMsg
#   /livox/lidar/pointcloud sensor_msgs/PointCloud2
#
# 它回答文档里每一条需要"实测"的问题：
#   · 频率：仿真时间（header.stamp 线性回归）与墙钟（回调到达时刻）各是多少
#   · 每帧点数：min/median/max
#   · offset_time：最大跨度、是否单调
#   · line 字段：取值集合
#   · frame_id：两路分别是什么（应为 livox_frame）
#   · 点云字段布局：fields/point_step/是否带 rgb
#   · 两实例一致性：同一帧（stamp 相同）两路的点数、逐点 xyz、时间戳是否一致
#   · **慢消费者**：--slow-ms 让回调里 sleep，用来验证 best-effort 下 gz 线程不被堵
#
# 用法：
#   gz_livox_probe.py --frames 100                       # 双路，默认 best_effort
#   gz_livox_probe.py --frames 100 --slow-ms 2000        # 故意慢的订阅者
#   gz_livox_probe.py --frames 50 --only pc2 --dump /tmp/pc2.npz
#   gz_livox_probe.py --frames 50 --qos reliable         # 复现"可靠写者被堵"
#
# 输出：最后一行是 JSON（便于脚本 `| tail -1 | jq`），过程日志走 stderr。
# =============================================================================
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:                                    # pragma: no cover
    CustomMsg = None

F32 = PointCloud2.FLOAT32 if hasattr(PointCloud2, 'FLOAT32') else 7
DTYPE = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def log(msg):
    print('[probe] ' + msg, file=sys.stderr)


class Probe(Node):
    def __init__(self, a):
        super().__init__('gz_livox_probe')
        self.a = a
        self.custom = []
        self.pc2 = []
        self.clock = []
        self.pc2_layout = None
        self.pc2_dump = []
        self.custom_dump = []
        self.t0 = time.monotonic()
        self.late_wall = []
        if a.qos == 'reliable':
            qos = QoSProfile(depth=a.depth, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)
        else:
            qos = QoSProfile(depth=a.depth, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.qos = qos
        # /clock：判"gz 线程有没有被堵"的独立观测量（仿真时间停 = 被堵）
        self.create_subscription(Clock, '/clock', self.on_clock, qos)
        if a.only in ('both', 'custom'):
            if CustomMsg is None:
                raise SystemExit('livox_ros_driver2 不可用（没 source 本仓 install？）')
            self.create_subscription(CustomMsg, a.custom_topic, self.on_custom, qos)
        if a.only in ('both', 'pc2'):
            self.create_subscription(PointCloud2, a.pc2_topic, self.on_pc2, qos)

    def on_clock(self, msg):
        self.clock.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    # -- 慢消费者：回调里 sleep，模拟 slam_toolbox / RViz / STVL 那种背压 --------
    def _maybe_slow(self):
        if self.a.slow_ms > 0:
            time.sleep(self.a.slow_ms / 1000.0)

    def on_custom(self, msg):
        wall = time.monotonic()
        n = int(msg.point_num)
        offs = np.fromiter((p.offset_time for p in msg.points), dtype=np.uint64, count=n) \
            if n else np.zeros(0, dtype=np.uint64)
        lines = np.fromiter((p.line for p in msg.points), dtype=np.uint8, count=n) \
            if n else np.zeros(0, dtype=np.uint8)
        rec = dict(wall=wall, stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   frame_id=msg.header.frame_id, n=n, timebase=int(msg.timebase),
                   off_min=int(offs.min()) if n else None,
                   off_max=int(offs.max()) if n else None,
                   monotonic=bool(np.all(np.diff(offs) >= 0)) if n > 1 else True,
                   lines=sorted({int(v) for v in lines}))
        if self.a.dump and n:
            self.custom_dump.append(np.stack([
                np.fromiter((p.x for p in msg.points), dtype=np.float32, count=n),
                np.fromiter((p.y for p in msg.points), dtype=np.float32, count=n),
                np.fromiter((p.z for p in msg.points), dtype=np.float32, count=n),
            ], axis=1))
        self.custom.append(rec)
        self._maybe_slow()

    def on_pc2(self, msg):
        wall = time.monotonic()
        names = [f.name for f in msg.fields]
        offs = {f.name: f.offset for f in msg.fields}
        layout = dict(point_step=int(msg.point_step), fields=names,
                      offsets=offs, has_rgb=('rgb' in names or 'rgba' in names),
                      is_dense=bool(msg.is_dense), height=int(msg.height),
                      width=int(msg.width),
                      datatypes={f.name: int(f.datatype) for f in msg.fields})
        if self.pc2_layout is None:
            self.pc2_layout = layout
        n = int(msg.width) * int(msg.height)
        rec = dict(wall=wall, stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   frame_id=msg.header.frame_id, n=n, layout_ok=(layout == self.pc2_layout))
        if self.a.dump and n:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
            def col(name):
                o = offs[name]
                dt = DTYPE[layout['datatypes'][name]]
                return buf[:, o:o + np.dtype(dt).itemsize].copy().view(dt).reshape(-1)
            self.pc2_dump.append(np.stack([col('x'), col('y'), col('z')], axis=1))
            rec['intensity_unique'] = sorted({float(v) for v in col('intensity')})[:4]
            rec['tag_unique'] = sorted({int(v) for v in col('tag')})[:4]
            rec['line_unique'] = sorted({int(v) for v in col('line')})[:8]
            ts = col('timestamp').astype(np.float64)
            rec['ts_max_rel_ns'] = int(round((ts.max() - rec['stamp'] * 1e9)))
        self.pc2.append(rec)
        self._maybe_slow()

    # -- 汇总 ----------------------------------------------------------------
    def summary(self):
        out = dict(qos=self.a.qos, slow_ms=self.a.slow_ms,
                   custom_topic=self.a.custom_topic, pc2_topic=self.a.pc2_topic)
        for tag, recs in (('custom', self.custom), ('pc2', self.pc2)):
            if not recs:
                out[tag] = dict(frames=0)
                continue
            stamps = np.array([r['stamp'] for r in recs])
            walls = np.array([r['wall'] for r in recs])
            ns = np.array([r['n'] for r in recs], dtype=np.int64)
            d = dict(frames=len(recs),
                     frame_id=sorted({r['frame_id'] for r in recs}),
                     n_min=int(ns.min()), n_median=float(np.median(ns)), n_max=int(ns.max()),
                     wall_hz=float((len(recs) - 1) / (walls[-1] - walls[0])) if len(recs) > 1 else None,
                     sim_span_s=float(stamps[-1] - stamps[0]),
                     wall_span_s=float(walls[-1] - walls[0]))
            if len(recs) > 2:
                k = np.polyfit(np.arange(len(recs)), stamps, 1)[0]
                d['sim_hz'] = float(1.0 / k)
            # 仿真时间相对墙钟推进得比 1 慢 => RTF < 1：这里只做"点云在仿真里等间隔"的证据
            out[tag] = d
        if self.custom:
            offs = np.array([r['off_max'] for r in self.custom if r['off_max'] is not None])
            if offs.size:
                out['custom']['offset_time_max_ns'] = int(offs.max())
                out['custom']['offset_time_max_ms'] = float(offs.max() / 1e6)
            out['custom']['offset_monotonic_all'] = all(r['monotonic'] for r in self.custom)
            out['custom']['lines'] = sorted({v for r in self.custom for v in r['lines']})
            out['custom']['timebase_nonzero'] = any(r['timebase'] for r in self.custom)
        if self.pc2_layout:
            out['pc2_layout'] = self.pc2_layout
        # 两实例一致性：按 stamp 配对
        if self.custom and self.pc2:
            cmap = {round(r['stamp'], 6): r for r in self.custom}
            pairs = [(cmap[round(r['stamp'], 6)], r) for r in self.pc2
                     if round(r['stamp'], 6) in cmap]
            d = dict(matched=len(pairs),
                     custom_frames=len(self.custom), pc2_frames=len(self.pc2))
            if pairs:
                d['n_equal'] = sum(1 for c, p in pairs if c['n'] == p['n'])
                d['n_delta_max'] = int(max(abs(c['n'] - p['n']) for c, p in pairs))
                d['frame_id_equal'] = all(c['frame_id'] == p['frame_id'] for c, p in pairs)
                d['stamp_equal'] = True     # 按定义（配对就按 stamp 做的）
            out['parity'] = d
        if self.clock:
            walls = np.array([c[0] for c in self.clock])
            sims = np.array([c[1] for c in self.clock])
            d = dict(msgs=len(self.clock),
                     span_sim_s=float(sims[-1] - sims[0]),
                     span_wall_s=float(walls[-1] - walls[0]))
            if len(sims) > 1:
                # 相邻 clock 消息的**仿真时间**最大跳变：> 0.1 s 说明仿真被按停过
                d['max_gap_sim_s'] = float(np.max(np.diff(sims)))
            if d['span_wall_s'] > 0:
                # RTF（与 Gazebo 自己的口径同义：仿真时间推进 / 墙钟）
                d['rtf'] = float(d['span_sim_s'] / d['span_wall_s'])
            out['clock'] = d
        if self.a.dump and self.pc2_dump:
            np.savez(self.a.dump, pc2=np.concatenate(self.pc2_dump, axis=0),
                     custom=(np.concatenate(self.custom_dump, axis=0)
                             if self.custom_dump else np.zeros((0, 3), np.float32)))
            out['dump'] = self.a.dump
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=100, help='收到这么多帧就收工（两路各自计）')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--only', choices=['both', 'custom', 'pc2'], default='both')
    ap.add_argument('--custom-topic', default='/livox/lidar')
    ap.add_argument('--pc2-topic', default='/livox/lidar/pointcloud')
    ap.add_argument('--qos', choices=['best_effort', 'reliable'], default='best_effort')
    ap.add_argument('--depth', type=int, default=5)
    ap.add_argument('--slow-ms', type=float, default=0.0)
    ap.add_argument('--dump', default='')
    ap.add_argument('--settle', type=float, default=0.0, help='先等这么多秒再开始计帧')
    a = ap.parse_args()

    rclpy.init()
    node = Probe(a)
    if a.settle:
        t_end = time.monotonic() + a.settle
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.custom.clear()
        node.pc2.clear()
    t_end = time.monotonic() + a.timeout
    while time.monotonic() < t_end:
        n = max(len(node.custom), len(node.pc2))
        if n >= a.frames:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary()
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(s, ensure_ascii=False))


if __name__ == '__main__':
    main()
