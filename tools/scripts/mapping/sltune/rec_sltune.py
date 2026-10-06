#!/usr/bin/env python3
"""slam_toolbox 调参 A/B 的记录仪（一条 JSONL = 一次跑图的全部原始证据）。

记录内容（全部是**只读订阅 + TF 查询**，不发任何东西 ⇒ 不干扰被测系统）：
  pose  : map→base_link（融合估计）、odom→base_link（LIO）、/odom_ground_truth（真值）
  graph : /slam_toolbox/graph_visualization 的 V（顶点数）与 E（边数）
          ⇒ 环秩 = E - V + 1 是"图里存在回环约束"的**直接**证据（无回环时应恒为 0）
  map   : /map 的占用/已知格数（看回访后地图有没有"抹平"))
  sys   : RTF（/clock 增量 / 墙钟增量）+ 进程 CPU（/proc/<pid>/stat 增量）
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

CLK = os.sysconf('SC_CLK_TCK')
# ⚠️ /proc/<pid>/stat 的 comm 字段**最多 15 字符**：`async_slam_toolbox_node` 在那里是
#    `async_slam_tool`、`small_point_lio_node` 是 `small_point_lio`。所以这里按
#    「可执行名（截断到 15 字符）」匹配；python 起的脚本（真值桥）comm 是 `python3`，
#    只能退回去读 cmdline。2026-10-06 修：没有这个，slam_toolbox 的 CPU 永远统计不到，
#    而丢帧归因正需要它（docs/slam_drops_loopclosure_odom.md §2）。
CARE_TOKENS = {
    'gzserver': 'gzserver',
    'slam_toolbox': 'async_slam_toolbox_node',
    'small_point_lio': 'small_point_lio_node',
    'pointcloud_to_laserscan': 'pointcloud_to_laserscan_node',
    'ground_segmentation': 'ground_segmentation_node',
    'robot_state_publisher': 'robot_state_publisher',
    'complementary_filter': 'complementary_filter_node',
    'cloud_accumulator': 'cloud_accumulator_node',
    'truth_odom_bridge': 'truth_odom_tf_bridge.py',
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _care(comm, cmdline):
    for key, tok in CARE_TOKENS.items():
        if comm == tok[:15] or (cmdline and tok in cmdline):
            return key
    return None


def proc_ticks():
    """返回 (按槽位聚合的 ticks, 全体进程 ticks 之和)。

    第二条（全机总数）是"机器是不是被吃满"的粗指标 —— Ceres 一次 `Solve` 会申请 50 线程
    （被 bound 到 28），那一刻全机 CPU 会跳，用来给丢帧做归因（见
    docs/slam_drops_loopclosure_odom.md §2）。
    """
    out = {}
    total = 0
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % pid) as f:
                s = f.read()
            rp = s.rindex(')')
            comm = s[s.index('(') + 1:rp]
            flds = s[rp + 2:].split()
            ut, st = int(flds[11]), int(flds[12])
        except Exception:
            continue
        total += ut + st
        cmdline = None
        if comm.startswith('python'):    # 只有 python 起的脚本需要看 cmdline
            try:
                with open('/proc/%s/cmdline' % pid, 'rb') as f:
                    cmdline = f.read().decode('utf-8', 'replace')
            except Exception:
                cmdline = ''
        k = _care(comm, cmdline)
        if k is not None:
            out[k] = out.get(k, 0) + ut + st
    return out, total


class Rec(Node):
    def __init__(self, out_path):
        super().__init__('sltune_rec')
        self.f = open(out_path, 'w')
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.gt = None
        self.clock = None
        self.mapmsg = None
        self.nmap = 0
        self.ngraph = 0
        self.last_graph = None
        q1 = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
        qb = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        qmap = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST)
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_gt, q1,
                                 callback_group=cb)
        self.create_subscription(Clock, '/clock', self.on_clock, qb, callback_group=cb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qmap, callback_group=cb)
        self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization',
                                 self.on_graph, q1, callback_group=cb)
        self.t0 = time.time()
        self.clock_hist = collections.deque(maxlen=2000)
        self._cpu_last, self._cpu_total_last = proc_ticks()
        self._cpu_t = time.time()
        self._clk_last = 0.0
        self._n = 0

    # ---------------- 回调
    def on_gt(self, m):
        self.gt = m

    def on_clock(self, m):
        c = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock = c
        # ⚠️ 不要用"最新值相减"算 RTF：/clock 的**消息间隔**（实测 ~7.7 Hz）与采样间隔
        #    （--sys-hz）会别名化，量出来的 RTF 会在 0.49/1.47 之间跳（2026-10-06 踩过）。
        #    正确做法 = 在窗口内对 /clock 的**消息序列**求 ΣΔsim/ΣΔwall（与探针同口径）。
        self.clock_hist.append((c, time.time()))

    def on_map(self, m):
        self.mapmsg = m
        self.nmap += 1
        n = len(m.data)
        occ = sum(1 for v in m.data if v >= 50)
        known = sum(1 for v in m.data if v >= 0)
        self.emit({'k': 'map', 'n': self.nmap, 'w': m.info.width, 'h': m.info.height,
                   'occ': occ, 'known': known, 'cells': n,
                   'origin': [round(m.info.origin.position.x, 3),
                              round(m.info.origin.position.y, 3)]})

    def on_graph(self, m):
        self.ngraph += 1
        V = E = 0
        maxe = 0.0
        nover = 0
        for mk in m.markers:
            if mk.action == 3:  # DELETEALL
                continue
            if mk.type == 2:  # SPHERE = 顶点
                V += 1
            elif mk.type == 5:  # LINE_LIST = 边
                E += len(mk.points) // 2
                for i in range(0, len(mk.points) - 1, 2):
                    a, b = mk.points[i], mk.points[i + 1]
                    d = math.dist((a.x, a.y), (b.x, b.y))
                    maxe = max(maxe, d)
                    if d > 1.0:
                        nover += 1
        # 顶点位置（用于事后核对"节点间距"，chain_size 的几何闸门直接由它决定）
        vpts = [[round(mk.pose.position.x, 3), round(mk.pose.position.y, 3)]
                for mk in m.markers if mk.type == 2 and mk.action != 3]
        sp = [math.dist(vpts[i], vpts[i + 1]) for i in range(len(vpts) - 1)]
        self.emit({'k': 'graph', 'n': self.ngraph, 'V': V, 'E': E, 'cyc': E - V + 1,
                   'maxedge': round(maxe, 3), 'nedge_gt1m': nover,
                   'sp_min': round(min(sp), 3) if sp else None,
                   'sp_mean': round(sum(sp) / len(sp), 3) if sp else None,
                   'sp_max': round(max(sp), 3) if sp else None,
                   'vpts': vpts[:60]})

    # ---------------- 工具
    def emit(self, d):
        d['tw'] = round(time.time() - self.t0, 4)
        d['ts'] = round(self.clock, 3) if self.clock is not None else None
        self.f.write(json.dumps(d) + '\n')
        self.f.flush()

    def tf_pose(self, parent, child):
        try:
            tr = self.buf.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        return [round(t.x, 4), round(t.y, 4), round(yaw_of(tr.transform.rotation), 5)]

    def sample(self):
        p = {'k': 'pose'}
        m = self.tf_pose('map', 'base_link')
        o = self.tf_pose('odom', 'base_link')
        if m:
            p['map'] = m
        if o:
            p['odom'] = o
        if self.gt is not None:
            t = self.gt.pose.pose.position
            p['gt'] = [round(t.x, 4), round(t.y, 4),
                       round(yaw_of(self.gt.pose.pose.orientation), 5)]
        if 'map' in p or 'odom' in p or 'gt' in p:
            self.emit(p)

    def sysline(self):
        now = time.time()
        cpu = {}
        cur, cur_total = proc_ticks()
        dt = now - self._cpu_t
        tot = None
        if dt > 0:
            for k, v in cur.items():
                d = v - self._cpu_last.get(k, v)
                cpu[k] = round(100.0 * d / CLK / dt, 1)
            tot = round(100.0 * (cur_total - self._cpu_total_last) / CLK / dt, 1)
        self._cpu_last, self._cpu_total_last, self._cpu_t = cur, cur_total, now
        rtf = None
        clk_hz = None
        hist = [x for x in self.clock_hist if x[1] > now - 1.0]   # 滑动 1 s 窗口（≥3 个样本）
        if len(hist) >= 3:
            dsim = hist[-1][0] - hist[0][0]
            dwall = hist[-1][1] - hist[0][1]
            if dwall > 0:
                rtf = round(dsim / dwall, 3)
                clk_hz = round((len(hist) - 1) / dwall, 2)
        if self.clock_hist:
            self._clk_last = self.clock_hist[-1][1]
        self.emit({'k': 'sys', 'rtf': rtf, 'clk_hz': clk_hz, 'cpu': cpu,
                   'cpu_total_pct': tot,
                   'load': [round(x, 2) for x in os.getloadavg()]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10.0, help='位姿采样率（Hz，默认 10）')
    ap.add_argument('--sys-hz', type=float, default=1.0,
                    help='CPU/RTF 采样率（Hz，默认 1；归因 Ceres 尖峰时可调到 5）')
    ap.add_argument('--dur', type=float, default=1200.0, help='采集时长（墙钟秒）')
    args = ap.parse_args()

    rclpy.init()
    n = Rec(args.out)
    # 多线程 executor：~80 Hz 的 /tf 洪水下，单线程 spin_once 会把回调饿死（docs/worlds.md §5.1）。
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    t_end = time.time() + args.dur
    t_sys = time.time()
    per = 1.0 / args.hz
    while rclpy.ok() and time.time() < t_end:
        ex.spin_once(timeout_sec=0.005)
        now = time.time()
        if now - t_sys >= 1.0 / max(args.sys_hz, 1e-3):
            n.sysline()
            t_sys = now
        n.sample()
        time.sleep(per * 0.5)
    n.emit({'k': 'end'})
    print('[rec] done, samples=%d' % n._n)
    n.f.close()


if __name__ == '__main__':
    main()
