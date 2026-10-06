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
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener

CLK = os.sysconf('SC_CLK_TCK')
CARE = ('gzserver', 'slam_toolbox', 'small_point_lio_node', 'pointcloud_to_laserscan_node',
        'ground_segmentation_node', 'robot_state_publisher', 'complementary_filter')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def proc_ticks():
    out = {}
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
        if comm in CARE:
            out[comm] = out.get(comm, 0) + ut + st
    return out


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
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_gt, q1)
        self.create_subscription(Clock, '/clock', self.on_clock, qb)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qmap)
        self.create_subscription(MarkerArray, '/slam_toolbox/graph_visualization',
                                 self.on_graph, q1)
        self.t0 = time.time()
        self._cpu_last = proc_ticks()
        self._cpu_t = time.time()
        self._clk_last = None
        self._n = 0

    # ---------------- 回调
    def on_gt(self, m):
        self.gt = m

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

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
        cur = proc_ticks()
        dt = now - self._cpu_t
        if dt > 0:
            for k, v in cur.items():
                d = v - self._cpu_last.get(k, v)
                cpu[k] = round(100.0 * d / CLK / dt, 1)
        self._cpu_last, self._cpu_t = cur, now
        rtf = None
        if self.clock is not None:
            if self._clk_last is not None:
                dclk = self.clock - self._clk_last[0]
                dw = now - self._clk_last[1]
                if dw > 0:
                    rtf = round(dclk / dw, 3)
            self._clk_last = (self.clock, now)
        self.emit({'k': 'sys', 'rtf': rtf, 'cpu': cpu,
                   'load': [round(x, 2) for x in os.getloadavg()]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--dur', type=float, default=1200.0)
    args = ap.parse_args()

    rclpy.init()
    n = Rec(args.out)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    t_end = time.time() + args.dur
    t_sys = time.time()
    per = 1.0 / args.hz
    while rclpy.ok() and time.time() < t_end:
        ex.spin_once(timeout_sec=0.005)
        now = time.time()
        if now - t_sys >= 1.0:
            n.sysline()
            t_sys = now
        n.sample()
        time.sleep(per * 0.5)
    n.emit({'k': 'end'})
    print('[rec] done, samples=%d' % n._n)
    n.f.close()


if __name__ == '__main__':
    main()
