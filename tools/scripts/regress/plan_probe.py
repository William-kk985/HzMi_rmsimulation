#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plan_probe.py —— 直接问 nav2 的规划器"这几个目标你能不能规划出来"（离线判据，不发车）。

为什么不用"发导航目标看它成不成"：GoalStatus 里混着控制器、恢复行为、代价图更新等一堆因素，
"A/B 两张图哪个能穿过坡道"这件事只取决于**规划器 + 静态层**。本工具只调
`nav2_msgs/action/ComputePathToPose`，一次问一组 (start, goal)，把
"有没有路径 / 路径多长 / 失败原因"原样打出来 ⇒ 是 A/B 的**可复现判据**。

用法（栈要已经在跑：`mode:=nav ...`）：
    python3 tools/scripts/regress/plan_probe.py --goal 0.5 3.0 --goal -6.6 -7.0 \
        --goal -5.0 -5.0 --start 0.0 0.0 --out .tmp_cache/plan/probe.json

语义：`--start` 不给就用机器人当前位姿（use_start=false）；
`use_start=true` 时由本工具灌一个显式起点（用来问"从出生点能不能到坡道"）。
退出码：0 = 至少有一个目标规划成功；3 = 全部失败（"规划器拒绝"）。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Probe(Node):
    def __init__(self, a):
        super().__init__('plan_probe')
        self.a = a
        self.cli = ActionClient(self, ComputePathToPose, a.action)

    def wait(self, timeout):
        return self.cli.wait_for_server(timeout_sec=timeout)

    def ask(self, sx, sy, syaw, gx, gy, gyaw, timeout):
        goal = ComputePathToPose.Goal()
        goal.use_start = bool(self.a.use_start)
        if goal.use_start:
            ps = PoseStamped()
            ps.header.frame_id = self.a.frame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = sx
            ps.pose.position.y = sy
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(syaw / 2), math.cos(syaw / 2)
            goal.start = ps
        g = PoseStamped()
        g.header.frame_id = self.a.frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = gx
        g.pose.position.y = gy
        g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        goal.goal = g
        goal.planner_id = self.a.planner_id

        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return {'goal': [gx, gy, gyaw], 'accepted': False, 'ok': False,
                    'why': 'goal 没被接受（planner server 在吗？）'}
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        res = rf.result()
        if res is None:
            return {'goal': [gx, gy, gyaw], 'accepted': True, 'ok': False, 'why': '超时'}
        path = res.result.path
        n = len(path.poses)
        L = 0.0
        for i in range(1, n):
            a = path.poses[i - 1].pose.position
            b = path.poses[i].pose.position
            L += math.dist((a.x, a.y), (b.x, b.y))
        return {'goal': [round(gx, 3), round(gy, 3), round(gyaw, 3)],
                'accepted': True, 'status': int(res.status), 'poses': n,
                'path_len_m': round(L, 3), 'ok': n > 0,
                'why': '' if n > 0 else '规划器返回空路径（拒绝 / 目标在代价图里是致命或未知）'}


def main():
    ap = argparse.ArgumentParser(description='用 nav2 ComputePathToPose 做"能不能规划过去"的 A/B 判据')
    ap.add_argument('--goal', nargs='+', type=float, action='append', required=True,
                    metavar='X Y [YAW]', help='目标（可重复）')
    ap.add_argument('--start', nargs='+', type=float, default=None, metavar='X Y [YAW]',
                    help='显式起点（不给 = 用机器人当前位姿）')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--action', default='/compute_path_to_pose')
    ap.add_argument('--planner-id', default='')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--out', help='结果 JSON')
    ap.add_argument('--label', default='', help='贴个标签写进 JSON（例如 map 名字）')
    args = ap.parse_args()
    args.use_start = args.start is not None
    goals = []
    for g in args.goal:
        goals.append((g[0], g[1], g[2] if len(g) > 2 else 0.0))
    s = (0.0, 0.0, 0.0)
    if args.start:
        s = (args.start[0], args.start[1], args.start[2] if len(args.start) > 2 else 0.0)

    rclpy.init()
    node = Probe(args)
    out = {'label': args.label, 'start': list(s), 'use_start': args.use_start,
           'planner_server': False, 'results': []}
    if not node.wait(args.timeout):
        print('PLANPROBE ' + json.dumps({**out, 'why': '没有 %s action server' % args.action},
                                        ensure_ascii=False))
        node.destroy_node(); rclpy.shutdown()
        return 3
    out['planner_server'] = True
    for (gx, gy, gyaw) in goals:
        r = node.ask(s[0], s[1], s[2], gx, gy, gyaw, args.timeout)
        out['results'].append(r)
        print('[plan] goal=(%.2f,%.2f,%.2f) ok=%s poses=%s len=%s %s'
              % (gx, gy, gyaw, r['ok'], r.get('poses'), r.get('path_len_m'), r.get('why', '')),
              flush=True)
    out['n_ok'] = sum(1 for r in out['results'] if r['ok'])
    print('PLANPROBE ' + json.dumps(out, ensure_ascii=False))
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if out['n_ok'] else 3


if __name__ == '__main__':
    sys.exit(main())
