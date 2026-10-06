#!/usr/bin/env python3
"""离线预演 coverage_drive.py 的 PathFollower：不跑 Gazebo，只用一个"理想点车"跟路线。

为什么要这个：`PathFollower` 有几个踩坑补丁（只从当前段往后搜、单调不回退、候选段要用航向判定），
**路线一旦"回到起点附近"就会在 t=0 把索引瞬移到末尾**（实测 A2：车 0.0 m 没动，wp 索引已经到 4/6）。
这个脚本用 2 秒就能在跑仿真之前把所有候选路线筛一遍。

理想点车模型：与 coverage_drive 主循环同一套控制律（纯追踪 + |herr|>2.2 原地转 + 速度随距离衰减）。
"""
import argparse
import math
import sys

sys.path.insert(0, 'tools/scripts/mapping')
from coverage_drive import PathFollower, wrap  # noqa: E402


def simulate(pts, look=0.8, speed=0.40, dt=0.1, tmax=200.0, yaw_gain=1.2, yaw_max=0.8):
    f = PathFollower(pts, look)
    x, y, yaw = pts[0][0], pts[0][1], 0.0
    rs = []          # (t, x, y, i, carrot)
    jump = None
    path = 0.0
    t = 0.0
    while t < tmax:
        cx, cy, i, done = f.update(x, y, yaw)
        if jump is None and i > 1 and path < 0.05:
            jump = (t, i)
        if done:
            break
        dx, dy = cx - x, cy - y
        dist = math.hypot(dx, dy)
        herr = wrap(math.atan2(dy, dx) - yaw)
        if abs(herr) > 2.2:
            v, w = 0.0, max(-yaw_max, min(yaw_max, yaw_gain * herr))
        else:
            slow = 1.0 - 0.45 * min(1.0, abs(herr) / math.pi)
            v = max(0.05, min(speed, speed * max(0.35, min(1.0, dist / 0.6)))) * slow
            w = max(-yaw_max, min(yaw_max, yaw_gain * herr))
        yaw = wrap(yaw + w * dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        path += v * dt
        t += dt
        rs.append((t, x, y, i, cx, cy))
    return {'pts': pts, 'total': f.total, 'sim_len': path, 't': t, 'done': f.i >= len(pts) - 1,
            'jump': jump, 'trace': rs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--points', required=True, help='分号分隔 x,y')
    ap.add_argument('--look', type=float, default=0.8)
    ap.add_argument('--speed', type=float, default=0.40)
    args = ap.parse_args()
    pts = [[float(v) for v in p.split(',')] for p in args.points.split(';')]
    r = simulate(pts, look=args.look, speed=args.speed)
    print('[sim] 计划 %.2f m → 理想点车走 %.2f m / %.1f s，走完=%s，起步索引瞬移=%s'
          % (r['total'], r['sim_len'], r['t'], r['done'], r['jump']))
    last = -1
    for (t, x, y, i, cx, cy) in r['trace']:
        if i != last:
            print('   t=%5.1f s  wp→%d  车(%6.2f,%6.2f)  胡萝卜(%6.2f,%6.2f)' % (t, i, x, y, cx, cy))
            last = i
    if r['jump']:
        print('[sim] ❌ 起步就瞬移（车没动索引已到 %d）⇒ 这条路线不能用' % r['jump'][1])
    elif not r['done']:
        print('[sim] ❌ 200 s 内没走完 ⇒ 路线可能自交/太绕')
    else:
        print('[sim] ✅ 索引单调推进，无起步瞬移')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
