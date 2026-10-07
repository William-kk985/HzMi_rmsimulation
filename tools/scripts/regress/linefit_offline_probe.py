#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""linefit_offline_probe.py —— 离线复算 linefit 的"能不能拟合出地面线"（纯几何，无 ROS）。

为什么需要它（docs/robot_models.md §3.2）：`robot:=hzmirm` 的雷达离地 0.80 m，
而 MID360 的下视只有 −7.22° ⇒ **6.3 m 以内的地面根本不在视野里**。
"地面分割几乎全废"到底是"参数没标定对"还是"几何决定的"，需要在**不跑 Gazebo** 的情况下
把 linefit 的判定逐条复算一遍才能说清楚。

本脚本把上游 `linefit_ground_segmentation`（BSD-2-Clause，pin 3e6903a1）的
  · `GroundSegmentation::insertPoints` 的分箱（水平距离、n_bins=120、r∈[0.2,50]）
  · `Segment::fitSegmentLines` 的逐段直线拟合（long_threshold / max_long_height / max_fit_error）
  · `assignClusterThread` 的 verticalDistanceToLine + max_dist_to_line
按**同样的门限**在 Python 里重算一遍，输入是 robot_model_probe.py `--dump-cloud` 落下的
(x,y,z) 帧（**传感器系**；要模拟 gravity_aligned_frame，先把点云绕传感器原点旋转到那个姿态）。

用法：
  python3 tools/scripts/regress/linefit_offline_probe.py <frame.csv> \
      [--sensor-height 0.8] [--pitch-deg 0] [--r-min 0.2] [--r-max 50] [--n-bins 120]
      [--n-segments 360] [--long-threshold 1.0] [--max-long-height 0.1] [--max-start-height 0.5]
      [--max-dist-to-line 0.05] [--max-fit-error 0.05] [--max-slope 0.4]
校验口径：拿默认模型（离地 0.226 m）的一帧跑，地面比例应当 ≈ 实测的 43%
          （default2：2707/6270）；对不上就说明本脚本的复算与 C++ 有偏差，别拿它下结论。
"""
import argparse
import math

import numpy as np


class Segment:
    """与 upstream Segment 一一对应（只保留判定用得到的部分）。"""

    def __init__(self, n_bins, bin_step, r_min, max_slope, max_error, long_threshold,
                 max_long_height, max_start_height, sensor_height):
        self.bin_step = bin_step
        self.r_min = r_min
        self.min_z = np.full(n_bins, np.nan)      # 每个 bin 的"最低点 z"
        self.min_d = np.full(n_bins, np.nan)      # 对应水平距离
        self.lines = []                            # [(d1,z1,d2,z2)]
        self.max_slope = max_slope
        self.max_error = max_error
        self.long_threshold = long_threshold
        self.max_long_height = max_long_height
        self.max_start_height = max_start_height
        self.sensor_height = sensor_height

    def add_point(self, d, z):
        # 与 insertionThread 一致：(range - r_min)/bin_step（真值直接由调用方算好传进来）
        i = self.bin_index
        if i is None:
            return
        if np.isnan(self.min_z[i]) or z < self.min_z[i]:
            self.min_z[i] = z
            self.min_d[i] = d

    # --- 直线拟合（同 upstream fitLocalLine：最小二乘 z = a*d + b）---
    @staticmethod
    def _fit(points):
        d = np.array([p[0] for p in points])
        z = np.array([p[1] for p in points])
        A = np.vstack([d, np.ones_like(d)]).T
        a, b = np.linalg.lstsq(A, z, rcond=None)[0]
        return a, b

    def fit_lines(self):
        idx = [i for i in range(len(self.min_z)) if not np.isnan(self.min_z[i])]
        if not idx:
            return
        pts = [(self.min_d[i], self.min_z[i]) for i in idx]
        cur = [pts[0]]
        cur_line = (0.0, 0.0)
        is_long = False
        cur_ground_height = -self.sensor_height
        k = 1
        while k < len(pts):
            p = pts[k]
            if p[0] - cur[-1][0] > self.long_threshold:
                is_long = True
            if len(cur) >= 2:
                expected_z = float('inf')
                if is_long and len(cur) > 2:
                    expected_z = cur_line[0] * p[0] + cur_line[1]
                cur.append(p)
                a, b = self._fit(cur)
                err = max(((a * q[0] + b) - q[1]) ** 2 for q in cur)
                if (err > self.max_error or abs(a) > self.max_slope
                        or (is_long and abs(expected_z - p[1]) > self.max_long_height)):
                    cur.pop()
                    if len(cur) >= 3:
                        a2, b2 = self._fit(cur)
                        self.lines.append((cur[0][0], a2 * cur[0][0] + b2,
                                           cur[-1][0], a2 * cur[-1][0] + b2))
                        cur_ground_height = a2 * cur[-1][0] + b2
                    is_long = False
                    cur = [cur[-1]]
                    k -= 1     # upstream 的 --line_iter
                else:
                    cur_line = (a, b)
            else:
                if (p[0] - cur[-1][0] < self.long_threshold
                        and abs(cur[-1][1] - cur_ground_height) < self.max_start_height):
                    cur.append(p)
                else:
                    cur = [p]
            k += 1
        if len(cur) > 2:
            a, b = self._fit(cur)
            self.lines.append((cur[0][0], a * cur[0][0] + b, cur[-1][0], a * cur[-1][0] + b))

    def distance_to_line(self, d, z):
        dist = -1.0
        for (d1, z1, d2, z2) in self.lines:
            if d1 - 0.1 < d and d2 + 0.1 > d:
                dd = d2 - d1
                if abs(dd) < 1e-9:
                    continue
                expected = (d - d1) / dd * (z2 - z1) + z1
                dist = abs(z - expected)
        return dist


class Linefit:
    def __init__(self, a):
        self.a = a

    def run(self, cloud):
        r_min, r_max, n_bins, n_seg = self.a.r_min, self.a.r_max, self.a.n_bins, self.a.n_segments
        bin_step = (math.sqrt(r_max ** 2) - math.sqrt(r_min ** 2)) / n_bins
        seg_step = 2 * math.pi / n_seg
        segs = [Segment(n_bins, bin_step, r_min, self.a.max_slope, self.a.max_fit_error ** 2,
                        self.a.long_threshold, self.a.max_long_height,
                        self.a.max_start_height, self.a.sensor_height) for _ in range(n_seg)]
        pts = []
        for (x, y, z) in cloud:
            rs = x * x + y * y
            if rs >= r_max ** 2 or rs <= r_min ** 2:
                pts.append(None)
                continue
            r = math.sqrt(rs)
            i = int((r - r_min) / bin_step)
            if i < 0 or i >= n_bins:            # upstream 用 unsigned，越界会写到别的段上；这里保守丢弃
                pts.append(None)
                continue
            s = int((math.atan2(y, x) + math.pi) / seg_step)
            s = 0 if s == n_seg else s
            if s < 0 or s >= n_seg:
                pts.append(None)
                continue
            segs[s].bin_index = i
            segs[s].add_point(r, z)
            pts.append((s, r, z))
        for s in segs:
            s.fit_lines()
        ground = np.zeros(len(cloud), dtype=bool)
        for i, p in enumerate(pts):
            if p is None:
                continue
            s, r, z = p
            dist = segs[s].distance_to_line(r, z)
            steps = 1
            while dist < 0 and steps * seg_step < self.a.line_search_angle:
                dist = segs[(s + steps) % n_seg].distance_to_line(r, z)
                if dist < 0:
                    dist = segs[(s - steps) % n_seg].distance_to_line(r, z)
                steps += 1
            if dist < self.a.max_dist_to_line and dist != -1:
                ground[i] = True
        n_lines = sum(len(s.lines) for s in segs)
        segs_with_lines = sum(1 for s in segs if s.lines)
        return ground, n_lines, segs_with_lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frame', help='robot_model_probe.py --dump-cloud 落下的 cloud_frame_XX.csv')
    ap.add_argument('--sensor-height', type=float, default=0.8)
    ap.add_argument('--roll-deg', type=float, default=0.0,
                    help='把点云绕 x 轴转这么多度（模拟 robot11 那种**绕 roll** 的安装：'
                         'gravity_aligned_frame 生效时等价于先转 −roll 再分割；Phase 3 新增）')
    ap.add_argument('--pitch-deg', type=float, default=0.0,
                    help='把点云绕传感器原点转 -pitch（模拟 gravity_aligned_frame 起作用的姿态）')
    ap.add_argument('--r-min', type=float, default=0.2)
    ap.add_argument('--r-max', type=float, default=50.0)
    ap.add_argument('--n-bins', type=int, default=120)
    ap.add_argument('--n-segments', type=int, default=360)
    ap.add_argument('--long-threshold', type=float, default=1.0)
    ap.add_argument('--max-long-height', type=float, default=0.1)
    ap.add_argument('--max-start-height', type=float, default=0.5)
    ap.add_argument('--max-dist-to-line', type=float, default=0.05)
    ap.add_argument('--max-fit-error', type=float, default=0.05)
    ap.add_argument('--max-slope', type=float, default=0.4)
    ap.add_argument('--line-search-angle', type=float, default=0.8)
    a = ap.parse_args()

    d = np.genfromtxt(a.frame, delimiter=',', names=True)
    cloud = np.vstack([d['x'], d['y'], d['z']]).T
    if a.roll_deg:
        t = math.radians(a.roll_deg)
        R = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
        cloud = cloud @ R.T
    if a.pitch_deg:
        t = math.radians(a.pitch_deg)
        R = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
        cloud = cloud @ R.T
    ground, n_lines, segs_with_lines = Linefit(a).run(cloud)
    z = cloud[:, 2]
    print('帧: %s  点数=%d  sensor_height=%.3f  roll=%.1f°  pitch=%.1f°'
          % (a.frame, len(cloud), a.sensor_height, a.roll_deg, a.pitch_deg))
    print('  z 范围 %.3f .. %.3f   地面点数（真值，z 最低那层附近）= %d'
          % (z.min(), z.max(), int((np.abs(z + a.sensor_height) < 0.05).sum())))
    print('  拟合出的地面线 %d 条（分布在 %d/%d 个 segment）' % (n_lines, segs_with_lines, a.n_segments))
    print('  linefit 判为地面的点数 = %d / %d = %.1f%%'
          % (int(ground.sum()), len(cloud), 100.0 * ground.sum() / max(1, len(cloud))))
    if ground.sum():
        print('  其中"真的是最低那层地面"的比例 = %.1f%%'
              % (100.0 * (np.abs(z[ground] + a.sensor_height) < 0.05).sum() / ground.sum()))


if __name__ == '__main__':
    main()
