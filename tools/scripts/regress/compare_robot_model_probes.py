#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compare_robot_model_probes.py —— 把若干份 robot_model_probe.py 的 JSON 并排打印。

用法：
  python3 tools/scripts/regress/compare_robot_model_probes.py \
      .tmp_robotslot/default2/probe.json .tmp_robotslot/hzmirm2/probe.json ...
（第一列是指标，其余每列一份探针结果；docs/robot_models.md §5 的表就是它打出来的）
"""
import json
import sys


def g(d, *ks):
    cur = d
    for k in ks:
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        else:
            return None
    return cur


ROWS = [
    ('RTF', [('rtf',)]),
    ('点云频率 Hz', [('rates_hz', 'cloud')]),
    ('IMU 频率 Hz', [('rates_hz', 'imu')]),
    ('/scan 频率 Hz', [('rates_hz', 'scan')]),
    ('点云 帧数', [('cloud', 'points_per_frame', 'frames')]),
    ('点云 点/帧 min/med/max', [('cloud', 'points_per_frame', 'min'),
                                ('cloud', 'points_per_frame', 'median'),
                                ('cloud', 'points_per_frame', 'max')]),
    ('点云 z 范围 min/max', [('cloud', 'z_min'), ('cloud', 'z_max')]),
    ('点云 z@r<1.5 / r<3.0', [('cloud', 'z_min_r_lt_1.5'), ('cloud', 'z_min_r_lt_3.0')]),
    ('点云 最大 z 峰 (z,n)', [('cloud', 'ground_peak_z'), ('cloud', 'ground_peak_points')]),
    ('点云 r<0.5 的点数', [('cloud', 'near_points_r_lt_0.5')]),
    ('z 直方图 top4', [('cloud', 'z_hist_top6',)]),
    ('obstacle 点/帧(中位)', [('obstacle_points_per_frame', 'median')]),
    ('ground 点/帧(中位)', [('ground_points_per_frame', 'median')]),
    ('/scan 有效波束 med', [('scan', 'beams_finite_median')]),
    ('/scan 有效波束 min/max', [('scan', 'beams_finite_min'), ('scan', 'beams_finite_max')]),
    ('/scan 总波束', [('scan', 'last', 'n_beams')]),
    ('/scan inf 占比 med', [('scan', 'inf_ratio_median')]),
    ('/scan 最近回波集合', [('scan', 'nearest_ranges',)]),
    ('/scan 分带束数', [('scan', 'band_beams_total',)]),
    ('真值 路径长 m', [('ground_truth', 'path_len')]),
    ('LIO  路径长 m', [('odom', 'path_len')]),
    ('真值 终点 x,y,z', [('ground_truth', 'last', 'x'), ('ground_truth', 'last', 'y'),
                         ('ground_truth', 'last', 'z')]),
    ('LIO  终点 x,y,z', [('odom', 'last', 'x'), ('odom', 'last', 'y'), ('odom', 'last', 'z')]),
    ('LIO 位移-真值位移 (x,y)', None),   # 特殊处理
    ('LIO 路径/真值 路径', [('drift', 'path_len_ratio')]),
    ('倾角 max (LIO r/p, 真值 r/p)', [('tilt_max_deg', 'odom_roll_pitch'),
                                      ('tilt_max_deg', 'gt_roll_pitch')]),
    ('base_link→livox_frame', [('tf', 'livox_frame', 'xyz')]),
    ('base_link→imu_link', [('tf', 'imu_link', 'xyz')]),
    ('base_link→turret_head', [('tf', 'turret_head', 'xyz')]),
    ('驾驶秒数', [('drive_seconds',)]),
    ('墙钟秒数', [('wall_seconds',)]),
]


def fmt(v):
    if isinstance(v, float):
        return '%.3f' % v
    if isinstance(v, list):
        return '[' + ', '.join(fmt(x) for x in v) + ']'
    return str(v)


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        return 2
    data = [(p, json.load(open(p))) for p in paths]
    names = [p.split('/')[-2] if p.endswith('probe.json') else p for p in paths]
    colw = max(22, max(len(n) for n in names) + 2)
    print('%-26s | %s' % ('指标', ' | '.join(n.ljust(colw) for n in names)))
    print('-' * (28 + (colw + 3) * len(names)))
    for label, keys in ROWS:
        cells = []
        for _, d in data:
            if keys is None:
                gt, od = g(d, 'ground_truth', 'last'), g(d, 'odom', 'last')
                if not gt or not od:
                    cells.append('n/a')
                else:
                    cells.append('[%s, %s]' % (fmt(round(od['x'] - gt['x'], 3)),
                                               fmt(round(od['y'] - gt['y'], 3))))
            else:
                vals = [g(d, *k) for k in keys]
                if len(vals) == 1:
                    cells.append(fmt(vals[0]))
                else:
                    cells.append(' / '.join(fmt(v) for v in vals))
        print('%-26s | %s' % (label, ' | '.join(c.ljust(colw) for c in cells)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
