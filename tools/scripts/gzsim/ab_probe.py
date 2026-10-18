#!/usr/bin/env python3
# =============================================================================
# ab_probe.py —— 一次「同协议」测量：Classic 与 gz 两个后端用**同一个探针**量同一批指标
# =============================================================================
# 为什么要有它：`docs/fortress_sim_backend.md` §2.8/§10-8 的并排表里，Classic 那一列
# 一直是**别的轮次**的历史基线（不同机器负载、不同世界版本）⇒ 只能看量级。本探针
# 就是"同轮 A/B"的尺：同一个进程、同一套订阅、同一段时长，只换后端。
#
# 量的东西（口径逐条写死）：
#   · RTF = Δ仿真时间(/clock) / Δ墙钟（同一段采样窗口内）
#   · 传感器频率：/livox/lidar 的**仿真** Hz（header.stamp 的线性回归/跨度）与**墙钟** Hz
#   · 每帧点数：CustomMsg.point_num 的中位/min/max
#   · 逐帧求交耗时：gz 侧取插件日志里的 `[MID360] Embree average raycast: X ms`
#     （每 100 帧一条的运行均值）取中位；Classic 侧没有插桩 ⇒ null。
#     两个后端**都有**的一个可比量是 `gzserver_cpu_ms_per_frame`（见下）。
#   · gzserver 的 RSS/CPU：按 /proc 里本 run 的 gzserver 进程树采样（0.5 s），
#     报 中位 RSS / 峰值 RSS / 平均 CPU%（单核百分比）；同时给
#     `cpu_ms_per_frame = CPU 秒 / 帧数`，这是"仿真服务端每帧成本"的统一口径
#     （Classic 的雷达插件是 gzserver 内的 plugin ⇒ 它的求交成本就在这个数里）。
#   · 启动 → 首帧：`--launch-epoch` 到本探针收到第一条 CustomMsg 的墙钟差
#   · /scan：有限波束数、range 的 min/max
#   · /cloud_registered：采样窗口内的条数 + width 中位（端到端可用性）
#
# 用法：
#   python3 tools/scripts/gzsim/ab_probe.py --backend gz --duration 20 \
#       --launch-epoch $(date +%s.%N) --log /path/launch.log --out /path/gz.json
# =============================================================================
import argparse
import json
import os
import statistics
import time


def _proc_tree(root):
    kids = {}
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % p) as f:
                st = f.read().split(') ', 1)[1].split()
            kids.setdefault(int(st[1]), []).append(int(p))
        except Exception:  # noqa: BLE001
            pass
    out, q = set(), [root]
    while q:
        c = q.pop()
        if c in out:
            continue
        out.add(c)
        q.extend(kids.get(c, []))
    return out


def _proc_stat(pid):
    """(comm, rss_kb, cpu_ticks)"""
    try:
        with open('/proc/%d/stat' % pid) as f:
            raw = f.read()
        comm = raw[raw.index('(') + 1:raw.rindex(')')]
        rest = raw[raw.rindex(')') + 2:].split()
        utime, stime = int(rest[11]), int(rest[12])
        rss = 0
        with open('/proc/%d/status' % pid) as f:
            for line in f:
                if line.startswith('VmRSS:'):
                    rss = int(line.split()[1])
                    break
        return comm, rss, utime + stime
    except Exception:  # noqa: BLE001
        return None, 0, 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', choices=['classic', 'gz'], required=True)
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--launch-epoch', type=float, default=0.0,
                    help='ros2 launch 的墙钟时刻（time.time()）')
    ap.add_argument('--pid', type=int, default=0, help='ros2 launch 的 PID（进程树根）')
    ap.add_argument('--log', default='', help='launch 日志路径（取 gz 的求交耗时）')
    ap.add_argument('--out', default='')
    ap.add_argument('--max-wait', type=float, default=300.0)
    ap.add_argument('--lidar-topic', default='/livox/lidar')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--cloud-topic', default='/cloud_registered')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan, PointCloud2
    from livox_ros_driver2.msg import CustomMsg

    rclpy.init(args=[])
    node = Node('ab_probe')

    st = {'clock': [], 'cloud': [], 'scan': [], 'pc': []}

    def clock_cb(m):
        st['clock'].append((time.time(), int(m.clock.sec) * 10**9 + int(m.clock.nanosec)))

    def cloud_cb(m):
        stamp = int(m.header.stamp.sec) * 10**9 + int(m.header.stamp.nanosec)
        st['cloud'].append((time.time(), stamp, len(m.points), m.header.frame_id))

    def scan_cb(m):
        rs = [r for r in m.ranges if r == r and r > 0.0 and r != float('inf')]
        st['scan'].append((time.time(), len(rs), min(rs) if rs else None,
                           max(rs) if rs else None, m.header.frame_id))

    def pc_cb(m):
        st['pc'].append((time.time(), m.width))

    node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data)
    node.create_subscription(CustomMsg, args.lidar_topic, cloud_cb, qos_profile_sensor_data)
    node.create_subscription(LaserScan, args.scan_topic, scan_cb, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, args.cloud_topic, pc_cb, qos_profile_sensor_data)

    # ---- 等首帧 ----
    t_start_wait = time.time()
    while not st['cloud'] and time.time() - t_start_wait < args.max_wait:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not st['cloud']:
        print('ERROR: %.0f s 内没有收到 %s 的第一帧' % (args.max_wait, args.lidar_topic))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    first_wall = st['cloud'][0][0]

    # ---- 采样窗口 ----
    proc_rows = []
    t0 = time.time()
    t_end = t0 + args.duration
    next_proc = t0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if args.pid and time.time() >= next_proc:
            next_proc = time.time() + 0.5
            rss = cpu = 0
            gz_rss = gz_cpu = 0
            n = 0
            for p in _proc_tree(args.pid):
                comm, r, c = _proc_stat(p)
                if comm is None:
                    continue
                n += 1
                rss += r
                cpu += c
                if comm in ('gzserver', 'ruby'):   # classic / gz 的仿真服务进程
                    gz_rss += r
                    gz_cpu += c
            proc_rows.append((time.time(), rss, cpu, gz_rss, gz_cpu, n))
    node.destroy_node()
    rclpy.shutdown()

    wall_span = st['cloud'][-1][0] - st['cloud'][0][0]
    sim_span = (st['cloud'][-1][1] - st['cloud'][0][1]) / 1e9
    # RTF 用 /clock 算（最干净）
    rtf = None
    if len(st['clock']) > 1:
        w = st['clock'][-1][0] - st['clock'][0][0]
        s = (st['clock'][-1][1] - st['clock'][0][1]) / 1e9
        rtf = s / w if w > 0 else None

    def q(vals, fn=statistics.median):
        return fn(vals) if vals else None

    out = {
        'backend': args.backend,
        'duration_s_wall': args.duration,
        'rtf_clock_sim_over_wall': rtf,
        'lidar_sim_hz': (len(st['cloud']) - 1) / sim_span if sim_span > 0 else None,
        'lidar_wall_hz': (len(st['cloud']) - 1) / wall_span if wall_span > 0 else None,
        'lidar_n_frames': len(st['cloud']),
        'points_median': q([c[2] for c in st['cloud']]),
        'points_min': min([c[2] for c in st['cloud']]) if st['cloud'] else None,
        'points_max': max([c[2] for c in st['cloud']]) if st['cloud'] else None,
        'lidar_frame_id': st['cloud'][0][3] if st['cloud'] else None,
        'startup_to_first_frame_s': (first_wall - args.launch_epoch) if args.launch_epoch else None,
        'scan_n_samples': len(st['scan']),
        'scan_beams_median': q([s[1] for s in st['scan']]),
        'scan_range_min': min([s[2] for s in st['scan'] if s[2]]) if st['scan'] else None,
        'scan_range_max': max([s[3] for s in st['scan'] if s[3]]) if st['scan'] else None,
        'scan_frame_id': st['scan'][0][4] if st['scan'] else None,
        'cloud_registered_n': len(st['pc']),
        'cloud_registered_hz': (len(st['pc']) - 1) / args.duration if st['pc'] else 0.0,
        'cloud_registered_width_median': q([p[1] for p in st['pc']]),
        'clock_msgs': len(st['clock']),
        'proc_samples': len(proc_rows),
    }
    if proc_rows:
        out['rss_total_median_mb'] = q([r[1] for r in proc_rows]) / 1024.0
        out['rss_total_peak_mb'] = max(r[1] for r in proc_rows) / 1024.0
        out['rss_server_median_mb'] = q([r[3] for r in proc_rows]) / 1024.0
        if len(proc_rows) > 1:
            dt = proc_rows[-1][0] - proc_rows[0][0]
            dcpu = proc_rows[-1][4] - proc_rows[0][4]          # server CPU ticks
            dct = proc_rows[-1][2] - proc_rows[0][2]           # whole tree
            hz = os.sysconf('SC_CLK_TCK')
            out['cpu_server_pct'] = 100.0 * dcpu / hz / dt
            out['cpu_tree_pct'] = 100.0 * dct / hz / dt
            out['cpu_server_s'] = dcpu / hz
            n_frames = max(1, len(st['cloud']))
            out['gzserver_cpu_ms_per_frame'] = 1000.0 * (dcpu / hz) / n_frames
    # gz 的插桩求交耗时
    if args.log and os.path.isfile(args.log):
        import re
        ansi = re.compile(r'\x1b\[[0-9;]*m')
        vals = []
        for line in open(args.log, errors='replace'):
            # launch 的日志带 ANSI 颜色码（[Msg] 前后都有），不去掉就匹配不到
            m = re.search(r'Embree average raycast: ([\d.]+) ms for (\d+) rays',
                          ansi.sub('', line))
            if m:
                vals.append(float(m.group(1)))
        if vals:
            out['raycast_ms_median'] = statistics.median(vals)
            out['raycast_ms_min'] = min(vals)
            out['raycast_ms_max'] = max(vals)
            out['raycast_samples'] = len(vals)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
