#!/usr/bin/env python3
# =============================================================================
# gz_stamp_probe.py —— 时间戳探针：把 `/livox/imu`、`/livox/lidar`、`/clock` 的
#                       header.stamp 序列逐条记下来，判「单调 / 重复 / 倒退」。
# =============================================================================
# 为什么需要它（docs/fortress_sim_backend.md §8.1 的未解问题）：
#   `small_point_lio` 在 gz 后端上持续打 `imu loop back`（preprocess.cpp:67 的
#   `imu_msg.timestamp < last_timestamp_imu`）。发布端被另一个 rclpy 探针量到
#   「严格 +10 ms 单调」，但 LIO 侧每条都倒退 ⇒ 必须**用与 LIO 完全相同的 QoS
#   订阅**再量一次，并且量「最早的那几十条」（环回一旦发生会一直发生）。
#
# 三个订阅（同一进程、同一时刻）：
#   ① /livox/imu  —— **QoS = SensorDataQoS（best_effort, keep_last 5）**，与
#      small_point_lio 逐字相同（small_point_lio_node.cpp 的 create_subscription）；
#   ② /livox/imu  —— QoS = reliable keep_last 10，与 FAST-LIO 的默认订阅相同；
#   ③ /livox/lidar（CustomMsg）或 /livox/lidar/pointcloud，SensorDataQoS；
#   ④ /clock —— 仿真钟（ros_gz_bridge 或 gazebo_ros）。
#
# 输出（JSON）：
#   {topic: {qos, n, first20:[[stamp_ns, recv_ns], ...],
#            n_back, n_dup, n_skip(>2*median), max_back_ns, max_fwd_ns,
#            dt_ns_median, dt_ns_min, dt_ns_max, ns_unique}}
#
# 用法（在 source 过 ROS + 本仓 install 之后）：
#   python3 tools/scripts/gzsim/gz_stamp_probe.py --duration 10 --out /tmp/stamps.json
#   python3 tools/scripts/gzsim/gz_stamp_probe.py --duration 10 --no-imu-reliable
# =============================================================================
import argparse
import json
import statistics
import sys
import time


def _stamp_ns(msg):
    # rosgraph_msgs/Clock 没有 header，直接是 clock 字段
    st = getattr(msg, 'clock', None)
    if st is None:
        st = msg.header.stamp
    return int(st.sec) * 1_000_000_000 + int(st.nanosec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=10.0, help='采样墙钟秒数')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--lidar-topic', default='/livox/lidar')
    ap.add_argument('--lidar-type', default='custom', choices=['custom', 'pc2'],
                    help="custom = livox_ros_driver2/msg/CustomMsg；pc2 = PointCloud2")
    ap.add_argument('--out', default='')
    ap.add_argument('--no-imu-reliable', action='store_true')
    ap.add_argument('--warmup', type=float, default=0.0,
                    help='先等这么多秒再开始计数（仍会记录第一批）')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[])
    node = Node('gz_stamp_probe')

    rec = {
        'imu_sensor_qos': {'qos': 'best_effort keep_last 5 (== SensorDataQoS, 与 small_point_lio 相同)',
                           'samples': []},
        'imu_reliable': {'qos': 'reliable keep_last 10 (== FAST-LIO 默认)',
                         'samples': []},
        'lidar': {'qos': 'best_effort (SensorDataQoS)', 'samples': []},
        'clock': {'qos': 'best_effort (SensorDataQoS)', 'samples': []},
    }

    def mk(name, ss_key='samples'):
        def cb(msg, _name=name):
            try:
                s = _stamp_ns(msg)
            except Exception:
                return
            rec[_name][ss_key].append([s, time.monotonic_ns()])
        return cb

    node.create_subscription(Imu, args.imu_topic, mk('imu_sensor_qos'), qos_profile_sensor_data)
    if not args.no_imu_reliable:
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(Imu, args.imu_topic, mk('imu_reliable'), rel)
    if args.lidar_type == 'custom':
        try:
            from livox_ros_driver2.msg import CustomMsg
        except Exception as exc:  # noqa: BLE001
            print('没有 livox_ros_driver2/CustomMsg：%s' % exc, file=sys.stderr)
            CustomMsg = None
        if CustomMsg is not None:
            node.create_subscription(CustomMsg, args.lidar_topic, mk('lidar'),
                                     qos_profile_sensor_data)
    else:
        node.create_subscription(PointCloud2, args.lidar_topic, mk('lidar'),
                                 qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', mk('clock'), qos_profile_sensor_data)

    # 采样窗口：warmup 后 duration 秒（warmup 期间来的样本照样收着，只是不打进 summary）
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.05)
    t_start = time.monotonic()
    t_end = t_start + args.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    out = {'imu_topic': args.imu_topic, 'lidar_topic': args.lidar_topic,
           'duration_s': args.duration, 'warmup_s': args.warmup,
           't_wall_start': time.time()}
    for name, r in rec.items():
        ss = r['samples']
        d = {'qos': r['qos'], 'n': len(ss)}
        if len(ss) == 0:
            out[name] = d
            continue
        stamps = [s[0] for s in ss]
        n_back = 0
        n_dup = 0
        n_tiny = 0
        backs = []
        fwds = []
        dts = []
        seen = set()
        for i in range(1, len(stamps)):
            dt = stamps[i] - stamps[i - 1]
            dts.append(dt)
            if dt < 0:
                n_back += 1
                backs.append((i, stamps[i - 1], stamps[i]))
            elif dt == 0:
                n_dup += 1
            elif dt < 1_000_000:          # <1 ms 的"近乎重复"
                n_tiny += 1
            else:
                fwds.append(dt)
            seen.add(stamps[i])
        seen.add(stamps[0])
        d.update({
            'n_back': n_back,
            'n_dup': n_dup,
            'n_dt_lt_1ms': n_tiny,
            'max_back_ns': min(dts) if dts else None,
            'dt_ns_min': min(dts) if dts else None,
            'dt_ns_median': statistics.median(dts) if dts else None,
            'dt_ns_max': max(dts) if dts else None,
            'n_unique_stamps': len(seen),
            'stamp_span_ns': stamps[-1] - stamps[0],
            'first20': [[s, w] for s, w in ss[:20]],
            'last5': [[s, w] for s, w in ss[-5:]],
            'back_events': backs[:20],
        })
        out[name] = d

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1, ensure_ascii=False)

    # 人类可读摘要（一行一个流）
    for name in ('imu_sensor_qos', 'imu_reliable', 'lidar', 'clock'):
        d = out.get(name, {})
        if not d.get('n'):
            print('%-16s n=0（没收到）' % name)
            continue
        print('%-16s n=%-5d back=%-4d dup=%-4d dt_med=%s dt_min=%s dt_max=%s uniq=%d'
              % (name, d['n'], d['n_back'], d['n_dup'],
                 _fmt(d['dt_ns_median']), _fmt(d['dt_ns_min']), _fmt(d['dt_ns_max']),
                 d['n_unique_stamps']))
        if d['n_back']:
            print('   ⚠ 倒退事件（前 5）：' + ', '.join(
                '%d: %d → %d (Δ=%+d ns)' % (i, a, b, b - a) for i, a, b in d['back_events'][:5]))
        print('   first10 stamps(ns) = ' + ', '.join(str(s[0]) for s in d['first20'][:10]))
    node.destroy_node()
    rclpy.shutdown()


def _fmt(v):
    if v is None:
        return 'None'
    if abs(v) < 10_000_000:
        return '%.3fms' % (v / 1e6)
    return '%.6fs' % (v / 1e9)


if __name__ == '__main__':
    main()
