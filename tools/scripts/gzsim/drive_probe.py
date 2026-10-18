#!/usr/bin/env python3
# =============================================================================
# drive_probe.py —— 量 gz 底盘的「命令 → 实际运动」权威（deg/s 与 m/s）
# =============================================================================
# 协议（与 Classic 的 §M 台架同形，只换命令来源）：
#   阶段 A：/cmd_vel_chassis = (vx=0, wz=+W) 持续 T 仿真秒  → 量 Δyaw / Δt = deg/s
#   阶段 B：/cmd_vel_chassis = (vx=+V, wz=0) 持续 T 仿真秒  → 量 Δs   / Δt = m/s
#   阶段 C：零命令 →
# 真值来源：`/odom_ground_truth`（gz 侧 = 驱动插件按模型位姿发的 nav_msgs/Odometry，
#           与 Classic 的 planar_move 同名同语义）。时间一律用**仿真时间**（消息戳），
#           所以 RTF<1 时"每秒"仍可比。
# 同时记录 `/livox/imu` 的角速度 z（验证驱动把命令写进了 IMU 的 AngularVelocity，
# 否则 LIO 会认为车没转）。
#
# 用法：
#   python3 tools/scripts/gzsim/drive_probe.py --wz 0.5 --vx 0.3 --duration 4 \
#       --out /tmp/drive.json
# =============================================================================
import argparse
import json
import math
import time


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cmd-topic', default='/cmd_vel_chassis')
    ap.add_argument('--odom-topic', default='/odom_ground_truth')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--lio-odom-topic', default='',
                    help='可选：LIO 的 /odom（证明 LIO 也看到车在转）。空 = 不量')
    ap.add_argument('--wz', type=float, default=0.5, help='阶段 A 的命令角速度 rad/s')
    ap.add_argument('--vx', type=float, default=0.3, help='阶段 B 的命令线速度 m/s')
    ap.add_argument('--duration', type=float, default=4.0, help='每阶段仿真秒数')
    ap.add_argument('--settle', type=float, default=1.0, help='开始前先等多久（墙钟秒）')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu

    rclpy.init(args=[])
    node = Node('drive_probe')
    pub = node.create_publisher(Twist, args.cmd_topic, 10)

    state = {'odom': None, 'imu_wz': [], 'n_imu': 0}

    def odom_cb(m):
        st = m.header.stamp
        ns = int(st.sec) * 1_000_000_000 + int(st.nanosec)
        p = m.pose.pose
        state['odom'] = {'t_ns': ns, 'x': p.position.x, 'y': p.position.y,
                         'yaw': yaw_of(p.orientation),
                         'vx': m.twist.twist.linear.x, 'wz': m.twist.twist.angular.z,
                         'frame': m.header.frame_id, 'child': m.child_frame_id}

    def imu_cb(m):
        state['n_imu'] += 1
        state['imu_wz'].append((time.monotonic(), m.angular_velocity.z))

    lio = {'yaw0': None, 'yaw1': None, 'n': 0}

    def lio_cb(m):
        y = yaw_of(m.pose.pose.orientation)
        if lio['yaw0'] is None:
            lio['yaw0'] = y
        lio['yaw1'] = y
        lio['n'] += 1

    node.create_subscription(Odometry, args.odom_topic, odom_cb, 50)
    node.create_subscription(Imu, args.imu_topic, imu_cb, 200)
    if args.lio_odom_topic:
        node.create_subscription(Odometry, args.lio_odom_topic, lio_cb, 50)

    def pump_sim(seconds, vx=0.0, wz=0.0):
        """按**仿真时间**推进 seconds：用 /odom_ground_truth 的戳做尺。"""
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        if state['odom'] is None:
            return
        t0 = state['odom']['t_ns']
        t_end = time.monotonic() + 180.0
        while time.monotonic() < t_end:
            pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.005)
            if state['odom'] and (state['odom']['t_ns'] - t0) >= seconds * 1e9:
                return

    # 先等起来
    t_end = time.monotonic() + args.settle
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    res = {'cmd_topic': args.cmd_topic, 'odom_topic': args.odom_topic,
           'wz_cmd': args.wz, 'vx_cmd': args.vx, 'duration_sim_s': args.duration,
           'odom_available': state['odom'] is not None}
    if state['odom'] is None:
        res['error'] = '没有 %s 数据（驱动插件没起？bridge 没起？）' % args.odom_topic
        _emit(res, args.out)
        return

    res['odom_frame'] = state['odom']['frame']
    res['odom_child'] = state['odom']['child']

    # ---- 阶段 A：纯自转 ----
    rclpy.spin_once(node, timeout_sec=0.05)
    yaw0 = state['odom']['yaw']
    start_ns = state['odom']['t_ns']
    imu0 = len(state['imu_wz'])
    pump_sim(args.duration, vx=0.0, wz=args.wz)
    end_ns = state['odom']['t_ns']
    yaw1 = state['odom']['yaw']
    dsim = (end_ns - start_ns) / 1e9
    dyaw = math.degrees(yaw1 - yaw0)
    lio_yaw0 = lio['yaw0']
    res['A_rotate'] = {
        'sim_s': dsim,
        'deg': dyaw,
        'deg_per_sim_s': dyaw / dsim if dsim > 0 else None,
        'cmd_deg_per_s': math.degrees(args.wz),
        'ratio': (dyaw / dsim) / math.degrees(args.wz) if dsim > 0 and args.wz else None,
        'imu_wz_mean_during': _mean([w for _, w in state['imu_wz'][imu0:]]),
        'imu_n_during': len(state['imu_wz']) - imu0,
        'lio_odom_msgs': lio['n'],
        'lio_dyaw_deg': (math.degrees(lio['yaw1'] - lio_yaw0)
                         if (lio_yaw0 is not None and lio['yaw1'] is not None) else None),
    }

    # ---- 阶段 B：纯直行 ----
    pump_sim(0.5, vx=0.0, wz=0.0)     # 停一下
    rclpy.spin_once(node, timeout_sec=0.05)
    x0, y0 = state['odom']['x'], state['odom']['y']
    start_ns = state['odom']['t_ns']          # ★ B 阶段自己的起点（原来漏了这行）
    pump_sim(args.duration, vx=args.vx, wz=0.0)
    end_ns = state['odom']['t_ns']
    dsim = (end_ns - start_ns) / 1e9
    ds = math.hypot(state['odom']['x'] - x0, state['odom']['y'] - y0)
    res['B_drive'] = {
        'sim_s': dsim,
        'm': ds,
        'm_per_sim_s': ds / dsim if dsim > 0 else None,
        'cmd_m_per_s': args.vx,
        'ratio': (ds / dsim) / args.vx if dsim > 0 and args.vx else None,
    }

    # ---- 阶段 C：停 ----
    pump_sim(0.5, vx=0.0, wz=0.0)
    _emit(res, args.out)


def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else None


def _emit(res, out):
    for k in ('A_rotate', 'B_drive'):
        d = res.get(k)
        if not d:
            continue
        print('%s: sim=%.3fs' % (k, d['sim_s']), end=' ')
        if k == 'A_rotate':
            print('Δyaw=%.2f° ⇒ %.2f °/s (命令 %.2f °/s，比率 %.3f)；IMU wz 均值=%s (%d 条)'
                  % (d['deg'], d['deg_per_sim_s'] or 0, d['cmd_deg_per_s'],
                     d['ratio'] or 0,
                     ('%.4f' % d['imu_wz_mean_during']) if d['imu_wz_mean_during'] is not None else 'n/a',
                     d['imu_n_during']))
        else:
            print('Δs=%.4f m ⇒ %.4f m/s (命令 %.4f m/s，比率 %.3f)'
                  % (d['m'], d['m_per_sim_s'] or 0, d['cmd_m_per_s'], d['ratio'] or 0))
    if res.get('error'):
        print('ERROR: ' + res['error'])
    if out:
        with open(out, 'w') as f:
            json.dump(res, f, indent=1, ensure_ascii=False)


if __name__ == '__main__':
    main()
