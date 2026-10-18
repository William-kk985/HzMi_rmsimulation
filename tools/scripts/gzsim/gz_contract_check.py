#!/usr/bin/env python3
# =============================================================================
# gz_contract_check.py —— TF 单发布者契约 + gz 通道独占性检查（`sim_backend:=gz`）
# =============================================================================
# 回答两个问题，并把证据落盘成 JSON：
#
# ① **每个 TF 边到底有几个发布者？**
#    ROS 2 的 `TFMessage` 不带 authority（`view_frames` 里所有 broadcaster 都是
#    `default_authority`），所以"按发布者计数"不能只靠订阅内容。本脚本用三路证据：
#      (a) `/tf` 与 `/tf_static` 的**位点（endpoint）清单**：`ros2 topic info -v` 里
#          每个 PUBLISHER 的 node 名 + GID ⇒ 每个话题上"有多少个发布者进程/节点"；
#      (b) 逐边的**消息率 + 同 stamp 冲突检测**：两个节点同时发同一条边时，该边的
#          到达率≈两者之和，并且同一 stamp 会出现**两个不同的变换值**（本脚本逐条
#          比较，记 `stamp_conflicts`）；
#      (c) `tf2_monitor` 的逐帧统计（它把同一帧的所有来源合成一行：率翻倍就会被看到）。
#    结论写法：`edges[<parent> -> <child>] = {n, rate_hz, publishers_expected, ...}`。
#
# ② **gz 通道是不是只有我们自己在发？**
#    跑 `ign topic -i -t /mid360/imu_gz`：`Publishers:` 只应该有 1 个（我们的传感器
#    模型）。多于 1 个 = 同机另一个 Fortress 世界和我们在**同一个 gz partition** 上
#    ⇒ `/livox/imu` 的戳会变成两个仿真钟交错 ⇒ small_point_lio 每条都报
#    `imu loop back`（实测机理见 docs/fortress_sim_backend.md §8.2）。
#    同时量 `/livox/imu` 的戳单调性（best_effort QoS，与 small_point_lio 相同）。
#
# 用法（先 source 本仓 install + gz 后端前缀；ROS_DOMAIN_ID 与目标栈一致）：
#   python3 tools/scripts/gzsim/gz_contract_check.py --duration 12 --out /tmp/contract.json
#   python3 tools/scripts/gzsim/gz_contract_check.py --skip-gz-side   # Classic 档也用
# =============================================================================
import argparse
import json
import subprocess
import sys
import time


def run(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except Exception as exc:  # noqa: BLE001
        return 'ERROR: %s' % exc


def parse_topic_info(text):
    """从 `ros2 topic info -v` 的文本里抽出发布者/订阅者清单。"""
    pubs, subs = [], []
    cur = None
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('Node name:'):
            cur = {'node': s.split(':', 1)[1].strip()}
        elif s.startswith('Endpoint type:'):
            kind = s.split(':', 1)[1].strip().upper()
            if cur is not None:
                cur['kind'] = kind
        elif s.startswith('GID:') and cur is not None:
            cur['gid'] = s.split(':', 1)[1].strip()
        elif s.startswith('Reliability:') and cur is not None:
            cur['reliability'] = s.split(':', 1)[1].strip()
        elif s.startswith('History (Depth):') and cur is not None:
            cur['depth'] = s.split(':', 1)[1].strip()
        elif s == '' and cur is not None:
            (pubs if cur.get('kind', '').startswith('PUB') else subs).append(cur)
            cur = None
    if cur is not None:
        (pubs if cur.get('kind', '').startswith('PUB') else subs).append(cur)
    return pubs, subs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=12.0)
    ap.add_argument('--out', default='')
    ap.add_argument('--skip-gz-side', action='store_true',
                    help='跳过 gz transport 独占性检查（Classic 档没有 gz 侧）')
    ap.add_argument('--skip-tf2-monitor', action='store_true')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--expect', action='append', default=[],
                    help='必须存在的边，形如 odom:base_link（可重复）')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                           ReliabilityPolicy, HistoryPolicy)
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    out = {
        'duration_s': args.duration,
        't_wall': time.time(),
        'topics': {},
        'edges': {},
        'checks': {},
    }

    # ---- ①a 位点清单 ------------------------------------------------------
    for topic in ('/tf', '/tf_static', '/odom', '/cloud_registered', '/scan',
                  '/livox/lidar', '/livox/lidar/pointcloud', args.imu_topic):
        txt = run(['ros2', 'topic', 'info', '-v', topic], timeout=15)
        pubs, subs = parse_topic_info(txt)
        out['topics'][topic] = {
            'n_publishers': len(pubs),
            'publishers': pubs,
            'n_subscriptions': len(subs),
            'subscriptions': subs,
        }

    rclpy.init(args=[])
    node = Node('gz_contract_check')

    # ---- ①b 逐边统计 ------------------------------------------------------
    edges = {}
    stamp_vals = {}

    def note(parent, child, stamp_ns, payload):
        key = '%s -> %s' % (parent, child)
        d = edges.setdefault(key, {'n': 0, 'static': False, 'first': None, 'last': None,
                                   'stamp_conflicts': 0})
        d['n'] += 1
        d['last'] = stamp_ns
        if d['first'] is None:
            d['first'] = stamp_ns
        sig = stamp_vals.setdefault(key, {})
        prev = sig.get(stamp_ns)
        # 同一 (边, stamp) 出现两次且内容不同 ⇒ 两个发布者抢同一条边
        if prev is not None and prev != payload:
            d['stamp_conflicts'] += 1
        sig[stamp_ns] = payload
        if len(sig) > 200000:
            sig.clear()

    def cb(msg, static=False):
        for tr in msg.transforms:
            st = tr.header.stamp
            ns = int(st.sec) * 1_000_000_000 + int(st.nanosec)
            payload = (round(tr.transform.translation.x, 9), round(tr.transform.translation.y, 9),
                       round(tr.transform.translation.z, 9), round(tr.transform.rotation.x, 9),
                       round(tr.transform.rotation.y, 9), round(tr.transform.rotation.z, 9),
                       round(tr.transform.rotation.w, 9))
            note(tr.header.frame_id or '', tr.child_frame_id or '', ns, payload)
            edges['%s -> %s' % (tr.header.frame_id or '', tr.child_frame_id or '')]['static'] = static

    node.create_subscription(TFMessage, '/tf', lambda m: cb(m), 200)
    stat = QoSProfile(depth=200, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(TFMessage, '/tf_static', lambda m: cb(m, True), stat)

    # ---- ② IMU 戳单调性（与 small_point_lio 同一个 QoS）-------------------
    imu = {'n': 0, 'n_back': 0, 'dt_med': None, 'first': None, 'last': None, 'back': []}
    dts = []

    def imu_cb(m):
        st = m.header.stamp
        ns = int(st.sec) * 1_000_000_000 + int(st.nanosec)
        if imu['first'] is None:
            imu['first'] = ns
        elif imu['last'] is not None:
            dt = ns - imu['last']
            dts.append(dt)
            if dt < 0:
                imu['n_back'] += 1
                if len(imu['back']) < 10:
                    imu['back'].append([imu['last'], ns])
        imu['n'] += 1
        imu['last'] = ns

    node.create_subscription(Imu, args.imu_topic, imu_cb, qos_profile_sensor_data)

    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    span = {}
    for k, d in edges.items():
        if d['first'] is not None and d['last'] is not None and d['last'] > d['first']:
            d['rate_hz'] = (d['n'] - 1) / ((d['last'] - d['first']) / 1e9)
        else:
            d['rate_hz'] = None
        d.pop('first', None)
        d.pop('last', None)
    out['edges'] = edges
    if dts:
        sd = sorted(dts)
        imu['dt_med_ns'] = sd[len(sd) // 2]
        imu['dt_min_ns'] = sd[0]
        imu['dt_max_ns'] = sd[-1]
    out['imu'] = imu

    # ---- ② gz 侧独占性 ----------------------------------------------------
    if not args.skip_gz_side:
        gz = {}
        gz['imu_gz_info'] = run(['ign', 'topic', '-i', '-t', '/mid360/imu_gz'], timeout=15)
        gz['imu_gz_n_publishers'] = gz['imu_gz_info'].count('tcp://') + \
            gz['imu_gz_info'].count('ipc://') + gz['imu_gz_info'].count('udp://')
        gz['clock_info'] = run(['ign', 'topic', '-i', '-t', '/clock'], timeout=15)
        gz['clock_n_publishers'] = gz['clock_info'].count('tcp://') + \
            gz['clock_info'].count('ipc://') + gz['clock_info'].count('udp://')
        out['gz'] = gz

    # ---- ①c tf2_monitor 第二意见 ------------------------------------------
    if not args.skip_tf2_monitor:
        out['tf2_monitor'] = run(['ros2', 'run', 'tf2_ros', 'tf2_monitor'], timeout=25)

    # ---- 判决 -------------------------------------------------------------
    chk = out['checks']
    chk['tf_publishers'] = out['topics'].get('/tf', {}).get('n_publishers')
    chk['tf_static_publishers'] = out['topics'].get('/tf_static', {}).get('n_publishers')
    chk['odom_base_link_edges'] = sum(1 for k in edges if k == 'odom -> base_link')
    chk['map_odom_edges'] = sum(1 for k in edges if k == 'map -> odom')
    chk['odom_base_link_stamp_conflicts'] = edges.get('odom -> base_link', {}).get('stamp_conflicts')
    chk['map_odom_stamp_conflicts'] = edges.get('map -> odom', {}).get('stamp_conflicts')
    chk['imu_monotonic'] = (imu['n'] > 0 and imu['n_back'] == 0)
    if not args.skip_gz_side:
        chk['gz_imu_gz_single_publisher'] = out['gz']['imu_gz_n_publishers'] == 1
        chk['gz_clock_single_publisher'] = out['gz']['clock_n_publishers'] <= 1
    for spec in args.expect:
        parent, _, child = spec.partition(':')
        key = '%s -> %s' % (parent, child)
        chk['edge_%s_present' % spec.replace(':', '_')] = key in edges

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(out, f, indent=1, ensure_ascii=False)

    # ---- 人类可读 ---------------------------------------------------------
    print('== TF 边（n / rate / 同 stamp 冲突 / static） ==')
    for k in sorted(edges):
        d = edges[k]
        print('  %-34s n=%-6d rate=%-8s conflicts=%-3d static=%s'
              % (k, d['n'], ('%.2f Hz' % d['rate_hz']) if d['rate_hz'] else 'n/a',
                 d['stamp_conflicts'], d['static']))
    print('== 话题发布者数 ==')
    for t, d in out['topics'].items():
        names = ','.join(p.get('node', '?') for p in d['publishers']) or '-'
        print('  %-28s publishers=%d [%s]  subs=%d'
              % (t, d['n_publishers'], names, d['n_subscriptions']))
    print('== IMU（QoS 与 small_point_lio 相同） ==')
    print('  n=%d  back=%d  dt_med=%s ns  dt_min=%s  dt_max=%s'
          % (imu['n'], imu['n_back'], imu.get('dt_med_ns'), imu.get('dt_min_ns'),
             imu.get('dt_max_ns')))
    if imu['back']:
        print('  ⚠ 倒退样例：%s' % imu['back'][:3])
    if not args.skip_gz_side:
        print('== gz transport ==')
        print('  /mid360/imu_gz publishers = %d（必须 =1）' % out['gz']['imu_gz_n_publishers'])
        print('  /clock        publishers = %d（必须 <=1）' % out['gz']['clock_n_publishers'])
    print('== 判决 ==')
    bad = [k for k, v in chk.items() if v is False]
    for k, v in chk.items():
        print('  %-38s %s' % (k, v))
    if bad:
        print('  ✗ 未通过：%s' % ', '.join(bad))
    else:
        print('  ✅ 全通过')


if __name__ == '__main__':
    main()
