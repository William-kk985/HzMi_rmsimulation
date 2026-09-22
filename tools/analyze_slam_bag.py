#!/usr/bin/env python3
"""离线分析 SLAM 相关 bag：时间戳体检 / IMU 启动质量 / 两条里程计是否一致 / map→odom 是否在甩。

为什么要它：`map→odom` 甩动、"地图跟着车转"这类问题，在线用 tf2_echo / topic hz 只能看零散切片，
而真正要回答的是四类问题，全都能从一段 bag 里离线算出来、且可复现：

  ① 三路（scan / odom / imu）**时间戳是否同轴**：有没有 6213559xx（墙钟 .NET ticks 被当成 ns）
     这类异常、有没有时间回退（乱序）；
  ② **IMU 启动质量**：前若干帧的 linear_acceleration 是否为 (0,0,0)（cartographer 的
     `imu_tracker.cc:67` 拿**第一帧**当重力基准，一帧零值就 abort —— FAST-LIO 多帧求均值所以没事）；
  ③ **两条里程计是否一致**：`/odom`（LIO）与 `/odom_ground_truth`（底盘真值）的角速度与**速率抖动**
     —— 若不一致，cartographer 的先验与 RViz 显示的机器人就会互相打架；
  ④ **map→odom 到底在怎么动**：从 `/tf` 里抽出来算"速率抖动/峰峰值/跳变"（等价于
     monitor_map_odom.py，但离线可复算）。

⚠️ 判读要点（工具已按这条实现）：机器人**在转**的时候，逐样本 |Δyaw| 大是正常的（那就是角速度）。
   所以"是否在甩"要看两个量：
     · 速率中位（=平均角速度，正常应接近真实转速）
     · **速率抖动 P95**（= 角速度的波动，正常应接近 0；这才是"甩"的度量）

用法:
  python3 tools/analyze_slam_bag.py --bag .tmp_bags/flail
  python3 tools/analyze_slam_bag.py --bag .tmp_bags/flail --imu-warmup 50 --csv /tmp/yaw.csv

约定：只读分析，不改任何配置（见 docs/sim_real_contract.md 的分类规则）。
"""

import argparse
import math
import os
import statistics
import sys

BOGUS_T = 1e6     # 时间戳大于这个值 = 不是仿真钟（例如 6213559xx 这种墙钟 .NET ticks 值）


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def yaw_stats(pairs):
    """pairs: [(t, yaw_deg)] → 角速度与"速率抖动"统计（按时间排序，剔除异常时间戳）。

    角速度用**逐区间的中位数**算（不是首尾差）：首尾差会被 360° 回绕毁掉，
    而机器人真的在转时"逐样本 |Δyaw| 大"是正常的 —— 真正要看的"甩"是**速率的波动**。
    """
    pairs = sorted((t, y) for t, y in pairs if t <= BOGUS_T)
    if len(pairs) < 3:
        return None
    ys = [y for _, y in pairs]
    ts = [t for t, _ in pairs]
    deltas = [abs(wrap180(ys[i] - ys[i - 1])) for i in range(1, len(ys))]
    rates = [wrap180(ys[i] - ys[i - 1]) / (ts[i] - ts[i - 1])
             for i in range(1, len(ys)) if ts[i] > ts[i - 1]]
    rate_med = statistics.median(rates) if rates else float("nan")
    dev = sorted(abs(r - rate_med) for r in rates)
    return {
        "n": len(ys), "pp": max(ys) - min(ys), "std": statistics.pstdev(ys),
        "rate": rate_med,
        "jit": dev[int(0.95 * (len(dev) - 1))] if dev else float("nan"),
        "maxd": max(deltas) if deltas else 0.0,
        "jumps": sum(1 for x in deltas if x > 0.5),
    }


def fmt(name, s):
    if not s:
        return f"  {name:34s} 样本不足"
    return (f"  {name:34s} n={s['n']:<6d} 角速度中位={s['rate']:+7.2f}°/s  "
            f"速率抖动P95={s['jit']:6.3f}°/s  峰峰值={s['pp']:7.2f}°  "
            f"最大单步={s['maxd']:6.2f}° 跳变>0.5°={s['jumps']}")


def stamp_sec(msg):
    st = msg.header.stamp
    return st.sec + st.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description="离线分析 SLAM bag（时间戳/IMU/两条里程计/map→odom）")
    ap.add_argument("--bag", required=True, help="bag 目录（ros2 bag record -o 的那个目录）")
    ap.add_argument("--imu-warmup", type=int, default=30, help="打印 IMU 前多少帧（默认 30）")
    ap.add_argument("--csv", default="", help="把两条里程计的 yaw 序列写到 CSV")
    args = ap.parse_args()

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print(f"需要 ROS 2 环境（source /opt/ros/humble/setup.bash）：{exc}")
        return 2
    if not os.path.exists(args.bag):
        print(f"找不到 bag：{args.bag}")
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print("== bag 内容 ==")
    for n, t in sorted(topics.items()):
        print(f"  {n:28s} {t}")

    WANT = ("sensor_msgs/msg/Imu", "nav_msgs/msg/Odometry", "sensor_msgs/msg/LaserScan",
            "tf2_msgs/msg/TFMessage", "rosgraph_msgs/msg/Clock")
    stamps = {}                     # topic -> [t]
    imu_acc, imu_t = [], []
    odom_yaw, gt_yaw = [], []       # [(t, yaw)]
    tf_pairs = {}                   # (parent, child) -> [(t, yaw)]

    while reader.has_next():
        topic, data, _wall = reader.read_next()
        mtype = topics.get(topic)
        if mtype not in WANT:
            continue
        msg = deserialize_message(data, get_message(mtype))
        if mtype == "rosgraph_msgs/msg/Clock":
            stamps.setdefault(topic, []).append(msg.clock.sec + msg.clock.nanosec * 1e-9)
            continue
        if mtype == "tf2_msgs/msg/TFMessage":
            for tr in msg.transforms:
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                stamps.setdefault(f"/tf {tr.header.frame_id}->{tr.child_frame_id}", []).append(t)
                tf_pairs.setdefault((tr.header.frame_id, tr.child_frame_id), []).append(
                    (t, yaw_of(tr.transform.rotation)))
            continue
        t = stamp_sec(msg)
        stamps.setdefault(topic, []).append(t)
        if mtype == "sensor_msgs/msg/Imu":
            a = msg.linear_acceleration
            imu_acc.append((a.x, a.y, a.z)); imu_t.append(t)
        elif mtype == "nav_msgs/msg/Odometry":
            y = yaw_of(msg.pose.pose.orientation)
            (gt_yaw if topic.endswith("odom_ground_truth") else odom_yaw).append((t, y))

    # ------------------------------------------------------------ ① 时间戳
    print("\n== ① 各话题：条数 / 时间范围 / 中位率 / 乱序 / 异常时间戳 ==")
    for topic in sorted(stamps):
        ts = sorted(stamps[topic])
        n = len(ts)
        bogus = sum(1 for t in ts if t > BOGUS_T)
        good = [t for t in ts if t <= BOGUS_T]
        dts = sorted(b - a for a, b in zip(good[:-1], good[1:]) if b > a)
        back = n - 1 - len(dts)
        rate = 1.0 / statistics.median(dts) if dts else float("nan")
        rng = f"[{good[0]:9.3f} .. {good[-1]:9.3f}]" if good else "[  --  ]"
        flag = ""
        if bogus:
            flag += f"  ← ⚠️ {bogus} 条时间戳 >1e6（不是仿真钟，例如 6213559xx）"
        if back:
            flag += f"  ← ⚠️ 时间回退 {back} 次（乱序）"
        print(f"  {topic:30s} n={n:<7d} {rng} {rate:7.2f} Hz{flag}")
    print("  说明：这里的率用**相邻间隔的中位数**算，避免个别异常戳把平均值带偏。")

    # ------------------------------------------------------------ ② IMU
    print("\n== ② IMU 启动质量（cartographer 拿第一帧当重力基准）==")
    if imu_acc:
        zeros = [i for i, a in enumerate(imu_acc[:args.imu_warmup])
                 if abs(a[0]) < 1e-6 and abs(a[1]) < 1e-6 and abs(a[2]) < 1e-6]
        print(f"  总帧数 {len(imu_acc)}；前 {args.imu_warmup} 帧里零加速度帧 = {len(zeros)}"
              + (f"  ← ⚠️ 索引 {zeros[:10]}（会让 cartographer 的 imu_tracker 直接 abort）"
                 if zeros else "  ✓"))
        for i, a in enumerate(imu_acc[:args.imu_warmup]):
            print(f"    #{i:<3d} t={imu_t[i]:9.3f}  acc=({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})"
                  + ("   ← 零值" if i in zeros else ""))
        norms = [math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2) for a in imu_acc]
        print(f"  全段 |acc| 均值 {statistics.mean(norms):.3f} m/s²（静止应 ≈9.8）")
    else:
        print("  bag 里没有 /livox/imu")

    # ------------------------------------------------------------ ③ 两条里程计
    print("\n== ③ 两条里程计 yaw（角速度应接近、抖动应接近 0；不一致 ⇒ 先验与显示互相打架）==")
    s_o, s_g = yaw_stats(odom_yaw), yaw_stats(gt_yaw)
    print(fmt("/odom（LIO）", s_o))
    print(fmt("/odom_ground_truth（真值）", s_g))
    if s_o and s_g:
        print(f"  角速度差 = {abs(s_o['rate'] - s_g['rate']):.3f} °/s"
              f"（大 ⇒ 两条里程计不同源）")
        worse = "/odom（LIO）" if s_o["jit"] > s_g["jit"] else "/odom_ground_truth"
        print(f"  抖动更大的是 {worse}（P95 {max(s_o['jit'], s_g['jit']):.3f}°）")

    # ------------------------------------------------------------ ④ /tf
    print("\n== ④ /tf 各边 yaw（重点 map->odom）==")
    keys = [k for k in tf_pairs if k[0] in ("map", "odom", "camera_init")]
    for key in sorted(keys, key=str):
        print(f"  {key[0]} -> {key[1]}:")
        print(fmt("   ", yaw_stats(tf_pairs[key])))
    if ("map", "odom") not in tf_pairs:
        print("  ⚠️ bag 里没有 map->odom（没录 /tf，或 cartographer 没起来）")

    if args.csv and odom_yaw:
        with open(args.csv, "w") as fh:
            fh.write("t_odom,yaw_odom_deg,t_gt,yaw_gt_deg\n")
            o = sorted(odom_yaw); g = sorted(gt_yaw)
            for i in range(max(len(o), len(g))):
                a = f"{o[i][0]:.3f},{o[i][1]:.3f}" if i < len(o) else ","
                b = f"{g[i][0]:.3f},{g[i][1]:.3f}" if i < len(g) else ","
                fh.write(f"{a},{b}\n")
        print(f"\nCSV 已写入 {args.csv}")

    print("\n提示：把这段输出整段贴回来即可；本工具只读，不改任何配置。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
