#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_node_alone_check.py —— **不开 Gazebo / 不开 nav2**，单独验证一个 LIO 节点"喂得进去"。

为什么需要它（问题背景）：
  · 全栈验证（bringup_sim.launch.py）要起 Gazebo + nav2，几十个节点，出了"没有 /odom"很难判是
    "LIO 算法不收敛"还是"话题/QoS/时间戳/字段 不匹配"；
  · `lio:=small_point_lio` 这个新槽位最需要先证明的恰恰是**输入契约**这一层：
    话题名 / 消息类型 / QoS / header.frame_id / **逐点时间戳**。
    其中"逐点时间戳"是 Point-LIO 家族的命门（small_point_lio 的 CustomMsg 适配器用
    `timebase + offset_time` 当**绝对时间**，而本仓仿真插件只填 header.stamp、timebase 恒为 0）。

本工具做什么：
  1. 从**已录好的 bag**里流式重放两组数据（默认 `.tmp_bags/ret4`）：
       · `/livox/imu`                  → sensor_msgs/Imu，100 Hz
       · `/livox/lidar/pointcloud`     → **转成 livox_ros_driver2/CustomMsg**，10 Hz
     ⇒ 用的是**我们真实世界的真实回波**（不是人造几何），只是把时间轴平移到"现在"。
  2. 自己补一条 `/tf_static`（base_link→livox_frame / base_link→imu_link），
     因为 small_point_lio **强依赖** `lookupTransform(livox_frame, "base_link")`：
     查不到就丢帧、连 /Odometry 都不发（small_point_lio_node.cpp:63-69 直接 return）。
     真跑时这条由 robot_state_publisher 从 URDF 提供。
  3. 同时当**判读端**：订阅 /odom、/tf、/cloud_registered，最后打一张
     「有没有出数 / 出数频率 / 帧名 / TF 边 / 轨迹长度」的表，并给 PASS/FAIL。

典型用法（两个终端，隔离域 87 + 工作区内 ROS_LOG_DIR）：

    # 终端 1 —— 只起 LIO 节点（不给它任何仿真）
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ROS_DOMAIN_ID=87 ROS_LOG_DIR=$PWD/.tmp_roslog/alone \\
    ros2 run small_point_lio small_point_lio_node --ros-args \\
      --params-file install/small_point_lio/share/small_point_lio/config/mid360_sim.yaml \\
      -p use_sim_time:=false -p save_pcd:=false

    # 终端 2 —— 喂数据 + 判读
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ROS_DOMAIN_ID=87 python3 tools/lio_node_alone_check.py --duration 40

复现"上游原版在无 timebase 的仿真数据上一条不发"（见 docs/lio_slots.md §5）：
    # 点云时间整体**落后** IMU 时钟 650 s —— 与"上游把 timebase=0 直接当 0.0 s、
    # 而仿真钟已在 650 s"是**同一套算术** ⇒ 实测 /Odometry 0 条、TF 0 条。
    python3 tools/lio_node_alone_check.py --lidar-time-lag -650 --duration 25

参数摘要：--bag/--imu-topic/--lidar-topic/--duration/--speed/--timebase sim|header|
         --lidar-time-lag/--qos-... （--help 看全部）
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sqlite3
import sys
import threading
import time
from collections import Counter

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                           QoSReliabilityPolicy, qos_profile_sensor_data)
    from rclpy.time import Time
    from rclpy.duration import Duration
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(f"[FATAL] 需要 ROS 2 环境（先 source /opt/ros/humble/setup.bash 与 install/setup.bash）：{exc}\n")
    raise SystemExit(2)

try:
    from rclpy.serialization import deserialize_message
except ImportError:  # pragma: no cover
    deserialize_message = None


# ---------------------------------------------------------------- bag 读取（纯 sqlite3，不依赖 rosbag2_py）

class BagStream:
    """按时间升序**流式**吐某个话题的 (t_bag_ns, raw_bytes)；不在内存里囤消息。"""

    def __init__(self, db3_path: str, topic: str):
        self.db3 = db3_path
        self.topic = topic
        self.count = 0
        con = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
        cur = con.cursor()
        cur.execute("select id, type from topics where name = ?", (topic,))
        row = cur.fetchone()
        if row is None:
            con.close()
            raise KeyError(f"bag 里没有话题 {topic}")
        self.topic_id, self.msg_type = row
        cur.execute("select count(*) from messages where topic_id = ?", (self.topic_id,))
        self.total = cur.fetchone()[0]
        cur.execute("select timestamp, data from messages where topic_id = ? order by timestamp asc",
                    (self.topic_id,))
        self._rows = cur  # 懒迭代，不 fetchall
        self._con = con
        self._next = None
        self._advance()

    def _advance(self):
        row = self._rows.fetchone()
        self._next = (row[0], row[1]) if row is not None else None

    def peek_t(self):
        return None if self._next is None else self._next[0]

    def pop(self):
        cur = self._next
        if cur is None:
            return None
        self.count += 1
        self._advance()
        return cur

    def close(self):
        try:
            self._con.close()
        except Exception:
            pass


def _msg_type_of(db3: str, topic: str):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    cur = con.cursor()
    cur.execute("select type from topics where name = ?", (topic,))
    row = cur.fetchone()
    con.close()
    return None if row is None else row[0]


# ---------------------------------------------------------------- 点云转换

def _read_xyz(msg: PointCloud2):
    """从 PointCloud2 里只取 x/y/z（字段偏移按 fields 表算，不假设 point_step）。

    用 numpy 向量化：一帧 30000 点，纯 Python 逐点解包要几百 ms（喂不到 10 Hz），
    numpy 视图 + 掩码是微秒级。返回 (N,3) 的 float32 数组。
    """
    import numpy as np
    off = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"} <= set(off):
        raise ValueError(f"点云缺少 x/y/z，只有 {list(off)}")
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    x, y, z = a["x"], a["y"], a["z"]
    good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & ~((x == 0) & (y == 0) & (z == 0))
    # (0,0,0) 是早期 bag 的占位点（现插件已不发）；非有限值一并丢掉
    return np.stack([x[good], y[good], z[good]], axis=1)


def make_custom_msg(pts, stamp: Time, frame_id: str, timebase_mode: str) -> CustomMsg:
    m = CustomMsg()
    m.header.stamp = stamp.to_msg()
    m.header.frame_id = frame_id
    if timebase_mode == "header":
        m.timebase = stamp.nanoseconds          # 真机 livox_ros_driver2 的语义
    else:
        m.timebase = 0                          # 本仓仿真插件的现状（从不赋值）
    m.point_num = len(pts)
    m.lidar_id = 0
    arr = []
    for (x, y, z) in pts:
        p = CustomPoint()
        p.x, p.y, p.z = float(x), float(y), float(z)
        p.offset_time = 0                       # 仿真插件：帧内无运动，恒 0
        p.reflectivity = 0
        p.tag = 0                               # 仿真插件不赋值 ⇒ 0 ⇒ 通过 (tag & 0b111111)==0
        p.line = 0
        arr.append(p)
    m.points = arr
    return m


# ---------------------------------------------------------------- 主节点

class Checker(Node):
    def __init__(self, args):
        super().__init__("lio_node_alone_check")
        self.args = args
        self.odom_topics = [t.strip() for t in args.odom_topics.split(",") if t.strip()]
        self.odom_stats = {t: {"n": 0, "first": None, "last": None, "child": None, "frame": None,
                               "path": 0.0, "prev": None, "traj": []} for t in self.odom_topics}
        self.tf_edges = Counter()
        self.tf_odom_base = {"n": 0, "first": None, "last": None, "t_last": None}
        self.cloud_registered = {"n": 0, "pts": 0}
        self.static_edges = Counter()

        qos_be = qos_profile_sensor_data
        for t in self.odom_topics:
            self.create_subscription(Odometry, t, lambda m, t=t: self._on_odom(m, t), 50)
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_be)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, 50)
        self.create_subscription(PointCloud2, "/cloud_registered", self._on_cloud, qos_be)

        # /tf_static：TRANSIENT_LOCAL ⇒ 节点后启动也能拿到（真跑时由 robot_state_publisher 发）
        qos_static = QoSProfile(depth=20, history=QoSHistoryPolicy.KEEP_LAST,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_static = self.create_publisher(TFMessage, "/tf_static", qos_static)
        imu_qos = qos_be if args.imu_qos == "sensor" else QoSProfile(
            depth=200, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.pub_imu = self.create_publisher(Imu, args.imu_topic, imu_qos)
        # ⚠️ 出话题是 --out-topic（默认 /livox/lidar），**不是** bag 里那个 --lidar-topic：
        #    早期版本这里写错成 args.lidar_topic ⇒ CustomMsg 发到 /livox/lidar/pointcloud，
        #    与节点的订阅（/livox/lidar）不匹配，表现为"喂了几万点却一条 odom 都没有"。
        self.pub_lidar = self.create_publisher(CustomMsg, args.out_topic, qos_be)

        self.tf_lookup = {"ok": 0, "err": None, "last_xyz": None}
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception as exc:  # pragma: no cover
            self.tf_buffer = None
            self.tf_lookup["err"] = f"tf2_ros 不可用: {exc}"

    # ---- 回调
    def _on_odom(self, m: Odometry, topic: str):
        s = self.odom_stats[topic]
        s["n"] += 1
        s["child"] = m.child_frame_id
        s["frame"] = m.header.frame_id
        p = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)
        if s["first"] is None:
            s["first"] = p
        if s["prev"] is not None:
            s["path"] += math.dist(p, s["prev"])
        s["prev"] = p
        s["last"] = p
        s["traj"].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p[0], p[1], p[2]))

    def _on_tf(self, m: TFMessage):
        for t in m.transforms:
            self.tf_edges[(t.header.frame_id, t.child_frame_id)] += 1
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                self.tf_odom_base["n"] += 1
                self.tf_odom_base["last"] = t.header.stamp
                tr = t.transform.translation
                self.tf_odom_base["t_last"] = (tr.x, tr.y, tr.z)
                if self.tf_odom_base["first"] is None:
                    self.tf_odom_base["first"] = t.header.stamp

    def _on_tf_static(self, m: TFMessage):
        for t in m.transforms:
            self.static_edges[(t.header.frame_id, t.child_frame_id)] += 1

    def _on_cloud(self, m: PointCloud2):
        self.cloud_registered["n"] += 1
        self.cloud_registered["pts"] = m.width * m.height

    # ---- 发布
    def publish_static_tf(self):
        msg = TFMessage()
        for parent, child, xyz in (("base_link", "livox_frame", (0.12, 0.0, 0.175)),
                                   ("base_link", "imu_link", (0.12, 0.0, 0.125))):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            msg.transforms.append(t)
        self.pub_static.publish(msg)

    def publish_imu(self, raw: bytes, stamp: Time):
        m = deserialize_message(raw, Imu)
        m.header.stamp = stamp.to_msg()
        self.pub_imu.publish(m)
        return m

    def publish_lidar(self, raw: bytes, stamp: Time):
        pc = deserialize_message(raw, PointCloud2)
        pts = _read_xyz(pc)
        if self.args.point_stride > 1:
            pts = pts[::self.args.point_stride]
        if len(pts) == 0:
            return 0
        self.pub_lidar.publish(make_custom_msg(pts, stamp, self.args.lidar_frame, self.args.timebase))
        return len(pts)

    def do_tf_lookup(self):
        if self.tf_buffer is None:
            return
        try:
            tr = self.tf_buffer.lookup_transform("odom", "base_link", Time())
            self.tf_lookup["ok"] += 1
            t = tr.transform.translation
            self.tf_lookup["last_xyz"] = (t.x, t.y, t.z)
            self.tf_lookup["err"] = None
        except Exception as exc:
            self.tf_lookup["err"] = f"{type(exc).__name__}: {exc}"


# ---------------------------------------------------------------- 主流程

def pick_db3(bag_dir: str) -> str:
    if os.path.isfile(bag_dir):
        return bag_dir
    cands = sorted(f for f in os.listdir(bag_dir) if f.endswith(".db3"))
    if not cands:
        raise SystemExit(f"[FATAL] {bag_dir} 下没有 .db3")
    return os.path.join(bag_dir, cands[0])


def main():
    ap = argparse.ArgumentParser(description="单独喂/验一个 LIO 节点（不开 Gazebo/nav2）")
    ap.add_argument("--bag", default=".tmp_bags/ret4", help="bag 目录或 .db3（默认 .tmp_bags/ret4）")
    ap.add_argument("--imu-topic", default="/livox/imu")
    ap.add_argument("--lidar-topic", default="/livox/lidar/pointcloud", help="bag 里的点云话题（PointCloud2）")
    ap.add_argument("--out-topic", default="/livox/lidar", help="往 LIO 发的 CustomMsg 话题")
    ap.add_argument("--frame-id", dest="lidar_frame", default="livox_frame")
    ap.add_argument("--duration", type=float, default=40.0, help="重放多少秒 bag 时间（默认 40）")
    ap.add_argument("--start-offset", type=float, default=0.0, help="从 bag 起点跳过多少秒（默认 0）")
    ap.add_argument("--speed", type=float, default=1.0, help="重放倍速（默认 1.0 实时）")
    ap.add_argument("--timebase", choices=["sim", "header"], default="sim",
                    help="sim = timebase 恒 0（本仓仿真插件的现状，默认）；header = timebase=header.stamp(ns)（真机语义）")
    ap.add_argument("--lidar-time-lag", type=float, default=0.0,
                    help="给点云 header.stamp 加这么多秒（**正则点时间超前于 IMU 时钟，负则落后**）。"
                         "`-650` 复现「上游原版在 timebase=0 的仿真数据上一条不发」：仿真钟跑到 ~650 s 时，"
                         "未打补丁的适配器把点时间算成 0.0 s ⇒ 点比 IMU 落后 650 s ⇒ 整帧点云被丢弃。"
                         "实测 `-650` = /Odometry 0 条、TF 0 条（静默）；`+650` = 71 Hz 但位姿恒 (0,0,0) 且无 /cloud_registered")
    ap.add_argument("--point-stride", type=int, default=1,
                    help="点云抽稀（默认 1 = 全量；Python 侧造 CustomPoint 太慢时可调 2/3）")
    ap.add_argument("--odom-topics", default="/odom,/Odometry,/aft_mapped_to_init",
                    help="要盯的里程计话题（逗号分隔；默认覆盖 fastlio/pointlio/small_point_lio 三种命名）")
    ap.add_argument("--imu-qos", choices=["sensor", "reliable"], default="sensor",
                    help="喂 IMU 用的 QoS：sensor=BEST_EFFORT（point_lio / small_point_lio 订的就是它，默认）；"
                         "reliable=RELIABLE（**FAST-LIO 订的是 RELIABLE** —— 用 sensor 喂它时会看到 "
                         "'offering incompatible QoS ... RELIABILITY_QOS_POLICY'，一条都收不到）")
    ap.add_argument("--ref-topics", default="/odom_ground_truth,/odom",
                    help="bag 内用于**对照轨迹长度**的话题（逗号分隔）。"
                         "`/odom_ground_truth` = 仿真真值；`/odom` = 那次跑的时候录下来的 FAST-LIO 输出 —— "
                         "后者是同一段数据上的现成基线，用来判「是喂法不对」还是「这个 LIO 真的跟丢了」")
    # ---- 2026-10-05：给 A/B 扫参用的机器可读输出（**可选**，不传时行为与旧版完全一致）
    ap.add_argument("--json-out", default=None,
                    help="把判读结果写成 JSON（供 tools/lio_param_sweep.py 扫参驱动解析；不传则不写）")
    ap.add_argument("--traj-out", default=None,
                    help="把「本节点里程计 + bag 参照」的逐帧轨迹 (t,x,y,z) 写成 JSON（诊断用；不传则不写）")
    args = ap.parse_args()

    db3 = pick_db3(args.bag)
    print(f"[info] bag      : {db3}")
    print(f"[info] imu      : {args.imu_topic}  →  {args.imu_topic}")
    print(f"[info] lidar    : {args.lidar_topic} (PointCloud2)  →  {args.out_topic} (CustomMsg)")
    print(f"[info] timebase : {args.timebase}   lidar_time_lag={args.lidar_time_lag}s   speed={args.speed}")

    try:
        imu_stream = BagStream(db3, args.imu_topic)
        lidar_stream = BagStream(db3, args.lidar_topic)
    except KeyError as exc:
        raise SystemExit(f"[FATAL] {exc}")
    ref_topics = [t.strip() for t in args.ref_topics.split(",") if t.strip()]
    ref_streams, ref_stats = {}, {}
    for t in ref_topics:
        if _msg_type_of(db3, t):
            try:
                ref_streams[t] = BagStream(db3, t)
                ref_stats[t] = {"n": 0, "path": 0.0, "prev": None, "first": None, "last": None, "traj": []}
            except KeyError:
                pass

    t_bag0 = min(x for x in (imu_stream.peek_t(), lidar_stream.peek_t()) if x is not None)
    t_bag_end = t_bag0 + int(args.start_offset * 1e9)

    rclpy.init()
    node = Checker(args)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    # 丢掉 start_offset 之前的数据
    while imu_stream.peek_t() is not None and imu_stream.peek_t() < t_bag_end:
        imu_stream.pop()
    while lidar_stream.peek_t() is not None and lidar_stream.peek_t() < t_bag_end:
        lidar_stream.pop()
    for st in ref_streams.values():
        while st.peek_t() is not None and st.peek_t() < t_bag_end:
            st.pop()

    t_wall0 = node.get_clock().now()
    t_bag_start = min(x for x in (imu_stream.peek_t(), lidar_stream.peek_t()) if x is not None)
    t_bag_stop = t_bag_start + int(args.duration * 1e9)

    n_imu = n_lidar = n_pts = 0

    node.publish_static_tf()

    def due(t_bag):
        """bag 时间 → 墙钟目标时刻（ns）。既用于**定时**，也用于**盖戳** ——
        盖戳用理想时刻而不是 now()，这样即使 Python 侧发得慢，消息之间的时间间隔仍与原始 bag 一致。"""
        return t_wall0.nanoseconds + int((t_bag - t_bag_start) / args.speed)

    next_report = time.time() + 5.0
    last_static = 0.0
    t_start = time.time()

    while rclpy.ok():
        now_ns = node.get_clock().now().nanoseconds
        # 谁的时间戳更早先发谁（保持 IMU 与点云的**原始先后关系**，这是 ESKF 正常工作的前提）
        t_imu = imu_stream.peek_t()
        t_lidar = lidar_stream.peek_t()
        nxt = None
        if t_imu is not None and t_imu <= t_bag_stop:
            nxt = t_imu
        if t_lidar is not None and t_lidar <= t_bag_stop and (nxt is None or t_lidar < nxt):
            nxt = t_lidar
        if nxt is None:
            break
        wait = (due(nxt) - now_ns) / 1e9
        if wait > 0:
            time.sleep(min(wait, 0.005))
            continue
        if t_imu is not None and t_imu == nxt:
            raw = imu_stream.pop()[1]
            # ★ 时间戳必须用"按 bag 时间轴重定基准后的**理想时刻**"，不能用"发布那一刻的墙钟"：
            #   本脚本是 Python，一帧 7500 点造 CustomMsg 要几十 ms ⇒ 用 now() 盖戳时，
            #   同一批积压消息会挤在几百微秒内（dt≈0），ESKF 会连着做多次同刻残差更新而发散。
            #   实测（未修前）：small_point_lio 轨迹 167 m / point_lio 25 m，而参照（那次跑的
            #   FAST-LIO 录制输出）只有 6.2 m —— 两个 LIO 一起"发散"就说明是喂法的问题，不是算法。
            node.publish_imu(raw, Time(nanoseconds=due(nxt)))
            n_imu += 1
        if t_lidar is not None and t_lidar == nxt:
            raw = lidar_stream.pop()[1]
            stamp = Time(nanoseconds=due(nxt) + int(args.lidar_time_lag * 1e9))
            n_pts += node.publish_lidar(raw, stamp)
            n_lidar += 1
        for topic, st in ref_streams.items():
            while st.peek_t() is not None and st.peek_t() <= nxt:
                raw = st.pop()[1]
                m = deserialize_message(raw, Odometry)
                p = (m.pose.pose.position.x, m.pose.pose.position.y)
                s = ref_stats[topic]
                if s["prev"] is not None:
                    s["path"] += math.dist(p, s["prev"])
                s["prev"] = p
                s["n"] += 1
                if s["first"] is None:
                    s["first"] = p
                s["last"] = p
                s["traj"].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p[0], p[1], 0.0))
        # 每 5 s 复述一次 /tf_static（模拟 robot_state_publisher 的重复发布），并做一次 TF 查询
        if time.time() - last_static > 5.0:
            last_static = time.time()
            node.publish_static_tf()
            node.do_tf_lookup()
        if time.time() > next_report:
            next_report = time.time() + 5.0
            tot = sum(s["n"] for s in node.odom_stats.values())
            print(f"  [t+{int(time.time() - t_start)}s] imu={n_imu} lidar={n_lidar} "
                  f"pts={n_pts} 里程计合计={tot} "
                  f"tf(odom→base_link)={node.tf_odom_base['n']} cloud_reg={node.cloud_registered['n']}",
                  flush=True)

    # 收尾：再等 2 s 让最后几帧的 TF/odom 回调落地
    time.sleep(2.0)
    node.do_tf_lookup()
    time.sleep(0.3)

    # ---------------------------------------------------------------- 判读
    print("\n================ 判读 ================")
    print(f"发布给 LIO 的输入 : Imu {n_imu} 条 / CustomMsg {n_lidar} 帧 / 共 {n_pts} 点 "
          f"（均 {n_pts/max(n_lidar,1):.0f} 点/帧）")

    ok_odom = False
    for topic, s in node.odom_stats.items():
        rate = s["n"] / max(args.duration, 1e-9)
        print(f"里程计话题 {topic:10s}: {s['n']} 条 ({rate:.1f} Hz)  "
              f"frame_id={s['frame']} child_frame_id={s['child']}")
        if s["n"]:
            ok_odom = True
            print(f"            首帧位置 {tuple(round(v,3) for v in s['first'])} → "
                  f"末帧位置 {tuple(round(v,3) for v in s['last'])}  轨迹长度 {s['path']:.2f} m")

    print(f"TF odom→base_link : {node.tf_odom_base['n']} 条 "
          f"({node.tf_odom_base['n']/max(args.duration,1e-9):.1f} Hz)  "
          f"末条位置 {tuple(round(v,3) for v in node.tf_odom_base['t_last'])}"
          if node.tf_odom_base["n"] else "TF odom→base_link : **0 条**")
    print(f"tf2 查询 odom→base_link : 成功 {node.tf_lookup['ok']} 次"
          + (f"，最近错误 {node.tf_lookup['err']}" if node.tf_lookup["err"] else ""))
    print(f"/cloud_registered : {node.cloud_registered['n']} 条 "
          f"(末帧 {node.cloud_registered['pts']} 点)  "
          f"—— 注意该话题**只有存在订阅者时**才发布（small_point_lio_node.cpp:102）")

    print("看到的 /tf 边（parent→child : 条数）:")
    for (p, c), n in node.tf_edges.most_common():
        print(f"    {p} → {c} : {n}")
    print("看到的 /tf_static 边（我们自己发的两条）:")
    for (p, c), n in node.static_edges.most_common():
        print(f"    {p} → {c} : {n}")

    # 与 bag 内参照轨迹对比：轨迹长度与"最大位移"都是**与坐标系原点无关**的量
    ref_paths = {t: s for t, s in ref_stats.items() if s["n"] > 1}
    if ref_paths:
        # 取"实际出数最多的那个里程计话题"参与比值
        best_topic, best = None, None
        for t, s in node.odom_stats.items():
            if s["n"] and (best is None or s["n"] > best["n"]):
                best_topic, best = t, s
        print("bag 内参照轨迹（同一窗口，轨迹长度与原点无关 ⇒ 可直接比）:")
        for t, s in ref_paths.items():
            print(f"    {t:22s}: {s['n']} 条  轨迹长度 {s['path']:7.2f} m  "
                  f"位移 {math.dist(s['first'], s['last']):6.2f} m")
        if best is not None:
            ref = ref_paths.get("/odom_ground_truth") or list(ref_paths.values())[0]
            ratio = best["path"] / max(ref["path"], 1e-9)
            print(f"    ⇒ {best_topic} 轨迹长度 / 参照 = {ratio:.2f} "
                  f"（≈1 说明跟住了；≫1 或 ≪1 说明**发得出数但跟丢了/尺度不对**）")
            if ratio > 3.0 or ratio < 0.33:
                print("    ⚠️ 注意：链路是通的，但**轨迹长度对不上参照** ⇒ 这是算法/参数问题，"
                      "不是话题/QoS/时间戳问题（真值见上表）")

    print("\n---------------- 结论 ----------------")
    one_map_odom = sum(1 for (p, c) in node.tf_edges if p == "map" and c == "odom")
    print(f"LIO 自己有没有发 map→odom：{'有（' + str(one_map_odom) + ' 条）' if one_map_odom else '没有 ✅'}"
          "  —— 按本仓契约，map→odom 只能由重定位槽发")
    if ok_odom and node.tf_odom_base["n"] > 0:
        print("✅ PASS：节点吃进了输入，并在发 /odom + odom→base_link TF。")
        print("   ⇒ 本槽位**不需要** lio_tf_adapter（它自己就是 odom→base_link 的唯一发布者）；")
        print("   ⇒ 若同时起 adapter，odom 会出现两个父边（必须靠 bringup 的条件分支关掉）。")
    elif n_lidar and n_imu and not ok_odom:
        print("❌ FAIL：输入发出去了，但**一条里程计/TF 都没有**。按以下顺序查：")
        print("   1) 逐点时间戳：点云 header.stamp / CustomMsg.timebase 是否与 IMU 同一时间轴、且不落后？")
        print("      （落后就命中小 point lio.cpp:103 的 `timestamp < time_current` ⇒ 每帧点云被整帧丢弃）")
        print("   2) base_link→livox_frame 静态 TF 是否存在（缺了会在节点日志刷")
        print("      'Failed to lookup transform from base_link to livox_frame' 并每帧 return）")
        print("   3) gravity 初始化：fix_gravity_direction=true 需要 ≥200 帧 IMU（@100 Hz ≈ 2 s）才开始")
    else:
        print("❌ FAIL：连输入都没发出去（bag 话题名/类型不对？）")

    # ---------------- 机器可读输出（--json-out / --traj-out，可选）----------------
    if args.traj_out:
        traj = {
            "node": {t: s["traj"] for t, s in node.odom_stats.items() if s["n"]},
            "ref": {t: s["traj"] for t, s in ref_stats.items() if s["n"]},
        }
        with open(args.traj_out, "w") as fh:
            json.dump(traj, fh)
        print(f"[json] 轨迹已写入 {args.traj_out}")

    if args.json_out:
        best_topic, best = None, None
        for t, s in node.odom_stats.items():
            if s["n"] and (best is None or s["n"] > best["n"]):
                best_topic, best = t, s
        ref_best = None
        for t in ("/odom_ground_truth", "/odom"):
            if ref_stats.get(t, {}).get("n", 0) > 1:
                ref_best = ref_stats[t]
                break
        if ref_best is None:
            for t, s in ref_stats.items():
                if s["n"] > 1:
                    ref_best = s
                    break

        def _slim(s):
            return {"n": s["n"], "frame": s.get("frame"), "child": s.get("child"),
                    "path": s["path"], "first": s["first"], "last": s["last"],
                    "displacement": (math.dist(s["first"], s["last"])
                                     if s["first"] and s["last"] else None)}

        payload = {
            "bag": db3, "duration": args.duration, "speed": args.speed,
            "inputs": {"imu": n_imu, "lidar": n_lidar, "points": n_pts},
            "odom": {t: _slim(s) for t, s in node.odom_stats.items()},
            "tf_odom_base": {"n": node.tf_odom_base["n"], "last": node.tf_odom_base["t_last"]},
            "cloud_registered": dict(node.cloud_registered),
            "refs": {t: _slim(s) for t, s in ref_stats.items()},
            "best_topic": best_topic,
            "best": _slim(best) if best else None,
            "ref_topic": next((t for t, s in ref_stats.items() if s is ref_best), None),
            "ref": _slim(ref_best) if ref_best else None,
            "ratio_vs_ref": ((best["path"] / ref_best["path"])
                             if (best and ref_best and ref_best["path"] > 1e-9) else None),
        }
        with open(args.json_out, "w") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=1)
        print(f"[json] 判读结果已写入 {args.json_out}")

    imu_stream.close()
    lidar_stream.close()
    for st in ref_streams.values():
        st.close()
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
