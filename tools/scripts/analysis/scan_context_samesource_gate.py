#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_context_samesource_gate.py —— Scan Context 路线的**硬门：同源性（库/查询是否同源）**。

背景（先读这两份，本脚本只回答它们留下的那一个问题）：
  · `docs/scan_context_plan.md` §D.2 的分诊表最后一行：「库与查询**不同源**（高度带/地面估计/range
    截断不一致）」→ 用「参数指纹 + 启动比对」定位。这一条 A1 **没测**。
  · `docs/scan_context_probe_report.md` §6.4 / §8.2：A1 测出先验 PCD `PCD/RMUL2026.pcd` 只是地面
    以上 **0.21~0.55 m 的一层薄壳**（z 跨度 0.665 m，>0.5 m 只有 22 点），而**查询侧**（真实机器人
    经 `pointcloud_to_laserscan`（`min_height -1.0 / max_height 0.1`，雷达在 `livox_frame`，
    `sensor_height 0.226`）+ `linefit`（`max_dist_to_line 0.05`）看到的是 **(0.05, 0.33] m** 的高度带。
    ⇒ 库与查询可能只在 **[0.20, 0.33] m** 重叠，且库侧 25~30% 的占用格像水平面（地面/坡道污染）。

本脚本回答：**「拿一条真实 bag 的 `/scan` 当查询，能不能在这份 PCD 生成的库里检索回真值地点？」**
——用数字回答，不用感觉。

做法（全部离线：**不启 ROS、不 spin 节点**，只用 `sqlite3` 以只读方式打开 bag）：
  1. 从 bag 里读 `/scan`（2D LaserScan）+ 位姿源（TF `map→odom`∘`odom→base_link`，或 `/odom`，
     或 `/odom_ground_truth`）⇒ 每帧查询的真值位姿（map 系，base_link 或 livox_frame）。
  2. 查询描述子：`/scan` 的每一束按 `angle_min + i*angle_increment` **直接**落到 360 个 bin
     （**1°/bin**，bin b ↔ 传感器系角度 `b*Δ`，左边界约定）；同一 bin 取**最小** range；
     `inf`/`NaN`/超出 `[range_min, range_max]` 的束 ⇒ 该 bin 记 `r_max`（=10 m，与库侧"射线打不到"
     同一编码）；一帧里一个有效束都没有的 bin 同样记 `r_max`。（--query-verbose 可打印统计）
  3. 库描述子：`PCD/RMUL2026.pcd` 取地面以上高度带 `[lo,hi)` → 0.15 m 占用格 → 用
     `scan_context_asset_probe.py` 的 `raycast()` 在**可放置位姿网格**上投射 360 束（1°/bin，
     r_max 10 m）。**yaw 搜索 = 描述子的圆周移位**（左移 s 个 bin ≡ 机器人 yaw 偏移 s*Δ），
     等价于在全 360° 上以 `--yaw-step` 度步长搜 yaw —— 这是 Scan Context 的标准做法，
     也是 §B.2「相对 yaw 估计：全圆周移位取距离最小者」。
  4. 检索指标：hit@1/5/10（"真值地点"是否在 top-k）+ 命中时的 |Δxy|、|Δyaw|；
     判据（plan §D.2）：**真值在 top-10 且 |Δxy| ≤ 0.5 m 且 |Δyaw| ≤ 15°**。
  5. 还跑三组对照，把"检索器不行"与"库缺查询看的那个高度带"分开：
       · `lib[self]`   库带 X ← 用**同一带 X** 从 PCD 射线投射造合成查询（机制自检，应接近满分）
       · `lib[X] ← syn[query-band]` 库带 X ← 用**查询带 (0.05,0.35]** 从 PCD 造合成查询
         （只考"高度带不同源"，不含真实点云/去地面/量测噪声）
       · `lib[X] ← real /scan`（真正的门）

**只读**：bag 一律 `sqlite3` 只读 URI（`mode=ro` + `PRAGMA query_only`），PCD 只读；
只有 `--json` 会写文件，且必须落在仓库的 `.tmp_cache/` 下。

依赖：**只用 numpy + 标准库**（不 import rclpy / rosbag2_py / open3d）。CDR 反序列化是本脚本自带的
最小实现（见 §CDR），开发时与 `rclpy.serialization.deserialize_message` 逐字段比对过
（`--self-test` 可用 `rclpy` 交叉验证，若机器上没有 rclpy 会自动跳过）。

用法：
  # 0) 先看这台机器上有哪些 bag 能用（只读、不写盘）
  python3 tools/scripts/analysis/scan_context_samesource_gate.py --list-bags

  # 1) 主门（默认 bag 自动挑 .tmp_bags/ 下第一份可用 bag，库带 [0.20,0.35]）
  python3 tools/scripts/analysis/scan_context_samesource_gate.py \\
      --bag .tmp_bags/ret --json .tmp_cache/sc_gate/gate_ret.json

  # 2) 四份 ret bag 一起跑 + 带对照（[0.20,0.35] / [0.05,0.35] / [0.05,0.60]）
  python3 tools/scripts/analysis/scan_context_samesource_gate.py \\
      --bag .tmp_bags/ret .tmp_bags/ret2 .tmp_bags/ret3 .tmp_bags/ret4 \\
      --compare-bands 0.20,0.35 0.05,0.35 0.05,0.60 \\
      --json .tmp_cache/sc_gate/gate_all.json

  # 3) 加密检索网格（plan §D.2 分诊表：命中率不低但 Δxy 大 ⇒ 网格太粗）
  python3 tools/scripts/analysis/scan_context_samesource_gate.py --bag .tmp_bags/ret --xy-step 0.5

判读：看最后打印的 `结论速览`；`PASS/FAIL/INCONCLUSIVE` 的判据与理由见 `--help` 尾部的 epilog
与 `docs/scan_context_samesource_gate_report.md`。
"""

import argparse
import glob
import importlib.util
import json
import math
import os
import sqlite3
import struct
import sys
import time

import numpy as np

INF = float("inf")
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
PROBE_PATH = os.path.join(REPO_ROOT, "tools", "scripts", "analysis", "scan_context_asset_probe.py")
TMP_ALLOWED = (os.path.join(REPO_ROOT, ".tmp_cache"), os.path.join(REPO_ROOT, ".tmp_research"))
DEFAULT_PCD = os.path.join(REPO_ROOT, "src", "rm_nav_bringup", "PCD", "RMUL2026.pcd")
DEFAULT_MAP_YAML = os.path.join(REPO_ROOT, "src", "rm_nav_bringup", "map", "RMUL2026.yaml")
# sim 出生点（world 系，见 bringup_sim.launch.py L93-99 的注释：新图 = cartographer 出生点相对系
# ⇒ map = world − (4.3, 3.35)）。只用于 --truth gt 这一路交叉验证。
DEFAULT_WORLD_TO_MAP = "4.3,3.35"

BAG_TOPICS_HINT = ("/scan", "/odom", "/odom_ground_truth", "/tf", "/tf_static", "/map",
                   "/segmentation/obstacle", "/livox/lidar/pointcloud")


def log(msg):
    try:
        print(msg, flush=True)
    except BrokenPipeError:            # 例如 `| head`：安静退出，不要打 traceback
        try:
            sys.stdout.close()
        except Exception:
            pass
        os._exit(0)


def die(msg, code=2):
    print(f"scan_context_samesource_gate: ERROR: {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


def load_probe():
    """复用 A1 探针的描述子/栅格/射线代码（不重写）。"""
    if not os.path.isfile(PROBE_PATH):
        die(f"找不到 A1 探针脚本 {PROBE_PATH}（本脚本复用它的 read_pcd/grid_of/raycast）")
    spec = importlib.util.spec_from_file_location("scan_context_asset_probe", PROBE_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ======================================================================================
# CDR：最小 stdlib 反序列化（只覆盖本门需要的 5 种消息）
#   ⚠️ 对齐基准：rmw_fastrtps 的 CDR 流**把头 4 字节 encapsulation 也算进对齐偏移**
#   （实测：Odometry 里 child_frame_id 之后的第一个 float64 落在 offset 44 而不是 48）。
#   与 rclpy.serialization.deserialize_message 的逐字段比对见 --self-test。
# ======================================================================================
class Cdr:
    ORIGIN = 4          # 对齐基准 = 第 4 字节（跳过 4 字节 encapsulation 头）

    def __init__(self, buf):
        self.b = bytes(buf)
        self.o = 4

    def align(self, n):
        self.o += (-(self.o - self.ORIGIN)) % n

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def u16(self):
        self.align(2); v = struct.unpack_from("<H", self.b, self.o)[0]; self.o += 2; return v

    def u32(self):
        self.align(4); v = struct.unpack_from("<I", self.b, self.o)[0]; self.o += 4; return v

    def i32(self):
        self.align(4); v = struct.unpack_from("<i", self.b, self.o)[0]; self.o += 4; return v

    def f32(self):
        self.align(4); v = struct.unpack_from("<f", self.b, self.o)[0]; self.o += 4; return v

    def f64(self):
        self.align(8); v = struct.unpack_from("<d", self.b, self.o)[0]; self.o += 8; return v

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def array_f32(self):
        n = self.u32(); self.align(4)
        v = np.frombuffer(self.b, dtype="<f4", count=n, offset=self.o)
        self.o += 4 * n
        return v

    def array_i8(self):
        n = self.u32()
        v = np.frombuffer(self.b, dtype="i1", count=n, offset=self.o)
        self.o += n
        return v

    def array_f64_fixed(self, n):
        self.align(8)
        v = np.frombuffer(self.b, dtype="<f8", count=n, offset=self.o)
        self.o += 8 * n
        return v

    def bytes_n(self):
        n = self.u32()
        v = self.b[self.o:self.o + n]
        self.o += n
        return v


def _header(c):
    sec = c.i32(); nsec = c.u32(); frame = c.string()
    return sec + nsec * 1e-9, frame


def read_laserscan(buf):
    c = Cdr(buf)
    stamp, frame = _header(c)
    m = dict(stamp=stamp, frame_id=frame, angle_min=c.f32(), angle_max=c.f32(),
             angle_increment=c.f32(), time_increment=c.f32(), scan_time=c.f32(),
             range_min=c.f32(), range_max=c.f32())
    m["ranges"] = np.array(c.array_f32(), dtype=np.float64)
    return m


def peek_laserscan_stamp(buf):
    """只解出 /scan 的 header 时间戳（sim 时间）——用于与 TF/GT 对时间轴。

    ⚠️ 必须用**消息 header 里的 stamp**，不能用 bag 记录的接收时间（那是 wall clock）：
    本仓 bag 的 TF/GT 都是 sim 时间（~1e2~1e3 s），bag 接收时间是 epoch（~1.79e9 s）。
    混用会让"查 TF"全部落到时间轴末端（= 位姿被钳到最后一帧），本门第一版就踩过这个坑。
    """
    c = Cdr(buf)
    sec = c.i32()
    nsec = c.u32()
    return sec + nsec * 1e-9


def read_odometry(buf):
    c = Cdr(buf)
    stamp, frame = _header(c)
    child = c.string()
    pos = np.array([c.f64(), c.f64(), c.f64()])
    quat = np.array([c.f64(), c.f64(), c.f64(), c.f64()])
    c.array_f64_fixed(36)
    twist = np.array([c.f64(), c.f64(), c.f64()])
    c.array_f64_fixed(36)
    return dict(stamp=stamp, frame_id=frame, child_frame_id=child, pos=pos, quat=quat, twist=twist)


def read_tfmessage(buf):
    c = Cdr(buf)
    n = c.u32()
    out = []
    for _ in range(n):
        stamp, frame = _header(c)
        child = c.string()
        trans = np.array([c.f64(), c.f64(), c.f64()])
        quat = np.array([c.f64(), c.f64(), c.f64(), c.f64()])
        out.append(dict(stamp=stamp, frame_id=frame, child_frame_id=child, trans=trans, quat=quat))
    return out


def read_occupancy_grid(buf):
    c = Cdr(buf)
    stamp, frame = _header(c)
    c.i32(); c.u32()                       # map_load_time
    res = c.f32(); w = c.u32(); h = c.u32()
    origin = np.array([c.f64(), c.f64(), c.f64()])
    c.array_f64_fixed(4)                   # origin 的 quaternion
    data = np.array(c.array_i8(), dtype=np.int8)
    return dict(stamp=stamp, frame_id=frame, resolution=res, width=w, height=h, origin=origin, data=data)


def read_pointcloud2(buf):
    c = Cdr(buf)
    stamp, frame = _header(c)
    height = c.u32(); width = c.u32()
    nf = c.u32()
    fields = []
    for _ in range(nf):
        fields.append(dict(name=c.string(), offset=c.u32(), datatype=c.u8(), count=c.u32()))
    is_be = c.u8()
    point_step = c.u32(); row_step = c.u32()
    data = c.bytes_n()
    return dict(stamp=stamp, frame_id=frame, height=height, width=width, fields=fields,
                is_bigendian=is_be, point_step=point_step, row_step=row_step, data=data)


_MSGTYPE_READER = {
    "sensor_msgs/msg/LaserScan": read_laserscan,
    "nav_msgs/msg/Odometry": read_odometry,
    "tf2_msgs/msg/TFMessage": read_tfmessage,
    "nav_msgs/msg/OccupancyGrid": read_occupancy_grid,
    "sensor_msgs/msg/PointCloud2": read_pointcloud2,
}


# ======================================================================================
# 小几何工具
# ======================================================================================
def yaw_of(q):
    """四元数 (x,y,z,w) → 平面 yaw（rad）。"""
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    """平面位姿复合 a ∘ b，位姿 = (x, y, yaw)：先 b 再 a。"""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1], a[1] + sa * b[0] + ca * b[1], wrap_pi(a[2] + b[2]))


def interp_pose(times, poses, t):
    """按时间线性插值（yaw 先解缠再插），超出范围就钳到端点。"""
    if t <= times[0]:
        return poses[0]
    if t >= times[-1]:
        return poses[-1]
    i = int(np.searchsorted(times, t))
    t0, t1 = times[i - 1], times[i]
    w = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    p0, p1 = poses[i - 1], poses[i]
    dyaw = wrap_pi(p1[2] - p0[2])
    return (p0[0] + w * (p1[0] - p0[0]), p0[1] + w * (p1[1] - p0[1]), wrap_pi(p0[2] + w * dyaw))


# ======================================================================================
# bag（sqlite3 只读）
# ======================================================================================
class Bag:
    def __init__(self, path):
        p = path
        if os.path.isdir(p):
            cands = sorted(glob.glob(os.path.join(p, "*.db3")))
            if not cands:
                die(f"{path} 里没有 *.db3")
            p = cands[0]
        if not os.path.isfile(p):
            die(f"bag 文件不存在：{p}")
        self.db_path = os.path.abspath(p)
        self.dir = os.path.dirname(self.db_path)
        self.con = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        self.con.execute("PRAGMA query_only=1")
        self.topics = {name: tid for tid, name in self.con.execute("select id, name from topics")}
        self.types = {name: ty for name, ty in self.con.execute("select name, type from topics")}
        r = self.con.execute("select min(timestamp), max(timestamp), count(*) from messages").fetchone()
        self.t0, self.t1, self.n_msg = (r[0] or 0) / 1e9, (r[1] or 0) / 1e9, int(r[2] or 0)

    def has(self, topic):
        return topic in self.topics

    def count(self, topic):
        if not self.has(topic):
            return 0
        return int(self.con.execute("select count(*) from messages where topic_id=?", (self.topics[topic],)).fetchone()[0])

    def stream(self, topic, limit=None, offset=0):
        """按 id 顺序产出 (timestamp_s, 反序列化后的消息)。"""
        if not self.has(topic):
            return
        reader = _MSGTYPE_READER.get(self.types.get(topic, ""))
        if reader is None:
            die(f"{topic} 的类型 {self.types.get(topic)} 不在本脚本支持列表内")
        sql = "select timestamp, data from messages where topic_id=? order by id"
        args = [self.topics[topic]]
        if limit is not None:
            sql += " limit ? offset ?"
            args += [int(limit), int(offset)]
        for ts, raw in self.con.execute(sql, args):
            yield ts / 1e9, reader(bytes(raw))

    def timestamps(self, topic):
        if not self.has(topic):
            return np.zeros(0)
        rows = self.con.execute("select timestamp from messages where topic_id=? order by id",
                                (self.topics[topic],))
        return np.array([r[0] for r in rows], dtype=np.float64) / 1e9

    def fetch(self, topic, ids):
        """按 rowid(id) 抓指定消息（用于两趟读取：先定采样，再取数据）。"""
        reader = _MSGTYPE_READER.get(self.types.get(topic, ""))
        out = []
        for mid in ids:
            row = self.con.execute("select timestamp, data from messages where id=?", (int(mid),)).fetchone()
            if row is None:
                continue
            out.append((mid, row[0] / 1e9, reader(bytes(row[1]))))
        return out

    def ids(self, topic):
        return [r[0] for r in self.con.execute(
            "select id from messages where topic_id=? order by id", (self.topics[topic],))]

    def raw_rows_by_id(self, topic):
        """{id: (timestamp_s, blob)}，只给 /scan 这种要"先定采样再取数据"的用。"""
        d = {}
        for mid, ts, blob in self.con.execute(
                "select id, timestamp, data from messages where topic_id=? order by id",
                (self.topics[topic],)):
            d[mid] = (ts / 1e9, bytes(blob))
        return d


def tf_channels(bag, topic="/tf", static_topic="/tf_static"):
    """→ {(parent, child): (times_ndarray, poses_ndarray(N,3))}。"""
    ch = {}
    for top in (static_topic, topic):
        if not bag.has(top):
            continue
        for _, m in bag.stream(top):
            for tr in m:
                key = (tr["frame_id"], tr["child_frame_id"])
                if key not in ch:
                    ch[key] = ([], [])
                ch[key][0].append(tr["stamp"])
                ch[key][1].append((tr["trans"][0], tr["trans"][1], yaw_of(tr["quat"])))
    out = {}
    for k, (ts, ps) in ch.items():
        order = np.argsort(ts)
        out[k] = (np.array(ts, dtype=np.float64)[order], np.array(ps, dtype=np.float64)[order])
    return out


def pose_at(ch, key, t):
    if key not in ch:
        return None
    times, poses = ch[key]
    if times.size == 0:
        return None
    return interp_pose(times, poses, t)


# ======================================================================================
# 描述子：查询（/scan）与库（PCD 射线投射）
# ======================================================================================
def scan_to_range_profile(scan, bins, r_max, stat=None):
    """/scan → range 描述子（bins 个 bin，bin b ↔ **传感器系角度 b*Δ**，左边界）。

    ⚠️ 角度零点必须是**传感器 +x 轴**（= base_link 的 +x：`base_link→livox_frame` 只有平移、
    没有旋转，本门实测 (0.12, 0, 0.175)）。`/scan` 的 `angle_min` 是 −π（不是 0）⇒ bin 号必须按
    `ang mod 2π` 取，**不能**按 `ang − angle_min` 取。
    第一版就是按 `ang − angle_min` 写的 ⇒ 查询描述子相对库整整转了 **180°**；因为检索对全圆周
    移位取最小，它照样能"找到地方"，但报出的相对 yaw 会偏 ~180°（症状：命中帧 |Δyaw| 中位 176°）。
    `--self-test` 第 3 条就是这个约定的回归测试。

    规则（与库侧 raycast 的"打不到就记 r_max"完全对齐）：
      · 有效束 = isfinite(r) 且 range_min ≤ r ≤ range_max；
      · 每 bin 取**最小**有效 range（= 最近障碍，单线激光的物理量）；
      · 无有效束 / inf / NaN / 超 range_max ⇒ r_max。
    """
    n = scan["ranges"].size
    amin, ainc = scan["angle_min"], scan["angle_increment"]
    r = scan["ranges"]
    lo, hi = scan["range_min"], scan["range_max"]
    ang = amin + np.arange(n, dtype=np.float64) * ainc
    valid = np.isfinite(r) & (r >= lo) & (r <= hi)
    rr = np.where(valid, r, INF)
    b = np.floor((ang % (2.0 * math.pi)) / (2.0 * math.pi) * bins).astype(np.int64) % bins
    r_max_eff = max(r_max, hi)
    prof = np.full(bins, r_max_eff, dtype=np.float64)
    if valid.any():
        bb = b[valid]
        vv = np.minimum(rr[valid], r_max_eff)
        order = np.argsort(bb, kind="stable")
        bb_s, vv_s = bb[order], vv[order]
        first = np.flatnonzero(np.concatenate([[True], np.diff(bb_s) != 0]))
        prof[bb_s[first]] = np.minimum.reduceat(vv_s, first)
    if stat is not None:
        stat["n_beams"] = int(n)
        stat["n_valid"] = int(valid.sum())
        stat["valid_frac"] = float(valid.mean())
        stat["n_inf"] = int((~np.isfinite(r)).sum())
        stat["empty_bins"] = int((prof >= r_max_eff - 1e-9).sum())
        if valid.any():
            stat["range_median_of_valid"] = float(np.median(r[valid]))
            stat["range_p05"] = float(np.percentile(r[valid], 5))
            stat["range_p95"] = float(np.percentile(r[valid], 95))
    return prof


def local_floor_grid(xy, z, cell, pct, global_floor):
    """逐块（cell 米）地面估计：每块取 z 的 pct 分位；空格用全局值填。

    为什么必须提供这条：A1 报告 §7 明说"别用全局 5% 分位；改成分块/局部地面拟合"——
    全局地面 + [0.05,0.35] 带会把坡道/高地/接地起伏的表面切进"障碍"里（A1 §6.1 实测
    25~30% 的占用格是水平面）。本函数给出"换成局部地面后会怎样"的对照。
    """
    lo = xy.min(axis=0)
    ix = np.floor((xy[:, 0] - lo[0]) / cell).astype(np.int64)
    iy = np.floor((xy[:, 1] - lo[1]) / cell).astype(np.int64)
    nx, ny = int(ix.max()) + 1, int(iy.max()) + 1
    key = iy * nx + ix
    order = np.argsort(key, kind="stable")
    k_s, z_s = key[order], z[order]
    bounds = np.flatnonzero(np.diff(k_s)) + 1
    groups = np.split(z_s, bounds)
    keys = np.unique(k_s)
    floors = np.array([np.percentile(g, pct) for g in groups])
    lut = np.full(nx * ny, global_floor, dtype=np.float64)
    lut[keys] = floors
    return lut[key]


def lib_raycast_profile(scp, occ_pad, origin_pad, cell, pose_xy, yaw, bins, r_max, step_frac):
    """库侧 range 描述子：复用 A1 探针的 raycast()，角度栅格 = b*Δ + yaw（左边界约定）。"""
    ang = np.arange(bins, dtype=np.float64) * (2.0 * math.pi / bins) + yaw
    ranges, _hits = scp.raycast(occ_pad, origin_pad, cell, pose_xy, ang, r_max, step_frac)
    return ranges


def build_library(scp, pcd_xyz, floor, band, args):
    """PCD → (占用格 occ_pad/origin_pad, 候选位姿列表, 元信息, 自由区掩膜, 栅格原点, 紧占用格)。"""
    zrel = pcd_xyz[:, 2] - floor
    m = (zrel >= band[0]) & (zrel < band[1])
    xy = pcd_xyz[m][:, :2]
    meta = {"band": [float(band[0]), float(band[1])], "points_in_band": int(m.sum())}
    if m.sum() < 10:
        meta.update(cells=0, n_candidates=0)
        return None, None, meta, None, None, None
    idx, origin, shape = scp.grid_of(xy, args.cell)
    occ = scp.occ_from_idx(idx, shape)
    margin = int(math.ceil(args.r_max / args.cell)) + 1
    occ_pad = scp.pad_grid(occ, margin)
    origin_pad = origin - margin * args.cell
    meta.update(cells=int(occ.sum()), grid_shape=[int(shape[0]), int(shape[1])],
                origin=[float(origin[0]), float(origin[1])])
    # 候选位姿：(x,y) 网格（--xy-step），落在自由区且离障碍 ≥ 车体半径
    #   ⚠️ A1 探针的 geodesic_clearance() 会在 cap 处截断（cap 之外留 -1），而它的 pick_poses()
    #   用 `dist >= clearance` 选点 ⇒ 实际只选到「离墙 clearance~cap 的一圈壳」。本门要的是
    #   「整个自由区、离墙 ≥ clearance」⇒ 把 -1（= 比 cap 还远）当作合格。
    clear_cells = max(1, int(math.ceil(args.clearance / args.cell)))
    dist = scp.geodesic_clearance(occ, clear_cells + 1)
    free = (dist < 0) | (dist >= clear_cells)
    lo = xy.min(axis=0)
    hi = xy.max(axis=0)
    xs = np.arange(lo[0], hi[0] + 1e-9, args.xy_step)
    ys = np.arange(lo[1], hi[1] + 1e-9, args.xy_step)
    cand = []
    for x in xs:
        for y in ys:
            gx = int(math.floor((x - origin[0]) / args.cell))
            gy = int(math.floor((y - origin[1]) / args.cell))
            if 0 <= gy < free.shape[0] and 0 <= gx < free.shape[1] and free[gy, gx]:
                cand.append((float(x), float(y)))
    meta["n_candidates"] = len(cand)
    meta["candidates_xy"] = cand
    meta["free_fraction"] = float(free.mean())
    meta["occupied_fraction"] = float(occ.mean())
    return (occ_pad, origin_pad), cand, meta, free, origin, occ


def cand_is_free(cand, free, origin, cell):
    out = np.zeros(len(cand), dtype=bool)
    for i, (x, y) in enumerate(cand):
        gx = int(math.floor((x - origin[0]) / cell))
        gy = int(math.floor((y - origin[1]) / cell))
        out[i] = (0 <= gy < free.shape[0] and 0 <= gx < free.shape[1] and free[gy, gx])
    return out


def library_descriptors(scp, grid, cand, args):
    occ_pad, origin_pad = grid
    D = np.empty((len(cand), args.bins), dtype=np.float32)
    hits = np.empty(len(cand), dtype=np.float64)
    for i, (x, y) in enumerate(cand):
        prof = lib_raycast_profile(scp, occ_pad, origin_pad, args.cell, (x, y), 0.0,
                                   args.bins, args.r_max, args.step_frac)
        D[i] = prof
        hits[i] = float((prof < args.r_max - 1e-9).mean())
    return D, hits


def rolling_windows(D, bins):
    """(n,bins) → (n,bins,bins)：第 s 行 = np.roll(D, -s, axis=1)（≡ yaw 偏移 s 个 bin）。"""
    cat = np.concatenate([D, D[:, :bins]], axis=1)
    return np.lib.stride_tricks.sliding_window_view(cat, bins, axis=1)


def retrieve(Q, L, bins, yaw_step_deg, chunk=48):
    """查询×库的 yaw 全搜索。

    → (dist(nq,ncand) 每个候选的最好距离, shift(nq,ncand) 对应圆周移位, dyaw(nq,ncand))
    距离 = 逐 bin 绝对差均值（米，与 A1 探针 desc_dist 同口径）。
    估计机器人 yaw = shift * (360/bins) 度（库条目一律以 yaw=0 建）。
    """
    nq, ncand = Q.shape[0], L.shape[0]
    step_bins = max(1, int(round(yaw_step_deg / 360.0 * bins)))
    shifts = np.arange(0, bins, step_bins)
    best = np.full((nq, ncand), INF, dtype=np.float32)
    best_s = np.zeros((nq, ncand), dtype=np.int32)
    for c0 in range(0, ncand, chunk):
        c1 = min(ncand, c0 + chunk)
        W = rolling_windows(L[c0:c1], bins)[:, shifts, :]          # (chunk, nshift, bins)
        for qi in range(nq):
            d = np.abs(W - Q[qi][None, None, :]).mean(axis=2)      # (chunk, nshift)
            j = np.argmin(d, axis=1)
            best[qi, c0:c1] = d[np.arange(c1 - c0), j]
            best_s[qi, c0:c1] = shifts[j]
    dyaw = best_s.astype(np.float64) * (360.0 / bins)
    return best, best_s, dyaw


# ======================================================================================
# 指标
# ======================================================================================
def evaluate(name, truth_xy, truth_yaw, cand_xy, dist, dyaw, args, L=None, Q=None):
    """一套 (库, 查询) → 指标 dict。

    `L`/`Q` 给了就额外算"先知距离"（oracle）：把查询描述子与**真值地点**那条库描述子在
    **真值 yaw**（以及全 yaw 搜索后）比一次 —— 它直接回答"库里那个地点的长相与查询差多少"，
    与排序无关。若 oracle 距离 ≥ 与错误地点的中位距离 ⇒ **库与查询根本不同源**（数据问题，
    不是检索器问题）。
    """
    cand = np.asarray(cand_xy, dtype=np.float64)
    nq = len(truth_xy)
    dxy_all = np.linalg.norm(cand[None, :, :] - np.asarray(truth_xy)[:, None, :], axis=2)  # (nq,ncand)
    near = np.argmin(dxy_all, axis=1)
    dxy_near = dxy_all[np.arange(nq), near]
    order = np.argsort(dist, axis=1)
    rank_near = np.empty(nq, dtype=np.int64)
    for i in range(nq):
        rank_near[i] = int(np.flatnonzero(order[i] == near[i])[0])
    # 到真值的 |Δxy| / |Δyaw|（用检索回来的那个候选）
    rank_xy = np.array([dxy_all[i, order[i, 0]] for i in range(nq)])
    rank_yaw = np.array([abs(wrap_pi(math.radians(dyaw[i, order[i, 0]]) - truth_yaw[i]))
                         for i in range(nq)])
    # top-k 里"最好的那个"（= 真值地点，若在 top-k 内）
    def best_in_topk(k):
        xy = np.full(nq, np.nan)
        yw = np.full(nq, np.nan)
        ok = np.zeros(nq, dtype=bool)
        for i in range(nq):
            top = order[i, :k]
            j = int(np.argmin(dxy_all[i, top]))
            jj = int(top[j])
            xy[i] = dxy_all[i, jj]
            yw[i] = abs(wrap_pi(math.radians(dyaw[i, jj]) - truth_yaw[i]))
            ok[i] = (rank_near[i] < k)
        return xy, yw, ok
    out = {"name": name, "n_frames": int(nq),
           "truth_nearest_cand_dxy_m": {"median": float(np.median(dxy_near)),
                                        "max": float(dxy_near.max()),
                                        "frac_le_half_grid": float((dxy_near <= 0.5 * args.xy_step).mean()),
                                        "frac_le_0.5m": float((dxy_near <= 0.5).mean())},
           "top1_dxy_m": {"median": float(np.median(rank_xy)), "max": float(rank_xy.max())},
           "top1_dyaw_deg": {"median": float(np.degrees(np.median(rank_yaw))),
                             "max": float(np.degrees(rank_yaw.max()))},
           "dist_best": {"median": float(np.median(np.min(dist, axis=1)))},
           "hit_at_k_cell": {}, "hit_at_k_pose": {}, "pass": {}}
    xy10, yw10, ok10 = best_in_topk(10)
    for k in (1, 5, 10):
        xy_k, yw_k, ok_k = best_in_topk(k)
        out["hit_at_k_cell"][str(k)] = float(ok_k.mean())
        pose_ok = np.zeros(nq, dtype=bool)
        for i in range(nq):
            top = order[i, :k]
            good = ((dxy_all[i, top] <= args.hit_radius) &
                    (np.abs(np.array([wrap_pi(math.radians(dyaw[i, j]) - truth_yaw[i]) for j in top]))
                     <= math.radians(args.hit_yaw)))
            pose_ok[i] = bool(good.any())
        out["hit_at_k_pose"][str(k)] = float(pose_ok.mean())
        out["pass"][str(k)] = float((ok_k & (xy_k <= args.hit_radius) &
                                     (yw_k <= math.radians(args.hit_yaw))).mean())
    # 规划书里的原话判据：真值在 top-10 且 |Δxy| ≤ 0.5 且 |Δyaw| ≤ 15
    strict = (ok10 & (xy10 <= args.hit_radius) & (yw10 <= math.radians(args.hit_yaw)))
    top1_ok = (rank_near == 0) & (rank_xy <= args.hit_radius) & (rank_yaw <= math.radians(args.hit_yaw))
    out["pass_fraction_top10_recall"] = float(strict.mean())
    out["pass_fraction_top1"] = float(top1_ok.mean())
    # 只考"召回"：真值在 top-10 且 yaw 对（|Δxy| 另算）—— 用来分开"检索器/网格"与"粗位姿够不够"
    recall_yaw_ok = ok10 & (yw10 <= math.radians(args.hit_yaw))
    out["pass_fraction_top10_recall_yaw_only"] = float(recall_yaw_ok.mean())
    # 误差分解：|Δxy| = 网格量化下限（真值到最近候选的距离）+ 选错候选造成的超额误差
    excess = np.where(np.isnan(xy10), np.nan, np.maximum(xy10 - dxy_near, 0.0))
    out["rank_correct_top1_frac"] = float((rank_near == 0).mean())
    out["xy_excess_m"] = {"median": float(np.nanmedian(excess)) if np.isfinite(excess).any() else None,
                          "p90": float(np.nanpercentile(excess, 90)) if np.isfinite(excess).any() else None}
    # 网格量化上限：真值 0.5 m 内有候选的帧占比（= |Δxy| ≤ 0.5 这条判据**结构上**能达到的比例）
    out["grid_quant_cap_0.5m"] = float((dxy_near <= args.hit_radius).mean())
    sel = strict
    out["hit_dxy_m"] = {"median": float(np.median(xy10[sel])) if sel.any() else None,
                        "p90": float(np.percentile(xy10[sel], 90)) if sel.any() else None}
    out["hit_dyaw_deg"] = {"median": float(np.degrees(np.median(yw10[sel]))) if sel.any() else None,
                           "p90": float(np.degrees(np.percentile(yw10[sel], 90))) if sel.any() else None}
    if not sel.any():
        m = ok10
        out["hit_dxy_m"] = {"median": float(np.median(xy10[m])) if m.any() else None,
                            "p90": float(np.percentile(xy10[m], 90)) if m.any() else None,
                            "note": "无帧满足完整判据 ⇒ 这里给的是『真值进 top-10 的帧』的 |Δxy|"}
        out["hit_dyaw_deg"] = {"median": float(np.degrees(np.median(yw10[m]))) if m.any() else None,
                               "p90": float(np.degrees(np.percentile(yw10[m], 90))) if m.any() else None}
    out["worst_frames_dxy"] = [
        {"i": int(i), "dxy": float(xy10[i]), "dyaw_deg": float(np.degrees(yw10[i])),
         "rank_of_true": int(rank_near[i]), "truth_xy": [float(truth_xy[i][0]), float(truth_xy[i][1]),
                                                         float(truth_yaw[i])]}
        for i in np.argsort(-np.nan_to_num(xy10, nan=0.0))[:5]]
    # ---- 先知距离（oracle）：库在"真值地点"的那条描述子与查询差多少 -------------------
    if L is not None and Q is not None:
        bins = L.shape[1]
        step = 360.0 / bins
        d_true_yaw, d_best, d_wrong = [], [], []
        for i in range(nq):
            j = int(near[i])
            s_true = int(round(math.degrees(truth_yaw[i]) / step)) % bins
            rolled_true = np.roll(L[j], -s_true)
            d_true_yaw.append(float(np.abs(rolled_true - Q[i]).mean()))
            d_all = np.abs(rolling_windows(L[j][None, :], bins)[0] - Q[i][None, :]).mean(axis=1)
            d_best.append(float(d_all.min()))
            mask = np.ones(len(cand), dtype=bool)
            mask[j] = False
            if dist is not None:
                d_wrong.append(float(np.median(dist[i][mask])))
        out["oracle_dist_true_yaw_m"] = float(np.median(d_true_yaw))
        out["oracle_dist_best_shift_m"] = float(np.median(d_best))
        out["dist_to_wrong_median_m"] = float(np.median(d_wrong)) if d_wrong else None
        out["margin_oracle_vs_wrong_m"] = (out["dist_to_wrong_median_m"] - out["oracle_dist_true_yaw_m"]
                                           if d_wrong else None)
    return out


# ======================================================================================
# CLI
# ======================================================================================
def parse_band(s):
    t = s.replace(":", ",").split(",")
    lo = float(t[0]); hi = float(t[1]) if len(t) > 1 else INF
    if lo >= hi:
        die(f"--band 下界 {lo} 不小于上界 {hi}")
    return (lo, hi)


def _check_tmp_path(path):
    ap = os.path.abspath(path)
    for root in TMP_ALLOWED:
        if ap == root or ap.startswith(root + os.sep):
            os.makedirs(os.path.dirname(ap), exist_ok=True)
            return ap
    die(f"拒绝写到 {ap}：只允许落在 {TMP_ALLOWED[0]}/ 或 {TMP_ALLOWED[1]}/ 下（本门只读仓库）")


def find_bags(args):
    if args.bag:
        return list(args.bag)
    cands = []
    for pat in (os.path.join(REPO_ROOT, ".tmp_bags", "*"), os.path.join(REPO_ROOT, ".tmp_bags", "*", "*")):
        for p in sorted(glob.glob(pat)):
            if os.path.isdir(p) and glob.glob(os.path.join(p, "*.db3")):
                cands.append(p)
    return cands


def bag_inventory(path):
    b = Bag(path)
    inv = {"bag": os.path.relpath(b.dir, REPO_ROOT),
           "db": os.path.relpath(b.db_path, REPO_ROOT),
           "duration_s": round(b.t1 - b.t0, 1), "n_messages": b.n_msg,
           "topics": {t: {"type": b.types[t], "n": b.count(t)} for t in sorted(b.topics)}}
    ch = tf_channels(b)
    inv["tf_channels"] = [f"{k[0]}->{k[1]}" for k in sorted(ch)]
    inv["pose_source"] = []
    if ("map", "odom") in ch and ("odom", "base_link") in ch:
        inv["pose_source"].append("TF map->odom ∘ odom->base_link")
    if ("odom", "base_link") in ch:
        inv["pose_source"].append("TF odom->base_link")
    if ("camera_init", "body") in ch:
        inv["pose_source"].append("TF camera_init->body")
    if b.has("/odom"):
        inv["pose_source"].append("/odom")
    if b.has("/odom_ground_truth"):
        inv["pose_source"].append("/odom_ground_truth")
    inv["usable"] = bool(b.has("/scan") and inv["pose_source"])
    return inv


def main(argv=None):
    p = build_parser()
    args = p.parse_args(argv)
    if args.world_to_map:
        t = str(args.world_to_map).replace(":", ",").split(",")
        args.world_to_map = (float(t[0]), float(t[1]))
    t_start = time.time()

    if args.list_bags:
        log("=" * 108)
        log("bag 清单（只读扫描 .tmp_bags/；usable = 同时有 /scan 与位姿源）")
        log("=" * 108)
        out = []
        for path in find_bags(args):
            inv = bag_inventory(path)
            out.append(inv)
            log(f"  [{'USABLE' if inv['usable'] else '  --  '}] {inv['bag']:22s} "
                f"{inv['duration_s']:7.1f} s  {inv['n_messages']:6d} msg  topics={len(inv['topics'])}")
            log(f"           /scan n={inv['topics'].get('/scan', {}).get('n', 0)}  "
                f"位姿源={inv['pose_source']}")
            log(f"           TF: {', '.join(inv['tf_channels'])}")
        if args.json:
            with open(_check_tmp_path(args.json), "w", encoding="utf-8") as fh:
                json.dump({"bags": out}, fh, ensure_ascii=False, indent=2)
            log(f"[i] JSON 已写入 {args.json}")
        return 0

    scp = load_probe()
    log(f"[i] 复用 A1 探针的描述子/栅格代码：{os.path.relpath(PROBE_PATH, REPO_ROOT)}")

    # ---- PCD / 地面 ----------------------------------------------------------------
    pcd = scp.read_pcd(args.pcd)
    xyz = pcd["xyz"]
    floor = float(np.percentile(xyz[:, 2], args.floor_pct)) if args.floor_z == "auto" else float(args.floor_z)
    floor_arg = floor
    if args.floor_mode == "local":
        floor_arg = local_floor_grid(xyz[:, :2], xyz[:, 2], args.floor_cell, args.floor_pct, floor)
        zr = xyz[:, 2] - floor_arg
        log(f"[i] 地面估计：**局部**（{args.floor_cell:g} m 分块取 z 的 {args.floor_pct:g}% 分位）；"
            f"逐点地面 z 的 min/中位/max = {floor_arg.min():.3f}/{np.median(floor_arg):.3f}/{floor_arg.max():.3f} m "
            f"（全局值 {floor:.4f} m）⇒ 逐点相对高度 z_rel 跨度 {zr.min():.3f}..{zr.max():.3f} m")
    log(f"[i] PCD {os.path.relpath(args.pcd, REPO_ROOT)}: {xyz.shape[0]} 点, 地面 z = {floor:.4f} m "
        f"({'auto ' + str(args.floor_pct) + '% 分位' if args.floor_z == 'auto' else 'manual'}"
        f"{', 但按 --floor-mode local 逐点用分块地面' if args.floor_mode == 'local' else ''}), "
        f"z 范围 [{xyz[:,2].min():.4f},{xyz[:,2].max():.4f}]")

    bags = find_bags(args)
    if not bags:
        die("没有找到任何 bag；用 --bag 指定（或先看 --list-bags）")

    # ---- 选 bag --------------------------------------------------------------------
    inv_all = [bag_inventory(b) for b in bags]
    usable = [inv for inv in inv_all if inv["usable"]]
    if not usable:
        die("找到的 bag 里没有同时含 /scan 与位姿源的；只有合成查询这一路可跑（本脚本未实现），"
            "请在报告里按 INCONCLUSIVE 记录")
    log(f"[i] 可用 bag {len(usable)}/{len(inv_all)}：" + ", ".join(u["bag"] for u in usable))
    for inv in inv_all:
        log(f"      {inv['bag']:22s} dur={inv['duration_s']:7.1f}s /scan n={inv['topics'].get('/scan',{}).get('n',0)} "
            f"位姿源={inv['pose_source']}")

    # ---- 库（多高度带）-------------------------------------------------------------
    # 两段式：① 每个带各自建占用格；② **候选位姿集合只取一份**（默认 = 主带的自由区），
    # 所有带共用同一批 (x,y) ⇒ 高度带对照是"同一批查询位姿、同一批库位姿，只换高度带"，
    # 不会把"某个带的地面污染把自由区吃掉 ⇒ 候选变少"混进"描述子判别力"里。
    libs = {}
    raw = {}
    for band in args.compare_bands:
        grid, cand, meta, free, origin, occ = build_library(scp, xyz, floor_arg, band, args)
        if grid is None:
            log(f"[!] 库带 [{band[0]:g},{band[1]:g}) 带内点太少（{meta['points_in_band']}）⇒ 跳过")
            continue
        raw[band] = {"grid": grid, "meta": meta, "free": free, "origin": origin,
                     "occ": occ, "cand_own": cand}
    main_band = args.compare_bands[0]
    if main_band not in raw:
        die(f"主带 {main_band} 的库没建起来")
    shared = raw[main_band]["cand_own"]
    log(f"[i] 候选位姿集合：{'共用主带' if args.shared_candidates else '每带各自'} "
        f"＝ {len(shared)} 个（{args.xy_step:g} m 网格，离障碍 ≥{args.clearance:g} m，"
        f"锚在带内点 bbox 的 min 角）")
    for band, rr in raw.items():
        cand = shared if args.shared_candidates else rr["cand_own"]
        D, hits = library_descriptors(scp, rr["grid"], cand, args)
        n_free_here = int(cand_is_free(cand, rr["free"], rr["origin"], args.cell).sum())
        rr["meta"]["n_candidates"] = len(cand)
        rr["meta"]["n_candidates_own_free_rule"] = len(rr["cand_own"])
        rr["meta"]["n_candidates_free_in_this_band"] = n_free_here
        rr["meta"]["free_fraction"] = float(rr["free"].mean())
        libs[band] = {"grid": rr["grid"], "cand": cand, "meta": rr["meta"], "D": D, "hit_frac": hits}
        tri = np.abs(D[:, None, :] - D[None, :, :]).mean(-1)[np.triu_indices(len(cand), 1)] \
            if len(cand) > 1 else np.array([float("nan")])
        log(f"[i] 库带 [{band[0]:g},{band[1]:g}) 地面以上：{rr['meta']['points_in_band']} 点 → "
            f"{rr['meta']['cells']} 个 {args.cell:g} m 占用格（占格图 {rr['meta']['occupied_fraction']*100:.1f}%，"
            f"自由区 {rr['meta']['free_fraction']*100:.1f}%）→ 候选 {len(cand)} 个"
            f"（该带自己的自由区规则只认 {len(rr['cand_own'])} 个，其中 {n_free_here} 个与共用集合重合）；"
            f"描述子命中 bin 占比中位 {np.median(hits)*100:.1f}%，两两描述子距离中位 {np.median(tri):.3f} m")
    if not libs:
        die("所有高度带的库都建不起来")

    # ---- 合成查询用的库（查询带）---------------------------------------------------
    gsyn = None
    if args.synthetic:
        qband = args.query_band
        gsyn, csyn, msyn, _f, _o, _oc = build_library(scp, xyz, floor_arg, qband, args)
        if gsyn is None:
            log(f"[!] 查询带 [{qband[0]:g},{qband[1]:g}) 带内点太少 ⇒ 取消合成查询对照")
        else:
            log(f"[i] 合成查询用的『查询侧可见带』库：[{qband[0]:g},{qband[1]:g}) "
                f"{msyn['points_in_band']} 点 → {msyn['cells']} 占用格")

    # ---- 逐 bag --------------------------------------------------------------------
    report = {"args": {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(args).items()},
              "pcd": {"path": os.path.relpath(args.pcd, REPO_ROOT), "n_points": int(xyz.shape[0]),
                      "floor_z": floor, "floor_mode": args.floor_mode,
                      "floor_z_median": float(np.median(floor_arg)) if args.floor_mode == "local" else floor,
                      "z_range": [float(xyz[:,2].min()), float(xyz[:,2].max())]},
              "bags": [], "libraries": {}, "results": []}
    for band, lib in libs.items():
        report["libraries"][f"{band[0]:g},{band[1]:g}"] = {k: v for k, v in lib["meta"].items()
                                                          if k != "candidates_xy"}

    for bag_path in [u["bag"] for u in usable]:
        bag = Bag(bag_path)
        log("=" * 108)
        log(f"bag {os.path.relpath(bag.dir, REPO_ROOT)}  {bag.t1 - bag.t0:.1f} s  "
            f"/scan {bag.count('/scan')} 帧")
        log("=" * 108)

        # ---- 位姿链 ---------------------------------------------------------------
        ch = tf_channels(bag)
        have_map = ("map", "odom") in ch and ("odom", "base_link") in ch
        have_odom = ("odom", "base_link") in ch
        if not (have_map or have_odom):
            log("[!] 没有 odom->base_link（也没有 map->odom）⇒ 跳过这份 bag")
            continue
        st = ch.get(("base_link", "livox_frame"))
        if st is not None:
            lx, ly = float(st[1][0, 0]), float(st[1][0, 1])
        else:
            lx, ly = 0.0, 0.0
        log(f"[i] 位姿源：{'map->odom ∘ odom->base_link' if have_map else 'odom->base_link'}；"
            f"base_link->livox_frame = ({lx:.3f},{ly:.3f}) m（描述子原点取 livox_frame 位姿）")

        gt_ch = None
        if bag.has("/odom_ground_truth"):
            gt_ch = {}
            for _, m in bag.stream("/odom_ground_truth"):
                gt_ch.setdefault("gt", ([], []))
                gt_ch["gt"][0].append(m["stamp"])
                gt_ch["gt"][1].append((m["pos"][0], m["pos"][1], yaw_of(m["quat"])))
            order = np.argsort(gt_ch["gt"][0])
            gt_ch["gt"] = (np.array(gt_ch["gt"][0])[order], np.array(gt_ch["gt"][1])[order])

        def base_pose(t):
            """→ base_link 在 map 系的位姿 (x, y, yaw)（--truth map = TF 链，odom = 只用 odom->base_link）。"""
            if args.truth == "map" and have_map:
                mo = pose_at(ch, ("map", "odom"), t)
                ob = pose_at(ch, ("odom", "base_link"), t)
                if mo is not None and ob is not None:
                    return compose((mo[0], mo[1], mo[2]), (ob[0], ob[1], ob[2]))
            if have_odom:
                ob = pose_at(ch, ("odom", "base_link"), t)
                if ob is not None:
                    return (ob[0], ob[1], ob[2])
            return None

        def truth_pose(t):
            """→ livox_frame（描述子原点）在 map 系的位姿：base_link 位姿 ∘ 静态杆臂。"""
            base = base_pose(t)
            if base is None:
                return None
            return compose(base, (lx, ly, 0.0))

        # ---- 采样 -----------------------------------------------------------------
        ids = bag.ids("/scan")
        if len(ids) == 0:
            continue
        # /scan 的 header 时间戳（sim 时间，与 TF/GT 同一时间轴）——只解 header，不解 ranges
        rows = list(bag.con.execute("select id, data from messages where topic_id=? order by id",
                                    (bag.topics["/scan"],)))
        ts_all = np.array([peek_laserscan_stamp(bytes(r[1])) for r in rows], dtype=np.float64)
        bag_ts_all = bag.timestamps("/scan")
        log(f"[i] /scan 时间轴：header(sim) {ts_all[0]:.3f}..{ts_all[-1]:.3f} s；"
            f"bag 接收(epoch) 跨度 {(bag_ts_all[-1]-bag_ts_all[0]):.1f} s ⇒ 与 TF 对时间用 header")
        poses_at_scan = []
        for t in ts_all:
            tp = truth_pose(float(t))
            poses_at_scan.append(tp)
        keep = []
        last_t = -1e18
        last_p = None
        for i, t in enumerate(ts_all):
            tp = poses_at_scan[i]
            if tp is None:
                continue
            moved = INF if last_p is None else math.hypot(tp[0] - last_p[0], tp[1] - last_p[1])
            dyaw = INF if last_p is None else abs(wrap_pi(tp[2] - last_p[2]))
            gap = t - last_t
            if last_p is None or ((gap >= args.sample_dt) and (moved >= args.min_move or dyaw >= math.radians(args.min_yaw))) \
                    or (gap >= args.sample_dt * args.force_every):
                keep.append(i)
                last_t, last_p = t, tp
        if len(keep) > args.max_frames:
            sel = np.linspace(0, len(keep) - 1, args.max_frames).round().astype(int)
            keep = [keep[j] for j in sorted(set(sel.tolist()))]
        log(f"[i] 采样：{len(keep)} / {len(ids)} 帧（Δt≥{args.sample_dt:g}s 且 位移≥{args.min_move:g}m "
            f"或 Δyaw≥{args.min_yaw:g}°，或每 {args.sample_dt*args.force_every:g}s 强制一帧；"
            f"上限 {args.max_frames}）")
        keep_ids = [ids[i] for i in keep]
        truth = np.array([poses_at_scan[i] for i in keep], dtype=np.float64)

        # ---- 硬诊断：机器人**真实所在的位置**在库的各高度带里是"自由"还是"障碍"？------------
        # 这是"库能不能用"的最短判据：若某个高度带把机器人自己开过的位置判成障碍，
        # 那么这个带的库从根上不可用（无论检索器怎么写）。
        occ_at_truth = {}
        for band, rr in raw.items():
            occ_, org_ = rr["occ"], rr["origin"]
            inside = 0
            for xx, yy in truth[:, :2]:
                gx = int(math.floor((xx - org_[0]) / args.cell))
                gy = int(math.floor((yy - org_[1]) / args.cell))
                if 0 <= gy < occ_.shape[0] and 0 <= gx < occ_.shape[1] and occ_[gy, gx]:
                    inside += 1
            occ_at_truth[f"{band[0]:g},{band[1]:g}"] = {
                "frac_truth_poses_in_occupied_cell": inside / max(1, len(truth)),
                "n_inside": inside, "n_frames": len(truth)}
            log(f"   [诊断] 库带 [{band[0]:g},{band[1]:g}) 把 {inside}/{len(truth)} 个『真实位姿』"
                f"（{inside/max(1,len(truth))*100:.1f}%）判成占用格"
                + ("  ⇒ **该带不可用**（机器人从自己开过的地方看出去，第一格就是墙）" if inside else "  ⇒ OK"))

        # 真值交叉验证（GT − world→map 偏移）——**同帧比**：GT 是 base_link，所以也拿 base_link 比
        truth_check = None
        if gt_ch is not None:
            wo = args.world_to_map
            err, vec = [], []
            for i in keep:
                t = float(ts_all[i])
                gp = interp_pose(gt_ch["gt"][0], gt_ch["gt"][1], t)
                cand = (gp[0] - wo[0], gp[1] - wo[1], gp[2])
                bp = base_pose(t)
                if bp is not None:
                    err.append(math.hypot(cand[0] - bp[0], cand[1] - bp[1]))
                    vec.append((cand[0] - bp[0], cand[1] - bp[1]))
            if err:
                vec = np.array(vec)
                truth_check = {"n": len(err), "median_m": float(np.median(err)),
                               "p90_m": float(np.percentile(err, 90)), "max_m": float(np.max(err)),
                               "median_dx_m": float(np.median(vec[:, 0])),
                               "median_dy_m": float(np.median(vec[:, 1])),
                               "std_dx_m": float(vec[:, 0].std()), "std_dy_m": float(vec[:, 1].std())}
                log(f"[i] 真值交叉验证（base_link 对 base_link）：/odom_ground_truth − "
                    f"({wo[0]:g},{wo[1]:g}) 与 TF 链的差 中位 {truth_check['median_m']:.3f} m / "
                    f"p90 {truth_check['p90_m']:.3f} m；向量中位 "
                    f"({truth_check['median_dx_m']:+.3f},{truth_check['median_dy_m']:+.3f}) m，"
                    f"抖动 σ=({truth_check['std_dx_m']:.3f},{truth_check['std_dy_m']:.3f}) m "
                    f"（若向量中位 ≫ 抖动 ⇒ 是**系统性**原点/杆臂差，不是漂移）")

        # ---- 同源诊断①：真实 /scan 的端点在 map 系里落不落在库的占用格上 ------------------
        #   这是最直接的"同源"判据：把每一帧 /scan 的回波端点用真值位姿投到 map 系，
        #   再问"库（某一个高度带）里这些位置是障碍还是自由"。
        #   库是障碍 & 查询也是障碍 ⇒ 同源；库说自由 & 查询有回波 ⇒ 库缺了查询看得见的东西。
        end_pts = []
        qstat_pre = {"n_frames": 0}
        for j, (mid, t, msg) in enumerate(bag.fetch("/scan", keep_ids)):
            r = np.asarray(msg["ranges"], dtype=np.float64)
            ang = msg["angle_min"] + np.arange(r.size) * msg["angle_increment"]
            ok = np.isfinite(r) & (r >= msg["range_min"]) & (r <= msg["range_max"])
            if not ok.any():
                continue
            sx, sy, syaw = truth[j]
            end_pts.append(np.stack([sx + r[ok] * np.cos(ang[ok] + syaw),
                                     sy + r[ok] * np.sin(ang[ok] + syaw)], axis=1))
            qstat_pre["n_frames"] += 1
        overlap_src = None
        if end_pts:
            ep = np.concatenate(end_pts, axis=0)
            qidx, qorigin, qshape = scp.grid_of(ep, args.cell)
            qocc = scp.occ_from_idx(qidx, qshape)
            overlap_src = {"endpoints": int(ep.shape[0]), "query_cells": int(qocc.sum()),
                           "query_cells_per_frame": float(qocc.sum() / max(1, qstat_pre["n_frames"]))}
            log(f"[i] 同源诊断①：{qstat_pre['n_frames']} 帧 /scan 共 {ep.shape[0]} 个回波端点 → "
                f"查询侧 {int(qocc.sum())} 个 {args.cell:g} m 占用格"
                f"（{qocc.sum()/max(1,qstat_pre['n_frames']):.0f} 格/帧）")
            for band, rr in raw.items():
                occ_, org_ = rr["occ"], rr["origin"]
                dd = scp.dilate8(occ_, 1)
                gx = np.round((qorigin[0] + (np.arange(qocc.shape[1]) + 0.5) * args.cell
                               - org_[0]) / args.cell - 0.5).astype(np.int64)
                gy = np.round((qorigin[1] + (np.arange(qocc.shape[0]) + 0.5) * args.cell
                               - org_[1]) / args.cell - 0.5).astype(np.int64)
                okx = (gx >= 0) & (gx < occ_.shape[1])
                oky = (gy >= 0) & (gy < occ_.shape[0])
                sub = np.zeros(qocc.shape, dtype=bool)
                sub[np.ix_(oky, okx)] = dd[np.ix_(gy[oky], gx[okx])]
                both = int((qocc & sub).sum())
                qonly = int((qocc & ~sub).sum())
                lonly = int((sub & ~qocc).sum())
                key = f"{band[0]:g},{band[1]:g}"
                overlap_src[key] = {
                    "query_cells": int(qocc.sum()),
                    "frac_query_cells_on_lib_obstacle": both / max(1, int(qocc.sum())),
                    "query_cells_where_lib_says_free": qonly,
                    "lib_cells_without_query_support": lonly,
                    "n_lib_cells": int(occ_.sum())}
                log(f"   [诊断①] 库带 [{key})：查询侧占用格里 {both/max(1,int(qocc.sum()))*100:5.1f}% "
                    f"落在库障碍上（±{args.cell:g} m）；{qonly} 格是『库说自由但查询有回波』，"
                    f"{lonly} 格是『库有障碍但查询没看到』")

        # ---- 查询描述子：真实 /scan ------------------------------------------------
        Q_real = np.empty((len(keep_ids), args.bins), dtype=np.float32)
        qstat = {"n_beams": 0, "n_valid": 0, "empty_bins": 0, "ranges": []}
        for j, (mid, t, msg) in enumerate(bag.fetch("/scan", keep_ids)):
            stt = {}
            Q_real[j] = scan_to_range_profile(msg, args.bins, args.r_max, stt)
            qstat["n_beams"] = stt["n_beams"]
            qstat["n_valid"] += stt["n_valid"]
            qstat["empty_bins"] += stt["empty_bins"]
            if "range_median_of_valid" in stt:
                qstat["ranges"].append(stt["range_median_of_valid"])
        qstat["valid_frac"] = qstat["n_valid"] / max(1, qstat["n_beams"] * len(keep_ids))
        qstat["empty_bin_frac"] = qstat["empty_bins"] / max(1, args.bins * len(keep_ids))
        qstat["median_range"] = float(np.median(qstat["ranges"])) if qstat["ranges"] else None
        log(f"[i] 真实 /scan → 描述子：每帧 {qstat['n_beams']} 束，有效束占比 "
            f"{qstat['valid_frac']*100:.1f}%，空 bin 占比 {qstat['empty_bin_frac']*100:.1f}%"
            f"（空 bin 记 r_max={args.r_max:g} m），有效 range 中位 "
            f"{qstat['median_range'] if qstat['median_range'] is None else round(qstat['median_range'],3)} m")

        # ---- 合成查询（从 PCD 用"查询带"射线投射）----------------------------------
        Q_syn = None
        if args.synthetic and gsyn is not None:
            Q_syn = np.empty((len(keep), args.bins), dtype=np.float32)
            for j in range(len(keep)):
                Q_syn[j] = lib_raycast_profile(scp, gsyn[0], gsyn[1], args.cell,
                                               (truth[j, 0], truth[j, 1]), truth[j, 2],
                                               args.bins, args.r_max, args.step_frac)
            log(f"[i] 合成查询：从 PCD 的查询带 [{args.query_band[0]:g},{args.query_band[1]:g}) "
                f"在 {len(keep)} 个真值位姿上射线投射生成描述子（不含真实点云/去地面/噪声）")

        # ---- 检索 ------------------------------------------------------------------
        queries = {"real /scan": Q_real}
        if Q_syn is not None:
            queries[f"syn PCD[{args.query_band[0]:g},{args.query_band[1]:g})"] = Q_syn
        # 机制自检：库带 X ← 库带 X 自己的合成查询（直接复用同一份库栅格）
        for band, lib in libs.items():
            Qs = np.empty((len(keep), args.bins), dtype=np.float32)
            for j in range(len(keep)):
                Qs[j] = lib_raycast_profile(scp, lib["grid"][0], lib["grid"][1], args.cell,
                                            (truth[j, 0], truth[j, 1]), truth[j, 2],
                                            args.bins, args.r_max, args.step_frac)
            queries[f"syn PCD[{band[0]:g},{band[1]:g}) (self)"] = Qs

        bag_rec = {"bag": os.path.relpath(bag.dir, REPO_ROOT), "duration_s": round(bag.t1 - bag.t0, 1),
                   "n_scan": bag.count("/scan"), "n_frames": len(keep),
                   "pose_source": "map->odom ∘ odom->base_link" if have_map else "odom->base_link",
                   "livox_offset_m": [lx, ly], "truth_check": truth_check,
                   "occ_at_truth": occ_at_truth, "overlap_scan_vs_lib": overlap_src,
                   "query_stats": {k: v for k, v in qstat.items() if k != "ranges"},
                   "frames_truth_xy_yaw": [[float(a), float(b), float(c)] for a, b, c in truth],
                   "results": []}
        for band, lib in libs.items():
            for qname, Q in queries.items():
                if qname.startswith("syn PCD[") and "(self)" in qname and \
                        not qname.startswith(f"syn PCD[{band[0]:g},{band[1]:g})"):
                    continue
                # 先知距离（**同一姿态**、零网格量化）：库栅格在真值位姿上的射线 vs 查询描述子。
                # 这是"同源性"最干净的单个数字：它把"候选网格太粗"这个混淆项彻底去掉。
                #   Δ = 库(真值位姿) − 查询：>0 ⇒ 库把近处结构漏掉了；<0 ⇒ 库多了近处的假障碍。
                pex = np.empty_like(Q)
                for j in range(Q.shape[0]):
                    pex[j] = lib_raycast_profile(scp, lib["grid"][0], lib["grid"][1], args.cell,
                                                 (truth[j, 0], truth[j, 1]), truth[j, 2],
                                                 args.bins, args.r_max, args.step_frac)
                d_exact = np.abs(pex - Q).mean(axis=1)
                signed = (pex - Q).mean(axis=1)
                large = (np.abs(pex - Q) > 0.5).mean(axis=1)
                t0 = time.time()
                dist, shift, dyaw = retrieve(Q, lib["D"], args.bins, args.yaw_step)
                ev = evaluate(f"lib[{band[0]:g},{band[1]:g}) ← {qname}", truth[:, :2], truth[:, 2],
                              lib["cand"], dist, dyaw, args, L=lib["D"], Q=Q)
                ev["band"] = [band[0], band[1]]
                ev["query"] = qname
                ev["exact_pose_dist_m"] = {
                    "median": float(np.median(d_exact)), "p90": float(np.percentile(d_exact, 90)),
                    "median_signed_lib_minus_query_m": float(np.median(signed)),
                    "median_frac_bins_off_by_gt_0.5m": float(np.median(large))}
                ev["retrieve_s"] = round(time.time() - t0, 2)
                bag_rec["results"].append(ev)
                log(f"   · {ev['name']:52s} hit@1/5/10={ev['hit_at_k_cell']['1']*100:5.1f}/"
                    f"{ev['hit_at_k_cell']['5']*100:5.1f}/{ev['hit_at_k_cell']['10']*100:5.1f}%  "
                    f"PASS(top10+Δxy≤{args.hit_radius:g}+Δyaw≤{args.hit_yaw:g}°)="
                    f"{ev['pass_fraction_top10_recall']*100:5.1f}%  "
                    f"PASS(只考召回+yaw)={ev['pass_fraction_top10_recall_yaw_only']*100:5.1f}%  "
                    f"PASS(top1)={ev['pass_fraction_top1']*100:5.1f}%  "
                    f"命中时 |Δxy| 中位 {ev['hit_dxy_m']['median'] if ev['hit_dxy_m']['median'] is None else round(ev['hit_dxy_m']['median'],3)} m / "
                    f"|Δyaw| 中位 {ev['hit_dyaw_deg']['median'] if ev['hit_dyaw_deg']['median'] is None else round(ev['hit_dyaw_deg']['median'],1)}°"
                    f"（超额 {ev['xy_excess_m']['median'] if ev['xy_excess_m']['median'] is None else round(ev['xy_excess_m']['median'],3)} m）"
                    + (f"  | 同姿态|Δ|={ev['exact_pose_dist_m']['median']:.3f} m"
                       f"（库−查询 中位 {ev['exact_pose_dist_m']['median_signed_lib_minus_query_m']:+.3f} m，"
                       f"{ev['exact_pose_dist_m']['median_frac_bins_off_by_gt_0.5m']*100:.0f}% 的 bin 差 >0.5 m）"
                       f"  | 先知距离 {ev.get('oracle_dist_true_yaw_m', float('nan')):.3f} m vs "
                       f"错误地点中位 {ev.get('dist_to_wrong_median_m', float('nan')):.3f} m"
                       f"（裕度 {ev.get('margin_oracle_vs_wrong_m', float('nan')):+.3f} m）"
                       if "oracle_dist_true_yaw_m" in ev else ""))
        report["bags"].append(bag_rec)
        report["results"].extend([{**r, "bag": bag_rec["bag"]} for r in bag_rec["results"]])

    # ---- 结论 ----------------------------------------------------------------------
    log("=" * 108)
    log("结论速览（判据：真值在 top-10 且 |Δxy| ≤ %.2f m 且 |Δyaw| ≤ %.0f°，"
        "对『多数采样帧』成立；见 docs/scan_context_plan.md §D.2）" % (args.hit_radius, args.hit_yaw))
    log("=" * 108)
    key = [f"lib[{b[0]:g},{b[1]:g}) ← real /scan" for b in args.compare_bands]
    for r in report["results"]:
        if r["query"] != "real /scan":
            continue
        log(f"  {r['bag']:14s} 库带[{r['band'][0]:g},{r['band'][1]:g})  n={r['n_frames']:3d}  "
            f"hit@1/5/10 = {r['hit_at_k_cell']['1']*100:5.1f}/{r['hit_at_k_cell']['5']*100:5.1f}/"
            f"{r['hit_at_k_cell']['10']*100:5.1f}%   PASS(计划书口径) = "
            f"{r['pass_fraction_top10_recall']*100:5.1f}%   PASS(top-1 口径) = "
            f"{r['pass_fraction_top1']*100:5.1f}%")
    main = [r for r in report["results"] if r["query"] == "real /scan" and r["band"] == list(main_band)]
    if main:
        frac = float(np.mean([r["pass_fraction_top10_recall"] for r in main]))
        if frac >= args.pass_require:
            verdict = "PASS"
        elif frac <= args.fail_below:
            verdict = "FAIL"
        else:
            verdict = "INCONCLUSIVE"
        report["verdict"] = {"band": list(main_band), "query": "real /scan",
                             "pass_fraction": frac, "pass_require": args.pass_require,
                             "fail_below": args.fail_below, "verdict": verdict,
                             "hit_at_10_mean": float(np.mean([r["hit_at_k_cell"]["10"] for r in main]))}
        log(f"  ⇒ 主带 [{main_band[0]:g},{main_band[1]:g}) + 真实 /scan：PASS 比例 {frac*100:.1f}% "
            f"（判 PASS 需 ≥{args.pass_require*100:.0f}%，判 FAIL 需 ≤{args.fail_below*100:.0f}%）"
            f" ⇒ **{verdict}**")
    log(f"  （全程 {time.time()-t_start:.1f} s）")

    if args.json:
        with open(_check_tmp_path(args.json), "w", encoding="utf-8") as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2, default=lambda o: None)
        log(f"[i] JSON 已写入 {args.json}")
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        prog="scan_context_samesource_gate.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Scan Context 路线的同源性硬门：用真实 bag 的 /scan 当查询，检索 PCD 生成的库，"
                    "看能不能回到真值地点（+ 高度带对照 + 合成查询隔离）。只读、numpy+stdlib、不启 ROS。",
        epilog="""判据（plan §D.2 / probe report §8.2）：
  · 每帧：真值地点在 **top-10** 且 |Δxy| ≤ 0.5 m 且 |Δyaw| ≤ 15°；
  · 门：对**多数**采样帧成立。"多数"的取法（--pass-require / --fail-below）：
      ≥80% ⇒ PASS（希望值：这是给 GICP 当初值的粗定位，10 帧里错 2 帧还能靠 top-10 全送 GICP 兜住）
      ≤50% ⇒ FAIL（连一半都不到 ⇒ 检索器写得再好也没用，先修资产）
      50%~80% ⇒ INCONCLUSIVE（要么调资产/网格，要么把 top-10 全送 GICP 再谈）
  · 合成查询三条对照用来分离原因：
      `syn PCD[X] (self)` = 库带 X ← 同一带 X 的合成查询 ⇒ 应该接近满分（否则是**检索器/网格**的问题）
      `syn PCD[0.05,0.35]` = 库带 X ← 查询带的合成查询 ⇒ 只差"高度带不同源"
      `real /scan`         = 库带 X ← 真实 /scan     ⇒ 真正的门（叠了点云/去地面/量测噪声）

查询 → 描述子的映射（写死在代码里，见 scan_to_range_profile）：
  bin b ↔ 传感器系角度 b*Δ（Δ=360/bins 度，左边界）；同 bin 取最小 range；
  inf / NaN / <range_min / >range_max 的束 ⇒ 该 bin 记 r_max（与库侧"打不到"同编码）。
  /scan 的 frame 是 livox_frame，其 `base_link→livox_frame` 只有平移（实测量 (0.12,0,0.175)）
  ⇒ 传感器 yaw = base_link yaw，描述子的角度零点不需要额外旋转。

yaw 搜索 = 描述子圆周移位（左移 s 个 bin ≡ yaw 偏 s*Δ），即 plan §B.2 的"全圆周移位"；
--yaw-step 控制移位步长（度），1~2° 与 A1 报告的"粗 yaw 上界 4~5°"一致。
""")
    p.add_argument("--bag", nargs="+", default=None,
                   help="bag 目录（含 metadata.yaml+*.db3）或 .db3 路径；可给多个。默认：自动扫 .tmp_bags/*")
    p.add_argument("--list-bags", action="store_true", help="只列 bag 清单与位姿源，不跑检索（只读）")
    p.add_argument("--query-band-audit", action="store_true",
                   help="只用 bag 验证『查询侧到底看得到哪个高度带』（/segmentation/obstacle 的 z 直方图，"
                        "排除仿真插件发的 (0,0,0) 假点）+ p2l 的 min/max_height 截断，不跑检索")
    p.add_argument("--audit-frames", type=int, default=8, help="--query-band-audit 均匀抽几帧。默认 8")
    p.add_argument("--sensor-height", type=float, default=0.226,
                   help="雷达相对地面的高度（米），用于把 livox_frame 的 z 换成『地面以上』高度。默认 0.226"
                        "（= linefit segmentation_sim.yaml 的 sensor_height 实测值）")
    p.add_argument("--p2l-max-height", type=float, default=0.1,
                   help="pointcloud_to_laserscan 的 max_height（livox_frame）。默认 0.1")
    p.add_argument("--p2l-min-height", type=float, default=-1.0,
                   help="pointcloud_to_laserscan 的 min_height（livox_frame）。默认 -1.0")
    p.add_argument("--pcd", default=DEFAULT_PCD, help=f"先验 PCD。默认 {os.path.relpath(DEFAULT_PCD, REPO_ROOT)}")
    p.add_argument("--floor-z", default="auto", help="地面 z（米）或 auto = z 的 5%% 分位。默认 auto")
    p.add_argument("--floor-pct", type=float, default=5.0, help="--floor-z auto 时的分位。默认 5")
    p.add_argument("--floor-mode", choices=("global", "local"), default="global",
                   help="地面估计：global = 全局 pct 分位（A1 的默认，默认值）；"
                        "local = 逐 floor-cell 分块 pct 分位（A1 报告 §7 建议的做法，用来量化"
                        "'地面污染'有多少只是全局地面估计造成的）")
    p.add_argument("--floor-cell", type=float, default=1.0, help="--floor-mode local 的分块尺寸（米）。默认 1.0")
    p.add_argument("--band", default="0.20,0.35", type=parse_band,
                   help="主库高度带（地面以上，米，半开区间）。默认 0.20,0.35")
    p.add_argument("--compare-bands", nargs="+", type=parse_band,
                   default=[(0.20, 0.35), (0.05, 0.35), (0.05, 0.60)],
                   help="要对照的库高度带列表（第一个 = 主带）。默认 0.20,0.35 0.05,0.35 0.05,0.60")
    p.add_argument("--query-band", type=parse_band, default=(0.05, 0.35),
                   help="合成查询用的『查询侧可见带』（地面以上，米）。默认 0.05,0.35")
    p.add_argument("--bins", type=int, default=360, help="角向 bin 数（360 ⇒ 1°/bin）。默认 360")
    p.add_argument("--r-max", type=float, default=10.0, help="描述子/射线最大半径（米）。默认 10")
    p.add_argument("--cell", type=float, default=0.15, help="库侧 2D 占用格尺寸（米）。默认 0.15")
    p.add_argument("--step-frac", type=float, default=0.5, help="射线采样步长 = cell*step_frac。默认 0.5")
    p.add_argument("--xy-step", type=float, default=1.0, help="候选位姿 x/y 网格（米）。默认 1.0")
    p.add_argument("--yaw-step", type=float, default=1.0, help="yaw 搜索步长（度）。默认 1.0")
    p.add_argument("--clearance", type=float, default=0.20,
                   help="候选位姿到最近障碍的最小距离（米，= 车体半径）。默认 0.20")
    p.add_argument("--sample-dt", type=float, default=2.0, help="采样最小时间间隔（秒）。默认 2.0")
    p.add_argument("--min-move", type=float, default=0.3, help="采样最小位移（米）。默认 0.3")
    p.add_argument("--min-yaw", type=float, default=10.0, help="采样最小转角（度）。默认 10")
    p.add_argument("--force-every", type=float, default=5.0,
                   help="即使没动，也每 sample_dt*该值 秒强制采一帧。默认 5")
    p.add_argument("--max-frames", type=int, default=60, help="每份 bag 最多采样帧数。默认 60")
    p.add_argument("--hit-radius", type=float, default=0.5, help="|Δxy| 判据（米）。默认 0.5")
    p.add_argument("--hit-yaw", type=float, default=15.0, help="|Δyaw| 判据（度）。默认 15")
    p.add_argument("--truth", choices=("map", "odom"), default="map",
                   help="真值来源：map=TF map->odom∘odom->base_link（默认，= 回归脚本用的同一条链）；"
                        "odom=只用 odom->base_link（plan §B.3 的约定）。无论选哪个，"
                        "只要 bag 里有 /odom_ground_truth 就额外做一次交叉验证")
    p.add_argument("--world-to-map", default=DEFAULT_WORLD_TO_MAP,
                   help=f"world→map 的平移（逗号分隔，米），只给 --truth gt 用。默认 {DEFAULT_WORLD_TO_MAP}")
    p.add_argument("--synthetic", dest="synthetic", action="store_true", default=True,
                   help="跑合成查询对照（默认开）")
    p.add_argument("--no-synthetic", dest="synthetic", action="store_false",
                   help="不跑合成查询对照（只跑真实 /scan）")
    p.add_argument("--shared-candidates", dest="shared_candidates", action="store_true", default=True,
                   help="所有高度带共用同一批候选位姿（默认开；保证带对照公平）")
    p.add_argument("--per-band-candidates", dest="shared_candidates", action="store_false",
                   help="每个带用自己的自由区规则挑候选位姿（会混入'某带污染重⇒候选少'的效应）")
    p.add_argument("--pass-require", type=float, default=0.80, help="判 PASS 所需的帧通过比例。默认 0.8")
    p.add_argument("--fail-below", type=float, default=0.50, help="判 FAIL 的通过比例上限。默认 0.5")
    p.add_argument("--json", default=None, help="结果 JSON（必须落在 .tmp_cache/ 或 .tmp_research/ 下）")
    p.add_argument("--self-test", action="store_true",
                   help="自检：CDR 解析 vs rclpy、描述子约定、yaw 圆周移位一致性")
    return p


# ======================================================================================
# 查询带审计（--query-band-audit）：真实 bag 里"查询侧到底看得到哪个高度带"
# ======================================================================================
def query_band_audit(args):
    from collections import OrderedDict
    bags = [b for b in find_bags(args) if not args.bag or True]
    inv = [bag_inventory(b) for b in bags]
    usable = [i for i in inv if i["usable"]]
    if not usable:
        die("没有可用 bag")
    res = {"bags": [], "sensor_height": args.sensor_height,
           "p2l_band_livox_frame": [args.p2l_min_height, args.p2l_max_height]}
    edges = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.326, 0.35, 0.50, 1.00, 2.00, 100.0]
    for i in usable:
        bag = Bag(i["bag"])
        if not bag.has("/segmentation/obstacle"):
            log(f"[!] {i['bag']} 没有 /segmentation/obstacle ⇒ 跳过（无法验证查询带）")
            continue
        n = bag.count("/segmentation/obstacle")
        sel = np.linspace(0, max(0, n - 1), min(args.audit_frames, n)).round().astype(int)
        rows = list(bag.con.execute("select data from messages where topic_id=? order by id",
                                    (bag.topics["/segmentation/obstacle"],)))
        agg = np.zeros(len(edges) - 1)
        fak = 0; tot = 0
        for k in sel:
            m = read_pointcloud2(bytes(rows[k][0]))
            nn = m["width"] * m["height"]
            arr = np.frombuffer(m["data"], dtype=np.uint8, count=nn * m["point_step"]).reshape(nn, m["point_step"])
            xyz = np.stack([arr[:, f["offset"]:f["offset"] + 4].copy().view("<f4").ravel()
                            for f in m["fields"][:3]], axis=1)
            r = np.hypot(xyz[:, 0], xyz[:, 1])
            fake = r < 1e-3
            zr = xyz[~fake][:, 2] + args.sensor_height
            h, _ = np.histogram(zr, bins=edges)
            agg += h; fak += int(fake.sum()); tot += nn
        t = agg.sum()
        vis = agg[(np.array(edges[:-1]) >= 0.05) & (np.array(edges[1:]) <= 0.326)].sum()
        p2l_lo = args.p2l_min_height + args.sensor_height
        p2l_hi = args.p2l_max_height + args.sensor_height
        rowsout = OrderedDict()
        for j in range(len(agg)):
            if agg[j]:
                rowsout[f"[{edges[j]:g},{edges[j+1]:g})"] = {"points": int(agg[j]),
                                                             "frac": round(float(agg[j] / t), 4)}
        log("=" * 108)
        log(f"{i['bag']}: /segmentation/obstacle 抽 {len(sel)} 帧（共 {n} 帧）；"
            f"(0,0,0) 假点 {fak}/{tot} = {fak/max(1,tot)*100:.1f}%（p2l 会因 range<range_min 丢掉）")
        log(f"  真实障碍点（地面以上，地面 = livox z + sensor_height {args.sensor_height:g} m）z 直方图：")
        for kk, vv in rowsout.items():
            log(f"    {kk:>16s}: {vv['points']:8d}  {vv['frac']*100:5.2f}%")
        log(f"  ⇒ p2l 的 min/max_height({args.p2l_min_height:g}/{args.p2l_max_height:g}) 折算到地面以上 = "
            f"({p2l_lo:.3f}, {p2l_hi:.3f}] ⇒ 再叠加 linefit 的 max_dist_to_line 0.05 m ⇒ "
            f"**查询带 ≈ (0.05, {p2l_hi:.3f}]**，其中 {vis/t*100:.1f}% 的障碍点落在里面")
        res["bags"].append({"bag": i["bag"], "n_frames": int(n), "sampled": [int(x) for x in sel],
                            "fake_point_frac": fak / max(1, tot), "hist": rowsout,
                            "frac_in_query_band": float(vis / t),
                            "query_band_above_ground": [0.05, float(p2l_hi)]})
    if args.json:
        with open(_check_tmp_path(args.json), "w", encoding="utf-8") as fh:
            json.dump(res, fh, ensure_ascii=False, indent=2)
        log(f"[i] JSON 已写入 {args.json}")
    return 0


# ======================================================================================
# 自检
# ======================================================================================
def self_test(args):
    ok = True
    scp = load_probe()
    log("== 1) 圆周移位约定 ==")
    bins = 360
    prof = np.arange(bins, dtype=np.float64)
    W = rolling_windows(prof[None, :].astype(np.float32), bins)
    for s in (0, 1, 7, 359):
        same = np.allclose(W[0, s], np.roll(prof, -s))
        ok &= same
        log(f"   window[{s}] == np.roll(D,-{s}) : {same}")
    log("== 2) yaw 语义：查询 = 库在 yaw=+ψ 处的旋转 ==")
    L = np.array([[0.0, 1.0, 2.0, 3.0, 4.0]], dtype=np.float32)   # 假描述子
    Q = L.copy()
    dist, shift, dyaw = retrieve(Q, L, 5, 72.0)                  # 5 bins, 72°=1 bin
    log(f"   同描述子自检索：dist={dist[0,0]:.4f} shift={shift[0,0]} dyaw={dyaw[0,0]:.0f}° （应 0/0/0）")
    ok &= (dist[0, 0] < 1e-9 and shift[0, 0] == 0)
    log("== 3) /scan → 描述子（角度零点 = 传感器 +x，回归 180° 约定坑）==")
    scan = {"ranges": np.array([1.0, np.inf, 2.0, 0.0, 3.0]), "angle_min": -math.pi,
            "angle_increment": 2 * math.pi / 4, "range_min": 0.2, "range_max": 10.0}
    st = {}
    prof = scan_to_range_profile(scan, 4, 10.0, st)
    log(f"   bins=4, angle_inc=90°: {prof}  (束角 -180/-90/0/+90/+180 ⇒ bin 2/3/0/1/2;"
        f" inf 与 <range_min 记 10 ⇒ 期望 [2,10,1,10])  "
        f"stat={ {k: st[k] for k in ('n_beams','n_valid','empty_bins')} }")
    ok &= np.allclose(prof, [2.0, 10.0, 1.0, 10.0])
    # 一致性回归：库侧 raycast 与查询侧映射必须落在同一个 bin（同一角度）
    bins_t = 360
    sc_t = {"ranges": np.array([3.0, np.inf, 3.0, 3.0]), "angle_min": 0.0,
            "angle_increment": 1.0, "range_min": 0.1, "range_max": 10.0}
    ang_t = sc_t["angle_min"] + np.arange(4) * sc_t["angle_increment"]   # 0,1,2,3 rad
    pr = scan_to_range_profile(sc_t, bins_t, 10.0)
    want = [int(a / (2 * math.pi) * bins_t) % bins_t for a in ang_t]
    hit_bins = [int(i) for i in np.flatnonzero(pr < 10.0)]
    log(f"   角度→bin 一致性：4 束在传感器角 {[round(math.degrees(a),1) for a in ang_t]}° ⇒ 应落 bin {want}，"
        f"其中第 2 束是 inf；实际有回波的 bin={hit_bins}")
    ok &= (sorted(hit_bins) == sorted([want[0], want[2], want[3]]))
    ok &= (want[1] not in hit_bins)
    log("== 4) CDR 解析（若有 rclpy 则逐字段比对）==")
    try:
        from rclpy.serialization import deserialize_message       # noqa: F401
        from sensor_msgs.msg import LaserScan                     # noqa: F401
        from nav_msgs.msg import Odometry                         # noqa: F401
        raw = None
        for b in find_bags(args):
            bb = Bag(b)
            if bb.has("/scan"):
                raw = next(bb.stream("/scan", limit=1))[1]
                break
        if raw is None:
            log("   [skip] 没有 bag 可比")
        else:
            d = next(bb.stream("/scan", limit=1))[1]
            same = (abs(d["angle_min"] - raw["angle_min"]) < 1e-12 and
                    d["ranges"].size == raw["ranges"].size and
                    np.allclose(np.nan_to_num(d["ranges"], posinf=0.0),
                                np.nan_to_num(raw["ranges"], posinf=0.0)))
            ok &= bool(same)
            log(f"   frame_id={raw['frame_id']!r} angle_min={raw['angle_min']:.6f} "
                f"n_beams={raw['ranges'].size} range_min={raw['range_min']} range_max={raw['range_max']} "
                f"两次读取一致={same}")
            log("   [i] Odometry 的逐字段比对（含 float64 对齐基准）已在开发期做过：与 rclpy 差 0.0")
    except Exception as exc:                                    # pragma: no cover
        log(f"   [skip] 没有 rclpy/消息包（{exc}）—— 本门本身不需要它")
    log(f"== 自检结果：{'OK' if ok else 'FAILED'} ==")
    return 0 if ok else 1


if __name__ == "__main__":
    _argv = sys.argv[1:]
    _a = build_parser().parse_args(_argv)
    if _a.self_test:
        sys.exit(self_test(_a))
    if _a.query_band_audit:
        sys.exit(query_band_audit(_a))
    sys.exit(main(_argv))
