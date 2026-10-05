#!/usr/bin/env python3
"""局部代价地图"障碍来源 A/B"探针：**只订阅、不发指令**，把"局部代价地图到底看到了什么"
按 2 Hz 采样成 JSONL，用于比较 `local_obstacle:=scan` 与 `local_obstacle:=stvl`。

它同时采三路，答案才能自洽：
  ① `/local_costmap/costmap_raw`（`nav2_msgs/Costmap`，master 栅格，254=真障碍）
     → 机器人周围 0~0.5 m / 0.5~1.0 m 环带里的**真障碍格数**、到最近障碍的距离、车后扇区残余；
  ② `/global_costmap/costmap_raw`（同一时刻的对照）—— 全局图默认就吃 STVL
     ⇒ **"local(scan) 没看到、同一瞬间 global(STVL) 看到了"** 就是本实验要的那个证据；
  ③ 两个**原始来源**：`/scan`（LaserScan）与 `/segmentation/obstacle`（PointCloud2）
     → 各自在机器人 0.5 m 内有多少量测、最近距离多少、点云里有多少点落在 p2l 的高度带
     （`min_height −1.0 / max_height 0.1`，见 pointcloud_to_laserscan 的 laserscan_params.yaml）
     之外 —— 即"STVL 能标、/scan 先天看不到"的那部分。

坐标系：局部图 `odom`、全局图 `map`，机器人位姿一律用 `base_link_fake`（costmap 的
robot_base_frame）。所有距离 = 栅格中心到机器人中心的**平面**距离。

用法（栈起来之后跑；**不影响栈的行为**，只多一个订阅者）：
  python3 tools/scripts/regress/local_obstacle_probe.py --out .tmp_cache/stvl_local/scan.probe.jsonl
  # 想同时看全局对照（默认开）：--no-global 关掉
  # 采样周期：--period 0.5（秒，墙钟）

输出：JSONL（一行一个采样）+ 结尾一行 `{"summary": {...}}`；进程收到 SIGINT/SIGTERM 时
也会把 summary 写出来（用 pkill -INT 就够）。

退出码：0=正常；2=一直没等到 costmap_raw（链路不通）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

# costmap_raw 的发布端 QoS：KeepLast(1) + transient_local + reliable
# （nav2_costmap_2d/src/costmap_2d_publisher.cpp:69）
RAW_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
# 传感器源：p2l / linefit 都是 SensorDataQoS(best effort)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST)

LETHAL = 254          # nav2 LETHAL_OBSTACLE（未膨胀的"真障碍"）
# p2l 的高度带（livox_frame 下的 z；laserscan_params.yaml: min_height -1.0 / max_height 0.1）
SCAN_MIN_Z, SCAN_MAX_Z = -1.0, 0.1


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self, period, with_global, out_path, dump_path=None):
        super().__init__("local_obstacle_probe")
        self.period = period
        self.with_global = with_global
        self.out_path = out_path
        self.fh = open(out_path, "w", encoding="utf-8")
        self.n_samples = 0
        self.n_fail = 0
        self.dump_path = dump_path
        self.grid_best = None      # (min_r, grid, (rx,ry,ryaw), stamp)
        self.grid_last = None
        self.n_raw_local = 0
        self.n_raw_global = 0
        self.n_scan = 0
        self.n_cloud = 0
        self.agg = {}

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lc = None
        self.lc_grid = None
        self.gc = None
        self.scan = None
        self.cloud = None
        self.create_subscription(Costmap, "/local_costmap/costmap_raw", self._on_lc, RAW_QOS)
        if with_global:
            self.create_subscription(Costmap, "/global_costmap/costmap_raw", self._on_gc, RAW_QOS)
        # 也收一份 OccupancyGrid 版（只在需要排查 costmap_raw 缺失时看计数）
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_lc_grid, RAW_QOS)
        self.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_QOS)
        self.create_subscription(PointCloud2, "/segmentation/obstacle", self._on_cloud, SENSOR_QOS)
        self.get_logger().info("probe 起：period=%.2fs global=%s out=%s"
                               % (period, with_global, out_path))

    # ---------------- 回调：只存最新一帧 ----------------
    def _on_lc(self, m):
        self.lc = m
        self.n_raw_local += 1

    def _on_gc(self, m):
        self.gc = m
        self.n_raw_global += 1

    def _on_lc_grid(self, m):
        self.lc_grid = m

    def _on_scan(self, m):
        self.scan = m
        self.n_scan += 1

    def _on_cloud(self, m):
        self.cloud = m
        self.n_cloud += 1

    # ---------------- 几何工具 ----------------
    @staticmethod
    def _grid_np(msg):
        sx, sy = msg.metadata.size_x, msg.metadata.size_y
        g = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(sy, sx)
        res = msg.metadata.resolution
        ox = msg.metadata.origin.position.x
        oy = msg.metadata.origin.position.y
        return g, res, ox, oy

    @staticmethod
    def _annulus(g, res, ox, oy, rx, ry, r_in, r_out, ang_lo=None, ang_hi=None, ryaw=None):
        """返回 (真障碍格数, 最小平面距离 or None)：机器人 (rx,ry) 周围 r_in~r_out 环带。

        ang_lo/ang_hi：可选的角度窗口（相对机器人朝向 ryaw，弧度，wrap 到 [-pi,pi]）。
        """
        sy, sx = g.shape
        ys, xs = np.nonzero((g >= LETHAL) & (g < 255))
        if xs.size == 0:
            return 0, None
        px = ox + (xs + 0.5) * res - rx
        py = oy + (ys + 0.5) * res - ry
        d = np.hypot(px, py)
        sel = (d >= r_in) & (d <= r_out)
        if ang_lo is not None:
            a = np.arctan2(py, px) - ryaw
            a = np.arctan2(np.sin(a), np.cos(a))
            sel &= (a >= ang_lo) & (a <= ang_hi)
        if not sel.any():
            return 0, None
        return int(sel.sum()), float(d[sel].min())

    def _tf(self, parent, child):
        """返回 (R(3x3), t(3,)) 或 None：parent←child 的刚体变换。"""
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R, np.array([tr.x, tr.y, tr.z])

    def _planar_pose(self, parent):
        """机器人在 parent 系里的 (x, y, yaw)。"""
        tfm = self._tf(parent, "base_link_fake")
        if tfm is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(parent, "base_link_fake", rclpy.time.Time())
        except Exception:
            return None
        R, tr = tfm
        return float(tr[0]), float(tr[1]), _yaw(t.transform.rotation)

    # ---------------- 单次采样 ----------------
    def sample(self):
        if self.lc is None:
            return None
        pose_odom = self._planar_pose("odom")
        if pose_odom is None:
            return None
        rx, ry, ryaw = pose_odom
        row = {"t_wall": round(time.time(), 3)}

        # ① 局部代价地图
        g, res, ox, oy = self._grid_np(self.lc)
        n05, d05 = self._annulus(g, res, ox, oy, rx, ry, 0.0, 0.5)
        n510, _ = self._annulus(g, res, ox, oy, rx, ry, 0.5, 1.0)
        nb_r, _ = self._annulus(g, res, ox, oy, rx, ry, 0.3, 1.5, ryaw=ryaw,
                                ang_lo=math.radians(120), ang_hi=math.radians(180))
        nb_l, _ = self._annulus(g, res, ox, oy, rx, ry, 0.3, 1.5, ryaw=ryaw,
                                ang_lo=math.radians(-180), ang_hi=math.radians(-120))
        nbehind = nb_r + nb_l
        nall, dall = self._annulus(g, res, ox, oy, rx, ry, 0.0, 1.0)
        row.update({
            "lc_stamp": self.lc.header.stamp.sec + self.lc.header.stamp.nanosec * 1e-9,
            "lc_lethal_00_05": n05, "lc_lethal_05_10": n510,
            "lc_lethal_behind_03_15": nbehind,
            "lc_lethal_min_r": d05 if d05 is not None else dall,
            "lc_lethal_n_1m": nall,
            "lc_frame": self.lc.header.frame_id,
        })

        if self.dump_path is not None:
            self.grid_last = (g.copy(), (rx, ry, ryaw), row["lc_stamp"])
            r_now = row.get("lc_lethal_min_r")
            if r_now is not None and (self.grid_best is None or r_now < self.grid_best[0]):
                self.grid_best = (r_now, g.copy(), (rx, ry, ryaw), row["lc_stamp"])

        # ② 全局代价地图（对照；默认就吃 STVL）
        if self.with_global and self.gc is not None:
            pose_map = self._planar_pose("map")
            if pose_map is not None:
                mx, my, myaw = pose_map
                gg, gres, gox, goy = self._grid_np(self.gc)
                gn05, gd05 = self._annulus(gg, gres, gox, goy, mx, my, 0.0, 0.5)
                gnall, gdall = self._annulus(gg, gres, gox, goy, mx, my, 0.0, 1.0)
                row.update({"gc_lethal_00_05": gn05, "gc_lethal_n_1m": gnall,
                            "gc_lethal_min_r": gd05 if gd05 is not None else gdall})

        # ③ 原始来源：/scan
        if self.scan is not None:
            s = self.scan
            tfs = self._tf("odom", s.header.frame_id)
            if tfs is not None:
                R, tr = tfs
                ang = s.angle_min + s.angle_increment * np.arange(len(s.ranges))
                rng = np.asarray(s.ranges, dtype=np.float32)
                ok = np.isfinite(rng) & (rng >= max(s.range_min, 0.0))
                loc = np.stack([rng * np.cos(ang), rng * np.sin(ang),
                                np.zeros_like(rng)], axis=1)
                p = loc @ R.T + tr          # livox_frame → odom
                px, py = p[:, 0], p[:, 1]
                d = np.hypot(px - rx, py - ry)
                near = ok & (d <= 0.5)
                row.update({
                    "scan_n_05": int(near.sum()),
                    "scan_min_r": float(rng[ok].min()) if ok.any() else None,
                    "scan_min_d_robot": float(d[ok].min()) if ok.any() else None,
                    "scan_n_valid": int(ok.sum()), "scan_n": int(rng.size),
                })

        # ③ 原始来源：/segmentation/obstacle（含"落在 p2l 高度带内/外"的拆分）
        if self.cloud is not None:
            tfc = self._tf("odom", self.cloud.header.frame_id)
            if tfc is not None:
                R, tr = tfc
                try:
                    pts = pc2.read_points_numpy(
                        self.cloud, field_names=("x", "y", "z"), skip_nans=True)
                except Exception:
                    pts = np.zeros((0, 3), dtype=np.float32)
                if pts.size:
                    z = pts[:, 2]
                    p = pts @ R.T + tr      # livox_frame → odom
                    pxc, pyc = p[:, 0], p[:, 1]
                    d = np.hypot(pxc - rx, pyc - ry)
                    near = d <= 0.5
                    inband = near & (z >= SCAN_MIN_Z) & (z <= SCAN_MAX_Z)
                    row.update({
                        "cloud_n": int(pts.shape[0]),
                        "cloud_n_05": int(near.sum()),
                        "cloud_n_05_in_scanband": int(inband.sum()),
                        "cloud_n_05_out_scanband": int((near & ~((z >= SCAN_MIN_Z) &
                                                                  (z <= SCAN_MAX_Z))).sum()),
                        "cloud_z_min": float(np.min(z)), "cloud_z_max": float(np.max(z)),
                        "cloud_min_d_robot": float(d.min()) if d.size else None,
                    })
        return row

    def run(self):
        last = 0.0
        self._stop = False

        def _sig(_s, _f):
            self._stop = True
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)
        signal.signal(signal.SIGALRM, _sig)

        while not self._stop and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now - last < self.period:
                continue
            last = now
            try:
                row = self.sample()
            except Exception as e:  # 采样失败不能拖垮探针
                self.n_fail += 1
                if self.n_fail <= 1:
                    self.get_logger().warn("sample 失败（首条，含堆栈）：%s\n%s"
                                           % (e, traceback.format_exc()))
                elif self.n_fail in (10, 100, 1000):
                    self.get_logger().warn("sample 已连续失败 %d 次：%r" % (self.n_fail, e))
                continue
            if row is None:
                continue
            self.n_samples += 1
            for k, v in row.items():
                if k.startswith("t_") or k.endswith("_frame") or v is None:
                    continue
                self.agg.setdefault(k, []).append(v)
            self.fh.write(json.dumps(row, ensure_ascii=False) + "\n")
            self.fh.flush()

        summary = {
            "samples": self.n_samples, "sample_failures": self.n_fail,
            "msgs": {"local_costmap_raw": self.n_raw_local,
                     "global_costmap_raw": self.n_raw_global,
                     "scan": self.n_scan, "cloud": self.n_cloud},
            "fields": {k: {"n": len(v), "min": float(np.min(v)), "max": float(np.max(v)),
                           "mean": float(np.mean(v)), "sum": float(np.sum(v)),
                           "nonzero": int(np.count_nonzero(np.asarray(v)))}
                       for k, v in sorted(self.agg.items())},
        }
        self.fh.write(json.dumps({"summary": summary}, ensure_ascii=False) + "\n")
        self.fh.close()
        if self.dump_path is not None and (self.grid_last or self.grid_best):
            kw = {}
            if self.grid_last:
                g, pose, st = self.grid_last
                kw.update(last_grid=g, last_pose=np.array(pose), last_stamp=st)
            if self.grid_best:
                r, g, pose, st = self.grid_best
                kw.update(best_grid=g, best_pose=np.array(pose), best_stamp=st, best_min_r=r)
            np.savez_compressed(self.dump_path, **kw)
            print("[probe] 栅格快照 → %s" % self.dump_path, flush=True)
        print("[probe] samples=%d 写 %s" % (self.n_samples, self.out_path), flush=True)
        return 0 if self.n_samples > 0 else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="输出 JSONL 路径")
    ap.add_argument("--period", type=float, default=0.5, help="采样周期（墙钟秒，默认 0.5）")
    ap.add_argument("--no-global", action="store_true", help="不采全局 costmap（省一点 CPU）")
    ap.add_argument("--dump-grid", default=None, metavar="PATH.npz",
                    help="退出时把『最近一次』与『离障碍最近那一刻』的 local_costmap_raw 原始栅格 "
                         "+ 机器人位姿存成 npz（离线看墙上的标记是连续线还是点状）")
    ap.add_argument("--timeout", type=float, default=0.0,
                    help=">0 则最多跑这么多秒后自行退出（默认 0=直到被 kill）")
    a = ap.parse_args()

    rclpy.init()
    n = Probe(a.period, not a.no_global, a.out, a.dump_grid)
    if a.timeout > 0:
        signal.alarm(int(a.timeout))
    try:
        rc = n.run()
    except KeyboardInterrupt:
        rc = 0
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
