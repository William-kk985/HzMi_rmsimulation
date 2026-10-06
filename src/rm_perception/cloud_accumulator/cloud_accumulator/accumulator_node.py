#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cloud_accumulator —— 3D 点云累加器：让 **3D 先验** 也能"在上次基础上继续"，且不串场地。

它解决的问题（与 2D 的 slam_toolbox 续建是同一件事的另一半）：
  `mode:=mapping` 里 2D 图由 slam_toolbox 在线维护，可以存成 posegraph 下次续建；
  但 **3D 先验**（ICP/GICP 重定位用的 PCD/<world>.pcd）过去只能"跑一次、存一次"，
  想覆盖更大范围就得把两次的点云**手工拼接**（外参/重叠区全靠人眼）。
  本节点把"每帧点云 → map 系 → 体素下采样 → 累积"这件事做成长跑进程，并提供
  save/load 两个服务 ⇒ 3D 先验与 2D 位姿图一样：**同名存档、下次续上、不需要拼接**。

契约（为什么这么设计）：
  · **只读消费者**：只订阅点云 + 查 TF，不发 /map、不发任何 TF、不发 /segmentation
    ⇒ 「map→odom 单一发布者」「/segmentation/* 单一发布者」两条契约逐字不变；
  · 累积在 **map 系**（= 出生点相对系）⇒ 与 2D 图、与 PCD/ 既有资产同一坐标系，
    以后 `pcd_to_nav2_map.py` / icp_registration 直接吃，没有额外变换；
  · 存档 `<pcd_dir>/<map_name>.pcd` + sidecar `<map_name>.meta.yaml`，
    world/出生点不一致时**默认拒绝 load 与 save**（判定与 launch/shell 共用
    rm_nav_bringup/scripts/map_asset_guard.py 的同一份文本，见 docs/continue_mapping.md）；
  · 打断（Ctrl+C）不会毁存档：只有 `save` 服务真的写文件，其余时间只在内存里累积。

服务（std_srvs/srv/Trigger，与仓库既有 /map_save 同一口径；名字由 `map_name` 参数决定）：
  · `~/save`   写 <pcd_dir>/<map_name>.pcd（同名覆盖，**写前先备份一代**）+ 刷新 sidecar
  · `~/load`   从 <pcd_dir>/<map_name>.pcd 续（**并集合并**到当前内存，不清空已有累积）
  · `~/status` 返回一行状态（点数/体素数/bbox/各类剔除计数），给脚本当体检用
  改名：`ros2 param set /cloud_accumulator map_name <新名>`（下次 save/load 用新名）

数据卫生（2026-10-06 事故后新增，**本节点是"决定什么进云"的清洗层，不是告警节点**）：
  ① 高度带：只保留相对**机器人高度**（`z_ref_frame` 在 map 系的 z，取不到退 0）在
     [z_band_min, z_band_max] 内的点 ⇒ 落在带外的点**丢弃并计数**；
  ② 帧级运动闸：与上一累积帧比，位姿单帧跳变/速度/转角超过阈值 ⇒ **整帧不进云**并计数；
  ③ TF 退化（带戳查询失败、退化到"最新"）：**单独计数**，默认**不累积**该帧
     （`allow_tf_fallback_frame:=True` 才放行）—— 位姿来路不明的帧不该污染先验；
  ④ `~/save` 落盘**前**体检 bbox：z 跨度/XY 跨度不合理时打**响亮 WARNING**（点名具体数字 +
     建议改名另存），但**仍然照写**（不静默拒绝）。
  阈值全部是参数（见各 `declare_parameter` 的中文注释：每个默认值都写了实测依据），
  启动横幅与 `[status]` 行都会把它们和剔除计数一起打出来。
"""
from __future__ import annotations

import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

# 与仓库其它诊断节点同一口径：LiDAR 点云一律 **BEST_EFFORT** 订阅
# （既能收 BEST_EFFORT 的 gazebo/livox 插件，也能收 RELIABLE 的发布者；
#  反过来用 RELIABLE 订阅 BEST_EFFORT 发布者会直接收不到 —— 见 docs/smoke_test_runbook.md 的同类坑）
CLOUD_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)


def _import_guard(share_dir=None):
    """加载 rm_nav_bringup/scripts/map_asset_guard.py（判定/消息文本的唯一出处）。"""
    import importlib.util
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('rm_nav_bringup'),
                                  'scripts', 'map_asset_guard.py'))
    except Exception:
        pass
    if share_dir:
        cands.append(os.path.join(share_dir, 'scripts', 'map_asset_guard.py'))
    for p in cands:
        if os.path.isfile(p):
            spec = importlib.util.spec_from_file_location('rm_map_asset_guard', p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise RuntimeError(
        "[cloud_accumulator] 找不到地图存档守卫模块 map_asset_guard.py（试过：%s）\n"
        '  它在 rm_nav_bringup 包里（scripts/），装法：\n'
        '      colcon build --symlink-install --packages-select rm_nav_bringup\n'
        '      source install/setup.bash' % ' / '.join(cands))


class CloudAccumulator(Node):
    def __init__(self):
        super().__init__('cloud_accumulator')
        g = self.declare_parameter
        self.map_name = g('map_name', '').value
        self.world = g('world', '').value
        self.allow_mismatch = bool(g('allow_world_mismatch', False).value)
        self.autoload = bool(g('autoload', True).value)
        self.voxel = float(g('voxel_size', 0.10).value)
        self.cloud_topic = g('cloud_topic', '/livox/lidar/pointcloud').value
        self.custom_topic = g('custom_topic', '').value
        self.target_frame = g('target_frame', 'map').value
        self.pcd_dir = g('pcd_dir', '').value
        self.max_voxels = int(g('max_voxels', 3000000).value)
        self.tf_timeout = float(g('tf_timeout', 0.20).value)
        self.status_period = float(g('status_period', 15.0).value)
        self.min_range = float(g('min_range', 0.30).value)
        self.max_range = float(g('max_range', 0.0).value)
        self.tf_fallback_latest = bool(g('tf_fallback_latest', True).value)

        # ---------------------------------------------------------------- 数据卫生阈值
        # 2026-10-06：用户从 map/RMUC2026.posegraph 续建时，云里**已经**混着 LIO 退化段
        # （ATE max 1.15 m）按错位姿累积的点：autoload 打出 bbox z[-2.62, 5.56]，
        # 而同一场地健康云实测 z[-0.25, 1.77]（见 docs/continue_mapping.md §9）。
        # 下面每个默认值都从**实测**来，不是拍的（注释里写清测量对象）。
        # ① 高度带（相对机器人高度，单位 m；map 系 = 出生点相对系 ⇒ 站着时 base_link z≈0）：
        #    实测健康累积云 z ∈ [-0.25, 1.77]（用户 2026-10-06 同一场地）、
        #    本仓 PCD/RMUC2026_mapped.pcd（1,450,064 点，全场地）z ∈ [-0.53, 1.43]，
        #    而污染云 PCD/RMUC2026_cont.pcd z ∈ [-4.31, 23.10]。
        #    ⇒ 下界取 -0.5（保住地面附近，离实测最低 -0.53 只差 3 cm，丢的只是零星地面点）；
        #      上界取 +1.8 = 实测最高结构 1.77 + 0.03 余量。
        #      为什么不用 +1.5：那会把实测 1.77 的结构顶**切掉** 27 cm（真数据比离群更珍贵）。
        #      污染两端（-2.62 / +5.56 与 -4.31 / +23.10）都远在带外 ⇒ 一样能剔掉。
        self.z_band_min = float(g('z_band_min', -0.5).value)
        self.z_band_max = float(g('z_band_max', 1.8).value)
        # 高度带的参考系：取它在 map 系的 z 当"本帧局部地面/机器人高度"（机器人上坡时带随之上移）。
        self.z_ref_frame = g('z_ref_frame', 'base_link').value
        # ② 帧级运动闸（与上一累积帧的 map→传感器位姿比）：
        #    实测依据：机器人速度 ≲1 m/s（仿真遥控/覆盖路线 13.9 m / 91 s），
        #    10 Hz 点云 ⇒ 健康单帧位移 ≲0.1 m；提高 5 倍留余量 ⇒ 单帧跳变阈 0.50 m。
        #    速度阈 1.50 m/s = 机器人实际速度上限的 1.5 倍（超过它只可能是位姿自己跳了）。
        #    转角：仿真原地自转 ~1 rad/s ⇒ 10 Hz 下 5.7°/帧，取 4 倍余量 ⇒ 25°/帧；
        #    帧间隔大（丢帧/停顿）时改用角速度阈 90°/s，免得长间隔被误判。
        self.max_frame_step = float(g('max_frame_step', 0.50).value)
        self.max_speed = float(g('max_speed', 1.50).value)
        self.max_frame_yaw_deg = float(g('max_frame_yaw_deg', 25.0).value)
        self.max_yaw_rate_dps = float(g('max_yaw_rate_dps', 90.0).value)
        # 单帧跳变/转角闸只在"帧间隔够小"时才有意义（间隔大 ⇒ 只按速度/角速度判）
        self.jump_gate_dt_max = float(g('jump_gate_dt_max', 0.50).value)
        # ③ TF 退化帧：带戳查询失败、退化到"最新"的帧默认**不累积**（位姿来路不明）
        self.allow_tf_fallback_frame = bool(g('allow_tf_fallback_frame', False).value)
        # ④ save 前体检（见 _do_save）：z 跨度 > 3.0 m 就打响 WARNING。
        #    实测：健康全场地云 z 跨度 1.96 m（RMUC2026_mapped.pcd）/ 2.02 m（用户实测 [-0.25,1.77]）；
        #    污染云 z 跨度 27.41 m（RMUC2026_cont.pcd）/ 8.18 m（用户实测 [-2.62,5.56]）
        #    ⇒ 3.0 m 落在"健康最大 2.02"与"污染最小 8.18"之间，两侧都有 1.5 倍以上间隔。
        self.save_warn_z_span = float(g('save_warn_z_span', 3.0).value)
        #    XY 跨度只是**兜底网**（默认很松，避免误报）：实测健康全场地云 XY 跨度 29.9 × 17.3 m
        #    ⇒ 取 40 m（1.34 倍）。真正的信号是 z 跨度。
        self.save_warn_xy_span = float(g('save_warn_xy_span', 40.0).value)
        # ⑤ 覆盖前备份（2026-10-06 事故：一次 save 就把好存档换成了坏存档）
        self.save_backup = bool(g('save_backup', True).value)
        self.save_backup_keep = int(g('save_backup_keep', 3).value)

        self.guard = _import_guard()
        if not self.pcd_dir:
            self.pcd_dir = self.guard.pcd_dir()
        if not self.map_name:
            self.map_name = self.world or 'default'
        self.base = os.path.join(self.pcd_dir, self.map_name)

        # 体素存储：{ (ix,iy,iz): [sum_x, sum_y, sum_z, n] }（质心口径，与 pcd_stats.py --voxel / small_point_lio 内部一致）
        self.voxels = {}
        self._lock = threading.Lock()
        self.frames_ok = 0
        self.frames_tf_fail = 0
        self.frames_tf_latest = 0                 # 带戳查不到、退化到"最新"成功
        self.frames_tf_latest_skipped = 0         # ……其中被**跳过不累积**的（默认就是这种）
        self.frames_nonfinite = 0
        self.points_range_dropped = 0
        self.points_height_dropped = 0            # 高度带外丢弃的点
        self.frames_height_empty = 0              # 高度带把整帧滤空的次数
        self.frames_jump_skipped = 0              # 运动闸跳过的整帧数
        self.frames_zref_fallback = 0             # 取不到 z_ref_frame 高度、按 0 处理
        self.last_skip_reason = None
        self.last_tf = None                       # 上一累积帧的 (stamp_ns, t, q)：map→传感器
        self.last_cloud_stamp = None
        self.blocked_reason = None
        self.loaded_from = None

        self.cb_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subs = []
        if self.cloud_topic:
            self.subs.append(self.create_subscription(
                PointCloud2, self.cloud_topic, self.on_cloud, CLOUD_QOS,
                callback_group=self.cb_group))
        if self.custom_topic:
            self._setup_custom_sub()

        self.create_service(Trigger, '~/save', self.on_save, callback_group=self.cb_group)
        self.create_service(Trigger, '~/load', self.on_load, callback_group=self.cb_group)
        self.create_service(Trigger, '~/status', self.on_status, callback_group=self.cb_group)
        if self.status_period > 0:
            self.create_timer(self.status_period, self.log_status, callback_group=self.cb_group)

        self.get_logger().info(
            'cloud_accumulator 起：map_name=%s world=%s voxel=%.2f m target_frame=%s pcd_dir=%s\n'
            '  订阅: %s%s（BEST_EFFORT）\n'
            '  契约: 只订阅+查 TF；不发 /map、不发 TF、不发 /segmentation（单一发布者契约不变）\n'
            '  数据卫生阈值（都可用 ros2 param set 改；默认值依据见节点源码注释）：\n'
            '    高度带=[%+.2f,%+.2f] m（相对 %s 在 %s 系的 z，取不到按 0）\n'
            '    单帧跳变<=%.2f m、速度<=%.2f m/s、转角<=%.1f°/帧（帧间隔>%.2f s 时改用角速度<=%.1f°/s）\n'
            '    测距=[%.2f,%s] m；TF 退化到"最新"的帧：%s\n'
            '    save 前体检：z 跨度>%.2f m 或 XY 跨度>%.2f m ⇒ WARNING（仍照写）；覆盖前备份：%s（留 %d 代）'
            % (self.map_name, self.world or '(未给)', self.voxel, self.target_frame, self.pcd_dir,
               self.cloud_topic or '(关)',
               (' + CustomMsg %s' % self.custom_topic) if self.custom_topic else '',
               self.z_band_min, self.z_band_max, self.z_ref_frame or '(关)', self.target_frame,
               self.max_frame_step, self.max_speed, self.max_frame_yaw_deg, self.jump_gate_dt_max,
               self.max_yaw_rate_dps,
               self.min_range, ('%.2f' % self.max_range) if self.max_range > 0 else 'inf',
               '累积' if self.allow_tf_fallback_frame else '默认跳过（allow_tf_fallback_frame:=True 才累积）',
               self.save_warn_z_span, self.save_warn_xy_span,
               '开' if self.save_backup else '关', self.save_backup_keep))
        if self.autoload:
            ok, msg = self._do_load()
            (self.get_logger().info if ok else self.get_logger().error)('autoload: ' + msg)
        else:
            self.get_logger().info('autoload=False ⇒ 不加载既有 PCD，从零点云开始累积')

    # ------------------------------------------------------------ 订阅
    def _setup_custom_sub(self):
        """livox CustomMsg（`/livox/lidar`）—— "适配器模式"：驱动包在就顺手支持，不在就算了。"""
        try:
            from livox_ros_driver2.msg import CustomMsg
        except Exception as e:                                    # pragma: no cover
            self.get_logger().warn(
                'custom_topic:=%s 指定了 Livox CustomMsg，但 import livox_ros_driver2.msg 失败（%s）'
                ' ⇒ 只用 PointCloud2 那一路' % (self.custom_topic, e))
            return
        self.subs.append(self.create_subscription(
            CustomMsg, self.custom_topic, self.on_custom, CLOUD_QOS, callback_group=self.cb_group))
        self.get_logger().info('已额外订阅 Livox CustomMsg：%s' % self.custom_topic)

    def on_custom(self, msg):                                     # pragma: no cover（仿真默认走 PointCloud2）
        n = int(getattr(msg, 'point_num', len(msg.points)))
        if n <= 0:
            return
        pts = np.array([[p.x, p.y, p.z] for p in msg.points[:n]], dtype=np.float64)
        self._ingest(pts, msg.header.frame_id, msg.header.stamp)

    def on_cloud(self, msg: PointCloud2):
        try:
            arr = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            arr = np.array(list(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
        if arr is None or len(arr) == 0:
            return
        self._ingest(np.asarray(arr, dtype=np.float64).reshape(-1, 3), msg.header.frame_id, msg.header.stamp)

    # ------------------------------------------------------------ 累积
    def _ingest(self, pts, frame_id, stamp):
        """一帧点云 → 清洗 → map 系 → 体素合并。任何"不进云"的帧都在这里被计数。"""
        self.last_cloud_stamp = stamp
        good = np.isfinite(pts).all(axis=1)
        if not good.all():
            self.frames_nonfinite += 1
            pts = pts[good]
        if len(pts) == 0:
            return
        r = np.linalg.norm(pts, axis=1)
        keep = r >= self.min_range
        if self.max_range > 0:
            keep &= (r <= self.max_range)
        if not keep.all():
            self.points_range_dropped += int((~keep).sum())
            pts = pts[keep]
        if len(pts) == 0:
            return
        # ---- TF：带戳查询优先；失败才（可选）退化到"最新"，且默认不累积退化帧
        degraded = False
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, stamp, Duration(seconds=self.tf_timeout))
        except TransformException as e:
            self.frames_tf_fail += 1
            if self.tf_fallback_latest:
                try:
                    tr = self.tf_buffer.lookup_transform(
                        self.target_frame, frame_id, rclpy.time.Time(),
                        Duration(seconds=self.tf_timeout))
                    self.frames_tf_latest += 1
                    degraded = True
                except TransformException as e2:
                    self.get_logger().warn('TF %s→%s 查不到（%s / 退化到最新也失败：%s）'
                                           % (frame_id, self.target_frame, e, e2), throttle_duration_sec=10.0)
                    return
            else:
                self.get_logger().warn('TF %s→%s 查不到：%s' % (frame_id, self.target_frame, e),
                                       throttle_duration_sec=10.0)
                return
        t = tr.transform.translation
        q = tr.transform.rotation
        t_now = (float(t.x), float(t.y), float(t.z))
        q_now = (float(q.x), float(q.y), float(q.z), float(q.w))
        if degraded and not self.allow_tf_fallback_frame:
            # 位姿来路不明的帧不进先验（2026-10-06：错位姿累积正是云被污染的主因）
            self.frames_tf_latest_skipped += 1
            self._note_skip('TF 退化到最新')
            self.get_logger().warn(
                'TF %s→%s 带戳查不到，退化到"最新"成功 ⇒ **跳过该帧不累积**'
                '（已经发生 %d 次；要照旧累积请设 allow_tf_fallback_frame:=True）'
                % (frame_id, self.target_frame, self.frames_tf_latest_skipped),
                throttle_duration_sec=10.0)
            self._remember_tf(stamp, t_now, q_now)
            return
        # ---- 运动闸：与上一累积帧比（跳变帧只丢这一帧，参考位姿仍推进 ⇒ 一次跳变不会连累后面所有帧）
        ok, why = self._motion_ok(stamp, t_now, q_now)
        if not ok:
            self.frames_jump_skipped += 1
            self._note_skip(why)
            self.get_logger().warn(
                '%s ⇒ **跳过该帧**（已跳 %d 帧；阈值 max_frame_step=%.2f m / max_speed=%.2f m/s / '
                'max_frame_yaw_deg=%.1f°；LIO 可能在退化，见 docs/continue_mapping.md §9）'
                % (why, self.frames_jump_skipped, self.max_frame_step, self.max_speed,
                   self.max_frame_yaw_deg), throttle_duration_sec=10.0)
            self._remember_tf(stamp, t_now, q_now)
            return
        self._remember_tf(stamp, t_now, q_now)
        pts_map = _apply_tf(pts, t_now, q_now)
        # ---- 高度带（相对本帧的机器人/局部地面高度）
        pts_map, z_ref = self._filter_height(pts_map)
        if len(pts_map) == 0:
            self.frames_height_empty += 1
            self._note_skip('整帧落在高度带外')
            return
        added = self._merge_voxels(pts_map)
        self.frames_ok += 1
        if added and self.get_logger().is_enabled_for(rclpy.logging.LoggingSeverity.DEBUG):
            self.get_logger().debug('+%d 体素（z 参考=%.3f）' % (added, z_ref))

    def _remember_tf(self, stamp, t, q):
        try:
            self.last_tf = (int(stamp.nanoseconds) if stamp else 0, t, q)
        except Exception:                                        # pragma: no cover
            self.last_tf = (0, t, q)

    def _note_skip(self, reason):
        self.last_skip_reason = reason

    def _motion_ok(self, stamp, t, q):
        """与上一累积帧的位姿比：单帧跳变 / 速度 / 转角 / 角速度 是否都在阈值内。

        返回 (ok, 原因文本)。没有上一帧（第一帧）或时间戳不可用 ⇒ 只查单帧跳变/转角。
        """
        prev = self.last_tf
        if prev is None:
            return True, ''
        try:
            dt = (int(stamp.nanoseconds) - prev[0]) / 1e9 if stamp else 0.0
        except Exception:                                        # pragma: no cover
            dt = 0.0
        step = float(np.linalg.norm(np.asarray(t, dtype=np.float64) - np.asarray(prev[1], dtype=np.float64)))
        dyaw = _yaw_between(prev[2], q)                          # rad
        dyaw_deg = abs(np.degrees(dyaw))
        small_dt = dt <= self.jump_gate_dt_max                # 帧间隔小 ⇒ 单帧闸有意义
        if small_dt:
            if self.max_frame_step > 0 and step > self.max_frame_step:
                return False, ('单帧跳变 %.2f m > %.2f m' % (step, self.max_frame_step))
            if self.max_frame_yaw_deg > 0 and dyaw_deg > self.max_frame_yaw_deg:
                return False, ('单帧转角 %.1f° > %.1f°' % (dyaw_deg, self.max_frame_yaw_deg))
        if dt > 1e-6:
            v = step / dt
            w = dyaw_deg / dt
            if self.max_speed > 0 and v > self.max_speed:
                return False, ('位姿隐含速度 %.2f m/s > %.2f m/s（Δt=%.3f s 走了 %.2f m）'
                               % (v, self.max_speed, dt, step))
            if self.max_yaw_rate_dps > 0 and w > self.max_yaw_rate_dps:
                return False, ('位姿隐含角速度 %.0f°/s > %.0f°/s（Δt=%.3f s 转了 %.1f°）'
                               % (w, self.max_yaw_rate_dps, dt, dyaw_deg))
        return True, ''

    def _filter_height(self, pts_map):
        """只保留相对机器人/局部地面高度在 [z_band_min, z_band_max] 内的点（带外计数丢弃）。"""
        if self.z_band_max <= self.z_band_min:
            return pts_map, float('nan')                          # 带设反/设死 ⇒ 关掉这一层
        z_ref = 0.0
        if self.z_ref_frame:
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.target_frame, self.z_ref_frame, rclpy.time.Time(),
                    Duration(seconds=self.tf_timeout))
                z_ref = float(tr.transform.translation.z)
            except TransformException:
                self.frames_zref_fallback += 1
                z_ref = 0.0                                       # map 系 = 出生点相对系 ⇒ 站着时 ≈0
                self.get_logger().warn(
                    'TF %s→%s 取不到（高度带按 z_ref=0 处理，已 %d 次）'
                    % (self.target_frame, self.z_ref_frame, self.frames_zref_fallback),
                    throttle_duration_sec=30.0)
        lo, hi = z_ref + self.z_band_min, z_ref + self.z_band_max
        keep = (pts_map[:, 2] >= lo) & (pts_map[:, 2] <= hi)
        if not keep.all():
            self.points_height_dropped += int((~keep).sum())
        return pts_map[keep], z_ref

    def _merge_voxels(self, pts_map):
        """把一帧 map 系点并进体素表（每体素存质心累加量 + 计数）。返回新增体素数。"""
        keys, sums, counts = _quantize(pts_map, self.voxel)
        added = 0
        with self._lock:
            for i, k in enumerate(keys):
                key = (int(k[0]), int(k[1]), int(k[2]))
                rec = self.voxels.get(key)
                if rec is None:
                    if len(self.voxels) >= self.max_voxels:
                        continue
                    self.voxels[key] = [float(sums[i, 0]), float(sums[i, 1]), float(sums[i, 2]),
                                        int(counts[i])]
                    added += 1
                else:
                    rec[0] += float(sums[i, 0]); rec[1] += float(sums[i, 1]); rec[2] += float(sums[i, 2])
                    rec[3] += int(counts[i])
        return added

    # ------------------------------------------------------------ 统计 / 状态
    def stats(self):
        with self._lock:
            n = len(self.voxels)
            if n == 0:
                return {'voxels': 0, 'points': 0, 'bbox': None, 'centroid': None, 'size_mb': 0.0}
            keys = None   # 体素键是 tuple(int,int,int)，不参与统计（质心从 sums/counts 算）
            recs = np.array(list(self.voxels.values()), dtype=np.float64)
        centers = recs[:, :3] / recs[:, 3:4]
        mn, mx = centers.min(axis=0), centers.max(axis=0)
        return {'voxels': int(n), 'points': int(recs[:, 3].sum()),
                'bbox': (mn.tolist(), mx.tolist()),
                'centroid': centers.mean(axis=0).tolist(),
                'size_mb': n * 8 * 4 / 1048576.0}

    def status_line(self):
        s = self.stats()
        bbox = ('x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
                % (s['bbox'][0][0], s['bbox'][1][0], s['bbox'][0][1], s['bbox'][1][1],
                   s['bbox'][0][2], s['bbox'][1][2])) if s['bbox'] else '(空)'
        zspan = ('%.2f' % (s['bbox'][1][2] - s['bbox'][0][2])) if s['bbox'] else '—'
        return ('存档=%s 体素=%d 累积点=%d 覆盖 bbox=%s（z 跨度=%s m / 告警线 %.2f m）体积=%.2f MB | '
                '帧: 成功=%d TF失败=%d(退化最新=%d/其中跳过=%d) 非法点帧=%d 跳变跳过=%d 高度带滤空=%d | '
                '点: 距离滤除=%d 高度带滤除=%d | 高度参考缺失=%d 最近跳过原因=%s | 最近点云戳=%s | '
                '阈值: 高度带=[%+.2f,%+.2f]m(相对 %s) 单帧跳变<=%.2f m 速度<=%.2f m/s 转角<=%.1f°/帧 '
                '角速度<=%.0f°/s 测距=[%.2f,%s] m'
                % (os.path.join(self.pcd_dir, self.map_name), s['voxels'], s['points'], bbox, zspan,
                   self.save_warn_z_span, s['size_mb'],
                   self.frames_ok, self.frames_tf_fail, self.frames_tf_latest,
                   self.frames_tf_latest_skipped, self.frames_nonfinite, self.frames_jump_skipped,
                   self.frames_height_empty,
                   self.points_range_dropped, self.points_height_dropped,
                   self.frames_zref_fallback, self.last_skip_reason or '(无)',
                   self.last_cloud_stamp if self.last_cloud_stamp else '(还没收到点云)',
                   self.z_band_min, self.z_band_max, self.z_ref_frame or '(关)',
                   self.max_frame_step, self.max_speed, self.max_frame_yaw_deg,
                   self.max_yaw_rate_dps, self.min_range,
                   ('%.2f' % self.max_range) if self.max_range > 0 else 'inf'))

    def log_status(self):
        try:
            self.get_logger().info('[status] ' + self.status_line())
        except Exception as e:                                    # pragma: no cover
            self.get_logger().warn('status 打印失败：%s' % e)

    # ------------------------------------------------------------ 守卫
    def _verify(self, kind):
        return self.guard.verify(self.base, self.map_name, self.world or self.map_name,
                                 kind=kind, allow_mismatch=self.allow_mismatch)

    # ------------------------------------------------------------ 服务
    def _do_load(self):
        v = self._verify(self.guard.KIND_PCD)
        art = v['archive']
        if v['code'] == 'fresh':
            self.blocked_reason = None
            return False, ('没有 3D 先验存档 %s ⇒ 从零点云开始累积（本次结束后用 '
                           'ros2 service call /cloud_accumulator/save std_srvs/srv/Trigger 落盘）' % art)
        if not v['may_load']:
            self.blocked_reason = v['code']
            return False, v['message']
        try:
            xyz = _pcd_read(art)
        except Exception as e:
            self.blocked_reason = 'read_error'
            return False, '读 %s 失败：%s' % (art, e)
        before = len(self.voxels)
        self._merge_voxels(xyz)
        self.blocked_reason = None
        self.loaded_from = art
        s = self.stats()
        return True, ('已续建 3D 先验：%s（读入 %d 点；体素 %d → %d，累积点 %d，%s）%s'
                      % (art, len(xyz), before, s['voxels'], s['points'],
                         ('覆盖 bbox ' + str(s['bbox'])) if s['bbox'] else '',
                         '' if v['code'] == 'resume' else '（⚠️ 已按 map_allow_world_mismatch 跳过隔离检查）'))

    def _health_check(self, s):
        """落盘前体检：返回 (warn: bool, 详情文本, 一句话原因)。不合理**只喊不拦**（照写，但必须让人看见）。"""
        if not s['bbox']:
            return False, '', ''
        mn, mx = s['bbox'][0], s['bbox'][1]
        zspan = float(mx[2] - mn[2])
        xspan, yspan = float(mx[0] - mn[0]), float(mx[1] - mn[1])
        bad = []
        if self.save_warn_z_span > 0 and zspan > self.save_warn_z_span:
            bad.append('z 跨度 %.2f m > %.2f m（实测健康云 1.96~2.02 m / 本次 z[%.2f,%.2f]）'
                       % (zspan, self.save_warn_z_span, mn[2], mx[2]))
        if self.save_warn_xy_span > 0 and max(xspan, yspan) > self.save_warn_xy_span:
            bad.append('XY 跨度 %.2f×%.2f m 超过 %.2f m（实测健康全场地云 29.9×17.3 m）'
                       % (xspan, yspan, self.save_warn_xy_span))
        if not bad:
            return False, '', ''
        reason = '；'.join(bad)
        art = os.path.join(self.pcd_dir, self.map_name + '.pcd')
        txt = ('\n'.join([
            '⚠️⚠️ 3D 先验体检不通过 —— 这份累积云**很可能已经被 LIO 退化/错位姿污染**（仍然照写，不拦）：',
            '  · 存档        : %s' % art,
            '  · 不合理之处  : %s' % reason,
            '  · 完整 bbox   : x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]（%d 点 / %d 体素）'
            % (mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], s['points'], s['voxels']),
            '  · 剔除计数    : 高度带滤除 %d 点 / 跳变跳过 %d 帧 / TF退化跳过 %d 帧 / 高度带滤空 %d 帧'
            % (self.points_height_dropped, self.frames_jump_skipped,
               self.frames_tf_latest_skipped, self.frames_height_empty),
            '  · 建议改名另存（别覆盖可能是好的那份存档）：',
            '        ros2 param set /cloud_accumulator map_name %s_bad   # 再 save ⇒ 写 PCD/%s_bad.pcd'
            % (self.map_name, self.map_name),
            '    或者先不用这份 3D 先验：检查 PCD/%s.pcd 的 z 分位数（python3 tools/scripts/mapping/pcd_stats.py ...），'
            '必要时用上一代备份回滚（tools/scripts/mapping/map_archive.sh restore --name %s）'
            % (self.map_name, self.map_name),
            '  · 2026-10-06 的事故就是这么发生的：续建时云里已有退化段 ⇒ save 同名覆盖掉了好存档。',
        ]))
        return True, txt, reason

    def _do_save(self):
        v = self._verify(self.guard.KIND_PCD)
        if not v['may_load'] and v['code'] != 'fresh':
            self.blocked_reason = v['code']
            return False, v['message']                       # ← 拒绝覆盖别场地的先验
        s = self.stats()
        if s['voxels'] == 0:
            return False, '内存里一个体素都没有（点云没进来 / TF 一直失败？）⇒ 不覆盖既有存档'
        art = v['archive']
        # ---- ① 体检（**写之前**喊；不拦）
        warn, warn_txt, warn_reason = self._health_check(s)
        if warn:
            self.get_logger().warn(warn_txt)
        # ---- ② 覆盖前备份一套（.pcd + .meta.yaml，同一代时间戳，默认留 3 代）
        backup_note = ''
        if self.save_backup:
            try:
                recs = self.guard.backup_files(
                    [art, self.guard.manifest_path(self.base)],
                    keep=self.save_backup_keep, log=lambda m: self.get_logger().info(m))
                done = [r for r in recs if r['dst']]
                if done:
                    backup_note = ('（写前已备份 %d 个文件 → `*.prev-%s`，keep=%d；'
                                   '回滚：tools/scripts/mapping/map_archive.sh restore --name %s）'
                                   % (len(done), done[0]['ts'], self.save_backup_keep, self.map_name))
                else:
                    backup_note = '（没有既有 PCD 可备份：这是第一次落盘）'
            except Exception as e:                            # 备份失败**不挡**落盘，但要说出来
                backup_note = '（⚠️ 写前备份失败：%s）' % e
                self.get_logger().error('写前备份失败（仍然继续落盘）：%s' % e)
        with self._lock:
            recs = np.array(list(self.voxels.values()), dtype=np.float64)
        centers = (recs[:, :3] / recs[:, 3:4]).astype(np.float64)
        os.makedirs(self.pcd_dir, exist_ok=True)
        n = _pcd_write(art, centers)
        res = self._verify(self.guard.KIND_PCD)              # 再查一次：写完之后 base 已存在 ⇒ 正常路径
        man = self.guard.write_manifest(self.base, self.map_name, self.guard.KIND_PCD, self.world or self.map_name,
                                        [art], resolution=None, spawn=self.guard.world_spawn(self.world),
                                        extra={'accumulated_points': s['points'],
                                               'voxel_size': self.voxel,
                                               'bbox': s['bbox'],
                                               'z_span': (float(s['bbox'][1][2] - s['bbox'][0][2])
                                                          if s['bbox'] else None),
                                               'health_warning': bool(warn),
                                               'dropped_points_height': self.points_height_dropped,
                                               'dropped_points_range': self.points_range_dropped,
                                               'skipped_frames_jump': self.frames_jump_skipped,
                                               'skipped_frames_tf_fallback': self.frames_tf_latest_skipped,
                                               'source': 'cloud_accumulator',
                                               'tf_frames_ok': self.frames_ok,
                                               'tf_frames_failed': self.frames_tf_fail})
        size = os.path.getsize(art) / 1048576.0
        self.blocked_reason = None
        msg = ('已落盘 3D 先验：%s（%d 点 / %.2f MB；累积点 %d，voxel=%.2f m）'
               ' + sidecar %s（world=%s）%s'
               % (art, n, size, s['points'], self.voxel,
                  self.guard.manifest_path(self.base), man['world'], backup_note))
        if warn:
            msg += ('\n⚠️ 健康告警（已照写）：%s —— 建议改名另存：'
                    'ros2 param set /cloud_accumulator map_name %s_bad 后重新 save'
                    % (warn_reason, self.map_name))
        return True, msg

    def on_save(self, req, resp):
        try:
            ok, msg = self._do_save()
        except Exception as e:
            ok, msg = False, 'save 异常：%s' % e
        resp.success, resp.message = bool(ok), msg
        (self.get_logger().info if ok else self.get_logger().error)('save: ' + msg)
        return resp

    def on_load(self, req, resp):
        try:
            ok, msg = self._do_load()
        except Exception as e:
            ok, msg = False, 'load 异常：%s' % e
        resp.success, resp.message = bool(ok), msg
        (self.get_logger().info if ok else self.get_logger().error)('load: ' + msg)
        return resp

    def on_status(self, req, resp):
        resp.success = True
        resp.message = self.status_line()
        return resp


# ---------------------------------------------------------------- 数学/IO 小工具
def _apply_tf(pts, t, q):
    x, y, z, w = q
    n = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float64)
    return pts @ n.T + np.asarray(t, dtype=np.float64)


def _yaw_between(q1, q2):
    """两个四元数之间的**相对偏航角**（rad，带符号）= (q1⁻¹ ⊗ q2) 的 yaw。

    用于运动闸的"单帧转角/角速度"：只关心绕 z 的转，与 roll/pitch 解耦。
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    ix, iy, iz, iw = -x1, -y1, -z1, w1              # q1⁻¹（单位四元数 ⇒ 共轭就是逆）
    rx = iw * x2 + ix * w2 + iy * z2 - iz * y2
    ry = iw * y2 - ix * z2 + iy * w2 + iz * x2
    rz = iw * z2 + ix * y2 - iy * x2 + iz * w2
    rw = iw * w2 - ix * x2 - iy * y2 - iz * z2
    return float(np.arctan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz)))


def _quantize(pts, voxel):
    """按体素量化：返回 (keys[K,3], sums[K,3], counts[K])。K = 本帧命中的体素数（远小于点数）。"""
    idx = np.floor(pts / voxel).astype(np.int64)
    keys, inv = np.unique(idx, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(keys), 3), dtype=np.float64)
    np.add.at(sums, inv, pts)
    counts = np.bincount(inv, minlength=len(keys))
    return keys, sums, counts


def _pcd_read(path):
    from .pcd_io import read_pcd_xyz
    return read_pcd_xyz(path)


def _pcd_write(path, xyz):
    from .pcd_io import write_pcd_binary
    return write_pcd_binary(path, xyz, legacy8=True)


def main(argv=None):
    rclpy.init(args=argv)
    node = CloudAccumulator()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
