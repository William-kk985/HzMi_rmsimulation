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
  · `~/save`   写 <pcd_dir>/<map_name>.pcd（同名覆盖）+ 刷新 sidecar
  · `~/load`   从 <pcd_dir>/<map_name>.pcd 续（**并集合并**到当前内存，不清空已有累积）
  · `~/status` 返回一行状态（点数/体素数/bbox/TF 失败计数），给脚本当体检用
  改名：`ros2 param set /cloud_accumulator map_name <新名>`（下次 save/load 用新名）
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
        self.frames_tf_latest = 0
        self.frames_nonfinite = 0
        self.frames_range_dropped = 0
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
            '  契约: 只订阅+查 TF；不发 /map、不发 TF、不发 /segmentation（单一发布者契约不变）'
            % (self.map_name, self.world or '(未给)', self.voxel, self.target_frame, self.pcd_dir,
               self.cloud_topic or '(关)',
               (' + CustomMsg %s' % self.custom_topic) if self.custom_topic else ''))
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
            self.frames_range_dropped += int((~keep).sum())
            pts = pts[keep]
        if len(pts) == 0:
            return
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
        pts_map = _apply_tf(pts, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        added = self._merge_voxels(pts_map)
        self.frames_ok += 1
        if added and self.get_logger().is_enabled_for(rclpy.logging.LoggingSeverity.DEBUG):
            self.get_logger().debug('+%d 体素' % added)

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
        return ('存档=%s 体素=%d 累积点=%d 覆盖 bbox=%s 体积=%.2f MB | '
                '帧: 成功=%d TF失败=%d(退化最新=%d) 非法点帧=%d 距离滤除点=%d | 最近点云戳=%s'
                % (os.path.join(self.pcd_dir, self.map_name), s['voxels'], s['points'], bbox,
                   s['size_mb'], self.frames_ok, self.frames_tf_fail, self.frames_tf_latest,
                   self.frames_nonfinite, self.frames_range_dropped,
                   self.last_cloud_stamp if self.last_cloud_stamp else '(还没收到点云)'))

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

    def _do_save(self):
        v = self._verify(self.guard.KIND_PCD)
        if not v['may_load'] and v['code'] != 'fresh':
            self.blocked_reason = v['code']
            return False, v['message']                       # ← 拒绝覆盖别场地的先验
        s = self.stats()
        if s['voxels'] == 0:
            return False, '内存里一个体素都没有（点云没进来 / TF 一直失败？）⇒ 不覆盖既有存档'
        art = v['archive']
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
                                               'source': 'cloud_accumulator',
                                               'tf_frames_ok': self.frames_ok,
                                               'tf_frames_failed': self.frames_tf_fail})
        size = os.path.getsize(art) / 1048576.0
        self.blocked_reason = None
        return True, ('已落盘 3D 先验：%s（%d 点 / %.2f MB；累积点 %d，voxel=%.2f m）'
                      ' + sidecar %s（world=%s）'
                      % (art, n, size, s['points'], self.voxel,
                         self.guard.manifest_path(self.base), man['world']))

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
