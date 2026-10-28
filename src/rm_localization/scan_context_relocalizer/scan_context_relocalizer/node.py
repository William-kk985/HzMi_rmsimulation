#!/usr/bin/env python3
"""scan_context_relocalizer_node —— `global_relocalizer:=scan_context` 槽的节点。

职责（契约 §4.16 / I3 / I4）：
  1. 取一帧激光（默认 `/cloud_registered`，odom 系）→ 用 **TF（odom→odom_level）**摆平到
     `level_frame`，再减去 `base_link` 原点 ⇒ 与建库**完全同源**的描述子输入；
  2. 查 keyframe DB（`PCD/<world>_sc_db.npz`）→ SC 距离 + 次优裕度做闸门；
  3. 通过 ⇒ 发 **`/initialpose`**（PoseWithCovarianceStamped，map 系机器人位姿）+ `~/health`；
     不通过 ⇒ **不发** `/initialpose`，`~/health` 明说 no-match（`valid=false`）；
  4. **绝不发任何 TF**：没有 TransformBroadcaster，`map→odom` 的唯一发布者仍是
     `localization:=gicp|small_gicp|icp|amcl|…`（契约 I3/I4）。
  5. 启动自动跑一次（`auto_start`），也可用 `~/relocalize`（std_srvs/Trigger）随时重跑。

参数见 `config/scan_context_relocalizer_sim.yaml`；DB 格式见 `db.py` 顶部与 docs §4.16。
"""
from __future__ import annotations

import json
import math
import os
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import db as scdb
from . import scan_context as SC


# --------------------------------------------------------------------------- small SE(3)
def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_to_mat(ts):
    t = ts.transform.translation
    q = ts.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def yaw_of(T):
    return math.atan2(float(T[1, 0]), float(T[0, 0]))


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ScanContextRelocalizer(Node):
    def __init__(self):
        super().__init__('scan_context_relocalizer')

        p = self.declare_parameter
        self.db_path = str(p('db_path', '').value)
        self.scan_topic = str(p('scan_topic', '/cloud_registered').value)
        self.map_frame = str(p('map_frame', 'map').value)
        self.base_frame = str(p('base_frame', 'base_link').value)
        self.level_frame = str(p('level_frame', 'odom_level').value)
        self.max_candidates = int(p('max_candidates', 20).value)
        self.candidate_min_sep = float(p('candidate_min_sep', 1.0).value)
        self.min_score = float(p('min_score', 0.70).value)
        self.min_margin = float(p('min_margin', 0.01).value)
        self.max_sc_dist = float(p('max_sc_dist', 0.60).value)
        self.min_scan_points = int(p('min_scan_points', 500).value)
        self.publish_initialpose = bool(p('publish_initialpose', True).value)
        self.publish_candidates = bool(p('publish_candidates', True).value)
        self.auto_start = bool(p('auto_start', True).value)
        self.auto_start_delay_sec = float(p('auto_start_delay_sec', 8.0).value)
        self.auto_start_timeout_sec = float(p('auto_start_timeout_sec', 90.0).value)
        self.health_rate_hz = float(p('health_rate_hz', 1.0).value)
        self.tf_timeout_sec = float(p('tf_timeout_sec', 0.30).value)

        # ---- DB（缺文件 = 硬报错：launch 本应在更早一步给出可操作提示）----
        if not self.db_path:
            raise RuntimeError(
                "参数 db_path 为空。`global_relocalizer:=scan_context` 需要先建 SC keyframe DB：\n"
                "  ros2 run scan_context_relocalizer build_sc_db -- --bag <mapping bag/raw> "
                "--world <world> --out <PCD/<world>_sc_db.npz>\n"
                "（格式/命令见 docs/frame_contract_v3.md §4.16）")
        self.db = scdb.ScanContextDB.load(self.db_path)

        # ---- ROS 接口 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # `/cloud_registered` 是 LIO 的 10 Hz 点云（BEST_EFFORT 订阅即可；发布端 RELIABLE 也兼容）
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, self.scan_topic, self._on_scan, qos)

        self.pub_initpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_health = self.create_publisher(
            DiagnosticStatus, '~/health', 10)
        self.pub_candidates = self.create_publisher(
            MarkerArray, '~/candidates', 1)
        self.srv = self.create_service(Trigger, '~/relocalize', self._on_relocalize)

        self._lock = threading.Lock()
        self._last_msg = None
        self._last_status = None
        self._last_hypothesis = None
        self._last_quality = 0.0
        self._last_reason = '等待第一帧点云'
        self._last_state = 'IDLE'
        self._last_stamp = None
        self._last_extra = {}
        self._attempts = 0
        self._accepted = 0

        self._t0 = self.get_clock().now()
        self._auto_pending = bool(self.auto_start)
        self._auto_done = False
        self._shutdown_status = False

        self.create_timer(0.5, self._tick)
        self.create_timer(1.0 / max(0.2, self.health_rate_hz), self._publish_health)
        self._publish_health()

        self.get_logger().info(
            'scan_context_relocalizer 起：db=%s（%d keyframes）scan=%s level=%s base=%s map=%s\n'
            '  闸门：score>=%.3f（= SC 距离 <= %.3f）margin>=%.3f max_sc_dist=%.3f '
            'topK=%d min_sep=%.2f m\n'
            '  契约：只发 /initialpose + ~/health + ~/candidates；**不发 TF**（map→odom 归 local tracker）'
            % (self.db_path, len(self.db.poses), self.scan_topic, self.level_frame,
               self.base_frame, self.map_frame, self.min_score,
               (1.0 / self.min_score - 1.0) if self.min_score > 0 else float('inf'),
               self.min_margin, self.max_sc_dist, self.max_candidates,
               self.candidate_min_sep))
        meta = self.db.meta
        self.get_logger().info(
            '  DB meta：world=%s bag=%s created=%s kf_rule=%s'
            % (meta.get('world', '?'), meta.get('source_bag', '?'),
               meta.get('created_at', '?'), meta.get('keyframe_rule', '?')))

    # ------------------------------------------------------------------ scan
    def _on_scan(self, msg):
        self._last_msg = msg
        if self._auto_pending:
            self._maybe_auto()

    def _tick(self):
        if self._auto_pending:
            self._maybe_auto()

    def _maybe_auto(self):
        now = self.get_clock().now()
        elapsed = (now - self._t0).nanoseconds * 1e-9
        if elapsed < self.auto_start_delay_sec:
            return
        if elapsed > self.auto_start_timeout_sec:
            self._auto_pending = False
            self._auto_done = True
            self._set_no_match('auto_start 超时：没有拿到可用的点云/TF')
            return
        if self._last_msg is None:
            return
        result = self._run_once(trigger='auto_start')
        if result in ('no_scan', 'no_tf'):
            return  # 保留 pending，下一 tick 再试
        self._auto_pending = False
        self._auto_done = True

    # ------------------------------------------------------------------ match
    def _run_once(self, trigger):
        with self._lock:
            msg = self._last_msg
            if msg is None:
                self._set_no_match('还没有收到 %s' % self.scan_topic)
                return 'no_scan'
            self._attempts += 1
            self._set_state('RELOCALIZING', 'attempt=%d trigger=%s' % (self._attempts, trigger))
            t0 = self.get_clock().now()

            stamp = Time.from_msg(msg.header.stamp) if msg.header.stamp.sec else Time()
            cloud_frame = msg.header.frame_id.lstrip('/') or self.level_frame
            try:
                T_lc = self._lookup(self.level_frame, cloud_frame, stamp)
                T_lb = self._lookup(self.level_frame, self.base_frame, stamp)
            except Exception as e:  # noqa: BLE001 - TF 未就绪是正常启动态
                self._set_no_match('TF 未就绪（%s←%s / %s←%s）：%s'
                                   % (self.level_frame, cloud_frame,
                                      self.level_frame, self.base_frame, e))
                return 'no_tf'

            base_yaw_level = yaw_of(T_lb)
            self._last_base_yaw_level = base_yaw_level
            try:
                pts = np.asarray(
                    pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True),
                    dtype=np.float64)
            except Exception as e:  # noqa: BLE001
                self._set_no_match('点云解析失败：%s' % e)
                return 'bad_cloud'
            if pts.ndim != 2 or pts.shape[0] < self.min_scan_points:
                self._set_no_match('点太少（%d < min_scan_points=%d）'
                                   % (0 if pts.ndim != 2 else pts.shape[0], self.min_scan_points))
                return 'no_scan'

            p_level = pts @ T_lc[:3, :3].T + T_lc[:3, 3]
            # ★ 描述子原点 = base_link（与建库同源；见 db.py 顶部）
            q = p_level - T_lb[:3, 3]
            qdesc = SC.descriptor(q, (0.0, 0.0), 0.0, SC.NUM_RINGS, SC.NUM_SECTORS)
            nnz = int(np.count_nonzero(qdesc))
            if nnz < 10:
                self._set_no_match('描述子有效 bin 只有 %d 个（几何太少/点云异常）' % nnz)
                return 'bad_cloud'

            res = self.db.query(qdesc, topk=self.max_candidates,
                                min_sep=self.candidate_min_sep)
            best_d = res['best_dist']
            margin = res['margin']
            score = 1.0 / (1.0 + best_d)
            accepted = (score >= self.min_score and best_d <= self.max_sc_dist
                        and margin >= self.min_margin)
            T_hyp, yaw_rel = self.db.hypothesis(
                res['best_index'], res['shift'], base_yaw_in_level=base_yaw_level)
            dt_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6

            if accepted:
                self._accepted += 1
                self._last_hypothesis = T_hyp
                self._last_quality = score
                self._last_stamp = msg.header.stamp
                self._last_base_yaw_level = base_yaw_level
                self._last_state = 'MATCH'
                self._last_reason = ('SC d=%.3f score=%.3f margin=%.3f '
                                     'kf=%d shift=%.1f°' % (
                                         best_d, score, margin, res['best_index'],
                                         math.degrees(SC.shift_to_yaw(res['shift']))))
                self._last_extra = {
                    'sc_distance': '%.6f' % best_d,
                    'score': '%.6f' % score,
                    'margin': '%.6f' % margin,
                    'best_index': str(res['best_index']),
                    'second_distance': ('%.6f' % res['second_dist']
                                        if math.isfinite(res['second_dist']) else 'inf'),
                    'shift_deg': '%.2f' % math.degrees(SC.shift_to_yaw(res['shift'])),
                    'hypothesis_yaw_deg': '%.2f' % math.degrees(yaw_of(T_hyp)),
                    'hypothesis_xyz': '%.3f,%.3f,%.3f' % (T_hyp[0, 3], T_hyp[1, 3], T_hyp[2, 3]),
                }
                self._publish_health(force=True)
                if self.publish_initialpose:
                    self._publish_initialpose(T_hyp, msg.header.stamp)
                if self.publish_candidates:
                    self._publish_candidates_markers(res)
                self.get_logger().info(
                    '[scan_context] %s MATCH d=%.3f score=%.3f margin=%.3f '
                    'hypothesis map=(%.2f, %.2f, %.2f, %.1f°) (%d kf, %.0f ms)'
                    % (trigger, best_d, score, margin,
                       T_hyp[0, 3], T_hyp[1, 3], T_hyp[2, 3],
                       math.degrees(yaw_of(T_hyp)), len(self.db.poses), dt_ms))
                return 'match'

            self._last_quality = score
            self._last_stamp = msg.header.stamp
            self._last_extra = {
                'sc_distance': '%.6f' % best_d,
                'score': '%.6f' % score,
                'margin': '%.6f' % margin,
                'best_index': str(res['best_index']),
                'second_distance': ('%.6f' % res['second_dist']
                                    if math.isfinite(res['second_dist']) else 'inf'),
                'shift_deg': '%.2f' % math.degrees(SC.shift_to_yaw(res['shift'])),
            }
            self._set_no_match(
                'no-match：SC d=%.3f（score=%.3f < %.3f）margin=%.3f（< %.3f）'
                % (best_d, score, self.min_score, margin, self.min_margin))
            self.get_logger().warn(
                '[scan_context] %s NO-MATCH d=%.3f score=%.3f margin=%.3f '
                '（best kf=%d, %.0f ms）⇒ 不发布 /initialpose'
                % (trigger, best_d, score, margin, res['best_index'], dt_ms))
            return 'no_match'

    # ------------------------------------------------------------------ tf
    def _lookup(self, target, source, stamp):
        # 先按点云时间戳查；启动初期 TF 还没覆盖到该戳时退"最新可用"（与 gicp 的语义一致）
        try:
            return transform_to_mat(self.tf_buffer.lookup_transform(
                target, source, stamp, Duration(seconds=self.tf_timeout_sec)))
        except Exception:  # noqa: BLE001
            return transform_to_mat(self.tf_buffer.lookup_transform(
                target, source, Time(), Duration(seconds=self.tf_timeout_sec)))

    # ------------------------------------------------------------------ publish
    def _publish_initialpose(self, T_map_base, stamp_msg):
        m = PoseWithCovarianceStamped()
        m.header.stamp = stamp_msg
        m.header.frame_id = self.map_frame
        p = m.pose.pose
        p.position.x = float(T_map_base[0, 3])
        p.position.y = float(T_map_base[1, 3])
        p.position.z = float(T_map_base[2, 3])
        cy, sy = math.cos(yaw_of(T_map_base) * 0.5), math.sin(yaw_of(T_map_base) * 0.5)
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = float(sy)
        p.orientation.w = float(cy)
        # 粗位姿协方差：位置 0.5 m / z 0.2 m；roll-pitch 5°，yaw 8°（§41 的候选原始误差 0.01–0.20 m）
        cov = [0.0] * 36
        cov[0] = 0.5 ** 2
        cov[7] = 0.5 ** 2
        cov[14] = 0.2 ** 2
        cov[21] = math.radians(5.0) ** 2
        cov[28] = math.radians(5.0) ** 2
        cov[35] = math.radians(8.0) ** 2
        m.pose.covariance = cov
        self.pub_initpose.publish(m)
        self.get_logger().info(
            '[scan_context] /initialpose ← map 系 base_link (%.3f, %.3f, %.3f, yaw %.2f°) '
            'frame=%s（cov xy 0.5 m / yaw 8°）'
            % (p.position.x, p.position.y, p.position.z,
               math.degrees(yaw_of(T_map_base)), self.map_frame))

    def _publish_candidates_markers(self, res):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for rank, (d, i, shift) in enumerate(res['candidates'][:self.max_candidates]):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'sc_candidates'
            m.id = rank
            m.type = Marker.ARROW
            m.action = Marker.ADD
            T = self.db.poses[int(i)]
            yaw = yaw_of(T) + SC.query_yaw_from_shift(int(shift), self.db.num_sectors)
            m.pose.position.x = float(T[0, 3])
            m.pose.position.y = float(T[1, 3])
            m.pose.position.z = 0.10
            m.pose.orientation.z = math.sin(yaw * 0.5)
            m.pose.orientation.w = math.cos(yaw * 0.5)
            m.scale.x = 0.6
            m.scale.y = 0.10
            m.scale.z = 0.10
            if rank == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.5, 0.0, 0.6
            arr.markers.append(m)
        self.pub_candidates.publish(arr)

    # ------------------------------------------------------------------ health
    def _set_state(self, state, reason):
        self._last_state = state
        self._last_reason = reason

    def _set_no_match(self, reason):
        self._last_state = 'NO_MATCH'
        self._last_reason = reason
        self._publish_health(force=True)

    def _publish_health(self, force=False):
        st = DiagnosticStatus()
        st.name = 'global_relocalizer=scan_context'
        st.hardware_id = self.db_path
        ok = self._last_state == 'MATCH'
        st.level = DiagnosticStatus.OK if ok else (
            DiagnosticStatus.WARN if self._last_state in ('NO_MATCH', 'IDLE') else DiagnosticStatus.OK)
        st.message = ('MATCH' if ok else self._last_state) + '：' + self._last_reason
        dbg = self.db.meta
        vals = {
            'valid': 'true' if ok else 'false',
            'quality': '%.6f' % self._last_quality,
            'state': self._last_state,
            'source': 'global_relocalizer=scan_context',
            'reason': self._last_reason,
            'attempts': str(self._attempts),
            'accepted': str(self._accepted),
            'n_keyframes': str(len(self.db.poses)),
            'db_world': str(dbg.get('world', '')),
            'db_legacy_no_level_yaws': str(dbg.get('legacy_no_level_yaws', False)),
            'base_yaw_in_level_deg': ('%.2f' % math.degrees(getattr(self, '_last_base_yaw_level', 0.0))),
            'db_path': self.db_path,
            'stamp': ('%d.%09d' % (self._last_stamp.sec, self._last_stamp.nanosec)
                      if self._last_stamp is not None else ''),
            'publish_initialpose': str(self.publish_initialpose),
            'gate_min_score': '%.3f' % self.min_score,
            'gate_min_margin': '%.3f' % self.min_margin,
            'gate_max_sc_dist': '%.3f' % self.max_sc_dist,
        }
        vals.update(self._last_extra)
        for k, v in vals.items():
            st.values.append(KeyValue(key=k, value=v))
        self._last_status = st
        self.pub_health.publish(st)

    # ------------------------------------------------------------------ service
    def _on_relocalize(self, req, resp):
        try:
            result = self._run_once(trigger='service')
            resp.success = (result == 'match')
            resp.message = json.dumps({
                'result': result,
                'state': self._last_state,
                'reason': self._last_reason,
                'attempts': self._attempts,
                'accepted': self._accepted,
                'db': self.db_path,
                'n_keyframes': len(self.db.poses),
            }, ensure_ascii=False)
        except Exception as e:  # noqa: BLE001
            resp.success = False
            resp.message = 'relocalize 异常：%r' % e
            self.get_logger().error('[scan_context] 服务调用异常：%r' % e)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanContextRelocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
