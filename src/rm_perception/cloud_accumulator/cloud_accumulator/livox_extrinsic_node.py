#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""livox_cloud_extrinsic —— 复刻实车 Livox 驱动器的 `extrinsic_parameter`。

为什么需要它（契约 `docs/frame_contract_v3.md` §4.12.3 / 实测 `docs/frame_contract_v3_measurements.md` §47）
    2026-10-29 起仿真按实车"分解"：
      · 物理雷达 = `livox_physical`（`base_link→livox_physical` rpy = −30°），mesh 挂这里 ⇒ **画出来是斜的**；
      · 射线传感器也挂在它上面 ⇒ 插件发出来的点云表达在**斜的物理系**里，话题 `/livox/lidar_raw`；
      · 本节点就是驱动器那一步：把点按 `Rx(+30°)`（安装倾角的逆）旋到**水平的 `livox_frame`**，
        再发 `/livox/lidar`、`frame_id = livox_frame`。
    ⇒ 下游（linefit / p2l / LIO）看到的**仍是水平点云、仍是 `/livox/lidar`**，数据链一字未改。

旋转从哪来
    默认**从 TF 读** `livox_frame ← livox_physical`（固定边，唯一真源 = URDF 的 `body_to_livox_physical`），
    所以参数不会和模型漂移；TF 一时拿不到才退回 `extrinsic_rpy` 参数（弧度，缺省 = +0.523598775598293 0 0）。

保真细节
    · 只改 x/y/z 三个字段，其余字段（intensity/tag/line…）**原样搬运**；`point_step`/`fields`/`height`/`width` 全保留；
    · `header.stamp` 原样，`frame_id` 改成 `frame_id` 参数；
    · 点云里可能含 NaN（插件不发 NaN，但保守处理：只旋转有限点）。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:  # pragma: no cover
    CustomMsg = None

try:
    import tf2_ros
except Exception:  # pragma: no cover - tf2 缺失时退回参数
    tf2_ros = None


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


class LivoxCloudExtrinsic(Node):
    def __init__(self):
        super().__init__('livox_cloud_extrinsic')
        # ★ 2026-10-29 21:5x 热修：真值链路的**原始**话题是**两种消息**：
        #   · `/livox/lidar_raw`           = livox_ros_driver2/msg/CustomMsg（LIO 吃这个）
        #   · `/livox/lidar_raw/pointcloud`= sensor_msgs/PointCloud2（linefit/p2l/accumulator 吃这个）
        #   所以本节点两个通道都要接、并且**按原类型**分别发到 `/livox/lidar` 与 `/livox/lidar/pointcloud`
        #   （只发 PointCloud2 会让 LIO 断流；只发 `/livox/lidar` 会让 linefit/accumulator 断流）。
        self.declare_parameter('input_topic', '/livox/lidar_raw')
        self.declare_parameter('input_cloud2_topic', '/livox/lidar_raw/pointcloud')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('output_cloud2_topic', '/livox/lidar/pointcloud')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('source_frame', 'livox_physical')
        self.declare_parameter('extrinsic_rpy', [0.523598775598293, 0.0, 0.0])
        self.declare_parameter('use_tf', True)

        inp = self.get_parameter('input_topic').value
        inp2 = self.get_parameter('input_cloud2_topic').value
        out = self.get_parameter('output_topic').value
        out2 = self.get_parameter('output_cloud2_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.source_frame = self.get_parameter('source_frame').value
        rpy = list(self.get_parameter('extrinsic_rpy').value)
        self.R = _rpy_to_matrix(*rpy)
        self.R_src = 'param rpy'
        if bool(self.get_parameter('use_tf').value) and tf2_ros is not None:
            self.buf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buf, self)
            self.create_timer(0.5, self._try_tf)
        else:
            self.buf = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        # 传感器数据一般是 BEST_EFFORT；两边都试：先按 best_effort 订阅（兼容插件），
        # 发布用 reliable（下游都是 reliable 订阅）。
        sqos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub2 = self.create_publisher(PointCloud2, out2, qos)
        self.sub2 = self.create_subscription(PointCloud2, inp2, self._cb_cloud2, sqos)
        if CustomMsg is not None:
            # CustomMsg：插件用默认 QoS（reliable, depth 50 量级）发
            self.pub1 = self.create_publisher(CustomMsg, out, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
            # ★ 2026-10-29 热修②：插件用 **SensorDataQoS（BEST_EFFORT）** 发 CustomMsg
            #   （实测 `ros2 topic info -v /livox/lidar_raw` ⇒ Reliability: BEST_EFFORT）。
            #   RELIABLE 订阅与它**不兼容** ⇒ 一条都收不到（第一版就是这个 bug，/livox/lidar 0 Hz）。
            self.sub1 = self.create_subscription(CustomMsg, inp, self._cb_custom, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        else:
            self.pub1 = None
            self.sub1 = None
        self.get_logger().info('livox_cloud_extrinsic: CustomMsg %s -> %s ; PointCloud2 %s -> %s '
                               '(frame_id=%s)，R 来源=%s'
                               % (inp, out, inp2, out2, self.frame_id, self.R_src))

    def _try_tf(self):
        if self.buf is None:
            return
        try:
            tf = self.buf.lookup_transform(self.frame_id, self.source_frame, rclpy.time.Time())
        except Exception:
            return
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        if self.R_src != 'tf %s<-%s' % (self.frame_id, self.source_frame):
            self.R_src = 'tf %s<-%s' % (self.frame_id, self.source_frame)
            self.get_logger().info('livox_cloud_extrinsic: 用 TF 的旋转（%s）' % self.R_src)

    def _cb_cloud2(self, msg: PointCloud2):
        fields = {f.name: f for f in msg.fields}
        try:
            ox, oy, oz = fields['x'].offset, fields['y'].offset, fields['z'].offset
        except KeyError:
            self.get_logger().error('点云里没有 x/y/z 字段，原样转发（不旋转）')
            out = msg
            out.header.frame_id = self.frame_id
            self.pub2.publish(out)
            return
        if msg.point_step % 4 != 0:
            self.get_logger().error('point_step=%d 不是 4 的倍数，原样转发' % msg.point_step)
            out = msg
            out.header.frame_id = self.frame_id
            self.pub2.publish(out)
            return
        n = msg.width * msg.height
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step).copy()
        f32 = arr.view(np.float32)
        xyz = np.column_stack([f32[:, ox // 4], f32[:, oy // 4], f32[:, oz // 4]]).astype(np.float64)
        ok = np.isfinite(xyz).all(axis=1)
        xyz[ok] = xyz[ok] @ self.R.T
        f32[:, ox // 4] = xyz[:, 0].astype(np.float32)
        f32[:, oy // 4] = xyz[:, 1].astype(np.float32)
        f32[:, oz // 4] = xyz[:, 2].astype(np.float32)
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.height, out.width = msg.height, msg.width
        out.fields, out.is_bigendian = msg.fields, msg.is_bigendian
        out.point_step, out.row_step = msg.point_step, msg.row_step
        out.is_dense = msg.is_dense
        out.data = arr.tobytes()
        self.pub2.publish(out)

    def _cb_custom(self, msg):
        """CustomMsg 通道（LIO 用）：**就地**旋转每个点的 x/y/z（不新建 3 万个消息对象），
        再按原类型发到 `/livox/lidar`。"""
        R = self.R
        (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = R
        for p in msg.points:
            x, y, z = p.x, p.y, p.z
            p.x = r00 * x + r01 * y + r02 * z
            p.y = r10 * x + r11 * y + r12 * z
            p.z = r20 * x + r21 * y + r22 * z
        msg.header.frame_id = self.frame_id
        self.pub1.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = LivoxCloudExtrinsic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
