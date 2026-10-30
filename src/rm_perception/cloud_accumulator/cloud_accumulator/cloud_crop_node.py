#!/usr/bin/env python3
"""cloud_crop_node —— 用**几何盒子**把车体自身的点从点云里裁掉（PCL CropBox 的等价物）。

为什么（2026-10-30）：`robot:=chassis` 的雷达平/斜装在车顶，**车体顶面就在雷达脚下 1.2 cm**
⇒ `segmentation/obstacle` 里永远有一圈"自己的车体"。之前是用 p2l 的 `min_height` 高度带
挡住它，但那个高度带是**相对 base_link** 的（base_link 离地 0.098）⇒ min_height 0.10
= 世界高度 0.198 ⇒ 把 **0.2 m 高的场地矮墙也几乎全裁掉**了。

正确做法（本节点）：在**点云层**按车体的几何盒子（AABB，base_link 系）剔除自击点，
再送 p2l ⇒ p2l 的 `min_height` 可以放回"离地 2 cm"（-0.078），矮墙完整保留、车体不残影。

盒子格式（base_link 系，米）：`[xmin, ymin, zmin, xmax, ymax, zmax, ...]`，每 6 个数一个盒子。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
import tf2_ros


class CloudCropNode(Node):
    def __init__(self):
        super().__init__('cloud_crop')
        self.declare_parameter('input_topic', '/segmentation/obstacle')
        self.declare_parameter('output_topic', '/segmentation/obstacle_cropped')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('boxes', [
            -0.228, -0.223, -0.019, 0.228, 0.223, 0.078,      # 车体
            -0.220, -0.155, -0.098, 0.220, -0.125, 0.018,     # 左轮（w1/w3）
            -0.220, 0.125, -0.098, 0.220, 0.155, 0.018,       # 右轮（w2/w4）
        ])
        self.declare_parameter('invert', False)   # False = 剔除盒内点
        # 近场半径（m，base_link 系 XY）：r_xy < min_radius 且 z ∈ [radius_z_min, radius_z_max] 的点
        # 也剔除。用于"车体 AABB 之外、但仍在车体半对角附近"的自击/近场点（盒子是轴对齐的，
        # 车体对角 0.319 m 的角落区域盒子盖不住）。0 = 关。
        self.declare_parameter('min_radius', 0.0)
        self.declare_parameter('radius_z_min', -0.12)
        self.declare_parameter('radius_z_max', 0.25)
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        raw = self.get_parameter('boxes').value
        self.boxes = [raw[i:i+6] for i in range(0, len(raw) - 5, 6)]
        self.invert = bool(self.get_parameter('invert').value)
        self.min_radius = float(self.get_parameter('min_radius').value)
        self.radius_z_min = float(self.get_parameter('radius_z_min').value)
        self.radius_z_max = float(self.get_parameter('radius_z_max').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, inp, self._cb, qos_profile_sensor_data)
        self.get_logger().info('cloud_crop: %s -> %s, target=%s, %d boxes, invert=%s'
                               % (inp, out, self.target_frame, len(self.boxes), self.invert))

    def _cb(self, msg: PointCloud2):
        # 取 xyz（float32 xyz 即可）
        n = msg.width * msg.height
        if n == 0:
            return
        data = np.frombuffer(msg.data, dtype=np.uint8)
        fields = {f.name: f for f in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            self.get_logger().warn('cloud_crop: 点云没有 xyz 字段，原样转发')
            self.pub.publish(msg)
            return
        off = {k: fields[k].offset for k in ('x', 'y', 'z')}
        step = msg.point_step
        # 步进读取：data[off::step] 是**非连续**的，不能直接 .view(float32)；
        # 用 strided ndarray 视图（等价 stride 读取）再 copy ⇒ 连续 float32
        def _read(name):
            return np.ndarray(shape=(n,), dtype=np.float32, buffer=data,
                              offset=off[name], strides=(step,)).copy()
        pts = np.stack([_read('x'), _read('y'), _read('z')], axis=1)
        # 需要把点变到 target_frame（p2l 也做这步；这里先做，保证盒子语义正确）
        frame = msg.header.frame_id.lstrip('/')
        if self.target_frame and frame != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame,
                                                     msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn('cloud_crop: TF %s<-%s 拿不到（%s），本帧跳过' % (self.target_frame, frame, e),
                                       throttle_duration_sec=2.0)
                return
            t = tf.transform.translation
            q = tf.transform.rotation
            # 四元数旋转
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            R = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
            pts = (R @ pts.T).T + np.array([t.x, t.y, t.z])
        inside = np.zeros(len(pts), dtype=bool)
        if self.min_radius > 0.0:
            r_xy = np.hypot(pts[:, 0], pts[:, 1])
            inside |= ((r_xy < self.min_radius) &
                       (pts[:, 2] >= self.radius_z_min) & (pts[:, 2] <= self.radius_z_max))
        for b in self.boxes:
            inside |= ((pts[:, 0] >= b[0]) & (pts[:, 0] <= b[3]) &
                       (pts[:, 1] >= b[1]) & (pts[:, 1] <= b[4]) &
                       (pts[:, 2] >= b[2]) & (pts[:, 2] <= b[5]))
        keep = (~inside) if not self.invert else inside
        msg.header.frame_id = self.target_frame or frame
        # 用 numpy 重建点云（只保留 xyz + 原 intensity 若存在）
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.is_bigendian = msg.is_bigendian
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.is_dense = msg.is_dense
        out.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
        kept = pts[keep]
        inten = np.zeros(len(kept), dtype=np.float32)
        arr = np.zeros((len(kept), 4), dtype=np.float32)
        arr[:, :3] = kept
        arr[:, 3] = inten
        out.data = arr.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CloudCropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
