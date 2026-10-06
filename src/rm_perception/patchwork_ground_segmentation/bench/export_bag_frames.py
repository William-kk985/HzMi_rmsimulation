#!/usr/bin/env python3
"""把 rosbag2 里的点云帧导成裸 float32 .bin（Nx3），供 ground_seg_ab 台架做同帧 A/B。

为什么不用 PCD：台架只要 x/y/z，裸 bin 没有头/字段解析、没有 PCL IO 版本差异，
且能保证"两个分割器吃的是同一段内存"。

用法：
    python3 export_bag_frames.py --bag .tmp_bags/ret4 --topic /livox/lidar/pointcloud \
        --out .tmp_ground_ab/ret4 --max-frames 80

输出：<out>/frame_000000.bin ... 以及 <out>/index.txt（帧号、stamp、点数、全零点数、z 中位数）

⚠ 本脚本**不**过滤任何点：全零填充点（如果这个 bag 是插件修复前录的）照原样导出，
   由台架自己统计并报告（台架会把它们计进"全零点"）。理由：真实链路里 linefit/patchwork
   也会收到它们，过滤掉就等于换了一份输入。
"""

import argparse
import os
import sys
import struct
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--topic', default='/livox/lidar/pointcloud')
    ap.add_argument('--out', required=True)
    ap.add_argument('--max-frames', type=int, default=80)
    ap.add_argument('--stride', type=int, default=1, help='每 N 帧取 1 帧（默认全取）')
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    os.makedirs(args.out, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    idx = 0
    saved = 0
    with open(os.path.join(args.out, 'index.txt'), 'w') as fi:
        fi.write('# frame stamp_sec n_points n_zero z_median frame_id\n')
        while reader.has_next() and saved < args.max_frames:
            topic, raw, stamp = reader.read_next()
            if idx % args.stride:
                idx += 1
                continue
            idx += 1
            msg = deserialize_message(raw, PointCloud2)
            pts = np.array(
                [[p[0], p[1], p[2]] for p in
                 point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)],
                dtype=np.float32)
            if pts.size == 0:
                continue
            n_zero = int(np.sum(np.all(pts == 0.0, axis=1)))
            nz = pts[~np.all(pts == 0.0, axis=1)]
            zmed = float(np.median(nz[:, 2])) if nz.size else float('nan')
            path = os.path.join(args.out, 'frame_%06d.bin' % saved)
            pts.tofile(path)
            fi.write('%d %.9f %d %d %.6f %s\n' % (
                saved, stamp / 1e9, pts.shape[0], n_zero, zmed, msg.header.frame_id))
            saved += 1
    print('导出 %d 帧 -> %s' % (saved, args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
