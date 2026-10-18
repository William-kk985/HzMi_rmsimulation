/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

// ============================================================================
// odometry_convention.cpp —— 本仓新增（2026-10-20，契约 v3 §2.1 / I2：选项 U）
//
// 只实现 `odom_convention:=sensor` 这一档的 `/Odometry` 发布：
//   · `body`  （默认，= 现状）：`/Odometry` = **合成** `T_ob = T_ol·T_bl`（姿态合成、平移有意保留旧值），
//                                与同一帧广播的 `odom→base_link` **逐位相同**（原实现，本文件不参与）；
//   · `sensor`（新档）        ：`/Odometry` = **LIO 原始状态** `T_ol`（= odom←IMU body），
//                                本节点**不广播任何 TF** ⇒ `odom→base_link` 由 `lio_tf_adapter` 负责，
//                                全栈**只有一个**发布者（契约 v3 I2）。
//
// ★ 为什么这段代码在独立 TU（**不是风格问题，是实测逼出来的**）：
//   本包编了 `-ffast-math -march=native`。把新的浮点代码放进 `small_point_lio_node.cpp`（odom 回调
//   所在 TU）会让 GCC 改变该 hot path 的 IPA/内联决策 ⇒ `/odom` 与 `odom→base_link` 出现
//   ≤1.11e-16（1 ulp）的差（与 §N.5 守卫那次同源，证据见 docs/tilted_lidar_fidelity.md §O.2）。
//   搬到独立 TU 后：**默认路径不调用本文件的任何函数** ⇒ 与改动前二进制逐位相同
//   （同一输入流 bag 重放的 A/B 见 docs/frame_contract_v3_measurements.md §4）。
//
// ★ 帧名约定（**为什么 `child_frame_id` 不是 base_link**）：
//   本节点的估计器状态 `kf.x.rotation/position` 严格说是 **odom←IMU body**（点先经
//   `extrinsic_R/T` 从 lidar 系搬到 IMU 系，见 small_point_lio.cpp 的出点公式），而本包历史上
//   一直把它当 `T_ol`（odom←`lidar_frame`）用（`lookupTransform(lidar_frame, "base_link")`）。
//   sensor 档因此写 `child_frame_id = lidar_frame`（默认 `livox_frame`，= "LIO 自己拥有的那个帧"），
//   **不写 base_link**（那是 adapter 的产物，写在这里就是第二个所有者 = 契约 I1/I2 的违例）。
//   两者之间那 5 cm（`extrinsic_T=[0,0,0.05]`，livox→imu）由 adapter 的 `xyz`（URDF 几何算出的
//   `T_imu←base_link`）吸收 ⇒ 全链路只算一次。详见 docs/frame_contract_v3.md §13。
// ============================================================================

#include "small_point_lio_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logger.hpp>

namespace small_point_lio {

    // sensor 档：把 LIO 原始状态 `T_ol` 原样发到 `/Odometry`（frame_id=odom、child=lidar_frame）。
    // 不做任何坐标合成（合成是 adapter 的事），不做 TF 查询（**本档刻意不依赖 base_link→livox_frame**
    // —— 那条 TF 缺失时旧实现会连 `/Odometry` 一起丢，契约 §4.3 已把它登记为强依赖）。
    // ⚠️ `last_odometry` 仍然要更新：点云回调的 header 戳取自它（`last_odometry.timestamp`），
    //    与 body 档保持同一个取数点 ⇒ 点云/里程计的戳关系逐纳秒不变。
    __attribute__((noinline)) void SmallPointLioNode::publish_odometry_sensor_convention(
            const common::Odometry &odometry, const builtin_interfaces::msg::Time &stamp) {
        last_odometry = odometry;

        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header.stamp = stamp;
        odometry_msg.header.frame_id = "odom";
        odometry_msg.child_frame_id = odom_child_frame_;
        odometry_msg.pose.pose.position.x = odometry.position.x();
        odometry_msg.pose.pose.position.y = odometry.position.y();
        odometry_msg.pose.pose.position.z = odometry.position.z();
        odometry_msg.pose.pose.orientation.x = odometry.orientation.x();
        odometry_msg.pose.pose.orientation.y = odometry.orientation.y();
        odometry_msg.pose.pose.orientation.z = odometry.orientation.z();
        odometry_msg.pose.pose.orientation.w = odometry.orientation.w();
        // twist 仍然留空：与 body 档一致（上游 TODO：那是 lidar_odom→lidar_frame 的量，
        // 要做就得先定义清楚在哪个系里，见 small_point_lio_node.cpp 的 TODO）。
        odometry_publisher->publish(odometry_msg);
    }

}// namespace small_point_lio
