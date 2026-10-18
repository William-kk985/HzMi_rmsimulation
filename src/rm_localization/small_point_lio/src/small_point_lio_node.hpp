/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "lidar_adapter/base_lidar.h"
#include "small_point_lio/small_point_lio.h"
#include "util/pointcloud_mapping.h"
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pch.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;
        // ================== 本仓新增：外参预条件守卫（§N.5 #1/#2，见 docs/tilted_lidar_fidelity.md §N.5 与 §O） ==================
        // ★ 为什么：两个 latent bug（合成被写成共轭③、对已配准的点再变换②）在 LiDAR→body 外参
        //   **只有平移**时数值上完全不可见（`T_bl` 纯平移 ⇒ 共轭与合成的姿态逐位相同）；一旦
        //   `robot11_mount:=urdf|sensor` 把倾角记进关节（`R_bl = Rx(±30°)`）就会静默出错。
        //   这里把"节点依赖的外参前提"变成**开机就能看到、写错就刷 ERROR** 的东西：
        //   #1 启动期一次性 readout（rpy + |t|）；#2 每帧位姿一致性断言 `T_ob·T_bl⁻¹ ≈ T_ol`。
        // ⚠️ 两者都是**诊断**（只写日志 / 计数），**不中止节点、不改变任何数值行为**。
        bool extrinsic_guard_logged_ = false;                       // #1 一次性日志是否已打
        std::uint64_t consistency_violations_ = 0;                  // #2 累计违例帧数（只计不抛）
        std::chrono::steady_clock::time_point last_consistency_error_{};  // #2 限速（1 条 / 5 s，墙钟）
        // #2 的输入快照：odom 回调里只**存**这两个已发布的 msg（14 个 double 的 POD 拷贝，
        //   无函数调用、无新浮点表达式），断言本身在**独立 TU** `src/extrinsic_guard.cpp` 里做
        //   （为什么必须独立 TU：见该文件头与 docs/tilted_lidar_fidelity.md §O.2 —— 放在本 TU 里
        //   会让 `/odom`+`odom→base_link` 出现 ≤1.11e-16 的 1 ulp 差；搬走后逐位 0 差异）。
        geometry_msgs::msg::Transform guard_T_ob_;
        geometry_msgs::msg::Transform guard_T_bl_;

        std::string lidar_frame_;                       // lidar_frame 参数（readout 日志要用）
        void log_extrinsic_readout_once();             // #1：一次性 readout（noinline）
        void check_pose_consistency();                 // #2：读上面的快照 + last_odometry 做断言（noinline）

        // ============ 本仓新增（契约 v3 §2.1 / I2：统一输出契约 = 选项 U，2026-10-20）============
        // 两个新参数（**默认值 = 现有行为，逐字节不变**）：
        //   publish_tf       (bool,   默认 true )：false ⇒ 不广播 odom→base_link（/Odometry 照发）。
        //   odom_convention  (string, 默认 body )：body   ⇒ /Odometry = 合成 T_ob（现状）；
        //                                          sensor ⇒ /Odometry = LIO 原始状态 T_ol（传感器壳体系）。
        // 目的：让 `lio_tf_adapter` 成为 odom→base_link 的**唯一发布者**（契约 I2），
        //       本节点只发"它自己拥有的那一段"（odom→传感器/IMU body）。
        // ★ 为什么新档的实现放在**独立 TU** `src/odometry_convention.cpp`（不是风格问题）：
        //   本包编了 `-ffast-math -march=native`。把新的浮点代码放进本 TU 会改变 GCC 对 odom
        //   回调（= 浮点 hot path）的 IPA/内联决策 ⇒ 发布数值出现 ≤1.11e-16 的 1 ulp 差
        //   （与 §N.5 守卫那次完全同源，见 docs/tilted_lidar_fidelity.md §O.2）。
        //   默认路径（body）**不调用**新 TU 的任何函数 ⇒ 与改动前二进制逐位相同（回归证据见
        //   docs/frame_contract_v3_measurements.md §4）。
        bool publish_tf_ = true;                 // 生效值（sensor 档会被强制成 false）
        bool odom_convention_sensor_ = false;    // odom_convention == "sensor"
        std::string odom_child_frame_;           // sensor 档 /Odometry 的 child_frame_id（= lidar_frame）
        // sensor 档的发布实现（**独立 TU**，out-of-line，不参与本 TU 任何浮点表达式的代码生成）
        void publish_odometry_sensor_convention(const common::Odometry &odometry,
                                                const builtin_interfaces::msg::Time &stamp);

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);
    };

}// namespace small_point_lio
