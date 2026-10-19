/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        // ===== 实验补丁（默认关；见 docs/odom_base_link_practice_survey.md §9）=====
        // gravity_align_mode：off（默认，原行为）| urdf（URDF 先验）| imu（IMU 均值）| hybrid（先验+互检）
        // gravity_align_rpy ：URDF 先验 = "车体水平时雷达在世界里的姿态" rpy(度)，即 URDF 关节 rpy（如 "-30 0 0"）
        // gravity_init      ：可选，body 系"重力向下"（= 静止时加速度计均值的反向）；给了就不必再等 IMU 均值
        // gravity_align_z_origin：E1b 的 z 原点约定（契约 v3 §4.3）
        //   sensor（**默认 = 现状**）：odom 原点 = 初始化时刻的传感器/IMU 原点（position=0，一字不改）
        //   ground                 ：odom 的 z=0 = 初始化时刻的地面（position.z = h − (R_ws·t_IL).z）
        // gravity_align_h：**传感器（livox_frame 原点）离地高度**（米），由 launch 从 URDF 几何算，
        //                  只在 z_origin=ground 时使用；≤0 ⇒ 该档不可用（ERROR + 退回 sensor）。
        std::string gravity_align_z_origin;
        double gravity_align_h = 0.0;
        std::string gravity_align_mode;
        bool gravity_align_rpy_set = false;
        Eigen::Vector3d gravity_align_rpy = Eigen::Vector3d::Zero();
        bool gravity_init_set = false;
        Eigen::Vector3d gravity_init = Eigen::Vector3d::Zero();
        double satu_acc;
        double satu_gyro;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double match_sqaured;

        bool publish_odometry_without_downsample = false;

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
