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
        // ===== 本仓新增（2026-10-21，Part 1：z 锚点的**根因**）=====
        // `init_settle_hold` > 0 ⇒ 开启"落定门"：**必须连续静止这么久（仿真秒）才允许初始化**。
        //   为什么：出生时车在**匀速下沉**（`gazebo_ros_planar_move` 每个物理步把 vz 清零 ⇒
        //   实测 ~5.4 cm/s、robot11 的 9.75 cm 落差 ~1.8 s 沉完），而旧初始化门只看
        //   "点云 >= init_map_size 个点 + 200 帧 IMU"（≈2 s）⇒ 初始化那一刻 `point_deque`
        //   里攒着**整段下沉过程**的点云，而初始化地图又**不带状态**直接入图
        //   ⇒ 地图里的地面被"抹"成 ~9 cm 厚的一条带 ⇒ 第一次点更新把状态**拽下去**，
        //   `odom` 的 z 原点比 URDF 的 h_imu 高 ~0.13 m（实测见 docs §30）。
        //   ⚠️ 判据**不能**用加速度计：`planar_move` 反复重置速度 ⇒ 匀速下沉期间 |a| 仍≈1 g，
        //      而静止时 |a| 本身在 7.8–11.8 m/s² 抖（实测）⇒ |a|≈g 既不能证明静止、也不稳。
        //   所以判据用**雷达自己看到的地面高度代理**：近场点云 z 的 10% 分位
        //   （车下沉 1 cm 它就变 1 cm；静止时峰峰值实测 <=1.5 mm）。
        //   0 = 关（**默认**，旧行为逐字节不变：不算代理、不清缓冲）。
        double init_settle_hold = 0.0;
        //: 落定判据的容差（米）：最近 `init_settle_hold` 秒内"地面高度代理"的**峰峰值**上限。
        double init_settle_tol = 0.005;
        //: 落定门的超时兜底（仿真秒；<=0 = 不兜底）。超时 ⇒ 响亮 WARN + 照旧初始化（绝不静默卡死）。
        double init_settle_timeout = 15.0;
        //: 初始化地图是否用**与逐点更新同一个**出点公式（含 `extrinsic_T` 平移）。
        //  旧行为（false）= 只旋 `R_IL`、**不加** `t_IL`（= 既有 5 cm 偏差，§29.2 的注释里登记过）；
        //  true ⇒ 初始化地图与逐点更新完全同口径（robot11：`t_IL` 在 odom 里的 z 分量 = +4.33 cm）。
        bool init_map_extrinsic = false;
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
