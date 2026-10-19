/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

// ============================================================================
// gravity_align_ground.h —— 本仓新增（2026-10-21，契约 v3 §4.3「odom 的 z 原点约定」）
//
// E1b：`gravity_align_z_origin:=ground` 时把 **初始化时刻的地面** 放到 odom 的 z=0：
//   ① `z_origin_state_z()`：算出初始状态位置 z（= 让**传感器 livox 原点**落在 +h 的那个值）；
//   ② `fit_ground_plane_sensor_frame()`：运行时互检 —— 从初始帧原始云拟合地面，量 h_meas。
// 两个函数都放在**独立 TU**（与 `odometry_convention.cpp` / `extrinsic_guard.cpp` 同一理由）：
// 本包用 `-ffast-math -march=native`，把新的浮点代码塞进 `small_point_lio.cpp`（逐帧 hot path
// 所在 TU）会让 GCC 改变 IPA/内联决策 ⇒ 默认路径（gravity_align_mode:=off）可能差 1 ulp。
// 独立 TU ⇒ 默认路径**不调用本文件任何函数**（源码级证据 = 本文件的两个符号不在 off 档的
// 调用图里；二进制级证据 = 同一输入流 bag 重放逐位相同，见 measurements §10）。
// ============================================================================

#pragma once

#include <common/common.h>
#include <deque>
#include <Eigen/Dense>

namespace small_point_lio {

    // 地面拟合结果（全部在**传感器/lidar 系**里量）。
    struct GroundFitResult {
        bool ok = false;        // 拟合可信（内点足够 + 法向与先验一致 + 高度在物理范围内）
        double height_m = 0.0;  // 传感器原点到地面的**垂距** = h_meas（m）
        double tilt_deg = 0.0;  // 拟合法向 vs 由 R_ws 预测的"上"的夹角（度）；小 = 确实是地面
        int inliers = 0;
        double resid_p50_m = 0.0;
        double resid_p95_m = 0.0;
    };

    // 初始状态位置 z，使**传感器（livox）原点**正好在 odom 的 +h：
    //   状态原点严格说是 IMU body（点先经 extrinsic_R/T 搬进 IMU 系，见 small_point_lio.cpp
    //   的出点公式）⇒   p_livox(world) = p_state(world) + R_ws · t_IL
    //   令其 z = h  ⇒   position.z = h − (R_ws · t_IL).z
    // robot11（sensor/urdf 挂法、h=0.259527、R_ws=Rx(−30°)）：
    //   (R_ws·t_IL).z = 0.05·cos30° = 0.043301 ⇒ position.z = 0.216226；
    //   随后（车体水平时）odom→base_link z = 0.102504、odom→livox_frame z = 0.259532，
    //   地面正好在 odom z = 0（URDF 几何；实测偏差由 ② 报出来）。
    double z_origin_state_z(double h, const Eigen::Matrix3d &R_ws, const Eigen::Vector3d &t_IL);

    // 从初始帧原始云（`point.position` 在 lidar 系）拟合地面，返回传感器离地高度。
    //   · 只用最近 `max_dt_s` 秒的点（0 = 全部）—— 初始化前有 ~2 s 的 IMU 门，点云跨多帧，
    //     取最后一帧可以避开出生落定窗口内的运动模糊；
    //   · 法向先验 = `R_IL^T · R_ws^T · e_z`（world up 在 lidar 系）⇒ 不会把墙/车身拟合进来；
    //   · RANSAC 固定种子（确定性）+ 内点 SVD 精修。
    GroundFitResult fit_ground_plane_sensor_frame(
            const std::deque<common::Point> &points,
            double max_dt_s,
            const Eigen::Matrix3d &R_ws,
            const Eigen::Matrix3d &R_IL);

}// namespace small_point_lio
