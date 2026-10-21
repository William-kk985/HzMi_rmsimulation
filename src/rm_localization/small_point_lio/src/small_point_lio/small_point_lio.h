/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

namespace small_point_lio {

    class SmallPointLio {
    private:
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        bool is_init = false;

        // ===== 本仓新增（2026-10-21，Part 1）：初始化"落定门"的状态 =====
        // (stamp, 地面高度代理) 历史；只在 init_settle_hold > 0 时维护（其它档零行为、零额外订阅）。
        std::deque<std::pair<double, double>> ground_proxy_hist;
        bool init_gate_t0_set = false;
        double init_gate_t0 = 0.0;                // 首次有点云/IMU 的时刻（= "出生"）
        bool init_settle_done = false;            // 门已放行（落定 / 超时兜底 / 门关）
        bool init_settle_timeout_hit = false;
        size_t init_settle_dropped_frames = 0;    // 落定前丢弃的点云帧数（收据里报出来）
        double init_settle_pp = -1.0;             // 放行时窗口内代理的峰峰值（收据用）

        void update_ground_proxy(const std::vector<common::Point> &pointcloud);

        bool init_settle_gate(double t_now);

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

    private:
        void publish_odometry(double timestamp);
    };

}// namespace small_point_lio
