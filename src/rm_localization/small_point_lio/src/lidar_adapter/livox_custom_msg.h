/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#ifdef HAVE_LIVOX_DRIVER

#include "base_lidar.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace small_point_lio {

    class LivoxCustomMsgAdapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback](const livox_ros_driver2::msg::CustomMsg &msg) {
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(msg.points.size());
                        common::Point new_point;
                        // ★★ [HzMi 本地补丁 2026-10-05] 不再无条件信任 `timebase`。
                        //   上游把点时间算成 (timebase + offset_time)，只有真机 livox_ros_driver2 才保证
                        //   timebase != 0 —— 它那句 `livox_msg.timebase = pkg.base_time;` 与
                        //   `livox_msg.header.stamp = rclcpp::Time(timestamp);` 取的是**同一个值**
                        //   （src/rm_driver/livox_ros_driver2/src/src/lddc.cpp:326-331），
                        //   即 timebase ≡ header.stamp（单位 ns，绝对时间）。
                        //   而本仓的仿真插件只填 header.stamp、**从不填 timebase**（保持默认 0），
                        //   offset_time 也恒为 0（帧内无运动，见 livox_points_plugin.cpp:267）⇒
                        //   点时间会全变成 0.0 s，而 IMU 时间是仿真钟（≈数百秒）。
                        //   后果不是"精度差"而是**整条链路死掉**：small_point_lio.cpp:103 的
                        //   `if (point_lidar_frame.timestamp < time_current) { pop; continue; }` 会把
                        //   每一帧点云全部丢弃，且 :84 的 is_publish_odometry 因
                        //   `imu.back().timestamp < point.back().timestamp` 恒假 ⇒ /Odometry 与
                        //   odom→base_link **一条都不发**（实测复现见 docs/lio_slots.md §5）。
                        //   修法：timebase 为 0 时退回 header.stamp —— 与真机驱动语义完全一致，
                        //   真机/其它驱动（timebase != 0）走原路径，行为不变。
                        const double base_time = (msg.timebase != 0)
                                                         ? static_cast<double>(msg.timebase) * 1e-9
                                                         : static_cast<double>(msg.header.stamp.sec) +
                                                                   static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
                        for (const auto &point: msg.points) {
                            if ((point.tag & 0b00111111) == 0b00000000) {
                                common::Point new_point;
                                new_point.position << point.x, point.y, point.z;
                                new_point.timestamp = base_time + static_cast<double>(point.offset_time) * 1e-9;
                                pointcloud.push_back(new_point);
                            }
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio

#endif
