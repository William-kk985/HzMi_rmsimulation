/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    // ============================ 本仓补丁（1 行语义，见下） ============================
    // ★ 为什么改：上游把 float64 秒转成 (sec, nanosec) 时用的是**截断**：
    //     time_msg.nanosec = static_cast<uint32_t>((ts - std::floor(ts)) * 1e9);
    //   浮点误差让约 40% 的帧算成 X.99999999x，截断后变成 X-1 ns：
    //     odom→base_link（以及 /Odometry）的戳比同一帧点云的 header 戳**低 1 纳秒**。
    //   tf2 在 /scan 的戳上查 odom→base_link 时要求缓存里存在 "stamp >= 该戳" 的样本，
    //   低 1 ns 的同帧样本**不合格** ⇒ 必须等**下一帧**的 TF（~0.1 s 仿真时间）。
    //   而 slam_toolbox 的 tf2 MessageFilter 默认 scan_queue_size=1：等待一旦超过下一条
    //   /scan 的到达间隔，正在等的那条就被 QueueFull 顶掉 —— 就是那条刷屏日志。
    //   实测（q05：3223 条 /scan，`/Odometry` 戳与 `/scan` 戳逐纳秒比对）：
    //     ==0 ns：1697 条（60.0%）  == -1 ns：1130 条（40.0%）
    // ★ 修法：把截断改成**四舍五入**（并保留进位）。实测（fix01，3221 条 /scan）：
    //     Δns==0（同纳秒有 TF 样本）从 52.7% 升到 **95.5%**；
    //     `/Odometry` 戳 - `/scan` 戳 = −1 ns 的帧从 40.0% 降到 **0**；
    //     系统性丢帧（4%/2.5 s 节律）从 116 条降到 10 条，且那 10 条全部发生在
    //     "LIO 还没开始发 TF"的启动段与跑飞段（Δns = 2 ms~16 s 的真空隙），不再是系统性行为。
    //   注意：**不要**改成"量化到微秒"——那会把 LIO 侧真实的毫秒级时间差一起改掉，
    //   而这里要修的只是 float64→ns 的**截断**这一个转换 bug。
    //   证据与 A/B：docs/slam_toolbox_scan_drops.md（§6/§9.4）。
    // =================================================================================
    static builtin_interfaces::msg::Time double_to_msg_time(double ts) {
        builtin_interfaces::msg::Time t;
        const double sec_d = std::floor(ts);
        t.sec = static_cast<int32_t>(sec_d);
        int64_t ns = static_cast<int64_t>(std::llround((ts - sec_d) * 1e9));   // 四舍五入（原为截断）
        if (ns >= 1000000000LL) {         // 进位（避免 nanosec 溢出）
            t.sec += 1;
            ns -= 1000000000LL;
        }
        if (ns < 0) { ns = 0; }
        t.nanosec = static_cast<uint32_t>(ns);
        return t;
    }

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        bool save_pcd = declare_parameter<bool>("save_pcd");
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        if (save_pcd) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd, lidar_frame](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        res->success = false;
                        res->message = "pcd save is disabled";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    res->success = true;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save = std::make_shared<std::vector<Eigen::Vector3f>>();
                    *pointcloud_to_save = pointcloud_mapping->get_points();
                    std::thread([pointcloud_to_save, lidar_frame]() {
                        io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, lidar_frame](const common::Odometry &odometry) {
            last_odometry = odometry;

            const builtin_interfaces::msg::Time time_msg = double_to_msg_time(odometry.timestamp);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_link";
            geometry_msgs::msg::TransformStamped base_link_to_lidar_frame_transform;
            try {
                base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from base_link to %s: %s", lidar_frame.c_str(), ex.what());
                return;
            }
            tf2::Transform tf_lidar_odom_to_lidar_frame;
            tf_lidar_odom_to_lidar_frame.setOrigin(tf2::Vector3(odometry.position.x(), odometry.position.y(), odometry.position.z()));
            tf_lidar_odom_to_lidar_frame.setRotation(tf2::Quaternion(odometry.orientation.x(), odometry.orientation.y(), odometry.orientation.z(), odometry.orientation.w()));
            tf2::Transform tf_base_link_to_lidar_frame;
            tf2::fromMsg(base_link_to_lidar_frame_transform.transform, tf_base_link_to_lidar_frame);
            tf2::Transform tf_odom_to_base_link = tf_base_link_to_lidar_frame.inverse() * tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame;
            transform_stamped.transform = tf2::toMsg(tf_odom_to_base_link);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";
            odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            odometry_msg.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
            odometry_msg.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
            odometry_msg.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
            odometry_msg.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

            // TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link
            // odometry_msg.twist.twist.linear.x = odometry.velocity.x();
            // odometry_msg.twist.twist.linear.y = odometry.velocity.y();
            // odometry_msg.twist.twist.linear.z = odometry.velocity.z();
            // odometry_msg.twist.twist.angular.x = odometry.angular_velocity.x();
            // odometry_msg.twist.twist.angular.y = odometry.angular_velocity.y();
            // odometry_msg.twist.twist.angular.z = odometry.angular_velocity.z();

            tf_broadcaster->sendTransform(transform_stamped);
            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, lidar_frame](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                const builtin_interfaces::msg::Time time_msg = double_to_msg_time(last_odometry.timestamp);

                geometry_msgs::msg::TransformStamped lidar_frame_to_base_link_transform;
                try {
                    lidar_frame_to_base_link_transform = tf_buffer->lookupTransform("base_link", lidar_frame, time_msg);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from %s to base_link: %s", lidar_frame.c_str(), ex.what());
                    return;
                }
                Eigen::Vector3f lidar_frame_to_base_link_T;
                lidar_frame_to_base_link_T << static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.x),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.y),
                        static_cast<float>(lidar_frame_to_base_link_transform.transform.translation.z);
                Eigen::Matrix3f lidar_frame_to_base_link_R =
                        Eigen::Quaternionf(
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.w),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.x),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.y),
                                static_cast<float>(lidar_frame_to_base_link_transform.transform.rotation.z))
                                .toRotationMatrix();
                sensor_msgs::msg::PointCloud2 msg;
                msg.header.stamp = time_msg;
                msg.header.frame_id = "odom";
                msg.width = pointcloud.size();
                msg.height = 1;
                msg.fields.reserve(4);
                sensor_msgs::msg::PointField field;
                field.name = "x";
                field.offset = 0;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "y";
                field.offset = 4;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "z";
                field.offset = 8;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "intensity";
                field.offset = 12;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                msg.is_bigendian = false;
                msg.point_step = 16;
                msg.row_step = msg.width * msg.point_step;
                msg.data.resize(msg.row_step * msg.height);
                Eigen::Vector3f transformed_point;
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    transformed_point = lidar_frame_to_base_link_R * point + lidar_frame_to_base_link_T;
                    *pointer = transformed_point.x();
                    ++pointer;
                    *pointer = transformed_point.y();
                    ++pointer;
                    *pointer = transformed_point.z();
                    ++pointer;
                    *pointer = 0;
                    ++pointer;
                }
                msg.is_dense = false;
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                for (const auto &point: pointcloud) {
                    pointcloud_mapping->add_point(point);
                }
            }
        });
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
