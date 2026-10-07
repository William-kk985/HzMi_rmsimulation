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
            // ★★ 2026-10-09 修复（bug ③，见 docs/tilted_lidar_fidelity.md §I.5.3 与 §J.1）：
            //   原来这里是**相似变换（共轭）**：`T_bl⁻¹ · T_ol · T_bl`，其中
            //     T_bl = lookupTransform(lidar_frame, "base_link") = T(livox_frame ← base_link)
            //          = "base_link 在 livox_frame 里的位姿"（本变量名就是这个意思）；
            //     T_ol = T(odom ← livox_frame) = 本函数上面的 LIO 状态
            //          （small_point_lio.cpp 的出点公式 `p_odom = R_ol·(extrinsic_R·p_lidar + extrinsic_T) + t_ol`
            //           定的就是这个语义）。
            //   **姿态**的正确值只能由坐标映射链式法则推：
            //     p_odom = T_ol · p_livox = T_ol · (T_bl · p_base)  ⇒  T_odom←base = T_ol · T_bl
            //   共轭只在 T_bl 是**纯平移**时"姿态碰巧对"；一旦倾角记进关节
            //   （`robot11_mount:=urdf|sensor`，T_bl 的旋转 = R_x(+30°)）就给出**假俯仰**：
            //   实测（`robot:=robot11 robot11_mount:=urdf`、车静止，.tmp_tiltmount/tm_urdf/）：
            //     发布的 odom→base_link rpy = [0.383, **4.890**, 7.701]°、pitch 随 yaw 一起长大
            //     （`/odom` 窗口内 pitch 跨度 4.924°，真值 `/odom_ground_truth` 只有 0.003°）；
            //     正演 R_bl·R_ol·R_blᵀ 与实测 rpy 逐位相同 ⇒ 机理钉死（nav2 拿到的车身姿态是假的）。
            //   本行 = **合成**（正确姿态）。正确性由 tools/scripts/tiltmount/tf2_compose_order_test.cpp 钉住
            //   （非单位 T_bl 下满足位姿一致性 `T_ob·T_bl⁻¹ == T_ol` + 逐点坐标恒等式）。
            tf2::Transform tf_odom_to_base_link = tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame;
            //   ⚠️⚠️ **平移故意保留旧写法（共轭）的值** —— 这是一个**实测过的、有意的**取舍，不是漏改：
            //     合成的平移 = t_ol − R_ol·p（p = livox 原点在 base_link 里的坐标 =
            //     (0.000562, 0.130916, 0.157028)，见 TF base_link→livox_frame）**才是物理真值**；
            //     共轭的平移 = t_ol + (I − R_ol)·p，两者相差**恰好 −p（0.2044 m）**。
            //     但把平移也改成真值会让**默认档（plugin）与默认模型**的局部代价图**整张变空**，
            //     机理与实测（2026-10-09，`.tmp_tiltmount/{tm_plugin,j1_plugin,j1_default}/`）：
            //       · nav2 的 `obstacle_layer.scan` **没有**配 `min_obstacle_height`
            //         ⇒ nav2 默认 0.0，而这条带子是量在**代价图帧 = odom** 里的
            //         （`rm_navigation/params/nav2_params_sim_base.yaml:128` global_frame: odom，
            //           :187 的注释写明设计假设"odom 的 z=0 ≈ base_link 起始高度、地面约 −0.05"）；
            //       · `/scan` 是一张**二维平盘**、盘面过它自己的帧原点（`livox_frame`）
            //         ⇒ 它落在 odom 里的 z = TF(odom←livox_frame) 的 z；
            //       · 实测 scan 盘面 z(odom)：旧平移 **+0.057…+0.084 m（0/949 在带外 ⇒ 全部进图）**；
            //         新平移（= 物理真值） **−0.102…−0.077 m（950/950 在带外 ⇒ 一条都不进图）**；
            //       · 后果：局部代价图 lethal **471 → 0**、inscribed **15909 → 0**、整张图全 free
            //         （默认模型同向：445 → 0、11029 → 0）⇒ 车会"看不见任何障碍"。
            //     为什么不能在这一提交里一并修：那条 `min_obstacle_height` 属于
            //     `src/rm_navigation/**/params`（**本任务禁改**），而它对**默认模型**同样生效
            //     ⇒ 任何"只改本 LIO"的方案都会破坏默认路径。⇒ **平移的完整修法 + 代价图高度带
            //     重新定基（把 odom z 基准或两条带子对齐到地面）登记为后续项**（§J.5/§J.8）。
            //     保守做法：姿态正确（这是用户实测到的那个 bug：假俯仰），平移与旧版**逐位相同**
            //     ⇒ 默认档/默认模型的行为**逐字节不变**（回归证据见 §J.2）。
            const tf2::Transform tf_legacy_odom_to_base_link =
                    tf_base_link_to_lidar_frame.inverse() * tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame;
            tf_odom_to_base_link.setOrigin(tf_legacy_odom_to_base_link.getOrigin());
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

                // ★★ 2026-10-09 修复（bug ②，见 docs/tilted_lidar_fidelity.md §I.5.1/§J.1）：
                //   进来的这朵云**已经在 odom 系里**了 —— small_point_lio.cpp 的出点公式
                //     `p_odom = R_ol · (extrinsic_R·p_lidar + extrinsic_T) + t_ol`
                //   用的是 LIO 自己的状态 (R_ol, t_ol)，与下面 `frame_id = "odom"` 一致。
                //   原来这里**又**查了一次 TF 并做 `p_pub = R(base_link←livox_frame)·p_odom + t`：
                //     · 默认档（`robot11_mount:=plugin`）那条 TF 的旋转是单位阵 ⇒ 只把整朵云平移了
                //       0.20 m（t = (0.000562, 0.130916, 0.157028)），肉眼/单帧统计都看不出来；
                //     · 倾角一记进关节（`urdf` 档）那条 TF 就带 R_x(−30°) ⇒ **整个 odom 场景被
                //       刚性旋转 30°**：实测 `/cloud_registered` 的地面倾角 30.970°，而把这朵云按
                //       同一条 TF 反变换回去只剩 **1.041°**（.tmp_tiltmount/tm_urdf）——这就是
                //       用户"点云像平放扫到的东西被倾斜了"的直接来源。
                //   改法 = 什么都不做（点云与 `frame_id = "odom"` 本来就自洽）+ 不再查 TF。
                //   ⚠️ 影响面：默认档这朵云会**整体移动 −t（0.20 m）**回到真正的 odom 坐标；
                //      帧内几何（地面倾角/点数/扇区分布）逐项不变（回归见 §J.2 的 before/after 表）。
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
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    *pointer = point.x();
                    ++pointer;
                    *pointer = point.y();
                    ++pointer;
                    *pointer = point.z();
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
