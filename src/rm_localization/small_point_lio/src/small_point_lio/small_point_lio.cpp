/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                // ===================== 实验补丁：初始化时对齐重力（E1） =====================
                // gravity_align_mode=off（**默认**）⇒ 本段完全不执行、rotation 保持单位阵
                //   ⇒ 与原实现逐字节相同（下面 init map / acceleration 也走原分支）。
                // 三种来源（都只在初始化那一刻用一次）：
                //   urdf  ：用参数 gravity_align_rpy（= URDF 里雷达相对 base_link 的安装 rpy，
                //           即"车体水平时雷达在世界里的姿态"）当先验 ⇒ 不需要 IMU、立刻可用；
                //           前提 = 出生时车体水平（仿真里成立；真机上就是"已知安装角 ±1°"）。
                //   imu   ：实测重力方向（body 系）↔ world −z 的最小旋转（= point_lio 的 Set_init）。
                //   hybrid：两者都算，Δ = 两旋转夹角；Δ<2° 用 URDF(INFO) / 2~5° 用 IMU(WARN) /
                //           ≥5° 用 IMU(ERROR)。Δ 大 = URDF 先验与实测不符（轴写错/车没停平）。
                // 对齐后三处必须一起设（否则 v̇ = R·acc + g ≠ 0）：
                //   R = rot_init、g = rot_init·g_body(≈(0,0,−|g|))、acc = −Rᵀ·g（= body 系比力）。
                bool gravity_aligned = false;
                if (parameters.gravity_align_mode != "off") {
                    const Eigen::Matrix<state::value_type, 3, 1> down_world = parameters.gravity.cast<state::value_type>();
                    // ---- (1) body 系"重力向下"方向 ----
                    Eigen::Matrix<state::value_type, 3, 1> down_body = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    bool have_body = false;
                    const char *body_src = "none";
                    if (parameters.gravity_init_set) {
                        down_body = parameters.gravity_init.cast<state::value_type>();
                        have_body = true;
                        body_src = "gravity_init";
                    } else if (parameters.fix_gravity_direction) {
                        down_body = estimator.kf.x.gravity;// 上面刚算好的 200 帧均值（−|g|·mean_acc/|mean_acc|）
                        have_body = true;
                        body_src = "imu_mean200";
                    } else if (!preprocess.imu_deque.empty()) {
                        for (const auto &imu_msg: preprocess.imu_deque) {
                            down_body += imu_msg.linear_acceleration.cast<state::value_type>();
                        }
                        down_body = -down_body;// 加速度计测比力（向上）⇒ 取反 = 重力向下
                        have_body = true;
                        body_src = "imu_deque";
                    }
                    if (have_body && down_body.norm() > static_cast<state::value_type>(1e-6)) {
                        down_body *= down_world.norm() / down_body.norm();// 归一到 |g|
                    } else {
                        have_body = false;
                    }
                    const auto rot_from_vectors = [](const Eigen::Matrix<state::value_type, 3, 1> &a,
                                                     const Eigen::Matrix<state::value_type, 3, 1> &b) {
                        const Eigen::Matrix<state::value_type, 3, 1> axis = a.cross(b);
                        const state::value_type s = axis.norm();
                        const state::value_type c = a.dot(b);
                        if (s <= static_cast<state::value_type>(1e-12)) {
                            return Eigen::Matrix<state::value_type, 3, 3>(Eigen::Matrix<state::value_type, 3, 3>::Identity());
                        }
                        return Eigen::Matrix<state::value_type, 3, 3>(exp<state::value_type>(axis / s * std::atan2(s, c)));
                    };
                    const auto rot_angle_deg = [](const Eigen::Matrix<state::value_type, 3, 3> &r1,
                                                  const Eigen::Matrix<state::value_type, 3, 3> &r2) {
                        const state::value_type c = (r1.transpose() * r2).trace();
                        return std::acos(std::clamp<state::value_type>((c - static_cast<state::value_type>(1)) / static_cast<state::value_type>(2), -1, 1)) *
                               static_cast<state::value_type>(57.29577951308232);
                    };
                    // ⚠️ 返回类型必须**显式**写成 Matrix：若让 lambda 推导，返回的是 Eigen 表达式
                    //   (CwiseUnaryOp)，它引用上面的局部 rpy ⇒ 出作用域后悬垂（实测日志打成 0.000）。
                    const auto rpy_deg_of = [](const Eigen::Matrix<state::value_type, 3, 3> &r)
                            -> Eigen::Matrix<state::value_type, 3, 1> {
                        Eigen::Matrix<state::value_type, 3, 1> rpy;
                        rpy << std::atan2(r(2, 1), r(2, 2)),
                                std::asin(std::clamp<state::value_type>(-r(2, 0), -1, 1)),
                                std::atan2(r(1, 0), r(0, 0));
                        return Eigen::Matrix<state::value_type, 3, 1>(rpy * static_cast<state::value_type>(57.29577951308232));
                    };
                    // ---- (2) 两个候选旋转 ----
                    const Eigen::Matrix<state::value_type, 3, 3> R_imu = have_body ? rot_from_vectors(down_body, down_world)
                                                                                   : Eigen::Matrix<state::value_type, 3, 3>::Identity();
                    Eigen::Matrix<state::value_type, 3, 3> R_urdf = Eigen::Matrix<state::value_type, 3, 3>::Identity();
                    if (parameters.gravity_align_rpy_set) {
                        constexpr state::value_type d2r = static_cast<state::value_type>(0.017453292519943295);
                        const Eigen::Matrix<state::value_type, 3, 1> rpy = parameters.gravity_align_rpy.cast<state::value_type>() * d2r;
                        R_urdf = (Eigen::AngleAxis<state::value_type>(rpy.z(), Eigen::Matrix<state::value_type, 3, 1>::UnitZ()) *
                                  Eigen::AngleAxis<state::value_type>(rpy.y(), Eigen::Matrix<state::value_type, 3, 1>::UnitY()) *
                                  Eigen::AngleAxis<state::value_type>(rpy.x(), Eigen::Matrix<state::value_type, 3, 1>::UnitX()))
                                         .toRotationMatrix();
                    }
                    // 没有实测时用 URDF 先验反推 body 系重力（= 假设车体水平）
                    const Eigen::Matrix<state::value_type, 3, 1> g_body_use =
                            have_body ? down_body : (parameters.gravity_align_rpy_set ? Eigen::Matrix<state::value_type, 3, 1>(R_urdf.transpose() * down_world) : down_world);
                    // ---- (3) 选哪个 ----
                    const std::string &mode = parameters.gravity_align_mode;
                    Eigen::Matrix<state::value_type, 3, 3> R_use = Eigen::Matrix<state::value_type, 3, 3>::Identity();
                    const char *used_src = "none";
                    state::value_type delta_deg = static_cast<state::value_type>(-1);
                    bool use = false;
                    if (mode == "urdf") {
                        if (parameters.gravity_align_rpy_set) {
                            R_use = R_urdf;
                            use = true;
                            used_src = "urdf";
                        } else if (have_body) {
                            R_use = R_imu;
                            use = true;
                            used_src = "imu(fallback:no_rpy)";
                            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_mode=urdf 但 gravity_align_rpy 没给 ⇒ 回退到 IMU 均值");
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                         "gravity_align_mode=urdf 但既没有 gravity_align_rpy 也没有 IMU 数据 ⇒ 无法对齐");
                        }
                    } else if (mode == "imu") {
                        if (have_body) {
                            R_use = R_imu;
                            use = true;
                            used_src = body_src;
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                         "gravity_align_mode=imu 但没有可用的 body 系重力（gravity_init 空且无 IMU 数据）⇒ 无法对齐");
                        }
                    } else if (mode == "hybrid") {
                        if (parameters.gravity_align_rpy_set && have_body) {
                            delta_deg = rot_angle_deg(R_urdf, R_imu);
                            if (delta_deg < static_cast<state::value_type>(2.0)) {
                                R_use = R_urdf;
                                used_src = "urdf";
                                RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                                            "gravity_align_init_hybrid: Δ(URDF,IMU)=%.3f° < 2° ⇒ 用 URDF 先验", delta_deg);
                            } else if (delta_deg < static_cast<state::value_type>(5.0)) {
                                R_use = R_imu;
                                used_src = "imu(delta_warn)";
                                RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                            "gravity_align_init_hybrid: Δ(URDF,IMU)=%.3f° ∈ [2,5)° ⇒ URDF 先验可疑，**改用 IMU**", delta_deg);
                            } else {
                                R_use = R_imu;
                                used_src = "imu(delta_error)";
                                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                             "gravity_align_init_hybrid: Δ(URDF,IMU)=%.3f° >= 5° ⇒ URDF 先验与实测不符（轴写错/车未停平），"
                                             "**改用 IMU**；请核对 gravity_align_rpy=[%.3f %.3f %.3f]",
                                             delta_deg, parameters.gravity_align_rpy.x(), parameters.gravity_align_rpy.y(), parameters.gravity_align_rpy.z());
                            }
                            use = true;
                        } else if (parameters.gravity_align_rpy_set) {
                            R_use = R_urdf;
                            use = true;
                            used_src = "urdf(fallback:no_imu)";
                            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_init_hybrid: 没有实测重力 ⇒ 只能用 URDF 先验、跳过互检");
                        } else if (have_body) {
                            R_use = R_imu;
                            use = true;
                            used_src = "imu(fallback:no_rpy)";
                            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_init_hybrid: 没给 gravity_align_rpy ⇒ 只能用 IMU、跳过互检");
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                         "gravity_align_init_hybrid: URDF 先验与 IMU 数据都没有 ⇒ 无法对齐");
                        }
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                    "gravity_align_mode=%s 不是 off|urdf|imu|hybrid 之一 ⇒ 按 off 处理", mode.c_str());
                    }
                    if (use) {
                        const Eigen::Matrix<state::value_type, 3, 1> g_world = R_use * g_body_use;
                        estimator.kf.x.rotation = R_use;
                        estimator.kf.x.gravity = g_world;
                        gravity_aligned = true;
                        const Eigen::Matrix<state::value_type, 3, 1> rpy = rpy_deg_of(R_use);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                                    "gravity_align_init: mode=%s src=%s R_world_body rpy(deg)=[%.3f %.3f %.3f] "
                                    "Δ(URDF,IMU)=%.3f° world_gravity=[%.3f %.3f %.3f] body_source=%s",
                                    mode.c_str(), used_src, rpy.x(), rpy.y(), rpy.z(), delta_deg,
                                    g_world.x(), g_world.y(), g_world.z(), body_src);
                    }
                }
                // init map
                // ⚠️ 原实现把**雷达系**的初始点直接塞进地图（等价于假设 init 时 R=I、t=0）。
                //    一旦 R=rot_init≠I（上面刚改过），初始地图会被整体转过 rot_init ⇒ 必须用与逐点
                //    更新相同的出点公式。gravity_align_mode=off 时保留原写法（默认行为不变）。
                for (const auto &point: preprocess.point_deque) {
                    if (gravity_aligned) {
                        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
                        if (parameters.extrinsic_est_en) {
                            point_imu_frame = estimator.kf.x.offset_R_L_I * point.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                        } else {
                            point_imu_frame = estimator.Lidar_R_wrt_IMU * point.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                        }
                        estimator.ivox->add_point((estimator.kf.x.rotation * point_imu_frame + estimator.kf.x.position).cast<float>());
                    } else {
                        estimator.ivox->add_point(point.position);
                    }
                }
                if (gravity_aligned) {
                    estimator.kf.x.acceleration = -(estimator.kf.x.rotation.transpose() * estimator.kf.x.gravity);
                } else {
                    estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                }
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                estimator.kf.update_point();

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                estimator.ivox->add_point(estimator.point_odom_frame);

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
