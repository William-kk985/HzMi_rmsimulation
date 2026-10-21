/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"
#include "gravity_align_ground.h"

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
        // ★ 2026-10-21（Part 1）：落定门开启时，每帧先更新"地面高度代理"（只读、O(N) 一次分位）。
        //   门关（init_settle_hold <= 0，**默认**）⇒ 这一行不执行 ⇒ 与改动前逐字节相同。
        if (parameters.init_settle_hold > 0.0 && !is_init) {
            update_ground_proxy(pointcloud);
        }
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    // ===================== 本仓新增（2026-10-21，Part 1）：落定门的两个函数 =====================
    // 地面高度代理 = 近场点云 z 的 **10% 分位**（车体/传感器系；只用来判"还动不动"，不用来定基）。
    //   · 为什么用分位而不是拟合平面：这里每帧都要算（10 Hz），RANSAC 太贵；而"整片点云的下沿"
    //     在车下沉时**整体**平移 ⇒ 分位数对下沉是**一阶敏感**的，对单点噪声不敏感（10% 分位 = 第
    //     N/10 个点，不是某个极值点）。
    //   · 半径窗 [1, 6] m：排除自击近场（<0.24 m）与远处稀疏点；保留地面环形回波。
    void SmallPointLio::update_ground_proxy(const std::vector<common::Point> &pointcloud) {
        if (pointcloud.empty()) {
            return;
        }
        std::vector<float> zs;
        zs.reserve(pointcloud.size());
        for (const auto &p: pointcloud) {
            const float r2 = p.position.x() * p.position.x() + p.position.y() * p.position.y();
            if (r2 < 1.0f || r2 > 36.0f) {
                continue;
            }
            zs.push_back(p.position.z());
        }
        if (zs.size() < 100) {
            return;
        }
        const size_t k = zs.size() / 10;
        std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(k), zs.end());
        const double stamp = pointcloud.back().timestamp;
        ground_proxy_hist.emplace_back(stamp, static_cast<double>(zs[k]));
        // 只保留最近 ~5 s（够 `init_settle_hold` 的最大合理取值，也不会无限长）
        while (!ground_proxy_hist.empty() && stamp - ground_proxy_hist.front().first > 5.0) {
            ground_proxy_hist.pop_front();
        }
    }

    // 返回 true = 可以初始化；false = 还**没落定**（调用方必须丢掉这帧点云，别让它进初始化地图）。
    bool SmallPointLio::init_settle_gate(double t_now) {
        if (init_settle_done) {
            return true;
        }
        if (parameters.init_settle_hold <= 0.0) {
            // 门关（默认）⇒ 立刻放行，且**不做任何清缓冲**（旧行为逐字节不变）
            init_settle_done = true;
            return true;
        }
        // ⚠️ 首帧 IMU / 点云的戳可能是 **0**（`/clock` 还没到时驱动先发的那一帧，实测）：
        //   拿它当"出生时刻"会把 elapsed 算成几百秒 ⇒ 立刻撞超时兜底（第一版就这么失败了）。
        //   ⇒ t<=0 一律不认（既不记 t0、也不判落定），等第一帧**有真戳**的数据。
        if (!(t_now > 0.0)) {
            return false;
        }
        if (!init_gate_t0_set) {
            init_gate_t0_set = true;
            init_gate_t0 = t_now;
            RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                        "初始化落定门：起点 t=%.6f s（hold=%.2f s，tol=%.4f m，超时兜底 %.1f s）",
                        t_now, parameters.init_settle_hold, parameters.init_settle_tol,
                        parameters.init_settle_timeout);
        }
        const double elapsed = t_now - init_gate_t0;
        const double hold = parameters.init_settle_hold;
        // 静止判据：最近 hold 秒内所有代理值的**峰峰值** <= tol；且窗口本身覆盖 >= 0.8*hold
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        double t_oldest = std::numeric_limits<double>::infinity();
        size_t n = 0;
        for (const auto &kv: ground_proxy_hist) {
            if (t_now - kv.first > hold) {
                continue;
            }
            lo = std::min(lo, kv.second);
            hi = std::max(hi, kv.second);
            t_oldest = std::min(t_oldest, kv.first);
            ++n;
        }
        const bool enough_frames = (n >= 5);
        const bool window_full = enough_frames && (t_now - t_oldest) >= 0.8 * hold;
        const double pp = enough_frames ? (hi - lo) : -1.0;
        if (window_full && pp <= parameters.init_settle_tol) {
            init_settle_done = true;
            init_settle_pp = pp;
            RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                        "初始化落定门：**已落定**（首帧后 %.2f s；最近 %.2f s 内地面高度代理峰峰值 "
                        "%.4f m <= tol %.4f m，%zu 帧）⇒ 放行初始化；落定前丢弃的 %zu 帧点云不进初始化地图",
                        elapsed, hold, pp, parameters.init_settle_tol, n,
                        init_settle_dropped_frames);
            return true;
        }
        if (parameters.init_settle_timeout > 0.0 && elapsed > parameters.init_settle_timeout) {
            init_settle_done = true;
            init_settle_timeout_hit = true;
            init_settle_pp = pp;
            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                        "初始化落定门：**超时 %.1f s 仍未判到静止**（最近窗口峰峰值 %.4f m > tol %.4f m）"
                        "⇒ 照旧初始化（响亮失败，不静默卡死）；`odom` 的 z 原点可能偏高，"
                        "静态边请核对 h_meas / 地面 z",
                        parameters.init_settle_timeout, pp, parameters.init_settle_tol);
            return true;
        }
        // 还没落定 ⇒ 丢掉这帧点云（初始化地图只允许取**落定之后**的帧）
        if (!preprocess.point_deque.empty() || !preprocess.dense_point_deque.empty()) {
            ++init_settle_dropped_frames;
        }
        preprocess.point_deque.clear();
        preprocess.dense_point_deque.clear();
        return false;
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            // ★ 2026-10-21（Part 1）：**先过落定门**再谈初始化。
            const double t_gate = !preprocess.imu_deque.empty()
                                  ? preprocess.imu_deque.back().timestamp
                                  : (!preprocess.point_deque.empty()
                                     ? preprocess.point_deque.back().timestamp : -1.0);
            if (t_gate < 0.0) {
                return;
            }
            if (!init_settle_gate(t_gate)) {
                return;
            }
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
                // ★★ 2026-10-21（契约 §4.14）：初始化地面互检用的 "world←IMU" 旋转。
                //   默认 = R_IL^T（= R(base_link←IMU)，因为 base_link↔livox_frame 的 rpy=0）：
                //   E1 关时（tf/base/raw）odom 就是初始 IMU 系、不是重力系，而点云在水平的
                //   livox_frame 里 ⇒ 把 "world up" 换到 IMU 系需要的就是这个旋转。
                //   重力对齐时（gravity:=lio）改用 E1 选中的 R_use（与旧 E1b 互检同一口径）。
                Eigen::Matrix<state::value_type, 3, 3> R_ws_check =
                        Eigen::Matrix<state::value_type, 3, 3>::Identity();
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
                        R_ws_check = R_use;
                        const Eigen::Matrix<state::value_type, 3, 1> rpy = rpy_deg_of(R_use);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                                    "gravity_align_init: mode=%s src=%s R_world_body rpy(deg)=[%.3f %.3f %.3f] "
                                    "Δ(URDF,IMU)=%.3f° world_gravity=[%.3f %.3f %.3f] body_source=%s",
                                    mode.c_str(), used_src, rpy.x(), rpy.y(), rpy.z(), delta_deg,
                                    g_world.x(), g_world.y(), g_world.z(), body_src);

                        // ===================== E1b：z 原点（契约 v3 §4.3）=====================
                        // 默认 gravity_align_z_origin=sensor ⇒ **本段完全不执行**（position 仍为 reset() 后的 0
                        // ⇒ off 档以及旧的 hybrid 档逐位不变）。
                        // ground 档：把 odom 的 z=0 放到"初始化时刻的地面"上 ——
                        //   状态原点严格说是 **IMU body**（点先经 extrinsic_T/R 搬进 IMU 系），传感器
                        //   livox 原点在 body 系里是 t_IL（= extrinsic_T，默认 (0,0,0.05)）⇒
                        //       p_livox(world) = p_state(world) + R_ws·t_IL
                        //   要求 livox 正好在 +h（h = 传感器离地高度）⇒
                        //       position.z = h − (R_ws·t_IL).z
                        //   **推导出的期望关系**（车体水平、URDF 与实物一致、robot11 sensor 档）：
                        //       ground z=0 · base_link z=+0.102499 · livox_frame z=+0.2595
                        //       （数值：h=0.102499+0.157028=0.259527；杆臂 (R_ws·t_IL).z=0.05·cos30°=0.043301
                        //        ⇒ position.z=0.216226；base_link 再减旋转后的 t_imu←base 的 z 杆臂 0.113722
                        //        = 0.102504；livox 再加 base←livox 的 z=0.157028 ⇒ 0.259532）
                        //   ⚠️ **只改旋转不改平移 = 本 bug**（地面被抬到 −h）；反过来只改平移不改旋转也不行。
                        //   初始地图点在上面那个循环里用**同一个 kf.x.position** ⇒ 与状态同步搬，
                        //   不需要另写一份平移（写了反而会有第二个真源）。
                        //   h 只取 **URDF 值**（确定性）；下面拟合出的 h_meas 只当诊断，不参与定基。
                        if (parameters.gravity_align_z_origin == "ground") {
                            if (!(parameters.gravity_align_h > 0.0)) {
                                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                             "gravity_align_z_origin=ground 但 gravity_align_h=%.6f m 无效（应 >0，"
                                             "由 launch 从 URDF 算）⇒ 退回 **sensor** 档（odom 原点 = 初始化时刻的传感器原点，"
                                             "地面不在 z=0）", parameters.gravity_align_h);
                            } else {
                                const Eigen::Matrix<state::value_type, 3, 1> t_IL =
                                        parameters.extrinsic_est_en ? estimator.kf.x.offset_T_L_I : estimator.Lidar_T_wrt_IMU;
                                const double lever_z = (R_use * t_IL).z();
                                const double state_z = z_origin_state_z(parameters.gravity_align_h, R_use, t_IL);
                                estimator.kf.x.position.z() = state_z;
                                RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                                            "gravity_align_z: z_origin=ground h(URDF)=%.6f m lever_z=(R_ws·t_IL).z=%.6f m "
                                            "⇒ state_z=%.6f m（odom 的 z=0 = 初始化时刻的地面；初始地图点用同一平移）",
                                            parameters.gravity_align_h, lever_z, state_z);
                            }
                        } else if (parameters.gravity_align_z_origin != "sensor") {
                            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_z_origin=%s 不是 sensor|ground 之一 ⇒ 按 sensor 处理（odom 原点 = 传感器原点）",
                                        parameters.gravity_align_z_origin.c_str());
                        }
                    }
                }
                // init map
                // ⚠️ 原实现把**雷达系**的初始点直接塞进地图（等价于假设 init 时 R=I、t=0）。
                //    一旦 R=rot_init≠I（上面刚改过），初始地图会被整体转过 rot_init ⇒ 必须用与逐点
                //    更新相同的出点公式。gravity_align_mode=off 时保留原写法（默认行为不变）。
                // ★★ 2026-10-21（契约 §4.12，**按实车改安装约定**）：`gravity_align_mode=off` 这条
                //   分支原来把**雷达系**的初始点**原样**塞进地图（= 假设点云与 IMU 同姿态、同原点）。
                //   旧约定下这条假设成立（点云在传感器系、IMU 与雷达同姿态 ⇒ R_IL=I），
                //   但**新约定**里点云是**水平**的（`base_link→livox_frame` rpy=0）、IMU 是**斜**的
                //   ⇒ `R_IL = Rx(+30°) ≠ I`。此时若仍原样入图，初始地图会比世界系**整体差 R_IL**
                //   （实测：robot11 + `gravity_align_mode:=off` ⇒ `map→odom` 跑到 1e5 m、GICP 全线
                //   `score=nan` = 发散）。修法 = 与逐点更新**同一个**旋转（只差那 5 cm 平移，
                //   与改动前 off 分支的既有偏差一致、不新增）：
                //       · `R_IL == I`（默认模型 / hzmirm / 旧约定）⇒ **原表达式一个字符都不动**
                //         （`add_point(point.position)`，逐字节相同）；
                //       · `R_IL != I`（robot11 新约定）⇒ 先旋进 IMU 系再入图。
                const Eigen::Matrix<state::value_type, 3, 3> R_IL_init =
                        parameters.extrinsic_est_en ? estimator.kf.x.offset_R_L_I : estimator.Lidar_R_wrt_IMU;
                const Eigen::Matrix<state::value_type, 3, 1> T_IL_init =
                        parameters.extrinsic_est_en ? estimator.kf.x.offset_T_L_I : estimator.Lidar_T_wrt_IMU;
                const bool R_IL_is_identity =
                        (R_IL_init - Eigen::Matrix<state::value_type, 3, 3>::Identity()).norm() < static_cast<state::value_type>(1e-12);
                // ★★ 2026-10-21（Part 1 / z 锚点的**根因之一**）：`init_map_extrinsic=true` 时，初始化地图
                //   用**与逐点更新完全相同**的公式（`R_IL·p + t_IL`）。旧写法少了 `t_IL` 的平移
                //   （§29.2 的注释里登记过"只差那 5 cm 平移"），但 robot11 的 `R_IL=Rx(30°)` 把 t_IL=(0,0,0.05)
                //   变成 odom 系里的 (0, +0.025, **+0.0433**) ⇒ 初始化地图整体低 4.33 cm ⇒ 第一次点更新
                //   把状态**往下拽 4.3 cm**（实测：只加落定门、不加这个 ⇒ 地面 z 仍差 −0.043 m，见 §30.4 的 A/B）。
                //   默认 false ⇒ 旧表达式逐字节不变（回滚档/E1 臂/其它模型不受影响）。
                for (const auto &point: preprocess.point_deque) {
                    if (gravity_aligned) {
                        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
                        if (parameters.extrinsic_est_en) {
                            point_imu_frame = estimator.kf.x.offset_R_L_I * point.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                        } else {
                            point_imu_frame = estimator.Lidar_R_wrt_IMU * point.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                        }
                        estimator.ivox->add_point((estimator.kf.x.rotation * point_imu_frame + estimator.kf.x.position).cast<float>());
                    } else if (parameters.init_map_extrinsic) {
                        // 初始化那一刻状态 = (R=I, position=0) ⇒ 出点公式退化成 `R_IL·p + t_IL`
                        if (R_IL_is_identity) {
                            // 快路径（R_IL=I）：`R_IL·p + t_IL` 与 `p + t_IL` **逐位相同**，
                            // 只省一次 3x3 乘（默认模型/hzmirm；与一般式不可区分）。
                            estimator.ivox->add_point(
                                    (point.position.cast<state::value_type>() + T_IL_init).cast<float>());
                        } else {
                            estimator.ivox->add_point(
                                    (R_IL_init * point.position.cast<state::value_type>() + T_IL_init).cast<float>());
                        }
                    } else if (R_IL_is_identity) {
                        estimator.ivox->add_point(point.position);
                    } else {
                        estimator.ivox->add_point((R_IL_init * point.position.cast<state::value_type>()).cast<float>());
                    }
                }
                // ★★ 2026-10-21（契约 §4.14）：**初始化地面互检**（落定门删除后的兜底信号）。
                //   旧的 `lio_init_settle_s` 门靠"雷达地面高度代理连续稳定"判出生是否落定；§32 之后
                //   出生即落定 ⇒ 门删掉，改用这条**只读**互检：从初始帧原始云（lidar 系）拟合地面，
                //   量出 h_meas 与 URDF 的 h（launch 从 URDF 算）比：
                //     |Δ| < 0.03 m ⇒ INFO；≥ 0.03 m ⇒ **响亮 WARN**（"出生未落定/地面不平/悬空"）；
                //     ≥ 0.05 m ⇒ ERROR（仍只改日志电平，**定基永远用 URDF 值**，确定性优先）。
                //   `gravity_align_h <= 0`（launch 没给 h）⇒ 跳过（不猜）。
                //   每次初始化只打一次（reset() 只在构造期调用）⇒ 天然 rate-limited。
                if (parameters.gravity_align_h > 0.0) {
                    if (!gravity_aligned) {
                        R_ws_check = R_IL_init.transpose();
                    }
                    const GroundFitResult fit = fit_ground_plane_sensor_frame(
                            preprocess.point_deque, 0.20, R_ws_check, R_IL_init);
                    if (!fit.ok) {
                        RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                    "gravity_align_h_check: 初始帧地面拟合不可信（inliers=%d tilt=%.2f°）⇒ **跳过互检**；"
                                    "仍用 URDF 值 h=%.6f m（确定性）。若 nav 里局部高度带异常，请先查"
                                    "「出生未落定/地面不平/悬空」",
                                    fit.inliers, fit.tilt_deg, parameters.gravity_align_h);
                    } else {
                        const double delta = std::fabs(fit.height_m - parameters.gravity_align_h);
                        if (delta < 0.03) {
                            RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_h_check: h_urdf=%.4f m h_meas=%.4f m |Δ|=%.4f m < 0.03 ⇒ INFO"
                                        "（inliers=%d tilt=%.2f° resid_p50=%.4f p95=%.4f；定基用 URDF 值）",
                                        parameters.gravity_align_h, fit.height_m, delta, fit.inliers,
                                        fit.tilt_deg, fit.resid_p50_m, fit.resid_p95_m);
                        } else if (delta < 0.05) {
                            RCLCPP_WARN(rclcpp::get_logger("small_point_lio"),
                                        "gravity_align_h_check: h_urdf=%.4f m h_meas=%.4f m |Δ|=%.4f m ∈ [0.03,0.05) ⇒ **WARN**"
                                        "（**出生未落定/地面不平/悬空**？URDF 与实测不符；inliers=%d tilt=%.2f° "
                                        "resid_p50=%.4f）—— 仍用 URDF 值定基",
                                        parameters.gravity_align_h, fit.height_m, delta, fit.inliers,
                                        fit.tilt_deg, fit.resid_p50_m);
                        } else {
                            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                                         "gravity_align_h_check: h_urdf=%.4f m h_meas=%.4f m |Δ|=%.4f m >= 0.05 ⇒ ERROR"
                                         "（**出生未落定/地面不平/悬空**，或 URDF 的 livox 离地高度与实物不符；"
                                         "inliers=%d tilt=%.2f° resid_p50=%.4f resid_p95=%.4f）"
                                         "—— **仍用 URDF 值 %.4f m 定基**（确定性；实测只作诊断）",
                                         parameters.gravity_align_h, fit.height_m, delta, fit.inliers,
                                         fit.tilt_deg, fit.resid_p50_m, fit.resid_p95_m,
                                         parameters.gravity_align_h);
                        }
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
                // ★ 2026-10-21（Part 1）：初始化收据（**一行**，把"什么时候初始化、z 原点在哪、
                //   初始化地图是哪几帧建的"写死；默认档与实验档都会打，属于可观测性，不改行为）。
                RCLCPP_INFO(rclcpp::get_logger("small_point_lio"),
                            "初始化（init）：t=%.6f s（首帧后 %.3f s）| 落定门=%s（hold=%.2f s，峰峰值=%.4f m，"
                            "丢弃 %zu 帧）| 初始化地图 = %zu 点 | z 原点 = **初始化时刻的 IMU 位姿**"
                            "（`odom` 的 z=0 在这里；地面在哪由静态边 `odom→odom_level` 用 URDF 的 h_imu 给）",
                            time_current, init_gate_t0_set ? (time_current - init_gate_t0) : -1.0,
                            parameters.init_settle_hold > 0.0
                            ? (init_settle_timeout_hit ? "超时兜底" : "on") : "off",
                            parameters.init_settle_hold, init_settle_pp,
                            init_settle_dropped_frames, preprocess.point_deque.size());
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
