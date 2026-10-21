/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

namespace small_point_lio {

    // ===================== 本仓新增（2026-10-20 防御）：定长向量参数的**长度守卫** =====================
    // 为什么：`gravity` / `extrinsic_T` / `extrinsic_R` 三个参数在下面都是按**固定下标**用的
    // （gravity_temp[0..2]、extrinsic_R_temp[0..8]），而 `std::vector::operator[]` 不查长度 ⇒
    //   · 键缺失：`declare_parameter<T>(name)`（无默认值）会抛 ParameterTypeException，
    //     节点在构造函数里直接 std::terminate（实测 = SIGABRT，不是可控报错）；
    //   · 键在但**短了**：越界读堆上的邻居字节 —— 实测 `extrinsic_R: [1,0,0]` / `extrinsic_T: [0.05]`
    //     都能"正常启动"，只是拿垃圾外参跑（静默错，比崩溃更难查）。
    // 修法：带默认值声明 + 长度校验；不满足 ⇒ **ERROR + 用本仓三份 config 的契约值**。
    //   ⇒ 配置写错时节点**照样能起来**并**响亮报错**，绝不越界读、绝不 terminate。
    //   ⚠️ 正常配置（长度对）走的分支与改动前**逐字节相同**：只是把 vector 原样拷出来。
    static std::vector<double> read_fixed_vector(rclcpp::Node &node, const char *name, size_t want,
                                                 const std::vector<double> &fallback) {
        const rclcpp::Logger logger = rclcpp::get_logger("small_point_lio");
        // 先看**参数文件里到底有没有这个键**：`declare_parameter<T>(name, default)` 在缺键时
        // 会静默用 default（这不响）；而旧写法 `declare_parameter<T>(name)`（无默认）在缺键时
        // 直接抛 ParameterTypeException ⇒ 构造函数里 std::terminate（实测 SIGABRT）。
        bool present = false;
        if (const auto params_iface = node.get_node_parameters_interface()) {
            const auto &overrides = params_iface->get_parameter_overrides();
            present = overrides.find(name) != overrides.end();
        }
        std::vector<double> v;
        try {
            v = node.declare_parameter<std::vector<double>>(name, fallback);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger,
                         "参数 %s 读取失败（%s）⇒ 用本仓契约默认值（%zu 个数），节点继续启动",
                         name, e.what(), fallback.size());
            return fallback;
        }
        if (!present) {
            RCLCPP_WARN(logger,
                        "参数 %s **没给**（参数文件里没有这个键）⇒ 用本仓契约默认值（%zu 个数）；"
                        "旧代码这里会抛 ParameterTypeException → std::terminate（实测 SIGABRT）",
                        name, fallback.size());
            return v;
        }
        if (v.size() != want) {
            RCLCPP_ERROR(logger,
                         "参数 %s 需要 %zu 个数，收到 %zu 个 ⇒ 用本仓契约默认值（%zu 个数）继续启动；"
                         "旧代码这里会越界读 std::vector（值来自堆上邻居字节 ⇒ 静默错误外参/重力）",
                         name, want, v.size(), fallback.size());
            return fallback;
        }
        return v;
    }

    void Parameters::read_parameters(rclcpp::Node &node) {
        // 点云过滤
        point_filter_num = static_cast<int>(node.declare_parameter<long>("point_filter_num"));
        auto min_distance = node.declare_parameter<float>("min_distance");
        auto max_distance = node.declare_parameter<float>("max_distance");
        min_distance_squared = min_distance * min_distance;
        max_distance_squared = max_distance * max_distance;
        space_downsample = node.declare_parameter<bool>("space_downsample");
        space_downsample_leaf_size = node.declare_parameter<float>("space_downsample_leaf_size");

        // IMU处理
        // 默认值 = 三份 config 里写的 Gazebo 重力（gravity: [0.0, 0.0, -9.810]）。
        const std::vector<double> gravity_temp = read_fixed_vector(node, "gravity", 3, {0.0, 0.0, -9.81});
        gravity << gravity_temp[0], gravity_temp[1], gravity_temp[2];
        check_satu = node.declare_parameter<bool>("check_satu");
        fix_gravity_direction = node.declare_parameter<bool>("fix_gravity_direction");
        // ===== 实验补丁（三个键都用**带默认值**的声明 ⇒ 现有 YAML 一个字节都不用改）=====
        gravity_align_mode = node.declare_parameter<std::string>("gravity_align_mode", "off");
        // E1b（契约 v3 §4.3）：z 原点约定。两个键都带默认值 ⇒ 现有 YAML 一个字节都不用改，
        // 默认 sensor = 原行为（position 保持 0，逐位不变）。
        gravity_align_z_origin = node.declare_parameter<std::string>("gravity_align_z_origin", "sensor");
        gravity_align_h = node.declare_parameter<double>("gravity_align_h", 0.0);
        // ===== 本仓新增（2026-10-21，Part 1）：初始化"落定门"+ 初始化地图口径 =====
        // 四个键**都带默认值** ⇒ 现有三份 config 一个字节都不用改；默认 = 旧行为。
        // 只由 launch 的**默认档** Node（level_frame=odom_level 且 gravity_align_mode=off）传进来。
        init_settle_hold = node.declare_parameter<double>("init_settle_hold", 0.0);
        init_settle_tol = node.declare_parameter<double>("init_settle_tol", 0.005);
        init_settle_timeout = node.declare_parameter<double>("init_settle_timeout", 15.0);
        init_map_extrinsic = node.declare_parameter<bool>("init_map_extrinsic", false);
        {
            auto rpy_temp = node.declare_parameter<std::vector<double>>("gravity_align_rpy", std::vector<double>{});
            if (rpy_temp.size() == 3) {
                gravity_align_rpy << rpy_temp[0], rpy_temp[1], rpy_temp[2];
                gravity_align_rpy_set = true;
            }
            auto gravity_init_temp = node.declare_parameter<std::vector<double>>("gravity_init", std::vector<double>{});
            if (gravity_init_temp.size() == 3) {
                gravity_init << gravity_init_temp[0], gravity_init_temp[1], gravity_init_temp[2];
                gravity_init_set = true;
            }
        }
        satu_acc = node.declare_parameter<double>("satu_acc") * 0.99;
        satu_gyro = node.declare_parameter<double>("satu_gyro") * 0.99;
        acc_norm = node.declare_parameter<double>("acc_norm");

        // 地图
        map_resolution = node.declare_parameter<float>("map_resolution");
        init_map_size = static_cast<size_t>(node.declare_parameter<long>("init_map_size"));

        // 雷达与IMU相对位姿
        // 默认值 = 本仓仿真契约（三份 config）：extrinsic_T = 雷达比 IMU 高 0.05 m、extrinsic_R = I。
        extrinsic_est_en = node.declare_parameter<bool>("extrinsic_est_en");
        const std::vector<double> extrinsic_T_temp = read_fixed_vector(node, "extrinsic_T", 3, {0.0, 0.0, 0.05});
        extrinsic_T << extrinsic_T_temp[0], extrinsic_T_temp[1], extrinsic_T_temp[2];
        const std::vector<double> extrinsic_R_temp =
                read_fixed_vector(node, "extrinsic_R", 9, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        extrinsic_R << extrinsic_R_temp[0], extrinsic_R_temp[1], extrinsic_R_temp[2],
                extrinsic_R_temp[3], extrinsic_R_temp[4], extrinsic_R_temp[5],
                extrinsic_R_temp[6], extrinsic_R_temp[7], extrinsic_R_temp[8];

        // 滤波器参数
        laser_point_cov = node.declare_parameter<double>("laser_point_cov");
        imu_meas_acc_cov = node.declare_parameter<double>("imu_meas_acc_cov");
        imu_meas_omg_cov = node.declare_parameter<double>("imu_meas_omg_cov");
        velocity_cov = node.declare_parameter<double>("velocity_cov");
        acceleration_cov = node.declare_parameter<double>("acceleration_cov");
        omg_cov = node.declare_parameter<double>("omg_cov");
        ba_cov = node.declare_parameter<double>("ba_cov");
        bg_cov = node.declare_parameter<double>("bg_cov");
        plane_threshold = node.declare_parameter<double>("plane_threshold");
        match_sqaured = node.declare_parameter<double>("match_sqaured");

        // 数据发布
        publish_odometry_without_downsample = node.declare_parameter<bool>("publish_odometry_without_downsample");
    }

}// namespace small_point_lio