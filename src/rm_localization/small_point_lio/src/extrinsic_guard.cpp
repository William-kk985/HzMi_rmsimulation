/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

// ============================================================================
// extrinsic_guard.cpp —— 本仓新增（2026-10-18，docs/tilted_lidar_fidelity.md §N.5 / §O）
//
// 外参预条件守卫（**只诊断，不改变任何发布数值、不中止节点**）：
//   #1 启动期一次性 readout：`T_bl = T(livox_frame ← base_link)` 的 rpy(度) 与 |t|，
//      按 |rpy| 分 INFO / WARN(>1e-3 rad) / ERROR(>0.05 rad) 三级；
//   #2 每帧位姿一致性断言：`T_ob ⊗ T_bl⁻¹ ≈ T_ol`（旋转严格；平移按"双合法值白名单"，
//      因为 §J.5 有意保留了旧平移）。违例**只计数 + 限速 ERROR（1 条/5 s）**，不抛、不停。
//
// ★ 为什么单独一个 TU（不是风格问题，是实测出来的）：本包编了 `-ffast-math -march=native`。
//   把这两段浮点代码放进 `small_point_lio_node.cpp`（odom 回调所在 TU）会改变 GCC 对该回调的
//   IPA/内联预算 ⇒ `tf2::Transform` 乘法的 FMA 收缩变化 ⇒ `/odom` 与 `odom→base_link` 出现
//   ≤1.11e-16 的 1 ulp 差异（`/cloud_registered` 仍逐位相同）。搬到独立 TU 后，同一输入流
//   （bag 重放）与改动前二进制**逐位 0 差异**。A/B 证据见 docs/tilted_lidar_fidelity.md §O.2。
// ============================================================================

#include "small_point_lio_node.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace small_point_lio {

    // ===================== 本仓新增：外参预条件守卫的阈值（§N.5 #1/#2） =====================
    // 设计出处 docs/tilted_lidar_fidelity.md §N.5（第 1/2 条）；实测与插入点见 §O。
    // ★ 全部是**诊断阈值**：超了只写日志/计数，**不抛异常、不中止、不改任何数值**。
    //   （用户明确否决"停/联锁"式行为 —— 这是"把隐藏前提喊出来"，不是安全联锁。）
    static constexpr double k_rad_to_deg = 180.0 / M_PI;
    // #1 启动期一次性 readout：|rpy(T_bl)| 的三个阈值（rad）。
    //   1e-3 rad ≈ 0.057°：外参"名义上是单位阵"的工程容差（URDF 关节的浮点表示误差远小于它）。
    static constexpr double k_extrinsic_warn_rpy_rad = 1e-3;    // ⇒ WARN：外参有旋转
    static constexpr double k_extrinsic_error_rpy_rad = 0.05;   // ⇒ ERROR：大旋转（≈2.9°）
    // #2 每帧位姿一致性断言 `T_ob·T_bl⁻¹ ≈ T_ol`：
    //   · 旋转：严格（合成写法下逐位成立，实测残差 ~1e-16 rad；共轭回归会立刻变成 4.9°/30° 级）；
    //   · 平移：本仓**有意**保留 §J.5 的旧平移（与合成真值差恰好一个杆臂 |p| = 0.2044 m），
    //     所以整条等式在平移上**永远不会**成立 ⇒ 这里对平移用"双合法值白名单"
    //     （合成真值 or 有意保留的旧值，两者本回调里都已算出），而不是断言等式；
    //     谁写了第三套平移公式才会触发。理由与代价见 §O.2。
    static constexpr double k_consistency_rot_tol_rad = 1e-6;   // 旋转残差容差（≈5.7e-5°）
    static constexpr double k_consistency_trans_tol_m = 1e-4;   // 平移残差容差（对"最近合法值"）

    // ============== 本仓新增（§N.5 守卫 #1）：启动期**一次性**外参 readout ==============
    // 用**节点本来就已经查到的**那条 TF（`lookupTransform(lidar_frame, "base_link")`，第一帧成功
    // 查到后的快照 `guard_T_bl_`），**不新增**任何 TF 查询/定时器/订阅；打一条日志，随后永不重复。
    // 打的就是 `T_bl = T(livox_frame ← base_link)`（= "base_link 在 livox_frame 里的位姿"，
    // 与合成 `T_ol·T_bl` 用的是**同一个**矩阵，记法见 §I.5.3/§J.1）。
    // 级别 = 按最大者（同一条日志，三个级别都含 rpy° 与 |t|）：
    //   恒 INFO；|rpy| > 1e-3 rad(≈0.057°) ⇒ WARN；|rpy| > 0.05 rad(≈2.9°) ⇒ ERROR。
    // ★ 为什么必须"喊出来"：`R_bl = I` 时 bug ③（共轭）与 bug ②（对已配准点再变换）
    //   在数值上完全不可见；`R_bl = Rx(±30°)` 时分别表现为"假俯仰 4.89°"与"整朵云被转 30°"。
    // ★ 为什么这段代码在本文件（独立 TU）（§O.2 A/B 实测）：把它放回 odom 回调所在的 TU
    //   （实测过两种放法：odom 回调内、点云回调内）都会让 `/odom`+`odom→base_link` 出现
    //   ≤1.11e-16 的 1 ulp 差异；搬到本文件后，同一输入流重放与改动前**逐位 0 差异**。
    __attribute__((noinline)) void SmallPointLioNode::log_extrinsic_readout_once() {
        if (extrinsic_guard_logged_) {
            return;
        }
        extrinsic_guard_logged_ = true;
        const tf2::Quaternion q_bl(guard_T_bl_.rotation.x, guard_T_bl_.rotation.y,
                                   guard_T_bl_.rotation.z, guard_T_bl_.rotation.w);
        double roll_bl, pitch_bl, yaw_bl;
        tf2::Matrix3x3(q_bl).getRPY(roll_bl, pitch_bl, yaw_bl);
        const double max_abs_rpy = std::max({std::fabs(roll_bl), std::fabs(pitch_bl), std::fabs(yaw_bl)});
        const double lever_arm = std::hypot(std::hypot(guard_T_bl_.translation.x,
                                                       guard_T_bl_.translation.y),
                                            guard_T_bl_.translation.z);
        const rclcpp::Logger logger = rclcpp::get_logger("small_point_lio");
        const char *tail =
                "。本节点按**合成** T_odom←base = T_ol·T_bl 发布姿态（共轭写法只在 T_bl 纯平移时碰巧对）"
                "；每帧另做一次 T_ob·T_bl⁻¹ ≈ T_ol 一致性断言（≈4 次 4×4 乘法，微基准 24.4 ns/帧，只计数不停机）"
                "。详见 docs/tilted_lidar_fidelity.md §N.5/§O";
        if (max_abs_rpy > k_extrinsic_error_rpy_rad) {
            RCLCPP_ERROR(logger,
                         "外参守卫(§N.5#1)：T_bl=T(%s←base_link) rpy=[%.6f, %.6f, %.6f]° (=%.6f rad)、|t|=%.6f m"
                         " ⇒ **外参大旋转**（>0.05 rad≈2.9°）：请核对 odom→base_link 的合成序与点云帧一致性"
                         "（/cloud_registered 的 frame_id=odom 是否真的在 odom 里）；"
                         "odom 自身斜（=初始化时刻的传感器系）时该倾角会出现在 base_link 的 roll/pitch 上，"
                         "那是物理事实而不是 bug%s",
                         lidar_frame_.c_str(), roll_bl * k_rad_to_deg, pitch_bl * k_rad_to_deg,
                         yaw_bl * k_rad_to_deg, max_abs_rpy, lever_arm, tail);
        } else if (max_abs_rpy > k_extrinsic_warn_rpy_rad) {
            RCLCPP_WARN(logger,
                        "外参守卫(§N.5#1)：T_bl=T(%s←base_link) rpy=[%.6f, %.6f, %.6f]° (=%.6f rad)、|t|=%.6f m"
                        " ⇒ **外参有旋转**（>1e-3 rad≈0.057°）：合成路径必须正确；"
                        "若这里退回共轭（bug ③）会出现随 yaw 长大的**假俯仰**，"
                        "若对已配准的点再乘一次 T_bl（bug ②）会把整朵 odom 点云刚性旋转 R_bl%s",
                        lidar_frame_.c_str(), roll_bl * k_rad_to_deg, pitch_bl * k_rad_to_deg,
                        yaw_bl * k_rad_to_deg, max_abs_rpy, lever_arm, tail);
        } else {
            RCLCPP_INFO(logger,
                        "外参守卫(§N.5#1)：T_bl=T(%s←base_link) rpy=[%.6f, %.6f, %.6f]° (=%.6f rad)、|t|=%.6f m"
                        " ⇒ 外参**旋转≈单位**（R_bl=I 档）：此时共轭与合成的姿态逐位相同，"
                        "两个 latent bug 在数值上不可见 ⇒ 一旦把倾角记进关节/静态 TF 必须重跑本守卫%s",
                        lidar_frame_.c_str(), roll_bl * k_rad_to_deg, pitch_bl * k_rad_to_deg,
                        yaw_bl * k_rad_to_deg, max_abs_rpy, lever_arm, tail);
        }
    }

    // ===================== 本仓新增：§N.5 守卫 #2 的实现（out-of-line + noinline） =====================
    // 目标：断言 `T_ob ⊗ T_bl⁻¹ ≈ T_ol`（链式法则 p_odom = T_ol·p_livox = T_ol·T_bl·p_base）。
    // 判据（两个分量，为什么不是一个数见 §O.2）：
    //   · 旋转：`R_ob·R_bl⁻¹ == R_ol` **严格**（合成写法下逐位成立，实测 ~1e-16 rad）；
    //     这正是 bug ③（共轭）会破坏的那一项 —— R_bl=Rx(30°) 时残差 4.9°~30°；
    //   · 平移：本仓按 §J.5 **有意**保留旧平移（与合成真值差一个杆臂 |p| = 0.2044 m）⇒
    //     整条等式在平移上永远不会成立 ⇒ 用"双合法值白名单"：发布的 t 必须等于
    //     `t_composed`（物理真值）或 `t_legacy`（有意保留），否则 = 有人写了第三套公式。
    // 输入全部是**已经在手**的量（无新 TF 查询、无新订阅、无新线程）：
    //   · `guard_T_ob_` = 本帧**已发布**的 odom→base_link（= `transform_stamped.transform` 的快照）；
    //   · `guard_T_bl_` = 本帧 lookupTransform(lidar_frame,"base_link") 的原始结果快照；
    //   · `last_odometry` = LIO 自己的 odom←sensor 位姿（odom 回调入参，本回调开头已存）。
    // ★ 为什么这段代码在本文件（独立 TU）（§O.2 A/B 实测）：本包用 `-ffast-math -march=native`；
    //   把它放回 odom 回调所在 TU（odom 回调内或点云回调内都试过）会让 `/odom`+`odom→base_link`
    //   出现 1 ulp 差异（实测 31/31 条差 ≤1.11e-16，`/cloud_registered` 仍逐位相同）；
    //   搬到独立 TU 后，同一输入流（bag 重放）与改动前二进制**逐位 0 差异**。
    // 成本：14 个 double 的拷贝（~5 ns）+ 本函数（≈4 次 4×4 乘法 + 四元数乘/normalize + acos）：
    //   §O.2 的独立微基准（同一套 tf2 运算、2e6 次、非恒定输入）实测 **24.4 ns/帧**；
    //   相对点云回调几十 µs 级开销可忽略，且 noinline + 独立 TU 保证它不改变 hot path 的代码生成。
    // 行为：**只计数 + 限速日志（1 条 / 5 s）**，不抛异常、不中止、不改发布内容。
    __attribute__((noinline)) void SmallPointLioNode::check_pose_consistency() {
        const tf2::Vector3 t_ob(guard_T_ob_.translation.x, guard_T_ob_.translation.y,
                                guard_T_ob_.translation.z);
        const tf2::Quaternion q_ob(guard_T_ob_.rotation.x, guard_T_ob_.rotation.y,
                                   guard_T_ob_.rotation.z, guard_T_ob_.rotation.w);
        const tf2::Vector3 t_bl(guard_T_bl_.translation.x, guard_T_bl_.translation.y,
                                guard_T_bl_.translation.z);
        const tf2::Quaternion q_bl(guard_T_bl_.rotation.x, guard_T_bl_.rotation.y,
                                   guard_T_bl_.rotation.z, guard_T_bl_.rotation.w);
        const tf2::Vector3 t_ol(last_odometry.position.x(), last_odometry.position.y(),
                               last_odometry.position.z());
        const tf2::Quaternion q_ol(last_odometry.orientation.x(), last_odometry.orientation.y(),
                                   last_odometry.orientation.z(), last_odometry.orientation.w());
        const tf2::Transform T_ob(q_ob, t_ob), T_bl(q_bl, t_bl), T_ol(q_ol, t_ol);
        // 一次 4×4 乘法：T_res = T_ob ⊗ T_bl⁻¹
        const tf2::Transform tf_consistency = T_ob * T_bl.inverse();
        tf2::Quaternion q_consistency = tf_consistency.getRotation() * q_ol.inverse();
        q_consistency.normalize();
        // 旋转残差 = 2·acos(|w|)（q 与 −q 同姿态 ⇒ 取绝对值；w→1 时精度 ~1e-8 rad）
        const double rot_res_rad = 2.0 * std::acos(std::min(1.0, std::fabs(q_consistency.w())));
        // 平移残差：对"合成真值"与"有意保留的旧平移"取较小者（双合法值白名单）
        const tf2::Vector3 t_composed = (T_ol * T_bl).getOrigin();
        const tf2::Vector3 t_legacy = (T_bl.inverse() * T_ol * T_bl).getOrigin();
        const double trans_res_vs_truth = (t_composed - t_ob).length();
        const double trans_res_vs_legacy = (t_ob - t_legacy).length();
        const double trans_res_m = std::min(trans_res_vs_truth, trans_res_vs_legacy);
        if (rot_res_rad > k_consistency_rot_tol_rad || trans_res_m > k_consistency_trans_tol_m) {
            ++consistency_violations_;
            const auto steady_now = std::chrono::steady_clock::now();
            if (consistency_violations_ == 1 ||
                steady_now - last_consistency_error_ >= std::chrono::seconds(5)) {
                last_consistency_error_ = steady_now;
                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                             "位姿一致性断言失败（§N.5#2）：T_ob·T_bl⁻¹ ≠ T_ol："
                             "残差 平移 %.9f m / 旋转 %.6f° (%.3e rad)；"
                             "判据 = 旋转≤%.1e rad 且 平移∈{合成真值, 有意保留的旧值(差杆臂 |p|)}±%.1e m；"
                             "累计违例 %llu 帧（限速 1 条/5 s；**只诊断、不停机、不改发布**）"
                             " ⇒ 若刚改过 odom 回调的合成/平移，见 docs/tilted_lidar_fidelity.md §O.2",
                             trans_res_vs_truth, rot_res_rad * k_rad_to_deg, rot_res_rad,
                             k_consistency_rot_tol_rad, k_consistency_trans_tol_m,
                             static_cast<unsigned long long>(consistency_violations_));
            }
        }
    }

    // ===================== 本仓新增（2026-10-20 防御）：TF 回退的判据 + 日志 =====================
    // 为什么放在本 TU：与 §O.2 同一条理由 —— 本包 `-ffast-math -march=native`，把**新的浮点
    // 表达式**放进 `small_point_lio_node.cpp`（odom 回调所在 TU）会改变 GCC 对那段 hot path 的
    // FMA 收缩/内联决策 ⇒ `/odom` 与 `odom→base_link` 出现 ≤1 ulp 差。这里的判据含 std::isfinite
    // 与四元数模长，属于"新的浮点表达式" ⇒ 必须留在独立 TU，由 odom 回调**out-of-line 调用**。
    //
    // 判据（都只读 msg，不改任何东西）：
    //   ① 平移 3 个数、四元数 4 个数**全有限**（!NaN/!Inf）。tf2 允许 NaN 通过（它不做校验），
    //      一旦把 NaN 位姿广播出去，下游 tf2/nav2 会一路 NaN（而日志里只有"等 TF"）。
    //   ② 四元数模长 > 0.5：静态 TF 的 `w=0`（未初始化/写漏）或 URDF 里的退化旋转都在这里被抓到；
    //      正常单位四元数模长 = 1（URDF/robot_state_publisher 给的都是归一化的）。
    // 不满足 ⇒ 调用方把 T_bl 换成**单位阵**（等价于"没有安装外参"），这一帧照发 /Odometry。
    __attribute__((noinline)) bool tf_transform_is_usable(const geometry_msgs::msg::TransformStamped &tf) {
        const auto &t = tf.transform.translation;
        const auto &q = tf.transform.rotation;
        if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
            return false;
        }
        const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        return qn > 0.5;
    }

    __attribute__((noinline)) void log_tf_fallback(std::uint64_t count, const char *why,
                                                   const std::string &detail) {
        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"),
                     "TF 回退（2026-10-20 防御）：%s（%s）⇒ 本帧 T(livox←base_link) 按**单位阵**合成，"
                     "/Odometry 与 odom→base_link **照发**（旧行为是直接 return ⇒ 整段里程计静默消失，"
                     "下游只报\"等 TF\"）。累计 %llu 帧（限速 1 条/5 s）。"
                     "TF 一恢复就自动回到正常路径（无粘滞）；要查："
                     "`ros2 run tf2_ros tf2_echo base_link %s`、`ros2 topic echo /tf_static --once`、"
                     "对照 URDF 里有没有这条 joint（robot11_mount:=sensor/urdf 时 base_link→livox_frame 带 30°）",
                     why, detail.c_str(), static_cast<unsigned long long>(count), detail.c_str());
    }

}// namespace small_point_lio
