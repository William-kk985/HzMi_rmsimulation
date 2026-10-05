#ifndef GICP_REGISTRATION__REGISTRATION_BACKEND_HPP_
#define GICP_REGISTRATION__REGISTRATION_BACKEND_HPP_

// 可切换配准后端（2026-10-05 新增）—— `backend` 参数：pcl（默认，既有已验证路径）| small_gicp。
//
// 为什么要有这一层：
//   · 现有 PCL GeneralizedIterativeClosestPoint 路径是**已验证**的（fitness ≈ 0.00123 m² 合成 /
//     0.0023 m² 实跑，契约与阈值都按它标定过）⇒ 它必须保持**逐字节不变**地可用；
//   · 但两级 leaf（地图 0.10 / 实时点云 0.05）下单帧 align 实测 327~389 ms（单线程），
//     是我们剩下 "Control loop missed its desired rate of 30 Hz" 的主要 CPU 来源之一；
//   · koide3/small_gicp（MIT，header-only）是同一算法的多线程实现（OpenMP 并行 KNN/协方差/法方程归约），
//     是 COD 2025 `small_gicp_relocalization` 用的后端 ⇒ 值得做 A/B。
//
// 设计约束（**契约不变**，见 gicp_registration.cpp 顶部）：
//   · 本类不碰 ROS：只做"目标点云预处理 + 单帧 align + 评分"，回调组/executor/TF 盖戳全在节点里；
//   · ~/fitness_score 的语义（= PCL getFitnessScore() 的"内点平均平方距离 m²"）由本类保证：
//     small_gicp 的 RegistrationResult::error 定义**不同**（Mahalanobis 加权残差和，无量纲），
//     因此 small_gicp 路径**另算**一个与 PCL 逐点等价的指标（见 .cpp 的 pclEquivalentFitnessScore），
//     并把它写进 RegistrationOutcome::fitness_score；small_gicp 的原始 error 只走
//     RegistrationOutcome::backend_error（节点把它发布到**另一个**话题 ~/small_gicp_error）。
//     绝不用 small_gicp 的 error 覆盖 ~/fitness_score。
//
// 点类型仍是 pcl::PointXYZ（clouds 无 intensity/normals，见 gicp_registration.hpp 顶部）。

#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gicp_registration
{

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

/// @brief 配准后端种类。
enum class RegistrationBackendKind {
  PCL,        ///< pcl::GeneralizedIterativeClosestPoint（默认；今天的已验证路径）
  SmallGicp,  ///< koide3/small_gicp 的 Registration<GICPFactor, ParallelReductionOMP>
};

/// @brief 解析 `backend` 参数的字符串值（"pcl" / "small_gicp"，大小写不敏感）。
/// @param value 参数原值
/// @param ok    [out] 是否识别（false ⇒ 调用方应 WARN 并退回默认 PCL）
RegistrationBackendKind parseRegistrationBackendKind(const std::string & value, bool & ok);

/// @brief 后端名（回显用；与 `backend` 参数取值一一对应）。
const char * registrationBackendKindName(RegistrationBackendKind kind);

/// @brief 后端参数（键名与 config/gicp_registration_sim.yaml 一一对应）。
struct RegistrationBackendOptions {
  RegistrationBackendKind kind = RegistrationBackendKind::PCL;
  /// 对应点最大距离（m）。两个后端语义相同：
  ///   PCL  → setMaxCorrespondenceDistance（变体：内层用它的平方当门限）
  ///   small_gicp → DistanceRejector::max_dist_sq = 本值的平方
  double max_correspondence_distance = 1.5;
  /// 外层迭代上限（PCL maximum_iterations / small_gicp LevenbergMarquardtOptimizer::max_iterations）
  int maximum_iterations = 16;
  /// 平移收敛阈值（PCL transformation_epsilon / small_gicp TerminationCriteria::translation_eps）
  double transformation_epsilon = 5.0e-4;
  /// 旋转收敛阈值（PCL rotation_epsilon / small_gicp TerminationCriteria::rotation_eps），**弧度**
  double rotation_epsilon = 2.0e-3;
  /// 估计协方差的 KNN 数（PCL correspondence_randomness / small_gicp estimate_covariances_omp 的 num_neighbors）
  int correspondence_randomness = 20;
  /// PCL 的 BFGS 内层迭代上限 / small_gicp LM 的 lambda 试探内层上限（max_inner_iterations）
  int maximum_optimizer_iterations = 20;
  /// **仅 small_gicp**：OpenMP 线程数（PCL 路径恒单线程，该键对它无影响）
  int small_gicp_num_threads = 4;
};

/// @brief 单帧配准结果。
struct RegistrationOutcome {
  /// 后端自己的收敛标志（PCL hasConverged() / small_gicp RegistrationResult::converged）
  bool converged = false;
  /// **PCL 等价指标**：内点平均平方距离（m²），内点 = final transform 后到 target 最近邻距离 ≤
  /// max_correspondence_distance 的 source 点；无内点 ⇒ quiet_NaN。两个后端同定义 ⇒ 阈值可 A/B。
  double fitness_score = std::numeric_limits<double>::quiet_NaN();
  /// **仅 small_gicp**：RegistrationResult::error 原值（Σ 0.5·rᵀΩr，Mahalanobis 加权，**不是** m²）
  double backend_error = std::numeric_limits<double>::quiet_NaN();
  /// **仅 small_gicp**：RegistrationResult::num_inliers（PCL 不暴露同义量 ⇒ 恒 0）
  size_t num_inliers = 0;
  /// **仅 small_gicp**：RegistrationResult::iterations（PCL 不暴露 ⇒ 恒 0）
  size_t iterations = 0;
  /// T_target←source（与 PCL getFinalTransformation() 同语义）
  Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
};

/// @brief 体素下采样（两个后端**共用**同一实现 ⇒ 源/目标点云逐点一致，A/B 才可比）。
PointCloudT::Ptr voxelDownsample(const PointCloudT::Ptr & in, double leaf_size);

/// @brief 可切换配准后端。
///
/// 生命周期：构造（配置）→ setTarget(先验地图) → 每帧 align(source, guess)。
/// **非线程安全**：与 PCL GICP 一样，绝不允许并发 align（节点靠 align_cb_group_ 的组内互斥保证）。
class RegistrationBackend
{
public:
  explicit RegistrationBackend(const RegistrationBackendOptions & options);
  ~RegistrationBackend();

  RegistrationBackend(const RegistrationBackend &) = delete;
  RegistrationBackend & operator=(const RegistrationBackend &) = delete;

  /// @brief 后端名（"pcl" / "small_gicp"）。
  const char * name() const;

  /// @brief 设置先验地图（已经体素下采样、已去 NaN 的 target）。
  ///   PCL 路径：只 setInputTarget（目标协方差按 PCL 原行为**首次 align 时**才预计算）；
  ///   small_gicp 路径：转 small_gicp::PointCloud + 建 KdTree + 估计协方差（**构造期一次**）。
  /// @return 目标点云的一次性预处理耗时（ms；PCL 路径为 0，因为它是惰性的）
  double setTarget(const PointCloudT::Ptr & map_cloud);

  /// @brief 目标点数（banner / [status] 用）。
  size_t targetSize() const { return target_size_; }

  /// @brief 单帧配准：返回 T_target←source、收敛标志与**PCL 等价** fitness score。
  ///        耗时统计由调用方（节点 / bench）负责 ⇒ 本函数里不含日志。
  RegistrationOutcome align(const PointCloudT::Ptr & source, const Eigen::Matrix4d & guess);

private:
  struct Impl;  // small_gicp / PCL 的具体类型藏在 .cpp（头文件不引入 small_gicp，编译面最小）
  std::unique_ptr<Impl> impl_;
  size_t target_size_ = 0;
};

}  // namespace gicp_registration

#endif  // GICP_REGISTRATION__REGISTRATION_BACKEND_HPP_
