// 可切换配准后端（backend 参数）的实现：pcl（默认）/ small_gicp。
//
// 依赖：third_party/small_gicp（koide3/small_gicp，MIT，pinned commit 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
// = v1.0.1）。CMake 侧见 CMakeLists.txt 的 add_subdirectory 段；本文件用的都是小头文件 API：
//   small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>  —— 与 PCL GICP 同算法
//   small_gicp::KdTree<small_gicp::PointCloud> / KdTreeBuilderOMP                        —— 静态先验地图的近邻结构
//   small_gicp::voxelgrid_sampling_omp / estimate_covariances_omp                        —— 预处理
//   small_gicp::RegistrationResult::{converged, error, num_inliers, iterations, T_target_source}
//   （small_gicp::IncrementalVoxelMap 也在这个 pinned commit 里，但本节点先验地图是**静态**的、
//     不需要增量插入/LRU 淘汰 ⇒ 用静态 KdTree 更省事也更快；留作以后 scan-to-map 增量更新用。）
//
// ⚠ 指标语义（本次改动最容易踩的坑）：small_gicp 的 RegistrationResult::error 是
//   Σ 0.5·rᵀ·(C_target + T·C_source·Tᵀ)⁻¹·r（Mahalanobis 加权、无量纲），
//   **不是** PCL getFitnessScore() 的"内点平均平方距离（m²）"。两者数值差好几个量级，
//   直接拿 error 当 ~/fitness_score 会让 max_fitness_score/fitness_score_warn 两个阈值静默失效
//   ⇒ 这里对 small_gicp 路径**另算** PCL 等价指标（pclEquivalentFitnessScore），
//   error 只经 RegistrationOutcome::backend_error 走 ~/small_gicp_error 这条独立话题。

#include "gicp_registration/registration_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

// vendored small_gicp 头文件按 `-I` 引入（add_subdirectory 的 INTERFACE include；见 CMakeLists.txt 里
// 为什么不用 -isystem）。上游代码在 -Wextra 下有 2 处自身告警，会把本包"0 warning"的构建契约打破：
//   small_gicp/registration/rejector.hpp:23   unused parameter ×5  [-Wunused-parameter]
//   small_gicp/ann/kdtree_omp.hpp:62          signed/unsigned 比较 [-Wsign-compare]
// 这里只**在 include 这几行周围**关掉这两类告警（GCC/Clang 的模板告警取"定义点"的 pragma 状态
// ⇒ 抑制范围仅限上游头文件），pragma pop 之后本包自己的代码照旧受 -Wall -Wextra -Wpedantic 管。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#pragma GCC diagnostic pop

namespace gicp_registration
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string toLower(const std::string & in)
{
  std::string out = in;
  std::transform(
    out.begin(), out.end(), out.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

// pcl::PointCloud<pcl::PointXYZ> → small_gicp::PointCloud（x/y/z + 齐次 1.0；不搬 intensity/normal：
// 我们的点云本来就没有，且 GICP 的两个实现都只用 x/y/z）。
small_gicp::PointCloud::Ptr toSmallGicpCloud(const PointCloudT & in)
{
  auto out = std::make_shared<small_gicp::PointCloud>();
  out->resize(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    const PointT & p = in.points[i];
    out->point(i) << static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z), 1.0;
  }
  return out;
}

// **PCL getFitnessScore() 的等价实现**（PCL 1.12.1 implementation 见 pcl/registration/impl/registration.hpp:140-165）：
//   把 source 用 final transform 变换到 target 系 → 每个点查 target 最近邻 →
//   若**平方**距离 ≤ max_range（注意 PCL 的入参就是平方距离，本节点传 max_corr_dist²）则计入内点 →
//   返回 内点平方距离之和 / 内点数；无内点返回 max()（由调用方统一成 nan）。
// 与 PCL 的差别（如实记录，不粉饰）：
//   · PCL 的 KdTreeFLANN 用 float 存平方距离，这里用 double ⇒ 门限边界上的点可能归属不同（影响 <1e-6 m²）；
//   · PCL 的 final_transformation_ 是 Matrix4f（float），这里 small_gicp 全程 double ⇒ 数值路径不同。
double pclEquivalentFitnessScore(
  const small_gicp::KdTree<small_gicp::PointCloud> & target_tree,
  const small_gicp::PointCloud & source, const Eigen::Isometry3d & T_target_source,
  double max_range_sq)
{
  double sum_sq = 0.0;
  size_t n_inliers = 0;
  for (size_t i = 0; i < source.size(); ++i) {
    const Eigen::Vector4d query = T_target_source * source.point(i);
    size_t k_index = 0;
    double k_sq_dist = std::numeric_limits<double>::max();
    if (target_tree.nearest_neighbor_search(query, &k_index, &k_sq_dist) == 0) {
      continue;
    }
    if (k_sq_dist <= max_range_sq) {
      sum_sq += k_sq_dist;
      ++n_inliers;
    }
  }
  if (n_inliers == 0) {
    return std::numeric_limits<double>::max();
  }
  return sum_sq / static_cast<double>(n_inliers);
}
}  // namespace

RegistrationBackendKind parseRegistrationBackendKind(const std::string & value, bool & ok)
{
  const std::string v = toLower(value);
  if (v == "pcl") {
    ok = true;
    return RegistrationBackendKind::PCL;
  }
  if (v == "small_gicp") {
    ok = true;
    return RegistrationBackendKind::SmallGicp;
  }
  ok = false;
  return RegistrationBackendKind::PCL;  // 未识别 ⇒ 退回默认（今天的已验证路径）
}

const char * registrationBackendKindName(RegistrationBackendKind kind)
{
  return kind == RegistrationBackendKind::SmallGicp ? "small_gicp" : "pcl";
}

PointCloudT::Ptr voxelDownsample(const PointCloudT::Ptr & in, double leaf_size)
{
  // 与本次改动之前 gicp_registration.cpp 里的 GicpNode::downsample() **逐行等价**（只是搬了个位置）：
  // 两个后端共用同一个下采样 ⇒ 源/目标点云一致，align ms 与 fitness 才可比。
  PointCloudT::Ptr out(new PointCloudT);
  pcl::VoxelGrid<PointT> voxel;
  const float leaf = static_cast<float>(leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(in);
  voxel.filter(*out);
  return out;
}

struct RegistrationBackend::Impl {
  explicit Impl(const RegistrationBackendOptions & o) : o(o) {}

  RegistrationBackendOptions o;

  // ---- PCL 后端（默认；行为与本次改动之前完全一致） ----
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

  // ---- small_gicp 后端 ----
  small_gicp::PointCloud::Ptr sg_target;                                     // 先验地图（含协方差）
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> sg_target_tree;  // 目标侧近邻结构
  // 每帧复用的源点云缓冲（避免每帧 malloc；resize 到同尺寸不会重分配）
  small_gicp::PointCloud::Ptr sg_source;
  // Registration 可复用（内部按 source 尺寸临时建 factors 数组；见 small_gicp registration.hpp:33）
  // 优化器 = 上游默认的 **LevenbergMarquardtOptimizer**（为什么不用 GaussNewton 见配置段末尾的实测说明）
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> sg_reg;
};

RegistrationBackend::RegistrationBackend(const RegistrationBackendOptions & options)
: impl_(new Impl(options))
{
  if (impl_->o.kind == RegistrationBackendKind::PCL) {
    // ============================ PCL GICP 配置（**原样保留**） ============================
    impl_->gicp.setMaxCorrespondenceDistance(impl_->o.max_correspondence_distance);
    impl_->gicp.setMaximumIterations(impl_->o.maximum_iterations);
    impl_->gicp.setTransformationEpsilon(impl_->o.transformation_epsilon);
    impl_->gicp.setRotationEpsilon(impl_->o.rotation_epsilon);
    impl_->gicp.setCorrespondenceRandomness(impl_->o.correspondence_randomness);
    impl_->gicp.setMaximumOptimizerIterations(impl_->o.maximum_optimizer_iterations);
  } else {
    // ============================ small_gicp 配置 ============================
    // 下面每一项都刻意与上面 PCL 的 setXxx **一一对应**（同一份 config 驱动两个后端）：
    const int threads = std::max(1, impl_->o.small_gicp_num_threads);
    impl_->sg_reg.reduction.num_threads = threads;
    // PCL: corr_dist_threshold_² 作对应点门限（impl/gicp.hpp:414）
    impl_->sg_reg.rejector.max_dist_sq =
      impl_->o.max_correspondence_distance * impl_->o.max_correspondence_distance;
    // PCL: transformation_epsilon / rotation_epsilon 两段式判据（impl/gicp.hpp:474-483）
    // small_gicp: TerminationCriteria::converged(delta) 同时看平移/旋转增量（termination_criteria.hpp:14）
    impl_->sg_reg.criteria.translation_eps = impl_->o.transformation_epsilon;
    impl_->sg_reg.criteria.rotation_eps = impl_->o.rotation_epsilon;
    // PCL: max_iterations_（外层）；small_gicp: LevenbergMarquardtOptimizer::max_iterations（外层）
    impl_->sg_reg.optimizer.max_iterations = impl_->o.maximum_iterations;
    // PCL: max_inner_iterations_（BFGS 内层）；small_gicp: LM 的 lambda 试探内层
    impl_->sg_reg.optimizer.max_inner_iterations = impl_->o.maximum_optimizer_iterations;
    // 注意：small_gicp 默认用 **LM**（LevenbergMarquardtOptimizer），PCL GICP 用 BFGS（Newton 型）
    // ⇒ 收敛轨迹/迭代次数不会逐帧一致，这是两个实现之间**无法消除**的行为差异（见 docs）。
    // ★ 实测行为差异（docs/gicp_backend_small_gicp.md「~/converged 抖动」一节有数据）：
    //   LM 在"已经处在极小点"时会因为 lambda 试探无法再降低误差而**提前 break 且 converged=false**
    //   （optimizer.hpp:170 的 `if (!success) break;`）⇒ 本节点实测有 4~8% 的帧在 score 已经是最优
    //   （0.00123 m²）的情况下 converged=false ⇒ 被"converged && score<=max"这条既有接受判据丢掉
    //   （**只影响更新频率，不影响位姿**：map→odom 沿用上一次估计）。
    //   对照实测（同机同数据）：换成 small_gicp::GaussNewtonOptimizer 后 ~/converged 100% true、
    //   耗时/fitness/坏初值恢复都无差别；但 GN 没有 LM 的阻尼 ⇒ 离解很远时更激进。
    //   这里**故意保留上游默认 LM**（重定位场景下稳健优先），把差异如实记录在 docs；若团队更在意
    //   ~/converged 不抖，把本行上面的 Registration 第三~五个模板参数写成
    //   `small_gicp::NullFactor, small_gicp::DistanceRejector, small_gicp::GaussNewtonOptimizer`
    //   并把上面的 max_inner_iterations 一行删掉即可（一行改动，已在 docs 里给出）。
  }
}

RegistrationBackend::~RegistrationBackend() = default;

const char * RegistrationBackend::name() const
{
  return registrationBackendKindName(impl_->o.kind);
}

double RegistrationBackend::setTarget(const PointCloudT::Ptr & map_cloud)
{
  target_size_ = map_cloud ? map_cloud->size() : 0;
  if (impl_->o.kind == RegistrationBackendKind::PCL) {
    // 目标只设一次；目标协方差按 PCL 原行为在**首次 align** 时预计算并复用
    // （这就是启动后第一帧 align 特别慢 ~2.3 s 的来源）。
    impl_->gicp.setInputTarget(map_cloud);
    return 0.0;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const int threads = std::max(1, impl_->o.small_gicp_num_threads);
  impl_->sg_target = toSmallGicpCloud(*map_cloud);
  impl_->sg_target_tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
    impl_->sg_target, small_gicp::KdTreeBuilderOMP(threads));
  // 协方差 KNN 数 = correspondence_randomness（PCL 的 k_correspondences_，两者默认都是 20）
  small_gicp::estimate_covariances_omp(
    *impl_->sg_target, *impl_->sg_target_tree, impl_->o.correspondence_randomness, threads);
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

RegistrationOutcome RegistrationBackend::align(
  const PointCloudT::Ptr & source, const Eigen::Matrix4d & guess)
{
  RegistrationOutcome out;
  const double max_range_sq =
    impl_->o.max_correspondence_distance * impl_->o.max_correspondence_distance;

  if (impl_->o.kind == RegistrationBackendKind::PCL) {
    impl_->gicp.setInputSource(source);  // 每帧设源：GICP 会重算源协方差（地图侧协方差被复用）
    PointCloudT aligned;
    impl_->gicp.align(aligned, guess.cast<float>());
    out.converged = impl_->gicp.hasConverged();
    // getFitnessScore 的入参在 PCL 里是**平方距离**阈值（impl/registration.hpp:152），所以要传 r²；
    // 返回**内点的平均平方距离（m²）**，没有任何内点时返回 numeric_limits<double>::max()。
    out.fitness_score = impl_->gicp.getFitnessScore(max_range_sq);
    out.T_target_source = impl_->gicp.getFinalTransformation().cast<double>();
    if (!std::isfinite(out.fitness_score) ||
      out.fitness_score >= std::numeric_limits<double>::max())
    {
      out.fitness_score = kNaN;  // "无有效内点" → nan（~/fitness_score 可见）
    }
    return out;
  }

  // ============================ small_gicp 路径 ============================
  const int threads = std::max(1, impl_->o.small_gicp_num_threads);
  // 源点云：复用缓冲（同尺寸 resize 不重分配）；协方差每帧重算（源帧帧变，与 PCL 一样是固定开销）
  if (!impl_->sg_source || impl_->sg_source->size() != source->size()) {
    impl_->sg_source = std::make_shared<small_gicp::PointCloud>();
  }
  impl_->sg_source->resize(source->size());
  for (size_t i = 0; i < source->size(); ++i) {
    const PointT & p = source->points[i];
    impl_->sg_source->point(i) << static_cast<double>(p.x), static_cast<double>(p.y),
      static_cast<double>(p.z), 1.0;
  }
  // 源侧近邻结构只用于**协方差估计**（配准只用 target 树）：KdTree 不持有 points 所有权，
  // 所以这个临时树必须在 sg_source 存活期内使用（同一作用域，安全）。
  small_gicp::UnsafeKdTree<small_gicp::PointCloud> source_tree(*impl_->sg_source);
  small_gicp::estimate_covariances_omp(
    *impl_->sg_source, source_tree, impl_->o.correspondence_randomness, threads);

  Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
  init_T.matrix() = guess;
  const small_gicp::RegistrationResult res = impl_->sg_reg.align(
    *impl_->sg_target, *impl_->sg_source, *impl_->sg_target_tree, init_T);

  out.converged = res.converged;
  // RegistrationResult::iterations 是"最后一次外层迭代的下标"（optimizer.hpp:100/170）
  out.iterations = res.iterations;
  out.num_inliers = res.num_inliers;
  out.backend_error = res.error;  // 原始 Mahalanobis 加权误差：只走 ~/small_gicp_error
  out.T_target_source = res.T_target_source.matrix();
  // ★ 契约：~/fitness_score 仍是 PCL 等价指标（内点平均平方距离 m²），**不是** res.error
  out.fitness_score = pclEquivalentFitnessScore(
    *impl_->sg_target_tree, *impl_->sg_source, res.T_target_source, max_range_sq);
  if (!std::isfinite(out.fitness_score) ||
    out.fitness_score >= std::numeric_limits<double>::max())
  {
    out.fitness_score = kNaN;
  }
  return out;
}

}  // namespace gicp_registration
