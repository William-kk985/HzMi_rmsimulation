// gicp_backend_bench —— 两个配准后端（pcl / small_gicp）的**离线 A/B 台架**（不启 ROS 图、不启 Gazebo/nav2）。
//
// 为什么要有它：节点里的 align ms 只能靠 [status] 行每秒看一个采样；A/B 需要**同一批帧**上的
// 逐帧耗时（median / p95 / max）、fitness 量级，以及"故意给坏初值"时的收敛行为。
// 本台架走的是与节点**完全相同的代码路径**（gicp_registration::RegistrationBackend + voxelDownsample），
// 合成点云也按 gicp_alone_probe.py 的口径生成（从 PCD/RMUL2026.pcd 抽 4000 点、只留 x/y/z、加注入偏移）
// ⇒ 台架数值与节点 [status] 行应当一致（docs/gicp_backend_small_gicp.md 记录了两者对照）。
//
// 用法（示例，见 docs 的"复现命令"）：
//   install/gicp_registration/lib/gicp_registration/gicp_backend_bench --pcd <PCD> --backend both
//   （可选参数：--frames 30 --threads 4 --map-leaf 0.10 --scan-leaf 0.05 --max-corr 1.5 …）
//   （`.tmp_cache/gicp_ab/` 下的驱动脚本把两个后端 × 线程数扫一遍并汇总成表）
//
// 注意：本文件里**没有** ROS 调用（只链接了本包的库以复用 RegistrationBackend）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>

#include "gicp_registration/registration_backend.hpp"

namespace
{
using gicp_registration::PointCloudT;
using gicp_registration::PointT;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Args {
  std::string pcd;
  std::string backend = "both";  // both | pcl | small_gicp
  double map_leaf = 0.10;
  double scan_leaf = 0.05;
  double max_corr = 1.5;
  int iters = 16;
  double trans_eps = 5.0e-4;
  double rot_eps = 2.0e-3;
  int corr_rand = 20;
  int opt_iters = 20;
  int threads = 4;
  int scan_points = 4000;
  double offset = 0.15;  // 合成扫描 = 地图点 + (+offset, -offset, 0)（与 probe 一致）
  int frames = 30;
  double max_fitness = 0.3;  // 与 config 的 max_fitness_score 一致（"采纳"判据）
};

void usage()
{
  std::printf(
    "用法: gicp_backend_bench --pcd <path.pcd> [--backend both|pcl|small_gicp] [--frames N]\n"
    "        [--map-leaf 0.10] [--scan-leaf 0.05] [--max-corr 1.5] [--iters 16]\n"
    "        [--threads 4] [--scan-points 4000] [--offset 0.15] [--max-fitness 0.3]\n");
}

bool parseArgs(int argc, char ** argv, Args & a)
{
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](const char * name) -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "[bench] 参数 %s 缺少取值\n", name);
          std::exit(2);
        }
        return argv[++i];
      };
    if (k == "--pcd") {a.pcd = next("--pcd");} else if (k == "--backend") {
      a.backend = next("--backend");
    } else if (k == "--map-leaf") {a.map_leaf = std::atof(next("--map-leaf"));} else if (k ==
      "--scan-leaf") {a.scan_leaf = std::atof(next("--scan-leaf"));} else if (k == "--max-corr") {
      a.max_corr = std::atof(next("--max-corr"));
    } else if (k == "--iters") {a.iters = std::atoi(next("--iters"));} else if (k == "--frames") {
      a.frames = std::atoi(next("--frames"));
    } else if (k == "--threads") {a.threads = std::atoi(next("--threads"));} else if (k ==
      "--scan-points") {a.scan_points = std::atoi(next("--scan-points"));} else if (k == "--offset") {
      a.offset = std::atof(next("--offset"));
    } else if (k == "--max-fitness") {a.max_fitness = std::atof(next("--max-fitness"));} else if (
      k == "--corr-rand") {a.corr_rand = std::atoi(next("--corr-rand"));} else if (k == "--opt-iters") {
      a.opt_iters = std::atoi(next("--opt-iters"));
    } else if (k == "-h" || k == "--help") {usage(); std::exit(0);} else {
      std::fprintf(stderr, "[bench] 未知参数: %s\n", k.c_str());
      usage();
      return false;
    }
  }
  if (a.pcd.empty()) {
    std::fprintf(stderr, "[bench] 必须给 --pcd\n");
    usage();
    return false;
  }
  return true;
}

struct Stats {
  double min = kNaN, median = kNaN, p95 = kNaN, max = kNaN, mean = kNaN;
};

Stats computeStats(std::vector<double> v)
{
  Stats s;
  if (v.empty()) {
    return s;
  }
  std::sort(v.begin(), v.end());
  s.min = v.front();
  s.max = v.back();
  s.median = v[v.size() / 2];
  s.p95 = v[std::min(v.size() - 1, static_cast<size_t>(0.95 * static_cast<double>(v.size())))];
  double sum = 0.0;
  for (double x : v) {sum += x;}
  s.mean = sum / static_cast<double>(v.size());
  return s;
}

double rotationErrorDeg(const Eigen::Matrix4d & a, const Eigen::Matrix4d & b)
{
  const Eigen::Matrix3d R = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const double c = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / M_PI;
}

double translationError(const Eigen::Matrix4d & a, const Eigen::Matrix4d & b)
{
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

struct Trial {
  std::string label;
  double dx = 0.0, dyaw_deg = 0.0;
  bool converged = false;
  bool accepted = false;
  double fitness = kNaN;
  double raw_error = kNaN;
  double trans_err = kNaN, yaw_err = kNaN;
  double ms = kNaN;
};

void printTrial(const char * backend, const Trial & t)
{
  std::printf(
    "[bench]   %-22s %-11s converged=%-5s accepted=%-5s fitness=%s m² | 最终误差 平移=%.3f m yaw=%.2f°"
    " | align=%.1f ms\n",
    t.label.c_str(), backend, t.converged ? "true" : "false", t.accepted ? "true" : "false",
    std::isfinite(t.fitness) ? std::to_string(t.fitness).c_str() : "nan",
    t.trans_err, t.yaw_err, t.ms);
}

void runOneBackend(
  const Args & a, gicp_registration::RegistrationBackendKind kind, const PointCloudT::Ptr & map_ds,
  const PointCloudT::Ptr & scan_raw, const Eigen::Matrix4d & T_true)
{
  gicp_registration::RegistrationBackendOptions o;
  o.kind = kind;
  o.max_correspondence_distance = a.max_corr;
  o.maximum_iterations = a.iters;
  o.transformation_epsilon = a.trans_eps;
  o.rotation_epsilon = a.rot_eps;
  o.correspondence_randomness = a.corr_rand;
  o.maximum_optimizer_iterations = a.opt_iters;
  o.small_gicp_num_threads = a.threads;

  gicp_registration::RegistrationBackend backend(o);
  const double prep_ms = backend.setTarget(map_ds);
  const char * name = backend.name();
  const PointCloudT::Ptr source = gicp_registration::voxelDownsample(scan_raw, a.scan_leaf);

  std::printf(
    "\n[bench] ===== backend=%s =====\n"
    "[bench]   target=%zu 点（leaf %.3f m）  source=%zu 点（leaf %.3f m）  max_corr=%.2f m"
    "  iters=%d  corr_rand=%d  threads=%d\n",
    name, backend.targetSize(), a.map_leaf, source->size(), a.scan_leaf, a.max_corr,
    a.iters, a.corr_rand, a.threads);
  std::printf(
    "[bench]   setTarget 一次性预处理=%.1f ms%s\n", prep_ms,
    kind == gicp_registration::RegistrationBackendKind::PCL ?
    "（PCL 惰性：目标协方差在**首帧 align** 里算，故首帧会特别慢）" :
    "（small_gicp：转点云 + 建 KdTree + 估协方差，都在构造期一次做完）");

  // ---- ① 稳态跟踪：初值 = 真值（第 0 帧），之后 = 上一次**被采纳**的估计（与节点一致） ----
  Eigen::Matrix4d T_odom = T_true;
  std::vector<double> ms_steady, fit_steady;
  double first_align_ms = kNaN;
  int converged_n = 0, accepted_n = 0;
  for (int f = 0; f < a.frames; ++f) {
    const auto t0 = std::chrono::steady_clock::now();
    const gicp_registration::RegistrationOutcome out = backend.align(source, T_odom);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (f == 0) {
      first_align_ms = ms;
    } else {
      ms_steady.push_back(ms);
    }
    const bool accepted = out.converged && std::isfinite(out.fitness_score) &&
      out.fitness_score <= a.max_fitness;
    if (accepted) {
      T_odom = out.T_target_source;
      accepted_n++;
    }
    if (out.converged) {converged_n++;}
    if (f > 0) {fit_steady.push_back(out.fitness_score);}
    std::printf(
      "[bench-frame] backend=%s frame=%d align_ms=%.2f fitness=%.6f converged=%d accepted=%d"
      " trans_err=%.4f yaw_err=%.3f\n",
      name, f, ms, out.fitness_score, out.converged ? 1 : 0, accepted ? 1 : 0,
      translationError(out.T_target_source, T_true), rotationErrorDeg(out.T_target_source, T_true));
  }
  const Stats s = computeStats(ms_steady);
  const Stats fs = computeStats(fit_steady);
  std::printf(
    "[bench]   首帧 align=%.1f ms（**不计入稳态统计**；PCL 含目标协方差预计算）\n", first_align_ms);
  std::printf(
    "[bench]   稳态 align（%zu 帧）：median=%.1f ms  mean=%.1f  p95=%.1f  max=%.1f  min=%.1f\n",
    ms_steady.size(), s.median, s.mean, s.p95, s.max, s.min);
  std::printf(
    "[bench]   ~/converged 等价：true %d/%d 帧 | 采纳（converged && fitness<=%.2f）：%d/%d 帧\n",
    converged_n, a.frames, a.max_fitness, accepted_n, a.frames);
  std::printf(
    "[bench]   fitness（PCL 等价，m²）：median=%.5f  min=%.5f  max=%.5f\n",
    fs.median, fs.min, fs.max);
  std::printf(
    "[bench]   最终位姿误差：平移=%.4f m  yaw=%.3f°（相对真值 T_target_source）\n",
    translationError(T_odom, T_true), rotationErrorDeg(T_odom, T_true));

  // ---- ② 坏初值：真值 ∘ (平移 dx, yaw dyaw) —— 等价于 RViz 给错的 2D Pose Estimate ----
  std::printf("[bench]   ---- 坏初值容忍度（同一帧、同一 source；初值 = 真值 ∘ 偏差）----\n");
  const struct { const char * label; double dx; double dyaw; } bad[] = {
    {"+0.5 m / +10°", 0.5, 10.0},
    {"+0.3 m / +5°", 0.3, 5.0},
    {"+1.0 m / +20°", 1.0, 20.0},
  };
  for (const auto & b : bad) {
    Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
    delta.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(b.dyaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    delta(0, 3) = b.dx;
    const Eigen::Matrix4d init = T_true * delta;
    const auto t0 = std::chrono::steady_clock::now();
    const gicp_registration::RegistrationOutcome out = backend.align(source, init);
    Trial t;
    t.label = b.label;
    t.dx = b.dx;
    t.dyaw_deg = b.dyaw;
    t.converged = out.converged;
    t.accepted = out.converged && std::isfinite(out.fitness_score) &&
      out.fitness_score <= a.max_fitness;
    t.fitness = out.fitness_score;
    t.raw_error = out.backend_error;
    t.trans_err = translationError(out.T_target_source, T_true);
    t.yaw_err = rotationErrorDeg(out.T_target_source, T_true);
    t.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    printTrial(name, t);
    if (std::isfinite(out.backend_error) || out.num_inliers > 0 || out.iterations > 0) {
      std::printf(
        "[bench]        （small_gicp 原生量：error=%.6f num_inliers=%zu iterations=%zu）\n",
        out.backend_error, out.num_inliers, out.iterations);
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  if (!parseArgs(argc, argv, a)) {
    return 2;
  }

  // ---- 与节点一致地读地图：loadPCDFile<PointT> → removeNaN → 两级 leaf 的粗档 ----
  PointCloudT::Ptr raw(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(a.pcd, *raw) < 0) {
    std::fprintf(stderr, "[bench] PCD 读取失败：%s\n", a.pcd.c_str());
    return 1;
  }
  pcl::Indices finite_indices;
  PointCloudT::Ptr finite(new PointCloudT);
  pcl::removeNaNFromPointCloud(*raw, *finite, finite_indices);
  const PointCloudT::Ptr map_ds = gicp_registration::voxelDownsample(finite, a.map_leaf);

  // ---- 合成实时点云：从**去 NaN 的原始地图**等间隔抽 scan_points 点 + 注入偏移（(+d, -d, 0)）----
  PointCloudT::Ptr scan(new PointCloudT);
  {
    const size_t n = finite->size();
    const size_t want = std::min<size_t>(static_cast<size_t>(std::max(1, a.scan_points)), n);
    const size_t stride = std::max<size_t>(1, n / want);
    for (size_t i = 0; i < n && scan->size() < want; i += stride) {
      PointT p = finite->points[i];
      p.x = static_cast<float>(p.x + a.offset);
      p.y = static_cast<float>(p.y - a.offset);
      scan->push_back(p);
    }
  }
  Eigen::Matrix4d T_true = Eigen::Matrix4d::Identity();
  T_true(0, 3) = -a.offset;  // source = map + (+d, -d) ⇒ T_map←sensor = Trans(-d, +d, 0)
  T_true(1, 3) = a.offset;

  std::printf(
    "[bench] pcd=%s  原始 %zu 点 → 去 NaN %zu → 地图 leaf %.3f m 后 %zu 点\n"
    "[bench] 合成扫描 %zu 点（等间隔抽样；只有 x/y/z；相对地图偏移 (+%.3f, -%.3f, 0) m）\n"
    "[bench] 真值 T_target_source: 平移 (%.3f, %.3f, 0) m；评测阈值 max_fitness=%.3f m²\n",
    a.pcd.c_str(), raw->size(), finite->size(), a.map_leaf, map_ds->size(), scan->size(), a.offset,
    a.offset, T_true(0, 3), T_true(1, 3), a.max_fitness);

  const std::vector<gicp_registration::RegistrationBackendKind> kinds =
    (a.backend == "pcl") ? std::vector<gicp_registration::RegistrationBackendKind>{
    gicp_registration::RegistrationBackendKind::PCL} :
    (a.backend == "small_gicp") ? std::vector<gicp_registration::RegistrationBackendKind>{
    gicp_registration::RegistrationBackendKind::SmallGicp} :
    std::vector<gicp_registration::RegistrationBackendKind>{
    gicp_registration::RegistrationBackendKind::PCL,
    gicp_registration::RegistrationBackendKind::SmallGicp};

  for (const auto kind : kinds) {
    runOneBackend(a, kind, map_ds, scan, T_true);
  }
  std::printf("\n[bench] 完成。\n");
  return 0;
}
