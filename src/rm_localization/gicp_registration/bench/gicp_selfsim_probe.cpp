// gicp_selfsim_probe —— **场地自相似性**离线探针（不启 ROS 图、不启 Gazebo/nav2）。
//
// 它回答的问题（docs/gicp_divergence_and_jitter.md §2 的机制证据就是它给的）：
//   「如果初值偏了 δ，**真的** GICP 会把估计拉回真值，还是把 δ 原样留下？」
//     · 拉回 ⇒ 这个方向上有唯一极小 ⇒ 逐帧跟踪不会自传播；
//     · 留下（甚至放大）⇒ 该方向上存在**连续的一片极小**（自相似）⇒
//       节点里"初值 = 上一次结果 + 里程计增量"的递推就是一个正反馈回路：
//       e_{n+1} ≈ e_n + f(e_n)，f 是 GICP 相对初值的**残留偏差**。
//   这正是用户那次跑里 map→odom 单调跑掉、而 fitness 仍然很好的机制。
//
// 与节点的**代码路径完全一致**：gicp_registration::voxelDownsample + RegistrationBackend（同参数），
// 所以这里的数字可以直接和节点 [status] 行的 fitness 量级对照。
//
// 扫描（source）怎么来 —— 两种，都能用：
//   ① `--scan <real_cloud.pcd>`：**真实点云**（从 /livox/lidar/pointcloud 落盘，见 --dump-clouds）；
//   ② 不给 --scan：从先验地图**合成**一帧"可见点"——按机器人在 map 系的位姿，只保留
//      距离 ≤ range 且**俯仰角在雷达 FOV 内**（Mid360: -7.22°~+55.22°，见 urdf/mid360.xacro）的点，
//      再变换到雷达系。它没有遮挡（Gazebo 的射线会停在第一个命中），所以是"自相似性上界"，
//      文档里必须如实标注这一点。
//
// 用法（示例）：
//   install/gicp_registration/lib/gicp_registration/gicp_selfsim_probe
//       --pcd src/rm_nav_bringup/PCD/RMUC2026.pcd --pose 0 0 0 --range 20
//       --backend pcl --max-corr 1.5 --axis x --sweep -2:2:0.25
//
// 输出（每行一台账，便于 grep/awk 成表）：
//   [selfsim] axis=x seed_dx=+0.75 converged=1 fitness=0.00213 residual=+0.71 (从初值又挪了多少)
//             final_err=+0.71 (结果离真值多远) seed_err=0.75 ms=380.1

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
  std::string pcd;          // 先验地图
  std::string scan;         // 可选：真实点云（不填 = 合成）
  double px = 0.0, py = 0.0, pz = 0.0, pyaw = 0.0;  // 机器人在 map 系的位姿（合成扫描 + 真值）
  double range = 20.0;      // 合成扫描用的最大距离（m）
  double elev_min_deg = -7.22, elev_max_deg = 55.22;  // Mid360 垂直 FOV（urdf/mid360.xacro）
  double sensor_z = 0.235;  // 雷达在 map 系的安装高度（base_link z≈0.06 + 0.175，见 urdf）
  double noise = 0.0;       // 合成扫描加的高斯噪声（m，0 = 不加）
  int scan_points = 4000;   // 合成扫描等间隔抽到多少点（**必须**：真实单帧 ~3~4 千点；
                            //   不抽的话合成切片有几万点，-O0 下源协方差 KNN 会跑几十分钟）
  std::string backend = "pcl";
  double map_leaf = 0.10, scan_leaf = 0.05, max_corr = 1.5;
  int iters = 16, corr_rand = 20, opt_iters = 20, threads = 4;
  std::string axis = "x";       // x | y | yaw | xy
  double sweep_lo = -2.0, sweep_hi = 2.0, sweep_step = 0.25;
};

void usage()
{
  std::printf(
    "用法: gicp_selfsim_probe --pcd <map.pcd> [--scan <real.pcd>] [--pose x y yaw]\n"
    "        [--axis x|y|yaw|xy] [--sweep lo:hi:step] [--range 20] [--noise 0]\n"
    "        [--backend pcl|small_gicp] [--map-leaf .10] [--scan-leaf .05] [--max-corr 1.5]\n"
    "        [--scan-points 4000]（合成扫描抽稀到多少点；真实单帧 ~3~4 千点）\n");
}

bool parseArgs(int argc, char ** argv, Args & a)
{
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](const char * n) -> const char * {
        if (i + 1 >= argc) {std::fprintf(stderr, "[selfsim] %s 缺取值\n", n); std::exit(2);}
        return argv[++i];
      };
    if (k == "--pcd") {a.pcd = next("--pcd");} else if (k == "--scan") {a.scan = next("--scan");} else if (
      k == "--pose") {
      a.px = std::atof(next("--pose")); a.py = std::atof(next("--pose")); a.pyaw = std::atof(next("--pose"));
    } else if (k == "--range") {a.range = std::atof(next("--range"));} else if (k == "--noise") {
      a.noise = std::atof(next("--noise"));
    } else if (k == "--sensor-z") {a.sensor_z = std::atof(next("--sensor-z"));} else if (k ==
      "--scan-points") {a.scan_points = std::atoi(next("--scan-points"));} else if (k == "--backend") {
      a.backend = next("--backend");
    } else if (k == "--map-leaf") {a.map_leaf = std::atof(next("--map-leaf"));} else if (k ==
      "--scan-leaf") {a.scan_leaf = std::atof(next("--scan-leaf"));} else if (k == "--max-corr") {
      a.max_corr = std::atof(next("--max-corr"));
    } else if (k == "--iters") {a.iters = std::atoi(next("--iters"));} else if (k == "--axis") {
      a.axis = next("--axis");
    } else if (k == "--sweep") {
      const std::string s = next("--sweep");
      if (std::sscanf(s.c_str(), "%lf:%lf:%lf", &a.sweep_lo, &a.sweep_hi, &a.sweep_step) != 3) {
        std::fprintf(stderr, "[selfsim] --sweep 需要 lo:hi:step\n"); return false;
      }
    } else if (k == "-h" || k == "--help") {usage(); std::exit(0);} else {
      std::fprintf(stderr, "[selfsim] 未知参数 %s\n", k.c_str()); return false;
    }
  }
  if (a.pcd.empty()) {std::fprintf(stderr, "[selfsim] 必须给 --pcd\n"); return false;}
  return true;
}

Eigen::Matrix4d poseMat(double x, double y, double z, double yaw)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  m(0, 3) = x; m(1, 3) = y; m(2, 3) = z;
  return m;
}
}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  if (!parseArgs(argc, argv, a)) {return 2;}

  // ---- 地图：与节点一致（loadPCDFile<PointXYZ> → removeNaN → 粗 leaf）----
  PointCloudT::Ptr raw(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(a.pcd, *raw) < 0) {
    std::fprintf(stderr, "[selfsim] PCD 读取失败 %s\n", a.pcd.c_str()); return 1;
  }
  pcl::Indices idx;
  PointCloudT::Ptr finite(new PointCloudT);
  pcl::removeNaNFromPointCloud(*raw, *finite, idx);
  const PointCloudT::Ptr map_ds = gicp_registration::voxelDownsample(finite, a.map_leaf);

  // ---- 真值位姿 T_map_sensor（合成扫描的"真值"，真实点云时由调用者用 --pose 声明）----
  const Eigen::Matrix4d T_true = poseMat(a.px, a.py, a.sensor_z, a.pyaw);

  // ---- source ----
  PointCloudT::Ptr scan_raw(new PointCloudT);
  if (!a.scan.empty()) {
    if (pcl::io::loadPCDFile<PointT>(a.scan, *scan_raw) < 0) {
      std::fprintf(stderr, "[selfsim] 真实点云读取失败 %s\n", a.scan.c_str()); return 1;
    }
    std::printf("[selfsim] source = **真实点云** %s（%zu 点）；真值位姿由 --pose 声明 ⇒ "
                "把雷达系点云用 T_true 变到 map 系再对齐\n", a.scan.c_str(), scan_raw->size());
    // 真实点云给的是**雷达系**坐标 ⇒ 转到 map 系，再当作"从 map 系观测到的点"，
    // 下面的合成路径统一按"map 系 → 雷达系"处理 ⇒ 这里先逆变换。
    PointCloudT::Ptr tmp(new PointCloudT);
    const Eigen::Matrix4d T_sensor_map = T_true.inverse();
    for (const auto & p : scan_raw->points) {
      const Eigen::Vector4d q = T_sensor_map * Eigen::Vector4d(p.x, p.y, p.z, 1.0);
      PointT o; o.x = q.x(); o.y = q.y(); o.z = q.z(); tmp->push_back(o);
    }
    scan_raw = tmp;
  } else {
    const double emin = a.elev_min_deg * M_PI / 180.0, emax = a.elev_max_deg * M_PI / 180.0;
    const Eigen::Matrix4d T_sensor_map = T_true.inverse();
    for (const auto & p : finite->points) {
      const double dx = p.x - a.px, dy = p.y - a.py, dz = p.z - a.sensor_z;
      const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (r < 1e-3 || r > a.range) {continue;}
      const double elev = std::asin(dz / r);   // 俯仰（+ = 向上）
      if (elev < emin || elev > emax) {continue;}
      const Eigen::Vector4d q = T_sensor_map * Eigen::Vector4d(p.x, p.y, p.z, 1.0);
      PointT o;
      o.x = static_cast<float>(q.x() + (a.noise > 0 ? a.noise * (std::rand() / double(RAND_MAX) - 0.5) : 0.0));
      o.y = static_cast<float>(q.y() + (a.noise > 0 ? a.noise * (std::rand() / double(RAND_MAX) - 0.5) : 0.0));
      o.z = static_cast<float>(q.z() + (a.noise > 0 ? a.noise * (std::rand() / double(RAND_MAX) - 0.5) : 0.0));
      scan_raw->push_back(o);
    }
    std::printf("[selfsim] source = **合成**（地图切片：range≤%.1f m，俯仰 %.2f°~%.2f°，"
                "**无遮挡** ⇒ 自相似性上界）%zu 点\n",
                a.range, a.elev_min_deg, a.elev_max_deg, scan_raw->size());
  }
  // 等间隔抽到 scan_points（与 bench/gicp_backend_bench 同口径）：真实单帧约 3~4 千点。
  if (a.scan_points > 0 && scan_raw->size() > static_cast<size_t>(a.scan_points)) {
    PointCloudT::Ptr thin(new PointCloudT);
    const size_t n0 = scan_raw->size();
    const size_t stride = std::max<size_t>(1, n0 / static_cast<size_t>(a.scan_points));
    for (size_t i = 0; i < n0 && thin->size() < static_cast<size_t>(a.scan_points); i += stride) {
      thin->push_back(scan_raw->points[i]);
    }
    scan_raw = thin;
  }
  const PointCloudT::Ptr source = gicp_registration::voxelDownsample(scan_raw, a.scan_leaf);

  // ---- 后端（与节点同参数）----
  gicp_registration::RegistrationBackendOptions o;
  bool ok = false;
  o.kind = gicp_registration::parseRegistrationBackendKind(a.backend, ok);
  o.max_correspondence_distance = a.max_corr;
  o.maximum_iterations = a.iters;
  o.correspondence_randomness = a.corr_rand;
  o.maximum_optimizer_iterations = a.opt_iters;
  o.small_gicp_num_threads = a.threads;
  gicp_registration::RegistrationBackend backend(o);
  const double prep = backend.setTarget(map_ds);
  std::printf("[selfsim] pcd=%s 原始 %zu → 去 NaN %zu → leaf %.3f 后 target %zu 点 | source %zu 点"
              "（leaf %.3f）| backend=%s max_corr=%.2f iters=%d | setTarget=%.0f ms\n",
              a.pcd.c_str(), raw->size(), finite->size(), a.map_leaf, backend.targetSize(),
              source->size(), a.scan_leaf, backend.name(), a.max_corr, a.iters, prep);

  // ---- 扫描 ----
  const int n = static_cast<int>(std::floor((a.sweep_hi - a.sweep_lo) / a.sweep_step + 0.5)) + 1;
  double worst_fit = 0.0, max_final_err = 0.0;
  int converged_n = 0, n_tot = 0;
  for (int i = 0; i < n; ++i) {
    const double d = a.sweep_lo + i * a.sweep_step;
    Eigen::Matrix4d seed_delta = Eigen::Matrix4d::Identity();
    if (a.axis == "x") {seed_delta(0, 3) = d;}
    else if (a.axis == "y") {seed_delta(1, 3) = d;}
    else if (a.axis == "yaw") {
      seed_delta.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(d * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    } else if (a.axis == "xy") {
      seed_delta(0, 3) = d; seed_delta(1, 3) = d;
    } else {std::fprintf(stderr, "[selfsim] 未知 axis %s\n", a.axis.c_str()); return 2;}
    // 初值 = 真值 ∘ 偏差（与"上一帧结果 + 里程计"偏掉的情形等价）
    const Eigen::Matrix4d guess = T_true * seed_delta;
    const auto t0 = std::chrono::steady_clock::now();
    const gicp_registration::RegistrationOutcome out = backend.align(source, guess);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // 结果相对**初值**挪了多少（残留偏差 = 自传播的驱动量）
    const Eigen::Matrix4d d_res = guess.inverse() * out.T_target_source;
    const double res_xy = std::hypot(d_res(0, 3), d_res(1, 3));
    const double res_yaw = std::atan2(d_res(1, 0), d_res(0, 0)) * 180.0 / M_PI;
    // 结果离**真值**多远
    const Eigen::Matrix4d d_err = T_true.inverse() * out.T_target_source;
    const double err_xy = std::hypot(d_err(0, 3), d_err(1, 3));
    const double err_z = d_err(2, 3);
    const double err_yaw = std::atan2(d_err(1, 0), d_err(0, 0)) * 180.0 / M_PI;
    std::printf(
      "[selfsim] axis=%-3s seed=%+7.3f converged=%d fitness=%s | 相对初值挪动 xy=%+.4f yaw=%+.3f° | "
      "离真值 xy=%+.4f z=%+.4f yaw=%+.3f° | %.1f ms\n",
      a.axis.c_str(), d, out.converged ? 1 : 0,
      std::isfinite(out.fitness_score) ? std::to_string(out.fitness_score).c_str() : "nan",
      res_xy, res_yaw, err_xy, err_z, err_yaw, ms);
    if (out.converged) {converged_n++;}
    n_tot++;
    if (std::isfinite(out.fitness_score)) {worst_fit = std::max(worst_fit, out.fitness_score);}
    max_final_err = std::max(max_final_err, err_xy);
  }
  std::printf("[selfsim] 汇总：%d/%d 收敛；fitness 最大 %.5f m²；离真值最大 %.3f m"
              "（节点接受阈值 max_fitness_score=0.3 ⇒ fitness ≤0.3 时这个偏差**看不见**）\n",
              converged_n, n_tot, worst_fit, max_final_err);
  return 0;
}
