// =============================================================================
// slope_speed_offline.cpp —— 前瞻限速的**离线合成扫掠**（不需要 Gazebo / ROS）
// -----------------------------------------------------------------------------
// 为什么要有它：整栈仿真的 A/B 很贵（一次跑 4~6 分钟）而且受定位/规划抖动影响，
//   "过坡脚时的速度上限到底降到多少"这件事用**合成点云**就能量：把坡脚/台阶放在车前 df 米，
//   按 MID360 的真实下视 FOV（−7.22°、离地 0.226 m）把看不见的点去掉，然后
//   `LowTerrainClassifier::{apply,describeCorridor}` + `SlopeSpeedLimiter::update` 逐帧推。
//
// 它**不是**仿真替代品（没有规划/控制/定位），只用来：① 调参数（brake / 死区）时看趋势；
//   ② 验收"平地不限速、坡脚/台阶前降到 v_req"这两条性质。整栈实测见
//   docs/slope_speed_limiting.md §3。
//
// 构建与运行（不需要 colcon；只需要 PCL 头）：
//   g++ -O2 -std=c++17 -I src/rm_perception/rm_ground_traversability/include
//       -I /usr/include/pcl-1.12 -I /usr/include/eigen3
//       src/rm_perception/rm_ground_traversability/bench/slope_speed_offline.cpp -o /tmp/ssl_bench
//   /tmp/ssl_bench              # 只看汇总
//   /tmp/ssl_bench -v           # 打印逐帧曲线
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "rm_ground_traversability/slope_speed_limit.hpp"

using namespace rm_ground_traversability;

namespace
{
constexpr double kSensorH = 0.226;                                   // 离地（与 URDF/标定一致）
const double kTanFov = std::tan(7.22 * kPi / 180.0);                  // 下视半角
constexpr double kNoise = 0.002;                                     // 仿真点云 σ

/// 车前方 df 米处放一个特征（坡脚 / 台阶）；只保留雷达 FOV 内的点。
pcl::PointCloud<pcl::PointXYZ> cloudAt(double df, double slope_deg, double step_m)
{
  std::mt19937 rng(1234);
  std::normal_distribution<double> nz(0.0, kNoise);
  pcl::PointCloud<pcl::PointXYZ> c;
  const double t = std::tan(slope_deg * kPi / 180.0);
  for (double x = 0.3; x <= 6.0; x += 0.06) {
    for (double y = -0.6; y <= 0.6; y += 0.05) {
      double z = 0.0;
      if (slope_deg > 0.0 && x >= df) {
        z = (x - df) * t;
      }
      if (step_m > 0.0 && x >= df) {
        z = step_m;
      }
      if (z < kSensorH - x * kTanFov) {
        continue;                                     // 这条射线打到地面以下 ⇒ 雷达看不见
      }
      pcl::PointXYZ p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z + nz(rng));
      c.push_back(p);
    }
  }
  return c;
}

struct SweepResult
{
  double v_at_feature{0.0};   ///< df=0 那一帧的速度上限（= 过特征时的速度）
  double v_min{1e9};          ///< 全程最小上限
  double v_min_df{0.0};
  double v_at_2m{0.0};        ///< df=+2.0 m 时的上限（看是否"提前"减速）
  bool limited_on_flat{false};
};

SweepResult sweep(const char * name, double slope_deg, double step_m, bool verbose)
{
  Criteria cr;
  LowTerrainClassifier cls(cr);
  CorridorQuery q;
  q.lookahead_m = 3.0;
  q.half_width_m = 0.28;
  q.spread_deg = 8.0;
  CorridorProfile prof;
  SpeedLimitCriteria sc;
  sc.drivable_slope_deg = cr.drivable_slope_deg;
  SlopeSpeedLimiter lim(sc);
  lim.reset();

  SweepResult r;
  if (verbose) {
    printf("  df(m)  限速(m/s)  why            v_req  承诺  Δ坡(deg)  台阶(m)  近坡(deg)\n");
  }
  for (double df = 2.8; df >= -2.0; df -= 0.1) {
    auto c = cloudAt(df, slope_deg, step_m);
    std::vector<uint8_t> g(c.size(), 1u);
    cls.apply(c, &g, nullptr);
    cls.describeCorridor(q, &prof);
    SpeedLimitDecision d = lim.update(prof, 0.1);
    if (std::abs(df) < 0.05) {
      r.v_at_feature = d.limit_mps;
    }
    if (std::abs(df - 2.0) < 0.05) {
      r.v_at_2m = d.limit_mps;
    }
    if (d.limit_mps < r.v_min) {
      r.v_min = d.limit_mps;
      r.v_min_df = df;
    }
    if (slope_deg <= 0.0 && step_m <= 0.0 && d.limited) {
      r.limited_on_flat = true;
    }
    if (verbose) {
      printf("  %+5.2f  %8.2f  %-13s %5.2f  %-4s  %6.1f  %6.3f  %6.2f\n", df, d.limit_mps,
        d.limited ? d.reason : "none", d.v_req_mps, d.from_hold ? "Y" : "-",
        d.feature_dtan_deg, d.feature_step_m, d.near_slope_deg);
    }
  }
  printf("%-22s 过特征上限=%.2f m/s | 2 m 前上限=%.2f | 全程最小=%.2f (df=%+.1f) | 平地误限=%s\n",
    name, r.v_at_feature, r.v_at_2m, r.v_min, r.v_min_df, r.limited_on_flat ? "**是**" : "否");
  return r;
}
}  // namespace

int main(int argc, char ** argv)
{
  const bool verbose = (argc > 1 && std::strcmp(argv[1], "-v") == 0);
  SpeedLimitCriteria sc;
  printf("=== 前瞻限速离线扫掠（合成点云 + MID360 下视 FOV −7.22°）===\n");
  printf("vx_max=%.2f m/s floor=%.2f brake=%.2f release=%.2f Δt_eff=%.2f ms\n\n",
    sc.vx_max, sc.floor_mps, sc.brake_mps2, sc.release_mps2, sc.dt_eff_s() * 1e3);
  sweep("平地（对照）", 0.0, 0.0, verbose);
  sweep("10% 坡(5.7°)坡脚", 5.7, 0.0, verbose);
  sweep("11° 坡脚", 11.0, 0.0, verbose);
  sweep("24° 坡脚", 24.0, 0.0, verbose);
  sweep("0.045 m 台阶", 0.0, 0.045, verbose);
  sweep("0.10 m 台阶", 0.0, 0.10, verbose);
  sweep("0.30 m 台阶(高台)", 0.0, 0.30, verbose);
  return 0;
}
