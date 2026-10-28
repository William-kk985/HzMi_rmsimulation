#ifndef GICP_REGISTRATION__SEED_MATH_HPP_
#define GICP_REGISTRATION__SEED_MATH_HPP_

// 初值（/initialpose / initial_pose 参数）→ map→odom 的**唯一**组合公式 + 回归不变量。
//
// 背景（契约 §4.17 / meas §43.2）：`/initialpose` 的语义是 **map 系机器人（base_link）位姿**
// T_map_base，而 tf2 的 `lookupTf(odom, base_link)` 返回的是 **T_odom←base**（target←source）。
// 所以正确组合是
//
//     T_map←odom = T_map←base · T_base←odom = T_map_base · T_odom_base.inverse()
//
// §43 的 bug 就是漏了 `.inverse()`。它只在 T_odom_base ≈ I（旧场地、LIO 刚初始化且纯平移噪声）
// 时隐形；本仓 2027 的 gravity:=tf 一旦让 T_odom_base 带上 30° 倾斜/米级平移，种子就被扔到
// 4.5 m / 125°（§43.1 回放对照）。
//
// 这里把公式**收成一个函数**，节点两条路径（/initialpose 与 initial_pose 参数）都调它，
// 并提供一个 round-trip 判据：把组合结果重新乘回 T_odom_base，必须回到 T_map_base
// （平移 < 1 mm、旋转 < 0.1°，契约 §4.17 的种子不变量）。`test/gicp_seed_math_test.cpp`
// 用随机位姿 + §43 的实测数值把这两条钉住（防回归）。

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gicp_registration
{

// T_map←odom = T_map←base · T_base←odom。T_odom_base 必须是 tf2 的 target←source 语义。
inline Eigen::Matrix4d composeMapOdomFromSeed(
  const Eigen::Matrix4d & T_map_base, const Eigen::Matrix4d & T_odom_base)
{
  return T_map_base * T_odom_base.inverse();
}

// 种子的 round-trip 误差：把 T_map_odom 重新乘上 T_odom_base，应当回到 T_map_base。
struct SeedRoundTrip
{
  double trans_m = 0.0;    // 平移误差的欧氏范数（m）
  double rot_deg = 0.0;    // 旋转误差的夹角（度）
  bool ok = false;         // trans_m <= tol_m && rot_deg <= tol_deg
};

// 旋转矩阵夹角（度）：acos((tr(R1ᵀR2) − 1)/2)，与本仓其它诊断同一口径。
inline double rotationAngleDeg(const Eigen::Matrix3d & R1, const Eigen::Matrix3d & R2)
{
  const double c = (R1.transpose() * R2).trace();
  constexpr double kPi = 3.14159265358979323846;
  return std::acos(std::max(-1.0, std::min(1.0, (c - 1.0) * 0.5))) * 180.0 / kPi;
}

inline SeedRoundTrip seedRoundTrip(
  const Eigen::Matrix4d & T_map_odom, const Eigen::Matrix4d & T_odom_base,
  const Eigen::Matrix4d & T_map_base, double tol_m = 1e-3, double tol_deg = 0.1)
{
  const Eigen::Matrix4d T_map_base_rt = T_map_odom * T_odom_base;
  SeedRoundTrip out;
  out.trans_m = (T_map_base_rt.block<3, 1>(0, 3) - T_map_base.block<3, 1>(0, 3)).norm();
  out.rot_deg = rotationAngleDeg(
    T_map_base_rt.block<3, 3>(0, 0), T_map_base.block<3, 3>(0, 0));
  out.ok = (out.trans_m <= tol_m) && (out.rot_deg <= tol_deg);
  return out;
}

}  // namespace gicp_registration

#endif  // GICP_REGISTRATION__SEED_MATH_HPP_
