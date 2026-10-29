#ifndef GICP_REGISTRATION__SEED_TRUST_HPP_
#define GICP_REGISTRATION__SEED_TRUST_HPP_

// 全局种子的**种后信任域**（契约 §4.17.6 / meas §46.1）——纯数学，便于单测。
//
// §44 的信任窗口只做了一件事：种后 T 秒内**禁用重锚豁免**。它留下两个尾巴（§45 / §46）：
//   ① 首帧创新 0.308 m 只比普通门限 0.300 m 多 8 mm ⇒ 一个**合法**的种子精修被
//      "毫米级"卡死，52 帧全拒 ⇒ NO_CONVERGE；
//   ② 没有"空间"约束：门限只约束**逐帧增量**，而窗口内每帧都从**新的**初值出发 ⇒ 理论上
//      可以每帧走一个门限、累积走到离种子任意远的地方。
//
// 本头文件提供两个纯函数：
//   · seedGateThresholds()   —— 种后前 K 帧把创新门限放宽到 seed_gate_*（取 max，不比普通门限更紧）；
//   · seedTrustDeviation()    —— 候选 map→odom 相对**种子**的偏差（平移 m / 旋转 °）+ 是否在域内。
//
// 为什么信任域作用在 **map→odom** 而不是 map→base_link：
//   机器人自身在窗口内可以合法移动（里程计会把它带走）。map→odom 是本节点拥有的"修正量"，
//   与机器人运动无关 ⇒ 同一段里"离种子多远"才是纯修正量，不会把里程计位移误判成跑飞。
//   对同一个 TF 链，平移/旋转差的量级与作用在 map→base 上等价（差一个刚体共轭），
//   这一点在 test/gicp_seed_trust_test.cpp 里用"机器人走了 0.8 m 但没做修正"的用例钉住。

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gicp_registration
{

// 有效创新门限：窗口内、且本帧是窗口内的第 1..gate_frames 帧 ⇒ 取 max(普通, seed_gate_*)。
// frames_in_window 是 **1-based** 的窗口内帧号（0/负数 = 不是窗口帧）。
struct SeedGateThresholds
{
  double trans_m = 0.0;
  double yaw_deg = 0.0;
  bool wide = false;   // true = 本帧用了放宽后的种子门
};

inline SeedGateThresholds seedGateThresholds(
  double base_trans_m, double base_yaw_deg, bool window_active, int frames_in_window,
  int gate_frames, double seed_gate_trans_m, double seed_gate_yaw_deg)
{
  SeedGateThresholds out{base_trans_m, base_yaw_deg, false};
  if (!window_active || gate_frames <= 0 || frames_in_window <= 0) {
    return out;
  }
  if (frames_in_window > gate_frames) {
    return out;
  }
  // 取 max：种子门**只能放宽**，不会把普通门限收得更紧（否则等于换了一个门槛，A/B 不可比）。
  out.trans_m = std::max(base_trans_m, seed_gate_trans_m);
  out.yaw_deg = std::max(base_yaw_deg, seed_gate_yaw_deg);
  out.wide = true;
  return out;
}

// 候选 map→odom 相对种子的偏差。checked=false 只在"没有种子/没比较"时出现（本函数总是 true）。
struct SeedTrustDeviation
{
  bool checked = false;
  bool ok = false;
  double trans_m = 0.0;   // ‖T_cand − T_seed‖ 的平移部分（m）
  double rot_deg = 0.0;   // 相对旋转夹角（°；纯 yaw 差时 = yaw 差，含 30° 倾斜链也成立）
};

inline SeedTrustDeviation seedTrustDeviation(
  const Eigen::Matrix4d & seed_T_map_odom, const Eigen::Matrix4d & candidate_T_map_odom,
  double max_trans_m, double max_rot_deg)
{
  SeedTrustDeviation out;
  out.checked = true;
  const Eigen::Matrix4d d = seed_T_map_odom.inverse() * candidate_T_map_odom;
  out.trans_m = d.block<3, 1>(0, 3).norm();
  // 旋转夹角用 acos((tr(R)−1)/2)（与 seed_math.hpp 的 rotationAngleDeg 同口径，避免
  // atan2(R10,R00) 在带 30° 倾斜的 map→odom 上把 roll 也算进 yaw）。
  const double c = d.block<3, 3>(0, 0).trace();
  constexpr double kPi = 3.14159265358979323846;
  out.rot_deg = std::acos(std::max(-1.0, std::min(1.0, (c - 1.0) * 0.5))) * 180.0 / kPi;
  out.ok = (out.trans_m <= max_trans_m) && (out.rot_deg <= max_rot_deg);
  return out;
}

}  // namespace gicp_registration

#endif  // GICP_REGISTRATION__SEED_TRUST_HPP_
