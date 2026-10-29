// 回归测试：种后**宽门**（前 K 帧）+ **空间信任域**（契约 §4.17.6 / meas §46.1）。
//
// 不启动 ROS、不读 PCD，只测 include/gicp_registration/seed_trust.hpp 的纯数学。
// 它钉住 §45/§46 留下的一对矛盾：
//   · 种子比普通门限就差 8 mm（首帧创新 0.308 vs 0.300）时，**合法**精修不能被卡死；
//   · 同时不允许"每帧走一个门限、累积走到离种子任意远"。
// 数值全部取 §45.5b 的实测（kf8 种子 / GT / 终点）。
//
// 构建/运行：
//   colcon build --symlink-install --packages-select gicp_registration
//   ./build/gicp_registration/gicp_seed_trust_test
//   colcon test --packages-select gicp_registration

#include <cmath>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gicp_registration/seed_trust.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

int g_failures = 0;

void expect(bool cond, const char * what)
{
  if (!cond) {
    std::printf("  ✗ FAIL: %s\n", what);
    ++g_failures;
  } else {
    std::printf("  ✓ %s\n", what);
  }
}

void expect_near(double got, double want, double tol, const char * what)
{
  const bool ok = std::abs(got - want) <= tol;
  if (!ok) {
    std::printf("  ✗ FAIL: %s（got %.9f, want %.9f ± %.9f）\n", what, got, want, tol);
    ++g_failures;
  } else {
    std::printf("  ✓ %s（%.6f）\n", what, got);
  }
}

Eigen::Matrix4d makePose(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) =
    (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  m(0, 3) = x;
  m(1, 3) = y;
  m(2, 3) = z;
  return m;
}

}  // namespace

int main()
{
  using gicp_registration::seedGateThresholds;
  using gicp_registration::seedTrustDeviation;

  std::printf("gicp_seed_trust_test：§4.17.6 种后宽门 + 空间信任域\n");

  // ---------------- ① 宽门：只在窗口内前 K 帧生效，且只放宽不收窄 ----------------
  {
    const auto f1 = seedGateThresholds(0.30, 3.0, true, 1, 5, 0.45, 10.0);
    expect(f1.wide, "窗口第 1 帧 ⇒ 宽门生效");
    expect_near(f1.trans_m, 0.45, 1e-12, "第 1 帧平移门 = 0.45 m");
    expect_near(f1.yaw_deg, 10.0, 1e-12, "第 1 帧 yaw 门 = 10°");

    const auto f5 = seedGateThresholds(0.30, 3.0, true, 5, 5, 0.45, 10.0);
    expect(f5.wide, "窗口第 5 帧（= K）⇒ 宽门仍生效");

    const auto f6 = seedGateThresholds(0.30, 3.0, true, 6, 5, 0.45, 10.0);
    expect(!f6.wide, "窗口第 6 帧（> K）⇒ 回到普通门");
    expect_near(f6.trans_m, 0.30, 1e-12, "第 6 帧平移门 = 0.30 m");

    const auto off = seedGateThresholds(0.30, 3.0, false, 1, 5, 0.45, 10.0);
    expect(!off.wide && std::abs(off.trans_m - 0.30) < 1e-12, "不在窗口内 ⇒ 普通门不变");

    // dt 变大时普通门可能已经 0.6（clamp 上限）⇒ 宽门不能把它**收窄**
    const auto wide_dt = seedGateThresholds(0.60, 15.0, true, 2, 5, 0.45, 10.0);
    expect_near(wide_dt.trans_m, 0.60, 1e-12, "普通门更宽时取 max（0.60 不收窄到 0.45）");
    expect_near(wide_dt.yaw_deg, 15.0, 1e-12, "yaw 同理（15° 不收窄到 10°）");

    // K=0 / 负帧号 = 关闭宽门
    const auto k0 = seedGateThresholds(0.30, 3.0, true, 1, 0, 0.45, 10.0);
    expect(!k0.wide, "seed_gate_frames=0 ⇒ 宽门关闭");
  }

  // ---------------- ② §45.5b 的 kf8 实测：种子 / GT / 终点 ----------------
  // 种子（SC kf8）= map 系 base_link 位姿：(2.1604, 0.9728) yaw 55.79°
  // GT（teleport 真值）= (1.9798, 0.7789) yaw 53.86°
  // 终点（§45 跑完）= (1.9798, 0.7439) yaw 60.47°
  const Eigen::Matrix4d T_seed = makePose(2.1604, 0.9728, 0.0, 0.0, 0.0, 55.79 * kPi / 180.0);
  const Eigen::Matrix4d T_gt = makePose(1.9798, 0.7789, 0.0, 0.0, 0.0, 53.86 * kPi / 180.0);
  const Eigen::Matrix4d T_end = makePose(1.9798, 0.7439, 0.0, 0.0, 0.0, 60.47 * kPi / 180.0);

  const auto d_gt = seedTrustDeviation(T_seed, T_gt, 0.35, 5.0);
  expect_near(d_gt.trans_m, 0.2650, 5e-4, "kf8 种子→GT 平移 ≈ 0.265 m");
  expect_near(d_gt.rot_deg, 1.93, 2e-2, "kf8 种子→GT yaw ≈ 1.93°");
  expect(d_gt.ok, "kf8 种子→GT 在 0.35 m / 5° 信任域内（允许精修）");

  const auto d_end = seedTrustDeviation(T_seed, T_end, 0.35, 5.0);
  expect_near(d_end.trans_m, 0.2916, 5e-4, "kf8 终点→种子 平移 ≈ 0.292 m");
  expect_near(d_end.rot_deg, 4.68, 2e-2, "kf8 终点→种子 yaw ≈ 4.68°");
  expect(d_end.ok, "kf8 终点在 0.35 m / 5° 信任域内（§45 的 0.292 m / 4.68°）");

  // 0.15 m 的信任域会把**合法**精修挡掉（这就是默认值不能取 0.15 的实测依据）
  const auto d_end_tight = seedTrustDeviation(T_seed, T_end, 0.15, 5.0);
  expect(!d_end_tight.ok, "0.15 m 信任域会拒绝 kf8 的合法精修（⇒ 默认取 0.35）");

  // 0.30 m 只剩 8 mm 余量（§45 终点 0.2916）⇒ 能过但太脆，文档里登记
  expect(
    seedTrustDeviation(T_seed, T_end, 0.30, 5.0).ok,
    "0.30 m 信任域能过（余量 8 mm，太脆 ⇒ 不取作默认）");

  // ---------------- ③ 故意错的种子（§44.3 iii：+2.0 m / +0.5 m / +15°）必须被拒 ----------------
  {
    const Eigen::Matrix4d T_wrong =
      makePose(2.1604 + 2.0, 0.9728 + 0.5, 0.0, 0.0, 0.0, (55.79 + 15.0) * kPi / 180.0);
    const auto d = seedTrustDeviation(T_wrong, T_gt, 0.35, 5.0);
    expect_near(d.trans_m, std::hypot(2.0 + 0.1806, 0.5 + 0.1939), 1e-3, "错种子→GT 平移 ≈ 2.29 m");
    expect_near(d.rot_deg, 15.0 + 1.93, 0.05, "错种子→GT 旋转 ≈ 16.93°");
    expect(!d.ok, "错种子的修正（≈2 m / 15°）被信任域拒绝");
  }

  // ---------------- ④ 运动不变性：信任域作用在 map→odom，不把机器人位移算成跑飞 ----------------
  {
    // 种子：T_odom_base 带 30° 倾斜（robot11 的 gravity:=tf 链），T_map_base = 种子位姿
    const Eigen::Matrix4d T_ob_seed = makePose(0.0, 0.1081, -0.1873, 30.0 * kPi / 180.0, 0.0, 0.0);
    const Eigen::Matrix4d seed_T_map_odom = T_seed * T_ob_seed.inverse();
    // 机器人走了 0.8 m（odom→base 变），GICP 一帧都没改（完美的里程计跟踪）
    const Eigen::Matrix4d T_ob_now = makePose(0.8, 0.1081, -0.1873, 30.0 * kPi / 180.0, 0.0, 0.0);
    const Eigen::Matrix4d cand_T_map_odom = seed_T_map_odom;   // map→odom 不变
    const auto d = seedTrustDeviation(seed_T_map_odom, cand_T_map_odom, 0.35, 5.0);
    expect_near(d.trans_m, 0.0, 1e-12, "机器人走 0.8 m 但无修正 ⇒ 信任域偏差 = 0");
    // 而 map→base 确实离种子 0.8 m —— 证明信任域量的**不是**机器人位移
    const double moved = ((seed_T_map_odom * T_ob_now).block<3, 1>(0, 3) -
      (seed_T_map_odom * T_ob_seed).block<3, 1>(0, 3)).norm();
    expect_near(moved, 0.8, 1e-9, "同一场景下 map→base 确实移动了 0.8 m（不是同一把尺子）");
  }

  // ---------------- ⑤ 带 30° 滚转的 map→odom：纯 yaw 修正的 rot_deg = yaw 差 ----------------
  {
    const Eigen::Matrix4d base = makePose(0.1, 0.2, -0.15, 30.0 * kPi / 180.0, 0.0, 0.3);
    Eigen::Matrix4d cand = base;
    cand.block<3, 3>(0, 0) =
      base.block<3, 3>(0, 0) * Eigen::AngleAxisd(2.5 * kPi / 180.0, Eigen::Vector3d::UnitZ())
      .toRotationMatrix();
    const auto d = seedTrustDeviation(base, cand, 0.35, 5.0);
    expect_near(d.trans_m, 0.0, 1e-12, "纯 yaw 修正的平移到 0");
    expect_near(d.rot_deg, 2.5, 1e-9, "30° 滚转链上 rot_deg 仍 = yaw 差（2.5°）");
  }

  std::printf("\n%s（failures=%d）\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
  return g_failures == 0 ? 0 : 1;
}
