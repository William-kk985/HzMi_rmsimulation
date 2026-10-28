// 回归测试：`/initialpose`（以及 initial_pose 参数）的种子组合 + round-trip 不变量。
//
// 这个测试**不启动 ROS、不读 PCD**，只测 `include/gicp_registration/seed_math.hpp` 的纯数学。
// 它钉住两件事（契约 §4.17 / meas §43.2）：
//   ① 正确公式 T_map_odom = T_map_base · T_odom_base.inverse() 的 round-trip 必须回到
//      T_map_base（平移 < 1 mm、旋转 < 0.1°）；
//   ② §43 那个"少求逆"的写法在 T_odom_base ≠ I 时**必须**被这项判据抓住
//      （否则这个测试就没有防回归价值）。
//
// 构建/运行：
//   colcon build --symlink-install --packages-select gicp_registration
//   ./build/gicp_registration/gicp_seed_math_test          # 直接跑
//   colcon test --packages-select gicp_registration       # 走 ctest

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gicp_registration/seed_math.hpp"

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
  using gicp_registration::composeMapOdomFromSeed;
  using gicp_registration::seedRoundTrip;

  std::printf("gicp_seed_math_test：§4.17 种子公式 + round-trip\n");

  // ---------------- ① §43.2 的实测数值（live/replay 的同一组） ----------------
  // /initialpose（SC keyframe 6）= map 系 base_link 位姿；T_odom_base = T_bl（纯平移，LIO 刚初始化）。
  const double yaw_seed = 42.62 * kPi / 180.0;
  const Eigen::Matrix4d T_map_base =
    makePose(1.1851, 0.1594, 0.0024, 0.0, 0.0, yaw_seed);
  const Eigen::Matrix4d T_odom_base =
    makePose(0.000562, 0.130916, 0.157028, 0.0, 0.0, 0.0);
  const Eigen::Matrix4d T_map_odom = composeMapOdomFromSeed(T_map_base, T_odom_base);

  // 修复后 map→odom 的解析值（§43.2「正确时应为」）
  expect(std::abs(T_map_odom(0, 3) - 1.2733) < 1e-3, "§43 种子：map→odom.x ≈ 1.2733");
  expect(std::abs(T_map_odom(1, 3) - 0.0627) < 1e-3, "§43 种子：map→odom.y ≈ 0.0627");
  expect(std::abs(T_map_odom(2, 3) + 0.1546) < 1e-3, "§43 种子：map→odom.z ≈ −0.1546");

  const auto rt = seedRoundTrip(T_map_odom, T_odom_base, T_map_base);
  std::printf("  round-trip: trans=%.9f m, rot=%.9f°\n", rt.trans_m, rt.rot_deg);
  expect(rt.ok, "§43 种子：round-trip 回到 /initialpose（<1 mm / <0.1°）");

  // 旧写法（少一个 inverse）必须被抓住：它把种子推到 T_map_base·T_bl²。
  const Eigen::Matrix4d T_map_odom_old = T_map_base * T_odom_base;
  const auto rt_old = seedRoundTrip(T_map_odom_old, T_odom_base, T_map_base);
  std::printf("  旧写法（少求逆）round-trip: trans=%.6f m, rot=%.6f°\n",
    rt_old.trans_m, rt_old.rot_deg);
  expect(!rt_old.ok, "旧写法（少求逆）必须被 round-trip 判据拒绝");
  expect(std::abs(rt_old.trans_m - 0.4089) < 5e-3, "旧写法的误差 ≈ §43 的 0.409 m（3D）");

  // ---------------- ② T_odom_base ≈ I：新旧写法一致（"默认路径逐字节等价"的数学面） ----------------
  {
    const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d a = composeMapOdomFromSeed(T_map_base, I);
    const Eigen::Matrix4d b = T_map_base * I;
    expect((a - b).norm() < 1e-12, "T_odom_base=I ⇒ 与旧写法逐元素相同");
    expect(seedRoundTrip(a, I, T_map_base).ok, "T_odom_base=I ⇒ round-trip 通过");
  }

  // ---------------- ③ 随机位姿（含 30° 倾斜、米级平移、随机 yaw） ----------------
  std::mt19937 rng(20261029u);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double max_trans = 0.0;
  double max_rot = 0.0;
  int n_ok = 0;
  constexpr int N = 500;
  for (int i = 0; i < N; ++i) {
    const Eigen::Matrix4d T_mb = makePose(5.0 * u(rng), 5.0 * u(rng), 0.5 * u(rng),
        0.6 * u(rng), 0.6 * u(rng), kPi * u(rng));
    // 覆盖 §43 的两种几何：live 种子时刻（纯平移 T_bl）与建图帧（30° 倾斜 + 米级平移）
    const Eigen::Matrix4d T_ob = (i % 2 == 0)
      ? makePose(0.3 * u(rng), 0.5 * u(rng), 0.3 * u(rng), 0.0, 0.0, 0.2 * u(rng))
      : makePose(2.0 * u(rng), 2.0 * u(rng), 0.3 * u(rng),
          30.0 * kPi / 180.0, 0.0, kPi * u(rng));
    const Eigen::Matrix4d T_mo = composeMapOdomFromSeed(T_mb, T_ob);
    const auto r = seedRoundTrip(T_mo, T_ob, T_mb);
    max_trans = std::max(max_trans, r.trans_m);
    max_rot = std::max(max_rot, r.rot_deg);
    if (r.ok) {
      ++n_ok;
    }
  }
  std::printf("  %d 个随机位姿：max trans=%.3e m, max rot=%.3e°\n", N, max_trans, max_rot);
  expect(n_ok == N, "500 个随机位姿全部 round-trip 通过");
  expect(max_trans < 1e-9 && max_rot < 1e-5, "随机位姿的 round-trip 数值误差可忽略");

  if (g_failures == 0) {
    std::printf("\n全部通过 ✅\n");
    return 0;
  }
  std::printf("\n%d 条失败 ❌\n", g_failures);
  return 1;
}
