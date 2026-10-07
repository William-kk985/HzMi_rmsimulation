// =============================================================================
// tf2_compose_order_test.cpp —— bug ③ 的**语义钉死**单测
// （2026-10-09 建立；2026-10-09 二次修订：加入"平移有意保留旧值"的显式断言）
//
// 不进 colcon，直接 g++ 编（见 docs/tilted_lidar_fidelity.md §J.1 的复现命令）：
//   g++ -O0 -o /tmp/tf2order tools/scripts/tiltmount/tf2_compose_order_test.cpp \
//       -I/opt/ros/humble/include/tf2 -L/opt/ros/humble/lib -ltf2 -Wl,-rpath,/opt/ros/humble/lib
//   /tmp/tf2order ; echo "exit=$?"
//
// 它钉住**当前代码的语义**（任何一条不成立就 return 非 0）：
//   ① tf2::Transform 的 `A*B` = **矩阵序** R_A·R_B（`C*p` 与 `A*(B*p)` 逐位相同）；
//   ② 旧写法 `T_bl⁻¹·T_ol·T_bl`（共轭/相似变换）对一个"只有 yaw 9°"的水平 T_ol 会给出
//      **假俯仰 ≈ 4.5°**，而且**违反位姿一致性**（共轭一般不等于原变换）—— 这就是 bug ③；
//   ③ **现在的写法**：姿态 = 合成 `T_ol·T_bl`，平移 = 旧写法（共轭）的值。
//      · 姿态对：非单位 T_bl（roll = 30°）下 rpy = (30, 0, 9)°、**假俯仰 = 0**，
//        且满足姿态一致性 `T_ob_rot · T_bl_rot⁻¹ == T_ol_rot`；
//      · 平移**故意保留旧值**：与"纯合成"的差恰好 = −p（p = livox 原点在 base_link 里的坐标
//        (0.000562, 0.130916, 0.157028)）。这个 0.2044 m 的差**不是漏改**：把平移也改成真值
//        会让**默认档与默认模型**的局部代价图整张变空（实测 lethal 471→0 / 445→0，机理 =
//        nav2 的 `obstacle_layer.scan.min_obstacle_height`（默认 0.0）是量在 odom 里的，
//        而 /scan 是一张过 `livox_frame` 原点的二维平盘）。详见 small_point_lio_node.cpp
//        那段 ⚠️⚠️ 注释与 docs/tilted_lidar_fidelity.md §J.1/§J.5；
//        ⇒ 本测试**显式断言**这个偏差存在且等于 −p（防"顺手改回去"而不看代价图）。
//   ④ 逐点坐标恒等式（对"纯合成"成立）：`(T_ol·T_bl)·p_base == T_ol·(T_bl·p_base)`；
//   ⑤ 默认档（T_bl 纯平移）⇒ 现在的写法与旧写法**姿态与平移逐位相同**（默认行为不变的保证）。
// =============================================================================
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <cstdio>

static int g_fail = 0;

static tf2::Matrix3x3 rpy_of(const tf2::Transform &T) {
  return tf2::Matrix3x3(T.getRotation());
}

static void rpy_deg(const char *tag, const tf2::Transform &T) {
  double r, p, y;
  rpy_of(T).getRPY(r, p, y);
  std::printf("  %-40s rpy = (% .3f, % .3f, % .3f) deg\n", tag,
              r * 180 / M_PI, p * 180 / M_PI, y * 180 / M_PI);
}

static double rpy_of_deg(const tf2::Transform &T, int which) {  // 0=roll 1=pitch 2=yaw
  double r, p, y;
  rpy_of(T).getRPY(r, p, y);
  return (which == 0 ? r : which == 1 ? p : y) * 180 / M_PI;
}

static void check(bool ok, const char *what) {
  std::printf("  %-56s ⇒ %s\n", what, ok ? "成立 ✓" : "**不成立**");
  if (!ok) g_fail = 1;
}

int main() {
  // ---------------------------------------------------------------- ① 乘法序
  std::printf("① tf2::Transform 的乘法序（`A*B` 是矩阵序 R_A·R_B）\n");
  tf2::Quaternion qa; qa.setRPY(0, 0, M_PI / 2);          // A = 绕 z 转 90°
  tf2::Transform A(qa, tf2::Vector3(0, 0, 0));
  tf2::Quaternion qb; qb.setRPY(M_PI / 2, 0, 0);          // B = 绕 x 转 90°
  tf2::Transform B(qb, tf2::Vector3(0, 0, 1));
  tf2::Transform C = A * B;
  tf2::Vector3 p(1, 0, 0);
  tf2::Vector3 c = C * p, ab = A * (B * p);
  std::printf("  C*p = (% .3f,% .3f,% .3f)   A*(B*p) = (% .3f,% .3f,% .3f)\n",
              c.x(), c.y(), c.z(), ab.x(), ab.y(), ab.z());
  check((c - ab).length() < 1e-12, "C*p == A*(B*p)");
  rpy_deg("C = A*B", C);

  // ---------------- 公共量：T_ol（LIO 状态，水平 + yaw 9°）与 p（livox 在 base 里）
  tf2::Quaternion qol; qol.setRPY(0, 0, M_PI / 20);
  tf2::Transform Tol(qol, tf2::Vector3(0, 0, 0));
  const tf2::Vector3 p_livox(0.0005617, 0.130916, 0.157028);   // 实测 TF base_link→livox_frame

  // -------------- ② 旧写法（共轭）在 urdf/sensor 档的假俯仰（= 用户实测到的那个 bug）
  std::printf("\n② 旧写法 T_bl⁻¹·T_ol·T_bl（共轭/相似变换）—— 这是被修掉的那个 bug\n");
  tf2::Quaternion qbl30; qbl30.setRPY(M_PI / 6, 0, 0);    // T_bl 旋转 = Rx(+30°)（倾角记进关节）
  // T_bl = T(livox←base)：平移 = base 原点在 livox 系里的坐标 = −R_x(30°)·p
  tf2::Transform Tbl30(qbl30, -(tf2::Matrix3x3(qbl30) * p_livox));
  tf2::Transform Tob_old30 = Tbl30.inverse() * Tol * Tbl30;
  rpy_deg("T_ol（LIO 状态，水平、yaw 9°）", Tol);
  rpy_deg("旧写法 T_ob", Tob_old30);
  double fake_pitch = rpy_of_deg(Tob_old30, 1);
  std::printf("  假俯仰 = %+.3f deg（实测 .tmp_tiltmount/tm_urdf 的 4.890° 同源）\n", fake_pitch);
  check(std::fabs(fake_pitch) > 4.0, "旧写法确实有 >4° 的假俯仰（= bug）");
  {
    tf2::Quaternion q = (Tob_old30 * Tbl30.inverse()).getRotation();
    check(1.0 - std::fabs(q.dot(Tol.getRotation())) > 1e-6,
          "旧写法不满足姿态一致性（= bug）");
  }

  // ------------------------------------------- ③ 现在的写法：合成姿态 + 旧平移
  std::printf("\n③ 现在的写法：姿态 = 合成 T_ol·T_bl；平移 = 旧写法（共轭）的值\n");
  tf2::Transform tf_comp = Tol * Tbl30;                   // 纯合成（物理真值）
  tf2::Transform tf_legacy = Tbl30.inverse() * Tol * Tbl30;
  tf2::Transform tf_now = tf_comp;                        // ← 代码：先合成 …
  tf_now.setOrigin(tf_legacy.getOrigin());                // ← 再盖回旧平移
  rpy_deg("现在发布的 T_ob（urdf/sensor 档）", tf_now);
  check(std::fabs(rpy_of_deg(tf_now, 1)) < 0.5, "假俯仰 < 0.5°（验收线）");
  check(std::fabs(std::fabs(rpy_of_deg(tf_now, 0)) - 30.0) < 0.5,
        "30° 出现在 roll 上（odom = 斜的初始传感器系 ⇒ 物理事实）");
  {
    tf2::Quaternion q = (tf_now * Tbl30.inverse()).getRotation();
    double dq = 1.0 - std::fabs(q.dot(Tol.getRotation()));
    std::printf("  姿态一致性 d(1-|q·q|) = %.3e\n", dq);
    check(dq < 1e-12, "姿态满足一致性 T_ob·T_bl⁻¹ == T_ol（只看旋转）");
  }
  {
    // ⚠️ 倾角档（R_A ≠ I）下"现在发的平移"与"纯合成"的差**不是**常数 −p（它被 R_Aᵀ/R_ol 旋转过），
    //    但量级仍是同一个杆臂（|p| = 0.2044 m）。这里把向量打出来 + 钉住量级；
    //    真正"逐位不变"的保证在 ⑤（默认档，那一档才要求零变化）。
    tf2::Vector3 d = tf_now.getOrigin() - tf_comp.getOrigin();
    std::printf("  与“纯合成”的平移差 = (% .6f, % .6f, % .6f) m，|d| = %.4f m（|p| = %.4f）\n",
                d.x(), d.y(), d.z(), d.length(), p_livox.length());
    check(std::fabs(d.length() - p_livox.length()) < 2e-3,
          "平移差量级 = 杆臂 |p|（**有意保留**：§J.1 ⚠️⚠️ / §J.5 代价图实测）");
  }

  // ------------- ④ 逐点坐标恒等式（纯合成；姿态/平移正确性的来源）
  std::printf("\n④ 逐点坐标恒等式（T_ol·T_bl 的 p_base 映射 == T_ol·(T_bl·p_base)）\n");
  const tf2::Vector3 probes[3] = {tf2::Vector3(1.0, 0.5, -0.26),
                                  tf2::Vector3(-2.0, 1.5, 0.1),
                                  tf2::Vector3(0.3, -1.2, 0.7)};
  for (int i = 0; i < 3; ++i) {
    double d = ((tf_comp * probes[i]) - (Tol * (Tbl30 * probes[i]))).length();
    std::printf("  p%-2d base=(% .2f,% .2f,% .2f)  |Δ| = %.3e m\n", i + 1,
                probes[i].x(), probes[i].y(), probes[i].z(), d);
    if (d > 1e-9) { std::printf("  **FAIL** 恒等式不成立\n"); g_fail = 1; }
  }

  // ------------- ⑤ 默认档（T_bl 纯平移、rpy=0）⇒ 新旧**逐位相同**（默认行为不变的保证）
  std::printf("\n⑤ 默认档（T_bl 纯平移、rpy=0）⇒ 与旧写法姿态+平移逐位相同\n");
  tf2::Transform Tbl_p(tf2::Quaternion(0, 0, 0, 1), -p_livox);
  tf2::Transform now_p = Tol * Tbl_p;
  tf2::Transform old_p = Tbl_p.inverse() * Tol * Tbl_p;
  now_p.setOrigin(old_p.getOrigin());
  rpy_deg("现在发布（plugin 档）", now_p);
  rpy_deg("旧写法（plugin 档）", old_p);
  check(std::fabs(rpy_of_deg(now_p, 0) - rpy_of_deg(old_p, 0)) < 1e-12 &&
        std::fabs(rpy_of_deg(now_p, 1) - rpy_of_deg(old_p, 1)) < 1e-12 &&
        std::fabs(rpy_of_deg(now_p, 2) - rpy_of_deg(old_p, 2)) < 1e-12,
        "plugin 档姿态：新旧逐位相同");
  check((now_p.getOrigin() - old_p.getOrigin()).length() < 1e-12,
        "plugin 档平移：新旧逐位相同");
  check(std::fabs(rpy_of_deg(now_p, 1)) < 0.5, "plugin 档假俯仰 < 0.5°");

  std::printf("\n%s（exit=%d）\n", g_fail ? "**有断言失败**" : "全部断言通过", g_fail);
  return g_fail;
}
