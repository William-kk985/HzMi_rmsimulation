// =============================================================================
// slope_speed_limit.hpp —— 「前瞻坡度/台阶连续量 ⇒ 速度上限」（header-only，无 ROS）
// -----------------------------------------------------------------------------
// 要解决的问题（实测，见 docs/path_clearance_and_contact.md §2.2 / §7.5）：
//   车以 v ≈ 1.7 m/s 冲上 map(−3.6,3.1) 的**可行驶**坡脚 ⇒ IMU |a| **79~90 m/s²**（≈8~9 g）
//   ⇒ 小点云 LIO 里程计发散（逐帧 0.326 m、跑飞 2021 m）⇒ 定位失效 ⇒ 导航崩。
//   感知修好之后（判据命中的点进 obstacle）第一次撞击 86.4 → 68.2 m/s²，**没有消除**——
//   因为坡面本身**可行驶**，把它标成障碍是错的（docs/traversability_plan.md §1.3 ①）。
//   唯一能治这一下的手段是**在到达之前把速度降到安全值**。
//
// 物理模型（一个测点标定，两处都只用一个公式）：
//   刚性轮在坡脚/台阶沿处，垂向速度在接触建立的 Δt 内从 0 变成 v·tanθ（θ = 前方局部坡度）
//       a_peak ≈ v · tanθ / Δt
//   实测锚点：(v, tanθ, a_peak) = (1.70 m/s, 0.10, 86 m/s²)
//       ⇒ Δt_eff = v·tanθ/a_peak = 1.70 × 0.10 / 86 = **1.98 ms**
//   于是"把峰值压到 a_target 以内"要求
//       v ≤ a_target · Δt_eff / tanθ = a_target · 1.7 · 0.10 / (86 · tanθ)
//   本文件把它写成**分段线性表**（表在 traversability_criteria.yaml 里，是单一真源）：
//     坡度表（deg → m/s）：线性插值，最后一段落到 floor_mps
//     台阶表（m   → m/s）：台阶先折成"格内等效坡度" tanθ_eff = step_m / 格边长（判据自己的 0.20 m 粗格）
//   两张表的每一个结点都由 `describe()` / 校验脚本反算 a_peak，必须 ≤ peak_target（除地板值）。
//
// 为什么还要"刹车距离界"（本文件的核心，也是"提前减速"的实现）：
//   只按"特征当前多远"给上限，等于**在特征上才减速**（车已经带着 1.7 m/s 到坡脚了）。
//   所以最终上限取所有前瞻特征上的
//       v_cap(d) = sqrt( v_req(feature)² + 2 · brake_mps2 · d )
//   ——"到 d 处必须已经降到 v_req"的运动学反解。特征越近，允许的当前速度越低；
//   特征在 1.6 m 以外时这个界高于 vx_max ⇒ **不限速**（所以平地上不花时间）。
//   brake_mps2 是**规划用**减速度预算（不是物理极限）：物理极限是 velocity_smoother 的
//   max_decel = 4.0 m/s²（10× 余量）。取 **0.4** 的后果是"上限曲线在特征处正好压到 v_req"：
//   上限曲线 v(d) = sqrt(v_req² + 2·brake·d) 就是一条"以 brake 减速就能刚好在 d=0 处降到 v_req"
//   的曲线；而雷达**看得见**坡脚的最远距离是有限的（下视 −7.22°、离地 0.226 m ⇒ 平地最近
//   只看得见 0.226/tan7.22° = 1.78 m；上坡面会提前进入视野，约 1.9~2.6 m），
//   ⇒ brake 越小、同样距离下允许的速度越低、越"提前减速"。离线扫掠（bench/slope_speed_offline.cpp）：
//       brake 0.8 ⇒ 过坡脚 1.18 m/s；0.5 ⇒ 0.90；**0.4 ⇒ 0.70**；0.3 ⇒ 0.65
//   0.4 是"够低"与"别把时间都花在爬"之间的折中（详见 docs/slope_speed_limiting.md §2）。
//
// 契约（与 docs/traversability_plan.md §9、docs/ground_segmentation_slots.md §10 一致）：
//   · 本文件**不产生 /cmd_vel**、不停车、不取消目标、不做 watchdog：它只输出一个"速度上限"，
//     由节点发成 `nav2_msgs/msg/SpeedLimit`，nav2 自己（controller_server → MPPI
//     `setSpeedLimit()`）把它变成 MPPI 的 vx_max/vy/wz 约束缩放 —— **命令链上没有新节点**。
//   · 判据（什么算可行驶）不在这里：可行驶坡度上限 drivable_slope_deg、台阶阈值
//     step_height_threshold 都来自 low_terrain_classifier.hpp 的 Criteria（同一个 YAML）。
// =============================================================================
#ifndef RM_GROUND_TRAVERSABILITY__SLOPE_SPEED_LIMIT_HPP_
#define RM_GROUND_TRAVERSABILITY__SLOPE_SPEED_LIMIT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "rm_ground_traversability/low_terrain_classifier.hpp"

namespace rm_ground_traversability
{

/// 限速参数（键名 = traversability_criteria.yaml 的 `speed_limit_*`；单一真源）。
struct SpeedLimitCriteria
{
  bool enable{true};
  /// 有效最高速（m/s）= min(MPPI `vx_max`, velocity_smoother `max_velocity[0]`)。
  /// 只用于：① "不限速"档的判据；② 刹车距离界的起点（最坏情况下车正以它行驶）。
  /// ⚠ 它**不是**第二个真源里的数：nav2 那两个键才是；本值是它们的 min，
  ///   由 tools/scripts/regress/check_slope_speed_table.py 断言（改 nav2 参数要同步改这里）。
  double vx_max{2.0};
  /// 是否按**运动方向**决定走廊朝向（订阅 `direction_topic` 的速度符号；见 ROS 层）。
  /// ⚠ **默认 false**：这条路径实测**没有触发**（2026-10-07 整栈跑里明明在倒车、日志仍打"前进"，
  ///   根因未定位）⇒ 默认走 `rear_lookahead_m` 的**双向窗口**（不依赖任何话题语义）。
  ///   真要再用符号判方向：`speed_limit_direction_aware: true`，并注意看启动后有没有
  ///   "拿不到 /odom" 的 WARN。
  /// 为什么必须：MID360 装在车顶，走廊只朝一个方向看；实测两次整栈跑里**最重的一击都发生在
  /// 倒车时**（MPPI 的 Omni 模型会倒着走；OFF 110.8 m/s² / ON 67.7 m/s² 都在 v<0 时），
  /// 而只朝前看的走廊完全看不见车尾后面那个坡脚。开：按符号双向；关：只朝前（旧行为）。
  bool direction_aware{false};
  double rear_lookahead_m{1.2};       ///< 反方向窗口（m）——默认的"倒车也能看见车尾坡脚"修法，见 CorridorQuery
  double direction_min_vx{0.05};      ///< |vx| ≤ 它视为静止（保持上一次方向，避免抖动）
  double direction_timeout_s{0.5};    ///< 速度话题超过它就当没有 ⇒ 回退"只朝前"
  double lookahead_m{3.0};            ///< 前瞻距离（m）
  double corridor_half_width_m{0.28}; ///< 走廊半宽（m）
  double corridor_spread_deg{8.0};    ///< 走廊张角（deg）
  double slope_baseline_m{0.40};      ///< 坡度描述子基线（m，= 2 个粗格；与 slopeOf() 一致）
  double slope_change_deadband_deg{5.5};  ///< 坡度变化死区（deg）：噪声/正常起伏不触发
  // ★ 2026-10-07：4.0 → 5.5（与 traversability_criteria.yaml 逐键一致，见该文件注释与
  //   docs/lio_divergence_no_impact.md §3：实测 why=slope_change 占限速行 56~79%）
  double step_deadband_m{0.060};      ///< 台阶残差死区（m）：噪声不触发（★2026-10-07：0.040→0.060）
  double step_ignore_above_m{0.350};  ///< 残差 > 它 ⇒ 认成墙/高台（交给规划器绕），本限速器不管
  /// **来自判据**（`Criteria::drivable_slope_deg`，同一个 YAML 键 `drivable_slope_deg`，
  /// 不是第二个真源；ROS 层在构造时从 Criteria 填进来）：
  /// 走廊格的局部地面坡度 > 它 ⇒ 那是陡面/墙（判据自己会把它标成 obstacle）
  /// ⇒ 本限速器**不为它减速**，否则"贴着墙走"会一路爬行（代价大、而且撞墙不是限速能解决的）。
  double drivable_slope_deg{25.0};
  double brake_mps2{0.4};             ///< 刹车距离界用的规划减速度（m/s²）
  double release_mps2{1.0};           ///< 放开（升速）速率上限（m/s²）：降是立即的，升要慢
  /// 已承诺特征的剩余距离前推系数：每帧按 `limit × 本系数 × dt` 扣减。
  /// 1.0 = 直接拿"上一帧上限"当速度估计（仿真里 planar_move 完美跟踪 ⇒ 就是真实行进距离）。
  /// **< 1 = 假设车走得比上限慢**（前推更慢 ⇒ 承诺保持更久 ⇒ 更保守）；> 1 = 更激进。
  /// 实测（离线扫掠 .tmp_slopespeed/approach*.cpp）：1.0 + "承诺用格子远边" ⇒ 上限在特征处正好
  /// 降到 v_req（过坡脚速度 = v_req ±0.1）；0.7 会让它晚 0.4 m 才降到位（反而不保守）。
  double hold_decay_factor{1.0};
  double floor_mps{0.60};             ///< 速度地板（m/s）：最低也就降到这（不是 0 ⇒ 永不停车）
  // ★ 2026-10-07：0.45 → 0.60（配合 peak_target 45；推导与实测见 YAML 同键注释）
  /// **支撑格数**：一个"定位型"特征必须在同一 0.40 m 距离带内被 ≥ 它个格确认才算数。
  /// 为什么需要：真实单帧走廊只有 7~14 个粗格、且集中在一两个距离带里（点云稀疏），
  /// 单格的坡度/台阶估计噪声很大（实测 maxΔ坡 在 0.6°~34° 之间跳）⇒ 不加支撑要求时
  /// 噪声会反复触发限速（实测整栈跑：38% 的运动时间被压在 0.6 m/s 以下、目标点超时）。
  int min_support_cells{2};
  std::vector<double> slope_knots_deg{2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0};
  // ★ 2026-10-07：target 30 → 45 后按 v = round_down_2(45·Δt_eff/tanθ × 0.975) 重算
  std::vector<double> slope_knots_mps{2.48, 1.65, 1.24, 0.99, 0.82, 0.61, 0.60};
  /// 台阶表：模型解在死区(0.04 m ⇒ 等效坡度 11.3°)就已经 ≤0.35 m/s ⇒ 有意义的只有"到地板"这一档。
  std::vector<double> step_knots_m{0.060};   // ★ 2026-10-07：死区 0.04 → 0.06 同步
  std::vector<double> step_knots_mps{0.60};
  // 锚点（只用于自检/文档：反算 Δt_eff 与每个结点的预期峰值）
  double anchor_v_mps{1.70};
  double anchor_tan_slope{0.10};
  double anchor_peak_mps2{86.0};
  double peak_target_mps2{45.0};      // ★ 2026-10-07：30 → 45（回退见 YAML 同键注释）

  /// 标定出来的等效接触时间（s）：Δt_eff = v_anchor·tanθ_anchor/a_anchor = 1.98 ms。
  double dt_eff_s() const
  {
    return anchor_v_mps * anchor_tan_slope / std::max(anchor_peak_mps2, 1e-9);
  }

  /// 模型预期峰值（m/s²）：a = v·tanθ/Δt_eff。
  double predicted_peak(double v_mps, double tan_slope) const
  {
    return v_mps * tan_slope / std::max(dt_eff_s(), 1e-9);
  }

  /// 台阶 → 等效坡度正切（判据自己的 0.20 m 粗格内完成这个抬升）。
  double step_tan_equivalent(double step_m) const
  {
    return step_m / std::max(slope_baseline_m * 0.5, 1e-6);   // 0.40/2 = 0.20 m = 粗格
  }

  /// 分段线性表的取值（knots 必须已排序；越界按端点夹取）。
  static double interp(const std::vector<double> & knots_x, const std::vector<double> & knots_y,
                       double x);

  double slope_limit(double slope_deg) const
  {
    if (slope_deg < slope_knots_deg.front()) {
      return vx_max;   // 第一结点以下 = 不限速
    }
    return std::min(vx_max, interp(slope_knots_deg, slope_knots_mps, slope_deg));
  }

  double step_limit(double step_m) const
  {
    if (step_m < step_knots_m.front()) {
      return vx_max;   // 死区以下 = 不限速
    }
    return std::min(vx_max, interp(step_knots_m, step_knots_mps, step_m));
  }

  /// 参数不合法的**原因**（空串 = 合法）。节点构造期调用 ⇒ "参数写错 = 限速静默失效"不可能发生。
  std::string reason_invalid() const
  {
    std::ostringstream os;
    if (vx_max <= 0.0) {
      os << "speed_limit_vx_max 必须 > 0；";
    }
    if (lookahead_m <= 0.0) {
      os << "speed_limit_lookahead_m 必须 > 0；";
    }
    if (rear_lookahead_m < 0.0) {
      os << "speed_limit_rear_lookahead_m 必须 ≥ 0；";
    }
    if (direction_min_vx < 0.0 || direction_timeout_s <= 0.0) {
      os << "speed_limit_direction_min_vx 必须 ≥ 0、speed_limit_direction_timeout_s 必须 > 0；";
    }
    if (corridor_half_width_m < 0.0 || corridor_spread_deg < 0.0 || corridor_spread_deg >= 90.0) {
      os << "走廊参数非法（half_width ≥ 0 且 0 ≤ spread < 90）；";
    }
    if (step_deadband_m < 0.0 || step_ignore_above_m <= step_deadband_m) {
      os << "台阶死区/忽略阈值非法（0 ≤ deadband < ignore_above）；";
    }
    if (slope_change_deadband_deg < 0.0 || slope_change_deadband_deg >= 90.0) {
      os << "speed_limit_slope_change_deadband_deg 必须在 [0, 90)；";
    }
    if (brake_mps2 <= 0.0 || release_mps2 < 0.0) {
      os << "brake/release 速率非法（brake > 0，release ≥ 0）；";
    }
    if (hold_decay_factor <= 0.0 || hold_decay_factor > 1.0) {
      os << "speed_limit_hold_decay_factor 必须在 (0, 1]；";
    }
    if (min_support_cells < 1) {
      os << "speed_limit_min_support_cells 必须 ≥ 1；";
    }
    if (floor_mps <= 0.0) {
      os << "speed_limit_floor_mps 必须 > 0（**不许为 0**：本机制不停车）；";
    }
    if (floor_mps > vx_max) {
      os << "speed_limit_floor_mps 不应大于 speed_limit_vx_max；";
    }
    if (slope_knots_deg.size() != slope_knots_mps.size() || slope_knots_deg.empty()) {
      os << "坡度表两个数组长度必须相同且非空；";
    }
    if (step_knots_m.size() != step_knots_mps.size() || step_knots_m.empty()) {
      os << "台阶表两个数组长度必须相同且非空；";
    }
    for (std::size_t i = 1; i < slope_knots_deg.size(); ++i) {
      if (slope_knots_deg[i] <= slope_knots_deg[i - 1] ||
        slope_knots_mps[i] > slope_knots_mps[i - 1])
      {
        os << "坡度表必须按坡度递增、速度单调不增（第 " << i << " 项）；";
        break;
      }
    }
    for (std::size_t i = 1; i < step_knots_m.size(); ++i) {
      if (step_knots_m[i] <= step_knots_m[i - 1] || step_knots_mps[i] > step_knots_mps[i - 1]) {
        os << "台阶表必须按高度递增、速度单调不增（第 " << i << " 项）；";
        break;
      }
    }
    // 物理自检：每个结点反算的预期峰值 ≤ peak_target，**或**该结点已到地板值
    // （地板是"机构上还愿意走"的最低速，不是模型解 —— 到地板就承认模型解不出来，由实测兜）。
    for (std::size_t i = 0; i < slope_knots_deg.size(); ++i) {
      const double v = slope_knots_mps[i];
      if (v <= floor_mps + 1e-9) {
        continue;
      }
      const double peak = predicted_peak(v, std::tan(slope_knots_deg[i] * kPi / 180.0));
      if (peak > peak_target_mps2) {
        os << "坡度表第 " << i << " 结点 (" << slope_knots_deg[i] << " deg → " << v
           << " m/s) 的预期峰值 " << peak << " m/s² > " << peak_target_mps2
           << "（要么降速、要么让它等于 floor）；";
      }
    }
    for (std::size_t i = 0; i < step_knots_m.size(); ++i) {
      const double v = step_knots_mps[i];
      if (v <= floor_mps + 1e-9) {
        continue;
      }
      const double peak = predicted_peak(v, step_tan_equivalent(step_knots_m[i]));
      if (peak > peak_target_mps2) {
        os << "台阶表第 " << i << " 结点 (" << step_knots_m[i] << " m → " << v
           << " m/s) 的预期峰值 " << peak << " m/s² > " << peak_target_mps2 << "；";
      }
    }
    return os.str();
  }

  std::string describe() const
  {
    std::ostringstream os;
    os << "enable=" << (enable ? "true" : "false")
       << " vx_max=" << vx_max << " m/s"
       << " lookahead=" << lookahead_m << " m (rear " << rear_lookahead_m << " m)"
       << " direction_aware=" << (direction_aware ? "true" : "false")
       << " (min|vx|=" << direction_min_vx << " m/s, timeout=" << direction_timeout_s << " s)"
       << " corridor=±" << corridor_half_width_m << " m/" << corridor_spread_deg << " deg"
       << " brake=" << brake_mps2 << " m/s²"
       << " release=" << release_mps2 << " m/s²"
       << " hold_decay=" << hold_decay_factor
       << " floor=" << floor_mps << " m/s"
       << " slope_change_deadband=" << slope_change_deadband_deg << " deg"
       << " step_deadband=" << step_deadband_m << " m"
       << " step_ignore_above=" << step_ignore_above_m << " m"
       << " Δt_eff=" << (dt_eff_s() * 1e3) << " ms"
       << " (anchor " << anchor_v_mps << " m/s @tanθ=" << anchor_tan_slope << " ⇒ "
       << anchor_peak_mps2 << " m/s²; target ≤ " << peak_target_mps2 << " m/s²)";
    return os.str();
  }
};

/// 「谁在限、限多少、为什么」—— 诊断/日志/JSON 都用这一个结构（免得三处各说各话）。
struct SpeedLimitDecision
{
  double limit_mps{0.0};        ///< 最终上限（m/s；= vx_max 表示不限速 ⇒ 发 NO_SPEED_LIMIT=0.0）
  double raw_mps{0.0};          ///< 未做"放开速率限制"前的本帧上限（m/s）
  bool limited{false};          ///< false = 不限速
  const char * reason{"none"};  ///< "slope_change" / "slope" / "step" / "none"
  double feature_d{0.0};        ///< 触发特征的距离（m）
  double feature_slope_deg{0.0};///< 触发格局部坡度（deg）
  double feature_dtan_deg{0.0}; ///< 触发格"坡度变化"（deg）
  double feature_step_m{0.0};   ///< 触发格台阶残差（m）
  double v_req_mps{0.0};        ///< 该特征的"到了就得降到"速度（m/s）
  bool from_brake_bound{false}; ///< true = 被刹车距离界（而不是特征本身）压住 ⇒ 说明在"提前减速"
  bool from_hold{false};        ///< true = 本帧上限来自"已承诺特征"的前推（不是本帧测量）
  double hold_d{0.0};           ///< 已承诺特征估计还剩多远（m）；-1 = 无承诺
  std::size_t cells{0};         ///< 走廊内可用格数
  double max_slope_deg{0.0};
  double max_dtan_deg{0.0};
  double max_step_m{0.0};
  double near_slope_deg{0.0};
  double data_min_d{-1.0};
  double data_max_d{-1.0};
  bool profile_valid{false};
};

/// 限速器：把一帧的走廊剖面 + 上一帧的上限 ⇒ 本帧上限（纯函数式，可离线单测）。
///
/// 三条规则（取最紧的一个），共用**同一张**速度表（slope_limit / step_limit）：
///   ① **坡度变化**（主力）：atan|Δtanθ|（相对最近 0.40 m 的参考坡度）≥ 死区 ⇒ 限速。
///      撞击物理来自**坡度变化**（a ≈ v·Δtanθ/Δt）而不是坡度本身 ⇒ 恒定坡面上不限速。
///   ② **台阶残差**（窄尺度）：格内抬升 − 该格坡度能解释的部分 ∈ [死区, 忽略上限] ⇒ 限速。
///   ③ **绝对坡度**（兜底）：局部坡度本身 ≥ 第一结点 ⇒ 限速。**为什么需要它**：雷达下视
///      −7.22°（离地 0.226 m）⇒ 平地最近只看得见 1.78 m，坡脚可能正好落在盲区里
///      （此时走廊里**全是坡面**、坡度变化≈0）—— 没有兜底规则就完全不会减速。
///
/// **已承诺特征（hold）+ 距离前推**：这一条是"把速度真的压到特征上"的关键。
///   只用"到特征还有 d 米 ⇒ 现在允许 sqrt(v_req²+2a·d)"的话，因为**看不见车下与近处地面**，
///   测量到的特征距离永远 ≥1.0~1.8 m ⇒ 上限永远降不到 v_req（实测会以 ~1.4 m/s 过坡脚，
///   只把 86 m/s² 降到 ~70，不够）。所以：一旦某帧测到"比当前承诺更紧"的特征，就把它**承诺**下来
///   （记住 v_req 与剩余距离），此后每帧把剩余距离按"上一帧上限 × dt"前推（拿上限当速度是
///   **保守**的：上限 ≤ 实际速度；只有刚开始减速那一两帧不成立），前推到 0（车已到/过了它）才释放。
///   ⇒ 车确实会以 v_req 到达特征，然后按 release_mps2 缓慢放开。
class SlopeSpeedLimiter
{
public:
  explicit SlopeSpeedLimiter(const SpeedLimitCriteria & criteria) : criteria_(criteria) {}

  const SpeedLimitCriteria & criteria() const {return criteria_;}
  void setCriteria(const SpeedLimitCriteria & c) {criteria_ = c;}
  void reset() {have_prev_ = false; prev_ = 0.0; hold_ = Hold();}

  /// @param profile 本帧前瞻剖面（describeCorridor 的输出）
  /// @param dt_s    距上一帧的秒数（≤0 或 >2 s ⇒ 不做前推/放开速率限制）
  SpeedLimitDecision update(const CorridorProfile & profile, double dt_s)
  {
    SpeedLimitDecision d;
    d.cells = profile.cells.size();
    d.max_slope_deg = profile.max_slope_deg;
    d.max_dtan_deg = profile.max_dtan_deg;
    d.max_step_m = profile.max_step_m;
    d.near_slope_deg = profile.near_slope_deg;
    d.data_min_d = profile.data_min_d;
    d.data_max_d = profile.data_max_d;
    d.profile_valid = profile.valid;

    const double dt = (dt_s > 0.0 && dt_s <= 2.0) ? dt_s : 0.0;
    for (int i = 0; i < kMaxBands; ++i) {
      support_[i].dtan = 0;
      support_[i].step = 0;
    }

    // ---- ① 已承诺特征的剩余距离前推（保守：用上一帧发布的上限当速度）----
    if (hold_.valid) {
      hold_.d -= prev_ * criteria_.hold_decay_factor * dt;
      if (hold_.d <= 0.0) {
        hold_.valid = false;
      }
    }

    // ---- ② 本帧测量：三条规则取最紧；其中只有**定位型**两条允许"承诺" ----
    //   绝对坡度那条**故意不承诺**：一片连续坡面的测量距离每帧都会刷新成 ~1.5 m，
    //   一旦让它承诺，就会"承诺→前推→过期→又被刷新"⇒ 锯齿，且在坡上无意义地长期爬行。
    //   而坡度变化/台阶残差是**固定位置**的跃变（测量距离随车前进真的变小）⇒ 承诺它是对的。
    //   ★ 定位型特征还要过"支撑格数"这一关：同一 0.40 m 距离带内 ≥ min_support_cells 个格
    //     都给出该特征才算数（真实点云稀疏、单格估计噪声大；见 min_support_cells 注释）。
    Candidate meas;        // 全部规则（决定本帧上限）
    Candidate meas_loc;    // 只有定位型（决定承诺）
    const double band = std::max(0.2, criteria_.slope_baseline_m);
    for (const CorridorCell & c : profile.cells) {
      const bool steep_face = c.slope_deg > criteria_.drivable_slope_deg;   // 陡面/墙 ⇒ 规划器的事
      const bool dtan_hit = !steep_face && c.dtan_deg >= criteria_.slope_change_deadband_deg;
      const bool step_hit = c.step_m >= criteria_.step_deadband_m &&
        c.step_m <= criteria_.step_ignore_above_m;
      if (dtan_hit) {
        const int b = static_cast<int>(c.d / band);
        ++support_[b < 0 ? 0 : (b < kMaxBands ? b : kMaxBands - 1)].dtan;
      }
      if (step_hit) {
        // 台阶残差很大（≥2× 死区）时单格也算数：真正的台阶沿本来就可能只占一格。
        const bool strong = c.step_m >= 2.0 * criteria_.step_deadband_m;
        const int b = static_cast<int>(c.d / band);
        if (strong) {
          ++support_[b < 0 ? 0 : (b < kMaxBands ? b : kMaxBands - 1)].step;
        }
      }
      if (!steep_face && !hold_.valid && c.slope_deg >= criteria_.slope_knots_deg.front()) {
        consider(meas, c, criteria_.slope_limit(c.slope_deg), "slope", false);
      }
    }
    // 支撑判定（第二遍：现在知道每个带里有多少格给出特征了）
    for (const CorridorCell & c : profile.cells) {
      const bool steep_face = c.slope_deg > criteria_.drivable_slope_deg;
      const int b = static_cast<int>(c.d / band);
      const int bi = b < 0 ? 0 : (b < kMaxBands ? b : kMaxBands - 1);
      const bool dtan_ok = !steep_face &&
        c.dtan_deg >= criteria_.slope_change_deadband_deg &&
        support_[bi].dtan >= criteria_.min_support_cells;
      const bool step_ok = c.step_m >= criteria_.step_deadband_m &&
        c.step_m <= criteria_.step_ignore_above_m &&
        (support_[bi].step >= criteria_.min_support_cells ||
        c.step_m >= 2.0 * criteria_.step_deadband_m);
      if (dtan_ok) {
        consider(meas, c, criteria_.slope_limit(c.dtan_deg), "slope_change", true);
        consider(meas_loc, c, criteria_.slope_limit(c.dtan_deg), "slope_change", true);
      }
      if (step_ok) {
        consider(meas, c, criteria_.step_limit(c.step_m), "step", true);
        consider(meas_loc, c, criteria_.step_limit(c.step_m), "step", true);
      }
    }

    // ---- ③ 承诺更新：只有"测量比已承诺更紧"时才换成测量（新鲜距离）----
    //   反过来：一片连续坡面（测量项每帧刷新、距离永远 ~1.5 m）不会反复把承诺拉回远处
    //   ⇒ 不会出现"承诺→前推→释放→又被刷新"的锯齿振荡。
    const double cap_hold = hold_.valid ?
      std::sqrt(hold_.v_req * hold_.v_req + 2.0 * criteria_.brake_mps2 * hold_.d) :
      criteria_.vx_max;
    if (meas_loc.valid && (!hold_.valid || meas_loc.cap < cap_hold - 0.02)) {
      // 承诺距离取该格的**远边**（d + 格边长）：判据说"这一格里出现了坡度变化/台阶"，
      // 跃变的真实位置在这一格内 ⇒ 用远边是保守估计（宁可多留一点距离，不要少留）。
      const double d_commit = meas_loc.d + meas_loc.cell_span_m;
      hold_ = Hold{true, d_commit, meas_loc.v_req, meas_loc.slope_deg, meas_loc.dtan_deg,
        meas_loc.step_m, meas_loc.reason};
    }
    const double cap_hold2 = hold_.valid ?
      std::sqrt(hold_.v_req * hold_.v_req + 2.0 * criteria_.brake_mps2 * hold_.d) :
      criteria_.vx_max;
    bool from_hold = false;
    Candidate use;
    if (hold_.valid && (!meas.valid || cap_hold2 <= meas.cap)) {
      use.valid = true;
      use.committable = true;
      use.cap = cap_hold2;
      use.d = hold_.d;
      use.v_req = hold_.v_req;
      use.slope_deg = hold_.slope_deg;
      use.dtan_deg = hold_.dtan_deg;
      use.step_m = hold_.step_m;
      use.reason = hold_.reason;
      from_hold = true;
    } else if (meas.valid) {
      use = meas;
    }

    double best = use.valid ? use.cap : criteria_.vx_max;
    best = std::max(best, criteria_.floor_mps);
    best = std::min(best, criteria_.vx_max);

    // ---- ④ 放开速率限制：降 = 立即（安全）；升 ≤ release_mps2（防抖、也防"过了坡就全油门"）----
    double out = best;
    if (have_prev_ && dt > 0.0 && best > prev_) {
      out = std::min(best, prev_ + criteria_.release_mps2 * dt);
    }
    const bool limited = use.valid && out < criteria_.vx_max - 1e-9;
    d.raw_mps = best;
    d.limit_mps = out;
    d.limited = limited;
    d.reason = (!use.valid || !limited) ? "none" : use.reason;
    d.feature_d = use.d;
    d.feature_slope_deg = use.slope_deg;
    d.feature_dtan_deg = use.dtan_deg;
    d.feature_step_m = use.step_m;
    d.v_req_mps = use.valid ? use.v_req : criteria_.vx_max;
    d.from_brake_bound = use.valid && use.cap > use.v_req + 1e-6;
    d.from_hold = from_hold;
    d.hold_d = hold_.valid ? hold_.d : -1.0;
    prev_ = out;
    have_prev_ = true;
    return d;
  }

private:
  /// 一个候选（本帧测量项，或已承诺项）。
  struct Candidate
  {
    bool valid{false};
    bool committable{false};  ///< true = **定位型**特征（坡度变化/台阶残差）⇒ 允许"承诺 + 前推"
    double cap{0.0};          ///< 现在允许的速度上限（m/s）
    double cell_span_m{0.0};  ///< 特征所在格边长（m）：承诺时用格子远边
    double d{0.0};            ///< 特征距离（m，格近边）
    double v_req{0.0};        ///< 到特征时该有的速度（m/s）
    double slope_deg{0.0};
    double dtan_deg{0.0};
    double step_m{0.0};
    const char * reason{"none"};
  };

  /// 已承诺的特征（v_req + 估计剩余距离）。
  struct Hold
  {
    bool valid{false};
    double d{0.0};
    double v_req{0.0};
    double slope_deg{0.0};
    double dtan_deg{0.0};
    double step_m{0.0};
    const char * reason{"none"};
  };

  /// 把"某格要求到 d 处降到 v_req"折成"现在允许多快"，并保留更紧的那个。
  void consider(
    Candidate & best, const CorridorCell & c, double v_req, const char * reason,
    bool committable) const
  {
    if (v_req >= criteria_.vx_max) {
      return;
    }
    const double cap = std::sqrt(v_req * v_req + 2.0 * criteria_.brake_mps2 * c.d);
    // cap ≥ vx_max ⇒ 这个特征现在**还不构成约束**（够远）⇒ 既不进"本帧上限"，也不该被承诺
    // （否则会把一个"当前根本不影响速度"的特征按它的近距离承诺下来，过一会儿莫名其妙把速度拉低）。
    if (cap >= criteria_.vx_max) {
      return;
    }
    if (best.valid && cap >= best.cap) {
      return;
    }
    best.valid = true;
    best.committable = committable;
    best.cell_span_m = c.cell_span_m;
    best.cap = cap;
    best.d = c.d;
    best.v_req = v_req;
    best.slope_deg = c.slope_deg;
    best.dtan_deg = c.dtan_deg;
    best.step_m = c.step_m;
    best.reason = reason;
  }

  /// 每个距离带里"给出定位型特征"的格数（支撑判定用）。带宽 = slope_baseline_m。
  static constexpr int kMaxBands = 16;
  struct Support { int dtan{0}; int step{0}; };
  Support support_[kMaxBands];

  SpeedLimitCriteria criteria_;
  double prev_{0.0};
  bool have_prev_{false};
  Hold hold_;
};

/// 分段线性插值（knots_x 升序；x 在两端外按端点夹取）。
inline double SpeedLimitCriteria::interp(
  const std::vector<double> & knots_x, const std::vector<double> & knots_y, double x)
{
  if (knots_x.empty() || knots_y.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (x <= knots_x.front()) {
    return knots_y.front();
  }
  for (std::size_t i = 1; i < knots_x.size(); ++i) {
    if (x <= knots_x[i]) {
      const double x0 = knots_x[i - 1], x1 = knots_x[i];
      const double y0 = knots_y[i - 1], y1 = knots_y[i];
      if (x1 - x0 <= 1e-12) {
        return y1;
      }
      const double t = (x - x0) / (x1 - x0);
      return y0 + t * (y1 - y0);
    }
  }
  return knots_y.back();
}

}  // namespace rm_ground_traversability

#endif  // RM_GROUND_TRAVERSABILITY__SLOPE_SPEED_LIMIT_HPP_
