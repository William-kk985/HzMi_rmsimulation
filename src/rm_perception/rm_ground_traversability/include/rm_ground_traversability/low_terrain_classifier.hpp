// =============================================================================
// low_terrain_classifier.hpp —— 地面分割之后的「坡度 / 台阶」判定（header-only）
// -----------------------------------------------------------------------------
// 为什么需要它（缺陷 ③，见 docs/low_terrain_representation.md / docs/path_clearance_and_contact.md）：
//   linefit（`max_slope ±0.4` ≈ 21.8°、`max_dist_to_line 0.05`）会把 ≤22° 的坡面和
//   0.06~0.30 m 的低矮台面**整片判成地面删掉**；patchwork 也有自己的漏判（35~60° 陡面
//   被它判成地面）。于是 `/segmentation/obstacle` 里没有这些几何 ⇒ `/scan` 没有 ⇒
//   代价图没有 ⇒ **规划器从"看不见的坡脚 / 台阶"上开过去**（实测 IMU 79~90 m/s²）。
//   而离线出先验图的 tools/scripts/mapping/pcd_to_nav2_map.py **本来就有一套坡度/台阶判据**
//   ⇒ 实时链路与先验图口径不一致，正是"图上看不见、车撞上去"的来源。
//
// 本文件把 pcd_to_nav2_map.py 的判据搬到**实时**链路里，判据与阈值完全一致：
//
//     局部地面 g(x,y) = 该点所在 0.20 m 粗格内、各 0.05 m 细格"最低点"的 p05
//                       （= "我脚下这块地大概多高"；细格用最低点 ⇒ 竖直墙面格子不会把自己抬高）
//     可行驶(drivable) ⟺ z − g ≤ step_height_threshold
//                        且 不是（局部地面坡度 > drivable_slope_deg 且 z − g > slope_min_height）
//     台阶/边沿(step_edge) ⟺ 判据命中（高度台阶 或 超限坡面）
//
//   · **只降不升**：本类只会把"上一级分割器判成 ground"的点**降级**为 obstacle，
//     永远不会把 obstacle 升级成 ground ⇒ 不会删掉任何已有障碍（安全性单调）。
//   · **可行驶的斜坡必须保持 free**：判据是"相对**局部**地面的高差"，坡面上每个点自己
//     就是局部地面 ⇒ 24° 的坡（0.20 m 粗格内只升 0.09 m）不会被误判。这一点是硬要求，
//     因为本车能爬 23°（见 docs/traversability_plan.md §1.1 表 A 行）。
//   · **完备性**：输入点云的每个点最终恰好落在 ground 或 obstacle 之一（与两个分割器节点的
//     现有契约一致）；step_edge 是 obstacle 的**子集**（诊断用，单独发一个话题，不参与契约）。
//
// 阈值真源（**单一真源**，不许在这里写第二份）：
//   src/rm_nav_bringup/config/traversability_criteria.yaml
//     · launch 把它当**参数文件**喂给两个地面分割节点 ⇒ 本类的 Criteria 由 ROS 参数填入；
//     · tools/scripts/mapping/pcd_to_nav2_map.py 读**同一份文件**当 argparse 默认值。
//   Criteria 结构体里的字面值只是"没拿到参数文件时的兜底"，必须与那份 YAML 逐键相同；
//   一致性由 tools/scripts/regress/check_traversability_criteria.py 断言。
// =============================================================================
#ifndef RM_GROUND_TRAVERSABILITY__LOW_TERRAIN_CLASSIFIER_HPP_
#define RM_GROUND_TRAVERSABILITY__LOW_TERRAIN_CLASSIFIER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rm_ground_traversability
{

/// π（不用 M_PI：它在 -std=c++17 且未定义 _USE_MATH_DEFINES 时不保证可见）。
constexpr double kPi = 3.14159265358979323846;

/// 判据参数（键名与 traversability_criteria.yaml / pcd_to_nav2_map.py 的 CLI 一一对应）。
struct Criteria
{
  bool enable{true};                          ///< false = 完全旁路（回退到"没有本判据"的行为）
  double step_height_threshold{0.15};         ///< == --height-threshold
  double drivable_slope_deg{25.0};            ///< == --slope-limit
  double slope_min_height{0.05};              ///< == --slope-min-height
  double ground_cell_m{0.20};                 ///< == --ground-cell（估局部地面的粗格）
  double fine_cell_m{0.05};                   ///< == --resolution（细格）
  double ground_percentile{5.0};              ///< == --ground-percentile
  int ground_min_points{2};                   ///< == --ground-min-points

  /// ★★ 2026-10-09：**近地剔除**（`obstacle_near_ground_m`，米；**0.0 = 关，默认**）。
  ///
  /// 语义：本判据算出的「离**局部地面**的高度」`dz = z − g(x,y)` **≤ 它**的点，
  /// 由调用方（`ground_segmentation_node.cc` 的 `applyFrame()` 返回值）从
  /// `/segmentation/obstacle` **剔除**（改判成 ground）—— 它**不是** `self_mask`（自击掩膜）：
  ///   · self_mask = "机器人自己的 collision 几何 / 近场死区"里的点（几何固定、与地面无关）；
  ///   · 本键      = "确实贴着地面"的点（判据用**局部地面**判，与车体几何无关）。
  ///
  /// 为什么必须有这一级（robot11 实测，docs/tilted_lidar_fidelity.md §K）：
  ///   `/scan` 是一张**二维平盘**（`pointcloud_to_laserscan` → LaserScan 没有高度）。nav2 的
  ///   `obstacle_layer` 用 `projectLaser`（z 强行置 0）再 `transformLaserScanToPointCloud` 搬到
  ///   代价图帧 ⇒ 每条波束在那个帧里的 z **恒等于"那一帧传感器原点的 z"**，与它实际打到的
  ///   三维点**无关**（实测：948 条波束的 odom z 全落在 [0.020, 0.089]、min_obstacle_height
  ///   0.0 一条都不丢）。⇒ **2D 那条链路根本没有"逐点高度"可判**，`min_obstacle_height`
  ///   在那里只是一条"传感器自身高度"的常量闸。
  ///   后果：linefit 漏判的近场地面点（水平 0.30~0.50 m，实测 100% 落在真值地面 ±0.023 m 内）
  ///   被当成障碍、在雷达高度上投成 lethal 格 ⇒ 车心到最近 lethal **0.39 m**、车半径圆内
  ///   `≥99` 440/1000 格、free 仅 26 格 ⇒ P2P 控制器判 `collision ahead` ⇒ **目标被接受但车不走**。
  ///   唯一能在**投影之前**用上"高度"的地方就是这里（本判据本来就有 dz）。
  ///
  /// 取值（robot11 槽位 = 0.05 m，见 config/traversability_near_ground_robot11.yaml）：
  ///   · 实测（同一帧 raw 云，`livox_frame`，真值地面 = −0.259527 m）：
  ///     水平 0.30–0.40 m 的 341 点离地 p05/p50/p95 = 0.0159/0.0183/0.0228 m；
  ///     0.40–0.50 m 的 693 点 = 0.0088/0.0122/0.0153 m ⇒ `dz ≤ 0.05` **把它们全剔掉**（100%）；
  ///   · 真障碍（场地低矮件）在 0.50–1.00 m 的 p95 = 0.169 m ⇒ 只剔掉贴地那一层（~19% 的点）；
  ///   · 本判据的台阶闸 `step_height_threshold` = 0.15 m ⇒ 任何会被判成"台阶/边沿"的点
  ///     `dz > 0.15`，**结构上不可能**被 0.05 剔掉 ⇒ "0.2/0.35 m 台阶仍是障碍"是构造性的。
  ///   · 保守方向：**只少标障碍**（可能漏一个 5 cm 以下的矮物），不会多标 ⇒ 不会凭空断路。
  double near_ground_m{0.0};

  /// 粗格/细格比（≥1）。0.20/0.05 = 4 ⇒ 每个粗格最多看 4×4 个细格。
  int cell_ratio() const
  {
    const double r = ground_cell_m / std::max(fine_cell_m, 1e-6);
    return std::max(1, static_cast<int>(std::lround(r)));
  }

  bool valid() const
  {
    return fine_cell_m > 0.0 && ground_cell_m >= fine_cell_m &&
           step_height_threshold > 0.0 && drivable_slope_deg > 0.0 &&
           drivable_slope_deg < 90.0 && slope_min_height >= 0.0 &&
           near_ground_m >= 0.0;
  }

  /// 参数不合法的**原因**（空串 = 合法）。节点在构造期调用它并直接报错退出，
  /// 避免"参数写错 ⇒ 判据静默不生效"这种最难查的故障。
  std::string reason_invalid() const
  {
    std::ostringstream os;
    if (fine_cell_m <= 0.0) {
      os << "fine_cell_m 必须 > 0；";
    }
    if (ground_cell_m < fine_cell_m) {
      os << "ground_cell_m 必须 >= fine_cell_m；";
    }
    const double ratio = ground_cell_m / std::max(fine_cell_m, 1e-9);
    if (std::abs(ratio - std::round(ratio)) > 1e-6) {
      os << "ground_cell_m 必须是 fine_cell_m 的整数倍（当前比 " << ratio << "）；";
    }
    if (step_height_threshold <= 0.0) {
      os << "step_height_threshold 必须 > 0；";
    }
    if (drivable_slope_deg <= 0.0 || drivable_slope_deg >= 90.0) {
      os << "drivable_slope_deg 必须在 (0, 90)；";
    }
    if (slope_min_height < 0.0) {
      os << "slope_min_height 必须 >= 0；";
    }
    if (ground_min_points < 1) {
      os << "ground_min_points 必须 >= 1；";
    }
    return os.str();
  }

  std::string describe() const
  {
    std::ostringstream os;
    os << "enable=" << (enable ? "true" : "false")
       << " step_height_threshold=" << step_height_threshold << " m"
       << " drivable_slope_deg=" << drivable_slope_deg
       << " slope_min_height=" << slope_min_height << " m"
       << " ground_cell=" << ground_cell_m << " m"
       << " fine_cell=" << fine_cell_m << " m"
       << " ground_percentile=p" << ground_percentile
       << " ground_min_points=" << ground_min_points
       << " near_ground=" << (near_ground_m > 0.0
        ? (std::to_string(near_ground_m) + " m（贴地即不算障碍）") : std::string("关（默认）"));
    return os.str();
  }

  /// ★ 2026-10-09：**近地剔除**是否启用（`near_ground_m > 0`）。调用方（节点）用它决定
  /// 要不要把 `LowTerrainClassifier::apply()` 填好的 `near_ground_flags` 变成"改判 ground"。
  bool nearGroundEnabled() const {return near_ground_m > 0.0;}
};

/// **自击掩膜**（self-hit mask）：把"近场自身回波"的点从**建格**里剔掉。
///
/// 为什么需要（robot:=robot11 实测，docs/robot_models.md §12）：本模型雷达装在底盘凹槽里，
/// 360° 视场里最近的部件是**云台** l10/l11。Gazebo 实测 **26.9~28.1% 的点落在 r ≤ 0.09 m**
/// （不是 0.116~0.139 —— 见下面"实测更正"）。这些近场点落在前瞻走廊的粗格 d=0 里 ⇒
/// 把"格内最高点 − 局部地面"抬到 **0.076~0.095 m** ⇒ 限速器判成台阶（> `step_deadband` 0.06）
/// ⇒ **车停着也被压到速度表地板 0.60 m/s**。
///
/// 掩膜由**两部分**组成（并集；默认两部分都关）：
///
/// **(a) 自身 collision 包络** `boxes`：本模型 12 个 link 的 `<collision>` 在传感器系下的 AABB。
///   Gazebo 的 ray sensor 走 ODE、只与 `<collision>` 求交 ⇒ 落在 collision 表面上的回波必在盒内。
///
/// **(b) 近场死区** `radius_m` + `z_min_m`：`r_xy ≤ radius_m 且 z ≥ z_min_m` 的点整片剔掉。
///   为什么 (a) 不够（Phase 4 实测更正）：本槽位近场那 27% 的点**不在任何 collision 几何上**
///   （逐点验：113 个盒只覆盖其中 3 个）。它们是**射线插件把点重建成 `range·axis`**
///   造成的系统内移：射线实际从 `minDist·axis`（=0.1 m）出发，而点按 `range·axis` 发布
///   ⇒ **每个点都朝传感器方向内移 0.1 m**。独立证据（与掩膜无关）：地面点的高度随距离单调变化
///   （r 0.25–0.35 → z −0.208；r 2–4 → z −0.253；几何真值 −0.2595），正是"沿射线内移 0.1 m"
///   应有的样子。修那个插件会改变**所有模型**的点云（默认模型也移了 0.1 m）⇒ 不属本主题、
///   且违反"默认逐字节不变"，所以这里只在**判据层**把这团近场剔掉。
///   `radius_m` 的上界是**几何硬约束**：本槽位雷达下俯 30°、离地 0.2595 m
///   ⇒ 最低那条射线的地面交点在 **0.3416 m**，即 **r < 0.3416 m 内不可能有地面回波**；
///   取 `radius_m = 0.3416 − 0.10 = 0.2416`（留 10 cm 余量）。
///   离线/在线双重验证：地面带（z < −0.20）被掩 **0 点**；近场那团点在 r ≤ 0.09 内、之后到
///   0.25 m 是**空的**（实测直方图 0.09–0.15 / 0.15–0.20 / 0.20–0.25 三档都是 0 点）。
///
/// 代价：`r_xy ≤ 0.2416 m` 或落在自身 collision 体内的**真障碍**会被一起掩掉 ——
/// 该区域整个在车体足印（外接半径 0.3565 m）之内 ⇒ 对静止障碍等价于"已经撞上了"。
///
/// 默认 `enable=false` ⇒ `buildGrid()` 完全不走这段代码（默认模型/其它槽位逐字节行为不变）。
struct SelfMask
{
  bool enable{false};
  /// 扁平数组，每个盒子 6 个数：xmin,ymin,zmin,xmax,ymax,zmax（**传感器系**，米）。
  std::vector<double> boxes;
  /// 近场死区半径（m，传感器系 XY 平面）。≤0 = 关闭这一部分。
  double radius_m{0.0};
  /// 近场死区的 z 下限（m，传感器系）。默认 −∞ ⇒ 不设 z 门（生成器会给"地面以上 clearance"）。
  double z_min_m{-std::numeric_limits<double>::infinity()};

  /// 参数不合法的**原因**（空串 = 合法）。节点在构造期调用 ⇒ "参数写错 = 掩膜静默不生效" 不可能。
  std::string reason_invalid() const
  {
    std::ostringstream os;
    if (boxes.size() % 6 != 0) {
      os << "self_mask_boxes 的长度必须是 6 的整数倍（每组 xmin,ymin,zmin,xmax,ymax,zmax），当前 "
         << boxes.size() << "；";
    }
    for (std::size_t i = 0; i + 5 < boxes.size(); i += 6) {
      if (boxes[i] > boxes[i + 3] || boxes[i + 1] > boxes[i + 4] || boxes[i + 2] > boxes[i + 5]) {
        os << "第 " << (i / 6) << " 个盒子的 min > max；";
        break;
      }
    }
    if (radius_m < 0.0) {
      os << "self_mask_radius_m 必须 >= 0；";
    }
    if (enable && boxes.empty() && radius_m <= 0.0) {
      os << "self_mask_enable=true 但 self_mask_boxes 与 self_mask_radius_m 都是空的/0"
            "（掩膜会静默不生效）；";
    }
    return os.str();
  }

  std::size_t n_boxes() const {return boxes.size() / 6;}

  std::string describe() const
  {
    std::ostringstream os;
    if (!enable) {
      os << "关（默认；建格不看自击点）";
      return os.str();
    }
    os << "开：(a) " << n_boxes() << " 个自身 collision AABB（传感器系）";
    if (has_global_) {
      os << "，联合 AABB=[" << global_lo_[0] << "," << global_lo_[1] << "," << global_lo_[2]
         << " .. " << global_hi_[0] << "," << global_hi_[1] << "," << global_hi_[2] << "]";
    }
    os << " + (b) 近场死区 r_xy<=" << radius_m << " m";
    if (std::isfinite(z_min_m)) {
      os << " 且 z>=" << z_min_m << " m";
    }
    return os.str();
  }

  /// 预计算"联合 AABB"（每点先过这一关 ⇒ 99.9% 的点一次比较就被排除，逐盒循环几乎不跑）。
  void build()
  {
    has_global_ = false;
    if (boxes.size() < 6) {
      return;
    }
    for (int k = 0; k < 3; ++k) {
      global_lo_[k] = std::numeric_limits<double>::infinity();
      global_hi_[k] = -std::numeric_limits<double>::infinity();
    }
    for (std::size_t i = 0; i + 5 < boxes.size(); i += 6) {
      for (int k = 0; k < 3; ++k) {
        global_lo_[k] = std::min(global_lo_[k], boxes[i + k]);
        global_hi_[k] = std::max(global_hi_[k], boxes[i + 3 + k]);
      }
    }
    has_global_ = true;
  }

  /// 点是否落在掩膜里（传感器系，米）。先过 (b) 近场死区（一次比较），再过 (a) 的联合 AABB。
  bool contains(double x, double y, double z) const
  {
    if (!enable) {
      return false;
    }
    // (b) 近场死区：r_xy ≤ radius_m 且 z ≥ z_min_m。默认 radius_m = 0 ⇒ 这一段恒假。
    if (radius_m > 0.0 && z >= z_min_m && (x * x + y * y) <= radius_m * radius_m) {
      return true;
    }
    // (a) 自身 collision 包络。
    if (!has_global_) {
      return false;
    }
    if (x < global_lo_[0] || x > global_hi_[0] || y < global_lo_[1] || y > global_hi_[1] ||
      z < global_lo_[2] || z > global_hi_[2])
    {
      return false;
    }
    for (std::size_t i = 0; i + 5 < boxes.size(); i += 6) {
      if (x >= boxes[i] && x <= boxes[i + 3] &&
        y >= boxes[i + 1] && y <= boxes[i + 4] &&
        z >= boxes[i + 2] && z <= boxes[i + 5])
      {
        return true;
      }
    }
    return false;
  }

private:
  double global_lo_[3]{0.0, 0.0, 0.0};
  double global_hi_[3]{0.0, 0.0, 0.0};
  bool has_global_{false};
};

/// 前瞻走廊查询参数（**只描述"往哪看、看多远"，不含任何速度映射** —— 速度那一步在
/// slope_speed_limit.hpp 里，见该文件头注）。
///
/// 为什么是"朝向走廊"而不是"贴着规划路径"：本类跑在**传感器帧**里（livox 安装 `rpy 0 0 0`
/// ⇒ 帧的 +x 就是车头方向），而地面分割节点**不订阅 /plan、不查 TF**（那是它零故障面的前提）。
/// 要用规划路径做前瞻就得把点云/路径互转到同一帧 ⇒ 给感知节点加一条 TF + 话题依赖。
/// 代价/收益：路径前瞻只比朝向走廊"省"掉转向时走廊扫过的不相关几何（表现为偶尔多限一点速度），
/// 而朝向走廊零依赖、零时序问题 ⇒ 选朝向走廊，并把走廊半宽 + 张角做成参数（转弯时靠张角覆盖）。
struct CorridorQuery
{
  bool reverse{false};            ///< true = 车正在**倒车** ⇒ 主窗口朝 −x（见 speed_limit_direction_topic）
  double lookahead_m{3.0};        ///< 主窗口（运动方向）前瞻距离（m）
  /// **反方向窗口**（m）。>0 时同时看运动方向的背面 —— 这是"倒车撞车尾后面的坡脚"这个坑的**默认修法**：
  ///   只朝一个方向看的走廊会完全漏掉背面的几何，而"运动方向"这个信息本身要么拿不到（话题没起）、
  ///   要么语义要看里程计约定（实测用 /odom 的 `twist.linear.x` 符号**没有**触发，见 docs §5/§7）。
  ///   取 1.2 m（比正向短）：倒车速度实测 ~1.0~1.4 m/s，需要的预警距离更短；同时把"过了特征还要慢多久"
  ///   限制在 1.2 m 以内（时间代价可控）。
  double rear_lookahead_m{1.2};
  double half_width_m{0.28};      ///< 走廊固定半宽（m）：车体半宽 0.155 + 余量
  double spread_deg{8.0};         ///< 走廊随距离张开的半角（deg）：覆盖转弯时的扫掠
  double slope_baseline_m{0.40};  ///< 坡度描述子的基线（m）= 2 个粗格（只用于文档/自检）
};

/// 一个走廊格（= 一个 0.20 m 粗格）的**连续量**（这就是"坡度/台阶连续量"的载体）。
struct CorridorCell
{
  double x{0.0};          ///< 传感器系 x（前向，m）
  double y{0.0};          ///< 传感器系 y（m）
  double d{0.0};          ///< 到车体前缘的距离（沿运动方向；取格子**近边**，保守，m）
  bool ahead{true};       ///< true = 在运动方向那一侧（false = 反方向窗口里的格）
  double ground_z{0.0};   ///< 该格局部地面高度（m）
  double slope_deg{0.0};  ///< 该格局部地面坡度（deg，复用 slopeOf() ⇒ 与判据同源）
  double dtan_deg{0.0};   ///< **坡度变化**（deg）：相对"脚下附近"参考坡度的等效角，atan|Δtanθ|
  double step_m{0.0};     ///< **台阶残差**（m）：格内最高点 − 局部地面 − 该格坡度能解释的抬升
  double cell_span_m{0.0};///< 该格边长（m），用于把残差折算成等效坡度
};

/// 一帧的前瞻走廊剖面（连续量的集合 + 极值）。
struct CorridorProfile
{
  std::vector<CorridorCell> cells;
  std::size_t cells_total{0};      ///< 落在走廊几何内的粗格数（含无地面的）
  double max_slope_deg{0.0};       ///< 走廊内最大局部坡度（deg）
  double max_slope_d{0.0};         ///< 它到车体的距离（m）
  double max_dtan_deg{0.0};        ///< 走廊内最大**坡度变化**（deg）
  double max_dtan_d{0.0};          ///< 它到车体的距离（m）
  double max_step_m{0.0};          ///< 走廊内最大台阶残差（m）
  double max_step_d{0.0};          ///< 它到车体的距离（m）
  double near_slope_deg{0.0};      ///< "脚下附近"参考坡度（deg）= 最近 0.40 m 内各格坡度的均值
  double data_min_d{-1.0};         ///< 有数据的最近/最远距离（m）；-1 = 没有数据
  double data_max_d{-1.0};
  bool valid{false};               ///< 至少有一个可用格
};

/// 单帧统计（节点把它打进日志 / 供离线核对）。
struct FrameStats
{
  std::size_t points{0};
  std::size_t ground_in{0};        ///< 上一级分割器判 ground 的点数
  std::size_t ground_out{0};       ///< 本判据之后仍是 ground 的点数
  std::size_t obstacle_in{0};
  std::size_t obstacle_out{0};
  std::size_t step_edge{0};        ///< 判据命中的点数（高度台阶 ∪ 超限坡面）
  std::size_t step_height{0};      ///< 其中因"高差 > step_height_threshold"命中
  std::size_t steep_face{0};       ///< 其中因"坡度 > drivable_slope_deg 且高差 > slope_min_height"命中
  std::size_t demoted_ground{0};   ///< 被本判据从 ground 降级成 obstacle 的点数
  std::size_t coarse_cells{0};     ///< 有点的粗格数
  std::size_t coarse_cells_no_ground{0};
  double classify_ms{0.0};
  /// ★2026-10-07 Phase 4：被**自击掩膜**剔出建格的点数（掩膜关时恒 0）。
  std::size_t self_masked{0};
  /// ★2026-10-09：被判成「贴地」（`dz ≤ near_ground_m`）的点数（该键关时恒 0）。
  ///   注意它**不影响**本类的建格/判据/限速，只是"给调用方一个把贴地点改判 ground 的掩码"。
  std::size_t near_ground{0};
  // ---- 前瞻走廊（describeCorridor() 填；不在 apply() 里算，见节点调用顺序） ----
  std::size_t corridor_cells{0};           ///< 走廊内可用粗格数
  double corridor_max_slope_deg{0.0};      ///< 走廊内最大局部坡度
  double corridor_max_step_m{0.0};         ///< 走廊内最大台阶残差
};

/// 坡度/台阶判定器（无 ROS、无 I/O；每帧调用 apply()，内部容器复用 ⇒ 无每帧分配尖峰）。
///
/// 坐标系要求：**z 轴向上**（重力对齐）。两个地面分割节点在 `gravity_aligned_frame` 为空时
/// 直接用原始点云帧 —— 本仓 livox 安装 `rpy = 0 0 0` ⇒ 已对齐（同 patchwork 节点注释）。
class LowTerrainClassifier
{
public:
  explicit LowTerrainClassifier(const Criteria & criteria) : criteria_(criteria) {}

  const Criteria & criteria() const {return criteria_;}
  void setCriteria(const Criteria & c) {criteria_ = c;}
  const FrameStats & stats() const {return stats_;}

  /// ★2026-10-07 Phase 4：装/卸**自击掩膜**（默认关 ⇒ 不在场时建格行为逐字节不变）。
  void setSelfMask(const SelfMask & m)
  {
    self_mask_ = m;
    self_mask_.build();
  }
  const SelfMask & selfMask() const {return self_mask_;}
  /// 本帧被掩膜剔掉的点数（诊断用；掩膜关时恒 0）。
  std::size_t selfMaskedPoints() const {return self_masked_;}

  /// 主入口。
  /// @param cloud          输入点云（帧与 labels 同序）
  /// @param ground_flags   in/out：1 = ground（可行驶），0 = obstacle。**只会被降级，不会被升级**
  /// @param step_edge_flags out：1 = 本判据命中的点（台阶/边沿 ∪ 超限坡面）；可为 nullptr
  /// @param near_ground_flags ★2026-10-09 out：1 = `dz = z − 局部地面 ≤ near_ground_m` 的点
  ///   （**贴地点**）。`near_ground_m` 关（默认 0.0）时**恒 0、一个字节都不写** ⇒ 默认路径不变。
  ///   调用方拿它把"贴地点"从 obstacle 改判成 ground（`obstacle_near_ground_m` 的语义，
  ///   见 Criteria::near_ground_m 的头注）。**不改本类的建格/判据/限速任何一步。**
  void apply(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    std::vector<uint8_t> * ground_flags,
    std::vector<uint8_t> * step_edge_flags,
    std::vector<uint8_t> * near_ground_flags = nullptr)
  {
    stats_ = FrameStats();
    stats_.points = cloud.size();
    near_ground_ = 0;
    if (step_edge_flags != nullptr) {
      step_edge_flags->assign(cloud.size(), 0u);
    }
    if (near_ground_flags != nullptr) {
      near_ground_flags->assign(cloud.size(), 0u);
    }
    if (ground_flags != nullptr) {
      for (std::size_t i = 0; i < ground_flags->size(); ++i) {
        if ((*ground_flags)[i] != 0u) {
          ++stats_.ground_in;
        } else {
          ++stats_.obstacle_in;
        }
      }
    }
    if (!criteria_.enable || !criteria_.valid() || cloud.empty() || ground_flags == nullptr) {
      if (ground_flags != nullptr) {
        stats_.ground_out = stats_.ground_in;
        stats_.obstacle_out = stats_.points - stats_.ground_in;
      }
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    buildGrid(cloud);
    classify(cloud, ground_flags, step_edge_flags, near_ground_flags);
    const auto t1 = std::chrono::steady_clock::now();
    stats_.classify_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.self_masked = self_masked_;   // ★ Phase 4：掩膜剔掉的点数（关=0）
    stats_.near_ground = near_ground_;   // ★ 2026-10-09：贴地点数（关=0）
    stats_.coarse_cells = ground_cache_.size();
    stats_.coarse_cells_no_ground = 0;
    for (const auto & kv : ground_cache_) {
      if (!std::isfinite(kv.second.ground)) {
        ++stats_.coarse_cells_no_ground;
      }
    }
  }

  /// 前瞻走廊剖面（**连续量的出口**）：在 apply() 之后调用。
  ///
  /// 只做"测量 + 描述"，不做任何判断/限速（判据在 apply() 里，速度映射在 slope_speed_limit.hpp）
  /// ⇒ 三个消费者（代价图 / 诊断 / 限速）看的是**同一份**坡度与台阶数。
  ///
  /// 每个走廊粗格给三个连续量（前两个是"跳跃"，第三个是"绝对坡度"）：
  ///   · dtan_deg  = **坡度变化**（相对"脚下附近" 0.40 m 内的参考坡度，atan|Δtanθ|）
  ///     —— 撞击的物理量就来自**坡度变化**（a ≈ v·Δtanθ/Δt），而不是坡度本身：
  ///        在**恒定**坡面上行驶没有垂向速度突变 ⇒ 不该因为"我在坡上"而限速；
  ///        坡脚/坡顶/倒角起的那个格才是要减速的地方。
  ///   · step_m    = 格内最高点 − 局部地面 − 该格坡度在格内能解释的抬升（**残差**）
  ///     —— **窄尺度**：整格内完成的尖台阶（它的抬升不改变邻格地面 ⇒ 坡度描述子看不见）。
  ///     减去"坡度能解释的抬升"是为了去掉与 dtan 的重复计数：坡面上 max_rise ≈ 格边长·tanθ。
  ///   · slope_deg = 局部地面坡度的**绝对**值（坡道本体；诊断 + 限速器的兜底规则用）
  ///     —— 兜底场景：坡脚落在雷达盲区里（下视 −7.22° ⇒ 平地最近只看得见 1.78 m），
  ///        此时走廊里**全是坡面**、dtan≈0，只有"绝对坡度"能提示"我正在往坡上开"。
  ///
  /// 坐标系：与 apply() 同一朵云（传感器帧，+x = 车头）。d 取格子**近边**（保守）。
  void describeCorridor(const CorridorQuery & q, CorridorProfile * out)
  {
    if (out == nullptr) {
      return;
    }
    out->cells.clear();
    out->cells_total = 0;
    out->max_slope_deg = 0.0;
    out->max_dtan_deg = 0.0;
    out->max_step_m = 0.0;
    out->max_slope_d = 0.0;
    out->max_dtan_d = 0.0;
    out->max_step_d = 0.0;
    out->near_slope_deg = 0.0;
    out->data_min_d = -1.0;
    out->data_max_d = -1.0;
    out->valid = false;
    if (!criteria_.enable || !criteria_.valid()) {
      return;
    }
    const double cell = criteria_.ground_cell_m;
    const double half_w = std::max(0.0, q.half_width_m);
    const double tan_spread = std::tan(std::max(0.0, q.spread_deg) * kPi / 180.0);
    const double look = std::max(0.0, q.lookahead_m);
    const double rear = std::max(0.0, q.rear_lookahead_m);

    // ① 先挑出几何上落在走廊里的粗格（**不要**在遍历 unordered_map 时调 slopeOf：
    //    它会往 ground_cache_ 里插新键 ⇒ 迭代器失效）。
    keys_.clear();
    for (const auto & kv : ground_cache_) {
      if (!std::isfinite(kv.second.ground)) {
        continue;
      }
      int64_t cx = 0, cy = 0;
      decodeKey(kv.first, &cx, &cy);
      const double x = (static_cast<double>(cx) + 0.5) * cell;
      const double y = (static_cast<double>(cy) + 0.5) * cell;
      const double xs = q.reverse ? -x : x;   // 运动方向坐标（倒车 ⇒ 往 −x 看）
      const double xr = -xs;                  // 反方向坐标
      const bool ahead = xs > 0.0 && xs <= look + 0.5 * cell;
      const bool behind = rear > 0.0 && xr > 0.0 && xr <= rear + 0.5 * cell;
      if (!ahead && !behind) {
        continue;   // 两个窗口之外（+半格容差：格心稍远但格边在窗口内）
      }
      const double dist = ahead ? xs : xr;
      if (std::abs(y) > half_w + dist * tan_spread) {
        continue;   // 走廊外（转弯时走廊随距离张开）
      }
      ++out->cells_total;
      keys_.push_back(kv.first);
    }

    // ② 逐格取连续量（这里才可能插缓存，安全：不再迭代 ground_cache_）
    double dmin = std::numeric_limits<double>::infinity(), dmax = -1.0;
    for (const int64_t key : keys_) {
      int64_t cx = 0, cy = 0;
      decodeKey(key, &cx, &cy);
      const CoarseCell & c = cellOf(cx, cy);
      if (!std::isfinite(c.ground)) {
        continue;
      }
      CorridorCell cc;
      cc.x = (static_cast<double>(cx) + 0.5) * cell;
      cc.y = (static_cast<double>(cy) + 0.5) * cell;
      const double xs = q.reverse ? -cc.x : cc.x;
      cc.ahead = xs > 0.0;
      // d = 沿**该格所属窗口**的距离（运动方向那一侧 = x；反方向 = −x）
      cc.d = std::max(0.0, (cc.ahead ? xs : -xs) - 0.5 * cell);
      cc.ground_z = c.ground;
      cc.slope_deg = slopeOf(cx, cy);
      const double explained = cell * std::tan(cc.slope_deg * kPi / 180.0);
      cc.step_m = std::max(0.0, static_cast<double>(c.max_rise) - explained);
      cc.cell_span_m = cell;
      if (cc.slope_deg > out->max_slope_deg) {
        out->max_slope_deg = cc.slope_deg;
        out->max_slope_d = cc.d;
      }
      if (cc.step_m > out->max_step_m) {
        out->max_step_m = cc.step_m;
        out->max_step_d = cc.d;
      }
      if (cc.ahead) {
        dmin = std::min(dmin, cc.d);
      }
      dmax = std::max(dmax, cc.d);
      out->cells.push_back(cc);
    }
    if (!out->cells.empty()) {
      out->valid = true;
      out->data_min_d = dmin;
      out->data_max_d = dmax;
      // ③ "脚下附近"参考坡度：**最近那一列**格（d ≤ dmin + 半个粗格）的坡度均值。
      //    不可见的地面在车下（盲区）拿不到 ⇒ 用最近的可见格当代理（在坡面上两者同坡）。
      //    为什么不是"最近 0.40 m 内全部格"：坡脚附近那一窗口会**同时**含平格与坡格 ⇒
      //    参考坡度取成中间值 ⇒ 平格与坡格都出现 2~3° 的假"坡度变化"（离线扫掠里表现为
      //    对同一片坡反复承诺、限速值上下跳）。取"最近一列"则参考始终是"我正要开上去的那块"。
      const double ref_win = 0.5 * criteria_.ground_cell_m;
      double sum = 0.0;
      int n = 0;
      // 参考坡度**优先只取"运动方向那一侧"的最近格**：前后两个窗口混在一起算参考坡度，
      // 会让"前方坡脚 vs 后方坡面"这种组合产生假的坡度变化。
      for (int pass = 0; pass < 2 && n == 0; ++pass) {
        for (const CorridorCell & c : out->cells) {
          if ((pass == 0) != c.ahead) {
            continue;   // pass 0 只看 ahead；pass 1 只看反方向
          }
          if (c.d <= dmin + ref_win) {
            sum += c.slope_deg;
            ++n;
          }
        }
      }
      out->near_slope_deg = (n > 0) ? (sum / n) : 0.0;
      const double tan_near = std::tan(out->near_slope_deg * kPi / 180.0);
      for (CorridorCell & c : out->cells) {
        const double tan_c = std::tan(c.slope_deg * kPi / 180.0);
        c.dtan_deg = std::atan(std::abs(tan_c - tan_near)) * 180.0 / kPi;
        if (c.dtan_deg > out->max_dtan_deg) {
          out->max_dtan_deg = c.dtan_deg;
          out->max_dtan_d = c.d;
        }
      }
    }
    stats_.corridor_cells = out->cells.size();
    stats_.corridor_max_slope_deg = out->max_slope_deg;
    stats_.corridor_max_step_m = out->max_step_m;
  }

private:
  // ---- 自击掩膜的状态（见 SelfMask 的头注；默认关） -------------------------
  SelfMask self_mask_;
  std::size_t self_masked_{0};

  // ---- 细格 / 粗格 ---------------------------------------------------------
  struct FineCell
  {
    float lo1{std::numeric_limits<float>::infinity()};   ///< 最低点
    float lo2{std::numeric_limits<float>::infinity()};   ///< 次低点（只为观测；地面取 lo1）
    float hi{-std::numeric_limits<float>::infinity()};   ///< 最高点（"这一细格里最高能踩到多高"）
    int n{0};
  };

  /// 一个 0.20 m 粗格的两种连续量（**判据与前瞻共用同一份缓存**）：
  ///   · ground  = 局部地面高度（= 格内各细格"最低点"的 p05，与 pcd_to_nav2_map.py 同秩公式）
  ///   · max_rise= 格内最高点 − 局部地面（连续"台阶/抬升"量；判据只看它 > step_height_threshold）
  struct CoarseCell
  {
    float ground{std::numeric_limits<float>::infinity()};   ///< +inf = 没有可用地面
    /// 台阶/抬升的连续量（m）。取格内各细格"最高点"的 **p90**（不是全局 max）：
    /// 真实点云里总有个别离群点（远处稀疏、擦边、垂向拖影），用 max 会被一个点带偏
    /// （实测：单帧走廊内 max 台阶残差噪声可达 0.15 m ⇒ 台阶规则被噪声反复触发、一路爬行）。
    float max_rise{0.0f};
  };

  static int64_t keyOf(int64_t ix, int64_t iy)
  {
    // ix/iy 各自 32 位足够（±0.05 m × 2^31 ≈ ±1e8 m）；负数用低 32 位截断后拼装。
    return (static_cast<int64_t>(static_cast<int32_t>(ix)) << 32) |
           static_cast<int64_t>(static_cast<uint32_t>(static_cast<int32_t>(iy)));
  }

  /// keyOf() 的逆（describeCorridor 要从 key 还原格号）。
  static void decodeKey(int64_t key, int64_t * ix, int64_t * iy)
  {
    *ix = static_cast<int32_t>(static_cast<uint64_t>(key) >> 32);
    *iy = static_cast<int32_t>(key & 0xffffffffLL);
  }

  void buildGrid(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    fine_.clear();
    ground_cache_.clear();
    slope_cache_.clear();
    fine_.reserve(cloud.size() * 2);
    self_masked_ = 0;
    const double inv_fine = 1.0 / criteria_.fine_cell_m;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const float x = cloud[i].x, y = cloud[i].y, z = cloud[i].z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;   // NaN/inf：不参与建格；标签保持上一级分割器的判定
      }
      // ★ Phase 4：自击掩膜（默认关 ⇒ 这一行不改变任何行为）。
      //   被掩掉的点**不进任何细格** ⇒ 既不参与"局部地面高度"，也不参与"格内最高点 − 地面"，
      //   也不进前瞻走廊。**只影响判据/限速**：点云与 /segmentation/* 的标签完全不变
      //   （自击是物理真实的回波，真机 360° 雷达也会照到云台）。
      if (self_mask_.contains(x, y, z)) {
        ++self_masked_;
        continue;
      }
      const int64_t ix = static_cast<int64_t>(std::floor(x * inv_fine));
      const int64_t iy = static_cast<int64_t>(std::floor(y * inv_fine));
      FineCell & cell = fine_[keyOf(ix, iy)];
      if (z < cell.lo1) {
        cell.lo2 = cell.lo1;
        cell.lo1 = z;
      } else if (z < cell.lo2) {
        cell.lo2 = z;
      }
      if (z > cell.hi) {
        cell.hi = z;
      }
      ++cell.n;
    }
  }

  /// 该粗格的两种连续量；没有可用地面时 ground = +inf（缓存，重复调用零成本）。
  const CoarseCell & cellOf(int64_t cx, int64_t cy)
  {
    const int64_t key = keyOf(cx, cy);
    const auto it = ground_cache_.find(key);
    if (it != ground_cache_.end()) {
      return it->second;
    }
    const int k = std::min(criteria_.cell_ratio(), 8);   // 上限 8×8 = 64 次查表，防病态参数
    std::vector<float> lows;
    std::vector<float> his;
    lows.reserve(static_cast<std::size_t>(k * k));
    his.reserve(static_cast<std::size_t>(k * k));
    int points = 0;
    for (int a = 0; a < k; ++a) {
      for (int b = 0; b < k; ++b) {
        const auto fit = fine_.find(keyOf(cx * k + a, cy * k + b));
        if (fit == fine_.end()) {
          continue;
        }
        lows.push_back(fit->second.lo1);
        points += fit->second.n;
        if (std::isfinite(fit->second.hi)) {
          his.push_back(fit->second.hi);
        }
      }
    }
    CoarseCell out;
    if (!lows.empty() && points >= std::max(1, criteria_.ground_min_points)) {
      std::sort(lows.begin(), lows.end());
      // 与 pcd_to_nav2_map.py 的 _per_cell_percentile 同一条取秩公式：
      //   target = floor(pct/100 * (cnt-1))。小样本时它退化成最小值 —— 两边一致。
      const std::size_t cnt = lows.size();
      const std::size_t target = static_cast<std::size_t>(
        std::floor(criteria_.ground_percentile / 100.0 *
        static_cast<double>(cnt > 0 ? cnt - 1 : 0)));
      out.ground = lows[std::min(target, cnt - 1)];
      if (!his.empty()) {
        std::sort(his.begin(), his.end());
        // p90（与 p05 同一条取秩公式）：一个离群点抬不动它
        const std::size_t hn = his.size();
        const std::size_t htarget = static_cast<std::size_t>(
          std::floor(90.0 / 100.0 * static_cast<double>(hn > 0 ? hn - 1 : 0)));
        const float hi_p90 = his[std::min(htarget, hn - 1)];
        if (hi_p90 > out.ground) {
          out.max_rise = hi_p90 - out.ground;
        }
      }
    }
    return ground_cache_.emplace(key, out).first->second;
  }

  /// 该点的粗格地面高度（m）；无穷大 = 没有可用地面。
  float groundOf(int64_t cx, int64_t cy)
  {
    return cellOf(cx, cy).ground;
  }

  /// 粗格地面坡度（deg）。用中心差分；邻居没有地面时退化为单侧差分；都没有 ⇒ 0。
  float slopeOf(int64_t cx, int64_t cy)
  {
    const int64_t key = keyOf(cx, cy);
    const auto it = slope_cache_.find(key);
    if (it != slope_cache_.end()) {
      return it->second;
    }
    const double d = criteria_.ground_cell_m;
    const float g = groundOf(cx, cy);
    if (!std::isfinite(g)) {
      slope_cache_[key] = 0.0f;
      return 0.0f;
    }
    const float l = groundOf(cx - 1, cy);
    const float r = groundOf(cx + 1, cy);
    const float u = groundOf(cx, cy + 1);
    const float dn = groundOf(cx, cy - 1);
    double gx = 0.0, gy = 0.0;
    if (std::isfinite(l) && std::isfinite(r)) {
      gx = (static_cast<double>(r) - l) / (2.0 * d);
    } else if (std::isfinite(l)) {
      gx = (static_cast<double>(g) - l) / d;
    } else if (std::isfinite(r)) {
      gx = (static_cast<double>(r) - g) / d;
    }
    if (std::isfinite(dn) && std::isfinite(u)) {
      gy = (static_cast<double>(u) - dn) / (2.0 * d);
    } else if (std::isfinite(dn)) {
      gy = (static_cast<double>(g) - dn) / d;
    } else if (std::isfinite(u)) {
      gy = (static_cast<double>(u) - g) / d;
    }
    const float slope = static_cast<float>(
      std::atan(std::hypot(gx, gy)) * 180.0 / kPi);
    slope_cache_[key] = slope;
    return slope;
  }

  void classify(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    std::vector<uint8_t> * ground_flags,
    std::vector<uint8_t> * step_edge_flags,
    std::vector<uint8_t> * near_ground_flags)
  {
    const double inv_coarse = 1.0 / criteria_.ground_cell_m;
    // ★ 2026-10-09：近地剔除（`near_ground_m > 0`）。**只在有地面估计的格里**判（`!isfinite(g)`
    //   的那些点连 dz 都算不出来 ⇒ 不动它们，与"这一段本来就保守"的既有原则一致）。
    //   与下面的台阶判据**互斥**：`near_ground_m (0.05) < slope_min_height (0.05) ≤
    //   step_height_threshold (0.15)` ⇒ 被判成台阶/陡面的点不可能同时被判成贴地。
    const bool near_ground_on = criteria_.near_ground_m > 0.0;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const float x = cloud[i].x, y = cloud[i].y, z = cloud[i].z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const int64_t cx = static_cast<int64_t>(std::floor(x * inv_coarse));
      const int64_t cy = static_cast<int64_t>(std::floor(y * inv_coarse));
      const float g = groundOf(cx, cy);
      if (!std::isfinite(g)) {
        continue;   // 这一格连地面都估不出来 ⇒ 不动上一级分割器的判定（保守）
      }
      const double dz = static_cast<double>(z) - g;
      if (near_ground_on && dz <= criteria_.near_ground_m) {
        ++near_ground_;
        if (near_ground_flags != nullptr) {
          (*near_ground_flags)[i] = 1u;
        }
      }
      const double slope = slopeOf(cx, cy);
      const bool is_step = dz > criteria_.step_height_threshold;
      const bool is_steep = (slope > criteria_.drivable_slope_deg) &&
        (dz > criteria_.slope_min_height);
      const bool bad = is_step || is_steep;
      if (bad) {
        ++stats_.step_edge;
        if (is_step) {
          ++stats_.step_height;
        }
        if (is_steep) {
          ++stats_.steep_face;
        }
        if (step_edge_flags != nullptr) {
          (*step_edge_flags)[i] = 1u;
        }
        if ((*ground_flags)[i] != 0u) {
          (*ground_flags)[i] = 0u;   // 只降不升
          ++stats_.demoted_ground;
        }
      }
    }
    for (std::size_t i = 0; i < ground_flags->size(); ++i) {
      if ((*ground_flags)[i] != 0u) {
        ++stats_.ground_out;
      }
    }
    stats_.obstacle_out = stats_.points - stats_.ground_out;
  }

  Criteria criteria_;
  FrameStats stats_;
  std::size_t near_ground_{0};   ///< 本帧"贴地"点数（`near_ground_m` 关时恒 0）
  std::unordered_map<int64_t, FineCell> fine_;
  std::unordered_map<int64_t, CoarseCell> ground_cache_;   ///< 判据与前瞻**共用**的粗格缓存
  std::unordered_map<int64_t, float> slope_cache_;
  std::vector<int64_t> keys_;                              ///< describeCorridor 的键缓冲（复用，免每帧分配）
};

}  // namespace rm_ground_traversability

#endif  // RM_GROUND_TRAVERSABILITY__LOW_TERRAIN_CLASSIFIER_HPP_
