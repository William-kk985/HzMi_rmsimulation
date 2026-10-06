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
           drivable_slope_deg < 90.0 && slope_min_height >= 0.0;
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
       << " ground_min_points=" << ground_min_points;
    return os.str();
  }
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

  /// 主入口。
  /// @param cloud          输入点云（帧与 labels 同序）
  /// @param ground_flags   in/out：1 = ground（可行驶），0 = obstacle。**只会被降级，不会被升级**
  /// @param step_edge_flags out：1 = 本判据命中的点（台阶/边沿 ∪ 超限坡面）；可为 nullptr
  void apply(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    std::vector<uint8_t> * ground_flags,
    std::vector<uint8_t> * step_edge_flags)
  {
    stats_ = FrameStats();
    stats_.points = cloud.size();
    if (step_edge_flags != nullptr) {
      step_edge_flags->assign(cloud.size(), 0u);
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
    classify(cloud, ground_flags, step_edge_flags);
    const auto t1 = std::chrono::steady_clock::now();
    stats_.classify_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.coarse_cells = ground_cache_.size();
    stats_.coarse_cells_no_ground = 0;
    for (const auto & kv : ground_cache_) {
      if (!std::isfinite(kv.second)) {
        ++stats_.coarse_cells_no_ground;
      }
    }
  }

private:
  // ---- 细格 / 粗格 ---------------------------------------------------------
  struct FineCell
  {
    float lo1{std::numeric_limits<float>::infinity()};   ///< 最低点
    float lo2{std::numeric_limits<float>::infinity()};   ///< 次低点（只为观测；地面取 lo1）
    int n{0};
  };

  static int64_t keyOf(int64_t ix, int64_t iy)
  {
    // ix/iy 各自 32 位足够（±0.05 m × 2^31 ≈ ±1e8 m）；负数用低 32 位截断后拼装。
    return (static_cast<int64_t>(static_cast<int32_t>(ix)) << 32) |
           static_cast<int64_t>(static_cast<uint32_t>(static_cast<int32_t>(iy)));
  }

  void buildGrid(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    fine_.clear();
    ground_cache_.clear();
    slope_cache_.clear();
    fine_.reserve(cloud.size() * 2);
    const double inv_fine = 1.0 / criteria_.fine_cell_m;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const float x = cloud[i].x, y = cloud[i].y, z = cloud[i].z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;   // NaN/inf：不参与建格；标签保持上一级分割器的判定
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
      ++cell.n;
    }
  }

  /// 该点的粗格地面高度（m）；无穷大 = 没有可用地面。
  float groundOf(int64_t cx, int64_t cy)
  {
    const int64_t key = keyOf(cx, cy);
    const auto it = ground_cache_.find(key);
    if (it != ground_cache_.end()) {
      return it->second;
    }
    const int k = std::min(criteria_.cell_ratio(), 8);   // 上限 8×8 = 64 次查表，防病态参数
    std::vector<float> lows;
    lows.reserve(static_cast<std::size_t>(k * k));
    int points = 0;
    for (int a = 0; a < k; ++a) {
      for (int b = 0; b < k; ++b) {
        const auto fit = fine_.find(keyOf(cx * k + a, cy * k + b));
        if (fit == fine_.end()) {
          continue;
        }
        lows.push_back(fit->second.lo1);
        points += fit->second.n;
      }
    }
    float g = std::numeric_limits<float>::infinity();
    if (!lows.empty() && points >= std::max(1, criteria_.ground_min_points)) {
      std::sort(lows.begin(), lows.end());
      // 与 pcd_to_nav2_map.py 的 _per_cell_percentile 同一条取秩公式：
      //   target = floor(pct/100 * (cnt-1))。小样本时它退化成最小值 —— 两边一致。
      const std::size_t cnt = lows.size();
      const std::size_t target = static_cast<std::size_t>(
        std::floor(criteria_.ground_percentile / 100.0 *
        static_cast<double>(cnt > 0 ? cnt - 1 : 0)));
      g = lows[std::min(target, cnt - 1)];
    }
    ground_cache_[key] = g;
    return g;
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
    std::vector<uint8_t> * step_edge_flags)
  {
    const double inv_coarse = 1.0 / criteria_.ground_cell_m;
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
  std::unordered_map<int64_t, FineCell> fine_;
  std::unordered_map<int64_t, float> ground_cache_;
  std::unordered_map<int64_t, float> slope_cache_;
};

}  // namespace rm_ground_traversability

#endif  // RM_GROUND_TRAVERSABILITY__LOW_TERRAIN_CLASSIFIER_HPP_
