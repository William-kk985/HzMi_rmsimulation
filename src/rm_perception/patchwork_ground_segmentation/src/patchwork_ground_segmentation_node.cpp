// =============================================================================
// patchwork_ground_segmentation_node —— 地面分割槽位 `ground:=patchwork` 的节点
// -----------------------------------------------------------------------------
// 目标：**与 linefit_ground_segmentation_ros/ground_segmentation_node 同契约**，
//       让下游（pointcloud_to_laserscan / STVL / 任何吃 /segmentation/obstacle 的东西）零改动。
//       参照实现：src/rm_perception/linefit_ground_segementation_ros2/
//                 linefit_ground_segmentation_ros/src/ground_segmentation_node.cc
//
// 契约（逐条对齐，对照表见 docs/ground_segmentation_slots.md §2）：
//   · 节点名            ground_segmentation（与 linefit 节点同名 ⇒ 参数段名可直接沿用同一份风格）
//   · 订阅              input_topic（默认 /livox/lidar/pointcloud），rclcpp::SensorDataQoS()
//                       = BEST_EFFORT + keep_last(5)（与 linefit 2026-09-23 定稿一致）
//   · 发布              obstacle_output_topic（默认 segmentation/obstacle，相对名 ⇒ /segmentation/obstacle）
//                       ground_output_topic（默认 segmentation/ground ⇒ /segmentation/ground）
//                       同为 SensorDataQoS()、同为 sensor_msgs/PointCloud2
//   · 帧处理            输出 header **整体抄输入**（stamp + frame_id 逐字段相同）；
//                       gravity_aligned_frame 非空时才做"只旋转不平移"的重力对齐（与 linefit 相同）
//   · 点类型            pcl::PointXYZ（x/y/z）⇒ 输出字段集与 linefit 逐字段一致
//   · 完备性            **每个输入点必定落进 ground 或 obstacle 之一**（linefit 的语义：
//                       越界点(r<r_min 或 r>r_max)算障碍；patchwork 的 pc2czm 同样是"越界→nonground"）
//   · 不使用 intensity  patchwork 的 RNR 需要第 4 列 intensity；我们的仿真点云没有 ⇒ enable_RNR 默认关
//   · 不使用 timebase   完全不读 timebase/时间字段（仿真插件不填）
//
// 唯一有意的差异（不改行为，只减故障面）：linefit 无条件构造 tf2_ros::Buffer + TransformListener；
//   本节点**只在 gravity_aligned_frame 非空时**才构造（默认空 ⇒ 不建 TF 监听）。
//   理由与 pointcloud_to_laserscan 的 target_frame:"" 同款（见该包 config 注释）：移除整类
//   "tf2 过滤器丢消息/静默排队"的故障面。契约（输出 header）不变。
// =============================================================================

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ⚠ patchworkpp.h 在全局作用域有 `using namespace std;`（上游如此）⇒ 只在本 TU include，
//   不要把它扩散到别的头文件（见 thirdparty/patchwork-plusplus/VENDORING.md §5）。
#include "patchwork/patchworkpp.h"

// ★ 2026-10-07（缺陷 ③）：地面分割之后的**坡度/台阶判定**（与 linefit 槽位、与离线
//   tools/scripts/mapping/pcd_to_nav2_map.py 共用同一份阈值文件）。判据/阈值/为什么见
//   src/rm_nav_bringup/config/traversability_criteria.yaml 与
//   docs/ground_segmentation_slots.md §10。
#include "rm_ground_traversability/traversability_ros.hpp"

namespace patchwork_ground_segmentation
{

/// 把 rclcpp 的 vector<int64_t>/vector<double> 形状校验成 patchwork 代码**不会越界**的形状。
/// patchworkpp 内部对这些 vector 一律用 `at()`/`[]` 且假定长度 ≥ num_zones / num_rings_of_interest；
/// 长度不够会抛 std::out_of_range 或读到栈外 ⇒ 这里在构造期一次性挡住，并给出可操作的报错。
static void require_size(
  const std::string & name, std::size_t got, std::size_t need, const std::string & why)
{
  if (got < need) {
    throw std::invalid_argument(
      "参数 '" + name + "' 长度 " + std::to_string(got) + " < 需要的 " + std::to_string(need) +
      "（" + why + "）。请修正参数文件后重启本节点。");
  }
}

class PatchworkGroundSegmentationNode : public rclcpp::Node
{
public:
  explicit PatchworkGroundSegmentationNode(const rclcpp::NodeOptions & node_options)
  : rclcpp::Node("ground_segmentation", node_options)
  {
    // ------------------------- 1. 契约参数（与 linefit 同名、同默认值） -------------------------
    // 默认值与 linefit_ground_segmentation_ros/config/segmentation_sim.yaml 逐键一致，
    // 这样"只换 ground 槽位、不改任何参数文件"时输入/输出接线完全相同。
    gravity_aligned_frame_ = declare_parameter("gravity_aligned_frame", std::string(""));
    const std::string input_topic = declare_parameter("input_topic", std::string("/livox/lidar/pointcloud"));
    const std::string ground_topic = declare_parameter("ground_output_topic", std::string("segmentation/ground"));
    const std::string obstacle_topic =
      declare_parameter("obstacle_output_topic", std::string("segmentation/obstacle"));
    publish_timing_ = declare_parameter("publish_timing", true);

    // ------------------------- 2. Patchwork++ 算法参数 -------------------------
    patchwork::Params p;
    // 默认值来源逐条见 config/ground_segmentation_sim.yaml 的注释与 docs/ground_segmentation_slots.md §3。
    p.sensor_height = declare_parameter("sensor_height", p.sensor_height);
    p.min_range = declare_parameter("min_range", p.min_range);
    p.max_range = declare_parameter("max_range", p.max_range);
    p.th_dist = declare_parameter("th_dist", p.th_dist);
    p.th_seeds = declare_parameter("th_seeds", p.th_seeds);
    p.th_seeds_v = declare_parameter("th_seeds_v", p.th_seeds_v);
    p.th_dist_v = declare_parameter("th_dist_v", p.th_dist_v);
    p.num_iter = declare_parameter("num_iter", p.num_iter);
    p.num_lpr = declare_parameter("num_lpr", p.num_lpr);
    p.num_min_pts = declare_parameter("num_min_pts", p.num_min_pts);
    p.num_zones = declare_parameter("num_zones", p.num_zones);
    p.num_rings_of_interest = declare_parameter("num_rings_of_interest", p.num_rings_of_interest);
    p.uprightness_thr = declare_parameter("uprightness_thr", p.uprightness_thr);
    p.adaptive_seed_selection_margin =
      declare_parameter("adaptive_seed_selection_margin", p.adaptive_seed_selection_margin);
    p.enable_RNR = declare_parameter("enable_RNR", p.enable_RNR);
    p.enable_RVPF = declare_parameter("enable_RVPF", p.enable_RVPF);
    p.enable_TGR = declare_parameter("enable_TGR", p.enable_TGR);
    p.RNR_ver_angle_thr = declare_parameter("RNR_ver_angle_thr", p.RNR_ver_angle_thr);
    p.RNR_intensity_thr = declare_parameter("RNR_intensity_thr", p.RNR_intensity_thr);
    p.max_flatness_storage = declare_parameter("max_flatness_storage", p.max_flatness_storage);
    p.max_elevation_storage = declare_parameter("max_elevation_storage", p.max_elevation_storage);

    const std::vector<int64_t> sectors =
      declare_parameter("num_sectors_each_zone", std::vector<int64_t>{16, 32, 54, 32});
    const std::vector<int64_t> rings =
      declare_parameter("num_rings_each_zone", std::vector<int64_t>{2, 4, 4, 4});
    const std::vector<double> elevation_thr =
      declare_parameter("elevation_thr", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    const std::vector<double> flatness_thr =
      declare_parameter("flatness_thr", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    p.verbose = declare_parameter("verbose", false);

    // ------------------------- 3. 形状校验（在构造 PatchWorkpp 之前） -------------------------
    // 上限 4 的硬约束来自上游代码本身：
    //   · PatchWorkpp 构造函数固定只算 4 组 min_ranges_/ring_sizes_/sector_sizes_（`{...}` 4 项）；
    //   · pc2czm() 只有 4 个分支（zone 0..3）；
    //   · update_elevation_/update_flatness_ 是固定长度 4 的数组。
    if (p.num_zones < 1 || p.num_zones > 4) {
      throw std::invalid_argument(
        "num_zones=" + std::to_string(p.num_zones) +
        " 越界：上游 Patchwork++ 的 CZM 固定 4 组（min_ranges_/ring_sizes_/sector_sizes_ 各 4 项、"
        "pc2czm 只有 4 个分支）⇒ 合法范围 1..4。");
    }
    if (p.num_rings_of_interest < 1 || p.num_rings_of_interest > 4) {
      throw std::invalid_argument(
        "num_rings_of_interest=" + std::to_string(p.num_rings_of_interest) +
        " 越界：update_elevation_/update_flatness_ 是固定长度 4 的数组、elevation_thr/flatness_thr 按它索引"
        "⇒ 合法范围 1..4。");
    }
    require_size("num_sectors_each_zone", sectors.size(), static_cast<std::size_t>(p.num_zones),
      "上游 `num_sectors_each_zone.at(zone_idx)` 按 num_zones 取用");
    require_size("num_rings_each_zone", rings.size(), static_cast<std::size_t>(p.num_zones),
      "上游 `num_rings_each_zone.at(zone_idx)` 按 num_zones 取用");
    require_size("elevation_thr", elevation_thr.size(),
      static_cast<std::size_t>(p.num_rings_of_interest),
      "上游按 `elevation_thr[concentric_idx]` 索引，concentric_idx < num_rings_of_interest");
    require_size("flatness_thr", flatness_thr.size(),
      static_cast<std::size_t>(p.num_rings_of_interest),
      "上游按 `flatness_thr[concentric_idx]` 索引，concentric_idx < num_rings_of_interest");
    if (p.min_range <= 0.0 || p.max_range <= p.min_range) {
      throw std::invalid_argument(
        "min_range/max_range 非法：要求 0 < min_range < max_range，当前 min_range=" +
        std::to_string(p.min_range) + " max_range=" + std::to_string(p.max_range) + "。");
    }
    if (p.enable_RNR) {
      // 不是致命错误，但必须让用户知道它在本数据上无效（否则会以为是"开了 RNR"）。
      RCLCPP_WARN(
        get_logger(),
        "enable_RNR=true，但本数据源是 pcl::PointXYZ（无 intensity 第 4 列）⇒ 上游 "
        "reflected_noise_removal() 会打印 \"RNR requires intensity information !\" 后直接 return，"
        "即**实际未生效**。要真正启用 RNR 需要 intensity 通道（当前仿真链路没有）。");
    }

    p.num_sectors_each_zone.assign(sectors.begin(), sectors.end());
    p.num_rings_each_zone.assign(rings.begin(), rings.end());
    p.elevation_thr.assign(elevation_thr.begin(), elevation_thr.end());
    p.flatness_thr.assign(flatness_thr.begin(), flatness_thr.end());

    params_ = p;
    segmenter_ = std::make_unique<patchwork::PatchWorkpp>(params_);

    // ------------------------- 4. 接线（与 linefit 逐字段一致） -------------------------
    // QoS：SensorDataQoS() = BEST_EFFORT + keep_last(5)。理由见 linefit 节点里 2026-09-23 那段注释
    //（RELIABLE 写者会被慢消费者堵死，实测把整条感知链冻 180 s）。
    const auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, qos,
      std::bind(&PatchworkGroundSegmentationNode::scanCallback, this, std::placeholders::_1));
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, qos);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic, qos);
    if (publish_timing_) {
      timing_pub_ = create_publisher<std_msgs::msg::Float64>("~/segmentation_time_ms", 10);
    }

    // ★ 2026-10-07：坡度/台阶判定（参数在 traversability_criteria.yaml；参数不合法会直接抛）
    traversability_ = std::make_unique<rm_ground_traversability::TraversabilityRos>(this);

    if (!gravity_aligned_frame_.empty()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    RCLCPP_INFO(
      get_logger(),
      "patchwork_ground_segmentation 就绪：in=%s out=%s,%s | sensor_height=%.3f m "
      "min_range=%.2f max_range=%.1f th_dist=%.3f num_iter=%d num_zones=%d "
      "RNR=%d RVPF=%d TGR=%d uprightness=%.3f gravity_aligned_frame='%s'",
      input_topic.c_str(), obstacle_topic.c_str(), ground_topic.c_str(), params_.sensor_height,
      params_.min_range, params_.max_range, params_.th_dist, params_.num_iter, params_.num_zones,
      static_cast<int>(params_.enable_RNR), static_cast<int>(params_.enable_RVPF),
      static_cast<int>(params_.enable_TGR), params_.uprightness_thr, gravity_aligned_frame_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);  // 只取 x/y/z；intensity/timebase 缺失或多余都不影响

    const std::size_t n = cloud.size();
    if (n == 0) {
      return;
    }

    // 重力对齐（只在 gravity_aligned_frame 非空时；与 linefit 相同：**只旋转、不平移**，
    // 且输出仍然用**原始点**（labels 与原始云同序）。
    // ⚠ 判据必须跑在**同一朵（可能已对齐的）云**上：本类假定 z 轴向上。
    std::vector<uint8_t> is_ground(n, 0);
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    const pcl::PointCloud<pcl::PointXYZ> * cloud_proc = &cloud;
    if (!gravity_aligned_frame_.empty() && tf_buffer_) {
      try {
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
        tf_stamped.transform.translation.x = 0;
        tf_stamped.transform.translation.y = 0;
        tf_stamped.transform.translation.z = 0;
        Eigen::Affine3d tf = Eigen::Affine3d::Identity();
        tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
        pcl::transformPointCloud(cloud, cloud_transformed, tf);
        cloud_proc = &cloud_transformed;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          get_logger(), "Failed to transform point cloud into gravity frame: %s", ex.what());
        cloud_proc = &cloud;
      }
    }
    segment(*cloud_proc, /*labels_out=*/&is_ground);

    // ★ 2026-10-07（缺陷 ③）：**地面分割之后**再判一次"坡度/台阶"。
    //   语义：只把判成 ground 的点按判据**降级**成 obstacle（永不升级）⇒ 坡脚/台阶进
    //   `/segmentation/obstacle` ⇒ 进而进 `/scan` 与代价图。可行驶的坡面（≤25°）按
    //   "相对局部地面的高差"判定 ⇒ 保持 free。详见 traversability_ros.hpp 头注。
    traversability_->applyFrame(*cloud_proc, &is_ground);

    // ★★ 2026-10-09：**近地剔除**（`obstacle_near_ground_m`，默认 **0.0 = 关**）。
    //   语义与 linefit 那一份**逐字相同**（见 ground_segmentation_node.cc 的同名段落与
    //   src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml 的文件头）：
    //   `dz = z − 局部地面 ≤ 阈值` 的点 = 贴着地面 ⇒ 改判 ground（这一步是**升**，
    //   所以显式写在这里，而不是塞进判据类的"只降不升"里）。
    //   为什么必须在这一级：`/scan` 是二维平盘，nav2 的 `obstacle_layer` 用 projectLaser（z=0）
    //   再搬到代价图帧 ⇒ 那条链路上的 z 恒等于"传感器原点的 z"，**没有逐点高度可判**。
    //   关掉时（默认）`nearGroundEnabled()` 为假 ⇒ 下面整个循环不执行，行为逐字节不变。
    if (traversability_->nearGroundEnabled()) {
      const std::vector<uint8_t> & near_ground = traversability_->nearGroundFlags();
      for (std::size_t i = 0; i < is_ground.size() && i < near_ground.size(); ++i) {
        if (near_ground[i] != 0u) {
          is_ground[i] = 1u;
        }
      }
    }

    // 拆两朵云（与 linefit 完全相同的写法：逐一按 label 归拢，保持原始顺序）
    pcl::PointCloud<pcl::PointXYZ> ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
    std::size_t n_ground = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (is_ground[i] == 1u) {
        ++n_ground;
      }
    }
    ground_cloud.reserve(n_ground);
    obstacle_cloud.reserve(n - n_ground);
    for (std::size_t i = 0; i < n; ++i) {
      if (is_ground[i] == 1u) {
        ground_cloud.push_back(cloud[i]);
      } else {
        obstacle_cloud.push_back(cloud[i]);
      }
    }

    auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(ground_cloud, *ground_msg);
    pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
    ground_msg->header = msg->header;      // 与 linefit 相同：stamp/frame_id 逐字段抄输入
    obstacle_msg->header = msg->header;
    ground_pub_->publish(*ground_msg);
    obstacle_pub_->publish(*obstacle_msg);
    // 诊断：判据命中的点（`/segmentation/obstacle` 的子集）+ 每帧统计（私有话题）
    traversability_->publishStepEdge(cloud, msg->header);
    traversability_->publishStats();
    // ★ 2026-10-07：前瞻限速（缺陷 ③ 后半段）。**必须在 applyFrame() 之后**：它复用同一帧的
    //   粗格缓存（局部地面 / 台阶连续量），不重算判据。只发 nav2 SpeedLimit，不产生 /cmd_vel。
    traversability_->updateSpeedLimit(msg->header);

    if (timing_pub_) {
      std_msgs::msg::Float64 t;
      t.data = last_segmentation_ms_;
      timing_pub_->publish(t);
    }
  }

  /// 跑一帧 Patchwork++，把结果摊成"每个输入点 1=ground / 0=obstacle"。
  ///
  /// 完备性论证：上游 estimateGround() 里
  ///   · pc2czm()：`min_range < r <= max_range` 的点进 CZM，**其余直接进 cloud_nonground_**（越界→障碍，
  ///     与 linefit 的 `range_square < r_max_square && range_square > r_min_square` 反例判据一致）；
  ///   · 主循环：每个 patch 的点或被并入 regionwise_ground_（→ 经 GLE 进 ground 或 nonground），
  ///     或被并入 regionwise_nonground_（→ nonground）；R-VPF 剔掉的点也进 non_ground_dst。
  ///   ⇒ ground ∪ nonground == 全部输入点，唯一例外是 pc2czm 里
  ///     `if (z == std::numeric_limits<float>::min()) continue;` 那个哨兵（RNR 用来标记"已删"，
  ///     我们没开 RNR ⇒ 不会产生）。这里对"两个列表都没有"的点**按障碍兜底**，
  ///     于是 ground+obstacle 恒等于输入点数（契约完备性可断言）。
  void segment(const pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<uint8_t> * labels_out)
  {
    const std::size_t n = cloud.size();
    std::vector<uint8_t> & is_ground = *labels_out;
    is_ground.assign(n, 0);

    Eigen::MatrixXf mat(static_cast<Eigen::Index>(n), 3);
    for (std::size_t i = 0; i < n; ++i) {
      mat(static_cast<Eigen::Index>(i), 0) = cloud[i].x;
      mat(static_cast<Eigen::Index>(i), 1) = cloud[i].y;
      mat(static_cast<Eigen::Index>(i), 2) = cloud[i].z;
    }

    const auto t0 = std::chrono::steady_clock::now();
    segmenter_->estimateGround(std::move(mat));
    const auto t1 = std::chrono::steady_clock::now();
    last_segmentation_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

    const Eigen::VectorXi ground_idx = segmenter_->getGroundIndices();
    std::size_t labelled = 0;
    for (Eigen::Index j = 0; j < ground_idx.size(); ++j) {
      const int idx = ground_idx(j);
      if (idx >= 0 && static_cast<std::size_t>(idx) < n) {
        is_ground[static_cast<std::size_t>(idx)] = 1u;
        ++labelled;
      }
    }
    // 不变量自检：ground 索引不应重复、不应越界（上游保证唯一且来自输入行号）。异常时只报一次。
    if (labelled != static_cast<std::size_t>(ground_idx.size()) && !index_warned_) {
      index_warned_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Patchwork++ 返回的 ground 索引里有 %zu 个越界/重复项（共 %zu）⇒ 已忽略；"
        "这些点按障碍处理。",
        static_cast<std::size_t>(ground_idx.size()) - labelled,
        static_cast<std::size_t>(ground_idx.size()));
    }
  }

  patchwork::Params params_;
  std::unique_ptr<patchwork::PatchWorkpp> segmenter_;
  std::unique_ptr<rm_ground_traversability::TraversabilityRos> traversability_;
  std::string gravity_aligned_frame_;
  bool publish_timing_{true};
  bool index_warned_{false};
  double last_segmentation_ms_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace patchwork_ground_segmentation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    auto node =
      std::make_shared<patchwork_ground_segmentation::PatchworkGroundSegmentationNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("patchwork_ground_segmentation"), "启动失败：%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
