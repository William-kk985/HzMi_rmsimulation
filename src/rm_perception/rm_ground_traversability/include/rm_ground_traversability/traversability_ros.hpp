// =============================================================================
// traversability_ros.hpp —— 把 LowTerrainClassifier 接进 ROS 节点的那一层（header-only）
// -----------------------------------------------------------------------------
// 两个地面分割节点（linefit / patchwork）都要做**完全相同**的这几件事：
//   ① 声明同一批参数（键名 = src/rm_nav_bringup/config/traversability_criteria.yaml）
//   ② 构造 Criteria + LowTerrainClassifier，并在参数不合法时**直接报错**（不许静默失效）
//   ③ 每帧：跑判据 ⇒ 把不合格的 ground 点降级成 obstacle
//   ④ 发诊断：`<ns>/step_edge`（判据命中的点，obstacle 的子集）+ `~/traversability_stats`（一行文本）
// 放在这里是为了"一个字节只写一遍"：两个节点的实现不可能再走偏。
//
// 契约（与 docs/ground_segmentation_slots.md §2 一致）：
//   · `/segmentation/ground` 与 `/segmentation/obstacle` 的**发布者个数不变**（各自 1 个），
//     消息类型/帧处理/QoS 全部由调用方（节点）保持原样；
//   · `segmentation/step_edge` 是**新增的诊断话题**（同一帧、同 QoS），下游**可以**不订阅。
// =============================================================================
#ifndef RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_
#define RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rm_ground_traversability/low_terrain_classifier.hpp"

namespace rm_ground_traversability
{

class TraversabilityRos
{
public:
  explicit TraversabilityRos(rclcpp::Node * node)
  : node_(node)
  {
    if (node_ == nullptr) {
      throw std::invalid_argument("TraversabilityRos: node 为空");
    }
    criteria_.enable = node_->declare_parameter("traversability_enable", criteria_.enable);
    criteria_.step_height_threshold = node_->declare_parameter(
      "step_height_threshold", criteria_.step_height_threshold);
    criteria_.drivable_slope_deg = node_->declare_parameter(
      "drivable_slope_deg", criteria_.drivable_slope_deg);
    criteria_.slope_min_height = node_->declare_parameter(
      "slope_min_height", criteria_.slope_min_height);
    criteria_.ground_cell_m = node_->declare_parameter("ground_cell_m", criteria_.ground_cell_m);
    criteria_.fine_cell_m = node_->declare_parameter("fine_cell_m", criteria_.fine_cell_m);
    criteria_.ground_percentile = node_->declare_parameter(
      "ground_percentile", criteria_.ground_percentile);
    criteria_.ground_min_points = node_->declare_parameter(
      "ground_min_points", criteria_.ground_min_points);
    const bool publish_step_edge =
      node_->declare_parameter("publish_step_edge", true);
    const bool publish_stats =
      node_->declare_parameter("publish_traversability_stats", true);
    const std::string step_topic =
      node_->declare_parameter("step_edge_output_topic", std::string("segmentation/step_edge"));

    const std::string why = criteria_.reason_invalid();
    if (!why.empty()) {
      throw std::invalid_argument(
        "可通行性判据参数不合法 ⇒ 拒绝启动（避免'参数写错 = 判据静默不生效'）：" + why +
        " 真源文件：src/rm_nav_bringup/config/traversability_criteria.yaml");
    }
    classifier_ = std::make_unique<LowTerrainClassifier>(criteria_);

    const auto qos = rclcpp::SensorDataQoS();   // 与 ground/obstacle 同一份 QoS
    if (criteria_.enable && publish_step_edge) {
      step_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(step_topic, qos);
    }
    if (criteria_.enable && publish_stats) {
      stats_pub_ = node_->create_publisher<std_msgs::msg::String>("~/traversability_stats", 10);
    }

    RCLCPP_INFO(
      node_->get_logger(), "可通行性判据（坡度/台阶）：%s | step_edge 话题=%s",
      criteria_.describe().c_str(),
      (step_pub_ != nullptr) ? step_topic.c_str() : "(关闭)");
    if (!criteria_.enable) {
      RCLCPP_WARN(
        node_->get_logger(),
        "traversability_enable=false ⇒ **完全旁路**坡度/台阶判据（回退档）："
        "坡脚/台阶不会因为本判据进 obstacle。");
    }
  }

  const Criteria & criteria() const {return criteria_;}
  const FrameStats & stats() const {return classifier_->stats();}
  bool enabled() const {return criteria_.enable;}

  /// 在一帧上应用判据：`ground_flags` 是 in/out（1=ground，0=obstacle），**只会被降级**。
  /// 同时把判据命中的点写进内部掩码（stepEdgeFlags() 取用，供诊断话题发布）。
  void applyFrame(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<uint8_t> * ground_flags)
  {
    classifier_->apply(cloud, ground_flags, &step_edge_);
  }

  const std::vector<uint8_t> & stepEdgeFlags() const {return step_edge_;}

  /// 发 `step_edge` 诊断话题（用与 ground/obstacle 相同的 header 与 QoS）。
  void publishStepEdge(
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const std_msgs::msg::Header & header)
  {
    if (step_pub_ == nullptr) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> out;
    out.reserve(step_edge_.size());
    for (std::size_t i = 0; i < step_edge_.size() && i < cloud.size(); ++i) {
      if (step_edge_[i] != 0u) {
        out.push_back(cloud[i]);
      }
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header = header;
    step_pub_->publish(msg);
  }

  /// 发一行统计（std_msgs/String，私有话题）。内容 = FrameStats 的关键字段。
  void publishStats()
  {
    if (stats_pub_ == nullptr) {
      return;
    }
    const FrameStats & s = classifier_->stats();
    std::ostringstream os;
    os << "{\"points\":" << s.points << ",\"ground_in\":" << s.ground_in
       << ",\"ground_out\":" << s.ground_out << ",\"obstacle_out\":" << s.obstacle_out
       << ",\"step_edge\":" << s.step_edge << ",\"step_height\":" << s.step_height
       << ",\"steep_face\":" << s.steep_face << ",\"demoted_ground\":" << s.demoted_ground
       << ",\"coarse_cells\":" << s.coarse_cells
       << ",\"coarse_cells_no_ground\":" << s.coarse_cells_no_ground
       << ",\"classify_ms\":" << s.classify_ms << "}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    stats_pub_->publish(msg);
  }

private:
  rclcpp::Node * node_{nullptr};
  Criteria criteria_;
  std::unique_ptr<LowTerrainClassifier> classifier_;
  std::vector<uint8_t> step_edge_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr step_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
};

}  // namespace rm_ground_traversability

#endif  // RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_
