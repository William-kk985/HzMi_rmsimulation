// =============================================================================
// traversability_ros.hpp —— 把 LowTerrainClassifier 接进 ROS 节点的那一层（header-only）
// -----------------------------------------------------------------------------
// 两个地面分割节点（linefit / patchwork）都要做**完全相同**的这几件事：
//   ① 声明同一批参数（键名 = src/rm_nav_bringup/config/traversability_criteria.yaml）
//   ② 构造 Criteria + LowTerrainClassifier，并在参数不合法时**直接报错**（不许静默失效）
//   ③ 每帧：跑判据 ⇒ 把不合格的 ground 点降级成 obstacle
//   ④ 发诊断：`<ns>/step_edge`（判据命中的点，obstacle 的子集）+ `~/traversability_stats`（一行文本）
//   ⑤ ★2026-10-07：**前瞻限速**（缺陷 ③ 的后半段，见 docs/slope_speed_limiting.md）：
//      同一帧上再算一次"车前 1~3 m 走廊里的坡度/台阶连续量"，按物理标定的速度表 + 刹车距离界
//      得到速度上限，发成 `nav2_msgs/msg/SpeedLimit` 给 controller_server
//      （nav2 自己转给 MPPI 的 setSpeedLimit ⇒ 缩放 vx_max/vy/wz；**命令链上没有新节点**）。
// 放在这里是为了"一个字节只写一遍"：两个节点的实现不可能再走偏。
//
// 契约（与 docs/ground_segmentation_slots.md §2 一致）：
//   · `/segmentation/ground` 与 `/segmentation/obstacle` 的**发布者个数不变**（各自 1 个），
//     消息类型/帧处理/QoS 全部由调用方（节点）保持原样；
//   · `segmentation/step_edge` / `segmentation/terrain_slope` / `segmentation/terrain_step`
//     是**新增的诊断话题**（同一帧、同 QoS），下游**可以**不订阅；
//   · `speed_limit`（SpeedLimit）是**新增的唯一输出**：本节点只发"上限"，不发 /cmd_vel、
//     不停车、不取消目标、不做 watchdog；/cmd_vel 的发布者仍然是 nav2（velocity_smoother）。
// =============================================================================
#ifndef RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_
#define RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_

#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rm_ground_traversability/low_terrain_classifier.hpp"
#include "rm_ground_traversability/slope_speed_limit.hpp"

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

    // ---- ★2026-10-07 Phase 4：自击掩膜（**默认关**；只有 robot 槽位的参数文件会打开它） ----
    //   为什么放在"判据层"而不是"点云层"：自击是**物理真实**的回波（真机 360° 雷达同样照到云台），
    //   不该从点云/`/segmentation/*` 里删掉；要修的是"它被当成台阶/坡度"这件事。
    //   参数文件按 robot 槽位选（`config/traversability_self_mask[_robot11].yaml`，
    //   机制与 linefit 的 `segmentation_sim_<slot>.yaml` 同款）⇒ 默认槽位读到的永远是
    //   `self_mask_enable: false`，与引入掩膜之前的行为逐字节相同。
    self_mask_.enable = node_->declare_parameter("self_mask_enable", self_mask_.enable);
    self_mask_.boxes = node_->declare_parameter("self_mask_boxes", self_mask_.boxes);
    self_mask_.radius_m = node_->declare_parameter("self_mask_radius_m", self_mask_.radius_m);
    self_mask_.z_min_m = node_->declare_parameter("self_mask_z_min_m", self_mask_.z_min_m);

    // ---- ★2026-10-07：前瞻限速参数（同一份 YAML 的 speed_limit_* 键） ----
    speed_.enable = node_->declare_parameter("speed_limit_enable", speed_.enable);
    speed_.vx_max = node_->declare_parameter("speed_limit_vx_max", speed_.vx_max);
    speed_.lookahead_m = node_->declare_parameter(
      "speed_limit_lookahead_m", speed_.lookahead_m);
    speed_.corridor_half_width_m = node_->declare_parameter(
      "speed_limit_corridor_half_width_m", speed_.corridor_half_width_m);
    speed_.corridor_spread_deg = node_->declare_parameter(
      "speed_limit_corridor_spread_deg", speed_.corridor_spread_deg);
    speed_.slope_baseline_m = node_->declare_parameter(
      "speed_limit_slope_baseline_m", speed_.slope_baseline_m);
    speed_.slope_change_deadband_deg = node_->declare_parameter(
      "speed_limit_slope_change_deadband_deg", speed_.slope_change_deadband_deg);
    speed_.step_deadband_m = node_->declare_parameter(
      "speed_limit_step_deadband_m", speed_.step_deadband_m);
    speed_.step_ignore_above_m = node_->declare_parameter(
      "speed_limit_step_ignore_above_m", speed_.step_ignore_above_m);
    speed_.brake_mps2 = node_->declare_parameter("speed_limit_brake_mps2", speed_.brake_mps2);
    speed_.release_mps2 = node_->declare_parameter(
      "speed_limit_release_mps2", speed_.release_mps2);
    speed_.hold_decay_factor = node_->declare_parameter(
      "speed_limit_hold_decay_factor", speed_.hold_decay_factor);
    speed_.min_support_cells = node_->declare_parameter(
      "speed_limit_min_support_cells", speed_.min_support_cells);
    speed_.floor_mps = node_->declare_parameter("speed_limit_floor_mps", speed_.floor_mps);
    speed_.slope_knots_deg = node_->declare_parameter(
      "speed_limit_slope_knots_deg", speed_.slope_knots_deg);
    speed_.slope_knots_mps = node_->declare_parameter(
      "speed_limit_slope_knots_mps", speed_.slope_knots_mps);
    speed_.step_knots_m = node_->declare_parameter("speed_limit_step_knots_m", speed_.step_knots_m);
    speed_.step_knots_mps = node_->declare_parameter(
      "speed_limit_step_knots_mps", speed_.step_knots_mps);
    speed_.anchor_v_mps = node_->declare_parameter("speed_limit_anchor_v_mps", speed_.anchor_v_mps);
    speed_.anchor_tan_slope = node_->declare_parameter(
      "speed_limit_anchor_tan_slope", speed_.anchor_tan_slope);
    speed_.anchor_peak_mps2 = node_->declare_parameter(
      "speed_limit_anchor_peak_mps2", speed_.anchor_peak_mps2);
    speed_.peak_target_mps2 = node_->declare_parameter(
      "speed_limit_peak_target_mps2", speed_.peak_target_mps2);
    speed_.direction_aware = node_->declare_parameter(
      "speed_limit_direction_aware", speed_.direction_aware);
    speed_.rear_lookahead_m = node_->declare_parameter(
      "speed_limit_rear_lookahead_m", speed_.rear_lookahead_m);
    speed_.direction_min_vx = node_->declare_parameter(
      "speed_limit_direction_min_vx", speed_.direction_min_vx);
    speed_.direction_timeout_s = node_->declare_parameter(
      "speed_limit_direction_timeout_s", speed_.direction_timeout_s);
    const std::string direction_topic = node_->declare_parameter(
      "speed_limit_direction_topic", std::string("odom"));
    const bool publish_speed_diag =
      node_->declare_parameter("speed_limit_publish_diag", true);
    log_period_s_ = node_->declare_parameter("speed_limit_log_period_s", log_period_s_);
    const std::string speed_topic =
      node_->declare_parameter("speed_limit_topic", std::string("speed_limit"));
    const std::string slope_topic = node_->declare_parameter(
      "slope_output_topic", std::string("segmentation/terrain_slope"));
    const std::string step_diag_topic = node_->declare_parameter(
      "step_output_topic", std::string("segmentation/terrain_step"));

    const std::string why = criteria_.reason_invalid();
    if (!why.empty()) {
      throw std::invalid_argument(
        "可通行性判据参数不合法 ⇒ 拒绝启动（避免'参数写错 = 判据静默不生效'）：" + why +
        " 真源文件：src/rm_nav_bringup/config/traversability_criteria.yaml");
    }
    const std::string why_speed = speed_.reason_invalid();
    if (!why_speed.empty()) {
      throw std::invalid_argument(
        "前瞻限速参数不合法 ⇒ 拒绝启动（避免'参数写错 = 限速静默不生效/静默乱限速'）：" +
        why_speed + " 真源文件：src/rm_nav_bringup/config/traversability_criteria.yaml" +
        " 物理推导与校验脚本：docs/slope_speed_limiting.md §2、"
        "tools/scripts/regress/check_slope_speed_table.py");
    }
    const std::string why_mask = self_mask_.reason_invalid();
    if (!why_mask.empty()) {
      throw std::invalid_argument(
        "自击掩膜参数不合法 ⇒ 拒绝启动（避免'参数写错 = 掩膜静默不生效/静默乱掩'）：" +
        why_mask + " 参数文件（按 robot 槽位选）："
        "src/rm_nav_bringup/config/traversability_self_mask[_robot11].yaml；"
        "生成/自检：tools/scripts/regress/robot11_self_mask.py --emit|--check|--verify");
    }
    classifier_ = std::make_unique<LowTerrainClassifier>(criteria_);
    classifier_->setSelfMask(self_mask_);   // ★ Phase 4（默认 enable=false ⇒ 建格行为不变）
    // "可行驶坡度上限"只有一份（Criteria 的 drivable_slope_deg，同一个 YAML 键）：
    // 限速器用它区分"可行驶坡面/坡脚"（要限速）与"陡面/墙"（交给规划器，不限速）。
    speed_.drivable_slope_deg = criteria_.drivable_slope_deg;
    limiter_ = std::make_unique<SlopeSpeedLimiter>(speed_);

    const auto qos = rclcpp::SensorDataQoS();   // 与 ground/obstacle 同一份 QoS
    if (criteria_.enable && publish_step_edge) {
      step_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(step_topic, qos);
    }
    if (criteria_.enable && publish_stats) {
      stats_pub_ = node_->create_publisher<std_msgs::msg::String>("~/traversability_stats", 10);
    }
    if (speed_.enable) {
      // ★ 唯一"对外生效"的输出：nav2 controller_server 的 speed_limit_topic（默认同名）。
      //   QoS 用默认（reliable/volatile, depth 10）：nav2 侧就是 create_subscription<...>(topic, 10)。
      speed_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>(speed_topic, 10);
      if (speed_.direction_aware) {
        // ★ 只**订阅**运动方向（速度符号），不发任何东西；拿不到就回退"只朝前看"。
        //   为什么订阅 /odom 而不是 /cmd_vel：/odom 是实际运动（倒车撞的那一下就是实际在倒），
        //   而且它在 mode:=mapping 下也存在（/cmd_vel 在 mapping 下没有）。
        //   契约影响：导航链路的**发布者**一个都没变（/cmd_vel 仍然只有 velocity_smoother 发）。
        dir_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          direction_topic, rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            last_vx_ = msg->twist.twist.linear.x;
            last_vx_stamp_ns_ = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
              static_cast<int64_t>(msg->header.stamp.nanosec);
            have_vx_ = true;
          });
      }
      if (publish_speed_diag) {
        speed_stats_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "~/slope_speed_stats", 10);
        slope_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(slope_topic, qos);
        step_diag_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          step_diag_topic, qos);
      }
    }

    RCLCPP_INFO(
      node_->get_logger(), "可通行性判据（坡度/台阶）：%s | step_edge 话题=%s",
      criteria_.describe().c_str(),
      (step_pub_ != nullptr) ? step_topic.c_str() : "(关闭)");
    // ★ Phase 4：把掩膜**生效值**打进日志（不是"配了没配"，而是"这次运行到底开没开、几个盒"）
    RCLCPP_INFO(
      node_->get_logger(), "自击掩膜（self_mask）：%s", self_mask_.describe().c_str());
    if (speed_.enable && speed_.direction_aware && !have_vx_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "speed_limit_direction_aware=true 但**还没收到** %s（速度符号）⇒ 先按「只朝前」处理；"
        "若一直如此，方向判据不生效（此时应改用速度表里的反方向窗口 speed_limit_rear_lookahead_m）。",
        direction_topic.c_str());
    }
    if (speed_.enable) {
      RCLCPP_INFO(
        node_->get_logger(),
        "前瞻限速（坡度/台阶 ⇒ 速度上限，发 nav2 SpeedLimit）：%s | 话题=%s（nav2 侧 "
        "controller_server 的 speed_limit_topic；0.0 = nav2 NO_SPEED_LIMIT）",
        speed_.describe().c_str(), speed_topic.c_str());
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "speed_limit_enable=false ⇒ **不发**任何 speed_limit（回退档）：坡脚/台阶前不会自动减速，"
        "速度完全由 nav2 自己的 vx_max / velocity_smoother.max_velocity 决定。");
    }
    if (!criteria_.enable) {
      RCLCPP_WARN(
        node_->get_logger(),
        "traversability_enable=false ⇒ **完全旁路**坡度/台阶判据（回退档）："
        "坡脚/台阶不会因为本判据进 obstacle，前瞻限速也不工作。");
    }
  }

  /// 最佳努力：析构时把"不限速"发回去。**不是 watchdog**（不监测别人、不产生 /cmd_vel）：
  /// 只是避免"本节点正常退出后，MPPI 里还留着最后一次缩放过的约束"。
  /// 进程被 SIGKILL 时这一发不会发生（残留的就是最后一次上限，恢复办法见 docs/slope_speed_limiting.md §4）。
  ~TraversabilityRos()
  {
    if (speed_pub_ && speed_pub_->get_subscription_count() > 0) {
      try {
        publishNoLimit(rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()));
      } catch (const std::exception &) {
        // 关停阶段的任何异常都不该把析构变成 terminate
      }
    }
  }

  const Criteria & criteria() const {return criteria_;}
  const SpeedLimitCriteria & speedCriteria() const {return speed_;}
  const FrameStats & stats() const {return classifier_->stats();}
  bool enabled() const {return criteria_.enable;}
  bool speedEnabled() const {return speed_.enable;}
  const SpeedLimitDecision & decision() const {return decision_;}
  const CorridorProfile & profile() const {return profile_;}

  /// 在一帧上应用判据：`ground_flags` 是 in/out（1=ground，0=obstacle），**只会被降级**。
  /// 同时把判据命中的点写进内部掩码（stepEdgeFlags() 取用，供诊断话题发布）。
  void applyFrame(
    const pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<uint8_t> * ground_flags)
  {
    classifier_->apply(cloud, ground_flags, &step_edge_);
  }

  /// ★2026-10-07：前瞻限速（**必须在 applyFrame() 之后调**：复用同一帧的粗格缓存）。
  /// 只做"测量 → 上限 → 发布"，**不产生 /cmd_vel**。
  /// @param header 本帧点云 header（stamp 用于放开速率限制的 dt）
  void updateSpeedLimit(const std_msgs::msg::Header & header)
  {
    if (!speed_.enable || !criteria_.enable) {
      return;
    }
    CorridorQuery q;
    q.reverse = reverseNow(header.stamp);
    q.lookahead_m = speed_.lookahead_m;
    q.rear_lookahead_m = speed_.rear_lookahead_m;
    q.half_width_m = speed_.corridor_half_width_m;
    q.spread_deg = speed_.corridor_spread_deg;
    q.slope_baseline_m = speed_.slope_baseline_m;
    classifier_->describeCorridor(q, &profile_);

    double dt = 0.0;
    const int64_t stamp_ns = static_cast<int64_t>(header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(header.stamp.nanosec);
    if (last_stamp_ns_ > 0 && stamp_ns > last_stamp_ns_) {
      dt = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
    }
    if (stamp_ns > 0) {
      last_stamp_ns_ = stamp_ns;
    }
    decision_ = limiter_->update(profile_, dt);

    publishSpeedLimit(header);
    publishSlopeSpeedStats();
    if (slope_pub_ || step_diag_pub_) {
      publishCorridorClouds(header);
    }
    logIfNeeded();
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
       << ",\"corridor_cells\":" << s.corridor_cells
       << ",\"corridor_max_slope_deg\":" << s.corridor_max_slope_deg
       << ",\"corridor_max_step_m\":" << s.corridor_max_step_m
       << ",\"self_masked\":" << s.self_masked
       << ",\"self_mask_on\":" << (self_mask_.enable ? "true" : "false")
       << ",\"classify_ms\":" << s.classify_ms << "}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    stats_pub_->publish(msg);
  }

private:
  /// 当前运动方向的符号（true = 倒车）。拿不到 / 太旧 / 接近静止 ⇒ 保持上一次判断；从未拿到 ⇒ 朝前。
  bool reverseNow(const builtin_interfaces::msg::Time & stamp)
  {
    if (!speed_.direction_aware || !have_vx_) {
      return false;
    }
    const int64_t s = static_cast<int64_t>(stamp.sec) * 1000000000LL +
      static_cast<int64_t>(stamp.nanosec);
    if (s > 0 && last_vx_stamp_ns_ > 0) {
      const double age = static_cast<double>(s - last_vx_stamp_ns_) * 1e-9;
      if (age > speed_.direction_timeout_s || age < -1.0) {
        return reverse_;   // 话题旧了/时钟不一致 ⇒ 保持上次判断（不轻易翻转）
      }
    }
    if (last_vx_ > speed_.direction_min_vx) {
      reverse_ = false;
    } else if (last_vx_ < -speed_.direction_min_vx) {
      reverse_ = true;
    }
    return reverse_;
  }

  /// 发 nav2 `SpeedLimit`：absolute m/s；**0.0 = nav2_costmap_2d::NO_SPEED_LIMIT（恢复 MPPI 基准约束）**。
  void publishSpeedLimit(const std_msgs::msg::Header & header)
  {
    if (!speed_pub_) {
      return;
    }
    nav2_msgs::msg::SpeedLimit msg;
    msg.header = header;
    msg.percentage = false;   // 绝对值：速度表是物理量（m/s），与 MPPI 的 vx_max 解耦
    msg.speed_limit = decision_.limited ? decision_.limit_mps : 0.0;
    speed_pub_->publish(msg);
  }

  void publishNoLimit(const rclcpp::Time & stamp)
  {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "";
    msg.percentage = false;
    msg.speed_limit = 0.0;
    speed_pub_->publish(msg);
  }

  /// 诊断：一行 JSON（限速决策 + 走廊连续量的逐格列表）。私有话题 ⇒ 不参与任何契约。
  void publishSlopeSpeedStats()
  {
    if (!speed_stats_pub_) {
      return;
    }
    const SpeedLimitDecision & d = decision_;
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "{\"limit\":" << d.limit_mps << ",\"raw\":" << d.raw_mps
       << ",\"limited\":" << (d.limited ? "true" : "false")
       << ",\"why\":\"" << d.reason << "\""
       << ",\"d\":" << d.feature_d
       << ",\"slope\":" << d.feature_slope_deg
       << ",\"dtan\":" << d.feature_dtan_deg
       << ",\"step\":" << d.feature_step_m
       << ",\"v_req\":" << d.v_req_mps
       << ",\"brake_bound\":" << (d.from_brake_bound ? "true" : "false")
       << ",\"from_hold\":" << (d.from_hold ? "true" : "false")
       << ",\"hold_d\":" << d.hold_d
       << ",\"vx_max\":" << speed_.vx_max
       << ",\"px\":" << (100.0 * d.limit_mps / std::max(speed_.vx_max, 1e-9))
       << ",\"max_slope_deg\":" << d.max_slope_deg
       << ",\"max_dtan_deg\":" << d.max_dtan_deg
       << ",\"max_step_m\":" << d.max_step_m
       << ",\"near_slope_deg\":" << d.near_slope_deg
       << ",\"reverse\":" << (reverse_ ? "true" : "false")
       << ",\"cvx\":" << last_vx_
       << ",\"cells\":" << d.cells
       << ",\"dmin\":" << d.data_min_d << ",\"dmax\":" << d.data_max_d
       << ",\"valid\":" << (d.profile_valid ? "true" : "false")
       << ",\"profile\":[";
    for (std::size_t i = 0; i < profile_.cells.size(); ++i) {
      const CorridorCell & c = profile_.cells[i];
      if (i != 0) {
        os << ",";
      }
      os << "[" << c.d << "," << c.slope_deg << "," << c.dtan_deg << "," << c.step_m << "]";
    }
    os << "]}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    speed_stats_pub_->publish(msg);
  }

  /// 诊断：走廊逐格的连续量（intensity = 坡度 deg / 台阶 m；z = 该格地面高度）。
  void publishCorridorClouds(const std_msgs::msg::Header & header)
  {
    if (slope_pub_) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(corridorCloud(true), msg);
      msg.header = header;
      slope_pub_->publish(msg);
    }
    if (step_diag_pub_) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(corridorCloud(false), msg);
      msg.header = header;
      step_diag_pub_->publish(msg);
    }
  }

  pcl::PointCloud<pcl::PointXYZI> corridorCloud(bool slope) const
  {
    pcl::PointCloud<pcl::PointXYZI> out;
    out.reserve(profile_.cells.size());
    for (const CorridorCell & c : profile_.cells) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(c.x);
      p.y = static_cast<float>(c.y);
      p.z = static_cast<float>(c.ground_z + (slope ? 0.0 : c.step_m));
      p.intensity = static_cast<float>(slope ? c.slope_deg : c.step_m);
      out.push_back(p);
    }
    return out;
  }

  /// `[slope_speed]` 状态行：**当前上限 + 为什么**（哪个特征、多远、坡度/台阶值）。
  /// 只在"上限变了 >0.03 m/s"或每 log_period_s_ 一条（10 Hz 的原始帧不刷屏）。
  void logIfNeeded()
  {
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool changed = std::abs(decision_.limit_mps - last_logged_limit_) > 0.03;
    if (!changed && (now_s - last_log_s_) < std::max(0.1, log_period_s_)) {
      return;
    }
    last_log_s_ = now_s;
    last_logged_limit_ = decision_.limit_mps;
    const SpeedLimitDecision & d = decision_;
    RCLCPP_INFO(
      node_->get_logger(),
      "[slope_speed] limit=%.2f m/s (%.0f%% of %.2f)%s | %s | why=%s d=%.2f m Δ坡=%.1f° 坡=%.1f° "
      "台阶=%.3f m v_req=%.2f%s%s | 走廊: maxΔ坡=%.1f@%.2f m 近坡=%.1f° max台阶=%.3f@%.2f m "
      "cells=%zu d=[%.2f,%.2f]%s",
      d.limit_mps, 100.0 * d.limit_mps / std::max(speed_.vx_max, 1e-9), speed_.vx_max,
      d.limited ? "" : "（不限速）", reverse_ ? "倒车" : "前进", d.reason, d.feature_d,
      d.feature_dtan_deg,
      d.feature_slope_deg, d.feature_step_m, d.v_req_mps,
      d.from_brake_bound ? " 刹车距离界(提前减速)" : "", d.from_hold ? " [已承诺特征前推]" : "",
      d.max_dtan_deg, profile_.max_dtan_d, d.near_slope_deg, d.max_step_m, profile_.max_step_d,
      d.cells, d.data_min_d, d.data_max_d, d.profile_valid ? "" : " **无前瞻数据**");
  }

  rclcpp::Node * node_{nullptr};
  Criteria criteria_;
  SelfMask self_mask_;   // ★ Phase 4（默认 enable=false）
  SpeedLimitCriteria speed_;
  std::unique_ptr<LowTerrainClassifier> classifier_;
  std::unique_ptr<SlopeSpeedLimiter> limiter_;
  CorridorProfile profile_;
  SpeedLimitDecision decision_;
  std::vector<uint8_t> step_edge_;
  int64_t last_stamp_ns_{0};
  double log_period_s_{2.0};
  double last_log_s_{-1e9};
  double last_logged_limit_{-1.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr step_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_stats_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slope_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr step_diag_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dir_sub_;
  double last_vx_{0.0};
  int64_t last_vx_stamp_ns_{0};
  bool have_vx_{false};
  bool reverse_{false};
};

}  // namespace rm_ground_traversability

#endif  // RM_GROUND_TRAVERSABILITY__TRAVERSABILITY_ROS_HPP_
