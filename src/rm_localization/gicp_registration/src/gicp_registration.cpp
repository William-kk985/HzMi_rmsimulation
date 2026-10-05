// GICP 重定位节点（localization:=gicp 槽）—— 只发布 map→odom。
//
// 数据流（每帧）：
//   实时点云(/livox/lidar/pointcloud, BEST_EFFORT) → 去 NaN → 体素下采样(source)
//   TF: T_sensor←odom（点云时间戳）                 —— 把 LIO 的里程计增量接到配准结果上
//   初值 guess = T_map←odom(上次) · T_odom←sensor   —— 用里程计做运动预测 ⇒ GICP 连续跟踪
//   GICP(source → map) → T_map←sensor_est
//   T_map←odom = T_map←sensor_est · T_sensor←odom  → 接受/拒绝 → TF / ~/pose / ~/fitness_score
//
// 行为约定（docs/localization_slots.md §1）：
//   ① **必须有初值**：/initialpose（RELIABLE，RViz 2D Pose Estimate，语义 = map 系下机器人位姿）
//      或 initial_pose 参数；没有初值就**不发 TF**，并在日志里说明原因（不静默）。
//   ② 每帧配准，但只在"收敛且评分达标"时才更新 map→odom；不达标就沿用上一次估计，
//      并（限频 WARN + 连续 N 帧后一条明确的 WARN）说明"定位已失效"。
//   ③ 健康信号：~/fitness_score(m²，无内点时为 nan) + ~/converged(bool)。二者都是
//      transient_local ⇒ 后订阅的监控端也能立刻拿到最后一帧的值。

#include "gicp_registration/gicp_registration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace gicp_registration
{
namespace
{
// TF 查询超时（秒）：精确时间戳那一档故意很短 —— 单线程 executor 里 TF 数据要等当前回调
// 返回后才会被 TransformListener 处理，"带超时等待"其实等不到新数据（tf2_ros/buffer.hpp:314
// 的告警即指此），短超时只保证失败路径快速返回（对比 icp_registration 用的 10 s：TF 缺失时
// 会把点云回调卡住 10 s）。
constexpr double kTfExactTimeoutSec = 0.05;
constexpr double kTfLatestTimeoutSec = 0.05;

Eigen::Quaterniond msgToQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond out(q.w, q.x, q.y, q.z);
  if (!std::isfinite(out.w()) || out.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  out.normalize();
  return out;
}

Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Transform & t)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) = msgToQuaternion(t.rotation).toRotationMatrix();
  m(0, 3) = t.translation.x;
  m(1, 3) = t.translation.y;
  m(2, 3) = t.translation.z;
  return m;
}

Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose & p)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) = msgToQuaternion(p.orientation).toRotationMatrix();
  m(0, 3) = p.position.x;
  m(1, 3) = p.position.y;
  m(2, 3) = p.position.z;
  return m;
}

void matrixToTransform(const Eigen::Matrix4d & m, geometry_msgs::msg::Transform & t)
{
  const Eigen::Quaterniond q(m.block<3, 3>(0, 0));
  const Eigen::Quaterniond qn =
    q.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : q.normalized();
  t.translation.x = m(0, 3);
  t.translation.y = m(1, 3);
  t.translation.z = m(2, 3);
  t.rotation.w = qn.w();
  t.rotation.x = qn.x();
  t.rotation.y = qn.y();
  t.rotation.z = qn.z();
}

// 日志用：位置 + yaw（map→odom 这种小量看 yaw 比看四元数直观）
std::string poseToStr(const Eigen::Matrix4d & m)
{
  char buf[128];
  std::snprintf(
    buf, sizeof(buf), "x=%.3f y=%.3f z=%.3f yaw=%.2f°",
    m(0, 3), m(1, 3), m(2, 3), std::atan2(m(1, 0), m(0, 0)) * 180.0 / M_PI);
  return std::string(buf);
}

// 日志用：fitness score（m²）——nan 表示"无有效内点"
std::string scoreToStr(double score)
{
  if (!std::isfinite(score)) {
    return "nan(无内点)";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.5f", score);
  return std::string(buf);
}
}  // namespace

GicpNode::GicpNode(const rclcpp::NodeOptions & options)
: Node("gicp_registration", options)
{
  // ============================ 参数 ============================
  // 每个键的来历 / 在 PCL 1.12.1 里的依据，逐条写在 config/gicp_registration_sim.yaml 的注释里。
  pcd_path_ = declare_parameter<std::string>("pcd_path", "");
  map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  laser_frame_id_ = declare_parameter<std::string>("laser_frame_id", "livox_frame");
  pointcloud_topic_ =
    declare_parameter<std::string>("pointcloud_topic", "/livox/lidar/pointcloud");

  use_initial_pose_ = declare_parameter<bool>("use_initial_pose", true);
  initial_pose_param_ =
    declare_parameter<std::vector<double>>("initial_pose", std::vector<double>{0.0, 0.0, 0.0});

  voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.25);
  max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.0);
  maximum_iterations_ = declare_parameter<int>("maximum_iterations", 32);
  transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 5.0e-4);
  rotation_epsilon_ = declare_parameter<double>("rotation_epsilon", 2.0e-3);
  correspondence_randomness_ = declare_parameter<int>("correspondence_randomness", 20);
  maximum_optimizer_iterations_ = declare_parameter<int>("maximum_optimizer_iterations", 20);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  fitness_score_warn_ = declare_parameter<double>("fitness_score_warn", 0.05);
  max_fitness_score_ = declare_parameter<double>("max_fitness_score", 0.3);
  no_improve_cycles_warn_ = declare_parameter<int>("no_improve_cycles_warn", 10);
  stale_warn_sec_ = declare_parameter<double>("stale_warn_sec", 3.0);
  // 注：euclidean_fitness_epsilon 与 use_reciprocal_correspondences **故意不声明**：
  //     PCL 1.12.1 的 GICP 覆写了 computeTransformation（impl/gicp.hpp:390），既不读
  //     euclidean_fitness_epsilon_（只有 ICP/JointICP 的 convergence_criteria 用，impl/icp.hpp:157），
  //     也不读 use_reciprocal_correspondence_（只有 ICP::determineCorrespondences 用，impl/icp.hpp:180）
  //     ⇒ 写了就是静默失效（详见 config/gicp_registration_sim.yaml 顶部说明）。

  if (initial_pose_param_.size() != 3 && initial_pose_param_.size() != 6) {
    RCLCPP_WARN(
      get_logger(),
      "initial_pose 需要 3 或 6 个元素 [x,y,z(,roll,pitch,yaw)]，实际 %zu 个 ⇒ 退回 [0,0,0]",
      initial_pose_param_.size());
    initial_pose_param_ = {0.0, 0.0, 0.0};
  }
  if (voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "voxel_leaf_size=%.3f 非法 ⇒ 用 0.25", voxel_leaf_size_);
    voxel_leaf_size_ = 0.25;
  }
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz=%.3f 非法 ⇒ 用 50.0", publish_rate_hz_);
    publish_rate_hz_ = 50.0;
  }
  if (maximum_iterations_ <= 0) {
    RCLCPP_WARN(get_logger(), "maximum_iterations=%d 非法 ⇒ 用 32", maximum_iterations_);
    maximum_iterations_ = 32;
  }
  std::string init_pose_str;
  for (size_t i = 0; i < initial_pose_param_.size(); ++i) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%s%.3f", i ? ", " : "", initial_pose_param_[i]);
    init_pose_str += buf;
  }

  // ============================ 先验地图（只加载一次） ============================
  if (pcd_path_.empty() || !std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(
      get_logger(),
      "pcd_path 无效：'%s'。launch 会注入 <rm_nav_bringup>/PCD/<world>.pcd"
      "（见 docs/localization_slots.md §1），也可用 --ros-args -p pcd_path:=/abs/path.pcd",
      pcd_path_.c_str());
    throw std::runtime_error("gicp_registration: invalid pcd_path");
  }
  PointCloudT::Ptr raw_map(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(pcd_path_, *raw_map) < 0) {
    RCLCPP_ERROR(get_logger(), "PCD 读取失败：%s", pcd_path_.c_str());
    throw std::runtime_error("gicp_registration: cannot read pcd");
  }
  // 去 NaN：RMUC.pcd 里确实存在 NaN 点（NaN 会污染 GICP 的 KNN 协方差与最近邻）
  pcl::Indices finite_indices;
  PointCloudT::Ptr map_finite(new PointCloudT);
  pcl::removeNaNFromPointCloud(*raw_map, *map_finite, finite_indices);
  map_cloud_ = downsample(map_finite);
  const size_t n_raw = raw_map->size();
  const size_t n_finite = map_finite->size();
  const size_t n_map = map_cloud_->size();
  PointT map_min;
  PointT map_max;
  pcl::getMinMax3D(*map_cloud_, map_min, map_max);
  if (n_map <= static_cast<size_t>(std::max(4, correspondence_randomness_))) {
    // PCL 的 computeCovariances 在 k_correspondences_ > cloud->size() 时只打一条 PCL_ERROR
    // 就返回（impl/gicp.hpp:57-60），协方差数组是空的 → 之后按 index 访问会越界。
    // 所以"地图点太少"在这里直接判为致命错误（顺带把点数与包围盒打出来：
    // 本仓库的 PCD/RMUC.pcd 就是**退化资产**——65 万点的 x/y/z 全挤在原点附近 3 cm 的盒子里，
    // 不论怎么调参都不可能用来定位，必须换资产）。
    RCLCPP_ERROR(
      get_logger(),
      "地图点太少：原始 %zu → 去 NaN %zu → 体素 %.2f m 后 %zu 点"
      "（<= correspondence_randomness=%d）⇒ GICP 无法工作。\n"
      "    下采样后包围盒：min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)\n"
      "    若包围盒小得离谱（如 <0.1 m）说明 PCD 本身就是退化资产（点全挤在一点），"
      "请换 PCD；否则可调小 correspondence_randomness / voxel_leaf_size",
      n_raw, n_finite, voxel_leaf_size_, n_map, correspondence_randomness_,
      map_min.x, map_min.y, map_min.z, map_max.x, map_max.y, map_max.z);
    throw std::runtime_error("gicp_registration: map cloud too small");
  }

  // ============================ GICP 配置 ============================
  gicp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
  gicp_.setMaximumIterations(maximum_iterations_);
  gicp_.setTransformationEpsilon(transformation_epsilon_);
  gicp_.setRotationEpsilon(rotation_epsilon_);
  gicp_.setCorrespondenceRandomness(correspondence_randomness_);
  gicp_.setMaximumOptimizerIterations(maximum_optimizer_iterations_);
  gicp_.setInputTarget(map_cloud_);  // 目标只设一次；目标协方差在首次 align 时预计算并复用

  // ============================ 状态初值 ============================
  last_cloud_stamp_ = now();
  if (use_initial_pose_) {
    param_init_pending_ = true;  // 首帧点云 + TF 就绪时惰性初始化（见 pointcloudCallback）
  } else {
    estimate_valid_ = true;  // 恒等 map→odom 起步：把 LIO 的 odom 直接当绝对位姿
    RCLCPP_WARN(
      get_logger(),
      "use_initial_pose=false ⇒ map→odom 从**恒等**起步（等价于把 LIO 的 odom 当绝对位姿，"
      "首帧 GICP 之后才被修正）；initial_pose 参数被忽略，但 /initialpose（RViz）仍然有效");
  }

  // ============================ ROS 接口 ============================
  // 点云：BEST_EFFORT（发布端 livox 用 SensorDataQoS；默认 RELIABLE 订阅会永远收不到
  // —— 这正是 icp_registration 在 f033d96 之前"跑起来但没数据"的原因）。KEEP_LAST(1)：
  // 配准慢时宁可丢旧帧，也不要排队处理过期点云。
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&GicpNode::pointcloudCallback, this, std::placeholders::_1));
  // /initialpose：RELIABLE（RViz 的 2D Pose Estimate 就是 RELIABLE 发的，不能共用点云 QoS）
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&GicpNode::initialPoseCallback, this, std::placeholders::_1));

  pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", rclcpp::QoS(1));
  fitness_score_pub_ = create_publisher<std_msgs::msg::Float64>(
    "~/fitness_score", rclcpp::QoS(1).transient_local());
  converged_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/converged", rclcpp::QoS(1).transient_local());

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&GicpNode::publishTimerCallback, this));

  // ============================ 启动摘要 ============================
  RCLCPP_INFO(
    get_logger(),
    "\n===== gicp_registration 启动 =====\n"
    "  pcd_path=%s\n"
    "  地图点数：原始 %zu → 去 NaN %zu → 体素 %.2f m 后 %zu（= GICP target）\n"
    "  地图包围盒：min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)"
    "（应覆盖机器人活动区；若小得离谱说明资产退化）\n"
    "  frames: map='%s' odom='%s' base='%s' laser(仅兜底)='%s'\n"
    "  pointcloud_topic=%s（SensorDataQoS / BEST_EFFORT）\n"
    "  GICP: max_corr_dist=%.3f m, maximum_iterations=%d, transformation_epsilon=%.2e,\n"
    "        rotation_epsilon=%.2e, correspondence_randomness=%d, maximum_optimizer_iterations=%d\n"
    "  初值: use_initial_pose=%s initial_pose=[%s]%s\n"
    "  输出: TF %s→%s @%.1f Hz + ~/pose + ~/fitness_score + ~/converged"
    "（fitness warn=%.3f / accept=%.3f m²）\n"
    "  阈值: no_improve_cycles_warn=%d, stale_warn_sec=%.1f\n"
    "  注：首个 fitness score 之前会先做一次目标协方差预计算，可能耗时数秒\n"
    "==================================",
    pcd_path_.c_str(), n_raw, n_finite, voxel_leaf_size_, n_map,
    map_min.x, map_min.y, map_min.z, map_max.x, map_max.y, map_max.z,
    map_frame_id_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(), laser_frame_id_.c_str(),
    pointcloud_topic_.c_str(),
    max_correspondence_distance_, maximum_iterations_, transformation_epsilon_, rotation_epsilon_,
    correspondence_randomness_, maximum_optimizer_iterations_,
    use_initial_pose_ ? "true" : "false", init_pose_str.c_str(),
    use_initial_pose_ ? "（首帧点云 + TF odom→base 就绪时生效）" : "（已忽略）",
    map_frame_id_.c_str(), odom_frame_id_.c_str(), publish_rate_hz_,
    fitness_score_warn_, max_fitness_score_, no_improve_cycles_warn_, stale_warn_sec_);
}

// ============================ 工具 ============================

PointCloudT::Ptr GicpNode::downsample(const PointCloudT::Ptr & in) const
{
  PointCloudT::Ptr out(new PointCloudT);
  pcl::VoxelGrid<PointT> voxel;
  const float leaf = static_cast<float>(voxel_leaf_size_);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(in);
  voxel.filter(*out);
  return out;
}

bool GicpNode::lookupTf(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  bool try_exact_stamp, Eigen::Matrix4d & out, bool & used_latest)
{
  used_latest = false;
  if (try_exact_stamp) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(kTfExactTimeoutSec));
      out = transformToMatrix(tf.transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      last_tf_error_ = ex.what();
    }
  }
  try {
    // tf2::TimePointZero = "取最新可用"（用户点 /initialpose、点云时间戳查不到时的兜底）
    const auto tf = tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero, tf2::durationFromSec(kTfLatestTimeoutSec));
    out = transformToMatrix(tf.transform);
    used_latest = try_exact_stamp;  // 只有"本想用精确时间戳"时才值得提示
    return true;
  } catch (const tf2::TransformException & ex) {
    last_tf_error_ = ex.what();
    return false;
  }
}

Eigen::Matrix4d GicpNode::initialPoseParamToMatrix() const
{
  // initial_pose 的语义 = **map 系下机器人（base_frame）的位姿**，与 /initialpose、
  // 与 AMCL 的 initial_pose_x/y/yaw 完全一致 ⇒ 同一套 RViz 操作对 amcl / icp / gicp 三个槽都成立。
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = initial_pose_param_[0];
  m(1, 3) = initial_pose_param_[1];
  m(2, 3) = initial_pose_param_[2];
  if (initial_pose_param_.size() == 6) {
    const Eigen::AngleAxisd roll(initial_pose_param_[3], Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(initial_pose_param_[4], Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(initial_pose_param_[5], Eigen::Vector3d::UnitZ());
    m.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  }
  return m;
}

// ============================ 回调 ============================

void GicpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // --- ① 转 PCL（点类型 pcl::PointXYZ：只要求 x/y/z，不碰 intensity） ---
  PointCloudT::Ptr raw(new PointCloudT);
  try {
    pcl::fromROSMsg(*msg, *raw);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "点云转换失败（%s）。本节点按 pcl::PointXYZ 读取，只需要 x/y/z 三个字段；请检查发布端",
      ex.what());
    publishHealth(false, 0.0, false);
    return;
  }
  const std::string sensor_frame =
    msg->header.frame_id.empty() ? laser_frame_id_ : msg->header.frame_id;
  if (sensor_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "点云 header.frame_id 为空且 laser_frame_id 参数也为空 ⇒ 无法查 TF，跳过本帧");
    publishHealth(false, 0.0, false);
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_cloud_stamp_ = stamp;
    cloud_seen_ = true;
  }

  // --- ② 去 NaN + 体素下采样（与地图用同一个 leaf size ⇒ 两侧点密度/协方差尺度一致） ---
  pcl::Indices finite_indices;
  PointCloudT::Ptr finite(new PointCloudT);
  pcl::removeNaNFromPointCloud(*raw, *finite, finite_indices);
  PointCloudT::Ptr source = downsample(finite);
  const size_t min_points = static_cast<size_t>(std::max(4, correspondence_randomness_)) + 1;
  if (source->size() < min_points) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "下采样后源点云仅 %zu 点（< %zu = correspondence_randomness+1）⇒ 跳过本帧，不跑 GICP"
      "（避免 PCL 内部协方差数组越界）", source->size(), min_points);
    publishHealth(false, 0.0, false);
    return;
  }

  // --- ③ T_sensor←odom（点云时间戳；查不到就退到最新可用） ---
  Eigen::Matrix4d T_sensor_odom;
  bool used_latest = false;
  if (!lookupTf(sensor_frame, odom_frame_id_, stamp, true, T_sensor_odom, used_latest)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF %s←%s 查询失败（%s）⇒ 跳过本帧。map→odom 仍沿用上一次估计"
      "（若从未有过估计则**不发 TF**，global_costmap 会报 Invalid frame ID \"map\"）",
      sensor_frame.c_str(), odom_frame_id_.c_str(), last_tf_error_.c_str());
    publishHealth(false, 0.0, false);
    return;
  }
  if (used_latest) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF %s←%s 在点云时间戳上查不到，已退到**最新可用**变换（会引入一点时间偏差）",
      sensor_frame.c_str(), odom_frame_id_.c_str());
  }

  // --- ④ 当前估计 ---
  Eigen::Matrix4d T_map_odom_cur;
  bool valid_cur = false;
  bool init_pending = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_cur = estimate_valid_;
    T_map_odom_cur = T_map_odom_;
    init_pending = param_init_pending_;
  }

  // ④b 首次：用 initial_pose 参数惰性初始化（需要 TF odom→base 已可用；失败则下一帧重试）
  if (!valid_cur && init_pending) {
    Eigen::Matrix4d T_odom_base;
    bool init_used_latest = false;
    if (lookupTf(odom_frame_id_, base_frame_id_, stamp, false, T_odom_base, init_used_latest)) {
      const Eigen::Matrix4d T_map_base_init = initialPoseParamToMatrix();
      T_map_odom_cur = T_map_base_init * T_odom_base;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        T_map_odom_ = T_map_odom_cur;
        estimate_valid_ = true;
        param_init_pending_ = false;
        no_improve_cycles_ = 0;
      }
      valid_cur = true;
      RCLCPP_INFO(
        get_logger(), "用参数 initial_pose 初始化：map 系机器人位姿 %s ⇒ map→odom = %s",
        poseToStr(T_map_base_init).c_str(), poseToStr(T_map_odom_cur).c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "有 initial_pose 参数，但 TF %s←%s 还查不到（%s）⇒ 暂不发布 map→odom；"
        "TF 就绪后会自动初始化，也可用 RViz 的 2D Pose Estimate 手动给初值",
        odom_frame_id_.c_str(), base_frame_id_.c_str(), last_tf_error_.c_str());
    }
  }
  if (!valid_cur) {
    // 没有初值 ⇒ 不做配准（GICP 无初值必然收敛到局部极小 = 车"跑到墙里"），也不发 TF。
    publishHealth(false, 0.0, false);
    return;
  }

  // --- ⑤ 初值 = 上一次 map→odom 乘上这帧的里程计增量（连续跟踪的关键） ---
  const Eigen::Matrix4d guess = T_map_odom_cur * T_sensor_odom.inverse();
  if (!guess.allFinite() || !T_sensor_odom.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "初值/TF 含非有限值 ⇒ 跳过本帧");
    publishHealth(false, 0.0, false);
    return;
  }

  // --- ⑥ GICP ---
  if (!first_align_done_) {
    RCLCPP_INFO(
      get_logger(), "首次 GICP align：会先预计算目标（地图 %zu 点）协方差，可能耗时数秒…",
      map_cloud_->size());
  }
  bool converged = false;
  double score = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix4d T_map_sensor = guess;
  try {
    gicp_.setInputSource(source);  // 每帧设源：GICP 会重算源协方差（地图侧协方差被复用）
    PointCloudT aligned;
    gicp_.align(aligned, guess.cast<float>());
    converged = gicp_.hasConverged();
    // getFitnessScore 的入参在 PCL 里是**平方距离**阈值（impl/registration.hpp:152 直接拿它与
    // kd-tree 的平方距离比较），所以要传 r²；返回**内点的平均平方距离（m²）**，
    // 没有任何内点时返回 numeric_limits<double>::max()。
    score =
      gicp_.getFitnessScore(max_correspondence_distance_ * max_correspondence_distance_);
    T_map_sensor = gicp_.getFinalTransformation().cast<double>();
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "GICP align 抛异常（%s）⇒ 本帧不更新 map→odom", ex.what());
  }
  first_align_done_ = true;
  if (!std::isfinite(score) || score >= std::numeric_limits<double>::max()) {
    score = std::numeric_limits<double>::quiet_NaN();  // "无有效内点" → nan（~/fitness_score 可见）
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    total_cycles_++;
  }

  // --- ⑦ 接受 / 拒绝 ---
  const bool accepted = converged && std::isfinite(score) && score <= max_fitness_score_;
  if (accepted) {
    T_map_odom_cur = T_map_sensor * T_sensor_odom;
    int n_acc = 0;
    int n_tot = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      T_map_odom_ = T_map_odom_cur;
      estimate_valid_ = true;
      no_improve_cycles_ = 0;
      n_acc = ++accepted_cycles_;
      n_tot = total_cycles_;
    }
    RCLCPP_DEBUG(
      get_logger(), "GICP OK：score=%s m², converged=true, map→odom=%s（%d/%d 帧被接受）",
      scoreToStr(score).c_str(), poseToStr(T_map_odom_cur).c_str(), n_acc, n_tot);
  } else {
    int n = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      n = ++no_improve_cycles_;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GICP 本帧未被采纳（converged=%s, score=%s m², accept 阈值=%.3f）⇒ map→odom 保持上一次估计"
      "（连续 %d 帧；TF 仍按上次估计发布）",
      converged ? "true" : "false", scoreToStr(score).c_str(), max_fitness_score_, n);
    if (n == no_improve_cycles_warn_) {
      RCLCPP_WARN(
        get_logger(),
        "★★ GICP 已连续 %d 帧未被采纳：map→odom 一直沿用旧值 ⇒ **定位实际已失效**"
        "（RViz 里车会停住不动或慢慢漂）。排查顺序：\n"
        "   ① 初值：用 RViz 2D Pose Estimate 给正确位置（或 initial_pose 参数）"
        "——ICP 族没有初值必然不收敛；\n"
        "   ② 看 ~/fitness_score 的实际量级再调 max_fitness_score / fitness_score_warn"
        "（单位 m² = 内点平均平方距离）；\n"
        "   ③ voxel_leaf_size（点太多/太少）、max_correspondence_distance；\n"
        "   ④ 确认 PCD 与 map/<world>.pgm 同源同系（否则先验地图本身就错位）。", n);
    }
  }
  publishHealth(true, score, accepted && score <= fitness_score_warn_);

  // --- ⑧ 调试位姿：map 系机器人位姿（≈ /amcl_pose 语义，便于 A/B 对照） ---
  {
    std::lock_guard<std::mutex> lock(mutex_);
    T_map_odom_cur = T_map_odom_;
    valid_cur = estimate_valid_;
  }
  if (valid_cur) {
    publishPose(stamp, T_map_odom_cur, accepted ? score : -1.0);
  }
}

void GicpNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // /initialpose 的语义（与 AMCL 一致）= map 系下**机器人（base_frame）**的位姿
  // ⇒ T_map←odom = T_map←base · T_base←odom，其中 T_base←odom 由 TF 提供（LIO 的 odom→base）。
  // 这样 GICP 不必自己猜雷达外参，也没改变用户对 RViz 的用法。
  const Eigen::Matrix4d T_map_base = poseToMatrix(msg->pose.pose);
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  Eigen::Matrix4d T_odom_base;
  bool used_latest = false;
  // 精确时间戳查不到会自动退到"最新可用"（用户点击的语义本来就是"此刻"）
  if (!lookupTf(odom_frame_id_, base_frame_id_, stamp, true, T_odom_base, used_latest)) {
    RCLCPP_WARN(
      get_logger(), "/initialpose 收到，但 TF %s←%s 查不到（%s）⇒ 忽略本次初值（LIO/TF 未就绪？）",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), last_tf_error_.c_str());
    return;
  }
  const Eigen::Matrix4d T_map_odom = T_map_base * T_odom_base;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    T_map_odom_ = T_map_odom;
    estimate_valid_ = true;
    param_init_pending_ = false;  // 人工给的初值优先于 initial_pose 参数
    no_improve_cycles_ = 0;
  }
  RCLCPP_INFO(
    get_logger(), "/initialpose 更新初值：map 系机器人位姿 %s ⇒ map→odom = %s",
    poseToStr(T_map_base).c_str(), poseToStr(T_map_odom).c_str());
  // 注意：rclcpp::Time::is_zero() 是 Kilted 之后才有的 API，Humble 里用 nanoseconds()==0 判断
  publishPose(
    stamp.nanoseconds() == 0 ? now() : stamp, T_map_odom, -1.0 /*本次没有 fitness score*/);
}

// ============================ 发布 ============================

void GicpNode::publishTimerCallback()
{
  Eigen::Matrix4d T_map_odom;
  bool valid = false;
  bool cloud_seen = false;
  rclcpp::Time last_cloud_stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    T_map_odom = T_map_odom_;
    valid = estimate_valid_;
    cloud_seen = cloud_seen_;
    last_cloud_stamp = last_cloud_stamp_;
  }

  if (valid) {
    // 按 publish_rate_hz 重复发布：nav2 每帧都查 map→odom，不能只依赖配准帧率
    publishTf(T_map_odom);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "尚无 map→odom：**不发 TF**（发错误的初值比不发更糟）。等 /initialpose 或 initial_pose 参数"
      " + TF odom→%s 就绪", base_frame_id_.c_str());
  }

  if (cloud_seen) {
    const double age = (now() - last_cloud_stamp).seconds();
    if (age > stale_warn_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "已 %.1f s 没有收到点云（topic=%s）⇒ 定位停止更新（map→odom 仍按上次估计发布）",
        age, pointcloud_topic_.c_str());
    }
  }
}

void GicpNode::publishTf(const Eigen::Matrix4d & T_map_odom)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();  // 用"当前"时间戳：nav2 在 now() 上查 map→odom
  tf_msg.header.frame_id = map_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  matrixToTransform(T_map_odom, tf_msg.transform);
  tf_broadcaster_->sendTransform(tf_msg);
}

void GicpNode::publishPose(
  const rclcpp::Time & stamp, const Eigen::Matrix4d & T_map_odom, double score)
{
  Eigen::Matrix4d T_odom_base;
  bool used_latest = false;
  if (!lookupTf(odom_frame_id_, base_frame_id_, stamp, true, T_odom_base, used_latest)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF %s←%s 查不到（%s）⇒ 本帧不发布 ~/pose（TF map→odom 不受影响）",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), last_tf_error_.c_str());
    return;
  }
  const Eigen::Matrix4d T_map_base = T_map_odom * T_odom_base;

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = map_frame_id_;
  out.pose.pose.position.x = T_map_base(0, 3);
  out.pose.pose.position.y = T_map_base(1, 3);
  out.pose.pose.position.z = T_map_base(2, 3);
  const Eigen::Quaterniond q(T_map_base.block<3, 3>(0, 0));
  const Eigen::Quaterniond qn = q.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : q.normalized();
  out.pose.pose.orientation.w = qn.w();
  out.pose.pose.orientation.x = qn.x();
  out.pose.pose.orientation.y = qn.y();
  out.pose.pose.orientation.z = qn.z();
  // 协方差：本节点不做协方差估计，只把 fitness score（m²）抄到 x/y/yaw 对角线当调试线索
  // （score<0 = 本帧没有新评分）。**下游不要当严格协方差用**。
  if (score >= 0.0 && std::isfinite(score)) {
    out.pose.covariance[0] = score;
    out.pose.covariance[7] = score;
    out.pose.covariance[35] = score;
  }
  pose_pub_->publish(out);
}

void GicpNode::publishHealth(bool has_score, double score, bool healthy)
{
  if (has_score) {
    std_msgs::msg::Float64 msg;
    msg.data = score;
    fitness_score_pub_->publish(msg);
  }
  std_msgs::msg::Bool conv;
  conv.data = healthy;
  converged_pub_->publish(conv);
}

}  // namespace gicp_registration

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gicp_registration::GicpNode)
