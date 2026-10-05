#ifndef GICP_REGISTRATION__GICP_REGISTRATION_HPP_
#define GICP_REGISTRATION__GICP_REGISTRATION_HPP_

// GICP 重定位槽（localization:=gicp）。
//
// 契约（与 icp_registration / amcl / slam_toolbox / cartographer 一致，见 docs/localization_slots.md）：
//   **只发布 map→odom**（TF）+ 调试/健康话题；绝不碰 odom→base_link（那是 LIO 的职责）。
//   时间戳契约：TF map→odom 与 ~/pose 都用 **now + tf_lookahead_sec** 盖戳（与 AMCL 的
//   transform_tolerance 同语义），因为 nav2 消费者是在 now+margin 处查 map→odom 的。
//
// 并发契约（2026-10-05 新增，治"align 饿死 TF 定时器"）：
//   本节点跑在 **MultiThreadedExecutor** 上（见 CMakeLists.txt 的 EXECUTOR MultiThreadedExecutor，
//   生成 main 里是 exec.add_node(node) + exec.spin()），并把回调分成**两个 MutuallyExclusive 组**：
//     · tf_cb_group_    —— 只放 TF/状态定时器（publishTimerCallback）；
//     · align_cb_group_ —— 点云订阅（重活 GICP align）+ /initialpose。
//   两组之间并行 ⇒ 单帧 align ~350 ms（实测 327~389 ms）也阻塞不了 50 Hz 的 TF 发布（改前实测 /tf 只有 2.3 Hz）。
//   因此**跨两组共享的成员必须在 mutex_ 下访问**（逐项清单见下面"状态"块）；
//   GICP 对象 / first_align_done_ / last_tf_error_ 只由 align 组内的回调触碰 —— 组内互斥保证
//   它们仍是单线程访问（PCL GICP 自身不是线程安全的，绝不允许并发 align）。
//
// 点类型选择：pcl::PointXYZ —— 只用 XYZ，两个理由：
//   ① PCL 1.12.1 的 GICP **自己算协方差**（pcl/registration/impl/gicp.hpp:51-125 的
//      computeCovariances 只读 x/y/z，累计 mean/cov 后做 SVD），normal_*/curvature/intensity
//      这些字段它一个都不读 ⇒ 用 PointNormal / PointXYZINormal 只是在浪费内存和 IO。
//   ② 我们的 PCD 资产字段并不统一：RMUL2026.pcd = x y z intensity normal_x..curvature；
//      RMUC.pcd = normal_x..z x y z _（**没有 intensity**）。用 PointXYZ 三种资产都能读；
//      若用 PointXYZI 去读 RMUC.pcd，PCL 会因缺 intensity 报 "Failed to find match for field"
//      并整帧失败。实时点云同理（不依赖 intensity）。

#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace gicp_registration
{

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class GicpNode : public rclcpp::Node
{
public:
  explicit GicpNode(const rclcpp::NodeOptions & options);

private:
  // ---- 回调 ----
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void publishTimerCallback();

  // ---- 工具 ----
  // 体素下采样：leaf_size 由调用方给（**两级 leaf**：地图 target 用 voxel_leaf_size_，
  // 实时点云 source 用 voxel_leaf_size_scan_）。两个 leaf 均为正数（构造时已校验）。
  PointCloudT::Ptr downsample(const PointCloudT::Ptr & in, double leaf_size) const;
  bool lookupTf(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    bool try_exact_stamp, Eigen::Matrix4d & out, bool & used_latest);
  // last_tf_error_ 的加锁快照（lookupTf 在 mutex_ 下写；日志在读之前先拷一份，不裸读成员）
  std::string lastTfError() const;
  Eigen::Matrix4d initialPoseParamToMatrix() const;
  // TF / ~/pose 的**发布戳** = now()（节点时钟；use_sim_time=true 时 = 仿真时间）+ tf_lookahead_sec_。
  // 与 AMCL 的 transform_tolerance 同语义，理由见 src/gicp_registration.cpp 里 publishTf 的注释。
  rclcpp::Time lookaheadStamp() const;
  // 返回本条 TF 用的戳（= lookaheadStamp()），供 publishPose 复用同一戳
  rclcpp::Time publishTf(const Eigen::Matrix4d & T_map_odom);
  void publishPose(const Eigen::Matrix4d & T_map_odom, double score);
  void publishHealth(bool has_score, double score, bool healthy);

  // ---- 参数（键名与依据逐条见 config/gicp_registration_sim.yaml） ----
  std::string pcd_path_;
  std::string map_frame_id_, odom_frame_id_, base_frame_id_, laser_frame_id_;
  std::string pointcloud_topic_;
  bool use_initial_pose_;
  std::vector<double> initial_pose_param_;
  // 两级下采样（2026-10-05 起，recipe 来自 COD 2025 small_gicp_relocalization 的
  // global_leaf_size / registered_leaf_size）：
  //   voxel_leaf_size_      = **先验地图 / GICP target** 的 leaf（默认 0.10 m）；
  //   voxel_leaf_size_scan_ = **实时点云 / GICP source** 的 leaf（默认 0.05 m，细一档）。
  // 键名 voxel_leaf_size 保持不变（向后兼容），语义收窄为"仅地图侧"。
  double voxel_leaf_size_;
  double voxel_leaf_size_scan_;
  double max_correspondence_distance_;
  int maximum_iterations_;
  double transformation_epsilon_, rotation_epsilon_;
  int correspondence_randomness_;
  int maximum_optimizer_iterations_;
  double publish_rate_hz_;
  double fitness_score_warn_, max_fitness_score_, stale_warn_sec_;
  int no_improve_cycles_warn_;
  // TF map→odom（以及 ~/pose）的时间戳前瞻量（秒）= AMCL transform_tolerance 语义：
  // 把 map→odom 盖成**未来**时间戳，nav2 消费者在 now+margin 处才查得到（否则 tf2 抛
  // "Lookup would require extrapolation into the future"）。详见 config 与本文件 publishTf 注释。
  // 取值：**参数文件里现值 0.45**（2026-10-05 commit c8863f1：0.3 → 0.45，给"TF 定时器被 CPU
  // 饿死"留余量）；本行代码里的兜底默认仍是 0.3（= AMCL transform_tolerance，仅当参数文件缺失
  // /未注入时生效 —— launch 一定注入 config，故实跑恒为 0.45）。
  double tf_lookahead_sec_;

  // ---- 地图 / 配准器 ----
  PointCloudT::Ptr map_cloud_;  // 已体素下采样、已去 NaN（GICP 的 target）
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp_;

  // ---- 状态（mutex_ 保护） ----
  // mutex_ 的职责（2026-10-05 逐项审计，MT executor + 双回调组之后这是正确性的关键）：
  //   **定时器组（publishTimerCallback / publishPose 复用戳）与 align 组（pointcloudCallback /
  //   initialPoseCallback）都会碰的成员，一律在 mutex_ 下读写**：
  //     · T_map_odom_ / estimate_valid_        —— align 写、定时器读（缓存的 map→odom）
  //     · last_tf_stamp_ / last_tf_stamp_valid_—— 定时器写、align 组（publishPose）读
  //     · cloud_seen_ / last_cloud_stamp_      —— align 写、定时器读（点云陈旧度 WARN）
  //     · accepted_cycles_ / total_cycles_ / no_improve_cycles_ / param_init_pending_
  //                                            —— align 写、定时器读（[status] 行 / 状态机）
  //     · last_align_ms_ / last_score_ / last_source_points_ —— align 写、定时器读（可观测量）
  //     · last_status_log_tp_ / status_log_ever_printed_     —— 定时器读写（~1 Hz 限频）
  //     · last_tf_error_                       —— lookupTf 写（加锁）、日志读（lastTfError() 加锁）
  //   **故意的例外**：三个健康话题（~/pose、~/fitness_score、~/converged）只由 align 组发布，
  //   定时器组从不碰它们（唯一的发布者是 pointcloudCallback / initialPoseCallback）；rclcpp 的
  //   Publisher::publish() 本身线程安全，而**持锁发布**会让一次阻塞的 DDS 写把定时器线程一起拖住
  //   ⇒ 等于把刚修好的"饿死"换个姿势带回来 ⇒ 发布**不进状态锁**，靠"单一回调组拥有"这条不变量保证。
  //   同理 tf_broadcaster_->sendTransform 只在定时器组里调用（它不与 align 组共享 publisher）。
  mutable std::mutex mutex_;
  Eigen::Matrix4d T_map_odom_ = Eigen::Matrix4d::Identity();
  bool estimate_valid_ = false;
  bool param_init_pending_ = false;
  bool cloud_seen_ = false;
  rclcpp::Time last_cloud_stamp_;
  // 最近一次真正发出去的 TF map→odom 的戳（= now+tf_lookahead_sec_）。~/pose 复用它，
  // 保证 "pose 的戳 == TF 的戳"（消费端拿 pose 的戳去查 TF 时不会落在最新条目之外）。
  rclcpp::Time last_tf_stamp_;
  bool last_tf_stamp_valid_ = false;
  int no_improve_cycles_ = 0;
  int accepted_cycles_ = 0;
  int total_cycles_ = 0;
  bool first_align_done_ = false;
  std::string last_tf_error_ = "n/a";

  // ---- 可观测量（~1 Hz 状态行用）：A/B 时要看**每帧配准耗时**与**评分量级** ----
  double last_align_ms_ = 0.0;                                    // 最近一帧 gicp_.align() 墙钟耗时
  double last_score_ = std::numeric_limits<double>::quiet_NaN();  // 最近一帧 fitness score（m²）
  size_t last_source_points_ = 0;                                 // 最近一帧下采样后的源点数
  std::chrono::steady_clock::time_point last_status_log_tp_{};    // 状态行限频（~1 Hz）
  bool status_log_ever_printed_ = false;

  // ---- ROS 接口 ----
  // 回调组（2026-10-05）：定时器与重活分属两个 MutuallyExclusive 组，由 MultiThreadedExecutor 并行调度。
  rclcpp::CallbackGroup::SharedPtr tf_cb_group_;     // 只放 publish_timer_（TF/状态发布）
  rclcpp::CallbackGroup::SharedPtr align_cb_group_;  // 点云订阅（GICP align）+ /initialpose
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr converged_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace gicp_registration

#endif  // GICP_REGISTRATION__GICP_REGISTRATION_HPP_
