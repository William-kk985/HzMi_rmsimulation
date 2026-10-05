#ifndef GICP_REGISTRATION__GICP_REGISTRATION_HPP_
#define GICP_REGISTRATION__GICP_REGISTRATION_HPP_

// GICP 重定位槽（localization:=gicp）。
//
// 契约（与 icp_registration / amcl / slam_toolbox / cartographer 一致，见 docs/localization_slots.md）：
//   **只发布 map→odom**（TF）+ 调试/健康话题；绝不碰 odom→base_link（那是 LIO 的职责）。
//   时间戳契约：TF map→odom 与 ~/pose 都用 **now + tf_lookahead_sec** 盖戳（与 AMCL 的
//   transform_tolerance 同语义），因为 nav2 消费者是在 now+margin 处查 map→odom 的。
//
// 并发契约（2026-10-05 新增，治"align 饿死 TF 定时器"；同日二次修复治"align 饿死 /initialpose"）：
//   本节点跑在 **MultiThreadedExecutor** 上（见 CMakeLists.txt 的 EXECUTOR MultiThreadedExecutor，
//   生成 main 里是 exec.add_node(node) + exec.spin()），并把回调分成**三个 MutuallyExclusive 组**：
//     · tf_cb_group_        —— 只放 TF/状态定时器（publishTimerCallback）；
//     · align_cb_group_     —— 点云订阅（重活 GICP align）；**/initialpose 的"应用"也在这条路上**；
//     · init_pose_cb_group_ —— 只放 /initialpose 订阅（**纯交接**：拷一份原始 msg + 置位，µs 级）。
//   组间并行 ⇒ 单帧 align ~350 ms（实测 327~389 ms）也阻塞不了 50 Hz 的 TF 发布（改前实测 /tf 只有 2.3 Hz）。
//   **为什么 /initialpose 有自己的组**（2026-10-05 二次修复，见 docs/gicp_initialpose_latency.md）：
//     它原来与点云同组，理由是"人工初值的写入必须与 align 的读改写串行"；但代价是点击要排在
//     在飞的 align 后面 —— PCL 后端 400~490 ms/帧 @10 Hz 时实测一次点击 **13.2 s 才被处理**、
//     另一次 **14 s 内完全没被处理**（空闲时 ~15 ms）。
//     现在把"写 map→odom"这一步整体搬进 **点云回调的帧首**（consumePendingInitialPose()）：
//     串行性由 align 组自己保证（同一条回调序列，比"组内互斥"更严格），交接回调只剩 POD 拷贝
//     ⇒ 可以安全地搬出 align 组，点击的"生效"只等下一帧点云开头（≈ 一个 align 周期）。
//   因此**跨组共享的成员必须在 mutex_ 下访问**（逐项清单见下面"状态"块）；
//   GICP 对象 / first_align_done_ / last_tf_error_ 只由 align 组内的回调触碰 —— 组内互斥保证
//   它们仍是单线程访问（PCL GICP 自身不是线程安全的，绝不允许并发 align）。
//
// 点类型选择：pcl::PointXYZ —— 只用 XYZ，两个理由：
//   ① PCL 1.12.1 的 GICP **自己算协方差**（pcl/registration/impl/gicp.hpp:51-125 的
//      computeCovariances 只读 x/y/z，累计 mean/cov 后做 SVD），normal_*/curvature/intensity
//      这些字段它一个都不读 ⇒ 用 PointNormal / PointXYZINormal 只是在浪费内存和 IO。
//      small_gicp 同理（estimate_covariances_omp 只用 x/y/z）。
//   ② 我们的 PCD 资产字段并不统一：RMUL2026.pcd = x y z intensity normal_x..curvature；
//      RMUC.pcd = normal_x..z x y z _（**没有 intensity**）。用 PointXYZ 三种资产都能读；
//      若用 PointXYZI 去读 RMUC.pcd，PCL 会因缺 intensity 报 "Failed to find match for field"
//      并整帧失败。实时点云同理（不依赖 intensity）。
//   ⇒ PointT / PointCloudT 的定义已移到 registration_backend.hpp（两个后端共用同一份）。

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "gicp_registration/registration_backend.hpp"

namespace gicp_registration
{

class GicpNode : public rclcpp::Node
{
public:
  explicit GicpNode(const rclcpp::NodeOptions & options);

private:
  // ---- 回调 ----
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  // /initialpose 的**交接**（handoff；跑在自己的 init_pose_cb_group_ 里）：
  // 只把原始 msg 的 pose/covariance/header 存进 pending_initial_pose_ 并置位，然后立刻返回 ——
  // 没有 TF 查询、没有变换、没有状态写入、没有 reset（唯一日志 = 一行 1 Hz 限频 INFO）。
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void publishTimerCallback();

  // ---- 工具 ----
  // /initialpose 的**应用**（apply）：把交接槽里的初值变成 map→odom（TF 查询 + 组合 + 状态写入 +
  // no_improve 计数清零 + 日志 + 一条 ~/pose）。**只在 pointcloudCallback 的帧首调用** ⇒ 与 align 的
  // 读改写天然串行（同一 MutuallyExclusive 组、同一条回调序列），PCL/small_gicp 对象照旧只在这条路上被碰。
  // 返回 true = 本帧开头成功应用了一次初值；TF odom→base 查不到时**丢弃**本次初值并 WARN（与改动前一致）。
  bool consumePendingInitialPose();
  // 体素下采样：**已移到 registration_backend.hpp 的 voxelDownsample()**（两个后端共用同一实现，
  // leaf_size 仍由调用方给：地图 target 用 voxel_leaf_size_，实时点云 source 用 voxel_leaf_size_scan_）。
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
  // ---- 配准后端（2026-10-05 新增）----
  // `backend` = "pcl"（默认，今天的已验证路径）| "small_gicp"（vendored koide3/small_gicp）。
  // 未识别的取值 ⇒ WARN + 退回 pcl（**绝不静默换成未验证路径**）。
  RegistrationBackendKind backend_kind_ = RegistrationBackendKind::PCL;
  std::string backend_param_raw_;  // 参数原值（banner 回显，便于确认 launch/命令行覆盖生效）
  // **仅 small_gicp**：OpenMP 线程数（PCL 路径单线程，该键对它无影响）。
  //   注意与节点自己的 MultiThreadedExecutor 是两套线程：align 回调组是 MutuallyExclusive
  //   ⇒ 同一时刻只有一个 align 在跑，small_gicp 的 num_threads 只在这一个 align 内部并行。
  int small_gicp_num_threads_;
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
  PointCloudT::Ptr map_cloud_;  // 已体素下采样、已去 NaN（后端 target）
  // 可切换配准后端（backend 参数）：pcl = pcl::GeneralizedIterativeClosestPoint（默认，行为与改动前一致）；
  // small_gicp = koide3/small_gicp 的 Registration<GICPFactor, ParallelReductionOMP>。
  // **非线程安全** ⇒ 与原来的 gicp_ 成员一样，只在 align_cb_group_ 内被触碰（组内互斥保证不并发 align）。
  std::unique_ptr<RegistrationBackend> backend_;
  // small_gicp 后端的目标侧一次性预处理耗时（ms；setTarget 里量，banner 打印）。
  // PCL 路径恒为 0（它的目标协方差按 PCL 原行为在**首帧 align** 里惰性计算 ⇒ 首帧明显变慢）。
  double target_prep_ms_ = 0.0;

  // ---- 状态（mutex_ 保护） ----
  // mutex_ 的职责（2026-10-05 逐项审计，MT executor + 多回调组之后这是正确性的关键）：
  //   **定时器组（publishTimerCallback / publishPose 复用戳）、交接组（initialPoseCallback）与 align 组
  //   （pointcloudCallback / consumePendingInitialPose）都会碰的成员，一律在 mutex_ 下读写**：
  //     · T_map_odom_ / estimate_valid_        —— align 写、定时器读（缓存的 map→odom）
  //     · last_tf_stamp_ / last_tf_stamp_valid_—— 定时器写、align 组（publishPose）读
  //     · cloud_seen_ / last_cloud_stamp_      —— align 写、定时器读（点云陈旧度 WARN）
  //     · accepted_cycles_ / total_cycles_ / no_improve_cycles_ / param_init_pending_
  //                                            —— align 写、定时器读（[status] 行 / 状态机）
  //     · last_align_ms_ / last_score_ / last_source_points_ —— align 写、定时器读（可观测量）
  //     · last_status_log_tp_ / status_log_ever_printed_     —— 定时器读写（~1 Hz 限频）
  //     · last_tf_error_                       —— lookupTf 写（加锁）、日志读（lastTfError() 加锁）
  //     · pending_initial_pose_ / pending_initial_pose_valid_ —— **交接组写、align 组消费**
  //       （2026-10-05 新增：唯一跨"人工输入"与 align 的共享状态；生产者只做 POD 拷贝 + 置位，
  //        消费者在帧首取出并清位 ⇒ 后续所有 TF/组合/写状态都只发生在 align 组内部）
  //   **故意的例外**：三个健康话题（~/pose、~/fitness_score、~/converged）只由 align 组发布
  //   （唯一发布者 = pointcloudCallback / consumePendingInitialPose，两者都在 align 组内；
  //   交接回调 initialPoseCallback **一个话题都不发**）；rclcpp 的 Publisher::publish() 本身线程安全，
  //   而**持锁发布**会让一次阻塞的 DDS 写把定时器线程一起拖住 ⇒ 等于把刚修好的"饿死"换个姿势带回来
  //   ⇒ 发布**不进状态锁**，靠"单一回调组拥有"这条不变量保证。
  //   同理 tf_broadcaster_->sendTransform 只在定时器组里调用（它不与 align 组共享 publisher）。
  //   2026-10-05 新增的 ~/small_gicp_error 也是 align 组独占（且只在 backend=small_gicp 时创建）。
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

  // ---- /initialpose 交接槽（2026-10-05 二次修复；mutex_ 保护）----
  // 生产者 = initialPoseCallback（init_pose_cb_group_，与 align 并行）；消费者 = consumePendingInitialPose()
  // （align 组、点云回调帧首）。槽里只放**原始 payload**：不做 TF 查询、不做变换、不碰 map→odom
  // ⇒ 交接是 O(POD 拷贝)，这也是"点击不再被 align 饿死"的全部秘密。
  struct PendingInitialPose
  {
    geometry_msgs::msg::PoseWithCovariance pose;  // 原始 pose + covariance（本节点不用协方差，照样带着）
    std_msgs::msg::Header header;                 // 原始 header：stamp 用于 TF 精确查询，frame_id 仅记录
  };
  PendingInitialPose pending_initial_pose_;
  bool pending_initial_pose_valid_ = false;  // true = 有一份待应用的初值（后到者覆盖先到者）

  // ---- 可观测量（~1 Hz 状态行用）：A/B 时要看**每帧配准耗时**与**评分量级** ----
  double last_align_ms_ = 0.0;                                    // 最近一帧 backend_->align() 墙钟耗时
  double last_score_ = std::numeric_limits<double>::quiet_NaN();  // 最近一帧 fitness score（m²）
  size_t last_source_points_ = 0;                                 // 最近一帧下采样后的源点数
  std::chrono::steady_clock::time_point last_status_log_tp_{};    // 状态行限频（~1 Hz）
  bool status_log_ever_printed_ = false;

  // ---- ROS 接口 ----
  // 回调组（2026-10-05）：定时器 / align / /initialpose 交接分属三个 MutuallyExclusive 组，
  // 由 MultiThreadedExecutor 并行调度（改前是单线程 executor 的默认组 ⇒ /tf 掉到 2 Hz）。
  rclcpp::CallbackGroup::SharedPtr tf_cb_group_;         // 只放 publish_timer_（TF/状态发布）
  rclcpp::CallbackGroup::SharedPtr align_cb_group_;      // 点云订阅（GICP align + 初值应用）
  rclcpp::CallbackGroup::SharedPtr init_pose_cb_group_;  // 只放 /initialpose（纯交接，µs 级）
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr converged_pub_;
  // **仅 backend=small_gicp**（其余情况下恒为 nullptr、话题不存在）：
  // small_gicp 的 RegistrationResult::error **原值**（Σ 0.5·rᵀΩr，Mahalanobis 加权、无量纲）。
  // 与 ~/fitness_score（PCL 等价：内点平均平方距离 m²）**不是一回事**，故意分成两条话题，
  // 免得有人拿它去比 max_fitness_score 阈值（见 docs/gicp_backend_small_gicp.md 的"指标语义"）。
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr small_gicp_error_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace gicp_registration

#endif  // GICP_REGISTRATION__GICP_REGISTRATION_HPP_
