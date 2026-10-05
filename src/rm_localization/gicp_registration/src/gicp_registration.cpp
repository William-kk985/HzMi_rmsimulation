// GICP 重定位节点（localization:=gicp 槽）—— 只发布 map→odom。
//
// 数据流（每帧）：
//   实时点云(/livox/lidar/pointcloud, BEST_EFFORT) → 去 NaN → 体素下采样(source, leaf=voxel_leaf_size_scan_)
//   TF: T_sensor←odom（点云时间戳）                 —— 把 LIO 的里程计增量接到配准结果上
//   初值 guess = T_map←odom(上次) · T_odom←sensor   —— 用里程计做运动预测 ⇒ GICP 连续跟踪
//   GICP(source → map) → T_map←sensor_est
//   T_map←odom = T_map←sensor_est · T_sensor←odom  → 接受/拒绝 → TF / ~/pose / ~/fitness_score
//
// **可切换配准后端（backend 参数，2026-10-05）**：`pcl`（默认，本文档描述的就是它）|
//   `small_gicp`（vendored koide3/small_gicp，MIT，pinned commit 57c1106…= v1.0.1）。
//   两个后端共用同一份参数/同一份体素下采样/同一套接受判据，只把"目标预处理 + 单帧 align + 评分"
//   交给 gicp_registration::RegistrationBackend（src/registration_backend.cpp）分流
//   ⇒ A/B 的唯一变量就是 backend。**~/fitness_score 的语义不因后端改变**（见 registration_backend.hpp）。
//   切回 PCL 只需 `backend: pcl`（或运行时 -p backend:=pcl）——契约、话题、时间戳、QoS 全不变。
//   详细对照、实测数据与风险：docs/gicp_backend_small_gicp.md。
//
// **两级下采样（two-tier leaf，2026-10-05）**：先验地图（target）用粗 leaf（voxel_leaf_size_，
//   默认 0.10 m）——建地图点云本身密度不均、粗一点省内存/CPU 且给 GICP 稳定的平面协方差；
//   实时点云（source）用细 leaf（voxel_leaf_size_scan_，默认 0.05 m）——单帧只有几千点，
//   细一点才能保住几何细节与配准精度。recipe 来自 COD 2025 的 small_gicp_relocalization
//   （global_leaf_size 0.25 / registered_leaf_size 0.05 / max_dist_sq 2.5 / num_threads 8），
//   我们把地图侧按本仓库资产密度收到 0.10 m（0.25 m 时 RMUL2026 只剩 2438 个 target 点，
//   实测 map→odom 在 30 s 窗口里漂 ~8 cm、settled=False）。
//
// 行为约定（docs/localization_slots.md §1）：
//   ① **必须有初值**：/initialpose（RELIABLE，RViz 2D Pose Estimate，语义 = map 系下机器人位姿）
//      或 initial_pose 参数；没有初值就**不发 TF**，并在日志里说明原因（不静默）。
//   ② 每帧配准，但只在"收敛且评分达标"时才更新 map→odom；不达标就沿用上一次估计，
//      并（限频 WARN + 连续 N 帧后一条明确的 WARN）说明"定位已失效"。
//   ③ 健康信号：~/fitness_score(m²，无内点时为 nan) + ~/converged(bool)。二者都是
//      transient_local ⇒ 后订阅的监控端也能立刻拿到最后一帧的值。
//   ④ **时间戳契约（tf_lookahead_sec）**：TF map→odom（以及 ~/pose）用
//      now()（节点时钟；use_sim_time=true 时 = 仿真时间）+ tf_lookahead_sec 盖戳 —— 与 AMCL 的
//      transform_tolerance 同语义。**不能用点云时间戳**：nav2 消费者在 now+margin 处查
//      map→odom，盖成"当前/过去"会让 buffer 里最新条目比请求时间旧 ⇒ tf2 抛
//      "Lookup would require extrapolation into the future" ⇒ follow_path 每周期 abort。
//
// **并发契约（2026-10-05 新增，治"align 饿死 TF 定时器"；同日二次修复治"align 饿死 /initialpose"）**：
//   两级 leaf 变细后单帧 align 实测 ~350 ms（327~389 ms），而 TF 定时器原来与点云订阅挤在**单线程
//   executor** 的同一个默认回调组里 ⇒ align 期间定时器排不上队：节点独占机器时实测 `/tf` 上的
//   map→odom 只剩 **~2 Hz**（TF 条数 ≈ 采纳帧数）⇒ 上面 ④ 的 tf_lookahead_sec=0.45 余量被吃满
//   ⇒ 整栈又见 "extrapolation into the future"（"刚修好的洞在别处漏水"）。修法**不改契约、只改调度**：
//     · 可执行文件跑 **MultiThreadedExecutor**（CMakeLists.txt 的 `EXECUTOR MultiThreadedExecutor`；
//       生成的 main 就是 exec.add_node(node) + exec.spin()）；
//     · **TF/状态定时器独占 tf_cb_group_**（MutuallyExclusive），**点云订阅（重活）放 align_cb_group_**
//       （另一个 MutuallyExclusive），**/initialpose 放第三个 init_pose_cb_group_** ⇒ 三组由 MT executor
//       并行调度，一次 350 ms 的 align 再也阻塞不了 50 Hz 的 TF 定时器；
//     · `/initialpose` 的**调度位置**（2026-10-05 二次修复）：原来它与点云**故意同组**，理由是
//       "人工初值的写入必须与 align 的读改写串行"；但同组 = 点击要排在在飞的 align 后面，
//       PCL 后端 400~490 ms/帧 @10 Hz 时实测**一次点击 13.2 s 才被处理**、另一次 **14 s 内完全没被处理**
//       （空闲时 ~15 ms；见 docs/gicp_backend_small_gicp.md §6.5 与 docs/gicp_initialpose_latency.md）。
//       现在改成 **handoff + apply**：
//         - initialPoseCallback（init_pose_cb_group_）= **只做交接**：在 mutex_ 下拷一份**原始**
//           pose/covariance + header(stamp/frame) 进 pending_initial_pose_ 并置位，然后立刻返回
//           （没有 TF 查询、没有变换、没有 reset、不发话题；唯一日志 = 一行 1 Hz 限频 INFO）；
//         - consumePendingInitialPose()（**只在点云回调帧首调用**，仍在 align_cb_group_ 内）= 真正的
//           "应用"：TF odom→base 查询、T_map_odom = T_map_base·T_odom_base、状态写入、
//           no_improve_cycles_ 清零、日志、一条 ~/pose。
//       ⇒ 串行性不但保住，而且更严格（写入与 align 的读改写现在是**同一条回调序列**，不再依赖
//         "组内互斥 + 两个不同回调"）；PCL/small_gicp 对象 / first_align_done_ 依旧只在这条路上被碰。
//         代价：点击的"生效"挂在**下一帧点云开头**（≈ 一个 align 周期）⇒ 点云完全不来时点击会**排队**
//         （旧代码是当场生效）。TF 可用性的判定时刻也从"点击时刻"挪到"下一帧开头"（≤ 一个 align 周期，
//         只会让原本会被丢弃的点击更可能被采纳）。
//     · 跨组共享的成员一律在既有 mutex_ 下读写（逐项清单见 .hpp 的"状态"块），本轮新增的唯一共享项是
//       交接槽 pending_initial_pose_ / pending_initial_pose_valid_；三个健康话题只由 align 组发布，
//       故意**不**持状态锁发布 —— 一次阻塞的 DDS 写会把定时器线程一起拖住，等于把"饿死"带回来。

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
#include <pcl/io/pcd_io.h>
#include <pcl/types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace gicp_registration
{
namespace
{
// TF 查询超时（秒）：两档都故意很短 —— 本节点的 TF 查询**只发生在 align 回调组里**
// （pointcloudCallback 自身 + 它帧首的 consumePendingInitialPose + publishPose），而该组是
// MutuallyExclusive ⇒ 一次长等待会直接推迟下一帧配准（点云 KEEP_LAST(1)，等 10 s 等于丢 10 s 的定位）。
// （2026-10-05 二次修复后，交接回调 initialPoseCallback 里**没有** TF 查询 —— 那正是它被饿死的根源之一。）
// 注：TransformListener 自带专用线程（tf2_ros::TransformListener 构造时创建
// MutuallyExclusive 回调组 + SingleThreadedExecutor + dedicated_listener_thread_），
// 所以 TF 的**接收**不受本节点 executor 影响；这里短的只是"查不到就快点失败"。
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

  // ---- 两级体素下采样（two-tier，2026-10-05）----
  // 键名 voxel_leaf_size **故意保留**（向后兼容既有 launch/参数覆盖），但语义收窄为
  // "**先验地图 / GICP target** 的 leaf"：默认 0.25 → 0.10。
  //   · 0.25 时 RMUL2026.pcd 只剩 2438 个 target 点（实测启动 banner），几何太稀疏 ⇒
  //     GICP 的目标协方差（KNN=20）被拉平、最近邻配对噪声大 ⇒ map→odom 漂（实测 30 s ~8 cm）。
  //   · 0.10 时 target 点数上一个量级（**实测 12450**，见下方启动 banner），代价是每帧 KNN/协方差更贵。
  //   依据：COD 2025 relocalizer small_gicp_relocalization 的 global_leaf_size（先验 PCD 侧）。
  voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.10);
  // 新键：**实时点云 / GICP source** 的 leaf，默认 0.05（细一档）。
  //   · 本节点原来用地图的 leaf 给实时点云下采样（0.25 m）——单帧被砍到几百点，配准精度被自己掐死；
  //   · 0.05 对应 COD small_gicp_relocalization 的 registered_leaf_size: 0.05。
  // 两级 leaf 的取值理由逐条写在 config/gicp_registration_sim.yaml 里。
  voxel_leaf_size_scan_ = declare_parameter<double>("voxel_leaf_size_scan", 0.05);
  // 1.0 → 1.5 m：对齐 COD 的 max_dist_sq 2.5（√2.5 ≈ 1.58 m，PCL 的对应点距离门限），
  // 给初值（LIO 里程计递推）多留收敛余量；注意 getFitnessScore 用的球半径同步变大
  // （1.5² = 2.25 m² vs 原来 1.0² = 1.0 m²）⇒ fitness 量级会整体上移，阈值需按实测量级重看。
  max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.5);
  // 2026-10-05：32 → 16（兜底默认与 config/gicp_registration_sim.yaml 保持一致）。
  // 依据：32 次迭代时本机单节点实测稳态 align = 327~389 ms（首个 2460 ms，含目标协方差预计算）；
  // GICP 通常 10~20 次内收敛，32 次属于过量迭代。
  // **实测提醒（别把它当"单帧成本砍半"）**：同一合成扫描下 maximum_iterations = 1/4/16/32/64 的
  // align 分别是 363/475/486/471/427 ms（噪声量级、非单调），fitness 全部 0.00123 m²。
  // 原因见 PCL 1.12.1 impl/gicp.hpp:420（`while (!converged_)`）与 :496（`nr_iterations_ >= max_iterations_
  // || delta < 1`）：
  // 跟踪场景（初值来自里程计递推、上一帧已收敛）里 delta 判据远早于 16 次就成立 ⇒ 上限不生效，
  // 每帧成本由固定开销（源协方差 KNN + 最近邻 + getFitnessScore）主导。
  // 保留 16 的理由：上限更低不会更慢/更差（实测一致），且能给"初值差的场景"的单帧耗时封顶。
  // 迭代数是"CPU ↔ 精度"旋钮：若 ~/fitness_score 变差 / ~/converged 掉 false，调回 32（config 同）。
  maximum_iterations_ = declare_parameter<int>("maximum_iterations", 16);
  transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 5.0e-4);
  rotation_epsilon_ = declare_parameter<double>("rotation_epsilon", 2.0e-3);
  correspondence_randomness_ = declare_parameter<int>("correspondence_randomness", 20);
  maximum_optimizer_iterations_ = declare_parameter<int>("maximum_optimizer_iterations", 20);
  // ---- 配准后端（2026-10-05 新增；契约见 docs/gicp_backend_small_gicp.md）----
  //   "pcl"        = pcl::GeneralizedIterativeClosestPoint（**默认**；今天已验证的那条路，行为不变）
  //   "small_gicp" = vendored koide3/small_gicp（MIT，pinned commit，多线程 GICP）
  // 未识别的取值 ⇒ WARN + 退回 pcl：**绝不静默换成未验证路径**。
  backend_param_raw_ = declare_parameter<std::string>("backend", "pcl");
  {
    bool ok = false;
    backend_kind_ = parseRegistrationBackendKind(backend_param_raw_, ok);
    if (!ok) {
      RCLCPP_WARN(
        get_logger(),
        "backend='%s' 不是已知取值（pcl | small_gicp）⇒ 退回默认 'pcl'（已验证路径）",
        backend_param_raw_.c_str());
      backend_kind_ = RegistrationBackendKind::PCL;
      backend_param_raw_ = "pcl";
    }
  }
  // 仅 small_gicp 生效：OpenMP 线程数（= COD small_gicp_relocalization 的 num_threads 同一个键）。
  // 默认 4（COD 用 8）：本节点与 LIO/nav2/Gazebo 同机跑，28 核上留余量比抢核更稳。
  // 实测（无 -O 构建、target 12450/source 3701）：1/2/4/8 线程 = 559/412/206/205 ms（median），
  // 4 线程即可（8 线程无进一步收益）；对照 pcl 后端同条件 387 ms ⇒ 约 1.9×。详见 docs。
  small_gicp_num_threads_ = declare_parameter<int>("small_gicp_num_threads", 4);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  // 与 AMCL 的 transform_tolerance 同语义：把 map→odom（和 ~/pose）盖成**未来**时间戳，
  // 保证 nav2 消费者在 now+margin 处能查到。
  //   过小 ⇒ tf2 抛 "Lookup would require extrapolation into the future"（最新条目比请求时间旧）
  //           ⇒ MPPI 的 transformPose 失败 ⇒ follow_path 每周期 abort、BT 反复恢复；
  //   过大 ⇒ 位姿被外推过头（消费端认为"车已经在未来位置上"）⇒ 高速下表现为定位滞后/猛修正、
  //           转弯时横向甩动。AMCL 默认 transform_tolerance=0.3（本仓库 nav2_params_sim_base.yaml 同值）。
  tf_lookahead_sec_ = declare_parameter<double>("tf_lookahead_sec", 0.3);
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
  // 两个 leaf 都必须是正数（PCL VoxelGrid 的 leaf<=0 无意义）。本节点**不提供**"0 = 不下采样"档：
  // 实时点云与地图都不下采样会让单帧点数/协方差计算量失控（本节点已知 CPU 敏感，见 config 注释），
  // 所以 <=0 一律判非法、退回默认值并 WARN（与旧版对 voxel_leaf_size 的处理一致）。
  if (voxel_leaf_size_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "voxel_leaf_size=%.3f 非法（必须是正数）⇒ 用 0.10（先验地图/target leaf）",
      voxel_leaf_size_);
    voxel_leaf_size_ = 0.10;
  }
  if (voxel_leaf_size_scan_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "voxel_leaf_size_scan=%.3f 非法（必须是正数）⇒ 用 0.05（实时点云/source leaf）",
      voxel_leaf_size_scan_);
    voxel_leaf_size_scan_ = 0.05;
  }
  if (max_correspondence_distance_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "max_correspondence_distance=%.3f 非法（必须是正数）⇒ 用 1.5 m",
      max_correspondence_distance_);
    max_correspondence_distance_ = 1.5;
  }
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz=%.3f 非法 ⇒ 用 50.0", publish_rate_hz_);
    publish_rate_hz_ = 50.0;
  }
  if (maximum_iterations_ <= 0) {
    RCLCPP_WARN(get_logger(), "maximum_iterations=%d 非法 ⇒ 用 16", maximum_iterations_);
    maximum_iterations_ = 16;
  }
  // small_gicp_num_threads <= 0 ⇒ OpenMP 的 num_threads(0) 是未定义行为；退回 4（backend=pcl 时该键无意义）
  if (small_gicp_num_threads_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "small_gicp_num_threads=%d 非法（必须 >= 1）⇒ 用 4", small_gicp_num_threads_);
    small_gicp_num_threads_ = 4;
  }
  if (tf_lookahead_sec_ < 0.0) {
    // 负的前瞻 = 把 map→odom 盖成过去 ⇐ 正是本次要修的 bug（tf2 extrapolation）。
    RCLCPP_WARN(
      get_logger(),
      "tf_lookahead_sec=%.3f 非法（负值会让 map→odom 落后于消费者的查询时间，"
      "直接复现 'extrapolation into the future'）⇒ 用 0.3", tf_lookahead_sec_);
    tf_lookahead_sec_ = 0.3;
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
  map_cloud_ = voxelDownsample(map_finite, voxel_leaf_size_);  // 两级 leaf 之"粗"档：先验地图/target
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

  // ============================ 配准后端（backend 参数） ============================
  // 两个后端**共用同一份参数**（leaf、max_corr_dist、迭代/收敛阈值、协方差 KNN 逐项对应），
  // 只在这一处分流 ⇒ A/B 时唯一变量就是 backend（+ small_gicp_num_threads）。
  //   · pcl        ：pcl::GeneralizedIterativeClosestPoint —— **改动前逐行等同的配置**；
  //   · small_gicp ：Registration<GICPFactor, ParallelReductionOMP>（LM 优化器 + OpenMP 归约）。
  // 目标（先验地图）在这里一次性交给后端：
  //   · pcl        → setInputTarget（目标协方差按 PCL 原行为**首帧 align** 才算 ⇒ 首帧 ~1.5 s 起）；
  //   · small_gicp → 转点云 + 建 KdTree + 估计协方差（**构造期一次**，耗时量在 target_prep_ms_）。
  // 两者都只做一次，故启动期多出的这点时间不影响每帧 align 的 A/B 口径。
  {
    RegistrationBackendOptions backend_options;
    backend_options.kind = backend_kind_;
    backend_options.max_correspondence_distance = max_correspondence_distance_;
    backend_options.maximum_iterations = maximum_iterations_;
    backend_options.transformation_epsilon = transformation_epsilon_;
    backend_options.rotation_epsilon = rotation_epsilon_;
    backend_options.correspondence_randomness = correspondence_randomness_;
    backend_options.maximum_optimizer_iterations = maximum_optimizer_iterations_;
    backend_options.small_gicp_num_threads = small_gicp_num_threads_;
    backend_ = std::make_unique<RegistrationBackend>(backend_options);
    target_prep_ms_ = backend_->setTarget(map_cloud_);
  }

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
  // ---- 回调组（2026-10-05：把 50 Hz 的 TF 定时器与重配准解耦；同日二次修复：把 /initialpose 摘出来）----
  // 为什么要多组：单帧 align 实测 ~350 ms（327~389 ms）；原来定时器与点云订阅共用**单线程 executor 的
  // 默认回调组** ⇒ align 期间定时器排不上队，`/tf` 上的 map→odom 实测只剩 ~2 Hz ⇒ nav2 在
  // now+transform_tolerance 处查到的最新条目越来越旧 ⇒ "extrapolation into the future" 复发。
  // 现在：MultiThreadedExecutor（CMakeLists.txt 的 EXECUTOR）+ 下面三个 MutuallyExclusive 组
  // 并行调度 ⇒ 定时器再也不受 align 影响（契约、话题、QoS、时间戳语义全不变）。
  //   · tf_cb_group_        ：只放 TF/状态定时器（组类型 MutuallyExclusive ⇒ 定时器回调不自我重叠）；
  //   · align_cb_group_     ：点云订阅（GICP 重活）+ **初值的应用**（consumePendingInitialPose，帧首）；
  //   · init_pose_cb_group_ ：只放 /initialpose 订阅（**纯交接**：mutex_ 下拷一份原始 msg + 置位）。
  // /initialpose 为什么能、也必须搬出 align 组（2026-10-05 二次修复）：
  //   原来与点云同组是为了"初值写入与 align 的读改写串行"；现在**写入本身搬进了点云回调帧首**
  //   ⇒ 串行性由 align 组自己保证（更严格：同一条回调序列），组内排队就不再需要 ——
  //   而正是那条排队让点击在 align 饱和时被饿死（实测 13.2 s / 14 s 内未处理，见 docs）。
  //   为什么不塞进 tf_cb_group_：那个组的契约是"50 Hz 的 TF/状态发布绝不被拖慢"，而 /initialpose 是
  //   人手点的话题（RViz 可能突发重发）⇒ 不该与它有调度耦合；独立组最干净（见 docs 的替代方案对比）。
  tf_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  align_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  init_pose_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 点云：BEST_EFFORT（发布端 livox 用 SensorDataQoS；默认 RELIABLE 订阅会永远收不到
  // —— 这正是 icp_registration 在 f033d96 之前"跑起来但没数据"的原因）。KEEP_LAST(1)：
  // 配准慢时宁可丢旧帧，也不要排队处理过期点云。
  rclcpp::SubscriptionOptions cloud_sub_options;
  cloud_sub_options.callback_group = align_cb_group_;  // 重活组：与 TF 定时器组并行
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&GicpNode::pointcloudCallback, this, std::placeholders::_1),
    cloud_sub_options);
  // /initialpose：**QoS 不变 = RELIABLE**（RViz 的 2D Pose Estimate 就是 RELIABLE 发的，
  // 不能共用点云的 SensorDataQoS/BEST_EFFORT；这是契约，不是实现细节）。
  // 变的是**回调组**：独立 init_pose_cb_group_ ⇒ 点击的"接收"不再排在在飞的 align 后面；
  // 真正的"应用"仍在 align 组内（点云回调帧首），串行性见文件顶部与 .hpp 的并发契约。
  rclcpp::SubscriptionOptions init_pose_sub_options;
  init_pose_sub_options.callback_group = init_pose_cb_group_;
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&GicpNode::initialPoseCallback, this, std::placeholders::_1),
    init_pose_sub_options);

  pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", rclcpp::QoS(1));
  fitness_score_pub_ = create_publisher<std_msgs::msg::Float64>(
    "~/fitness_score", rclcpp::QoS(1).transient_local());
  converged_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/converged", rclcpp::QoS(1).transient_local());
  // ~/small_gicp_error：**只在 backend=small_gicp 时创建**（PCL 路径下这条话题根本不存在 ⇒
  // "PCL 路径行为不变"包括"不多出话题"）。语义 = small_gicp RegistrationResult::error 原值，
  // **不是** m²、**不能**与 max_fitness_score 比较；与 ~/fitness_score 的分工见 .hpp 的成员注释。
  if (backend_kind_ == RegistrationBackendKind::SmallGicp) {
    small_gicp_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/small_gicp_error", rclcpp::QoS(1).transient_local());
  }

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // TF/状态定时器：**独占 tf_cb_group_** ⇒ 与 align_cb_group_ 并行（MT executor），
  // 单帧 350 ms 的 GICP 再也阻塞不了它。这也是本次"extrapolation"修复的关键一环。
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&GicpNode::publishTimerCallback, this),
    tf_cb_group_);

  // ============================ 启动摘要 ============================
  // A/B 要看的三件事都在这里：**两个 leaf**、**target 点数**（下采样后参与 GICP 的地图点数）、
  // 以及 max_correspondence_distance（它同时决定 getFitnessScore 的球半径 ⇒ 影响评分量级）。
  // 2026-10-05 追加：**当前生效的配准后端**（backend 参数；A/B 时第一眼就要能看到自己在跑哪条路）。
  RCLCPP_INFO(
    get_logger(),
    "\n===== gicp_registration 启动 =====\n"
    "  ★ 配准后端 backend=%s（参数原值 '%s'；可选 pcl | small_gicp，默认 pcl）\n"
    "     · pcl        = pcl::GeneralizedIterativeClosestPoint（单线程，已验证路径）\n"
    "     · small_gicp = koide3/small_gicp（MIT，vendored，pinned 57c1106/v1.0.1；"
    "OpenMP 多线程，num_threads=%d）\n"
    "     目标侧一次性预处理：%.1f ms%s\n"
    "  pcd_path=%s\n"
    "  地图点数（GICP target）：原始 %zu → 去 NaN %zu → 体素 %.3f m 后 %zu\n"
    "  两级下采样 leaf（来源：COD 2025 small_gicp_relocalization 的 global/registered_leaf_size）："
    "地图/target %.3f m（voxel_leaf_size）· 实时点云/source %.3f m（voxel_leaf_size_scan）\n"
    "  地图包围盒：min=(%.2f, %.2f, %.2f) max=(%.2f, %.2f, %.2f)"
    "（应覆盖机器人活动区；若小得离谱说明资产退化）\n"
    "  frames: map='%s' odom='%s' base='%s' laser(仅兜底)='%s'\n"
    "  pointcloud_topic=%s（SensorDataQoS / BEST_EFFORT）\n"
    "  GICP: max_corr_dist=%.3f m（≈COD max_dist_sq 2.5）, maximum_iterations=%d,"
    " transformation_epsilon=%.2e,\n"
    "        rotation_epsilon=%.2e, correspondence_randomness=%d, maximum_optimizer_iterations=%d\n"
    "  初值: use_initial_pose=%s initial_pose=[%s]%s\n"
    "  输出: TF %s→%s @%.1f Hz + ~/pose + ~/fitness_score + ~/converged%s"
    "（fitness warn=%.3f / accept=%.3f m²）\n"
    "  时间戳: TF 与 ~/pose 都用 now+%.2f s（tf_lookahead_sec，≈ AMCL transform_tolerance；"
    "点云时间戳不用来盖 TF）\n"
    "  并发: MultiThreadedExecutor + 三个回调组 —— TF/状态定时器组（%.1f Hz）‖ 点云组（GICP align）"
    "‖ /initialpose 交接组\n"
    "        ⇒ 单帧 align 再慢也不会饿死 TF 定时器（改前单线程：实测 /tf 只有 ~2 Hz）；"
    "/initialpose 只做交接（µs 级），\n"
    "          真正的应用在**下一帧点云开头**、与 align 串行（改前与点云同组：实测被饿死 13.2 s）\n"
    "  阈值: no_improve_cycles_warn=%d, stale_warn_sec=%.1f；"
    "状态行每 ~1 s 一条（含 align 耗时 ms 与 fitness score）\n"
    "  注：pcl 后端在首个 fitness score 之前要先做一次目标协方差预计算（target 越密越慢），"
    "可能耗时数秒；\n"
    "      small_gicp 后端的目标协方差已在启动时算完（见上面「目标侧一次性预处理」），首帧不额外慢。\n"
    "==================================",
    backend_->name(), backend_param_raw_.c_str(), small_gicp_num_threads_,
    target_prep_ms_,
    backend_kind_ == RegistrationBackendKind::PCL ?
    "（PCL 惰性：目标协方差在首帧 align 里算 ⇒ 首帧明显变慢）" :
    "（small_gicp：建 KdTree + 估协方差，构造期一次）",
    pcd_path_.c_str(), n_raw, n_finite, voxel_leaf_size_, n_map,
    voxel_leaf_size_, voxel_leaf_size_scan_,
    map_min.x, map_min.y, map_min.z, map_max.x, map_max.y, map_max.z,
    map_frame_id_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(), laser_frame_id_.c_str(),
    pointcloud_topic_.c_str(),
    max_correspondence_distance_, maximum_iterations_, transformation_epsilon_, rotation_epsilon_,
    correspondence_randomness_, maximum_optimizer_iterations_,
    use_initial_pose_ ? "true" : "false", init_pose_str.c_str(),
    use_initial_pose_ ? "（首帧点云 + TF odom→base 就绪时生效）" : "（已忽略）",
    map_frame_id_.c_str(), odom_frame_id_.c_str(), publish_rate_hz_,
    backend_kind_ == RegistrationBackendKind::SmallGicp ?
    " + ~/small_gicp_error（small_gicp 原生 error，**非 m²**）" : "",
    fitness_score_warn_, max_fitness_score_, tf_lookahead_sec_,
    publish_rate_hz_,
    no_improve_cycles_warn_, stale_warn_sec_);
}

// ============================ 工具 ============================

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
      // last_tf_error_ 是 std::string：跨回调共享（点云 / publishPose / 初值应用 —— 三者现在都在
      // align 组内，但定时器组的日志仍会读它），MT executor 下必须加锁写（读走 lastTfError() 快照）。
      std::lock_guard<std::mutex> lock(mutex_);
      last_tf_error_ = ex.what();
    }
  }
  try {
    // tf2::TimePointZero = "取最新可用"（用户点 /initialpose 的初值应用、点云时间戳查不到时的兜底）
    const auto tf = tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero, tf2::durationFromSec(kTfLatestTimeoutSec));
    out = transformToMatrix(tf.transform);
    used_latest = try_exact_stamp;  // 只有"本想用精确时间戳"时才值得提示
    return true;
  } catch (const tf2::TransformException & ex) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_tf_error_ = ex.what();
    return false;
  }
}

std::string GicpNode::lastTfError() const
{
  // 加锁快照：日志里打印 last_tf_error_ 一律走这里（不要裸读成员）。
  std::lock_guard<std::mutex> lock(mutex_);
  return last_tf_error_;
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
  // --- ⓪ 应用待处理的 /initialpose（2026-10-05：交接 → 应用的**唯一**消费点）---
  // 放在最开头（比点云转换还早）的两个理由：
  //   ① 点击的"生效"只等这一帧**开始**，不再等回调组空闲（旧设计要排在在飞的 align 后面，
  //      实测被饿死 13.2 s）；也就等于"延迟 ≈ 一个 align 周期"；
  //   ② 本帧之后可能因为点云非法/TF 查不到而提前 return，但点击**已经生效**（旧代码在点击回调里
  //      当场生效，语义一致，不会因为一帧坏点云把人的操作吞掉）。
  // 内部只做 TF odom→base 查询 + 组合 + 状态写入（与旧 initialPoseCallback 逐行等价），
  // 全程在本回调线程内 ⇒ 与下面 align 的读改写是同一条回调序列（串行性比"组内互斥"更强）。
  consumePendingInitialPose();

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

  // --- ② 去 NaN + 体素下采样（**两级 leaf 的细档**：实时点云/source 用 voxel_leaf_size_scan_，
  //         与地图 target 的 voxel_leaf_size_ 解耦；原来是两级共用地图 leaf ⇒ 单帧被砍得过稀） ---
  pcl::Indices finite_indices;
  PointCloudT::Ptr finite(new PointCloudT);
  pcl::removeNaNFromPointCloud(*raw, *finite, finite_indices);
  PointCloudT::Ptr source = voxelDownsample(finite, voxel_leaf_size_scan_);
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
      sensor_frame.c_str(), odom_frame_id_.c_str(), lastTfError().c_str());
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
        odom_frame_id_.c_str(), base_frame_id_.c_str(), lastTfError().c_str());
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

  // --- ⑥ 配准（backend 参数决定走 PCL 还是 small_gicp；两条路共用同一份参数与同一份下采样） ---
  if (!first_align_done_) {
    if (backend_kind_ == RegistrationBackendKind::PCL) {
      RCLCPP_INFO(
        get_logger(), "首次 GICP align（backend=pcl）：会先预计算目标（地图 %zu 点）协方差，"
        "可能耗时数秒…", map_cloud_->size());
    } else {
      RCLCPP_INFO(
        get_logger(), "首次 GICP align（backend=small_gicp，num_threads=%d）：目标（地图 %zu 点）的 "
        "KdTree/协方差已在启动时算完（%.1f ms）⇒ 首帧没有额外的预计算开销",
        small_gicp_num_threads_, map_cloud_->size(), target_prep_ms_);
    }
  }
  bool converged = false;
  double score = std::numeric_limits<double>::quiet_NaN();
  double backend_error = std::numeric_limits<double>::quiet_NaN();  // 仅 small_gicp：原生 error
  Eigen::Matrix4d T_map_sensor = guess;
  // 每帧配准耗时（ms）：两级 leaf 变细后这是最直接的 CPU 指标（A/B 要能看见它）。
  // 口径：**只包住 backend_->align()**（= setInputSource/预处理 + 迭代 + 评分），与两个后端一致。
  const auto align_t0 = std::chrono::steady_clock::now();
  try {
    // 后端内部做的事（两条路一一对应）：
    //   pcl        ：setInputSource（重算源协方差）→ align → hasConverged → getFitnessScore(r²)
    //   small_gicp ：源转点云 → 源协方差（OMP）→ Registration<GICPFactor,OMP>::align →
    //                converged → **PCL 等价 fitness**（另算，见 registration_backend.cpp）
    const RegistrationOutcome outcome = backend_->align(source, guess);
    converged = outcome.converged;
    score = outcome.fitness_score;  // 两个后端同定义：内点平均平方距离（m²）
    backend_error = outcome.backend_error;
    T_map_sensor = outcome.T_target_source;
    if (backend_kind_ == RegistrationBackendKind::SmallGicp && std::isfinite(backend_error)) {
      RCLCPP_DEBUG(
        get_logger(),
        "small_gicp 原生量：error=%.6f（Σ0.5·rᵀΩr，**非 m²**）, num_inliers=%zu, iterations=%zu",
        backend_error, outcome.num_inliers, outcome.iterations);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "GICP align 抛异常（%s）⇒ 本帧不更新 map→odom", ex.what());
  }
  const double align_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - align_t0).count();
  first_align_done_ = true;
  if (!std::isfinite(score) || score >= std::numeric_limits<double>::max()) {
    score = std::numeric_limits<double>::quiet_NaN();  // "无有效内点" → nan（~/fitness_score 可见）
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    total_cycles_++;
    // 可观测量：供 ~1 Hz 状态行 / 未采纳 WARN 打印（align 耗时 + score + 源点数）
    last_align_ms_ = align_ms;
    last_score_ = score;
    last_source_points_ = source->size();
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
      get_logger(),
      "GICP OK：score=%s m², converged=true, align=%.1f ms, source=%zu 点, map→odom=%s"
      "（%d/%d 帧被接受）",
      scoreToStr(score).c_str(), align_ms, source->size(), poseToStr(T_map_odom_cur).c_str(),
      n_acc, n_tot);
  } else {
    int n = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      n = ++no_improve_cycles_;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GICP 本帧未被采纳（converged=%s, score=%s m², align=%.1f ms, source=%zu 点, "
      "accept 阈值=%.3f）⇒ map→odom 保持上一次估计（连续 %d 帧；TF 仍按上次估计发布）",
      converged ? "true" : "false", scoreToStr(score).c_str(), align_ms, source->size(),
      max_fitness_score_, n);
    if (n == no_improve_cycles_warn_) {
      RCLCPP_WARN(
        get_logger(),
        "★★ GICP 已连续 %d 帧未被采纳：map→odom 一直沿用旧值 ⇒ **定位实际已失效**"
        "（RViz 里车会停住不动或慢慢漂）。排查顺序：\n"
        "   ① 初值：用 RViz 2D Pose Estimate 给正确位置（或 initial_pose 参数）"
        "——ICP 族没有初值必然不收敛；\n"
        "   ② 看 ~/fitness_score 的实际量级再调 max_fitness_score / fitness_score_warn"
        "（单位 m² = 内点平均平方距离；注意半径已随 max_correspondence_distance=%.2f m 变大，"
        "量级会整体上移）；\n"
        "   ③ 两个 leaf（voxel_leaf_size=%.3f 地图 / voxel_leaf_size_scan=%.3f 实时点云，"
        "太密则 align 耗时涨、太稀则不收敛）、max_correspondence_distance；\n"
        "   ④ 确认 PCD 与 map/<world>.pgm 同源同系（否则先验地图本身就错位）。",
        n, max_correspondence_distance_, voxel_leaf_size_, voxel_leaf_size_scan_);
    }
  }
  publishHealth(true, score, accepted && score <= fitness_score_warn_);
  // small_gicp 后端：**另发**一条 small_gicp 原生 error（无内点/未收敛时为 nan）。
  // 只在这条路里发布（PCL 路径下该 publisher 是 nullptr、话题不存在）⇒ 两个后端的
  // ~/fitness_score **定义完全一致**，不会被原生 error 悄悄改写。
  if (small_gicp_error_pub_) {
    std_msgs::msg::Float64 raw;
    raw.data = backend_error;
    small_gicp_error_pub_->publish(raw);
  }

  // --- ⑧ 调试位姿：map 系机器人位姿（≈ /amcl_pose 语义，便于 A/B 对照） ---
  // 戳不再用点云时间戳（那是"过去"），而是与 TF 同一条 lookahead 戳 ⇒ 见 publishPose 注释。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    T_map_odom_cur = T_map_odom_;
    valid_cur = estimate_valid_;
  }
  if (valid_cur) {
    publishPose(T_map_odom_cur, accepted ? score : -1.0);
  }
}

void GicpNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // ================== 廉价交接（handoff）：只拷贝 + 置位，然后立刻返回 ==================
  // 本回调跑在**独立**的 init_pose_cb_group_（2026-10-05 二次修复），**故意不做**任何"应用"动作：
  //   · 没有 TF 查询（旧代码在这里查 odom→base，超时 0.05 s×2 档，且要排在在飞的 align 后面）；
  //   · 没有变换/组合、没有 map→odom 写入、没有 no_improve 清零、不发任何话题；
  //   · 唯一日志 = 一行 1 Hz 限频 INFO（交接是热路径：RViz 连点不该刷屏）。
  // 存的是**原始** payload：pose/covariance 一个字段不改，header 连 stamp 一起带走 ——
  // 应用的语义（含"点云/TF 变化"的判定时刻）留给 consumePendingInitialPose()，与旧实现逐行等价。
  // 后到者覆盖先到者：连续点击的**终态**与旧实现一致（最后一次点击生效），只是中间那几次不再
  // 各写一次 map→odom / 各发一条 ~/pose（旧实现每次点击都会各写一条，属实现细节而非契约）。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_initial_pose_.pose = msg->pose;      // 原始 pose + covariance（本节点不用协方差，照样带着）
    pending_initial_pose_.header = msg->header;  // 原始 stamp（TF 精确查询用）+ frame_id（仅记录）
    pending_initial_pose_valid_ = true;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "/initialpose 收到（handoff）：map 系机器人位姿 x=%.3f y=%.3f z=%.3f frame='%s' "
    "⇒ 将在**下一帧点云开头**应用（TF 查询/组合/状态写入都在那一步，与 align 串行）",
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
    msg->header.frame_id.c_str());
}

bool GicpNode::consumePendingInitialPose()
{
  // ---------- ① 取出交接槽：锁内只做 POD 拷贝 + 清标志（µs 级） ----------
  geometry_msgs::msg::PoseWithCovariance pose_raw;
  std_msgs::msg::Header header_raw;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_initial_pose_valid_) {
      return false;
    }
    pending_initial_pose_valid_ = false;  // 取出即消费：TF 查不到也**不再重试**（与改动前一致）
    pose_raw = pending_initial_pose_.pose;
    header_raw = pending_initial_pose_.header;
  }

  // ---------- ② 以下与改动前的 initialPoseCallback **逐行等价**（语义一个不改） ----------
  // /initialpose 的语义（与 AMCL 一致）= map 系下**机器人（base_frame）**的位姿
  // ⇒ T_map←odom = T_map←base · T_base←odom，其中 T_base←odom 由 TF 提供（LIO 的 odom→base）。
  // 这样 GICP 不必自己猜雷达外参，也没改变用户对 RViz 的用法。
  // 注：`use_initial_pose` 参数**与本路径无关**（它只管 initial_pose 参数那条惰性初始化）
  // ⇒ /initialpose 在 use_initial_pose=false 时同样有效（与改动前一致）。
  const Eigen::Matrix4d T_map_base = poseToMatrix(pose_raw.pose);
  const rclcpp::Time stamp(header_raw.stamp, get_clock()->get_clock_type());
  Eigen::Matrix4d T_odom_base;
  bool used_latest = false;
  // 精确时间戳查不到会自动退到"最新可用"（用户点击的语义本来就是"此刻"）
  if (!lookupTf(odom_frame_id_, base_frame_id_, stamp, true, T_odom_base, used_latest)) {
    RCLCPP_WARN(
      get_logger(), "/initialpose 收到，但 TF %s←%s 查不到（%s）⇒ 忽略本次初值（LIO/TF 未就绪？）",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), lastTfError().c_str());
    return false;  // 不改任何状态（estimate_valid_ 不会因此变 true ⇒ 仍然不发 map→odom）
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
  // ~/pose 的戳由 publishPose 统一盖成"与 TF 相同的那条 lookahead 戳"
  // （不再用点击时刻：点击时刻是"过去"，且与 TF 不一致）。
  // 这里**立刻**发一条（与改动前 /initialpose 的即时反馈一致；score=-1 = 本次没有 fitness score）；
  // 本帧末尾 pointcloudCallback 还会按帧的正常路径再发一条（值可能已被本帧 align 精修）。
  publishPose(T_map_odom, -1.0 /*本次没有 fitness score*/);
  return true;
}

// ============================ 发布 ============================

void GicpNode::publishTimerCallback()
{
  // 运行上下文（2026-10-05）：本回调跑在**独立的 tf_cb_group_**（MutuallyExclusive）里，
  // 由 MultiThreadedExecutor 与 align_cb_group_（GICP 重活）**并行**调度 ⇒ 单帧 align 350 ms
  // 也饿不死 50 Hz 的 TF/状态发布（改前实测 /tf 掉到 ~2 Hz，正是 extrapolation 的根因）。
  // ⇒ 本回调读到的每个共享成员都必须在 mutex_ 下取快照（逐项清单见 .hpp 的"状态"块）。
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
    // 按 publish_rate_hz 重复发布：nav2 每帧都查 map→odom，不能只依赖配准帧率。
    // 戳用 now()+tf_lookahead_sec（**不是**点云时间戳）——见 publishTf。
    const rclcpp::Time tf_stamp = publishTf(T_map_odom);
    std::lock_guard<std::mutex> lock(mutex_);
    last_tf_stamp_ = tf_stamp;
    last_tf_stamp_valid_ = true;  // ~/pose 复用这条戳 ⇒ pose 与 TF 的时间戳一致
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

  // ---------------- ~1 Hz 状态行（2026-10-05 新增，纯可观测量） ----------------
  // A/B 要一眼看到四件事：**采纳率**（定位是否在更新）、**align 耗时 ms**（两级 leaf 变细后的 CPU 代价）、
  // **fitness score 量级**（半径 1.5 m 后量级会整体上移）、以及 source/target 点数与两个 leaf。
  // 限频用 steady_clock（跑在 wall timer 上，不受 use_sim_time 跳变影响）：约 1 Hz、单行、不刷屏。
  {
    const auto now_tp = std::chrono::steady_clock::now();
    bool due = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!status_log_ever_printed_ ||
        std::chrono::duration<double>(now_tp - last_status_log_tp_).count() >= 1.0)
      {
        last_status_log_tp_ = now_tp;
        status_log_ever_printed_ = true;
        due = true;
      }
    }
    if (due) {
      int n_acc = 0;
      int n_tot = 0;
      double last_score = std::numeric_limits<double>::quiet_NaN();
      double last_align_ms = 0.0;
      size_t last_src_pts = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        n_acc = accepted_cycles_;
        n_tot = total_cycles_;
        last_score = last_score_;
        last_align_ms = last_align_ms_;
        last_src_pts = last_source_points_;
      }
      if (n_tot == 0) {
        // 还没跑过任何一帧 align ⇒ 只报状态，避免打出误导性的 0 ms / nan
        RCLCPP_INFO(
          get_logger(), "[status] backend=%s 尚无配准帧：%s | target=%zu 点（地图 leaf %.3f m）",
          backend_->name(),
          valid ? "等点云 / 等 TF" :
          "**无初值 ⇒ 不发 map→odom**（等 /initialpose 或 initial_pose + TF odom→base）",
          map_cloud_->size(), voxel_leaf_size_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "[status] backend=%s 采纳 %d/%d 帧 | 最近 score=%s m² | align=%.1f ms | "
          "source=%zu 点（leaf %.3f）→ target=%zu 点（leaf %.3f）| map→odom %s",
          backend_->name(), n_acc, n_tot, scoreToStr(last_score).c_str(), last_align_ms,
          last_src_pts, voxel_leaf_size_scan_, map_cloud_->size(), voxel_leaf_size_,
          poseToStr(T_map_odom).c_str());
      }
    }
  }
}

rclcpp::Time GicpNode::lookaheadStamp() const
{
  // now() = 节点时钟：use_sim_time=true 时是 Gazebo 的仿真时间（由 launch 注入，见 config 顶部）。
  // 用节点时钟而不是点云 header.stamp：点云戳是"采集那一刻"（已经过去几十~上百 ms），
  // 拿它盖 TF 会让 buffer 里的最新条目落在消费者的查询时间之前。
  return now() + rclcpp::Duration::from_seconds(tf_lookahead_sec_);
}

rclcpp::Time GicpNode::publishTf(const Eigen::Matrix4d & T_map_odom)
{
  // ============================ TF 时间戳契约（本次修复的核心） ============================
  // 症状（用户实跑 localization:=gicp）：
  //   [controller_server] Exception in transformPose: Lookup would require extrapolation into the
  //     future. Requested time 654.682000 but the latest data is at time 654.582000,
  //     when looking up transform from frame [odom] to frame [map]
  //   [controller_server] Unable to transform robot pose into global plan's frame
  //   [controller_server] [follow_path] [ActionServer] Aborting handle.   ← 每周期 abort ⇒ 导航卡死
  // 原因：nav2 的消费者**不在"此刻"查 map→odom**，而是在 now + transform_tolerance 那一档查
  //   （MPPI 的 PathHandler::transformPose 把 transform_tolerance 当 tf2 等待/前瞻；
  //    本仓库 nav2_params_sim_controller_mppi.yaml 里 FollowPath.transform_tolerance=0.1，
  //    日志里 requested 恒 = 最新数据 + 0.100 s 正好对应它）。
  //   AMCL 之所以从来不出这个错，是因为 **transform_tolerance 让它把 map→odom 盖成未来时间戳**
  //   （本仓库 amcl transform_tolerance=0.3，见 nav2_params_sim_base.yaml）；我们原来用 now() 盖戳，
  //   buffer 里最新条目就永远比请求时间旧 0.1 s ⇒ tf2 抛 ExtrapolationException。
  // 修法：戳 = now() + tf_lookahead_sec_（参数文件现值 0.45，与 AMCL 同语义）；值（map→odom 的
  //   数值语义）完全不变，只改"这条变换属于哪个时刻"。**故意不用点云时间戳**盖 TF。
  // 过大/过小的取舍见 config/gicp_registration_sim.yaml 的 tf_lookahead_sec 注释。
  // 2026-10-05：光靠 0.45 的余量撑不住"定时器被饿死"（实测 /tf 掉到 ~2 Hz）⇒ 本轮把定时器
  //   搬进独立回调组 + MultiThreadedExecutor（见文件顶部的"并发契约"）。0.45 保持不变（契约不动）。
  // ======================================================================================
  const rclcpp::Time stamp = lookaheadStamp();
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = map_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  matrixToTransform(T_map_odom, tf_msg.transform);
  tf_broadcaster_->sendTransform(tf_msg);
  return stamp;
}

void GicpNode::publishPose(const Eigen::Matrix4d & T_map_odom, double score)
{
  // 运行上下文（2026-10-05）：~/pose 仍**只由 align 组**发布（pointcloudCallback：每帧一次，
  // 以及其中的 consumePendingInitialPose 在应用 /initialpose 时立刻补一条），**没有**搬到 TF 定时器里
  // —— 话题语义/频率与改动前完全一致（交接回调 initialPoseCallback 一个话题都不发）；
  // 定时器组只负责 TF（map→odom）与状态行。last_tf_stamp_ 是"定时器组写、本组读"的跨组共享
  // 成员 ⇒ 下面的取戳在 mutex_ 下做。
  // ① 戳：与 TF **同一条**（优先复用最近一次真正发出去的 TF 戳；若还没有 TF 就用同一条公式）。
  //    这样"pose 的戳"与"TF 的覆盖范围"天然对齐；用点云时间戳则 pose 会显得比 TF 旧一整帧。
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stamp = last_tf_stamp_valid_ ? last_tf_stamp_ : lookaheadStamp();
  }

  // ② 值：T_odom←base 一律取**最新可用**（不做精确时间戳查询）。两个理由：
  //    · pose 的语义是"此刻机器人在 map 系的位姿"，与某个历史帧时间无关；
  //    · 上面的 stamp 是 **now+lookahead（未来戳）**，拿未来戳做精确查询在 tf2 里必然
  //      extrapolation 失败 ⇒ 每次都白等一个超时（0.05 s）+ 污染 last_tf_error_。
  Eigen::Matrix4d T_odom_base;
  bool used_latest = false;
  if (!lookupTf(odom_frame_id_, base_frame_id_, stamp, false, T_odom_base, used_latest)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF %s←%s 查不到（%s）⇒ 本帧不发布 ~/pose（TF map→odom 不受影响）",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), lastTfError().c_str());
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
  // 时间戳：**健康信号不带戳**——std_msgs/Float64、std_msgs/Bool 都没有 header 字段，没有可设的
  // stamp；它们隐含的时间就是发布时刻（sim now）。这两个话题只给人/监控用，不参与 TF 查询，
  // 因此本次的时间戳修复**不需要**（也不应该）给它们加前瞻：诊断用的"这一帧是什么时候算的"
  // 恰恰应该留在"现在"。若以后要看"每帧的处理延迟"，应显式加 header，用点云戳而非 lookahead 戳。
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
