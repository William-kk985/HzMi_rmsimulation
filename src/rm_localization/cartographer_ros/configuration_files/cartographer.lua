-- HZMIR Sentry Cartographer 建图配置
-- 适配 Livox MID360 激光雷达
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- ===== 帧契约（2026-09 修正，须与 docs/tf_interface_contract.md 一致）=====
  -- 本工程分工：odom→base_link 由 LIO(lio_tf_adapter) 发；map→odom 由"全局对齐提供者"发。
  -- Cartographer 扮演后者，故**只应发 map→odom**：
  --   published_frame="odom" + provide_odom_frame=false  -> 只发 map→odom（与 amcl/icp 一致）
  -- 旧配置 published_frame="body" + provide_odom_frame=true 会额外发 odom→body，
  -- 而 body 已被 FAST-LIO 占用（laserMapping.cpp 的 child_frame_id="body"）-> TF 多父边。
  map_frame = "map",
  -- ⚠️ tracking_frame 必须取 **IMU 所在帧**，不能取雷达帧：
  --   sensor_bridge.cpp 对 IMU 有一条硬 CHECK ——「IMU 帧必须与 tracking_frame 重合（平移 < 1e-5 m），
  --   否则把线加速度转到 tracking_frame 会不准」，不满足直接 abort（exit code -6，2026-09-21 实际踩到）。
  --   本工程 URDF：livox_frame 在 base_link+(0.12,0,0.175)、imu_link 在 +(0.12,0,0.125)，相差 5cm；
  --   而这 5cm 是**故意**的（FAST-LIO 的 extrinsic_T=[0,0,0.05] 就是它）→ 不能靠改 URDF 消除。
  --   官方 mir-100-mapping.lua 同样拿 IMU 帧（"imu_frame"）当 tracking_frame，是标准做法。
  --   代价只是：/scan 在 livox_frame 里，cartographer 会用 URDF 静态 TF 把它转到 imu_link（纯 5cm 平移）。
  tracking_frame = "imu_link",
  published_frame = "odom",                -- 只发布到 odom，即 map→odom 由本节点提供
  odom_frame = "odom",
  provide_odom_frame = false,              -- 不再自造 odom→xxx，避免与 LIO 争 body 的子帧
  publish_frame_projected_to_2d = true,    -- 投影到2D平面
  -- ★★ 2026-09-22（三次修正，路线①）：先验**要开**，但必须换成"独立的底盘/轮速里程计"。
  --   历史三步，别再来回翻：
  --     ① 一开始 use_odometry=false → 转起来"墙跟着车走"；
  --     ② 2026-09-21 打开 use_odometry=true 但喂的是 **LIO 自己的 /odom**
  --        → cartographer_node 建图中 exit -6(SIGABRT)，地图照样跟着车转（方向错在这）；
  --     ③ 2026-09-22 一度关掉（纯扫描匹配）→ 空洞期间没有东西推位姿，仍会跟转；
  --        现在改回 true，但把 odom 换成**独立来源**。
  --   为什么 LIO 的 /odom 不能当先验（已证）：
  --     a. 同源：/odom 与 /scan 来自同一份雷达点云 → 反馈回路（LIO 漂移直接拖地图）；
  --     b. 晚到：LIO 要处理完一整帧才发 odom → "第 k 帧的 odom"常在"第 k+1 帧的 scan 已处理"
  --        之后才到 → 撞 pose_extrapolator 的时间序 CHECK（libcartographer.a 里只有两条，这是其一）：
  --          Check failed: timed_pose_queue_.empty() || odometry_data.time >= timed_pose_queue_.back().time
  --        → abort(exit -6)；没 abort 的时段先验按错时刻套用 → 每帧被拖一下。
  --   为什么独立底盘 odom 能同时解决"崩"和"糊"：
  --     · 它由**物理插件/下位机**按自己的时刻产出、立即到达 → collator 永远有一个
  --       ≥ 上一帧位姿的 odom 头 → 上面那条 CHECK 不可能被踩；
  --     · /scan 出现空洞时（实测 2.8Hz、每 ~0.5s 一个洞）靠它把位姿推过去
  --       → 下一帧扫描的初值仍落在搜索窗内（空洞 × 角速度 ≤ 搜索窗，见 runbook §7）。
  --   ⚠️ 本行的 use_odometry=true 与"bringup 把 odom 重映射到独立话题"是**一对**：
  --      只开这个而不 remap，就会退回订阅 LIO 的 /odom → 复现 exit -6。
  --      仿真 = /odom_ground_truth（gazebo_ros_planar_move）；实车 = 下位机轮速 odom。
  --   ⚠️ 台架偏差：planar_move 的 odom 是**无打滑的理想值**，实车轮速会打滑 →
  --      这一项台架比实车"容易"，见 docs/sim_real_contract.md §三。
  --   另：上游 revo_lds.lua（只有 2D 激光、无任何 odom/IMU）是 use_odometry=false + 
  --      use_online_correlative_scan_matching=true —— 我们是"有独立先验"的另一种合法形态。
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- ===== 输入源（2026-09 修正）=====
  -- 默认走 2D 激光：吃感知域 p2l 产出的 /scan（**已去地面、已按高度带切好**），
  -- "什么算障碍"由感知域单点决策，且与 slam_toolbox 公平可比、更省 CPU。
  num_laser_scans = 1,                     -- 使用 2D 激光（/scan）
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  -- 备选路线（点云直喂，由 Cartographer 自己做高度带）：
  --   num_laser_scans = 0; num_point_clouds = 1; points2 remap 到
  --   /livox/lidar/pointcloud（**不是 /livox/lidar —— 那是 CustomMsg，cartographer_ros 不支持**），
  --   且 min_z/max_z 必须设成"排除地面"的值（它们是**相对传感器**的，不是相对地面！）
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,      -- 增加超时
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,          -- 200Hz 位姿发布
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ============================================================================
-- 2D SLAM 配置 (适合室内平面导航)
-- ============================================================================
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- ============================================================================
-- 轨迹构建器配置 - 优化稳定性
-- ============================================================================
-- ★ 2026-09-22：关掉 IMU，只留 odom 作为运动先验
-- 原因：同时喂 IMU（Gazebo 插件直发 100Hz）与 odom（FAST-LIO 处理完才吐 10Hz）时，
-- 两者时间戳会交错出现"odom 比已收到的最后一帧 IMU 早几十微秒"（实测差 39µs），
-- 触发 cartographer 的硬 CHECK 直接 abort：
--   F pose_extrapolator.cc:229] Check failed: time >= imu_tracker->time()
--     @ PoseExtrapolator::ExtrapolateRotation()
--     @ PoseExtrapolator::AddOdometryData()
--     @ Node::HandleOdometryMessage()
-- 这是已知问题（https://answers.ros.org/question/320444/）。
-- 我们的 /odom 本来就是 FAST-LIO 融合了 IMU+LiDAR 的 10Hz 结果（比原始 IMU 更好用），
-- cartographer 不必再吃原始 IMU → 单一运动先验，彻底避开时间戳交错。
-- （平地仿真不需要 IMU 做重力对齐；use_imu_data=false 后 cartographer 也不再订阅 /livox/imu，
--   collator 只需等 scan+odom 两路，反而更不容易卡。）
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.0  -- 该键仍会被读取（不能删）

-- 点云范围过滤 (适配 RMUL 赛场 PVC 地胶)
-- ⚠️ min_range 由 0.45 改 0.2：原值理由是"避开 38cm 的地面最近点"，但我们吃的是感知域 p2l 的 /scan
--    （已去地面 + 已切高度带的平面点），该理由不成立；0.45 会把 p2l 保留的 0.2~0.45m 近点又丢掉
--    → 地图里看不见近处障碍，而 costmap（直接读 /scan）却看得见，两者不一致。
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 12.0          -- 减小最大距离，提高稳定性
-- 高度过滤 (基于机器人高度和 PVC 地面特性)
-- ⚠️ min_z/max_z 是**相对 tracking_frame**的高度带，不是相对地面！
-- （tracking_frame=imu_link，与雷达 livox_frame 只差 5cm，所以数值不用动）
-- 走 /scan 时点是激光平面上的 z=0，只要带包含 0 即可；高度决策已由感知域 p2l 完成。
TRAJECTORY_BUILDER_2D.min_z = -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 体素滤波 - 精细配置，保留 RMUL 场地细节
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- 5cm 体素，保留墙壁和台阶特征

-- 自适应体素滤波
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.8
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 12.

-- 回环检测体素滤波
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.2
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 80
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 12.

-- 扫描匹配 - 增大搜索窗口，提高稳定性
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2   -- 增大
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- 增大
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 减小
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.     -- 减小

-- Ceres 扫描匹配器 - 平衡权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.   -- 增加旋转权重，减少旋转漂移
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- 运动滤波器 - 适当放宽，减少计算量
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- ============================================================================
-- 子图配置 (关键修改：减少num_range_data，适配MID360高频)
-- ============================================================================
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30   -- 从60→30，子图更小，定位更灵活
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm 分辨率
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- ============================================================================
-- 位姿图优化配置
-- ============================================================================
POSE_GRAPH.optimize_every_n_nodes = 30      -- 减少优化频率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- ★ 2026-09-22（二次修正）：回环门槛调回"能找到"的水平。
--   上一版把 min_score 提到 0.72（global 0.8）后，cartographer 收尾时打印的是
--     Score histogram: Count: 0   （0 computations resulted in 0 additional constraints）
--   即**一条回环都没有**→ 位姿图完全没有全局锚定，地图只剩 local SLAM 的结果，
--   于是"地图跟着车转/糊"再怎么调 min_score/搜索窗都治不了本（调的是已经不工作的东西）。
--   回到上游 revo_lds.lua 的 min_score=0.65；global 用 0.70（比上游略严，防跨场地假匹配）。
--   搜索距离/搜索窗仍保持收紧（4m / 2m / 10°）——这才是真正挡住"跨场地误回环"的那两个参数。
POSE_GRAPH.constraint_builder.max_constraint_distance = 4.0            -- 10.0 -> 4.0
POSE_GRAPH.constraint_builder.min_score = 0.65                         -- 0.72 -> 0.65（上游 2D 参考值）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70     -- 0.80 -> 0.70
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- 快速相关扫描匹配器：搜索窗也收紧（现在有 odom 先验，不需要 5m / 20° 那么宽）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.0          -- 5. -> 2.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)  -- 20° -> 10°
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres 扫描匹配器
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 优化问题配置
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.global_constraint_search_after_n_seconds = 30.   -- 10s -> 30s（小场地里全局瞎找很容易假匹配）

return options
