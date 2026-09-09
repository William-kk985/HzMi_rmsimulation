-- HZMIR Sentry Cartographer 建图配置
-- 适配 Livox MID360 激光雷达
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "livox_frame",          -- Livox MID360 内置 IMU，数据在此坐标系
  published_frame = "body",                -- 发布的机器人位姿
  odom_frame = "odom",
  provide_odom_frame = true,               -- Cartographer 提供 odom frame
  publish_frame_projected_to_2d = true,    -- 投影到2D平面
  use_odometry = false,                    -- 不使用外部里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,                     -- 不使用2D激光
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                    -- 使用3D点云输入
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
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.0  -- 减小，更快响应

-- 点云范围过滤 (适配 RMUL 赛场 PVC 地胶)
TRAJECTORY_BUILDER_2D.min_range = 0.45          -- 最小距离 0.45m，避开 38cm 的地面最近点
TRAJECTORY_BUILDER_2D.max_range = 12.0          -- 减小最大距离，提高稳定性
-- 高度过滤 (基于机器人高度和 PVC 地面特性)
TRAJECTORY_BUILDER_2D.min_z = 0.05              -- 地面上 5cm，过滤 PVC 噪声
TRAJECTORY_BUILDER_2D.max_z = 0.8               -- 地面上 80cm，匹配机器人高度上限
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
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

-- 快速相关扫描匹配器
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
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
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

return options
