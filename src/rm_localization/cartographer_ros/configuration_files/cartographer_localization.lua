-- HZMIR Sentry Cartographer 纯定位配置
-- 加载已有地图进行定位，不建立新地图
-- 优化版本：降低 CPU 占用，确保 Nav2 能正常运行

-- 第一步：完整继承建图配置
include "cartographer.lua"

-- 第二步：重新定义options，确保参数继承完整
options = options or {}

-- ============================================================================
-- 纯定位模式核心开关
-- ⚠️ 这两个键挂在 **TRAJECTORY_BUILDER**（顶层），不是 TRAJECTORY_BUILDER_2D！
--    层级写错的后果不是"参数无效"，而是 cartographer 的 LuaParameterDictionary
--    在析构时 CHECK「每个键必须被读恰好一次」→ **启动即 FATAL**：
--      Check failed: 1 == reference_counts_.count(key)
--      Key 'pure_localization' was used the wrong number of times.
--    （2026-09-21 实测。官方 backpack_2d_localization.lua 同样写在 TRAJECTORY_BUILDER 上；
--      读取点见 cartographer/mapping/trajectory_builder_interface.cc 的 kDictionaryKey 与
--      map_builder.cc 的 has_pure_localization_trimmer()。）
--    另：`pure_localization` 这个 bool 在 2.0 里已 deprecated（设了只会打警告，
--    map_builder.cc 明说改用 pure_localization_trimmer）→ 现在只写 trimmer。
-- ============================================================================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- 只保留最近 3 个子图，降低内存与计算量
}

-- ============================================================================
-- 纯定位模式优化 - 降低 CPU 占用
-- ============================================================================
POSE_GRAPH.optimize_every_n_nodes = 100   -- 从50→100，进一步减少优化频率
POSE_GRAPH.global_sampling_ratio = 0.003 -- 减少全局采样，降低 CPU
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1  -- 大幅减少约束采样！
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 提高阈值，减少无效匹配

-- 回环检测 - 降低频率
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.  -- 减少搜索频率！

-- 搜索窗口 - 适当减小，降低计算量（和建图配置对齐）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 运动滤波器 - 复用建图配置，避免冲突
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- 减少后台线程数，给 Nav2 留出 CPU（明确修改map_builder参数）
MAP_BUILDER.num_background_threads = 2

-- 第三步：返回完整的options
return options
