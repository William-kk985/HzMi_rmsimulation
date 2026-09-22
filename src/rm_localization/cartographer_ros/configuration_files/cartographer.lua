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
-- ★★ 2026-09-22（六次修正）：
-- (A) IMU 又必须关回 false —— 第五次打开后拿到的是**另一条** CHECK（说明时间基修复有效，
--     上一层乱序不再出现，现在是 IMU 自身的数据/时间戳问题）：
--       F imu_tracker.cc:67] Check failed: (orientation_ * gravity_vector_).z() > 0. (0 vs. 0)
--         @ ImuTracker::AddImuLinearAccelerationObservation()
--         @ PoseExtrapolator::AdvanceImuTracker()
--         @ PoseExtrapolator::AddPose()   ← Node::PublishLocalTrajectoryData()
--     含义：重力估计被**清零**。`ImuTracker` 的首个加速度样本 alpha≈1（delta_t = now - Time::min
--     极大）→ gravity_vector_ 直接被赋成"第一帧 IMU 的 linear_acceleration"；只要那一帧是
--     (0,0,0)，后面 `(orientation_*gravity).z() > 0` 必然失败。
--     → 与"FAST-LIO 能用同一路 IMU"并不矛盾：FAST-LIO 是**多帧求均值**做重力初始化，
--       天然容忍启动阶段的几帧零值；cartographer 是拿**第一帧**当基准。
-- (B) 同时把"四次修正"改过的两个局部匹配参数**恢复成上游默认**（回到基线，一次只改一个变量）：
--     21:0x 那次实测（`monitor_map_odom.py`：|Δyaw| P95 1.94°/样本、max 83°、143 次跳变）
--     说明"更信任先验 + 5° 小搜索窗"并没有让 map→odom 变稳，反而可能剥夺了匹配器的纠正能力
--     （若先验旋转与扫描不一致，5° 窗口让它够不着）。先回到默认，用 monitor 采一条干净基线。
-- ----------------------------------------------------------------------------
-- 保留（已证必要）：use_odometry=true + 先验为独立底盘 odom（/odom_ground_truth，#21 路线①）、
--   时间基修复（插件 #19）、回环门槛 0.65/0.70 与收紧的约束搜索距离/窗口（#20/#22）。
-- 打开 IMU 的前置条件（#24）：先确认 `/livox/imu` 的前几帧不是 (0,0,0)、且三路时间戳同轴。
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
-- ★★★ 2026-09-23（十一次修正）：**本参数在本链路里是空转的**，留 0.05 只为无害。
--   读源码（cartographer 2.0.9004 / cartographer_ros 2.0.9002，与上游 master 逐字节相同）：
--     `local_trajectory_builder_2d.cc` 里 `missing_data_ray_length` **只在** "range > max_range" 分支生效：
--        } else {                       // range > options_.max_range()
--          hit.position = origin + options_.missing_data_ray_length() / range * delta;
--          accumulated_range_data_.misses.push_back(hit.position); }
--     而 /scan 的 range_max = 10.0（p2l 侧），这里 max_range = 12.0 ⇒ **永远进不了这个分支**
--     ⇒ `range_data.misses` 恒为空 ⇒ 该参数改多少都不影响地图。
--     · 另外 p2l 的 `use_inf: true` 把"无回波"的 bin 发成 inf，cartographer_ros 的
--       `msg_conversion.cpp:LaserScanToPointCloudWithIntensities` 用
--       `msg.range_min <= first_echo && first_echo <= msg.range_max` 过滤 ⇒ inf **直接被丢掉**（既非命中也不清除）。
--   ⇒ 结论：八/九/十次修正里"改 missing_data_ray_length 来擦墙/留墙"的推理**全部无效**，
--     那两步测到的变化只来自同时改动的 hit/miss 与 num_range_data。
--     **清空/擦墙的唯一来源是"打到东西的光束"（起点→命中点）沿途的 miss 写**（同见下方 11 次修正）。
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.05   -- ← 空转（见上），别再拿它当旋钮
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- num_accumulated_range_data 实测（离线复现）：1 最好；2 ⇒ +20帧留存 42.2%→46.3%（略好但会把 0.2s 内的
--   运动糊成一个位姿，实车/转弯时是隐患）；3 ⇒ 掉到 30.2%（糊得比"命中优先"赚的还多）。保持 1。

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

-- ★ 2026-09-22（六次修正）：局部匹配恢复**上游默认**（见文件上方 (B) 说明）。
--   历史：30°/0.2m（"没有可用先验时代"的遗留）→ 5°/0.1m（四次修正，"更信任先验"）→ 实测变差 → 回默认。
--   教训：先验可信 ≠ 匹配器可以没有纠正能力；一次只改一个变量并用 monitor_map_odom.py 采数据。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1   -- 上游默认
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- 上游默认
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 偏离先验的代价（>1 = 更信任先验）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.

-- Ceres 扫描匹配器 —— 同样回到上游默认 10 / 40（四次修正曾提到 1e2 / 4e2，实测没帮上忙）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- 运动滤波器 - 适当放宽，减少计算量
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- ============================================================================
-- 子图 / 概率栅格写入（2D SLAM 唯一的"记忆"全在这两个参数上）
-- ============================================================================
-- ★★★ 2026-09-23（十一次修正）：**离线复现之后重写本段**。新工具 tools/replay_scan_grid.py：
--   把 bag 里的 /scan + /tf 位姿离线重放成 cartographer 的概率栅格（逐条照抄
--   probability_grid_range_data_inserter_2d.cc：**命中先写 → 清除后写 → 同周期内先写者胜**），
--   再用 tools/analyze_slam_bag.py 第 ⑤ 段同一套指标打分。校验（对 ret4 **同一条轨迹**）：
--   真 /map 末态仍占据 40.5% ↔ 复现 43.3%；留存 +10 帧 52.9% ↔ 52.8%；闪烁中位 3 ↔ 3
--   ⇒ 可以信它的**相对**结论（绝对值受位姿来源/子图近似影响）。
--
--   ❌ 已证伪（八~十次修正的推理）："37% 无回波光束 × missing_data_ray_length 是擦墙元凶"
--      —— 本链路 `range_data.misses` 恒空、该参数空转（逐条源码见文件上方 missing_data_ray_length 段）。
--   ✅ 真正的擦除机理（读源码 + 实测）：
--      · 一帧内命中优先，但**跨帧没有任何保护** ⇒ 下一帧"打到东西的光束"沿途仍会把上一帧的墙格子写成 free；
--      · 实测（ret4；tools/diag_wall_passes.py）：真 /map 里的墙格子每帧只有 **~12% 被命中**、
--        **~17% 被"更远的回波"压过** ⇒ 净票 ≈ 0（+0.017/帧）⇒ 栅格在阈值附近随机游走
--        = 用户看到的"闪 + 慢慢化掉"；
--      · 压过它的回波长什么样：**82% 来自距本格子 50cm 以内的回波**（0~5cm 19%、5~10cm 23%、
--        10~20cm 17%、20~50cm 23%），只有 7% 是"那一帧该方位根本没看见墙"。
--        ⇒ 元凶是 **5cm 栅格把墙面量化成锯齿，打到"深齿"的射线会压过"浅齿"的格子**（掠射+量化），
--          既不是无回波光束，也不是"墙看不见了"；
--      · 命中为什么这么稀：一帧 30000 点 → p2l 压成 1310 条 range → cartographer 5cm 体素滤波后
--        只剩 **~373 个不同的命中格子**（墙总共几千格）；且世界里挡板只有 ~0.4m 高、雷达在 0.226m，
--        落进 p2l 高度带的竖直窗口很小 ⇒ 每个墙格子平均 **约 9 帧才轮到一次命中**；
--      · `/map` 的取值上限是 **75**（不是 100）：P=0.80→65、P=0.90→75（submaps.h + submap_painter.cc
--        + CreateOccupancyGridMsg 三段串起来算出来的）⇒ tools 里的 `>=65` 即 P>=0.80
--        ⇒ 墙要 **2 次命中**才"变黑"，但只要 **3 次清除**就掉出去（阈值贴着天花板）。
--   ⇒ 能撬动的只有：①命中票加重；②子图别换那么勤（每 3 秒清零一次证据）；③关掉清除（自由空间一起没，已否决）。
--     离线**同轨迹**扫描结果（hit/miss/num_range_data）：
--      | 参数 | 曾稳定≥5帧 | 末态仍在 | 留存+20帧 | 闪烁中位 | 末态占据 |
--      | 0.68/0.40/30（旧）              | 2001 | 43.3% | 42.2% | 3 | 1087 |
--      | **0.85/0.40/30**                | 4421 | 44.3% | 57.0% | 2 | **2073** |
--      | **0.85/0.40/90（本次采用）**     | 4474 | 44.8% | **58.0%** | 2 | 2098 |
--      | 0.68/0.40/∞（完全不换子图）      | 2973 | 47.6% | 59.5% | 1 | 1452 |
--      | 0.68/0.40/30 + insert_free_space=false | — | ~89% | ~78% | — | 自由格=0（已否决）|
--      | 原始 3D 点云(z∈[−0.15,0.2]) 当输入 | 2727 | 47.0% | 46.9% | 3 | 1544 |
--   ⇒ 本次取 **hit 0.85 + num_range_data 90**（90 = 上游默认值）。
--     预期：实心墙格子约翻倍，+20 帧留存 42%→58%，自由空间不变（~8000 格），闪烁中位 3→2。
--   ⚠️ 仍有约一半"曾经很实的"墙格子最后会掉出去 —— 这是本传感/世界几何的**结构性上限**
--      （低雷达 0.226m + 约 0.4m 挡板 ⇒ 单格证据天生稀疏），不是再调一个参数能消掉的。要更稳只能：
--      抬高雷达 / 加高挡板 / 换更密且不重复的输入，或改 cartographer 源码给"已占据格子"加清除豁免
--      （third_party/ 保持原样，不做）。
--   ⚠️ 历史教训：ret/ret2/ret3/ret4 是**四条不同路线/时长**的 bag（167s/137s/78s/104s），
--      跨 bag 比"留存百分比"没有意义（九次修正"实测变差"就栽在这：5 个参数是在不同轨迹上比的）。
--      以后只认"同一条 bag 离线扫参数"，再让用户跑 1~2 次确认。
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm 分辨率
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
-- insert_free_space=true 必须保留：**"擦旧墙"和"标空地"是同一个写**（都是给射线途经的格子写 miss），
--   关掉它确实能到 89% 留存，但自由格子=0、地图全灰（ret2 判别实验）。
--   本场景没有动态物，但实车有，清除能力不能丢。推导见 docs/debug_fastlio_cartographer.md §5.2.3。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.85   -- ← 十一次修正：0.68 → 0.85（命中更粘）
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.40   -- 0.40 是甜点：0.49 ⇒ 自由格子几乎长不出来(实测 31 格)；0.30 ⇒ 墙被擦得更快

-- ============================================================================
-- 位姿图优化配置
-- ============================================================================
-- ⚠️ 同上：随 num_range_data 一起回滚到 30（实测负收益）。
-- ★（原八次修正的想法）：30 → 90。理由同 num_range_data：10Hz 插入下
--   30 节点 = 每 3 秒优化一次，位姿图修正过频也会让栅格反复重画。
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
