-- ============================================================================
-- 全包形态（启动参数 lio:=cartographer）—— cartographer 同时担任两个角色：
--   ① 里程计源：发 odom→base_link（靠 provide_odom_frame = true）
--   ② 全局对齐源：发 map→odom
-- 契约与边界见 docs/tf_interface_contract.md「全包形态」一节，要点：
--   • 本形态下**不能**再起 FAST-LIO / point_lio，也不允许有第二个 map→odom 发布者
--     （bringup 已按 lio 参数门控：mapper 槽与 localization 槽都被跳过）；
--   • odom→base_link 的连续性由 cartographer 的 pose extrapolator 提供
--     （use_pose_extrapolator 默认 true，本质是"IMU 外推 + 上次匹配结果"），
--     精度与鲁棒性弱于 FAST-LIO 的紧耦合 IEKF —— 这是本形态的主要代价；
--   • **不发布 nav_msgs/Odometry**（cartographer 没有这个输出）→ `nav:=teb` 不适用
--     （TEB 需要 /odom 做速度反馈）；`rpp`/`dwb` 正常；
--     velocity_smoother 是 OPEN_LOOP，不需要 /odom（已核对源码）。
-- ============================================================================
include "cartographer.lua"

-- published_frame 直接取 base_link：provide_odom_frame=true 时它会发
--   map  → odom_frame     (= local_to_map)
--   odom_frame → published_frame (= odom→base_link)
-- 正好就是系统需要的那两条边，与 FAST-LIO 形态下的契约完全一致 → nav2 侧零改动。
-- 其中 base_link←imu_link 的静态变换由 URDF 提供（published_to_tracking 查 TF 得到）。
options.published_frame = "base_link"
options.provide_odom_frame = true

-- ============================================================================
-- 接一路"底盘里程计"当运动先验（2026-09-21 新增，解决实测的静止漂移）
-- 为什么必须补：没有 odom 时，cartographer 的 odom→base_link 只能靠 pose extrapolator
--   （= 上次匹配位姿 + IMU 二次积分）。IMU 的重力对齐残差/零偏被二次积分放大后，
--   **机器人静止也在漂**，而这个漂移又被当作扫描匹配的初始猜测 → 地图被拖着走。
--   实测（RMUL2026，静止）：odom→base_link 以 ~4 cm/s 平移、~13°/min 转动漂移。
-- 数据来源：由 bringup 用 odom_topic 参数 remap（仿真 = Gazebo 底盘里程计
--   /odom_ground_truth；实车 = 下位机轮速里程计），见 cartographer_sim.launch.py。
-- 为什么可以给"世界系绝对位姿"：cartographer 只用 odom 的**增量**——
--   速度估计用相邻两帧 odom 的 delta（pose_extrapolator.cc:AddOdometryData），
--   位姿图约束也用 CalculateOdometryBetweenNodes 的 delta（optimization_problem_2d.cc），
--   所以常量偏移会被自动消掉。
-- 调参提醒：odometry_translation/rotation_weight 目前是 1e5（在 cartographer.lua 里），
--   仿真这条是真值所以合适；**实车轮速有滑移时要调小**（例如 1e3），否则回环拉不动轨迹。
-- ============================================================================
options.use_odometry = true

return options
