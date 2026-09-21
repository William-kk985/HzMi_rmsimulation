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

return options
