# 问题与发现汇总（HzMi RM 仿真 bench）

> 本文汇总本轮仿真联调中**实际踩到的问题**（现象 → 根因 → 修复 → 证据）、**静默失效类坑**、
> **地图资产/坐标系结论**、**架构改动**与**概念澄清索引**，最后是**未完成事项**。
> 详细复现命令见 `docs/smoke_test_runbook.md`，分层设计见 `docs/architecture.md`。
> 想照着方法**一步步定位**（而不是查具体某条故障）见 `docs/debug_fastlio_cartographer.md`（分层判读法 + 工具 + 症状速查表）。

---

## 一、运行时故障：根因 → 修复

| # | 现象 | 根因 | 修复 | 证据 / 落点 |
|---|---|---|---|---|
| 1 | `Unable to parse the value of parameter robot_description as yaml` | launch_ros 把 URDF(XML) 当 YAML 解析（Humble 行为） | `ParameterValue(robot_description, value_type=str)` 包裹 | `hzmi_rm_simulation/launch/rm_simulation.launch.py` |
| 2 | `spawn_entity` 报 `No module named 'numpy'` | conda 的 python(3.13) 抢占 PATH；ROS Humble 需系统 python 3.10 | `conda config --set auto_activate_base false` + `~/.bashrc` 末尾 `export PATH=/usr/bin:$PATH` | 用户环境；**不要激活 `.venv`** |
| 3 | 成批 `Could not find requested resource in ament index`（nav2 组件）+ 缺 `nav2_rviz_plugins` | nav2 主体包未安装 | `apt install ros-humble-navigation2 nav2-bringup nav2-rviz-plugins`（+ `dwb-critics`） | 装后组件正常加载 |
| 4 | `/scan` 报 `incompatible QoS` | costmap `obstacle_layer` 订阅 reliable，而 `/scan` 是 best-effort | 三份 `nav2_params_sim_*.yaml` 的 scan 源加 `reliability_policy/qos_policy: best_effort` | `ros2 topic info /scan --verbose`：1 pub + 3 sub **全 BEST_EFFORT** |
| 5 | costmap 刷屏 `Sensor origin ... out of map bounds` + `odom→base_link` 抖动 | `odom` 有两个来源：Gazebo 真值 + LIO 都发 `/odom` | sim URDF 把 Gazebo 真值 remap 到 **`/odom_ground_truth`**，`publish_odom_tf=false`（T4） | `/odom` 由 LIO 独占 |
| 6 | RViz 里车/雷达**倾斜**（Gazebo 里是正的） | spawn `z=1.16` 而地面在 z≈0 → 悬空 1.1 m 坠落，FAST-LIO 在坠落中做重力初始化 | RMUL/RMUL2026 spawn `z` 改 **0.2** | `tf2_echo odom base_link` RPY ≈ (0.34°,0.23°,-0.35°) |
| 7 | `Package 'rm_nav_bringup' not found ... searching: ['/opt/ros/humble']` | 新终端只 source 了 /opt/ros | `source install/setup.bash` | runbook §0.1 |
| 8 | nav+amcl 卡在 `amcl: Waiting for map....` / `global_costmap: Invalid Frame "map"` | ① `map_server_launch.py` 与 `localization_amcl_launch.py` **各起了一个同名 `lifecycle_manager_localization`**；② `nav2_params_sim_*.yaml` 里 `yaml_filename` 被注释，而 nav2 的 `RewrittenYaml` **只替换已存在的键** → `parameter 'yaml_filename' is not initialized` | ① amcl launch 用**单一 manager 同时管 `map_server`+`amcl`**；② 恢复 `yaml_filename: ""` 键 | 沙箱实测：`Read map RMUL.pgm: 272 X 210` → `amcl: Received a 272 X 210 map` → 双双 Activating |
| 9 | `tf2_echo map odom` 报 `frame does not exist` | **amcl 只在收到 `/scan` 之后才发 `map→odom`**（`sendMapToOdomTransform` 只被 `laserReceived` 调用，且 `initial_pose_is_known_` 为假时 return） | 判据改为"先查 `ros2 topic hz /scan`" | 无头实测：无 `/scan` → 无 `map` 帧；补 `/scan` → `map→odom` 立现 |
| 10 | 发目标后车不走，`/cmd_vel` 只有 `linear.x: -0.05` + 旋转 | `-0.05` = nav2 BT `BackUp backup_speed`；根因是 **`planner_server: failed to generate a valid path`** → BT 恢复行为循环 | 见 §三（幽灵墙）；并新增 `tools/check_map_reachable.py` 选点前验证 | 用户机日志 + 工具判定 |
| 11 | 场景三(nav+ICP) 里 `map`/`odom` 帧一直不存在，Nav2 无法激活 | `spawn_entity: Spawn service failed` 后机器人晚 ~4 s 才插入，期间 LIO 尚未吐 `/odom`；**RMUL 场地网格 44 万三角面**（RMUL2026 仅 3187）加载慢 | 等 30~60 s 再判断；按 `/livox/lidar → /livox/imu → /imu/data → /odom` 逐段定位 | ICP 本身正常：`pcd point size: 97642, 4866`、`icp_registration initialized` |
| 12 | `ros2 topic hz /livox/lidar` 只有 ~1 Hz + 2.8 s 抖动 | **测量假象**：CustomMsg 每帧 3 万点，`topic hz` 反序列化跟不上 | 看 `/scan`（轻量）或 `/odom` 判断频率 | `/odom` = 8.1 Hz 正常 |
| 13 | `/amcl_pose` 的 covariance ≈ 0 | nav2 `set_initial_pose` 路径**不填协方差**（`amcl_node.cpp` 只设 position/orientation） | 正常现象；但初值给错时 AMCL 难以自纠 → 用 RViz `2D Pose Estimate` 重给 | 源码 L271-281 |
| 14 | `cartographer_node` 启动数秒后 `exit code -6`，日志 `Check failed: sensor_to_tracking->translation().norm() < 1e-5 The IMU frame must be colocated with the tracking frame` | lua 里 `tracking_frame = "livox_frame"`，而 `/livox/imu` 的 `frame_id` 是 **`imu_link`**（URDF：livox 在 `base_link+(0.12,0,0.175)`、imu 在 `+(0.12,0,0.125)`，差 **5cm**；而这 5cm 是 FAST-LIO 的 `extrinsic_T=[0,0,0.05]` 依赖的，**不能靠改 URDF 消除**）→ `sensor_bridge.cpp:136` 的 IMU 共位硬 CHECK 失败 → abort | **2026-09-21 修复**：`tracking_frame = "imu_link"`（官方 `mir-100-mapping.lua` 用 `imu_frame` 同做法）；`/scan` 由 URDF 静态 TF 转到 `imu_link`（纯 5cm 平移）。同时 `TRAJECTORY_BUILDER_2D.min_range` 0.45→0.2（与 p2l 对齐） | **A/B 证据**（沙箱，隔离 `ROS_DOMAIN_ID=42` + 合成 TF/IMU 消息）：旧配置 → `Check failed` + 中止；新配置 → 无 `Check failed`、`Added trajectory with ID '0'` 正常 |
| 15 | **`localization:=cartographer` 一启动就 FATAL**：`Check failed: 1 == reference_counts_.count(key)` / `Key 'pure_localization' was used the wrong number of times` | `cartographer_localization.lua` 把 `pure_localization` 与 `pure_localization_trimmer` 写在了 **`TRAJECTORY_BUILDER_2D`** 上，而 cartographer 是从**顶层 `TRAJECTORY_BUILDER`** 读（`cartographer/mapping/trajectory_builder_interface.cc` 的 `kDictionaryKey = "pure_localization_trimmer"`、`map_builder.cc` 的 `has_pure_localization_trimmer()`）。写错层级的键**永远不会被读**，而 `LuaParameterDictionary` 要求"每个键被读恰好一次" → 析构时 FATAL。另：`pure_localization` 这个 bool 在 2.0 已 deprecated（设了只会打警告） | **2026-09-21 修复**：改为官方写法 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`，不再设 deprecated 的 bool | **证据**：直接跑 `cartographer_node -configuration_basename cartographer_localization.lua -load_state_filename <不存在的路径>` —— 修复前报上述 CHECK；修复后正常走到 `Failed to open proto stream`（说明 lua 已通过解析）。⚠️ 这条此前一直没暴露：`localization:=cartographer` 因缺 pbstream 从未真跑过，**"launch 能解析"掩盖了它** |
| 16 | **全包形态（`lio:=cartographer`）下 RViz 里"车自己在动、地图跟着小车走"**，Gazebo 里车是静止的 | 全包形态的 `odom→base_link` 由 cartographer 的 pose extrapolator 提供（= 上次匹配位姿 + **IMU 二次积分**）。IMU 的重力对齐残差/零偏被二次积分放大 → **静止也在漂**；而漂移又被当作扫描匹配的初始猜测（搜索窗 ±0.2 m / ±30°）→ 位姿继续偏、地图被拖着走 | **2026-09-21 修复**：给全包形态接一路底盘里程计当运动先验 —— lua 里 `use_odometry = true`，`cartographer_sim.launch.py` 新增 `odom_topic` 参数，bringup 在 `lio==cartographer` 时传 `/odom_ground_truth`（仿真底盘里程计；实车换下位机轮速 odom）。cartographer 只用 odom 的**增量**，所以世界系绝对位姿可直接用 | **实测数据（RMUL2026，车静止）**：`odom→base_link` 漂 x `+3.1 cm/s`、y `−2.5 cm/s`（合 **≈4 cm/s**），yaw `2.21°→1.72°/2.3 s` ≈ **13°/min**；同一时刻 `map→odom` **恒定** `(0.088, 0.075, −0.37°)` → **排除回环问题**；`/odom_ground_truth` 的 twist 全 0 → 车确实没动。修后复测：静止 12.1 s 平移仅 3 mm（0.025 cm/s）、yaw 抖动 <0.5° |
| 17 | RViz 里**整张地图跟着小车转/平移，还留残影**；`map→odom` 从 ≈0 变成 **(0.281, 0.149, 30.88°)** | 两件事要分开：① 若显示用的 **Fixed Frame 不是 `map`**（或 `Target Frame` 选成了 `/base_link`），整个场景会跟着车体坐标系转 —— **地图其实没动**（`/map` 的单元格固定发布在 `map` 系里）；② `map→odom` 出现 30° 量级的**持续修正** = **误回环**：RM 场地小且四角/边线高度对称，而原回环参数偏松（`max_constraint_distance 10 m` ≈ 整个场地、`min_score 0.55`、`global_constraint_search_after_n_seconds 10 s`），跨场地的假匹配命中一次就把整条轨迹拧一下 → 地图整体转 + 每次重画留残影 | **2026-09-21**：① 文档写明先把 `Fixed Frame=map`、`Target Frame=<Fixed Frame>`；② **收紧回环参数**（`cartographer.lua`）：`max_constraint_distance 10→4`、`min_score 0.55→0.72`、`global_localization_min_score 0.6→0.8`、`fast_correlative_scan_matcher` 搜索窗 `5m/20°→2m/10°`、`global_constraint_search_after_n_seconds 10→30`。**判据**：重跑后 `tf2_echo map odom` 应始终 ≈0，转弯/走回起点时最多小幅跳一次 |
| 18 | **RViz 里栅格地图绕着小车转**（车原地转最明显），但 `map→odom` 恒定 ≈0、TF 树也完全正确（`map→odom→base_link` 每帧一个父） | **`lio_tf_adapter` 丢了杆臂**：LIO 的 `/odom` 里 `child_frame_id = "body"`，而 body 就是雷达/IMU 的位置（URDF：`livox_frame = base_link+(0.12,0,0.175)`、`imu_link = +(0.12,0,0.125)`；FAST-LIO/Point-LIO 的 `extrinsic_T = [0,0,0.05]` → **body == imu_link**）。本节点却把 body 的位姿直接改名成 `base_link` 发布（配置 `xyz: [0,0,0]`）→ 车**原地转**时"base_link"绕真实中心画半径 **≈0.17 m 的圆**，而地图静止 → 相对运动看起来就是"整张地图绕着车转/地图跟着车" | **2026-09-21 修复**：`lio_tf_adapter/config/lio_tf_adapter.yaml` 设 `xyz: [-0.12, 0.0, -0.125]`（= `T_body←base_link` 的平移，取负号）。两个 LIO 的 `extrinsic_T` 相同 → 同一套杆臂，改一处即可。**验证**：原地转 360°，`tf2_echo map base_link` 的 x/y 应基本不动（修前会画圆） | **出处**：同根因公开案例 nav2 issue #4135（雷达装偏 → 转圈时 map 跟着画圈）；另 cartographer_ros #1170 说明 **map 帧初始朝向每次重启都不同**、[#254](https://github.com/cartographer-project/cartographer/issues/254) 说明 map 锚在第一个子图上 → 解释 `map→odom` 每次是 +30°/-18°/-130° 这种随机值 |
| 19 | **建图中 `cartographer_node` 偶发 `exit code -6`（SIGABRT），同时 RViz 里整张地图跟着车转** | 仿真雷达插件**混用两套时间基**：`header.stamp` 用**仿真钟**（`node_->get_clock()->now()`），逐点 `offset_time` 却用**墙钟**（`boost::chrono::high_resolution_clock` 累计"生成一帧花了多久"，`livox_points_plugin.cpp:149/198`）。FAST-LIO 的用法是 `lidar_end_time = header 戳 + 末点 offset`（`laserMapping.cpp:396-410`），再拿它当 `odom.header.stamp`（:632）→ **`/odom` 的时间戳 = 仿真戳 + 一帧墙钟耗时**：RTF<1 时是 1.3~3 倍（本机 RTF≈0.76）且逐帧随 CPU 负载抖动。LIO 要处理完一整帧才发 odom → "第 k 帧的 odom"经常在"第 k+1 帧的 scan 已处理"之后才到 → 撞 `pose_extrapolator.cc` 的 `Check failed: timed_pose_queue_.empty() \|\| odometry_data.time >= timed_pose_queue_.back().time` → 直接 abort；没 abort 的时段先验按错时刻套用 → 每帧被拖一下 = 地图跟着车转。另外这个假跨度还会被 FAST-LIO 当帧内运动拿去做**去畸变**，把"其实没发生"的运动补偿掉 → 转起来时整帧被拧、odom 带负载相关偏差 | **2026-09-22 修复**（`livox_points_plugin.cpp`）：① CustomMsg 与 PointCloud2 **共用同一个仿真戳**（原来各调一次 `now()`）；② **`p.offset_time = 0`** —— Gazebo 是"一次回调把所有射线全部打完"（`rayShape->Update()`），帧内没有任何运动，0 才是真值，FAST-LIO 于是得到 `lidar_end_time == header 戳` 自洽；③ 删掉 `boost/chrono.hpp`；④ 加载时打一行 `timebase: SIM clock, per-point offset_time = 0` 便于确认新库生效。**需 `colcon build --packages-select ros2_livox_simulation`，然后重启 launch** | **证据**：`~/.ros/log/2026-09-22-00-53-31-*/launch.log`（起步 51 s 后 `exit code -6`）、`2026-09-22-01-12-34-*`（31 s 后 `exit code -6`）；对 `/opt/ros/humble/lib/libcartographer.a` 做 `strings \| grep "Check failed"`，`pose_extrapolator.cc` 只有两条时间序 CHECK，都是跨时间源乱序；插件 149/198 行确认墙钟；FAST-LIO 396-410/632 行确认取用方式 |
| 20 | 上一轮"修地图跟着车转"的**方向错了**：把位姿图回环门槛收紧（`min_score 0.55→0.72`、`global 0.6→0.8`） | 收紧后收尾日志变成 `Score histogram: Count: 0`（`0 computations resulted in 0 additional constraints`）= **一条回环都没命中** → 位姿图完全没有全局锚定，地图只剩 local SLAM 的结果；而被调的门槛属于"已经不工作的东西"，所以怎么调都没用。另外 `use_odometry=true` 把 LIO 的 `/odom` 当先验是**同源冗余**：`/odom` 本来就由同一份雷达点云融合而来，喂回去等于把 LIO 自身的漂移反馈给 cartographer。上游官方"只有 2D 激光"的参考配置 `revo_lds.lua` 就是 `use_odometry=false` + `use_imu_data=false` + `use_online_correlative_scan_matching=true` | **2026-09-22 修复**（`c4a4fd1`）：`cartographer.lua` 改回 `use_odometry=false`（纯 2D 扫描匹配）；回环门槛回到上游 `min_score 0.65` / `global_localization_min_score 0.70`；真正挡"跨场地误回环"的 `max_constraint_distance 4m` 与搜索窗 `2m/10°` 保持收紧。`odom→base_link` 仍由 `lio_tf_adapter` 发，TF 契约不变 | **证据**：收尾日志 `Count: 0`（见 `~/.ros/log/2026-09-22-00-53-31-*/launch.log` 尾部）；[revo_lds.lua](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/configuration_files/revo_lds.lua)；相关公开案例 [cartographer #579](https://github.com/cartographer-project/cartographer/issues/579)、[cartographer_ros #1051](https://github.com/cartographer-project/cartographer_ros/issues/1051) |
| 21 | **cartographer "地图跟着车转"：先验给错了源**（#20 的后续；#19 修的是时间基） | `/scan` 有洞（实测 2.8 Hz、每 ~0.5 s 一个）时，**洞期间必须有东西把位姿推过去**，否则下一帧扫描的初值偏几十度 → 子图按错的旋转角铺下去 = 整张图跟着车转。cartographer 本来就是用 odom 干这件事的，但我们喂的是 **LIO 自己的 `/odom`**：① **同源**（与 `/scan` 来自同一份点云）→ 反馈回路，LIO 漂移直接拖地图；② **晚到**（LIO 要处理完一整帧才发）→ 撞 `pose_extrapolator` 时间序 CHECK → `exit -6`。所以"干脆关掉"（#20）与"用 LIO odom"两个方向都不对，正确的是**换一路独立的里程计** | **2026-09-22 修复（路线①）**：`cartographer.lua` 恢复 `use_odometry = true`，同时把先验源换成**独立底盘/轮速里程计** —— `bringup_sim.launch.py` 的**标准形态**（`mapper:=cartographer` + 有 LIO）也传 `odom_topic:=/odom_ground_truth`（原先只有全包形态传）。仿真 = `gazebo_ros_planar_move` 的底盘 odom（10 Hz、物理插件按自己的时刻产出、**立即到达**）；**实车 = 下位机轮速 odom**（这一条可迁移）。`odom→base_link` 仍由 `lio_tf_adapter` 发，TF 契约不变 | **预期证据（重跑核对）**：① 不再出现 `exit code -6`；② `tf2_echo map odom` 始终 ≈0；③ 收尾 `Score histogram` 不再是 `Count: 0`（回环真的命中）；④ 原地转一圈地图不跟转。**台架偏差**：`planar_move` 的 odom 是**无打滑的理想值**，比实车容易 → 记在 `sim_real_contract.md` §三（后续用打滑/延迟注入槽位补） |
| 22 | **地图"跟着车转 → 又被矫正回去 → 留下残影"**（路线①之后的残余问题）。实测 `map→odom` 的 yaw 呈锯齿：从 ~1° 慢慢涨到 **17°** 再被拉回 ~1°，而平移只有 5~16 cm | 官方调参文档描述的正是这一现象：**局部 SLAM 在一个子图内部走偏 → 全局 SLAM 事后只能部分矫正 → 而"坏掉的子图永久保留"** → 残影。我们的具体成因：先验已换成真值底盘 odom（残差≈0），但**实时相关扫描匹配器的搜索窗仍是"没有可用先验时代"的 ±30° / 0.2 m** —— 它允许匹配器一步跳到**对称场地**的错误朝向 → 局部位姿相对优化轨迹缓慢走偏 → 位姿图周期性拉回（锯齿）→ 子图与矫正后的轨迹不一致（残影）。同时 `ceres_scan_matcher` 的权重还是上游默认 10/40，对"偏离先验"的惩罚不足 | **2026-09-22 修复（四次修正）**：① `real_time_correlative_scan_matcher.linear_search_window 0.2→0.1`、`angular_search_window 30°→5°`（依据 #534 的经验法则：窗口 ≈ 一帧内最坏里程计漂移 +10~50%；**有先验时窗口就该很小**）；② `ceres_scan_matcher.translation_weight 10→1e2`、`rotation_weight 40→4e2`（官方 tuning 文档的示例值：先验越可信，越要 penalize 偏离先验）。**若仍不干净 → 下一步诊断（官方方法论）**：把 `POSE_GRAPH.optimize_every_n_nodes` 临时设为 **0**（关掉全局 SLAM）单独看局部 SLAM —— 地图不再"转回去"（只剩缓慢漂移）⇒ 问题在全局矫正；仍然跟着转 ⇒ 问题在局部 SLAM | **证据**：`ros2 run tf2_ros tf2_echo map odom` 实测锯齿（sim time 769→781：yaw 8.56°→11.36°→14.17°→16.96°→**2.02°**→4.68°→0.51°→…；同时 `/odom_ground_truth` 的 `Subscription count: 1 = cartographer_node` ✓、`exit code -6` 已消失 ✓）；[官方 tuning 文档](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)（"The broken submap is broken forever though" + CeresScanMatcher 权重示例）；[cartographer #534](https://github.com/cartographer-project/cartographer/issues/534)（Mofef：odometry + 实时相关匹配器 + **很小的搜索窗**） |
| 23 | **`map→odom` 被高频甩动**：50 Hz 采样实测 `|Δyaw|` 中位 0.333°、**P95 1.938°**、max **83.2°**、跳变 143 次、峰峰值 83°、单向漂移 **−13.7°/min**；现象是"原地转+走一点点还行，一旦移动地图立刻飞" | 单样本 P95 ≈ 1.94° @50 Hz ⇒ 等效 **≈97°/s 的连续转动**，再夹 83° 级别的**单样本大跳** —— 这**不是"矫正"**（正常矫正应是"很久才跳一次、每次几度"）。最符合的机制是**帧间没有旋转来源**：`/scan` 只有 ~2.8 Hz 且每 ~0.5 s 一个空洞（相邻两帧可差几十度），而我们当时把 `use_imu_data` 关掉了 —— 2D 且无 IMU 时 `PoseExtrapolator` 的旋转外推只能靠 odom+位姿历史，cartographer 维护者/贡献者在 #534 里明确指出这个组合在 2D 下不可靠。**对照组：slam_toolbox 没有这个问题**，因为它从 TF（`odom→base_link`，lio_tf_adapter 发）直接取运动模型，不走 cartographer 的 IMU-tracker 路径 | **2026-09-22 修复（五次修正）**：`TRAJECTORY_BUILDER_2D.use_imu_data = true`。当初关它是为了躲 `pose_extrapolator.cc:229 Check failed: time >= imu_tracker->time()`，而那个崩溃的根因是①**插件时间基 bug**（`cb24ba8` 已修）②**LIO odom 晚到**（已换成独立底盘 odom，`34c64a6`/`9a9b580`）→ CHECK 的前提消失。`tracking_frame=imu_link` 本来就是为 IMU 共位 CHECK 选的，正好用上 | **证据**：`python3 tools/monitor_map_odom.py --csv /tmp/mo.csv` 的 112 s 记录（判读=高频抖动）；IMU 接线已核（`sentry_robot_sim.xacro:246-258`：`frame_name=imu_link`、`~/out:=/livox/imu`、100 Hz、`imu_joint` rpy=0）；**待重跑核对**：P95 应降到 ≈0.1° 量级、不再有几十度大跳。⚠️ **同时必查（30 秒，排除环境因素）**：`ros2 node list` 有没有残留的 `static_transform_publisher`（T1 桥会发 `camera_init→map` 与 `body→odom`，一旦残留就让 `map`/`odom` **多父边** → 查找在两条路径间跳，症状与本文完全一致）；`ros2 topic echo /tf_static --qos-durability transient_local` 看静态边清单；`ros2 run tf2_tools view_frames` 看每帧父边数 |
| 24 | **打开 IMU 后 cartographer 立刻 FATAL**：`imu_tracker.cc:67 Check failed: (orientation_ * gravity_vector_).z() > 0. (0 vs. 0)`（栈：`ImuTracker::AddImuLinearAccelerationObservation ← PoseExtrapolator::AdvanceImuTracker ← AddPose ← Node::PublishLocalTrajectoryData`）。现象：`Added trajectory` → `Inserted submap (0,0)` → 1 帧后 abort，建图完全起不来 | `ImuTracker` 首个加速度样本的 `alpha = 1 - exp(-delta_t / imu_gravity_time_constant)`，而首帧的 `delta_t = now - Time::min()` 极大 → **alpha≈1 → `gravity_vector_` 被直接赋成"第一帧 IMU 的 linear_acceleration"**；只要那一帧是 `(0,0,0)`，随后 `(orientation_*gravity).z() > 0` 必失败。**与"FAST-LIO 用同一路 IMU 没问题"并不矛盾**：FAST-LIO 是**多帧求均值**做重力初始化，天然容忍启动几帧零值；cartographer 拿**第一帧**当基准。⚠️ 同一次日志里还有强线索：`ordered_multi_queue.cc:172 All sensor data for trajectory 0 is available starting at '621355974448820000'` —— `621355974.448820 s` = **墙钟 .NET ticks 被当成纳秒**（`.NET 1970 纪元 621355968 s + 仿真时间 6.448820 s`），说明 scan/imu/odom 三路里有**一路的时间戳不是仿真钟**（与插件那次同源的时间基问题，待三路 echo 确认） | **2026-09-22 处置（六次修正）**：`use_imu_data` 先改回 **false** 解锁建图；同时把"四次修正"改过的局部匹配参数（5° 窗口、1e2/4e2 权重）**恢复上游默认**（0.1/20°、10/40）—— 21:0x 的实测证明"更信任先验 + 小窗口"没让 map→odom 变稳（P95 1.94°、max 83°），反而可能剥夺匹配器的纠正能力。**打开 IMU 的前置条件**：① `/livox/imu` 前几帧的 `linear_acceleration` 不是 `(0,0,0)`；② 三路 + `/clock` 时间戳同轴（用下面的命令核） | **待核（30 秒）**：`ros2 topic echo /livox/imu --field linear_acceleration \| head -20`（重启 launch 时立刻抓，看前几帧是不是零）；`ros2 topic echo /clock --once`、`/livox/imu`、`/scan`、`/odom_ground_truth` 各抓一次 `header.stamp` 对比（有没有 `6213559xx` 这种值）。FAST-LIO 侧佐证：同一次日志里 `IMU Initial Done` 正常（说明均值可用） |
| 25 | **`/segmentation/obstacle` 只有 0.31 Hz、单次空洞最长 20.8 s（仿真时间），而 CPU 92% 空闲** —— 直接后果：`/scan` 长期 1.7~3.3 Hz 且带秒级空洞 → cartographer 在扫描之间"朝向不转、每来一帧扫描补一刀"（`map→odom` 84 次正向大跳、累计漂 66°、单步最大 76.8°）→ 表现就是"地图跟着车转 / 地图飞"（这是 #23 的上游原因） | 分层实测（同一个 192 s bag，全部按**仿真时间**算）：① 插件 `/livox/lidar/pointcloud` = **10.00 Hz、0 空洞**；② linefit `/segmentation/obstacle` = **0.31 Hz、49 个 >0.5 s 空洞、最长 20.8 s**；③ p2l `/scan` = **68 = 68**（一帧不丢）⇒ **丢帧点唯一：linefit**。但**不是算力**：用 `tools/seg_bench_offline.cc` 把录到的真实点云离线喂给 linefit 核心库，**`segment()` 平均 1.01 ms/帧、最慢 1.80 ms**（30000 点、完全相同的 sim 参数）。⇒ 卡的是**大点云的订阅交付**：**同一个发布端，reliable 订阅者（rosbag2 录制）拿到 2182/2182 = 100%**，而 **best-effort 订阅者（linefit 的 `SensorDataQoS`、`ros2 topic hz`）只拿到 3%~26%**（隔离回放里 `topic hz` 实测 2.6 Hz、最大空洞 5.9 s，linefit 60 s 内一帧未出）。原理：480 KB 的 `PointCloud2` 必须分片，best-effort 不重传 ⇒ 丢一个分片整帧丢；再叠加 `SensorDataQoS` 的 KEEP_LAST(5)，一卡就雪崩。**对照组**：FAST-LIO 订同一个插件的大 `CustomMsg` 用的是**默认 RELIABLE(depth 20)**，10 Hz 一帧不丢 | **2026-09-22 已修（管道类，1 行）**：linefit 订阅由 `rclcpp::SensorDataQoS()`（BEST_EFFORT, depth 5）改为 **`rclcpp::QoS(rclcpp::KeepLast(10)).reliable()`**，与插件发布端对齐（= FAST-LIO 的做法）；`colcon build --symlink-install --packages-select linefit_ground_segmentation_ros` 通过。**验证**：`ros2 topic info /livox/lidar/pointcloud --verbose` 里 linefit 的 Reliability 应变成 RELIABLE；`ros2 topic hz /segmentation/obstacle` 应从 0.3 Hz 升到 ~7.5 Hz（墙钟）。⚠️ **更正**：我早先说"`/segmentation/ground` 在发 426 KB"是把 ground/obstacle 搞混了 —— 离线实测 ground 只有 ~700 点（≈11 KB），**不构成带宽问题，因此不动它**；真正的浪费在 obstacle 云（26649 点里 88% 是插件发的 `(0,0,0)` 假点，≈426 KB/帧）。**另有仿真保真度问题（需拍板）**：插件把**无回波射线**也填成 `(0,0,0)` 点发出去 —— 实测**每帧 78.4%**（23500/30000）是这种假点；真实 Livox 驱动只发有效回波。后果：障碍云 26649 点里 88% 是假点（带宽/CPU 白烧）、地面只剩 ~700-900 点（89~98% 被判障碍）。修法 = 出帧时跳过无回波射线（CustomMsg/PointCloud2 都不发） | **证据**：`python3 tools/analyze_slam_bag.py --bag .tmp_bags/startup`（逐段率/空洞）；`tools/seg_bench_offline.cc`（1.01 ms/帧）；`ros2 topic info /livox/lidar/pointcloud --verbose`（RELIABLE 发布 / BEST_EFFORT 订阅）；`ros2 topic hz /segmentation/obstacle` 实测 0.45~1.04 Hz、max 4.232 s；bag 内 2182 帧的 `(0,0,0)` 点占比 ≈78.4% |
| 26 | **内部小墙"扫到就有、被挡住/角度扫开就没了"**（外侧大墙不受影响） | cartographer 的 2D 栅格是**每帧投票**的概率图：命中 +1（`hit_probability`）、穿过 −1、"**无回波**"则把 **0~`missing_data_ray_length`** 整段标为**自由**。本链路 `/scan` 里 **`inf` 占 37.3%**（1462 bin 中 546 个）—— 来源是"地面被 linefit 去掉"+"MID360 上视射线打空"，都属物理正常。我们原来 `missing_data_ray_length = 3.0` ⇒ **每帧有 1/3 的方向在擦 3m 内的地**。实测命中距离 **P25=1.8 / P50=2.3 / P75=3.1 m**、4~10m **仅占 16%** ⇒ **约 3/4 的命中都落在 3m 常清区内**，所以"死的是内部小墙"；外墙多在区外、命中票也多，故留得住。再叠加命中稀疏（每帧 ~3100 障碍点 / 1462 bin，且车一转同一面墙落进不同 bin）而清除票每帧必到 ⇒ 概率在阈值附近摆动 = 闪烁 | **2026-09-22 修复（七次修正，一组变量）**：`cartographer.lua` —— `missing_data_ray_length 3.0 → 1.0`（常清区缩到贴身，不再擦 1m 外的墙）、`hit_probability 0.55 → 0.62`、`miss_probability 0.49 → 0.45`（命中更粘、清除更弱）。**保持 `insert_free_space = true`**（只调弱清除，不取消，否则失去清动态物的能力）。`num_accumulated_range_data`（1→3，稀疏特征多帧累积）**留作下一组变量** | **证据**：`bag` 内 `/scan` 实测 —— 有回波 62.7%（`inf` 37.3%）；距离分位 P5=1.0/P25=1.8/P50=2.33/P75=3.12/P95=5.56m；0.2~1m 5%、1~2m 32%、2~4m 47%、**4~10m 16%**。机制与处置详见 `docs/debug_fastlio_cartographer.md` §5.2。**⚠️ 2026-09-22 二次结论：七次修正已回退（方向错）**。用 8 帧 bag 逐带统计（**已排除插件发的 `(0,0,0)` 假点**，地面按实测 `z=-0.226m` 标定）：raw→障碍保留率 —— 0~3cm **0.1%**、3~8cm **2.8%**、8~15cm 59.1%、15~30cm 43.7%、30~60cm 92.8%；即 **`linefit` 把 ≤8cm 的低矮特征基本删光**（`max_dist_to_line=0.1` 的 10cm 容差 + **`sensor_height=0.275` 而实测雷达离地仅 0.226cm→差 4.9cm**），之后 p2l 的"每角度 bin 只留最近点"再砍 60~90%（近处残余遮住远处）。另有**几何**原因：MID360 下视 FOV 仅 −7°，高 h 的墙需 d ≥ (0.226−h)/tan7°（h=0.15m→0.62m）⇒ 越近越看不见（真机同样）。**下一步**：先修数据侧（`sensor_height 0.275→0.226`、`max_dist_to_line 0.1→0.05`，感知参数待拍板），验收=重录 bag 量"低 3~8cm"保留率是否显著上升；确认低墙进 scan 后再回来谈栅格清除/累积 | **2026-09-22 三次结论（已改感知侧）**：`segmentation_sim.yaml` 的 `sensor_height 0.275→0.226`（实测标定）、`max_dist_to_line 0.1→0.05`；这两个是 linefit 的地面分割参数，**与 FAST-LIO 无关**（不同节点/话题/无共享状态），也不改"什么算障碍"的语义。`analyze_slam_bag.py` 新增 **⑤ `/map` 留存分析**（留存曲线 + occupied→free 闪烁统计 + 栅格取值分布），用来量化"留不住"并分辨"被主动擦除"（⇒ 调 cartographer 清除/命中）还是"数据没进来"（⇒ 修感知/几何）；录制时请把 `/map` 也录上。**几何那条（MID360 下视 FOV −7°）任何参数都修不了**：矮墙"离得够远才看得见"，真机同样 ⇒ 建图时别贴着矮墙走 | **2026-09-22 四次结论（已量化，判定"被主动擦除"）**：`.tmp_bags/ret`（222 帧 `/map`）实测 —— 曾占据 3779 格、结束仍占据 **49%**（**51% 被擦掉**）；留存曲线 +2/+5/+10/+20 帧 = 87.5/74.4/63.9/**63.8%**；闪烁（occupied→free）中位 2、P95 6、max 12，**87% 的曾占据格子都闪过** ⇒ 判为**被主动擦除**（每帧 37% 无回波光束 × `missing_data_ray_length=3.0` 把 0~3m 标自由），不是"数据没进来"。同时确认：`/scan` 已 **10.00Hz/最大空洞 200ms**（QoS 修复彻底生效）、`map→odom` 峰峰值 7.04°/单步 2.54°（定位稳）。**处置**：临时置 `insert_free_space = false` 做判别实验（预期被擦掉→~0、留存→≈100%），确认后改为`missing_data_ray_length 3.0→0.5~1.0` + `hit/miss 0.62/0.45`，并把 `num_range_data`/`optimize_every_n_nodes` 由 30 调回上游 90（扫描率提高 10 倍后，30 会让子图每 3 秒就换一个，重叠缝增多） | **2026-09-22 五次结论（判别实验已完成 + 已按平衡点修改）**：`insert_free_space=false` 诊断跑（`.tmp_bags/ret2`，183 帧 `/map`）—— 结束仍占据 49%→**89%**、被擦掉 51%→**11%**、留存 +20 帧 63.8%→**78.9%**、闪烁≥1 由 87%→54%，但 **自由格子(0~30) 35856→0** ⇒ 关掉清除连自由空间一起没了（"栅格一点点出来很艰难/全灰"就是这个）。⇒ 机制确认：**留不住=被清除**，但正确处置是"保留清除 + 调弱"：`insert_free_space=true`、`missing_data_ray_length 3.0→1.0`、`hit/miss 0.62/0.45`，并把 `num_range_data`/`optimize_every_n_nodes` 30→**90**（上游默认；10Hz 扫描下 30 = 每 3 秒一个子图/一次优化，重叠缝暴增）。**复测判据**：留存 +20 帧 ≥90%、被擦掉 <10%、且**自由格子 >0 且明显增多**（灰色变少） | **2026-09-22 六次结论（再定量 + 九次修正）**：逐帧重建 p2l 的每-bin 选点与高度后测得 —— 低矮特征(离地3~15cm)的 2D 格子每帧 **90.4% 被命中**，只有 **36.3% 会被"命中高度>25cm 的光束从上方穿过"**（命中:清除 ≈ 1:0.4）⇒ **主导清除是 37% 的无回波光束 × `missing_data_ray_length`**；关键认识：真实 2D 雷达"无回波=空"成立，但 **3D FOV(−7°~+52°) 转 2D 时无回波多来自"朝天打空"** ⇒ 假设不成立 ⇒ 半径必须小。另：`num_accumulated_range_data` 不改变命中/清除**比值**，不是这里的旋钮。现场观感为"墙变淡/变灰、慢慢化掉"（清除仍压过命中）⇒ **九次修正**：`missing_data_ray_length 1.0→0.5`、`hit 0.62→0.68`、`miss 0.45→0.40`（odds 比 1.27→2.0→**3.2**；清除面积约 1/6）。复测仍用第 ⑤ 段：留存 +20 帧 ≥90%、被擦掉 <10%、自由格子 >0 | **⚠️ 2026-09-22 七次结论（九次修正失败 + 回滚；教训：一次只改一项）**：九次修正把 5 项一起改了（`missing_data_ray_length 1.0→0.5`、`hit 0.62→0.68`、`miss 0.45→0.40` **加上** `num_range_data 30→90`、`optimize_every_n_nodes 30→90`），实测（`.tmp_bags/ret3`，110 帧）**留存反而从 49% 掉到 30%**、被擦掉 70%、**曾占据格子 3779→2291**（连"曾占据"都变少 ⇒ 是**证据变少**而非清除变强）。指向我多加的两项（子图/优化频率 30→90：子图覆盖范围变大后，超出子图栅格的数据会被裁掉，墙的证据成片减少）⇒ **已回滚到 30/30**，保留清除改弱那三项。**教训：一次只改一项、其余保持不动**。**另一条可选路线**：若只要墙、不在意自由空间，`insert_free_space=false` 实测留存 89%（ret2），**2026-09-22 八次结论（解耦，取代"不清除"路线）**：栅格里"擦旧墙"与"标空地"是**同一个写**（都给射线途经格写 miss），所以开关二值 ⇒ 关掉就没自由空间（自由格=0、全灰）。但**强度可解耦**：白格真正来自**打到东西的射线**（真观测过的空地），而**无回波光束**（占 37%，多来自朝天打空）标的 0~`missing_data_ray_length` 只是**假设**、没观测过，是**乱擦墙的元凶**。⇒ **十次修正**：保留 `insert_free_space=true`，把 `missing_data_ray_length 0.5→0.05`（≈关掉假想空地）。预期：留存 ≥ 不清除时的 89% 且**自由格子 >0**（两全）。代价是自由格子=0（地图全灰），导航侧需把 `track_unknown_space` 关掉/`allow_unknown: true` 才可用 |





---

## 二、静默失效类（最危险：不报错但不生效）

| 现象 | 根因 | 修复 |
|---|---|---|
| 参数写在 yaml 里却"没生效" | **节点名跨版本改名**：Galactic `recoveries_server` → Humble **`behavior_server`**；`recovery_plugins` → `behavior_plugins`；插件类型 `nav2_recoveries/*` → `nav2_behaviors/*`。对不上的整段被**静默忽略** | 三份 sim 变体已改名并把 `robot_base_frame` 改回 `base_link_fake`、`max_rotational_vel: 3.0`。**佐证**：日志里创建了 4 个插件（=代码默认列表），而 yaml 只写了 2 个 |
| **建图模式下 `/map` 被旧图抢占** | `map_server` 的启动条件只判断 `localization`（建图模式为空 → 条件成立）→ 它把磁盘旧 pgm 发到 `/map`，与 slam_toolbox 抢；`map_saver_cli` 可能存下**旧图** | 条件加 `mode=='nav'`（已修，5 种组合验证） |
| `/map_save` 无处可写 | `fastlio_mid360_sim.yaml` 里 `map_file_path` 被注释 → ICP 在 RMUL2026 上没有底图 | bringup 按 `world` 自动注入 `PCD/<world>.pcd`（与 ICP 的 `pcd_path` 同路径） |
| `nav_rviz`/`lio_rviz` 都给 `False` | 屏幕上一个可视化都没有（曾误判"没有机器人"） | runbook §0.4：mapping 用 `lio_rviz`，nav 用 `nav_rviz`（默认 True），**只开一个** |
| `velocity_smoother.odom_topic: "Odometry"` | T2 已把里程计统一为 `/odom`，旧键名是遗留 | 改为 `odom` |
| **cartographer 的栅格被硬 remap 到 `/cartographer_map`** | nav2 `static_layer`、`map_saver_cli`、RViz 的 Map 显示项都订阅 `/map` → 边建边导 `mapper:=cartographer` 时 costmap 无静态图、建图落盘存不到东西 | **2026-09 修复**：`cartographer_sim.launch.py` 默认发 `/map`；纯定位另有 `map_server` 时用 `occupancy_grid_topic:=/cartographer_map` 覆盖 |
| **纯建图 `mode:=mapping` 下 `nav_rviz:=True` 却没有任何 RViz** | 三形态拆分后 `mapping` 不再启动 nav2，而 RViz 原先由 nav2 的 `rviz_launch` 带起 | **2026-09 修复**：bringup 为 `mode=='mapping' and nav_rviz=='True'` 单独补一块 RViz（`nav2.rviz`） |
| **`global_obstacle:=scan` 时 global 图累积幽灵障碍** | 新增该槽位时把 local 的 `obstacle/raytrace_max_range: 6.0` 照搬到 global；而 global 是**全图**（13×10 m），raytrace 只能清 6 m 内的旧标记 | **2026-09 修复**：改为 **10.0 m**（与 `p2l` 的 `range_max` 对齐）——「扫描能看到多远，就要能清多远」 |
| **T6 撤销**：`base_link_fake` 不是脏帧 | `fake_vel_transform` 20 Hz 发 `base_link→base_link_fake`（含云台转角），并做 `/cmd_vel → /cmd_vel_chassis` 旋转；角速度非零时按 `spin_speed` 原地转底盘 = **哨兵小陀螺**。改成 `base_link` 会丢功能 | `docs/tf_interface_contract.md` 已撤销 T6 并写明理由 |
| **感知链断掉 → costmap 冻在最后一帧：无报错、不停车（静默失效）** | ① nav2 障碍源的 `expected_update_rate` 默认 **0 = 不检查**（`observation_buffer.cpp` 的 `isCurrent()` 直接 return true）；② `ObservationBuffer` 在 `observation_keep_time=0` 时**永远保留最后一条**（`purgeStaleObservations()` 只 `erase(++begin, end)`）→ 每轮 costmap 更新把**同一帧旧点云**重新 mark；③ `/scan` 是**串行单点**（插件 → `linefit` → `p2l`），local 没有第二来源 | **2026-09 修复**：① 所有障碍源加 `expected_update_rate: 0.5` → 源停即 `current_=false` → `controller_server.cpp` / `planner_server.cpp` 拒绝算速度 → 已有 `velocity_smoother.velocity_timeout: 1.0` 发零速停车 + 日志 WARN；② 新增 `local_obstacle:=scan\|cloud\|both` 槽位（`cloud` = 点云直投，不经 `p2l`，作第二来源）。验证：`pkill -f pointcloud_to_laserscan` 后应出现 WARN 且 ~1 s 内停车 |
| **`p2l` 的 `range_min: 0.45` ⇒ 贴身 45cm 既看不见、又被清成 free** | 小于 `range_min` 的点被 **丢弃**（`pointcloud_to_laserscan_node.cpp` 的 `continue`），该角度 bin 保持 `inf`；`inf_is_valid: true` 时 nav2 把 `inf` 换成 `range_max-ε`（=10 m，`obstacle_layer.cpp` 的 `laserScanValidInfCallback`）→ 沿射线**一路 clear**。原值 0.45 的理由是"避开 38cm 地面最近点"，但 `p2l` 的输入已是 `linefit` **去地面后**的 `/segmentation/obstacle`，该理由不成立 | **2026-09 修复**：`range_min: 0.45 → 0.2`（与车体半径 0.20、`linefit` 的 `r_min` 对齐 → 盲区缩到车体内部）；同时 `scan_time: 0.3333 → 0.1`（与 10 Hz 传感器一致） |
| **`missing_data_ray_length` 在本链路里是空转的（八~十次修正全白改）** | 逐行读源码：它只在 `local_trajectory_builder_2d.cc` 的 `range > max_range` 分支生效（把超距回波截到该距离当 miss）。而 `/scan` 的 `range_max = 10.0`（p2l）< `TRAJECTORY_BUILDER_2D.max_range = 12.0` ⇒ **该分支永不执行** ⇒ `range_data.misses` 恒空。另外 p2l `use_inf:true` 把无回波 bin 发成 `inf`，`msg_conversion.cpp` 的 `LaserScanToPointCloudWithIntensities` 用 `range_min <= r <= range_max` 过滤 ⇒ **`inf` 直接丢弃**（既不命中也不清除） | **2026-09 十一次修正**：`cartographer.lua` 注释已改写并标注"别再拿它当旋钮"；真正的旋钮是 `hit_probability` / `num_range_data` / `insert_free_space`（见 `docs/debug_fastlio_cartographer.md` §5.2.4） |
| **`/map` 的取值上限是 75，永远不出现 100** | `msg_conversion.cpp::CreateOccupancyGridMsg` 的 `value = round((1 - color/255)*100)`，而 `color` 是 `DrawToSubmapTexture` 的 `delta = 128 - ProbabilityToLogOddsInteger(P)` 在**暗红底上预乘合成**的结果 ⇒ 实测映射：`P=0.5→50、0.68→58、0.80→65、0.90→75`。**任何 `lethal_threshold: 100` 之类"取满值"的消费者永远看不到障碍** | 阈值按 `>=65`（= P>=0.80）判读；§5.2.4 的所有"占据"统计都用这个口径 |
| **★ 墙"留不住"的真凶：本项目把 `miss_probability` 从上游 0.49 调到了 0.40，单张清除票强度是上游 ~10 倍（命中/清除比 5.0→1.9）；又叠加 `num_range_data 90→30`（证据窗口砍到 3 秒）** | upstream `trajectory_builder_2d.lua`：`hit 0.55 / miss 0.49 / num_range_data 90`，hit:miss 的 log-odds 比 = `+0.201 : −0.040`（**1 次命中顶 5 次清除**）；本项目 `0.68/0.40` = `+0.754 : −0.405`（1 次命中只顶 1.9 次清除）。而"调猛清除"的理由是**已证伪**的"无回波 × `missing_data_ray_length` 乱擦墙"。另外 `nrd=30` 让"自由空间"也长不出来（`0.68/0.49/30` 只剩 31 个自由格）——这才是上一轮误判"弱清除会让地图没空地"的原因 | **2026-09 十二次修正**：`miss 0.40 → 0.49`（回上游）+ `num_range_data 90 → 300`（30 秒窗口），`hit` 保持 0.85。离线同轨迹（ret4）：留存 +2 帧 69.1%→**94.3%**、+20 帧 42.2%→**75.2%**、闪烁中位 3→**1**、实心墙格子 1087→**2585**、自由格 7664→6556（**不损失**）。即"`insert_free_space=false` 的 89% 效果"与"白格自由空间"同时拿到 |

---

## 三、地图资产与坐标系（含"幽灵墙"完整证据链）

| **★ 墙"运行久了就没了"：上游 2D 概率栅格没有任何"已占据格子免清"机制，`insert_free_space` 又是全有/全无** | 逐行源码：`insert_free_space=true` 时对每条 return 都 `RayToPixelMask(origin→hit)` 并把 miss 表写满沿途每个格子；`false` 时在 miss 循环前直接 `return`。没有中间档、没有"别擦已占据格子"的开关。参数只能拉长时间常数（离线长时程：最好的 weak-clear 配置 +2 帧 94% → **+100 帧 46%**；`insert_free_space=false` 才 100% 但自由格=0） | **2026-09 十三次修正**：工作区内 fork 核心 `src/rm_localization/cartographer/`（`third_party/` 保持原样），插入器新增 `min_probability_to_clear`（P≥阈值免清，0=上游行为）。离线同轨迹预测 `0.68/0.49/3000/0.80`：**留存 +2/+20/+100 帧 100%/99.9%/100%、闪烁 0、自由格 5037、占据 4794**。副作用见 `PATCH_README.md` 。**2026-09-23 现场确认：用户反馈"这个地图是可以的"（墙不再随时间化掉）** |
| **nav2 陷阱：`static_layer` 的 `lethal_cost_threshold` 默认 100 + `trinary_costmap: true`，而 cartographer 的 `/map` 上限只有 75 ⇒ 每一面墙都被判成 FREE_SPACE** | `msg_conversion.cpp::CreateOccupancyGridMsg` 的 `value = round((1-color/255)*100)`，`color` 经暗红底预乘合成后上限对应 P=0.9 ⇒ 出图最大 **75**；nav2 `static_layer` 用 `map >= lethal_cost_threshold` 判致命 | 用 cartographer 的图喂 costmap 时设 **`lethal_cost_threshold: 60`**（社区 issue #628 用的就是 60）。本仓库 `nav2_params*.yaml` 尚未设置，属待办 |

### 3.0 两个都叫 `odom` 的坐标系并不一致（**建图图/导航图对齐的坑**）

| 来源 | 出生点处数值 | 性质 |
|---|---|---|
| `/odom_ground_truth`（话题，`frame_id: odom`） | `x=4.30, y=3.35` | **世界坐标**（= 出生点在世界里的位置） |
| `/tf` 的 `odom→base_link` | `x=-0.106, y=0.007` | **出生点相对系** |

而 cartographer 的 `map` 系 = **出生点相对系**（实测 `map→odom ≈ 恒等`，SLAM `/map` 的内容整体比世界坐标偏 `−(4.3, 3.35)`）。
⇒ ① 离线复现必须换算系（`tools/replay_scan_grid.py --pose tf` 默认用 `/tf` 复合，**别用 `gt`**）；
② **cartographer 建出来的图与 `map/RMUL2026.pgm`（世界系，`origin: [2.68, 0.228]`）天然差 `(4.3, 3.35)`** ——
纯建图落盘、替换底图、给 `amcl_init_x/y` 初值时都要意识到这一点。

---

## 三、地图资产与坐标系（含"幽灵墙"完整证据链）

### 3.1 三场地的 map 系约定不同（**关键坑**）

判定方法：**场地 STL 的世界包围盒** vs **pgm 已知区域（非 205 像素）包围盒**，尺寸 + 位置双证据：

| world | 场地 mesh 世界 bbox | pgm 已知区域 bbox | 判定 | **AMCL 初值** |
|---|---|---|---|---|
| `RMUC` | x[0,29.20] y[0,15.20] | x[-6.35,22.50] y[-7.60,7.45] | mesh−spawn(6.35,7.6) ≈ pgm → **出生点系** | **(0,0,0)** |
| `RMUL` | x[0.51,14.23] y[-1.25,9.30] | x[-3.75,9.80] y[-4.49,5.96] | mesh−spawn(4.30,3.35) ≈ pgm → **出生点系** | **(0,0,0)** |
| `RMUL2026` | x[2.20,14.80] y[0.20,8.80] | x[2.68,14.68] y[0.23,8.43] | mesh **直接**等于 pgm → **世界系** | **(4.3,3.35,0)** |

→ 已实现 **按 `world` 自动注入 AMCL 初值**（`set_initial_pose: true` + `RewrittenYaml` **全路径** `amcl.ros__parameters.initial_pose.*`，避免叶子短名误伤）。

### 3.2 RMUL2026 的"幽灵墙"（规划失败的根因）

- `RMUL2026.pgm` 在 **x≈5.2** 有一条 **1 像素宽竖直虚线**（y≈1.8→6.0，带小缺口）；
- **RMUL2026 世界网格在该处零顶点**（x∈[5.05,5.35]、y∈[2.8,4.0] 顶点数 = **0**，任何 z）→ 世界不存在这道墙；
- 经 `robot_radius=0.2` 内切膨胀后走廊被封死：**r=0.20 目标不可达 / r=0.15 可达**；
- 地图被切成 **3 块连通域**（24246 / 3692 / 693 格），目标点落在另一块 → 必然 `failed to generate a valid path`；
- 结论：该 pgm 更像**官方场地平面图**（含分区虚线），不是 sim 建图产物 → **应在 sim 里重建**（见 §六）。

### 3.3 "一开始就有完整地图"是正常的

nav 模式的地图来自 `map_server` 加载的**磁盘既有 pgm**（`src/rm_nav_bringup/map/<world>.pgm`），不是实时扫描。实时建图要 `mode:=mapping`。

---

## 四、架构改动与新增工具

### 4.1 `mode` 拆成三种场景形态（"地图从哪来"是分界线）

| `mode` | 中文 | 在线 SLAM | nav2 | `map_server` | 重定位 | `map→odom` 来源 |
|---|---|---|---|---|---|---|
| `mapping` | 纯建图 | ✅ | ❌ **不启动**（省 CPU） | ❌ | ❌ | 在线 SLAM |
| `slam_nav` | 边建图边导航 | ✅ | ✅ | ❌ | ❌ | 在线 SLAM 直接喂 costmap |
| `nav` | 先建图后导航 | ❌ | ✅ | 仅 `icp` / 留空 | ✅ | 所选重定位模块 |

- 真值表已逐组合验证（8 种 `mode`×`mapper`×`localization`）；
- QoS 已核：slam_toolbox 与 cartographer occupancy_grid 的 `/map` 均 `transient_local`，与 costmap `static_layer` 订阅匹配。

### 4.2 新增装配级参数 `spin_speed`

`fake_vel_transform` 的小陀螺固定角速度：**仿真的 `planar_move` 没有基线自转**，任何角速度修正都会让底盘按 5 rad/s 转，而**仿真里没有云台补偿、雷达跟着转**（10 Hz LIO 每帧承受 ~29°）→ 排查导航问题先用 `spin_speed:=0.0`。

### 4.3 新增工具

| 工具 | 作用 | 验证 |
|---|---|---|
| `tools/check_map_reachable.py` | 地图连通域/目标点可达性/推荐可达目标点（含净空）；也可当**幽灵墙检测器** | RMUL2026 判出 3 块连通域、目标不可达；RMUL 用起点 (0,0) 落在 64.5% 大连通域（反向印证出生点系） |
| `tools/pcd_to_grid_map.py` | LIO 的 3D `.pcd` 切层投影 → `.pgm/.yaml`（**第三条 2D 地图来源**），带与已有图的比对指标 | RMUL.pcd → 与 slam_toolbox 的图**同一坐标系**（最佳平移 1~2 格 = 5~10cm），±1 格容差下参考图墙覆盖 **84.7%** |

### 4.4 文档新增索引

- `docs/architecture.md`：§3.2.1 三层职责（lio/localization/mapper）、§3.2.2 2D/2.5D/3D 与降维、§3.2.3 "3D 怎么用"、§3.2.4 感知与决策接口、§3.2.5 点云多路切分/2D 为何能承载 3D/两张 costmap、§3.2.6 插件协议与调参/算力/行业实践、§3.2.7 坡道隧道的 2.5D 方案与落位；
- `docs/smoke_test_runbook.md`：§0.4 RViz 开关、§0.4.1 spin_speed、§0.5 地图资产与坐标系、§0.6 选点先验连通域、§0.7 三形态对照、§1.1.1 LIO 点云投影、§1.2 边建图边导航、§9.1/9.2 判据、§10 错误对照（新增多行）、§12 实测记录；
- `docs/tf_interface_contract.md`：T1–T5 已实施，**T6 撤销**。

---

## 五、概念澄清（本轮讨论的结论速记）

1. **三层职责**：`lio` 出 `odom→base_link`（高频连续、会漂）；`localization` 出 `map→odom`（低频、全局不漂）；`mapper` 造 `/map`（在线建图时**兼任**发 `map→odom`，因此 `slam_nav` 与 `nav` 的重定位模块**绝不能同时开**）。
2. **LIO 不只能定位**：FAST-LIO/Point-LIO 是 LiDAR-Inertial SLAM，跑里程计同时维护 **3D 点云地图**（`PCD/<world>.pcd`）。2D 地图有三条来源：① 在线 2D SLAM ② LIO 点云切层投影 ③ 外部/官方平面图。
3. **降维的判据**：降维是否无损，取决于**被丢掉的维度是否影响决策正确性**。平地 → 无损（z 只影响"挡不挡"）；坡道（z 的变化率）、隧道（z 的分布）→ 有损 → 需要 2.5D/3D。
4. **2D 栅格不"承载"3D 信息**：它只存**一个由 3D 算出的判断**（占据/自由/未知的**类别编码**）；3D 语义藏在**规则**里（高度带、地面分割、投影、时间衰减）。单格无法区分"横梁 z=1.5"和"纸箱 z=0.1"。
5. **代价的数值从哪来**：不是降维给的，而是后续图层给的 —— `static_layer` 翻译值域 → 多层按 **max** 合并 → `inflation_layer` 生成 `252·exp(−scale·(d−r_in))` 的梯度（global `scale=8, r_in=0.2`；0.24m→183、0.40m→50、0.60m→10）。
6. **`/map` 与 costmap 标尺不同**：`/map` 是 **-1/0~100**（未知/自由/占据概率），costmap 是 **0~255**（0 自由、1~252 代价、253 内切、254 致命、255 未知）。**灰色能不能走由 `allow_unknown` 决定**（我们设 true）。
7. **205 像素的灰/白**：`free_thresh=0.25` 时 `occ=0.196<0.25` → **自由(白)**；经典 `free_thresh=0.196` 时才判为**未知(灰)**。这解释了"调着调着灰变白"。
8. **时间维 vs 刷新率**：刷新率 = 多久重算/发布（costmap update/publish）；时间维 = 表示里带不带"何时观测/何时过期"（STVL `voxel_decay`、raytrace 清除）。要把三个率分开：**传感器率**（触发 mark/clear）、**update_frequency**（写主图、结算衰减）、**publish_frequency**（仅可视化）。
9. **nav2 的 2D 契约**：规划/控制/行为插件只拿 2D costmap。想用 3D：① 加**图层插件**（改动小、一致性最好）② 加**规划器插件**（接口仍 2D，插件内部自订阅 3D，但 BT/behavior/controller 仍按 2D 走）。`nav_msgs/Path` 本身是 3D 的，卡点在输入侧与控制侧。
10. **sim2real**：能迁的是**架构/接口/选型/参数初值/地图/工具流程**；不能迁的是**鲁棒性**。三大杀手及解法：
    - **标定/时间同步** → LI-Init/lidar_align 标外参；PPS+触发或 `time_sync_en`；判据=静止点云不分层、闭合路径误差 cm 级；
    - **打滑/延迟** → 用 LIO 替轮速（已做）、标定"指令速度 vs 实际速度"、缩短链路、提控制频率、`velocity_smoother` 切 CLOSED_LOOP、控制器选 DWB/TEB；
    - **感知噪声→幽灵障碍** → 建图期 SOR/多帧一致性/射线清除/多走两遍；建图后形态学+小团块过滤+可达性检查；运行时静态层只放确认结构、临时物交给带时间维的层（静态图**不会自愈**）。
    - **最高价值用法**：在仿真里**注入缺陷**（`cmd_vel` 延迟/丢包、摩擦/打滑、点云离群点）→ 选型阶段就筛出鲁棒算法。

---

## 七、2026-09-24 复盘：`mode:=nav` 下「车不动 + 秒报 SUCCEEDED」的链路定位

### 7.1 症状
RViz 给目标 `(-1.00, 2.00)`（AMCL 重定位）→ BT 立刻 `Goal succeeded`，`distance_remaining` 停在 **2.3042919635772705**，`/cmd_vel` 全程**零样本**，车体一步不动。

### 7.2 决定性证据（全部在活图上实测）

| 观测 | 命令 | 实测 | 结论 |
|---|---|---|---|
| sim 时钟 | `ros2 topic echo /clock --field clock.sec --once` | 1069 | 时钟在跑 |
| **costmap 自己发的栅格** | `ros2 topic echo /local_costmap/costmap_raw --qos-durability transient_local --field header.stamp --once` | **1067.58**（global 1068.88） | costmap 节点时钟、发布线程、更新循环**全部正常** |
| **costmap 的 footprint** | `ros2 topic echo /local_costmap/published_footprint --field header.stamp --once` | **644.682，相隔 181 秒两次采样一字不变** | 它的 `getRobotPose()` 永远返回**启动那一瞬**的僵住位姿 |
| TF 线上是否新鲜 | `ros2 run tf2_ros tf2_echo <A> <B>`（base_link→base_link_fake、odom→base_link、map→odom、map→base_link_fake） | 949~958 **全部新鲜**，`/tf` 42.9 Hz | **TF 本身没有断** |
| 控制器自己的日志 | `~/.ros/log/controller_server_*.log` | `Exception in transformPose: Lookup would require extrapolation into the past. Requested time 644.682000 but the earliest data is at time 644.882000, when looking up transform from frame [odom] to frame [map]` → `Unable to transform robot pose into global plan's frame` → **`Reached the goal!`**（相隔 20 µs） | 见 7.3 |

### 7.3 因果链（已闭环，可稳定复现）
1. 两个 costmap 子节点（`local_costmap` 在 `controller_server` 进程内、`global_costmap` 在 `planner_server` 进程内）的 **tf2 缓冲区在启动约 0.2 秒后就不再进新数据**，永远停在 `644.682` / `646.282`。
2. ⇒ `Costmap2DROS::getRobotPose()`（内部 `nav2_util::getCurrentPose()`）用「最新可用」查询，于是**一直成功**返回**同一个 644.682 的旧位姿**（`published_footprint` 以 20 Hz 重复发这同一戳，所以后来才加入的订阅者也能收到，看起来"话题还活着"）。
3. ⇒ `ControllerServer::isGoalReached()` 里 `nav_2d_utils::transformPose(costmap_ros_->getTfBuffer(), ...)` 要把目标姿态从 map 转到 odom：它拿的正是这个**旧缓存**（最早数据 644.882）去查 **644.682** ⇒ `ExtrapolationException`。
4. ⇒ **上游 `controller_server.cpp` 打印了 ERROR，却把 `transformPose()` 的 bool 返回值丢掉**，`transformed_end_pose` 保持默认 `(0,0,0)`。
5. ⇒ `SimpleGoalChecker`（xy 容差 0.25 m / yaw 0.25 rad）比较车实际位姿 `(0.013, -0.007)` 与 `(0,0,0)`：相差 **1.4 cm < 容差** ⇒ **“Reached the goal!”** ⇒ 零速 ⇒ BT 报 `Goal succeeded` ⇒ **车一步没动**。
   （两次目标相隔 203 秒、结果完全一致；这条链解释了每一个观测到的现象。）

### 7.4 已排除的假设（都有活图证据，不必重复试）

| 假设 | 证据 | 结论 |
|---|---|---|
| `/tf` QoS 不匹配 | `ros2 topic info -v /tf`：6 个发布者 + 9 个订阅者**全部** RELIABLE / KEEP_LAST(100) / VOLATILE | 排除 |
| RMW 是 Fast DDS（需换 CycloneDDS） | 本栈**已经在跑** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（`ros2-humble-rmw-cyclonedds-cpp 1.3.5` 已装） | 排除（这一招已在用） |
| 有僵尸/重复进程在抢 | `/cmd_vel` 的 5 个发布者 = `velocity_smoother` + `behavior_server` 的 4 个行为插件（Humble 正常）；`/tf` 6 个发布者全是本尊；`/clock` **只有 1 个发布者** | 排除 |
| 某些节点 `use_sim_time` 没设 | 逐节点 `ros2 param get`：controller / planner / bt_navigator / amcl / map_server / velocity_smoother / behavior_server / **两个 costmap** 全 True | 排除 |
| 组合容器共享 executor 导致互锁 | `use_composition:=False`（进程已分离，节点列表里无 component_container）后**同样复现** | 排除 |
| `transform_tolerance` 太小 | 曾放大到 1000.0：在 tf2 里这个值经 `getCurrentPose()` 传成了**等待超时**，遇到「过去的时间点」这种永远等不来的查询，会让 costmap 线程一次阻塞到超时（1000 秒）—— **已撤销回 0.3** | 不是解，且危险 |

### 7.5 仍未知 + 下一步（按优先级）
1. **未解的问题**：为什么这两个 costmap 的 `tf2_ros::Buffer` 只在启动一瞬进数据、之后永久不进。节点时钟（`costmap_raw` 新鲜）、executor（动作与发布都在转）、线上 TF（新鲜）、QoS、RMW 都已排除 ⇒ 卡点在这条订阅的**摄入回调**上，而不是上游数据。
   - 待查线索 A：buffer 时钟类型的差异 —— 实测 **`tf2_echo`（墙钟缓存）能正常读到 sim 戳的 TF**，而 costmap（`use_sim_time=True` 的 sim 钟缓存）读不到；需确认 tf2 的 `TF_OLD_DATA` 剪枝是否按 buffer 时钟判定，以及 `/clock` 只有 **7.76 Hz**（不是常见的 100 Hz）是否让剪枝窗口抖动。
2. **该修的两处（互相独立）**：
   - ① **上游 `nav2_controller/src/controller_server.cpp::isGoalReached()` 必须尊重 `transformPose()` 的返回值**（失败就 `return false` 并明确报错），让"假到达"至少变成"诚实失败"。做法与 cartographer 一样：把 nav2 拷进 `src/` 做 overlay 构建，`third_party/nav2` 保持只读。
   - ② **让 costmap 的 tf 摄入不冻**：候选是把 `costmap_2d_ros.cpp:187` 的 `TransformListener(*tf_buffer_)` 改成带节点/回调组（`tf2_ros::TransformListener(*tf_buffer_, node, true)`），或给它独立 spin 线程。
3. **5 分钟判据实验**：只重启 nav2（sim/LIO 不动），每 2 秒采一次 `ros2 topic echo /local_costmap/published_footprint --field header.stamp`，看戳是否**先跟 `/clock` 走几秒、然后固定不动**；若是，即确认「启动窗口内 TF 尚未就绪 ⇒ 摄入线程死掉」这一形态，再做 ②。

### 7.6 上游查证结论（2026-09-24 联网调研）

**① 「丢弃 `transformPose` 返回值」是上游已知缺陷：main 已修，Humble 永不回移**
- 引入：[PR #2780](https://github.com/ros-navigation/navigation2/pull/2780)（2022-01-21）把目标 TF 查询挪进 `isGoalReached()`，同时**丢掉了返回值**。
- 修复：[PR #6436](https://github.com/ros-navigation/navigation2/pull/6436)「Reusing a staleness-proofed tf lookup…」（2026-09-20 合入 main，关 [#6320](https://github.com/ros-navigation/navigation2/issues/6320)/[#6316](https://github.com/ros-navigation/navigation2/issues/6316)）：把 `nav_2d_utils::transformPose` 换成 `nav2_util::transformPoseInTargetFrame`（**false 就抛 `nav2_core::ControllerTFError`**），并新增 `transform_staleness_threshold` 新鲜度阈值。
- **#6436 没有 `backport-*` 标签，Humble 无回移**（Humble 分支最近的 controller_server 改动是 #6191）⇒ 想让"假到达"变成"诚实失败"，只能自己把这套纪律补回本地（overlay）。

**② 日志里那两行的分工（纠正 7.2 的表述 —— 这一点很关键）**
- `[tf_help] Transform data too old when converting from map to odom` + `Data time: … / Transform time: …` = **`isGoalReached()` 那条路**。`nav_2d_utils::transformPose` 会**吞掉** ExtrapolationException，退而取"最新缓存变换"并按 `transform_tolerance` 判年龄 ⇒ 超龄就 `return false`；上游又不看这个返回值 ⇒ 目标姿态保持默认 `(0,0,0)` ⇒ 假到达。**这才是主证据。**
- `[controller_server] Exception in transformPose: … from frame [odom] to frame [map]` = **同一控制周期里「全局路径转局部系」那条路**（只有 `transformPoseInTargetFrame` 会打印 `ex.what()`），是同一个旧缓存造成的**第二个症状**，不是假到达的直接原因。
- 顺带核实：本地 costmap 是 `global_frame: odom`（**不是** map）✓，所以常规查询不走 `map↔odom` 那条慢链。

**③ `transform_tolerance` 在这个 bug 上根本不是杠杆（撤销是对的）**
- tf2 里 `lookupTransform(…, timeout)` 的 `timeout` **只是"等数据到达"的等待时长，绝不是外插容差**，对"过去的时间点"永远无效（后到的数据戳只会更晚）。
- Humble 的 `nav_2d_utils::transformPose` 甚至**不用**它当超时，只在 Extrapolation 回退分支里拿它判"旧变换能否凑合" ⇒ 调大要么无效，要么变成「拿 400 秒前的旧变换凑合并用它判距离」，同样是错的。与上游 [#5234](https://github.com/ros-navigation/navigation2/issues/5234)（"transform_tolerance 对 controller_server 无影响"，closed **未修**）完全吻合。

**④ 「costmap 的 tf 摄入停掉」没有对应的上游报告**（我们这一例是残余病例）
- 最接近的是 [#3352](https://github.com/ros-navigation/navigation2/issues/3352)：Gazebo 下 footprint 冻结、导航失效；维护者判定为 **RMW 缺陷**（让去报 Fast-DDS，关联 [eProsima/Fast-DDS #3195](https://github.com/eProsima/Fast-DDS/pull/3195)），报告者换 CycloneDDS 后消失。**我们已经在 CycloneDDS 上，所以不是它。**
- [geometry2 #727](https://github.com/ros2/geometry2/issues/727) 给出唯一"节点看着健康但 /tf 回调饿死"的可复现原因：`~/.bashrc` 里残留的 `FASTRTPS_DEFAULT_PROFILES_FILE` 自定义 DDS profile。**本机已查：只有 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 与 `ROS_LOCALHOST_ONLY=0`，无任何 DDS profile 变量 ⇒ 排除。**（附带发现：`~/.bashrc` 192~198 行把同一行 RMW 导出**重复写了 7 次**，无害但建议清理。）
- tf2 会在**时间跳变**时清空整个缓冲区（`Detected jump back in time / Detected time source change. Clearing TF buffer.`，连静态帧一起清）⇒ **已 grep 全部日志：无任何清空事件 ⇒ 排除。**
- tf2 缓冲区 cache 固定 10 s、nav2 不暴露该参数；nav2 用的正是"隐藏节点 + 独立线程 + `setUsingDedicatedThread(true)`"，**上游 main 也没改这个策略** ⇒ 不要照搬「给 `TransformListener` 传 node + `spin_thread=true`」这类改法（会破坏该契约，除非同时把回调组挂进自己的 executor 并手动置 `setUsingDedicatedThread(true)`）。

**⑤ 所以本地该做的是「补回 #6436 的纪律 + 单独解决摄入冻结」，不是继续找配置开关。**

---

## 六、待办（未完成）

| 优先级 | 事项 | 说明 |
|---|---|---|
| ★★★ | **定位 `/scan` 丢帧环节** | 实测 `/scan` 只有 **0.55~3.03 Hz**（`/odom` 10 Hz、IMU 100 Hz），这是"转起来跟不住"的前置条件（10 Hz/100ms 帧内转 60°/s 就是 6°，1 Hz 就是几十度，任何 2D SLAM 都无解）。3 万采样保持不动的前提下逐段测 `/livox/lidar/pointcloud` → `/segmentation/obstacle` → `/scan` 的 hz，判断丢在 `linefit`（`n_threads 4`/`n_segments 360`）还是 `p2l`；再决定是加线程/QoS 深队列还是调 `angle_increment` |
| ★★★ | **落盘本次建图（patched 核心已生效）** | 顺序：① `tools/scripts/mapping/save_grid_map.sh`（= `map_saver_cli -f src/rm_nav_bringup/map/RMUL2026`，存 `.pgm/.yaml`）；② `/finish_trajectory` + `/write_state` 存 `.pbstream`（runbook §5，绝对路径）；③ `tools/scripts/mapping/save_pcd.sh` 存 `PCD/RMUL2026.pcd`（需 LIO 在跑）。**⚠️ 坐标系**：cartographer 的 `map` 系是**出生点相对系**，与现有 `map/RMUL2026.pgm`（世界系，`origin:[2.68,0.228]`）差 ≈`(4.3,3.35)`⇒ 覆盖前先另存一份比对，并把 `amcl_init_x/y` 改成 `0.0` |
| ★★★ | **重建 RMUL2026 地图** | `mode:=mapping` + 遥控 + 落盘三件套（`.pgm/.yaml`、`.posegraph`、`PCD/RMUL2026.pcd`）；落盘前备份旧图。建完跑 `check_map_reachable.py --start 0 0`，并把 RMUL2026 的 `amcl_init_x/y` 改为 `0.0`（新图为出生点系）、同步 §0.5 表格 |
| ★★★ | **场景三 nav+ICP @ RMUL 重跑** | 等 30~60 s；按 `/livox/lidar→/livox/imu→/imu/data→/odom` 逐段定位；必要时 `lio:=pointlio` 做 A/B |
| ★★ | 场景 4/5/6/7 未测 | slam_toolbox 纯定位、cartographer 建图/纯定位、`nav:=dwb|teb` |
| ★★ | 在新图上验证"发目标能走" | 之前被幽灵墙挡住，未真正验证循迹与小陀螺 |
| ★★ | **`global_obstacle` A/B** | 同一场地/目标点分别跑 `stvl` 与 `scan`，比较：全局路径是否绕开临时障碍、CPU 占用、0.1 m 矮台在 local/global 的判断是否一致（结果记入 `algorithm_matrix.md` §四） |
| ★★ | **`local_obstacle` A/B + 降级验证** | 同一场景跑 `scan` / `cloud` / `both`：① 贴墙 0.3 m 时 local 是否看到（`scan` 应看不到）；② `pkill -f pointcloud_to_laserscan` 后是否 WARN + 1 s 内停车（runbook §7.3）；③ CPU 增量 |
| ★ | 工具增强 | `pcd_to_grid_map.py` 加 SOR + 小团块过滤 + 形态学细化 + 用 `/path` 做射线清除（把"洪泛 free"升级为"射线 free"） |
| ★ | 退化测试槽位 | 写 `cmd_vel` 延迟/丢包注入节点；摩擦/打滑与点云离群点注入 |
| ★ | 未来 2.5D / 3D 槽位 | 高程/坡度/净空图生产（`src/rm_perception/rm_elevation_map/`）+ costmap 图层插件（`src/rm_navigation/rm_costmap_layers/`）；云台瞄准（2–3 DOF）规划 |
