# cod_nav_2026 逐块参数评审：singlenav2 / multiplenav2 / goal_approach_controller / 航点 CSV

> 范围：只读 `third_party/cod_nav_2026/**` 与 `src/**`，**未改动任何代码 / 配置 / 其它文档**，只新增本文件。
> 每条结论都带 `file:line`。第三方仓库只当参考数据，未执行其中任何代码。

---

## 0. 基准、缩写与判定口径

### 0.1 版本基准（本次评审最关键的前提）

| 事实 | 证据 |
|---|---|
| 本机 nav2 = **1.1.20 (Humble/jammy)** | `dpkg -l` → `ros-humble-nav2-mppi-controller 1.1.20-1jammy.20260908.012051` 等 32 个包全是 1.1.20 |
| 仓库内 nav2 源码 = 同一版本 | `third_party/nav2/nav2_mppi_controller/package.xml:5` `<version>1.1.20</version>`；`third_party/nav2/nav2_controller` 同；git `3c3db59d`，分支 `humble` |
| `install/` 里**没有** nav2 overlay 补丁 | `ls install/` 只有 cartographer / fast_lio / rm_* / teb_local_planner 等，无 `nav2_controller`、`nav2_mppi_controller` |

**推论（贯穿全文）**：cod_nav_2026 的 yaml 里有一大批键是 **nav2 Jazzy/main 才存在**的，在 1.1.20 上**既不报错也不生效（静默失效）**。所以「照抄他们的 yaml」在两边行为**不一样**——这是本次评审最重要的发现，汇总表见 §A.16。

### 0.2 缩写表（表格里全部用它）

| 缩写 | 全路径 |
|---|---|
| **S** | `third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml` |
| **M** | `third_party/cod_nav_2026/src/cod_bringup/params/multiplenav2_params.yaml` |
| **SL** / **ML** | `.../cod_bringup/launch/singlenav_launch.py` / `multiplenav_launch.py` |
| **NL** / **LOC** | `.../cod_bringup/launch/navigation_launch.py` / `localization_launch.py` |
| **BT1** / **BTTP** | `.../cod_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` / `navigate_through_poses_...xml` |
| **GAC** | `third_party/cod_nav_2026/src/goal_approach_controller/src/goal_approach_controller.cpp` |
| **GACX** | `.../goal_approach_controller/goal_approach_controller_plugin.xml` |
| **GACC** | `.../goal_approach_controller/CMakeLists.txt` |
| **MAP** | `.../cod_bringup/maps/rmul2026.yaml` |
| **SLAMc** | `.../cod_bringup/params/mapper_params_online_async.yaml`（他们的 slam_toolbox） |
| **FVTc** | `third_party/cod_nav_2026/src/fake_vel_transform/src/fake_vel_transform.cpp` |
| **WE** | `third_party/cod_nav_2026/src/waypoint_editor/` |
| **R** / **D** / **T** | `src/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml` |
| **NAV** | `src/rm_navigation/rm_navigation/launch/navigation_launch.py` |
| **BP** | `src/rm_navigation/rm_navigation/launch/bringup_rm_navigation.py` |
| **LAUNCH** | `src/rm_nav_bringup/launch/bringup_sim.launch.py` |
| **FVT** / **FVTC** | `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp` / `config/fake_vel_params.yaml` |
| **P2L** | `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml` |
| **SLAMrm** | `src/rm_navigation/rm_navigation/params/mapper_params_online_async.yaml` |
| **N2/…** | `third_party/nav2/…`（≡ 本机 /opt/ros/humble 的 1.1.20） |

N2 细则（全部相对 `third_party/nav2/`）：
- `N2/ctrl` = `nav2_controller/src/controller_server.cpp`
- `N2/ppgc` = `nav2_controller/plugins/position_goal_checker.cpp`
- `N2/mppi/opt` / `N2/mppi/ctrl` / `N2/mppi/ph` / `N2/mppi/ng` / `N2/mppi/tv` / `N2/mppi/cm` = `nav2_mppi_controller/src/{optimizer,controller,path_handler,noise_generator,trajectory_visualizer,critic_manager}.cpp`
- `N2/mppi/cf` = `nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp`
- `N2/mppi/cc` = `nav2_mppi_controller/src/critics/cost_critic.cpp`；`N2/mppi/oc` = `…/critics/obstacles_critic.cpp`；`N2/mppi/gc` = `…/critics/goal_critic.cpp`；`N2/mppi/pfc` = `…/critics/path_follow_critic.cpp`；`N2/mppi/pac` = `…/critics/path_align_critic.cpp`；`N2/mppi/con` = `…/critics/constraint_critic.cpp`
- `N2/cmros` = `nav2_costmap_2d/src/costmap_2d_ros.cpp`；`N2/static` = `nav2_costmap_2d/plugins/static_layer.cpp`；`N2/infl` = `nav2_costmap_2d/plugins/inflation_layer.cpp`；`N2/cmpub` = `nav2_costmap_2d/src/costmap_2d_publisher.cpp`
- `N2/smac2` = `nav2_smac_planner/src/smac_planner_2d.cpp`；`N2/smac_t` = `nav2_smac_planner/include/nav2_smac_planner/types.hpp`；`N2/navfn` = `nav2_navfn_planner/src/navfn_planner.cpp`
- `N2/beh` = `nav2_behaviors/src/behavior_server.cpp`；`N2/spin` = `nav2_behaviors/plugins/spin.cpp`；`N2/timed` = `nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp`
- `N2/bt` = `nav2_bt_navigator/src/bt_navigator.cpp`；`N2/bta` = `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`
- `N2/wpf` = `nav2_waypoint_follower/src/waypoint_follower.cpp`；`N2/waw` = `nav2_waypoint_follower/plugins/wait_at_waypoint.cpp`
- `N2/vs` = `nav2_velocity_smoother/src/velocity_smoother.cpp`

### 0.3 判定口径

- **✅ 可直接用**：键名在 1.1.20 存在，且值对我们（mecanum/omni、`robot_radius 0.40`、Gazebo Classic sim、costmap 5–20 Hz、`base_link_fake`）语义成立，抄过去不改也能跑。
- **⚠️ 需改造**：键名有效，但值必须按我们的几何/频率/底盘改；或语义依赖他们的架构（LIO + 静态 `map→odom`、云台小陀螺、0.1 m 车身）。
- **❌ 不适用**：1.1.20 无此键（静默失效），或物理前提不成立（0.1 m 车、D435i、DUBIN/Hybrid 车体、50 Hz 控制环）。

### 0.4 一页结论（细节见 A–E）

| 类别 | 条目 |
|---|---|
| **可直接用** | stvl 体素层参数块；Smac2D 的 9 个有效键 + `smoother.*` 子块；velocity_smoother 键名；map_saver；MPPI 骨架键名与 6 个 critic 的键名；waypoint CSV 格式 + `waypoint_to_nav2`/`waypoint_through_nav2` 两个桥节点 |
| **需改造** | `goal_approach_controller`（3 处硬伤，§C.6）；MPPI 全部数值（50 Hz / 1.6 s 时域 / 2.5 m/s）；BT 树（3 Hz、删 Spin/Wait、BackUp 1.0 m）；costmap 几何（0.1 m vs 0.40 m）；goal checker（`path_length_tolerance` 是死键）；velocity_smoother 数值 |
| **不适用** | `vy_min/wz_min/ax_*/ay_*/az_*`（MPPI 1.1.20 无）；`publish_critics_stats`；planner 的 Hybrid/Lattice 整段；`local_costmap_topic` 等 4 个 topic 键 + `local_frame`；`pb_nav2_behaviors/BackUpFreeSpace`（需额外包）；深度相机观测源；静态 `map→odom` 定位法；`auto_save_map.launch.py`（硬编码 `/home/cod-sentry`） |

---

## A. `S`（singlenav2_params.yaml）逐块对照

### A.1 顶层元信息

| 键 | 他们（S 行） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| 文件总行数 | 483 行（整文件） | R 408 / D 419 / T 418 | — | — |
| `use_sim_time` | `false`（S:3/66/185/276/357/364/372/424/458/462） | `True`（R:3/104/163/245/347/356/391/397） | ⚠️ | 我们全栈 sim 钟；他们实车。**且他们的 `use_sim_time` 是被 `NL:61-71` 的 RewrittenYaml 从 launch 参数覆写的，yaml 里的值基本是惰性的** |
| 帧约定 | `global_frame: map`（S:4/273）、`robot_base_frame: base_link_fake`（S:5/184/274）、`odom_frame: odom`（S:183）、静态 `map→odom`（SL:67-89，yaw **−0.5**；ML:95-116，yaw **0.0**） | 同帧名（R:64/65/161/162/243/244），但 `map→odom` 由 amcl/slam_toolbox/icp/cartographer 提供（LAUNCH:303-406），仅 `mode=nav,localization=''` 才用静态桥（LAUNCH:446-468） | ⚠️ | 帧**名字**一致（`base_link_fake` 是他们的也是我们的），但**谁发 `map→odom`** 完全不同：他们是「LIO 当绝对定位 + 静态桥」，我们是「AMCL/SLAM/ICP/Cartographer」。这直接决定 §C.4 的帧 bug 严重程度 |
| odom 话题名 | `Odometry`（S:6、S:482） | `odom`（R:407）／`/Odometry`（R:66，**疑似过期**） | ⚠️ | 他们 LIO 原生话题就是 `Odometry`；我们 T2 已统一到 `/odom`（LAUNCH:256/282），但 `bt_navigator.odom_topic` 还是 `/Odometry`（R:66）→ BT 的 `OdomSmoother` 收不到数据（N2/bt_navigator.cpp:98/135 只声明/订阅这一路）。低危但不干净 |
| 机器人几何 | `robot_radius: 0.1` + `footprint ±0.1`（S:190-191/277-278） | `robot_radius: 0.40` 且**无 footprint**（R:178/252） | ❌ | 见 §A.5/A.6；且**无 footprint 这一点会直接触发 MPPI critic 抛异常**（§A.4、§E） |
| 速度量级 | 2.5 m/s、wz 1.5（S:99-105） | RPP `desired_linear_vel: 0.5`（R:130）、DWB `max_vel_x 2.5`（D:133）、TEB `max_vel_x 0.5`（T:144） | ❌ | 他们哨兵实车高速；我们 sim + 场地（RMUL2026 图 12.75 m × 8.85 m，见 §A.13） |

### A.2 controller_server 框架参数

| 键 | 他们（S 行） | 我们（R 行） | 判定 | 理由 |
|---|---|---|---|---|
| `publish_critics_stats` | `True`（S:65） | 无 | ❌ | **1.1.20 全 nav2 无此键**（`grep -rn publish_critics_stats third_party/nav2/{nav2_controller,nav2_mppi_controller}` 无命中）→ 静默失效 |
| `controller_frequency` | `50.0`（S:67） | `20.0`（R:105） | ⚠️ | 见 §A.3 `model_dt`：**50 Hz 是他们的隐含前提**；我们 20 Hz 直接抄 `model_dt: 0.02` 会让 MPPI 在 configure 阶段抛异常 |
| `min_x_velocity_threshold` | `0.001`（S:68） | `0.001`（R:106） | ✅ | 一致 |
| `min_y_velocity_threshold` | `0.001`（S:69） | `0.5`（R:107） | ⚠️ | 我们 0.5 → 侧向**里程计反馈**小于 0.5 m/s 被抹成 0（`N2/ctrl` 的 `getThresholdedTwist`）。RPP 不产生 vy 所以以前无害，**接 MPPI(Omni) 前必须降到 0.001** |
| `min_theta_velocity_threshold` | `0.001`（S:70） | `0.001`（R:108） | ✅ | 一致 |
| `failure_tolerance` | `0.3`（S:71） | `0.3`（R:109） | ✅ | 一致（秒） |
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker`（S:76） | 同（R:116） | ✅ | — |
| `progress_checker.required_movement_radius` | `0.1`（S:77） | `0.5`（R:117） | ⚠️ | 0.1 m 配合 `movement_time_allowance 999 s`（S:78）≈ **关掉卡死检测**；我们 0.5 m/10 s 更严。抄他们等于放弃 progress 保护，建议保留我们的 |
| `progress_checker.movement_time_allowance` | `999.0`（S:78） | `10.0`（R:118） | ⚠️ | 同上（单位秒；999 s 实为禁用） |
| `goal_checker_plugins` | `["general_goal_checker"]`（S:73） | `["general_goal_checker"]`（R:111） | ✅ | 名字一致 |
| `general_goal_checker.plugin` | `nav2_controller::PositionGoalChecker`（S:81） | `nav2_controller::SimpleGoalChecker`（R:123） | ⚠️ | **`PositionGoalChecker` 在 1.1.20 存在**（`third_party/nav2/nav2_controller/plugins.xml` 的 `position_goal_checker` 段；`N2/ppgc:28-52`）→ 可换。它只看 XY、忽略 yaw（`N2/ppgc:76-87`），与「omni 车不必对朝向 + direct-approach 强制 `angular.z=0`」自洽 |
| `general_goal_checker.stateful` | `False`（S:80） | `True`（R:122） | ⚠️ | `PositionGoalChecker` 的 `stateful` 是**真键**（`N2/ppgc:47-52`）。false ⇒ 每周期重算「是否在 0.2 m 内」；true ⇒ 一旦进入就锁存（`N2/ppgc:70-73`）。配合他们的 BT（到点即结束）false 更安全 |
| `general_goal_checker.xy_goal_tolerance` | `0.2`（S:82） | `0.25`（R:124） | ⚠️ | 0.2 比我们 0.25 严；注意他们 `direct_approach_distance 2.0` 使末端由直接驱动接管，所以 0.2 不会引起绕圈 |
| `general_goal_checker.path_length_tolerance` | `0.5`（S:83） | — | ❌ | **1.1.20 的 PositionGoalChecker 不声明此键**：`N2/ppgc:44-52` 只声明 `xy_goal_tolerance` + `stateful`；对已安装 `libposition_goal_checker.so` 取字符串也只有 `.xy_goal_tolerance`。该键只在 nav2 更新版（2025 年 Prabhav Saxena 版本）里存在 → 我们这边**静默失效**，注释「path_length_tolerance」不可作为采用依据 |
| `general_goal_checker.yaw_goal_tolerance` | 无（PositionGoalChecker 无此键） | `0.25`（R:125，SimpleGoalChecker 有） | — | 换 PositionGoalChecker 时必须删掉 `yaw_goal_tolerance`，否则是死键 |
| `FollowPath.plugin` | `goal_approach_controller::GoalApproachController`（S:85） | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`（R:129）／`dwb_core::DWBLocalPlanner`（D:129）／`teb_local_planner::TebLocalPlannerROS`（T:130） | ⚠️ | 见 §C 全节 |

### A.3 FollowPath：wrapper + MPPI 骨架参数

他们 FollowPath 段 = S:84-176。`S:85-86` 说明是 `goal_approach_controller` 包 MPPI；**wrapper 与 MPPI 共用同一个参数命名空间 `FollowPath.*`**（GAC:63 `inner_controller_->configure(parent, name, tf, costmap_ros)` 传的是同一个 `name`）。

| 键 | 他们（S 行 = 值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `inner_plugin` | `nav2_mppi_controller::MPPIController`（S:86） | 无（MPPI 未接线，但 `ros-humble-nav2-mppi-controller 1.1.20` 已装、`third_party/nav2/nav2_mppi_controller` 在库） | ⚠️ | wrapper 参数，GAC:36-38 声明默认即 MPPI |
| `approach_distance` | `2.5`（S:87） | 无 | ⚠️ | wrapper 专有（GAC:39-41，默认 1.5）；见 §C.3 |
| `approach_velocity` | `0.2`（S:88） | 无 | ⚠️ | wrapper 专有（GAC:42-44，默认 0.5）；见 §C.3 |
| `direct_approach_distance` | `2.0`（S:89） | 无 | ⚠️ | wrapper 专有（GAC:45-47，默认 0.5）；见 §C.4 |
| `direct_approach_kp` | `3.0`（S:90） | 无 | ⚠️ | wrapper 专有（GAC:48-50，默认 1.0） |
| `time_steps` | `80`（S:91） | 无 | ⚠️ | **真键**（`N2/mppi/opt:70`，默认 56）。80×0.02 = **1.6 s** 时域（他们注释写 4.8 s，见 §F-1） |
| `model_dt` | `0.02`（S:92） | 无 | ⚠️⚠️ | **真键**（`N2/mppi/opt:69`，默认 0.05）。**硬约束**：`N2/mppi/opt:95-113` 要求 `1/controller_frequency == model_dt`；我们 `controller_frequency 20.0`（R:105）→ 周期 0.05 > 0.02 → **`throw std::runtime_error("Controller period more then model dt")`**（`N2/mppi/opt:111-112`）→ controller_server configure 失败、整栈起不来。两条出路：(a) `controller_frequency: 50.0`（照抄他们的 50 Hz）；(b) `model_dt: 0.05`（= 我们 20 Hz，且会打开 `shift_control_sequence`，`N2/mppi/opt:104-109`）。周期 < model_dt 只 WARN（`N2/mppi/opt:100-103`），所以 M 的 0.06（周期 0.02）合法但会刷警告 |
| `batch_size` | `1500`（S:93） | 无 | ⚠️ | 真键（`N2/mppi/opt:71`，默认 1000）。1500×80 的采样规模在 sim 里 CPU 开销大，建议先 1000 |
| `iteration_count` | `1`（S:115） | 无 | ✅ | 真键（`N2/mppi/opt:72`，默认 1）；值=默认 |
| `temperature` | `0.25`（S:118） | 无 | ⚠️ | 真键（`N2/mppi/opt:73`，默认 0.3）。0.25 更「果断」，但对他们 2.5 m/s 调的，我们低速需重调 |
| `gamma` | `0.015`（S:119） | 无 | ✅ | 真键（`N2/mppi/opt:74`）；= 默认 |
| `motion_model` | `"Omni"`（S:120） | 无 | ✅ | 真键（`N2/mppi/opt:84`；合法值 DiffDrive/Omni/Ackermann，`N2/mppi/opt:406-420`）。我们 mecanum ⇒ **Omni 正确** |
| `vx_max` | `2.5`（S:99） | DWB `max_vel_x: 2.5`（D:133）；RPP `desired_linear_vel: 0.5`（R:130） | ⚠️ | 真键（`N2/mppi/opt:75`）。我们 sim 建议 0.5–1.0 |
| `vx_min` | `-2.5`（S:103） | DWB `min_vel_x: -2.5`（D:131） | ⚠️ | 真键（`N2/mppi/opt:76`）。允许全速倒车，sim 里危险 |
| `vy_max` | `2.5`（S:100） | DWB `max_vel_y: 2.5`（D:134） | ⚠️ | 真键（`N2/mppi/opt:77`，注意 **1.1.20 只有 `vy_max` 一个键，vy 对称**） |
| `wz_max` | `1.5`（S:101） | RPP `rotate_to_heading_angular_vel: 1.8`（R:135）；velocity_smoother `max_velocity[2]=3.0`（R:401） | ⚠️ | 真键（`N2/mppi/opt:78`）。注意他们 `velocity_smoother.max_velocity` 写的是 2.5（S:476），与 `wz_max 1.5` **不一致** |
| `vx_std` / `vy_std` / `wz_std` | `0.3 / 0.3 / 0.2`（S:95-97） | 无 | ⚠️ | 真键（`N2/mppi/opt:79-81`，默认 0.2/0.2/0.4）。wz_std 比默认**小** |
| **`vy_min`** | `-2.5`（S:104） | — | ❌ | **1.1.20 无此键**（`N2/mppi/opt:75-84` 全部 `getParam` 清单里没有；全包 grep `"vy_min"` 无命中） |
| **`wz_min`** | `-1.5`（S:105） | — | ❌ | 同上，无此键 |
| **`ax_max` / `ay_max` / `az_max`** | `1.5 / 1.5 / 1.0`（S:107-109） | 无 | ❌ | 1.1.20 MPPI **没有加速度约束参数**（`N2/mppi/opt:69-84` 无 ax/ay/az）⇒ 这三行连同注释「制动力必须对称，高速下刹得住」**完全不生效**。真正限加速度的是 `velocity_smoother.max_accel/max_decel`（S:480-481 / R:405-406） |
| **`ax_min` / `ay_min` / `az_min`** | `-3.5 / -3.5 / -3.0`（S:111-113） | 无 | ❌ | 同上，死键 |
| `prune_distance` | `1.7`（S:116） | 无 | ⚠️ | 真键（`N2/mppi/ph:38`，默认 1.5） |
| `transform_tolerance` | `0.1`（S:117） | 无（我们控制器侧不设；costmap 是 0.3，R:158） | ⚠️ | 真键（`N2/mppi/ph:39`） |
| `visualize` | `false`（S:121） | 无 | ✅ | 真键（`N2/mppi/ctrl:41`，默认 false） |
| `reset_period` | `0.08`（S:122，注释「only in Humble」） | 无 | ✅ | 真键（`N2/mppi/ctrl:42`，默认 1.0）；注释正确，1.1.20 确实有 |
| `retry_attempt_limit` | 仅 M（M:117 `2`）；S 无 | 无 | ⚠️ | 真键（`N2/mppi/opt:82`，默认 1）。注意 `N2/mppi/opt:175-177` 超限会 `throw "Optimizer fail to compute path"` |
| `regenerate_noises` | 仅 M（M:118 `true`）；S 无 | 无 | ⚠️ | 真键（`N2/mppi/ng:35`，默认 false） |
| `TrajectoryVisualizer.trajectory_step` | `5`（S:124） | 无 | ✅ | 真键，命名空间 `FollowPath.TrajectoryVisualizer.*`（`N2/mppi/tv:34-36`），默认 5 |
| `TrajectoryVisualizer.time_step` | `3`（S:125） | 无 | ✅ | 同上，默认 3 |

### A.4 critics 列表与逐个参数

`critics`（S:126-129）= `["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]`，是 1.1.20 的真键（`N2/mppi/cm:39`）。**注意：critic 的启用只由这份列表决定（`N2/mppi/cm:50-59`），每段里的 `enabled:` 也是真键（`N2/mppi/cf:81`，默认 true）**，两者都写才是他们那样。

| 键 | 他们（S 行=值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `enabled`（各 critic） | 全 `true`（S:132/137/142/152/162/169） | — | ✅ | 真键（`N2/mppi/cf:81`），默认就是 true，写了也无害 |
| `ConstraintCritic.cost_power/cost_weight` | `1 / 4.0`（S:133-134） | — | ✅ | `N2/mppi/con:25-26`，默认正是 1 / 4.0 |
| `GoalCritic.cost_weight` | `25.0`（S:139） | — | ⚠️ | 默认 5.0（`N2/mppi/gc:28`）。25 是他们「防画弧」的手段，MPPI 权重需按我们速度重调 |
| `GoalCritic.threshold_to_consider` | `4.0`（S:140） | — | ⚠️ | 默认 1.4（`N2/mppi/gc:29`） |
| `ObstaclesCritic.repulsion_weight` | `1.5`（S:144） | — | ✅ | 默认 1.5（`N2/mppi/oc:27`） |
| `ObstaclesCritic.critical_weight` | `20.0`（S:145） | — | ✅ | 默认 20.0（`N2/mppi/oc:28`） |
| `ObstaclesCritic.consider_footprint` | `false`（S:146） | — | ✅ | 默认 false（`N2/mppi/oc:25`）；**我们必须保持 false**，见下方 CostCritic 行 |
| `ObstaclesCritic.collision_cost` | `10000.0`（S:147） | — | ✅ | 默认 10000（`N2/mppi/oc:29`） |
| `ObstaclesCritic.collision_margin_distance` | `0.2`（S:148） | — | ⚠️ | 默认 0.10（`N2/mppi/oc:30`）。0.2 对 0.1 m 车是 2 倍车宽；对我们 0.40 半径含义不同 |
| `ObstaclesCritic.near_goal_distance` | `0.5`（S:149） | — | ⚠️ | 默认 0.5（`N2/mppi/oc:31`）；值=默认 |
| （他们没写但真实存在的 ObstaclesCritic 键） | — | — | ⚠️ | `ObstaclesCritic.cost_scaling_factor`（默认 10.0）与 `ObstaclesCritic.inflation_radius`（默认 0.55）是真键（`N2/mppi/oc:91-93`），可用来覆盖 **critic 自己**看到的膨胀参数（与 costmap 的 `inflation_layer` 解耦）。这是他们没用、但对我们「两套 inflation 数值不一致」问题有用的旋钮 |
| `CostCritic.cost_weight` | `3.8`（S:154） | — | ⚠️ | 默认 3.81（`N2/mppi/cc:28`）≈默认（注释「适当降低」不成立，见 §F-5） |
| **`CostCritic.critical_cost`** | `253.0`（S:155，注释「只有 inscribed/lethal 才触发碰撞惩罚」） | — | ❌（语义被误解） | 这是**惩罚幅度**不是阈值：默认 300.0（`N2/mppi/cc:29`）。「只有 inscribed 以上才罚」是源码**硬编码**的 `if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE)`（`N2/mppi/cc:165`，1.1.20）。设成 253.0 只是把惩罚值调成与 253 同量级，与「关键修正」无关 → §F-4 |
| **`CostCritic.consider_footprint`** | `true`（S:156） | — | ❌ | **会直接抛异常**：`N2/mppi/cc:56-65` 判 `costmap_ros_->getUseRadius() == consider_footprint_`，相等且 `getUseRadius()` 为真时 `throw PlannerException`（`cc:62-64`）。我们**没有 footprint、只有 `robot_radius: 0.40`**（R:178/252）⇒ `use_radius_ = true`（`N2/cmros:408-416`）⇒ `true == true` ⇒ **configure 阶段抛异常、控制器起不来**。要用 CostCritic 必须：先加 footprint，或把 `consider_footprint` 设 false（`ObstaclesCritic` 有**完全相同**的检查，`N2/mppi/oc:46-55`，所以我们的 `ObstaclesCritic.consider_footprint` 也必须为 false） |
| `CostCritic.collision_cost` | `1000000.0`（S:157） | — | ✅ | 默认 1e6（`N2/mppi/cc:30`） |
| `CostCritic.near_goal_distance` | `0.5`（S:158） | — | ✅ | 默认 0.5（`N2/mppi/cc:31`） |
| **`CostCritic.trajectory_point_step`** | `2`（S:159） | — | ❌ | **1.1.20 的 `CostCritic` 不读此键**：它的 `getParam` 只有 `consider_footprint/cost_power/cost_weight/critical_cost/collision_cost/near_goal_distance/inflation_layer_name`（`N2/mppi/cc:26-32`）。全包读 `trajectory_point_step` 的只有 `N2/mppi/pac:34` 与 `path_align_legacy_critic.cpp:34` ⇒ 这一行**静默失效** |
| `PathFollowCritic.cost_weight` | `15.0`（S:164） | — | ⚠️ | 默认 5.0（`N2/mppi/pfc:32`） |
| `PathFollowCritic.offset_from_furthest` | `5`（S:165） | — | ⚠️ | 默认 6（`N2/mppi/pfc:30`） |
| `PathFollowCritic.threshold_to_consider` | `1.0`（S:166） | — | ⚠️ | 默认 1.0（`N2/mppi/pfc:27-29`）；值=默认 |
| `PathAlignCritic.cost_weight` | `15.0`（S:171） | — | ⚠️ | 默认 10.0（`N2/mppi/pac:30`） |
| `PathAlignCritic.max_path_occupancy_ratio` | `0.05`（S:172） | — | ⚠️ | 默认 0.07（`N2/mppi/pac:32`） |
| `PathAlignCritic.trajectory_point_step` | `4`（S:173） | — | ✅ | 默认 4（`N2/mppi/pac:34`） |
| `PathAlignCritic.threshold_to_consider` | `0.8`（S:174） | — | ⚠️ | 真键（`N2/mppi/pac:35-37`） |
| `PathAlignCritic.offset_from_furthest` | `20`（S:175） | — | ✅ | 默认 20（`N2/mppi/pac:33`） |
| `PathAlignCritic.use_path_orientations` | `false`（S:176） | — | ✅ | 默认 false（`N2/mppi/pac:38`）。omni 车正确 |
| 未列入的可用 critic | `GoalAngleCritic` / `PathAngleCritic` / `TwirlingCritic` / `PreferForwardCritic` / `VelocityDeadbandCritic` / `PathAlignLegacyCritic`（`third_party/nav2/nav2_mppi_controller/critics.xml:16-50`） | — | ⚠️ | 他们**一个都没用**：说明整套设计刻意不做朝向跟踪（omni + 无 yaw 目标 + direct-approach `angular.z=0`）。我们要接 MPPI 若要保朝向，得自己加 `GoalAngleCritic`/`PathAngleCritic` |

### A.5 local_costmap

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `update_frequency` | `20.0`（S:181） | `20.0`（R:159） | ✅ | 一致 |
| `publish_frequency` | `20.0`（S:182） | `10.0`（R:160） | ✅ | 20 Hz 发布两张 costmap 在 sim 里白烧 CPU，我们 10 Hz 更稳 |
| `global_frame` | `odom`（S:183） | `odom`（R:161） | ✅ | 一致；也是 §C.4 帧 bug 的根 |
| `robot_base_frame` | `base_link_fake`（S:184） | `base_link_fake`（R:162） | ✅ | 一致 |
| `rolling_window` | `true`（S:186） | `true`（R:164） | ✅ | 一致 |
| `width` / `height` | `10` / `10`（S:187-188） | `5` / `5`（R:165-166） | ⚠️ | 他们 10 m 窗口配 2.5 m/s；我们 5 m（R:165-166）在 0.5 m/s 够用，但若提速要先放大 |
| `resolution` | `0.05`（S:189） | `0.02`（R:167） | ⚠️ | 我们 2 cm 更细（为了 0.40 半径下的窄缝判断，见 R:168-178 注释）；他们的 5 cm 与其 stvl `voxel_size 0.05` 对齐。**若抄他们的 stvl 参数，注意 voxel 5 cm > 我们 costmap 2 cm** |
| **`robot_radius`** | `0.1`（S:190） | `0.40`（R:178） | ❌ | 0.1 m 是 0.2 m 见方车体的一半（配 footprint `±0.1`）；我们实车/仿真按 0.40 设计（R:173-177），**窄于 0.8 m 的缝判不可通行**是刻意的安全几何。抄 0.1 会让规划器贴墙钻盲区 |
| `footprint` | `"[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]"`（S:191） | 无（R 全文无 footprint） | ⚠️ | 他们的 footprint **非空** ⇒ `use_radius_ = false`（`N2/cmros:411-416`）⇒ 有效内切半径 = 0.1，`robot_radius` 反而**被忽略**。我们无 footprint ⇒ 圆形 0.40。这是两侧几乎所有「几何类参数」不可比的根源 |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（S:192） | `["obstacle_layer","obstacle_cloud_layer","inflation_layer"]`（R:182） | ❌（结构不同） | 他们局部**没有 `obstacle_layer`**，也没有 `/scan` 源；靠 stvl 吃 `/livox/lidar_filtered`+`/camera/.../points`（S:218-262）。我们在局部**刻意不用 static_layer**（局部不要静态先验压住实时障碍）且 `local_obstacle` 槽位在 scan/cloud/both 间切（NAV:76-90） |
| `static_layer.map_subscribe_transient_local` | `True`（S:195） | 局部无 static_layer | ⚠️ | 他们在 rolling local costmap 里塞 static_layer，`N2/static:193-205, 254-256` 对 rolling 有专门分支 ⇒ 只是把静态图贴进滚动窗，语义与我们「局部只用实时传感器」不同 |
| `inflation_layer.cost_scaling_factor` | `5.0`（S:198） | `5.0`（R:232） | ✅ | **完全一致** |
| `inflation_layer.inflation_radius` | `0.55`（S:199） | `0.5`（R:235） | ⚠️ | 他们 0.55 对 0.1 m 车是 5.5 倍；我们 0.5 对 0.40 m 车（必须 ≥ `robot_radius`，`N2/infl:173-181` 会在小于内切半径时报 ERROR）。若我们要 0.55 也合法（0.55 > 0.40），可作为「更早看到障碍」的 A/B 项 |
| `stvl_voxel_layer.*` | S:200-262（含 `voxel_decay 0.5`、`decay_model 0`、`voxel_size 0.05`、`track_unknown_space true`、`observation_persistence 0.0`、`max_obstacle_height 2.0`、`mark_threshold 0`、`update_footprint_enabled true`、`combination_method 1`、`obstacle_range 3.0`、`origin_z 0.0`、`publish_voxel_map false`、`transform_tolerance 0.2`、`mapping_mode false`、`map_save_duration 60.0`） | 全局 stvl 同键名（R:288-333），局部**没有 stvl** | ✅（键名） | 键名逐条与我们的 STVL 配置同构（我们另加 `publish_voxel_map: true`、`livox_clear` 第二源）→ 这一块**可直接抄**；但**数据源不同**：他们 `topic: /livox/lidar_filtered`（S:221）+ `min_obstacle_height: 0.15`（S:229）；我们 `topic: /segmentation/obstacle`（R:303）+ `min_obstacle_height: 0.0`（R:311） |
| `realsense_source` | S:241-262（`/camera/camera/depth/color/points`，`model_type 0`，`min_z 0.15`，`min_obstacle_height 0.18`） | 无 | ❌ | 我们无 D435i；源话题在我们栈里不存在（RViz 能看到但 costmap 只会刷 MessageFilter 警告） |
| `always_send_full_costmap` | `False`（S:263） | `True`（R:236） | ⚠️ | 他们只要增量；我们 `True` 是为了 RViz/调试与上游 `isGoalReached()` 丢 transformPose 那个坑（见 R:154-157 与 LAUNCH:187-195 的长注释）。**不要抄 False**，会退回我们已踩过的坑 |

### A.6 global_costmap

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `width` / `height` | `25` / `25`（S:269-270） | 无（不滚动，尺寸随静态图） | ❌ | 他们全局 costmap 是 **rolling 25×25 m**（S:275）；我们是**非滚动 + 跟随 map_server 图**（`N2/static:193-205, 254-256` 在非 rolling 时会 resize 主 costmap） |
| `rolling_window` | `true`（S:275） | 无（=false） | ❌ | 这是两侧全局规划语义的**根本差异**：他们「地图只是滚动窗里的一块纹理」，我们「全局图就是地图」。他们的 `allow_unknown: true`（S:378）+ `track_unknown_space: true`（S:280）+ 窗口大于地图（25 m > MAP 12.75 m）⇒ **图外未知区可被规划器当作可通行**。我们正是要避免这个（R:158-178 的安全几何思路） |
| `update_frequency` | `20.0`（S:271） | `5.0`（R:241） | ⚠️ | 20 Hz 全局图在 sim 里是纯浪费（全局图只在重规划时用，我们 5 Hz + `expected_planner_frequency 5.0` R:355 已自洽） |
| `publish_frequency` | `20.0`（S:272） | `2.0`（R:242） | ⚠️ | 同上 |
| `global_frame` / `robot_base_frame` | `map` / `base_link_fake`（S:273-274） | `map` / `base_link_fake`（R:243-244） | ✅ | 一致 |
| `robot_radius` | `0.1`（S:277） | `0.40`（R:252） | ❌ | 同 A.5 |
| `footprint` | `±0.1`（S:278） | 无 | ⚠️ | 见 A.5 的 `use_radius_` 说明 |
| `resolution` | `0.04`（S:279） | `0.04`（R:253） | ✅ | **完全一致** |
| `track_unknown_space` | `true`（S:280） | `true`（R:254） | ⚠️ | 键值一致，但配合他们「rolling + allow_unknown」语义变成「图外可走」；我们是「图外=未知=不可走」的用法 |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（S:281） | `["static_layer","obstacle_layer","stvl_layer","inflation_layer"]`（R:258） | ✅（键名/结构） | **结构完全同构**：都是 static + (stvl/obstacle) + inflation；我们多一个 `obstacle_layer` 并靠 `enabled` 在 stvl/scan/none 间切（NAV:65-90）——这正是可以直接借鉴他们的「槽位」写法 |
| `static_layer.enable` | `true`（S:285） | `footprint_clearing_enabled: True`（R:336），无 `enable` | ⚠️ | **`static_layer.enable` 在 1.1.20 不是 nav2 `StaticLayer` 的参数**（1.1.20 的 StaticLayer 没有 enabled 概念；它是 `CostmapLayer`/`Layer` 但 `enable` 键只有部分图层读）。`map_subscribe_transient_local: True`（S:284 / R:337）是真键且两边一致 ✅ |
| `inflation_layer.cost_scaling_factor` | `5.0`（S:288，注释「与 local_costmap 一致」） | `8.0`（R:340） | ⚠️ | 他们 5.0 == 自己 local 5.0（S:198），注释成立；我们 global 8.0 ≠ local 5.0（R:340 vs R:232）。**注意 M 里这条注释与数值矛盾**（M:282 注释说一致，值却是 15.0），见 §F-3 |
| `inflation_layer.inflation_radius` | `0.75`（S:289） | `0.55`（R:342） | ⚠️ | 0.75 对 0.1 m 车；对我们 0.40 也合法（≥ 内切半径）。可作为「全局更早避障」的 A/B 项，但会加剧窄缝不可通行 |
| `stvl_voxel_layer.*` | S:290-352 | R:284-333 | ✅（键名） | 同 A.5；`min_obstacle_height` 他们 0.15（S:319）/ 我们 0.0（R:311，2026-09-24 特意调低，理由是全局图才决定规划） |
| `always_send_full_costmap` | `False`（S:353） | `True`（R:343） | ⚠️ | 同 A.5 |

### A.7 planner_server（Smac2D）

他们 = S:370-405（`nav2_smac_planner/SmacPlanner2D`，S:376）。**关键：1.1.20 的 `SmacPlanner2D` 只读 9 个键 + `smoother.*` 子块**（`N2/smac2:64-92` 逐个 `declare_parameter_if_not_declared`；`N2/smac_t:70-90` 的 `SmootherParams::get`）。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `plugin` | `nav2_smac_planner/SmacPlanner2D`（S:376） | `nav2_navfn_planner/NavfnPlanner`（R:359） | ⚠️ | `ros-humble-nav2-smac-planner 1.1.20` 已装 ⇒ 可切 |
| `tolerance` | `0.5`（S:377） | `0.5`（R:360） | ✅ | **完全一致**，且都是真键（`N2/smac2:64-66`；NavFn 侧 `navfn_planner.cpp:84`） |
| `allow_unknown` | `true`（S:378） | `true`（R:362） | ✅ | 一致（但见 A.6 rolling 的语义差） |
| `downsample_costmap` | `false`（S:379） | — | ✅ | 真键（`N2/smac2:67-69`），默认 false |
| `downsampling_factor` | `1`（S:380） | — | ✅ | 真键（`N2/smac2:70-72`） |
| `max_iterations` | `1000000`（S:381） | — | ✅ | 真键（`N2/smac2:80-82`），默认同 |
| `max_on_approach_iterations` | `1000`（S:382） | — | ✅ | 真键（`N2/smac2:83-85`），默认同 |
| `max_planning_time` | `4.5`（S:383） | — | ⚠️ | 真键（`N2/smac2:90-92`，默认 2.0）。4.5 s 在 20 Hz 控制下会拖慢；建议 1.0–2.0 |
| `cost_travel_multiplier` | `4.0`（S:384） | — | ⚠️ | 真键（`N2/smac2:73-75`，默认 1.0）。4.0 强烈贴通道中心——我们已有 `robot_radius 0.40` 的粗安全几何，1.0–2.0 更合适 |
| `smoother.max_iterations` | `10000`（S:401） | — | ✅ | 真键（`N2/smac_t:78-80`，默认 1000） |
| `smoother.w_smooth` | `0.4`（S:402） | — | ✅ | 真键（`N2/smac_t:84-86`，默认 0.3） |
| `smoother.w_data` | `0.1`（S:403） | — | ✅ | 真键（`N2/smac_t:81-83`，默认 0.2） |
| `smoother.tolerance` | `1.0e-10`（S:404） | — | ✅ | 真键（`N2/smac_t:75-77`，默认 1e-10） |
| `smoother.do_refinement` | `true`（S:405） | — | ✅ | 真键（`N2/smac_t:87-89`，默认 true） |
| **`motion_model_for_search: "DUBIN"`** | S:385 | — | ❌ | 只在 `N2/smac_planner_hybrid.cpp:137-138` 读；2D 计划器不读（`N2/smac2:114` 硬编码 `_motion_model = MotionModel::TWOD`） |
| **`angle_quantization_bins: 72`** | S:386 | — | ❌ | 同上（`smac_planner_hybrid.cpp:78-79`） |
| **`analytic_expansion_ratio: 3.5`** | S:387 | — | ❌ | Hybrid/Lattice 专用（`smac_planner_hybrid.cpp:121`、`analytic_expansion.cpp:71`） |
| **`analytic_expansion_max_length: 3.0`** | S:388 | — | ❌ | 同上（`smac_planner_hybrid.cpp:124`） |
| **`minimum_turning_radius: 0.05`** | S:389 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:100`）。0.05 m 转弯半径本身就是「差不多原地转」，只在差速上才有意义 |
| **`retrospective_penalty: 0.025`** | S:390 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:118`） |
| **`reverse_penalty: 1.0`** | S:391 | — | ❌ | Reeds-Shepp/Hybrid 专用（`smac_planner_hybrid.cpp:106`） |
| **`change_penalty: 0.0`** | S:392 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:109`） |
| **`non_straight_penalty: 0.0`** | S:393 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:112`）。注释写「必须 ≥ 1」但给 0.0 → 注释/值矛盾（若真在 Hybrid 上会告警） |
| **`cost_penalty: 2.0`** | S:394 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:115`）。2D 用的是 `cost_travel_multiplier` |
| **`rotation_penalty: 5.0`** | S:395 | — | ❌ | Lattice 专用（`node_lattice.cpp:66`） |
| **`lookup_table_size: 20.0`** | S:396 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:133`） |
| **`cache_obstacle_heuristic: True`** | S:397 | — | ❌ | Hybrid 专用（`smac_planner_hybrid.cpp:103`） |
| **`allow_reverse_expansion: True`** | S:398 | — | ❌ | Lattice 专用（`node_lattice.cpp:65`） |
| **`smooth_path: True`** | S:399 | — | ❌ | 只有 `smac_planner_hybrid.cpp:96` 读；2D 侧 `_smoother->smooth(...)` 是**无条件**调用（`N2/smac2:299-300`），所以写不写都一样平滑 |
| `use_astar` | 无 | `false`（R:361） | ✅ | NavFn 真键（`navfn_planner.cpp:86`）；保留我们的 |

> **结论**：他们 planner 段 30 行里，**16 行是 Hybrid/Lattice 专有、在 Smac2D 上完全无效**。可安全搬运的只有 `tolerance / allow_unknown / downsample_costmap / downsampling_factor / max_iterations / max_on_approach_iterations / max_planning_time / cost_travel_multiplier / smoother.*` 这几项。

### A.8 smoother_server

他们 = S:407-420。

| 键 | 他们（S 行=值） | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `costmap_topic` / `footprint_topic` | `global_costmap/costmap_raw` / `global_costmap/published_footprint`（S:409-410） | 我们**完全没有 `smoother_server:` 段**（R/D/T 全文无） | ✅（键名） | 1.1.20 `nav2_smoother` 真键；`nav2_smoother` 服务已由 `NAV:161-169`（非组合）/`NAV:244-248`（组合）拉起，走内置默认。抄这一段是纯增量 |
| `robot_base_frame` | `base_link_fake`（S:411） | 无（默认 `base_link`） | ⚠️ | 若抄必须一起改，否则 SmootherServer 的 TF 查询会找不到 `base_link`（我们只有 `base_link_fake`） |
| `transform_tolerance` | `0.1`（S:412） | 无 | ✅ | 真键 |
| `smoother_plugins` | `["savitzky_golay_smoother"]`（S:413） | 无 | ✅ | `nav2_smoother::SavitzkyGolaySmoother` 在 1.1.20 存在（`third_party/nav2/nav2_smoother/plugins.xml`） |
| `window_size` / `poly_order` | `7` / `3`（S:416-417） | 无 | ✅ | 真键，默认同 |
| `do_refinement` / `refinement_num` | `True` / `2`（S:418-419） | 无 | ✅ | 真键 |
| `enforce_path_inversion` | `True`（S:420） | 无 | ⚠️ | 真键；但**他们的 BT 里根本没有 `SmoothPath` 节点**（BT1:4-40、BTTP:4-45 只有 ComputePath*/RemovePassedGoals/FollowPath/ClearEntireCostmap）⇒ **整个 smoother_server 配了但从不被调用**（除了 RViz 手动调服务）。抄了也只是「多一个可用服务」 |

### A.9 velocity_smoother

他们 = S:471-483；我们 R:395-408。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `smoothing_frequency` | `30.0`（S:473） | `20.0`（R:398） | ⚠️ | 真键（`nav2_velocity_smoother/src/velocity_smoother.cpp:55`，默认 20.0） |
| `scale_velocities` | `False`（S:474） | `False`（R:399） | ✅ | 一致 |
| `feedback` | `OPEN_LOOP`（S:475） | `OPEN_LOOP`（R:400） | ✅ | **一致**；我们更旧的 `nav2_params.yaml:362` 是 `CLOSED_LOOP`，说明已演进到 OPEN_LOOP |
| `max_velocity` | `[2.5, 2.5, 2.5]`（S:476） | `[2.0, 2.0, 3.0]`（R:401） | ⚠️ | 真键。注意他们 `wz` 给 2.5 而 MPPI `wz_max 1.5`（S:101）→ **两处不一致**（注释却写「与 MPPI 限速一致」） |
| `min_velocity` | `[-2.5, -2.5, -2.5]`（S:477） | `[-2.0, -2.0, -3.0]`（R:402） | ⚠️ | 同上 |
| `deadband_velocity` | `[0.05, 0.05, 0.05]`（S:478） | `[0.0, 0.0, 0.0]`（R:403） | ⚠️ | 他们 0.05 会把 5 cm/s 以下的指令抹平（对 2.5 m/s 车无感）；我们 0 是为了低速精调（sim 0.5 m/s）⇒ **不要抄** |
| `velocity_timeout` | `1.0`（S:479） | `1.0`（R:404） | ✅ | 一致 |
| `max_accel` | `[2.5, 2.5, 1.5]`（S:480，注释「与 MPPI 加速度一致」） | `[4.0, 4.0, 6.0]`（R:405） | ⚠️ | **注释是错的**：MPPI 1.1.20 没有加速度参数（见 A.3 `ax_max` 行）。所以**这里才是唯一真正限加速度的地方**。他们的值对我们 0.5 m/s 车偏松，我们 4/4/6 偏激进 |
| `max_decel` | `[-3.5, -3.5, -3.0]`（S:481） | `[-4.0, -4.0, -6.0]`（R:406） | ⚠️ | 同上 |
| `odom_topic` | `"Odometry"`（S:482） | `"odom"`（R:407） | ✅ | 键名相同（`velocity_smoother.cpp:93`）；我们值已统一到 `/odom`（LAUNCH:256） |
| `odom_duration` | `0.1`（S:483） | `0.1`（R:408） | ✅ | 一致 |

> 我们 `LAUNCH:551-562` → `BP:130-140` → `NAV:210-220` 的 remap 链与他们 `NL:173-183` **逐字相同**（`cmd_vel→cmd_vel_nav`、`cmd_vel_smoothed→cmd_vel`），所以 `velocity_smoother` 的输出落在 `/cmd_vel`，正好是 FVT 的输入（FVT:11/31-32、FVTc:17）。这一层两边同构，**可直接沿用**。

### A.10 bt_navigator（+ BT XML）

他们 = S:1-62；我们 R:61-100。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `global_frame` / `robot_base_frame` | `map` / `base_link_fake`（S:4-5） | `map` / `base_link_fake`（R:64-65） | ✅ | 一致 |
| `odom_topic` | `Odometry`（S:6） | `/Odometry`（R:66） | ⚠️ | 真键（`nav2_bt_navigator/src/bt_navigator.cpp:98`，只用于 `OdomSmoother`，`:135`）。我们 `/Odometry` 在 T2 之后**没有发布者**（LAUNCH:256 把它 remap 成 `/odom`）→ BT 的测速一直为 0。建议改 `odom` |
| `bt_loop_duration` | `10`（S:7） | `10`（R:67） | ✅ | 真键（`N2/bta:57-58`，默认 10 ms）；一致 |
| `default_server_timeout` | 无（默认 20） | `20`（R:68） | ✅ | 真键（`N2/bta:60-61`）。他们靠默认值，我们显式写，等价 |
| `wait_for_service_timeout` | `1000`（S:9） | 无（默认 1000） | ✅ | 真键（`N2/bta:66-67`） |
| **`timeout: 100`** | S:8 | — | ❌ | **1.1.20 无此键**（`N2/bta:57-67` 只有 `bt_loop_duration`/`default_server_timeout`/`wait_for_service_timeout`），静默失效 |
| `default_nav_to_pose_bt_xml` | `$(find-pkg-share cod_bringup)/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`（S:11） | 无 → 走 nav2 内置 `navigate_to_pose_w_replanning_and_recovery.xml`（`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/…`） | ⚠️ | 见下方 BT 差异表 |
| `default_nav_through_poses_bt_xml` | `.../cod_bringup/.../navigate_through_poses_w_replanning_and_recovery.xml`（S:12） | 无 → 走 nav2 内置同名树 | ⚠️ | 同上 |
| `plugin_lib_names`（长列表） | S:15-62（含 `nav2_smooth_path_action_bt_node`（S:18）、`nav2_goal_updater_node_bt_node`、`nav2_path_longer_on_approach_bt_node`（S:56）等） | R:69-100（无 `nav2_smooth_path_action_bt_node`、无 `nav2_goal_updated_controller_bt_node`、无 `nav2_path_longer_on_approach_bt_node`、无 `nav2_is_battery_charging_condition_bt_node`） | ⚠️ | 我们的列表够跑内置 `navigate_to_pose/through_poses`（含 `nav2_compute_path_through_poses_action_bt_node` R:71、`nav2_remove_passed_goals_action_bt_node` R:97、`nav2_navigate_through_poses_action_bt_node` R:95）⇒ **多点/through-poses 我们现成可用**；缺的只是 `SmoothPath`、`PathLongerOnApproach`、`GoalUpdatedController`、电池条件这几个我们树里用不到的 |
| `ros__parameters` 之外的 `robot_state_publisher` 段 | S:456-458 | — | ❌ | 两个 launch 都不起 `robot_state_publisher`（SL:34-146、NL:111-193 无此节点）⇒ 惰性段 |
| `waypoint_follower` 段 | S:460-469（`loop_rate 20`、`stop_on_failure false`、`wait_at_waypoint`、`waypoint_pause_duration 200`） | R/D/T **无此段**；但旧文件 `src/rm_navigation/rm_navigation/params/nav2_params.yaml:347-355` **逐字相同**（含 200） | ⚠️ | 键名全真（`N2/wpf:37-42`、`N2/waw:46-56`）。**但 200 的单位是毫秒**（`N2/waw:78-81` `sleep_for(milliseconds(...))`）⇒ 只有 0.2 s，不是「停 2 秒」。见 §D |

**BT 树差异（他们 vs 内置 = 我们）**

| 项 | 他们 BT1 / BTTP | nav2 内置（= 我们） | 判定 | 理由 |
|---|---|---|---|---|
| 重规划率 | 3.0 Hz（BT1:8 / BTTP:8） | 1.0 Hz（to_pose）/ 0.333 Hz（through_poses） | ⚠️ | 3 Hz 配 2.5 m/s；我们 0.5 m/s 用 1 Hz 足够，且我们全局 costmap 只有 5 Hz（R:241）⇒ 抄 3 Hz 会白算 |
| `RecoveryNode number_of_retries` | `10`（BT1:6 / BTTP:6） | `6` | ⚠️ | 10 次配合 `failure_tolerance 0.3`（S:71）会长时间卡住 |
| `FollowPath` retries | `10`（BT1:19 / BTTP:24） | `1` | ⚠️ | 他们靠 10 次重试 + 清 costmap 硬扛；我们 1 次 |
| 恢复动作 | 只有「清两张 costmap」+（BTTP）`BackUp dist 1.0 / speed 1.0`（BTTP:40-41） | 清 costmap + `Spin 1.57` + `Wait 5` + `BackUp 0.30 / 0.05` | ❌（结构） | **他们删掉了 `Spin` 和 `Wait`**——合理：他们的 FVT 把 `angular.z` 强制换成 `spin_speed`（`FVTc:60`，launch 未传参 ⇒ 0.0，`third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18`）⇒ Spin 行为根本转不动。**我们 FVT 有修复（`FVT:80-84`：`spin_speed==0` 时角速度直通）**，所以我们的 Spin 是有效的，**不能照抄他们的树**（会把我们可用的恢复手段删掉）；他们的 `BackUp 1.0 m/s` 在我们 sim + 0.40 半径下也偏暴力 |
| `RemovePassedGoals radius` | `0.7`（BTTP:14） | `0.7` | ✅ | 一致 |

### A.11 behavior_server

他们 = S:422-454；我们 R:364-388。

| 键 | 他们（S 行=值） | 我们（R 行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `local_costmap_topic` / `global_costmap_topic` | `local_costmap/costmap_raw` / `global_costmap/costmap_raw`（S:425-426） | — | ❌ | **1.1.20 `nav2_behaviors` 无这两个键**（grep `"local_costmap_topic"` 于 `third_party/nav2/nav2_behaviors` 无命中；1.1.20 只有 `costmap_topic`/`footprint_topic`，`N2/beh:34-39`）。这是 Jazzy 重构后的键名 |
| `local_footprint_topic` / `global_footprint_topic` | S:427-428 | — | ❌ | 同上，无此键 |
| `costmap_topic` / `footprint_topic` | 他们**没写** | `local_costmap/costmap_raw` / `local_costmap/published_footprint`（R:369-370） | ✅ | **这才是 1.1.20 的键**（`N2/beh:34-39`、`:81-82`）。我们的才是对的 |
| `cycle_frequency` | `10.0`（S:429） | `10.0`（R:371） | ✅ | 一致（`timed_behavior.hpp:119`） |
| `behavior_plugins` | `["spin","backup","drive_on_heading","assisted_teleop","wait"]`（S:430） | `["spin","backup","drive_on_heading","wait"]`（R:372） | ⚠️ | 我们少 `assisted_teleop`；`nav2_behaviors/AssistedTeleop` 在 1.1.20 存在（`third_party/nav2/nav2_behaviors/behavior_plugin.xml:27`）。可加，非必需 |
| `backup.plugin` | `pb_nav2_behaviors/BackUpFreeSpace`（S:434） | `nav2_behaviors/BackUp`（R:376） | ❌ | 需要额外编译 `third_party/cod_nav_2026/src/pb_nav2_plugins`（`behavior_plugin.xml`），且它依赖 `nav2_msgs/srv/GetCostmap`（`back_up_free_space.cpp:39`，服务由 `N2` 的 `costmap_2d_publisher.cpp:81-82` 提供）。我们没这个包，抄了会 pluginlib 加载失败 |
| `spin/backup/drive_on_heading/wait/assisted_teleop.plugin`（其余） | S:431-440 | R:373-380 | ✅ | 与 1.1.20 的插件名逐字一致 |
| `local_frame` | `odom`（S:441） | — | ❌ | 1.1.20 behavior 只读 `global_frame`/`robot_base_frame`/`transform_tolerance`（`timed_behavior.hpp:119-122`）⇒ `local_frame` 无效 |
| `global_frame` | `map`（S:442） | `odom`（R:381） | ⚠️ | **两边不同且都有道理**：他们 map（配静态 map→odom）；我们 odom（配 amcl/icp 等重定位，避免恢复动作被定位跳变影响）。改成 map 会让恢复动作对定位跳变敏感 |
| `robot_base_frame` | `base_link_fake`（S:443） | `base_link_fake`（R:382） | ✅ | 一致 |
| `transform_tolerance` | `0.1`（S:444） | `0.1`（R:383） | ✅ | 一致 |
| `simulate_ahead_time` | `2.0`（S:445） | `1.0`（R:385） | ⚠️ | 真键（`N2/spin:54-56`、`drive_on_heading.hpp:233-236`）。2.0 会让 Spin/BackUp 预演更久，更保守但更慢 |
| `max_rotational_vel` | `1.0`（S:446） | `3.0`（R:386） | ⚠️ | 真键（`N2/spin:59-61`，默认 1.0）。我们 3.0 是刻意放大（R:364-368 注释提到曾被错段名吃掉过） |
| `min_rotational_vel` | `0.4`（S:447） | `0.4`（R:387） | ✅ | 一致（`N2/spin:65-68`） |
| `rotational_acc_lim` | `3.2`（S:448） | `3.0`（R:388） | ✅ | 真键（`N2/spin:70-72`），默认 3.2 |
| `robot_radius`（给 BackUpFreeSpace） | `0.1`（S:450） | — | ❌ | `back_up_free_space.cpp` 只 `get_parameter` 了 `global_frame/max_radius/service_name/visualize`（`:28-37`），**没读 `robot_radius`** ⇒ 死键 |
| `max_radius` | `3.5`（S:451） | — | ❌ | 仅 `pb_nav2_behaviors/BackUpFreeSpace` 读（`:29/35/88`），我们没这插件 |
| `service_name` | `global_costmap/get_costmap`（S:452） | — | ❌ | 同上；服务本身存在（`costmap_2d_publisher.cpp:81-82`，在 costmap 节点命名空间下 ⇒ `/global_costmap/get_costmap`） |
| `free_threshold` | `5`（S:453） | — | ❌ | `back_up_free_space.cpp` 未读取 ⇒ 死键 |
| `visualize` | `True`（S:454） | — | ❌ | 同上（真键但属该插件） |

### A.12 collision_monitor

| 项 | 他们 | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `collision_monitor:` 段 | **不存在**（`grep -rn collision_monitor S M NL ML LOC` 无命中） | 也不存在 | ✅（一致） | `ros-humble-nav2-collision-monitor 1.1.20` 已装，两边都没用。**这是一个双方都空着的槽位**：若要给 sim 加一层「速度级急停」，可以直接接（但需要新的 params 段 + `collision_monitor` 生命周期节点，属新增功能） |

### A.13 map_server / map_saver / 地图资产

| 键 | 他们 | 我们 | 判定 | 理由 |
|---|---|---|---|---|
| `map_server.yaml_filename`（S:360） | 硬编码 `/home/cod-sentry/dyx_ws/cod-sentry/src/cod_bringup/maps/rmul2026.yaml` | `""`（R:393，注释说明必须留键给 RewrittenYaml 替换） | ⚠️ | 他们的值**会被 `LOC:56-58` 的 `param_rewrites={'yaml_filename': map_yaml_file}` 覆写**（默认 `bringup_dir/maps/rmul2026.yaml`，`LOC:76-79`）⇒ 硬编码路径是噪音，但**他们的 `LOC` 用了 `root_key=namespace`**（LOC:63）而 `NL:68` 把 `root_key` **注释掉了** —— 这是我们和他们 launch 的一处真实差异（见 §F-9） |
| `map_saver.save_map_timeout` | `5.0`（S:365） | `5.0`（R:348） | ✅ | 一致 |
| `map_saver.free_thresh_default` | `0.25`（S:366） | `0.25`（R:349） | ✅ | 一致 |
| `map_saver.occupied_thresh_default` | `0.65`（S:367） | `0.65`（R:350） | ✅ | 一致 |
| `map_saver.map_subscribe_transient_local` | `True`（S:368） | `True`（R:351） | ✅ | 一致 |
| 地图 `resolution` | `0.050`（MAP:3） | 我们的图是 `map/RMUL2026.yaml`（LAUNCH:60）— 不同资产 | ⚠️ | 他们 0.05 m/px、255×177 px（`rmul2026.pgm` 头 `P5 255 177`）= **12.75 m × 8.85 m**；origin `[-1.756, -7.036, 0]`（MAP:4）⇒ x∈[-1.756, 10.994]、y∈[-7.036, 1.814]。**他们 `global_costmap` 的 25×25 m 滚动窗比地图大 2 倍**（S:269-270 vs 12.75×8.85）⇒ 图外未知区参与规划（见 A.6） |
| 地图 `mode/negate` | `trinary` / `0`（MAP:2/5） | — | ✅ | 常规 |
| 地图 `occupied_thresh` / `free_thresh` | `0.65` / **`0.196`**（MAP:6-7） | 我们图资产不同 | ⚠️ | `0.196` 与他们 `map_saver.free_thresh_default 0.25`（S:366）**不一致** ⇒ §F-10 |

### A.14 slam_toolbox（`SLAMc` vs `SLAMrm`）

他们的 `SLAMc` 只被 **ML:85-94** 用（多点导航）；`SL`（单点）**不起 slam_toolbox**，靠 `map_server` + 静态 `map→odom`。我们的 slam_toolbox 由 `LAUNCH:503-512`（mapping/slam_nav 且 `mapper:=slam_toolbox`）拉起。

| 键 | `SLAMc`（行=值） | `SLAMrm`（行=值） | 判定 | 理由 |
|---|---|---|---|---|
| `mode` | `lifelong`（SLAMc:19） | `mapping`（SLAMrm:17） | ⚠️ | lifelong 会持续改写地图（配 `use_map_saver: true` SLAMc:18）；我们 sim 要的是可复现的 `mapping` ⇒ **不要抄** |
| `base_frame` | `base_link`（SLAMc:16） | `livox_frame`（SLAMrm:15） | ⚠️ | 我们刻意用雷达帧（`P2L` 的 `target_frame: ""` 意味着 `/scan` 的 frame 就是点云帧 `livox_frame`，P2L:9）；抄 `base_link` 会因缺 `base_link→livox_frame` 链路（我们 TF 里是 `livox_frame` 挂 `imu_link`）而报错 |
| `odom_frame` / `map_frame` / `scan_topic` | `odom` / `map` / `/scan`（SLAMc:14/15/17） | 同（SLAMrm:13/14/16） | ✅ | 一致 |
| `transform_publish_period` | `0.0`（SLAMc:30，注释「if 0 never publishes odometry」） | `0.02`（SLAMrm:28） | ❌ | 0.0 = **slam_toolbox 不发 `map→odom`**！这与他们 `ML:95-116` 那条静态 `map→odom` 搭配才成立 → 见 §F-11 |
| `map_update_interval` | `1.0`（SLAMc:31） | `5.0`（SLAMrm:29） | ⚠️ | 1 Hz 重算栅格对 sim CPU 不友好；我们 5 s 更稳 |
| `minimum_time_interval` | `0.3`（SLAMc:35） | `0.5`（SLAMrm:32） | ⚠️ | 我们 0.5 s 更省 |
| `min_laser_range` | `0.2`（SLAMc:33） | 无 | ⚠️ | 我们靠 `P2L.range_min: 0.05`（P2L:37）与 `inf_is_valid` 处理近距；抄 0.2 会把贴身回波丢掉（正是我们 2026-09-26 踩过的坑） |
| `max_laser_range` | `10.0`（SLAMc:34） | `20.0`（SLAMrm:31） | ⚠️ | 与 `P2L.range_max: 10.0`（P2L:38）一致性：他们的 10.0 与 p2l 对齐；我们 20.0 与 `range_max 10` 不一致（无害但语义含糊） |
| `minimum_travel_distance` / `minimum_travel_heading` | `1.0` / `0.1`（SLAMc:44-45） | `0.5` / `0.5`（SLAMrm:41-42） | ⚠️ | 影响图节点密度；我们 sim 0.5/0.5 更密 |
| `throttle_scans` / `use_scan_matching` / `do_loop_closing` / 求解器块 / 相关参数块 | SLAMc:29/42/51/5-10/57-76 | SLAMrm 同键同值 | ✅ | **除上表所列外逐字相同**（两边同源）⇒ 这部分可直接互换 |
| `use_map_saver` | `true`（SLAMc:18） | 无（默认 false） | ⚠️ | 只有 lifelong 模式有意义 |

### A.15 我们的 `D` / `T` 变体与他们的关系

`D` 与 `T` 相对 `R` **只差 `FollowPath` 段（+几行注释）**，其余（两张 costmap、planner、behavior、velocity_smoother、map_server）逐字相同：`diff -u R D` 只有 3 个 hunk（`@@ -124,38` FollowPath、`@@ -187,9`、`@@ -262,10`，后两个只是注释删除）。

- DWB（D:128-166）：`max_vel_x/y 2.5`（D:133-134）与他们的 MPPI `vx_max/vy_max 2.5`（S:99-100）**同量级**；`max_vel_theta 12.0`（D:135）远超他们 `wz_max 1.5`（S:101）——**DWB 的角速度上限在我们侧是空转的**（被 `velocity_smoother.max_velocity[2]=3.0`，R:401 截住）。
- TEB（T:129-165）：`max_vel_x 0.5`（T:144）、`footprint_model.circular.radius 0.2`（T:149-151）、`min_obstacle_dist 0.27`（T:152）、`weight_kinematics_nh 1000`（T:157）——**TEB 的 `footprint_model.radius 0.2` 与两张 costmap 的 `robot_radius 0.40` 不一致**（T:150 vs R:178/252），这是一个独立于 cod_nav 的现存不一致，接入 MPPI 前后都值得一并修（见 §F-13）。
- 他们没有任何等价于 DWB/TEB 的段：`FollowPath` 只有 wrapper+MPPI 一种。

### A.16 「1.1.20 里不存在」死键汇总（照抄会静默失效）

| 死键 | 出现位置 | 1.1.20 只在哪里存在 |
|---|---|---|
| `publish_critics_stats` | S:65 | 全 nav2 无 |
| `vy_min`, `wz_min` | S:104-105 | Jazzy/main 的 MPPIController |
| `ax_max`, `ay_max`, `az_max`, `ax_min`, `ay_min`, `az_min` | S:107-113 | 同上 |
| `CostCritic.trajectory_point_step` | S:159 | 只有 PathAlign/PathAlignLegacy critic 读（`N2/mppi/pac:34`）；CostCritic 的 `getParam` 清单里没有（`N2/mppi/cc:26-32`） |
| `path_length_tolerance` | S:83 | 新版 `PositionGoalChecker`（1.1.20 只有 `xy_goal_tolerance`/`stateful`，N2/ppgc:44-52） |
| `timeout`（bt_navigator） | S:8 | 无（1.1.20 用 `default_server_timeout`/`wait_for_service_timeout`，N2/bta:57-67） |
| `motion_model_for_search`, `angle_quantization_bins`, `analytic_expansion_ratio`, `analytic_expansion_max_length`, `minimum_turning_radius`, `retrospective_penalty`, `reverse_penalty`, `change_penalty`, `non_straight_penalty`, `cost_penalty`, `rotation_penalty`, `lookup_table_size`, `cache_obstacle_heuristic`, `allow_reverse_expansion`, `smooth_path` | S:385-399 | 只有 Hybrid/Lattice 计划器读（smac_planner_hybrid.cpp / node_lattice.cpp） |
| `local_costmap_topic`, `global_costmap_topic`, `local_footprint_topic`, `global_footprint_topic`, `local_frame` | S:425-428, 441 | Jazzy 版 behavior_server；1.1.20 用 `costmap_topic`/`footprint_topic`（`N2/beh:34-39`） |
| `robot_radius`, `free_threshold`（behavior_server 段） | S:450, 453 | pb 插件也没读（back_up_free_space.cpp:28-37） |
| `static_layer.enable` | S:285 | 1.1.20 `StaticLayer` 不读（只有部分图层有 `enabled`） |

---

## B. `M`（multiplenav2）相对 `S` 的差异清单

`diff -u S M` 的全部差异如下（**没有遗漏任何 hunk**）。

| # | 键 | S | M | 判定 | 为什么不同（多点/through-poses 视角） |
|---|---|---|---|---|---|
| B1 | `bt_navigator.default_nav_to_pose_bt_xml` | `cod_bringup` 自定义树（S:11） | nav2 内置树（M:10） | ⚠️ | 多点模式**回退到官方树**：官方树带 `Spin 1.57`+`Wait 5` 恢复，自定义树只有清理+BackUp。与 B2 的方向一致（多点模式不想让单点的激进改造影响连贯穿越） |
| B2 | `bt_navigator.default_nav_through_poses_bt_xml` | `cod_bringup` 自定义（S:12） | nav2 内置（M:11） | ⚠️ | **through-poses 才是多点的核心**：M 明确用官方 through-poses 树（`RemovePassedGoals 0.7` + `0.333 Hz` 重规划，见 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:10-14`） |
| B3 | `controller_server.publish_critics_stats` | `True`（S:65） | 删除 | ❌ | 死键，删了无影响（§A.16） |
| B4 | goal checker 插件 | `PositionGoalChecker`（S:81） | `SimpleGoalChecker`（M:77） | ⚠️ | 见下两行 |
| B5 | `general_goal_checker.stateful` | `False`（S:80） | `True`（M:76） | ⚠️ | through-poses 下车要走「一串点」，stateful 锁存可避免在通过中途因抖动反复判定 |
| B6 | `general_goal_checker.xy_goal_tolerance` | `0.2`（S:82） | `0.30`（M:78，注释写「放宽到25cm」） | ⚠️ | 巡逻点不需要 0.2 m 精度；**注释与数值矛盾**（§F-2） |
| B7 | `general_goal_checker.yaw_goal_tolerance` | 无（PositionGoalChecker 无此键） | `6.28`（M:79） | ⚠️ | 6.28 rad ≈ 360° ⇒ **等价于忽略朝向**，但用的是 SimpleGoalChecker（有该键）。效果与 PositionGoalChecker 相同，可能是为了绕开 PositionGoalChecker 在 through-poses 下与 `path_length_tolerance`（死键）的混淆 |
| B8 | `FollowPath.direct_approach_distance` | `2.0`（S:89） | **删除** → 取 GAC:46 默认 **0.5** | ⚠️ | **多点模式主动关掉「近距离直接驱动」**：连续穿越时若末端强插一段直冲，会与 MPPI 的路径跟踪打架；单点任务才需要防画弧 |
| B9 | `FollowPath.direct_approach_kp` | `3.0`（S:90） | **删除** → 取 GAC:49 默认 **1.0** | ⚠️ | 同上；给默认 1.0 时 `speed = min(0.2, dist*1.0)`，0.2 m/s 上限仍在（GAC:110） |
| B10 | `FollowPath.model_dt` | `0.02`（S:92） | `0.06`（M:86，同一句注释） | ⚠️ | 时域从 **1.6 s → 4.8 s**（80×0.06）。注释「配合 time_steps 延长预测时域」只在 M 成立（§F-1）。代价：周期 0.02 < 0.06 ⇒ 只 WARN（N2/mppi/opt:100-103） |
| B11 | `FollowPath.vx_std` / `vy_std` | `0.3 / 0.3`（S:95-96） | `0.35 / 0.35`（M:89-90） | ⚠️ | 连续穿越需要更大多样性 |
| B12 | `FollowPath.az_max` | `1.0`（S:109） | `3.0`（M:103） | ❌ | 死键（§A.16）；两边都无效 |
| B13 | `FollowPath.retry_attempt_limit` / `regenerate_noises` | 无（默认 1 / false） | `2` / `true`（M:117-118） | ⚠️ | 长路径跟踪更容易出现优化失败，所以提高重试 + 每次重采样噪声 |
| B14 | `GoalCritic.cost_weight` / `threshold_to_consider` | `25.0` / `4.0`（S:139-140） | `20.0` / `3.0`（M:132-133） | ⚠️ | 多点不需要那么强的「末端吸引」，改为更早让 GoalCritic 退出、PathFollow 主导 |
| B15 | `ObstaclesCritic.collision_margin_distance` / `near_goal_distance` | `0.2` / `0.5`（S:148-149） | `0.25` / `0.1`（M:142-143） | ⚠️ | 更宽的安全余量 + 更小的「近目标豁免区」（巡逻点常在障碍附近） |
| B16 | `PathFollowCritic.cost_weight` / `threshold_to_consider` | `15.0` / `1.0`（S:164-166） | `8.0` / `3.0`（M:158/160） | ⚠️ | 权重降一半：多点靠「一直跟着路径」，不靠末端爆发 |
| B17 | `PathAlignCritic.threshold_to_consider` | `0.8`（S:174） | `3.0`（M:168） | ⚠️ | 与 B16 同步（两个 threshold 都是 3.0），让路径对齐一直生效到接近目标 |
| B18 | `local_costmap.robot_radius` / `footprint` | `0.1` / `±0.1`（S:190-191） | `0.2` / `±0.15`（M:184-185） | ⚠️ | 多点模式保守化。**但 footprint 非空 ⇒ 有效内切半径 = 0.15（不是 0.2）**，`robot_radius` 被忽略（N2/cmros:408-416） |
| B19 | `local_costmap.inflation_layer.inflation_radius` | `0.55`（S:199） | `0.75`（M:193） | ⚠️ | 与 B18 配套加宽 |
| B20 | `local_costmap.stvl.voxel_decay` / `observation_persistence` | `0.5` / `0.0`（S:203/207） | `1.0` / `0.5`（M:197/201） | ⚠️ | 连续穿越多点，需要更久保留障碍航迹（多点巡逻路径反复经过同一区） |
| B21 | `local/global stvl.livox_source.min_obstacle_height` | `0.15`（S:229/319） | `0.1`（M:223/313） | ⚠️ | 更激进地看到矮障碍（多点巡逻会贴场地元素） |
| B22 | `realsense_source.min_z` / `min_obstacle_height` | `0.15` / `0.18`（S:250/252、340/342） | `0.1` / `0.1`（M:244/246、334/336） | ❌（对我们） | 我们无 D435i |
| B23 | `local/global always_send_full_costmap` | `False`（S:263/353） | `True`（M:257/347） | ⚠️ | 多点/through-poses 需要更频繁拿到完整图（我们本来就是 True，见 A.5） |
| B24 | `global_costmap.width` / `height` | `25/25`（S:269-270） | `50/50`（M:263-264） | ⚠️ | 多点巡逻覆盖更大范围；50 m×0.04 = **1250×1250 格**（约为主图的 15 倍面积） |
| B25 | `global_costmap.inflation_layer.cost_scaling_factor` | `5.0`（S:288） | `15.0`（M:282，注释仍写「与 local_costmap 一致」，而 M local 是 5.0） | ⚠️ | 全局代价衰减更陡 ⇒ 全局路径更贴墙、更短；**注释与数值矛盾**（§F-3） |
| B26 | `global_costmap.robot_radius` / `footprint` | `0.1` / `±0.1`（S:277-278） | `0.2` / `±0.15`（M:271-272） | ⚠️ | 同 B18 |
| B27 | `map_server.yaml_filename` | 硬编码绝对路径（S:360） | `""`（M:354） | ✅ | **M 的写法与我们一致**（R:393）⇒ 若要抄这段，抄 M 的 |
| B28 | `behavior_server.use_sim_time` | `false`（S:424） | `true`（M:418） | ❌ | 与全文其它 `false` 矛盾，且会被 `NL:61-71` 的 RewrittenYaml 用 launch 参数（"false"）覆写 ⇒ **惰性**（§F-7） |
| B29 | `behavior_server.robot_radius`（pb 插件） | `0.1`（S:450） | `0.2`（M:444） | ❌ | 死键（该插件不读，§A.11） |
| B30 | `velocity_smoother.max_accel` | `[2.5, 2.5, 1.5]`（S:480） | `[2.5, 2.5, 3.0]`（M:474） | ⚠️ | 与 B12（`az_max 3.0`）同一意图：多点需要更快的偏航加速。但 MPPI 侧无效，真正生效的是这里 |
| B31 | 文件末尾换行 | 有 | **无** | — | 纯格式 |

**B 小结（对多点/through-poses 的真正含义）**：M 相对 S 的改动可归纳成四条主线——①**关掉末端直接驱动**（B8/B9）让穿越连续；②**把「路径跟随」权重体系整体换成「长时间跟线」**（B14/B16/B17）；③**几何/障碍保守化并放大全局窗**（B18-B26）；④**goal checker 从「只判位置」换成「判位置 + 忽略朝向」**（B4-B7）。这与「单点要防画弧、多点要顺滑穿越」的工程直觉一致。

---

## C. `goal_approach_controller` 解读

### C.1 它是什么、怎么代理 MPPI

| 项 | 内容 | 证据 |
|---|---|---|
| 类型 | `nav2_core::Controller` 的 **pluginlib 插件**（wrapper/装饰器），不是独立节点 | GAC:20-24、GAC:151-153；GACX:1-9 |
| 导出 | `<library path="goal_approach_controller">` + `<class name="goal_approach_controller::GoalApproachController" base_class_type="nav2_core::Controller">` | GACX:1-9 |
| 包依赖 | `rclcpp, rclcpp_lifecycle, geometry_msgs, nav_msgs, nav2_core, nav2_util, nav2_costmap_2d, pluginlib, tf2_ros` | GACC:7-16、package.xml:12-20 |
| 内部控制器加载 | 运行时用 `pluginlib::ClassLoader<nav2_core::Controller>("nav2_core","nav2_core::Controller")` + `createUniqueInstance(inner_plugin_type)`，再 `inner_controller_->configure(parent, name, tf, costmap_ros)` | GAC:60-63 |
| **参数命名空间** | `configure(parent, name, …)` 里 `name` **原样透传**给内部 MPPI ⇒ MPPI 的所有参数与 wrapper 参数**同级**在 `FollowPath.*` 下 | GAC:37-57、:63；S:84-176 正是这种扁平写法 |
| 生命周期 | `cleanup/activate/deactivate` 直接转调内部（无自身状态需要清） | GAC:73-86 |
| 速度上限 | `setSpeedLimit` 直接转调内部（wrapper 不额外限速） | GAC:133-136 |
| 头文件/安装 | 仓库里**没有 `include/` 目录**（`GACC:33-36` 指向不存在的 `include`，无害）；`.cpp` 是唯一源文件 | `ls third_party/cod_nav_2026/src/goal_approach_controller/` → 只有 `src/` |
| `on_configure` 里无 try/catch | 内部插件类型写错会直接抛到 controller_server | GAC:62 |

### C.2 接口：订阅/发布什么

**它自己不订阅也不发布任何话题**（`GAC` 全文无 `create_subscription`/`create_publisher`）。它是 Nav2 controller 插件，靠 **Nav2 的 C++ 调用链**工作：

| 方向 | 接口 | 时机 | 证据 |
|---|---|---|---|
| ← 输入 | `setPlan(const nav_msgs::msg::Path&)` | controller_server 每次收到新路径（`N2/ctrl:445-453`） | GAC:88-94 |
| ← 输入 | `computeVelocityCommands(pose, velocity, goal_checker)` | 每个控制周期（`N2/ctrl:470-485`） | GAC:96-101 |
| → 输出 | `geometry_msgs::msg::TwistStamped`（返回给 controller_server，再由其发到 `cmd_vel`→remap `cmd_vel_nav`） | 每周期 | GAC:130；`NL:122`/`NAV:159` |
| → 内部副作用 | 内部 MPPI 自己的可视化/轨迹发布（`/trajectories`、`transformed_global_plan`、`visualize`） | 每周期 | `N2/mppi/tv:29-30`；S:121 `visualize: false` |

**关键实现细节（也是第一处硬伤）**：

```cpp
// GAC:88-94
void setPlan(const nav_msgs::msg::Path & path) override {
  if (!path.poses.empty()) { goal_ = path.poses.back(); }   // ← 空路径时保留【上一次】的目标（陈旧）
  inner_controller_->setPlan(path);
}
// GAC:103-106
double dx = goal_.pose.position.x - pose.pose.position.x;
double dy = goal_.pose.position.y - pose.pose.position.y;
```

- `goal_` 来自**全局规划器给的路径末点 ⇒ `map` 帧**（我们的 planner 的 `global_frame: map`，R:243）。
- `pose` 来自 `N2/ctrl:611-617` → `costmap_ros_->getRobotPose()` → 局部 costmap 的 `global_frame` = **`odom`**（R:161；`N2/cmros:639-644`）。
- ⇒ `dx/dy` **在混帧相减**。MPPI 自己不会犯这个错：它用 `N2/mppi/ph:69-98,145-161` 把路径转到 costmap 帧。wrapper 没有这一步。
- 危害量级：等于 `map→odom` 的旋转。他们 `SL:81-82` 的静态桥是 **yaw = −0.5 rad（−28.6°）** ⇒ 单点导航最后 2 m 的驱动方向**偏 28.6°**；他们 `ML:109-110` 是 **yaw = 0.0** ⇒ 多点模式反而没这个误差。我们自己 `mode=nav, localization=''` 的回退静态桥是 `camera_init→map`/`body→odom` 单位旋转（LAUNCH:452-468），若用 amcl/icp/slam_toolbox/cartographer，则 `map→odom` 会**漂移**，误差是变化的，比固定 0.5 rad 更糟。
- 次要：`setPlan` 在空路径时不清 `goal_`；`goal_checkers_[…]->reset()`（`N2/ctrl:457`）不影响 wrapper 的 `goal_`。

### C.3 参数表（wrapper 自有 5 个 + 建议值）

| 参数 | 默认值（声明处） | 他们（S 行=值） | 含义 | 对 sim 的建议 |
|---|---|---|---|---|
| `inner_plugin` | `nav2_mppi_controller::MPPIController`（GAC:36-38） | `nav2_mppi_controller::MPPIController`（S:86） | 被装饰的控制器插件名 | 保持；也可换 `dwb_core::DWBLocalPlanner` 做 A/B |
| `approach_distance` | `1.5`（GAC:39-41） | `2.5`（S:87） | 距目标小于此值时**按比例钳位合速度**到 `approach_velocity` | 0.8–1.5（我们 0.5 m/s 巡航，2.5 m 太早减速） |
| `approach_velocity` | `0.5`（GAC:42-44） | `0.2`（S:88） | 钳位后的合速度上限（m/s） | 0.15–0.2（配 `xy_goal_tolerance 0.25`，R:124） |
| `direct_approach_distance` | `0.5`（GAC:45-47） | `2.0`（S:89）；**M 删除 ⇒ 0.5** | 距目标小于此值时**绕过内部控制器**，直接朝目标平移 | 0.3–0.5（见 C.4：「直接驱动」与我们的 spin 语义冲突，越小越好） |
| `direct_approach_kp` | `1.0`（GAC:48-50） | `3.0`（S:90）；M 用默认 1.0 | 直接驱动段的 P 增益 | 1.0–2.0；注意 `speed = min(approach_velocity, dist*kp)`（GAC:110），kp≥3 且 dist≥0.067 时**恒等于** `approach_velocity`，增益实际失效 |

> 声明方式 `nav2_util::declare_parameter_if_not_declared`（GAC:36-50）⇒ 这些键**可以**写在 `FollowPath.` 下（S:87-90 就是这么写的），不写就是上表默认。

### C.4 `direct_approach_*` 分支与 `angular.z = 0` 的确切行为

```cpp
// GAC:108-128
if (dist < direct_approach_distance_) {            // ① 直接驱动分支
  double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
  if (dist > 0.01) {
    cmd.twist.linear.x = target_speed * (dx / dist);   // ② 把【map 帧的方向余弦】直接当成【底盘 x 指令】
    cmd.twist.linear.y = target_speed * (dy / dist);
  } else { cmd.twist.linear.x = 0.0; cmd.twist.linear.y = 0.0; }
  cmd.twist.angular.z = 0.0;                       // ③ 角速度强制清零
} else if (dist < approach_distance_) {            // ④ 仅限速分支
  double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
  if (speed > approach_velocity_) {
    double scale = approach_velocity_ / speed;
    cmd.twist.linear.x *= scale; cmd.twist.linear.y *= scale;
    cmd.twist.angular.z *= scale;                  // ⑤ 角速度跟着等比缩小
  }
}
```

**逐条含义**：

1. **① 进入条件**：只与「路径末点」的欧氏 XY 距离有关（GAC:104-108），与朝向、速度、路径形状都无关。
2. **② 方向**：直接把 `dx/dist, dy/dist` 塞进 `Twist.linear.x/y`。Nav2 里 `Twist` 是**底盘坐标系**（`base_link_fake`）下的指令 ⇒ 这段代码等价于「假设底盘朝向与 `odom` 一致」。在他们的云台小陀螺架构下**恰好近似成立**（他们 FVT 把 `base_link_fake` 的 yaw 恒定成 `-base_link_yaw`，`FVTc:44-52`，使 `base_link_fake` 在 odom 里近似不转）；在我们 `FVT` 的实现里 `base_link_fake` 的 yaw = `current_angle_`（`FVT:105-115`），而 `current_angle_` 来自 `/local_plan`（`FVT:35-37`、`FVT:46-67`）——**详见 C.5**。
3. **③ `angular.z = 0.0` 的确切行为**：wrapper **单方面把角速度写成 0**（GAC:118），并且是在**内部 MPPI 已经算完之后覆盖**（GAC:101 → GAC:118）。后果分三层：
   - 对 **MPPI 的求解**没有任何影响（MPPI 已经返回了含 `angular.z` 的指令，只是被丢弃）；
   - 对 **底盘**的影响取决于下游 FVT：他们 `FVTc:60` 无条件 `angular.z = spin_speed_`（launch 未传参 ⇒ 0.0，见 `third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18`）⇒ **底盘角速度恒为 0**，机器人靠 vx/vy 平移到位，**终点朝向完全不受控**；
   - 对**我们**：`FVT:94` 是 `aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;` ⇒ `angular.z = 0` 走进「0」分支 ⇒ **底盘停止自转**（且 `spin_speed` 非 0 时，这一刻从「小陀螺」变成「不转」）。同时 `FVT:91-96` 仍会用 `-current_angle_` 旋转线速度 ⇒ **线速度方向取决于 `/local_plan`，而 `/local_plan` 在我们栈里没有任何发布者**（见 C.5）。
4. **④/⑤ 限速分支**：只按**合速度**等比缩小，方向不变；`angular.z` 也乘同一个 scale（GAC:120-127）。所以他们 `approach_distance 2.5 / approach_velocity 0.2` 的效果是「距目标 2.5 m 内合速度被压到 ≤0.2 m/s」——**这会让最后 2.5 m 变得非常慢**（2.5 m ÷ 0.2 m/s ≈ 12.5 s），而 `direct_approach_distance 2.0` 又在这段里覆盖了它 ⇒ 两个分支在 2.0 m 内**同时**满足，代码按 `if/else if` 只走 ①，所以实际是「2.5→2.0 m 之间限速，<2.0 m 直接驱动」。
5. **无 `dist` 最小值保护**：`dist ≤ 0.01` 时停车（GAC:114-117），但**仅在 `< direct_approach_distance_` 时**；若 `direct_approach_distance` 被设成 0（有人会这么干），整个直接分支永不触发，只剩限速分支 ⇒ 行为完全不同。
6. **不做 TF 查询、也不做失败处理**：`GAC:104-106` 是纯算术 ⇒ 即使 TF 断了也会照样发指令（可能方向完全错），没有 `PlannerException`。

### C.5 与我们 `fake_vel_transform` / spin 模式的兼容性

先给事实基线（两边 FVT 实现不同）：

| 维度 | 他们 `FVTc` | 我们 `FVT` |
|---|---|---|
| `spin_speed` 默认/实际 | `0.0`（`FVTc:19`），launch 不传参（`fake_vel_transform_launch.py:9-18`） | 包内默认 `-6.0`（`FVT:22`），但**实际参数文件是 `5.0`**（`FVTC:3`），launch 也默认 `5.0`（`LAUNCH:114`），并把 `spin_speed` 当 launch 参数覆盖（`LAUNCH:417-419`） |
| TF `robot_base→fake` | 在 **odom 回调**里用 `-yaw(odom)`（`FVTc:40-52`）⇒ fake 帧在 odom 里近似**恒定朝向** | 在 **20 Hz 定时器**里用 `current_angle_`（`FVT:40-42`、`FVT:105-115`），`current_angle_` = `teb_angle - base_link_angle_`，来自 **`/local_plan` 路径**（`FVT:46-67`） |
| 角速度处理 | **无条件** `angular.z = spin_speed_`（`FVTc:60`） | `spin_speed_ == 0` ⇒ **原样直通**（`FVT:80-84`，你们的修复）；否则 `angular.z = (angular.z != 0) ? spin_speed_ : 0`（`FVT:94`） |
| 线速度旋转 | 按 `current_robot_base_angle_`（odom 里的真实底盘角）旋转（`FVTc:57-63`） | 按 `-current_angle_` 旋转（`FVT:91-98`） |
| 输入/输出话题 | `cmd_vel` → `aft_cmd_vel`（`FVTc:17-18`） | `/cmd_vel` → `/cmd_vel_chassis`（`FVT:11-12`） |
| `/local_plan` 输入 | 不订阅 | 订阅（`FVT:13`、`FVT:35-37`）——**但 1.1.20 的 nav2 没有任何节点发布 `local_plan`**（`grep -rn local_plan third_party/nav2/nav2_controller/src` 无命中；对已安装 `libmppi_controller.so` 取字符串也没有）。发布者是 cod_nav 里的 **`pb_omni_pid_pursuit_controller`**（`third_party/cod_nav_2026/src/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:164`），而**我们没用那个控制器**（R:129/D:129/T:130 分别是 RPP/DWB/TEB） |

**兼容性矩阵（我们现有栈）**：

| 场景 | `angular.z` 进 FVT | 我们 FVT 的行为 | 对 goal_approach_controller 的后果 |
|---|---|---|---|
| `spin_speed:=0.0`（`LAUNCH:114`/`:415-419` 可覆盖） | `0` | 直通模式（`FVT:80-84`）：角速度 0、线速度**不再旋转**、TF 恒等 ⇒ `base_link_fake ≡ base_link` | ✅ **这是唯一干净的组合**：direct-approach 的 `angular.z=0` 与「小陀螺关掉」自洽；但代价是**整个导航栈没有自旋**，纯靠 omni 平移（我们 RPP 默认 `use_rotate_to_heading: true`，R:146 ⇒ 会先转朝向再走，转了也没意义，等于白等） |
| `spin_speed:=5.0`（默认，`FVTC:3`） | `0` | 走 `(angular.z != 0) ? spin_speed_ : 0` 的 **0 分支** ⇒ **底盘停止自转**；同时线速度按 `-current_angle_` 旋转，而 `current_angle_` **恒为 0**（因为 `/local_plan` 没人发）⇒ 实际是「直通线速度 + 角速度 0」 | ⚠️ 「小陀螺」在末端 2 m 内被**关掉**。若我们真的想要小陀螺行为，这与 §C.4③ 直接冲突；若我们只想让导航跑通，结果与 `spin_speed:=0` 等价，但**其它阶段**（>2 m）角速度非 0 时又会被替换成 5.0 rad/s ⇒ **同一次导航里角速度语义前后不一致**（2 m 外是 5 rad/s，2 m 内是 0） |
| `spin_speed:=0.0` 且 `direct_approach_distance: 0`（禁用直接分支） | MPPI 的原始 `angular.z` | 角速度直通（`FVT:80-84`） | ✅ 兼容，但就**放弃了 wrapper 的核心功能**，只剩限速分支 |

**结论**：`angular.z = 0` 在**他们**的栈里是「无所谓」（角速度反正被 `FVTc:60` 覆盖成 `spin_speed`）；在**我们**的栈里是一个**语义开关**——它会关掉小陀螺、并让我们 FVT 里那段依赖 `/local_plan` 的线速度旋转逻辑退化成空转（因为 `current_angle_` 本来就不更新）。所以：

- 若我们要用 `goal_approach_controller`，**推荐 `spin_speed:=0.0` + 把 `direct_approach_distance` 收到 0.3–0.5**（既保住「末端防画弧」，又避免「>2 m 与 <2 m 角速度语义跳变」）。
- 若我们要保住小陀螺 + 朝向控制，则**必须改 wrapper**（把 `angular.z = 0.0` 换成「角速度交给内部控制器」或加开关），否则小陀螺在末端被静默关掉。

### C.6 能否直接用：结论

| 维度 | 结论 | 依据 |
|---|---|---|
| 能否编译/加载 | **✅ 可以**：单文件 CMake，依赖 `nav2_core/nav2_util/nav2_costmap_2d/pluginlib/tf2_ros` 全在；`pluginlib_export_plugin_description_file(nav2_core …)` 正确（GACC:39） | GACC:7-45、GACX:1-9 |
| 参数命名空间是否贴合我们 | **✅ 是**：我们只要把 MPPI/wrapper 参数写在 `FollowPath.` 下即可（`NAV:159`/`NAV:242` 的 `controller_plugins: ["FollowPath"]`，R:112） | S:84-176 与 R:128-149 结构相同 |
| 帧正确性 | **❌ 必须改**：`goal_`(map) vs `pose`(odom) 混帧（GAC:104-106 vs `N2/ctrl:611-617`/`R:161`） | §C.2 |
| `angular.z=0` 语义 | **⚠️ 必须做决定**：与我们 `FVT:94` 的 `spin_speed` 语义冲突（§C.5） | §C.5 |
| 与 MPPI 参数并存 | **⚠️ 需一并解决**：`model_dt` 必须 = 1/`controller_frequency`（§A.3），`CostCritic.consider_footprint: true` 在无 footprint 时会抛异常（§A.4） | `N2/mppi/opt:95-113`、`N2/mppi/cc:56-65`、`N2/cmros:408-416` |
| 综合 | **⚠️ 需改造后可用**（不是「直接可用」）：最小改造 = ①`direct_approach` 分支里把 `goal_` 用 TF 转到 `pose.header.frame_id` 后再算 dx/dy（或改为用 `costmap_ros_->getGlobalFrameID()` 做变换）；②`angular.z = 0.0` 加参数开关（默认「不改角速度」）；③`setPlan` 空路径时清 `goal_` 并置无效标志；④配 `spin_speed:=0.0` + `direct_approach_distance ≤ 0.5` | GAC:88-131 |

---

## D. 航点 CSV + `waypoint_editor`

### D.1 格式（贴真实数据）

`third_party/cod_nav_2026/src/cod_bringup/wps/` 下 6 个文件，表头**逐字节相同**（如 `patrol_center.csv:1`）：

```
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
```

真实数据行（原样）：

```
# patrol_center.csv:2-4
0,4.62889,-3.84107,1.62125e-05,0,0,-0.979174,0.203024,
1,2.96424,-4.28705,6.67572e-06,0,0,0.566315,0.824189,
2,3.57432,-3.1591,5.72205e-06,0,0,-0.602533,0.798094,
```
```
# go_gain.csv:2-4
0,-1.59403,-4.04669,-9.53674e-07,0,0,-0.241624,0.97037,
1,2.2723,-6.15121,9.53674e-07,0,0,0.461767,0.887001,
2,3.52213,-4.15178,0,0,0,0.542143,0.840286,
```
```
# go_home.csv:2
0,3.5846,-4.37716,-9.53674e-07,0,0,0.884719,-0.466125,
```

6 个文件与 README 的对应（名字语义）与行数：

| 文件 | 行数（含表头） | 航点数 | README:70 的叙述 |
|---|---|---|---|
| `go_gain.csv` | 4 | 3 | 「去增益点」 |
| `go_home.csv` | 4 | 3 | 「回启动区」 |
| `patrol_behind.csv` | 4 | 3 | 「增益点后半区巡逻」 |
| `patrol_center.csv` | 5 | 4 | 「增益点内四角巡逻」 |
| `patrol_front.csv` | 4 | 3 | 「增益点前半区巡逻」 |
| `preload_patrol.csv` | 4 | 3 | 「前压巡逻」 |

（`README.md:70` 只给中文顺序、**没给文件名** ⇒ 上表的文件↔叙述对应是按名字推断的，见 §F-14。）

### D.2 单位与坐标系

| 项 | 结论 | 证据 |
|---|---|---|
| 列语义 | 0=`id`（整数，**读取端完全忽略**），1-3=`position x/y/z`，4-7=`orientation x/y/z/w`，8=`command`（**自由文本**），行尾有一个多余逗号 | `waypoint_to_nav2.cpp:116-137`（只读 `tokens[1..7]`）、`waypoint_through_nav2.cpp:109-127`、`waypoint_editor_tool.cpp:455-477`（写表头/行）、`:507-538`（读回） |
| 位置单位 | **米**（数值范围 −1.6…7.1，与 12.75 m×8.85 m 的地图一致） | 见 D.1 数据；`MAP:3-4` |
| `pose_z` | 恒 ≈0（±1e-5 浮点残差），即**平面航点** | `patrol_center.csv:2` 1.62e-05 等 |
| 朝向 | 单位四元组（roll/pitch 恒 0，只有 yaw） | `rot_x/rot_y` 全 0，`rot_z/rot_w` 组模长 1 |
| 坐标系 | **`map`**：桥节点参数 `frame_id` 默认 `"map"`（`waypoint_to_nav2.cpp:21`、`waypoint_through_nav2.cpp:20`），并写进 `pose.header.frame_id`（`:123`、`:115`）；编辑器也硬编码 `"map"`（`waypoint_editor_tool.cpp:515`） | 同上 |
| 时间戳 | `pose.header.stamp = rclcpp::Time(0)` ⇒ 让 Nav2 用当前时间 | `waypoint_to_nav2.cpp:124`、`waypoint_through_nav2.cpp:116` |
| `command` 列 | **没有任何节点解释它**：编辑器只把它当标签显示/编辑（`waypoint_editor_tool.cpp:132/208/356-369`），两个桥节点**只读前 8 列** | `waypoint_through_nav2.cpp:109-127`、`waypoint_to_nav2.cpp:116-137` |
| `sample_wp.csv` 里的 `skip:2 / wait:2 / stop:3 / end` | **是示例数据，不是协议**：对 `third_party/cod_nav_2026/src/**`（`*.cpp/*.hpp/*.py`）搜这三个模式只命中一处 —— `waypoint_through_nav2.cpp:96` 的行末注释 `// skip header` ⇒ **无任何实现** | `third_party/cod_nav_2026/src/waypoint_editor/data/sample_wp.csv:2-16`；`USAGE_WITH_NAV2.zh.md:5-10`（「waypoint_editor 保存的 CSV 格式航点不能直接被 Nav2 使用」） |

### D.3 谁消费这些 CSV

**结论：`wps/*.csv` 没有任何程序自动读取。** 证据：

1. `cod_bringup/CMakeLists.txt:25-28` 只是把 `wps` 目录**安装**到 share，让它「随包发布」。
2. 全仓库对 `csv/wps/patrol_*/go_gain/…` 的引用只有：`CMakeLists.txt:26`（安装）、`README.md:19/70`（文档）、`waypoint_editor` 的文档与 GUI 代码 —— **没有一个 ROS 节点**。
3. 实际消费路径是**人手动的**：RViz2 里 `WaypointEditorPanel` 的 **Load WPs** 打开 CSV（`waypoint_editor_tool.cpp:487-538` 的 `handleLoadWaypoints`，由服务 `load_waypoints` 触发，见 `waypoint_editor_panel.cpp` 的按钮绑定），或用桥节点：

| 消费者 | 类型 | 读取方式 | 动作 | 行为 |
|---|---|---|---|---|
| `waypoint_editor`（RViz panel/tool） | RViz 插件（GUI） | `QFileDialog::getOpenFileName` + `QTextStream`，`cols.size() >= 9`（`waypoint_editor_tool.cpp:487-538`） | 只是**编辑/可视化**航点（`/waypoint_markers`、连线、总长度），不发给 Nav2 | `README.md:66-80`、`USAGE_WITH_NAV2.zh.md:5-10` |
| `waypoint_to_nav2` | ROS 节点（可执行） | 参数 `waypoint_file`（`waypoint_to_nav2.cpp:20`）→ 逐行 `std::getline` + 逗号切分，**只跳过第 1 行表头**，要求 `tokens.size() >= 8`（`:117-120`） | 服务 `/start_waypoint_following`（`:24-27`）触发 → `FollowWaypoints` action client（`:30-31`）发**全部航点** | **逐点停靠**（`CMakeLists.txt:107-109` 注释） |
| `waypoint_through_nav2` | ROS 节点（可执行） | 同上（`waypoint_through_nav2.cpp:19/85-137`） | 服务 `/start_waypoint_through`（`:22-25`）触发 → `NavigateThroughPoses` action client（`:27-28`） | **连续穿越、不停留**（`CMakeLists.txt:118-119` 注释；`USAGE_WITH_NAV2.zh.md:19-23`） |

**触发方式（原样）**：

```bash
ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=/abs/path/wps/patrol_center.csv
ros2 service call /start_waypoint_through std_srvs/srv/Trigger
# 或逐点停靠版
ros2 launch waypoint_editor waypoint_to_nav2.launch.py waypoint_file:=/abs/path/wps/go_gain.csv
ros2 service call /start_waypoint_following std_srvs/srv/Trigger
```

（`waypoint_through_nav2.launch.py:11-33`、`waypoint_to_nav2.launch.py:11-33`、`USAGE_WITH_NAV2.zh.md:50-70`。）

**与 `waypoint_follower` 的关系**：`waypoint_to_nav2` 走的是 **`/follow_waypoints` action**，由 `nav2_waypoint_follower/waypoint_follower` 节点提供 —— 该节点**两个 launch 都会起**（`NL:163-172`；我们 `NAV:200-209` 同样），其参数在 `S:460-469`（我们 sim 变体里**没有这一段**，走默认；旧 `nav2_params.yaml:347-355` 有逐字相同的段）。`waypoint_through_nav2` 走 **`/navigate_through_poses`**，由 `bt_navigator` 提供（两边都在跑）。

### D.4 对我们「长目标分段 / 拓扑航点（路线 B/C）」的具体用法建议

1. **格式层：✅ 可直接沿用**。CSV 契约只有「`map` 帧下的 x/y/z + 四元数」，与我们的 `map`（R:243）+ `base_link_fake`（R:244）完全兼容：桥节点发的是 `PoseStamped`，由 `bt_navigator`/`controller_server` 自己按 TF 变换，不涉及 `base_link_fake`。我们可以直接拿他们的 CSV **当模板**（但坐标必须在**我们的图**上重采，见下条）。
2. **坐标必须在我们的地图系里重采**。他们的航点长在他们的 `MAP`（origin `[-1.756,-7.036]`，12.75×8.85 m）里；我们的 `RMUL2026` 图与初始位姿是另一套（`LAUNCH:60`、`:67-80`，注释里明确说过 RMUL2026 曾从「世界系图」换成「cartographer 出生点系图」，两者差 ≈(4.3, 3.35)）⇒ **照抄 CSV 数值 = 把车指到错的地方**。
3. **路线 B/C 用哪个桥**：
   - **连续穿越（推荐给「长目标分段」）** → `waypoint_through_nav2` + `NavigateThroughPoses`。理由：① 它天然就是「一串中间点 + 一个终点」的分段语义；② 我们的 BT 已经能跑（`nav2_compute_path_through_poses_action_bt_node` R:71、`nav2_remove_passed_goals_action_bt_node` R:97、`nav2_navigate_through_poses_action_bt_node` R:95）；③ 中间不停车，适合竞速式通过。
   - **拓扑航点（每个点要停下做动作/等待）** → `waypoint_to_nav2` + `FollowWaypoints`（`nav2_waypoint_follower`）。此时才需要 `waypoint_follower` 参数段。
4. **必须注意的两个坑**：
   - **`waypoint_pause_duration: 200` 的单位是毫秒**（`N2/waw:78-81`）⇒ 只停 **0.2 s**。若我们要「在每个拓扑点停 2 s 等机构动作」，要写 `2000`。我们已有这段（`src/rm_navigation/rm_navigation/params/nav2_params.yaml:355`），但 **sim 变体 R/D/T 里没有** ⇒ 走默认值。
   - **`stop_on_failure: false`**（`S:464`，我们旧文件同）：某个航点失败会**静默跳到下一个**。对「长目标分段」这是危险的（缺一段仍算成功）；建议 `stop_on_failure: true`，并在上层用 `/follow_waypoints` 的 feedback 做断点续跑。
5. **through-poses 的节奏**：官方 through-poses 树是 `RateController hz="0.333"`（`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml:10`）⇒ **3 s 重规划一次**，而 `RemovePassedGoals radius="0.7"`（同文件 `:13`）把 0.7 m 内的点丢掉。我们的全局 costmap 是 `update_frequency 5.0`（R:241）⇒ 3 s 重规划对 5 Hz 图是浪费，若要多点激进行驶，应改自定义 BT 的 `hz`（他们 `BTTP:8` 用 3.0 Hz，可参考但要按我们 0.5 m/s 重调）。
6. **落地清单（不改第三方代码）**：
   - 装 `waypoint_editor` 包（`CMakeLists.txt:107-131` 的两个可执行 + 可选 RViz 插件），或在我们的 `rm_navigation` 里照抄那两个 `.cpp`（各 ~190 行、只依赖 `rclcpp/rclcpp_action/nav2_msgs/geometry_msgs/std_srvs`，`CMakeLists.txt:110-127`）。
   - 在 `R`/`D`/`T` 里补 `waypoint_follower` 段（照抄 `S:460-469`，但 `waypoint_pause_duration` 按需改成毫秒真值，`stop_on_failure: true`）。
   - 用 RViz `WaypointEditorPanel` 在**我们的 RMUL2026 图**上采点 → Save WPs → 得到 `map` 系 CSV。
   - 用 `ros2 launch waypoint_editor waypoint_through_nav2.launch.py waypoint_file:=…` + `ros2 service call /start_waypoint_through std_srvs/srv/Trigger` 触发。

---

## E. 总结论：三张清单

### E.1 ✅ 可直接用（键名与值都成立；抄过去不改也能跑）

| # | 项 | 落到我们哪个文件哪个键 | 风险 | 回退方式 |
|---|---|---|---|---|
| E1-1 | stvl 体素层参数块（`voxel_decay/decay_model/voxel_size/track_unknown_space/observation_persistence/max_obstacle_height/mark_threshold/update_footprint_enabled/combination_method/origin_z/publish_voxel_map/mapping_mode/map_save_duration/transform_tolerance/filter/voxel_min_points/clear_after_reading`） | `R:288-333`（已同构）；要抄的具体值：`max_obstacle_height 2.0`(R:312)、`mark_threshold 0`(R:292)、`combination_method 1`(R:294)、`update_footprint_enabled true`(R:293) | 我们的源是 `/segmentation/obstacle`(R:303) 而非他们的 `/livox/lidar_filtered`(S:221) ⇒ **只抄参数不抄话题** | 我们现有的 `publish_voxel_map: true`(R:296) 与两个源 `livox_mark/livox_clear`(R:300-333) 保留即可；出现幽灵障碍就回退 `min_obstacle_height` 0.05（R:307-311 已写好回退建议） |
| E1-2 | Smac2D 的 9 个有效键 + `smoother.*` 子块 | 新增 `R`/`D`/`T` 的 `planner_server.GridBased`（替换 `R:359` 的 NavfnPlanner）：`tolerance 0.5`、`allow_unknown true`、`downsample_costmap false`、`downsampling_factor 1`、`max_iterations 1000000`、`max_on_approach_iterations 1000`、`max_planning_time`（我们建议 1.0–2.0，不用他们的 4.5）、`cost_travel_multiplier`（建议 1.0–2.0，不用 4.0）、`smoother.{max_iterations,w_smooth,w_data,tolerance,do_refinement}`（S:400-405） | `GACC`/`S` 里那 16 个 Hybrid 键**不能一起抄**（§A.16）；Smac2D 侧 `_smoother->smooth()` 无条件执行（N2/smac2:299-300）⇒ 平滑开销一直存在 | 保留 `NavfnPlanner` 分支（`use_astar: false`，R:361）；计划器按 `planner_plugins` 一键切回 |
| E1-3 | `velocity_smoother` 键名全表 + `cmd_vel` remap 链 | `R:395-408`（键名已一致）；值建议保留我们的（`deadband [0,0,0]`、`max_accel [4,4,6]`） | 抄他们的 `deadband 0.05`(S:478) 会在 0.5 m/s 巡航下吃掉小指令 | 改回 `R:401-406` 原值即可（可 A/B） |
| E1-4 | `map_saver` 段 | `R:345-351`（已逐字相同） | 无 | — |
| E1-5 | MPPI 骨架键名 + 6 个 critic 键名（≠ 值） | 新增 `FollowPath.time_steps/model_dt/batch_size/iteration_count/temperature/gamma/motion_model/vx_std/vy_std/wz_std/prune_distance/transform_tolerance/visualize/reset_period/retry_attempt_limit/regenerate_noises` + `critics` + 各 critic 段（键名照 `S:126-176`，**但要剔掉 `CostCritic.trajectory_point_step`，它是死键**） | **值不能照抄**（§A.3/A.4：20 Hz 与 model_dt 冲突、2.5 m/s、`consider_footprint: true` 抛异常） | 我们可先只接 MPPI + DiffDrive/Omni 默认值，再逐项调 |
| E1-6 | `waypoint_editor` 的两个桥节点 + CSV 格式 | 新增包（或把 `waypoint_to_nav2.cpp`/`waypoint_through_nav2.cpp` 抄进 `rm_navigation`） | 需 `nav2_msgs`/`std_srvs`（已有）；坐标必须在我们图系重采 | 不用时不影响任何现有栈（纯附加节点） |
| E1-7 | `PositionGoalChecker`（插件名，不抄 `path_length_tolerance`） | `R:123` 可改为 `nav2_controller::PositionGoalChecker` + `xy_goal_tolerance`（`R:124`）+ `stateful`（`R:122`），**删掉 `yaw_goal_tolerance`（R:125）** | 忽略朝向 ⇒ 终点朝向不受控（若上层有朝向要求就不能换） | 改回 `SimpleGoalChecker`（`R:123-125`） |
| E1-8 | `nav2_smoother::SavitzkyGolaySmoother` 段（S:407-420） | 新增 `R`/`D`/`T` 的 `smoother_server:` 段，**必须把 `robot_base_frame` 改成 `base_link_fake`** | 我们的 BT 不会调它（默认树无 SmoothPath）⇒ 只是多一个服务，无副作用 | 删掉该段即回默认 |

### E.2 ⚠️ 需改造

| # | 项 | 落到我们哪个文件哪个键 | 风险 | 回退方式 |
|---|---|---|---|---|
| E2-1 | `goal_approach_controller` 本体（帧 bug + `angular.z=0` 语义 + 空路径不复位） | 新包 + `R:128`（`FollowPath.plugin`）；参数 `approach_distance/approach_velocity/direct_approach_distance/direct_approach_kp` | 混帧（GAC:104-106 vs R:161）⇒ 末端朝错误方向平移；`angular.z=0`（GAC:118）在 `spin_speed=5.0` 时**关掉小陀螺**且让 `FVT:91-96` 的线速度旋转退化成空转 | 保留 `R:129`(RPP)/`D:129`(DWB)/`T:130`(TEB) 三个 `FollowPath.plugin` 现值，用 `nav` 启动参数（`LAUNCH:151-156`）一键切回 |
| E2-2 | 接 MPPI 的 `controller_frequency × model_dt` | `R:105`（`controller_frequency: 20.0`）与新增 `FollowPath.model_dt` | 若 `model_dt 0.02` + 20 Hz ⇒ `N2/mppi/opt:111-112` 抛异常，**controller_server 起不来** | 两个选项：`model_dt: 0.05`（保持 20 Hz，`N2/mppi/opt:104-109` 会开 `shift_control_sequence`）或 `controller_frequency: 50.0`（照 S:67） |
| E2-3 | `min_y_velocity_threshold` | `R:107`（`0.5`） | 侧向里程计反馈被抹零，Omni MPPI 的速度反馈是错的 | 改成 `0.001`（照 S:69）；若要回退，RPP 不受影响 |
| E2-4 | `CostCritic.consider_footprint`（`ObstaclesCritic` 有同样检查） | 新增 critic 段 | 我们无 footprint ⇒ `N2/mppi/cc:56-65`（`ObstaclesCritic` 见 `N2/mppi/oc:46-55`）**抛异常** | 两者都设 `false`（照 `S:146` 的 ObstaclesCritic 写法），或先给两张 costmap 加 footprint |
| E2-5 | BT 树（3 Hz 重规划、retries 10、删 Spin/Wait、BackUp 1.0 m/1.0 m/s） | 新增 `behavior_trees/*.xml` + `bt_navigator.default_nav_to_pose_bt_xml`（`R:61-100` 当前无此键） | 我们 `FVT:80-84` 的直通修复让 `Spin` **可用**；照抄他们的树会**删掉我们唯一有效的原地恢复**；`BackUp 1.0 m/s` 在 0.40 半径 + sim 场地会撞 | 不设 `default_nav_to_pose_bt_xml`（回 nav2 内置树，即当前 `R:61-100` 的现状）；或只搬「3 Hz + retries」不搬「删 Spin」 |
| E2-6 | costmap 几何（`robot_radius 0.1` / `footprint ±0.1` / `inflation 0.55→0.75`） | `R:178/252`（`robot_radius: 0.40`）、`R:235/342`（inflation 0.5/0.55） | 抄 0.1 会让规划器贴墙钻 `P2L` 盲区（R:168-177 的整段推理） | 两个方向都可 A/B：半径在 0.35–0.45 之间（R:177 已写回退区间）；inflation 若调到 0.55/0.75，必须 ≥ `robot_radius`（`N2/infl:173-181` 只在小于时报 ERROR，不会自动夹紧） |
| E2-7 | MPPI 速度/权重/时域数值（2.5 m/s、`wz_max 1.5`、`time_steps 80`、`temperature 0.25`、GoalCritic 25、PathFollow 15、PathAlign 15） | 新增 `FollowPath.*` | 这些是给他们 2.5 m/s 车 + 10 m 局部窗 + 实车小陀螺调的；我们 0.5 m/s（R:130）、5 m 窗（R:165-166）、2 cm 分辨率（R:167）⇒ 直接抄会抖/绕圈 | 从 1.1.20 默认值起步（`model_dt 0.05`、`time_steps 56`、`batch_size 1000`、`temperature 0.3`、各 critic 默认），逐项调 |
| E2-8 | `progress_checker`（0.1 m / 999 s ⇒ 等效关闭） | `R:117-118`（0.5 / 10.0） | 抄他们 = 放弃卡死检测；MPPI 早期调试阶段这个检测恰好最有用 | 保留我们现值；调好后再考虑放宽 |
| E2-9 | `behavior_server.global_frame` | `R:381`（`odom`） | 改 `map`（S:442）会让恢复动作受定位跳变影响（我们 `map→odom` 是 amcl/icp 发的高频量） | 保留 `odom`；他们用 `map` 是因为他们的 `map→odom` 是固定的 |
| E2-10 | `waypoint_follower` 段（我们 sim 变体缺） | 新增到 `R`/`D`/`T`（可照 `nav2_params.yaml:347-355`） | `waypoint_pause_duration 200` 是**毫秒**（`N2/waw:78-81`）；`stop_on_failure` 默认 true（`N2/wpf:37`）与他们写的 false 不同 | 删段即回默认 |
| E2-11 | slam_toolbox 参数（`lifelong`、`transform_publish_period 0.0`、`base_frame base_link`） | `SLAMrm:15/17/28` | `transform_publish_period 0.0` = slam_toolbox 不发 `map→odom`（他们靠 `ML:95-116` 静态桥补）；`base_frame base_link` 在我们 TF 里不是 `/scan` 的帧（P2L:9 `target_frame: ""` ⇒ 帧是 `livox_frame`） | 保留 `SLAMrm` 现值（`mapping` / `0.02` / `livox_frame`） |
| E2-12 | `P2L`（`range_min 0.05`、`max_height 0.1`、`target_frame ""`）vs 他们 `ML:66-84`（`target_frame base_link`、`min_height 0.1`、`max_height 1.0`、`range_min 0.5`、`range_max 20.0`、`angle_increment 0.0087`） | `P2L:9/14/15/18/37/38` | 我们的 `max_height 0.1` + `min_height -1.0` 是「只留地面附近之上 10 cm」的策略（配 linefit 去地面）；他们 0.1–1.0 是「哨兵高度带」；`range_min 0.5` 会再造出我们花大力气消掉的 45 cm 盲区（P2L:20-36 的整段考证） | 保留 `P2L` 现值；`range_min` 回退档位已写在 `P2L:29/36`（0.15 / 0.2） |

### E.3 ❌ 不适用

| # | 项 | 出现位置 | 落到我们哪个键（若要抄会写在哪） | 风险/原因 | 处理 |
|---|---|---|---|---|---|
| E3-1 | `publish_critics_stats` | S:65 | `R:102` 段 | 1.1.20 无此键 | 不抄 |
| E3-2 | `vy_min`, `wz_min`, `ax_max/ay_max/az_max/ax_min/ay_min/az_min` | S:103-113 | `FollowPath.*` | 1.1.20 MPPI 只有 `vx_max/vx_min/vy_max/wz_max`（`N2/mppi/opt:75-78`）⇒ 6 个加速度键**全无效**；真正限加速的是 `velocity_smoother.max_accel/max_decel` | 用 `velocity_smoother`（R:405-406）表达加速度约束 |
| E3-2b | `CostCritic.trajectory_point_step` | S:159 | `FollowPath.CostCritic.*` | 1.1.20 `CostCritic` 不读此键（`N2/mppi/cc:26-32`），只有 PathAlign 系 critic 读 | 放到 `PathAlignCritic.trajectory_point_step`（S:173）才有意义 |
| E3-3 | `path_length_tolerance` | S:83 | `R:121-125` 段 | 1.1.20 `PositionGoalChecker` 不声明（`N2/ppgc:44-52`） | 不抄 |
| E3-4 | `bt_navigator.timeout` | S:8 | `R:68` 附近 | 1.1.20 无此键（`N2/bta:57-67`） | 用 `default_server_timeout`（`R:68` 已是 20） |
| E3-5 | planner 的 16 个 Hybrid/Lattice 键 | S:385-399 | `planner_server.GridBased.*` | 只有 Hybrid/Lattice 读（N2/smac2 只读 9 键） | 不抄；要抄就换 `SmacPlannerHybrid` |
| E3-6 | `local_costmap_topic/global_costmap_topic/local_footprint_topic/global_footprint_topic/local_frame` | S:425-428, 441 | `behavior_server.*`（`R:364-388`） | 1.1.20 behaviors 只读 `costmap_topic/footprint_topic/global_frame/robot_base_frame/transform_tolerance`（`N2/beh:34-39`、`N2/timed:119-122`） | 我们 `R:369-370` 的 `costmap_topic/footprint_topic` 才是对的 |
| E3-7 | `pb_nav2_behaviors/BackUpFreeSpace`（含 `robot_radius`/`max_radius`/`service_name`/`free_threshold`/`visualize`） | S:434, 450-454 | `behavior_server.backup.plugin`（`R:376`） | 需编译 `third_party/cod_nav_2026/src/pb_nav2_plugins`；`free_threshold`/`robot_radius` 连它自己都没读（`back_up_free_space.cpp:28-37`） | 保留 `nav2_behaviors/BackUp`（`R:376`） |
| E3-8 | `realsense_source`（D435i） | S:241-262、331-352 | 两张 costmap 的 stvl 源 | 我们无该设备/话题 | 不抄 |
| E3-9 | 他们的定位架构（LIO + 静态 `map→odom`；`SL:67-89`/`ML:95-116`） | launch | `LAUNCH:446-468`（我们的静态桥只在 `localization=''` 时启） | 与我们 `mode × localization` 槽位体系冲突；且 `ML` 里静态桥与 slam_toolbox **同时发 `map→odom`** | 沿用我们的槽位（LAUNCH:303-406） |
| E3-10 | `auto_save_map.launch.py` | `ML:154-158` 引入 | 无（不要引入） | 硬编码 `/home/cod-sentry/dyx_ws/...`（`auto_save_map.launch.py:12-15`）⇒ 在 `-c` 里 `source` 不存在的 setup.bash、`mkdir -p` 到无权路径；且它在启动时立刻注册 10 个定时保存（`:18-24`） | 不抄；我们用 `map_saver_cli` 手动/脚本保存 |
| E3-11 | `robot_state_publisher` 段 | S:456-458 | `R` 顶层 | 两个 launch 都不起该节点 ⇒ 惰性 | 不抄 |
| E3-12 | 他们的 `robot_radius 0.1` + `footprint ±0.1` + 2.5 m/s + 50 Hz 控制环 | S:67/99-105/190-191 | 多处 | 物理前提是 0.2 m 见方哨兵 + 深度相机 + 实车电调 | 只作为「量级参考」，数值必须重标定 |
| E3-13 | `wps/*.csv` 的**坐标值** | `wps/*.csv` | 若直接喂给我们的桥 ⇒ 车去错地方 | 图/原点不同（`MAP:4` origin `[-1.756,-7.036]` vs 我们 `LAUNCH:60/67-80`） | 只借格式，坐标在我们图上重采 |

---

## F. 未能确定的 / 文件内自相矛盾之处

### F.1 自相矛盾（注释 vs 数值、文件之间）

| # | 现象 | 证据 |
|---|---|---|
| F-1 | 注释「预测时域4.8s」，但 `time_steps 80 × model_dt 0.02` = **1.6 s**（4.8 s 只在 M 的 `model_dt 0.06` 下成立） | `S:91-92` vs `M:82-83` |
| F-2 | 注释「放宽到25cm」，数值是 **0.30** | `M:78` |
| F-3 | 注释「与 local_costmap 一致」，但 global 是 **15.0**、local 是 **5.0** | `M:282` vs `M:192`（对比 `S:288` 与 `S:198` 确实都是 5.0，说明 M 是复制后只改了值没改注释） |
| F-4 | 注释声称 `critical_cost: 253.0` 做到「只有 inscribed/lethal 才触发碰撞惩罚」，但该判定是**源码硬编码** `pose_cost >= INSCRIBED_INFLATED_OBSTACLE`（=253）；`critical_cost` 只是**惩罚幅度**（默认 300.0） | `S:155` vs `third_party/nav2/nav2_mppi_controller/src/critics/cost_critic.cpp:164` 与 `:29` |
| F-5 | `S:154` 注释「适当降低（cost_weight 3.8）」——但 3.8 与 1.1.20 默认 3.81 几乎相同，谈不上「降低」 | `S:154` vs `cost_critic.cpp:28` |
| F-6 | 同一句注释「放大关闭距离」被同时贴在 `threshold_to_consider: 1.0`（S:166）与 `0.8`（S:174）上；而 M 把两者都**调大**到 3.0，注释却变成「缩小关闭距离…防止圆周运动」 | `S:166/174` vs `M:160/168` |
| F-7 | M 的 `behavior_server.use_sim_time: true` 与全文其余 `false` 矛盾；且会被 `NL:61-71` 的 RewrittenYaml 用 launch 实参（`SL:123`/`ML:141` 传 `"false"`）覆盖 ⇒ **惰性** | `M:418` vs `M:3/60/366/…`；`SL:123`、`ML:141` |
| F-8 | `non_straight_penalty: 0.0` 的注释写「必须 ≥ 1」（在 Smac2D 上该键本来就无效，所以也没人会报错） | `S:393` |
| F-9 | `NL:68` 把 `root_key=namespace` **注释掉**了，而 `LOC:63` 保留了 `root_key=namespace`（同一仓库两个 launch 的 RewrittenYaml 语义不一致 ⇒ `use_sim_time`/`yaml_filename` 的覆写范围不同） | `NL:65-71` vs `LOC:60-66` |
| F-10 | 地图 yaml 的 `free_thresh: 0.196`（`MAP:7`）与 `map_saver.free_thresh_default: 0.25`（`S:366`）不一致 | `MAP:7` vs `S:366` |
| F-11 | `ML` 同时起 slam_toolbox（`ML:85-94`，其 `SLAMc:30` 明确 `transform_publish_period: 0.0` = 不发 `map→odom`）**和**静态 `map→odom` 广播（`ML:95-116`）——这是「用静态桥补 map→odom」的设计，但 SLAM 建图模式下 map 系本应由 slam_toolbox 接管；二者语义重叠 | `ML:85-116`、`SLAMc:30` |
| F-12 | `S:360` 的 `map_server.yaml_filename` 是硬编码 `/home/cod-sentry/...`，与 `LOC:56-58/76-79` 的「由 launch `map` 参数覆写」相矛盾（照 `LOC` 的说法，yaml 里这行不该有真实路径） | `S:360` vs `LOC:56-58` |
| F-13 | （我们侧独立问题）TEB 的 `footprint_model.circular.radius: 0.2`（`T:150`）与 R/D/T 共用的 two costmap `robot_radius: 0.40`（`R:178/252`）不一致；DWB 的 `max_vel_theta 12.0`（`D:135`）被 `velocity_smoother.max_velocity[2]=3.0`（R:401）截住 | `T:150`、`D:135`、`R:401` |
| F-14 | `README.md:70` 只说「wps 中 csv 文件的航点路径依次为…（6 个中文名）」但**不给文件名**；文件名↔叙述的对应只能按名字推断（§D.1） | `README.md:70` |

### F.2 未能确定的项

| # | 未能确定的内容 | 为什么无法确定 | 影响 |
|---|---|---|---|
| F-15 | `S:198` 注释说「原 0.3 衰减太慢」——0.3 这个历史值在仓库里 **找不到**（无 git 历史可查，`third_party/cod_nav_2026/.git` 存在但本次任务要求只读文件、未做历史考古） | 只读了工作树 | 不影响结论（我们 local 本来就是 5.0，`R:232`） |
| F-16 | 他们实车底盘的真实 `spin_speed`：`FVTc:19` 默认 0.0，且 `third_party/cod_nav_2026/src/fake_vel_transform/launch/fake_vel_transform_launch.py:9-18` 不传参；但 `FVTc:60` 无条件把角速度写成 `spin_speed` ⇒ 若真是 0.0，**他们的底盘在任何导航里都不会自转**（与 `wz_max 1.5`、`Spin` 恢复动作、`max_rotational_vel 1.0` 全部矛盾） | 仓库里没有任何参数文件设置 `spin_speed`；串口下位机行为未知 | 「他们为什么能跑」的解释存疑：可能实车下位机自己在转，也可能他们另有未入库的 launch。**因此 §C.5 的结论只对他们的代码文本成立** |
| F-17 | `get_costmap` 服务的实际命名空间（`/global_costmap/get_costmap` 还是 `/get_costmap`） | `costmap_2d_publisher.cpp:81-82` 用相对名 `"get_costmap"` 创建，具体前缀取决于 `Costmap2DPublisher` 被构造时用的节点；未起栈实测（任务要求只读、不执行） | 只影响 `pb_nav2_behaviors/BackUpFreeSpace` 这一条（我们本来就不用） |
| F-18 | 我们的 `/local_plan` 到底有没有第二发布者（除 nav2 与 TEB 之外，比如仿真插件或外部脚本） | 仓库内 `grep -rn "local_plan" src/` 排除 `teb_local_planner` 子串后无命中；未在运行中的系统上 `ros2 topic info` 实测 | 决定 §C.5「`spin_speed=5.0` 时线速度会不会被旋转」。**但即使有发布者，`angular.z=0` 会顺手停掉小陀螺这一点不变** |
| F-19 | 他们 `M` 里 `direct_approach_*` 被删是否**有意**（vs 忘写） | 无注释、无文档说明；只能从 S/M 的用途差异推断（§B-8/B-9） | 若是有意，则「多点模式不要末端直接驱动」这一条经验值得我们参考；若是忘写，则他们的多点模式末端行为与单点不同却无人知晓 |
| F-20 | 他们的 BT 树 `RecoveryFallback` 里 `BackUp` 的 plugin 解析（BT 里写 `BackUp`，behavior_server 里 `backup` 是 `BackUpFreeSpace`）——BT 节点名是 action 名不是插件名，理论上会路由到 `backup` 插件；未起栈验证 | 未运行 | 只影响我们对「他们恢复行为到底是什么」的描述精度 |
| F-21 | `S:83` 的 `path_length_tolerance` 在他们自己的 nav2 版本上是否真的存在（他们用的是 Humble 吗？） | `README.md:6` 写 `ROS2 Humble`，但 yaml 里同时出现 Humble 无、Jazzy 有的键（`path_length_tolerance`、`publish_critics_stats`、`local_costmap_topic`、`vy_min/ax_*`）。⇒ 他们**实际编译运行的 nav2 版本很可能比本机 1.1.20 新**（或混用了 main 分支源码），但仓库里**没有任何版本锁定文件**（无 `ros2.repos`/`COLCON_IGNORE`/依赖清单） | 这是本文最重要的未确定项：**他们能跑的 yaml 我们不一定能跑**。建议在任何移植前先 `ros2 param list /controller_server` 在两边对齐键名 |

### F.3 交叉验证建议（不改任何文件，仅运行期确认）

1. 起一次他们的栈（在**他们**的工作区，非本仓库）后执行 `ros2 param list /controller_server | grep -E "vy_min|ax_max|publish_critics_stats"` —— 若列出 ⇒ 他们的 nav2 版本 ≥ Jazzy 语义，本文 §A.16 的死键表就要整体重算。
2. 在本仓库起栈后 `ros2 topic info /local_plan -v` 确认 §F-18。
3. `ros2 param get /bt_navigator odom_topic`（我们侧预期 `/Odometry`，与实际 `/odom` 不符，见 §A.10）。
