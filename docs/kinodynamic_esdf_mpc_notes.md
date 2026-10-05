# `865749/Kinodynamic_esdf_mpc` 调研笔记（对照我们 RM 哨兵 bench）

> **调研日期**：2026-10-05（只读调研：`raw.githubusercontent.com` + GitHub REST API + `web_search`）。
> **仓库快照**：branch `master`，HEAD = `8e6561ea`（2026-05-06T11:47:00Z），34★ / 3 fork，创建于 2025-01-12。
> **引用约定**：
> - `R:<path>:L<n>` = <https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/><path>#L<n>（master 上逐行核对）。
> - 本机文件用相对工作区路径的 Markdown 链接（工作区 = `/home/weicheng/HzMi_rmsimulation`）。
> - 本机环境检查命令：`ls /opt/ros/humble/share/*`、`dpkg -l`、`ls /usr/include/...`、`ros2 pkg list`。
> - 【事实】= 仓库/本机可直接核对的原文；【推论】= 我基于事实的判断，**请按打折使用**。

---

## 0. 一句话结论

【事实+推论】这是 **RoboMaster 竞赛（RMUC/RMUL）的一台全向/麦轮底盘的自主导航代码**，本质是 **HKUST Fast-Planner 系（GPL-3.0）的 ROS 2 Humble 移植 + 2D 化**：`plan_env`（**2D** 增量 EDT/ESDF，吃 `/scan`）+ `path_searching`（kinodynamic A*）+ `bspline_opt`（NLopt 轨迹优化）+ `planner_manager`（**ACADO 导出 RTI 求解器**，30 步 @0.1 s 的一阶全向运动学**跟踪型 MPC**，控制量 `vx, vy, w`，控制箱约束 ±0.5/±0.5/±0.1）。
**关键反直觉点：它的 MPC 里没有障碍项、没有 ESDF**——避障全在 kino A* + B-spline 里（`ACADO_NOD = 0`，无在线数据，R:planner_manager/omni_model/acado_common.h:L67）。所以"ESDF MPC"这个名字是**松的**：ESDF 供规划用，MPC 只做轨迹跟踪。

---

## 1. 身份 / 形态 / 许可证

| 项 | 值 | 出处 |
|---|---|---|
| 作者 | user `865749`（GitHub id 128976783，创建 2023-03-26，**仅 1 个公开仓库**，无 name/bio/org） | <https://api.github.com/users/865749> |
| 包维护者 | `z <1141942432@qq.com>`（五个自研包 package.xml 均此署名） | R:planner_manager/package.xml:L7；R:plan_env/package.xml:L7 |
| 上游/血统 | 【推论】Fast-Planner（HKUST-Aerial-Robotics，**GPL-3.0**，3418★）的 ROS 2 + 2D 化改造：命名空间 `fast_planner`、类名 `SDFMap`/`EDTEnvironment`/`KinodynamicAstar`/`BsplineOptimizer`/`kino_replan_fsm` 全同源；`fillESDF` 即 Felzenszwalb–Huttenlocher 一维平方距离变换（R:plan_env/src/sdf_map.cpp:L352-L390） | R:plan_env/include/plan_env/edt_environment.hpp:L11；<https://api.github.com/search/repositories?q=Fast-Planner+in:name> |
| 首次提交 | `c1a76395`（作者 **Ouuu**，2025-01-12T12:47:48Z "update"）；第二个提交 `fc46d37d`（865749，"acado mpc"）；共 **13** 个提交，之后全是 README 编辑 | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/commits?per_page=30> |
| 分支 | 只有 `master`（**无 `main`**） | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/branches> |
| 是什么 | **比赛代码**（不是论文代码）：地图/PCD 全是赛场资产 `RMUC2025.pgm`/`RMUC.pcd`/`RMUL.pcd`；串口驱动直连 RoboMaster C 板发 `vx,vy,vz`。 | R:planner_manager/map/RMUC2025.yaml；R:planner_manager/PCD/RMUC.pcd；R:rm_serial_driver_only_speed/README.md:L34-L67 |
| 许可证 | **无**：仓库根只有一个 `README.md` + `ACODO_ws.zip`（无 LICENSE/COPYING/.gitignore）；API `license = null`；五个自研包全部 `<license>TODO: License declaration</license>`。vendored 包各带自己的许可（fast_lio `LICENSE` 正文是 **GPL-2.0** 而 package.xml 写 `BSD`，linefit 是 BSD-3-Clause） | R:planner_manager/package.xml:L8；R:fast_lio/LICENSE:L1-L2；R:fast_lio/package.xml:L15；R:linefit_ground_segementation_ros2/LICENSE:L1 |
| ROS 版本 | **ROS 2 Humble / Ubuntu 22.04**（README 原文），全部 `ament_cmake` | R:README.md:L1-L2 |
| 仿真器 | **仓库内没有任何仿真包**；README 指向外部 Gazebo 仿真 `LihanChen2004/rmul24_gazebo_simulator`（该仓库已迁移为 `SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator`，Apache-2.0，2026-09-28 仍在更新） | R:README.md:L15-L16；<https://api.github.com/repos/LihanChen2004/rmul24_gazebo_simulator>（301→ 上述仓库） |
| 目标平台 | **全向/麦轮**（omni）底盘：MPC 控制量 = 车体系 `vx, vy` + `w`，状态 = `x,y,yaw`；`/cmd_vel` 用 `Twist.linear.x/linear.y/angular.z`；`CarType 0 = 圆形底盘`（半径 `Car_L 0.27 m`）；串口下行包是 `vx,vy,vz` | R:planner_manager/src/mpc.cpp:L109-L113；R:planner_manager/src/kino_replan_fsm.cpp:L346-L348；R:planner_manager/config/planner_manager.yaml:L16-L19 |
| 感知链 | Livox **MID-360**（`MID360_config.json`）→ `fast_lio`（另有 `point_lio`）→ `linefit_ground_segmentation`（去地面）→ `pointcloud_to_laserscan`（`/scan`）→ 2D ESDF | R:planner_manager/launch/plan.launch.py:L50-L130 |
| 定位链 | 外部里程计 `/Odometry`（默认参数）＋ 可选先验图 ICP 配准（`icp_registration`，底图 `RMUC.pcd`，`map_frame_id: world`） | R:planner_manager/config/planner_manager.yaml:L4；R:icp_registration/config/icp.yaml:L6-L9 |
| 体积 | `size = 162560 KB`（≈159 MB，含 50 MB `RMUL.pcd`、18 MB `RMUC.pcd`、52 MB gif、13 MB PDF）→ 克隆建议 `--depth 1` | <https://api.github.com/repos/865749/Kinodynamic_esdf_mpc> |

---

## 2. ESDF 部分（**是 2D，不是 3D**）

| 项 | 结论 | 出处 |
|---|---|---|
| 库/算法 | **自研/移植的 2D 增量 EDT**，**不是** voxblox / nvblox / FIESTA。`fillESDF` 是两遍（x 后 y）一维平方距离变换（Felzenszwalb–Huttenlocher），再开方乘分辨率 | R:plan_env/src/sdf_map.cpp:L352-L419 |
| 维度 | **2D 栅格**：全 `Eigen::Vector2d`，索引 `id(0)*num_y + id(1)`，入口叫 `updateESDF2d()`；无 z、无体素 | R:plan_env/include/plan_env/sdf_map.hpp:L37-L94,L164,L187-L190 |
| 符号距离 | 正距离 + `occupancy_buffer_neg` 负距离（障碍内部），合并成 `distance_buffer_all_`（内部为负）：`all = pos + (-neg + resolution)` | R:plan_env/src/sdf_map.cpp:L421-L473 |
| 输入 | `sensor_msgs/LaserScan`（参数 `sdf_map.laser_topic`，实配 `/scan`）；回调里 `laser_geometry::projectLaser` → tf2 变换到 `odom` → 按 `obstacles_inflation` 膨胀写占据 | R:planner_manager/config/planner_manager.yaml:L34；R:plan_env/src/sdf_map.cpp:L236-L313 |
| 分辨率/尺寸 | **`resolusion_ = 0.01 m`**（参数名拼错但代码就用这个键）、`map_size 30×30 m` → 3000×3000 = 9e6 格；局部更新窗口 **3.0×3.0 m** | R:planner_manager/config/planner_manager.yaml:L23-L25,L36-L37；R:plan_env/src/sdf_map.cpp:L42-L65 |
| 更新率 | **20 Hz**：`create_wall_timer(0.05s, updateESDFCallback)`（另有 20 Hz 可视化定时器）；只在 `esdf_need_update_` 时重算，且**只算局部窗口** `local_bound_min_..local_bound_max_` | R:plan_env/src/sdf_map.cpp:L61,L76,L331-L351,L393-L394 |
| 膨胀 | `obstacles_inflation = 0.0001 m`（实配）→ `inf_step = ceil(0.0001/0.01) = 1` 格 ≈ **1 cm**，等于几乎不膨胀（安全间隙靠 `Car_radius 0.27 m` 在碰撞检查里补） | R:planner_manager/config/planner_manager.yaml:L40；R:plan_env/src/sdf_map.cpp:L271,L292-L313 |
| 距离滤除 | 丢弃 `‖p − laser_pos‖ < 0.2 m` 的点（避免扫到自身） | R:plan_env/src/sdf_map.cpp:L286-L289 |
| 梯度用法 | `getSurroundPts` 取 4 邻格 → **双线性插值**，梯度解析给出并乘 `1/resolution`（`grad[1]=(v1-v0)/res`, `grad[0]=…/res`）；`evaluateEDTWithGrad` 被 B-spline 优化器的距离代价调用；`evaluateCoarseEDT`（最近格距离）被 kino A* 与 20 Hz 安全检查调用 | R:plan_env/src/edt_environment.cpp:L71-L114；R:bspline_opt/src/bspline_optimizer.cpp:L392；R:planner_manager/src/kino_replan_fsm.cpp:L26,L36 |
| 先验图模式 | 支持 `use_global_map` + `global_map_path`（ROS 地图 YAML+PGM，实配 `RMUC2025.yaml`）作为全局占据层，可多图切换 `setLocalMap(num)`；也发 `nav_msgs/OccupancyGrid` 到 `map_topic` | R:planner_manager/config/planner_manager.yaml:L21,L29-L32；R:plan_env/src/sdf_map.cpp:L9-L39,L139-L197；R:plan_env/include/plan_env/sdf_map.hpp:L125-L130 |
| **MPC 是否用 ESDF** | **不用**。ACADO 模型无在线数据（`ACADO_NOD 0`）、无 ESDF 参数；QP 变量数 `ACADO_QP_NV 90 = NU×N` 只有控制量 → 障碍/间隙完全不进 MPC | R:planner_manager/omni_model/acado_common.h:L67,L83 |
| 实测 ESDF 耗时 | 有打印代码（`show_esdf_time`），但**实配 false，仓库未给任何数值** | R:planner_manager/config/planner_manager.yaml:L35；R:plan_env/src/sdf_map.cpp:L346-L348 |

【推论】内存足迹：3000×3000 格 ×（1 char 占据 ×2 + 1 char neg + 4 个 double 缓冲）≈ **300 MB 常驻**；但每周期 EDT 只扫 601×601 的局部窗（3 m/0.01 m），20 Hz 在 28 核 i7-14700HX 上属轻载。

---

## 3. MPC 部分

**求解器**：ACADO Toolkit 导出的 **RTI（real-time iteration）** 求解器，内嵌 **qpOASES3** 解 QP；仓库直接链接**预编译静态库**（不装 ACADO 也能编：`find_package(ACADO)` 被注释掉）。

| 项 | 值 | 出处 |
|---|---|---|
| 求解器/框架 | ACADO 导出码 + qpOASES3（`ACADO_QP_SOLVER ACADO_QPOASES`、`ACADO_QPOASES3 1`）；静态库 `planner_manager/omni_model/libacado_exported_rti.a`（240 958 B） | R:planner_manager/omni_model/acado_common.h:L37-L48；R:planner_manager/CMakeLists.txt:L23-L32 |
| RTI 迭代 | `NUM_STEPS 10`（每次 solve 跑 10 次 feedback+preparation） | R:planner_manager/include/planner_manager/mpc.hpp:L21；R:planner_manager/src/mpc.cpp:L94-L99 |
| 状态维 | `ACADO_NX = 3`：`x, y, yaw` | R:planner_manager/omni_model/acado_common.h:L73 |
| 控制维 | `ACADO_NU = 3`：`vx, vy, w`（车体系速度 + 偏航角速度） | R:planner_manager/omni_model/acado_common.h:L71；R:planner_manager/src/mpc.cpp:L109-L113 |
| 时域 | `ACADO_N = 30`，`OCP(0.0, 3.0, 30)` → **Ts = 0.1 s，时域 3.0 s**（代码里 `#define Ts 0.1`） | R:planner_manager/omni_model/acado_common.h:L65；R:planner_manager/include/planner_manager/mpc.hpp:L24 |
| **动力学模型** | **一阶（速度级）全向运动学**，不是二阶动力学：`ẋ = vx·cosψ − vy·sinψ`，`ẏ = vx·sinψ + vy·cosψ`，`ψ̇ = w`。生成器原文见 `ACODO_ws.zip → ACODO_ws/src/acado_test/src/main.cpp`；FSM 里同一模型的离散递推 `update_states()` 也标了 `// based on kinematic model` | R:ACODO_ws.zip（内 `ACODO_ws/src/acado_test/src/main.cpp`）；R:planner_manager/src/kino_replan_fsm.cpp:L195-L209 |
| 积分/离散 | `MULTIPLE_SHOOTING` + `INT_RK45` + `NUM_INTEGRATOR_STEPS 30` + `GAUSS_NEWTON` + `FULL_CONDENSING` + `HOTSTART_QP YES` + `LEVENBERG_MARQUARDT 1e2` | 同上 main.cpp |
| 代价项 | `minimizeLSQ(W, rf)`，`rf = [x y ψ vx vy w]`（NY=6）；`minimizeLSQEndTerm(WN, rfN)`，`rfN = [x y ψ]`（NYN=3）→ **位姿+速度跟踪 + 终端位姿**。对角权重运行时可配：`weight_p 9.0`、`weight_yaw 5.0`、`weight_v 0.1`、`weight_w 0.1`（代码默认 10/10/0.3/0.3） | R:planner_manager/omni_model/acado_common.h:L79-L81；R:planner_manager/config/planner_manager.yaml:L78-L82；R:planner_manager/include/planner_manager/mpc.hpp:L42-L45；R:planner_manager/src/mpc.cpp:L9-L18 |
| **约束** | **只有控制箱约束（硬）**：`-0.5 ≤ vx ≤ 0.5`，`-0.5 ≤ vy ≤ 0.5`，`-0.1 ≤ w ≤ 0.1`。已烘焙进生成码：`acado_solver.c` 的 `acadoVariables.lbValues[...] = -5e-1 / -1e-1`。**无状态约束、无加速度/加加速度约束、无软约束/slack、无障碍项** | R:ACODO_ws.zip（main.cpp 原文）；R:planner_manager/omni_model/acado_solver.c:L1212-L1223 |
| 速度/加速度限制在哪 | 速度/加速度限制在 **B-spline 层**：`pos.setPhysicalLimits(max_vel, max_acc)` + `checkFeasibility` + 最多 3 次 `reallocateTime`；`max_vel = 1.5`、`max_acc = 0.8`、`max_jerk = 1.0`（`max_jerk_` 只被读取、**未见用于任何硬约束**，平滑性由 λ1 jerk 代价承担） | R:planner_manager/src/kino_replan_fsm.cpp:L100-L110；R:planner_manager/config/planner_manager.yaml:L6-L8,L55-L56；R:bspline_opt/include/bspline_opt/bspline_optimizer.hpp:L45 |
| 求解频率 | **控制/MPC 100 Hz**（`manager.control_cmd_frequency: 100` → `publish_control_cmd` 定时器）；安全检查 20 Hz；重规划检查 20 Hz | R:planner_manager/config/planner_manager.yaml:L12-L14；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L81-L100 |
| 求解耗时 | **仓库未给数值**：计时语句存在但被注释（`// std::cout << "cost_time" …`）；只有搜索/优化耗时打印 | R:planner_manager/src/kino_replan_fsm.cpp:L83,L112,L335 |
| 参考轨迹来源 | 自己的 `position_traj_`（位置 B-spline）与 `yaw_traj_`：每 Ts=0.1 s 采 30 个点，输出 6 元组 `[x y ψ vx vy w]` 共 180 个参考量 | R:planner_manager/src/kino_replan_fsm.cpp:L259-L324 |
| 热启动 | 用**同一运动学模型**把上一周期控制序列前推 N+1 步，作为 ACADO 状态初值（`motion_prediction`） | R:planner_manager/src/kino_replan_fsm.cpp:L210-L245 |

【事实·矛盾点】**规划 1.5 m/s，MPC 上限 0.5 m/s**：`search.max_vel / optimization.max_vel = 1.5`，而生成码把控制硬限在 ±0.5（合速度上限 0.707 m/s）。⇒ 期望速度永远进不了可行域，MPC 必然长期饱和在约束边界。

【事实·隐患】`motion_prediction()` 对只有 3 个元素的 state 访问 `cur_state[3]`（偏航溢出归一化），是**越界读**；`calculate_ref_states()` 是死代码且有同样问题。 | R:planner_manager/src/kino_replan_fsm.cpp:L178-L194,L224-L231

---

## 4. 实现事实（节点 / 话题 / 配置 / 依赖 / 分工）

**入口**：单节点组件 `planner_manager_node`（插件类 `fast_planner::kino_replan_fsm`，节点名 `planner_manager`）。 | R:planner_manager/CMakeLists.txt:L35-L38；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L44；R:planner_manager/src/kino_replan_fsm.cpp:L488

| 话题/帧 | 名称 | 出处 |
|---|---|---|
| 订阅 | `/Odometry`（`nav_msgs/Odometry`）、`/goal_pose`（`geometry_msgs/PoseStamped`） | R:planner_manager/config/planner_manager.yaml:L4-L5；R:.../kino_replan_fsm.hpp:L56-L62 |
| 发布 | `/cmd_vel`（`geometry_msgs/Twist`，`linear.x`/`linear.y`/`angular.z`）、`robot_path`、`predict_path`（`nav_msgs/Path`）、`visited_node`（`PointCloud2`）、`map_topic`（`OccupancyGrid`） | R:.../kino_replan_fsm.hpp:L84-L90；R:planner_manager/src/kino_replan_fsm.cpp:L346-L355；R:plan_env/src/sdf_map.cpp:L6 |
| 帧 | `sdf_map.frame_id = /odom`；path 的 `frame_id = "/odom"`（**带前导斜杠**，不符合 ROS 2 惯例） | R:planner_manager/config/planner_manager.yaml:L39；R:planner_manager/src/kino_replan_fsm.cpp:L135,L359 |

**配置实值**（[planner_manager.yaml](https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/planner_manager/config/planner_manager.yaml)）：
`max_vel_ 1.5`、`max_acc_ 0.8`、`max_jerk_ 1.0`、`dynamic_environment 0`、`control_points_distance 0.5`、`control_cmd_frequency 100`、`safety_cheak_frequency 20`、`plan_cheak_frequency 20`、`CarType 0`、`Car_L 0.27`、`Car_W 0.40`（L6-L19）；
`search: max_tau 0.3 / init_max_tau 0.4 / lambda_heu 5.0 / w_time 5.0 / horizon 100.0 / resolution_astar 0.01 / time_resolution 0.8 / allocate_num 100000 / check_num 30`（L41-L52）；
`optimization: lambda1 20 / lambda2 8 / lambda3 1e-5 / lambda4 0.05 / lambda7 100 / dist0 0.6 / max_iteration_num2..4 300/200/200 / algorithm1 11 / algorithm2 15 / order 3`（L54-L76）；`mpc: weight_p 9.0 / weight_yaw 5.0 / weight_v 0.1 / weight_w 0.1`（L78-L82）。
【事实】代码读的是 `manager.plan_cheak`（R:.../kino_replan_fsm.hpp:L97），配置里写的是 `manager.plan_cheak_frequency`（L14）→ **键名不匹配**，实际取默认值 20（恰好同值）。
【推论】`algorithm1/2 = 11/15` 对应 NLopt `LD_LBFGS` / `LD_TNEWTON`（NLopt `nlopt_algorithm` 枚举次序），即"二次代价用 L-BFGS、一般代价用 TNewton"，与 `nlopt::opt(nlopt::algorithm(...), …)` 用法一致（R:bspline_opt/src/bspline_optimizer.cpp:L176）。

**规划–控制分工**（同一个 FSM 内串起来，无 Nav2 参与）：
1. `plan()`：**kinodynamic A***（`path_searching`，状态 4 维 `(x,y,vx,vy)`、控制 = 加速度 2 维、ESDF 碰撞、四阶多项式时间启发式，`max_vel 1.5 / max_acc 0.8 / w_time 5 / horizon 100 m`） | R:path_searching/include/path_searching/kinodynamic_astar.hpp:L26-L29,L143-L171；R:path_searching/src/kinodynamic_astar.cpp:L36-L44,L159-L243,L341-L412
2. 采样 → 3 次 B-spline 控制点（`ts = ctrl_pt_dist/max_vel = 0.333 s`）→ **NLopt** 优化 `NORMAL_PHASE = SMOOTHNESS|DISTANCE|FEASIBILITY`（λ1/λ2/λ3，距离项用 `evaluateEDTWithGrad`） | R:planner_manager/src/kino_replan_fsm.cpp:L84-L99；R:bspline_opt/src/bspline_optimizer.cpp:L13-L15,L290-L315,L388-L392
3. `planYaw()`：第二次 B-spline 只优化航向（`SMOOTHNESS|WAYPOINTS`，`dt_yaw≈0.3 s`，前视 2.0 s） | R:planner_manager/src/kino_replan_fsm.cpp:L406-L484
4. 100 Hz `publish_control_cmd()`：采参考 → ACADO RTI MPC → 发 `/cmd_vel` 第一拍 | R:planner_manager/src/kino_replan_fsm.cpp:L246-L375
5. 20 Hz 安全检查：目标净空 <0.3 m 时按 `dr=0.27 m / dθ=30°` 螺旋找更空的目标点（**自动挪目标**）；沿 B-spline 以 0.02 s 步长查碰撞（半径 3 m 内） | R:.../kino_replan_fsm.hpp:L243-L327；R:planner_manager/src/kino_replan_fsm.cpp:L4-L49
6. 20 Hz 重规划：跟踪误差 > 1.0 m 或轨迹碰撞 → `GEN_NEW_TRAJ`/`REPLAN_TRAJ`（FSM：`INIT→WAIT_TARGET→GEN_NEW_TRAJ→EXEC_TRAJ`） | R:.../kino_replan_fsm.hpp:L344-L500

**依赖清单**：

| 包 | 依赖（原文） | 出处 |
|---|---|---|
| `plan_env` | rclcpp, visualization_msgs, std_msgs, **PCL**, eigen, message_filters, nav_msgs, pcl_conversions, sensor_msgs, geometry_msgs, laser_geometry, tf2_geometry_msgs, **OpenCV**, tf2_ros, tf2_sensor_msgs | R:plan_env/package.xml:L12-L26 |
| `path_searching` | plan_env, eigen, boost, rclcpp, laser_geometry, sensor_msgs, nav_msgs | R:path_searching/package.xml:L12-L18 |
| `bspline_opt` | plan_env, **NLopt**, eigen, bspline（`find_package(NLopt REQUIRED)`） | R:bspline_opt/package.xml:L12-L17；R:bspline_opt/CMakeLists.txt:L9 |
| `planner_manager` | rclcpp, rclcpp_components, std_msgs, geometry_msgs, plan_env, path_searching, bspline, bspline_opt, **cppad**（声明了但代码不用）, tf2_ros, tf2_geometry_msgs, **rmoss_interfaces**, laser_geometry + 预编译 `libacado_exported_rti.a` | R:planner_manager/package.xml:L12-L25；R:planner_manager/CMakeLists.txt:L30 |
| README 要求的环境 | ACADOtoolkit、nlopt、glog、**osqp + osqp-eigen**、`pip install xmacro`、`rosdep install -r --from-paths src` | R:README.md:L3-L13 |

【事实】`rmoss_interfaces` 在 git tree 里是 **submodule gitlink（`type: commit`），而仓库没有 `.gitmodules`**；`kino_replan_fsm.hpp:563` 真的用了 `rmoss_interfaces::msg::GimbalCmd`，`package.xml:23` 也依赖它 → **直接 clone 编不过**。
【事实】硬编码绝对路径：`mpc.hpp` 用 `../../../src/planner_manager/omni_model/...` 相对包含（包必须放在 `<ws>/src/planner_manager`）；配置/launch 里散落 `/home/z/Ouuu_fast_planner_2D/...`、`/home/livox/livox_test.lvx`。 | R:planner_manager/include/planner_manager/mpc.hpp:L8-L9；R:planner_manager/config/planner_manager.yaml:L31；R:planner_manager/launch/plan.launch.py:L13,L44,L46
【事实】仓库里也带 `.o` 目标文件、`ACODO_ws.zip`（2.2 MB，含 2024-12~2025-01 的 colcon 构建日志、`getting_started_export/` 与 `omni_model1.zip`）；`planner_manager/CMakeLists.txt:18` 还 include 了仓库里不存在的 `getting_started_export`。
【事实】噪声/死代码：`minDistToAllBox` 每次调用 `std::cout << "asd"`；动态障碍预测存在但 `dynamic_environment: 0` 关闭，`obj_predictor.cpp` 仅 874 B。 | R:plan_env/src/edt_environment.cpp:L44,L49；R:planner_manager/config/planner_manager.yaml:L9；R:plan_env/src/obj_predictor.cpp

---

## 5. 是否有配套论文 / 博客 / 视频 / 学位论文

【事实】**没找到**。依据：① 仓库根无 `docs/`、无 arXiv/DOI 链接、无 citation 文件（tree 548 项逐项看过，顶层只有 `README.md` + `ACODO_ws.zip` + 14 个包目录）；② README 全文 580 B，只写环境、"需要装 ACADOtoolkit/nlopt/glog/osqp"、仿真链接；③ `web_search`（用户 `865749` / 仓库名 / "kinodynamic ESDF MPC"）只返回第三方聚合页 <https://relatedrepos.com/gh/865749/Kinodynamic_esdf_mpc> 与不相关的 RM/无人机论文，**没有作者本人的论文/博客/视频/Thesis**。
【推论】因此**没有任何可引用的性能数字**（求解耗时、最高速度、障碍密度）：这既是"无据可依"，也意味着不存在"论文声称 vs 代码实现"的落差问题——一切都以代码为准。

---

## 6. 它在"导航策略设计轴"上的位置（对齐我们自己的分类）

轴名沿用本工作区 [nav_strategy_taxonomy.md](docs/nav_strategy_taxonomy.md#L23)（9 轴）与 [algorithm_axes.md](docs/algorithm_axes.md#L43)（6 轴）：

| 轴 | 本仓库所属族 | 依据 |
|---|---|---|
| **轴 1 世界表示** | **1.4 ESDF/TSDF —— 但只是 2D 版本**（局部增量 EDT），叠加 **1.1 2D 占据栅格**（可选先验 PGM，`RMUC2025.yaml`）与 **1.12 先验 PCD 子图 + ICP**（`RMUC.pcd`）。**完全没有 3D 体素/octomap/voxblox** | [nav_strategy_taxonomy.md:83](docs/nav_strategy_taxonomy.md#L83)；R:planner_manager/config/planner_manager.yaml:L29-L32；R:icp_registration/config/icp.yaml:L6 |
| **轴 2 定位** | 不自己做：吃外部 `nav_msgs/Odometry`（实车=d；仿真）；可选 `icp_registration` 先验图 ICP 重定位（`map_frame_id: world` / `odom_frame_id: odom`），仓库内 vendored FAST-LIO / Point-LIO | R:planner_manager/config/planner_manager.yaml:L4；R:icp_registration/config/icp.yaml:L7-L8 |
| **轴 3 全局规划** | **3.2 的血亲但更"动力学"**：**kinodynamic A\***（连续状态 + 数值积分 + ESDF 碰撞 + 多项式时间启发式），本仓库地图里没有"精确对应族"——建议在轴 3 补一族 **"kinodynamic search / 动力学约束图搜索"**（介于 3.2 state lattice 与 3.10 MPC 式全局规划之间）；其后接 **3.5 轨迹优化**（B-spline + NLopt） | [nav_strategy_taxonomy.md:202](docs/nav_strategy_taxonomy.md#L202)（3.2）、[:220](docs/nav_strategy_taxonomy.md#L220)（3.5）、[:250](docs/nav_strategy_taxonomy.md#L250)（3.10）；R:path_searching/src/kinodynamic_astar.cpp:L49-L243 |
| **轴 4 局部控制** | **4.7 MPC**（ACADO RTI + qpOASES，梯度式、显式约束、热启动）**＋** 4.5 式"局部轨迹优化"（NLopt B-spline 避障/平滑，**避障在这里**）；运动学属 **4.10 全向/麦轮（Omni）** | [nav_strategy_taxonomy.md:294](docs/nav_strategy_taxonomy.md#L294)（4.7）、R:planner_manager/src/mpc.cpp:L61-L117；R:bspline_opt/src/bspline_optimizer.cpp:L290-L315 |
| **轴 5 任务指定** | **5.x 单目标位姿**：收 `/goal_pose`，无巡逻/覆盖/多点排序；目标被占时**自动螺旋外挪目标**（0.27 m 步长 / 30°） | R:.../kino_replan_fsm.hpp:L254-L297 |
| **轴 6 不确定性与安全** | 无不确定度建模、无 CBF/可达集/概率保证；安全 = **几何间隙（`Car_radius 0.27`）＋ 20 Hz 碰撞检查 ＋ 重规划 ＋ 目标外挪**（"检查-重规划兜底"族） | R:planner_manager/src/kino_replan_fsm.cpp:L4-L49；R:.../kino_replan_fsm.hpp:L243-L327 |
| **轴 7 学习** | **无**任何学习成分（tree 里无 torch/tf/onnx，无训练脚本） | 548 项文件清单（<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/git/trees/master?recursive=1>） |
| **维度轴 D（2D→2.5D→3D）** | **明确 = 2D**，甚至比我们"更平"：3D 点云先用**高度带**（`min_height -1.0`／`max_height 0.5`）压成 LaserScan，再建 2D ESDF。**没有任何 2.5D 可通行性/高程层** | [algorithm_axes.md:99](docs/algorithm_axes.md#L99)；R:planner_manager/launch/plan.launch.py:L94-L101 |

---

## 7. 与 MPPI 的本质差异（本仓库 MPC vs 我们 bench 的 Nav2 MPPI）

我们的 MPPI 实配（[nav2_params_sim_mppi.yaml](src/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml)）：`controller_frequency 30 Hz`[:107]、`motion_model "Omni"`[:143]、`time_steps 60`[:145]、`batch_size 1500`[:151]、6 个 critic[:170-171]、局部代价图 `resolution 0.02`[:235]、`robot_radius 0.22`[:248]、`inflation_radius 0.5`[:305]、STVL `voxel_size 0.05`[:362]。

| 维度 | 本仓库 MPC（梯度/RTI-QP） | 我们 MPPI（采样） | 谁更强 |
|---|---|---|---|
| 优化机制 | 显式 OCP：`minimizeLSQ` + Gauss-Newton，10 次 RTI 迭代，qpOASES 热启动 QP | 1500×60 = **9e4 采样/周期 @30 Hz**，代价指数加权，无梯度 | 各有：MPC 解**平滑、可预测**；MPPI 免梯度、能吞非凸/不可微代价 |
| 动力学模型 | 显式一阶全向（`ẋ=vx cosψ−vy sinψ`…），状态 3 维、控制 3 维，Ts 0.1 s × 30 步 = **3.0 s 时域** | Nav2 内置 Omni 运动学（同样一阶全向），时域 = 60×(1/30) = **2.0 s** | 平手（同一族运动学）；本仓库可显式约束、MPPI 不能 |
| 速度/加速度约束 | **硬约束**但只有控制箱 ±0.5 / ±0.5 / ±0.1 rad/s（**且比规划器 1.5 m/s 小 3 倍**）；加速度限制在 B-spline 层用可行性代价软处理 | 无硬约束：`ConstraintCritic`/`ObstaclesCritic` 是**软惩罚**，`vx_max` 等只在参数里"约定" | **MPC（在它自己的包络内）**；但本仓库包络被写死，实车速度上限反而低 |
| 避障 | **MPC 内部没有障碍项**（`ACADO_NOD=0`，无 ESDF 参数）；避障靠 **20 Hz 重规划 + B-spline 距离代价 + ESDF**，即"控制器外闭环" | `CostCritic`/`ObstaclesCritic` **直接用 costmap（含 inflation/STVL 体素）给采样轨迹打分**，即"控制器内闭环" | **MPPI**：对突发障碍/动态障碍的响应在控制周期内；本仓库要等一次重规划（≤50 ms 决策 + B-spline 求解） |
| 精度/平滑 | 连续量输出，横移+自旋解耦显式建模；跟踪误差可做到 cm 级（无数据，[推论]） | 受 `batch_size`/噪声 σ 限制，控制有采样抖动；靠 `Savitzky-Golay`/`velocity_smoother` 收拾 | **MPC** |
| 调参成本 | 4 个权重（YAML 可调）＋ **模型/约束改动需重新生成 ACADO 代码**（要 MATLAB+ACADO 或 ACADO C++ 工具链） | 全在 YAML：critics 权重、`batch_size`、`time_steps`、噪声参数（我们已有注释体系） | **MPPI** |
| 依赖门槛 | ACADO 预编译 `.a` + qpOASES + **NLopt**（缺失）＋ `rmoss_interfaces`（缺失）＋ 代码生成链 | **零额外依赖**（`nav2_mppi_controller` 1.1.20 已装） | **MPPI** |
| CPU/GPU | 90 个 QP 变量 × 10 次 RTI，纯 CPU、微秒~亚毫秒级【推论】；无 GPU 依赖 | 9e4 轨迹点/周期纯 CPU（我们注释里已记为"sim 里开销大"） | **MPC** |
| 确定性 | 热启动 QP，同输入同输出（迭代上限/浮点误差外） | 噪声发生器驱动（可 seed），每周期轨迹不同 → **近似随机** | **MPC** |
| 碰撞保证 | **都不给保证**：MPC 只保证控制箱约束，因为障碍根本不在 OCP 里；想有保证必须把 ESDF 线性化成状态约束 + slack（**本仓库没做**） | 无任何硬保证（critic 软惩罚） | 都不行；这是"真·ESDF-MPC"才可能补的洞 |

【推论·最重要的一条】本仓库的 MPC **不具备**"梯度式 MPC"的招牌卖点（把障碍当约束/可微代价进优化）。它是**纯跟踪器**：亮点在"显式约束 + 高频率 + 平滑 + 低算力"，避障与 MPPI 一样靠外环。所以用它替换 MPPI，**避障只会变差，不会变好**；要拿到"ESDF-MPC"的真正价值，必须自己把 ESDF 距离/梯度写进 OCP（项 8.3 的工作量来源）。

---

## 8. 引入我们 bench 需要什么

### 8.1 依赖：装什么 / 本机已有什么

| 依赖 | 本机状态（2026-10-05 实测） | 结论 |
|---|---|---|
| ROS 2 Humble | ✅ `/opt/ros/humble`，`ROS_DISTRO=humble` | 满足 |
| Nav2（对照用） | ✅ `nav2_mppi_controller` / `nav2_controller` / `nav2_costmap_2d` / `nav2_smac_planner` / `nav2_navfn_planner` 均 **1.1.20** | 满足 |
| STVL | ✅ `/opt/ros/humble/share/spatio_temporal_voxel_layer` | 满足 |
| `pointcloud_to_laserscan`、`linefit`、`fast_lio`、`point_lio`、`livox_ros_driver2` | ✅ 源码在 `src/rm_perception/...`、`third_party/`，且 `install/` 里已构建 | 满足 |
| `icp_registration` | ✅ **我们已有**：[src/rm_localization/icp_registration](src/rm_localization/icp_registration)（本仓库同款包） | 满足 |
| **NLopt** | ❌ 无 `/usr/include/nlopt*`，`dpkg -l \| grep nlopt` 空 → `sudo apt install libnlopt-dev`（`bspline_opt/CMakeLists.txt:9` 是 `REQUIRED`） | **必装** |
| **ACADO Toolkit** | ❌ 无 `/usr/local/include/acado*`、无 apt 包 —— **但构建不需要**（`find_package(ACADO)` 已注释，只链预编译 `.a`）；**只有"要改模型/约束/N"时才需要**（MATLAB+ACADO 或 ACADO C++ 工具链） | 视目标而定 |
| **qpOASES** | ❌ 独立库没装，**但源码随包内嵌**（`omni_model/qpoases/SRC|INCLUDE`） | 无需另装 |
| glog | ✅ `libgoogle-glog-dev 0.5.0` | 满足 |
| Eigen3 / PCL / OpenCV | ✅ `libeigen3-dev 3.4.0`、`libpcl-dev 1.12`、`libopencv-dev 4.5.4`（`/usr/include/opencv4`） | 满足 |
| OSQP / osqp-eigen / `xmacro` | ❌ 都没有（`python3 -c "import osqp/casadi/xmacro"` 全失败）——【推论】**当前 master 并不需要**：tree 里无 osqp 源/CMake 引用，`planner_manager` 只链 ACADO 静态库；README 那几条属遗留说明 | 可跳过（先试） |
| cppAD | ❌ 未装，但 `planner_manager/package.xml:L20` 声明 `<depend>cppad</depend>` → `rosdep install` 会报缺（README 自己写了"cppAD 已废除"） | 建议删依赖或补装 |
| **rmoss_interfaces** | ❌ 仓库里是空 submodule 且无 `.gitmodules`；但我们自己的 RM 栈里通常有同类 msg → 需自备或把 `kino_replan_fsm.hpp:563` 的 `GimbalCmd` 发布者摘掉 | **必补** |

### 8.2 缺的"部件"（逐条核对）

1. **"从 Livox 点云建 ESDF" —— 其实不缺**：`plan_env` 直接吃 `LaserScan`（`laser_topic`），而我们的 `/scan` 正是 `Livox → linefit(/segmentation/obstacle) → pointcloud_to_laserscan` 同一条链（[bringup_sim.launch.py:37](src/rm_nav_bringup/launch/bringup_sim.launch.py#L37)、[:264-275](src/rm_nav_bringup/launch/bringup_sim.launch.py#L264)、[laserscan_params.yaml](src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml)）。差别只在参数：我们 `max_height 0.1` / `range_min 0.12`，仓库 `max_height 0.5` / `range_min 0.0` / `range_max 10.0`（R:planner_manager/launch/plan.launch.py:L94-L101）。
2. **2D→3D / 2.5D 表示 —— 不需要**（见 8.4）。
3. **麦轮运动学模型 —— 已经有等价物**：`omni_model` 的模型就是全向一阶模型（`ẋ=vx cosψ−vy sinψ`…），与 Nav2 MPPI 的 `Omni` 同族。要改的是**速度包络**：把它从 ±0.5 / ±0.1 换成我们的（参照 [nav2_params_sim_mppi.yaml](src/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml#L141) 附近的 `vx_max/w_max`），**而这需要重新生成 ACADO 代码**（`lbValues/ubValues` 在 `acado_initializeSolver` 里写死，手改生成码属于 hack）。
4. **求解器构建**：`libacado_exported_rti.a` 是 x86-64 预编译产物；本机是 x86-64（i7-14700HX，28 线程）→ 可用，但要注意 ABI/`-lrt` 链接与 GCC 版本。【推论】若不想碰 ACADO：这个 OCP 极小（3 状态 × 3 控制 × 30 步 + 仅箱约束），用 **CasADi+OSQP / acados / 手写 iLQR-DDP** 重写约 200–400 行，反而更可控（也顺便解决"包络写死"与"加 ESDF 项"两个问题）。
5. **集成契约**：它发 `/cmd_vel` 的 `Twist`（含 `linear.y`、`angular.z`）。注意我们踩过的坑——`fake_vel_transform` 会在小陀螺模式下替换角速度（见 [algorithm_axes.md:129-131](docs/algorithm_axes.md#L129)）⇒ 该节点必须挂在 `fake_vel_transform` **之前**（或车上关 spin），否则行为不可预期。

### 8.3 工作量估计【推论，非实测】

| 目标 | 内容 | 估时 |
|---|---|---|
| A. 先跑通（验证它到底能做什么） | 装 `libnlopt-dev`；补 `rmoss_interfaces`；改 `mpc.hpp` 的 `../../../src/planner_manager/...` 包含路径；清掉配置/launch 里的 `/home/z/...`；只启 `planner_manager`（不启它的 FAST-LIO/串口/驱动），把我们的 FAST-LIO odom remap 成 `/Odometry`、`/scan` 直连、`/cmd_vel` 接 Nav2 之前的通道；用 `/goal_pose` 手动给点 | **0.5–1 人日** |
| B. 换成我们的速度包络 | 方案 B1：装 ACADO 工具链，改 `main.cpp` 的 `subjectTo` 后重新 export（推荐）；方案 B2：改用 acados/CasADi 重写同一 OCP | **1–2 人日** |
| C. 做"真·ESDF-MPC"（避障进 OCP） | 在 OCP 里加障碍项：取 ESDF 距离/梯度线性化为状态约束（+ slack 软约束），或直接用 `dist(x,y)` 的可微代价；调参 + 仿真回归 | **+3–8 人日** |
| D. 接成 Nav2 插件形态 | 见 §9 形态 2（需自建"路径→参考状态序列 + yaw 规划 + 避障层"） | **3–6 人日** |

### 8.4 2D/2.5D 简化够不够？—— **够，而且它就是本仓库的答案**

【事实】本仓库本身已经是 **2D ESDF + 高度带压缩**的打法，并在 RM 赛场用过（地图/PCD 命名 RMUC/RMUL）。
【推论】对**平坦的 RM 场地**：
- 全 3D ESDF（voxblox/nvblox）的边际收益 ≈ 0：障碍贴地、雷达装在 ~0.3 m 高、机器人是 2D 运动学；3D 只会把"可通行"判据变复杂（还要处理悬空结构/点云稀疏）。
- **2.5D 只在"高地/斜坡/台阶"（RMUC 高地）才有意义**；而我们现有的 `pointcloud_to_laserscan` 高度带（`min_height -1.0 / max_height 0.1`）**已经把高地当障碍**，等价于"把不可通行的 3D 结构投影成 2D 障碍"——这正是最省事的 2.5D。
- 所以正确路线是：**用现有 2D 表示做距离场/梯度**（`/scan`→EDT，或直接在 costmap 上算 EDT），把距离场喂给 MPC 的障碍项；**不要**引入 3D ESDF。
- 距离场来源三选一：① 直接移植 `plan_env`（**但见 §10 许可风险**，且它自带 30 m×30 m / 0.01 m 的大缓冲）；② 在 Nav2 costmap 的 master grid 上自写 ~50 行 Felzenszwalb EDT（避开许可问题，复用它 20 Hz 局部更新的思路）；③ 抄其"局部窗口 + 双缓冲"的工程技巧（只算 3 m 窗口，R:plan_env/src/sdf_map.cpp:L393-L394）。

### 8.5 主要风险

1. **许可**：【事实】本仓库 **无 LICENSE**，自研包 `<license>TODO</license>`；血统指向 **GPL-3.0** 的 Fast-Planner 系（`HKUST-Aerial-Robotics/Fast-Planner` = GPL-3.0，`ZJU-FAST-Lab/ego-planner` = GPL-3.0，均已用 GitHub API 核对）。⇒ 直接搬 `plan_env`/`bspline_opt` 代码进我们的仓库有**传染性许可与"未授权"双重风险**，引入前必须做来源与许可确认。
2. **包络写死**：不重新生成代码就只能用 0.5 m/s（对我们 1.5+ m/s 的哨兵基本不可用）。 | R:planner_manager/omni_model/acado_solver.c:L1212-L1223
3. **子模块缺失 + 硬编码绝对路径 + 仓库内提交 `.o`/zip**：可移植性差，`colcon build` 大概率要手工改 3~5 处。 | R:planner_manager/package.xml:L23；R:planner_manager/config/planner_manager.yaml:L31
4. **避障能力倒退**：只搬 MPC ⇒ 没有 costmap/STVL 语义（禁行区、膨胀、动态障碍），避障弱于现有 MPPI（见 §7）。
5. **契约冲突**：`/cmd_vel` 全向速度 + `fake_vel_transform` 小陀螺替换角速度；且它期望 20~100 Hz 的 `odom` 与 `/scan`（我们的 p2l 是 10 Hz 附近）→ 频率/时延契约要对齐。
6. **代码质量**：越界索引（`cur_state[3]`）、循环里打印 `"asd"`、参数名拼写（`resolusion_`）、配置键不匹配（`plan_cheak_frequency`）——搬过来会带进噪声与隐 bug。
7. **CPU/内存**：0.01 m × 30 m 的 2D 网格 ≈ 300 MB 缓冲；若要 20 Hz 全场 EDT（而不是 3 m 局部窗）会明显吃 CPU。

---

## 9. 与 Nav2 的集成方式（能不能只换 local controller？）

【事实】它**不是** `nav2_core::Controller` 插件：节点继承 `rclcpp::Node`（R:.../kino_replan_fsm.hpp:L41），用 `rclcpp_components_register_node` 注册（R:planner_manager/CMakeLists.txt:L35-L38），package.xml 里**没有 `nav2_core`/`pluginlib`**（R:planner_manager/package.xml:L12-L25）。
【事实】它是一个**自带全局规划 + 轨迹优化 + 偏航规划 + 跟踪 + 安全检查的独立 FSM 节点**：输入 `/Odometry` + `/goal_pose`，输出 `/cmd_vel`（R:.../kino_replan_fsm.hpp:L56-L90；R:planner_manager/src/kino_replan_fsm.cpp:L346-L355）。**它不消费 `nav_msgs/Path`，也不读 costmap。**

| 集成形态 | 可行性 | 代价/风险 | 估时 |
|---|---|---|---|
| **① 独立节点（绕过 Nav2）** | ✅ 最省事：BT/脚本里把目标转成 `/goal_pose` 发给它，让它自己规划+控制 | 失去 Nav2 的 costmap 层（静态图/STVL/禁行区）、行为树、恢复行为、`velocity_smoother`；两套规划器并存易冲突 | 0.5–1 人日 |
| **② 包成 `nav2_core::Controller` 插件（只换局部控制器）** | ⚠️ 接口上可以：`computeVelocityCommands()` 能拿到 `nav_msgs/Path`（全局路径）+ 当前位姿 + 当前速度 | **必须自己补三样**：(a) 路径→参考状态序列（它现在从自己的 B-spline+yaw 规划采样 180 个参考量，R:planner_manager/src/kino_replan_fsm.cpp:L259-L324）；(b) 偏航参考生成（`planYaw` 用的是自己的位置 B-spline）；(c) **避障**（MPC 无障碍项）→ 实际等于把 2D ESDF + B-spline 优化器一起搬进来 ≈ 重写一个 `mpc_local_planner` | 3–6 人日 |
| **③ 只借 `Mpc` 类 + ACADO 求解器** | ✅ 可行：`Mpc`（`mpc.hpp`/`mpc.cpp`）只依赖 Eigen + ACADO 头（R:planner_manager/include/planner_manager/mpc.hpp:L6-L9），把参考轨迹从外部喂进来即可 | 需自备参考生成器与避障；包络写死问题依旧；适合做"MPPI vs 梯度 MPC 跟踪同一参考"的 A/B | 2–4 人日 |

【推论】它**不需要**自己的全局规划器才能跑（形态 ① 自带、形态 ②/③ 不需要），但**需要**"参考位姿+偏航+速度"三件套；Nav2 只给 (x,y,θ?) 路径 ⇒ 形态 ② 的主要工作就是补这个参考层 + 避障层。
【对照】Nav2 官方本来就没有 MPC 族（我们自己的分类也这么写：[nav_strategy_taxonomy.md:297](docs/nav_strategy_taxonomy.md#L297)）⇒ 引入它等于"补一个族"，但生态成本高；若只想在 `nav` 槽位里体验"有显式约束的控制器"，**先做形态 ③ 的 A/B 最划算**（同一参考轨迹、同一场景，比较跟踪误差/平滑度/CPU）。

---

## 10. 无法确定的（明确列出）

1. **所有性能数字缺失**：求解耗时、MPC 实际频率是否稳定在 100 Hz、最高速度、能处理的障碍密度、成功率——仓库没有任何 bag/图表/表格，计时打印被注释（R:planner_manager/src/kino_replan_fsm.cpp:L335）。§7/§8 里带"【推论】"的 CPU/精度判断均为量级估算。
2. **实车机器人型号**（哨兵？步兵？）、底盘到底是麦轮还是全向轮、质量/惯量/URDF：仓库**不含 description/URDF**，只能由 `vx,vy,w` 控制与 `CarType 0` 圆盘（半径 0.27 m）推断"具全向运动能力"。
3. **是否真跑过 RMUC2025 并取得成绩**：`RMUC2025.pgm`/`RMUC.pcd` 只能说明用过这些场地资产。
4. **`omni_model` 生成码与 `ACODO_ws.zip` 里 `main.cpp` 是否同一次生成**：【推论】极可能一致（`acado_common.h` 8908 B、`acado_solver.c` 105 375 B、`libacado_exported_rti.a` 240 958 B **字节数完全相同**，且 `lbValues` 的 0.5/0.5/0.1 与 `main.cpp` 的 `subjectTo` 一致），但**无法证明**（zip 里没有模型哈希/时间戳对应关系）。
5. **在我们机器上能否 `colcon build` 成功**：**未实测**（本任务为只读调研；本机缺 NLopt、缺 `rmoss_interfaces`、缺 ACADO）。另外 `rmoss_interfaces` 是空 submodule 且无 `.gitmodules`，能否找到匹配版本同样未验证。
6. **`dynamic_environment: 0` 之外的动态障碍路径是否可用**：`obj_predictor.cpp` 仅 874 B、`minDistToAllBox` 循环打印 `"asd"`，【推论】该路径基本是死代码/半成品，但未逐一读完（`obj_predictor.hpp` 未展开分析）。
7. **许可归属**：无 LICENSE、无作者声明、上游血统只能"由命名与算法推断"；**这不是法律意见**，任何代码复用前需自行确认（Fast-Planner 系 = GPL-3.0 已核对：<https://api.github.com/search/repositories?q=Fast-Planner+in:name>）。
8. **MPPI 与本仓库 MPC 的直接对比实验**：两边场地/底盘/传感链虽高度相似（都是 Livox→linefit→p2l→2D、都是全向底盘），但**从未在同一场景下比过**——本节所有"谁更强"均为机制层面的推论，不是实测。

---

### 附：最小可复现的证据索引

- 仓库全景：<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/git/trees/master?recursive=1>（548 项，未截断）
- 提交史：<https://api.github.com/repos/865749/Kinodynamic_esdf_mpc/commits?per_page=30>（13 条，最早 `Ouuu` 2025-01-12）
- **ACADO 模型定义原文**：`ACODO_ws.zip` → `ACODO_ws/src/acado_test/src/main.cpp`（<https://github.com/865749/Kinodynamic_esdf_mpc/blob/master/ACODO_ws.zip>）
- 生成码维度/约束：R:planner_manager/omni_model/acado_common.h:L65-L89；R:planner_manager/omni_model/acado_solver.c:L1212-L1223
- 2D EDT：R:plan_env/src/sdf_map.cpp:L352-L473；梯度：R:plan_env/src/edt_environment.cpp:L71-L114
- 规划–控制流：R:planner_manager/src/kino_replan_fsm.cpp:L50-L164（规划）、L246-L375（控制）；R:planner_manager/include/planner_manager/kino_replan_fsm.hpp:L344-L500（FSM）
- 本机对照：`/opt/ros/humble/share/{nav2_mppi_controller,spatio_temporal_voxel_layer}`；[nav2_params_sim_mppi.yaml](src/rm_navigation/rm_navigation/params/nav2_params_sim_mppi.yaml#L141)；[bringup_sim.launch.py](src/rm_nav_bringup/launch/bringup_sim.launch.py#L37)
