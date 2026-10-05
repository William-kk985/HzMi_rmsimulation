# 重定位槽位登记表（localization slots）

> 入口：`bringup_sim.launch.py` 的 `localization` 参数（**仅 `mode:=nav` 生效**）
> 当前 choices：`['', 'amcl', 'beluga', 'slam_toolbox', 'icp', 'gicp', 'cartographer']`
> **统一契约**：重定位槽**只负责发布 `map→odom`**；`odom→base_link` 由 LIO（+ `lio_tf_adapter`）负责；Nav2 用 `base_link_fake`。
> **`map→odom` 只能有一个发布者** ⇒ 重定位槽之间、以及与 `mapper:=*` 之间必须互斥。

## 1. 已可用（现成入口）

| 槽位值 | 节点 / 包 | 参数文件 | 需要的资产（现状） |
|---|---|---|---|
| `amcl` | `nav2_amcl` | `rm_navigation/params/nav2_params_sim_base.yaml`（已调：`transform_tolerance 0.3` / `update_min_d,a 0.05` / `recovery_alpha_*` 已打开，见工单 §K） | 2D 栅格图 `rm_nav_bringup/map/RMUL2026.pgm|.yaml` ✅ |
| `beluga` | `beluga_amcl/amcl_node`（AMCL 的现代实现，**同名参数、同 lifecycle 形态**；2026-10-05 新增，见 §1.2） | `rm_navigation/params/nav2_params_sim_beluga.yaml`（`amcl` 段；`transform_tolerance 0.3` / `update_min_d,a 0.05` / `recovery_alpha_*` 原样搬运） | 2D 栅格图 `rm_nav_bringup/map/RMUL2026.pgm|.yaml` ✅ **+ 需装 `ros-humble-beluga-amcl`**（Humble 有官方二进制；源码路线见 §1.2） |
| `icp` | `icp_registration/icp_registration_node`（**我们自己的包**，可改） | `icp_registration/config/icp_registration_sim.yaml`（含 `pcd_path`） | 先验点云 `rm_nav_bringup/PCD/RMUL2026.pcd` ✅（RMUL/RMUC 也有文件，但 **RMUC.pcd 是退化资产**，见 §1.1） |
| `gicp` | `gicp_registration/gicp_registration_node`（**我们自己的包**，2026-10-05 新增） | `gicp_registration/config/gicp_registration_sim.yaml`（含 `pcd_path`、`tf_lookahead_sec`、两级 leaf `voxel_leaf_size 0.10` / `voxel_leaf_size_scan 0.05`，见 §1.1） | 先验点云 `rm_nav_bringup/PCD/<world>.pcd` ✅（RMUL2026 实测可用，0.10 m leaf 后 target ≈1.2e4 点；**必须有初值**，见 §1.1） |
| `slam_toolbox` | `slam_toolbox/localization_slam_toolbox_node` | `slam_toolbox/config/mapper_params_localization_sim.yaml` + launch 注入 `map_file_name=map/<world>`、`map_start_pose=[0,0,0]` | **序列化位姿图 `map/RMUL2026.posegraph(+.data)` ❌ 缺**（RMUC/RMUL 有）⇒ 需先建图并 `serialize_map` |
| `cartographer` | `cartographer_node`（纯定位） | cartographer 配置 | `map/RMUL2026.pbstream` ✅ |
| `''`（留空） | 无重定位：LIO 当绝对定位 + 静态桥补帧 | — | — |

运行示例（把 `localization:=` 换成上表任一项）：
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=icp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
```

### 1.1 `gicp` 槽位细节（2026-10-05 新增；与 `icp` 同资产、同初值契约）

- **节点**：`gicp_registration/gicp_registration_node`（`ament_cmake` + `rclcpp_components`）；
  算法 = PCL `pcl::GeneralizedIterativeClosestPoint`，点类型 = **`pcl::PointXYZ`**。
  为什么不用 `PointNormal/PointXYZINormal`：PCL 1.12.1 的 GICP 用 KNN 邻域**自己算协方差**
  （`pcl/registration/impl/gicp.hpp:51-125`，只读 x/y/z），normal/intensity 一律不读；而我们的
  PCD 资产字段并不统一（`RMUC.pcd` 连 intensity 都没有）⇒ `PointXYZ` 三种资产都能读，
  实时点云也不依赖 intensity（缺字段不会刷 "Failed to find match for field"）。
- **契约**：只发 `map→odom`（TF，`publish_rate_hz` 默认 50 Hz），外加三个**调试/健康**话题：
  `~/pose`（map 系机器人位姿 ≈ `/amcl_pose` 语义，便于 A/B）、
  `~/fitness_score`（m² = 内点平均平方距离；无内点时 nan）、`~/converged`(bool = 本帧被采纳且
  score ≤ `fitness_score_warn`)。后两者 `transient_local` ⇒ 监控端后订阅也能拿到最后一帧。
- **时间戳契约 `tf_lookahead_sec`（2026-10-05 修复；≈ AMCL 的 `transform_tolerance`）**：
  TF `map→odom` 与 `~/pose` 都盖 **`now() + tf_lookahead_sec`**（现值 **0.45 s**），其中 `now()` 是节点时钟
  （`use_sim_time=true` 时 = 仿真时间）——**故意不用点云 `header.stamp` 盖 TF**。
  （历史：2026-10-05 先取 0.3 s，同日 commit `c8863f1` 提到 **0.45 s** —— 依据是"CPU 过载时 TF 定时器
  被 align 饿死"的实跑日志；代码里的兜底默认仍是 0.3，launch 一定注入参数文件 ⇒ 实跑恒为 0.45。）
  为什么：nav2 的消费者**不在「此刻」查 `map→odom`**，而是在 `now + transform_tolerance` 那一档查
  （MPPI 的 `PathHandler::transformPose` 用 `FollowPath.transform_tolerance`；本仓库 = 0.1 s）。
  2026-10-05 实跑 `localization:=gicp` 的 `controller_server` 日志逐条是 `requested = 最新数据 + 0.100 s`：
  `Exception in transformPose: Lookup would require extrapolation into the future. Requested time
  654.682000 but the latest data is at time 654.582000, when looking up transform from frame [odom]
  to frame [map]` ⇒ `Unable to transform robot pose into global plan's frame` ⇒ `[follow_path] Aborting
  handle`（每周期 abort + BT 反复恢复 ⇒ 导航卡死）。
  **AMCL 从不报这个错，正是因为 `transform_tolerance` 让它把 `map→odom` 盖成未来时间戳**
  （本仓库 amcl `transform_tolerance: 0.3`，`nav2_params_sim_base.yaml`）；gicp 原来用 `now()` 盖戳，
  最新条目就永远比请求时间旧 0.1 s ⇒ tf2 抛 `ExtrapolationException`。
  取值：**过小** ⇒ 复现上述报错；**过大** ⇒ 位姿被外推过头（2 m/s 时 0.3 s ≈ 0.6 m 提前量），
  高速/转弯表现为定位滞后→猛修正→来回抖（与 `transform_tolerance` 1.0→0.3 同一个理由）。
  调法：**≥ 消费端最大的 `transform_tolerance` + 一个 TF 发布周期**（50 Hz ⇒ 20 ms），先与 AMCL 一致
  （0.3），2026-10-05 起按"定时器可能被 CPU 饿死"的实测余量提到 **0.45**（commit `c8863f1`；同时用
  多线程 executor + 独立回调组把"饿死"本身治掉，见下面「并发模型」条）。
  本次**静态 + 探针实测**（只跑 gicp 节点，不启 Gazebo/nav2）：`tf_lookahead_sec=0.0`（≈ 修复前的
  `now()` 盖戳）时，`now+0.00…+0.40 s` 的 `map→odom` 查询**全部**抛 `ExtrapolationException`
  （报文与实跑日志逐字一致，含 `from frame [odom] to frame [map]`）；`=0.3`（当时的默认）时
  `now+0.00/0.05/0.10/0.20 s` 全部成功（覆盖 MPPI 的 0.1 前瞻，余量 0.2 s），
  而 `now+0.30 s`（正好等于 lookahead）仍可能因"最新条目比查询时刻旧约 1~20 ms"而失败
  ⇒ 所以余量要算上发布周期，不能只等于消费端的 `transform_tolerance`。
  附带：`~/fitness_score`/`~/converged` 是 `std_msgs/Float64`/`Bool`（**没有 header，没有 stamp**），
  只给人看、不参与 TF 查询 ⇒ 本次不给它们加前瞻；`~/pose` 与 TF 用**同一条**戳。
- **初值（硬要求）**：`/initialpose`（RELIABLE，RViz 2D Pose Estimate，语义 = **map 系下机器人位姿**，
  与 AMCL 一致）或参数 `initial_pose`（默认 `[0,0,0]`，3 或 6 元）。
  **没有初值就不发 TF**（每 2 s 日志说明一次）——不发比发错的更安全；
  `use_initial_pose: false` 则从恒等 `map→odom` 起步（= 把 LIO 当绝对位姿，`/initialpose` 仍有效）。
  RMUL2026 的 map 系 = 出生点相对系 ⇒ 出生即 `(0,0,0)`；若换回 `map/RMUL2026_world_backup.*` 那类
  世界系栅格图，出生点应给 `(4.3, 3.35, 0)`（依据见 `bringup_sim.launch.py` 里 `amcl_init_*` 的注释）。
- **每帧行为**：点云（SensorDataQoS / BEST_EFFORT，KEEP_LAST(1)）→ 去 NaN → 体素下采样
  （**source 侧用 `voxel_leaf_size_scan`，与地图 target 侧解耦**，见下条"两级下采样"）→
  初值 = 上一次 `map→odom` × 本帧里程计增量（`T_sensor←odom`，故能连续跟踪）→ GICP →
  **只在「收敛 且 score ≤ `max_fitness_score`」时**更新 `map→odom`；否则沿用旧值并限频 WARN，
  连续 `no_improve_cycles_warn`（默认 10）帧后补一条明确的「定位已失效 + 排查顺序」WARN。
- **参数键核实（PCL 1.12.1 = 本机 `libpcl-dev 1.12.1+dfsg-3build1`，头文件 `/usr/include/pcl-1.12/`）**：
  `max_correspondence_distance`→`corr_dist_threshold_`（`impl/gicp.hpp:414` 距离门限）、
  `maximum_iterations`→`max_iterations_`（`impl:496` 收敛判据）、
  `transformation_epsilon` / `rotation_epsilon`（`impl:474-483` GICP 特有的两段式收敛判据）、
  `correspondence_randomness`→`k_correspondences_`（`impl:51-125` 算协方差的 KNN 数）、
  `maximum_optimizer_iterations`→`max_inner_iterations_`（BFGS 内层上限）。
  **两个候选键已核实无效 ⇒ 故意不写**（写了是静默失效，与 nav2 "Jazzy 键不声明" 同一个坑）：
  `euclidean_fitness_epsilon`（只有 `ICP/JointICP` 的 `convergence_criteria` 读它，`impl/icp.hpp:157`；
  GICP **覆写**了 `computeTransformation`，`impl/gicp.hpp:390`）与
  `use_reciprocal_correspondences`（只有 `ICP::determineCorrespondences` 读它，`impl/icp.hpp:180`；
  GICP 自己找最近邻，`impl/gicp.hpp:438`）。任务书里的 `set_use_reciprocal_correspondences` 即后者，
  按"去 set 前缀"的同一约定落名，核实后**不实现**。
- **两级下采样 leaf（two-tier；2026-10-05 改，recipe 照抄 COD 2025 的 `small_gicp_relocalization`）**：
  上游参照实现 = COD（`cod_nav`）2025 的 `small_gicp_relocalization`，其参数为
  `num_threads: 8` / `global_leaf_size: 0.25`（**先验 PCD** 侧）/ `registered_leaf_size: 0.05`
  （**实时点云** 侧）/ `max_dist_sq: 2.5`（对应点距离门限的**平方** ⇒ √2.5 ≈ 1.58 m）。

  | 键 | 作用侧 | 本节点 | COD 对应键 |
  |---|---|---|---|
  | `voxel_leaf_size` | **先验地图 / GICP target** | `0.25 → 0.10`（键名保持不变以兼容旧配置，语义收窄为"仅地图侧"） | `global_leaf_size: 0.25` |
  | `voxel_leaf_size_scan` | **实时点云 / GICP source** | `0.05`（**新增**；原实现**复用地图 leaf 0.25** 给实时点云下采样 ⇒ 单帧只剩几百点） | `registered_leaf_size: 0.05` |
  | `max_correspondence_distance` | GICP 对应点距离门限 | `1.0 → 1.5 m`（给初值更多收敛余量） | `max_dist_sq: 2.5`（√2.5 ≈ 1.58 m） |

  为什么必须改：0.25 m 时 `RMUL2026.pcd` 只剩 **2438** 个 target 点（启动 banner 实测；GICP 的目标
  协方差用 KNN=20 估计，平均间距 ~0.4 m ⇒ 邻域退化、平面假设失真）⇒ 实跑 `map→odom` 在 30 s 窗口里
  漂 **~8 cm**（regression 判 `settled=False`）。改成 0.10 m 后 target = **12450** 点（banner 实测，
  ×5.1）⇒ 目标协方差与最近邻配对都稳定得多。COD 之所以能直接用 0.25，是因为它的先验 PCD 密度比
  我们这份高一个量级（我们的 PCD 总共才 53164 点）；source 侧照抄它的 0.05（细一档）——
  既保住单帧几何，也避免"细地图 + 粗点云"的尺度错配。两个 leaf 都必须 > 0（`<=0` 判非法并退回默认；
  本节点**不提供**"0 = 不下采样"档，不采样会让单帧点数/协方差计算量失控）。
  可观测量（本次一并加）：启动 banner 打**两个 leaf + target 点数**；每 ~1 s 一条 `[status]` 行给
  **采纳帧数 / 最近 score / align 耗时 ms / source→target 点数 / map→odom**；未采纳的限频 WARN 也带
  align ms（用 `steady_clock` 限频、单行、不刷屏）。
  代价（**本机实测**，单节点、不启 Gazebo/nav2；12450 target + 3701 source）：首个 align **2460~2549 ms**
  （含目标协方差预计算），其后每帧 **327~389 ms**（PCL GICP 单线程；`maximum_iterations` 32→16 后不变，
  原因见下条「并发模型」的最后一段）⇒ 实时跑仍要注意与 nav2 抢 CPU（日志出现
  `Control loop missed its desired rate` 就是它）。align 原来还跑在**单线程 executor** 里 ⇒ 50 Hz 的 TF
  发布定时器被它阻塞：同一次实测里 `/tf` 上的 `map→odom` 实际只有 **2.3 Hz** ⇒ 整栈再出现
  `extrapolation into the future`（`tf_lookahead_sec=0.45` 的余量被 align 吃满）。
  **2026-10-05 已治本**（见下条）：多线程 executor + 独立回调组。
  COD 的对策是 `num_threads: 8` 的多线程 small_gicp，本节点仍是单线程 PCL
  `GeneralizedIterativeClosestPoint` ⇒ 若跟不上：先把 `voxel_leaf_size_scan` 放到 0.10、
  再降 `maximum_iterations`（16→8；精度优先则回 32），最后才回退地图 leaf。
  回退（= 恢复 098078d 之前）：`voxel_leaf_size: 0.10 → 0.25`、删掉 `voxel_leaf_size_scan`
  （或设成与地图 leaf 同值）、`max_correspondence_distance: 1.5 → 1.0`；`git revert <commit>` 亦可。
  ⚠️ 注意 `max_correspondence_distance` 同时是 `getFitnessScore` 的球半径（PCL 里传平方），
  1.0→1.5 m 让门限从 1.0 m² 变 2.25 m² ⇒ **fitness score 量级会整体上移**，
  `fitness_score_warn / max_fitness_score`（0.05 / 0.3 m²）本次**故意不动**，实跑后按实测量级重定。
- **并发模型：TF 定时器与 GICP 解耦（2026-10-05 修复；本次新增，`maximum_iterations 32→16` 同一 commit）**：
  症状链 —→ 两级 leaf 变细后单帧 align **~350 ms**，而 TF 定时器与点云订阅原来挤在**同一个单线程
  executor** 的默认回调组里 ⇒ align 期间定时器根本排不上队：`ros2 topic hz /tf` 实测 map→odom 只有
  **2.3 Hz**（同一次实测里 TF 条数 ≈ 采纳帧数，57 条 / 24.7 s），50 Hz 只是"名义值" ⇒
  `tf_lookahead_sec=0.45` 的余量被吃满 ⇒ nav2 消费者又看到 **`extrapolation into the future`**。
  修法（**契约完全不变**：只发 `map→odom`、点云仍 SensorDataQoS/BEST_EFFORT、`/initialpose` 仍
  RELIABLE、`tf_lookahead_sec` 语义不变、无初值仍不发 TF）：
  · `gicp_registration_node` 改用 **`MultiThreadedExecutor`** —— `CMakeLists.txt` 里
    `rclcpp_components_register_node(... EXECUTOR MultiThreadedExecutor)`，生成 main 里就是
    `exec.add_node(node); exec.spin();`（生成物：`build/gicp_registration/rclcpp_components/
    node_main_gicp_registration_node.cpp`）；
  · **TF/状态定时器独占 `tf_cb_group_`**（`MutuallyExclusive`），**点云订阅（GICP 重活）+ `/initialpose`
    放 `align_cb_group_`**（另一个 `MutuallyExclusive`）⇒ 两组由 MT executor **并行**调度；
    `/initialpose` 与点云同组是有意的：人工初值的写入必须与"align 读改写 map→odom"串行
    （否则点击会被在飞的 align 结果覆盖），且组内互斥保证 PCL GICP 对象 / `first_align_done_` 仍是
    单线程访问（GICP 本身不是线程安全的，绝不能并发 align）；
  · **跨两组共享的状态一律在既有 `mutex_` 下读写**：缓存的 `T_map_odom_`/`estimate_valid_`、
    `last_tf_stamp_`(+valid)、`cloud_seen_`/`last_cloud_stamp_`、`accepted/total/no_improve_cycles_`、
    `param_init_pending_`、`last_align_ms_`/`last_score_`/`last_source_points_`、
    `last_status_log_tp_`/`status_log_ever_printed_`，以及本次**新补锁**的 `last_tf_error_`
    （std::string，lookupTf 加锁写 + `lastTfError()` 加锁读）。三个健康话题（`~/pose`、
    `~/fitness_score`、`~/converged`）只由 align 组发布（rclcpp 的 `publish()` 本身线程安全），
    **故意不持状态锁发布** —— 一次阻塞的 DDS 写会把定时器线程一起拖住，等于把"饿死"换个姿势带回来。
  · **实测（单节点、不启 Gazebo/nav2、`ROS_DOMAIN_ID=97`）**：`/tf` 的 map→odom
    **2.3 Hz → 50.0 Hz**（`ros2 topic hz /tf` 中位数 50.000，min/max 20/21 ms），
    而 align 仍是 **344~389 ms**（≈ 改前的 368 ms）⇒ **修复来自解耦，不是"迭代数变少"**：
    同一个 32 次迭代的对照跑（`-p maximum_iterations:=32`）也是 50.0 Hz。
  · 附：`maximum_iterations 32→16`（同一 commit）的实测结论 —— 在同一合成扫描下把
    `maximum_iterations` 扫成 1/4/16/32/64，align 分别 **363/475/486/471/427 ms**（噪声量级、非单调），
    fitness 全部 **0.00123 m²**、`~/converged` 全 true ⇒ **跟踪场景下这个上限根本不生效**：
    PCL 1.12.1 的 `while (!converged_)`（`impl/gicp.hpp:420`）退出判据是
    `nr_iterations_ >= max_iterations_ || delta < 1`（`impl/gicp.hpp:496`），而 `delta` 由 `transformation_epsilon`/`rotation_epsilon`
    加权 —— 初值来自里程计递推、上一帧已收敛时它远早于 16 次就成立；每帧成本由固定开销
    （源协方差 KNN + 最近邻 + `getFitnessScore`）主导。改成 16 无害（与 32 完全一致），
    并给"初值差、需要更多迭代"的场景封顶；**精度退化就回 32**。
  · **回退阶梯**（仅当整栈仍出现 `Control loop missed its desired rate` / `extrapolation` ——
    解耦只治"饿死"，不治"总算力不够"）：① `voxel_leaf_size_scan` **0.05 → 0.10**（先牺牲实时点云
    精度，保住地图侧收益）→ ② `maximum_iterations` **16 → 8**（精度优先则反向回 32）→
    ③ 最后才把 `voxel_leaf_size` 回 **0.25**（= 098078d 之前的配置，可同时把
    `max_correspondence_distance` 回 1.0）。三步都不动契约与话题。
- **资产现状（2026-10-05 实测：节点启动日志 + 独立 numpy 解析互证；同日补测 0.10 m 列）**：

  | PCD | 原始点 | 去 NaN | **0.10 m 体素后（新默认）** | 0.25 m 体素后（旧默认/回退档） | 包围盒（0.10 m 下实测） | 可用 |
  |---|---|---|---|---|---|---|
  | `RMUL2026.pcd` | 53164 | 53164 | **12450** | **2438** | x[-1.93,10.06] y[-2.94,5.31] z[-0.43,0.23] | ✅ |
  | `RMUL.pcd` | 1589841 | 1589841 | 97642 | 15506 | x[-4.94,10.36] y[-4.95,7.56] z[-0.32,14.06] | ✅ |
  | `RMUC.pcd` | 649995 | 649995 | **8** | **8** | 全部挤在原点 ±1 cm（两种 leaf 都一样） | ❌ **退化资产** |

  ⇒ `RMUC.pcd` 的 x/y/z 全落在 3 cm 盒子里（换 reader/独立解析结论相同），**不能用于定位**；
  节点启动即 `ERROR` 退出并打出点数与包围盒（有意为之：把"跑起来但定位是垃圾"的静默失败变成显式失败；
  改 leaf 不改变这个结论——0.10 m 下仍然只有 8 点）。
- **已知限制（本次未实测项）**：① **整栈运行时的收敛性未实测**（本机不启动仿真，只跑过单节点）——
  单节点实测（12450 target + 3701 source）：每帧 align 327~389 ms、首个 2460~2549 ms（含目标协方差预计算），
  见上面"两级下采样"条；`maximum_iterations`（现 **16**）与两个 leaf 是 CPU/精度旋钮，整栈若出现
  `Control loop missed its desired rate`，按上面「并发模型」条给的回退阶梯调；
  ② `fitness_score_warn / max_fitness_score`（0.05 / 0.3 m²）是按 PCL 语义给的**首跑起始值**，
  且本次 `max_correspondence_distance` 1.0→1.5 让评分球半径变大（1.0→2.25 m² 门限）⇒ 量级会整体上移，
  必须按实测量级重定；③ 与 icp 一样**初值敏感**：给错初值会静默收敛到局部极小，
  只能靠 `~/fitness_score` + `~/converged` 发现 ⇒ 要"随便摆"仍需 `scan_context` 类全局检索；
  ④ `tf_lookahead_sec`（现 **0.45**，commit `c8863f1`）按 AMCL 语义实现，并用 tf2 探针做过 A/B（见上）；
  `map→odom` 的**发布节奏**已用单节点实测确认（`ros2 topic hz /tf` = 50.0 Hz，见「并发模型」条），
  但**整栈运行时**是否还有别的消费者在更远的时间点查 `map→odom`、以及 TF 是否平滑，仍需实跑确认
  （验证方法：`ros2 topic hz /tf` 看 map→odom 是否稳定 ~50 Hz、`ros2 run tf2_ros tf2_echo map odom`
  是否连续无跳变，再跑一次 P0 回归 `--goal -1.0 2.0`）。

运行 / 检查（新包必须先 build，否则 `install/` 里没有参数文件）：

```bash
colcon build --symlink-install --packages-select gicp_registration
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
# 若日志说"尚无 map→odom"：给初值（RMUL2026 出生点 = map 原点）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
# 健康三连
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /gicp_registration/pose
ros2 topic hz   /gicp_registration/fitness_score     # ≈ 点云帧率
ros2 topic echo /gicp_registration/fitness_score     # 量级：旧基线 0.05~0.3 m²（半径 1.5 m 后会整体上移）
ros2 topic echo /gicp_registration/converged         # false ⇒ 本帧未采纳/评分超 warn
# 节点自己的 [status] 行（~1 Hz，单行）：采纳帧数 / 最近 score / align 耗时 ms / source→target 点数 / map→odom
#   ⇒ A/B 两级 leaf 时先看这一行：target 是否 ≈1.2e4、align ms 是否可接受、采纳率是否接近 1
```

### 1.2 `beluga` 槽位细节（2026-10-05 新增；`beluga_amcl` = AMCL 的现代实现）

**事实核对（均在 2.1.1 上核实，出处见括号）**

- **上游 / 许可**：`Ekumen-OS/beluga`，**Apache-2.0**（GitHub API `license.spdx_id = apache-2.0`），默认分支
  `main`，自述"C++17 的通用 MCL 实现 + ROS 1/2 封装"。包：`beluga`（算法库）/ `beluga_ros`（ROS 接口）/
  `beluga_amcl`（AMCL 节点）/ `beluga_system_tests`（系统测试）/ `beluga_vdb` / `beluga_example` /
  `beluga_tutorial` / `beluga_benchmark`。
- **Humble 有官方二进制 ⇒ 不走源码构建**：`ros-humble-beluga-amcl 2.1.1-1jammy.20260908.012840`
  （另有 `ros-humble-beluga-ros`、`ros-humble-beluga`，均 2.1.1）。本机 `apt-cache policy` 直接读到候选版本，
  源 = `http://mirrors.aliyun.com/ros2/ubuntu jammy/main`（packages.ros.org 镜像）；`humble/distribution.yaml`
  里该包的 release 版本也是 `2.1.1-1`（`ros2-gbp/beluga-release`）。
- **版本↔提交**：apt 装的 2.1.1 对应上游 tag `2.1.1` = **`b06f9060ed6d38d3559b4d05368f5fbe74c6594f`**
  （本文所有参数与行为结论都按这个 tag 的源码逐行核对；若走源码路线请 checkout 这个 commit 以保持一致）。
- **构建要求**（源码路线才需要）：**C++17**（`beluga_amcl/CMakeLists.txt` 的
  `target_compile_features(amcl_node_component PUBLIC cxx_std_17)`），构建类型 `ament_cmake`；
  依赖 `beluga`/`beluga_ros`/`message_filters`/`std_srvs`/`bondcpp`/`rclcpp`/`rclcpp_components`/
  `rclcpp_lifecycle`/`tf2_ros`（`beluga_amcl/package.xml`），beluga 本体还要
  `libeigen3-dev / libhdf5-dev / librange-v3-dev / libtbb-dev / ros-humble-sophus`。
  本机 GCC 11 / libstdc++6 ≥ 11 满足 deb 的 `libstdc++6 (>= 11)`；GCC 版本没有额外下限。
- **是 lifecycle 节点**（这点决定接线形态）：`class AmclNode : public BaseAMCLNode` 且
  `class BaseAMCLNode : public rclcpp_lifecycle::LifecycleNode`；可执行文件 `amcl_node`
  （`rclcpp_components_register_node(... PLUGIN "beluga_amcl::AmclNode" EXECUTABLE amcl_node)`），
  带 `bondcpp` bond（`bond_timeout` 默认 4.0）。上游 example 就是「`nav2_map_server` + `amcl` 交给同一个
  `lifecycle_manager_localization`（`node_names: [map_server, amcl]`）」（`beluga_example/launch/utils/localization_launch.py`）
  ⇒ 与本仓库 amcl 槽的形态**逐条同构**，所以 beluga 分支照抄 amcl 分支。
- `beluga_amcl` 包本身**不带 launch 文件**（`beluga_amcl/` 下只有 `src`/`include`/`test`/`docs`），
  launch 在上游的 `beluga_example` 里 ⇒ 我们自己的 launch 是必要的，不是重复造轮子。

**出处（URL；本次全部用 GitHub REST `api.github.com/.../contents/<path>?ref=2.1.1` 取原文核对，
下面给等价的人类可读链接）**

| 结论 | 出处 |
|---|---|
| Apache-2.0 / 默认分支 `main` / 包清单 | https://github.com/Ekumen-OS/beluga 、`https://api.github.com/repos/Ekumen-OS/beluga` |
| tag `2.1.1` = commit `b06f906…` | https://api.github.com/repos/Ekumen-OS/beluga/tags |
| Humble release 版本 `2.1.1-1` | `https://api.github.com/repos/ros/rosdistro/contents/humble/distribution.yaml` |
| 参数声明（59 个键的完整清单）| https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/src/ros2_common.cpp 、 https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/src/amcl_node.cpp |
| lifecycle 基类 / bond / autostart | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/include/beluga_amcl/ros2_common.hpp |
| C++17 / 可执行文件名 / 组件插件名 | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/CMakeLists.txt |
| 依赖与许可声明 | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/package.xml |
| **nav2 vs beluga 参数兼容性表**（本次映射表的上游依据）| https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/docs/ros2-reference.md#compatibility-notes |
| 话题/服务/TF 语义 | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_amcl/README.md |
| 官方 example 的 lifecycle_manager 形态 | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_example/launch/utils/localization_launch.py |
| 参考参数文件（含 beluga 独有键）| https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_example/params/default.ros2.yaml |
| `laser_min/max_range` 的 clamp 语义 | https://github.com/Ekumen-OS/beluga/blob/2.1.1/beluga_ros/include/beluga_ros/laser_scan.hpp |
| nav2_amcl 侧参数名（`save_pose_rate`/`initial_pose.z`/beam skip 四件套确实存在）| https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp |
| Humble 二进制存在（本机实证）| `apt-cache policy ros-humble-beluga-amcl` → `2.1.1-1jammy.20260908.012840`，源 `http://mirrors.aliyun.com/ros2/ubuntu jammy/main` |

**本仓库接线（只动 3 处，见 §3 的三步法）**

| 文件 | 改了什么 |
|---|---|
| `bringup_sim.launch.py` | ① `localization` choices 加 `'beluga'`（**默认仍是 `''`**）；② 新分支 `localization == 'beluga' and lio != 'cartographer'`（与 amcl 分支同 `IfCondition` 形态，include 下面的 launch；传 `map`/`use_sim_time`/`params_file*`/`beluga_params_file`/`initial_pose_x,y,yaw`，**故意不传 `initial_pose_z`**）；③ 独立 `map_server` include 的条件补 `localization != 'beluga'`（amcl/beluga 槽都自带 map_server ⇒ 否则 `/map` 两个发布者 + 两个同名 lifecycle_manager 抢节点） |
| `rm_navigation/launch/localization_beluga_launch.py`（**新**） | `localization_amcl_launch.py` 的平行实现：`nav2_map_server/map_server` + **`beluga_amcl/amcl_node`（`name=amcl`）** + `nav2_lifecycle_manager`（`node_names=['map_server','amcl']`、`autostart`、`use_sim_time`）。含**选槽预检**：没装 `beluga_amcl` 时直接抛错并打印安装命令；本文件**不在生成期**调用 `get_package_share_directory('beluga_amcl')` ⇒ 没装 beluga 也不会把 amcl/其它槽/其它 mode 的 launch 带崩 |
| `rm_navigation/params/nav2_params_sim_beluga.yaml`（**新**） | beluga 专用参数文件（只含 `amcl` 段；`map_server` 段仍来自 `nav2_params_sim_base.yaml`）。**不复用** base 的 amcl 段，理由见下面的"为什么是兄弟文件" |

**契约核对**（与 §5 的四条硬约束；上游代码位置按 2.1.1）

| 契约 | beluga 的行为 | 依据 |
|---|---|---|
| `map→odom` **只有一个发布者** | `tf_broadcast: true` ⇒ 只发 `global_frame_id→odom_frame_id` = `map→odom`；**不碰** `odom→base_link`（那是 LIO 的）| `ros2_common.cpp` 的 TF broadcaster + `amcl_node.cpp:627-635` |
| `map_server` 照常跑 | 本槽自带的 lifecycle_manager 把 `map_server` 与 `amcl` 一起 configure/activate，与 amcl 槽完全一致；独立 map_server include 已排除 beluga | 本文 §1.2 接线表 |
| 有定位器时**不加静态桥** | 静态桥条件仍是 `localization == ''` ⇒ beluga 选了就不会起 | `bringup_sim.launch.py` 的 `icp_frame_bridge_condition` |
| 传感器话题 **BEST_EFFORT** | `/scan` 用 `rclcpp::SensorDataQoS()`（= BEST_EFFORT）；`/map` 用 `KeepLast(1).transient_local().reliable()`（与 nav2 map_server 的 latched 发布端匹配）；`/initialpose` 用 `SystemDefaultsQoS()`（RELIABLE）| `amcl_node.cpp:219-267`、`ros2_common.cpp:475-478` |
| nav2 的 `robot_base_frame` 保持 `base_link_fake` | 与本槽无关（beluga 只做 `map→odom`）；beluga 的 `base_frame_id` 与 amcl 一样是 `base_link` | 参数文件 |

**为什么是"兄弟参数文件"而不是复用 base 的 amcl 段**

两者参数集**不是超集关系**（上游 2.1.1 的兼容性表）：
- nav2 有、beluga 没有：`do_beamskip` / `beam_skip_distance` / `beam_skip_threshold` /
  `beam_skip_error_threshold` / `save_pose_rate` / `initial_pose.z`；
- beluga 有、nav2 没有：`initial_pose.covariance_*` / `spatial_resolution_[x,y,theta]` /
  `selective_resampling` / `execution_policy` / `model_unknown_space` / `only_obstacle_boundaries` /
  `point_cloud_topic` / `initial_pose_topic` / `autostart` / `autostart_delay` / `bond_timeout` / `debug`。

ROS 2 对「参数文件里写了但节点没声明」的键是**静默忽略**（与 §1.1 里"写了是静默失效"同一个坑）
⇒ 复用 base 会同时留下无效键与缺失键（尤其 `initial_pose.z`：beluga 没有 z 键，写了不报错也没作用）。
所以 beluga 用兄弟文件，两份文件各自只写"该实现真的会读"的键。

**参数映射表（本仓库现用值 → beluga）**；"存在"= 同名同义，可直接搬

| nav2_amcl 键（`nav2_params_sim_base.yaml` 的 `amcl` 段） | 本仓库现值 | beluga 2.1.1 | 说明 |
|---|---|---|---|
| `transform_tolerance` | **0.3** | ✅ 存在 | 语义与 nav2 一致：`map→odom` 盖章 = **`scan.header.stamp + transform_tolerance`**（"发到未来"，`amcl_node.cpp:629-634`）；beluga **额外**用它当 `/scan` 的 tf2 MessageFilter 容忍度（`amcl_node.cpp:247,262`）⇒ 同一个键影响两处，取值理由与 §1.1/§九 完全相同 |
| `update_min_d` / `update_min_a` | **0.05 / 0.05** | ✅ 存在 | §九 修复值原样搬运 |
| `recovery_alpha_slow` / `recovery_alpha_fast` | **0.001 / 0.1** | ✅ 存在 | §九 修复值原样搬运（beluga 默认也是 0.0/0.0 ⇒ 必须显式打开，否则跟丢即永久跟丢） |
| `alpha1..alpha5` | 0.2 ×5 | ✅ 存在 | |
| `base_frame_id` / `odom_frame_id` / `global_frame_id` | `base_link` / `odom` / `map` | ✅ 存在 | beluga 默认 `base_footprint` ⇒ **必须显式写** |
| `robot_model_type` | `nav2_amcl::OmniMotionModel` | ✅ 存在（**接受 nav2 插件名**） | beluga 内部换成等价的 Beluga 全向模型 |
| `scan_topic` | `scan` | ✅ 存在 | beluga 默认为空串 → 回落 `scan`；显式写更清楚 |
| `map_topic` | （未写，默认 `map`） | ✅ 存在 | beluga 显式写 `map` |
| `laser_model_type` | `likelihood_field` | ✅ 存在 | beluga 另有 `likelihood_field_prob` |
| `laser_min_range` / `laser_max_range` | `-1.0` / `100.0` | ✅ 存在（**阈值语义不同**） | beluga 是 **clamp**：`min_range=max(scan.range_min, 值)`、`max_range=min(scan.range_max, 值)`（`beluga_ros/include/beluga_ros/laser_scan.hpp:55-61`）⇒ `-1.0` 等价于"用 `/scan` 的 `range_min`"，与 nav2 对负值的处理一致 |
| `max_beams` / `max_particles` / `min_particles` | 60 / 2000 / 500 | ✅ 存在 | |
| `pf_err` / `pf_z` | 0.05 / 0.99 | ✅ 存在 | |
| `resample_interval` | 1 | ✅ 存在 | beluga 声明了整数范围 `[1, INT_MAX]` |
| `sigma_hit` / `z_hit` / `z_max` / `z_rand` / `z_short` / `lambda_short` | 0.2 / 0.5 / 0.05 / 0.5 / 0.05 / 0.1 | ✅ 存在 | `z_rand` 与 `z_hit` 都是 0.5 是 nav2 的默认量级，A/B 时保持不变 |
| `laser_likelihood_max_dist` | 2.0 | ✅ 存在 | |
| `tf_broadcast` | true | ✅ 存在 | 契约关键键（只发 `map→odom`） |
| `set_initial_pose` | true | ✅ 存在 | |
| `initial_pose.x` / `.y` / `.yaw` | 0 / 0 / 0（由 launch 按 world 覆盖） | ✅ 存在 | launch 用 `RewrittenYaml` 全路径覆盖，已实测生效 |
| `always_reset_initial_pose` | false | ✅ 存在 | |
| `first_map_only` | （未写） | ✅ 存在 | beluga 显式写 `false` |
| `use_sim_time` | True | ✅ 存在 | 由 launch 注入 |
| `initial_pose.z` | 0.0 | ❌ **不存在** | beluga 是 2D 节点（`Sophus::SE2d`），没有 z 键；`localization_beluga_launch.py` 因此**不声明/不注入** `initial_pose_z`（与 amcl 那份的唯一参数差异） |
| `save_pose_rate` | 0.5 | ❌ 不存在 | nav2 用它把最后位姿写回参数服务器；beluga 无此功能 ⇒ 不写 |
| `do_beamskip` / `beam_skip_distance` / `beam_skip_threshold` / `beam_skip_error_threshold` | false / 0.5 / 0.3 / 0.9 | ❌ **不存在** | beluga **不支持 beam skipping**（上游兼容性表明确写"Beluga AMCL does not support beam skipping"）⇒ 4 个键全不写；`do_beamskip` 本来就是 false，**行为差异可忽略** |
| — | — | ➕ `initial_pose.covariance_[x,y,xy,yaw,xyaw,yyaw]` | nav2 **忽略**初值协方差（本仓库 `issues_and_findings.md #13`：`/amcl_pose` 协方差≈0）⇒ nav2 初值等价于"零散布单点"。为 A/B 公平，beluga 侧用其默认量级 `1e-6` 复刻该语义；**要抗人工摆放误差就放大**（上游 example 用 `0.25/0.25/0.0685`）|
| — | — | ➕ `initial_pose_topic` | 默认 `initialpose`（nav2 只能靠 remap）⇒ 显式写，保证 RViz `2D Pose Estimate` / `ros2 topic pub` 打到同一个话题 |
| — | — | ➕ `spatial_resolution_[x,y,theta]` | beluga 的 KLD 空间分桶分辨率（默认 0.5 / 0.5 / 10°），显式写出便于调参 |
| — | — | ➕ `execution_policy` | `seq`（默认，单线程）| `par`（多线程，吃 CPU 换吞吐）—— CPU 吃紧时的第一个旋钮 |
| — | — | ➕ `selective_resampling` | 默认 `false`；`true` = `N_eff < N/2` 才重采样（ROS 2 版 nav2 没有这个特性）|
| — | — | ➕ `model_unknown_space` | 显式 `false` = 把未知栅格当自由空间（= nav2 行为）|
| — | — | ➕ `only_obstacle_boundaries` | 显式 `true`（上游默认）：只把障碍**边界**当障碍。这是 beluga 与 nav2 似然场构造的**潜在差异点**，见"要盯什么" |
| — | — | ➕ `point_cloud_topic` | 不用（我们用 `/scan`，与 amcl 同源资产）；它与 `scan_topic` **互斥**，两个都非空会直接抛异常 |
| — | — | ➕ `autostart` / `autostart_delay` | 默认 `false` ⇒ 由 lifecycle_manager 管（与 amcl 一致）；`true` 可免 manager（本仓库不用）|
| — | — | ➕ `bond_timeout` | 默认 4.0，与 lifecycle_manager 心跳配合 |
| — | — | ➕ `debug` | 默认 `false`；`true` 会多发 `/likelihood_field` 并降性能 |

**话题/服务差异（A/B 时必须知道）**

| 项 | nav2_amcl | beluga_amcl |
|---|---|---|
| 位姿 | `/amcl_pose`（`PoseWithCovarianceStamped`）| **`/pose`**（同上类型；节点名 `amcl`、话题名相对名 `pose`）——**不是参数**，要改只能 remap；本仓库**故意不 remap**（保持上游接口直观），A/B 时看 `/pose` |
| 粒子云 | `/particle_cloud` | `/particle_cloud`（同）|
| 粒子可视化 | `/particle_markers` | `/particle_markers`（同）|
| 服务 | `reinitialize_global_localization` / `request_nomotion_update` | 同名同义 |
| `map→odom` 发布时机 | 每次 `/scan` 回调（含"本帧没更新也重发"）| **每次 `/scan` 回调**（`amcl_node.cpp:627-635`，也含重发）⇒ **速率 ≈ `/scan` 速率**，不是固定 50 Hz |
| `/scan` 订阅 | `SensorDataQoS`（BEST_EFFORT）| 同 |

**安装路线（本仓库采用 A）**

```bash
# (A) 二进制（Humble 官方包，推荐；不需要源码构建、不需要 vendoring）
sudo apt update && sudo apt install ros-humble-beluga-amcl      # 连带 beluga / beluga_ros
ros2 pkg prefix beluga_amcl                                     # 自检：应打印 /opt/ros/humble
```
```bash
# (B) 源码（仅当要改 beluga 本体 / 要 pin 一个更新的 commit）
#   放到 colcon **会**构建的位置：src/rm_localization/beluga/
#   ⚠️ third_party/ 下有 COLCON_IGNORE ⇒ 放那里 colcon 不会构建
git -C src/rm_localization clone https://github.com/Ekumen-OS/beluga.git
git -C src/rm_localization/beluga checkout b06f9060ed6d38d3559b4d05368f5fbe74c6594f   # = 2.1.1
sudo apt install libeigen3-dev libhdf5-dev librange-v3-dev libtbb-dev ros-humble-sophus
colcon build --symlink-install --packages-up-to beluga_amcl
```
- **本次采用的路线 = (A)**。理由：Humble 有官方二进制且版本就是我们要核对的 2.1.1
  （= `b06f906…`），`ldd` 显示运行期依赖全部可满足，`ros2 pkg prefix` 可解析；源码构建要多装 5 个依赖、
  且会把一个 C++ 库塞进我们的 workspace（升级/回退成本都更高）。**没有 vendor 任何代码**，因此
  **没有新增子模块、没有 pin 到仓库里的 commit**（pin 的是 apt 版本 2.1.1 = 上游 commit `b06f906…`，
  记录在本文）。
- ⚠️ 本沙箱里 `sudo` 被禁（"no new privileges"），所以**没有真的装到 `/opt/ros/humble`**：
  静态验证是用 `apt-get download` 把 3 个 deb 下下来、`dpkg -x` 解到
  `.tmp_cache/beluga_overlay/opt/ros/humble` 再 source 该 overlay 做的（`ros2 pkg prefix beluga_amcl` 解析成功、
  `ldd` 无缺失、launch 预检通过）。**在真机上请走上面的 `apt install`**（或同样用 overlay）。

**运行 / 检查**

```bash
# 新文件必须先 build（本仓两个包；beluga 是 apt 包，colcon 会当作"workspace 外"忽略）
colcon build --symlink-install --packages-select rm_nav_bringup rm_navigation
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=beluga nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True

# 健康三连（beluga 的"位姿话题"是 /pose，不是 /amcl_pose）
ros2 run tf2_ros tf2_echo map odom            # 台阶式跳变？长期不更新？
ros2 topic hz /tf                             # map→odom ≈ /scan 速率（10 Hz 量级），不是 50 Hz
ros2 topic echo /pose --once                  # 位姿 + 真实协方差（nav2 的 /amcl_pose 协方差≈0）
ros2 topic echo /particle_cloud --once | head -5   # 粒子云（可选）
ros2 topic info /scan -v                      # 必须仍是 BEST_EFFORT
# 若 30 s 内没有 map→odom：给初值（RMUL2026 map 系原点 = 出生点）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**A/B 协议（照 §4；amcl 与 beluga 必须分两次跑）**

```bash
# ① 基线
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=amcl nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
python3 tools/scripts/diag/record_tf_monotonic.py
# ② beluga（同上，只把 localization:=amcl 换成 localization:=beluga）
# ③ 比：map→odom 跳变/单调性、位姿话题频率、P0 回归 PASS/到达误差/用时、CPU/RTF
```
注意 beluga 的**触发式更新**与 nav2 amcl 同为"阈值触发"（`update_min_d/a`），所以「车不动 ⇒ `/pose`
不刷新」是**正常现象**（同 `docs/mapping/README.md` 里对 `/amcl_pose` 的说明）；`map→odom` 仍随每帧
`/scan` 重发。

**要盯什么**

1. **`map→odom` 的速率与连续性**：beluga 只在**收到 `/scan` 时**发 TF ⇒ 期望 ~`/scan` 速率；若
   远低于 `/scan` 速率，说明 `transform_tolerance`（MessageFilter 容忍度）把扫描丢在门外了；
2. **`ExtrapolationException`**（nav2 消费者报"future"）：本仓库 amcl 的 `transform_tolerance 0.3`
   已够用，beluga 盖戳公式**与 nav2 逐字相同**（`scan.stamp + tolerance`）⇒ 若仍报，先查 TF 速率是否被
   算力拖慢（beluga 单线程 `execution_policy: seq`，2000 粒子上限）；
3. **`only_obstacle_boundaries`（beluga 默认 true）**：似然场只认障碍**边界**，在"薄墙/单像素墙"
   地图上可能与 nav2 amcl 表现不同 ⇒ A/B 时若 beluga 明显更差，第一件事就是把它设 `false` 再试；
4. **CPU / `Control loop missed its desired rate`**：旋钮顺序 ① `execution_policy: par`
   （多线程）→ ② `max_particles 2000→1000` → ③ `max_beams 60→30`；
5. **`/initialpose` 的语义**：`set_initial_pose: true` + launch 注入初值（RMUL2026 = `(0,0,0)`）；
   运行中 RViz `2D Pose Estimate` 仍可覆盖（同 nav2）；`initial_pose.covariance_*` 只有 beluga 读，
   本仓库刻意用 `1e-6` 复刻 nav2 的"零散布"语义（见映射表）。

**回退**

- 槽位级：`localization:=` 换回 `amcl`（或留空 + LIO 当绝对定位 + 静态桥）——**beluga 分支是纯增量**，
  不选它就完全不生效（连 `beluga_amcl` 没装都不影响，已实测 `--show-args` 与本文件之外的槽位）；
- 代码级：`git revert <本次 commit>`（3 个文件：bringup 的 choices/分支/map_server 条件 +
  新增 launch + 新增 params）；
- 卸载：`sudo apt remove ros-humble-beluga-amcl`（连带 beluga/beluga_ros）。

**⚠️ 未验证清单（本次只做静态验证：`py_compile` / `yaml.safe_load` / `--show-args` / colcon / 逐条
`IfCondition` 真值表；**没有启动 Gazebo、nav2 或 beluga 节点**）**

1. **整栈运行时的定位精度、收敛性、CPU**：全部未测（本机不启仿真）；
2. **`map→odom` 实测速率/连续性**：按代码应为 `/scan` 速率，未实跑确认；
3. **beluga 与 nav2 amcl 的 A/B 结论**：`--goal -1.0 2.0` 回归、到达误差、用时、恢复次数**均未跑**；
4. **`autostart` 路径**：本仓库走 lifecycle_manager（`autostart: false`），beluga 自带的 `autostart: true`
   免 manager 路径未试；
5. **`use_composition: true` 路径**：`beluga_amcl::AmclNode` 组件与 `nav2_container` 的组合未试
   （默认 false；amcl 槽的这条路同样没在 bringup 里接容器）；
6. **`only_obstacle_boundaries` / `model_unknown_space` / `selective_resampling` / `execution_policy: par`
   的实际影响**：未调、未测；
7. **`localization:=beluga` + `lio:=cartographer` 这个非法组合**：与 amcl 一样会「两个槽都不起 + 无
   map_server」（`lio==cartographer` 时 beluga 分支条件为假，而独立 map_server 又被排除）——**这是
   amcl 早就有的同款行为**，本次只做"与 amcl 对齐"，未修；
8. **apt 安装本身**：本沙箱 `sudo` 被禁，只验证了「deb 解包 + overlay source 后包可解析、依赖无缺失」，
   没有在真机 `/opt/ros/humble` 上执行 `apt install`。

## 2. 待补入口（**已登记、未实现**）

| 计划槽位值 | 用什么 | 依赖 / 资产 | 落地要点 | 估时 |
|---|---|---|---|---|
| `scan_context` | Scan Context **全局检索** + ICP/GICP **精配准**（两级） | 需引入 Scan Context 实现 + 用 PCD 建描述子库 | 解决"**车随便摆 / 被搬动**"（ICP 类天生初值敏感）；检索出粗位姿 → 现有精配准 | 2~3 天 |
| `fastlio_loc` | LIO + 先验 PCD 做配准得 `map→odom`（一体化配方） | PCD 已有 | 与我们 `lio_tf_adapter` 有职责重叠 ⇒ 作为**对照实现** | 1~2 天 |
| `teaserpp` | TEASER++ 无初值全局配准 | 需引入 TEASER++ + 特征 | 远期；开销大 | 远期 |

> ✅ **`beluga` 已于 2026-10-05 落地**（`localization:=beluga`，装 `ros-humble-beluga-amcl` 2.1.1 即可，
> 无需源码构建）⇒ 从上表移出，详见 §1.2。它**不新增算法包**、不 vendor 代码：走的正是原计划那条
> "沿用 AMCL 参数语义做 A/B"的路（同名参数 + 兄弟参数文件），只是把"接口兼容"落实成了逐键映射表。

> ✅ **`gicp` 已于 2026-10-05 落地**（独立包 `gicp_registration`，槽位值 `localization:=gicp`）⇒ 从上表移出，
> 详见 §1.1。走的**不是**原计划"在 `icp_registration` 内换后端"那条路，而是**新包**：
> 理由是 icp 槽要保留 ICP 原样做三方 A/B（AMCL / ICP / GICP），两套后端各占一个槽位互不干扰。
> 若以后要引入 small_gicp/fast_gicp（更快），换的只是 `gicp_registration` 内部的配准后端。

## 3. 新增一个重定位槽的**标准三步**（与 planner/controller 槽同构）

1. `bringup_sim.launch.py` 的 `localization` `choices` 加一项 + 写对应 launch 分支（Node + 参数文件）；
2. 参数文件放 `src/rm_localization/<pkg>/config/`，并保证**只发 `map→odom`**（绝不碰 `odom→base_link`）；
3. `colcon build --symlink-install --packages-select <pkg>`（**新文件/新包必须 build**，否则 install 里没有 → 启动报找不到文件）。

## 4. A/B 协议（怎么比才公平）

- 同一次运行里**比不了**（`map→odom` 只能一个发布者）⇒ **分开跑**，保证 **同一 world / 同一目标 / 同一路线 / 同一 `nav`+`planner`**；
- 指标：
  1. `python3 tools/scripts/diag/record_tf_monotonic.py` → `map→odom` 的**跳变次数/幅度/单调性**；
  2. 位姿话题频率（amcl/beluga 是 `/amcl_pose` / **`/pose`**，或对应话题）；
  3. **P0 回归 PASS/FAIL + 到达误差 + 用时**（`tools/scripts/regress/nav_smoke_regression.py`）；
  4. CPU / RTF。
- 记录：`algorithm_matrix.md §四` 加一行 + 本表状态列更新。

## 5. 契约与经验提醒

1. **AMCL 是阈值触发的离散修正器**（`update_min_d/a` 决定更新时机）；**slam_toolbox 定位 / GICP / NDT 是连续型**（每帧匹配、`transform_publish_period` 可到 0.02 s = 50 Hz）⇒ **高速下连续型更"跟手"**（详见工单 §K）；
2. **`localization:=icp/gicp` 类方法初值敏感** ⇒ 常需"固定起点"或"上次位姿"作为初值；要"随便摆"就得配全局检索（`scan_context`）或粒子类（`amcl`/`beluga`）；
3. **恢复行为直接发 `/cmd_vel`**（绕过 velocity_smoother）；**`spin_speed != 0` 时 `fake_vel_transform` 会替换 `angular.z`** ⇒ 高速自转下定位更容易被拖偏，A/B 时把 `spin_speed` 固定为 0.0；
4. **切换重定位后先看 `map→odom`**：`ros2 run tf2_ros tf2_echo map odom`（是否台阶式跳变/是否长期不更新）。

---

## 6. 决策记录（2026-10-05）

| 决定 | 内容 | 理由 / 备注 |
|---|---|---|
| **先试 `icp`** | 用现成入口跑 `localization:=icp`（资产与接线已核：节点 `icp_registration_node`、参数 `icp_registration_sim.yaml`、资产 `PCD/RMUL2026.pcd`） | 与刚调优的 AMCL 直接 A/B，看 `map→odom` 跳变与到达精度 |
| **`slam_toolbox` 暂缓** | **保留入口与代码，暂不实现/不试** | 用户判断：项目偏老、担心以后跟不上赛场。**记录一条客观补充**：它的**更新率并不低**（`transform_publish_period` 可到 0.02 s = 50 Hz），真正的短板是 ① 要长期维护一份 `.posegraph` 资产 ② **全局重定位弱**（依赖初值）③ 维护节奏慢。⇒ 若以后要"随便摆 + 高更新率"，**优先考虑 `scan_context`（全局检索）+ GICP（连续型现代实现）** 这条组合，而不是回头用 slam_toolbox |
| **`gicp` 已落地**（2026-10-05） | 新包 `src/rm_localization/gicp_registration/` + `localization:=gicp` 槽（choices 加一项 + launch 分支，与 icp 同 `IfCondition` 形态、同资产 `PCD/<world>.pcd`） | 与"用现代实现"的取向一致；**独立包**而非改 `icp_registration` ⇒ icp 后端保持原样，形成 AMCL / ICP / GICP 三方 A/B。细节、参数核实、资产实测见 §1.1 |
| **`beluga` 已落地**（2026-10-05） | 新槽位值 `localization:=beluga` + 新 launch `rm_navigation/launch/localization_beluga_launch.py` + 兄弟参数文件 `rm_navigation/params/nav2_params_sim_beluga.yaml`；**装 apt 包 `ros-humble-beluga-amcl` 2.1.1（= 上游 commit `b06f906…`，Apache-2.0），不 vendor 代码** | 取向是"用现代实现做 A/B"：beluga 与 nav2 amcl 同名同义参数 + 同 lifecycle 形态 ⇒ 是**最干净的一对对照**（连 §九 的 5 个调参键都同名）。参数集非超集 ⇒ 用兄弟文件而不是复用 base（静默失效键问题）。细节/映射表/未验证项见 §1.2 |
| **goal checker 改位置-only**（2026-10-05） | 四个 controller 槽文件（`nav2_params_sim_controller_{rpp,dwb,teb,mppi}.yaml`）的 `general_goal_checker.plugin`：`SimpleGoalChecker` → **`PositionGoalChecker`**，只留 `xy_goal_tolerance: 0.25`（删掉 `yaw_goal_tolerance`），别名 `general_goal_checker` 与 `stateful: True` 不变 | 全向车（mecanum）**没有"车头"概念** ⇒ 终点只约束位置；顺带消除"位置到了但朝向过不了 → progress checker 判失败 → 反复恢复 → ABORT"这一失败模式。已核已装 nav2 **1.1.20**：`share/nav2_controller/plugins.xml` 有该类、`libposition_goal_checker.so` 导出其符号，且该插件**只声明** `xy_goal_tolerance` / `stateful`（`yaw_goal_tolerance`、`path_length_tolerance` 在 1.1.20 的该插件里不存在 ⇒ 写了是静默失效，故删除） |


---

## 7. 验收记录：`localization:=gicp`（2026-10-05，用户验收"效果可以接受"）

**验收条件**（一次完整运行，栈：`world:=RMUL2026 mode:=nav lio:=fastlio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True`）：

| 判据 | 实测 | 结论 |
|---|---|---|
| `/tf` 速率 | **92~94 Hz（聚合）** = `map→odom` 50 + `base_link_fake` ~20 + `odom→base_link` ~10 + 静态/wheel | ✅ `map→odom` 满速（多线程解耦生效；改前 2 Hz 级） |
| `~/fitness_score` | **0.0023 m²**（RMS ≈ 4.8 cm） | ✅ 与单节点合成测 0.00123 同量级 |
| P0 回归 `--goal -1.0 2.0` | **PASS / SUCCEEDED / 用时 3.2 s / `recoveries=0` / 轨迹 2.45 m** | ✅ 对比修复前 28 s / 23 次恢复 / ABORT ⇒ abort 风暴消失 |
| 命令链 | `nav=(0.72,0.78)` → `smooth` → `chassis` **四跳一致**，`spin_speed=0.0` 直通 | ✅ 无 Spin/Backup 介入 |

**通的四关（修复链条，供以后复用）**：
1. **口径**：两级下采样 图 0.10 m / 点云 0.05 m（COD 2025 `small_gicp_relocalization` 的 `global_leaf_size`/`registered_leaf_size` 配方）+ `max_correspondence_distance 1.5`（≈ `max_dist_sq 2.5`）；
2. **盖戳**：TF 与 `~/pose` 用 `now + tf_lookahead_sec(0.45)`（≈ AMCL `transform_tolerance`；规则 = 消费端最大 tolerance + 发布周期 + 最坏掉帧余量）；
3. **调度**：`MultiThreadedExecutor` + TF/状态定时器**独立 callback group**（align ~350 ms 不再饿死 50 Hz TF）；跨组共享状态加锁，健康话题**故意不持锁发布**；
4. **初值**：默认 `use_initial_pose: true` + `initial_pose [0,0,0]` ⇒ **开机即发**；真正的"不发 TF"只在 `use_initial_pose: true` **且**无 `odom→base` TF **且**无 `/initialpose` 时出现（最后保护分支，不是常见路径）。

**已知遗留（均为"可接受/待办"，非阻塞）**：
1. **工具 settle 阈值偏严**：车静止时 30 s 漂 3.4 cm / 0.0139 rad，略超工具默认（0.02 m / 0.01 rad）⇒ 该值应视为**GICP 噪声底**参考信息；如需消除每次 WARNING，可放宽 `TH.settle_dxy→0.05` / `settle_dyaw→0.02`（一行、可回退）。
2. **`Control loop missed its desired rate of 30 Hz`**：属"总算力不够"（MPPI 30 Hz + GICP 10 Hz + Gazebo + LIO + RViz，RTF≈0.72），**解耦只治"TF 被饿死"、不治这一条**；真降 CPU 靠 leaf 阶梯（`voxel_leaf_size_scan 0.05→0.10` → `maximum_iterations 16→8` → 地图 leaf 回 0.25）。
3. **`maximum_iterations 32→16` 不降 CPU**（实测 1/4/16/32/64 迭代 align 363/475/486/471/427 ms，非单调、fitness 全同）⇒ 跟踪场景下该上限不生效，成本由固定开销主导。
4. **资产**：`PCD/RMUC.pcd` 退化（65 万点挤在 ±1 cm，0.10 m 体素后 8 点）⇒ `world:=RMUC` 用 icp/gicp 会启动即报错（设计如此）；`RMUL.pcd @0.10 = 97642` 点；`RMUL2026.pcd` 仅 53164 点 ⇒ 12450 target 已近上限（**要再提精度须先有更密的先验 PCD**，属建图侧）。
5. **三方对比暂缓（用户决定）**：`amcl` / `icp` / `gicp` 同一 world、同一目标 `--goal -1.0 2.0`、同一 `nav:=mppi planner:=smac2d spin_speed:=0.0`，各跑一次并把 `.tmp_bags/regress_<ts>.json` 聚合进 `docs/algorithm_matrix.md §四`。
