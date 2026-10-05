# 重定位槽位登记表（localization slots）

> 入口：`bringup_sim.launch.py` 的 `localization` 参数（**仅 `mode:=nav` 生效**）
> 当前 choices：`['', 'amcl', 'slam_toolbox', 'icp', 'gicp', 'cartographer']`
> **统一契约**：重定位槽**只负责发布 `map→odom`**；`odom→base_link` 由 LIO（+ `lio_tf_adapter`）负责；Nav2 用 `base_link_fake`。
> **`map→odom` 只能有一个发布者** ⇒ 重定位槽之间、以及与 `mapper:=*` 之间必须互斥。

## 1. 已可用（现成入口）

| 槽位值 | 节点 / 包 | 参数文件 | 需要的资产（现状） |
|---|---|---|---|
| `amcl` | `nav2_amcl` | `rm_navigation/params/nav2_params_sim_base.yaml`（已调：`transform_tolerance 0.3` / `update_min_d,a 0.05` / `recovery_alpha_*` 已打开，见工单 §K） | 2D 栅格图 `rm_nav_bringup/map/RMUL2026.pgm|.yaml` ✅ |
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

## 2. 待补入口（**已登记、未实现**）

| 计划槽位值 | 用什么 | 依赖 / 资产 | 落地要点 | 估时 |
|---|---|---|---|---|
| `beluga` | `beluga_amcl`（AMCL 的现代化实现，接口兼容） | 需装 `beluga`/`beluga_amcl`；2D 栅格图已有 | 新增槽位值 + 参数文件（可先沿用 AMCL 参数语义做 A/B） | 0.5~1 天 |
| `scan_context` | Scan Context **全局检索** + ICP/GICP **精配准**（两级） | 需引入 Scan Context 实现 + 用 PCD 建描述子库 | 解决"**车随便摆 / 被搬动**"（ICP 类天生初值敏感）；检索出粗位姿 → 现有精配准 | 2~3 天 |
| `fastlio_loc` | LIO + 先验 PCD 做配准得 `map→odom`（一体化配方） | PCD 已有 | 与我们 `lio_tf_adapter` 有职责重叠 ⇒ 作为**对照实现** | 1~2 天 |
| `teaserpp` | TEASER++ 无初值全局配准 | 需引入 TEASER++ + 特征 | 远期；开销大 | 远期 |

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
  2. 位姿话题频率（`ros2 topic hz /amcl_pose` 或对应话题）；
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
| **goal checker 改位置-only**（2026-10-05） | 四个 controller 槽文件（`nav2_params_sim_controller_{rpp,dwb,teb,mppi}.yaml`）的 `general_goal_checker.plugin`：`SimpleGoalChecker` → **`PositionGoalChecker`**，只留 `xy_goal_tolerance: 0.25`（删掉 `yaw_goal_tolerance`），别名 `general_goal_checker` 与 `stateful: True` 不变 | 全向车（mecanum）**没有"车头"概念** ⇒ 终点只约束位置；顺带消除"位置到了但朝向过不了 → progress checker 判失败 → 反复恢复 → ABORT"这一失败模式。已核已装 nav2 **1.1.20**：`share/nav2_controller/plugins.xml` 有该类、`libposition_goal_checker.so` 导出其符号，且该插件**只声明** `xy_goal_tolerance` / `stateful`（`yaw_goal_tolerance`、`path_length_tolerance` 在 1.1.20 的该插件里不存在 ⇒ 写了是静默失效，故删除） |

