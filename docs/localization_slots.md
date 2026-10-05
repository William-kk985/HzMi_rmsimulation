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
| `gicp` | `gicp_registration/gicp_registration_node`（**我们自己的包**，2026-10-05 新增） | `gicp_registration/config/gicp_registration_sim.yaml`（含 `pcd_path`） | 先验点云 `rm_nav_bringup/PCD/<world>.pcd` ✅（RMUL2026 实测可用；**必须有初值**，见 §1.1） |
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
- **初值（硬要求）**：`/initialpose`（RELIABLE，RViz 2D Pose Estimate，语义 = **map 系下机器人位姿**，
  与 AMCL 一致）或参数 `initial_pose`（默认 `[0,0,0]`，3 或 6 元）。
  **没有初值就不发 TF**（每 2 s 日志说明一次）——不发比发错的更安全；
  `use_initial_pose: false` 则从恒等 `map→odom` 起步（= 把 LIO 当绝对位姿，`/initialpose` 仍有效）。
  RMUL2026 的 map 系 = 出生点相对系 ⇒ 出生即 `(0,0,0)`；若换回 `map/RMUL2026_world_backup.*` 那类
  世界系栅格图，出生点应给 `(4.3, 3.35, 0)`（依据见 `bringup_sim.launch.py` 里 `amcl_init_*` 的注释）。
- **每帧行为**：点云（SensorDataQoS / BEST_EFFORT，KEEP_LAST(1)）→ 去 NaN → 体素下采样 →
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
- **资产现状（2026-10-05 实测：节点启动日志 + 独立 numpy 解析互证）**：

  | PCD | 原始点 | 去 NaN | 0.25 m 体素后 | 包围盒（体素后） | 可用 |
  |---|---|---|---|---|---|
  | `RMUL2026.pcd` | 53164 | 53164 | **2438** | x[-1.86,10.04] y[-2.81,5.31] z[-0.35,0.11] | ✅ |
  | `RMUL.pcd` | 1589841 | 1589841 | 15506 | x[-4.94,10.36] y[-4.95,7.56] z[-0.32,14.06] | ✅ |
  | `RMUC.pcd` | 649995 | 649995 | **8** | 全部挤在原点 ±1 cm | ❌ **退化资产** |

  ⇒ `RMUC.pcd` 的 x/y/z 全落在 3 cm 盒子里（换 reader/独立解析结论相同），**不能用于定位**；
  节点启动即 `ERROR` 退出并打出点数与包围盒（有意为之：把"跑起来但定位是垃圾"的静默失败变成显式失败）。
- **已知限制（本次未实测项）**：① **运行时收敛性与 CPU 未实测**（本机不启动仿真）——
  `voxel_leaf_size`（0.25）与 `maximum_iterations`（32）是 CPU/精度旋钮，若跟不上再调；
  ② `fitness_score_warn / max_fitness_score`（0.05 / 0.3 m²）是按 PCL 语义给的**首跑起始值**，
  必须按实测量级收紧；③ 与 icp 一样**初值敏感**：给错初值会静默收敛到局部极小，
  只能靠 `~/fitness_score` + `~/converged` 发现 ⇒ 要"随便摆"仍需 `scan_context` 类全局检索。

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
ros2 topic echo /gicp_registration/converged         # false ⇒ 本帧未采纳/评分超 warn
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

