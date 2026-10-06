# 重定位槽位登记表（localization slots）

> 入口：`bringup_sim.launch.py` 的 `localization` 参数（**仅 `mode:=nav` 生效**）
> 当前 choices：`['', 'amcl', 'beluga', 'slam_toolbox', 'icp', 'gicp', 'small_gicp', 'cartographer']`
> （`ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args` 能直接看到这行说明）
> **统一契约**：重定位槽**只负责发布 `map→odom`**；`odom→base_link` 由 LIO（+ `lio_tf_adapter`）负责；Nav2 用 `base_link_fake`。
> **`map→odom` 只能有一个发布者** ⇒ 重定位槽之间、以及与 `mapper:=*` 之间必须互斥。

> 🔗 相关：`docs/path_clearance_and_contact.md`（2026-10-06）——
> 「贴膨胀边 / 撞进去 / 定位跟着坏」的实测归因：撞的**不是地图上的墙**，而是**被 linefit 去地面删掉的
> 坡道面**（静态图判自由而场地高度 >0.10 m 的格占自由区 51.6%）；GICP 判据没坏，
> 坏在**上游 LIO 被坡脚冲击打散**（实测 79~90 m/s²），以及**停发 `map→odom` 之后 nav2 仍在开车**
> （实测盲开 110 m、撞到 474 m/s²）。本文件的「`map→odom` 单一发布者」契约在那份报告里被逐条核对过。

## 1. 槽位一览（**一个槽位值一行**，选一个就跑）

> **选槽位的人只需要看这张表**：一个槽位值 = 一个重定位器；"它内部怎么实现、要不要额外配参数"一律不用管
> （`icp` / `gicp` / `small_gicp` 是**三个并列的值**，各自一条命令 —— 见下表与本节末尾的实现细节脚注）。
>
> 命令列 = `world:=RMUL2026` 下的**完整启动命令**（最小形态：`nav`/`planner` 走默认值）；
> 要 A/B 就在后面追加同一组公共选项 `nav:=mppi planner:=smac2d spin_speed:=0.0`（几路必须一致，见 §4）。

| 槽位值 | 一句话是什么 | 需要的资产 | 运行命令 | 状态 | 已知限制 |
|---|---|---|---|---|---|
| `amcl` | `nav2_amcl` 粒子滤波（2D 栅格图上定位；**阈值触发**的离散修正器） | 2D 栅格图 `map/RMUL2026.pgm`+`.yaml` ✅（RMUC/RMUL 也有） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=amcl` | ✅ 已验收（2026-10-05 五路对照：P0 回归 PASS / 3.2 s / `recoveries=0`） | 离散修正（`update_min_d,a` 决定何时更新）⇒ 高速下不如连续型跟手；需初值（launch 已按 world 自动注入出生点，见 `amcl_init_*` 注释）。参数在 `rm_navigation/params/nav2_params_sim_base.yaml`（已调 `transform_tolerance 0.3`） |
| `beluga` | `beluga_amcl/amcl_node` —— **AMCL 的现代实现**（同名参数、同 lifecycle 形态） | 同 `amcl` 的 2D 栅格图 ✅ **+ 需装 apt 包 `ros-humble-beluga-amcl`**（2.1.1，不 vendor 代码） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=beluga` | ✅ 已验收（同日五路对照：P0 回归 PASS / 3.3 s / `recoveries=0`） | 同 AMCL 的离散修正特性；`laser_min_range` 必须 **≥0**（与 nav2 的 `-1.0` 哨兵值不同，写错 = **启动即崩**，见 §1.2.0）；Ctrl-C 收尾会 `exit code -6`（只在收尾，不影响运行，见 §1.2.1）；`/scan` 用 BEST_EFFORT。参数在 `rm_navigation/params/nav2_params_sim_beluga.yaml`（兄弟文件，非复用 base） |
| `slam_toolbox` | `slam_toolbox/localization_slam_toolbox_node` —— 序列化位姿图上的扫描匹配定位（**连续型**） | **`map/RMUL2026.posegraph`(+`.data`) ❌ 缺**（RMUC/RMUL 有）⇒ 需先建图并 `serialize_map` | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=slam_toolbox` | ⏸ **待实跑**（缺资产；入口与代码保留） | 要长期维护一份 `.posegraph`；**全局重定位弱**（依赖初值）；上游维护节奏慢（§6 决策记录）。参数在 `slam_toolbox/config/mapper_params_localization_sim.yaml` + launch 注入 `map_file_name=map/<world>` |
| `icp` | `icp_registration/icp_registration_node` —— **PCL ICP** 逐帧精配准（点到点） | 先验点云 `PCD/<world>.pcd` ✅（RMUL2026 有；⚠️ **`RMUC.pcd` 是退化资产**，见 §1.1 资产表） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=icp` | ✅ 已验收（同日五路对照：P0 回归 PASS / 3.2 s / `recoveries=0`；⚠️ 那次初值恰好正确 —— `algorithm_matrix.md` §9.2.4 第 5 条） | **初值敏感**：需 `/initialpose`（RViz 2D Pose Estimate）或 `initial_pose` 参数；给错初值会**静默**收敛到局部极小（只能靠 `map→odom` 是否合理发现）；点到点代价 ⇒ 精度/鲁棒性低于 GICP 族。参数在 `icp_registration/config/icp_registration_sim.yaml` |
| `gicp` | `gicp_registration/gicp_registration_node` —— **GICP** 逐帧精配准（面元协方差，比 ICP 稳） | 先验点云 `PCD/<world>.pcd` ✅（RMUL2026 实测：0.10 m leaf 后 target ≈1.2e4 点） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=gicp` | ✅ 已验收（用户验收"效果可以接受" §7 + 同日五路对照 P0 回归 PASS / 3.3 s / `recoveries=0`） | **必须有初值**（`/initialpose` 或 `initial_pose`；**没有初值就不发 TF**，每 2 s 日志说明一次）；与 `icp` 一样初值敏感；`fitness_score_warn / max_fitness_score`（0.05 / 0.3 m²）是**首跑起始值**，要按实测量级重定。参数在 `gicp_registration/config/gicp_registration_sim.yaml` |
| `small_gicp` | **`small_gicp` 库的 GICP** 精配准（同一套 GICP 数学、另一个实现，多线程） | 先验点云 `PCD/<world>.pcd` ✅（与 `gicp` **完全同一份**资产） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=small_gicp` | ⚠️ **待实跑**（本槽位是新增入口，尚未整栈验收）；**实测数据已有**：整栈 P0 回归 PASS / 3.4 s / `recoveries=0` | 与 `gicp` 同契约、同限制（**必须有初值**；初值敏感）；单帧更省 CPU（整栈 align 2.9 ms vs 15.7 ms），代价是 `~/converged` 有帧级抖动（89~92%，**位姿不受影响**）—— 并列取舍见本节末脚注 |
| `cartographer` | `cartographer_node` **纯定位**（加载 `.pbstream`，frozen state） | `map/RMUL2026.pbstream` ✅ | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=cartographer` | ⚠️ **待实跑**（入口与资产就绪；本轮五路对照未含它） | 栅格另发 `/cartographer_map`（`/map` 留给 `map_server` 的先验图）；`lio:=cartographer` 的**全包形态**不走本槽（那时 `localization` 留空）；配置在 cartographer 包 |
| `''`（留空） | **不回退到任何重定位**：把 LIO 直接当绝对定位，再用静态桥补帧 | —（不需要地图 / PCD 资产） | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio localization:=''` | ✅ 回退路径（长期在用；`mode:=mapping`/`slam_nav` 也是不用重定位） | **没有真正的重定位**：`map≡camera_init` 恒等静态桥 ⇒ LIO 漂多少就是多少，无回环、无先验图约束；`lio:=cartographer` 时不用它，`lio:=none` 时也无效 |

**三个重定位器的命令只差 `localization:=` 一个词**（复制粘贴即可；`world` 换成 RMUC/RMUL 同理）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio \
  localization:=icp        nav:=mppi planner:=smac2d spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio \
  localization:=gicp       nav:=mppi planner:=smac2d spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio \
  localization:=small_gicp nav:=mppi planner:=smac2d spin_speed:=0.0
```

> ### ※ 实现细节（槽位用户无需关心）
>
> `gicp` 与 `small_gicp` 共用**同一个节点**（`gicp_registration/gicp_registration_node`）、
> 同一套 GICP 数学、**同一份参数文件**（`gicp_registration/config/gicp_registration_sim.yaml`）、
> 同一份资产与初值契约，只有**库**不同；节点内部有一个 `backend`（`pcl` | `small_gicp`）开关。
> **这个开关由 launch 各自的槽位分支注入**：`localization:=gicp` → 不注入（= 文件默认 `pcl`）；
> `localization:=small_gicp` → launch 注入 `backend: "small_gicp"`。
> ⇒ **选槽位的人不用改任何配置文件，也不用 `-p backend:=...`**。
> 之所以仍然分成两个槽位值：① 体验者要的是"选一个重定位器"，不是"配一个后端"；
> ② `map→odom` 同一时刻只能有一个发布者 ⇒ 两个实现**没法在同一次运行里比**，各自占一个槽位值才能各自复现
> （这也是"分清楚、不要融合在一起"的落地方式）。
>
> 这两条路是**对等取舍，没有主次**（整栈实测，Release 口径，`world:=RMUL2026`，同一目标 `--goal -1.0 2.0`；
> 出处 `docs/gicp_backend_small_gicp.md` §4.5 与 `algorithm_matrix.md` §9.2.3）：
>
> | 指标（整栈） | `localization:=gicp` | `localization:=small_gicp` | 谁更好 |
> |---|---|---|---|
> | align 中位数 | 15.7 ms（11.8~25.4） | **2.9 ms**（2.6~4.0） | `small_gicp` 快 **5.4×**（CPU 更省） |
> | fitness（末帧） | **0.00226 m²** | 0.00230 m² | 同级（差 0.00004 m² = 噪声底） |
> | 采纳帧数（末帧 `[status]`） | 165/165（100%） | 167/167（100%） | 并列 |
> | `~/converged` true 比例 | **100%**（20/20、24/24、25/25、127/127） | 89~92%（113/127、117/127；小样本 92~96%） | `pcl` 更干净；`small_gicp` 的抖动是**帧级判定**问题，**位姿不受影响**（机理见 `docs/gicp_backend_small_gicp.md` §6.2） |
> | 整栈 P0 回归 | PASS / 3.3 s / `recoveries=0` | PASS / 3.4 s / `recoveries=0` | 并列 |
> | 库与线程 | PCL 1.12.1 `GeneralizedIterativeClosestPoint`（单线程） | vendored `koide3/small_gicp` v1.0.1（MIT，OpenMP，`small_gicp_num_threads: 4`） | — |
>
> **一句话**：`small_gicp` 省 CPU（单帧快 5.4×），`pcl` 的 `~/converged` 更干净；精度同级，两边都过整栈 P0 回归。
> 配置里那句 `backend: "pcl"` **是"选择"，不是因为 `small_gicp` 更差**：现有阈值
> （`fitness_score_warn 0.05` / `max_fitness_score 0.3` / `tf_lookahead_sec 0.45`）都是按 PCL 路径实测标定的，
> 改默认值等于换一个未标定的定位栈。要在槽位层面用 `small_gicp`，**直接 `localization:=small_gicp` 即可**
> （2026-10-05 五路对照时那种"临时把 YAML 改成 `small_gicp`、跑完 `git checkout` 还原"的做法**已不需要**，
> 见 `algorithm_matrix.md` §9.2.4 第 2 条）。
> 回退：`localization:=gicp`（= 文件默认 `pcl`）；整包回退 = `git revert` 引入 small_gicp 的那个 commit。
> 节点内部细节见 §1.1，完整对照 / 台架数据见 `docs/gicp_backend_small_gicp.md`。

### 1.1 `gicp` 槽位细节（2026-10-05 新增；与 `icp` 同资产、同初值契约）

> ★ **2026-10-06 行为变更**（用户实跑 `world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp
> nav:=mppi planner:=smac2d` 报"到点后四处抖动"，日志里 `map→odom` 单调跑掉而 `~/fitness_score` 一直很好）：
> 本槽位与 `small_gicp` 槽位新增 **三道接受判据 + 环路内低增益滤波**
> （运动一致性门限 / 合理性-定义域 / 连续拒绝判"定位失效"并停发 TF / tau=1.0 s 平滑），
> 新增键 `gate_*`、`plausible_*`、`lost_after_rejections`、`smoothing_*`（**全都有默认值**，
> 逐键依据写在 `gicp_registration/config/gicp_registration_sim.yaml`）。
> **契约不变**（只发 `map→odom`、时间戳契约、三个健康话题、`/initialpose` handoff/apply 都不动）；
> 唯一语义收紧：`~/converged` 现在 = "被采纳 **且** 通过新判据"（原来只看 `fitness_score_warn`）。
> 机制（自相似性实测：偏 0.07~2.5 m 的位姿 fitness 仍 0.002~0.10 m²）、复现数字、A/B 表与回退：
> **`docs/gicp_divergence_and_jitter.md`**。回退到改动前行为：
> `-p gate_enable:=false -p smoothing_enable:=false`（或 `git revert` 那个 commit）。

- **节点**：`gicp_registration/gicp_registration_node`（`ament_cmake` + `rclcpp_components`）；
  **两个对等配准后端**（`backend: pcl | small_gicp`，**默认 `pcl`**）：
  `pcl` = PCL `pcl::GeneralizedIterativeClosestPoint`（单线程，BFGS 内层迭代），
  `small_gicp` = vendored `koide3/small_gicp` @ `v1.0.1`（MIT，OpenMP 多线程，LM 优化器）。
  两者**契约、话题、参数键、健康指标、点类型完全同一套**。
  ★ **槽位层面它们是两个独立的值**（`localization:=gicp` / `localization:=small_gicp`，由 `bringup_sim.launch.py`
  各自的槽位分支注入 `backend`）—— **槽位用户不要在这里改 `backend`**；§1.1 以下讲的都是节点内部细节，
  并列的实测取舍（align / fitness / 采纳率 / `~/converged`）与回退见 §1 末尾的脚注
  与 **`docs/gicp_backend_small_gicp.md`**——那是"两个平等选项"的对照表，**没有主次**。
  点类型 = **`pcl::PointXYZ`**（两个后端共用）。
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

#### 1.2.0 ★★ 2026-10-05 修复：`localization:=beluga` 节点**启动即崩**（`laser_min_range: -1.0`）

**症状**（`localization:=beluga` 第一次实跑；本文件曾把 nav2 的哨兵值原样搬过来）：

```
[amcl_node] terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterValueException'
  what():  parameter 'laser_min_range' could not be set: Parameter {laser_min_range} doesn't comply with floating point range.
[ERROR] [amcl_node-13]: process has died [exit code -6]      # -6 = SIGABRT
```

**根因（已按装机二进制 + 源码逐行核实）**：beluga 给 `laser_min_range` / `laser_max_range` 声明的是
**浮点区间 `[0.0, DBL_MAX]`**（`ros2_common.cpp:296-310` @2.1.1，`floating_point_range[0].from_value = 0`；
上游文档 likewise 写 "Must be nonnegative"），而 **nav2_amcl 对同一对键的默认是 `-1.0` =
"用 /scan 自己的 range_min"**（`third_party/nav2/nav2_amcl/src/amcl_node.cpp:114`，且 nav2 只在
`> 0` 时才 clamp，`amcl_node.cpp:828-829`）。本文件第一版照抄了 nav2 的值 ⇒ 参数文件覆盖值
在 **节点构造期**（`declare_parameter` 应用 override）就被 rcl 的区间校验拒掉 ⇒ 抛异常 → 进程 abort。

三个必须记住的判据：
1. **抛在构造期，不是 configure 期**：`lifecycle_manager` 连 configure 都发不出去（节点进程已经没了）
   ⇒ `/amcl/get_state` 之类的"救火"手段一律无效，**只能改参数文件**；
2. **级联现象不是独立故障**：没有 `map→odom` ⇒ `planner_server` 一直
   `Invalid frame ID "map"` / `Timed out waiting for transform`（本次报告里那些报错都是这一条引起的）；
3. **ROS 2 对"文件里有、节点没声明"的键是静默忽略，但对"声明了、值越界"是直接抛**——
   这两类错误的处置完全不同（本仓库 §1.1 踩过的是前一类）。

**修法（本文件已改）**：**删掉 `laser_min_range` 整个键**，用 beluga 的默认 `0.0`；`laser_max_range: 100.0` 保留。
为什么这是**语义等价**而不是"妥协"：
- beluga 侧是 **clamp**：`min_range_ = max(scan->range_min, laser_min_range)`、
  `max_range_ = min(scan->range_max, laser_max_range)`（`beluga_ros/include/beluga_ros/laser_scan.hpp:55-61` @2.1.1）；
- 本仓 `/scan` 由 `pointcloud_to_laserscan` 产生，`range_min: 0.05` / `range_max: 10.0`
  （`src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`）；
- 于是默认 `0.0` ⇒ `max(0.05, 0.0) = 0.05` = `scan.range_min`，**与 nav2 的 `-1.0` 逐字等价**；
  `100.0` ⇒ `min(10.0, 100.0) = 10.0` = `scan.range_max`，与 nav2（`ldata.range_max = min(scan.range_max, 100)`）一致；
- 上游自己的参考参数文件 `beluga_example/params/default.ros2.yaml` **也不写 `laser_min_range`**（同为默认 0.0）——
  也就是说"省略该键"正是上游的推荐用法。

⚠️ **`laser_max_range` 在 beluga 里是"一键两用"**（审计时发现的唯一语义差，**不致命**）：
除了上面的 clamp，它还**直接**被当作 `LikelihoodFieldModelParam::max_laser_distance`
（`amcl_node.cpp:379` @2.1.1），而后者是似然场里 `z_rand` 项的归一化分母
（`offset = z_random / max_laser_distance`，`beluga/include/beluga/sensor/likelihood_field_model_base.hpp:142`）。
nav2 用的是**被 clamp 后的 `scan.range_max`**（=10.0）⇒ 两边随机项地板不同：
**beluga = 0.5/100 = 0.005，nav2 = 0.5/10 = 0.05**（差 10 倍，只影响权重形状，不影响能否起节点）。
**A/B 时若 beluga 明显更差，第一个旋钮就是把它设成 `/scan` 的 `range_max`（本仓 = `10.0`）**，
让两边的传感器模型数值完全对齐；保持 100.0 的理由是"与 amcl 槽的键值逐字一致"+ 上游默认值。

**装机二进制的实测约束表（本次审计；证据来自跑起来的节点本身）**

证据获取方式：把节点按修好的参数文件**真的起起来**，再对 `/amcl` 调
`rcl_interfaces/srv/DescribeParameters`（脚本 `.tmp_beluga_verify/dump_descriptors.py`，原始输出
`.tmp_beluga_verify/log_131559/param_descriptors.txt`）⇒ 下表"声明区间"列**不是读源码抄的，是节点自己报的**；
交叉核对源码：`ros2_common.cpp:33-405` + `amcl_node.cpp:89-203` @2.1.1（tag = deb 版本 2.1.1，
`beluga_amcl/include/beluga_amcl/{amcl_node,ros2_common}.hpp` 与上游 md5 逐字节一致）。
`/amcl` 共声明 **72** 个参数（含 10 个 `qos_overrides.*` 只读键）。

| 参数键（本文件写的 45 个） | 类型 | 装机二进制声明的区间 | 不写时的默认 | 本文件取值 | 判定 |
|---|---|---|---|---|---|
| `use_sim_time` | bool | — | `false` | `True`（launch 注入） | ✅ |
| `global_frame_id` / `odom_frame_id` / `base_frame_id` | string | — | `map` / `odom` / **`base_footprint`** | `map` / `odom` / `base_link` | ✅（`base_frame_id` 必须显式写）|
| `robot_model_type` | string | — | `differential_drive` | `nav2_amcl::OmniMotionModel` | ✅（接受 nav2 插件名；非法值只在建滤波器时 ERROR，不崩）|
| `alpha1`…`alpha5` | double | float `[0, 1.79769e+308]` | `0.2` | `0.2` ×5 | ✅ |
| `min_particles` / `max_particles` | integer | int `[0, 2147483647]` step=1 | `500` / `2000` | `500` / `2000` | ✅ |
| `pf_err` | double | float `[0, 1]` | `0.05` | `0.05` | ✅ |
| `pf_z` | double | — | `0.99` | `0.99` | ✅ |
| `resample_interval` | integer | int **`[1, 2147483647]`** step=1 | `1` | `1` | ✅（写 0 会抛）|
| `recovery_alpha_fast` / `recovery_alpha_slow` | double | float `[0, 1]` | `0.0` / `0.0` | **`0.1` / `0.001`**（§九 搬运） | ✅ |
| `spatial_resolution_x` / `_y` | double | float `[0, 1.79769e+308]` | `0.5` | `0.5` | ✅ |
| `spatial_resolution_theta` | double | float **`[0, 6.28319]`** | `0.174533`(10°) | `0.174533` | ✅ |
| `execution_policy` | string | — | `seq` | `seq` | ✅（`seq`/`par` 之外的串只在建滤波器时 ERROR）|
| `selective_resampling` | bool | — **只读** | `false` | `false` | ✅（只读只挡"运行期 set"；参数文件里的初值照收，已实测 `:=true` 也不抛）|
| `update_min_d` | double | float `[0, 1.79769e+308]` | `0.25` | **`0.05`**（§九） | ✅ |
| `update_min_a` | double | float **`[0, 6.28319]`** | `0.2` | **`0.05`**（§九） | ✅ |
| `transform_tolerance` | double | float `[0, 1.79769e+308]` | `1.0` | **`0.3`**（§九） | ✅ |
| `laser_model_type` | string | — | `likelihood_field` | `likelihood_field` | ✅ |
| `laser_likelihood_max_dist` | double | float `[0, 1.79769e+308]` | `2.0` | `2.0` | ✅ |
| **`laser_min_range`** | double | float **`[0, 1.79769e+308]`** | **`0.0`** | **删掉（=默认 0.0）** | ✅ **本次修复**：`-1.0` 违反区间 ⇒ 构造期抛（就是本节的故障）|
| `laser_max_range` | double | float `[0, 1.79769e+308]` | `100.0` | `100.0` | ✅（⚠️ 一键两用，见上面语义差）|
| `max_beams` | integer | int **`[2, 2147483647]`** step=1 | `60` | `60` | ✅ |
| `z_hit` / `z_max` / `z_rand` / `z_short` | double | float `[0, 1]` | `0.5` / `0.05` / `0.5` / `0.05` | 同左 | ✅ |
| `sigma_hit` / `lambda_short` | double | float `[0, 1.79769e+308]` | `0.2` / `0.1` | 同左 | ✅ |
| `model_unknown_space` | bool | — | `false` | `false` | ✅ |
| `only_obstacle_boundaries` | bool | — | `true` | `true` | ✅ |
| `scan_topic` / `map_topic` / `initial_pose_topic` | string | — | `""`（回落 `scan`）/ `map` / `initialpose` | `scan` / `map` / `initialpose` | ✅（`scan_topic` 与 `point_cloud_topic` **互斥**，两个都非空 ⇒ activate 期抛异常）|
| `tf_broadcast` | bool | — | `true` | `true` | ✅ |
| `set_initial_pose` | bool | — | `false` | `true` | ✅ |
| `initial_pose.x` / `.y` / `.yaw` | double | — | `0.0` | `0.0`（launch 按 world 覆盖） | ✅ |
| `initial_pose.covariance_x` / `_y` / `_yaw` | double | — | `1e-6` | `1.0e-6`（YAML 科学计数法实测被当 double，不是 string） | ✅ |
| `initial_pose.covariance_xy` / `_xyaw` / `_yyaw` | double | — | `0.0` | `0.0` | ✅ |
| `always_reset_initial_pose` / `first_map_only` | bool | — | `false` | `false` | ✅ |

**故意不写的键**（写了也只会被静默忽略，或者根本不存在）：

| 键 | 为什么 |
|---|---|
| `initial_pose.z` / `save_pose_rate` / `do_beamskip` / `beam_skip_distance` / `beam_skip_threshold` / `beam_skip_error_threshold` | **beluga 2.1.1 根本没声明**（grep 上游 `beluga_amcl/src` 无命中；nav2 侧确实有）⇒ 写了静默失效（`localization_beluga_launch.py` 也因此不注入 `initial_pose_z`）|
| `point_cloud_topic` / `map_path` | 存在但不用（`map_path` 只在 HDF5 加载路径用；`point_cloud_topic` 与 `scan_topic` 互斥）|
| `autostart` / `autostart_delay` | 默认 `false`，由 `lifecycle_manager` 管（与 amcl 槽同形态）|
| `bond_timeout` | 默认 `4.0`，与 `lifecycle_manager` 心跳配合（**注意**：见下面"已知残留问题"）|
| `debug` | 默认 `false`；`true` 会多发 `/likelihood_field` 并降性能 |
| `qos_overrides.*` | 节点自动声明的只读键（`/tf` 发布 100/reliable/volatile 等），**不要写** |

**本次孤立实跑验证（不启 Gazebo / 不启 nav2 controller/planner/bt_navigator）**

```bash
# 一键复现本节的验证（隔离域 88 + 工作区内 ROS_LOG_DIR；只起 map_server + amcl_node + lifecycle_manager）
bash .tmp_beluga_verify/run_isolated.sh          # 脚本 + 探针在 .tmp_beluga_verify/（临时目录，未入库）
```
实测结果（原始输出 `.tmp_beluga_verify/run_report.txt`，日志 `log_131559/`）：

| 要求 | 实测 |
|---|---|
| configure/activate 不抛 | ✅ `lifecycle_manager_localization`: `Configuring amcl` → `Activating amcl` → `Server amcl connected with bond.` → `Managed nodes are active`；`ros2 lifecycle get /amcl` = **`active [3]`**；`amcl.log` 里 **0 条 WARN/ERROR**（除了 SIGINT 收尾那条，见下）|
| `/scan` 订阅 QoS | ✅ `ros2 topic info /scan -v`：`Reliability: BEST_EFFORT`（SensorDataQoS，KEEP_LAST 5）|
| 有扫描就发 `map→odom` | ✅ 合成 `/scan`（BEST_EFFORT，frame_id=`base_link`，`range_min 0.05`/`range_max 10.0`，360 束）+ `odom→base_link` 静态/动态 TF ⇒ **首条 `map→odom` 在探针开始后 104 ms 出现**（车还没动）|
| `map→odom` 速率 | ✅ **139 条 / 13.80 s = 10.07 Hz**（`/scan` 10.00 Hz；相邻间隔中位 100.0 ms、max 102.3 ms）⇒ **≈ /scan 速率，不是固定 50 Hz**（与 §1.2 契约表一致）|
| 是否"发到未来"（nav2 消费者要求） | ✅ `tf.stamp − scan.stamp` = **0.2997 / 0.2998 / 0.2999 s（min/均值/max）= `transform_tolerance` 0.3**，与 `amcl_node.cpp:628-630` 的 `expiration_stamp = scan.stamp + transform_tolerance` 逐字吻合；**139/139 条戳都在"收到时刻"之后**（未来量均值 0.2992 s）|
| 真滤波更新（不是只重发 TF） | ✅ `/pose` 共 40 条：静止段（0~6 s，`update_min_d=0.05` 不触发）**1 条**，运动段（0.3 m/s）39 条 ≈ **4.9 Hz**（`Particle filter update iteration stats: 500 particles 360 points - ~0.3 ms`）|
| 只发 `map→odom` | ✅ `/tf` 上只出现 `map→odom` 与（探针自己发的）`odom→base_link`，beluga 没碰 `odom→base_link` |
| SIGINT 干净退出 | ❌ **不干净**：`Destroying` → `Shutting down` → `Deactivating` → `terminate called ... RCLError: Couldn't initialize rcl timer handle ... rcl_shutdown() was called` ⇒ **exit 134 (SIGABRT)** |

**★ 已知残留问题：SIGINT 收尾会 abort（上游 bug，与本次参数修复无关）**

gdb 抓到的栈（`handle SIGINT nostop pass` + SIGABRT 时 `bt`）：

```
#11 ?? () from /opt/ros/humble/lib/libbondcpp.so        <- 抛 RCLError 的地方
#12 bond::Bond::~Bond() () from /opt/ros/humble/lib/libbondcpp.so
#13 beluga_amcl::BaseAMCLNode::on_deactivate(rclcpp_lifecycle::State const&)
#14 beluga_amcl::BaseAMCLNode::on_shutdown(rclcpp_lifecycle::State const&)
#15 beluga_amcl::AmclNode::~AmclNode()
#16 main
```
因果链：`rclcpp` 的 SIGINT handler **先** `rcl_shutdown()` → `spin()` 返回 → `main` 析构节点 →
`~AmclNode` 里**又**调 `on_shutdown` → `on_deactivate` → `bond_.reset()` → `~Bond()` 在这个**已失效的
context** 上 `create_timer`（bondcpp 的 `xxxTimerReset` 家族）⇒ 异常从析构函数里逃出 ⇒ `std::terminate`。
机制侧上游相关 issue：[rclcpp#2793 "create_timer throws but isn't documented to throw"](https://github.com/ros2/rclcpp/issues/2793)；
beluga 侧另有症状不同的旧 issue：[beluga#67 "amcl_node doesn't terminate after SIGINT"](https://github.com/Ekumen-OS/beluga/issues/67)。

**实测的变体表（决定要不要为此改接线）**：

| 变体 | 节点 active? | `lifecycle_manager` 反应 | SIGINT 退出码 | 结论 |
|---|---|---|---|---|
| A 默认（`bond_timeout` 不写 = 4.0） | active | 正常，bond 成型 | **134（abort）** | 本仓现状 |
| B 只给节点 `bond_timeout:=0.0` | active | bond 0.2 s 就"broken" ⇒ `CRITICAL FAILURE: SERVER amcl IS DOWN ... Shutting down related nodes` ⇒ 反复 deactivate/re-activate（**定位会被拆掉**） | 0 | ❌ **不可用** |
| C 先 `ros2 lifecycle set /amcl deactivate` 再 SIGINT | inactive | — | 0 | 证明"bond 还在 ⇒ 必崩" |
| D `bond_disable_heartbeat_timeout:=true`（bondcpp 自带键，`/bond_disable_heartbeat_timeout`） | active | 正常，无 CRITICAL | **134** | ❌ 无效 |
| E 节点 **和** manager **都** `bond_timeout:=0.0` | active | manager 不建 bond 监视（日志里连 bond 行都没有） | **0** | ✅ 唯一可用的"退出码干净"方案，但**等于关掉 bond 看门狗**（manager 不再能发现卡死的 amcl）|

**本次决定**：**保持 A（不动 bond 配置）**——理由是 ① 与 `localization:=amcl` 槽行为对齐（bond 是
lifecycle_manager 的安全网）；② 这个 abort 只发生在**进程收尾**（Ctrl-C 时整栈本来就在退），
**不影响启动/激活/发 TF/定位**；③ 下面 §1.2.1 的 respawn 审计（2026-10-05 补做）把"beluga 节点会不会
因为这次 abort 被反复拉起"逐节点核实过：**beluga 节点在这条路上 `respawn=False`，不会被重启**；
真实 Ctrl-C 下 launch 自己的 shutdown 门闩还会再挡一层 ⇒ **不构成重启循环**。
若用户不接受 Ctrl-C 时那条 `terminate called ...`/`exit code -6` 日志，按 **E** 改两处即可
（参数文件加 `bond_timeout: 0.0` + `localization_beluga_launch.py` 的 manager 参数加 `bond_timeout: 0.0`），
代价是失去 bond 看门狗——**这是一个需要用户拍板的取舍，本次没有替用户改**。

#### 1.2.1 ★ 2026-10-05 补做：`respawn` 审计（结论 = **不改代码**）

**为什么要补**：初版只写了"我们 launch 里 `use_respawn` 默认 `False`"，但审计发现
`bringup_rm_navigation.py:112` 的 `use_respawn` 默认是 **`True`** 且会透传给 Nav2 节点（`:164`）
⇒ "这条路上没人开 respawn"这句话必须**逐节点**核实，不能靠默认值推断。

**逐节点审计（`localization:=beluga` 路径 = `bringup_sim.launch.py` 起的全部进程）**：

> 行号口径：`localization_beluga_launch.py` / `bringup_rm_navigation.py` / `navigation_launch.py`
> 按本仓库 HEAD（`0a2e74d`，这三个文件当前无人并行改动）；`bringup_sim.launch.py` 的行号也按 HEAD，
> 但**该文件正被另一条任务并行改动**（同日新增 `localization:=small_gicp` 槽）⇒ 若行号对不上，
> 按括号里的 `include`/节点名 / `LaunchConfiguration` 名定位。

| 进程 / include | respawn 取值 | 出处（file:line） |
|---|---|---|
| Gazebo / 感知 / 桥接等（`start_rm_simulation`、imu 互补滤波、地面分割、p2l、LIO+adapter、`fake_vel_transform`、建图 RViz） | **不设**（= 无 respawn） | `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py`（全文件无 `respawn`）、`bringup_sim.launch.py:271-300`、`:600-690` |
| `localization_beluga_launch.py`（include 本体） | 父层**没传** `use_respawn`（`launch_arguments` 里没有该键） | `bringup_sim.launch.py:446-458` |
| ↳ `map_server` | `respawn=False` | `localization_beluga_launch.py:199`（`respawn=use_respawn`）+ `:168`（本 launch 自带默认 `False`） |
| ↳ **`amcl_node`（beluga）** | **`respawn=False`** ← 关键 | `localization_beluga_launch.py:212` + `:168` |
| ↳ `lifecycle_manager_localization` | 不设（= 无 respawn） | `localization_beluga_launch.py:217-225` |
| `start_navigation2` → `bringup_rm_navigation.py` | `use_respawn` 默认 **`True`**；`use_composition` 声明默认 `True` | `bringup_rm_navigation.py:111-113`、`:107-109` |
| ↳ `nav2_container`（组合容器） | 不设（= 无 respawn） | `bringup_rm_navigation.py:142-150` |
| ↳ **7 个 Nav2 节点**（controller / smoother / planner / behavior / bt_navigator / waypoint_follower / velocity_smoother） | **`respawn=True`**（本次实测**确实生效**） | `bringup_rm_navigation.py:164` → `navigation_launch.py:177/187/197/207/217/227/237` |
| ↳ `lifecycle_manager_navigation`、`rviz_launch.py` | 不设（= 无 respawn） | `navigation_launch.py:243-251`、`bringup_rm_navigation.py:167-170` |

**两条容易搞错的机制**（源码 + 实测都核过）：

> 下面出现的 `launch/actions/*.py`、`launch/utilities/*.py`、`launch/launch_service.py` 都指
> **ROS 2 Humble 安装里的 `launch` 包**（本机 `/opt/ros/humble/lib/python3.10/site-packages/launch/`），
> **不是本仓文件**；带 `:行号` 的引用按 0.19.14（`launch_ros` 0.19.14）核过。

1. **`use_composition` 被"泄漏"成 `False`**，所以上面那 7 个 Nav2 节点是**独立进程**
   （`respawn=True` 才真有落点）：`bringup_sim.launch.py:265` 给 *Gazebo 的* include 传了
   `'use_composition': 'False'`，而 `IncludeLaunchDescription` 的 `launch_arguments` 是**全局**
   `SetLaunchConfiguration`（`launch/actions/include_launch_description.py` 的 `execute()` 返回
   `[SetLaunchConfiguration(...), <子 ld>]`，**不还原**），后到的 `DeclareLaunchArgument` 只在
   "键还没被设置"时才采用默认值（`launch/actions/declare_launch_argument.py` 的 `execute()`：
   `if self.name not in context.launch_configurations:`）⇒ 10 s 后 `bringup_rm_navigation.py:108`
   的默认 `True` **输给**了 0 s 泄漏进来的 `False`。（旁证：五次整栈日志里 Nav2 节点全是独立进程，
   没有 `nav2_container`。）
   ⇒ 副作用提醒：**任何**在 `start_rm_simulation` 之后才 include 的 launch，其同名
   `DeclareLaunchArgument` 默认值都可能被这次泄漏顶掉（beluga 分支自身不受影响：它要的 `use_respawn`
   在 4 s 那次 include 里解析，此时上下文里还没有这个键）。
2. **真实 Ctrl-C 不会重启任何进程**：`launch/actions/execute_local.py:583` 的重启条件是
   `if not context.is_shutdown and not self.__shutdown_future.done() and self.__respawn:`
   —— SIGINT 一进 launch 就会 `_shutdown()` ⇒ `context.is_shutdown=True` ⇒ respawn 被门闩挡住
   （`respawn_delay` 那条分支同样先等 `__shutdown_future`）。本次用一个**不含任何 ROS 节点**的最小
   launch（两个 `sleep`，一个 `respawn=True` 一个 `respawn=False`）实测：以 SIGINT **默认处置**起
   `ros2 launch` 再对整组发 SIGINT ⇒ launch 打印 `user interrupted with ctrl-c (SIGINT)`、自身
   退出码 0、`respawn=True` 的那个 `sleep` **没有被重启**（若是 SIGINT 被忽略的场景则会重启，见下条）。

**"headless 五路测试里 SIGINT 之后节点被重新拉起"——本次解释清楚了：那是测试驱动的假象**，
不是用户 Ctrl-C 的行为。`run_one.sh` 用 `setsid ros2 launch ... &`（**非交互 shell 的后台作业
SIGINT 处置 = `SIG_IGN`**）；launch 的 `AsyncSafeSignalManager` 只调用 `signal.set_wakeup_fd`、
**不调用 `signal.signal`**（`launch/utilities/signal_management.py` 的 `__install_signal_writers`）
⇒ launch 进程**根本没收到**这次 SIGINT（五份日志里 `user interrupted with ctrl-c` **0 行**），
而子进程里的 rclcpp **自己装了 SIGINT handler** ⇒ 节点全死、launch 还活着 ⇒ 那 7 个 `respawn=True`
的 Nav2 节点按 `respawn_delay=2.0` 被重新拉起。证据（`.tmp_cache/five_way/`）：

| 证据 | 值 |
|---|---|
| 五份日志的 `process started with pid` 行数 | 每份 **29** = 初始 22 + SIGINT 后重启 7（模式完全一致） |
| 定位节点启动次数 | `gicp`/`amcl`/`beluga`/`icp` 每次运行**都只有 1 次** |
| beluga 的 `amcl_node-13` | `beluga.log:58` 启动（pid 484）→ `:334` 收到 SIGINT → `:431` `exit code -6` 死亡；**再无第二次启动** |
| 重启的 7 个 | `beluga.log:473/479/485/486/491/503/514`（新 pid 3102…3174，SIGINT 后 ≈2~3 s） |

⇒ ① **beluga 的 abort 既不是重启的原因，也没有被重启**（`respawn=False`，日志里只有"一次启动 +
一次 `exit code -6`"）；② 若哪天要修"测试收尾后栈还活着/节点被拉起"，要改的是**测试发 SIGINT 的方式**
（用 SIGINT 默认处置，例如 `python3` + `os.killpg`；或在 `setsid` 之外再对 launch 单独补一发）
或 `bringup_sim.launch.py` / `bringup_rm_navigation.py` 的 `use_respawn` 透传，
**都不在 beluga 这一槽**（本次按规定只审计 + 记录，未改任何代码/launch）。

**§1.2.1 结论**：abort 只发生在本进程**自己的收尾路径**上，而 ① beluga 节点这条路 `respawn=False`
（不可能被重启）；② 即便有人显式 `use_respawn:=True` 打开它，一次 SIGINT 也只会换来**一次**重启
（新起的进程不会自己再 abort ⇒ **不成循环**）；③ 真实 Ctrl-C 下 launch 的 shutdown 门闩直接禁掉重启。
⇒ **本次不改代码**（bond 配置保持 **A** = `bond_timeout` 不写，即 4.0）。
变体 **E**（节点 + manager 都 `bond_timeout: 0.0`，实测退出码 0）仍然只是"用户想要干净退出码"时的
**可选项**，为防重启而改它是**没有依据**的。

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
| `laser_min_range` / `laser_max_range` | `-1.0` / `100.0` | ⚠️ 存在但**区间不同** | beluga 是 **clamp**：`min_range=max(scan.range_min, 值)`、`max_range=min(scan.range_max, 值)`（`beluga_ros/include/beluga_ros/laser_scan.hpp:55-61`），且声明区间是 **`[0, DBL_MAX]`** ⇒ **nav2 的 `-1.0` 会把节点打成构造期异常**。本文件**删掉 `laser_min_range`**（默认 0.0，与本仓 `/scan` 的 `range_min 0.05` 组合后与 nav2 的 `-1.0` 等价），`laser_max_range` 保持 `100.0`。详见 §1.2.0 |
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
- ✅ **2026-10-05（同日稍后）：路线 (A) 已真正落地**——`ros-humble-beluga-amcl / beluga / beluga-ros
  2.1.1-1jammy.20260908.012840` 已装进 `/opt/ros/humble`（`dpkg -l | grep beluga` 三条；
  `ros2 pkg prefix beluga_amcl` = `/opt/ros/humble`；`/opt/ros/humble/include/beluga_amcl/**` 与上游
  tag `2.1.1` 的对应头文件 **md5 逐字节一致**）。§1.2.0 的全部结论与实测都跑在这个装机二进制上。
- ⚠️ 历史记录（本节第一版写的时候）：那时本沙箱 `sudo` 被禁（"no new privileges"），**没有真的装到
  `/opt/ros/humble`**：静态验证是用 `apt-get download` 把 3 个 deb 下下来、`dpkg -x` 解到
  `.tmp_cache/beluga_overlay/opt/ros/humble` 再 source 该 overlay 做的（`ros2 pkg prefix beluga_amcl` 解析成功、
  `ldd` 无缺失、launch 预检通过）。⇒ **那条"未在真机验证过安装"的保留意见现在已作废**。

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
- 代码级（本槽整体，commit `21fc301`）：`git revert 21fc301`（3 个文件：bringup 的 choices/分支/
  map_server 条件 + 新增 launch + 新增 params）；
- 代码级（**本次启动崩溃修复**，2 个文件：`nav2_params_sim_beluga.yaml` + 本文档）：
  `git revert <本次 commit>`；**只想临时回到"能起但不发 TF"的状态**没有意义——修复前的版本是
  **节点构造期 abort**，所以回退这个 commit = 回到完全不可用，除非另配 `laser_min_range` 的合法值
  （唯一要求：`>= 0`；`0.0` 与"删掉该键"等价）；
- 只回退行为、不改代码：把 `laser_max_range: 100.0` 改成 `10.0`（= p2l 的 `range_max`，见 §1.2.0 的两用说明）
  即可，无需动别的键；
- 卸载：`sudo apt remove ros-humble-beluga-amcl`（连带 beluga/beluga_ros）。

**⚠️ 未验证清单（2026-10-05 更新版）**

> 第一版写的"只做静态验证、没有启动 Gazebo/nav2/beluga 节点"**已被 §1.2.0 的孤立实跑取代**：
> 节点**单独**起过（map_server + amcl_node + lifecycle_manager，隔离域），configure/activate、
> `/scan` QoS、`map→odom` 速率与盖戳、真滤波更新、SIGINT 退出码都已实测。**仍未验证的**是"整栈"：

1. **整栈运行时的定位精度、收敛性、CPU/RTF**：**仍未测**（不启 Gazebo；孤立探针喂的是合成 `/scan`，
   只有"起得来 + 发 TF"的结论，**没有任何精度结论**）；
2. **beluga 与 nav2 amcl 的 A/B 结论**：`--goal -1.0 2.0` 回归、到达误差、用时、恢复次数**均未跑**；
   另外注意 §1.2.0 里那条 `laser_max_range` 一键两用导致的 `z_rand` 地板差 10 倍 ⇒ A/B 前先决定要不要把它改成 10.0；
3. **`map→odom` 在真栈里的连续性**：孤立实测是 10.07 Hz / 间隔 max 102 ms（合成扫描 10 Hz）；真栈里
   `/scan` 由 LIO+p2l 产生，**丢帧/抖动下的表现未测**；
4. **`use_sim_time:=true` 路径**：孤立验证为了让节点自己走时钟用的是 `-p use_sim_time:=false`；
   **仿真时钟（/clock、Gazebo）下的表现未测**（真栈由 bringup 注入 `true`）；
5. **`autostart` 路径**：本仓库走 lifecycle_manager（`autostart: false`），beluga 自带的 `autostart: true`
   免 manager 路径未试；
6. **`use_composition: true` 路径**：`beluga_amcl::AmclNode` 组件与 `nav2_container` 的组合未试
   （默认 false；amcl 槽的这条路同样没在 bringup 里接容器）；
7. **`only_obstacle_boundaries` / `model_unknown_space` / `selective_resampling` / `execution_policy: par`
   的实际影响**：未调、未测；
8. **`localization:=beluga` + `lio:=cartographer` 这个非法组合**：与 amcl 一样会「两个槽都不起 + 无
   map_server」（`lio==cartographer` 时 beluga 分支条件为假，而独立 map_server 又被排除）——**这是
   amcl 早就有的同款行为**，本次只做"与 amcl 对齐"，未修；
9. **SIGINT 收尾 abort（§1.2.0）**：根因已定位（bondcpp `~Bond` 在 `rcl_shutdown()` 之后建 timer），
   但**没有修**（属上游 + 只在收尾）⇒ 整栈 Ctrl-C 时 beluga 进程会以 **exit code 134 / -6** 收场。
   ✅ **"会不会触发重启"这一问已销案**（2026-10-05 §1.2.1：节点 `respawn=False` + 五路整栈日志实证
   `amcl_node` 只启动一次；真实 Ctrl-C 下 launch 还有 `execute_local.py:583` 的 shutdown 门闩）；
   仍未实测的只剩"**用户在场**时那条 `terminate called ...` 日志的观感"与变体 E 的实际取舍；
10. **apt 安装**：已完成（`/opt/ros/humble`，2.1.1-1jammy.20260908.012840），**此项销案**。

## 2. 待补入口（**已登记、未实现**）

下表这几条**还没有槽位值**（在 `--show-args` 的 choices 里选不到 ⇒ 与 §1 那八个值不冲突）；
落地时按 §3 的三步加一个槽位值，然后从本表移进 §1。

| 计划槽位值 | 用什么 | 依赖 / 资产 | 落地要点 | 估时 |
|---|---|---|---|---|
| `scan_context` | Scan Context **全局检索** + ICP/GICP **精配准**（两级） | 需引入 Scan Context 实现 + 用 PCD 建描述子库 | 解决"**车随便摆 / 被搬动**"（ICP 类天生初值敏感）；检索出粗位姿 → 现有精配准 | 2~3 天 |
| `fastlio_loc` | LIO + 先验 PCD 做配准得 `map→odom`（一体化配方） | PCD 已有 | 与我们 `lio_tf_adapter` 有职责重叠 ⇒ 作为**对照实现** | 1~2 天 |
| `teaserpp` | TEASER++ 无初值全局配准 | 需引入 TEASER++ + 特征 | 远期；开销大 | 远期 |

> **已从本表移出、进了 §1 槽位一览的三项**（都发生在 2026-10-05，都按"一个实现 = 一个槽位值"落的）：
> · **`beluga`** —— apt 装 `ros-humble-beluga-amcl` 2.1.1（Humble 官方二进制，**不 vendor 代码、不新增算法包**），
>   走的正是原计划那条"沿用 AMCL 参数语义做 A/B"的路（同名参数 + 兄弟参数文件），
>   把"接口兼容"落实成了逐键映射表 ⇒ 细节见 **§1.2**。
> · **`gicp`** —— **新包** `src/rm_localization/gicp_registration/`，**不是**原计划那条"在 `icp_registration` 内换后端"：
>   理由是 icp 槽要保留 ICP 原样，好做 AMCL / ICP / GICP 的 A/B ⇒ 细节见 **§1.1**。
> · **`small_gicp`** —— **同日、同包，但自己占一个槽位值 `localization:=small_gicp`**
>   （不再只是"`gicp` 槽背后的一个 `backend` 参数"）：体验者从槽位表里直接选，**实现细节由 launch 注入**。
>   并列取舍与实测数据见 §1 末尾的脚注，节点内部细节见 §1.1。
>
> 仍未引入：`fast_gicp`（另一个 GICP 实现，需要时按 §3 再加一个槽位值）、以及上表三条。

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
