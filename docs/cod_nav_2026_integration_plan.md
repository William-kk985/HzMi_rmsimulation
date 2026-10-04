# COD_NAV(2026) 架构融入我们 bench 的**规划**（只规划，不实验）

> 上游：https://github.com/qza36/COD_NAV 分支 **`rmul2026`**（最后提交 2026-03-10）
> 对比依据：`docs/cod_nav_comparison.md`
> 本文件性质：**设计/排期文档**。在明确说"改"之前，不动任何配置与代码。

> ⚠️ **2026-09-26 修正声明**：`docs/cod_nav_2026_deep_dive.md` 对该分支做了逐文件深挖，**发现本文若干行有误**（共 25 条，见那份文档 §E）。以下是最影响决策的 8 条，**以深挖文档为准**：
> 1. **裁剪盒的 `leaf_size: 0.05` 是死参**：`pcl::VoxelGrid` 整段被注释，节点只做 crop、不做降采样；
> 2. **裁剪盒在输入点云坐标系（`livox_frame`）里裁，但 marker 硬编码发在 `base_link`** ⇒ 两者错开；**不要照抄他们的 box 数值**，要按我们的 livox_frame 重新算；
> 3. 他们的两条 launch 用**两套不同的 box**：`slam.launch.py` x±0.2/y−0.2~0.4；`nav.launch.py` x±0.3/y−0.3~0.5（z 都是 −0.1~0.2）；
> 4. **p2l 真值**（写在 launch 内联，不在包内）：`target_frame: base_link`、`min_height **0.15**(slam)/**0.10**(nav)`、`max_height 1.00`、`angle ±3.1416`（**整圈**，包内注释写 −π/2 是错的）、`increment 0.0087`（723 线）、`range_min 0.5`、`range_max 20.0`、`use_inf true`、输入是 **`/livox/lidar`(PointCloud2)** ⇒ 本文第 1 节里的「0.01~1.00」应改为「0.10/0.15~1.00」；
> 5. **`smoother_server` 在他们的实时链里是死代码**（自定义 BT 里没有 `<SmoothPath>`）⇒ 真正的平滑是 **SmacPlanner2D 的 `smooth_path: true` + 内建 smoother**；本文"借 savgol"要改成"先把 Smac 的 smooth_path 打开"，savgol 只是我们自己可选的加分项；
> 6. **BT 事实修正**：两个 XML 里**都没有 `IsStuck` 节点**；`BackUp` 是 **1.0 m @ 1.0 m/s**（不是 1.5）且只在 through-poses BT 里；`navigate_to_pose` 的恢复只有 `ClearEntireCostmap`；`RateController 3 Hz` 与重试 10 次是对的；
> 7. **他们其实从不自转**：`spin_speed` 全分支只出现 5 次（源码/头文件/README），**没有任何文件把它设成非零** ⇒ `angular.z ≡ 0`（MPPI 的 wz ±2.5 被丢弃）+ `robot_base_frame: base_link_fake` + `yaw_goal_tolerance 6.28` = 纯位置控制。**所以"只控位置"是他们的事实行为，而"`spin_speed` 直通修复 + sentry_spin 模式"是我们的设计，不是抄来的**；
> 8. **换 LIO 有硬门槛**：`small_point_lio` 的 PointCloud2 适配**要求 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段**（`src/lidar_adapter/livox_pointcloud2.h`），我们的仿真点云没有 ⇒ **不能直接替换**；它的 `frame_id/child_frame_id` 硬编码 `odom/base_link`，且 **twist 全零**（源码 TODO）。本文 §7 第 4 条"先用 `lio:=pointlio` 顶替"仍然成立，但"vendor small_point_lio"要重新评估成本。

## 0. 总原则（五条）

1. **只"增槽"，不"换默认"**：COD_NAV 的做法一律作为**新槽位值/新候选**加入；默认路径（rpp + cartographer + amcl）保持不变，随时可回退。
2. **一次一个变量 + P0 回归做闸门**：每步都用 `tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5` 验收；不通过就回退，不叠加下一步。
3. **先契约，后算法**：TF / 时间戳 / QoS / 消息类型先对齐（前两天 6 个病根全是契约问题），再谈换算法。
4. **不引入非必要依赖**：能用现有包（`nav2_smac_planner`、`nav2_mppi_controller`、`spatio_temporal_voxel_layer` 都已安装）就不新编译。
5. **每步写回退方式**：文档里必须写清"怎么退回去"。

## 1. 目标架构对照（轴 × 现状 × COD_NAV 2026 × 落地方式）

| 轴 | 我们现状 | COD_NAV 2026 | 我们的落地方式 | 优先级 |
|---|---|---|---|---|
| LIO | `lio:=fastlio|pointlio|none|cartographer` | **`small_point_lio`（仓库内自带；2026 分支没有 FAST-LIO）** | 先用已有的 `lio:=pointlio`（同族、零成本）；**直接 vendor 有门槛**：它要求点云带 `tag`+`timestamp` 字段（见深挖 §E/§F） | P3 |
| 在线建图 | `mapper:=cartographer|slam_toolbox` | **slam_toolbox async `mode: lifelong`** | `mapper` 增加取值 `slam_toolbox_lifelong`（只改参数文件，不新增包） | P3 |
| 重定位 | AMCL / ICP / slam_toolbox / cartographer | **无（静态 `map→odom`）** | **不采用**（我们四种更强）；只在"纯在线建图"模式下允许静态桥 | — |
| 3D 点云预处理 | 无（直接用 `/segmentation/obstacle`） | **`cpp_lidar_filter` 车体裁剪盒**（x±0.3, y−0.3~0.5, z−0.1~0.2, `negative:true`, leaf 0.05） | 新增 **`rm_cloud_crop`**（或先用 `pcl_ros` 的 `PassThrough`×3 组合）→ `/livox/lidar_filtered`，**只接 stvl 与 local cloud 层**；`/scan` 链不动 | **P0** |
| `/scan` 射程与高度带 | `range_min 0.05`、`range_max **10.0**`、带在雷达面**之下**(−1.0~0.1) | `range_min 0.5`、`range_max **20.0**`、带在雷达面**之上**(0.01~1.00) | `range_max → 20.0`（P0）；高度带**必须按我们传感器重算**（我们的墙仅 0.40 m、近场回波 0.10~0.196 m ⇒ 照抄会丢矮墙） | **P0** |
| 全局规划 | NavFn（`allow_unknown: true`） | Smac2D + `cost_travel_multiplier 4.0` + `tolerance 0.5`（2025 是 SmacHybrid DUBIN） | `planner` 槽：`navfn`(默认) / `smac2d` / `smac_hybrid`；Smac2D 参数抄上游、`allow_unknown` 保持 true | P1 |
| 路径平滑 | `SimpleSmoother` | **SmacPlanner2D 的 `smooth_path: true` + 内建 smoother**（他们的 `smoother_server` 在实时链里是**死代码**，BT 无 SmoothPath） | 先只打开 Smac 的 `smooth_path`；`smoother:=savgol` 降为可选 | P1 |
| 局部障碍表示 | local: `scan|cloud|both`（无时间维） | **local 也用 STVL**（`voxel_decay 0.5`、`voxel_size 0.05`、`obstacle/raytrace 8/9 m`、`min_h 0.1 / max_h 1.0`） | `local_obstacle` 增加取值 `stvl`；先只加"衰减"这一条，不动其它层 | **P0** |
| 局部控制 | `nav:=rpp|dwb|teb` | **MPPI Omni 50 Hz**（vx/vy 7.5 m/s，critics：GoalCritic 15/2.5、CostCritic `critical_cost 253`+`consider_footprint`、PathFollow/PathAlign `threshold 1.5`、`temperature 0.25`、`gamma 0.008`、60 步/2000 采样） | `nav:=mppi`（包已装）→ 独立 params 文件，**速度从 2.0 m/s 起调**，`controller_frequency 30` 起 | P2 |
| 任务层 | stock BT + 自研分段工具 | BT：**3 Hz 重规划** + `IsStuck→BackUp(1 m@1.5 m/s)` + 10 次重试；目标由 bash 轮询裁判串口 | 先**只借 BT 结构**（若确认有利再改 XML）；目标选择仍用我们的方式 | P4 |
| 哨兵语义 | `spin_speed==0` 直通（已修）；`spin_speed!=0` 小陀螺 | **`enable_rotation: false` + `yaw_goal_tolerance 6.28`**（只控位置、朝向交给自转） | 新增**模式**（非新包）：`sentry_spin` 组合（`spin_speed!=0` + yaw 容差放宽），与位置控制解耦 | P4 |
| 未知区策略 | `allow_unknown: true`；`track_unknown_space`(根层级疑惰性) + stvl 内 true | 同样 `true` 双开，但图是 72×23 m 大面积 unknown | **不学**（他们更糟）；我们按 `debug_fastlio_cartographer.md §10.5 遗留⑤` 单独定策略 | — |

## 2. 分阶段路线（每阶段：目标 → 改动 → 判据 → 风险 → 回退）

### P0（直击当前痛点，风险最低）
- **改动**：① 车体裁剪盒 → `/livox/lidar_filtered` → stvl + local cloud；② `p2l.range_max: 10 → 20`；③ local costmap 增加 STVL（带 `voxel_decay`）。
- **判据**：`cloud_z_profile.py` 的 0–0.5 m 桶不再是空的；贴墙时 `inf` 探针在该方位有回波；`costmap_marking_check.py` 近距标记不下降；P0 回归 PASS；`range_max 20` 后 `inf` 占比明显下降。
- **风险**：裁剪盒太紧会删真障碍（须按我们车体标定，尤其 y 方向不对称要重算）；STVL 清除过激会擦薄结构（range 先给保守 8/9 m）。
- **回退**：裁剪盒节点停用即恢复原链路；`range_max` 与 STVL 都是单参数/单层，改回即退。

### P1（规划质量）
- **改动**：`planner:=smac2d`（`cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown: true`）+ `smoother:=savgol`。
- **判据**：同一目标点的路径是否居中于代价谷底（不再贴缝走）；`velocity_smoother` 饱和次数下降；规划耗时 < `max_planning_time`；P0 回归 PASS。
- **风险**：Smac 比 NavFn 慢（我们地图 15×28 m 比他们 72 m 小，风险较低）；SG 平滑可能切角 ⇒ 必须核 footprint 碰撞。
- **回退**：`planner:=navfn`、`smoother:=simple`。

### P2（控制器）
- **改动**：`nav:=mppi`（Omni，30 Hz 起，速度 2.0 m/s 起，critics 抄上游配方）+ `velocity_smoother` 匹配（限速/限加速度）。
- **判据**：贴墙最小距离、窄缝通过率、震荡幅度、到达时间、CPU；与 rpp/dwb 三方对照写入 `algorithm_matrix.md §四`。
- **风险**：`batch_size 2000 × time_steps 60` 每周期 ⇒ CPU 高；参数多，须先跑默认再微调。
- **回退**：`nav:=rpp`。

### P3（LIO + 在线建图一条龙）
- **改动**：`mapper:=slam_toolbox_lifelong`（只加参数文件）；评估是否引入 `small_point_lio` 作为 `lio` 新槽。
- **判据**：长走廊/退化场景的漂移；`/map` 生长是否跟得上；与 cartographer 对照。
- **风险**：lifelong 模式与**任何重定位槽冲突**（都发 `map→odom`）⇒ 必须与 `localization` 互斥（launch 层强制）。
- **回退**：`mapper:=cartographer`。

### P4（哨兵语义 + 任务层）
- **改动**：`sentry_spin` 模式（`spin_speed!=0` + `yaw_goal_tolerance` 放宽）；按需借 BT 结构（3 Hz 重规划 / IsStuck→BackUp）。
- **判据**：目标点位置到位即可（朝向由自转负责）；卡住恢复成功率；与"只控朝向"模式对比。
- **风险**：朝向安全变成**另一个独立问题**（要有单独的朝向风险判据）；背退速度 1.5 m/s 在拥挤场地可能触发自身避障。
- **回退**：恢复 `spin_speed:=0` 直通模式（已验证可行）。

## 3. 兼容性 / 冲突矩阵

| 组合 | 是否冲突 | 说明 |
|---|---|---|
| `mapper:=slam_toolbox_lifelong` × `localization:=*` | **冲突** | 两者都发 `map→odom` ⇒ 必须互斥（沿用 launch 层条件） |
| 静态 `map→odom`（COD_NAV 2026 做法） × AMCL/ICP | **冲突** | 只在"纯在线建图、无重定位"模式下才允许 |
| `nav:=mppi` × `spin_speed!=0` | **需设计** | MPPI 会输出 `wz`，而 `fake_vel_transform` 在 `spin_speed!=0` 时用 `spin_speed` 替换它 ⇒ 二者择一：要么 `spin_speed=0` 让 MPPI 控朝向，要么把朝向交给自转（`yaw_goal_tolerance` 放宽） |
| `smoother:=savgol` × `nav:=teb` | **冗余** | TEB 自带轨迹优化 ⇒ sg 只配 rpp/dwb/mppi |
| 车体裁剪盒 × `linefit` | **不冲突** | 裁剪盒只作用于 3D 链路（stvl/local cloud）；`/scan` 仍走 linefit |
| 高度带改到雷达面之上 × 我们场地矮墙 | **有风险** | 我们墙高 0.40 m、近场回波 0.10~0.196 m ⇒ 带必须按我们传感器重算，照抄 COD_NAV 会丢矮墙 |
| `range_max 20` × cartographer 在线图 | **需观察** | 射程变远 ⇒ 图更大、单帧点数更多 ⇒ 关注 RTF 与 `num_range_data` 的相对关系 |

## 4. 新增槽位的参数骨架（标注来源，**待我们标定**）

```yaml
# nav:=mppi  （抄自 COD_NAV rmul2026，速度已按我们仿真下调为待定）
controller_frequency: 30.0        # 他们 50.0；我们先用 30
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"
  time_steps: 60
  model_dt: 0.05
  batch_size: 2000
  vx_max: 2.0                     # 他们 7.5，待标定
  vy_max: 2.0
  wz_max: 2.5
  temperature: 0.25
  gamma: 0.008
  critics: ["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]
  CostCritic: {critical_cost: 253.0, consider_footprint: true, weight: 3.0}
  GoalCritic: {weight: 15.0, threshold_to_consider: 2.5}
  PathFollowCritic: {weight: 5.0, threshold_to_consider: 1.5}
  PathAlignCritic: {weight: 10.0, threshold_to_consider: 1.5}
# planner:=smac2d（抄自 COD_NAV 2026；我们的地图小得多，耗时风险低）
GridBased: {plugin: "nav2_smac_planner/SmacPlanner2D", allow_unknown: true, tolerance: 0.5, cost_travel_multiplier: 4.0}
# smoother:=savgol
smoother_plugins: ["SmoothPath"]
SmoothPath: {plugin: "nav2_smoother::SavitzkyGolaySmoother", do_refinement: true, refinement_num: 2, enforce_path_inversion: true}
# local_obstacle:=stvl（先只加衰减这一条）
local_costmap: {plugins: [obstacle_layer, obstacle_cloud_layer, stvl_layer, inflation_layer]}
stvl_layer_local: {voxel_decay: 0.5, voxel_size: 0.05, obstacle_range: 8.0, raytrace_range: 9.0, min_obstacle_height: 0.1, max_obstacle_height: 1.0, model_type: 1}
# 车体裁剪盒（negative passthrough；数值须按我们车体重标）
crop_box: {min_x: -0.30, max_x: 0.30, min_y: -0.30, max_y: 0.50, min_z: -0.10, max_z: 0.20, negative: true, leaf_size: 0.05}
```

## 5. 明确**不做**的事

1. 不引入 ESDF / 动态障碍跟踪与预测（**他们也没做**，不是这次的收益点）；
2. 不采用他们 72 m 大面积 unknown 图与"静态 `map→odom`"作为默认；
3. 不学他们的工程卫生问题（硬编码绝对路径、提交 `cmake-build-debug/`/.swp、死配置块、`odom_topic` 拼错）；
4. **不照抄速度**（7.5 m/s 是实车参数，仿真里不可直接移植）；
5. 不在未过 P0 回归的情况下叠加两个以上变量。

## 6. 验收与记录方式

- 每阶段结束：`nav_smoke_regression.py`（PASS/FAIL + JSON 快照）→ `algorithm_matrix.md §四` 加一行 → 本文件对应阶段打勾；
- 链路体检：`tools/scripts/diag/watch_startup_chain.py`；
- 若出现新坑：写进 `debug_fastlio_cartographer.md §9.0 总表`（症状 → 判据 → 处置 → 状态）。

---

## 7. 关于"2026 用的是不是 FAST-LIO"——已核实（2026-09-26）

抓 `https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026` 得到该分支**根目录**：

```
.github/  .gitignore  .idea/  CLAUDE.md  LICENSE  README.md
cpp_lidar_filter/          ← 车体裁剪盒（自带包）
fake_vel_transform/        ← 小陀螺/云台解耦（自带包）
nav_bringup/               ← launch/params/map/BT
pointcloud_to_laserscan/   ← 自带 fork
resource/
small_point_lio/           ← ★ LIO 就是它：**仓库内自带目录，不是 submodule**
```

**结论**：
1. **`rmul2026` 的 LIO = `small_point_lio`，分支里没有 `FAST_LIO/` 目录** ⇒ **2026 路线不用 FAST-LIO**（FAST-LIO 只留在 `master`/2025 那套）；
2. 该分支**没有 `.gitmodules`**（我抓 `.../rmul2026/.gitmodules` 得 404），而 `master` 有 4 个 submodule（patchwork-plusplus、pcd2pgm、pb_omni_pid_pursuit_controller、pb_nav2_plugins）⇒ **2026 是一次"做减法"的重构**：去掉地面分割（改高度带）、去掉先验图与全部重定位（纯在线 lifelong）、去掉 pb_omni（改 MPPI）、去掉 patchwork++/pcd2pgm/pb_nav2_plugins。**整个工作区只剩 5 个包**（上面 5 个目录）；
3. 他们 2026 的文档主要在 **`CLAUDE.md`（5.3 KB）**，`README.md` 只有 1.1 KB ⇒ 想看他们的设计意图，读 `CLAUDE.md`；
4. **我们的最短对齐路径**：我们工作区**已经有 Point-LIO**（`src/rm_localization/point_lio`，槽位 `lio:=pointlio`）——`small_point_lio` 属同一族（名字即 "Small Point-LIO"），所以**先用 `lio:=pointlio` 做等效验证，零引入成本**；确有效益再考虑 vendor 他们的 `small_point_lio`（那是 P3，且要先看它的 license 与依赖）。
5. 顺带修正一条认知：**"2026 架构"不能整体照搬** —— 它同时**砍掉了重定位**（`map→odom` 静态）和**地面分割**，这两条在我们这里是资产（四种重定位可选、linefit 可测），**只借它做加法的部分（裁剪盒/STVL 双图/MPPI/Smac/SG 平滑）**。

---

## 8. 融入可行性分析（结论：**架构不用改，只加"候选"**）

判据来自我们架构的三条硬约束：① 槽位化（同一链路换算法做 A/B）；② 契约固定（TF 链、`robot_base_frame: base_link_fake`、仿真钟、cmd_vel 转换）；③ 先验图与在线建图两条路都要保留 + 仿真要能验收。

| COD_NAV 2026 组件 | 判定 | 关键理由 |
|---|---|---|
| `cpp_lidar_filter` 车体裁剪盒 | **✅ 直接融** | 纯 ROS 节点，进出都是 PointCloud2；**必须按我们的 `livox_frame` 重算 box**（他们的 marker 在 `base_link` 是 bug）；`leaf_size` 是死参 ⇒ 要降采样得自己加 |
| STVL 双图 + 衰减 | **✅ 直接融** | 包已装、纯 YAML；照抄 `decay_model 0`、`voxel_decay 0.5`、`voxel_size 0.05`、**`model_type: 1`**、`filter: voxel`、`obstacle/raytrace 8/9 m` |
| `range_max: 10 → 20` | **✅ 直接融** | 一行参数；我们地图 15×28 m 比他们 72 m 小，耗时/RTF 风险低 |
| 高度带 `min/max_height` | **⚠️ 必须重算，禁止照抄** | 他们的 `min_height 0.10/0.15`（livox_frame）**会砍掉我们近场回波**：实测近场 `z_map 0.107~0.196` ⇒ `z_livox ≈ −0.12~−0.03`，**全在 0.10 以下** ⇒ 照抄等于把近场墙全删（= 又制造"缝"） |
| Smac2D（`cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown true`） | **✅ 直接融** | `nav2_smac_planner` 已装；注意 `motion_model_for_search`/`minimum_turning_radius`/`angle_quantization_bins` 是 **Hybrid 专用**，Smac2D 下是惰性键（别抄） |
| Smac 的 `smooth_path: true` + 内建 smoother | **✅ 直接融** | 这才是他们真正的平滑（`smoother_server` 在他们链里是**死代码**，BT 无 `<SmoothPath>`）⇒ 比接 savgol 便宜 |
| MPPI Omni + 他们的调参史 | **✅ 融（成本中等）** | 包已装；需新 params + `nav:=mppi` 槽 + velocity_smoother 匹配。**速度必须降**（7.5 m/s 是实车）；**与自转模式互斥**（见下） |
| BT 结构（3 Hz 重规划 / BackUp） | **△ 要自己写** | 他们 XML 里**没有 `IsStuck`**，`BackUp` 只在 through-poses 且是 **1.0 m @ 1.0 m/s** ⇒ 借结构得自己加节点，收益一般 |
| `small_point_lio` | **❌ 不能直接融** | 其 PointCloud2 适配**要求 `tag`(uint8)+`timestamp`(float64 秒)** 字段（我们点云没有）；`twist` 全零、帧硬编码。**替代：用我们已有的 `lio:=pointlio`（同族）先验证收益** |
| `map_resolution 0.5` / PCD 写编译期源码目录 | **❌ 不该融** | 写死路径 + 往源码目录写文件，不适合可复现的 bench |
| slam_toolbox `lifelong`（纯在线） | **△ 可作为 `mapper` 新取值** | 但必须**让它发 `map→odom`**（他们设 `transform_publish_period 0.0` = 不发 ⇒ 才需要静态桥）；与重定位槽**互斥** |
| 静态 `map→odom`(z=0.05) + 零重定位 | **❌ 不融（与核心资产冲突）** | 我们 `mode:=nav` 的意义就是"先验图 + 重定位"；静态桥要求机器人**从图原点原朝向起飞**（他们图仅 10.65×10.2 m）。最多作为 slam_nav 调试的临时开关 |
| 去掉 `linefit`（改高度带） | **❌ 不该融** | linefit 是我们可测/可 A/B 的资产（z-profile 诊断就靠它），且他们的高度带会砍掉我们近场回波 |
| `enable_rotation: false` + `yaw_goal_tolerance 6.28` | **△ 作为我们自己的模式** | **注意**：他们**从不自转**（`spin_speed` 全分支无一处非零）⇒ "他们为自转而这样做"是错的推断；真实作用只是"目标朝向随便"。`sentry_spin` 是我们的设计 |
| 未知区 / 动态障碍 / ESDF / 前沿探索 | **—（无内容可融）** | 他们全分支 grep 0 命中；得我们自己定策略 |

### 融入的三个前置冲突（必须先解决，否则会重演"指令被静默替换"）

1. **`wz` 归属冲突**：MPPI（或任何控朝向的控制器）要控 `wz` ⇒ 必须 `spin_speed=0`；要自转 ⇒ 必须放弃控制器控朝向。**launch 层必须互斥**，不能两个都开。
2. **`map→odom` 双发布者**：`localization:=*`（四种）与 `mapper:=slam_toolbox_lifelong` 都会发 ⇒ `mapper` 新取值必须加进 launch 的互斥条件。
3. **高度带标定**：`min_height`/`max_height` 必须按我们传感器的**实测 z 剖面**重算（近场 `z_livox < 0.10`），否则融入即制造新"缝"。

### 一句话结论

**能融，但"融入"的正确形状不是搬他们的架构，而是把他们 6~8 个组件作为我们槽位的"新取值/新参数集"加进来**：默认路径（rpp + cartographer + amcl + linefit）一个字不动，随时可 A/B 与回退。**不能融的三样**（静态 `map→odom` 零重定位、去掉 linefit、直接 vendor `small_point_lio`）都触及我们的核心资产或硬门槛。
