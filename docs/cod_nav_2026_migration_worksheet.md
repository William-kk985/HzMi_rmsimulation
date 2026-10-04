# COD_NAV(2026) → 我们 bench 的移植工单（**只登记，不动手**）

> 依据：`docs/cod_nav_2026_params_review.md`（773 行、34 表、逐条 file:line）、`docs/cod_nav_2026_deep_dive.md`、`docs/cod_nav_2026_integration_plan.md`
> 参考代码已就位：`third_party/cod_nav_2026/`（163 文件，已在 `.gitignore`，`third_party/COLCON_IGNORE` 保证不参与构建）
> 记号：S=`singlenav2_params.yaml`，M=`multiplenav2_params.yaml`，GAC=`goal_approach_controller`，FVT=`fake_vel_transform`
> **状态**：全部 `TODO`。每条都要"一次一个变量 + P0 回归验收 + 可单独回退"。

## A. 可直接用（8 条中的 6 条已确认，余 2 条见 §E）

| # | 抄什么 | 来源 | 落到我们哪里 | 风险 | 回退 | 状态 |
|---|---|---|---|---|---|---|
| 1 | **`PositionGoalChecker`**（只看位置：xy + path_length，不看朝向） | S goal_checker 段 | `params/nav2_params_sim_*.yaml` → `controller_server.goal_checker_plugins` + 新块 | 低（1.1.20 确认有此插件）；与我们 `spin_speed` 模式天然匹配 | 换回 `goal_checker`（xy+yaw） | TODO |
| 2 | **Smac2D 的 9 个有效键** + `smoother.*` 子块（**跳过 16 个 Hybrid/Lattice 无效键**） | S planner_server / N2-smac2 | 新增 `planner:=smac2d` 槽对应的 yaml 块 | 低-中：A* 比 NavFn 慢（我们图 15×28 m，风险小） | 回到 NavFn | TODO |
| 3 | **`velocity_smoother` 键名 + `cmd_vel` remap 链** | S（与我们逐字相同） | 对照现有 velocity_smoother 段 | 低 | 逐键回退 | TODO |
| 4 | **`map_saver` 段** | S（逐字相同） | 我们的建图保存链 | 低 | 无 | TODO |
| 5 | **STVL 体素层参数块**（`decay_model 0`/`voxel_decay 0.5`/`voxel_size 0.05`/`model_type 1`/`filter voxel`/`obstacle 8.0`/`raytrace 9.0`）—— **别抄 `livox_filtered`、realsense 源** | S stvl 段 | `local_obstacle:=stvl`（local 也上 STVL） | 中：清除过激会擦薄结构 | 移除 local stvl 层 | TODO |
| 6 | **航点 CSV 格式 + 两个桥节点**：`waypoint_to_nav2`(FollowWaypoints 逐点停) / **`waypoint_through_nav2`(NavigateThroughPoses 连续穿越)** | `wps/*.csv` + `waypoint_editor/` | 长目标分段（路线 B）—— **我们的默认 BT 已支持 through-poses** | 中低；坑：`waypoint_pause_duration` 单位是**毫秒**；`stop_on_failure: false` 会**静默跳过失败点** | 停用桥节点，回到 `segment_goal_navigator.py` | TODO |

> 余 2 条以 `docs/cod_nav_2026_params_review.md §E`（"可直接用 8 条"）为准。

## B. 需改造（12 条里的关键 5 条）

| # | 项 | 必须先改什么 | 风险 | 状态 |
|---|---|---|---|---|
| 1 | **MPPI（`nav:=mppi`）** | ① `model_dt` = 1/`controller_frequency`；② `consider_footprint: false`（或先加 footprint）；③ `min_y_velocity_threshold → 0.001`；速度从 2.0 m/s 起 | 中：CPU + 调参 | TODO |
| 2 | **`goal_approach_controller`** | 修**帧 bug**（`goal_`(map) 与 `pose`(odom) 混帧相减）；加 **spin 开关**（否则末端 2 m 内静默关掉小陀螺） | 中高 | TODO |
| 3 | 裁剪盒 `cpp_lidar_filter` | 按**我们的 `livox_frame`** 重算 box（他们的 marker 在 `base_link` 是 bug；`leaf_size` 是死参） | 中 | TODO |
| 4 | 高度带 `min/max_height` | **按我们实测 z 剖面重算**（近场 `z_livox < 0.10`，照抄他们的 0.10/0.15 会砍掉近场墙 ⇒ 又制造"缝"） | 中 | TODO |
| 5 | `pb_nav2_behaviors/BackUpFreeSpace` | 需额外编译 `pb_nav2_plugins` | 低-中 | TODO |

## C. 不适用（明确"不跟"）

1. **`rolling_window: true` + 25×25 窗 + `allow_unknown: true`**（图外未知区可规划）—— 与我们"未知=不可走"的安全几何**方向相反**；
2. **他们的 BT**（删了 `Spin`/`Wait`，因为他们的 FVT 让 Spin 失效）—— **我们的 FVT 有修复，`Spin` 可用，不能照抄**；
3. **静态 `map→odom` + 零重定位** —— 撞我们 `mode:=nav` 的核心资产；
4. **直接 vendor `small_point_lio`** —— 需 `tag`+`timestamp` 字段、twist 全零；
5. `map_resolution 0.5` / 硬编码 `/home/cod-sentry/...` 路径 / `auto_save_map.launch.py`；
6. 各类 Jazzy/main 专有键（见 §10.4 第 4 条）。

## D. 落地顺序（建议）

```
① PositionGoalChecker（最低风险，先验"只控位置"是否合我们）
② Smac2D 有效键 + smoother 子块
③ waypoint_through_nav2 承载分段（替代自研分段循环）
④ MPPI（先做三处硬约束改造）
⑤ goal_approach_controller（先修帧 bug + spin 开关）
── 每步：nav_smoke_regression.py 验收 → algorithm_matrix.md §四 记一行 → 本表状态改 DONE/REVERTED
```

---

## E. 决策记录（2026-09-26，用户确认）

### E1. 静态 `map→odom`：**不采用**（明确不做）
理由：撞我们 `mode:=nav` 的核心资产（先验图 + 4 种重定位）；作者原话也承认它只在「环境简单、面积小、误差容忍度大」时成立。仅保留"纯在线建图无重定位"这一档（`mode:=nav + localization:=''`，我们本来就有）作为**调试对照**，不作为方案。

### E2. `linefit` 地面分割：**保留**（不抄 COD 的"去掉"）
COD 2026 去掉 patchwork++/地面分割的原因（有据）：① 2026 改由**两层 STVL 直接吃 3D 点云**，地面点由 STVL 的 `min_obstacle_height: 0.1` + p2l 的高度带一起挡掉，不再需要专门的地面分割器；② 做减法/减依赖（他们 2026 主线是"提速"）；③ 固定安装高度 + 场地基本水平 ⇒ 固定高度带比在线地面拟合更可预测。
**我们不该抄的原因**：① 实测我们**近场回波 `z_livox ≈ −0.12~−0.03`，全在 0.10 以下** ⇒ 照抄他们的 `min_height 0.15/0.10` 会把近场墙**整段砍掉**（= 又制造"缝"）；② linefit 是我们**可测/可 A/B 的资产**（`cloud_z_profile.py`、`costmap_marking_check.py` 都建立在它输出的 `/segmentation/obstacle` 上）；③ 场地若有坡道/台阶，高度带假设直接失效。

### E3. `small_point_lio`：**要，且提升为 `lio` 槽位的同级取值**
目标形态：`lio: = fastlio | pointlio | **smallpointlio** | none | cartographer`（与 fastlio/pointlio 同级，可 A/B）。
落地清单（P3）：
1. 代码位置：`src/rm_localization/small_point_lio`（上游 <https://github.com/Yancey2023/small_point_lio>，MIT，分支 `ros2`；**不要放 `third_party/`**，那里有 `COLCON_IGNORE` 不会被构建）；
2. TF 契约：它直接发 `odom→base_link` ⇒ 与 fastlio 的 `camera_init→body` + `lio_tf_adapter` 路径不同，launch 里必须**按槽位切换**（选 smallpointlio 时不经 adapter）；
3. **硬门槛（必须先解决）**：其 PointCloud2 适配器要求点云带 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段（`src/lidar_adapter/livox_pointcloud2.h`，只保留 `(tag & 0b00111111)==0`），并硬编码 `frame_id/child_frame_id = odom/base_link`；我们的仿真点云没有这两个字段 ⇒ 需要写一个**适配器节点**补 `tag=0` 与**逐点 timestamp**（逐点时间正是 Point-LIO 系的价值所在，必须从我们雷达插件的扫描时序/CSV 图案里合成，不能简单全填同一时间）；
4. 配置：`mid360_sim.yaml`（`lidar_type: livox_pointcloud2`、`map_resolution` 与 `save_pcd` 要改掉他们的 0.5/编译期写死路径）；
5. 已知短板：仓库里 `twist` 六行**仍是注释**（无速度输出）；`save_pcd` 写到 `@CMAKE_CURRENT_SOURCE_DIR@`；
6. 验收判据：`/Odometry`（或等价）10 Hz 出数 + `odom→base_link` TF 连续 + `lio:=smallpointlio` 下 **P0 回归 PASS** + 与 fastlio/pointlio 的**漂移对比**（同一条轨迹的 end-to-end 误差）。

---

## F. 候选实验记录：**两层 STVL**（全局 + 局部都上 `spatio_temporal_voxel_layer`）　【用户标记：后面可以尝试】

> 来源：COD_NAV 2026（`singlenav2_params.yaml` / `multiplenav2_params.yaml`，两张 costmap 的 `plugins` 里都有 STVL）
> 状态：**候选（TODO，未实施）**。与 §A-5（STVL 参数块）、§B-3（裁剪盒）、§E2（linefit 保留）互为引用。

### F1. 他们的做法（参数）
`voxel_decay 0.5`（线性衰减，`decay_model 0`）· `voxel_size 0.05` · `observation_persistence 0.0` · `combination_method 1` · `track_unknown_space true` · **`model_type: 1`（3D 雷达）** · `vertical_fov_angle 2.00` / `horizontal_fov_angle 6.28` · `filter: "voxel"` · `transform_tolerance 0.2`；**层级 `obstacle_range 3.0` 被源级 `8.0/9.0`（obstacle/raytrace）覆盖**；`min_obstacle_height 0.1`、`max_obstacle_height 1.0`。

### F2. 两层 STVL「导致」了什么（连带效应 —— 这才是值得记的部分）
1. **给局部加了时间维**：障碍体素 **~0.5 s 自动消失** ⇒ 局部不再累积"幽灵障碍"。我们现在 local **没有时间维**（只靠 `obstacle_layer` 的射线清除），这是与他们的实质差距。
2. **局部也能吃原生 3D 点云**：STVL 是体素层 ⇒ 局部不必只依赖 p2l 的 2D 投影（对应我们 `local_obstacle:=cloud` 那条路的"升级版"）。
3. **这是他们能删掉地面分割的前提之一**：地面点由 STVL 的 `min_obstacle_height` + p2l 高度带共同挡掉 ⇒ 少一个独立模块（与 §E2 相连）。
4. **必须与"车体裁剪盒"配套**：**STVL 没有"最小障碍半径"概念**（他们 `box_lidar_filter` README 原话）⇒ 自身点云只能先从点云里抠掉。**「两层 STVL + 裁剪盒」是一套，不能只上其一**，否则车身自身点会进体素图。
5. **代价**：两张图都做体素化+射线清除 ⇒ CPU 上升；清除过激会擦薄结构；**与 static/obstacle 层的写序耦合** —— 我们已经见过"缝"就是后写层的清除覆盖了 static 层写下的墙（见 `debug_fastlio_cartographer.md`），两层 STVL 会**增加一个清除来源**。

### F3. 我们的现状与尝试方案（一次一个变量）
- 现状：**global 已有 STVL**（`min_obstacle_height 0.0`、`voxel_decay 0.5`）；**local 没有**，是 `[obstacle_layer, obstacle_cloud_layer, inflation_layer]`；`stvl_layer.min_obstacle_height` 全局已为 **0.0**（对应我们近场 `z_livox < 0.1` 的实测，**这正是不能照抄他们 0.1 的地方**）。
- 方案（顺序不可颠倒）：**① 先上裁剪盒**（§B-3，按我们 `livox_frame` 重算 box）→ **② 再给 local 加 `stvl_layer`**（保守：`obstacle_range 8.0` / `raytrace_range 9.0` / `voxel_decay 0.5` / `voxel_size 0.05` / **`min_obstacle_height 0.0`（不是 0.1）** / `max_obstacle_height 1.0` / `model_type 1`），图层顺序建议 **`stvl` 放在 `obstacle*` 之后、`inflation` 之前**，并观察"缝"是否恶化。
- 判据：**P0 回归 PASS**（能走到目标）· 幽灵障碍消失时间（人放一个临时障碍后 ~0.5 s 内清掉）· 墙面**没有变薄/裂缝变宽** · CPU/RTF 无明显下降。
- 回退：从 local `plugins` **移除一行**即恢复；或用 `local_obstacle:=cloud`（保留点云、不加时间维）。
- 关联风险：若"缝"恶化 ⇒ 说明多了一个清除来源 ⇒ 转去查 `inf_is_valid`（他们 `obstacle_layer.inf_is_valid: true` 与我们一致）与图层顺序。

---

## G. 关于 `small_point_lio`（§E3）的排期约定：**可以后面补，且不阻塞 D3/D4**

**为什么可以延后**：`lio` 是**独立轴** —— Nav2 那一侧只依赖三个契约：`/odom`、`odom→base_link`、`/scan`（由 linefit+p2l 从 `/livox/*` 产出，与该轴无关）。换/加 LIO 是**换生产者**，不动消费者 ⇒ 只要槽位契约不变，随时可加。

**排期约定（用户确认口径）**：
1. **D3 / D4 一律用已验证的 `lio:=fastlio` 验收**（保持"一次一个变量"，让 D3/D4 的结论干净）；
2. `lio:=smallpointlio` 落地后，**D3/D4 的验收项复跑一遍**（里程计变了，控制器/规划器的最优参数可能要微调），并做**同轨迹端到端漂移对比**（fastlio vs pointlio vs smallpointlio）；
3. **不阻塞**：三者互不依赖；只有一种情况需要提前上它 —— **D3/D4 的失败模式被怀疑来自里程计**（判据：`/odom` 话题频率正常但位姿漂移大；或 `spin_speed != 0` 自转时定位抖动明显）。

**"风险高"的三条，其实都是可控/可延后的工程量，不是架构风险**：
| 风险 | 性质 | 低成本先行做法 |
|---|---|---|
| 逐点 `timestamp` 需合成 | **工作量**（其价值所在，必须做对） | 先做**退化版**（扫描内按 CSV 图案顺序/均匀插值合成逐点时间）跑通链路，再逐步精确 |
| TF 契约切换（它直发 `odom→base_link`，fastlio 走 `camera_init→body` + adapter） | **launch 层一次性条件** | 按槽位切通路，写好即不再动 |
| 仓库短板（`twist` 全零、写死 `save_pcd` 路径、`map_resolution 0.5`） | **配置/patch 层** | 在我们自己的 `mid360_sim.yaml` 里全部覆盖 |

**零成本预研动作（随时可做，不动主链路）**：把 `small_point_lio` 当**独立进程**跑起来（不接 Nav2、不改 `lio` 槽位），只验证它能从我们的仿真点云产出 `/Odometry` + `odom→base_link` TF —— 即"**喂得进去**"。这一步就能把上面三条门槛里最难的那条（逐点时间）**提前证伪或证实**。

