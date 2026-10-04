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

---

## H. D3-① 实施记录：`planner:=smac2d` 槽位（2026-09-26，已提交）

**提交**：`d4e662b`（master 已推送）。**改动 2 个文件**：`src/rm_nav_bringup/launch/bringup_sim.launch.py`（+18/−1）、新增 `src/rm_navigation/rm_navigation/params/nav2_params_sim_rpp_smac2d.yaml`（435 行）。
**默认未变**：`planner` 默认 `navfn` ⇒ 解析出的 params 路径与改动前**完全相同**（用 `importlib` 加载 launch、对六种 `nav × planner` 组合实测过）⇒ **现有已验证路径零影响**。

**新 yaml 的单变量保证**：与 `nav2_params_sim_rpp.yaml` 的 `diff -u` **只有 1 个 hunk**（整段落在 `planner_server` 内）。
**写入的 12 个键**（每个都在**已安装的 1.1.20** 里核实过）：
`tolerance 0.5` · `allow_unknown true` · `downsample_costmap false` · `downsampling_factor 1` · `max_iterations 1000000` · `max_on_approach_iterations 1000` · `max_planning_time 5.0` · **`cost_travel_multiplier 4.0`** · `use_final_approach_orientation false` · `smoother.{w_smooth 0.4, w_data 0.1, do_refinement true}`。
**核实方法**（三重）：① `strings libnav2_smac_planner_2d.so`（插件用 `name + ".<key>"` 声明参数）；② 已装头文件 `types.hpp` 的 `SmootherParams::get()`；③ 交叉核对 `third_party/nav2/nav2_smac_planner/src/smac_planner_2d.cpp:64-92`（同版本 1.1.20）。
**按规则剔除**：`refinement_num`（1.1.20 **不存在** ⇒ 写了就是静默失效，这正是我们防的那类错）· `use_astar`（NavFn 专有）· `smooth_path` 与 16 个 Hybrid/Lattice/Jazzy 专有键（2D 插件里 strings 命中 0）。
（另注：`smoother.max_iterations`(默认 1000) 与 `smoother.tolerance`(默认 1e-10) 在 1.1.20 确实存在，本次未写；要复现 COD 的 10000 再补。）

### ⚠️ 新踩到的坑（值得记进 §10.4 通用规律）
**新增 params 文件后必须先重新 build**：`install/` 里**不会**因为 `--symlink-install` 就自动出现新文件（已核对 `install/rm_navigation/share/rm_navigation/params/` 里没有 smac2d 文件）⇒ 必须先
`colcon build --symlink-install --packages-select rm_navigation`
否则 launch 会因为找不到 params 文件而失败（表现为"改了配置没生效/启动报错"，极易误判为插件问题）。

### 待验收（未实测项）
实跑尚未进行 ⇒ **"Smac2D 插件运行时真的加载成功 + 路径形态变好"两点未经实测**（离线证据仅到 pluginlib 声明 + ament resource index 存在）。
验收命令：见下方 §H.1；**回退**：省略 `planner` 或 `planner:=navfn`（一行）。

### H.1 验收清单（用户执行）
```bash
colcon build --symlink-install --packages-select rm_navigation      # ← 新增文件必须先 build
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=amcl nav:=rpp planner:=smac2d spin_speed:=0.0 nav_rviz:=True
ros2 param get /planner_server GridBased.plugin               # 期望 nav2_smac_planner/SmacPlanner2D
ros2 param get /planner_server GridBased.cost_travel_multiplier   # 期望 4.0
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5
```
判据：① 两个 `param get` 正确；② 启动日志出现 `Configuring GridBased of type SmacPlanner2D ... tolerance 0.50, maximum iterations 1000000`；③ **P0 回归 PASS**；④ RViz 里 `/plan` **不再贴缝走、更居中**；⑤ 规划耗时可接受（用 `navigate_to_pose` 结果里的 `planning_time` 或 smoke 工具"用时"与 NavFn 对照）。


---

## I. 卡住恢复链核查（2026-09-26；**只读核查 + 用户执行步骤，本步不改任何代码**）

> 起因：车可能**开进** inflation/内切带后卡死（planner 把起点当障碍、controller 报无效控制）。Step 1 已把
> `robot_radius` 从 0.40 改回 0.22（commit `dc03e11`）。本节核查"改回之后，恢复链是否真的会救场"。
> 引用行号：params = `src/rm_navigation/rm_navigation/params/nav2_params_sim_rpp.yaml`（**step 1 之后**的行号）；
> `N2/xxx` = `third_party/nav2/nav2_mppi_controller` 等**同版本 1.1.20** 源码。

### I.1 我们实际加载的是哪棵 BT

- `bt_navigator` 段（params:61-100）**没有** `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`
  （全 params 文件 grep `bt_xml` 命中 0；launch 侧 `RewrittenYaml` 只重写 `use_sim_time` / `yaml_filename`，
  不注入 BT）⇒ **走 nav2 内置默认**。默认值的产生处：`N2 bt_navigator/src/navigators/navigate_to_pose.cpp:58-79`
  与 `navigate_through_poses.cpp:52-73`（`has_parameter(...)` 为假时 `declare_parameter` 成
  `get_package_share_directory("nav2_bt_navigator") + "/behavior_trees/<名字>.xml"`；已装
  `/opt/ros/humble/lib/libbt_navigator_core.so` 里的字符串正是这两个文件名）。
- **加载的精确路径**：
  - `NavigateToPose` → `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
  - `NavigateThroughPoses` → `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`
- 我们自己的两个 goal 发送端都**不填** `NavigateToPose.Goal.behavior_tree`
  （`tools/scripts/regress/nav_smoke_regression.py:235`、`tools/scripts/nav/segment_goal_navigator.py:134`）
  ⇒ 一定落到上面这棵默认树（运行期铁证见 I.4 的 `ros2 param get /bt_navigator default_nav_to_pose_bt_xml`）。

**内置树里有没有恢复元素**（两个文件逐字读完）：

| 元素 | 在不在 | 参数 / 重试次数 |
|---|---|---|
| `RecoveryNode`（外层 `NavigateRecovery`） | ✅ | **`number_of_retries="6"`** |
| `RecoveryNode`（`ComputePathToPose`） | ✅ | **1**（失败 → `ClearEntireCostmap` 全局） |
| `RecoveryNode`（`FollowPath`） | ✅ | **1**（失败 → `ClearEntireCostmap` 局部） |
| `RateController` | ✅ | `hz="1.0"`（to_pose）/ `hz="0.333"`（through_poses） |
| `ClearEntireCostmap` | ✅ | 4 处（2 处上下文恢复 + `ClearingActions` 里 local/global 各一） |
| `Spin` | ✅ | `spin_dist="1.57"`；`time_allowance` 默认 10.0（`N2 nav2_behavior_tree/plugins/action/spin_action.hpp:57`） |
| `Wait` | ✅ | `wait_duration="5"` |
| `BackUp` | ✅ | `backup_dist="0.30"` / `backup_speed="0.05"`；`time_allowance` 默认 10.0（`back_up_action.hpp:56-58`） |
| 恢复顺序（`RoundRobin`） | ✅ | `Sequence(ClearLocal,ClearGlobal)` → `Spin` → `Wait` → `BackUp`（`GoalUpdated` 在前，目标没变才做恢复） |

- `plugin_lib_names`（params:69-100）与树所需节点逐一对照：树用到的 13 个 BT 节点库**全部**在列表里，且
  `/opt/ros/humble/lib/libnav2_*_bt_node.so` 同名文件全存在（含 `nav2_recovery_node_bt_node` /
  `nav2_round_robin_node_bt_node` / `nav2_rate_controller_bt_node` / `nav2_clear_costmap_service_bt_node` /
  `nav2_back_up_action_bt_node` / `nav2_spin_action_bt_node` / `nav2_wait_action_bt_node`）
  ⇒ 树能被完整实例化，**不会**出现"节点未注册"的加载失败。

### I.2 `behavior_server` 配了没有 / 1.1.20 有哪些恢复插件

- **配了**（params:368-392，dwb/teb 同）：
  `costmap_topic: local_costmap/costmap_raw` · `footprint_topic: local_costmap/published_footprint` ·
  `cycle_frequency: 10.0` · `behavior_plugins: ["spin","backup","drive_on_heading","wait"]`
  （类型 `nav2_behaviors/Spin|BackUp|DriveOnHeading|Wait`）· `global_frame: odom` ·
  `robot_base_frame: base_link_fake` · `transform_tolerance: 0.1` · `simulate_ahead_time: 1.0` ·
  `max_rotational_vel: 3.0` · `min_rotational_vel: 0.4` · `rotational_acc_lim: 3.0`。
- 启动链：`bringup_sim.launch.py` → `rm_navigation/launch/bringup_rm_navigation.py` →
  `rm_navigation/launch/navigation_launch.py`；`behavior_server` 在 `lifecycle_nodes`（`navigation_launch.py:46`），
  非组合（`:182`）与组合（`:257` `behavior_server::BehaviorServer`）两条路都起 ⇒ **节点确实被拉起来**
  （Humble 节点名是 `behavior_server`，不是 Galactic 的 `recoveries_server`，params 里已注明）。
- **键名核对（对已装 1.1.20）**：`costmap_topic` / `footprint_topic` / `cycle_frequency` / `behavior_plugins` /
  `global_frame` / `robot_base_frame` / `transform_tolerance` 由 `N2 nav2_behaviors/src/behavior_server.cpp:34-57`
  + `include/nav2_behaviors/timed_behavior.hpp:119-122` 声明；`simulate_ahead_time` 由
  `plugins/drive_on_heading.hpp:235`（BackUp/DriveOnHeading）与 `plugins/spin.cpp:53-56`（Spin）声明；
  `max_rotational_vel` / `min_rotational_vel` / `rotational_acc_lim` 只被 Spin 读（`spin.cpp:58-71`）。
  **`local_frame` 在 1.1.20 不存在**（`strings libbehavior_server_core.so libnav2_*_behavior.so | grep -cx local_frame` = 0；
  `local_costmap_topic` / `global_costmap_topic` / `local_footprint_topic` 同为 0）⇒ 那是 Jazzy 键，我们没写是对的。
- 已装恢复插件（`/opt/ros/humble/share/nav2_behaviors/behavior_plugin.xml` +
  `/opt/ros/humble/lib/libnav2_*_behavior.so`）：**`Spin` ✅ · `BackUp` ✅ · `DriveOnHeading` ✅ · `Wait` ✅ ·
  `AssistedTeleop` ✅**（五个都在）。我们只加载前四个；`assisted_teleop` 未配（见 I.5-5）。
  动作名 = 插件 id ⇒ `/spin`、`/backup`、`/drive_on_heading`、`/wait`。

### I.3 恢复链能转的两个前提（一个已满足，一个是 Step 1 给的）

1. **`Spin`/`BackUp` 会被"当前位置已在碰撞态"一票否决**：两者每周期都做前向 `isCollisionFree()`
   （`N2 nav2_behaviors/plugins/spin.cpp:149-153`、`include/nav2_behaviors/plugins/drive_on_heading.hpp:165-169`），
   不通过就 `stopRobot()` + `RCLCPP_WARN("Collision Ahead - Exiting Spin / DriveOnHeading")` + 返回
   **`Status::FAILED`**。⇒ `robot_radius: 0.40` 时车一进带，`local_costmap/costmap_raw` 就把车自己标成内切
   ⇒ 每个恢复动作都立刻 FAILED（BT 日志上"一直在恢复"，车一动不动）——这就是那个死锁的第二个根。
   **Step 1 把半径改回 0.22 正是让这个前提成立**：车贴墙 ~0.25 m 时仍判无碰撞，恢复才有机会成功。
   两者是配套的：以后再放大 `robot_radius` 会**重新关掉**这条链。
2. **FVT 不会吃掉恢复的角速度**（与参考队的关键差异）：`fake_vel_transform.cpp:80-84` 有
   `spin_speed_ == 0.0` 的**直通分支**（角速度原样下发、且不做 TF 查询）；参考队的 FVT 无论 `spin_speed` 取值
   都把 `angular.z` 换成 `spin_speed`（其 launch 不传参 ⇒ 0.0）⇒ 他们的 `Spin` 转不动、只能从树里删掉
   （§C-2）。**我们 `spin_speed:=0.0` 时 `Spin` 与一切角速度指令有效 ⇒ 内置树的恢复可用，不能照抄他们的树。**
   ⚠️ 但 `spin_speed:=5.0`（默认）时 `fake_vel_transform.cpp:94` 仍会把 `angular.z` 替换成常量 5.0 rad/s：
   底盘确实转，但**不是** Spin 行为闭环控制的那 1.57 rad ⇒ **验证恢复链请用 `spin_speed:=0.0`**（也正好是 §H.1 口径）。

**恢复指令从哪出去**（看错话题会以为"没有倒车速度"）：`navigation_launch.py` 里 `behavior_server` **只 remap 了 TF**，
它直接发 **`/cmd_vel`**、**绕过 `velocity_smoother`**（smoother 是 `cmd_vel→cmd_vel_nav` 进、`cmd_vel_smoothed→cmd_vel` 出，
`:219-220` / `:278-279`），再进 FVT（`/cmd_vel` → `/cmd_vel_chassis`）。
⇒ 看倒车速度要看 **`/cmd_vel` 或 `/cmd_vel_chassis`**；`/cmd_vel_nav` 只是 controller 的输出口，**看不到 BackUp**。

### I.4 运行时验证步骤（用户执行；全程只读，不改文件）

```bash
# 0) step 1 改的是既有文件（--symlink-install 已生效）；若还没 build 过 step 3 的新文件，见 §H.1
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0 nav_rviz:=True
```
```bash
# 1) 先确认链"接上了"（3 条）
ros2 action list | grep -E "backup|spin|wait|drive_on_heading"   # 期望 /backup /spin /wait /drive_on_heading
ros2 param get /behavior_server behavior_plugins                  # 期望 ["spin","backup","drive_on_heading","wait"]
ros2 param get /bt_navigator default_nav_to_pose_bt_xml           # 期望 .../behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```
```bash
# 2) 开观察窗口（另开终端）
ros2 topic echo /behavior_tree_log     # BT 每个节点的状态迁移 → 恢复链最直接的证据
ros2 topic echo /cmd_vel               # controller(经 smoother) 与恢复动作的实际下发
ros2 topic echo /cmd_vel_chassis       # FVT 之后真正给底盘的量
```
```bash
# 3) 造"路径被堵"（要 FollowPath 失败，不是 planner 失败）：车正前方 ~0.6 m 放一个临时障碍
#    Gazebo GUI：Insert → Box，放在 world ≈ (4.9, 3.35, 0.3)（RMUL2026 出生点 world(4.3,3.35)，yaw=0）
#    或命令行：
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: tmp_obstacle, xml: '<sdf version=\"1.7\"><model name=\"tmp_obstacle\"><static>true</static><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.5 0.5 0.6</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.6</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: 4.9, y: 3.35, z: 0.3}, orientation: {w: 1.0}}, reference_frame: world}"
#    用完删掉：
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: tmp_obstacle}"
#    然后 RViz 用 2D Goal Pose 让车继续朝那个方向走（别直接发到障碍里，否则失败的是 planner）
```
拿一个**死胡同/墙角**做同样的事也可以（RViz 发一个"必须转身才能出去"的目标，车会贴进去再被 progress checker 抓住）。

> ⚠️ 别指望**第一次**恢复就看到 `Spin`：`RoundRobin` 从下标 0 开始，而 `ClearingActions`（清两张 costmap）几乎必然 SUCCESS
> ⇒ 第 1 次恢复 = 只清图并重试主树；**第 2 次**才是 `Spin`，第 3 次 `Wait`，第 4 次 `BackUp`（然后回绕）。
> 即"清图救不了 → 才转/等/退"。所以障碍要留着别撤，让它连着失败几轮。

| 顺序 | 看什么 | 期望 | 在哪看 |
|---|---|---|---|
| ① | controller 报错 | `RegulatedPurePursuitController detected collision ahead!`（RPP，`N2 nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:359`）或 **`Failed to make progress`**（progress checker 超时：`required_movement_radius 0.5 m` / `movement_time_allowance 10.0 s`，`N2 nav2_controller/src/controller_server.cpp:475`） | launch 终端 |
| ② | BT 进恢复 | `NavigateRecovery` 连续失败 → `RecoveryFallback`；随后 `RateController` 仍按 1 Hz 重规划 | `/behavior_tree_log` |
| ③ | 恢复动作依次执行 | `ClearLocalCostmap-Subtree` / `ClearGlobalCostmap-Subtree` → `Spin` → `Wait` → `BackUp`（`RoundRobin` 顺序：当前子节点 SUCCESS 即返回，FAILURE 才顺延到下一个，`N2 nav2_behavior_tree/plugins/control/round_robin_node.cpp:40-79`） | 同上 + launch 终端 |
| ④ | 动作自身日志 | `Running spin` / `spin completed successfully` / `Running backup` / `backup completed successfully`（`N2 nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp:187/239`） | launch 终端 |
| ⑤ | 倒车速度 | `/cmd_vel.linear.x ≈ -0.05`（`BackUp backup_dist 0.30 / backup_speed 0.05`；`time_allowance` 默认 10 s ⇒ 6 s 走完 0.30 m，余量够） | `/cmd_vel` 或 `/cmd_vel_chassis` |
| ⑥ | 车真的退了 | 里程计位置后退 **≥ 0.2 m**，随后 `/plan` 变化、`ComputePathToPose` 再次 SUCCESS、车继续走 | RViz + `/behavior_tree_log` |

**判据**：BT 日志里出现过 `BackUp` + `/cmd_vel.linear.x` 出现负值 + 车实际后退 ≥0.2 m + 之后重新规划成功 ⇒ 恢复链成立。
（默认 `global_obstacle:=stvl local_obstacle:=scan` 即可；`local_obstacle:=cloud` 时 `/segmentation/obstacle` 也能看到这个盒子。）

### I.5 发现的缺口（只登记，本次不修）

1. **恢复动作会被"自身已在碰撞态"一票否决**（I.3-1）⇒ 恢复链**不是**卡死的万能兜底，它只在"costmap 认为当前位置无碰撞"时有效。与 Step 1 配套；任何再次放大 `robot_radius` 的改动会重新关掉它。
2. **`bt_navigator.odom_topic: /Odometry` 是死订阅**：fastlio 的发布话题已被 remap 成 `/odom`（`bringup_sim.launch.py:271-272`）⇒ `/Odometry` 无发布者，`OdomSmoother` 恒 0。内置树里**没有**节点读 `{odom_smoother}`（两棵 XML 全文无该 port）⇒ **对恢复链无影响**，属清理项（§A.10 已记）。
3. **`BackUp 0.30 m` 与"贴墙"是紧张关系**：车尾 0.30 m 内有障碍/膨胀代价时 BackUp 会提前 `Collision Ahead` 失败（RoundRobin 会顺延，不会死锁，但这一轮恢复白费）。窄场地可考虑把 BT 的 `backup_dist` 调小 —— 那属于**改树**，本次未做。
4. **卡住的唯一检测器是 controller 的 progress checker**：内置树里没有 `IsStuck` 条件、没有 `PathLongerOnApproach`，`GoalUpdated` 只用于"目标变了就跳过恢复"。⇒ ① "慢慢蹭不动"要 **10 s** 才被发现；② `RecoveryNode` 6 次用尽后 `NavigateToPose` 直接 **abort**（不会无限重试）—— 在长目标分段（`segment_goal_navigator.py`）里表现为"这一段失败"，需注意上层是否处理。
5. **`behavior_server` 未配 `assisted_teleop`**（1.1.20 该插件存在）⇒ "人工接管把车蹭出来"这条路我们暂时没有；要加就是 `behavior_plugins` 里加一个 id + 一段参数（未做）。
6. `local_frame` 等 Jazzy 键：**不要**从参考队 yaml 抄进来（I.2），1.1.20 既不报错也不生效。
