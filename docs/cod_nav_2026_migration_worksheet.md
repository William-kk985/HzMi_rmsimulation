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
