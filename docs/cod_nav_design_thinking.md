# COD_NAV(2026) 思路整理 —— 重点在「定位层为什么特殊」

> 素材：`docs/cod_nav_comparison.md`、`cod_nav_2026_deep_dive.md`、`cod_nav_2026_params_review.md`、`cod_nav_macro_integration.md`、`cod_nav_nav_stack_inventory.md`、`cod_nav_open_source_materials.md`、`slam_nav_goal_strategies.md`、`nav_strategy_taxonomy.md`
> 参考代码：`third_party/cod_nav_2026/`（Gitee 成品 `master@0127e200`，只读）

## 0. 一句话总纲

COD 2026 的思路是 **「做减法」**：把导航拆成 **在线建图 + 纯 LIO 定位 + 实车调过的 MPPI + 示教航点任务层**，主动砍掉先验图重定位、地面分割、多余插件；**用"可复现的物理启动条件"替代"定位对齐"这一层**。

## 1. 逐层思路（做法 → 理由 → 代价）

| 层 | 他们的做法 | 理由（有据可查） | 代价 |
|---|---|---|---|
| 传感器 | 只留 MID-360（2026 配了 RealSense 但**没进 `observation_sources`**） | 单雷达够用、减依赖 | 近距/自身点云问题 |
| 感知预处理 | **车体裁剪盒**（`cpp_lidar_filter`，CropBox，`negative: true`）扣掉自身点云；**删掉地面分割**（patchwork++），p2l 直接吃原始点云 + 高度带 `0.10~1.00`、`range 0.5~20` | README 原话：**「STVL 无法设最小障碍半径」** ⇒ 只能从点云里抠掉车体；用高度带代替地面分割更省 | 高度带会砍低矮/近场回波；裁剪盒数值与其 marker 帧不一致、`leaf_size` 是死参 |
| 里程计 | **Small Point-LIO**（ACE 的 Point-LIO MIT 重制，官方口径快 2–3×），发 `/Odometry`、`odom→base_link` | **唯一有据可查的理由是"速度/算力"**；「高动态/自旋更强」被反证（不做去畸变，且他们从不自转） | 需 `tag`+`timestamp` 字段；`twist` 全零 |
| **定位（特殊层）** | **静态 `map→odom`**（identity + z=0.05，单点再带 yaw=−0.5）；`slam_toolbox` 的 `transform_publish_period: **0.0**`（`slam_toolbox_common.cpp` 首行直接 return ⇒ **结构上不发**）；**无 AMCL** | 作者原话：**「环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法」** + KISS「发布坐标系静态转换比用 urdf 更简单」 | **必须从固定摆位起飞**；漂移直接变图畸变；被搬动/换场地即失效 |
| 建图 | 在线 **slam_toolbox async lifelong**（`map_update_interval 1.0`）+ `auto_save_map.launch.py` 在线存图；`map_server` 加载自己存的图 | 图是**规划资产**，不是定位资产 | 图滞后/畸变没有定位层纠正 |
| 代价图 | 全局 25×25@0.04 `rolling_window: true`；局部 14×14@0.05（含 static）；**两层都 STVL**（`voxel_decay 0.5` 线性、`voxel_size 0.05`、`model_type 1`、`obstacle/raytrace 8/9`） | 体素层给**时间维**，幽灵障碍 ~0.5 s 自动消失 | 清除过激会擦薄结构 |
| 全局规划 | **SmacPlanner2D** + `cost_travel_multiplier 4.0` + `tolerance 0.5` + `allow_unknown: true`；平滑靠 **Smac 自带 `smooth_path`**（`smoother_server` 在他们链里是**死代码**） | 代价感知 ⇒ 路径居中代价谷底（治"贴缝走"） | 未知区可规划（与我们"未知=不可走"相反） |
| 局部控制 | **MPPI Omni**（实车 ±2.5 m/s、wz ±1.5、`time_steps 80`、`batch 1500`、GoalCritic 25/4.0、`temperature 0.25`、`gamma 0.008`、`CostCritic critical_cost 253`）+ **`goal_approach_controller`** 代理（`approach_distance 2.5` 限速 0.2 m/s；`direct_approach_distance 2.0` 内**绕过 MPPI 直接 P 控**并把 `angular.z=0`） | 免梯度采样控制；梯度依赖"陡"的膨胀层 —— 他们的注释：**「原 0.3 衰减太慢，整个膨胀区代价都极高，MPPI 没有梯度可用」**；"绕圈"的正解是"近目标限速 + 直接驱动" | wrapper 有**帧 bug**（map 与 odom 混帧相减）；`angular.z=0` 与自转互斥 |
| 任务层 | **手教航点 CSV（6 条序列）+ 决策按局势选路 + `NavigateThroughPoses`（多点连续穿越）**；`waypoint_editor` 在 RViz 点选 | 比赛局势有限、路径短 ⇒ 示教比在线决策便宜且可靠 | **换场地必须重教**；`wps/*.csv` 无程序自动读取 |
| 停止语义 | `PositionGoalChecker`（xy 0.2 + path_length 0.5，**不看朝向**）+ 多点 `yaw_goal_tolerance 6.28`；配合 FVT **无条件**把 `angular.z` 写成 `spin_speed`（默认 0） | 哨兵只关心"到位"，朝向交给云台/自转 | `wz` 被丢弃（MPPI 的 wz 实际无效） |
| 决策/下位机 | 按血量阈值 210/350 选目标串；下位机功率 RLS 拟合 + 热量控制 | 机动受功率/热量约束 | 不在导航层 |

## 2. 定位层为什么「特殊」—— 它把算法问题转成了纪律问题

1. **不是"弱化定位"，而是"取消定位层"**：不重定位 ⇒ 用**每次摆在同一个物理位置、同一朝向**来保证 `map` 帧与物理场地对齐。
2. **三段硬证据**：① 静态 TF 是唯一 `map→odom` 发布者（slam_toolbox 因 `transform_publish_period: 0.0` 结构上不发）；② `PositionGoalChecker` + `yaw 6.28` + FVT 覆盖 `angular.z` ⇒ 整套**只控位置**；③ 作者原话 + KISS。
3. **在文献/工业界都有对应**：teach-and-repeat **不需要显式定位**（arXiv:1711.05348）；工业界 Brain Corp 的 Teach & Repeat 同样是"从 home marker 起跑、布局一改就重教"。
4. **代价可证伪**：起点依赖 · 里程计漂移 · 换布局必须重教 · 基线**假设参考轨迹周围是空的**（被遮挡会跟丢/碰撞，arXiv:2201.03938）。
5. ⇒ **特殊点在于**：它以"**小场地 + 短时间 + 严格摆位**"为前提，用物理纪律替代在线对齐；前提一破，整套失效。

## 3. 为什么"哪都省了"还能跑（内在自洽性）

```
省掉重定位        ⇒ 少一整类失败（定位串了；我们踩过"假到达"）
省掉地面分割      ⇒ 少一个参数面
用示教序列代替决策 ⇒ 少一套任务层算法
用固定摆位代替对齐 ⇒ 少一套地图资产管理（auto_save_map 自己长图）
────────────────────────────────────────────────
全部代价压在一个前提上：必须复现物理条件
```

## 4. 对我们 D3 / D4 验证的启示

**D3（可抄）**：裁剪盒 · **STVL 双图带衰减** · **Smac2D 的 `cost_travel_multiplier 4.0`** · **MPPI 实车参数（速度降到 2.0 m/s 起，含三个硬约束改造）** · `goal_approach_controller`（**先修帧 bug + 加 spin 开关**）· `PositionGoalChecker`。

**D4（抄形态，不抄定位层）**：任务层形态 = **CSV 航点 + `NavigateThroughPoses` + 局势选路**；但**航点必须落在我们的 `map` 帧**（我们有重定位 ⇒ 车随便摆，反而**免重教**，比他们更通用）。

**明确不抄**：静态 `map→odom` / `rolling_window + allow_unknown` 的"未知可规划" / 去掉地面分割 / 直接 vendor `small_point_lio` / Jazzy-only 键 / 硬编码路径。

## 5. 一句话对照

**COD：用"可复现的物理条件"换系统的简单。**
**我们：用"定位与仿真能力"换系统的通用。**
两套都自洽 —— 所以 D3/D4 应该抄他们的**感知预处理、控制器参数、任务层形态**，而**保留我们自己的定位层**（这也是我们比他们更适合"换场地/换起点"的原因）。
