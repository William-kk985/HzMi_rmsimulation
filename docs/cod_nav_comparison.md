# COD_NAV 对比分析（RoboMaster 哨兵导航开源项目）

> 对象：https://github.com/qza36/COD_NAV —— "Sentry Navgation System for RoboMaster Session 2025"
> （辽宁科技大学 COD 战队哨兵，BSD-3-Clause，30 stars / 3 forks / 0 open issues，CI 仅编译无测试）
> 结论来源：只读分析（raw.githubusercontent + GitHub API），2026-09-26。**本文只做对比与借鉴记录，不改任何代码。**

## 0. 先弄清：仓库里有**两套不同架构**，默认分支是旧的

| 分支 | 最后提交 | 架构要点 |
|---|---|---|
| **`master`（默认，已过时）** | 2025-08-02 | FAST-LIO v2 fork（改 TF，`/Odometry` 发 `odom→chassis`）+ **patchwork++** 地面分割 + p2l(`/patchworkpp/nonground`→`/scan`，**range_min 0.48 / range_max 20.0**) + **small_gicp_relocalization**（对先验 PCD 求 `map→odom`）+ pcd2pgm 离线先验 2D 图；Nav2 = **SmacPlannerHybrid(DUBIN, allow_unknown true)** + **pb_omni_pid_pursuit_controller** 20 Hz；**无 AMCL、无轮式里程计** |
| **`rmul2026`（最新）** | 2026-03-10 | **small_point_lio** + **slam_toolbox async `mode: lifelong`**（**只支持边建图边导航**，先验图重定位被砍）+ **`cpp_lidar_filter` 裁剪盒去车体点云** → `/livox/lidar_filtered` + **MPPI Omni @50 Hz（vx/vy_max 7.5 m/s）** + **两张 costmap 都用 STVL** + Savitzky-Golay 平滑；`map→odom` 是**静态 TF**（z=0.05） |

另有 `ul_mppi`、`rmul2026_sim`、`feature-obstacle_escape_node`。`rmul2026` 分支有 `CLAUDE.md` 写架构（含自记 bug：`bt_navigator.odom_topic: "odomety"` 拼错）。

## 1. 逐项对照

| 维度 | COD_NAV（master / rmul2026） | 我们 | 判断 |
|---|---|---|---|
| 上下文 | ROS 2 Humble + Nav2，RM 哨兵，分支活跃 | ROS 2 Humble + Gazebo Classic RM 哨兵 bench | 平；**bench/实验设施我们更强** |
| LIO | FAST-LIO fork / Small Point-LIO | FAST-LIO / pointlio（槽位可选） | 平 |
| 2D 建图 | pcd2pgm 离线图 + slam_toolbox；**cartographer 配置是死的**（节点被注释） | cartographer（**改了核心 `min_probability_to_clear`**）+ slam_toolbox，在线/离线分开 | **我们强** |
| 先验图重定位 | 只有 small_gicp（硬编码 PCD 路径）；rmul2026 **完全没有** | AMCL / ICP / slam_toolbox / cartographer 四选一 | **我们强** |
| `/scan` 射程 | **20 m**（两个分支都是） | **10 m** | **他们强**（场地 15×28 m ⇒ 我们的 10 m 必然"远距缺墙"） |
| 近距/自身点 | 2026 用**裁剪盒去除车体点云**后喂 STVL（可到 0.1 m 高度） | 靠 p2l `range_min`（已试 0.05）+ local cloud 层 | **他们强**（方案更干净） |
| 全局规划 | SmacPlannerHybrid(DUBIN) / Smac2D + `cost_travel_multiplier` + `tolerance 0.5` | NavFn | **他们强** |
| 全局 costmap | res 0.04、`robot_radius 0.15`、inflation 0.7/5.0、`track_unknown_space true`、只吃 `/scan` | `robot_radius 0.40`、inflation 0.55、stvl + scan | 混合：他们更宽松；**我们的 footprint 更诚实** |
| 局部 costmap | 10×10(2025) / 14×14(2026)，**体素层/STVL（带时间衰减）** | 10×10，obstacle+cloud，**无衰减** | **他们强**（时间维） |
| 局部控制器 | pb_omni PID pursuit 20 Hz / **MPPI Omni 50 Hz、7.5 m/s** | RPP / DWB / TEB（**MPPI 已装未接**） | **他们强**（控制器与速度包络） |
| 障碍时间维 | rmul2026 两图都 STVL（`voxel_decay 0.5`） | 只有 global STVL | **他们强** |
| 未知区策略 | `allow_unknown true` + `track_unknown_space true`，且**提交的图 72×23 m 大面积 unknown** | `allow_unknown true`；场地 15×28 m | 平——**他们也没解决，甚至更糟** |
| 动态障碍 | **无跟踪、无预测、无速度估计**；只有 STVL 衰减 | 同上 | 平（都没做） |
| 任务层 | BT：`RateController 3 Hz` 重规划 + `IsStuck→BackUp 1 m@1.5 m/s` + 10 次重试；**目标由 bash 脚本轮询裁判系统 `game_type`** | stock BT + 自研分段工具 | 平（都薄；**他们没航点图/前沿探索**） |
| 仿真 | master 有 Gazebo Classic(UL24)；**rmul2026 无仿真** | 一等公民的多算法仿真 | **我们强** |
| 多算法 A/B | 靠改参数/换 launch 文件 | 槽位化（mode/lio/mapper/localization/nav/obstacle） | **我们强** |
| 工程卫生 | 硬编码 `/home/cod-sentry/...`、提交了 `.swp` 与 `cmake-build-debug/`、`map,yaml` 拼错、死配置、`clear_costmap_caller` 空指针 | 有文档、有回归工具、无硬编码 | **我们强** |

## 2. 可借鉴清单（按优先级，含代价/风险）

| # | 改什么 | 收益 | 代价/风险 |
|---|---|---|---|
| **1** | **裁剪盒去车体点云**（x ±0.3、y −0.3~0.5、z −0.1~0.2、`negative: true`、leaf 0.05）→ 喂 **stvl + local cloud 层**（`/scan` 仍走 linefit） | **直接打我们"近距看不到墙/自身遮挡"那条**：不用再纠结 `range_min`，近距点云可以放心用 | 多一个 10 Hz 节点；裁剪盒要按我们车体调（他们 y 0.5 是不对称的），太紧会删掉真障碍 |
| **2** | `p2l.range_max: 10 → 20`；local costmap `10×10 → 14×14` | 15×28 m 场地一次看全 ⇒ **消灭"远距离规划穿空白"** | costmap 更新成本 2~4×；高度带要重标（他们的带在雷达面**之上** 0.01~1.0，我们的是之下） |
| **3** | **局部 costmap 也上 STVL**（带 `voxel_decay 0.5`、`voxel_size 0.05`、`obstacle/raytrace_range 8~9 m`） | 局部幽灵障碍 ~0.5 s 内消失（我们现在没有时间维） | CPU；`raytrace` 太激进会擦掉薄结构 |
| **4** | 全局规划换 **Smac2D**（`allow_unknown true`、`tolerance 0.5`、`cost_travel_multiplier 4.0`）+ **Savitzky-Golay 平滑**（窗口 7/3 阶/refinement 2） | `cost_travel_multiplier` 是"窄缝被当走廊"的**直接对策**（把路径推向代价谷底）；`tolerance 0.5` 让目标落在膨胀格也能规划 | Smac 比 NavFn 慢（他们 `max_planning_time 4.5 s`）；SG 平滑可能切角，须验碰撞 |
| **5** | 接 **MPPI Omni** 作为第 4 个 `nav` 槽（用他们的 critics 配方：`critical_cost 253` + `consider_footprint`、`GoalCritic 15`、PathFollow/PathAlign `threshold 1.5`、`temperature 0.25`） | 免梯度的采样式控制 + 现成的战训参数；**`threshold 1.5` 是他们对"近目标切向震荡"的修正** | `batch_size 2000 × time_steps 60` 每周期 ⇒ CPU；**速度要从 2.0 m/s 起调**，别照抄 7.5 |
| **6** | BT 结构借：`RateController 3 Hz` 重规划 + `IsStuck → BackUp(1 m @1.5 m/s)` + 10 次重试；哨兵模式可考虑 **`yaw_goal_tolerance: 6.28`（只控位置）** | 恢复逻辑便宜且鲁棒；"只控位置"与我们 `spin_speed!=0` 的小陀螺模式天然匹配 | 6.28 意味着**朝向安全要单独负责**；BackUp 1.5 m/s 在拥挤场地可能触发自身避障 |

## 3. 他们**没解决**的（别去那儿找答案）

1. **未知区**：`allow_unknown true` + `track_unknown_space true`，且提交的静态图是 72×23 m / 72×42 m **大面积 unknown** ⇒ "远距离穿空白/窄缝当走廊"在他们那儿**更严重**，没有可抄的策略；
2. **动态障碍**：无跟踪、无预测、无 ESDF，只有 STVL 衰减；
3. **重定位鲁棒性**：master 只有 small_gicp（硬编码 PCD），rmul2026 干脆没有（纯 lifelong 建图）；**我们有 4 种可选**；
4. **任务层**：目标选择是 bash 轮询裁判串口；**没有航点图/拓扑/前沿探索/调度**；
5. **测试/CI**：仅编译；他们自己写"参数调优是主要开发活动""没有测试设施"；
6. **rmul2026 没有仿真**；工程卫生较差（见上表）。

## 4. 一个重要旁证：他们的 `fake_vel_transform` **有我们已经修掉的那个 bug**

两个分支的 `fake_vel_transform` 都是无条件 `aft_tf_vel.angular.z = spin_speed_`，`spin_speed` 默认 **0.0**，而**仓库内没有任何 launch 给它赋值** ⇒ **在仓库自带配置下，Nav2 的每一条转向指令都被清零**（车只能靠自转/被外部 launch 覆盖才能转）。他们另配 `enable_rotation: false` + `yaw_goal_tolerance: 6.28`，即刻意做"只控位置"的导航。

⇒ 两点：① **我们对该 bug 的定位与修复是对的**（并有独立旁证）；② "放弃朝向控制"是哨兵的一种合理设计，正对应我们 `spin_speed != 0` 的小陀螺模式。

## 5. 待确认（本次无法验证）

1. 真机 RMUC/RMUL 2025 实际跑的是哪套（`nav2_params.yaml` 是 launch 默认，但 README 与 2026 的 `CLAUDE.md` 都说 MPPI）；
2. `spin_speed` 是否被**仓库外**的 launch 设成非零（只能证明仓库内没设）；
3. `robot_base_frame` 用 fake 帧后，朝向在 `map→odom` 与小陀螺解耦之间如何维持（与我们的结构同源，值得警惕）；
4. 真机雷达外参（xacro 写 `0.0 -0.045 0.3`，而 bringup 里静态 TF 又发 z=0.5，且 `robot_state_publisher` 被注释）；
5. 2026 真机是否有深度相机（STVL 里配了 RealSense 但不在 `observation_sources`）；
6. 性能证据（无 bag、无基准、无实测数据）。

## 6. 建议的落地顺序（一次一个变量，P0 回归做闸门）

1. **裁剪盒去车体点云** → 喂 stvl/local cloud（打"近距盲区"这条，风险最低、收益最直接）；
2. **`p2l.range_max: 10 → 20`**（打"远距缺墙"，一行）；
3. **局部也上 STVL（带衰减）**；
4. 再考虑 **Smac2D + Savitzky-Golay**、**MPPI Omni**（两条都要调参，放在最后）；
5. 每步都用 `python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5` 验收，并把结论写回 `algorithm_matrix.md §四`。
