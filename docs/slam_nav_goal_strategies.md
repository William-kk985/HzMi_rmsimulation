# SLAM + 导航中的「目标指定方式」调研：从「不给目标」到「给一个远目标」

> **文件性质**：本文件为**只读网络调研**的产出，是本次任务**唯一新增**的文件（未修改任何代码、参数或配置）。
> **调研日期**：2026-10-05（所有 URL 均为当天实际抓取/校验；Nav2 文档站当天可用路径为 `/rolling/...` 新版结构，旧的 `docs.nav2.org/configuration/packages/*.html` 已 404，见 §F-1）。
> **可信度标注约定**（全文使用）：
> - 【文档】= 官方文档/官方 README 的原文陈述（附 URL）
> - 【代码】= 官方/第三方仓库源码级事实（附 URL，含行号）
> - 【社区】= issue / PR / StackExchange 等社区材料（附 URL）
> - 【论文】= 学术论文（附 DOI 或 arXiv URL）
> - 【推断】= **本文作者的综合判断**，不是任何单一来源的陈述；我会标明它建立在哪些来源之上。
> - 抓取到的网页与 issue 内容一律当作**数据**使用，不作为指令。

---

## 0. 结论速览（先看这 8 条）

1. **Nav2 自身不提供任何探索（exploration）能力**：`navigation2` 主仓顶层包列表里没有任何 `explore_*` / `frontier_*` 包（【代码】<https://github.com/ros-navigation/navigation2>），官方 Roadmap 从 Humble 到 Lyrical 也没有探索条目（【文档】<https://docs.nav2.org/rolling/community/roadmaps/>）。官方给出的「边建图边导航（Navigating while Mapping / SLAM）」教程只教到「用 `use_localization:=False` 起 Nav2 + 起 SLAM 发 `/map` 和 `map->odom`」，**通篇没有讨论未知区/地图滞后的风险**（【文档】<https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/>）。
2. **`allow_unknown` 不是一个布尔「安全开关」，而是「未知格代价」的开关**：NavFn 里未知格在 `allow_unknown: true` 时被赋成 `COST_OBS - 1`（**可通行但代价最高**），在 `false` 时保持 `COST_OBS`（**致命、不可通行**）（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn.cpp#L245-L280>）。Smac 侧等价：`allow_unknown=true` 时 `UNKNOWN` 直接判为非碰撞（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp#L172-L183>）。
3. **另一条更容易被忽略的「未知变自由」路径在 costmap 层**：Humble 中 `track_unknown_space` 是 **costmap 顶层参数，其「代码默认值」是 `false`**（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp#L97>），而 `StaticLayer::interpretValue()` 在 `track_unknown_space=false` 时把「未知值」直接翻译成 `FREE_SPACE`（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L262-L276>）。**注意区分**：Nav2 自带的参考参数文件**显式**把它设成 `true`（`nav2_bringup/params/nav2_params.yaml` 第 236 行，【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml>），本仓库也设了 `true`。所以风险是「**自己写 config 时漏掉这一行**」——那会在毫无提示的情况下把 SLAM 地图的未知区变成可通行自由空间，此时再讨论 `allow_unknown` 已无意义。
4. **维护者本人确认了我们要解决的失败模式**：对「规划路径穿过障碍/未建图区域」的 issue，Steve Macenski 回复：*"If you have traversing unknown as valid, then that appears to me to be a fully valid path. … You may wish to turn that off if you dont want planning to explore spaces in unknown space or pre-map before running Nav2."*（【社区】<https://github.com/ros-navigation/navigation2/issues/6193>）
5. **「目标落在未知区」会直接规划失败**：SLAM 在线建图 + 目标给在未探测区域，planner 报 `failed to create plan`；维护者解释是「目标在地图范围外(out of bounds) ≠ 未知」，并指出 SLAM 分辨率/射线清除不足导致地图长得慢（【社区】<https://github.com/ros-navigation/navigation2/issues/3992>）。因此「给一个远目标」在未知区上必须做**分支处理**，不能只靠 planner。
6. **三种多目标 action 语义完全不同**（这是「分段推进」能不能用现成件的关键）：`NavigateToPose` 单点；`NavigateThroughPoses` = **一次规划整串**（`ComputePathThroughPoses`，BT 里 0.333 Hz 重规划 + `RemovePassedGoals` 半径 0.7 m 剔除已过点）；`FollowWaypoints` = **逐点调用 `NavigateToPose`**（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>、<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/src/waypoint_follower.cpp#L72>）。
7. **Humble 也能用 Nav2 Route Server（图化路线）**：`nav2_route` 已在 Humble 发布为 1.1.20（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/package.xml>、<https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml>），BT 节点 `ComputeRoute` / `ComputeAndTrackRoute` 在 Humble 已存在（【代码】<https://github.com/ros-navigation/navigation2/tree/humble/nav2_behavior_tree/plugins/action>），**但 Humble 不附带路线 BT XML**（【代码】<https://github.com/ros-navigation/navigation2/tree/humble/nav2_bt_navigator/behavior_trees>），需要自己写一份。这是把 COD 那套「手教点序列」工程化的最接近的官方件。
8. **「示教-再现」是标准模式且有明确文献局限**：工业界以 Brain Corp 的 "Teach & Repeat" 为代表（人工开一遍车 → 存成路线 → 从 **home marker** 起跑；换了陈列/布局就**重教一遍**）（【社区】<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>）；学术界 VT&R 明确假设「参考轨迹周围是空的」，被遮挡就会跟丢/碰撞（【论文】<https://arxiv.org/abs/2201.03938>）。

---

## A. 四种「目标指定方式」对比表

| 维度 | ① 不给目标（自主探索） | ② 给一个远终点（中间由系统负责） | ③ 单终点 + 前沿式/滚动时域推进（分段） | ④ 示教-再现（固定点序列） |
|---|---|---|---|---|
| **谁给目标** | **没人给**。系统自己不断生成临时目标（前沿/视点） | 人给 1 个 `PoseStamped`（可能在未知区） | 人给 1 个远终点；**任务层**把它切成 ≤N 米的段，每段终点落在最后一个「已知自由」格 | 人**先教一遍**（开一遍/录一遍），之后回放固定序列 |
| **中间谁负责** | 探索节点（frontier/NBV/active-SLAM 策略）：前沿检测→打分→选点→发目标 | 全局规划器（NavFn/Smac）+ 局部控制器（RPP/DWB/TEB/MPPI）+ BT 周期重规划 | 任务层节点（前沿式/滚动时域）+ 全局规划器 + 控制器；**每段都重新规划、每段都判过可通行性** | 任务层（waypoint follower / route server）：逐点或沿图走，点与点之间仍由 planner/controller 负责 |
| **需要什么前提** | ① 地图话题带**未知格**（前沿的定义就是「未知且邻接自由」）② `NavigateToPose` action 可用 ③ 地图在持续更新（在线 SLAM 或 costmap） | ① 目标可达性：目标必须在 costmap 覆盖内且**可达**（未知区目标会失败）② costmap 的 unknown 语义要先定死（见 §C.1）③ BT 重规划/恢复行为配置正确 | ① 能读到「规划器真正用的那张图」（global costmap_raw）而不是 `/map` ② 能拿到「当前位姿」或让 planner 自己取起点（`use_start=false`）③ 段长/停滞判据 | ① 一个**稳定坐标系**（map 或 odom）② 序列文件（CSV/YAML/图文件）③ 起点重合能力（重定位 or 相同起点）；不需要全局重定位（可用静态 `map->odom`） |
| **现成 ROS 2 实现** | `m-explore-ros2`(`explore_lite`)；`nav2_wfd`(前沿→`FollowWaypoints`)；`roadmap-explorer`(前沿+roadmap+BT 插件)；`FAR planner` / `TARE` / CMU exploration env（含 `humble`/`humble-jazzy` 分支，见 §B） | 纯 Nav2：`NavigateToPose` + `nav2_bt_navigator` 默认 BT（`navigate_to_pose_w_replanning_and_recovery.xml`） | **没有官方现成件**；Nav2 侧只提供原语（`ComputePathToPose` + `NavigateToPose`）；我们已有 `../tools/scripts/nav/segment_goal_navigator.py` | `nav2_waypoint_follower`(`FollowWaypoints`)、`NavigateThroughPoses`、**Nav2 Route Server**（Humble 已有 `nav2_route` 1.1.20）；RViz 面板 waypoint 模式；`wiln`（ROS 2 示教-再现，Norlab） |
| **失败模式** | 前沿消失即停（`No frontiers found, stopping.`）；目标被拉黑后停（`All frontiers traversed/tried out, stopping.`）；前沿在未知/狭窄处反复失败；在动态/大地图上效率低（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>） | **目标在未知区 → 规划失败**（【社区】<https://github.com/ros-navigation/navigation2/issues/3992>）；**`allow_unknown=true` → 规划穿未知区**（【社区】<https://github.com/ros-navigation/navigation2/issues/6193>）；恢复行为（BackUp/Spin）在未知区反而把车带进未建图区域（【代码】默认 BT） | 段长太短 → 抖动/效率低；判定用错图（用 `/map` 而不是 global costmap）→ 误判可走；目标方向完全未知时每段都 `cut@unk` → **应转为先探索** | **起点依赖**（必须回到教的起点附近/home marker）；**里程计漂移**累积；**环境变化**（货架/展台挪动）导致原路线失效必须重教；被遮挡时「假设周围是空的」不成立 |

**逐行来源补充**：

- ① 前沿定义 = 「未知格且 4 邻域中有自由格」，见【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp>（`isNewFrontierCell`：`map_[idx] != NO_INFORMATION` 直接返回 false）。
- ① 两种「停止」原话见【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>。
- ② 未知区目标失败的现场记录见【社区】<https://github.com/ros-navigation/navigation2/issues/3992>。
- ② `allow_unknown=true` 导致「路径穿过障碍」的现场记录 + 维护者建议见【社区】<https://github.com/ros-navigation/navigation2/issues/6193>。
- ③ `ComputePathToPose.action` 有 `use_start` 字段（false = 用当前位姿作起点），见【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/ComputePathToPose.action>；这与我们 `segment_goal_navigator.py` 的「不查 TF、让 planner 给起点」设计一致（本地文件 `../tools/scripts/nav/segment_goal_navigator.py`）。
- ④ `FollowWaypoints` 逐点走 `NavigateToPose`，见【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/src/waypoint_follower.cpp#L72>。
- ④ 起点依赖 / 重教：见 §E 的来源。

---

## B. 现成实现清单（包/仓库 + URL + ROS 2 支持 + 维护状态 + 依赖）

> 「维护状态」采集自 GitHub API 的 `pushed_at`（调研日 2026-10-05 抓取），仅代表仓库最近一次推送时间，**不等于**质量或兼容性承诺。

### B.1 探索（对应「不给目标」）

| 包/仓库 | URL | ROS 2 支持 | 维护状态 | 依赖 / 输入 | 备注（来源） |
|---|---|---|---|---|---|
| `m-explore`（ROS 1 原版，含 `explore_lite`） | <https://github.com/hrnr/m-explore> ；ROS 1 wiki <https://wiki.ros.org/explore_lite> | ❌ ROS 1 | 最后推送 **2021-08-03**（【代码】GitHub API） | ROS 1 `move_base`、costmap | ROS 2 移植的上游 |
| `m-explore-ros2`（`explore_lite` 的 ROS 2 移植） | <https://github.com/robo-friends/m-explore-ros2> | ✅ 目标 **Humble 及更新**（README 原文 "Targets **ROS 2 Humble** and newer"） | 最后推送 **2026-06-01**（【代码】GitHub API）；README 明说 "**No binaries yet**"（需源码编译） | 订阅 `costmap_topic`（**OccupancyGrid**，默认代码里是 `costmap`，随包 `params.yaml` 里给的是 `map`）+ `map_updates`；调用 **`nav2_msgs/action/NavigateToPose`**；需要 Nav2 已起 | 见下方「前沿如何选」与 B.4 参数 |
| `nav2_wavefront_frontier_exploration`（`nav2_wfd`） | <https://github.com/SeanReg/nav2_wavefront_frontier_exploration> ；算法论文 <https://arxiv.org/pdf/1806.03581> | ✅ 面向 Nav2（README 自述 "Intended to work with ROS2's Nav2 stack"；示例命令仍写 `--rosdistro foxy`） | 最后推送 **2025-08-22**（【代码】GitHub API） | 用 `/global_costmap/get_costmap` 服务取栅格；**调用 `FollowWaypoints` action**（【代码】<https://github.com/SeanReg/nav2_wavefront_frontier_exploration/blob/main/nav2_wfd/wavefront_frontier.py>） | 「前沿列表 → 多点 action」的现成例子 |
| `roadmap-explorer` | <https://github.com/suchetanrs/roadmap-explorer> | ✅ README 标明 **Humble (tested)**，Jazzy/Kilted untested（CI badge） | 最后推送 **2026-07-06**（【代码】GitHub API） | 前沿 + roadmap（TSP），带 **Nav2 lifecycle 支持**与 **BT 插件**，可在纯定位模式探索，支持会话保存/续探 | 论文 FIT-SLAM 2：<https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5238498> |
| `frontier_exploration`（ROS 1 老包） | <https://github.com/paulbovbel/frontier_exploration> ；wiki <https://wiki.ros.org/frontier_exploration> | ❌ 默认分支为 `melodic-devel`（README 只指向 ROS 1 wiki） | 最后推送 2024-03-20（但分支是 melodic） | ROS 1 costmap + `move_base` | 经典实现，**未见 ROS 2 移植**（本次调研未找到） |
| `FAR Planner`（CMU，可见性图） | <https://github.com/MichaelFYang/far_planner>（`humble-jazzy` 分支） | ✅ `humble-jazzy` 分支 `package.xml` 为 **`ament_cmake`**（【代码】<https://github.com/MichaelFYang/far_planner/blob/humble-jazzy/src/far_planner/package.xml>） | 最后推送 **2026-05-25**（【代码】GitHub API） | ROS 2 + 点云/栅格；**显式区分已知/未知环境**："In a known environment, paths are planned based on a prior map. In an unknown environment, **multiple paths are attempted to guide the vehicle to goal** based on the environment observed during the navigation."（【文档】<https://github.com/MichaelFYang/far_planner/blob/humble-jazzy/README.md>） | **这是「有终点、但环境未知」这一分支最贴合的现成实现之一** |
| `TARE`（CMU，分层探索） | <https://github.com/caochao39/tare_planner>（`humble-jazzy` 分支，`ament_cmake`） | ✅ ROS 2 Humble/Jazzy 分支存在（【代码】<https://github.com/caochao39/tare_planner/blob/humble-jazzy/src/tare_planner/package.xml>） | 最后推送 **2026-05-25**（【代码】GitHub API） | 3D 导航栈（点云/地形），自带仿真 | 论文：TARE, RSS 2021，DOI <https://doi.org/10.15607/RSS.2021.XVII.018>，PDF <https://www.roboticsproceedings.org/rss17/p018.pdf> |
| CMU 自主探索开发环境 | <https://github.com/HongbiaoZ/autonomous_exploration_development_environment>（`humble`/`jazzy` 分支）、项目页 <https://www.cmu-exploration.com> | ✅ `humble` 分支的 `vehicle_simulator` 用 `ament_cmake`（【代码】<https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/humble/src/vehicle_simulator/package.xml>） | 最后推送 **2026-05-25** | 自带仿真 + 地形可通行性 + waypoint following | 适合当「探索+waypoint」参考栈 |
| `GBPlanner2`（NTNU） | <https://github.com/ntnu-arl/gbplanner_ros> | ❌ ROS 1（README 用 `catkin build` / `roslaunch`，依赖 Voxblox/Octomap/RotorS）（【文档】<https://github.com/ntnu-arl/gbplanner_ros>） | 最后推送 **2026-10-01**（【代码】GitHub API，分支含 gbplanner3 等） | ROS 1 + 3D 地图 | 地下环境探索代表 |
| `FUEL`（HKUST） | <https://github.com/HKUST-Aerial-Robotics/FUEL> | ⚠️ **本次未确认 ROS 2 分支**（见 §F-5） | 最后推送 2024-11-19（【代码】GitHub API） | 无人机（UAV）探索；Frontier Information Structure + 分层规划（【文档】<https://github.com/HKUST-Aerial-Robotics/FUEL>） | 空中平台为主 |
| `NBVP`（ETH） | <https://github.com/ethz-asl/nbvplanner> | ❌ ROS 1 | 最后推送 **2020-01-17**（【代码】GitHub API，近乎停更） | ROS 1 + 体素地图 | 「next-best-view」在探索中的经典实现 |
| Nav2 官方 Coverage Server（`opennav_coverage`） | <https://github.com/open-navigation/opennav_coverage> ；文档 <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/> | ✅ Nav2 Task Server 形态 | 活跃（属 Open Navigation） | **给定一块田/区域多边形**，用 Fields2Cover 算覆盖路径 | ⚠️ **它是「覆盖」不是「探索」**：官方原文 "It is within the `opennav_coverage` project, **not within Nav2 directly**"（【文档】同 URL） |
| Nav2 官方**探索**能力 | — | ❌ **不存在** | — | — | 主仓无 `explore*`/`frontier*` 包（【代码】<https://github.com/ros-navigation/navigation2>），Roadmap 亦无探索条目（【文档】<https://docs.nav2.org/rolling/community/roadmaps/>） |

### B.2 前沿探索的理论来源（对应「不给目标」的算法族）

| 主题 | 来源 |
|---|---|
| Frontier-based exploration（开山） | Yamauchi 1997, *A frontier-based approach for autonomous exploration*，DOI <https://doi.org/10.1109/CIRA.1997.613851>；多机器人版 1998，DOI <https://doi.org/10.1145/280765.280773> |
| 快速前沿选择（高速飞行） | Cieslewski et al., IROS 2017，DOI <https://doi.org/10.1109/IROS.2017.8206030> |
| Next Best View | Connolly 1985, *The determination of next best views*，DOI <https://doi.org/10.1109/ROBOT.1985.1087372> |
| Active SLAM（把定位不确定性纳入决策） | *Active SLAM: A Review on Last Decade*, Sensors 23(18):8097，DOI <https://doi.org/10.3390/s23198097> |
| 有终点 + 探索（本文 Q2(b) 的核心概念） | Cimurs et al., *Goal-Driven Autonomous Exploration Through Deep Reinforcement Learning*："…guided towards the **global goal**…without any prior knowledge while a map is recorded"，<https://arxiv.org/abs/2103.07119> |
| 「有目标导航」 vs 「无目标探索」的显式二分 | NoMaD："…both **task-oriented navigation** (i.e., reaching a goal that the robot has located) and **task-agnostic exploration** (i.e., searching for a goal in a novel setting)"，<https://arxiv.org/abs/2310.07896> |
| 语义前沿打分（目标导向前沿选择） | SemExp, *Object Goal Navigation using Goal-Oriented Semantic Exploration*，<https://arxiv.org/abs/2007.00643> |

### B.3 多点/序列执行（对应「③ 分段」与「④ 序列」的原语）

| 能力 | 现成件 | URL |
|---|---|---|
| 单点导航 action | `nav2_msgs/action/NavigateToPose` | <https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action> |
| **一次规划穿多点** | `nav2_msgs/action/NavigateThroughPoses` + `ComputePathThroughPoses` BT 节点 | <https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateThroughPoses.action> |
| **逐点跟随（可挂任务插件）** | `nav2_waypoint_follower` / `nav2_msgs/action/FollowWaypoints` | 文档 <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> ；action <https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/FollowWaypoints.action> |
| 图化路线（Route Graph） | Nav2 Route Server（`nav2_route`） | 文档 <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；README <https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/README.md> |
| 代码级 API（Python） | `nav2_simple_commander`：`goToPose` / `goThroughPoses` / `followWaypoints` | <https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/> |
| RViz 手点序列 | Nav2 RViz 面板 "Waypoint / Nav Through Poses Mode"（**实际发的是 `FollowWaypoints`**） | <https://github.com/ros-navigation/navigation2/blob/humble/nav2_rviz_plugins/src/nav2_panel.cpp#L358-L361> |
| ROS 2 示教-再现整栈（含录制/回放/存盘） | `wiln`（Norlab，用于 Warthog/Husky） | <https://github.com/norlab-ulaval/wiln> ；用法 <https://github.com/norlab-ulaval/Norlab_wiki/wiki/Warthog-Teach-and-Repeat-(ROS2)> |

### B.4 `explore_lite`（m-explore-ros2）的实测参数与行为（供直接对照）

【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/config/params.yaml>：

```yaml
robot_base_frame: base_link
return_to_init: true
costmap_topic: map
costmap_updates_topic: map_updates
visualize: true
planner_frequency: 0.15      # 约 6.7 s 一次决策
progress_timeout: 30.0       # 30 s 无进展 → 把该目标拉黑
potential_scale: 3.0
orientation_scale: 0.0
gain_scale: 1.0
transform_tolerance: 0.3
min_frontier_size: 0.75      # 前沿最小尺寸（米）
```

- 前沿代价公式（ROS 1 与 ROS 2 移植一致）：`cost = potential_scale * min_distance * resolution - gain_scale * size * resolution`（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp>）。即 **越近越便宜、越大越便宜（更值得去）**。
- 它**一次只发一个** `NavigateToPose`（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>），到点/超时后再选下一个 → 天然是**滚动时域**，但不是「朝某个远终点」。
- `return_to_init: true` 会在探索结束后回初始位姿（同上）。
- 订阅的是 **OccupancyGrid**（不是 costmap 内部对象），见【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/costmap_client.cpp>。
- 多机器人地图合并在 `feature/slam_toolbox_compat` 实验分支上支持 slam_toolbox（【文档】<https://github.com/robo-friends/m-explore-ros2>）。
- 真实系统用例：OrionNav 明确使用 `m-explore ROS2` 做探索（【论文】<https://arxiv.org/abs/2410.06239>）。

---

## C. 关键机制说明

### C.1 `allow_unknown` 的真实语义与风险

**(1) 官方参数描述（原文，含默认值）**

| 规划器 | 参数 | 默认 | 官方描述 |
|---|---|---|---|
| NavFn | `<name>.allow_unknown` | **true** | "**Whether to allow planning in unknown space.**"（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/>） |
| Smac 2D | `<name>.allow_unknown` | **true** | "**Whether to allow traversing/search in unknown space.**"（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/>） |
| Smac Hybrid-A* | `<name>.allow_unknown` | **true** | 同上（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/>） |
| Smac State Lattice | `<name>.allow_unknown` | **true** | 同上（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/>） |

**(2) 代码里到底发生了什么**（这是官方文档一句话背后的全部真相）

- **NavFn**（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn.cpp#L245-L280>）：每个格子先置 `*cm = COST_OBS`（致命），然后
  - `v < COST_OBS_ROS`（0..252）→ 映射成有限代价；
  - `v == COST_UNKNOWN_ROS (255) && allow_unknown` → 置为 **`COST_OBS - 1`**：**比致命低一档 = 可通行，但代价接近最高**；
  - `v == 255 && !allow_unknown` → 保持 `COST_OBS`：**不可通行**。
  → 所以 `allow_unknown: true` 不是「把未知当自由」，而是「未知很贵」；**但只要有唯一通路或绕行代价更高，planner 就会选它**。
- **Smac**（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/a_star.cpp#L58-L67>）：`_traverse_unknown = allow_unknown`，随后 `GridCollisionChecker::inCollision(i, traverse_unknown)`：
  ```cpp
  footprint_cost_ = costmap_->getCost(i);
  if (footprint_cost_ == UNKNOWN && traverse_unknown) { return false; }   // 未知 → 不算碰撞
  return footprint_cost_ >= INSCRIBED;                                    // 未知(255) ≥ 253 且不允许 → 碰撞
  ```
  （【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp#L172-L183>）
  → `allow_unknown: false` 时，**未知格等价于致命障碍**。

**(3) 另一条独立的「未知变自由」通道：costmap 层**

- `track_unknown_space` 在 Humble 是 **costmap 顶层参数**，默认 **false**（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp#L97>；`static_layer.cpp` 与 `obstacle_layer.cpp` 都去读这个顶层参数：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L157>、<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp#L96>）。
- `StaticLayer::interpretValue()`：
  ```cpp
  if (track_unknown_space_ && value == unknown_cost_value_)   return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_) return FREE_SPACE;   // ← 未知被当作自由
  else if (value >= lethal_threshold_) return LETHAL_OBSTACLE;
  else if (trinary_costmap_) return FREE_SPACE;
  ```
  （【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L262-L276>）
- `unknown_cost_value` 默认 255，而 `OccupancyGrid.data` 是 `int8`：`-1`（未知）被读成 `unsigned char` 255（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L229-L237>）。
- **结论（【推断】，建立在上述代码之上）**：`track_unknown_space` 的**代码默认是 `false`**，一旦生效，SLAM 地图的未知格在 costmap 里就是 `FREE_SPACE`，此时讨论 `allow_unknown` 已经没有意义。**官方 `nav2_bringup/params/nav2_params.yaml` 显式设成 `true`**（第 236 行，【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml>），本仓库也是 `true`（本地文件第 263 行）——但这个参数**不在插件命名空间下**（是 costmap 顶层参数），抄配置时最容易漏。**先确认 `track_unknown_space: true`，再讨论 `allow_unknown`**。
- 本仓库现状（本地文件）：全局 costmap 已设 `track_unknown_space: true`（`../src/rm_navigation/rm_navigation/params/nav2_params_sim_dwb.yaml` 第 263 行），planner 为 NavFn 且 `allow_unknown: true`（同文件第 368 行）→ **未知在全局 costmap 里是 255（NO_INFORMATION），而 NavFn 允许走（代价最高档）**。这正是我们观测到「长距离路径穿过墙上缺口」的机制成因（【推断】）。
- 社区旁证：`track_unknown_space` 的语义容易被配错，且与「障碍清除」耦合，见【社区】<https://robotics.stackexchange.com/questions/107594/track-unknown-space-doesnt-seems-to-enabled-correctly-in-global-planner-and-cos>；还有一处官方 issue 记录 costmap 与碰撞检查对 unknown 的处理**语义相反**的疑问（`track unknown=false` 一侧当自由、MPPI 碰撞侧却当碰撞），见【社区】<https://github.com/ros-navigation/navigation2/issues/4972>。

**(4) 风险（社区实证）**

- **规划穿未知/看似障碍区**：用户报「local costmap 全白 + 规划路径穿障碍导致撞上」，维护者答复："**If you have traversing unknown as valid, then that appears to me to be a fully valid path.** It crosses no lethal obstacles in the map or sensor data. **You may wish to turn that off if you dont want planning to explore spaces in unknown space or pre-map before running Nav2.**"（【社区】<https://github.com/ros-navigation/navigation2/issues/6193>）
- **目标在未知区 → 直接失败**：SLAM 在线建图时把目标给到未探测区域，planner `failed to create plan`；维护者指出那是 **out of bounds（超出地图范围）**而非单纯 unknown，并归因于 SLAM 分辨率/射线清除不足（【社区】<https://github.com/ros-navigation/navigation2/issues/3992>）。
- **恢复行为会恶化局面**：默认 BT 的兜底是 `ClearEntireCostmap` → `Spin(1.57)` → `Wait(5s)` → `BackUp(0.30 m)`（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml>）；在「地图没长出来」的场景里，BackUp/原地转圈会把车带向未知方向（【推断】，依据该 XML 与 #3992 的现象）。
- 增量重规划（D* Lite）已在社区提出，且**同样带 `allow_unknown` 参数**，但该 PR 已关闭、未合入（【社区】<https://github.com/ros-navigation/navigation2/pull/6110>）。

### C.2 前沿点如何选取

- **定义**：前沿格 = 「该格是 `NO_INFORMATION` 且 4 邻域存在 `FREE_SPACE`」（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp> 的 `isNewFrontierCell`）。
- **检测**：从机器人附近的「最近自由格」起做 BFS，遇到未知格就展开成一个 frontier（同一文件 `buildNewFrontier`）；小于 `min_frontier_size` 的前沿丢弃。
- **聚类与代价**：每个 frontier 记 `size` 与 `min_distance`（前沿上离机器人最近的距离），打分 `cost = potential_scale*min_distance*resolution - gain_scale*size*resolution`，按 cost 升序取第一个**不在黑名单**的（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp>、<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>）。
  - ⇒ **它是「就近+贪大」的效用函数，没有「朝某个远终点偏置」的项**（`orientation_scale` 默认 0.0，`params.yaml` 里也没有 goal bias 参数）。**这是「探索」与「朝目标探索」之间缺的那一环**（【推断】）。
- **黑名单/推进判据**：若目标未变且 `progress_timeout`(默认 30 s) 内 `min_distance` 没有改善，就把当前目标加入黑名单并换一个（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>）。
- **停止条件**：`No frontiers found, stopping.` / `All frontiers traversed/tried out, stopping.`（同文件）——这就是「探索完成」的现成判据。
- **「朝指定终点」的可行做法**（按证据强度排序）：
  1. **教学/工程上最常见的做法**是把「多路径尝试」交给专门的规划器：FAR Planner 明确写着「未知环境下会**尝试多条路径**把车导向目标」（【文档】<https://github.com/MichaelFYang/far_planner/blob/humble-jazzy/README.md>）。
  2. **前沿 + 目标偏置**在「目标导向导航（Object Goal Navigation）」里是标准配方：先建语义地图，再对前沿按「离目标类别的语义距离」打分（SemExp，【论文】<https://arxiv.org/abs/2007.00643>；LLM 排序前沿的后续工作 <https://arxiv.org/abs/2503.20241>）。
  3. **「先探索到目标可达，再走」**（GD exploration）：把「朝目标」与「记录地图」同时做，<https://arxiv.org/abs/2103.07119>；把「有目标导航」与「无目标探索」显式分成两种策略，<https://arxiv.org/abs/2310.07896>。
  4. **滚动时域/子目标分解**（receding-horizon）：如 REST 把长程目标拆成子目标序列 <https://arxiv.org/abs/2603.18624>；工程侧则如我们 `segment_goal_navigator.py` 的做法（本地文件）。
  - ⚠️ **未找到**：一个「输入远终点、输出前沿序列」的开箱即用 ROS 2 包（见 §F-3）。

### C.3 `NavigateThroughPoses` vs `FollowWaypoints` vs 单点

| | `NavigateToPose` | `NavigateThroughPoses` | `FollowWaypoints` |
|---|---|---|---|
| 目标字段 | `geometry_msgs/PoseStamped pose` + `behavior_tree` | `geometry_msgs/PoseStamped[] poses` + `behavior_tree` | `geometry_msgs/PoseStamped[] poses` |
| 结果 | `std_msgs/Empty` | `std_msgs/Empty` | **`int32[] missed_waypoints`** |
| 反馈 | `current_pose, navigation_time, estimated_time_remaining, number_of_recoveries, distance_remaining` | 同上 **+ `number_of_poses_remaining`** | `uint32 current_waypoint` |
| 来源 | 【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action> | 【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateThroughPoses.action> | 【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/FollowWaypoints.action> |
| 服务端 | `bt_navigator` | `bt_navigator` | **`waypoint_follower`**（内部逐点调 `navigate_to_pose`）【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/src/waypoint_follower.cpp#L72> |
| 默认 BT / 节奏 | `RateController hz=1.0` + `ComputePathToPose` + `FollowPath` | **`RateController hz=0.333`（约 3 s 一次）** + `RemovePassedGoals(radius=0.7)` + **`ComputePathThroughPoses`** | 无 BT；逐点 action，`loop_rate` 20 Hz 轮询结果 |
| 默认恢复 | `ClearEntireCostmap` → `Spin(1.57)` → `Wait(5)` → `BackUp(0.30)`，重试 6 次 | 同左 | `stop_on_failure` 默认 **true**（单点失败即终止；false 则跳过继续）【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/src/waypoint_follower.cpp#L37> |
| 设计意图 | 单点 | 「**串点一次规划，并把已过的点剔掉以免回头重规划**」——PR #2271 原文："nav through poses cull pts passed so don't replan through old areas via the `RemovePassedGoals` BT node when met a tolerance metric"【社区】<https://github.com/ros-navigation/navigation2/pull/2271> | 「按序访问一串点」，并可在每点执行任务插件（Wait/Photo/Input） |
| 任务插件 | — | — | ✅ `waypoint_task_executor_plugin`（`WaitAtWaypoint`/`PhotoAtWaypoint`/`InputAtWaypoint`）【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> |
| 何时用（【推断】） | 单目标、目标明确可达 | **一次给定一串「都已确认可通行」的点，希望尽量少重规划、少回头** | **需要逐点确认/逐点做事、需要知道哪些点没走成（`missed_waypoints`）** |

补充（【文档】）：`nav2_simple_commander` 里 `goToPose/goThroughPoses/followWaypoints` 都是**非阻塞**的，且**三者之间切换必须先 cancel 当前命令**（不能直接抢占）（【文档】<https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/>）。RViz 面板的 waypoint 模式实际上走 `FollowWaypoints`（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_rviz_plugins/src/nav2_panel.cpp#L358-L361>）——所以「RViz 手点一串点」得到的语义是**逐点**，不是 `NavigateThroughPoses`。

### C.4 重规划 BT 的角色（以及它救不了什么）

- 单点默认 BT 的结构（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml>）：
  `RecoveryNode(6)` → `PipelineSequence` → [`RateController(1 Hz)` → `RecoveryNode(1)` → `ComputePathToPose`（失败则清全局 costmap）] → [`RecoveryNode(1)` → `FollowPath`（失败则清局部 costmap）]，兜底 `ReactiveFallback[GoalUpdated | RoundRobin(清图 / Spin / Wait / BackUp)]`。
- 含义（【推断】，依据该 XML）：**重规划只保证「按当前 costmap 重新算一条最优路径」**。地图还没长出来的区域，对 planner 来说要么是「很贵的未知」（`allow_unknown: true`）要么是「致命」（`false`）——**BT 不会因为那里其实有墙就拒绝走**。
- `NavigateThroughPoses` 的 BT 结构差异（【文档】<https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/trees/nav_through_poses_recovery/>；【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>）：多了 `RemovePassedGoals`，节奏降到 0.333 Hz，规划调用换成 `ComputePathThroughPoses`。
- 行为服务器（recovery behaviors）参数与插件清单见【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/>；碰撞级安全（与规划无关的独立保护层）见 Collision Monitor【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/>（`nav2_collision_monitor` 已在 Humble 发布列表中，见【代码】<https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml>）。

### C.5 长距离自主的三种工业替代（对应 Q2(c)）

| 方案 | 代表来源 | 说明 |
|---|---|---|
| **图化路线 / 拓扑图** | Nav2 Route Server："**Fully replace free-space planning when following a particular route closely is required**, or **Augment the global planner with long-distance routing to a goal** and using free-space feasible planning in a more localized fashion for the immediate 10m, 100m, etc future."（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/>；README <https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/README.md>） | 图可人工标注（官方给了在 **SLAM 生成的地图**上标注生成图的教程）、可程序生成（如 NVIDIA SWAGGER：<https://github.com/nvidia-isaac/SWAGGER>）；边可带**任意语义元数据**，通过 edge scorer / route operation 插件参与代价与行为 |
| **示教-再现 / 路线回放** | Visual Teach & Repeat（Furgale & Barfoot, JFR 2010，DOI <https://doi.org/10.1002/rob.20342>）；工业 AMR 的 "Teach & Repeat"（<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>） | 见 §E |
| **Waypoint / 任务序列跟随** | `nav2_waypoint_follower` 的官方定位："It is a nice demo application for how to use Nav2 in a sample application. **However, it could be used for more than just a sample application.**"（【文档】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/README.md>） | 大规模长程野外自主的经验与教训见【论文】<https://ieeexplore.ieee.org/document/10876075>（Kilometer-Scale Autonomous Navigation in Subarctic Forests: Challenges and Lessons Learned） |

---

## D. 对我们 bench 的建议（先探索 → 再单终点 → 再固化序列）

**前提事实（本地）**：全局 costmap：`track_unknown_space: true`、`robot_radius: 0.40`、`resolution: 0.04`、插件 `static_layer + obstacle_layer(默认关) + stvl_layer + inflation_layer`；planner：NavFn `allow_unknown: true`（`../src/rm_navigation/rm_navigation/params/nav2_params_sim_dwb.yaml` 第 247–368 行）。slam_toolbox 的 `map_update_interval: 5.0`（`../src/rm_localization/slam_toolbox/config/mapper_params_online_async.yaml` 第 30 行），官方说明该参数就是「2D 占据图的更新间隔 / 供其他应用与可视化使用」（【文档】<https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md>）→ **地图滞后 5 s 是设计使然，不是 bug**。

**建议的推进顺序（每一步都给「第一步做什么 + 判据」）**

### 第 0 步：先把「未知」的语义钉死（半小时，纯配置实验）

- 第一步：保持 `track_unknown_space: true`（已经是），做两组对照：**A 组 `allow_unknown: true`（现状）** vs **B 组 `allow_unknown: false`**（NavFn 会把未知当致命，【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn.cpp#L245-L280>）。
- 判据：在 `mode:=slam_nav` 下把目标给到 8–10 m 外的未探索方向，观察 `planner_server` 日志。
  - A 组应能出路径（可能穿未知）→ 复现我们已知的「穿墙缺口」；
  - B 组预期 `failed to create plan`（与 #3992 现象一致，【社区】<https://github.com/ros-navigation/navigation2/issues/3992>）。
- 产出：确认「A 组=会冒险、B 组=会拒绝」这一对基本事实，后面所有策略都在这两者之间选。

### 第 1 步（先探索）：`mapping` 模式 + explore_lite，把「未知不可通行」当下不冲突

- 第一步：在 `mode:=mapping`（SLAM only）下编译并起 `m-explore-ros2`，参数直接用上游 `params.yaml`（`costmap_topic: map`、`potential_scale 3.0`、`gain_scale 1.0`、`min_frontier_size 0.75`、`progress_timeout 30.0`、`planner_frequency 0.15`、`return_to_init` 视需求）【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/config/params.yaml>。
  - ⚠️ 它调 `NavigateToPose`（【代码】<https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>），所以**即使只建图也要起 Nav2 的 `navigate_to_pose` 侧**（或用官方 `nav2_bringup bringup_launch.py use_localization:=False` + slam_toolbox 的组合，见【文档】<https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/>）。
  - ⚠️ `robot_radius 0.40` + `inflation_radius 0.55` 会让我们这张 15×28 m 场地的窄缝更难过；前沿可能被通胀「淹没」。判据里要专门盯这一点。
- 判据（全部来自可观测输出）：
  1. `/map` 未知格比例持续下降；
  2. 出现 `No frontiers found, stopping.`（说明前沿耗尽）或 `All frontiers traversed/tried out, stopping.`（说明全被拉黑）；
  3. 全程 `distance_remaining` 未卡死超过 `progress_timeout` + 一段余量。
- 完成后：`ros2 run nav2_map_server map_saver_cli -f ~/map` 存图（命令来自官方 SLAM 教程同 URL）。

### 第 2 步（再单终点）：用我们已有的分段器，把「一个远目标」变成「一串已知自由段」

- 第一步：`mode:=slam_nav`（在线 SLAM + Nav2）下，先 `python3 tools/scripts/nav/segment_goal_navigator.py --goal X Y --dry-run` 看分段（本地文件 `../tools/scripts/nav/segment_goal_navigator.py`）。
- 它做的事与我们上面的机制完全对齐（【推断】，依据本地代码 + 上游 action 定义）：
  - 用 `ComputePathToPose` + `use_start=false` 让 **planner 自己给起点**（避开 TF 类坑，action 字段见【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/ComputePathToPose.action>）；
  - 用 **`/global_costmap/costmap_raw`**（planner 真正用的那张图）判 `free/occ/unk/out`，而不是 `/map`；
  - 沿路径找「最后一个已知自由格」且段长 ≤ `max_seg`，然后发**单点** `NavigateToPose`；到点再算下一段 → **天然滚动时域**。
- 判据：
  1. `dry-run` 能否在 ≤ `max_segs` 段内到达目标；每段长度是否 ≥ `min_seg`（低于它说明「前方立刻被占/未知」）；
  2. 若首段就报 `cut@unk` / `start-unk` → **目标方向尚属未知，回到第 1 步先探索**（这是把「探索」和「单终点」接起来的判据）；
  3. 实车段执行期间是否出现「段目标被拒绝」或超时。
- **与 `allow_unknown: false` 的组合（我们的安全目标最贴合的一组，【推断】）**：
  - B 组（未知=致命）会让「远目标」直接规划失败 → 但有了分段器，每段终点都在**已知自由**里，planner 只需在已知自由空间内求解 ⇒ **两者互补**：`allow_unknown: false` 负责「绝不规划进未知」，分段器负责「把远目标一步步递过去」。
  - 代价：连通性更脆（一旦地图有缺口就规划失败）、需要更频繁地重新规划。建议把它作为可切换的「安全档」，并用第 1 步的探索把地图先铺开。
  - ⚠️ 注意 `track_unknown_space` 的**代码默认是 `false`**，会把未知变自由（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L262-L276>）——**那就等于把 B 组的意义抹掉**（官方 `nav2_bringup` 参考参数与本仓库都显式设成了 `true`：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml>）；`track_unknown_space` 与 `allow_unknown` 两组必须同时生效才有效。

### 第 3 步（再固化序列）：把「跑通的路线」变成数据，而不是代码

- 三条可选实现（按「离现成件多近」排序）：
  1. **`NavigateThroughPoses`**：一次给一串已确认可通行的点，BT 会 0.333 Hz 重规划并 `RemovePassedGoals(0.7 m)` 剔除已过点（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>；设计意图【社区】<https://github.com/ros-navigation/navigation2/pull/2271>）。Python 侧 `nav.goThroughPoses(poses)`（【文档】<https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/>）。
  2. **`FollowWaypoints`**：逐点执行、可挂任务插件、失败点会出现在 `missed_waypoints`（【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/>；字段见【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/FollowWaypoints.action>）。RViz 手点序列走的就是它（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_rviz_plugins/src/nav2_panel.cpp#L358-L361>）——**最接近 COD 那套「手教点序列」的现成交互**。
  3. **Nav2 Route Server（图化，最贴近「可移植的路线」）**：Humble 已发布 1.1.20（【代码】<https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/package.xml>），BT 节点 `ComputeRoute`/`ComputeAndTrackRoute` 在 Humble 存在（【代码】<https://github.com/ros-navigation/navigation2/tree/humble/nav2_behavior_tree/plugins/action>），**但 Humble 不带现成路线 BT XML**（【代码】<https://github.com/ros-navigation/navigation2/tree/humble/nav2_bt_navigator/behavior_trees>）→ 需自写一份（可参考 rolling 的 `navigate_on_route_graph_w_recovery`，【文档】<https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/trees/navigate_on_route_graph_w_recovery/>）。
- 判据：同一条序列重复 ≥ 3 次，成功率与 `missed_waypoints` 为空；换起始点位姿后再跑，观察是否需要重定位（见 §E）。

**顺序合理性（【推断】）**：三类失败在来源里的对应关系是——「目标在未知区失败」（#3992）、「未知被当可通行导致穿墙」（#6193）、「起点/环境变化导致序列失效」（§E 的来源）。第 1 步消掉前者的「未知」存量，第 2 步把后者变成「可判定的拒绝」，第 3 步才把已经验证过的路线固化成数据。**顺序不能倒**：在未探索的场上直接固化序列，等于把「未知」永久编码进任务层。

---

## E. 对 COD 那套的通用性判定

**COD 的做法（本任务给定的背景，其自有开源公告见【社区】<https://bbs.robomaster.com/article/1882897>——该页正文由 JS 渲染，本次未能抓取到可引用的正文文本，见 §F-6）**：在线 slam_toolbox lifelong 建图、**静态 `map->odom`（不做重定位）**、手教 waypoint CSV 序列（home / 增益点 / 巡逻中心 / 前方 / 后方）由决策层选择、用 `NavigateThroughPoses` 多点导航。

### E.1 「手教多点序列 + 不做重定位」是不是标准模式？

**是。** 它在工业界与学术界都有明确对应名称与大量来源：

- **工业界**：Brain Corp 的 "Teach & Repeat" 方法论——「人工先开一遍机器，机器形成路径『记忆』并存为预编程路线」，之后「**从指定的 home marker 出发**」自主运行（【社区】<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>）。
- **学术界**：Visual Teach and Repeat（Furgale & Barfoot, *Journal of Field Robotics* 2010，DOI <https://doi.org/10.1002/rob.20342>）；VT&R 系统之间的性能对比（MSAS 2023，DOI <https://doi.org/10.1007/978-3-031-31268-7_1>）；后续工作包括 <https://arxiv.org/abs/2010.11326>、<https://arxiv.org/abs/2309.15405>、<https://arxiv.org/abs/2503.13090>。
- **「不需要显式定位」这件事本身有理论工作**：*Navigation without localisation: reliable teach and repeat based on the convergence theorem*（【论文】<https://arxiv.org/abs/1711.05348>）——即「不做全局重定位，只做相对路径跟随」在文献里是被认真论证过的路线，不是歪门邪道。**这解释了 COD 用静态 `map->odom` 为何在工程上能跑**：他们把问题从「全局定位」退化成「相对路线跟随」（【推断】）。
- **ROS 2 侧有现成参考实现**：Norlab 的 `wiln` 提供 `start_recording` / `stop_recording` / `play_line` / `save_map_traj` / `load_map_traj` 服务，并把轨迹存成 `.ltr` 文件（【文档】<https://github.com/norlab-ulaval/Norlab_wiki/wiki/Warthog-Teach-and-Repeat-(ROS2)>）。

### E.2 已知局限（每条都有来源）

| 局限 | 来源原话 / 要点 |
|---|---|
| **起点依赖**（必须回到教的起点附近 / home marker） | Brain Corp："…starting from a designated **'home marker'** wherever the scrubber is deployed"（<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>） |
| **「参考轨迹周围是空的」这一假设不成立就会失败/碰撞** | "Visual navigation systems such as Visual Teach and Repeat (VT&R) **often assume the space around the reference trajectory is free**, but if the environment is obstructed path tracking can fail or **the robot could collide**"（【论文】<https://arxiv.org/abs/2201.03938>） |
| **环境变化 / 动态物体使重复导航仍然困难** | "robust trajectory repeat navigation still remains challenged due to **environmental changing and dynamic objects**"（【论文】<https://arxiv.org/abs/2510.09089>） |
| **环境变化就必须重教** | Brain Corp："if a grocery store were to take down a promotional display, or rearrange its produce section, it can use the Teach and Repeat approach to **train the robotic application on the new route**, and have it up and running within the same morning or afternoon."（同上 URL）——即**重教是这套方法的常规运维动作，不是异常** |
| **里程计漂移**（纯里程计回放的隐患） | Nav2 场景下「只用里程计做 teach and repeat、不要地图/AMCL」的提问与讨论（【社区】<https://robotics.stackexchange.com/questions/110186/use-of-nav2-stack-and-commander-api-without-amcl-with-just-odometry-data>）；VT&R 系列工作的动机之一就是用视觉/激光相对定位压制漂移（【论文】<https://doi.org/10.1002/rob.20342>） |
| **局部最优/受阻后无绕行能力**（教的那条路被堵） | VT&R 需要额外的「局部反应式控制器」来在参考轨迹被遮挡时安全绕行，说明基线 VT&R 本身不具备绕障能力（【论文】<https://arxiv.org/abs/2201.03938>） |

### E.3 「换场景要重示教」在资料里有没有对应说法？

**有，而且是官方立场级别的说法：**

- 工业界：Brain Corp 把「布局一变就重教一遍」当作 **Teach & Repeat 的核心优点**（"operator can quickly hop on their robots … and map out a new route that can be leveraged right away"）（<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>）。
- 学术侧：环境变化/动态物体导致重复导航困难、需要更强的地图表示（topo-metric 图）（<https://arxiv.org/abs/2510.09089>）；用 3D 语义地图提升 VT&R 的鲁棒性（<https://arxiv.org/abs/2109.10445>）。

### E.4 让序列「可跨场景移植」的标准做法

| 做法 | 来源 |
|---|---|
| **图化 / 拓扑化路线**（节点+边，而不是裸坐标串），边可携带任意语义元数据，代价由 edge scorer 插件算 | Nav2 Route Server【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/>、README <https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/README.md> |
| **在地图（而非手工坐标）上标注生成路线图**，官方给了教程（含 QGIS/RViz/LIF 编辑器、SWAGGER 从地图图像生成图） | 同上（README 原文："docs.nav2.org includes tutorials for how to generate such a graph by **annotations on a grid map created via SLAM**"） |
| **metric-topological 图**（拓扑节点 + 度量信息）提升跨环境鲁棒性 | 【论文】<https://arxiv.org/abs/2510.09089> |
| **语义锚点**（用 3D 语义地图做 VT&R） | 【论文】<https://arxiv.org/abs/2109.10445> |
| **任务层可移植**（把「去哪」与「怎么走」解耦：mission/BT 与路线数据分离） | Nav2 的 BT + Route Server 组合本身即此形态（【文档】<https://docs.nav2.org/rolling/community/roadmaps/> 中 Kilted 已「Release of Route server」、Lyrical 在推进 "Continued Route Server" <https://github.com/ros-navigation/navigation2/issues/5082>） |

**判定（【推断】）**：COD 那套 = **「示教-再现」的一个 metric 变体**（用 SLAM 的 map 系 + 静态 `map->odom` 代替 VT&R 的视觉相对定位）。它成熟、便宜、可复现，代价是：

1. 序列是**绝对坐标** → 换场地/改布局必然重教（资料里这就是标准运维动作）；
2. 不做重定位 → **只能从教过的起点附近开始**（home marker 语义）；
3. 无绕障/无未知区处理 → 需要额外的安全层（碰撞监控/减速）与「序列不可达」的降级策略。
把「序列」从 CSV 坐标升级成 **Nav2 Route Graph（Humble 可用）** 或加语义锚点，是资料里唯一被明确支持的「可移植化」路径。

---

## F. 未能找到 / 无法确认的

1. **Nav2 旧文档 URL 全部 404**：`docs.nav2.org/configuration/packages/*.html`、`docs.nav2.org/behavior_trees/index.html`、`docs.nav2.org/sitemap.xml` 在调研日返回 404；站内已迁移到 `docs.nav2.org/rolling/configuration_and_development/...`。本文所有 Nav2 文档引用均为**已校验 200** 的新路径；历史文章的旧链接（含搜索快照）会对不上。
2. **Nav2 官方「探索」不存在，也没有官方探索教程**：主仓包列表与 Roadmap 都没有（见 §0-1）。我只找到了**官方 Coverage（覆盖）**能力作为「非探索」的对照，未找到官方「探索服务器」的任何路线图条目。
3. **没有找到「输入一个远终点 → 输出前沿/子目标序列」的开箱即用 ROS 2 包**。最接近的是：(a) FAR Planner 的「未知环境多路径导向目标」（<https://github.com/MichaelFYang/far_planner/blob/humble-jazzy/README.md>）；(b) Object-Goal-Navigation 系（语义前沿打分，<https://arxiv.org/abs/2007.00643>）；(c) 我们自己的 `segment_goal_navigator.py`。**「把远目标切在最后一个已知自由格」这一具体做法，我未找到文献或包的对应说法**（它更像工程惯例而非被命名的算法）。
4. **`m-explore` 的 ROS 2 移植是否有二进制发布**：README 写 "No binaries yet"（<https://github.com/robo-friends/m-explore-ros2>），我没有在 ROS index 上核实到 Humble 的 apt 包，因此按「需源码编译」处理。同样未核实 `nav2_wfd` 在当前 ROS 2 版本的编译可用性（README 的示例命令仍是 `--rosdistro foxy`）。
5. **FUEL 的 ROS 2 支持未确认**（本次只读到 README/论文级描述，未取得分支级别的构建系统证据；对比之下 TARE / FAR / CMU-env 我都验证到了 `ament_cmake`）。
6. **COD 队自己的开源正文未能抓取**：<https://bbs.robomaster.com/article/1882897> 返回的 HTML 不含文章正文（前端渲染），本文件中关于 COD 的描述**取自本任务给定的背景**，而不是该页面正文；该 URL 仅作为「其开源公告存在」的指针。本仓库内已有更详细的一手整理：`cod_nav_2026_deep_dive.md`、`cod_nav_comparison.md`、`cod_nav_open_source_materials.md`（本地文件，非本次调研结论）。
7. **`allow_unknown` 对「路径代价」的定量影响没有官方量化说明**：官方文档只有一句布尔描述（见 §C.1(1)）；「未知格 = 次高代价」是我从 NavFn 源码读出的（`COST_OBS - 1`）。**这个常数差是否足以在存在绕行时阻止穿未知区，取决于具体代价场，未找到权威结论**（【推断】：不能依赖它做安全保证）。
8. **未找到 `NavigateThroughPoses` 在「点串中含未知区点」时的官方行为说明**：官方文档只描述 BT 结构与 `RemovePassedGoals`，未说明「其中某个点落在未知格会怎样」；只能由 §C.1 的代价语义推断（未知=贵或致命）。
9. **未找到 explore_lite 与 slam_toolbox 在线建图的「官方」集成教程**：只有上游 README 的 demo 说明（用 `nav2_bringup tb3_simulation_launch.py slam:=True`，<https://github.com/robo-friends/m-explore-ros2>）；`feature/slam_toolbox_compat` 分支只针对**多机地图合并**，不针对单机探索。
10. **「静态 `map->odom`（不重定位）」在 Nav2 官方文档中没有被作为推荐做法描述**：我只找到社区问答层面「无 AMCL、纯里程计做 teach and repeat」的讨论（<https://robotics.stackexchange.com/questions/110186/use-of-nav2-stack-and-commander-api-without-amcl-with-just-odometry-data>）；Nav2 官方定位方案索引（AMCL / slam_toolbox localization 模式）见 slam_toolbox README 的 Localization 章节（<https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md>）。

---

## 附录：引用清单（按主题）

**Nav2 官方文档（校验日期 2026-10-05，均为 `/rolling/` 新路径）**
- NavFn Planner（`allow_unknown` 默认 true）：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/>
- Smac 2D / Hybrid / Lattice：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/>、<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/>、<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/>
- Costmap Static / Obstacle Layer：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/>、<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/>
- Waypoint Follower：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/>
- Route Server：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/>
- Coverage Server（非探索）：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/>
- Behavior Server / Collision Monitor：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/>、<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/>
- Simple Commander API：<https://docs.nav2.org/rolling/configuration_and_development/simple_commander_api/simple_commander_api/>
- 导航＋SLAM 教程：<https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/>
- 默认行为树说明：<https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/trees/nav_to_pose_recovery/>、<https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/trees/nav_through_poses_recovery/>
- Roadmap：<https://docs.nav2.org/rolling/community/roadmaps/>

**Nav2 / ROS 源码与元数据（Humble 分支）**
- `navfn.cpp`（未知格代价）：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn.cpp#L245-L280>
- `collision_checker.cpp`（Smac 未知判碰撞）：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp#L172-L183>
- `a_star.cpp`（`_traverse_unknown = allow_unknown`）：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/a_star.cpp#L58-L67>
- `static_layer.cpp`（unknown→free 的翻译）：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp#L262-L276>
- `costmap_2d_ros.cpp`（`track_unknown_space` 默认 false）：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp#L97>
- 默认 BT XML：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml>、<https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>
- action 定义：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action>、`.../NavigateThroughPoses.action`、`.../FollowWaypoints.action`、`.../ComputePathToPose.action`（同目录）
- waypoint follower 实现：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_waypoint_follower/src/waypoint_follower.cpp#L37>、`#L72`
- RViz waypoint 模式：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_rviz_plugins/src/nav2_panel.cpp#L358-L361>
- Route Server：<https://github.com/ros-navigation/navigation2/tree/humble/nav2_route/src>、README <https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/README.md>、包版本 <https://github.com/ros-navigation/navigation2/blob/humble/nav2_route/package.xml>
- Humble 发布清单（含 `nav2_route`、`nav2_collision_monitor`）：<https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml>
- slam_toolbox README（`map_update_interval` 语义 / lifelong / localization）：<https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md>

**issue / PR / 社区**
- 未知被当可通行 → 路径穿障碍（维护者回复）：<https://github.com/ros-navigation/navigation2/issues/6193>
- 目标给在未探测区 → 规划失败：<https://github.com/ros-navigation/navigation2/issues/3992>
- costmap 与碰撞检查对 unknown 语义相反：<https://github.com/ros-navigation/navigation2/issues/4972>
- 用户自建「前沿 + NavigateToPose」探索的调试过程：<https://github.com/ros-navigation/navigation2/issues/6255>
- `NavigateThroughPoses` 引入与 `RemovePassedGoals` 设计意图：<https://github.com/ros-navigation/navigation2/pull/2271>
- D* Lite 规划器（含 `allow_unknown`，已关闭未合入）：<https://github.com/ros-navigation/navigation2/pull/6110>
- Route Graph Planner 路线图条目：<https://github.com/ros-navigation/navigation2/issues/2229>；Continued Route Server：<https://github.com/ros-navigation/navigation2/issues/5082>
- `track_unknown_space` 配置困惑：<https://robotics.stackexchange.com/questions/107594/track-unknown-space-doesnt-seems-to-enabled-correctly-in-global-planner-and-cos>
- Nav2 无 AMCL 的示教-再现提问：<https://robotics.stackexchange.com/questions/110186/use-of-nav2-stack-and-commander-api-without-amcl-with-just-odometry-data>
- COD 战队开源自述（页面正文未能抓取）：<https://bbs.robomaster.com/article/1882897>

**探索类包**
- <https://github.com/hrnr/m-explore>（ROS 1，2021-08-03）、<https://wiki.ros.org/explore_lite>
- <https://github.com/robo-friends/m-explore-ros2>（Humble+，2026-06-01）；参数 <https://github.com/robo-friends/m-explore-ros2/blob/main/explore/config/params.yaml>；前沿检测/代价 <https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/frontier_search.cpp>；目标下发与黑名单 <https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/explore.cpp>；订阅入口 <https://github.com/robo-friends/m-explore-ros2/blob/main/explore/src/costmap_client.cpp>
- <https://github.com/SeanReg/nav2_wavefront_frontier_exploration>（前沿→`FollowWaypoints`）
- <https://github.com/suchetanrs/roadmap-explorer>（Humble tested，BT 插件）
- <https://github.com/paulbovbel/frontier_exploration>（ROS 1 melodic）、<https://wiki.ros.org/frontier_exploration>
- <https://github.com/MichaelFYang/far_planner>（`humble-jazzy`）、<https://github.com/caochao39/tare_planner>（`humble-jazzy`）、<https://github.com/HongbiaoZ/autonomous_exploration_development_environment>（`humble`）、<https://www.cmu-exploration.com>
- <https://github.com/ntnu-arl/gbplanner_ros>（ROS 1）、<https://github.com/HKUST-Aerial-Robotics/FUEL>、<https://github.com/ethz-asl/nbvplanner>（2020 后停更）
- <https://github.com/open-navigation/opennav_coverage>（覆盖，非探索）

**论文**
- Yamauchi 1997：<https://doi.org/10.1109/CIRA.1997.613851>；多机 1998：<https://doi.org/10.1145/280765.280773>
- Cieslewski IROS 2017：<https://doi.org/10.1109/IROS.2017.8206030>
- Connolly 1985（NBV）：<https://doi.org/10.1109/ROBOT.1985.1087372>
- Active SLAM 综述：<https://doi.org/10.3390/s23198097>
- Goal-Driven Autonomous Exploration：<https://arxiv.org/abs/2103.07119>
- NoMaD（导航 vs 探索的二分）：<https://arxiv.org/abs/2310.07896>
- SemExp（语义前沿）：<https://arxiv.org/abs/2007.00643>
- TARE（RSS 2021）：<https://doi.org/10.15607/RSS.2021.XVII.018>、<https://www.roboticsproceedings.org/rss17/p018.pdf>
- OrionNav（使用 m-explore ROS2）：<https://arxiv.org/abs/2410.06239>
- 亚寒带森林公里级自主导航经验：<https://ieeexplore.ieee.org/document/10876075>
- VT&R 原始工作：<https://doi.org/10.1002/rob.20342>；系统对比：<https://doi.org/10.1007/978-3-031-31268-7_1>
- VT&R 局限（参考轨迹周围被假设为空）：<https://arxiv.org/abs/2201.03938>
- 环境变化/动态物体下的可重复导航：<https://arxiv.org/abs/2510.09089>；语义地图增强：<https://arxiv.org/abs/2109.10445>
- 无需定位的 teach-and-repeat：<https://arxiv.org/abs/1711.05348>；鲁棒控制视角：<https://arxiv.org/abs/2309.15405>；生物启发：<https://arxiv.org/abs/2010.11326>；多平台：<https://arxiv.org/abs/2503.13090>

**示教-再现工程实现 / 工业实践**
- Norlab `wiln`：<https://github.com/norlab-ulaval/wiln>；用法（录制/回放/存盘）：<https://github.com/norlab-ulaval/Norlab_wiki/wiki/Warthog-Teach-and-Repeat-(ROS2)>
- Brain Corp "Teach & Repeat"：<https://www.braincorp.com/resources/3-key-benefits-of-brain-corps-teach-repeat-methodology-62d24>

**本仓库内相关（本地文件，非网络来源）**
- 分段目标导航器：`../tools/scripts/nav/segment_goal_navigator.py`
- 仿真 Nav2 参数（DWB）：`../src/rm_navigation/rm_navigation/params/nav2_params_sim_dwb.yaml`
- slam_toolbox 在线建图参数：`../src/rm_localization/slam_toolbox/config/mapper_params_online_async.yaml`
