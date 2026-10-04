# COD 战队（辽宁科技大学）导航开源资料完整清单 + 优化/取舍汇总

> 调研对象：RoboMaster 哨兵 —— GitHub 用户 [`qza36`](https://github.com/qza36?tab=repositories)、Gitee 组织 [`codnavgation`](https://gitee.com/codnavgation)、[`ustl-cod`](https://gitee.com/ustl-cod)、[`cod_-control`](https://gitee.com/cod_-control)，赛季 2024 / 2025 / 2026。
> 调研方式：**只读**。GitHub REST API + `raw.githubusercontent.com`；Gitee API v5（`/orgs/<org>/repos`、`/repos/<owner>/<repo>/branches`）+ `git clone --filter=blob:none`（blobless 浅克隆，逐文件 `git show`）；BBS 文章的 `window.__NUXT__` payload 解析。
> 本次 Gitee **未遇到限流或拒绝**（API、raw、clone 均成功）；GitHub 未鉴权核心配额全程够用。
> 本文件是本次调研**唯一写入**的文件，未改动本仓库任何其它文件、未运行任何代码。
> 前置文档（本文件不重复其内容，只做增量与修正）：[`docs/cod_nav_open_source_materials.md`](cod_nav_open_source_materials.md)、[`docs/cod_nav_2026_deep_dive.md`](cod_nav_2026_deep_dive.md)、[`docs/cod_nav_comparison.md`](cod_nav_comparison.md)、[`docs/cod_nav_2026_integration_plan.md`](cod_nav_2026_integration_plan.md)。

---

## A. 完整性判定

### A0 先说结论（回答「我们拉取的库是否不够完善、缺了什么」）

1. **我们其实一个 COD 仓库都没拉。** 工作区根目录只有 `src/rm_*`（我们自己的 bench）、`tools/`、`third_party/`、`docs/`；`.tmp_research/` 里放的是 **Nav2 上游**文件（`up_nav2_sim.yaml`、`nav2_humble_obstacle_layer.cpp` 等），不是 COD 的。也就是说，此前所有关于 COD 的结论都是**从 raw 文件读出来的文档级复刻**，没有任何可直接编译/复用的代码（见工作区根目录列举与 `.tmp_research/` 内容）。
2. **即使只看 GitHub，也漏掉了一整代 2026 导航。** 2026 赛季导航上位机**不在 GitHub 的 `COD_NAV` 任何分支里**，而在 Gitee [`cod_-rm2026_-navigation`](https://gitee.com/codnavgation/cod_-rm2026_-navigation)（163 文件，仅 `master`，HEAD `0127e200` 2026-04-06「fix: 更新正确的地图文件名」）。它与 GitHub `rmul2026` 有 **10 个同名文件内容不同**，并多出 **90 个文件**（见 §B2）。
3. **最关键的缺失是 `goal_approach_controller`。** BBS 帖里「目标点附近圆周运动 → 让 claude 写了个 wrapper 控制器」这条，落地代码**只在 Gitee 2026 仓库**里（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>），GitHub `rmul2026` 里既没有这个包、`nav2_params.yaml` 里也没有它的参数。我们此前只知道"他们用 wrapper 解决"，不知道它怎么实现（透明代理 MPPI + 近目标限速 + 2 m 内直驱）。
4. **历史与场地资产也缺。** Gitee 的 [`ustl-cod/cod_nav`](https://gitee.com/ustl-cod/cod_nav) fork 保留了 GitHub 上已删除的 `dev` / `dyx-dev` 分支、`terrain_analysis` 子模块指针、以及 **2024/2025 赛季的场地 pgm 地图**（`rmuc_2024` / `rmul_2024` / `test_data/rmul_2025`）；Gitee 2026 仓库另有 2026 场地图 `maps/rmul2026.pgm`。
5. **2026 的"实车收敛值"与 GitHub `rmul2026` 的"激进值"不同。** 例：MPPI `vx_max` 从 GitHub 的 `7.5` 降到 Gitee 2026 的 `2.5`；`cost_scaling_factor` 全局从 `5.0` 提到 `15.0`（多点模式）。**移植时必须以 Gitee 2026 为准**，GitHub `rmul2026` 是季前状态。

### A1 资料级完整性表（我们已有 vs 全部导航相关）

| # | 资料 | 平台 / 链接 | 我们是否已有 | 缺失的内容 | 优先级 |
|---|---|---|---|---|---|
| 1 | `COD_NAV` GitHub（`master`/`rmul2026`/`ul_mppi`/`rmul2026_sim`/`feature-obstacle_escape_node`） | <https://github.com/qza36/COD_NAV> | ✅ 已深读（[`docs/cod_nav_2026_deep_dive.md`](cod_nav_2026_deep_dive.md)） | 代码本体未落地；`ul_mppi` / `rmul2026_sim` / `feature-obstacle_escape_node` 三个分支本次未读 | P3 |
| 2 | **Gitee 2026 导航** `cod_-rm2026_-navigation` @`master` | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | ❌ **完全没有** | `goal_approach_controller`、`cod_bringup`（`singlenav2_params.yaml` / `multiplenav2_params.yaml` / `mapper_params_online_async.yaml` / `navigation_launch.py` / `localization_launch.py` / `auto_save_map.launch.py`）、`maps/rmul2026.*`、6 组 `wps/*.csv`、仓内 `waypoint_editor` 与 `pb_nav2_plugins`、Realsense 真正接入 STVL 的参数 | **P0** |
| 3 | Gitee 2026 决策 `cod_-rm2026_-behavior-tree` @`feature/finish`（9 分支） | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature%2Ffinish> | ⚠️ 只有文档摘要 | `COD_Behavior/src/cod_behavior.cpp`、`cod_bt/*.xml`（10 个树）、`test/test_*.cpp`、`launch/*.csv`、`doc/PubNav2Goal_Patrol.md` 全文、`COD_Serial/` | P1 |
| 4 | Gitee `ustl-cod/cod_nav` fork（`master`/`dev`/`dyx-dev`/`rmul2026`） | <https://gitee.com/ustl-cod/cod_nav> | ❌ 只知道有这两个分支 | `terrain_analysis` 子模块指针、2024/2025 场地 pgm、6 套 `nav2_*_params.yaml` 变体、2025「中线架构」（`FAST_LIO_ROS2`+`loam_interface`+`sensor_scan_generation`+`small_gicp_relocalization`） | P1 |
| 5 | 2024 哨兵上位机方案 `cod_sentry` | <https://gitee.com/ustl-cod/cod_sentry> | ⚠️ 仅 README 摘要 | `rm_navigation_ws` / `rm_decision_ws` / `rm_vision_ws` 内容与脚本 `nav.sh` | P2 |
| 6 | 2026 下位机 `rmcod2026_-sentry` | <https://gitee.com/cod_-control/rmcod2026_-sentry> | ❌ | 舵轮解算、RLS 功率拟合、变速小陀螺、超电、热量控制（**决定上位机 cmd_vel 语义**）；`H7文档.pdf`、`gimbal/Document/Quaternion.pdf` 未下载 | P2 |
| 7 | 2026 自瞄 `sentry-auto-aim`（含 `rm_bringup`） | <https://gitee.com/ustl-cod/sentry-auto-aim> | ⚠️ 仅提到 | 16 字节上下位机帧协议、`rm_bringup` 启动项（**其导航包 `rm_navigation` 不在仓库内**） | P3 |
| 8 | 2025 电控 `sentry_steering_25` | <https://gitee.com/cod_-control/sentry_steering_25> | ❌ | 舵轮底盘文件结构（与 `COD_NAV master` 同赛季） | P3 |
| 9 | BBS 技术报告 1882897 | <https://bbs.robomaster.com/article/1882897> | ⚠️ 摘要 | **全文已抓取**（payload 里 `htmlContent`），本文 §C 逐条落地 | ✅ 本文件完成 |
| 10 | `COD_loopback_sim`（7★） | <https://github.com/qza36/COD_loopback_sim> | ❌ | Humble 版 `nav2_loopback_sim`、`pixi` 一键部署、配套 `nav2_params.yaml` 与地图 | **P0** |
| 11 | `waypoint_editor`（fork + Gitee 仓内副本） | <https://github.com/qza36/waypoint_editor> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/waypoint_editor> | ❌ | 两个桥接节点（`waypoint_to_nav2` / `waypoint_through_nav2`）与 CSV 格式 | **P0** |
| 12 | `box_lidar_filter` / `cpp_lidar_filter` | <https://github.com/qza36/box_lidar_filter> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cpp_lidar_filter> | ⚠️ 只有参数摘要 | 代码 + **按我们 `livox_frame` 重标定**；README 与源码的坐标系说法不一致（见 §C-15） | **P0** |
| 13 | `COD_Behavior`(2025) / `ros2_simple_serial` / `Sentry_vision` / `Sentry_ws` | <https://github.com/qza36/COD_Behavior> · <https://github.com/qza36/ros2_simple_serial> · <https://github.com/qza36/Sentry_ws> | ⚠️ 仅提到 | 2025 决策树、串口包、汇总工作空间 `.gitmodules` | P2 |
| 14 | `map_module` / `planning_module` / `litenav` | <https://github.com/qza36/map_module> · <https://github.com/qza36/planning_module> · <https://github.com/qza36/litenav> | ❌ | 作者新一代自研导航雏形（脱离 Nav2） | P3 |
| 15 | `COD_PCD2PGM` / `gridmap` | <https://github.com/qza36/COD_PCD2PGM> · <https://github.com/qza36/gridmap> | ❌ | 2.5D 高程图路线（vendor `grid_map`） | P3 |
| 16 | `enemy_tracking` / `CODSentryPAC` | <https://github.com/qza36/enemy_tracking> · <https://github.com/qza36/CODSentryPAC> | ❌ | 追击攻击点生成 / 自研 ReplanFSM 轨迹生成 | P3 |
| 17 | `rmu_gazebo_simulator`（fork，`jazzy`） | <https://github.com/qza36/rmu_gazebo_simulator> · 上游 <https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator> | ❌ | `rmul_2024/rmuc_2024/rmul_2025/rmuc_2025/rmuc_2026` 世界模型、**离线 PCD 导出** | P1（注意是 `jazzy`，与我们 Humble bench 不直接通用） |
| 18 | 研究型 fork 群（2026+ 方向） | ROG-Map: <https://github.com/qza36/ROG-Map-ROS2> · far_planner: <https://github.com/qza36/far_planner> · PCT_planner: <https://github.com/qza36/PCT_planner> · PCT_planner_ros2_cpu（自建）: <https://github.com/qza36/pct_planner_ros2_cpu> · jie_3d_nav: <https://github.com/qza36/jie_3d_nav> · KISS-Matcher: <https://github.com/qza36/KISS-Matcher> · kiss-icp: <https://github.com/qza36/kiss-icp> · small_gicp_relocalization: <https://github.com/qza36/small_gicp_relocalization> · spark-fast-lio: <https://github.com/qza36/spark-fast-lio> · DDR-opt: <https://github.com/qza36/DDR-opt> · TRG-planner: <https://github.com/qza36/TRG-planner> · Kinodynamic_esdf_mpc: <https://github.com/qza36/Kinodynamic_esdf_mpc> · MARSIM-ROS2: <https://github.com/qza36/MARSIM-ROS2> | ❌ | 全部为 fork，**本次未读内容**，仅据 BBS 展望与描述推断其方向（**不推测**为自研） | P3（储备） |
| 19 | B 站讲解视频 2 集 | <https://www.bilibili.com/video/BV1XSXZBUEYL/> · <https://www.bilibili.com/video/BV1r79cBME6Y/> | ❌ | 视频内容未转写（无字幕接口可读），只知道 Gitee README 将其标为「该导航程序详细解说教程」 | P2 |
| 20 | 场地 pgm 地图（2024/2025/2026） | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmuc_2024.yaml> · <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmul_2024.yaml> · <https://gitee.com/ustl-cod/cod_nav/blob/master/test_data/rmul_2025.yaml> · <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/maps/rmul2026.yaml> | ❌ | 分辨率/origin 见 §B4；可用于与我们仿真场地做对照 | P1 |

**一句话**：我们此前的资料是**文档级**的，且**只覆盖 GitHub 的 2025 主线 + 2026 季前**；缺的是 **Gitee 2026 成品导航（含 goal wrapper）、2024/2025 历史与场地资产、以及可直接跑的 loopback/waypoint 工具链**。

---

## B. 导航相关仓库逐项清单

> 「与 COD_NAV 的关系」指与 <https://github.com/qza36/COD_NAV>（2025，`master`/`rmul2026`）的关系。

### B1 GitHub `qza36`（共 74 仓；下表只列导航相关 27 个）

| 仓库 | 赛季/年份 | 内容 | 与 COD_NAV 的关系 | 独有内容 | 建议拉取 |
|---|---|---|---|---|---|
| [`COD_NAV`](https://github.com/qza36/COD_NAV) | 2025（`master` 2025-08-02）/ 2026 季前（`rmul2026` 2026-03-10） | 哨兵导航上位机；`master`=FAST-LIO+patchwork+++small_gicp 重定位+pcd2pgm+SmacHybrid+pb_omni PID；`rmul2026`=small_point_lio+slam_toolbox lifelong+裁剪盒+MPPI Omni+双 STVL | 本体 | `CLAUDE.md` 架构文档（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md>） | ✅ 仅作对照（已深读） |
| [`COD_loopback_sim`](https://github.com/qza36/COD_loopback_sim) 7★ | 2026-04-25 | 把 Nav2 官方 `loopback_sim` **回移到 Humble** + RoboStack/pixi 一键部署；含 `nav2_loopback_sim/params/nav2_params.yaml`、`maps/{depot,warehouse,tb3_sandbox}.pgm`、`graphs/*.geojson` | 独立工具，被 2026 决策开发依赖（BBS 明确推荐） | README 自述动机：**用 Gazebo 做早期路径规划/控制验证「不必要地麻烦」，物理引擎伪影（odom 漂移、定位误差）是干扰**；loopback 隔离控制逻辑（<https://github.com/qza36/COD_loopback_sim>） | ✅ **P0** |
| [`waypoint_editor`](https://github.com/qza36/waypoint_editor)（fork of [kzm784](https://github.com/kzm784/waypoint_editor)） | 2026-03-01 | RViz 面板打点 + CSV 导出；分支 `feature/adaptToNav2` 与 `USAGE_WITH_NAV2.zh.md` | 2026 导航与决策的航点来源 | 两个桥接节点：`waypoint_to_nav2`（`follow_waypoints` 逐点停靠）/ `waypoint_through_nav2`（`navigate_through_poses` 平滑穿越）；文档明说「waypoint_editor 和 Nav2 都会启动 `map_server` 导致冲突」 | ✅ **P0** |
| [`box_lidar_filter`](https://github.com/qza36/box_lidar_filter) | 2026-01-11 | `pcl::CropBox` 挖掉车体点云，输出 `/livox/lidar_filtered` | 2026 导航仓里改名为 `cpp_lidar_filter` | README 明确动机：「**由于 `stvl` 无法显式设置最小障碍物半径**，所以开发此包裁切源点云以避免将机器人自身注册成障碍物」（<https://github.com/qza36/box_lidar_filter>） | ✅ **P0** |
| [`pointcloud_fusion`](https://github.com/qza36/pointcloud_fusion) | 2025-11-04 | 双雷达 `message_filters` 近似时间同步 + TF2 对齐融合 | 未被 COD_NAV 引用 | 多雷达方案储备 | △ 备用 |
| [`map_module`](https://github.com/qza36/map_module) | 2026-04-08 | 最小 2D 地图模块（`map_io.hpp/cpp`），部分改写自 `nav2_map_server` | 同作者新一代自研导航的一块 | 脱 Nav2 依赖的地图 IO | △ P3 |
| [`planning_module`](https://github.com/qza36/planning_module) | 2026-04-08 | 最小 2D 规划模块（`planning.hpp/cpp`），与 `map_module` 配套 | 同上 | 最小规划实现 | △ P3 |
| [`litenav`](https://github.com/qza36/litenav) | 2026-05-01 | `app_module` / `core_types` / `litenav_ros2` / `map_module` / `plan_module`，pixi+robostack（含 `.codex`） | 作者「轻量导航」雏形 | 脱离 Nav2 的完整 app 骨架 | △ P3 |
| [`COD_PCD2PGM`](https://github.com/qza36/COD_PCD2PGM) 5★ | 2026-09-28 | pcd → 2.5D 高程栅格，vendor `grid_map`；保存方式 `map_saver_cli -t /elevation_grid` | 与 `master` 的 pcd2pgm 子模块同路线 | 独立可跑的 2.5D 高程图工具 | △ P3 |
| [`gridmap`](https://github.com/qza36/gridmap) | 2024-12-25 | `grid_map` vendor + `pcd_to_gridmap_demo` | `COD_PCD2PGM` 的前身 | README：**修改自沈阳航空航天大学 TUP 战队 2023 年哨兵导航模块**（<https://github.com/qza36/gridmap>） | △ P3 |
| [`enemy_tracking`](https://github.com/qza36/enemy_tracking) | 2026-01-23 | 订阅 `rm_interfaces/msg/Target` → 敌人周围圆形生成候选攻击点 → costmap 过滤 → 选最近可达 → 调 Nav2；含 `BehaviorTree.ROS2` | 独立任务层包 | 算法参考 `pb2025_sentry_behavior` 的 `CalculateAttackPose`（<https://github.com/qza36/enemy_tracking>） | △ P3 |
| [`CODSentryPAC`](https://github.com/qza36/CODSentryPAC) | 2026-04-08 | 自研轨迹生成：`ReplanFSM` → `planner_manager` → `TopoSearcher`/`AstarFinder`/`GlobalMap`，面向低速差速履带 | 与 Nav2 路线不同的自研架构 | `trajectory_generation/` 全套 + `bev/occ` 栅格 map | △ P3 |
| [`Sentry_ws`](https://github.com/qza36/Sentry_ws) | 2025-04-20 | 汇总工作空间；`.gitmodules` 串 `COD_Behavior`/`COD_NAV`/`Sentry_vision`/`ros2_simple_serial` | 2025 全栈入口 | 一次性拉 2025 四件套 | △ P2 |
| [`COD_Behavior`](https://github.com/qza36/COD_Behavior) | 2025-04-30 | 2025 决策（`cod_bt/t1.xml`、`src/sentry_behavior.cpp`） | 被 `Sentry_ws` 引用 | 2025 版决策 | △ P2 |
| [`ros2_simple_serial`](https://github.com/qza36/ros2_simple_serial) | 2025-12-06 | `/cmd_vel` → 串口；README 自述**已知问题：下位机连发导致串口卡死** | 2026 导航仓里为 `ros2_simple_serial`（包名 `cod_serial_ul26`） | 串口最小实现 | △ P2 |
| [`COD_NAV_NEXT`](https://github.com/qza36/COD_NAV_NEXT) | 2025-09-20 | 顶层 `cnn_driver` / `cnn_perception` / `cnn_planner`，**无 README** | 早期「下一代」尝试 | 用途无法确证（见 §E） | ❌ 暂不 |
| [`rmu_gazebo_simulator`](https://github.com/qza36/rmu_gazebo_simulator)（fork） | fork，`jazzy` 分支，2026-09-29 | `rmul_2024/rmuc_2024/rmul_2025/rmuc_2025/rmuc_2026` 世界模型、网页联机、**地图模型离线导出 PCD**、`rmoss_core`/`pb2025_robot_description` | 与 COD_NAV 无直接依赖 | 2026 场地世界模型 + PCD 导出（Jazzy，需评估 Humble 兼容） | △ P1（仅取 world/PCD 资产） |
| [`sentry_planning`](https://github.com/qza36/sentry_planning)（fork of [zzt-180/sentry_planning](https://github.com/zzt-180/sentry_planning)） | fork | 描述：「cod_nav——基于哈工大 RM2024 赛季烧饼自主导航规划代码」 | 他队代码，非 COD 自研 | 归属明确为他队 | ❌ |
| [`pct_planner_ros2_cpu`](https://github.com/qza36/pct_planner_ros2_cpu) | 自建，2026-07-07 | PCT 3D 导航（CPU 版） | 2026 之后的新方向 | 3D 导航储备 | ❌ 暂不 |
| fork 群 | — | `ROG-Map-ROS2`、`far_planner`、`PCT_planner`、`jie_3d_nav`、`KISS-Matcher`、`kiss-icp`、`small_gicp_relocalization`、`spark-fast-lio`、`DDR-opt`、`TRG-planner`、`Kinodynamic_esdf_mpc`、`MARSIM-ROS2`、`autonomous_exploration_development_environment`、`FreeDOM-ROS2`、`octomap_mapping`、`LIO-SAM-MID360-ROS2` | 全部为他人上游 | 反映其 2026+ 关注面（3D 导航 / 重定位 / 动态物体去除 / 探索） | ❌ 暂不（本次未读） |
| [`myblog`](https://github.com/qza36/myblog) / <https://qza36.github.io/myblog/> | 2026-06-24 | 个人博客（已确认存在 1 篇 CLion + micro-ROS 技术文，见 `docs/cod_nav_open_source_materials.md` §B1） | 作者视角 | 导航相关文章**本次未逐一核对** | △ P3 |

### B2 Gitee `codnavgation/cod_-rm2026_-navigation` @`master`（**本次最重要的新发现**）

- 仓库：<https://gitee.com/codnavgation/cod_-rm2026_-navigation>（15★，163 文件，仅 `master`，HEAD `0127e200`「fix: 更新正确的地图文件名」2026-04-06）。
- README：<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/README.md> — 10 个包：`cod_bringup`（launch/params/maps/wps/behavior_trees）、`cpp_lidar_filter`、`fake_vel_transform`、**`goal_approach_controller`**、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`pointcloud_to_laserscan`、`ros2_simple_serial`、`small_point_lio`、`waypoint_editor`。
- 与 GitHub `rmul2026` 的关系：**同一作者的后续版本**。文件级比对（本次 blobless 克隆后逐文件对比）：**71 个同名文件，其中 10 个内容不同、61 个完全相同**；Gitee 多 **90 个文件**（`cod_bringup` 全套、`goal_approach_controller`、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`ros2_simple_serial`、`waypoint_editor`、`README.md`），GitHub 多 **40 个**（`.idea/`、`CLAUDE.md`、`nav_bringup/scripts/`、`urdf/`、`resource/`、旧 `nav_bringup/*`）。
- 内容不同的 10 个文件（`<` = GitHub `rmul2026`，`>` = Gitee `master`）：
  - `cpp_lidar_filter/src/filter_node.cpp`：输出 QoS 从 `rclcpp::SensorDataQoS()` 回退为 `10`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cpp_lidar_filter/src/filter_node.cpp>）。
  - `small_point_lio/config/mid360.yaml`：`save_pcd: false`（GitHub 为 `true`）（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/small_point_lio/config/mid360.yaml>）。
  - `fake_vel_transform/*`（5 个文件）与 `README.md`/`.gitignore`：逐行比对**无实质差异**（疑似行尾差异导致 blob 哈希不同）。**结论：GitHub 的 `spin_speed` 无条件覆盖 `angular.z` 行为在 Gitee 2026 里同样存在**。
  - `behavior_trees/*.xml` 与 GitHub 完全一致（`diff` 为空）。
- **独有的 2026 成品部件**：
  - `goal_approach_controller`：Nav2 控制器 wrapper（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>）。
  - `cod_bringup/params/singlenav2_params.yaml`（先验图+场地图，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml>）与 `multiplenav2_params.yaml`（slam 在线建图，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/multiplenav2_params.yaml>）——**两套参数差别很大**（见 §C）。
  - `cod_bringup/wps/`：`go_gain.csv` / `go_home.csv` / `patrol_center.csv` / `patrol_front.csv` / `patrol_behind.csv` / `preload_patrol.csv`，README 给出语义顺序「去增益点 / 回启动区 / 增益点后半区巡逻 / 增益点内四角巡逻 / 增益点前半区巡逻 / 前压巡逻」（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cod_bringup/wps>）。
  - `cod_bringup/launch/auto_save_map.launch.py`：slam_toolbox 模式下按 `intervals=[30,60,...,300]` 秒用 `map_saver_cli` 存图（**路径写死 `/home/cod-sentry/dyx_ws/...`**，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/auto_save_map.launch.py>）。
  - `cod_bringup/launch/singlenav_launch.py`：`cpp_lidar_filter` + `small_point_lio` + **静态 `map→odom`（z=0.05，yaw=`-0.5`）** + `fake_vel_transform` + `cod_serial_ul26` + RealSense（`depth_module.depth_profile: 424x240x90`「最高帧率」+ spatial/temporal filter + 无序点云）+ Nav2 + RViz（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/singlenav_launch.py>）。
  - `cod_bringup/launch/multiplenav_launch.py`：裁剪盒 + small_point_lio + `pointcloud_to_laserscan` + `slam_toolbox`（`mapper_params_online_async.yaml`）+ 静态 TF + `auto_save_map.launch.py`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/multiplenav_launch.py>）。
  - `cod_bringup/launch/localization_launch.py`：`lifecycle_nodes = ['map_server']`，默认图 `maps/rmul2026.yaml`（**依然无 AMCL**，<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/launch/localization_launch.py>）。
  - `pb_nav2_plugins`：`BackUpFreeSpace`（行为）+ `IntensityVoxelLayer`（代价图层），来自北极熊（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/pb_nav2_plugins/README.md>）；**`behavior_server.backup` 已启用 `pb_nav2_behaviors/BackUpFreeSpace`，但 `intensity_voxel_layer` 在两套 params 里都未被引用（grep 0 命中）**。
  - 仓内 `waypoint_editor/USAGE_WITH_NAV2.zh.md`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/waypoint_editor/USAGE_WITH_NAV2.zh.md>）。
  - `ros2_simple_serial/src/cod_serial.cpp`：订阅 `aft_cmd_vel`，帧 = `0xA5` + `float vx/vy/vz`（偏移 1/5/9）+ `uint16 checksum`（偏移 13）= 15 字节（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/ros2_simple_serial/src/cod_serial.cpp>）。
- `mapper_params_online_async.yaml` 与 GitHub `mapper_params_async.yaml` 的差异（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/mapper_params_online_async.yaml>）：`mode: lifelong` 不变、`transform_publish_period: 0.0` 不变；`map_update_interval` 2.5→**1.0**、`minimum_time_interval` 0.5→**0.3**、`minimum_travel_heading` 0.5→**0.1**、`max_laser_range` 5.0→**10.0**、`min_laser_range` 0.01→**0.2**。

### B3 Gitee `codnavgation/cod_-rm2026_-behavior-tree`（9 分支；BBS 指向 `feature/finish`）

- 仓库：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>；分支：`dev`、`feat/AddNavigateThroughPoses`、`feat/AddPubGoalInterval`、`feat/AddWaypointFollowAction`、`feature/finish`、`feature/finish1`、`feature/pub-nav2-patrol`、`master`、`test1`。`feature/finish` HEAD `7cba6de`「latest」2026-04-02。
- 内容：`COD_Behavior/`（`src/cod_behavior.cpp`、`include/cod_behavior/{action,condition,include}.h`、`cod_bt/` 10 个 XML、`test/test_{GoZone,LowHp,OnZone,Recover}.cpp`、`launch/*.csv|*.yaml`）+ `COD_Serial/` + `rm_interfaces/` + `BehaviorTree.ROS2/`。
- 独有文档：《COD 哨兵决策说明文档（26 赛季 RMUL）》<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>（裁判系统消息 0x0001/0x0002/0x0003/0x0101/0x0104/0x0201/0x0203/0x0206/0x0207/0x0208/0x020B/0x020D 与每个功能的触发条件）；《PubNav2Goal 巡逻方案使用文档》<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/doc/PubNav2Goal_Patrol.md>（`LoadWaypoints`/`GetCurrentWaypoint`/`WaitUntilReached`/`WaitDuration`/`NextWaypoint` 端口表）。
- 与 GitHub `COD_Behavior`（2025）的关系：**完全重写**（2025 版只有 `cod_bt/t1.xml` + `sentry_behavior.cpp`）。
- 工程卫生：`feature/finish` 里提交了 `COD_Behavior/build/`、`install/`、`log/`、`.idea/`（`git ls-tree` 可见）。

### B4 Gitee `ustl-cod/cod_nav`（GitHub 的 fork，但**保留 GitHub 已删除的分支与地图**）

分支与 HEAD（`git clone` 实测）：

| 分支 | HEAD | 独有内容 |
|---|---|---|
| `master` | `4aa7095`「final rmul」2025-04-02 | 2024/2025 场地图 + 6 套参数变体：`nav_bringup/params/nav2_params.yaml`、`nav2_pure_params.yaml`、`nav2_pbOmni_params.yaml`、`nav2_slam_params.yaml`、`nav2_slop_params.yaml`、`sim_nav2_params.yaml`；`codmap.pgm/yaml`；`test_data/rmul_2025.*`（<https://gitee.com/ustl-cod/cod_nav/tree/master>） |
| `dyx-dev` | `df1dec3`「5.11」2025-05-11 | **`terrain_analysis`（gitlink，commit `c82f23c774251a564f5039b9eb3ffb4ab0a9a59c`）**；`bringup.launch.py` 里有 `get_package_share_directory('terrain_analysis')`，但 `IncludeLaunchDescription` 被**注释掉**（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/launch/bringup.launch.py>）；`nav2_params.yaml` 里 `voxel_layer` 吃 `/patchworkpp/nonground`，并**注释保留**了 `intensity_voxel_layer`（`pb_nav2_costmap_2d::IntensityVoxelLayer` + `terrain_map` 观测源）整段（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/params/nav2_params.yaml>） |
| `dev` | `492fcc4`「update README」2025-07-31 | 带 `pcd2pgm/`、`small_gicp_relocalization/`、`cmake-build-debug/`（提交进仓库） |
| `rmul2026` | `6e03b83`「chore:add stvl to global map」2025-12-19 | **2025→2026 的中间路线快照**：`FAST_LIO_ROS2/`、`nav_bringup/map/{map,blue}.pgm|yaml`、`nav_bringup/params/nav2_params_amcl.yaml`（有 AMCL 参数）、`script/pcd2pgm.sh`（<https://gitee.com/ustl-cod/cod_nav/tree/rmul2026>）。比 GitHub `rmul2026`（2026-03-10）**旧**，但保留了后者后来删掉的重定位/AMCL/pcd2pgm 路线 |

场地图 yaml（`resolution: 0.05, mode: trinary` 全部一致）：

| 地图 | origin | 赛季 | 链接 |
|---|---|---|---|
| `codmap` | `[-7.2, -8.18, 0]` | 2024 练习 | <https://gitee.com/ustl-cod/cod_nav/blob/master/codmap.yaml> |
| `nav_bringup/map/rmuc_2024` | `[-6.35, -7.6, 0]` | RMUC 2024 | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmuc_2024.yaml> |
| `nav_bringup/map/rmul_2024` | `[-3.75, -4.54, 0]` | RMUL 2024 | <https://gitee.com/ustl-cod/cod_nav/blob/master/nav_bringup/map/rmul_2024.yaml> |
| `test_data/rmul_2025` | `[-1.33, -4.51, 0]` | RMUL 2025 | <https://gitee.com/ustl-cod/cod_nav/blob/master/test_data/rmul_2025.yaml> |
| `nav_bringup/map/map`（rmul2026 分支） | `[-20.8, -9.42, 0]` | 2026（大图） | <https://gitee.com/ustl-cod/cod_nav/blob/rmul2026/nav_bringup/map/map.yaml> |
| `nav_bringup/map/blue`（rmul2026 分支） | `[-27.1, -18.4, 0]` | 2026（大图） | <https://gitee.com/ustl-cod/cod_nav/blob/rmul2026/nav_bringup/map/blue.yaml> |
| `cod_bringup/maps/rmul2026`（Gitee 2026） | `[-1.756, -7.036, 0]`，pgm 尺寸 **255×177**（≈12.75 m × 8.85 m） | RMUL 2026 场地 | <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/maps/rmul2026.yaml> |

`terrain_analysis` 的事实边界：它是 **gitlink（160000）**，但 `dyx-dev` 的 `.gitmodules` **只列了 `patchwork-plusplus`**，没有 `terrain_analysis` 条目 ⇒ 无法从该 fork 得到其源码；`pb_nav2_plugins` README 推荐的是北极熊的 [`terrain_analysis`](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis)，但**COD 用的具体来源本次未能确证（不推测）**，见 §E。

### B5 其它 Gitee 资料（与导航相关部分）

| 资料 | 链接 | 与导航的关系 | 独有内容 |
|---|---|---|---|
| `ustl-cod/cod_sentry`（2024 哨兵上位机方案，2★） | <https://gitee.com/ustl-cod/cod_sentry> | 2024 导航栈 = `rm_navigation_ws` | README 即方案文档：三模块（决策/导航/自瞄）交互极少、硬件 Livox MID-360 + i7-1270P + 海康 MV-CS016-10UC 8mm、导航**参考深北莫北极熊 `pb_rm_simulation`**；运行脚本 `nav.sh`/`decision.sh`/`vision.sh` |
| `ustl-cod/sentry-auto-aim`（2026 自瞄） | <https://gitee.com/ustl-cod/sentry-auto-aim> | 其 `rm_bringup` 是**视觉侧**启动包 | `使用方法.md` 给出上下位机 **16 字节帧**（上位→下位：`0xff` + Fire + pitch(4B) + yaw(4B) + distance(4B) + 空 + `0x0d`；下位→上位：`0xff` + mode + roll/pitch/yaw），并写明「**当前并未添加导航模块**」；`rm_bringup/launch/bringup_navigation.launch.py` 依赖外部 `rm_navigation` 包（该包不在此仓） |
| `cod_-control/rmcod2026_-sentry`（2026 下位机，11★） | <https://gitee.com/cod_-control/rmcod2026_-sentry> | **决定上位机 cmd_vel 如何变成轮速**（BBS 明确要求先对齐这一环） | 舵轮解算四轮 `atan2(Vx±Vw·sin45·R, Vy±Vw·sin45·R)` + 优劣弧 `K[i]=±1`；功率控制 RLS 拟合（`RLS_Init(&RLS_Power_Info, 4, 1, 0.99999, 1e-5)`，`W[0..3]=1.453e-07/1.23e-07/…`，舵向优先 80% 功率，隶属度分配 + 求根公式解限幅）；超电阈值 30%；变速小陀螺；热量控制 |
| `cod_-control/sentry_steering_25`（2025 舵轮哨兵电控） | <https://gitee.com/cod_-control/sentry_steering_25> | 与 `COD_NAV master` 同赛季 | 底盘/云台分仓、DM-MC-Board02(H723) + Keil/AC6 工具链 |
| 组织主页 | <https://gitee.com/codnavgation>（4 仓）· <https://gitee.com/ustl-cod>（13 仓）· <https://gitee.com/cod_-control>（18 仓） | 索引 | `codnavgation/cod_-rm2026_-{perception,simluation}` 仍为占位 |

---

## C. 优化点汇总表（最重要一节）

> 来源标注：**[BBS]** = <https://bbs.robomaster.com/article/1882897>（作者原文，本文件已从 `window.__NUXT__` 的 `htmlContent` 完整提取）；**[G26]** = Gitee 2026 导航仓 `master`；**[GH26]** = GitHub `COD_NAV` `rmul2026`；**[CTL]** = <https://gitee.com/cod_-control/rmcod2026_-sentry>。
> 「对我们适用性」以我们 bench 的现状为基线（`robot_radius 0.40`、场地 15×28 m、NavFn + cartographer + 4 种重定位、`/scan` 10 m、GST 仿真；见 [`docs/cod_nav_2026_integration_plan.md`](cod_nav_2026_integration_plan.md)）。

| # | 问题（他们的语境） | 做法 | 参数 / 文件 | 理由（原文/注释） | 对我们适用性 |
|---|---|---|---|---|---|
| C1 | **VoxelLayer 清除不干净**：高速移动时频繁避障、「遥遥晃晃」 | 改用 **STVL**（不靠射线清理，靠时间衰减），两张 costmap 都挂 | `voxel_decay 0.5`、`decay_model 0`(linear)、`voxel_size 0.05`、`observation_persistence 0.0`、`combination_method 1`、`update_footprint_enabled true`、源级 `obstacle_range 8.0/raytrace_range 9.0`、`min_obstacle_height 0.15`、`max_obstacle_height 1.0`、**`model_type 1`**、`vertical_fov_angle 2.00`、`filter: voxel`；**[G26]** <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml> | **[BBS]**「nav2_costmap_2d::VoxelLayer 会导致成本地图的清除不够干净，总有噪点残留，会让高速移动下的哨兵频繁避障，遥遥晃晃」；STVL 官方描述被其原样引用：「he Temporal in this package is the novel concept of voxel_decay whereas we have configurable functions that expire voxels」 | **直接可用**（包已装）。注意 layer 级 `obstacle_range 3.0` 被源级 `8.0` 覆盖（同名参数陷坑） |
| C2 | 地形/坡道感知怎么做 | **把地形处理放到 costmap 之外**（解耦），外部算好再喂；上赛季坡道用 `patchwork-plusplus`，「效果还凑合，调一调也能过洞」；`pb_nav2_plugins.IntensityVoxelLayer` 就是为此（按强度标记 + `terrain_map`） | **[BBS]**；`IntensityVoxelLayer` 参数 `min/max_obstacle_intensity`（域在 `obstacle_layer` 下而非 source 下），推荐配 `terrain_analysis`；[G26] `pb_nav2_plugins/README.md` <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/pb_nav2_plugins/README.md>；COD 自己的候选接线在 [fork `dyx-dev`] <https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/params/nav2_params.yaml>（整段被注释） | 解耦「方便算法测试以及更换」 | **需改造**：我们走 linefit → `/segmentation/obstacle`。可借其「外部处理 + 强度层」的思路，但 `intensity_voxel_layer` 在 2026 实跑里**未被启用**（grep 0），不要误判为"他们在用" |
| C3 | **单点导航容易撞墙**，脱困极浪费时间，「在 RM 分秒必争的比赛里是致命的」 | 抛弃单点 `NavigateToPose`，改 **`Navigate Through Poses`**：规划一条连续路径穿越所有航点、中间不停留 | BT：`nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`（`RemovePassedGoals radius=0.7` → `ComputePathThroughPoses`）；**[GH26]** <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml>（与 [G26] 同名文件 `diff` 为空） | **[BBS]**「机器人规划一条连续路径穿越所有航点，中间不停留，这样就可以让机器人非常丝滑地在无先验地图的情况下快速按照既定路线移动」 | **直接可用**（我们 stock BT 未用 through-poses；`nav2_bt_navigator` 已装） |
| C4 | 航点怎么来、怎么被决策消费 | RViz 打点 → 存 CSV → 决策解析 CSV；巡逻/前压也用 CSV | **[BBS]**；`waypoint_editor`（[G26] 仓内）<https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/waypoint_editor>；6 组语义 CSV <https://gitee.com/codnavgation/cod_-rm2026_-navigation/tree/master/src/cod_bringup/wps>；CSV 列 `id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command` | **[BBS]**「此地图由仿真环境构建，路径点由 waypoint_editor 创建」 | **直接可用**。附带的两个坑要注意：`waypoint_editor` 与 Nav2 **都会启 `map_server` 冲突**；CSV 不能直接被 Nav2 用，要过 `waypoint_to_nav2` / `waypoint_through_nav2` 桥（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/waypoint_editor/USAGE_WITH_NAV2.zh.md>） |
| C5 | MPPI「调试过程及其痛苦」 | **不要一味调 `std`**，关注 `temperature` 与 `gamma` | `temperature: 0.25`、`gamma: 0.015`（[G26] 单点）/ `0.008`（[GH26]）；`vx_std/vy_std 0.3`、`wz_std 0.2`（[G26] 单点）/ `0.35/0.35`（[G26] 多点） | **[BBS]**「个人认为不能一味调试 std，而是去关注 temperature 和 gamma，可以有效的避免晃动」 | **直接可用**（我们 P2 计划里就有 MPPI 槽） |
| C6 | `PathFollow` / `PathAlign` 权重与阈值 | **[BBS] 说**权重不宜 >10、`threshold_to_consider` 不宜 >1.0；**代码里并不一致** | [G26] 单点：`PathFollowCritic cost_weight 15.0 / threshold 1.0`、`PathAlignCritic 15.0 / 0.8`；[G26] 多点：`PathFollow 8.0 / 3.0`、`PathAlign 15.0 / 3.0`；[GH26]：`5.0/1.5`、`10.0/1.5` | [BBS] 是口头经验；代码注释另有一套（「放大关闭距离，让 GoalCritic 在近目标区域主导，防止画弧」） | **需改造**：文档与代码矛盾，**不要照抄**；我们按自己的目标点震荡现象做 A/B（每次一个变量） |
| C7 | 目标点附近**切向绕圈** | **拉高 `GoalCritic` 权重与 `threshold_to_consider`**（作者自称「邪修调参」） | [G26] 单点 `GoalCritic cost_weight 25.0 / threshold 4.0`；多点 `20.0 / 3.0`；[GH26] `15.0 / 2.5` | **[BBS]**「而我实际的做法是拉高 GoalCritic 的权重和 threshold_to_consider，邪修调参」 | **直接可用**（低成本，先试） |
| C8 | 圆周运动「怎么也调不好」 | **不调 MPPI，做一层 wrapper 控制器**：`GoalApproachController` 透明代理内部控制器（默认 MPPI），近目标限速 + 更近时绕开 MPPI 直接 P 控朝目标走 | `inner_plugin: nav2_mppi_controller::MPPIController`；`approach_distance 2.5`、`approach_velocity 0.2`、`direct_approach_distance 2.0`、`direct_approach_kp 3.0`；直驱分支 `cmd.twist.angular.z = 0`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/src/goal_approach_controller.cpp>）；插件声明 <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/goal_approach_controller/goal_approach_controller_plugin.xml> | **[BBS]**「后来干脆就不调了，让 claude 写了个 wrapper 控制器，实际的控制器还是 mppi…在靠近目标点附近时，强制固定速度，且只用 kp 引导至目标点，这样做非常完美的解决了上述的问题」 | **直接可用（最高价值）**，但两条硬约束：① 它会把 `angular.z` 置 0，**与"控制器控朝向"互斥**；② 需自己 `colcon build` 这个包（仅依赖 `nav2_core`/`pluginlib`） |
| C9 | 到点判定：10 cm 太严，高速 omni 会绕圈 | 单点用 **`PositionGoalChecker`**（只查 xy + 路径长度，不看朝向）；多点用放宽的 `SimpleGoalChecker` | [G26] 单点 `plugin: nav2_controller::PositionGoalChecker, xy_goal_tolerance 0.2, path_length_tolerance 0.5, stateful False`；多点 `SimpleGoalChecker xy 0.30 / yaw 6.28`；注释「放宽到 25cm，高速 omni 机器人 10cm 太严导致绕圈」 | **[BBS]**「goal checker 采用的是 nav2_controller::PositionGoalChecker，只需设置 xy 的阈值而不考虑角度朝向」 | **直接可用**（注意我们现有 `goal_checker` 键名是 `general_goal_checker`，需要同名替换） |
| C10 | MPPI 在膨胀区没有梯度 | 调 **inflation 梯度**：加大 `cost_scaling_factor` 让代价快速衰减；`CostCritic.critical_cost` 只对 inscribed/lethal 生效 | [G26] 单点 局部 `cost_scaling_factor 5.0 / inflation_radius 0.55`、全局 `5.0 / 0.75`；[G26] 多点 全局 `cost_scaling_factor **15.0**`；`CostCritic cost_weight 3.8, critical_cost 253.0, consider_footprint true, collision_cost 1e6, trajectory_point_step 2` | 注释原文：「关键修正！原 0.3 衰减太慢，整个膨胀区代价都极高，**MPPI 没有梯度可用**。5.0 让代价快速衰减，形成清晰的『远离障碍物』梯度」；「只有 inscribed/lethal 才触发碰撞惩罚」 | **需改造**：我们 `robot_radius 0.40`（他们单点 footprint 只有 0.2 m 方框、多点 0.3 m），膨胀半径必须按比例重算；建议先只改 `cost_scaling_factor` |
| C11 | 全局规划贴缝走 / 目标落在膨胀格无法规划 | 换 **`SmacPlanner2D`** + `cost_travel_multiplier` + `tolerance` + 内置 `smooth_path` | [G26] `plugin nav2_smac_planner/SmacPlanner2D, tolerance 0.5, allow_unknown true, max_planning_time 4.5, max_iterations 1e6, max_on_approach_iterations 1000, cost_travel_multiplier 4.0, smooth_path True`；`smoother: {max_iterations 10000, w_smooth 0.4, w_data 0.1, tolerance 1e-10, do_refinement true}`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml>） | `cost_travel_multiplier` 注释：「较大的值将更精确地放置在通道的中心（如果存在非 FREE 代价势场）」 | **直接可用**；**不要抄** `motion_model_for_search: "DUBIN"`、`angle_quantization_bins`、`minimum_turning_radius` 等 Hybrid 专用键（2D 插件下无效，是残留） |
| C12 | 路径平滑 | **真正生效的平滑是 Smac 的 `smooth_path: true`**；`smoother_server` 虽启动但 BT 里没有 `<SmoothPath>` ⇒ **死代码** | [G26]/[GH26] `singlenav2_params.yaml` / `nav2_params.yaml`：`smoother_plugins: ["savitzky_golay_smoother"]`, `window_size 7, poly_order 3, refinement_num 2, enforce_path_inversion true`；BT XML 无 `SmoothPath`（两版 XML 内容一致） | 事实（配置与 BT 对照） | **认知项**：不要把 savgol 当成他们的收益来源；我们要么在自研 BT 里显式加 `<SmoothPath>`，要么只用 Smac `smooth_path` |
| C13 | 代价图几何与算力折中（单点/多点两套） | 单点（有场地图）：局部 10×10@0.05、全局 25×25@0.04 滚动窗、`always_send_full_costmap False`；多点（slam）：局部 10×10、全局 **50×50**、`always_send_full_costmap True` | [G26] <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/singlenav2_params.yaml> / <https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cod_bringup/params/multiplenav2_params.yaml>；`update_frequency/publish_frequency 20.0` | 事实：多点模式没有先验图，需要更大滚动窗 | **需改造**：我们场地 15×28 m，**不要抄 50×50@0.04 + 全量发布**（每帧 ≈1250×1250） |
| C14 | 速度包线（"冲刺"理念 vs 实车收敛） | GitHub 季前：`vx/vy ±7.5 m/s`、`ax/ay ±5.0`、`controller_frequency 50`、`wz_max 2.5`、`prune_distance 1.7`、`reset_period 1.0`；**Gitee 2026 实跑降到**：`vx/vy ±2.5`、`wz ±1.5`、`ax/ay_max 1.5`、`ax_min -3.5`、`ay_min -3.5`、`az_max 1.0`（单点）/`3.0`（多点）、`reset_period 0.08`、`batch_size 1500`、`time_steps 80` | [GH26] <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml>；[G26] 同参数文件 | **[BBS]** 开头「我的设计理念就一个：冲刺！」；但 Gitee 2026 的实际数值明显收敛（2.5 m/s） | **直接可用**：`±2.5 m/s` 正好落在我们「2.0 m/s 起标」的计划区间，可直接作为首版 A/B 值 |
| C15 | 车体自遮挡：**STVL 无法设最小障碍半径** | `pcl::CropBox` 负向裁剪车体 → `/livox/lidar_filtered` 只喂 3D 代价图层 | [G26] `cpp_lidar_filter/src/filter_node.cpp`（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/cpp_lidar_filter/src/filter_node.cpp>）；两套 launch 内联值均为 `x ±0.2 / y -0.2~0.4 / z -0.1~0.2 / negative True / leaf_size 0.05`；`box_lidar_filter` README：坐标「符合 REP 103…原点在 `base_link`」且注明「**降采样未启用**」 | **[BBS]/README**：「由于 `stvl` 无法显式设置最小障碍物半径，所以开发此包裁切源点云以避免将机器人自身注册成障碍物」（<https://github.com/qza36/box_lidar_filter>） | **直接可用但必须重标定**：源码对 `CropBox` **不做 TF**（作用在输入点云自身坐标系），而 marker 发在 `base_link` ⇒ README 的「base_link」与实现不符，照抄数值会错位。`leaf_size` 是死参数（`VoxelGrid` 整段注释） |
| C16 | 深度相机要不要接 | **真接**：2026 单点模式把 RealSense 加进 STVL 观测源，并按分辨率限定有效距离 | `observation_sources: livox_source realsense_source`；RealSense `obstacle_range 3.0 / raytrace_range 3.5 / min_z 0.15 / max_z 3.5 / min_obstacle_height 0.18 / max_obstacle_height 1.5 / model_type 0`；驱动 `depth_module.depth_profile: 424x240x90`「最高帧率」+ spatial/temporal filter + `pointcloud.ordered_pc: false` | 注释：「424x240 低分辨率有效探测距离」「低分辨率超过 3.5 m 精度差」「哨兵高度以上不关心」 | **不适用**（我们无深度相机）；但"**按传感器分辨率设定有效探测距离**"这条思路可用 |
| C17 | 上位机速度调不动？ | **先确认下位机能正确把控制器速度转成轮速**，再谈上位机提速 | **[BBS]** 图注：「蓝色是 lio 算出的车体实际速度，绿色是 cmd_vel 的期望速度，可见，这是明显的没有正确转换」；链路 `cmd_vel → velocity_smoother(30 Hz, OPEN_LOOP, deadband 0.05) → fake_vel_transform → /aft_cmd_vel → cod_serial_ul26`；帧 `0xA5`+3×float+checksum=15 B | **[BBS]**「NOTE：一定要注意的是，不能一味去调上位机的速度，先要确保电控那边能正确把控制器的速度转换成轮速」 | **直接可用（契约级）**：对应我们的 [`docs/sim_real_contract.md`](sim_real_contract.md)；建议在 bench 里加一条「cmd_vel → 轮速」的一致性检查 |
| C18 | 姿态控制归属（哨兵自转） | **把朝向从 Nav2 里摘出去**：伪底盘 `base_link_fake` + `fake_vel_transform` 用 `spin_speed` 无条件覆盖 `angular.z`；容差 `yaw_goal_tolerance 6.28` ⇒ 只控位置 | [G26]/[GH26] `fake_vel_transform/src/fake_vel_transform.cpp`（`aft_tf_vel.angular.z = spin_speed_;`，默认 `0.0`，**仓库内无任何文件设非零**）；`base_link_fake` 出现在两个 costmap / bt_navigator / smoother / behavior server | README：「`base_link_fake` 的 yaw 固定指向正前方，避免云台自旋时 Nav2 局部规划器被朝向变化带偏」；**[BBS]** 另说 `fake yaw 也是之前抄的北极熊 24 年开源` | **部分可用**：我们已修成「`spin_speed==0` 直通」，**语义与他们不同**（他们恒为 0 ⇒ MPPI 的 `wz` 被丢弃）。若要抄"只控位置"，必须同批放宽 `yaw_goal_tolerance` |
| C19 | 建图与定位：要不要重定位 | **不要**：静态 `map→odom`（`z=0.05`）+ 纯 LIO；`slam_toolbox` 只做 lifelong 建图且 `transform_publish_period 0.0`（不发布 `map→odom`） | [G26] `singlenav_launch.py` 静态 TF **yaw=`-0.5`**（非 0）；`localization_launch.py` `lifecycle_nodes=['map_server']`；`mapper_params_online_async.yaml` `mode: lifelong`、`transform_publish_period 0.0` | **[BBS]**「只要提供了 map->odom->baselink 的 tf…nav2 就可以正常运行」「**由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法**」；[G26] README：「**Keep It Simple Stupid**…发布坐标系静态转换比运用 urdf 维护更简单易操作」 | **不适用（作为默认）**：静态桥要求每次从同一位姿起飞、先验图同原点采集；我们 4 种重定位是资产（[`docs/cod_nav_2026_deep_dive.md`](cod_nav_2026_deep_dive.md) §D1）。**但可作纯在线建图模式的临时开关** |
| C20 | 在线建图参数（slam_toolbox） | 提高更新/键帧密度，把 laser range 提到 10 m | [G26] `map_update_interval 1.0`、`minimum_time_interval 0.3`、`minimum_travel_heading 0.1`、`max_laser_range 10.0`、`min_laser_range 0.2`（[GH26] 分别为 2.5 / 0.5 / 0.5 / 5.0 / 0.01） | 事实（两版参数差异） | **需改造**：我们 `mapper:=slam_toolbox` 已有；这些值可作 `slam_toolbox_lifelong` 新槽起点 |
| C21 | 比赛阶段切换怎么做 | 决策 BT（XML 加载）分两类：**多点决策**（适应性训练建图用）与**单点决策**（基于场地图，更快）；巡逻用 **Topic 发 `/goal_pose`** 取代嵌套 Action | `COD_Behavior/cod_bt/{singlenav,multiplenav}_tree.xml`（<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature/finish/COD_Behavior/cod_bt>）；`PubNav2Goal` 方案文档 | **[BBS]**「FollowWaypointsAction 偶尔会导致行为树死掉」；文档：「原方案链路 BT → waypoint_follower → bt_navigator（双层 Action，cancel/重发易出问题）；新方案 BT → /goal_pose Topic → bt_navigator（无状态，简单可靠）」 | **需改造**（我们任务层不同）；**值得我们引用的正是"双层 Action 易死"这条工程结论** |
| C22 | 哨兵"急停"配合自瞄 | 血量健康 + 在增益区 + 敌方装甲板距离 < 阈值 → **电控强制速度置零**（静止时自瞄命中率更高） | **[BBS]**（策略描述）；实现落在下位机 `Auto/Chassis_Auto` 分支与视觉判定（[CTL] README「导航模式」段 <https://gitee.com/cod_-control/rmcod2026_-sentry>） | **[BBS]**「由于自瞄在车自身静止时命中率相对较高，所以我们想出了一套『急停』策略」 | **需改造**（我们仿真无自瞄）；可作任务层的一个"停住增益"原语 |
| C23 | 决策阈值/功能触发 | 开局占中心增益区；我方占 → 前压，否则 → 中心区巡逻；**HP<210 回家、>350 出去**；PH<30% 补血；被弹丸攻击 → 加速小陀螺；热量/功率到上限 → 停射 | **[BBS]**（210/350）；`COD_Behavior/README.md`（<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>）：`1 号站位`=哨兵 PH>50% 且我方英雄/步兵占增益区、`2 号站位`=我方英雄与步兵都低血、`补血`=PH<30%、`占领`=开局或敌方占增益区、`识别`=0x0101 且 1→3 视为我方 | **[BBS]** + BT 文档 | **需改造**；注意 **[BBS] 的 210/350 与 BT 文档的百分比阈值不是同一套口径**（两处并列，不要混用） |
| C24 | 下位机：功率/热量/超电/小陀螺（影响可达速度） | 功率：西交模型 + **RLS 递归最小二乘在线拟合参数**，舵向优先 80% 功率；超电：电量 <30% 自动关闭；小陀螺：低速给超电充电、高速降低被瞬秒风险并干扰对方卡尔曼；热量：>100 按 19 弹频打、100→20 线性减速、<=30 停 | **[CTL]** <https://gitee.com/cod_-control/rmcod2026_-sentry>：`RLS_Init(&RLS_Power_Info, 4, 1, 0.99999, 1e-5)`、`W[0..3]=1.453e-07/1.23e-07/1.453e-07/1.23e-07`、`Err_Lower/Upper = 0.01/50`（舵向）与 `0.001/500`（行进）；超电 `Persent>=30 → Power_Max = limit - PID_Buffer.Output + 100`；导航模式 `if(HP>280 && Vision_Grap && Area_Status && Fire_Switch_Spin){Vx=Vy=5}`，`Vw` 每 1200 tick 循环 `6500/4000/2000`，否则 `6000`；热量 `SYS_Qres>100 → Shoot_DP=19`、`>20 && <100 → Shoot_Speed=(d*m-a-3d)/(d*T/100)+a/d`（`m<50`）/`(d*m-a-7d)/…`（`m>=50`）、`<=30 → 0` | **[BBS]** 与 [CTL] README 原文（含功率分配「隶属度」、求根公式解限幅） | **大多不适用**（我们无下位机仿真）；**唯一该抄的是契约**：上位机速度语义必须与下位机轮速换算对齐（见 C17） |
| C25 | 快速迭代：不想每次都起 Gazebo | 把 Nav2 `loopback_sim` 回移到 **Humble**，pixi/RoboStack 一键部署（MacBook Air 验证）；不启 Gazebo 就能跑决策/控制 | [COD_loopback_sim](https://github.com/qza36/COD_loopback_sim)：`nav2_loopback_sim/launch/loopback_simulation.launch.py`、`nav2_loopback_sim/params/nav2_params.yaml`、`maps/{depot,warehouse,tb3_sandbox}.pgm`；配套视频 <https://www.bilibili.com/video/BV1r79cBME6Y/> | README：「Using Gazebo for initial path planning and control testing can be unnecessarily cumbersome. It forces developers to contend with physics-engine artifacts—such as odometry drift and positioning errors—which are distractions at this stage.」 | **直接可用**：正好补我们"必须起 Gazebo"的短板，可做 MPPI/wrapper 的快速 A/B |
| C26 | 他们对"下一步方向"的判断 | nav2 对 RM 哨兵局限太大，认为**无人机式快速探索导航框架**更适合；随后 fork/自建 3D 导航栈 | **[BBS]**「nav2 对于 rm 的哨兵来说局限性还是太大了，个人认为，像无人机的那种快速探索导航框架才是最适合 RM 的」；对应仓库 `pct_planner_ros2_cpu`(<https://github.com/qza36/pct_planner_ros2_cpu>)、`ROG-Map-ROS2`、`far_planner`、`litenav`、`map_module`/`planning_module` | 作者自述 | **不适用（当期）**；作为路线储备（我们不必跟随） |
| C27 | 未知区 / 动态障碍 / ESDF / 前沿探索 | **都没做**（2026 唯一的"时间维"就是 STVL 的 0.5 s linear decay；`esdf`/`frontier` 在 `rmul2026` 全分支 grep 0 命中） | [GH26]（见 [`docs/cod_nav_2026_deep_dive.md`](cod_nav_2026_deep_dive.md) §A15） | 事实 | **别去那儿找答案**：这条修正我们"他们更全"的预期 |

---

## D. 建议的最小拉取集合

> 目标：**只要 4 个来源、12 个文件**就能覆盖 2026 导航的全部"可落地增量"。全部为只读拉取，不编译不运行。

1. **`gitee.com/codnavgation/cod_-rm2026_-navigation` @`master`（163 文件，11 MB 级）** —— 2026 成品导航，优先级最高。
   第一步读：① `README.md`（10 包结构 + 6 组 CSV 语义 + 「KISS / 静态 TF」设计声明）② `src/cod_bringup/params/singlenav2_params.yaml`（单点全套参数）③ `src/goal_approach_controller/src/goal_approach_controller.cpp`（wrapper 实现）④ `src/cod_bringup/launch/singlenav_launch.py`（静态 TF yaw=-0.5 / 裁剪盒值 / RealSense）⑤ `src/cod_bringup/wps/go_gain.csv` + `src/cod_bringup/maps/rmul2026.yaml`（航点格式与场地图）。
   为什么：wrapper、两套 params、场地图、CSV 只在这里。
2. **`gitee.com/codnavgation/cod_-rm2026_-behavior-tree` @`feature/finish`** —— 决策/巡逻与"双层 Action 易死"的工程结论。
   第一步读：① `COD_Behavior/README.md`（触发条件表）② `COD_Behavior/doc/PubNav2Goal_Patrol.md`（Topic vs Action + 5 个节点的端口表）③ `COD_Behavior/cod_bt/singlenav_tree.xml` ④ `COD_Serial/README.md`。
3. **`github.com/qza36/COD_loopback_sim` @`master`** —— 不起 Gazebo 的快速验证通道。
   第一步读：① `README.md`（动机 + `pixi` 用法）② `nav2_loopback_sim/launch/loopback_simulation.launch.py` ③ `nav2_loopback_sim/params/nav2_params.yaml`（对照它默认给的控制/代价图配置）。
4. **`github.com/qza36/waypoint_editor`（分支 `feature/adaptToNav2`）或 Gitee 仓内副本 `src/waypoint_editor`** —— 航点工作流（含 `waypoint_through_nav2` 源码）。
   第一步读：① `USAGE_WITH_NAV2.zh.md` ② `launch/waypoint_through_nav2.launch.py` ③ `src/waypoint_through_nav2.cpp` ④ `data/sample_wp.csv`。
5. **（可选、低成本）`github.com/qza36/box_lidar_filter`** —— 只有 4 个文件；读 `README.md` + `src/filter_node.cpp`，作为 `cpp_lidar_filter` 的等价最小实现（Gitee 那份是同一份代码的更晚版本）。
6. **（历史对照，稀疏拉取）`gitee.com/ustl-cod/cod_nav`**
   `@dyx-dev` 只取 `nav_bringup/launch/bringup.launch.py` + `nav_bringup/params/nav2_params.yaml`（看 `terrain_analysis`/`intensity_voxel_layer` 是怎么被搁置的）；`@master` 只取 `nav_bringup/map/*.yaml` + `test_data/rmul_2025.yaml`（场地资产）。
7. **（接口契约，只读 README）`gitee.com/cod_-control/rmcod2026_-sentry`** —— 只读 README 的「功率控制 / 导航模式 / 哨兵自动开关超电」三节，用来核对 `cmd_vel` 语义与可达速度。
8. **（可选）`github.com/qza36/rmu_gazebo_simulator`（`jazzy`）** —— 只为 `rmul_2025/rmuc_2026` 世界模型与"离线导出 PCD"工具；**注意分支是 `jazzy`，与我们 Humble bench 不通用**，建议只取 world/PCD 资产。

不必要拉的：`COD_NAV_NEXT`（无 README、用途不明）、`sentry_planning`（他队 fork）、研究型 fork 群（本次未读、当期不用）、`gridmap`/`COD_PCD2PGM`（2.5D 路线，与我们当前 linefit/STVL 路线不冲突但非急需）。

---

## E. 未获取到的（明确列出，不含推测）

1. **`terrain_analysis` 的实际源码与来源**：在 `ustl-cod/cod_nav` 的 `dyx-dev` 里只是一个 **gitlink（`160000 commit c82f23c…`）**，而该分支 `.gitmodules` **没有对应条目** ⇒ 无法从该仓库取得内容；`pb_nav2_plugins` README 推荐的是北极熊的 [`terrain_analysis`](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis)，但**COD 用的是否就是这一份，本次未确证**。
2. **`terrain_analysis` 在 COD 里的实际启用状态**：`dyx-dev` 的 `bringup.launch.py` 里 `IncludeLaunchDescription(terrain_analysis_launch.py)` 是**被注释掉的**（<https://gitee.com/ustl-cod/cod_nav/blob/dyx-dev/nav_bringup/launch/bringup.launch.py>），2026 版本也没有该包 ⇒ 只能确认"存在指针 + 未接线"，不能确认历史上是否真跑过。
3. **BBS 帖里的图片内容**：帖中 6 张图/动图（含"提高速度的官方建议"截图、goal wrapper 代码截图、lio 速度 vs cmd_vel 对比图）**未做 OCR**，只记录了它们在文中的位置与图注，图片 URL 见 <https://bbs.robomaster.com/article/1882897>。
4. **B 站两集视频的内容**：无可用字幕接口，**未转写**（<https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>）；只知道 Gitee README 将其标为「该导航程序详细解说教程」。
5. **`small_point_lio` "可以输出速度"的那份修改版**：**[BBS]** 说「这是我修改过的 small point lio，可以输出速度」，但 Gitee 2026 `master` 与 GitHub `rmul2026` 的 `small_point_lio_node.cpp` **完全相同**，其中 `odometry_msg.twist.twist.*` 六行仍是**注释状态**（<https://gitee.com/codnavgation/cod_-rm2026_-navigation/blob/master/src/small_point_lio/src/small_point_lio_node.cpp>）⇒ **该修改版未公开在此仓库**，无法获取。
6. **GitHub `COD_NAV` 的 `ul_mppi`、`rmul2026_sim`、`feature-obstacle_escape_node` 三个分支**：本次未读（前序文档只记了提交日期）。
7. **`myblog` 的导航相关文章**：仓库存在（<https://github.com/qza36/myblog>、<https://qza36.github.io/myblog/>），**本次未逐篇核对**。
8. **`map_module` / `planning_module` / `litenav` / `CODSentryPAC` / `enemy_tracking` / `gridmap` / `COD_PCD2PGM` 的源码内部**：本次只读 README 与文件清单，未逐文件读实现。
9. **`COD_NAV_NEXT` 的用途**：无 README、无描述，`git ls-tree` 只见 `cnn_driver/cnn_perception/cnn_planner` 与 `livox_ros_driver2` vendor ⇒ **用途无法确证**（不推测）。
10. **`sentry-auto-aim` 的导航侧**：其 `使用方法.md` 明确写「**当前并未添加导航模块**」，`rm_bringup/launch/bringup_navigation.launch.py` 引用外部 `rm_navigation` 包，而该包**不在任何已确认的 COD 仓库里** ⇒ 无法获取其内容。
11. **`cod_-control/rmcod2026_-sentry` 的 PDF 文档**：`H7文档.pdf`、`gimbal/Document/Quaternion.pdf`、`原理图/`、`电机说明/` **未下载**（PDF 解析未做）。
12. **`ustl-cod/cod_sentry` 的三个子工作空间内容**：只读了 README（2024 方案），`rm_navigation_ws` / `rm_decision_ws` / `rm_vision_ws` 的 launch/params **未逐一读**。
13. **参数文档与代码不一致之处（已记录但未澄清原因）**：① [BBS] 说 `PathFollow/PathAlign` 权重不宜 >10，而 [G26] 单点代码是 15/15；② [BBS] 说热量控制「100 到 40 之间…40 到 30 之间…」，而 [CTL] 代码是 `>20 && <100` 线性区、`<=30` 停、`m<50`/`m>=50` 两种公式；③ [G26] 注释写「time_steps 80，预测时域 4.8 s」，但 `80 × model_dt 0.02 = 1.6 s`；④ `velocity_smoother.smoothing_frequency 30.0` 注释写「与 controller_frequency 一致」，而 `controller_frequency` 是 `50.0`。以上均为**事实性不一致**，原因未获取。
14. **`rmul2026.pgm` 的像素分布**：只读到 PGM header（`P5`、`255×177`、`255`）与 yaml（`origin [-1.756, -7.036, 0]`、`resolution 0.05`），**未核对占据/自由分布**。
15. **fork 群与 `rmu_gazebo_simulator` 的实际质量**：均**未深读**，本文件只按描述列出（不做评价）。
16. **GitHub 代码搜索未执行**（需鉴权，见 [rate_limit](https://api.github.com/rate_limit)）⇒「哪些第三方仓库抄了 `fake_vel_transform`/`cpp_lidar_filter`」仍无完整清单。
