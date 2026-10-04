# COD_NAV 及其「兄弟仓库」公开开源资料清单

> 调研对象：辽宁科技大学 **COD 战队** RoboMaster 哨兵项目（2024–2026 赛季），主力仓库 <https://github.com/qza36/COD_NAV>（作者 齐志昂 / qza36）。
> 调研日期：**2026-10-04**。
> 调研方式：只读。GitHub REST API、`git ls-remote` / `--filter=blob:none` 裸克隆（可绕开 API 限流）、Gitee API v5、BBS(Nuxt SSR) 页面 payload 解析、bilibili 公开 API、web 检索。
> 本文件是本次调研**唯一写入**的文件，未改动本仓库其它任何文件。
> 局限：GitHub 未鉴权调用限额 60 次/小时（<https://api.github.com/rate_limit>），**代码搜索 API 需鉴权**，故 `search/code` 未执行；bilibili 空间归档 API 返回风控 `-799`，无法枚举 UP 主全部投稿。这些在 §F 明确列出。

---

## A. 结论：是不是「全部开源」？

**结论：COD_NAV 只是 COD 战队 2025 赛季导航上位机的其中一个仓库。战队在 GitHub（个人号 `qza36` + 队员 `GrassFanWang`）与 Gitee（3 个组织）上按「导航 / 决策 / 自瞄 / 下位机」分模块开源；2026 赛季这四个模块的仓库均已逐个核实存在（帖中自述「全开源」，见 <https://bbs.robomaster.com/article/1882897>）。但没有任何统一索引，且 **2026 赛季代码几乎全部在 Gitee 而非 GitHub**。**

已在公开渠道确认存在的内容（每条附证据链接）：

1. **2026 赛季四件套（全在 Gitee）**——BBS 开源帖标题即「导航、决策、自瞄、下位机控制全开源」：
   - 导航：<https://gitee.com/codnavgation/cod_-rm2026_-navigation>（15 ★，163 文件，仅 `master`）
   - 决策：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（4 ★，9 分支，含 `COD_Behavior` + `COD_Serial` + `rm_interfaces`）
   - 自瞄：<https://gitee.com/ustl-cod/sentry-auto-aim>（含 `rm_bringup`）
   - 下位机：<https://gitee.com/cod_-control/rmcod2026_-sentry>（11 ★，857 文件）
   - 出处：<https://bbs.robomaster.com/article/1882897>
2. **2025 赛季（GitHub `qza36`）**：`COD_NAV`（导航，<https://github.com/qza36/COD_NAV>）、`COD_Behavior`（决策，<https://github.com/qza36/COD_Behavior>）、`ros2_simple_serial`（串口，<https://github.com/qza36/ros2_simple_serial>）、`Sentry_vision`（视觉，<https://github.com/qza36/Sentry_vision>），并由汇总工作空间 `Sentry_ws` 的 `.gitmodules` 指向这四个仓（<https://github.com/qza36/Sentry_ws/blob/master/.gitmodules>）。
3. **2024 赛季**：`ustl-cod/cod_sentry`（哨兵上位机 + 实现思路文档，<https://gitee.com/ustl-cod/cod_sentry>）、`ustl-cod/cod-2024-radar`（雷达站，29 ★，<https://gitee.com/ustl-cod/cod-2024-radar>），论坛帖 <https://bbs.robomaster.com/wiki/4577/9649>。
4. **其它子系统**：机械（<https://bbs.robomaster.com/article/1883384>）、电控 H7 通用控制系统（<https://bbs.robomaster.com/article/434199>）、工程机械（<https://bbs.robomaster.com/article/1883259>）、串联腿 MJCF/USD（<https://bbs.robomaster.com/article/1315669>）。
5. **教学/科普产出**：导航分享视频 2 集（<https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>）、新人知识库（<https://gitee.com/ustl-cod/cod_beginner>）、2024 算法组培训（<https://gitee.com/ustl-cod/2024-algorithm-group-training>）。

**确认「不在公开渠道」或只能间接获得的内容**（详见 §D）：

- `spatio_temporal_voxel_layer`、`slam_toolbox`、`nav2_*`、`patchwork-plusplus`、`pcd2pgm`、`pb_nav2_plugins`、`waypoint_editor` 上游：都是第三方，COD 仓里只留参数/子模块指针，不是 COD 自研开源。
- `livox_ros_driver2`：2026 赛季被从 COD_NAV 删除，CI 只用 `qza36` 的第三方 fork（<https://github.com/qza36/livox_driver2_ros2>），见 <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>。
- **没有**找到 2025 赛季「导航」单独的论坛开源帖（2025 导航只在 GitHub，未在 BBS 单独立帖）；**没有** GitHub Wiki 页面（见 §F）。

---

## B. 资料清单表

### B1. 代码仓库 —— GitHub（个人号 `qza36`，共 74 个仓库，含 fork）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| COD_NAV（2025 哨兵导航） | <https://github.com/qza36/COD_NAV> | 代码 | 30 ★ / 3 fork / BSD-3-Clause，创建 2024-12-12，最后推送 2026-03-10；5 分支、0 issues、2 已关闭 PR、无 tag/release | 我们的主要对比对象（见 `docs/cod_nav_comparison.md`） |
| 分支 master | <https://github.com/qza36/COD_NAV/tree/master> | 代码 | 80 commits，2025-08-02；FAST-LIO fork + patchwork++ + small_gicp 重定位 + pcd2pgm 先验图 + pb_omni_pid 控制器 + rm_simulation（Gazebo UL24） | 唯一带 Gazebo 仿真的分支；`rm_simulation/pb_rm_simulation/world/TestWorld/` |
| 分支 rmul2026 | <https://github.com/qza36/COD_NAV/tree/rmul2026> | 代码 | 116 commits，2026-03-10；small_point_lio + slam_toolbox lifelong + cpp_lidar_filter + MPPI Omni + 双 STVL；含 `CLAUDE.md` 架构文档 | 参数配方来源（MPPI/STVL/裁剪盒） |
| 分支 ul_mppi | <https://github.com/qza36/COD_NAV/tree/ul_mppi> | 代码 | 115 commits，2026-03-10，与 rmul2026 同源（"increase min pcl"） | 参数微调对照 |
| 分支 rmul2026_sim | <https://github.com/qza36/COD_NAV/tree/rmul2026_sim> | 代码 | 112 commits，2026-01-04，`fake_vel_transform` 适配仿真 | 仿真→实车改法 |
| 分支 feature-obstacle_escape_node | <https://github.com/qza36/COD_NAV/tree/feature-obstacle_escape_node> | 代码 | 69 commits，2025-05-07，新增 `obstacle_escape` 包 | 脱困实现（我们尚未做） |
| COD_Behavior（2025 决策） | <https://github.com/qza36/COD_Behavior> | 代码 | 5 ★；`cod_bt/t1.xml`、`src/sentry_behavior.cpp`、`src/CheckNavResult.cpp`、`include/cod_behavior/sentry_behavior_actions.hpp` | **缺失模块 `cod_behavior` 的公开版本** |
| ros2_simple_serial（串口） | <https://github.com/qza36/ros2_simple_serial> | 代码 | 1 ★；CMake target/IDE 工程名为 `cod_serial`（`.idea/cod_serial.iml`）；订阅 `/cmd_vel` → 串口 `/dev/ACM0`；README 自述「下位机连发导致串口卡死」已知问题 | **缺失模块 `cod_serial` 的公开版本** |
| Sentry_ws（汇总工作空间） | <https://github.com/qza36/Sentry_ws> | 代码/文档 | 仅 8 文件；`.gitmodules` 串起 COD_Behavior / COD_NAV / Sentry_vision / ros2_simple_serial，README 一行「COD-哨兵工作空间」 | 一次性拉齐全栈的入口 |
| Sentry_vision（2025 视觉） | <https://github.com/qza36/Sentry_vision> | 代码 | 201 文件；FYT2024 视觉框架改；**含 `rm_bringup/`**、`rm_auto_aim/`、`fyt修改rv记录.md` | **缺失模块 `rm_bringup` 的公开版本（视觉侧）** |
| COD_auto_aim（2024 自瞄） | <https://github.com/qza36/COD_auto_aim> | 代码 | 2024-03，AngleSolver/ArmorDetector/Buffdetect 等传统自瞄源码 | 历史版本参考 |
| COD_loopback_sim | <https://github.com/qza36/COD_loopback_sim> | 代码/文档 | 7 ★；把 Nav2 `loopback_sim` 回移到 Humble，RoboStack+pixi 一键部署，可在 macOS 上跑；README 解释「为何不用 Gazebo 先验验证控制」 | **仿真 bench 可直接借鉴**（我们目前依赖 Gazebo） |
| waypoint_editor（fork） | <https://github.com/qza36/waypoint_editor> | 代码/文档 | 2 ★，`feature/adaptToNav2` 分支有 `USAGE_WITH_NAV2.zh.md` | 多点航点编辑 + CSV 导出 |
| box_lidar_filter | <https://github.com/qza36/box_lidar_filter> | 代码/文档 | `pcl::CropBox` 挖掉车体点云；README 明确写「因 STVL 无法设置最小障碍半径」而开发，给出一组 `min/max_x/y/z` | **3D→2D/代价图近距遮挡的直接对策** |
| pointcloud_fusion | <https://github.com/qza36/pointcloud_fusion> | 代码 | 多雷达点云 TF 对齐 + 近似时间同步融合 | 多传感器方案储备 |
| map_module | <https://github.com/qza36/map_module> | 代码 | 2D map module，部分改写自 `nav2_map_server` | 地图模块最小实现参考 |
| planning_module | <https://github.com/qza36/planning_module> | 代码 | 2D plan module，与 `map_module` 配套 | 规划模块最小实现参考 |
| litenav | <https://github.com/qza36/litenav> | 代码 | 2026-05；`app_module/core_types/litenav_ros2/map_module/plan_module`，pixi + robostack，含 `.codex` | 作者新一代自研导航（脱离 Nav2 的雏形） |
| enemy_tracking | <https://github.com/qza36/enemy_tracking> | 代码/文档 | 敌人追踪导航包：订阅 `rm_interfaces/msg/Target`，在敌人周围生成候选攻击点→costmap 过滤→选最近可达点→调 Nav2；参考 PolarBear `pb2025_sentry_behavior` | 哨兵「追击/卡位」任务层可迁移 |
| CODSentryPAC | <https://github.com/qza36/CODSentryPAC> | 代码/文档 | 2026-04；轨迹生成系统（ReplanFSM 状态机 + planner_manager，面向低速差速履带） | 与 Nav2 不同的自研规划架构 |
| COD_NAV_NEXT | <https://github.com/qza36/COD_NAV_NEXT> | 代码 | 2025-09；顶层 `cnn_driver/cnn_perception/cnn_planner`（无 README） | 早期「下一代」尝试 |
| COD_PCD2PGM | <https://github.com/qza36/COD_PCD2PGM> | 代码/文档 | pcd → 2.5D 高程栅格并保存地图；pixi 管理 | 2.5D 高程图路线 |
| livox_converter | <https://github.com/qza36/livox_converter> | 代码/文档 | Livox CustomMsg → `PointCloud2`（保留 intensity/tag/line） | 若不用 livox 官方驱动作转换 |
| livox_driver2_ros2 | <https://github.com/qza36/livox_driver2_ros2> | 代码 | 2026 赛季 CI 用的 Livox 驱动 fork（原版被从 COD_NAV 删除） | 驱动版本对齐 |
| rm_interfaces | <https://github.com/qza36/rm_interfaces> | 代码 | 自定义 msg/srv 包 | 与决策/自瞄接口对齐 |
| myblog（个人博客） | <https://github.com/qza36/myblog> / <https://qza36.github.io/myblog/> | 文档 | 作者个人主页 + 1 篇技术文（2026/01/14 CLion + micro-ROS）；自述「architected my RoboMaster team's full navigation-and-decision stack」 | 作者视角的项目定位 |
| Sentry_ws 子模块之外的历史仓 | <https://github.com/qza36?tab=repositories> | 索引 | 另有 `dataPreprocess`、`Color_TXT`、`gridmap`、`aloam-ros2`、`AStar_Matlab`、`PersonWOWPlugins` 等非哨兵仓 | 工具类可复用 |

### B2. 代码仓库 —— Gitee（3 个组织）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| 组织 ustl-cod（算法组，13 仓） | <https://gitee.com/ustl-cod> | 索引 | COD 算法组组织主页 | 找历史方案的总入口 |
| cod_sentry（2024 哨兵上位机） | <https://gitee.com/ustl-cod/cod_sentry> | 代码+文档 | 2 ★，771 文件，`master`/`自瞄` 分支；含 `rm_decision_ws`（BehaviorTree.ROS2 + rm_behavior_tree + rm_decision_interfaces）与 `rm_navigation_ws`（FAST_LIO/point_lio/rm_driver/rm_navigation），README 即《2024 赛季哨兵上位机方案》 | **设计文档级**：三模块分工与 2024 架构 |
| cod-2024-radar | <https://gitee.com/ustl-cod/cod-2024-radar> | 代码 | 29 ★，无激光雷达/相机的纯运算端雷达站 | 历史 |
| cod_beginner | <https://gitee.com/ustl-cod/cod_beginner> | 文档 | 12 ★；`Linux/`、`Docker/`、`git/git.md`、`SLAM/ros2_setup_notes.pdf`、C++/Python/OpenCV 教程 | 新人上手材料 |
| 2024-algorithm-group-training | <https://gitee.com/ustl-cod/2024-algorithm-group-training> | 文档 | 15 ★；培训计划/大纲（FlowUs 外链）+ 培训作业 | 培训体系参考 |
| sentry-auto-aim（2026 自瞄） | <https://gitee.com/ustl-cod/sentry-auto-aim> | 代码+文档 | 1 ★；9 分支（`tmp`、`sentry_sp_version`、`feature/YOLO-Detection`…）；含 `rm_bringup/`、`rm_auto_aim/`、`rm_hardware_driver/`、`rm_rune/`、`rm_upstart/`、`使用方法.md`、`OPTIMIZATION.md` | **`rm_bringup` + 上下位机 16 字节帧协议** |
| cod_nav（Gitee 镜像 fork） | <https://gitee.com/ustl-cod/cod_nav> | 代码 | fork=True；**比 GitHub 多 `dev`、`dyx-dev` 两个分支**（GitHub 上已无），`dyx-dev` 含 `terrain_analysis/`，`master` 含 `codmap.pgm/yaml`、`nav_bringup/map/rmuc_2024.*`、`rmul_2024.*`、`test*.pgm`、`test_data`；其 `rmul2026` 分支是另一条中间路线（FAST_LIO_ROS2 + loam_interface + sensor_scan_generation） | **GitHub 上已看不到的历史分支与场地地图文件** |
| 组织 codnavgation（2026 导航/决策组，4 仓） | <https://gitee.com/codnavgation> | 索引 | 2026 赛季主组织 | 2026 代码入口 |
| cod_-rm2026_-navigation | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | 代码+文档 | 15 ★，163 文件，仅 master；`cod_bringup`（launch/params/maps/wps CSV/bt xml）、`cpp_lidar_filter`、`fake_vel_transform`、**`goal_approach_controller`**、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`pointcloud_to_laserscan`、**`ros2_simple_serial`**、`small_point_lio`、`waypoint_editor` | 2026 导航最新版；CSV 航点（去增益/回家/巡逻）+ 目标点附近限速 wrapper |
| cod_-rm2026_-behavior-tree | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> | 代码+文档 | 4 ★，9 分支；`COD_Behavior`、**`COD_Serial`**、`rm_interfaces`、`BehaviorTree.ROS2`、`Project.btproj` | 2026 决策 + 串口（见 §D） |
| cod_-rm2026_-perception | <https://gitee.com/codnavgation/cod_-rm2026_-perception> | 占位 | 3 文件，README 仍是 Gitee 模板占位 | 无实质内容 |
| cod_-rm2026_-simluation | <https://gitee.com/codnavgation/cod_-rm2026_-simluation> | 占位 | 2 文件，README 占位 | 无实质内容 |
| 组织 cod_-control（电控组，18 仓） | <https://gitee.com/cod_-control> | 索引 | 2026 赛季电控主组织 | 下位机入口 |
| rmcod2026_-sentry | <https://gitee.com/cod_-control/rmcod2026_-sentry> | 代码+文档 | 11 ★，`dev`/`master`/`CI`；857 文件，含 `chassis/`、`gimbal/`（含 `gimbal/Document/Quaternion.pdf`）、`H7文档.pdf`、`原理图/`、`电机说明/`；README 详述舵轮解算公式、云台/底盘 CAN 通信、功率与热量控制 | **上下位机接口与底盘运动学的权威出处** |
| sentry_steering_25 | <https://gitee.com/cod_-control/sentry_steering_25> | 代码 | 2025 舵轮哨兵电控 | 与 COD_NAV(master) 同赛季 |
| cod_-h7_-template / COD_F4_Template / hero_2025 / rmcod2026_-infantry_-balance 等 | <https://gitee.com/cod_-control/cod_-h7_-template> | 代码 | H7 通用控制模板（2026-10-01 仍有推送）与其它兵种 | 电控底座 |

### B3. 论坛帖（bbs.robomaster.com）

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| **RM2026 哨兵导航/决策/自瞄/下位机全开源**（作者 齐志昂，2026-03-31，15167 阅读） | <https://bbs.robomaster.com/article/1882897> | 帖子 | 全文含：建图方案（slam_toolbox + small_point_lio，纯 LIO 定位）、**Navigate Through Poses 取代单点导航**、waypoint_editor + CSV、STVL 取代 VoxelLayer 的理由、MPPI 调参经验（temperature/gamma、PathFollow/PathAlign 权重<10、阈值<1.0、GoalCritic 加权的「邪修」）、目标点附近圆周运动的 wrapper 解法、`PositionGoalChecker`、fake yaw/nav 树来自北极熊、决策逻辑（血量 210/350 阈值、开局占中心增益、急停策略）、下位机（超电、变速小陀螺、热量控制 19 弹频、功率控制 RLS 拟合）与各模块仓库链接 | **本次调研价值最高的一份**，等于一份答辩/技术报告 |
| RM2026 哨兵舵步机械结构开源 | <https://bbs.robomaster.com/article/1883384> | 帖子+附件 | 机械开源 zip + 《结构部分说明文档.pdf》 | 机械/整备信息 |
| **RM2025 STM32H7 电控通用控制系统开源**（作者 王草凡，2025-06-11，10253 阅读） | <https://bbs.robomaster.com/article/434199> | 帖子+附件 | 达妙 MC-02(STM32H723VGT6) 通用控制系统；CLion/Arm GNU 版；链接 GitHub/Gitee 双仓 + `README.pdf`；配套知乎 STM32H7 系列教程 | 下位机底座；电控文档 |
| RM2025 工程机器人机械结构开源 | <https://bbs.robomaster.com/article/1883259> | 帖子+附件 | 结构说明 PDF + 机械 zip | 旁支 |
| RM2026 串联腿闭链结构 MJCF&USD 开源 | <https://bbs.robomaster.com/article/1315669> | 帖子+代码 | <https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File>（41 ★），IsaacSim5/MuJoCo 文件 + 视频 | 平衡/仿真资产 |
| RM2024 雷达站算法开源（算法开源专栏） | <https://bbs.robomaster.com/wiki/4577/9649> | 帖子 | 2024 雷达站算法 | 历史 |

### B4. 视频 / 社区

| 资料 | 链接 | 类型 | 内容摘要 | 对我们有用的部分 |
|---|---|---|---|---|
| RM COD 导航分享一（UP：昂哥救我，2026-03-31） | <https://www.bilibili.com/video/BV1XSXZBUEYL/> | 视频 | 2291 播放；Gitee 导航仓 README 明确推荐为「该导航程序详细解说教程」 | **导航方案讲解（对应 gitee 导航仓）** |
| RM COD 导航分享二：loopback sim 和 waypoint editor（2026-03-31） | <https://www.bilibili.com/video/BV1r79cBME6Y/> | 视频 | 1253 播放；讲 loopback 仿真与航点编辑器 | 仿真/航点工作流 |
| COD26 联盟赛东北站季军哨兵集锦（UP：王草凡，2026-04-01） | <https://www.bilibili.com/video/BV1759KB1EXK/> | 视频 | 3127 播放；简介直接指向 BBS 1882897 | 实车表现参考 |
| 串联腿仿真演示 | <https://www.bilibili.com/video/BV1Z5mGB7ESd/>、<https://www.bilibili.com/video/BV1CLm8B6E82/> | 视频 | 出处 <https://bbs.robomaster.com/article/1315669> | 旁支 |
| bilibili 镜像站（同视频） | <https://www.snm0516.aisee.tv/video/BV1759KB1EXK/> | 视频 | 第三方镜像 | 备用 |

---

## C. 与 COD_NAV 仓库的关系

### C1. 同一战队的其它模块/仓库（**不是**外部项目）

| 模块名（COD_NAV 里被引用的） | 是否存在公开版本 | 证据 |
|---|---|---|
| `cod_behavior` | ✅ 有，两代 | 2025：<https://github.com/qza36/COD_Behavior>；2026：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（`COD_Behavior/`） |
| `cod_serial` | ✅ 有 | <https://github.com/qza36/ros2_simple_serial>（IDE 工程名 `cod_serial.iml`、旧 CMake target `cod_serial`） |
| `cod_serial_ul26` | ✅ 有 | Gitee 导航仓 `src/ros2_simple_serial/CMakeLists.txt` 里 `project(cod_serial_ul26)`、`package.xml` `<name>cod_serial_ul26</name>`，并被 `cod_bringup/launch/multiplenav_launch.py`、`singlenav_launch.py` 启动：<https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| `rm_bringup` | ✅ 有，但属**视觉/自瞄栈** | <https://gitee.com/ustl-cod/sentry-auto-aim>（`rm_bringup/launch/bringup.launch.py`、`bringup_navigation.launch.py`、`config/node_params/*`）；<https://github.com/qza36/Sentry_vision> 同样含 `rm_bringup/`。导航侧的启动包名是 `nav_bringup`（COD_NAV）/ `cod_bringup`（Gitee 2026） |
| `rm_interfaces` | ✅ 有 | <https://gitee.com/ustl-cod/cod_sentry>、<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>（`rm_interfaces/`）、<https://github.com/qza36/rm_interfaces> |
| 汇总关系 | `Sentry_ws` 用 submodule 明确指向 COD_Behavior / COD_NAV / Sentry_vision / ros2_simple_serial | <https://github.com/qza36/Sentry_ws/blob/master/.gitmodules> |

**2025 → 2026 的迁移关系**（同战队内）：2025 导航 = GitHub `COD_NAV`（FAST-LIO 系）；2026 导航 = Gitee `cod_-rm2026_-navigation`（small_point_lio + slam_toolbox + MPPI，LIO 换成 ACE 战队的 small_point_lio，作者在帖中致谢 ACE 开源：<https://bbs.robomaster.com/article/1882897>）。两者不是同一份代码，**2026 版本不在 COD_NAV 任何分支里**（COD_NAV 5 个分支成员来自 `git ls-remote`：master / rmul2026 / ul_mppi / rmul2026_sim / feature-obstacle_escape_node）。

### C2. **其它战队**的项目（明确区分，别混淆）

| 项目 | 链接 | 归属 |
|---|---|---|
| pb2025_sentry_nav（440 ★） | <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav> | 深圳北理莫斯科大学 北极熊战队 —— **本项目（HzMi_rmsimulation）的上游**，COD 明确「抄」了其导航树/脱困插件/fake yaw（<https://bbs.robomaster.com/article/1882897>） |
| pb_omni_pid_pursuit_controller（18 ★） | <https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller> | 北极熊；COD_NAV 里是 `qza36` 的 fork 子模块 |
| pb_nav2_plugins | <https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins> | 北极熊；COD_NAV `master` 的 `.gitmodules` 直接指向它 |
| sentry_navigation（65 ★） | <https://github.com/IRobot-Algorithm/sentry_navigation> | 西安电子科技大学 IRobot 2024 哨兵导航 |
| SCURM_SentryNavigation（378 ★） | <https://github.com/PolarisXQ/SCURM_SentryNavigation> | 其它战队（非 COD） |
| PnX-HKUSTGZ/sentry-navigation（3 ★，fork） | <https://github.com/PnX-HKUSTGZ/sentry-navigation> | 港科大广州；Mid360 + FASTLIO 仿真 |
| laohao78/ROS2_RM_Navigation（36 ★） | <https://github.com/laohao78/ROS2_RM_Navigation> | 非 COD；RM 场地 Nav2 全流程 |
| small_point_lio（121 ★） | <https://github.com/Yancey2023/small_point_lio> | ACE 战队；被 COD 2026 采用并改速度输出（<https://bbs.robomaster.com/article/1882897>） |
| spatio_temporal_voxel_layer | <https://github.com/SteveMacenski/spatio_temporal_voxel_layer> | 第三方（Nav2 社区） |
| patchwork-plusplus | <https://github.com/url-kaist/patchwork-plusplus> | 第三方；COD_NAV `master` 子模块 |
| pcd2pgm | <https://github.com/LihanChen2004/pcd2pgm> | 第三方（北极熊 Lihan Chen）；COD_NAV `master` 子模块 |
| waypoint_editor 上游 | <https://github.com/kzm784/waypoint_editor> | 第三方；COD 用 fork <https://github.com/qza36/waypoint_editor> |
| COD-2026 平衡仿真文件 | <https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File> | **COD 队员** GrassFanWang（王草凡）个人号 —— 算 COD 的 |
| COD-H7-Template / CLion 版 | <https://github.com/GrassFanWang/COD-H7-Template>、<https://github.com/GrassFanWang/COD_H7_Template_CLion>、<https://gitee.com/wangcaofan/cod-h7-template> | **COD 电控**（论坛帖 <https://bbs.robomaster.com/article/434199>） |
| 0xUwUi/fake_vel_transform | <https://github.com/0xUwUi/fake_vel_transform> | 第三方或他队同名实现（0 ★，无法确认归属，仅记录） |

> 另注：`fake_vel_transform` 这个标识符同时出现在 COD_NAV、其它战队的 fork 中；GitHub 代码搜索未执行（需鉴权），因此**没有**跨仓同名文件的完整清单。

---

## D. 缺失模块清单（COD_NAV 引用但仓库内没有）

| 模块 | 有无公开替代 / 搜到了什么 | 证据链接 |
|---|---|---|
| `cod_behavior` | ✅ **有**。2025 版：<https://github.com/qza36/COD_Behavior>（`cod_bt/t1.xml`、`sentry_behavior.cpp`、`CheckNavResult.cpp`）；2026 版：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> 的 `COD_Behavior/`，文档 <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md> | 同左 |
| `cod_serial` | ✅ **有**。<https://github.com/qza36/ros2_simple_serial>；2026 决策仓里另有一份 `COD_Serial/`（含 `include/message.h` 裁判系统结构体、`src/cod_serial.cpp`、`src/simple_serial.cpp`）：<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree> | 同左 |
| `cod_serial_ul26` | ✅ **有**，即 Gitee 导航仓内的 `src/ros2_simple_serial`（package 名 `cod_serial_ul26`），被 2026 的两个 launch 调用 | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| `rm_bringup` | ✅ **有**，但在自瞄/视觉栈：<https://gitee.com/ustl-cod/sentry-auto-aim>（`rm_bringup/`，含 `bringup.launch.py`、`bringup_navigation.launch.py`、`config/node_params/*.yaml`）与 <https://github.com/qza36/Sentry_vision>。**导航**的等价物是 `nav_bringup`（COD_NAV）/ `cod_bringup`（Gitee 2026），不含 `rm_bringup` 同名包 | 同左 |
| `rm_interfaces` | ✅ **有** | <https://gitee.com/ustl-cod/cod_sentry>、<https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree>、<https://github.com/qza36/rm_interfaces> |
| `livox_ros_driver2` | ⚠️ 被 COD_NAV 2026 分支**移除**，CI 用第三方 fork | <https://github.com/qza36/livox_driver2_ros2>、<https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>（"External Dependencies (not in this repo)"） |
| `spatio_temporal_voxel_layer` | ⚠️ 上游第三方，COD 仓只留参数 | <https://github.com/SteveMacenski/spatio_temporal_voxel_layer> |
| `slam_toolbox` / `nav2_*` / `nav2_mppi_controller` | ⚠️ 上游 ROS 2 官方组件，无 COD 版本 | 无 COD 专属链接 |
| `fast_lio`（rmul2026 launch 里的死引用） | ⚠️ 该分支实际 LIO 是 `small_point_lio`，`fast_lio` 是残留引用 | <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md> |
| `terrain_analysis`（仅出现在 Gitee 老分支） | ✅ 在 Gitee 镜像 fork 的 `dyx-dev` 分支中可见 | <https://gitee.com/ustl-cod/cod_nav>（分支 `dyx-dev`） |

---

## E. 文档质量评估：哪些是「设计文档级」

按可读性与信息密度排序，最有价值的 5 份：

1. **BBS《RM2026-哨兵机器人导航、决策、自瞄、下位机控制全开源》** — <https://bbs.robomaster.com/article/1882897>
   最接近答辩材料：写清了方案取舍（为何抛弃 slam 单点导航改 Navigate Through Poses、为何用 STVL 而非 VoxelLayer）、MPPI 具体调参建议、目标点附近圆周运动的 wrapper 解法、决策逻辑阈值（血量 210/350）、下位机功率/热量控制策略与致谢参考来源。缺点：无公式化推导、无失败数据。
2. **COD_NAV `rmul2026` 分支 `CLAUDE.md`** — <https://github.com/qza36/COD_NAV/blob/rmul2026/CLAUDE.md>
   架构说明级：包职责表、数据流图、TF 树、关键参数文件与 MPPI 调参思路、"External Dependencies (not in this repo)" 清单，甚至记录了 `bt_navigator.odom_topic: "odomety"` 拼写 bug。
3. **《COD 哨兵决策说明文档（26 赛季 RMUL）》** — <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/README.md>
   逐条列出裁判系统消息（0x0001/0x0002/0x0101/0x0201/0x0203/0x0206…）与哨兵各功能的**触发条件**，附「未实现功能」章节；是接口级设计文档。
4. **Gitee `ustl-cod/cod_sentry` 的《2024 赛季哨兵上位机方案》** — <https://gitee.com/ustl-cod/cod_sentry/blob/master/README.md>
   讲清 2024 三模块（决策/导航/自瞄）分工、环境与硬件（Mid360、i7-1270P、海康 MV-CS016-10UC 8mm）、目录框架；可看清三年架构演进起点。
5. **《PubNav2Goal 巡逻方案使用文档》** — <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/blob/feature/finish/COD_Behavior/doc/PubNav2Goal_Patrol.md>
   节点级方案文档：`LoadWaypoints/GetCurrentWaypoint/WaitUntilReached/WaitDuration/NextWaypoint` 端口表 + 行为树结构 + 「BT→waypoint_follower→bt_navigator 双层 Action 易出问题，改 BT→/goal_pose Topic」的动机说明。同类还有 `COD_Serial/README.md`。

补充（非导航但同样成体系）：`rmcod2026_-sentry/README.md` 给出舵轮解算公式与功率/热量控制实现（<https://gitee.com/cod_-control/rmcod2026_-sentry>）；`cod_beginner` 是新人知识库（<https://gitee.com/ustl-cod/cod_beginner>）；导航讲解视频 <https://www.bilibili.com/video/BV1XSXZBUEYL/>、<https://www.bilibili.com/video/BV1r79cBME6Y/>。

---

## F. 未能找到 / 无法访问（不编造）

1. **GitHub 代码搜索未执行**：`/search/code` 返回 403（需鉴权），见 <https://api.github.com/rate_limit> 与 <https://docs.github.com/rest/search/search>。因此「`fake_vel_transform` / `cpp_lidar_filter` / `small_point_lio` / `COD_NAV` / `pb_omni_pid_pursuit_controller` 被哪些第三方仓库复制」**没有**完整清单；仅通过仓库名搜索确认：`fake_vel_transform`→<https://github.com/0xUwUi/fake_vel_transform>、`small_point_lio`→<https://github.com/Yancey2023/small_point_lio>（121 ★，COD 采用的上游）、`pb_omni_pid_pursuit_controller`→<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller>。`grep.app` 检索被 Vercel 人机校验拦截，未取到结果。
2. **GitHub Wiki 为空**：仓库元数据 `has_wiki: true`（<https://api.github.com/repos/qza36/COD_NAV>），但 <https://github.com/qza36/COD_NAV/wiki> 与 `/wiki/Home` 均 302/跳回仓库首页，且 `git ls-remote https://github.com/qza36/COD_NAV.wiki.git` 要求登录（即 wiki 仓不存在）⇒ **没有任何 wiki 页面**。
3. **Issues / PR / Release**：`issues?state=all` 返回 2 条，均为已关闭 PR（#1「稳定版本」2025-05-01、#2「Dyx dev」2025-07-17），无 issue、无 tag、无 release（`git ls-remote --tags` 为空）。URL：<https://github.com/qza36/COD_NAV/pulls?q=is%3Apr>。
4. **3 个 fork 无额外内容**：<https://github.com/ZeroHour-Z/COD_NAV>、<https://github.com/loml13/COD_NAV>、<https://github.com/U-Nori/COD_NAV>，`git ls-remote --heads` 均只有 `master`；fork 列表亦见 <https://api.github.com/repos/qza36/COD_NAV/forks>。
5. **bilibili 投稿无法全量枚举**：空间归档 API 返回 `code:-799 请求过于频繁`；仅确认 3 个视频（§B4）。UP「昂哥救我」是否还有「导航分享三」**无法核实**。
6. **知乎 / CSDN / 微信公众号**：未检索到 COD 关于导航的第三方分析文或官方公众号文章。BBS 434199 中出现的知乎链接（<https://zhuanlan.zhihu.com/p/714301640>、<https://zhuanlan.zhihu.com/p/720966722>、<https://zhuanlan.zhihu.com/p/4218673539>）是**电控 STM32H7 教程**，与导航无关。`gitee.com/zhang-jire/bisai`（<https://gitee.com/zhang-jire/bisai>）是队员个人仓库，未见导航文档。
7. **无 2025 赛季导航专属论坛帖**：站内检索（含 BBS「算法开源专栏」<https://bbs.robomaster.com/wiki/4577> 的完整目录、关键词检索）只找到 COD 的 2024 雷达（<https://bbs.robomaster.com/wiki/4577/9649>）、2025 电控（<https://bbs.robomaster.com/article/434199>）、2025 工程机械（<https://bbs.robomaster.com/article/1883259>）、2026 导航/决策/自瞄/下位机（<https://bbs.robomaster.com/article/1882897>）、2026 哨兵机械（<https://bbs.robomaster.com/article/1883384>）、2026 串联腿（<https://bbs.robomaster.com/article/1315669>）。**未发现** 2025 赛季哨兵导航的单独开源帖。
8. **占位仓库无内容**：<https://gitee.com/codnavgation/cod_-rm2026_-perception>、<https://gitee.com/codnavgation/cod_-rm2026_-simluation>、<https://gitee.com/ustl-cod/rm2026-cod_-radar> 的 README 仍是 Gitee 模板文本。
9. **无法核实的细节**：Gitee 部分仓库的许可证未在页面上确认；`https://github.com/qza36/COD_NAV_NEXT` 无 README、无描述，仅顶层 `cnn_driver/cnn_perception/cnn_planner`，用途只能从命名推断（**标注为推断**）。
10. **BBS 未开放检索/REST API**：`/developers-server/rest/article/...`、`/rest/search` 等推测端点均 404（实测 2026-10-04），站内检索只能靠搜索引擎与页面 payload 解析，故 §B3 的帖子清单可能不完整。

---

## 附：本次调研可直接落地的三条

1. **近距/自身点云**：`box_lidar_filter`（<https://github.com/qza36/box_lidar_filter>）给出「STVL 无法设最小障碍半径 ⇒ 用 CropBox 挖掉车体」的完整参数样例；2026 导航仓对应包名 `cpp_lidar_filter`。
2. **目标点附近震荡**：2026 方案用 `goal_approach_controller` 做限速 wrapper 而不是继续调 MPPI（<https://gitee.com/codnavgation/cod_-rm2026_-navigation>，帖子说明见 <https://bbs.robomaster.com/article/1882897>）。
3. **不启动 Gazebo 的先验验证**：`COD_loopback_sim`（<https://github.com/qza36/COD_loopback_sim>）+ `waypoint_editor`（<https://github.com/qza36/waypoint_editor>）组合，可先在 Humble 上把多航点逻辑跑通；配套视频 <https://www.bilibili.com/video/BV1r79cBME6Y/>。
