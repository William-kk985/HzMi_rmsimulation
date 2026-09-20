# 术语表（Glossary）

> 用途：把本工程与 ROS 导航/SLAM 生态里反复出现的名词**一句话定义清楚**，并指向详细讨论所在章节。
> 排布：按主题分组；`→ 详见` 指向本文档体系中的出处。
> 维护：新增术语随手加；详细长文一律放 `architecture.md` / `3d_to_2d_survey.md`，本表只做"索引 + 一句话"。

---

## 一、定位与位姿

| 术语 | 一句话定义 | 详见 |
|---|---|---|
| **里程计（odometry）** | 从传感器推算"相对起点走了多少"的连续位姿，高频但**会漂**，无全局参考 | `architecture.md` §3.2.1 |
| **LIO（LiDAR-Inertial Odometry）** | 激光 + IMU 的里程计；本工程用 FAST-LIO / Point-LIO。**它同时维护一张 3D 点云地图，但无回环** | `architecture.md` §3.2.1、`rm_algorithm_catalog.md` §〇.0 |
| **前端 / 后端（front-end / back-end）** | 前端 = 逐帧匹配出相对位姿（快、局部）；后端 = 位姿图优化修正全局（慢、全局）。LIO 只有前端，Cartographer 两者都有 | `rm_algorithm_catalog.md` §〇.0 |
| **扫描匹配（scan matching）** | 把当前扫描与已有地图对齐求位姿；相关匹配（correlative，粗）+ 优化匹配（Ceres，精） | — |
| **分支定界（branch-and-bound）** | Cartographer 找回环的搜索法：在多层分辨率上剪枝，兼顾速度与完备性 | — |
| **回环（loop closure）** | 识别"我来过这里"并加约束，用于消除累积误差。**没有回环 = 地图会缓慢弯折、闭环处双墙** | `issues_and_findings.md` §10.1 |
| **优化粒度（optimization granularity）** | 后端优化作用在什么单元上：逐帧（scan node）还是**分块（submap）**。粒度越粗，优化后更新地图越便宜（只挪块），但块内误差越修不了；粒度越细，地图更新越贵 | `glossary.md` 子图条 |
| **位姿图（pose graph）** | 节点=历史位姿、边=相对约束（含回环）；优化它就能全局修正地图。slam_toolbox 的 `.posegraph` 就是它 | — |
| **子图（submap）** | Cartographer 的地图基本拼块：一小块局部占据栅格，由 N 帧累积后**冻结**（块内位姿不再变）。**它的核心价值是"优化粒度"，不是省内存**：① 扫描匹配只需在活跃子图（2~3 块）上做 → 算力与地图大小无关；② 回环优化后只需把整块**刚性平移/旋转**，不必重绘栅格 → 全局修正传播廉价。顺带带来 trim（丢弃旧块）/ 增量加载 / 纯定位只留 N 块的能力。代价：块内一旦冻结，块内局部误差就修不了（`num_range_data` 是粒度权衡旋钮） | 见上文问答；`rm_algorithm_catalog.md` §〇.0 |
| **重定位（relocalization）** | 在**已有先验地图**里找回自己的位姿 → 输出 `map→odom`。与"里程计"是两层 | `architecture.md` §3.2.1 |
| **全局定位 / kidnapped robot** | 完全不知道初始位姿时找回位置；AMCL 用全域撒粒子，3D 需**全局描述子检索**（Scan Context 等）后再 ICP 精配准 | `architecture.md` §3.2.6 |
| **漂移（drift）** | 误差随距离/时间**累积**的系统性偏移；与"随机抖动"是两回事 | `smoke_test_runbook.md` §10.1 |
| **退化（degeneracy）** | 环境几何信息不足（长走廊、空旷、大平面）导致某些方向不可观，里程计沿该方向漂 | `issues_and_findings.md` |
| **外参 / 时间同步** | IMU↔LiDAR 的相对位姿 / 时间偏移；真机头号杀手（仿真里不存在） | `issues_and_findings.md` §五 |

## 二、地图与表示

| 术语 | 一句话定义 | 详见 |
|---|---|---|
| **占据栅格（occupancy grid）** | 2D 格子表，`nav_msgs/OccupancyGrid`。map_server/SLAM 用 **-1/0~100**（未知/自由/占据概率），costmap 用 **0~255**（见下） | `architecture.md` §3.2.5 |
| **代价地图（costmap）** | nav2 的 2D 代价栅格：`0`自由、`1~252`膨胀代价、`253`内切不可通行、`254`致命、`255`未知 | `architecture.md` §3.2.5 |
| **膨胀（inflation）** | 由障碍向外生成代价梯度：`252·exp(−scale·(d−r_in))`，让"贴墙走更贵" | `architecture.md` §3.2.5 |
| **代价图层（costmap layer）** | nav2 costmap 的可插拔组件（static/obstacle/voxel/STVL/inflation/filter），多层按 `combination_method`（max/override）合并 | `architecture.md` §3.2.4 |
| **体素（voxel）/ 体素栅格** | 3D 的"像素"；每格存占据/空闲/未知（常带概率、时间戳）。内存 ∝ n³，故实际用稀疏结构 | `architecture.md` §3.2.2 |
| **八叉树（OctoMap）** | 体素的自适应稀疏表示：大块空处用一个大节点，只在有物处细分 | `architecture.md` §3.2.2 |
| **ESDF / TSDF** | 距离场：每体素存"到最近障碍的距离"→ "别撞墙"变成**可微罚项**，供轨迹优化使用 | `architecture.md` §3.2.3 |
| **高程图（elevation map）** | 2.5D 表示：每格存高度/坡度/粗糙度，索引仍是 2D → 可判可通行性 | `architecture.md` §3.2.2 |
| **可通行性（traversability）** | "这块地能不能过"的代价（坡度、台阶高度、粗糙度），区别于"有没有障碍" | `architecture.md` §3.2.7 |
| **净空图（clearance / passable height）** | 每格记"地面到最低天花板的高度"，低于车高即不可通行 —— **解决隧道/悬垂的唯一办法** | `architecture.md` §3.2.7 |
| **滚动窗口（rolling window）** | 局部代价地图只保留机器人周围一块（我们 5×5 m），移出即忘 | `architecture.md` §3.2.5 |
| **时间维 / 衰减（decay）** | 表示里带"何时观测、何时过期"：STVL 的 `voxel_decay`、raytrace 清除。**与"刷新率"是两件事** | `architecture.md` §3.2.6 |
| **降维 / 3D→2D** | 把 3D 数据变成 2D 表示。本工程发生在三处（`p2l`、STVL、离线投影）+ 感知域的地面分割 | `3d_to_2d_survey.md` |
| **2D / 2.5D / 3D** | 2D=单层栅格；2.5D=2D 索引+每格高度/坡度/多层体素；3D=体素/八叉树 | `architecture.md` §3.2.2 |
| **高度带（band-pass）** | "哪个高度区间算障碍"的判定（`min/max_obstacle_height`、`min_z/max_z`、`min/max_height`） | `3d_to_2d_survey.md` §二 |
| **地面分割（ground segmentation）** | 在 3D 里把地面从障碍中剔除（我们 `linefit`：高度+坡度模型）→ 2D 图上的"自由"其实是"可行驶地面" | `3d_to_2d_survey.md` §一 |
| **射线清除 vs 洪泛** | 射线清除=沿射线把障碍前的格子标"空"（可信 free）；洪泛=从起点连通填充（近似，会外漏）。**这是自由空间质量的分水岭** | `3d_to_2d_survey.md` §二 |
| **占据概率 / log-odds** | 用概率累积多帧观测再阈值化，比"单点命中即占据"抗噪 | `3d_to_2d_survey.md` §二 |

## 三、导航与规划

| 术语 | 一句话定义 | 详见 |
|---|---|---|
| **配置空间 / SE(2)** | 地面机器人的自由度就是 (x, y, yaw)；**规划维数由配置空间决定，不由传感器决定** | `architecture.md` §3.2.2 |
| **全局规划器** | 插件基类 `nav2_core::GlobalPlanner`；输入 costmap+起点+终点，输出**整条 `nav_msgs/Path`**。本工程固定 NavFn | `architecture.md` §3.2.6 |
| **局部规划器 / 控制器** | 插件基类 `nav2_core::Controller`；输入路径+局部 costmap，输出**速度指令 `Twist`**。成员：RPP / DWB / TEB / MPPI | `architecture.md` §3.2.6 |
| **NavFn / Smac / RPP / DWB / TEB / MPPI** | Dijkstra-A* 栅格规划 / 状态格规划（含 Hybrid 支持运动学）/ 纯跟踪 / 采样评价 / 时间弹性带优化 / 现代采样式 | `algorithm_matrix.md` §一 |
| **lattice / 运动原语** | 先把控制量离散成"动力学可行的短轨迹"，再在轨迹图上搜索 → 3D 路径才真的可执行 | `architecture.md` §3.2.3 |
| **行为树（BT）/ 恢复行为** | nav2 的任务编排；失败时轮转 清代价地图 → Spin → Wait → BackUp。`/cmd_vel` 长期 `-0.05` 就是它在 BackUp | `smoke_test_runbook.md` §10 |
| **生命周期节点（lifecycle node）** | unconfigured→inactive→active 状态机，由 `lifecycle_manager` 统一驱动；**同名 manager 会冲突** | `issues_and_findings.md` §一 |
| **QoS** | ROS2 通信质量：`reliability`（reliable/best_effort）、`durability`（volatile/transient_local）。**订阅要求强于发布即不兼容** | `issues_and_findings.md` §一 |
| **TF 帧树 / 单父边** | 每个坐标系只能有一个父；`map→odom` 与 `/map` 同一时刻只能有一个发布者 → 这是"形态/槽位互斥"的根因 | `algorithm_matrix.md` §一.2 |
| **未知 / `allow_unknown`** | 未观测区域；能不能走由规划器的 `allow_unknown` 决定（我们设 true → 灰区可走） | `architecture.md` §3.2.5 |

## 四、本工程专有

| 术语 | 一句话定义 | 详见 |
|---|---|---|
| **三种场景形态（`mode`）** | `mapping` 纯建图 / `slam_nav` 边建图边导航 / `nav` 先建图后导航；区别**只在"地图从哪来"与"是否跑导航栈"** | `architecture.md` §九、`smoke_test_runbook.md` §0.7 |
| **角色槽位** | `lio` 里程计 / `mapper` 在线建图 / `localization` 重定位 / `nav` 局部规划 / `global_obstacle` 全局障碍来源 / `spin_speed` / `world` | `algorithm_matrix.md` §一 |
| **小陀螺 / `fake_vel_transform` / `base_link_fake`** | 哨兵机制：nav2 在"云台系"`base_link_fake` 里规划，角速度非零时底盘按 `spin_speed` 原地自转；**仿真里没有云台补偿，雷达会跟着转** | `architecture.md` §0.4.1、`smoke_test_runbook.md` §0.4.1 |
| **出生点系 vs 世界系** | 地图坐标约定：RMUC/RMUL 的 pgm 原点=机器人出生点（AMCL 初值 `(0,0)`）；RMUL2026 的 pgm 是世界系（初值 `(4.3,3.35)`） | `smoke_test_runbook.md` §0.5 |
| **幽灵墙 / 幽灵障碍** | 地图上存在但真实世界没有的结构（RMUL2026 的 x≈5.2 虚线）；会导致规划永久失败。也指实时的"旧标记没被清除" | `issues_and_findings.md` §三 |
| **资产配对** | `.pgm+.yaml`→AMCL；`.posegraph`→slam_toolbox 纯定位；`.pcd`→ICP；`.pbstream`→cartographer 纯定位 | `algorithm_matrix.md` §三 |
| **三处"跨维度桥"** | `pointcloud_to_laserscan`（在线 3D→2D）、STVL（3D 体素→2D 代价）、`tools/pcd_to_grid_map.py`（离线 3D→2D） | `algorithm_matrix.md` §一.1 |
