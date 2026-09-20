# HzMi_rmsimulation 目录架构说明

> 本文档描述仓库**当前实际结构**（2026-09 参数回归 R1 完成后的状态），供团队日常查阅与新成员上手。
> 相关文档：`docs/rm_bench_refactor_plan.md`（改造路线）、`docs/params_ownership_checklist.md`（参数归属清单）、`docs/rm_algorithm_catalog.md`（算法选型）。

---

## 一、项目定位

- **ROS2 humble 的多算法仿真实验台**：Gazebo 场地 + 旋转 Mid360 仿真 + FAST-LIO/Point-LIO 定位 + Nav2 导航，用于**测试/对比不同职能的算法或成品包**；
- 传感器构型与真车一致（旋转 Mid360 + 麦轮全向），仿真结论可迁移；
- **不追求 sim2real 一键打通**：真车配置冻结，真车部署在"按包清单导出"阶段另行处理。

---

## 二、顶层目录总览

| 路径 | 作用 | 是否参与 colcon 构建 |
|---|---|---|
| `src/` | **全部 ROS2 功能包**（19 个，按角色域分组） | ✅ |
| `third_party/` | **官方原版第三方库参考**（只读对照，不编译） | ❌（`COLCON_IGNORE`） |
| `tools/` | 工具脚本：`tools/*.py`＝Python 工具；`tools/scripts/`＝Shell 脚本 | ❌（无 package） |
| `docs/` | 规划/清单/操作指南文档 | ❌ |
| `archive/` | 一次性报告归档 + `reality_frozen/`（真车配置冻结快照） | ❌ |
| `build/` `install/` `log/` | colcon 构建产物（已 gitignore） | — |
| `dockerfile`、`.devcontainer/` | 容器化开发环境 | — |
| `.docs/` | README 引用的图片/GIF | — |
| `.vscode/`、`.venv/` | 本地 IDE / Python 工具环境（已 gitignore） | — |
| `.gitmodules` | 9 个子模块登记表 | — |

---

## 三、`src/` —— 按"角色域"分组的包

### 3.1 域 ↔ 包 ↔ 角色速查

| 域目录 | 包（ROS 包名） | 角色 | 源码形态 |
|---|---|---|---|
| `rm_simulation/` | `hzmi_rm_simulation` | Gazebo 场地（RMUC/RMUL/RMUL2026）+ 哨兵机器人模型 | 本仓直接管理 |
| | `ros2_livox_simulation`（目录名 `livox_laser_simulation_RO2`） | Gazebo 里仿真 Mid360，发 `/livox/lidar` | 本仓直接管理 |
| `rm_driver/` | `livox_ros_driver2` | 真车雷达驱动 + **CustomMsg 消息定义**（仿真不启动节点，但编译期必需） | git 子模块 |
| `rm_perception/` | `linefit_ground_segmentation` / `linefit_ground_segmentation_ros` | 点云地面分割（`/segmentation/obstacle`） | 本仓直接管理 |
| | `pointcloud_to_laserscan` | 3D 障碍点云 → 2D `/scan`（**降维，2D 世界唯一入口**） | 本仓直接管理 |
| | `imu_complementary_filter` | IMU 滤波 → `/imu/data`（喂 FAST-LIO） | 本仓直接管理 |
| `rm_localization/` | `fast_lio` | 里程计 + 3D 建图（一体） | **git 子模块（自有 fork）** |
| | `point_lio` | 高频里程计 + 建图（`fast_lio` 的替身） | **git 子模块（自有 fork）** |
| | `icp_registration` | 3D 重定位（吃 `.pcd`），发布 `map→odom` | 本仓直接管理 |
| | `lio_tf_adapter` | **LIO 位姿 → 标准帧树适配**（`/odom` ⇒ `odom→base_link`，T3） | 本仓直接管理 |
| | `slam_toolbox` | 2D 建图 / 纯定位（官方源码 vendored，**编译覆盖 apt**） | 内嵌源码（去 .git） |
| | `cartographer_ros` | 2D 建图 / 纯定位（**ros2-gbp humble 官方 ament 源码**，编译覆盖 apt） | 内嵌源码（去 .git） |
| `rm_navigation/` | `rm_navigation` | Nav2 组装（launch + **nav2 参数** + rviz） | 本仓直接管理 |
| | `teb_local_planner` / `teb_msgs` | TEB 局部规划器（Nav2 插件） | git 子模块 |
| | `costmap_converter` / `costmap_converter_msgs` | TEB 依赖（costmap→几何图形） | git 子模块 |
| | `fake_vel_transform` | 云台旋转速度补偿胶水 | 本仓直接管理 |
| `rm_nav_bringup/` | `rm_nav_bringup` | **总装层**：launch 入口 + 地图/PCD/rviz/urdf 资产（**无 config/**，参数已全部回归各包） | 本仓直接管理 |

> **维度（2D/2.5D/3D）标注与算法组合现状不在这张表里**，统一维护在 `docs/algorithm_matrix.md`（§一.1 维度归类、§三 资产、§四 实测状态）；本表只管「哪个域有哪些包」。
>
> 19 个 colcon 包 = 上表除子模块内嵌包外的全部；`slam_toolbox/lib/karto_sdk` 由 slam_toolbox 自带、不单独出现在 colcon 列表。

### 3.2 数据流（仿真，默认 mapping/nav 组合）

```
Gazebo(hzmi_rm_simulation 世界 + 机器人)
   ├─ 旋转 Mid360 仿真插件(ros2_livox_simulation) ─► /livox/lidar (CustomMsg) ─┬─► fast_lio / point_lio ─► 里程计位姿 + 3D 图
   └─ IMU 插件 ─► /livox/imu ─► imu_complementary_filter ─► /imu/data ────────┘
                                                          │
                        /livox/lidar/pointcloud ─► linefit 地面分割 ─► /segmentation/obstacle
                                                          │
                                          pointcloud_to_laserscan ─► /scan
                                                          │
            ┌─────────────────────────────────────────────┴──────────────┐
   2D 建图/定位: slam_toolbox / cartographer_ros            2D 导航: Nav2(rm_navigation)
            └─────────────────────────────────────────────┬──────────────┘
                                          fake_vel_transform ─► /cmd_vel ─► 底盘
```

### 3.2.1 三层职责：`lio` / `localization` / `mapper` 各管一段

**`lio` 管"相对起点走了多少"（局部、连续）→ `localization` 管"我在场地里的哪个位置"（全局、不漂）→ `mapper` 管"把场地地图造出来"。**
三者恰好对应 TF 树上三条不同的边（nav2 只用合成的 `map→base_link`，但这条链必须由两层各出一段）：

```
map ──[localization（已知地图）或 在线 mapper（边建边用）]──► odom ──[lio]──► base_link ──[fake_vel_transform]──► base_link_fake ──► livox_frame
     全局：不漂、低频修正                                          局部：连续、高频、会漂
```

| 层 | 参数 | 发布哪条边 | 输入 | 输出 | 特性 | 取值 |
|---|---|---|---|---|---|---|
| **里程计** | `lio` | `odom → base_link` | `/livox/lidar`(CustomMsg 10Hz) + `/imu/data`(100Hz) | `/odom`（≈10Hz）→ 由 `lio_tf_adapter` 转成 TF | **高频连续**；无全局参考、**有累积漂移**；只回答"相对起点" | `fastlio` / `pointlio` / `none` |
| **重定位** | `localization` | `map → odom` | **磁盘地图资产** + `/scan`(或点云) + `odom→base_link` | `map→odom` TF（amcl 另有 `/amcl_pose`） | **低频修正、全局不漂**；amcl/icp 需要初值 | `amcl` / `slam_toolbox`(localization 模式) / `icp`；留空 = 回退（LIO 当绝对定位 + 静态桥补帧） |
| **建图** | `mapper` | `/map`（在线建图时**同时**发 `map→odom`） | `odom→base_link` + `/scan`（或点云） | `/map`(OccupancyGrid)；离线落盘 `.pgm/.yaml` | **造地图**；含回环优化（`map→odom` 会跳变） | `slam_toolbox`(online_async) / `cartographer` |

最容易混的三点：

1. **同一条 `map→odom` 边，`localization` 和"在线 `mapper`"都会发**：前者是"用已知地图去对齐"，后者是"边造地图边给"。
   **两者绝不能同时开**（会出现两个 `map→odom` 父边），这正是把形态拆成 `nav`（重定位）与 `slam_nav`（在线 SLAM）的根本原因（见 §九 启动集表）。
2. **`mapper` 是"生产地图"，`localization` 是"消费地图"**：`mapper` 离线产出资产，`localization` 加载资产。**资产与模块必须配对**，配错的表现是"参数传了但模块不工作"。
3. **`lio` 不依赖任何地图资产**，三种形态、所有场地都能跑；`localization` 强依赖磁盘资产；`mapper` 什么都不依赖（从零开始）。

**地图资产 ↔ 生产者 ↔ 消费者**（`rm_nav_bringup/map/`、`PCD/`）：

| 资产 | 生产者 | 消费者 | 备注 |
|---|---|---|---|
| `.pgm` + `.yaml` | `mapper`（或 `map_saver_cli`） | `localization:=amcl`（经 `map_server`） | 纯 2D 栅格 |
| `.posegraph` | `mapper:=slam_toolbox`（`/slam_toolbox/serialize_map`） | `localization:=slam_toolbox` | 带位姿图，可续建/精定位 |
| `.pcd` | LIO（`/map_save` 服务，路径 `PCD/<world>.pcd` 由 bringup 注入） | `localization:=icp` | 3D 点云配准 |
| `.pbstream` | `mapper:=cartographer` | cartographer 纯定位模式 | 含子图+位姿图 |

**生效条件**：`mapper` 仅 `mapping` / `slam_nav`；`localization` 仅 `nav`；`lio` 三种形态都生效（`none` = 由外部提供 odom/TF）。

> **补充：LIO 不只能做定位。** FAST-LIO/Point-LIO 是 LiDAR-Inertial SLAM，跑里程计的同时就在建 3D 点云地图
> （`PCD/<world>.pcd`，RMUL 为 159 万点）——本工程已把它当资产用（ICP 底图），另外可用
> `tools/pcd_to_grid_map.py` 切层投影成 `.pgm/.yaml` 直接喂 AMCL，于是存在第三条 2D 地图来源
> （① mapper 在线 SLAM ② LIO 点云投影 ③ 外部/官方平面图）。实测 RMUL.pcd 投影图与 slam_toolbox 的
> `map/RMUL.pgm` 在同一坐标系、±1 格容差下参考图墙覆盖率 84.7%。

### 3.2.2 2D / 2.5D / 3D：降维到底发生在哪一层

**先厘清一个常被混淆的点**：地面机器人的**配置空间本身就是 SE(2)**（x, y, yaw）——高度/俯仰/横滚被地面约束。
所以"导航都用 2D"不是偷懒，而是规划维度与机器人自由度同维；被降维的是**传感器与环境的表示**，不是机器人状态。

本工程里降维发生**两次**，而 3D 信息**在两处被保留**：

| 环节 | 做什么 | 3D 是否保留 | 本工程的位置/参数 |
|---|---|---|---|
| ① 传感器端 | 3D 点云 → 压成**一个高度切片**的 `/scan` | ❌ 只留一层 | `pointcloud_to_laserscan`：`min_height: -1.0`、`max_height: 0.1`（相对 `livox_frame`，该帧在车顶 ≈0.275 m）→ `/scan` 供 AMCL/slam_toolbox/local_costmap |
| ② 地面分割 | 在 3D 里**剔除地面**，留下障碍点云 | ✅ 保留 3D | `linefit_ground_segmentation`：`sensor_height: 0.275`、坡度 ±0.4、`max_dist_to_line: 0.1` → `/segmentation/obstacle` |
| ③ 代价地图 | **3D 体素栅格**按高度带筛选后**投影成 2D costmap**（取 max） | ✅ 体素在层内保留 | `global_costmap.stvl_layer`：`voxel_size: 0.05`、`voxel_decay: 0.5`(s，线性衰减)、`min_obstacle_height: 0.2`、`max_obstacle_height: 2.0`、`publish_voxel_map: true`（RViz 可直接看 3D 体素） |
| ④ 全局 3D 地图 | LIO 的 3D 点云地图落盘 | ✅ 全保留 | `PCD/<world>.pcd`（`/map_save`） |

**"伪 2D / 2.5D"指的就是 ③ 这种做法**：索引仍是 2D 栅格，但每格背后堆着体素/高度信息，投影时才做取舍。
三条技术路线：

| 路线 | 环境表示 | 规划器 | 能表达 / 丢失 | 典型场景 |
|---|---|---|---|---|
| **2D 平面** | 单层 `OccupancyGrid` | NavFn/A*/RPP/DWB（SE(2)） | 丢失"多高"：低矮可跨越结构会误判为障碍，悬垂可穿过结构也误判 | 平地 + 竖直墙（**RM 哨兵正是这类**） |
| **2.5D**（伪 2D） | 2D 索引 + 每格高度/坡度/多层体素（`grid_map`/`elevation_mapping`/STVL） | 吃可通行性的规划器 | 保留"多高/多陡"→ 可判可跨越性；仍不能表达真悬垂/桥下 | 越野、四足、坡道 |
| **真 3D** | 体素/八叉树（OctoMap） | 3D lattice / OMPL | 全部几何；但算力高一个量级、需 3D 碰撞模型 | 无人机、机械臂、立体机动 |

**什么时候"降维"会真出错**（也是调参判据）：

1. **低矮但可跨越**（飞坡边缘、路沿）→ 2D 会把它们标成障碍，机器人绕路甚至无路可走。
   本工程靠 ①③ 的高度带滤掉：`min_obstacle_height: 0.2`（低于 0.2 m 不算障碍）。
   实验：把它调到 `0.4`，看 0.2~0.4 m 的低矮结构是否被忽略。
2. **悬垂/桥下可穿过** → 2D 投影必然误判；只能靠 2.5D 分层或真 3D。
3. **坡道/起伏地形** → 2D 无法表达坡度，需要高程图（2.5D）。
4. **瞬时障碍（人/车）** → 靠 STVL 的 `voxel_decay`（0.5 s 线性衰减）自动清除过期占据。

**"能不能不降维？"** 可以，三条可落地路径（按改动量排序）：

1. **只把 3D 用在避障、规划仍 2D**（最小改动）：保留 STVL 体素，自己加 3D 碰撞检查
   （注意 nav2 的 `use_collision_detection` 是 2D 的，不能直接提供 3D 语义）；
2. **上 2.5D 高程图**：用 `grid_map`/`elevation_mapping` 产出带高度/坡度的 2D 图，替换 costmap 的
   static/obstacle 层，规划器改用吃可通行性的实现 —— 这也是本工程"以后要加的 2.5D 角色槽位"的正路；
3. **真 3D**：OctoMap + 3D lattice 规划器。对平整场地 + 竖直矮墙的哨兵场景，性价比低。

> 结论：**降维不是缺陷，而是"与配置空间同维"的必然选择**；关键是高度带（`min/max_obstacle_height`、
> `pointcloud_to_laserscan` 的 `min/max_height`）要按机器人实际能跨越的高度来定，必要时把 2D 升级成 2.5D。

### 3.2.3 "3D 到底怎么用"——四种用法 + 真 3D 规划器内部机制

"用 3D"这句话至少对应四种截然不同的东西，先分清再谈值不值得：

| # | 用法 | 3D 用在哪 | 决策维数 | 例子 |
|---|---|---|---|---|
| **A** | **3D 感知 → 2D 决策** | 用 3D 点云算"哪些格不能走"，决策仍是 2D | SE(2) | **本工程现状**：地面分割 + STVL 体素 → 2D costmap；95% 的地面机器人 |
| **B** | **3D 表示 → 2.5D 决策** | 3D 压成"每格高度/坡度/粗糙度"，索引仍是 2D | SE(2) + 可通行性 | `grid_map`/`elevation_mapping`（ANYbotics）、越野车、四足 |
| **C** | **真 3D 决策（位置）** | 状态 = (x,y,z)，在 3D 体素/连续空间里搜路径 | SE(3) 位置部分 | 无人机、水下、桥下穿越 |
| **D** | **真 3D 决策（含姿态/接触）** | 状态含姿态/速度/加速度/接触序列 | SE(3)+ 或更高维 | 无人机高速飞行的动力学轨迹、腿足落脚点+接触序列、机械臂（关节空间 n 维） |

#### 真 3D 规划器内部是怎么工作的（以无人机为主线）

```
① 表示      3D 点云 → OctoMap(八叉树/概率占据·多分辨率) 或 体素栅格 或 ESDF(欧氏符号距离场)
② 状态空间  (x,y,z) [无人机再加 yaw；高速飞行用微分平坦 4 维：x,y,z,yaw]
③ 搜索/优化 体素 A*(26 邻域) | lattice(离散运动原语, 满足动力学) | RRT*/BIT*(连续采样) | 轨迹优化(CHOMP/MINCO/EGO，多项式+QP)
④ 碰撞检测  球/盒 vs 体素；或直接查 ESDF 的距离与梯度（优化里做罚项，这是 3D 最典型的"用法"）
⑤ 跟踪控制  3D 路径 → 位置/速度指令 → 姿态环（飞控）
```

几个关键机制的理解要点：

- **ESDF 是"3D 怎么用"的核心技巧**：每个体素存"到最近障碍的距离"，于是"别撞墙"变成一个**可微的罚项**（往梯度方向推），
  轨迹优化才能直接在这个场里下降。代表：`voxblox`（增量 TSDF/ESDF）、`nvblox`（GPU）。
- **lattice / 运动原语**：先把控制量离散成一段段**动力学可行**的短轨迹，再在这些轨迹构成的图上搜 ——
  这样搜出来的 3D 路径才是机器人真的能飞的，而不是折线。
- **体素数决定算力**：20×20×5 m 空间，0.1 m 体素 = 200×200×50 = **200 万格**（3D A* 勉强）；
  0.05 m → **1600 万格**（很重）。这就是"3D 搜索贵"的具体来源——3D 是 n³ 增长，2D 是 n²。
- **腿足的做法**：全局仍走 2.5D 高程图（B），但**落脚点选择**是 3D 的（在高程图里挑一块够平的接触面 → 形成接触序列），
  属 D 类，只是"3D"体现在接触而不是整条路径。
- **机械臂/移动操作**：OMPL 在关节空间（n 维）规划，本质上与 A–D 无关，是"高维而不是 3D"。

#### 为什么地面机器人即使有 3D 数据也常不升到 C/D

1. **规划维数由配置空间决定，不由传感器决定**：轮式底盘能做的事就是 SE(2)，3D 路径里"抬高 0.3 m"这种动作它做不到；
2. 3D 搜索空间 n³ 膨胀，而收益只在"高度决定可通行性/机动性"时才出现；
3. 地面约束使 z/roll/pitch 基本确定，把自由度花在它们上是浪费；
4. 3D 地图的不确定性更大（点云稀疏、动态物体、遮挡），2D 投影反而更稳。

#### 那本工程/哨兵场景里"3D 真正有用的地方"在哪

| 场景 | 该用哪种 | 说明 |
|---|---|---|
| 平地 + 竖直矮墙（现在的 RM 场地） | A（+轻量 B） | 只要高度阈值选对，2D 就是最优 |
| 低矮可跨越 / 悬垂可穿 / 坡道 | B（2.5D 高程图） | 高度决定可通行性 |
| **云台瞄准规划**（哨兵真正需要 3D 的地方） | **D（2–3 自由度，臂式）** | 让"枪口对准目标"而不是"底盘对准路径"，这在数学上是 2–3 DOF 规划问题；`base_link_fake` 机制本质上是把这个问题**绕过**了——用"底盘小陀螺 + 假云台系"让 2D 规划器与云台共存，而不是真去规划云台 |
| 若将来加飞行/爬坡机器人 | C/D | 才需要真 3D 规划器 |

> 所以"3D 怎么用"的实用答案是：**先把 3D 用在"能不能走"（A/B）上，只有当"高度/姿态本身就是任务"（飞行、攀爬、瞄准）时，才升级到 C/D。**
> 本工程若要开这条研究线，最自然的入口是给 costmap 加一个**高程/分层层**（B），而不是直接上 3D 规划器。

### 3.2.4 感知与决策的接口：谁吃什么、以什么形式吃

**一句话**：**规划器吃的是"表示"，不是数据。** nav2 的规划/控制插件只拿到 **2D costmap**（`OccupancyGrid`），
它根本看不到点云；点云在各种"感知/建图层"里被加工成 costmap，才进入决策。

#### 本工程真实的消费树（注意：原始数据是**多路并行消费**的）

```
/livox/lidar (CustomMsg 10Hz) ─► fast_lio / point_lio ─┬─► /odom(≈10Hz) ─► lio_tf_adapter ─► TF odom→base_link
                                                       └─► /cloud_registered(3D 地图) ─► Rviz；/map_save ─► PCD/<world>.pcd
/livox/imu (100Hz) ─► imu_complementary_filter ─► /imu/data ─┘（FAST-LIO 的 imu_topic；point_lio 直接用 /livox/imu）

/livox/lidar/pointcloud (PointCloud2 10Hz) ─┬─► linefit 地面分割 ─► /segmentation/obstacle（3D 障碍点云）
                                            │        ├─► pointcloud_to_laserscan（切高度带 z∈[-1.0, 0.1]）─► /scan（2D）
                                            │        │        ├─► local_costmap.obstacle_layer
                                            │        │        ├─► amcl（仅 mode:=nav + localization:=amcl）
                                            │        │        └─► Rviz
                                            │        └─► global_costmap.stvl_layer（3D 体素，高度带 0.2~2.0m）
                                            └─► icp_registration（仅 localization:=icp）─► TF map→odom

汇聚点（唯一的"统一表示"）：costmap 2D = static_layer + stvl/obstacle + inflation 在该节点内合并
        ├─► planner_server(NavFn) ─► /plan
        ├─► controller_server(RPP/DWB/TEB) ─► /cmd_vel ─► fake_vel_transform ─► /cmd_vel_chassis ─► Gazebo
        └─► behavior_server(spin/backup/drive_on_heading/wait)
定位合成：map→odom（amcl / icp / slamTB-loc / 在线 SLAM） × odom→base_link（lio_tf_adapter） = map→base_link
```

**结论：感知侧是"分别处理"（同一份点云被 3~4 个节点各自加工成各自的表示），只有 costmap 是"汇合后的统一表示"。**

#### 抽象阶梯：每上一层，丢信息、换可规划性

| 层 | 表示 | 谁产出 | 谁消费 | 丢掉了什么 |
|---|---|---|---|---|
| 原始 | `PointCloud2`（3 万点/帧 @10Hz） | 雷达插件 | LIO / 地面分割 / ICP / Rviz | — |
| 3D 障碍 | 去地面的障碍点云 | `linefit` | p2l / STVL | 地面、强度、时间 |
| 2D 切片 | `LaserScan` | `pointcloud_to_laserscan` | AMCL / local costmap | **高度**、3D 形状 |
| 体素 | 3D 占据 + 时间衰减 | `stvl_layer` | （投影给 costmap） | 精确形状（离散化） |
| **2D 代价栅格** | `OccupancyGrid`(int8 0~255) | costmap | **planner / controller / behavior** | 高度、速度、语义 |
| 路径 | `nav_msgs/Path` | planner | controller | 全局路径不含时间/动力学 |
| 指令 | `Twist` | controller | 底盘 | — |

#### nav2 的"2D 契约"有多硬

| 插件接口 | 入参 | 所以 |
|---|---|---|
| `nav2_core::GlobalPlanner` / `Controller` / `Behavior` | `Costmap2DROS`（2D costmap）+ start/goal | **只能 2D**；想喂 3D 只能通过**图层**（STVL/高程层）把 3D 信息"投影/筛选"进 costmap（=2.5D） |
| 3D 规划器（OMPL、3D lattice/SBPL、无人机栈 EGO-Planner 等） | 3D 体素 / ESDF / 连续空间 | 它们是**另一套决策子系统**，不共享 nav2 的 costmap/BT |

所以准确的判断是：**"决策层能不能吃 3D"取决于你选了哪套决策层**——nav2 = 2D 契约；3D 规划器 = 3D 契约；
想让 nav2 用上 3D，只有两条路：① 在 costmap 上游加 3D/2.5D **图层**（推荐，改动小）；② 换掉整条规划子系统（改动大）。

#### 多路并行处理的代价：**一致性风险**（本工程就有实例）

- `local_costmap.obstacle_layer` 吃的是 `/scan`，高度带 **z∈[-1.0, +0.1] m（相对车顶 livox_frame）**；
- `global_costmap.stvl_layer` 吃的是 `/segmentation/obstacle`，高度带 **z∈[0.2, 2.0] m**；
- 两者**准入门槛不同** → 一个 0.1 m 高的矮台：local 切片看得见（会绕），global 视而不见（会直接规划穿过去）。
  这类"不同消费者看到的世界不一样"是分别处理架构的固有代价，调参时要**成对校准高度带**。
> **2026-09 起这个来源可切换**：`global_obstacle:=stvl`（默认，3D 体素层）/ `scan`（global 与 local 同源，
> 都吃 `/scan`，"什么算障碍"只在感知域 `p2l` 决策一处，即消除上述不一致）/ `none`（只 static+inflation）。

**什么时候该换成"统一世界模型"**：当消费者变多（导航 + 瞄准 + 决策 + 学习型模块）、或需要跨模块一致性保证时，
应引入单一环境表示节点（2D costmap / 3D ESDF / 语义+可通行性图），所有下游只读这一份。
本工程目前是"半统一"：**对导航而言 costmap 是统一表示**，但 costmap 之外（AMCL、ICP、LIO、Rviz）各自又直接吃点云/scan。

### 3.2.5 一张点云被切成很多份 / 2D 栅格为何能承载 3D / 两张 costmap 的分工

#### ① 一份点云被切成多份，是**需求不同**而不是浪费

| 消费者 | 吃哪一路 | 为什么必须是这一种形式 |
|---|---|---|
| FAST-LIO / Point-LIO | `/livox/lidar`（**CustomMsg**） | 需要**逐点时间戳**做运动去畸变 + 与 100Hz IMU 同步 |
| 地面分割 | `/livox/lidar/pointcloud`（PointCloud2） | 需要完整 3D 坐标做地面拟合 |
| `pointcloud_to_laserscan` → `/scan` | `/segmentation/obstacle` | AMCL/local costmap 要 **2D + 高频 + 轻量** |
| STVL | `/segmentation/obstacle` | 要 **3D + 时间衰减**（体素） |
| ICP 重定位 | `/livox/lidar/pointcloud` | 要**原始精度**做配准（不能被切层/降采样） |
| Rviz | 各路 | 人看 |

> 同一个点云被多路并行加工，是"各自最合适"的工程折中；代价是重复计算与**一致性风险**（见 §3.2.4 的高度带实例）。

#### ② 2D 栅格能承载 3D 信息，中间"隐藏"了这五个机制

1. **高度准入（band-pass）——最关键**：障碍判定发生在**图层内部**，2D 图只承载**结论**。
   `min/max_obstacle_height` 本质是"机器人碰撞体的高度范围"：本工程 global 取 **0.2~2.0 m**
   （低于 0.2 m 可跨越 → 不算障碍；高于 2.0 m 忽略），local 的 `/scan` 是**相对车顶 z∈[-1.0, +0.1] m 的一片切片**。
2. **地面分割**：先在 3D 里把"地面"从"障碍"中剔除（`linefit`：`sensor_height 0.275`、坡度 ±0.4）。
   所以 2D 图上"自由"的语义其实是"**可行驶地面**"，而不是"没有点"。
3. **体素→最大值投影（STVL）**：3D 体素保留在层内部（`voxel_size 0.05`、`voxel_decay 0.5 s`），
   投影到 2D 时取该列 max → "这一列任意高度有障碍，则这一格贵"。于是 2D 图实际是 **(x,y,z,t) 的有损投影**。
4. **代价而非布尔 + 膨胀层**：栅格是 0~255 的**代价**，`inflation_layer` 把"离障碍的距离"编码成代价梯度
   （global `r=0.7/scale=8`，local `r=0.6/scale=5`）→ 2D 图承载了一张 **2D 距离场**（ESDF 的 2D 轻量版），
   这就是"贴着墙走会变贵"的来源。
5. **多图层合并**：`static + stvl + inflation` 在**同一个 costmap 节点内**按规则合并（STVL `combination_method: 1` = max），
   多个 3D 信息源因此统一表达在一张 2D 图上。

> 一句话：**"能不能走"这个 3D 判断被提前做掉了，2D 图只留结论 + 代价。** 所以 2D 规划器能在 3D 环境里避障，
> **各家实现对照见 `docs/3d_to_2d_survey.md`**：`pointcloud_to_laserscan`（高度带+每角度取最小）、nav2 `ObstacleLayer`（高度带+raytrace）/`VoxelLayer`（体素列计数阈值）、STVL（体素+衰减+max 投影）、**cartographer 2D 自带 `min_z/max_z` 高度带**（所以 3D 雷达能直接喂它）、octomap `projected_map`。
> 但它对"低矮可跨越 / 悬垂可穿过"这类几何的判断完全依赖上面那组高度阈值。

#### ③ global_costmap 与 local_costmap：**同一套代码，两个独立实例，目的不同**

| | `global_costmap` | `local_costmap` |
|---|---|---|
| 坐标系 | **`map`**（全局一致） | **`odom`**（局部连续） |
| 范围 | 整张地图（不 rolling） | **rolling 5×5 m**（贴着机器人） |
| 分辨率 | 0.04 m | 0.02 m（更细） |
| 更新/发布 | 5 Hz / 2 Hz | **20 Hz / 10 Hz**（更新鲜） |
| 图层 | `static + stvl + inflation` | `obstacle(/scan) + inflation` |
| 障碍来源 | `/segmentation/obstacle`，z∈[0.2, 2.0]（3D 体素） | `/scan`，相对车顶 z∈[-1.0, 0.1]（2D 切片） |
| 膨胀 | r=0.7, scale=8 | r=0.6, scale=5 |
| 谁消费 | `planner_server`(NavFn 全局找路) | `controller_server`(RPP/DWB/TEB) + `behavior_server`（`local_costmap/costmap_raw`） |
| 目的 | **找路**：全局一致、记得住远处静态障碍 | **避障跟踪**：新鲜、高频、只管身边 |

**为什么坐标系故意不同（这是最容易忽略的设计）**：

- **local 用 `odom`**：局部避障必须相对**连续里程计**做。若局部图挂 `map`，一旦重定位修正导致 `map→odom` 跳变，
  机器人周围的障碍会"瞬移"，控制器立刻做出错误的避障动作。→ 局部图只依赖 `odom→base_link`（LIO）。
- **global 用 `map`**：全局路径必须与地图墙体在同一坐标系里才能规划。→ 全局图依赖 `map→odom`（重定位模块）。

**两张图可以"看到不同的世界"**（本工程实例）：0.1 m 高的矮台，local（切片含它）会绕，global（z≥0.2 不算障碍）会直接规划穿过去。
调参时 **高度带要成对校准**，否则会出现"全局路径穿过局部认为存在的障碍"这类难查现象。

#### ④ "2.5D / 3D 有正式规划器吗"

| 层 | 正式/主流规划器与表示 | 契约 |
|---|---|---|
| 2D | **Nav2**（NavFn/Smac/RPP/DWB/TEB）、`costmap_2d` | 2D costmap + start/goal，标准插件接口 |
| 2.5D | `grid_map`(ANYbotics) + 可通行性代价、`elevation_mapping` + 腿足规划（free_gait/towr）、各家越野 traversability 规划器 | **碎片化**：多为自研"高程图 + 图搜索"，没有 nav2 那样的统一插件契约 |
| 3D | `OMPL`(RRT*/BIT*)、`SBPL`(3D lattice)、`mav_planning`/`mav_trajectory_generation`、`Fast-Planner`/`EGO-Planner`(无人机)、`OctoMap`/`voxblox`/`nvblox`(表示与 ESDF)、`MoveIt2`(机械臂，n 维) | 3D 体素/ESDF + 动力学约束，状态空间与碰撞检查都不同 |

**要点**：nav2 是**唯一有"标准 2D 契约"**的那一层；2.5D/3D 各有实现但接口不统一 —— 这也是为什么
"想让 nav2 用上 3D"最现实的做法是**在 costmap 上游加图层（2.5D）**，而不是期待 nav2 直接吃 3D。

### 3.2.6 插件协议、降维调参、算力量级、行业分层实践

#### ① 我们用了 2.5D/3D 规划器吗？—— 没有

本工程只有 **nav2 的 2D 栈**（NavFn 全局 + RPP/DWB/TEB 局部 + behavior/smoother）。
3D 只出现在**感知与表示**侧（LIO 的 PCD、STVL 体素），进入决策前都被压成了 2D costmap。

#### ② "我给 2.5D 加个协议不就行了？" —— 可以，而 nav2 的协议就是**插件（pluginlib）**

nav2 的扩展点（都是插件）：`nav2_costmap_2d::Layer` / `CostmapFilter`（keepout/speed/preferred lanes）、
`nav2_core::GlobalPlanner` / `Controller` / `Behavior` / `Smoother` / `GoalChecker` / `ProgressChecker`、
`nav2_behavior_tree::BTPlugin`。两条实现 2.5D/3D 的路子：

| 路子 | 怎么做 | 改动 | 一致性 | 适合 |
|---|---|---|---|---|
| **A. 加图层**（我推荐的"上游加图层"） | 写一个 `Layer` 插件，把 3D/高程信息**投影成 2D 代价**写进 master grid | 小（不改 planner/BT/controller） | **好**：规划、控制、行为看同一份 2D 真值 | 高度影响可通行性；想少踩坑 |
| **B. 加规划器插件** | 写 `GlobalPlanner` 插件：**接口仍是 2D**（收 `Costmap2DROS` + start/goal），但插件内部**自己订阅 3D 数据**（体素/ESDF）做 2.5D/3D 搜索，输出 `nav_msgs/Path` | 大（自己写规划器） | 差：BT 的 `IsPathValid`/清代价地图、behavior、controller 仍按 2D 走，你得自己保证两边不打架 | 真的要"坡度/高度最优"或 3D 穿越 |

两个关键事实：
- **插件是组件节点，能自己订阅任何话题** —— 所以"nav2 只能 2D"说的是**默认契约**，不是硬限制；
- **`nav_msgs/Path` 本身就是 3D 的**（`PoseStamped` 带 z 与完整姿态），卡点在**输入侧**（costmap 是 2D）和**控制/行为侧**（它们只吃 2D 局部图）。
  所以"协议留了口子，但只在输出侧"。

#### ③ 算力量级（数量级估计，用于判断"小电脑够不够"）

| 结构 | 规模 | 备注 |
|---|---|---|
| 局部 2D costmap | 5×5 m @0.02 = **6.3 万格**，20 Hz | 很轻 |
| 全局 2D costmap | 13×10 m @0.04 = **8 万格**，5 Hz | 很轻 |
| 2.5D 图层（投影） | 每更新遍历一遍上游体素/高程 | 增量式几乎免费 |
| **2.5D lattice 搜索** | 5×5 m @0.1 × 8 朝向 = **2 万状态** | 轻（这就是"2.5D 便宜"的原因） |
| STVL 体素 | 20×20×3 m @0.05 ≈ **960 万体素**（稀疏+衰减） | **nav2 里最重的图层**，常是 CPU 大头 |
| 3D lattice | 同上加 20 层高度 → **40 万状态**，×26 邻域 | 比 2.5D 重 **10~50 倍** |
| 3D ESDF（voxblox/nvblox） | 增量维护距离场 | CPU 每帧 ~10~100 ms；GPU(nvblox) 可实时 |

结论：**2.5D 图层在小电脑上是"舒服"的；真正吃力的是 3D ESDF/轨迹优化**（那才需要 GPU 或更强算力）。

#### ④ 3D→2D/2.5D 的"降维内容"要调什么

**要调，而且必须按机器人+场地调**。要调的旋钮（都在本工程现有文件里）：

| 环节 | 参数 | 决定 |
|---|---|---|
| 切片 | `pointcloud_to_laserscan.min_height/max_height` | 哪一段高度进入 `/scan` |
| 地面 | `linefit.sensor_height/min_slope/max_slope/max_dist_to_line` | 什么算"地面（可走）" |
| 体素 | `stvl.voxel_size/voxel_decay` | 分辨率与"障碍多久过期" |
| **高度准入** | `stvl.min/max_obstacle_height`、`obstacle_layer.max_obstacle_height` | **可跨越 vs 阻挡**（最关键） |
| 范围 | `obstacle_max_range/raytrace_max_range` | 看多远、能否清除 |
| 膨胀 | `inflation_radius/cost_scaling_factor`（global/local 不同） | 离墙多远的代价梯度 |
| 栅格 | `resolution/robot_radius(or footprint)/track_unknown_space` | 几何精度与未知区语义 |
| 一致性 | global 与 local 的**高度带/范围/膨胀成对** | 避免"全局路径穿过局部认为的障碍" |

**调参方法论**（本工程已具备的工具）：
1. 离线先看：`tools/pcd_to_grid_map.py --min-z/--max-z` 扫几个高度带，**秒级**看清"哪些几何在哪个高度"；
2. 由机器人碰撞体高度定 `min/max_obstacle_height`（能跨过的要排除、挡得住的要包含）；
3. 用 `tools/check_map_reachable.py` 验证降维后**可通行区没被切碎**（连通域、可达目标点）；
4. 场景用例回归：能跨的矮台/坡道**不该绕路**，挡得住的墙**必须绕**，悬垂结构单独测；
5. global/local 都跑一遍同一用例，确认两者判断一致。

#### ⑤ 行业是"公认一套"还是"混搭"？

- **分层是共识，实现是混搭**：`里程计 → 建图 → 定位/重定位 → 世界模型(代价/高程) → 规划 → 控制 → 行为` 每层可替换，
  没有哪家把全栈写成一个包；
- **nav2 是"2D 导航胶水层"的事实标准**（研究、教育、轻量 AMR 大量使用），但工业 AGV/AMR 常自研或买商业栈；
- **自动驾驶是另一套**（Autoware/Apollo：感知-预测-决策-规划-控制，规划多用 lattice/优化 + 更重的世界模型）；无人机=3D 栈；腿足=2.5D 高程；机械臂=MoveIt2；
- **LIO/SLAM 包不集成导航**：FAST-LIO/Cartographer 只做"里程计+地图(+重定位)"，**故意不做规划控制** ——
  这是正交性设计，保证任一层可替换（也正是本工程 bench 的基础）；
- 近期趋势是把"**定位 + 地图 + 世界模型**"打成模块（Autoware 的 map/localization、Isaac 的 nav2+cuVSLAM），
  但"规划/控制吃 2D 还是 3D"仍按平台分（地面 2D、越野/腿足 2.5D、空中 3D）。

> 对本工程的启示：**保持现在的分层**（lio / localization / mapper / costmap / planner / controller），
> 想加 2.5D 就先加**图层插件**；只有当你确实要做"坡度/高度最优"或"云台瞄准"这类任务时，
> 才写**规划器插件**（接口 2D、内部 2.5D/3D），并接受"要与 BT/behavior/controller 的 2D 视图保持一致"这项额外成本。

### 3.2.7 上坡/下坡/过隧道：2.5D 要做什么、放哪里

#### ① 为什么"只按高度切点云"一定不够

切高度只产出**布尔/代价**（"这一格有没有落在高度带内的点"），它能表达"挡/不挡"，**不能表达"多陡/多高/多糙"**：

| 场景 | 只用高度带的失败模式 |
|---|---|
| **上坡/下坡** | 坡面点被地面分割剔除 → 2D 图上就是"自由"，规划器**直接规划上去/横穿**，不管坡度多大、会不会翻；想"能上但别太陡"需要**代价随坡度变化**，布尔图无从表达；横穿陡坡 vs 直上直下的姿态安全差异也表达不了；下坡还要按坡度限速（涉及控制器） |
| **隧道/悬垂** | 天花板若落在高度带内 → 投影把**整条通道**标成障碍（能过的地方被自己封死）；把 `max_obstacle_height` 抬高把天花板排除 → 真正的墙也一起漏掉。**阈值不可能两边都对** |

隧道/悬垂的唯一出路是**净空图（clearance / passable height）**：每格记"地面到最低天花板的高度"，低于车高即致命。
所以 2.5D 的两种典型产物是：**高程+坡度+粗糙度图**（坡道）与**净空图**（隧道/悬垂）。

#### ② 什么时候"加图层就够"，什么时候"必须换规划器"

| 需求 | 表示要有 | nav2 加图层够吗 | 换规划器吗 |
|---|---|---|---|
| 只区分"可跨越/阻挡" | 高度带（**已有**） | ✅ | ❌ |
| 优先走缓坡 / 避开陡坡 | 每格**坡度** → 代价 | ✅（NavFn 按累计代价绕开高代价区） | ❌ |
| 隧道 / 悬垂 | 每格**净空高度** → 低于车高记致命 | ✅ | ❌ |
| 坡度影响速度 / 禁止横穿陡坡 / 限制俯仰 | 坡度 + 机器人爬坡能力模型 | ❌ 代价表达不了运动学 | ✅ 2.5D lattice 规划器（+ 控制器按坡度限速） |
| 多层空间（真悬垂穿越） | 体素/多层 | ❌ | ✅ 3D 规划器 |

> 代价图层的**天花板**：NavFn/Smac 只做"累计代价最短"，它能避开高代价区，但**不会因为"横穿陡坡"这个方向性危险而拒绝**，也不会按坡度限速 —— 那要规划器与控制器配合。

#### ③ 我们现在有什么（3D）／没有什么（2.5D）

**有 3D**：LIO 的 3D 点云地图（`PCD/<world>.pcd`、`/cloud_registered`）；`linefit` 的 3D 障碍点云 `/segmentation/obstacle`；
**STVL 的 3D 体素栅格**（带时间衰减）——但它是 **apt 第三方插件**，且**只输出投影后的 2D 代价**，不对外提供"每格高度/坡度/净空"。

**没有 2.5D**：仓库里没有高程图/坡度图/粗糙度/台阶高度/净空图。
唯一带 "slope" 的是 `linefit` 的地面线坡度阈值（那是"算不算地面"的判据，不是可通行性图）。

#### ④ 这些代码现在在哪／要加的话放哪

| 层 | 现在的位置 |
|---|---|
| 3D 里程计/建图 | `src/rm_localization/FAST_LIO/`、`src/rm_localization/point_lio/`（原版对照在 `third_party/`） |
| 地面分割 / 点云→scan / IMU 滤波 | `src/rm_perception/{linefit_ground_segementation_ros2, pointcloud_to_laserscan, imu_complementary_filter}/` |
| 3D 体素层（STVL） | **apt 第三方**（`/opt/ros/humble/share/spatio_temporal_voxel_layer`）；参数在 `src/rm_navigation/rm_navigation/params/nav2_params_sim_*.yaml` 的 `stvl_layer` |
| 2D costmap / 规划 / 控制装配 | `src/rm_navigation/rm_navigation/`（launch/params/rviz）；`fake_vel_transform/`、`teb_local_planner/`、`costmap_converter/` |
| 离线工具 | `tools/pcd_to_grid_map.py`（2D 投影，**不是 2.5D**）、`tools/check_map_reachable.py` |

**要做 2.5D，建议这样落位**（符合"目录=角色域"约定）：

| 新组件 | 放哪 | 做什么 |
|---|---|---|
| 2.5D 表示生产者 | `src/rm_perception/rm_elevation_map/`（新包） | 从点云/体素产出高程、坡度、粗糙度、**净空**图（ROS 话题形式） |
| costmap 图层插件 | `src/rm_navigation/rm_costmap_layers/`（新包） | 实现 `nav2_costmap_2d::Layer`：净空<车高→致命；坡度>阈值→代价↑/致命 |
| （可选）2.5D 规划器 | `src/rm_navigation/rm_planner_25d/`（新包） | 实现 `nav2_core::GlobalPlanner`：坡度相关的运动约束 |

#### ⑤ 最小可行路线（三步，可独立验收）

1. **先把表示做出来（离线即可，最省）**：用 `tools/pcd_to_grid_map.py` 扩展出 `--mode elevation|slope|clearance`，
   把 LIO 的 `PCD/<world>.pcd` 变成 2.5D 图并**扫一眼坡道/隧道处这些量的分布**（不用跑仿真）；
2. **再加图层**：写 `Layer` 插件（净空/坡度 → 代价或致命），用"坡道/隧道用例"回归；
3. **按需升级规划器**：若出现"横穿陡坡 / 坡上不减速 / 隧道内姿态不对"，再做 2.5D lattice 规划器 + 控制器坡度限速。

### 3.3 `rm_nav_bringup`（总装层）内部

```
rm_nav_bringup/                          # 总装层（无 config/：参数已全部回归各包）
├── launch/
│   ├── bringup_sim.launch.py        # 仿真总入口（world/mode/lio/localization 参数）
│   └── cartographer_sim.launch.py   # cartographer 专用启动（默认引包内 lua）
├── map/        # 各场地地图产物 + empty_map（mapping 模式用）
├── PCD/        # 3D 点云图（icp_registration 用）
├── rviz/       # fastlio/pointlio 可视化配置
└── urdf/       # 机器人描述（sim）
```

> 平台外参已移至 `src/rm_simulation/hzmi_rm_simulation/config/measurement_params_sim.yaml`；
> 真车配置与入口已归档至 `archive/reality_frozen/`。

---

## 四、参数归属规则（R1–R5）与当前落地

**规则**：算法参数一律回归**算法包自己的 `config/`**；总装层只留平台参数与冻结内容；每个包由自身 CMake 安装配置。

| 参数（原都在 bringup） | 现在归属 | 状态 |
|---|---|---|
| `fastlio_mid360_sim.yaml` | `rm_localization/FAST_LIO/config/`（fork 内） | ✅ |
| `pointlio_mid360_sim.yaml`（含原 launch 内嵌调参） | `rm_localization/point_lio/config/`（fork 内） | ✅ |
| `icp_registration_sim.yaml` | `rm_localization/icp_registration/config/` | ✅ |
| `segmentation_sim.yaml` | `rm_perception/.../linefit_ground_segmentation_ros/config/` | ✅ |
| `mapper_params_*_sim.yaml` | `rm_localization/slam_toolbox/config/` | ✅ |
| `nav2_params_sim_{rpp,dwb,teb}.yaml` | `rm_navigation/rm_navigation/params/` | ✅（由 `nav:=` 选择） |
| `cartographer.lua` / `cartographer_localization.lua` | `rm_localization/cartographer_ros/configuration_files/`（官方示例同目录） | ✅ |
| imu 滤波 / laserscan / fake_vel 参数（原 launch 内嵌） | 各自包 `config/` | ✅ |
| `measurement_params_sim.yaml` | `rm_simulation/hzmi_rm_simulation/config/`（**平台参数，规则③**） | ✅ 已归平台包 |
| reality 全套（原 `config/reality/*`） | `archive/reality_frozen/`（**冻结归档，规则④**） | 已归档 |

> **`rm_nav_bringup/config/` 已清空移除**（2026-09）：平台外参→仿真平台包；reality→archive；lua→cartographer_ros。`bringup_sim.launch.py` 是当前唯一仿真入口。

**地图产物配对铁律**：`.pgm`→AMCL ｜ `.posegraph`→slam_toolbox(localization) ｜ `.pcd`→icp_registration ｜ `.pbstream`→Cartographer 纯定位。

---

## 五、`third_party/` —— 官方原版参考（只读，不编译）

| 目录 | 上游 | 锁定 commit | 用途 |
|---|---|---|---|
| `fast_lio` | hku-mars/FAST_LIO | 7cc4175 | 与 `src/.../FAST_LIO`（改动版）对照 |
| `point_lio` | hku-mars/Point-LIO | 4b86a46 | 与 `src/.../point_lio` 对照 |
| `cartographer` | cartographer-project/cartographer | 877157a | 核心库参考（ROS2 用法由 `src/.../cartographer_ros` 提供） |
| `nav2` | ros-navigation/navigation2 (humble) | 3c3db59 | **完整源码对照学习**（本工程运行用 apt + 自研 rm_navigation 组装） |

规则：**只读**；`third_party/COLCON_IGNORE` 保证不参与编译；新增候选库用 `git submodule add` 并登记到 `third_party/README.md`。

---

## 六、子模块清单（9 个，`git clone --recursive` 可完整还原）

| 路径 | 上游 | 性质 |
|---|---|---|
| `src/rm_driver/livox_ros_driver2` | gitee SMBU-POLARBEAR（humble 分支） | 第三方 |
| `src/rm_localization/FAST_LIO` | **github.com/William-kk985/FAST_LIO** | **自有 fork**（含本地适配 + ikd-Tree 扁平化） |
| `src/rm_localization/point_lio` | **github.com/William-kk985/Point-LIO** | **自有 fork** |
| `src/rm_navigation/teb_local_planner` | rst-tu-dortmund/teb_local_planner | 第三方 |
| `src/rm_navigation/costmap_converter` | LihanChen2004/costmap_converter | 第三方 |
| `third_party/fast_lio` | hku-mars/FAST_LIO | 原版参考 |
| `third_party/point_lio` | hku-mars/Point-LIO | 原版参考 |
| `third_party/cartographer` | cartographer-project/cartographer | 原版参考 |
| `third_party/nav2` | ros-navigation/navigation2 (humble) | 原版参考 |

> 内嵌源码（无 .git，随主仓提交）：`src/rm_localization/slam_toolbox`、`src/rm_localization/cartographer_ros`。

---

## 七、`tools/` —— 脚本入口

```
tools/
├── check_map_reachable.py   # ★地图可达性/目标点检查（选测试目标点前必跑，含连通域判定）
├── *.py                     # Python 工具（评测/后处理等，按需新增）
└── scripts/                 # Shell 脚本（原顶层 scripts/ 已合并至此）
    ├── build.sh
    ├── control/start_sentinel.sh        # ★一键启动仿真
    ├── control/improved_teleop.sh
    ├── mapping/quick_start_cartographer.sh
    ├── mapping/generate_cartographer_pbstream.sh
    ├── mapping/save_pcd.sh  save_grid_map.sh
    ├── create_config_package.sh  setup_from_package.sh
    └── ../README.md
```

常用：

```bash
tools/scripts/control/start_sentinel.sh                      # 默认 RMUL2026 + mapping + fastlio
tools/scripts/control/start_sentinel.sh -w RMUC -m nav --lio pointlio
# 选导航测试目标点前先验证可达性（避免目标点被墙隔开导致规划失败）
/usr/bin/python3 tools/check_map_reachable.py --map src/rm_nav_bringup/map/RMUL.yaml --start 0 0 --goal 1.68 3.44
# 或手动
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=fastlio
```

---

## 八、`docs/` 文档索引

> **完整索引见 [`docs/README.md`](README.md)**（唯一入口，含每份文档的状态与维护约定）。
> 这里只留"最常用的三份"：

| 你最可能想看的 | 文档 |
|---|---|
| 怎么跑、怎么判、错了怎么查 | `docs/smoke_test_runbook.md` |
| 现在有哪些算法/能不能跑（真值来源） | `docs/algorithm_matrix.md` |
| 目录架构与分层概念（§3.2.1–3.2.7） | `docs/architecture.md`（本文件） |

其余：`glossary.md`（术语）、`issues_and_findings.md`（坑与根因）、`3d_to_2d_survey.md`（降维对照）、
`tf_interface_contract.md`（TF 契约）、`params_ownership_checklist.md`（参数归属）、
`rm_algorithm_catalog.md`（选型池）、`mapping/`、`package/`（专题），历史草案见索引"状态"列。

---

## 九、构建与运行要点

```bash
# 构建（19 个包）
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 一键启动（推荐）
tools/scripts/control/start_sentinel.sh
```

注意：
- 若 shell 里 conda/miniconda 的 `python3` 抢占 PATH，CMake 会因缺 `catkin_pkg` 报错 → 构建时用系统 python（`PATH=/usr/local/bin:/usr/bin:/bin:...`）；
- 依赖 `ros-humble-dwb-critics`（teb 编译需要）；
- `third_party/` 不参与构建（COLCON_IGNORE）；`slam_toolbox`、`cartographer_ros` 为源码 overlay，**编译产物优先于 apt 版**。

### 仿真参数速查（`bringup_sim.launch.py`）

| 参数 | 取值 | 说明 |
|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026`（默认） | 场地（同时决定 map/PCD 前缀） |
| `mode` | **`mapping`（纯建图）/ `slam_nav`（边建图边导航）/ `nav`（先建图后导航）** | 三种**场景形态**，启动的节点集明显不同，见下表；`localization` 仅 `nav` 生效 |
| `lio` | `fastlio`（默认） / `pointlio` / **`none`** | 里程计实现选择；**`none` = 不启动任何 LIO**（须由外部提供 odom/TF，如轮式里程计或 cartographer；此时 LIO 相关的静态 TF 桥不会启动） |
| `nav` | `rpp`（默认） / `dwb` / `teb` | **局部规划器变体**：对应 `rm_navigation/params/nav2_params_sim_<nav>.yaml`（全局规划统一为 Navfn）；`nav`/`slam_nav` 形态生效 |
| `mapper` | `slam_toolbox`（默认） / `cartographer` | **在线 2D 建图后端**：`mapping`/`slam_nav` 生效 |
| `localization` | `amcl` / `slam_toolbox` / `icp` / **`cartographer`**（**仅 `nav` 生效**） | 重定位方式；留空 = 回退用法（LIO 当绝对定位 + 静态桥补帧） |
| `global_obstacle` | `stvl`（默认）/ `scan` / `none` | **全局代价地图的实时障碍来源**（bench 槽位）：`stvl`=3D 体素层；`scan`=2D `/scan`（与 local 同源 → 降维只在感知域一处）；`none`=只 static+inflation。见 `docs/3d_to_2d_survey.md` §六 |
| `lio_rviz` / `nav_rviz` | `True` / `False` | 可视化开关 |
| `spin_speed` | `5.0`（默认，上游哨兵语义）/ `0.0` | `fake_vel_transform` 的小陀螺固定角速度；**仿真排查导航问题先用 `0.0`**（角速度直通） |

**三种场景形态的启动集（2026-09 拆分）**：

| `mode` | 在线 SLAM 后端 | 导航栈 nav2 | `map_server` | 重定位模块 | `map→odom` 来源 |
|---|---|---|---|---|---|
| `mapping` 纯建图 | ✅ | ❌ **不启动**（省 CPU） | ❌ | ❌ | 在线 SLAM（供离线保存） |
| `slam_nav` 边建图边导航 | ✅ | ✅ | ❌ | ❌ | 在线 SLAM 直接喂 costmap |
| `nav` 先建图后导航 | ❌ | ✅ | 仅 `icp` / 留空时 | ✅ 按 `localization` | 所选重定位模块 |

> 设计要点：**"地图从哪来"是这两条导航链唯一的分界**——在线 SLAM（含回环优化，`map→odom` 会跳变）vs 磁盘地图 + 重定位（`map→odom` 平滑）。三种形态下 `/map` 都只有一个发布者。


> `lio:=none` 的用途：跑**纯 2D 组合**（例如 cartographer 自带前端，或轮式里程计）时避免 LIO 与之争抢位姿/TF。注意 2D 定位/导航链仍需要 `odom→base_link` 与 `map→odom`，`none` 只是"不由本工程提供"，需另接来源。
>
> **TF/话题契约（T1–T5 已实施，2026-09）**：① 两套 LIO 的里程计话题在 bringup 内统一 remap 为 **`/odom`**；② 新增 **`lio_tf_adapter`** 把 LIO 位姿转成标准 **`odom→base_link`**；③ sim URDF 关闭了 Gazebo 的 `publish_odom_tf`，使 LIO 成为**唯一位姿来源**（仿真不再依赖真值 TF）；④ 静态帧桥改为**仅在 `mode:=nav` 且未选任何重定位模块**时启动（amcl/slam_toolbox/ICP 都自发布 `map→odom`，不得叠加），并删除了重复的 `base_link→base_link_fake` 静态桥；⑤ `icp_registration` 的 `range_odom_frame_id` 修正为 `odom`，其 `map→odom` 才真正生效。~~剩余 T6（nav2 `robot_base_frame` 去 `base_link_fake`）~~ **T6 已撤销**：`base_link_fake` 是哨兵云台机制的载体（`fake_vel_transform` 20Hz 发布 + `/cmd_vel`→`/cmd_vel_chassis` 旋转链路），改为 `base_link` 会丢掉小陀螺与云台解耦。详见 `docs/tf_interface_contract.md`。

---

## 十、当前待办（不阻塞）

1. **TF 桥收口**：`bringup_sim` 中 `camera_init→map`、`body→odom`、`base_link→base_link_fake` 为帧对齐调试手段，需理清帧树后正式化；
2. **`RMUL2026.pbstream` 仅 526B**（疑空）→ 需重新建图；RMUL2026 亦缺 `.posegraph`；
3. **整体实跑验证**：`start_sentinel.sh` 的 mapping/nav × fastlio/pointlio 组合逐一跑通（含参数回归后的行为确认）。
