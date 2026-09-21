# 算法组合总表（当前能力矩阵）

> 本文件是 bench 的**「算法现状唯一真值来源」（living doc）**：当前有哪些算法、能怎么组合、
> 资产齐不齐、哪些已实跑、卡在哪、下一步加什么。
>
> **维护约定（照这个更新，别另开文档）**
> | 什么时候 | 更新哪一节 |
> |---|---|
> | 新增/替换算法包 | §一 槽位表 + §一.1 维度表 |
> | 跑通一个组合 | §四 加一行（状态 + 证据） |
> | 发现阻塞 / 资产缺失 | §五 |
> | 想要但还没有的算法 | §六 扩展位 |
> | 目录结构类决策 | §一.1 开头（先例：**不按 2D/3D 物理重组目录**，2026-09-16 定） |
>
> 设计原则：**目录=技术域（稳定轴）；启动参数=角色槽位；维度只做文档标注（不物理重组）**；
> 算法参数各自回归所属包的 `config/`（R1）。

---

## 一、角色槽位与实现（`bringup_sim.launch.py`）

| 槽位 | 参数 | 现有实现 | 对应包 / 节点 | 生效条件 |
|---|---|---|---|---|
| **场景形态** | `mode` | `mapping` 纯建图 / `slam_nav` 边建边导 / `nav` 先建后导 | — | 必填 |
| **里程计** | `lio` | `fastlio` / `pointlio` / `none` / **`cartographer`（全包）** | `src/rm_localization/FAST_LIO`、`point_lio`；`none` 需外部提供 odom/TF；`cartographer` = 同一个 cartographer 兼任里程计源（`mapper`/`localization` 槽被跳过，lua `cartographer_lio*.lua`） | 全形态 |
| **在线建图** | `mapper` | `slam_toolbox` / `cartographer` | `src/rm_localization/slam_toolbox`（async）、`cartographer_ros`（+ `cartographer_occupancy_grid_node`） | `mapping` / `slam_nav` |
| **重定位** | `localization` | `amcl` / `slam_toolbox`(纯定位) / `icp` | `nav2_amcl`(+`map_server`)、`slam_toolbox`(localization)、`src/rm_localization/icp_registration` | 仅 `nav` |
| **局部规划器** | `nav` | `rpp` / `dwb` / `teb` | `nav2_regulated_pure_pursuit_controller` / `nav2_dwb_controller` / `teb_local_planner`（+ `costmap_converter`） | `nav` / `slam_nav` |
| **全局规划器** | —（固定） | `NavfnPlanner` | `nav2_navfn_planner` | 同上 |
| **场地** | `world` | `RMUC` / `RMUL` / `RMUL2026` | `hzmi_rm_simulation` 世界 + `map/<world>.*` + `PCD/<world>.pcd` | 全形态 |
| **全局障碍来源** | `global_obstacle` | `stvl`（3D 体素层，默认）/ `scan`（2D `/scan`，与 local 同源）/ `none`（只 static+inflation） | `nav` / `slam_nav` |
| **局部障碍来源** | `local_obstacle` | `scan`（`/scan`，默认=原行为）/ `cloud`（`/segmentation/obstacle` 点云直投，不经 `p2l`）/ `both`（双源冗余） | `nav` / `slam_nav` |
| **小陀螺** | `spin_speed` | `5.0`（哨兵语义）/ `0.0`（角速度直通，排查用） | `fake_vel_transform` | 全形态 |
| 可视化 | `lio_rviz` / `nav_rviz` | True/False | `fastlio.rviz` / `pointlio.rviz` / `nav2.rviz` | 全形态 |

## 一.1 维度归类：哪些模块是 2D / 2.5D / 3D（**按域分文件夹，按维度做标注**）

**结论先说（2026-09-16 决策）：不按 2D/3D 物理重组目录，维度只做文档标注。**

**"两个正交的轴"是什么意思**（人话）：
- **技术域** = 这个东西**是干什么的**（定位 / 感知 / 导航 / 仿真 / 驱动）；
- **维度** = 它**处理的数据是几维的**（2D / 2.5D / 3D）。
- 这两件事**互不决定**：知道它在定位域，猜不出它是 2D 还是 3D；知道它是 3D，也猜不出它属于哪个域。
  所以叫"正交"（独立）。用表格摊开看最清楚：

| | 2D | 2.5D | 3D |
|---|---|---|---|
| **定位** | slam_toolbox | — | FAST-LIO、point_lio、ICP |
| **感知** | — | **（空缺：高程/坡度/净空）** | linefit、IMU 滤波、（p2l 是桥） |
| **导航** | costmap/planner/controller/behavior、TEB、fake_vel | **STVL** | — |
| **仿真** | — | — | 世界/URDF、livox 仿真 |
| **驱动** | — | — | livox_ros_driver2 |

每一行里都有多个维度、每一列里都有多个域 → 两个轴都得"摊开"，说明它们独立。

**为什么不能硬拆成目录**：目录树**只能按一个轴分**，另一个轴就会被拆散。像衣柜：
你可以按"用途"（上衣/裤子/鞋）放，也可以按"季节"（夏/冬）放，但**柜子只有一层格子**——
按用途放，"四季外套"和"夏冬都穿的鞋"就得靠标签；按季节放，"用途"就得靠标签。
强行按季节放，遇到 `cartographer_ros`（**同时是 2D 和 3D**）就只能塞进一个格子里，另一个身份丢了。

具体代价：
1. **跨维度的东西无处安放**：`cartographer_ros`(2D+3D)、`pointcloud_to_laserscan`(3D→2D 桥)、STVL(导航里的 2.5D)；
2. **一条流水线被拆散**：`linefit`(3D) → `p2l`(3D→2D) → `/scan`(2D) 会分散到两个目录，读代码要来回跳；
3. **colcon/launch/install 全靠包路径**：移动包要改所有 `get_package_share_directory`、参数路径、`tools/` 引用、submodule 引用；
4. **维度会变**：今天 2.5D 空缺、明天新增两个包；今天 slam_toolbox 只做 2D、明天支持 3D —— 按维度分目录就得反复搬家。

**所以：目录按"稳定"的轴（技术域）分，维度用这张表标注。**

| 包 / 模块 | 技术域 | **维度** | 角色 | 说明 |
|---|---|---|---|---|
| `rm_localization/FAST_LIO` | 定位 | **3D** | 里程计（+3D 建图） | 3D 点云 + IMU；无回环 |
| `rm_localization/point_lio` | 定位 | **3D** | 里程计（+3D 建图） | 同上，逐点更新、高速更稳 |
| `rm_localization/icp_registration` | 定位 | **3D 算法 / 2D 效果** | 重定位（一次性引导） | 3D 点云配准 → 输出 `map→odom` |
| `rm_localization/slam_toolbox` | 定位 | **2D** | 在线建图 / 纯定位 | 2D 激光 + `.posegraph` |
| `rm_localization/cartographer_ros` | 定位 | **2D + 3D** | 在线建图 / 纯定位 | 我们只用 2D lua |
| `rm_localization/lio_tf_adapter` | 定位 | 维度无关（桥接） | `/odom` → TF | — |
| `rm_perception/linefit_ground_segementation_ros2` | 感知 | **3D** | 去地面 | 3D 点云 → `/segmentation/obstacle` |
| `rm_perception/pointcloud_to_laserscan` | 感知 | **桥：3D→2D** | 高度带切片 | 输出 `/scan` |
| `rm_perception/imu_complementary_filter` | 感知 | 3D 数据 | IMU 滤波 | 供 LIO 使用 |
| `rm_navigation/rm_navigation` | 导航 | **2D**（参数里含 2.5D 体素层） | nav2 装配/参数/rviz | costmap、planner、controller、behavior |
| `rm_navigation/teb_local_planner` | 导航 | **2D** | 局部控制 | — |
| `rm_navigation/costmap_converter` | 导航 | **2D** | costmap→多边形 | 供 TEB |
| `rm_navigation/fake_vel_transform` | 导航 | **2D** | 速度变换/假云台系 | 平面运动 + 小陀螺语义 |
| `rm_simulation/livox_laser_simulation_RO2` | 仿真 | **3D 传感器** | 3D 雷达仿真 | 出 CustomMsg + PointCloud2 |
| `rm_simulation/hzmi_rm_simulation` | 仿真 | **3D 世界** | 世界/URDF/机器人 | — |
| `rm_driver/livox_ros_driver2` | 驱动 | **3D 传感器** | 真机 3D 雷达 | — |
| （第三方 apt）`spatio_temporal_voxel_layer` | 导航 | **2.5D** | 3D 体素 → 2D 代价投影 | 现成的 2.5D 环节；**已做成可切换槽位**：`global_obstacle:=stvl`(默认) / `scan` / `none` |
| **（空缺）** | 感知 | **2.5D** | 高程/坡度/净空图 | 待建 `src/rm_perception/rm_elevation_map/` |
| **（空缺）** | 导航 | **2.5D** | 把高程/净空变成代价 | 待建 `src/rm_navigation/rm_costmap_layers/` |
| `tools/pcd_to_grid_map.py` | 工具 | **3D→2D（离线）** | 点云切层投影成栅格 | — |
| `tools/check_map_reachable.py` | 工具 | **2D** | 连通域/选点 | — |
| `rm_nav_bringup` | 装配 | 维度无关 | launch/总装 | 谁在岗由 `mode` 决定 |

**读表要点**：
- **3D 侧** = LIO 系（FAST-LIO/Point-LIO）+ 点云资产（`.pcd`）+ ICP + 3D 传感器/驱动；
- **2D 侧** = 2D SLAM 后端（slam_toolbox、cartographer-2D）+ nav2 全套（costmap/plan/control/behavior）；
- **跨维度桥只有三处**：`pointcloud_to_laserscan`（在线 3D→2D）、STVL（3D 体素→2D 代价）、`tools/pcd_to_grid_map.py`（离线 3D→2D）——**"降维发生在哪"就查这三处**；
- **2.5D 是空缺**：只有 STVL 的体素层算"半个"，高程/坡度/净空图还没有（落位见 architecture §3.2.7）。

## 一.2 为什么 `localization` 只在 `nav` 生效、`mapper` 只在 `mapping|slam_nav` 生效

**先分清两个词**：
- **定位（localization）** = "我现在在哪" → 输出 `map→odom`。**三种形态都需要**；
- **重定位（relocalization）** = "在**已有的先验地图**里找回我的位置" → 是定位的一种**特殊场景**（有先验图）。
  `mapping` / `slam_nav` 里没有（或不需要）先验图，所以**不需要重定位模块** —— 它们的定位由**在线 SLAM 自己**提供。

所以三个角色在三种形态下的"在岗情况"是：

| | 谁提供 `odom→base_link` | **谁提供 `map→odom`（全局对齐）** | 地图资产从哪来 |
|---|---|---|---|
| `mapping` 纯建图 | LIO | **在线 mapper**（slam_toolbox/cartographer） | 正在实时产出 |
| `slam_nav` 边建边导 | LIO | **在线 mapper**（同上） | 实时产出、边造边用 |
| `nav` 先建后导 | LIO | **`localization` 槽位**（amcl/slamTB-loc/icp） | 磁盘（先前建好） |

**于是两条"生效条件"的动机就清楚了**：

1. **`map→odom` 同一时刻只能有一个发布者**（TF 单父边）。在线 mapper 和重定位模块**都会发** `map→odom`，
   并行就会分叉 → 所以必须由 `mode` 决定"这一形态里归谁"；
2. **`/map` 同一时刻也只能有一个发布者**。在线 mapper 发 `/map`（建图产物），`map_server` 发 `/map`（先验图），
   并行就会抢话题（这正是 2026-09 修掉的那个坑）→ 所以 `mapping/slam_nav` 不起 `map_server`、`nav` 才起。

> 结论：**不是"另外两个形态不需要定位"，而是"那里的全局对齐已经由在线 SLAM 承担了"。**
> 若确实想在建图时也用先验图，正确做法是**序列化**（先 ICP/AMCL 引导一次 → 停发 TF → 交给 SLAM），
> 而不是让两个模块并行（见 architecture §3.2.7 与"ICP+AMCL handover"讨论）。

## 二、组合数量（核心组合 = 120）

| 形态 | 组合数 | 算式 |
|---|---|---|
| `mapping` | **12** | 3 场地 × 2 LIO × 2 mapper |
| `slam_nav` | **36** | 3 × 2 × 2 × 3 局部规划器 |
| `nav` | **72** | 3 × 2 × **4 重定位**(amcl/slamTB/icp/cartographer) × 3 局部规划器 |
| **合计** | **120** | 不含 `spin_speed`/`global_obstacle`/`local_obstacle`/`*_rviz` 等开关 |

另有特殊用法：`lio:=none`（需外部 odom/TF，例如轮式里程计或纯 2D 组合）、**`lio:=cartographer`（全包形态：cartographer 兼任里程计源，`mapper`/`localization` 槽被跳过）**、`mode:=nav localization:=''`（LIO 当绝对定位的静态桥回退用法）。

**加上"回退用法"（`mode:=nav` 且 `localization` 留空）**：nav 变为 3×2×**5**×3 = 90 → 合计 **138**。

**三个"装配级开关"会成倍影响行为，A/B 时一次只动一个（注意各自生效范围不同）**：

| 形态 | 核心组合 | `spin_speed`(×2) | `global_obstacle`(×3) | `local_obstacle`(×3) | 小计 |
|---|---|---|---|---|---|
| `mapping` | 12 | ✅ 生效 | ❌ 不起 nav2，不适用 | ❌ 不适用 | **24** |
| `slam_nav` | 36 | ✅ | ✅ | ✅ | **648** |
| `nav` | 72 | ✅ | ✅ | ✅ | **1296** |
| **合计** | **120** | | | | **1968** |

（若把 `localization:=''` 回退用法计入，nav 变 90 → 合计 **2292**；`*_rviz` 属纯可视化开关，不计入。）

## 三、资产可用性矩阵（决定组合"能不能真跑"）

| 资产（`src/rm_nav_bringup/`） | RMUC | RMUL | RMUL2026 | 谁消费 |
|---|---|---|---|---|
| `map/<w>.pgm` + `.yaml` | ✅ 577×301 | ✅ 272×210 | ⚠️ 240×169（**含幽灵墙**，见 §五） | `localization:=amcl`（经 `map_server`） |
| `map/<w>.posegraph` | ✅ 13.7 MB | ✅ 13.8 MB | ❌ **缺** | `localization:=slam_toolbox` |
| `PCD/<w>.pcd` | ✅ 18 MB | ✅ 50 MB | ❌ **缺**（可用 `/map_save` 现场生成） | `localization:=icp` |
| `map/<w>.pbstream` | ❌ 缺 | ❌ 缺 | ⚠️ 526 B 空壳（旧的失败产物） | cartographer 纯定位（`localization:=cartographer`） |
| `map/<w>.data` | ✅ 8.5 MB | ✅ 4.2 MB | ❌ | 当前流程**未使用**（遗留资产） |

**推论（当前的"硬约束"）**：
- **AMCL 路线三个场地都能跑**（RMUL2026 的图需先重建，否则发目标会规划失败）；
- **slam_toolbox 纯定位 / ICP 只能 RMUC、RMUL**；
- **cartographer 纯定位目前无场地可用**（三个 pbstream 都不存在/为空）→ 必须先按 `docs/smoke_test_runbook.md` §5 跑一次 cartographer 建图，再 `finish_trajectory` + `write_state` 导出；
- RMUL/RMUC 的地图是**出生点系**（AMCL 初值 `(0,0)`），RMUL2026 是**世界系**（`(4.3,3.35)`）——判定依据见 `docs/smoke_test_runbook.md` §0.5。

## 四、实测状态（每跑通一个组合就加一行）

| 组合 | 状态 | 结论/证据 |
|---|---|---|
| `nav` + `fastlio` + `amcl` + `rpp` @ **RMUL2026** | ✅ **全链路通过** | `/controller_server`+`/planner_server` active；`map→odom`=(4.294,3.357,0.052)；`/amcl_pose`=(4.300,3.350)；`/scan` QoS 全 BEST_EFFORT |
| `nav` + `fastlio` + `amcl` + `rpp` @ RMUL2026 **发目标** | ⚠️ 被地图挡住 | `planner_server: failed to generate a valid path` → BT 恢复循环；根因=幽灵墙（§五） |
| `nav` + `fastlio` + `icp` + `rpp` @ RMUL | ⚠️ 未进入 ICP | `spawn_entity` 失败 + `odom` 帧长时间不存在（RMUL 世界 44 万面加载慢）；ICP 自身初始化正常 |
| `mapping`（三种形态的启动集） | ✅ 启动集已验 | 8 种 `mode`×`mapper`×`localization` 真值表验证；建图模式不再起 nav2/map_server |
| `mapping` 完整落盘（pgm+posegraph+pcd） | ❌ 未跑 | 流程见 runbook §1.1 |
| `slam_nav` 边建边导 | ❌ 未跑 | QoS 已核（两个后端的 `/map` 都是 transient_local） |
| `nav` + `slam_toolbox` 纯定位 | ❌ 未跑 | 需 `.posegraph`（RMUC/RMUL 有） |
| `nav` + cartographer 纯定位 | ❌ 未跑 | **缺 pbstream** |
| `mapping mapper:=cartographer` | ❌ 未跑 | — |
| `nav:=dwb` / `nav:=teb` | ❌ 未跑 | 参数文件已就绪 |

## 五、已知阻塞项（先解决这几个，组合才能真跑）

| 阻塞 | 影响 | 解决 |
|---|---|---|
| **RMUL2026.pgm 幽灵墙**（x≈5.2 的 1 像素虚线，世界网格该处零顶点） | 该场地**任何** nav 组合发目标都会规划失败（地图被切成 3 块） | 在 sim 里重建 RMUL2026 图（pgm+posegraph+pcd 三件套），并把 `amcl_init_x/y` 改回 `0.0` |
| **RMUL2026 缺 posegraph / pcd** | `localization:=slam_toolbox` / `icp` 在该场地不可用 | 同上，重建时一并落盘 |
| **无可用 pbstream** | `localization:=cartographer` 起不来 | **cartographer 建图的对接 bug 已修**（输入改 `/scan`、帧契约、高度带），现在可 `mode:=mapping mapper:=cartographer` 建图后 `finish_trajectory`+`write_state` 生成 |
| RMUL 世界加载慢（44 万面） | 组合在 RMUL 上启动易失败 | 等 30~60 s；按话题逐段排查 |

## 六、下一步要加的槽位（扩展位）

| 槽位 | 候选 | 说明 |
|---|---|---|
| 里程计 | **轮式里程计**（`lio:=none` 的真实用法） | 目前 `none` 没有外部 odom 提供者 |
| 里程计 | **带回环的 LIO**：LIO-SAM / GLIM / hdl_graph_slam | 3D 地图也全局一致（见 `docs/rm_algorithm_catalog.md`） |
| 全局规划 | `SmacPlanner2D` / `SmacPlannerHybrid` | 支持运动学约束 |
| 局部规划 | `MPPI` | 现代采样式，鲁棒 |
| 感知/表示 | **2.5D 高程/坡度/净空图层**（`src/rm_perception/rm_elevation_map/` + `src/rm_navigation/rm_costmap_layers/`） | 上坡/下坡/隧道场景（见 architecture §3.2.7） |
| 任务 | **云台瞄准规划**（2–3 DOF） | 哨兵场景真正需要 3D 的地方 |
| 退化测试 | `cmd_vel` 延迟/丢包注入、打滑注入、点云离群点注入 | 选型阶段筛"谁在退化条件下还能活" |

---

## 七、更新日志

| 日期 | 变更 |
|---|---|
| 2026-09-16 | 建文件：§一 槽位与实现（102 个核心组合）、§一.1 维度归类、§三 资产可用性矩阵、§四 实测状态、§五 阻塞项、§六 扩展位 |
| 2026-09-16 | 新增装配级槽位 **`global_obstacle`**（`stvl`/`scan`/`none`，默认 `stvl` 行为不变）：全局代价地图的实时障碍来源可切换，用于 A/B 研究「谁来做 3D→2D」 |
| 2026-09-16 | 新增 `docs/glossary.md`（术语表）并登记进 architecture/runbook 文档索引 |
| 2026-09-16 | 修 `global_obstacle:=scan` 的量程隐患：global 的 scan 源 `obstacle/raytrace_max_range` 由照抄 local 的 6.0 改为 **10.0**（与 p2l `range_max` 对齐），否则全图 6 m 外的旧标记永不清除 → 幽灵障碍 |
| 2026-09-16 | **决策：不按 2D/3D 物理重组目录**（维度与技术域正交：目录按域分、维度用文档标注）；后续算法情况一律在本文件更新 |
| 2026-09-21 | 新增装配级槽位 **`local_obstacle`**（`scan`/`cloud`/`both`，默认 `scan` 行为不变）：局部代价地图可加第二路（点云直投，不经 `p2l`）→ 破 `p2l` 单点、消 45cm 盲区；组合数 672 → **1968**（含回退 2292） |
| 2026-09-21 | 新增里程计槽位取值 **`lio:=cartographer`（全包形态）**：同一个 cartographer 兼任里程计源（`provide_odom_frame=true`，发 `odom→base_link` + `map→odom`）→ `mapper`/`localization` 槽与 `lio_tf_adapter`/T1 桥全部跳过；新增 lua `cartographer_lio.lua` / `cartographer_lio_localization.lua`。代价：无独立故障域、无 `/odom` 话题（`nav:=teb` 不适用） |
| 2026-09-21 | 修 **`localization:=cartographer` 启动即 FATAL**：`cartographer_localization.lua` 把 `pure_localization`/`pure_localization_trimmer` 写在 `TRAJECTORY_BUILDER_2D` 上，而 cartographer 从顶层 `TRAJECTORY_BUILDER` 读 → 键永不被读 → `LuaParameterDictionary` CHECK `Key 'pure_localization' was used the wrong number of times`。改为官方写法 `TRAJECTORY_BUILDER.pure_localization_trimmer` |
| 2026-09-21 | 修**静默失效**：所有障碍源加 `expected_update_rate: 0.5`（原默认 0=不检查）→ 源停即 WARN + 拒绝算速度 + `velocity_timeout` 1s 停车；另修 `p2l` 的 `range_min: 0.45 → 0.2`（45cm 盲区会被反向清成 free）、`scan_time: 0.3333 → 0.1` |
