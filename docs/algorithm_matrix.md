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
| **里程计** | `lio` | `fastlio` / `pointlio` / **`small_point_lio`**（2026-10-05 新增）/ `none` / **`cartographer`（全包）** | `src/rm_localization/FAST_LIO`、`point_lio`、**`small_point_lio`**（vendored `Yancey2023/small_point_lio`@`688d75c`，MIT）；`none` 需外部提供 odom/TF；`cartographer` = 同一个 cartographer 兼任里程计源（`mapper`/`localization` 槽被跳过，lua `cartographer_lio*.lua`）。**`small_point_lio` 自己直发 `odom→base_link` ⇒ 不起 `lio_tf_adapter`**；契约、实测与回退见 `docs/lio_slots.md` | 全形态。<br>⚠️ **`small_point_lio` 状态（2026-10-05，别读成"已完成"）**：**可跑通、契约合规**（节点级 40 s 稳定、零 ERROR/WARN、`/Odometry`+TF 有数），但**精度未达标** —— 同一 bag、同一喂法的重放轨迹 **36.6 m** vs `fast_lio` 参照 **6.2 m**（真值 5.51 m）；**整栈（Gazebo+nav2）尚未验证**；调参由另一任务进行中（工具 `tools/lio_node_alone_check.py`；详见 `docs/lio_slots.md` §5）|
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
| `nav + lio:=small_point_lio`（新槽，2026-10-05 起） | ⚠️ **不可算通过**：节点级**可跑通/契约合规**，但**精度未达标**、整栈未跑 | 节点级重放（`tools/lio_node_alone_check.py`，不启 Gazebo/nav2）：`/Odometry`+TF 有数、40 s 不掉线、零 ERROR/WARN；但轨迹 **36.6 m** vs `fast_lio` 参照 **6.2 m**（真值 5.51 m）= 发散。**整栈（Gazebo+nav2）一次都没跑过**；调参进行中（另一任务）⇒ 见 `docs/lio_slots.md` §5 |
| `nav:=dwb` / `nav:=teb` | ❌ 未跑 | 参数文件已就绪 |
| `nav + lio:=pointlio`（任一 localization） | ❌ 未跑（**从建仓起从未全栈验证**） | 节点级重放同一 bag：轨迹 **25.3 m**（参照 6.22 m）且第 ~100 帧被自身 `pcd_save` 打死 ⇒ `docs/params_ownership_checklist.md` §4、`docs/lio_slots.md` §5.4 |

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

## 八、技术方向与优先级（2026-09-24 决策）

### 8.1 判断依据：这两天的失败**全部是契约问题，没有一个是算法问题**

6 个病根（详见 `debug_fastlio_cartographer.md` §9）：cartographer 参数漂移、`<always_on>` 缺失、
CustomMsg QoS 背压、`use_sim_time` 键名漏引号、上游 nav2 丢弃 `transformPose` 失败、`fake_vel_transform`
把角速度替换成 `spin_speed`。**没有一条能靠"换算法/升维度"解决。**
⇒ 结论：**先投资"契约与回归"，再投资"算法选型"，最后才谈"维度升级"**。

### 8.2 维度决策：2D 为主，2.5D 只在两处增量，3D 不进在线决策层

| 层 | 决策 | 理由 |
|---|---|---|
| **导航决策层（规划/控制/costmap）** | **继续 2D，不上 3D 规划器** | nav2 的插件契约就是 2D（`nav_msgs/Path` 是 3D 但输入/控制侧卡死在 2D）；麦克纳姆平地车 2D 足够；3D 规划器算力/调参成本高、实车难复现 |
| **2.5D 增量①：高度带过滤** | **已具备，继续用**（`min/max_obstacle_height`） | 把"横梁/高台"与"地面可越障"分开，是 2D 框架内最划算的维度信息 |
| **2.5D 增量②：坡度/台阶判据** | **条件触发**：场地出现"必须跨/必须绕的高度决策"（坡道、高台、限高）才做；本仓库世界文件当前坡/台阶相关行数 = **1** | 只作 costmap 的一层或减速触发，**不做 3D 规划**。生产端从 `/segmentation/ground` 或 LIO 点云算局部坡度/台阶高度 |
| **3D 的正确用法** | ① LIO 内部（已是 3D）② **离线/验证**：PCD 底图、点云切层投影（见 `3d_to_2d_survey.md`） | 在线决策层保持 2D 契约，3D 只在"造图/验证/标定"环节 |

### 8.3 感知：摄像头**上，但不为导航**（独立感知线）

- **对导航的边际收益低**：平地 + 2D 雷达 + costmap 已够；相机带来的语义障碍需要**内外参标定 + 时间同步**，
  正是 `issues_and_findings.md` 里列的 sim2real 三大杀手之一（标定/时间同步、打滑/延迟、感知噪声）。
- **对哨兵本体的价值高**：装甲板识别与**自动瞄准**、敌人检测与决策 ⇒ 这是**独立于 nav 的感知线**，
  应单独开包（例：`rm_perception/rm_camera_detect` + 云台控制），**不与 costmap 耦合**。
- **若最终要回喂导航**（把敌人当动态障碍）：那才是"视觉×导航"的交汇点，必须建立在标定/时间同步已经可靠之后。
- 顺序：**相机数据流 + 云台 2DOF（契约先行：`camera_info`、时间戳、频率、坐标变换）→ 检测/跟踪 → 再考虑回喂 costmap**。

### 8.4 下一步实验：按性价比排序（每条都有明确判据）

| 优先级 | 实验 | 判据 | 成本 |
|---|---|---|---|
| **P0** | **把"链路看门狗 + 无头目标点"做成一键回归**（`tools/scripts/diag/watch_startup_chain.py` + `ros2 action send_goal` 固定点） | 全绿 + 到点 + 无 `Failed to make progress`；**任何改动跑一次** | 极低（工具已就绪） |
| **P0** | **四跳命令链回归**：`/cmd_vel_nav → /cmd_vel → /cmd_vel_chassis → /odom_ground_truth` | 每跳数值一致且真值变化（今天就是靠它抓到 bug 6） | 极低 |
| **P1** | `local_obstacle: scan → cloud / both`（同一目标点 A/B） | 贴墙 0.3 m 时 local 是否看到；`/scan` 45 cm 盲区是否被覆盖；是否绕开 | 低（改一个槽位） |
| **P1** | `nav: rpp → dwb` | 窄缝通过率、震荡幅度、到达时间、CPU；rpp 作为基线 | 中（调参） |
| **P1** | 修 `stvl_layer` 丢点云（时间区间）或改用 `global_obstacle:=scan` | 全局是否绕开临时障碍；两者 CPU 与保守度对比 | 低~中 |
| **P2** | `localization: amcl → icp`（配 `PCD/<world>.pcd`） | LIO 漂移下是否仍到点；重定位收敛时间 | 中 |
| **P2** | `nav:=teb` | 仅在 `lio:=cartographer`（不发 `/odom`）场景外尝试；对比 rpp/dwb | 高（调参） |
| **P3** | 2.5D 坡度/台阶层 | 仅当场地确有高差决策需求（当前坡/台阶行数 = 1） | 高 |
| **P3** | 相机 + 云台感知线 | 数据流契约（频率/时间戳/`camera_info`）→ 检测 → 跟踪 | 高（另开一条线） |

### 8.5 一条铁律（这两天最贵的教训）

**任何"某环节没数据"的判断，先确认那条命令本身能出数。**
我在这上面栽了三次（`hz` 不认 `--qos-reliability`、CustomMsg 类型 CLI 加载不了、
`launch.log` 不含各节点输出）。工具已把这些坑封起来：**统一用 `watch_startup_chain.py`**
（每个话题的正确 QoS 写死在代码里），不要再用裸 `ros2 topic echo/hz` 判断链路活性。

---

### 8.6 前提修正与"体验优先"路线图（2026-09-24，用户澄清）

**本仓库的定位是算法体验 bench**（见 `rm_algolab_plan.md`），目标是**把 2D/2.5D/3D、各规划器、各感知路线都实际跑一遍并感受差异**，
不是"只选比赛最优解"。因此 §8.2/§8.4 的取舍应读作**性价比排序**，**不是"不做"**。

**体验的四条原则**：
1. **一次只动一个轴**（否则失败时无法归因——这两天已付过学费）；
2. **先做 P0 回归**（看门狗 + 无头目标点 + 四跳命令链），把"单次体验成本"压到 1 分钟级；
3. **先体验"改配置即得"的路线，再体验"要写代码"的路线**；
4. 要写代码的路线之间必须**互相解耦**（相机线不得阻塞雷达/维度线）。

**五条正交轴与"配置即得"入口**：

| 轴 | 可体验项 | 是否需写代码 | 入口 |
|---|---|---|---|
| A 建图/定位 | cartographer 建图/纯定位、slam_toolbox 建图/定位、ICP 重定位、AMCL | 否 | `mode` / `mapper` / `localization` / `lio` |
| B 规划控制 | rpp / dwb / teb | 否（teb 需调参） | `nav:=` |
| C 障碍表示（**最接近维度体验**） | local: `scan`(2D 单线) / `cloud`(3D 点云直投) / `both`；global: `stvl`(**3D 体素**) / `scan` / `none` | 否 | `local_obstacle` / `global_obstacle` |
| D 维度 | 高度带(2.5D) → 坡度/台阶层(2.5D+) → 3D 图层 → 3D 规划器 | 前两项否，后两项是 | 参数 / `src/rm_navigation/rm_costmap_layers/` |
| E 感知 | 雷达-only → +相机（检测/瞄准） | 是（新包） | `src/rm_perception/rm_camera_detect/` |
| F 运动学 | 小陀螺 on/off、云台解耦 | 否 | `spin_speed` |

**关键洞察：想体验"3D/2.5D"其实大部分不需要新代码**
- **体验 3D 的最低成本入口 = `stvl_layer`**（`spatio_temporal_voxel_layer`，本就是 3D 体素→2D 投影），
  调 `voxel_decay` / `mark_threshold` / 高度带即可感受"带时间维的 3D 障碍表示"；
- 再加**已有工具**：`tools/scripts/mapping/pcd_to_grid_map.py` + `3d_to_2d_survey.md`（点云切层投影）⇒ 完整的 3D→2D 体验闭环；
- **体验 2.5D 的最低成本入口 = 高度带 + 坡度/台阶层**（costmap 图层接口现成）；
- **只有 3D 规划器**才需要真正写插件（且 nav2 插件契约是 2D，成本最高）⇒ 排最后。

**体验时间预算（按"先配置、后代码；先单轴、后组合"）**：

| 阶段 | 内容 | 预计 | 能体验到的 |
|---|---|---|---|
| 0 | P0 回归工具化 | 0.5 天 | （地基）单次体验 = 1 分钟 |
| 1 | 轴 C 全跑：`local_obstacle`×3、`global_obstacle`×3 | 1 天 | **2D 单线 vs 3D 点云 vs 3D 体素**的盲区/幽灵障碍/保守度差异 |
| 2 | 轴 B：rpp/dwb/teb 同目标点 | 1~2 天 | 控制器风格（平滑/激进/窄缝/震荡） |
| 3 | 轴 A：cartographer 纯定位 / slam_toolbox 定位 / ICP / AMCL | 2~3 天 | 重定位鲁棒性与收敛速度 |
| 4 | 轴 D：高度带 → 坡度层 → 3D 图层（STVL 已具） → 3D→2D 切层工具 | 3~5 天 | **从 2D 到 2.5D 到 3D 表示**的完整梯度 |
| 5 | 轴 E：相机数据流+标定 → 检测/跟踪 → （最后）回喂 costmap | 1 周+ | 视觉感知线；**契约先行**（内外参、时间戳） |
| 6 | 轴 F + 组合矩阵复跑 | 2 天 | 小陀螺/云台解耦；用 §二 的 120 组合做抽样 |

**每条路线的"看什么"（体验判据）**：贴墙 0.3 m 是否可见 / 45 cm 盲区是否被覆盖 / 幽灵障碍 / 窄缝通过 / 震荡幅度 /
重定位收敛时间 / 到达时间 / CPU / 是否需人工干预。**每体验一条就写一行到 §四 实测状态表**（这是 bench 的资产）。

**最后再上的两条**：① 3D 规划器插件（成本最高、契约冲突）；② 相机回喂 costmap（必须在标定与时间同步达标后，
否则就是新的"假数据"来源）。

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
| 2026-09-21 | 收紧 **cartographer 回环参数**（修"整张地图跟着车转 + 残影"）：实测 `map→odom` 被拧到 30.88° = 误回环。`max_constraint_distance 10→4`、`min_score 0.55→0.72`、`global_localization_min_score 0.6→0.8`、`fast_correlative_scan_matcher` 搜索窗 `5m/20°→2m/10°`、`global_constraint_search_after_n_seconds 10→30` |
| 2026-09-21 | 修 **全包形态静止漂移**：给 `lio:=cartographer` 接一路底盘里程计（lua `use_odometry=true` + `cartographer_sim.launch.py` 新增 `odom_topic` 参数，bringup 传 `/odom_ground_truth`）。实测不加时 `odom→base_link` 漂 ≈4 cm/s、13°/min（`map→odom` 恒定 → 非回环问题）；cartographer 只用 odom 增量，故世界系绝对位姿可直接喂 |
| 2026-09-21 | `tools/scripts/control/improved_teleop.sh` 键位改为**方向键**（↑↓ 前进后退 / ←→ 左右转 / `<` `>` 线速度 / `,` `.` 角速度 / 空格停 / q 退出，支持 `TELEOP_TOPIC` 覆盖话题）；键位表写进 runbook §1 |
| 2026-09-21 | 修**静默失效**：所有障碍源加 `expected_update_rate: 0.5`（原默认 0=不检查）→ 源停即 WARN + 拒绝算速度 + `velocity_timeout` 1s 停车；另修 `p2l` 的 `range_min: 0.45 → 0.2`（45cm 盲区会被反向清成 free）、`scan_time: 0.3333 → 0.1` |
| 2026-10-05 | §一 里程计行 + §四：登记新槽 `lio:=small_point_lio`（vendored `Yancey2023/small_point_lio@688d75c`，MIT）的**真实状态** —— **可跑通、契约合规，但精度未达标**（节点级重放轨迹 36.6 m vs `fast_lio` 参照 6.2 m）、**整栈未验证**、调参由另一任务进行中；同批记录 `lio:=pointlio` **从未全栈验证** + 同一 bag 上 25.3 m 未跟住（证据工具 `tools/lio_node_alone_check.py`；详见 `docs/lio_slots.md` §5、`docs/params_ownership_checklist.md` §4）|

---

## 九、重定位线结果记录（2026-10-05）

> 本轮只动重定位（`localization`）一线：AMCL 调参 → 终点检查器 → ICP 订阅 QoS → 新增 GICP 槽 →
> TF 盖戳契约 → 下采样口径 → 调度解耦 → 回归工具语义对齐。验收记录另见 `docs/localization_slots.md §7`。
>
> 两句话结论：**`localization:=gicp` 已可验收**（P0 回归 PASS / SUCCEEDED / 3.2 s / `recoveries=0`）；
> **`localization:=icp` 属"局部配准、初值敏感"** —— 无初值/初值差时会静默收敛到局部极小，
> believed pose 落进图内墙体 ⇒ 控制器冻死（28~68 s、19~23 次恢复、ABORT）；
> 手动 RViz 点目标（人会给合理初值/时机）正常。

### 本轮修复链（按提交顺序）

| 提交 | 做了什么 | 根因 / 实测 |
|---|---|---|
| `48703e6` | AMCL 高速跟踪参数：`transform_tolerance 1.0→0.3`、`update_min_d/a 0.25/0.2→0.05/0.05`、`recovery_alpha_slow/fast 0.0→0.001/0.1` | 根因：map→odom 被外推到未来 1 s、阈值触发导致更新被节流、随机重采样恢复被关闭；详见工单 §K |
| `47d7411` | 终点检查器 `SimpleGoalChecker → PositionGoalChecker` | 全向车没有"车头"概念 ⇒ 终点只约束位置；删掉该插件不存在的 `yaw_goal_tolerance` |
| `f033d96` | `icp_registration` 点云订阅 QoS → `SensorDataQoS` | 原来 RELIABLE 与 BEST_EFFORT 发布者不兼容 ⇒ 收不到点云 ⇒ 不发 map→odom ⇒ global_costmap 卡在 `Invalid frame ID "map"` |
| `28deaf1` | 新增 `localization:=gicp` 槽（GICP 精配准） | 初值来自 `/initialpose` 或 `initial_pose`；健康话题 `~/pose`、`~/fitness_score`、`~/converged` |
| `36a71cf`＋`c8863f1` | TF 盖戳契约：TF 与 `~/pose` 用 `now + tf_lookahead_sec`（= AMCL `transform_tolerance` 语义；0.3→0.45） | 根因：消费者请求 `now+0.1`（MPPI `FollowPath.transform_tolerance`），发布者若用 now 盖戳 ⇒ 最新条目永远旧 0.1 s ⇒ `ExtrapolationException` ⇒ `follow_path` 每周期 abort |
| `098078d` | 两级下采样（图 0.10 / 点云 0.05，照 COD 2025 `small_gicp_relocalization` 的 `global_leaf_size`/`registered_leaf_size` 配方）+ `max_correspondence_distance 1.0→1.5` | 实测：target 2438→**12450** 点，fitness **0.00976→0.00123 m²**，align **120→350 ms**，`/tf` **7→2 Hz** |
| `8b47ff9` | 调度解耦：`MultiThreadedExecutor` + TF/状态定时器独立 callback group（点云与 `/initialpose` 同组） | 实测（单节点配对对照）：`/tf` **2.3 Hz → 50.0 Hz**，fitness 不变；对照组证明 50 Hz 来自解耦而非迭代数。另：`maximum_iterations 32→16` 实测**不降 CPU**（1/4/16/32/64 → align 363/475/486/471/427 ms、fitness 全同） |
| `629c971` | 回归工具发目标语义对齐人工 RViz | `--yaw auto`（用 TF 链平面复合出的车当前朝向）、`--settle`（等 map→odom 稳定再发）、目标可达性预检（非 free ⇒ exit 3 且不发）、新增 JSON 字段 |

### 验收结论（`localization:=gicp`，commit `708b35d` 记于 `docs/localization_slots.md §7`）

| 判据 | 实测 | 结论 |
|---|---|---|
| `/tf` 速率 | **92~94 Hz（聚合）** = `map→odom` 满速 | ✅ 多线程解耦生效（改前 2 Hz 级） |
| `fitness_score` | **0.0023 m²**（RMS ≈4.8 cm） | ✅ 与单节点合成测 0.00123 同量级 |
| P0 回归 `--goal -1.0 2.0` | **PASS / SUCCEEDED / 3.2 s / `recoveries=0` / 轨迹 2.45 m** | ✅ 修复前同目标 28 s / 23 次恢复 / ABORT |
| 命令链 | `nav=(0.72,0.78)` → `smooth` → `chassis` **四跳一致**，`spin_speed=0.0` 直通 | ✅ 无 Spin/Backup 介入 |

### 四条"通了的关卡"（供复用）

| 关卡 | 做法 | 为什么 |
|---|---|---|
| **口径** | 两级 leaf（图 0.10 / 点云 0.05）+ `max_correspondence_distance 1.5` | 点数与对应关系同时够用；单调调一个参数会顾此失彼 |
| **盖戳** | TF 与位姿话题用 `now + tf_lookahead_sec` | 规则 = **消费端最大 tolerance + 发布周期 + 最坏掉帧余量**（AMCL 的 `transform_tolerance` 就是这个语义） |
| **调度** | `MultiThreadedExecutor` + TF/状态定时器独立 callback group | 单线程下 350 ms 的 align 会把 50 Hz TF 饿到 2.3 Hz |
| **初值** | 默认 `use_initial_pose:true` + `initial_pose [0,0,0]` ⇒ 开机即发 | 真正的"不发 TF"只在 `use_initial_pose:true` **且**无 `odom→base` TF **且**无 `/initialpose` 时出现（最后保护分支，不是常见路径） |

### 遗留 / 待办

1. 工具 settle 阈值偏严：静止 30 s 漂 3.4 cm / 0.0139 rad 略超 0.02 m / 0.01 rad ⇒ 应视为 **GICP 噪声底参考**（要消 WARNING 可放宽 `TH.settle_dxy→0.05` / `settle_dyaw→0.02`，一行可回退）。
2. `Control loop missed its desired rate of 30 Hz` = **总算力不足**（解耦只治"饿死"这一种，不治总量）。
3. `RMUC.pcd` 退化资产（0.10 m 体素后 8 点）⇒ `world:=RMUC` 用 icp/gicp 启动即报错（设计如此）。
4. `RMUL2026.pcd` 仅 53164 点 ⇒ 想让精度再上层须先有**更密的先验 PCD**（属建图侧）。
5. **三方对比（amcl/icp/gicp）待做** —— 协议见 §9.1（工具已就绪）。
6. 其他路线候选：`scan_context` 全局检索（解决"随便摆"）、Beluga AMCL、small_gicp（多线程实现）。

### 9.1 三方对比怎么跑

**同一条命令，只换 `localization:=`**；其余（world / 目标 / 规划器 / `spin_speed`）必须逐字一致，否则不可比。

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True \
  localization:=gicp          # ① 先 gicp，再 ② amcl，再 ③ icp：每次**重启整个栈**后跑这一条
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
```

三次跑完后（**快照按时间顺序排，先跑的在上**）：

```bash
# 快照不记录 localization ⇒ 方法列默认是 ?，用 --label 事后标注（时间戳数字取快照文件名里的那串）
python3 tools/scripts/regress/compare_regress_snapshots.py --last 3 --markdown \
  --label regress_<第1次时间戳>=gicp --label regress_<第2次时间戳>=amcl --label regress_<第3次时间戳>=icp

# 不想标注也能出表（方法列全为 ?，靠"时间顺序"自己对号）：
python3 tools/scripts/regress/compare_regress_snapshots.py --markdown
```

**纪律（否则表格会骗人）**：

- 换 `localization` **必须重启栈**（`map→odom` 只能有一个发布者，同一次运行里比不了）；
- 三次都用同一条 `--goal -1.0 2.0`，`spin_speed` 固定 0.0，`--settle` 用默认值（工具语义 = 人在 RViz 里等定位稳了再点目标）；
- `icp` 那次若"启动就不动"，先看它有没有初值：**局部配准 + 初值差 = 静默局部极小**，这本身就是对比结论的一条，不要当成工具故障；
- 把工具输出的表贴回本节（或 `§四` 加一行），并写清每次的 `result/用时/recoveries`。

### 9.2 新构建口径下的五路对照（2026-10-05）

> 口径：**Release 构建**（`6ae2ada` 的 `colcon_defaults.yaml`）+ gicp `backend` 开关（`d60c21e`）
> + `/initialpose` 延迟修复（`c7bcd6d`）之后，第一次把五路定位放在**同一条命令、同一个目标**下
> 并排跑完。全程 **headless**（`nav_rviz:=False`，无 gzclient / 无 RViz 窗口）。
>
> **一句话结论：五路全 PASS**（5/5 `SUCCEEDED`、`recoveries=0`、`d_min=0.000 m`）；
> `Control loop missed its desired rate` **5 次全 0**（§9 遗留第 2 条"总算力不足"在本口径下**没复现**）；
> gicp 两个后端**精度同级**（fitness 0.00226 / 0.00230 m²）、**small_gicp 的 align 快 5.4×**
> （中位数 15.7 → 2.9 ms）。

#### 9.2.1 口径与可比性前提

| 项 | 值 / 证据 |
|---|---|
| 构建口径 | `build/gicp_registration/cmake_args.last` = `['-DCMAKE_BUILD_TYPE=Release', '-DAMENT_CMAKE_SYMLINK_INSTALL=1']`；`build/gicp_registration/CMakeCache.txt`：`CMAKE_BUILD_TYPE=Release`、`CMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG` |
| 「Release 真生效」的独立佐证 | gicp `[status]` 的 `align=` **13~17 ms**（PCL 后端）—— 同数据 `-O0` 时代是 370~480 ms（`docs/build_optimization.md` §2.2 台架值 369.9 → 11.3 ms） |
| launch 参数（5 次逐字一致） | `world:=RMUL2026 mode:=nav lio:=fastlio nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False localization:=<五选一>` |
| 回归命令（5 次逐字一致） | `python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization <NAME>`（工具默认 `--settle 3.0`、`--yaw auto` ⇒ 与 §9.1 协议一致） |
| launch 文件版本（5 次同一份） | `src/rm_nav_bringup/launch/bringup_sim.launch.py` 的 mtime = 13:52:54，**早于**首次起栈 13:55:34，且窗口内未再改动 ⇒ 5 次用的是同一版本（该份已含并行 agent 新增的 `lio:=small_point_lio` 槽**选项**；本测量一律走 `lio:=fastlio`，不受影响） |
| 串行纪律 | 5 次**各自重启整个栈**；逐份 launch 日志核对：每次只启动**一个**定位进程（gicp 两次 = `gicp_registration_node`，amcl = `amcl`，beluga = `amcl_node`，icp = `icp_registration_node`）⇒ 同一时刻只有一个 `map→odom` 发布者 |
| 收尾 | 每次向 launch 的**进程组**发 SIGINT → 等 20 s → SIGKILL 兜底 → `pgrep -af 'gzserver\|gzclient\|ros2 launch\|nav2\|fastlio'` 复查**无残留**；每次 SIGINT 的时刻都落在该次回归结束**之后 ≤1 s**（见 9.2.4），即所有进程退出/重启都发生在测量窗口之外 |
| 机器负载 | loadavg(1min) 2.57~4.64（28 核）；5 次期间**没有任何 colcon 编译**（`log/latest_build/events.log` 全程未被写：阶段 0 记录的"未写秒数"从 123 s 单调增到 405 s） |

⚠️ **一个只属于本次沙箱的适配（用户本机正常跑不需要）**：本次是在 `$HOME` **只读**的沙箱里跑的，
而 `gzserver` 启动时必定 `create_directory($HOME/.gazebo/server-<port>)` ⇒ 抛
`boost::filesystem::filesystem_error` 并 **SIGABRT**（症状是连 `/clock` 都没有、整个栈像没起来）。
实测 Gazebo 读的是 **`$HOME` 环境变量**（不是 `getpwuid`）⇒ 把 `HOME` 指到可写的
`/tmp/gzhome-<tag>` 后一切正常。另外 `ROS_DOMAIN_ID=77`（隔离另一个 agent 的栈）、
`GAZEBO_MASTER_URI=http://127.0.0.1:11399`（避开默认 11345 端口）、`unset DISPLAY`（让 gzclient
直接退出 = 无 GUI）也是本次测量用的隔离手段。

#### 9.2.2 五路并排表（`compare_regress_snapshots.py --last 5 --markdown` 原样输出）

方法列**不再需要 `--label` 人工标注**：`nav_smoke_regression.py` 现在把 `--localization` 写进快照
（工具脚注自动报「方法来源：字段 localization」）。两个 gicp 的槽位名相同（都是 `gicp`），
区分靠 `backend`（见下条），故本表用 `--label` 把方法列显式写成 `gicp (pcl)` / `gicp (small_gicp)`
（**只改表头显示，快照原始内容未改**）：

| 时间戳/文件 | 方法 | goal | pass | result/exit | status | 用时(s) | d_min(m) | rec | nav\|v\|,\|w\| | smooth | chassis | poseΔ(m) | 真值twist | settle(settled/xy/yaw) | map | yaw源 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10-05 13:56 regress_1791179769.json | gicp (pcl) | (-1.00,2.00) | ✅ | pass/0 | SUCCEEDED | 3.3 | 0.000 | 0 | (1.46,0.87) | (1.46,0.87) | (1.46,0.87) | 2.802 | 1.456 | ✅ 0.0077/0.0032 | free | tf |
| 10-05 13:57 regress_1791179865.json | gicp (small_gicp) | (-1.00,2.00) | ✅ | pass/0 | SUCCEEDED | 3.4 | 0.000 | 0 | (1.42,0.82) | (1.41,0.71) | (1.41,0.71) | 2.782 | 1.395 | ✅ 0.0062/0.0025 | free | tf |
| 10-05 13:58 regress_1791179928.json | amcl | (-1.00,2.00) | ✅ | pass/0 | SUCCEEDED | 3.2 | 0.000 | 0 | (1.45,0.83) | (1.45,0.83) | (1.45,0.83) | 2.730 | 1.449 | ✅ 0.0000/0.0000 | free | tf |
| 10-05 13:59 regress_1791179989.json | beluga | (-1.00,2.00) | ✅ | pass/0 | SUCCEEDED | 3.3 | 0.000 | 0 | (1.40,0.80) | (1.40,0.74) | (1.40,0.74) | 2.767 | 1.400 | ✅ 0.0000/0.0000 | free | tf |
| 10-05 14:00 regress_1791180051.json | icp | (-1.00,2.00) | ✅ | pass/0 | SUCCEEDED | 3.2 | 0.000 | 0 | (1.45,0.91) | (1.45,0.91) | (1.45,0.91) | 2.657 | 1.450 | ✅ 0.0000/0.0000 | free | tf |

（不外挂 `--label` 时，头两行的方法列都是 `gicp`；其余三行同上。`共 5 个快照：PASS 5 / FAIL 0`。）

#### 9.2.3 工具表里没有、但本次同口径采到的量

`RTF`、`missed` 由工具与 launch 日志给出，`align`/`采纳` 由 gicp 的 `[status]` 行给出：

| 配置 | RTF | `Control loop missed its desired rate` 条数 | gicp align 中位数（min~max） | gicp `采纳 N/M`（末帧） | fitness（末帧） | `map→odom`（末次 `[status]`） |
|---|---|---|---|---|---|---|
| `localization:=gicp`，`backend: pcl` | **0.760** | **0** | **15.7 ms**（11.8~25.4） | **165/165（100%）** | 0.00226 m² | x=0.001 y=0.016 z=-0.046 yaw=-0.06° |
| `localization:=gicp`，`backend: small_gicp` | **0.776** | **0** | **2.9 ms**（2.6~4.0） | **167/167（100%）** | 0.00230 m² | x=0.011 y=0.008 z=-0.015 yaw=-0.00° |
| `localization:=amcl` | **0.780** | **0** | —（该槽无 `[status]`） | — | — | — |
| `localization:=beluga` | **0.769** | **0** | — | — | — | — |
| `localization:=icp` | **0.776** | **0** | — | — | — | — |

补充量（同一批快照）：

| 配置 | hz pcloud/imu/scan/odom | fp 戳数 | tf_age(s) | 目标落格 | 载入目标 PCD |
|---|---|---|---|---|---|
| gicp (pcl) | 7.7 / 79.0 / 8.0 / 7.7 | 110 | 0.10 | free (23,102) | 12450 点（leaf 0.100） |
| gicp (small_gicp) | 8.0 / 77.7 / 8.0 / 7.7 | — | 0.10 | free | 12450 点 |
| amcl | 7.7 / 78.7 / 8.0 / 8.0 | — | 0.10 | free | — |
| beluga | 8.0 / 77.3 / 8.0 / 7.7 | — | 0.10 | free | — |
| icp | 8.0 / 79.3 / 8.0 / 7.7 | — | 0.10 | free | — |

#### 9.2.4 逐路备注

1. **gicp / `backend: pcl`（13:55:34 起栈，13:56:09 回归 PASS）** —— `settle` 3.0 s、窗口漂移
   0.0077 m / 0.0032 rad（阈值 0.02 m / 0.01 rad，**过**）；`nav=smooth=chassis=(1.46,0.87)`
   四跳一致 ⇒ `spin_speed=0.0` 直通；真值位移 2.802 m；missed **0**；align 中位数 **15.7 ms**。
   与 `docs/build_optimization.md` §2.2 台架的 11.3 ms 同量级（节点里还有下采样/评分/日志开销）。
2. **`localization:=small_gicp`（13:57:11 起栈，13:57:45 PASS）** —— 唯一差别是槽位换成
   `small_gicp`：**`small_gicp` 自成一个 `localization` 值，`backend: "small_gicp"` 由 launch
   自己注入 ⇒ 不改任何 YAML、也不用 `git checkout`**（`gicp` 槽位不注入 ⇒ 保持节点默认 `pcl`；
   见 `docs/localization_slots.md` §1 的"一槽位一行"表 + 实现细节脚注）。
   （历史：2026-10-05 当时还没有这个槽位，测量时是把
   `src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml` 的 `backend: "pcl"`
   临时改成 `"small_gicp"`（`--symlink-install` ⇒ 免编译即生效）、跑完 `git checkout` 还原，
   当时快照记录的方法也是 `gicp` —— 两条路的代码路径完全相同，故下面数字仍然有效；
   该做法已由 `2c42339`（2026-10-05 同日）简化。）accuracy 同级（0.00230 vs 0.00226 m²）、
   align 中位数 **15.7 → 2.9 ms（5.4×）**，与台架的 11.3 → 2.3 ms（4.9×）一致 ⇒ **两者互相印证**。
   注意 `smooth/chassis` 的 |v|,|w| 是 (1.41,0.71) 而 `nav` 是 (1.42,0.82)：**速度平滑器限幅**，
   不是命令链断（四跳仍逐级贯通，真值 twist 1.395 / 位移 2.782 m）。
3. **amcl（13:58:15 起栈，13:58:48 PASS）** —— `settle` 漂移 **0.0000 / 0.0000**（静止时
   AMCL 的 `map→odom` 是分段常量，本来就不抖）；`nav=smooth=chassis=(1.45,0.83)` 四跳完全一致；
   `recoveries=0`、用时 3.2 s。
4. **beluga（13:59:16 起栈，13:59:49 PASS）** —— 同 AMCL：漂移 0.0000 / 0.0000、用时 3.3 s。
   `beluga_amcl` 的 `amcl_node` 在**收尾 SIGINT 时 SIGABRT(-6)**（退出期崩溃，测量窗口内一直正常 ——
   它的 `map→odom`、目标执行、真值位移都成立）。这条记下来是为了下次看到别误判为"启动即崩"
   （`c655d77` 修的那个是**启动**崩溃，症状完全不同）。
5. **icp（14:00:16 起栈，14:00:51 PASS）—— 本次没复现 §9 记录的失败**：`recoveries=0`、
   用时 3.2 s、真值位移 2.657 m。原因是**初值口径**：`RMUL2026` 的 `map` 系 = 出生点相对系，
   ICP 槽默认 `initial_pose [0,0,0]` 恰好是真值 ⇒ 局部配准站在"初值正确"的有利面上。
   §9 里"icp 静默收敛到局部极小/被搬动或初值差"的结论**没有被本次证伪**，只是本次没踩到；
   要复现那条结论，必须用**差的初值**（RViz 里乱点 `/initialpose` 或改 `initial_pose`）再跑一次。

**所有 5 次的共同点**：`yaw_source=tf`（目标朝向 = 车当前朝向，人工 RViz 语义）、目标落格
`free`、`d_min=0.000 m`、`recoveries=0`、`tf_age=0.10 s`、`/clock` RTF 0.76~0.78。

#### 9.2.5 与 §9「遗留 / 待办」的对照（本次更新了哪几条）

| §9 遗留条目 | 本次结果 |
|---|---|
| 5. 三方对比（amcl/icp/gicp）待做 | ✅ 本次做完，并扩成**五路**（gicp 两个 backend + amcl + beluga + icp），见 9.2.2/9.2.3 |
| 2. `Control loop missed its desired rate of 30 Hz` = 总算力不足 | ⚠️ **本口径下未复现**：5 次全 0。与 Release 构建（PCL align 370→13 ms）、headless（无 gzclient/RViz 渲染）、无并发编译三者一致；**不等于**"总算力永远够"——把 GUI、RViz、并发编译加回来仍可能复现 |
| 1. 工具 settle 阈值偏严（GICP 静止漂 3.4 cm / 0.0139 rad） | ⚠️ 本次 gicp 两次都**过**（0.0077 / 0.0032 与 0.0062 / 0.0025），未触发该阈值 ⇒ 阈值暂不用放宽 |
| 6. 候选路线：small_gicp | ✅ 已可用且实测更快（9.2.3），默认仍是 `pcl`（保持既有 fitness 阈值标定） |
| 3. `RMUC.pcd` 退化资产 ⇒ icp/gicp 启动即报错 | 未测（本次只用 `world:=RMUL2026`） |
| 4. `RMUL2026.pcd` 仅 53164 点 | 未变（两次 gicp 都是 target 12450 点 @ leaf 0.10） |

#### 9.2.6 复现命令（copy-paste）

```bash
# 0) 构建口径：必须在**仓库根**执行（colcon 只从当前目录读 colcon_defaults.yaml）
cd /home/weicheng/HzMi_rmsimulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gicp_registration
cat build/gicp_registration/cmake_args.last     # 必须含 -DCMAKE_BUILD_TYPE=Release

# 1) 逐路跑（每一路都：起栈 → 回归 → 收尾，再起下一路；5 路只换 localization:= 一个词）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False localization:=gicp
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization gicp

# ① → ② gicp 换 small_gicp 后端：**只换槽位**（launch 自己注入 backend: "small_gicp" ——
#     不用改 YAML、不用 -p backend:=...、也不用 git checkout；见 docs/localization_slots.md §1）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False localization:=small_gicp
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization small_gicp
# ⚠️ 2026-10-05 那次记录的快照是 --localization gicp（当时还没有 small_gicp 槽位，靠临时改 YAML 的
#    backend 切换 ⇒ 快照 localization=gicp）；同一份表里的这个槽位名以当时的记录为准，今天重跑用上面这行
# ③ amcl / ④ beluga / ⑤ icp：只把 localization:= 换成 amcl / beluga / icp

# 2) 出表（--localization 已进快照 ⇒ 方法列自动填；本批两个 gicp 行的 localization 都记成 `gicp`，
#    故用 --label 把方法列显式区分 backend）
python3 tools/scripts/regress/compare_regress_snapshots.py --last 5 --markdown \
  --label regress_1791179769='gicp (pcl)' --label regress_1791179865='gicp (small_gicp)'
python3 tools/scripts/regress/compare_regress_snapshots.py --last 5 --markdown   # 不标注也能出表（头两行都显 gicp）
```

#### 9.2.7 回滚 / 清理

- **本节的代码改动只有一处**（独立提交）：`nav_smoke_regression.py` 新增 `--localization`
  —— 回滚 = `git revert <该提交>`；不传该参数时行为与旧版**逐字节相同**（快照不写
  `localization` 键）。
- **测量用的临时脚本/日志**都在 `.tmp_cache/five_way/`（未跟踪）：`run_one.sh`（单次测量驱动）、
  `collect_one.sh`（只重算汇总）、`<tag>.log` / `<tag>.regress.log` / `<tag>.metrics.json`、
  `compare_last5*.txt`；`probe.log` / `probe2.sh` 是 headless 可行性探针（`$HOME` 只读导致
  gzserver SIGABRT 的现场记录）。
- **配置没有被永久改动**：`backend` 临时改成 `small_gicp` 后已在同一次调用里 `git checkout` 还原
  （`git status` 干净）；快照写入的是既有的 `.tmp_bags/`（未跟踪）。**该临时改法已不需要**（`2c42339`
  起 `localization:=small_gicp` 即可，见 9.2.4 第 2 条），这里只是当时那次测量的现场记录。
- **进程**：每次测量结束都 SIGINT→20 s→SIGKILL 进程组并 `pgrep` 复查无残留；
  沙箱本身用独立 PID namespace（调用一结束，该次的所有进程由内核清掉）⇒ 不会留下 gzserver/nav2。
