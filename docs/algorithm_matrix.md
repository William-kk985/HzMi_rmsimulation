# 算法组合总表（当前能力矩阵）

> 本文件是 bench 的"当前有什么、能怎么组合、哪些能用"的单一事实来源。
> 每跑通一个组合就往 §四 填一行实测结果；新增槽位就往 §一/§六 加。
> 设计原则：**目录=技术域；启动参数=角色槽位**；算法参数各自回归所属包的 `config/`（R1）。

---

## 一、角色槽位与实现（`bringup_sim.launch.py`）

| 槽位 | 参数 | 现有实现 | 对应包 / 节点 | 生效条件 |
|---|---|---|---|---|
| **场景形态** | `mode` | `mapping` 纯建图 / `slam_nav` 边建边导 / `nav` 先建后导 | — | 必填 |
| **里程计** | `lio` | `fastlio` / `pointlio` / `none` | `src/rm_localization/FAST_LIO`、`point_lio`；`none` 需外部提供 odom/TF | 全形态 |
| **在线建图** | `mapper` | `slam_toolbox` / `cartographer` | `src/rm_localization/slam_toolbox`（async）、`cartographer_ros`（+ `cartographer_occupancy_grid_node`） | `mapping` / `slam_nav` |
| **重定位** | `localization` | `amcl` / `slam_toolbox`(纯定位) / `icp` | `nav2_amcl`(+`map_server`)、`slam_toolbox`(localization)、`src/rm_localization/icp_registration` | 仅 `nav` |
| **局部规划器** | `nav` | `rpp` / `dwb` / `teb` | `nav2_regulated_pure_pursuit_controller` / `nav2_dwb_controller` / `teb_local_planner`（+ `costmap_converter`） | `nav` / `slam_nav` |
| **全局规划器** | —（固定） | `NavfnPlanner` | `nav2_navfn_planner` | 同上 |
| **场地** | `world` | `RMUC` / `RMUL` / `RMUL2026` | `hzmi_rm_simulation` 世界 + `map/<world>.*` + `PCD/<world>.pcd` | 全形态 |
| **小陀螺** | `spin_speed` | `5.0`（哨兵语义）/ `0.0`（角速度直通，排查用） | `fake_vel_transform` | 全形态 |
| 可视化 | `lio_rviz` / `nav_rviz` | True/False | `fastlio.rviz` / `pointlio.rviz` / `nav2.rviz` | 全形态 |

## 一.1 维度归类：哪些模块是 2D / 2.5D / 3D（**按域分文件夹，按维度做标注**）

**结论先说：不建议按 2D/3D 物理重组目录。**

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
| （第三方 apt）`spatio_temporal_voxel_layer` | 导航 | **2.5D** | 3D 体素 → 2D 代价投影 | 唯一现成的 2.5D 环节 |
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

## 二、组合数量（核心组合 = 102）

| 形态 | 组合数 | 算式 |
|---|---|---|
| `mapping` | **12** | 3 场地 × 2 LIO × 2 mapper |
| `slam_nav` | **36** | 3 × 2 × 2 × 3 局部规划器 |
| `nav` | **54** | 3 × 2 × 3 重定位 × 3 局部规划器 |
| **合计** | **102** | 不含 `spin_speed`/`*_rviz` 等开关（加上会 ×8，但它们不是算法） |

另有特殊用法：`lio:=none`（需外部 odom/TF，例如轮式里程计或纯 2D 组合）、`mode:=nav localization:=''`（LIO 当绝对定位的静态桥回退用法）。

## 三、资产可用性矩阵（决定组合"能不能真跑"）

| 资产（`src/rm_nav_bringup/`） | RMUC | RMUL | RMUL2026 | 谁消费 |
|---|---|---|---|---|
| `map/<w>.pgm` + `.yaml` | ✅ 577×301 | ✅ 272×210 | ⚠️ 240×169（**含幽灵墙**，见 §五） | `localization:=amcl`（经 `map_server`） |
| `map/<w>.posegraph` | ✅ 13.7 MB | ✅ 13.8 MB | ❌ **缺** | `localization:=slam_toolbox` |
| `PCD/<w>.pcd` | ✅ 18 MB | ✅ 50 MB | ❌ **缺**（可用 `/map_save` 现场生成） | `localization:=icp` |
| `map/<w>.pbstream` | ❌ 缺 | ❌ 缺 | ⚠️ **526 B 空壳** | cartographer 纯定位 |
| `map/<w>.data` | ✅ 8.5 MB | ✅ 4.2 MB | ❌ | 当前流程**未使用**（遗留资产） |

**推论（当前的"硬约束"）**：
- **AMCL 路线三个场地都能跑**（RMUL2026 的图需先重建，否则发目标会规划失败）；
- **slam_toolbox 纯定位 / ICP 只能 RMUC、RMUL**；
- **cartographer 纯定位目前无场地可用**（三个 pbstream 都不存在/为空）→ 必须先 `tools/scripts/mapping/generate_cartographer_pbstream.sh` 生成；
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
| **无可用 pbstream** | cartographer 纯定位不可用 | 用 `generate_cartographer_pbstream.sh` 生成 |
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
