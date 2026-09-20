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
