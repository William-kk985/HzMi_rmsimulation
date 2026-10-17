# Livox（非重复扫描）雷达在新 Gazebo 上的既有实现 —— 调研与「复用 / 适配 / 自研」裁决

> **状态**：调研完成（2026-10-18）。**只读代码、只查资料，未编译、未运行、未改任何功能代码。**
> **回答的问题**：用户原话「有人这么干吗，自研的这个普便性大吗」。
> **上位约束与前置结论**看 [`gazebo_sim_migration_plan.md`](gazebo_sim_migration_plan.md)（Humble↔Fortress 配对、`gpu_lidar` 不支持任意图案、无 GPU 裁决、R0/R1/R3 四方案决策表、我们自己的 Bullet BVH 实测）。

---

## §0 结论速览（TL;DR）

**三句话：**

1. **有人干过，而且不止一个。** 2025-04 ～ 2026-10 之间至少有 **6 个** 把 Livox 图案搬进**新 Gazebo**（`ign gazebo` / `gz sim`）的独立实现，其中 **2 个明确跑在 Fortress 上**，**2 个是 MIT / Apache-2.0 可自由复用**。
2. **最接近我们的那个几乎是现成的 R1**：`ashduwihch/mid360_gazebo_harmonic`（**MIT**）= `gz::sim::System` + **rmagine/Embree CPU BVH 批量求交** + **MID-360 CSV 图案** + 逐点 `offset_time/tag/line/reflectivity` + 直接发 `PointCloud2`/`CustomMsg`/`IMU`；**唯一硬缺口是它钉在 Harmonic（`gz-sim8`）**，Fortress 化在源码层只需 **3 处**（CMake 包名、插件注册宏、编掉 `RaycastData` 那条后端）。
3. **自研的"普遍性"很大——但正因为大，已经被人写完了。** CPU 批量射线求交当 LiDAR 仿真器是**主流做法**（PyBullet `rayTestBatch`、rmagine 的 ICRA 2023 论文、上游 Jetty 官方 `cpu_lidar` 都是这个思路），所以 R1 的**架构**没有新意、**不必重新发明**；我们真正独有、也真正需要自己把关的只有三件事：**Fortress 版本钉子**、**契约对齐**（frame/话题/字段/自机排除）、**30k@10 Hz 的实测验收**。

**⇒ 裁决：把 R1 从「写」降级为「移植 + 适配」**（工作量 **S–M**，见 §4）。**首选 A 方案**：以 `ashduwihch` 的 MIT 插件为骨架 + 上游 `uos/rmagine`（BSD-3，**README 明示 Humble+Fortress 已测**）作为 BVH 地基。

> ⚠️ **一个必须同时说清的坏消息**：**没有一个候选是"Fortress + 任意图案 + 现成可跑"三者齐备的**。
> Fortress 的 2 个（`DHA-Tappuri`、`jongbob1918`）分别是 **GPU 过采样重采样** 和 **专有许可 + 只支持 box/plane**；
> 纯 CPU BVH 的 2 个（`ashduwihch`、`rmagine_gazebo_plugins`）都对准 **Harmonic**（`rmagine_gazebo_plugins` 是双支持）。
> 所以「有没有现成的」答案是 **"有 90% 现成的"**，不是「有 100% 现成的」。

---

## §0.1 七条硬事实（决定一切，均可一手复核）

| # | 事实 | 证据（一手） | 对我们的意义 |
|---|---|---|---|
| F1 | **Fortress 没有 `components/RaycastData`**（`gz-sim8/9/10` 才有） | 本机 `/usr/include/ignition/gazebo6/gz/sim/components/` 下 `ls \| grep -i ray` **为空**；上游 `raw.githubusercontent.com/gazebosim/gz-sim/{gz-sim6,gz-sim7}/…/RaycastData.hh` = **404**，`{gz-sim8,gz-sim9,gz-sim10}` = **200** | Jetty 那条「把射线塞进 ECM、让 `gz-physics` 批量求交」的路在 Fortress **不存在** ⇒ **BVH 必须自己持有**（rmagine/Embree、Bullet）或退回渲染器 |
| F2 | **Jetty 的 `cpu_lidar` 自己不做求交** | `gz-sim10/src/systems/cpu_lidar/CpuLidar.cc:34` 引入 `components/RaycastData.hh`；`:71-79` 建 `RaycastData`+`NeedsRaycast`；`:217-242` 从 `_raycastData->Data().results` 读结果 | 它只是「**规则网格生成射线 + 交给物理引擎**」，**不是**可移植的 BVH 实现 ⇒ 只能当架构参考 |
| F3 | **`GenerateRays()` 不是 `virtual`**，`unitVectors` 是私有 `dataPtr` | `gz-sensors10/include/gz/sensors/CpuLidarSensor.hh:116-117` | 即便升到 gz-sensors10，子类也换不了图案（我们迁移计划 §3.4 的结论仍然成立） |
| F4 | **Fortress 支持 `<sensor type="custom">`** | `ign-gazebo6/src/SdfEntityCreator.cc:1022` 处理 `sdf::SensorType::CUSTOM` → 建 `components::CustomSensor`；本机 `/usr/include/ignition/gazebo6/gz/sim/components/CustomSensor.hh` 存在 | rmagine 那套「custom sensor 工厂」在 Fortress **可用**（这条我之前在迁移计划里没核实，现在核实了） |
| F5 | **Fortress 的 `gz/plugin/Register.hh` 只有 `IGNITION_ADD_PLUGIN`，没有 `GZ_ADD_PLUGIN`** | 本机 `/usr/include/ignition/plugin1/gz/plugin/Register.hh:53` 只定义 `IGNITION_ADD_PLUGIN` | **Harmonic 代码搬到 Fortress 唯一必改的 C++ 层就是注册宏**（1 个 `#ifdef`），其余 `gz::sim`/`gz::math`/`<gz/...>` 路径 Fortress 全有 |
| F6 | **`ros_gz_bridge`（Humble/Fortress 0.244.26）支持 `PointCloudPacked` ↔ `PointCloud2`** | 本机 `strings /opt/ros/humble/lib/libros_gz_bridge.so` → `Factory<sensor_msgs::msg::PointCloud2, ignition::msgs::PointCloudPacked>` | 走「gz 侧发 `PointCloudPacked` + 桥接」的路线**不用自己写桥**（但 `CustomMsg` 永远过不了 `ros_gz_bridge`，只能进程内发，见 §3.1） |
| F7 | **我们的 CSV 与候选的 CSV 是同一份数据、同一套方向公式** | 我们 `scan_mode/mid360.csv` 表头 `Time/s,Azimuth/deg,Zenith/deg`，`zenith∈[37.836, 97.2123]`；`ashduwihch` 的 `mid360-real-centr.csv` **同一范围**；我们 `livox_points_plugin.cpp:38-39` 做 `zenith−π/2` 后再 `Euler(0,zenith,azimuth)·X̂`，与 `ashduwihch/csv_scan_pattern.hpp:43-50` 的 `(sin z·cos a, sin z·sin a, cos z)` **数值等价**（我用 ignition-math6 编译对拍过） | **图案层零改动**：把 `<csv_file>` 指到我们自己的 CSV 即可；`(37.8…97.2)` 正好是 MID-360 的 `−7°…+52°` 俯仰（`90−ze`） |

---

## §1 我们的约束（一屏，用于对照表）

| 维度 | 我们的现状 |
|---|---|
| 目标栈 | ROS 2 **Humble** + `ros_gz` **0.244.26**（≡ **Fortress / `ignition-gazebo6`**，本机已装） |
| 传感器需求 | **30,000 条/帧、10 Hz、任意逐射线图案**（`mid360.csv` 800,001 行，滚动窗口 30k/帧 ⇒ 26.7 帧一循环） |
| 世界几何 | `RMUC2026_world`：**visual 和 collision 都是同一个 `RMUC2026.stl` mesh**（≈45,900 三角面，mm 单位、scale 0.001） |
| 下游契约 | `/livox/lidar` = `livox_ros_driver2/msg/CustomMsg`（FAST-LIO 吃这个）；`/livox/lidar/pointcloud` = `PointCloud2`（我们发 **xyz + rgb**）；`/livox/imu` = 100 Hz |
| 现状痛点 | Classic 里 **每条射线一个 ODE collision** ⇒ 30k 射线 = 2.74 GiB、gzserver 3.2 GiB、RTF 0.32–0.46 |
| 我们已有的实测 | 纯 CPU BVH（Bullet `btBvhTriangleMeshShape` + 真 CSV）：**30k 射线 12.5–29.4 ms（1 线程）/ 5.6–13.6 ms（4）/ 3.2–7.4 ms（8）** ⇒ 100 ms 预算够宽 |

---

## §2 候选总表

> 观察时间 **2026-10-18**。证据等级：**A**=读过实现源码；**B**=读过 README/CMake/SDF；**C**=只有搜索摘要。
> "任意图案"= 能否表达 **逐射线任意方向**（不是规则网格）。

| # | 候选 | 许可 | 最后活动 | 目标 Gazebo | 传感器 API / 求交方式 | 任意图案 | 性能说辞 | Fortress+Humble 直接可用 | 证据 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | [`uos/rmagine`](https://github.com/uos/rmagine) + [`uos/rmagine_gazebo_plugins`](https://github.com/uos/rmagine_gazebo_plugins) | **BSD-3** | 2026-08-06（76★/17★） | **Harmonic + Fortress（README 明示都测过）** | `gz::sim::System` 成对（map system 建 BVH + sensor system 打射线）；后端 **Embree(CPU)** / OptiX(GPU) / Vulkan(GPU) | ✅ **O1Dn/OnDn**：`<rays_file>` YAML 里给 `width×height` 个 `dirs`（任意方向，单一/逐条原点） | README：Embree「低端硬件也顺」；示例 100 台 VLP-16 批量；TBB `parallel_for` 多线程 | **✅ 是**（但需自己写 Livox 图案与 `CustomMsg` 那层，见 #2） | **A** |
| 2 | [`ashduwihch/mid360_gazebo_harmonic`](https://github.com/ashduwihch/mid360_gazebo_harmonic) | **MIT**（内置 rmagine = BSD-3） | 2026-10-13（18★） | **Harmonic（`gz-sim8`，CMake 硬钉）** | 模型级 `gz::sim::System` + **rmagine `O1DnSimulatorEmbree`**（CPU BVH 批量）；另有 `ray_backend=physics`（需 `RaycastData`，Harmonic+） | ✅ **读 CSV**（`time,az,ze`，与我们同一份数据/同一套公式） | 200,000 点/秒、10 Hz、**20,000 射线/帧**；代码里每 100 帧打印 **Embree 平均 ms** | **🟡 差 3 处编译级改动 + 契约对齐**（见 §5） | **A** |
| 3 | [`FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor`](https://github.com/FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor) | **Apache-2.0** | 2026-10-11（1★） | **Harmonic（`gz-sensors8`/`gz-sim8`）** | 自定义 **`gz::sensors::Sensor`** + `components::RaycastData`（**求交交给物理引擎**） | ✅ 读 CSV（自带 7 种 Livox `scan_mode/*.csv`） | 未声明 | ❌ **不可**（`RaycastData` 在 gz-sim6 不存在 ⇒ 整条路线不成立） | **A** |
| 4 | [`jongbob1918/livox_mid360_gz_sim_ros2`](https://github.com/jongbob1918/livox_mid360_gz_sim_ros2) | **Proprietary（明确禁止复制/再分发）** | 2026-07-16（1★） | **Fortress（README 表：Ubuntu 22.04 + Humble + `ign gazebo`）** | 模型级 `gz::sim::System` + **自己实现的暴力求交**（遍历 ECM 里的 box/plane） | ✅ 读 CSV（滚动窗口，20000/帧） | 20000 点 CPU 求交「复杂世界可能到不了 10 Hz」 | **✅ 技术上是**，❌ **许可上不是** | **A** |
| 5 | [`DHA-Tappuri/livox_ignition`](https://github.com/DHA-Tappuri/livox_ignition) | **无 LICENSE（默认全保留）** | 2025-04-03（1★） | **Fortress（`ignition-gazebo6`，CMake 逐字）** | **GPU `gpu_lidar` 过采样 1440×1440（≈2.07M 条）→ 按 CSV 方向查最近邻索引重采样** + gz protobuf 自定义消息 + ROS 2 桥节点 | 🟡 只能「近似」（角度量化 = 360/1440 = 0.25°） | 未声明 | 🟡 是（Fortress），但**不是 CPU、精度受网格限制** | **A** |
| 6 | [`acodeiro-dev/lidar-livox-mid360`](https://github.com/acodeiro-dev/lidar-livox-mid360) | **无 LICENSE** | 2026-10-13（0★） | Harmonic | `gpu_lidar` + `ros_gz_bridge` + 自写 `livox_converter` 节点转 `CustomMsg` | ❌（规则网格 + 事后转格式） | 未声明 | ❌ 不是 CPU 路线 | **B** |
| 7 | Classic 家族：[`Livox-SDK/livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation)（**MIT**，348★，2023-11）→ [`LCAS/livox_laser_simulation_ros2`](https://github.com/LCAS/livox_laser_simulation_ros2)（MIT，13★，2025-05）→ `CMU-cabot` fork（MIT，1★，2025-01） | **MIT** | 见左 | **Gazebo Classic** | 每条射线一个 ODE collision（= 我们 2.74 GiB 的根源） | ✅（就是我们的来源） | —— | ❌ 不能直接用；**但 CSV 格式/方向公式/`CustomMsg` 语义是真值来源** | **A** |
| 8 | 上游 Jetty `gz-sim10` 的 `cpu_lidar` | Apache-2.0 | PR #3343 合入 **2026-05-20** | **gz-sim10 only** | `CpuLidar` system + `CpuLidarSensor`，**求交交给 `gz-physics`（`RaycastData`）** | ❌ **规则网格**，`GenerateRays()` 非虚 | 号称「优化过的物理射线」 | ❌ **不可移植**（F1/F2/F3） | **A** |

**旁证（不算候选）**：`ZhuoruiYUN/Sentry-Navigation`（RoboMaster 哨兵、Humble、Livox MID360）与 `TIANN-ENG/px4-mid360-fastlio-sim` 都是**Classic** 路线（前者未逐文件核实是否含 gz 插件）——说明"Rm 圈子里大家也还在 Classic 上受苦"。

---

## §3 逐个候选：源码级证据与裁决

### 3.1 `uos/rmagine` + `rmagine_gazebo_plugins` —— **地基**（BSD-3，Fortress 已测）

**它是什么**：Alexander Mock（Uni Osnabrück）的射线追踪传感器仿真库（ICRA 2023 论文），`rmagine_gazebo_plugins` 是它的 Gazebo 前端。架构是**成对的两个 `gz::sim::System`**：

- **map system**（挂 `<world>`）：把世界几何同步成**一棵常驻 BVH**，几何动了就 **refit**（不重建），支持 `ignore_model` / `ignore_link` 排除自机；通过进程内 `MapRegistry` 按 `map_key` 发布。
- **sensor system**（挂 `<world>`，每个 backend 一个）：用 `components::CustomSensor` 自动发现 `<sensor type="custom" gz:type="rmagine_embree">`，按 `model_type` 打射线，发 `gz.msgs.PointCloudPacked`（单环还发 `LaserScan`）。

**关键证据（A 级，都读过）**：

- **Fortress 支持是写在 CMake 里的、不是口号**：`CMakeLists.txt:27-53` 先试 `gz-sim8`，不中就 `find_package(ignition-gazebo6 REQUIRED)`，并切换 `ignition-plugin1 / transport11 / common4 / math6 / msgs8 / rendering6`；`package.xml:61-62` 还有 `condition="$ROS_DISTRO == 'humble'"` 的依赖。
- **版本差异被压缩到 1 个头文件**：`include/rmagine_gazebo_plugins/gz/gz_compat.hpp` 全文只做一件事——`RMAGINE_GZ_IGNITION_ERA` 为真时把 `GZ_ADD_PLUGIN` 映射到 `IGNITION_ADD_PLUGIN`（与我在本机 F5 的核实完全对上）。
- **任意图案**：`src/gz/sensor_model_config.cpp:157-245`（`LoadO1Dn`）——`<o1dn><scan><rays_file>` 读 YAML `{width,height,rays:{orig,dirs[]}}`，`dirs` 是**任意向量列表**；也支持内联 `<rays><dirs><dir>…`。
- **批量 + 多线程**：`src/rmagine_embree/include/rmagine/simulation/O1DnSimulatorEmbree.tcc` 用 **`tbb::parallel_for(blocked_range3d<…>)`** 一次调用吃下整帧射线。
- **发布字段**：`sensor_model_publish.hpp:158-260` 的 `PointCloudPacked` = `xyz(F32) + ring(U16)`，可选 `normals/obj_id/face_id`；**没有 intensity、没有逐点时间**。

**裁决：REUSE（作为底座）**。它解决的是最难、最不该自己写的那一层（**常驻 BVH + 增量 refit + 多线程批量化 + 任意方向表**），而且许可干净（BSD-3）、Fortress 有官方口径。

**缺口（相对我们）**：①只有 `gz.msgs` 类型，**给不出 `livox_ros_driver2/CustomMsg`**；②没有逐点 `offset_time`；③`sensor type="custom"` 需要 SDF 侧配合，与我们「模型级插件」的用法不同；④`mid360.csv` → `rays.yaml` 需要一次转换（800k×3 float ≈ 10 MB YAML，能跑但难看）。

---

### 3.2 `ashduwihch/mid360_gazebo_harmonic` —— **最接近现成 R1**（MIT，Harmonic）

**它是什么**：把 `Livox-SDK/livox_laser_simulation` 的思路整体搬到新 Gazebo，并用 **rmagine/Embree** 替代 ODE 逐射线碰撞体。`packages/` 下三个组件：`mid360_simulation_plugin_ros2`（**MIT**）+ `rmagine`（BSD-3，trim 到 core+embree）+ `rmagine_gazebo_plugins`（BSD-3，trim 到「Harmonic 的 Embree **collision** map system」）。`THIRD_PARTY.md` 逐条说明了 trim 与许可保留。

**为什么说它"几乎就是 R1"（逐条对上我们的需求）**：

| 我们的需求 | 它的实现（源码位置） |
|---|---|
| `gz::sim::System` | `src/mid360_system.cpp:92-95`（`ISystemConfigure/PreUpdate/PostUpdate`） |
| CPU BVH 批量求交 | `src/mid360_system.cpp:385` `embreeSimulator->simulate(sensorToWorld, embreeResults)`（`O1DnSimulatorEmbree`，TBB 多线程） |
| 任意 CSV 图案 + 滚动窗口 | `csv_scan_pattern.hpp:21-56`（读 `time,az,ze`）+ `mid360_system.cpp` 的 `startIndex` 滚动 |
| 逐点时间（FAST-LIO 去畸变要） | `mid360_system.cpp:480-481` `offsetNs = i*downsample*1e9/point_rate`；`CustomMsg.timebase = 帧首` |
| `line/tag/reflectivity` | `csv_scan_pattern.hpp:50`（`index%4` → line）、`mid360_system.cpp:478-506`（tag `0x10`、refl 100） |
| 直接发 ROS 2（**绕开 `ros_gz_bridge`**，因为 `CustomMsg` 桥不了） | `mid360_system.cpp:187-…` 在插件里 `rclcpp::init` + 建 node/publisher；`PublishCloud()` 同时填 `PointCloud2`（26 B/点：xyz+intensity+tag+line+timestamp）与 `CustomMsg` |
| 自机排除 | `mid360_system.cpp:150` `MapRegistry::AddIgnoredModelEntity(entity)` + RMAGINE map 侧的 ignore |
| IMU | `OnImu()`（`:525-582`）把 gz `IMU` 话题转成 `sensor_msgs/Imu` 发 `/livox/imu` |
| 性能可观测 | `:405-411` 每 100 帧打印 `[MID360] Embree average raycast: X ms for N rays` |

**性能口径**：README 写「**200,000 点/秒，10 Hz，每帧 20,000 条射线**」（= 真机 MID-360 的规格），没有给 ms 数——但代码自带计量，我们接上就能测。

**裁决：ADAPT（首选骨架）**。它把「`gz::sim::System` 里怎么做 Livox」这件事**从头到尾写了一遍**，包括那些不看代码想不到的坑（`offset_time` 用 `point_rate` 推、`output_yaw` 处理 180° 安装、`disableFixedJointLumping` 之类）。**工作量 S–M**，具体改动见 §5。

**为什么不是 REUSE（原样用）**：CMake **硬钉 `gz-sim8`**（`find_package(gz-sim8 REQUIRED)` + `gz-plugin2/gz-transport13/gz-msgs10` + `GZ_ADD_PLUGIN`），而且 `ray_backend=physics` 分支依赖 `components/RaycastData`（Fortress 没有，F1）。

---

### 3.3 `FZ-XiaoXi/Livox-Lidar-Gazebo-Sim-Sensor`（Apache-2.0，Harmonic）

**它是什么**：一个**自定义 `gz::sensors` 传感器**（`gz-sensors8`）+ 一个发现它的 `gz::sim::System`。求交方式是**把 `RaycastData` 组件写进 ECM 让物理引擎算**（`src/livox_lidar_system.cpp:96-152`，注释里逐字写「Gazebo 8 raycasts every non-empty RaycastData component on each step」）。自带 7 种 Livox `scan_mode/*.csv`（HAP/avia/horizon/mid360/mid40/mid70/tele）与 7 个 xacro。

**价值**：①**Apache-2.0**，许可最宽松；②**证明了「自定义 sensor + 逐射线任意方向 + 物理引擎批量求交」这条路在 gz-sim8 上是通的**；③`scan_mode` CSV 全套可参考。

**裁决：ADAPT-for-Harmonic / 不适用 Fortress**。在 Fortress 上整条路不成立（F1）。**但如果哪天升到 Harmonic，这个是比 #2 更"原生"的写法**（走 `gz-sensors` 而不是模型插件），值得留档。

---

### 3.4 `jongbob1918/livox_mid360_gz_sim_ros2`（**专有**，Fortress）—— 结构最像，许可最不能用

**它是什么**：**明确针对 Fortress 验证**（README 支持表：Ubuntu 22.04 / Humble / `ign gazebo`；CMake 里 `gz-sim8→gz-sim7→ignition-gazebo6` 三级探测），模型级 `gz::sim::System`，读 CSV 滚动窗口，**在插件内建 `rclcpp` 节点**直接发 `CustomMsg`(`/livox/lidar`) + `PointCloud2`(`/livox/lidar/points`) + ground-truth odom，还带**逐点时间插值 + 帧内位姿插值**（`scanFraction`，模拟扫描期间的机器人运动，`src/livox_mid360_gz_plugin.cpp:419-474`）——这一点比 #2 还细。

**致命的三条**：

1. **许可**：`LICENSE` 全文 = `Copyright (c) LK Robotics. All rights reserved.` + 「This source code and the bundled scan-pattern data are proprietary. No right to copy, publish, sublicense, or redistribute is granted…」——**不能 vendor、不能抄、不能改**（法务上连"照着重写"都要小心，只能吸收"公开可见的思想"）。
2. **求交只支持 box / plane**：`src/livox_mid360_gz_plugin.cpp:344-357` 只处理 `sdf::GeometryType::BOX` 与 `PLANE`，README 的「已知限项」也直说 mesh/cylinder/sphere 不参与求交 ⇒ **我们的 45,900 面 STL 世界直接 0 命中**。
3. **性能模型是 O(射线×box) 暴力**：`RaycastCollision()` 每条射线遍历所有碰撞体（`:230-306`），没有 BVH ⇒ 30000 射线 × N 个 box 在 Fortress 上到不了 10 Hz。

**裁决：DO-NOT-USE（许可）/ READ-ONLY 参考**。**唯一值得吸收的是它的"帧内运动"模型**（扫描窗口内按 `scanFraction` 插值位姿，每点一个 offset_time）——这是真机 MID-360 的行为，`#2` 是"一帧一个位姿"，我们 R1 若追求保真度可以自己实现这 30 行。

---

### 3.5 `DHA-Tappuri/livox_ignition`（无 LICENSE，Fortress）—— 「过采样 + 重采样」的 Fortress 版

**它是什么**：README 逐字说明了动机——「**fortress では GPU ベースのカスタムレイキャスト機能がないため**，用 `gpu_lidar` 生成高密度扫描，再只抽取匹配 Livox 图案的射线」。实现：`sdf/sample.sdf:16-51` 里 `gpu_lidar` 开 **1440×1440**（≈2.07M 条/帧！），`LivoxLidar.cc:152-159` 把每条 CSV 方向**投影到最近邻网格索引** `_dir_table[i].idx = (v*h_samples+h) % …`，`OnLaserScan()`（`:186-245`）按索引取距离、用极坐标重建点，发 gz protobuf 自定义消息，再由 `livox_ign_ros2_bridge` 节点转成 `livox_ros_driver2/CustomMsg`。

**评价**：这就是我们迁移计划里 **R3 的"穷人版"**（不 fork 渲染器，改成过采样）。**在 Fortress 上确实能出点**，但代价是：①**2M 条 GPU 射线/帧**（llvmpipe 下无法接受，见迁移计划 §4 的无 GPU 裁决）；②角度量化 0.25°，比 MID-360 平均点距（≈0.84°）细，但**边缘目标命中与否会跳动**；③**无许可**。

**裁决：DISCARD**（性能模型与我们的"无 GPU/headless"约束冲突；许可也不明）。**但它的"投影表 + 索引重采样"技巧值得记住**——如果哪天只想在 Fortress 上先"有个能出点的 Livox"，这是最省事的一小时方案。

---

### 3.6 `acodeiro-dev/lidar-livox-mid360`（无 LICENSE，Harmonic）

`gpu_lidar` → `ros_gz_bridge` → 自写 `livox_converter` 节点 → `CustomMsg`（README 的 ASCII 图很直白）。**规则网格 + 事后转格式**，不解决图案问题。**裁决：DISCARD**（仅作"转换节点怎么写"的参考）。

---

### 3.7 Classic 家族 —— 我们的来源，也是 CSV/语义的真值

| 仓库 | 许可 | 活动 | 说明 |
|---|---|---|---|
| `Livox-SDK/livox_laser_simulation` | **MIT** | 348★，2023-11 最后 push | 原版（ROS 1）。`livox_ode_multiray_shape.cpp` 的 `AddRay()` 每条射线建一个 `ODECollision` —— **我们 2.74 GiB 的根因** |
| `LCAS/livox_laser_simulation_ros2` | **MIT** | 13★，2025-05 | ROS 2 移植（Classic） |
| `CMU-cabot/livox_laser_simulation_ros2` | **MIT** | 1★，2025-01 | 同上的另一份 fork |

**裁决：不能直接用（Classic），但它是我们"必须逐字对齐"的真值**：CSV 列语义、`zenith−90°` 的方向公式、`offset_time/line/tag` 的取值、滚动窗口语义，都在这里。**F7 已经确认候选 #2 与它同源同公式**，这是 #2 能被我们采信的最强理由。

---

### 3.8 上游 Jetty `cpu_lidar` —— 架构参考，**不可移植**

- 合入：`gazebosim/gz-sim` **PR #3343**，commit `839e621`，**2026-05-20**。
- 实现：`src/systems/cpu_lidar/CpuLidar.cc`（283 行）—— 生成规则网格射线 → 写 `components::RaycastData` + `NeedsRaycast` → 物理引擎算 → 读回 `results` → `SetRaycastResults()`。
- **两条硬阻断**：①`RaycastData` 在 `gz-sim6/7` 不存在（F1）；②`GenerateRays()` 非虚（F3）。
- 另外 Jetty **没有 Humble 的 `ros_gz` 包**（迁移计划 §2.1 实测）。

**裁决：只借"分工"（system 负责图案与发布、引擎负责求交），不借代码**。

---

## §4 裁决表：REUSE / ADAPT / WRITE + 工作量

| 方案 | 做法 | 工作量 | 具体 gap | 风险 |
|---|---|---|---|---|
| **A（推荐）** | **`ashduwihch` 的 MIT 插件（骨架 + CSV + CustomMsg）+ 上游 `uos/rmagine_gazebo_plugins`（BSD-3，Fortress 已测）做 map/BVH** | **S–M（1–4 人天）** | ①CMake 换 `ignition-*` 包名（照上游 `CMakeLists.txt:27-53` 的写法）②注册宏加 `#ifdef`（F5，1 行）③删/`#ifdef` 掉 `ray_backend=physics`（`RaycastData` 在 Fortress 不存在）④map system 用上游（visual 建 BVH）或移植它的 collision 版（我们世界两者同一 STL，等价）⑤`<csv_file>` 指到我们 `scan_mode/mid360.csv` ⑥`samples 20000→30000` + `point_rate 200000→300000` ⑦`frame_id`/话题/字段对齐 ⑧自机排除 ⑨30k@10 Hz 实测 + 与 Classic 几何对拍 | 中低：Embree 4 需联网下载（apt 只有 3.12）；mesh 加载走 assimp（本机已装） |
| **B（许可最干净）** | **只用上游 `rmagine`/`rmagine_gazebo_plugins`（BSD-3）+ 自己写 ~200–300 行 `Mid360System`**（照 #2 的公开结构，不抄它的代码文本） | **M（3–5 人天）** | 需要自己实现：CSV 滚动窗口、`offset_time` 推导、`CustomMsg` 填充、IMU 转换、`output_yaw` 那类安装补偿 | 中：Fortress 侧由上游背书，自己写的部分全在自己手里 |
| **C（兜底）** | 自研 R1（我们已验证的 Bullet `btBvhTriangleMeshShape` + CSV） | **L（1–2 周）** | BVH 常驻/增量 refit、动态模型同步、多线程、`CustomMsg`、自机排除、时间模型，**全部从零** | 高：等于把 #1/#2 已经踩过的坑再踩一遍 |
| D | R3 fork `gz-rendering`（立方图重采样） | **L+** | 3 个包、精度损失 | 已否决（迁移计划 §9.4.6） |
| E | 升 Harmonic 用 `RaycastData`（#3 路线） | **XL** | `gzharmonic` 会 `Remv` 掉 Classic（迁移计划 §2.2 实测） | 已否决 |

**一句话建议**：**先花半天做 A 的"编译打通"**（改 CMake + 宏 + 编掉 physics backend，跑它的 demo world），
**打通后再花 1–2 天做契约对齐与 30k 实测**；如果 A 在 Fortress 上出现不可预期的 API 坑，再退到 B。

---

## §5 去 Fortress 的具体改动清单（A 方案，文件级）

> 只列「必须动什么」，不写代码。**这些是适配项，不是设计项**——设计已经被 #1/#2 做完了。

| # | 位置 | 改什么 | 依据 |
|---|---|---|---|
| 1 | `CMakeLists.txt` | `gz-sim8`→`ignition-gazebo6`；`gz-plugin2`→`ignition-plugin1`；`gz-transport13`→`ignition-transport11`；`gz-msgs10`→`ignition-msgs8`；target 名 `gz-sim8::gz-sim8`→`ignition-gazebo6::ignition-gazebo6` | 上游 `rmagine_gazebo_plugins/CMakeLists.txt:27-53` 就是这么写的 |
| 2 | `mid360_system.cpp:632-641` | `GZ_ADD_PLUGIN` / `GZ_ADD_PLUGIN_ALIAS` 加 `#ifndef … #define … IGNITION_ADD_PLUGIN` 兜底 | 本机 F5：Fortress 的 `gz/plugin/Register.hh` 只有 `IGNITION_ADD_PLUGIN` |
| 3 | `mid360_system.cpp:21,180-185` + PreUpdate 的 physics 分支 | 删掉或 `#if GZ_SIM_VERSION>=8` 包住 `components/RaycastData`（Fortress 无此组件） | F1/F2 |
| 4 | map system 来源 | 用上游 `rmagine_embree_map_system`（**visual** 建 BVH，`ignore_model` 排除自机）；或把 #2 的 collision 版移植过来（它改成了 **collision** 建 BVH） | 我们世界 visual/collision **是同一个 STL**，两者等价；但**动态物体的 visual/collision 一致性**要实测 |
| 5 | `model.sdf` 的 `<plugin>` 块 | `<csv_file>` 指向 `…/livox_laser_simulation_RO2/scan_mode/mid360.csv`（列格式/公式已验证一致，F7）；`samples=30000`；`point_rate=300000`（**否则 `offset_time` 会跨 150 ms > 100 ms 帧**） | F7 + `mid360_system.cpp:480` |
| 6 | 输出契约 | `frame_id` = 我们的 `livox_frame`（或按 `output_yaw` 补偿安装）；`/livox/lidar` 走 `CustomMsg`；`/livox/lidar/pointcloud` 的**字段表**要核对：我们发 **xyz+rgb**，它发 **xyz+intensity+tag+line+timestamp** ⇒ 若有下游用 `pcl::fromROSMsg<PointXYZRGB>` 会炸，用 `PointXYZI`/`PointXYZ` 则无碍 | 我们 `livox_points_plugin.cpp:83-98,271` |
| 7 | 自机排除 | `MapRegistry` 的 `ignore_model` / `AddIgnoredModelEntity`，并把机器人自身从 BVH 里剔掉（否则每帧自击） | `mid360_system.cpp:150` |
| 8 | 验收 | ①30k 点/10 Hz ②同一位姿下与 Classic 点云几何对拍（本仓已有 `tools/scripts/regress/linefit_replay_probe.py`、`robot11_livox_fov.py` 一类探针）③RSS/RTF 不劣于 Classic（3.2 GiB / 0.32–0.46） | 迁移计划 §7.4 出口判据 |

**明确不用动的**：`gpu_lidar`/SDF `<lidar>` schema、`ros_gz_bridge`（PointCloudPacked 路线才需要，我们走进程内 `rclcpp` 就不需要）、`gz-physics`、渲染器。

---

## §6 许可证影响（vendor 还是 depend）

| 组件 | 许可 | 能不能 vendor | 义务/注意 |
|---|---|---|---|
| `ashduwihch/mid360_simulation_plugin_ros2` | **MIT** | ✅ | 保留版权与许可文本；无专利条款 |
| `uos/rmagine` / `rmagine_gazebo_plugins` | **BSD-3-Clause** | ✅ | 保留版权/条件/免责三段；**不得用作者名背书**；学术使用请引用 ICRA 2023 论文（README 给了 BibTeX） |
| `FZ-XiaoXi/…` | **Apache-2.0** | ✅ | 保留 `LICENSE` + 改过的文件要有变更声明 + `NOTICE`；含**专利授权**（对我们更安全） |
| `jongbob1918/…` | **Proprietary** | ❌ **绝对不行** | 「No right to copy, publish, sublicense, or redistribute」——**连把它的代码贴进我们的仓都不行**；只能读思路 |
| `DHA-Tappuri/…`、`acodeiro-dev/…` | **无 LICENSE** | ❌ | 默认**全保留**（无授权即无许可），技术上"能用"≠法律上"能用" |
| `mid360.csv` 图案数据 | 我们仓内已有（来自 Livox 官方 `scan_mode`） | —— | 注意 #4 的 README 明确写了「bundled scan-pattern 的**外部再分发依据未整理**」——**图案数据本身的再分发是另一个法务问题**，与我们已有这份无关，但**不要把别人的 CSV 再分发** |
| Embree 4（rmagine 首次编译自动下载） | Apache-2.0 | 构建期下载 | 离线/CI 环境要预置；apt 的 `libembree-dev` 是 **3.12**，rmagine 要 **4**（`RMAGINE_EMBREE_LOCAL_VERSION 4`，下载 4.4.0） |
| TBB / Assimp / yaml-cpp | 各自宽松 | —— | 本机 assimp 已装 |

**结论**：**A 方案许可完全干净**（MIT + BSD-3，都是可 vendor 的宽松许可）；**B 方案更干净**（只剩 BSD-3）；**C 自研**无许可问题但最贵。

---

## §7 「CPU 批量射线求交当 LiDAR 仿真器」是不是常规做法？

**是，而且是这个问题的标准答案。** 四条独立证据：

1. **PyBullet 官方就带这个用法**：`rayTestBatch` 是内建 API，仓库里直接有示例 [`examples/pybullet/examples/batchRayCast.py`](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py)（本沙箱 HTTP 200 已核实存在），社区教程也就是拿它模拟 2D/3D lidar。⇒ **"一次提交几千条射线、拿回距离数组"是物理引擎的常规能力**，不是奇技淫巧。
2. **有专门的论文级工作**：`rmagine`（**ICRA 2023**，DOI `10.1109/ICRA48891.2023.10161388`）的整篇贡献就是「在三角面地图上用 CPU/GPU 光追做距离传感器仿真」，README 的示例是**一次仿真 100 台 VLP-16**。⇒ 学术界把它当一个正经方向在做。
3. **上游 Gazebo 自己也在 2026 年补了 CPU lidar**（`gz-sim` PR #3343，2026-05-20），说明"要一条不依赖渲染器的 LiDAR 路径"是**官方认可的需求**——只是它选择了"让物理引擎批量算"，而 Fortress 没有那个组件（F1）。
4. **我们自己的实测**：45,900 面真实场地 + 真 30k CSV 图案，Bullet BVH **1 线程 12.5–29.4 ms**、8 线程 3.2–7.4 ms，**100 ms 预算下余量 3–30×**（迁移计划 §9）。

**所以"普遍性"的准确回答**：
- **架构层面**：极其普遍（引擎批量射线、离线点云生成、PyBullet/MuJoCo/rmagine/Jetty 都是），**没有创新风险**；
- **工程层面**：真正稀缺的不是"会写"，而是"**在 Fortress 的 `gz::sim` 里把 CSV 图案、逐点时间、`CustomMsg`、自机排除、BVH 常驻/refit 全部接对**"——而这恰好已经有 2 个开源实现替我们做完了（一个 MIT、一个 BSD-3）；
- **结论**：**自研的"必要性"很低，"普遍性"很高**——高到应该直接复用。

---

## §8 诚实清单：没抓到 / 没验证

**抓不到（网络侧）**：

- `lxrobotics.com/blog/livox-mid-360-gazebo/`（搜索命中的 "How to simulate a Livox MID-360 using Gazebo"）：DNS 解析到非公网 IP，**本沙箱拒绝访问**，只看到搜索标题，**内容未读**。
- `arxiv.org/abs/2209.13397`（rmagine 论文预印本）：同上，**摘要未读**；因此论文里的具体性能数字**没有引用**，只用仓库 README 的说法。
- GitHub issue 正文：`gz-sim#1958`（Custom lidar sensor）、`gz-sim#2645`（Build my own Lidar Sensor with custom scanning shape）、`gz-sensors#26`（CPU lidar (no rendering)）——**只拿到标题**，正文未能完整抓取，**结论未逐句核对**（`#1958/#2645` 与迁移计划 §3.6 的记录一致）。
- `rmagine_gazebo_plugins` 的 Wiki：未抓。

**没验证（工程侧）**：

- **没有编译、没有运行任何候选**（本任务只读代码）。所以「#2 在 Fortress 上能编过」是**基于源码 + 本机头文件的推断**，**不是实测**——这是 A 方案第一步就要打的靶。
- **#2 的 Embree 实测 ms 未知**（它只在自己机器上跑过 20k/10 Hz）；**30k@10 Hz 未验**。
- **rmagine 的 mesh 加载**（assimp / `gz-common` mesh loader）对我们这份 **mm 单位、scale 0.001、45,900 面**的 STL 是否 1:1 正确，**未验**（我们的 mm/m 单位 bug 在 Bullet 上踩过一次，见迁移计划 §9.2）。
- **#2 的 `PointCloud2` 字段（intensity）与我们下游**（期望 xyz+rgb，PCL 转换）**是否兼容**：只核对了字段列表，**没有逐个消费者核对**。
- **`mid360-real-centr.csv` 与我们的 `mid360.csv` 是否逐行相同**：只对了表头/zenith 范围（一致）与首行（azimuth 差 180°），**疑似同一数据的不同起点或朝向约定**，未逐行 diff。
- **#2 的 collision-map 与上游的 visual-map 在"动态物体 + 只有 visual 的装饰物"上的差异**：未验。
- `ZhuoruiYUN/Sentry-Navigation`：只看了 README，**未逐文件确认**它是否自带 gz 插件（大概率是 Classic）。
- **Fortress + `rmagine` 的组合没有 CI 证据**：上游 README 表写「Humble / Fortress」已测，但**我没找到对应的 CI 日志**（只有 rmagine 本体有 `ros2-humble` workflow）。

---

## §9 参考链接（按重要性）

**可复用（读源码）**

1. `ashduwihch/mid360_gazebo_harmonic` — <https://github.com/ashduwihch/mid360_gazebo_harmonic>（MIT；`packages/mid360_simulation_plugin_ros2/src/mid360_system.cpp`）
2. `uos/rmagine_gazebo_plugins` — <https://github.com/uos/rmagine_gazebo_plugins>（BSD-3；`src/gz/rmagine_embree_sensor_system.cpp`、`src/gz/sensor_model_config.cpp`、`include/.../gz/gz_compat.hpp`）
3. `uos/rmagine` — <https://github.com/uos/rmagine>（BSD-3；`src/rmagine_embree/include/rmagine/simulation/O1DnSimulatorEmbree.tcc`）

**Fortress 侧参考（许可受限）**

4. `DHA-Tappuri/livox_ignition` — <https://github.com/DHA-Tappuri/livox_ignition>（无许可；Fortress + `gpu_lidar` 过采样重采样）
5. `jongbob1918/livox_mid360_gz_sim_ros2` — <https://github.com/jongbob1918/livox_mid360_gz_sim_ros2>（专有；Fortress + 暴力 box/plane 求交 + 帧内运动插值）

**上游 / 先例**

6. `gz-sim` Jetty `cpu_lidar`（PR #3343，2026-05-20）— <https://github.com/gazebosim/gz-sim/commit/839e621c852df43b8092d103f304779b47b91b61>
7. `gz-sensors10` `CpuLidarSensor.hh` — <https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/include/gz/sensors/CpuLidarSensor.hh>
8. `gz-sensors` issue #26「CPU lidar (no rendering)」— <https://github.com/gazebosim/gz-sensors/issues/26>
9. `gz-sim` issue #2645「Build my own Lidar Sensor with custom scanning shape」— <https://github.com/gazebosim/gz-sim/issues/2645>
10. PyBullet `batchRayCast.py` — <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/batchRayCast.py>
11. rmagine ICRA 2023（DOI `10.1109/ICRA48891.2023.10161388`）— <https://ieeexplore.ieee.org/document/10161388>

**本仓内相关**

12. [`gazebo_sim_migration_plan.md`](gazebo_sim_migration_plan.md) —— Humble↔Fortress、`gpu_lidar` 图案裁决、无 GPU 裁决、R0/R1/R3 决策表、**我们自己的 Bullet BVH 实测**（§9）
13. `src/rm_simulation/livox_laser_simulation_RO2/` —— 我们 Classic 插件的 CSV 读取、方向公式、滚动窗口（本轮 F7 的对拍基线）
