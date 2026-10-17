# Gazebo Fortress 仿真后端（`sim_backend:=gz`）

> 状态：**opt-in 可用**。默认仍是 `sim_backend:=classic`（Gazebo Classic 11），
> 不传该参数时 `bringup_sim.launch.py` 的节点集合/命令行/时序**逐字节不变**（证据见 §9）。
>
> 一句话：**把"起仿真世界"这一件事换成 Gazebo Fortress（`ignition-gazebo6` 6.18），
> 感知 / LIO / nav2 一行不改**，话题/帧/QoS/字段与本仓既有契约对齐；点云是
> 30000 束 CPU 射线 + 我们自己的 `mid360.csv` 非重复图案，不是规则网格。
>
> 相关文档：
> - 上游选型与实测（编不编得过、几何对拍 49 µm）→ [`livox_sensor_prior_art.md`](livox_sensor_prior_art.md) §10
> - 迁移总计划 P0–P4 → [`gazebo_sim_migration_plan.md`](gazebo_sim_migration_plan.md)（本文件是它的**最终状态**）
> - 斜装的帧/契约规则 → [`tilted_lidar_fidelity.md`](tilted_lidar_fidelity.md) §I/§J/§N
> - TF 契约 → [`tf_interface_contract.md`](tf_interface_contract.md)

---

## 0. 用法（复制即可）

```bash
# ① 一次性：取上游 + 打补丁 + 编译（源码不进本仓库，编到独立前缀）
tools/scripts/gzsim/fetch_and_patch_mid360.sh

# ② 整栈（推荐）：与 Classic 同一条 launch，只多一个参数
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
    world:=RMUC2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0

# ③ 只想看传感器（不开感知/LIO）：台架世界
ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=False

# ④ 带 GUI（无 GPU 机器上要 Xvfb；本仓不替你起 Xvfb）
xvfb-run -s "-screen 0 1280x800x24" ros2 launch rm_livox_gz_sim gz_sim.launch.py \
    robot:=none gui:=True
```

`sim_backend:=gz` 的**两个硬要求**（缺了会明确报错，不会静默跑错）：

| 要求 | 为什么 |
|---|---|
| `robot:=robot11` | 只有这个槽位在 gz 侧有 Fortress 化的模型生成路径（从同一份 URDF 派生 SDF） |
| `robot11_mount:=sensor` | 倾角必须在 **URDF** 里，`robot_state_publisher` 发的 `base_link→livox_frame` 才与物理一致 |

---

## 1. 架构：谁在哪一层

```
┌─────────────────────────── gzserver 进程（ign gazebo） ───────────────────────────┐
│  世界 rmuc2026_gz.sdf（SDF 1.9）                                                   │
│   ├── model://RMUC2026_world      真 STL（45,900 面，mm→m）                        │
│   ├── model://rm_robot11_gz       ← 运行期由本仓 URDF 派生（113 个真碰撞体）        │
│   │     ├── link livox_frame      <pose>0.000562 0.130916 0.157028 -0.523599 0 0</pose>
│   │     │     └── <sensor type="imu"> <topic>/mid360/imu_gz</topic>                │
│   │     ├── <plugin libmid360_system.so>  ×2（同一 link、同一姿态）                 │
│   │     │     ├── #1 publish_pointcloud_type=3 → /livox/lidar          (CustomMsg)  │
│   │     │     └── #2 publish_pointcloud_type=2 → /livox/lidar/pointcloud (PC2)      │
│   └── <plugin librmagine_embree_map_system.so>  一棵 Embree BVH（全域一次）          │
└────────────────────────────────────────────────────────────────────────────────────┘
        │ gz transport /mid360/imu_gz                 ▲ rclcpp 直发（进程内，不过桥）
        ▼                                            │
   两个 mid360 插件实例订阅 IMU、转成 ROS /livox/imu    │
                                                     │
        ┌────────────────────────────────────────────┴───────────────┐
        │ ros_gz_bridge（**只桥 /clock**）→ /clock                     │
        │ robot_state_publisher + joint_state_publisher（TF，同 Classic）│
        └────────────────────────────────────────────────────────────┘
        ▼
   linefit(gravity_aligned_frame=base_link) → pointcloud_to_laserscan(target_frame=base_link)
        → /segmentation/* + /scan → LIO(small_point_lio) → odom→base_link + /cloud_registered
        → slam_toolbox(map→odom) → nav2（mode:=slam_nav 时）
```

**为什么点云不过 `ros_gz_bridge`**：MID-360 是"进程内 rclcpp 直发"，所以
① 没有 `PointCloudPacked` 中转、② 没有"桥出来 `frame_id` 变成 gz scoped name"的坑
（那是 `gazebo_sim_migration_plan.md` §6.7 点名的最高风险项）、③ 少一次序列化/拷贝。

---

## 2. 与之前的区别（**详表**）

### 2.1 话题（名字 / 类型 / 来源）

| 话题 | Classic（今天） | gz 后端 | 类型 | gz 侧来源 | 一致？ |
|---|---|---|---|---|---|
| `/livox/lidar` | ✅ | ✅ | `livox_ros_driver2/msg/CustomMsg` | 插件实例 #1，**进程内** rclcpp | **一致** |
| `/livox/lidar/pointcloud` | ✅ | ✅ | `sensor_msgs/msg/PointCloud2` | 插件实例 #2，进程内 rclcpp | **一致** |
| `/livox/imu` | ✅ | ✅ | `sensor_msgs/msg/Imu` | 插件把 gz `<sensor type="imu">` 转成 ROS | **一致**（100 Hz，`frame_id=imu_link`） |
| `/clock` | gazebo_ros 插件 | ✅ | `rosgraph_msgs/msg/Clock` | `ros_gz_bridge`（**唯一**过桥的东西） | 一致 |
| `/cmd_vel_chassis` | `libgazebo_ros_planar_move.so` 订阅 | ❌ **未实现** | — | — | **不一致**（见 §8） |
| `/odom_ground_truth` | planar_move 发（真值，仅调试用） | ❌ 无 | — | — | 缺失（契约上不要求；`odom→base_link` 归 LIO） |
| `/segmentation/*`、`/scan`、`/cloud_registered`、`/Odometry` | ✅ | ✅ | 同 | **下游节点一行未改** | **一致** |

### 2.2 `frame_id` 约定

| 项 | Classic | gz 后端 | 说明 |
|---|---|---|---|
| 点云 `frame_id` | `livox_frame` | **`livox_frame`** | gz 的 `model::link::sensor` 命名**不会**出现在 ROS 侧（我们不用 `PointCloudPacked` 桥） |
| `/livox/imu` 的 `frame_id` | `imu_link` | **`imu_link`** | 由插件的 `<imu_frame_id>` 钉住；不钉会退回 `livox_frame`，与 Classic 不一致 |
| TF `base_link→livox_frame` | `robot11_mount:=sensor` 时 rpy **−30.000 0 0** | 同 | 两边同一份 URDF、同一个 `robot_state_publisher` |
| 点云数字表达在哪个系 | `sensor` 档：**斜的传感器系** | 同 | gz 侧由 `<link><pose>` 的 roll −0.523599 承担 ⇒ 帧与数据自洽（§I.2 的"账"对上了） |
| 谁发 `odom→base_link` | LIO（`lio:=small_point_lio` 时它自己发） | 同 | gz 侧**不**发任何 odom TF |
| 谁发 `map→odom` | slam_toolbox / amcl / ICP（按 `mode`/`localization`） | 同 | 未改动 |

> **和 `robot11_mount:=plugin`（Classic 默认档）的差别**：那一档点云是**水平的**、
> `frame_id` 却叫 `livox_frame`（帧是平的），倾角藏在插件 `<tilt_rpy>` 里。
> gz 后端**只实现 sensor 语义**（物理倾斜 + 帧也斜 + 下游重力对齐），
> 因为"平帧 + 斜数据"是自相矛盾的组合，没有移植价值。

### 2.3 QoS

| 话题 | Classic | gz 后端（默认） | 备注 |
|---|---|---|---|
| `/livox/lidar` | `SensorDataQoS`（**best effort** + keep_last 5） | **同** | `<qos_reliability>best_effort</qos_reliability>`；上游原值 `rclcpp::QoS(KeepLast(20)).reliable()` 已改掉 |
| `/livox/lidar/pointcloud` | `SensorDataQoS` | **同** | 同上 |
| `/livox/imu` | 默认 reliable（`gazebo_ros_imu_sensor`） | **reliable 保留** | FAST-LIO 用默认 depth-10（reliable）订阅 IMU；best-effort 写者与 reliable 读者 **QoS 不兼容 ⇒ 一条都收不到**。IMU 帧 ~250 B @ 100 Hz，且发布发生在 gz transport 回调线程（不在 PostUpdate），不构成背压源 |

`reliable` 仍可一键复现（`gz_qos:=reliable`），只用于对照实验，**不要**用于生产。

### 2.4 点云字段布局

| 项 | Classic | gz 后端 |
|---|---|---|
| 字段 | `x,y,z`（f32）+ **`rgb`**（f32，`setPointCloud2FieldsByString(2,"xyz","rgb")`） | `x,y,z`（f32, 0/4/8）+ `intensity`（f32, 12）+ `tag`（u8, 16）+ `line`（u8, 17）+ `timestamp`（f64, 18） |
| `point_step` | 32 | **26** |
| `rgb` | **有** | **没有** |
| 只发有效回波 | ✅ | ✅（30000 束里 26–27% 命中） |
| `is_dense` | — | `true` |

**谁会被这个差异打到**：见 §7「会坏的地方」。结论是**本仓没有消费者硬依赖 `rgb`**：
linefit 用 `pcl::fromROSMsg<PointXYZ>`（按字段名取 xyz）、`pointcloud_to_laserscan` 用
`PointCloud2ConstIterator<float>("x"/"y"/"z")`、LIO 走 `CustomMsg`、其余探针只读 xyz。

### 2.5 传感器类型与插件

| 项 | Classic | gz 后端 |
|---|---|---|
| 传感器 | URDF `<sensor type="ray" name="livox_frame">` + `libros2_livox.so` | 模型级 `<plugin filename="libmid360_system.so">`（gz **system** 插件） |
| 求交 | ODE 逐射线 `<collision>` | **Embree BVH**（rmagine 的 `O1DnSimulatorEmbree`），BVH 从世界几何建一次 |
| 图案 | `mid360.csv` | **同一份** `mid360.csv`（不复制、不提交副本，launch 运行期替换 `@MID360_CSV@`） |
| 30000 rays | ✅ | ✅ `<samples>30000</samples>` |
| 逐点时间 | `offset_time` 恒 0（帧内无运动） | `offset_time = i·1e9/point_rate`，**单调、≤100 ms** |
| 双发布 | 插件同时发两个话题 | **两个插件实例**（同 link、同姿态）：实例 #1 只发 CustomMsg，实例 #2 只发 PC2 |
| 自机排除 | `livox_frame` 故意不给 collision | 插件把所属 model 登记进 `MapRegistry` ⇒ 整车 113 个碰撞体不进 BVH |
| 物理后端 | ODE | 只有 Embree（Fortress 无 `components/RaycastData`，physics 后端编译期关闭） |

### 2.6 world / SDF 版本要求

| 项 | 要求 |
|---|---|
| SDF 版本 | **≤ 1.9**（`sdformat12` 的上限）。写成 1.10 会 `Unable to convert from SDF version 1.10 to 1.9`，**整个世界加载失败、一帧点云都不出** |
| `model.config` | `<sdf>` **不能为空**，必须写文件名：`<sdf version="1.9">model.sdf</sdf>`（Classic 容忍空，sdformat12 不容忍 → `Failure to detect an sdf tag`）。本仓两份 `world/model.config` 已在 `da5c2fe` 修好 |
| 资源解析 | `model://` 走 `GZ_SIM_RESOURCE_PATH`；launch 自动加 `<install>/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes`、`<install>/robot11/share`、运行期模型目录 |
| 插件解析 | `GZ_SIM_SYSTEM_PLUGIN_PATH` **和** `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 都设（Fortress 6.18 两个拼法都在二进制里，实测两个都设才稳） |
| `HOME` | gz 需要可写的 `$HOME`（`~/.ignition`）。本机 `$HOME` 只读 ⇒ **GUI 会 abort 并 SIGKILL server**。launch 自动把 gz 子进程的 `HOME`/`XDG_*` 重定向到运行期临时目录 |

### 2.7 启动方式与参数

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz …
```

| 参数 | 默认 | 作用 |
|---|---|---|
| `sim_backend` | `classic` | `classic` \| `gz` |
| `world` | `RMUL2026` | gz 侧目前只支持 `RMUC2026`（其它值明确报错） |
| `robot` | `''` | gz 侧必须 `robot11` |
| `robot11_mount` | `plugin` | gz 侧必须 `sensor`（`urdf` 亦可） |
| `gui` | `True` | `False` = 只起 `ign gazebo -s` |
| `gz_world` | `''` | `bench` = 台架世界（只有传感器） |
| `gz_qos` | `best_effort` | `reliable` 仅用于复现"写者被堵" |
| `gz_ignore_self` | `true` | `false` **只用于对照实验**（让传感器看得见自己的车体） |
| `gz_home` | 运行期临时目录 | 仅 gz 子进程的 `HOME` |
| `gz_csv` | 空（用 `ros2_livox_simulation` 里那份） | 指定 `mid360.csv` |
| `gz_runtime_dir` | `/tmp/rm_livox_gz_sim_<rand>` | 生成物落盘位置（调试用） |

### 2.8 性能对照（Classic vs gz）

> ⚠️ **口径**：同一台机器、同一世界（RMUC2026）、同一机器人（`robot11`，`robot11_mount:=sensor`）、
> 同一份 `mid360.csv`、`samples 30000`、`point_rate 300000`、10 Hz；无 GPU；GUI 用
> Xvfb `1280x800` + llvmpipe。Classic 的数字是**本机同轮实测**（`docs/…` 里旧值在括号内对照）。
> 机器上同时有其它 agent 在跑 ⇒ 单个数字有 ±3% 抖动，结论看量级。

| 指标 | **Classic 11**（既有基线，⚠️ 不同轮次） | **gz（Fortress）**（本轮实测） | 变化 |
|---|---|---|---|
| RTF（`/clock` 的 仿真时间/墙钟） | 0.32–0.46（`gazebo_sim_migration_plan.md` §1/§9，`robot11` 全栈；**不是**本轮同机同刻） | **0.97**（台架世界，e3a）；**0.97**（`mode:=mapping` 整栈的 `/clock`） | **≈ +2.4×**（按 0.40 中值算） |
| gzserver / ign-gazebo-server RSS | 3.2 GiB（`livox_sensor_prior_art.md` §10.3 的对照行） | **328 MB**（台架，整棵进程树）/ **441 MB**（含 robot11 + TF 发布者）/ **652 MB**（整栈 `mode:=mapping`，含 linefit/LIO/slam_toolbox） | **−80% … −90%** |
| CPU | 未单独记录（Classic 的瓶颈是 ODE 逐射线 collision） | 单进程 `ign-gazebo-server` 1 s 采样的**累计 tick** 记录在 `runs/*/rss.txt`（本机同时有别的 agent 跑，绝对值不宜引用） | — |
| 逐帧求交耗时 | 无（ODE 逐射线，未插桩） | **0.325 ms / 30000 rays**（单实例中位，台架）/ **0.293 ms**（含 robot11 的世界）；双实例合计 ≈0.65 / 0.59 ms | 量级 <1 ms |
| 启动 → 首帧点云 | 未记录（Classic 要起 gzserver + 载入 45k 面 STL + ODE trimesh） | **3.00 s**（launch → 日志 `Loaded 800000 scan directions`） | — |
| GUI（Xvfb 1280x800，llvmpipe） | 未记录 | **本轮未拿到成体系数字**（GUI 进程起来了：Xvfb + `ign gazebo gui`，整树 RSS **910 MB**；但这一轮的点云探针没采到数据）。上一轮台架单实例的口径：9.77 Hz / RTF 0.96–1.0 / 843 MB（`livox_sensor_prior_art.md` §10.4） | — |
| 每帧点数（中位） | `robot11` 全栈约 11,568（`tilted_lidar_fidelity.md` §J 的 `plugin` 档） | **8018**（台架）/ **8040**（robot11，sensor 档） | 差异来自命中率与 `max_range`，不是丢点 |

> ⚠️ **这张表怎么读**：gz 列是**本轮**实测；Classic 列是**本仓既有文档里**的基线
> （不同时间、机器负载不同）。**没有做同轮 A/B**，所以"变化"列只能看量级，
> 不要当精密对照。原因见 §10 第 8 条。

### 2.9 会坏的地方（**逐条**）

1. **依赖 `rgb` 的消费者**：gz 后端的 `/livox/lidar/pointcloud` **没有 `rgb`**、`point_step=26`
   （Classic 是 32）。本仓 grep 结论：没有节点硬依赖 rgb（详见 §7）。**但外部脚本/RViz 配色
   插件**（按 rgb 上色）会变成单色。修法：按 `intensity` 上色，或给插件加一个写入 `rgb` 的字段。
2. **`<tilt_rpy>` 不再存在**：那是 Classic 插件的键。gz 侧倾角写在 SDF `<link><pose>` 的
   roll（`-0.523598775598293`），并且**要求 URDF 的关节也斜**（`robot11_mount:=sensor`）。
   想"平帧 + 斜射线"的组合在 gz 后端**不支持**。
3. **以为两个话题同时发的人**：Classic 的插件一次同时发 `CustomMsg` 与 `PointCloud2`；
   gz 后端是**两个插件实例**，各自只发一路。所以：
   - 两路**同 stamp、同点数、逐点相同**（实测 §5），但不是"一次 publish 两个消息"；
   - 每路各做一次 30000 束求交 ⇒ **逐帧求交耗时 ≈ 2×**（这不是 bug，是 A① 决策的代价）；
   - IMU 只由实例 #1 发（`<publish_imu>`），否则 `/livox/imu` 会双倍频率 + 双发布者。
4. **SDF ≤ 1.9**：见 §2.6。`ign sdf -k` 可预检。
5. **`world:=` 只支持 `RMUC2026`**：其它场地在 gz 侧还没有 SDF 1.9 资产；加一份
   `src/rm_simulation/rm_livox_gz_sim/worlds/<name>.sdf` 并登记到 `WORLD_MAP` 即可。
6. **`/cmd_vel_chassis` 不驱动底盘**（未实现，见 §8）：车在 gz 里是**静态**的。
   nav2 规划/控制链路仍会跑，但底盘不动。
7. **`robot:=robot11` 之外没有 gz 模型**：`hzmirm` / 默认模型都只有 Classic 资产。
8. **`mode:=nav`（加载磁盘地图 + 重定位）未验证**：本文件的端到端只覆盖 `mode:=mapping`。
9. **`/odom_ground_truth` 不存在**：任何依赖"真值里程计"的调试脚本在 gz 档会拿不到数据。
10. **`ign gazebo` 的 `HOME`**：见 §2.6。裸 shell 里手动起 `ign gazebo` 前请自己重定向 `HOME`。

### 2.10 需要跟着改的配置

| 配置 | 什么时候要改 |
|---|---|
| `segmentation_sim_robot11_sensor.yaml`（`gravity_aligned_frame: base_link`） | 已在 `robot11_mount:=sensor` 档自动生效；**别**在 gz 档用 `plugin` 档那份（`gravity_aligned_frame: ""`），否则斜着的地面线会把地面判成障碍 |
| `laserscan_params_sensor_frame.yaml`（`target_frame: base_link`） | 同上（`/scan` 要落回水平面的一圈） |
| `nav2_params_sim_*.yaml` | 未验证；`robot_base_frame` 仍是 `base_link_fake`，与 gz 无关 |
| `lio` 配置 | gz 档**不需要**改：`extrinsic_R=I`（IMU 与雷达同壳同斜），`frame_id` 都是我们约定的名字 |
| 世界资产 | 新增场地时：SDF ≤ 1.9、`model.config` 的 `<sdf>` 非空、`model://` 能解析 |

### 2.11 回退方法

```bash
# 去掉 sim_backend 就是 Classic（默认路径零改动）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 …
```

* 代码层：`sim_backend` 只加了一个 `DeclareLaunchArgument` + 两个 `IncludeLaunchDescription`
  的 `condition`；**没有任何 Classic 文件被改**（`rm_simulation.launch.py`、worlds、xacro、
  configs、感知、nav2 全部一字未动）。
* 资产层：`src/rm_simulation/rm_livox_gz_sim/` 是**新增包**，删掉它 + 回退
  `bringup_sim.launch.py` 的那 3 个 Action 即回到 100% 原状（`git revert <commit>`）。
* 后端层：`tools/scripts/gzsim/fetch_and_patch_mid360.sh` 编出来的前缀可以直接
  `rm -rf`；它不装进 `/opt/ros`、不碰系统库。

---

## 3. 双实例（A① 决策）实测

**判据**：两个实例是否共用同一棵 BVH（建树一次、内存一份）、逐帧求交是否 ≈2×、
两路话题的 `frame_id`/时间戳/点数是否一致（甚至逐点一致）。

环境：台架世界 `rmuc2026_gz_bench.sdf`（只有传感器，无机器人）、headless、120 帧。

| 判据 | 实测 | 结论 |
|---|---|---|
| **BVH 是不是只建一份** | gz 日志里 `Embree map bootstrapped with 1 supported visuals` **只出现 1 次**（不是 2 次）；两个实例的 `<map_key>default</map_key>` 相同 ⇒ `MapRegistry::GetEmbreeMap("default")` 返回**同一个 `EmbreeMapPtr`** | ✅ 共用一棵 |
| 每实例一个 `O1DnSimulatorEmbree` | 是（各自持有 sim 对象，但都指向同一 `EmbreeMap`） | 结构上必然 |
| **逐帧求交耗时** | 插件每 100 帧打印一次，28 次采样（两实例交替）：**min 0.307 / median 0.325 / max 0.358 ms @30000 rays**（单实例）；⇒ **两实例合计 ≈0.65 ms/帧** | ≈2×，符合 A① 预期（0.33→0.65，绝对量仍 <1 ms） |
| 内存 | 只有传感器时整棵进程树 **328 MB**（VmRSS 合计，含 launch/bridge/ign）；有机器人时 **441 MB** | 没有出现"BVH 两份"的内存台阶 |
| `frame_id` | 两路都是 `livox_frame` | ✅ |
| 时间戳 | 两路 **同 `stamp`**（119/119 配对） | ✅ |
| 点数 | **119/119 帧完全相同**（`n_delta_max = 0`） | ✅ |
| **逐点坐标** | 从 `cloud.npz` 逐点比较：**313,721 个点 xyz 完全相同（atol=0，即比特级相同）** | ✅ 两个实例算的是同一次求交结果 |

> 也就是说：双实例的代价只有 **CPU（2× 求交）**，没有内存/一致性代价。
> 若哪天不接受这 2×，正确做法是给插件加"一次求交、发两路"的开关（约 15 行），
> **不要**改成两个姿态不同的传感器。

---

## 4. QoS（§2.3）改前/改后实测

### 4.1 改了哪一行

`mid360_system.cpp`（上游）：
```cpp
// 改前（上游原值）
const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
this->cloudPublisher = ...create_publisher<PointCloud2>(this->topic, qos);
this->customPublisher = ...create_publisher<CustomMsg>(this->topic, qos);
// 改后（我们的补丁 0003）
const auto cloudQos = (this->qosReliability == "reliable")
  ? rclcpp::QoS(rclcpp::KeepLast(20)).reliable()
  : rclcpp::SensorDataQoS();           // best_effort + keep_last 5，与 Classic 一致
```
`<qos_reliability>` 默认 `best_effort`；IMU 的 publisher **保持 reliable**（理由见 §2.3）。

### 4.2 谁在订阅（必须都是 best-effort，否则 best-effort 写者会"发了个寂寞"）

| 订阅者 | 话题 | QoS | 与 best-effort 写者兼容？ |
|---|---|---|---|
| `FAST_LIO`（AVIA 分支） | `/livox/lidar` | `SensorDataQoS` | ✅ |
| `small_point_lio`（`livox_custom_msg.h`） | `/livox/lidar` | `SensorDataQoS` | ✅ |
| `point_lio` | `/livox/lidar` | `SensorDataQoS` | ✅ |
| `linefit_ground_segmentation` | `/livox/lidar/pointcloud` | `SensorDataQoS` | ✅ |
| `pointcloud_to_laserscan` | `cloud_in` | `SensorDataQoS` | ✅ |
| `patchwork_ground_segmentation` | `/livox/lidar/pointcloud` | `SensorDataQoS` | ✅ |
| `gicp_registration` | `/livox/lidar/pointcloud` | `SensorDataQoS().keep_last(1)` | ✅ |
| `cloud_accumulator` | 两路 | `BEST_EFFORT` | ✅ |
| `tools/scripts/**` 各探针 | 两路 | `BEST_EFFORT` | ✅ |

⇒ 全仓**没有**一个 reliable 订阅者，所以改成 best-effort 写者**不会**造成"收不到"。

### 4.3 "故意慢的订阅者"实验（改前 vs 改后）

同一台机、同一台架世界，一个 **2 s/帧** 的订阅者挂在两路上；同时另一路快探针量
"别人还收得到吗"，并用 `/clock` 量仿真时间有没有被按停。

| 场景 | 写者 QoS | 慢消费者在时，快探针的墙钟频率 | `/clock` 相邻最大仿真跳变 |
|---|---|---|---|
| 基线（无慢消费者） | best_effort | **9.743 Hz** | 18 ms |
| **慢消费者（2 s/帧）在时** | **best_effort** | **9.711 Hz** | 18 ms |
| 慢消费者退场后 | best_effort | 9.705 Hz | 21 ms |
| 基线（无慢消费者） | reliable | 9.703 Hz | 26 ms |
| 慢消费者（2 s/帧）在时 | reliable | 9.689 Hz | 32 ms |
| 慢消费者退场后 | reliable | 9.730 Hz | 22 ms |

**判决性版本**：把订阅者进程直接 `SIGSTOP`（它的 DDS 线程一起停 ⇒ 不再 ack）：

| 写者 QoS | SIGSTOP 期间快探针频率 | 结论 |
|---|---|---|
| **best_effort** | **9.740 Hz** | 写者不可能阻塞（best-effort 无背压，只丢样本） |
| reliable | 9.660 Hz | 本机**也没有**被拖住 |

### 4.4 诚实结论（这个实验**没有**复现出"堵死"）

* **best-effort 下 gz 线程不被堵** —— 这条**成立**（两种慢消费者都不影响，仿真时间连
  18–32 ms 的正常抖动都没有超出）。
* **reliable 下也没被堵** —— 在本机的 `rclcpp + Fast-DDS` 组合上，**复现不出** Classic
  那次的"写者阻塞"。原因（源码级）：DDS 的可靠性边界是**读者缓存**，样本一进缓存就被
  ack；应用层回调慢（甚至进程被 SIGSTOP）不会立刻形成写者侧背压，而 rclcpp 的
  publisher 又是异步发送 + `max_blocking_time` 有上限。Classic 那次的机理牵涉
  **写者与读者在同一进程/同一执行器**（`livox_points_plugin.cpp` 的 sensor 回调），
  gz 插件是"PostUpdate 里 publish、DDS 线程发"，**结构上就不同**。
* 因此这条改动的收益是**契约与保证**，不是"本机测出来的性能提升"：
  ① 与 Classic 的 `SensorDataQoS` **逐字对齐**（同一套订阅 QoS 两边都能用）；
  ② best-effort 写者**按定义永不阻塞**，不依赖 DDS 实现/配置的细节；
  ③ 保持 `reliable` 一键可切（`gz_qos:=reliable`），便于将来复现。
* ⚠️ **仍未被验证的**：Classic 那次真机上"停更且不自恢复"的完整链条，本次没有在 gz 侧
  复现，也没有在 Classic 侧复跑对照。

---

## 5. 契约逐条实测（话题 / 类型 / 频率 / frame_id / 字段）

命令形状（全部在 `.tmp_gzlivox2/gz_run.sh` 里逐条跑过，产物在该目录的 `runs/<tag>/`）：

```bash
ros2 topic list / ros2 topic type <t> / ros2 topic info -v <t>
ros2 topic echo --once --field header.frame_id <t>
ros2 topic hz --window 40 <t>
python3 tools/scripts/gzsim/gz_livox_probe.py --frames 120
```

| 项 | 实测值 |
|---|---|
| `/livox/lidar` 类型 | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/lidar/pointcloud` 类型 | `sensor_msgs/msg/PointCloud2` |
| `/livox/imu` 类型 | `sensor_msgs/msg/Imu`（实测 **100 Hz**，`stamp` 严格 +10 ms 单调） |
| `/clock` | `rosgraph_msgs/msg/Clock`（唯一过桥的东西） |
| **仿真频率**（`CustomMsg` header 戳线性回归，120 帧） | **10.0000 Hz**（误差 <1e-12） |
| 墙钟频率（同一段，headless 台架） | **9.72 Hz**（RTF ≈0.97；整栈时 9.48 Hz，见 §8） |
| 每帧点数（中位 / min / max，台架） | **8018.5 / 7854 / 8167**（30000 束 26.7–27.2% 命中） |
| `offset_time` 最大跨度 | **99,996,667 ns = 99.9967 ms**（`i·1e9/point_rate`，`i=29999`） |
| `offset_time` 单调 | ✅ 全帧 |
| `line` 字段取值 | `{0,1,2,3}` |
| 两路 `frame_id` | `livox_frame`（`ros2 topic echo --field header.frame_id` 实测） |
| `/livox/imu` 的 `frame_id` | **`imu_link`**（与 Classic 的 `gazebo_ros_imu_sensor` 一致） |
| PointCloud2 字段 | `x,y,z,intensity`(f32, 0/4/8/12) + `tag`(u8,16) + `line`(u8,17) + `timestamp`(f64,18) |
| `point_step` | **26** |
| `has_rgb` | **false** |
| `height×width` | 1 × 8012（单帧） |
| 启动 → 首帧点云 | **3.00 s**（launch 起到日志出现 `Loaded 800000 scan directions`） |

启动开销分解（`gz.log` 里的 `[MEAS]`/`[MID360]` 行，本机实测）：`mid360.csv` 解析
800,000 行 ≈0.2 s；Embree BVH 从 STL 建一次（15 ms 量级，见
`livox_sensor_prior_art.md` §10.3）；其余是 ign gazebo 起进程 + 场景加载。

---

## 6. 自机排除（robot11 真碰撞几何）

**这是本轮才真正测到的一条**（`livox_sensor_prior_art.md` §10.7 里它还是"未验证"）。

做法：整栈世界 `rmuc2026_gz.sdf` 里放 `robot11`（从本仓 URDF 派生的 SDF，
**113 个真碰撞体**），传感器插件挂在**机器人自己**的 `livox_frame` 上。
同一个世界、同一个位姿，只切一个开关 `<ignore_self>true|false</ignore_self>`：

| 量 | `ignore_self:=true`（生产档） | `ignore_self:=false`（对照档） |
|---|---|---|
| map 系统的建树日志 | `Embree map bootstrapped with **1** supported visuals` | `… with **14** supported visuals` |
| 每帧点数（中位） | **8040.5** | **0**（`n_max` 也是 0） |
| 最近点距离 `r_min` | **0.4292 m** | —（没有点） |
| `r < 0.12 m` 的点 | **0** | — |
| `r < 0.30 m` 的点 | **0** | — |
| RTF | 0.9686 | 0.9985 |

**结论**：
1. **自机排除生效**：机器人 113 个碰撞体**没有**进 BVH（1 vs 14 个 visual），
   点云里没有 `r < 0.43 m` 的回波 —— 最近点 0.429 m 正好是机器人
   `0.6 × 0.6` 底盘的半对角线（0.42 m）之外。
2. **对照档为什么是 0 点而不是"看见自己"**：传感器装在车体内、被自己的碰撞体
   **整个包住**，每条射线都从几何体内部出发 ⇒ 单面几何判不到有效交点 ⇒ **全盲**。
   这比"看见自己"更糟，也更能说明这条排除是**必需**的，不是优化项。
   （`r_min=0.429` 只说明"没有近场回波"，不构成"看不见车体"的独立证据 ——
   `ignore_self` 的 A/B 才是。）
3. ⚠️ 与 Classic 的差别：Classic 的 `livox_frame` **故意不给 collision**，所以它
   "不打自己"是靠**几何缺失**；gz 侧 `livox_frame` 也没有 collision，但**底盘/云台的
   113 个体积**在同一个 model 里 ⇒ 必须靠实体级排除。这条差异写进 §2.9 第 3 条。

---

## 7. 依赖 `rgb` / `PointXYZRGB` 的消费者（grep 结论）

grep 面：`src/**` + `tools/**`，排除 `third_party/`、`build/`、`install/`、`.tmp_*`。

```bash
grep -rn "PointXYZRGB|PointField::RGB|\"rgb\"|'rgb'|setPointCloud2FieldsByString" \
  --include=*.cpp --include=*.h --include=*.hpp --include=*.py src/ \
  | grep -v "third_party/\|ikd-Tree/examples"
```

命中**只有两处**：

| 位置 | 内容 | 是否硬依赖 | 判决 |
|---|---|---|---|
| `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp:271` | Classic 插件**生产** rgb：`modifier.setPointCloud2FieldsByString(2,"xyz","rgb")` | 生产端 | 只影响 Classic；gz 后端不产生 rgb |
| `src/rm_localization/point_lio/include/common_lib.h:66` | `typedef pcl::PointXYZRGB PointTypeRGB;` + `:68 PointCloudXYZRGB` | **不依赖**：全仓没有任何地方使用这两个别名（`grep -rn PointTypeRGB` 只有这两行定义） | 不受影响 |

**按字段名取点、因此 rgb 无关的消费者**：

| 节点 | 代码 | 结论 |
|---|---|---|
| `linefit_ground_segmentation` | `pcl::fromROSMsg<PointXYZ>`（`ground_segmentation_node.cc:107-108`） | PCL 按字段名映射，缺 rgb 只是不填该通道 ⇒ 安全 |
| `pointcloud_to_laserscan` | `PointCloud2ConstIterator<float>("x"/"y"/"z")` | 安全 |
| `patchwork_ground_segmentation` | `pcl::fromROSMsg` + 注释"只取 x/y/z" | 安全 |
| `gicp_registration` | `pcl::fromROSMsg` | 安全 |
| `FAST_LIO` / `point_lio` / `small_point_lio` | 走 `CustomMsg`（不是 PointCloud2） | 安全 |
| `cloud_accumulator` | 按字段名解包 | 安全 |
| `tools/scripts/diag/cloud_z_profile.py`、`costmap_marking_check.py` | 先 `{f.name: f.offset}` 再按偏移解 | 安全（不假设 `point_step`） |
| `tools/scripts/mapping/pcd_stats.py` | 读 **PCD 文件**，不读话题 | 无关 |

**结论**：本仓**没有任何消费者硬依赖 `rgb`/`PointXYZRGB`**，所以 gz 后端去掉 rgb
**不会让任何本仓节点坏掉**。唯一的实际影响是**外部工具**：
RViz2 的 `PointCloud2` 显示若把 `Color Transformer` 设成 `RGB8` 会全黑（改成
`Intensity` 即可）；任何自写的"按 rgb 上色"脚本要改成按 `intensity`。
若将来确实需要 rgb，最小改法是给插件加一个写 `rgb` 字段的分支（`point_step` 变 32）。

---

## 8. 端到端（linefit → /scan + LIO）

命令：`sim_backend:=gz world:=RMUC2026 mode:=mapping lio:=small_point_lio robot:=robot11
robot11_mount:=sensor gui:=False`（`mode:=mapping` = 建图 + LIO，不起 nav2）。

| 链路环节 | 实测 | 结论 |
|---|---|---|
| gz → `/livox/lidar` | **9.479 Hz**（整栈、40 帧窗口） | ✅ 10 Hz 量级 |
| gz → `/livox/lidar/pointcloud` | **9.514 Hz**；单帧 `width` **7940** | ✅ |
| 两路一致性 | 39/39 帧同 stamp 同点数、`frame_id` 相同 | ✅ |
| linefit | `/segmentation/ground`、`/segmentation/obstacle` 正常（日志有 `[slope_speed]` 输出） | ✅ |
| `pointcloud_to_laserscan` | `/scan` = `sensor_msgs/msg/LaserScan`，`frame_id=**base_link**`，实测 418 条有限波束（2.75–8.09 m） | ✅ **linefit → /scan 打通** |
| IMU | `/livox/imu` 100 Hz、`stamp` 严格 +10 ms、`frame_id=imu_link` | ✅ 话题层对 |
| **LIO（`small_point_lio`）** | ❌ **未通过**：节点持续打印 `imu loop back`（`preprocess.cpp:67` 的 `imu_msg.timestamp < last_timestamp_imu`），**没有发出 `odom→base_link`**；`view_frames` 里没有 `odom`/`map` 帧 | ❌ **未验证** |
| `/cloud_registered` | 类型/发布者都在，但 `header.frame_id` 取不到（LIO 没出数据） | ❌ 未验证 |
| TF 单发布者契约 | `/tf` 的真实发布者 = `small_point_lio` + `slam_toolbox` + `robot_state_publisher` + `fake_vel_transform` + `complementary_filter`；`view_frames` 的边：`base_link→livox_frame`、`livox_frame→imu_link`、`base_link→base_link_fake` | ⚠️ 部分：**没有 odom/map 边**（因为 LIO 没起来），"单发布者"这条**没有被真正考到** |

**`/scan` 的 418 条有限波束 + `frame_id=base_link`** 是这轮端到端里最硬的一条：
它证明"斜装 + 重力对齐 + p2l 落回水平面"这条链在 gz 后端上是对的
（`robot11_mount:=sensor` 的既定规则原样生效）。

### 8.1 `imu loop back` 的已知事实与下一步

* 现象：`small_point_lio` 的 IMU 回调里 `imu_msg.timestamp < last_timestamp_imu` 连续触发，
  而**同一时刻用 rclpy 直接收 `/livox/imu` 得到的戳是严格 +10 ms 单调的**
  （381 条，`min=max=10,000,000 ns`，0 条负差）。
* 因此问题**不在发布端**：要么是 LIO 侧的执行器把两条来源的样本交错（同进程内
  `imu_complementary_filter` 只订阅不发布这条话题，已排除双发布者：
  `ros2 topic info -v /livox/imu` 实测 **Publisher count: 1**），要么是它内部
  `last_timestamp_imu` 的更新路径在"先被一条小戳污染后不再更新"的语义下持续报错。
* ⚠️ 该文件属**只读**目录（`src/rm_localization/small_point_lio/**` 归另一个 agent），
  本轮**没有**改它，也没有继续深挖。
* 建议的下一步（按代价排序）：① 换成 `lio:=fastlio` 复跑（`FAST_LIO` 的 IMU 订阅
  QoS 是默认 reliable，与我们的 reliable IMU 写者兼容），确认 gz 后端本身能不能喂出
  odom；② 给 `small_point_lio` 的 IMU 回调加"打印前 20 条的戳"临时日志；③ 检查它是否
  同时被 `imu_complementary_filter` 的 `/imu/data` 影响（remap 只到 `/livox/imu` 的**入**边）。

---

## 9. 默认路径零改动的证据

工具：`tools/scripts/gzsim/launch_increment_diff.py`（用同一组 launch 参数构建两个版本的
**整棵 Action 树**，每个 Action 的全部字段 + condition + 嵌套子 Action 都进比较）。

```bash
git show HEAD:src/rm_nav_bringup/launch/bringup_sim.launch.py > /tmp/old.py
python3 tools/scripts/gzsim/launch_increment_diff.py \
  --old /tmp/old.py --new src/rm_nav_bringup/launch/bringup_sim.launch.py
```

| 比较口径 | 结果 |
|---|---|
| Action 数量 | 旧 **61** → 新 **63** |
| 新增（整棵子树逐字段） | ① `DeclareLaunchArgument name=sim_backend`；② `IncludeLaunchDescription`（`start_gz_simulation`，`condition=LaunchConfigurationEquals('sim_backend','gz')`）；③ `IncludeLaunchDescription`（`start_rm_simulation` 加上了 `condition=…== 'classic'`） |
| **消失/被改** | 1 个 —— 就是 ③ 的那个 `start_rm_simulation`：**内容一字未改，只多了一个默认恒真的门** |
| `--strip-conditions` 后的净变化 | **+2 新增 / 0 删除 / 顺序一致** ⇒ **纯增量 ✅** |
| 共有 Action 的相对顺序 | **一致**（时序不变） |
| `ros2 launch … --show-args` | 默认参数值全部不变（只多一个 `sim_backend`） |

另外：`sim_backend:=gz` 那条 `IncludeLaunchDescription` 的 launch 文件路径用的是
本文件既有的惰性 substitution 机制（`_PackageShareFile`）⇒ **没装 `rm_livox_gz_sim`
时也不会在解析期报错**（与 `lio:=small_point_lio` 等槽位同一套设计）。

---

## 10. 未实现 / 未验证清单（诚实）

**未实现**
1. **底盘运动**：`/cmd_vel_chassis` 不驱动底盘（Fortress 没有 `planar_move` 的对应物）。
   gz 侧的机器人在世界文件里是 `<static>true</static>`。⇒ gz 后端目前只能做
   "静止的感知/定位/建图"验收，**不能**跑 nav2 的移动闭环。
2. **多场地**：只有 `RMUC2026`。其它场地缺 SDF ≤1.9 资产。
3. **其它机器人槽位**：只有 `robot11`（`hzmirm` / 默认模型没有 gz 模型生成路径）。
4. **`worlds/*` 的障碍物模型**：本仓 `meshes/obstacles/*` 没有进 gz 世界
   （Classic 那套 `obstacle_plugin` 是 Classic 插件）。

**未验证**
5. **LIO 端到端**：`small_point_lio` 在 gz 档持续 `imu loop back`、不发 `odom→base_link`
   （§8.1）。⇒ `map→odom→base_link` 的**单发布者契约没有被真正考到**，
   `/cloud_registered` 没有数据。
6. **动态障碍 refit**：`rmagine_embree_map_system` 的 `delta_trans/delta_rot` 分支
   在本轮**没有被走到**（世界全静态、机器人在 gz 侧也是静态）。§10.7 的这条仍然是空的。
7. **`mode:=slam_nav` / `mode:=nav`**：只跑了 `mode:=mapping`。nav2 在 gz 档未验证。
8. **Classic vs gz 的对照性能**：本文件 §2.8 的 Classic 列填的是
   `livox_sensor_prior_art.md` §1 里既有的基线（3.2 GiB / RTF 0.32–0.46），
   **不是**本轮同机同时刻新测的 —— 本轮把机器时间都花在 gz 侧。
9. **GUI（Xvfb）在 gz 档**：脚本已就绪（`.tmp_gzlivox2/gz_run.sh` + Xvfb 拷贝），
   但本轮没有拿到成体系的数字；`livox_sensor_prior_art.md` §10.4 有上一轮的
   9.77 Hz / RTF 0.96–1.0 / 843 MB（那是**台架世界 + 单实例**的口径）。
10. **QoS "堵死"的复现**：见 §4.4，本机两种 QoS 都没复现出来。
11. **`ign sdf -k` 全仓预检**：只对本文件涉及的两个世界做了，没有扫全仓 SDF。
12. **多实例 >2 / 多传感器同世界**：只测了 2 个实例。
13. **CPU 峰值**：`rss.txt` 里的 CPU 是 0.5 s 采样的累计 tick 差，没有 `perf` 火焰图。
