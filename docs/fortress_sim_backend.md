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
| `ros2 launch … --show-args` | 默认参数值全部不变（只多一个 `sim_backend`）。实测输出：`'sim_backend': … (default: 'classic')`，`Valid choices are: ['classic', 'gz']` |

> ⚠️ 本机 `$HOME` 只读，`ros2 launch --show-args` 会先在 `~/.ros/log` 建目录并 OSError
> （**与本次改动无关**，改造前也一样）。跑这类命令前把 `HOME`/`ROS_LOG_DIR` 指到可写目录即可。

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

---

# 第二轮（2026-10-18）：把 gz 后端从"传感器台架"补成"能定位 + 能开的仿真"

> **本节是追加的**，不改上面任何一行历史记录；凡是与本节冲突的旧结论，以本节为准。
> 本节修订/替代的旧条目（逐条点名）：
>
> | 旧条目 | 旧结论 | 现在 |
> |---|---|---|
> | §2.9-6 | `/cmd_vel_chassis` 不驱动底盘，车是静态的 | **已实现**（§12）：运动学底盘驱动 + `/odom_ground_truth` |
> | §8.1 / §10-5 | gz 侧 LIO 不明原因 `imu loop back`、不出 odom | **两个根因都找到了**（§11.1 / §11.2），三个 LIO 槽位实测在 §11.3 |
> | §10-1 | 未实现：底盘运动 | 已实现（§12），**权威 100%**，与 Classic 的 5–16% 不是同一个数（§12.5） |
> | §10-8 | 没有同轮 Classic vs gz A/B | **做了**（§13）：同一会话、同一探针、同一时长 |
> | §10-5 末句 | "单发布者契约没有被真正考到" | **考到了**（§14）：`map→odom` / `odom→base_link` 各 1 个发布者、0 个同 stamp 冲突 |
> | §9 | 默认路径零改动的证据（61→63 Action） | **仍然成立且更强**：本轮的 `bringup_sim.launch.py` 与 HEAD **逐字节相同**（`launch_increment_diff.py`：`新增 0 / 消失 0 / 顺序一致`） |
>
> **本轮新增/改动的文件**（全部在 gz opt-in 路径里，Classic 一行未动）：
>
> | 文件 | 性质 |
> |---|---|
> | `src/rm_simulation/rm_gz_planar_move/`（新包） | 我们的底盘驱动 `librm_gz_planar_move.so`（自有代码，Apache-2.0，无第三方源码） |
> | `src/rm_simulation/rm_livox_gz_sim/launch/gz_sim.launch.py` | + `gz_partition`（每次运行唯一）、+ `gz_drive`、+ `gz_custom_msg_tag`、+ 驱动插件目录进 `GZ_SIM_SYSTEM_PLUGIN_PATH` |
> | `src/rm_simulation/rm_livox_gz_sim/config/ros_gz_bridge.yaml` | + `/cmd_vel_chassis`（`geometry_msgs/Twist` → `ignition.msgs.Twist`，ROS_TO_GZ） |
> | `src/rm_simulation/rm_livox_gz_sim/scripts/make_robot_gz_sdf.py` | + `--drive` / `--drive-odom-rate` / `--custom-msg-tag` |
> | `src/rm_simulation/rm_livox_gz_sim/models/rm_mid360_gz/model.sdf` | + `<custom_msg_tag>0</custom_msg_tag>`（台架世界也一致） |
> | `tools/scripts/gzsim/patches/0004-mid360-custommsg-tag.patch` | 新补丁（上游 ③，`+13/−1`） |
> | `tools/scripts/gzsim/{gz_stamp_probe,gz_contract_check,drive_probe,ab_probe}.py`、`ab_backends.sh` | 新工具（探针/契约/A-B） |

## 11. LIO 在 gz 侧为什么不动 —— **两个独立根因**，都不是 LIO 的锅

> 症状（旧记录）：`small_point_lio` 在 gz 档每条 IMU 都打 `imu loop back`（`preprocess.cpp:67`），
> 且即使不打，也**没有** `odom→base_link`、没有 `/cloud_registered`。本轮证明这是**两件事**。

### 11.1 根因 A：`imu loop back` = **gz transport partition 没隔离**（同机另一个 Fortress 世界串进来了）

**判据链（三条，逐条可复算）**

1. **旧日志里的铁证**（`.tmp_gzlivox2/runs/e5_e2e/launch.log`）：
   ```
   [Wrn] [SimulationRunner.cc:662] Found additional publishers on /stats, using namespaced stats topic only
   [Wrn] [SimulationRunner.cc:695] Found additional publishers on /clock, using namespaced clock topic only
   ```
   这两行只在"**同一个 gz transport partition 上还有别的 gz publisher**"时出现。
   同一轮里 `/clock` 的 ROS 侧实测 **0 条**（`hz_clock.txt` 为空）——因为 gz 见冲突后把钟改发
   **namespaced 话题**，我们的桥订的是 `/clock` ⇒ 静默收不到。这同时解释了那轮
   `use_sim_time` 为什么形同虚设。
2. **判决性实验（本轮新增，可复现）**：故意在**同一个 partition** 上再起一个 Fortress 世界
   （台架世界，`robot:=none`，ROS 换 domain 99 以免 ROS 侧混淆），用**插桩版** LIO 打印每条
   IMU 的 `(ts, last)`：
   ```
   [DBG] imu #5  ts=3.670000000 last=6.170000000 BACK      ← 外来世界的仿真钟（落后 2.5 s）
   [DBG] imu #6  ts=6.180000000 last=6.170000000 ok        ← 我们自己的
   [DBG] imu #7  ts=3.680000000 last=6.180000000 BACK
   ...
   ```
   ⇒ `/livox/imu` 上变成**两个仿真钟交错**；LIO 把"落后的那一路"全判为倒退
   （≈100 条/s，与旧轮 e5 的 **94.3 条/s** 同量级）。同时刻同进程的 rclpy 探针只看我们自己的流
   ⇒ 完全单调（这就是旧轮"发布端测不出问题"的原因：**探针与 LIO 订阅的是同一个 ROS 话题，
   但旧轮的量测窗口/时机不同；污染是"别的进程往同一个 gz 话题里也发"**）。
3. **修好后**：`GZ_PARTITION` 唯一 ⇒ 20 s 内 `imu loop back` **0 条**、IMU 戳严格 `+10 ms`、
   `n_back=0`（§11.3 的表）。

**修在哪**（我们的集成层，不碰 LIO）：
`src/rm_simulation/rm_livox_gz_sim/launch/gz_sim.launch.py`
* 新增 `gz_partition` 参数（默认**空 = 本次运行随机** `rm_gz_<uuid10>`），
  同时设 `GZ_PARTITION` 与 `IGN_PARTITION`（Fortress 读 `IGN_PARTITION`，gz-transport12+ 读
  `GZ_PARTITION`；与既有的 "两个插件路径都设" 同一套策略），作用域覆盖 `ign gazebo`
  **和** `ros_gz_bridge`（桥必须同 partition 才收得到 `/clock`）；
* 想复现"污染"就显式给同一个名字：`gz_partition:=shared_x`（用于对照实验）。

### 11.2 根因 B：`tag=0x10` ⇒ `small_point_lio` 把**每一帧的每个点**都丢掉（所以就算 IMU 干净也不出 odom）

**机理（源码级，逐字）**
* gz 插件（上游 ③）在 `PublishCloud()` 里给每个点写死 `const std::uint8_t tag = 0x10;`
  （PC2 的 `tag` 字段与 CustomMsg 共用这个值）。
* `small_point_lio` 的 `lidar_adapter/livox_custom_msg.h` 只保留
  `(point.tag & 0b00111111) == 0` 的点 ⇒ `0x10 & 0x3f = 0x10 ≠ 0` ⇒ **0 个点入队**。
* `Preprocess::on_point_cloud_callback` 于是永远不往 `point_deque` 里放东西；而
  `SmallPointLio::handle_once()` 的初始化条件是
  `point_deque.size() >= init_map_size (=10)` **且** `imu_deque.size() >= 200`
  ⇒ `is_init` 永远 false ⇒ 既不 `publish_odometry()`（无 `/Odometry`、无 `odom→base_link`），
  也不 `pointcloud_callback()`（无 `/cloud_registered`）。
* **Classic 侧为什么没这个问题**：`libros2_livox.so` 只填 `x/y/z/reflectivity/offset_time`，
  `tag`/`line` 留默认 **0**（`livox_points_plugin.cpp` 的 `CustomPoint p;` 之后没有写 `p.tag`）。
  ⇒ 本仓既有契约就是 `tag=0`。
* **为什么 FAST-LIO / Point-LIO 收 0x10**：它们的 preprocess 条件是
  `(tag & 0x30) == 0x10 || (tag & 0x30) == 0x00`（`FAST_LIO/src/preprocess.cpp:120`、
  `point_lio/src/preprocess.cpp:106`）⇒ `0x10` 在白名单里。所以这个 bug **只打 small_point_lio**。

| 消费者 | 接受的 tag | `0x10`（上游写死） | `0x00`（Classic / 现在） |
|---|---|---|---|
| `small_point_lio` | `(tag & 0x3f) == 0` | ❌ 全丢 | ✅ |
| `FAST_LIO` | `(tag & 0x30) ∈ {0x00,0x10}` | ✅ | ✅ |
| `point_lio` | 同上 | ✅ | ✅ |

**修在哪**（我们的 gz 集成层，不碰 `small_point_lio`）：
* 新补丁 `tools/scripts/gzsim/patches/0004-mid360-custommsg-tag.patch`：插件读
  `<custom_msg_tag>`、**默认 0**（上游值 16 可复现）；
* `make_robot_gz_sdf.py` 生成的 SDF 与 `models/rm_mid360_gz/model.sdf` 都显式写
  `<custom_msg_tag>0</custom_msg_tag>`（两个出口 PC2/CustomMsg 仍然**同值**，§3 的"逐点相同"
  结论不受影响）；
* `fetch_and_patch_mid360.sh` 打 0004（幂等，`git apply --reverse --check` 判定）。

> **为什么不在 `small_point_lio` 里放宽 tag 过滤**：那会让"Classic 契约 = 0"这条隐式约定
> 变成"两个后端各有各的 tag"，而 `src/rm_localization/small_point_lio/**` 也不是本轮的写入面。
> 让 gz 侧发**和 Classic 逐字节一致**的东西，是唯一不需要给下游加分支的修法。

### 11.3 三个 LIO 槽位在 gz 侧的实测（同一世界、同一 spawn、`mode:=mapping`）

| 槽位 | 结果 | 关键证据 | 为什么 |
|---|---|---|---|
| `lio:=small_point_lio` | ✅ **通过**（需 A+B 两个修） | `odom→base_link` **100.00 Hz**；`/odom` 1 发布者；`/cloud_registered` 有数据（86–97 Hz）；`map→odom` 10.2 Hz；TF 链 `map→odom→base_link` 解得开（§13/§14） | 它自己发 TF；A 修好 IMU 单调、B 修好点云入队 |
| `lio:=fastlio` | ✅ **通过**（本来就能过 A） | `odom→base_link` **10.00 Hz**（由 `lio_tf_adapter` 发）、`/cloud_registered` 发布者 `laser_mapping`、`camera_init→body` 10 Hz、`/livox/imu` 订阅是 **reliable depth-10**（与我们的 reliable IMU 写者兼容） | FAST-LIO 的 tag 白名单含 `0x10`；它的 IMU 订阅 QoS 与我们的写者匹配 |
| `lio:=pointlio` | ✅ **通过**（但要先补一个目录，**与 gz 无关**） | 未补目录时：节点在**第一帧**就 `terminate called after throwing ... pcl::IOException ... [pcl::PCDWriter::writeBinary] Error during open!` —— 因为 `pointlio_mid360_sim.yaml` 是 `pcd_save.pcd_save_en: true` + `interval: 100`，而它写的是**编译期** `ROOT_DIR`=`src/rm_localization/point_lio/` 下的 `PCD/`，该目录在本次 checkout 里不存在。**`mkdir -p src/rm_localization/point_lio/PCD` 之后**：`odom→base_link` **10.00 Hz**（`lio_tf_adapter` 发）、`/cloud_registered` 发布者 `laserMapping`、`camera_init→aft_mapped` 10 Hz、`/odom` 1 发布者、`imu n_back=0`（1168 条） | Point-LIO 的 `(tag & 0x30)` 白名单含 `0x10`，所以它**不需要** 0004 那个修；阻塞它的只是自己的落盘路径 |

**IMU 侧的统一结论**：三个槽位共用同一条 `/livox/imu`（发布者 `mid360_gz_custom`，reliable
keep_last 20；`small_point_lio` 用 best-effort 订、FAST-LIO 用 reliable 订，两种都兼容）。
本轮在 **20 s 窗口内 1900+ 条样本上 `n_back=0`、`dt_med=dt_min=dt_max=10 ms`**（§14.3）。

## 12. 底盘运动：`/cmd_vel_chassis` → 车体运动（**新实现**）

### 12.1 为什么是"我们自己的运动学系统插件"（四条候选逐条否掉）

| 候选 | 能不能用 | 判决理由 |
|---|---|---|
| `gz::sim::systems::DiffDrive` | ❌ | 要**轮子关节**。robot11 的 URDF 里 `j2..j11` 是云台/炮管（`.xacro` 自己写着"不加轮子控制器"），没有可驱动的轮子 |
| `gz::sim::systems::JointController` | ❌ | 同上（没有轮子关节；只能控已有关节） |
| `gz::sim::systems::VelocityControl` | ❌（实测机理） | 它写 `LinearVelocityCmd/AngularVelocityCmd` 让物理引擎积分。这条在 Classic 上被实测否过：多刚体 + 接触约束每步把速度"掰回来"，角速度执行率被吃到 1% 量级（`tilted_lidar_fidelity.md` §M.3）。本仓 gz 侧的模型是 `<static>`（URDF 派生的 SDF 只保留 `livox_frame`/`imu_link` 两个 fixed 子 link），改动态要重建整套关节/惯量/接触 |
| `libgazebo_ros_planar_move.so` | ❌ | Classic 专有（`gazebo/physics`），Fortress 加载不了 |
| **自写 `librm_gz_planar_move.so`（本轮）** | ✅ | 运动学、平面、与 planar_move **同话题/同单位/同语义**；不需要关节、不需要接触；权威可精确复算 |

### 12.2 机制（以及一个必须知道的坑：**只能在 `Update` 阶段写**）

gz-sim 的一步是 `PreUpdate(所有系统) → Update(所有系统) → PostUpdate(所有系统)`，而**物理系统在
`Update` 里"步进 + 把 link/sensor 的 `WorldPose`/`AngularVelocity`/`LinearAcceleration` 从物理
引擎写回"**（gz-sim6 `Physics::Update` → `PhysicsPrivate::UpdateSim`，`src/systems/physics/Physics.cc:841`）。
本轮**踩过**：第一版把角速度写在 `PreUpdate`，物理系统随后把它覆盖成"静止车 = 0" ⇒ 实测
`wz=0.5 rad/s` 命令下 `/livox/imu` 的陀螺均值 **0.0000** ⇒ LIO 会认为车没转。
改到本插件自己的 `Update`（模型插件后加载 ⇒ 在物理之后、所有 PostUpdate 之前）后，陀螺均值
**0.4000/0.4988**（对应命令 0.4/0.5）。

每步写四类 ECM 数据（`<static>true</static>` 保持不动，物理引擎不参与）：
1. `components::Pose`（**model**，世界系）—— 射线插件用 `gz::sim::worldPose()`（沿父链读 `Pose`）
   算传感器位姿，读的就是它；
2. `components::WorldPose`（model 的每个 link）；
3. `components::WorldPose` + `components::AngularVelocity`（**IMU 传感器实体**：
   `ignition-gazebo-imu-system` 读的是传感器实体的这两个组件，不写陀螺恒 0）；
4. `/odom_ground_truth`（`nav_msgs/Odometry`，**本进程内** rclcpp 直发，与 mid360 插件同一做法）。

积分：`yaw += wz·dt`；`p += R(yaw)·(vx,vy,0)·dt`（`vx` 在**车体系**，与 planar_move 一致）。

### 12.3 命令通道（ROS 2 → gz，**过桥**）

```yaml
# src/rm_simulation/rm_livox_gz_sim/config/ros_gz_bridge.yaml（追加）
- ros_topic_name: "/cmd_vel_chassis"
  gz_topic_name:  "/cmd_vel_chassis"
  ros_type_name:  "geometry_msgs/msg/Twist"
  gz_type_name:   "ignition.msgs.Twist"
  direction:      ROS_TO_GZ
```
插件订阅的是 **gz** 的 `/cmd_vel_chassis`（`gz::transport`），所以
`ros2 topic info -v /cmd_vel_chassis` 看到 ROS 侧的发布者（nav2/脚本）**和桥**，
`ign topic -i -t /cmd_vel_chassis` 看到 gz 侧的桥发布者 —— 两边各自可查。
**为什么命令过桥、而 IMU/点云不过桥**：输入侧只有这一条命令，过桥让"ROS↔gz 边界"只有一处；
输出侧的 IMU/点云留在进程内是为了 QoS/frame_id/零拷贝（理由见 §1）。

### 12.4 实测权威（`tools/scripts/gzsim/drive_probe.py`，真值 = `/odom_ground_truth`）

协议：阶段 A 只发 `wz`、阶段 B 只发 `vx`，各按**仿真时间** 4 s（用消息戳做尺，RTF<1 也公平）。
命令以 200 Hz 持续发（`command_timeout: 0.0` ⇒ 与 planar_move 一样"最后一条命令一直生效"）。

| 量 | 命令 | 实测 | 比率 |
|---|---|---|---|
| 角速度（`wz=0.5 rad/s`） | 28.65 °/s | **28.65 °/s**（Δyaw 114.59°/4.0 s） | **1.000** |
| 角速度（`wz=0.4 rad/s`） | 22.92 °/s | **22.92 °/s**（Δyaw 91.67°/4.0 s） | **1.000** |
| 线速度（`vx=0.3 m/s`） | 0.300 m/s | **0.3000 m/s**（Δs 1.2030 m/4.01 s） | **1.000** |
| `/livox/imu` 陀螺 z（`wz=0.4` 期间 399 条均值） | 0.4 rad/s | **0.4000 rad/s** | **1.000** |
| **LIO 的 `/odom` 同期 Δyaw**（同一次 run） | 91.67°（真值） | **89.70°**（97.9%） | — |

第三、四行是"车真的动了 + 传感器真的看见了"的闭环证据：**驱动写的角速度进了 IMU，
LIO 用点云 + 陀螺把 91.7° 的旋转估成 89.7°**。

### 12.5 与 Classic 的**不同**（诚实清单，别当成"对齐了"）

| 项 | Classic（`libgazebo_ros_planar_move.so` + 焊接底盘） | gz（本轮） |
|---|---|---|
| 命令语义 | `/cmd_vel_chassis`（Twist）→ 车体 | **同**（同话题、同单位、`vx` 车体系、最后命令生效） |
| **权威（执行率）** | 实测 **5.15%**（整栈 `wz=1.0`×12 s：+35.42°/687.5°，`tilted_lidar_fidelity.md` §M.7）；台架焊接单刚体 **16.2–16.5%**（§M.3） | **100%**（运动学直接积分，无摩擦/接触） |
| 碰撞响应 | 有（设速度让 ODE 积分 ⇒ 撞墙被挡） | **没有**（运动学穿墙）——要让 gz 等价就得把模型改动态 + 造轮子/接触，等于把 §M 已修好的事再坏一次 |
| 加速度计 | 有向心/切向项 | **没有**（只保留 gz 物理给的重力项）；陀螺有 |
| 真值里程计 | `/odom_ground_truth`（planar_move 发） | **同名同语义**（本轮补上，§2.9-9 的缺口） |
| 其它 | — | 平面（z、roll、pitch 恒定），无差速/阿克曼/悬挂 |

## 13. 同一轮 Classic vs gz 的 A/B（**首次**）

### 13.0 协议与命令（两个后端逐字相同，只换 `sim_backend`）

```bash
# 一条命令跑完两个后端（顺序执行，各自独占 domain/URI/partition/HOME；只杀自己的进程）
tools/scripts/gzsim/ab_backends.sh ab_1840 20

# 它内部就是这一条（每个后端各跑一次）：
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    sim_backend:=<classic|gz> world:=RMUC2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0 \
    nav_rviz:=False lio_rviz:=False gz_partition:=<唯一名>
# 同一个探针（launch 那一刻就开始等首帧，然后量 20 墙钟秒）：
python3 tools/scripts/gzsim/ab_probe.py --backend <b> --duration 20 \
    --launch-epoch <launch 的 time.time()> --pid <launch pid> \
    --log <launch.log> --out <b>/metrics.json
```

* 机器：同一台、同一个 20 s 窗口、先后各一次（**不并发**，并发会互相抢 CPU）；
* 世界：**同一份**场地资产 `RMUC2026_world`（同一 STL）、同一起始点 `x=10.925, y=2.525, yaw=0`；
  ⚠️ **两处已知不同**（不是本轮引入的）：(a) 起始 `z`：Classic 0.2（落地一下），gz 0.1025
  （静态直接到位）；(b) Classic 世界里**额外**有 `meshes/obstacles/*` 的障碍物模型，gz 世界
  **还没有**（§10-4 的既有缺口）——这正是 `/scan` 近距回波差异的来源（见下）。
* 采样窗口 = 收到首帧之后的 20 墙钟秒；RTF 用 `/clock` 的 Δ仿真/Δ墙钟。

### 13.1 并排表（`ab_1840`，2026-10-18 18:40 本地）

| 指标 | **Classic 11** | **gz / Fortress 6.18** | 变化 |
|---|---|---|---|
| RTF（`/clock` 仿真/墙钟） | **0.431** | **0.973** | **×2.26** |
| `/livox/lidar` 仿真 Hz | 10.000 | 10.000 | 一致 |
| `/livox/lidar` 墙钟 Hz | 4.315 | 9.725 | ×2.25 |
| 每帧点数（中位 / min / max） | 11365 / 10999 / 11511 | 8050 / 7893 / 8204 | −29%（含 Classic 的自机回波 + `max_range` 口径差，见 §13.2-2） |
| **逐帧求交 ms**（插件插桩，中位） | **无插桩**（ODE 逐射线，见下行） | **0.243**（min 0.231 / max 0.264，10 次采样，双实例合计 ≈0.49 ms/帧） | — |
| **gzserver CPU ms / 帧**（统一口径，两个后端可比） | **176.2** | **57.0** | **−68%** |
| gzserver RSS 中位（MB） | **3071** | **297** | **−90%** |
| 整棵进程树 RSS 中位（MB） | 3406 | 664 | −80% |
| gzserver CPU%（单核） | 76.5 | 55.7 | −27% |
| 整棵树 CPU%（单核） | 87.1 | 159.7 | +83%（下游在同一台机器上跑得更快 ⇒ 吃更多 CPU） |
| 启动 → 首帧（s） | 7.06 | **3.65** | −48% |
| `/scan` 有限波束数（中位） | 465 | 402 | −14% |
| `/scan` range min / max（m） | **0.19 / 9.92** | **2.64 / 9.87** | 见 §13.2 第 2 条（Classic 含自机回波） |
| `/cloud_registered` 条数（20 s 窗口） | 67（3.3 Hz） | 1730（86.5 Hz） | 见 §13.2 第 3 条 |
| `/cloud_registered` width 中位（点/条） | 11388 | 841 | 同上 |
| 端到端 LIO | ✅ `odom→base_link` 10.00 Hz | ✅ `odom→base_link` 100.00 Hz | 两个后端都通 |

`/tmp` 里的原始 JSON：`.tmp_gz_ab/ab_1840/{classic,gz}/metrics.json`（`contract.log` 同目录）。

### 13.2 怎么读这张表（三条，缺一条就会误读）

1. **"求交 ms"两个后端的口径不同，不能直接减**：gz 一列是插件自己插桩的
   `Embree average raycast: X ms for 30000 rays`（每 100 帧一条运行均值，取中位）；
   Classic 的雷达是 `gzserver` **进程内**的 ODE 插件，**没有任何插桩**，所以只能给
   "gzserver CPU 秒 / 帧"这个**统一口径**（176 ms/帧 vs 57 ms/帧）。**结论只能写到
   "gz 的求交本体是 0.24 ms/帧量级、Classic 的服务端每帧成本高 3 倍"**，不能写
   "0.24 ms vs 176 ms 是同一种测量"。
2. **`/scan` 近距差异（0.19 m vs 2.64 m）的主因是"自机回波"，不是传感器坏了**：
   Classic 的 ray 插件挂在 `livox_frame` 上，`livox_frame` 自己没有 collision，但**同模型其它
   link（底盘/轮/云台）有** ⇒ Classic 的 `/livox/lidar/pointcloud` 里**含自身回波**（正因为
   如此 linefit 才有那份"113 个自身 collision AABB"的**自击掩膜**；掩膜只作用于判据/限速，
   **不动** `/segmentation/*` ⇒ `/scan` 里仍然有近距自击点）；
   gz 侧是**模型级**排除（`ignore_self` 把整车 113 个碰撞体从 BVH 里剔掉，§6 的 A/B 实测
   0 点）⇒ gz 的 `/scan` 里没有自击点，最近的 2.64 m 是**真障碍**。
   ⚠️ 两个世界的**其它资产是同一份**：Classic 的 `RMUC2026_world.world` 里也只有场地 STL
   （没有额外障碍模型，本轮已用源码核对），所以这条差异**不是**"世界不同"造成的。
   点数差（11365 vs 8050，差 3315 ≈ 11% 射线）与"Classic 打到自己"的量级相符，但本轮
   **没有**做逐点归因（列为未验证）。另有一处**配置口径**不同（未在本轮统一）：
   Classic 的 ray `<range>` 是 **min 0.1 / max 200**，gz 插件是 `min_range 0.1 / max_range 40`。
3. **`/cloud_registered` 的粒度差异有明确机理**：gz 的 `CustomMsg` 每个点带
   `offset_time = i·1e9/point_rate`（跨 **100 ms**，像真机），Classic 的 `offset_time` **恒 0**。
   `small_point_lio` 的 `handle_once()` 在 `imu.front().timestamp < point.back().timestamp`
   时发布 ⇒ gz 侧几乎每条 IMU 都发（≈86–97 Hz、每次只发"上次发布以来新收的点"≈841 个），
   Classic 侧每帧发一次（3.3 Hz、11388 个）。**总量级相同（~72k vs ~37k 点/s）、语义相同
   （都是 odom 系里的配准点云）**；差别是"帧的切分"。如果哪天需要与 Classic 逐帧对齐，
   给插件加一个 `<offset_time_span>0</offset_time_span>` 开关即可（本轮未做）。

## 14. 单发布者契约（`map→odom` / `odom→base_link`）—— gz 侧**首次真正考到**

### 14.1 工具与口径

```bash
python3 tools/scripts/gzsim/gz_contract_check.py --duration 12 \
    --out contract.json --expect odom:base_link --expect map:odom
```
三路证据（ROS 2 的 `TFMessage` **不带 authority**，`view_frames` 里所有 broadcaster 都是
`default_authority`，所以不能只看内容）：
1. `/tf` 与 `/tf_static` 的**位点清单**（`ros2 topic info -v` 的每个 PUBLISHER：node 名 + GID）；
2. **逐边消息率 + 同 stamp 冲突检测**：两个节点发同一条边时，率会≈相加，且同一 stamp 会出现
   **两个不同变换值**（脚本逐条比较，记 `stamp_conflicts`）；
3. `tf2_monitor` 的逐帧统计作为第二意见。
另外查 gz 侧：`ign topic -i -t /mid360/imu_gz` 的 `Publishers:` 必须 = 1（= §11.1 的回归判据），
`/clock` 必须 ≤1。

### 14.2 计数（同一轮 A/B 的两个 `contract.log`）

| 边 | Classic 11 | gz（`lio:=small_point_lio`） | gz（`lio:=fastlio`） |
|---|---|---|---|
| `odom → base_link` | **1 条边**，10.00 Hz，`stamp_conflicts=0` | **1 条边**，100.00 Hz，`stamp_conflicts=0` | **1 条边**，10.00 Hz，`stamp_conflicts=0` |
| `map → odom` | **1 条边**，108.5 Hz，`stamp_conflicts=0` | **1 条边**，51.6 Hz，`stamp_conflicts=0` | **1 条边**，63.8 Hz，`stamp_conflicts=0` |
| `/tf` 发布者数（全部边） | 6（`robot_state_publisher`/`slam_toolbox`×2/`fake_vel_transform`/`complementary_filter`/`small_point_lio`） | 6（同上，末项换成 `small_point_lio`） | 7（+ `laser_mapping`+`lio_tf_adapter`，- `small_point_lio`） |
| `/tf_static` 发布者数 | 1（`robot_state_publisher`） | 1 | 1 |
| `/odom` 发布者数 | 1 | 1 | 1 |
| `/cloud_registered` 发布者数 | 1 | 1 | 1 |
| `/livox/lidar`（CustomMsg）发布者数 | 1（`livox_frame_plugin`） | 1（`mid360_gz_custom`） | 1 |
| `/livox/imu` 发布者数 | 1（`imu_plugin`） | 1（`mid360_gz_custom`） | 1 |
| gz 侧 `/mid360/imu_gz` 发布者数 | — | **1** | **1** |
| IMU 单调性（20 s） | `n_back=0`（459 条） | `n_back=0`（1162 条，`dt=10 ms` 恒定） | `n_back=0`（1168 条） |

⇒ **`map→odom→base_link` 的每一段都恰好一个发布者**，且 gz 侧同 stamp 冲突 = 0。
（`map→odom` 的"率"两后端都远高于 10 Hz：slam_toolbox 每帧重发一次，不是多发布者——
`stamp_conflicts=0` 且发布者清单里只有 `slam_toolbox`、位点数 1。）

### 14.3 为什么这次才算"真考到"

旧轮 §8 的判决是"⚠️ 部分：没有 odom/map 边（因为 LIO 没起来）"。本轮 gz 侧 **LIO 在跑**，
两条边都真实存在、都能被 `tf2_echo` 解出来：

```
$ ros2 run tf2_ros tf2_echo map base_link
At time 13.170000000
- Translation: [0.013, -0.029, -0.000]
- Rotation: in RPY (degree) [-0.014, -0.003, 0.182]

$ ros2 run tf2_tools view_frames        # 逐边 rate
base_link:   parent 'odom'       rate 100.206
odom:        parent 'map'        rate  10.208
livox_frame: parent 'base_link'  rate 10000.000 (static)
imu_link:    parent 'livox_frame' rate 10000.000 (static)
```

## 15. 更新后的「未实现 / 未验证」清单（**替代 §10**）

**仍未实现**
1. **障碍物模型没进 gz 世界**（§10-4 仍在）：`meshes/obstacles/*` 没有对应的 gz SDF
   ⇒ 若将来在 Classic 世界里放了障碍物，gz 世界不会有。（本轮核对了 `RMUC2026_world.world`：
   它**只**含场地 STL，所以**本轮 A/B 的 `/scan` 差异不是这一条造成的**，而是自机回波，见 §13.2-2。）
2. **底盘无碰撞响应**（§12.5）：运动学穿墙；要等价得把模型改动态 + 造轮子/接触。
3. **加速度计无向心/切向项**（§12.5）。
4. 多场地（只有 `RMUC2026`）、其它机器人槽位（只有 `robot11`）——§10-2/§10-3 不变。
5. `/cloud_registered` 的帧切分与 Classic 不同（§13.2-3）；要逐帧对齐需给插件加
   `<offset_time_span>` 开关（未做）。

**仍未验证**
6. **`mode:=slam_nav` / `mode:=nav`** 在 gz 档：本轮只跑了 `mode:=mapping`（nav2 仍未验）。
7. **动态障碍 refit**（`rmagine_embree_map_system` 的 `delta_trans/delta_rot`）：世界与机器人都
   还是静态模型（驱动是运动学改 pose，不是物理运动）⇒ 该分支**仍未走到**。
8. **Classic 侧的"求交 ms"**：无插桩，只有 `gzserver CPU ms/帧` 的统一口径（§13.2-1）。
9. **`lio:=pointlio` 的落盘路径**：`pcd_save_en: true` 会往编译期 `ROOT_DIR/PCD/` 写 PCD，
   该目录不存在时**第一帧就 terminate**（§11.3）。本轮只**建目录**（未改它的配置），
   跑通后删掉了产出的 PCD；**没有**在 Classic 侧复跑同一崩溃（同一份代码、同一条路径，按机理两边都会崩）。
10. **CPU 峰值 / `perf` 火焰图**：仍只有 0.5 s 采样的瞬时值（§10-13）。
11. **`ign sdf -k` 全仓预检**、**多实例 >2**、**GUI 成体系数字**：§10-9/§10-11/§10-12 不变。
12. **同轮 A/B 只做了一轮（`ab_1840`）**：±3% 级别的抖动没有做重复实验；`RTF ×2.26` 与
    `RSS −90%` 是量级结论，不是精密对照。

## 16. 新增命令速查（复制即可）

```bash
# ① 一次性：取上游 + 打 4 个补丁 + 编译（含 0004 的 custom_msg_tag）
tools/scripts/gzsim/fetch_and_patch_mid360.sh

# ② 本仓编译（新增包 rm_gz_planar_move 会自动跳过"没有 Fortress"的机器）
colcon build --symlink-install

# ③ 整栈 + 可开（gz）：LIO 出 odom、底盘能动
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz world:=RMUC2026 \
    mode:=mapping lio:=small_point_lio robot:=robot11 robot11_mount:=sensor \
    gui:=False spin_speed:=0.0

# ④ 开一下（命令从 ROS 走桥进 gz；真值看 /odom_ground_truth）
ros2 topic pub -r 50 /cmd_vel_chassis geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.4}}"
#   ↑ 想量"命令 vs 实测权威"就用脚本（含 IMU/LIO 同向核对）：
python3 tools/scripts/gzsim/drive_probe.py --wz 0.4 --vx 0.3 --duration 4 \
    --lio-odom-topic /odom --out /tmp/drive.json

# ⑤ 契约：TF 单发布者 + gz 通道独占性（gz 侧必须 1 个 /mid360/imu_gz 发布者）
python3 tools/scripts/gzsim/gz_contract_check.py --duration 12 \
    --out /tmp/contract.json --expect odom:base_link --expect map:odom

# ⑥ 同轮 Classic vs gz 并排（顺序跑，各自独占 domain/URI/partition）
tools/scripts/gzsim/ab_backends.sh ab_$(date +%H%M) 20

# ⑦ 时间戳探针（查 `imu loop back` 那一类问题；同时用 LIO 的 best_effort 与 FAST-LIO 的 reliable）
python3 tools/scripts/gzsim/gz_stamp_probe.py --duration 10 --out /tmp/stamps.json

# ⑧ 复现"partition 污染"（对照实验：两个世界共享 gz 话题 ⇒ IMU 戳交错）
ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=robot11 gui:=False gz_partition:=shared_x
ros2 launch rm_livox_gz_sim gz_sim.launch.py gz_world:=bench robot:=none gui:=False gz_partition:=shared_x
```

**回退**：`sim_backend` 不传即 Classic（行为与改动前逐字节相同）；只想退驱动：
`gz_drive:=false`；想复现两个旧 bug：`gz_custom_msg_tag:=16`（点云全丢）+
`gz_partition:=<同一个名字>`（IMU 交错）。
