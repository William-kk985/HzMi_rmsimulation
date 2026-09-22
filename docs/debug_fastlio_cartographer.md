# 调试手册：建图链路 `mode:=mapping` + `lio:=fastlio` + `mapper:=cartographer`（实战沉淀）

> **本文是一次完整排查的可复用路径**：从"RViz 里地图跟着车转"一路查到根因（时间基 → 先验源 → QoS 丢帧 → 稀疏扫描 → 栅格清除），
> 含**分层判读法、三个工具、实测基线数字、症状→判据→处置速查表、以及踩过的坑**。
> 相关：`issues_and_findings.md`（#19–#25 是这条链上的每一环）、`sim_real_contract.md`（改动该落在 sim/real 哪一侧）、
> `architecture.md` §3.2.4（谁吃什么）、`tf_interface_contract.md`（帧契约）。
>
> 适用症状：地图跟着车转 / 地图飞 / 残影 / 特征先有后没 / 建图时好时坏 / `/scan` 频率异常 / cartographer 崩 `exit -6`。

---

## 0.1 这套链路是什么（本文的适用范围）

**本文所有数字与结论都在这一套上实测得到：**

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 \
    mode:=mapping lio:=fastlio mapper:=cartographer
```

| 角色 | 节点 | 输入 → 输出 | 本文涉及的文件 |
|---|---|---|---|
| 数据源（仿真替身） | Gazebo `LivoxPointsPlugin` + `imu_plugin` + `mecanum_controller` | `/livox/lidar`(CustomMsg)、`/livox/lidar/pointcloud`(PC2)、`/livox/imu`、`/odom_ground_truth` | `livox_points_plugin.cpp`（时间基、假点）、`sentry_robot_sim.xacro` |
| **LIO** | `fastlio_mapping` | `/livox/lidar` + `/imu/data` → `/odom`（+ 孤立岛 `camera_init→body`） | `FAST_LIO/src/laserMapping.cpp`（`child_frame_id`） |
| 里程计 TF | `lio_tf_adapter` | `/odom` → TF `odom→base_link`（含杆臂 `xyz`） | `lio_tf_adapter.yaml` |
| 感知① | `ground_segmentation`（linefit） | `/livox/lidar/pointcloud` → `/segmentation/obstacle` | `ground_segmentation_node.cc`（**订阅 QoS**）、`segmentation_sim.yaml` |
| 感知② | `pointcloud_to_laserscan` | `/segmentation/obstacle` → `/scan`（高度带 -1.0~0.1） | `laserscan_params.yaml` |
| **mapper** | `cartographer_node` + `cartographer_occupancy_grid_node` | `/scan` + **`/odom_ground_truth`**（先验）→ TF `map→odom` + `/map` | **`cartographer.lua`**、`bringup_sim.launch.py`（传 `odom_topic:=/odom_ground_truth`） |
| 其它 | `robot_state_publisher`（URDF 静态边）、`fake_vel_transform`、`rviz2` | — | `sentry_robot_sim.xacro` |

**哪些结论换配置后仍适用 / 要重测：**

- ✅ 仍适用：时间基（#19）、QoS 大消息丢帧（#25）、TF 单父边、`map→odom` 三态判读、栅格清除（§5.2）。
- 🔁 要重测：先验源（#21）—— `lio:=cartographer`（全包形态）没有 LIO 的 `/odom`，用的是 `cartographer_lio*.lua`；换 `mapper:=slam_toolbox` 时 `cartographer.lua` 的参数完全不适用（但 QoS/时间基/扫描率三条照样适用）。
- ⚠️ 与 `localization`（amcl/icp/纯定位）**不能同时开**：`map→odom` 只能有一个发布者。

---

## 0. 一句话结论（本轮根因链）

```
① 插件时间基混用（header 仿真钟 / 逐点 offset 墙钟）
      → /odom 时间戳带墙钟抖动 → cartographer 时间序 CHECK → exit -6            (#19)
② 先验源给错：用 LIO 自己的 /odom（同源 + 晚到）
      → 反馈回路 + 撞 CHECK；改为**独立底盘 odom**（/odom_ground_truth）        (#21)
③ linefit 订阅 QoS 与插件发布端不对齐（BEST_EFFORT 收 RELIABLE 发的 480KB 大消息）
      → 分片丢失不重传 → 2182 帧只收到 68 帧（3%）                              (#25)
      → /scan 只有 0.31~3.3Hz 且带秒级空洞
④ cartographer 在扫描之间"朝向不转"，每来一帧扫描补一刀
      → map→odom 84 次正向大跳、单步最大 76.8°、累计 66° ⇒ **地图跟着车转**     (#23)
⑤ 修好 ③ 后地图不再跟转；剩下的"特征先有后没"是**栅格清除**问题：
      /scan 有 37% 的角度 bin 是 inf（无回波：地面被 linefit 去掉 + 上视射线打空），
      cartographer 把这些方向**清到 missing_data_ray_length=3m** ⇒ 3m 内的内部小墙
      一旦不再被命中就被主动擦掉 ⇒ "扫描到就有、被挡住就没了"                  （本文 §5.2）
```

---

## 1. 分层判读法（**先定量，再猜**）

| 层 | 看什么 | 命令 | 判据 |
|---|---|---|---|
| **L0 时钟/时间基** | 各话题 `header.stamp` 是否同轴 | `ros2 topic echo <t> --field header.stamp --once`、`/clock` | 出现 `6213559xx`（墙钟 .NET ticks 被当 ns）这类值 = 混时钟 |
| **L1 输入流** | **每个话题**：平均率 / 中位间隔 / **最大空洞** / 空洞数 / 戳异常 | `python3 tools/analyze_slam_bag.py --bag <bag>` | ⚠️ **只看中位间隔会骗人**：/scan 中位 300ms 像 3.3Hz，实际平均 1.67Hz、空洞 6.6s |
| **L2 感知链逐段** | 插件 → linefit → p2l 各自出帧率 | 同上（bag 里录 `/livox/lidar/pointcloud`、`/segmentation/obstacle`、`/scan`） | 保留率掉在哪一段，就是那一段的问题 |
| **L3 调度/CPU** | 是否算力饱和 | `top -H -b -n6 -d1 -p $(pgrep -x ground_segmentation_node)` | CPU 空闲 + 输出低 ⇒ **不是算力，是交付/阻塞** |
| **L4 传输/QoS** | 两端 Reliability/Depth | `ros2 topic info <t> --verbose \| grep -E "Node name\|Reliability\|History"` | RELIABLE 发 + BEST_EFFORT 收 = 大消息静默丢帧（见 §5.3） |
| **L5 算法/参数** | 单个算法本身要多久 | `tools/seg_bench_offline.cc`（离线喂数据计时） | linefit 实测 1.01ms/帧 ⇒ 排除算力 |
| **L6 TF 与帧契约** | 帧树单一父边 + `map→odom` 曲线形态 | `ros2 run tf2_tools view_frames`、`python3 tools/monitor_map_odom.py` | 多父边 = 查找结果在两条路径间跳；`map→odom` 分三态（§5.1） |

**核心原则**：每一步都要能"一句话给数字"。说不清数字的猜测，先别改配置。

---

## 2. 工具（本轮新建，都在 `tools/`）

| 工具 | 用途 | 关键判据 |
|---|---|---|
| `monitor_map_odom.py` | **在线**采样 `map→odom`，判"平滑 / 锯齿 / 高频抖动" | P95(\|Δyaw\|) > `--jump-deg` ⇒ 高频抖动；少数大跳 ⇒ 锯齿 |
| `analyze_slam_bag.py` | **离线**bag 体检：各流 平均率/中位/最大空洞/戳异常；IMU 启动零值；两条里程计对比；`/tf` 各边 | 判"在甩"看**速率抖动 P95**，不是逐样本 Δyaw |
| `seg_bench_offline.cc` | 把录到的点云**直接喂 linefit 核心库**计时（不经 ROS/DDS） | 用来证伪"算力不够" |

配套：`ros2 bag record -o .tmp_bags/x /clock /scan /livox/imu /odom /odom_ground_truth /tf /tf_static /livox/lidar/pointcloud /segmentation/obstacle`
（**录制要早于 launch**，否则抓不到启动瞬间；bag 放在工作区内便于离线复算。）

---

## 3. 本轮实测基线（留档，便于对比）

| 量 | 实测值 |
|---|---|
| 插件 `/livox/lidar/pointcloud` | **10.00 Hz（仿真时间）/ 0 个 >0.5s 空洞**；墙钟 7.96 Hz（RTF≈0.8） |
| linefit `/segmentation/obstacle`（修 QoS 前） | **0.31 Hz / 49 个 >0.5s 空洞 / 最长 20.8 s** |
| p2l `/scan` | 68 条 = linefit 的 68 条（**一帧不丢**） |
| `segment()` 单帧耗时 | **平均 1.01 ms、最慢 1.80 ms**（30000 点，sim 参数） |
| 同一发布端、不同订阅者 | RELIABLE(rosbag2) **2182/2182=100%**；BEST_EFFORT(linefit/hz) **3%~26%** |
| `map→odom`（修 ② 前） | 84 次 >2° 跳变**全为正向**、单步最大 **76.8°**、累计 **−66.6°**；同期机器人净转 +3.8° |
| `/scan` 构成 | 1462 bin；有回波 **62.7%**（37.3% 为 `inf`）；距离 P50 2.3 m、4~10 m 占 16% |
| 插件发的假点 | 每帧 **78.4%**（23500/30000）是 `(0,0,0)`（无回波被填零；真实驱动只发有效回波） |
| `base_link→base_link_fake` | 5508 条中 **3317 条戳非单调**（`fake_vel_transform` 待修） |

---

## 4. 症状 → 判据 → 处置（速查表）

| 症状 | 先量什么 | 判据 | 处置 |
|---|---|---|---|
| **地图跟着车转 / 地图飞** | `monitor_map_odom.py` | `map→odom` 出现**单向大跳**且跳变紧跟 scan ⇒ "扫描间朝向不转、每帧补刀" | 先修**扫描率**（§5.3），`map→odom` 会同时变好；IMU 是可选补强 |
| **残影 / 双层墙** | 收尾日志 `Score histogram` | `Count: 0` = 回环没命中、无全局锚定 | 回环门槛回到 `0.65/0.70`；搜索距离/窗口保持收紧 |
| **特征先有后没（内部小墙、遮挡后消失）** | `/scan` 的 `inf` 比例 + `missing_data_ray_length` | `inf` 比例高（我们 37%）且该参数 3 m ⇒ 无回波方向每帧清 3 m | **调小 `missing_data_ray_length`**（§5.2） |
| **静默丢帧（话题"看着有"但下游没有）** | 逐段率 + QoS | 大消息 + BEST_EFFORT 收 | 订阅端改 **RELIABLE**（§5.3） |
| **`cartographer_node` exit -6** | 崩溃栈**第一行** | `Check failed: ...` | 读 CHECK 语义；时间序类看时间基（#19/#21），IMU 类看数据（#24） |
| **帧树诡异 / 车乱抖** | `view_frames` 的父边计数 | 任一帧**两个父** ⇒ 查找结果跳变 | 删掉多余的静态桥/残留节点（`static_transform_publisher`） |
| **`topic hz` 看着正常但仍丢帧** | bag（存储时间）vs header 戳 | 两个时间轴不一致 | 用 bag 的**接收时间**看真实节奏（§6.1） |

---

## 5. 关键机制（本轮真正学到的三条）

### 5.1 `map→odom` 的定义就是"矫正量"——先分清三态

`T_map_odom = T_map_base(全局) × T_odom_base(里程计)⁻¹`。它**必然在动**；不动只意味着"完全信任里程计、永不纠偏"。
判据（用 `monitor_map_odom.py`）：

| 形态 | 含义 | 处置 |
|---|---|---|
| 平滑、缓慢、几度内 | **正常**（这就是它的职责） | 不动 |
| **锯齿**：慢慢涨到十几度再被拉回 | 局部 SLAM 在子图内走偏 + 全局事后矫正 → **留残影** | 查局部输入/扫描率 |
| **高频抖动**：逐样本就跳 | 局部匹配/时间戳/交付问题 | 查输入与 QoS |

⚠️ 别把 `map→odom` 有常数偏置当 bug：map 帧原点=建图起点、朝向每次重启都不同（cartographer_ros #1170，wontfix）。

### 5.2 特征"先有后没" = 无回波方向被清成自由空间（**主要吃掉的是内部小墙**）

**先记住一件事**：cartographer 的 2D 栅格不是"画上去就永久"，而是**每一帧对每个格子投票**：

| 票 | 什么时候投 | 效果 |
|---|---|---|
| **命中 +** | 这一束真的打在这个格子上 | 概率上升（`hit_probability = 0.55` → odds ×1.22） |
| **穿过 −** | 射线从原点走到命中点，途经的格子 | 概率下降（正常、且正确） |
| **无回波 −（关键）** | 这个角度**没有障碍物**（`inf`） | cartographer 把 **0 ~ `missing_data_ray_length`** 这一整段标成**自由**（我们设 **3.0 m**） |

而这条链路里 **`inf` 的 bin 特别多：实测 37.3%**（1462 个 bin 里 546 个）。来源是两个"物理上正常"的原因：

- `linefit` 把**地面**去掉了 → 朝地面的那些角度**没有障碍物** → `inf`；
- MID360 垂直 FOV 是 −7°~+52°，**上视**的射线打不到东西 → `inf`。

于是**每帧都有 37% 的方向把 3 m 以内"投票成空地"**。这解释了为什么"**主要是内部小墙**"消失：

```
        外侧大墙（4~7 m，在"3 m 常清区"之外）
   ┌────────────────────────────────────────┐
   │                                        │
   │         ▮ 内部小墙（1.2 m）
   │         ▲
   │         │ ← 这一束打中小墙 → 小墙拿到 1 张"命中"票（稀疏！）
   │      (车)
   │         │
   │         └──► 车一转 / 小墙被挡 → 同一束变成 inf
   │                → cartographer 把 0~3 m 全标 FREE
   │                → 小墙每帧吃一张"清除"票
   └────────────────────────────────────────┘
```

**为什么内墙死、外墙活 —— 数字正好对上**（我们实测有回波点的距离分布）：

| 距离 | 占比 | 是否在 3 m"常清区"内 |
|---|---|---|
| 1~2 m | 32% | ✅ 在内 → **每帧被清** |
| 2~4 m | 47% | ✅ 大部分在内 |
| **4~10 m** | **16%** | ❌ 在外 → 只被"真正穿过的射线"清，命中票还多 → **留得住** |

P25=1.8 m / P50=2.3 m / P75=3.1 m ⇒ **约 3/4 的命中都落在 3 m 的常清区里**，而外侧大墙只有 16% 的命中、且大多在常清区外。

再叠加两件事，小墙就更站不住：
1. **命中票稀疏**：`/scan` 每帧只有约 3100 个有效障碍点摊在 1462 个 bin 上（≈2 点/bin），而**机器人一转，同一面小墙每帧落进不同的 bin** ⇒ 同一个格子被命中的频率很低；
2. **清除票每帧必到**：只要那一束没打中小墙，`inf` 就给它一张"清 3 m"的票。

⇒ 概率在阈值附近来回摆 ⇒ **"扫到就有、扫不到就没"的闪烁**。（这不是"忘了"，是**被主动擦掉**。）

**处置（`cartographer.lua`）**：

| 参数 | 现值 | 建议 | 作用 |
|---|---|---|---|
| `TRAJECTORY_BUILDER_2D.missing_data_ray_length` | **3.0** | **0.5~1.0** | **首选**：常清区从 3 m 缩到贴身范围，1.2 m 的小墙不再被无回波束擦掉 |
| `...probability_grid_range_data_inserter.hit_probability` | 0.55 | **0.62** | 命中更"粘" |
| `...miss_probability` | 0.49 | **0.45** | 清除更弱（两者拉开差距，墙才立得住） |
| `TRAJECTORY_BUILDER_2D.num_accumulated_range_data` | 1 | **3** | 3 帧累积再插入 ⇒ 同一格有效命中 ×3 |

判别实验：临时 `insert_free_space = false` 跑一圈 —— 若小墙立刻稳稳留住，即确认"清除太狠"；确认后按上表**调弱**，不建议永久关掉（会失去清动态物的能力）。

> **2026-09-22 已改第一组（待重跑验证）**：`missing_data_ray_length 3.0 → 1.0`、`hit_probability 0.55 → 0.62`、`miss_probability 0.49 → 0.45`；`num_accumulated_range_data` 仍为 1（留作下一组变量）。一次只动一组，便于归因。

### 5.3 大消息 + BEST_EFFORT = 静默丢帧（本轮最隐蔽的一环）

- 480 KB 的 `PointCloud2` 必须**分片**发送；**BEST_EFFORT 不重传** ⇒ 丢任意一个分片，**整帧作废**。
- ROS 2 的 `rclcpp::SensorDataQoS()` 预设 = `BEST_EFFORT + KeepLast(5)`，会让这问题雪上加霜。
- **判决实验（最省事）**：同一个发布端、同一台机器上比两种订阅者 —— RELIABLE 拿到 100%、BEST_EFFORT 拿到 3%。
- 处置：订阅端改 `rclcpp::QoS(rclcpp::KeepLast(10)).reliable()`，与发布端对齐（FAST-LIO 一直就是这么订的）。
- 同类风险：任何"大消息 + 传感器预设 QoS"的链路（点云/图像/大 CustomMsg）都要核两端 Reliability。

---

## 6. 踩过的坑（每条一句话，都是真的疼过）

1. **只看中位间隔会掩盖长空洞**：`/scan` 中位 300ms（像 3.3Hz），实际平均 1.67Hz、98 个 >0.5s 空洞、最长 6.6s → 必须同时报 **平均率 + 最大空洞 + 空洞计数**。
2. **`ros2 topic hz` 会被自己的反序列化拖死**：30000 点的 `PointCloud2`/CustomMsg 用 Python 工具测会低估 → 更可信的是 **bag**（rosbag2 订阅）与**双时间轴**对照。
3. **两个时间轴要分开看**：header 戳（仿真钟，反映"数据时刻"）vs bag 接收时间（墙钟，反映"真实节奏"）。我们遇到过"仿真时间 10Hz 完美、墙钟成串带洞"。
4. **症状在感知层 ≠ 根因在感知层**：插件时间基 bug（#19）的症状是 cartographer 崩 + 地图跟转，害我们在感知/cartographer 参数上兜了几圈。
5. **`exit -6` 要读栈的第一行**：`F <file>:<line> Check failed: ...`；没有源码时可 `strings libcartographer.a | grep "Check failed"` 把候选 CHECK 全列出来，再按语义定位。
6. **"关掉先验"和"用同源先验"都是坑**：先验要**独立且不晚到**（实车=轮速 odom，仿真=`/odom_ground_truth`）；LIO 自己的 odom 与 scan 同源且晚一整帧。
7. **打开 IMU 有前置条件**：`use_imu_data=true` 前要确认 ①首帧 IMU 不是 `(0,0,0)`（`imu_tracker.cc:67` 拿**第一帧**当重力基准）②三路时间戳同轴。FAST-LIO 没事只因为它**多帧求均值**。
8. **任何"为了跑得动而改感知输出"都要拒绝**：那是拿 sim2real 保真度掩盖仿真伪影（见 `sim_real_contract.md` §四）。
9. **多父边会让 TF 查找在两条路径间跳**：症状与"SLAM 在矫正"极像；`view_frames` 看每帧父边数，`node list` 找残留进程。
10. **`pkill -f gzserver` 会匹配到自己的 shell**（`-f` 匹配整条命令行）→ 用 `pkill -x`（精确进程名）或按 PID 杀。
11. **测量要早于/同时于数据源**：`ros2 topic hz` 必须在话题已有发布者时启动；录制要早于 launch 才能抓启动瞬间（我们连续两次错过）。
12. **工具自身的判读口径要先用合成数据验**：`monitor_map_odom.py` 与 `analyze_slam_bag.py` 都先用合成 bag/曲线验过判读（锯齿/抖动/平滑三态），否则会拿错结论去改配置。

---

## 7. 复现与验收命令合集

```bash
# 录制（放在工作区内，便于离线复算；先开录制再 launch 才能抓启动瞬间）
mkdir -p .tmp_bags && ros2 bag record -o .tmp_bags/x /clock /scan /livox/imu /odom \
  /odom_ground_truth /tf /tf_static /livox/lidar/pointcloud /segmentation/obstacle

# 离线体检（各流率/空洞/戳、IMU 启动、两条里程计、map→odom）
python3 tools/analyze_slam_bag.py --bag .tmp_bags/x

# 在线看 map→odom 形态
python3 tools/monitor_map_odom.py --csv /tmp/mo.csv

# QoS 两端
ros2 topic info /livox/lidar/pointcloud --verbose | grep -E "Node name|Reliability|History"

# 感知链逐段（仿真在跑时）
ros2 topic hz /livox/lidar/pointcloud ; ros2 topic hz /segmentation/obstacle ; ros2 topic hz /scan

# 帧树
ros2 run tf2_tools view_frames
```

**验收判据（本轮）**：`/segmentation/obstacle` ≈ 7.5 Hz（墙钟）/ 10 Hz（仿真）；`/scan` 同步；
`monitor_map_odom.py` 无锯齿（单帧转角 = 角速度 × 0.1 s < 20° 搜索窗）；转一圈后墙是连续深色线、不再"先有后没"。

---

## 8. 未决项

| 项 | 说明 |
|---|---|
| 插件不发 `(0,0,0)` 假点 | 无回波射线现在被填成 `(0,0,0)` 发出（每帧 78.4%）。真实驱动只发有效回波；修掉后消息 480 KB→~100 KB。属**仿真保真度**修复，需拍板 + rebuild |
| IMU 首帧重力 | 要"先录制 → 全新 launch"才能抓到首帧；确认后加"丢掉开头零加速度帧"的管道滤波（实车同样需要） |
| `fake_vel_transform` 戳 | `base_link→base_link_fake` 5508 条里 3317 条戳非单调（会让 tf2 报 TF_OLD_DATA） |
| `/odom`(LIO) 重复戳 | 1905 条里 20 条重复（同样会让显示侧抖） |
