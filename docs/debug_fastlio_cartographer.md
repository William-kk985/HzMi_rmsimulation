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

> **2026-09-22 **第一组已改、但当场被实测否掉 → 已回退****：`missing_data_ray_length 3.0 → 1.0`、`hit_probability 0.55 → 0.62`、`miss_probability 0.49 → 0.45`；`num_accumulated_range_data` 仍为 1（留作下一组变量）。一次只动一组，便于归因。

#### 5.2.1 实测：低矮特征到底死在哪儿（8 帧 bag，**已排除插件发的假点**，地面按实测 `z=-0.226m` 标定）

| 离地高度带 | 真实点/帧 | linefit 障碍/帧 | 进入 `/scan` | raw→障碍 | 障碍→scan |
|---|---|---|---|---|---|
| 地面 0~3cm | 453 | 1 | 0 | **0.1%** | — |
| **低 3~8cm** | 347 | 10 | 4 | **2.8%** | 36.8% |
| 低 8~15cm | 1061 | 627 | 173 | 59.1% | 27.6% |
| 中 15~30cm | 3246 | 1417 | 584 | 43.7% | 41.2% |
| 中 30~60cm | 927 | 860 | 103 | 92.8% | 11.9% |

**结论（三层，缺一不可）**：

1. **linefit 把 ≤8cm 的东西几乎删光**（存活 0.1% / 2.8%）——两个参数共同造成：
   - `max_dist_to_line = 0.1`：**离"地面线"10cm 以内都算地面**；
   - `sensor_height = 0.275`，而**实测雷达离地只有 0.226 m（差 4.9cm）** ⇒ 地面线被整体抬高约 5cm，再叠 10cm 容差 ⇒ **15cm 以内的低矮特征大量被当"地面附近"删掉**（表里 8~15cm 只活 59%、15~30cm 只活 44% 也与场地的坡道/高地有关）。
2. **p2l 再砍 60~90%**：每角度 bin **只留最近的那个点** ⇒ 近处残余（地面漏网点、车体自身）会**遮住**后面的墙 ⇒ 这是"越近越留不下"的第二层原因。
3. **第一层是几何，参数改不了**：MID360 垂直 FOV 下视只有 **−7°**，雷达离地 0.226m ⇒ 一堵高 h 的墙只有距离 **d ≥ (0.226−h)/tan7°** 才可能被看到：
   `h=0.20m → d≥0.21m`；`h=0.15m → d≥0.62m`；`h=0.10m → d≥1.03m`；`h=0.05m → d≥1.44m`
   ⇒ **离得越近，低墙越在雷达下视视野之外**（真机同样，所以这条也解释"为什么是内部低墙"）。

**⇒ 因此"七次修正"（调弱清除）已回退**：当数据里根本没有低墙时，减少清除只会让残影/噪声留得更久（观感更差）。
正确的顺序是：**先把低墙送进 `/scan`**，再谈栅格累积/清除的调参。

**修法候选（都是感知参数，需拍板）**：
- `segmentation_sim.yaml`：`sensor_height 0.275 → 0.226`（按实测标定，纯修正）
- `segmentation_sim.yaml`：`max_dist_to_line 0.1 → 0.05`（≤8cm 的小台阶不再算地面；场地有坡道/高地，别调更小）
- 可选（p2l）：`min_height -1.0 → -0.15` 可减少"近处遮住远处"，但**会连真低墙一起挡掉**，必须与上面两条一起评估
- **验收判据（可量化）**：改完重录一段 bag，再跑同样的分带统计 —— 期望 **"低 3~8cm" 的 raw→障碍保留率从 2.8% 显著上升**，且 `/scan` 的低带返回数增加。

> **2026-09-22 已改（感知侧两个参数）**：`segmentation_sim.yaml` 的
> `sensor_height 0.275 → 0.226`（实测标定）、`max_dist_to_line 0.1 → 0.05`。
> 注意：这两个是 **linefit 的地面分割参数**，与 FAST-LIO 无关（不同节点、不同输入话题、无共享状态），
> 也不改"什么算障碍"的语义，只是把"地面线"标定对、把 10cm 容差收到 5cm。
> ⚠️ `sensor_height` 是**每个平台各自标定**的量（换车/换安装/上实车都要重量）。
> ⚠️ 几何那条（MID360 下视 FOV −7°）**任何参数都修不了**：低矮特征"离得够远才看得见"，
> 真机同样 —— 所以建图时**别贴着矮墙走**。

---

#### 5.2.2 用 `/map` 直接把"留不住"量化（`analyze_slam_bag.py` 第 ⑤ 段）

录制时**把 `/map` 也录上**（`cartographer_occupancy_grid_node` 以 ~1Hz 发，带 `transient_local`），
然后 `python3 tools/analyze_slam_bag.py --bag <bag>` 会给出：

| 输出 | 含义 |
|---|---|
| 曾占据格子数 / 结束时仍占据（%）/ 被擦掉 | "留不住"的直接比例 |
| **留存曲线 +2/+5/+10/+20 帧** | 首次占据后还能保持多久 |
| **闪烁统计**（同一格 occupied→free 次数） | 高 = 被**主动擦除**（清除与命中在拉锯） |
| 最终栅格取值分布（自由/中间/占据） | 占据数远小于中间数 ⇒ 概率卡在阈值附近 |

**判读（决定往哪边修）**：
- 闪烁多 + 留存曲线掉得快 ⇒ **被主动擦除** ⇒ 调 cartographer 的清除/命中（先做 `insert_free_space = false` 判别实验）；
- 曾占据很少 + 闪烁少 ⇒ **数据里就没进来** ⇒ 修感知/几何（linefit 容差、`sensor_height`、MID360 下视 FOV）。

**实测（2026-09-22，`.tmp_bags/ret`，222 帧 `/map`，感知参数已修 + QoS 已修）**：

| 指标 | 值 | 判读 |
|---|---|---|
| `/scan` | **10.00 Hz / 最大空洞 200 ms** | QoS 修复彻底生效（原来 0.31 Hz / 20.8 s） |
| `map→odom` | 峰峰值 **7.04°**、单步最大 2.54° | 定位已稳（原来 83° / 76.8°） |
| 曾占据 → 结束仍占据 | 3779 → 1837（**49%**） | **51% 被擦掉** |
| 留存曲线 +2/5/10/20 帧 | 87.5 / 74.4 / 63.9 / **63.8%** | 10 帧内掉到 64% 后走平 |
| 闪烁（occupied→free） | 中位 2 / P95 6 / max 12，**87% 的格子闪过** | **被主动擦除**（不是数据没进来） |

⇒ **下一步判别实验（已设为临时值）**：`insert_free_space = false`，复测第 ⑤ 段。
预期：被擦掉 51% → ~0、闪烁消失、留存曲线走平 ≈100%。
**判别实验结果（`.tmp_bags/ret2`，183 帧 `/map`，`insert_free_space = false`）**：

| 指标 | true（旧） | **false（诊断）** |
|---|---|---|
| 结束仍占据 | 49% | **89%** |
| 被擦掉 | 51% | **11%** |
| 留存 +20 帧 | 63.8% | **78.9%** |
| 闪烁≥1 的格子 | 87% | 54% |
| **自由格子(0~30)** | 35856 | **0** ← 关掉清除 ⇒ 自由空间也没了 ⇒ 栅格全灰（"一点点出来很艰难"） |

⇒ **"留不住"确实是清除造成的**；但**不能靠关掉清除解决**（自由空间会一起消失，地图没法用；
本题场景虽无动态物，实车有，清除能力要保留）。**已改为"保留清除 + 调弱"**（八次修正）：
`insert_free_space = true`、`missing_data_ray_length 3.0→1.0`、`hit/miss 0.62/0.45`，
并把 `num_range_data` / `optimize_every_n_nodes` 由 30 调回上游 **90**（扫描率提高 10 倍后，
30 = 每 3 秒换一个子图/优化一次，子图重叠缝暴增）。

**"谁在清它"再定量（ret2 的 7 帧，逐帧重建 p2l 的每-bin 选点与其高度）**：

```
低矮特征(离地 3~15cm)的 2D 格子：每帧 90.4% 会被某束光**命中**
                                36.3% 会被"命中高度>25cm 的光束"**从上方穿过**（= 一张清除票）
⇒ 命中票 : 清除票 ≈ 1 : 0.4
```
⇒ **主导的清除不是"从上方穿过"，而是 37% 的无回波光束 × `missing_data_ray_length`**。
关键认识：对**真实 2D 雷达**"无回波 = 这段是空的"成立；但我们是 **3D FOV(−7°~+52°) 转 2D**，
无回波大多来自"朝天上打空" ⇒ 该假设**不成立** ⇒ 这个半径必须小。
另外：`num_accumulated_range_data` **不解决**这个问题 —— 累积多帧把命中与清除**同时放大，比值不变**。

**九次修正（现场观感"墙变淡/变灰、慢慢化掉" ⇒ 清除仍压过命中）**：
`missing_data_ray_length 1.0→0.5`、`hit 0.62→0.68`、`miss 0.45→0.40`
⇒ odds 比 hit/miss：1.27（旧）→ 2.0（八次）→ **3.2**；清除半径 3.0→1.0→**0.5 m**（被清面积约 1/6）。
**并顺手把 `submaps.num_range_data` 与 `optimize_every_n_nodes` 从 30 调回上游默认 90**：
扫描率从 0.3 Hz 提到 10 Hz 后，节点插入率涨了约 10 倍，30 意味着"3 秒一个子图 / 3 秒优化一次"，
子图间重叠缝会明显增多（这也是"留不住"的一个来源）。

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
