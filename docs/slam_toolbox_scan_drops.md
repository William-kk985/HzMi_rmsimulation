# `slam_toolbox` 的 `/scan` 丢帧：根因考证、时间戳链实测与 `scan_queue_size` A/B

> 面向的问题（现场观察）：`world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox`
> 建图时，`async_slam_toolbox` 持续刷
> `Message Filter dropping message: frame 'livox_frame' at time <t> for reason 'discarding message because the queue is full'`，
> **每 2.5 仿真秒一条**（min=p50=p90=2.5 ⇒ 系统性），约 **4.0~4.3% 的 `/scan`**，每次跑都有。
>
> 前置工作：`docs/mapping_2d_from_cloud.md` §2 已用 A/B 证明
> `target_frame: base_link`（p2l）、`transform_timeout 0.2→0.5`、`minimum_time_interval 0.5→0.2`
> **都不能降低丢帧率**（4.03/4.03/4.04/4.04%），并在 §9 把「`scan_queue_size` 的 A/B」列为
> **最值得接着做的实验**。本文就是那一步：先考据上游语义与其他项目做法（§2–§5），
> 再实测我们的时间戳链（§6）、量化丢帧的影响（§7）、做 `scan_queue_size` 的 A/B（§8），
> 最后给出**改还是不改**的判定（§9）。

> **本文的证据分级**：`[V]` = 本机源码/实测可复现；`[R]` = 上游仓库/issue/文档（给了 URL）；
> `[I]` = 推断（明确写出推断链与它的证伪条件）。**"事实"与"推断"分开写**，推断一律带 `[I]`。

---

> ★ **2026-10-06 后续（`docs/lio_drift_diagnosis.md`）**：本文 §7 的"处理门
> （`minimum_time_interval 0.5 s` + `minimum_travel_distance 0.447 m`）⇒ 只有 ~3% 的 `/scan`
> 真的被拿去匹配"这一条，是理解 **`map→base_map` 为什么是"分段常数"**、
> 以及**为什么 slam_toolbox 救不回一次大跳**的关键。分段诊断里实测：
> 正常工况下 `map→base_link` 的误差是 LIO 的 **1.66~1.94 倍**且不发散；
> 但 LIO 一旦崩（高角速度甩转），融合位姿与 LIO **1:1 同步**（比值 0.869），slam 没有拉回来。

## 0. 一句话结论

| # | 问题 | 结论 |
|---|---|---|
| A | `scan_queue_size` 是什么 | `slam_toolbox` 交给 `tf2_ros::MessageFilter` 的**"等 TF 的消息队列深度"**，默认 **1**，由上游 PR #526（2022-08-24 合入）从硬编码 `1` 改成参数。上游 README 明写 **"Should always be set to 1 in async mode"** |
| B | 为什么会被丢 | 队列深度 1 时，**只要"某条扫描还在等它的 TF"期间又来了下一条，先到的那条就被顶掉**（tf2 MessageFilter 的 `QueueFull`），日志里那条 `at time` 是**被顶掉的那条**的戳 |
| C | 上游/其他项目怎么处理 | 维护者口径：这是 **TF/时间戳问题**不是 slam_toolbox 的 bug（#720/#777/#806/#834 全部以"用户侧 TF 配置问题"关闭）；[I] 真正的上游修法在 `ros2/geometry2#544`（**2022-07 提出，至今 open**）。可比项目（COD 2026 / TurtleBot 4 / uOttawa rover）**都没配 `scan_queue_size`**（=默认 1） |
| D | 我们的真因 | **LIO 发的 `odom→base_link` 戳比同一帧 `/scan` 的戳低 1 纳秒**：上游 `small_point_lio_node.cpp:57` 用 `static_cast<uint32_t>((ts-floor(ts))*1e9)` **截断**，浮点误差让 ~40% 的帧变成 `X.99999999x → X−1 ns`；tf2 在扫描戳上要求"缓存里有 stamp ≥ 该戳的样本"，低 1 ns 的同帧样本不合格 ⇒ 只能等**下一帧** TF（~0.1 s 仿真秒）⇒ 队列深度 1 时被 `QueueFull` 顶掉。实测：**Δns=0 的扫描一条都没丢（1658 条 / 0 丢），116 条丢帧的 Δns 全部 > 0**（§6.3） |
| E | 丢帧有害吗 | 见 §7：`minimum_time_interval 0.5` + `minimum_travel_distance 0.5`（实际门限 0.447 m）把**绝大多数** `/scan` 本来就判掉（本路线实测每 **≈2.5~7 仿真秒**才吃一帧 ⇒ 每用 1 帧丢 ~24~30 帧）。被丢的那 4% 里 **58.6% 被两个门"证明"本来就会忽略**；剩下 41.4% 的代价上界 = 被选中的帧晚 **0.1 仿真秒 / 2 cm** |
| F | 改了什么 | **改 LIO 的戳构造（截断 → 四舍五入 + 进位）**，让 TF 戳与点云戳逐纳秒对齐 —— 这是**根因修复**，不是队列补丁；`slam_toolbox` 的参数**一个都没改**（保持默认 `scan_queue_size=1`）。**验证跑 fix01：丢帧 116 → 10（4.3% → 0.35%），2.5 s 节律消失，等待 TF 的 p90 从 136 ms 降到 13 ms**（§9.4）。**没有加任何监控/哨兵节点** |

---

## 1. 复现环境与口径

* 栈：`world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox nav_rviz:=False`，
  驱动用 `tools/scripts/mapping/coverage_drive.py --route .tmp_cache/hzmap2d/route_ramp.json --speed 0.20`（20 航点 / 19.6 m，
  含 23° 坡道与 1.05 m 宽走廊，与 `docs/mapping_2d_from_cloud.md` §2 同一条路线、同一速度）。
* 一次跑图 = **一次 bash 调用**（每个 bash 调用一个 PID namespace，调用结束进程全清）：
  `HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、`unset DISPLAY`、独立 `GAZEBO_MASTER_URI` 端口。
* 所有"仿真时间"都用 `/clock`（`use_sim_time: true`）；**rclpy 订阅 `/clock` 必须用 BEST_EFFORT**
  （`rclpy/time_source.py:70`）——用 RELIABLE 会一帧都收不到。
* 丢帧分母同一把尺子：`ramp_section_probe.py` 旁路收到的 `/scan` 条数（不经过 slam_toolbox 的过滤器）。
* 版本：本仓 `src/rm_localization/slam_toolbox` = **2.6.10**（`package.xml`），与系统包
  `ros-humble-slam-toolbox 2.6.10-1jammy` 同版本；系统 `tf2_ros 0.25.23`。
  ⚠️ `install/share/slam_toolbox/config/*.yaml` 是指向 `src/` 的**符号链接**（`--symlink-install`），
  所以 A/B 里临时改 `src/` 的 yaml 会**真的**作用到跑起来的节点（这一点专门验证过，`diff` 为空 + 链接可查）。

---

## 2. 事实：`scan_queue_size` 到底是什么 `[V]/[R]`

### 2.1 声明与默认值（本机源码）

`src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp`：

```cpp
:155  scan_queue_size_ = 1.0;                       // ← 默认 1（注意成员是 int，见下）
:156  scan_queue_size_ = this->declare_parameter("scan_queue_size", scan_queue_size_);
...
:240  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
:241      shared_from_this().get(), scan_topic_, rmw_qos_profile_sensor_data);
:243  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
:244      *scan_filter_sub_, *tf_, odom_frame_, scan_queue_size_, shared_from_this(),
:245      tf2::durationFromSec(transform_timeout_.seconds()));
```

* **类型是整数**：成员声明在 `include/slam_toolbox/slam_toolbox_common.hpp:148`
  （`int throttle_scans_, scan_queue_size_;`），`declare_parameter` 由默认值推断为 `PARAMETER_INTEGER`
  ⇒ yaml 里要写 `scan_queue_size: 5`，写 `5.0` 会类型不匹配。
* **语义 = "等 TF 的待办消息条数上限"**，不是"处理积压队列"：见 §3。
* 本仓的 `config/mapper_params_online_async_sim.yaml` **没有这个键** ⇒ 跑的就是默认 1 `[V]`。

### 2.2 上游文档怎么说 `[R]`

仓库内 README（`src/rm_localization/slam_toolbox/README.md:227`，与上游 `ros2` 分支一致）：

> `scan_queue_size` - The number of scan messages to queue up before throwing away old ones.
> **Should always be set to 1 in async mode**
> —— https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md （Parameters 一节）

### 2.3 它是**什么时候**出现的、为什么加的 `[R]`

* 参数由 PR **#526 "Midigate spamming for 50Hz lidar"** 引入，合入时间 **2022-08-24**，
  commit `cede9e8248abf791fcdc45b80ebf9bdabc20f637`；commit message：
  `* Increase scan filter queue_size / * Add scan_queue_size as parameter`。
  —— https://github.com/SteveMacenski/slam_toolbox/pull/526 ，
  https://github.com/SteveMacenski/slam_toolbox/commit/cede9e8248abf791fcdc45b80ebf9bdabc20f637
* **改动前是硬编码 1**（PR diff 原文）：
  ```diff
  -    *scan_filter_sub_, *tf_, odom_frame_, 1, shared_from_this(),
  +    *scan_filter_sub_, *tf_, odom_frame_, scan_queue_size_, shared_from_this(),
  ```
  ⇒ 我们看到的"队列深度 1"**不是本仓的配置遗漏造成的偏差，而是上游一直以来的默认行为**；
  这个参数只是给了"想改的人一个口子"。
* PR 的动机是**日志刷屏**，而且报的是 **sync** 节点（`sync_slam_toolbox_node`）：
  50 Hz 的 LMS111 + 默认 `minimum_time_interval`，过滤器消息量大。
  ⇒ 它**不是**为"提高建图质量"加的，README 也明确要求 async 模式保持 1。

---

## 3. 事实：`QueueFull` 的触发条件（tf2 源码级）`[V]`

系统头文件 `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp`（tf2_ros 0.25.23）：

```cpp
:410      if (queue_size_ != 0 && messages_.size() + 1 > queue_size_) {
:411        ++dropped_message_count_;
:412        const MessageInfo & front = messages_.front();
...
:419        messageDropped(front.event, filter_failure_reasons::QueueFull);
:420        messages_.pop_front();
:421      }
...
:113        case filter_failure_reasons::QueueFull:
:114          return "discarding message because the queue is full";
```

四条**关键语义**（都影响我们怎么读日志）：

1. `messages_` 里放的**只是"TF 还没到、还在等"的消息**。TF 已经能查到 ⇒ `add()` 里
   `buffer_.waitForTransform(...)` 会**同步**回调 `transformReadyCallback` ⇒ 立刻派发给
   `laserCallback`，**根本不进队列**（同一文件 :438-447 的注释：句柄为 0 表示回调已被同步调用）。
2. 队列满时被丢的是 **`messages_.front()` = 最老的那条**，不是新来的那条
   ⇒ **日志里 `at time X` 的 X 是"被顶掉的那条扫描"的时间戳**（所以能直接拿来算丢帧率）。
3. 触发条件是"**又来了**一条消息"（`add()`），且此刻老的那条**仍在等 TF**
   ⇒ 队列深度 1 时，等价于：**"这条扫描的 TF 等待时间 > 下一条扫描的到达间隔"**。
4. 这个等待**不阻塞 executor**：tf2 用的是 `waitForTransform` 的 **future + 定时器**，
   `transform_timeout`（我们配 0.2 s）只是这条 future 的超时（超时后按 `OutTheBack` 处理），
   **不会**让线程睡在那儿等 ⇒ 加 `transform_timeout` 并不能"多等一会儿让队列不溢出"
   （与前置 A/B 的实测一致：0.2→0.5 丢帧率不变）。

---

## 4. 事实：上游 issue / PR 里这件事怎么被处理的 `[R]`

| 编号 | 内容 | 处置/维护者口径 | 链接 |
|---|---|---|---|
| **#516**（2022-07）| **与本案同构**：20 Hz 扫描 + 50 ms 的 TF 延迟 ⇒ **全部**扫描被丢。作者做了两个 bag：`valid`（正常）与 `shifted_50ms`（把 TF 整体推迟 50 ms）；后者在 queue_size=1 时**全丢**，queue_size=3 才恢复 | Steve：*"Why exactly is your TF delayed so much? … Message filters are everywhere in ROS so if you have a problem here, you're going to run into that all over the place."*；对"队列 1 被抢占"的机制他一开始**不信**（"I do not believe message filters is going to preempt trying to find a transform…"），后来认可 patch（*"Yeah, that patch looks reasonable!"*）并说 *"It wouldn't be a big deal either way actually to keep a queue size of 2 if it functionally resolves a problem."*；而提 issue 的人也同意 **"The queue size should remain 1."** | https://github.com/SteveMacenski/slam_toolbox/issues/516 |
| **geometry2 #544** | 上游真正的修法：*"Prevent tf message filter from preempting the oldest message while it is waiting for transforms."*（queue 满时**不**抢占正在等 TF 的最老消息）。PR 描述里写得很直白：*"…due to messages waiting for transforms, the oldest message is removed, preempting it. This is especially apparent when the queue size is 1… Increasing the queue size can mitigate this, but there are cases where we want to be able to set the queue size very low or even to 1."* | **状态：open，2022-07-21 创建，至今未合**（2026-10-06 核查）⇒ 这个"抢占"行为在 Humble/`tf2_ros 0.25.23` 上**依然是现状** | https://github.com/ros2/geometry2/pull/544 |
| #526 | 见 §2.3（加参数 + 关日志刷屏） | 已合入 | https://github.com/SteveMacenski/slam_toolbox/pull/526 |
| #720（2024-07）| 日志与本案一字不差；提问者说 *"cannot get rid of this error, tried changing queue size and launch file many times"* | 回帖结论：**TF 树问题**（"check your TF tree…"）；维护者以 *"Closing - as this is a user configuration issue"* 关闭 | https://github.com/SteveMacenski/slam_toolbox/issues/720 |
| #777（2025-05）| SICK TiM + `base_link→cloud`，队列满 | 自答：**TF 树坏了**（`sick_scan_xd` 自己发了一棵带 `map` 父边的树） | https://github.com/SteveMacenski/slam_toolbox/issues/777 |
| #806（2025-10）| 与本案几乎同样的话题组合（3D 雷达转 `/scan`） | 维护者：*"Did you store your TF information to be able to transform from the base_link frame to the odom frame? … Yes, you must have odometry to work with slam toolbox"*；提问者补上 `odom` 后该错误消失（换成"Failed to compute odom pose"） | https://github.com/SteveMacenski/slam_toolbox/issues/806 |
| #834（2026-01）| 定位模式导航时队列溢出 + 时间戳延迟 | 维护者：*"That usually points to a TF or timing issue in your system"*；提问者自查：**轮式里程计的时间戳与其它传感器不在同一时间基**上，改好之后明显改善 | https://github.com/SteveMacenski/slam_toolbox/issues/834 |
| #594（2023-04）| 加载大图时队列满 | 提问者问"能不能加大队列"，**没有维护者答复**；无人给出 `scan_queue_size` 方案 | https://github.com/SteveMacenski/slam_toolbox/issues/594 |

**归纳（`[R]` 层面）**：

* 上游**不认为这是 slam_toolbox 的 bug**，一律指向"TF/时间戳/里程计"这三件事。
* "加队列"在上游语境里是**缓解**手段（Steve 说 queue=2 可以接受；#526 只是让大家能调它），
  而**"队列应该保持 1（async）"是写入 README 的设计约束**。
* 机制层面的**根治补丁在 `geometry2`（#544），四年未合** ⇒ 我们不可能靠"升级依赖"解决。

---

## 5. 事实：可比项目怎么配的 `[V]/[R]`

| 项目 | `/scan` 来源 | `scan_queue_size` | `transform_timeout` | `minimum_time_interval` | 其它 |
|---|---|---|---|---|---|
| **RoboMaster COD 2026**（本仓 `third_party/cod_nav_2026/src/cod_bringup/params/mapper_params_online_async.yaml`；`[V]` 本地 clone 逐行核对） | `pointcloud_to_laserscan`，**`target_frame: base_link`**、`transform_tolerance: 0.5`、`min/max_height 0.1/1.0`、`range 0.5~20`、`angle_increment 0.0087`、`scan_time 0.3333` | **没有配**（=默认 1） | 0.2 | 0.3 | `mode: lifelong`；`transform_publish_period: 0.0`（节点不发 `map→odom`，外面静态桥，见 `docs/cod_nav_macro_integration.md`）；`minimum_travel_distance 1.0`、`minimum_travel_heading 0.1`、`map_update_interval 1.0` |
| **TurtleBot 4**（`turtlebot4_navigation/config/slam.yaml`）| 雷达 | **没有配**（=默认 1） | 0.2 | 0.25 | `mode: mapping` |
| **uOttawa Mars Rover Team `rover_workspace`**（GitLab，公开）| 雷达 | **没有配**（=默认 1） | 0.2 | 0.5 | `throttle_scans: 1`、`tf_buffer_duration: 30`、`mode: mapping` |
| **本仓现状** | `linefit → pointcloud_to_laserscan`，`target_frame: ""`、`range_min 0.05`、`max_height 0.1` | **没有配**（=默认 1） | 0.2 | 0.5 | `tf_buffer_duration: 30`、`minimum_travel_distance 0.5`、`minimum_travel_heading 0.5`、`map_update_interval 5.0` |

* 三个公开项目**都没有**动 `scan_queue_size`：说明"配 1"是这个生态里的**主流默认**，
  也说明**不能拿"别人都调大了"当依据**（不存在这样的别人）。
* COD 的 `/scan` 是 **`target_frame: base_link`**（p2l 内部自己等 TF，`queue_size` 默认
  = `std::thread::hardware_concurrency()`，即**很深**）⇒ `[I]` 他们的扫描是"**等到 TF 齐了才发**"，
  所以 slam_toolbox 侧几乎不会进队列。这正好是我们 `target_frame:""` 的**相反**取舍：
  我们换来了 p2l 更短的延迟与更少的 tf2 依赖，代价是把"等 TF"这件事推给了 slam_toolbox 的**深度 1** 队列。
* 参考值汇总（上游默认，`mapper_params_online_async.yaml`）：`transform_timeout: 0.2`、
  `tf_buffer_duration: 30.`、`minimum_time_interval: 0.5`、`throttle_scans: 1`、**无 `scan_queue_size`**。
  见 https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
* ⚠️ 想用 GitHub 代码搜索做"全网 `scan_queue_size`"普查**做不到**：
  `GET /search/code` 未鉴权返回 **401 Requires authentication**（2026-10-06 实测）`[V]`。
  本文的"可比项目"是**逐个仓库抓文件核对**得到的，不是全网普查（列入 §11 未验证）。

---

## 6. 实测：我们的时间戳链（为什么过滤器必须等 TF）

> 仪器：`tools/scripts/diag/scan_tf_timing_probe.py`（**一次性测量工具，不被任何 launch 启动**）。
> 它做的事：① 记录每条 `/scan` 的到达时刻（墙钟 + 仿真钟）与 `header.stamp`；
> ② 在**到达那一刻**用 tf2 复刻 MessageFilter 的判据 `can_transform(odom, livox_frame, stamp)`；
> ③ 查不到就用 250 Hz 墙钟轮询到"能查到"为止 ⇒ 得到**等待时间**；
> ④ 记录 `odom→base_link` 每条 TF 的 stamp 与到达时刻；⑤ 订阅 slam_toolbox 自己的 `/pose`
> （源码 `slam_toolbox_common.cpp:624`，只在 `addScan` 成功那一支调用）⇒ **不改 slam_toolbox 源码**
> 就能拿到"哪些 `/scan` 真被拿去匹配了"；⑥ 记录 `/map` 每次发布的格子数。
>
> 数据：`q01b` 跑（`scan_queue_size=1`，300 s 墙钟驱动，RTF p50 = 0.80，3144 条 `/scan` / 320.4 仿真秒）。

### 6.1 链路

```
插件 /livox/lidar (10 Hz 仿真)  ──┬─→ linefit 去地面 ─→ p2l ─→ /scan   （stamp = 点云 header；frame=livox_frame）
                                  └─→ small_point_lio ─→ /odom + TF odom→base_link （stamp 同一格，**但要整帧算完才发**）
slam_toolbox: MessageFilter(queue=1) 需要 odom→livox_frame @ /scan.header.stamp
              = (odom→base_link)(@stamp) ∘ 静态(base_link→livox_frame)
```

### 6.2 实测数字 `[V]`

| 量 | 值 | 说明 |
|---|---|---|
| `/scan` 端到端延迟（墙钟） | p50 **55.8 ms**、p90 **148.8 ms**、mean 81.8 ms | 插件→linefit→p2l→DDS→探针；**stamp 用的是点云时间，不是发布时刻** |
| `/scan` 端到端延迟（仿真钟） | p50 0.000、p90 0.100 | 被 10 Hz（0.1 s）的 `/clock` 量化，只能当上界看 |
| `/scan` 频率 | 10.0 Hz（间隔 p50 = 0.100 仿真秒 / 0.1247 墙钟秒） | |
| `odom→base_link` TF 条数 | 2865 条 / 320.4 仿真秒 = **8.94 Hz**（不是 10 Hz） | 间隔 p50 = 0.100、p90 = **0.198**、max 0.202 ⇒ **LIO 跳掉了 11.7% 的帧** |
| TF stamp 与 `/scan` stamp 的关系 | 到最近 `/scan` stamp 的距离 p50 = **0.0000**、p90 = 0.001 秒 | 两者**同一时间格**（同一帧点云的 header 戳）⇒ 差的是**发布先后**，不是时间基不同 |
| TF 到达滞后（相对它自己的 stamp） | p50 = **0.002 秒**、p90 = 0.10、p99 = 0.90 | LIO 一发就是"当前的"；不是"戳在未来" |
| **到达时 TF 可查吗** | 可查 7.4%，**查不到 92.6%** | 复刻 MessageFilter 判据：`can_transform(odom, livox_frame, stamp)` |
| 查不到时的等待（墙钟） | p50 **12.9 ms**、p90 **136 ms**、p99 163 ms、max 1.82 s | 其中 **41.8% > 一个扫描周期**（0.1247 s 墙钟） |
| **被丢帧的等待** | p10/p50/p90 = **126.5 / 131.4 / 139.0 ms** | 极其集中：≈ **一个 LIO 帧**（0.1 仿真秒） |
| 全体阻塞帧的等待 | p50 = 12.9 ms | ⇒ 被丢的是"等了一整帧"的那一小撮 |
| 队列语义判据命中率 | 被丢的 109 条里 **97.2%** 满足 `wait > 后一条扫描的到达间隔`（91.7% 满足 `> 前一条`） | 与 §3 的 `QueueFull` 语义一致 |

**跨跑一致性**（同一套仪器，两次跑）：

| 量 | q01b（`scan_queue_size=1`，3144 条） | q05（`scan_queue_size=5`，3223 条） |
|---|---|---|
| `/scan` 端到端延迟（墙钟，从 `/clock` 对齐算出） | p50 **55.8 ms**、p90 148.8 ms | p50 **53.6 ms**、p90 58.2 ms |
| 到达时 TF 查不到的比例 | **92.6%**（86.5% 被轮询解决 + 6.1% 到结束仍未解决） | **100%**（每一条都要等） |
| 等待（墙钟） | p50 12.9 ms、p90 **136.0 ms** | p50 10.6 ms、p90 **136.1 ms** |
| 被丢帧数 | **116（3.69%）** | **0（0.00%）** |
| 真被匹配的帧（`/pose`） | 103 | 16 |

⇒ 两次跑的**时间戳链形状完全一致**（"`/scan` 总是先到、等一帧 TF"），差别只在队列深度：
**队列 1 → 4% 被顶掉；队列 5 → 一条不丢**。这是本文最直接的一组因果证据。

### 6.3 真因：**1 纳秒**的戳错位（逐纳秒实测）

把戳都换算成**整数纳秒**再比（float64 的 `sec + nanosec*1e-9` 在这个量级能无损还原 ns）：

**(a) `/scan` 的戳，在 TF 缓存里能不能找到"stamp ≥ 该戳"的样本？**（q01b，queue=1，3144 条）

| 情况 | 条数 | 占比 | 其中被 `QueueFull` 顶掉 |
|---|---|---|---|
| 同纳秒就有一条 `odom→base_link`（Δns = 0） | 1658 | 52.7% | **0 条** |
| 第一条 stamp ≥ 该戳的 TF 在 **+1 ns**（同帧，但戳低了 1 ns） | 1118 | 35.6% | 90 条（8.0%） |
| 该戳被 LIO **整帧跳过**（下一条 TF 在 +0.1 s） | 368 | 11.7% | 26 条（7.1%） |
| 其它（+0.19~0.2 s） | — | — | 少量 |

**116 条丢帧的 Δns 全部 > 0**（典型值 `99 999 999 ns` / `100 000 000 ns` / `195 000 000~199 000 000 ns`）——
**没有一条丢帧是 Δns = 0 的**。

**(b) 那 1 ns 是从哪来的？** 直接比"LIO 自己发的戳"与"点云的戳"（q05，3223 条 `/scan`，
用 `/Odometry`——它与 `odom→base_link` 出自**同一个** `time_msg`）：

| `/Odometry` 戳 − `/scan` 戳 | 条数 | 占比 |
|---|---|---|
| **0 ns** | 1697 | 60.0% |
| **−1 ns** | 1130 | 40.0% |
| 其它 | 0 | 0% |

源码（改前 `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:55-57`）：

```cpp
builtin_interfaces::msg::Time time_msg;
time_msg.sec = std::floor(odometry.timestamp);
time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);  // ← 截断
```

* float64 复算：`(ts - floor(ts)) * 1e9` 对 `ts = 18.2 / 25.2 / 33.8 …` 会算出 `199999999.99999xxx`，
  `static_cast<uint32_t>` 是**截断** ⇒ 199 999 999（**低 1 ns**）；在 0.1 s 网格上这类值约占 6%。
* 实跑到 40%，是因为 `odometry.timestamp` 不是"点云 header"而是**最后一个被处理样本的时间**
  （`small_point_lio/src/small_point_lio/small_point_lio.cpp:146-148`，由 `header + offset_time` 浮点累加而来），
  更容易落在 `X.99999999x` 一侧。
* 于是：**同一帧的 TF 戳比 `/scan` 的戳低 1 ns**。
  tf2 在扫描戳上要求 `stamp >= 该戳`（插值区间必须**包住**查询时刻），低 1 ns 的同帧样本
  **不合格** ⇒ 只能等**下一帧**的 TF（~0.1 s 仿真秒后）⇒ 队列深度 1 时被顶掉。
* 注意这**不是** `transform_timeout` / `tf_buffer_duration` 能救的：它们管的是"等多久"和"存多久"，
  而这里的问题是"**要等的那个样本根本还没产生**"。

**(c) 为什么只有 ~8% 的"戳对不上"最终变成丢帧？**（见 6.4）

### 6.4 是否真丢，取决于单线程 executor 的毫秒级竞态

* 按"消费者立刻 `add`"预测会丢 **1117 条（35.5%）**，实际只丢 116 条（3.7%）⇒ 召回 0.85、精度 0.09。
* 差额来自**串行化**：`async_slam_toolbox` 用 `rclcpp::spin`
  （`slam_toolbox_async_node.cpp` 末尾 = `SingleThreadedExecutor`），`/scan` 与 `/tf` 回调在同一线程；
  TF 到位时会**在 `/tf` 回调里直接派发**在等的扫描（§3 第 1 条）。
  只要 executor 把"下一条 `/scan` 的 `add()`"推迟到 TF 到达之后，这帧就**不丢**。
* 但这也解释了另一件事：**LIO 跳帧/戳错位的那 47% 帧，等待 ≈ 一个 LIO 帧（0.1 s 仿真 = 0.125 s 墙钟）
  恰好略大于 `/scan` 的到达间隔（0.1247 s 墙钟）** ⇒ 是一次"毫秒级平局"：
  | 量 | 值 |
  |---|---|
  | 被丢帧的等待（墙钟） | p10/p50/p90 = **126.5 / 131.4 / 139.0 ms** |
  | `/scan` 到达间隔（墙钟） | p50 = **124.7 ms** |
  | 被丢帧满足 `wait > 后一条到达间隔` 的比例 | **97.2%** |

### 6.5 那个 2.5 仿真秒的节律（部分可解释，如实登记）

* 丢帧事件间隔中位数 = **25 条 `/scan` = 2.5 仿真秒**（自相关 lag=25 上 **0.70**），
  与前置文档、本次四次跑一致 ⇒ **可复现**。
* 现在能解释一部分：**"该戳上 TF 样本对不上"这个触发条件本身就带 2.5 s 节律** ——
  Δns>0 的指示序列在 lag=25 上的自相关 = **0.56**（lag=5 上 0.65，说明还叠了一个 ~0.5 s 的短周期），
  这来自 LIO 侧的"哪些帧的 `odometry.timestamp` 会落到 X.99999999x 一侧"的浮点/样本交错模式
  （IMU 200 Hz × 点云 10 Hz × 每帧 30000 点的相位）。
* 但仍**没有定位到"为什么恰好 25 帧"**。已排除的候选（都是同一跑的序列，自相关 lag=25）：
  | 候选 | ac[25] | 结论 |
  |---|---|---|
  | `/scan` 到达延迟 | +0.28 | 不是驱动源 |
  | `/scan` 到达间隔 | +0.08 | 同上 |
  | 全体 TF 等待 | +0.20 | 同上 |
  | LIO 跳帧事件（无同戳 TF） | +0.23 | 只解释 11.7% 的那部分 |
  | LIO TF 到达滞后（大滞后事件） | +0.15 | 同上 |
  | `/map` 发布（实测间隔 p50 = 4.0 s） | 与随机无差别 | 排除 |
  | 被处理帧 `/pose` 事件（理智段间隔 p50 = 7.2 s） | 0.25 s 内 28.6% vs 随机 19.6% | 只解释一小部分 |
* `[I]` 推断：节律 = "Δns>0 的模式"（ac 0.56）再乘上"executor 何时会把两条 `/scan` 背靠背 `add`"
  的调度竞态（§6.4）。**这不影响根因与修法**：无论节律从哪来，能丢的帧必须先是 Δns > 0 的帧，
  而 Δns > 0 正是本文要修的那个 1 ns 错位 + LIO 跳帧。列入 §11 未验证。

---

## 7. 丢帧的影响：被丢的帧本来会被用吗（结论：几乎都不会）

### 7.1 处理门（源码 `slam_toolbox_common.cpp:512-557`）

`shouldProcessScan()` 的顺序是：`throttle_scans`（=1）→ `minimum_time_interval`（**≥0.5 s**）
→ `0.8 × minimum_travel_distance²`（= 0.8×0.5² = **0.2 m² ⇒ ≥0.447 m**，用 odom 位姿算）→ `scan_ctr ≥ 5`。
⇒ 一条 `/scan` 要被真正拿去匹配，必须**同时**满足"距上一次被处理 ≥0.5 仿真秒"**且**"车走了 ≥0.447 m"。

### 7.2 实测（同一跑，`/pose` = 真的被 `addScan` 的帧）`[V]`

| 量 | 值 |
|---|---|
| `/scan` 总数 | 3144 |
| 真的被匹配的帧（`/pose` 事件） | **103（3.3%）** |
| 被 MessageFilter 顶掉的帧 | 116（3.7%） |
| 理智段（<250 s）处理间隔 | p50 = **7.2 s**（车经常卡住 ⇒ **距离门**主导） |
| 发散段（≥250 s）处理间隔 | p50 = **0.5 s**（LIO 漂移 ⇒ 时间门主导） |
| 被丢帧里"**被 0.5 s 时间门证明无害**" | **45 / 116 = 38.8%**（丢在上一被处理帧之后 0.5 s 内 ⇒ 就算送到了也会 `return false`） |
| 再加"**距离门证明无害**"（离上一被处理位姿 < 0.447 m） | **68 / 116 = 58.6%** |
| 剩下"**可能被选上**"的 | 48 / 116 = 41.4% |
| 这 48 条的下一帧（+0.1 仿真秒）能否过门 | **时间门 100%、距离门 100%**（距离只会更大、时间只会更久） |
| ⇒ 每条"可能被选上"的丢帧的真实代价 | **被选中的那一帧晚 0.1 仿真秒 ≈ 2 cm**（0.2 m/s） |
| 对照：随机抽一条 `/scan` 会被门判掉的比例 | 51.3%（丢帧组 58.6%）⇒ 丢帧并不偏向"有用的帧" |

### 7.3 结论

**被丢的帧绝大多数是 slam_toolbox 本来就会忽略的帧**（时间/距离门），剩下 ~40% 的最坏代价是
"这一帧晚 0.1 仿真秒 / 2 cm 被采用"——一次 320 仿真秒的跑图里累计 ≈ 5 帧 × 0.1 s ≈ 0.5 秒的帧陈旧度。
这解释了为什么前置 A/B 里丢帧率在 4.03~4.04% 之间变化时**图质量看不出差别**：
丢帧不是"地图歪"的原因（前置文档 §4 的三条主因仍然成立：物理卡死、1.05 m 平行墙退化、坡面被地面分割吃掉）。

⚠️ 反过来说：**"无害"的结论建立在"`minimum_time_interval`/`minimum_travel_distance` 维持现在的粗粒度"这个前提上**。
如果哪一天把这两个门调细（例如 `minimum_time_interval: 0.05` 想要更密的图），被丢的那 4% 就会**立刻变成真损失**
（每丢一帧就是少一个图节点）。这条写进 §11。

---

## 8. A/B：`scan_queue_size` = 1 / 5 / 10

### 8.1 怎么跑的

* harness：`tools/scripts/mapping/mapping_ab_run.sh`（本次**新增** `--slam-scan-queue-size` 与 `--timing-probe` 两个开关；
  参数改动仍走 `trap` 逐字节还原，跑完打印 `配置已还原…yes/yes`）。
* 同一条路线 `--route .tmp_cache/hzmap2d/route_ramp.json`、同速度 0.20 m/s、同 timeout 300 s 墙钟、
  同 `lio:=small_point_lio`；每跑一次一个隔离环境（独立 `ROS_DOMAIN_ID`/Gazebo 端口/`HOME`）。
* 分母口径：丢帧数 ÷ **探针旁路收到的 `/scan` 条数所覆盖的仿真时长**（与前置文档同一把尺子）。
* ⚠️ 验证过"临时改参数真的作用到节点"：`install/share/slam_toolbox/config/*.yaml` 是指向 `src/` 的**符号链接**
  （`--symlink-install`），`diff` 为空 ⇒ 改 `src/` 就是改节点读的那份。

### 8.2 结果

| 跑次 | `scan_queue_size` | 丢帧条数 | 丢帧率（次/仿真秒） | 占 `/scan` | `/scan` 条数 | 仿真时长 (s) | 被匹配帧(`/pose`) | 到达时TF查不到 | RTF | 航点 | 轨迹长 (m) | LIO ATE RMSE (m) | `/map` 窗口 | `map→odom` 跳变 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| q01 | **1**（显式） | 121 | 0.4241 | **4.24%** | 2853 | 285.3 | — | — | 0.804 | 17/20 | 15.7 | 0.0172 | 304×335 | 3 |
| 上一轮 base（前置文档） | **1**（未配=默认） | 95 | 0.4283 | **4.28%** | 2218 | 221.8 | — | — | 0.806 | 12/20 | — | 0.0279 | 343×298 | — |
| q01b | **1** | 116 | 0.3620 | **3.69%** | 3144 | 320.4 | 103 | 92.6% | 0.802 | 11/20 | 482 ⚠️ | 71.3 ⚠️ | 4062×6846 ⚠️ | 83 ⚠️ |
| q05 | **5** | **0** | **0.0000** | **0.00%** | 3223 | 322.2 | 16 | 100% | 0.797 | 15/20 | 22.5 | 0.0862 | 324×309 | 3 |
| q10 | **10** | **0** | **0.0000** | **0.00%** | 2781 | 278.1 | 103 | 88.6% | 0.786 | 11/20 | 3944 ⚠️ | 814.7 ⚠️ | 9315×19982 ⚠️ | 90 ⚠️ |
| **fix01**（§9.4：**LIO 戳四舍五入** + 队列 1） | **1** | **10** | **0.0351** | **0.35%** | 2848 | 284.8 | 78 | 98.9% | 0.803 | 12/20 | 105 ⚠️ | 4.72 ⚠️ | 643×820 ⚠️ | 46 ⚠️ |

⚠️ = 该跑发生 LIO 发散（ATE/轨迹长/地图窗口不可用于比较），只有"丢帧数"这一列仍然有效。

**读法（诚实版）**：

1. **丢帧这一维是干净的、决定性的**：队列 1 的三次跑是 **4.24% / 4.28% / 3.69%**（同一量级，与前置文档的
   4.03~4.28% 一致）；队列 5 与队列 10 都是 **0**（0 条 / 3223 条、0 条 / 2781 条）。
2. **其它维度被"物理卡死/LIO 发散"主导，不能当因果**：q01b 与 q10 两次跑跑到 ~150~180 仿真秒时
   LIO 发散（ATE 71 m / 815 m，轨迹长 482 m / 3944 m）；q05 的 ATE（0.086 m）比 q01（0.017 m）差，
   但 q05 多跑了 6.8 m、卡死更少（`motion_checks_failed` 7 vs 14）⇒ 两次跑"难度"不同，
   差异**不能归因**给队列深度。`/map` 规模（93.4 vs 96.9 m² 已知面积）同样跟着"跑了多远"走。
3. **区域计数（`compare_2d_maps.py`，同一套窗口）**：q01 vs q05 —— 坡道区 free 5166→5536、occ 168→181；
   走廊区 free 334→710、occ 54→46。方向是"q05 覆盖率更好"，但 q05 也多跑了 6.8 m ⇒ **同样不可归因**。
   对比图：`.tmp_research/slam_queue/cmp_q01_q05.png`（研究产物，未入库）。
4. **fix01（根因修复，队列仍为默认 1）**：丢帧 **10 条 / 2848 条 = 0.35%**，且**没有 2.5 s 节律**
   （残余全部落在"LIO 还没发 TF 的启动段"与"跑飞段"，见 §9.4）⇒ **修时间戳比加队列更彻底**：
   加队列只是"不让它被顶掉"，修戳是"根本不产生那个 0.1 s 的等待"。
5. 结论：**队列深度只改"丢帧数"，在本次可测量的范围内没有改善（也没有恶化）地图/定位**。
   这与 §7 的"丢帧无害"互为印证：**既然被丢的帧本来就不参与建图，那么"不丢"自然也不会让图变好**。
   （⚠️ 但见 §7.3 的前提：门一旦调细，这个结论立刻反转。）

---

## 9. 结论与配置决定

### 9.1 根因（证据链，全部可复现）

1. **`odom→base_link` 的戳比同一帧 `/scan` 的戳低 1 纳秒**（实测 40.0% 的帧，见 §6.3）：
   上游 `small_point_lio_node.cpp:57/105` 把 float64 秒转 `(sec, nanosec)` 时用
   `static_cast<uint32_t>((ts - floor(ts)) * 1e9)` —— **截断**，浮点误差让 `X.99999999x` 变成 `X−1 ns`。
2. tf2 在 `/scan` 的戳上查 `odom→base_link` 时，要求缓存里存在 **stamp ≥ 该戳** 的样本
   （插值区间必须包住查询时刻）⇒ 低 1 ns 的同帧样本**不合格** ⇒ 只能等**下一帧**的 TF
   （~0.1 仿真秒 / 0.125 墙钟秒后）。LIO 跳帧（`q01b` 11.7%）的帧同理。
3. slam_toolbox 把扫描交给 `tf2_ros::MessageFilter`，队列深度 = `scan_queue_size` = **上游默认 1**
   （本仓 yaml 没这个键，**不是配置遗漏**）。tf2 在队列满时**顶掉最老的那条**并报 `QueueFull`
   = 我们看到的日志（`message_filter.hpp:410-419`；日志里的 `at time` 是**被顶掉那条**的戳）。
4. ⇒ 触发条件 = "这条扫描等 TF 的时间 > 下一条扫描的到达间隔"。
   **实测：被丢的 109 条里 97.2% 满足它**，被丢帧的等待高度集中在 **131 ms 墙钟**
   （= 一个 LIO 帧），而全体"需要等 TF"的帧的中位等待只有 ~11~13 ms。
5. **交叉表（最强证据）**：Δns = 0 的 1658 条扫描**一条都没丢**；116 条丢帧的 Δns **全部 > 0**（§6.3a）。
6. 是否真的丢，还取决于**单线程 executor 的毫秒级竞态**（`slam_toolbox_async_node.cpp` =
   `rclcpp::spin` ⇒ `SingleThreadedExecutor`，`/scan` 与 `/tf` 回调同线程）：
   按"消费者立刻入队"预测会丢 1117 条（35.5%），实际只丢 116 条（3.7%）
   ⇒ 召回 0.85、精度 0.09。**这解释了为什么 `transform_timeout` / `minimum_time_interval` /
   `target_frame` 三个参数都改不动它**（前置 A/B 的结论 + 本文的机制解释）。

**一句话**：不是参数配错，而是 **LIO 把 TF 的戳截断低了 1 ns**，配合 tf2 的
"stamp ≥ 查询时刻"语义 + slam_toolbox 异步模式默认队列深度 1，三者叠加成了那条刷屏日志。
上游层面的"另一种"根治补丁是 `ros2/geometry2#544`（**2022-07 提出，至今 open**）——
它让队列满时**不抢占**正在等 TF 的消息，从而不用碰时间戳；本文走的是**把戳对齐**这条路
（更根本：它同时还修掉了 `/Odometry` 的戳）。

### 9.2 影响（§7）：无害，但"无害"依赖门是粗的

* 116 条丢帧里 **58.6% 被"0.5 s 时间门 + 0.447 m 距离门"证明本来就会被忽略**；
* 剩下 41.4% 的代价上界 = **被选中的帧晚 0.1 仿真秒（2 cm）**（因为下一帧必然同时过两个门）；
* 一条 320 仿真秒的跑图里，"真有可能影响"的帧 ≈ 5 条 ⇒ 累计 ~0.5 s 的**帧陈旧度**。

### 9.3 配置决定：**`slam_toolbox` 的参数一个都不改**（保持上游默认 `scan_queue_size` = 1）

| 判据 | 结论 |
|---|---|
| 丢帧数/率 | `scan_queue_size: 5` 能把它从 ~4% 降到 **0**（§8 实测），`10` 也是 0 ⇒ 这一条上"加队列"有效 |
| 图质量 / 定位精度 / 航点 / RTF | §8 的 A/B 里**没有可归因的改善**（差异被"卡死/发散"这种随机事件主导）；也没有恶化 |
| 丢帧是否造成损失 | §7：**证明无害**（58.6% 必被忽略；其余 ≤2 cm 陈旧度） |
| 语义代价 | 上游 README 明写 async **应保持 1**；深度 >1 会把"永远处理最新一帧"的异步不变量削弱成"最多晚 depth×0.1 s"。仿真里无所谓，**实车（LIO 延迟更大、速度更高）是真实风险** |
| 是否根治 | **不是**。队列深度只是把"戳错位造成的等待"吸收掉，属于**缓解**；根治在时间戳本身（§9.1）与上游 `geometry2#544` |

⇒ **mapper 参数保持默认**；把"如果你就是想把日志刷屏消掉、且不想动其它代码"的**可逆配方**记在这里
（实测数据见 §8，未采纳）：

```yaml
# src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml
    scan_queue_size: 5     # 实测：丢帧 116(3.7%) → 0；代价 = 扫描最多晚 depth×0.1 s 被处理
```

⚠️ 三条**必须一起记住**的前提：
1. **只有当 `minimum_time_interval` / `minimum_travel_distance` 保持现在的粗粒度（0.5 s / 0.5 m）时，
   丢帧才无害。** 一旦把门调细（想要更密的图），那 4% 就是**真丢帧**，就必须修根因或配 `scan_queue_size ≥ 2`。
2. 上游若哪天合入 `geometry2#544`，"队列深度 1 + 不抢占"就同时满足两个目标。
3. `scan_queue_size` 是共用算法层参数（`docs/sim_real_contract.md` §三/§四），不能只改 `_sim` 变体了事。

### 9.4 根因修复：把 LIO 的戳从"截断"改成"四舍五入"（**已实测验证**）

**改哪**：`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp`
（两处调用点 + 新加的 `double_to_msg_time()`；文件头有完整补丁说明）。核心一行语义：

```diff
-            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);  // 截断
+            const builtin_interfaces::msg::Time time_msg = double_to_msg_time(odometry.timestamp);
+            // double_to_msg_time(): ns = llround((ts - floor(ts)) * 1e9)，带进位
```

**为什么这才是根因修复**：
* tf2 的判据是"缓存里必须有 **stamp ≥ 查询时刻** 的样本"。只要 TF 的戳比点云戳低哪怕 **1 ns**，
  同帧那个样本就**不合格** ⇒ 必须等下一帧（~0.1 s）⇒ 队列深度 1 下被顶掉。
* 所以修法不是"少等一会儿"（`transform_timeout` 做不到）、也不是"多排几条"（`scan_queue_size` 只是吸收），
  而是**让 TF 的戳与点云的戳逐纳秒相等**。截断 → 四舍五入 正是这一点。
* 这也**不是**"为仿真打的补丁"：实车 Livox 驱动的点云戳同样由 float 时间构造，
  同样可能落到扫描戳之下；而且改动只影响消息戳的最后 1 ns，对任何下游都无副作用。
* 试过但**否决**的更强版本：把戳"量化到微秒"。实测（fix01）显示 −1 ns 那一类**已经被四舍五入全部消掉**
  （剩下 4.5% 的 Δns>0 是 LIO 侧**真实的** 2 ms 量级时间差），再量化到微秒只会把真实的毫秒级时间差一起改掉、
  属于过度修改 ⇒ **不采用**。

**验证跑（fix01）**：`lio:=small_point_lio`（打了补丁）+ `slam_toolbox` 参数**全部默认**
（`scan_queue_size` 显式写 1 = 默认），同一条路线/速度/时长，独立域名与 Gazebo 端口：

| 量 | 基线（q01/q01b，未打补丁） | **fix01（打过补丁）** |
|---|---|---|
| 丢帧条数 | **95 / 116 / 121**（三次跑） | **10** |
| 占 `/scan` 比例 | 3.69% ~ 4.28% | **0.35%** |
| 丢帧节律 | 每 **2.5 仿真秒**一条（min=p50=p90=2.5） | **没有节律**：2.4/5.3/7.8/12.3/16.8/23.5/28.0/30.9 s（全在启动段）+ 301.2/305.1 s（跑飞段） |
| 等待 TF 的时间（墙钟，阻塞时） | p50 12.9 ms、**p90 136.0 ms** | p50 **7.5 ms**、**p90 13.3 ms** |
| "消费者立刻入队"预测的 QueueFull 条数 | **1117**（35.5%） | **3**（0.093%） |
| `/scan` 的戳在 TF 缓存里"同纳秒有样本" | 52.7%（q01b） | **95.5%** |
| `/Odometry` 戳 − `/scan` 戳 = **−1 ns** 的帧 | **40.0%**（q05/q10） | **0%**（剩下 4.5% 是 −2 ms 量级的**真实** LIO 时间差） |
| 那 10 条残余丢帧的 Δns | — | 16198 / 13298 / 10798 / 6298 / 1798 ms（**LIO 还没开始发 TF 的启动段**）+ 198 ms×2 + 0 + 0（跑飞段） |

⇒ **系统性丢帧（4%、2.5 s 节律）被彻底消除**；残余 10 条全部来自
"LIO 还没输出 TF 的启动阶段"与"LIO 跑飞阶段"的**真实数据真空隙**，
不是戳转换问题（那两类在打补丁前也存在，只是被 116 条系统性丢帧淹没了）。

**代价 / 风险**：改动只在 `odom→base_link` 与 `/Odometry` 的消息戳上（±1 ns 级别，物理含义不变），
不触碰 LIO 的滤波/建图逻辑；`/scan`、`map→odom`、TF 树拓扑、单发布者契约**全部不变**。
A/B 里 RTF 0.803（基线 0.80~0.81），无性能影响。

---

## 10. 回滚

* 本次 A/B 全部由 `tools/scripts/mapping/mapping_ab_run.sh` **临时**改参数（`trap` 里逐字节还原，
  每次跑完打印 `配置已还原（与跑前逐字节相同：yes / yes）`）⇒ **跑图参数文件在跑完后与 HEAD 一致**。
  校验：`git diff --stat -- src/rm_localization/slam_toolbox/config/ src/rm_perception/pointcloud_to_laserscan/config/` 应为空。
* 仓库里的新增物：`tools/scripts/diag/scan_tf_timing_probe.py`（**一次性测量工具**，不被任何 launch 启动、
  不进默认链路）、本文、以及 `mapping_ab_run.sh` 新增的两个开关
  （`--slam-scan-queue-size` / `--timing-probe`）。回滚 = `git revert <commit>`（或删掉那两个开关的判断块），
  **运行期行为零依赖**。
* **本次真正的代码改动只有一处**：`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp`
  （戳构造：截断 → 四舍五入）。回滚 = `git revert <commit>` 后
  `colcon build --symlink-install --packages-select small_point_lio`。
  该改动只影响 `odom→base_link` 与 `/Odometry` 的消息戳（±1 ns），
  不触碰滤波/建图逻辑 ⇒ 回滚无残留状态。
* 如果最后决定再改 `mapper_params_online_async_sim.yaml` 的 `scan_queue_size`：
  回滚 = 删掉那一行（即回到上游默认 1）。**本次没有采纳这条**（§9.3）。

---

## 11. 未验证 / 打折清单

1. **2.5 仿真秒的节律没查到驱动源**（§6.5 排除表）。它的存在是实测的、可复现的，但"为什么恰好 2.5 s"
   本文给不出证据级答案。它不影响 §7/§8/§9 的结论（丢帧的代价由门判据决定，不由节律决定）。
2. **每次跑图的"图质量"受物理卡死/LIO 发散主导**：本次 q01b 就发生了 LIO 发散（ATE RMSE 71 m、
   地图 4062×6846 格）。因此 §8 的图质量对比只能"如实报告"，不能当稳健结论；
   稳健的只有**丢帧计数**这一维（见 §8 的方差说明）。
3. **`scan_queue_size` 只测了 1 / 5 / 10**（按任务口径）。"最小可行值"（2 或 3）没测；
   §3/§8 的机制分析说明 2 就够吸收"一帧 TF 延迟"，但**没有实测**。
   （该配方最终**未采纳**——根因修复已把系统性丢帧消掉，见 §9.3/§9.4。）
4. **没有测"改时间戳链"的其它方案**：§9 列了 `p2l target_frame: odom`（把"等 TF"搬到 p2l）。
   它理论上能消掉丢帧，但会把 `/scan` 的 frame 从 `livox_frame` 换成 `odom`，
   而 p2l 的 `min_height/max_height` 是**相对输出帧**的高度切片（这正是前置 A/B 里
   `target_frame: base_link` 让有限束数 −29% 的原因）⇒ 需要连带重调高度切片，
   风险与工作量都超出本次范围，**没做**。
5. **`/clock` 只有 10 Hz** ⇒ 所有"仿真时间"量测被量化到 0.1 s（= 一个扫描周期）。
   本文的等待/延迟统计以**墙钟**为主、仿真钟只作参考；`/scan` 端到端延迟的仿真口径因此只能当上界。
6. **没测 CPU**：harness 未采 CPU（用 RTF 当算力代理）。`scan_queue_size` 只影响"TF 到位时多派发几条
   `laserCallback`"（每条都是 `getOdomPose` + 门判据，绝大多数立即返回），预期开销可忽略，但没量。
7. **"其他项目怎么配"是抽查不是普查**：GitHub 代码搜索 API 未鉴权返回 401（`/search/code`），
   所以只逐仓库核对了 COD 2026 / TurtleBot 4 / uOttawa rover 三个（§5），并核对了上游 README/默认 yaml。
8. **离线重放 `shouldProcessScan` 失败**：用 `/odom` 插值复算处理门，只能对上实际 `/pose` 的 22%
   （距离门对 odom 插值/坐标系细节敏感）⇒ §7 改用**逐帧门判据**（时间是精确的；距离用同一 `/odom`
   在"上一被处理戳"与"被丢戳"两点上取值，两点都用同一来源，误差被差分抵消），没有用重放结论。
9. **实车外推**：本仓实车栈的 `odom→base_link` 由 `lio_tf_adapter` 发（`docs/tf_interface_contract.md`），
   LIO 的延迟/CPU 与仿真不同。§7 的"无害"结论**只在"门是 0.5 s / 0.447 m 这一档"时成立**；
   实车若把门调细，丢帧就会变成真损失（见 §7.3 的警告）。
10. **`fastlio` / `pointlio` 两个 LIO 槽位没有查/改**：它们由别的会话占着未提交改动
    （`git status` 里 `m src/rm_localization/FAST_LIO`、`m .../point_lio`），本次**一律不碰**；
    这两个槽位是否也有同类戳构造问题**未验证**（`grep` 未命中同样的 `static_cast<uint32_t>` 写法，
    但它们各自有 `lio_tf_adapter` 那条链路，未逐行核对）。
11. **fix01 那次跑本身也发生了 LIO 发散**（ATE 4.72 m、地图 643×820），所以它的
    "图质量/ATE"列**不可用于比较**；但**丢帧计数**（10 条，且时间戳分布不再是 2.5 s 节律）是有效的。
    跟基线三次跑相比，打补丁后的丢帧**数量级**差异（116 → 10）远大于这类跑间波动。

---

## 参考（URL）

**上游源码/文档**

1. `scan_queue_size` 声明与 MessageFilter 构造（本仓 2.6.10）：
   `src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:155-156,240-245`；
   成员声明 `include/slam_toolbox/slam_toolbox_common.hpp:148`
2. 上游 README（参数表，"Should always be set to 1 in async mode"）：
   https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md
3. 引入 `scan_queue_size` 的 PR/commit（2022-08-24）：
   https://github.com/SteveMacenski/slam_toolbox/pull/526 ，
   https://github.com/SteveMacenski/slam_toolbox/commit/cede9e8248abf791fcdc45b80ebf9bdabc20f637
4. 上游默认参数文件：https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
5. tf2 `MessageFilter` 源码（本机 Humble 头文件 = 上游 humble 分支）：
   `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp:113,410-421`；
   https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/message_filter.hpp
6. `geometry2` 的根治补丁（**open**）：https://github.com/ros2/geometry2/pull/544

**上游 issue**

7. https://github.com/SteveMacenski/slam_toolbox/issues/516 （机制 + bag 复现）
8. https://github.com/SteveMacenski/slam_toolbox/issues/720 （同症状，TF 树）
9. https://github.com/SteveMacenski/slam_toolbox/issues/777 （TF 树被 sick_scan_xd 污染）
10. https://github.com/SteveMacenski/slam_toolbox/issues/806 （缺 odom）
11. https://github.com/SteveMacenski/slam_toolbox/issues/834 （里程计时间基不一致）
12. https://github.com/SteveMacenski/slam_toolbox/issues/594 （队列问题无维护者答复）

**可比项目**

13. COD 2026（本仓 vendored，`third_party/cod_nav_2026/`）：
    `src/cod_bringup/params/mapper_params_online_async.yaml`、`src/cod_bringup/launch/multiplenav_launch.py:60-94`
14. TurtleBot 4：https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/config/slam.yaml
15. uOttawa Mars Rover Team `rover_workspace`（GitLab）：
    https://gitlab.com/uorover/rover_workspace/-/blob/master/src/autonomous_navigation/config/mapper_params_online_async.yaml

**本仓前置文档**

16. `docs/mapping_2d_from_cloud.md` §2（丢帧量化 + 三个变体 A/B）、§9（把 `scan_queue_size` 列为待做实验）
17. `docs/research_p2l_scan_stall.md`（p2l 侧 tf2 过滤器/`queue_size` 的源码考据）
18. `docs/tf_interface_contract.md`（`map→odom` / `odom→base_link` 单发布者契约）
