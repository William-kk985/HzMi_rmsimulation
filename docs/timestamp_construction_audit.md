# 时间戳构造审计（全仓）：1 ns 截断类 bug 及其它戳/队列/QoS 风险

> 起因：`a281636` 修掉 `small_point_lio` 的「float64 秒 → 纳秒**截断**」后，我们第 3 次被
> 时间戳/QoS 类问题咬到（`map→odom` 盖戳、Livox `CustomMsg` 的 `timebase=0`、这次的 1 ns 截断）。
> 本文是把**同一类**问题在整个工作区里扫一遍的结果：先读代码，再对能安全测的做**实测**，
> 最后只改「证据无歧义 + 不在子模块 + 不在已验证导航栈关键路径」的东西。
>
> 结论先行（细节见各节）：
> · **`FAST_LIO` 有完全相同的 bug，已实测**：Δ(TF戳 − `/scan`戳) = **−1 ns 占 40.0%**，
>   `slam_toolbox` 丢帧 **126/3003 = 4.2%**（2.5 s 节律）——与修复前的 `small_point_lio` 逐项吻合；
>   同一条路线上的 `small_point_lio`（修复后）是 **Δ=−1 ns 占 0.0%、丢帧 0 条**。
> · **`point_lio` 代码逐字相同**（`publish_odometry_without_downsample` 在仿真配置里是 `false`
>   ⇒ 走的是同一条路径），判定同一 bug；本轮**没能实测**——因为该槽位在仿真里第 100 帧就被自己的
>   `pcd_save` 打死（`/odom` 一条不发），见 §4.4 与 §9。
> · 除这两个**子模块**（只报告、给补丁）与本仓已修的 `small_point_lio` 外，
>   全仓**没有第 3 处**截断类构造（含 `rm_perception` / `rm_simulation` / `rm_driver` /
>   `slam_toolbox` / `icp` / `gicp` / `teb` / `cartographer_ros`，逐一核过，见 §3）。
> · 真正把「1 ns」放大成「丢数据」的只有 `slam_toolbox` 的 `scan_queue_size=1`；
>   costmap 侧是 `MessageFilter(queue=50)` + `transform_tolerance`，**只会多等一帧、不会丢**。
> · 本轮**改的代码只有工具**（`tools/scripts/diag/`，两个文件），
>   被验证过的导航栈**一行未动**；两个 LIO 子模块**一行未动**。

---

## 0. 读法：本文的标记约定

| 标记 | 含义 |
|---|---|
| **【事实】** | 有可复现证据：代码行、实测数字、或落盘产物路径 |
| **【推断】** | 由事实推导但未直接测量（给出推导链与可证伪点） |
| `file:line` | 相对仓库根 `/home/weicheng/HzMi_rmsimulation` |
| `[V]` | 已验证（verify）；`[R]` | 引用他人/上游结论 |

---

## 1. 我们要找的到底是什么（可复用的判据）

### 1.1 已修 bug 的因果链（4 步，每步都可单独验证）

```
① 仿真插件用**仿真钟**给点云盖戳：stamp = node_->get_clock()->now()   ← rclcpp::Time，整数纳秒，精确
   src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp:185,189,195
② /scan 这条链**不经过 LIO**，逐字段抄 header：
   插件 /livox/lidar/pointcloud →(linefit)→ /segmentation/obstacle →(pointcloud_to_laserscan)→ /scan
   ground_segmentation_node.cc:135-136（header = msg->header）
   pointcloud_to_laserscan_node.cpp:149（scan_msg->header = cloud_msg->header）
   ⇒ /scan 的戳 == 插件戳（精确，整数纳秒）
③ 另一条链（LIO）把插件戳转成 float64 秒再转回整数纳秒：
   small_point_lio（修复前）: nanosec = static_cast<uint32_t>((ts - floor(ts)) * 1e9)   ← **截断**
   同一个 double 在 ~1e3 s 处的往返误差 ~1e-7 ns 量级，方向随数值而变 ⇒ 约 40% 的帧落在 X.99999999x
   ⇒ 截断后变成 X−1 ns ⇒ **TF 戳比同一帧 /scan 戳低 1 纳秒**
④ tf2 的 MessageFilter 判据是「在**消息自己的戳**上 canTransform」：
   /opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp:564
       buffer_.canTransform(target, frame_id, fromRclcpp(stamp), NULL)
   低 1 ns 的同帧样本**不合格**（会被判成"要外推到未来"）⇒ 只能等下一帧 TF（~0.1 s）；
   队列深度 1 时，新来的 /scan 会把还在等的那条顶掉：
   message_filter.hpp:411-419   if (messages_.size() + 1 > queue_size_) → QueueFull
   日志原文：`discarding message because the queue is full`（slam_toolbox 侧打印）
```

**为什么「1 ns」值得花一整轮审计**：③ 里的误差本身无害，④ 的判据把它变成了**整帧丢弃**。
换句话说：**同一个 1 ns，在「按消息戳精确查 TF」的消费者身上是丢帧，在「按最新可用时间查」的消费者身上完全无害**。
审计的关键不是"哪里有 double"，而是**"哪里在用自己造出来的戳做精确查询，而那个戳来自另一条流水线"**。

### 1.2 三个可机械搜索的模式（本次全仓扫的就是这三个）

| 模式 | 形态 | 为什么要查 |
|---|---|---|
| **P1 截断构造** | `uint32_t nanosec = <double>;`、`static_cast<uint32_t>((ts-floor(ts))*1e9)`、`(int)(t*1e9)`、`fmod(t,1.0)*1e9` | 直接产生「低 1 ns」 |
| **P2 自造戳做精确查询** | `lookupTransform(..., stamp)`（stamp 自己算的/来自另一条链）、`MessageFilter(..., target_frame, ...)`、`tf2_buffer_.transform(cloud, ..., timeout)`（内部用 `tf2::getTimestamp(in)`） | 1 ns 在这里变成失败 |
| **P3 队列深度 1 + 阻塞等待** | `scan_queue_size=1`、`KeepLast(1)`、`queue_size=1` | 把「等一帧」放大成「丢一帧」 |

复现用的 grep（本轮实际执行的命令，可直接复用）：

```bash
# P1：截断构造
grep -rn --include=*.cpp --include=*.hpp --include=*.h --include=*.py -E \
  "uint32_t[[:space:]]+[a-z_]*(nanosec|nsec|ns)[a-z_]*[[:space:]]*=[^=]|\(uint32_t\)[[:space:]]*\(|\(int\)[[:space:]]*\([^)]*1e9|%[[:space:]]*1000000000|fmod\([^)]*1\.0\)" src/
# P1 变体：把 1e9 / 1e-9 附近的所有点都拉出来人工过（本轮共 208 处命中，逐条判过）
grep -rn --include=*.cpp --include=*.hpp --include=*.py -E "1e9|1e-9|1000000000" src/
# P2/P3：查 TF 与过滤器
grep -rn --include=*.cpp --include=*.hpp -E "lookupTransform|MessageFilter|message_filters::Subscriber|canTransform" src/
```

---

## 2. 仪器：怎么把「1 ns」直接量出来

### 2.1 复用 + 扩展：`tools/scripts/diag/scan_tf_timing_probe.py`

一次性旁路探针（不被任何 launch 启动、不参与链路），本轮**扩展**了一节：

* 新增 **整数纳秒序列** `self.scan_ns` / `self.tf_ob_ns`（直接从消息的 `sec`/`nanosec` 拼，
  **不经过 double**），并落进 `<out>.npz`（键名 `scan_ns` / `tf_ob_ns`）；
* 新增汇总指标 `stamp_delta_ns_tf_minus_scan`：逐 `/scan` 找时间最近的 `odom→base_link` TF 样本
  （同帧窗口 ±5 ms），给出 `Δ = TF戳 − scan戳` 的 `Δ=0` / `Δ=−1 ns` 百分比与 |Δ|≤10 ns 直方图。

> 口径说明：老的 npz 没有整数键，新指标会自动退回「`round(sec + nanosec*1e-9)` 重建整数」的等价算法，
> 因此**旧数据也能用同一把尺子复算**（下面 q05/fix01 就是这么算的）。
>
> 已在本轮的真实跑里验证过：`ts_spl_postfix` 的 `timing.json` 直出
> `stamp_delta_ns_tf_minus_scan = {n_matched: 3118, exact_0ns_pct: 100.0, minus_1ns_pct: 0.0}`。

### 2.2 新增：`tools/scripts/diag/stamp_delta_ns_report.py`（离线复算器）

读探针的 npz，输出每个跑次的 Δ 分布 + 保守判定（`TRUNCATION-FINGERPRINT` / `OK` / `OTHER`）。
它是本轮的**主测量仪**，跑完不用再写一次性脚本：

```bash
python3 tools/scripts/diag/stamp_delta_ns_report.py --npz \
  .tmp_cache/slamq/q05/q05.timing .tmp_cache/slamq/fix01/fix01.timing \
  .tmp_tsaudit/fastlio/ts_fastlio.timing --json /tmp/delta.json
```

### 2.3 单元级复现：忠实的 C++ 对照（不依赖 ROS/仿真）

`FAST_LIO`/`point_lio` 的写法与修复后的写法各跑一遍，输入是**真实的 3223 条仿真戳**
（取自 q05 的 `/scan` 戳序列，即插件戳的真值）：

```cpp
// 上游（FAST_LIO/include/common_lib.h:262-268 = point_lio/include/common_lib.h:215-221）
int32_t sec = std::floor(ts);
auto nanosec_d = (ts - std::floor(ts)) * 1e9;
uint32_t nanosec = nanosec_d;              // ← 隐式截断
// 本仓修复版（small_point_lio_node.cpp:40-52）
int64_t ns = static_cast<int64_t>(std::llround((ts - sec_d) * 1e9));  // + 进位
```

**实测输出【事实】**：

```
N=3223  截断版 < 真值: 1289 (39.99%)   四舍五入版 < 真值: 0 (0.00%)
```

⇒ 「约 40% 的帧低 1 ns」不是估计值，是这两种写法在**我们真实戳序列**上的确定性结果。
（复现物：`.tmp_tsaudit/repro_trunc.cpp`，未入库；命令见附录 A。）

---

## 3. 审计表

> 覆盖范围（逐个包读完）：`src/rm_localization/{FAST_LIO, point_lio, small_point_lio, icp_registration,
> gicp_registration, lio_tf_adapter, slam_toolbox}`、`src/rm_perception/{linefit_ground_segementation_ros2,
> patchwork_ground_segmentation, pointcloud_to_laserscan, imu_complementary_filter}`、
> `src/rm_simulation/{hzmi_rm_simulation, livox_laser_simulation_RO2}`、`src/rm_driver/livox_ros_driver2`、
> `src/rm_navigation/**`、`src/rm_nav_bringup/**`，另附 `src/rm_localization/cartographer_ros` 的
> 时间转换（顺手核过）。`third_party/**` 与 apt 安装的 nav2/slam_toolbox 源码只在"被本仓调用到的行为"上引用。

### 3.1 截断类（同一 bug 家族）

| # | file:line | 代码 | 类别 | 判定 | 影响面 | 建议 / 已修 |
|---|---|---|---|---|---|---|
| **A1** | `src/rm_localization/FAST_LIO/include/common_lib.h:262-268` | `int32_t sec = std::floor(timestamp); auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9; uint32_t nanosec = nanosec_d;` | 截断 | **有 bug（实测确认）** | `/Odometry`（→`/odom`）、`camera_init→body` TF、`/cloud_registered`、`/path` **全套戳**；实测 Δ=−1 ns **40.0%**、丢帧 **126 条（4.2%）/2.5 s 节律** | 子模块，本次**只报告**；补丁见 §7.2 |
| **A2** | `src/rm_localization/point_lio/include/common_lib.h:215-221` | 同上（逐字相同） | 截断 | **有 bug（代码等价 ⇒【推断】；未实测）** | 同 A1（`/Odometry`→`/odom`、`tf_send_en` 时的 `camera_init→aft_mapped`、`/cloud_registered`） | 同上；另建议把 `publish_odometry_without_downsample` 钉死为 `false`（见 §4.3） |
| **A3** | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:40-52` | `llround` + 进位（`double_to_msg_time`） | —— | **已修**（`a281636`） | 修后 Δ=−1 ns **0%**、Δ=0 **95.5%**、丢帧 **116 → 10** | 已修，勿动 |
| **A4** | 单元级复现（§2.3） | 两种写法对照 | 截断 | **复现【事实】** | 真实 3223 条戳：截断 1289 条（39.99%）低 1 ns；四舍五入 0 条 | 作为 A1/A2 判定的直接证据 |
| **A5** | `src/rm_navigation/teb_local_planner/teb_local_planner/include/teb_local_planner/misc.h:155-165` | `nsec = static_cast<int32_t>(std::round((t_sec - sec) * 1e9));` + 进位 | 构造 | **安全（正确的反面教材对照）** | 只用于 `Duration` 构造，不参与 TF 查找 | 无需改（子模块 + 已正确） |
| **A6** | `src/rm_driver/livox_ros_driver2/src/src/lddc.cpp:287,332,399` | `cloud.header.stamp = rclcpp::Time(timestamp);`（`timestamp = pkg.base_time`，`uint64_t` 纳秒） | 构造 | **安全（整数域，无截断）** | 真机路径；`livox_msg.timebase = timestamp` 与 `header.stamp` **同值**（:328-332）⇒ 印证 `small_point_lio` 适配器注释里的判断 | 无需改（子模块） |
| **A7** | `src/rm_localization/cartographer_ros/src/time_conversion.cpp:24-32` | `uts_timestamp × 100`（`10000000ll` ticks/s，全整数） | 构造 | **安全** | 不在本次范围但顺手核过 | 无需改 |

**全仓 P1 扫描的净结果【事实】**：除 A1/A2（两个子模块）与 A3（本仓已修）外，
`src/` 下**没有第 3 处**截断式时间构造。`rm_perception`、`rm_simulation`、`slam_toolbox`、
`icp_registration`、`gicp_registration`、`lio_tf_adapter`、`fake_vel_transform`、`rm_nav_bringup` 全为 0 命中。

### 3.2 相等比较 / 自造戳查 TF / MessageFilter 队列

| # | file:line | 代码 | 类别 | 判定 | 影响面 | 建议 |
|---|---|---|---|---|---|---|
| **B1** | `src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:155-156,243-246` | `scan_queue_size_ = 1.0;` / `MessageFilter(*scan_filter_sub_, *tf_, odom_frame_, scan_queue_size_, ..., tf2::durationFromSec(transform_timeout_.seconds()))` | 队列 | **脆弱（把上游 1 ns 放大成丢帧的唯一消费者）** | 实测：Δ=−1 ns 的帧被排队等待 ~0.1 s，被下一条 `/scan` 顶掉（`QueueFull`）；队列=5/10 时同一条链**0 丢帧**（q05/q10） | 消费端**不改**（上游 README 明确 async 模式应为 1）；已由 A3 从根因修掉（本轮复测：修复后同路线 **0 丢帧**） |
| **B2** | 系统头 `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp:564` 与 `:411-419` | `buffer_.canTransform(target, frame_id, stamp)`；`if (messages_.size() + 1 > queue_size_) → QueueFull` | 相等比较 | **机理【事实】** | 解释"为什么 1 ns 足够致命"：判据在**消息自己的戳**上，且深度 1 时"新消息顶掉旧消息" | 引用即可（上游代码，勿改） |
| **B3** | `third_party/nav2/nav2_costmap_2d/plugins/obstacle_layer.cpp:230-235, 265-270` | `MessageFilter(*sub, *tf_, global_frame_, 50, ..., tf2::durationFromSec(transform_tolerance))` + `setTolerance(0.05)` | 队列 | **安全（不丢，代价是延迟）** | 队列 50 ⇒ 等一帧只是排队；实测历史日志里 costmap 侧**没有** `QueueFull`（只有启动段的"earlier than all the data"） | 无需改；但见 B4 的延迟 |
| **B4** | `third_party/nav2/nav2_costmap_2d/src/observation_buffer.cpp:100-116` | `local_origin.header.stamp = cloud.header.stamp;` `tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);`（`BufferInterface::transform` 内部用 `tf2::getTimestamp(in)`，即**消息戳**） | 自造戳查 TF | **可疑（40% 的观测会多等一帧）【推断】** | 观测在**点云/扫描自己的戳**上查 TF，超时= `transform_tolerance`（本仓 local/global 均 0.2，`nav2_params_sim_base.yaml:240,340`）；LIO 帧周期 0.1 s < 0.2 s ⇒ 最终成功、不丢，但**每次多阻塞 ~0.1 s**（这是修复前 40% 帧的隐性代价；修复后归零） | 无需改（不改 nav2）；把 `transform_tolerance` **保持在 ≥ 一个 LIO 帧周期**并在文档里写明理由 |
| **B5** | `third_party/nav2/nav2_amcl/src/amcl_node.cpp:1503-1512` | `MessageFilter(*laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, ..., transform_tolerance_)` | 队列 | **安全** | 队列 10 + 容忍 0.3 s ⇒ 完全吸收 1 ns 级错位（历史日志里 AMCL 的 10 条丢帧都在启动段） | 无需改 |
| **B6** | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:99,142` | `tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg)` / `(base_link, lidar_frame, time_msg)`（自查自造的戳） | 自造戳查 TF | **安全（查的是静态 TF）** | `base_link→livox_frame` 来自 URDF ⇒ `/tf_static`，任意时刻都可查。**但**：若将来把这条边改成动态发布，这里会立刻变成"1 ns 就失败"的第二个爆点 | 保留注释；若改成动态 TF，必须补容差 |
| **B7** | `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:88` | `tf2_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero)` | 自造戳查 TF | **安全（推荐范式）** | `TimePointZero` = 取最新可用，天然不受 1 ns 影响 | 作为范式写进 §8 |
| **B8** | `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:108` | `t.header.stamp = get_clock()->now();`（`base_link→base_link_fake`） | 发到未来/最新 | **安全（by design）** | 用最新仿真时间盖戳；`spin_speed=0` 时为直通模式、不做 TF 查询（:96-99 已修过的坑） | 若开启 `spin_speed`，注意查询仍用 `TimePointZero`（安全） |
| **B9** | `src/rm_localization/icp_registration/src/icp_registration.cpp:142,202` | `map_to_odom_.header.stamp = now();`（100 Hz 线程）/ `lookupTransform(laser, odom, now(), Duration(10s))` | 用 now() 当查询戳 | **可疑但安全** | 查询戳= `now()` 要求 TF 至少新到 `now()`，实际会等下一帧；有 **10 s 超时兜底**，且只在初始化时执行一次 | 无需改；登记为"容忍 now() 但依赖超时"的写法 |
| **B10** | `src/rm_localization/gicp_registration/src/gicp_registration.cpp:1010-1012,1039,1078` | `return now() + rclcpp::Duration::from_seconds(tf_lookahead_sec_);`（默认 0.45）⇒ TF/pose 戳**发到未来** | 发到未来 | **安全（有实测依据的刻意设计）** | 源码注释记录了症状（MPPI 在 `now + transform_tolerance` 查 map→odom ⇒ 用 `now()` 盖戳必然 extrapolation）与修法 | 无需改；是"消费者按未来时间查"这一族的正解 |
| **B11** | `src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:263-273` | `msg.header.stamp = scan_timestamp + transform_timeout_;`（除非 `restamp_tf_`） | 发到未来 | **安全** | map→odom 比 scan 戳早 0.2 s 发到未来，专门喂给"按 now+tolerance 查"的规划器 | 无需改 |
| **B12** | `src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc:105-106,135-136` | `lookupTransform(gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp)`（仅当参数非空）/ `ground_msg->header = msg->header;` | 自造戳查 TF + 抄戳 | **安全（配置 `gravity_aligned_frame: ""` ⇒ 不查）** | 抄戳是"插件戳一路传到 `/scan`"的关键环节（也是 1 ns 能致命的原因） | 保持 `""`；若要用重力对齐，需面对同一类风险 |
| **B13** | `src/rm_perception/patchwork_ground_segmentation/src/patchwork_ground_segmentation_node.cpp:221,264-265` | 同 B12（与 linefit 同契约） | 同上 | **安全** | 同上 | 同上 |
| **B14** | `src/rm_perception/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp:82-96,149` | `if (!target_frame_.empty())` 才建 `tf2_`/`MessageFilter`；`scan_msg->header = cloud_msg->header;` | 队列/抄戳 | **安全（已加固）** | 本仓配置 `target_frame: ""`（`config/laserscan_params.yaml` + `docs/research_p2l_scan_stall.md`）⇒ 整类"tf2 过滤器静默排队/丢消息"不存在 | 保持 `target_frame: ""` |

### 3.3 QoS / durability / 时间基

实测手段：在 `lio:=fastlio` 的仿真跑动中执行
`ROS_DOMAIN_ID=<run> ros2 topic info -v <topic>`（只读图查询，不订阅数据），
下面是**实测到的**两端 QoS（不是从代码猜的）：

| topic | 发布者（节点/QoS） | 订阅者（节点/QoS） | 判定 |
|---|---|---|---|
| `/livox/lidar` (CustomMsg) | `livox_frame_plugin` BEST_EFFORT / KEEP_LAST 5 | `laser_mapping` BEST_EFFORT / 5 | ✅ 一致（历史修复：RELIABLE 写者会阻塞 Gazebo sensor 回调） |
| `/livox/lidar/pointcloud` | `livox_frame_plugin` BEST_EFFORT / 5 | `ground_segmentation` BEST_EFFORT / 5 | ✅ |
| `/segmentation/obstacle` | `ground_segmentation` BEST_EFFORT / 5 | `pointcloud_to_laserscan` BEST_EFFORT / 5 | ✅ |
| `/scan` | `pointcloud_to_laserscan` BEST_EFFORT / 5 | `slam_toolbox` BEST_EFFORT / 5（`rmw_qos_profile_sensor_data`） | ✅（nav 模式下另有 AMCL/costmap，见下） |
| `/odom` | `laser_mapping` **RELIABLE** / 20 | `lio_tf_adapter` BEST_EFFORT / 5 | ⚠️ 兼容（RELIABLE 写 + BEST_EFFORT 读 合法），但 nav 模式下 nav2 的 `OdomSubscriber` 是 `rclcpp::SystemDefaultsQoS()`（RELIABLE，`third_party/nav2/nav2_util/src/odometry_utils.cpp:34-37`）⇒ **控制卡顿会给 LIO 施加背压**（同一类已发生过的故障） |
| `/livox/imu` | `imu_plugin` **RELIABLE** / 5 | `complementary_filter_gain_node` RELIABLE / 10 | ⚠️ 兼容，但"RELIABLE 传感器写者"是历史坑的形态（消费者一旦卡住会阻塞 sensor 回调） |
| `/imu/data` | `complementary_filter_gain_node` RELIABLE / 5 | `laser_mapping` RELIABLE / 10 | ✅（`FAST_LIO` 的 IMU 来源是滤波后话题，见下） |

代码侧命中：

| # | file:line | 代码 | 类别 | 判定 | 说明 |
|---|---|---|---|---|---|
| **C1** | `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp:83-96` | 两个 publisher 都是 `rclcpp::SensorDataQoS()` | QoS | **安全（历史修复）** | 源码注释完整记录了"RELIABLE 写者阻塞 sensor 回调 ⇒ 整链停更 180 s"的故障与判据 |
| **C2** | `src/rm_localization/FAST_LIO/src/laserMapping.cpp:934,939-940` | IMU 订阅 `imu_topic, 10, imu_cbk`（默认 **RELIABLE** /10）；`/Odometry`、`/cloud_registered` 等 `create_publisher(..., 20)`（RELIABLE /20） | QoS | **可疑（潜在反压）** | `/Odometry` 的可靠读者（nav2 OdomSubscriber）一旦跟不上，`publish()` 会阻塞在 LIO 主循环里；**本轮未观察到**（nav 跑通），登记为可疑项。⚠️ 别把 LIO 改成 BEST_EFFORT——那会与 nav2 的 RELIABLE 读者**不兼容**（直接一条都收不到） |
| **C3** | `src/rm_localization/point_lio/src/laserMapping.cpp:348-357` | 点云 `SensorDataQoS()`；IMU `SensorDataQoS()` | QoS | **安全** | 与插件/`/livox/imu`（RELIABLE 写）兼容 |
| **C4** | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:63-64,240-244` | `/Odometry` 深度 **1000**（RELIABLE）；云/IMU 订阅 `SensorDataQoS()` | QoS | **可疑（同 C2）** | 同 C2 的背压机理 |
| **C5** | `src/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:42-44,55` | `/odom` 订阅 `SensorDataQoS()`；`tf_msg.header.stamp = msg->header.stamp;`（逐字段抄） | QoS + 抄戳 | **安全** | 它**不加不减**地转发戳——这正是"上游修好，下游自动好；上游不修，下游照样丢"的原因 |
| **C6** | `src/rm_nav_bringup/launch/bringup_sim.launch.py:423-433` | `imu_complementary_filter` 节点**没有** `use_sim_time`（参数文件 `config/imu_filter_params.yaml` 也没写） | 时间基 | **可疑（与已修的两处同类：linefit / p2l 曾漏 `use_sim_time`）** | 实测后果**目前为零**：该节点不读自己的钟、`publish_tf` 默认 `false`（`complementary_filter_ros.cpp:105`）、输出戳逐字段抄输入（:233）。**但**一旦有人打开 `publish_tf` 或加 `now()` 逻辑，就会变成"墙钟节点用仿真戳发 TF" | 
| **C7** | `src/rm_perception/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp:108-142` | 保留订阅（去掉上游的"无订阅者就退订"逻辑） | 其它 | **安全（本仓加固）** | 消除"退订后不恢复 ⇒ `/scan` 永久停更" |
| **C8** | `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp:267` | `p.offset_time = 0;`（帧内无运动，逐点==header 戳） | 构造 | **安全（且是 1 ns 判定的前提）** | 若将来做帧内运动补偿，`lidar_end_time` 会变成 "插件戳 + 真实帧长"，`/scan` 与 TF 的错位将从 1 ns 变成**几十毫秒**（那是另一类问题，需要重新定义契约） |
| **C9** | `src/rm_navigation/costmap_converter/costmap_converter/src/costmap_converter_node.cpp:151,198` | `line_list.header.stamp = now();` | 其它 | **安全** | 可视化 `PolygonStamped`，不参与 TF 查找 |
| **C10** | `src/rm_perception/imu_complementary_filter/src/complementary_filter_ros.cpp:156-173,199` | `dt = (time - time_prev_).nanoseconds() * 1e-9;` | 构造 | **安全** | 整数纳秒相减后转 double，无截断；戳来源是消息本身 |
| **C11** | `src/rm_simulation/livox_laser_simulation_RO2/include/ros2_livox/livox_points_plugin.h` + `gazebo_ros/node.hpp:179` | 插件用 `node_->get_clock()->now()`；`gazebo_ros::Node` **强制** `use_sim_time=true` | 时间基 | **安全**【事实】 | 插件戳 = 仿真钟整数纳秒（这也是为什么真值是"精确的整数纳秒"） |

---

## 4. Q1：别的 LIO 槽位有没有同一个 bug？

### 4.1 代码层：三份实现的关系

```
FAST_LIO/include/common_lib.h:262-268      get_ros_time(double)   ← 截断（隐式 uint32_t 转换）
point_lio/include/common_lib.h:215-221     get_ros_time(double)   ← 逐字相同
small_point_lio/src/small_point_lio_node.cpp:40-52  double_to_msg_time(double)  ← 本仓已修（llround+进位）
```

**调用点（决定了"哪些输出带这个 1 ns"）【事实】**：

| LIO | 输出 | 戳表达式 | file:line |
|---|---|---|---|
| FAST_LIO | `/Odometry` | `get_ros_time(lidar_end_time)` | `laserMapping.cpp:632` |
| FAST_LIO | TF `camera_init→body` | 同上 | `laserMapping.cpp:650` |
| FAST_LIO | `/cloud_registered` / `_body` / `/Laser_map` / `/path` | 同上 | `:507,559,598,664` |
| point_lio | `/Odometry`→`/odom` | `publish_odometry_without_downsample ? get_ros_time(time_current) : get_ros_time(lidar_end_time)` | `laserMapping.cpp:247-251` |
| point_lio | TF `camera_init→aft_mapped` | `odomAftMapped.header.stamp`（同上） | `:265-271` |
| point_lio | `/cloud_registered` | `get_ros_time(lidar_end_time)` | `:160,174,212` |

**关键点（也是当初漏判的原因）**：`lidar_end_time` 在仿真里**恰好等于插件给的点云戳**——
`offset_time ≡ 0`（`livox_points_plugin.cpp:267`）⇒
FAST_LIO `lidar_end_time = lidar_beg_time + points.back().curvature/1000 = lidar_beg_time`（`laserMapping.cpp:396-410`）；
point_lio `lidar_end_time = meas.lidar_beg_time + end_time/1000`（`li_initialization.cpp:202,241`）。
于是：
* **LIO 自己的点云戳 与 它自己的 odom/TF 戳** ⇒ 走**同一个 double、同一次转换** ⇒ **永远相等**（Δ=0）；
* **LIO 的 odom/TF 戳 与 `/scan` 的戳**（后者来自插件→linefit→p2l，完全不经过 LIO）⇒ 相差**恰好是那个转换误差** ⇒ **40% 的帧 −1 ns**。

⇒ 这解释了为什么"看 LIO 内部自洽"永远查不出这个 bug，只有拿**两条不同流水线**的戳对拉才看得见。

### 4.2 实测：`fastlio` vs `small_point_lio`

同一套无头跑图脚本、同一条路线（`.tmp_cache/spl2026/run4/spl4.route.json`）、
同一台仪器（探针 + `stamp_delta_ns_report.py`）：

| 跑次 | LIO 槽位 | 戳构造 | `/scan` 条数 | **Δ = TF(odom→base_link) − `/scan`** | `slam_toolbox` 丢帧 | 等待 TF（被阻塞时） |
|---|---|---|---|---|---|---|
| `ts_fastlio`（**本轮**，2026-10-06，domain 111） | `lio:=fastlio` | **截断** | 3003（探针）/2633（drive 计数） | **Δ=0：60.0%（1801）　Δ=−1 ns：40.0%（1201）** | **126 条**（4.2~4.8%，取决于分母；2.5 s 节律：120/125 个相邻间隔 = 2.5 s） | `wait_sim` p50 = **0.10 s**（正好一个 LIO 帧周期）；`predict_queuefull` 1130 条（37.6%） |
| `ts_spl_postfix`（**本轮，同一条路线**，domain 113） | `lio:=small_point_lio` | **四舍五入**（`a281636`） | 3118 | **Δ=0：100.0%（3118）　Δ=−1 ns：0.0%（0）** | **0 条**（`丢帧条数=0`） | `wait_sim` p50/p90/p99 = **0.00 s**（`wait_wall` p50 5.1 ms = 纯到达抖动）；`predict_queuefull` **0** |
| `q05`（历史，`docs/slam_toolbox_scan_drops.md` §8） | `lio:=small_point_lio` | **截断** | 3223 | Δ=0：60.0%　Δ=−1 ns：**40.0%** | 0 条（该跑 `scan_queue_size=5`） | —— |
| `q01b`（历史，同上） | 同上 | 截断 | 3144 | —— | **116 条 = 3.69%**（`scan_queue_size=1`） | —— |
| `fix01`（历史，`a281636` 的验证跑） | 同上 | **四舍五入** | 3221 | Δ=0：**95.5%**　Δ=−1 ns：**0%** | **10 条 = 0.35%**（且全在启动段/跑飞段的真实真空隙） | `wait_sim` p90 0.1 → **0.0 s** |
| `ts_pointlio`（**本轮**，domain 112） | `lio:=pointlio` | —— | 3215 | **无法测量**：`tf_odom_base = 0`（该槽位在第 100 帧 SIGABRT，见 §4.4） | 149 条，但**全是"根本没有 TF"**（`wait_sim_when_blocked: null`），不是本 bug 的指纹 | —— |

**读法（三条独立证据互相印证）**：
1. `fastlio` 的 Δ 分布（40.0% / 60.0%）与**修复前**的 `small_point_lio`（q05：40.0% / 60.0%）
   在统计上**完全一致**，而修复后（fix01）是 0% / 95.5%；
2. `fastlio` 的丢帧率（4.20%）落在**修复前** `small_point_lio` 的区间内（q01 4.24% / q01b 3.69%），
   比修复后（0.35%）高一个数量级，且同样是 **2.5 s 节律**（= 队列 1 顶掉的节律）；
3. 单元级复现（§2.3）给出同一机理的确定性解释：截断在 3223 条真实戳上产生 39.99% 的 −1 ns。

⇒ **答案：`fastlio` 有和 `small_point_lio` 完全相同的 bug【事实，实测】。
`point_lio` 的代码与调用路径等价（且仿真配置 `publish_odometry_without_downsample: false`
⇒ 与点云共用同一个 double），判定**同一 bug**【推断】**；本轮**没能实测**——原因不是戳，而是
**该槽位在仿真里跑不到测量窗口就崩了**（§4.4），这也是一个必须登记的独立问题。

#### 4.2.1 同路线复核（`small_point_lio` 修复后）——**成对对照**

`ts_spl_postfix` 与 `ts_fastlio` **同一条路线、同一脚本、同一仪器**（只换 `--lio`），
所以两行可以直接对拉（表见上）：**`fastlio`（截断）40.0% 的帧低 1 ns、126 条丢帧；
`small_point_lio`（四舍五入）100.0% 同纳秒、0 条丢帧**。
这一跑同时承担两个作用：
① 把 Q1 的"修复 vs 未修复"对照补齐到同一路线；
② 证明**本轮的工具改动没有触碰链路**（该跑用的就是被我改过的探针，指标 `stamp_delta_ns_tf_minus_scan`
   在真实跑里端到端跑通：`{"n_matched": 3118, "exact_0ns_pct": 100.0, "minus_1ns_pct": 0.0, "hist_abs_le_10ns": {"0": 3118}}`）。

### 4.3 point_lio 的额外风险（比 1 ns 严重得多，务必登记）

`src/rm_localization/point_lio/src/laserMapping.cpp:247-251`
```cpp
if (publish_odometry_without_downsample) {
  odomAftMapped.header.stamp = get_ros_time(time_current);   // = 最后一个 IMU 时刻
} else {
  odomAftMapped.header.stamp = get_ros_time(lidar_end_time); // = 点云帧尾（与 /scan 同源）
}
```
仿真配置现值 `config/pointlio_mid360_sim.yaml` → `odometry: publish_odometry_without_downsample: false`。
**一旦改成 `true`**：TF/`/odom` 的戳会变成"帧内最后一个 IMU 时刻"，与 `/scan` 的戳相差
**毫秒~十几毫秒**——那不是 1 ns 而是 **帧级错位**，`slam_toolbox`（队列 1）、AMCL、costmap
都会按这条差去等 TF ⇒ 系统性丢帧/滞后，且症状与这次一模一样但更难查。
**建议**：在文档与配置注释里把该键标注为"**不得为 true**（除非同时改 `/scan` 契约）"。

### 4.4 附带发现：`lio:=pointlio` 槽位在本 checkout 上**根本跑不起来**（不是戳问题）

本轮为测 point_lio 而跑的一次无头跑（`ts_pointlio`，domain 112）结果是**决定性否证**：

```
[pointlio_mapping-9] first imu time: 0.100000
[pointlio_mapping-9] terminate called after throwing an instance of 'pcl::IOException'
[pointlio_mapping-9]   what():  : [pcl::PCDWriter::writeBinary] Error during open!
[ERROR] [pointlio_mapping-9]: process has died [pid 150, exit code -6(SIGABRT), ...]
```

* **原因【事实】**：`config/pointlio_mid360_sim.yaml:67-68` 是 `pcd_save_en: true` / `interval: 100`，
  而 `ROOT_DIR = @CMAKE_CURRENT_SOURCE_DIR@`（源码树）下的 **`src/rm_localization/point_lio/PCD/` 不存在**
  （`FAST_LIO/PCD/` 有、`point_lio/PCD/` 没有）⇒ 第 100 帧写盘时抛 `pcl::IOException` 且**没人接** ⇒ `terminate`。
* **后果【事实】**：`/odom` 一条都不发（探针 `counts.odom = 0`、`tf_odom_base = 0`），
  驱动脚本直接放弃（`❌ 没有 /odom（LIO 没出数）`），slam_toolbox 的 `/map` **0 增长**
  （`occ 2773 → 2773`，`free 45864 → 45864`，全程不变）。
* **这不是新问题【R】**：`docs/lio_slots.md:362-366` 已记载"point_lio 25.3 m，且它在第 100 帧被自己的
  `pcd_save` 打死"，并注明"`lio:=pointlio` 在本仓其实也**从未在全栈里验证过**"。
  本轮只是**复现并量化了后果**（整个槽位不可用，而不是"精度差"）。
* **顺带一个"别误读日志"的教训【事实】**：这一跑 slam_toolbox 有 **149 条丢帧**，但
  `reason` 全是 `discarding message because the queue is full` + 2 条缓存太旧，
  **`wait_sim_when_blocked = null`**（永远等不到 TF）⇒ 它们的成因是"完全没有 TF"，
  **不是**本 bug。判据：本 bug 必然伴随 `wait_sim ≈ 0.1 s`（等一帧后成功），而不是"永远等不到"。
* **补法（提案，属子模块改动）**：① `mkdir src/rm_localization/point_lio/PCD`（不改代码，但会在子模块里
  产生输出目录）；或 ② 把 `pcd_save_en` 改成 `false`；或 ③ 用节点级参数覆盖
  （`ros2 run point_lio pointlio_mapping --ros-args --params-file ... -p pcd_save_en:=false`，
  需确认参数名层级）——**三者都要动子模块或运行命令，本轮一律不动**。

---

## 5. Q2：还有哪些地方会被 1 ns（或 1 tick）悄悄劣化？

按"消费者查询戳的来源"分类，结论只有一句话：
**用 `tf2::TimePointZero`/最新可用时间查的，天然免疫；用消息自己的戳查的，就要看队列深度与容忍度。**

| 消费者 | 查询戳 | 队列/容忍 | 1 ns 错位的实际后果 |
|---|---|---|---|
| `slam_toolbox`（MessageFilter） | `/scan` 的 `header.stamp` | **1** / 0.2 s | **丢帧**：截断时实测 126/3003 = **4.2%**；修复后同一路线 **0 条** — 唯一被放大的消费者 |
| `nav2` costmap obstacle layer（MessageFilter） | 消息戳 | **50** / 0.2~0.3 s + `setTolerance(0.05)` | 不丢；**每次多等 ~0.1 s**（推断，§3.2 B4）；实测历史日志无 costmap 侧 QueueFull |
| `nav2` costmap `ObservationBuffer::bufferCloud` | 点云/扫描戳（`tf2::getTimestamp(in)`） | 阻塞超时 = `transform_tolerance` | 同上（迟滞）；若把 `transform_tolerance` 调到 < 一个 LIO 帧周期（0.1 s）就会开始丢观测 ⇒ **别调小** |
| `nav2` AMCL（MessageFilter） | `/scan` 的戳 | **10** / 0.3 s | 无感（吸收掉） |
| `nav2` `getRobotPose` / static_layer / costmap `waitForTransform` | `tf2::TimePointZero` | —— | 无感（`costmap_2d_ros.cpp:278,740`、`static_layer.cpp:435`） |
| `fake_vel_transform` | `TimePointZero` | —— | 无感；它自己发的 `base_link→base_link_fake` 用 `now()`（最新） |
| `lio_tf_adapter` | ——（不查 TF，只转发戳） | —— | 无感，但**会原样传递上游的 1 ns** |
| `linefit` / `patchwork` | 消息戳（仅当 `gravity_aligned_frame` 非空） | 直接 try/catch，无队列 | 配置为空 ⇒ 无感 |
| `pointcloud_to_laserscan` | 不查（`target_frame: ""`） | —— | 无感 |
| `icp_registration` | `now()` + 10 s 超时（初始化一次性） | —— | 无感（有兜底） |
| `gicp_registration` | 发到未来（`now()+0.45`），查询用"最新可用" | —— | 无感（刻意设计） |
| Gazebo 插件（点云/IMU 发布端） | `get_clock()->now()`（仿真钟整数纳秒） | —— | 无感（它是"真值"的来源） |
| `livox_ros_driver2`（真机） | `rclcpp::Time(uint64 ns)` | —— | 无感（整数域） |
| `spin_speed` / `fake_vel_transform` 直通模式 | `TimePointZero` | —— | 无感（`:96-99` 已在 `spin_speed==0` 时跳过 TF 查询） |
| `spatio_temporal_voxel_layer`（STVL，local costmap 的体素层） | 未验证（apt 包**只装了生成的头文件**，无源码可读） | 参数 `transform_tolerance: 0.2` | **未知**：与 costmap obstacle layer 同族（按消息戳截断+超时）；登记在 §9 |

**两个"看起来像、其实不是"的对照，避免下次误判**：
* 历史日志里 costmap/AMCL 的少量 `Message Filter dropping ... the timestamp on the message is
  earlier than all the data in the transform cache`（例如 `local_costmap` 在仿真时间 0~20 s、
  645 s 附近的若干条）**不是**本 bug：原因是"消息比 TF 缓存里最老的样本还老"（启动段/缓存窗口），
  与 1 ns 无关；本 bug 的原文是 `discarding message because the queue is full`。
* `Minimum time interval throttling`（slam_toolbox 的 `minimum_time_interval=0.5`）是**故意丢**
  （只处理 2 Hz），与戳无关——统计"被处理的扫描"时别把它算成丢帧。

---

## 6. Q3：还有哪些"自己造戳"的点？要不要共用一个 `double_to_msg_time()`？

**清单（本仓 + 子模块，全部"从 double/整数造 (sec, nanosec)"的地方）【事实】**：

| 位置 | 造法 | 安全性 |
|---|---|---|
| `small_point_lio_node.cpp:40-52` | `floor` + `llround` + 进位 | ✅（本仓唯一需要它的地方，已经是局部 static 函数） |
| `FAST_LIO/include/common_lib.h:262-268`、`point_lio/include/common_lib.h:215-221` | `floor` + **隐式截断** | ❌ |
| `teb_local_planner/misc.h:155-165`（`durationFromSec`） | `floor` + `round` + 进位 | ✅（构造 `Duration`，不查 TF） |
| `imu_complementary_filter`（C10） | 整数纳秒相减 → double 秒 | ✅ |
| Gazebo 插件（C11） | `get_clock()->now()`（整数纳秒） | ✅ |
| `livox_ros_driver2`（A6） | `rclcpp::Time(uint64 ns)` | ✅ |
| `lidar_adapter/*.h`（`sec + nanosec*1e-9`） | 整数 → double（进 LIO 内部） | ✅ 只要出口是"四舍五入"（本轮 A3 已保证） |
| `cartographer_ros/time_conversion.cpp` | 整数 ticks | ✅ |

**结论：不建议**为了统一而引入跨包共享头文件（`time_utils/msg_time.hpp` 之类）。理由【推断，但依据充分】：
1. 真正需要它的**只有 2 个包**（`small_point_lio` 已内联 8 行；`FAST_LIO`/`point_lio` 是**子模块**，
   加一个新依赖头比就地改 3 行更难维护、还会在 rebase 上游时冲突）；
2. `teb_local_planner` / `livox_ros_driver2` / `costmap_converter` 都是子模块或上游包，
   引本仓头文件会破坏"每包独立"的构建契约（我们已经在 `bringup` 里为"缺包"付过代价，见
   `docs/lio_slots.md` 的构建期陷阱记录）；
3. 这个 bug 的根因**不是"缺少工具函数"，而是"没人检查"**。所以本轮把预算花在**检查器**上：
   `stamp_delta_ns_report.py`（离线复算）+ 探针的新指标（在线直出）+ §8 的规则与 grep。

如果将来一定要共享，建议只在**同一个包内**共享（例如把 `double_to_msg_time` 从 node.cpp 提到
`small_point_lio/util/time_utils.h`），并配一条"所有 `*1e9` 附近必须有 `llround`"的审查规则。

---

## 7. 本轮改了什么 / 没改什么

### 7.1 改的（全部是工具，零行为变更）

| 文件 | 改动 | 验证 |
|---|---|---|
| `tools/scripts/diag/scan_tf_timing_probe.py` | 新增整数纳秒序列 `scan_ns`/`tf_ob_ns`（落 npz）+ 汇总指标 `stamp_delta_ns_tf_minus_scan` | `py_compile` 通过；在 `ts_spl_postfix`（修复后，`exact_0ns_pct = 100.0`，n=3118）与 `ts_pointlio`（跑通但无 TF 可测）两次真实跑里**端到端跑通**，且与离线复算器结果一致 |
| `tools/scripts/diag/stamp_delta_ns_report.py`（新增） | 离线复算 Δns 分布 + 保守判定 | 在 4 份 npz 上复现：q05 39.99% / fix01 0% / ts_fastlio 40.01% / ts_spl_postfix 0%（与 `docs/slam_toolbox_scan_drops.md` 的 40%/0% 一致） |

两者都**不被任何 launch 启动**（保持"一次性仪器"定位），也不改任何话题/参数。

### 7.2 明确**没有**改的（连同给子模块/导航栈的补丁提案）

**（1）`FAST_LIO` / `point_lio`（子模块，只读）**——补丁提案（3 行 + 进位，语义等价于 `a281636`）：

```cpp
// 以 FAST_LIO/include/common_lib.h:262-268 为例（point_lio/include/common_lib.h:215-221 同改）
rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    const double frac = timestamp - std::floor(timestamp);
    // ★ 原来 uint32_t nanosec = frac * 1e9; 是**截断**：约 40% 的帧会比同一帧点云戳低 1 ns
    //   ⇒ tf2 MessageFilter 在 /scan 戳上查不到同帧 TF ⇒ 等下一帧 ⇒ scan_queue_size=1 时被顶掉。
    //   实测（2026-10-06，lio:=fastlio）：Δ(TF−/scan) = −1 ns 占 40.0%，丢帧 126/3003 = 4.2%。
    int64_t ns = static_cast<int64_t>(std::llround(frac * 1e9));   // 四舍五入
    if (ns >= 1000000000LL) { sec += 1; ns -= 1000000000LL; }      // 进位
    if (ns < 0) { ns = 0; }
    return rclcpp::Time(sec, static_cast<uint32_t>(ns));
}
```

补丁收益（**推断，基于 `a281636` 的同构实测**）：`Δ=−1 ns` 40% → 0%，`slam_toolbox` 丢帧
4.2% → 与 `small_point_lio` 修复后同量级的残余（0.3% 级，且只剩启动段/真空隙）。
**为什么本轮不改**：① 两个都是子模块且**带有他人未提交改动**（`git status` 显示
`M src/laserMapping.cpp`），改它就是碰别人的工作区；② 指令明确要求"子模块只报告"。

**（2）`rm_nav_bringup` 的 `imu_complementary_filter` 缺 `use_sim_time`（C6）**——提案：

```python
# src/rm_nav_bringup/launch/bringup_sim.launch.py:423-433
    parameters=[os.path.join(get_package_share_directory('imu_complementary_filter'),
                             'config', 'imu_filter_params.yaml'),
                {'use_sim_time': use_sim_time}],        # ★ 新增
```
**为什么本轮不改**：它在已验证导航栈的 launch 里（`lio:=fastlio` 的 IMU 源就是它的输出），
而"改动导航栈"的门槛是"证据无歧义 + 行为影响明确"。当前实测影响为 **0**（不读自己的钟、
戳抄输入、`publish_tf=false`），属于"正确的时机未到"而不是"没证据"。建议**下次动 launch 时顺手加**。

**（3）`/Odometry` 的 RELIABLE 背压风险（C2/C4）**——**提案：先观察，不要动 QoS**。
把 LIO 的 `/Odometry` 改成 BEST_EFFORT 会让 nav2 的 RELIABLE 读者**一条都收不到**（QoS 不兼容），
风险远大于收益；真要解，得改 `nav2_util` 的 `OdomSubscriber`（那是 apt/vendored 代码）。

**（4）`slam_toolbox` 的 `scan_queue_size`**——**明确不改**。把它调大只是把"丢帧"换成"延迟处理旧扫描"，
`a281636` 已经从根因上把等待时间打掉了（这也是 `docs/slam_toolbox_scan_drops.md` §8 的结论）。

---

## 8. 如何避免再犯（规则 + 可直接粘的检查）

1. **小数秒 → 整数纳秒，只允许"四舍五入 + 进位"，永远不要截断。**
   反例：`uint32_t nanosec = (ts - floor(ts)) * 1e9;`（A1/A2）；
   正例：`llround(...)` + `if (ns >= 1e9) { sec++; ns -= 1e9; }`（A3/A5）。
2. **别用"自己造的戳"去精确查 TF**；要么用 `tf2::TimePointZero`（最新可用），
   要么显式给出容差。正例：`fake_vel_transform.cpp:88`、costmap `getRobotPose`（`TimePointZero`）。
3. **永远不要假设 `stamp >= 请求戳`。** tf2 的判据是"缓存里得有 ≥ 请求戳的样本"
   （`message_filter.hpp:564`）；同帧样本哪怕低 1 ns 也不合格。
4. **消费者侧的队列深度 ≥ 2**（要等 TF 的地方）。深度 1 = 把"等待"变成"丢弃"（B1/B2）。
5. **查 TF 的地方，`transform_tolerance`/超时必须 ≥ 上游 TF 帧周期。**
   本仓 LIO 帧周期 ~0.1 s（实测 TF 周期 p50 = 100 ms）⇒ costmap 的 0.2/0.3 s 是**下限附近**，
   不要往下调；`slam_toolbox` 的 `transform_timeout=0.2` 同理。
6. **跨流水线对齐的验收指标是"Δ纳秒分布"，不是"看起来能跑"。**
   验收命令（可写进任何 PR 描述）：
   ```bash
   python3 tools/scripts/diag/stamp_delta_ns_report.py --npz <跑次>.timing
   # 期望：Δ=0 占比 ~95%+，Δ=−1 ns 占比 0（出现 ~40% 就是本次这个 bug）
   ```
7. **换 LIO 槽位/换传感器/换驱动时，先跑一次 6 的验收**（三个槽位共用同一套 `/scan` 链，
   所以这条检查对 `fastlio`/`pointlio`/`small_point_lio` 都适用）。
8. **新节点必须显式给 `use_sim_time`**（漏了就是"墙钟节点消费仿真戳"，
   linefit / p2l 都踩过：`bringup_sim.launch.py:435-441,463-475` 的注释）。
9. **加"发到未来/最新"的盖戳（`now()+lookahead`）时，必须写清消费者是在哪个时刻查的**
   （正例：`gicp_registration.cpp:1028-1032` 的注释直接给了 MPPI 的 `transform_tolerance=0.1`）。
10. **提交前跑一遍 P1 grep**（§1.2），把 `* 1e9` 附近的每一处都人工判一次。

---

## 9. 未验证清单（诚实边界）

| 项 | 为什么没验 | 影响 / 下一步 |
|---|---|---|
| **`point_lio` 的实测 Δns 与丢帧数** | 本轮**尝试并失败**（`ts_pointlio`，§4.4）：该槽位在第 100 帧被自己的 `pcd_save` 打死 ⇒ `tf_odom_base = 0`，探针连一个 TF 样本都没有 ⇒ 无法计算 Δ。代码与调用路径已逐行核对 | 【推断】同一 bug（代码等价 + `publish_odometry_without_downsample: false`）。**下一步**（二选一，都要先解 §4.4 的写盘问题）：<br>① 全栈：`mkdir -p src/rm_localization/point_lio/PCD` 后再跑 `mapping_ab_run.sh --lio pointlio --timing-probe`；<br>② 节点级（推荐，不碰子模块文件）：用 `tools/lio_node_alone_check.py` 重放 `.tmp_bags/ret4`，只起 `pointlio_mapping` 并覆盖 `pcd_save_en:=false`，再对 `/Odometry`(或 `camera_init→aft_mapped`) 与输入云 header 戳做同样的 Δns 统计 |
| **costmap 因 1 ns 多等的 0.1 s 是否可观测** | 需要 nav 模式（`mode:=nav`）+ costmap 侧打点；本轮只做了 mapping 模式 | 【推断】按 `MessageFilter(queue=50)` + `ObservationBuffer` 的按戳查询机理推导；可证伪点：costmap 的 `updateMap` 周期分布（若出现 ~0.1 s 的台阶即证实） |
| **`/Odometry` RELIABLE 背压是否真的发生过** | 需要 nav 模式 + 控制卡顿复现；本轮只做了 live QoS 查询（C2/C4） | 历史事故（2026-09-24 插件 RELIABLE）是同机理的**前例**，但不是同一个话题 |
| **`third_party/**`（nav2 全量、fast_lio/point_lio 上游、cartographer 上游）** | 超出本轮范围（指令只覆盖我们的包 + 实际在跑的分支）；只在"被本仓调用到的行为"上引用了 nav2/ tf2 的具体代码行 | 若将来 fork nav2，需重跑同一套审计 |
| **`src/rm_localization/{cartographer, cartographer_ros}` 的深度审计** | 不在指令给的包列表里；只核了时间转换（A7） | cartographer 在本仓是 `lio:=cartographer` / `mapper:=cartographer` 槽位，若启用需补审 |
| **`livox_ros_driver2` 在真机上 `base_time` 是否为绝对时间** | 需要真机/录包；本仓只读代码（A6） | 若 `base_time` 是设备上电计时（非 epoch），`/livox/lidar` 与 `/livox/imu` 会不在同一时间轴（那是 `timebase=0` 那次事故的邻居） |
| **`rviz` 配置与 `use_sim_time`（`bringup_sim.launch.py` 里 3 个 rviz2 节点未显式设）** | RViz 不产生数据戳，只影响显示 | 纯显示问题，不列入 bug |
| **`spatio_temporal_voxel_layer`（STVL）的过滤器/队列/QoS** | `/opt/ros/humble` 里只有它生成的 msg/srv 头文件，**没有 `.cpp` 源码**；本仓配置只暴露 `transform_tolerance: 0.2` | 【推断】与 `nav2_costmap_2d` obstacle layer 同族（按消息戳查 TF）；若它用队列 1 就会像 slam_toolbox 一样丢帧。可证伪点：nav 模式日志里 STVL 侧的 `Message Filter dropping` |
| **`ros2 topic info -v` 的 QoS 快照只覆盖 `lio:=fastlio`（mapping 模式）** | 时间预算；nav 模式另有 costmap/AMCL/controller 参与 | 表 §3.3 的"实测"仅限该跑次；nav 模式的成本 QoS 未逐条实测（但代码行已引） |

---

## 10. 回滚

本轮**只改了 2 个工具脚本**，没有触碰任何被验证过的运行链路（导航栈、LIO、launch、参数）。

```bash
# 只回滚工具（若需要）
git checkout -- tools/scripts/diag/scan_tf_timing_probe.py
rm -f tools/scripts/diag/stamp_delta_ns_report.py
# 若已经 commit：
git revert <本轮 commit hash>        # 工具文件的增改，无运行时依赖
```

**注意**：`FAST_LIO` / `point_lio` 的 bug 本轮**没有修**（子模块，只给补丁）。
它们的回滚不存在（未改动）；但**只要它们还被用作 `lio:=` 槽位，1 ns 丢帧就还在**——
判定依据：§4.2 的 `ts_fastlio` 实测。

---

## 附录 A：证据文件与复现命令

| 证据 | 路径 |
|---|---|
| fastlio 跑（本轮主证据） | `.tmp_tsaudit/fastlio/ts_fastlio.{log,summary.json,timing.json,timing.npz,drops.txt,delta.json}` |
| pointlio 跑（本轮，**槽位在第 100 帧崩**⇒无 TF 可测） | `.tmp_tsaudit/pointlio/ts_pointlio.*` |
| **同路线修复后对照**（本轮） | `.tmp_tsaudit/spl_postfix/ts_spl_postfix.{log,summary.json,timing.json,timing.npz,drops.txt}` |
| 历史 A/B（`a281636` 的验证） | `.tmp_cache/slamq/{q01,q01b,q05,q10,fix01}/`、`docs/slam_toolbox_scan_drops.md` §8/§9.4 |
| 单元级复现 | `.tmp_tsaudit/repro_trunc.cpp`、`.tmp_tsaudit/stamps_ns.txt` |
| Δns 复算 | `python3 tools/scripts/diag/stamp_delta_ns_report.py --npz <prefix>.timing` |

复现本轮 fastlio 测量（一条 bash 调用，隔离 HOME/domain/port，无 RViz，`spin_speed=0`）：

```bash
bash tools/scripts/mapping/mapping_ab_run.sh --tag ts_fastlio --lio fastlio --timing-probe \
  --route .tmp_cache/spl2026/run4/spl4.route.json --out-dir .tmp_tsaudit/fastlio \
  --domain 111 --port 11561 --speed 0.20 --timeout 300 --warmup 15
python3 tools/scripts/diag/stamp_delta_ns_report.py --npz .tmp_tsaudit/fastlio/ts_fastlio.timing
```

单元级复现：

```bash
cd .tmp_tsaudit && g++ -O2 -o repro_trunc repro_trunc.cpp && ./repro_trunc < stamps_ns.txt
# N=3223  截断版 < 真值: 1289 (39.99%)   四舍五入版 < 真值: 0 (0.00%)
```

## 附录 C：本轮测量的环境版本（可追溯性）

| 项 | 值 |
|---|---|
| 仓库 HEAD（跑测时） | `a281636`（`small_point_lio` 的截断修复已入库；工作树里 `small_point_lio` 与 HEAD 逐字节一致） |
| 三次跑的 launch 启动时刻 | `ts_fastlio` 14:30:17、`ts_pointlio` 14:37:55、`ts_spl_postfix` 14:46:48（2026-10-06） |
| launch 文件状态 | 上述三次跑用的都是**同一份** `bringup_sim.launch.py`；该文件在 **14:53:25** 之后被**其他 agent** 的未提交改动改写（「续建/存档守卫」功能，与本审计无关）⇒ 本审计的结论不受其影响，但**再跑一次会得到不同的栈组成**，追溯时请以 commit `5410412`/本轮 doc commit 为界 |
| ab 脚本 / 路线 | `tools/scripts/mapping/mapping_ab_run.sh` + `.tmp_cache/spl2026/run4/spl4.route.json`（三次跑完全相同） |
| 隔离 | 非默认 `ROS_DOMAIN_ID`（111/112/113）、独立 `GAZEBO_MASTER_URI` 端口（11561/11562/11563）、`HOME=/tmp/gzhome-<tag>`、`unset DISPLAY`、`nav_rviz:=False`、`spin_speed:=0.0` |
| RTF | `ts_fastlio` p50 0.747；`ts_spl_postfix` p50 0.768（同一台机、同一 world） |

## 附录 B：事实 / 推断 分界（一页速查）

**事实（可直接复核）**
1. `FAST_LIO`/`point_lio` 的 `get_ros_time` 用隐式 `uint32_t` 截断（A1/A2，代码原文）。
2. 截断在 3223 条真实仿真戳上产生 39.99% 的 −1 ns（§2.3，可重跑）。
3. `lio:=fastlio` 实测 Δ(TF−`/scan`)：Δ=0 60.0%、Δ=−1 ns 40.0%；丢帧 126/3003 = 4.20%，2.5 s 节律（§4.2）。
4. 修复后的 `small_point_lio`：Δ=−1 ns 0%、Δ=0 95.5%、丢帧 10（0.35%）（历史 `fix01`）。
5. tf2 `MessageFilter` 在**消息自己的戳**上 `canTransform`，且队列满时"新消息顶掉最老的"（B2，系统头原文）。
6. 各 topic 的两端 QoS（BEST_EFFORT/RELIABLE、深度）为 live 实测（§3.3）。
7. 除 A1/A2/A3 外，全仓无其它截断式构造（§3.1，grep 命令可复跑）。
8. `gazebo_ros::Node` 强制 `use_sim_time=true`；插件戳=仿真钟整数纳秒（C11）。
9. 修复后的 `small_point_lio` 在同一条路线上：Δ=0 **100.0%**（n=3118）、丢帧 **0**（`ts_spl_postfix`）。
10. `lio:=pointlio` 在仿真里第 100 帧 SIGABRT（`pcd::IOException`，`PCD/` 目录不存在）⇒ `/odom` 0 条、`/map` 0 增长（§4.4）。
9. `imu_complementary_filter` 节点缺 `use_sim_time`（C6，launch 原文）。

**推断（未直接测量，给出证伪方式）**
1. `point_lio` 有同一 bug（代码等价；证伪：跑一次 §8 第 6 条）。
2. costmap 在修复前每次观测多等 ~0.1 s（证伪：nav 模式下打点 `updateMap` 周期）。
3. `FAST_LIO` 补丁后丢帧会降到 0.3% 量级（同构于 `small_point_lio` 的 4.2%→0.35%）。
4. `/Odometry` 的 RELIABLE 写者存在被 nav2 RELIABLE 读者拖慢的风险（未观测到）。
