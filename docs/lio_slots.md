# 里程计槽位补充：`lio:=small_point_lio`（vendored `Yancey2023/small_point_lio`）

> 2026-10-05 · 新增 `bringup_sim.launch.py` 的 `lio` 取值 `small_point_lio`（默认值仍是 `fastlio`，零影响）
> 2026-10-05（同日稍后）· **调参完成**：该槽位从"跟不住（30 m 级、不可重复）"调到
> "与 FAST-LIO 参照同量级（长度 0.95~1.15×、形状误差 ATE 0.24~0.42 m）"，新参数文件
> `config/mid360_sim_tuned.yaml`（只改 4 个键）。**新增内容集中在 §0.7 与 §5.5~§5.8**；
> 上游默认参数档（`config/mid360_sim.yaml`）**原样保留**作为对照基线。
> **与 `docs/localization_slots.md` 的分工**：那篇是**重定位槽**（谁发 `map→odom`）的登记表；
> 本文是**里程计槽**（谁发 `odom→base_link`）新增取值的登记。之所以另开一篇：`localization_slots.md`
> 正在被"五路重定位对比"那条线占用，避免并发改同一文件。
> 契约总纲仍以 `docs/tf_interface_contract.md` 与 `docs/architecture.md` 为准。

---

## 0. 一句话结论（先看这个）

1. **上游确权完成**：<https://github.com/Yancey2023/small_point_lio>，**MIT**（`LICENSE.txt` + GitHub API
   `license.spdx_id = MIT` + `package.xml <license>MIT</license>` 三者一致），pin commit
   **`688d75cfa780049ae532e5100ca64f46ad8b1a93`**（分支 `ros2`，2026-08-31 "fix bugs"；**上游没有 tag**）。
   MIT 允许 vendored 再分发 ⇒ **已落地** `src/rm_localization/small_point_lio/`（不是 `third_party/`，
   那里有 `COLCON_IGNORE`），并登记进 `THIRD_PARTY_NOTICES.md`。
2. **输入契约不需要"适配器节点"**：上游原生支持 `lidar_type: livox_custom_msg`，吃的正是我们仿真
   `/livox/lidar` 那路 **`livox_ros_driver2/msg/CustomMsg`**。之前担心的 `tag`/`timestamp` 字段门槛
   **只存在于它的另一条路（`livox_pointcloud2` 适配器）**，我们根本不用走那条路。
   唯一的真门槛是 **`timebase`**（见第 3 条）。
3. **真正的坑：逐点时间戳**。它的 CustomMsg 适配器把点时间算成 `(timebase + offset_time) / 1e9`，
   而**我们的仿真插件从不填 `timebase`**（保持默认 0）、`offset_time` 也恒 0（帧内无运动）
   ⇒ 点时间全变成 `0.0 s`，而 IMU 时间是**仿真钟**（跑起来就是几百秒）。
   后果不是"精度差"而是**静默失效**：一条 `/Odometry`、一条 `odom→base_link` 都不发（实测见 §5）。
   ⇒ 在 vendored 副本里打了 **3 行补丁**：`timebase == 0` 时退回 `header.stamp`
   （真机 `livox_ros_driver2` 本来就是 `timebase ≡ header.stamp`，所以这是"补齐鲁棒性"，不是改语义）。
4. **TF 契约天然吻合，但要关掉适配器**：它**自己就发 `odom→base_link`**（帧名硬编码，`small_point_lio_node.cpp:61-62`）
   ⇒ 必须**排除 `lio_tf_adapter`**（否则 odom 双父边），`map→odom` 仍只由重定位槽发。
   它**强依赖** `base_link→livox_frame` 静态 TF（查不到就丢帧、连 odom 都不发）——仿真里由
   `robot_state_publisher` 从 URDF 提供（实测 `/tf_static` 有 `base_link→livox_frame (0.12,0,0.175)`）。
5. **节点级验证通过（链路层）**：用**真实 bag**（`.tmp_bags/ret4`，RMUL2026 场地的真实回波）重放
   IMU 78.5 Hz + 点云 7.85 Hz，节点稳定输出 `/Odometry` **7.3 Hz** + TF `odom→base_link` 294 条 +
   `/cloud_registered` 293 帧，**零 ERROR**（§5）。
6. ⚠️ **但"链路通"不等于"能用"**：同一次重放里它的轨迹**跟不住**（轨迹长度 36.6 m，
   而同一窗口的真值 5.5 m / 那次跑的 FAST-LIO 录制输出 6.2 m）。**同一喂法**下 `fast_lio` 把录制参照
   复现到**末位置差 4 cm**（§5.4）⇒ 喂法可信，是**该算法在我们的仿真数据上（上游默认参数）没跟住**。
   ⇒ 用户在真跑前**必须先看 `/odom` 与真值/基线的偏差**，不要直接信它跑 nav（§7 的检查项）。
7. ✅ **【2026-10-05 调参结论，直接看 §5.5~§5.8】已经被调住了**：判明是**滤波器参数**问题，不是算法不能用。
   只改 4 个键（`space_downsample: false`、`imu_meas_omg_cov: 0.01→0.2`、`velocity_cov: 20→0.3`、
   `acceleration_cov: 500→50`）后，**同一条 bag、同一个 40 s 窗口**：

   | 来源 | 轨迹长度 | 末位置 | 与录制 FAST-LIO 末位置之差 | 形状误差 ATE |
   |---|---|---|---|---|
   | 真值 `/odom_ground_truth` | 5.51 m | (3.13, 7.66)（世界系，不可直接比） | — | — |
   | `fast_lio` 重放（参照） | 7.06 m | (-1.11, 4.36) | **0.04 m**（自证喂法忠实） | 0.59 m |
   | `small_point_lio` 上游默认参数 | 30.20 m（**10 次重复落在 30.1~46.4**） | (0.88, -6.07) | 10.64 m | 1.82 m |
   | **`small_point_lio` + `mid360_sim_tuned.yaml`** | **6.90 m**（重复 6.88/6.90/6.92） | **(-1.21, 4.29)** | **0.10 m** | **0.42 m** |

   ⇒ **轨迹长度 = FAST-LIO 参照的 0.98 倍、末位置差 10 cm、形状误差还比 FAST-LIO 小**
   （0.42 vs 0.59 m）。另在 bag 的**另外两个时间窗**（40~70 s、90~120 s）独立复测，
   长度分别是参照的 **1.09× / 1.15×**、ATE **0.30 / 0.24 m**（FAST-LIO 那两个窗口是 0.37 / 0.38 m）
   ⇒ **不是只对第一个窗口过拟合**。
   新参数文件 = `src/rm_localization/small_point_lio/config/mid360_sim_tuned.yaml`，
   `bringup_sim.launch.py` 的 `lio:=small_point_lio` 分支已改读它（回退只需改回一行，§8）。
   ⚠️ 但**全栈闭环仍未验证**（本任务禁止起 Gazebo/nav2）——见 §9。

---

## 1. 上游确权（附核实方式）

| 项 | 值 | 核实方式 |
|---|---|---|
| 上游 | <https://github.com/Yancey2023/small_point_lio>（作者 **Yingjie Huang**，东莞理工学院 ACE 战队；RM26 开源贴 <https://bbs.robomaster.com/article/813022>） | GitHub REST API `repos/Yancey2023/small_point_lio` |
| 许可证 | **MIT** —— `LICENSE.txt`："The MIT License (MIT) / Copyright (c) 2025 Yingjie Huang" | API `license.spdx_id = MIT`；`package.xml:8 <license>MIT</license>`；三者一致（**没有** §四 那类"package.xml 与 LICENSE 打架"的问题） |
| 默认分支 / ROS 2 | `ros2` 分支（`main` 也在）；`ament_cmake` + `rclcpp_components`；CI 有 `humble.yml`、`jazzy.yml`、`humble_with_livox_driver.yml` | API `branches`、仓库 `.github/workflows/` |
| **本仓 pin** | commit **`688d75cfa780049ae532e5100ca64f46ad8b1a93`**（`ros2` HEAD，2026-08-31 "fix bugs"）；上一提交 `3edd799` "update default param" | `git clone` 后 `git rev-parse HEAD`；API `commits?sha=ros2` |
| 上游 tag | **无**（`api.github.com/.../tags` 返回 `[]`）⇒ 只能 pin commit | API |
| 仓库活跃度 | 创建 2025-10-09，121 ★ / 22 fork，最后 push 2026-08-31 | API |
| 依赖 | **Eigen3 + OpenMP + rclcpp/rclcpp_components/rclpy + sensor_msgs/geometry_msgs/nav_msgs + tf2_ros/tf2_geometry_msgs + std_srvs**；**可选** `livox_ros_driver2`（`find_package(... QUIET)`，找到就定义 `HAVE_LIVOX_DRIVER`）；**没有 PCL、没有 GTSAM、没有 Ceres、没有 TBB**（`400097a5` "remove pcl dependency"；`target_link_libraries` 里那个 `${PCL_LIBRARIES}` 是空变量） | 上游 `CMakeLists.txt`、`package.xml` |
| 构建口径 | C++20、`-march=native -ffast-math -fno-math-errno`、OpenMP、PCH；`CMAKE_BUILD_TYPE` 未设时自设 `Release` | 上游 `CMakeLists.txt` |
| 算法 | ESKF + **iVox** 增量体素图、逐点平面更新（最近 5 点、协方差最小特征向量当法向），**无 ikd-Tree、无 scan-to-map ICP、无回环、无 PGO** | `estimator.cpp`、`small_ivox.h`（详见 `docs/cod_nav_2026_deep_dive.md` §A3） |
| 我们的构建实测 | `HAVE_LIVOX_DRIVER` **已定义**（本仓 `src/rm_driver/livox_ros_driver2` 在 AMENT_PREFIX_PATH 里）；编译旗标 `-O3 -DNDEBUG -march=native -ffast-math -fopenmp -std=gnu++20` | `log/latest_build/small_point_lio/stdout_stderr.log`、`build/small_point_lio/compile_commands.json` |

clone 命令（可复现）：

```bash
git clone https://github.com/Yancey2023/small_point_lio.git /tmp/spl
git -C /tmp/spl checkout 688d75cfa780049ae532e5100ca64f46ad8b1a93
rsync -a --exclude='.git' /tmp/spl/ src/rm_localization/small_point_lio/   # 与 third_party/small_gicp 同一形态：去 .git
```

---

## 2. 输入/输出契约：上游 vs **我们的仿真实际发的东西**

### 2.1 契约表（源码逐条核对；行号 = pinned commit）

| 方向 | 项 | 上游 `small_point_lio` | 我们仿真**实际**发布 | 匹配？ |
|---|---|---|---|---|
| 入 | 点云话题 | 参数 `lidar_topic`，默认 `/livox/lidar` | `/livox/lidar`（`ros2_livox_simulation` 的 `libros2_livox.so`，`livox_points_plugin.cpp:95-96`） | ✅ |
| 入 | 点云类型 | 由 `lidar_type` 决定；`livox_custom_msg` → `livox_ros_driver2/msg/CustomMsg` | **CustomMsg** | ✅ 走这条路 |
| 入 | 点云 QoS | `rclcpp::SensorDataQoS()`（**BEST_EFFORT**，`livox_custom_msg.h:24`） | **BEST_EFFORT**（2026-09-24 修复后，`livox_points_plugin.cpp:95`） | ✅ |
| 入 | 点云 `frame_id` | **不读**（只用 xyz + tag + 时间）；雷达系名走参数 `lidar_frame` | `livox_frame`（`raySensor->Name()`，`livox_points_plugin.cpp:190`） | ✅（`lidar_frame: livox_frame`） |
| 入 | 点云 `tag` | 只保留 `(tag & 0b00111111) == 0` 的点（`livox_custom_msg.h:30`） | 插件**不赋值** ⇒ 恒 0 ⇒ **全部保留** | ✅ |
| 入 | 点云 `offset_time` | 逐点时间 = `(timebase + offset_time)/1e9`（秒） | 插件恒 **0**（帧内无运动，`livox_points_plugin.cpp:267`，有注释说明） | ✅ 语义正确（仿真帧内确实无运动） |
| 入 | 点云 **`timebase`** | 作为**绝对时间基**（ns） | 插件**从不赋值** ⇒ 恒 **0** ⚠️ | ❌ **本仓补丁修掉了**（见 §3.2） |
| 入 | IMU 话题/类型 | 参数 `imu_topic`，`sensor_msgs/msg/Imu` | `/livox/imu`，`sensor_msgs/msg/Imu`（`gazebo_ros_imu_sensor`，`simulation_waking_robot.xacro:239-251`） | ✅ |
| 入 | IMU QoS | `SensorDataQoS()`（BEST_EFFORT，`small_point_lio_node.cpp:201-203`） | 插件侧默认 QoS（RELIABLE）⇒ BEST_EFFORT 读者兼容 | ✅ |
| 入 | IMU `frame_id` | **不读** | `imu_link` | ✅ |
| 入 | IMU `header.stamp` | `sec + nanosec*1e-9`（`small_point_lio_node.cpp:208`） | 仿真钟（实测 bag 里 sec=655） | ✅ |
| 入 | IMU 频率 | 不限（按时间戳） | 实测 **78.4 Hz**（xacro 写 100 Hz，RTF<1 时实际 78） | ✅ |
| 入 | **`base_link→livox_frame` TF** | **强依赖**：`lookupTransform(lidar_frame, "base_link", t)` 失败就 `return`（`small_point_lio_node.cpp:64-69`）——**丢帧、不发 odom** | URDF 固定关节 → `robot_state_publisher` → `/tf_static`：实测 `(0.12, 0, 0.175)` | ✅（前提是 rsp 起了） |
| 出 | 里程计 | `/Odometry`，`nav_msgs/Odometry`，`frame_id=odom`、`child_frame_id=base_link`（**硬编码**） | 期望 `/odom` ⇒ launch 里 `remappings=[('/Odometry','/odom')]` | ✅（靠 remap） |
| 出 | 点云 | `/cloud_registered`（`frame_id=odom`）；**只有存在订阅者时才发**（`get_subscription_count()>0`，`small_point_lio_node.cpp:102`） | — | ℹ️ 无订阅者时"静默不发"是正常的 |
| 出 | TF | **`odom→base_link`**（`small_point_lio_node.cpp:61-62, 98`） | — | ✅ **正好是我们的契约** |
| 出 | 其它 | `map_save` 服务（`std_srvs/Trigger`，仅在 `save_pcd: true` 时有意义）、`~` 无 | — | ℹ️ |
| 出 | `twist` | **六行全是注释**（`small_point_lio_node.cpp:90-96`）⇒ `/odom.twist` 恒 0 | — | ⚠️ 任何依赖 `/odom.twist` 的模块读到 0 |

### 2.2 "我们实际发什么"的证据（不是推测）

- **源码**：`src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`
  （`p.offset_time = 0` 在 :267；`timebase` 全文件只在 :127 的一句日志字符串里出现 ⇒ **确实没赋值**）。
- **实录音 bag**（`.tmp_bags/ret4`，sqlite3 + `deserialize_message` 直接读）：
  - `/livox/lidar/pointcloud` = `sensor_msgs/PointCloud2`，`frame_id=livox_frame`，字段只有
    **`x,y,z,rgb`**（`point_step=32`）⇒ **没有 `tag`、没有 `timestamp`** ⇒ 上游那条
    `livox_pointcloud2` 适配器**确实喂不进去**（它的回答是：**不用那条路**）。
  - `/livox/imu` = `sensor_msgs/Imu`，`frame_id=imu_link`，`stamp=(655, 0.103)`，`acc≈(0.03,2.28,9.59)`
    首两帧有噪声、之后恒定 `(0, 0.003, 9.800)`；**没有启动零值帧**。
  - `/tf_static`：`base_link→imu_link (0.12,0,0.125)`、`base_link→livox_frame (0.12,0,0.175)`（rpy 全 0）。
  - 实测速率：IMU **78.41 Hz**、点云 **7.85 Hz**（`update_rate 10` 在 RTF<1 时就是这个数）。

> **结论**：**不需要 topic remap 之外的任何适配器节点，不需要 frame_id 改写，也不需要 relay。**
> 唯一必须做的是把 `timebase` 补上/兜住（§3.2），而这件事在**节点内部**做掉了，没有增加进程与 DDS 流量。

---

## 3. 我们对上游改了什么（逐条，全部登记在 `THIRD_PARTY_NOTICES.md`）

### 3.1 新增 `config/mid360_sim.yaml`（**不覆盖**上游 `config/mid360.yaml`）

上游那份是**真机 Mid360 + livox_ros_driver2** 的；仿真必须改 4 个键（其余原样搬），每条都在文件里写了理由：

| 键 | 上游 | 我们 | 为什么 |
|---|---|---|---|
| `acc_norm` | `1.0` | **`9.81`** | `imu_acceleration_scale = |gravity| / acc_norm`（`estimator.cpp:113`）。上游按"IMU 输出单位 = g"调；**Gazebo 的 IMU 是 m/s²**（实测 z≈9.6）⇒ 照抄会让加速度被乘 9.81 倍。与 `pointlio_mid360_sim.yaml` 的 `acc_norm: 9.81` 同口径 |
| `satu_acc` | `3.0` | **`30.0`** | `check_satu` 比较的是**原始加速度**。3.0 是"3 g"的写法，而我们的 z 轴恒 ≈9.6 m/s² ⇒ 照抄会**每一帧都把 z 轴判饱和、残差清零**。换算成 3 g ≈ 29.4，取 30.0 |
| `extrinsic_T` | `[-0.011,-0.02329,0.04412]`（真机内参） | **`[0,0,0.05]`** | URDF：`livox_frame = base_link+(0.12,0,0.175)`、`imu_link = +(0.12,0,0.125)`、rpy 全 0 ⇒ 雷达比 IMU 高 0.05 m。与 fastlio/pointlio 的 `extrinsic_T` 一致 |
| `imu_topic` | `/livox/imu`（同） | `/livox/imu`（**不改**） | 注意：FAST-LIO 用 `/imu/data`（互补滤波后），point_lio 与**本槽位**用原始 `/livox/imu`。将来若要 A/B 可改成 `/imu/data` |
| `save_pcd` | `false`（同） | **保持 `false`，并写进注释禁止打开** | 打开后 `map_save` 会把 PCD 写到 `ROOT_DIR/pcd/scan.pcd`，而 `ROOT_DIR = @CMAKE_CURRENT_SOURCE_DIR@`（`include/param_deliver.h.in`）⇒ **直接写进源码树** |
| 其余 | — | 原样 | `map_resolution 0.2` / `space_downsample_leaf_size 0.2` / `point_filter_num 1` / `min_distance 0.5` / `max_distance 1000` / 滤波器 R、Q 全部保持上游值（它们是调参入口，见 §6） |

### 3.2 补丁：`src/lidar_adapter/livox_custom_msg.h`（**唯一**的源码改动，3 行 + 注释）

```cpp
// 原版：new_point.timestamp = static_cast<double>(msg.timebase + point.offset_time) / 1e9;
const double base_time = (msg.timebase != 0)
        ? static_cast<double>(msg.timebase) * 1e-9                       // 真机路径：行为完全不变
        : static_cast<double>(msg.header.stamp.sec) +
          static_cast<double>(msg.header.stamp.nanosec) * 1e-9;          // 兜底：timebase 缺失时用 header.stamp
new_point.timestamp = base_time + static_cast<double>(point.offset_time) * 1e-9;
```

- **为什么这不是"改语义"**：真机驱动 `lddc.cpp:326-331` 里
  `livox_msg.timebase = pkg.base_time;` 与 `livox_msg.header.stamp = rclcpp::Time(timestamp);`
  **取的是同一个值** ⇒ `timebase ≡ header.stamp`（ns）。我们只是把这条不变量在缺失时补上。
- **为什么必须改**：仿真插件 timebase 恒 0 ⇒ 点时间 0.0 s、IMU 时间 = 仿真钟（几百秒）
  ⇒ `small_point_lio.cpp:103` 的 `if (point.timestamp < time_current) { pop; continue; }`
  把**每一帧点云逐点丢弃**，且 `:84` 的 `is_publish_odometry` 恒假 ⇒ **静默零输出**（§5.3 实测）。
- 文件内以 `[HzMi 本地补丁 2026-10-05]` 标注，未删除/未修改上游 LICENSE 与版权头。

### 3.3 上游**推荐但本仓未做**的一处（登记为待办，不是遗漏）

更"治本"的修法是在仿真侧把字段补齐——`livox_points_plugin.cpp` 每帧加一句
`pp_livox.timebase = static_cast<uint64_t>(stamp.nanoseconds());`（与真机驱动一致）。
**本次故意没动它**：那条链路上正跑着"五路重定位对比"的实测，而 `ros2_livox_simulation` 是全链路
共享的感知源，为一个新槽位去改它会把在跑的实验引入变量；且改完我**无法**在本次任务里用 Gazebo 验证。
补丁放在**新包内部**，影响面被限制在 `lio:=small_point_lio` 这一个槽位。
（对现有链路而言那句也是无副作用的：FAST-LIO / Point-LIO 只读 `offset_time`，不读 `timebase`。）

---

## 4. 接线：`bringup_sim.launch.py` 改了什么

| 位置 | 改动 | 理由 |
|---|---|---|
| `lio` 参数 `choices` | `['fastlio','pointlio','none','cartographer']` → **加入 `'small_point_lio'`**（**默认值仍是 `fastlio`**） | 新槽位；描述里写明"自己直发 odom→base_link，不经 adapter" |
| `bringup_LIO_group` | 新增 `GroupAction(condition=LaunchConfigurationEquals('lio','small_point_lio'))`：起 `small_point_lio/small_point_lio_node`（`name='small_point_lio'`，这是参数文件键名，**不能改**），`parameters=[.../config/mid360_sim.yaml, {'use_sim_time': use_sim_time}]`，`remappings=[('/Odometry','/odom')]` | 与 pointlio 分支同构 |
| `lio_tf_adapter_node` 条件 | 追加 `and lio != 'small_point_lio'` | 它自己发 `odom→base_link` ⇒ 再起 adapter 就是**双父边** |
| `icp_frame_bridge_condition`（`camera_init→map` + `body→odom` 两条静态桥） | 追加 `and lio != 'small_point_lio'` | 本槽位**不存在** `camera_init`/`body` 这两个帧，发出去就是孤立岛；且 `map→odom` 会缺 |
| 新增 `tf_bridge_spl_map_to_odom_node` | 仅 `mode:=nav and localization:='' and lio:='small_point_lio'` 时发**一条** `map→odom` 恒等静态桥 | 保留"`localization` 留空 = 把 LIO 当绝对定位"的回退语义（它对 `odom` 就是世界系） |
| RViz | 复用 `rviz/pointlio.rviz`（`Fixed Frame: odom`，TF/Odometry/Path/PointCloud2 四类显示都在） | 不值得为它多维护一份 300 行 `.rviz` |
| **未动** | `nav2_params_sim_beluga.yaml`、beluga launch、重定位槽、mapper 槽、`nav2` 参数 | 任务要求不打扰 |

> ℹ️ **重定位槽照旧可用**（`amcl/beluga/slam_toolbox/icp/gicp/cartographer`）：`lio` 与 `localization`
> 是两根独立的轴，本槽位只换"谁发 `odom→base_link`"。

---

## 5. 验证：**只跑节点，不起 Gazebo、不起 nav2**

工具：`tools/lio_node_alone_check.py`（本次新增，可复用）：从**已录好的 bag** 流式重放
`/livox/imu`(100 Hz 档) + `/livox/lidar/pointcloud` → 转成 **CustomMsg** 喂给 LIO；
自己补 `/tf_static`（`base_link→livox_frame`/`imu_link`）；同时订阅 `/odom`、`/tf`、
`/cloud_registered` 并做 `tf2` 查询；最后打一张判读表。
**用真实回波而不是人造几何**，这样"喂得进去"的结论才对我们的世界有效。

```bash
# 终端 1（只起 LIO 节点）
source /opt/ros/humble/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=87 ROS_LOG_DIR=$PWD/.tmp_roslog/spl \
ros2 run small_point_lio small_point_lio_node --ros-args \
  --params-file install/small_point_lio/share/small_point_lio/config/mid360_sim.yaml \
  -p use_sim_time:=false -p save_pcd:=false
# 终端 2（喂 + 判读）
ROS_DOMAIN_ID=87 python3 tools/lio_node_alone_check.py --duration 40
```

### 5.1 结果（补丁版，喂法 = 仿真插件契约 `timebase=0`）

| 观测项 | 实测 |
|---|---|
| 输入 | IMU **3141** 条（78.5 Hz）/ CustomMsg **314** 帧（7.85 Hz）/ 共 2,361,939 点（**7522 点/帧**） |
| `/Odometry` | **294** 条（**7.3 Hz**），`frame_id=odom`，`child_frame_id=base_link` |
| TF `odom→base_link` | **294** 条（7.3 Hz）；tf2 查询成功；`/tf` 里**只有这一条边** |
| LIO 有没有发 `map→odom` | **没有** ✅（符合契约：`map→odom` 只归重定位槽） |
| `/cloud_registered` | **293** 帧（末帧 8822 点）—— 注意它**只在有订阅者时**才发 |
| 节点日志 | **零 ERROR / 零 WARN**（启动无报错、无 TF 查询失败） |
| 稳定性 | 40 s 连续运行不掉线、不崩、速率稳定（无"跑一会儿停更"） |

### 5.2 ⚠️ 精度：链路通 ≠ 跟得住（**必须看这一条**）

同一窗口（bag 前 40 s）的**与坐标原点无关**的轨迹长度对比：

| 来源 | 轨迹长度 | 末位置 |
|---|---|---|
| 真值 `/odom_ground_truth`（bag 内） | **5.51 m** | (3.13, 7.66) |
| 那次跑的 FAST-LIO 录制输出 `/odom`（bag 内） | **6.22 m** | (-1.15, 4.37) |
| **本次重放的 `small_point_lio`** | **36.55 m** | **(-4.48, -7.33)** |
| 试调粗体素（`map_resolution=0.5`、`leaf=0.5`、`min_distance=0.3`） | 47.36 m（更差） | (-7.08, 1.16) |

> 📌 2026-10-05 复现补充：同一条命令（§5 的两终端口径）今天复现出来是 **30.20 m / (0.88, -6.07)**，
> 而且**同一配置在同一个窗口上重复 10 次，落在 30.13~46.43 m**
> （30.20×5、30.13 / 30.25 / 30.67 / 43.14 / 46.43 各 1 次；另有两次从 40 s 起播的窗口是 28.92 / 30.40 m，
> 详见 §5.8c）。⇒ "上游默认参数"这一档不只是差，而是**落在发散边界上、结果不可重复**；
> 36.55 m 与 30.20 m 是同一个病，不是两个不同的病。

### 5.3 失败模式 A/B（证明 §3.2 那个补丁是必需的，且是**静默**的）

| 场景 | 点时间相对 IMU | `/Odometry` | TF | `/cloud_registered` |
|---|---|---|---|---|
| 补丁后，`timebase=0` → `header.stamp`（= 我们的契约） | 同轴 | **294 条** | **294 条** | 293 |
| `--lidar-time-lag -650`（= 上游原版遇 `timebase=0` 的**等价算术**：点 0.0 s vs IMU 650 s） | 落后 650 s | **0 条** | **0 条** | 0（**且节点日志零 ERROR**） |
| `--lidar-time-lag +650`（对照：点时间跑到未来） | 超前 650 s | 1778 条 **@71 Hz 但位姿恒 (0,0,0)** | 1777 条恒 0 | 0 |

⇒ 上游原版在**不改**的情况下的表现就是第 2 行：**输入在流、节点在跑、日志干净、什么都没有**。
这也解释了为什么这类问题在全栈里极难定位（会被误判成"LIO 没启动"或"TF 没接上"）。

### 5.4 喂法可信度对照（**为什么上面第 3 行能判成"算法没跟住"而不是"喂法不对"**）

把**同一份重放**喂给本仓已长期使用的 `fast_lio`（`common.imu_topic:=/livox/imu`，
`--imu-qos reliable` —— 注意 **FAST-LIO 订 IMU 用的是 RELIABLE**，用 BEST_EFFORT 喂它一条都收不到，
日志会写 `offering incompatible QoS ... RELIABILITY_QOS_POLICY`）：

| 来源 | 轨迹长度 | 末位置 |
|---|---|---|
| bag 内录制的 FAST-LIO `/odom` | 6.22 m | (-1.15, 4.37) |
| **本次重放的 `fast_lio`** | **7.06 m** | **(-1.113, 4.365)** ← 与录制输出**差 4 cm** |

⇒ 重放链路（话题/类型/QoS/时间戳/字段/几何）**是忠实的**；`point_lio` 与 `small_point_lio`
在同一份数据上都没跟住（`point_lio` 25.3 m，且它在第 100 帧被自己的 `pcd_save` 打死）。
**注**：`lio:=pointlio` 在本仓其实也**从未在全栈里验证过**
（`docs/params_ownership_checklist.md:71` 那个勾仍是空的），所以这不是"新槽位独有的问题"。

---

### 5.5 调参工具与判据（2026-10-05 新增，全部可复现）

**工具**（都在 `tools/`，都不需要 Gazebo/nav2，秒级~几十秒一轮）：

| 工具 | 作用 |
|---|---|
| `tools/lio_node_alone_check.py` | 既有：重放 bag 喂 LIO + 判读。本次**向后兼容地**加了两个可选开关：`--json-out`（机器可读判读结果）与 `--traj-out`（逐帧轨迹，供形状对齐用）；不传时行为与旧版一字不差 |
| `tools/lio_param_sweep.py` | 新增：**并行扫参驱动**。每组一个独立 `ROS_DOMAIN_ID`（域号用队列发放，不会撞域）、可对每组单独指定 `--start-offset`/`duration`、自动起停节点、解析判读结果、打一张汇总表 + `summary.json` |

```bash
# 单组（命令行给覆盖）：
python3 tools/lio_param_sweep.py --label t1 --param imu_meas_omg_cov:=0.6 --duration 40
# 一组配置（JSON 列表，可带 params_file / node / duration / start_offset）：
python3 tools/lio_param_sweep.py --configs .tmp_sweep/b12.json --jobs 4 --duration 40
# 注意：每组都要 source /opt/ros/humble/setup.bash && source install/setup.bash；ROS_LOG_DIR 必须可写
```

**四个判据**（缺一个都可能自欺）：

| 判据 | 含义 | 什么时候能用 |
|---|---|---|
| 轨迹长度、末位置 | 与 bag 内**录制的 FAST-LIO `/odom`**（或真值）直接比 | ⚠️ **只有当重放窗口从 bag 的起点开始**（=录制那条 odom 的原点）时才成立；从 40 s 起播时两个轨迹的原点不同，末位置不可直接比 |
| `endΔfl` | 本节点末位置与**录制 FAST-LIO 末位置**的欧氏距离（同一 odom 帧口径） | 同上（窗口从起点开始）。这是任务口径的"末位置"判据 |
| **ATE** | 把两条轨迹按**相对时间**重采样后做 **SE(2) 最小二乘对齐**（**不允许缩放**，否则"轨迹长度错了"会被洗掉）后的位置 RMSE | **任何窗口都能用**，是"形状跟没跟住"的诚实判据 |
| `/odom` 频率 | 节点是否跟得上点云（应与 `/livox/lidar` 同量级） | 任何窗口 |

### 5.6 单键 A/B 扫描（**窗口 0-40 s，duration 40**，未注明的键 = 上游默认值）

一次只动一个键，其余照旧；`fast_lio` 重放作为参照列（同窗口真值 5.51 m）：

| 改动（单键） | 轨迹长度 | 末位置 | 长度/真值 | ATE | `endΔfl` |
|---|---|---|---|---|---|
| **（基线）上游默认** | **30.20** | (0.88, -6.07) | 5.48× | 1.82 | 10.64 |
| `imu_meas_acc_cov: 0.01→0.6` | 45.75 | (-6.01, -4.20) | 8.30× | — | 9.85 |
| `imu_meas_omg_cov: 0.01→0.05` | 15.89 | (0.13, 1.43) | 2.88× | 0.45 | 3.21 |
| `imu_meas_omg_cov: 0.01→0.1` | 14.99 | (-0.20, 0.54) | 2.72× | 0.41 | 3.95 |
| `imu_meas_omg_cov: 0.01→0.2` | 12.42 | (1.57, 3.29) | 2.25× | 0.40 | 2.93 |
| `imu_meas_omg_cov: 0.01→0.4` | 11.05 | (0.77, 4.41) | 2.01× | 0.41 | 1.93 |
| **`imu_meas_omg_cov: 0.01→0.6`** | **9.49** | (-1.03, 4.42) | 1.72× | 0.41 | **0.13** |
| `imu_meas_omg_cov: 0.01→1.5` | 9.20 | (-1.14, 4.29) | 1.67× | 0.41 | 0.08 |
| `point_filter_num: 1→2` | 34.64 | (-3.67, -3.29) | 6.29× | — | 8.06 |
| `fix_gravity_direction: true→false` | 30.21 | (2.93, -5.64) | 5.48× | — | 10.81 |
| `min_distance: 0.5→0.3` | 36.90 | (4.66, -1.89) | 6.70× | — | 8.54 |
| `space_downsample_leaf_size: 0.2→0.5` | **79.73** | (8.81, -19.16) | 14.47× | — | 25.55 |
| `plane_threshold: 0.1→0.3` | 32.98 | (1.67, -4.22) | 5.99× | — | 9.04 |
| `laser_point_cov: 0.01→0.1`（在 omg=0.6 上） | 16.94 | (0.35, -0.89) | 3.08× | 0.46 | 5.47 |
| `laser_point_cov: 0.01→1.0`（同上） | 56.75 | (3.45, 7.48) | 10.30× | 5.68 | 5.56 |
| `map_resolution: 0.2→0.1`（同上） | 24.46 | (-6.71, -6.18) | 4.44× | 1.46 | 11.92 |
| `match_sqaured: 81→9`（同上） | 9.57 | (-1.09, 4.36) | 1.74× | 0.41 | 0.06 |
| `plane_threshold: 0.1→0.05`（同上） | 9.64 | (0.08, 4.79) | 1.75× | 0.41 | 1.30 |
| `omg_cov: 1000→100`（同上，角速度过程噪声） | 8.54 | (-1.15, 4.28) | 1.55× | 0.41 | 0.09 |
| `omg_cov: 1000→10000`（同上） | 11.98 | (1.23, 3.01) | 2.17× | 0.41 | 2.74 |
| **`omg_cov: 1000→1.0`（注意：R 保持 0.01）** | **40.31** | (-4.09, -5.65) | 7.32× | 2.11 | 10.44 |
| `velocity_cov: 20→2`（在 omg=0.6 上） | 7.89 | (-1.07, 4.37) | 1.43× | 0.41 | 0.08 |
| `velocity_cov: 20→1.0`（同上） | 7.45 | (-1.14, 4.30) | 1.35× | 0.41 | 0.08 |
| `velocity_cov: 20→0.5`（同上） | 7.61 | (-1.10, 4.35) | 1.38× | 0.41 | 0.06 |
| `acceleration_cov: 500→50`（同上） | 8.83 | (-1.11, 4.34) | 1.60× | 0.41 | 0.05 |
| **参照：`fast_lio` 重放（同一袋、同一窗口）** | **7.06** | **(-1.11, 4.36)** | 1.28× | 0.59 | **0.04** |

**读数（谁动了指针）**：
1. **`imu_meas_omg_cov` 是唯一一个"从发散救回跟踪"的键**：0.01→0.6 把长度从 30.2 m 砍到 9.5 m、
   ATE 从 1.82 m 砍到 0.41 m，**末位置从"差 10.6 m"变成"差 0.13 m"**。0.05 开始就已经"形状对了"
   （ATE 0.40~0.45），但长度还虚高——虚高的部分靠 `velocity_cov` 去掉。
2. **`velocity_cov` 只改"抖动"，不改"形状"**：20→1.0 让长度 9.49→7.45 m，而 ATE 一动没动（0.41→0.41）。
   这一条很重要：**不能拿"轨迹变短"当成功**，必须同时看 ATE/末位置。
   （反例：`laser_point_cov=1.0` 也能靠"把点更新调软"把长度压到 56 m 之类，但 ATE 5.68 m = 彻底跟丢。）
3. `imu_meas_acc_cov` **反向**：0.6 更差（45.75 m）。加速度计无噪声 ⇒ 本来就该信它，
   把它调软会让 `v̇ = R·a + g` 的平衡被破坏（速度是二次积分，最怕加速度滞后）。
4. 点云"越多越好"：`leaf 0.5 → 79.7 m`、`leaf 0.3 → 20.1 m`、`leaf 0.2 → 7.4 m`、
   **关掉降采样 → 6.9 m**（§5.7）；`point_filter_num: 2`（抽一半）→ 34.6 m；
   `map_resolution 0.1`（体素更细）反而 24.5 m（点云稀疏时平面拟合退化）。
5. `plane_threshold` / `match_sqaured` / `min_distance` / `fix_gravity_direction` / `check_satu`：
   **在最优解附近基本是噪声**（±0.1 m 到 ±1 m），不是主要矛盾。

### 5.7 最优配置：只改 4 个键（新文件 `mid360_sim_tuned.yaml`）

在 `mid360_sim.yaml`（仿真输入契约 + 上游滤波器默认值）基础上只改这 4 个：

| 键 | 上游/契约版 | 调参后 | 一句话理由 |
|---|---|---|---|
| `space_downsample` | `true`（leaf 0.2） | **`false`** | 0.2 m 体素降采样后**每帧只剩 ~620 点**（原始 6393、`min_distance>0.5` 后 ~3500）；Livox 非重复扫描让"每个体素留哪个点"逐帧变化 ⇒ 位姿抖动 ~5 cm/帧。关掉后 ~3500 点/帧，抖动降到 ~2 cm/帧 |
| `imu_meas_omg_cov` | `0.01` | **`0.2`** | 陀螺"量测"方差。仿真陀螺无噪声，但**越小越发散**：它决定每帧点更新通过交叉协方差给角速度的"踢腿"有多大。0.2 ≈ 让角速度状态的时间常数 ≈ 一个雷达帧长（0.126 s） |
| `velocity_cov` | `20.0` | **`0.3`** | 上游 20 相当于 4.4 (m/s)/√s 的速度随机游走，对地面机器人荒谬地松；本路线最大 0.36 m/s ⇒ 0.3 才对得上真实动力学 |
| `acceleration_cov` | `500.0` | **`50.0`** | 同理收紧（加速度计无噪声、真值平滑） |

**同一份 bag、三个互不重叠的时间窗**（每个 30 s，三次独立重放；表中 ATE 越小越"形状对"）：

| 窗口 | 真值长度 | `fast_lio` 参照 | 上游默认参数 | **`mid360_sim_tuned`** |
|---|---|---|---|---|
| 0~30 s | 4.69 m | 5.69 m / ATE 0.59 | 21.82 m / ATE 2.05 | **5.38 m / ATE 0.36** |
| 40~70 s | 2.84 m | 4.21 m / ATE 0.37 | 77.48 m / ATE 10.17 | **4.60 m / ATE 0.30** |
| 90~120 s | 4.17 m | 4.94 m / ATE 0.38 | 33.07 m / ATE 2.33 | **5.69 m / ATE 0.24** |
| （40 s 全窗口） | 5.51 m | 7.06 m / ATE 0.59 | 30.20 m / ATE 1.82 | **6.90 m / ATE 0.42** |

⇒ 长度 = FAST-LIO 参照的 **0.95× / 1.09× / 1.15× / 0.98×**（**全部落在 ±15% 内**），
**ATE 每个窗口都比 FAST-LIO 参照小**，末位置在唯一可比的那个窗口（0~40 s）差 **0.10 m**
（FAST-LIO 自己复现录制输出的误差是 0.04 m ⇒ 两者同一量级）。

### 5.8 机理、重复性、CPU，以及**诚实结论**

**(a) 为什么"无噪声 IMU"反而要把 `imu_meas_omg_cov` 调大 20 倍？**
本算法（Point-LIO 系）把 IMU 当**量测**：`h_imu` 的残差是 `ω_meas − omg − bg`，`omg` 是状态，
`imu_meas_omg_cov` 是它的**量测方差 R**。R 越小 ⇒ `(omg,bg)` 后验协方差被压得越小 ⇒
每帧那几百/几千个点更新通过交叉协方差 `P(omg,·)` 给角速度的修正被"当真"的比例越高。而我们的数据有一个
**结构性缺陷**：一帧内**所有点共用同一个时间戳**（插件 `offset_time≡0`，一帧跨 126 ms 的运动没被补偿），
所以点-面残差里带着一个**与姿态强相关**的系统性成分 ⇒ 形成
「姿态误差 → 残差 → 角速度修正 → 姿态误差」的逐帧正反馈，R=0.01 时这个环的增益接近 1 ⇒ 发散。
R=0.2 时角速度状态的时间常数 τ≈0.1 s ≈ **一个雷达帧长（0.126 s）**：
上一帧的"踢腿"在下一帧到来前基本衰减掉，而真实角速度（变化尺度 0.1~1 s）仍然跟得上。

**两端都会坏**（这是"是一对参数"的证据，不是单调调参）：

| 组合 | 等效"角速度量测增益" | 等效时间常数 | 结果（窗口 0-40 s） |
|---|---|---|---|
| R=0.01, Q=1000（上游） | 0.94 | 1 步（13 ms） | **30.2 m 发散** |
| R=0.6, Q=1000 | 0.21 | ~5 步（64 ms） | 9.5 m |
| R=0.6, Q=100 | 0.026 | ~40 步（0.5 s） | 8.5 m |
| R=0.6, Q=10000 | 0.73 | ~1.4 步 | 12.0 m |
| R=0.01, Q=1 | 0.016 | ~80 步（1 s，角速度状态几乎冻住） | **40.3 m 发散** |

⇒ 太硬（把逐帧假修正当真）和太软（角速度跟不上真值）都不行，**最优在 τ≈一个雷达帧长**。
我们没有改上游源码，只是把这个"时间常数"调到与**我们 7.85 Hz 的雷达帧率**匹配——
真机 10 Hz 时最优 R 会略小，这是**参数与帧率耦合**的直接推论，也是"为什么不能照抄真机参数"的物理解释。

**(b) 为什么"关掉体素降采样"最有效？**
降采样后每帧只有 ~620 点做逐点更新（5 点拟合平面 + 一个 30 维卡尔曼更新），
每帧的平面法向/截距估计噪声大，且**两次重放之间"留下哪些点"都可能不同**（Livox 非重复扫描 +
浮点体素边界），于是位姿被"每帧不同的约束集"来回拉 ⇒ 抖动。关掉后每帧 ~3500 点，
约束稠密且统计稳定 ⇒ 抖动从 ~5 cm/帧 降到 ~2 cm/帧。

**(c) 重复性（重要，且是"发散 vs 收敛"的分水岭）**

| 配置 | 同一配置重复运行的结果 |
|---|---|
| 上游默认（窗口 0-40 s，10 次） | **30.13 / 30.20 / 30.20 / 30.20 / 30.20 / 30.20 / 30.25 / 30.67 / 43.14 / 46.43 m**（跨 1.5×；另两个窗口上最坏到 **77.48 m**，见 §5.7）—— 落在发散边界上，**不可重复** |
| `mid360_sim_tuned`（窗口 0-40 s，3 次） | **6.88 / 6.90 / 6.92 m**，末位置 (-1.21, 4.29) 三次逐位一致；另两窗口的重复组：40~70 s = 4.60/4.63/4.63 m、90~120 s = 5.67/5.69/5.70 m —— **可重复（组内 ≤5 cm）** |

原因：节点是**单线程回调**、`handle_once()` 处理"当前队列里已有的数据"，
DDS 投递抖动会让每帧点云到达时"后面那几条 IMU 到没到"不同 ⇒ 默认参数下这点差别就足以让它发散。
关掉降采样后每帧点数多、相对时序影响被稀释，重复性随之变好。

**(d) CPU（实测，40 s 重放）**：上游默认 **0.8 s CPU**（≈2% 单核）；调参后 **3.1 s CPU**（≈8% 单核）。
关掉降采样让每帧点数 ×5.6，代价是 4 倍 CPU，但**绝对量仍然很小**（`-O3 -march=native` + OpenMP，
30 维状态是固定尺寸矩阵）。⇒ CPU **不是**本槽位的主要风险（全栈里的真实占用仍未测，见 §9）。

**(e) 诚实结论（不要只看"轨迹变短"）**
- ✅ **跟得住了**：在 4 个窗口上，调参后的轨迹长度 = FAST-LIO 参照的 0.95~1.15 倍，
  形状误差 ATE 0.24~0.42 m（占路径 5~8%），**每个窗口都比 FAST-LIO 参照更接近真值形状**；
  唯一可直接比末位置的窗口上差 0.10 m。
- ⚠️ **但仍比"真值长度"长 1.15~1.63 倍**（真值 5.51/4.69/2.84/4.17 m）。注意 FAST-LIO 自己也是
  真值的 1.18~1.48 倍 ⇒ 这条链路上"用差分求路径长度"本身就会把静止段的噪声算进去；
  这也说明**判据必须是"与参照比"而不是"与真值长度绝对比"**。
- ⚠️ **窗口起点的影响**：从 40 s 起播（机器人**正在运动**时初始化）比从 0 s 起播（起步静止）稍差
  （长度 1.09~1.15× 参照，vs 0.95~0.98×）。**真跑时是从静止起步** ⇒ 用 0 s 那一档更贴近实际。
- ⚠️ **`/odom.twist` 仍然恒 0**、杆臂写法瑕疵仍在（§6.2/§6.3），它们不影响上面的判据，
  但会影响"谁读 twist/谁读 TF"的下游模块。
- ❌ **全栈闭环仍未验证**：上面全部是"只跑 LIO 节点 + 离线重放"，没有 Gazebo/nav2/AMCL/costmap 参与（§9）。

---

## 6. 已知问题 / 调参线索（诚实清单，按优先级）

1. ✅ **【已解决，2026-10-05】跟不住**：上游默认参数在我们的仿真数据上发散（§5.2），
   现已用**离线重放 A/B** 调住（§5.5~§5.8）：新配置 `mid360_sim_tuned.yaml` 在 4 个窗口上
   轨迹长度 = FAST-LIO 参照的 0.95~1.15 倍、形状误差 ATE 0.24~0.42 m、末位置差 0.10 m。
   改的 4 个键与物理理由见 §5.7/§5.8。**上游默认参数（`mid360_sim.yaml`）仍保留**，
   它现在是"复现/对照用的基线"，不再是槽位的默认（launch 已改读 tuned 版）。
   ⚠️ 但这只是**离线开环**结论；全栈闭环仍未验证（§9）。
2. **`odom→base_link` 的杆臂换算写法有瑕疵**（读码结论，未闭环验证）：`small_point_lio_node.cpp:75`
   用的是**共轭** `T_bl⁻¹ · T_ol · T_bl`，而正确写法是 `T_ol · T_bl⁻¹`。在"base_link→livox 只有平移
   `u`、没有旋转"（我们的 URDF 正是如此）时可化简为**结果 = 正确值 + u**（即恒定偏 0.12/0.175 m，
   在 odom 系里固定，不是随姿态漂）。静止时它报 (0,0,0) 而真值应是 `-u`。⇒ **它的 `/odom` 更接近
   "雷达位姿改名叫 base_link"**，与 `lio_tf_adapter` 文档里反复强调的那个坑同源。
   这也意味着**不能**再叠加 `lio_tf_adapter` 的 `xyz:[-0.12,0,-0.125]` 补偿（会双重扣）。
   ⚠️ 0.21 m 的常值偏置在 nav 里通常会被 `map→odom` 吸收，但**转弯时的表现需要实跑确认**。
   📌 本轮实测的一个旁证：末位置与 FAST-LIO 录制输出只差 **0.10 m**（§5.7），
   比"0.21 m 常值偏置"还小 ⇒ 说明**这个偏置至少在平动轨迹的末位置上没有以 0.21 m 的形态出现**
   （FAST-LIO 自己发布的是 IMU/body 位姿、本槽位发布的是雷达位姿+u，两者在同一 odom 帧里
   恰好接近），但**旋转段的等效性仍未被独立验证**。
3. **`/odom.twist` 恒 0**（上游源码里那六行是注释、带 TODO）。⇒ 任何读 `/odom.twist` 的监控/平滑模块
   都会读到 0；`docs/tf_interface_contract.md` 里如果有依赖 twist 的检查项，**本槽位不适用**。
   （本轮**没有**改它：改了就等于动上游源码，超出"调参"范围。）
4. **它不发 `/odom` 之外的 path 话题**（RViz 里 `path` 显示会空；FAST-LIO/Point-LIO 有 Path）。
5. **`save_pcd`**：别开（写源码树，见 §3.1）。
6. **`-march=native`**：编出来的二进制**绑定编译机 CPU**；换机器跑要重编（`CMakeLists.txt:34`）。
7. **C++20 + PCH + OpenMP**：本机（GCC 11 / Humble）构建通过；老工具链可能要降标准。
8. **它自己处理杆臂 ⇒ 不需要 adapter**，但也**不会再有人**替它做 `camera_init→map` 那种回退桥；
   `localization:=''` 的回退路径已按 `map≡odom` 补了一条（§4）。
9. **【新】上游默认参数档不可重复**：同一配置 11 次重放落在 28.9~46.4 m（§5.8c）。
   这本身不是 bug 报告，而是"它落在发散边界上"的证据 —— 以后拿本槽位做 A/B，
   **单次结果不作数**，至少重复 2~3 次并看 `endΔfl/ATE`。
10. **【新】调参后的参数与帧率耦合**：`imu_meas_omg_cov` 的最优值对应"角速度时间常数 ≈ 一个雷达帧长"
    （§5.8a）。我们仿真实测 7.85 Hz（真机 10 Hz）⇒ **如果以后雷达帧率变了（改 `update_rate`、
    或 RTF 变化导致实际频率漂移），这组参数需要按同一比例重调**。当前实测帧率 7.85 Hz、
    配置按 0.126 s 对齐。

---

## 7. 跑起来（用户口径）

```bash
# ① 构建（small_point_lio 包 + 参数文件；2026-10-05 新增了 mid360_sim_tuned.yaml，必须重跑一次构建）
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select small_point_lio
source install/setup.bash

# ② 起全栈（默认值不变，只有这一条命令是新槽位）
#    注：launch 的 lio:=small_point_lio 分支**已经默认加载 mid360_sim_tuned.yaml**（§5.7），
#    不需要在命令行里传任何调参文件。
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=small_point_lio localization:=amcl nav:=rpp spin_speed:=0.0
```

**起来后按这个顺序看（前 3 条不过就别往下走）**：

```bash
# (1) 里程计出数且频率对得上点云（期望 ≈7~8 Hz，与 /livox/lidar 同量级）
ros2 topic hz /odom

# (2) TF 真在动、且量纲合理（期望位置在米级、平滑；不是恒 0 也不是每秒几十米）
ros2 run tf2_ros tf2_echo odom base_link

# (3) ★ 关键：/odom 的轨迹要跟真值/基线对得上
#     —— 2026-10-05 起本槽位**离线已调住**（§5.7）；但"全栈里到底差多少"仍必须自己看一遍。
#     同一条路线分别用 lio:=fastlio 与 lio:=small_point_lio 跑一遍，比 end-to-end 到达误差/漂移；
#     或先用离线重放做 A/B（不需要 Gazebo、一轮 = duration 秒）：
python3 tools/lio_node_alone_check.py --duration 40          # 看"轨迹长度 / 末位置 / 参照"那几行
python3 tools/lio_param_sweep.py --configs .tmp_sweep/b15.json --jobs 3 --duration 30   # 想再扫参时

# (4) 帧树：主链应是 map→odom→base_link→base_link_fake→…，且**没有多父边/闭环**
ros2 run tf2_tools view_frames            # 本槽位不应出现 camera_init→body 那条孤岛（见 docs/tf_interface_contract.md §P1）
#     注：map→odom 的唯一发布者 = localization 槽那一个节点（本槽位不参与），可用
#     `ros2 run tf2_tools view_frames` 的图 + `ros2 node info <重定位节点>` 交叉确认

# (5) P0 回归（另开一个终端；脚本只发目标并监视，不自起栈）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 \
        --localization small_point_lio        # --localization 只作快照标签，见 docs/smoke_test_runbook.md
```

**判据**：`/odom` ≈7~8 Hz 且 `tf2_echo odom base_link` 连续 → 链路 OK；
**再加两条**：①「同轨迹 vs `lio:=fastlio` 的漂移差」；②「全程路径长度量级」——
离线口径下 tuned 版 ≈ FAST-LIO 的 0.95~1.15 倍（真跑会因 CPU/RTF 不同而变，但**不该出现 3 倍以上**）。

## 8. 回退

- **不改默认值** ⇒ 什么都不用做，`lio` 仍是 `fastlio`（本槽位所有改动都在 `lio:=small_point_lio` 分支内）；
- 单次回退：去掉 `lio:=small_point_lio`（或显式 `lio:=fastlio`）；
- **【最常用】把本槽位退回"上游默认滤波参数"**：把
  `src/rm_nav_bringup/launch/bringup_sim.launch.py:61-62` 的 `'mid360_sim_tuned.yaml'`
  改回 `'mid360_sim.yaml'`（**一行**；两个文件都还在，`mid360_sim.yaml` 未被修改）。
  退回后**它会重新变成"跟不住"**（30 m 级、且不可重复），所以这只用于对照实验；
- 只想临时试别的参数：不改文件，直接在 `ros2 run` 上手传（见 `mid360_sim_tuned.yaml` 头部注释）；
- 整体回退：`git revert <本次提交>`（新增包是独立目录，删掉即可，不留残留）；
- 若怀疑是那个 `timebase` 补丁：把 `src/rm_localization/small_point_lio/src/lidar_adapter/livox_custom_msg.h`
  恢复成上游原版并重编 —— 但那样它会**静默不出 odom**（§5.3），所以正确顺序是"先改仿真插件补 `timebase`"。

## 9. 未验证清单（本次**没有**做、也不该假装做了的）

1. ⚠️ **全栈闭环（仍然是第一条）**：本任务禁止起 Gazebo/nav2 ⇒ `lio:=small_point_lio`
   **至今没有**和 nav2/costmap/AMCL/mapper/Gazebo 一起跑过。§5 **全部**是"只跑 LIO 节点 +
   离线重放 bag"的结果，**没有任何一条是全栈里的**。具体没验的：
   ① 全栈下 `/odom` 的实速率与实时性（重放是 1:1 墙钟，Gazebo 里 RTF<1、CPU 还要和 nav2 分）；
   ② 与 `map→odom`（AMCL/beluga 等）叠加后的整体 TF 行为；③ end-to-end 到达误差/任务成功率。
2. **精度/漂移**：§5.7 的结论是**离线开环、单条 bag、四个窗口**上的；同一份数据上
   长度 0.95~1.15× FAST-LIO 参照、ATE 0.24~0.42 m。**"在我们的场地里跑一条完整路线差多少"仍没有答案**
   （bag 里的路线是真实的 RMUL2026 场地回波，但只是一小段）。
3. **CPU/实时性**：本轮**测了 LIO 节点自己**（§5.8d：上游默认 0.8 s / tuned 3.1 s CPU 每 40 s 重放，
   ≈2% / 8% 单核）—— 但这是**离线重放**、且 Python 喂数据端也在抢 CPU。
   **全栈里（Gazebo + nav2 + costmap + 本节点同时跑）的真实占用与最坏帧耗时仍未测**；
   与 FAST-LIO 的同口径 CPU 对比也**没有**数据（上游自称比 Point-LIO 快 2~3×，**未复核**）。
4. **`spin_speed != 0`（小陀螺）下的表现**、以及 §6.2 那个杆臂偏置在**转弯/自转**时的影响：
   本轮 bag 里最大偏航角速度只有 0.24 rad/s，**没有**覆盖高速自转。
5. **建图产物**：`save_pcd` 关着 ⇒ 本槽位没有 PCD 输出，`PCD/<world>.pcd` 不会被它更新。
6. **与 `localization:=''` 回退路径的联调**：那条 `map→odom` 恒等静态桥只做了静态检查（条件互斥性），
   **没有**实跑过。
7. **ROS 2 Jazzy / 其它平台**：只在 Humble + 本机 GCC 11 上构建过。
8. **上游 `main` 分支与 `ros2` 的差异**：只 pin 了 `ros2`，没比较过 `main`。
9. **【新】雷达帧率变化后的参数有效性**：`imu_meas_omg_cov=0.2` 是按"角速度时间常数 ≈ 一帧长
   （0.126 s @7.85 Hz）"定的（§5.8a）。**如果以后 `update_rate`/RTF 让实际点云频率明显变化，
   这组参数需要按同一比例重调**；"帧率变了还灵不灵"**没有**测。
10. **【新】`mid360_sim_tuned.yaml` 只在一条 bag（`.tmp_bags/ret4`）上验证过**：
    该 bag 是 RMUL2026 场地的真实回波（6 千点/帧、7.85 Hz、无噪声 IMU、机器人最大 0.36 m/s），
    **换场地/换机器人速度/换点云密度后是否需要重调，未知**。
11. **【新】tuned 参数只做了"4 个键"，没有做**：`imu_meas_acc_cov` 与 `laser_point_cov` 的联合微调
    （单键扫描里两者都"越调越差"，但**没有**在关掉降采样后重新做二维联合扫描）；
    也没试 `publish_odometry_without_downsample: true`（它会把 `/odom` 提到逐点频率，
    对下游 nav2 是另一套时序，**未测**）。
12. **【新】扫描是"单键 + 少量组合"**，不是全局最优搜索：一共 15 批、约 100 次重放。
    没有做的：贝叶斯/网格联合搜索、`map_resolution × leaf` 二维、`extrinsic_est_en: true`
    （在线估计雷达-IMU 外参）——**都没有**试。
