# 里程计槽位补充：`lio:=small_point_lio`（vendored `Yancey2023/small_point_lio`）

> 2026-10-05 · 新增 `bringup_sim.launch.py` 的 `lio` 取值 `small_point_lio`（默认值仍是 `fastlio`，零影响）
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

## 6. 已知问题 / 调参线索（诚实清单，按优先级）

1. **跟不住（最重要）**：上游默认参数在我们的仿真数据上发散（§5.2）。已知的**没试过**的旋钮：
   `map_resolution`/`space_downsample_leaf_size`（试过 0.5，更差）、`point_filter_num`、
   `imu_meas_acc_cov`/`imu_meas_omg_cov`（0.01 → 0.6，point_lio 的 sim 参数就是这个量级）、
   `plane_threshold`/`match_sqaured`、`check_satu`、`fix_gravity_direction`（关掉则用参数里的
   `gravity` 常量，省掉"前 200 帧定重力"这一步）、`min_distance`（0.5 → 0.3）。
   **建议的调参方法**：用 `tools/lio_node_alone_check.py` 做**离线重放 A/B**（秒级一轮，不需要 Gazebo），
   以"轨迹长度/末位置 vs bag 内 `/odom`"为判据，收敛后再上全栈。
2. **`odom→base_link` 的杆臂换算写法有瑕疵**（读码结论，未闭环验证）：`small_point_lio_node.cpp:75`
   用的是**共轭** `T_bl⁻¹ · T_ol · T_bl`，而正确写法是 `T_ol · T_bl⁻¹`。在"base_link→livox 只有平移
   `u`、没有旋转"（我们的 URDF 正是如此）时可化简为**结果 = 正确值 + u**（即恒定偏 0.12/0.175 m，
   在 odom 系里固定，不是随姿态漂）。静止时它报 (0,0,0) 而真值应是 `-u`。⇒ **它的 `/odom` 更接近
   "雷达位姿改名叫 base_link"**，与 `lio_tf_adapter` 文档里反复强调的那个坑同源。
   这也意味着**不能**再叠加 `lio_tf_adapter` 的 `xyz:[-0.12,0,-0.125]` 补偿（会双重扣）。
   ⚠️ 0.21 m 的常值偏置在 nav 里通常会被 `map→odom` 吸收，但**转弯时的表现需要实跑确认**。
3. **`/odom.twist` 恒 0**（上游源码里那六行是注释、带 TODO）。⇒ 任何读 `/odom.twist` 的监控/平滑模块
   都会读到 0；`docs/tf_interface_contract.md` 里如果有依赖 twist 的检查项，**本槽位不适用**。
4. **它不发 `/odom` 之外的 path 话题**（RViz 里 `path` 显示会空；FAST-LIO/Point-LIO 有 Path）。
5. **`save_pcd`**：别开（写源码树，见 §3.1）。
6. **`-march=native`**：编出来的二进制**绑定编译机 CPU**；换机器跑要重编（`CMakeLists.txt:34`）。
7. **C++20 + PCH + OpenMP**：本机（GCC 11 / Humble）构建通过；老工具链可能要降标准。
8. **它自己处理杆臂 ⇒ 不需要 adapter**，但也**不会再有人**替它做 `camera_init→map` 那种回退桥；
   `localization:=''` 的回退路径已按 `map≡odom` 补了一条（§4）。

---

## 7. 跑起来（用户口径）

```bash
# ① 构建（只需这一次；small_point_lio 是新包）
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select small_point_lio
source install/setup.bash

# ② 起全栈（默认值不变，只有这一条命令是新槽位）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=small_point_lio localization:=amcl nav:=rpp spin_speed:=0.0
```

**起来后按这个顺序看（前 3 条不过就别往下走）**：

```bash
# (1) 里程计出数且频率对得上点云（期望 ≈7~8 Hz，与 /livox/lidar 同量级）
ros2 topic hz /odom

# (2) TF 真在动、且量纲合理（期望位置在米级、平滑；不是恒 0 也不是每秒几十米）
ros2 run tf2_ros tf2_echo odom base_link

# (3) ★ 关键：/odom 的轨迹要跟真值/基线对得上（本槽位**已知会跟不住**，见 §5.2）
#     同一条路线分别用 lio:=fastlio 与 lio:=small_point_lio 跑一遍，比 end-to-end 到达误差/漂移；
#     或先用离线重放做 A/B（不需要 Gazebo、秒级一轮）：
python3 tools/lio_node_alone_check.py --duration 40          # 看"轨迹长度 / 参照"那一行

# (4) 帧树：主链应是 map→odom→base_link→base_link_fake→…，且**没有多父边/闭环**
ros2 run tf2_tools view_frames            # 本槽位不应出现 camera_init→body 那条孤岛（见 docs/tf_interface_contract.md §P1）
#     注：map→odom 的唯一发布者 = localization 槽那一个节点（本槽位不参与），可用
#     `ros2 run tf2_tools view_frames` 的图 + `ros2 node info <重定位节点>` 交叉确认

# (5) P0 回归（另开一个终端；脚本只发目标并监视，不自起栈）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 \
        --localization small_point_lio        # --localization 只作快照标签，见 docs/smoke_test_runbook.md
```

**判据**：`/odom` ≈7~8 Hz 且 `tf2_echo odom base_link` 连续 → 链路 OK；**再加一条**
「同轨迹 vs `lio:=fastlio` 的漂移差」→ 才谈得上"能不能用"。

## 8. 回退

- **不改默认值** ⇒ 什么都不用做，`lio` 仍是 `fastlio`；
- 单次回退：去掉 `lio:=small_point_lio`（或显式 `lio:=fastlio`）；
- 整体回退：`git revert <本次提交>`（新增包是独立目录，删掉即可，不留残留）；
- 若怀疑是那个 `timebase` 补丁：把 `src/rm_localization/small_point_lio/src/lidar_adapter/livox_custom_msg.h`
  恢复成上游原版并重编 —— 但那样它会**静默不出 odom**（§5.3），所以正确顺序是"先改仿真插件补 `timebase`"。

## 9. 未验证清单（本次**没有**做、也不该假装做了的）

1. **全栈闭环**：本任务禁止起 Gazebo/nav2（另一条线正在跑五路重定位对比）⇒ `lio:=small_point_lio`
   **从未**和 nav2/costmap/AMCL/mapper 一起跑过。§5 全部是**只跑 LIO 节点**的结果。
2. **精度/漂移**：§5.2 的重放已经**显示跟不住**，但那是**离线开环**、且时间轴是重放的墙钟；
   "在我们的场地里跑一条完整路线差多少"**没有**答案。
3. **CPU/实时性**：没测。它吃 7522 点/帧 + iVox 逐点更新，`-march=native` 编译；
   与 FAST-LIO 的 CPU 对比**没有**数据（上游自称比 Point-LIO 快 2~3×，**未复核**）。
4. **`spin_speed != 0`（小陀螺）下的表现**、以及 §6.2 那个杆臂偏置在转弯时的影响。
5. **建图产物**：`save_pcd` 关着 ⇒ 本槽位没有 PCD 输出，`PCD/<world>.pcd` 不会被它更新。
6. **与 `localization:=''` 回退路径的联调**：那条 `map→odom` 恒等静态桥只做了静态检查（条件互斥性），
   **没有**实跑过。
7. **ROS 2 Jazzy / 其它平台**：只在 Humble + 本机 GCC 11 上构建过。
8. **上游 `main` 分支与 `ros2` 的差异**：只 pin 了 `ros2`，没比较过 `main`。
