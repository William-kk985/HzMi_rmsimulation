# 地面分割槽位（`ground`）：linefit ↔ Patchwork++

> 入口：`rm_nav_bringup/launch/bringup_sim.launch.py` 的 **`ground`** 参数
> 当前 choices：`['linefit', 'patchwork']`，**默认 `linefit`**（= 现有行为不变）
> （`ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args` 能直接看到这行说明）
> **统一契约**：地面分割槽**只负责把输入点云一分为二**，发 `/segmentation/obstacle` + `/segmentation/ground`；
> **同一个话题同一时刻只能有一个发布者** ⇒ 两个分割器必须互斥（由 launch 的 `condition` 保证，见 §4.3）。
> 下游（`pointcloud_to_laserscan` → `/scan` → costmap/STVL/cartographer/slam_toolbox）**零改动**。

## 0. 一句话结论（先看这个）

| 问题 | 结论 |
|---|---|
| Patchwork++ 能 vendor 吗 | **能**。`url-kaist/patchwork-plusplus` 本体是 **BSD-2-Clause**（pin `3e6903a1…` = tag v1.4.1）。 |
| 它的 ROS 2 wrapper 能 vendor 吗 | **没 vendor**。上游 `ros/package.xml` 写 **GPL-3.0**，而 `ros/LICENSE` 是 **MIT** —— 两处声明互相矛盾，按"许可证不清就不 vendor"处理 ⇒ **自己写薄节点**，只依赖 BSD-2 的 `cpp/**`。 |
| 默认值改了吗 | **没改**。`ground` 默认 `linefit`，linefit 节点、参数文件、p2l、其它所有槽位**一个字节都没动**。 |
| patchwork 更好吗 | **在斜面/倒角上明显更好，在低矮障碍上要调一个参数才追平，CPU 更省**。逐项见 §5/§6 与 §7 的结论。 |
| 能用了吗 | ⚠️ **可以 A/B，但建议先别换默认**：本次只做了离线 + 建图链 smoke，**没做 `mode:=nav` 整栈回归**（§9）。 |

---

## 1. 上游事实（全部带 URL；抓取方式 = GitHub REST API + `raw.githubusercontent.com` 的 `curl`）

### 1.1 仓库与 pin

| 项 | 值 | 来源 |
|---|---|---|
| 仓库 | <https://github.com/url-kaist/patchwork-plusplus> | `GET /repos/url-kaist/patchwork-plusplus` → `full_name` |
| 简介 | "Patchwork++: Fast and robust ground segmentation method for 3D LiDAR scans. @ IROS'22" | 同上 → `description` |
| 论文 | Patchwork++（IROS 2022），Patchwork（RA-L 2022）的改进版：RNR / CZM / R-VPF / R-GPF / GLE / TGR | 上游 `README.md` |
| **pinned commit** | **`3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`** | `GET /repos/.../commits/master` → `sha` |
| 对应 tag | **`v1.4.1`** | `GET /repos/.../tags` → `v1.4.1` 指向同一 sha |
| commit 日期 / 信息 | 2026-05-23 / `chore(release): v1.4.1 (#101)` | `GET /repos/.../commits/master` |
| 默认分支 / star / 归档 | `master` / 1102 / 未归档 | `GET /repos/...` |
| 上游版本号 | `project(patchworkpp VERSION 1.4.1)` | `cpp/CMakeLists.txt` |

**为什么 pin 这个而不是更老的版本**：抓取时 `master` HEAD 就是一个**已发布 tag**（v1.4.1），且 v1.3.1 修了 TGR 的 `ringwise_flatness` 泄漏（上游 issue #69）、v1.4.1 修了每 patch 的堆抖动（+14.8% Hz）、v1.2.0 修了"非 360° 点云崩溃"。**没有理由用旧版本**。

### 1.2 许可证（关键判断）

| 路径 | 声明的许可证 | 证据 |
|---|---|---|
| 仓库根 `LICENSE` | **BSD-2-Clause**，`Copyright (c) 2024, Urban Robotics Lab. @ KAIST` | `GET /repos/...` → `license.spdx_id = "BSD-2-Clause"`；`curl raw.../LICENSE` 全文核对 |
| **`cpp/**`（库本体）** | 继承根 LICENSE ⇒ **BSD-2-Clause** | `cpp/` 下**没有**独立 LICENSE 文件；`cpp/cmake/LICENSE` 是**Eigen 的**许可说明（MPL2/LGPL），只对下载来的 Eigen 生效 |
| **`ros/**`（上游 ROS 2 wrapper）** | ⚠️ **矛盾**：`ros/package.xml` 写 `<license>GPL-3.0</license>`，而 `ros/LICENSE` 是 **MIT**，`Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss`（明显是从 KISS-ICP 抄来的模板，作者都不是 Patchwork++ 的作者） | 两个文件原文 |

⇒ **决策**：只 vendor `cpp/**`（BSD-2-Clause，许可清楚），**不 vendor `ros/**`**（许可不清），自己写一个薄 ROS 2 节点。
副作用是好的：我们的节点只依赖 `Patchworkpp` 这个类，不需要上游 wrapper 那一堆 `tf2/nav_msgs/std_msgs/rcutils` 依赖。

### 1.3 构建系统与依赖

| 项 | 事实 | 证据 |
|---|---|---|
| 构建系统 | **CMake**（`cmake_minimum_required(VERSION 3.11)`） | `cpp/CMakeLists.txt` |
| **是 header-only 吗** | **不是**。两个静态库：`ground_seg_common`（`cpp/common/src/plane_fit.cpp`）+ `ground_seg_cores`（`cpp/patchworkpp/src/patchworkpp.cpp`）；头文件在 `cpp/{common,patchworkpp}/include/patchwork/` | `cpp/{common,patchworkpp}/CMakeLists.txt` |
| Eigen 依赖 | **有**。`target_link_libraries(${TARGET_NAME} Eigen3::Eigen ground_seg_common)` | `cpp/patchworkpp/CMakeLists.txt` |
| Eigen 从哪来 | ⚠️ 上游默认 `option(USE_SYSTEM_EIGEN3 ... OFF)`，找不到系统 Eigen 就 **`FetchContent` 从 gitlab 下载 Eigen 3.4.0 源码** ⇒ **构建期联网**。而且 `cmake_minimum_required(3.11)` ⇒ CMP0077 为 OLD ⇒ **普通变量设不进去**，只能 `-DUSE_SYSTEM_EIGEN3=ON` | `cpp/CMakeLists.txt` + `cpp/cmake/eigen.cmake` |
| **我们的处置** | **不用上游 `cpp/CMakeLists.txt`**，改写一个 `thirdparty/patchwork-plusplus/CMakeLists.txt`（只把两个 `.cpp` 编成一个静态库、链系统 `Eigen3::Eigen`）；**上游 `.h/.cpp` 一个字节没改** | 见该文件顶部注释 + `VENDORING.md` |
| C++ 标准 | 上游 `set(CMAKE_CXX_STANDARD 20)`。我们核对过这两个 `.cpp`/`.h` **没有任何 C++17 之后的构造**（无 concepts/ranges/span/`<=>`/`<format>`）⇒ 本包按 **C++17**（Humble 主口径）编译，结果一致 | `grep` 全量源码 |
| TBB | 只在**经典 Patchwork**（`cpp/patchwork/**`）里可选用；**Patchwork++ 不用 TBB**（上游注释：实测多核反而慢 30~50%） | `cpp/CMakeLists.txt` 注释 + `patchworkpp.cpp` 主循环注释 |
| 本机工具链 | gcc 11.4.0 / Eigen 3.4.0（`libeigen3-dev`）/ PCL 1.12 / VTK 9.1 / ROS 2 Humble | `g++ --version`、`dpkg -l` |

### 1.4 ROS 2 现状（上游**有**官方 ROS 2 端口）

- 上游有 `ros/` 目录：`project(patchworkpp VERSION 1.0.4 LANGUAGES CXX)`，`ament_cmake` + `rclcpp_components`，
  `rclcpp_components_register_node(gseg_component PLUGIN "patchworkpp_ros::GroundSegmentationServer" EXECUTABLE patchworkpp_node)`。
- **CI 明确覆盖 ROS 2**：`.github/workflows/ros.yml` 的 matrix 是 `[humble, jazzy]`，容器 `osrf/ros:${{ matrix.release }}-desktop`。
- 它的发布话题是 `/patchworkpp/{cloud,ground,nonground}`，**不是**我们的 `/segmentation/*` —— 契约不同，所以即使许可证没问题也要写适配层。
- **本节点没有用它**，只用 `cpp/**` 的 `patchwork::PatchWorkpp` 类。

### 1.5 COD 是怎么接的（可核对的一手证据）

| 事实 | 证据（URL） |
|---|---|
| COD_NAV `master`（2025 赛季线，冻结于 `54dc95af4` / 2025-08-02）把 patchwork++ 作为 **git submodule** | <https://github.com/qza36/COD_NAV/blob/master/.gitmodules> → `[submodule "patchwork-plusplus"] url = https://github.com/url-kaist/patchwork-plusplus.git` |
| 他们 pin 的 commit | `b608129a4549523067eae461da656195026cd7ca`（2025-01-28，`EigenMatToPointCloud2 - Use rows() of Eigen::MatrixX3f &points (#68)`）—— 比 v1.0（2024-10）新、比 v1.1.0 旧 |
| 接线：**直接用上游的 ROS 2 wrapper launch** | <https://github.com/qza36/COD_NAV/blob/master/nav_bringup/launch/bringup.launch.py> → `patchworkpp_dir = get_package_share_directory('patchworkpp')` … `IncludeLaunchDescription(PythonLaunchDescriptionSource([patchworkpp_dir,'/launch/patchworkpp.launch.py']))` |
| 接线：p2l 吃 **nonground** | 同上 → `Node(package='pointcloud_to_laserscan', …, remappings=[('cloud_in','/patchworkpp/nonground'),('scan','/scan')], parameters=[{… 'min_height': 0.01, 'max_height': 1.00, 'range_min': 0.48, 'range_max': 20.0, 'target_frame': 'chassis' …}])` |
| 他们实际用的算法参数 = **上游 launch 的默认值**（他们没有覆盖） | 上游 <https://github.com/url-kaist/patchwork-plusplus/blob/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/ros/launch/patchworkpp.launch.py> → `sensor_height: 1.88, num_iter: 3, num_lpr: 20, num_min_pts: 0, th_seeds: 0.3, th_dist: 0.125, th_seeds_v: 0.25, th_dist_v: 0.9, max_range: 80.0, min_range: 1.0, uprightness_thr: 0.101, verbose: True` |
| **2026 赛季把它整条砍了** | 本仓自身考证：`docs/cod_nav_2026_deep_dive.md:322`「`patchwork`/`linefit`/`terrain_analysis` **0 命中**」；`docs/cod_nav_nav_stack_inventory.md:151-152`（`rmul2026` 去掉 submodule）；`docs/cod_nav_migration_worksheet.md:59`（改由两层 STVL + p2l 硬高度带挡住地面） |

本仓 `third_party/cod_nav_2026/` 里 `grep -ril patchwork` **0 命中** —— 与上面的结论一致（他们的 2026 分支确实删干净了）。

### 1.6 本次**没能确认**的（不要当成已知）

1. **`ros/` 到底适用哪个许可证** —— `package.xml`(GPL-3.0) 与 `LICENSE`(MIT) 矛盾，无法从仓库内部判定；也没找到上游 issue/PR 说明。
2. **COD 实跑时 `patchworkpp_node` 的真实参数值**：他们只是 include 上游 launch，而该 launch **硬写死了**参数（注释原话："This configuration parameters are not exposed through the launch system"）⇒ 我们只能确认"他们用的是上游 launch 里那组值"，**不能**确认他们后来有没有改 `ros/launch/patchworkpp.launch.py`（那需要 clone submodule，本沙箱没做）。
3. **上游 `intensity_thr` 是死参数**：`patchwork::Params` 里有它，但 `patchworkpp.cpp` 里 `params_.intensity_thr` **0 次引用**（`grep -c` = 0）⇒ 本节点**故意不声明**它。
4. **RNR 在我们链路上无效**：`reflected_noise_removal()` 第一句是 `if (cloud_in.cols() < 4) { cout << "RNR requires intensity information !"; return; }`，我们的仿真点云是 `pcl::PointXYZ`（3 列）⇒ 开了也只有一行 stdout。真机接 Livox 原始点（带 intensity）后才可能有用。
5. **上游 wrapper 的实际运行效果**：本沙箱没装/没跑上游 `patchworkpp` 包，一切"上游 wrapper 怎么样"的说法都只来自源码阅读。

---

## 2. 契约表（逐字段对齐 linefit 节点）

参照实现：`src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`
新实现：`src/rm_perception/patchwork_ground_segmentation/src/patchwork_ground_segmentation_node.cpp`

| 契约项 | `ground:=linefit` | `ground:=patchwork` | 一致? |
|---|---|---|---|
| 节点名 | `ground_segmentation` | `ground_segmentation`（同名；两者互斥 ⇒ 不会撞名） | ✅ |
| 订阅话题 | `input_topic` 默认 `/livox/lidar/pointcloud` | 同 | ✅ |
| 订阅 QoS | `rclcpp::SensorDataQoS()` = BEST_EFFORT + keep_last(5) | 同 | ✅ |
| 发布话题 | `obstacle_output_topic` = `segmentation/obstacle`（相对名 ⇒ `/segmentation/obstacle`）<br>`ground_output_topic` = `segmentation/ground` | 同 | ✅ |
| 发布 QoS | `SensorDataQoS()` | 同 | ✅ |
| 消息类型 | `sensor_msgs/msg/PointCloud2` | 同 | ✅ |
| 点类型/字段 | `pcl::PointXYZ`（x/y/z + padding） | 同 | ✅ |
| 帧处理 | 输出 `header` **整体抄输入**（stamp + frame_id 逐字段相同） | 同 | ✅ |
| 重力对齐 | `gravity_aligned_frame` 默认 `""` ⇒ 不做 TF、不做变换 | 同默认；**只有非空时才建 TF buffer/listener**（见下） | ✅（行为等价） |
| intensity | 不读 | 不读（RNR 需要 intensity ⇒ 默认关） | ✅ |
| `timebase` | 不读（仿真插件不填） | 不读 | ✅ |
| **分类完备性** | `segmentation->resize(cloud.size(), 0)`，越界点（`r<r_min` 或 `r>r_max`）`bin_index=(-1,-1)` ⇒ 保持 0 = **障碍** | 每个输入点必进 `cloud_ground_` 或 `cloud_nonground_`：`pc2czm()` 里 `min_range<r<=max_range` 进 CZM，**其余直接进 nonground**；patch 内点或被 GLE 判 ground，或进 nonground ⇒ **越界也是障碍** | ✅ **语义相同** |
| 输出点顺序 | 按输入下标顺序归拢 | 同（`getGroundIndices()` 返回的是**输入行号**：`pc2czm` 第 611 行 `PointXYZ(x,y,z,i)`） | ✅ |
| `/segmentation/*` 发布者个数 | 1 | 1（互斥，见 §4.3） | ✅ |

**唯一有意的差异**（不改契约，只减故障面）：
linefit 节点**无条件**构造 `tf2_ros::Buffer` + `TransformListener`；patchwork 节点**只在 `gravity_aligned_frame` 非空时**才构造。
理由与 `pointcloud_to_laserscan` 的 `target_frame: ""` 同款（见该包 config 注释）：把整类"tf2 过滤器丢消息/静默排队"的故障面从链路里去掉。输出 `header` 不变。

---

## 3. 参数表（每个默认值的来源都写清楚）

参数文件：`src/rm_perception/patchwork_ground_segmentation/config/ground_segmentation_sim.yaml`（31 个键）。
来源标注四档：**[上游默认]** = 上游 `patchwork::Params()` 构造函数字面值 · **[上游 ROS]** = 上游 `ros/launch/patchworkpp.launch.py` 里写的值（= COD 实际用的那组）· **[本仓实测]** = 我们量出来的 · **[本仓决定]** = 拍的值 + 理由。

| 键 | 我们的默认 | 上游默认 | 上游 ROS/COD | 来源与理由 |
|---|---|---|---|---|
| `input_topic` | `/livox/lidar/pointcloud` | — | — | [本仓] = linefit 同值 |
| `obstacle_output_topic` / `ground_output_topic` | `segmentation/obstacle` / `segmentation/ground` | — | — | [本仓] = linefit 同值 |
| `gravity_aligned_frame` | `""` | — | — | [本仓] = linefit 同默认。livox 安装 `rpy = 0 0 0` ⇒ z 轴已与重力对齐，不需要 TF |
| `sensor_height` | **0.226 m** | 1.723 | 1.88 | **[本仓实测]** RMUL2026 仿真雷达系地面峰 `z = −0.226 m`（2026-09-22 标定，与 linefit `segmentation_sim.yaml` 同源；URDF 0.175 + 车高≈0.06）。⚠ **它只是种子**：上游 `update_elevation_thr()` 每帧把 `sensor_height` 改写成 `-mean(近处地面 patch 的 z)`（`patchworkpp.cpp:360`） |
| `min_range` | **0.2 m** | 2.7 | 1.0 | [本仓决定] = linefit 的 `r_min: 0.2`。**必须与 linefit 对齐**：小于它的点被判 nonground=障碍，若取 1.0 会在车身周围造出一圈假障碍 |
| `max_range` | **20.0 m** | 80.0 | 80.0 | [本仓决定] = COD 的 p2l `range_max: 20.0`（更远的点 p2l 本来就不要）。**而且**：`max_range` 越小 ⇒ 同心区分环越细 ⇒ patch 越小 ⇒ 坡面拟合越好 |
| `th_dist` | **0.08 m** ⚠ **我们唯一偏离上游默认的算法键** | **0.125** | 0.125 | 见下方「为什么改 `th_dist`」 |
| `th_seeds` | 0.125 | 0.125 | 0.3 | [上游默认] |
| `th_seeds_v` / `th_dist_v` | 0.25 / 0.1 | 0.25 / 0.1 | 0.25 / **0.9** | [上游默认]。上游 ROS launch 把 `th_dist_v` 放到 0.9（极激进）；我们保持库默认 |
| `num_iter` | 3 | 3 | 3 | [上游默认]（= 上游 ROS/COD 同值） |
| `num_lpr` | 20 | 20 | 20 | [上游默认] |
| `num_min_pts` | 10 | 10 | **0** | [上游默认]。`0` 会让 1~2 点的 patch 也去拟合 PCA ⇒ 退化法向；本仓单帧 ~7e3 有效点，10 不会饿死 patch |
| `num_zones` | 4 | 4 | 4 | [上游默认]。**上游代码硬上限 4**（`min_ranges_`/`ring_sizes_`/`sector_sizes_` 各 4 项、`pc2czm` 4 个分支、`update_elevation_[4]`）⇒ 节点里做了形状校验 |
| `num_sectors_each_zone`（**方位**阈值） | `[16,32,54,32]` | 同 | 同 | [上游默认] |
| `num_rings_each_zone`（**径向**环数） | `[2,4,4,4]` | 同 | 同 | [上游默认]。配 `min_range 0.2 / max_range 20` ⇒ 区边界 2.675 / 5.15 / 10.1 m，zone0 环宽 1.24 m，zone3 环宽 2.48 m |
| `num_rings_of_interest` | 4 | 4 | 4 | [上游默认]（GLE 自适应阈值只对最近的 4 个 ring 更新） |
| `uprightness_thr` | **0.707**（≈45° 坡度上限） | 0.707 | **0.101**（≈84°） | [上游默认]。COD/上游 launch 的 0.101 几乎不挡坡度 ⇒ 近垂直面片也可能被当 upright 进而进 GLE 判地面；我们场地最大设计坡度是 22°，`cos22° = 0.927 ≫ 0.707` ⇒ 保持库默认更保守 |
| `adaptive_seed_selection_margin` | −1.2 | −1.2 | 同 | [上游默认] |
| `elevation_thr` / `flatness_thr` | `[0,0,0,0]` / `[0,0,0,0]` | 同 | 同 | [上游默认]。**这是冷启动种子，不是常量阈值**：每帧被 `mean ± k·stdev` 覆盖。别把它当固定高度带用 |
| `max_flatness_storage` / `max_elevation_storage` | 1000 / 1000 | 同 | 同 | [上游默认]（自适应滑窗长度） |
| `enable_RNR` | **false** | true | 上游 wrapper 里硬写 false | [本仓实测] 我们的点云无 intensity ⇒ RNR 无效（见 §1.6 第 4 条）。上游 wrapper 也是 false |
| `enable_RVPF` | true | true | true | [上游默认] 保留：坡道/路缘的**竖直面**正是要判成障碍的东西 |
| `enable_TGR` | true | true | true | [上游默认] 保留：Patchwork++ 相对 Patchwork 的主要增益（v1.3.1 修了它的 `ringwise_flatness` 泄漏） |
| `RNR_ver_angle_thr` / `RNR_intensity_thr` | −15.0 / 0.2 | 同 | 同 | [上游默认]（`enable_RNR=false` 时不生效，只为参数完整） |
| `verbose` | **false** | false | **true** | [本仓决定] 上游 launch 的 true 会每帧打十几行 cout；仿真机 RTF 紧张。A/B 取耗时改用节点自己发的 `~/segmentation_time_ms` |
| `publish_timing` | true | —（我们新增） | — | [本仓] 发 `~/segmentation_time_ms`（`std_msgs/Float64`）= 本帧 `estimateGround()` 墙钟毫秒。**私有命名空间、不参与 `/segmentation/*` 契约** |
| ~~`intensity_thr`~~ | **不声明** | 0.2 | — | 上游死参数（`grep -c params_.intensity_thr` = 0）⇒ 故意不暴露，避免"设了以为生效" |

### 3.1 为什么把 `th_dist` 从 0.125 改成 0.08（可复现的实测理由）

`th_dist` = "点到拟合平面的距离小于它才算地面"，也就是**地面层的厚度**。它有一个直接后果：
**任何竖直特征最下面 `th_dist` 那一段会被并进地面**。

台架实测（`ground_seg_ab --synthetic`，§5.2）：`th_dist = 0.125` 时，一堵 **0.15 m 薄墙**在 p2l 口径下的
**方位覆盖率只剩 11.8%**（linefit 是 100%）⇒ **矮墙从 `/scan` 里消失**。这直接违反本项目
"矮墙必须留成障碍"的硬要求（`min_obstacle_height 0.0` 的存在理由）。三档对照（覆盖率 = 该特征的方位 bin 里还有没有障碍点）：

| `th_dist` | 0.15 m 薄墙 覆盖 | 0.30 m 薄墙 覆盖 | 0.40 m 薄墙 覆盖 | 0.2 m/15° 倒角判地面 | 0.2 m 平台顶面 覆盖 |
|---|---|---|---|---|---|
| 0.125（上游默认） | **11.8%** ❌ | 100% | 100% | **91.3%** | 100% |
| **0.08（采用）** | **100%** ✅ | 100% | 100% | **69.1%** | 100% |
| 0.05（= linefit） | 100% | 100% | 100% | 53.3% | 100% |

⇒ **0.08 = 折中**：矮墙与平台全部保住，同时倒角判地面的比例仍是 linefit 的 **2.6 倍**。
`0.06` 更保守（矮墙 51% 点、倒角 54%）；`0.10` 介于两者之间（矮墙覆盖率见 §5.2 表）。
**这是一处明确的上游默认偏离，已在此与参数文件里双处登记。**

---

## 4. 槽位用法

### 4.1 launch 的改动（只有这些，别处一行没动）

```diff
+    declare_ground_cmd = DeclareLaunchArgument(
+        'ground',
+        default_value='linefit',
+        choices=['linefit', 'patchwork'],
+        description='地面分割器槽位…')
+
     bringup_linefit_ground_segmentation_node = Node(
         package='linefit_ground_segmentation_ros',
         executable='ground_segmentation_node',
-        parameters=[segmentation_params, {'use_sim_time': use_sim_time}]
+        parameters=[segmentation_params, {'use_sim_time': use_sim_time}],
+        condition=LaunchConfigurationEquals('ground', 'linefit'),
     )
+
+    bringup_patchwork_ground_segmentation_node = Node(
+        package='patchwork_ground_segmentation',
+        executable='patchwork_ground_segmentation_node',
+        name='ground_segmentation',
+        parameters=[_PackageShareFile('patchwork_ground_segmentation', 'config',
+                                      'ground_segmentation_sim.yaml'),
+                    {'use_sim_time': use_sim_time}],
+        condition=LaunchConfigurationEquals('ground', 'patchwork'),
+    )

     ld.add_action(declare_mapper_cmd)
+    ld.add_action(declare_ground_cmd)
     ld.add_action(declare_gloal_obstacle_cmd)          # 原文如此
     ld.add_action(bringup_linefit_ground_segmentation_node)
+    ld.add_action(bringup_patchwork_ground_segmentation_node)
```

两个实现细节：

1. patchwork 的参数文件用 **`_PackageShareFile` 惰性 substitution**（本文件顶部那段"描述构建期陷阱"的老问题）：
   没构建过 `patchwork_ground_segmentation` 时，**只要不选这个槽位**，`--show-args` 与其它任何组合都不受影响；
   选了但没构建，报错是可操作的一句话（含 `colcon build --packages-select` 与替代槽位值）。
2. **默认 `linefit` ⇒ 现有行为逐字节不变**：linefit 节点原来怎么写现在还怎么写，只是多了一层**恒真**的 condition。

### 4.2 `--show-args` 证据（原文）

```text
$ ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args
    'ground':
        地面分割器槽位（A/B 对照用，只换"谁发 /segmentation/*"，其它一律不动）: linefit = 已验证默认
        （linefit_ground_segmentation_ros，每扇区直线拟合 + max_dist_to_line 硬容差；参数在
        linefit_ground_segmentation_ros/config/segmentation_sim.yaml）| patchwork = Patchwork++
        （url-kaist/patchwork-plusplus，BSD-2-Clause，vendored 在 patchwork_ground_segmentation 包内，
        pin 3e6903a1 = v1.4.1；自适应平面拟合，坡面更稳；参数在 patchwork_ground_segmentation/config/
        ground_segmentation_sim.yaml）。两者同契约 ⇒ 下游（pointcloud_to_laserscan / STVL / costmap）
        零改动、互斥切换；回退 = 省略本参数或 ground:=linefit. Valid choices are: ['linefit', 'patchwork']
```

### 4.3 互斥真值表（**不是 grep**，是把 LaunchDescription 真跑出来求值 condition）

```text
$ python3 src/rm_perception/patchwork_ground_segmentation/bench/verify_ground_slot.py
LaunchDescription 来源: install/rm_nav_bringup/share/rm_nav_bringup/launch/bringup_sim.launch.py

== ground:=linefit ==
  [ON ] linefit_ground_segmentation_ros/ground_segmentation_node  condition=LaunchConfigurationEquals
  [off] patchwork_ground_segmentation/patchwork_ground_segmentation_node  name='ground_segmentation'
  ✅ 恰好 1 个分割器被启用：linefit_ground_segmentation_ros/ground_segmentation_node

== ground:=patchwork ==
  [off] linefit_ground_segmentation_ros/ground_segmentation_node
  [ON ] patchwork_ground_segmentation/patchwork_ground_segmentation_node  name='ground_segmentation'
  ✅ 恰好 1 个分割器被启用：patchwork_ground_segmentation/patchwork_ground_segmentation_node

互斥断言: PASS
```

**运行期实证**（`ground:=patchwork` 实跑中查 `/segmentation/obstacle`）：

```text
$ ros2 topic info -v /segmentation/obstacle
Publisher count: 1
Node name: ground_segmentation      Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Subscription count: 2
Node name: pointcloud_to_laserscan  Node namespace: /
```

### 4.4 跑起来

```bash
# A：现有默认（= 什么都没变）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox ground:=linefit

# B：换 Patchwork++
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox ground:=patchwork

# 只看分割器（不要 Gazebo/LIO，方便对着 ros2 bag play 调试）
ros2 launch patchwork_ground_segmentation patchwork_ground_segmentation.launch.py

# 判活
ros2 topic hz /segmentation/obstacle          # 期望 ≈ 输入点云频率
ros2 topic echo /segmentation/obstacle --field header --once
ros2 topic hz /ground_segmentation/segmentation_time_ms   # patchwork 的每帧耗时
```

---

## 5. 离线 A/B（同一批点、逐点比对）

台架：`src/rm_perception/patchwork_ground_segmentation/bench/ground_seg_ab.cpp`
（把**同一份点集**先后喂给 linefit 库与 Patchwork++ 库 ⇒ 不存在"两次回放不同帧"的噪声；
两侧参数默认值 = 两边节点正在用的参数文件的值）

### 5.1 真实帧（`.tmp_bags/ret4`，RMUL2026，60 帧，已剔除插件的 `(0,0,0)` 填充点）

命令：`ground_seg_ab --frames-dir .tmp_ground_ab/ret4 --drop-zeros --sensor-height 0.226`

| 指标 | linefit | Patchwork++ | 谁好 |
|---|---|---|---|
| 逐点一致率 | — | **85.61%**（逐帧 79.8~89.4%） | — |
| 混淆矩阵 | both_ground 24.64% / **linefit_only_ground 9.77%** / **patchwork_only_ground 4.62%** / both_obstacle 60.97% | | — |
| 判为地面的比例 | 25.02% | 39.40% | （patchwork 更"敢"判地面） |
| **CPU / 帧（中位 / p95 / max）** | 1.013 / 1.330 / 1.396 ms | **0.522 / 0.579 / 0.904 ms** | **patchwork 快 1.94×** |
| 高度带 0.20~0.30 m 的障碍保留率 | 98.65% | **99.10%** | patchwork ✅ |
| 高度带 0.10~0.20 m | 94.18% | **95.19%** | patchwork ✅ |
| 高度带 0.30~0.40 m | **99.52%** | 99.15% | 并列 |
| 倾角 10~15° 的障碍率 | 5.43% | **2.20%** | patchwork（更少把坡面当障碍）✅ |
| 倾角 15~22° 的障碍率 | 9.02% | **6.79%** | patchwork ✅ |
| 倾角 60~90°（竖直结构）的障碍率 | 85.62% | **91.03%** | patchwork ✅ |

⚠ 限制：`.tmp_bags/ret4` 是**插件修复前**录的，30 000 点里 23 774 点是 `(0,0,0)` 填充点（79%）——
已用 `--drop-zeros` 剔除并在此声明；剔除前两者一致率会被这些点抬高（两个分割器都把 `r=0` 判障碍）。
另外 `z_ground` 是按"两侧都判地面且 r<3 m 的点的 z 中位数"逐帧估的，个别帧被污染（-0.12~-0.23），
所以高度分桶的**桶边界**有少量抖动，但同一帧内两侧用的是同一个边界 ⇒ 对比仍公平。

### 5.2 合成地形（**RMUC2026.stl 实测几何的忠实代理** —— 这是我们能给出的最接近"坡"的东西）

先量了场地本身（`RMUC2026.stl`，binary STL 45 900 三角面，`<scale>0.001`，visual pose `z=+1.6413436`）：

| 场地事实（世界系，0 = 可行驶地面） | 数值 |
|---|---|
| 可行驶地面 | z = 0.000 m（655 m²） |
| **0.2 m 平台** | z = +0.200 m（106.2 m²） |
| **0.3 m 平台** | z = +0.300 m（70.7 m²） |
| **坑** | z = −0.200 m（466 m²） |
| **倒角坡度** | 10~15°（11.2 m²）+ 15~22°（16.7 m²），**集中在 z ∈ [0, 0.3]** ⇒ 就是平台边缘的倒角 |
| 竖直面 | 集中在 z ∈ [0.0, 0.3]（台阶/路缘立面） |
| 长坡 | **没有**（22~35° 仅 7.5 m²、35~60° 仅 7.4 m²，且都是局部小面） |

⇒ 台架按这些数值建了 7 个"**STL 代理**"场景（0.2/0.3 m 平台 + 10/15/22° 倒角 + −0.2 m 坑），
外加 3 个长坡压力场景与 3 个薄墙场景。全部解析生成 ⇒ **每个点的真值类别已知**。

**命令**：`ground_seg_ab --synthetic --sensor-height 0.226`（默认 `th_dist 0.08`）

| 真值类别 | linefit 判障碍 | Patchwork++ 判障碍 | 谁好 |
|---|---|---|---|
| **0.2 m 平台顶面**（应判障碍） | 99.53% | **100.00%** | ✅ 两者都保住 |
| **0.3 m 平台顶面** | 99.53% | **100.00%** | ✅ |
| **0.15 m 薄墙**（应判障碍） | 66.67% | 34.72%（但**方位覆盖 100%**） | ✅ 两者都进 `/scan` |
| **0.30 m 薄墙** | 80.00% | 60.83%（覆盖 100%） | ✅ |
| **0.40 m 薄墙**（= 护墙量级） | 85.71% | 72.02%（覆盖 100%） | ✅ **保住** |
| **10~22° 倒角**（应判地面） | 71.8~73.5% ❌ | **30.9~31.3%** | **patchwork 好 2.3×** ✅ |
| 0.3 m 平台的 10~22° 倒角 | 81.0~81.8% ❌ | **53.9~56.5%** | patchwork 好 ✅ |
| **坡面点判成地面的比例** | 26.5~28.2% | **68.7~69.1%** | **patchwork 好 2.6×** ✅ |
| −0.2 m 坑的下坡倒角 | **8.1% 障碍（= 91.9% 判地面）** | 52.5% 障碍 | **linefit 好** ❌ patchwork 在下坡倒角上更差 |
| 平地（真值地面）被误判成障碍 | **0.08%** | 0.68~0.96%（但**方位覆盖 8.8~10.8%** vs linefit 1.0~1.2%） | **linefit 好** ⚠️ patchwork 会在倒角附近的平地上留下少量假障碍 |
| 长坡（3 m，10~22°）压力场景 | 判地面 6.6~39.3% | 判地面 11.4~42.9% | patchwork 略好，**但两者都做不好** |

**倒角方位覆盖率（p2l 口径：该方位还有没有障碍点）**

| 场景 | linefit | Patchwork++ |
|---|---|---|
| stl-terrace0.2-chamfer10 | 94.28% | **80.24%** |
| stl-terrace0.2-chamfer15 | 94.25% | **77.74%** |
| stl-terrace0.2-chamfer22 | 90.15% | **69.33%** |
| stl-terrace0.3-chamfer15 | 96.70% | **91.15%** |
| 平台顶面（0.2/0.3 m） | 99.6~100% | 100% |
| 薄墙 0.15/0.30/0.40 m | 100% | 100% |

### 5.3 离线结论（诚实版）

1. **"矮/低障碍要留成障碍"这条硬要求满足了**：0.2/0.3 m 平台顶面 100% 覆盖；0.15/0.30/0.40 m 薄墙 100% 覆盖。
   代价是 **`th_dist` 必须从上游默认 0.125 收到 0.08**（§3.1）—— 用上游默认值时 0.15 m 薄墙只剩 11.8% 覆盖。
2. **斜面确实变好了，但没到"解决"**：STL 倒角上 patchwork 判地面的比例是 linefit 的 **2.6 倍**（69% vs 27%），
   倒角方位覆盖率从 90~94% 降到 69~80%（**假墙少了一部分，不是没有**）。
   在**长坡**（3 m、10~22°）上两者都做得不好（判地面 7~43%）。
3. **patchwork 有它自己的新毛病**：倒角附近的**平地**上会出现少量假障碍（方位覆盖 8.8~10.8% vs linefit 1.0~1.2%），
   以及 **−0.2 m 坑的下坡倒角**它反而做得比 linefit 差（52.5% vs 8.1% 障碍率）。
4. **CPU 上 patchwork 明显更省**：真实帧中位 0.52 ms vs 1.01 ms（1.94×）。

---

## 6. 整栈 smoke A/B（headless，`mode:=mapping`）

**同一脚本、同一时序、同一场地**：`bench/ground_slot_smoke.sh`（自动做 HOME/DOMAIN/DISPLAY/GAZEBO_MASTER_URI 隔离、
`nav_rviz:=False`、结束时发零 Twist 停车、只录 `/livox/lidar/pointcloud` 供离线复用）。

```bash
bash src/rm_perception/patchwork_ground_segmentation/bench/ground_slot_smoke.sh \
  --ground patchwork --tag pw --domain 91 --out-dir .tmp_ground_ab/smoke_pw
bash src/rm_perception/patchwork_ground_segmentation/bench/ground_slot_smoke.sh \
  --ground linefit   --tag lf --domain 92 --out-dir .tmp_ground_ab/smoke_lf
```

固定组合：`world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox spin_speed:=0.0 nav_rviz:=False`，
脚本化行驶（直行 0.35 m/s 14 s → 停 3 s → 原地左转 0.6 rad/s 8 s → 直行 → 停车），采集窗 105 s。

### 6.1 结果

| 指标 | `ground:=linefit` | `ground:=patchwork` | 谁好 |
|---|---|---|---|
| `/livox/lidar/pointcloud` 频率 | 8.12 Hz | 8.11 Hz | 并列 |
| `/segmentation/obstacle` 频率 | 8.123 Hz | 8.108 Hz | 并列 |
| `/segmentation/ground` 频率 | 8.131 Hz | 8.108 Hz | 并列 |
| `/scan` 频率 | **8.131 Hz** | **8.123 Hz** | 并列（**两者都跟着输入走，没有掉链**） |
| RTF | **0.819** | ⚠️ **未采到**（采集器 `/clock` 订阅用了 RELIABLE，与 gzserver 的 BEST_EFFORT 不兼容 ⇒ 收不到 `/clock`；已修脚本，但没重跑） | 未测 |
| `/scan` 有限回波占比（均值） | 19.05% | 16.06% | — |
| `/scan` 回波数（均值） | 278.5 | 234.8 | — |
| **`/scan` 里 <1.5 m 的近距回波数（均值）** | **91.32**（占有限回波 15.4%） | **34.43**（占 5.5%） | **patchwork：近距假回波少 2.65×** ✅ |
| `/scan` 距离中位（均值） | 6.23 m | 6.70 m | — |
| `/map` 占用格数（末帧） | 6544 | **7756**（+18.5%） | patchwork 留下更多占据 |
| `/map` 已知格数（已知=占用+空闲） | 184 537 | **216 322**（+17.2%） | patchwork |
| CPU：分割器进程（核） | 0.0836 | **0.0562** | **patchwork 省 33%** ✅ |
| CPU：`pointcloud_to_laserscan` | 0.0599 | 0.0611 | 并列 |
| CPU：`slam_toolbox` | 0.0279 | 0.0286 | 并列 |
| CPU：`gzserver` | 0.532 | 0.518 | 并列（两侧仿真负载相同 ⇒ 对比公平） |

**"有没有假墙"**：`/scan` 里 **<1.5 m 的近距回波数**是"地面被判成障碍、在车身周围投出一圈假墙"的客观代理
（`linefit` 的 `max_dist_to_line=0.05` 对坡面/起伏很敏感）。**linefit 91.3 个 vs patchwork 34.4 个**，
与 §5.3 的机制一致。⚠ 但两次跑的**轨迹不完全逐点相同**（Gazebo 非确定性 + 少量 RTF 差异），
所以这是"同脚本、同时序、同场地"的对照，不是逐帧同轨迹对照。

### 6.2 用这次跑出来的**新鲜 RMUC2026 真帧**再做离线逐点 A/B

两次 smoke 各录了 `/livox/lidar/pointcloud`（**0 个 `(0,0,0)` 填充点** —— 插件修复已生效），
各取 60 帧（`--stride 12`，覆盖整段行驶）跑台架：

| 指标 | linefit | Patchwork++ | 谁好 |
|---|---|---|---|
| 逐点一致率 | — | 72.9% / 75.1%（两次跑各自） | — |
| 判为地面的比例 | 15.1% / 17.5% | 36.7% / 39.3% | — |
| CPU / 帧（中位） | 0.541~0.548 ms | **0.098~0.099 ms**（**快 5.5×**） | patchwork ✅ |
| **倾角 22~35° 的障碍率** | **35.7% / 44.3%** | **2.5% / 3.0%** | **patchwork ✅✅（把 97% 的缓坡判成地面 —— 这正是本改动的目的）** |
| 倾角 0~5°（平地）障碍率 | 11.5% / 11.9% | **4.2% / 4.3%** | patchwork ✅（平地更干净） |
| 倾角 35~60° 障碍率 | 80.8% / 86.2% | 45.9% / 66.8% | **linefit 好** ⚠️（patchwork 把一部分陡但不垂直的面判成地面） |
| 倾角 60~90°（近垂直）障碍率 | 84.7% / 83.9% | 83.0% / 84.2% | 并列 ✅（**墙面两者都保住**） |

**按"离地高度带"的方位覆盖率（= 这个高度的特征还会不会出现在 `/scan` 里，p2l 口径）**

| 高度带（离地） | bins | linefit 覆盖 | patchwork 覆盖 | 判定 |
|---|---|---|---|---|
| 0.05~0.10 m | 1256 / 1286 | 93.9% / 95.7% | 84.2% / 92.5% | patchwork 略差 ⚠️ |
| 0.10~0.20 m | 1460 / 1462 | 99.7% / 99.6% | 89.9% / 93.4% | patchwork 略差 ⚠️ |
| **0.20~0.30 m（台阶/平台）** | 968 / 993 | **96.3% / 95.5%** | **93.7% / 94.8%** | **基本持平 ✅（要求满足）** |
| **0.30~0.40 m** | 932 / 926 | 83.3% / 78.1% | 85.3% / 73.9% | **持平 ✅** |
| 0.40~1.00 m | 971 / 972 | 0.0% / 7.3% | 0.0% / 7.3% | 都≈0（**因为 p2l 的 `max_height 0.1` 把 z_sensor>0.1 的点全丢了**，与分割器无关） |
| ≥1.00 m | 874 / 875 | 0% | 0% | 同上 |

⚠ **重要更正**：`0.40~1.00 m` 那一档覆盖率≈0 **不是分割器的锅**，是 `pointcloud_to_laserscan` 的
`max_height: 0.1`（= 离地 0.326 m）把更高的点全过滤掉了。所以"0.4 m 护墙靠 p2l 保住"这个说法
**只在护墙下沿 0.326 m 以内成立**；护墙上部要靠 STVL（`min_obstacle_height 0.0`）那条路。
这一点与本次改动无关，只是本次量出来了。

---

## 7. 结论与建议

### 7.1 该不该把默认从 `linefit` 换成 `patchwork`

**不换。** 理由按重要性排序：

1. **收益是真的，但只在缓坡上，而且没到"解决"**：真实 RMUC2026 帧上，22~35° 缓坡被判成障碍的比例
   从 **35.7~44.3%（linefit）降到 2.5~3.0%（patchwork）** —— 这是本次改动最硬的一条正面证据，方向完全对。
   整栈 smoke 里 `/scan` 的近距假回波也从 **91.3 降到 34.4**（2.65×）。
   但 STL 代理倒角的方位覆盖只从 90~94% 降到 69~80%（**假墙少了一部分，不是没有**），
   3 m 长坡上两者都做不好。RMUC2026 **没有设计长坡**（22~35° 只有 7.5 m²、35~60° 只有 7.4 m²），
   所以"坡面问题"在这个场地上的实际权重本来就低。
2. **有明确的新代价**：0.05~0.20 m 高度带的方位覆盖率从 94~100% 掉到 84~93%（`th_dist 0.08` 下）；
   35~60° 陡面被误判成地面的比例上升（linefit 80.8~86.2% 判障碍 vs patchwork 45.9~66.8%）；
   倒角附近的**平地**上会多出少量假障碍（合成地形实测方位覆盖 8.8~10.8% vs linefit 1.0~1.2%）；
   −0.2 m 坑的下坡倒角上 patchwork 明显更差（52.5% vs 8.1% 障碍率）。
3. **`linefit` 路径是已验证资产，`patchwork` 只做到"离线 + 建图链 smoke"**：没有 `mode:=nav` 整栈、
   没有 P0 回归、没有多场地、参数没调优（§9）。**在没跑完这些之前换默认 = 拿未标定的栈替换已验证的栈。**
4. **唯一没有代价的收益是 CPU**：真实帧 0.098 ms vs 0.541 ms（**5.5×**）、整栈进程 0.056 核 vs 0.084 核（省 33%）。
   但 CPU 不是当前瓶颈（`gzserver` 占 0.52 核是分割器的 6~9 倍）。

### 7.2 什么时候该换（明确的触发条件）

满足**全部**下面几条再考虑把默认改成 `patchwork`：

- [ ] `mode:=nav` 整栈 A/B 跑完，且 `/scan` 率、costmap 更新、`recoveries`、到点结果与 `linefit` 同级或更好；
- [ ] `tools/scripts/regress/nav_smoke_regression.py` 的 P0 回归 PASS；
- [ ] 至少再跑一个场地（RMUL2026 / RMUC）；
- [ ] §5.3 第 3 条那两个新毛病（倒角附近平地的假障碍、下坡倒角）在**实车或目标场地**上确认不影响任务；
- [ ] `sensor_height` 在**实车**上重新标定过。

### 7.3 不换默认时本次改动仍然值得留下的理由

- **A/B 成本极低**（一个词 `ground:=patchwork`），且**默认路径逐字节不变** ⇒ 风险≈0；
- 它是**可诊断的对照臂**：这次就是靠它量出了 `linefit` 在 22~35° 缓坡上的真实缺陷
  （以及 `p2l max_height 0.1` 砍掉护墙上部这个**与分割器无关**的事实）；
- 真机/新场地上如果出现坡道，"换一个词就能试"这件事本身有价值；
- 库里那份 BSD-2 的实现被完整、可复现地固定住了（pin + `VENDORING.md`），
  将来真要用（例如做 COD 2025 那样的 `/patchworkpp/nonground` 接线）不用重新考证许可证。

---

## 8. 回退

| 粒度 | 怎么做 |
|---|---|
| 只回退本次 A/B | 省略 `ground:=`（或写 `ground:=linefit`）——**现有链路一个字节都没变**，不需要改任何文件 |
| 回退整个特性 | `git revert <本 commit>`；或手工删掉：`declare_ground_cmd` 一行 + `ground` condition 一行 + `bringup_patchwork_ground_segmentation_node` 一段 + `_SLOT_ALTERNATIVES` 里那一行；再删 `src/rm_perception/patchwork_ground_segmentation/` 整个包 |
| 只回退 `th_dist` | 把 `config/ground_segmentation_sim.yaml` 的 `th_dist` 改回 `0.125`（= 上游默认）⇒ 坡面更好、**0.15 m 矮墙会从 `/scan` 消失**（§3.1） |
| 上游代码本身 | 不需要"回退"：vendored 源码**没有本地修改**，逐文件 `diff` 应为空（见 §1.3） |

---

## 9. 未验证清单（不要当成已知）

1. **`mode:=nav` / `slam_nav` 整栈没跑过**（本次只做离线 + `mode:=mapping` smoke）。
   代价：`/scan` 内容变化对 `costmap` / STVL / planner 的连锁影响**未测**；`localization:=*` 各槽位与 patchwork 的组合**未测**。
2. **只跑了 RMUC2026 一个场地**；RMUL2026 / RMUC 未跑。
3. **`ground:=patchwork` 没有做过 P0 回归**（`tools/scripts/regress/nav_smoke_regression.py` 口径）。
4. **参数没有调优**：`th_dist` 只扫了 0.05/0.06/0.08/0.10/0.125 五档（合成地形）；
   `uprightness_thr`、`num_min_pts`、`num_sectors_each_zone`、`elevation_thr/flatness_thr` 的初值**都没扫**。
   §5.3 第 3 条那两个新毛病（倒角附近平地的假障碍、下坡倒角）**没有针对性地调过参**。
5. **真机未验**：`sensor_height` 必须按实车重新标定（`0.226` 是仿真值）；真机点云带 intensity 时
   `enable_RNR` 才有意义，**开了会怎样没测**。
6. **`~/segmentation_time_ms` 只是自报的 `estimateGround()` 耗时**，不含 Eigen 矩阵构造、PCL 转换、发布开销；
   整栈 CPU 对比见 §6。
7. **上游 `ros/**` wrapper 没跑过**，所以"我们的节点 vs 上游 wrapper"没有对照。
8. **`th_dist` 与 `max_range` 的交互没扫**：`max_range` 决定 patch 尺寸，patch 尺寸 × `th_dist` 共同决定坡面拟合质量，
   本次只固定 `max_range=20` 扫 `th_dist`。
