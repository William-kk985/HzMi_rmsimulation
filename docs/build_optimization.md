# 构建配置审计：colcon 默认不设 `CMAKE_BUILD_TYPE`（= 部分包整个 -O0），实测 32× 配准损失

> 2026-10-05 · 本文对应改动：新增仓库根目录 `colcon_defaults.yaml`（**唯一**代码/配置改动，
> 没有改任何包的 CMakeLists）。
> 审计范围：`build/*/CMakeCache.txt` + `build/*/CMakeFiles/*/flags.make` 全量 23 个包，
> 外加 `install/`、`colcon`/`colcon-defaults`/`colcon-ros`/`ament_cmake` 的**源码级**取证。
> 隔离口径：所有实测都在**单节点**下做（`ROS_DOMAIN_ID=88`、`ROS_LOG_DIR` 落在工作区内），
> **没有**启 Gazebo / nav2 / LIO。

---

## 0. TL;DR

1. **现象确认（但范围比最初报告的小）**：本工作区 `colcon build` 不传 `CMAKE_BUILD_TYPE`
   ⇒ `CMakeCache.txt` 里为空 ⇒ **6 个包**的编译命令里**一个 `-O` 都没有**（= `-O0`）。
2. **重要更正**：**不是所有"热"包都没优化**。审计发现 9 个包靠**包内硬编码**已经拿到了 `-O3`
   （`add_compile_options(-O3)` / `set(CMAKE_CXX_FLAGS "... -O3")` /
   `set(CMAKE_BUILD_TYPE Release)`）。其中 `set(CMAKE_BUILD_TYPE Release)` 那 5 个包
   **`CMakeCache.txt` 里依然是空的**——只看 cache 会误判成"没优化"。
   *（这正是"一个包的结论不能外推全工作区"的地方。）*
3. 真正在 `-O0` 且在热路径上的只有 6 个：
   `gicp_registration`、`icp_registration`、`lio_tf_adapter`、`fake_vel_transform`、
   `livox_ros_driver2`、`ros2_livox_simulation`（Gazebo 雷达插件 —— **仿真里常驻**）。
4. **实测（同机、同数据、未改一行业务代码，只换构建口径）**：

   | 台子 | `-O0`（改动前） | `Release`（改动后） | 倍数 | fitness / 位姿 |
   |---|---|---|---|---|
   | 台架 `gicp_backend_bench`，PCL GICP | median **369.9 ms** | median **11.3 ms** | **32.7×** | 0.00122 m² → 0.00122 m²（一致） |
   | 台架 `gicp_backend_bench`，small_gicp t=4 | median **254.1 ms** | median **2.3 ms** | **110×** | 0.00122 m²（一致） |
   | 真节点 `gicp_registration_node`，PCL | align **480~496 ms** | align **10.7~10.9 ms** | ≈45× | `0.00123 m²` / `x=-0.150 y=0.150 yaw=-0.01°` 逐字一致 |
   | 真节点，small_gicp | align **236~270 ms** | align **2.3~3.1 ms** | ≈90× | `0.00123 m²` / `x=-0.149 y=0.151` 一致 |

5. **落地方式**：仓库根目录提交 `colcon_defaults.yaml`（`build.cmake-args: [-DCMAKE_BUILD_TYPE=Release]`），
   于是本仓库文档里惯用的 `colcon build --symlink-install` **自动**带上 Release，不必再靠"记得加参数"。
6. **全量重编验证**：`colcon build --symlink-install` → **22/22 包成功、0 失败、0 编译器告警、27.7 s**
   （大多数包旗标不变 ⇒ 不重编，只有旗标变化的包真正重编）。

---

## 1. 审计

### 1.1 审计口径（两个必须先说清的坑，否则结论是错的）

**坑 ①：`CMakeCache.txt` 里 `CMAKE_BUILD_TYPE` 为空 ≠ 没有优化。**

反例 `FAST_LIO`（`src/rm_localization/FAST_LIO/CMakeLists.txt:4-6`）：

```cmake
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # ← 普通变量，**不写 cache**
endif()
```

- `build/fast_lio/CMakeCache.txt` 里仍然是 `CMAKE_BUILD_TYPE:STRING=`（空）；
- 但 CMake 生成器用的是**普通变量**，于是 `build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make` 里
  真实旗标是 `... -fexceptions -fopenmp -O3 -DNDEBUG ...`（`-O3 -DNDEBUG` 正是 CMake 对 GNU 的
  `CMAKE_CXX_FLAGS_RELEASE` 默认值）。

⇒ **唯一可信的口径是 `flags.make`**，本文所有"有没有 `-O`"都以它为准：

```bash
grep -oh -- '-O[0-9s]' build/<pkg>/CMakeFiles/*/flags.make | sort -u
```

**坑 ②：`-O3` 不止一种来源。** 本仓库里三种都有：

| 来源 | 例子 | cache 里 BT |
|---|---|---|
| `add_compile_options(-std=c++17 -O3)` | `pointcloud_to_laserscan`、`imu_complementary_filter` | 空 |
| `add_definitions(-std=c++17 -O3)` | `linefit_ground_segmentation{,_ros}` | 空 |
| `set(CMAKE_CXX_FLAGS "-std=c++14 -O3")` | `FAST_LIO`、`point_lio` | 空 |
| `set(CMAKE_BUILD_TYPE Release)`（非 cache） | `slam_toolbox`、`teb_local_planner`、`costmap_converter` | 空 |
| `-DCMAKE_BUILD_TYPE=Release`（cache） | `cartographer`、`cartographer_ros` | `Release` |
| **什么都没有** | 见 §1.3 六个包 | 空 |

> 附带结论：`set(CMAKE_CXX_FLAGS "... -O3")` 这种写法在加了 Release 之后**不会降级**——
> 编译行是 `CMAKE_CXX_FLAGS` + `CMAKE_CXX_FLAGS_RELEASE` 拼接，两者都是 `-O3`，
> 后出现的同优先级旗标取值相同（实测 `fast_lio` 重配后旗标逐字未变、未重编，见 §5）。

### 1.2 审计总表（改动前 → 改动后，23 个包全量）

`-O` 列 = `flags.make` 里出现过的优化旗标（`NONE` = 一个都没有 ⇒ `-O0`）；
`SRC` = `CMakeCache.txt` 的 `CMAKE_HOME_DIRECTORY`（colcon 实际用的源码目录，全部在 `src/**`；
`third_party/**` 有 `COLCON_IGNORE`，colcon 不构建它们，只作参考代码）。

| 包 | BT(cache) 前→后 | `-O` 前→后 | `-DNDEBUG` 前→后 | SRC | 归类 |
|---|---|---|---|---|---|
| `cartographer` | `Release`→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_localization/cartographer` | 建图槽（本来就好） |
| `cartographer_ros` | `Release`→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_localization/cartographer_ros` | 建图槽（本来就好） |
| **`gicp_registration`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_localization/gicp_registration` | **热：GICP 配准（localization:=gicp）** |
| **`icp_registration`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_localization/icp_registration` | **热：ICP 配准（localization:=icp）** |
| **`ros2_livox_simulation`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_simulation/livox_laser_simulation_RO2` | **热：Gazebo 雷达插件（仿真常驻，出 30k 点/帧）** |
| **`livox_ros_driver2`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_driver/livox_ros_driver2/src` | **热：真机雷达驱动 + `CustomMsg` 序列化** |
| **`fake_vel_transform`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_navigation/fake_vel_transform` | 常驻（TF/速度变换，计算量小；带 `-Werror`） |
| **`lio_tf_adapter`** | ``→`Release` | **`NONE`→`-O3`** | **NO→yes** | `src/rm_localization/lio_tf_adapter` | 常驻（TF 适配，计算量小） |
| `fast_lio` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_localization/FAST_LIO` | 热（LIO 槽）但**本来就 -O3** |
| `point_lio` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_localization/point_lio` | 同上 |
| `linefit_ground_segmentation` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation` | 热（地面分割）但本来就 `-O3` |
| `linefit_ground_segmentation_ros` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | 同上 `.../linefit_ground_segmentation_ros` | 同上 |
| `pointcloud_to_laserscan` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/rm_perception/pointcloud_to_laserscan` | 热但本来就 `-O3` |
| `imu_complementary_filter` | ``→`Release` | `-O3`→`-O3` | NO→**yes** | `src/rm_perception/imu_complementary_filter` | 热但本来就 `-O3` |
| `slam_toolbox` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_localization/slam_toolbox` | 建图槽（本来就 Release） |
| `teb_local_planner` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_navigation/teb_local_planner/teb_local_planner` | 规划（本来就 Release） |
| `costmap_converter` | ``→`Release` | `-O3`→`-O3` | yes→yes | `src/rm_navigation/costmap_converter/costmap_converter` | 同上 |
| `teb_msgs` | ``→`Release` | `NONE`→**`-O3`** | NO→**yes** | `src/rm_navigation/teb_local_planner/teb_msgs` | 消息序列化代码（30 Hz 控制环话题） |
| `costmap_converter_msgs` | ``→`Release` | `NONE`→**`-O3`** | NO→**yes** | `src/rm_navigation/costmap_converter/costmap_converter_msgs` | 消息序列化代码 |
| `rm_nav_bringup` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/rm_nav_bringup` | **纯安装包**（只有 launch/params，无编译目标）⇒ 无意义 |
| `rm_navigation` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/rm_navigation/rm_navigation` | 纯 Python/params ⇒ 无意义 |
| `hzmi_rm_simulation` | ``→`Release` | `NONE`→`NONE` | NO→NO | `src/rm_simulation/hzmi_rm_simulation` | 纯 URDF/world ⇒ 无意义 |
| `cartographer_ros_msgs` | ``→``（未动） | `NONE` | NO | `src/rm_localization/cartographer_ros/cartographer_ros_msgs`（**已不存在**） | ⚠️ 陈旧残留，见 §1.5 |

**不在上表里的东西：**

- **nav2 全家（`nav2_controller` / `nav2_planner` / `nav2_bt_navigator` / `nav2_costmap_2d` …）来自 apt**：
  `ros2 pkg prefix nav2_controller` = `/opt/ros/humble` ⇒ 官方 Release 预编译，**不是**本次问题的一部分
  （`third_party/nav2/**` 只是参考代码，`COLCON_IGNORE` 不构建）。
- `cartographer` 是 C++ 库（工作区自编）；`cartographer_ros` 是 `ros.ament_cmake`，两者 cache 里本来就是 `Release`。

### 1.3 "没有 `-O` 且在热路径上"的判定

热路径的判定依据是本仓库的实际装配（`src/rm_nav_bringup/launch/bringup_sim.launch.py`）：

| 节点 | 何时在跑 | 改动前的构建口径 |
|---|---|---|
| `ros2_livox_simulation`（Gazebo 插件 `libros2_livox.so`，由 `urdf/mid360.xacro:58` 加载） | **仿真常驻** | **`-O0`** ⇒ 已修 |
| `fake_vel_transform`（:508） | **常驻** | **`-O0`** ⇒ 已修 |
| `lio_tf_adapter`（:524，`lio!=none/cartographer` 时） | **常驻** | **`-O0`** ⇒ 已修 |
| `imu_complementary_filter`(:255) / `linefit`(:267) / `pointcloud_to_laserscan`(:277) | 常驻（感知） | 已经 `-O3`（包内硬编码） |
| `fast_lio`(:295) / `point_lio`(:326) | 按 `lio:=` 选一个 | 已经 `-O3` |
| `slam_toolbox`(:365) | 按 `slam:=` | 已经 Release |
| `icp_registration`(:454) / `gicp_registration`(:473) | 按 `localization:=icp` / `:=gicp` | **`-O0`** ⇒ 已修 |
| `livox_ros_driver2` | 真机（仿真里由插件替代） | **`-O0`** ⇒ 已修 |
| nav2 全家 | 常驻 | apt 预编译 Release |

⇒ **仿真里"常驻且原来 `-O0`"的只有 3 个**：Gazebo 雷达插件、`fake_vel_transform`、`lio_tf_adapter`；
再加按槽启用的 `gicp`/`icp`。这也解释了为什么这条线索以前没被发现：**最常被怀疑的 LIO/感知包本来就是 `-O3`**。

### 1.4 `-O` 到底从哪来（源码级证据）

```
$ grep -rn --include=CMakeLists.txt -E '\-O[0-9s]\b|CMAKE_BUILD_TYPE' src/ | sed 's|/CMakeLists.txt:|: |'
src/rm_perception/pointcloud_to_laserscan: 4:add_compile_options(-std=c++17 -O3)
src/rm_perception/imu_complementary_filter: 7:add_compile_options(-std=c++17 -O3)
src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation: 6:add_definitions(-std=c++17 -O3)
src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros: 8:add_definitions(-std=c++17 -O3)
src/rm_navigation/teb_local_planner/teb_local_planner: 5:set(CMAKE_BUILD_TYPE Release)
src/rm_navigation/costmap_converter/costmap_converter: 5:set(CMAKE_BUILD_TYPE Release)
src/rm_localization/point_lio: 4:if(NOT CMAKE_BUILD_TYPE) / 5:set(CMAKE_BUILD_TYPE Release) / 10:set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
src/rm_localization/FAST_LIO: 4:if(NOT CMAKE_BUILD_TYPE) / 5:set(CMAKE_BUILD_TYPE Release) / 10:set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
src/rm_localization/slam_toolbox: 4:set(CMAKE_BUILD_TYPE Release)
src/rm_localization/slam_toolbox/lib/karto_sdk: 7:set(CMAKE_BUILD_TYPE Release)
src/rm_localization/FAST_LIO/include/ikd-Tree: 6:set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
```

### 1.5 顺手发现的两个"脏"点（**未**在本次改动里处理）

1. **`cartographer_ros_msgs` 是陈旧残留**：`build/`、`install/` 里都有它（2026-09-09 的产物，
   `-O0`），但源码目录 `src/rm_localization/cartographer_ros/cartographer_ros_msgs` **已经不存在**
   （`colcon list` 里也没有这个名字）⇒ **无法重编**，也不该重编：
   真正的包由 **apt 提供**（`/opt/ros/humble/share/cartographer_ros_msgs`，Release）。
   它的 `install/cartographer_ros_msgs/lib/` 是**空的**（`ls` 无输出）⇒ 运行时 `.so` 走 apt，
   **不构成性能或正确性风险**；只是 `install/` 里多一份旧 metadata。
   可选清理（**本仓库不自动做**，需要用户拍板）：
   ```bash
   rm -rf build/cartographer_ros_msgs install/cartographer_ros_msgs
   ```
2. `install/` 里有一套 `--symlink-install` 的符号链接（可执行文件指向 `build/**`），
   这解释了为什么"重编即生效"（不需要重新 `--symlink-install` 一次）。

---

## 2. 实测（同机 A/B，未改一行业务代码）

### 2.1 测量口径

* **机器**：i7-14700HX（28 线程），Ubuntu 22.04 + ROS 2 Humble，PCL 1.12.1，cmake 3.22.1。
* **隔离**：只有**单个节点**在跑。`ROS_DOMAIN_ID=88`（非默认域），`ROS_LOG_DIR` 落在
  `.tmp_cache/build_opt/roslog_*`（工作区内）。**没有** Gazebo / nav2 / LIO / RViz。
* **数据/参数**：与 `docs/gicp_backend_small_gicp.md` §4.0 完全同一口径
  （`src/rm_nav_bringup/PCD/RMUL2026.pcd` → target 12450 点 @leaf 0.10；合成扫描 4000 点 →
  source 3701/3710 点 @leaf 0.05；注入 `(+0.15, −0.15, 0) m` 偏移；`max_corr 1.5`、`iters 16`）。
* **A/B 之间只差一个变量**：`-DCMAKE_BUILD_TYPE=Release`（业务代码 = 同一个 commit）。
* **负载（诚实披露）**：改动前的台架 run 在 `load≈2.1~2.2`，改动后在 `load≈6.2`（**更高**）。
  ⇒ 下面的加速比**不是**"后测时机器更闲"造成的，反而偏保守。
* **复现命令**：
  ```bash
  source /opt/ros/humble/setup.bash && source install/setup.bash
  install/gicp_registration/lib/gicp_registration/gicp_backend_bench \
    --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd --backend both --frames 12
  # 真节点：
  bash .tmp_cache/gicp_ab/ab_run.sh <TAG> 14 <pcl|small_gicp> tfon \
    -p initial_pose:="[-0.15,0.15,0.0,0.0,0.0,0.0]"     # 注意：必须全写成浮点，见 §8.4
  ```

### 2.2 台架 `gicp_backend_bench`（12 帧，同一批帧）

| 后端 | 首帧 | 稳态 median | mean | p95 | max | fitness(median) | 最终位姿误差 |
|---|---|---|---|---|---|---|---|
| `pcl` **-O0** | 1427.7 ms | **369.9 ms** | 429.0 | 714.2 | 714.2 | 0.00122 m² | 0.0023 m / 0.028° |
| `pcl` **Release** | 37.5 ms | **11.3 ms** | 11.6 | 14.0 | 14.0 | 0.00122 m² | 0.0023 m / 0.028° |
| `small_gicp` t=4 **-O0** | 253.7 ms | **254.1 ms** | 237.8 | 278.2 | 278.2 | 0.00122 m² | 0.0023 m / 0.027° |
| `small_gicp` t=4 **Release** | 2.4 ms | **2.3 ms** | 2.2 | 2.4 | 2.4 | 0.00122 m² | 0.0023 m / 0.027° |

倍数：**PCL 32.7×**、**small_gicp 110×**（稳态 median）。附带：

* PCL 首帧（含目标协方差预计算）1427.7 → 37.5 ms（**38×**）；
* small_gicp 目标侧一次性预处理（转点云+建 KdTree+估协方差，target 12450 点）
  **287.9 → 5.3 ms（54×）**；
* **坏初值容忍度不变**：`+0.5 m/+10°` PCL 2744.9 → 27.1 ms（最终误差 5 mm/0.06° 不变）；
  `+1.0 m/+20°` PCL 5278.0 → 37.4 ms（8 mm/0.08° 不变）；
  small_gicp `+1.0 m/+20°` 631.0 → 4.3 ms（3 mm/0.04° 不变）。

### 2.3 真节点 `gicp_registration_node`（node-alone，14 s，探针 10 Hz 发合成点云）

| 后端 / 配置 | `[status] align` | `/tf` Hz | `~/pose` 条数（收到 127 帧点云） | `~/fitness_score` | `~/converged` true | 最终 map→odom |
|---|---|---|---|---|---|---|
| `pcl` **-O0** | **480.2 / 495.5 / 483.4 ms** | 50.001 | **30** | 0.00123 m² | 30/30 = 100% | `x=-0.150 y=0.150 yaw=-0.01°` |
| `pcl` **Release** | **10.7 / 10.9 / 10.9 ms** | 49.999 | **127** | 0.00123 m² | 127/127 = 100% | `x=-0.150 y=0.150 yaw=-0.01°`（逐字一致） |
| `small_gicp` **-O0** | **270.4 / 235.9 / 261.6 ms** | 49.997 | **53** | 0.00123 m² | 53/53 = 100% | `x=-0.149 y=0.151 yaw=-0.01°` |
| `small_gicp` **Release** | **3.1 / 2.3 / 3.1 ms** | 49.999 | **127** | 0.00123 m² | 113/127 = 89%↓ | `x=-0.149 y=0.151 yaw=-0.01°`（一致） |
| `small_gicp` Release（**复跑对照**） | 3.3 / 4.0 / 4.0 ms | 50.000 | 127 | 0.00123 m² | 117/127 = 92% | 同上 |

**三条结论：**

1. **align 快 45×（PCL）/ 90×（small_gicp），而 `/tf` 频率完全没变（50.00 Hz）**——
   这与 `docs/gicp_backend_small_gicp.md` §4.2 的判断一致：`/tf` 由**定时器**驱动，
   与 align 耗时解耦；所以**不能用 `/tf` Hz 衡量本次收益**，要看 align 与 CPU。
2. **fitness / 采纳判据 / 最终 TF 数值逐字一致** ⇒ 这是纯粹的性能改动，不是行为改动。
3. **次要发现（原来没人量过）**：`-O0` 时 align ≈ 400 ms 而点云 10 Hz（周期 100 ms）
   ⇒ **节点只能吃下 30/127 = 24% 的点云**（其余在回调组里排队后被 `KEEP_LAST` 丢掉）；
   Release 后变成 **127/127 = 100%**。也就是说 `-O0` 不只是"慢"，它还**静默改变了数据流**
   （定位更新率 2.1 Hz → 9.1 Hz）。这会让"GICP 定位看起来还行"的现场结论变得不可信。

### 2.4 small_gicp 的 `~/converged` 抖动**不是**本次改动引入的（控制实验）

现象：Release 下 `~/converged` 从 100% 掉到 89%（113/127）。做了两个控制实验：

1. **同一个 Release 二进制、同一批帧、重跑台架** ⇒ 上一轮 `converged=0` 的 frame 7/8
   这一轮变成 `converged=1`，median 仍是 2.3 ms，`converged 12/12`；
2. **同一个 Release 节点重跑** ⇒ `~/converged` 117/127 = 92%（第一次 113/127 = 89%）。

⇒ 这是 **run-to-run 抖动**（small_gicp 的 LM 优化器在极小点附近提前 break，
OpenMP 归约顺序影响最后几个 bit），**早在 `-O0` 就有**：本仓库
`docs/gicp_backend_small_gicp.md` §6.2 记录 `-O0` 时节点里就是 **92~96%**，
`src/rm_localization/gicp_registration/src/registration_backend.cpp:190-198` 也把这条行为差异写在注释里
（换 `GaussNewtonOptimizer` 可让 `~/converged` 100%）。
**位姿、fitness、采纳后的 TF 全部一致**（`x=-0.149 y=0.151`）⇒ 只是健康话题的抖动，
不是精度回归。默认后端是 `pcl`（100% converged），完全不受影响。

---

## 3. 策略：为什么用 `colcon_defaults.yaml`（而不是记一条命令）

### 3.1 colcon / ament 到底怎么处理 build type（源码取证）

| 事实 | 证据 |
|---|---|
| **colcon-cmake 不会替你设 build type**，只把你给的 `--cmake-args` 原样转给 cmake（再加 `-DCMAKE_INSTALL_PREFIX`） | `/usr/lib/python3/dist-packages/colcon_cmake/task/cmake/build.py:144-146` |
| colcon-cmake 里唯一和 build type 有关的逻辑 `_get_configuration()` **只对多配置生成器（VS / Ninja Multi-Config）** 有意义，且它只是从 `--cmake-args` 或 cache 里读出来、兜底 `Release` | 同上 `:249-262`（`--config` 只在 `multi_configuration_generator` 分支用，`:238-239`） |
| **ament_cmake 也没有任何 build type 默认值** | `grep -rn CMAKE_BUILD_TYPE /opt/ros/humble/share/ament_cmake*/cmake/` → **无输出** |
| ⇒ 不写配置时 cache 里就是 `CMAKE_BUILD_TYPE:STRING=`（空），`flags.make` 里没有任何 `-O` | 本文 §1.2 审计 |
| **colcon-defaults 插件**：全局文件 = `$COLCON_DEFAULTS_FILE`，未设则 `$COLCON_HOME/defaults.yaml`；**工作区文件 = 当前目录的 `colcon_defaults.yaml`**，深度合并时**工作区覆盖全局** | `colcon_defaults/argument_parser/defaults.py:28-34`（`WORKSPACE_DEFAULTS_FILE='colcon_defaults.yaml'`）、`:78-80`（全局路径）、`:141-146`（`_deep_update(data, local_data)`，local 在后 = 优先）、`:144`（`Path('colcon_defaults.yaml')` = **相对 CWD**） |
| `--symlink-install` 是**追加**（不是替换）cmake 参数 ⇒ 本仓库惯用命令仍吃默认值 | `colcon_ros/task/ament_cmake/build.py:50-53`：`args.cmake_args.append('-DAMENT_CMAKE_SYMLINK_INSTALL=1')` |
| colcon 只在 `--cmake-args` **与上次不同**时才重跑 cmake ⇒ `build/<pkg>/cmake_args.last` 可以直接验证"默认值被吃到了" | `colcon_cmake/task/cmake/build.py:134-138` |

### 3.2 三个选项对比

| 方案 | 可复现性 | 评价 |
|---|---|---|
| 每次手敲 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | ❌ 靠记忆 | **就是本次问题的成因**（有人漏敲 ⇒ 静默 -O0） |
| 全局 `~/.colcon/defaults.yaml` 或 `COLCON_DEFAULTS_FILE` | ⚠️ 机器级、仓库外 | 对"克隆即可构建"不可复现；换机器/换人即失效 |
| **仓库根 `colcon_defaults.yaml`（本次采用）** | ✅ 随 git 提交 | 与"克隆即可构建"一致；`colcon build --symlink-install` 自动生效；改了有 diff 可 review |

### 3.3 本次提交的内容与"生效"的证据

新增 `colcon_defaults.yaml`（仓库根，唯一配置改动）：

```yaml
build:
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release
```

生效证据（构建后立刻可查）：

```bash
$ cat build/lio_tf_adapter/cmake_args.last
['-DCMAKE_BUILD_TYPE=Release', '-DAMENT_CMAKE_SYMLINK_INSTALL=1']
$ grep -m1 '^CMAKE_BUILD_TYPE:' build/lio_tf_adapter/CMakeCache.txt
CMAKE_BUILD_TYPE:STRING=Release
$ grep CXX_FLAGS build/lio_tf_adapter/CMakeFiles/*/flags.make
CXX_FLAGS = -O3 -DNDEBUG -Wall -Wextra -Wpedantic -std=gnu++17
```

> 注意第二个参数：它来自 `colcon build --symlink-install`（colcon_ros 追加）——
> 这同时证明了**文档里惯用的那条命令确实吃到了本文件的默认值**。

`-O3 -DNDEBUG` 里的 `-O3` 是 CMake 对 GNU 的 Release 默认值，可在本工作区直接核对：
`grep CMAKE_CXX_FLAGS_RELEASE build/cartographer/CMakeCache.txt` → `-O3 -DNDEBUG`。

### 3.4 使用注意（三条坑，都已写进 `colcon_defaults.yaml` 的注释）

1. **必须在仓库根目录执行 `colcon build`**（`defaults.py:144` 是相对 CWD 的路径）
   ⇒ 在子目录里 build 会**静默**退回 `-O0`。
2. **命令行显式 `--cmake-args ...` 会整体替换默认值**（argparse 语义）
   ⇒ 要加别的 `-D` 时，请自己把 `-DCMAKE_BUILD_TYPE=Release` 一起写上。
   （`--symlink-install` 不在此列：它是**追加**。）
3. **换口径要显式覆盖**，不要改本文件：
   ```bash
   # 要调试符号：
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   # 完全回到旧口径（-O0）做对照实验：
   COLCON_DEFAULTS_FILE=/dev/null colcon build --symlink-install \
     --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=
   ```

---

## 4. `-DNDEBUG` 副作用（assert 被关掉）

`Release` 对 GNU 的默认旗标是 `-O3 -DNDEBUG` ⇒ **`assert()` 变成空语句**。
逐包查了会**新**吃到 `-DNDEBUG` 的包（改动前没有 `-DNDEBUG` 的包）：

| 包 | `assert(` 出现次数 | 结论 |
|---|---|---|
| `gicp_registration` | 0 | 无影响 |
| `icp_registration` | 0 | 无影响 |
| `lio_tf_adapter` | 0 | 无影响 |
| `fake_vel_transform` | 0 | 无影响 |
| `ros2_livox_simulation` | 0 | 无影响 |
| `linefit_ground_segmentation{,_ros}` | 0 | 无影响 |
| `pointcloud_to_laserscan` | 0 | 无影响 |
| `imu_complementary_filter` | 0 | 无影响 |
| `livox_ros_driver2` | 3 | **全部在 vendored `3rdparty/rapidjson/rapidjson.h`**（`RAPIDJSON_ASSERT`，只用于 JSON 配置解析的内部不变量）；rapidjson 官方就推荐 Release+`NDEBUG`，且**不在每帧热路径上** |
| `teb_msgs` / `costmap_converter_msgs` | 0（生成代码） | 无影响 |

补充说明：

* 工作区**本来就已经是混合口径**：`cartographer`、`cartographer_ros`、`fast_lio`、`point_lio`、
  `slam_toolbox`、`teb_local_planner`、`costmap_converter` 早就在 `-DNDEBUG` 下构建；
  apt 的 nav2 全家（`/opt/ros/humble`）也是 Release。⇒ 本次改动是**消除不一致**，不是引入新风险。
* 本仓库自己的代码里没有依赖 `assert` 做**运行时输入校验**的地方（上表 0 命中）；
  输入校验走的是显式 `if + return/throw`（例如 `icp_registration` 的 `Invalid pcd path` 就是
  `std::runtime_error`，**不是** assert ⇒ `-DNDEBUG` 下依然会拦）。
* 想彻底避开这一条：把 `colcon_defaults.yaml` 换成
  `-DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g"`（会保留 assert、
  拿到 `-O2` 与符号，但比 `-O3` 略慢，且**本次实测的 32× 是用 `-O3` 得到的**）。

---

## 5. 逐包 rebuild 结果（时间 / 成功 / 产物大小）

### 5.1 第一轮：只重编"改动前 `-O0`"的包（`colcon build --symlink-install --packages-select ...`）

| 包 | 耗时 | 结果 | 备注 |
|---|---|---|---|
| `costmap_converter_msgs` | 2.7 s | ✅ | 消息包（顺带补 `-O3`） |
| `teb_msgs` | 3.5 s | ✅ | 消息包（顺带补 `-O3`） |
| `lio_tf_adapter` | 6.5 s | ✅ | 另一次单独构建（用来先验证 `colcon_defaults.yaml` 生效） |
| `livox_ros_driver2` | 11.1 s | ✅ | 驱动 + `livox_ros_driver2`/`livox_interfaces2` 序列化代码 |
| `fake_vel_transform` | 12.4 s | ✅ | **带 `-Wall -Werror`，`-O3` 下 0 新告警** |
| `ros2_livox_simulation` | 15.8 s | ✅ | Gazebo 插件 |
| `icp_registration` | 24.3 s | ✅ | |
| `gicp_registration` | 31.9 s | ✅ | 含 vendored small_gicp |
| **合计（并行口径）** | **35.8 s wall / 2m44s user** | ✅ 7/7 | 与上面那 6.5 s 的那次合计 8 包 |

> `stderr` 里出现的三行都是**既有噪音**，不是新告警：
> `ament_auto_package` 的 "headers install destination … Kilted" 提示（4 个包）、
> `fast_lio` 的 CMake `CMP0074/PCL_ROOT` dev 警告。
> **全量日志里 `grep -c 'warning:'` = 0**（含 `-Werror` 包）。

### 5.2 第二轮：全量 `colcon build --symlink-install`（验证用户将执行的那条命令）

```
Summary: 22 packages finished [27.7 s]
  3 packages had stderr output: fast_lio linefit_ground_segmentation pointcloud_to_laserscan   ← 全是既有提示
```

* **22/22 成功、0 失败、0 编译器告警**；
* 为什么这么快：大多数包的旗标**逐字未变** ⇒ make 判定无需重编。
  真正重编的只有旗标变化的那些（`linefit*` 9.5/13.5 s、`pointcloud_to_laserscan` 27.5 s、
  `imu_complementary_filter` 20.1 s —— 它们只是多了 `-DNDEBUG`），
  以及 `fast_lio` 5.8 s / `point_lio` 4.0 s / `slam_toolbox` 7.2 s / `cartographer_ros` 3.9 s（重配+增量）。

### 5.3 产物大小（同一 commit，只换构建口径）

| 产物 | `-O0` | `Release` | Δ |
|---|---|---|---|
| `build/gicp_registration/libgicp_registration.so` | 21 869 656 | 2 104 632 | **−90.4%** |
| `build/gicp_registration/gicp_registration_node` | 204 752 | 57 208 | −72.1% |
| `build/gicp_registration/gicp_backend_bench` | 686 816 | 53 928 | −92.1% |
| `build/gicp_registration/small_gicp/libsmall_gicp.so` | 7 465 208 | 167 856 | −97.8% |
| `build/icp_registration/libicp_registration.so` | 17 281 912 | 1 805 176 | −89.6% |
| `build/lio_tf_adapter/lio_tf_adapter_node` | 3 099 728 | 640 992 | −79.3% |
| `build/fake_vel_transform/libfake_vel_transform.so` | 9 257 024 | 1 509 384 | −83.7% |
| `build/livox_ros_driver2/liblivox_ros_driver2.so` | 3 128 904 | 616 040 | −80.3% |
| `build/livox_ros_driver2/liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so` | 64 024 | 25 368 | −60.4% |
| `build/ros2_livox_simulation/libros2_livox.so` | 3 541 848 | 769 696 | −78.3% |
| `build/teb_msgs/libteb_msgs__rosidl_typesupport_fastrtps_cpp.so` | 108 224 | 37 728 | −65.1% |

（`-O0` 的 `.so` 之所以巨大，是因为没有内联/没有死代码消除 ⇒ 模板实例化全留在二进制里。
体积本身不是目标，但 −80%~−98% 的幅度说明"以前确实一行都没优化"。）

### 5.4 节点级冒烟（重建后的二进制真的能跑）

| 节点 | 方式 | 结果 |
|---|---|---|
| `gicp_registration_node`（pcl / small_gicp） | 真数据 node-alone，14 s ×4 次 | ✅ 见 §2.3 |
| `fake_vel_transform_node` | `ros2 run` + 本包 params，5 s | ✅ `Start FakeVelTransform!` → 收到 SIGINT 干净退出 |
| `lio_tf_adapter_node` | 同上 | ✅ `订阅 /odom -> 广播 TF odom -> base_link` → 干净退出 |
| `icp_registration_node` | 同上 + `-p pcd_path:=…RMUL2026.pcd` | ✅ `pcd point size: 12450` / `icp_registration initialized` → 干净退出 |
| `ros2_livox_simulation`（Gazebo 插件） | **无法单独起**（要 Gazebo 加载） | ⏳ 未验证，见 §7 |

---

## 6. `small_gicp` 的 `FORCE` 与本包 backup/restore 的相容性（**已确认不打架**）

上游 `third_party/small_gicp/CMakeLists.txt:7-9`：

```cmake
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "..." FORCE)   # ← FORCE 写进 cache，全局生效
endif()
```

`gicp_registration` 用 `add_subdirectory` 引入它（为了让 RMW 之外的依赖无需 `sudo make install`），
所以必须防"被第三方 FORCE 掉构建口径"。本包在
`src/rm_localization/gicp_registration/CMakeLists.txt:35-55` 已经做了 backup/restore：

```cmake
set(_gicp_registration_build_type_backup "${CMAKE_BUILD_TYPE}")
add_subdirectory("${SMALL_GICP_VENDOR_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/small_gicp")
set(CMAKE_BUILD_TYPE "${_gicp_registration_build_type_backup}"
  CACHE STRING "Build type（本包已还原成进入 add_subdirectory 之前的值；见上方注释）" FORCE)
```

**加了全局 `Release` 之后的行为**：进入时 cache 已经是 `Release` ⇒ `_backup = "Release"`
⇒ small_gicp 的 `if(NOT CMAKE_BUILD_TYPE)` **不再成立**（不触发 FORCE）
⇒ 出栈时还原成 `Release`（同一个值）⇒ **本包与 vendored 库都按 Release 编译，没有任何冲突**。

反过来，这段 backup/restore 的存在保证了：即使将来有人把 `colcon_defaults.yaml` 删掉，
`gicp_registration` 也**不会**被 small_gicp 静默改成 `-O3`（回到 `-O0` 基线），
A/B 仍然是在同一口径下做的。**两者设计目标一致，不打架；本次没有改这个文件。**

---

## 7. 无法验证 / 未验证清单（**请勿当成已验证**）

1. **RTF / `Control loop missed its desired rate of 30 Hz` / 整栈 CPU**：
   本次**没有**启 Gazebo/nav2/LIO（按任务要求只做单节点）。
   ⇒ **"RTF 会从 0.72 涨到多少"完全没有实测数据**。能确定的只是"少算了"：
   `docs/localization_slots.md:689` 记录的 RTF≈0.72 那次，算力构成里明确含 "GICP 10 Hz"，
   而那时 GICP 每帧 400~480 ms（Release 后 10~11 ms）；
   另外仿真常驻的 Gazebo 雷达插件与 `fake_vel_transform`/`lio_tf_adapter` 也从 `-O0` 变 `-O3`。
2. **`ros2_livox_simulation`（Gazebo 插件）的运行时收益**：无法脱离 Gazebo 测；
   只有"旗标 `-O0`→`-O3`、`libros2_livox.so` 3.54 MB→0.77 MB（−78%）"这两条静态证据。
3. **`livox_ros_driver2` / `CustomMsg` 序列化的收益**：`CustomMsg`（30k 点/帧、10 Hz）的
   fastrtps 序列化代码就在这个包里（`liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so`，
   −60%），但**没有实测吞吐**。真机上才用得到这个驱动（仿真里由插件直接发同类型消息）。
4. **`-DNDEBUG` 下的长时间行为**：只做了静态 assert 审计（§4），**没有**跑长时压力测试。
5. **`icp_registration` 的精度/行为**：只做了"能起来"的冒烟（§5.4），**没有**做 A/B 数值对拍。
6. **`cartographer`/`slam_toolbox` 路径完全没动**（它们本来就是 Release），因此建图相关结论不受影响。
7. **未做**：`teb_local_planner`/`costmap_converter`/`nav2` 的进一步调优（不在本次范围）。

---

## 8. 用户下一步

### 8.1 重编（二选一）

```bash
cd /home/weicheng/HzMi_rmsimulation          # ⚠ 必须在仓库根目录（见 §3.4-①）
source /opt/ros/humble/setup.bash

# ① 推荐：全量（本次已实测 22/22 成功、27.7 s；顺手把消息包与感知包的口径统一）
colcon build --symlink-install

# ② 只重编本次真正变化的包（更快，但消息包仍是 -O0）
colcon build --symlink-install --packages-select \
  gicp_registration icp_registration lio_tf_adapter fake_vel_transform \
  livox_ros_driver2 ros2_livox_simulation teb_msgs costmap_converter_msgs
```

自检（三行，任一不符就是没吃到默认值，见 §3.4）：

```bash
cat build/gicp_registration/cmake_args.last        # 应含 '-DCMAKE_BUILD_TYPE=Release'
grep -m1 '^CMAKE_BUILD_TYPE:' build/gicp_registration/CMakeCache.txt   # =Release
grep -oh -- '-O[0-9s]' build/gicp_registration/CMakeFiles/*/flags.make | sort -u   # 应有 -O3
```

### 8.2 重启整栈后的验收标准

启动命令见 `docs/smoke_test_runbook.md`（`ros2 launch rm_nav_bringup bringup_sim.launch.py …`）。

| 指标 | 期望 | 怎么量 |
|---|---|---|
| **RTF** | 应**高于**改动前的 ≈0.72（**具体多少未知**，见 §7-1） | Gazebo 窗口 / `gz stats`；或用 `ros2 topic hz /clock` 与墙钟对比 |
| **`Control loop missed its desired rate of 30 Hz`** | 出现次数应**明显减少**（不一定归零：`docs/localization_slots.md:689` 认为这条属总算力不足，本次是"减算力"而非"改架构"） | `grep -c 'Control loop missed' <launch>.log`，与改动前的同一场景日志对比 |
| 整机 CPU | 应下降（尤其 `gzserver` 与 `gicp_registration_node`/`icp_registration_node`） | `top -H -p $(pgrep -d, gzserver)`、`ps -o %cpu,cmd -C gicp_registration_node` |
| **GICP 槽（`localization:=gicp`）** | `[status] align=` 从 **~400 ms → ~10 ms**；`采纳 N/M` 里 **N ≈ M**（改前只吃下 ~24% 的点云） | `grep '\[status\]' <launch>.log \| tail` |
| **ICP 槽（`localization:=icp`）** | 没有基线数据，至少不应变差（行为未改，只换旗标） | 同上（该包的 `[status]` 行） |
| **回归（必须做）** | `fitness_score` 仍 ≈`0.0012 m²`、`~/converged` 语义不变、**恰好一条** `map→odom`、`/scan` 与 `/segmentation/obstacle` 频率不低于改前、导航能跑完一条 goal | `docs/smoke_test_runbook.md` 的既有检查项；`ros2 topic hz /tf` 应仍是 50 Hz |
| 小车行为 | 无漂移/无 TF 甩动（本改动不改数学，**不该**出现新的 TF 问题） | RViz / `docs/monitor_map_odom.py` |

**注意**：`/tf` 的 50 Hz **不会**因为本次改动变化（定时器驱动），别用它当验收指标。

### 8.3 回滚（一条命令 / 一个文件）

```bash
# ① 立刻回到 -O0（不改任何文件）：
cd /home/weicheng/HzMi_rmsimulation
COLCON_DEFAULTS_FILE=/dev/null colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=
# ② 或临时用别的口径（保留 assert、带调试符号）：
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
# ③ 彻底回滚本次提交：
git revert <本提交的 SHA>      # 然后 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=
```

> 注意 ①③ 之后**必须显式传 `--cmake-args -DCMAKE_BUILD_TYPE=`**（空值）才能把 cache 里的
> `Release` 抹掉；只删 `colcon_defaults.yaml` 再 `colcon build` 的话，cache 里仍是 `Release`
> （CMake 不会自己清 cache 变量）。

### 8.4 两个踩过的坑（写下来免得再踩）

1. **`-p initial_pose:=[...]` 的列表必须全浮点**：`[-0.15,0.15,0,0,0,0]` 会被 ROS 参数解析器拒
   （`Sequence should be of same type. Value type 'integer' do not belong` → 节点 `abort`，exit 134）。
   写成 `[-0.15,0.15,0.0,0.0,0.0,0.0]`，且**不要带空格**（`ab_run.sh` 里 `$EXTRA` 是不加引号展开的）。
2. `icp_registration` 的 `pcd_path` 默认是空串（`icp_registration_sim.yaml:6`），
   不传就 `std::runtime_error: Invalid pcd path` 直接 abort —— 是**参数**问题，不是构建问题。

---

## 9. 证据文件（都在工作区内，未提交）

| 内容 | 路径 |
|---|---|
| 审计脚本 / 审计表（前、后） | `.tmp_cache/build_opt/audit.sh`、`audit_before.txt`、`audit_after.txt`、`audit_final.txt` |
| 台架输出（前/后 × 两种后端 + 复跑对照） | `.tmp_cache/build_opt/bench_{before,after}_{pcl,sgicp}.txt`、`bench_after_sgicp_rerun.txt` |
| 真节点 A/B 全过程（探针汇总 + `[status]` + `/tf` hz） | `.tmp_cache/build_opt/node/{before,after}_{pcl,sgicp}.txt`、`node/after_sgicp2.txt` |
| 增量重编 / 全量重编日志 | `.tmp_cache/build_opt/rebuild_2026.log`、`rebuild_full.log` |
| 二进制大小/哈希快照（前） | `.tmp_cache/build_opt/bins_before.txt` |
| 冒烟日志（3 个常驻节点） | `.tmp_cache/build_opt/smoke_*.log` |
| 复用的单节点探针台架（**上一轮** `small_gicp` 改动留下的，本次未修改） | `.tmp_cache/gicp_ab/ab_run.sh`、`probe2.py` |

## 10. 相关文档

* `docs/gicp_backend_small_gicp.md` §4（原有 A/B 口径）、§6.2（`~/converged` 抖动，`-O0` 时代就有）
* `docs/localization_slots.md:689`（RTF≈0.72 与 `Control loop missed` 的既有结论）
* `docs/issues_and_findings.md` #25（大点云订阅交付）、#19（雷达插件时间基）
* `docs/smoke_test_runbook.md`（重启整栈与回归检查的启动命令）
