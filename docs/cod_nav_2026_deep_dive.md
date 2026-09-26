# COD_NAV `rmul2026` 分支深读（只读，逐环节 + 可采纳/不采纳 + 对集成计划的修正）

> 上游：<https://github.com/qza36/COD_NAV> 分支 **`rmul2026`**
> 读取方式：全程只读（`raw.githubusercontent.com` + GitHub REST API），**未 clone、未运行任何代码、未改动本仓库任何其它文件**；本文件是本次唯一写入。
> 分支版本：HEAD = `485f2ebfad3870925d85dfeb1c9f5fb940b50646`（`increase min plc`，2026-03-10T07:30:13Z）。
> 分支树：<https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1>（35 个 blob，`truncated: false`）
> 提交史：<https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>（返回 100 条，最早 2024-12-24）
>
> 下文所有 `路径` 的原文链接均为 `https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/<路径>`；
> 本文只写**该分支当前快照里真实存在**的内容。凡属推断都显式标注「**推测**」；读不到的东西集中在 §F。

---

## A. 2026 分支架构摘要（逐环节，含确切参数与文件路径）

### A0 数据流与 TF（原文出处 `CLAUDE.md`，<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md>）

```
/livox/lidar (PointCloud2)
  ├─> cpp_lidar_filter ──> /livox/lidar_filtered ──> STVL（local + global costmap）
  ├─> pointcloud_to_laserscan ──> /scan ──> slam_toolbox ──> /map
  └─> small_point_lio (+/livox/imu) ──> /Odometry + /cloud_registered + TF odom→base_link

Nav2: SmacPlanner2D + MPPI(Omni) + SavitzkyGolaySmoother
  └─> /cmd_vel ──> velocity_smoother ──> fake_vel_transform ──> /aft_cmd_vel ──> cod_serial_ul26
```

TF 树（`CLAUDE.md`「TF Frame Tree」节 + `nav_bringup/launch/slam.launch.py` 静态发布器）：

```
map ──(static_transform_publisher, z=0.05, 0,0,0 rpy)──> odom ──(small_point_lio 节点内硬编码)──> base_link
                                                                                        └──(fake_vel_transform, yaw=−yaw(base_link))──> base_link_fake
```

- `map→odom` 的**唯一发布者**是静态 TF：`nav_bringup/launch/slam.launch.py`（arguments `--x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom`）与 `nav_bringup/launch/nav.launch.py` 中同款节点。
- Nav2 全栈 `robot_base_frame: base_link_fake`：`nav2_params.yaml:5`（bt_navigator）、`:204/:286`（两个 costmap）、`:419`（smoother_server）、`:450`（behavior_server）。

### A1 传感器 / 驱动层

| 项 | 值 | 出处 |
|---|---|---|
| LiDAR 话题 | `/livox/lidar`，**PointCloud2**（不是 CustomMsg） | `small_point_lio/config/mid360.yaml`：`lidar_type: livox_pointcloud2`、`lidar_topic: /livox/lidar` |
| IMU 话题 | `/livox/imu`（`rclcpp::SensorDataQoS()`） | 同上 `imu_topic: /livox/imu`；`small_point_lio/src/small_point_lio_node.cpp` |
| 雷达坐标系 | `livox_frame` | `mid360.yaml: lidar_frame: livox_frame` |
| Livox 驱动 | **不在仓库内**（`9856cff66` 2025-12-19 `remove livox driver` 删掉 `livox_ros_driver2/`；`9d54b5ab8` 删掉 `FAST_LIO/livox_ros_driver2/`）；CI 里用第三方 fork `https://github.com/qza36/livox_driver2_ros2`（`./build.sh humble`） | `.github/workflows/ci.yml` |
| 仿真 | **本分支没有任何仿真世界/机器人模型/plugin 启动**（`nav_bringup/urdf/*.xacro` 只是两个残存 URDF，未被任何 launch 引用；`simulation_waking_robot.xacro` 的父链接是 `chassis`，`carto.xacro` 是 `base_link`） | 树里无 `worlds/`、无 `pb_rm_simulation`、无 `gazebo` launch |
| RealSense | 相机**驱动**只在 `nav.launch.py` 启动（`realsense2_camera` 的 `rs_launch.py`，`depth_module.depth_profile: 1280x720x30`、`pointcloud.enable: true`）；`slam.launch.py` 里同一段被注释掉；`nav2_params.yaml` 里定义了 `realsense_source`（topic `/camera/camera/depth/color/points`）但 `observation_sources: livox_source #realsense_source` ⇒ **点云未接入任何代价图** | `nav_bringup/launch/nav.launch.py:120-133`、`slam.launch.py:130-141`、`nav2_params.yaml:240,263,323,346` |

### A2 点云预处理：`cpp_lidar_filter` 车体裁剪盒

源码 `cpp_lidar_filter/src/filter_node.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/cpp_lidar_filter/src/filter_node.cpp>）：

| 项 | 值 | 出处 |
|---|---|---|
| 节点/可执行 | `lidar_filter_node`（`name=my_lidar_filter`） | 启动内联参数，见下 |
| 滤波器 | `pcl::CropBox<pcl::PCLPointCloud2>`，`setNegative(param)` | `filter_node.cpp` |
| 降采样 | **`pcl::VoxelGrid` 整段被注释掉**（第 96–107 行）⇒ `leaf_size` 是**死参数** | `filter_node.cpp` |
| 输入/输出话题 | `input_topic` 默认 `/livox/lidar`；`output_topic` 默认 `/livox/lidar_filtered` | 同上 |
| QoS | 订阅与发布都用 `rclcpp::SensorDataQoS()`（best-effort）；`632fbb4eb`「稳定在先验地图下到达增益点」把发布端从 depth 10 改成 SensorDataQoS | 同上 |
| 可视化 | `crop_box_marker`（`visualization_msgs/Marker`，CUBE，1 Hz，默认 QoS depth 10），**`marker.header.frame_id = "base_link"`** | 同上 |
| 默认参数（代码内 declare） | `min_x −0.4 / max_x 0.4 / min_y −0.3 / max_y 0.3 / min_z −0.1 / max_z 0.6 / negative true / leaf_size 0.05` | 同上 |
| **实际参数（`slam.launch.py` 内联）** | `min_x −0.2 max_x 0.2, min_y −0.2 max_y 0.4, min_z −0.1 max_z 0.2, negative True, leaf_size 0.05` | `nav_bringup/launch/slam.launch.py` |
| **实际参数（`nav.launch.py` 内联）** | `min_x −0.3 max_x 0.3, min_y −0.3 max_y 0.5, min_z −0.1 max_z 0.2, negative True, leaf_size 0.05` | `nav_bringup/launch/nav.launch.py` |
| 独立 launch | `nav_bringup/launch/lidar_filter.launch.py` 曾存在（`2fa9c091c` 2025-12-23 加入），已被 `632fbb4eb` **删除** ⇒ 该分支只有「内联在两套 launch 里的两套数值」 | `632fbb4eb` diff |

⚠ 两处坐标系事实：`CropBox` 直接作用在输入点云自身坐标（`/livox/lidar` 的 `frame_id = livox_frame`，代码不做 TF），而 marker 画在 `base_link`。我们 `livox_frame` 相对 `base_link` 有 z 偏移（`nav_bringup/urdf/carto.xacro` 的 `livox_joint` 默认 `xyz="0.0 -0.045 0.3"`）⇒ **可视化框与实际裁剪区在 z 上错开约 0.3 m**（该 xacro 未被任何 launch 使用，实际外参由外部提供）。

### A3 LIO：`small_point_lio`（自研目录，非 submodule）

- 分支内 `small_point_lio/` 是**整体 vendored 的目录**（含自己的 `.github/workflows/`、`.clang-format`、`.clang-tidy`），`d1c027958`（2025-12-28 `add small point lio and ferfect mppi`）一次性加入 40+ 文件；**该分支无 `.gitmodules`**（我抓 `.../rmul2026/.gitmodules` 得 404）。
- 上游：README 自述「Small Point-LIO is an advanced implementation of the Point-LIO algorithm, delivering a 2-3x speed improvement over the original」，作者 Yingjie Huang（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/README.md>），并链到 <https://bbs.robomaster.com/article/813022>（东莞理工学院 ACE 战队 RM26 开源贴，来自 web 检索）。第三方致谢列 Eigen / `ankerl::unordered_dense` / `small_gicp` / Open3D。**同名的公开仓库很可能是 `Yancey2023/small_point_lio`（推测**，依据：作者英文名 Yancey=Yingjie Huang，检索到 <https://relatedrepos.com/gh/Yancey2023/small_point_lio>）；README 内**没有**任何 git URL，我没有做进一步的仓库确权。
- 算法（源码证据）：ESKF + **iVox** 增量体素图（`estimator.cpp`: `ivox = std::make_shared<SmallIVox>(map_resolution, 1000000)`；`small_point_lio.cpp`: `estimator.ivox->add_point(...)`），逐点平面更新：取最近 `NUM_MATCH_POINTS = 5` 点、协方差特征分解取最小特征向量当法向（`estimator.cpp`），无 ikd-Tree、**无 scan-to-map ICP、无回环、无 PGO**。点云降采样用从 `small_gicp` 抄来的 `util/voxelgrid_sampling.cpp`（文件头注释写明 copy from koide3/small_gicp）；CMake 侧 `find_package(small_gicp)` **并不存在**，CI 里装 small_gicp 属冗余（`CMakeLists.txt` 与 `.github/workflows/ci.yml` 对照可知）。
- 构建：C++20、`-march=native -ffast-math -fno-math-errno`、OpenMP、PCH（`small_point_lio/CMakeLists.txt`）。
- 雷达适配器：`livox_pointcloud2.h` 用 `PointCloud2ConstIterator` 读 `x,y,z` 之外**还必须存在 `tag`(uint8) 与 `timestamp`(float64, 乘 1e-9 当秒)**，只保留 `(*tag & 0b00111111) == 0` 的点；`livox_custom_msg` 适配器需编译期 `-DHAVE_LIVOX_DRIVER`（`CMakeLists.txt`：`find_package(livox_ros_driver2 QUIET)`，找不到只 warning）。

`small_point_lio/config/mid360.yaml` **全部值**（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/config/mid360.yaml>）：

```yaml
lidar_topic: /livox/lidar        imu_topic: /livox/imu
lidar_type: livox_pointcloud2    lidar_frame: livox_frame    save_pcd: true
point_filter_num: 1              min_distance: 0.5           max_distance: 1000.0
space_downsample: true           space_downsample_leaf_size: 0.5
gravity: [0.0, 0.0, -9.810]      fix_gravity_direction: true
check_satu: true                 satu_acc: 3.0               satu_gyro: 35.0     acc_norm: 1.0
map_resolution: 0.5              init_map_size: 10
extrinsic_est_en: false          extrinsic_T: [-0.011, -0.02329, 0.04412]        extrinsic_R: 单位阵
laser_point_cov: 0.01            imu_meas_acc_cov: 0.01      imu_meas_omg_cov: 0.01
velocity_cov: 20.0               acceleration_cov: 500.0     omg_cov: 1000.0
ba_cov: 0.0001                   bg_cov: 0.0001
plane_threshold: 0.1             match_sqaured: 81.0
publish_odometry_without_downsample: false
```
（`config/unilidar_l2.yaml` 是另一套 Unitree L2 配置，未被使用。）

### A4 LIO 输出、话题、TF、频率

`small_point_lio/src/small_point_lio_node.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio_node.cpp>）：

| 项 | 值 |
|---|---|
| 里程计 | `/Odometry`（`nav_msgs/Odometry`，publisher queue 1000），`header.frame_id = "odom"`、`child_frame_id = "base_link"`（**硬编码，不可参数化**） |
| TF | 同回调里 `tf_broadcaster->sendTransform`：`odom → base_link`（**由 LIO 节点自己发**，对应提交 `093d13a66`「feat:lio interface pub odom2base_link」；他们没有我们那种独立 `lio_tf_adapter`） |
| 外参处理 | 内部 `tf_buffer->lookupTransform(lidar_frame="livox_frame", "base_link", t)`；**TF 查不到就 `return`** ⇒ 该帧既不发 TF 也不发 odom，并打 `RCLCPP_ERROR` |
| 点云 | `/cloud_registered`（PointCloud2，`frame_id = "odom"`，逐点先转到 base_link 再按位姿搬到 odom；**只有订阅数 > 0 时才发布**） |
| twist | **全 0**：源码里 `odometry_msg.twist.twist.*` 五行全被注释，留有 `// TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link` |
| 频率 | 无独立发布频率参数。`handle_once()` 在每个点云回调与每个 IMU 回调里各调一次；`publish_odometry_without_downsample: false` 时 odom 在「三队列都非空」的循环尾部发布 ⇒ **≈ IMU 频率（实车 MID-360 100 Hz）**；置 true 会每点都发一份（配置注释里明说会掉性能） |
| 地图保存 | `save_pcd: true` 时用 `util::PointcloudMapping(0.02)` 在线累积；`ros2 service call /map_save std_srvs/srv/Trigger` → 写 `ROOT_DIR + "/pcd/scan.pcd"`；`ROOT_DIR` 由 `include/param_deliver.h.in` 经 `configure_file` 注入 `@CMAKE_CURRENT_SOURCE_DIR@` ⇒ **编译期写死源码目录** |
| ROS 接口 | `rclcpp_components` 注册 `small_point_lio::SmallPointLioNode`；节点名 `small_point_lio` |
| 与 FAST-LIO / Point-LIO 的差异（我方归纳） | ①ESKF 逐点更新（Point-LIO 族）而非 FAST-LIO 的 iterated-EKF+ikd-Tree；②地图是 iVox 哈希体素（`map_resolution 0.5`）而非 ikd-Tree；③无 `filter_size_surf/map`、无 `blind`、无 `feature_extract_enable` 那套 FAST-LIO 参数；④无 IMU 与点云的 buffer 外推参数（`max_iterations`、`filter_size_*`），换成 `space_downsample_leaf_size`、`plane_threshold`、`match_sqaured`；⑤整体做了 OpenMP/PCH/`-march=native` 的工程加速 |

### A5 重定位：**没有**（2026 的核心减法）

| 事实 | 证据 |
|---|---|
| 只有静态 `map→odom`（z=0.05，rpy=0） | `slam.launch.py` / `nav.launch.py` 的 `tf2_ros static_transform_publisher` |
| 定位 launch 只启 `map_server`，**没有 AMCL** | `nav_bringup/launch/localization_launch.py`：`lifecycle_nodes = ['map_server']` |
| `nav2_params.yaml` 里**没有 `amcl:` 段**（文件第一段就是 `bt_navigator:`） | `nav2_params.yaml:1` |
| AMCL 参数文件、cartographer lua、ICP/`small_gicp_relocalization`/`lidar_localization_ros2` 全被删 | `632fbb4eb`（2026-03-04「稳定在先验地图下到达增益点」）删除 `nav_bringup/launch/bringup.launch.py`（内含 `patchworkpp`、`small_gicp_relocalization`、`lidar_localization_ros2`）与 `nav2_params_amcl.yaml`、`params/carto.config.lua`、`params/carto.localization.lua`、`map/*.pbstream` |
| slam_toolbox **不补发** `map→odom` | `mapper_params_async.yaml: transform_publish_period: 0.0` + 同行注释 `#if 0 never publishes odometry`（slh 语义：0 = 永不发布）⇒ 静态桥是唯一发布者（**意图为推测**，但发布者唯一性是事实） |
| 先验图 | `nav_bringup/map/t1.yaml`：`image t1.pgm, mode trinary, resolution 0.05, origin [-3.84, -6.05, 0], occupied 0.65, free 0.25`；t1.pgm header `P5 213 204 255` ⇒ 地图仅 **10.65 m × 10.20 m**（x∈[−3.84, 6.81]，y∈[−6.05, 4.15]） |
| `map_server` 的 `yaml_filename` 被写死为作者机器路径 | `nav2_params.yaml:368` = `/home/cod-sentry/qza_ws/cod_nav_rmul2026/src/nav2Ver/COD_NAV/nav_bringup/map/t1.yaml`（实际被 launch 的 `map` 参数覆盖为 `nav_bringup/map/t1.yaml`，`localization_launch.py` 的 `declare_map_yaml_cmd`） |

**推论（对我们最要命的一条）**：`map→odom ≡ (0,0,0.05,rpy=0)` ⇒ 机器人在 `map` 里的位姿**等于** LIO 在 `odom` 里的位姿。这条链路只有在「先验图就是在同一 odom 原点、同一朝向采集的，且每次启动都停在同一位置同一朝向」时才成立。`nav.launch.py` 并没有任何初始化位姿的东西（无 AMCL、无 initialpose 注入）。

### A6 在线建图：slam_toolbox async / lifelong

`nav_bringup/params/mapper_params_async.yaml`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/mapper_params_async.yaml>，节点 `slam_toolbox`，可执行 `async_slam_toolbox_node`，在 `slam.launch.py` 中启动）：

| 组 | 参数 |
|---|---|
| frames/模式 | `odom_frame: odom`、`map_frame: map`、`base_frame: base_link`、`scan_topic: /scan`、`mode: lifelong`、`use_map_saver: true` |
| 求解器 | `solver_plugin: solver_plugins::CeresSolver`、`ceres_linear_solver: SPARSE_NORMAL_CHOLESKY`、`ceres_preconditioner: SCHUR_JACOBI`、`ceres_trust_strategy: LEVENBERG_MARQUARDT`、`ceres_dogleg_type: TRADITIONAL_DOGLEG`、`ceres_loss_function: None` |
| 更新/节流 | `throttle_scans: 1`（注释「信任里程计」）、`transform_publish_period: 0.0`、`map_update_interval: 2.5`、`resolution: 0.05`、`minimum_time_interval: 0.5`、`transform_timeout: 0.2`、`tf_buffer_duration: 30.0`、`stack_size_to_use: 40000000` |
| 栅格范围 | `min_laser_range: 0.01`、`max_laser_range: 5.0` |
| 关键帧 | `use_scan_matching: true`、`use_scan_barycenter: true`、`minimum_travel_distance: 1.0`、`minimum_travel_heading: 0.5`、`scan_buffer_size: 10`、`scan_buffer_maximum_scan_distance: 10.0` |
| 回环 | `do_loop_closing: true`、`loop_search_maximum_distance: 3.0`、`loop_match_minimum_chain_size: 10`、`loop_match_maximum_variance_coarse: 3.0`、`loop_match_minimum_response_coarse: 0.35`、`loop_match_minimum_response_fine: 0.45`、`loop_search_space_dimension: 8.0`、`loop_search_space_resolution: 0.05`、`loop_search_space_smear_deviation: 0.03` |
| 相关搜索 | `correlation_search_space_dimension: 0.5`、`…resolution: 0.01`、`…smear_deviation: 0.1` |
| 其他 | `link_match_minimum_response_fine: 0.1`、`link_scan_maximum_distance: 1.5`、`distance_variance_penalty: 0.5`、`angle_variance_penalty: 1.0`、`fine_search_angle_offset: 0.00349`、`coarse_search_angle_offset: 0.349`、`coarse_angle_resolution: 0.0349`、`minimum_angle_penalty: 0.9`、`minimum_distance_penalty: 0.5`、`use_response_expansion: true`、`enable_interactive_mode: true`、`debug_logging: false` |

`README.md` 明写「**目前只支持边建图边导航**」，入口 `ros2 launch nav_bringup slam.launch.py`。`nav.launch.py` 才是「先验图 + 无重定位」模式（但 `CLAUDE.md` 与 README 都没提它，只有 `632fbb4eb` 的提交信息指向它）。

### A7 3D→2D：`pointcloud_to_laserscan`（仓库内 fork）

`nav_bringup` 是 `pointcloud_to_laserscan` 的 fork（`package.xml: maintainer arlo@todo.todo`、`README.md` 是上游 ROS2 port 文档、`launch/__pycache__/*.pyc` 被提交进仓库）。

**真正被使用的参数**（不在任何 `pointcloud_to_laserscan/launch/*` 里，而是内联在 `nav_bringup` 的 launch 中）：

| 参数 | `slam.launch.py`（建图主入口） | `nav.launch.py`（先验图） |
|---|---|---|
| `cloud_in`（remap） | `/livox/lidar` | `/livox/lidar` |
| `scan`（remap） | `/scan` | `/scan` |
| `target_frame` | `base_link` | `base_link` |
| `transform_tolerance` | 0.5 | 0.5 |
| `min_height` | **0.15**（`485f2ebfa` 从 0.1 提到 0.15） | **0.10** |
| `max_height` | 1.00 | 1.00 |
| `angle_min` / `angle_max` | −3.1416 / **+3.1416（整圆！注释写 `-M_PI/2` 是错的）** | 同 |
| `angle_increment` | 0.0087（=π/360 ⇒ 723 线，注释也错写成 π/360.0 之外的说明） | 同 |
| `scan_time` | 0.3333 | 同 |
| `range_min` / `range_max` | **0.5 / 20.0** | 同 |
| `use_inf` / `inf_epsilon` | true / 1.0 | 同 |

节点源码（`pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp`）两条重要行为：
1. 输出 `scan` 用 `rclcpp::QoS(10).reliable()`；输入**只有当 `scan` 有订阅者时**才真正 `subscribe()`（`subscriptionListenerThreadLoop`，100 ms 轮询 graph），没人订阅 `/scan` 时它静默不消费点云；
2. 高度过滤是对**变换到 `target_frame` 之后**的 `z` 做（先 `tf2_->transform(*cloud_msg, *cloud, target_frame_)`，再判 `*iter_z > max_height_ || *iter_z < min_height_`）⇒「高度带是相对 `base_link` 水平面」，不是相对雷达。

仓库自带的两套 sample（**未被 `nav_bringup` 使用**，但对我们有信息量）：
- `launch/sample_pointcloud_to_laserscan_launch.py`：`cloud_in=/livox/lidar/pointcloud`、`target_frame=chassis`、`min_height 0.0 / max_height 0.65`、`angle ±1.5708`、`increment 0.0087`、`range 0.05 / 5.0`；
- `launch/sample_laserscan_to_pointcloud_launch.py`（**名字反了，实际是 pointcloud→laserscan**）：`cloud_in=/livox/idar/pointcloud`（**上游 typo `idar`**）、`target_frame=livox_frame`、`min_height −1.0 / max_height 0.1`、`range 0.45 / 10.0`。
  ⚠ 这套 `target_frame=livox_frame, −1.0~0.1, 10.0` 与**我们 bench 现有 `/scan` 配置高度重合**（我们：`livox_frame`, `min_height −1.0`, `max_height 0.1`, `range_max 10.0`）⇒ **推测**我们的 scan 参数最初就是从这份 sample 抄来的。

### A8 代价图（`nav2_params.yaml`：local 200–279 行，global 281–362 行）

| 参数 | local_costmap | global_costmap |
|---|---|---|
| `update_frequency` / `publish_frequency` | 20.0 / 20.0 | 20.0 / 20.0 |
| `global_frame` | `odom` | `map` |
| `robot_base_frame` | `base_link_fake` | `base_link_fake` |
| `rolling_window` | true | **true**（全局也是滚动窗！） |
| 尺寸 | 14 × 14 m | **50 × 50 m** |
| `resolution` | 0.05 | **0.04** |
| `robot_radius` / `footprint` | 0.2 / `[[0.15,0.15],[0.15,−0.15],[−0.15,−0.15],[−0.15,0.15]]` | 同 |
| `track_unknown_space` | （未设） | true |
| `plugins` | `["static_layer","stvl_voxel_layer","inflation_layer"]`（**局部也挂 static_layer**，type 层我们换成 `obstacle/obstacle_cloud`） | 同 |
| `inflation_layer` | `cost_scaling_factor: 5.0`、`inflation_radius: 0.75` | 同（注释「与 local_costmap 一致」） |
| `always_send_full_costmap` | True | True |
| `static_layer` | `map_subscribe_transient_local: True` | 同 + `enable: true` |

`stvl_voxel_layer`（两个 costmap 数值完全相同）：

```yaml
plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
enabled: true            voxel_decay: 0.5        decay_model: 0      # 0=linear
voxel_size: 0.05         track_unknown_space: true
observation_persistence: 0.0                    max_obstacle_height: 2.0   # layer 级
mark_threshold: 0        update_footprint_enabled: true   combination_method: 1  # 1=max
obstacle_range: 3.0      origin_z: 0.0           publish_voxel_map: true
transform_tolerance: 0.2 mapping_mode: false     map_save_duration: 60.0
observation_sources: livox_source                # realsense_source 被注释掉
livox_source:
  data_type: PointCloud2   topic: /livox/lidar_filtered   transport_type: "raw"
  marking: true            clearing: true
  obstacle_range: 8.0      raytrace_range: 9.0
  min_obstacle_height: 0.1 max_obstacle_height: 1.0
  expected_update_rate: 0.0   observation_persistence: 0.0
  inf_is_valid: false      clear_after_reading: true
  filter: "voxel"          voxel_min_points: 0
  model_type: 1            vertical_fov_angle: 2.00   vertical_fov_offset: 0.0   horizontal_fov_angle: 6.28
realsense_source:          # 定义了但未被 observation_sources 选中
  enabled: true   topic: /camera/camera/depth/color/points
  min_z: 0.2  max_z: 7.0  min_obstacle_height: 0.2  vertical_fov_angle: 0.7
  horizontal_fov_angle: 1.04  decay_acceleration: 1.  model_type: 0
```
层级陷阱：`stvl_voxel_layer.obstacle_range: 3.0`（layer 级）与 `livox_source.obstacle_range: 8.0`（source 级）**同时存在**，实际生效的是 source 级 8.0/9.0（`8d7c472ba` 2026-02-02「update:reduce min obstacle height」是最后一次动这一块）。

### A9 全局规划：SmacPlanner2D

`nav2_params.yaml:378-414`，`planner_server`: `expected_planner_frequency: 10.0`、`planner_plugins: ["GridBased"]`，`GridBased.plugin: "nav2_smac_planner/SmacPlanner2D"`：

```
tolerance: 0.5                    allow_unknown: true               downsample_costmap: false
downsampling_factor: 1            max_iterations: 1000000           max_on_approach_iterations: 1000
max_planning_time: 4.5            cost_travel_multiplier: 4.0
motion_model_for_search: "DUBIN"  angle_quantization_bins: 72       analytic_expansion_ratio: 3.5
analytic_expansion_max_length: 3.0  minimum_turning_radius: 0.05    retrospective_penalty: 0.025
reverse_penalty: 1.0  change_penalty: 0.0  non_straight_penalty: 0.0  cost_penalty: 2.0
rotation_penalty: 5.0  lookup_table_size: 20.0  cache_obstacle_heuristic: True
allow_reverse_expansion: True     smooth_path: True
smoother: {max_iterations: 10000, w_smooth: 0.4, w_data: 0.1, tolerance: 1.0e-10, do_refinement: true}
```
`motion_model_for_search / angle_quantization_bins / minimum_turning_radius / analytic_expansion_* / rotation_penalty` 都是 **Hybrid-A\* / Lattice 专用**字段，对 `SmacPlanner2D` 不产生 Dubin 搜索行为（`CLAUDE.md` 写的「SmacPlanner2D/Dubin」不准确）。真正对 2D 生效的是 `cost_travel_multiplier 4.0`（代价场里把路径推向谷底）、`tolerance 0.5`、`allow_unknown true` 与**内置 `smooth_path: True` + `w_smooth 0.4 / w_data 0.1`**。

### A10 平滑：SavitzkyGolay（**在实跑链路里是死代码**）

`smoother_server`（`nav2_params.yaml:415-429`）：`costmap_topic: global_costmap/costmap_raw`、`footprint_topic: global_costmap/published_footprint`、`robot_base_frame: base_link_fake`、`transform_tolerance: 0.1`、`smoother_plugins: ["savitzky_golay_smoother"]`，`savitzky_golay_smoother`: `plugin nav2_smoother::SavitzkyGolaySmoother`、`window_size: 7`、`poly_order: 3`、`do_refinement: True`、`refinement_num: 2`、`enforce_path_inversion: True`。

**但**：`navigation_launch.py` 的 `lifecycle_nodes` 里有 `smoother_server`，而两棵自定义 BT XML（`nav_bringup/behavior_trees/*.xml`）里**没有 `<SmoothPath>` 动作**（只有 `ComputePathToPose` → `FollowPath`）。⇒ smoother_server 被启动但从不被调用；路径平滑实际只来自 Smac 的 `smooth_path` 与 MPPI 自身的轨迹优化。（`CLAUDE.md` 的「+ SavitzkyGolaySmoother」是按其配置写的，不是按其 BT 写的。）

### A11 局部控制：MPPI（Omni）

`nav2_params.yaml:60-79`（controller_server）+ `:111-198`（FollowPath）：

```
controller_frequency: 50.0        failure_tolerance: 0.3
min_x/y/theta_velocity_threshold: 0.001
progress_checker: SimpleProgressChecker  required_movement_radius: 0.4  movement_time_allowance: 10.0
general_goal_checker: stateful True, SimpleGoalChecker, xy_goal_tolerance: 0.25, yaw_goal_tolerance: 6.28
FollowPath.plugin: "nav2_mppi_controller::MPPIController"     motion_model: "Omni"
time_steps: 60   model_dt: 0.05   batch_size: 2000   iteration_count: 1
vx_std 0.5  vy_std 0.5  wz_std 0.4      vx_max 7.5  vy_max 7.5  wz_max 2.5     vx_min −7.5 vy_min −7.5 wz_min −2.5
ax_max 5.0  ay_max 5.0  az_max 3.5      ax_min −5.0 ay_min −5.0 az_min −3.5
prune_distance: 1.7   transform_tolerance: 0.1   temperature: 0.25   gamma: 0.008
visualize: true   reset_period: 1.0   retry_attempt_limit: 2   regenerate_noises: true
TrajectoryVisualizer: {trajectory_step: 5, time_step: 3}
critics: ["ConstraintCritic","CostCritic","GoalCritic","PathFollowCritic","PathAlignCritic","ObstaclesCritic"]
  ConstraintCritic   cost_weight 4.0
  GoalCritic         cost_weight 15.0  threshold_to_consider 2.5
  ObstaclesCritic    repulsion_weight 1.5  critical_weight 20.0  consider_footprint false
                     collision_cost 10000.0  collision_margin_distance 0.1  near_goal_distance 0.5
  CostCritic         cost_weight 3.0  critical_cost 253.0  consider_footprint true
                     collision_cost 1000000.0  near_goal_distance 0.5  trajectory_point_step 2
  PathFollowCritic   cost_weight 5.0  offset_from_furthest 5  threshold_to_consider 1.5
  PathAlignCritic    cost_weight 10.0  max_path_occupancy_ratio 0.05  trajectory_point_step 4
                     threshold_to_consider 1.5  offset_from_furthest 20  use_path_orientations false
```
被注释保留的备选控制器是 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`（`nav2_params.yaml:80-110`，含 `translation_kp 3.5/ki 0.45/kd 1.5`、`enable_rotation: false`、`lookahead 0.5/1.0`、曲率减速等）。

`velocity_smoother`（`:473-485`）：`smoothing_frequency: 50.0`（注释「与 controller_frequency 一致，不拖后腿」）、`scale_velocities: False`、`feedback: "OPEN_LOOP"`（⇒ `odom_topic` 实际不生效）、`max_velocity [7.5,7.5,2.5]`、`min_velocity [−7.5,−7.5,−2.5]`、`deadband_velocity [0.05,0.05,0.05]`、`velocity_timeout 1.0`、`max_accel [5.0,5.0,3.5]`、`max_decel [−5.0,−5.0,−3.5]`、`odom_topic: "odometry"`、`odom_duration: 0.1`。

### A12 速度指令与「伪底盘」：`fake_vel_transform`

`fake_vel_transform/src/fake_vel_transform.cpp`（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/src/fake_vel_transform.cpp>）：

```cpp
declare_parameter<std::string>("robot_base_frame", "base_link");
declare_parameter<std::string>("fake_robot_base_frame", "base_link_fake");
declare_parameter<std::string>("odom_topic", "Odometry");
declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
declare_parameter<float>("spin_speed", 0.0);
...
// odomCallback: 用 odom 位姿 yaw 发 TF  robot_base_frame -> fake_robot_base_frame, rotation.z = -yaw(odom pose)
// cmdVelCallback:
aft_tf_vel.angular.z = spin_speed_;                       // ← 无条件覆盖
aft_tf_vel.linear.x  =  vx*cos(a) + vy*sin(a);
aft_tf_vel.linear.y  = -vx*sin(a) + vy*cos(a);
```
README 自述用途：`base_link_fake` 的 yaw 固定指向正前方，避免云台自旋时 Nav2 局部规划器被朝向变化带偏。

**你要求确认/反驳的三点，逐条给结论（依据：整分支 tarball grep）**：
1. **确认**：`aft_tf_vel.angular.z = spin_speed_` 是**无条件赋值**，`msg->angular.z` 从未被读取（`fake_vel_transform.cpp:60`）。
2. **确认**：`spin_speed` 默认 **0.0**（`fake_vel_transform.cpp:19`）。
3. **确认（反驳「2026 用非零自转」）**：**该分支没有任何文件把 `spin_speed` 设为非零**。全分支 `spin_speed` 只出现 5 次：`src/fake_vel_transform.cpp` 的 declare/get/赋值 3 次、`include/.../fake_vel_transform.hpp:35` 成员声明 1 次、`README.md:24` 文档 1 次。`slam.launch.py`/`nav.launch.py`/`fake_vel_transform/launch/fake_vel_transform_launch.py` 都只传 `use_sim_time`；`nav2_params.yaml` 里也没有 `fake_vel_transform:` 段（参数来自 launch，yaml 不覆盖）。
   ⇒ 在 2026 分支上，`spin_speed == 0` 让 `angular.z ≡ 0`，**MPPI 的 `wz`（±2.5）被整体丢弃**。这一点与 `yaw_goal_tolerance: 6.28` + Omni 平移是自洽的：他们只控位置不控朝向。我们的修法是「0 时直通」——**语义与他们不同**（我们在 `spin_speed==0` 时保留 `wz`），这一点在移植时必须明确。

### A13 任务层 / 决策层

| 组件 | 内容 | 出处 |
|---|---|---|
| BT（到点） | `RecoveryNode(number_of_retries=10)` → `PipelineSequence`：`RateController hz=3.0` → `RecoveryNode(ComputePathToPose, retries=1)`（失败→`ClearEntireCostmap global`）；`RecoveryNode(FollowPath, retries=10)` → `ReactiveFallback(FollowPath)`（失败→`ClearEntireCostmap local`）；最外层 `ReactiveFallback`：`GoalUpdated` / `RoundRobin(ClearEntireCostmap local+global)` | `nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` |
| BT（多点） | 同上，但 `RemovePassedGoals radius=0.7` → `ComputePathThroughPoses(planner_id=GridBased)`，恢复集多一个 `BackUp backup_dist=1.0 backup_speed=1.0` | `navigate_through_poses_w_replanning_and_recovery.xml` |
| 注意 | XML 里**没有 `IsStuck`**（`nav2_is_stuck_condition_bt_node` 只在 `plugin_lib_names` 里）；`ReactiveFallback name="FollowPathWithStuckDetection"` 里只有一个子节点；`nav2_through_poses` BT 的元数据块未写 `TreeNodesModel` | 同上 |
| 航点 | `nav_bringup/WPs/t1.csv` 三行：`(0.0620837,−1.11927)`、`(2.81072,−1.3887)`、`(5.64674,0.0852474)`（quaternion z/w，yaw≈0） | `nav_bringup/WPs/t1.csv` |
| 航点打点 | RViz 面板 `waypoint_editor/WaypointEditorPanel` + `InteractiveMarkers`（namespace `/interactive_marker_server`，另一个是 `/slam_toolbox`）；`waypoint_editor` 包**不在仓库内**（外部依赖），CSV 列名 `id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command` 正是它的格式 | `nav_bringup/rviz/cod_nav.rviz`、`WPs/t1.csv` |
| 航点执行 | `waypoint_follower`：`loop_rate 20`、`stop_on_failure false`、`waypoint_task_executor_plugin: wait_at_waypoint`、`wait_at_waypoint.enabled True`、`waypoint_pause_duration 200` | `nav2_params.yaml:462-472` |
| 裁判/比赛逻辑 | `nav_bringup/scripts/bt.sh`：无限循环 `ros2 topic echo /serial/receive --once` 取 `game_type`；`==3` → `NavigateToPose` 回 `(0,0,yaw=0)`；`==2` → 去 `(4.55,−2.0,z=0,w=1)`；其它值等待 | `nav_bringup/scripts/bt.sh` |
| 执行机构 | `cod_serial_ul26`（`executable cod_serial`）由 `slam.launch.py`/`nav.launch.py` 启动，**不在仓库内**；它吃 `/aft_cmd_vel` | `CLAUDE.md` 数据流 + 两个 launch |
| 其它脚本 | `scripts/nav_bringup.sh`、`scripts/sim_nav_bringup.sh` 都指向**已删除的** `nav_bringup.launch.py` / `sim_nav_bringup.launch.py`，且写死 `/home/cod-sentry/...`、`/home/arlo/...` ⇒ 死脚本 | `nav_bringup/scripts/` |

### A14 `resource/`

只有两张 PNG：`resource/cod-1.png`（43,453 B，README 顶部 logo）、`resource/cod.png`（129,659 B，未被任何文件引用）。**没有** 地图、标定文件、bag、urdf mesh。`README.md` 仅 1,105 B（标题 + 环境 + 编译 + `ros2 launch nav_bringup slam.launch.py` + 遥操 + 一句「参数模板：`nav_bringup/params/nav2_params.yaml`」）。

### A15 明确确认「不存在」的东西

对整个分支 tarball 做 grep（`--include=*.py,*.yaml,*.yml,*.xml,*.cpp,*.h,*.hpp,*.txt,*.md`，排除 `3rdparty/`）：

| 关键词 | 结果 |
|---|---|
| `amcl` | **0 命中** |
| `cartographer` | **0 命中**（连 `carto.xacro` 的**文件名**都只是残留，内容里只有 `base_link`+`livox_frame`） |
| `relocali`（relocalization） | **0 命中** |
| `patchwork` / `linefit` / `terrain_analysis` | **0 命中**（`patchwork` 只在我 grep 到的 `.idea/vcs.xml` 里有一条失效的 vcs 映射） |
| `esdf` | **0 命中** |
| `frontier` | **0 命中** |
| `small_gicp` | 4 命中，全部是**源码注释/README/CI**（`util/voxelgrid_sampling.*` 的文件头、README 第三方表、CI 安装步骤）⇒ small_gicp **不是运行期依赖**，voxelgrid 是抄进来的代码 |
| 动态障碍跟踪/预测 | **无**任何模块；唯一的「时间维」是 STVL 的 `voxel_decay: 0.5`（linear，0.5 s 后体素归零） |

⇒ 2026 分支**没有** RealSense-驱动以外的相机算法、**没有** ESDF、**没有** frontier 探索、**没有** 显式动态障碍跟踪/预测。与我们 bench 不同的是：它也**没有地面分割**（`linefit_ground_segmentation` 那一环被 `min_height 0.10/0.15` 硬高度带取代）。

---

## B. 只属于 2026 的关键设计决定（含 commit 意图；标注推测）

| # | 决定 | 时间 / commit | 能看出的意图（原文优先） | 性质 |
|---|---|---|---|---|
| B1 | **LIO 从 FAST-LIO 换成 `small_point_lio`** | `9d54b5ab8`「feat:replace fastlio2」(2025-11-18) 删除 `FAST_LIO/`；`d1c027958`「add small point lio and ferfect mppi」(2025-12-28) 真正落地 | 提交信息只写 replace；真正动机来自其 README：「**2-3x speed improvement over the original**（Point-LIO）」+ bbs 开源贴标题「比 Point-LIO 快 2-3 倍的里程计」。**推测**：为 7.5 m/s 高速 + 实车算力做的选择 | 事实 + 推测 |
| B2 | **做减法：去掉所有 submodule** | `9856cff66`「remove livox driver」(2025-12-19)、`9dfbeabdd`（删 `stvl_plugin/` 自建副本）、`632fbb4eb`（删 `bringup.launch.py`、amcl/carto 参数、`map/*.pbstream`）、`a7299b60a`（删 `FAST_LIO_ROS2/`、`loam_interface/`、`pb_omni_pid_pursuit_controller/`、`sensor_scan_generation/`、`script/`） | 分支现无 `.gitmodules`，工作区只剩 5 个包。**推测**：赛季前收敛维护面，把地面分割/pcd2pgm/重定位全部外移或砍掉 | 事实 + 推测 |
| B3 | **砍重定位 → 静态 `map→odom` + 只留 `map_server`** | `632fbb4eb`「**稳定在先验地图下到达增益点**」(2026-03-04)，同批新增 `nav.launch.py`（先验图模式）与 `WPs/t1.csv` | 提交信息直说是「在先验地图下稳定到达增益点」——即用固定先验图 + 固定起点换取稳定性，不依赖重定位。**推测**：比赛场地固定、起点固定，AMCL/ICP 调参时间不值得 | 事实 + 推测 |
| B4 | **slam_toolbox 只做 lifelong 建图，不发布 `map→odom`** | `mapper_params_async.yaml`: `mode: lifelong` + `transform_publish_period: 0.0`；`ffc4e24e3`「feat:rmul2026 slam navgation param」(2025-11-18) | **推测**：让 `map→odom` 的唯一发布者是静态桥（避免 TF 多发布者争用），代价是回环修正只改 `/map` 不改位姿 | 推测（参数是事实） |
| B5 | **STVL 取代 nav2 自带 voxel_layer，并双图（global+local）都用** | `9dfbeabdd`「feat:use stvl instead of voxel_layer」(2025-12-19)、`6e03b83e0`「chore:add stvl to global map」；`202f29a24` 曾以 git subtree 拉入 `stvl_plugin/`，随后删掉 | 现在是**外部依赖** `spatio_temporal_voxel_layer`（我们也有）。注释里明确 `voxel_decay 0.5` + `min_obstacle_height 0.1` | 事实 |
| B6 | **控制器从 `pb_omni_pid_pursuit` 换成 MPPI** | `0639ec26b`「feat:copy pb pib controller」(2025-12-23) → `a7299b60a`「**MPPI provides stable tracking and obstacle avoidance, but it oscillates around the target point**」(2026-03-10) 引入 MPPI 全套 + 新增 `CLAUDE.md` + 删 `pb_omni_pid_pursuit_controller/` | 提交信息直接给出了「MPPI 好，但目标点附近震荡」的结论；随后 `a1c33ad80`「Eliminate oscillations near the target point」→ `4651c925a`「increase speed」→ `485f2ebfa`「increase min plc」是连续三轮调参。**残留证据**：`cod_nav.rviz` 里仍留 `lookahead_point`、`curvature_points_marker_array` 两个 pb_omni 专有话题 | 事实 |
| B7 | **inflation 从「0.3/0.4」直接改成「cost_scaling_factor 5.0 / inflation_radius 0.75」** | `a7299b60a`（并在 `872adfafd` 前是 0.5/0.2） | 代码注释原文：「关键修正！原 0.3 衰减太慢，整个膨胀区代价都极高，**MPPI 没有梯度可用**。5.0 让代价快速衰减，形成清晰的『远离障碍物』梯度」；同时 `CostCritic.critical_cost: 253.0`「只有 inscribed/lethal 才触发碰撞惩罚，不再把整个膨胀区都当碰撞」 | 事实 |
| B8 | **极度激进的实车速度包线** | `4651c925a`「increase speed」 | `vx/vy ±7.5 m/s`、`ax/ay ±5.0`、`controller_frequency 50`、`vz_max 2.5`、`prune_distance 1.7`、`reset_period 1.0`（注释「延长重置周期，保持 warm-start 让速度持续攀升」）、`local_costmap 14×14`（注释「7.5 m/s × 2s ≈ 15m」） | 事实 |
| B9 | **姿态控制整体外置：`base_link_fake` + `angular.z ≡ 0` + `yaw_goal_tolerance 6.28`** | 沿用 `f768e0c23`「add fake_vel_transform」(2024-12-24) 的设计；2026 用 `spin_speed=0`（从不设非零） | README：让 Nav2 看到「yaw 固定指向正前方」的伪底盘；配合 `yaw_goal_tolerance 6.28`（等于不检查朝向）与 Omni 平移，**只控位置**。**推测**：朝向由云台/底盘自转承担，或本就不要求朝向 | 事实 + 推测 |
| B10 | **全局代价图用 50×50 滚动窗 + `resolution 0.04`，而不是「整张先验图」** | `a7299b60a` / `872adfafd` 期间的参数演化 | 先验图只有 10.65×10.2 m，而全局窗 50×50 m：`static_layer` 只覆盖场地那一小块，其余是 `track_unknown_space: true` 的未知区 + `allow_unknown: true`。**推测**：为高速预留感知余量，宁可让全局层跟着车跑 | 事实 + 推测 |
| B11 | **任务层用 bash 轮询裁判系统 + RViz 手工打点** | `632fbb4eb` 新增 `WPs/t1.csv` + rviz 的 `waypoint_editor/WaypointEditorPanel`；`bt.sh` 时间不明（不在最近 100 条提交的改动里） | 没有状态机/行为树决策层，比赛阶段切换由 `game_type` 的 if/else 决定 | 事实 |
| B12 | **RealSense 只加了参数与 RViz 帧，未接入导航** | `4de98f194`「feat: integrate realsense camera parameters and update voxel decay settings」(2025-12-23) | `observation_sources: livox_source #realsense_source` 明确注掉了。**推测**：留给视觉/自瞄组，或曾试过深度相机后放弃 | 事实 + 推测 |
| B13 | **作者自述的坑写进了 `CLAUDE.md`** | `a7299b60a` 新增 `CLAUDE.md`，`485f2ebfa` 最后修订 | 「Known Issues in Config: `bt_navigator.odom_topic` is set to `"odomety"` (typo for `"odometry"`)」；同处还承认「slam.launch.py contains **dead references to `fast_lio`** package (legacy)」 | 事实 |

---

## C. 对我们**可采纳**清单

> 每条格式：**改什么 / 抄哪个文件哪个值 / 预期收益 / 风险 · 回退**。只列 2026 分支真实存在的东西。
> 上游基线：<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml>

### C1 STVL 的 `model_type: 1` + 3D 视锥（**最低风险、最该先抄**）
- **改什么**：我们 `stvl` 的观测源加 `model_type: 1`、`vertical_fov_angle: 2.00`、`vertical_fov_offset: 0.0`、`horizontal_fov_angle: 6.28`、`filter: "voxel"`、`voxel_min_points: 0`。
- **抄哪**：`nav2_params.yaml:240-262`（local）/`:323-345`（global）的 `livox_source` 段。
- **收益**：MID-360 是 3D 雷达，`model_type: 0`（深度相机视锥）会按 0.7 rad 垂直 FOV 裁切；改成 1 + 2.00 rad 才与 -7°~52° 的 MID-360 匹配 ⇒ 标记/清除范围正确，减少「明明看到却不标记」。
- **风险 · 回退**：`vertical_fov_angle 2.00` 比 MID-360 实际 (~1.03 rad) 大一倍，属保守取值，副作用只是多算；单参数回退。

### C2 STVL 时间衰减参数组（`voxel_decay 0.5` / `decay_model 0` / `voxel_size 0.05`）
- **改什么**：`local_obstacle` 增加取值 `stvl`；`voxel_decay: 0.5`、`decay_model: 0`(linear)、`voxel_size: 0.05`、`observation_persistence: 0.0`、`combination_method: 1`、`update_footprint_enabled: true`、`transform_tolerance: 0.2`。
- **抄哪**：`nav2_params.yaml:227-240` 与 `:310-323`。
- **收益**：动态障碍与「人走过」的残影在 0.5 s 内消失；`local_obstacle:=stvl` 让 local 也有时间维（我们现在只有 `scan/cloud/both`，无时间维）。
- **风险 · 回退**：0.5 s 太短会让**静态薄结构**（我们的 0.40 m 矮墙、桌腿）在传感器短暂遮挡时被擦掉；`8d7c472ba`「update:reduce min obstacle height」说明他们自己也在反复动这一块。先只开在 local，global 保持持久；回退 = 删该层。

### C3 观测源级 `obstacle_range 8.0 / raytrace_range 9.0`
- **改什么**：STVL `observation_sources` 的 source 级显式给 `obstacle_range: 8.0`、`raytrace_range: 9.0`（layer 级他们写 3.0，实际被 source 级覆盖）。
- **抄哪**：`nav2_params.yaml:251-253`。
- **收益**：我们 layer 级若只写 3.0，会造成「3 m 外不标记」；8/9 m 与 `/scan range_max 20` 的远场策略一致。
- **风险 · 回退**：清除过激（9 m raytrace）在玻璃/低反射面会误清；回退到 5/6 或去掉 source 级。

### C4 车体裁剪盒（`cpp_lidar_filter` 思路）
- **改什么**：新增裁剪盒节点 → `/livox/lidar_filtered`，**只喂 STVL 与 local cloud 层**，`/scan` 链不动。
- **抄哪**：`cpp_lidar_filter/src/filter_node.cpp` 的 `pcl::CropBox` + `setNegative`；QoS **必须**是 `rclcpp::SensorDataQoS()`（`632fbb4eb` 的修正）；数值参考 `nav.launch.py` 的 `x±0.3 / y −0.3~0.5 / z −0.1~0.2`，但**必须重标定**（见风险）。
- **收益**：车身/云台自遮挡从 3D 代价图里消失，减少局部代价图中心假障碍。
- **风险 · 回退**：①`CropBox` 在**输入点云坐标系（我们 `livox_frame`）**里裁，不是 `base_link`；而他们的 marker 画在 `base_link`（`filter_node.cpp` 里 `marker.header.frame_id = "base_link"`），⇒ 视觉与实际错位，照抄数值必错。②`leaf_size` 是**死参数**（VoxelGrid 整段注释掉），别指望降采样。③盒子太紧会删真障碍（y 方向不对称）。回退：停节点即恢复原链路。

### C5 `/scan` 射程：`range_max 10 → 20`、`range_min 0.05 → 0.5`
- **改什么**：我们 p2l 的 `range_max: 20.0`、`range_min: 0.5`。
- **抄哪**：`nav_bringup/launch/slam.launch.py`（p2l 节点参数块）。
- **收益**：远场墙/长走廊进 scan；`range_min 0.5` 顺带压掉近场自车回波（与他们一致）。
- **风险 · 回退**：①我们场地小（15×28 m），20 m 会把整场墙都收进来，`inf` 占比下降但单帧点数上升（`increment 0.0087` 已是 723 线）；②`range_min 0.5` 会让我们**丢掉 0.5 m 内的近距回波**（我们现在的痛点恰是近场），这一条我只建议先 A/B，不要与 `range_max` 同批改。回退：单参数。

### C6 高度带：从「雷达面之下」改成「`base_link` 之上 0.10~1.00」
- **改什么**：p2l `target_frame: base_link`（我们现在是 `livox_frame`）、`min_height 0.10`（nav 模式）/`0.15`（建图）、`max_height 1.00`。
- **抄哪**：`slam.launch.py` / `nav.launch.py` 的 p2l 参数块（注意：**不是** `pointcloud_to_laserscan/launch/*` 那两个 sample）。
- **收益**：把地面回波整段切掉（我们现在 `-1.0~0.1` 是**从上往下扫**，会收进地面），SLAM 匹配更干净；且高度带相对 `base_link` 后与雷达安装高度解耦。
- **风险 · 回退**：我们墙只 0.40 m、近场回波 0.10~0.196 m ⇒ 若扫描面抬到雷达上方 0.10 m，**矮墙可能整段掉出高度带**（这就是集成计划里担心的那条，方向是对的）。必须先跑 z 剖面（`cloud_z_profile.py`）再改。回退：单参数。

### C7 MPPI 配方（我们 P2 想做的事，这里给「必须一起抄」的全集）
- **改什么**：`nav:=mppi` 时，除 `motion_model: Omni / time_steps 60 / model_dt 0.05 / batch_size 2000` 外，把「消震荡三件套」一起抄：`temperature: 0.25`、`gamma: 0.008`、`reset_period: 1.0`、`prune_distance: 1.7`；critics 权重 `GoalCritic 15.0/thr 2.5`、`CostCritic cost_weight 3.0 + critical_cost 253.0 + consider_footprint true`、`PathFollowCritic 5.0/1.5`、`PathAlignCritic 10.0/1.5`、`ObstaclesCritic repulsion 1.5/critical 20.0/consider_footprint false/collision_margin 0.1`；`goal_checker xy_goal_tolerance 0.25 + yaw_goal_tolerance 6.28`；`velocity_smoother smoothing_frequency == controller_frequency`、`feedback OPEN_LOOP`、`deadband 0.05`、`max_accel/decel` 与 MPPI 的 `ax/ay/az` 对齐。
- **抄哪**：`nav2_params.yaml:111-198` + `:75-79` + `:473-485`。
- **收益**：这三件套是他们**用三轮 commit 换来的**（`a7299b60a`→`a1c33ad80`→`4651c925a`），直接抄可以跳过同样的震荡调试；`temperature 0.25/gamma 0.008` 是「更果断、允许更快速度变化」，`reset_period 1.0` 是 warm-start 提速。
- **风险 · 回退**：`PathAlignCritic.use_path_orientations: false` 是 Omni 专用假设；`critical_cost 253.0` 依赖 0.05/0.04 m 分辨率与 `inflation_radius 0.75` 的梯度形状，**与 C8 必须同批**否则又变成「整个膨胀区都当碰撞」。回退：`nav:=rpp`。

### C8 inflation 梯度（`cost_scaling_factor 5.0` + `inflation_radius 0.75`）
- **改什么**：两图 `inflation_layer` 统一 `cost_scaling_factor: 5.0`、`inflation_radius: 0.75`。
- **抄哪**：`nav2_params.yaml:219-221`（local）/`:304-306`（global）。
- **收益**：注释直说「原 0.3 衰减太慢，MPPI 没有梯度可用」；对我们 NavFn + MPPI 都有好处（路径居中于代价谷底）。
- **风险 · 回退**：我们机器人 `robot_radius 0.40`（他们是 0.30 m 方框），0.75 的膨胀在窄通道可能直接判死。建议按 `inflation_radius = robot_radius + 0.35` 比例落地（≈0.75 对 0.40 半径就是 0.75），先只改 `cost_scaling_factor`，`radius` 保持 0.5/0.55 做对照。回退：单参数。

### C9 Smac2D 的**有效**参数
- **改什么**：`planner:=smac2d` 时抄 `tolerance: 0.5`、`allow_unknown: true`、`cost_travel_multiplier: 4.0`、`max_planning_time: 4.5`、`max_iterations: 1000000`、`max_on_approach_iterations: 1000`、`cache_obstacle_heuristic: true`、`smooth_path: true` + `smoother: {max_iterations 10000, w_smooth 0.4, w_data 0.1, tolerance 1.0e-10, do_refinement true}`。
- **抄哪**：`nav2_params.yaml:383-413`。
- **收益**：`cost_travel_multiplier 4.0` 把路径推向代价谷底（我们 NavFn 容易贴缝走）；`smooth_path` + `w_smooth 0.4` 是**他们链路里真正生效的平滑**。
- **风险 · 回退**：Smac 比 NavFn 慢；`smooth_path` 只保证「平滑」不保证不切角 ⇒ 必须核 footprint 碰撞。**不要抄** `motion_model_for_search: "DUBIN"`、`angle_quantization_bins: 72`、`minimum_turning_radius: 0.05`、`analytic_expansion_*`、`rotation_penalty`、`lookup_table_size`、`allow_reverse_expansion`（Hybrid/Lattice 专用，2D 插件无效）。回退：`planner:=navfn`。

### C10 代价图几何方法论（不是照抄数值）
- **改什么**：局部/全局 `update_frequency`、`publish_frequency` 提到 20.0；全局用 `rolling_window: true` + 大窗口（他们 50×50 @0.04）。
- **抄哪**：`nav2_params.yaml:202-215` / `:283-296`。
- **收益**：`20 Hz` 发布 + `always_send_full_costmap: true` 让 MPPI 每个控制周期拿到新鲜代价图；全局滚动窗在「先验图很小（他们 10.65×10.2 m）」时不会把规划限制在已知区。
- **风险 · 回退**：`always_send_full_costmap: true` 在 50×50@0.04 上是 1250×1250（≈1.5 MB/帧 @20 Hz）——**我们场地更小，别抄 50×50**。回退：回 5 Hz。

### C11 footprint 覆盖 robot_radius 的写法（Nav2 语义提醒）
- **改什么**：不要同时依赖 `robot_radius` 与 `footprint`。他们写了 `robot_radius: 0.2` 但又给了 `footprint: [[0.15,0.15],[0.15,-0.15],[-0.15,-0.15],[-0.15,0.15]]`（0.30 m 方框）——**Nav2 以 `footprint` 为准**。
- **抄哪**：`nav2_params.yaml:210-211` / `:288-289`。
- **收益/风险**：知道了这一点才能正确解释他们「0.30 m 方框 + inflation 0.75」的激进组合；我们 `robot_radius 0.40` 比它大 33%，任何照抄的膨胀/间距参数都要按比例重算。**风险**：误以为 `robot_radius` 生效会低估他们的真实几何。回退：无（认知项）。

### C12 `fake_vel_transform` 的「朝向解耦」语义（设计级借鉴）
- **改什么**：把 `spin_speed` 的两种语义**显式分档**：`spin_speed == 0` → 直通（我们已修）；`spin_speed != 0` → 覆盖 `angular.z` 且把 Nav2 的 `yaw_goal_tolerance` 放宽（他们 `6.28`）、`robot_base_frame` 指向伪帧。
- **抄哪**：`fake_vel_transform/src/fake_vel_transform.cpp`（TF 用 `-yaw(odom pose)`）、`nav2_params.yaml:79`（`yaw_goal_tolerance: 6.28`）、`CLAUDE.md` 的 TF 树说明。
- **收益**：自转哨兵语义下 Nav2 只控位置，避免局部规划器被朝向变化带偏；他们整条链路是自洽的（`angular.z≡0` + Omni + yaw 容差 6.28）。
- **风险 · 回退**：一旦 `spin_speed != 0`，MPPI 的 `wz`（±2.5）被**丢弃**，若 `yaw_goal_tolerance` 未同步放宽就会永远到不了「到点」判定；回退：`spin_speed := 0` 且容差回默认。

### C13 任务层 BT 结构（只借结构，不借数值）
- **改什么**：`RateController hz="3.0"` 重规划 + `RecoveryNode` 分层重试（ComputePath 1 次 / FollowPath 10 次）+ 失败分别清 global/local 代价图 + `RemovePassedGoals radius="0.7"`。
- **抄哪**：`nav_bringup/behavior_trees/*.xml`。
- **收益**：3 Hz 重规划对动态障碍足够且省 CPU；「清图恢复」比「原地转圈」便宜。
- **风险 · 回退**：`number_of_retries=10` 且 XML 里**没有** `IsStuck` 判据 ⇒ 可能长时间卡死而不触发 back up（他们的 `BackUp 1.0 m @1.0 m/s` 只在多点 BT 里）。回退：用回 stock BT。

### C14 诊断技巧：p2l 是「惰性订阅」
- **改什么**：A/B 时确认 `/scan` 至少有 1 个订阅者，否则 p2l 不消费点云（`subscriptionListenerThreadLoop`）。
- **抄哪**：`pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp`。
- **收益**：避免把「没人订阅 `/scan`」误判为「点云/驱动坏了」。回退：无。

---

## D. 对我们**不采纳**清单（含理由）

| # | 不采纳的东西 | 理由 |
|---|---|---|
| D1 | **无重定位 + 静态 `map→odom`(z=0.05)** | 我们 4 种重定位（amcl/icp/slam_toolbox/cartographer）是资产；静态桥把「机器人在 `map` 里的位姿」直接等同于 LIO 的 `odom` 位姿，**只有在**①先验图恰在同一 odom 原点/朝向采集、②每次启动都停在同一位置同一朝向、③能容忍 odom 长期漂移时才成立；`nav.launch.py` 里没有任何初始化位姿机制（无 AMCL、无 initialpose 注入）。**适用条件**就写在 §A5 的推论里：可用于「纯在线建图（无重定位）」模式的过渡，不能作为默认。 |
| D2 | **`mapper:=slam_toolbox` lifelong + `transform_publish_period: 0.0`** | ①lifelong 与我们的先验图/重定位资产冲突；②`transform_publish_period 0.0` 让 SLAM **只改 `/map` 不改位姿**，回环修正无法体现在机器人位姿上 ⇒ 静态层会在机器人脚下平移。 |
| D3 | **`save_pcd: true` 常开 + `ROOT_DIR` 编译期写死源码目录** | 全量点云（`PointcloudMapping(0.02)`）常驻内存并在长跑中膨胀；`param_deliver.h.in` 把 `@CMAKE_CURRENT_SOURCE_DIR@` 编进二进制，`map_save` 会往**源码树**写 `pcd/scan.pcd`。要用就改成参数化路径 + 按需开启。 |
| D4 | **速度包线：±7.5 m/s、±5.0 m/s²、50 Hz、`vx_std 0.5`、`reset_period 1.0`** | 实车参数；仿真里 RoboMaster 底盘/接触模型与实车差距大，直接移植会让 MPPI 输出被物理引擎「吃掉」并掩盖真实问题。我们按 2.0 m/s 起标。 |
| D5 | **全局 50×50 m @0.04 + `always_send_full_costmap: true`** | 1250×1250 的每帧全量发布对我们 15×28 m 场地纯属浪费（CPU/带宽），且会把大量 `unknown` 卷进规划。 |
| D6 | **裁剪盒数值照抄（±0.3/−0.3~0.5/−0.1~0.2）** | 车体不同、裁剪坐标系是 `livox_frame` 而 marker 画在 `base_link`（z 差约 0.3 m），照抄等于把裁剪区放错位置。 |
| D7 | **工程卫生问题** | ①`bt_navigator.odom_topic: "odomety"`（他们自己在 `CLAUDE.md` 里承认的 typo）；②`velocity_smoother.odom_topic: "odometry"`；③`map_server.yaml_filename` 写死 `/home/cod-sentry/qza_ws/...`；④`slam.launch.py:15` 的 `get_package_share_directory('fast_lio')` 是**死引用**——`CLAUDE.md` 只说「dead references」，但该 API 在包不存在时会**抛 `PackageNotFoundError` 让整个 launch 起不来**（**推断**，我未运行验证）；⑤`cod_nav.rviz` 里 pb_omni 的遗留话题（`/lookahead_point`、`/curvature_points_marker_array`）；⑥`smoother_server` 启动了但 BT 里没有 `SmoothPath`（死代码）；⑦仓库里带 `.idea/`（含 39,943 B 的 `editor.xml`）、`launch/__pycache__/*.pyc`、根级 `.idea/` 与 `nav_bringup/.idea/` 两份。 |
| D8 | **RealSense 相机链路** | `nav.launch.py` 启了 `realsense2_camera`，但 `observation_sources` 只选 `livox_source`，点云未接入任何代价图 ⇒ 是「未接线的死配置」，且对我们 Gazebo 仿真无意义。 |
| D9 | **`plugin_lib_names` 里 BT 未用到的节点** | `nav2_is_stuck_condition_bt_node`、`nav2_spin_action_bt_node`、`nav2_assisted_teleop_*` 等只是被列在 yaml 里，两棵 BT XML 都没用（缺 `IsStuck` 恰恰是 D 项理由之一）。 |
| D10 | **「大图 unknown」策略** | `track_unknown_space: true` 双开 + 50 m 滚动窗的组合对我们小场地无收益；我们按 `docs/debug_fastlio_cartographer.md §10.5 遗留⑤` 单独定策略（与集成计划 §1 最后一行结论一致）。 |
| D11 | **地面分割被高度带取代** | 我们保留 `linefit_ground_segmentation` 作为可测资产；他们的 `min_height 0.10/0.15` 硬高度带对坡道/矮墙不鲁棒（他们 2025 还有 `patchworkpp`，2026 也砍了）。 |

---

## E. 对 `docs/cod_nav_2026_integration_plan.md` 的修正/补充建议（逐条，不直接改文件）

> 该文件我已只读通读（144 行）。以下按它的行号/小节给建议。**我没有改动它。**

1. **§1 表「3D 点云预处理」行 —— `leaf 0.05` 是错的（死参数）**。`cpp_lidar_filter/src/filter_node.cpp` 里 `pcl::VoxelGrid` 整段被注释（第 96–107 行），`leaf_size` 只被 `declare_parameter`/`get_parameter`，从未被使用。建议改为：「只裁剪、无降采样（`leaf_size` 为死参数）」。
2. **§1 同一行 —— 只写了 `nav.launch.py` 的一套盒子**。`slam.launch.py`（建图主入口）用的是另一套：`x±0.2 / y −0.2~0.4 / z −0.1~0.2`。建议注明「两套 launch 各一套数值」，避免误以为是唯一值。
3. **§1 同一行 —— 补坐标系事实**：`CropBox` 在**输入点云自身的坐标系**（`livox_frame`）工作，不做 TF；而节点发布的 `crop_box_marker` 的 `frame_id` 是 `base_link`（源码硬编码）⇒ 可视化与实际裁剪区错位（我们 `livox_frame` 相对 `base_link` 有 z≈0.3 偏移）。建议把「数值须按我们车体重标」升级为「**必须在 `livox_frame` 下重标，且不要相信 marker 的位置**」。
4. **§1 表「/scan 射程与高度带」行 —— `0.01~1.00` 应改为 `0.10/0.15~1.00`**：`slam.launch.py` 是 `min_height 0.15`（`485f2ebfa` 从 0.1 提上去的），`nav.launch.py` 是 `0.10`；两者 `max_height` 都是 `1.00`。同时补 `target_frame: base_link`（**不是 `livox_frame`**）与 `angle_min/max = ∓3.1416`（**整圆**；源码里注释写 `-M_PI/2` 是错的）、`angle_increment 0.0087`（723 线）、`scan_time 0.3333`。
5. **§1 同一行 —— 输入话题写清楚**：真正被使用的是 `/livox/lidar`（PointCloud2）；`/livox/lidar/pointcloud` 只出现在仓库自带的 `sample_pointcloud_to_laserscan_launch.py` 里（那份还有 `target_frame: chassis`、`5.0` 射程，未被 `nav_bringup` 使用）。另外提醒：我们 bench 现在的 `/scan` 参数（`livox_frame`、`−1.0~0.1`、`10.0`）与**他们另一份 sample**（`sample_laserscan_to_pointcloud_launch.py`，`livox_frame`、`−1.0~0.1`、`0.45~10.0`）几乎一致 ⇒ 推测我们当初就是抄那份 sample 的，可作为「为什么我们高度带在雷达面之下」的溯源依据。
6. **§1 表「路径平滑」行 —— 收益证据不足，需改叙述**。他们的自研 BT 里**没有 `<SmoothPath>`**，`smoother_server` 只被 `navigation_launch.py` 启动、从不被调用 ⇒ **smoother 在他们的实跑链路里是死代码**；真正的平滑来自 `SmacPlanner2D.smooth_path: true` + 内置 `smoother{w_smooth 0.4, w_data 0.1, do_refinement true}`。建议把 `smoother:=savgol` 的定位改成「我们自研槽位（需在自己的 BT 里显式加 `SmoothPath` 才生效）」，并把「抄 Smac 的 `smooth_path`」列成更优先的低风险项。
7. **§1 表「全局规划」行 —— 删掉/标注 Hybrid 专用参数**。`motion_model_for_search: "DUBIN"`、`angle_quantization_bins: 72`、`minimum_turning_radius: 0.05`、`analytic_expansion_*`、`rotation_penalty`、`lookup_table_size`、`allow_reverse_expansion` 都是 Hybrid/Lattice 字段，对 `SmacPlanner2D` 无效；真正生效的是 `cost_travel_multiplier 4.0`、`tolerance 0.5`、`allow_unknown true`、`max_planning_time 4.5`、`smooth_path true`。另：他们不是「2025 是 SmacHybrid」——2025 的 `nav2_params` 已用 `SmacPlanner2D`，只是同时留了 Hybrid 字段。
8. **§1 表「局部控制」行 —— 补三个漏掉的关键值**：`temperature: 0.25`、`gamma: 0.008`、`reset_period: 1.0`（Humble 专属）、`prune_distance: 1.7`、`ax/ay_max 5.0`、`az_max 3.5`、`ObstaclesCritic{repulsion 1.5, critical 20.0, consider_footprint false, collision_margin_distance 0.1}`、`CostCritic{trajectory_point_step 2, near_goal_distance 0.5}`。这些正是 `a7299b60a→a1c33ad80→4651c925a` 三轮「消目标点震荡 + 提速」的产物。
9. **§1 表「局部控制」行 / §4 骨架 —— 补 goal checker**：`xy_goal_tolerance: 0.25`（`a1c33ad80` 注释：「放宽到 25cm，**高速 omni 机器人 10cm 太严导致绕圈**」）与 `yaw_goal_tolerance: 6.28`。`nav:=mppi` 若不放宽 xy 容差，会复现他们踩过的绕圈。
10. **§1 表「局部障碍表示」行 —— 补 STVL 的层级覆盖陷阱**：layer 级 `obstacle_range: 3.0` 与 source 级 `8.0/9.0` 并存，生效的是 source 级；以及 `decay_model: 0`(linear)、`voxel_size: 0.05`、`observation_persistence: 0.0`、`combination_method: 1`、`track_unknown_space: true`、`model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、`transform_tolerance: 0.2`。**特别是 `model_type: 1`** 我们计划里没写，这是最容易漏且最影响正确性的一个。
11. **§1 表「任务层」行 —— 三处与 XML 不符**：①`IsStuck→BackUp` 不成立：两棵 BT XML 里都没有 `IsStuck` 节点（它只在 `plugin_lib_names` 里），`ReactiveFallback name="FollowPathWithStuckDetection"` 下只有一个 `FollowPath`；②`BackUp` 只在 `navigate_through_poses_*` XML 里，且是 `backup_dist="1.0"` `backup_speed="1.0"`（**不是 1.5 m/s**）；③`navigate_to_pose_*` 的恢复动作只有清 global/local 代价图。§2 P4 与 §3 矩阵里「1.5 m/s」的表述要同步改。
12. **§1 表「任务层」行 —— 补两个 2026 特有的事实**：`RateController hz="3.0"` 确实存在；`RemovePassedGoals radius="0.7"`（只在多点 BT）；航点来自 `WPs/t1.csv`（3 点）+ RViz `waypoint_editor/WaypointEditorPanel`（外部包，`InteractiveMarkers` namespace `/interactive_marker_server`）；决策来自 `scripts/bt.sh` 轮询 `/serial/receive` 的 `game_type`（2→(4.55,−2.0)，3→(0,0)）。
13. **§1 表「哨兵语义」行 —— 表述需修正为「他们其实不自转」**。全分支 `spin_speed` 仅出现 5 次（全部在 `fake_vel_transform` 包内），**没有任何文件把它设为非零**，`slam/nav.launch.py` 都只传 `use_sim_time` ⇒ 他们实跑时 `angular.z ≡ 0`（`wz` 被丢弃），配合 `robot_base_frame: base_link_fake` + `yaw_goal_tolerance 6.28` 实现「只控位置」。所以：**`sentry_spin` 组合是我们的自研设计，不是「抄他们」**；另外要标明我们的语义与他们不同（我们 `spin_speed==0` 时直通、保留 `wz`）。
14. **§1 表「重定位」行 —— 结论正确，建议补证据与前提**。证据：`localization_launch.py` 的 `lifecycle_nodes = ['map_server']`（无 amcl）、`nav2_params.yaml` 无 `amcl:` 段、`slam.launch.py`/`nav.launch.py` 的静态 `map→odom`(z=0.05)、`mapper_params_async.yaml` 的 `transform_publish_period: 0.0`。前提：**必须在 `t1.yaml` 的 origin（`[-3.84, −6.05]`，图仅 10.65×10.2 m）处、同朝向启动**。
15. **§1 表「在线建图」行 —— 补 `transform_publish_period: 0.0` 及其后果**（SLAM 只改图不改位姿），并把 `mode: lifelong` 之外的关键参数补上（`map_update_interval 2.5`、`minimum_time_interval 0.5`、`minimum_travel_distance 1.0`、`max_laser_range 5.0`、`resolution 0.05`）。
16. **§1 表「LIO」行 —— 「先用 `lio:=pointlio` 顶替验证」这个推论需要加一个大前提**（**这是全篇最重要的修正**）：`small_point_lio` 的 PointCloud2 适配器**强制要求点云带 `tag`(uint8) 与 `timestamp`(float64, 秒) 字段**（`livox_pointcloud2.h`：`PointCloud2ConstIterator<uint8_t> out_tag(msg,"tag")`、`...<double> out_timestamp(msg,"timestamp")`，只保留 `(*tag & 0b00111111)==0` 的点），否则迭代器构造即失败。我们仿真链路的 `/livox/lidar` 是 CustomMsg、`/livox/lidar/pointcloud` 是普通 PointCloud2 ⇒ **不能直接换 LIO**。可选项：①走 `livox_custom_msg` 适配器（需编译期 `HAVE_LIVOX_DRIVER`，即装 `livox_ros_driver2`）；②给仿真点云补 `tag`/`timestamp` 字段；③只当参考实现读，不落地。另外它把 `header.frame_id`/`child_frame_id` 硬编码为 `odom`/`base_link`，且 **`twist` 全 0**（源码 TODO 注释）——任何依赖 `/Odometry.twist` 的模块（含我们自己可能的 CLOSED_LOOP 平滑、速度监视）都会读到 0。
17. **§2 P0 —— 补「先验图/静态桥的启动前提前提」与「STVL 0.5 s 衰减会擦薄结构」的风险**；并注明他们 `slam_toolbox` 侧 `max_laser_range: 5.0`，所以 `range_max 20` 的收益**主要在 costmap 与 scan 层**，不会让 SLAM 的 raster 图变大（把 P0 判据「`inf` 占比明显下降」的前提写清）。
18. **§2 P3 —— 「评估是否引入 `small_point_lio`」应补三条硬约束**：`save_pcd: true` 默认常开 + `ROOT_DIR` 写死源码目录；`map_resolution: 0.5`（比我们 0.05 粗 10 倍，对我们 0.40 m 半径车是否够用需实测）；上游确权（README 无 git URL；**推测**是 `Yancey2023/small_point_lio`，MIT）。
19. **§3 矩阵 —— 建议新增/改写两行**：①「`p2l` 惰性订阅 × A/B 无人订阅 `/scan`」→ 假故障（`subscriptionListenerThreadLoop` 只在有订阅者时才订阅 `cloud_in`）；②「`smoother:=savgol` × 我们自研 BT」→ 必须显式加 `<SmoothPath>`，「冗余」这一栏的理由应改成这个（与 TEB 无关的那半边结论保留）。
20. **§3 矩阵「高度带改到雷达面之上 × 我们场地矮墙」**：结论对，建议补上他们的真实数值（`base_link` 之上 `0.10`/`0.15`~`1.00`，且**高度判定在 `target_frame` 变换之后**做，源码可证），并注明我们墙 0.40 m 是反对照抄的量化依据。
21. **§4 骨架 —— 键名与来源修正**：`CostCritic` / `GoalCritic` / `PathFollowCritic` / `PathAlignCritic` 的权重键名是 **`cost_weight`**（不是 `weight`）；`CostCritic` 补 `cost_power: 1`、`collision_cost: 1000000.0`、`near_goal_distance: 0.5`、`trajectory_point_step: 2`；`ObstaclesCritic` 补齐；`smoother:=savgol` 补 `window_size: 7`、`poly_order: 3`、`refinement_num: 2`；`planner:=smac2d` 删掉 Hybrid 专用字段；`local_obstacle:=stvl` 补 `model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、`decay_model: 0`；`crop_box` 段加注释「坐标在 `livox_frame`；`leaf_size` 无效」。
22. **§5「明确不做的事」—— 建议把 §5.3 拆细并加两条硬理由**：①`get_package_share_directory('fast_lio')` 的死引用会让 `slam.launch.py` **启动即失败**（推断）；②`save_pcd: true` + `ROOT_DIR` 会往源码树写文件。另外把「不引入 ESDF/动态障碍」的理由从「他们也没做」升级为「2026 唯一的动态性只是 STVL 的 0.5 s linear decay，`esdf`/`frontier` 全分支 grep 0 命中」（更硬）。
23. **§6 验收方式 —— 建议加一条上游溯源记录**：本文 §A/§B 的 URL 与 commit SHA 可作为引用锚点（`485f2ebfa` HEAD、`a7299b60a`/`a1c33ad80`/`4651c925a` 消震荡三轮、`632fbb4eb` 砍重定位、`9d54b5ab8`+`d1c027958` 换 LIO、`9856cff66`/`9dfbeabdd` 做减法）。
24. **§7「关于『2026 用的是不是 FAST-LIO』」—— 三点补充**：①时间线可写死（`9d54b5ab8` 2025-11-18 remove FAST_LIO → `d1c027958` 2025-12-28 add small_point_lio → `a7299b60a` 2026-03-10 删 `FAST_LIO_ROS2/`）；②`CLAUDE.md` 确实是他们唯一设计文档（5,306 B，`a7299b60a` 新增、`485f2ebfa` 最后改），但它同时也是「自承 typo」的地方；③第 5 条「只借加法部分」的判断我完全同意，可再加一句量化依据：他们 2026 的有效创新几乎全在**参数层**（MPPI critics、inflation 梯度、STVL decay、高度带、BT 结构），唯一的结构层变化是**换 LIO** 与**砍重定位/地面分割**。
25. **建议新增一节 §8「本文与 `cod_nav_2026_deep_dive.md` 的对应关系」**：把本文件 §C（可采纳）与 §D（不采纳）逐条映射到该计划的 P0–P4，避免两份文档各自漂移。

---

## F. 未能确定 / 读不到的东西（明确列出）

1. **`livox_pointcloud2.h` 的实时性未能 100% 复核**（首次抓取成功、后续 SSL 重试失败）：该文件的核心逻辑我已读到（`tag`/`timestamp` 字段、`(*tag & 0b00111111)==0` 过滤、`SensorDataQoS`），但**未做第二次独立复核**。其余 40 余个 `small_point_lio` 文件也未逐行读完（`small_ivox.h`、`eskf.h`、`so3_math.h`、`pcd_io.cpp`、`voxelgrid_sampling.cpp` 只读了接口与文件头注释）。
2. **上游仓库确权未完成**：`small_point_lio/README.md` 里**没有任何 git URL**（只有邮箱 `1709185482@qq.com`、QQ 群 1070252119、bbs 链接）。检索到的最可能候选是 `Yancey2023/small_point_lio`（**未用 API 确权**）。另外 `small_gicp` 在 CI 里被安装，但 `CMakeLists.txt` **不 find 它**，属冗余步骤（未确认是否为历史残留）。
3. **谁发布 `base_link→livox_frame`**：分支内**没有** `robot_state_publisher` 的启动、`bringup.launch.py`/`lio_only.launch.py` 等已被删；`small_point_lio` 又强依赖该 TF（`lookupTransform(lidar_frame, "base_link")` 失败即丢帧、无 odom/TF）。唯一仓库内的定义是 `nav_bringup/urdf/carto.xacro`（`xyz="0.0 -0.045 0.3"`，**未被任何 launch 引用**）和 `simulation_waking_robot.xacro`（父链接是 `chassis`，同样未引用）。⇒ **无法确定他们在实机/仿真里从哪加载 URDF 与静态 TF**，这直接影响「他们那套能不能被我们直接复现」。
4. **`nav_bringup/map/t1.pgm` 的像素内容未读**：只读了 header（`P5 213 204 255`）与 `t1.yaml`（`origin [-3.84, -6.05, 0]`、`resolution 0.05`、`trinary`）⇒ 推得 10.65 m × 10.20 m，未核对实际占据/自由分布。
5. **`nav_bringup/rviz/cod_nav.rviz` 的 39 KB 全文未逐行读**：只提取了 display 名称、话题（`/scan`、`/global_costmap/costmap(+_updates)`、`/local_costmap/costmap(+_updates)`、`/local_costmap/voxel_grid`、`/plan`、`/local_plan`、`/lookahead_point`、`/curvature_points_marker_array`、`/trajectories`、`/slam_toolbox/graph_visualization`、`/crop_box_marker`、`/global_costmap/published_footprint`，Fixed Frame `map`，面板 `waypoint_editor/WaypointEditorPanel`，`InteractiveMarkers` namespace `/slam_toolbox` 与 `/interactive_marker_server`）。
6. **`.idea/editor.xml`（39,943 B）与 `nav_bringup/.idea/editor.xml`（39,943 B，同大小）未读**：判定为 IDE 元数据、无技术价值；若其中含调试运行配置（Run/Debug configuration 常被 Copilot/IDEA 写入）可能有额外信息，我**没有**核实。`.idea/copilot.data.migration.*.xml`、`.idea/cod_nav.iml`、`.idea/misc.xml`、`.idea/vcs.xml`（内含一条失效的 `patchwork-plusplus` vcs 映射）也未细读。
7. **外部依赖的行为无法核实**（不在分支内，只能从我们的 launch/params 反推）：`livox_ros_driver2`（他们 CI 用 `qza36/livox_driver2_ros2` fork）、`cod_serial_ul26`（吃 `/aft_cmd_vel`、发 `/serial/receive` `game_type`）、`spatio_temporal_voxel_layer`（2026 只留参数，subtree 副本当天删掉）、`waypoint_editor`、`realsense2_camera`、`slam_toolbox`、`nav2_smac_planner`/`nav2_mppi_controller`。⇒ 这些包的话题名/QoS/频率我**没有**独立证据。
8. **`pointcloud_to_laserscan` fork 与上游的差异未逐行 diff**：我在意的是 `package.xml` 版本号与他们的本地改动量（只读了 `README.md`、`CHANGELOG.rst` 头部、`CMakeLists.txt`、两个 sample launch 与 node 源码的关键段落）。我们 bench 里的 p2l 若是同源同版本，则 §C5/§C6 的移植是纯参数改动；**这一点未验证**。
9. **所有「会导致失败」的判断都是源码/文档推断，未实测**：①`get_package_share_directory('fast_lio')` 在包缺失时抛 `PackageNotFoundError` 导致 `slam.launch.py` 起不来；②`small_point_lio` 因 `tag`/`timestamp` 字段缺失而无法消费我们的仿真点云；③STVL `voxel_decay 0.5` linear 会擦掉 0.40 m 矮墙；④`angular.z ≡ 0` 时 MPPI 的 `wz` 被丢弃导致朝向不受控。**我没有运行任何代码**（本任务禁止执行，且我在受限沙箱内）。
10. **branch 的 CI 状态未知**：`.github/workflows/ci.yml` 只对 `push: rmul2026` 与 `pull_request: master` 触发，`skip-tests: true`，且第 3 步 `cd ws_liovx/src/...`（`liovx` 拼写错误，目录其实是 `ws_livox`）⇒ 该 CI 的第 3 步**很可能一直失败**（**推断**，未查 Actions 运行记录）。
11. **`bt.sh` 的引入时间未定位**：它不在最近 100 条提交的改动清单里（说明早于 2024-12 或通过 merge 带入），我未做完整 `git log --follow`（无 clone）。
12. **`LICENSE`**：`BSD 3-Clause`，`Copyright (c) 2025, Zhiang QI`（`README.md` 与 `CLAUDE.md` 里写的却是「2025 COD Team」）；`small_point_lio/LICENSE.txt` 是 MIT（`Copyright (C) 2025 Yingjie Huang`）——两套 license 并存，**未做合规判断**。

---

### 附：本文件所有引用的原文 URL（可直接点开复核）

| 用途 | URL |
|---|---|
| 分支树（recursive） | <https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1> |
| 提交史 | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100> |
| 架构文档 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/CLAUDE.md> |
| README | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/README.md> |
| 主 launch（建图+导航） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py> |
| 先验图 launch | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/nav.launch.py> |
| Nav2 参数 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/nav2_params.yaml> |
| slam_toolbox 参数 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/params/mapper_params_async.yaml> |
| 定位 launch（只有 map_server） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/localization_launch.py> |
| BT（到点 / 多点） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml> |
| 航点 / 地图 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/WPs/t1.csv> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/map/t1.yaml> |
| URDF（未被引用） | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/urdf/carto.xacro> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/urdf/simulation_waking_robot.xacro> |
| 裁剪盒源码 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/cpp_lidar_filter/src/filter_node.cpp> |
| LIO 配置 / 节点 / 适配器 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/config/mid360.yaml> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio_node.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/lidar_adapter/livox_pointcloud2.h> |
| LIO 参数声明 / 预处理 / 估计器 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/parameters.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/preprocess.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/small_point_lio/src/small_point_lio/estimator.cpp> |
| 伪底盘 | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/src/fake_vel_transform.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/fake_vel_transform/README.md> |
| p2l 节点与 sample | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/launch/sample_pointcloud_to_laserscan_launch.py> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/pointcloud_to_laserscan/launch/sample_laserscan_to_pointcloud_launch.py> |
| 决策脚本 / CI | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/scripts/bt.sh> · <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/.github/workflows/ci.yml> |
| 关键提交 | `a7299b60a`(MPPI+CLAUDE.md) · `a1c33ad80`(消震荡) · `4651c925a`(提速) · `485f2ebfa`(HEAD) · `632fbb4eb`(砍重定位) · `9d54b5ab8`(replace fastlio2) · `d1c027958`(add small point lio) · `9856cff66`(remove livox driver) · `9dfbeabdd`(use stvl) · `872adfafd` · `4de98f194`(realsense) |
| small_point_lio 上游候选（推测） | <https://relatedrepos.com/gh/Yancey2023/small_point_lio> · <https://bbs.robomaster.com/article/813022> |
