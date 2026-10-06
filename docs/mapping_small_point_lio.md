# `lio:=small_point_lio` 建图：接线审计、3D 导出路线、RMUC2026 实跑产物

> 本文只讲**建图**这一条链（`mode:=mapping lio:=small_point_lio mapper:=slam_toolbox`）：
> 接线对不对、3D 先验点云怎么导出来、在 `world:=RMUC2026` 上实跑出来的 2D/3D 产物是什么、
> 这些产物能不能直接喂给 `localization:=gicp`、以及**哪些还没验证**。
> 槽位本身（算法/参数/契约）见 `docs/lio_slots.md`；场地资产见 `docs/worlds.md`。

## 0. 一句话结论

* **接线是对的，一处都不用改**：建图模式下 `small_point_lio` 会起、`slam_toolbox` 会起、
  `/map` **只有一个发布者**（不需要"把旧图挪走"那种操作）、`odom→base_link` 也只有一个发布者。
* **3D 导出走得通，但任务书里的前提要更正**：这个 vendored 版本**有 `/map_save` 服务**
  （`small_point_lio_node.cpp:34-51`），不是"没有"。它要求**构造期** `save_pcd: true`
  （事后 `ros2 param set` 无效），落盘路径**硬编码**为 `ROOT_DIR/pcd/scan.pcd`
  （`ROOT_DIR=@CMAKE_CURRENT_SOURCE_DIR@` ⇒ 写进源码树，**没有任何路径参数**）
  ⇒ 本仓做法：配置里开 `save_pcd`，调服务后把文件**搬**到 `PCD/<world>_spl.pcd`。
  **不要**走"订阅 `/cloud_registered` 再拼图"那条路 —— 那个话题的点被**多乘了一个常值杆臂平移**
  （源码证据见 §2.3），拼出来整体偏 `(0.12, 0, 0.175)` m。
* **产物可用**：`map/RMUC2026_spl.pgm/.yaml`（新 2D 图）+
  `PCD/RMUC2026_spl.pcd`（新 3D 先验，21,387 点 / 0.68 MB）。
  用它们跑 `mode:=nav localization:=gicp` **P0 回归 PASS**，
  GICP `score=0.0018 m²`、**880/880 帧全采纳**、`align≈22 ms`、`map→odom` 稳定
  —— 比 STL 合成那份 PCD（0.0179 m²）**好约 10 倍**（§7）。
* 既有合成资产 `RMUC2026.pgm/.yaml` 与 `PCD/RMUC2026.pcd` **全程没被覆盖**（§6.3 有还原证据）。

---

## 1. 接线审计（静态 + 运行时证据）

被审计的组合：`world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox spin_speed:=0.0`。
下表 `file:line` 一律指 `src/rm_nav_bringup/launch/bringup_sim.launch.py`（除注明外）。

| 检查项 | 结论 | 依据（file:line） |
|---|---|---|
| LIO 节点在**建图模式**下也起 | ✅ 起 | `:496-521` 的 `GroupAction` 条件只有 `lio=='small_point_lio'`，**不含 mode** ⇒ 三种 mode 都起；参数文件 `:153-154`（`config/mid360_sim_tuned.yaml`）；`/Odometry→/odom` 重映射 `:513` |
| 参数文件真的被加载（不是猜） | ✅ 日志有横幅 | `:499-501` 的 `LogInfo` 打 `lio:=small_point_lio 使用的参数文件 = .../mid360_sim_tuned.yaml`（实测输出见下） |
| **不起** `lio_tf_adapter`（避免 `odom→base_link` 双发布者） | ✅ 已排除 | 条件 `:736-740` 显式 `!= 'small_point_lio'`；节点加入 launch `:934` |
| **不起** T1 回退静态桥（`camera_init→map`、`body→odom`） | ✅ 已排除 | `:764-769`（要求 `mode=='nav' and localization=='' and lio∉{none,cartographer,small_point_lio}`）；节点 `:771-787`、加入 `:925-926` |
| `map≡odom` 那条单桥**只在 nav+空 localization** 起 | ✅ 建图不起 | `:792-802`（条件含 `mode=='nav'`）、加入 `:928` ⇒ 建图模式下 `map→odom` 只有 slam_toolbox 发 |
| 它自己发 `odom→base_link` | ✅ 源码硬编码 | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:61-62`（frame 名）、`:98`（`sendTransform`）、`:80-81`（`/odom` 的 frame 名） |
| 它需要的 `base_link→livox_frame` 静态 TF 有 | ✅ URDF/RSP 提供 | `src/rm_nav_bringup/urdf/sentry_robot_sim.xacro:263-264`（`xacro:mid360 parent="base_link"`，origin `0.12 0 0.175`）；`robot_state_publisher` 由 `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py:132-135,197` 起。**没有这条 TF 它一帧都不出**（`small_point_lio_node.cpp:63-68` lookup 失败就 `return`，连 `/odom` 都不发） |
| Gazebo 不抢 `odom→base_link` | ✅ 关掉了 | `urdf/sentry_robot_sim.xacro:223-224`（`publish_odom_tf=false`）；真值里程计改名 `:220`（`odom:=odom_ground_truth`） |
| `mapper:=slam_toolbox` 拿到 `/scan` 与合理 TF 链 | ✅ | `:837-846` 起 `async_slam_toolbox_node`，条件 `:828-831`（`mode∈{mapping,slam_nav}` + `mapper=='slam_toolbox'` + `lio!='cartographer'`），`TimerAction(4 s)` `:935`；参数 `slam_toolbox/config/mapper_params_online_async_sim.yaml:13-16`（`odom_frame: odom`、`map_frame: map`、`base_frame: base_link`、`scan_topic: /scan`）、`:30`（`resolution: 0.05`） |
| `/scan` 上游链完整 | ✅ | `linefit` 地面分割 `:407-415` → `/segmentation/obstacle` → `pointcloud_to_laserscan` `:417-429`（`cloud_in:=/segmentation/obstacle`、`scan:=/scan`）；`config/laserscan_params.yaml` 里 `range_max: 10.0`、`target_frame: ""`（不建 TF 过滤器） |
| **没有** map_server 抢 `/map` | ✅ 建图模式不起 | `:690-710` 的独立 map_server include 条件 `:701-705` 含 `mode=='nav'`；`:698-700` 的注释就是这个坑（"若不判断 mode，map_server 会把磁盘上的旧 pgm 发到 /map，导致 map_saver_cli 存下旧图"）。`:807-809`、`:898-899` 是历史注释，**当前代码里建图模式确实没有第二个发布者** |
| 导航栈不启动（省 CPU） | ✅ | `nav_stack_condition` `:819-820`（只要 nav/slam_nav）、`start_navigation2` `:885-896` |
| `fake_vel_transform` 常开但无害 | ✅ | `:933` 无条件起；`spin_speed:=0.0` 时对 `/cmd_vel` 只是直通（`src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:80-82`），且**只在收到 `/cmd_vel` 时才转发**，不会周期性地往 `/cmd_vel_chassis` 写 0 |

**运行时证据**（`world:=RMUC2026 mode:=mapping lio:=small_point_lio`，日志在 `.tmp_cache/spl2026/run*/`）：

```
# ros2 node list（节选）：有 small_point_lio / slam_toolbox；**没有** map_server / lio_tf_adapter / nav2
/complementary_filter_gain_node  /fake_vel_transform  /gazebo  /ground_segmentation  /imu_plugin
/joint_state_publisher  /livox_frame_plugin  /mecanum_controller  /pointcloud_to_laserscan
/robot_state_publisher  /slam_toolbox  /small_point_lio

# ros2 topic info /map -v      ← 这是"会不会存错图"那个坑的判据
Publisher count: 1
Node name: slam_toolbox

# 参数文件横幅
[INFO] [launch.user]: lio:=small_point_lio 使用的参数文件 = .../share/small_point_lio/config/mid360_sim_tuned.yaml
```

⇒ **接线无需修复**。唯一为建图做的改动是 `config/mid360_sim_tuned.yaml` 的 `save_pcd: false → true`
（§2.4 说明为什么必须写在文件里、以及为什么对导航数值行为无影响）。

---

## 2. 3D 先验点云怎么导出来（本任务的核心 GAP）

### 2.1 三条路线与取舍

| 路线 | 可行性 | 判断依据 |
|---|---|---|
| **(a) 开 `save_pcd` + 调 `/map_save`** | ✅ **采用** | 服务存在（`small_point_lio_node.cpp:34-35`）；累积的点是**世界系**（`src/small_point_lio/small_point_lio.cpp:150-152` 交给回调的是 `pointcloud_odom_frame`）⇒ 直接就是地图 |
| (b) 订阅 `/cloud_registered` 逐帧拼 | ❌ 不用 | 该话题的点被多乘了一个常值杆臂平移（§2.3），拼出来整体偏 `(0.12, 0, 0.175)` m |
| (c) 改代码加"输出路径参数" | ❌ 不做 | 落盘路径是**编译期**常量，改它要动已经在跑、且已被别的槽位验收过的 vendored 包源码 |

### 2.2 路线 (a) 的细节与坑（全部有源码/实测依据）

1. **服务在**：`small_point_lio_node.cpp:34-51`，名字 `map_save`（相对名、节点在 `/` 下 ⇒ `/map_save`），
   类型 `std_srvs/srv/Trigger`。
2. **必须构造期打开**：`save_pcd` 在构造函数里读一次（`:24`），决定 `PointcloudMapping` 是否创建
   （`:31-32`，0.02 m 体素），并被服务 lambda 捕获（`:36-41`：`!save_pcd` ⇒ `success=false` +
   `"pcd save is disabled"`）。**`ros2 param set` 事后打开没有用** ⇒ 只能写进参数文件。
3. **落盘路径不可配**：`:48` `io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", ...)`，
   而 `ROOT_DIR` = `include/param_deliver.h.in` 的 `@CMAKE_CURRENT_SOURCE_DIR@`
   = `src/rm_localization/small_point_lio`（实测 `build/small_point_lio/include/param_deliver.h`）
   ⇒ **一定写进源码树**。本仓做法：`coverage_drive.py::save_3d()` 调完服务后轮询
   "文件出现且大小稳定"，再 `cp` 到 `--save-3d` 的位置，并删掉源码树里那份
   （包内 `.gitignore` 也有 `*.pcd`，不会脏 git）。
4. **顺序坑（自己踩的）**：服务**立刻**回 `success=true`，真正的 `write_pcd` 跑在**分离线程**里
   （`:47-50`）⇒ ① 不能只看服务返回；② 更**不能**在服务返回后再删旧文件 —— 那条线程可能已经
   `fopen` 了，`unlink` 之后 fd 还有效但目录项没了，现象是
   "日志里 `save pcd success`，但文件哪儿都找不到"。第一版工具就这么栽的，现已改成**先删后调**。
5. **`write_pcd` 返回值被丢弃**（`:48` 不看返回值），而它在**点云为空**时直接 `return false`
   且**不建文件**（`src/io/pcd_io.cpp::GenerateHeader`）⇒ 必须**校验文件真的出现**，不能信日志。

### 2.3 为什么不用 `/cloud_registered`（源码级证据）

`/cloud_registered` 的发布代码（`small_point_lio_node.cpp:101-170`）在发之前做了一次**多余的**变换：

```
:109  lookupTransform("base_link", lidar_frame, t)   // = T_base←livox，本 URDF 里是常值 (0.12,0,0.175)
:157  transformed_point = R * point + T              // 但 point 已经是 **odom 系**（见下）
:127  msg.header.frame_id = "odom"                   // 却标成 odom
```

而喂给它的点确实已经是世界系：`small_point_lio.cpp:98` 逐点算 `R_world*point_imu + p_world`，
存进 `pointcloud_odom_frame`，`:150-152` 原样交给回调。
⇒ 发布出来的点 = **真实 odom 系点 + 常值 (0.12, 0, 0.175)**（两帧同姿态 ⇒ R≡I），
拿它拼图会整体错位一个杆臂。`/map_save` 累积的是**没被这次变换污染的那份**
（`:172-175` 直接用回调的 `pointcloud`）⇒ 用它。

> **实测交叉验证**（同一次跑，两边都降到 0.05 m 体素后比 bbox，工具：
> `coverage_drive.py --dump-registered` + `pcd_stats.py`）：
>
> | | `/map_save`（正确） | `/cloud_registered` 累计（被污染） | 差值 |
> |---|---|---|---|
> | 体素数 | 280,316 | 287,204 | — |
> | bbox min | (-27.017, -9.595, -0.547) | (-26.900, -9.600, -0.400) | **(+0.117, -0.005, +0.147)** |
> | bbox max | (4.068, 8.078, 1.540) | (4.150, 8.050, 1.700) | (+0.082, -0.028, **+0.160**) |
>
> 差值 ≈ 预测的常值杆臂 `(0.12, 0, 0.175)`（z 略小是点分布造成的）⇒ **源码推断被实测证实**：
> `/cloud_registered` 拼出来的图整体多偏一个杆臂，**要用只能用 `/map_save`**。

### 2.4 打开 `save_pcd` 对导航有没有影响？**没有**（为了 P0 验收站得住，专门核过）

`save_pcd` 只出现在**节点层**：`node.cpp:24`（读参数）、`:31-32`（建一个体素容器）、
`:36-41`（服务开关）、`:101/172-175`（每帧往容器插一次点）。**估计器一层都不读它**
（`src/small_point_lio/*` 里没有 `save_pcd`）⇒ 滤波/建图数值行为按构造不变；代价只有一个
0.02 m 体素哈希表插入（~3500 点 @8 Hz ≈ 2.8 万次/s）与内存。
本仓把这条改动写进 `config/mid360_sim_tuned.yaml`（`save_pcd: true` + 大段注释），
`mid360_sim.yaml`（契约版）保持 `false` 不动。

---

## 3. 跑图工具（新增，全部可离线复跑）

| 工具 | 干什么 | 关键设计 |
|---|---|---|
| `tools/scripts/mapping/coverage_route.py` | 从**先验 2D 栅格图**生成"保证不跨半场、不撞墙"的覆盖路线（JSON） | `distance_transform_edt` 求可行驶掩码 → `ndimage.label` 取**起点连通域**（这一步就是"不跨半场"的硬保证）→ 割草机栅格目标 → 相邻目标间 **A\*** 连成合法折线 → RDP 抽稀；打印面积/长度/耗时估计，可出 PNG 俯视图 |
| `tools/scripts/mapping/coverage_drive.py` | 跟踪路线 + 全程体检 + 存 2D/3D | **MultiThreadedExecutor**（避开 `/tf` 80 Hz 饿死 `spin_once` 那个测量陷阱）、`/odom` 与 `/odom_ground_truth` **按仿真时间配对**算 ATE、纯追踪 + 航向消歧的路径跟随、`/map_save` + `map_saver_cli` 落盘、体素图导 `/cloud_registered`（A/B 证据） |
| `tools/scripts/mapping/pcd_stats.py` | PCD 体检 / 体素下采样 / 两份对比 | 支持 `binary/ascii/binary_compressed`；`--out` 写**与本仓既有资产同构的 8 字段**布局（`x y z intensity normal_x normal_y normal_z curvature`）；`--compare` 直接给 bbox/质心差 |
| `tools/scripts/mapping/run_mapping_headless.sh` | 一条命令跑完：launch → 等链路 → 静态核对 → 跑图 → 存图 → 体检 | 无头隔离（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、独立 `GAZEBO_MASTER_URI`、`unset DISPLAY`）；**全部在同一个 bash 调用里**（本环境每条命令一个 PID namespace，跨调用看不见进程） |

`PathFollower`（纯追踪 + 航向消歧）一共踩了 5 个坑，都写在代码注释里，供以后复用：
① `i` 是"下一个航点"⇒ 当前段是 `i-1`；② 弧长必须单调；③ 候选段全都远（>1 m）时**不许**
取"最小距离那个"；④ 折返路线下近平行 lane 的投影歧义要用**航向点积**消歧；
⑤ 卡死判据不能只看"到目标的距离变小"（会把原地旋转误判成卡死）。

---

## 4. 实测环境与命令

```bash
# 环境（每次都必须这样隔离；沙箱里 $HOME 只读，gzserver 建 ~/.gazebo 会 SIGABRT）
WS=/home/weicheng/HzMi_rmsimulation
HOME=/tmp/gzhome-spl ROS_DOMAIN_ID=93 GAZEBO_MASTER_URI=http://127.0.0.1:11503
unset DISPLAY

# 一条命令跑完整个建图（launch + 覆盖路线 + 存 2D/3D + 体检）
bash tools/scripts/mapping/run_mapping_headless.sh \
  --lio small_point_lio --tag spl --domain 93 --port 11503 \
  --map-yaml src/rm_nav_bringup/map/RMUC2026.yaml \
  --speed 0.25 --timeout 700 --clearance 0.40 --lane-spacing 2.5 --goal-step 1.5 \
  --save-2d src/rm_nav_bringup/map/RMUC2026_spl \
  --save-3d-raw .tmp_cache/spl2026/raw/RMUC2026_spl_raw.pcd \
  --final-pcd src/rm_nav_bringup/PCD/RMUC2026_spl.pcd --pcd-voxel 0.10
```

> `/map_save` 与 `map_saver_cli` 的**交互式**等效命令见 §11。

---

## 5. 实测结果

数字全部来自 `.tmp_cache/spl2026/` 下的 JSON/日志，可逐个核对。共跑了 4 次（都记录了失败原因）：

| | `diag`（直线诊断） | `run1`（LIO 闭环 + 直冲控制） | `run4`（GT 闭环 + 纯追踪；**产物用这一次**） |
|---|---|---|---|
| 路线 | 单段 6.7 m 直线 | NN 覆盖 60.6 m | 宽区覆盖 52.4 m |
| 实走 / 计划 | 7.45 / 6.72 m | 40.4 / 50.6 m（+7 m 原地/绕行） | 11.4 / 52.4 m（§5.3 卡住） |
| RTF（驱动段） | 0.783 | 0.767 | **0.762** |
| `/odom`、`/livox/*`、`/scan` | 7.84 / 7.8~8.0 Hz | 7.67 Hz | 7.62 Hz |
| LIO 位移/真值（终值） | **0.994** | 0.998 | **0.996** |
| LIO 轨迹长度比 | **0.994** | **1.50** ← 抖 | 1.29 |
| ATE（RMSE / **max**） | 0.082 / 0.187 m | 0.224 / **1.606 m** | **0.045 / 0.066 m** |
| `/map` 已知面积 | 20.7 → 61.0 m² | 20.8 → 104.9 m² | 20.6 → 65.5 m² |
| 2D 图"墙召回"（±0.20 m） | — | 0.63 | **1.01** |
| 占用格落在空闲区内部比例 | — | 0.21 | **0.13** |
| 结论 | LIO 在平滑直线段很好 | LIO 被"频繁原地转"带坏，图糊 | **图干净、LIO 好**，但走得不远 |

### 5.1 LIO 质量（对真值，MultiThreadedExecutor 采样、按仿真时间配对）

* **平滑直线（`diag`）**：ATE RMSE **0.082 m**、max 0.187 m、轨迹长度比 **0.994**
  ⇒ 与 `docs/lio_slots.md` §5 的离线重放结论一致（同一份 tuned 参数）。
* **纯追踪 + 平滑转弯（`run4`）**：ATE RMSE **0.045 m**、**max 0.066 m**、
  `z_err` 均值 -0.233 m / std 0.011 m（z 是常值坐标系差，不是漂移）。
* ⚠️ **原地/大角速度转弯（`run1`）**：轨迹长度比涨到 **1.50×**（LIO 40.7 m vs 真值 27.1 m）、
  ATE max **1.61 m**，2D 图墙面被"糊"成 ~1 m 宽带（中位 chamfer 5 cm、p90 1.12 m、墙召回 0.63）。
  机理与 `config/mid360_sim_tuned.yaml` 头部① 是同一件事：仿真插件把**一帧内所有点的
  `offset_time` 置 0**，帧内 126 ms 的旋转没被补偿 ⇒ 转得越快、点云越歪。
  ⇒ **给这个槽位跑建图，应当尽量少打大角速度**（本仓工具因此改成纯追踪 + 末尾才做原地旋转）。

### 5.2 2D 图增长与"存错图"风险

* `/map` 会随运动增长：`diag` 20.7 → **61.0 m²**（只走 7.4 m）；`run4` 20.6 → **65.5 m²**。
* **存图没有存错的风险**：建图模式下 `/map` 的发布者只有 `slam_toolbox`（§1 的
  `ros2 topic info` 实测 `Publisher count: 1`），磁盘上那份旧 `map/RMUC2026.pgm` **压根没有
  节点在读**（map_server 只在 `mode:=nav` 起，`:701-705`）。
  ⇒ 本次**不需要**"把旧图挪走"这种操作；`map_saver_cli -f .../RMUC2026_spl` 存下来的
  就是刚建的那张（§6.1 的墙召回 1.01 也证明它不是旧图）。

### 5.3 过程不干净的地方（如实记录）

1. ⚠️ **仿真底盘会在"窄口/收窄处"把车硬卡住**：`run1`/`run3`/`run4` 都在
   **map≈(0.5, 3.0~6.2)**（world≈(11.4, 5.5~8.7)，北侧通道）出现"任何方向都不动"的卡死
   —— `/odom` 与 `/odom_ground_truth` 同时不动（不是采样问题），而 `/clock` 照常推进
   （实测 sim 170.2→187.7 s / 墙钟 25 s ⇒ **不是仿真停了**）。
   而 STL 在那一带**是平的**（heightmap lift ≤ 1 cm）、先验图也给 ≥0.30 m 余量
   ⇒ 与地图/LIO 无关，是**仿真接触/网格**层面的问题（未定位根因，登记在 §9）。
   规避：把路线限制在更宽的区域（`--clearance 0.60`，`run4` 就是这么跑的）。
2. ⚠️ **`run1` 的图虽然覆盖更大（104.9 m²）但几何是糊的**（墙召回 0.63、占用格 13~21% 落在
   空闲区内部、p90 chamfer 1.12 m）⇒ 最终产物选了 `run4`（覆盖小一些、但墙召回 1.01）。
   两套都在 `.tmp_cache/spl2026/run1/artifacts/` 留了备份，可随时 A/B。
3. ⚠️ **`run4` 的"跑图"是用仿真真值闭环的**（`coverage_drive.py --pose-source gt`）：
   仿真里 `/odom_ground_truth` 是 planar_move 发的**世界位姿**，工具会把它搬进 spawn(map) 系
   （静止段取 origin＋初始 yaw）再用作控制反馈。这么做的理由：`run1` 证明"用 LIO 自己的
   `/odom` 闭环 + 大角速度"会把定位带坏（§5.1），而本次要交付的是**先验图的质量**
   ⇒ 让"跑图"这一段像人拿遥控器一样平滑。**代价/含义要看清**：LIO 的质量不再由"能不能开到"
   间接体现，而是**单独用 ATE 报**（`diag` 0.082 m / `run4` 0.045 m，都在 §5.1）。
   实车/真实用法下没有这条真值，只能用 LIO 自己的 `/odom`（`--pose-source lio`，默认值），
   此时建议：速度 ≤0.25 m/s、纯追踪、少打原地转（附录 §5.1 的退化机理）。
4. ⚠️ `run4` 的 2D 图**只覆盖出生点半场里机器人实际走到/看到的部分**：目标点
   `(-6.0, 3.5)`（合成图那次 P0 回归用的点）在新图上是 **unknown**（格值 205）⇒
   新图的 P0 回归必须换一个 free 目标（本次用 `(0.5, 3.0)`，仍是出生点半场内的 free 点）。

---

## 6. 两个产物（路径 + 统计）

| 产物 | 路径 | 统计 |
|---|---|---|
| 2D 栅格图 | `src/rm_nav_bringup/map/RMUC2026_spl.pgm` + `.yaml` | 237×265 @0.05 m，`origin=[-8.36, -6.30]`，62,820 B；已知 65.5 m²（其中占用 1,493 格 = 3.7 m²）；墙召回 1.01、内部误占 13% |
| 3D 先验点云 | `src/rm_nav_bringup/PCD/RMUC2026_spl.pcd` | **21,387 点 / 0.68 MB**（原始 587,770 点 / 7.05 MB → 0.10 m 体素）；bbox `x[-25.62, 3.44] y[-8.41, 7.02] z[-0.438, 1.304]`；8 字段布局（与既有资产同构）；地面 z≈**-0.339** |

### 6.1 2D 图对照（相对 STL 合成图）

* 尺寸/口径：`resolution 0.05`、`mode: trinary`、`free_thresh 0.25`、`occupied_thresh 0.65`
  —— 与 `map/RMUC2026.yaml` **同一口径**（`map_saver_cli --free 0.25 --occ 0.65`）。
* **map 系与合成图同源**：新图 `origin=[-8.36,-6.30]` 与合成图 `origin=[-25.925,-9.425]`
  都是"**出生点相对系**"（合成图原点 = 场地左下角 − 出生点；slam_toolbox 的原点 = 出生点），
  所以 `amcl_init_x/y = 0.0`、gicp 的 `initial_pose = [0,0,0]` 这些约定**都不用改**。
  实测出生点在两套坐标里的偏差：LIO 对齐平移 = `(10.9435, 2.5285)` vs 真值 `(10.925, 2.525)`
  ⇒ **1.9 cm / 0.4 cm**。
* 墙召回（新图占用格 vs STL 墙线 ±0.20 m）= **1.01**，占用格里只有 13% 落在空闲区内部
  ⇒ 墙是"薄且对得上"的，不是 `run1` 那种糊带。

### 6.2 3D 点云与合成 PCD 的对照

| | 合成 `PCD/RMUC2026.pcd`（未被覆盖） | 新 `PCD/RMUC2026_spl.pcd` |
|---|---|---|
| 点数 / 体积 | 26,618 点 / 0.85 MB | 21,387 点 / 0.68 MB |
| 0.10 m 体素数 | 21,161 | 21,387（**+1.1%**） |
| bbox（map 系） | `x[-25.50,3.65] y[-8.93,7.12] z[-0.14,0.54]` | `x[-25.62,3.44] y[-8.41,7.02] z[-0.44,1.30]` |
| 地面 z | **-0.06**（与 `initial_pose=[0,0,0]` 精确自洽） | **-0.339**（LIO 的 odom 系原点在 lidar/IMU 上，比 base_link 高） |
| 来源 | STL 采样（"地面以上薄壳" + 两档体素） | **真实 LiDAR 建图**（LIO 位姿 + 0.02 m 体素均值，本仓下采样到 0.10 m） |

⇒ 两者**xy 尺度一致到 0.1 m 量级**（bbox 差 ≤0.5 m，体素数差 1.1%），
z 基准差 **0.28 m**（`-0.06` vs `-0.339`）—— GICP 靠
`max_correspondence_distance: 1.5 m` 吸收（§7 实测收敛后 `map→odom.z = -0.13`）。

### 6.3 既有资产没被覆盖（复核证据）

```
$ sha256sum src/rm_nav_bringup/PCD/RMUC2026.pcd          # GICP 验证跑完"还原"之后
1a3d3b74ffbd92c947a2db04822437fca03c1324d14372e02d79ad37465e5895
$ git status --porcelain -- src/rm_nav_bringup/map src/rm_nav_bringup/PCD
 D src/rm_nav_bringup/map/RMUC2026.pgm        # ← 本次开跑**前**就是这个状态（见 §9 第 7 条）
 D src/rm_nav_bringup/map/RMUC2026.yaml
?? src/rm_nav_bringup/PCD/RMUC2026_spl.pcd
?? src/rm_nav_bringup/map/RMUC2026_spl.pgm
?? src/rm_nav_bringup/map/RMUC2026_spl.yaml
```

---

## 7. GICP 验证（用新产物跑 `mode:=nav localization:=gicp`）

⚠️ **launch 把资产名硬编码为 `<world>.pgm/.yaml` 与 `PCD/<world>.pcd`**
（`bringup_sim.launch.py:168`、`:230`），**没有任何 launch 参数能改文件名**。
所以"用新产物验证"的**最小改动**就是**临时换文件**（脚本：`.tmp_cache/spl2026/run_gicp_validate.sh`，
它先备份 + 记 sha256，跑完逐字节还原）：

```bash
cp src/rm_nav_bringup/map/RMUC2026_spl.pgm  src/rm_nav_bringup/map/RMUC2026.pgm
cp src/rm_nav_bringup/map/RMUC2026_spl.yaml src/rm_nav_bringup/map/RMUC2026.yaml
cp src/rm_nav_bringup/PCD/RMUC2026_spl.pcd  src/rm_nav_bringup/PCD/RMUC2026.pcd
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav \
  lio:=small_point_lio localization:=gicp nav:=rpp planner:=navfn spin_speed:=0.0 nav_rviz:=False
python3 tools/scripts/regress/nav_smoke_regression.py --goal 0.5 3.0 --localization RMUC2026-spl
```

实测（`.tmp_cache/spl2026/gicp/`）：

```
GICP 侧（新 PCD，21,387 点）：
  地图点数：原始 21387 → 去 NaN 21387 → 体素 0.100 m 后 21387
  [status] 采纳 880/880 帧 | 最近 score=0.00172 m² | align=22.3 ms
           source=4404 点（leaf 0.050）→ target=21387 点（leaf 0.100）
           map→odom x=-0.112 y=0.007 z=-0.129 yaw=0.02°

P0 回归（--goal 0.5 3.0 --localization RMUC2026-spl）：
  ✅ PASS
  · 地图 0.050 m/格 237x265 origin=(-8.36, -6.30)；目标落格 (177, 185) = free (value=0)
  · 结果 status=SUCCEEDED 用时=42.0s d_min=0.0m recoveries=3
  · 命令链 max|v|,|w|: nav=(0.50,0.75) smooth=(0.50,0.75) chassis=(0.50,0.75) spin_speed=0.0
  · 真值: twist_max=0.750 pose_delta=3.584m
  · RTF=0.78 hz={pcloud 8.0, imu 79.3, scan 8.0, odom 8.0} tf_age=0.1
  · 稳定性：settled=True 等待=3.0s 漂移 xy=0.0069 yaw=0.0014
  · 首个断点：无
```

### 7.1 与"STL 合成 PCD + 合成 pgm"那次的对照（`docs/worlds.md` §5.3）

| 指标 | 合成先验（baseline） | **新 prior（small_point_lio 建图）** | 结论 |
|---|---|---|---|
| GICP target 点数 | 原始 26,618 → 0.10 m 后 21,161 | 原始 21,387 → 0.10 m 后 21,387 | 同量级 |
| **fitness score** | **0.01788 m²** | **0.0017~0.0018 m²** | **好 ~10×** |
| 采纳率 | 1082/1082 = 100% | **880/880 = 100%** | 同为 100% |
| align 单帧 | 21.1 ms | 21~23 ms | 相同 |
| `map→odom` 收敛值 | x=-0.119 y=-0.029 z=0.225 yaw=-0.65° | x=-0.112 y=0.007 **z=-0.129** yaw=0.02° | 平移修正同量级；**z 修正正是 §6.2 那 0.28 m 基准差的一半**（另一半被 source 点云自身的 z 分布吸收） |
| 定位稳定性（settle） | ❌ `settled=False`（30 s 窗口漂 0.053 m / 0.0129 rad） | ✅ `settled=True`（3 s 窗口漂 **0.0069 m / 0.0014 rad**） | 新图更稳 |
| 到点 | SUCCEEDED，94.1 s，recoveries=6，pose_delta 9.155 m | SUCCEEDED，**42.0 s**，recoveries=3，pose_delta 3.584 m | ⚠️ **目标点不同**（新图在那个点上是 unknown），时长/绕路不能直接比 |

⇒ **新 prior 是可用的**（PASS + fitness 好一个数量级 + 定位更稳）。
⚠️ 唯一要打折的地方：新图的**已知区域比合成图小**（65.5 m² vs 237.8 m² 的 free），
所以"图外/未知区"的目标点在新图下不可用（回归脚本会直接判 exit 3，不算导航失败）。

---

## 7.5 后续：2D 先验图**不再从 `/scan` 推**（2026-10-06）

本文的 2D 图（`map/RMUC2026_spl.pgm`）是 `slam_toolbox` 用 `/scan` 攒出来的。后续实测确认：
**这条来源天然看不见"可行驶的斜坡"** —— `/scan` 是 `linefit 地面分割 → pointcloud_to_laserscan`
（传感器系窄高度带）的产物，坡度够缓的斜面会被判成地面而**从 `/scan` 里消失**，
于是"3D 点云里清清楚楚的坡道，在 2D 图上要么空白、要么根本不在图窗内"。

⇒ **决定：2D 导航图改为从 3D 点云直接投影**，工具是
`tools/scripts/mapping/pcd_to_nav2_map.py`（判据：相对**局部地面**的高度 + 坡度闸；
缓坡保持 FREE），产物 `map/RMUC2026_cloud.pgm/.yaml`。
两套 2D 先验的区域计数 A/B、坡道段判定、`/scan` 丢帧 A/B、`map→odom` 跳变哨兵与
"怎么在这个 world 上建图而不把图跑歪"的规程，全部见
**`docs/mapping_2d_from_cloud.md`**。

---

## 8. 与 FAST-LIO 的 A/B（能比什么、不能比什么）

| 维度 | small_point_lio（本次） | FAST-LIO（`docs/worlds.md` §5.1/§5.3 既有数据） | 可比性 |
|---|---|---|---|
| 驱动剖面 | 覆盖路线（多腿 + 转弯），本工具 | 直线 6 s @0.4 m/s | ❌ 不同剖面 |
| LIO 位移/真值 | 0.994（直线段）/ 0.996（覆盖） | **1.002**（直线段） | ✅ 同量级（都 ≈1）；spl 在**多转弯**剖面下退化到 1.29~1.50 倍轨迹长度 |
| ATE | 0.045~0.082 m（平滑）/ 0.224 m（多转弯） | 未单独报（2D 图干净） | ⚠️ 只能定性 |
| `/map` 已知面积 | 20.6 → 65.5 m²（走 11.4 m） | 20.8 → 58.5 m²（走 2.6 m）/ 一次 74.2 m² | ⚠️ 半可比（走的路不同） |
| 3D 点云 | 21,387 点 / bbox `x[-25.62,3.44] y[-8.41,7.02]` | `/map_save` 22,947 点 / bbox `x[-14.64,14.45] y[-6.19,9.56]`（world 系） | ⚠️ 都是真实建图产物，但 ①spl 那份是 **0.10 m 体素下采样后**的点数、②FAST-LIO 那份是它自己 ikd-tree 的滤波粒度、③驱动时长/路径不同 ⇒ **点数不能直接比**；bbox 换算到同一坐标后可比重合度（两者都覆盖整场） |
| GICP fitness | **0.0017 m²**（真实建图 PCD） | 0.0179 m²（STL 合成 PCD，**不是** FAST-LIO 建图产物） | ⚠️ 关键差异：FAST-LIO 那条是**合成**资产，不是它自己建的图 ⇒ 严格说这一列不是"FAST-LIO 的建图质量" |
| **没做**的 | — | 用 FAST-LIO **自己建出来的 PCD** 跑 gicp 做同口径 A/B | ❌ 需要再跑一次 `lio:=fastlio` 覆盖建图 + 一次 nav+gicp（本次没做，登记在 §9） |

---

## 9. 已知坑 / 未验证

1. **仿真底盘会在窄口硬卡**（§5.3）：`world:=RMUC2026` 北侧收窄处（map≈(0.5,3~6)）实测三次把车
   卡死，与地图/LIO 无关（STL 平、sim 时钟正常推进、两个里程计同时不动）。
   **未定位根因**（候选：网格接触约束抵消 `SetLinearVel`/薄壳自锁/局部几何自锁）。
   规避：路线留在宽区（`--clearance ≥ 0.6`）。
2. **LIO 在频繁原地转/大角速度下退化**（§5.1）：本槽位固有短板（帧内旋转未补偿），
   不是本次改动引入。建图跑法应尽量平滑。
3. `save_pcd: true` **没有重跑 P0 导航回归**（`localization:=amcl` 那条）：改动对估计器只读
   （§2.4 源码依据），但要"严格验收"仍需重跑一次
   `tools/scripts/regress/nav_smoke_regression.py --localization RMUC2026-spl-amcl`。
4. `/cloud_registered` 的常值杆臂错位（§2.3）是**源码推断 + 量级核对**，没做逐点级证明。
5. `world:=RMUC2026` 的**另一半场**没有覆盖（0.45 m 窄口过不去，`docs/worlds.md` §6 已论证）。
6. 新 2D 图**已知区域较小**（65.5 m²）⇒ 图外/未知区的目标不可用；
   若要更大覆盖，得先解决 §9 第 1 条的卡死问题（或把跑法改成"多段短直线、每段之间手动回正"）。
7. ⚠️ **`map/RMUC2026.pgm/.yaml` 在本次工作开始前就已经被移走**（工作树里是 `D` 状态，
   旁边有 `RMUC2026.pgm.prior` / `.yaml.prior`，`cmp` 与 `git show HEAD:` **逐字节相同**）。
   本次**没有动它们**，也没动那两个 `.prior`；GICP 验证时临时占用过这两个文件名，
   跑完已还原成"不存在"（§6.3）。要恢复：`git checkout -- src/rm_nav_bringup/map/RMUC2026.{pgm,yaml}`。
8. `nav:=dwb/teb/mppi`、`localization:=icp/small_gicp/amcl` 用**新产物**都没跑（只有 `gicp` 跑了）。

---

## 10. 回滚

```bash
# 1) 关掉 save_pcd（一行；3D 导出能力随之关闭，导航不受影响）
sed -i 's/^        save_pcd: true /        save_pcd: false/' \
  src/rm_localization/small_point_lio/config/mid360_sim_tuned.yaml

# 2) 删掉本次新增的跑图工具与文档（或直接 git revert 本次提交）
git rm tools/scripts/mapping/{coverage_route.py,coverage_drive.py,pcd_stats.py,run_mapping_headless.sh} \
       docs/mapping_small_point_lio.md

# 3) 删掉新产物（既有合成资产 RMUC2026.pgm/.yaml + PCD/RMUC2026.pcd 全程没被覆盖）
git rm src/rm_nav_bringup/map/RMUC2026_spl.pgm src/rm_nav_bringup/map/RMUC2026_spl.yaml \
       src/rm_nav_bringup/PCD/RMUC2026_spl.pcd

# 4) 只有改了包内文件才需要重编
colcon build --symlink-install --packages-select small_point_lio
```

## 11. 用户交互式复现（launch + 遥控 + 存图）

```bash
# ① 起栈（带 RViz 看 /map 边建边长；lio_rviz 可看 /cloud_registered）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox \
  nav_rviz:=False lio_rviz:=True spin_speed:=0.0

# ② 另一个终端：遥控（⚠️ 插件没有命令超时，松手必须显式发零速）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_chassis
ros2 topic pub -r 10 /cmd_vel_chassis geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'          # 停

# ③ 存 2D（mapping 模式下 /map 只有 slam_toolbox 一个发布者，直接存就是你在建的图）
ros2 run nav2_map_server map_saver_cli -f src/rm_nav_bringup/map/RMUC2026_spl -t /map \
  --free 0.25 --occ 0.65 --ros-args -p save_map_timeout:=60.0

# ④ 存 3D（save_pcd 已在 mid360_sim_tuned.yaml 里打开；服务立刻返回，文件由后台线程写）
ros2 service call /map_save std_srvs/srv/Trigger "{}"
ls -la src/rm_localization/small_point_lio/pcd/scan.pcd       # 等它出现（别只看服务返回！）
cp src/rm_localization/small_point_lio/pcd/scan.pcd /tmp/spl_raw.pcd
rm src/rm_localization/small_point_lio/pcd/scan.pcd           # 源码树里别留垃圾
python3 tools/scripts/mapping/pcd_stats.py /tmp/spl_raw.pcd --voxel 0.10 \
  --out src/rm_nav_bringup/PCD/RMUC2026_spl.pcd
```
