> **STATUS（2026-10，仓库整理）**：本文 §0.3/§6.3「新增一条冻结的重力中间帧 `world_grav`（推荐）」的结论已被 `docs/frame_contract_v3.md` §2/§4（E1 估计器初始化对齐，`952a6da`）取代；其余内容仍有效。
# 重力对齐归谁：IMU/INS · LIO/SLAM · 重定位 · 感知 —— 四层归属调研

> **调研问题（用户原话）**：「这个（把倾斜安装的点云/位姿对齐到重力系）一定要额外算法处理吗？定位/重定位算法、感知算法没有以及包含处理好吗？」
> 即：**重力对齐是一件"额外算法"，还是 IMU/INS 滤波 · LIO/SLAM · 重定位 · 感知 四层里本来就该有人管的标准接线？每一层对"倾斜安装"到底自带不带？它假设了什么？**
>
> **标记约定（全文强制）**
> - **【引用】** = 有 URL 出处，关键处给原文英文引句（出处见 §7 引用清单）。
> - **【推断】** = 由代码/文档/约定**推出来**、但原文没有直说（附推理链）。
> - **【本仓实测】** = 在本仓代码/配置/文档里**查到**的事实（给文件:行）。
>
> **调研边界**：只做文献/源码调研，未改任何代码、配置、launch，未跑仿真。

---

## 0. TL;DR（先给答案）

1. **四层里"自带重力对齐"的只有两类**：**IMU 姿态滤波器**（只输出 `R_grav←imu`，yaw 任意，且**不改任何帧**）和**部分 SLAM**——**Point-LIO 初始化时把世界系 z 转到重力方向**（`Set_init`，无条件执行）、**Cartographer 3D** 有显式重力 EMA 对齐、**hdl_graph_slam** 有地面约束；而 **FAST-LIO 明确不转世界系**、**2D AMCL 结构上做不了**、**地面分割全部需要"z 朝上"或 `sensor_height` 先验**。→ §2
   > ⚠️ **对任务书前提的一处更正**：不是"FAST-LIO / Point-LIO 都不对齐"。两者**行为相反**：**FAST-LIO 不转**（那行转正的代码被注释掉了），**Point-LIO 转**（`kf_input.x_.rot = rot_init`）。源码与参数注释见 **C30**，详见 §2.3(a′)。
2. 因此在 **Nav2 + LIO + IMU** 的典型栈里，**"map 的 z 轴朝上"没有任何一个组件被文档指定为负责人**：Nav2 文档说 `map→odom` 由定位包提供，而 LIO 只提供 `odom→base_link`，且**FAST-LIO 系**的世界系就是**初始 IMU 系**（斜的）；AMCL 只给 (x, y, yaw)。⇒ **这是一个真实的"无主孤儿"（orphaned）**，不是我们没找对包。→ §3.5
3. **本仓不需要新算法。** 重力方向**已经在** `imu_complementary_filter` 的输出里【本仓实测】，也**已经存在** FAST-LIO 的 `grav` 状态里【引用】；缺的只是**把这条旋转写成一条 TF（初始化时算一次、然后冻结）**。REP-105 明确允许**中间帧**，并明确允许"只有加速度计时，在当前位置把 map 的 z 轴朝上初始化"。→ §6
4. 不能省的两件事：**初始化时算一次并且验收一次**（静止、地面平、残差 <1.5°），以及**yaw 不归重力管**（重力只定 2 个自由度，yaw 仍必须由地图/重定位给）。→ §1.2、§6.4

---

## 1. 数学：`R_grav←odom` 到底在算什么

### 1.1 定义

设 `odom`（LIO 世界系 / `camera_init`）中表示的重力**向下**单位矢量为 `ĝ_o`，则"上"为 `û_o = −ĝ_o`。定义

```
R ≜ R_grav←odom ∈ SO(3),   p_grav = R · p_odom
约束：  R · û_o = ẑ        ⟺        R · ĝ_o = −ẑ
最小旋转解： R = FromTwoVectors(û_o, ẑ)      （Eigen::Quaterniond::FromTwoVectors）
```

**这个最小旋转族在两处源码里能直接看到同构写法：**

- Cartographer `ImuTracker` 的核心三行【引用 C5】：
  ```cpp
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
  ```
  —— 即"把测到的重力方向转到 +z"，并且用 `CHECK` 把"z 分量 > 0.99"（≈8° 以内）当作不变量来守。
- FAST-LIO 初始化时把重力写成**世界系里的一个自由三维矢量**（不是 `(0,0,−g)`）【引用 C3】：
  ```cpp
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  ```
  注意被注释掉的第二行**正是**"顺手把世界系转正"的那一步——上游**选择不做**。

### 1.2 只有 2 个自由度：yaw 不归重力管

`û_o` 绕竖直轴的旋转不变 ⇒ `R` 只有 **2 个可观测自由度（roll、pitch）**，绕重力的 yaw 不可观测【推断，由 1.1 的约束式直接得出】。
这一点是后面所有"重力对齐 ≠ 定位"结论的根源：**重力对齐把 z 摆正，但不告诉你朝哪边**。yaw 必须另有来源（先验地图 / AMCL / Scan Context / 磁力计）。

### 1.3 谁算它、何时算、存在哪

| 计算者 | 输入 | 何时 | 结果存在哪 | 证据 |
|---|---|---|---|---|
| IMU 姿态滤波器（互补 / Mahony / Madgwick / EKF） | 加计 + 陀螺 | **连续**（每帧） | 只进 `sensor_msgs/Imu.orientation`；**不碰任何 TF 帧** | 【引用 C21/C22/C28】 |
| LIO 初始化（FAST-LIO / Point-LIO） | 前 N 帧加计均值 | **一次**（init） | 滤波器状态 `S2 grav`（世界系三维矢量） | 【引用 C3/C4】【本仓实测】`src/rm_localization/FAST_LIO/src/IMU_Processing.hpp:193` |
| SLAM 内部（Cartographer 3D） | IMU 重力低通 | **连续 EMA**（τ = 10 s） | 每个 node 的 `gravity_alignment`；`local` 帧被定义为重力系 | 【引用 C5/C6/C7/C8/C9】 |
| 图优化先验（hdl_graph_slam） | 每帧 RANSAC 地面 | 每帧检测 + 图优化 | 位姿图里的 floor 边 ⇒ 最终进 6DoF `map→odom` | 【引用 C10/C11/C12/C13】 |
| 静态/离线（人工或脚本） | 地面拟合 / 人工摆正 | **一次** | 一条 TF、或一个**预旋转过的 PCD**（把旋转烘进地图资产） | 【推断，见 §6.3】 |

> **一张表读出来的结论**：`R_grav←odom` 这个东西**从来没人"顺便"帮你写进 TF**。IMU 滤波只把它写进 `Imu.orientation`；SLAM 只把它写进自己的内部状态/图；要让它变成**帧**，必须有人做"状态 → TF"这一步【推断】。

### 1.4 三种等价的重力观测（任选其一即可算出 R）

| 观测 | 适用 | 备注 |
|---|---|---|
| 静止时加计均值 `a_up`（比力朝上） | 有 IMU、起始静止 | `R = FromTwoVectors(â_up, ẑ)`；FAST-LIO 就是这么取初值的【引用 C3】 |
| 地面平面法向 `n̂`（RANSAC/PCA） | 无 IMU，或作为**交叉校验** | `R = FromTwoVectors(n̂, ẑ)`；hdl_graph_slam 每帧做这件事【引用 C11】 |
| 图优化里的重力/地面先验 | 建图期 | 把 tilt 烘进 PCD ⇒ 之后 map 天然是重力系【推断】 |

---

## 2. 四层归属：谁自带重力对齐？

### 2.1 汇总表（一行一层）

| 层 | 自带重力对齐？ | 对"倾斜安装"的假设 / 行为 | 标准组件（可引用的实现） |
|---|---|---|---|
| **IMU / INS 滤波** | **是，但只到"姿态"为止**（连续给 `R_grav←imu`，yaw 任意） | **对安装姿态零假设**：装多斜都行，它估的就是"加计指向哪"。但它**只发 `Imu.orientation`，不改帧、不改世界系**；`two_d_mode` 的 EKF 还会把 3D 姿态**清零** | `imu_complementary_filter`、`imu_filter_madgwick`、`robot_localization`（ekf/ukf）【引用 C21/C22/C28】 |
| **LIO / SLAM** | **分家三派**：**FAST-LIO 不做**；**Point-LIO 做**（初始化时把世界 z 转到重力）；**Cartographer 3D / hdl_graph_slam 做**（EMA 重力 / 地面约束） | FAST-LIO：世界系 = **初始 IMU 系**，重力只是状态 ⇒ 斜装 ⇒ **地图整体就是斜的**。Point-LIO：初始化用 `rot_init` 把测到的重力对到配置的 `gravity`（默认 `0 0 −9.81`）⇒ **世界 z 朝上、yaw 任意**。Cartographer 3D：假设有 IMU、tracking frame 与 IMU 同向、且愿等 10 s EMA。hdl：假设**看得见地面**且 odom 已接近竖直（法向 10° 闸门） | FAST-LIO、Point-LIO、Cartographer、hdl_graph_slam【引用 C1–C13、C30】 |
| **重定位** | **2D 的不行，3D 的"能但不负责"** | AMCL：`Pose2D`、纯 2D 波束模型，**把斜扫描当水平扫描**；它会把 base 摆平（roll/pitch 被塞进 `map→odom`）。3D 重定位器（Scan Context + ICP/GICP）：能输出 6DoF `map←base`，但**结果只和先验地图一样正**，且初值敏感 | `nav2_amcl`、`beluga_amcl`、`slam_toolbox`、`icp_registration`、`gicp_registration`（本仓 6 槽）【引用 C14–C17】【本仓实测】 |
| **感知** | **不是"自带"，是"要求"**：几乎全部**要求 z 朝上**，或要求 `sensor_height` 先验 | linefit：`sensor_height` 是**硬前提**；Patchwork++：能估平面，但 GLE 用 `u_z=[0,0,1]` 判"竖直度"；TRAVEL：球投影 + 图，同样以 z 为"上" | `linefit_ground_segmentation`、`patchwork-plusplus`、`TRAVEL`【引用 C23–C26】 |

### 2.2 IMU / INS 滤波层：**是，但只输出姿态，不出帧**

**它做了什么**：融合加计+陀螺，输出一个"世界系 z = 重力、yaw 任意"的姿态四元数。本仓 vendored 的 `imu_complementary_filter` 头文件里，加计输入的注释就是 `// [ax, ay, az]: Normalized gravity vector.`【本仓实测】`src/rm_perception/imu_complementary_filter/include/imu_complementary_filter/complementary_filter.h:76`。

**它没做什么**：它**不发布任何 TF**，也不改变 LIO 的世界系。EKF 版本（robot_localization）如果开 `two_d_mode`，还会**主动把 3D 姿态清零**：

> "If your robot is operating in a planar environment ... set this to true. It will fuse 0 values for all 3D variables (Z, roll, pitch, and their respective velocities and accelerations)."【引用 C22】

**它假设什么**：只有两条物理假设——(a) 加计测到的是比力，静止时 ≈ 重力；(b) 运动加速度不会长期淹没重力（靠 `gain_acc` 小来压）。**对安装姿态没有任何假设**：装 30° 也好倒装也好，它估的就是"加计指向哪"，所以 IMU 层天生"斜装免疫"【推断，由算法定义得出】。

**结论**：IMU 层**拥有 R 的"观测"**，但**不拥有 R 的"发布"**。这就是后面所有麻烦的起点。

### 2.3 LIO / SLAM 层：三家三种态度

#### (a) FAST-LIO：**不转世界系（明确、有维护者原话）**

维护者（Member）在 issue #284 里的原话【引用 C1】：

> "As we build the map in the IMU frame, I would suggest conducting a gravity alignment during the initialization step. Read the IMU to identify the gravity direction, and record a transformation matrix to translate your IMU to the gravity frame. After that, you only need to transform the point cloud via that matrix when publish the point clouds. You'll obtain the point cloud in the gravity frame by doing so."

—— 即：**"地图建在 IMU 系里"是设计事实；重力对齐是"建议你初始化时自己做"的外部步骤。**

issue #433 里有一条更直白的社区回答（**注意：不是维护者，是社区用户**；结论与 #284 一致，但它不是一手权威，引用时应降级）【引用 C2】：

> "FAST-LIO does not perform automatic horizontal leveling. It assumes the sensor frame is already consistent with the gravity direction from the IMU. If the LiDAR is mounted with a tilt (roll/pitch), the estimator will build the map from that orientation."

**源码级证据（比论坛更硬）**【引用 C3/C4】：

- 世界系 = 初始 IMU 系：初始化只设 `grav`、`bg`、外参，`rot` 保持单位（那行"把世界系转正"的代码**被注释掉了**）；
- 重力是**被估计的状态**：`include/use-ikfom.hpp:20` 里 `((S2, grav))`，且协方差与状态耦合（同文件 `:72-74`）⇒ ESKF 会**细化重力方向**，但**从不拿它去转世界系**；
- 它**被打印出来**：`laserMapping.cpp:940` 每帧打印 `state_point.grav`【引用 C3】⇒ **即使不改代码，也能从运行日志里读出重力矢量**（这是 §6 的一条实操路径）。

> ⚠️ **对本仓的意义（本仓实测）**：本仓 `src/rm_localization/FAST_LIO/src/IMU_Processing.hpp:193` 与上游逐字一致 ⇒ **FAST-LIO 槽位把倾斜留在世界系里**。**但这不能推广到 Point-LIO**，见 (a′)。

#### (a′) Point-LIO：**转！初始化时就把世界 z 对到重力**（任务书前提在此更正）

**它的做法**（源码，本仓 `point_lio` 子模块，除 `laserMapping.cpp` 一行调参外与上游一致）【引用 C30】：

```cpp
// src/IMU_Processing.cpp:34  —— 算一个把测到的重力转到"已知重力"的旋转
void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  ...
  V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
  rot = Exp(align_angle(0), align_angle(1), align_angle(2));      // 最小旋转
}

// src/laserMapping.cpp:466-478 —— 初始化时直接把它当成初始姿态
p_imu->Set_init(tmp_gravity, rot_init);
kf_input.x_.rot  = rot_init;
kf_output.x_.rot = rot_init;
kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
```

- `tmp_gravity = -mean_acc/|mean_acc| * G`（静止时测到的重力方向），`gravity_` 来自参数 `mapping.gravity`，默认 `[0, 0, -9.810]`【本仓实测】`src/rm_localization/point_lio/config/mid360.yaml:49`。
- ⇒ **初始姿态 = "把测到的重力摆到世界 −z"的旋转** ⇒ **世界系 z 朝上、yaw 任意**，即**它自己完成了 §1 的 `R_grav←camera_init`**。
- 配置里甚至有一条注释写明了这个意图（虽然该键在代码里**没有被读取**，是无效果的遗留键）【本仓实测】：
  `gravity_align: true # true to align the z axis of world frame with the direction of gravity, and the gravity direction should be specified below`
  `src/rm_localization/point_lio/config/pointlio_mid360_sim.yaml:50`

**它假设什么**：初始化期间**静止**（否则 `mean_acc` 混入运动加速度，`rot_init` 就带误差）；`mapping.gravity` 填的是真实重力方向（默认对）。

> **⇒ 结论修正**：**FAST-LIO = 不转（斜的）**，**Point-LIO = 转（正的，yaw 自由）**。本仓两个槽位行为不同，`camera_init` 的语义**随槽位而变**——这一点在跨槽位对比时必须显式记录【推断】。

#### (b) Cartographer 3D：**做，而且是"显式重力对齐"的教科书实现**

三层证据链：

1. **重力是低通估计出来的**，参数有正式文档【引用 C8】：
   > "`imu_gravity_time_constant` — Time constant in seconds for the orientation moving average based on **observed gravity via the IMU**."
   默认 `10.` 秒【引用 C9】。
2. **每帧算一次重力朝向**：`PoseExtrapolator::EstimateGravityOrientation()` 直接返回 `imu_tracker.orientation()`【引用 C6】。
3. **并且真的用它定义"重力系"**：3D 前端里
   ```cpp
   const Eigen::Quaterniond local_from_gravity_aligned =
       pose_estimate.rotation() * gravity_alignment.inverse();
   ```
   把子图插入定义在**重力对齐的 local 帧**里，`gravity_alignment` 还随 node 一起存进 `TrajectoryNode::Data`【引用 C7】。

**它假设什么**：有 IMU、`tracking_frame` 与 IMU 同向（配置项）、并且**愿意等 10 s 让 EMA 收敛**。所以它是"能吸收斜装"的，但代价是**前 10 秒的姿态是"正在变正"的**，且 yaw 仍然自由【推断】。

#### (c) hdl_graph_slam：**做，靠"每帧 RANSAC 地面 + 图优化 + 一个倾角先验"**

- README 自述【引用 C10】：
  > "*floor_detection_nodelet* detects floor planes by RANSAC."
  > "This constraint optimizes the graph so that the floor planes (detected by RANSAC) of the pose nodes becomes the same. This is designed to compensate the accumulated rotation error of the scan matching in large flat indoor environments."
- 代码里**先验写得很清楚**（`apps/floor_detection_nodelet.cpp`）【引用 C11】：
  ```cpp
  tilt_deg = ...;         // approximate sensor tilt angle [deg]
  sensor_height = ...;    // approximate sensor height [m]
  height_clip_range = ...;// points with heights in [sensor_height ± height_clip_range] ...
  floor_normal_thresh = ...;// verticality check thresold for the detected floor plane [deg]
  Eigen::Matrix4f tilt_matrix = ... AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()) ...;
  ...
  if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {  // the normal is not vertical
  ```
  ⇒ **它能"估平面"，但必须先知道"大概多斜"（`tilt_deg`，且只绕 Y 轴 = 只算 pitch）和"大概多高"，并把"法向偏离竖直 10° 以上"直接判失败**。
- 它的 `map→odom` 是**完整 6DoF**（发四元数）【引用 C13】`map2odom_publisher.py`：`self.broadcaster.sendTransform(pos, quat, ...)`。

**一句话**：hdl 的"能吸收 tilt"是**图优化 + 先验**换来的，而不是"随便斜都行"。

### 2.4 重定位层：2D 结构上不行，3D 能但不负责（详见 §4）

### 2.5 感知层：不是"自带"，是"要求"（详见 §5）

---

## 3. 约定：到底"谁应该"发布重力对齐的世界系？

### 3.1 REP-103 / REP-105

**REP-103**（坐标约定）【引用 C20】：

> "In relation to a body the standard is: * x forward * y left * z up"
> "For short-range Cartesian representations of geographic locations, use the `east north up` (ENU) convention: X east, Y north, Z up"

**REP-105**（帧语义）——三句话决定了全部归属【引用 C19】：

1. **重力对齐的义务落在 `map` 上**：
   > "The coordinate frame called `map` is a world fixed frame, **with its Z-axis pointing upwards**. The pose of a mobile platform, relative to the `map` frame, should not significantly drift over time."
2. **`odom` 没有这个义务**：
   > "the pose of a robot in the `odom` frame **can drift over time, without any bounds** ... However, the pose of a robot in the `odom` frame is guaranteed to be **continuous**."
   ⇒ REP-105 对 `odom` **只要求连续，不要求重力对齐**【引用 + 推断（原文没有正面要求）】。
3. **允许插中间帧**（这条是我们方案的法律依据）：
   > "**Extra Intermediate Frames** — This graph shows the minimal representation of this graph. The basic topology should stay the same, however **it is fine to insert additional links in the graph** which may provide additional functionality."
4. **并且 REP-105 亲自批准了"用加速度计把 map 摆正"**：
   > "An example of an application which cannot meet the above requirements is a robot starting up without an external reference device such as a GPS, compass, nor altimeter. But **if the robot still has an accelerometer it can intialize the map at its current location with the z axis upward**."【引用 C19，原文拼写 `intialize`】

> 这最后一条几乎就是给我们的答案：**"有加速度计 ⇒ 初始化时把 map 的 z 朝上" 是 REP 作者眼里的标准做法，不是额外算法。**

### 3.2 Nav2：`map→odom` 归"定位包"，而定位包只承诺 (x, y, yaw)

Nav2 官方 Transform 教程把三棵树的责任划分写死了【引用 C18】：

> "The first transform `map` => `odom` is usually provided by a different ROS package dealing with localization and mapping such as AMCL. ... **All ROS compliant SLAM and localization packages will provide you with this transformation automatically on launch.**"
> "The `odom` => `base_link` is usually published by our odometry system ... typically computed via sensor fusion of odometry sensors (IMU, wheel encoders, VIO, etc) using the `robot_localization` package."
> "All other statically defined transforms (e.g. `base_link` => `base_laser` ...) ... is usually provided to Nav2 through the Robot State Publisher and the URDF."

并且教程给的**传感器安装偏移的标准写法就是一条静态 TF**【引用 C18】：

```
ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0 --z 0.2 \
    --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id base_laser
```

**但 AMCL 这个"定位包"给出的 `map→base` 是 `Pose2D`**【引用 C17】：

> "`initial_pose` — Type: **Pose2D** — X, Y, **Z, and yaw** coordinates of initial pose ... of robot base frame in global frame."

⇒ Nav2 的"定位包负责 map→odom"这句话，**在 2D 定位包里实际只承诺 x/y/yaw**（z 与 roll/pitch 不在语义里）【推断，由 Pose2D 语义 + §4.1 代码得出】。

### 3.3 robot_localization：文档说得最清楚的一层

- **`world_frame` 决定谁发哪条边**【引用 C21】：
  > "If the user's `world_frame` parameter is set to the value of `map_frame`, a transform is published from the frame given by the `map_frame` parameter to the frame given by the `odom_frame` parameter."
  > "If the user's `world_frame` parameter is set to the value of `odom_frame`, a transform is published from the frame given by the `odom_frame` parameter to the frame given by the `base_link_frame` parameter."
- **它在 2D 模式下主动放弃 3D**（这正是大多数 Nav2 配置的默认）【引用 C22】：
  > "It will fuse 0 values for all 3D variables (Z, roll, pitch, ...). This keeps the covariances for those values from exploding while ensuring that your robot's state estimate remains affixed to the X-Y plane."
- **它把 IMU 的绝对姿态当作重力参考**（说明"重力信息来自 IMU"是行业共识）【引用 C21】：
  > "This uses the absolute orientation estimate from the IMU message, if available. If the IMU does not provide an orientation estimate, the first element of the orientation covariance must be set to -1 ..."

⇒ **robot_localization 是唯一"文档级别"承认 `map` 帧可能由它发布的组件，但它的 2D 模式明确不做 3D。** 要用它做重力对齐，必须 `two_d_mode: false` + `world_frame: map`【推断】。

### 3.4 Autoware：`imu_corrector` **不做**姿态/重力对齐

Autoware 的 `autoware_imu_corrector` README 列了它做的三件事【引用 C27】：

> "1. Correct **yaw rate offset** b ... 2. Correct **yaw rate standard deviation** σ ... 3. Correct **yaw rate scale** s using NDT pose as a ground truth."
> （参数表里只有 `angular_velocity_offset_x/y/z`、`angular_velocity_stddev_*`）

⇒ 它只修**角速度的零偏/尺度**，**不估 roll/pitch、不做重力对齐**【引用 + 推断】。Autoware 的重力对齐实际来自**先建的 NDT 点云地图**（地图本身是重力系）——这与 §2.3(b)(c) 的"把 tilt 烘进地图"是同一思路【推断】。

### 3.5 孤儿是怎么产生的（本节的结论）

把 §3.1–§3.4 拼起来，一条最常见的栈长这样：

```
map ──(AMCL：x,y,yaw)──► odom ──(LIO：odom→base_link)──► base_link ──(URDF)──► livox_frame
        ▲                        ▲
        │                        └── LIO 的世界系 = 初始 IMU 系（斜的）
        └── 只承诺 2D，roll/pitch 不在语义里
```

- REP-105 说 `map` 必须 z 朝上【引用 C19】；
- Nav2 说 `map→odom` 由定位包给【引用 C18】；
- 2D 定位包只给 (x, y, yaw)【引用 C17 + §4.1 代码】；
- LIO 只给 `odom→base_link`，世界系斜着【引用 C1/C3】；
- 感知全部要求 z 朝上【引用 C23–C26】。

**⇒ "把 odom 系转正"这件事，在这条链上没有任何一个组件的文档认领。它是孤儿。** 这不是本仓配置错误，是**栈的约定空隙**（REP-105 允许中间帧、Nav2 允许任意定位包，但没人负责"z 朝上"）【推断，推理链如上】。

---

## 4. 重定位专节

### 4.1 AMCL：**"看起来能吸收"，但机制是把 tilt 藏进 `map→odom`**

**代码事实（三条，逐条引用）**【引用 C14/C15】：

1. 激光安装角被**直接丢掉**：
   ```cpp
   pf_vector_t laser_pose_v;
   laser_pose_v.v[0] = laser_pose.pose.position.x;
   laser_pose_v.v[1] = laser_pose.pose.position.y;
   // laser mounting angle gets computed later -> set to 0 here!
   laser_pose_v.v[2] = 0;
   ```
   （`v[2]` 是 yaw；本仓对 `amcl_node.cpp` 全文 grep `v[2]` 只有此处对 laser pose 赋值 ⇒ "computed later" **在 `addNewScanner` 之后没有兑现**）【推断，基于 grep】。
2. 波束模型**纯 2D**，没有任何 z / pitch：
   ```cpp
   hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
   hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);
   ```
   ⇒ 斜装雷达的扫描被**当成水平扫描**做似然场匹配：真实水平投影应是 `range·cos(pitch)`，AMCL 用的是 `range` 本身。
3. 输出位姿**只有 yaw**：
   ```cpp
   q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
   tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));
   ```
   ⇒ `map→base` 被**强制**成平面位姿（z=0、roll=pitch=0）。

**这三条合起来的后果（关键，容易看漏）**【推断，由 1+3 的代数推出】：
`map→odom = (map→base)_planar ∘ (odom→base)^{-1}`。因为 `odom→base` 里带着 LIO 的倾斜，所以 **AMCL 会把倾斜的逆塞进 `map→odom`** —— 于是：
- **2D 视角看起来"正常了"**：`base_link` 在 `map` 里是平的；
- **但 `map` 帧相对重力是歪的**：3D 消费者（点云、STVL、地面分割、高度带）拿到的 `map` 继承了 −tilt；
- 而且这件事**只在有扫描更新时发生**，是**跳变**（AMCL 的 `map→odom` 本来就允许跳）。

> **回答"2D AMCL 能不能吸收倾斜的 odom"**：
> **能"看起来吸收"，但 (a) 它不 estimate 重力，(b) 它把误差转移进 `map`（对 3D 消费者是反向污染），(c) 它对斜扫描的物理建模是错的。⇒ 不能把它当作重力对齐的负责人。**【引用 C14/C15/C17 + 推断】

### 4.2 3D 重定位器：**能给 6DoF，但"正不正"取决于先验地图**

- **Scan Context + ICP/GICP**：Scan Context 是"3D 点云地图上的地点描述子 + 全局检索"，解决初值问题；ICP/GICP 做 6DoF 精配准【引用 C29】。
  - **它的描述子按"方位角扇区 × 径向环 × 每格最大高度"编码 ⇒ 只有 yaw 方向不变性，隐含"z 朝上"**【推断，由描述子构造得出；本仓 `docs/scan_context_probe_report.md` 的实测也只做了 yaw 扫描、未做 roll/pitch——**本仓文档**】。
  - **ICP/GICP 本身是 6DoF 的**，只要先验地图是重力系且初值落在收敛域内，它**能**把倾斜吸收进位姿【推断】。
- **hdl_graph_slam**：直接把"地面"作为图的边，并输出**完整 6DoF `map→odom`**【引用 C10/C13】——这是"3D 重定位能顺便把重力对齐做掉"的标准范例。

> **判据**：3D 重定位"能不能吸收 tilt"，等价于问 **"先验地图的 z 是不是重力"**。地图正 ⇒ 它能纠；地图本身是斜的（比如直接用 FAST-LIO 的 PCD）⇒ 它会把斜**当成真值**固化了【推断】。

### 4.3 只纠 `(x, y, yaw)` 会留下什么？

- **留下 2 个自由度（roll、pitch）没有任何组件纠正**：`map→odom` 只有 (x,y,yaw) ⇒ `odom` 的倾斜原样透传给 `map`【推断，由 TF 代数】。
- **"有没有文档明说 roll/pitch 不可纠？"— 没有一句正面的话**。能拿到的最强证据是**行为层**的：
  - Nav2 AMCL 的 `initial_pose` 类型是 `Pose2D`【引用 C17】；
  - `q.setRPY(0, 0, yaw)` + `Vector3(x, y, 0)`【引用 C14】；
  - robot_localization `two_d_mode` 明确"zeros out 3D variables"【引用 C22】。
  ⇒ **"2D 定位的语义边界 = 3 个自由度"是社区共识但缺少一行明文**；本调研把它记为**推断（强）**，而不是引用。

---

## 5. 感知专节

### 5.1 三种"倾斜鲁棒性"等级

| 方法 | 是否估计地面平面 | 对倾斜的鲁棒性 | 硬前提 | 证据 |
|---|---|---|---|---|
| **linefit**（Himmelsbach 2010 / `lorenwel` 实现） | 只在**每个径向列内**拟合直线段，不做全局平面 | **弱**：种子点按"高度 ≈ −sensor_height"挑 | **必须知道 `sensor_height`，且 z 朝上** | 【引用 C23/C24】 |
| **Patchwork++** | **是**：同心区 + 区域 R-GPF（PCA 拟合平面） | **中**：能处理坡面/不平，但"竖直度"仍以 `[0,0,1]` 为基准 | 需要**近似 z 朝上**；`sensor_height` 只是种子（会被 `update_elevation_thr()` 每帧改写） | 【引用 C25】【本仓实测】`docs/ground_segmentation_slots.md:132` |
| **TRAVEL** | **是**：tri-grid 图 + 局部凸凹搜索 | **中偏强**（为越野设计） | 球投影以 z 为"上"，仍隐含重力系 | 【引用 C26】【推断】 |

**linefit 的硬前提（原文）**【引用 C23】：

> "This algorithm works on the assumption that you known the height of the sensor above ground. Therefore, **you have to adjust the `sensor_height`** to your robot specifications, otherwise, **it will not work**."

**Patchwork++ 的"仍然以 z 为基准"（原文 + 公式）**【引用 C25】：

> "GLE is determined by **uprightness, elevation, and flatness** indicator functions with `v_{3,n}`, `z̄_n`, and `σ_n` as variables"
> 判定竖直点的条件里 `u_z` 定义为：`"u_z denotes the unit normal vector in the z-axis direction, i.e., [0 0 1]^T"`

⇒ **Patchwork++ 会估平面，但它判断"这个平面算不算地面"的那把尺子仍是世界 z 轴**。所以它"能容忍坡"，"不能容忍整帧被转过 30°"【推断，由上句 + 阈值 θ_v 的语义得出】。

### 5.2 本仓的现状（本仓实测）

- **我们的感知代码自己写下了这条要求**：
  `src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/low_terrain_classifier.hpp:390`
  > `/// 坐标系要求：**z 轴向上**（重力对齐）。两个地面分割节点在 gravity_aligned_frame 为空时…`
- **两个地面分割节点都已实现"只旋转、不平移"的 `gravity_aligned_frame`**：
  - `linefit`：`gravity_aligned_frame` 默认 `""`；`robot11_mount:=sensor` 档设为 `base_link`【本仓实测】`src/rm_perception/linefit_ground_segementation_ros2/.../config/segmentation_sim_robot11_sensor.yaml:35`，语义见 `:18-24` 注释。
  - `patchwork`：同款键、同款语义【本仓实测】`src/rm_perception/patchwork_ground_segmentation/src/patchwork_ground_segmentation_node.cpp:17,197,224`。
- **但这两处只解决"雷达壳体系 → 车体系（base_link）"这一段**，前提是 `base_link` 在 `map` 里是平的。**它不解决 LIO 世界系的倾斜**【推断，由 `gravity_aligned_frame` 的实现（只查 `base_link←livox_frame`）得出】。
- 另外，感知的"高度"判据本身是**相对局部地面**的（`obstacle_near_ground_m`：`dz = z − 局部地面 g(x,y)`）【本仓实测】`docs/tilted_lidar_fidelity.md:1561`、`docs/slam_lio_primer.md:2677` ⇒ 对**地面本身的倾斜**比较宽容，但对"点云整体被转过 30°"仍然会失效（地面线会斜着穿过高度带）【推断】。

---

## 6. 本仓的落脚点：最小标准方案

### 6.1 现状盘点（本仓实测，逐条给文件:行）

| # | 事实 | 位置 |
|---|---|---|
| 1 | **IMU 互补滤波已经在跑**：`do_bias_estimation: True`、`gain_acc: 0.01` ⇒ 它**已经在连续估重力/roll-pitch** | `src/rm_perception/imu_complementary_filter/launch/complementary_filter.launch.py:14,17` |
| 2 | **`camera_init→map` 已经有一条静态桥，但 6 个数全是 0**（即 `map ≡ camera_init`，把 LIO 的倾斜原样当成 map） | `src/rm_nav_bringup/launch/bringup_sim.launch.py:2117-2118` |
| 3 | **`body→odom` 也有一条全 0 静态桥** | 同上 `:2126` |
| 4 | **LIO 世界系的正/斜随槽位而变**：FAST-LIO **不转**（倾斜留在 `camera_init`）；Point-LIO **初始化时已转正**（`rot_init`） | `src/rm_localization/FAST_LIO/src/IMU_Processing.hpp:193`；`src/rm_localization/point_lio/src/IMU_Processing.cpp:34` + `src/laserMapping.cpp:466-478`；配置 `pointlio_mid360_sim.yaml:50` |
| 5 | **地面分割已有 `gravity_aligned_frame`（只旋转）** | 见 §5.2 |
| 6 | **6 个重定位槽位**：amcl / beluga / slam_toolbox / icp / gicp(=small_gicp) / LIO-当绝对定位回退 | `bringup_sim.launch.py:2095-2135` 注释 |
| 7 | **已有"地面倾角 < 1.5°"的验收判据** | `docs/cod_vs_ours_frame_architecture.md` §8 选项 B ③ |

> 第 2 条是全局的关键：**我们不是"没有重力对齐"，而是"用一条全 0 的静态 TF 声明了 map = camera_init"**。倾斜不是缺失的模块，是**被这条边写死成了恒等**。

### 6.2 两条等价的一次性计算（都不需要新算法）

**路径 A（首选，零成本）：用已有的 IMU 滤波输出**

- 关系（**只对 FAST-LIO / small_point_lio 这类"世界系 = 初始 IMU 系"的槽位成立**）：`camera_init` 定义为**初始 IMU 姿态**，所以
  `R_grav←camera_init = R_grav←imu(t₀)`。
  > ⚠️ **Point-LIO 槽位不要用这条**：它的 `camera_init` 已经被 `rot_init` 转正（§2.3(a′)），此时 `R_grav←camera_init ≈ I`（yaw 仍自由），只需要处理 yaw 与 `map` 的对齐【推断，由 C30 的初始化逻辑得出】。
- `imu_complementary_filter` 输出的四元数**就是** `R_world←imu`，其 world 系 z 轴 = 重力（yaw 任意）【本仓实测（源码注释 `// [ax, ay, az]: Normalized gravity vector.`）+ 推断】。
- ⇒ **启动后静止 2–3 s，把 `/imu/data` 的 orientation 取平均，直接就是 `R_grav←camera_init`**（在 yaw 任意的意义下）。
- 换算到 TF（**最容易写反的一步**）：要让 `world_grav`（child）的 z 轴朝上，需要 `R_camera_init←world_grav = R_grav←camera_init⁻¹`，即
  `q_tf = q_filter(t₀).conjugate()`（见 §6.3）。

**路径 B（无 IMU / 交叉校验）：第一帧点云拟合地面**

- 对第一帧 `/cloud_registered`（或 raw）取 ±0.3 m 高度带内的点做 RANSAC/PCA，得法向 `n̂`（在 `camera_init` 系里），
  `R = FromTwoVectors(n̂, ẑ)`。hdl_graph_slam 每帧做的就是这件事【引用 C11】，我们只做一次。

**两条路都只定 2 个自由度**（§1.2）：yaw 保持 `camera_init` 原来的朝向，不引入新的 yaw 误差。**这一步不会和任何重定位槽位打架**——它只旋转，不改平移，也不改 yaw。

**一个廉价的一致性检查**：若 IMU 与车体的外参已知（URDF 的 `base_link→imu_link`），且初始化时车在平地上，则应有
`R_grav←camera_init ≈ R_base_link←imu_link`
——即"倾斜量 ≈ 安装角"（本仓 robot11 的安装档就是 30° 量级，见 `docs/cod_vs_ours_frame_architecture.md` §2.2/§2.3）。两条路算出来的角度若差很多，说明**初始化时车没停稳/地面不平**，应重来。

### 6.3 这条旋转放到哪？（三种放法，推荐第 2 种）

| 放法 | 做法 | 优点 | 风险 |
|---|---|---|---|
| ① 直接改 `camera_init→map` 的 6 个数 | 把 `bringup_sim.launch.py:2118` 的 `'0','0','0','0','0','0'` 换成算出的 roll/pitch | 改动最小（6 个字符） | `map` 同时被重定位槽位发布 ⇒ 可能多父边；yaw 语义被搅进同一条边 |
| ② **新增一个中间帧 `world_grav`（推荐）** | `map ← world_grav ← camera_init ← imu_link`，`camera_init→world_grav` 为**冻结静态** | **REP-105 明文允许中间帧**；`map` 的语义（2D 地图、AMCL/GICP 发布）完全不动；3D 消费者显式改 `target_frame: world_grav` | 需要把 3D 消费者的 frame 从 `map` 改成 `world_grav`（配置层） |
| ③ 烘进资产 | 建图时把 PCD 转正，地图/坐标系都以重力系为准 | 零运行时开销 | 换一次挂装就要重建资产；仿真/实车资产会分叉 |

**TF 方向（容易写反，务必按这个填）**：`static_transform_publisher` 发的是 **`T_parent←child`**。若 `parent=camera_init`、`child=world_grav`，要让"新系 z 朝上"，需
```
q = FromTwoVectors(û_ci, ẑ).conjugate()      // û_ci = 测到的"上"方向在 camera_init 里的单位矢量
即 arguments = 0 0 0  <yaw=0> <pitch=p> <roll=r>  camera_init  world_grav
```
（`static_transform_publisher` 的位置参数顺序是 `x y z yaw pitch roll`，**不是** `roll pitch yaw`；见 `bringup_sim.launch.py:2118` 的既有写法【本仓实测】。）

### 6.4 验收与失效模式

| 验收项 | 判据 | 依据 |
|---|---|---|
| 地面法向与 z 的夹角 | **< 1.5°**（本仓既有判据） | 【本仓实测】`docs/cod_vs_ours_frame_architecture.md` §8 ③ |
| `base_link` 在 `world_grav` 里的 roll/pitch | ≈ 0（初始化时车在平地） | 【推断】 |
| yaw 是否被意外改变 | 与改动前**逐位相同**（只动 2 个 DoF） | 【推断】 |
| 静态冻结的代价 | 初始化误差会**永久保留**；若起始地面有坡/车在动 ⇒ 必须重算 | 【推断】 |
| 什么时候必须重算 | 每次启动（或每次换挂装/换车）；**不是每帧** | 【推断】 |

### 6.5 结论：**不需要任何新算法**

- **"是不是一定要额外算法处理？"——不是。** 重力方向已经在三个地方现成存在：① `imu_complementary_filter` 的输出（本仓在跑）【本仓实测】；② FAST-LIO 的 `S2 grav` 状态（甚至会打进日志）【引用 C3】；③ 第一帧地面 RANSAC 的法向【引用 C11】。缺的只是**"把这条旋转发布成一条 TF 并冻结"**这一步——这是**标准接线**（Nav2 教程里传感器安装偏移就是用静态 TF 表达的，见 §3.2）【引用 C18】。
- **REP-105 双重背书**：`map` 必须 z 朝上【引用 C19】；**中间帧允许**；**有加速度计就可以在当前位置把 map 的 z 轴朝上初始化**【引用 C19】。
- **唯一"必须做对"的是语义分工**：重力对齐 = **2 DoF 的 tilt**；yaw 仍然由重定位（AMCL / Scan Context / GICP）负责。**不要用重力对齐去替代重定位，也不要指望重定位替你做重力对齐**（§4.3）。
- **谁不该背这个锅**：FAST-LIO（设计上不转世界系）【引用 C1/C3】、2D AMCL（语义只有 Pose2D）【引用 C14/C17】、地面分割（它们是**消费者**，`low_terrain_classifier.hpp:390` 已明写要 z 朝上）【本仓实测】。
- **谁已经背了**：**Point-LIO**（初始化时 `rot_init`，§2.3(a′)）【引用 C30】、**Cartographer 3D**（EMA 重力对齐）【引用 C5–C9】、**hdl_graph_slam**（地面约束 + 6DoF `map→odom`）【引用 C10–C13】。
  ⇒ **所以本仓的"倾斜问题"是槽位相关的**：跑 Point-LIO / Cartographer 时世界系已经是正的（只需处理 yaw 与 map 对齐）；跑 **FAST-LIO / small_point_lio** 时才是"必须有人补一条 `R_grav←camera_init`"。【推断，基于 §2.3 + §6.1 第 4 条】

---

## 7. 引用清单

| 编号 | 内容 | URL |
|---|---|---|
| C1 | FAST_LIO issue #284，维护者（Member）答"地图建在 IMU 系、建议初始化时做重力对齐" | https://github.com/hku-mars/FAST_LIO/issues/284 |
| C2 | FAST_LIO issue #433，**社区用户**答"不做自动水平校正"（非维护者，降级引用） | https://github.com/hku-mars/FAST_LIO/issues/433 |
| C3 | FAST-LIO `src/IMU_Processing.hpp`（`init_state.grav = S2(...)`；`laserMapping.cpp` 打印 `grav`） | https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/src/IMU_Processing.hpp · https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/src/laserMapping.cpp |
| C4 | FAST-LIO `include/use-ikfom.hpp`（`((S2, grav))`） | https://github.com/hku-mars/FAST_LIO/blob/main/include/use-ikfom.hpp |
| C5 | Cartographer `mapping/imu_tracker.cc`（重力 EMA + `FromTwoVectors`） | https://raw.githubusercontent.com/cartographer-project/cartographer/master/cartographer/mapping/imu_tracker.cc |
| C6 | Cartographer `mapping/pose_extrapolator.cc`（`EstimateGravityOrientation`） | https://raw.githubusercontent.com/cartographer-project/cartographer/master/cartographer/mapping/pose_extrapolator.cc |
| C7 | Cartographer `mapping/internal/3d/local_trajectory_builder_3d.cc`（`local_from_gravity_aligned`） | https://raw.githubusercontent.com/cartographer-project/cartographer/master/cartographer/mapping/internal/3d/local_trajectory_builder_3d.cc |
| C8 | Cartographer 参数文档 `imu_gravity_time_constant` | https://raw.githubusercontent.com/googlecartographer/cartographer/master/docs/source/configuration.rst |
| C9 | Cartographer `configuration_files/trajectory_builder_3d.lua`（默认 10.） | https://raw.githubusercontent.com/googlecartographer/cartographer/master/configuration_files/trajectory_builder_3d.lua |
| C10 | hdl_graph_slam README（floor_detection_nodelet / floor 边） | https://github.com/koide3/hdl_graph_slam |
| C11 | hdl_graph_slam `apps/floor_detection_nodelet.cpp`（`tilt_deg`/`sensor_height`/`floor_normal_thresh`） | https://raw.githubusercontent.com/koide3/hdl_graph_slam/master/apps/floor_detection_nodelet.cpp |
| C12 | hdl_graph_slam `launch/hdl_graph_slam.launch`（`tilt_deg`/`sensor_height`/`floor_edge_stddev`） | https://raw.githubusercontent.com/koide3/hdl_graph_slam/master/launch/hdl_graph_slam.launch |
| C13 | hdl_graph_slam `src/hdl_graph_slam/map2odom_publisher.py`（6DoF `map→odom`） | https://raw.githubusercontent.com/koide3/hdl_graph_slam/master/src/hdl_graph_slam/map2odom_publisher.py |
| C14 | nav2_amcl `src/amcl_node.cpp`（`laser_pose_v.v[2]=0`；`q.setRPY(0,0,yaw)`；`Vector3(x,y,0)`） | https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_amcl/src/amcl_node.cpp |
| C15 | nav2_amcl `src/sensors/laser/likelihood_field_model.cpp`（纯 2D 波束端点） | https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp |
| C16 | nav2_amcl README（"using a 2D laser scanner"） | https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_amcl/README.md |
| C17 | Nav2 文档 configuring_amcl（`initial_pose` = **Pose2D**） | https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/ |
| C18 | Nav2 文档 setup_transforms（map→odom 由定位包、odom→base_link 由里程计、静态 TF 用法） | https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/ |
| C19 | REP-105（map z-up / odom 连续 / **Extra Intermediate Frames** / 加速度计把 map z 朝上） | https://www.ros.org/reps/rep-0105.html （镜像：https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst） |
| C20 | REP-103（body x-forward/y-left/z-up；ENU） | https://www.ros.org/reps/rep-0103.html （镜像：https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst） |
| C21 | robot_localization `doc/state_estimation_nodes.rst`（`world_frame` 决定发哪条边；IMU 绝对姿态做重力扣除） | https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/state_estimation_nodes.rst |
| C22 | robot_localization `doc/configuring_robot_localization.rst`（`two_d_mode` 把 3D 变量清零） | https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/configuring_robot_localization.rst |
| C23 | linefit_ground_segmentation README（`sensor_height` 是硬前提） | https://github.com/lorenwel/linefit_ground_segmentation |
| C24 | Himmelsbach, Hundelshausen, Wuensche, "Fast Segmentation of 3D Point Clouds for Ground Vehicles", IEEE IV 2010, pp. 560–565（bibtex 见 C23 README；**未取到论文全文/DOI 未核实**） | https://github.com/lorenwel/linefit_ground_segmentation （README 内 bibtex） |
| C25 | Patchwork++（arXiv 2207.11919；GLE 的 uprightness/elevation/flatness 与 `u_z=[0,0,1]`） | https://arxiv.org/abs/2207.11919 · https://ar5iv.labs.arxiv.org/html/2207.11919 |
| C26 | TRAVEL（arXiv 2206.03190，tri-grid 图 + 局部凸凹） | https://arxiv.org/abs/2206.03190 |
| C27 | Autoware `autoware_imu_corrector` README（只修 yaw rate 偏置/尺度） | https://raw.githubusercontent.com/autowarefoundation/autoware_universe/main/sensing/autoware_imu_corrector/README.md |
| C28 | `imu_complementary_filter`（本仓 vendored；"Normalized gravity vector"） | `src/rm_perception/imu_complementary_filter/include/imu_complementary_filter/complementary_filter.h:76` |
| C29 | Scan Context（Kim & Kim, IROS 2018） | https://doi.org/10.1109/IROS.2018.8593953 |
| C30 | **Point-LIO 的初始化重力对齐**：`src/IMU_Processing.cpp` 的 `Set_init`（算 `align_angle`→`Exp(...)`）、`src/laserMapping.cpp` 的 `kf_input.x_.rot = rot_init`、参数注释 `gravity_align: true # true to align the z axis of world frame with the direction of gravity` | 本仓 `src/rm_localization/point_lio/src/IMU_Processing.cpp:34-56` · `src/laserMapping.cpp:466-478` · `config/pointlio_mid360_sim.yaml:50`（上游仓库：https://github.com/hku-mars/Point-LIO ） |

**引用计数**：30 条（C1–C27、C29 为外部 URL，C28、C30 为本仓 vendored 源码/子模块；另有 8 处以上本仓文件:行作为"本仓实测"证据，不计入引用数）。

---

## 8. 未能取到 / 存疑（诚实登记）

| # | 项 | 状态 | 影响 |
|---|---|---|---|
| 1 | `www.ros.org/reps/*` 正文 | **被 Anubis 反爬页挡住**（返回 "Making sure you're not a bot!"） | 改用 `ros-infrastructure/rep` 的 `.rst` 原文（C19/C20），**文字逐句核对过，内容同一手稿** |
| 2 | `docs.ros.org` 上的 robot_localization 文档 | **同样被挡** | 改用仓库内 `doc/*.rst`（C21/C22），是同一份文档的源文件 |
| 3 | GitHub REST API（issues） | **限流**（`API rate limit exceeded`） | 改用 GitHub HTML 页面 + 文本抽取拿到 issue 正文与评论（C1/C2） |
| 4 | hdl_graph_slam 论文全文（SAGE, 10.1177/1729881419841532） | **未取到**（付费墙） | 只用 README + 源码 + 参数（C10–C13）作为证据；"论文里怎么描述 floor 约束"未能逐字引用 |
| 5 | Scan Context 论文 PDF（ACM DL） | **未取到**（付费墙） | §4.2 里"只有 yaw 不变性 / 隐含 z 朝上"标为**【推断】**，未升级为引用 |
| 6 | Scan Context 参考实现源码（`gisbi-kim/scancontext_tro`） | **路径 404，未取到** | 同上，不引用代码 |
| 7 | TRAVEL 全文 | **只取到摘要**（arXiv API） | 只引用摘要层面的"tri-grid 图 + 局部凸凹"；其坐标轴假设标为**【推断】** |
| 8 | Autoware 侧"重力对齐由地图负责" | **未找到明文**（`imu_corrector` README 只讲 yaw rate） | §3.4 的结论标为**【推断】** |
| 9 | "2D 定位不纠 roll/pitch" 的**明文文档** | **没找到** | 只用代码行为（C14/C17/C22）作证据，标为**【推断（强）】** |
| 10 | FAST_LIO #433 的"不做水平校正" | **是社区用户回答，不是维护者**（与任务书里的说法不同，此处更正） | 一手权威改用 #284 维护者原话（C1）+ 源码（C3） |
| 11 | **"FAST-LIO / Point-LIO 都不做重力对齐"** | **前提有误，此处更正**：FAST-LIO 不做，**Point-LIO 做**（`Set_init` → `kf_input.x_.rot = rot_init`，无条件执行）【引用 C30】 | 本报告 §2.3(a′) 据此改写；"本仓两个 LIO 槽位行为相反"成为 §6 的关键前提 |
| 12 | Point-LIO 上游 GitHub raw | `hku-mars/Point-LIO` 的 `src/IMU_Processing.cpp` 在 `main`/`master` 均 **404**（未能直连取到） | 改用**本仓 `point_lio` 子模块**（`git status` 显示仅 `laserMapping.cpp` 有一行调参改动，`IMU_Processing.cpp` 干净）作为证据；结论标为【引用 C30 + 本仓实测】 |
| 13 | Himmelsbach 2010 论文 DOI | **未核实**（只有 README 里的 bibtex） | C24 改为"给 bibtex、不给 DOI"，避免编造 |

---

## 附：一页纸回答（给评审/队友）

1. **要不要额外算法？** 不要。要一条**冻结的静态旋转**（REP-105 允许中间帧），值来自**已经在跑**的 IMU 互补滤波（或 FAST-LIO 日志里的 `grav`，或一次地面 RANSAC）。
2. **四层谁管？** IMU 层给"观测"（但不发帧）；**FAST-LIO 不管 / Point-LIO 管（初始化时转正）**；Cartographer 3D / hdl_graph_slam **管**（各自的机制）；2D AMCL **管不了**；感知**只消费**（且明文要求 z 朝上）。
3. **为什么我们会觉得"没人管"？** 因为它真的是孤儿：Nav2 把 `map→odom` 外包给"定位包"，2D 定位包的语义只有 `Pose2D`，而 **FAST-LIO 的世界系是初始 IMU 系**（Point-LIO 不在此列）。
4. **本仓最小改法**：**先按槽位判断**——用 Point-LIO / Cartographer 时世界系已正，只需处理 yaw 与 `map` 对齐；用 **FAST-LIO / small_point_lio** 时新增中间帧 `world_grav`（或填 `camera_init→map` 那条静态边的 roll/pitch），一次算、冻结、验收（地面倾角 <1.5°）；yaw 不动；3D 消费者把 `target_frame` 指到 `world_grav`。
