# `odom → base_link` 在「斜装 / 云台时变」下的开源实践（代码级调研）

> **本文要回答的问题（用户原话）**
> 1. "`odom_grav`（一条冻结的重力对齐中间帧）这个角色很麻烦，我感觉其实不必要，是哪里没做清楚" —— 这条中间帧是**别人也在用的模式**，还是过度设计？真实栈用什么替代？
> 2. "而且很局限，未必只有 pitch/roll 会变化" —— 时变挂装 + 漂移，静态帧都治不了。真实栈怎么处理**时变挂装**和**漂移**？
> 3. "`odom → base_link` 这里也是，有没有别的战队开源看看是怎么处理这个部分的" —— **RoboMaster 战队**与通用机器人栈里，**谁**算/发 `odom → base_link`，斜装怎么处理？
> 4. "自研一个包处理这个有可能是没搞清楚的担忧" —— 有没有**现成包**可以采纳，而不是自研？
>
> **方法**：GitHub 仓库检索（API，未带 token ⇒ 只有 repo 搜索，无 code 搜索）+ 浅克隆 9 个 RoboMaster 导航栈 + `curl` 取上游逐字源码（raw.githubusercontent，4xx 立即失败、网络/5xx 最多重试 5 次、间隔 10 s）。**不做**：不改代码/配置/launch，不跑仿真，不跑任何节点。
>
> **证据分级**（全文逐条标注）：
> - 【引用】= 外部 URL 原文（代码逐字或文档逐字），行号以抓取时 `main`/`master`/指定分支为准；
> - 【本仓实测】= 本仓已落盘的测量，指向 `docs/` 内的节号/行号，**不重跑**；
> - 【推断】= 由前两者推出的结论，**没有**直接源码/文档原话支撑。
>
> **读者顺序建议**：§0 → §1 → §6（结论与建议）→ 需要证据时回查 §2–§5。

---

## 0. 结论速览（TL;DR）

| # | 问题 | 判决 |
|---|---|---|
| **A** | 冻结的重力对齐中间帧（`odom_grav`）是标准做法吗？ | **作为「一条 TF 发布者」= 缺席（absent）**。9 个 RoboMaster 导航栈里**没有一个**在 `map` 与 `odom` 之间插重力帧；通用栈里也没有。只有**参数约定**层面存在同名概念（`gravity_aligned_frame`，见 §3.2），而它指向的是**别人已经发好的帧**（通常是 `odom` 或 `base_link` 本身）。 |
| **B** | 那真实栈把「斜装」放在哪？ | 三个位置，各自有主：**① 估计器初始化参数**（Point-LIO `gravity`/`gravity_init`，pb2025 用的就是这条，§2 R1）；**② 传感器系→车体系的 TF/URDF 外参**，由 **tf 适配器按消息时间戳查一次并左乘合成**（pb2025 / laohao78 / LIO-SAM 三处同构，§2 R1/R4/R7）；**③ 下游消费者声明自己要哪个帧**（linefit `gravity_aligned_frame`、Nav2 `robot_base_frame`）。 |
| **C** | 时变挂装（云台）怎么办？ | **只能靠"每条消息查一次 TF"**——即 `lookupTransform(lidar, base, stamp)` 后再合成，外参**不能**是常数。真例：pb2025 把 MID-360 挂在 `gimbal_yaw` 关节上（俯仰/横滚固定、yaw 由编码器驱动），`sensor_scan_generation` 每条点云按戳查 `front_mid360→base_footprint` 再合成（§4.2）；XDU 用串口角度发 `base_link→left/right_gimbal`（§2 R2）。物理事实：**只要时变的那一自由度是绕重力轴的 yaw，传感器系里的重力矢量就不变** ⇒ 一次性重力参数仍然成立；一旦 roll/pitch 时变，这条立刻失效（§4.4【推断】）。 |
| **D** | 漂移呢？ | 一次性/静态变换**结构上**治不了 roll/pitch 漂移。实践用三类纠正源：**IMU 姿态**（重力 EMA / 互补滤波，Cartographer `ImuTracker`）、**地面约束**（hdl_graph_slam 把地面平面节点固定在 z=0）、**地图匹配/重定位**（GICP/NDT 把位姿拉回先验图）。§5 给三条引用。 |
| **E** | 要不要自研包？ | **不要。** 我们已有的两块东西已经覆盖了真实栈的全部做法：`small_point_lio` 已经在用**按戳查 TF + 合成**（我们已把共轭修成合成）【本仓实测】`docs/tilted_lidar_fidelity.md`，`lio_tf_adapter` 是同一模式的独立版（但外参走**常数参数**，见 §6.2）。真正缺的只有一件事：**让估计器自己的世界系在初始化时对齐重力（E1）**——point_lio 一行参数 / small_point_lio 十几行初始化代码；不愿动估计器就退到 **E2（先摆平 IMU）**，但必须同步把 adapter 外参归零（§6.5）。**都不是新包。** |
| **F** | `odom_grav` 到底错在哪？ | 它不是"不能干活"：**它是唯一能直接摆平 `odom` 的 TF 层手段**（本仓 Nav2 局部代价地图确实在 `odom` 里切片，斜 30° 会把障碍物压扁约 13%，§6.1）。但它**放错层 + 多一份真相**：(i) 倾斜被表示两遍（URDF 一遍、`odom_grav` 一遍，正是 FAST-LIO 维护者点名的 "rotate the frame twice" 形状）；(ii) 估计器自建的 PCD/地图仍在斜系里，与"看起来水平"的帧树不一致；(iii) 挂装一变（云台）或初始化姿态一变，冻结值立刻过期；(iv) 漂移完全管不着。**它等价于"在 TF 层做 E1"，但把 E1 的四个前提藏进了实现里。** |

---

## 1. 把问题说清楚：三件被混为一谈的事

用户的三句质疑其实指向**三个不同的物理/估计问题**，它们在代码里落点不同、处置层不同、症状也不同。先把它们拆开，后面所有"别人怎么做"才有可比性。

### 1.1 事一：静态安装偏置（static mount offset）

- **定义**：`T_base_link←lidar_frame` 是一个**常数**刚体变换（含 roll/pitch/yaw + 杆臂）。本仓是 −30° roll、|t| = 0.204443 m【本仓实测】`docs/tilted_lidar_fidelity.md` §N。
- **数学**：链式法则 `T_odom←base = T_odom←lidar · T_lidar←base`。**只乘一次、只在一个地方定义**。
- **正确处置层**：外参定义（URDF/静态 TF）+ 合成（tf 适配器/估计器）。
- **搞错的症状**：`odom→base_link` 的 rpy 出现常数倾角；或倾角出现两次（60°）；或合成顺序错导致 **yaw 耦合出假 pitch**。
- **本仓状态**：修好后 `odom→base_link` rpy = `[30.452, −0.082, 0.211]°`【本仓实测】`docs/tilted_lidar_fidelity.md:2959`。**这 30.452° 不是 bug，是"odom 定义在初始化时刻的传感器系"的必然结果**【推断】（同 `docs/lio_extrinsic_application_survey.md` §0 第 4 条）。

### 1.2 事二：时变挂装（time-varying mount，云台/伺服）

- **定义**：`T_base_link←lidar_frame(t)` 随关节角变化。典型：哨兵把 LiDAR 装在云台 yaw（甚至 pitch）轴上。
- **数学**：`T_odom←base(t) = T_odom←lidar(t) · T_lidar←base(t)`。第二项**必须逐帧取**，任何常数（参数、URDF 里写死的 `rpy`、构造函数里缓存的 `lidar2Baselink`）都会在关节动起来之后变成**系统性错误**。
- **正确处置层**：`robot_state_publisher` + `joint_state_publisher`（或串口/编码器驱动）+ **每消息 `lookupTransform`**。
- **搞错的症状**：云台一动，`base_link` 在 `odom` 里"自己游/自己转"（实际车没动）；局部代价地图跟着转；scan matcher 与 TF 打架；回环时 yaw 对不上。真例引用见 §4.3。

### 1.3 事三：漂移（drift）

- **定义**：估计器的 roll/pitch（乃至 yaw、z）**随时间缓慢偏离真值**，与挂装是否变化无关。
- **数学**：`T_odom←base(t) = T_odom←base_true(t) · E(t)`，`E(t)` 是缓慢增长的误差；**任何在 t₀ 时刻算出的常数旋转都不改变 `E(t)`**。
- **正确处置层**：IMU 姿态（重力观测）/ 地面平面约束 / 地图匹配重定位 / 零速修正。
- **搞错的症状**：跑 10 min 后地面在点云里"翘起来"、2D 切片把地板切丢、`map→odom` 每次重定位都跳一大下。

### 1.4 三者对照（这张表是本文后面所有判断的坐标）

| | 事一 静态偏置 | 事二 时变挂装 | 事三 漂移 |
|---|---|---|---|
| 数学形态 | 常数 `T_lb` | `T_lb(t)`，由关节角驱动 | 误差 `E(t)` 缓慢增长 |
| 能否用"一次算好的常数旋转"解决 | **能** | **不能** | **不能** |
| 正解所在层 | 外参 + 合成 | 关节 TF + 逐帧查 | IMU/地面/地图匹配 |
| 一个冻结的重力帧能否覆盖 | 能（但它是**替代方案**，不是唯一方案，见 §6） | 不能 | 不能 |
| 主要症状 | 常数倾角 / 倾角翻倍 / 假 pitch | `base_link` 自己游 | 地板翘、切片丢、跳变 |

> **一句话**：`odom_grav` 只能覆盖**事一**，而且是用"第二条冻结 TF"去覆盖"估计器世界系没摆正"。**事二、事三它一点都管不了**——这正好对上用户的两句质疑（"很局限"、"未必只有 pitch/roll 会变化"）。【推断】

### 1.5 本仓那条 30.452° 属于哪一类？

- 它属于**事一**，而且是**事一的正确表现**：`odom` = 初始化时刻的传感器系（FAST-LIO / small_point_lio 的既有约定）⇒ `odom→base_link` 必然带 −30° 的反向倾角。【本仓实测】`docs/tilted_lidar_fidelity.md:2959`（"30° 出现在 roll 上 = odom 定义在斜的初始传感器系"）+【引用】<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L591-L619>（FAST-LIO 只发 `camera_init→body`，世界系不转正）。
- 它**不是**漂移：数值是常数、可复现、与运行时长无关【本仓实测】同上。
- 由此得到一条重要区分：**"odom 斜" 与 "base_link 斜" 是两件事**。按 §2 的表，**Point-LIO 一族基本都把世界系对齐了重力**（R1 参数化 / R2 / R3 / R4 / R9），**而 FAST-LIO、small_point_lio、LIO-SAM 一族不对齐**（R5 / R6 / R7 / R8）。本仓用的是后者 ⇒ `odom` 必然斜。**要不要摆平它，是一个产品级选择，不是一个必须补的 bug**——但本仓 Nav2 的局部代价地图恰好跑在 `odom` 里（§6.1），所以这个选择在本仓**有实际后果**。

---

## 2. 谁在发 `odom → base_link`：15 行对照表

### 2.1 对照表

图例：**许可**取自仓库 `LICENSE`/`package.xml`（未标"未核"的均已 `curl` 逐字确认）；**判决**是"我们能不能直接采纳这个做法"的意思（adopt = 直接照做；adapt = 拿思路但必须改；avoid = 别学）。

| # | 仓库 / 包 | 许可 | **谁发 `odom→base_link`** | 挂装外参怎么处理 | 重力怎么办 | 云台/时变感知 | 判决 |
|---|---|---|---|---|---|---|---|
| **R1** | `SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav`（★452，RM 哨兵旗舰） | Apache-2.0 | **独立适配节点** `sensor_scan_generation`（LIO 的 TF 被显式关掉 `tf_send_en: False`） | **逐帧 `lookupTransform(lidar, base, stamp)` 后合成** `T_ol·T_lb` | **写进估计器参数** `gravity: [0, −4.9, −8.487]  # rpy = [0, pi/6, 0]` | **是**：LiDAR 的父关节是 `gimbal_yaw`；关节位姿由串口模块 `standard_robot_pp_ros2` 提供 | **adopt**（架构），重力数字 **adapt** |
| **R2** | `IRobot-Algorithm/sentry_navigation`（西电 IRobot，RM2024） | 无 LICENSE | **Point-LIO 节点自己发**，`frame_id="odom", child="base_link"` | 只加**平移**杆臂（`res = quatRotate(quat, p)` 后加到位置），**无旋转合成** | 未做（用 Point-LIO 默认） | **是**：串口角度直接发 `base_link→left_gimbal` / `right_gimbal` | **avoid**（LIO 内改帧名+只加平移）；**adopt** 它的编码器 TF |
| **R3** | `LRaina215/HNU_NHS_SENTRY_UP`（HNU 哨兵） | 无 LICENSE | **Point-LIO 节点自己发**，child 硬编码 `base_footprint` | 无合成；外参写在 LIO 的 `extrinsic_R`（90° yaw） | **`gravity_align: true`** + `gravity/gravity_init = [0,0,−9.81]` | 无（LiDAR 在 `base_link` 上，`lidar_joint` 固定 `rpy="0 0 -1.57"`） | **adapt**（只学 `gravity_align` 这个开关） |
| **R4** | `laohao78/ROS2_RM_Navigation`（★41，MIT，与本仓同族命名） | MIT | **LIO 节点内**发 `lidar_odom→base_link` | **逐帧 lookup + 合成** `tf_lidar_odom_to_livox_frame * tf_livox_frame_to_base_link`；URDF 里 `livox_joint` 的 `rpy` 是 xacro 参数 | 未做（`point_lio` 默认 `gravity_init=[0,0,-9.81]`） | 无（固定关节） | **adopt**（合成 + URDF 参数化倾角） |
| **R5** | `Yancey2023/small_point_lio`（ros2 分支，本仓 vendor 的上游） | MIT | **LIO 节点自己发** `odom→base_link` | 逐帧 `lookupTransform` ✔，但合成本写成**共轭** `T_bl⁻¹·T_ol·T_bl` ✘ | 未做（`rotation=Identity()`，均值加速度只进 `gravity` 状态） | 无 | **avoid**（共轭；本仓已修为合成） |
| **R6** | `qza36/COD_NAV`（★260 级参考栈） | 无 LICENSE | **FAST-LIO 节点自己发** `odom→chassis` | **只是改名**：把 LIO body 位姿直接当 `chassis` 发出去（`T_bl ≡ I` 约定） | 未做 | 无 | **avoid**（"把 base_link 定义成斜的那个系"，代价转移给下游语义） |
| **R7** | `TixiaoShan/LIO-SAM` | BSD-3 | **`imuPreintegration.cpp` 里的 `TransformFusion`** 同时发 `map→odom` 与 `odom→base_link` | `tCur = tCur * lidar2Baselink`（合成 ✔），但 `lidar2Baselink` 在**构造函数里查一次就缓存** | 未做（世界系 = 初始 IMU 系） | **否**（缓存 ⇒ 时变挂装会错） | **adopt**（合成写法），注意它的**静态缓存** |
| **R8** | `hku-mars/FAST_LIO` | GPL-2.0 | **不发** `odom→base_link`；只发 `camera_init→body` | 不管（外参只进 `body` 系） | **不做重力对齐**（`rot` 保持单位） | 无 | **avoid**（GPL + 不管 base_link） |
| **R9** | `hku-mars/Point-LIO` | GPL-2.0 | **不发** `odom→base_link`；发 `camera_init→aft_mapped`，且可 `publish.tf_send_en: False` 关掉 | 不管 | **做**：`Set_init` 用测到的重力算 `rot_init` 当初始姿态；`gravity`/`gravity_init` 可参数化 | 无 | **adapt**（用它的初始化重力；GPL 注意） |
| **R10** | `autowarefoundation/autoware_universe` → `sensing/autoware_imu_corrector` | Apache-2.0 | 不发 TF；**把 IMU 数据重发到 `base_link` 系**（`target_frame` / `base_link` 参数） | 用 TF 把 accel/gyro **旋转**到 `base_link`（`transform_vector3`），协方差**不旋转只膨胀** | **不做**重力/roll-pitch 估计（只做陀螺零偏/尺度） | 依赖 TF ⇒ 时变可用（但取 `get_latest_transform`） | **adapt**（E2「先把 IMU 摆平再喂 LIO」的现成件） |
| **R11** | `ros-perception/imu_pipeline` → `imu_transformer`（ros2 分支） | **GPL-3.0** | 不发 TF；把 `sensor_msgs/Imu` 变换到 `target_frame` | `tf2_buffer_->transform(*imu_in, imu_out, target_frame_)`（含协方差旋转） | 不做 | 依赖 TF ⇒ 时变可用 | **avoid**（GPL-3.0 传染性，见 §6.4） |
| **R12** | `cra-ros-pkg/robot_localization` | BSD-3 | **EKF/UKF 自己发** `map→odom` 或 `odom→base_link`（`publish_tf`、`world_frame` 决定） | 由输入帧 TF 决定（`base_link_frame`） | 不做重力对齐；`two_d_mode` 直接把 3D 信息**丢掉** | 不感知 | **adopt**（E4：把 `map→odom` 的所有权交给唯一一个滤波器） |
| **R13** | `cartographer-project/cartographer_ros` | Apache-2.0 | 由 `published_frame`/`provide_odom_frame` 决定：要么只发 `map→odom`，要么发 `map→base_link` | 由 `tracking_frame` 决定 | **做**：3D 用 `ImuTracker` 的重力 EMA（`imu_gravity_time_constant`）；另有 `publish_frame_projected_to_2d` 强行把 roll/pitch 清零 | 不感知 | **adopt**（两个概念：帧所有权开关 + 投影到 2D） |
| **R14** | `koide3/hdl_graph_slam` | BSD-2 | **不发 TF**（只发优化后的位姿/地图话题） | 不管 | **做**：地面平面节点 `(0,0,1,0)` **固定**进图 ⇒ 全局 roll/pitch 被地面钉住 | 不感知 | **adapt**（§5 的"地面约束"引用） |
| **R15** | `lorenwel/linefit_ground_segmentation` | BSD-3 | 不发 TF | 不管 | **消费方**声明：`gravity_aligned_frame`（z 轴对齐重力的帧名） | 不感知 | **adopt**（§3 的"参数约定"引用） |

> 另有两条**只作旁证**（未入表，避免噪声）：
> - `SMBU-PolarBear-Robotics-Team/small_gicp_relocalization`：**`map→odom` 的发布者**（`map_frame_ → odom_frame_`）【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/blob/main/src/small_gicp_relocalization.cpp#L36-L51>、<https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/blob/main/src/small_gicp_relocalization.cpp#L192-L206>。⇒ pb2025 的**所有权很干净**：重定位发 `map→odom`，适配器发 `odom→base_footprint`，LIO 的 TF 关掉。
> - `locusrobotics/fuse`（BSD）、`ethz-asl/ethzasl_msf`（Apache-2.0）：都是"通用传感器融合框架"，**都没有**"重力对齐帧"这一概念，`msf` 的 README 甚至把自己劝退（"MaRS is a actively maintained continuation…"）【引用】<https://github.com/ethz-asl/ethzasl_msf/blob/master/README.md>。

### 2.2 逐条引文（逐字）

#### R1 pb2025：LIO 关 TF，适配器逐帧查 TF 再合成

```cpp
// pb2025_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp:74-82
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  tf_lidar_to_robot_base_ = getTransform(lidar_frame_, robot_base_frame_, pcd_msg->header.stamp);
  tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp);

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  publishTransform(
    tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/sensor_scan_generation/src/sensor_scan_generation.cpp#L74-L82>

- 注意 `getTransform` 的实现：查不到就 **返回单位阵并告警**（不丢帧、不崩）：

```cpp
// 同上 :89-101
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/sensor_scan_generation/src/sensor_scan_generation.cpp#L89-L101>

- LIO 侧 TF 被关掉、外参只留杆臂、重力被写死成"安装倾角"：

```yaml
# pb2025_nav_bringup/config/reality/nav2_params.yaml:63-81（point_lio 段）
      gravity: [0.0, 0.5305473804473877, -0.8383715748786926]            # gravity to be aligned
      gravity_init: [0.0, 0.5305473804473877, -0.8383715748786926]       # preknown gravity in the first IMU body frame, use when imu_en is False or start from a non-stationary state
      ...
      tf_send_en: False                                 # True: send transform from 'camera_init' to 'aft_mapped'
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/pb2025_nav_bringup/config/reality/nav2_params.yaml#L63-L81>

- **仿真**里同一条参数把"这就是安装倾角"写成了注释（cos/sin 30°）：

```yaml
# pb2025_nav_bringup/config/simulation/nav2_params.yaml:63-64
      gravity: [ 0.0, -4.9, -8.487047153776548 ]        # gravity to be aligned # rpy = [0, pi/6, 0]
      gravity_init: [ 0.0, -4.9, -8.487047153776548 ]   # preknown gravity in the first IMU body frame, ...
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/pb2025_nav_bringup/config/simulation/nav2_params.yaml#L63-L64>（`−4.9 = −9.81·sin30°`、`−8.487 = −9.81·cos30°`【推断】）

- **LiDAR 真的挂在云台上**（SDF 关节父级 = `gimbal_yaw`，且带 −33.44° 横滚）：

```xml
<!-- pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro:24-25 -->
        <!--livox-->
        <xmacro_block name="livox" prefix="front_" parent="gimbal_yaw" pose="-0.0496 0.136 0.435 -${pi/6 + 0.06} 0 0" update_rate="20" samples="1875"/>
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description/blob/main/resource/xmacro/pb2025_sentry_robot.sdf.xmacro#L24-L25>

- 他们**自己**在 README 里把这件事讲清楚了（斜装 + 谁发 `odom→chassis`）：

> "The Livox mid360 is mounted **at an incline** on the chassis and uses the point_lio as odometry, small_gicp for localization, and loam_interface will transform PointCloud from `lidar_odom` frame to `odom` frame. The [sensor_scan_generation](./sensor_scan_generation/) transform PointCloud from `odom` frame to `front_mid360` frame and publishes the transform `odom -> chassis`."
> 【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/README.md#L25>

- 关节角从哪来（**编码器/串口**，不是猜的）：

> "in reality, it is **recommended** to use an independent package to publish the robot's TF information. For example, the `gimbal_yaw` and `gimbal_pitch` joint poses are provided by the serial communication module [standard_robot_pp_ros2](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2)"
> 【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/README.md#L222>

- 另外他们还插了一条**动态**的 "fake" 基座帧（与我们仓的 `base_link_fake` 同构）：

```cpp
// fake_vel_transform/src/fake_vel_transform.cpp:136-146
void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}
```
【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/fake_vel_transform/src/fake_vel_transform.cpp#L136-L146>，参数 `robot_base_frame: "gimbal_yaw"`、`fake_robot_base_frame: "gimbal_yaw_fake"`（:31-32 同文件 + nav2_params.yaml:157-158）

> **R1 的完整读法**：倾斜 ← 估计器参数；挂装几何 ← URDF/SDF 关节；时变 ← 编码器 + 逐帧 lookup；`map→odom` ← 重定位；`odom→base` ← 适配器。**五件事五个主人，一条重力帧都不需要。**【推断】

#### R2 XDU IRobot：LIO 节点自己发 + 只加平移 + 串口发云台 TF

```cpp
// src/Point-LIO/src/laserMapping.cpp:735-748
    odom_base.pose.pose.position.x = odomAftMapped.pose.pose.position.x + res[0];
    ...
    tf::quaternionTFToMsg(geoQuat, odom_base.pose.pose.orientation);
    ...
    br.sendTransform(tf::StampedTransform(transform, odom_base.header.stamp, "odom", "base_link"));
```
【引用】<https://github.com/IRobot-Algorithm/sentry_navigation/blob/main/src/Point-LIO/src/laserMapping.cpp#L735-L748>；`res = tf::quatRotate(quat, p)` 见同文件 :729 ⇒ **只有杆臂、没有旋转合成**。【引用】<https://github.com/IRobot-Algorithm/sentry_navigation/blob/main/src/Point-LIO/src/laserMapping.cpp#L729-L736>

```cpp
// src/nav_transporter/src/usb_comm_node.cpp:423-425（云台角度来自串口包）
        right_trans_.setRotation(tf::createQuaternionFromYaw(package.RightMotorAngle));
        left_trans_.setRotation(tf::createQuaternionFromYaw(package.LeftMotorAngle));
        br_.sendTransform(tf::StampedTransform(right_trans_, ros::Time::now(), "base_link", "right_gimbal"));
        br_.sendTransform(tf::StampedTransform(left_trans_, ros::Time::now(), "base_link", "left_gimbal"));
```
【引用】<https://github.com/IRobot-Algorithm/sentry_navigation/blob/main/src/nav_transporter/src/usb_comm_node.cpp#L423-L425>

#### R3 HNU：LIO 自己发 `odom→base_footprint`，但重力开关打开

```cpp
// rm_navi/rm_localization/point_lio/src/laserMapping.cpp:617 / 631-646
    odomAftMapped.child_frame_id = "base_footprint";
    ...
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    ...
    tf_br->sendTransform(transform);
```
【引用】<https://github.com/LRaina215/HNU_NHS_SENTRY_UP/blob/main/rm_navi/rm_localization/point_lio/src/laserMapping.cpp#L617-L646>（原注释 `//edition by vito to try navigation odom mapping`）

```yaml
# rm_navi/rm_localization/point_lio/config/mid360.yaml:41-42
            gravity_align: true # true to align the z axis of world frame with the direction of gravity, and the gravity direction should be specified below
            gravity: [ 0.0, 0.0, -9.810 ] # [0.0, 9.810, 0.0] # gravity to be aligned
```
【引用】<https://github.com/LRaina215/HNU_NHS_SENTRY_UP/blob/main/rm_navi/rm_localization/point_lio/config/mid360.yaml#L41-L42>

#### R4 laohao78：与本仓同族的合成写法（逐帧查 TF）

```cpp
// src/localization/point_lio/src/laserMapping.cpp:302-331
      livox_to_base_link_transform =
        tf_buffer->lookupTransform("livox_frame", "base_link", odomAftMapped.header.stamp);
    ...
  tf2::Transform tf_lidar_odom_to_base_link =
    tf_lidar_odom_to_livox_frame * tf_livox_frame_to_base_link;
```
【引用】<https://github.com/laohao78/ROS2_RM_Navigation/blob/main/src/localization/point_lio/src/laserMapping.cpp#L302-L331>

```xml
<!-- src/rm_nav_bringup/urdf/sentry_robot_real.xacro:17-18, 156-160 -->
  <xacro:arg name="xyz" default="0.0 0.045 0.49" />
  <xacro:arg name="rpy" default="0 0 0" />
  ...
  <joint name="livox_joint" type="fixed">
    <origin xyz="$(arg xyz)" rpy="$(arg rpy)" />
    <parent link="base_link"/>
    <child link="livox_frame"/>
  </joint>
```
【引用】<https://github.com/laohao78/ROS2_RM_Navigation/blob/main/src/rm_nav_bringup/urdf/sentry_robot_real.xacro#L156-L160>

#### R5 上游 small_point_lio：查 TF 是对的，合成是错的（共轭）

```cpp
// src/small_point_lio_node.cpp:64-75
            base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
            ...
            tf2::Transform tf_odom_to_base_link = tf_base_link_to_lidar_frame.inverse() * tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame;
```
【引用】<https://github.com/Yancey2023/small_point_lio/blob/ros2/src/small_point_lio_node.cpp#L64-L75>
【本仓实测】本仓 `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:107-132` 为同一段代码（含本仓注释与 `2a5a8b6` 的修法），推导见 `docs/cod_vs_ours_frame_architecture.md` §3.3。

#### R6 COD_NAV：只改名，不合成

```cpp
// COD_NAV/FAST_LIO/FAST_LIO/src/laserMapping.cpp:648-658
    trans.header.frame_id = "odom";
    trans.child_frame_id = "chassis";
    trans.header.stamp = get_ros_time(lidar_end_time);
    ...
    tf_br->sendTransform(trans);
```
【引用】<https://github.com/qza36/COD_NAV/blob/master/FAST_LIO/FAST_LIO/src/laserMapping.cpp#L648-L658>

#### R7 LIO-SAM：合成写法 + 构造函数缓存

```cpp
// src/imuPreintegration.cpp:46-51（TransformFusion 构造函数）
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
```
```cpp
// 同文件 :124-130
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);
```
【引用】<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L46-L51>、<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L124-L130>
同文件 `:92` 还发 `map→odom`（`tfMap2Odom.sendTransform(... mapFrame, odometryFrame)`）⇒ **一个节点同时拥有两条边**。【引用】<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L92>

```cpp
// src/mapOptmization.cpp:1645-1651（另一条：odom→lidar_link，用 6DoF 全姿态）
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);
```
【引用】<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/mapOptmization.cpp#L1645-L1651>

#### R8 / R9 FAST-LIO、Point-LIO：只发自己的内部帧

```cpp
// FAST_LIO/src/laserMapping.cpp:591-619
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    ...
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
```
【引用】<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L591-L619>

```cpp
// point_lio/src/laserMapping.cpp:256-269（本仓 vendor 版，逐字同构上游）
  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "aft_mapped";
    ...
    tf_br->sendTransform(transform);
  }
```
【本仓实测】`src/rm_localization/point_lio/src/laserMapping.cpp:256-269`；参数声明 `publish.tf_send_en` 在 `src/rm_localization/point_lio/src/parameters.cpp:198-199`。

#### R10 Autoware `imu_corrector`：不估 roll/pitch，只"把 IMU 摆到 base_link 再发"

```cpp
// sensing/autoware_imu_corrector/src/imu_corrector_core.cpp:57-58, 160-176
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & options)
: autoware::agnocast_wrapper::Node("imu_corrector", options),
  output_frame_(declare_parameter<std::string>("base_link", "base_link"))
...
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(imu_msg.header.frame_id, output_frame_);
  ...
  imu_msg_base_link->header.frame_id = output_frame_;
  imu_msg_base_link->linear_acceleration =
    transform_vector3(imu_msg.linear_acceleration, *tf_imu2base_ptr);
  imu_msg_base_link->linear_acceleration_covariance =
    transform_covariance(imu_msg.linear_acceleration_covariance);
  imu_msg_base_link->angular_velocity =
    transform_vector3(imu_msg.angular_velocity, *tf_imu2base_ptr);
```
【引用】<https://github.com/autowarefoundation/autoware_universe/blob/main/sensing/autoware_imu_corrector/src/imu_corrector_core.cpp#L160-L176>

- 它的 README 只承诺三件事（**没有** roll/pitch 估计、**没有**重力对齐）：

> "1. Correct yaw rate offset $b$ by reading the parameter. 2. Correct yaw rate standard deviation $\sigma$ by reading the parameter. 3. Correct yaw rate scale $s$ using NDT pose as a ground truth."
> 【引用】<https://github.com/autowarefoundation/autoware_universe/blob/main/sensing/autoware_imu_corrector/README.md#L5-L8>

> ⚠️ **对任务书前提的更正**：`imu_corrector` **不是**"估计 IMU roll/pitch 偏置并重发一个摆平的 IMU"。它是 (a) 陀螺零偏/尺度修正 + (b) **用 TF 把量测旋到 `base_link` 系**并重发。**"摆平"这件事它不做——它假定 `base_link` 已经是平的**。【引用】同上 +【推断】

#### R11 `imu_transformer`：TF 变换 + 协方差旋转的现成件（GPL-3.0）

```xml
<!-- imu_transformer/launch/ned_to_enu.launch.xml -->
  <!-- Sample launch file, transforms NED referenced data in imu_link_ned to ENU equivalent data in imu_link -->
  <node pkg="tf2_ros" exec="static_transform_publisher" name="tf_imu_ned_enu"
        args="0 0 0 1.5708 0 3.1416 imu_link_ned imu_link" output="screen"/>
  <node pkg="imu_transformer" exec="imu_transformer_node" name="imu_data_transformer" output="screen">
    ...
    <param name="target_frame" value="imu_link"/>
  </node>
```
【引用】<https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/launch/ned_to_enu.launch.xml#L2-L16>

```cpp
// imu_transformer/src/imu_transformer.cpp:20, 46
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    ...
      tf2_buffer_->transform(*imu_in, imu_out, target_frame_);
```
【引用】<https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/src/imu_transformer.cpp#L20-L46>
许可：GPL-3.0【引用】<https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/LICENSE>

#### R12 `robot_localization`：EKF 拥有 `map→odom` / `odom→base_link`

```yaml
# params/ekf.yaml:44-72
        # Whether to broadcast the transformation over the /tf topic. Defaults to true if unspecified.
        publish_tf: true
        ...
        #     3b. MAKE SURE something else is generating the odom->base_link transform. Note that this can even be another state
        #         estimation node from robot_localization! However, that instance should *not* fuse the global data.
        map_frame: map              # Defaults to "map" if unspecified
        odom_frame: odom            # Defaults to "odom" if unspecified
        base_link_frame: base_link  # Defaults to "base_link" if unspecified
        world_frame: odom           # Defaults to the value of odom_frame if unspecified
```
【引用】<https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml#L44-L72>

```yaml
# 同文件 :12-18（two_d_mode 的语义）
        # ekf_localization_node and ukf_localization_node both use a 3D omnidirectional motion model. If this parameter is
        # set to true, no 3D information will be used in your state estimate. Use this if you are operating in a planar
        # environment and want to ignore the effect of small variations in the ground plane that might otherwise be detected
        # by, for example, an IMU. Defaults to false if unspecified.
        two_d_mode: false
```
【引用】<https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml#L12-L18>
⇒ 它对"斜装"的答案是：**要么把外参放进输入帧的 TF，要么 `two_d_mode` 直接把 roll/pitch 扔掉**；它**从不**插入重力帧。【推断】

#### R13 Cartographer：帧所有权开关 + 强制 2D 投影

```rst
published_frame
  The ROS frame ID to use as the child frame for publishing poses. For example
  "odom" if an "odom" frame is supplied by a different part of the system. In
  this case the pose of "odom" in the *map_frame* will be published. Otherwise,
  setting it to "base_link" is likely appropriate.
...
provide_odom_frame
  If enabled, the local, non-loop-closed, continuous pose will be published as
  the *odom_frame* in the *map_frame*.
publish_frame_projected_to_2d
  If enabled, the published pose will be restricted to a pure 2D pose (no roll,
  pitch, or z-offset). This prevents potentially unwanted out-of-plane poses in
  2D mode that can occur due to the pose extrapolation step
```
【引用】<https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/configuration.rst#L39-L58>

#### R14 hdl_graph_slam：地面平面节点被钉在 z=0

```cpp
// apps/hdl_graph_slam_nodelet.cpp:490-499
      if(!floor_plane_node) {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }
      ...
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
```
【引用】<https://github.com/koide3/hdl_graph_slam/blob/master/apps/hdl_graph_slam_nodelet.cpp#L490-L499>
README 说得很直白：

> "This constraint optimizes the graph so that the floor planes (detected by RANSAC) of the pose nodes becomes the same. This is designed to **compensate the accumulated rotation error of the scan matching** in large flat indoor environments."
> 【引用】<https://github.com/koide3/hdl_graph_slam/blob/master/README.md#L59>

> 注意：`hdl_graph_slam` 的 nodelet 里**没有** `tf::TransformBroadcaster`（`apps/hdl_graph_slam_nodelet.cpp` 全文只出现 `tf::TransformListener`，:1048）⇒ 它**不抢** `odom→base_link` 的所有权。【引用】<https://github.com/koide3/hdl_graph_slam/blob/master/apps/hdl_graph_slam_nodelet.cpp#L1048>

#### R15 linefit：`gravity_aligned_frame` 的原始定义

> "**gravity_aligned_frame**  Name of a coordinate frame which has its z-axis aligned with gravity. If specified, the incoming point cloud will be **rotated, but not translated** into this coordinate frame. If left empty, the sensor frame will be used."
> 【引用】<https://github.com/lorenwel/linefit_ground_segmentation/blob/master/README.md#L57>

### 2.3 从表里读出来的三条规律

1. **LIO 一般不发 `odom→base_link`。** FAST-LIO / Point-LIO 只发自己的内部帧（`camera_init→body` / `camera_init→aft_mapped`），R1/R2/R3/R4/R5/R6 全部是**有人替它改名或改写**才变成 `odom→base_link`。⇒ "**谁发**"在工程上是个**选择题**，不是定理。
2. **合成顺序全行业一致：`T_ol · T_lb`。** R1、R4、R7 三处逐字同构；唯一的反例是 R5 的共轭（上游 bug，本仓已修）。⇒ 我们的修法**与主流一致**，不是自创。
3. **没有任何一行把"重力"做成一条 TF。** 重力出现在三个地方：**估计器参数**（R1）、**估计器初始化代码**（R9）、**消费者的帧名参数**（R15）。⇒ §3 的判决由此而来。

---

## 3. 「重力对齐中间帧」是标准 / 罕见 / 缺席？

### 3.1 判决

| 层面 | 判决 | 依据 |
|---|---|---|
| 作为**一条由本节点发布的、冻结的静态 TF**（`odom_grav` 这种） | **缺席（absent）** | 9 个 RM 栈（R1–R6 及其同族）+ 9 个通用包（R7–R15），**0 例**（§2 全表；§3.2 逐条） |
| 作为**消费者声明的帧名参数** | **存在，但罕见（rare；属"标准参数约定"而非"标准架构"）** | linefit `gravity_aligned_frame`（R15）、本仓 patchwork 同名参数【本仓实测】 |
| 作为**估计器内部的状态/中间量** | **标准（standard）** | Point-LIO `Set_init`（R9）、Cartographer 3D `gravity_alignment`（R13）、hdl 地面约束（R14） |
| REP-105 是否**允许**在 `map` 与 `odom` 之间插帧 | **允许**（但只是"允许"，不是"建议"） | 见 §3.2 (a) |

**一句话结论**：`odom_grav` **不是别人在用的模式**，它是"把估计器内部该做的事（世界系对齐重力）搬到 TF 层"的**局部替代品**；而真实栈把同一件事放在估计器初始化（E1）或 IMU 预处理（E2）里。

### 3.2 证据

**(a) REP-105 只说"可以插中间帧"，没说"要插重力帧"。**

> "**Extra Intermediate Frames**  This graph shows the minimal representation of this graph. The basic topology should stay the same, however, it is fine to insert additional links in the graph which may provide additional functionality. **Pressure Altitude** An example of a potential additional coordinate frame is one to represent pressure altitude for flying vehicles…"
> 【引用】<https://www.ros.org/reps/rep-0105.html#extra-intermediate-frames>

同时 REP-105 关于 `odom` 只规定"world-fixed、连续、可漂"，**没有**规定 `odom` 的 z 必须朝上（规定"z 轴朝上"的是 `map`）：

> "map — The coordinate frame called map is a world fixed frame, **with its Z-axis pointing upwards**."
> "odom — The coordinate frame called odom is a world-fixed frame. The pose of a mobile platform in the odom frame can drift over time, without any bounds."
> 【引用】<https://www.ros.org/reps/rep-0105.html#coordinate-frames>

⇒ 法理上，**"odom 斜着"并不违反 REP-105**；插一条重力帧也不违反。**两边都合法**，所以判决只能靠实践证据。【推断】

**(b) 真实栈的"重力对齐世界系"往往就是 `odom` 本身。** 一个非 ROS、但与 IMU 生态同构的实现把这件事写成注释：

```rust
// pollen-robotics/microduck, duck-ipc-proto/src/lib.rs:4058-4059
    /// Orientation trunk → world, scalar-first `[w, x, y, z]`. The world is the IMU's own
    /// gravity-aligned frame with an arbitrary yaw at boot — the same frame `odom` lives in.
```
【引用】<https://github.com/pollen-robotics/microduck/blob/main/duck-ipc-proto/src/lib.rs#L4058-L4059>

> 这句话把标准做法说穿了：**"重力对齐、yaw 任意的世界系" 就是 `odom` 的定义**，不是 `odom` 的父帧。

**(c) 消费者侧的标准写法是"帧名参数"，而且默认指向 `base_link`/`odom`。** R15 的 linefit 参数、本仓 patchwork 的同名参数（`gravity_aligned_frame`，为空则用传感器系）【本仓实测】`src/rm_perception/patchwork_ground_segmentation/src/patchwork_ground_segmentation_node.cpp:89,197`。它们的共同假设是：**这条帧已经有人发好了**；没人要求它是"新加的一条"。

**(d) 没有一家 RM 战队有 `odom_grav` 这类帧。** 在 9 个克隆仓库里全文检索 `grav*_frame` / `_grav` / `odom_grav`：只命中 `gravity_aligned_frame`（HNU 的 linefit 配置，值为空串）与 point_lio 内部的 `hat_grav`/`gravity_`【本仓实测：本地克隆`.tmp_odomlink/`内 grep】。

**(e) 唯一的"冻结的重力对齐"确实存在，但它在估计器里，而且不叫帧**：Point-LIO 的 `rot_init`（`Set_init`）——它**每个进程只在初始化时算一次**，把世界系 z 对到重力，然后再也不动。这正是"冻结的重力对齐"的正确位置。【引用】R9 +【本仓实测】`src/rm_localization/point_lio/src/IMU_Processing.cpp:34-54`、`src/rm_localization/point_lio/src/laserMapping.cpp:463-476`。

### 3.3 为什么"允许却没人做"（推断）

1. **多一个发布者就多一份真相**：倾斜会被表示两遍（URDF/外参一遍、`odom_grav` 一遍），而 FAST-LIO 维护者对"两次旋转"的批评正是这个形状（"an extra TF can effectively rotate the frame twice"，转引自本仓 `docs/lio_extrinsic_application_survey.md` §0 第 2 条）。
2. **它不解决估计器内部**：LIO 自己算的地图/PCD/scan 仍在斜系里，`/cloud_registered`（frame_id=`odom`）与"看起来水平的帧树"会不一致。
3. **它把"初始化一次性"这件事藏起来了**：读者看不出这条帧是什么时候、按哪一帧的姿态算的；换个出生点/换台车就悄悄错了。
4. **收益可用更便宜的方式拿到**：E1 一行参数（point_lio）或十几行初始化（small_point_lio/FAST-LIO）即可，且顺带把内部地图也摆正。

---

## 4. 时变挂装（云台）：三种实现档位与症状

### 4.1 三种档位（这是本节的核心结论）

| 档位 | 外参从哪来 | 时变可用？ | 代表 |
|---|---|---|---|
| **S（static param / 缓存）** | 参数文件里的常数 rpy/xyz；或构造函数里 `lookupTransform` 一次就缓存 | **否** | `lio_tf_adapter` 的 `rpy/xyz` 参数（本仓）【本仓实测】`src/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:37-38`；LIO-SAM `lidar2Baselink`（R7）；XDU 的平移杆臂（R2） |
| **L（per-message lookup）** | 每条消息按 `stamp` 调 `lookupTransform` | **是** | pb2025 `sensor_scan_generation`（R1）；laohao78 `point_lio`（R4）；上游 small_point_lio（R5）；本仓 small_point_lio（已修合成）【本仓实测】`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:110` |
| **J（关节链）** | URDF/SDF 里 LiDAR 挂在 movable joint 下，`robot_state_publisher` + `joint_state_publisher`（或串口/编码器）发关节角 | **是**（L 档的前提） | pb2025（`parent="gimbal_yaw"`，R1）；XDU（串口角，R2）；`ZhuoruiYUN/Sentry-Navigation`（`gimbal_yaw_joint` type="continuous" + `lidar_joint` 挂在其下，`rpy="0 ${mount_pitch} 0"`）【引用】<https://github.com/ZhuoruiYUN/Sentry-Navigation/blob/main/src/sentry_description/urdf/gimbal.xacro#L11-L13>、<https://github.com/ZhuoruiYUN/Sentry-Navigation/blob/main/src/sentry_description/urdf/mid360.xacro#L13-L16> |

> **本仓落在 S 档 + L 档的混合**：`small_point_lio` 自己是 L 档（逐帧 lookup），`lio_tf_adapter` 是 S 档（常数参数）。⇒ **如果哪天真上云台，`lio_tf_adapter` 会立刻错，而 `small_point_lio` 的 lookup 会活着**。【推断】+【本仓实测】`src/rm_nav_bringup/launch/bringup_sim.launch.py:1376-1377` 自己就写着：`small_point_lio … 自己直发 odom→base_link，因此不经过 lio_tf_adapter`。
>
> 注意本仓 adapter 的 `rpy` **不是**"重力补丁"，而是**挂装关系** `T_imu←base_link`（launch 里由 `_lio_adapter_robot11()` 按几何算出来：`rpy_out = [-v for v in rpy]`、`t = -Rᵀ·p_imu`）【本仓实测】`src/rm_nav_bringup/launch/bringup_sim.launch.py:219-252`。这条区分在 §6.3 会用到。

### 4.2 真例：pb2025 的"斜装 + 云台"是同一件事

把 §2 R1 的四条引文串起来：

1. LiDAR 的 SDF 父关节 = `gimbal_yaw`，且自身带 −(`π/6 + 0.06`) ≈ **−33.44°** 的固定倾角【引用】R1 (SDF)；
2. 于是 `front_mid360` 相对 `base_footprint` 的位姿 = **固定倾角 ∘ 可变 yaw**；
3. `sensor_scan_generation` **每条点云按戳**查 `front_mid360→base_footprint` ⇒ 可变 yaw 自动进 `odom→base_footprint`【引用】R1 (cpp)；
4. 关节角由**编码器/串口**给 `joint_state_publisher`【引用】R1 (README)；
5. Nav2 的 `robot_base_frame` 用 `gimbal_yaw`（瞄准方向），另发一条 `gimbal_yaw→gimbal_yaw_fake` 给"不转的底盘"用【引用】R1 (yaml/fake_vel)。

⇒ **他们并没有"用一个常数把倾斜吸收掉"**，而是**把"固定的部分"和"时变的部分"分开**：倾角走估计器参数，yaw 走关节 TF + 逐帧查。这正是我们该学的形状。【推断】

### 4.3 把时变挂装当静态：症状表

| 错误形态 | 现象 | 机理 |
|---|---|---|
| 常数外参（S 档）用在会动的关节上 | 云台一动，`base_link` 在 `odom` 里"自己游/自己转"；车看着没动，代价地图在转 | `T_odom←base = T_ol · T_lb_const`，缺了 `ΔT_lb(t)`，误差被直接加到基座位姿上【推断】 |
| 外参在两处各乘一次 | 倾角翻倍（−30° 变 −60° 的等效）；点云与地图系统性错位 | 与 FAST-LIO #433 维护者点名的 "**an extra TF can effectively rotate the frame twice**" 同形【引用：转引自 `docs/lio_extrinsic_application_survey.md` §0 C11】 |
| 合成顺序写成共轭 | **假 pitch**：pitch 随 yaw 一起长（本仓修前 `4.890°`），修后 `−0.082°` | `T_bl⁻¹T_olT_bl` 只在 `T_bl=I` 或纯平移时姿态碰巧对【本仓实测】`docs/tilted_lidar_fidelity.md:2959`、推导见 `docs/cod_vs_ours_frame_architecture.md` §3.3 |
| 关节角**时间戳**与点云不一致 | 高速转云台时基座位姿抖动/滞后 | `lookupTransform` 必须用**该帧点云的 stamp**，不能用 `now()`；pb2025 用的是 `pcd_msg->header.stamp`，XDU 用的是 `ros::Time::now()`（更弱）。【引用】R1 vs R2 |
| 用 `latest` 而不是按戳 | 快速运动时旋转量测与位姿错配 | `imu_corrector` 用 `get_latest_transform`（R10）；对 IMU 这种高频量测一般可接受，但对"云台在动"的场景是隐患【推断】 |

> 另一类"地图跟着车转"的公开症状（雷达装偏/杆臂漏掉）：nav2 issue #4135（本仓既有文档引用，本轮**未**复核原文）：<https://github.com/ros-navigation/navigation2/issues/4135>

### 4.4 一个常被忽略的物理事实（重要，能省掉很多设计）

> **若时变的那个自由度是绕重力轴（≈z）的 yaw，则重力矢量在传感器系里不随关节角变化。**

因为把重力从父系搬到子系用的是 `R_child←parent = R_z(θ)·R_mount`，而 `R_z(θ)ᵀ·(0,0,−g) = (0,0,−g)`；yaw 不改变 z 轴。【推断，标准刚体运动学】

推论：

- **纯 yaw 云台**（RM 哨兵最常见）：一次性重力参数（R1 的做法）**在物理上是成立的**，不需要在线估重力；
- **含 roll/pitch 的云台**（如某些会俯仰的云台、或我们这种"倾角挂在会转的关节下且倾角轴不竖直"的构型）：**一次性重力参数立刻失效**，必须靠 IMU 姿态连续估计（§5）或在线标定（§4.5）。**这正是用户说"未必只有 pitch/roll 会变化"的技术含义。**【推断】

### 4.5 在线重估外参：学术侧的做法

> "Dynamic Camera Clusters (DCCs) are multi-camera systems where one or more cameras are mounted on **actuated mechanisms such as a gimbal**. Existing methods for DCC calibration rely on **joint angle measurements to resolve the time-varying transformation** between the dynamic and static camera. This information is usually provided by motor encoders, however, joint angle measurements are not always readily available… we present an **encoderless** approach for DCC calibration which simultaneously estimates the kinematic parameters of the transformation chain as well as the **unknown joint angles**."
> 【引用】<https://arxiv.org/abs/1807.09304>

工程侧对应物：`LI-Init`（LiDAR-IMU 初始化，**在线标定外参 + 时间偏移 + 重力矢量 + 零偏**）：

> "The proposed method calibrates the temporal offset and extrinsic parameter between LiDARs and IMUs, and also **the gravity vector** and IMU bias. Our method does not require any target or extra sensor… or initial values of extrinsic and time offset."
> 【引用】<https://github.com/hku-mars/lidar_imu_init/blob/main/README.md#L5-L9>

⇒ **"在线重估"是真实存在的分支**，但它解决的是"标定值未知/会漂"，不是"关节会动"。**关节会动的主线解法仍然是编码器 + 逐帧 TF**【推断】。

---

## 5. 漂移：一次性帧结构上治不了

### 5.1 一次性变换能做什么、不能做什么

设真实关系 `T_odom←base(t) = T̂_odom←base(t) · E(t)`，`E(t)` 为缓慢增长的误差。

- **一次性变换**（`odom_grav`、E1 的初始化旋转、URDF 外参）只能改 **坐标表示**，即把 `T̂` 左乘/右乘一个常数；对 `E(t)` **零作用**。【推断】
- 因此：**如果一个栈的 roll/pitch 会漂，那么"冻结的重力帧"在 t₀ 之后误差只会越来越大**，而且它会**掩盖**问题（看起来一开始是平的）。【推断】

### 5.2 实践中的三类纠正源

| 类型 | 代表 | 逐字证据 |
|---|---|---|
| **IMU 姿态（重力的连续观测）** | Cartographer 3D `ImuTracker`：用 `imu_gravity_time_constant` 对重力做 EMA，并每步把姿态拧到与重力一致 | "Update the 'gravity_vector_' with an exponential moving average using the 'imu_gravity_time_constant'… Change the 'orientation_' so that it agrees with the current 'gravity_vector_'."【引用】<https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/imu_tracker.cc#L50-L67> |
| **地面/平面约束（图优化层）** | hdl_graph_slam：把地面平面节点设为 `(0,0,1,0)` 且 `setFixed(true)`，用 RANSAC 地面边把每个关键帧钉到地面 | "designed to **compensate the accumulated rotation error of the scan matching**"【引用】<https://github.com/koide3/hdl_graph_slam/blob/master/README.md#L59>、<https://github.com/koide3/hdl_graph_slam/blob/master/apps/hdl_graph_slam_nodelet.cpp#L490-L499> |
| **地图匹配/重定位（离散跳变）** | 重定位节点发 `map→odom`（pb2025 的 `small_gicp_relocalization`） | `transform_stamped.header.frame_id = map_frame_; transform_stamped.child_frame_id = odom_frame_;`【引用】<https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/blob/main/src/small_gicp_relocalization.cpp#L192-L206> |

补充两条"承认漂移、但不假装解决"的工程态度：

- REP-105 把 `odom` 的漂移写进了定义："The pose of a mobile platform in the odom frame **can drift over time, without any bounds**."【引用】<https://www.ros.org/reps/rep-0105.html#coordinate-frames>
- `robot_localization` 的 `two_d_mode` 是另一种态度：**直接把 roll/pitch 从状态里去掉**（"no 3D information will be used in your state estimate… ignore the effect of small variations in the ground plane"）【引用】R12。

### 5.3 结论

> **漂移属于"估计"问题，不属于"坐标"问题。** 冻结帧、URDF 外参、`odom_grav` 都在"坐标"层；roll/pitch 漂移只能靠"每时每刻还有人在看重力/看地面/看地图"。⇒ 用户"未必只有 pitch/roll 会变化"的担忧，**正确回应不是再加一条帧，而是明确"谁在持续观测重力"**。【推断】

---

## 6. 本仓建议

### 6.1 现状分解（谁在做什么）

| 环节 | 现在是谁 | 证据 |
|---|---|---|
| `map→odom` | 建图时 `slam_toolbox`；定位时 `gicp_registration`/AMCL（按 `mode`） | 【本仓实测】`docs/tf_interface_contract.md`、`docs/algorithm_matrix.md` §一 |
| `odom→base_link` | **`small_point_lio` 节点自己发**（已修成合成 + 逐帧 lookup） | 【本仓实测】`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:105-140`、`docs/tilted_lidar_fidelity.md:2959` |
| `base_link→livox_frame` | URDF 静态 TF（−30° roll，杆臂长度 0.204443 m） | 【本仓实测】`docs/tilted_lidar_fidelity.md` §N |
| `odom→base_link`（其它 LIO） | `lio_tf_adapter`（**常数** `xyz`/`rpy` 参数；对 `small_point_lio` **被排除**） | 【本仓实测】`src/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:34-38`、`config/lio_tf_adapter.yaml` |
| 重力对齐 | **没有人在做**（`odom` 仍是初始传感器系） | 【本仓实测】`odom→base_link` rpy 的 30.452° |
| **下游谁假设 z 朝上** | **Nav2 局部代价地图**：`global_frame: odom` + `robot_base_frame: base_link_fake` + `rolling_window: true`；controller 同样 `global_frame: odom` | 【本仓实测】`src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml:128-131`、`:413-414` |
| 感知侧"重力对齐帧" | 默认 `gravity_aligned_frame: ""`（= 用传感器系）；只有 `robot11_mount:=sensor` 那一档被 launch **增量覆盖**成 `base_link`（p2l 同时 `target_frame: base_link`） | 【本仓实测】`src/rm_perception/patchwork_ground_segmentation/config/ground_segmentation_sim.yaml:22`；`src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro:43`、`:790-806` |

**为什么"odom 斜"在本仓是真问题（而不仅是美观问题）**：局部代价地图跑在 `odom` 里、把激光点投影到 `odom` 的 z=0 平面；当 `odom` 相对重力斜 30° 时，这个"平面"本身就斜了 30°，地面点云在该平面上的投影会被**各向异性压缩**（一个方向 ×cos30° ≈ 0.866），障碍物位置随之有约 13% 的系统性畸变，`base_link_fake` 的投影同样被压。【推断，可由 §1.4 + Nav2 的 2D 投影语义直接推出】

⇒ 所以"把 `odom` 摆平"这个诉求**是合理的**；有争议的只是"在哪一层摆平"（§6.2/§6.3）。

### 6.2 四条路线排序（成本 / 风险 / **能不能真的让 `odom` 变水平**）

| 排序 | 路线 | 具体动作 | 能让 `odom` 水平？ | 成本 | 风险 | 备注 |
|---|---|---|---|---|---|---|
| **① 推荐** | **E1：估计器初始化时对齐重力** | `point_lio`：`gravity_align: true` + `gravity/gravity_init`（本仓 fork 已支持 `gravity_init`，`parameters.cpp:176-177`）——**一行参数**；`small_point_lio`：照抄 `ImuProcess::Set_init`（`point_lio/src/IMU_Processing.cpp:34-54`）把 `rot_init` 写进 `kf.x.rotation`（它现在**只设了 `kf.x.gravity`，`rotation` 保持单位阵**：`small_point_lio.cpp:48-64` + `estimator.cpp:22-27`），约 10–20 行 | **✓**（`odom` 从此就是"重力对齐、yaw 任意"的帧，§3.2 (b)） | **极低 / 低** | **低**：只影响初始化；初始化必须**静止**（`mean_acc` 混入运动加速度会带误差）；内部地图/PCD 也一起变水平（好事） | point_lio 原生（R9）；pb2025 就这么干（R1）。**注意**：`odom→base_link` 会从此≈单位阵，别再在别处补 30°（§6.5） |
| **② 次选** | **E2：把 IMU 先摆平再喂 LIO** | 用 `autoware_imu_corrector` 的 20 行（Apache-2.0）或 `imu_transformer`（**GPL-3.0**）把 accel/gyro 旋到 `base_link` 再发 `/livox/imu` | **✓**（LIO 的初始 body 系 = `base_link` ⇒ 世界系水平） | 低 | 中：**协方差**（`imu_corrector` 取最大方差而非旋转协方差，R10）；GPL 传染；**耦合改动**：body 变成 `base_link` 后，`lio_tf_adapter` 的外参必须归零（§6.5） | 唯一适用于"估计器一行都不能改"的路线（FAST-LIO，R8） |
| **③ 备选** | **E4：EKF 拥有 `map→odom`** | `robot_localization`（BSD-3）`world_frame: map`，融合 IMU 姿态 + 重定位位姿 | **✗**（它只接管 `map→odom`，`odom→base_link` 仍由 LIO 发，斜的还是斜的） | 中 | 中高：多一个滤波器多一个调参面；`two_d_mode` 会把 roll/pitch 直接扔掉（R12），**把"倾斜"变成"不可见"而不是"正确"** | 只在"要多源融合/做 `map` 级平滑"时才值得 |
| ✘ 不建议 | **`odom_grav` 冻结帧** | 插一条静态 TF 把 `odom` 摆平 | **✓**（这就是它唯一的真本事） | 低（看起来） | **高**：双份真相 / 内部地图仍斜 / 时变即失效 / 掩盖漂移（§3.3、§1.4） | 见 §6.3：它等价于"在 TF 层做 E1"，但把 E1 的四个前提**藏进了实现里** |

### 6.3 `odom_grav` 的判决与"唯一成立的场景"

**判决：不必要（unnecessary），且方向不对（wrong layer）。** 理由（三条，逐条对应证据）：

0. **先承认它对的部分**：如果只想要"Nav2 局部代价地图别被 30° 压扁"（§6.1），`odom_grav` 确实能做到——它把 `odom` 的**表示**摆平了。所以它是**能干活**的，只是**在错误的层干活**。以下三条说清"代价是什么"。
1. **它替代的是一条更便宜的 E1**：point_lio 只需一行参数（R9/R1）；small_point_lio 只需在初始化处写一次 `rotation`（本仓 `small_point_lio_node.cpp:105-140` 已有现成的 `lookupTransform` 与状态量可挂），而且 E1 顺带把**内部地图/PCD** 也摆平了，`odom_grav` 做不到这一点。
2. **它把倾斜表示两遍**：URDF 已经有 `base_link→livox_frame` 的 −30°（§6.1），再插一条 `odom_grav→odom` 的 30°，就变成"两个地方各自描述同一个物理事实"。只要有一处不同步（换车、换支架、仿真与实车不同），就出现**双倍/半倍倾角**这类难查的错——与 FAST-LIO 维护者批评的 "extra TF … rotate the frame twice" 同形。⚠️ 特别地：**E1 与 `odom_grav` 绝不能同时存在**，那必然是双倍补偿。
3. **它对事二/事三无能为力**：时变挂装（§4）、roll/pitch 漂移（§5）都不在"常数旋转"的能力范围内。

**唯一可以接受的场景**（四条**同时**满足）：**估计器一行都不能改**（第三方二进制/不许动 vendor），**且**挂装 roll/pitch 永远不变，**且**地图/PCD 也在同一个摆平后的帧里重建（否则内部地图与帧树不一致），**且**这条帧的**发布者唯一**、数值来源可追溯到"初始化那一刻的 IMU 姿态"。本仓目前**不满足第 3 条**（PCD 是在 `odom` 里建的）【本仓实测】`docs/mapping_small_point_lio.md`，所以：**不采用**。

> ⚠️ 若将来照抄 pb2025 的 `gravity` 数字，**必须先核对语义**：pb2025 的 `reality` 与 `simulation` 两份配置在**轴向（`# rpy=[0,pi/6,0]` vs SDF 的 roll −33.44°）与模长（0.9921 vs 9.81）上互不一致**，说明这些数字是**手调出来的**，不是可直接搬运的物理量。【引用】R1 (两份 yaml + SDF)【推断】

### 6.4 要不要自研一个包？—— **不要**

| 我们以为要自研的能力 | 现成的东西 | 许可 | Humble 可用 | 判决 |
|---|---|---|---|---|
| `odom→base_link` + 挂装外参 + 逐帧查 TF | **本仓 `small_point_lio` 已实现**（L 档）；或 `sensor_scan_generation`（pb2025，Apache-2.0）；或 LIO-SAM 的 `TransformFusion`（BSD-3，S 档） | MIT / Apache-2.0 / BSD-3 | 是 | **adopt**：把 `lio_tf_adapter` 从 S 档升级成 L 档（把 `rpy/xyz` 参数换成 `lookupTransform(lidar_frame, base_frame, stamp)`），**约 20 行** |
| 重力对齐 | Point-LIO 原生（GPL-2.0，`Set_init`）；Cartographer 3D 原生 | GPL-2.0 / Apache-2.0 | 是 | **adapt**（E1）：point_lio 一行参数；small_point_lio 十几行 |
| 摆平的 IMU（E2） | `autoware_imu_corrector`（Apache-2.0，可只抄 20 行 TF+旋转）；`imu_transformer`（**GPL-3.0**） | Apache-2.0 / GPL-3.0 | 是 | **adapt**；`imu_transformer` **avoid**（GPL 传染，除非整栈 GPL 兼容） |
| `map→odom` 唯一所有者（E4） | `robot_localization`（BSD-3）、`small_gicp_relocalization`（Apache-2.0，若采用需核许可） | BSD-3 / Apache-2.0 | 是 | **adapt**（只在需要多源融合时） |
| 地面/坡度约束 | hdl_graph_slam（BSD-2）太重；本仓已有 `patchwork`/`linefit` + `gravity_aligned_frame` | BSD-2 | 是 | **avoid**（为一个约束引入整套图优化不划算） |
| 在线外参标定 | `LI-Init`（HKU，需核许可） | 未核 | 部分（ROS1 为主） | **avoid**（当前不需要） |

**结论**：**没有任何一项需要我们写新包。** 需要的两个动作都是"改现成节点里的十几行"：(a) `lio_tf_adapter`/`small_point_lio` 的外参来源从"常数"改成"逐帧 TF"（`small_point_lio` 已经是了）；(b) E1 的重力对齐（`point_lio` 一行 / `small_point_lio` 十几行）。

### 6.5 同步改什么（迁移陷阱，做完 E1/E2 必须一起看）

| 做完哪条 | `odom→base_link` 会变成 | 必须同步改什么 | 不改会怎样 |
|---|---|---|---|
| **E1** | ≈ 单位阵（倾角被初始姿态吸收，外参在两处相消） | **什么都不用改**；但**不要**再往任何地方加 30°（尤其不要同时上 `odom_grav`） | 双倍/半倍倾角 |
| **E2** | ≈ 单位阵，且 LIO 的 `body` 帧**语义变成 `base_link`** | `lio_tf_adapter` 的 `xyz`/`rpy` 必须**归零**（launch 里 `_lio_adapter_robot11()` 的返回值此时不再适用）；`small_point_lio` 的 `lookupTransform(lidar_frame, "base_link")` 也要重新审视（它的 `T_bl` 语义会变） | 外参被**乘第二次** ⇒ 与 FAST-LIO #433 同形的"转两次" |
| **E4** | 不变（仍斜） | 无 | 局部代价地图仍然是斜的（E4 不解决 §6.1 的问题） |

> 这三行是本仓过去几个 bug（共轭、双重变换、`lio_tf_adapter` 被排除）的**共同根因**：**同一个物理倾角在多个层各自被"补"了一次**。任何新方案（含 `odom_grav`）都要先回答"这一次补的是哪一份，别的层要不要撤"。【推断】+【本仓实测】`docs/cod_vs_ours_frame_architecture.md` §3.3、§7.2

### 6.6 如果只做一件事

> **先做 E1（估计器初始化对齐重力），什么都不加。**
> 做完之后：`odom` 自己就是"重力对齐、yaw 任意的世界系"（§3.2 (b) 的定义）；`odom→base_link` 的 30.452° 自然归零；Nav2 局部代价地图不再被 30° 压扁（§6.1）；内部地图/PCD 也一起水平；`gravity_aligned_frame` 这个参数可以直接填 `odom`；**`odom_grav` 这条帧就完全没有存在理由**。
>
> 若选的是 `small_point_lio` 且不愿动它的初始化代码，就退一步做 **E2**（摆平的 IMU），**同时按 §6.5 把 adapter 外参归零**。

---

## 7. 引用清单与打不通清单

### 7.1 外部引用（按出现顺序，共 **36** 条；本仓证据另见 §7.2）

| # | URL | 用途 |
|---|---|---|
| C1 | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/sensor_scan_generation/src/sensor_scan_generation.cpp#L74-L82 | R1 合成 |
| C2 | …/sensor_scan_generation.cpp#L89-L101 | R1 lookup 失败退单位阵 |
| C3 | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/pb2025_nav_bringup/config/reality/nav2_params.yaml#L63-L81 | R1 reality 重力 + `tf_send_en: False` |
| C4 | …/config/simulation/nav2_params.yaml#L63-L64 | R1 sim 重力 `# rpy = [0, pi/6, 0]` |
| C5 | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/README.md#L25 | "mounted at an incline" + 谁发 `odom→chassis` |
| C6 | …/README.md#L222 | 云台关节由串口模块提供 |
| C7 | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description/blob/main/resource/xmacro/pb2025_sentry_robot.sdf.xmacro#L24-L25 | LiDAR 挂 `gimbal_yaw`、−33.44° |
| C8 | …/pb2025_sentry_nav/blob/main/fake_vel_transform/src/fake_vel_transform.cpp#L136-L146 | 动态 fake 基座帧 |
| C9 | https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/blob/main/src/small_gicp_relocalization.cpp#L192-L206 | `map→odom` 所有者 |
| C10 | https://github.com/IRobot-Algorithm/sentry_navigation/blob/main/src/Point-LIO/src/laserMapping.cpp#L729-L748 | R2 LIO 自发 + 只加平移 |
| C11 | …/src/nav_transporter/src/usb_comm_node.cpp#L423-L425 | R2 串口角度发云台 TF |
| C12 | https://github.com/LRaina215/HNU_NHS_SENTRY_UP/blob/main/rm_navi/rm_localization/point_lio/src/laserMapping.cpp#L617-L646 | R3 自发 `odom→base_footprint` |
| C13 | …/rm_localization/point_lio/config/mid360.yaml#L41-L42 | R3 `gravity_align: true` |
| C14 | https://github.com/laohao78/ROS2_RM_Navigation/blob/main/src/localization/point_lio/src/laserMapping.cpp#L302-L331 | R4 逐帧 lookup + 合成 |
| C15 | …/src/rm_nav_bringup/urdf/sentry_robot_real.xacro#L156-L160 | R4 URDF `livox_joint` rpy 参数化 |
| C16 | https://github.com/Yancey2023/small_point_lio/blob/ros2/src/small_point_lio_node.cpp#L64-L75 | R5 共轭（上游） |
| C17 | https://github.com/qza36/COD_NAV/blob/master/FAST_LIO/FAST_LIO/src/laserMapping.cpp#L648-L658 | R6 只改名 |
| C18 | https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L46-L51 | R7 构造函数缓存外参 |
| C19 | …/src/imuPreintegration.cpp#L124-L130 | R7 合成 + 发 `odom→base_link` |
| C20 | …/src/imuPreintegration.cpp#L92 | R7 同时发 `map→odom` |
| C21 | https://github.com/TixiaoShan/LIO-SAM/blob/master/src/mapOptmization.cpp#L1645-L1651 | R7 另一条 `odom→lidar_link` |
| C22 | https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L591-L619 | R8 只发 `camera_init→body` |
| C23 | https://github.com/autowarefoundation/autoware_universe/blob/main/sensing/autoware_imu_corrector/README.md#L5-L8 | R10 能力边界（无重力/无 roll-pitch） |
| C24 | …/sensing/autoware_imu_corrector/src/imu_corrector_core.cpp#L160-L176 | R10 TF → `base_link` 重发 |
| C25 | https://github.com/ros-perception/imu_pipeline/blob/ros2/imu_transformer/launch/ned_to_enu.launch.xml#L2-L16 | R11 NED→ENU 样例 |
| C26 | …/imu_transformer/src/imu_transformer.cpp#L20-L46 | R11 `target_frame` + `tf2_buffer_->transform` |
| C27 | …/imu_transformer/LICENSE | R11 GPL-3.0 |
| C28 | https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml#L44-L72 | R12 帧所有权 + `publish_tf` |
| C29 | …/params/ekf.yaml#L12-L18 | R12 `two_d_mode` 语义 |
| C30 | https://github.com/cartographer-project/cartographer_ros/blob/master/docs/source/configuration.rst#L39-L58 | R13 `published_frame`/`provide_odom_frame`/2D 投影 |
| C31 | https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/imu_tracker.cc#L50-L67 | §5 重力 EMA |
| C32 | https://github.com/koide3/hdl_graph_slam/blob/master/README.md#L59 | R14 地面约束的目的 |
| C33 | …/apps/hdl_graph_slam_nodelet.cpp#L490-L499 | R14 平面节点固定 |
| C34 | https://github.com/lorenwel/linefit_ground_segmentation/blob/master/README.md#L57 | R15 `gravity_aligned_frame` 定义 |
| C35 | https://www.ros.org/reps/rep-0105.html | §3.2 REP-105（`odom`/`map` 语义 + 允许中间帧） |
| C36 | https://github.com/pollen-robotics/microduck/blob/main/duck-ipc-proto/src/lib.rs#L4058-L4059 | §3.2 "odom 就活在那个重力对齐帧里" |

补充引用（非编号主体，正文内已给链接）：`https://github.com/ZhuoruiYUN/Sentry-Navigation`（云台关节 + `mount_pitch`）、`https://arxiv.org/abs/1807.09304`（云台动态标定）、`https://github.com/hku-mars/lidar_imu_init`（在线标定+重力）、`https://github.com/ethz-asl/ethzasl_msf`（拒绝自研的又一个理由）、`https://github.com/locusrobotics/fuse`、`https://github.com/ros-navigation/navigation2/issues/4135`（未复核）。

### 7.2 本仓证据索引（【本仓实测】逐条落点）

> 本文**没有**重跑任何节点；下列均为**已落盘的文件/行**，可直接打开核对。

| 事实 | 落点 |
|---|---|
| `odom→base_link` rpy = `[30.452, −0.082, 0.211]°`（修后）、`4.890°` 假 pitch（修前） | `docs/tilted_lidar_fidelity.md:2959` |
| `small_point_lio` 逐帧 `lookupTransform(lidar_frame, "base_link", time_msg)` + 合成 | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:105-140`（lookup 在 `:110`） |
| `lio_tf_adapter` 外参是**常数参数**（`xyz`/`rpy`），且对 `small_point_lio` 不启动 | `src/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:34-38`；`src/rm_nav_bringup/launch/bringup_sim.launch.py:1376-1377` |
| adapter 的 `rpy` 是**挂装关系**（几何算出，非重力补丁） | `src/rm_nav_bringup/launch/bringup_sim.launch.py:219-252` |
| Nav2 局部代价地图在 `odom` 里、`robot_base_frame: base_link_fake` | `src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml:128-131`、`:413-414` |
| `gravity_aligned_frame` 默认空；仅 `robot11_mount:=sensor` 档覆盖成 `base_link` | `src/rm_perception/patchwork_ground_segmentation/config/ground_segmentation_sim.yaml:22`；`src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro:43`、`:790-806` |
| `point_lio` 支持 `gravity_init` / `publish.tf_send_en` | `src/rm_localization/point_lio/src/parameters.cpp:176-177`、`:198-199`；`src/rm_localization/point_lio/src/laserMapping.cpp:256-269`、`:463-476` |
| `Set_init` 的重力对齐算法 | `src/rm_localization/point_lio/src/IMU_Processing.cpp:34-54` |
| `small_point_lio` **没有** `rot_init`：只设 `kf.x.gravity`，`rotation` 保持单位阵 | `src/rm_localization/small_point_lio/src/small_point_lio/small_point_lio.cpp:48-64`、`src/rm_localization/small_point_lio/src/small_point_lio/estimator.cpp:22-27` |
| 共轭 vs 合成的推导与判决 | `docs/cod_vs_ours_frame_architecture.md` §3.3、§7.2 |
| PCD 在 `odom` 里建（重建后才谈得上"摆平的帧"） | `docs/mapping_small_point_lio.md` |

### 7.3 打不通 / 需更正 / 未验证

| 项 | 状态 | 说明 |
|---|---|---|
| `tu-darmstadt-ros-pkg/hector_pose_estimation` | **打不通（404）** | GitHub 仓库 API 与 raw README（`master`/`melodic-devel`）均 404；检索 `hector_pose_estimation` 只得一个无关 fork。⇒ **无法引用其代码**，本文未把它写进对照表。若需要，得走 ROS wiki 镜像（本轮未做）。 |
| `ros-perception/imu_pipeline` 的 `master` 分支 | **404 → 更正** | 默认分支是 **`ros2`**；`imu_transformer` 在 `ros2` 分支下存在。 |
| LIO-SAM `src/transformFusion.cpp` | **已不存在** | 当前 `master` 只有 `featureExtraction/imageProjection/imuPreintegration/mapOptmization` 四个 cpp；`TransformFusion` 类**搬进了 `imuPreintegration.cpp`**。⇒ 网上大量"引用 `transformFusion.cpp`"的二手资料在**现行 master 上对不上**，本文按 `imuPreintegration.cpp` 引用。 |
| 本仓既有文档对 COD 的归属 | **需更正** | `qza36/COD_NAV` 的 `master`（末次提交 2025-08-02）**不含** `small_point_lio` 目录；`small_point_lio_node.cpp` 的共轭代码属**上游 `Yancey2023/small_point_lio`**（本仓 vendor 的同一份）。本文因此把 R5 归给上游、R6 归给 COD 的 FAST_LIO。 |
| pb2025 的 `point_lio` / `small_gicp_relocalization` | **浅克隆未取子模块** | 二者是 git submodule；本文用 `SMBU-.../point_lio`（`RM2025_SMBU_auto_sentry` 分支）与 `SMBU-.../small_gicp_relocalization`（`main`）单独 `raw` 取文件。 |
| GitHub **code search** | **不可用** | 未带 token，`/search/code` 需要认证；本文用 repo 检索 + 浅克隆 + 本地 grep 替代，**不排除**漏掉未按关键词命名的实现。 |
| pb2025 `gravity` 参数的**精确语义** | **未验证（诚实边界）** | 上游 `Set_init` 把"把 `gravity` 旋到实测重力"的旋转直接赋给 `x_.rot`；该旋转按 Rodrigues 是 `R_body←world`，而 `x_.rot` 又被当作 `R_world←body` 使用（`pointBodyToWorld`）。**两种读法在"参数=实测值 ⇒ 旋转≈I"时结果相同**，所以 pb2025 的数值能自洽，但**语义无法仅从代码断言**。⇒ 本文只断言"**倾斜被写进了估计器参数**"（可逐字验证），**不断言**"它一定把世界系摆正了"。 |
| pb2025 云台的 roll 轴是否随 yaw 转 | **未验证** | SDF 里 −33.44° 写在 `gimbal_yaw` **之下**，严格说 roll 轴会随 yaw 转 ⇒ §4.4 的"yaw 不改重力"结论**不严格成立**。但他们的 `gravity` 是常数（一处），说明工程上按"近似成立"处理，或 gimbal 实际不转 LiDAR。⇒ 该点标为【推断】，建议实机核对。 |
| 本仓"上云台"场景 | **不适用（当前无云台）** | 本仓 `base_link→livox_frame` 是**固定关节**（−30° roll），没有 movable joint；§4 的时变结论是**前瞻性**的。 |

---

*本文只做调研，不含任何代码/配置/launch 改动，未运行仿真。*

---

## 9. 本仓实测（2026-10-19）：斜装该怎么被「初始化」吸收 —— `gravity` 参数 vs `Set_init`

> **本节是受控实验**（`gui:=False`、`robot:=robot11 robot11_mount:=sensor` 的**诚实斜装链**、
> 静止、同一世界 `RMUL2026` / 出生点 / `spin_speed:=0.0`、`lio:=` 三档 + `small_point_lio` 五种初始化档）。
> 原始数据全部在 `.tmp_gravtest/`（gitignored；每个变体一个目录：`probe.json` / `clouds.npz` /
> `launch.log` / `params.txt` / `log_grep.txt`），汇总脚本同目录（`grav_probe.py`、`raw_world_fit.py`、
> `squash.py`、`summarize.py`、`experiment.patch`）。
> ⚠️ 本文档末尾那句「不含任何代码改动」是 2026-10-19 之前写的；§9 带了一个**默认 off** 的补丁（见 §9.5），
> 默认路径实测与未打补丁的二进制**同值**（§9.3 的 `v0_base` vs `smoke0`）。

### 9.1 先读代码：参数该覆盖什么、在哪个系（file:line）

| 事实 | 落点 | 推论 |
|---|---|---|
| 传播用 `v += (R·acc + g)·dt` | `src/rm_localization/small_point_lio/src/small_point_lio/eskf.h:103` | `x.gravity` 是**世界系**重力；`x.acceleration` 是**body(IMU) 系**比力（`z = a_meas·scale − acc − ba`，`estimator.cpp:113`） |
| `fix_gravity_direction: true` ⇒ `g = −|g|·Σacc/|Σacc|`（200 帧） | `small_point_lio.cpp:50-57`（HEAD） | **只借参数的模长**，方向来自 IMU；产出的正是"世界=初始传感器系"里的世界系重力 |
| `false` ⇒ `g = parameters.gravity` 逐字 | `small_point_lio.cpp:57-59`（HEAD） | 参数被当成**世界系**用（R=I 时与世界=body 不可分 ⇒ pb2025 那种"body 系读法"在数值上也能自洽） |
| 初始化**从不写** `rotation`；`x.rotation` 只在 `eskf.h:30` 初始化成 `Identity()` | `eskf.h:30`、`small_point_lio.cpp:45-79`（整块 `!is_init` 里没有 `rotation =`） | **世界系 = 初始化时刻的传感器系**（斜 30°）。参数再怎么写都不能把世界 z 摆正 |
| 参数已存在：`parameters.cpp:22-25`；三份配置 `gravity: [0,0,-9.810]` + `fix_gravity_direction: true` | `config/mid360*.yaml` | **不需要新增参数名**；缺的是"初始旋转" |
| 初始化把 `point_deque`（**雷达系**点）直接塞进 ivox | `small_point_lio.cpp:49-52`（HEAD） | 等价于假设 init 时 `R=I, t=0`。**一旦设 R≠I，初始地图会被整体转过 R** ⇒ E1 在 small_point_lio 里不是"5 行"，初始地图那 3 行必须一起改 |
| 对照：`Set_init` 把 `rot_init`（实测重力→世界 −z 的最小旋转）写进 `x_.rot`，且 `x_.acc = −rot_init^T·g` | `point_lio/src/IMU_Processing.cpp:34-56`、`laserMapping.cpp:466-478` | point_lio 的 `mapping.gravity` 是**世界系目标**；它同时有"参数 + 初始旋转"两件事 |

**⇒ STEP 0 的答案**：`gravity` 参数必须覆盖 **`state.gravity`（世界系）**，但**光覆盖它不够** ——
必须**同时**覆盖 `state.rotation`（初始姿态）与 `state.acceleration`（body 系比力，`= −Rᵀg`），
否则 `v̇ = R·acc + g ≠ 0`。`gravity` 参数本身的系：**世界系**（`fix_gravity_direction=false` 时逐字使用）。

### 9.2 测量方法与它的标定（先证明能量出 0° vs 30°）

**主指标（统一口径，跨 LIO 可比）**：把**同一朵原始云** `/livox/lidar/pointcloud`（~11.8k 点、`livox_frame`）
经 TF 搬进 LIO 自己的世界系：
`p_world = T_world←base · T_base←livox · p_raw`（前一条 = `odom→base_link`，后一条 = URDF 静态 `base_link→livox_frame`），
然后在 **r∈[0.8, 8] m、且平面离世界原点 0.05–0.60 m** 的点上做 RANSAC（700 次、0.03 m）+ 内点 SVD 精修，
**残差 = 地面法向 vs 该世界系 +z 的夹角**（法向朝 z>0），同时报内点数、内点残差 p50/p95、倾角轴。

*为什么这样选点*（三点，都可复核）：
1. **不能只用"正前方锥"**：物理斜装 −30° 下这台 Mid-360 的可见地面只落在**左侧/左后**（本次实测内点方位
   **36°–167°**、r 0.6–5.5 m），正前方锥里地面点不足 ⇒ 会拟到竖直面。改用"可见地面 swath + 地面是唯一
   离原点 0.05–0.60 m 的大平面"（传感器离地 0.216 m；墙/车身的 \|d\| ≥ 3 m 或内点跨度 < 1 m 被排除）。
2. **自击/车身**：近场点团簇在 r<0.2 m、跨度 <0.1 m ⇒ 被 `r>0.8` + 内点跨度下限（max 1.5 m / min 1.0 m）排除。
3. 同一帧另报一个**无先验**的"整帧最大平面"角作交叉检查（两者差 <0.06°）。

**次指标**：`/cloud_registered` 自己 `frame_id` 上的同类拟合。只有 `small_point_lio` 的云够密（11917 点/帧）；
`pointlio`/`fastlio` 的 `/cloud_registered` 只有 **145/146 点/帧**（`filter_size_*: 0.5` + `point_filter_num: 3`）⇒ 拟不出，
所以主指标用原始云（这对三档 LIO 是同一份输入，反而更公平）。

**标定（必须能分开 0° 与 30°，否则指标不能用）**：

| 对照 | 主指标（原始云→TF） | 次指标（`/cloud_registered`） | `odom→base_link` rpy（独立证人） |
|---|---|---|---|
| `pointlio`（原生 `Set_init`，正向对照） | **0.166°** | 拟不出（145 点） | [-0.22, -0.07, 0.02]° |
| `small_point_lio`（现状，基线） | **30.110°** | 30.057° | [30.03, -0.09, -0.13]° |
| 同上，**未打补丁**的二进制（`smoke0`） | 30.129° | 30.053° | [30.05, -0.08, -0.14]° |
| `fastlio`（不做对齐，负向对照） | **30.464°** | 拟不出（146 点） | [30.44, -0.39, -0.33]° |
| 真值：`/odom_ground_truth` 的 base_link rpy | — | — | [0.017, 0.000, 0.004]°（车真的水平） |
| IMU 加速度均值在 **base_link** 里 vs +z | — | — | **0.019°**（安装角是真的 −30°：在 imu 系里 29.98°） |

⇒ **指标分辨力 = 30.1° / 0.17° ≈ 180×**，三个独立证人（原始云→TF、`/cloud_registered`、TF rpy）互相一致；
指标自身的系统偏差 ≲0.15°（30.11° 比 IMU 实测倾角 29.98° 高 0.13°）。

**障碍物压扁（§6.1 的 ~13%）—— 用数据量，不靠推**：取同一朵云的地面内点，随机 20 万对，比较
**3D 距离 L3**（odom 是刚体变换 ⇒ L3 = 真值）与**投影到世界 z=0 的 2D 距离 L2**，按"与坡向的夹角"分箱：

| 与坡向夹角 | 0–15° | 15–30° | 30–45° | 45–60° | 60–75° | 75–90° |
|---|---|---|---|---|---|---|
| 现状（残差 30.05°）的 L2/L3 中位 | **0.8683** | 0.8866 | 0.9181 | 0.9531 | 0.9823 | 0.9980 |
| 已对齐（`v6b`，残差 0.55°） | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

⇒ 沿坡向 **压缩 13.17%**，垂直坡向 0.2%；理论值 `1−cos(30.05°)=13.44%` ⇒ 与"~13%"一致（差 0.27 个百分点）。
**这就是 nav2 局部代价图（`global_frame: odom`）看到的那个压缩**：一个贴地、沿坡向 1 m 的足迹在 odom 里只剩 0.868 m。

### 9.3 矩阵表（同一世界/出生点/时长；`gui:=False`、`spin_speed:=0.0`、静止）

`fix` = `fix_gravity_direction`；`align` = 本补丁的 `gravity_align_mode`；残差单位 deg。
`—` = 该档云太稀拟不出；「发散」= odom 位置在 15 s 内跑到百米量级、地面拟合无解。

| # | 变体 | lio / align | gravity / fix | 残差（主，原始云→TF） | 残差（次，`/cloud_registered`） | `odom→base_link` rpy | 判决 |
|---|---|---|---|---|---|---|---|
| 1 | `v0_base`（现状） | spl / off | (0,0,−9.810) / true | **30.110** | 30.057 | [30.03,−0.09,−0.13] | 稳定，未对齐（= 安装角） |
| 2 | `smoke0`（**未打补丁**二进制） | spl / — | 同上 | 30.129 | 30.053 | [30.05,−0.08,−0.14] | 回归基准：与 #1 同值 |
| 3 | `v3_pointlio`（正向对照） | pointlio（原生 Set_init） | (0,0,−9.81) | **0.166** | — | [−0.22,−0.07,0.02] | 稳定，已对齐 |
| 4 | `v4_fastlio`（负向对照） | fastlio | — | **30.464** | — | [30.44,−0.39,−0.33] | 稳定，未对齐 |
| 5 | `v1_worldparam` | spl / off | **(0,−4.905,−8.495)** / false（pb2025 字面值） | **发散** | 发散 | [110.67,−27.07,−49.54] | **发散**：odom 平移 (−716,−530,−275) m |
| 6 | `v2_bodyparam` | spl / off | (0,+4.905,−8.495) / false（= 代码自动算出的那一个） | **发散** | 发散 | [43.08,−0.95,−25.22] | **发散**：(212,−933,−848) m |
| 7 | `v2b_lateinit` | spl / off | (0,+4.905,−8.495) / false + `init_map_size=120000` | **30.670** | 30.597 | [30.62,−0.23,0.16] | **稳定、仍 30.6°** ⇒ 参数本身不转世界系 |
| 8 | `v5_imu` | spl / **imu** | (0,0,−9.810) / true | **0.519** | 0.615 | [0.39,−0.33,−0.26] | 稳定，已对齐 |
| 9 | `v5d_imu`（重跑） | spl / **imu** | 同上 | **0.564** | 0.566 | [0.37,−0.30,−0.28] | 稳定；日志 `rot=(-30.005,−0.012,0.003)°` |
| 10 | `v5b_ginit` | spl / imu + `gravity_init` | (0,0,−9.810) / false | **发散** | 发散 | [−142.8,1.4,37.1] | **发散** 546 m（早期初始化） |
| 11 | `v5c_deque` | spl / imu（deque 均值） | (0,0,−9.810) / false | **发散** | 发散 | [75.2,51.0,−8.4] | **发散**（早期初始化） |
| 12 | `v6_urdf` | spl / **urdf** (−30,0,0) | (0,0,−9.810) / false | **3.132**（p95 25 mm） | 0.893 | [0.57,−0.61,0.19] | 未发散但**云被拖花**（早期初始化） |
| 13 | `v6b_urdf` | spl / **urdf** (−30,0,0) | (0,0,−9.810) / true | **0.474**（p95 4.8 mm） | 0.562 | [0.27,−0.32,−0.12] | 稳定；**本表最优** |
| 14 | `v7_hybrid` | spl / hybrid (−30,0,0) | (0,0,−9.810) / false | **2.995**（p95 26 mm） | 0.951 | [0.65,−0.60,0.09] | 云被拖花（早期初始化） |
| 15 | `v7b_hybrid` | spl / **hybrid** (−30,0,0) | (0,0,−9.810) / true | **0.581**（p95 4.0 mm） | 0.581 | [0.44,−0.24,−0.12] | 稳定；日志 `Δ(URDF,IMU)=0.011° ⇒ INFO/URDF` |
| 16 | `v8_hybrid_bad` | spl / hybrid (**0,+30,0** 错轴) | (0,0,−9.810) / false | **发散** | 发散 | [23.4,−28.8,−133.5] | 早期初始化 + 那次**没有 IMU 可用** ⇒ 跳过互检 ⇒ 发散 |
| 17 | `v8b_hybrid_bad` | spl / hybrid (**0,+30,0** 错轴) | (0,0,−9.810) / true | **0.646** | 0.685 | [0.45,−0.35,−0.09] | 日志 `Δ=42.206° ≥ 5° ⇒ ERROR，改用 IMU` ⇒ **恢复**（否则会是 30°） |
| 18 | `v6c_urdf_long` | spl / urdf (−30,0,0) | (0,0,−9.810) / true，**settle 45 s** | **0.610**（p95 8.3 mm） | 0.505 | [0.28,−0.35,0.01] | 漂移检验：与 #13（15 s）**无显著增长** ⇒ 0.5° 更像稳态偏置 |

**每档的 STEP-3 附带量**（表格太长，只列结论；逐档数字在 `.tmp_gravtest/<tag>/probe.json`）：

| 量 | 实测（各档共同） | 读法 |
|---|---|---|
| `/cloud_registered` | `frame_id` = `odom`（spl）/ `camera_init`（pointlio、fastlio）；**9.999–10.000 Hz**；11917 点/帧（spl）/ 145–147（pointlio、fastlio） | 帧名随槽位变（`camera_init` 与 `odom` 同向，adapter 只做杆臂），频率/点数与 LIO 无关 |
| `/scan` | `frame_id` = `base_link`（`robot11_mount:=sensor`）；**1462 束 / 676–687 有限 / 跨度 5.672–5.686 m**（lo 0.189、hi 5.861） | **odom 斜不斜都不改 `/scan` 的波束集合** —— 这一档 linefit `gravity_aligned_frame:=base_link`、p2l `target_frame:=base_link`，两者都在 base_link 里算 ⇒ 「odom 斜 ⇒ 丢掉哪些波束」只在默认/`urdf` 档成立（§I 的表：755 vs 948） |
| RTF | 0.400–0.409（pointlio 0.405、fastlio 0.375） | 对齐/未对齐**同量级**（本实验静止、无导航栈） |
| RSS（`small_point_lio`） | 33–35 MB（`off`/`imu`/`urdf`/`hybrid` 全一样）；pointlio 83 MB | 补丁**零运行时成本** |
| 初始化耗时（墙钟代理：首个 gzserver 插件日志 → LIO 首条日志） | `fix=true`（200 帧门）**7.6–8.2 s**；`fix=false`（第一帧就 init）**3.5–4.2 s**；`v2b`（点云数门 120k）7.0 s | 200 帧门 ≈ 2.0 s 机时（仿真 RTF≈0.4 ⇒ 约 4 s 墙钟）；这是"要不要省这 2 s"的实测代价 |
| 话题发布者数 | `/livox/lidar`、`/cloud_registered`、`/scan`、`/odom`、`/segmentation/obstacle` **各 = 1**（每档都查） | 契约不变 |
| 不稳定性 | 见 §9.3 的「发散」**5 例**（v1、v2、v5b、v5c、v8）+「云被拖花」**2 例**（v6、v7） | 全都发生在 `fix=false`（早期初始化）那一组（8 个 `fix=false` 变体里唯一稳定的是把 init 推迟的 v2b） |

### 9.4 判决

**(1) 代码期望哪个约定？** `state.gravity` = **世界系**（`eskf.h:103`）；`fix_gravity_direction=false` 时
`parameters.gravity` 被**逐字当世界系重力**用。pb2025 的 `[0.0, −4.9, −8.487]` 在数值上
= `R_x(−30°)·(0,0,−9.81)`（把"世界向下"按安装角转一次），它在**我们的**代码里恰好是**错的那一边**：
- 我们的代码自动算出的正确世界系重力 = **(0, +4.905, −8.495)**（= `R_x(+30°)·(0,0,−9.81)`，因为世界=传感器系）；
- 把 pb2025 的字面值喂进去（`v1`）⇒ 与实测比力差 ~9.8 m/s² ⇒ **15 s 发散 897 m**；
- 即使喂**对的**那一个（`v2`/`v2b`）⇒ 世界 z **不会**被摆正：`v2b`（把初始化推迟到落定后）稳稳停在 **30.67°**。

**(2) pb2025 的"gravity 当初始化参数"能复现他们的结果吗？** 在我们的 `small_point_lio` 里 **不能**。
他们那套能成立，是因为 point_lio 里 `gravity` 参数**只是 Set_init 的目标**、真正摆正世界的是
`kf.x_.rot = rot_init`（`laserMapping.cpp:472-477`）—— 也就是"参数 + 初始旋转"两件事。**单搬数字不行**。

**(3) 只设参数 vs 设初始旋转**：`v2b`（只设参数、值还对、稳定）**30.67°**；`v6b`（urdf 先验 + 初始旋转）**0.474°**。
⇒ **必须动初始旋转**，且（因为 §9.1 那条初始地图的假设）必须**同时**把初始地图放进世界系。

**(4) 残差 < 0.5° 吗？** **勉强，只有 urdf 先验这一档过线**：`v6b` **0.474°**（边际）；
`imu` 0.519–0.564°；`hybrid` 0.581°；pointlio 0.166° 才是干净地 < 0.5°。
而且这 0.5° **不是初始化误差**：`v5d` 的日志显示初始旋转 `R_world_body rpy = (-30.005, −0.012, 0.003)°`
（准到 0.005°），`v6b` 的先验更是精确的 −30.000° —— 残差却仍有 0.47–0.56° ⇒ 误差产生在**初始化之后**
（本 LIO 用 0.2 m 体素地图 + 单时间戳帧的固有姿态偏置；pointlio 的 0.166° 说明链路本身能到 0.2°）。
**它是稳态偏置而不是持续漂移**：同档把 settle 从 15 s 拉到 45 s（`v6b` 0.474° → `v6c` **0.610°**，
次指标 0.562° → **0.505°**，p95 4.8 → 8.3 mm）⇒ 两个指标一个略升一个略降、幅度都在 ±0.1° 的
跑间/测量噪声内，**没有随 3 倍时间长大**。

**(5) 三个来源怎么选**：
| 来源 | 需要什么 | 本仓实测残差 | 优点 | 风险 |
|---|---|---|---|---|
| `imu`（200 帧均值，= Set_init） | IMU + 静止 2 s | 0.519–0.564° | **不需要任何安装参数**；真机通用 | 必须静止；起步就在动 ⇒ 均值带误差（`v5c` 用 1 s 内的 deque 均值 ⇒ 发散） |
| `urdf`（先验 rpy） | URDF（或手填 `gravity_align_rpy`） | **0.474°** | **不需要 IMU、不需要静止**，瞬时 | 先验错（换支架/轴写错）⇒ 世界按错的角度摆正、**没有任何告警** |
| `hybrid`（先验 + IMU 互检） | 两者 | 0.581°（Δ=0.011°） | 先验给精度，**Δ 当健康信号**：≥5° ERROR + 回退 IMU（`v8b`：Δ=42.2° ⇒ 恢复 0.646°，否则 30°） | 需要 IMU（无 IMU 时退回纯先验、跳过互检：`v8_hybrid_bad` 就是这么发散的） |

**推荐 = hybrid + 保留 200 帧门**（本仓）。仿真里 Δ=0.011° ⇒ 走 URDF、0.58°；
故意把轴写错 Δ=42.2° ⇒ ERROR + 回退 IMU、0.65°。真机"倾角已知到 ±1°"落在 Δ<2° 的 INFO 带（用先验），
若 URDF 与实际差 3°（换支架）则落进 2–5° 的 WARN 带（改用 IMU）⇒ 正好覆盖两种现实。

**(6) 200 帧门是**独立**的第二件事（本实验最意外的发现）**：`fix_gravity_direction=false` 会把初始化提前到
**第一帧点云**（≈0.1–1 s），正好落在 Gazebo 出生时的**掉落/落定窗口** ⇒ 8 个 `fix=false` 变体里
**5 个发散**（v1、v2、v5b、v5c、v8）；另 2 个（v6、v7）没发散但**云被拖花**（平面 p95 25–26 mm
vs 稳定档 4–8 mm、原始云残差 3.0–3.1°、与 TF rpy 自相矛盾）；唯一稳的是把 init 推迟到落定之后的
`v2b`（`init_map_size 10 → 120000`）⇒ **稳定、不发散**（30.67°）。
⇒ 早期初始化的问题与重力值无关，是"静态初始化"这个前提被破坏。
**落地含义**：`fix_gravity_direction=false` 这条"省掉 2 s"的路在本仓仿真里**不可用**；E1 应当挂在
"机器人已经静止"这个门上（仿真=200 帧 IMU 均值；真机建议换成显式的静止检测，见 §9.6）。

### 9.5 补丁、代价、回滚

补丁（`.tmp_gravtest/experiment.patch`，`git diff --stat` = **3 文件 / +211 −5 行**）：
`parameters.h`（+9）、`parameters.cpp`（+14）、`small_point_lio.cpp`（+193，其中 ~120 行是注释/日志/回退分支）。
新参数三个，**全部带默认值**（现有 YAML 一个字节都不用改）：
```yaml
gravity_align_mode: "off"      # off | urdf | imu | hybrid   （默认 off = 原行为）
gravity_align_rpy:  [-30.0, 0.0, 0.0]   # URDF 先验：车体水平时雷达在世界里的 rpy(度)
gravity_init:       [0.0, 4.893, -8.481]  # 可选：body 系重力向下（免得等 200 帧；= 静止时加速度均值的反向）
```
核心数学只有 12 行（其余是 fallback/日志）：
```cpp
// down_body: (a) gravity_init 参数 / (b) fix_gravity_direction 的 200 帧均值 / (c) IMU deque 均值
const Eigen::Vector3d axis = down_body.cross(down_world);       // down_world = parameters.gravity
const double s = axis.norm(), c = down_body.dot(down_world);
Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
if (s > 1e-12) R = exp<double>(axis / s * std::atan2(s, c));    // 最小旋转：body 重力 → 世界 −z
estimator.kf.x.rotation = R;                                    // ★ 缺的就是这一行
estimator.kf.x.gravity = R * down_body;                          // ≈ (0,0,−|g|)
estimator.kf.x.acceleration = -(R.transpose() * estimator.kf.x.gravity);   // body 系比力
```
**外加 3 行**（`gravity_aligned` 时）把初始地图按出点公式放进世界系（`R·(R_L_I·p+T_L_I)+t`），
否则初始地图被整体转过 R（§9.1 最后一条）。

*代价*：RSS **33–35 MB 不变**；RTF 0.40–0.41 与基线同量级（静止、无导航栈，未量导航下的差别）；
初始化多等 200 帧（真机 2.0 s；本仿真墙钟 ~4 s @RTF 0.4）。
*风险*：`urdf`/`hybrid` 的先验若与实物不符，会**按错的角度把 odom 摆正**（urdf 档无告警，hybrid 有 Δ 门槛）。
*默认路径不变*：`v0_base`（已打补丁、`mode=off`）30.110°/30.057° vs `smoke0`（未打补丁）30.129°/30.053°，
TF rpy [30.03,−0.09,−0.13] vs [30.05,−0.08,−0.14] ⇒ 差在跑间噪声内。
*回滚*：`git checkout -- src/rm_localization/small_point_lio/src/small_point_lio/{parameters.h,parameters.cpp,small_point_lio.cpp}`
（或反向应用 `.tmp_gravtest/experiment.patch`）；不删任何槽位、不动契约（五个关键话题发布者数仍各 = 1）。

### 9.6 没能验证 / 诚实边界

1. **0.5° 的拆分**：`v6c`（同档、settle 45 s）与 `v6b`（15 s）只构成**两个时间点**的对照
   ⇒ 能排除"15→45 s 之间持续长大"，但**不能**把 0.47–0.58° 进一步拆成"init 后立刻就有"与"前几秒建立起来"两份
   （没有逐帧姿态序列）。
2. **早期初始化发散的确切机理**：只证明了与"初始化落在出生掉落窗口"的相关性（4/6 发散、把同一个
   `fix=false` 的 init 推迟就稳定），**没有**做逐帧归因（哪一帧开始偏、是 IMU 更新踢的还是点更新拉的）。
3. **真机**：全部数据来自仿真（gazebo、静止、无噪声 IMU、URDF 精确）。真机上"倾角已知到 ±1°"的
   先验误差、IMU 零偏/噪声、起步就在动，都会改变 §9.4 的表；`hybrid` 的 Δ 门槛（2°/5°）在真机上
   应当按你们的安装公差重定（±1° ⇒ 2° 门槛偏松还是偏紧，需实测定）。
4. **`/scan` 侧的压缩**：本实验的 `robot11_mount:=sensor` 档把 linefit/p2l 都放在 base_link 里，
   所以 odom 斜不斜 **不改** `/scan` 的波束集合（实测差 <2%）。"odom 斜 ⇒ 丢掉哪些波束"只在
   默认/`urdf` 档成立（§I 的 755 vs 948），本轮**没有**重测那两档。
5. **pb2025 的真机/上游代码**：本节只判"在我们 vendored 的 small_point_lio 里，这种参数式做法成立吗"（不成立）；
   他们仓库里的 `gravity` 到底是"世界系目标"还是"body 系实测值"，仍以
   `docs/gravity_alignment_ownership_survey.md` §7.3 的"未验证"为准 —— 我们的实验能解释**为什么**
   两种读法在他们那儿都能自洽（有 Set_init 时，参数=实测值 ⇒ rot_init≈I ⇒ 世界不转也是"自洽"的，
   但 odom 就是斜的；参数=世界目标 ⇒ rot_init 真的把世界转正）。
