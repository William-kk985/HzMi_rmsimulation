# 外参到底该在哪一步乘一次：FAST-LIO / Point-LIO / LIO-SAM / Livox 驱动 源码调研 + 本仓 `small_point_lio` 两处变换的判决（2026-10-18）

> **本文要定论的问题**（用户原话）："对『已配准的点』又乘了一次外参这个上网查一下吧，这个外参是不是来矫正倾斜点云的，对齐到重力系"。
>
> 具体四问：① LiDAR→IMU/body 外参在 LIO 里到底干什么、在哪一步乘；② 有没有任何实现会把外参**乘到已经配准到 odom/world 的点上**；③ 社区认为的正确 pattern 是什么（点云发哪个帧、谁发 `odom→base_link`、外参乘几次）；④ "重力对齐" 是不是外参的活。
>
> **方法**：`curl` 取上游**逐字源码**（FAST-LIO `main`、Point-LIO `master`、LIO-SAM `master`、`livox_ros_driver2` `master`）+ GitHub issue / REP / nav2 文档。**证据分级**：`［引用］`= 外部 URL 原文；`［实测］`= 本仓已落盘的测量（指向 `docs/tilted_lidar_fidelity.md` 的节号，不重跑）；`［推断］`= 我由前两者推的结论。
>
> **不做**：不改代码/配置/launch，不跑仿真。本文只回答"这是不是 bug / 该怎么写"。

---

## 0. 一句话结论（先给判决）

1. **外参（LiDAR→IMU/body）的唯一职责 = 把"传感器系里的量测"搬到"算法 body 系"，且必须在流水线里只乘一次**；它**不是**重力对齐工具，也**不是**安装位姿的运行时修正量。FAST-LIO 的 README 把它定义成"LiDAR 在 IMU body 系里的位姿"，代码在 `pointBodyToWorld` 里乘**恰好一次**。［引用 C1/C3］
2. **找不到任何 LIO 实现会对"已经配准到 odom/world 的点"再乘一次 `T(base←lidar)`**。FAST-LIO / Point-LIO / LIO-SAM 三条线在这件事上完全一致；社区维护者在 FAST-LIO #433 里把这种写法直接点名成 bug 形状："an extra TF can effectively rotate the frame twice"。⇒ **本仓 bug ② 是 bug，不是"约定差异"**。［引用 C1/C2′/C11/C6］
3. **`odom→base_link` 的正确写法是合成 `T_ol·T_bl`（右乘），共轭 `T_bl⁻¹·T_ol·T_bl` 是"同一变换换基"，只在 `T_bl` 为纯平移时姿态碰巧对**。LIO-SAM 用的是**同样方向的 `lookupTransform(lidarFrame, baselinkFrame)`**，写的是 `tCur = tCur * lidar2Baselink` —— 与我们的修法逐字同构，与共轭相反。⇒ **bug ③ 是 bug**。［引用 C10］
4. **倾斜挂装导致"odom 自己斜"是 FAST-LIO 系的常规约定，不是 bug**：世界系取"初始化那一刻的 IMU 系"，重力只作为**状态向量**存着，初始姿态**不**被重力纠正（`init_state.grav = ...`，而 `rot` 那行被注释掉）。FAST-LIO 维护者的建议是"初始化时做一次重力对齐，把这个矩阵在**发布点云时**用一次"。⇒ 我们 `small_point_lio` 的行为（`rotation = Identity()` + 把均值加速度写进 `gravity`）**与上游同构，属正常约定**；要重力对齐的 `odom`，得**另加一步**，而不是怪外参。［引用 C2/C5］

---

## 1. 外参的角色：把传感器系的量测搬到 body 系，且只乘一次

### 1.1 定义（逐字）

FAST-LIO README 的配置说明：

> "The extrinsic parameters in FAST-LIO is defined as the **LiDAR's pose (position and rotation matrix) in IMU body frame** (i.e. the IMU is the base frame)."
> — <https://github.com/hku-mars/FAST_LIO/blob/main/README.md>［引用 C3］

即 `T_IL = T(IMU ← LiDAR)`：**子帧是 LiDAR，父帧是 IMU body**。它的语义是"刚体安装关系"，而不是"姿态修正量"。同一份 README 给的 MID-360 出厂配置是 `extrinsic_R = I`、`extrinsic_T = [-0.011, -0.02329, 0.04412]`（同壳体、只有杆臂）—— <https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml>［引用 C4］。

> ⚠️ 这条事实是**本仓 bug ②③ 在默认档被完全掩盖**的根源：`extrinsic_R = I` 让人以为"外参已经配好了"，而真正的自变量是另一条 TF（`base_link→livox_frame`）。

### 1.2 三个必须分开的"外参"（这是"斜装"问题的关键）

| # | 变换 | 谁提供 | 谁用 | 语义 |
|---|---|---|---|---|
| (i) | **LiDAR→IMU** `T_IL` | LIO 参数文件（`extrinsic_R/T`） | 估计器内部，**乘一次** | 出厂/标定的壳体关系 |
| (ii) | **IMU→body** `T_body←imu` | URDF / `robot_state_publisher` | 估计器之外（TF 消费者） | 车体定义 |
| (iii) | **LiDAR→body** `T_bl = T(base_link←lidar_frame)` | URDF 关节 / 用户静态 TF | **谁都不该在"已配准点"上用它**，只在"帧换算"用它 | 挂装关系 |

本仓已把这张表按真机 MID-360 逐项对过账（含"内置 IMU 不免除 (iii)"的论证）：`docs/tilted_lidar_fidelity.md` §N.3.1。［实测 B1］

**`［推断］`** 用户问的"这个外参是不是来矫正倾斜点云的"—— 答案是**不是**：能矫正"倾斜"的量在 (i) 里只以"标定值"的身份存在，(iii) 里以"安装关系"的身份存在；两者都不能承担"把点云摆正到重力系"的职责（见 §4.4）。

### 1.3 四个代表系统的流水线位置（逐字）

#### (a) FAST-LIO（`main`）—— 外参在 `body→world` 映射内部乘一次

```cpp
// src/laserMapping.cpp:178-187
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    ...
}
```
<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L178-L187>［引用 C1］

读法：`p_body` 是**LiDAR 系**的点（变量名叫 body，历史包袱），`offset_R_L_I/offset_T_L_I` 就是 `T_IL`；**一次**映射 `p_world = R_wl·(R_IL·p_l + t_IL) + t_wl` 完成"LiDAR→IMU→world"两级链。发布处：

```cpp
// src/laserMapping.cpp:478-499（节选）
PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
...
RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
...
laserCloudmsg.header.frame_id = "camera_init";     // :496
```
<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L478-L499>［引用 C1］

另外它还提供**sensor 系版本**（配置项 `scan_bodyframe_pub_en: true`，注释 "output the point cloud scans in IMU-body-frame"）：

```cpp
// src/laserMapping.cpp:532-548（节选）
RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);  // 只乘 T_IL
laserCloudmsg.header.frame_id = "body";            // :546
```
［引用 C1/C4］

TF：**只发自己拥有的两个帧**，`camera_init→body`，且直接用 LIO 状态，**没有任何 `lookupTransform`**：

```cpp
// src/laserMapping.cpp:589-620（节选）
odomAftMapped.header.frame_id = "camera_init";     // :591
odomAftMapped.child_frame_id = "body";             // :592
...
br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );  // :619
```
<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L589-L620>［引用 C1］
全文件 `grep base_link` = **0 次**［实测·本次 `curl` 后 grep］——FAST-LIO **不认 `base_link` 这个概念**。

#### (b) Point-LIO（`master`）—— 同构，外参也是乘一次

```cpp
// src/laserMapping.cpp:125-149（pointBodyLidarToIMU）
p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;   // :133
```
```cpp
// src/laserMapping.cpp:974-988（地图更新里先把点搬到 IMU 系，再做 body→world）
point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;     // :983
pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));      // :950
```
```cpp
// src/laserMapping.cpp:574-598：publish_frame_world 直接发"已经算好的 world 点"
laserCloudWorld->points[i].x = feats_down_world->points[i].x;   // 不再做任何变换
laserCloudmsg.header.frame_id = "camera_init";                 // :597
```
<https://github.com/hku-mars/Point-LIO/blob/master/src/laserMapping.cpp>［引用 C2］
TF 同样是自己的帧：`:681-709` `camera_init→body`（`:708` `sendTransform`），全文件 `base_link` = 0 次。［引用 C2/实测 grep］

#### (c) LIO-SAM（`master`）—— **反方向**：外参乘在 IMU 上，点云从不用外参

```cpp
// include/utility.h:252-271（imuConverter，IMU 回调的入口）
acc  = extRot * acc;                    // :257
gyr  = extRot * gyr;                    // :263
Eigen::Quaterniond q_final = q_from * extQRPY;   // :269
```
<https://github.com/TixiaoShan/LIO-SAM/blob/master/include/utility.h#L252-L271>［引用 C12］
README 把设计意图写明了：

> "**IMU alignment**. LIO-SAM transforms IMU raw data from the IMU frame to the Lidar frame, which follows the ROS REP-105 convention (x - forward, y - left, z - upward)."
> — <https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md>［引用 C13］

即：LIO-SAM 的 odom 定义在 **LiDAR 系**，外参在 **IMU 边界**乘一次（把 IMU 搬进 LiDAR 系），点云一次都不乘。发布处只乘 `odom←lidar` **位姿**（一次、且标签一致）：

```cpp
// src/mapOptmization.cpp:1721-1729（pubCloudRegisteredRaw）
*cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);      // 只乘 T_odom←lidar
publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
```
<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/mapOptmization.cpp#L1721-L1729>［引用 C11］
其余点云：deskewed 发 `lidarFrame`（`src/imageProjection.cpp:603`）；局部地图/关键帧发 `odometryFrame`（`src/mapOptmization.cpp:489`）。［引用 C11/C14］

#### (d) livox_ros_driver2 —— 驱动**不乘任何外参**，只贴一个可配的 `frame_id`

```cpp
// src/lddc.cpp
cloud.header.frame_id.assign(frame_id_);        // :263, :355, :421
imu_msg.header.frame_id = "livox_frame";        // :481
```
<https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp>［引用 C15］
⇒ 驱动吐出来的点就是**传感器系**的原始点，`frame_id` 只是标签。这也解释了 FAST-LIO #284 里那位用户的坑：他把安装倾角写进 `MID360_config.json` 的 `extrinsic_parameter`，结果**只有点云被转、IMU 没转**，定位直接漂掉。［引用 C5］

### 1.4 小结（回答第 1 问）

- 外参的职责 = **坐标搬家**（sensor → body），乘一次，位置在"量测进入算法 body 系"的那一步；三类实现的落点不同（FAST-LIO/Point-LIO 在出点公式里、LIO-SAM 在 IMU 回调里、驱动里不乘）。
- **它不是**"把倾斜点云摆正"的旋钮：三条实现里没有任何一条把外参当姿态修正用；能改"倾斜"的是**初始姿态 / 重力对齐**（§4.4）。

---

## 2. 对照表

| 系统 | 发布的点云帧（topic） | 发布的 TF（parent→child） | 外参乘几次 / 在哪 | 倾斜挂装 & 重力怎么处理 |
|---|---|---|---|---|
| **FAST-LIO** `main` | `/cloud_registered` → `camera_init`（`:496`）；可选 `/cloud_registered_body` → `body`（`:546`） | `camera_init→body`（`:619`），**只发自己的帧，无 `base_link`** | **1 次**，`pointBodyToWorld`（`:178-187`）；可在线估计（`extrinsic_est_en`） | 世界系 = 初始化那刻的 IMU 系；重力存成状态 `grav`（`IMU_Processing.hpp:196`），**初始姿态不纠偏**（`:198` 的纠偏行被注释掉）［C1/C2 引用］ |
| **Point-LIO** `master` | 同款：`camera_init`（`:597`）；另有 `body` 系云（`:649`） | `camera_init→body`（`:708`），无 `base_link` | **1 次**（`:133`/`:983`） | 同上；gravity 为 `vect3` 在线估计（维护者在 #80 里说明"在线重力估计不会明显改变重力"）［C2/C8 引用］ |
| **LIO-SAM** `master` | deskewed → `lidarFrame`（`imageProjection.cpp:603`）；局部地图/关键帧 → `odometryFrame`（`mapOptmization.cpp:489`）；`cloud_registered_raw` → `odometryFrame`（`:1729`） | 估计器发 `odom→lidar_link`（`:1650`）；**另一个节点** `transformFusion` 发 `odom→base_link`（`:128-130`） | **1 次**，且在 **IMU 边界**（`utility.h:257/263/269`）——点云侧 0 次 | 用 **9 轴 IMU** 的 roll/pitch **初始化姿态**（README："roll and pitch estimation is mainly used to initialize the system at the correct attitude"）⇒ odom 天生重力对齐［C10/C12/C13 引用］ |
| **livox_ros_driver2** | `/livox/lidar`，`frame_id` 可配（`:263`） | **不发 TF** | **0 次**（只贴标签） | 不管；`extrinsic_parameter` 只打点云不打 IMU（#284 的坑）［C15/C5 引用］ |
| **small_point_lio 上游**（`Yancey2023/small_point_lio@688d75c`，= 本仓 `third_party/cod_nav_2026` 副本） | `/cloud_registered` → 声称 `odom`（上游 `:127`），但内容是 `T(base←lidar)·p_odom`（上游 `:159`） | `odom→base_link`（`:61-62`），姿态 = **共轭** `T_bl⁻¹·T_ol·T_bl`（`:75`） | **2 次**（算法内 1 次 + 发布时又 1 次） | 初始 `rotation = Identity()` + 均值加速度写进 `gravity` ⇒ **odom ≡ 初始传感器壳体系**（本仓 `eskf.h:30`、`small_point_lio.cpp:48-64`）［引用 C21·本仓 vendored 逐字］ |
| **small_point_lio 本仓修后**（`2a5a8b6` = 当前 HEAD） | `/cloud_registered` → `odom`，内容 = `p_odom`（不再变换，`node.cpp:227-229`） | `odom→base_link`：姿态 = **合成** `T_ol·T_bl`（`:137`），平移**故意保留旧值**（`:160-162`，理由见 §J.5） | **1 次**（算法内） | 同上（未改）——倾斜 odom 属上游约定，见 §4.4［实测/本仓源码］ |
| **本仓 `lio_tf_adapter`** | 不碰点云 | 订阅 `/Odometry`，发 `odom→base_link`：`q_out = q_lio*q_offset`、`t = t_lio + quatRotate(q_lio, t_offset)`（`:63-68`）= **合成** | 0 次（不碰点） | 不涉及；这是本仓**已经写对**的那条路［实测/本仓源码］ |

---

## 3. 对本仓两处代码的判决

### 3.0 口径先说清（避免"你骂的是谁"）

- **当前 HEAD 已经修了**：`2a5a8b6 fix(small_point_lio): 修 bug②「点云多乘一次 T(base←livox)」+ bug③「odom→base 姿态用共轭」`（`git log -1 2a5a8b6`）。所以本文判的是**这两个模式**，不是"今天的代码还错着"。
- **上游代码原样未改**：`third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio_node.cpp:75`（共轭）、`:107-124 + :159`（发布时再乘一次 `T(base←lidar)`），与 `Yancey2023/small_point_lio@688d75c` 一致［实测·本仓副本逐字］。
- 测量数字来自本仓已落盘的 A/B：`docs/tilted_lidar_fidelity.md` §I.5 / §J.1 / §J.2（`dot` 一次没重跑）。［实测 B1］

### 3.1 bug ②：对"已经在 odom 的点"再乘一次 `T(base←lidar)`

**代码事实（上游/副本）**：算法出点公式已经是 odom 点 ——
`p_odom = R_ol·(R_ext·p_lidar + t_ext) + t_ol`（本仓 `small_point_lio.cpp:98`：`pointcloud_odom_frame.emplace_back((kf.x.rotation * dense_point_imu_frame + kf.x.position))`）［实测·本仓源码］；而发布回调又做了一次 `p_pub = R(base←lidar)·p_odom + t`（上游 `:159`），并盖 `frame_id="odom"`（上游 `:127`）。

**为什么判成 bug（三条独立证据）**：

1. **消息契约**：`sensor_msgs/PointCloud2` 的 header 定义就是"3D 点的坐标帧" ——
   `# Time of sensor data acquisition, and the coordinate frame ID (for 3d points).`（<https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg>）［引用 C16］。
   点乘了 `T(base←lidar)` 之后**不再是 odom 系的点**，标签就成了假话。
2. **上游三条实现都没有这个动作**：FAST-LIO/Point-LIO 在"出点公式里乘一次、发布时直接发"（§1.3a/b），LIO-SAM 在"IMU 边界乘一次、发布时只乘 `odom←lidar` 位姿"（§1.3c）。**没有任何实现会在发布点云时再乘一次 `T(base←sensor)`**。［引用 C1/C2′/C11］
3. **社区维护者把它点名成 bug 形状**：FAST-LIO #433 的回复逐字——"**TF duplication**: Make sure you are not applying an additional rotation via TF (e.g., a static transform between os_sensor, os_imu, or base_link). **FAST-LIO already applies the extrinsic_R/T internally, so an extra TF can effectively rotate the frame twice.**"（<https://github.com/hku-mars/FAST_LIO/issues/433>）［引用 C6］

**"外参到底是不是来矫正倾斜点云的"** —— #433 同一段还给了反面答案："FAST-LIO **does not perform automatic horizontal leveling**. It assumes the sensor frame is already consistent with the gravity direction from the IMU. If the LiDAR is mounted with a tilt (roll/pitch), **the estimator will build the map from that orientation**."［引用 C6］⇒ 外参不矫正倾斜；倾斜照原样进地图。

**后果（本仓实测）**：
- `T_bl = I` 时：只把整朵云平移 `|t| = 0.2044 m`，肉眼与单帧统计都看不出（`t = (0.000562, 0.130916, 0.157028)`）。
- `R_bl = R_x(−30°)` 时：**整个 odom 场景被刚性旋转 30.970°**（地面倾角），按同一条 TF 反变换回去只剩 1.041°；RViz 里就是"平放扫到的东西被倾斜了"。修后 1.054°。［实测 B1 §I.5.1/§J.1］
- 下游如何"吃"这个错：nav2 的 `ObservationBuffer::bufferCloud` 会**按 `header.frame_id` 通过 TF 把云搬到 global_frame**，然后才做 `min/max_obstacle_height` 判据（`// transform the point cloud` → `tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, ...)`，<https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/src/observation_buffer.cpp#L79-L125>）［引用 C19］。
  ⇒ 标签错 ⇒ nav2 会**再转一次**才进代价图，且高度带是在**它以为的 odom** 里量的 —— 这正是我们 §J.5 里"换正确平移后 scan 盘面 z 掉到 −0.10 m，950/950 落到高度带外、代价图 lethal 471→0"的机理。［实测 B1 §J.5］

### 3.2 bug ③：`odom→base_link` 用**共轭**而不是**合成**

**变量语义先对齐**：上游 `:75` 的 `tf_base_link_to_lidar_frame = tf2::fromMsg(lookupTransform(lidar_frame, "base_link"))` = `T(livox_frame ← base_link)` = **`base_link` 在 `livox_frame` 里的位姿**（变量名就是这个意思）。上游写的是

```cpp
tf2::Transform tf_odom_to_base_link =
        tf_base_link_to_lidar_frame.inverse() * tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame;   // 共轭 = T_bl⁻¹·T_ol·T_bl
```
（`third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio_node.cpp:75`）［实测·本仓副本逐字］

**正确性只能由坐标映射链式法则推**（`［推断］`，与 §1.3 的三条实现一致）：
`p_odom = T_ol · p_livox = T_ol · (T_bl · p_base)` ⇒ **`T_odom←base = T_ol · T_bl`（右乘合成）**。
而 `T_bl⁻¹·T_ol·T_bl` 是**相似变换/换基**：它回答的是"同一个线性映射在另一组基下的矩阵"，不是"从 odom 到 base 的位姿"。两者只在 `T_bl` 的旋转为 `I`（纯平移）时**姿态**碰巧相同，平移还差 `−p`。

**最硬的外部对照：LIO-SAM 用同样方向的 lookup，写的是合成**

```cpp
// src/imuPreintegration.cpp:46-51（TransformFusion 构造）
if(lidarFrame != baselinkFrame)
{
    tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
    tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);   // :51  同方向：T(lidar←base)
}
...
// src/imuPreintegration.cpp:124-130（发布 odom→base_link）
static tf::TransformBroadcaster tfOdom2BaseLink;
tf::Transform tCur;
tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
if(lidarFrame != baselinkFrame)
    tCur = tCur * lidar2Baselink;                       // :128  右乘 = 合成 T_ol·T_bl
tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
tfOdom2BaseLink.sendTransform(odom_2_baselink);          // :130
```
<https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L46-L51> / <...#L124-L130>［引用 C10］

**同构到我们这儿**：`lidar2Baselink` ≡ 我们的 `T_bl`（同一次 `lookupTransform`、同方向、同变量含义），LIO-SAM 写 `tCur * lidar2Baselink`，我们上游写 `inverse() * tCur * lidar2Baselink`。**差的就是那一个 `inverse()` 和乘法位置** ⇒ 这不是风格差异，是数学错误。

**后果（本仓实测）**：静止、`R_bl = R_x(+30°)`（`robot11_mount:=urdf`）时，发布的 `odom→base_link` rpy = `[0.383, 4.890, 7.701]°`，而真值 `/odom_ground_truth` 的 pitch 只有 0.003°（窗口内跨度 0.003° vs 4.924°），**pitch 随 yaw 一起长大**；正演 `R_bl·R_ol·R_blᵀ` 与实测 rpy 逐位相同 ⇒ 机理钉死。修后同口径 0.317°。［实测 B1 §I.5.3/§J.1.2］

### 3.3 可观测性：identity 外参下**看不见**（这就是它活了这么久的原因）

| 量 | `T_bl = I`（默认档 `plugin`；上游 launch 的单位静态 TF） | `R_bl = R_x(−30°)`（`urdf\|sensor` 档；真机斜装） |
|---|---|---|
| bug ② `/cloud_registered` | 只平移 `−t`（0.2044 m），**帧内几何不变**：地面 0.742° | **整朵 odom 场景刚性旋转 30.970°**（反算 1.041°，修后 1.054°） |
| bug ③ `odom→base_link` / `/odometry` | 姿态与共轭**逐位相同**（纯平移时 `R_bl·R_ol·R_blᵀ = R_ol`），只有平移差 `−p` | **假俯仰 4.890°**（真值 0.003°），`pitch` 随 `yaw` 增长 |

来源：`docs/tilted_lidar_fidelity.md` §I.5.2/§I.5.3/§J.1.2/§J.2（本仓 A/B 实测）。［实测 B1］

**`［推断］`** 上游之所以"这么多人在用也没炸"，是三件事叠加：① 上游 `launch/small_point_lio.launch.py` 自己发了一条**单位**静态 TF `base_link→livox_frame`（`--x/y/z 0 --roll/pitch/yaw 0`），把 `R_bl` 钉成 `I`；② `/cloud_registered` 只在有订阅者时才发（`get_subscription_count() > 0`）；③ 多数人只把 LIO 当里程计源、在 `camera_init/odom` 里看车，不看 `base_link` 的姿态。

### 3.4 有没有"反例"：任何实现会变换"已配准的点"吗？

有，但**形式完全不同**，正好反证我们的写法错在哪：

| 谁 | 变换的对象 | 乘的是什么 | 发布帧 | 为什么合法 |
|---|---|---|---|---|
| FAST-LIO `publish_frame_world` | **body/lidar 系**的点 | `T_world←body`（含一次 `T_IL`） | `camera_init` | 乘的是"到**标签所指帧**"的完整位姿，一次 |
| LIO-SAM `pubCloudRegisteredRaw` | **lidar 系**的 deskewed 点 | `T_odom←lidar`（`transformTobeMapped`） | `odometryFrame` | 同上 |
| 本仓 bug ② | **已经是 odom 系**的点 | `T(base_link←livox_frame)`（**不含任何 odom 信息**） | `odom` | ❌ 既重复、又乘错了量，标签还写 odom |

⇒ 判据可以写成一句话：**"点云上乘的矩阵，必须正好等于 `T_{标签帧 ← 点当前所在帧}`"**。bug ② 违反的是这条恒等式，不是"某处写法不合口味"。

### 3.5 判决表

| 项 | 判决 | 依据 |
|---|---|---|
| bug ② 对已配准点再乘 `T(base←lidar)` | **是 bug**（帧语义错误：标签 `odom` ≠ 内容） | 消息契约［C16］+ 三条实现对照［C1/C2′/C11］+ 维护者原话"rotate the frame twice"［C6］+ 本仓 30.970° 实测［B1］ |
| bug ③ 共轭 `T_bl⁻¹·T_ol·T_bl` | **是 bug**（数学错误：换基 ≠ 位姿合成） | 链式法则 + LIO-SAM 同方向 lookup 的合成写法［C10］+ 本仓 4.890° 假俯仰［B1］ |
| "倾斜挂装 ⇒ odom 自己斜" | **不是 bug，是 FAST-LIO 系约定** | `init_state.grav` + 纠偏行被注释［C1］+ 维护者建议"初始化时做重力对齐"［C5］ |
| `extrinsic_R = I`（内置 IMU）能否免除 ②③ | **不能** | 自变量是 `base_link↔livox_frame` 这条 TF，与 `T_IL` 无关［B1 §N.3.1］ |

---

## 4. 我们这种 bench 的推荐 pattern（还要给 nav2 一个重力对齐的 `base_link`）

### 4.1 五条原则（每条都有上面的出处）

1. **外参只在"量测进入 body 系"时乘一次**，之后任何环节都不再碰它（FAST-LIO/Point-LIO 的出点公式、LIO-SAM 的 `imuConverter` 都是这个形状）。［C1/C2′/C12］
2. **点云上乘的矩阵 = `T_{header.frame_id ← 点当前所在帧}`**，不多不少（§3.4 的恒等式）。［C16］
3. **估计器只发它拥有的帧**（`odom→传感器/IMU body`），**不发 `base_link`**——FAST-LIO/Point-LIO 全仓 `base_link` 出现 0 次；需要 `base_link` 时用一个**独立适配节点**（LIO-SAM 的 `transformFusion`、本仓的 `lio_tf_adapter`）。［C1/C2′/C10/本仓源码］
4. **`odom→base_link` 永远是合成 `T_ol·T_lb`**（右乘）；共轭只用于"同一算子的换基"，不用于位姿链。［C10］
5. **外参不承担重力对齐**；重力对齐是"初始姿态"或"额外的对齐变换"的事。［C5/C6/C2］

### 4.2 点云发哪个帧：(a) sensor 帧 vs (b) odom 帧

| | (a) **发 sensor 帧**（`livox_frame`，让 TF+消费者去转） | (b) **发 odom 帧**（本仓修后现状） |
|---|---|---|
| 先例 | FAST-LIO 的 `body` 云（`scan_bodyframe_pub_en`，`:546`）；LIO-SAM 的 deskewed 云（`lidarFrame`，`:603`） | FAST-LIO `camera_init`（`:496`）；Point-LIO `camera_init`（`:597`）；LIO-SAM `cloud_registered_raw`（`:1729`） |
| 消费者负担 | nav2/octomap 自己 `tf2_buffer_.transform(...)`（nav2 已内建，见 `observation_buffer.cpp`）［C19］ | 零转换，直接进代价图 |
| 对"帧标签写错"的抗性 | **强**：标签是传感器帧，不可能混进 `base_link` 的东西 | **弱**：一旦有人在这里再乘一次外参，就会像 bug ② 一样"看起来仍自洽" |
| 对高度带（`min/max_obstacle_height`）的影响 | 高度带是在 **global_frame** 里判的（nav2 先转再判）⇒ 与发布帧无关［C19］ | 同左 |
| 我们的建议 | **可以作为"守卫"形态保留**（多一条 topic 或 RViz 用），不替代 (b) | **主路径保持 (b)**：改动最小、`/scan`/p2l/linefit 的既有链路不变 |

**结论**：两种都是合法约定，**只要标签诚实**；本仓现状（发 `odom` 且内容是 `p_odom`）符合主流做法，**不需要为了"对齐上游"改成 sensor 帧**。［推断，依据 C1/C11/C16/C19］

### 4.3 `odom→base_link` 谁负责

| 方案 | 谁发 | 先例 | 代价 |
|---|---|---|---|
| A. 估计器自己发（现状，`small_point_lio`） | LIO 节点内 `lookupTransform` + 合成 | **上游没有先例**（上游是共轭，见 §3.2）；LIO-SAM 的估计器只发 `odom→lidar_link` | 让 LIO 依赖一条它**不拥有**的 TF：丢 TF ⇒ 连 odom 都不发（本仓已登记为强依赖）［B1 §N.3.2］ |
| B. 独立适配节点（**推荐**） | `lio_tf_adapter`（已有，`q_out=q_lio*q_offset`、`t=t_lio+quatRotate(q_lio,t_offset)`） | LIO-SAM `transformFusion`（`:128`）、本仓 `lio_tf_adapter` | LIO 只管发 `odom→传感器/IMU body`；`base_link` 的杆臂/倾角**只有一个真值来源**（launch/URDF 的参数），不会两处各算一遍 |

> 注意本仓现状的一个"历史包袱"：`lio:=small_point_lio` 时 `lio_tf_adapter` 被**显式排除**（因为该节点"自己就发 `odom→base_link`"），于是**本仓唯一写对合成的那条路恰好没被用上**。［实测 B1 §N.2］⇒ 若走 B，必须先让 `small_point_lio` 只发 `odom→livox_frame`（它的自然输出），再由 adapter 合成。**这是设计建议，本次不改代码。**

### 4.4 重力对齐放哪儿（第 4 问的落点）

**先确认"我们的现象是常规"**：`small_point_lio` 初始化时把前 200 帧加速度均值写进 `gravity` 状态，`rotation` 保持 `Identity()`（`small_point_lio.cpp:48-64`、`eskf.h:30`）⇒ **`odom` 的定义就是"初始化那一刻的传感器壳体系"**，斜装必然给出斜 odom。［实测·本仓源码］
**这与 FAST-LIO 逐字同构**：`init_state.grav = S2(-mean_acc/mean_acc.norm()*G_m_s2)`（`IMU_Processing.hpp:196`），而"用 `Exp(mean_acc × (0,0,-1))` 把初始姿态转正"那行是**注释掉的**（`:198`：`//state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));`）［引用 C1］。
**维护者的建议也正是"另加一步"**：

> "As we build the map in the IMU frame, I would suggest conducting a **gravity alignment during the initialization step**. Read the IMU to identify the gravity direction, and record a transformation matrix to translate your IMU to the gravity frame. After that, you only need to **transform the point cloud via that matrix when publish the point clouds**. You'll obtain the point cloud in the gravity frame by doing so."
> — FAST-LIO 维护者 `Ecstasy-EC`，<https://github.com/hku-mars/FAST_LIO/issues/284>［引用 C5］

对照 LIO-SAM 的路线（另一条合法解）：**用 9 轴 IMU 的 roll/pitch 在初始化时定姿态**，"The roll and pitch estimation is mainly used to **initialize the system at the correct attitude**"（README）［引用 C13］⇒ 它的 `odom` 天生重力对齐，所以 `transformFusion` 只需要合成、不需要额外纠偏。

**三条可选路线（按"要不要动 LIO"排序）**：

| 路线 | 做法 | 优点 | 代价 / 风险 |
|---|---|---|---|
| **R1 初始化时定姿态**（最干净） | 在 `small_point_lio` 初始化里，用均值重力方向解 `R_world←imu = Exp(π·ĝ)` 写进 `kf.x.rotation`（而不是只写 `gravity`） | 一次解决：`odom` 天生重力对齐，`odom→base_link` 的合成自动给出"水平车体"，物理倾角正确落进 `base_link→livox` | 要改估计器初始化（**本次禁改**）；需准静态起步的 N 帧；`gravity` 状态与 `rotation` 的语义要一起改，否则会"两次纠偏" |
| **R2 额外一条重力对齐 TF**（不动 LIO） | 由一个小节点在初始化窗口内估一次 `T(odom←odom_gravity)`，之后当**静态** TF 一直发；nav2 的 `global_frame` 用 `odom_gravity`，点云仍发 `odom`（nav2 自己转） | 零侵入；REP-105 允许"插入额外中间帧"（"it is fine to insert additional links in the graph"，<https://www.ros.org/reps/rep-0105.html>）［引用 C17］ | TF 树多一层（`map→odom→odom_gravity→base_link` 的顺序要理清，别插反）；代价图 `global_frame` 要跟着改；一次性估计必须**冻结**，否则 odom 不连续 |
| **R3 下游统一在重力对齐系里工作** | 感知/代价图都用 `base_link`（水平）当 `target_frame`（本仓 `robot11_mount:=sensor` 已这么做：linefit `gravity_aligned_frame: base_link`、p2l `target_frame: base_link`） | 不动 TF 树 | 代价图那层仍按 `odom` 的高度带判据 ⇒ §J.5 那个"整张图变空"的坑还在［B1 §J.5］ |

**推荐**：**R1（首选，作为下一轮的独立任务）→ 短期用 R2 兜底**。理由：`odom` 斜着本身**不违反 REP-105**（REP-105 只要求 `map` 的 Z 向上、"`odom` 连续无跳变"，<https://www.ros.org/reps/rep-0105.html>；REP-103 的 "x forward / y left / z up" 是**body 帧**的偏好朝向）［引用 C17/C18］，但**nav2 的实际默认**（本仓 `global_frame: odom`、`min_obstacle_height` 默认 0）等于把 `odom` 当水平系用 ⇒ 实际要求 `odom ≈ 重力对齐`。R1 让"约定"和"nav2 的假设"对齐；R2 是不动估计器时的最小妥协。

### 4.5 落地清单（只列，不改）

1. `small_point_lio`：把 `odom→base_link` 的发布**移出**估计器（改发 `odom→livox_frame`），由 `lio_tf_adapter` 合成 —— 消除"两处各算一遍"的可能（§4.3 B）。
2. 若保留在节点内：只允许合成 `T_ol·T_bl`；**禁止**在点云发布路径上 `lookupTransform`（bug ② 的形状，本仓 HEAD 已删并加了守卫 `check_pose_consistency()`）。
3. `odom` 的重力对齐走 R1（初始化）或 R2（额外静态 TF）；`base_link` 的倾角永远只在 `base_link→livox_frame` 这一条 TF 里出现，且与 URDF 同源。
4. 回归判据（沿用本仓已有口径）：`T_ob·T_bl⁻¹ ≈ T_ol`（位姿一致性）+ 逐点恒等式 `p_pub = T_{label←p_src}·p_src`；`R_bl = I` 与 `R_bl = R_x(−30°)` 两档都要跑，**只在后者能看见 ②③**。

---

## 5. 引用清单 + 打不通的东西

### 5.1 引用（编号 → URL → 用在哪）

| # | 来源 | URL | 用在哪 |
|---|---|---|---|
| C1 | FAST-LIO `src/laserMapping.cpp`（`main`） | <https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp#L178-L187> / <#L478-L499> / <#L589-L620> | §1.3a、§2、§3.1、§4.1 |
| C2 | FAST-LIO `src/IMU_Processing.hpp`（初始化） | <https://github.com/hku-mars/FAST_LIO/blob/main/src/IMU_Processing.hpp#L159-L201> | §0.4、§2、§4.4 |
| C2′ | Point-LIO `src/laserMapping.cpp`（`master`） | <https://github.com/hku-mars/Point-LIO/blob/master/src/laserMapping.cpp> | §1.3b、§2 |
| C3 | FAST-LIO `README.md`（外参定义） | <https://github.com/hku-mars/FAST_LIO/blob/main/README.md> | §1.1 |
| C4 | FAST-LIO `config/mid360.yaml`（`extrinsic_R=I`、`scan_bodyframe_pub_en`） | <https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml> | §1.1、§1.3a |
| C5 | FAST-LIO issue #284（倒装 MID360）**含维护者重力对齐建议** | <https://github.com/hku-mars/FAST_LIO/issues/284> | §1.3d、§2、§4.4 |
| C6 | FAST-LIO issue #433（"TF duplication … rotate the frame twice"、"does not perform automatic horizontal leveling"） | <https://github.com/hku-mars/FAST_LIO/issues/433> | §3.1、§3.5、§4.1 |
| C8 | Point-LIO issue #80（重力在线估计的中文讨论） | <https://github.com/hku-mars/Point-LIO/issues/80> | §2 |
| C9 | Point-LIO issue #115（倾斜安装的参数） | <https://github.com/hku-mars/Point-LIO/issues/115> | 背景（未作为判决依据） |
| C10 | LIO-SAM `src/imuPreintegration.cpp`（`TransformFusion`：`lookupTransform` + 合成 + 发 TF） | <https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp#L46-L51> / <#L124-L130> | §3.2、§4.1、§4.3 |
| C11 | LIO-SAM `src/mapOptmization.cpp`（`odom→lidar_link`、`cloud_registered_raw`） | <https://github.com/TixiaoShan/LIO-SAM/blob/master/src/mapOptmization.cpp#L1633-L1651> / <#L1721-L1729> | §1.3c、§2、§3.4 |
| C12 | LIO-SAM `include/utility.h`（`imuConverter`：外参乘在 IMU 上） | <https://github.com/TixiaoShan/LIO-SAM/blob/master/include/utility.h#L252-L271> | §1.3c、§2、§4.1 |
| C13 | LIO-SAM `README.md`（9 轴 IMU、roll/pitch 用于姿态初始化） | <https://github.com/TixiaoShan/LIO-SAM/blob/master/README.md> | §1.3c、§2、§4.4 |
| C14 | LIO-SAM `src/imageProjection.cpp`（点云发 `lidarFrame`） | <https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imageProjection.cpp#L603> | §1.3c、§2 |
| C15 | `livox_ros_driver2` `src/lddc.cpp`（只贴 `frame_id`，不乘外参） | <https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp> | §1.3d、§2 |
| C16 | `sensor_msgs/msg/PointCloud2.msg`（header = 3D 点的坐标帧） | <https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg> | §3.1、§3.5、§4.1 |
| C17 | REP 105（`base_link`/`odom`/`map` 语义、`map` 的 Z 向上、允许插入中间帧） | <https://www.ros.org/reps/rep-0105.html> | §4.4 |
| C18 | REP 103（body 帧 x forward / y left / z up；世界帧 ENU） | <https://www.ros.org/reps/rep-0103.html> | §4.4 |
| C19 | nav2 `nav2_costmap_2d/src/observation_buffer.cpp`（按 `frame_id` 走 TF 转进 global_frame，再判高度带） | <https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/src/observation_buffer.cpp#L79-L125> | §3.1、§4.2 |
| C20 | nav2 `nav2_costmap_2d/plugins/obstacle_layer.cpp`（ObservationBuffer 用 `global_frame_` 构造） | <https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/obstacle_layer.cpp#L240-L260> | §4.2 |
| C21 | 上游 `small_point_lio`（`Yancey2023@688d75c`，= 本仓 `third_party/cod_nav_2026` 副本） | <https://github.com/Yancey2023/small_point_lio>（本仓副本逐字：`third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio_node.cpp:75`、`:107-124`、`:159`） | §2、§3.0、§3.2 |
| **B1** | **本仓实测账本**：`docs/tilted_lidar_fidelity.md` §I.5.1/§I.5.3/§J.1/§J.2/§J.5/§N.2/§N.3 | `docs/tilted_lidar_fidelity.md` | §3 全部数字 |

> 计数：**引用 22 条** = 外部 URL 20 条（C1–C21，其中 C7 为空号；C21 是上游仓库 + 本仓副本逐字）＋ 本仓实测账本 1 条（B1）＋ 本仓源码/适配节点 1 条（§2 末两行的“本仓源码”）。其中 C2/C8/C9/C20 只作背景/旁证，**判决只依赖 C1/C2′/C5/C6/C10/C11/C12/C16 + B1**。

### 5.2 打不通 / 降级替代（诚实清单）

1. **`web_fetch` 打不开 `github.com`**：本会话的 `web_fetch` 对 `github.com` 返回 `URL hostname "github.com" resolves to a non-public IP address`（环境 DNS/代理所致），因此**所有 GitHub 内容都改用 `curl` 直取** `raw.githubusercontent.com`（源码，HTTP 200 校验过字节数）与 `api.github.com`（issue 正文 + 全部评论 JSON，`per_page=100`）。⇒ 结论未降级，但**无法**用浏览器渲染态截图核对。
2. **GitHub code-search API**：`/search/code` 返回 `401 Requires authentication` ⇒ "全仓 grep `base_link` = 0 次"这类结论**只对我实际下载的 4 个文件**成立（`laserMapping.cpp`、`IMU_Processing.hpp`、`p2.cpp`、`lddc.cpp`），不是全仓证明。
3. **`TixiaoShan/LIO-SAM` 的上游拼写坑**：文件真名是 `src/mapOptmization.cpp`（上游把 Optimization 拼错），我第一轮取 `mapOptimization.cpp` 得到 14 B 的 `404: Not Found`（曾据此误得"LIO-SAM 不用外参"的空结论），已重取 74 655 B 真文件并据此改判 —— 记在这里防后人再踩。
4. **`livox_ros_driver2` 的官方 wiki / MID360 用户手册**：没有取（`livox_wiki` 站外链未验证可达）。`extrinsic_parameter` 的语义**改用 issue #284 的用户实证 + `lddc.cpp` 源码**（驱动里不出现该参数 ⇒ 变换发生在 SDK 侧）。
5. **`bullet3` `LinearMath/Transform.h`**：`raw` 路径 404（文件已迁移）⇒ "`A*B` 就是 `R_A·R_B`"这条**没用外部源码钉**，而是用 LIO-SAM 的合成写法（C10）+ 本仓 `tools/scripts/tiltmount/tf2_compose_order_test.cpp`（§I.9 的实测）。
6. **Point-LIO 的 `pointBodyToWorld` 定义体**：`src/laserMapping.cpp` 里只有调用（`:950/:1137/:1297`），定义在别的 TU/头文件里，本轮没追 ⇒ Point-LIO 的"外参乘一次"结论由 `:133`/`:983`（显式 `offset_R_L_I*p + offset_T_L_I`）+ `:950` 的调用共同支撑，未逐行核对定义体。
7. **没跑仿真、没改代码**：本文所有"修后"数字都是**引用**本仓既有的 A/B 账本（B1），不是本轮新测。
