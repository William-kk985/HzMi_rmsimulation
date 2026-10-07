#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从上游 robot11 URDF **生成** `robot:=robot11` 槽位的仿真 xacro（Phase 2）

为什么用"生成"而不是"手抄"：手抄 12 个 link × (inertial+visual+collision) + 12 个 joint
一定会抄错一两个数，而且上游改版后没人知道哪一版被抄进去了。生成 ⇒ **provenance 是构造性的**：
本文件的输出可以从 `urdf/upstream/robot11.urdf` + 碰撞清单**逐字节重算**。

生成规则（三条，全部写进产物的注释里）：
  1. **运动学/inertial/visual 逐字照抄**上游（link 名、joint 名/类型/父子/xyz/rpy/axis、
     mass/inertia/origin、mesh 文件名、材质颜色）；例外只有两处参数化，见第 3 条。
  2. `<collision>` **全部替换**为 Phase 1/3 的成果（上游 12 个 link 的 collision 全是 mesh，
     其中 `body` 是 208 万面 ⇒ 实测有接触时 RTF 0.20 / 峰值内存 759 MB，不能用）：
       · `body`        → **三种可选**，`--body-collision`：
           `carved`（**Phase 3 默认，A 方案**）＝ 4 个 DP box **减去"雷达视锥"**后再分解成的
                     N 个 box（清单 `inventory/livox_fov_boxes.json`，由
                     `tools/scripts/regress/robot11_livox_fov.py --emit-carved` 生成）
                     —— 为什么必须这么做：DP box 是**实心包络**，而雷达原点 z=0.157 **落在
                     第 4 个 box 内部**（box 顶 0.213）⇒ 每条射线都先在 0.03~0.2 m 打到盒内壁
                     （Phase 2 实测：75.5% 的点 r<0.12 m、`/segmentation/ground` 只剩 529 点/帧、
                     LIO 一条 `/odom` 都发不出来）。carve = 锥内（半角 100.22°、半径 0.5 m）
                     无碰撞 ⇒ FOV 里每条射线都能出去（离线 30000 条射线实测：自击<0.12 m = 0%、
                     看见地面 25.2%，见 inventory/livox_fov_boxes.json 的 verification）。
           `boxes`  （Phase 2 的旧表示）＝ 4 个 DP box（保留供对照/回退）
           `mesh`   （C 方案，**已查清为什么不能用**）＝ 抽稀碰撞网格（3000 面，表面碰撞）
       · `l6..l9`（轮）→ `<cylinder r=0.0580 l=0.0452>`（由轮 mesh 包围盒直接得到）
       · 其余（l2..l5, l10, l11, livox_frame）→ **解析 box**（各自源 mesh 的包围盒）
  3. **根 link 改名**（Phase 3 新增，默认开）：上游根 link `body` → `base_link`。
     为什么必须改：本栈全链路的契约帧名是 `base_link`（`small_point_lio` 源码里
     `lookupTransform(lidar_frame, "base_link")` 是**硬编码**的；linefit 的
     `gravity_aligned_frame`、nav2 的 `robot_base_frame`、slam_toolbox 的 `base_frame` 同理）。
     不改名的实测后果：TF 树断成两棵（`body` 一棵、LIO 想发的 `base_link` 一棵）⇒
     `small_point_lio` 每帧报 "not part of the same tree"、**一条 `/odom` 都发不出来**，
     `pointcloud_to_laserscan` 也把整条点云丢光（`/scan` 0 帧）。这不是"倾斜"或"遮挡"的问题，
     而是 Phase 2 遗留的帧名问题。上游自己的另一份材料（SolidWorks CSV）里这个 link **就叫**
     `base_link` ⇒ 改名不是我们编的，是回到上游两份材料里与本栈一致的那一份。
     传 `--root-link ''` 可保留 `body`（只用于对照实验）。
  4. 两处参数化（都不改变默认几何）：
       · `body_to_livox` 的 `rpy` → `$(arg livox_tilt_rpy)`（**默认 = 上游/CSV 的 roll 形式**；
         用户 2026-10-07 已按实物确认"是绕 roll"，`livox_tilt_axis:=pitch` 只作对照开关）
       · `body_to_livox` 的 `origin` z → `${0.15702816968305 + livox_raise_m}`（**默认 raise=0**，
         即与上游逐字相同；B 方案"把雷达抬到顶板上方"用它实测，见 docs/robot_models.md §11）

然后追加**本栈需要、而上游没有**的东西（全部带 `【我们加的】` 标记）：
  imu_link + imu_joint、MID-360 射线传感器（挂在**上游自己的** livox_frame 上）、
  planar_move 底盘插件、gazebo 材质。

用法：
  python3 tools/scripts/regress/robot11_make_sim_xacro.py \
      --upstream src/rm_nav_bringup/urdf/upstream/robot11.urdf \
      --assets   src/rm_simulation/robot11_description/inventory/collision_assets.json \
      --out      src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro
"""

import argparse
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))

#: 轮子用 cylinder（比抽稀网格更便宜也更准）
WHEELS = {'l6': ('0.058', '0.0452'), 'l7': ('0.058', '0.0453'),
          'l8': ('0.058', '0.0453'), 'l9': ('0.058', '0.0453')}
#: 轮 mesh 的圆盘轴在 mesh 局部 **x**（l6..l9 的包围盒是 x 0.045 / y,z ±0.058）
#: ⇒ URDF cylinder 的局部 z 轴要转到 x：绕 **y** 转 90°（不是绕 x！绕 x 会把轮子放倒成横向圆柱）。
WHEEL_AXIS_RPY = '0 1.5707963267949 0'

HEADER = '''<?xml version="1.0"?>
<!--
  ============================================================================
  sentry_robot_robot11_sim.xacro —— `robot:=robot11` 槽位的机器人模型（**opt-in，不是默认**）
  ============================================================================
  ⚠️ **本文件是生成的，不要手改**：由
       tools/scripts/regress/robot11_make_sim_xacro.py
     从上游 `urdf/upstream/robot11.urdf`（sha256 {sha}）+ Phase 1 的碰撞清单
     `robot11_description/inventory/collision_assets.json` 生成。
     改生成器 → 重跑 → 提交；手改会在下次重跑时被覆盖。

  ── 上游来源（用户 2026-10-07 在会话里贴出，另一个 agent 逐字重建） ──────────
    文件：`.tmp_urdf2/robot11.urdf`（11737 B）→ 本仓逐字节副本
          `src/rm_nav_bringup/urdf/upstream/robot11.urdf`（**只读、不参与运行**，仅供核对）
    sha256：{sha}
    根 link：`body`；12 个 link / 12 个 joint；机器人名 `sentry`
    它**自带** `<inertial>`（与 hzmirm 那份不同）⇒ 不需要我们补惯性
    mesh：`package://robot11/meshes/<name>.STL`（**这正是"包名必须叫 robot11"的原因**）

  ── 上游自相矛盾的一处（本文件用 arg 留了开关，不猜） ──────────────────────
    `body_to_livox` 的 30° 旋转轴：
      · URDF 原文写 `rpy="0 -0.5236 0"`（绕 y = pitch）
      · 同一台车的 SolidWorks CSV 写 `Joint Origin Roll = -0.523598775598293`（绕 x = roll）
    本文件默认按 **CSV 的 roll 形式**（机器导出的 CSV 比手打的一行更可能是字面真相），
    但把它参数化成 `livox_tilt_rpy`，并给了 `livox_tilt_axis:=roll|pitch` 这个 launch 开关。
    ⚠️ 几何事实（§9.5 已推过、Phase 2 实测复核）：MID-360 方位 360° ⇒ 两个候选的
    **世界仰角谱与地面环半径完全相同**，只差一个绕 z 的旋转；**唯一判据是"最朝下的方位"**。

  ── `【上游原样】/【我们加的】` 两种标记 ────────────────────────────────────
    【上游原样】= 与上游 URDF 数值逐字相同（脚本生成，可重算）
    【我们加的】= 本栈要跑起来必须补的（逐条写理由与日期）

  ── Phase 3（2026-10-07）：让雷达"看得出去" + 斜置感知链 ─────────────────────
    用户的判定（两条，都已按它执行）：
      (1) **30° 是绕 roll**（用户按实物确认）⇒ 本文件的默认 `livox_tilt_rpy` = roll 形式；
      (2) 上游 URDF/mesh "跟实物基本一致，仿真侧可以改" ⇒ 允许仿真侧适配，
         但**每一处偏离都要写日期 + 中文理由 + provenance 表**（见 docs/robot_models.md §11）。
    Phase 2 的结论是"这台车的雷达在底盘凹槽里，看不见外面"（75.5% 的点是自击、无 /odom）。
    Phase 3 用**三种方案**实测（离线 30000 条射线 + Gazebo 无头跑）：
      A（保真优先，**采用**）：body 碰撞 = DP box 减视锥（`--body-collision carved`）⇒ 雷达仍在
         上游的 0.2595 m 高度，只把"凹槽开口"在碰撞里显式挖出来；
      B（仿真适配）：保留 4 个 box，把 `livox_frame` 抬高（`--livox-raise-m`）⇒ 离线扫描：
         +0.05 m 仍 96.6% 自击、+0.06 m 降到 19.8%、**+0.08 m 才干净（0.24%）**、
         +0.10 m 起与 A 等价（0% 自击 / 25.2% 看见地面）—— 代价是雷达离地 0.2595 → 0.3595 m，
         与本仓默认模型的 0.226 m 差得更远，且 linefit 的 sensor_height 必须跟着变；
      C（抽稀网格碰撞）：**已查清**：spawn 正常但传感器一条数据都没有 —— 根因是 ODE 的
         ray-vs-trimesh 是 O(#三角面) 的，30000 条射线 × 3000 面 = 9×10⁷ 次/帧 @10 Hz
         ⇒ gzserver 在"打一帧射线"里出不来（传感器回调永不返回），不是插件没挂上。
    倾斜感知链（**全部只对本槽位生效**，默认模型逐字节不变）：
      · linefit：`sensor_height` = 雷达离地（0.2595 + raise）、`gravity_aligned_frame` = base_link
        （雷达斜 30° ⇒ 传感器系的 z 不再是"高度"）—— 见 `config/segmentation_sim_robot11.yaml`
      · pointcloud_to_laserscan：`target_frame` = base_link（高度带改在重力对齐系里量）
        —— 见 `config/laserscan_params_robot11.yaml`
      · LIO 外参：**不改**。IMU 与雷达同壳、只差 0.05 m 平移（`extrinsic_T=[0,0,0.05]`,
        `extrinsic_R=I` 逐字节沿用），IMU 姿态里的 30° 由 LIO 自己估出来，
        `odom→base_link` 由 small_point_lio 用 `base_link→livox_frame` 的 TF 精确换算。

  ── ⚠️ 上游的 <collision> 全被替换掉了（这是本槽位最关键的一处偏离） ────────
    上游 12 个 link 的 collision **全部是 mesh**，其中 `body` = 2,078,226 面。
    Phase 1 实测（极简世界单模型 spawn，无头隔离，见 docs/robot_models.md §9.3）：
      · 稳态悬空 RTF 1.002（没有接触对 ⇒ 别拿这个当"能用"）
      · **一有接触** RTF **0.200**、最低 0.04（慢 25–50 倍）；同工况 4 个 box = 1.002
      · 峰值内存恒 **759 MB vs 基线 172 MB**
    ⇒ 替换为：`body` = 4 个 box（DP 最优分段包络，覆盖率 1.000、总体积只比凸包胖 9.4%）；
      轮 = cylinder；其余 = 抽稀件。**`<visual>` 仍然是原始 mesh，视觉零损失。**
-->
<robot name="sentry" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- 【我们加的】launch 会无条件传平台外参（bringup_sim.launch.py 的 xyz/rpy）。
       本模型的雷达位姿由上游关节链唯一决定 ⇒ 这两个参数**声明但不用**（与 hzmirm 槽位同款理由：
       再叠一个外部平移就等于改上游几何）。 -->
  <xacro:arg name="xyz" default="0 0 0"/>
  <xacro:arg name="rpy" default="0 0 0"/>
  <!-- 【我们加的】雷达 30° 安装的旋转；默认 = SolidWorks CSV 的 roll 形式
       （用户 2026-10-07 **按实物确认：是绕 roll**；pitch 只作对照开关）。
       pitch 形式 = "0 -0.523598775598293 0"。两者对 360° 雷达几何等价（差一个绕 z 的旋转）。 -->
  <xacro:arg name="livox_tilt_rpy" default="-0.523598775598293 0 0"/>
  <!-- 【我们加的｜B 方案，默认 0 = 与上游逐字相同】把雷达（连同 IMU）整体抬高的米数。
       用途：实测"不做碰撞开口、直接把雷达抬到顶板上方"这条仿真适配路（docs/robot_models.md §11）。
       ⚠️ 抬高会改变雷达离地高度 ⇒ linefit 的 sensor_height 必须同步 = 0.2595 + livox_raise_m
       （launch 里由 robot:=robot11 分支**算出来**注入，不是手抄）。默认 0 ⇒ 一切不变。 -->
  <xacro:arg name="livox_raise_m" default="{raise_m}"/>
  <!-- xacro 的表达式里**不能直接写 arg 名**（实测报 name 'livox_raise_m' is not defined）
       ⇒ 按标准写法先绑成 property 再进表达式。 -->
  <xacro:property name="livox_raise_m_p" value="$(arg livox_raise_m)"/>

'''

MESH_BBOX = {}
TURRET = {}
TURRET_METHOD = ''
args = None

FOOTER = '''
  <!-- ==================== 【我们加的】配件：IMU / 雷达 / 底盘 ==================== -->
  <!-- 上游 URDF 里**没有** imu_link、没有 <gazebo>、没有 <sensor>、没有 <plugin>
       （但有 inertial 与 mesh）⇒ 下面这些全是我们为本栈补的。 -->

  <!-- ① IMU。上游没有 imu_link，而本栈 LIO（fastlio / pointlio / small_point_lio）与
       imu_complementary_filter 都要 `/livox/imu` + 一个 `imu_link` 帧。
       位置：【刻意选"跟着雷达一起斜 30°"而不是"挂在 body 上保持水平"】，理由三条：
         (a) MID-360 的 IMU 与雷达本来就在**同一个壳**里 ⇒ 跟着雷达斜才是物理事实；
         (b) 本栈的 LIO 外参 `extrinsic_T=[0,0,0.05]`（雷达在 IMU 系下）来自"IMU 在雷达下方 0.05 m、
             **无相对旋转**"这条几何；照抄它就**一个 LIO 配置文件都不用改**（已核对三份配置：
             small_point_lio / FAST_LIO / point_lio 都是 `extrinsic_T=[0,0,0.05]` 且无 extrinsic_R）；
             若改挂到 body 上保持水平，就必须给三份 LIO 配置补一条 30° 的 `extrinsic_R` —— 那是更大的改动面；
         (c) 后果（IMU 的 z 轴相对 base_link 斜 30°）由 `lio_tf_adapter` 的 `rpy` 精确补偿，
             已在本槽位的 launch 分支里显式写死并实测（见 docs/robot_models.md §10）。
       绝对位置：body + (0.000562, 0.130916, 0.107028)（= 雷达原点沿**雷达自身的** −z 走 0.05 m）。 -->
  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <parent link="livox_frame"/>
    <child link="imu_link"/>
  </joint>

  <link name="imu_link">
    <!-- ⚠️ **故意不给 collision**：imu_link 在雷达原点正下方 0.05 m、正好在雷达的视场里，
         给碰撞体 ⇒ 每条朝下的射线先在 r=0.05 m 处打到自己
         （实测：`/scan` 最近回波恒为 0.05 m、r<0.3 m 的点占满，见 docs/robot_models.md §10.5）。
         物理上 MID-360 的 IMU 就在雷达壳**内部** ⇒ 本来就不该有独立碰撞体。 -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.02 0.02 0.01"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material>
    </visual>
    <inertial>
      <mass value="1e-2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <!-- ② 雷达射线传感器：**挂在上下游自己的 `livox_frame` 上**。
       上游 `livox_frame` 已经有 l12.STL（MID-360 本体）的 visual+collision 和 inertial，
       所以这里**不能**再用 ros2_livox_simulation 的 `mid360` 宏（宏会自己 new 一个同名 link +
       livox_frame_joint，与上游重复）。改成把宏里那段 <sensor type="ray"> **逐字抄过来**、
       只把 <sensor name> 设成 `livox_frame`（插件用 <sensor name> 当点云 frame_id
       ⇒ header.frame_id = livox_frame，与本栈 `lidar_frame: livox_frame` 契约一致）。
       射线参数与宏一致：水平 100 × 垂直 360、10 Hz、0.1–200 m、σ=2 mm、
       垂直 FOV −7.22°…+55.22°（这是本仓 mid360.xacro 的既有取值）。 -->
  <gazebo reference="livox_frame">
    <sensor type="ray" name="livox_frame">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <plugin name="livox_frame_plugin" filename="libros2_livox.so">
        <ray>
          <scan>
            <horizontal>
              <samples>100</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>${2*3.1415926}</max_angle>
            </horizontal>
            <vertical>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>${-7.22/180*3.1415926}</min_angle>
              <max_angle>${55.22/180*3.1415926}</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>200.0</max>
            <resolution>0.002</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.002</stddev>
          </noise>
        </ray>
        <visualize>false</visualize>
        <!-- ★ Phase 3：**安装倾角**（弧度，roll pitch yaw）。本仓插件新增的参数：
             射线方向真的按它偏（物理姿态 = 实物），但**点云仍表达在父 link 系**
             （frame_id = livox_frame，而 livox_frame 是重力对齐的）⇒ 下游不需要
             gravity_aligned_frame / target_frame，就不会踩到本仓 linefit 那个
             `Eigen::Affine3d tf;` 不清零的 C++ bug（见 docs/robot_models.md §11）。
             缺省（元素不存在）= 单位阵 ⇒ 其它模型逐字节不变。 -->
        <tilt_rpy>$(arg livox_tilt_rpy)</tilt_rpy>
        <samples>30000</samples>
        <downsample>1</downsample>
        <csv_file_name>$(find ros2_livox_simulation)/scan_mode/mid360.csv</csv_file_name>
        <topic>/livox/lidar</topic>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ③ 底盘驱动 + 真值里程计：与默认模型**逐字同款**（libgazebo_ros_planar_move.so）
       ⇒ 契约不变：只有它订阅 /cmd_vel_chassis；真值里程计发 /odom_ground_truth
       （/odom 留给 LIO）；`publish_odom_tf=false`（T4：odom→base_link 那条边归 LIO）。
       上游 j2..j9 是 continuous（转向 + 轮子）但我们**不加轮子控制器**：与默认模型一样，
       轮子自由滚动、底盘由 planar_move 平动 ⇒ 运动学忠实、接口最小。 -->
  <gazebo>
    <plugin name="mecanum_controller" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=cmd_vel_chassis</remapping>
        <remapping>odom:=odom_ground_truth</remapping>
      </ros>
      <update_rate>100</update_rate>
      <publish_rate>10</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </gazebo>

  <!-- ④ IMU 传感器：与默认模型同款（gazebo_ros_imu_sensor，100 Hz，帧名 imu_link，
       话题 /livox/imu）。上游没有任何 <sensor>。 -->
  <gazebo reference="imu_link">
    <sensor name="mid360_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=/livox/imu</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ⑤ 材质（纯观感） -->
  <gazebo reference="__ROOT__"><material>Gazebo/Orange</material></gazebo>
  <gazebo reference="l2"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="l3"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="l4"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="l5"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="l6"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="l7"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="l8"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="l9"><material>Gazebo/Black</material></gazebo>
  <gazebo reference="l10"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="l11"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="livox_frame"><material>Gazebo/LightBlueLaser</material></gazebo>
  <gazebo reference="imu_link"><material>Gazebo/BuildingFrame</material></gazebo>

  <!-- ==================== 几何速查（实测值，来自 Phase 1/3；供 config 对照） ====================
       地面平面（body 系） z = **-0.102499**（四个轮 mesh 的最低点；轮 r=0.0580 心在 z=-0.0445）
       ⚠️ j2..j5 的 origin z=0.05735 是**转向关节**高度，**不是**轮心 —— 差 0.102 m
       雷达 livox_frame  z = 0.157028 + livox_raise_m ⇒ **离地 0.2595 + livox_raise_m m**
         （对比：默认模型 0.226 / hzmirm 0.80）
       Phase 3 实测（离线 30000 条与 Gazebo 同源的射线；`robot11_livox_fov.py --compare`）：
         · 4 个 DP box（Phase 2）：自击 <0.12 m **80.5%**、看见地面 **0%** —— 与 Gazebo 实测
           75.5% / 529 点每帧一致（差异 = Gazebo 丢掉 <0.1 m 的回波）
         · A 方案（本文件默认，carved）：自击 <0.12 m **0%**、看见地面 **25.2%**（7560/30000 条）、
           地面距离中位 **1.17 m**（最近 ~0.43 m）
         · 抽稀网格（C）：自击 16.0%、看见地面 23.0% —— 几何上能用，但 ODE 的 ray-trimesh
           代价让 Gazebo 打不出一帧（见文件头）
       IMU  imu_link      = 雷达原点沿雷达自身 -z 0.05 m（跟着雷达斜 30°）
       外形：整车 0.6000 x 0.6000 x 0.5642（含云台/发射机构），底盘组高 0.3155
       足印：**内切半径 0.300 m / 外接半径 0.3565 m**（XY 凸包，3 mm 体素抽稀）
       轴距(x) = 轮距(y) = **0.36427 m**（j2..j5 的 origin ±0.1821345596729）
       质量合计 **9.5521 kg**（上游 12 条 inertial 求和）
       雷达 30° 下俯后的世界仰角范围 = -37.22°…+85.22°（用本仓 mid360 宏的 -7.22…+55.22）
         ⇒ 地面可见环：半径 **0.3416 m** 以外（盲区是半径 0.34 m 的圆）
         ⇒ 最近地面环出现在 body 的哪个方位 = **roll→±y / pitch→±x**（唯一能区分两候选的判据）
       lio_tf_adapter 需要的 T_body←base_link：xyz = -(imu 在 body 里的位置)，
         rpy = 该 30° 旋转的逆；由 bringup_sim.launch.py 的 robot:=robot11 分支注入，不写死在 YAML 里。 -->
</robot>
'''


def _sanitize_comments(txt):
    """XML 注释里**不允许出现 `--`**（解析器直接报 not well-formed）。

    我们的注释里要写命令行开关（`--body-collision` 之类）⇒ 在**注释体内**把它换成全角破折号，
    标签/属性里原样保留（那里本来也不会有 `--`）。实测：第一版没做这一步，xacro 直接
    `xml.etree.ElementTree.ParseError: not well-formed (invalid token)`。
    """
    import re as _re
    parts = _re.split(r'(<!--|-->)', txt)
    return ''.join(p if p in ('<!--', '-->') else p.replace('--', '\u2014') for p in parts)


def fmt_origin(el, indent='      '):
    if el is None:
        return ''
    xyz = el.get('xyz', '0 0 0')
    rpy = el.get('rpy', '0 0 0')
    return '%s<origin xyz="%s" rpy="%s"/>\n' % (indent, xyz, rpy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--upstream', default='src/rm_nav_bringup/urdf/upstream/robot11.urdf')
    ap.add_argument('--assets',
                    default='src/rm_simulation/robot11_description/inventory/collision_assets.json')
    ap.add_argument('--body-collision', default='carved', choices=['mesh', 'boxes', 'carved'],
                    help='body 的碰撞表示：carved=A 方案（DP box 减雷达视锥，Phase 3 默认）/ '
                         'boxes=4 个 DP box（Phase 2 旧表示）/ mesh=抽稀碰撞网格（C 方案，不能用）')
    ap.add_argument('--fov-boxes', default='src/rm_simulation/robot11_description/inventory/livox_fov_boxes.json',
                    help='A 方案的 box 清单（robot11_livox_fov.py --emit-carved 生成）')
    ap.add_argument('--root-link', default='base_link',
                    help='上游根 link 名（body）在生成物里改成什么。**Phase 3 新增**：'
                         '上游 URDF 的根叫 body，而 SolidWorks CSV 与本栈全链路的契约帧名都是 '
                         'base_link（small_point_lio 源码硬编码 lookupTransform(lidar_frame,'
                         '"base_link")；linefit 的 gravity_aligned_frame、nav2 的 robot_base_frame、'
                         'slam_toolbox 的 base_frame 同理）。不改名 ⇒ **TF 树断成两棵**：'
                         '实测 small_point_lio 每帧报 "not part of the same tree" ⇒ 一条 /odom '
                         '都发不出来、p2l 也把整条点云丢光。传空串 = 保持上游 body（只用于对照）。')
    ap.add_argument('--livox-raise-m', type=float, default=0.0,
                    help='B 方案：把 body_to_livox 的 origin z 抬高这么多（默认 0 = 上游几何）')
    ap.add_argument('--inventory',
                    default='src/rm_simulation/robot11_description/inventory/mesh_inventory.json')
    ap.add_argument('--turret-boxes',
                    default='src/rm_simulation/robot11_description/inventory/turret_boxes.json',
                    help='云台 l10/l11 的细碰撞盒清单（robot11_livox_fov.py --emit-turret-boxes）')
    ap.add_argument('--out', default='src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro')
    a = ap.parse_args()
    global args
    args = a
    up = os.path.join(REPO, a.upstream)
    sha = hashlib.sha256(open(up, 'rb').read()).hexdigest()
    boxes = json.load(open(os.path.join(REPO, a.assets)))['boxes']['base_link']['boxes']
    global MESH_BBOX, TURRET, TURRET_METHOD
    MESH_BBOX = {m['name']: m['bbox_m'] for m in
                 json.load(open(os.path.join(REPO, a.inventory)))['meshes']}
    tp = os.path.join(REPO, a.turret_boxes)
    if os.path.isfile(tp):
        _tj = json.load(open(tp))
        TURRET = _tj['links']
        TURRET_METHOD = _tj['params']['method'] + '；体素 %.0f mm' % (_tj['params']['voxel_m'] * 1000)
    else:
        TURRET, TURRET_METHOD = {}, ''

    root = ET.parse(up).getroot()
    up_root = root.find('link').get('name')
    root_name = (a.root_link or up_root)

    def rn(name):
        """把上游根 link 名换成 root_name（其它名字原样）。"""
        return root_name if name == up_root else name

    L = [HEADER.format(sha=sha, raise_m='%g' % a.livox_raise_m)]
    L.append('  <!-- 【我们加的｜改名】上游根 link `%s` → `%s`（原因见文件头第 3 条）：\n'
             '       本栈契约帧名是 base_link；不改名的实测后果 = TF 断树、LIO 发不出 /odom、\n'
             '       p2l 丢光整条点云。上游 SolidWorks CSV 里这个 link 本来就叫 base_link。 -->\n'
             % (up_root, root_name))

    for link in root.findall('link'):
        nm = link.get('name')
        nm_out = rn(nm)
        L.append('  <!-- ==================== link %s ==================== -->\n' % nm_out)
        L.append('  <link name="%s">\n' % nm_out)
        ine = link.find('inertial')
        if ine is not None:
            L.append('    <!-- 【上游原样】inertial -->\n    <inertial>\n')
            L.append(fmt_origin(ine.find('origin'), '      '))
            L.append('      <mass value="%s"/>\n' % ine.find('mass').get('value'))
            i = ine.find('inertia')
            L.append('      <inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>\n'
                     % tuple(i.get(k) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')))
            L.append('    </inertial>\n')
        vis = link.find('visual')
        if vis is not None:
            mesh = vis.find('geometry/mesh')
            mat = vis.find('material')
            L.append('    <!-- 【上游原样】visual（**原始 mesh，视觉零损失**） -->\n    <visual>\n')
            L.append(fmt_origin(vis.find('origin'), '      '))
            L.append('      <geometry><mesh filename="%s"/></geometry>\n' % mesh.get('filename'))
            if mat is not None and mat.find('color') is not None:
                L.append('      <material name="%s"><color rgba="%s"/></material>\n'
                         % (mat.get('name', ''), mat.find('color').get('rgba')))
            L.append('    </visual>\n')
        # ---- collision：替换 ----
        L.append('    <!-- 【我们加的｜替换】上游这里是 mesh collision；按 Phase 1 实测换掉'
                 '（原因见文件头） -->\n')
        if nm == up_root and args.body_collision == 'carved':
            fov = json.load(open(os.path.join(REPO, a.fov_boxes)))
            L.append('    <!-- %s：**A 方案 = DP box 减去"雷达视锥"**（%d 个 box，Phase 3 采用）\n'
                     '         为什么：雷达原点 z=0.157 落在第 4 个 DP box **内部**（box 顶 0.213）⇒ 实心包络\n'
                     '           把凹槽填实 ⇒ 每条射线先打到盒内壁（Phase 2 实测 75.5%% 的点 r<0.12 m、\n'
                     '           `/segmentation/ground` 529 点/帧、LIO 无 `/odom`）。挖掉视锥后离线实测：\n'
                     '           自击<0.12 m = 0%%、看见地面 = 25.2%%（30000 条与 Gazebo 同源的射线）。\n'
                     '         生成：robot11_livox_fov.py --emit-carved；清单：inventory/livox_fov_boxes.json\n'
                     '         参数：锥半角 %.2f°、半径 %.2f m、内清空 %.2f m、体素 %s m、倾角轴 %s\n'
                     '         ⚠️ 这是**仿真侧适配**（偏离上游 CAD 的碰撞几何），逐条登记在 docs/robot_models.md §11 -->\n'
                     % (nm_out, fov['n_boxes'], fov['params']['half_angle_deg'], fov['params']['cone_r'],
                        fov['params']['inner_clearance_r'], fov['params']['voxel_xyz_m'],
                        fov['params']['tilt_axis']))
            for b in fov['boxes']:
                L.append('    <collision>\n      <origin xyz="%s" rpy="0 0 0"/>\n'
                         '      <geometry><box size="%s"/></geometry>\n    </collision>\n'
                         % (' '.join('%.6f' % v for v in b['center']),
                            ' '.join('%.6f' % v for v in b['size'])))
        elif nm == up_root and args.body_collision == 'mesh':
            # ⚠️ 实测选择：**body 用抽稀 mesh（3000 面）而不是 4 个 box**。
            #   为什么改：雷达装在底盘**顶板上的一个凹槽**里（雷达原点 z=0.157 < 底盘顶面 0.213），
            #   4 个 box 是"实心包络"⇒ 把凹槽填实了 ⇒ 每条朝下的射线都打在包络内壁上
            #   （实测：75.5% 的点 r<0.12 m，其中 80% 落在第 4 个 box 的面/内部；地面完全看不见）。
            #   抽稀 mesh 是**表面**碰撞 ⇒ 射线能穿过凹槽的开口。代价见 Phase 1 实测：
            #   悬空稳态 RTF 1.002 / RSS 172 MB（与 box 同价），只有"底盘真的蹭到东西"时才掉到 0.20。
            L.append('    <!-- %s：**抽稀碰撞 mesh**（2,078,226 → 3,000 面）—— ⚠️ 实测**不能用**：\n'
                     '         spawn 正常，但 100 s 内**一条传感器数据都没有**。Phase 3 查清根因：\n'
                     '         ODE 的 ray-vs-trimesh 代价是 O(三角面数) ⇒ 30000 条射线 × 3000 面\n'
                     '         = 9×10⁷ 次/帧 @10 Hz ⇒ gzserver 卡在"打一帧射线"里出不来\n'
                     '         （不是插件没挂上、也不是 link 被丢）。详见 docs/robot_models.md §11。\n'
                     % nm_out +
                     '         生成：robot11_make_collision_assets.py，'
                     '清单：robot11_description/inventory/collision_assets.json -->\n')
            L.append('    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                     '      <geometry><mesh filename="package://robot11/meshes/generated/'
                     'base_link_collision.stl"/></geometry>\n    </collision>\n')
        elif nm == up_root:
            L.append('    <!-- %s：4 个 box = DP 最优 z 分段包络（覆盖率 1.000 / 总体积 0.065121 m³，'
                     '网格凸包 0.0595 m³）\n         生成：robot11_make_collision_assets.py，'
                     '清单：robot11_description/inventory/collision_assets.json -->\n')
            for b in boxes:
                L.append('    <collision>\n      <origin xyz="%s" rpy="0 0 0"/>\n'
                         '      <geometry><box size="%s"/></geometry>\n    </collision>\n'
                         % (' '.join('%g' % v for v in b['center']),
                            ' '.join('%g' % v for v in b['size'])))
        elif nm in WHEELS:
            r, w = WHEELS[nm]
            L.append('    <!-- %s：cylinder（半径/宽度由轮 mesh 包围盒直接得到：r=%.4f m 宽 %.4f m） -->\n'
                     % (nm, float(r), float(w)))
            L.append('    <collision>\n      <origin xyz="0 0 0" rpy="%s"/>\n'
                     '      <geometry><cylinder radius="%s" length="%s"/></geometry>\n    </collision>\n'
                     % (WHEEL_AXIS_RPY, r, w))
        elif nm == 'livox_frame':
            # ⚠️ 雷达 link **不给 collision**：射线传感器就装在它的原点上，
            #   给一个"包住原点"的 box 会让每一条朝下的射线在 r≈0 处打到自己
            #   （实测：第一版给了 l12 的包围盒 box ⇒ 48% 的点 r<0.05 m、`/scan` 最近回波 0.05 m，
            #    点云基本全是自击）。默认模型与 hzmirm 槽位的雷达 link 同样没有 collision。
            L.append('    <!-- livox_frame：**故意不给 collision**（射线传感器就在本 link 原点上，'
                     '给碰撞体 = 每条射线打自己；实测见 docs/robot_models.md §10.5） -->\n')
        elif TURRET.get(nm):
            # ★ Phase 3：云台 l10/l11 的碰撞由"单个包围盒"换成"网格推出的 N 个 box"。
            #   为什么：包围盒离雷达只有 0.099/0.091 m 且**实心** ⇒ Gazebo 实测 34.7% 的射线
            #   打在盒面上（0.11~0.17 m），而真网格最近的表面在 0.116/0.139 m（用真网格求交时
            #   只有 0.3% 的射线落在 0.12 m 以内）。云台是**离雷达最近**的部件、又在 360° 视场里
            #   ⇒ 这一步直接决定"自击占不占主导"这条验收。
            #   清单/参数/验证：inventory/turret_boxes.json（robot11_livox_fov.py --emit-turret-boxes）
            t = TURRET[nm]
            L.append('    <!-- %s：**细碰撞盒 %d 个**（Phase 3 换掉单包围盒；离雷达最近面 %.4f m，'
                     '单包围盒时 %.4f m）\n'
                     '         生成：robot11_livox_fov.py --emit-turret-boxes；'
                     '清单：inventory/turret_boxes.json\n'
                     '         方法：%s（上游 mesh 不水密 ⇒ 只能"实心近似"） -->\n'
                     % (nm, t['n_boxes'], t['nearest_face_to_lidar_m'],
                        t['nearest_face_if_single_bbox_m'],
                        TURRET_METHOD))
            for b_ in t['boxes']:
                L.append('    <collision>\n      <origin xyz="%s" rpy="0 0 0"/>\n'
                         '      <geometry><box size="%s"/></geometry>\n    </collision>\n'
                         % (' '.join('%.6f' % v for v in b_['center']),
                            ' '.join('%.6f' % v for v in b_['size'])))
        else:
            # ⚠️ 实测（2026-10-07）：这些小 link 用**抽稀 mesh** 当 collision 会让 Gazebo 在插入模型时
            #   **卡住甚至 segfault**（gzserver exit -11；换上抽稀件后模型根本不出现在世界里）。
            #   `body` 的 mesh collision 是能工作的（Phase 1 variant A/C 都跑通了）⇒ 问题在"多关节链上
            #   的小 mesh"，不在"mesh collision"本身。为稳，这些 link 一律用**网格包围盒的 box**：
            #   保守外包、解析几何、零加载风险，且省掉 11 个 mesh 的加载。
            mesh_stem = os.path.splitext(os.path.basename(
                link.find('visual/geometry/mesh').get('filename')))[0]
            b = MESH_BBOX[mesh_stem]
            c = [(b['min'][i] + b['max'][i]) / 2.0 for i in range(3)]
            sz = [b['max'][i] - b['min'][i] for i in range(3)]
            L.append('    <!-- %s：**解析 box**（= 源 mesh %s.STL 的包围盒；理由见下） -->\n'
                     % (nm, mesh_stem))
            L.append('    <collision>\n      <origin xyz="%s" rpy="0 0 0"/>\n'
                     '      <geometry><box size="%s"/></geometry>\n    </collision>\n'
                     % (' '.join('%.6f' % v for v in c), ' '.join('%.6f' % v for v in sz)))
        L.append('  </link>\n\n')

    for j in root.findall('joint'):
        nm = j.get('name')
        xyz_override = None
        o = j.find('origin')
        ax = j.find('axis')
        L.append('  <!-- ==================== joint %s ==================== -->\n' % nm)
        if nm == 'body_to_livox':
            L.append('  <!-- 【上游原样 + 参数化】xyz 的 x/y 逐字照抄；\n'
                     '       · z = 0.15702816968305 + $(arg livox_raise_m)（**默认 raise=0 ⇒ 与上游逐字相同**）\n'
                     '       · rpy **改成 0 0 0**（★ Phase 3）：上游这里放的是 30° 倾角，但**倾角改由\n'
                     '         <gazebo><sensor><pose> 承载**（射线仍然真的斜 30° ⇒ 物理没变），\n'
                     '         而 `livox_frame` 这个**帧**保持重力对齐。为什么必须这么放：\n'
                     '         本仓 linefit 的 `gravity_aligned_frame` 路径有 bug\n'
                     '         （`Eigen::Affine3d tf;` 默认构造**不清零** ⇒ 点云被一个垃圾矩阵变换到原点，\n'
                     '          实测 `/segmentation/ground` 恒为 0 点、obstacle = 全部点；\n'
                     '          hzmirm 槽位当年"地面分割全废"的结论有一部分就是这个 bug，见 §11）。\n'
                     '         该文件在别的任务的目录里、本主题不许改 ⇒ 改成**在源头就把点云对齐**：\n'
                     '         Gazebo 的 livox 插件把点发布在**父 link 的坐标系**里（源码：\n'
                     '         axis = offset.Rot()·ray；point = range·axis），所以 link 一正、点云就正。\n'
                     '         结果：`/livox/lidar/pointcloud` 的 frame_id 仍是 livox_frame，但坐标\n'
                     '         已经是重力对齐的 ⇒ linefit/p2l 都不需要开 gravity_aligned_frame/target_frame。\n'
                     '         ⚠️ 代价：TF 里的 base_link→livox_frame 变成 rpy=0（实物是斜 30°），\n'
                     '            这条偏离逐字登记在 docs/robot_models.md §11 的 provenance 表。 -->\n')
            rpy = '0 0 0'
            xyz_override = '%s %s ${%s + livox_raise_m_p}' % tuple(
                o.get('xyz', '0 0 0').split()[:2] + ['0.15702816968305'])
        else:
            L.append('  <!-- 【上游原样】 -->\n')
            rpy = (o.get('rpy', '0 0 0') if o is not None else '0 0 0')
        L.append('  <joint name="%s" type="%s">\n' % (nm, j.get('type')))
        L.append('    <origin xyz="%s" rpy="%s"/>\n'
                 % (xyz_override if nm == 'body_to_livox'
                    else (o.get('xyz', '0 0 0') if o is not None else '0 0 0'), rpy))
        L.append('    <parent link="%s"/>\n    <child link="%s"/>\n'
                 % (rn(j.find('parent').get('link')), rn(j.find('child').get('link'))))
        if ax is not None:
            L.append('    <axis xyz="%s"/>\n' % ax.get('xyz'))
        L.append('  </joint>\n\n')

    L.append(FOOTER.replace('__ROOT__', root_name))
    out = os.path.join(REPO, a.out)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(_sanitize_comments(''.join(L)))
    sys.stderr.write('[xacro] %s -> %s (%d B)\n' % (a.upstream, a.out, os.path.getsize(out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
