#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从上游 robot11 URDF **生成** `robot:=robot11` 槽位的仿真 xacro（Phase 2）

为什么用"生成"而不是"手抄"：手抄 12 个 link × (inertial+visual+collision) + 12 个 joint
一定会抄错一两个数，而且上游改版后没人知道哪一版被抄进去了。生成 ⇒ **provenance 是构造性的**：
本文件的输出可以从 `urdf/upstream/robot11.urdf` + 碰撞清单**逐字节重算**。

生成规则（三条，全部写进产物的注释里）：
  1. **运动学/inertial/visual 逐字照抄**上游（link 名、joint 名/类型/父子/xyz/rpy/axis、
     mass/inertia/origin、mesh 文件名、材质颜色）；唯一例外见第 3 条。
  2. `<collision>` **全部替换**为 Phase 1 的成果（上游 12 个 link 的 collision 全是 mesh，
     其中 `body` 是 208 万面 ⇒ 实测有接触时 RTF 0.20 / 峰值内存 759 MB，不能用）：
       · `body`        → 4 个 box（DP 最优 z 分段包络，覆盖率 1.0，见 inventory/collision_assets.json）
       · `l6..l9`（轮）→ `<cylinder r=0.0580 l=0.0452>`（由轮 mesh 包围盒直接得到）
       · 其余（l2..l5, l10, l11, livox_frame）→ `meshes/generated/<n>_collision.stl`（抽稀件）
  3. `body_to_livox` 的 `rpy` 参数化成 `$(arg livox_tilt_rpy)`（**默认值 = 上游/CSV 的 roll 形式**），
     因为"30° 到底绕哪个轴"上游两份材料自相矛盾，必须留一个开关实测（见 docs/robot_models.md §9.5）。

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
  <!-- 【我们加的】雷达 30° 安装的旋转；默认 = SolidWorks CSV 的 roll 形式。
       pitch 形式 = "0 -0.523598775598293 0"。两者对 360° 雷达几何等价（差一个绕 z 的旋转）。 -->
  <xacro:arg name="livox_tilt_rpy" default="-0.523598775598293 0 0"/>

'''

MESH_BBOX = {}
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
  <gazebo reference="body"><material>Gazebo/Orange</material></gazebo>
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

  <!-- ==================== 几何速查（实测值，来自 Phase 1；供 config 对照） ====================
       地面平面（body 系） z = **-0.102499**（四个轮 mesh 的最低点；轮 r=0.0580 心在 z=-0.0445）
       ⚠️ j2..j5 的 origin z=0.05735 是**转向关节**高度，**不是**轮心 —— 差 0.102 m
       雷达 livox_frame  z = 0.157028 ⇒ **离地 0.2595 m**（对比：默认模型 0.226 / hzmirm 0.80）
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
    ap.add_argument('--body-collision', default='boxes', choices=['mesh', 'boxes'],
                    help='body 的碰撞表示：mesh=抽稀碰撞网格（默认，保留雷达凹槽开口）/ boxes=4 个 DP box')
    ap.add_argument('--inventory',
                    default='src/rm_simulation/robot11_description/inventory/mesh_inventory.json')
    ap.add_argument('--out', default='src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro')
    a = ap.parse_args()
    global args
    args = a
    up = os.path.join(REPO, a.upstream)
    sha = hashlib.sha256(open(up, 'rb').read()).hexdigest()
    boxes = json.load(open(os.path.join(REPO, a.assets)))['boxes']['base_link']['boxes']
    global MESH_BBOX
    MESH_BBOX = {m['name']: m['bbox_m'] for m in
                 json.load(open(os.path.join(REPO, a.inventory)))['meshes']}

    root = ET.parse(up).getroot()
    L = [HEADER.format(sha=sha)]

    for link in root.findall('link'):
        nm = link.get('name')
        L.append('  <!-- ==================== link %s ==================== -->\n' % nm)
        L.append('  <link name="%s">\n' % nm)
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
        if nm == 'body' and args.body_collision == 'mesh':
            # ⚠️ 实测选择：**body 用抽稀 mesh（3000 面）而不是 4 个 box**。
            #   为什么改：雷达装在底盘**顶板上的一个凹槽**里（雷达原点 z=0.157 < 底盘顶面 0.213），
            #   4 个 box 是"实心包络"⇒ 把凹槽填实了 ⇒ 每条朝下的射线都打在包络内壁上
            #   （实测：75.5% 的点 r<0.12 m，其中 80% 落在第 4 个 box 的面/内部；地面完全看不见）。
            #   抽稀 mesh 是**表面**碰撞 ⇒ 射线能穿过凹槽的开口。代价见 Phase 1 实测：
            #   悬空稳态 RTF 1.002 / RSS 172 MB（与 box 同价），只有"底盘真的蹭到东西"时才掉到 0.20。
            L.append('    <!-- body：**抽稀碰撞 mesh**（2,078,226 → 3,000 面；保留凹槽开口）\n'
                     '         生成：robot11_make_collision_assets.py，'
                     '清单：robot11_description/inventory/collision_assets.json -->\n')
            L.append('    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                     '      <geometry><mesh filename="package://robot11/meshes/generated/'
                     'base_link_collision.stl"/></geometry>\n    </collision>\n')
        elif nm == 'body':
            L.append('    <!-- body：4 个 box = DP 最优 z 分段包络（覆盖率 1.000 / 总体积 0.065121 m³，'
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
        else:
            # ⚠️ 实测（2026-10-07）：这些小 link 用**抽稀 mesh** 当 collision 会让 Gazebo 在插入模型时
            #   **卡住甚至 segfault**（gzserver exit -11；换上抽稀件后模型干脆不出现在世界里）。
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
        o = j.find('origin')
        ax = j.find('axis')
        L.append('  <!-- ==================== joint %s ==================== -->\n' % nm)
        if nm == 'body_to_livox':
            L.append('  <!-- 【上游原样 + 参数化】xyz 逐字照抄；rpy 参数化：默认 = CSV 的 roll 形式，\n'
                     '       可切 pitch（两份上游材料矛盾，见文件头与 docs/robot_models.md §9.5） -->\n')
            rpy = '$(arg livox_tilt_rpy)'
        else:
            L.append('  <!-- 【上游原样】 -->\n')
            rpy = (o.get('rpy', '0 0 0') if o is not None else '0 0 0')
        L.append('  <joint name="%s" type="%s">\n' % (nm, j.get('type')))
        L.append('    <origin xyz="%s" rpy="%s"/>\n'
                 % ((o.get('xyz', '0 0 0') if o is not None else '0 0 0'), rpy))
        L.append('    <parent link="%s"/>\n    <child link="%s"/>\n'
                 % (j.find('parent').get('link'), j.find('child').get('link')))
        if ax is not None:
            L.append('    <axis xyz="%s"/>\n' % ax.get('xyz'))
        L.append('  </joint>\n\n')

    L.append(FOOTER)
    out = os.path.join(REPO, a.out)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(''.join(L))
    sys.stderr.write('[xacro] %s -> %s (%d B)\n' % (a.upstream, a.out, os.path.getsize(out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
