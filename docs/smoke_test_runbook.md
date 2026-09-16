# 仿真运行验证手册（Smoke Test Runbook）

> 用途：用**原生 `ros2 launch` 指令**逐场景验证仿真链路（含 T1–T5 TF/话题契约改造后的效果）。
> 配套文档：`docs/architecture.md`（目录架构）、`docs/tf_interface_contract.md`（TF/接口契约设计）、`docs/params_ownership_checklist.md`（参数归属）。
> 说明：本文不使用任何封装脚本，全部为可直接复制的原生命令。

---

## 0. 前置

### 0.0 依赖前提（首次务必确认）

**① 系统 python 必须是 3.10（不能用 conda）**
ROS Humble 的 `rclpy` 等 C 扩展是 **cpython-310** 编译的；conda base 若是 3.13 等版本，**无法通过 pip 补齐**（`import rclpy` 直接失败），并且 `spawn_entity.py` 会因 conda python 缺 numpy 而崩溃 → 机器人不生成。
```bash
conda deactivate            # 方式一：退出 conda
# 方式二（不退出 conda，让系统 python 优先）：
export PATH=/usr/bin:$PATH
which python3               # 必须 /usr/bin/python3
```
推荐在 `~/.bashrc` 里加别名，之后每个终端执行 `rosenv` 即可：
```bash
alias rosenv='export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.bash; source ~/HzMi_rmsimulation/install/setup.bash'
```

**② Nav2 主体包必须安装**（否则组件与 RViz 面板全部加载失败）
```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins
ls /opt/ros/humble/share | grep -c "^nav2"      # 期望 20+（只装基础件时只有 7）
```
另外 teb 编译依赖 `ros-humble-dwb-critics`（若尚未安装：`sudo apt install -y ros-humble-dwb-critics`）。

### 0.1 环境（每个新终端都要做）
```bash
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash
source install/setup.bash
# 若 shell 里有 conda/miniconda 干扰 ROS 工具：export PATH=/usr/bin:$PATH
# 若终端启动时提示 xxx/vision_ws/install/setup.bash 不存在：那是 ~/.bashrc 里遗留的旧工作区路径，无害，可自行清理
```

### 0.2 清理残留进程（切换场景前执行一次）
```bash
pkill -f gzserver; pkill -f gzclient; pkill -f rviz2
pkill -f fastlio_mapping; pkill -f pointlio_mapping
pkill -f cartographer_node; pkill -f slam_toolbox
pkill -f controller_server; pkill -f planner_server; pkill -f bt_navigator
sleep 2
```

### 0.3 通用约定
- **终端 A** 跑 launch（会弹 Gazebo/RViz），**终端 B** 跑检查命令；一个场景 `Ctrl+C` 结束后再跑下一个；
- `bringup_sim.launch.py` 常用参数与默认值：

| 参数 | 取值 | 默认 |
|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026` | `RMUL2026` |
| `mode` | `mapping` / `nav` | 空（**必填**） |
| `lio` | `fastlio` / `pointlio` / `none` | `fastlio` |
| `localization` | `amcl` / `slam_toolbox` / `icp`（仅 nav） | 空（nav 模式**必选**） |
| `nav` | `rpp` / `dwb` / `teb` | `rpp` |
| `mapper` | `slam_toolbox` / `cartographer`（仅 mapping） | `slam_toolbox` |
| `lio_rviz` / `nav_rviz` | `True` / `False` | `False` / `True` |

---

## 1. 场景一：mapping + fastlio（先跑这个）

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=slam_toolbox
```

**终端 B（检查）**
```bash
ros2 topic hz /odom                          # 期望 ≈10Hz（fastlio）
ros2 topic hz /livox/lidar                   # 期望：有数据
ros2 topic echo /scan --once                 # 期望：一帧 LaserScan
ros2 run tf2_ros tf2_echo odom base_link     # 期望：持续输出，无 Invalid frame ID
ros2 run tf2_tools view_frames               # 生成 ~/HzMi_rmsimulation/frames.pdf
```

---

## 2. 场景二：nav + AMCL

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp
```

**终端 B**
```bash
ros2 run tf2_ros tf2_echo map odom           # 期望：由 amcl 提供
ros2 lifecycle get /controller_server        # 期望：active
ros2 lifecycle get /planner_server           # 期望：active
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty   # 可选
ros2 action list | grep navigate_to_pose
```

---

## 3. 场景三：nav + ICP（T5 关键验证点）

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=icp nav:=rpp
```

**终端 B**
```bash
ros2 run tf2_ros tf2_echo map odom           # 关键：应由 icp_registration 发布（T5 验证）
ros2 topic echo /initialpose --once          # 可选：给初值，观察 map→odom 是否修正
```

---

## 4. 场景四：nav + slam_toolbox

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=slam_toolbox nav:=rpp
```

**终端 B**
```bash
ros2 run tf2_ros tf2_echo map odom
```

---

## 5. 场景五：cartographer 建图

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=cartographer
```

**终端 B**
```bash
ros2 topic list | grep cartographer
ros2 run tf2_ros tf2_echo map odom
```

---

## 6. 场景六：cartographer 纯定位（显式加载 pbstream）

**终端 A**
```bash
ros2 launch rm_nav_bringup cartographer_sim.launch.py \
  configuration_basename:=cartographer_localization.lua \
  load_state_filename:=$HOME/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL.pbstream \
  load_frozen_state:=true
```

**终端 B**
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /cartographer_map --once
```

> 注意：`RMUL2026.pbstream` 目前仅 526B（疑空），纯定位请先用 `RMUL.pbstream` 或重新建图。

---

## 7. 局部规划器变体（RPP / DWB / TEB）

```bash
# RPP（默认）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp
# DWB
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=dwb
# TEB
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=teb
```

> 变体文件位置：`src/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml`。

---

## 8. 全矩阵指令

### 8.1 mapping（3 场地 × 2 LIO）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=pointlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=mapping lio:=pointlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=mapping lio:=fastlio
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=mapping lio:=pointlio
```

### 8.2 nav × RMUL（2 LIO × 3 重定位）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio  localization:=icp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=pointlio localization:=icp
```

### 8.3 nav × RMUC（2 LIO × 3 重定位）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=fastlio  localization:=icp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=slam_toolbox
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav lio:=pointlio localization:=icp
```

### 8.4 nav × RMUL2026（仅 AMCL：无 posegraph、无 pcd）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=fastlio  localization:=amcl
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav lio:=pointlio localization:=amcl
```

### 8.5 特殊场景
```bash
# 关闭 LIO（预期：无 /odom、无 odom→base_link；nav 报 TF 超时 = 关闭生效）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=none localization:=amcl
# 不开 RViz
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav_rviz:=False
# 开 LIO 点云可视化
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl lio_rviz:=True
```

---

## 9. 判据速查

| 检查 | 正常表现 |
|---|---|
| `tf2_echo odom base_link` | 持续输出、随车移动，无 `Invalid frame ID` |
| `tf2_echo map odom`（nav 模式） | 由所选重定位模块提供（amcl / slam_toolbox / icp） |
| `topic hz /odom` | fastlio ≈10Hz；pointlio 更高（数十~100Hz） |
| `view_frames` | 主链 `map→odom→base_link→livox_frame…`；`camera_init→body` 为孤岛（正常） |
| `lifecycle get /controller_server` | `active` |

---

## 10. 错误对照

| 现象 | 原因 | 处理 |
|---|---|---|
| `Timed out waiting for transform from base_link to map` | `map→odom` 缺失 | nav 模式必须指定 `localization:=amcl\|slam_toolbox\|icp` |
| `Invalid frame ID "base_link"` / fake_vel 报 `Could not transform odom to base_link` | `/odom` 无数据 → LIO 或 `lio_tf_adapter` 未启动 | 查终端 A 是否打印 `lio_tf_adapter 启动` |
| `Found two parents` / 帧树分叉 | 旧进程残留 | 执行 §0.2 清理后重跑 |
| `nav:=teb` 插件找不到 | 未 source 工作区 | `source install/setup.bash`（teb 已编译） |
| `TF_OLD_DATA` | 仿真时间不一致 | 确认 `use_sim_time:=True`（launch 默认） |
| `Unable to parse the value of parameter robot_description as yaml` | launch_ros 把 URDF(XML) 当 YAML 解析（Humble 行为） | **已于 2026-09 修复**：`hzmi_rm_simulation/launch/rm_simulation.launch.py` 用 `ParameterValue(robot_description, value_type=str)` 包裹；若仍出现，说明用的是修复前的 launch |
| `ModuleNotFoundError: No module named 'numpy'` 且 `spawn_entity` 退出 | 用了 conda 的 python（ROS Humble 需系统 python 3.10） | 见 §0.0①：`conda deactivate` 或 `export PATH=/usr/bin:$PATH` |
| `Could not find requested resource in ament index`（nav2 组件成批加载失败） | Nav2 主体包未安装 | 见 §0.0②：apt 安装 navigation2 / nav2-bringup / nav2-rviz-plugins |
| RViz 报 `nav2_rviz_plugins/... does not exist` | 缺 `nav2-rviz-plugins` | 同上 |
| `New subscription discovered on topic '/scan', requesting incompatible QoS` | costmap `obstacle_layer` 默认 reliable，而 `/scan` 是 best-effort | **已于 2026-09 修复**：三份 `nav2_params_sim_{rpp,dwb,teb}.yaml` 的 scan 源加了 `reliability_policy: best_effort`；若仍出现，检查是否有其它 reliable 订阅者（AMCL/自定义节点） |
| cartographer 纯定位报找不到状态文件 | pbstream 路径错或文件为空 | 用绝对路径；`RMUL2026.pbstream` 疑空，改用 `RMUL` |

---

## 11. 出问题时提供什么

```bash
# 1) 运行日志
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl 2>&1 | tee /tmp/run.log

# 2) 三条关键输出（各贴几行即可）
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /odom
ls -l ~/HzMi_rmsimulation/frames.pdf

# 3) 或录 20 秒包供离线分析（Ctrl+C 结束）
ros2 bag record -o /tmp/smoke /odom /tf /tf_static /scan /cmd_vel
```

**优先验证顺序**：场景一（确认 `/odom` 与 `odom→base_link` 存在） → 场景三（确认 ICP 能发 `map→odom`） → 其余场景。
