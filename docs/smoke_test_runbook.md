# 仿真运行验证手册（Smoke Test Runbook）

> 用途：用**原生 `ros2 launch` 指令**逐场景验证仿真链路（含 T1–T5 TF/话题契约改造后的效果）。
> 配套文档：`docs/architecture.md`（目录架构）、`docs/tf_interface_contract.md`（TF/接口契约设计）、`docs/params_ownership_checklist.md`（参数归属）。
> 说明：本文不使用任何封装脚本，全部为可直接复制的原生命令。

---

## 0. 前置

### 0.0 依赖前提（首次务必确认）

**① 系统 python 必须是 3.10（不能用 conda）**
ROS Humble 的 `rclpy` 等 C 扩展是 **cpython-310** 编译的；conda base 若是 3.13 等版本，**无法通过 pip 补齐**（`import rclpy` 直接失败），并且 `spawn_entity.py` 会因 conda python 缺 numpy 而崩溃 → 机器人不生成。

**本项目统一采用"全跟系统 python"方案**，推荐做法（一次配置，永久生效）：
```bash
# 方式一（推荐）：关闭 conda base 自动激活 —— 新终端直接就是系统 python
conda config --set auto_activate_base false      # 重开终端生效；需要时再 conda activate base
# 方式二：每个 ROS 终端手动退出
conda deactivate
# 方式三：不退出 conda，只让系统 python 优先
export PATH=/usr/bin:$PATH
```
**保底做法**：若新终端里 `which python3` 仍是 `~/miniconda3/bin/python3`，在 `~/.bashrc` **末尾**追加一行即可（conda 的 init 块会把 PATH 插到前面，末尾覆盖最稳）：
```bash
echo 'export PATH=/usr/bin:$PATH' >> ~/.bashrc && source ~/.bashrc
```
（当前终端若已残留 conda PATH：`conda deactivate; export PATH=/usr/bin:$PATH; hash -r`）
验证（必须全部满足）：
```bash
which python3                                   # 必须是 /usr/bin/python3
python3 -c "import rclpy, numpy; print('OK', numpy.__version__)"   # 必须是 3.10 的 numpy 1.x
grep -n "vision\|hzmirmvision" ~/.bashrc        # 若有旧的 vision 工作区 source 行，建议删掉（会污染 AMENT_PREFIX_PATH）
```

**② Nav2 主体包必须安装**（否则组件与 RViz 面板全部加载失败）
```bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins
ls /opt/ros/humble/share | grep -c "^nav2"      # 期望 20+（只装基础件时只有 7）
```
另外 teb 编译依赖 `ros-humble-dwb-critics`（若尚未安装：`sudo apt install -y ros-humble-dwb-critics`）。

**③ 不要激活仓库里的 `.venv`**
`.venv/` 是工具链（MCP 等）用的隔离 Python 环境（`include-system-site-packages = false`），**与 ROS 无关**：激活后 `which python3` 会指向 `.venv/bin/python3`，`import numpy / rclpy` 全部失败。
```bash
deactivate                    # 若提示符是 (.venv)，先退出
which python3                 # 必须是 /usr/bin/python3，而不是 .../.venv/bin/python3
```

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

### 0.4 看哪块 RViz（别把两个都关掉）

| 你想看什么 | 用哪个 | 打开的配置 | 里面有什么 |
|---|---|---|---|
| **LIO 点云 / 建图效果**（mapping 模式） | `lio_rviz:=True` | `rm_nav_bringup/rviz/fastlio.rviz` / `pointlio.rviz` | 原始点云、`cloud_registered`、体素地图 |
| **导航效果**（nav 模式，**推荐**） | `nav_rviz:=True`（默认） | `rm_navigation/rviz/nav2.rviz` | `RobotModel`(`/robot_description`)、`Map`(`/map`)、本地/全局代价地图、`/scan`、TF、**Navigation2 面板**、**2D Pose Estimate / 2D Goal Pose 工具**（Fixed Frame = `map`） |

- ⚠️ **两个都给 `False` = 屏幕上什么可视化都没有**（Gazebo 里还有机器人，但没法判断导航效果）；
- 两个都给 `True` = 两个 RViz 同时抢 GPU，RMUL2026 这种重场景容易卡死/闪退，**只开一个**；
- 定目标/给初值：nav 模式下直接用 `nav2.rviz` 的工具栏按钮，比命令行方便。

### 0.5 三个场地的地图资产现状（`rm_nav_bringup/map/`）

| world | 栅格地图 | `origin` / 尺寸 | map 系约定 | **AMCL 初值（map 系）** | 可用重定位 | 备注 |
|---|---|---|---|---|---|---|
| `RMUC` | `RMUC.pgm` 577×301 | `(-6.35,-7.6)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) / ICP(`.data`) | 最全 |
| `RMUL` | `RMUL.pgm` 272×210 | `(-3.75,-4.54)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) | |
| `RMUL2026` | `RMUL2026.pgm` 240×169 | `(2.68, 0.228)` | **世界系** ⚠️ 与上两者不同 | `(4.3, 3.35, 0)` | **仅 AMCL** | 无 `.posegraph`、无 `.pcd`；`RMUL2026.pbstream` 仅 526 B（空，cartographer 纯定位不可用） |

#### 怎么判定出来的（别再靠猜）

用**场地 STL 的世界包围盒**与 **pgm 已知区域（非 205 像素）包围盒**比对，尺寸与位置两项都对上才算数：

| world | 场地 mesh 世界包围盒 | pgm 已知区域 bbox | 判定 |
|---|---|---|---|
| `RMUC` | x[0.00, 29.20] y[0.00, 15.20] | x[-6.35, 22.50] y[-7.60, 7.45] | mesh − spawn(6.35,7.6) = x[-6.35,22.85] y[-7.60,7.60] ≈ pgm → **出生点系** |
| `RMUL` | x[0.51, 14.23] y[-1.25, 9.30] | x[-3.75, 9.80] y[-4.49, 5.96] | mesh − spawn(4.30,3.35) = x[-3.79,9.93] y[-4.60,5.95] ≈ pgm → **出生点系** |
| `RMUL2026` | x[2.20, 14.80] y[0.20, 8.80] | x[2.68, 14.68] y[0.23, 8.43] | mesh **直接**等于 pgm（无 spawn 偏移）→ **世界系** |

推论与注意事项：
1. **`RMUL`/`RMUC` 的 pgm 是 sim 建图导出**（slam_toolbox 的 map 原点 = LIO 起点 = 机器人出生点），所以地图坐标里机器人在 `(0,0)`。之前按世界出生点 `(4.3,3.35)` 给 AMCL 初值是**错的**，会把机器人放到地图里偏差 5.4 m 的位置——这与你「rviz 看不出来、判断不了效果」直接相关。
2. `RMUL`/`RMUC` 的 `.posegraph`（slam_toolbox 纯定位的 `map_start_pose`）与 `.data`/pcd（ICP 的 `initial_pose`）本来就是 `[0,0,0]`，与 ① 一致 → **这两套资产内部自洽，不要动**。
3. ⚠️ **`RMUL2026` 是唯一的例外**：pgm 是世界系，而它没有 posegraph/pcd。将来若用 sim 建图给 RMUL2026 生成 `.posegraph`/`.pcd`，那些产物会变成**出生点系**，与现有 pgm 差 `(4.3, 3.35)` → 三种重定位模式会各自用不同的 map 系。**建议：做一次 RMUL2026 建图，用 `tools/scripts/mapping/save_grid_map.sh` 重新导出 pgm+posegraph，让该场地也统一成出生点系**（届时把 `bringup_sim.launch.py` 里 `amcl_init_x/y` 的 RMUL2026 也改成 `0.0`）。
4. 当前初值由 `bringup_sim.launch.py` 按 `world` **自动注入**（`amcl.ros__parameters.initial_pose.*` + `set_initial_pose: true`），无需手动发 `/initialpose`；运行中仍可用 `/initialpose` 或 RViz 的 `2D Pose Estimate` 覆盖。


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
ros2 topic echo /map --once --field info     # 期望：272 X 210（map_server 已激活）
ros2 action list | grep navigate_to_pose
```

> ✅ **初值已自动化**：`amcl` 参数里 `set_initial_pose: true`，初值由 `bringup_sim` 按 `world` 注入
> （`RMUC/RMUL → (0,0,0)`，`RMUL2026 → (4.3,3.35)`，依据见 §0.5），**不需要再手敲 `/initialpose`**。
> 想手动纠正（例如机器人被撞偏了）：RViz 工具栏 **"2D Pose Estimate"**，或
> ```bash
> # 以 RMUL 为例：map 系原点就是出生点，所以给 (0,0)
> ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
> "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.0685,0,0,0, 0,0,0,0,0.0685,0, 0,0,0,0,0,0.0685, 0,0,0,0,0,0]}}"
> ```
>
> 📌 若在启动早期手动发 `/initialpose`，可能看到
> `amcl: Failed to transform initial pose in time (Lookup would require extrapolation into the future ...)` —
> 位姿时间戳取 `now()`，而 10 Hz 的 `odom→base_link` 最新帧落后几十毫秒，tf2 不外推未来。
> **可忽略**（下一行仍会 `Setting pose (...)`）；想彻底没有这条告警，等 `tf2_echo odom base_link` 能持续输出后再发。

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

### 9.1 「导航到底就绪没有」一把梭（在终端 B 依次跑）

```bash
ros2 node list | grep -E "amcl|map_server|controller_server|planner_server"
ros2 lifecycle get /controller_server; ros2 lifecycle get /planner_server     # 期望 active
ros2 topic echo /map --once --field info                                     # 期望 宽x高 与你选的场地一致
ros2 topic hz /scan                                                          # ★ 期望稳定有频率（关键！见 9.2）
ros2 run tf2_ros tf2_echo odom base_link                                     # LIO 里程计
ros2 run tf2_ros tf2_echo map odom                                           # ★ 重定位模块输出
ros2 topic echo /amcl_pose --once                                            # amcl 模式下：position 应≈初值
ros2 topic info /scan --verbose                                              # 看订阅者与其 QoS（排查 incompatible QoS）
```

### 9.2 ⚠️ `map→odom` 依赖 `/scan`（源码级结论，实测确认）

`nav2_amcl` 里 `sendMapToOdomTransform()`（`amcl_node.cpp` L1007）**只被 `laserReceived()` 调用**（L716/L728），
且开头 `if (!initial_pose_is_known_) return;`。所以：

> **amcl 必须同时满足 ①已加载地图 ②已知初始位姿 ③收到 `/scan`，才会发布 `map→odom`。三条缺一，`map` 帧就根本不存在。**

沙箱无头实测（只跑 map_server+amcl，无 Gazebo）：

| 条件 | `tf2_echo map odom` |
|---|---|
| 有地图 + 自动初值，**无 `/scan`** | `Invalid frame ID "map" ... frame does not exist` ← 与"看起来没反应"完全一致 |
| 再补上 `/scan` | 立刻输出 `map→odom = 平移(4.3, 3.35)`，`/amcl_pose` = `x:4.30 y:3.35` ✅ |

所以出现 `map` 帧不存在时，**先查 `ros2 topic hz /scan`**，而不是怀疑初值。

### 9.3 常规判据

| 检查 | 正常表现 |
|---|---|
| `tf2_echo odom base_link` | 持续输出、随车移动，无 `Invalid frame ID` |
| `tf2_echo map odom`（nav 模式） | 由所选重定位模块提供（amcl / slam_toolbox / icp）；**前提见 9.2** |
| `topic hz /odom` | fastlio ≈10Hz；pointlio 更高（数十~100Hz） |
| `view_frames` | 主链 `map→odom→base_link→base_link_fake→…`、`base_link→livox_frame…`；`camera_init→body` 为孤岛（正常） |
| `lifecycle get /controller_server` | `active` |
| `/map` 的 `info` | 宽高 = 对应场地的 pgm 尺寸（map_server 加载**磁盘上的既有地图**，不是实时扫描结果） |


---

## 10. 错误对照

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'rm_nav_bringup' not found ... searching: ['/opt/ros/humble']` | 新终端**只 source 了 /opt/ros，没 source 工作区** | `source ~/HzMi_rmsimulation/install/setup.bash`（见 §0.1；可写进 `~/.bashrc`） |
| `view_frames` 里看到 `camera_init→body` 孤立小岛 | LIO 仍广播内部帧；T3 后导航层不再使用（`odom→base_link` 由 `lio_tf_adapter` 提供） | 正常现象，无需处理（回退用法 `localization:=''` 仍依赖它；T6 阶段可一并移除） |
| `Robot is out of bounds of the costmap!`（仅启动时出现几次） | slam_toolbox 地图尚在生长、global_costmap 正在 resize 的瞬态 | 若**持续刷屏**再排查 `map→odom`（`ros2 run tf2_ros tf2_echo map base_link`） |
| RViz 里**车/雷达看起来是斜的**，但 Gazebo 里车是正的 | spawn 高度过高：RMUL/RMUL2026 原来写 `z=1.16`，而地面在 z≈0（轮半径 0.06 → 落地时 base_link 仅 0.06 m），机器人**悬空 1.1 m 落下**，FAST-LIO 在坠落中做重力初始化 → 地图/位姿倾斜 | **已于 2026-09 修复**：`rm_simulation.launch.py` 中 RMUL / RMUL2026 的 spawn `z` 改为 **0.2**。验证：`ros2 run tf2_ros tf2_echo odom base_link` 的 roll/pitch 应≈0 |
| nav 模式卡在 `amcl: Waiting for map....` / `global_costmap: Invalid frame ID "map"` | ① `map_server_launch.py` 与 `localization_amcl_launch.py` **各起了一个同名 `lifecycle_manager_localization`**（冲突）；② `nav2_params_sim_*.yaml` 里 `yaml_filename` 被注释掉，而 nav2 的 `RewrittenYaml` **只替换已存在的键** → map_server 报 `parameter 'yaml_filename' is not initialized` | **已于 2026-09 修复**：① amcl launch 现在用**单一 lifecycle_manager 同时管理 `map_server`+`amcl`**，bringup 仅在 `icp`/未选重定位时单独起 map_server；② 三份 nav2 参数恢复 `yaml_filename: ""` 键（launch 会注入实际地图路径） |
| Nav2 在 `odom`/`map` 出现前就激活，刷 `Timed out waiting for transform ...` | Gazebo 生成机器人 + LIO 初始化需要数秒，而 Nav2 立即启动 | **已于 2026-09 缓解**：`bringup_sim` 中定位链延后 **4s**、mapping 后端延后 **4s**、Nav2 延后 **10s** 启动 |
| `spawn_entity: Spawn status: ... timed out waiting for entity to appear` | RMUL2026 世界加载慢，spawn 默认超时过短（实体其实已生成） | **已于 2026-09 修复**：spawn 参数加 `-timeout 60.0` |
| `Timed out waiting for transform from base_link to map` | `map→odom` 缺失 | nav 模式必须指定 `localization:=amcl\|slam_toolbox\|icp` |
| `Invalid frame ID "base_link"` / fake_vel 报 `Could not transform odom to base_link` | `/odom` 无数据 → LIO 或 `lio_tf_adapter` 未启动 | 查终端 A 是否打印 `lio_tf_adapter 启动` |
| `Found two parents` / 帧树分叉 | 旧进程残留 | 执行 §0.2 清理后重跑 |
| `nav:=teb` 插件找不到 | 未 source 工作区 | `source install/setup.bash`（teb 已编译） |
| `TF_OLD_DATA` | 仿真时间不一致 | 确认 `use_sim_time:=True`（launch 默认） |
| `Unable to parse the value of parameter robot_description as yaml` | launch_ros 把 URDF(XML) 当 YAML 解析（Humble 行为） | **已于 2026-09 修复**：`hzmi_rm_simulation/launch/rm_simulation.launch.py` 用 `ParameterValue(robot_description, value_type=str)` 包裹；若仍出现，说明用的是修复前的 launch |
| `ModuleNotFoundError: No module named 'numpy'` 且 `spawn_entity` 退出 | 用了 conda 的 python（ROS Humble 需系统 python 3.10） | 见 §0.0①：`conda deactivate` 或 `export PATH=/usr/bin:$PATH` |
| `Could not find requested resource in ament index`（nav2 组件成批加载失败） | Nav2 主体包未安装 | 见 §0.0②：apt 安装 navigation2 / nav2-bringup / nav2-rviz-plugins |
| RViz 报 `nav2_rviz_plugins/... does not exist` | 缺 `nav2-rviz-plugins` | 同上 |
| `New subscription discovered on topic '/scan', requesting incompatible QoS` | ① costmap `obstacle_layer` 默认 reliable，而 `/scan` 是 best-effort；②**残留的上一轮进程/RViz** 用旧参数（reliable）订阅 | ① **已于 2026-09 修复**：三份 `nav2_params_sim_{rpp,dwb,teb}.yaml` 的 scan 源加了 `reliability_policy/qos_policy: best_effort`；② 跑之前先执行 §0.2 清理；③ 用 `ros2 topic info /scan --verbose` 看清订阅者，本仓库内 `nav2.rviz` 对 `/scan`、`/particle_cloud` 都声明的是 **Best Effort**（与 best-effort 发布方兼容），这条告警多为 RViz 启动瞬间的瞬态，不影响链路 |
| `tf2_echo map odom` 报 `Invalid frame ID "map" ... frame does not exist`，但 `/map` 有数据 | **amcl 还没发布 `map→odom`**：缺 `/scan`、或初值未知、或地图未收到 —— 三者任一都会导致 `map` 帧不存在（源码级解释与实测见 §9.2） | 先 `ros2 topic hz /scan`；再 `ros2 param get /amcl set_initial_pose`；再确认 amcl 日志有 `Received a W X H map` 与 `Setting pose (...)` |
| **RViz 里一开始就有很完整的地图**，不是慢慢扫出来的 | nav 模式下那是 `map_server` 加载的**磁盘既有 pgm**（`src/rm_nav_bringup/map/<world>.pgm`），不是实时建图 | 正常。想看实时建图用 `mode:=mapping`（slam_toolbox/cartographer）；nav 模式要判断定位对不对，看 **scan/costmap 与这张地图的墙体是否重合** |
| costmap 刷屏 `Sensor origin at (x,y) is out of map bounds`（数值在 spawn 坐标与 LIO 估计间跳变） | **同一 `odom` 有两个来源**：Gazebo 真值 + LIO 都发到 `/odom`，`lio_tf_adapter` 交替收到两者 → `odom→base_link` 抖动 | **已于 2026-09 修复**：`sentry_robot_sim.xacro` 把 Gazebo 真值 remap 到 **`/odom_ground_truth`**（`publish_odom_tf=false`），`/odom` 由 LIO 独占。对比真值请看 `/odom_ground_truth` |
| RViz 报 `Message Filter dropping message ... queue is full` 且退出时 `rviz2 exit code -11` | RViz 在高频 TF/点云负载下丢帧，退出时崩溃（常见现象，不影响仿真链路） | 可先 `nav_rviz:=False` 验证导航链路；或减少 RViz 中 PointCloud2/STVL 显示项 |
| FAST-LIO 在 `Ctrl+C` 时报 `exit code -11` | FAST-LIO 已知的退出崩溃 | 忽略；不影响运行期 |
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

---

## 12. 实测记录

| 日期 | 场景 | 命令要点 | 结果 |
|---|---|---|---|
| 2026-09 | 一 mapping+fastlio | `mode:=mapping lio:=fastlio` | ✅ `/odom` 连续、`odom→base_link` 连续；曾修：Gazebo 里程计抢 `/odom`（remap 到 `/odom_ground_truth`）、出生 z=1.16 自由落体致 RViz 车体倾斜（改 z=0.2） |
| 2026-09 | 二 nav+amcl | `mode:=nav localization:=amcl nav:=rpp` | ✅ `Managed nodes are active`；`map_server 272×210` → `amcl Received a 272 X 210 map`。启动期约 4s 刷 `Timed out waiting for transform from base_link_fake to map` 与 `extrapolation` 属**瞬态**（TF 各帧刚建立、10Hz LIO TF 略滞后于 `now()`），给完 `/initialpose` 后自行恢复，不影响激活 |
| 2026-09 | 二 关机 | Ctrl-C | `fastlio_mapping` / `component_container_mt` 退出码 **-11** 属 Humble 关机期已知现象（进程已 Deactivate/Cleanup，非运行期崩溃） |
| 2026-09 | 二 地图系排查 | `world:=RMUL` 下按「世界出生点」给 AMCL 初值 | ❌ **判断错误已修正**：RMUL/RMUC 的 pgm 是**出生点系**（初值必须 `(0,0,0)`），只有 RMUL2026 是**世界系**（`(4.3,3.35)`）。判定方法与证据见 §0.5；已改为 launch 按 `world` 自动注入初值 |
| 2026-09 | 无头 amcl 单测（砂箱，无 Gazebo/RViz） | 只起 map_server+amcl，注入 `initial_pose_x/y=4.3/3.35` | ✅ **自动初值生效**（`/amcl_pose` = 4.30, 3.35）；且证明 **无 `/scan` 时 `map` 帧根本不存在**，补上 `/scan` 后立刻出现 `map→odom`（见 §9.2） |

