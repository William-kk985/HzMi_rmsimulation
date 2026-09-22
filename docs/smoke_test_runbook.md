# 仿真运行验证手册（Smoke Test Runbook）

> 用途：用**原生 `ros2 launch` 指令**逐场景验证仿真链路（含 T1–T5 TF/话题契约改造后的效果）。
> 配套文档：`docs/algorithm_matrix.md`（**算法现状唯一真值来源**：槽位/资产/实测状态）、
> `docs/architecture.md`（目录架构与概念 §3.2.x）、`docs/issues_and_findings.md`（坑与根因汇总）、
> `docs/3d_to_2d_survey.md`（3D→2D 各家实现对照）、`docs/tf_interface_contract.md`（TF 契约）、
> `docs/params_ownership_checklist.md`（参数归属）。
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
| `mode` | **`mapping`（纯建图）/ `slam_nav`（边建边导）/ `nav`（先建后导）** | 空（**必填**） |
| `lio` | `fastlio` / `pointlio` / `none` / `cartographer`（全包，见 §6.1） | `fastlio` |
| `localization` | `amcl` / `slam_toolbox` / `icp` / `cartographer`（**仅 `mode:=nav`** 生效） | 空 |
| `nav` | `rpp` / `dwb` / `teb`（`nav` / `slam_nav` 生效） | `rpp` |
| `mapper` | `slam_toolbox` / `cartographer`（`mapping` / `slam_nav` 生效） | `slam_toolbox` |
| `lio_rviz` / `nav_rviz` | `True` / `False` | `False` / `True` |
| `spin_speed` | 任意（rad/s） | `5.0` |
| **`global_obstacle`** | `stvl`（3D 体素层）/ `scan`（2D，与 local 同源）/ `none` | `stvl` |
| **`local_obstacle`** | `scan`（`/scan`，原行为）/ `cloud`（3D 点云直投，不经 `p2l`）/ `both`（双源冗余） | `scan` |

### 0.4 看哪块 RViz（别把两个都关掉）

| 你想看什么 | 用哪个 | 打开的配置 | 里面有什么 |
|---|---|---|---|
| **LIO 点云**（任何模式） | `lio_rviz:=True` | `rm_nav_bringup/rviz/fastlio.rviz` / `pointlio.rviz` | 原始点云、`cloud_registered`、体素地图 |
| **建图 / 导航效果**（**推荐**） | `nav_rviz:=True`（默认） | `rm_navigation/rviz/nav2.rviz` | `RobotModel`(`/robot_description`)、`Map`(`/map`)、本地/全局代价地图、`/scan`、TF、**Navigation2 面板**、**2D Pose Estimate / 2D Goal Pose 工具**（Fixed Frame = `map`） |

- `mode:=mapping`（纯建图，不起 nav2）下 `nav_rviz:=True` 也会给一块 RViz（2026-09 起 bringup 单独补的），能看 `/map` 边建边长；
- ⚠️ **两个都给 `False` = 屏幕上什么可视化都没有**（Gazebo 里还有机器人，但没法判断导航效果）；
- 两个都给 `True` = 两个 RViz 同时抢 GPU，RMUL2026 这种重场景容易卡死/闪退，**只开一个**；
- 定目标/给初值：nav 模式下直接用 `nav2.rviz` 的工具栏按钮，比命令行方便。

### 0.4.1 `spin_speed`：小陀螺在仿真里的陷阱

`fake_vel_transform` 把 nav2 的角速度指令**替换成固定角速度** `spin_speed`（`/cmd_vel` 里角速度非零 → `/cmd_vel_chassis` 就用 `spin_speed`）。

- **真实哨兵**：电控本来就让底盘持续自转，nav2 的角速度只是"增减"信号，云台机械补偿保证雷达朝向稳定；
- **仿真**：`planar_move` 没有基线自转，任何一点角速度修正都会让底盘以 **5 rad/s（≈286°/s）** 旋转，而**仿真里没有云台补偿，雷达跟着底盘一起转** → 10 Hz 的 FAST-LIO 每帧要承受约 29° 旋转，跟踪容易退化，进而"控制器看不到进展 → 进恢复行为 → 越转越糟"。

所以排查导航问题时**先用 `spin_speed:=0.0`**（角速度直通，等价普通 nav2），确认基础导航链路通了之后，再打开 `5.0` 做小陀螺对比实验。日志判据：`/cmd_vel_chassis` 里出现 `angular.z: 5.0` 就说明小陀螺在动作（那是 `spin_speed` 的值，不是 nav2 发的角速度）。

### 0.5 三个场地的地图资产现状（`rm_nav_bringup/map/`）

| world | 栅格地图 | `origin` / 尺寸 | map 系约定 | **AMCL 初值（map 系）** | 可用重定位 | 备注 |
|---|---|---|---|---|---|---|
| `RMUC` | `RMUC.pgm` 577×301 | `(-6.35,-7.6)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) / ICP(`.data`) | 最全 |
| `RMUL` | `RMUL.pgm` 272×210 | `(-3.75,-4.54)` | **出生点系** | `(0, 0, 0)` | AMCL / slam_toolbox(`.posegraph`) | |
| `RMUL2026` | `RMUL2026.pgm` 240×169 | `(2.68, 0.228)` | **世界系** ⚠️ 与上两者不同 | `(4.3, 3.35, 0)` | **仅 AMCL** | 无 `.posegraph`、无 `.pcd`；`RMUL2026.pbstream` 仅 526 B（空，cartographer 纯定位不可用）。⚠️ 用 cartographer/slam_toolbox **重建**后新图变**出生点系**（初值改 `(0,0,0)`）；新旧图差 `(4.3,3.35)`，**不可混用**（§5/§6） |

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


### 0.6 ⚠️ 选测试目标点：**先验连通域，别靠眼估**

nav2 全局规划把「障碍 + `robot_radius` 内切膨胀带」视为不可通行，**地图上 1 像素（5cm）宽的虚线就足以把一条走廊彻底封死** → `planner_server: failed to generate a valid path` → BT 反复跑恢复行为（`/cmd_vel` 只会出现 `-0.05` 的 BackUp 和 `spin_speed` 的旋转）。

```bash
# 检查"某个目标点从起点是否可达"（最常用）
/usr/bin/python3 tools/check_map_reachable.py --map src/rm_nav_bringup/map/RMUL2026.yaml \
  --start 4.3 3.35 --goal 6.0 3.4
# 让工具推荐一批可达且离墙够远的目标点
/usr/bin/python3 tools/check_map_reachable.py --map src/rm_nav_bringup/map/RMUL2026.yaml --start 4.3 3.35
```
> `--start` 用**该场地 map 系**的起点：`RMUC/RMUL` 是 `0 0`，`RMUL2026` 是 `4.3 3.35`（依据见 §0.5）。
> 工具还会报"地图被切成几块"——切成多块时，目标点跨块必然规划失败。

**RMUL2026 的实测结论（重要）**：该场地被切成 3 块（#2=24246 格 / #1=3692 格 / #0=693 格），
起点在 #2，而 **`(6.0, 3.4)` 落在 #1 → 必然规划失败**。原因是 `RMUL2026.pgm` 里存在一道 **x≈5.2 的竖直虚线（1 像素宽、带小缺口，从 y≈1.8 延伸到 6.0）**，
而 **RMUL2026 世界网格在该处完全没有几何**（逐顶点核对：x∈[5.05,5.35]、y∈[2.8,4.0] 的顶点数 = 0）——
即 **pgm 里有世界不存在的"幽灵墙"**，膨胀后把走廊封死。这与 §0.5 的结论一致：`RMUL2026.pgm` 更像**官方场地平面图**（含分区虚线），而不是在 sim 里建出来的图。

**因此：**
1. 立刻验证导航链路 → 用**同一连通域**的目标点，例如 `(4.01, 4.70)`（1.38m）、`(3.91, 5.40)`（2.09m）、`(8.26, 5.45)`（4.48m，会绕行，可验证长路径）；
2. 彻底解决 → **在 sim 里重新给 RMUL2026 建一次图**（`mode:=mapping` + `tools/scripts/mapping/save_grid_map.sh`），
   得到与世界一致、且与 RMUL/RMUC 统一成"出生点系"的 `pgm+posegraph`（同时替换掉 526 B 的空 `RMUL2026.pbstream`），
   之后把 `bringup_sim.launch.py` 里 RMUL2026 的 `amcl_init_x/y` 改回 `0.0`。

---

### 0.7 三种场景形态（`mode`）各起什么 —— 别混用

| `mode` | 中文 | Gazebo+LIO | 在线 SLAM 后端 | 导航栈 nav2 | `map_server` | 重定位模块 | `map→odom` 来源 |
|---|---|---|---|---|---|---|---|
| `mapping` | **纯建图** | ✅ | ✅ `mapper:=slam_toolbox\|cartographer` | ❌ **不启动** | ❌ | ❌ | 在线 SLAM（离线保存用） |
| `slam_nav` | **边建图边导航** | ✅ | ✅ 同上 | ✅ | ❌ | ❌ | 在线 SLAM 直接喂 costmap |
| `nav` | **先建图后导航** | ✅ | ❌ | ✅ | 仅 `localization:=icp` 或留空时 | `localization:=amcl\|slam_toolbox\|icp` | 所选重定位模块 |

要点：
- `mapping` 不启动 nav2（2026-09 起）：省下 7 个 nav2 节点 + 两张 costmap，重场景下建图帧率明显更稳；
- `slam_nav` 与 `nav` 的区别**只在"地图从哪来"**：前者是在线 SLAM（含回环优化，`map→odom` 会随优化跳变），后者是磁盘地图 + 重定位（`map→odom` 由重定位模块平滑给出）；
- `localization` 只在 `mode:=nav` 生效（`slam_nav`/`mapping` 传了也会被忽略）；
- `mode:=nav` 留空 `localization` 是**回退用法**：直接用 LIO 当绝对定位，并由 `camera_init→map`、`body→odom` 两条静态桥补帧（只在 `mode=='nav' and localization==''` 时启动）；
- 三种形态的 `/map` 发布者都只有一个（在线 SLAM 或 map_server），2026-09 已修掉建图模式下 `map_server` 抢 `/map` 的问题（见 §10）。

### 0.8 文档地图 & 当前推荐顺序

| 文档 | 用途 |
|---|---|
| `docs/smoke_test_runbook.md` | **本文件**：怎么跑、怎么判、错了怎么查 |
| `docs/algorithm_matrix.md` | **算法现状唯一真值来源**：有哪些算法/组合、资产齐不齐、实测状态、阻塞项 |
| `docs/architecture.md` | 目录/分层/参数/概念（§3.2.1–3.2.7 是概念长文） |
| `docs/glossary.md` | **术语表**：名词一句话定义 + 指向详述（子图/回环/ESDF/2.5D/降维/代价…） |
| `docs/glossary.md` | **术语表**：名词一句话定义 + 指向详述（子图/回环/ESDF/2.5D/降维/代价…） |
| `docs/issues_and_findings.md` | 踩过的坑与根因汇总（含 RMUL2026 幽灵墙证据链） |
| `docs/tf_interface_contract.md` | TF/接口契约（T1–T5 已实施，T6 撤销） |

**当前推荐顺序**（按资产齐备度排，先解开阻塞再看矩阵）：
1. **重建 RMUL2026 地图**（§1 建图 + §1.1 落盘三件套）← 解开该场地**所有** nav 组合；
2. `nav + amcl` @ RMUL2026 发目标（§2 + §0.6 选点）；
3. `nav + slam_toolbox` @ RMUL（§4）与 `nav + icp` @ RMUL（§3）；
4. `slam_nav` 边建边导（§1.2）；5. cartographer 建图/纯定位（§5/§6，纯定位需先生成 pbstream）；6. `nav:=dwb|teb`（§7）。

---

### 0.9 5 分钟上手（TL;DR）

```bash
# 0) 环境 + 清理（见 §0.1 / §0.2）
cd ~/HzMi_rmsimulation && source /opt/ros/humble/setup.bash && source install/setup.bash
pkill -f gzserver; pkill -f gzclient; pkill -f rviz2; pkill -f component_container; sleep 2

# 1) 起仿真（资产最全、最稳的组合：先建后导 + AMCL）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0

# 2) 等 30~60 s（RMUL 世界重），另开终端跑就绪检查（§9.1 是完整版）
ros2 lifecycle get /controller_server      # 期望 active
ros2 run tf2_ros tf2_echo map odom         # 有输出（≈AMCL 初值）
ros2 topic hz /scan                        # 有稳定频率

# 3) 先验可达性再发目标（§0.6，别靠眼估）
/usr/bin/python3 tools/check_map_reachable.py --map src/rm_nav_bringup/map/RMUL.yaml --start 0 0 --goal 1.68 3.44
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.68, y: 3.44, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```
**判据**：`/cmd_vel` 出现**正向** `linear.x`、Gazebo 里车在动、RViz 里 scan 与地图墙体重合。
**不通时**：先按 §10.1「现象 → 归属」分类，再查 §10 具体行。

### 0.11 启动参数总表（`bringup_sim.launch.py`，全部原样可用）

| 参数 | 取值 | 默认 | 说明 |
|---|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026` | `RMUL2026` | 场地；同时决定 `map/<world>.*` 与 `PCD/<world>.pcd` 前缀 |
| `mode` | `mapping` / `slam_nav` / `nav` | 空（**必填**） | 场景形态，决定启动哪套节点集（§0.7） |
| `lio` | `fastlio` / `pointlio` / `none` / `cartographer` | `fastlio` | 里程计实现；`none` 需外部提供 odom/TF；`cartographer` = 全包形态（兼任里程计源，见 §6.1） |
| `localization` | `amcl` / `slam_toolbox` / `icp` / `cartographer` / 空 | 空 | **仅 `mode:=nav` 生效**；空 = 回退用法（LIO 当绝对定位 + 静态桥） |
| `mapper` | `slam_toolbox` / `cartographer` | `slam_toolbox` | 在线 2D 建图后端；`mapping`/`slam_nav` 生效 |
| `nav` | `rpp` / `dwb` / `teb` | `rpp` | 局部规划器变体；`nav`/`slam_nav` 生效 |
| **`global_obstacle`** | `stvl` / `scan` / `none` | `stvl` | 全局代价地图的实时障碍来源（A/B 槽位，见 `docs/3d_to_2d_survey.md` §七） |
| **`local_obstacle`** | `scan` / `cloud` / `both` | `scan` | 局部代价地图障碍来源（A/B 槽位，§7.2）：`cloud` 用点云直投破 `p2l` 单点并消 45cm 盲区；`both` = 双源冗余 |
| `spin_speed` | 任意（rad/s） | `5.0` | `fake_vel_transform` 小陀螺固定角速度；排查导航先用 `0.0`（§0.4.1） |
| `lio_rviz` | `True` / `False` | `False` | 开 LIO 点云 RViz |
| `nav_rviz` | `True` / `False` | `True` | 开 nav2 RViz（`mode:=mapping` 也会给一块） |
| `use_sim_time` | `True` / `False` | `True` | 仿真是 `True` |

> 另有两个**只对 cartographer 生效**的参数（`cartographer_sim.launch.py`）：
> `load_state_filename`（pbstream 路径，留空=纯建图）与 `load_frozen_state`，以及
> `occupancy_grid_topic`（栅格输出话题，默认 `map`；纯定位另有 map_server 时传 `/cartographer_map`）。

### 0.10 场景 × 资产前提 × 命令（总览）

| 场景 | **必需资产** | 命令要点 | 关键判据 |
|---|---|---|---|
| §1 纯建图 | 无（从零） | `mode:=mapping mapper:=slam_toolbox\|cartographer` | `/map` 只有 1 个发布者且边长；落盘见 §1.1 |
| §1.2 边建图边导航 | 无 | `mode:=slam_nav mapper:=...` | `node list` **无** amcl/map_server；`map→odom` 由在线 SLAM |
| §2 nav+AMCL | `.pgm`+`.yaml` | `mode:=nav localization:=amcl` | `map→odom` 由 amcl；`/amcl_pose`≈注入初值 |
| §3 nav+ICP | **`PCD/<world>.pcd`**（仅 RMUC/RMUL） | `mode:=nav localization:=icp` | `map→odom` 由 `icp_registration`（T5） |
| §4 nav+slam_toolbox | **`.posegraph`**（仅 RMUC/RMUL） | `mode:=nav localization:=slam_toolbox` | `map→odom` 由 slam_toolbox；无 map_server/amcl |
| §5 cartographer 建图 | 无 | `mode:=mapping mapper:=cartographer` | `/map` 1 个发布者（cartographer occupancy_grid） |
| §6 cartographer 纯定位 | **`.pbstream`（三个场地都缺，需先用 §5 生成）** | `localization:=cartographer`（一键） | 加载状态成功、`map→odom` 由 cartographer、`/map` 仍是 map_server 的先验图 |
| §7 局部规划变体 | 同 §2 | `nav:=rpp\|dwb\|teb` | 三个都 active 且能发目标 |

> 资产盘点与"哪些组合真能跑"以 `docs/algorithm_matrix.md` §三/§四 为准。

---

## 1. 场景一：mapping + fastlio（建图 / 重建地图）

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=slam_toolbox spin_speed:=0.0
```
> `spin_speed:=0.0`：遥控建图时若底盘按 5 rad/s 跟着角速度指令转，雷达会一起转，LIO 容易退化（见 §0.4.1）。
> 建图模式（`mapping`）**既不起 `map_server`、也不起导航栈**（2026-09 起）：
> 前者会拿磁盘旧 pgm 抢 `/map`（`map_saver_cli` 可能存下旧图），后者白白吃掉 7 个 nav2 节点 + 两张 costmap 的 CPU。

**终端 B（检查）**
```bash
ros2 topic hz /odom                          # 期望 ≈10Hz（fastlio）
ros2 topic hz /livox/lidar                   # 期望 10Hz（CustomMsg，FAST-LIO 的输入）
ros2 topic echo /scan --once                 # 期望：一帧 LaserScan
ros2 run tf2_ros tf2_echo odom base_link     # 期望：持续输出，无 Invalid frame ID
ros2 topic echo /map --once --field info     # 期望：宽高随建图增长（只有 slam_toolbox 一个发布者）
```

**终端 C（遥控走遍场地）**
```bash
tools/scripts/control/improved_teleop.sh      # 键盘遥控（建议速度慢一点、覆盖整场、回到起点附近收尾）
```

**键位（2026-09-21 改成方向键）**：

| 键 | 作用 |
|---|---|
| `↑` / `↓` | 前进 / 后退（默认 **0.5 m/s**） |
| `←` / `→` | 左转 / 右转（默认 **1.0 rad/s**） |
| `空格` | **紧急停止** |
| `>` / `<` | 线速度 +0.1 / −0.1（0.1 ~ 2.0 m/s） |
| `.` / `,` | 角速度 +0.1 / −0.1（0.2 ~ 3.0 rad/s） |
| `0` | 速度重置为默认值 |
| `s` / `h` | 系统状态 / 帮助 |
| `q` | 退出（退出前自动发一次零速） |
| 兼容旧键 | `i/k/j/l` = 前进/后退/左转/右转；`[` / `]` = 角速度 ± |

> ⚠️ **按一下只发一次 Twist，底盘会保持该速度** → 停车必须按 `空格`（`q` 退出时也会自动停）。
> 默认发到 **`/cmd_vel_chassis`**（直连底盘，绕过 `fake_vel_transform`）；
> 想让指令走 nav2 那条链（经 `fake_vel_transform`、受 `spin_speed` 影响）时用：
> `TELEOP_TOPIC=/cmd_vel tools/scripts/control/improved_teleop.sh`
> 建图建议：线速 0.3–0.5 m/s、转向 0.5–0.8 rad/s，绕场一圈、回到起点附近收尾。

### 1.1 落盘三件套（.pgm/.yaml + .posegraph + .pcd）

> ⚠️ **先备份**：`save_grid_map.sh` 会**覆盖** `map/RMUL2026.{pgm,yaml}`。

```bash
cp src/rm_nav_bringup/map/RMUL2026.pgm  src/rm_nav_bringup/map/RMUL2026_official.pgm
cp src/rm_nav_bringup/map/RMUL2026.yaml src/rm_nav_bringup/map/RMUL2026_official.yaml

# ① 栅格地图（map_saver_cli -f src/rm_nav_bringup/map/<world>）
tools/scripts/mapping/save_grid_map.sh

# ② 位姿图（slam_toolbox 纯定位与 nav 模式 map_start_pose 都靠它）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026'}"

# ③ 点云底图（ICP 重定位用；路径由 bringup 按 world 自动设为 PCD/<world>.pcd）
ros2 service call /map_save std_srvs/srv/Trigger
ls -l src/rm_nav_bringup/map/RMUL2026.* src/rm_nav_bringup/PCD/RMUL2026.pcd
```

建完图后**务必做两件事**：
1. 用可达性工具确认全场连通、记下可用的测试目标点：
   `tools/check_map_reachable.py --map src/rm_nav_bringup/map/RMUL2026.yaml --start 0 0`
2. 新图是**出生点系**（与 RMUL/RMUC 一致）→ 把 `bringup_sim.launch.py` 里 `amcl_init_x/y` 的 **RMUL2026 也改成 `0.0`**
   （同时 `docs/smoke_test_runbook.md` §0.5 的表格要同步更新）。

### 1.1.1 另一条拿到 2D 地图的路线：LIO 点云切层投影

`FAST-LIO/Point-LIO` 本身就是 LiDAR-Inertial **SLAM**：跑里程计的同时维护着一张三 3D 点云地图，
`/map_save` 落盘的 `PCD/<world>.pcd`（例：RMUL 是 **159 万点**）就是它建的图。**它不只能做定位**。
把它沿 z 切一层投影，就能直接得到 nav2/AMCL 能吃的 2D 栅格图（**不依赖 slam_toolbox/cartographer**）：

```bash
/usr/bin/python3 tools/pcd_to_grid_map.py --pcd src/rm_nav_bringup/PCD/RMUL.pcd \
  --out src/rm_nav_bringup/map/RMUL_frompcd --min-z 0.10 --max-z 0.50 \
  --compare src/rm_nav_bringup/map/RMUL.yaml
```

实跑验证（RMUL.pcd → 2D 图 vs slam_toolbox 的 `map/RMUL.pgm`）：
- 两图**同一坐标系**（bbox 重合，最佳平移仅 1~2 格 = 5~10cm）；
- **±1 格容差下参考图的墙被覆盖 84.7%** → 场地结构一致；
- 严格 IoU 只有 0.21，因为 3D→2D 投影的墙更厚、且把切层内的立体结构都算进来了。

**这工具的第二个用途：独立核对地图资产。** 怀疑某张官方图有"世界不存在的墙"（§0.6 的幽灵墙）时，
用 LIO 的 pcd 投影一张同场地的图，一比就知道那道墙在**实际观测**里存不存在。
切层用 `--min-z/--max-z` 控制（贴近平面的 0.10~0.50 最接近 2D 雷达视角）；自由空间由起点洪泛得到，
所以**若墙有缺口会外漏**，必要时先看投影图的墙是否闭合。

### 1.2 边建图边导航（`mode:=slam_nav`）

与 §2 的「先建图后导航」是**两条不同的启动装配**（见 §0.7）：这里**不加载磁盘地图、不起任何重定位模块**，
costmap 的 `static_layer` 直接吃在线 SLAM 发布的 `/map`，`map→odom` 也由在线 SLAM 提供。

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=slam_nav lio:=fastlio mapper:=slam_toolbox nav:=rpp spin_speed:=0.0
```

```bash
ros2 topic info /map --verbose | grep -c "PUBLISHER"   # 期望 1（在线 SLAM，无 map_server）
ros2 run tf2_ros tf2_echo map odom                     # 期望：由 slam_toolbox 提供，随建图/回环缓慢变化
ros2 lifecycle get /controller_server                  # 期望 active（nav2 延后 10s 起，等 /map 与 map→odom）
ros2 node list | grep -E "amcl|map_server"             # 期望：**空**（这条形态不该有它们）
```

判据与注意：
- RViz 里地图**边建边长**，可以直接用工具栏 `2D Goal Pose` 发目标让车一边探索一边走；
- 未探索区域是 unknown，`GridBased.allow_unknown: true` 允许穿越未知区；
- **回环优化会让 `map→odom` 跳变**（SLAM 修正累积误差的正常行为），此时车在 map 里的位置会"瞬移"一下，costmap 随之更新 —— 这与 §2/§3 里重定位模块给出的平滑 `map→odom` 是本质区别。

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

**资产前提**：ICP 吃 **3D 点云图** `PCD/<world>.pcd` → 目前只有 **RMUC / RMUL** 有（RMUL2026 需先用 `/map_save` 生成）。

**终端 A**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=icp nav:=rpp spin_speed:=0.0
```
> ⚠️ **RMUL 世界重（44 万三角面）：启动后等 30~60 s 再判断**（ICP 节点本身延后 7 s 启动；
> 过早判断会看到 `odom` 帧尚不存在、Nav2 无法激活，误以为链路坏了）。

**终端 B（就绪判据）**
```bash
ros2 node list | grep icp_registration            # ICP 是否起来
ros2 topic hz /livox/lidar/pointcloud             # ★ ICP 的输入（必须持续有数据）
ros2 run tf2_ros tf2_echo odom base_link          # LIO 里程计；无 → LIO 未初始化
ros2 run tf2_ros tf2_echo map odom                # ★ 关键：应由 icp_registration 发布（T5）
ros2 topic echo /map --once --field info          # 静态图（RMUL = 272×210）
ros2 lifecycle get /controller_server             # active
```
**逐段排查**（哪段先没数据，问题就在那一段）：`/livox/lidar`(CustomMsg) → `/livox/imu` → `/imu/data` → `/odom` → `map→odom`。
若 `/livox/lidar` 有数据而 `/odom` 长时间没有 → 换 `lio:=pointlio` 做 A/B（它吃原始 `/livox/imu`），
以区分是"雷达/世界侧"还是"FAST-LIO + 互补滤波支路"的问题。

---

## 4. 场景四：nav + slam_toolbox（纯定位）

**资产前提**：`localization:=slam_toolbox` 需要 `.posegraph` → **RMUC / RMUL 有，RMUL2026 没有**（要先建图，见 §1.1）。

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL mode:=nav lio:=fastlio localization:=slam_toolbox nav:=rpp spin_speed:=0.0
```
```bash
ros2 node list | grep -E "map_server|amcl"   # 期望：空（这条形态不需要它们）
ros2 run tf2_ros tf2_echo map odom           # 由 slam_toolbox 提供
ros2 lifecycle get /controller_server        # active
```

---

## 5. 场景五：cartographer 建图

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=mapping lio:=fastlio mapper:=cartographer spin_speed:=0.0
```
> 建图模式**不起 nav2**，但 `nav_rviz:=True` 仍会给一块 RViz（能看 `/map` 边建边长）。
> 栅格话题 **2026-09 已修正为 `/map`**（原先硬 remap 到 `/cartographer_map`，会让 nav2 的 `static_layer`、
> `map_saver_cli`、RViz 的 Map 显示项全都吃不到图）。

> **cartographer 2D 三个 mode 的跑通顺序**（严格按序；中间产物不可跨次混用）：
>
> | 序 | 命令 | 通过判据 | 落盘 |
> |---|---|---|---|
> | ① | `mode:=mapping mapper:=cartographer`（本节） | `/map` 宽高随建图增长；`/scan` 的订阅者里有 `cartographer_node`；`tf2_echo map odom` 有输出 | **pgm + pbstream，必须同一次运行存** |
> | ② | 同①但 `mode:=slam_nav`（是否 `spin_speed:=0.0` 按 §0.4.1 决定） | nav2 起来；`ros2 topic info /map --verbose` 订阅者含 `global_costmap`；`ros2 lifecycle get /planner_server` = active；发目标能规划出路径 | 不落盘（边建边导） |
> | ③ | §6：`mode:=nav localization:=cartographer` | pbstream 加载成功；`map→odom` 由 cartographer 发；`/map` 来自 `map_server`；`/cartographer_map` 是 cartographer 自己那张 | — |
>
> ⚠️ **pgm 与 pbstream 必须来自同一次建图**：cartographer 的 `map` 系原点 = 建图时机器人的起点（**出生点系**），
> 所以它导出的 pgm 与 pbstream 天然同一套坐标；若拿**旧 pgm**（现有 `RMUL2026.pgm` 是**世界系**）配**新 pbstream**，
> 两者差一个常量平移（RMUL2026 是 `(4.3, 3.35)`）→ 现象是 RViz 里先验图整体偏移、costmap 与定位错位。

**终端 B**
```bash
ros2 node list | grep cartographer                    # cartographer_node + cartographer_occupancy_grid_node
ros2 topic info /scan --verbose | grep "Node name"    # ★ 必须看到 cartographer_node（证明输入类型/话题对上了）
ros2 topic info /map --verbose | grep -c PUBLISHER    # 期望 1（cartographer occupancy_grid）
ros2 run tf2_ros tf2_echo map odom                    # 有输出（由 cartographer 发）
python3 tools/monitor_map_odom.py                     # ★ 50Hz 采样 + 判读：平滑漂移 / 锯齿矫正 / 高频抖动
                                                      #   （tf2_echo 每 ~0.8s 一行，看不出高频抖动；判据见 tools/README.md）
ros2 topic echo /map --once --field info              # 宽高随建图增长
ros2 run tf2_tools view_frames                        # ★ 帧树：body 只能有一个父(camera_init)，odom 下只应有 base_link
```
> **2026-09 修了四处致命对接问题**（这就是仓库里 `RMUL2026.pbstream` 只有 526 B 的原因）：
> ① 输入类型：`points2` 原 remap 到 `/livox/lidar`（**CustomMsg**，cartographer_ros 不支持）→ 改为吃
> `/scan`（`num_laser_scans=1`）；② 帧契约：原 `published_frame="body"`+`provide_odom_frame=true`
> 会与 FAST-LIO 争 `body` 的子帧 → 改为 `published_frame="odom"`+`provide_odom_frame=false`（只发 `map→odom`）；
> ③ 高度带：`min_z/max_z` 是**相对传感器**的（不是相对地面），原 0.05~0.8 只切到墙顶 → 改回 -0.8~2.0；
> ④ **IMU 帧必须与 `tracking_frame` 重合**：`sensor_bridge.cpp:136` 有硬 CHECK（平移 < 1e-5 m），
> 不满足**直接 abort（exit -6）**。我们 URDF 的 `livox_frame` 与 `imu_link` 相差 5cm（该 5cm 是
> FAST-LIO `extrinsic_T` 依赖的，不能改 URDF）→ `tracking_frame` 由 `livox_frame` 改为 **`imu_link`**
> （官方 `mir-100-mapping.lua` 同做法）。同时 `TRAJECTORY_BUILDER_2D.min_range` 0.45→0.2（与 p2l 对齐）。

**落盘**（三件套都可用；字段名已核对）
```bash
tools/scripts/mapping/save_grid_map.sh                                            # /map -> map/<world>.pgm+yaml
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"
# 可选：直接导出子图栅格
ros2 service call /write_assets cartographer_ros_msgs/srv/WriteAssets "{stem: '/tmp/<world>_carto', image_format: 'png'}"
```
> 注：`cartographer_occupancy_grid_node` 的 `/map` 发布器是 `QoS(10).transient_local()`（已核对源码
> `occupancy_grid_node_main.cpp`）→ nav2 `static_layer`（默认 `map_subscribe_transient_local: true`）与
> `map_saver_cli` 都能收到，nav2 晚启动也不会漏图。

---

## 6. 场景六：cartographer 纯定位（`localization:=cartographer`，2026-09 接入槽位）

**资产前提**：需要 `map/<world>.pbstream`。先用**场景五**跑一次 cartographer 建图并 `finish_trajectory` +
`write_state` 生成（三个场地目前都还没有）。

**终端 A（一键，与 amcl/slam_toolbox/icp 同构）**
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  use_sim_time:=True lio_rviz:=False nav_rviz:=True \
  world:=RMUL2026 mode:=nav lio:=fastlio localization:=cartographer nav:=rpp spin_speed:=0.0
```
接线（槽位实现细节）：
- cartographer 加载 `map/<world>.pbstream`（`load_frozen_state:=true`）→ **只发 `map→odom`**（契约与 amcl/icp 一致）；
- `odom→base_link` 仍由 LIO 提供；
- 它的栅格发到 **`/cartographer_map`**，`/map` 留给 `map_server` 的先验栅格图（避免双发布者）；
- 纯定位默认以 `map` 原点为起始猜测；而 cartographer 的 `map` 系原点 = **建图时的起点**，所以
  **建图与定位两次的出生点应一致**（`RMUL` / `RMUL2026` 都是 world 出生点 `(4.3, 3.35)`，两次同点起步即天然吻合）。
  出生点不同时只能靠全局搜索（`POSE_GRAPH.global_constraint_search_after_n_seconds`）——慢且可能失败；
  此时可用 `start_trajectory` 服务显式给初值（该 srv 有 `use_initial_pose` + `initial_pose` 字段）。
- ⚠️ **不要混用坐标系**：现有 `RMUL2026.pgm` 是**世界系**，而 cartographer 导出的 pgm/pbstream 是**出生点系**。
  重建 RMUL2026 后，§0.5 表里的 AMCL 初值也要从 `(4.3, 3.35)` 改成 `(0, 0)`。

**终端 B**
```bash
ros2 node list | grep cartographer                    # cartographer_node + occupancy_grid_node
ros2 run tf2_ros tf2_echo map odom                    # 由 cartographer 提供
ros2 topic echo /map --once --field info              # 先验栅格图（map_server，宽高=场地尺寸）
ros2 topic echo /cartographer_map --once --field info # cartographer 自己的栅格（可视/对比用）
ros2 lifecycle get /controller_server                 # active
```

> 若想脱离 bringup 单独调试 cartographer，可仍用
> `ros2 launch rm_nav_bringup cartographer_sim.launch.py configuration_basename:=cartographer_localization.lua load_state_filename:=<abs>.pbstream load_frozen_state:=true`
> （但那样没有仿真/LIO/nav2，仅适合看 cartographer 自身日志）。

> ⚠️ **资产前提（当前阻塞）**：三个场地**都没有可用的 pbstream** —— RMUL/RMUC 没有该文件，
> `RMUL2026.pbstream` 仅 526 B（空壳）。**本场景必须先自己生成** —— 方法就在本场景下方
> （`/finish_trajectory` → `/write_state` 两条服务调用，无需任何脚本）。
> 纯定位时若另有 `map_server` 在发先验 `/map`，请加 `occupancy_grid_topic:=/cartographer_map`，
> 避免两个发布者抢 `/map`（该参数默认已改为 `map`）。

---

### 6.1 全包形态：`lio:=cartographer`（cartographer 兼任里程计源，2026-09 新增）

**它是什么**：`lio` 槽取 `cartographer` 时，同一个 `cartographer_node` 既发 `odom→base_link`
（`provide_odom_frame=true` + `published_frame="base_link"`），又发 `map→odom`。
于是 `mapper` 槽、`localization` 槽、`lio_tf_adapter`、T1 静态桥**全部跳过**（launch 已按 `lio` 门控）。

```bash
# 建图（不写 mapper，写了也会被跳过）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping \
  lio:=cartographer spin_speed:=0.0 nav_rviz:=True
# 边建边导
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
  lio:=cartographer nav:=rpp spin_speed:=0.0 nav_rviz:=True
# 纯定位（需 map/RMUL2026.pbstream；localization 必须留空）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=cartographer nav:=rpp spin_speed:=0.0 nav_rviz:=True
```

**验收**：
```bash
ros2 run tf2_ros tf2_echo odom base_link       # 由 cartographer 发（不是 lio_tf_adapter）
ros2 run tf2_ros tf2_echo map odom
ros2 node list | grep -c fastlio               # 期望 0
ros2 node list | grep lio_tf_adapter           # 期望空
```

**三个限制**（详见 `docs/tf_interface_contract.md` §八）：
① `odom` 由 cartographer 的 pose extrapolator 提供，弱于 FAST-LIO 的紧耦合 IEKF
   → **2026-09-21 已缓解**：全包形态现在接一路底盘里程计（`use_odometry=true`，仿真用
   `/odom_ground_truth`、实车用下位机轮速 odom），见下面那段；
② **没有 `/odom` 话题** → `nav:=teb` 不适用（用 `rpp`/`dwb`）；
③ 失去独立故障域：cartographer 挂了 `odom` 与 `map` 一起没。

**⚠️ 全包形态必须有 odom（2026-09-21 实测修复）**

不接 odom 时，`odom→base_link` 只能靠 pose extrapolator（IMU 二次积分），**静止也会漂**：

| 实测（RMUL2026，车静止） | 值 |
|---|---|
| `odom→base_link` 平移漂移 | x +3.1 cm/s、y −2.5 cm/s → 合 **≈4 cm/s** |
| `odom→base_link` yaw 漂移 | 2.21° → 1.72° / 2.3 s ≈ **13°/min** |
| `map→odom` | 恒定 `(0.088, 0.075, −0.37°)`（说明**不是回环问题**，位姿图没动） |
| `/odom_ground_truth` twist | 全 0（车确实没动） |

后果：漂移被当作扫描匹配的初始猜测 → 地图被拖着走（RViz 里"车自己在动、地图跟着小车走"）。

**修法**：给 cartographer 接一路底盘里程计。配置已在仓库里（`cartographer_lio*.lua` 里
`use_odometry = true`，bringup 用 `odom_topic:=/odom_ground_truth` remap）——**直接重跑上面的命令即可**。
验证：

```bash
ros2 topic list | grep odom_ground_truth      # 确认有这路 odom
ros2 run tf2_ros tf2_echo odom base_link      # 静止 30 s：应几乎不动（之前是 ~4 cm/s 匀速漂）
```
> 仿真这路是 Gazebo 底盘真值（相当于**理想轮速里程计**）。实车换成下位机轮速 odom 时，
> 因为有滑移/噪声，要把 `POSE_GRAPH.optimization_problem.odometry_translation/rotation_weight`
> 从 `1e5` 调小（例如 `1e3`），否则回环拉不动轨迹。

---

## 7. 局部规划器变体（RPP / DWB / TEB）

```bash
# 统一加 spin_speed:=0.0（先把小陀螺关掉，只比控制器本身，见 §0.4.1）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=dwb spin_speed:=0.0
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio localization:=amcl nav:=teb spin_speed:=0.0
# 想对比"全局障碍来源"再叠加：global_obstacle:=stvl（默认）/ scan / none
```

> 变体文件位置：`src/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml`。
> **比较控制器时要固定其他维度**（同一场地/同一 LIO/同一重定位/同一 `spin_speed`/同一目标点），
> 否则比出来的差异不归控制器。

---

### 7.1 全局障碍来源 A/B（`global_obstacle`，2026-09 新增槽位）

```bash
# A：3D 体素层（默认）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=stvl
# B：2D /scan（与 local 同源）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=scan
# C：只用 static + inflation（全局不看实时障碍）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 global_obstacle:=none
```
**运行期也能切**（不用重启整场仿真，两个图层都支持 `enabled` 动态参数）：
```bash
ros2 param set /global_costmap/global_costmap.stvl_layer.enabled false
ros2 param set /global_costmap/global_costmap.obstacle_layer.enabled true
```
**要比较的三件事**（结果记入 `docs/algorithm_matrix.md` §四）：
1. **一致性**：造一个 0.1 m 矮台用例，看 local 与 global 判断是否一致（`scan` 模式应一致）；
2. **行为**：global 路径是否绕开临时障碍、是否出现"幽灵障碍"（`scan` 模式量程已修为 10 m）；
3. **资源**：`top` 看 `component_container_mt` 的 CPU（`stvl` 明显更重）。

---

### 7.2 局部障碍来源 A/B（`local_obstacle`，2026-09 新增槽位）

```bash
# A：只吃 /scan（默认 = 原行为）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=scan
# B：只吃点云直投（不经 p2l → 无 45cm 盲区）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=cloud
# C：双源冗余（任一路挂掉仍能避障）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav lio:=fastlio \
  localization:=amcl nav:=rpp spin_speed:=0.0 local_obstacle:=both
```

**这一槽位解决什么**：`/scan` 是**串行单点**（插件 → `linefit` → `p2l`），且 `p2l` 有 45 cm 盲区；
`cloud` 这一路直接吃 `/segmentation/obstacle`（3D 障碍点云），**绕过 `p2l`** → 破单点 + 消盲区。
注意它是**部分冗余**：仍依赖 `linefit`（点云来源），只是不再依赖 `p2l`。

运行期也能切（两个图层都支持 `enabled` 动态参数，不必重启整场仿真）：
```bash
ros2 param set /local_costmap/local_costmap.obstacle_layer.enabled false
ros2 param set /local_costmap/local_costmap.obstacle_cloud_layer.enabled true
```

**验证要点**：
1. `ros2 topic info /segmentation/obstacle --verbose | grep "Node name"` → 应出现 local costmap（`both`/`cloud` 时）；
2. 把车停到**离墙 0.3 m**（`scan` 模式看不见）→ `cloud`/`both` 模式下 local costmap 应出现障碍格；
3. 比 CPU（多一个 3D 点云消费者）。

---

### 7.3 失效检测 / 降级验证（`expected_update_rate`，2026-09 修复）

**为什么要有**：障碍源停发时 nav2 默认**不检查**（`expected_update_rate: 0`），缓存会永远重放最后一帧
→ costmap 冻住、无报错、车继续走（**静默失效**）。修复后应变成"报警 + 停车"。

```bash
# ① 正常：/scan 在发
ros2 topic hz /scan
# ② 手动断掉感知链（也可以断更上游：pkill -f ground_segmentation）
pkill -f pointcloud_to_laserscan
# ③ 期望：
#    - 日志出现 The /scan observation buffer has not been updated for X seconds,
#      and it should be updated every 0.50 seconds.
#    - ~1 s 内 /cmd_vel 归零（velocity_smoother 的 velocity_timeout: 1.0）
ros2 topic hz /cmd_vel
```

---

## 8. 全矩阵指令

> 下面矩阵是**核心维度**（场地 × LIO × 重定位/建图后端 × 局部规划器）。另外三个装配级开关会成倍影响行为，
> A/B 时**一次只动一个**：`spin_speed`（5.0 哨兵小陀螺 / 0.0 直通，见 §0.4.1）、
> `global_obstacle`（stvl / scan / none，见 §0.7 与 `docs/3d_to_2d_survey.md` §七）、
> `local_obstacle`（scan / cloud / both，见 §7.2）。

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

### 8.4.1 slam_nav（边建图边导航：3 场地 × 2 LIO × 2 建图后端）
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=pointlio mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL     mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=slam_nav lio:=fastlio  mapper:=slam_toolbox  nav:=rpp
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC     mode:=slam_nav lio:=fastlio  mapper:=cartographer   nav:=rpp
# 局部规划器变体同样适用：在 slam_nav 下追加 nav:=dwb / nav:=teb
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
# 全局障碍来源确认（global_obstacle 槽位，二选一生效）
ros2 param get /global_costmap/global_costmap.stvl_layer.enabled              # stvl 模式应为 True
ros2 param get /global_costmap/global_costmap.obstacle_layer.enabled          # scan 模式应为 True
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
| `tf2_echo map odom`（nav 模式） | 由所选重定位模块提供（amcl / slam_toolbox / icp / cartographer）；**前提见 9.2** |
| `topic hz /odom` | fastlio ≈10Hz；pointlio 更高（数十~100Hz） |
| `view_frames` | 主链 `map→odom→base_link→base_link_fake→…`、`base_link→livox_frame…`；`camera_init→body` 为孤岛（正常） |
| `lifecycle get /controller_server` | `active` |
| `/map` 的 `info` | 宽高 = 对应场地的 pgm 尺寸（map_server 加载**磁盘上的既有地图**，不是实时扫描结果） |
| `tools/check_map_reachable.py` | 选目标点前跑：能判"起点/目标是否同一连通域"、推荐可达点、也能当**幽灵墙检测器**（连通域被切成多块=异常） |
| `tools/pcd_to_grid_map.py` | 离线把 LIO 的 `.pcd` 投成 `.pgm`（第三条建图路线）；`--compare` 可与已有栅格图比对结构一致性 |
| `global_obstacle` 生效确认 | `ros2 param get /global_costmap/global_costmap.{stvl_layer,obstacle_layer}.enabled` —— 两个里恰有一个 True |
| `local_obstacle` 生效确认 | `ros2 param get /local_costmap/local_costmap.{obstacle_layer,obstacle_cloud_layer}.enabled` —— `scan` → 前者 True；`both` → 都 True |


---

## 10. 错误对照

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'rm_nav_bringup' not found ... searching: ['/opt/ros/humble']` | 新终端**只 source 了 /opt/ros，没 source 工作区** | `source ~/HzMi_rmsimulation/install/setup.bash`（见 §0.1；可写进 `~/.bashrc`） |
| `view_frames` 里看到 `camera_init→body` 孤立小岛 | LIO 仍广播内部帧；T3 后导航层不再使用（`odom→base_link` 由 `lio_tf_adapter` 提供） | 正常现象，无需处理（回退用法 `localization:=''` 仍依赖它；**T6 已撤销**，`base_link_fake` 不再移除） |
| `Robot is out of bounds of the costmap!`（仅启动时出现几次） | slam_toolbox 地图尚在生长、global_costmap 正在 resize 的瞬态 | 若**持续刷屏**再排查 `map→odom`（`ros2 run tf2_ros tf2_echo map base_link`） |
| RViz 里**车/雷达看起来是斜的**，但 Gazebo 里车是正的 | spawn 高度过高：RMUL/RMUL2026 原来写 `z=1.16`，而地面在 z≈0（轮半径 0.06 → 落地时 base_link 仅 0.06 m），机器人**悬空 1.1 m 落下**，FAST-LIO 在坠落中做重力初始化 → 地图/位姿倾斜 | **已于 2026-09 修复**：`rm_simulation.launch.py` 中 RMUL / RMUL2026 的 spawn `z` 改为 **0.2**。验证：`ros2 run tf2_ros tf2_echo odom base_link` 的 roll/pitch 应≈0 |
| nav 模式卡在 `amcl: Waiting for map....` / `global_costmap: Invalid frame ID "map"` | ① `map_server_launch.py` 与 `localization_amcl_launch.py` **各起了一个同名 `lifecycle_manager_localization`**（冲突）；② `nav2_params_sim_*.yaml` 里 `yaml_filename` 被注释掉，而 nav2 的 `RewrittenYaml` **只替换已存在的键** → map_server 报 `parameter 'yaml_filename' is not initialized` | **已于 2026-09 修复**：① amcl launch 现在用**单一 lifecycle_manager 同时管理 `map_server`+`amcl`**，bringup 仅在 `icp`/未选重定位时单独起 map_server；② 三份 nav2 参数恢复 `yaml_filename: ""` 键（launch 会注入实际地图路径） |
| Nav2 在 `odom`/`map` 出现前就激活，刷 `Timed out waiting for transform ...` | Gazebo 生成机器人 + LIO 初始化需要数秒，而 Nav2 立即启动 | **已于 2026-09 缓解**：`bringup_sim` 中定位链延后 **4s**、mapping 后端延后 **4s**、Nav2 延后 **10s** 启动 |
| `spawn_entity: Spawn status: ... timed out waiting for entity to appear` | RMUL2026 世界加载慢，spawn 默认超时过短（实体其实已生成） | **已于 2026-09 修复**：spawn 参数加 `-timeout 60.0` |
| `cartographer_node` 启动几秒后 `exit code -6`，日志 `Check failed: ... The IMU frame must be colocated with the tracking frame` | `tracking_frame="livox_frame"`，而 `/livox/imu` 的 frame 是 `imu_link`，URDF 里两者差 5cm → cartographer 的 **IMU 共位硬 CHECK** 失败（`sensor_bridge.cpp:136`） | **2026-09-21 已修**：`tracking_frame = "imu_link"`（官方 `mir-100-mapping.lua` 同做法）。沙箱 A/B 验证：隔离 domain + 合成 TF/IMU 消息下，旧配置必 abort、新配置通过 |
| `cartographer_node` 加载纯定位 lua 即 FATAL：`Key 'pure_localization' was used the wrong number of times` | `cartographer_localization.lua` 把 `pure_localization`/`pure_localization_trimmer` 写在 `TRAJECTORY_BUILDER_2D`，而 cartographer 从**顶层 `TRAJECTORY_BUILDER`** 读（`trajectory_builder_interface.cc` 的 `kDictionaryKey`）；写错层级的键永不被读，而它要求"每键读恰好一次" | **2026-09-21 已修**：改成 `TRAJECTORY_BUILDER.pure_localization_trimmer = {...}`（官方写法）；deprecated 的 bool 不再设 |
| `spawn_entity: Spawn service failed. Exiting.` + 随后 `local_costmap: ... "odom" ... frame does not exist` 一直刷、Nav2 永不激活 | `spawn_entity` 放弃后**机器人其实晚了 3~5 秒才被插入**（gzserver 仍会打印 `mecanum_controller: Subscribed to [/cmd_vel_chassis]`、`LivoxPointsPlugin: ros topic name: /livox/lidar`）。而 Nav2 在 `odom` 帧出现前激活不了；若在此期间 LIO 还没吐 `/odom`，就一直是这个循环。**RMUL 场地网格 44 万三角面（RMUL2026 仅 3187），加载明显更慢** | 不要 20 秒就下结论：**给 30~60 秒**再判断。用 `ros2 topic hz /livox/lidar`（CustomMsg 10Hz）→ `/livox/imu`（100Hz）→ `/imu/data`（互补滤波输出 100Hz，FAST-LIO 的 `imu_topic`）→ `/odom`（≈10Hz）**逐段定位**；`/odom` 一出，`odom` 帧即有、local_costmap 立即恢复、Nav2 自行激活。若 `/livox/lidar` 有数据而 `/odom` 始终没有 → 换 `lio:=pointlio` 做 A/B（它用原始 `/livox/imu`）以区分是雷达侧还是 FAST-LIO+互补滤波支路 |
| `Timed out waiting for transform from base_link to map` | `map→odom` 缺失 | nav 模式必须指定 `localization:=amcl\|slam_toolbox\|icp\|cartographer` |
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
| `global_obstacle:=scan` 下 global 图上出现幽灵障碍（规划绕开不存在的东西） | 照抄了 local 的 `raytrace_max_range: 6.0`，而 global 是**全图**（13×10 m）→ 6 m 以外的旧标记永不被清除 | **2026-09 已修**：global 的 scan 源量程改为 **10.0 m**（与 p2l 的 `range_max` 对齐）—— 原则是「扫描能看到多远，就要能清多远」 |
| **RViz 里一开始就有很完整的地图**，不是慢慢扫出来的 | nav 模式下那是 `map_server` 加载的**磁盘既有 pgm**（`src/rm_nav_bringup/map/<world>.pgm`），不是实时建图 | 正常。想看实时建图用 `mode:=mapping`（slam_toolbox/cartographer）；nav 模式要判断定位对不对，看 **scan/costmap 与这张地图的墙体是否重合** |
| costmap 刷屏 `Sensor origin at (x,y) is out of map bounds`（数值在 spawn 坐标与 LIO 估计间跳变） | **同一 `odom` 有两个来源**：Gazebo 真值 + LIO 都发到 `/odom`，`lio_tf_adapter` 交替收到两者 → `odom→base_link` 抖动 | **已于 2026-09 修复**：`sentry_robot_sim.xacro` 把 Gazebo 真值 remap 到 **`/odom_ground_truth`**（`publish_odom_tf=false`），`/odom` 由 LIO 独占。对比真值请看 `/odom_ground_truth` |
| RViz 报 `Message Filter dropping message ... queue is full` 且退出时 `rviz2 exit code -11` | RViz 在高频 TF/点云负载下丢帧，退出时崩溃（常见现象，不影响仿真链路） | 可先 `nav_rviz:=False` 验证导航链路；或减少 RViz 中 PointCloud2/STVL 显示项 |
| **发目标后车不走**，`/cmd_vel` 长时间只有 `linear.x: -0.05`（偶尔 `0` + 角速度） | `-0.05` = nav2 自带 BT `BackUp` 的 `backup_speed="0.05"`（见 `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`）→ **BT 进了恢复行为循环**（清代价地图→Spin→Wait→BackUp 轮转），说明 `ComputePathToPose` 或 `FollowPath` 连续失败 | 按顺序查：① 终端 A 里 `planner_server` / `controller_server` 的 WARN/ERROR（**最直接**，会写明原因）；② `ros2 topic echo /plan --once --field poses`（空=规划失败）；③ `ros2 topic echo /odom_ground_truth --field pose.pose.position --once` 与 `/amcl_pose` 对比（RMUL2026 的 map 系=世界系，两者必须接近）；④ `spin_speed:=0.0` 重跑（见 §0.4.1） |
| `mapper:=cartographer` 时 costmap 没有静态图 / `map_saver_cli` 存不到图 | cartographer 的栅格被硬 remap 到 `/cartographer_map`，而 nav2 `static_layer` 与 `map_saver_cli` 都订阅 `/map` | **已于 2026-09 修复**：`cartographer_sim.launch.py` 默认发 `/map`；纯定位另有 `map_server` 时用 `occupancy_grid_topic:=/cartographer_map` 覆盖 |
| 纯建图 `mode:=mapping` 下 `nav_rviz:=True` 却没有任何 RViz | 三种形态拆分后 `mapping` 不再启动 nav2，而 RViz 原先由 nav2 的 `rviz_launch` 带起 | **已于 2026-09 修复**：bringup 为 `mode=='mapping' and nav_rviz=='True'` 单独补一个 RViz（`nav2.rviz`） |
| `mapper:=cartographer` 建出来的图**几乎是空的**（526 B pbstream / `/map` 没东西） | 三处对接错误：① `points2` 被 remap 到 `/livox/lidar`（**CustomMsg**，cartographer_ros 只支持 `sensor_msgs/PointCloud2`）→ **静默收不到数据**；② `published_frame="body"` + `provide_odom_frame=true` 与 FAST-LIO 争 `body` 子帧；③ `min_z/max_z` 被当成“相对地面”（实际**相对传感器**）→ 只切到墙顶 | **已于 2026-09 修复**（改走 `/scan`、`published_frame="odom"`、带改回 -0.8~2.0）；判据：`ros2 topic info /scan --verbose` 里应出现 `cartographer_node` |
| `planner_server: GridBased: failed to create plan with tolerance 0.50` / `Planning algorithm GridBased failed to generate a valid path to (x, y)` | **目标点与起点不在同一连通域**（中间被墙隔开），或起点/终点落在膨胀带内。地图上 1 像素宽的虚线经 `robot_radius` 膨胀后就能封死走廊 | 先跑 `tools/check_map_reachable.py --map <map.yaml> --start <map 系起点> --goal <目标>` 判定；换用同一连通域的目标点（§0.6）。若是**地图资产**问题（例：RMUL2026.pgm 的 x≈5.2 幽灵虚线，世界网格里没有实体）→ 重新建图 |
| 参数写在 `nav2_params_*.yaml` 里却"没生效" | **节点名对不上**：nav2 跨版本改过名（如 Galactic `recoveries_server` → Humble **`behavior_server`**；`recovery_plugins` → `behavior_plugins`；插件类型 `nav2_recoveries/*` → `nav2_behaviors/*`）。对不上的整段被**静默忽略**，节点改用内置默认值 | 拿 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` 的顶层键做参照逐个核对；**已于 2026-09 修复**三份 sim 变体的 `recoveries_server` 段。判断某段是否生效的最快办法：看节点启动日志（如 `behavior_server: Creating behavior plugin ...` 的**个数/名字**是否与 yaml 一致） |
| FAST-LIO 在 `Ctrl+C` 时报 `exit code -11` | FAST-LIO 已知的退出崩溃 | 忽略；不影响运行期 |
| cartographer 纯定位报找不到状态文件 | **三个场地都没有可用 pbstream**（RMUL/RMUC 无该文件；`RMUL2026.pbstream` 仅 526 B 空壳） | 先按 §5 跑一次 cartographer 建图并 `finish_trajectory` + `write_state` 导出（路径用绝对路径，见 §6） |

### 10.1 现象 → 归属（先分类，再查上表）

| 现象 | 归属 | 一句话判据 |
|---|---|---|
| 点云转瞬即逝 / 手遮挡很明显 | **RViz 显示设置** | PointCloud2 的 Decay Time = 0（只画最新帧），不是地图问题 |
| 改了"刷新率"后点留住了 | **RViz 显示缓冲** | 那是屏幕缓冲，不是建图（见 `docs/architecture.md` §3.2.6） |
| 房间点云**跟着车跑** | **帧树 / 显示系 / 里程计** | 显示项是不是车体系？Fixed Frame 对吗？`tf2_echo` 链通吗？ |
| 地图**缓慢弯折 / 闭环处双墙** | **建图（缺回环）** | 绕一圈看闭环误差与双墙；LIO 无回环 |
| 位姿**突然跳到另一处** | **重定位** | `tf2_echo map odom` 是"跳"；看 `/particle_cloud` 是否多峰 |
| 位姿**高频小抖**（cm/度级来回） | **时序 / 刷新** | 时间戳不同步、TF 与 costmap 更新节奏不匹配 |
| 位姿**连续缓慢漂** | **里程计（LIO）** | 与 `/odom_ground_truth` 比，误差随距离增长 |
| 目标点规划失败（`failed to generate a valid path`） | **地图资产 / 选点** | 先 `check_map_reachable.py`；再看地图有没有幽灵结构 |
| **costmap 冻住**：车照原速继续走、日志无报错、RViz 里 costmap 还在刷新 | **静默失效**：障碍源（`/scan` 链）已断，而 nav2 的 `expected_update_rate` 默认 `0` = 不检查；`ObservationBuffer` 会一直重放最后一帧 | **2026-09 已修**：障碍源加 `expected_update_rate: 0.5` → 源停即 WARN + 拒绝算速度 + 1 s 内停车（`velocity_timeout`）。定位顺序：`ros2 topic hz /scan`（发了吗）→ `ros2 topic info /scan --verbose`（谁在收）→ `ros2 node list`（`linefit`/`p2l` 还在吗）→ §7.3 |
| 贴身 0.3 m 的障碍看不见、且那个方向被清成 free | `p2l` 的 `range_min`（原 0.45）把近点**丢弃**，bin 保持 `inf`，而 `inf_is_valid: true` 让 nav2 当"10 m 处有回波"→ 沿射线清空 | **2026-09 已修**：`range_min: 0.45 → 0.2`（与车体半径/`linefit r_min` 对齐）；需要更早发现就用 `local_obstacle:=cloud\|both`（点云直投无此盲区） |
| RViz 里**整张地图跟着小车转/平移，还留残影** | 先分清"显示"还是"真转"：① RViz `Global Options → Fixed Frame` 不是 `map`（或 `Views → Current View → Target Frame` 选了 `/base_link`）时，整个场景会跟着车体坐标系转 —— **地图其实没动**（`/map` 的单元格固定发布在 `map` 系）；② `ros2 run tf2_ros tf2_echo map odom` 若在转弯/回环时**突然跳到几十度**，那才是位姿图在修正 = **误回环**（RM 场地小且对称），地图整体被拧并留残影 | ① 把 `Fixed Frame` 设成 `map`、`Target Frame` 设成 `<Fixed Frame>`；② 误回环 → **2026-09-21 已收紧回环参数**：`max_constraint_distance 10→4`、`min_score 0.55→0.72`、`global_localization_min_score 0.6→0.8`、`fast_correlative_scan_matcher` 搜索窗 `5m/20°→2m/10°`、`global_constraint_search_after_n_seconds 10→30`。判据：重跑后 `map→odom` 应始终 ≈0 |
| RViz 里**栅格地图绕着小车转**（尤其车原地转时），但 `map→odom` 一直是 ≈0/恒定 | **`lio_tf_adapter` 丢了杆臂**：它把 LIO 的 `body`（= 雷达/IMU，在 `base_link` 前 0.12 m、上 0.125 m）位姿直接改名成 `base_link` 发布（配置里 `xyz: [0,0,0]`）→ 车原地转时"base_link"绕真实中心画半径 **≈0.17 m 的圆**，地图静止 → 相对运动看起来就是地图在转。同根因公开案例：nav2 issue [#4135](https://github.com/ros-navigation/navigation2/issues/4135)（雷达装偏 → 转圈时 map 跟着画圈） | **2026-09-21 已修**：`lio_tf_adapter/config/lio_tf_adapter.yaml` 的 `xyz: [-0.12, 0.0, -0.125]`（FAST-LIO 与 Point-LIO 的 `extrinsic_T` 都是 `[0,0,0.05]`，同一套杆臂，改一处即可）。**验证**：原地转 360°，`ros2 run tf2_ros tf2_echo map base_link` 的 x/y 应基本不动（修前会画圆） |
| 车不走但 `/cmd_vel` 有 -0.05 + 旋转 | **BT 恢复行为循环** | 说明规划或控制连续失败（见上表对应行） |
| `/cmd_vel_chassis` 出现 `angular.z: 5.0` | **小陀螺在动作** | 那是 `spin_speed`，不是 nav2 发的角速度（§0.4.1） |

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

**按"归属"收集证据（配合 §10.1）**

```bash
# 时序/刷新类：看 map→odom 的时间戳是否连续、频率是否稳定
ros2 topic hz /tf; ros2 run tf2_ros tf2_echo map odom
# 里程计类：与真值对比（RMUL2026 的 map 系=世界系，可直接比；RMUL/RMUC 只比相对变化）
ros2 topic echo /odom_ground_truth --field pose.pose.position --once
ros2 topic echo /amcl_pose --field pose.pose.position --once
# 重定位类：粒子云是否多峰（RViz 看 /particle_cloud），或 ICP 是否在配准
ros2 node list | grep -E "amcl|icp_registration"
# 建图类：闭环误差（绕一圈回起点）与"双墙"
```

**当前优先验证顺序**（按资产齐备度，见 `docs/algorithm_matrix.md` §五）：
1. **重建 RMUL2026 地图**（§1 + §1.1）← 解开该场地所有 nav 组合；
2. `nav+amcl` @ RMUL2026 发目标（§2 + §0.6）；
3. `nav+slam_toolbox` @ RMUL（§4）、`nav+icp` @ RMUL（§3）；
4. `slam_nav`（§1.2）→ cartographer 建图/纯定位（§5/§6）→ `nav:=dwb|teb`（§7）。

---

## 12. 实测记录

> **实测状态的唯一真值来源是 `docs/algorithm_matrix.md` §四**（每跑通一个组合在那里加一行）。
> 本表只保留"实跑时特有的、值得复现的现象"。

| 日期 | 场景 | 命令要点 | 结果 |
|---|---|---|---|
| 2026-09 | 一 mapping+fastlio | `mode:=mapping lio:=fastlio` | ✅ `/odom` 连续、`odom→base_link` 连续；曾修：Gazebo 里程计抢 `/odom`（remap 到 `/odom_ground_truth`）、出生 z=1.16 自由落体致 RViz 车体倾斜（改 z=0.2） |
| 2026-09 | 二 nav+amcl | `mode:=nav localization:=amcl nav:=rpp` | ✅ `Managed nodes are active`；`map_server 272×210` → `amcl Received a 272 X 210 map`。启动期约 4s 刷 `Timed out waiting for transform from base_link_fake to map` 与 `extrapolation` 属**瞬态**（TF 各帧刚建立、10Hz LIO TF 略滞后于 `now()`），给完 `/initialpose` 后自行恢复，不影响激活 |
| 2026-09 | 二 关机 | Ctrl-C | `fastlio_mapping` / `component_container_mt` 退出码 **-11** 属 Humble 关机期已知现象（进程已 Deactivate/Cleanup，非运行期崩溃） |
| 2026-09 | 二 地图系排查 | `world:=RMUL` 下按「世界出生点」给 AMCL 初值 | ❌ **判断错误已修正**：RMUL/RMUC 的 pgm 是**出生点系**（初值必须 `(0,0,0)`），只有 RMUL2026 是**世界系**（`(4.3,3.35)`）。判定方法与证据见 §0.5；已改为 launch 按 `world` 自动注入初值 |
| 2026-09 | 无头 amcl 单测（砂箱，无 Gazebo/RViz） | 只起 map_server+amcl，注入 `initial_pose_x/y=4.3/3.35` | ✅ **自动初值生效**（`/amcl_pose` = 4.30, 3.35）；且证明 **无 `/scan` 时 `map` 帧根本不存在**，补上 `/scan` 后立刻出现 `map→odom`（见 §9.2） |
| 2026-09 | 二 nav+amcl @ **RMUL2026**（用户机实跑） | `world:=RMUL2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp nav_rviz:=True` | ✅ **全链路通过**：`/controller_server`+`/planner_server` = active；`/map` 240×169 origin(2.68,0.228)；`map→odom` = **(4.294, 3.357, 0.052)**、RPY (-0.30°,-0.03°,-0.07°)；`/amcl_pose` = **(4.300,3.350)**（=注入初值）；`/scan` 1 pub + 3 sub（amcl/local_costmap/rviz）**QoS 全 BEST_EFFORT 匹配** |
| 2026-09 | 二 `/amcl_pose` 的 covariance ≈ 0 | nav2 `set_initial_pose` 路径**不填协方差**（`amcl_node.cpp` L271-281 只设 position/orientation） | 正常现象：初始粒子云是**零散布单点**。位置给对无影响；若初值给错，AMCL 难以自行纠回 → 必须用 RViz `2D Pose Estimate` 重给 |
| 2026-09 | 二 发目标 @ RMUL2026 | `/cmd_vel` 长时间 `linear.x: -0.05`、`/cmd_vel_chassis` 出现 `angular.z: 5.0` | ⚠️ **未通过**：`-0.05` = BT `BackUp` 的 `backup_speed`，`5.0` = `fake_vel_params.yaml` 的 `spin_speed` → 命令链通、小陀螺生效，但 Nav2 卡在**恢复行为循环**（路径/控制失败）。顺带查出 `recoveries_server` 段（Galactic 名字）被 Humble 静默忽略，已改为 `behavior_server` |
| 2026-09 | 二 发目标 @ RMUL2026（第二轮，带日志） | `planner_server: failed to generate a valid path to (6.00, 3.40)`；`odom_ground_truth=(4.2226,3.3442)` vs `amcl_pose=(4.300,3.350)` 差 8cm | ✅ **根因确定**：定位/地图坐标系没问题（真值与 amcl 一致），失败在于**目标点被地图上不存在的"幽灵墙"隔开**——`RMUL2026.pgm` 在 x≈5.2 有一条 1 像素宽虚线（y≈1.8→6.0），而世界网格在该处**零顶点**；经 `robot_radius=0.2m` 膨胀后走廊被封死（r=0.15m 时同一点即可达），地图被切成 3 块、目标落在另一块。已新增 `tools/check_map_reachable.py` 用于选点前验证 |
| 2026-09-16 | 文档/架构（非实跑） | 新增 `docs/algorithm_matrix.md`（算法现状唯一真值来源）与 `docs/issues_and_findings.md`（问题汇总）；决策**不按 2D/3D 物理重组目录**（维度只做文档标注） | ✅ 已登记到 architecture 索引 |
| 2026-09-16 | cartographer 话题修正 | 栅格输出由 `/cartographer_map` 改回标准 `/map`（原 remap 导致 `static_layer` 与 `map_saver_cli` 静默失效） | ✅ 可用 `occupancy_grid_topic` 覆盖；场景五判据已加 `topic info /map` |
| 2026-09-16 | 纯建图 RViz | 形态拆分后 `mode:=mapping` 丢失 RViz | ✅ bringup 单独补 RViz（`nav2.rviz`） |
| 2026-09 | 三 nav+ICP @ RMUL（第一次） | `spawn_entity: Spawn service failed` + `odom` 帧 13 秒不存在 | ⚠️ **未进入 ICP**：机器人插件晚 4 s 加载，LIO 尚未吐 `/odom`，Nav2 无法激活（现场 20 s 就 Ctrl-C 了）。ICP 本身正常：`pcd point size: 97642, 4866`、`pointcloud_topic: /livox/lidar/pointcloud`、`icp_registration initialized`。结论：RMUL 世界重（44 万面），需给 30~60 s |

