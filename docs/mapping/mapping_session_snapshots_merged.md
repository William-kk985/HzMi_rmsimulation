# 建图会话快照（合并版）：Cartographer 建图 → 保存 `.pbstream`

> **STATUS（2026-10 合并）**：本文件**合并自** `docs/mapping/` 下 2 份**同一会话**的建图状态快照
> （`建图操作指南.md` 325 行 + `立即开始建图.md` 115 行，均 2026-09-20 提交 `84f3de6`，记录的是 RMUL2026 场地那次会话）。
> 原文已用 `git mv` 移到 [`docs/archive/mapping/`](../archive/mapping/)（**保留 git 历史**），每份顶部加了指向本文件的 `STATUS`。
> **现行建图流程以 [`../smoke_test_runbook.md`](../smoke_test_runbook.md) §1/§1.1（含落盘三件套）为准**；
> 本文件的价值是"**当时这几步是怎么走的、键位/观察/排错细节**"，以及 §9 的"哪里已经过时"。
>
> **证据约定**：除 §9 另标外，正文全部是【引用】（逐字或等价改写自两份原文；命令、键位、路径、数字保持原样）。
> **两份原文的关系**：`建图操作指南.md` = 完整步骤版（从"停掉旧 SLAM"开始）；`立即开始建图.md` = 同一会话**后段**的
> "一切已就绪，你现在只要操作机器人"简版。两版**不冲突，只是起点不同**（一个假设"需要启动 Cartographer"，一个假设"已启动"）。

## 版本来源

| # | 原文件 | 行数 | 最后提交 | 会话状态假设 | 独有内容 |
|---|---|---|---:|---|---|
| 1 | `建图操作指南.md` | 325 | `84f3de6 2026-09-20` | 已启动 Gazebo + 键盘；**Cartographer 还没起** | 启动/停止命令、Cartographer launch 与预期输出、完整键位表（含斜行 U/O/M/.）、三阶段路线、应该做/避免、三种质量观察法、4 条故障排除、时间预估 |
| 2 | `立即开始建图.md` | 115 | `84f3de6 2026-09-20` | **全部已就绪**（Cartographer 轨迹 ID 0 在跑） | "现在只要按键"的最短路径、5-8 分钟三步路线、`ls -lh` 的预期输出样例（3.2M）、"绿色 >90% 才保存"的判据 |

---

## 1. 当时的前置状态（【引用】）

**`建图操作指南.md` 的起点**：✅ 已启动 Gazebo 仿真环境（RMUL2026 场地）、✅ 已启动键盘控制节点（`teleop_twist_keyboard`）；
❌ **还需要启动** Cartographer SLAM 节点。

**`立即开始建图.md` 的起点**：所有系统已就绪并运行中 —— ✅ Gazebo 仿真（RMUL2026 场地）、✅ Cartographer SLAM（轨迹 ID: 0）、
✅ 键盘控制节点。

> 【推断】两段合起来就是**同一次会话的前后两个时刻**：先按指南把 Cartographer 起起来，起好后作者又写了一份"立即开始"的简版。

---

## 2. 完整启动步骤（【引用】`建图操作指南.md`）

### 步骤 1：先停止当前的 SLAM 系统

当时跑的是 SLAM Toolbox，需要先停：

```bash
# 在运行 start_sentinel.sh 的终端按 Ctrl+C
# 或者执行：
pkill -f start_sentinel.sh
```

### 步骤 2：重新启动纯 Gazebo 环境（不含 SLAM）

```bash
cd ~/HzMi_rmsimulation
source install/setup.bash

# 只启动 Gazebo，不启动任何 SLAM 算法
./start_sentinel.sh -w RMUL2026 -m mapping --lio false
```

或者手动启动 Gazebo：

```bash
ros2 launch rm_nav_bringup gazebo_rmul2026.launch.py
```

### 步骤 3：在新终端启动 Cartographer

```bash
cd ~/HzMi_rmsimulation
source install/setup.bash

ros2 launch cartographer_ros cartographer.launch.py \
  configuration_directory:="src/rm_localization/cartographer_ros/configuration_files" \
  configuration_basename:="cartographer.lua"
```

**预期输出**（【引用】原文）：

```text
[INFO] [...] Loading state file: ...
[INFO] [...] Starting trajectory 0 ...
[INFO] [...] Using 3D point clouds from Livox MID360 ...
[INFO] [...] Initialized submap with 30 range data ...
```

### 步骤 4：启动键盘控制（第三个终端）

```bash
cd ~/HzMi_rmsimulation
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 3. 键位与建图路线

### 3.1 键位表（两份原文合并）

| 按键 | `建图操作指南.md` | `立即开始建图.md` |
|---|---|---|
| **I** | 前进（慢速前进，推荐） | 前进（按几次走几步） |
| **,** | 后退（倒车调整） | 后退 |
| **J** | 左转（逆时针旋转） | 左转（原地旋转） |
| **L** | 右转（顺时针旋转） | 右转（原地旋转） |
| **K** | 停止（紧急停止） | 停止（紧急制动） |
| **U** | 左前斜行（全向移动，需配合 Shift） | — |
| **O** | 右前斜行（全向移动，需配合 Shift） | — |
| **M** | 左后斜行（全向移动，需配合 Shift） | — |
| **.** | 右后斜行（全向移动，需配合 Shift） | — |

【引用】速度微调：`# 如果太快，按 Ctrl 减速 / # 如果太慢，按 Shift 加速`（`建图操作指南.md`）。

### 3.2 三阶段路线（两份一致，措辞合并）

**📍 第一阶段：探索边缘（2-3 分钟）**
1. 按 `I` 键缓慢前进
2. 沿着墙壁走一圈（RMUL2026 的墙壁）
3. 在每个角落稍作停留并旋转（按 `J` 或 `L`）

**📍 第二阶段：覆盖中间区域（2-3 分钟）**
1. 走 "之" 字形路线
2. 确保覆盖所有通道 / 走遍所有通道
3. 避开障碍物 / 经常小角度转向

**📍 第三阶段：触发回环检测（1-2 分钟）**
1. 回到起点位置（附近）
2. 原地旋转 360°（长按 `J` 或 `L`）
3. 重复经过已探索区域 / 触发回环检测

> 【引用】`立即开始建图.md` 给这条路线标的预算：**5-8 分钟**（第 1 步 2 分钟 / 第 2 步 2 分钟 / 第 3 步 1 分钟）。

---

## 4. 建图要点（【引用】`建图操作指南.md` §建图要点总结）

### ✅ 应该做的

1. **慢速稳定** —— 速度控制在 **0.3-0.5 m/s**（太快按 Ctrl 减速、太慢按 Shift 加速）。
2. **多旋转** —— 在关键位置原地旋转 360°：帮助 Cartographer 识别环境特征、提高定位精度。
3. **走闭合回路** —— 最终回到起点：触发回环检测（Loop Closure）、消除累积误差。
4. **覆盖全面** —— 确保所有区域都被扫描到：墙壁边缘、通道走廊、角落和障碍物周围。

### ❌ 避免的

1. **快速转向** —— 会导致匹配失败。
2. **长时间直线** —— 缺少特征点。
3. **遗漏区域** —— 超过 **30%** 未探索。
4. **碰撞墙壁** —— 会影响 IMU 数据。

---

## 5. 观察建图质量（【引用】§观察建图质量）

### 方法 1：看 Cartographer 日志

```bash
# 好的标志：
[INFO] ... Added node ... to trajectory 0
[INFO] ... Localizing trajectory ...
[INFO] ... Found constraint ...

# 警告标志：
[WARN] ... Scan matching failed ...
[WARN] ... Too few points ...
```

### 方法 2：RViz 可视化（可选）

```bash
# 新终端
rviz2
```

添加显示：**Map**（话题 `/map`）、**LaserScan**（话题 `/scan`）、**TF**。
观察要点：激光束是否与地图边缘对齐；绿色区域（已探索）是否覆盖全场；灰色区域（未知）是否逐渐减少。

### 方法 3：检查话题数据

```bash
# 查看子图列表
ros2 topic echo /submap_list

# 查看轨迹信息
ros2 topic echo /trajectory_node_list

# 查看里程计
ros2 topic echo /odom
```

> 【引用】保存判据：`建图操作指南.md` 说"地图完整度 **> 90%**"；`立即开始建图.md` 说"RViz 中**绿色区域 > 90%**"。同一口径。

---

## 6. 保存 `.pbstream`（【引用】两份一致的三步）

### 6.1 结束轨迹

```bash
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}"
```

**预期输出**：

```text
Success: true
Status: Trajectory 0 finished.
```

（简版说法：看到 `Success: true` 表示成功。）

### 6.2 写入 `.pbstream`

```bash
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"
```

**预期输出**：

```text
Success: true
Status: State written to /home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream
```

### 6.3 验证

```bash
ls -lh /home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream
```

【引用】`立即开始建图.md` 给的预期输出样例：

```text
-rw-r--r-- 1 weicheng weicheng 3.2M 3 月 21 17:40 RMUL2026.pbstream
```

【引用】文件大小通常在 **1-10 MB** 之间。

---

## 7. 建好之后（【引用】§下一步）

1. **复制 `.pbstream` 到目标项目**

```bash
cp src/rm_nav_bringup/map/RMUL2026.pbstream \
   /path/to/target_project/maps/
```

2. **在目标项目中测试纯定位**

```bash
ros2 launch cartographer_ros cartographer.launch.py \
  configuration_directory:="src/rm_localization/cartographer_ros/configuration_files" \
  configuration_basename:="cartographer_localization.lua" \
  load_state_filename:="maps/RMUL2026.pbstream"
```

3. **验证定位精度**：观察激光匹配情况、测试重定位功能、记录定位误差。

---

## 8. 故障排除（【引用】`建图操作指南.md` §故障排除）

| # | 症状 | 原因/解决 |
|---|---|---|
| 1 | **Cartographer 启动失败**（报错找不到话题或坐标系） | 检查点云话题：`ros2 topic list \| grep point_cloud`；没有就检查传感器驱动：`ros2 launch livox_driver mid360.launch.py` |
| 2 | **机器人无法移动**（按键盘不动） | 检查速度话题 `ros2 topic echo /cmd_vel`；没输出就重启键盘节点：`pkill teleop_twist_keyboard` 然后 `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 3 | **建图漂移严重** | 原因：运动过快或 IMU 数据异常。解决：① 降低移动速度 ② 检查 IMU `ros2 topic echo /imu/data` ③ 增加旋转动作 |
| 4 | **地图不完整** | 原因：有些区域没走到。解决：① 继续探索未覆盖区域 ② 在 RViz 里看灰色（未知）区域 ③ 针对性探索 |

【引用】预计时间（`建图操作指南.md`）：启动系统 1-2 分钟 + 建图过程 5-8 分钟 + 保存文件 30 秒 = **约 10 分钟**。

---

## 9. 与现行（2026-10）流程的差异

> 本节是本轮整理**新加的**，不是原文内容；**没有运行任何仿真**，只读当前 launch/脚本与现行文档。

| 主题 | 快照里的做法（2026-09） | 现行（【引用】源码/文档） | 差异性质 |
|---|---|---|---|
| 启动方式 | `./start_sentinel.sh -w RMUL2026 -m mapping --lio false`（只起 Gazebo）+ 另起 `cartographer_ros` | `ros2 launch rm_nav_bringup bringup_sim.launch.py mode:=mapping lio:=<...> mapper:=cartographer` 一体化；`lio` 的合法取值是 `fastlio｜pointlio｜small_point_lio｜none｜cartographer`（**没有 `false`**，要"不起 LIO"用 **`none`**） | 【引用】`bringup_sim.launch.py:1544-1558`；【推断】`--lio false` 在今天的 launch 上会被当成非法值/被包装脚本改写 ⇒ **别照抄** |
| Cartographer 入口 | `ros2 launch cartographer_ros cartographer.launch.py ...` | 仓库内已有 [`cartographer_sim.launch.py`](../../src/rm_nav_bringup/launch/cartographer_sim.launch.py)，由 `mapper:=cartographer` 分支调用；纯定位用 `configuration_basename:=cartographer_localization.lua` + `load_state_filename` + `load_frozen_state:=true` | 【引用】该 launch 头部注释（建图/纯定位两种用法） |
| 手动 Gazebo launch | `ros2 launch rm_nav_bringup gazebo_rmul2026.launch.py` | **该文件名在当前 `launch/` 下不存在**（目录里只有 `bringup_sim.launch.py`、`cartographer_sim.launch.py`） | 【引用】`ls src/rm_nav_bringup/launch/`；【推断】这条老命令已不可用，世界选择改由 `world:=` 参数完成 |
| 落盘脚本 | 手敲 `/finish_trajectory` + `/write_state`（本文 §6） | 仍是同两条原生服务（runbook §5 逐字保留）；另有 `tools/scripts/mapping/{save_grid_map.sh,save_pcd.sh}` 负责 2D 栅格与 3D 点云 | 【引用】`docs/mapping/README.md`、`tools/scripts/mapping/` |
| 现行总览 | — | 建图 → runbook §1（含落盘三件套 §1.1）；边建图边导航 → §1.2；cartographer 建图/纯定位 → §5/§6 | 【引用】`docs/mapping/README.md` 顶部状态说明 |

> 【推断】快照里的**键位、路线、观察点、故障排除**仍然成立（teleop_twist_keyboard 的键位没变；Cartographer 的两条落盘服务没变），
> 真正过时的是**启动命令的形态**与 **`gazebo_rmul2026.launch.py`**。这也是本文件保留全部操作细节、但在 §9 单独列"过时项"的原因。

---

> 【引用】两份原文结尾都指向同一份详细文档：`.pbstream` 的完整生成/使用说明见 [`CARTOGRA_PBSTREAM_GUIDE.md`](CARTOGRA_PBSTREAM_GUIDE.md)
> （`建图操作指南.md` 结尾："如有问题，请查看 CARTOGRA_PBSTREAM_GUIDE.md 获取详细帮助"）。该文件**未**并入本文，仍在本目录、状态为
> 「历史（结论仍有效），含"需谨慎对待"的旧做法」，见 [`README.md`](README.md)。
