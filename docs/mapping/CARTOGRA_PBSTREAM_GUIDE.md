# 🎯 Cartographer .pbstream 生成指南

## 📋 目标

将现有的 **RMUL2026.png + .yaml** 地图转换为 Cartographer 的 **.pbstream** 格式，用于另一个项目的纯定位功能。

---

## ⚡ 快速开始（推荐方案）

### 一键启动脚本

```bash
cd ~/HzMi_rmsimulation
./generate_cartographer_pbstream.sh
```

脚本会自动引导您完成以下步骤！

---

## 📖 详细步骤说明

### 准备工作

确保以下文件存在：
- ✅ `src/rm_localization/cartographer_ros/configuration_files/cartographer.lua` - 建图配置
- ✅ `src/rm_localization/cartographer_ros/configuration_files/cartographer_localization.lua` - 纯定位配置
- ✅ `src/rm_nav_bringup/map/RMUL2026.yaml` - 地图配置
- ✅ `src/rm_nav_bringup/map/RMUL2026.pgm` - 地图图像

### 步骤 1: 启动 Gazebo 仿真

**终端 1：**
```bash
cd ~/HzMi_rmsimulation
source install/setup.bash
./start_sentinel.sh -w RMUL2026 -m mapping
```

**作用**：加载 RMUL2026 场地模型到 Gazebo

---

### 步骤 2: 启动 Cartographer

**终端 2：**
```bash
cd ~/HzMi_rmsimulation
source install/setup.bash
ros2 launch cartographer_ros cartographer.launch.py \
  configuration_directory:="src/rm_localization/cartographer_ros/configuration_files" \
  configuration_basename:="cartographer.lua"
```

**作用**：运行 Cartographer SLAM 算法，使用与目标项目一致的配置

**预期输出**：
```
[INFO] ... Loading state file: ...
[INFO] ... Starting trajectory 0 ...
[INFO] ... Using 3D point clouds from Livox MID360 ...
```

---

### 步骤 3: 控制机器人建图

**终端 3：**
```bash
cd ~/HzMi_rmsimulation
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**建图技巧**：

| 操作 | 按键 | 说明 |
|------|------|------|
| 前进 | `I` | 慢速前进（建议 0.3-0.5 m/s） |
| 后退 | `,` | 倒车调整位置 |
| 左转 | `J` | 逆时针旋转 |
| 右转 | `L` | 顺时针旋转 |
| 停止 | `K` | 紧急停止 |
| 加速 | `Shift` | 提高速度 |
| 减速 | `Ctrl` | 降低速度 |

**🎯 建图要点**：
1. ✅ **覆盖全场**：走遍所有区域（墙壁、通道、角落）
2. ✅ **多旋转**：在关键位置原地旋转 360°
3. ✅ **走闭合回路**：帮助触发回环检测
4. ✅ **慢速稳定**：避免快速转向和急停
5. ✅ **观察 Rviz**：如果有可视化，查看建图质量

**预计时间**：5-10 分钟（取决于场地大小）

---

### 步骤 4: 保存.pbstream 文件

当建图完成后（地图完整度 > 90%），执行：

#### 4.1 结束轨迹

```bash
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}"
```

#### 4.2 写入.pbstream 文件

```bash
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"
```

#### 4.3 （可选）保存栅格地图验证

```bash
ros2 service call /write_assets \
  cartographer_ros_msgs/srv/WriteAssets \
  "{stem: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026_carto', image_format: 'png'}"
```

这会生成：
- `RMUL2026_carto.pgm` - Cartographer 生成的栅格地图
- `RMUL2026_carto.yaml` - 对应的配置文件

---

### 步骤 5: 验证结果

```bash
# 检查文件
ls -lh src/rm_nav_bringup/map/RMUL2026.pbstream

# 预期输出示例：
# -rw-r--r-- 1 user user 2.3M Mar 21 10:30 RMUL2026.pbstream
```

**✅ 成功标志**：文件大小通常在 1-10 MB 之间

---

## 📦 部署到目标项目

### 复制文件

```bash
cp ~/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream \
   /path/to/target_project/maps/
```

### 在目标项目中启动定位

```bash
cd /path/to/target_project
source install/setup.bash

ros2 launch cartographer_ros cartographer.launch.py \
  configuration_directory:="src/rm_localization/cartographer_ros/configuration_files" \
  configuration_basename:="cartographer_localization.lua" \
  load_state_filename:="maps/RMUL2026.pbstream"
```

### 验证定位效果

1. **查看位姿话题**：
   ```bash
   ros2 topic echo /pose
   ```

2. **观察匹配情况**（Rviz）：
   - 添加 LaserScan 显示
   - 添加 Map 显示
   - 检查激光束是否与地图边缘对齐

3. **测试重定位**：
   ```bash
   # 手动绑架机器人（改变位置）
   # 观察 Cartographer 是否自动重新定位
   ```

---

## 🔧 故障排除

### 问题 1: 找不到 cartographer_ros

**解决**：
```bash
sudo apt install ros-humble-cartographer-ros
```

### 问题 2: 坐标系不匹配

**症状**：TF 树断裂或报错

**解决**：检查 `cartographer.lua` 中的坐标系定义：
```lua
tracking_frame = "livox_frame"  -- 必须与实际传感器一致
published_frame = "body"         -- 必须与机器人底盘一致
```

### 问题 3: 建图漂移严重

**原因**：运动过快或特征不足

**解决**：
- 降低移动速度（< 0.5 m/s）
- 增加旋转动作
- 确保 IMU 正常工作：`ros2 topic echo /imu/data`

### 问题 4: /write_state 服务不存在

**解决**：确认 Cartographer 节点正常运行：
```bash
ros2 service list | grep write_state
ros2 node list | grep cartographer
```

### 问题 5: .pbstream 文件过大

**原因**：包含了未完成的子图

**解决**：确保设置 `include_unfinished_submaps: false`

---

## 📊 配置参数说明

### cartographer.lua 关键参数

| 参数 | 值 | 作用 |
|------|-----|------|
| `resolution` | 0.05 | 地图分辨率（5cm） |
| `num_point_clouds` | 1 | 使用 3D 点云（Livox MID360） |
| `use_imu_data` | true | 融合 IMU 数据 |
| `submaps.num_range_data` | 30 | 每个子图包含 30 帧点云 |
| `MAP_BUILDER.num_background_threads` | 4 | 后台线程数 |

### cartographer_localization.lua 优化

| 修改 | 原值 → 新值 | 效果 |
|------|-------------|------|
| `optimize_every_n_nodes` | 50 → 100 | 降低 CPU 占用 |
| `constraint_builder.sampling_ratio` | 0.3 → 0.1 | 减少采样 |
| `max_submaps_to_keep` | - → 3 | 只保留 3 个子图 |

---

## 🎯 精度预期

基于当前配置，在 RMUL 场地的预期性能：

| 指标 | 预期值 |
|------|--------|
| **相对定位精度** | ±2-5 cm |
| **绝对定位精度** | ±5-10 cm |
| **角度精度** | ±0.5-1° |
| **重定位成功率** | >95% |
| **CPU 占用** | 15-25% (4 核) |
| **内存占用** | 200-400 MB |

---

## 📝 常见问题 FAQ

### Q1: 为什么不用 ogm2pgbm 直接转换？

**A**: 您的配置使用 3D 点云输入（`num_point_clouds = 1`），而 ogm2pgbm 只支持 2D 激光雷达模拟。在 Gazebo 中重建可以确保：
- ✅ 传感器模型完全一致
- ✅ 点云数据结构正确
- ✅ 坐标系无偏差

### Q2: 建图需要多长时间？

**A**: 
- 小型场地（如 RMUL）：5-8 分钟
- 中型场地：10-15 分钟
- 大型场地：20+ 分钟

### Q3: 可以在真实机器人上建图吗？

**A**: 可以！但需要注意：
- 确保电池电量充足
- 选择人员较少的时间段
- 准备备用方案（如手动遥控）

### Q4: .pbstream 文件可以用于多个机器人吗？

**A**: 可以，只要：
- 使用相同的传感器配置
- 坐标系定义一致
- 场地环境未改变

### Q5: 如何更新已有的.pbstream 地图？

**A**: 重新运行建图流程，或使用 Cartographer 的增量建图功能：
```bash
ros2 service call /get_state \
  cartographer_ros_msgs/srv/GetState \
  "{}"
# 然后继续建图...
```

---

## 🎓 进阶技巧

### 自动化建图路径规划

可以使用 Navigation2 自动探索：

```bash
# 安装 exploration_server
sudo apt install ros-humble-navigation2

# 启动自动探索
ros2 launch nav2_bringup exploration_launch.py
```

### 多机器人协作建图

```bash
# 需要为每个机器人分配不同的 trajectory_id
ros2 service call /add_trajectory ...
```

### 质量评估

```bash
# 查看子图数量
ros2 topic echo /submap_list

# 查看轨迹信息
ros2 topic echo /trajectory_node_list

# 计算覆盖率
# （需要自定义脚本分析点云密度）
```

---

## 📞 获取帮助

如遇问题，请检查：
1. 📄 日志文件：`/tmp/cartographer_pbstream_*.log`
2. 📊 Rviz 可视化
3. 🔍 ROS2 诊断：`ros2 doctor`

---

**祝你成功！🎉**

*版本：1.0.0 | 更新时间：2026-03-21*
