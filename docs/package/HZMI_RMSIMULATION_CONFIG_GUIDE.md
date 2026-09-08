# HzMi_rmsimulation 离线配置包使用指南

## 📦 一、配置包介绍

本配置包为赫兹矩阵RM仿真导航系统（改造自深圳北理莫斯科大学北极熊战队的 RM 哨兵导航仿真项目，在此向原作者 Lihan Chen 及北极熊战队致以诚挚感谢），支持在 Ubuntu 22.04 + ROS2 Humble 环境下运行，无需 Git/Gitee，通过文件夹压缩包传播。

### 功能特性
- ✅ Gazebo 仿真环境（RMUC/RMUL场地）
- ✅ Fast_LIO / Point_LIO 激光 SLAM
- ✅ Navigation2 导航框架
- ✅ 支持仿真和实车两种模式
- ✅ 动态避障、轨迹跟踪、一键偷家

---

## 🚀 二、快速开始（3 步完成配置）

### 步骤 1：准备基础文件

从已安装好的同学电脑复制以下文件到你的工作目录：

```bash
# 假设你从同学那里复制了整个 HzMi_rmsimulation 文件夹
# 复制到你的家目录
cp -r /path/to/classmate/HzMi_rmsimulation ~/HzMi_rmsimulation
```

或者如果同学给你发送了压缩包：

```bash
# 解压压缩包
cd ~
tar -xzvf HzMi_rmsimulation_config.tar.gz
# 或者
unzip HzMi_rmsimulation_config.zip
```

### 步骤 2：安装系统依赖

```bash
# 1. 更新软件源
sudo apt update

# 2. 安装 ROS2 Humble 基础包（如果未安装）
# 参考：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 3. 安装项目依赖包
cd ~/HzMi_rmsimulation
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-robot-localization \
    ros-humble-depthimage-to-laserscan \
    ros-humble-interactive-markers \
    ros-humble-gazebo-ros-pkgs \
    liblivox-sdk-dev

# 4. 安装 Livox SDK2（如果未安装）
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 步骤 3：编译工作空间

```bash
cd ~/HzMi_rmsimulation

# 设置环境变量
source /opt/ros/humble/setup.bash

# 安装 ROS 依赖
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

# 编译
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

---

## 🎮 三、运行仿真

### 方式 1：使用启动脚本（推荐）

```bash
cd ~/HzMi_rmsimulation

# 查看帮助
./start_sentinel.sh --help

# 默认启动（RMUL场地 + 建图模式 + Fast-LIO）
./start_sentinel.sh

# 自定义配置
./start_sentinel.sh -w RMUC -m nav --lio pointlio

# 启用LIO可视化
./start_sentinel.sh --lio-rviz True
```

### 方式 2：手动启动

```bash
cd ~/HzMi_rmsimulation
source install/setup.bash

# 启动仿真系统
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=fastlio \
    lio_rviz:=False \
    nav_rviz:=True
```

### 控制机器人

新打开一个终端：

```bash
source ~/HzMi_rmsimulation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**按键说明：**
- `i` - 前进
- `,` - 后退
- `j` - 左移
- `l` - 右移
- `u` - 左旋
- `o` - 右旋
- `空格` - 停止

---

## ⚙️ 四、配置说明

### 4.1 主要参数

在 `src/rm_nav_bringup/config/simulation/` 目录下：

#### 1. **nav2_params_sim.yaml** - 导航参数
- `robot_radius`: 机器人半径（默认 0.2 米）
- `max_velocity`: 最大速度 `[vx, vy, vz]`
- `inflation_radius`: 膨胀半径（默认 0.7 米）

#### 2. **fastlio_mid360_sim.yaml** - Fast-LIO 参数
- `extrinsic_T`: 雷达到 IMU 的外参平移
- `extrinsic_R`: 雷达到 IMU 的外参旋转
- `filter_size_surf`: 特征点滤波尺寸

#### 3. **measurement_params_sim.yaml** - 测量参数
- 雷达安装位置偏移量
- 云台中心到雷达中心的距离

### 4.2 修改配置

```bash
# 编辑导航参数
vim src/rm_nav_bringup/config/simulation/nav2_params_sim.yaml

# 编辑 LIO 参数
vim src/rm_nav_bringup/config/simulation/fastlio_mid360_sim.yaml

# 重新编译使配置生效
cd ~/HzMi_rmsimulation
colcon build --symlink-install
source install/setup.bash
```

---

## 🗺️ 五、地图管理

### 保存地图

```bash
# 保存 PCD 点云地图
./save_pcd.sh

# 保存栅格地图
./save_grid_map.sh
```

保存的地图位于：
- PCD 地图：`src/rm_nav_bringup/PCD/`
- 栅格地图：`src/rm_nav_bringup/map/`

### 使用已有地图导航

```bash
# 确保地图文件存在
# src/rm_nav_bringup/PCD/YOUR_MAP.pcd
# src/rm_nav_bringup/map/YOUR_MAP.yaml

# 启动导航
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=YOUR_MAP \
    mode:=nav \
    lio:=fastlio \
    localization:=icp \
    nav_rviz:=True
```

---

## 🔧 六、常见问题

### Q1: 编译失败 "找不到 livox_sdk"
**解决：**
```bash
# 确认 Livox SDK2 已正确安装
ls /usr/local/lib/liblivox_sdk_shared.so
# 如果没有，重新安装 Livox SDK2
```

### Q2: Gazebo 无法启动或黑屏
**解决：**
```bash
# 检查显卡驱动
nvidia-smi  # NVIDIA 显卡

# 尝试使用软件渲染
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch rm_nav_bringup bringup_sim.launch.py ...
```

### Q3: 导航时机器人不移动
**解决：**
1. 检查 `/cmd_vel` 话题是否有数据
2. 检查 costmap 是否有障碍物
3. 调整 `robot_radius` 和 `inflation_radius`

### Q4: LIO 漂移严重
**解决：**
1. 检查初始位置是否在地图范围内
2. 调整 `extrinsic_T` 和 `extrinsic_R` 外参
3. 降低运动速度

---

## 📁 七、目录结构

```
HzMi_rmsimulation/
├── src/                          # 源码目录
│   ├── rm_simulation/            # 仿真模型和世界
│   │   └── hzmi_rm_simulation/
│   │       ├── launch/           # 启动文件
│   │       ├── world/            # Gazebo 世界文件 (RMUC/RMUL)
│   │       ├── urdf/             # 机器人模型
│   │       └── meshes/           # 3D 模型
│   ├── rm_localization/          # 定位算法
│   │   ├── FAST_LIO/             # Fast-LIO
│   │   ├── point_lio/            # Point-LIO
│   │   └── icp_registration/     # ICP 配准
│   ├── rm_navigation/            # 导航模块
│   │   └── fake_vel_transform/   # 速度变换
│   ├── rm_perception/            # 感知模块
│   │   ├── linefit_ground_segmentation_ros2/
│   │   └── imu_complementary_filter/
│   └── rm_nav_bringup/           # 启动配置
│       ├── config/               # 配置文件
│       │   ├── simulation/       # 仿真配置
│       │   └── reality/          # 实车配置
│       ├── launch/               # Launch 文件
│       ├── map/                  # 栅格地图
│       └── PCD/                  # 点云地图
├── start_sentinel.sh             # 主启动脚本
├── save_pcd.sh                   # 保存 PCD 地图
├── save_grid_map.sh              # 保存栅格地图
└── README.md                     # 详细说明
```

---

## 📞 八、获取帮助

### 官方文档
- [Nav2 官方文档](https://docs.nav2.org/)
- [Fast-LIO GitHub](https://github.com/LihanChen2004/FAST_LIO/tree/ROS2)
- [Point-LIO GitHub](https://github.com/LihanChen2004/Point-LIO/tree/RM2024_Sentry)

### 视频教程
- [寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)

### 问题反馈
联系项目维护者或提交 Issue。

---

## 📝 九、版本记录

- **v1.0.0** (2026-03-19)
  - 首次离线配置包发布
  - 支持 Ubuntu 22.04 + ROS2 Humble
  - 包含完整仿真和导航功能

---

## 🙏 十、致谢

感谢以下开源项目：
- Livox SDK2
- Fast-LIO / Point-LIO
- Navigation2
- Gazebo
- 中南大学 FYT 战队

---

**祝你使用愉快！🎉**
