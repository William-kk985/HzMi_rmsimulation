# 📦 HzMi_rmsimulation 离线配置包

**赫兹矩阵RM仿真导航系统**

> 本配置包改造自 [深圳北理莫斯科大学 北极熊战队](https://gitee.com/SMBU-POLARBEAR/HzMi_rmsimulation) 的 RM 哨兵导航仿真项目（原 `PB_RM_Simulation` / `pb_rm_simulation`），在此向原作者 Lihan Chen 及北极熊战队致以诚挚感谢。

这是一个完整的 ROS2 导航仿真配置包，无需 Git/Gitee，通过文件夹压缩包传播。

---

## ⚡ 快速开始（3 步完成）

### 1️⃣ 运行配置脚本

```bash
cd HzMi_rmsimulation
chmod +x setup_from_package.sh
./setup_from_package.sh
```

配置脚本会自动：
- ✅ 检查 Ubuntu 22.04 + ROS2 Humble 环境
- ✅ 安装必要的系统依赖
- ✅ 编译工作空间
- ✅ 设置所有脚本权限

### 2️⃣ 启动仿真

```bash
source install/setup.bash
./start_sentinel.sh
```

### 3️⃣ 控制机器人

新打开一个终端：

```bash
source ~/HzMi_rmsimulation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🎮 使用说明

### 启动不同配置

```bash
# 查看帮助
./start_sentinel.sh --help

# RMUL 场地（默认）
./start_sentinel.sh

# RMUC 场地
./start_sentinel.sh -w RMUC

# 使用 Point-LIO
./start_sentinel.sh --lio pointlio

# 已知地图导航
./start_sentinel.sh -m nav
```

### 保存地图

```bash
# 保存点云地图
./save_pcd.sh

# 保存栅格地图
./save_grid_map.sh
```

---

## 📋 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2 版本**: ROS2 Humble Hawksbill
- **Gazebo**: Gazebo Classic 11.10.0
- **内存**: 建议 8GB 以上
- **显卡**: 支持 OpenGL 3.3+

---

## 📁 包含内容

```
HzMi_rmsimulation/
├── src/                          # 源码目录
│   ├── rm_simulation/            # Gazebo 仿真模型
│   ├── rm_localization/          # SLAM 算法 (Fast-LIO, Point-LIO, ICP)
│   ├── rm_navigation/            # Navigation2 配置
│   ├── rm_perception/            # 点云处理
│   └── rm_nav_bringup/           # 启动配置
├── start_sentinel.sh             # 主启动脚本
├── setup_from_package.sh         # 一键配置脚本
├── save_pcd.sh                   # 保存 PCD 地图
├── save_grid_map.sh              # 保存栅格地图
├── QUICK_START.md                # 快速开始指南
├── HZMI_RMSIMULATION_CONFIG_GUIDE.md  # 详细文档
└── README_PACKAGE.md             # 本文件
```

---

## 🔧 环境准备

如果还没有 ROS2 环境，请先安装：

### 安装 ROS2 Humble

参考官方教程：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

```bash
# 简要步骤
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 安装 Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
sudo ldconfig
```

---

## ❓ 常见问题

### Q: 编译失败怎么办？
A: 确保已安装所有依赖，运行 `./setup_from_package.sh` 会自动检测并安装缺失的依赖。

### Q: Gazebo 启动黑屏？
A: 尝试更新显卡驱动，或使用软件渲染：
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Q: 导航时机器人不动？
A: 检查是否发布了速度指令，调整 `robot_radius` 参数。

### Q: LIO 漂移严重？
A: 检查雷达外参配置，降低运动速度。

详细问题请查看 `HZMI_RMSIMULATION_CONFIG_GUIDE.md`

---

## 📖 文档说明

- **QUICK_START.md** - 快速上手指南，5 分钟开始使用
- **HZMI_RMSIMULATION_CONFIG_GUIDE.md** - 完整配置文档，包含详细说明和故障排除
- **README_PACKAGE.md** - 本文件，包的基本介绍

---

## 🎥 功能演示

- **Gazebo 仿真**: RMUC/RMUL 场地完全仿真
- **Fast-LIO/Point-LIO**: 高性能激光 SLAM
- **Navigation2**: 强大的导航框架
- **动态避障**: 实时躲避移动障碍物
- **一键偷家**: 自动规划路径到目标点

演示视频：[寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)

---

## 🙏 致谢

感谢以下开源项目：
- Livox SDK2
- Fast-LIO / Point-LIO
- Navigation2
- Gazebo
- 中南大学 FYT 战队

---

## 📞 获取帮助

如遇问题，请查看：
1. `HZMI_RMSIMULATION_CONFIG_GUIDE.md` 详细文档
2. `QUICK_START.md` 快速指南
3. 相关 GitHub Issues

---

**祝你使用愉快！🎉**

---

*版本：1.0.0*  
*更新时间：2026-03-19*
