# 离线配置包：使用、分发与版本合并说明

> **STATUS（2026-10 合并）**：本文件**合并自** `docs/package/` 下 4 份同主题历史文档（2026-03-19 v1.0.0 体系的四个版本，
> 冻结于 2026-09-08/09-15）。原文已用 `git mv` 移到 [`docs/archive/package/`](../archive/package/)（**保留 git 历史**），
> 每份顶部加了指向本文件的 `STATUS` 行。
>
> **证据约定**：除 §10/§11 另标外，本文**所有内容都是【引用】**——逐字或等价改写自下面 4 份原文（命令、路径、参数、
> 数字一律保留原样，不做"现代化"）。**§11 的 2026-10 现状核对**是本轮新加的，逐条标了【引用】（读仓库源码/文件）或【推断】。
> **本文不是现行操作手册**：现行跑法以 [`../smoke_test_runbook.md`](../smoke_test_runbook.md) 为准；本文件的价值是
> **"当年的分发体系长什么样、为什么这么设计、现在哪些路径已经漂移"**。

## 版本来源

| # | 原文件 | 行数 | 最后提交 | 版本日期 | 在本文中的角色 / 独有内容 |
|---|---|---|---:|---|---|
| 1 | `HZMI_RMSIMULATION_CONFIG_GUIDE.md` | 323 | `e8a07ca 2026-09-15` | v1.0.0（2026-03-19） | **主文档（最新、最全）**：依赖安装 → 编译 → 运行 → 参数 → 地图 → FAQ → 完整目录树 |
| 2 | `配置包使用说明.md` | 468 | `ee6600c 2026-09-08` | 2026-03-19 | **面向创建者**：两份脚本的能力清单、7 份文档各自定位、实际使用场景、文件大小预估、更新策略、发送/接收清单、教学建议、改进建议 |
| 3 | `如何分发配置包.md` | 340 | `ee6600c 2026-09-08` | 2026-03-19 | **面向发送方**：自动/手动两种打包、排除项、tar/zip 命令、完整流程示例、文件必需性表、MD5 校验、与 Git 方式对比 |
| 4 | `README_PACKAGE.md` | 214 | `ee6600c 2026-09-08` | 2026-03-19 | **随包发给接收方**：3 步快速开始、系统要求、包含内容、ROS2/Livox 安装、4 条 FAQ、致谢 |

> 【引用】当时的子索引 [`README.md`](README.md) 把第 1 份标为「★ 主文档（先看这份）」，第 2 份标为「历史版本，内容已被
> 第 1 份覆盖」，第 3、4 份标为「现行（与主文档互补 / 面向上手者）」。**合并时以第 1 份为骨架**，第 2/3/4 份独有的
> 内容分别标注来源（如 `（配置包使用说明.md）`）。

> ⚠️ **路径提醒（【推断】，细节见 §11）**：下面所有 `./start_sentinel.sh`、`./save_pcd.sh`、`./setup_from_package.sh`、
> `./create_config_package.sh` 都是 **2026-03 的仓库根目录版**。2026-10 的现行位置在 `tools/scripts/…`（根目录已无这些
> 脚本），**直接照抄会找不到文件**；§11 给了现行路径与两处仍然存在的 `PROJECT_ROOT` 缺陷。

---

## 1. 这是什么 / 包含什么

### 1.1 定位

【引用】本配置包为赫兹矩阵 RM 仿真导航系统（改造自深圳北理莫斯科大学北极熊战队的 RM 哨兵导航仿真项目，在此向原作者
Lihan Chen 及北极熊战队致以诚挚感谢），支持在 **Ubuntu 22.04 + ROS2 Humble** 环境下运行，**无需 Git/Gitee**，通过
文件夹压缩包传播（`README_PACKAGE.md` 还写明原项目名 `PB_RM_Simulation` / `pb_rm_simulation`）。

### 1.2 功能特性

【引用】
- ✅ Gazebo 仿真环境（RMUC / RMUL 场地）
- ✅ Fast_LIO / Point_LIO 激光 SLAM
- ✅ Navigation2 导航框架
- ✅ 支持仿真和实车两种模式
- ✅ 动态避障、轨迹跟踪、一键偷家

【引用】`README_PACKAGE.md` 的演示功能描述同一组：Gazebo 仿真（RMUC/RMUL 完全仿真）、Fast-LIO/Point-LIO、Navigation2、
动态避障、一键偷家；演示视频：[寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)。

### 1.3 包内结构（历史版本，逐字）

【引用】（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §七）

```text
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

【引用】（`README_PACKAGE.md` 的简版，多列了分发相关三件）

```text
├── start_sentinel.sh             # 主启动脚本
├── setup_from_package.sh         # 一键配置脚本
├── save_pcd.sh                   # 保存 PCD 地图
├── save_grid_map.sh              # 保存栅格地图
├── QUICK_START.md                # 快速开始指南
├── HZMI_RMSIMULATION_CONFIG_GUIDE.md  # 详细文档
└── README_PACKAGE.md             # 本文件
```

### 1.4 核心脚本与文档清单

【引用】（`配置包使用说明.md` §创建的文件列表）

| 文件名 | 作用 | 位置（历史） |
|---|---|---|
| `setup_from_package.sh` | 一键配置脚本 | `/home/weicheng/HzMi_rmsimulation/` |
| `create_config_package.sh` | 创建配置包脚本 | `/home/weicheng/HzMi_rmsimulation/` |

| 文件名 | 用途 | 目标读者 |
|---|---|---|
| `README_PACKAGE.md` | 分发包介绍文档 | 接收方（第一次使用） |
| `QUICK_START.md` | 快速开始指南 | 接收方（快速上手） |
| `HZMI_RMSIMULATION_CONFIG_GUIDE.md` | 详细配置指南 | 接收方（深入使用） |
| `如何分发配置包.md` | 分发方法指南 | 发送方 |
| `快速参考卡片.txt` | 速查手册 | 所有人 |
| `本文件`（= 配置包使用说明） | 总体说明 | 你（项目创建者） |

> 【引用/§11】`QUICK_START.md` 现在**不在仓库里**：它由 `create_config_package.sh` 在打包时用 heredoc **现场生成**（脚本第 99 行起）。

### 1.5 文件大小

【引用】（`配置包使用说明.md` §文件大小预估）

| 内容 | 大小 |
|---|---|
| 源码 | ~50 MB |
| 3D 模型 | ~30 MB |
| 地图文件 | ~20 MB |
| 配置文件 | ~5 MB |
| 文档 | ~2 MB |
| **总计** | **~107 MB** |

【引用】压缩后约 **80-100 MB**。另一份（`如何分发配置包.md`）给的口径是 **100-200 MB**（"完整的配置包大约 100-200MB"，
主要是源码 / 3D 模型 / 地图文件）；两份数字不完全一致，**取哪个都行，量级一致**。太大时可考虑：移除不必要的地图文件、压缩 3D 模型。

---

## 2. 接收方：从零到跑起来

### 2.1 准备基础文件（两种来源）

【引用】（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §二 步骤 1）如果是从**已装好的同学电脑**整目录复制：

```bash
# 假设你从同学那里复制了整个 HzMi_rmsimulation 文件夹
# 复制到你的家目录
cp -r /path/to/classmate/HzMi_rmsimulation ~/HzMi_rmsimulation
```

或者收到的是压缩包（推荐路径，见下）：

```bash
cd ~
tar -xzvf HzMi_rmsimulation_config.tar.gz
# 或者
unzip HzMi_rmsimulation_config.zip
```

### 2.2 三步快速开始

【引用】（`README_PACKAGE.md` / `配置包使用说明.md`，命令逐字）

```bash
# 1) 解压
tar -xzvf HzMi_rmsimulation_config_package_*.tar.gz
cd HzMi_rmsimulation

# 2) 一键配置（自动装依赖 + 编译 + 设权限）
chmod +x setup_from_package.sh
./setup_from_package.sh

# 3) 启动仿真
source install/setup.bash
./start_sentinel.sh
```

【引用】`setup_from_package.sh` 会自动：检查 Ubuntu 22.04 + ROS2 Humble 环境、安装必要的系统依赖、安装 Livox SDK2（如未安装）、
安装 ROS 依赖包、编译工作空间、设置所有脚本权限、显示使用说明。若解压后脚本没有执行权限，先 `chmod +x *.sh`
（`setup_from_package.sh` 也会自动处理权限问题）。特点：交互式安装、自动检测缺失组件、彩色输出、错误处理和恢复机制。

### 2.3 系统要求

【引用】
- 操作系统：Ubuntu 22.04 LTS
- ROS2 版本：ROS2 Humble Hawksbill
- Gazebo：Gazebo Classic 11.10.0
- 内存：建议 8GB 以上
- 显卡：支持 OpenGL 3.3+
- 磁盘：至少 **10 GB** 可用空间（`配置包使用说明.md`）；编译需要 **5-10 分钟**
- 网络：需要（下载依赖）

### 2.4 安装 ROS2 Humble（如果没有）

【引用】（`README_PACKAGE.md` §环境准备）

```bash
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

参考官方教程：<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

### 2.5 安装项目依赖

【引用】（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §二 步骤 2）

```bash
sudo apt update

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
```

### 2.6 安装 Livox SDK2

【引用】（两份文档的命令合并；`README_PACKAGE.md` 多了 `sudo ldconfig`）

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
sudo ldconfig      # README_PACKAGE.md 有这一行，CONFIG_GUIDE 没有
```

### 2.7 编译工作空间

【引用】（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §二 步骤 3）

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

### 2.8 启动仿真

**方式 1：启动脚本（推荐）**（【引用】）

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

【引用】`README_PACKAGE.md` 的同一组示例（标注了每个选项的含义）：

```bash
./start_sentinel.sh --help      # 查看帮助
./start_sentinel.sh             # RMUL 场地（默认）
./start_sentinel.sh -w RMUC     # RMUC 场地
./start_sentinel.sh --lio pointlio   # 使用 Point-LIO
./start_sentinel.sh -m nav      # 已知地图导航
```

**方式 2：手动 launch**（【引用】）

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

### 2.9 控制机器人

【引用】新开一个终端：

```bash
source ~/HzMi_rmsimulation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 按键 | 动作 |
|---|---|
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左移 |
| `l` | 右移 |
| `u` | 左旋 |
| `o` | 右旋 |
| `空格` | 停止 |

---

## 3. 配置说明

### 3.1 主要参数

【引用】（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §四；该文当时已注明"参数已回归各算法包，详见 `docs/params_ownership_checklist.md`"）

| 参数文件 | 关键参数 | 说明 |
|---|---|---|
| `nav2_params_sim.yaml` | `robot_radius`（默认 0.2 米）、`max_velocity`（`[vx, vy, vz]`）、`inflation_radius`（默认 0.7 米） | 导航参数 |
| `fastlio_mid360_sim.yaml` | `extrinsic_T`（雷达到 IMU 的外参平移）、`extrinsic_R`（外参旋转）、`filter_size_surf`（特征点滤波尺寸） | Fast-LIO 参数 |
| `measurement_params_sim.yaml` | 雷达安装位置偏移量、云台中心到雷达中心的距离 | 测量参数 |

### 3.2 修改配置

【引用】

```bash
# 编辑导航参数
vim src/rm_navigation/rm_navigation/params/nav2_params_sim.yaml

# 编辑 LIO 参数
vim src/rm_localization/FAST_LIO/config/fastlio_mid360_sim.yaml

# 重新编译使配置生效
cd ~/HzMi_rmsimulation
colcon build --symlink-install
source install/setup.bash
```

### 3.3 地图管理

【引用】保存地图：

```bash
# 保存 PCD 点云地图
./save_pcd.sh

# 保存栅格地图
./save_grid_map.sh
```

【引用】保存位置：PCD 地图在 `src/rm_nav_bringup/PCD/`，栅格地图在 `src/rm_nav_bringup/map/`。

【引用】用已有地图导航：确认 `src/rm_nav_bringup/PCD/YOUR_MAP.pcd` 与 `src/rm_nav_bringup/map/YOUR_MAP.yaml` 存在后：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=YOUR_MAP \
    mode:=nav \
    lio:=fastlio \
    localization:=icp \
    nav_rviz:=True
```

---

## 4. 发送方：打包与分发

### 4.1 方法一：自动脚本（推荐）

【引用】

```bash
cd ~/HzMi_rmsimulation
./create_config_package.sh
```

【引用】脚本会自动：复制所有必要的源文件、包含配置文档和使用指南、**排除** `build/install/log` 等无关文件、创建 `.gitignore`
防止误用、生成 `tar.gz`（必选）与 `zip`（可选）、清理临时文件。输出：

```text
/home/weicheng/HzMi_rmsimulation/HzMi_rmsimulation_config_package_YYYYMMDD_HHMMSS/
├── HzMi_rmsimulation_config_package_YYYYMMDD_HHMMSS.tar.gz
└── HzMi_rmsimulation_config_package_YYYYMMDD_HHMMSS.zip (如果安装了 zip)
```

【引用】把生成的压缩包通过 **QQ / 微信 / U 盘** 发给同学即可。

### 4.2 方法二：手动打包

【引用】需要包含的文件：

```text
HzMi_rmsimulation/
├── src/                          # 必须！所有源码
├── start_sentinel.sh             # 启动脚本
├── save_pcd.sh                   # 保存地图脚本
├── save_grid_map.sh              # 保存地图脚本
├── setup_from_package.sh         # 一键配置脚本
├── README.md                     # 原始文档
├── README_PACKAGE.md             # 分发包说明
├── HZMI_RMSIMULATION_CONFIG_GUIDE.md  # 详细配置指南
└── QUICK_START.md                # 快速开始
```

【引用】**不要包含**：`build/`（编译产物）、`install/`（安装文件）、`log/`（日志）、`.git/`（不用 Git 传播）、`.vscode/`（可选）。

```bash
# 方法 1: 使用 tar
cd ~
tar -czvf HzMi_rmsimulation_config.tar.gz \
    --exclude='HzMi_rmsimulation/build' \
    --exclude='HzMi_rmsimulation/install' \
    --exclude='HzMi_rmsimulation/log' \
    HzMi_rmsimulation

# 方法 2: 使用 zip
cd ~/HzMi_rmsimulation
zip -r ../HzMi_rmsimulation_config.zip . \
    -x "build/*" \
    -x "install/*" \
    -x "log/*" \
    -x ".git/*"
```

### 4.3 文件必需性表

【引用】（`如何分发配置包.md` §文件说明）

| 文件名 | 作用 | 是否必需 |
|---|---|---|
| `src/` | 所有源代码 | ✅ 必需 |
| `setup_from_package.sh` | 一键配置脚本 | ✅ 必需 |
| `start_sentinel.sh` | 主启动脚本 | ✅ 必需 |
| `save_pcd.sh` | 保存 PCD 地图 | ⭕ 可选 |
| `save_grid_map.sh` | 保存栅格地图 | ⭕ 可选 |

| 文件名 | 作用 | 建议包含 |
|---|---|---|
| `README_PACKAGE.md` | 分发包说明 | ✅ 强烈推荐 |
| `QUICK_START.md` | 快速开始指南 | ✅ 强烈推荐 |
| `HZMI_RMSIMULATION_CONFIG_GUIDE.md` | 详细配置文档 | ✅ 推荐 |
| `README.md` | 原始项目文档 | ⭕ 可选 |

### 4.4 完整流程示例

【引用】发送方：

```bash
# 1. 进入项目目录
cd ~/HzMi_rmsimulation

# 2. 运行打包脚本
./create_config_package.sh

# 3. 等待脚本完成
# 输出：HzMi_rmsimulation_config_package_20260319_143022.tar.gz

# 4. 发送压缩包给同学
# 可以通过 QQ、微信、U 盘等方式
```

【引用】接收方：

```bash
# 1. 接收压缩包，保存到Downloads
cd ~/Downloads

# 2. 移动到家目录并解压
mv HzMi_rmsimulation_config_package_*.tar.gz ~
cd ~
tar -xzvf HzMi_rmsimulation_config_package_*.tar.gz

# 3. 进入项目目录
cd HzMi_rmsimulation

# 4. 运行配置脚本
chmod +x setup_from_package.sh
./setup_from_package.sh

# 5. 等待编译完成（可能需要几分钟）

# 6. 启动仿真
source install/setup.bash
./start_sentinel.sh
```

### 4.5 版本管理

【引用】每次更新代码后重新打包，并写版本号：

```bash
# 更新代码后
./create_config_package.sh

# 记录版本号
echo "v1.0.0 - 2026-03-19" > VERSION.txt
```

【引用】更新通知模板：

```text
【更新通知】HzMi_rmsimulation v1.0.1
更新内容：
- 修复了 XXX 问题
- 优化了 XXX 性能
- 新增了 XXX 功能

请使用新版本替换旧版本
```

【引用】原文的 v1.0.0 版本记录（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §九）：
- **v1.0.0**（2026-03-19）：首次离线配置包发布；支持 Ubuntu 22.04 + ROS2 Humble；包含完整仿真和导航功能。

【引用】版本记录建议：

```text
版本记录:
v1.0.0 (2026-03-19) - 首次发布
v1.0.1 (2026-03-XX) - 修复 XXX 问题
v1.1.0 (2026-04-XX) - 新增 XXX 功能
```

### 4.6 发送时的附带说明（模板）

【引用】（`配置包使用说明.md` / `如何分发配置包.md` 两个模板内容一致）

```text
【HzMi_rmsimulation 配置包 v1.0.0】

包含内容：
- 完整的仿真和导航代码
- 自动配置脚本
- 详细使用文档

使用方法：
1. 解压到任意目录
2. 运行 ./setup_from_package.sh
3. 等待编译完成
4. source install/setup.bash
5. ./start_sentinel.sh

详细文档见压缩包内的 README_PACKAGE.md
```

### 4.7 最佳实践

【引用】
1. **统一版本管理**：每次更新重新打包 + `VERSION.txt` + 发送时注明版本。
2. **提供清晰的说明**：用 §4.6 模板。
3. **测试验证**：发之前找一台干净机器测：

```bash
# 在新环境中测试
docker run -it ubuntu:22.04 bash

# 安装 ROS2 Humble
# 解压配置包
# 运行 setup_from_package.sh
# 验证是否能正常启动
```

4. **MD5 校验**（`如何分发配置包.md` §故障排除）：

```bash
# 发送方
md5sum HzMi_rmsimulation_config_package_*.tar.gz

# 接收方对比 MD5 值
```

### 4.8 与 Git 方式的对比

【引用】

| 特性 | Git/Gitee | 压缩包分发 |
|---|---|---|
| 上手难度 | 需要了解 Git | 解压即用 |
| 网络要求 | 需要稳定网络 | 一次性传输 |
| 版本管理 | 方便 | 手动管理 |
| 更新频率 | 频繁更新 | 定期更新 |
| 适用场景 | 开发协作 | 教学分发 |

---

## 5. 常见问题 / 故障排除

### 5.1 详细版（`HZMI_RMSIMULATION_CONFIG_GUIDE.md` §六）

**Q1: 编译失败 "找不到 livox_sdk"**（【引用】）

```bash
# 确认 Livox SDK2 已正确安装
ls /usr/local/lib/liblivox_sdk_shared.so
# 如果没有，重新安装 Livox SDK2
```

**Q2: Gazebo 无法启动或黑屏**（【引用】）

```bash
# 检查显卡驱动
nvidia-smi  # NVIDIA 显卡

# 尝试使用软件渲染
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch rm_nav_bringup bringup_sim.launch.py ...
```

**Q3: 导航时机器人不移动**（【引用】）：① 检查 `/cmd_vel` 话题是否有数据 ② 检查 costmap 是否有障碍物
③ 调整 `robot_radius` 和 `inflation_radius`。

**Q4: LIO 漂移严重**（【引用】）：① 检查初始位置是否在地图范围内 ② 调整 `extrinsic_T` 和 `extrinsic_R` 外参
③ 降低运动速度。

### 5.2 简版（`README_PACKAGE.md` §常见问题）

【引用】
- **编译失败怎么办？** 确保已安装所有依赖，`./setup_from_package.sh` 会自动检测并安装缺失的依赖。
- **Gazebo 启动黑屏？** 更新显卡驱动，或 `export LIBGL_ALWAYS_SOFTWARE=1`。
- **导航时机器人不动？** 检查是否发布了速度指令，调整 `robot_radius` 参数。
- **LIO 漂移严重？** 检查雷达外参配置，降低运动速度。

### 5.3 接收方故障排除（`如何分发配置包.md` §故障排除）

【引用】
- **接收方编译失败** → 检查是否已安装 ROS2 Humble，`source /opt/ros/humble/setup.bash`。
- **缺少依赖** → `setup_from_package.sh` 会自动安装，确保有网络连接。
- **文件损坏** → 重新发送压缩包，或按 §4.7 用 MD5 校验。

### 5.4 文档阅读顺序（出问题时）

【引用】场景 2（同学收到压缩包后）的阅读顺序：**先看 `README_PACKAGE.md` 了解概况 → 再看 `QUICK_START.md` 快速上手 →
遇到问题查 `HZMI_RMSIMULATION_CONFIG_GUIDE.md`**（`配置包使用说明.md` §场景 2）。

【引用】`QUICK_START.md`（快速开始）→ `HZMI_RMSIMULATION_CONFIG_GUIDE.md`（详细指南）→ `快速参考卡片.txt`（速查手册）；
常见问题速判：编译失败 → 检查 ROS2 环境；Gazebo 黑屏 → 更新显卡驱动；LIO 漂移 → 调整外参配置。

---

## 6. 检查清单

【引用】发送前：

- [ ] 已运行 `create_config_package.sh`
- [ ] 压缩包包含所有必要文件
- [ ] 不包含 `build/install/log`
- [ ] 文档文件齐全
- [ ] 已测试压缩包可正常解压
- [ ] 附带了使用说明
- [ ] 提供了文档联系方式

【引用】接收后：

- [ ] Ubuntu 22.04 + ROS2 Humble
- [ ] 有网络连接（下载依赖）
- [ ] 至少 10GB 可用磁盘空间
- [ ] 编译需要 5-10 分钟
- [ ] 先阅读文档再操作
- [ ] 压缩包 MD5 校验通过
- [ ] `setup_from_package.sh` 执行成功
- [ ] 能够正常启动仿真

---

## 7. 教学与团队培训建议

【引用】（`配置包使用说明.md` §教学建议）

**课堂教学**：① 课前：提前发送配置包、提供安装指南、建答疑群；② 课中：统一解压、一起运行配置脚本、同步启动仿真；
③ 课后作业：修改导航参数、保存自定义地图、尝试不同算法。

**团队培训**：① 新人培训：发送配置包 → 自学文档 → 实际操作练习；② 进阶学习：阅读详细文档、修改配置文件、添加新功能。

---

## 8. 后续可改进项

【引用】（`配置包使用说明.md` §改进建议）

1. **图形化界面**：Python Tkinter 做 GUI 配置工具、更直观的参数配置界面。
2. **在线更新**：版本检测 + 在线下载更新包。
3. **Docker 镜像**：预配置完整镜像，一键启动。
4. **视频教程**：安装配置视频 + 使用演示。

---

## 9. 获取帮助 / 致谢

【引用】
- 官方文档：[Nav2](https://docs.nav2.org/)、[Fast-LIO](https://github.com/LihanChen2004/FAST_LIO/tree/ROS2)、
  [Point-LIO](https://github.com/LihanChen2004/Point-LIO/tree/RM2024_Sentry)
- 视频教程：[寒假在家，怎么调车！？更适合新手宝宝的 RM 导航仿真](https://b23.tv/xSNQGmb)
- 反馈：项目 GitHub Issues / 邮件 / QQ 微信群（原文给的三个渠道）
- 致谢：Livox SDK2、Fast-LIO / Point-LIO、Navigation2、Gazebo、中南大学 FYT 战队、深圳北理莫斯科大学北极熊战队（原作者 Lihan Chen）

---

## 10. 版本差异与取舍（合并时的判断）

| 主题 | `HZMI_…_CONFIG_GUIDE`（主） | `配置包使用说明` | `如何分发配置包` | `README_PACKAGE` |
|---|---|---|---|---|
| 依赖安装（apt 全表） | ✅ 独有 | — | — | 只给 ROS2/Livox |
| `rosdep update/install`、`colcon build` | ✅ 独有 | — | — | — |
| 参数文件逐项说明 | ✅ 独有 | — | — | — |
| 地图管理（保存/已有地图导航） | ✅ 独有 | — | — | 简版 |
| 两份脚本的**能力清单**（⭐ 评级） | — | ✅ 独有 | 简版 | — |
| 7 份文档的**定位表** | — | ✅ 独有 | 简版 | — |
| 实际使用场景（发送/接收逐步） | — | ✅ 独有 | ✅ 独有 | — |
| 文件大小预估 | — | ✅ 独有 | 口径 100-200MB | — |
| 更新策略 / 版本记录模板 | — | ✅ 独有 | ✅ 独有 | — |
| 教学 / 团队培训建议 | — | ✅ 独有 | — | — |
| 改进建议（GUI/在线更新/Docker/视频） | — | ✅ 独有 | — | — |
| 手动 tar/zip 打包 + 排除项 | — | 一句"参考" | ✅ 独有 | — |
| MD5 校验、干净机器/docker 测试 | — | — | ✅ 独有 | — |
| 与 Git 方式对比表 | — | — | ✅ 独有 | — |
| 发送前/接收后检查清单 | — | ✅ 独有 | ✅ 独有 | — |
| 系统要求（含 Gazebo 11.10.0） | — | 环境要求 | — | ✅ 独有 |
| ROS2 Humble 安装命令块 | — | — | — | ✅ 独有 |
| FAQ | ✅ 4 条详细 | 3 条速判 | 3 条接收方 | ✅ 4 条简版 |

【引用】结论（原文子索引的判断，合并后仍成立）：**`HZMI_RMSIMULATION_CONFIG_GUIDE.md` 是最新最全的主文档**；
`配置包使用说明.md` 是**面向创建者**的总览（含大量"当时做了什么"的记录，内容已被主文档覆盖但**独有清单/检查表/教学建议**）；
`如何分发配置包.md` 与主文档**互补**（打包/分发视角）；`README_PACKAGE.md` 是**随包发出去的那份**（面向上手者）。

---

## 11. 2026-10 现状核对：路径漂移与两处 `PROJECT_ROOT` 缺陷

> 本节是**本轮文档整理新加的**，不是原文内容。原文冻结在 2026-03/09，仓库后来把根目录脚本统一挪进了 `tools/scripts/`。
> 下面每条都标明证据类型；**没有运行任何脚本**（只读源码/`ls`）。

### 11.1 脚本位置（【引用】= `ls` / `git ls-files` 的当前结果）

| 原文里的路径 | 2026-10 的现行路径 |
|---|---|
| `./start_sentinel.sh` | `tools/scripts/control/start_sentinel.sh` |
| `./save_pcd.sh` | `tools/scripts/mapping/save_pcd.sh` |
| `./save_grid_map.sh` | `tools/scripts/mapping/save_grid_map.sh` |
| `./setup_from_package.sh` | `tools/scripts/setup_from_package.sh` |
| `./create_config_package.sh` | `tools/scripts/create_config_package.sh` |
| `QUICK_START.md`（仓库文件） | **不存在**；由 `create_config_package.sh` 打包时生成 |
| `快速参考卡片.txt` | `docs/mapping/快速参考卡片.txt`（历史文件，未跟随本节迁移） |

### 11.2 两处 `PROJECT_ROOT` 少算一层（【引用】源码 +【推断】后果）

【引用】当前脚本里的定义（`grep -n PROJECT_ROOT=`）：

```text
tools/scripts/setup_from_package.sh:12:PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
tools/scripts/create_config_package.sh:12:PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
tools/scripts/control/start_sentinel.sh:11:PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"   # 正确
tools/scripts/mapping/save_pcd.sh:5:PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"          # 正确
tools/scripts/mapping/save_grid_map.sh:4:PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"     # 正确
```

【推断】前两个脚本仍在 `tools/scripts/` 这一层，`${SCRIPT_DIR}/..` 只到 `.../tools`（不是仓库根）⇒
`setup_from_package.sh` 会 `cd tools` 后再做目录检查；`create_config_package.sh` 里 `${PROJECT_ROOT}/tools/scripts/control/start_sentinel.sh`
会变成 `tools/tools/scripts/...`（不存在，脚本里对 start_sentinel 有 `2>/dev/null || 跳过` 的兜底），
`${PROJECT_ROOT}/docs/package/HZMI_RMSIMULATION_CONFIG_GUIDE.md` 会变成 `tools/docs/package/...`（不存在）。
**这与 `docs/mapping/README.md` 记录的 `save_*` 脚本同类缺陷（已修为 `${SCRIPT_DIR}/../../..`）**；
本次是**文档整理，未改脚本**，登记在此供后续修（改法参照 `save_pcd.sh` 的 `../../..` 口径）。

### 11.3 打包脚本 `create_config_package.sh` 的输入清单（【引用】源码）

- 第 60 行：`cp "${PROJECT_ROOT}/tools/scripts/control/start_sentinel.sh" "$TEMP_DIR/"` ⇒ 已按**新路径**取启动脚本；
- 第 68 行：`cp "${PROJECT_ROOT}/docs/package/HZMI_RMSIMULATION_CONFIG_GUIDE.md" "$TEMP_DIR/"` ⇒ 取本目录第 1 份（**已归档到
  `docs/archive/package/`**，见 §11.4：这条 `cp` 的源路径也需要跟着改，否则打包时会缺主文档）；
- 第 99-184 行：`QUICK_START.md` 是 heredoc **现场生成**的（内容 = 三步快速开始 + 常用命令），不是仓库文件。

### 11.4 本次合并对 `docs/package/` 的改动

| 变化 | 说明 |
|---|---|
| 4 份原文 → `docs/archive/package/` | `git mv` 保留历史；每份顶部加 `STATUS` 指向本文件 |
| 本文件 = `docs/package/config_package_guide_merged.md` | 唯一"合并版"；`docs/package/README.md` 子索引已改为指向它 + 归档 |
| 仍然有效的内容 | 所有命令/参数/流程（§1–§9）作为**历史版本**保留；现行跑法以 `docs/smoke_test_runbook.md` 为准 |

> 【推断】上面 11.2/11.3 的路径问题**不影响本文件作为"当年用法记录"的价值**；但如果今天真要按这份文档发一个包，
> 应先修 `setup_from_package.sh` / `create_config_package.sh` 的 `PROJECT_ROOT` 与主文档 `cp` 源路径（**属于代码修改，
> 不在本次文档整理范围内**）。
