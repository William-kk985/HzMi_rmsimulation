# HzMi_rmsimulation 目录架构说明

> 本文档描述仓库**当前实际结构**（2026-09 参数回归 R1 完成后的状态），供团队日常查阅与新成员上手。
> 相关文档：`docs/rm_bench_refactor_plan.md`（改造路线）、`docs/params_ownership_checklist.md`（参数归属清单）、`docs/rm_algorithm_catalog.md`（算法选型）。

---

## 一、项目定位

- **ROS2 humble 的多算法仿真实验台**：Gazebo 场地 + 旋转 Mid360 仿真 + FAST-LIO/Point-LIO 定位 + Nav2 导航，用于**测试/对比不同职能的算法或成品包**；
- 传感器构型与真车一致（旋转 Mid360 + 麦轮全向），仿真结论可迁移；
- **不追求 sim2real 一键打通**：真车配置冻结，真车部署在"按包清单导出"阶段另行处理。

---

## 二、顶层目录总览

| 路径 | 作用 | 是否参与 colcon 构建 |
|---|---|---|
| `src/` | **全部 ROS2 功能包**（19 个，按角色域分组） | ✅ |
| `third_party/` | **官方原版第三方库参考**（只读对照，不编译） | ❌（`COLCON_IGNORE`） |
| `tools/` | 工具脚本：`tools/*.py`＝Python 工具；`tools/scripts/`＝Shell 脚本 | ❌（无 package） |
| `docs/` | 规划/清单/操作指南文档 | ❌ |
| `archive/` | 一次性报告归档 | ❌ |
| `build/` `install/` `log/` | colcon 构建产物（已 gitignore） | — |
| `dockerfile`、`.devcontainer/` | 容器化开发环境 | — |
| `.docs/` | README 引用的图片/GIF | — |
| `.vscode/`、`.venv/` | 本地 IDE / Python 工具环境（已 gitignore） | — |
| `.gitmodules` | 9 个子模块登记表 | — |

---

## 三、`src/` —— 按"角色域"分组的包

### 3.1 域 ↔ 包 ↔ 角色速查

| 域目录 | 包（ROS 包名） | 角色 | 源码形态 |
|---|---|---|---|
| `rm_simulation/` | `hzmi_rm_simulation` | Gazebo 场地（RMUC/RMUL/RMUL2026）+ 哨兵机器人模型 | 本仓直接管理 |
| | `ros2_livox_simulation`（目录名 `livox_laser_simulation_RO2`） | Gazebo 里仿真 Mid360，发 `/livox/lidar` | 本仓直接管理 |
| `rm_driver/` | `livox_ros_driver2` | 真车雷达驱动 + **CustomMsg 消息定义**（仿真不启动节点，但编译期必需） | git 子模块 |
| `rm_perception/` | `linefit_ground_segmentation` / `linefit_ground_segmentation_ros` | 点云地面分割（`/segmentation/obstacle`） | 本仓直接管理 |
| | `pointcloud_to_laserscan` | 3D 障碍点云 → 2D `/scan`（**降维，2D 世界唯一入口**） | 本仓直接管理 |
| | `imu_complementary_filter` | IMU 滤波 → `/imu/data`（喂 FAST-LIO） | 本仓直接管理 |
| `rm_localization/` | `fast_lio` | 里程计 + 3D 建图（一体） | **git 子模块（自有 fork）** |
| | `point_lio` | 高频里程计 + 建图（`fast_lio` 的替身） | **git 子模块（自有 fork）** |
| | `icp_registration` | 3D 重定位（吃 `.pcd`） | 本仓直接管理 |
| | `slam_toolbox` | 2D 建图 / 纯定位（官方源码 vendored，**编译覆盖 apt**） | 内嵌源码（去 .git） |
| | `cartographer_ros` | 2D 建图 / 纯定位（**ros2-gbp humble 官方 ament 源码**，编译覆盖 apt） | 内嵌源码（去 .git） |
| `rm_navigation/` | `rm_navigation` | Nav2 组装（launch + **nav2 参数** + rviz） | 本仓直接管理 |
| | `teb_local_planner` / `teb_msgs` | TEB 局部规划器（Nav2 插件） | git 子模块 |
| | `costmap_converter` / `costmap_converter_msgs` | TEB 依赖（costmap→几何图形） | git 子模块 |
| | `fake_vel_transform` | 云台旋转速度补偿胶水 | 本仓直接管理 |
| `rm_nav_bringup/` | `rm_nav_bringup` | **总装层**：launch 入口 + 平台参数 + 地图/PCD/rviz 资产 | 本仓直接管理 |

> 19 个 colcon 包 = 上表除子模块内嵌包外的全部；`slam_toolbox/lib/karto_sdk` 由 slam_toolbox 自带、不单独出现在 colcon 列表。

### 3.2 数据流（仿真，默认 mapping/nav 组合）

```
Gazebo(hzmi_rm_simulation 世界 + 机器人)
   ├─ 旋转 Mid360 仿真插件(ros2_livox_simulation) ─► /livox/lidar (CustomMsg) ─┬─► fast_lio / point_lio ─► 里程计位姿 + 3D 图
   └─ IMU 插件 ─► /livox/imu ─► imu_complementary_filter ─► /imu/data ────────┘
                                                          │
                        /livox/lidar/pointcloud ─► linefit 地面分割 ─► /segmentation/obstacle
                                                          │
                                          pointcloud_to_laserscan ─► /scan
                                                          │
            ┌─────────────────────────────────────────────┴──────────────┐
   2D 建图/定位: slam_toolbox / cartographer_ros            2D 导航: Nav2(rm_navigation)
            └─────────────────────────────────────────────┬──────────────┘
                                          fake_vel_transform ─► /cmd_vel ─► 底盘
```

### 3.3 `rm_nav_bringup`（总装层）内部

```
rm_nav_bringup/
├── launch/
│   ├── bringup_sim.launch.py        # 仿真总入口（world/mode/lio/localization 参数）
│   ├── bringup_real.launch.py       # 真车入口（reality 已冻结）
│   └── cartographer_sim.launch.py   # cartographer 专用启动（默认引包内 lua）
├── config/
│   ├── simulation/measurement_params_sim.yaml   # ★平台外参（不属于算法，按规则留此）
│   └── reality/                                 # ★冻结快照（FROZEN.md），不再维护
├── map/        # 各场地地图产物 + empty_map（mapping 模式用）
├── PCD/        # 3D 点云图（icp_registration 用）
├── rviz/       # fastlio/pointlio 可视化配置
└── urdf/       # 机器人描述（sim/real）
```

---

## 四、参数归属规则（R1–R5）与当前落地

**规则**：算法参数一律回归**算法包自己的 `config/`**；总装层只留平台参数与冻结内容；每个包由自身 CMake 安装配置。

| 参数（原都在 bringup） | 现在归属 | 状态 |
|---|---|---|
| `fastlio_mid360_sim.yaml` | `rm_localization/FAST_LIO/config/`（fork 内） | ✅ |
| `pointlio_mid360_sim.yaml`（含原 launch 内嵌调参） | `rm_localization/point_lio/config/`（fork 内） | ✅ |
| `icp_registration_sim.yaml` | `rm_localization/icp_registration/config/` | ✅ |
| `segmentation_sim.yaml` | `rm_perception/.../linefit_ground_segmentation_ros/config/` | ✅ |
| `mapper_params_*_sim.yaml` | `rm_localization/slam_toolbox/config/` | ✅ |
| `nav2_params_sim.yaml` | `rm_navigation/rm_navigation/params/` | ✅ |
| `cartographer.lua` / `cartographer_localization.lua` | `rm_localization/cartographer_ros/configuration_files/`（官方示例同目录） | ✅ |
| imu 滤波 / laserscan / fake_vel 参数（原 launch 内嵌） | 各自包 `config/` | ✅ |
| `measurement_params_sim.yaml` | `rm_nav_bringup/config/simulation/`（**平台参数，规则③**） | 保留 |
| `config/reality/*` | `rm_nav_bringup/config/reality/`（**冻结，规则④**） | 冻结 |

**地图产物配对铁律**：`.pgm`→AMCL ｜ `.posegraph`→slam_toolbox(localization) ｜ `.pcd`→icp_registration ｜ `.pbstream`→Cartographer 纯定位。

---

## 五、`third_party/` —— 官方原版参考（只读，不编译）

| 目录 | 上游 | 锁定 commit | 用途 |
|---|---|---|---|
| `fast_lio` | hku-mars/FAST_LIO | 7cc4175 | 与 `src/.../FAST_LIO`（改动版）对照 |
| `point_lio` | hku-mars/Point-LIO | 4b86a46 | 与 `src/.../point_lio` 对照 |
| `cartographer` | cartographer-project/cartographer | 877157a | 核心库参考（ROS2 用法由 `src/.../cartographer_ros` 提供） |
| `nav2` | ros-navigation/navigation2 (humble) | 3c3db59 | **完整源码对照学习**（本工程运行用 apt + 自研 rm_navigation 组装） |

规则：**只读**；`third_party/COLCON_IGNORE` 保证不参与编译；新增候选库用 `git submodule add` 并登记到 `third_party/README.md`。

---

## 六、子模块清单（9 个，`git clone --recursive` 可完整还原）

| 路径 | 上游 | 性质 |
|---|---|---|
| `src/rm_driver/livox_ros_driver2` | gitee SMBU-POLARBEAR（humble 分支） | 第三方 |
| `src/rm_localization/FAST_LIO` | **github.com/William-kk985/FAST_LIO** | **自有 fork**（含本地适配 + ikd-Tree 扁平化） |
| `src/rm_localization/point_lio` | **github.com/William-kk985/Point-LIO** | **自有 fork** |
| `src/rm_navigation/teb_local_planner` | rst-tu-dortmund/teb_local_planner | 第三方 |
| `src/rm_navigation/costmap_converter` | LihanChen2004/costmap_converter | 第三方 |
| `third_party/fast_lio` | hku-mars/FAST_LIO | 原版参考 |
| `third_party/point_lio` | hku-mars/Point-LIO | 原版参考 |
| `third_party/cartographer` | cartographer-project/cartographer | 原版参考 |
| `third_party/nav2` | ros-navigation/navigation2 (humble) | 原版参考 |

> 内嵌源码（无 .git，随主仓提交）：`src/rm_localization/slam_toolbox`、`src/rm_localization/cartographer_ros`。

---

## 七、`tools/` —— 脚本入口

```
tools/
├── *.py                     # Python 工具（评测/后处理等，按需新增）
└── scripts/                 # Shell 脚本（原顶层 scripts/ 已合并至此）
    ├── build.sh
    ├── control/start_sentinel.sh        # ★一键启动仿真
    ├── control/improved_teleop.sh
    ├── mapping/quick_start_cartographer.sh
    ├── mapping/generate_cartographer_pbstream.sh
    ├── mapping/save_pcd.sh  save_grid_map.sh
    ├── create_config_package.sh  setup_from_package.sh
    └── ../README.md
```

常用：

```bash
tools/scripts/control/start_sentinel.sh                      # 默认 RMUL2026 + mapping + fastlio
tools/scripts/control/start_sentinel.sh -w RMUC -m nav --lio pointlio
# 或手动
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping lio:=fastlio
```

---

## 八、`docs/` 文档索引

| 文档 | 内容 |
|---|---|
| `docs/architecture.md` | **本文件**：当前目录架构总览 |
| `docs/rm_bench_refactor_plan.md` | 实验台改造路线（M0–M4）、现状诊断、已知问题清单 |
| `docs/params_ownership_checklist.md` | 参数归属清单（R1 逐项状态） |
| `docs/rm_algorithm_catalog.md` | 2D/3D 建图·重定位算法候选池（含官方链接） |
| `docs/rm_algorithm_overview.md` | 角色 × 一体性 × 两大场景 |
| `docs/rm_algolab_plan.md` | 宏架构早期规划（历史草案） |
| `docs/mapping/*` | 建图操作指南（cartographer 建图/存 pbstream） |
| `docs/package/*` | 配置包分发说明 |

---

## 九、构建与运行要点

```bash
# 构建（19 个包）
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 一键启动（推荐）
tools/scripts/control/start_sentinel.sh
```

注意：
- 若 shell 里 conda/miniconda 的 `python3` 抢占 PATH，CMake 会因缺 `catkin_pkg` 报错 → 构建时用系统 python（`PATH=/usr/local/bin:/usr/bin:/bin:...`）；
- 依赖 `ros-humble-dwb-critics`（teb 编译需要）；
- `third_party/` 不参与构建（COLCON_IGNORE）；`slam_toolbox`、`cartographer_ros` 为源码 overlay，**编译产物优先于 apt 版**。

### 仿真参数速查（`bringup_sim.launch.py`）

| 参数 | 取值 | 说明 |
|---|---|---|
| `world` | `RMUC` / `RMUL` / `RMUL2026`（默认） | 场地（同时决定 map/PCD 前缀） |
| `mode` | `mapping` / `nav` | 边建图边导航 / 已知地图导航 |
| `lio` | `fastlio`（默认） / `pointlio` | 里程计选择 |
| `localization` | `slam_toolbox` / `amcl` / `icp`（仅 nav 模式） | 重定位方式 |
| `lio_rviz` / `nav_rviz` | `True` / `False` | 可视化开关 |

---

## 十、当前待办（不阻塞）

1. **TF 桥收口**：`bringup_sim` 中 `camera_init→map`、`body→odom`、`base_link→base_link_fake` 为帧对齐调试手段，需理清帧树后正式化；
2. **`RMUL2026.pbstream` 仅 526B**（疑空）→ 需重新建图；RMUL2026 亦缺 `.posegraph`；
3. **整体实跑验证**：`start_sentinel.sh` 的 mapping/nav × fastlio/pointlio 组合逐一跑通（含参数回归后的行为确认）。
