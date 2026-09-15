# THIRD_PARTY_NOTICES —— 第三方组件署名与许可证

> 本仓库（HzMi_rmsimulation）是**集成与适配**工程：核心算法、驱动、工具库的**版权与著作权归各上游原作者所有**；
> 我们做的是"选型 + 编排 + 适配 + 参数调优"，并对上游代码做了少量本地修改（见下表"我们的改动"列）。
> 本文件用于集中声明来源、作者与许可证，替代"靠搜索排名署名"的做法。

## 一、项目本身的来源与致谢

- 本仓库改造自 **深圳北理莫斯科大学 北极熊战队** 的开源项目（原 `PB_RM_Simulation` / `pb_rm_simulation`，作者 **Lihan Chen**），README 第 5 行与"致谢"节已署名；
- 导航框架参考 **中南大学 FYT 战队 RM 哨兵上位机算法（CSU-RM-Sentry）**；
- 云台旋转速度补偿思路来自 **深技大 Shockley**；
- Mid360 点云仿真参考 `livox_laser_simulation` 系列开源实现；
- 其余参考见 README"致谢（不分先后）"一节。

## 二、组件清单

| 组件（本仓路径） | 上游 / 原作者 | 许可证 | 本仓形态 | 我们的改动 |
|---|---|---|---|---|
| `src/rm_localization/FAST_LIO` | hku-mars/FAST_LIO → LihanChen2004/FAST_LIO（ROS2 fork） | **GPL-2.0**（LICENSE 文件；⚠️ package.xml 误写 BSD，见 §四-1） | **自有 fork**（`William-kk985/FAST_LIO`，快照根，基于上游 `5d9dc72`） | 本地适配；config 增 sim 参数；ikd-Tree 扁平化入库（含本地新增 `ikd_Tree.cpp/h`） |
| `src/rm_localization/point_lio` | hku-mars/Point-LIO → LihanChen2004/Point-LIO | **BSD-3-Clause**（LOAM 派生：©2013 Ji Zhang/CMU，©2016 SRI） | **自有 fork**（`William-kk985/Point-LIO`，完整历史保留） | config 增 sim 参数与调参键 |
| `src/rm_localization/slam_toolbox` | SteveMacenski/slam_toolbox（humble 分支） | **LGPL-2.1** | 官方源码 vendored（去 `.git`），colcon 编译覆盖 apt | 新增 `config/mapper_params_*_sim.yaml`（本工程 sim 参数） |
| `src/rm_localization/cartographer_ros` | ros2-gbp/cartographer_ros-release（`debian/humble/cartographer_ros`，源自 The Cartographer Authors） | **Apache-2.0**（LICENSE 由我们从同项目上游补回，见 §四-2） | 官方 ament 源码 vendored，colcon 编译覆盖 apt | 新增 `configuration_files/cartographer.lua`、`cartographer_localization.lua` |
| `src/rm_navigation/teb_local_planner`(+`teb_msgs`) | rst-tu-dortmund/teb_local_planner | **BSD** | git 子模块（未改动） | — |
| `src/rm_navigation/costmap_converter`(+`_msgs`) | LihanChen2004/costmap_converter（源自 rst-tu-dortmund） | **BSD**（package.xml 声明；⚠️ 上游无 LICENSE 文件，见 §四-3） | git 子模块（未改动） | — |
| `src/rm_driver/livox_ros_driver2` | gitee SMBU-POLARBEAR/livox_ros_driver2_humble（© ziknagXie） | **MIT** | git 子模块（未改动） | — |
| `src/rm_simulation/livox_laser_simulation_RO2`（包名 `ros2_livox_simulation`） | livox_laser_simulation / ROS2 移植版 | 见包内 `LICENSE` | 本仓直接管理 | 仿真适配 |
| `src/rm_perception/linefit_ground_segementation_ros2` | Lorenz Wellhausen（linefit_ground_segmentation） | **BSD-3-Clause** | 本仓直接管理 | ROS2 适配 |
| `src/rm_perception/pointcloud_to_laserscan` | Paul Bovbel（ros-perception） | 见包内 `LICENSE`（BSD） | 本仓直接管理 | 参数外置 |
| `src/rm_perception/imu_complementary_filter` | ccny-ros-pkg/imu_tools（© DFKI 2021 / CUNY 2015 / Willow Garage 2012 等） | **BSD-3-Clause**（LICENSE 已补，见 §四-4） | 本仓直接管理 | 参数外置 |
| `src/rm_simulation/hzmi_rm_simulation` | 本项目（场地/机器人模型改编自 PB 与 RM 公开场地资源） | 随本仓（见 §四-5） | 本仓直接管理 | 更名、2026 场地、参数外置 |
| `src/rm_navigation/fake_vel_transform`、`src/rm_localization/icp_registration`、`src/rm_navigation/rm_navigation`、`src/rm_nav_bringup` | 本项目（思路参考 CSU-RM-Sentry 等） | 随本仓（package.xml 待填，见 §四-5） | 本仓直接管理 | 编排、参数回归、接口适配 |
| `third_party/fast_lio` | hku-mars/FAST_LIO | GPL-2.0 | 原版参考（不编译） | — |
| `third_party/point_lio` | hku-mars/Point-LIO | BSD-3-Clause | 原版参考（不编译） | — |
| `third_party/cartographer` | cartographer-project/cartographer | Apache-2.0 | 原版参考（不编译） | — |
| `third_party/nav2` | ros-navigation/navigation2（humble） | Apache-2.0 | 原版参考（不编译） | — |

## 三、许可证义务要点（分发时注意）

| 许可证 | 义务 |
|---|---|
| MIT / BSD-3-Clause | 保留版权声明与许可证文本即可（本仓已保留） |
| Apache-2.0 | 保留 LICENSE/NOTICE，并**声明我们做过修改**（本文件即该声明） |
| LGPL-2.1（slam_toolbox） | 正常调用/链接无碍；对外分发二进制需保证可替换/提供对应源码 |
| **GPL-2.0（FAST-LIO）** | **若对外分发二进制（仿真镜像、真车交付包等），必须同时提供完整对应源码**，且不得修改许可证与署名。源码在本仓库中已具备 |

## 四、已知不规范项（登记待处理）

1. **FAST_LIO 许可证自相矛盾**：`LICENSE` 为 GPL-2.0 全文，而 `package.xml` 写 `<license>BSD</license>`（上游问题）。**本仓按更严格的 GPL-2.0 对待**，未擅自修改上游声明；
2. **cartographer_ros 缺 LICENSE**：上游 bloom 打包分支不含 LICENSE，我们已补入 Apache-2.0 全文（文本取自同项目 `cartographer-project/cartographer`）与 `AUTHORS`；
3. **costmap_converter 缺 LICENSE 文件**：上游（含所用 fork）仅有 `package.xml` 的 `<license>BSD</license>`、无许可证文本。因其为 git 子模块（不便于向内添加文件），此处登记而不修改；建议向上游提 issue，或后续用自有 fork 补文本；
4. **imu_complementary_filter 缺 LICENSE**：已从上游 `ccny-ros-pkg/imu_tools` 取回 `LICENSE.bsd`（BSD-3-Clause）补入；
5. **本仓自身许可证已明确（2026-09）**：顶层 `LICENSE` = **MIT ©2026 HzMi（赫兹矩阵）**；`hzmi_rm_simulation`、`icp_registration`、`rm_navigation` 的 `package.xml` 已由 `TODO` 更新为 `MIT`（`fake_vel_transform`、`rm_nav_bringup` 原本即为 MIT）。各 `package.xml` 的 `<maintainer>`/`<author>` 仍保留上游原作者署名（Lihan Chen / zcf / fyt），这是对各包来源的尊重；如团队希望补充本方维护者，可另行追加；
6. **FAST_LIO 自有 fork 为快照根**：仅 2 个提交，未保留上游提交历史（提交信息已注明基于 `5d9dc72`）。如需完整血统，可在 GitHub 对上游点一次 **Fork** 后叠加我等适配提交，并更新 `.gitmodules` 与 pin。

## 五、维护约定

- 新增第三方组件（fork / vendored / submodule / third_party 参考）时，**必须在本表登记**：上游 URL、原作者、许可证、形态、我们的改动；
- 不删除、不替换上游 `LICENSE` / 版权声明；不在上游包的 `package.xml`/源文件里改许可证；
- 对外分发（镜像、交付包、比赛提交）前，按 §三 核对义务，GPL 组件务必附带完整源码。
