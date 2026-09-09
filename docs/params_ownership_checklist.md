# 算法参数归属清单（回归各算法包 —— 只列清单，未搬家）

> 目标（R1，见 `docs/rm_bench_refactor_plan.md`）：**算法自己的参数回归各自的算法包**（回到包最原始的样子），总装层不再集中存放算法配置。
> 本清单登记"每个配置文件现在在哪、属于谁、该回归到哪里、回归时要同步改什么"。**当前只是清单，代码未动。**

---

## 0. 三条约定（先立规矩）

| 约定 | 内容 |
|---|---|
| ① 有源码的包 | 参数文件放进**该包自己的 config/ 目录**，由包 CMakeLists 安装到 install，launch 通过 `FindPackageShare('该包')` 引用 |
| ② 原"apt 无源码 → variants/" 方案 **已取消（2026-09）** | 改为：**apt 包源码化进 src**（vendored/submodule），源码编译覆盖 apt，参数照常归包内 config/；实在不源码化的（如 nav2 全家 30+ 包）本体放 `third_party/nav2` 参考、参数归自研装配包（rm_navigation/params） |
| ③ 平台参数（雷达安装外参等） | **不属于算法**：留在机器人描述/装配处（当前 `measurement_params_sim.yaml`），不搬 |
| ④ config/reality 全家 | 按 R5 冻结：不回归、不维护、不再双份同步（真车配置将来在部署导出阶段另行生成） |

---

## 1. 待回归清单（rm_nav_bringup/config/simulation → 目标归属）

| # | 配置文件（现在位置） | 参数属于谁 | 谁在引用（launch 位置） | 回归目标 | 回归需同步改 |
|---|---|---|---|---|---|
| 1 | ~~`config/simulation/fastlio_mid360_sim.yaml`~~ → `FAST_LIO/config/fastlio_mid360_sim.yaml`（**✅ 已回归**） | **fast_lio**（自有 fork 子模块） | bringup_sim 改引 `get_package_share_directory('fast_lio')/config/...` | ① 已在包 config/（CMake 本就安装 config） | 完成（fork 已推送） |
| 2 | ~~`config/simulation/pointlio_mid360_sim.yaml`~~ → `point_lio/config/pointlio_mid360_sim.yaml`（**✅ 已回归**） | **point_lio**（自有 fork 子模块） | bringup_sim 改引 `get_package_share_directory('point_lio')/config/...` | ① 已在包 config/（CMake 本就安装 config） | 完成（fork 已推送） |
| 3 | ~~`config/simulation/icp_registration_sim.yaml`~~ → `icp_registration/config/icp_registration_sim.yaml`（**✅ 已回归**） | **icp_registration**（自研/借鉴包） | bringup_sim：改引 `get_package_share_directory('icp_registration')/config/...`；bringup_real：误引用修正为 reality yaml | ① 已在包 config/（包 CMake 本就 INSTALL_TO_SHARE config） | 完成（launch 引用同步改好） |
| 4 | ~~`config/simulation/segmentation_sim.yaml`~~ → `linefit_ground_segmentation_ros/config/segmentation_sim.yaml`（**✅ 已回归**） | **linefit_ground_segmentation_ros** | bringup_sim 改引 `get_package_share_directory('linefit_ground_segmentation_ros')/config/...` | ① 已在包 config/（CMake 加 INSTALL_TO_SHARE config） | 完成 |
| 5 | ~~`config/simulation/mapper_params_online_async_sim.yaml`~~ → `slam_toolbox/config/mapper_params_online_async_sim.yaml`（**✅ 已回归**） | **slam_toolbox**（已源码化 vendored 进 src，编译覆盖 apt） | bringup_sim 改引 `get_package_share_directory('slam_toolbox')/config/...` | ① 已在包 config/（CMake 本装 config） | 完成 |
| 6 | ~~`config/simulation/mapper_params_localization_sim.yaml`~~ → `slam_toolbox/config/mapper_params_localization_sim.yaml`（**✅ 已回归**） | **slam_toolbox**（同上源码化） | 同上 | ① 同上 | 完成 |
| 7 | ~~`config/simulation/nav2_params_sim.yaml`~~ → `rm_navigation/params/nav2_params_sim.yaml`（**✅ 已回归**） | **nav2 组装参数**（nav2 本体不源码化，仅 third_party 参考；参数归自研 rm_navigation） | bringup_sim 改引 `get_package_share_directory('rm_navigation')/params/...` | ① rm_navigation/params/（CMake 本装 params） | 完成 |
| 8 | ~~`config/lua/*.lua`~~ → **`cartographer_ros/configuration_files/`**（官方示例同目录） | **cartographer_ros**（**官方 ament 源码 vendored，colcon 编译覆盖 apt**） | `cartographer_sim.launch.py` 默认引 `FindPackageShare('cartographer_ros')/configuration_files` | ① 完成：ros2-gbp humble(ament) 源码 vendored，lua 随包安装，全量 19 包编译通过 | **✅ 完成（2026-09）** |
| 9 | `config/simulation/measurement_params_sim.yaml` | **平台外参**（base_link↔livox） | bringup 拼 robot_description | ③ 留在装配层/机器人描述，不搬 | — |
| — | `config/reality/*.yaml`（9 个） | 真车 | bringup_real | ④ 冻结 | — |

---

## 2. 还"藏"在 launch 节点里的参数（也该回归，但容易被漏）

| # | 参数（原写在 bringup_sim.launch.py 节点内） | 属于谁 | 回归目标 / 状态 |
|---|---|---|---|
| a | imu 互补滤波参数（gain_acc/gain_mag/do_bias_estimation…） | imu_complementary_filter | ✅ `imu_filter_params.yaml` 已入该包 config/（CMake 加装 config） |
| b | pointcloud_to_laserscan 参数（height band/angle/range…） | pointcloud_to_laserscan | ✅ `laserscan_params.yaml` 已入该包 config/（CMake 加装 config） |
| c | fake_vel_transform 参数（spin_speed） | fake_vel_transform | ✅ `fake_vel_params.yaml` 已入该包 config/（CMake 加装 config） |
| d | point_lio 运行时覆盖参数（use_imu_as_input、filter_size 等） | point_lio | ✅ 已并入 `pointlio_mid360_sim.yaml`（R1，#2 同一文件） |

---

## 3. 回归后的引用形态（示意，非代码）

```python
# launch 里不再写 rm_nav_bringup 下的长路径，而是：
fastlio_params = PathJoinSubstitution([FindPackageShare('fast_lio'), 'config', 'fastlio_mid360_sim.yaml'])
# 无源码的（apt 包）：
nav2_params = os.path.join(get_package_share_directory('rm_nav_bringup') 或 variant 安装路径, ...)
#   —— 具体机制（variants/ 装到哪个包）待 M1 设计时定
```

---

## 4. 状态跟踪

- [x] #3 icp_registration 参数回归（**2026-09 试水完成**）：yaml 移回包 config/，bringup_sim 改引 `FindPackageShare('icp_registration')/config/`；包原生 `config/icp.yaml` 保留未动（作为原始样本）；bringup_real 原先误指向 sim yaml 一并修正为 reality
- [x] #1 fastlio 参数回归（**2026-09 完成**）：子模块已归自有 fork（William-kk985/FAST_LIO），yaml 在包 config/，bringup_sim 改引 `get_package_share_directory('fast_lio')/config/`，编译验证通过
- [x] #2 pointlio 参数回归（**2026-09 完成**）：子模块已归自有 fork（William-kk985/Point-LIO），yaml 在包 config/，bringup_sim 改引 `get_package_share_directory('point_lio')/config/`，编译验证通过
- [x] #4 segmentation 参数回归（**2026-09 完成**）：yaml 入 `linefit_ground_segmentation_ros/config/`，CMake 加装 config，bringup 改引包 share
- [x] a/b/c launch 内嵌参数回归（**2026-09 完成**）：imu/laserscan/fake_vel 各自 config yaml + CMake 安装 + launch 引用
- [x] d point_lio 覆盖参数并入 pointlio_mid360_sim.yaml（**2026-09 完成**）
- [x] reality 分支冻结标记（**2026-09**）：`config/reality/FROZEN.md`
- [x] #5/#6 slam_toolbox 参数回归（**2026-09 完成**）：slam_toolbox 已**源码化 vendored 进 src**（编译覆盖 apt），mapper 参数入其 config/
- [x] #7 nav2 参数拆分（**2026-09 完成**）：nav2 本体**不源码化**（30+ 包过大），完整源码放 `third_party/nav2` 参考；参数归自研 `rm_navigation/params/`
- [x] #8 cartographer lua（**2026-09 完成**）：换用 **ros2-gbp humble 官方 ament 源码**（非 catkin 版），vendored 进 src 并 colcon 编译覆盖 apt；lua 随 `cartographer_ros/configuration_files/` 安装，wrapper launch 默认即可用
- [ ] 回归后跑通 bringup_sim 验证（mapping/nav × fastlio/pointlio × 各 localization）
