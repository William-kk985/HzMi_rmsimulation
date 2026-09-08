# 算法参数归属清单（回归各算法包 —— 只列清单，未搬家）

> 目标（R1，见 `docs/rm_bench_refactor_plan.md`）：**算法自己的参数回归各自的算法包**（回到包最原始的样子），总装层不再集中存放算法配置。
> 本清单登记"每个配置文件现在在哪、属于谁、该回归到哪里、回归时要同步改什么"。**当前只是清单，代码未动。**

---

## 0. 三条约定（先立规矩）

| 约定 | 内容 |
|---|---|
| ① 有源码的包 | 参数文件放进**该包自己的 config/ 目录**，由包 CMakeLists 安装到 install，launch 通过 `FindPackageShare('该包')` 引用 |
| ② apt 安装、无源码的包（nav2 / slam_toolbox / cartographer_ros） | 参数**无法放回包源码**（包里没有我们的代码），统一放 `variants/<算法名>/` 目录（装配层之下、独立于总装包），launch 引用之 |
| ③ 平台参数（雷达安装外参等） | **不属于算法**：留在机器人描述/装配处（当前 `measurement_params_sim.yaml`），不搬 |
| ④ config/reality 全家 | 按 R5 冻结：不回归、不维护、不再双份同步（真车配置将来在部署导出阶段另行生成） |

---

## 1. 待回归清单（rm_nav_bringup/config/simulation → 目标归属）

| # | 配置文件（现在位置） | 参数属于谁 | 谁在引用（launch 位置） | 回归目标 | 回归需同步改 |
|---|---|---|---|---|---|
| 1 | `config/simulation/fastlio_mid360_sim.yaml` | **fast_lio**（FAST_LIO 子模块） | `bringup_sim.launch.py` `fastlio_mid360_params` | ① fast_lio 包 config/ | FAST_LIO CMake install config；launch 改用 `FindPackageShare('fast_lio')` |
| 2 | `config/simulation/pointlio_mid360_sim.yaml` | **point_lio** | 同上 `pointlio_mid360_params` | ① point_lio 包 config/ | point_lio CMake；launch 路径 |
| 3 | `config/simulation/icp_registration_sim.yaml` | **icp_registration**（自研/借鉴包） | 同上 `icp_registration_params_dir` | ① icp_registration 包 config/ | 该包 CMake；launch 路径 |
| 4 | `config/simulation/segmentation_sim.yaml` | **linefit_ground_segmentation_ros** | 同上 `segmentation_params` | ① linefit_ground_segmentation_ros 包内 | 该包 CMake；launch 路径 |
| 5 | `config/simulation/mapper_params_online_async_sim.yaml` | **slam_toolbox**（apt，无源码） | 同上（mapping 模式） | ② `variants/slam_toolbox/` | launch 路径 |
| 6 | `config/simulation/mapper_params_localization_sim.yaml` | **slam_toolbox**（apt，无源码） | 同上（nav+localization=slam_toolbox） | ② `variants/slam_toolbox/` | launch 路径 |
| 7 | `config/simulation/nav2_params_sim.yaml` | **Nav2 装配参数**（半 apt + 大量自调） | bringup → `bringup_rm_navigation.py` params_file | ② `variants/nav2/<组合>.yaml`（teb/dwb/rpp 分开） | launch 路径；注意 rm_navigation/params/nav2_params.yaml 是旧默认，以 variants 为准 |
| 8 | `config/lua/cartographer.lua` + `cartographer_localization.lua` | **cartographer_ros**（apt，无源码） | `cartographer_sim.launch.py`（默认引用） | ② `variants/cartographer/`（连 launch 片段一起） | launch 路径 |
| 9 | `config/simulation/measurement_params_sim.yaml` | **平台外参**（base_link↔livox） | bringup 拼 robot_description | ③ 留在装配层/机器人描述，不搬 | — |
| — | `config/reality/*.yaml`（9 个） | 真车 | bringup_real | ④ 冻结 | — |

---

## 2. 还"藏"在 launch 节点里的参数（也该回归，但容易被漏）

| # | 参数（写在 bringup_sim.launch.py 节点内） | 属于谁 | 回归目标 |
|---|---|---|---|
| a | imu 互补滤波参数（gain_acc/gain_mag/do_bias_estimation…） | imu_complementary_filter | ① 该包 config/ |
| b | pointcloud_to_laserscan 参数（height band/angle/range…） | pointcloud_to_laserscan | ① 该包 config/ |
| c | fake_vel_transform 参数（spin_speed） | fake_vel_transform | ① 该包 config/ |
| d | point_lio 的运行时覆盖参数（use_imu_as_input、filter_size 等一堆） | point_lio | ① 并入 pointlio 的 yaml（见上表 #2） |

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

- [ ] #1 fastlio 参数回归 fast_lio 包
- [ ] #2 pointlio 参数回归 point_lio 包
- [ ] #3 icp_registration 参数回归
- [ ] #4 segmentation 参数回归
- [ ] #5/#6 slam_toolbox 参数 → variants/
- [ ] #7 nav2 参数拆分 → variants/nav2/<组合>.yaml
- [ ] #8 cartographer lua + launch 片段 → variants/cartographer/
- [ ] a/b/c/d launch 内嵌参数回归各自包
- [ ] reality 分支冻结标记
- [ ] 回归后跑通 bringup_sim 验证（mapping/nav × fastlio/pointlio × 各 localization）
