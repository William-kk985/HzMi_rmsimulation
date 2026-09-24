# 算法正交轴与选项总表（一次只动一个轴）

> **本文的定位**：从「**轴**」的视角索引整个 bench —— 回答"这条路线属于哪个轴、怎么选中它、代码在哪、
> 现在能不能玩、和谁冲突、体验起来看什么"。
>
> 分工（避免重复）：
> - 算法**选型理由/优劣对比** → `rm_algorithm_catalog.md`（2D×3D、经典×先进；含相机算法 §五）
> - **角色与场景**（谁一体、场景 A 建图后导航 / B 边建图边导航） → `rm_algorithm_overview.md`
> - **能力矩阵与组合数**（120 组合怎么来）、**实测状态表 §四** → `algorithm_matrix.md`
> - **维度归类 2D/2.5D/3D** → `algorithm_matrix.md §一.1`；**体验优先路线图** → §8.6
> - **两天失败链与契约教训** → `debug_fastlio_cartographer.md §9`

---

## 0. 为什么要分「轴」

1. **120 组合不是 120 个独立项目**，而是 6~8 条轴的选项相乘。把轴分清，新路线 = 换一个轴的一个选项，
   而不是重搭一套栈。
2. **一次只动一个轴** ⇒ 失败时可归因。这是这两天最贵的教训：6 个病根（cartographer 参数漂移、
   `<always_on>` 缺失、CustomMsg QoS 背压、`use_sim_time` 键名、上游丢弃 `transformPose`、
   `fake_vel_transform` 替换角速度）**全部是契约问题，没有一个能靠换算法解决**。
3. **轴内可替换 = 同插槽不同实现**；**跨轴 = 不同职责**。混着改就会同时动两个变量。

## 1. 轴的拓扑（谁依赖谁）

```
                 ┌── F 运动学（小陀螺/云台解耦，spin_speed）─────────────────┐
                 │                                                          │
雷达/IMU ─► E 感知 ─► A0 前端里程计(lio) ─► A1 在线建图(mapper) ─┐          │
   │         (分割/投影/相机)      │                              ├─► /map │
   │                              └─► A2 重定位(localization) ────┘   │     │
   │                                        │                          ▼     │
   └────────────────────────────────────────┴─► C 障碍表示(local/global costmap)
                                                          │  ▲
                                                     D 维度（2D→2.5D→3D）
                                                          ▼
                                                    B 规划控制(nav) ──► /cmd_vel ──► 底盘
```

> 关键点：**A2 与 A1 大多互斥**（都在发 `map→odom`）；**D 是"表示层"**，贯穿 costmap 与规划；
> **F 在控制之后**，是这两天最后被抓到的 bug。

## 2. 六条轴（含子轴）

### 轴 A｜建图 / 定位（3 个子轴）

**A0 前端里程计（谁发 `odom→base_link`）—— 入口 `lio:=`**

| 选项 | 状态 | 代码位置 | 体验看点 / 判据 |
|---|---|---|---|
| `fastlio` | ✅ 默认，已跑通 | `src/rm_localization/FAST_LIO`（子模块） | 基线；看 `/odom` 频率、漂移、对 IMU 初始化的依赖 |
| `pointlio` | ✅ 可用，待体验 | `src/rm_localization/point_lio` | 与 FAST-LIO 对比：建图质量、退化场景（长走廊）鲁棒性 |
| `none` | ✅ 语义槽位 | —（需外部提供 odom/TF，如轮速） | 体验"纯轮式里程计"的下限 |
| `cartographer` | ✅ 全包形态 | `src/rm_localization/cartographer{,_ros}` | **兼任里程计源**：跳过 `mapper` 与 `localization`；`mode:=nav` 时用 `map/<world>.pbstream` 纯定位。**不发 `/odom` 话题 ⇒ `nav:=teb` 不适用** |

**A1 在线建图（谁造 `/map`）—— 入口 `mapper:=`（仅 `mode:=mapping`）**

| 选项 | 状态 | 体验看点 |
|---|---|---|
| `cartographer` | ✅ 特征"留不住"问题已解决（`min_probability_to_clear: 0.80` + 参数回调） | 2D 图质量、闭环、圈数对留存的影响 |
| `slam_toolbox` | ✅ 可用 | 与 cartographer 对比：调参量、在线性、CPU |

**A2 重定位（谁发 `map→odom`）—— 入口 `localization:=`（仅 `mode:=nav`）**

| 选项 | 状态 | 依赖资产 | 体验看点 |
|---|---|---|---|
| `amcl` | ✅ 已跑通 | `.pgm/.yaml` | 粒子收敛速度、初始位姿敏感度、`transform_tolerance` 的未来戳（我们踩过） |
| `icp` | ✅ 包在，待体验 | `PCD/<world>.pcd` | LIO 漂移下的鲁棒性（3D 点云配准，不依赖 2D 图） |
| `slam_toolbox` | ✅ 可用 | `.posegraph` | 图优化式重定位 |
| `cartographer` | ✅ 可用 | `.pbstream` | 纯定位（frozen state），与在线 SLAM 同源 |

> **禁区**：`mapper` 与 `localization` 不得同开；`lio:=cartographer` 会自动跳过二者；
> AMCL / ICP / slam_toolbox / cartographer **都会自发 `map→odom`** ⇒ 不能再叠加静态桥（多父边）。

### 轴 B｜规划与控制 —— 入口 `nav:=`

| 选项 | 状态 | 代码位置 | 体验看点 |
|---|---|---|---|
| `rpp` | ✅ 默认，已跑通 | nav2 `nav2_regulated_pure_pursuit_controller` | 平滑、参数少；窄缝/动态障碍表现 |
| `dwb` | ✅ 实现（nav2 自带）+ `nav2_params_sim_dwb.yaml` | nav2 | 局部避障更稳；参数多；与 RPP 对比震荡与通过率 |
| `teb` | ✅ 包在（`teb_local_planner` + `costmap_converter`） | `src/rm_navigation/teb_local_planner` | 时间最优轨迹、支持倒车/全向；**调参成本最高**；`lio:=cartographer` 下不可用 |

> 相关可单独玩的组件：`velocity_smoother`（加速度/加加速度约束）、`behavior_server`（backup/spin/wait）、
> `costmap_converter`（障碍多边形化，TEB 的依赖，可单独观察其输出）。

### 轴 C｜障碍表示（最接近"维度体验"）—— 入口 `local_obstacle:=` / `global_obstacle:=`

| 位置 | 选项 | 状态 | 语义 / 体验看点 |
|---|---|---|---|
| local | `scan` | ✅ 默认 | 2D 单线（p2l 输出）；**有约 45 cm 盲区**；链路单点 |
| local | `cloud` | ✅ | 3D 点云直投（`/segmentation/obstacle`，不经 p2l）⇒ **无盲区**，但算力高 |
| local | `both` | ✅ | 双源冗余（任一路挂掉仍能避障） |
| global | `stvl` | ✅（**遗留**：仍 100% 丢 `/segmentation/obstacle`，时间区间问题） | **3D 体素层 + 时间维**（`spatio_temporal_voxel_layer`，系统包）；`voxel_decay`/`mark_threshold`/高度带 |
| global | `scan` | ✅ | 与 local 同源，把"什么算障碍"收敛到 p2l 一处 |
| global | `none` | ✅ | 只用 static+inflation（体验"全局看不到实时障碍"的后果） |

**统一判据**：贴墙 0.3 m 是否可见 / 矮台（0.1 m）判为障碍还是可越 / 幽灵障碍 / 保守度（绕行幅度）/ CPU。

### 轴 D｜维度（2D → 2.5D → 3D）

| 层级 | 实现 | 状态 | 入口 / 代码 |
|---|---|---|---|
| 2D 栅格 | nav2 costmap（`static/obstacle/inflation`） | ✅ | 参数 |
| **2.5D 高度带** | `min/max_obstacle_height` 过滤 | ✅ 已用 | 参数（区分"横梁/高台"与"地面可越障"） |
| **2.5D+ 坡度/台阶层** | 局部坡度/台阶高度 → costmap 一层或减速触发 | ❌ 待写 | `src/rm_navigation/rm_costmap_layers/`（接口现成） |
| **3D 体素层** | `stvl_layer` | ✅ **已具备** | 见轴 C ⇒ **体验 3D 的最低成本入口** |
| **3D→2D 切层投影（离线）** | PCD 切层/投影成 2D 图 | ✅ 工具在 | `tools/scripts/mapping/pcd_to_grid_map.py`、`docs/3d_to_2d_survey.md` |
| 高程/净空图 | `rm_elevation_map` | ❌ 未来槽位 | 新包 |
| **3D 规划器** | 需自写 nav2 插件（接口是 2D） | ❌ 成本最高 | 排最后 |

> **洞察**：想体验 3D/2.5D，**大部分不用写代码** —— `stvl` 本就是 3D 体素层，切层工具已有，
> 高度带是现成参数。真正要写代码的只有"坡度层"和"3D 规划器"。

### 轴 E｜感知（雷达 → +相机）

| 现状 | 说明 |
|---|---|
| 雷达侧感知 ✅ | `linefit_ground_segmentation`（地面/障碍分离）、`pointcloud_to_laserscan`（3D→2D 扫描）、`imu_complementary_filter`（IMU 姿态） |
| **相机/云台 ❌** | **URDF 里既无相机也无云台**（实测 grep 无 `camera/gimbal/depth`）⇒ 从零起步 |
| 相机算法候选 | 见 `rm_algorithm_catalog.md §五`（RGB-D / 单目 / 双目；检测、深度、语义） |
| **契约先行**（必须） | `camera_info` + 内外参标定 + 时间戳/频率 + 坐标变换（`optical_frame`）——否则就是新的"假数据"来源 |
| 与导航的关系 | **先独立成线**（瞄准/目标检测）；**最后**才考虑把"敌人/语义障碍"回喂 costmap |

### 轴 F｜运动学（含这两天刚修的坑）

| 选项 | 状态 | 语义 / 注意 |
|---|---|---|
| `spin_speed:=0`（关小陀螺） | ✅ **已修**：`fake_vel_transform` 直通模式（`base_link_fake ≡ base_link`），指令含角速度原样下发 | 修前会把角速度替换成 `spin_speed` ⇒ **恒 0 ⇒ 车永不转弯**（`Failed to make progress`） |
| `spin_speed!=0`（小陀螺） | ✅ 原语义保留 | 底盘恒定自转 + 云台系反向补偿；nav 的转向量**被丢弃**（设计如此） |
| 云台 2DOF（瞄准） | ❌ 未做 | 与轴 E 相机线绑定 |
| 底盘运动学 | ✅ 麦轮（`mecanum_controller`），可全向 | 体验"全向 vs 差速"对规划的影响 |

## 3. 组合禁区（都是这两天实测出来的，别重踩）

| 禁止 | 原因 |
|---|---|
| `mapper` 与 `localization` 同开 | 两者都发 `map→odom` ⇒ TF 多父边 |
| 重定位模块 + 静态 `map→camera_init` 桥 | 同上（launch 已按 `localization` 是否为空自动处理） |
| `lio:=cartographer` + `nav:=teb` | cartographer 全包形态**不发 `/odom` 话题**，TEB 需要 |
| `lio:=amcl` ❌ 非法值 | `lio` 只接受 `fastlio|pointlio|none|cartographer`；`amcl` 属于 `localization`（已加 `choices` 校验，传错直接报错退出） |
| `spin_speed:=0` + 期望"原地转弯" | **修好后可用**；修之前恒不转（已修，见轴 F） |

## 4. 每轴"最小体验清单"（先玩这 8 个，覆盖全部轴）

| # | 命令/改动 | 轴 | 看什么 |
|---|---|---|---|
| 1 | `mode:=mapping lio:=fastlio mapper:=cartographer` | A1 | 2D 图质量与留存基线 |
| 2 | `mode:=mapping lio:=fastlio mapper:=slam_toolbox` | A1 | 与 #1 对比（调参量/在线性/CPU） |
| 3 | `mode:=nav lio:=pointlio localization:=amcl` | A0 | pointlio 的漂移与退化鲁棒性 |
| 4 | `mode:=nav lio:=fastlio localization:=icp` | A2 | 3D 点云重定位 vs AMCL |
| 5 | 轴 C：`local_obstacle:=scan|cloud|both` × `global_obstacle:=stvl|scan|none` | C | **2D 单线 / 3D 点云 / 3D 体素** 的盲区·幽灵障碍·保守度 |
| 6 | `nav:=rpp|dwb|teb`（同一目标点） | B | 控制器风格差异 |
| 7 | 调 `stvl` 的 `voxel_decay`/高度带 + 跑一次 `pcd_to_grid_map.py` | D | 带时间维的 3D 表示、3D→2D 投影 |
| 8 | `spin_speed:=0` vs `-6.0` | F | 小陀螺/云台解耦的体感与代价 |

## 5. 待写清单（要动代码的，按成本排序）

| 优先级 | 事项 | 落点 |
|---|---|---|
| ★ | **P0 回归**（看门狗 + 无头目标点 + 四跳命令链断言） | `tools/scripts/`（把每次体验压到 1 分钟） |
| ★★ | 2.5D 坡度/台阶层 | `src/rm_navigation/rm_costmap_layers/` |
| ★★ | 相机数据流 + 云台 2DOF（契约先行） | `src/rm_perception/rm_camera_detect/` |
| ★★★ | 高程/净空图 | `src/rm_perception/rm_elevation_map/` |
| ★★★ | 3D 规划器插件 | 新包（nav2 插件接口 2D，最贵） |

## 6. P0 回归怎么用（一条命令出结论）

栈起来后（本脚本会等它就绪）：

```bash
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0   # 链路体检 + 发目标 + 结果断言
python3 tools/scripts/regress/nav_smoke_regression.py --skip-goal      # 只体检链路（换感知/建图轴时用）
```

判据（集中在脚本顶部 `TH` 字典，可按 world/起始点调）：链路四环节频率、TF 新鲜度、
`footprint` 戳是否持续更新、**四跳命令链是否一致**、真值是否真动、
以及**拒绝"假到达"**（<2 s 就 SUCCEEDED 而残余 >0.5 m ⇒ FAIL）。
输出 `PASS/FAIL` + **首个断点** + `.tmp_bags/regress_<ts>.json` 快照 ⇒ 抄一行进 `algorithm_matrix.md §四`。

## 7. 记录方式

- **每玩一条路线** → 在 `algorithm_matrix.md §四 实测状态` 加一行（组合 / 日期 / 结论 / 判据数据）；
- **轴有增减或语义变化** → 改本文；
- **踩到坑** → `debug_fastlio_cartographer.md §9.0 总表` 加一行（症状 → 判据 → 处置 → 状态）。
