# 帧与契约规范 v3（规范性）：三层参数 · 最终 TF 树 · 重力契约 · 重定位契约

> **本文性质**：**规范性文档（normative）**。它写"应该是什么"，不写"现在代码是什么"，也不记录
> 为什么这么定（理由、证据链、被否掉的方案、我们对自己旧结论的更正，全在
> **`docs/frame_contract_v3_rationale.md`**，下称 **R**）。
>
> **状态**：**设计定稿，未实施**。本文**没有**改任何代码/配置/launch，**没有**跑仿真。
> 文中所有"默认值 = 现状"的表述指 **HEAD = `c996365`** 的行为。
>
> **与 `docs/tf_interface_contract.md` 的关系**：那份是 T1–T6 的**实施记录**（T6 已撤销）；本文是
> **目标契约**，吸收它并把范围扩到"参数分层 + 重力 + 重定位 + 校验"。两者冲突时以本文为准，
> 但**在 v3 落地之前，运行期仍以 `tf_interface_contract.md` + launch 的现状为准**。
>
> **发布方式**：分两步。**v3a = 别名期**（§7 兼容层，现有命令零改动）；**v3b = 删除期**
> （§10 删除清单生效）。本文不写日期，只写"进入 v3b 的前置条件"。

---

## 证据约定（全文强制）

每个数字/判决后面必须能追到下面某个编号的 `§` 或 commit，并标 **【实测】/【引用】/【推断】/【设计】**：

| 编号 | 来源 | 本文用到它的哪一节 |
|---|---|---|
| **S1** | `docs/odom_base_link_practice_survey.md`（commit `b4e02c7`；§9 + `0ee92b6`/`c996365`） | §9 受控实验（矩阵 / 200 帧门 / 残差 / 压扁 13.17% / `/scan` / 成本 / 未验证） |
| **S2** | `docs/gravity_alignment_ownership_survey.md`（`3017355`） | 四层归属、REP-105、孤儿、`yaw` 不归重力 |
| **S3** | `docs/relocalization_practice_survey.md`（`e065107`） | 术语、能力表、M1–M4、SICK/Autoware/Nav2、健康/丢失、`TransformAvailable` 只查存在 |
| **S4** | `docs/cod_vs_ours_frame_architecture.md`（`057294c`） | H1/H2/H3、O1–O6、I1–I6、`T_bl` 语义、选项 C/B |
| **S5** | `docs/lio_extrinsic_application_survey.md`（`68d936f`） | bug②/bug③ 判决、FAST-LIO #284/#433、LIO-SAM 合成 |
| **S6** | `docs/tilted_lidar_fidelity.md`（§N `b2caff3`，§O `fe0dc31`） | 外参守卫、`robot11_mount` 三档、`/scan` 高度带、provenance |
| **S7** | `docs/round_summary_gz_frames.md`（`9152d90`/`057294c`） | 本轮四条线、口径冲突 |
| **S8** | `docs/fortress_sim_backend.md`（`9152d90`） | `sim_backend:=gz` 的事实与硬要求 |
| **L** | `src/rm_nav_bringup/launch/bringup_sim.launch.py`（HEAD） | 现行参数默认值、静态桥、`fake_vel_transform` |
| **C-\*** | commit：`98baa55`（vendor）· `2a5a8b6`（修 bug②③）· `952a6da`（E1 补丁）· `0ee92b6`（v6c）· `fe0dc31`（守卫）· `057294c`（COD/gz） | 见对应 `§` |

**标记**：
**【实测】** = 本仓真跑出来的数字（S1 §9 / S6 §J/§K/§O）；**【引用】** = 上游源码/文档原文（S1–S5 的引用编号）；
**【推断】** = 由源码/几何/约定推出但本轮无数据；**【设计】** = 本文规定的、尚无实测的取值。

---

## §0 五条原则

### P1 三层分离 —— 参数按"谁拥有这个事实"分三层，不按"哪个文件方便"分

| 层 | 拥有什么事实 | 例 | 谁能读它 |
|---|---|---|---|
| **描述层** | 机器人**物理几何**（link、关节、倾角、杆臂） | `robot` | 所有层（只读） |
| **仿真/显示层** | **这次怎么跑**（后端、世界、GUI、可视化、转速、资产名） | `sim_backend` `world` `gui` `nav_rviz` `robot_visual` `spin_speed` `cloud_accumulator` `map_name` | 仿真与显示节点；**算法层不得依赖** |
| **算法层** | **这次用哪个算法**（8 个槽） | `odom` `map_source` … | 算法节点与 launch 装配 |

**判据**：一个参数若改变"物理事实"，它在描述层；若改变"这次实验的装扮"，它在仿真/显示层；
若改变"谁来算"，它在算法层。**同一个事实只允许出现在一层**（见 P2）。
**违反的典型症状**：倾角同时写在 URDF 与插件里 ⇒ 双倍倾角（S4 §6.5、S1 §6.3）。

### P2 唯一事实 —— 三件事各有唯一所有者

| 事实 | **唯一所有者** | 禁止的第二个所有者 | 依据 |
|---|---|---|---|
| **安装倾角 / 杆臂** | **URDF**（`base_link→livox_frame` 的固定关节） | 插件 `<tilt_rpy>`、`lio_tf_adapter` 的 `rpy` 参数、"重力帧" | S4 §2.2 / §7.2 O3；S6 §J.1.2 |
| **重力方向 → 世界系 z 朝上** | **估计器初始化**（E1，`gravity_align_mode`） | 任何 TF 边（含 `odom_grav`）、任何静态桥的 roll/pitch | S1 §9.4(3)；R §4 |
| **地图 / 先验资产** | **`map_source`** | 任何发 TF 的节点、`localization` 枚举里的"顺带加载" | S3 §6.3 |

**判据**：对任一物理量问"如果它变了，要改几个地方？"。答案必须是 **1**。
**为什么"倾角只在 URDF"**：URDF 是唯一**同时**被 `robot_state_publisher`（发 TF）、被 LIO（`lookupTransform`）、
被感知（`target_frame`）读到的地方（S4 §2.2）；插件档把倾角藏进 `<tilt_rpy>` 后，TF 说"平"、数据说"斜"，
是**自相矛盾**的组合（S8 §2.2 原文判语）。

### P3 唯一发布者由 launch 强制 —— tf2 不做检查，所以必须有人替它做

`tf2` 的 README 全文**没有**"每条变换只能有一个发布者"的条文，运行期也**不做检查**（S3 §4.1 负证据 E30）。
"两个节点同发 `map→odom`"在 tf2 看来**语法合法**，后果是查询抖动/跳变。
⇒ 契约不能靠"大家自觉"，只能靠 **launch 的静态互斥 + `check_contract` 的静态检查**（§6/§8）。

**本仓的既有事实**：`localization` 的 8 个值靠 launch 条件互斥（L:1353-1366），`mapper` 与它按 `mode` 互斥
（L:1430-1434、L:1815-1819）——**已经做到了"选的时候唯一"**，缺的是"运行期唯一 + 坏了谁接"（S3 §6.1 R1/R6/R8）。

### P4 能力声明 —— 每个槽位声明"我能做什么"，组合由能力相容性决定，而不是由枚举值决定

现状：`localization` 一个枚举混装 8 个值，其中既有 2D 粒子滤波、纯局部配准、也有完整 SLAM 包（S3 §6.1 R1）。
⇒ 用户选的是"包名"，不是"能力"，于是"全局/局部"这层区别在配置里**不可见**。

v3 要求每个槽位**声明**三项能力：`cap_global`（能否无先验整图求解）、`cap_local`（能否逐帧跟踪）、
`cap_tf`（是否发 TF），以及**必须**声明的 `cap_health`（是否能发健康信号）。
`check_contract` 用能力做相容性判定（§6），**不再用包名硬编码规则**。

### P5 `check_contract` 可解释 —— 每条不变式有名字、有证据、有修复建议

`check_contract:=True` 的输出必须能让**没读过本文的人**修好问题（§8 样例）：
每条 finding 带 `id`（如 `E-TF-01`）、**判据**（哪条不变式）、**证据**（哪个节点/哪条参数/哪个文件:行）、
**修复建议**（改哪个参数、有哪两个合法选择）。**不允许**只输出"契约检查失败"。

---

## §1 三层参数

### 1.1 描述层：`robot`（唯一参数）

| 参数 | 取值 | 拥有的事实 | 能力声明 | 备注 |
|---|---|---|---|---|
| `robot` | `''`（默认模型）\| `hzmirm` \| `robot11` | link/关节/惯量/碰撞、**倾角与杆臂**、`base_link` 原点语义 | 提供 `base_link→livox_frame` / `base_link→imu_link` 静态边 | 默认 `''`（L:1233）；`robot11` 在 v3 语义（`sensor`）下倾角记在关节 `rpy=[-30,0,0]`、杆臂 `[0.000562,0.130916,0.157028]`（S4 §2.2 + S6 §J.1.2） |

**删除**：
* `robot11_mount`（`plugin|urdf|sensor`，L:1287-1301）——三档并存 = 倾角有三个所有者（S4 §7.2 O3）。
* `robot11_mount_axis` —— **本仓不存在此参数**（全仓 `grep` 无命中，已核）。真正存在的是
  `livox_tilt_axis`（`roll|pitch`，L:1257-1268），随 P2 一并删除；其取值之争见 S4 §7.2 O6
  （URDF 写 pitch、CSV 写 roll；`livox_tilt_axis` 默认 `roll`）。
* v3 中 `sensor` 的语义**升格为唯一语义**：倾角在 URDF 里，点云表达在**传感器系**，
  `frame_id=livox_frame` 与数据自洽（S6 §J.3、S8 §2.2）。`plugin`（平帧 + 斜数据）与
  `urdf`（斜帧 + 平数据，诊断档）在 v3b 删除；v3a 期间仅作别名接受并打 DEPRECATED（§7）。

### 1.2 仿真/显示层（8 个参数）

| 参数 | 默认（HEAD） | 取值 | 拥有的事实 | 谁读它 | 依据 |
|---|---|---|---|---|---|
| `sim_backend` | `classic` | `classic` \| `gz` | 用哪个仿真器 | 仅仿真装配 | L:1223；S8 §0/§9 |
| `world` | `RMUL2026` | 世界名（同时决定 map/PCD 前缀） | 这次跑哪个场地 | 仿真 + 资产解析 | L:1206 |
| `gui` | `True` | `True`\|`False` | 起不起 gzclient | 仅仿真 | L:1172 |
| `nav_rviz` | `True` | `True`\|`False` | 起不起 nav2 RViz | 仅显示 | L:1146 |
| `robot_visual` | `decimated` | `decimated`\|`full` | `<visual>` mesh 档 | 仅显示 | L:1315-1327 |
| `spin_speed` | 槽位相关：`robot11` ⇒ `0.0`，其余 `5.0` | rad/s | `fake_vel_transform` 的小陀螺角速度 | `fake_vel_transform` | L:1193-1203 |
| `cloud_accumulator` | `False` | `True`\|`False` | 起不起 3D 累加器（仅 `mode:=mapping`） | 建图链路 | L:1497-1517 |
| `map_name` | `''`（= `<world>`） | 存档基名 | 这次读/写哪份存档 | 建图/定位 | L:1467-1474 |

> **约束（P1）**：算法层**不得**读这一层的任何参数。反例（v3 禁止）：`lio` 分支里读 `gui` 决定行为、
> 感知参数里写死 `world` 名字。

### 1.3 算法层：8 个槽

| # | 槽 | 取值（v3a 兼容值在前） | 能力声明 | 发 TF？ | 必须发健康信号？ |
|---|---|---|---|---|---|
| 1 | **`odom`** | `fastlio`（默认）\| `pointlio` \| `small_point_lio` \| `none` \| `cartographer`（全包，降级） | 提供连续 `odom→base_link`（**经 adapter**）；`cap_gravity` = 世界系是否重力对齐（E1 决定） | **否**（由 `lio_tf_adapter` 发，§2） | **是**（`/odom` 频率 + 时间戳 + `gravity_align` 结果行） |
| 2 | **`map_source`** | `2d_grid` \| `pcd` \| `pbstream` \| `posegraph` \| `none` | 提供地图/先验资产；`cap_version`（世界/出生点一致性 sidecar） | **否（硬约束）** | **是**（加载完成 + 版本 + 失败原因） |
| 3 | **`global_relocalizer`** | `none` \| `amcl_global` \| `pcd_multistart` \| `descriptor` | `cap_global=true`：无先验 → 位姿假设 + 质量 | **否（硬约束）** | **是** |
| 4 | **`local_tracker`** | `amcl` \| `beluga` \| `slam_toolbox_localization` \| `cartographer_localization` \| `gicp` \| `small_gicp` \| `icp` \| `ndt` \| `none` | `cap_local=true`；**唯一** `map→odom` 发布者 | **是**（`map→odom`） | **是**（`valid/quality/state`） |
| 5 | **`nav`** | `rpp`（默认）\| `dwb` \| `teb` \| `mppi` | 控制器槽 | 否 | 否（消费者） |
| 6 | **`planner`** | `navfn`（默认）\| `smac2d` | 全局规划器槽 | 否 | 否（消费者） |
| 7 | **`ground`** | `linefit`（默认）\| `patchwork` | 发 `/segmentation/{ground,obstacle}`；**要求 z 朝上**（S2 §2.1） | 否 | **是**（两路点数 + `sensor_height` 生效值） |
| 8 | **`local_obstacle` + `global_obstacle`** | `scan`（默认）/`cloud`/`both`/`stvl`/`stvl_both`；全局 `stvl`（默认）/`scan`/`none` | 代价图观测源；高度带语义见 §10 | 否 | **是**（层更新频率 + 高度带丢点率） |

**槽位与旧参数的对应**见 §7。**`mode`** 不是槽：`mapping|slam_nav|nav` 决定"哪些槽必须存在"（§6 矩阵）。

---

## §2 TF 帧树（最终版，无 `odom_grav`）

```
map ──(local_tracker / M1–M4 中被选中的那一个)──► odom ──(lio_tf_adapter)──► base_link ──(URDF)──► livox_frame
                                                                                   │                └► imu_link
                                                                                   └──(fake_vel_transform)──► base_link_fake
```

**硬性要求**：
1. **`odom` 必须重力对齐**（z 朝上、yaw 任意），由**估计器初始化 E1** 保证（§4）。
   ⚠️ **合规窗口**：HEAD 的默认 `gravity_align_mode:=off` 下 `odom` **不**重力对齐（实测倾角 30.110°，S1 §9.3 #1）。
   因此 v3a 把"`odom` 重力对齐"声明为**分级合规**：`gravity_align_mode∈{urdf,imu,hybrid}` ⇒ **合规**；
   `off` ⇒ **grandfathered（继承态，允许但必须由 `check_contract` 报 WARN）**。v3b 要求默认 `hybrid`。
2. **没有 `odom_grav` / `world_grav` / 任何"重力帧"**（S1 §3.1 判决：18 个仓库/包 **0 例**；R §4.1）。
3. `camera_init` / `body` / `aft_mapped` 等 **LIO 内部帧不进入契约**：允许存在，**任何消费者不得依赖**
   （它们是 S1 §2 R8/R9 说的"只发自己的内部帧"）。

### 2.1 每条边

| 边（parent→child） | owner（唯一） | 类型 | 速率 | 内容 | 失败模式 |
|---|---|---|---|---|---|
| `map→odom` | **`local_tracker`**（M1 时是同一个一体机；M4 时是融合器） | dynamic | 10 Hz（AMCL 离散更新，有效 1–10 Hz）【引用 S3 §5.1】 | `T_map←odom`；2D 追踪器只给 `(x,y,yaw)`（S2 §3.2），3D 追踪器可 6DoF | **缺失** ⇒ nav2 `TransformAvailable` 失败、控制器中止（S3 §3.2 E6）；**冻结** ⇒ 无任何内建检测，实测 575/581 样本仍在发 `/cmd_vel_nav`、最坏盲开 **110 m**（S3 §6.1 R5） |
| `odom→base_link` | **`lio_tf_adapter`** | dynamic | 9.999–10.000 Hz【实测 S1 §9.3】 | **合成** `T_ol·T_bl`（`q=q_lio·q_off`、`p=p_lio+R_lio·t_off`），S4 §3.5 | 缺失 ⇒ 无里程计；**共轭** ⇒ 假俯仰 **4.890°**（真值 0.003°，S5 §3.2）；**双重变换** ⇒ 场景刚性转 **30.970°**（S5 §3.1）；守卫 `fe0dc31` 会在旋转残差 >`1e-6 rad` 时限速 ERROR |
| `base_link→livox_frame` | **URDF**（`robot_state_publisher`） | static | 一次（latched） | `R_x(−30°)` + `t=(0.000562,0.130916,0.157028)`（`\|t\|=0.204443 m`）【实测 S4 §2.2 / S5 §3.1】 | 与插件 `<tilt_rpy>` 同时存在 ⇒ **双倍倾角**；写错轴 ⇒ `livox_tilt_axis` 之争（S4 §7.2 O6） |
| `base_link→imu_link` | **URDF** | static | 一次（latched） | IMU 安装位姿；MID-360 内置 IMU 的 `T_IL` 是**估计器参数**、不是这条 TF（S5 §1.2） | 与 LIO `extrinsic_T/R` 混用 ⇒ "转两次"（S5 §3.1 引 #433） |
| `base_link→base_link_fake` | **`fake_vel_transform`** | dynamic | 20 Hz【实测 L:2146】 | **只含 yaw**（小陀螺角），pitch/roll 透传（S4 §7.1 I6） | 缺失 ⇒ nav2 的 `robot_base_frame` 解析不到；`spin_speed≠0` 时它凭空转而雷达不跟着转（L:1198） |

### 2.2 `T_ol·T_bl` 的方向（最容易写反，写死在这里）

* `T_ol` = LIO 输出的 body(=livox) 位姿；`T_bl = lookupTransform(livox_frame, "base_link")` = **base_link 在 livox 系里的位姿**。
* 链式法则：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ **`T_ob = T_ol·T_bl`（右乘合成）**。
* `T_bl⁻¹·T_ol·T_bl` 是**换基（共轭）**，只在 `T_bl` 的旋转为 `I` 时姿态碰巧相同（S5 §3.2；上游 `qza36/COD_NAV` 的 `small_point_lio_node.cpp:75` 就是共轭，被 COD 的 `T_bl≡I` 约定掩盖，S4 §3.3）。

---

## §3 不变式 I1–I7

> 每条都给**可执行判据**（`check_contract` 怎么查）与**违反的实测后果**。这七条是 §9 验收测试的对象。

### I1 唯一发布者

**判据**：对 TF 树里每条边，在 `(mode, robot, sim_backend, 8 个槽)` 的**笛卡尔积**上静态求值 launch 条件，
**恰好一个**节点声明该边的 `parent→child`。
**违反后果**：查询抖动/跳变；tf2 不报错（S3 §4.1）。
**现状**：`map→odom` 已由"选的时候唯一"保证（L:1353-1366、L:1430-1434）；`odom→base_link` 在
`lio:=small_point_lio` 时**唯一**（LIO 自己发，L:2028-2033 排除 adapter），其余 LIO 由 adapter 发 —— 也唯一，
但**权威不统一**（S4 §7.2 O2）。

### I2 `odom→base_link` 只由 `lio_tf_adapter` 发

**判据**：全栈 `grep` 该边的 `sendTransform` / `StaticTransformBroadcaster`；只允许 adapter 一处。
**违反后果**：两个发布者（LIO + adapter）⇒ 双父边；`small_point_lio` 槽位当前违反此条（它自己发，S4 §3.5）。
**修复路径（S4 §8 选项 B1）**：给 LIO 加 `publish_tf: true|false`（默认 `true`，保持默认路径逐字节不变），
bringup 传 `false`；**在 B1 落地前，I2 对 `small_point_lio` 记为"已知例外"**（由 `check_contract` 报 WARN，不报 ERROR）。

### I3 `map→odom` 只由 `local_tracker` / M1 发

**判据**：`global_relocalizer` 与 `map_source` 的节点**不得**出现在该边的发布者集合里；`mapper` 与
`localization` 不得同时生效（`mode:=slam_nav` 时不加载任何重定位槽，L:1815-1819）。
**违反后果**：与 I1 同；且"全局/局部"失去分工（S3 §5.4 反模式）。

### I4 `global_relocalizer` 不发 TF

**判据**：该槽的节点在 `ros2 node info` 里**没有** `tf2_ros/transform_broadcaster` 相关 publisher；
其输出只有"位姿假设 + 质量"（服务/话题）。
**依据**：Autoware `pose_initializer` 只发 `/initialpose3d` 给 EKF（S3 §6.3 表）、SICK initial positioning 只产一次位姿（S3 §3.4）。

### I5 `odom` 必须重力对齐；所有 z 参数只属于重力对齐帧；yaw 不归重力

**判据三条**：
1. `gravity_align_mode` ≠ `off`（或 `check_contract` 报 WARN，§2 合规窗口）；
2. `linefit.sensor_height` / p2l `min/max_height` / STVL `min/max_obstacle_height` / 自击掩膜 z 门限
   这四组参数**只在** `odom`（或 `base_link`）这样的**重力对齐帧**里才有物理意义；
   写在斜系里必须显式声明"这是传感器高度闸，不是障碍高度"（S6 §K.1.4）；
3. 重力只定 **2 个自由度**（roll/pitch），**yaw 不归重力**（S2 §1.2）；任何"重力对齐"实现都**不得**改 yaw。
**违反后果**：局部代价图把地面投影到斜的 `z=0` 平面 ⇒ 沿坡向**压扁 13.17%**（理论 `1−cos30.05°=13.44%`，
对齐后 **1.0000**，S1 §9.2）；`/scan` 波束在 odom 里出高度带 **39.6–40.4%**（S6 §J.5/§K.1.3）。

### I6 每个位姿源必须有健康信号，≥1 Hz

**判据**：每个 `cap_tf=true` 或 `cap_global=true` 的槽，必须发布 §5.4 的字段表；`now-stamp>1 s` ⇒ 违约。
**现状**：只有 `gicp`/`small_gicp` 有（`~/fitness_score`、`~/converged`，S3 §6.1 R3）；
`amcl`/`beluga`/`cartographer` **没有**（S3 §1.3）。
**违反后果**：下游无法反应（S3 §4.2 判决 2）。

### I7 消费者用 TF **时效**而非**存在性**

**判据**：nav2 侧不得把 `TransformAvailable` 当作健康判据；必须另有时效 watchdog（§5.5）。
**依据**：Nav2 的 `TransformAvailable` 原文 *"Once found, it will always return success."*（S3 §3.2 E6）；
本仓实测"冻结 TF + 继续开 110 m"正是这条缺失的直接后果（S3 §6.1 R5）。

---

## §4 重力契约（E1）

### 4.1 参数

```yaml
gravity_align_mode: "off"        # off | urdf | imu | hybrid   （默认 off = 现状；推荐 hybrid）
gravity_align_rpy:  [-30.0, 0.0, 0.0]   # 先验；由 launch 从 URDF 自动填入（此值是 robot11 sensor 档的结果），不要手填
gravity_init:       [0.0, 4.893, -8.481] # 可选：body 系重力向下（省掉 200 帧等待）
```

* **默认 `off`，推荐 `hybrid`**：`off` = 与未打补丁的二进制**同值**（实测 `v0_base` 30.110° vs `smoke0` 30.129°，
  S1 §9.3 #1/#2）⇒ 默认路径零风险。
* **`gravity_align_rpy` 必须由 launch 从 URDF 自动算**（读 `base_link→livox_frame` 的静态 TF 再取 rpy，
  与 `_LioAdapterRobot11` 现在从几何算 rpy 的做法同源，L:2073/219-252）。**禁止手填**：
  手填的值与实物不符时，`urdf` 档**没有任何告警**（S1 §9.4(5) 风险列）。

### 4.2 机制（两步，缺一不可）

1. **设定初始旋转**：`R = FromTwoVectors(down_body, down_world)`（最小旋转，把 body 系测到的"下"转到世界 −z），
   写进 `kf.x.rotation`，同时 `kf.x.gravity = R·down_body`、`kf.x.acceleration = −Rᵀ·g`（保证 `v̇=R·acc+g=0`）。
2. **同步把初始地图搬进世界系**：`gravity_aligned` 时初始地图必须按出点公式 `R·(R_L_I·p+T_L_I)+t` 落进世界系。
   **只做第 1 步会把初始地图整体转过 R**（S1 §9.1 最后一条）。
3. **`gravity` 参数本身仍然是世界系**：`fix_gravity_direction=false` 时逐字使用（`small_point_lio.cpp:57-59`，S1 §9.1）。

### 4.3 来源与互检

| 来源 | 需要什么 | 实测残差 | 风险 |
|---|---|---|---|
| `urdf` | URDF（或自动算出的先验） | **0.474°**（本表最优）【实测】 | 先验错 ⇒ 按错角度摆正、**无告警** |
| `imu` | IMU + 静止 ~2 s（200 帧均值） | **0.519–0.564°**【实测】 | 起步就在动 ⇒ 均值带误差 |
| `hybrid` | 两者 | **0.581°**（Δ=0.011°）【实测】 | 无 IMU 时退回纯先验、跳过互检 |

**互检阈值（`hybrid`）**：

| Δ(URDF, IMU) | 级别 | 行为 |
|---|---|---|
| **< 2°** | INFO | 用 **URDF** 先验 |
| **2–5°** | WARN | 用 **IMU**（并提示"先验可能过期/换过支架"） |
| **≥ 5°** | ERROR | 用 **IMU**（并提示"URDF 与实物不符，检查 `base_link→livox_frame`"） |

【实测】`v7b` Δ=0.011° ⇒ INFO/URDF（0.581°）；`v8b` 故意把轴写成 `(0,+30,0)` ⇒ Δ=42.206° ⇒ ERROR ⇒ 改用 IMU ⇒ **恢复**（0.646°，否则会是 30°）。
【实测】`v8`（同错轴但**没有 IMU 可用**）⇒ 跳过互检 ⇒ **发散**。

### 4.4 200 帧门必须保留

`fix_gravity_direction=true` 的 200 帧 IMU 均值门是**独立于重力值的第二个问题**：它把初始化推到
"机器人已经静止"之后。**去掉它 = 把初始化提前到第一帧点云（≈0.1–1 s），正好落在 Gazebo 出生掉落窗口**：
8 个 `fix=false` 变体里 **5 个发散 + 2 个云被拖花**，唯一稳定的是把 init 推迟到落定之后的 `v2b`
（【实测】S1 §9.3/§9.4(6)，计数已由 `0ee92b6` 更正）。
**代价**：≈2.0 s 机时（仿真 RTF≈0.4 ⇒ 约 4 s 墙钟；实测初始化 7.6–8.2 s vs 3.5–4.2 s，S1 §9.3）。
**真机建议**：把"200 帧"换成**显式静止检测**（S1 §9.6 第 2 条），但**不得**取消"静止才初始化"这个前提。

### 4.5 验收线：残余地面倾角 **< 0.7°**

| 变体 | 残差（主指标：原始云→TF） | 备注 |
|---|---|---|
| `pointlio`（原生 `Set_init`，正向对照） | **0.166°** | 干净地 <0.5° |
| `v6b`（`urdf` 先验 + 初始旋转 + 200 帧门） | **0.474°** | 本实验最优 |
| `v5d`（`imu` 重跑） | **0.564°** | 日志 `rot=(-30.005,−0.012,0.003)°` |
| `v5`（`imu`） | 0.519° | — |
| `v7b`（`hybrid`，推荐档） | **0.581°** | Δ=0.011° |
| `v8b`（`hybrid` 错轴 ⇒ ERROR⇒IMU） | 0.646° | 互检救回 |
| `v0_base`（基线，`off`） | **30.110°** | = 安装角 |
| `fastlio`（负向对照） | 30.464° | 不做对齐 |

**为什么线是 0.7° 而不是 0.5°**：`urdf` 0.474° 只是**边际**过 0.5°，`hybrid` 0.581° 直接不过；
跑间/测量噪声 ±0.1° ⇒ 0.7° 给推荐档留余量，同时仍比"什么都不做"（30.1°）严 43 倍。

### 4.6 这 0.5° 是**稳态偏置**，不是 init 误差

* 初始旋转准到 **0.005°**（`v5d` 日志 `(-30.005,−0.012,0.003)°`）；`v6b` 的先验更是精确的 −30.000°；
* 残差却仍有 0.47–0.58° ⇒ 误差产生在**初始化之后**（本 LIO 用 0.2 m 体素地图 + 单时间戳帧的固有姿态偏置）；
* 同档把 settle 从 15 s 拉到 45 s（`v6b`→`v6c`）：主指标 0.474°→**0.610°**、次指标 0.562°→**0.505°**、
  p95 4.8→8.3 mm ⇒ 一升一降、都在 ±0.1° 噪声内，**没有随 3 倍时间长大**（【实测】S1 §9.4(4)，`0ee92b6`）。
* ⇒ 契约只承诺"**稳态偏置 < 0.7°**"，**不承诺**"初始化误差 < 0.7°"（后者实测 < 0.01°）。

---

## §5 重定位契约

### 5.1 能力分层（P4 的落地）

| 槽 | 能力 | 只做 | **不许做** | 参照 |
|---|---|---|---|---|
| `map_source` | 资产 | 提供地图/先验 | 不发任何 TF | Nav2 `map_server` 不发 TF（S3 §6.3 表） |
| `global_relocalizer` | `cap_global` | 无先验 → 位姿假设 + 质量；**只回答"大概在哪"** | **不写 `map→odom`**；不与 tracker 抢发布权 | Autoware `pose_initializer`（S3 §3.1）、SICK initial positioning（S3 §3.4） |
| `local_tracker` | `cap_local` | 逐帧跟踪；**唯一** `map→odom`；输出质量 | 不"发明"全局位姿；丢失时**明确置 invalid**（不是静默保持） | REP-105 Frame Authorities（S2 §3.1/S3 §4.1）、AMCL `initial_pose_is_known_`（S3 §3.2） |

### 5.2 `initial_pose:=known|unknown|continue`

| 取值 | 含义 | 谁必须先跑 | `map→odom` 何时开始发 | 依据 |
|---|---|---|---|---|
| **`known`** | 操作员/API 已给初值（`/initialpose`、`initial_pose_*`、`map_start_pose`） | 可只起 `local_tracker` | **立即**（首帧配准成功后） | AMCL 的 `set_initial_pose` 语义（S3 §3.2 E4） |
| **`unknown`** | 无先验 | **必须**起 `global_relocalizer` | **只有**全局重定位给出位姿且质量过阈后 | AMCL `reinitialize_global_localization`（S3 §3.2 E7） |
| **`continue`** | 沿用上次会话的位姿（存档/`.posegraph`/`.pbstream`） | `map_source` + 一致性检查 | 一致性检查通过后 | 本仓已有 `map_autocontinue`/`map_start_pose`/`map_resume_check`（L:1475/1482/1518） |

**铁律**：`initial_pose:=unknown` 且 `global_relocalizer:=none` ⇒ **非法组合**（§6 `E-INIT-01`）。
任何 `local_tracker` 在"没有可信初值"时**不得**发 `map→odom` —— 这是本仓 `gicp` 已有的语义（"没有初值就不发 TF"，S3 §6.1 R4），与 AMCL 同源，不是本仓发明（S3 §3.2 第 5 条）。

### 5.3 状态机

```
        ┌──────┐  initial_pose:=known / relocalizer 成功   ┌──────────┐
        │ IDLE │ ─────────────────────────────────────────► │ TRACKING │
        └──────┘                                            └────┬─────┘
            ▲   ▲                                                  │ health.invalid
            │   │ initial_pose:=unknown                          │ 或 TF 停发 > T_trip
            │   └──────────► ┌───────────────┐                     ▼
            │                │ RELOCALIZING  │ ◄─────────────── ┌──────┐
            │                └───────┬───────┘   操作员/API 重初始化 │ LOST │
            │                        │ 成功                           └──────┘
            │                        └──────────► TRACKING
            └──── tf_watchdog:=stop 且 T_trip 超时（发零速、取消 goal）
```

| 状态 | 发 `map→odom`？ | `health.state` | 下游动作 |
|---|---|---|---|
| `IDLE` | **否** | `IDLE` | 允许规划，**不**允许运动（无定位） |
| `RELOCALIZING` | **否** | `RELOCALIZING` | 同上；要求**静止**（SICK 明写；Autoware 讨论同结论，S3 §3.4 注） |
| `TRACKING` | **是** | `TRACKING` | 正常运行 |
| `LOST` | **否**（明确失效，**不**静默保持） | `LOST` | `tf_watchdog` 按档位处置（§5.5） |

### 5.4 健康信号字段表（统一接口）

话题：`/<source>/health`（每个位姿源一个），频率 **≥1 Hz**（I6）。

| 字段 | 类型 | 必需 | 含义 / 判据 |
|---|---|---|---|
| `valid` | `bool` | ✅ | 本帧输出可用；`false` ⇒ 下游**不得**使用其位姿 |
| `quality` | `float64` | ✅ | `[0,1]`；含义由源定义（GICP：`fitness_score` 映射；AMCL：粒子协方差 + 最近更新龄） |
| `state` | `string` | ✅ | `IDLE`/`RELOCALIZING`/`TRACKING`/`LOST` |
| `stamp` | `builtin_interfaces/Time` | ✅ | **源时刻**（不是发布时刻）；I7 的时效判据用它 |
| `source` | `string` | ✅ | 哪个槽/节点（`local_tracker=gicp` 等） |
| `reason` | `string` | ⭕ | `valid=false` 或 `state=LOST` 时**必须**给可读原因 |
| `detail` | `KeyValue[]` | ⭕ | 源特有量（`fitness_score`、`inliers`、`rejections`） |

【引用 S3 §6.4 第 1 条】`bool valid + float64 quality + string state` 是 SICK "measurement quality"（S3 §3.4）
与 Autoware `/diagnostics`（S3 §3.1）的最小公共子集；`stamp/source/reason/detail` 是本文【设计】补充。

### 5.5 `tf_watchdog:=warn|degrade|stop`

| 取值 | `now - stamp > T_warn(0.5 s)` | `> T_trip(2 s)` | 适用 |
|---|---|---|---|
| **`warn`（默认）** | 限速 WARN（1 条/5 s）+ `health.state=STALE` | 继续 WARN，**不**干预 | 现状兼容（等价于"只加观测"） |
| `degrade` | WARN | 允许**有限外推**（≤ **3 m**），到界即停 | 对齐 SICK 的 virtual positioning（S3 §3.4） |
| `stop` | WARN | 发零速 + 取消当前 goal + `state=LOST` | 工业默认（Autoware MRM，S3 §3.1/§4.2） |

**阈值的来源**：【设计】。`T_warn=0.5 s` ≈ 5 帧 @10 Hz（LIO 实测 10.000 Hz，S1 §9.3）；
`T_trip=2 s` 与既有 `gicp.lost_after_rejections=25`（2.5 s @10 Hz）同量级且更保守（`gicp_registration_sim.yaml:262`）。
**3 m** 直接取 SICK 的有界外推值（S3 §3.4 E21）。
**这些是默认值，需在真机按"定位更新率/最高车速"重定**（S1 §9.6 第 3 条同款告诫）。

---

## §6 校验矩阵

### 6.1 合法组合（示例，非穷举）

| # | `mode` | `map_source` | `global_relocalizer` | `local_tracker` | 形态 | 说明 |
|---|---|---|---|---|---|---|
| L1 | `mapping` | `none` | `none` | `none` | 建图 | `mapper` 槽生效（slam_toolbox/cartographer），`map→odom` 由它发 |
| L2 | `slam_nav` | `none` | `none` | `none` | 边建边导航 | 在线 SLAM 直接持有 `map→odom`（L:1815-1819） |
| L3 | `nav` | `2d_grid` | `amcl_global` | `amcl` | **M1** | 一体机；`global_relocalizer` 与 tracker 是**同一节点**，允许 |
| L4 | `nav` | `2d_grid` | `none` | `amcl` | M1（只给初值） | `initial_pose:=known` 时必须 |
| L5 | `nav` | `pcd` | `descriptor` | `gicp` | **M2 串联** | 全局给先验 → 局部精配准；`initial_pose` 可为 `unknown` |
| L6 | `nav` | `pcd` | `none` | `gicp` | M2（外部初值） | 必须 `initial_pose:=known` 或 `/initialpose` |
| L7 | `nav` | `posegraph` | `none` | `slam_toolbox_localization` | M1 变体 | 起点靠 `/initialpose`（S3 §2） |
| L8 | `nav` | `pbstream` | `none` | `cartographer_localization` | M1 变体 | 图冻结；只支持局部窗口（S3 §3.3） |
| L9 | `nav` | `none` | `none` | `none` | **LIO 当绝对定位**（回退） | 需要 `map≡odom` 单条静态桥（L:2135-2142）；v3 记为 `degraded` |

### 6.2 非法组合（每条对应一个 `check_contract` id）

| # | 组合 | 为什么非法 | id |
|---|---|---|---|
| X1 | `nav` + `local_tracker:=none` + 无 `map≡odom` 桥 | 没人发 `map→odom` ⇒ nav2 起不来 | `E-TF-01` |
| X2 | `global_relocalizer:=amcl_global` + `local_tracker:=amcl` **启动成两个进程** | 两个 `map→odom` 发布者（I1/I3） | `E-TF-02` |
| X3 | `map_source:=2d_grid` + `local_tracker:=gicp` | 资产类型与算法不符（2D 栅格喂 3D 配准） | `E-CAP-01` |
| X4 | `initial_pose:=unknown` + `global_relocalizer:=none` | 永远进不了 TRACKING（§5.2） | `E-INIT-01` |
| X5 | `initial_pose:=unknown` + `local_tracker:=gicp`（且无全局器） | `gicp` 无初值不发 TF ⇒ 永远 IDLE | `E-INIT-02` |
| X6 | `gravity_align_rpy` 手填值与 URDF 自动算值不一致（含错轴） | 先验说错、实物 −30° ⇒ 世界按错角度摆正；`urdf` 档**无告警** | `E-GRAV-03` |
| X7 | `gravity_align_mode:=off` + 任何"z 参数住在斜 odom 里"的槽 | I5 违约（继承态，WARN） | `W-GRAV-01` |
| X8 | `tf_watchdog:=stop` 但 `local_tracker` 无健康信号 | 无法判失效 ⇒ `stop` 永远不会触发 | `E-HEALTH-02` |
| X9 | `mapper:=slam_toolbox` + `localization:=slam_toolbox` 同时生效 | 同一包两个入口抢 `map→odom` | `E-TF-03` |
| X10 | `sim_backend:=gz` + `robot11_mount:=plugin` | gz 只实现 sensor 语义（S8 §0） | `E-SIM-01` |

### 6.3 三条错误文案（逐字样例）

```
[check_contract] E-TF-01  没有 map→odom 的发布者
  判据 I3：map→odom 必须由 local_tracker（或 M1 一体机）发布
  证据：mode:=nav, map_source:=2d_grid, global_relocalizer:=none, local_tracker:=none,
        且未启用 map≡odom 回退桥（bringup_sim.launch.py:2135-2142 的条件为假）
  修复：① local_tracker:=amcl（M1，配 map_source:=2d_grid）
        ② global_relocalizer:=amcl_global + local_tracker:=amcl（注意：同一体机不要起两份）
        ③ 明确接受降级：local_tracker:=none + 启用 map≡odom 桥（L9）
```

```
[check_contract] E-GRAV-03  gravity_align_rpy 的手填值与 URDF 自动算值不一致
  判据 I5/P2：倾角只在 URDF；先验 rpy 必须由 launch 从 base_link→livox_frame 自动算
  证据：robot:=robot11 的 URDF 关节自动算得 rpy=[-30.000,0,0]°；
        而 gravity_align_rpy 被手工设为 [0,+30,0]°（错轴）
  修复：删掉手填值（让 launch 自动算），或改 gravity_align_mode:=hybrid
        （hybrid 用 Δ(URDF,IMU) 抓这种不一致：v8b 实测 Δ=42.206° ⇒ ERROR + 改用 IMU，S1 §9.3 #17）
```

```
[check_contract] E-HEALTH-02  tf_watchdog:=stop 但没有健康信号
  判据 I6/I7：每个位姿源必须发 ≥1 Hz 的 /<source>/health；消费者用时效而非存在性
  证据：local_tracker:=amcl —— 该槽只发 /amcl_pose 与 /particle_cloud，无健康话题（S3 §1.3）
  修复：① 换 local_tracker:=gicp|small_gicp（已有 ~/fitness_score + ~/converged）
        ② 或给 amcl 加一个合成健康节点（最近更新龄 + 粒子协方差，S3 §6.4 第 1 条）
        ③ 或把 tf_watchdog 降为 warn（只观测，不联锁）
```

---

## §7 兼容层（v3a：旧参数 → 新槽位的别名表，现有命令零改动）

| 旧参数（HEAD） | v3a 行为 | 映射到 | 删除于 |
|---|---|---|---|
| `lio:=fastlio\|pointlio\|small_point_lio\|none\|cartographer` | **原样可用**，不发 DEPRECATED | `odom:=<同值>`（`cartographer` 记为全包降级） | v3b |
| `localization:=amcl` | 原样 | `map_source:=2d_grid` + `global_relocalizer:=amcl_global` + `local_tracker:=amcl`（M1，同一节点） | v3b |
| `localization:=beluga` | 原样 | 同上（`local_tracker:=beluga`） | v3b |
| `localization:=slam_toolbox` | 原样 | `map_source:=posegraph` + `global_relocalizer:=none` + `local_tracker:=slam_toolbox_localization` | v3b |
| `localization:=cartographer` | 原样 | `map_source:=pbstream` + `local_tracker:=cartographer_localization` | v3b |
| `localization:=icp\|gicp\|small_gicp` | 原样 | `map_source:=pcd` + `global_relocalizer:=none` + `local_tracker:=<同值>` | v3b |
| `mapper:=slam_toolbox\|cartographer`（仅 `mode:=mapping`） | 原样 | `map_source:=posegraph\|pbstream` + `mode:=mapping` | v3b |
| `robot11_mount:=sensor` | 原样（= v3 的唯一语义） | （无，语义合并进 `robot`） | v3b（参数消失，行为不变） |
| `robot11_mount:=plugin` | **原样**，打 `DEPRECATED: 平帧+斜数据，v3b 移除` | 暂时保留 | v3b（**唯一的破坏性变更**） |
| `robot11_mount:=urdf` | **原样**，打 `DEPRECATED: 诊断档，v3b 移除` | 暂时保留 | v3b |
| `livox_tilt_axis` | 原样 | （无） | v3b |
| `world`/`gui`/`nav_rviz`/`robot_visual`/`spin_speed`/`cloud_accumulator`/`map_name`/`sim_backend` | **原样**，语义不变 | 仿真/显示层 | 不删 |

**"现有命令零改动"的验收**：任一 HEAD 上验证过的命令行，在 v3a 上**逐字节相同的参数**必须跑出
**相同的契约**（`check_contract` 不新增 ERROR；默认路径的 TF/话题集合不变）。
**进入 v3b 的前置条件**（三条同时）：① gz 后端已不依赖 `robot11_mount`（S8 §0 的硬要求消失）；
② `lio_tf_adapter` 已成为所有 LIO 槽的唯一 `odom→base_link` 权威（I2 无例外）；
③ `check_contract` 在 CI 上对 `(mode, robot, 8 槽)` 笛卡尔积全绿。

---

## §8 `check_contract:=True` 输出样例

> **【设计】** 以下为 v3 规定的输出格式与内容，**尚未实现**。它由 launch 在 `OpaqueFunction` 阶段
> （与 `_validate_livox_mount` 同时机，L:175-181）静态求值，**不启动任何节点**，秒级返回。

```text
$ ros2 launch rm_nav_bringup bringup_sim.launch.py mode:=nav robot:=robot11 \
      lio:=small_point_lio localization:=gicp check_contract:=True
[check_contract] frame_contract_v3 · 3 条 WARN（I2 已知例外 / I5 继承态 / I6 缺健康），0 条 ERROR
[check_contract] 上下文
  mode=nav  robot=robot11  sim_backend=classic  world=RMUL2026
  odom=small_point_lio  map_source=pcd  global_relocalizer=none  local_tracker=gicp
  ground=linefit  nav=rpp  planner=navfn  local_obstacle=scan  global_obstacle=stvl
[check_contract] 参数分层（P1）
  描述层     : robot=robot11  → base_link→livox_frame rpy=[-30.000,0.000,0.000]° |t|=0.204443 m  [URDF]
  仿真/显示层: sim_backend=classic  world=RMUL2026  gui=True  nav_rviz=True  robot_visual=decimated  spin_speed=0.0  cloud_accumulator=False  map_name=<world>
  算法层     : 8 槽全部已解析（见上）
[check_contract] TF 边与 owner（P3 / I1–I4）
  map→odom        : gicp_registration                动态 10 Hz     [OK] 唯一
  odom→base_link  : small_point_lio（节点内联）       动态 10 Hz     [WARN I2] 应为 lio_tf_adapter
                    └ 已知例外：B1（publish_tf:=false）未落地前不报 ERROR（S4 §8）
  base_link→livox_frame : robot_state_publisher      静态 latched   [OK] 与 URDF 一致
  base_link→base_link_fake : fake_vel_transform      动态 20 Hz     [OK]
  camera_init/body/aft_mapped : 不在契约内（LIO 内部帧，无消费者）
[check_contract] 重力（I5）
  gravity_align_mode=off → odom 不重力对齐（继承态）
  [WARN W-GRAV-01] 实测基线 odom→base_link roll≈30.110°（S1 §9.3 #1）
    影响：局部代价图沿坡向压扁 13.17%；/scan 出高度带 39.6–40.4%（S1 §9.2；S6 §K.1.3）
    修复：gravity_align_mode:=hybrid（推荐；实测残差 0.581°，S1 §9.3 #15）
          先验由 launch 自动算（-30.000°），不要手填
[check_contract] 健康信号（I6）
  local_tracker=gicp : ~/fitness_score + ~/converged → /local_tracker/health  [OK]（映射后 ≥1 Hz）
  odom=small_point_lio : 无 health 话题                                        [WARN I6]
    修复：adapter/节点补 1 Hz `/odom/health`（含 gravity_align 结果行与 /odom 龄期）
[check_contract] 时效（I7）
  tf_watchdog=warn（默认）T_warn=0.5 s / T_trip=2.0 s  [OK]
  注：警告——nav2 的 TransformAvailable 只查存在性，不能替代本项（S3 §3.2 E6）
[check_contract] 能力相容性（P4 / §6）
  map_source=pcd × local_tracker=gicp          [OK] L6（需 initial_pose:=known 或 /initialpose）
  initial_pose 未显式给出                      [OK] 默认 known（兼容现状）
[check_contract] 结论：可启动（3 条 WARN 必须知情：I2 的 B1 未落地 / I5 的 off 继承态 / I6 的 odom 无健康信号）
```

**格式要求**：每条 finding = `[级别 id]` + 一行结论 + `判据/证据/修复` 三段（可缩进）；
级别只有 `ERROR`（拒绝启动）/`WARN`（允许启动但必须打印）/`INFO`（一次性说明）。
**退出码**：有 ERROR ⇒ launch 非零退出（与 `_validate_livox_mount` 的"可操作报错"同一风格，S4 §8 选项 C ②）。

---

## §9 验收测试（每条不变式怎么验）

### 9.1 I1–I7 的验法

| 不变式 | 怎么验 | 命令/证据 | 期望 |
|---|---|---|---|
| **I1** 唯一发布者 | 对 `(mode, robot, sim_backend, 8 槽)` 笛卡尔积**静态**求值；再对每个组合跑 `view_frames` 检查重复边 | `ros2 run tf2_tools view_frames`；`ros2 topic info /tf -v`（查 publisher 数）；静态：`python3 tools/scripts/regress/<新增>_tf_publisher_uniqueness.py`（S4 §8 验收①） | 每条边恰好 1 个 owner；`/tf` 的 publisher 数 = 声明数 |
| **I2** adapter 唯一 | `grep` 全栈 `sendTransform` + 运行期 `ros2 node info` | `grep -rn "sendTransform" src/` | 只有 adapter（B1 落地后） |
| **I3** map→odom 唯一 | 同上 + 检查 `mapper`/`localization` 互斥条件 | L:1815-1819 的条件求值 | 无两个 `map→odom` 发布者 |
| **I4** 全局器不发 TF | `ros2 node info <global_relocalizer>` 无 broadcaster；`view_frames` 里 `map→odom` 的 owner 不是它 | 同 I1 | 通过 |
| **I5** 重力对齐 | ① `check_contract` 的 `W-GRAV-01`；② 跑一次静止 15 s，用**原始云经 TF 搬进 odom** 拟合地面（S1 §9.2 的方法） | 复用 `tools/scripts/.../grav_probe.py`（S1 §9 汇总脚本） | 残差 **<0.7°**（§4.5） |
| **I6** 健康信号 | `ros2 topic hz /<source>/health` + `ros2 topic echo` 看字段齐全 | — | ≥1 Hz，§5.4 字段全有 |
| **I7** 时效而非存在 | 见 §9.2 的 `SIGSTOP` 测试 | `kill -STOP <pid>`；`ros2 topic echo /cmd_vel_nav` | 冻结后 **≤ T_trip + 1 帧** 内停车/告警 |

### 9.2 `SIGSTOP` 看门狗测试（一条命令可复现）

1. 起栈：`mode:=nav lio:=small_point_lio localization:=gicp ...`（任一条已验证命令）；
2. 记录 `local_tracker` 的 PID：`ros2 node info` 或 `pgrep -f gicp_registration`；
3. **冻结**：`kill -STOP <pid>`（`SIGSTOP` 让节点停止调度但**不退出** ⇒ 复现"TF 还在、内容冻结"这个最坏情况，
   它与 R5 的失败模式同形，而"进程崩了"是更容易被发现的另一种情况）；
4. 观测：`ros2 topic echo /local_tracker/health` ⇒ `state` 应变 `STALE`/`LOST`、`valid=false`；
   `ros2 topic hz /tf` 里 `map→odom` 停发；
5. 判据：`tf_watchdog:=warn` ⇒ 限速 WARN 出现；`:=degrade` ⇒ 外推 ≤3 m 后停；`:=stop` ⇒ `/cmd_vel_nav` 归零 + goal 取消；
6. **恢复**：`kill -CONT <pid>` ⇒ 状态机回 `TRACKING`（若质量过阈）或停在 `LOST` 等 `/initialpose`；
7. **反例（必须失败）**：只靠 `TransformAvailable` —— 冻结后它**仍然返回 success**（S3 §3.2 E6），
   控制器继续跟路径（R5：575/581 样本仍有 `/cmd_vel_nav`，最坏 110 m）。

### 9.3 C1–C4：四种初值情形

| # | 情形 | 起什么 | 期望 | 判据 |
|---|---|---|---|---|
| **C1** | `initial_pose:=unknown`，有地图、有全局器 | `global_relocalizer` + `local_tracker` | `IDLE→RELOCALIZING→TRACKING`；**RELOCALIZING 期间 `map→odom` 不发** | 状态序列与 `health.state` 一致；RELOCALIZING 期 `/tf` 无该边 |
| **C2** | `initial_pose:=known`，给对初值 | 只 `local_tracker` | 首帧后 `TRACKING`；`map→odom` 立刻发 | `map→odom` 首条 ≤1 s |
| **C3** | `initial_pose:=continue`，存档一致 | `map_source` + `map_resume_check` | 检查通过后 `TRACKING`；yaw/位置与存档差 < 容差 | `map_resume_check` 的 pos/yaw 判据（L:1534-1539） |
| **C4** | `initial_pose:=known` 但**给错**（偏移 2 m / yaw 错 90°） | 只 `local_tracker` | **必须**出现 `valid=false`/`quality` 跌穿阈值 ⇒ `LOST`，**不得**静默继续 | `gicp` 的 `gate_*`+`lost_after_rejections`（S3 §6.1 R3）或 AMCL 粒子协方差；`/cmd_vel_nav` 在 `T_trip` 内归零（`stop`/`degrade`） |

> **C4 是最重要的一条**：它把"位姿源坏了"变成**可观测**的；缺了它，`warn`（默认档）就退化成"知道坏了但不处置"。
> **注意**：C4 的"2 m / 90°"是【设计】取值（要大于 `gicp` 的 `plausible_margin_xy: 1.0 m`、`gate_yaw_step_max_deg: 15°`），
> 落地时应按实车的"最后一次可信位姿 + 运动包络"重定。

---

## §10 删除清单（进入 v3b 时生效）

### 10.1 `robot11_mount` / `robot11_mount_axis`

* **删什么**：`robot11_mount`（`plugin|urdf|sensor`，L:1287-1301）与其 `choices=list(_LIVOX_MOUNTS)`
  （L:172、L:1303）、`_validate_livox_mount`（L:175-181）、`livox_tilt_axis`（L:1257-1268）。
  `robot11_mount_axis` **本仓不存在**（全仓 grep 无命中）——任务书里这个名字对应的是 `livox_tilt_axis`。
* **为什么**：三档并存 = 倾角有三个所有者（P2、S4 §7.2 O3）；`<tilt_rpy>` 让 TF 说"平"、数据说"斜"（S8 §2.2）。
* **删之前必须做**：`sensor` 语义成为唯一语义（倾角在 URDF、点云在传感器系）；Classic 默认档从 `plugin` 迁到
  `sensor`（**唯一的破坏性变更**，必须单独一个 commit + 回归）；gz 后端的 `robot11_mount:=sensor` 硬要求消失（S8 §0）。
* **影响面**：`robot:=robot11` 的默认点云表达（水平 → 斜传感器系）、`linefit.gravity_aligned_frame`/p2l `target_frame`
  的增量覆盖（L 里只对 `sensor` 档加的那两份增量文件，S6 §J.3）、所有已实测基线（S6 §A/§J/§K 的数字）都要重标口径。

### 10.2 `camera_init→map` 全 0 静态桥（`launch:2117-2118`）

* **删什么**：`tf_bridge_camera_init_to_map`（L:2116-2119，`arguments=['0','0','0','0','0','0','camera_init','map']`）
  与兄弟桥 `tf_bridge_body_to_odom`（L:2125-2128）。
* **为什么**：它们把 `map ≡ camera_init` **写死成恒等**——"倾斜不是缺模块，是被这条边写死成了恒等"（S2 §6.1 第 2 条）。
  在 E1 之后 `camera_init` 这个帧要么不存在（`small_point_lio` 没有它），要么已被对正，桥只会制造第二份真相。
* **附带清掉**：`src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` 的**单位**
  `base_link→livox_frame` 静态 TF（上游是死代码，我们把它加进了 `LaunchDescription`，S4 §7.2 O4）——
  它与 URDF 的 −30° 关节**互斥**，同时存在即双倍倾角。**注意**：当前 bringup 不走这个 launch（L:1794-1806 自建 Node）
  ⇒ 是**潜伏**坑。
* **删之前必须做**：`mode:=nav` 的 L9 回退（LIO 当绝对定位）需要一条**显式**的 `map→odom` 发布者，
  不能再用"全 0 静态桥"冒充；改由 `local_tracker:=none` + `map_source:=none` + 明确的 `degraded` 声明（§6 L9）。

### 10.3 插件 `<tilt_rpy>` / `<cloud_frame>` 开关

* **删什么**：Livox 插件 SDF 里的 `<tilt_rpy>`（承担倾角）与 `<cloud_frame>`（决定点云表达在哪个系）两个开关。
* **为什么**：`<tilt_rpy>` 与 URDF 关节是同一事实的两个所有者（P2）；`<cloud_frame>` 是"点云表达在父 link 还是传感器系"
  的开关，而 v3 只允许**一个**答案（传感器系，与 `frame_id=livox_frame` 自洽，S6 §J.3）。
* **删之前必须做**：URDF 关节承载倾角（`sensor` 语义）已在 Classic/gz 两侧都验证过（S6 §J、S8 §5）；
  历史 A/B 档位在文档里保留为反例（S6 §C/§I 的数字仍在）。
* **影响面**：`robot11_mount:=plugin` 路径消失（同 10.1）；`urdf` 诊断档消失（它的价值已由文档承担）。

### 10.4 `localization` 的双重身份

* **删什么**：`localization`（8 值枚举，L:1353-1366）与 `mapper`（L:1430-1434）的**同名两项**
  （`slam_toolbox`/`cartographer` 同时出现在两个枚举里）。
* **为什么**：同一个包的两个入口、不同模式、靠 launch 条件互斥——是**命名/分层重复**，不是双发布者（S3 §6.1 R2），
  但它让"能力"不可见（P4），也让 `check_contract` 必须硬编码"哪些名字是 SLAM"。
* **删之前必须做**：`map_source` × `global_relocalizer` × `local_tracker` 三槽落地；旧命令由 §7 别名表承接。
* **影响面**：`--show-args` 的帮助文本、所有文档里的 `localization:=*` 示例、`algorithm_matrix.md` 的槽位表。

---

## §11 回退（每项怎么退，默认值 = 现状）

| 项 | 回退动作 | 退回到 | 风险 |
|---|---|---|---|
| `gravity_align_mode` | 设 `off`（或删掉三个新键） | 现状：`v0_base` 30.110°（S1 §9.3 #1） | 零（默认路径实测同值：`v0_base` vs 未打补丁 `smoke0` 差 0.019°，S1 §9.3 #1/#2） |
| `gravity_align_rpy` 自动算 | 改回手填（不推荐） | 现状：手填可省一次 TF 读 | 先验错 ⇒ 无告警（§4.1） |
| 200 帧门 | **不提供回退**（§4.4） | — | 删了会重现"5 发散 + 2 拖花"（S1 §9.3） |
| `check_contract` | 设 `False`（默认） | 现状：无静态检查 | 零（它不启动节点） |
| `tf_watchdog` | 设 `warn`（默认） | 现状：只观测 | 零 |
| `initial_pose` | 删掉（默认 `known`） | 现状：靠 `/initialpose`/`initial_pose_*` | 零 |
| 三槽拆分（`map_source`/`global_relocalizer`/`local_tracker`） | 用旧 `localization`/`mapper`（§7 别名全期可用） | 现状 | 零（别名期） |
| 删除 `robot11_mount`（v3b） | 重新加回参数与三档 | 现状：`plugin` 默认 | **唯一的破坏性变更**，需单独回归 |
| 删除 `camera_init→map` 桥（v3b） | 加回静态桥 | 现状 | 只有 L9 回退用法依赖它 |
| 删除插件 `<tilt_rpy>`/`<cloud_frame>`（v3b） | 加回两个开关 | 现状 | 与 URDF 倾角互斥，需同时撤 URDF 关节 |

**总则**：**每个 v3 新增项都有"关掉 = 现状"的开关**；只有三个**物理删除**项（10.1/10.2/10.3）是**单向**的，
它们必须在 v3b 一次性完成并配回归，不允许"半删"；10.4（`localization` 双重身份）由 §7 别名承接，**可回退**。

---

## §12 明确**不在**契约内

| # | 不承诺什么 | 为什么 | 谁负责 |
|---|---|---|---|
| 1 | **6DoF 精度指标**（ATE/RPE、绝对角精度） | 契约管"谁发哪条边、语义是什么"，不管"多准"；精度是各算法的验收项 | `algorithm_matrix.md` §四、各算法文档 |
| 2 | **多机共享地图** | 单机命名空间、单 `map` 帧、单 `map_source`；多机要另一套帧/命名空间设计 | 未立项（S3 §8 亦未覆盖） |
| 3 | **动态地图**（实时更新 `map` 资产） | `map_source` 是"加载资产"，不是"维护地图"；在线建图走 `mode:=mapping/slam_nav` | `mapping_*.md` |
| 4 | **多机器人命名空间**（`/robot1/tf` 等） | 与 2 同因；当前 launch 全是全局名 | 未立项 |
| 5 | **云台时变倾角** | 需要 J 档（关节链）+ URDF `movable joint`；本仓 `base_link→livox_frame` 是**固定关节**（S4 §7.1 I1、S1 §4.1） | 前瞻项，见 R §8 |
| 6 | **roll/pitch 漂移的在线估计** | 一次性初始化只定初值；持续漂移要靠 IMU 重力 EMA / 地面约束 / 地图匹配（S1 §5.2） | 未立项 |
| 7 | **`odom` 的绝对 yaw** | 重力只定 2 DoF，yaw 永远另有来源（地图/重定位）（S2 §1.2） | `local_tracker` |

> **一句话**：v3 只承诺 **"帧树 + 参数归属 + 重力语义 + 谁发 TF + 怎么发现自己坏了"** 这五件事；
> 精度、多机、动态地图、时变挂装、漂移估计**都不在里面**——写清楚"不做什么"与写清楚"做什么"同等重要。

---

## §13 实现状态（2026-10-20：选项 U 落地 + 实测）

> 本节只回答两件事：**§2/§3/§4 的哪几条现在真的在代码里**、**哪几条仍是设计**。
> 实测数字全部在 **`docs/frame_contract_v3_measurements.md`**（效率 + 精度三列表、命令、方法、未验证项）。

### 13.1 现在**已实现**（有代码 + 有实测）

| 契约条目 | 实现 | 证据 |
|---|---|---|
| **§2.1 `odom→base_link` 唯一 owner = `lio_tf_adapter`** | `lio:=small_point_lio` 且 `gravity_align_mode != off` 时：LIO 传 `publish_tf:=false`、`odom_convention:=sensor`，并且**给这个槽位也起 adapter**（此前被显式排除）⇒ 该边全栈恰好一个发布者 | `small_point_lio_node.cpp`（新参数 + `publish_tf_` 分支）、`odometry_convention.cpp`（sensor 档发布）、`bringup_sim.launch.py`（两个互斥 Node + `lio_tf_adapter_spl_node`）；实测：`ros2 node info /small_point_lio` **无 `/tf` 发布者**、`/lio_tf_adapter` 有（M §3.3） |
| **§3 I2（`odom→base_link` 只由 adapter 发）** | 同上；默认档（`off`）仍由 LIO 自己发 = **已知例外保留**（`check_contract` 未实现，所以它仍会 WARN 而不是 ERROR） | 同上 |
| **§4 E1 重力对齐（`urdf|imu|hybrid`）接入 launch** | `gravity_align_mode:=off|urdf|imu|hybrid`（默认 `off`）；**`gravity_align_rpy` 由 launch 从 URDF 自动算**（`_GravityAlignRpy` → `_urdf_livox_mount_rpy_deg`，与 `_lio_adapter_robot11` 同源同一条安装边） | `bringup_sim.launch.py`；日志实测 `R_world_body rpy=[-30.000 0.000 0.000]`、`Δ(URDF,IMU)=0.006/0.024/0.007° ⇒ INFO/URDF`（M §3.2） |
| **§4.5 验收线 < 0.7°** | 主指标（原始云→TF→世界系拟合）实测：`hybrid` **0.346/0.488/0.384°**、`urdf` **0.613°**、基线 **30.08–30.38°**、`pointlio` 0.111/0.267° | M §3.1 |
| **§2.1 `T_ol·T_bl` 的**一次**合成 + 三方 `T_bl` 记账** | launch 里写成**注释块**（LIO-internal 点一次 · E1 先验 `R_bl⁻¹` 一次 · adapter `T_bl` 一次，三者都来自 URDF 的同一条边） | `bringup_sim.launch.py`「三方 `T_bl` 记账」注释；数值交叉验证：`/Odometry` rpy −29.735° + adapter rpy +30° = TF rpy **0.265°**（M §3.2） |
| **§4.4 200 帧门保留** | 未动（`fix_gravity_direction` 逻辑一行未改） | 代码 diff；实测 init 2.1–2.2 s 仿真 / 11.2–11.7 s 墙钟（与 E1 的 7.6–8.2 s 差异见 M §6） |
| **默认路径零改动**（§11 总则） | `off` 档：节点集合/参数/顺序不变（launch 增量证明），**同一输入流重放逐位相同**（141 行全等） | M §4 |

### 13.2 仍是**设计**（本轮没做，别当成已实现）

| 未实现项 | 现状 | 阻塞点 |
|---|---|---|
| **`check_contract:=True`**（§6/§8 的静态校验矩阵） | **不存在**（本轮明确不做）。当前的替代品 = launch 期组合检查（`_check_gravity_align_contract`：选错组合直接**报错停机**）+ `gravity_align_mode` 的 `choices` | 需要一条"静态求值 launch 条件"的评估器；`launch_increment_diff.py` 只做到"比较两棵 Action 树" |
| **I1 全笛卡尔积静态证明** | 只做了**点坐标**上的节点集合证明（`lio∈{small_point_lio}×gravity_align_mode∈{off,urdf,imu,hybrid}×robot∈{robot11}`） | 同上 |
| **I3/I4/I5/I6/I7** | 原样：`local_tracker` 唯一 `map→odom` 已由"选的时候唯一"保证；健康信号仍只有 `gicp`/`small_gicp`；`tf_watchdog` 未实现 | 与本轮无关项 |
| **§10 删除清单（v3b）** | 一项未删：`robot11_mount` 三档、`camera_init→map` 桥、插件 `<tilt_rpy>`/`<cloud_frame>` 全在 | 本任务**明令不删槽位** |
| **`odom` 重力对齐成为默认** | 默认仍是 `off`（= `odom` 不重力对齐 = **grandfathered**，按 §2 的合规窗口报 WARN） | v3b 才把默认改成 `hybrid` |
| **`odom_convention:=sensor` 下的守卫 #2** | 该档**跳过** `T_ob·T_bl⁻¹ ≈ T_ol` 断言（`T_ob` 不在本节点产生了）；等价保障改由 **adapter + launch 的 URDF 外参**承担 | 要断言就得在 adapter 侧加；本轮未加 |
| **点云/外参的 `pub` 边界**（`/cloud_registered` 仍发 `odom`，不是 sensor 帧） | 未改动（仍发 `odom`、内容 = `p_odom`） | §4.2 的结论不变：主路径保持发 `odom` |

### 13.3 三个容易踩的**实施细节**（写在这里，省下一次返工）

1. **`publish_tf` / `odom_convention` 是"发布契约"参数，不是"算法"参数**：它们放在节点层
   （`small_point_lio_node.cpp` 构造期 `declare_parameter`，与 `save_pcd` 同一层），**不进** `Parameters` 类
   ⇒ 算法侧（`preprocess/eskf/ivm`）一行未动，`mid360_sim_tuned.yaml` 一个键都不用加。
2. **`odom_convention:=sensor` 的 `child_frame_id` = `lidar_frame`（默认 `livox_frame`），不是 `base_link`**：
   估计器状态严格说是 `odom←IMU body`（点先经 `extrinsic_T/R` 搬进 IMU 系），本包历史上把它当 `T_ol` 用；
   那 5 cm（livox↔imu）由 adapter 的 `xyz`（`T_imu←base_link`，URDF 几何算出）吸收 ⇒ 全链路只算一次。
   把 `base_link` 写进 LIO 的 `/Odometry` 会是**第二个所有者**（违 I1/I2）。
3. **sensor 档下本节点不再查询 `base_link→livox_frame`**：那条静态 TF 缺失时 `/Odometry` 照发
   （旧实现 `lookupTransform` 失败会连里程计一起丢，§4.3 A 的"强依赖"由此解除）——
   但代价是 `publish_tf:=false` 时**不再有外参守卫 #1 的 readout**（它依赖那次 lookup）。

### 13.4 与 §2 合规窗口的关系（一句话）

`gravity_align_mode:=off` 仍**不**满足 §2 的"`odom` 必须重力对齐"（实测倾角 **30.08–30.38°**）——
本轮把**合规档做成可一键启用**（`gravity_align_mode:=hybrid` 即满足 §4.5 与 I2），
但**没有把默认值改成合规**。默认改 `hybrid` 是 v3b 的动作，且要先解决"没有 IMU / 起步就在动"的降级语义（R §9(a)）。
