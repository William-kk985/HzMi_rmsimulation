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

> **行号基线（2026-10 复核）**：文中 `L:NNNN` / `launch:NNNN` 一律指
> `src/rm_nav_bringup/launch/bringup_sim.launch.py` 的行号，已按 **`8e5e928`（2853 行）** 逐条用
> `grep -n` / `sed -n` 复核过；旧稿是按 `c996365`（2584 行）编的，launch 重构（选项 U）后整体偏移，
> 本次已全部更正。`file:NNNN` 一类同批复核（`small_point_lio*`、`eskf.h`、`gicp_registration_sim.yaml`）。
> launch 再变长时**请重新 `grep -n`**，别照抄本文数字。

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

**本仓的既有事实**：`localization` 的 8 个值靠 launch 条件互斥（L:1528-1531），`mapper` 与它按 `mode` 互斥
（L:1605-1608、L:2027-2035）——**已经做到了"选的时候唯一"**，缺的是"运行期唯一 + 坏了谁接"（S3 §6.1 R1/R6/R8）。

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
| `robot` | `''`（默认模型）\| `hzmirm` \| `robot11` | link/关节/惯量/碰撞、**倾角与杆臂**、`base_link` 原点语义 | 提供 `base_link→livox_frame` / `base_link→imu_link` 静态边 | 默认 `''`（L:1383）；`robot11` 在 v3 语义（`sensor`）下倾角记在关节 `rpy=[-30,0,0]`、杆臂 `[0.000562,0.130916,0.157028]`（S4 §2.2 + S6 §J.1.2） |

**删除**：
* `robot11_mount`（`plugin|urdf|sensor`，L:1437-1452）——三档并存 = 倾角有三个所有者（S4 §7.2 O3）。
* `robot11_mount_axis` —— **本仓不存在此参数**（全仓 `grep` 无命中，已核）。真正存在的是
  `livox_tilt_axis`（`roll|pitch`，L:1407-1416），随 P2 一并删除；其取值之争见 S4 §7.2 O6
  （URDF 写 pitch、CSV 写 roll；`livox_tilt_axis` 默认 `roll`）。
  **2026-10-21 已实施**：launch 中 `livox_tilt_axis` 删除并冻结 `roll`；`livox_raise_m` 同时删除并冻结 0
  （证据与默认路径逐字节不变见 `docs/frame_contract_v3_measurements.md` §28）。
* v3 中 `sensor` 的语义**升格为唯一语义**：倾角在 URDF 里，点云表达在**传感器系**，
  `frame_id=livox_frame` 与数据自洽（S6 §J.3、S8 §2.2）。`plugin`（平帧 + 斜数据）与
  `urdf`（斜帧 + 平数据，诊断档）在 v3b 删除；v3a 期间仅作别名接受并打 DEPRECATED（§7）。

### 1.2 仿真/显示层（8 个参数）

| 参数 | 默认（HEAD） | 取值 | 拥有的事实 | 谁读它 | 依据 |
|---|---|---|---|---|---|
| `sim_backend` | `classic` | `classic` \| `gz` | 用哪个仿真器 | 仅仿真装配 | L:1373；S8 §0/§9 |
| `world` | `RMUL2026` | 世界名（同时决定 map/PCD 前缀） | 这次跑哪个场地 | 仿真 + 资产解析 | L:1356 |
| `gui` | `True` | `True`\|`False` | 起不起 gzclient | 仅仿真 | L:1322 |
| `nav_rviz` | `True` | `True`\|`False` | 起不起 nav2 RViz | 仅显示 | L:1296 |
| `robot_visual` | `decimated` | `decimated`\|`full` | `<visual>` mesh 档 | 仅显示 | L:1490-1498 |
| `spin_speed` | 槽位相关：`robot11` ⇒ `0.0`，其余 `5.0` | rad/s | `fake_vel_transform` 的小陀螺角速度 | `fake_vel_transform` | L:1343-1354 |
| `cloud_accumulator` | `False` | `True`\|`False` | 起不起 3D 累加器（仅 `mode:=mapping`） | 建图链路 | L:1672-1679 |
| `map_name` | `''`（= `<world>`） | 存档基名 | 这次读/写哪份存档 | 建图/定位 | L:1642-1648 |

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

> ★ 2026-10-21（**§4.11 起生效，本节保留为"回滚档 / E1 实验臂"的形状**）：默认路径的帧树已经变成
> `map ─► odom ─► odom_level ─► base_link`（重力对齐从估计器补丁搬到**静态边**）。
> 本节以下正文、§3 的 I1–I7 与 §4.7/§4.8/§4.10 在**默认档**上被 §4.11 取代；
> `level_frame:=odom`（回滚）时**逐字生效**。新增那条边的定义/所有者/不变量只看 §4.11。

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
| `base_link→livox_frame` | **URDF**（`robot_state_publisher`） | static | 一次（latched） | `R_x(−30°)` + `t=(0.000562,0.130916,0.157028)`（`\|t\|=0.204443 m`）【实测 S4 §2.2 / S5 §3.1】 | 与插件 `<tilt_rpy>` 同时存在 ⇒ **双倍倾角**；写错轴 ⇒ `livox_tilt_axis` 之争（S4 §7.2 O6；该参数已于 2026-10-21 删除并冻结 `roll`，见 measurements §28） |
| `base_link→imu_link` | **URDF** | static | 一次（latched） | IMU 安装位姿；MID-360 内置 IMU 的 `T_IL` 是**估计器参数**、不是这条 TF（S5 §1.2） | 与 LIO `extrinsic_T/R` 混用 ⇒ "转两次"（S5 §3.1 引 #433） |
| `base_link→base_link_fake` | **`fake_vel_transform`** | dynamic | 20 Hz【实测 L:2400】 | **只含 yaw**（小陀螺角），pitch/roll 透传（S4 §7.1 I6） | 缺失 ⇒ nav2 的 `robot_base_frame` 解析不到；`spin_speed≠0` 时它凭空转而雷达不跟着转（L:1348） |

### 2.2 `T_ol·T_bl` 的方向（最容易写反，写死在这里）

* `T_ol` = LIO 输出的 body(=livox) 位姿；`T_bl = lookupTransform(livox_frame, "base_link")` = **base_link 在 livox 系里的位姿**。
* 链式法则：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ **`T_ob = T_ol·T_bl`（右乘合成）**。
* `T_bl⁻¹·T_ol·T_bl` 是**换基（共轭）**，只在 `T_bl` 的旋转为 `I` 时姿态碰巧相同（S5 §3.2；上游 `qza36/COD_NAV` 的 `small_point_lio_node.cpp:179-187、:218-219` ——本仓 vendored 同源副本的行号—— 就是共轭；旧稿说"被 COD 的 `T_bl≡I` 约定掩盖"，**该前提已被 2026-10-18 实车实测证伪**：`T_bl` 非单位阵、共轭未退化，见 [`cod_real_frame_measurement.md`](cod_real_frame_measurement.md)，S4 §3.3 的 C 型结论已降级）。

---

## §3 不变式 I1–I7

> 每条都给**可执行判据**（`check_contract` 怎么查）与**违反的实测后果**。这七条是 §9 验收测试的对象。

### I1 唯一发布者

**判据**：对 TF 树里每条边，在 `(mode, robot, sim_backend, 8 个槽)` 的**笛卡尔积**上静态求值 launch 条件，
**恰好一个**节点声明该边的 `parent→child`。
**违反后果**：查询抖动/跳变；tf2 不报错（S3 §4.1）。
**现状**：`map→odom` 已由"选的时候唯一"保证（L:1528-1531、L:1605-1608）；`odom→base_link` 在
`lio:=small_point_lio` 时**唯一**（LIO 自己发，L:2240-2248 排除 adapter），其余 LIO 由 adapter 发 —— 也唯一，
但**权威不统一**（S4 §7.2 O2）。

### I2 `odom→base_link` 只由 `lio_tf_adapter` 发

**判据**：全栈 `grep` 该边的 `sendTransform` / `StaticTransformBroadcaster`；只允许 adapter 一处。
**违反后果**：两个发布者（LIO + adapter）⇒ 双父边；`small_point_lio` 槽位当前违反此条（它自己发，S4 §3.5）。
**修复路径（S4 §8 选项 B1）**：给 LIO 加 `publish_tf: true|false`（默认 `true`，保持默认路径逐字节不变），
bringup 传 `false`；**在 B1 落地前，I2 对 `small_point_lio` 记为"已知例外"**（由 `check_contract` 报 WARN，不报 ERROR）。

### I3 `map→odom` 只由 `local_tracker` / M1 发

**判据**：`global_relocalizer` 与 `map_source` 的节点**不得**出现在该边的发布者集合里；`mapper` 与
`localization` 不得同时生效（`mode:=slam_nav` 时不加载任何重定位槽，L:2037-2038）。
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
  与 `_LioAdapterRobot11` 现在从几何算 rpy 的做法同源，L:2285/396-418）。**禁止手填**：
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

### 4.7 odom 的 z 原点约定（E1b，2026-10-21）

> 任务书写的是"§4.3"；本体 §4.3 已被「来源与互检」占用（且被多份文档交叉引用），
> 为不打乱既有编号，新增节落在 **§4.7**。内容 = 规范；实测见
> `docs/frame_contract_v3_measurements.md` §10。

**规范（`gravity_align_mode != off` 且 `gravity_align_z_origin:=ground` 时）**：

1. **`odom` 的 z=0 = 初始化时刻的地面**。初始化状态位置
   `position.z = h − (R_ws·t_IL).z`，其中
   * `h` = **传感器（`livox_frame` 原点）离地高度**，由 launch 从 **URDF 几何**算（见第 2 条）；
   * `R_ws` = E1 算出的 body→world 初始旋转；
   * `t_IL` = 传感器在状态系（IMU body）里的位置 = `extrinsic_T`（默认 `(0,0,0.05)`）。
   * 为什么不是直接 `position.z = h`：本 LIO 的**状态原点是 IMU body**，livox 原点比它高
     `(R_ws·t_IL).z`。robot11 sensor 档 `R_ws=Rx(−30°)` ⇒ 该杆臂 = `0.05·cos30° = 0.043301`，
     `position.z = 0.259527 − 0.043301 = 0.216226`。
2. **初始地图必须用同一个平移**。初始地图点的出点公式是
   `R_ws·(R_IL·p + t_IL) + position`；只要 `position` 在上面设好，地图与状态就同步搬，
   **不要**另写一份 `+h`（那是第二个真源）。**只改旋转不改平移 = E1 遗留的 bug**
   （地面被抬到 −h，nav2 局部代价图整盘掉出高度带）；反过来只改平移不改旋转也不行。
3. **`h` 逐模型（默认姿态；`livox_raise_m` 已于 2026-10-21 删除并冻结 0，全部从 URDF/轮几何算，不手填）**：

   | 模型 | `base_link` 原点离地 | `livox_frame` z 偏置 | **`h`** | 依据 |
   |---|---|---|---|---|
   | `robot:=robot11` | 0.102499 | 0.15702816968305（launch 的 `livox_raise_m` 已删除、冻结 0） | **0.259527** | 轮 mesh 最低点 −0.102499；`body_to_livox` 关节 z；与 linefit `sensor_height: 0.2595` 交叉一致 |
   | `robot:=hzmirm` | 0.0 | 0.6 + 0.2·cos(`turret_pitch_deg`) | **0.800000**（pitch=0） | 轮 r=0.08 心 z=0.08 ⇒ 轮底在 base_link z=0；云台链 0.3+0.3+0.2 |
   | 默认模型（`robot:=''`） | 0.06 | 0.175 | **0.235000** | 轮 r=0.06 心 z=0；`measurement_params_sim.yaml` 的 xyz z |

   车体水平、URDF 与实物一致时 ⇒ **ground z=0、`base_link` z=+0.102499、`livox_frame` z=+0.2595**
   （robot11；数值 0.102504/0.259532，差 ≤5 µm 来自四舍五入）。
4. **运行时互检（loud, never silent）**。初始化时从**最后一帧**原始云（lidar 系）拟合地面，
   量出 `h_meas`，与 `h_urdf` 比较：

   | `\|Δ\|` | 级别 | 行为 |
   |---|---|---|
   | **< 0.03 m** | INFO | 用 URDF 值；日志同时打 h_urdf / h_meas / Δ / 内点 / 倾角 / 残差 |
   | **0.03–0.05 m** | WARN | 同上；提示"URDF 与实测不符 / 场地不平" |
   | **≥ 0.05 m** | ERROR | 同上；提示"URDF 写错或出生不在平地上" |

   **定基永远用 URDF 值（确定性）**，`h_meas` 只作诊断：实测会被噪声/坡面/车体点带偏，
   而 URDF 可复现。每次初始化只打一次（`reset()` 只在构造期调用）⇒ 天然 rate-limited。
5. **默认值**：节点参数 `gravity_align_z_origin` 默认 **`sensor`**（= 旧行为，position 保持 0，
   `/Odometry`/TF/点云逐位不变）；launch 里 `gravity_align_z_origin` 的默认值是 **`ground`**，
   但**只在 `gravity_align_mode != off` 时**把它和 `gravity_align_h` 传给节点 ——
   `off` 档那份 Node 的参数表一个键都不多，默认路径不受影响。
6. **验收线（nav 侧）**：`mode:=nav` 下 scan 盘面在 odom 的 z 必须落在 nav2
   `obstacle_layer.scan` 的 `[min_obstacle_height, max_obstacle_height] = [0, 2.0]` 内
   （`local` 与 `global` 两处运行期回读），且 local costmap 的 lethal/inscribed 非空。
   E1 档未修 z 时实测 **scan z = −0.158 m ⇒ 677/677 波束掉出带 ⇒ lethal 193→0**
   （measurements §9.7）；E1b 修复后 3 跑验收见 measurements §10。
7. **已知边界**：`ground` 档的"地面"是 **URDF 几何说的地面**（轮底平面），不是"雷达在当前
   场景里看到的那层地面"。两者不一致时（换场地/模型下沉/斜坡）由第 4 条的 INFO/WARN/ERROR
   报出来，**定基不跟着实测走**。

### 4.8 世界地面校准是**前置条件**（2026-10-19）

> **world 的地面必须落在 z = 0**；`RMUC2026` 与 `RMUL2026` **都已校准到 z=0**（RMUL2026 的
> Δz = −0.049662，见 `docs/frame_contract_v3_measurements.md` §13.6）。这是 §4.7 的
> `h` / `odom z=0` 能对上的前提；反例：`cloud_frame=sensor` 档的点云曾整朵偏 (0.6 mm, 19.2 cm,
> 7.05 cm)，让 `h_meas` 在**地面已经正确的 RMUC2026** 里也报 ERROR —— 那次不是地面，是 Livox
> 插件（measurements §13.4）。

### 4.9 高度带静默过滤的守卫（opt-in，2026-10-20）

> nav2 的 `min/max_obstacle_height` 是**静默**的（E1b 的 scan 盘面、measurements §15 的
> 全局 `map` 都吃过这个亏）：`band_guard:=True`（默认 `False`，`rm_nav_bringup`）只读地
> 复算每束 z 并报 `in-band==0`（ERROR）/`<10%`（WARN）/`kept-y` 单侧（WARN），
> 只写日志 + `/band_guard/status`；不改 scan/costmap/TF/控制。详见
> `docs/frame_contract_v3_measurements.md` §17。

### 4.10 默认档决定：`gravity_align_mode:=auto`（2026-10-20，决策 A）

> 本节是**追加**（append-only）：§4.1–§4.9 / §13 的正文一个字都没改。它**取代**下面三处关于
> "默认 = `off`"的表述：§4.1 的参数注释（`gravity_align_mode: "off"` ⇒ 默认 off）、§13.2 表格的
> 「`odom` 重力对齐成为默认 / 默认仍是 off」、§13.4 的「没有把默认值改成合规」。那三处写于
> 2026-10-20 选项 U 落地时，保留为历史；**默认值口径以本节为准**。

**决定**：launch 的 `gravity_align_mode` 默认值 = **`auto`**，语义为

```text
auto = hybrid   当 lio == small_point_lio      （= E1 补丁唯一存在的槽位）
     = off      其它 lio（fastlio | pointlio | none | cartographer）
```

即 **`lio:=small_point_lio` 这条路线（nav 的 SPL 槽位）默认就重力对齐**（用户签字的选项 A），
其余槽位保持逐字节现状。`auto` **本身不是一档算法**，只是一条解析规则。

#### 4.10.1 为什么必须是"解析档"，而不是把默认值直接写成 `hybrid`

* launch 的 `lio` 默认是 **`fastlio`**；而 §4.1 的硬约束是「**显式**非 off + 非 `small_point_lio`
  ⇒ `_check_gravity_align_contract` **t=0 硬报错**」（E1 补丁只打在 vendored 的 `small_point_lio`
  里；fastlio/pointlio 没有 `gravity_align_mode` / `gravity_align_rpy` 这些参数，传了也被无声丢弃）。
* ⇒ 默认值字面写 `hybrid` = **每一个默认（`lio:=fastlio`）launch 在 t=0 就停**。实测（§4.10.6 (d)）：
  `lio:=fastlio gravity_align_mode:=hybrid` ⇒ `exit 1` + 逐字报错
  「`[launch] gravity_align_mode:=hybrid 只对 lio:=small_point_lio 生效（…）⇒ 当前 lio:=fastlio：…`」。
* ⇒ 需要一个**只解析、不下传**的档：`auto`。它让两个默认同时成立：
  **默认路径（fastlio）逐字节不动**（有效值 off）＋ **SPL 路线默认对齐**（有效值 hybrid）。

#### 4.10.2 实现（"解析一次、到处使用"）

launch 里只有**一个**解析点（`_effective_gravity_align_mode` / `_GravityAlignEffectiveMode`），
下游六处一律只读**有效值**：

| 位置（`bringup_sim.launch.py`） | 改动 |
|---|---|
| launch 参数（`L2120`） | `_GRAVITY_ALIGN_MODES` 增加 `'auto'`；`DeclareLaunchArgument('gravity_align_mode')` 默认值 `'off'` → **`'auto'`**（description 写明 `auto = hybrid 当 lio==small_point_lio，否则 off`） |
| **唯一解析点**（`L826` / `L847`） | `_effective_gravity_align_mode(context)` + substitution `_GravityAlignEffectiveMode`：`auto` ⇒ `hybrid` / `off`；**显式值原样返回** |
| 默认档 SPL Node 的 `== 'off'` 条件（`L2680`） | 换成有效值 |
| 统一输出契约档 SPL Node 的 `!= 'off'` 条件（`L2699`）**及其 `gravity_align_mode` 参数**（`L2713`） | 换成有效值 |
| `lio_tf_adapter_spl_node` 的条件（`L3037`） | 换成有效值 |
| 组合检查 `GroupAction` 的条件（`L3536`）与 `_check_gravity_align_contract()` 内读的 `mode`（`L223`） | 换成有效值 |
| 收据（`L3520`） | `auto` 时 t=0 打一行 `[gravity_align] gravity_align_mode=auto ⇒ 有效值=…（…）`；**显式值不打**（⇒ 显式路径的 Action 集不变） |

**错误语义不变**：显式 `urdf|imu|hybrid` + 非 `small_point_lio` 仍然**逐字硬报错**
（`auto` 不替用户的明确请求兜底）；`auto` 在**所有** lio 上都不报错（25 格矩阵见 M §18.7）。
`auto` **不会**出现在任何节点参数/condition 里 —— 实测 `ros2 param get /small_point_lio
gravity_align_mode` = **`hybrid`**（M §18.4）。

#### 4.10.3 证据表

`off` 臂 = 显式 `gravity_align_mode:=off`（斜 odom）；`auto` 臂 = `lio:=small_point_lio` + 默认
`auto` ⇒ 有效值 **hybrid**（统一输出契约 + E1 + E1b `z_origin=ground`）。
数字全部来自本仓实测（`mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor`）：

| 指标 | **off**（斜 odom） | **auto ⇒ hybrid** | 出处 |
|---|---|---|---|
| local `/scan` 在带 `[0,2]` | **333 / 776**（42.9%） | **776 / 776（100%）**（历史 779.5/779.5；本轮 776/776，守卫 11 样本 772–783 全 100%） | M §17.8 / §18.4 |
| kept-y（带内波束的 y 区间） | **[0.268, 3.404] m，单侧 `+y`** | **[−0.636, 5.086] m，整盘**（本轮 [−0.635, 5.090]） | M §17.8 / §18.4 |
| local **lethal** | **288**（插件修复前口径 193） | **267**（修复前 288.5；另一轮 284–292；本轮 **277**） | M §17.8 / §10.4 / §13.8 / §18.4 |
| local **最近 lethal** | **0.755 m**（0.760） | **0.703 m**（0.769；本轮 **0.7215**） | M §17.8 / §10.4 / §18.4 |
| 前方 2.0×1.2 m lethal | **17**（21） | **28**（18；本轮 **21**） | M §17.8 / §10.4 / §18.4 |
| **地面残余倾角** | **30.115°** | **0.518°**（本轮 **0.442°**，n=5 帧 [0.419–0.532]） | M §10.4.1 / §18.4 |
| `odom→base_link` rpy | [30.06…30.20, −0.12…−0.18, −0.07…−0.21]° | **[0.30…0.43, −0.21…−0.38, −0.04…+0.05]°**（本轮 [−0.04, −0.25, −0.22]°；窗口末端 tf2_echo [−0.02…−0.12, −0.39, −0.26…−0.28]°） | M §10.4.1 / §18.4 |
| `h_meas`（E1b 地面互检） | —（off 档不跑互检） | **0.2616 m，Δ=0.0020 m ⇒ INFO**（另 0.2603 / 0.2615） | 本轮 / M §13.9 |
| ms/帧（窗口） | 15.43 [14.30–15.80] | **13.90 [12.80–15.15]**（同轮 A/B；本轮单跑 16.15 不可跨轮比） | M §10.5 / §18.4 |
| 默认路径（off）输出 | — | **141/141 行逐位相同**（同一输入流 bag 重放） | M §10.6 |

#### 4.10.4 生效条件与**收益边界**

1. **只对 `lio:=small_point_lio` 生效**；其它槽位（fastlio / pointlio / none / cartographer）
   有效值恒 `off`（launch 默认 `lio:=fastlio` 因此仍是**逐字节现状**）。
2. **只在 `gravity_align_z_origin:=ground`（默认）时**给出上表的局部收益。`sensor` 是旧行为
   （odom 原点在传感器壳上、地面不在 z=0）⇒ 会把 nav2 局部代价图打空（M §10.7），只用于复现 A/B。
3. **收益是局部/odom 的**：`off` 的 odom 斜 30° ⇒ 局部 scan 盘面跨过 `min_obstacle_height` ⇒
   只有 43% 波束进带、kept-y 单侧；对齐后 100% 进带。**全局/map 层两档逐值相同**
   （global lethal / inscribed / free = **3713 / 16886 / 10176**，M §10.4.1；本轮 3725/16875/10175）
   —— 全局层的问题是**另一个独立的静默缺陷**（`global_obstacle:=scan` 的 z 闸，§15，**opt-in 修复**，
   不在本节范围）；Livox 插件 `cloud_frame=sensor` 的传感器系偏置是**第三个**独立缺陷（§13.4）。
4. **默认路径零改动**是本节的主验收（`lio:=fastlio`）：相对改动前，静态只有**两处 launch 声明
   元数据**（`gravity_align_mode` 的默认值/choices/description、`gravity_align_z_origin` 的
   description 文案）与**一行 t=0 收据**不同；**执行集（Node 集合 / 参数 / opaque / 顺序）逐字段相同**，
   `robot_description` 的 xacro 命令子树 md5 相同（M §18.3）。

#### 4.10.5 回滚（逐字节）

`gravity_align_mode:=off` ⇒ **与今天完全一致**：

* `lio:=small_point_lio` + 显式 `off`：执行集 diff（visited，剔除上面两处 launch 声明元数据）
  **0 新增 / 0 删除 / 0 改动**，process 执行集 md5 与原 launch **相同**；运行期
  `ros2 node list`（11 个节点）+ 每节点 `ros2 param dump`（11 份）**逐字节相同**（M §18.5）；
* `lio:=fastlio` 默认：有效值已是 `off`，同理（M §18.3；12 节点 / 12 份 param dump 逐字节相同）；
* 二进制层的同一性另有 §10.6 的 bag 重放 141/141 行背书（本节**没有**改任何二进制）；
* 只想回滚默认值：显式传 `gravity_align_mode:=off`，或把 `DeclareLaunchArgument` 的
  `default_value` 改回 `'off'`（`auto` 仍可选，不影响其它参数）。

#### 4.10.6 本轮验证（a–d）与**已知边界**

| # | 场景 | 结果 |
|---|---|---|
| (a) | 默认（`lio:=fastlio`，不传 mode） | 有效值 **off**；执行集与改动前**逐字段相同**（process md5 相同、12 节点 / 12 份 `param dump` 逐字节相同、topics 31 / services 84 相同、`robot_description` 子树 md5 相同）；唯一新增 = 一行 `auto ⇒ 有效值=off（lio=fastlio；E1 补丁只存在于 small_point_lio）` 收据 |
| (b) | `lio:=small_point_lio`（不传 mode） | 有效值 **hybrid**；adapter 启动、SPL 节点回读 `gravity_align_mode=hybrid` + `gravity_align_rpy=[−30,0,0]` + `gravity_align_z_origin=ground` + `gravity_align_h=0.25952717`；nav 1 跑：in-band **776/776**、lethal **277**、最近 **0.7215 m**、ahead **21**、倾角 **0.442°**、`h_meas` **INFO**（M §18.4） |
| (c) | `lio:=small_point_lio gravity_align_mode:=off` | 与今天的 off 路径**逐字节相同**（执行集 0/0/0 + 运行期 11 节点/11 份 param dump/topics/services 全等，M §18.5） |
| (d) | `lio:=fastlio gravity_align_mode:=hybrid` | **仍然硬报错**（t=0、`exit 1`、逐字文案不变；`auto` 不触发此错误，M §18.6） |

**已知边界（本轮没验的，别当成已验）**：

1. **只实测 `robot:=robot11` + `robot11_mount:=sensor`**（本仓唯一 30° 斜装槽位）；默认模型 /
   `hzmirm` / `robot11_mount:=plugin|urdf` 的 off-vs-hybrid **没有**在本轮实跑。
2. **`hzmirm` 的 `turret_yaw/pitch ≠ 0`** 只做了 launch 求值（`h = 0.6 + 0.2·cos(pitch)`），
   没跑 Gazebo 验证 TF/scan（M §10.9 第 5 条）。
3. **E1b 的 3–5 cm WARN 带**没有构造出真实数据（sim 里直接落到 INFO 或 ERROR，M §10.9 第 6 条）。
4. **`lio:=small_point_lio` + 默认模型槽位（`R_bl=I` 档）的 SIGSEGV** 是**独立**缺陷
   （M §16.9 / §16.4 坑 2），与本节无关；修好之前，`auto ⇒ hybrid` 的收益只在
   `R_bl ≠ I` 的模型（robot11 / hzmirm）上可见。
5. 上表的 off 臂数字来自**不同轮次**（M §10.4 的 3 跑/档、§13.8 的 1 跑/档、§17.8 的守卫 run），
   不是同一批次重新配对；量级/方向一致，但**不许**逐格当同一次实验比较。
   本轮的 auto 臂是**单跑**（n=1），只作"默认值真的落到 hybrid"的存在性证据，不作性能/精度 A/B。

---

## §4.11 `odom_level`：重力对齐做成**帧图**的事（2026-10-21，默认路径）

> **本节只新增**（append-only）：§4.1–§4.10 / §13 的正文一个字都没改。本节是**用户决定**：
> "不要 E1 吧，加个 tf"。它**取代**下面三处在**默认路径**上的效力（回滚档 `level_frame:=odom`
> 时三处逐字生效，保留为历史/实验臂）：
> * **§4.7**（odom 的 z 原点约定，E1b）—— z 原点现在由静态边给，公式没变（`z_origin_state_z` 的
>   推导被原样复用），只是执行者从估计器换成 `odom_level_tf`；
> * **§4.8**（世界地面校准是前置条件）—— **仍然成立**（`h` 的前提），默认档同样依赖它；
> * **§4.10**（`gravity_align_mode:=auto` ⇒ hybrid）—— 默认档 `auto` **解析成 off**；
>   只有回滚档保留 §4.10 的规则。
>
> 实测/迁移清单/回归矩阵见 `docs/frame_contract_v3_measurements.md` §29。

### 4.11.1 决定与帧树

```yaml
level_frame: odom_level      # odom_level（默认）| odom（回滚到 §4.7/§4.10 的行为）
```

```
map ──(重定位器: gicp|amcl|beluga|slam_toolbox|cartographer…  可跳变)──► odom
                                                                         │  ★ 新增：静态边（latched，唯一发布者 odom_level_tf）
                                                                         ▼
                                                                     odom_level ──(lio_tf_adapter)──► base_link ──(URDF)──► livox_frame
                                                                                                          │                └► imu_link
                                                                                                          └──(fake_vel_transform)──► base_link_fake
```

| 帧 | 定义 | 所有者（唯一） |
|---|---|---|
| `odom` | **LIO 的原始世界系**。IMU 斜装 ⇒ 它就是斜的（robot11 sensor 档 = 车体水平时 z 轴相对重力斜 **30°**）；z 原点 = 初始化时刻的状态原点（IMU body）。**没有任何节点会把重力对齐写进它** | 估计器（发 `/Odometry` + 不参与 TF 时由 adapter 发） |
| **`odom_level`** | `odom` 的**常量**变换：z 轴朝上（重力对齐）、原点在**初始化时刻的地面**（`z=0 ≈ 地面`；robot11 的 `base_link z ≈ +0.1025`）、x/y 取"IMU 在地面的垂足" | **`odom_level_tf`**（`rm_nav_bringup/scripts/odom_level_tf.py`）**独占**：它往 `/tf_static` 发**一条** latched 变换 |
| `base_link` | 车体 | `lio_tf_adapter`（父帧 = `odom_level`） |

**数值（robot11 + `robot11_mount:=sensor`，全部由 URDF + 几何算出，禁手填）**：

```text
R_mount = R(base_link←livox_frame) = Rz(0)·Ry(0)·Rx(−30°)            # launch 从这次 spawn 的 xacro 解析
h       = 0.259527170 m                                              # base_link 离地 0.102499 + livox_z 0.157028
t_IL    = extrinsic_T = (0, 0, 0.05) m                               # livox 原点在 IMU body 系
h_imu   = h − (R_mount·t_IL)_z = 0.259527170 − 0.05·cos30° = 0.216225900 m
T_odom←odom_level :  R = R_mountᵀ = Rx(+30°)
                     t = −R_mountᵀ·(0,0,h_imu) = (0, +0.108112950, −0.187257122) m
⇒ 期望：地面 z(odom_level) = 0、livox z = h、base_link z = +0.102499（实测见 M §29）
```

**一般式**（三个"输入事实"，`odom_level_tf` 的参数 `input_world` / `input_z` / `output_z`）：

```text
R_level←odom = R_mount   当输入世界**未**重力对齐（input_world=tilted：SPL-off / fastlio）
             = I         当输入世界**已经**重力对齐（input_world=level：pointlio 的 Set_init /
                         E1 实验臂 / none / cartographer）—— **绝不能再转一次**
t_level      = (0,0,h_imu)  当 input_z=sensor 且 output_z=ground（默认）
             = (0,0,0)      其它（E1 已把地面放到 0 / 明确要 sensor 档）
发布值 = (R_level←odomᵀ, −R_level←odomᵀ·t_level)      # 即 T_odom←odom_level，是上面那个的**逆**
```

### 4.11.2 不变量（I1/I5 在默认档上的新落点）

| # | 不变量 | 判据（可执行） |
|---|---|---|
| **I8** | **`odom→odom_level` 是静态边**：值在整跑里**逐位不变**（不随 LIO/重定位/时间变） | `/tf_static` 上两条采样（窗口首末）逐字段相同；`odom_level_tf` **不发** `/tf`（只发 `/tf_static`） |
| **I9** | **唯一发布者**：全栈只有一个节点声明这条边 | `ros2 topic info /tf_static -v` 的发布者 + 逐消息解析 ⇒ 该边只出现 1 次；`grep -rn "odom_level" src/` 只有 `odom_level_tf.py` 写 `/tf_static` |
| **I10** | **`odom_level` 是"z 朝上 + 原点在地面"** | 点云在 `odom_level` 里拟合地面：`floor_z ∈ 0 ± 0.006 m`、`tilt < 0.7°`；静止时 `base_link z ≈ +0.102499`（robot11） |
| **I11** | **`odom_level→base_link` 只含车体真姿态**（roll/pitch ≈ 0，yaw 任意） | 静止窗口 `tf2_echo odom_level base_link` 的 roll/pitch **< 0.5°** |
| **I12** | **无环**：`odom_level` 不是"第二个父"；`base_link` 只有一个父 | `base_link` 的父帧集合 = {`odom_level`}；`odom_level` 的父帧集合 = {`odom`}；`map→odom` 仍是重定位器的边 |
| **I13** | **`odom` 上没有重力假设**：消费者不得把 `odom` 当水平帧 | 局部代价图 / LIO rviz 固定帧 / adapter 父帧都用 `odom_level`；`odom` 只出现在 `map→odom`（重定位）与"原始世界"语义里 |

**E1 的语义在新的位置**：§4.5 的验收线（残余地面倾角 < 0.7°）与 §4.7 的 z 原点约定**没有变**，
只是"谁来做"从**估计器初始化**（改 `kf.x.rotation/position` + 初始地图）换成
**一条静态变换**。数学上等价（同一组 `R_ws` / `h_imu` 公式），工程上分成两件事：
**标定**（静态边，确定性）与**估计**（LIO，不动它）。

### 4.11.3 参数与默认值

| 参数 | 默认 | 说明 |
|---|---|---|
| `level_frame` | **`odom_level`** | 本节的**总开关**。`odom` = 回滚：逐字节回到 §4.7/§4.10 的行为（不发布静态边、`auto ⇒ hybrid`、adapter 父帧 = `odom`、局部代价图 = `odom`） |
| `gravity_align_mode` | `auto` | **默认档解析成 `off`**（E1 不在默认路径上）。显式 `urdf|imu|hybrid` 仍打 E1 补丁（**实验臂**，A/B 用）⇒ 那时静态边退化成**恒等**（`input_world=level, input_z=ground`） |
| `gravity_align_rpy` / `gravity_align_h` | 从 URDF 自动算 | **同名同义**，现在同时喂两个地方：E1 实验臂（`small_point_lio` 的参数）与静态边（`odom_level_tf`）。**仍然禁止手填** |
| `gravity_align_z_origin` | `ground` | 默认档：`odom_level` 的 z 原点（`ground` ⇒ 地面在 0；`sensor` ⇒ 原点留在 IMU，**A/B 复现旧行为用**，nav2 局部高度带会掉出去） |

`level_frame:=odom_level` 时被**换成** `odom_level` 的三个消费者（其余一个不动，见 M §29）：

1. `local_costmap.local_costmap.global_frame`（nav2 覆盖层 `rm_nav_bringup/config/nav2_params_sim_level_frame.yaml`，
   深合并，回滚档整层不命中）；
2. `behavior_server.global_frame`（它吃的就是局部代价图）；
3. LIO rviz 的固定帧（`rviz2 -f <level_frame>`，`.rviz` 文件不改）。

**保持不变**（有意）：`global_costmap.global_frame = map`（**stvl 全局 profile 与
`global_obstacle:=scan` 的 −1.0 覆盖层逐字节不变**）、`/scan` 的 `frame_id = base_link`、
p2l 的 `target_frame: base_link`、linefit 的 `gravity_aligned_frame: base_link`、
重定位槽的 `odom_frame_id: odom`（`map→odom` 仍是重定位器的边，契约 I3）、
`fake_vel_transform` 的 `base_link→base_link_fake`。

### 4.11.4 互检（loud, never silent；**只作诊断，不改发布值**）

| 检查 | 数据 | 阈值 | 级别 | 行为 |
|---|---|---|---|---|
| `gravity_align_h_check` | 一帧 `/cloud_registered` 拟合地面 ⇒ `h_meas`（传感器到地面垂距）vs `h(URDF)` | <0.03 / 0.03–0.05 / ≥0.05 m | INFO / WARN / ERROR | **定基仍用 URDF 值**（确定性）—— 与 §4.7 第 4 条逐字一致 |
| URDF↔IMU `Δ` | 静止 200 帧 `/imu/data` 均值 ⇒ `R_imu` vs `R_mount` | <2° / 2–5° / ≥5° | INFO / WARN / ERROR | **不会**改用 IMU 值（见下） |

**⚠️ 与 E1 `hybrid` 档有意不同**：`hybrid` 在 Δ≥2° 时**改用 IMU**（把实测提升成"第二个所有者"）；
静态边**不这么做** —— 它的值是**标定事实**，唯一所有者是 URDF，实测只负责**响亮报错**。
需要"IMU 救援"的场景请走实验臂：`gravity_align_mode:=hybrid`（+ `level_frame:=odom`），
那条路径的语义与 §4.3/§4.10 **逐字不变**。

### 4.11.5 回滚

```text
level_frame:=odom      ⇒ 行为与 2026-10-21 之前逐字节相同（§4.7/§4.10 全部生效）
```

逐项证据（M §29）：节点集合相同、E1 有效值回到 `hybrid`、`odom→base_link` 仍由 LIO 自己发
（adapter 不启动）、局部代价图 `global_frame=odom`、没有 `odom_level` 这个帧、合并后的 nav2
参数文件与改动前逐字节相同。

### 4.11.6 边界（明确不在本节范围）

1. **`lio:=cartographer` / `lio:=none`**：静态边做成**恒等别名**（`odom_level ≡ odom`）——
   只为了让局部代价图/消费者的**帧名**统一，**不对**这两个槽位的世界系做任何重力假设
   （cartographer 自己发 `odom→base_link`，`odom_level` 只是它的别名）。
2. **`localization:=''`（LIO 当绝对定位的回退用法）**：静态桥 `map≡odom` 把 `map` 钉在**原始世界**上
   ⇒ 全局代价图（`map`）不跟着摆正，只有局部（`odom_level`）是正的。launch 在 t=0 **响亮 WARN**
   （不是回归：改动前这条回退路径同样是"斜 odom + 正 map 资产"）。正式用法 = 给重定位槽。
3. **`lio:=fastlio` / `pointlio`**：静态边照用（`tilted` / `level` 两档），但它们的**初始化门**
   （FAST-LIO 的 `IMU_init`、Point-LIO 的 `Set_init`）不在本契约内；本节只承诺"给这条边一个
   正确的值"，不承诺这两个估计器自己的残余倾角（实测见 M §29）。
4. **`hzmirm` 的云台转动**（`turret_pitch_deg ≠ 0`）：静态边的 `R_mount` 是**默认姿态**下的安装值，
   云台一动 `livox_frame` 相对 `base_link` 就变 ⇒ 这条边不再准确（与改动前的 E1/adapter 同一个
   局限，未新增）。
5. **`mapping` 模式**：静态边照发（`odom_level` 存在），但建图资产/`cloud_accumulator` 的
   `target_frame:=odom` 选项现在指**原始世界**（它本来就是"平滑帧"语义，不是"水平帧"，
   落盘前有一次 `map←odom` 刚体变换 ⇒ 几何不变）。

### 4.11.7 局部/全局帧的最终归属（2026-10-21 用户确认）

| 消费者 | 帧 | 为什么 |
|---|---|---|
| **局部代价图**（`local_costmap.global_frame`） | **`odom_level`** | 这一条是**整张图**的帧，不是某一层的开关 ⇒ **所有**局部障碍层（`obstacle_layer` 的 2D scan、`stvl_layer`、`cloud` / `voxel` 变体、`inflation`）都在 `odom_level` 里 ⇒ STVL 的 3D 点的"方向"与点云那个"正"的帧一致（用户原话："STVL 的方向要和点云'正'的帧一致"）。覆盖层 `nav2_params_sim_level_frame.yaml` 只写这一个叶子键 ⇒ 与 `local_obstacle:=scan\|cloud\|both\|stvl\|stvl_both` **全部兼容**（换层不换帧）。 |
| **全局代价图**（`global_costmap.global_frame`） | **`map`**（不变） | `map` 是重定位器 `map→odom` 之上的**全局**帧；全局层的 z 闸问题由 `global_obstacle:=scan` 的 −1.0 覆盖层处理（§15/§25.6，本轮**一个字节都没动**）。 |
| `/scan` 的 `frame_id` | **`base_link`**（不变） | 契约要求；p2l `target_frame: base_link` 保持。 |
| linefit / p2l / 自击掩膜 / 近地剔除 | **`base_link` / `livox_frame`**（不变） | 它们是**自车/传感器系**的量（`sensor_height`、"离传感器的高度"、自击 AABB）⇒ 见 §4.11.8 的两帧语义。 |

**`map` 是不是水平的？（实测，2026-10-21）** `mode:=nav`（`localization:=gicp`）下
`map→base_link` 的 rpy ≈ **[−0.29, −3.05, +0.30]°**（7 跑样本一致）⇒ `map` 相对重力**差 ~3°**
（这是**地图资产 + GICP 对齐**的既有性质，不是本轮引入的；`odom_level→base_link` 的
roll/pitch 是 **<0.5°**）。⇒ 全局层的"水平"只能和 `map` 一样好；把**障碍高度语义**放在
`odom_level`（或 §4.11 之后的 `map`，前提是重定位把倾角吸收干净）是正确做法，但**不要**指望
`map` 精确水平。`mode:=mapping` 时 nav2 **根本不启动**（launch 里 `start_navigation2` 只在
`nav`/`slam_nav` 下执行）⇒ "全局代价图在斜的 map 里"这个情形**不存在**。

**高度带的数值（本轮不动）**：局部 2D scan 层 `min_obstacle_height = scan_band_min`（默认
**−0.05**，§26）+ `max = 2.0`；STVL 的 3D 带保持原值。**注意**：`odom_level` 的原点现在在
地面上 ⇒ 高度带第一次有了"离地高度"的物理语义 ⇒ COD 那种 `[0.1, 1.0]`（离地 10 cm–1 m）的
写法**现在是可表达的**（以前在斜 `odom` 里不可表达）。本轮**不改**它（只记录）。

### 4.11.8 两帧语义（写死，避免以后再混）

| 帧类 | 例子 | 能用来做什么 | **不能**用来做什么 |
|---|---|---|---|
| **车体/传感器系** | `base_link`、`base_link_fake`、`livox_frame`、`imu_link` | 自击裁剪、传感器量程/盲区、`sensor_height`、"离传感器多高"、点云→2D 投影的目标帧 | **障碍物高度语义** —— `base_link` 在坡/台阶上会**跟着车体倾**，同一高度的障碍在它里面不是常量 |
| **重力对齐世界系** | **`odom_level`**、`map` | 高度带（`min/max_obstacle_height`）、地面/台阶/近地剔除、3D 点的"上"方向、建图资产的 z | 自车几何（车体系才有意义） |

> **2026-10-27 追加（measurements §37，append-only）**：上表"`base_link` **不能**用来做障碍物高度语义"
> 这一条**部分被实测推翻** —— 在**均匀坡 + 车体与坡面平行**时，`base_link` 的 z **就是**"垂直于坡面的
> 净空"（4 个 x 误差 **0.0 cm**、拟合斜率 **0.0000**；`odom_level` 的固定带在同一工况是海拔，误差 =
> `x·sin15°`）。**但**：①车体不平行于地形时（平地看坡、坡顶/台阶）`base_link` 与海拔法**逐位同错**；
> ②它不是重力竖直帧 ⇒ **帧仍用 `odom_level`**；③"带"必须**地形相对**（`ground:=linefit` 的
> `dz = z − 局部地面`），不能是 `odom_level` 里的固定 z。详见 measurements **§37.5–§37.7**。
> 原文一个字节未改，只加这条指针。

---

## §4.12 雷达安装约定：**按实车**（2026-10-21）

> **本节只新增**（append-only）。用户决定："按照实车来吧也好理解"。
> 它**取代** §4.11 之前 sim 侧的"倾角记在 URDF 关节上"这一约定（`robot11_mount` 的
> plugin / urdf / sensor 三档），但**不改**任何物理几何、`h`、`h_imu`、也不改 §4.11 的静态边数值。

### 4.12.1 实车是什么样

1. Livox 驱动器的 `extrinsic_parameter`（roll 180 / yaw 90）**只旋点云**：它把点从传感器系
   旋进机器人系；驱动器的 **IMU 通路用另一套 `extrinsic_global`（单位阵）**，IMU 数据不被它旋
   （依据：`pub_handler.cpp:131-150`）。
2. 因此实车的 TF `base_link→livox_frame` 的 **rpy = 0**（只有平移），而**物理上的倾斜**在
   IMU 那条链上（`livox_frame→imu_link` 带着安装倾角 ⇒ 加速度计真的感受到它）。

### 4.12.2 sim 怎么复现（`robot:=robot11`）

| 面 | 改前（2026-10-20 及以前） | **改后（= 实车约定）** |
|---|---|---|
| `base_link→livox_frame` | `rpy = Rx(−30°)`、`xyz = (0.000562, 0.130916, 0.157028)` | **`rpy = 0`**、`xyz` **逐位不变** |
| `livox_frame→imu_link` | `rpy = 0`、`xyz = (0, 0, −0.05)` | **`rpy = Rx(−30°)`**、`xyz = Rx(−30°)·(0,0,−0.05) = (0, −0.025, −0.043301)` |
| ⤷ 派生：`base_link→imu_link` | `rpy = Rx(−30°)`、`xyz = (0.000562, 0.105916, 0.113727)` | **完全相同**（物理几何一个字节没动） |
| 插件 `<tilt_rpy>` | 单位阵（倾角由 link 姿态带斜） | **`Rx(−30°)`**（射线真的斜） |
| 点云表达 | 传感器系（`frame_id=livox_frame`，而该帧自己斜 30°） | **水平**的 `livox_frame`（与实车驱动器旋完点云后的语义一致） |
| LIO `extrinsic_R` | `I`（IMU 与雷达同姿态） | **`Rx(+30°)`**（水平点云 → 斜的 IMU 系；由 launch 从 URDF 解析注入） |
| LIO `extrinsic_T` | `[0, 0, 0.05]` | **`[0, 0, 0.05]`（不变）** |
| `gravity_align_rpy`（= E1 先验 / §4.11 的 `R_grav`）来源 | `base_link→livox_frame` | **`base_link→imu_link`**（值同样是 `[−30, 0, 0]°` ⇒ **§4.11 静态边的数值不变**） |
| `robot11_mount` | `plugin｜urdf｜sensor` | **只剩 `sensor`**（另两个实验臂删除，传它们**硬报错**） |

**三方一致性（写死在这里，改任何一处都要三处一起改）**：

```text
驱动器/插件把点云旋进机器人系的旋转  R_pub
   =  TF 的 R(base_link←livox_frame)      ← 现在 = I（实车 TF 的 rpy=0）
   ∘  TF 的 R(livox_frame←imu_link)       ← 现在 = Rx(−30°)（物理倾角）
   ∘  LIO  extrinsic_R 的逆               ← 现在 = Rx(+30°)，即把"水平点云"搬回"斜的 IMU 系"
⇒ 点云在 livox_frame 里是**水平**的（地面法向 ≈ +z），IMU 在 `imu_link` 里是**斜**的，
  估计器靠 extrinsic_R 把两者接上；`odom` 因此仍然是 **IMU 系的原始世界**（§4.11 的前提不变）。
```

**为什么插件一行 C++ 都没改**：`livox_points_plugin.cpp` 的出点公式本来就是
`point = (range+minDist)·(mount_rot·ray·x̂)` —— 它把点表达在 `header.frame_id` 声称的那个帧里，
而射线方向是 `offset.Rot()·mount_rot·ray·x̂`（父 link 系）。所以只要
**关节 rpy = 0（`offset.Rot()=I`）+ `<tilt_rpy>` = 倾角**，射线就真的斜、点就落在水平的
`livox_frame` 里 —— 正是"驱动器只旋点云"的语义。改前那 30° 的错位是**关节**斜了、点却按
`frame_id` 除以斜帧造成的（缺陷③）。真正要改的只有 URDF 的两条链 + LIO 外参（本节的表）。

### 4.12.3 这条约定影响了什么（逐项）

| 项 | 影响 |
|---|---|
| `h = 0.259527 m`（livox 离地） | **不变**（livox 原点的平移没动） |
| `h_imu = 0.216226 m`、§4.11 的静态边 `R_grav` / `t` | **不变**（IMU 的物理位姿没动） |
| linefit | `robot11_mount:=sensor` 的 `gravity_aligned_frame: base_link` **覆盖文件已删除**：点云本来就水平 ⇒ 回到槽位那份 `segmentation_sim_robot11.yaml`（`gravity_aligned_frame: ""` + `sensor_height: 0.2595`） |
| p2l | 仍用 `target_frame: base_link`（契约要求 `/scan` 在 `base_link`；点云水平后这条只是"换帧"，语义不变） |
| `lio_tf_adapter` | **不用改**：它的 `T(imu←base_link)` 与改动前逐位相同（IMU 的物理位姿没动） |
| 默认模型 / `hzmirm` | **零改动**：它们的 IMU 链本来就没有倾角（解析出来的 `extrinsic_R` = I、`gravity_rpy` = 0） |
| 画出来的雷达 mesh | 现在**水平**（物理上斜）—— 纯显示差异，与实车的"TF 是平的"一致 |
| `robot11_mount:=plugin｜urdf` | **删除**（硬报错）。它们是"倾角记在哪"的实验臂，新约定下没有意义 |

**验收（实测见 measurements §29）**：`base_link→livox_frame` rpy = 0；`base_link→imu_link`
rpy = −30°；原始云（`livox_frame`）里地面法向 vs +z ≈ 0°；`odom→base_link`（`gravity_align_mode:=off`）
仍 ≈ +30°（odom = IMU 原始世界）；nav 局部 in-band 100%（叠加 §4.11 的 `odom_level` 之后）。

### 4.12.4 安装角的单一来源（2026-10-21 收口）：上游 URDF → 生成器 → 生成物 xacro

> 用户决定："那就是我们自己生成的xacro改呗，然后收口"。本节把"倾角到底写在哪"收成**一条链**，
> 不再有第二份冻结常数。

| 环节 | 位置 | 角色 |
|---|---|---|
| 物理事实 | `src/rm_nav_bringup/urdf/upstream/robot11.urdf` 的 `joint name="body_to_livox"` → `<origin rpy>` | **唯一来源**（只读、不参与运行） |
| 生成 | `tools/scripts/regress/robot11_make_sim_xacro.py`（`--livox-tilt-rpy` 仅 scratch 覆盖） | 读上游 rpy，烘进生成物 |
| 生成物 | `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` 第 12 行的 `<xacro:arg name="livox_tilt_rpy" default="..."/>` | 默认值 = 上游值；插件 `<tilt_rpy>` 与 `imu_joint` 的 rpy/xyz 都由它算 |
| 运行期 | `bringup_sim.launch.py` 的 `_parsed_livox_mount()`（launch 第 285 行的 `_robot11_tilt_rpy_from_xacro()` 只服务旧表回退） | 展开**这次真的要 spawn 的** xacro → `gravity_align_rpy` / `h` / `extrinsic_R` / adapter / `odom_level` 静态边 |

- **删除了什么**：`bringup_sim.launch.py` 的 `_LIVOX_TILT_RPY_ROLL`，以及 `_robot_xacro_args()`
  里 `('livox_tilt_rpy', ...)` 那一条。现在 launch **不传**这个 xacro arg（xacro 用自己的默认值）。
- **默认路径逐字节不变**：生成物 xacro 的 md5 不变（`c5e12d36…` / 56140 B），9 个
  (robot, mount) 组合的 `robot_description` 输出 md5 不变，`h` 不变（robot11 = 0.25952717）；
  `--show-args` 421 行逐字相同（105 个参数）。实测表见 measurements §31.2。
- **覆盖方式（只用于 scratch，别提交到默认路径）**：
  1. `python3 tools/scripts/regress/robot11_make_sim_xacro.py --livox-tilt-rpy "r p y" --out /tmp/x.xacro`
     （弧度，与 xacro 同格式；`r p y` = URDF 的 `Rz·Ry·Rx` 约定）；
  2. 或直接 `xacro ... livox_tilt_rpy:="r p y"`（生成物的 arg 仍可被 xacro 覆盖；launch 不再传）。
  **不要**再往 launch 里加一个常数/开关：那会让"改上游 URDF"不再自动生效。
- **验证方式（一次性、可复算）**：改 scratch 上游副本的 `body_to_livox.rpy`（如 30°→20°、
  roll→pitch）→ 重跑生成器 → `_parsed_livox_mount()` 的 `gravity_rpy_deg`、`h`、`extrinsic_R`、
  `odom_level_tf` 的静态边全部跟着变；仓库里的上游文件 md5 不变。实测见 measurements §31.3。

### 4.12.5 LIO 外参的**键名契约**与运行期断言（2026-10-27）

> **本节只新增**（append-only）。它补的是 §4.12 表里那一行「LIO `extrinsic_R`」的**落地方式**：
> 值由 URDF 解析是对的，但**键名每个后端不一样** —— 2026-10-27 之前 launch 给三个槽位都传顶层
> `extrinsic_T/R`，只有 SPL 读顶层；fastlio/pointlio 读 `mapping.*` ⇒ robot11 的 +30° **静默丢失**，
> 运行期实测 `mapping.extrinsic_R = I`（measurements §34.3#4 / §35.3）。

**键表（唯一真源 = `bringup_sim.launch.py` 的 `_LIO_EXTRINSIC_KEYS`；launch 参数字典与断言
节点都从它派生）**：

| LIO 槽位（`lio:=`） | **节点实际读取的键** | 读取处（file:line） | launch 写键的方式 | 运行期断言节点 |
|---|---|---|---|---|
| `fastlio` | `mapping.extrinsic_T` / `mapping.extrinsic_R` | `src/rm_localization/FAST_LIO/src/laserMapping.cpp:834–835`（declare）、`:870–871`（get） | `_lio_extrinsic_params('fastlio')` | `lio_extrinsic_receipt.py` → `/laser_mapping` |
| `pointlio` | `mapping.extrinsic_T` / `mapping.extrinsic_R` | `src/rm_localization/point_lio/src/parameters.cpp:179–183` | `_lio_extrinsic_params('pointlio')` | `lio_extrinsic_receipt.py` → `/laserMapping` |
| `small_point_lio` | **顶层** `extrinsic_T` / `extrinsic_R` | `src/rm_localization/small_point_lio/src/small_point_lio/parameters.cpp:110–116` | `_lio_extrinsic_params('small_point_lio')` | `lio_extrinsic_receipt.py` → `/small_point_lio` |

- **值**永远是 `T(imu←livox)`：`t_IL = [0, 0, 0.05] m`、`R_IL = R(base←imu)ᵀ`（robot11 =
  `Rx(+30°)`；默认模型 / hzmirm = `I`）。三个后端都用同一公式
  `p_imu = R_IL·p_lidar + t_IL`（fastlio `laserMapping.cpp:173/185`、pointlio
  `Estimator.cpp:381–382`、SPL `small_point_lio.cpp` 的出点公式）⇒ 同一个
  `(R_IL, t_IL)` 三个后端**同义**，只有键名不同。
- **死键**：另一个命名空间（fastlio/pointlio 的顶层 `extrinsic_*`；SPL 的 `mapping.extrinsic_*`）
  一律**不许有值/不许被声明**。断言节点用 `list_parameters` 查它们；出现 ⇒ **ERROR**。
- **yaml 默认值**：`point_lio/config/pointlio_mid360_sim.yaml:53`（T）/`:62–64`（R）的
  `mapping.extrinsic_R` 现在是**真的** `Rx(+30°)`（不再是 `I`），`extrinsic_T=[0,0,0.05]` 不变。
  为什么两边都写：launch **覆盖**它（URDF 是唯一真源，默认模型/hzmirm 在 launch 下仍是 `I`），
  但**独立跑这个 yaml**（不经 launch）时没有 URDF 解析 ⇒ 文件里的默认值必须也是真值，否则
  独立使用仍然丢 30°。
- **运行期断言（不是 launch 期自检）**：`src/rm_nav_bringup/scripts/lio_extrinsic_receipt.py`
  是一个**一次性** rclpy 节点，launch 在每个 LIO 的 Node 后面各起一个（三个槽位互斥）：
  1. 等目标节点的 `get_parameters` 服务就绪（`timeout_s=90`）；
  2. 用 `get_parameters` 读回**该后端自己的**两个键（必须一次只读这两个：rclpy 的 parameter
     service 遇到未声明名会清空整条 response，混入死键会假失败）；再用 `list_parameters`
     查另一个命名空间有没有**死键**（只读名字，不触发 rclcpp 的
     `Failed to get parameters` 假 WARN）；
  3. 与 launch 从 URDF 解析出的 expected_T/R 逐元素比对（`tol=1e-9`）；
  4. 打一行收据 `[lio_extrinsic_receipt] OK/ERROR …`；不一致 / 键缺失 / 类型错 / 死键 /
     超时 ⇒ **ERROR + exit 1**（`strict:=True`）。
  **故意不自动收栈**：宁可让用户看到红色 ERROR + `process has died`，也不要因一次误判
  （慢机器上的 90 s 超时）把整栈收掉。t=0 另有一条 `[lio_extrinsic]` 收据（`LogInfo`，与
  `[gravity]`/`[odom_level]` 同一批），把"这个后端读哪两个键、expected 多少、谁断言"写死。
- **默认路径的有效参数不变**（`gravity:=tf` + 默认模型）：
  `lio:=fastlio` / `lio:=small_point_lio` 的 `mapping.extrinsic_R` / `extrinsic_R` 都仍是 `I`、
  `T` 仍是 `[0,0,0.05]`（与非 robot11 槽位的 yaml 默认值逐位相同）；变化只发生在
  **robot11**（现在真的拿到 `Rx(+30°)`，以前是 `I`）与**键名**（顶层 → `mapping.*`）。
  实测见 measurements §35.3 / §35.7。

---

## §4.14 `gravity`：重力/水平帧参数组收口成**一个开关**（2026-10-21，默认路径）

> **本节只新增**（append-only）：§4.1–§4.13 的正文一个字都没改。用户决定（原话）：
> "123都可以感觉raw也可以留着可以学cod以baselink为基准"。
> 本节**取代** §4.11.3 的参数表、§4.13.2 的参数表（`lio_init_settle_s` / `lio_init_map_extrinsic`
> / `z_origin_source` 三项）在**用户可见参数**层面的效力；§4.11 的帧树/静态边数值/消费者归属与
> §4.13 的 z 锚点结论**全部继续成立**（它们现在是 `gravity:=tf` 的实现细节）。
> 实测/迁移/诚实清单见 `docs/frame_contract_v3_measurements.md` §33。

### 4.14.1 决定：4 档映射

```yaml
gravity: tf        # 默认 | base | lio | raw —— 重力/水平帧的**唯一**用户开关
```

| `gravity` | `odom` | 局部代价图水平帧 | 估计器 E1 | z 原点 | 障碍过滤 |
|---|---|---|---|---|---|
| **`tf`（默认）** | **原始**（IMU） | **`odom_level`**（静态边 on，唯一发布者 `odom_level_tf`） | **off** | 地面（`z_origin_source=urdf`，`t_level=h_imu`） | 今天：`local_obstacle` 的 2D scan / stvl |
| **`base`** | **原始**（IMU） | **`odom`**（**没有** `odom_level` 边） | off | n/a（不做帧级 z 原点） | **强制逐点层**（`cloud` 或 `stvl`）；显式 `local_obstacle:=scan` ⇒ **t=0 硬报错** |
| **`lio`** | **水平**（E1 hybrid） | `odom` | **hybrid**（**只对 `lio:=small_point_lio`**，其它 LIO 硬报错） | 地面（E1b `ground`） | 今天（`local_obstacle`） |
| **`raw`** | **原始**（IMU） | `odom` | off | 地面（无帧级消费者） | 今天的 off 档（2D scan **允许** = 复现 §25 的"半盲"臂） |

四档的**共同点**（写死，别再分叉）：

* `tf`/`base`/`raw` 的 `odom` **一模一样**（LIO 的原始世界，IMU 斜装就斜 30°）；它们只差
  "谁负责水平"与"障碍层用哪种高度语义"。
* `lio` 是**唯一**会让估计器改 `odom` 的档，也是**唯一**要求 `lio:=small_point_lio` 的档。
* `base` 的"高度语义在 `base_link`"由**感知侧**承担：`pointcloud_to_laserscan` 的
  `target_frame=base_link`（既有）+ linefit 的 `gravity_aligned_frame=base_link`（本档新增的
  互斥覆盖键）。局部代价图帧仍是 `odom` —— 所以 **2D `/scan` 盘面没有逐点高度**（§25 实测
  in-band 43.7% / kept-y 单侧），必须换成吃 `/segmentation/obstacle` 的**逐点层**。
* `raw` 与 `base` **都**关闭 `odom_level` 静态边、局部代价图 = `odom`、`odom_level_tf` 不启动；
  唯一差别就是障碍层与 linefit。

### 4.14.2 参数面：**一个**开关 + 6 个硬报错别名

| launch 参数 | 状态 | 默认 | 说明 |
|---|---|---|---|
| **`gravity`** | **唯一**用户开关 | **`tf`** | `choices=['tf','base','lio','raw']`；t=0 打一行 `[gravity] …` 收据（odom/帧/E1/z/障碍层/lio） |
| `gravity_align_mode` | **删除 → 硬报错别名** | `auto`（旧默认） | 传非默认值 ⇒ t=0 停，消息指向 `gravity` |
| `gravity_align_z_origin` | **删除 → 硬报错别名** | `ground` | 同上 |
| `level_frame` | **删除 → 硬报错别名** | `odom_level` | 同上 |
| `z_origin_source` | **删除 → 硬报错别名** | `urdf` | 同上 |
| `lio_init_settle_s` | **删除 → 硬报错别名** | `1.0` | 行为已硬编码：**不等落定** |
| `lio_init_map_extrinsic` | **删除 → 硬报错别名** | `True` | 行为已硬编码：初始化地图**一律** `R_IL·p + t_IL` |

**"硬报错别名"的准确语义**：参数仍然 `DeclareLaunchArgument`（所以 `--show-args` 还能看到它们、
description 以 `[已删除 / 硬报错别名]` 开头），但**不参与任何解析**；用户传的值 ≠ 旧默认值 ⇒
`_check_gravity_contract`（`OpaqueFunction`，launch 第 1123 行）在 t=0 `RuntimeError`，
`ros2 launch` 退出码 **1**，消息逐条给出"旧参数:=值（旧默认值=…）⇒ 改用 …"。
**已知边界**：launch 不保留"值来自默认还是显式"的来源；显式传回**旧默认值**与"没传"不可区分
（按默认处理）。其余 5 个非默认值都能被抓到。

**启动期组合检查**（全部 t=0、全部硬报错，不静默回退）：

1. `gravity:=lio` + `lio != small_point_lio` ⇒ 停（E1 补丁只存在于 SPL 包）；
2. `gravity:=base` + 显式 `local_obstacle:=scan` ⇒ 停（消息解释 2D cliff 与 §25 的 43.7%）；
3. `gravity:=base` + 默认 `local_obstacle`（= `scan`）⇒ **强制 `cloud`** + 响亮 WARN；
   `both ⇒ cloud`、`stvl_both ⇒ stvl`（同样 WARN）；
4. 旧参数非默认值 ⇒ 停（上表）。

### 4.14.3 实现（唯一真源）

| 位置 | 做什么 |
|---|---|
| `bringup_sim.launch.py` `_gravity()` / `_Gravity()`（L854） | 读/校验 `gravity`；**唯一**的档位来源 |
| 同上 `_level_frame()` / `_LevelFrame()`（L877） | 派生水平帧：`tf ⇒ odom_level`，其余 `⇒ odom`。局部代价图合并层、adapter 父帧、rviz `-f`、`odom_level_tf` 的 condition 全部读它 |
| 同上 `_effective_gravity_align_mode()`（L1042） | 派生 E1：`lio ⇒ hybrid`（非 SPL 在这里硬报错），其余 `off`；旧 `auto/urdf/imu` 不再存在 |
| 同上 `_effective_local_obstacle()` / `_EffectiveLocalObstacle()`（L914） | `base` 档强制逐点层；显式 scan 硬报错；nav2 include 与 `scan_band_min` 覆盖层都读它 |
| 同上 `_check_gravity_contract()`（L1123） | t=0：旧参数别名、base 障碍层、`gravity:=tf + localization:=''` 的既有 WARN、四行收据（`[gravity]` / `[odom_level]` / `[gravity_align]`） |
| 同上 SPL **两份**互斥 Node（L~3150） | ① `eff==off`（tf/base/raw）：`publish_tf=false + odom_convention=sensor` + **硬编码** `init_settle_hold=0.0`、`init_map_extrinsic=True`、`gravity_align_h=<URDF>`（诊断），**没有** `gravity_align_*` 键；② `eff!=off`（lio）：① + E1/E1b（`gravity_align_z_origin` 固定 `'ground'`）。旧的"off+level=odom 由 LIO 自己发 TF"那份**已删除** —— §4.12 之后那条 `body` 档 TF 不含 `extrinsic_R`，会吞掉 30° 倾角 |
| 同上 linefit（L3022） | `base` 档用第二份互斥 Node（只多 `gravity_aligned_frame: base_link`）；默认档参数表逐字段不变 |
| 同上 `odom_level_tf`（L3649） | **只在 `gravity:=tf` 启动**（condition = `_LevelFrame()=='odom_level'`）；`z_origin_source` 固定 `'urdf'` |
| `small_point_lio.cpp` | 初始化地图 `init_map_extrinsic=true` 分支加 `R_IL=I` 快路径（`p + t_IL`，与一般式逐位相同）；**初始化地面互检**（L450–497）变成与 E1 无关的兜底：`gravity_align_h>0` 时拟合第一帧地面，`|Δ|≥3 cm ⇒ 响亮 WARN（"出生未落定/地面不平/悬空"）`、`≥5 cm ⇒ ERROR`，**定基永远用 URDF 值** |

**为什么 `tf` 仍然"逐字节等于今天"**（在**行为/执行集**口径上；参数面按本节要求**有意**变了 3 处）：

* 帧树/静态边数值/z 原点/局部代价图帧/adapter 父帧/rviz 固定帧/障碍层选择**全部**与改动前相同
  （`tf` 就是旧的"`level_frame:=odom_level` + `gravity_align_mode:=auto⇒off` + `z_origin_source:=urdf`"）；
* `robot_description` 的 xacro 命令与 9 个 (robot, mount) 组合的输出 md5 **逐字节相同**（本节只改
  launch 的 substitution，不碰 URDF/xacro）；
* 运行期节点集合相同（唯一新增的 Node **定义**是 `base` 档的 linefit，它与原 linefit 的
  condition 互斥 ⇒ `tf` 下**只有一个** `ground_segmentation` 在跑）；
* 有意改变：SPL 的初始化参数从 `init_settle_hold=1.0 / init_map_extrinsic=True` 变成
  `0.0 / True`（§32.8 证明门本来就不推迟任何一帧；所有帧都是落定后的静止帧），以及 launch
  收据/参数表（`-4/+1` 面 + 6 个别名）。

### 4.14.4 不变量（新增）

| # | 不变量 | 判据（可执行） |
|---|---|---|
| **I17** | `gravity` 是重力/水平帧的**唯一**用户输入 | `--show-args` 里只有 `gravity` 有非 `[已删除 / 硬报错别名]` 的重力语义；6 个旧参数非默认值 ⇒ t=0 `exit 1`；launch 源码里不存在对 6 个旧参数的 `LaunchConfiguration(...)` 读 |
| **I18** | `odom_level` 只在 `gravity:=tf` 存在；其它三档**没有这个帧** | `ros2 run tf2_ros tf2_echo odom odom_level` 在 base/lio/raw 下失败（或 `/tf_static` 里没有该边）；`ros2 node list` 里 `odom_level_tf` 不出现（`gravity:=tf` 下出现） |
| **I19** | `gravity:=base` 的局部障碍层是**逐点**的 | live param：`local_costmap.obstacle_layer.enabled=false`、`obstacle_cloud_layer.enabled=true`（或 `stvl_layer.enabled=true`）；`band_guard` 对这档 **IDLE**（没有启用的 2D scan 高度带）；显式 `local_obstacle:=scan` ⇒ t=0 `exit 1` |
| **I20** | `gravity:=base` 的高度语义在 `base_link` | `/scan` 的 `frame_id=base_link`（既有）；linefit 的 live 参数 `gravity_aligned_frame=base_link`；`/segmentation/{ground,obstacle}` 只有一个发布者 |
| **I21** | `gravity:=lio` 只有 SPL 能选 | `lio:=fastlio|pointlio|cartographer|none` + `gravity:=lio` ⇒ t=0 `exit 1`；SPL 下 `ros2 param get /small_point_lio gravity_align_mode = hybrid`、`publish_tf=false`、`odom_convention=sensor` |
| **I22** | `gravity:=raw` 刻意保留半盲 | 局部帧 `odom`、`obstacle_layer`（2D scan）enabled、`band_guard` 给 `WARN ~43.7%` / kept-y 单侧（§25 的 off 档复现） |

### 4.14.5 边界（明确不在本节范围）

1. **`gravity:=base` 的"逐点层"是 `cloud` 或 `stvl`，不是 2D scan**：base 档的局部代价图帧仍是
   `odom`（斜的），`obstacle_cloud_layer` / `stvl_layer` 吃的是 `/segmentation/obstacle`（已被
   linefit 按 `base_link` 高度语义分好类）⇒ "高度语义在 `base_link`"是**感知侧**的性质，
   **不是**代价图里又做了一层 z 闸。要真正在 `base_link` 里做代价图 z 闸 = 换帧，不在本节。
2. **`gravity:=base` 的显式 `local_obstacle:=both|stvl_both`** 会被静默降级成 `cloud|stvl`（+WARN），
   不是硬报错（硬报错只留给显式 `scan`，与用户批准的设计一致）。
3. **`lio:=fastlio|pointlio` 的 `gravity:=base|raw`**：没有 SPL 的初始化地面互检（那条 WARN 只在
   SPL 的初始化里做）；它们的 `odom` 仍是各自初始化器决定的原始世界，本节不对它们加任何新约束。
4. **`gravity:=tf` 的 `localization:=''` 回退**：仍会打 §4.11.6 的 WARN（map 钉在原始世界）；
   `gravity:=base|raw` 的同类回退**不另打**（它们本来就不承诺水平帧）。
5. **旧参数"显式传回旧默认值"** 不可检测（见 §4.14.2 的边界）。
6. **`hzmirm` 的云台转动**（`turret_pitch_deg≠0`）：`base` 档的 linefit `gravity_aligned_frame=
   base_link` 仍假设 `base_link` 水平；云台一动这条语义就退化（与 §4.11.6#4 同一局限）。
7. **本节没有改任何地图/PCD 资产、`src/rm_simulation/**`、`docs` §10–§32 的正文**（只追加本节
   与 measurements §33）。
8. **实测（2026-10-27，第二十四轮）**：`gravity:=tf|base` 各 3 跑的正式 A/B、已知障碍物落点、
   以及 point_lio「初始化图 vs 逐帧图」一致性核验见 `docs/frame_contract_v3_measurements.md` §34；
   数值支持**默认档保持 `tf`**（§34.2.4）。

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
| **`continue`** | 沿用上次会话的位姿（存档/`.posegraph`/`.pbstream`） | `map_source` + 一致性检查 | 一致性检查通过后 | 本仓已有 `map_autocontinue`/`map_start_pose`/`map_resume_check`（L:1650/1657/1693） |

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
`T_trip=2 s` 与既有 `gicp.lost_after_rejections=25`（2.5 s @10 Hz）同量级且更保守（`gicp_registration_sim.yaml:245`）。
**3 m** 直接取 SICK 的有界外推值（S3 §3.4 E21）。
**这些是默认值，需在真机按"定位更新率/最高车速"重定**（S1 §9.6 第 3 条同款告诫）。

---

## §6 校验矩阵

### 6.1 合法组合（示例，非穷举）

| # | `mode` | `map_source` | `global_relocalizer` | `local_tracker` | 形态 | 说明 |
|---|---|---|---|---|---|---|
| L1 | `mapping` | `none` | `none` | `none` | 建图 | `mapper` 槽生效（slam_toolbox/cartographer），`map→odom` 由它发 |
| L2 | `slam_nav` | `none` | `none` | `none` | 边建边导航 | 在线 SLAM 直接持有 `map→odom`（L:2027-2035） |
| L3 | `nav` | `2d_grid` | `amcl_global` | `amcl` | **M1** | 一体机；`global_relocalizer` 与 tracker 是**同一节点**，允许 |
| L4 | `nav` | `2d_grid` | `none` | `amcl` | M1（只给初值） | `initial_pose:=known` 时必须 |
| L5 | `nav` | `pcd` | `descriptor` | `gicp` | **M2 串联** | 全局给先验 → 局部精配准；`initial_pose` 可为 `unknown` |
| L6 | `nav` | `pcd` | `none` | `gicp` | M2（外部初值） | 必须 `initial_pose:=known` 或 `/initialpose` |
| L7 | `nav` | `posegraph` | `none` | `slam_toolbox_localization` | M1 变体 | 起点靠 `/initialpose`（S3 §2） |
| L8 | `nav` | `pbstream` | `none` | `cartographer_localization` | M1 变体 | 图冻结；只支持局部窗口（S3 §3.3） |
| L9 | `nav` | `none` | `none` | `none` | **LIO 当绝对定位**（回退） | 需要 `map≡odom` 单条静态桥（L:2388-2397）；v3 记为 `degraded` |

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
        且未启用 map≡odom 回退桥（bringup_sim.launch.py:2388-2397 的条件为假）
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
| `livox_tilt_axis` | **已删除（2026-10-21，冻结 `roll`）** | （无） | 已落地 |
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
| **I3** map→odom 唯一 | 同上 + 检查 `mapper`/`localization` 互斥条件 | L:2027-2038 的条件求值 | 无两个 `map→odom` 发布者 |
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
| **C3** | `initial_pose:=continue`，存档一致 | `map_source` + `map_resume_check` | 检查通过后 `TRACKING`；yaw/位置与存档差 < 容差 | `map_resume_check` 的 pos/yaw 判据（L:1709-1717） |
| **C4** | `initial_pose:=known` 但**给错**（偏移 2 m / yaw 错 90°） | 只 `local_tracker` | **必须**出现 `valid=false`/`quality` 跌穿阈值 ⇒ `LOST`，**不得**静默继续 | `gicp` 的 `gate_*`+`lost_after_rejections`（S3 §6.1 R3）或 AMCL 粒子协方差；`/cmd_vel_nav` 在 `T_trip` 内归零（`stop`/`degrade`） |

> **C4 是最重要的一条**：它把"位姿源坏了"变成**可观测**的；缺了它，`warn`（默认档）就退化成"知道坏了但不处置"。
> **注意**：C4 的"2 m / 90°"是【设计】取值（要大于 `gicp` 的 `plausible_margin_xy: 1.0 m`、`gate_yaw_step_max_deg: 15°`），
> 落地时应按实车的"最后一次可信位姿 + 运动包络"重定。

---

## §10 删除清单（进入 v3b 时生效）

### 10.1 `robot11_mount` / `robot11_mount_axis`

* **删什么**：`robot11_mount`（`plugin|urdf|sensor`，L:1437-1452）与其 `choices=list(_LIVOX_MOUNTS)`
  （L:172、L:1453）、`_validate_livox_mount`（L:175-181）、`livox_tilt_axis`（L:1407-1416）。
  `robot11_mount_axis` **本仓不存在**（全仓 grep 无命中）——任务书里这个名字对应的是 `livox_tilt_axis`。
  **2026-10-21 已实施**：`livox_tilt_axis` 与 `livox_raise_m` 已从 launch 删除（冻结 `roll` / 0），
  生成物逐字节不变；见 `docs/frame_contract_v3_measurements.md` §28。
* **为什么**：三档并存 = 倾角有三个所有者（P2、S4 §7.2 O3）；`<tilt_rpy>` 让 TF 说"平"、数据说"斜"（S8 §2.2）。
* **删之前必须做**：`sensor` 语义成为唯一语义（倾角在 URDF、点云在传感器系）；Classic 默认档从 `plugin` 迁到
  `sensor`（**唯一的破坏性变更**，必须单独一个 commit + 回归）；gz 后端的 `robot11_mount:=sensor` 硬要求消失（S8 §0）。
* **影响面**：`robot:=robot11` 的默认点云表达（水平 → 斜传感器系）、`linefit.gravity_aligned_frame`/p2l `target_frame`
  的增量覆盖（L 里只对 `sensor` 档加的那两份增量文件，S6 §J.3）、所有已实测基线（S6 §A/§J/§K 的数字）都要重标口径。

### 10.2 `camera_init→map` 全 0 静态桥（`L:2371-2372`）

* **删什么**：`tf_bridge_camera_init_to_map`（L:2367-2374，`arguments=['0','0','0','0','0','0','camera_init','map']`）
  与兄弟桥 `tf_bridge_body_to_odom`（L:2376-2383）。
* **为什么**：它们把 `map ≡ camera_init` **写死成恒等**——"倾斜不是缺模块，是被这条边写死成了恒等"（S2 §6.1 第 2 条）。
  在 E1 之后 `camera_init` 这个帧要么不存在（`small_point_lio` 没有它），要么已被对正，桥只会制造第二份真相。
* **附带清掉**：`src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:24-45` 的**单位**
  `base_link→livox_frame` 静态 TF（上游是死代码，我们把它加进了 `LaunchDescription`，S4 §7.2 O4）——
  它与 URDF 的 −30° 关节**互斥**，同时存在即双倍倾角。**注意**：当前 bringup 不走这个 launch（L:1979-1994 自建 Node）
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

* **删什么**：`localization`（8 值枚举，L:1528-1531）与 `mapper`（L:1605-1608）的**同名两项**
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

### 13.5 安装参数改为从 URDF 解析（2026-10-21；旧表只作交叉校验/回退）

> 任务书点名的 "§4.3" 已被「来源与互检」占用；E1b 的 z 原点规范在 §4.7。
> 逐组合对照与决定性测试（含 30°→20°）见 **`docs/frame_contract_v3_measurements.md` §11**。

**已实现**（`bringup_sim.launch.py`，无新节点/新包）：

- **解析**：用与 `robot_description` **同一份** xacro 参数（`_robot_xacro_args`）展开这次真要
  spawn 的 xacro，沿 fixed 关节链合成 `T(base_link←livox_frame)`（rpy **度** + xyz + `livox_z`）
  与 `T(imu_link←base_link)`（adapter 的 xyz + rpy，**弧度**，进 `setRPY`）。链上出现非 fixed
  关节 ⇒ 报关节名/类型/路径并拒绝（不猜运动学）。
- **展开顺序（两级）**：① `xacro.process_file`（与 launch 的 xacro CLI 同一库、同一 mappings）；
  ② 文件本来就是 plain URDF 时直读 XML。都失败 ⇒ **已知模型 WARN + 旧表回退**（banner 标
  `table-fallback`）；**未知模型硬报错**。
- **消费方**：`gravity_align_rpy`（E1）、`_LioAdapterExtrinsic`/`_LioAdapterRobot11`（adapter 的
  xyz/rpy）、`gravity_align_h`（= 地面偏移表 + 解析出的 `livox_z`；`livox_raise_m` 已于 2026-10-21 删除并冻结 0）。
- **交叉校验**：解析 vs 旧表差 **>0.05°/轴 或 >1 mm** ⇒ 响亮 WARN（两个值 + 哪个 URDF/slot 产出），
  **保留解析值**；linefit 的 `sensor_height` 与该槽位 YAML 差 **>1 cm** 也 WARN（**不改** YAML）。
- **仍用字面值（有意）**：robot11 的 30° 是 launch **传给 xacro 的输入**（`_LIVOX_TILT_RPY_ROLL`，冻结 roll；2026-10-21 起无 launch 开关）；
  地面偏移 `_HZMI_BASE_LINK_HEIGHT_M` 是 URDF 给不了的那一个先验（轮半径+轮心，逐台有证据）；
  `lio_tf_adapter_hzmirm_node`（mode=off 的 legacy 非-spl 路径）与默认模型的
  `lio_tf_adapter.yaml` 不改（换参数会动默认 Action 树/默认路径，解析值已与它们交叉校验）。

**默认路径证明**：`tools/scripts/gzsim/launch_increment_diff.py` 5 组参数（`robot:=''` 默认、
robot11 `plugin`/`sensor`、hzmirm、`gravity_align_mode:=hybrid`）全部 **新增 0 / 消失 0 /
共有 Action 相对顺序一致**；`robot_description` 的 xacro 命令字符串 6 组组合逐字节相同；
`gravity_align_mode:=off` ⇒ 组合检查入口即返回、解析器 **0 次求值**。（M §11.5。）

---

## §4.13 `odom` 的 z 锚点：**初始化落定门 + 初始化地图口径**（2026-10-21，默认档）

> **本节只新增**（append-only）：§4.1–§4.12 一个字都没改。它**不动** §4.11 的帧树/静态边/消费者
> 三件事，只把 §4.11.1 里 "`odom_level` 的 z=0 ≈ 地面（`base_link z ≈ +0.1025`）" 那条**前提**
> 变成**可复现的事实**（= §4.11.2 的 I10）。根因/实测/A-B 见
> `docs/frame_contract_v3_measurements.md` §30。

### 4.13.1 问题（一句话）

`odom` 的 z 原点 = **LIO 初始化那一刻的 IMU 位姿**。§4.11 的静态边用 URDF 的 `h_imu` 把它换算成
地面 —— 这只有在"① 初始化发生在车**落定之后**、② 初始化地图与逐点更新**同一口径**"时才成立。
实测（修前）两条都不成立：

* 初始化地图把 `point_deque` 里**整段出生下沉过程**的点云一起入图（26 帧 / 16 万点；`init_map_size`
  是**点数**不是帧数，而 `point_deque` 从不清）⇒ 地面被"抹"成 ~9 cm 厚的带；
* 初始化地图**少了 `t_IL` 平移** ⇒ robot11（`R_IL=Rx(30°)`）上再低 4.33 cm。

⇒ 第一次点更新把状态**拽下 9.8 cm** ⇒ 静态边用 URDF 的 `h` 时地面落在 `odom_level` 的 **−0.13 m**，
`h_meas` 必然 **ERROR**（§29.3 的 0.3912 就是这个）。§29 的 `measured` 挡板把地面按回 0，但
`h_meas` 仍然 ERROR、且 z 原点变成一次启动测量。

### 4.13.2 参数与默认值（都可单独回滚）

| launch 参数 | 节点参数 | 默认 | 作用 |
|---|---|---|---|
| `lio_init_settle_s` | `small_point_lio.init_settle_hold` | **1.0 s**（`0` = 关 = 旧行为） | **落定门**：必须**连续** `hold` 秒"雷达看到的地面高度不动"（近场点云 z 的 10% 分位，峰峰值 ≤ `init_settle_tol` = 5 mm）才允许初始化；**落定前的点云全部丢弃** ⇒ 初始化地图 = 落定后的**一帧**；超过 `init_settle_timeout`（15 s）⇒ 响亮 WARN + 照旧初始化 |
| `lio_init_map_extrinsic` | `small_point_lio.init_map_extrinsic` | **True** | 初始化地图用**与逐点更新同一个**出点公式（`R_IL·p + t_IL`）；`False` = §29 的旧式（只旋 `R_IL`） |
| `z_origin_source` | `odom_level_tf.z_origin_source` | **`urdf`** | 静态边的 z 原点：`urdf` = `t_level = h_imu`（**确定性**，逐位可复算）；`measured` = 第一帧点云拟合地面（§29 的临时挡板，**只作诊断/A-B**） |

**为什么当轮不是"把 spawn z 调成落定高度"**（★ 已被 **§4.13.5 取代**）：当轮的前提是"不许改
`src/rm_simulation/**`"，且 `get_world_config()` 的 `z` 被 4 个 world × 3 个机器人槽位共用
（落定高度各不相同）、`spawn_entity.py` 的 `-z` 没有可透传的参数 ⇒ 一个常数不可能同时对。
**第二十二轮**用户批准后改走"新增 `spawn_z` 参数 + 调用方按模型传值"（§4.13.5）；本节的门与
初始化地图口径**保留**，但角色从"修 z 锚点的主力"降为**兜底**。
**为什么不判"IMU 静止 ≥1 s"**：本仿真的 `gazebo_ros_planar_move` 每步清 vz ⇒ 匀速下沉期间
加速度计仍读 ≈1 g，而静止时 `|a|` 在 7.8–11.8 m/s² 抖 ⇒ `|a|≈g` 既不能证明静止也不稳。
判据因此换成**雷达自己的地面高度**（与"车动不动"同一件事，且不受 `planar_move` 骗）。

### 4.13.3 不变量（新增/收紧）

| # | 不变量 | 判据（可执行） |
|---|---|---|
| **I10′** | `odom_level` 的 z=0 = 地面，**用 URDF 值就该成立**（不需要实测挡板） | 静止窗口内点云在 `odom_level` 拟合：`floor_z ∈ 0 ± 0.01 m`、`base_link z = +0.1025 ± 0.01 m`、`h_meas` **INFO**（Δ<3 cm）；**同一命令 3 跑的 floor_z spread < 1 cm** |
| **I14** | 静态边的 z 原点**不依赖启动测量**（默认档） | `z_origin_source=urdf` ⇒ 静态边 = URDF 几何（`t_level=(0,0,h_imu)`，逐位可复算）；`measured` 只出现在显式选择时，且那时 `h_meas` 必然 ERROR（有意的） |
| **I15** | 初始化**不吞**出生过程的点云 | `small_point_lio` 的 init 收据里"丢弃 N 帧"+"初始化地图 = M 点"满足 M ≲ 一帧的量级（robot11 实测 8119–8120 点；修前 156266–164179） |

**实测（3 跑默认档 + 1 跑对照，详见 M §30.3）**：`floor_z = +0.0053 / +0.0058 / +0.0056 m`
（spread **0.5 mm**）、`base_link z = +0.1081 / +0.1080 / +0.1077`（spread 0.4 mm，判据 +0.1025，
系统偏 +5.5 mm）、`h_meas = 0.2551 / 0.2536 / 0.2537`（Δ 4.4–5.9 mm，**INFO**）、
扫描面 z（代价图帧）= +0.108（与 `base_link` z 一致）、局部 in-band **100%**。

### 4.13.4 边界

1. 门与初始化口径**只在默认档**（`level_frame=odom_level` 且 `gravity_align_mode` 有效值 `off`）
   的 `small_point_lio` Node 上生效（launch 里只往那**一份**参数表加键）；回滚档
   （`level_frame:=odom`）、E1 实验臂、`lio:=fastlio|pointlio|cartographer` **一个键都没多**
   ⇒ 那些路径的行为/参数表逐字节不变。
2. `lio:=pointlio` 的初始化在 `point_lio::ImuProcess::Set_init` 里，**不在本节范围**。实测它
   在首帧后 **0.5 s** 初始化（车还在下沉）⇒ 它的 z 锚点仍低 **7.3 cm**（`base_link z=+0.0298`），
   数字见 M §30.5.2。要修得动那个包（未做）。
3. 门的判据是"雷达能看到地面"。悬空 / 楼梯 / 被抬起的出生点走 **15 s 超时兜底**（响亮 WARN，
   z 锚点退回修前水平）——**不静默**。本机只在 RMUL2026 + robot11 上实测过。
4. 残余 **+5.5 mm** 的 `base_link z` 来自 LIO 初始化后前 2 s 的暂态（state z +0.012），不是静态边；
   要消掉得动估计器的初始速度/重力估计（未做）。

### 4.13.5 出生高度 = 落定高度（2026-10-21 第二十二轮，**默认路径**）

> ★ 本节**取代** §4.13.2 里"为什么当轮不是把 spawn z 调成落定高度"那一段（前提已变：用户批准
> "直接改机器人位置让他直接在地面上"；改动面 = `src/rm_simulation/**` + `bringup_sim.launch.py`，
> **不碰**地图资产 / `src/rm_localization/**`）。§4.13.2 的**落定门**与**初始化地图口径**保留。

**做法**：`rm_simulation.launch.py` 新增 launch 参数 `spawn_z`（默认 `''` = 回落 world 配置的 z，
= 改造前行为；非空必须是浮点数，否则 t=0 硬报错），只改 `spawn_entity.py` 的 `-z`；
`bringup_sim.launch.py` 对 `world ∈ {RMUC2026, RMUL2026}`（floor z=0 的标定世界）传
`_HZMI_BASE_LINK_HEIGHT_M[robot] + 1 mm`（**同一张表**也是 `gravity_align_h` 的来源）：

| robot | 落定 `base_link` z（表，URDF 几何） | 传下去的 `spawn_z` |
|---|---|---|
| `robot11` | 0.102499 | **0.103499** |
| `hzmirm` | 0.0 | **0.001000** |
| `''`（默认模型） | 0.06 | **0.061000** |

1 mm 余量 = 让接触在第一个物理步里解出来，避免一出生就穿透地面（穿透会弹一下，等于又制造一个
小下落）。**`get_world_config()` 的 `z` 一个字节没动**（仍是 0.2）：它是
`map_asset_guard.py::WORLD_SPAWN` 的镜像源、也是既有存档 sidecar `spawn_pose.z` 的比对基准
（改它会让既有 RMUL2026/RMUC2026 存档全部 `refused_spawn`）；地图原点只取决于 x/y/yaw。

**不变量（新增/修订）**

| # | 不变量 | 判据（可执行） |
|---|---|---|
| **I16** | 出生**不发生有意义的自由落体**，LIO 初始化时车已静止 | GT（`/odom_ground_truth`，planar_move 报的模型世界位姿）在整跑内 z 的 **max−min < 1 mm**；`spawn -z − GT(z) ≤ 1 mm`（= 那 1 mm 余量）；SPL 初始化在首帧后 2.0 s / pointlio 0.5 s / fastlio 0.3 s，都在 GT 已稳定之后 |
| **I10″** | `odom_level` 的 z 锚点**只由 URDF 决定**（不依赖出生轨迹、不需要实测挡板） | 同 I10′，但 `base_link z` 判据收紧到 **+0.1025 ± 0.01 m**；SPL 实测 +0.1032/+0.1020/+0.1031（spread 1.2 mm）、`floor_z` +0.0023/+0.0021/+0.0007（spread 1.6 mm）、`h_meas` 0.2587–0.2600（Δ 0.1–0.8 mm，**INFO**） |
| **I15′** | 初始化**不吞出生下沉段的点云**（I15 的数值口径修订） | 丢弃帧数 = 落定前帧（SPL 实测 9 帧，修前 26 帧）；**初始化地图可以 >1 帧**，但必须全部是落定后的静止帧（实测 90209–90212 点 ≈ 11 帧；§30 是 8120 点 = 1 帧，因为那时门在 2.6 s 才放行）。"吞下沉段"这一点仍成立 |

**落定门（§4.13.2）的角色：兜底，默认值保持 1.0 s**。实测它在**首帧后 0.84–0.89 s** 放行
（峰峰值 0.0000 m，8 帧），而 SPL 的 200 帧 IMU 初始化门在 **2.0 s** ⇒ 门**不推迟**任何一帧
（3 跑 init 都在首帧后 1.990–1.992 s）。它现在的价值只在"出生高度不对 / 地面不平 / 车被抬起"
时兜底（15 s 超时 + 响亮 WARN）；关掉（`lio_init_settle_s:=0`）在本轮场景下结果相同，但会失去
这层兜底 ⇒ **保留 1.0**。

**实测**（`docs/frame_contract_v3_measurements.md` §32.3–§32.5）：GT z 整跑 max−min = **9 µm**、
`spawn -z − GT = 1.000 mm`；SPL 3 跑 z 锚点见 I10″；**pointlio 的 6.5 cm 锚点偏差消失**
（`base_link z = +0.0961/+0.0964/+0.0974`，换算 `floor ≈ −6.4/−6.1/−5.1 mm`；§30 是
+0.0298/+0.0362/+0.0356）；SPL/pointlio 局部 in-band **100%**、`odom_level→base_link`
roll/pitch ≤ 0.21°、静态边逐位不变、`z_origin_source` 仍为 `urdf`。

**边界**：① `spawn_z` 只对 `world∈{RMUC2026,RMUL2026}` 生效；RMUC/RMUL 2024、单跑
`rm_simulation.launch.py` 的默认 `''`、以及 `robot` 不在表里时**逐字节回到改造前**。
② `sim_backend:=gz` 有自己的 spawn 路径（模型 pose 在世界 SDF 里），本轮**没改**。
③ 默认模型（`robot:=''`）的轮胎下沉 ~9 mm 不在表里（表是 URDF 几何）⇒ 它实际仍会落 ~10 mm；
本轮没在 nav 里实测默认模型。④ fastlio 臂**没有** z 锚点逻辑（不在本节范围），只回归到
"出生/落定 + 局部 in-band（生效带）100%"（§32.7）。

---

## §4.15 删除三个冗余 launch 参数：`robot11_mount` / `turret_yaw_deg` / `turret_pitch_deg`（2026-10-27）

> 用户批准（原话）："robot11_mount，turret_yaw_deg / turret_pitch_deg 删除吧"。
> 目标：三个参数从 **launch 接口**里消失（`--show-args` 不再列出），但**默认路径逐字节不变**；
> 仍传旧参数 ⇒ **t=0 硬报错**（不静默忽略、不静默等价）。
> 本节只新增；§1–§4.14 的历史叙述一律不改（旧节里的 `robot11_mount:=sensor` 字样是**历史**，
> 不再是可传参数）。实测表（md5 / show-args / 守卫 / 冒烟）见 measurements **§38**。

### 4.15.1 删了什么、为什么能删

| 参数 | 曾经的作用 | 为什么可以删（默认值即唯一值） | 删除后的替代 |
|---|---|---|---|
| `robot11_mount` | 仅 `robot:=robot11`；唯一合法值 `sensor`（`_LIVOX_MOUNTS=('sensor',)`，plugin/urdf 已硬报错）；只喂 xacro `livox_mount` + 一个组合检查 | 生成物 xacro 里 `<xacro:arg name="livox_mount" default="sensor"/>`，且三条 property 全是常量（`livox_mount_tilt_in_joint=False` / `livox_mount_sensor=False` / `livox_joint_rpy="0 0 0"`）⇒ **不传 = 传 `sensor`**，输出逐字节相同 | 无：安装方式的唯一语义（§4.12）由 xacro 默认值承担 |
| `turret_yaw_deg` | 仅 `robot:=hzmirm`；`base_to_turret` 关节的静态 yaw（度），默认 0 | 上游 URDF 两个云台关节都是 **fixed**，本仓**没有 gimbal 控制器** ⇒ 它从来不是控制量，只是 spawn 时的**静态姿态**；默认 0 = 与上游数值等价 | 无：云台**固定** yaw=0 |
| `turret_pitch_deg` | 仅 `robot:=hzmirm`；`turret_to_head` 关节的静态 pitch（度），默认 0 | 同上；`h = 0.6 + 0.2·cos(pitch)`，pitch=0 ⇒ h=0.8 m。pitch≠0 还会破坏雷达重力对齐（历史实验，见 docs/robot_models.md §3.3） | 无：云台**固定** pitch=0，h=0.8 m |

为什么现在删（而不是继续保留一个"唯一值参数"）：唯一值参数只会制造"第二个所有者"的错觉 ——
xacro 的默认值已经写着 `sensor`，launch 再传一份；云台角同理（默认 0/0 与不传完全等价）。
删掉后接口只剩**真正会变的东西**。安装方式的完整历史（plugin/urdf/sensor 三档）见 §4.12 与
`docs/tilted_lidar_fidelity.md` §J；云台的判断见 `docs/robot_models.md` §3.3。

### 4.15.2 实现（file:line）

**`src/rm_nav_bringup/launch/bringup_sim.launch.py`（主改动）**

| 位置 | 改动 |
|---|---|
| 155–162 | `robot11_mount` 的历史注释替换成"已删除 / 为什么 / 去哪看" |
| 219 | `_lio_adapter_robot11(mount='sensor')` → `_lio_adapter_robot11()`（`mount` 不再参与几何） |
| 333–358 | `_robot_xacro_args()`：hzmirm **不再追加** `turret_yaw_deg`/`turret_pitch_deg`；robot11 **不再追加** `livox_mount`（只剩 `visual_decimated`） |
| 561–584 | `_urdf_livox_mount_table()`：robot11 不再校验 mount；hzmirm 的 R(yaw,pitch) 在 0/0 时是单位阵 ⇒ 直接写 `xyz=[0,0,0.8]`、`rpy=[0,0,0]` |
| 713–727 | `_parsed_livox_mount()`：`mount` 只作日志/缓存标签（`'sensor'`），不再读 launch 参数 |
| 868–916 | 新增 `_REMOVED_LAUNCH_ARGS` + `_check_removed_launch_args()`（t=0 守卫，见 4.15.3） |
| 1852–1870 | `_MountReadout` 改为静态收据（不再按 `robot11_mount` 分支） |
| 2604–2612 | `sim_backend` 的 description 不再教人传 `robot11_mount:=sensor` |
| 2648–2655 / 2746–2751 | 两处"已删除"说明；三个 `DeclareLaunchArgument` 全部删除 |
| 3159–3179 | p2l **只剩一份**（`laserscan_params_sensor_frame.yaml` 增量覆盖，无条件）——删除前唯一合法值就是 `sensor`，`!= 'sensor'` 的门恒假 |
| 4191 | 守卫作为 **LaunchDescription 的第一个 Action**（先于任何声明/节点） |
| （删除） | `_LIVOX_MOUNTS` / `_validate_livox_mount()` / `_check_robot11_mount_combination()` / `declare_robot11_mount_cmd` / `declare_turret_yaw_deg_cmd` / `declare_turret_pitch_deg_cmd` / 组合检查的 `OpaqueFunction` / gz include 里的 `'robot11_mount': LaunchConfiguration(...)` |

**`src/rm_simulation/rm_livox_gz_sim/launch/gz_sim.launch.py`（唯一一处越界改动，用户/父任务批准）**

| 位置 | 改动 |
|---|---|
| 24 | 示例命令删掉 `robot11_mount:=sensor` |
| 127–141 | `_materialize()` 不再读 `LaunchConfiguration('robot11_mount')` |
| 158–161 | 删掉 `mount not in ('sensor','urdf')` 的校验（没有第二个取值可校验） |
| 209–229 | 单独跑时的 `xacro` 命令去掉 `livox_mount:=$mount`（取 xacro 默认 `sensor`，输出逐字节不变） |
| 353–358 | 删掉 `DeclareLaunchArgument('robot11_mount', …)` |

为什么只动这一处：该文件是 `--show-args` 里 `robot11_mount` 的**唯一残余声明**（bringup 的声明
删掉后，gz include 的同名声明会在 `get_launch_arguments()` 的递归里露出来，让总数只 −2）。
删它是一个**声明**的删除，不改变 gz 生成物（见 §38.2 的 xacro md5）。除此之外
`src/rm_simulation/**` 一个字节没动。

### 4.15.3 t=0 硬报错守卫（no silent ignores）

ROS 2 Humble 对**未知**的 `name:=value` 是**静默接受**的（§28.4 实测：exit 0、无 WARN），所以
"删掉声明"本身**不够**。`_check_removed_launch_args()` 是 `LaunchDescription` 的**第一个 Action**，
在 t=0 抛 `RuntimeError`，两个检测面任一命中：

1. `context.launch_configurations` 里出现这三个名字。它们**没有声明** ⇒ 没有默认值 ⇒ 出现即
   "有人显式设了"（命令行，或父 include 的 `launch_arguments` —— launch 的 include 会先执行
   `SetLaunchConfiguration` 再访问子 LD，所以父 include 也逃不掉）；
2. `context.argv` 原文（复用 §4.14 的 `_launch_arg_was_given()`）。①已覆盖②；保留②是为了"以后
   即使有人给它们加回声明（带默认值），命令行显式传仍然会被抓"。

消息给出每个被传的名字/值 + 替代说明，并指向本节 + measurements §38。实测（§38.5）：
单独传三个名字、一次传三个、以及**父 include 传**都 exit 1，且**没有任何节点被启动**。
为什么不用 §4.14 那种"声明成硬报错别名"：本任务的验收要求这三项从 `--show-args` **消失**，
而声明了就一定还在。

### 4.15.4 不变量

| # | 不变量 | 判据（可执行） |
|---|---|---|
| **I17** | 默认路径（`robot:=''` / `hzmirm` / `robot11`）的生成物**逐字节不变** | `robot_description` 的 md5 = `bc62af22…` / `a13e0530…` / `4f19a8b4…`（§38.2 表）；生成物 xacro 文件本身 md5 不变；`h` = 0.235 / 0.8 / 0.25952717 |
| **I18** | 传任一已删除参数 ⇒ **t=0 硬报错**，退出码 1，0 个节点启动 | §38.5 的 5 条命令；错误消息含 `§4.15` / `§38` |
| **I19** | `--show-args` 不再列出这三个名字 | bringup 107 → **104**；gz 单独 17 → **16**；逐块 diff 只少这三个（§38.3） |
| **I20** | 真正起进程的执行集不变 | 默认 / hzmirm / robot11 三组：process `Node/IncludeLaunchDescription/ExecuteProcess` **9 = 9**，归一化 md5 相同（§38.4） |

### 4.15.5 边界与回滚

- **回滚到旧行为**：`git show c661c2c:src/rm_nav_bringup/launch/bringup_sim.launch.py` 到 scratch
  副本（不要覆盖工作树）；或对 `turret_*` 直接用 scratch xacro 传
  `turret_yaw_deg:=… turret_pitch_deg:=…`（xacro 的 arg 仍在，只是 launch 不再暴露）。
- **`robot11_mount` 的旧三档（plugin/urdf）**：xacro 侧 `livox_mount` arg 仍在但**不再改变几何**
  （2026-10-21 起）；要让插件/关节真的变，必须改 scratch 生成物 + 上游 URDF，launch 不提供。
- **`ground:=patchwork`**：旧 `_check_robot11_mount_combination()` 会因默认 `robot11_mount=sensor`
  把 patchwork 硬拦下（该检查的前提 —— sensor 专属的 `gravity_aligned_frame` 覆盖文件 —— 已在
  2026-10-21 删除）。这个检查随参数一起删除后，`ground:=patchwork` 不再被误拦；这不是本轮新增
  的行为目标，登记在 §38.7 的诚实清单。
- **`sim_backend:=gz`**：bringup 不再转发 `robot11_mount`；gz 后端自己的同名参数也已删除
  （安装方式固定 = xacro 默认 `sensor`，gz 侧 `--tilt-rpy` 仍把传感器 link pose 钉成 −0.5236）。
  直接 `ros2 launch rm_livox_gz_sim` 时若仍传 `robot11_mount:=…`，ROS 2 会**静默忽略**
  （那个 launch 没有守卫）—— 这是"只允许改这一处"的已知边界；走 bringup 的路径会被 t=0 拦住。
