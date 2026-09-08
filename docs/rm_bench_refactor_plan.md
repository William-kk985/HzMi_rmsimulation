# HzMi RM 多算法仿真实验台 —— 改造设计（草案 v0.1）

> 本文档是**路线图/设计草案**，沉淀 2026 赛季前关于"把本仓库改造成大型多算法仿真实验台"的讨论结论，供后续**逐步实现**。
> 状态约定：本文只做设计，不承诺任何代码改动已完成；每一阶段的动工前需先在对话/评审里确认该阶段范围。
> 关联文档：`docs/rm_algolab_plan.md`（宏架构原草案）、`docs/rm_algorithm_catalog.md`（算法候选池）、`docs/rm_algorithm_overview.md`（角色×场景）。

---

## 1. 背景与目标转变

**现状**：本仓库是 ROS2 humble 的哨兵导航仿真，谱系上参考了 中南 CSU / 北极熊 PB / 哈工大 HIT 等多套开源（互相借鉴整合，非直接复制），Gazebo + 旋转 Mid360 + FAST-LIO/Point-LIO + Nav2。

**目标转变（重要）**：
- ❌ **不做**：一套配置 sim2real 打通到我们真车（本仓库不是给真车直接用的）。
- ✅ **要做**：改造成一个**大型多算法仿真实验台**——
  1. 在清晰的角色框架下**灌入并对比大量职能算法**（建图 / 定位 / 重定位 / 规划 / 感知；自研算法或成品包都行）；
  2. 评测出效果好的组合，**按"包清单"冻结导出**到独立的真车 workspace（真车集成另算，不在这套里）；
  3. 因此仓库的**一等公民是仿真 + 算法测试**，`config/reality`、`bringup_real` 等真车分支**降级**（见 §3.3/§4.1-R5）。

---

## 2. 已确认的架构判断（本会话结论锚点）

| 编号 | 结论 |
|---|---|
| A1 | **目录 = 域**（上游粗分：simulation/driver/perception/localization/navigation…）；**角色 = 声明**（注册表/文档）。不要把目录改成严格按职能命名——一体包（如 LIO 同时是里程计+建图）放哪个职能目录都不对。 |
| A2 | **第三方成品包整包复用、代码层永不拆**（拆了 = 上游无法合入、没人维护、行为不一致）。一体包的多角色身份用"注册表声明 + launch 模式参数"表达（例：Cartographer 一个包有 mapping / pure_localization 两个模式）。 |
| A3 | **双轴选型**：① 组合轴 = launch + yaml，管**第三方包与跨包组合**；② 实现轴 = 宏规范（config.hpp 体系），**只用于自研包内**的编译期隔离与调试开关。 |
| A4 | 现架构本质是 **3D 感知 + 3D 定位 喂 2D 规划**（"伪 3D"）；`/scan` 是 2D 世界的唯一入口，**降维是命门**。2D 定位族（AMCL/slam_toolbox）只收 LaserScan。 |
| A5 | "下坡/过洞"对轮式机器人 = **2.5D 地形感知谱系**，不是真 3D 规划。预留"感知层可开关降维 + 3D/2.5D 规划变体族"作为远期选项。 |
| A6 | perception 三件套（互补滤波 / 地面分割 / 点云降维）= 被**编排成链的单功能小包**，各自可独立启停/替换；当前 bringup 把它们**无条件常开**是省事写法，存在冗余（互补滤波实际只喂 FAST-LIO，Point-LIO 吃原始 /livox/imu）。 |
| A7 | 边界消息（裁判/目标/控制）与视觉解耦：导航侧只依赖**边界契约**。契约内容暂缓定义，但架构上必须留一个接口槽位。 |
| A8 | 本机所有算法包 = 借鉴整合多套开源，表述上不写"源自/复制自某项目"，只写"参考"。 |

---

## 3. rm_nav_bringup 现状诊断

### 3.1 现状：一个包装了 6 类职责

| 职责 | 现有内容 | 属于 |
|---|---|---|
| 装配逻辑 | `launch/bringup_sim.launch.py`(363行) / `bringup_real.launch.py` / `cartographer_sim.launch.py` | 装配层 |
| 算法参数 | `config/simulation/*.yaml`(8份) + `config/reality/*.yaml`(9份) + `config/lua/`(2份) | **算法层（错位）** |
| 场地/机器人模型 | `urdf/sentry_robot_sim.xacro` / `_real.xacro` | 仿真/平台层 |
| 地图资产 | `map/RMUC.* RMUL.* RMUL2026.*` | 数据资产 |
| 点云资产 | `PCD/RMUC.pcd RMUL.pcd` | 数据资产 |
| 可视化 | `rviz/fastlio.rviz pointlio.rviz` | 工具 |

### 3.2 症状（都有证据）

1. **大 launch 条件爆炸**：角色开关（world/mode/lio/localization）用 `LaunchConfigurationEquals` 叠分支；每加一个算法 = 往这个 363 行函数里再插一段（`cartographer_sim.launch.py` 已经开了"新算法塞进 rm_nav_bringup/launch"的先例）。
2. **配置归属错位**：每个算法的参数不跟算法走，而是全部集中在 `rm_nav_bringup/config/`——算法一多必然混乱（"这份 yaml 是谁的？被谁引用？"）。
3. **sim/reality 双份维护**：`config/simulation` 与 `config/reality` 几乎逐文件对应，任何参数改动要改两遍；而我们的目标**不做 sim2real 一键打通**，reality 分支目前是纯负担。
4. **WIP 遗留未收口**（后续 §6 列表）：`use_slam` 参数声明了没用；`empty_map.yaml` 被引用但不存在；`pointcloud_downsampling_sim.yaml` 被引用但不存在；静态 TF 桥（camera_init→map / body→odom / base_link→base_link_fake）属于帧对齐调试的临时手段；启动脚本默认 world=RMUL2026 与 launch 默认 world=RMUL 不一致。
5. **资产与逻辑耦合**：地图/PCD/URDF/rviz 全部在包内，导致"测一个新场地/新算法"都要动这一个包。

### 3.3 根因

`rm_nav_bringup` 继承自上游"**单套导航包**"时代的单装配包设计——当时它只需要编排一套固定管线，配置集中反而方便。**新目标（多算法实验台 + 不 sim2real）与这个单包设计直接冲突**：装配逻辑、算法参数、平台资产需要按不同节奏演进，塞在一个包里必然互相拖累。

---

## 4. 目标架构

### 4.1 职责与配置归属原则（解决混乱的核心）

| 规则 | 内容 |
|---|---|
| R1 | **算法自身参数 → 跟随算法变体/包**：每个被测算法一个独立 config 目录（可放算法包内 `config/`，或放变体目录），不再堆进总装包。 |
| R2 | **装配层只做"选型 + 装配"**：只放组合 profile（选哪几个变体、参数 override）、场地/模式等全局项。 |
| R3 | **资产分离**：场地资产（world/urdf/mesh）归仿真包；地图/PCD 等数据归独立数据目录（按场地命名），不进装配逻辑。 |
| R4 | **变体自包含、可独立描述**：每个被测变体 = 一个清单化目录/条目：`{包名, launch 片段, 参数, 吃的地图产物, 输出, 评测挂点}`。新增算法 = 新增一个变体条目，**不改总装大函数**。 |
| R5 | **本仓库只维护 sim**：`config/reality`、`bringup_real.launch.py` 冻结为历史参考或移出激活路径；真车配置在"部署导出"阶段（§4.6）另行生成。 |

### 4.2 目标目录形态（草案，名字均为占位，待评审）

```
HzMi_rmsimulation/                    # 仿真实验台（meta 仓库）
├── src/
│   ├── sim_platform/                 # [可选重组] Gazebo 场地/URDF/仿真插件（现 rm_simulation）
│   ├── rm_driver/                    # 不变：驱动
│   ├── rm_perception/                # 不变：感知小包（链保持可拆）
│   ├── rm_localization/              # 不变：定位/建图/重定位域（算法替换主战场）
│   ├── rm_navigation/                # 不变：导航域（Nav2 装配件 + teb 等）
│   └── rm_nav_bringup → bench_bringup # 改造：只保留 装配 + profiles
│       ├── profiles/                 # ★ 组合注册表：roles.yaml + 各 combo profile
│       ├── launch/                   # 只剩"入口 + 各角色的变体接入片段（薄）"
│       └── assets/  (或独立目录)      # 地图/PCD 等数据资产（不再与配置混）
├── variants/                         # [可选] 被测变体的"包名+参数+说明"清单目录
├── docs/                             # 规划/规范文档（本文档所在）
└── tools/  bench/                    # 评测脚本（同场景 A/B 对比）—— M4
```

> 说明：目录**重组是可选项**，ROS 按包名找包，移动目录不影响编译；是否重组、何时重组放到阶段里评审。**先落原则（R1–R5），再考虑搬目录。**

### 4.3 角色注册表 schema（草案）

`profiles/roles.yaml` 集中登记每个角色可用变体，组合校验自动执行"地图产物配对 + 场景约束"：

```yaml
roles:
  lio:        # 里程计/连续定位（3D LIO 系）
    fastlio:  { package: fast_lio,  roles: [odom, mapping_3d],  map_out: pcd,  scene: [A,B] }
    pointlio: { package: point_lio, roles: [odom, mapping_3d],  map_out: pcd,  scene: [A,B] }
    # kiss_icp / dlio / glim …（候选，加一行即可）
  mapper_2d:
    carto:    { package: cartographer_ros, mode: mapping, map_out: [pgm, pbstream] }
    slam_toolbox: { package: slam_toolbox, map_out: [pgm, posegraph] }
  local:      # 重定位（必须与地图产物配对）
    amcl:         { map: pgm }
    slam_toolbox: { map: posegraph }
    icp:          { map: pcd }
  nav:        # 规划导航（将来含 3D/2.5D 族）
    nav2_teb: { family: 2d }
    nav2_dwb: { family: 2d }
    # 自研/轨迹/2.5D 系：family: 3d（远期）

scenes:
  map_then_nav: { local: 必选1, mapper_2d: 关 }
  sim_map_nav:  { local: 免,   mapper_2d/lio 在线 }
```

### 4.4 新算法接入的"标准五步"（将来写进规范）

1. 放好包（自研 → 角色域目录 + 自研规范；成品 → apt 或子模块 pin，见 A2）；
2. 建变体条目（一个 launch 片段 + 参数，放 profiles/ 或算法包 config/）；
3. 注册表 roles.yaml 加一行（含角色/地图配对/场景）；
4. 自检：跑该变体的最小端到端（§4.6 评测挂点）；
5. 评测对比、决定去留；胜出者进入"部署清单"（§4.6）。

### 4.5 感知 2D/3D 双支路（远期变体，对应 A5）

- `pointcloud_to_laserscan` 从"唯一出口"改为**可开关环节**；
- 感知层出口保留两条：`/scan`（2D 世界入口，喂 Nav2 族）与 分割后障碍点云/带高度切片的点云（喂 3D/2.5D 族）；
- 待仿真世界加入坡道/桥洞等 2.5D 结构后，用于测试"下坡/过洞"类变体。

### 4.6 评测与"部署导出"（明确不 sim2real 打通）

- **评测**：统一场景脚本（同场地/起点/目标/指标：到达率、耗时、振荡、急停、里程精度），变体 A/B 输出报告；
- **部署导出**：选定黄金组合 → 生成一份**包清单**（包名 + 版本 pin + 配置 + 真车 launch 骨架），真车侧据此在独立 workspace 组装。本仓库不承担真车运行。

---

## 5. 分阶段实现路线（每步先评审、后动码）

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| **M0 盘点** | 全仓库包总表（角色/来源/版本/配置位置/接线点）+ 本文档评审 | 一份 inventory 文档，人人能答"某包在哪、干什么、配置在哪" |
| **M1 bringup 拆分** | 按 R1–R5 重构：算法参数移出总装包；WIP 问题逐条收口（§6）；`config/reality`/`bringup_real` 降级 | bringup_sim 恢复干净；增删一个算法不再动大函数；sim 单份配置 |
| **M2 组合矩阵** | profiles/roles.yaml 落地；launch 补 `mapper`/`nav` 轴（现只有 lio/localization） | 现有算法都能用"一个 profile 一行"表达；组合校验（地图配对/场景约束）自动生效 |
| **M3 感知双支路** | 降维节点可开关；出 3D 支路最小样例；仿真加坡/洞测试场景 | 同一场地可分别喂 2D 族与 3D/2.5D 族跑通 |
| **M4 评测闭环** | bench 评测脚本 + 部署清单导出工具 | 一键 A/B 对比出报告；一键导出黄金组合包清单 |

> M0–M1 是当前最值得先做的（成本低、立刻消混乱）；M3–M4 可等 2026 场地需求明确后再启动。

---

## 6. 现存待修问题清单（M1 收口对象）

| # | 问题 | 位置 | 影响 | 建议 |
|---|---|---|---|---|
| 1 | `empty_map.yaml` 被引用但不存在（mapping 模式 nav2 加载它） | bringup_sim.launch.py | map_server 起不来/异常 | M1 补文件或改装配逻辑 |
| 2 | `use_slam` 参数声明后未接任何条件 | bringup_sim.launch.py | 死参数误导 | M1 接逻辑或删除 |
| 3 | `pointcloud_downsampling_sim.yaml` 被引用但不存在 | bringup_sim.launch.py | 死引用 | 删除引用或补文件 |
| 4 | 静态 TF 桥 camera_init→map / body→odom / base_link→base_link_fake | bringup_sim.launch.py | 帧对齐临时手段，正式化前易埋雷 | 按 A4 理清帧设计后收口 |
| 5 | 启动脚本默认 world=RMUL2026 vs launch 默认 RMUL | scripts / launch | 不一致 | 统一默认值 |
| 6 | RMUL2026.pbstream 仅 526B（疑空）；RMUL2026 无 .posegraph | map/ | slam_toolbox 定位用不了、carto 定位可疑 | 重新建图或补产物 |
| 7 | perception 三件套无条件常开；互补滤波只喂 fastlio | bringup_sim.launch.py | 冗余计算/误导 | M1 按角色条件化（A6） |
| 8 | sim/reality 双份配置 | config/ | 改动两遍 | R5 降级 reality |

---

## 7. Backlog（后续慢慢勾选）

- [ ] M0：写"包总表"inventory 文档
- [ ] M1：bringup 拆分 + 配置归属重构 + §6 问题收口
- [ ] M1.5：帧树设计定稿（2D 世界 / 3D 定位的 tf 契约）
- [ ] M2：profiles/roles.yaml + launch 组合矩阵
- [ ] M2.5：边界 msgs 契约槽位落地（先空包 + 文档）
- [ ] M3：感知双支路 + 坡/洞测试场地
- [ ] M3.5：Nav2 体素/净高变体（2.5D 最小改动方案）
- [ ] M4：bench 评测 + 部署清单导出
- [ ] 远期：3D/2.5D 规划变体族（下坡/过洞场景）
- [ ] 远期：仿真加深度相机（真车有深度相机辅助，仿真目前无）

---

## 8. 附录：角色域与现有包速查

| 域目录 | 包 | 一句话职责 |
|---|---|---|
| rm_simulation | hzmi_rm_simulation / livox_laser_simulation_RO2 | Gazebo 场地+机器人 / 仿真 Mid360 |
| rm_driver | livox_ros_driver2 | 读雷达 |
| rm_perception | imu_complementary_filter / linefit_ground_segmentation / pointcloud_to_laserscan | 加工观测（滤波/地面分割/降维）|
| rm_localization | FAST_LIO / point_lio / icp_registration | 里程计+3D建图 / 高频替身 / 3D 重定位(.pcd) |
| rm_navigation | rm_navigation / teb_local_planner / costmap_converter / fake_vel_transform | Nav2 装配件 / 局部规划 / TEB 依赖 / 云台速度补偿 |
| （装配） | rm_nav_bringup | ★ 本次改造对象：总装/配置/资产（见 §3、§4） |
