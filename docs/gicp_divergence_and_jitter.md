# GICP 重定位的"发散 + 抖动"：复现、机制与修复（2026-10-06）

> 入口现象（用户实跑）：
> `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d`
> → 车到点后"四处抖动"，日志里 **`map→odom` 单调跑掉（y: 0.47 → 5.45 → 21.8 → 62.1 m，z 到 +8.28 m）
> 而 `~/fitness_score` 一直 0.002~0.005 m²（"很好"）、`~/converged` 一直 true**。
>
> 本文是这件事的**实测报告 + 修复说明**：复现了什么、机制是什么、改了什么、A/B 数字、没保留什么、
> 以及**没有验证的东西**。所有数字都能用 §6 的命令复跑（`tools/scripts/localization/` 下的三个脚本 +
> `gicp_registration/bench/gicp_selfsim_probe`）。

---

> 🔗 相关（2026-10-06）：`docs/path_clearance_and_contact.md` —— 用户下一次实跑（`spin_speed:=0.0`）的
> 失败**不是**本文件的"自相似/发散"问题：抖动已修好，GICP 采纳率 100%，
> 是**车在坡脚挨了 8~9 g 的冲击把 LIO 的里程计打散**（定位误差 0.06→2.2 m 用 2.4 s），
> 三道判据随后**正确地**拒收并要求人工复位；报告里还量化了"停发 `map→odom` 之后 nav2 仍在开车"
> 这个安全缺口（盲开 110 m、474 m/s²）。

## §0 结论（一句话 + 五条）

**一句话**：这次失败**不是 GICP"算错了"，而是"没有分辨力"**——本场地的先验地图（大片地板 + 规则重复墙体）
让**任何位姿都能配得很"好"**（实测：偏 0.07~2.5 m 的位姿 fitness 仍 0.002~0.10 m²，远低于接受阈值 0.3），
于是融合没有绝对锚点：上游 LIO 的 odom 一旦出现较大误差（本次实测**最坏 13 m / 甚至失控**），
`map→odom` 与融合位姿就一起跟着假位姿走，nav2 拿到假位姿 ⇒ 车乱走 + 抖。修法有三条（都在本地化节点内）：

1. **`map→odom` 不再逐帧"积分"配准结果**：改为**环路内低增益滤波**（发布值同时是下一帧 GICP 的初值，
   朝最近采纳的测量按 `1-exp(-dt/tau)` 推进，tau=1.0 s）。这条同时治**抖动**（逐样本跳变 ~10 倍↓）
   与**静止随机游走**（~√α 倍↓），而 LIO 的慢漂移（实测 1~3 cm/s）几乎无损。
2. **三道接受判据**（配准自洽 + **运动一致性门限** + **合理性/定义域**）：把"配准自洽但位姿离谱"的帧挡在门外，
   不再需要被 fitness 看见。用户那次跑的 `z=+8.28 m`、`xy=62 m` 会被**第③条直接拦下**（实测拦下过 11 帧）。
3. **连续 25 帧被拒 ⇒ 判"定位失效"：停发 `map→odom`**（沿用"没有初值就不发 TF"的语义），大声 ERROR，
   等 `/initialpose` —— **绝不编造位姿**。实测这条在 LIO 失控的那次跑里正确触发。

**四条"没做到、别误读"**：

* 修复**不能**修上游 LIO 的 odom 发散（本次实测到 odom 自身偏真值 13 m；这是 `small_point_lio`
  在"被 nav2 反复 recovery 甩来甩去"时的失效，与本节点无关，见 §2.3）。本节点能做的是**不跟着它编**。
* 本场地"沿墙方向不可观测"是**地图资产 + 场地几何**的性质，不是参数问题：调 `max_correspondence_distance`
  只能收窄盆地、不能造出信息（§5）。
* A/B 里三次**行驶**跑的导航结局（1 次成功 / 2 次 ABORT）**不能**当作"修复好坏"的判据：那两次都有
  LIO 失控参与，且 recovery 次数 15/17/21 说明这个"场地 + 目标点 + 小陀螺"组合本身就在临界点上。
  真正能对照的是**静止 A/B**（§4.2，单变量、同一二进制）与**逐帧抖动**指标。
* `mode:=mapping` 与其它 `localization` 槽位**未受影响**（判据只在 `GicpNode` 内、只在"采纳/拒绝"这一步；
  见 §3.5 的不变量清单）。

---

## §1 复现的失败（硬数字）

装置：`tools/scripts/localization/run_gicp_nav_ab.sh`（一次 bash 调用 = 起栈 → 记录 → 发目标 → 收尾，
隔离 `HOME=/tmp/gzhome-<tag>` / 非默认 `ROS_DOMAIN_ID` / 独立 `GAZEBO_MASTER_URI` / `unset DISPLAY` /
`nav_rviz:=False` / 收尾发零速）+ `gicp_nav_watch.py`（只读记录仪，50 Hz 采样 `map→odom`、`odom→base_link`、
`/odom_ground_truth`、三路 `/cmd_vel`、`~/fitness_score`）+ `gicp_ab_report.py`（离线指标/画图）。

**命令与用户那次逐字同参**：`world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp
nav:=mppi planner:=smac2d`（`spin_speed` 也未覆盖 = 默认 5.0 小陀螺），目标 `(-12.64, -0.31)`。

| 跑（tag） | 配置 | 目标结果 | 采纳率 | fitness p95 | `map→odom` 逐帧步长 p95/max（行驶） | `map→odom` 相对基线最大偏离 xy/z | 融合位姿逐样本跳变 p95（行驶） | 真值 post 峰峰 yaw | recovery |
|---|---|---|---|---|---|---|---|---|---|
| **a_baseline** | 改动前（无门限/无平滑） | ✅ SUCCEEDED（49 s） | 100% | 0.0029 m² | **0.074 / 0.310 m** | **0.97 / 0.41 m** | **0.164 m** | 1.0° | 15 |
| **b（紧门限）** | 门限 0.8 m/s（cap 0.12 m / 20°/s）+ 平滑 τ=0.6 s | ⛔ ABORTED（30 s） | 82%（后判失效） | 0.081 m² | 0.022 / 0.059 m | 0.77 / 0.72 m | 0.280 m | 25.1° | 17 |
| **c（交付配置）** | 门限 3.0 m/s（cap 0.6 m / 60°/s）+ 平滑 τ=1.0 s | ⛔ ABORTED（34 s） | 87%（后判失效） | — | 0.016 / 0.049 m | 0.36 / 0.20 m | — | — | 21 |

> 两处口径说明（避免误读）：① (b)/(c) 都是**门限 + 平滑**两件套一起开；
> ② (b)/(c) 的导航结局是被**上游 LIO 失控**主导的（§2.3），不是判据本身"拦坏了"——
> 判据在 (c) 里抓到 11 帧越界、并按设计停发 TF。三跑共用的 `--gicp-kv` 覆盖在 (b) 那次因为
> 脚本 bug（`d['gicp_registration']` 键名应为 `/gicp_registration`）**静默没生效**，
> 跑的是当时配置文件里的默认值；脚本已修（`tools/scripts/localization/run_gicp_nav_ab.sh`
> 现在按内容找 `ros__parameters`），(c) 与两组静止跑都核对过 `config.used.yaml`。

**（a）基线复现到的失败签名（与用户那次同形，量级小 60 倍）**：

* `map→odom` 在一次 12.7 m 的行驶里**单调漂到 y=+0.97 m**（开机时是 (0.08, −0.007, −0.148)），
  全程 **100% 被采纳、fitness 一直 0.0022~0.0029 m²** —— **对既有健康判据完全不可见**（这正是用户那次的形状）；
* 逐帧修正步长：静止段 p50 3.4 mm / p90 23 mm / p99 53 mm；行驶段 **p50 8.3 mm / p90 46 mm / p99 155 mm / max 249 mm**
  （10 Hz 采纳率 ⇒ max 相当于 2.5 m/s 的"修正速度"，是 p50 的 30 倍）；
* 融合位姿（= `map→odom ∘ odom→base_link`，**nav2 真正看到的量**）逐样本跳变 p95 **16.4 cm**；
* 机器人**静止**（`cmd_vel` 恒 0、真值位移 4 mm）的 120 s 里，定位误差仍在长：真值对齐后
  融合位姿误差 p50 14.4 cm → max **47.8 cm**（就是"到了点还在飘/抖"）；
* 与真值一致性：行驶段误差 p50 8.6 / p95 19.2 / max 33.4 cm（同段 **LIO odom 自身**误差 p50 7.1 / p95 29.5 / max 43.6 cm
  ⇒ GICP 在**部分**纠正 LIO，但纠不干净）；
* 车的行为：走 12.7 m 的路程实际走了 **29.7 m**（2.3×）、`/cmd_vel` 角速度换向 **35 次**、**15 次 recovery**
  —— 这就是"四处抖动"在运动学上的样子。

**（a）与用户那次证据的对照**（同一坐标系、同一量级）：

| 时刻 | 用户那次 `map→odom` | 本次 (a) 基线 |
|---|---|---|
| 开机静止 | (0.08, −0.006, −0.15)，score 0.0029 | (0.078, −0.007, −0.148)，score 0.0029 |
| 行驶中 | (0.06,0.076) → (0.27,0.472)，score 0.0021 | (0.10,0.02) → (0.23,0.12, z −0.13) |
| 到点 | (0.271, **5.45**, 0.904) | (0.08, **0.31**, 0.13)（max 偏离 0.97 m） |
| 之后 | → (1.86,21.8) → (4.98,**62.1**,8.28)，score 0.0053 | 慢速游走，score 0.0022~0.0027 |

⇒ **同一种失败**（修正量在 fitness 看不见的情况下自行漂移），本次没到 62 m，但**机制与判据缺口完全一致**；
62 m 那一档在本次的 (b)/(c) 里以"被新判据抓住"的形式出现（见 §2.3 的 LIO 失控）。

---

## §2 机制（三块实测证据）

### 2.1 先验地图**没有分辨力**：偏 0.07~2.5 m 的位姿 fitness 一样好

离线探针 `bench/gicp_selfsim_probe`（**与节点同一份代码路径**：`voxelDownsample` + `RegistrationBackend`，
参数同 config；扫描 = 地图在"机器人位姿 + 雷达 FOV（−7.22°~+55.22°）+ 20 m 距离"内的切片，
抽稀到 4000 点、加 3 cm 噪声；**无遮挡 ⇒ 这是自相似性的上界**）：

```
# 目标点 (-12.64, -0.31)，max_corr=1.5 m，扫"初值沿 y 偏 δ"
seed=-3.00  fitness=0.02059 | 相对初值挪动 2.392 m | 离真值 0.665 m
seed=-1.00  fitness=0.00467 | 相对初值挪动 0.807 m | 离真值 0.206 m
seed=-0.25  fitness=0.00333 | 相对初值挪动 0.086 m | 离真值 0.164 m
seed= 0.00  fitness=0.00022 | 相对初值挪动 0.000 m | 离真值 0.0003 m   ← 只有真值本身是"完美"的
seed=+0.25  fitness=0.00202 | 相对初值挪动 0.177 m | 离真值 0.073 m
seed=+1.00  fitness=0.00350 | 相对初值挪动 0.821 m | 离真值 0.181 m
seed=+3.00  fitness=0.00706 | 相对初值挪动 2.583 m | 离真值 0.417 m
# 沿 x 偏（同一次跑）：-3 m → 离真值 2.497 m（fitness 0.1034）；-2 m → 0.829 m（0.0275）
汇总：25/25 收敛；fitness 最大 0.1034 m²；离真值最大 2.497 m
     （节点接受阈值 max_fitness_score=0.3 ⇒ fitness ≤0.3 时这个偏差**看不见**）
```

读法（**这就是机制的核心**）：

* **真值本身是唯一"完美"的极小**（fitness 0.0002），所以 GICP 不是"错"，它只是**没有把真值挑出来的能力**；
* 但只要初值偏出去，结果就落到**另一片极小**上：离真值 0.07~2.5 m，而 fitness 只有 0.002~0.10 m²
  —— **全部远低于 `max_fitness_score=0.3`** ⇒ 单靠 fitness 永远分不出"真值"和"差 0.5 m 的孪生位姿"；
* 于是"每帧采纳 + 用它做下一帧初值"= **在一个连续极小（自相似平台）上随机游走**：
  静止实测 120 s 漂 40 cm（§1），行驶时被 LIO 误差喂大 ⇒ 跳盆地 ⇒ 用户看到的单调跑飞；
* yaw 方向**是可观测的**（同一探针的 yaw 扫描：25/25 收敛、离真值最大 0.035 m ⇒ yaw 误差 < 3°），
  所以失败总是"位置在自相似方向上滑"，而不是"整体转错"。

### 2.2 GICP 的初值来自 LIO，所以 LIO 的误差直接喂进配准

链路：`guess = T_map_odom(发布值) ∘ T_odom←sensor`。里程计误差 e_odom ⇒ 初值偏 e_odom ⇒
按 §2.1，配准结果**可能**落在离真值 0.07~2.5 m 的孪生极小里、且 fitness 依旧"很好" ⇒ 假位姿被采纳并发布 ⇒
nav2 用它规划/控制。**实测**（基线 a 跑，真值对齐后）：

| 阶段 | LIO `odom→base_link` 相对真值 | 融合位姿相对真值（GICP 纠正后） |
|---|---|---|
| 静止 | p50 0.2 cm / max 0.5 cm | p50 7.7 cm（初值本身的固定偏差） |
| 行驶 | p50 7.1 / p95 29.5 / max 43.6 cm | p50 8.6 / p95 19.2 / max 33.4 cm |
| 到点后（车不动） | p50 20.2 / p95 31.9 / max 72.7 cm | p50 14.4 / p95 27.7 / **max 47.8 cm** |

⇒ GICP **确实在纠正** LIO（每段都比 odom 好），但**纠不干净**，且误差随时间累积。

### 2.3 这一次的放大器：**上游 LIO 自己失控**（本次实测）

同参数、同目标的三次跑里，有两次的**根因在 LIO**，而且是"肉眼可见"的失控（记录仪里的 `odom→base_link` 与真值对照）：

* (b)：行驶段 LIO odom 误差 **p50 0.70 m / p95 12.8 m / max 13.3 m**；到点后机器人**静止**时
  odom y 从 9.5 → 16.4 → 12.9 m 自己走（真值一动不动）；
* (c)：post 段 odom 直接数值失控（融合位姿被带到 **10^4 m 量级**），同时 `~/fitness_score` 变成 `nan(无内点)`
  ——因为地图点云在 1.5 m 内**根本找不到对应点**了；
* 两次都有 **17 / 21 次 recovery**（nav2 的 Spin/BackUp），`spin_speed=5.0` 的小陀螺把底盘按 5 rad/s 甩，
  这正是 point-LIO 族最容易失效的工况。

本节点对它的正确反应是**不跟随**：`map→odom` 的候选修正被门限/定义域拦下（(c) 实测 **越界拒绝 11 帧**，
日志原文：`合理性 ✗map→base_link xy=(-14.75, 10.95) 超出地图范围 x[-26.88, 5.33] y[-10.75, 9.66]`），
连续 25 帧后判失效、停发 TF（实测在( c) 里 abort 前 3.5 s 触发）。
**判据**：`grep -c "定位失效" launch.log`、`grep "越界" status.txt`。

> ⚠️ 结论的边界：本节点**修不了** LIO 的失控。这条留给 `lio` 槽位的后续工作
> （`spin_speed:=0.0` 关小陀螺、或换/调 LIO；见 §7 未验证）。

---

## §3 修复（判据 + 阈值 + 依据）

三条判据 + 一条滤波，**全部在 `GicpNode::pointcloudCallback` 的"接受/拒绝"这一步**，不改 GICP 数学、
不改 TF 契约、不加任何节点（无 watchdog）。

### 3.1 ① 运动一致性门限（创新）

* **定义**：`创新 = 本帧配准结果 ⊖ 本帧初值`。本帧初值 = **发布值 ∘ 本帧里程计增量** = "只信里程计、
  GICP 一点都不改"时的位姿 ⇒ 创新就是 **GICP 这一帧想施加的修正量**（平移模长与参考系无关，取
  `guess⁻¹·T_map_sensor` 的平移范数与等效 yaw）。
* **判据**：`创新_xy ≤ clamp(rate·dt, min, max)` 且 `创新_yaw ≤ 同式`；`dt` = 与上次**采纳帧**的点云戳差
  （不用墙钟：点云戳才是"这帧数据属于哪个时刻"，CPU 过载拉长的墙钟会把非法大跳变"合法化"）。
* **阈值与依据**（实测，10 Hz 采纳率）：

| 键 | 值 | 依据 |
|---|---|---|
| `gate_trans_rate` | 3.0 m/s | 机器人能力 MPPI `vx_max=2.5 m/s`；健康跑行驶段逐帧步长 p95=74 mm（=0.74 m/s）⇒ 留 4 倍余量 |
| `gate_trans_step_min` | 0.10 m | 静止段 p90=23 mm、行驶段 p90=46 mm ⇒ 下限只切 p95 以上的尾巴 |
| `gate_trans_step_max` | 0.6 m | 掉帧/长间隔时也不许"一次跳半米以上"（用户那次单步到 5~40 m） |
| `gate_yaw_rate_deg` | 60 °/s | `wz_max=2.5 rad/s=143°/s`；健康跑 yaw 逐帧 p99=1.60°（=16°/s） |
| `gate_yaw_step_min_deg` / `max` | 3° / 15° | 同上；yaw 本身可观测（§2.1），门限只防"整帧转飞" |

* **注意（A/B 结论）**：门限**不负责**压抖动 —— 噪声交给 §3.4 的低增益滤波。把门限收紧到 0.8 m/s
  会把合法修正也拦下（实测拒绝率 18%、连续 25 帧后判失效）⇒ 见 §5.1。

### 3.2 ② 合理性 / 定义域（结果蕴含的 `map→base_link`）

| 键 | 值 | 依据 |
|---|---|---|
| `plausible_margin_xy` | 1.0 m | 本场地 PCD 包围盒 30.2×18.4 m（x[−25.88,4.33] y[−9.75,8.66]），2D 可行驶区 28.4×15.9 m ⇒ 1 m 余量容得下车体探出边缘，又能抓住"跑到 60 m 外" |
| `plausible_z_min/max` | −0.9 / +0.7 m | 实测 `map→base_link` 的 z ≈ **−0.30 m**（地面在 −0.36 一带，地图 z 范围 [−0.59,1.82]）；用户那次 `map→odom` 的 z 涨到 **+8.28** |

判据作用在**结果蕴含的 `map→base_link`**（= `T_map_odom_candidate ∘ T_odom←base_link`，用最新可用 TF），
而不是 `map→odom` —— 因为"车在图里的位置"才是物理量。越界时日志给出**具体是哪个界、差多少**
（见 §2.3 的原文）。

### 3.3 ③ 连续拒绝 ⇒ 定位失效（不编造位姿）

连续 `lost_after_rejections=25` 帧（10 Hz ⇒ 2.5 s）被拒 ⇒ `estimate_valid_=false`（**复用"没有初值就不发 TF"
的既有语义**）、停止配准、ERROR 日志 + 状态行改 ERROR、等 `/initialpose`。
`<=0` = 永不放弃（只沿用上次值）。恢复路径：RViz 2D Pose Estimate 或重启导航栈（`/initialpose` 会清掉
失效态、拒绝计数与滤波状态，并**直接对齐**操作员给的初值）。

### 3.4 ④ 环路内低增益滤波（用户抱怨的"抖动"的正解）

* **做法**：发布值 `T_map_odom_` **同时是下一帧 GICP 的初值**；它朝"最近采纳的测量"按
  `α = 1-exp(-dt/tau)` 推进（平移线性插值 + 旋转 slerp，50 Hz 定时器推进；align 帧末尾也推进一步，
  两条路径共用同一个 `steady_clock` 基准 ⇒ 发布出去的是**同一条连续轨迹**）。
* **为什么滤波必须在环路里**（而不是只平滑输出）：配准的答案在一小片连续极小里跳（§2.1），
  "每帧把测量当新位姿"（增益 1）= 把跳变**积分**进去 ⇒ 静止也会漂。低增益 ⇒ 噪声每帧只注入 α 倍
  ⇒ 抖动 ~1/α、随机游走 ~√α。
* **tau=1.0 s 的依据**：采纳率 10 Hz ⇒ α≈0.095；LIO 慢漂移实测 1~3 cm/s ⇒ tau=1 s 的滞后仅 1~3 cm
  （低于 LIO 自身噪声），而跳变噪声被压 ~10 倍。`smoothing_enable:=false`（或 tau=0）= 逐字节回到改动前。
* **高频运动仍由里程计提供**：本节点只发布 `map→odom`，**绝不碰 `odom→base_link`**（契约不变）。

### 3.5 不变量（改动的边界）

* 只发 `map→odom`（单一发布者）；TF/`~/pose` 的**时间戳契约**（`now+tf_lookahead_sec`）不变；
* `~/fitness_score` 语义不变（仍 m²、无内点 nan）；`~/converged` 现在 = `被采纳 && score ≤ fitness_score_warn`
  ⇒ **新判据拒掉的帧会让 `~/converged=false`**（这正是"让失败可见"的一部分）；
* `/initialpose` 的 handoff/apply 结构、三个回调组、MultiThreadedExecutor 全部不变；
* 新键全部有默认值 + 非法值 WARN 回退；`gate_enable:=false` + `smoothing_enable:=false` = 改动前行为；
* `mode:=mapping` 与 `localization:=amcl/beluga/icp/small_gicp/slam_toolbox/cartographer` 不经过本节点
  （`small_gicp` 槽共用本节点 ⇒ 同样获得这三条判据，见 §4.4）。

---

## §4 A/B（数字）

### 4.1 三次"行驶"跑（同命令、同目标、单变量是"本节点改动"）

见 §1 的表。**判读**：

* **(a) → (b)/(c)：逐帧修正步长与 `map→odom` 偏离都明显下降**（行驶段 step p95 74 mm → 22/16 mm；
  `map→odom` 相对基线最大偏离 0.97 m → 0.77/0.36 m；`@10Hz 累计修正路程` 8.4 m → 1.7/0.84 m）；
* **但导航结局变差**（SUCCEEDED → ABORTED）：这两次跑的 **LIO 自己失控**（§2.3），
  nav2 拿不到可信定位 ⇒ recovery 数涨到 17/21 ⇒ BT 放弃。
  在"上游坏掉"的前提下，本节点的三条判据触发（越界拒绝 11 帧、判失效、停发 TF）——**这是设计行为**，
  但它**不是**"导航成功率"这个指标上的改进；真实收益是"不再把假位姿喂给 nav2"。
* 因此：**不要把 4.1 当作修复的验收**，验收看 4.2/4.3。

### 4.2 静止 A/B（**单变量、同一二进制**：只切 `gate_enable` / `smoothing_enable`）

`--no-goal`：起栈 → 静置 20 s → 记录 150 s（车不动、`cmd_vel` 恒 0、真值位移 mm 级）。
两格用**同一个二进制**，唯一差别是那两个开关 ⇒ 这一组是"抖动/静止漂移"的干净对照。

| 指标（静置 20 s 后，记录 **150 s**） | `s_nofix`（= 改动前） | `s_fix`（门限+滤波） | 倍率 |
|---|---|---|---|
| `map→odom` **逐次更新**步长 p95 / max（m） | 0.0087 / 0.0543 | **0.0005 / 0.0283** | **17× / 1.9×** |
| `map→odom` @10 Hz 重采样步长 p95 / max（m） | 0.0086 / 0.0161 | **0.0005 / 0.0008** | **17× / 20×** |
| `map→odom` yaw 逐次步长 p95（°） | 0.127 | **0.008** | **16×** |
| `map→odom` **累计修正路程**（150 s，m） | **4.085** | **0.247** | **16.5×** |
| 融合位姿逐样本跳变 p95 / max（m） | 0.0048 / 0.0129 | **0.0020 / 0.0027** | 2.4× / 4.8× |
| 融合位姿 yaw 逐样本跳变 p95（°） | 0.073 | **0.048** | 1.5× |
| 融合位姿 vs 真值（起点对齐）p50 / p95 / max（m） | 0.083 / 0.083 / 0.087 | 0.085 / 0.085 / 0.087 | 同（**都没漂**） |
| fitness p95 / max（m²） | 0.00298 / 0.00301 | 0.00298 / 0.00301 | 同（判据不改变配准质量） |
| 采纳率 / 更新率（Hz） | 100% / 9.64 | 100% / 9.66 | 同 |

读法：**"抖动"= 修正量每帧动多少** ⇒ 逐帧步长与"累计修正路程"是主指标（17× / 16.5× 改善）；
融合位姿跳变是 nav2 实际看到的量（2.4~4.8× 改善，剩下的部分来自 **LIO 自身**的 TF 噪声，
本节点不该也不可能替它消掉）。两个静止跑的"相对真值误差"都稳定在 8.5 cm 且**不随时间增长**
——这一点很重要：**改动前那份 4 m 的修正路程并没有变成净漂移**（是均值回复的抖动），
真正的漂移发生在**行驶**时（基线 a 的 `map→odom` 偏离 0.97 m、到点后误差长到 48 cm，见 §1）。

### 4.3 图

三张图（`--plot` 生成，落在 `.tmp_gicp_ab/report/`，**不进仓库**：`.tmp_*` 已被忽略）：

* **`ab_static_zoom.png`**（静止 A/B，**最能说明用户抱怨的那张**）：左 = `map→odom` 逐帧更新点的
  xy 轨迹（±3 cm 放大），`s_nofix` 是一团 ±1 cm 的乱线、`s_fix` 是一条几乎笔直的慢线；
  右 = `|map→odom − 均值|` 随时间。
* **`ab_map_odom.png`**：`a_baseline` 的 `map→odom` 在 12.7 m 行驶里画出一张 1 m 大小的"蜘蛛网"，
  两个静止跑几乎是一个点；右图是 `|map→odom − 起点|`（基线到 0.97 m，改动后 0.09 m 且平）。
* `{a_baseline,s_fix,s_nofix}_traj.png`：每跑的 `map→odom` / 真值 / `cmd_vel` / `fitness` 时间序列
  （虚线 = 事件：目标发出/结果/录制结束）。

### 4.4 既有回归（P0 `nav_smoke_regression.py`，accepted path 不受影响）

`bash tools/scripts/localization/run_nav_smoke_regress.sh --tag regress_gicp --domain 151 --port 11821
--localization gicp --goal -1.0 2.0 --spin-speed 0.0`（无头、单次 bash 调用、`world:=RMUC2026
lio:=small_point_lio nav:=rpp planner:=navfn`）：

* **✅ PASS**：`status=SUCCEEDED`、用时 75.5 s、`d_min=0.0 m`、`recoveries=5`、命令链四跳一致
  （`nav=(0.50,0.75)` → `smooth` → `chassis`）、真值 `pose_delta=2.83 m`、RTF 0.8；
* **新判据零误报**：`[status]` 末行 `采纳 762/762 帧 | 拒绝 门限 0 / 越界 0（连续 0）`、
  `~/fitness_score` 0.0027~0.0031 m²、align 14~17 ms；
* 附带收益：回归自带的"发目标前等 map→odom 稳定"判据从**旧文档里的 3.4 cm/30 s 漂**
  （`docs/algorithm_matrix.md` §遗留-1 记的"GICP 噪声底"）降到 **0.0020 m / 5 s**（`settled=True`）。

> 口径提醒：这条回归的数字**不能**与 `docs/algorithm_matrix.md` 里旧的 "
> PASS / 3.2 s / recoveries=0" 直接比 —— 那次是 `world:=RMUL2026` + 默认 lio/nav，
> 本次是 `world:=RMUC2026` + `small_point_lio`（这个场地 + 这个 LIO 更吃力，recovery 5 次）。
> 要比就同场地同槽位重跑（§6 命令）。

### 4.5 `small_gicp` 槽

`localization:=small_gicp` 与本槽**共用同一个节点**，只是 `backend` 由 launch 注入 ⇒ 三条判据同样生效。
本次**没有**在 `small_gicp` 槽上做行驶 A/B（见 §7）。

---

## §5 没保留 / 没帮助的东西

### 5.1 紧门限（0.8 m/s、cap 0.12 m、20°/s）—— 实测有害，**未保留**

第一版把门限按"健康 p99 × 2"来定（0.8 m/s = 80 mm/帧）。实测（(b) 跑，同上目标）：

* 拒绝率 **18%**（44/242 帧），而且拒的是**合法修正**（典型：`创新 0.1226 m / 0.538°`、fitness 0.0024 m²
  —— 配准自洽、score 与健康帧同级）；
* 于是发布值被"冻住"而里程计继续走 ⇒ 创新越来越大（0.31 → 0.50 → 0.95 → 2.24 m，score 涨到 0.22）
  ⇒ 连续 25 帧 ⇒ 判失效（**死锁**：越拒越偏、越偏越拒）；
* 结论：**门限只该切灾难性跳变，不该管噪声**（噪声交给 §3.4 的低增益滤波）。
  ⇒ 最终 `gate_trans_rate` 3.0 m/s（≈v_max）、`gate_trans_step_min` 0.10 m。

### 5.2 "只平滑输出、初值仍取上一帧测量"—— 实测无效（随机游走照旧）

滤波若不在环路里，测量链本身仍是"增益 1 的积分器"：静止 120 s 漂 40 cm、逐样本跳 16 cm 都在。
⇒ 最终把**发布值本身**作为下一帧初值（`T_map_odom_target_` 只做目标）。

### 5.3 更紧的 `max_correspondence_distance`（1.5 → 0.8~1.0 m）—— 本次**未测**（见 §7）

理论上它能收窄自相似盆地（对应点门限变小 ⇒ 错的配对更少），但：① 会同时减少内点、抬高 fitness 量级，
使既有阈值需要重新标定；② 本次时间用在了"先复现 + 判定根因"上。**未测就不写进默认值**。

### 5.4 没有做的事（有意）

* **不加 watchdog/告警节点**：三条判据都在本地化节点内，是"接受/拒绝"逻辑而不是旁路监控；
* **不动 `map→odom` 之外的东西**：不碰 `odom→base_link`、不碰 nav2 参数、不碰 goal checker 的
  "只看位置不看朝向"语义（§3.5）。

---

## §6 怎么复跑（一条命令一步）

```bash
# 0) 构建（注意：本仓库其它包都是 Release；A/B 必须同构建口径，否则 align 耗时能差 200 倍）
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gicp_registration --cmake-args -DCMAKE_BUILD_TYPE=Release

# 1) 行驶跑（复现用户那次；发目标 → 记录 → 自动收尾 + 发零速）
bash tools/scripts/localization/run_gicp_nav_ab.sh --tag a_run --domain 141 --port 11811 \
     --goal -12.64 -0.31 --post-goal-sec 150 --settle 10
#   单变量：--gicp-kv "gate_enable: false" --gicp-kv "smoothing_enable: false"（= 改动前行为）
#           --gicp-kv "gate_trans_rate: 0.8"（复现 §5.1 的紧门限）

# 2) 静止跑（抖动/静止漂移的干净对照；车不动，记录 150 s）
bash tools/scripts/localization/run_gicp_nav_ab.sh --tag s_fix --domain 145 --port 11815 \
     --no-goal --settle 20 --post-goal-sec 150
bash tools/scripts/localization/run_gicp_nav_ab.sh --tag s_nofix --domain 146 --port 11816 \
     --no-goal --settle 20 --post-goal-sec 150 \
     --gicp-kv "gate_enable: false" --gicp-kv "smoothing_enable: false"

# 3) 指标 + 图
python3 tools/scripts/localization/gicp_ab_report.py --plot \
     --run a_run=.tmp_gicp_ab/out/a_run --run s_fix=.tmp_gicp_ab/out/s_fix \
     --run s_nofix=.tmp_gicp_ab/out/s_nofix --outdir .tmp_gicp_ab/report

# 4) 场地自相似性探针（离线，不需要 Gazebo/nav2；跑 25 个初值 × 3 个方向约 1 分钟）
B=install/gicp_registration/lib/gicp_registration/gicp_selfsim_probe
$B --pcd src/rm_nav_bringup/PCD/RMUC2026.pcd --pose -12.64 -0.31 0 --sensor-z -0.124 \
   --range 20 --noise 0.03 --axis y --sweep -3:3:0.25
#   --scan <real.pcd> 可喂**真实点云**；不给就按"地图切片 + 雷达 FOV"合成（无遮挡 = 自相似上界）

# 5) 判据是否触发的快速检查
grep -a "越界\|定位失效\|创新超门限" .tmp_gicp_ab/out/<tag>/status.txt | head
```

判据/阈值集中在 `src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml` 的
"运动一致性门限 / 合理性 / 输出平滑"一节（逐键都写了依据与回退）。

---

## §7 未验证 / 已知边界

1. **`max_correspondence_distance` 收紧（1.5→0.8/1.0 m）没有实测**（§5.3）。它是最可能进一步收窄自相似盆地的旋钮，
   但会改变 fitness 量级 ⇒ 需要同时重标 `fitness_score_warn` / `max_fitness_score`。
2. **`small_gicp` 槽（`localization:=small_gicp`）没做行驶 A/B**：判据共用、后端不同，
   只做过离线后端的对照（见 `docs/gicp_backend_small_gicp.md`）。
3. **粗到细（coarse-to-fine）/ 全局重定位**没做：本场地的信息缺口不是"初值搜索范围"，而是"没有任何方向能分辨"，
   粗到细解决不了（需要额外的绝对信息源，如反光板/二维码/scan-context 全局检索；仓库里 `docs/scan_context_plan.md` 是那条线）。
4. **`spin_speed` 未做 A/B**：全部行驶跑都用默认 `spin_speed=5.0`（用户那次也是默认）。
   `spin_speed:=0.0`（launch 自己的注释就建议"仿真里排查导航问题先用 0.0"）**未测**，
   而 §2.3 推测 LIO 失控与小陀螺强相关 ⇒ 这是最该补的一组。
5. **三次行驶跑的结局差异很大**（1 成功 / 2 ABORT），**且两次的根因是 LIO 失控**：
   本报告没有把"LIO 失控的概率"量出来（需要多次重复；本文只有 3 次）。
   推测的触发工况（**未验证**）：`spin_speed=5.0` 的小陀螺 + nav2 的 Spin/BackUp recovery。
6. **`lost_after_rejections=25`（2.5 s）没有扫参**：更短会更早停发 TF（更安全、更容易误停），
   更长会给 nav2 更久的假位姿。本文只有 "25 触发过" 这一个事实。
7. **实车未验证**：全部数字来自 Gazebo 无头仿真。实车的 LIO 质量、点云噪声、`odom` 跳变特性都不同；
   门限（3.0 m/s、60°/s）是按"仿真里 MPPI 的 v_max/w_max"标的，实车需按实车能力重标。
8. **`~/converged` 的语义变化未在 nav2 侧验证**：本次没有消费者读它（nav2 不读），
   但若有外部监控依赖"converged=true 表示 GICP 自洽"，语义已变为"自洽 **且** 被判据接受"。
