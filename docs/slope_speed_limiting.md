# 前瞻坡度/台阶限速（过坡脚/台阶前自动减速）—— 实现 + 物理推导 + 整栈 A/B

> **一句话**：感知现在把"车前 1~3 m 走廊里的**坡度/台阶连续量**"按一个用实测标定过的物理模型
> 换成**速度上限**，发成 nav2 原生的 `nav2_msgs/msg/SpeedLimit` 给 `controller_server`
> （它自己转给 MPPI 的 `setSpeedLimit()` 缩放 `vx_max/vy/wz`）⇒ **命令链上没有新节点、
> 不产生 `/cmd_vel`、不停车、不取消目标、没有 watchdog**。整栈 A/B（用户同参 + 目标
> `(-12.64,-0.31)`）：坡脚那一击 **110.8 → 50.8 m/s²**，`map→odom` 两次都没停发、
> LIO 都没发散，ON 也**到点成功**；代价是全程速度 p50 1.23 → 0.71 m/s（`|v|<0.6` 的时长 4.8 → 9.4 s）。
> **这一击是被压低（−54%）而不是被消除**：目标 ≲30 m/s² 还没达到（详见 §0/§3/§7）。
>
> **★ 2026-10-07 提速档**：本文 §3 的"慢"在 `map(RMUC2026_v3_spl)` 这张图上更严重（旧表 **0/4 到点**、
> `|v|<0.6` 占 61~73 s）。已把 `speed_limit_*` 改成 **提速档**（peak 30→45、floor 0.45→0.60、
> 死区 4°→5.5° / 0.04→0.06 m、两张表按同一公式重算），实测 **2/2 到点、`|v|` p50 1.16~1.19 m/s、
> `|v|<0.6` 0.3~0.6 s、平地回归只差 0.2 s**：见 `docs/lio_divergence_no_impact.md` §3/§4
> （那里同时给了"限速≠防撞"的实测证据与回退方法）。

---

## 0. 结论（先看这段）

| 问题 | 答案 |
|---|---|
| 机制落在哪 | `rm_ground_traversability`（header-only）扩展 + 两个地面分割节点各接一次；**单一真源**仍是 `src/rm_nav_bringup/config/traversability_criteria.yaml` |
| nav2 有原生机制吗 | **有**：`nav2_controller` 1.1.20 声明 `speed_limit_topic`（默认 `speed_limit`）并把 `SpeedLimit` 转给控制器插件的 `setSpeedLimit()`；MPPI 的实现在 `optimizer.cpp`：按比例**缩放 `vx_max/vx_min/vy/wz` 四个约束**（`0.0` = `NO_SPEED_LIMIT` ⇒ 恢复基准）。⇒ 用它，**没有自己写 governor、没有新节点** |
| 第一击消除了吗 | **没有，是压低**：`OFF` 首次撞击簇峰值 110.8 m/s² → `ON` 50.8 m/s²（−54%）。目标（≲30 m/s²）**在整栈实测里还没达到**，原因与下一步见 §5/§7 |
| 会不会把车"限停" | 不会：速度地板 `speed_limit_floor_mps: 0.45`（**不许配 0**，校验脚本会拦）；实测 ON 全程 `\|v\|` 的 p50 = 0.71 m/s、`max` = 1.87 m/s、仍然到点 |
| 平地会不会变慢 | 离线合成扫掠：平地（含 5 mm 噪声）**一次都没限速**；整栈回归跑见 §3.3 |
| 回退 | 一个键：`speed_limit_enable: false`（或 `traversability_enable: false` 连判据一起退） |

---

## 1. 机制：从"连续量"到"速度上限"，以及它住在哪

### 1.1 数据流（谁算什么）

```
/livox/lidar/pointcloud ──► 地面分割节点（ground:=linefit|patchwork，**同一个节点内**）
                                │
                                ├─① 判据 applyFrame()：坡度/台阶 ⇒ ground 降级 obstacle（**这次没改**）
                                │     复用：局部地面 g(x,y) = 0.20 m 粗格内各 0.05 m 细格"最低点"的 p05
                                │
                                ├─② describeCorridor()：同一帧、**同一份粗格缓存**上的
                                │     「车前/车后 1~3 m 走廊」逐格连续量：
                                │       · slope_deg  局部地面坡度（复用 slopeOf()，0.40 m 基线）
                                │       · dtan_deg   **坡度变化**（相对最近 0.40 m 的参考坡度）
                                │       · step_m     **台阶残差** = 格内抬升(p90) − 该格坡度能解释的部分
                                │
                                ├─③ SlopeSpeedLimiter::update()：三条规则 → 取最紧 → 刹车距离界 → 速率限制
                                │
                                └─④ 发 nav2_msgs/SpeedLimit（绝对值 m/s；0.0 = NO_SPEED_LIMIT）
                                       │
                         controller_server（订阅 speed_limit_topic）─► MPPI: setSpeedLimit()
                                       │                                  ⇒ 缩放 vx_max/vx_min/vy/wz
                                       └─ /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel（**唯一发布者没变**）
```

**为什么前瞻用"朝向走廊"而不是"沿规划路径"**：这两个地面分割节点**不订阅 `/plan`、不查 TF** —
那是它们"零额外故障面"的前提（`docs/ground_segmentation_slots.md` §2 的契约）。云本身在
**传感器帧**里，而 livox 安装 `rpy = 0 0 0`（`measurement_params_sim.yaml`）⇒ 帧的 +x 就是车头。
要用规划路径做前瞻，就得把点云/路径互转到同一帧 ⇒ 给感知加 TF + 话题依赖；收益只是"转向时少看
一点不相关几何"。⇒ 选**朝向走廊**（半宽 `0.28 m` + 随距离张开 `8°`，覆盖转弯扫掠），
零依赖、零时序问题。

**为什么要按运动方向翻转走廊**：`(见 §5 第 1 条)` 实测两次整栈跑里**最重的一击都发生在倒车时**
（`v<0`）。MID360 在车顶、走廊只朝一个方向 ⇒ 只看车头就完全看不见车尾后面那个坡脚。
现在**只订阅** `/odom` 的速度符号（`speed_limit_direction_topic`，拿不到就回退"只朝前"），
`v<0` 时走廊朝 −x。契约不变：`/cmd_vel` 的发布者仍然只有 `velocity_smoother`。

### 1.2 为什么必须有"已承诺特征 + 距离前推"（这一条是能不能真减速的关键）

只用"特征在 d 米外 ⇒ 现在允许 `sqrt(v_req²+2a·d)`"是**不够**的：雷达**看不见车下与近处地面**
（MID360 下视 −7.22°、离地 0.226 m ⇒ 平地最近看得见 `0.226/tan7.22° = 1.78 m`），
所以测量到的特征距离**永远 ≥1.0~1.8 m** ⇒ 上限永远降不到 `v_req`：离线扫掠里表现为
"过坡脚时仍以 1.2~1.4 m/s 通过"（只把 86 压到 ~70）。做法：

1. 某帧测到"比**当前承诺**更紧"的定位型特征（坡度变化 / 台阶残差）⇒ **承诺**它（记住 `v_req` 与距离）；
2. 之后每帧把剩余距离按 `上一帧上限 × hold_decay_factor(1.0) × dt` **前推**；
3. 前推到 0（车已到/过了它）才释放；放开（升速）受 `release_mps2` 限制。

承诺距离取**该格的远边**（`d + 0.20 m`）：判据说"这一格里出现了跃变"，跃变真实位置在格内。
⚠ 前推用的速度是**发布出去的上限**（≤ 实际车速 ⇒ 方向上保守），代价与残余风险见 §7。

**绝对坡度规则（兜底）为什么必须单独存在、又为什么"只在没有承诺时"参与**：
坡脚可能正好落在盲区里（此时走廊里**全是坡面**、`dtan≈0`）⇒ 没有它就完全不减速；
但一片连续坡面的测量距离每帧都刷新成 ~1.5 m，若让它在承诺生效期间参与，会把已经承诺的减速
顶回 ~1.4 m/s（离线扫掠实测）⇒ 只在"没有承诺"时用。

### 1.3 三条规则（都在 `SlopeSpeedLimiter::update()` 里，共用同一张速度表）

| # | 规则 | 量 | 为什么是它 |
|---|---|---|---|
| ① | **坡度变化** | `atan|Δtanθ|`（相对最近 0.40 m 参考坡度）≥ 4° | 撞击来自**坡度变化**（`a≈v·Δtanθ/Δt`）：恒定坡面上行驶没有垂向速度突变 ⇒ **不该因为"我在坡上"而限速** |
| ② | **台阶残差** | 格内抬升(p90) − 该格坡度能解释的抬升 ∈ [0.04, 0.35] m | 窄尺度：整格内完成的尖台阶不会改变邻格地面 ⇒ 坡度描述子看不见它。>0.35 m 认成墙 ⇒ 交给规划器 |
| ③ | **绝对坡度**（兜底） | 局部坡度 ≥ 2°（且无承诺在生效） | 坡脚落在雷达盲区时的唯一提示 |

**抗噪**（真实点云很稀：单帧走廊只有 7~20 个粗格、集中在一两个距离带）：
· 格内抬升取**细格最高点的 p90**（不是 max，避免单点离群：实测 max 噪声可达 0.15 m）；
· 定位型特征要过**支撑格数**（同一 0.40 m 距离带内 ≥2 格一致；台阶 ≥2× 死区时单格也认）；
· `Δ坡` 死区 4°、台阶死区 0.04 m。

---

## 2. 速度表：物理推导（一个实测锚点标定）

**模型**：刚性轮在坡脚/台阶沿处，垂向速度在接触建立的 `Δt` 内从 0 变成 `v·tanθ`
⇒ `a_peak ≈ v·tanθ/Δt`。

**锚点（实测）**：`docs/path_clearance_and_contact.md` §2.2 —— 车以 `v≈1.7 m/s` 冲上
`map(−3.6,3.1)` 的 ~10% 坡脚，IMU `|a|` **79~90 m/s²**（车静止时是 9.8）。
⇒ 反算 **`Δt_eff = v·tanθ/a = 1.70×0.10/86 = 1.98 ms`**（单一真源里的
`speed_limit_anchor_v_mps/anchor_tan_slope/anchor_peak_mps2`，校验脚本每次都会重算它）。

**目标**：把预期峰值压到 `peak_target = 30 m/s²`（= 实测发散那一击 86 的 1/3；
`docs/path_clearance_and_contact.md` §3.5 的读法：决定成败的是"这一次有没有在坡脚挨那一下"）：

```
v ≤ a_target · Δt_eff / tanθ = 30 × 0.00198 / tanθ = 0.0593 / tanθ     （m/s）
```

| 坡度 θ | 2° | 3° | 4° | 5° | 6° | 8° | ≥10° |
|---|---|---|---|---|---|---|---|
| 表值 (m/s) | 1.66 | 1.10 | 0.83 | 0.66 | 0.55 | 0.45 | 0.45（地板）|
| 模型解 (m/s) | 1.70 | 1.13 | 0.85 | 0.68 | 0.56 | 0.42 | 0.34 |
| **预期峰值** (m/s²) | 29.3 | 29.2 | 29.4 | 29.2 | 29.2 | 32.0† | 40.1† |

† = 已到地板 `0.45 m/s`（机构上还愿意走的最低速）。⇒ **模型意义上的"≤30 m/s²"覆盖到 6~7°**；
更陡的一律走地板。台阶表同理（等效坡度 `tanθ_eff = 台阶 / 0.20 m`）：
`0.04 m`（等效 11.3°）的模型解已是 0.30 m/s < 地板 ⇒ 台阶只有"到地板"一档。

**这条曲线的形状为什么重要**：`v(d) = sqrt(v_req² + 2·brake·d)` 是"以 `brake` 减速就**正好在
`d=0`（特征处）降到 `v_req`**"的运动学反解 ⇒ 特征越近、允许的当前速度越低 ⇒
**是"过坡脚前开始减速"，不是"在坡脚上减速"**。`brake_mps2 = 0.4`（物理极限是
`velocity_smoother.max_decel = 4.0`，10× 余量）；离线扫掠挑的值：

| brake (m/s²) | 0.8 | 0.5 | **0.4** | 0.3 |
|---|---|---|---|---|
| 过坡脚速度 (m/s) | 1.18 | 0.90 | **0.70** | 0.65 |

`0.4` 是"够低"与"别一路爬"的折中。校验脚本断言 `brake ≤ |velocity_smoother.max_decel|`、
`speed_limit_vx_max == min(MPPI vx_max 2.5, velocity_smoother.max_velocity[0] 2.0)`。

---

## 3. 实测（整栈、无头、一次 bash 调用）

跑法（与用户那次同参）：`localization:=gicp nav:=mppi planner:=smac2d lio:=small_point_lio
world:=RMUC2026 spin_speed:=0.0`，目标 `(-12.64,-0.31)`，先验图 = 用户当前
`map/RMUC2026.yaml`（**未改动**）。

```bash
# A/B 基线（限速关）——--kv 只临时改 install 里那份真源，跑完自动还原并 diff 核对
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag ssl_off --domain 181 --port 11881 \
  --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 90 --post-goal-sec 20 \
  --kv "$PWD/src/rm_nav_bringup/config/traversability_criteria.yaml:speed_limit_enable=false"
# A/B 限速开
bash tools/scripts/regress/run_nav_clearance_ab.sh --tag ssl_on3 --domain 184 --port 11884 \
  --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 90 --post-goal-sec 20
# 一次看全部指标（本仓新加的分析器）
python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on3
```

### 3.1 A/B 表

| 指标 | `ssl_off`（限速**关**） | `ssl_on3`（限速**开**） | 读法 |
|---|---|---|---|
| IMU 全程峰值 (m/s²) | 110.8 | 50.8 | −54% |
| IMU>30 / >50 样本数 | 7 / 3 | 1 / 1 | |
| 首次撞击簇 | t=16.2 s，110.8 @map(−3.89,2.56)，车速 −1.04 m/s | t=20.5 s，50.8 @map(−3.94,2.91)，车速 **+0.81** m/s（前进） | |
| `loc_err` max (m) | 0.371 | 0.318 | 两次都**没有**发散 |
| `lio_step` / `mo_step` max (m) | 0.202 / 0.037 | 0.195 / 0.010 | 逐帧步长没有爆 |
| `map→odom` 停发次数 | 0 | 0 | 没有"盲开" |
| 结局 | **到点**（status=4，最近 0.25 m） | **到点**（status=4，最近 0.23 m） | |
| 里程 (m) | 21.54 | 22.18 | |
| 接触/阻塞事件 | 2 / 2 | 1 / 1 | |
| 车速 `|v|` p50 / p90 / max (m/s) | 1.23 / 1.82 / 1.88 | 0.71 / 1.36 / 1.87 | **平地不变慢**的代价项 |
| `|v|<0.6 m/s` 时长 (s) | 4.8 | 9.4 | |
| `/scan` 最小距离 p50 / p05 (m) | 0.778 / 0.370 | 0.628 / 0.341 | 余量没变差 |
| 限速器行为 | 不发 | 362 行 `[slope_speed]`；limit p50=0.85、min=0.45、max=2.00；**不限速占比 9%**；`why` 分布 slope_change 122 / step 197 / slope 12 / none 31；实测 `|cvx|` 与上限之差 p50=−0.12、p95=+0.05（"压住"占 66%） | |

### 3.2 撞击点对照（用户那次的位置）

`docs/path_clearance_and_contact.md` §2.2/§7.5 的撞击点是 `map(−3.6,3.1)`；本次两次跑的首个
撞击簇分别落在 `map(−3.89,2.56)`（OFF）与 `map(−3.94,2.91)`（ON）（ON）—— 同一段坡脚/台面边沿。

### 3.3 平地回归（`tools/scripts/localization/run_nav_smoke_regress.sh`）

同一路线（`--localization gicp --nav mppi --planner smac2d --goal -1.0 2.0`）各跑一次：

| 跑 | 结果 | 用时 | 真值 max\|v\| | 里程 | recoveries |
|---|---|---|---|---|---|
| `ssl_reg_off2`（限速**关**，临时改 `speed_limit_enable: false`，跑完还原+diff 核对） | ✅ PASS | **3.2 s** | 1.54 m/s | 2.88 m | 0 |
| `ssl_reg_on`（限速**开**） | ✅ PASS | **3.6 s** | 1.19 m/s | 2.90 m | 0 |

读法：**两次都 PASS、都没有 recovery**；限速开的那次**峰值速度被压到 1.19 m/s（关时 1.54）**，
用时 +0.4 s（≈+12%，这条路线只有 ~2.9 m）。⇒ 平地**不是"完全不受影响"**，而是"限速器在这条
路线上也看到了低地形并小幅压速"；离线上"纯平地不限速"的性质由
`bench/slope_speed_offline.cpp` 的 `平地（对照）` 一行守着（含 5 mm 噪声也不触发）。
要注意：这条路线的相关性只有 1 次跑，**0.4 s 是单次差值、不是统计量**。

```
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_reg_on --domain 187 --port 11887 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
# OFF 对照：临时把真源里的 speed_limit_enable 改成 false（跑完**必须**还原并核对）
sed -i 's/^\(\s*\)speed_limit_enable: true/\1speed_limit_enable: false/' \
     src/rm_nav_bringup/config/traversability_criteria.yaml
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_reg_off2 --domain 189 --port 11889 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
sed -i 's/^\(\s*\)speed_limit_enable: false/\1speed_limit_enable: true/' \
     src/rm_nav_bringup/config/traversability_criteria.yaml
```

---

## 4. 参数、文件、回退

**单一真源**：`src/rm_nav_bringup/config/traversability_criteria.yaml` 的 `speed_limit_*` 一组键
（两个地面分割节点当 ROS 参数文件读；离线出图脚本**不读**这些键）。校验：

```bash
python3 tools/scripts/regress/check_slope_speed_table.py     # 参数一致 + 表↔模型 + 与 nav2 vx_max/max_decel 对账
python3 tools/scripts/regress/check_traversability_criteria.py  # 判据那一套（没动）
```

| 键 | 默认 | 作用 / 什么时候改 |
|---|---|---|
| `speed_limit_enable` | true | **总开关**（回退用）；false ⇒ 完全不发 SpeedLimit |
| `speed_limit_topic` | `speed_limit` | 必须 == controller_server 的 `speed_limit_topic` |
| `speed_limit_vx_max` | 2.0 | = min(MPPI `vx_max` 2.5, smoother `max_velocity[0]` 2.0)；改 nav2 那两处要同步 |
| `speed_limit_lookahead_m` | 3.0 | 前瞻距离。**再大也没用**：平地最近只看得见 1.78 m（上坡面 ~2.6 m） |
| `speed_limit_corridor_half_width_m` / `_spread_deg` | 0.28 / 8.0 | 走廊半宽/张角（车半宽 0.155 + 余量） |
| `speed_limit_slope_change_deadband_deg` | 4.0 | 坡度变化死区（**噪声大就调大**；调小会误触发） |
| `speed_limit_step_deadband_m` / `_ignore_above_m` | 0.04 / 0.35 | 台阶残差死区 / 上限（上限以上认成墙，交给规划器） |
| `speed_limit_min_support_cells` | 2 | 定位型特征的支撑格数（**点云稀疏时保命**） |
| `speed_limit_brake_mps2` | 0.4 | 刹车距离界（越小越提前减速）；必须 ≤ `velocity_smoother.max_decel` |
| `speed_limit_release_mps2` | 1.0 | 放开（升速）速率；防止"过了坡就全油门" |
| `speed_limit_floor_mps` | 0.45 | **速度地板（不许 0）**：慢也要走 |
| `speed_limit_hold_decay_factor` | 1.0 | 承诺特征的距离前推系数（<1 更保守） |
| `speed_limit_direction_aware` / `_min_vx` / `_timeout_s` / `_topic` | true / 0.05 / 0.5 / `odom` | 按运动方向翻转走廊（倒车时看车尾） |
| `speed_limit_slope_knots_*` / `_step_knots_*` | 见 §2 | 速度表本体 |
| `speed_limit_publish_diag` / `_log_period_s` | true / 2.0 | 诊断话题与 `[slope_speed]` 日志节流 |

**回退（三种粒度）**

1. **只关限速**（判据照旧）：`speed_limit_enable: false` ⇒ 不发任何 `SpeedLimit`（速度回到 nav2 自己的
   `vx_max`/`velocity_smoother`）。也可临时 `ros2 param set /ground_segmentation speed_limit_enable false`——
   注意本节点是**构造期**读参数，运行时 set 不生效，要重启节点或改文件。
2. **连判据一起关**：`traversability_enable: false`（回到"没有坡度/台阶判据"的旧链路）。
3. **残留上限的手工恢复**（本节点被 SIGKILL 时最后一次上限会留在 MPPI 里；正常退出会自己发回
   `NO_SPEED_LIMIT`）：`ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"`

**改动文件一览**

```
src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/low_terrain_classifier.hpp   (+走廊剖面/连续量)
src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/slope_speed_limit.hpp       (新：速度表+限速器)
src/rm_perception/rm_ground_traversability/include/rm_ground_traversability/traversability_ros.hpp      (ROS 接线：发 SpeedLimit)
src/rm_perception/rm_ground_traversability/bench/slope_speed_offline.cpp                                (新：离线合成扫掠，无需 Gazebo)
src/rm_perception/{linefit_ground_segmentation_ros,patchwork_ground_segmentation}/…                     (每帧调用 + nav2_msgs/nav_msgs 依赖)
src/rm_nav_bringup/config/traversability_criteria.yaml                                                  (**真源**：新增 speed_limit_* 键)
tools/scripts/regress/check_slope_speed_table.py                                                        (新：真源/物理/与 nav2 对账)
tools/scripts/regress/analyze_slope_speed_ab.py                                                         (新：A/B 指标提取)
docs/slope_speed_limiting.md（本文）、docs/traversability_plan.md §9                                     (文档)
```

---

## 5. 什么**没有**帮上忙（别重复走）

1. **"标障碍"这条路**（上一轮做的）：坡面**本身可行驶**，把它标成障碍是错的（`traversability_plan.md`
   §1.3 ①）；实测只把第一击 86.4 → 68.2 m/s²。⇒ 必须限速，不能靠 occupancy。
2. **只朝前看的走廊**：两次整栈跑里最重的一击都发生在**倒车**时（`v = −1.04 / −1.00 m/s`），
   车尾后面的坡脚根本不在走廊里 ⇒ 第一版（`ssl_on`）峰值 66.5、`ssl_on2` 67.7、`ssl_on3` 86.5
   —— 不是限速器没生效，是**它压根没看到那一击的来源**。改成**双向窗口**（`ssl_on4`）之后
   才降到 50.8 m/s²。
   · 顺带一条**没走通**的路：先用"订阅 `/odom` 的 `twist.linear.x` 符号来翻转走廊"（`ssl_on3`），
     机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 一直打"前进"⇒ **方向判据
     没有生效，根因未定位**（订阅建了没有？时间戳/时钟？符号约定？本次没查到底）。
     ⇒ 默认改成**双向窗口**（不依赖任何话题语义）；方向判据保留但 `speed_limit_direction_aware: false`。
3. **纯"距离刹车界"（没有承诺/前推）**：因为雷达看不见车下与近处地面，上限永远降不到 `v_req`
   （离线扫掠：过坡脚 1.18~1.4 m/s）⇒ 必须"承诺 + 前推"。
4. **让绝对坡度规则一直参与**：会把已承诺的减速顶回 ~1.4 m/s（离线扫掠复现）⇒ 只在无承诺时用。
5. **不做抗噪**（格内取 max、无支撑格数要求）：第一次整栈跑 `ssl_on` 里限速器**全程都在限速**
   （不限速占比 0%），38% 的运动时间压在 0.6 m/s 以下、90 s 没到点。p90 + 支撑格数 + 更大死区
   之后（`ssl_on2`）到点了，但代价仍在（§3.1 的速度列）。
6. **调 nav2 的全局限速**（`velocity_smoother.max_velocity 2.0 → 1.0`）不是替代方案：它不看地形，
   平地也慢——本机制的意义正是"只在低地形前慢"。

---

## 6. 怎么复跑

```bash
# ① 离线（不需要 ROS/Gazebo）：调速度表/死区时看趋势 + 两条硬性质验收
g++ -O2 -std=c++17 -I src/rm_perception/rm_ground_traversability/include \
    -I /usr/include/pcl-1.12 -I /usr/include/eigen3 \
    src/rm_perception/rm_ground_traversability/bench/slope_speed_offline.cpp -o /tmp/ssl_bench
/tmp/ssl_bench          # 期望：平地"平地误限=否"；坡脚/台阶过特征上限 0.5~0.7 m/s
/tmp/ssl_bench -v       # 逐帧曲线

# ② 真源/物理自检
python3 tools/scripts/regress/check_slope_speed_table.py

# ③ 整栈 A/B（§3 的两条命令）+ 指标提取
python3 tools/scripts/regress/analyze_slope_speed_ab.py .tmp_clear/out/ssl_off .tmp_clear/out/ssl_on3

# ④ 平地回归（一次 bash 调用，含起栈/收尾）
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag ssl_regress --domain 185 --port 11885 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0
```

**看日志**：地面分割节点每帧（变化 >0.03 m/s 或每 2 s）打一条
`[slope_speed] limit=… | 前进/倒车 | why=… d=… Δ坡=… 坡=… 台阶=… v_req=… | 走廊: …`；
诊断话题：`/ground_segmentation/slope_speed_stats`（每帧 JSON，含逐格 `[d, slope, dtan, step]`）、
`segmentation/terrain_slope` / `segmentation/terrain_step`（逐格连续量的点云，intensity 带值）。

---

## 7. 未验证 / 已知残余风险

1. **"第一击消除"还没做到**：整栈实测只把峰值从 110.8 压到 50.8 m/s²（目标 ≲30）。
   最可能的原因（未逐条证实）：走廊里的**坡度/台阶估计仍然很稀**（单帧 7~20 格、集中在一两个
   距离带），4° / 0.04 m 的死区是为了压住噪声而设的，代价是**弱特征不触发**；
   而实测那一击的落点恰好是"台面边沿 + 坡脚"的混合体。
1b. **"拿 `/odom` 的 `twist.linear.x` 符号判运动方向"这条路没走通**：`ssl_on3` 里机器人确实在倒车
   （`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 日志一直打"前进" ⇒ 方向判据没有生效，
   根因**未定位**（订阅建立了吗？时间戳/时钟？符号约定？本次没查到底）。⇒ 改成**双向窗口**这条
   不依赖任何话题语义的修法；方向判据保留但默认关闭（§4/§7）。
2. **承诺特征的前推用"上限"当速度**：仿真里 `planar_move` 完美跟踪 ⇒ 等于真实行进距离；
   实车/被打滑/被挡（车没动但一直发着上限）时会**低估剩余距离、提前释放**。缓解：
   `hold_decay_factor < 1`（更保守）；彻底解需要订阅 `/odom` 的**速度**（本次只用了它的符号）。
3. **只验证了 `nav:=mppi`**：`SpeedLimit` 经 MPPI 的 `setSpeedLimit()` 缩放约束；RPP/DWB/TEB 的
   `setSpeedLimit()` 实现在本仓**没测**（nav2_core 里它是纯虚函数，各插件自己实现，语义可能不同）。
4. **`ground:=linefit` 槽位**：代码路径完全相同（同一个 `TraversabilityRos`），但本次 A/B 只跑了
   默认槽位，**没跑 patchwork 槽**。
5. **只在一张先验图、一个目标、各一次跑**上量的：撞击量级本身是概率性的
   （`docs/path_clearance_and_contact.md` §3.5），单次跑的差值不能当"统计显著"。
6. **`mode:=mapping` 未跑**：新增的是"订阅 + 发 SpeedLimit"，理论上不影响 ground/obstacle 两朵云
   （`applyFrame()` 的判据与降级逻辑一个字没改），但本次**没有**跑 mapping 冒烟。
7. **真车**：所有数字来自 Gazebo（`planar_move` 速度控制、无悬架）。真车有悬架/轮胎 ⇒ 同样的
   坡脚冲击会小很多；但"限速在地形前生效"这件事与平台无关。
8. **`/odom` 方向源在 LIO 发散时会跟着错**：发散时 `twist.linear.x` 的符号可能不可信 ⇒ 走廊可能
   朝错方向。此时兜底是"绝对坡度 + 台阶"两条规则仍在前方走廊上工作（如果方向没错）。
9. **负障碍（坑/落空）**：本文的台阶残差对**下跳**是部分覆盖（格内抬升为负时不算），
   真正的"落空"判据仍是 `docs/traversability_plan.md` §9 里那条空缺。
