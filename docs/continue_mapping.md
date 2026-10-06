# 上次基础上继续建图（续建 + 存档 + 场地隔离）

> 面向问题（用户原话）：
> 「能不能就是我在上次基础上继续建，手动指定一个地图名字去覆盖之类的」
> 「以后换地图换场地会不会有干扰」
>
> 一句话结论：**可以。**`mode:=mapping` 起来时若同名存档已存在，slam_toolbox 会
> **反序列化并接着建**（`map_file_name` + `map_start_pose`），走完再 `save` 就是**同名覆盖**；
> 存档旁边有一份 `<名字>.meta.yaml` 记录"它属于哪个 world / 哪个出生点"，
> **换场地续建会被默认拒绝并终止 launch**（除非显式 `map_allow_world_mismatch:=True`）。
> 3D 点云先验（`PCD/<名字>.pcd`）走同一套名字、同一套守卫，所以 3D 先验也不需要手工拼接。

本文替代旧的"分块建图 + 手工拼接"建议（`docs/mapping_small_point_lio.md` §9 第 6 条、
`docs/mapping_2d_from_cloud.md` §7）：**分块照建，但不用拼——每块之间用同一个存档名续上就行。**

---

## 0. 用户操作流程（TL;DR）

```bash
# ── 第一次（或想从零重来）：起栈 ─────────────────────────────────────────────
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox \
  map_name:=RMUC2026_home nav_rviz:=True spin_speed:=0.0
#   ↑ map_name 就是"存档名"。留空 = 取 <world>（这里 = RMUC2026）。
#   日志里会明确写出这次是"从零建图"还是"续建"，以及存档的绝对路径。

# ── 走（遥控 或 脚本化覆盖路线）──────────────────────────────────────────────
tools/scripts/control/improved_teleop.sh          # 键盘；⚠️ 松手必须显式发零速
#   或： python3 tools/scripts/mapping/coverage_drive.py --route <route.json> ...

# ── 存档（另一个终端；栈不用停）────────────────────────────────────────────
tools/scripts/mapping/map_archive.sh save
#   默认就用**本次 launch 的 map_name**（launch 会把会话信息写到 map/.session.yaml）。
#   要存成别的名字： map_archive.sh save --name RMUC2026_home2
#   它做四件事：① **覆盖前先备份**既有存档（<文件>.prev-<时间戳>，默认留 3 代，见 §3.2）
#              ② /slam_toolbox/serialize_map → map/<名字>.{posegraph,data}（同名覆盖）
#              ③ 等两个文件真的落盘，再刷新 sidecar map/<名字>.meta.yaml
#              ④ 若 3D 累加器在跑（cloud_accumulator:=True），顺带存 PCD/<名字>.pcd
#                 （PCD 那一侧的写前备份由累加器自己打印；写完还会再体检一次 z 跨度，见 §3.3）

# ── 关栈 ────────────────────────────────────────────────────────────────────
#   在 launch 的终端 Ctrl+C（SIGINT）。⚠️ 关栈**不会**存图：存档只在 save 那一步发生。

# ── 下次：同一条命令 ⇒ 自动续建（同名覆盖）───────────────────────────────────
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox \
  map_name:=RMUC2026_home nav_rviz:=True spin_speed:=0.0
#   日志（launch 横幅，t=0 就打出来）：
#     [map_archive] ✅ 续建（slam_toolbox 反序列化）：加载 …/map/RMUC2026_home.posegraph
#       · 存档 world=RMUC2026 / 本次 world=RMUC2026
#       · 存档 spawn: x=10.925 y=2.525 z=0.200 yaw=0.000
#   ★ 再等 ~8 s（map_resume_check_delay），会出现**续建一致性检查**的结论（见 §8）：
#     ✅ 续建一致性检查通过 …   —— 或 ——
#     ❌❌ 续建一致性检查**不通过** …（存档很可能是退化过的；别往它上面 save）
#   然后**接着走没走过的区域**，再 save —— 同名覆盖（写前已备份），两张图是同一张。
```

### 0.1 新增的 11 个启动参数（`bringup_sim.launch.py`，默认值 = 现有行为；前 5 个 = 续建/隔离，中 5 个 = 续建后一致性检查 §8，最后 1 个 = 会话播报器 §11）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_name` | `''` | 存档基名；留空 ⇒ 取 `<world>`。存档 = `map/<名字>.{posegraph,data,meta.yaml}` + `PCD/<名字>.{pcd,meta.yaml}` |
| `map_autocontinue` | `True` | 同名位姿图存在 ⇒ 反序列化续建；不存在 ⇒ 从零建（两种情况都在日志里写明路径）。`False` = 永远从零建 |
| `map_start_pose` | `[0.0, 0.0, 0.0]` | 续建时告诉 slam_toolbox「机器人现在在**旧图**的哪个位姿」（x, y, θ，`map` 系） |
| `map_allow_world_mismatch` | `False` | `True` = 显式跳过 world/spawn 隔离检查（**日志会一直提醒**）。只想从零建请用 `map_autocontinue:=False` |
| `cloud_accumulator` | `False` | 仅 `mode:=mapping`。`True` = 起 3D 点云累加器（见 §3），让 `PCD/<名字>.pcd` 也跨会话续建 |
| `map_resume_check` | `True` | **续建后的一次性一致性检查**（见 §8）：比一次 `map→base_link` vs 存档记录的 `map_start_pose`，对不上就喊（`strict` 时收栈）。`False` = 关掉 |
| `map_resume_check_delay` | `8.0` | 检查的静置/收敛窗口（秒）：从"第一次拿到 `map→base_link`"起算，等这么久再采样，让扫描匹配先收敛 |
| `map_resume_check_strict` | `False` | `True` = 检查不通过时**直接收栈**（检查节点退出码 1 ⇒ launch 收掉所有节点）；`False` = 只打响亮 WARNING |
| `map_resume_check_pos_tol` | `0.5` | 位置偏差阈值（m）：健康续建实测在**厘米级**（0.000~0.03 m），0.5 m 留足余量；当天事故的 ATE max 是 1.15 m ⇒ 抓得住 |
| `map_resume_check_yaw_tol_deg` | `10.0` | 偏航偏差阈值（度），同口径 |
| `map_session_announce` | `True` | **会话播报器**（§11）：只发布的 `map_session` 节点，在 latched 话题 `/map_session/info` + 服务 `/map_session/query` 上广播"本次 launch 用了哪个 `map_name`/`world`/存档基名/出生点/`session_id`"，并自检"图上只有一个 `/slam_toolbox` 且它有 `/slam_toolbox/serialize_map`"。`map_archive.sh save` 先问它 ⇒ 名字来自**活栈自己**。`False` = 不起（save 退到活进程/文件证据，仍不猜） |

为什么 `cloud_accumulator` 默认 **关**：它是本仓库新增的一条路径（只读消费者，不发 `/map`、
不发 TF、不发 `/segmentation`，所以对既有契约零影响），但还没有长跑验收 ⇒ 默认不改变任何现有启动集；
要用显式打开。2D 那条（slam_toolbox 续建）默认开，因为它用的就是 slam_toolbox 自己的原生能力。

---

## 1. 机制：续建到底改了什么（源码级依据）

| 环节 | 依据 |
|---|---|
| 存档写出 | `/slam_toolbox/serialize_map`（`<base>.posegraph` + `<base>.data`，见 `slam_toolbox/include/slam_toolbox/serialization.hpp:44-45`） |
| 续建读取 | 给节点传 `map_file_name=<base>`：`SlamToolbox::loadPoseGraphByParams()`（`slam_toolbox_common.cpp:313`）在**节点构造后立刻**反序列化这一对文件并接着建 |
| 从零建 | 同一个函数第 349 行 `if (!filename.empty())`：`map_file_name` 为空 ⇒ **什么都不加载**（这正是"不存在就全新开始"的天然实现） |
| `map_start_pose` | 只在"要反序列化"时有意义：`START_AT_GIVEN_POSE` ⇒ `PROCESS_NEAR_REGION`（`slam_toolbox_common.cpp:836-842`），即"旧图里机器人当前在哪" |
| `map` 系 = 出生点相对系 | 见 `docs/worlds.md` §3~§4：`map` 原点 = 机器人出生点 ⇒ 只要出生点不变，`[0,0,0]` 对任何 world 都正确 |

⚠️ **`map_start_pose` 只在续建时传**：从零建图时传它没有任何意义（slam_toolbox 会忽略）。
`mode:=nav localization:=slam_toolbox` 那条**纯定位**路径有它自己的 `map_file_name`/`map_start_pose` 处理，
本次**一行没动**（见 §5 无回归）。

---

## 2. 场地隔离：为什么必须拒绝，以及拒绝时看到什么

### 2.1 物理依据（这不是洁癖）

本工程的 `map` 系是**出生点相对系**（`docs/worlds.md` §3~§4）：

| world | 出生点（world 系） | `map` 原点 |
|---|---|---|
| RMUC | (6.35, 7.6, 0.2) | 出生点 |
| RMUL / RMUL2026 | (4.3, 3.35, 0.2) | 出生点 |
| RMUC2026 | (10.925, 2.525, 0.2) | 出生点 |

⇒ 同一份位姿图/点云在**不同场地**里天然整体错位；把 A 场地的先验加载到 B 场地 =
把两场比赛的地图叠在一起 ⇒ 假墙、回环错配、之后所有定位/导航全部报废（而且要等到跑歪了才发现）。

### 2.2 sidecar：`<名字>.meta.yaml`

每份存档旁边都有一份 sidecar（由 `src/rm_nav_bringup/scripts/map_asset_guard.py` 写）：

```yaml
name: RMUC2026_home
kind: posegraph            # posegraph | pcd
world: RMUC2026
created_at: '2026-10-06T15:10:22+08:00'
updated_at: '2026-10-06T15:24:03+08:00'
spawn_pose: {x: 10.925, y: 2.525, z: 0.2, yaw: 0.0}
map_start_pose: [0.0, 0.0, 0.0]
resolution: 0.05
frames: {map: map, odom: odom, base: base_link, convention: 'map 系 = 出生点相对系…'}
files: [RMUC2026_home.posegraph, RMUC2026_home.data]
file_sizes: {…}
versions: {ros_distro: humble, packages: {rm_nav_bringup: …, slam_toolbox: …}}
```

### 2.3 判定分级

| 判定 | 条件 | 行为 |
|---|---|---|
| `fresh` | 存档文件不存在 | 从零建（日志写明"没有找到存档 … ⇒ 本次从零建图"） |
| `resume` | sidecar 与当前 `world`+出生点一致 | ✅ 续建 |
| `override` | 不一致，但给了 `map_allow_world_mismatch:=True` | ⚠️ 续建，且日志每处都写"已跳过隔离检查" |
| **拒绝** | world 不同 / 出生点不同 / **sidecar 缺失（旧版存档）** / sidecar 类型不符 / `.data` 缺失 | ❌ **launch 期直接终止**（退出码 1）；3D 累加器则拒绝 load 与 save，且**不覆盖**文件 |
| 软警告 | `resolution` 或 `map_start_pose` 与 sidecar 记录不同 | 继续，但打印 ⚠️（这两项可能是用户在纠正，不一定错） |

**旧版存档**（`map/RMUC.posegraph`、`map/RMUL.posegraph`、`PCD/*.pcd` 这些本仓库既有的、
没有 sidecar 的图）**默认不许被续建** —— 因为无法确认它属于哪个场地。
要把它纳入新体系，只有两条路：显式 `map_allow_world_mismatch:=True`（每次都写），
或者人工担保一次：

```bash
tools/scripts/mapping/map_archive.sh adopt --name RMUC --world RMUC
#   ⚠️ 这个断言只有人能下：脚本无法验证它真的是 RMUC 的图。写错 ⇒ 以后续建就歪。
```

### 2.4 拒绝时长什么样（原文照抄，`map_name:=RMUL2026_iso` 跑在 `world:=RMUC2026`）

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
[map_archive] ❌ 拒绝加载存档（world 隔离保护）
  · 存档名        : RMUL2026_iso（位姿图 posegraph）
  · 存档文件      : /home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026_iso.posegraph（+ .data）
  · sidecar       : /home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026_iso.meta.yaml
  · 存档记录 world: RMUL2026（2026-10-06T15:07:39+08:00 写）
  · 本次 world    : RMUC2026
  · 存档记录 spawn: x=4.300 y=3.350 z=0.200 yaw=0.000
  · 本次 spawn    : x=10.925 y=2.525 z=0.200 yaw=0.000
  · 为什么必须拒绝: 存档记录的 world 与本次 `world:=` 不是同一个场地。
  为什么这事很严重：本工程的 `map` 系是**出生点相对系**（原点=出生点）；
  把 A 场地的位姿图/点云先验加载到 B 场地 = 把两场比赛的地图叠在一起 ⇒
  假墙、回环错配、之后所有定位/导航全部报废（而且要等到跑歪了才发现）。
  四选一（都在**启动命令**上，不用改代码）：
    ① 换个名字（最推荐，各场地各一份存档）：map_name:=RMUC2026_RMUL2026_iso
    ② 删掉或改名这份存档（连同 .data / .meta.yaml 一起）：
         mv …/map/RMUL2026_iso.posegraph …/map/RMUL2026_iso.posegraph.bak
    ③ 本次不要续建、从零开始建图：加 map_autocontinue:=False
    ④ 你确认这份存档确实属于当前 world（例如只是改了出生点、或刚手工搬过来）：
         显式承担风险 ⇒ 加 map_allow_world_mismatch:=True（默认 False）
  存档约定与完整流程见 docs/continue_mapping.md
```

判定放在哪、为什么：**launch 期**（`OpaqueFunction`，t=0 执行），不是另起一个"守卫节点"。
理由：守卫节点只能打印/退出，slam_toolbox 照样会把错的图 load 进来 ⇒ 拦不住；
而 launch 期判定能在**构造节点参数之前**决定"传不传 `map_file_name`"，并且能直接 `raise`
⇒ launch 收尾、退出码 1、消息完整（与文件顶部 `_PackageShareFile` 同一机制）。
`OpaqueFunction` 是 Action，`ros2 launch … --show-args` **不执行** Action
⇒ 存档缺失/不匹配时 `--show-args` 依然正常（§5 实测）。

---

## 3. 3D 先验续建：`cloud_accumulator`（`cloud_accumulator:=True`）

包：`src/rm_perception/cloud_accumulator/`（ament_python，纯 Python + numpy，不依赖 PCL/open3d）。
**为什么放这里**：它本质是感知域里"消费 LiDAR、产出 3D 先验"的节点，与
`patchwork_ground_segmentation` / `pointcloud_to_laserscan` 同域；而"存档/隔离"的资产归属
（`map/`、`PCD/`）在 `rm_nav_bringup`，所以判定逻辑放在
`rm_nav_bringup/scripts/map_asset_guard.py`，**三处共用同一份文本**（launch / 本节点 / shell 脚本）。

它做什么：订阅 LiDAR 点云（默认 `/livox/lidar/pointcloud`，**BEST_EFFORT**；
给 `custom_topic:=/livox/lidar` 且 `livox_ros_driver2` 在时也能吃 Livox `CustomMsg`）
→ 按 TF 变换到 **`map` 系** → 体素下采样（默认 0.10 m，与 `PCD/` 既有资产同口径）→ 累积。

契约（**不变**）：只订阅 + 只查 TF；不发 `/map`、不发任何 TF、不发 `/segmentation`
⇒ 「`map→odom` 单一发布者」「`/segmentation/*` 单一发布者」两条契约逐字不变。

```bash
# 用法：起栈时打开（仅 mode:=mapping 生效）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox map_name:=RMUC2026_home cloud_accumulator:=True

# 状态（点数 / 体素数 / 覆盖 bbox / TF 失败计数；节点自己也每 15 s 打一行 [status]）
ros2 service call /cloud_accumulator/status std_srvs/srv/Trigger
# 存（同名覆盖 PCD/<map_name>.pcd + 刷新 sidecar；map_archive.sh save 会自动带上这一步）
ros2 service call /cloud_accumulator/save std_srvs/srv/Trigger
# 续（也可手工触发；节点参数 autoload 已按 map_autocontinue 自动做过一次）
ros2 service call /cloud_accumulator/load std_srvs/srv/Trigger
# 临时换个名字（下次 save/load 用新名）
ros2 param set /cloud_accumulator map_name RMUC2026_home2
```

守卫语义（与 2D 完全同一份判定）：`load` 与 `save` **都**先核对 sidecar；
不一致时 `success=False` 并把上面那段可操作消息原样返回，**一个字节都不写**
（§5 的隔离测试里核对了被保护 PCD 的 md5 前后一致）。
`autoload` 跟随 `map_autocontinue`：`map_autocontinue:=False` ⇒ 3D 也不加载。

### 3.1 数据卫生：什么进云、什么被丢（2026-10-06 事故后新增）

`cloud_accumulator` 不是告警节点，它是**决定什么进累积云**的清洗层：被丢的帧**整帧不进云**，
被丢的点单独计数；阈值全部是参数（`ros2 param set` 可改），启动横幅与每 15 s 的 `[status]` 行
都会把**阈值 + 全部剔除计数**打出来。

| 规则（参数） | 默认 | 默认值的实测依据 | 计数（`[status]` 字段） |
|---|---|---|---|
| 高度带 `z_band_min` / `z_band_max` | `[-0.5, +1.8] m`，相对 `z_ref_frame:=base_link` 在 `map` 系的 z（取不到按 0） | 健康累积云实测 z ∈ [−0.25, 1.77]（用户 2026-10-06 同场地）、`PCD/RMUC2026_mapped.pcd` 145 万点 z ∈ [−0.53, 1.43]；污染云 z ∈ [−4.31, 23.10]。上界取 **1.8 = 实测最高结构 1.77 + 0.03**（不用 1.5：那会把实测结构顶切掉 27 cm）。两端离群（−2.62 / +5.56）都远在带外 | `高度带滤除=N 点`、`高度带滤空=N 帧` |
| 单帧跳变 `max_frame_step` | `0.50 m` | 机器人速度 ≲1 m/s（覆盖路线 13.9 m / 91 s）+ 10 Hz 点云 ⇒ 健康单帧位移 ≲0.1 m，留 5 倍余量 | `跳变跳过=N 帧`、`最近跳过原因` |
| 速度 `max_speed` | `1.50 m/s` | 同上，1.5 倍余量。帧间隔 > `jump_gate_dt_max`（0.5 s）时单帧闸无意义 ⇒ 只看速度 | 同上 |
| 转角 `max_frame_yaw_deg` | `25°/帧` | 仿真原地自转 ~1 rad/s ⇒ 10 Hz 下 5.7°/帧，取 4 倍余量；帧间隔大时改用 `max_yaw_rate_dps:=90°/s` | 同上 |
| TF 退化帧 `allow_tf_fallback_frame` | `False`（**默认跳过**） | "带戳查询失败、退化到最新"= 位姿来路不明，不该进先验。实测一次健康无头长跑：TF 失败 30 次，其中 5 次走了退化路径 ⇒ 这 5 帧不再进云 | `TF失败=N(退化最新=M/其中跳过=K)` |

**运动闸的细节（容易误解，写清楚）**：跳变帧只丢**这一帧**，参考位姿仍推进到最后一次观测
⇒ 一次跳变不会把后面所有帧都判成跳变（否则退化开始那一下会吃掉整段）。

**已知边界（诚实登记）**：以上都是**位姿层面**的判据 —— 慢漂（例：1.15 m 用 15 s 漂完 =
0.077 m/s）**看不出来**，因为只靠位姿无法与"机器人真的慢慢走"区分。抓得住的是：跳变、突变、
TF 来路不明的帧、高度上离谱的点。慢漂只能靠 ① `save` 时的 bbox 体检（§3.3）与 ②
`tools/scripts/mapping/pcd_stats.py` 的 z 分位数体检。

### 3.2 覆盖前备份 + 恢复（2026-10-06 事故后新增）

`save` 一直是**同名覆盖**。现在它**写之前**先把既有产物复制一份带时间戳的备份：

- 命名 `<文件名>.prev-<YYYYmmdd-HHMMSS>`（例 `RMUC2026.posegraph.prev-20261006-161530`），
  **默认留最新 3 代**，更老的自动删。为什么不用固定 `.bak`：`save` 会被反复调用，固定名只有一代，
  连续两次坏 save 就把好存档挤掉；时间戳可留多代，代价只是磁盘（一份 ~64 s 存档 = 5.4 MB posegraph
  + 0.6 MB data ⇒ 3 代 ≈ 18 MB；PCD 侧一份 3~18 MB ⇒ 3 代 ≈ 9~54 MB）。
- **谁写谁备份**（避免双重备份/两套命名）：`map_archive.sh save` 备 `map/<名字>.{posegraph,data,meta.yaml}`；
  累加器的 `~/save` 备 `PCD/<名字>.{pcd,meta.yaml}`。两侧时间戳天然不同。
- 一整套一起备、一起恢复（位姿图侧 3 个 / PCD 侧 2 个）——单独恢复 `*.data` 没有意义。
- **恢复命令**（`tools/scripts/mapping/map_archive.sh`）：

  ```bash
  map_archive.sh backups --name X                              # 现存几代（新的在前）
  map_archive.sh restore --name X                              # 回滚：两侧各取本组最新一代
  map_archive.sh restore --name X --from 20261006-164418       # 指定某一代（也可给 *.prev-* 路径）
  map_archive.sh restore --name X --dry-run                    # 只看会做什么
  map_archive.sh backup  --name X --kind posegraph             # 手工备份一代（save 内部调的就是这条）
  ```
  回滚前会把**当前**文件也留一代 ⇒ **restore 本身可逆**；只回滚备份过的文件，不会删任何东西；
  退出码 `5` = 没有可用的备份代。
- ⚠️ 位姿图侧与 PCD 侧是**两个写者、两个时刻** ⇒ `restore` 按**组**对齐（各自取本组最新一代），
  而不是"全局最新一个时间戳"。2026-10-06 实测：按全局最新只恢复了 PCD 侧、位姿图被漏掉
  （证据 `.tmp_hygiene/out/real1/run.txt`）⇒ 已改成按组对齐。
- 恢复出来的 sidecar 与图/云**同代** ⇒ 守卫看到的仍是一套自洽的（不会出现"图是旧的、sidecar 是新的"）。

### 3.3 `~/save` 落盘前的健康告警（**仍然照写**）

`~/save` 在写之前体检当前累积云的 bbox：

- `z 跨度 > save_warn_z_span`（默认 **3.0 m**）或 `XY 跨度 > save_warn_xy_span`（默认 40 m，故意很松）
  ⇒ 打**响亮 WARNING**，点名数字、完整 bbox、剔除计数，并建议**改名另存**
  （`ros2 param set /cloud_accumulator map_name <名字>_bad` 后再 save）或回滚备份。
- 阈值依据：健康云 z 跨度 1.96~2.02 m，污染云 27.41 m（`PCD/RMUC2026_cont.pcd`）/ 8.00 m（合成测试云）
  ⇒ 3.0 m 落在两侧都 ≥1.5 倍间隔处。
- **为什么只喊不拦**：① 拒绝落盘会把"想留一份坏数据取证/对比"的路堵死；② "别场地"这种硬冲突已经由
  守卫（§2.3）负责拒绝；③ 用户明确要求"仍然写、但要说清楚"。
- `map_archive.sh save` 在写完之后还会用 `tools/scripts/mapping/pcd_bbox_health.py` **独立**再量一次
  （同口径阈值，退出码 3 = 不合理）——因为节点的告警在**另一个终端**的日志里，而这一步的输出里
  也该有数字。

---

## 4. 目录与"存了却不生效"这个坑（**必读**）

`colcon build --symlink-install` 下 `install/rm_nav_bringup/share/rm_nav_bringup/map/` 是一个**真目录**，
里面每个文件是**指向 `src` 的单独软链**。⇒ 运行期新建的 `map/<名字>.posegraph`
**不会**自动出现在 install 里（要重编才会有软链）。
如果 launch 从 `share/…/map/` 读、脚本往 `src/…/map/` 写，就会出现
**"明明存了，下次却当没存"**这种最难查的静默失败。

处理：`map_asset_guard.py::canonical_asset_dir()` 统一解析"到底该读写哪个目录"——
只要发现 share 目录里有指向别处的软链，就以那份**真实目录**（= 仓库 `src/rm_nav_bringup/{map,PCD}`）为准；
非 symlink-install（整份拷贝）时退回 share 目录本身。launch、节点、shell 脚本**都走这一个函数**，
所以"存档写哪、下次读哪"永远一致。核对命令：

```bash
tools/scripts/mapping/map_archive.sh dirs
# map_dir=/home/…/HzMi_rmsimulation/src/rm_nav_bringup/map
# pcd_dir=/home/…/HzMi_rmsimulation/src/rm_nav_bringup/PCD
```

另外 launch 会把"本次会话用了什么名字/哪个 world/存档目录"写到 `<map_dir>/.session.yaml`
（git 忽略）。⚠️ **2026-10-06 事故后这个文件的地位变了**（详见 §11）：它会被**任何后启动的
launch 覆盖**，所以 `map_archive.sh save` **不再**"读它就当名字"，而是：
① 显式 `--name` ＞ ② ROS 图上的**活会话播报器**（`/map_session/info` / `/map_session/query`）＞
③ 活着的 `bringup_sim.launch.py` 进程命令行（看不到进程时退到 ③b：活映射器自报的存档基名）＞
④ `.session.yaml` + 把它钉在活栈上的证明 ＞
⑤ **拒绝**（列出查了什么、给 `--name` 等出路，退出码 3）。
写 `.session.yaml` 失败只警告、不影响建图（那种情况下 save 会走 ②/③，都不可用就必须 `--name`）。

---

## 5. 实测（2026-10-06，全部无头：`HOME=/tmp/…`、非默认 `ROS_DOMAIN_ID`、
`unset DISPLAY`、独立 `GAZEBO_MASTER_URI`、`nav_rviz:=False`）

命令（run A / run B **完全同一条**）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox nav_rviz:=False lio_rviz:=False spin_speed:=0.0 \
  map_name:=RMUC2026_cont cloud_accumulator:=True
```

| 指标 | run A（从零） | run B（续建） |
|---|---|---|
| launch 横幅 | `没有找到存档 … ⇒ 本次从零建图` | ✅ `续建（slam_toolbox 反序列化）：加载 …/RMUC2026_cont.posegraph` |
| 3D 横幅 / 节点自述 | `没有 PCD/… ⇒ 从零点云开始累积` | ✅ `续建（3D 先验点云）`；`autoload: 已续建 3D 先验…读入 36964 点；体素 0 → 36964` |
| 行驶区域（脚本路线） | 出生点 → 北 2.1 m → 西 2 m → 南 4.2 m | 出生点 → 南 2.8 m → 东 → 南 → 沿 x≈2 北 5.6 m（**不同象限**） |
| 行驶距离 / 墙钟 | 10.14 m / 63.9 s（路线走完，5/5 航点） | 13.90 m / 91.0 s（7/9 航点，到时停止） |
| LIO vs 真值 | ATE RMSE **0.043 m**、max 0.117、轨迹长度比 1.015 | ATE RMSE 0.267 m、max 1.15、比 1.906（**末段退化**，见下） |
| **`/map` 起点** | occ=281（w=228 h=284，origin −8.392,−7.821） | **occ=1728、w=263、h=293、origin −10.157,−8.174** = run A 结束时的同一张图 |
| **`/map` 终点** | occ=**1704**、w=263、h=293、known 31259（78.1 m²） | occ=**2268**、w=283、h=307、known 35715（89.3 m²；相对起点 **+4234 格 ≈ +10.6 m²**，其中占用格 +540） |
| 占用格 bbox（map 系） | x[−9.41, 2.99] y[−8.12, 5.43] | x[−9.41, **3.99**] y[−7.67, **6.83**]（+x/+y 方向长出新区） |
| 存档 `map/RMUC2026_cont.posegraph` | 5,674,161 B（新写） | **6,351,884 B**（同名覆盖，+677,723 B） |
| 存档 `.data` | 588,399 B | **1,262,032 B**（+673,633 B） |
| 3D `PCD/RMUC2026_cont.pcd` | 36,964 点 / 1.13 MB | **109,665 点 / 3.35 MB** |

**结论**：run B 的 `/map` 就是 run A 那张图（起点 occ 1728 vs run A 终点 1704，差的是加载后
驱动脚本采样前那几秒的新观测），并在 **+x/+y** 方向长出了 run A 没走过的区域；
存档是**同一个名字被覆盖**（文件变大、不是新建第二份）⇒ **不需要手工拼接**。

> 本次实测留下的存档**没有入库**（untracked）：`map/RMUC2026_cont.{posegraph,data,meta.yaml}` +
> `PCD/RMUC2026_cont.{pcd,meta.yaml}`（约 11 MB）。想直接体验续建可以用它：
> `… mode:=mapping world:=RMUC2026 map_name:=RMUC2026_cont`（日志会打"✅ 续建"）；
> 不想要就按 §7 的第 4 条删掉。

⚠️ **诚实登记**：run B 末段（sim 59→74 s）LIO 退化（ATE max 1.15 m）⇒ 那十几秒的点云被
按错的位姿累积进 PCD，3D bbox 出现离群（z 到 23.10 m、y 到 19.30 m，而正常场地 z ∈ [−0.3, 1.8]）；
**2D 图没受影响**（occ/bbox 都正常）。3D 先验的干净程度 = LIO 的干净程度 ⇒
用之前先体检：`python3 tools/scripts/mapping/pcd_stats.py PCD/<名字>.pcd`（看 z 分位数与 bbox）。

### 5.1 隔离验收（同一次无头运行，脚本 `.tmp_cache/contmap/iso.sh`）

| 检查 | 结果 |
|---|---|
| 造一份 sidecar 写 `world: RMUL2026` 的存档，用 `world:=RMUC2026` 续建 | ❌ 拒绝，退出码 **1**，日志里 `拒绝加载存档` 出现 1 次，**被我起始的进程数 = 0**（拒绝发生在起任何节点之前） |
| 同一场景下 `ros2 launch … --show-args` | ✅ 退出码 **0**（70 个参数，含 `map_allow_world_mismatch`）—— 守卫不挡 `--show-args` |
| `map_allow_world_mismatch:=True` | ✅ 起栈成功；两条横幅都写明"跳过了 world/spawn 隔离检查"；3D 也加载了那份 PCD（体素 109,665） |
| 累加器单独跑（`autoload:=True`，同一份不匹配 PCD） | ❌ `autoload` 打 ERROR（同一段消息）；`~/load` 与 `~/save` 都 `success=False` |
| 被保护的 `PCD/RMUL2026_iso.pcd` | md5 前后一致（`b2eb4d8256bb`）⇒ **没有覆盖** |
| 累加器 `allow_world_mismatch:=True` | ✅ 加载成功（日志带"已按 map_allow_world_mismatch 跳过隔离检查"） |
| SIGINT 关栈退出码 | **0**（与改造前一致） |

### 5.2 无回归

| 检查 | 结果 |
|---|---|
| `--show-args` | 退出码 0（存档缺失 / 存档不匹配两种场景都测了；70 个参数） |
| `mode:=nav`（`world:=RMUL2026 localization:=amcl lio:=fastlio`）节点集 | **13 个节点**，与改造前的 launch 文件（`HEAD~3` 版本）逐个数一致；`amcl=active[3]`、`map_server=active[3]`、`/map` 发布者恒 1 个（`map_server`；订阅者是 amcl / global_costmap）、`tf2_echo map odom` 正常（amcl 在发） |
| `/map` 发布者（mapping） | 恒为 1 个：`slam_toolbox`（`ros2 topic info /map -v`，run A/B 都核过）；节点表里没有 `map_server`/`amcl`/`lio_tf_adapter` ⇒ `map→odom` 也只有 `slam_toolbox` |
| **SIGINT 关栈退出码**（同一套仪器：`setsid bash -c 'ros2 launch …; echo $? > rc'`，pgrep 取真实 pgid 送 SIGINT，并核对日志里 `signal_handler(SIGINT)` 出现次数证明信号送达） | **mapping + 当前 launch：退出码 0，≈3 s 关完**（`signal_handler` 7 次）。**nav：改造前/后完全一样** —— 两次都是"90 s 内没退完"（`signal_handler` 各 17 次）⇒ 这是 nav2 lifecycle deactivate + Gazebo 的**既有**行为，不是本次改动引入；真要快速收尾用 `kill -9`（本仓库其它脚本也一直是这么兜底的） |
| 默认值不变 | 5 个新参数都有默认值；不传时启动集与旧代码逐字相同（唯一行为差别：**同名存档存在**时改成续建） |

> 注：上面 nav 那条最初被我误判成"改造引入的关栈不退出"，原因是脚本用 `setsid … & ; $!` 取 pgid，
> 在 `setsid` 会 fork 的情况下拿到的是已死 wrapper 的 pid ⇒ **SIGINT 根本没送出去**。
> 改成"pgrep 找真正的 ros2 launch 进程 + 退出码写文件"后才拿到可信数字（表里就是这个版本）。

### 5.3 2026-10-06 三项修复的验收（脚本与原始输出都在 `.tmp_hygiene/`，未入库）

**① 数据卫生（合成点云，无需 Gazebo；`.tmp_hygiene/filter_test.sh`）**
合成源每帧 440 点：200 点地面（map z≈0.05）+ 200 点墙（z≈0.5~1.8）+ **20 点 z≈+5.5** + **20 点 z≈−2.5**
（照着当天污染云的 ± 值造），另加**一帧单帧跳变 2.0 m** 与**一帧 stamp 打到未来 1 s**（带戳 TF 必失败 ⇒ 只能退化到最新）。

| 指标 | A 旧行为（卫生全关） | B 新默认 |
|---|---|---|
| 累积 bbox | x[2.36,9.54] y[−1.98,2.00] **z[−2.50,5.50]**（z 跨度 **8.00 m**） | x[2.37,9.54] y[−1.94,2.00] **z[0.05,1.78]**（z 跨度 **1.73 m**） |
| 累积点 / 帧 | 28600 = 65 帧 × 440 | 24800 = 62 帧 × 400 |
| 高度带滤除点 | 0 | **2480 = 62 帧 × 40 点**（正好是注入的离群点，一个不多一个不少） |
| 跳变跳过帧 | 0 | **1**（那帧 2.0 m 跳变） |
| TF 退化帧 | 退化最新=1 / 跳过 **0**（照旧累积） | 退化最新=1 / 跳过 **1** |
| `~/save` 结果 | **⚠️ 健康告警触发**（z 跨度 8.00 m > 3.00 m；仍照写） | ✅ 无告警 |

**② 覆盖前备份 / 恢复（`.tmp_hygiene/backup_test.sh` + `.tmp_hygiene/restore_regression.sh`）**
真点云两代（污染代 z 跨度 8.00 m vs 干净代 1.73 m，哈希与 bbox 都能一眼分辨）+ map/ 侧合成的
`.posegraph/.data/.meta.yaml`：破坏 live 后 `restore` ⇒ **5 个文件全部逐字节恢复**（sha256 一致，
含 sidecar）；restore 前会把被破坏的那一代也留档 ⇒ **restore 本身可逆**；`keep=3` 轮转生效。
两侧在不同时刻备份时，`restore` 按**组**各取本组最新一代（这正是当天实测踩到的坑，见 §3.2）。

**③ 续建后一致性检查（合成 7 场景 `.tmp_hygiene/resume_check_test.sh` + 真机无头两跑）**

| 场景 | 结果 |
|---|---|
| 健康 + 静止 | ✅ 通过（0.000 m / 0.00°） |
| 健康 + **一起来就开走**（1 m/s） | ✅ 通过（修正量 0.000 m；**不误报**） |
| 起点被挪 1.5 m（doctored 存档） | ❌ 报警（绝对偏差 1.500 m） |
| 图歪了 + 开走（每帧被拉 0.08 m） | ❌ 报警（修正量 6.480 m） |
| 图歪了 + 静止（每帧被拉 0.10 m） | ❌ 报警（修正量 6.200 m） |
| `strict=True` + doctored | 退出码 **1** ⇒ launch 收栈（真机：`process has died … exit code 1` + 整栈被收） |
| `strict=True` + 健康 | 退出码 **0**（不误收栈） |

真机无头（`mode:=mapping lio:=small_point_lio mapper:=slam_toolbox cloud_accumulator:=True`，
续建一份 `RMUC2026_cont` 的**测试改名副本**）：健康续建 ⇒ ✅ 通过（偏差 **0.021 m / 0.03°**，无任何
WARN/ERROR）；从零建图 ⇒ 日志里 `map_resume_check` 出现 **0** 次（只在真的续建时才创建）；
SIGINT 关栈退出码 **0**。

**④ 无回归（真机无头 A/B）**：`--show-args` 退出码 0、**75** 个参数（原 70 + 新 5）；
`mode:=nav`（amcl+fastlio）用 **HEAD 版 launch** 与**改后 launch** 各跑一次 ⇒ 节点清单**逐行一致**
（34 行，仅 `transform_listener_impl_<随机后缀>` 辅助节点名字不同）；mapping 下 `/map` 发布者恒 1
（slam_toolbox）、`/segmentation/obstacle` 与 `/segmentation/ground` 各恒 1 个发布者、
节点表里没有 `amcl`/`map_server`/`lio_tf_adapter` ⇒ `map→odom` 仍只有 slam_toolbox；
检查节点与累加器**都不创建任何 publisher**（源码级核对：两个文件里 `create_publisher` 出现 0 次）。

---

## 6. 命令行速查（`tools/scripts/mapping/map_archive.sh`）

```bash
map_archive.sh save [--name X] [--world W] [--no-cloud] [--allow-world-mismatch]
#   ⓪ **名字先严格解析**（§11）：--name ＞ 活会话播报器 ＞ 活 launch 进程 ＞ .session.yaml+活性证明
#      ＞ 拒绝（退出码 3，列出证据链与出路；**不写任何文件**）。多会话（两套栈）一律拒绝
#   ① 守卫先查（场地不一致 ⇒ 退出码 3，**不写任何文件**）
#   ② /slam_toolbox/serialize_map 写 map/<名字>.{posegraph,data}（同名覆盖）
#   ③ 等两个文件都出现（默认最多 60 s）后刷新 sidecar
#   ④ 3D 累加器在跑 ⇒ 顺带 save PCD
map_archive.sh save --check            # 只打印"会用哪个名字 + ①~⑤ 证据链 + 会做什么"，**不写盘**
map_archive.sh save --name X --allow-cross-session
#   X ≠ 活栈会话名时默认**拒绝**（跨会话写盘 = 另起一份）；这条 = 明确要"另存一份"
map_archive.sh info [--name X]      # sidecar + 每个文件的大小/时间戳（2D 与 3D 都打）
map_archive.sh list                 # map/ 与 PCD/ 下所有存档 + world 摘要（旧版会标"无 sidecar"）
map_archive.sh adopt --name X --world W   # 给旧版存档补 sidecar（人工担保）
map_archive.sh dirs                 # launch/脚本实际读写的那两个目录
# ★ 2026-10-06 新增（覆盖前备份 / 恢复，详见 §3.2）
map_archive.sh backup  --name X [--kind posegraph|pcd|all] [--keep 3]
#   手工备份一代（save 内部调的是同一条 guard 命令）：<文件>.prev-<时间戳>
map_archive.sh backups --name X     # 列出各代（新的在前；没有 ⇒ 退出码 5）
map_archive.sh restore --name X [--from <时间戳|*.prev-* 路径>] [--dry-run]
#   回滚：位姿图侧与 PCD 侧各取本组最新一代；回滚前把当前文件也留一代 ⇒ 可逆
```

> `info`/`backups`/`backup`/`restore` **不写存档**，而且常常在栈关掉之后跑 ⇒ 它们走"宽松取名"
> （`--name` ＞ `$MAP_NAME` ＞ `.session.yaml`（会打"这文件可能过期"的警告）＞ 活节点参数）。
> 只有 `save` 走严格解析 —— 严格解析需要活栈，`restore` 时需要栈已经死了。

退出码：`0` 成功；`2` 用法/环境问题（比如没有 `/slam_toolbox/serialize_map`）；
`3` **拒绝**（场地不一致，或会话名无法确认 / 多会话 / 跨会话）；`4` 落盘超时；
`5` `restore`/`backups` 找不到可用的备份代。

---

## 7. 回滚

```bash
# 1) 只想关掉续建（保留新参数、保留存档）：加一个参数即可，什么都不用改
ros2 launch … map_autocontinue:=False

# 2) 关掉 3D 累加器：不加 cloud_accumulator:=True 就不会起（默认就是关）
# 3) 完全回滚本次改动（代码层）
git revert <本次提交…>
colcon build --symlink-install --packages-select rm_nav_bringup cloud_accumulator

# 4) 删掉本次产生的存档/会话文件（不影响既有资产）
rm -f src/rm_nav_bringup/map/<名字>.{posegraph,data,meta.yaml} \
      src/rm_nav_bringup/PCD/<名字>.{pcd,meta.yaml} \
      src/rm_nav_bringup/map/.session.yaml

# 5) 只想让某份存档"重新可用于续建"而不删它：改名即可（守卫按名字找文件）
mv src/rm_nav_bringup/map/X.posegraph src/rm_nav_bringup/map/X.posegraph.bak   # .data/.meta.yaml 同步

# 6) 关掉/放宽"续建后一致性检查"（§8）
ros2 launch … map_resume_check:=False                 # 完全不要这个检查
ros2 launch … map_resume_check_delay:=20.0            # 让它等更久（机器人起得慢/扫描匹配慢）
ros2 launch … map_resume_check_pos_tol:=1.0           # 你的场地确实允许 1 m 级偏差时的放宽

# 7) 存档被写坏了：回滚到上一代（§3.2；两侧各取本组最新一代，可逆）
tools/scripts/mapping/map_archive.sh backups --name X
tools/scripts/mapping/map_archive.sh restore --name X
tools/scripts/mapping/map_archive.sh restore --name X --from 20261006-164418   # 指定某一代

# 8) 关掉 3D 数据卫生的某一层（默认全开；排查"是不是过滤太狠"时用）
ros2 param set /cloud_accumulator z_band_max 3.0      # 放宽高度带上界
ros2 param set /cloud_accumulator max_frame_step 0.0  # 0 = 关掉单帧跳变闸
ros2 param set /cloud_accumulator allow_tf_fallback_frame true   # 退化帧照旧累积（不推荐）
```

---

---

## 8. 续建后的**一次性**一致性检查（`map_resume_check*`，2026-10-06 事故后新增）

### 8.1 它补的是守卫补不了的那一格

守卫（§2）只管 **"world / 出生点对不对"**；它管不了 **"图本身歪了"**：当天用户续建的
`map/RMUC2026.posegraph` 里**已经含有一段 LIO 退化**（ATE max 1.15 m）的位姿 —— world 对、出生点也对，
但加载进来的图与真实场地不一致 ⇒ 建出来的图/定位"飘"（详见 §9）。

> **互补关系（写清楚）**：§2 的场地/出生点隔离守卫 + §8 的续建一致性检查是**两道不同的门**，
> 谁也不能替代谁。守卫在 **launch 期**拦"拿错场地的存档"（硬拒绝、退出码 1）；
> 本检查在**起来之后**量"这份存档与现实是否自洽"（默认只喊不拦、`strict` 才收栈）。

### 8.2 它是什么、放在哪、为什么放那

- **一次性启动体检**，不是持续 watchdog：拿到结论就退出（不留常驻进程）；**不发布任何话题、不发布任何 TF**
  ⇒ 「`map→odom` 单一发布者」契约逐字不变。
- 实现：`src/rm_nav_bringup/scripts/map_resume_check.py`（装到 `lib/rm_nav_bringup/`，由 launch 用
  `Node(package='rm_nav_bringup', executable='map_resume_check.py')` 起）。
- **只在"真的续建"时创建**（`verdict['may_load']` 为真才会往 launch 里加这个 Node）；从零建图时
  日志里连它的名字都不会出现（§5 无回归里核过：`map_resume_check` 出现次数 = 0）。
- 为什么不在别处：
  · **不能放 launch 期**（`OpaqueFunction`）：t=0 时既没有 TF 也没有 slam_toolbox，硬等会**推迟所有节点启动**；
    而本节点由 launch 在 t=0 与其它节点**并行**起，自己的定时器负责等 ⇒ **不推迟任何节点的启动时序**。
  · **不能塞进 slam_toolbox**：那是上游包，且"体检"与"建图"职责不同。
  · 也不做成持续监控 / 不发任何修正（发 `map→odom` 会直接破坏单一发布者契约）。

### 8.3 判定口径（两条判据，缺一不可）

采样时机：**第一次拿到 `map→base_link`**（= slam_toolbox 反序列化完并开始发 TF）起算，等
`map_resume_check_delay`（默认 8 s）的静置/收敛窗口再判。原因：刚 resume 时 slam_toolbox 把位姿设成
`map_start_pose`（自我一致、偏差 0）；真正暴露问题的是**随后扫描匹配按"（可能是歪的）图"把位姿拉走**的量。

| 判据 | 定义 | 什么时候参与判定 | 阈值 |
|---|---|---|---|
| ① **修正量** | 实测 `map→base_link` vs「基线位姿 ⊕ 自基线以来的里程计位移」= 自基线以来 `map→odom` 修正了多少 | **总是**参与（不需要机器人静止） | `map_resume_check_pos_tol`=0.5 m / `yaw_tol_deg`=10° |
| ② **绝对偏差** | 实测 `map→base_link` vs **存档 sidecar 记录的 `map_start_pose`** | 只在「自基线以来**一步没动**」时参与 | 同上 |

为什么必须两条：

- 只用 ② ⇒ 用户"一起来就开走"会被误报（实测假偏差 **7.5 m**，见 `.tmp_hygiene/out/resume_drive_healthy_strictfalse.log`）；
- 只用 ① ⇒ 存档 sidecar 记的起点与现实不符（起点被挪 / 存档被 doctored）时，扫描匹配**不会产生任何修正**
  （① ≈ 0），抓不住 —— 这时只有 ② 能发现。

阈值来源：健康续建的实测偏差是**厘米级**（真机无头实测 `map→base_link` 与记录起点差 **0.021 m / 0.03°**），
0.5 m / 10° 是给"扫描匹配收敛 + 里程计补偿"留的余量；而当天事故的 ATE max = 1.15 m ⇒ 0.5 m 抓得住。
`map_resume_check_delay` 也可以调大（机器人起得慢/场地纹理差时）。

### 8.4 看到什么（真机无头实测原文，2026-10-06）

健康续建（**不误报**，`strict=False`）：

```
✅ 续建一致性检查通过：加载的位姿图与出生点一致
  · 实测 map→base_link : x=-0.000 y=0.003 z=-0.021 yaw=-0.03°
  · ① 修正量（期望=基线x=0.000 y=0.006 z=-0.006 yaw=-0.01° ⊕ 里程计位移）= x=-0.000 y=0.003 z=-0.021 yaw=-0.03°
       偏差 0.000 m / 0.00°（阈值 0.50 m / 10.0°）⇒ ✅ 在阈值内
  · ② 与存档记录 map_start_pose['0.000', '0.000', '0.000'] 的绝对偏差 = 0.021 m / 0.03°（阈值同上）⇒ ✅ 在阈值内
  · 机器人状态  : 自基线以来一步没动（自基线以来最大位移 0.016 m；静止判据 0.050 m）
```

存档与现实不一致（`strict=False` 只喊；`strict=True` 追加"收栈"并把整栈收掉）：

```
❌❌ 续建一致性检查**不通过**：加载进来的位姿图与出生点/现实对不上，这份存档很可能是**退化过的**
  · 判定依据：实测起点与存档记录的起点差 1.497 m / 0.02°（= 存档的 map_start_pose 与现实不符）
  · ① 修正量 … 偏差 0.000 m / 0.00° ⇒ ✅ 在阈值内
  · ② 与存档记录 map_start_pose['1.500', '0.000', '0.000'] 的绝对偏差 = 1.497 m / 0.02° ⇒ ❌ 超阈值
  建议（按优先级）：
    ① 本次不要 save 到这个名字：换一个新名字另存（map_name:=X_new / ros2 param set …）
    ② 这份存档先留着别动，用备份回滚到上一代：map_archive.sh restore --name X
    ③ 顺手体检 3D 先验的 z 跨度：python3 tools/scripts/mapping/pcd_stats.py PCD/X.pcd
    ④ 确认是"起点猜错"而不是"图歪了"：检查本次 map_start_pose 是否真的等于机器人在旧图里的位姿
    ⑤ 只想先跑起来、不要这个检查：加 map_resume_check:=False
（strict=True 时还会打：map_resume_check_strict:=True ⇒ **收栈**（退出码 1）…；
  launch 侧同时打：`[map_resume_check] ❌ 续建一致性检查不通过（退出码 1）⇒ map_resume_check_strict:=True，收栈。`）
```

### 8.5 怎么关 / 怎么调

```bash
ros2 launch … map_resume_check:=False              # 完全不要这个检查
ros2 launch … map_resume_check_strict:=True        # 不通过就收栈（CI/比赛前一晚自检推荐）
ros2 launch … map_resume_check_delay:=20.0         # 收敛窗口加长（默认 8 s）
ros2 launch … map_resume_check_pos_tol:=1.0        # 位置阈值放宽（默认 0.5 m）
ros2 launch … map_resume_check_yaw_tol_deg:=20.0   # 偏航阈值放宽（默认 10°）
```

其余细分参数（`still_window` / `still_eps` / `settle_timeout` / `tf_wait_timeout`）在节点上，
可用 `ros2 param set /map_resume_check …` 或参数文件覆盖。

---

## 9. 事故复盘：2026-10-06「续建后地图发飘」

### 9.1 事件顺序（用户实际遇到的）

| # | 发生了什么 | 证据 |
|---|---|---|
| 1 | 上一次（run B）建图中，**sim 59→74 s 一段 LIO 退化**（ATE max **1.15 m**） | §5 的 run B 行；`docs/continue_mapping.md` §5 的"诚实登记" |
| 2 | 那十几秒的点云被**按错的位姿**累积进 `PCD/RMUC2026_cont.pcd` ⇒ 3D 先验被污染（z 到 23.10 m、y 到 19.30 m，健康应是 z ∈ [−0.3, 1.8]） | `PCD/RMUC2026_cont.pcd` 实测 bbox；`pcd_stats.py` |
| 3 | **同一段退化也被写进了 `map/RMUC2026.posegraph`** ⇒ 存档里的几何与真实场地不一致 | 用户续建后 `map→base_link` 与真实出生点对不上（"发飘"） |
| 4 | 用户从 `map/RMUC2026.posegraph` **续建**：`cloud_accumulator` autoload 打出 `bbox z[-2.62, 5.56]`（健康云是 `z[-0.25, 1.77]`） | 用户当天的日志原文 |
| 5 | **`save` 同名覆盖** ⇒ 坏数据把好存档换掉了（当时的 `save` 没有任何备份） | 本次改动前的 `map_archive.sh` / 累加器 `~/save` 都是直接覆盖 |
| 6 | 结果：续建的图/定位发飘；`map/RMUC2026.{posegraph,data}` 被父进程删除、`PCD/RMUC2026.pcd` 从合成备份恢复 ⇒ 用户稍后重新建图 | 事故处置记录 |

### 9.2 四个修复各自拦住哪一步

| 修复（本次） | 作用位置 | 拦住的是第几步 |
|---|---|---|
| ① 累加器**数据卫生**（高度带 + 跳变/速度/转角闸 + TF 退化帧跳过，§3.1） | 3D 点云**进云之前** | 第 2 步（把退化段的点挡在云外；当天那朵 `z[-2.62, 5.56]` 会被高度带大量剔掉，突变帧整帧丢） |
| ② **落盘前 bbox 体检**（`~/save` 与 `map_archive.sh save` 各一次，§3.3） | 3D 先验**写盘之前** | 第 2/4 步的发现环节：`z 跨度 8.18 m > 3.0 m` 会当场打响 WARNING 并建议改名另存（不会再"静默"留一份坏先验） |
| ③ **覆盖前自动备份 + `restore`**（§3.2） | 写盘**之前** | 第 5 步：即使真的存了坏数据，好存档还在 `*.prev-<ts>` 里，一条 `map_archive.sh restore --name X` 就能回到上一代 |
| ④ **续建后一次性一致性检查**（§8） | 续建**起来之后 ~8 s** | 第 3/4/6 步：图本身歪了的话，`map→base_link` 与实际起点对不上 ⇒ 立刻响亮 WARNING（`strict` 时直接收栈），并明确提示"别往这份存档上 save / 换个 map_name / 体检 `PCD/<名字>.pcd` 的 z 跨度" |

### 9.3 仍然防不住的（别把这几条当成万能）

- **慢漂**（≲1.5 m/s 的位姿漂移、单帧步长 ≲0.5 m）：位姿层面与正常运动无法区分 ⇒ 只能靠
  `save` 时体检 + `pcd_stats.py` 事后体检（§3.1 的"已知边界"）。
- **一直不 save 的重建**：三个修复都在"进云/写盘/续建"三个点上，如果用户从不 save、也不续建，
  它们不会说话（那时问题本来就只影响本次会话）。
- **2D 图本身**：本次的一致性检查只看 `map→base_link` 与存档记录是否自洽；2D 栅格的质量仍要靠
  `/map` 的 occ/bbox 与 `pcd_to_nav2_map.py` 的体检（`docs/mapping_2d_from_cloud.md`）。

---

## 10. 未验证 / 已知坑

1. **`lio:=fastlio` / `pointlio` 没跑续建验证**：本次只验了 `lio:=small_point_lio`。
   机制上续建与 LIO 无关（只跟 `map_file_name` 和 `map/odom` 有关），但**未实测**。
2. **`mapper:=cartographer` 不支持续建**：cartographer 的存档是 `.pbstream`（另一套
   `load_state_filename`/`load_frozen_state` 机制），本次**没有**接入本守卫；
   用 `mapper:=cartographer` 时这 5 个参数里只有 `cloud_accumulator` 与 `map_name`（3D 名字）生效。
3. **`mode:=slam_nav` 下也会走同一套续建判定与一致性检查**（同一个节点/分支），但**没有单独跑验收**
   （§8 的验收全部在 `mode:=mapping` 下做的）。
4. **`resolution` 改变只报警不拦**：同一份存档被 0.05 → 0.10 的栅格续建时，
   旧图与新帧不在同一栅格上，质量自负（日志里有 ⚠️）。
5. **3D 先验的干净程度仍取决于 LIO**（§5 的 run B 末段就是反例）。2026-10-06 已补上
   "高度带 + 跳变/速度/转角闸 + TF 退化帧跳过"（§3.1），但它们都是**位姿层面**的判据
   ⇒ **慢漂看不出来**（1.15 m 用 15 s 漂完 = 0.077 m/s，与"机器人真的慢慢走"无法只靠位姿区分）。
   慢漂只能靠 `save` 时的 bbox 体检（§3.3）与 `pcd_stats.py` 的 z 分位数。
6. **存档体积**：一次 ~64 s 的建图 = 5.4 MB posegraph + 0.6 MB data。
   长跑（十几分钟、多回环）会长到几十 MB，**别顺手 `git add`**（`RMUC.posegraph` 13 MB /
   `RMUC.data` 8.5 MB 就是历史教训）。存档是否入库由你决定，本仓库的 `.gitignore` **没有**忽略
   `map/*.posegraph`。
7. ~~**多机/多栈同名**：`.session.yaml` 只记"最近一次 launch" ⇒ 多栈请显式 `--name`。~~
   **2026-10-06 已改**（§11）：`save` 现在**不再**从 `.session.yaml` 直接取名字；它先问图上的
   活会话播报器（`/map_session/info`），并且**检测到两套栈就直接拒绝并列出**（带 `--name` 也拒绝 ——
   同一 ROS 域里两个同名 `/slam_toolbox`，服务 `/slam_toolbox/serialize_map` 无法指定目标）。
   两套栈要同时存在 ⇒ 用不同 `ROS_DOMAIN_ID`。
8. **`adopt` 是人工担保**，脚本无法验证真伪；写错 sidecar ⇒ 以后续建就歪。
9. **没做**：`map_allow_world_mismatch` 的"只警告不拦"中间档；同名存档的**完整版本历史**
   （现在只有覆盖前自动留的 3 代 `*.prev-*`，见 §3.2）；`restore` 的"恢复后自动跑一次守卫验收"。
10. **会话播报器只在 `mapper:=slam_toolbox` 的 mapping/slam_nav 分支里起**（它和 `.session.yaml`
    是同一处写的）。`mapper:=cartographer` 或 `bringup_real.launch.py` 那条路径**没有**播报器
    ⇒ `save` 会退到"活 launch 进程命令行 / `.session.yaml`+活性证明"，两条都不成立就必须 `--name`
    （这两条路径本来也没有 `/slam_toolbox/serialize_map`，`save` 对它们本来就不适用）。
11. **另一个 PID namespace 里的进程看不见**：`save` 的"活 launch 进程命令行"这条证据靠读 `/proc`，
    而本仓库的沙箱 bash（`bwrap --unshare-pid`）每条命令一个 PID namespace。用户自己的终端里
    （同一 namespace）没这个问题；万一遇到，`save` 会**拒绝**（不会猜）——那时用 `--name` 或靠播报器。

---

## 11. 事故复盘 ②：2026-10-06「save 把 31.89 MB 位姿图写到了**别人那套会话**的名字上」

> 与 §9 是**两次不同的事故**（§9 = 图本身被 LIO 退化污染；本节 = 名字来源不可信）。
> 本节 = 这次修复的完整口径：解析顺序、拒绝文案、`--name` 纪律、多会话规则、验收证据。

### 11.1 事件顺序（用户实际遇到的）

| # | 发生了什么 | 证据 |
|---|---|---|
| 1 | 18:08 用户起自己的建图栈：`map_name:=RMUC2026_v2`（launch 把会话写进 `map/.session.yaml`） | 用户操作记录 + `map/RMUC2026_v2.*`（18:00 那份存档被续建） |
| 2 | 18:34~19:09 并发的**自动化测试**连续起了多套栈（`run_dropab*.sh` → `setsid ros2 launch … map_name:=RMUC2026_dropab_ab_*`），每起一套都**覆盖**同一份 `map/.session.yaml` | `tools/scripts/mapping/sltune/run_dropab.sh:135`；`map/RMUC2026_dropab_ab_*.posegraph` 时间戳 18:34/18:55/19:00/19:14 |
| 3 | 19:09:40 最后那次测试会话写下的状态是 `map_name: RMUC2026_dropab_ab_g_long / session_pid: 23936`；它跑完就退了（pid 23936 不在） | 当时 `.session.yaml` 的内容；`RMUC2026_dropab_ab_g_long.posegraph` = 33,440,928 B = **31.89 MiB**（19:14） |
| 4 | 用户（自己的栈还活着）跑 `map_archive.sh save`：旧代码**直接**读 `.session.yaml` 取名字，唯一的活性检查是 `kill -0 23936` ⇒ 只打了一句 `⚠️ 会话状态里的 launch pid=23936 已经不在了 ⇒ 下面这个名字可能来自上一次会话` 就**继续写盘** | 旧版 `map_archive.sh:86-103`（本仓库历史版本） |
| 5 | `/slam_toolbox/serialize_map` 把**用户自己那套栈**的活图写进了 `map/RMUC2026_dropab_ab_g_long.*` —— 名字错了，但图是真的 | 该文件 33,440,928 B；用户随后把它另存/改名为 `RMUC2026_good.*`（同尺寸） |

**两个根因**（都被这次修复直接针对）：
- **(a) 名字来自一个"谁后启动谁覆盖"的可变文件**，且没有任何"它属于**活栈**"的验证；
- **(b) 唯一的活性检查是 pid**：`kill -0` 既能**假阳性**（pid 回收 ⇒ 早已结束的会话"看起来还活着"），
  又能**假阴性**（记录的常是**包装进程** —— `setsid ros2 launch … &` 的 `$!`、`timeout`、外层脚本；
  或 launch 主进程先走而子节点还活着。§5.2 已记过 `setsid`/`$!` 这个坑）。

### 11.2 新的名字解析顺序（`map_asset_guard.py::resolve_session()`；`save` 走的就是它）

| 优先级 | 证据 | 说明 |
|---|---|---|
| ① | `--name X` | 人担保；打印为"显式"。若 `X` ≠ 活栈会话名 ⇒ **仍然拒绝**（除非 `--allow-cross-session`） |
| ② | **ROS 图上的活会话播报器** | launch 起的 `map_session` 节点：latched（transient_local）话题 `/map_session/info` + 服务 `/map_session/query`，负载（JSON，`schema=rm_nav_bringup/map_session@1`）含 `map_name` / `world` / `archive_base` / `map_start_pose` / `resumed` / `started_at` / **`session_id`** / `launch_pid` / `mapper_nodes` / `serialize_present` / **`verified`**。它随本次 launch 生、随本次 launch 死 ⇒ **谁也覆盖不了**。`verified=True` 的条件（广播那一刻实时核对）：图上 `/slam_toolbox` **恰好 1 个** 且 它提供 `/slam_toolbox/serialize_map`，且只看到 1 个 `map_session` |
| ③ | 活着的 `bringup_sim.launch.py` 进程命令行 | 读 `/proc/<pid>/cmdline` 取 `map_name:=` / `world:=`（`mode:=mapping|slam_nav` 才算建图栈；要求恰好 1 个，且图上映射器唯一、有 serialize 服务） |
| ③b | **活映射器自己报的存档基名** | 看不到 launch 进程时（组合 launch / 进程在另一个 PID namespace）退一步：`slam_toolbox` 的 `map_file_name` 基名 —— 那是**它正在续的那份存档**，同样钉在活栈上（从零建图的会话没有这个值 ⇒ 这条自然失效） |
| ④ | `.session.yaml` **+ 把它钉在活栈上的证明** | 三条证明**至少一条**：〔a〕活 launch 进程自己的 `map_name` 就是它；〔b〕`slam_toolbox` 的 `map_file_name` 基名就是它；〔c〕记录的 `session_pid` 活着 **且** `/proc/<pid>/cmdline` 与记录里的 `launch_cmdline` 一致 **且** 进程起始时刻与 `started_at` 一致（pid 单独**不算**证据） |
| ⑤ | 都不成立 | **拒绝**（退出码 3），打印 ①~⑤ 五行证据链 + 三条出路；**不写任何文件** |

**交叉核对（要求 2）**：无论名字从哪来，只要 `slam_toolbox` 的 `map_file_name` 有值（= 本次是续建），
它的**基名必须等于解析出来的名字**，否则拒绝 —— 那意味着"会话状态"和"实际加载的图"不是一次启动的产物。

**多会话规则（要求 5）**：出现下列任一情况 ⇒ 拒绝并列出（**`--name` 也不例外**）：
两个同名 `/slam_toolbox` 节点、两个 `/map_session` 播报器、两份 `session_id` 不同的播报、
两个 mapping/slam_nav 形态的 `bringup_sim.launch.py` 进程。出路写在拒绝文案里：
关掉多余的那套 / 两套栈各用各的 `ROS_DOMAIN_ID` / 先关一套再 `--name X`。

### 11.3 拒绝文案（原文，`save` 与 `save --check` 都会打；完整版见 `.tmp_mapfix/logs/`）

```text
[map_archive] ❌ 拒绝存档：检测到**多于一套**在跑的建图栈 —— 现在写盘可能写到你没在看的那一套上。
[map_archive]    · 同名映射器 /slam_toolbox 有 2 个（ROS 图里同名节点，服务 /slam_toolbox/serialize_map 也无法指定目标；param get 同样二义）
[map_archive]    · 活会话播报器 /map_session 有 2 个（= 同时起了两套会写 .session.yaml 的栈）
[map_archive]    · 收到 2 份不同的会话播报（session_id 不同）
[map_archive]    · 活会话：map_name=… session_id=… / map_name=… session_id=…
[map_archive]   ⇒ 绝不替你猜。三条出路：
[map_archive]      ① 只留一套栈（关掉多余的那套）后重跑 save；
[map_archive]      ② 两套栈各用各的 ROS_DOMAIN_ID（各自的 save 只看得到自己那一套）：
[map_archive]           ROS_DOMAIN_ID=<n> tools/scripts/mapping/map_archive.sh save
[map_archive]      ③ 明确知道在写谁：先关掉一套，再 save --name X。
```

```text
[map_archive] ❌ 拒绝存档：**没有活着的建图栈**（拿不到任何活性证明），.session.yaml 里的名字无法证明属于谁 ⇒ 不写任何文件。
[map_archive]    查了什么（都不是"活"的）：
[map_archive]      ① /map_session 播报器（/map_session/info / /map_session/query）: 没有
[map_archive]      ② 活着的 bringup_sim.launch.py 进程        : 0 个
[map_archive]      ③ /slam_toolbox 唯一且提供服务 /slam_toolbox/serialize_map: /slam_toolbox ×0，服务 不在
[map_archive]      ④ .session.yaml         : map_name=RMUC2026_dropab_ab_g_long session_pid=23936 started_at=…（session_pid=23936 不在 /proc（进程已退出，或它只是包装进程的 pid，或它在一个看不到的 PID namespace 里））
[map_archive]   ⇒ 这就是 2026-10-06 事故的形态（文件是上一次/别人那次会话留下的）。
[map_archive]      请：① 起栈后重跑 save（推荐）；或 ② 显式 save --name X（你担保这个名字）
```

```text
[map_archive] ❌ 拒绝存档：**没法把 .session.yaml 钉在活栈上** —— 它可能是被更晚的 launch 覆盖过的（2026-10-06 事故就是这一条）。
[map_archive]    · .session.yaml      : map_name=… session_pid=… started_at=…
[map_archive]    · 活着的 launch 进程 : …
[map_archive]    · slam_toolbox      : map_file_name=…
[map_archive]    · 证明失败的原因    :
[map_archive]        - …
[map_archive]   ⇒ 没有写任何文件。三条出路：① save --name <活栈的 map_name>；② 重启栈；③ 删掉 .session.yaml 后重启 launch
```

```text
[map_archive] ❌ 拒绝存档：**--name 指定的名字 ≠ 正在跑的那套栈的会话名**（跨会话写盘 = 另起一份，而不是"覆盖同一个名字"）。
[map_archive]    · --name            : X
[map_archive]    · 活栈会话名         : Y（来源=live-helper，session_id=…）
[map_archive]    · 活栈的 map_name 参数: …
[map_archive]   ⇒ 没有写任何文件。两条出路：① --name Y；② 确实要另存一份 ⇒ 加 --allow-cross-session
```

另外，当名字**成功**来自活证据、而 `.session.yaml` 与它不一致时（= 事故形态），每次都打：

```text
[map_archive]   ⚠️⚠️ .session.yaml 说 map_name=RMUC2026_dropab_ab_g_long（session_pid=23936，started_at=…），
                与活栈（来源=live-helper）的 RMUC2026_v2 **不一致** ⇒ 这份文件已被更晚的 launch 覆盖过，已忽略它（2026-10-06 事故就是这个形态）
```

### 11.4 `--name` 纪律 与 日常用法

- **默认什么都不用给**：`map_archive.sh save` —— 名字来自活栈自己（②/③），并会打印证据链。
- **想先看一眼再写**：`map_archive.sh save --check`（等价 `--dry-run`）：打印证据链 +
  "会用哪个名字 + 会做什么"，**一个字节都不写**（连备份都不做）。
- **显式名字**：`save --name X`。它**始终优先**（打印 `session_source=explicit`），但：
  · `X` ≠ 活栈会话名 ⇒ 拒绝（跨会话），除非加 `--allow-cross-session`；
  · 那代表"另存一份"而不是"覆盖同一个名字" ⇒ 下次续建要用 `map_name:=X` 才会读它。
- `$MAP_NAME` 仍可用（等同 `--name`，会打一行说明）。
- `info` / `backups` / `backup` / `restore` 走**宽松取名**（`--name` ＞ `$MAP_NAME` ＞ `.session.yaml`（带
  "这文件可能被覆盖"的警告）＞ 活节点参数）—— 因为它们不写存档，而且常常在**栈关掉之后**跑
  （`restore` 时栈必须已经死了，严格解析在那种场景下必然失败）。

### 11.5 会话播报器是什么、放在哪、为什么放在那

| 项 | 值 |
|---|---|
| 文件 | `src/rm_nav_bringup/scripts/map_session_announcer.py`（装到 `lib/rm_nav_bringup/`，`install(PROGRAMS …)`） |
| 节点名 | `map_session` |
| 话题 | `/map_session/info`（`std_msgs/String`，JSON 负载，QoS = RELIABLE + TRANSIENT_LOCAL(depth 1) ⇒ latched，晚来的 `save` 立刻能收到） |
| 服务 | `/map_session/query`（`std_srvs/Trigger`，`message` = 同一份 JSON；`success` = `verified`） |
| 起它的地方 | `bringup_sim.launch.py` 的 `_launch_slam_toolbox_mapping()`（与 slam_toolbox 同一个 `OpaqueFunction`，`map_session_announce:=True` 默认开） |
| 为什么不会影响建图 | 它是**只发布**节点：不发 `/map`、不发 TF、不订阅任何话题、不写任何文件、不改既有节点集/时序（slam_toolbox 仍在 `TimerAction(4.0)` 之后起，一行没动） |
| 拿不到它会怎样 | `save` 退到 ③/④（活 launch 进程 / `.session.yaml`+证明），都不成立 ⇒ **拒绝**（不会静默写） |
| 手工看一眼 | `ros2 topic echo /map_session/info --once` 或 `ros2 service call /map_session/query std_srvs/srv/Trigger {}` |

### 11.6 这次的验收（无头；脚本与原始输出在 `.tmp_mapfix/`，未入库）

| 场景 | 结果 |
|---|---|
| **一次活栈**（假 `slam_toolbox` + 播报器 = `ZZTEST_mapfix_a`；同时把 `.session.yaml` 造成**事故形态**：`RMUC2026_dropab_ab_g_long` / pid 23936 已不在） | `save --check` 退出码 0，证据链打印 `②活会话播报 … map_name=ZZTEST_mapfix_a`、`session_source=live-helper`，并**大声提示**"`.session.yaml` 与活栈不一致 ⇒ 已忽略（事故形态）"；`--check` **一个文件都没写**；真 `save` 把图写到 `ZZTEST_mapfix_a.*`（sidecar `world: RMUC2026`），**没有**创建文件里那个名字 |
| **同名覆盖 + 写前备份 + restore 回归** | 第二次 `save` 生成 `ZZTEST_mapfix_a.posegraph.prev-<ts>`，其 sha256 = 第 1 代；当前文件确实变了；`restore --name ZZTEST_mapfix_a` 后 sha256 回到第 1 代 |
| **守卫/隔离回归** | `check --world RMUC2026` ⇒ 0（可续建）；`check --world RMUL2026` ⇒ 3（拒绝） |
| **两套栈（同一 `ROS_DOMAIN_ID`）** | `save` ⇒ 退出码 3，文案"检测到**多于一套**在跑的建图栈"，列出两套会话、给出 `ROS_DOMAIN_ID` 出路；`save --name ZZTEST_mapfix_a` ⇒ **仍退出码 3**（同名服务无法指定目标） |
| **两套栈（不同 `ROS_DOMAIN_ID`）** | 域 A 的 `save --check` ⇒ `session_name=ZZTEST_mapfix_a`；域 B ⇒ `session_name=ZZTEST_mapfix_c`（各自只看得到自己那一套） |
| **只有陈旧的 `.session.yaml`**（没有活栈） | `save` / `save --check` ⇒ 退出码 3，文案"**没有活着的建图栈**"+四条检查+`--name` 出路；**文件列表前后完全一致**（没写任何东西）；`save --name X` ⇒ 退出码 2（没有 `/slam_toolbox/serialize_map`，也没写出文件） |
| **没有播报器、只有活 launch 进程** | 用同名 argv 的桩进程模拟 `bringup_sim.launch.py`：`save --check` ⇒ `session_source=live-launch`、`session_name=ZZTEST_mapfix_e`；`--name ZZTEST_mapfix_f`（≠ 活栈）⇒ 跨会话拒绝 |
| **`--name` 显式 + 跨会话** | `--name <活栈名>` ⇒ `session_source=explicit`；`--name <别的>` ⇒ 退出码 3（跨会话拒绝）；加 `--allow-cross-session` ⇒ 放行并打警告 |
| **离线判定单测**（注入假 probe/procs/state，`python3 .tmp_mapfix/test_resolve_offline.py`） | 22 条断言全过：播报器 verified/未 verified、活 launch、活映射器自报基名（③b）、pid 三重证明、pid 回收（cmdline 不符）、pid 不在、`map_file_name` 对上/对不上、多会话三种、无证据、事故现场（文件说 `g_long`、映射器说 `v2` ⇒ 取 `v2` 并大声警告）… |
| **launch 参数无回归** | `ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args` ⇒ 退出码 0，参数表里能看到 `map_session_announce`；`DeclareLaunchArgument` **24 → 25**（只增不减；§5.1/§5.2 里"70 个参数"是**当时**的实测值） |
| **真栈（无头 Gazebo + small_point_lio + slam_toolbox + 真 launch）** | 全过（30 条断言），逐条见下 |

**真栈验收（`.tmp_mapfix/real_launch_case.sh`，`ROS_DOMAIN_ID=66`、独立 `GAZEBO_MASTER_URI`、`unset DISPLAY`）**：

| 检查 | 实测 |
|---|---|
| 播报器是**真 launch** 起的 | 起栈约 8 s 后 `ros2 node list` 同时有 `/slam_toolbox` 与 `/map_session`；`ros2 service call /map_session/query std_srvs/srv/Trigger {}` ⇒ `success=True`（自检通过），负载 `"map_name": "ZZTEST_mapfix_live"` |
| latched 话题（`save` 真正走的那条路） | `ros2 topic echo /map_session/info --once --qos-durability transient_local --reliability reliable --full-length` ⇒ 立刻拿到负载，含 `map_name` / `session_id` |
| `save --check` | 退出码 0；`session_source=live-helper`、`session_name=ZZTEST_mapfix_live`；证据链里写着 `map_file_name=（未设置/取不到 ⇒ 本次是"从零建图"）`（本次 `map_autocontinue:=False`）；**没写任何文件** |
| `.session.yaml` 新增字段 | `map_name=ZZTEST_mapfix_live` + `session_id=<uuid12>` + `launch_cmdline=['/opt/ros/humble/bin/ros2','launch','rm_nav_bringup','bringup_sim.launch.py',…]` |
| 真 `save` | 退出码 0，`会话来源: live-helper`；写出 `ZZTEST_mapfix_live.{posegraph,data,meta.yaml}`（5,006,281 B / 50,130 B；sidecar `world: RMUC2026`） |
| 同名覆盖 + 写前备份（真栈） | 第二次 `save` ⇒ 生成 `ZZTEST_mapfix_live.posegraph.prev-20261006-194011`（内容 = 第二次写之前那一代），当前文件 mtime 前进。**注意**：真 slam_toolbox 对"没有新观测的同一张图"写出的位姿图是**逐字节相同**的（这也是个有用的观察），所以这里用"备份存在 + mtime 前进"判覆盖，不用 sha256 变化 |
| 身份随栈消失 | SIGINT 关栈后 `ros2 node list` 里**没有** `/map_session`（播报器与栈同生共死 ⇒ 名字不可被后人伪造） |
| 数据安全 | 全程只用 `ZZTEST_mapfix_live*`；跑完删除；`.session.yaml` 跑前备份、跑后**还原**；`protect_before` 里 23 个用户资产（`RMUC2026_good.*` / `RMUC2026_v2.*` / `RMUC2026.pgm|yaml` / `.synth.bak` / `RMUC2026_cont.*` / `PCD/RMUC2026*.pcd*` / `good_map_preview.png`）**逐个指纹核对未变** |

**测试用的都是 `ZZTEST_mapfix_*` 名字，脚本末尾已经删干净**；用户的
`map/RMUC2026_good.*`、`map/RMUC2026_v2.*`、`map/RMUC2026.pgm|yaml`（+ `.synth.bak`）、
`map/RMUC2026_cont.*`、`PCD/RMUC2026*.pcd*`、`good_map_preview.png` 全程**只读**
（验收脚本只 `stat` 它们、从不写）。
