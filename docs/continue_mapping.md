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
#   它做三件事：① /slam_toolbox/serialize_map → map/<名字>.{posegraph,data}（同名覆盖）
#              ② 等两个文件真的落盘，再刷新 sidecar map/<名字>.meta.yaml
#              ③ 若 3D 累加器在跑（cloud_accumulator:=True），顺带存 PCD/<名字>.pcd

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
#   然后**接着走没走过的区域**，再 save —— 同名覆盖，两张图是同一张。
```

### 0.1 新增的 5 个启动参数（`bringup_sim.launch.py`，默认值 = 现有行为）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_name` | `''` | 存档基名；留空 ⇒ 取 `<world>`。存档 = `map/<名字>.{posegraph,data,meta.yaml}` + `PCD/<名字>.{pcd,meta.yaml}` |
| `map_autocontinue` | `True` | 同名位姿图存在 ⇒ 反序列化续建；不存在 ⇒ 从零建（两种情况都在日志里写明路径）。`False` = 永远从零建 |
| `map_start_pose` | `[0.0, 0.0, 0.0]` | 续建时告诉 slam_toolbox「机器人现在在**旧图**的哪个位姿」（x, y, θ，`map` 系） |
| `map_allow_world_mismatch` | `False` | `True` = 显式跳过 world/spawn 隔离检查（**日志会一直提醒**）。只想从零建请用 `map_autocontinue:=False` |
| `cloud_accumulator` | `False` | 仅 `mode:=mapping`。`True` = 起 3D 点云累加器（见 §3），让 `PCD/<名字>.pcd` 也跨会话续建 |

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
（git 忽略），`map_archive.sh save` 默认读它 ⇒ **不用手工重复名字**。
写失败只警告，不影响建图（那时 save 需要显式 `--name`）。

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

---

## 6. 命令行速查（`tools/scripts/mapping/map_archive.sh`）

```bash
map_archive.sh save [--name X] [--world W] [--no-cloud] [--allow-world-mismatch]
#   ① 守卫先查（拒绝 ⇒ 退出码 3，**不写任何文件**）
#   ② /slam_toolbox/serialize_map 写 map/<名字>.{posegraph,data}（同名覆盖）
#   ③ 等两个文件都出现（默认最多 60 s）后刷新 sidecar
#   ④ 3D 累加器在跑 ⇒ 顺带 save PCD
map_archive.sh info [--name X]      # sidecar + 每个文件的大小/时间戳（2D 与 3D 都打）
map_archive.sh list                 # map/ 与 PCD/ 下所有存档 + world 摘要（旧版会标"无 sidecar"）
map_archive.sh adopt --name X --world W   # 给旧版存档补 sidecar（人工担保）
map_archive.sh dirs                 # launch/脚本实际读写的那两个目录
```

退出码：`0` 成功；`2` 用法/环境问题（比如没有 `/slam_toolbox/serialize_map`）；`3` **守卫拒绝**；`4` 落盘超时。

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
```

---

## 8. 未验证 / 已知坑

1. **`lio:=fastlio` / `pointlio` 没跑续建验证**：本次只验了 `lio:=small_point_lio`。
   机制上续建与 LIO 无关（只跟 `map_file_name` 和 `map/odom` 有关），但**未实测**。
2. **`mapper:=cartographer` 不支持续建**：cartographer 的存档是 `.pbstream`（另一套
   `load_state_filename`/`load_frozen_state` 机制），本次**没有**接入本守卫；
   用 `mapper:=cartographer` 时这 5 个参数里只有 `cloud_accumulator` 与 `map_name`（3D 名字）生效。
3. **`mode:=slam_nav` 下也会走同一套续建判定**（同一个节点），但**没有单独跑验收**。
4. **`resolution` 改变只报警不拦**：同一份存档被 0.05 → 0.10 的栅格续建时，
   旧图与新帧不在同一栅格上，质量自负（日志里有 ⚠️）。
5. **3D 先验的干净程度完全取决于 LIO**（§5 的 run B 末段就是反例）。
   目前没有"离群帧剔除 / z 带过滤 / 跳变闸"——`status` 的 bbox 与 `pcd_stats.py`
   的 z 分位数是唯一的体检手段。要做需要另开一条（跳变闸在
   `tools/scripts/diag/map_odom_jump_gate.py` 里有现成的 alert-only 版本可参考）。
6. **存档体积**：一次 ~64 s 的建图 = 5.4 MB posegraph + 0.6 MB data。
   长跑（十几分钟、多回环）会长到几十 MB，**别顺手 `git add`**（`RMUC.posegraph` 13 MB /
   `RMUC.data` 8.5 MB 就是历史教训）。存档是否入库由你决定，本仓库的 `.gitignore` **没有**忽略
   `map/*.posegraph`。
7. **多机/多栈同名**：`.session.yaml` 只记"最近一次 launch"。同时在两个终端起两套不同
   `map_name` 的栈，`map_archive.sh save`（不带 `--name`）只会认最后写会话的那一套 ⇒ 多栈请显式 `--name`。
8. **`adopt` 是人工担保**，脚本无法验证真伪；写错 sidecar ⇒ 以后续建就歪。
9. **没做**：`map_allow_world_mismatch` 的"只警告不拦"中间档；存档自动备份（覆盖前留 `.bak`）；
   同名存档的版本历史。要的话都得再改一层。
