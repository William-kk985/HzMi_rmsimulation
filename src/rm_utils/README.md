# `rm_utils` —— 运行期「胶水 / 诊断」包

> **一句话规则**：`rm_utils` 只放**运行期**的守卫、断言、体检、播报；`tools/` 放**离线**工具；
> **功能**（谁发布哪条 TF / 哪个话题、算法本身）留在它所属的包里。
> 这里的每一条都有**删除条件**——它们不是长期架构的一部分，而是为了把已知缺陷变得**可见**。

## 1. 为什么有这个包

这些脚本原先散在 `rm_nav_bringup/scripts/`，和「地图资产 / URDF / launch」混在一起，
导致两个后果：

1. 看 `rm_nav_bringup` 的人分不清**功能**（例如 `odom_level_tf` 这条静态重力边）和
   **补丁**（例如「LIO 外参键名写错了，读回来 assert 一下」）；
2. 补丁没有到期日。它们被写进文档、被复制到 `tools/`、被 launch 到处 import，
   最后没人说得清「这条现在还需要吗」。

把运行期胶水集中到 `rm_utils` 之后，包边界就是分类边界：
**`rm_nav_bringup` 里剩下的脚本 = 功能；`rm_utils` 里的 = 可删除的观测/守卫。**
离线工具（`tools/scripts/mapping/map_archive.sh` 等）**不**搬进来——它们不在 ROS 图里跑。

### 1.1 边界（什么不该放进来）

| 东西 | 放哪 | 理由 |
|---|---|---|
| 发布 TF / 话题、参与控制或估计的功能 | 它所属的功能包 | 例如 `odom_level_tf`（`rm_nav_bringup`，契约 v3 §4.11）——删了帧图就缺一条边 |
| 离线脚本（存档、回放、对账、bench） | `tools/scripts/` | 不需要 ament 安装，也不该被 `ros2 run` |
| 算法本身的参数/实现 | 对应算法包 | `rm_utils` 不改任何算法参数 |

## 2. 安装与直接运行

```bash
colcon build --symlink-install --packages-select rm_utils
source install/setup.bash

ros2 run rm_utils lio_extrinsic_receipt.py --ros-args -p target_node:=/laser_mapping ...
ros2 run rm_utils map_asset_guard.py dirs
ros2 run rm_utils map_asset_guard.py list
ros2 run rm_utils map_resume_check.py --ros-args -p use_sim_time:=true
ros2 run rm_utils map_session_announcer.py --ros-args -p map_name:=RMUL2026 -p world:=RMUL2026
ros2 run rm_utils band_guard.py --ros-args -p use_sim_time:=true
```

`setup.py` 用 `scripts=[...]` 把五个文件**按原名**装到 `lib/rm_utils/`（保留 `.py` 后缀），
所以 launch 里的 `Node(package='rm_utils', executable='<name>.py')` 与迁移前
`Node(package='rm_nav_bringup', executable='<name>.py')` 一一对应，参数/话题/名字都不变。

`map_asset_guard.py` 同时是**共享模块**（launch 惰性 import、`cloud_accumulator` 启动/保存时
import、`tools/scripts/mapping/map_archive.sh` 直接 `python3` 调），因此另外装一份到
`share/rm_utils/scripts/`，三处继续读**同一份**判定与错误文本。

## 3. 分类表（taxonomy）

分类只有四种（代码类），外加一类**非代码**的 `history`：

- **diagnostic**：只读观测/断言，不改数据或控制路径；删掉只损失「可见性」。
- **migration shim**：为了让人从旧参数/旧路径迁过来，过渡期保留；过期就该删。
- **workaround**：已知缺陷的临时绕法；缺陷修好后删，**不要**把它当架构。
- **feature**：功能本身；不适用「到期删除」，只能随契约变更。
- **history**：**已经没有代码**（参数/文件已删除），只作为「不要再加回来」的记录；
  「何时可删」= 文档清理时可以把这一行移进历史归档。

| # | 项 | 现在在哪 | 是什么 | 为什么存在（契约/章节） | 分类 | 何时可删（owner / expiry） |
|---|---|---|---|---|---|---|
| 1 | `lio_extrinsic_receipt.py` | `rm_utils`（本包） | 一次性节点：等 LIO 起好后读回**生效的**外参参数，与 launch 从 URDF 解析的期望值逐元素比对；不一致/死键 ⇒ ERROR + exit 1 | 三个 LIO 读的键名不同（fastlio/pointlio = `mapping.extrinsic_*`，SPL = 顶层 `extrinsic_*`）；2026-10-27 前 fastlio/pointlio 收到的是死键，robot11 的 +30° 静默丢失。契约 §4.12.5；meas §34.3#4 / §35 | **diagnostic**（运行期断言） | owner = LIO 槽位/帧图契约。expiry：三个后端统一外参键名、或在各自源码里对参数声明做启动断言之后；届时 launch 不再需要「读回」 |
| 2 | `map_asset_guard.py` | `rm_utils`（本包） | 纯 Python 共享模块 + CLI：存档 sidecar（`<name>.meta.yaml`）读写、world/spawn/kind 隔离判定、覆盖前备份/恢复（`*.prev-<ts>`）、会话名解析 | `map` 系 = **出生点相对系**，换 world 就换原点；两个场地的位姿图叠在一起 = 假墙/回环错配/定位全废。2026-10-06 事故后新增。docs/continue_mapping.md §5/§9/§11；docs/map_assets.md | **diagnostic / guard**（含 safety 备份） | owner = 建图/存档。expiry：存档格式自带坐标系与来源、加载器能**自证**不属于本场地时；或部署固定单一 world 且不再跨场续建 |
| 3 | `map_resume_check.py` | `rm_utils`（本包） | 续建后**一次性**体检：比较「加载后的 `map→base_link`」与 sidecar 记录的起点/修正量，超阈值响铃；`strict:=True` 收栈 | 场地隔离守卫只管 world/spawn **对不对**，管不了「位姿图本身歪了」（2026-10-06 ATE max 1.15 m 的续建事故）。docs/continue_mapping.md §8 | **diagnostic** | owner = 建图/定位。expiry：存档在落盘前有离线一致性校验（或建图链能拒绝退化段），续建不再需要事后体检 |
| 4 | `map_session_announcer.py` | `rm_utils`（本包） | 把本次 launch 的 `map_name/world/archive_base/session_id/...` 挂到 ROS 图（latched `/map_session/info` + `/map_session/query`），并自检 mapper 唯一 | `.session.yaml` 会被任何后启动的 launch 覆盖；2026-10-06 `save` 把 31.89 MB 位姿图写到别人会话名下。docs/continue_mapping.md §11（schema 字符串 `rm_nav_bringup/map_session@1` 保持原样 = 线协议，不是路径） | **diagnostic**（身份播报） | owner = 建图工具。expiry：`save` 能从单一写者/映射器自身参数稳定拿到会话身份（不再需要广播），或禁止并发 launch |
| 5 | `band_guard.py` | `rm_utils`（本包）——**本轮从 `rm_nav_bringup` 移入** | 只读节点：订阅 `/scan` + TF，回读两张 costmap 的**生效**高度带参数，报 in-band/below/above、kept-y 单侧盲区；发 `/band_guard/status` | nav2 的 `obstacle_layer` / `ObservationBuffer` 静默丢波束，没有 WARNING/计数；实测全局 774/774 全丢、局部 43.1% + 半盲。meas §15 / §17 | **diagnostic** | owner = nav2/感知。expiry：高度带改为单一真源并有启动断言（或 nav2 侧有 dropped-beam 计数），不再需要旁路复算 |
| 6 | `odom_level_tf.py` | **`rm_nav_bringup`（不搬）** | 静态边 `odom → odom_level` 的**唯一发布者**（latched `/tf_static`）+ 两条只读互检 | 这是帧图契约的功能：重力对齐从估计器补丁搬到帧图，`odom_level` 是常量变换。契约 §4.11；meas §31 | **feature** | 不适用「到期删除」。只有换帧图契约（例如取消 `odom_level`）时才动；owner = 帧图契约 |
| 7 | 旧 `gravity` 参数硬报错别名（`_LEGACY_GRAVITY_ALIASES`：`gravity_align_mode`、`gravity_align_z_origin`、`level_frame`、`z_origin_source`、`lio_init_settle_s`、`lio_init_map_extrinsic`） | `rm_nav_bringup/launch/bringup_sim.launch.py` | 传非旧默认值 ⇒ t=0 硬报错，消息指向唯一开关 `gravity:=tf|base|lio|raw` | 2026-10-21 收敛为**一个**用户开关；Humble 对未知/旧参数会静默接受，所以必须显式拦。契约 §4.14.2；meas §33 | **migration shim** | owner = 帧图契约。expiry：下一个 breaking 清理轮（建议 ≤ 2026-12）或确认没有脚本/文档再传旧名后 |
| 8 | 已删除参数硬报错（`_REMOVED_LAUNCH_ARGS`：`robot11_mount`、`turret_yaw_deg`、`turret_pitch_deg`） | `rm_nav_bringup/launch/bringup_sim.launch.py` | 同上：这三项已从 `--show-args` 消失，但仍传 ⇒ t=0 硬报错 | 安装方式只剩 URDF `sensor` 一种、云台是静态的；删声明不会让 Humble 报错，所以加守卫。契约 §4.15；meas §38.5 | **migration shim** | owner = 帧图契约/模型。expiry：同 #7 的清理轮；或 Humble 换成对未知 launch 参数会报错的版本（那时守卫可删） |
| 9 | 历史：`robot11_mount` / `turret_yaw_deg` / `turret_pitch_deg` | **已删除**（历史，仅 #8 的守卫还在） | 曾经的三档安装方式 / 云台静态姿态参数 | 三档并存 = 倾角有三个所有者；契约 §4.15 / meas §38 | **history** | 已删；不要再加回来。新需求走 URDF / xacro，不要走 launch |
| 10 | 历史：`livox_mount` / `livox_tilt_rpy` / `livox_tilt_axis` / `livox_raise_m` | **已删除**（历史） | 曾经由 launch 传的雷达安装方式/倾角/轴/抬升 | 安装角唯一来源收口到上游 URDF → 生成物 xacro（§4.12.4；meas §28/§31）。⚠️ 这组**没有**像 #8 那样的 t=0 硬报错守卫：今天再传会被 Humble 静默接受 | **history** | 已删；如需要「传了就停」，把它们加进 `_REMOVED_LAUNCH_ARGS`（见 §5 的诚实清单） |
| 11 | `gate_reanchor_after_rejects` + `lost_probe_period_sec` | `rm_localization/gicp_registration/config/gicp_registration_sim.yaml` | 连续被创新门拒 N 帧 ⇒ 锚点拉回「只信里程计」的预测位姿，并**豁免下一帧的创新门**；判失效后每 1 s 跑一帧只算不发 | 传感器外参偏置这类恒定创新会被创新门永久拒绝；「永久停发」比「接受偏置继续跟」更糟。**但 §43.3 证明它把 0.46–2.3 m 的跳变放行了** | **workaround → 已被 §4.17 的「种子信任窗口」取代/收窄**（历史行保留） | owner = gicp/定位。2026-10-29 起：`/initialpose` 后的 `seed_trust_window_sec`（默认 5 s，或直到第一帧被采纳）内**禁用**重锚+豁免；到期仍无采纳 ⇒ NO_CONVERGE。窗口外的旧语义只在「没有新全局种子」时仍起作用（无种子的恒定创新自愈，§44.4 死锁回归）；彻底删除仍待信任域判据落地，届时 `gate_reanchor_after_rejects: 0` + `lost_probe_period_sec: 0.0` |
| 12 | `scan_band_min`（默认 `-0.05`，只许 ≤0） | `rm_nav_bringup/launch/bringup_sim.launch.py` + `config/nav2_params_sim_local_scan_band.yaml` | 局部 2D scan 高度带下界的小负余量 | hybrid 最低波束端点只有 +0.007 m，而 LIO z 漂移 −0.065…−0.077 m ⇒ 只剩 7 mm 余量。meas §25/§26 | **workaround** | owner = nav2/帧图。expiry：LIO 的 z 漂移被压到厘米级以下、或高度带由标定自动生成时；回滚 = `scan_band_min:=0.0` |
| 13 | `cloud_accumulator_target_frame:=odom` | `rm_nav_bringup/launch/bringup_sim.launch.py` + `rm_perception/cloud_accumulator` | 3D 先验累积改用平滑的 LIO `odom` 系，落盘后再用一次 `map←odom` 搬进 `map` | `map→odom` 的修正会撕裂已累积的点云，实测墙面残差 0.2–0.4 m「鬼影」。meas §23.9#1 / §24.5 | **workaround**（默认仍是 `map`） | owner = 3D 先验/建图。expiry：累积器能对每帧记录当时的 `map→odom` 并回搬（或 SLAM 后端提供连续 `map→odom`）后可去掉这个开关 |
| 14 | 出生点表镜像（`bringup_sim.launch.py::world_configs` ↔ `map_asset_guard.py::WORLD_SPAWN`） | launch + 本包 | 两张表必须逐 world 的 x/y/z/yaw 一致；`ros2 run rm_utils map_asset_guard.py spawn-table --check` 对账 | 存档 sidecar 的 `spawn_pose` 比对以 `WORLD_SPAWN` 为基准；改了 launch 的 z 会让既有存档全判 `refused_spawn` | **workaround**（重复真源） | owner = 建图资产。expiry：出生点表移进**一份** YAML，launch 与守卫都读它（那时删掉镜像与对账命令） |
| 15 | `smoothing_enable` / `smoothing_tau`（gicp 输出平滑） | `gicp_registration_sim.yaml` | `map→odom` 一阶低通，压掉每帧修正噪声 | 用户抱怨「四处抖动」；高频运动仍由 `odom→base_link` 提供。meas §4 | **workaround**（噪声对策） | owner = gicp/定位。expiry：trust-region/后端滤波在源头给出连续 `map→odom` 后可关；回退 = `smoothing_enable: false` |

> 说明：本表覆盖**运行期胶水**与本轮搬迁涉及的 launch/config 补丁。#7/#8/#10 在 `rm_nav_bringup`
> 的 launch 里，**没有**搬进 `rm_utils`——它们是 launch 的参数契约（t=0 求值），搬走会改变
> 启动时序/依赖；这里只登记分类与到期条件。算法包内部的「兜底默认值」不在本表范围。

## 4. 逐项说明（为什么搬 / 为什么不搬）

### 4.1 搬进 `rm_utils` 的四个（+ `band_guard`）

- **`lio_extrinsic_receipt.py`**：纯运行期断言，不属于「起栈」也不属于「地图资产」。
  Moved：`package='rm_nav_bringup'` → `'rm_utils'`，`executable` 不变。
- **`map_asset_guard.py`**：它被三处共用，**不是** `rm_nav_bringup` 私有实现。
  放 `rm_utils` 后，`rm_nav_bringup` 只保留「map/PCD 资产 + launch」；判定逻辑仍只有一份。
  唯一改动是 sidecar 的 `written_by` 字符串（`rm_utils/rm_utils/map_asset_guard.py`）——
  这是**新写** manifest 的来源标注；既有 `map/*.meta.yaml` 里的旧字符串属于历史资产，未改。
- **`map_resume_check.py`**：一次性体检节点，只在续建时由 launch 起，默认不参与从零建图。
- **`map_session_announcer.py`**：只发布/只应答的会话身份广播，不订阅、不发 TF/map。
- **`band_guard.py`（决定：搬）**：它是**只读诊断**，默认 `band_guard:=False` 时节点集完全不变，
  launch 接线只有一处 `Node(package=..., executable=...)`；搬走后 `rm_utils` 成为运行期诊断的
  单一入口。代价：旧文档里的 `ros2 run rm_nav_bringup band_guard.py` 需要读成
  `ros2 run rm_utils band_guard.py`（docs §1–§43 body 未改，见 §6）。

### 4.2 留在 `rm_nav_bringup` 的

- **`odom_level_tf.py`**：**功能，不是胶水**。它发布 `odom→odom_level` 这条静态边，
  是契约 v3 §4.11 的一部分；删掉/搬走都会让「重力对齐归帧图」的契约缺一条边。
  仍由 `install(PROGRAMS ...)` 装到 `lib/rm_nav_bringup/`。
- **`rm_nav_bringup/scripts/`** 现在只剩它一个 `.py`；CMake 的 `INSTALL_TO_SHARE scripts`
  已删（不再有需要按 share 路径 import 的脚本）。

## 5. 诚实清单（本轮明确没做/做不到）

1. **旧文档路径没有逐条改**：`docs/frame_contract_v3.md`、`docs/frame_contract_v3_measurements.md`、
   `docs/continue_mapping.md`、`docs/map_assets.md` 里仍写着
   `src/rm_nav_bringup/scripts/<name>.py`。本轮按约束**不改 docs §1–§43 body**；
   那些路径是**历史坐标**，现行路径以本 README 为准。`docs/README.md` 顶部已加指引。
2. **`livox_*` 四个旧参数没有 t=0 硬报错**（见 #10）：今天传它们会被 Humble 静默接受。
   要补的话是 launch 的活（加进 `_REMOVED_LAUNCH_ARGS`），不是 `rm_utils` 的活；
   本轮不改行为。
3. **`map_asset_guard.py` 的 `written_by` 字符串变了**：新写 sidecar 会记
   `rm_utils/rm_utils/map_asset_guard.py`。这是路径变更，不是判定逻辑变更；
   既有 sidecar 不重写。
4. **`map_session_announcer` 的 schema 字符串保持 `rm_nav_bringup/map_session@1`**：
   它是线协议标识（`save` 按它解析），不是文件路径；改它才是行为变更。
5. **`band_guard` 的 launch 参数 description 文本改了**（`rm_nav_bringup/band_guard.py`
   → `rm_utils/band_guard.py`），因此 `--show-args` 有 **1 行**合法包名差异；
   参数名/默认值/choices/数量都不变。
6. **`gate_reanchor_after_rejects` 仍是 workaround，但已收窄**：rm_utils 迁移轮只登记、没改
   `rm_localization/**`；2026-10-29 的修复轮在 gicp 包加了 `seed_trust_window_sec`（§4.17），
   在**新全局种子**后禁用「重锚+豁免」；窗口外的旧语义仍保留（无种子死锁自愈，§44.4）。
   彻底删除要等信任域判据落地，不能靠调小阈值长期共存。
7. **`band_guard` / `map_session_announcer` 的用法 docstring** 从
   `ros2 run rm_nav_bringup ...` 改成 `ros2 run rm_utils ...`（包路径）；wire schema
   （`rm_nav_bringup/band_guard@1` / `rm_nav_bringup/map_session@1`）**保持原样**——
   它是协议标识，不是路径。`map_asset_guard.py` 里的 `find_share_dir('rm_nav_bringup')`
   也保持原样：它找的是 map/PCD **资产**的归属包，和守卫代码放在哪个包无关。
8. **`tools/scripts/regress/nav_smoke_regression.py` 的 TF 链已修（2026-10-29，§4.17/§44.5）**：
   旧版在 `FULL_CHAIN` 里写死 `odom->base_link`，默认 `gravity:=tf` 的健康栈会被判
   `链路未就绪，缺: TF:odom->base_link`（工具陈旧）。现在检查当前契约：
   `map→odom`、`odom→odom_level`（静态，`--robot` 决定期望 rpy：robot11 = [30,0,0]°，
   `''`/hzmirm = [0,0,0]°）、`odom_level→base_link`、`map→base_link` 可解析；
   **不要求** `odom→base_link`，且 `gravity:=tf` 下出现它即 FAIL；`map→odom` 的单发布者
   用「/tf 端点清单 + tracker 唯一 + 同戳冲突」多路证据断言。
   RMUL2026 默认档与 `robot:=robot11` 两跑均 PASS（§44.5）。

## 6. 验证（本轮实测口径）

```bash
# 1) 构建
colcon build --symlink-install --packages-select rm_utils rm_nav_bringup

# 2) launch 参数面（私有 HOME/ROS_LOG_DIR，避免只读 $HOME）
HOME=$PWD/.tmp/rm_utils_home ROS_LOG_DIR=$PWD/.tmp/rm_utils_log \
  ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args

# 3) 可执行文件（这些是**节点**，会一直跑；用 timeout 看启动无 traceback 即可）
ros2 pkg executables rm_utils
ros2 run rm_utils map_asset_guard.py dirs
timeout 5 ros2 run rm_utils lio_extrinsic_receipt.py --ros-args -p target_node:=/laser_mapping
timeout 5 ros2 run rm_utils map_resume_check.py --ros-args -p use_sim_time:=false
timeout 5 ros2 run rm_utils map_session_announcer.py --ros-args -p map_name:=X -p world:=RMUL2026
timeout 5 ros2 run rm_utils band_guard.py --ros-args -p use_sim_time:=false

# 4) 同源判定（launch / 节点 / shell 读的是同一份）
ros2 run rm_utils map_asset_guard.py spawn-table --check
tools/scripts/mapping/map_archive.sh info --name RMUC2026   # 或 dirs
```

本轮实测（2026-10-29，本机）：

| 项 | 结果 |
|---|---|
| `colcon build --symlink-install --packages-select rm_utils rm_nav_bringup` | 成功（rm_utils 0.87 s / rm_nav_bringup 0.31 s） |
| `--show-args` | exit **0**；与迁移前逐行 diff = **1 行**（仅 `band_guard` description 的包名） |
| `Node(...)` 调用点（AST） | **34 → 34**；`IncludeLaunchDescription` 10 → 10 |
| `robot_description` md5（`''`/`hzmirm`/`robot11`） | `bc62af22f7be49395525978b1264750c` / `a13e053090bf1344a06d58241bf845f9` / `4f19a8b4faa3a7c659850d0064d3c5ca`（10353 / 18630 / 47745 B）——**逐字节不变** |
| `ros2 run rm_utils <5 个脚本>` | 5/5 可执行（`ros2 pkg executables rm_utils` 列出 5 个） |
| nav 冒烟（RMUL2026，`lio:=fastlio localization:=amcl nav:=rpp spin_speed:=0.0 band_guard:=True`，默认 `gravity:=tf`） | 目标 `(3.77, 2.03)` ⇒ **SUCCEEDED**（11 s，0 recoveries）；`/controller_server` `/planner_server` `/bt_navigator` `/amcl` `/map_server` 全 `active [3]`；`map→base_link` 与 `odom_level→base_link` 可解析；`lio_extrinsic_receipt` 打 **OK**；`/band_guard` 节点 + `/band_guard/status` 有数据 |
| mapping 冒烟（RMUL2026，`cloud_accumulator:=True map_session_announce:=True`） | `/map_session` 节点在；`/map_session/query` 返回 `verified: true`（`mapper_count: 1`、`serialize_present: true`）；`cloud_accumulator` 成功从 `rm_utils` 载入守卫（autoload 拒绝文本与旧版一致） |

> 默认路径的等价性由 `tools/scripts/gzsim/launch_increment_diff.py` 的既有口径核对；
> `Node(...)` 调用点数量与 `robot_description` md5 是上面这张表的硬数字。
> 构建/Run 时用私有 `HOME`/`ROS_LOG_DIR`：本机 `~/.ros/log` 所在文件系统只读。
