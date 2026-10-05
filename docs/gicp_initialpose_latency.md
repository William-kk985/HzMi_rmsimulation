# gicp_registration：`/initialpose` 的延迟/饥饿问题 —— 现象、修复设计与实测（2026-10-05）

> 范围：`src/rm_localization/gicp_registration/` 的节点代码，以及本文件。
> **契约一个都没动**：只发 `map→odom`；`tf_lookahead_sec=0.45`；MultiThreadedExecutor + TF 独立回调组；
> 三条健康话题；两级 leaf；`backend` 默认 `pcl`；`/initialpose` 仍然 **RELIABLE**。
> 变的只有**调度与"谁来写 map→odom"**：`/initialpose` 回调从"当场做完全部工作"改成"只做交接"，
> 真正的工作搬到**下一帧点云的回调开头**（那本来就在 align 回调组里）。

---

## 1. 一句话结论

* 改前：`/initialpose`（RViz 的 2D Pose Estimate）与点云订阅**同处一个 `MutuallyExclusive` 回调组**
  （align 组）。点云 10 Hz、PCL 后端单帧 align 400~490 ms ⇒ 该组**接近 100% 占用**，
  点击的回调要排在在飞的 align（以及后面排队的点云）之后 ⇒ 实测**一次点击 13.2 s 才被处理**、
  另一次 **14 s 内完全没被处理**（`docs/gicp_backend_small_gicp.md` §6.5；本轮又复现出 **22.7 s**）。
* 改后：点击回调只做 **POD 交接**（拷一份原始 pose/covariance + header，置 `pending_initial_pose_valid_`），
  跑在**自己的回调组**里 ⇒ 交接延迟 < 1 ms；真正的"应用"（TF 查询 + `T_map←odom = T_map←base·T_base←odom`
  + 状态写入 + no-improve 清零 + 日志 + 一条 `~/pose`）在**下一帧点云的帧首**执行
  ⇒ 生效延迟 = **一个 align 周期**（本机实测 **0.24 s**，同场景改前 22.7 s）。
* "初值与 align 必须串行"这条不变量**不但保住，而且更严格**：写 `map→odom` 的代码现在就在 align 回调
  **同一条回调序列**里（不再是"两个不同回调靠组内互斥串行"），PCL / small_gicp 对象依旧只在这条路上被碰。

---

## 2. 问题

### 2.1 实测数字与出处

| 场景 | 延迟 | 出处 |
|---|---|---|
| 节点空转（无点云），`ros2 topic pub --once /initialpose` | **~15 ms** | `docs/gicp_backend_small_gicp.md` §6.5；原始记录 `.tmp_cache/gicp_ab/initpose_lat/`（`initpose_latency.sh`） |
| align 组被点云占满（PCL 400~490 ms/帧 @10 Hz），探针（RELIABLE，5 条） | **≈ 13.2 s**（直到点云停下来的那一刻才被处理） | 同上；原始记录 `.tmp_cache/gicp_ab/initpose_starve/`（`initpose_starve.sh`） |
| 同上，`ros2 topic pub --once` | **14 s 内完全没有被处理** | 同上 |
| 同上（本轮复现，同口径、本机 load 偏高、align 实测 455~483 ms） | **22.7 s / 22.6 s**，且**只在点云流停止后 0.6~0.7 s** 才被处理 ⇒ 饱和期间等于"永不处理" | 本轮 `.tmp_cache/initpose_fix/runs/before_sat_r{1,2}/summary.json`（§4.2） |

> 注：任务书里把这段记作 `docs/gicp_backend_small_gicp.md` §7.4；**实际位置是 §6.5**
> （§7 是"复现命令"，其中 ③ 就是当年那两个脚本）。本文用 §6.5 的编号。

### 2.2 机制（为什么"同组"必然导致饥饿）

* 回调组是 `MutuallyExclusive`：**同一时刻组内只有一个回调在跑**，且 rclcpp 的 executor 只有在
  "上一个回调返回"之后才会从该组里再取下一个（可取项里包含点云订阅与 `/initialpose` 订阅）。
* 点云用 `SensorDataQoS().keep_last(1)`，10 Hz 到达；单帧 align 400~490 ms ⇒ **点云订阅永远有货**。
* 实测行为：executor 取到的几乎总是点云（`/initialpose` 的样本一直排在后面），于是点击被"饿死"；
  只有点云流断了，组空出来，点击才被处理（13.2 s / 22.7 s 都是这么来的）。
* 这不是"RELIABLE 丢包"的问题：样本是 RELIABLE、也没超时丢；纯粹是**调度饥饿**。
  （§6.5 原文也承认"未做根因定位"；本轮用"交接延迟 < 1 ms、应用延迟 ≈ 一个 align 周期"的对照实验
  把根因钉在调度上：同一个探针、同一条 RELIABLE 消息，只是回调组换了。）

---

## 3. 修复设计：handoff（交接）+ apply（应用）

### 3.1 数据流

```
/initialpose ──(init_pose_cb_group_，独立 MutuallyExclusive 组)──► initialPoseCallback()   【交接】
                  锁内：pending_initial_pose_.pose   = msg->pose    （原始 pose + covariance，一个字段不改）
                        pending_initial_pose_.header = msg->header  （原始 stamp + frame_id）
                        pending_initial_pose_valid_  = true
                  锁外：一行 1 Hz 限频 INFO
                  **没有 TF 查询 / 没有变换 / 没有写 map→odom / 没有 reset / 不发任何话题**

点云帧 ──(align_cb_group_，与上面并行)──► pointcloudCallback() 帧首 ⓪
                  consumePendingInitialPose()                                            【应用】
                    · 锁内：取出并清 pending_initial_pose_valid_（取出即消费，TF 查不到也不重试）
                    · lookupTf(odom, base, 原始 stamp, 精确→退最新)   ← 与旧代码逐行一致
                    · T_map←odom = T_map←base · T_base←odom           ← 与旧代码逐行一致
                    · 锁内写：T_map_odom_ / estimate_valid_=true / param_init_pending_=false /
                              no_improve_cycles_=0                     ← 与旧代码逐行一致
                    · INFO "/initialpose 更新初值：…" + 立刻 publishPose(T_map_odom, -1.0)
                  ↓ 同一帧继续 ①转 PCL … ⑥align … ⑦接受/拒绝 … ⑧~/pose（与改动前完全同一条路径）
```

代码位置：
* `src/rm_localization/gicp_registration/src/gicp_registration.cpp`：`initialPoseCallback()`（交接，约 834 行）、
  `consumePendingInitialPose()`（应用，紧随其后）、`pointcloudCallback()` 帧首的 ⓪ 段；
* `src/rm_localization/gicp_registration/include/gicp_registration/gicp_registration.hpp`：`PendingInitialPose` 交接槽（`mutex_` 保护）
  与第三个回调组 `init_pose_cb_group_`。

### 3.2 `/initialpose` 的订阅放在哪个组：**自己的第三个组**（不是 TF 组）

* **必须在 align 组之外**：留在 align 组里，交接回调仍然要排在在飞的 align 后面（一次 400~500 ms），
  而且点云订阅一直有货 ⇒ 饥饿照旧。这是这次修复的全部意义。
* **不放 `tf_cb_group_`**（TF/状态定时器组）：那个组的契约是"50 Hz 的 TF/状态发布绝不被拖慢"
  （改前 `/tf` 掉到 2 Hz 就是它被拖慢的后果）。`/initialpose` 是**人手点**的话题，RViz 可能连点/突发重发，
  把它塞进去等于让人类操作与 TF 发布共享调度预算；独立组最干净，也最好推理
  （组内只有一个订阅、回调只有一个 µs 级动作）。
* **QoS 保持 RELIABLE**：`rclcpp::QoS(rclcpp::KeepLast(10))`（默认 RELIABLE）一字未动。
  RViz 的 2D Pose Estimate 就是 RELIABLE 发的；改成 BEST_EFFORT 会在网络抖动时**静默丢点击**，
  这是契约不是实现细节。（交接变便宜之后，RELIABLE 也不再有任何"占着组"的代价。）

### 3.3 串行性不变量：保住且更强

* 旧设计的不变量 = "人工初值的写入"与"align 的读改写"不交错，靠**组内互斥**实现
  （两个不同回调共享一个 `MutuallyExclusive` 组）。
* 新设计：写 `T_map_odom_` 的那段代码**物理上搬进了 `pointcloudCallback`**，而 `pointcloudCallback`
  本身独占 align 组 ⇒ 初值应用与 align 现在是**同一条回调序列里的相邻两步**（先应用、再配准），
  连"两个回调互相插队"的可能都不存在了。
* 交接回调与 align 的**唯一**共享状态是 `pending_initial_pose_*`，全部在既有 `mutex_` 下读写；
  交接回调不碰 `T_map_odom_`、不碰 TF buffer、不碰 PCL/small_gicp 对象。
* 仍只有一个线程会进 `backend_->align()`（align 组 `MutuallyExclusive`）⇒ PCL GICP / small_gicp
  "非线程安全"这条前提一字未改。
* "三个健康话题只由 align 组发布、且不持锁发布"这条不变量也更强了：`initialPoseCallback` 现在
  **一个话题都不发**（`~/pose` 由 `consumePendingInitialPose` 在 align 组内补发）。

### 3.4 语义逐条对齐（哪些一字未改，哪些是有意为之的差异）

**一字未改（逐行等价）**：
1. 语义 = map 系下 `base_frame` 的位姿；`T_map←odom = T_map←base · T_base←odom`；
2. `lookupTf(odom, base, 原始 header.stamp, try_exact=true)`：精确时间戳查不到就退"最新可用"
   （`ros2 topic pub` 那种 `stamp=0` 的样本依旧走退路；RViz 的真实戳依旧走精确查询）；
3. `odom→base` TF 查不到 ⇒ **丢弃**这次初值 + 同一条 WARN（`/initialpose 收到，但 TF … 查不到 ⇒ 忽略本次初值`），
   且**不改任何状态**（`estimate_valid_` 不会因此变 true ⇒ 仍然不发 `map→odom`）；
4. `use_initial_pose` 参数与本路径无关：`use_initial_pose:=false` 时 `/initialpose` **照样有效**
   （只有 `initial_pose` 参数那条惰性初始化被关掉）；
5. 应用成功后置 `estimate_valid_=true`、`param_init_pending_=false`（人工初值优先于参数）、
   `no_improve_cycles_=0`；
6. 日志文本：`/initialpose 更新初值：map 系机器人位姿 … ⇒ map→odom = …` 原文保留（交接那条是**新增**的、
   1 Hz 限频的一行）；
7. 立刻发一条 `~/pose`（`score=-1`，与旧实现"点击后马上有一条 ~/pose"一致），本帧末尾仍按正常路径再发一条；
8. 接受/拒绝路径、`map→odom` 数值、`/tf` 时间戳（`now+tf_lookahead_sec`）、话题集合与 QoS 全部不变。

**有意为之的差异（3 条，都会在实跑里体现）**：
1. **应用挂在"下一帧点云"上**：点云完全不来时，点击会**排队**（不再当场生效），恢复点云后的第一帧应用最新一次点击。
   旧实现是当场生效。⇒ 代价是"没有点云的场景下点击不再有即时反馈"；收益是延迟与 align 成本解耦。
   （实跑里 LIO/Livox 一直在发点云，这条不构成问题；但**要先起来 LIO 再点**这个顺序要求变得更明确。）
2. **`odom→base` TF 可用性的判定时刻**从"点击时刻"挪到"下一帧开头"（≤ 一个 align 周期）。
   方向上只会让原本会被丢弃的点击**更可能**被采纳（TF 晚 0.5 s 就绪也算数）。
3. **连续点击只保留最后一次**（交接槽被覆盖）：终态与旧实现一致（最后一次生效），
   但中间那几次不再各写一次 `map→odom` / 各发一条 `~/pose`（旧实现每次点击都会写一遍，属实现细节）。

---

## 4. 实测：before / after

### 4.1 实验口径（可复现）

* **单节点**：只跑 `gicp_registration_node`，**不启 Gazebo / nav2 / LIO**；`ROS_DOMAIN_ID=87`（与其它 agent 隔离）、
  `ROS_LOG_DIR` 落在工作区内。
* 点云：从 `src/rm_nav_bringup/PCD/RMUL2026.pcd` 随机采样 4000 点（`np.random.default_rng(7)`）、
  只带 x/y/z（无 intensity）、相对地图偏移 `(+0.15, -0.15, 0)`，按 **10 Hz** 持续发布
  （= 文档里既有的合成扫描口径，稳态 `source=3701` 点、`target=12450` 点）。
* TF：`odom→base_link`、`odom→livox_frame` 用静态 TF 广播（等同 LIO 的 TF 已就绪；
  与当年记录 13.2 s 的实验同口径）。
* 点击：**只发一条** RELIABLE 的 `/initialpose`（= RViz 一次点击），点击值刻意与真值差
  `+0.5 m / +10°`（`0.35, -0.35, 0, 10°`；收敛真值约 `-0.148, 0.146, 0°`）⇒ 生效时 `/tf` 上会出现
  一个明确可辨的阶跃。
* **三个独立可观测量**同时测（互相印证）：
  1. 节点日志里 `/initialpose 更新初值…` 行的墙钟戳 − 点击墙钟戳（ns 级日志戳）；
  2. 探针收到的**第一条** `~/pose`，其数值与预期应用值 `T_map_base·T_odom_base` 一致（1e-4 m / 0.01°）；
  3. 探针收到的**第一条** `/tf` 上的 `map→odom` 与预期应用值一致（TF 以 50 Hz 发布 ⇒ 误差 ≤ 20 ms）。
* 驱动脚本：`.tmp_cache/initpose_fix/driver.py`（探针 + 节点子进程 + 三个观测量的 JSON 记录）；
  矩阵：`.tmp_cache/initpose_fix/run_matrix.py`；原始记录：`.tmp_cache/initpose_fix/runs/<tag>/`。
* 复现（把本文两张表重新量一遍；`<BIN>` = 待测构建的
  `install*/gicp_registration/lib/gicp_registration/gicp_registration_node`）：

```bash
cd /home/weicheng/HzMi_rmsimulation
# ① 隔离构建（不碰共享 build/install；显式空 build type = 本包已验证的无优化基线）
source /opt/ros/humble/setup.bash
colcon build --packages-select gicp_registration \
  --build-base .tmp_cache/initpose_fix/build_before --install-base .tmp_cache/initpose_fix/install_before \
  --cmake-args -DCMAKE_BUILD_TYPE=
#    Release 口径把最后一行换成 -DCMAKE_BUILD_TYPE=Release
# ② 单场景（饱和：4000 点点云 @10 Hz；t=8 s 发一条 RELIABLE /initialpose；三个观测量自动记录）
python3 .tmp_cache/initpose_fix/driver.py --bin <BIN> --tag my_sat \
  --duration 30 --click-at 8 --click-pose '0.35 -0.35 0 10' --node-args '-p backend:=pcl'
# ③ 一键跑完整矩阵（延迟 12 次 + 契约 10 次；约 10 min）
python3 .tmp_cache/initpose_fix/run_matrix.py --phase all
# ④ 运行时 QoS 抽查（确认 /initialpose 仍是 RELIABLE）
python3 .tmp_cache/initpose_fix/qos_check.py <BIN> my_qos
```

### 4.2 结果（饱和场景：PCL 后端 400~490 ms/帧 @10 Hz）

`before` = 改动前的构建；`after` = 本次修复；两者都是**同一份实验代码、同一台机器、同一天、
同一个隔离构建树口径**（见 4.3）。

| 运行 | 点云 | 云帧 | `~/pose` | `/tf` map→odom | handoff 日志 | apply 日志 | `~/pose` 生效 | `/tf` 生效 | 点云停流(点击后) | SIGINT |
|---|---|---|---|---|---|---|---|---|---|---|
| **before** 饱和 r1 | 4000 点 @10 Hz | 272 | 66 | 1601 | — | 22.783 | 22.784 | 22.800 | 22.0 | 0 |
| **before** 饱和 r2 | 4000 点 @10 Hz | 271 | 65 | 1600 | — | 22.408 | 22.408 | 22.420 | 22.0 | 0 |
| **before** 饱和 r3（点击晚 4 s） | 4000 点 @10 Hz | 270 | 64 | 1600 | — | 18.478 | 18.479 | 18.480 | 18.0 | 0 |
| **before** 饱和 r4（点击晚 10 s） | 4000 点 @10 Hz | 271 | 62 | 1600 | — | 12.484 | 12.485 | 12.500 | 12.0 | 0 |
| **after** 饱和 r1 | 4000 点 @10 Hz | 270 | 58 | 1600 | 0.0004 | 0.162 | 0.162 | 0.180 | 22.0 | 0 |
| **after** 饱和 r2 | 4000 点 @10 Hz | 272 | 58 | 1601 | 0.0004 | 0.303 | 0.303 | 0.320 | 22.0 | 0 |
| **after** 饱和 r3（点击晚 4 s） | 4000 点 @10 Hz | 272 | 56 | 1601 | 0.0004 | 0.263 | 0.263 | 0.280 | 18.0 | 0 |
| **after** 饱和 r4（点击晚 10 s） | 4000 点 @10 Hz | 272 | 57 | 1600 | 0.0003 | 0.197 | 0.198 | 0.216 | 12.0 | 0 |

* 三个观测量在每一行里互相吻合到 ~2 ms（apply 日志 → `~/pose` → 下一个 `/tf` tick）⇒ 测量的不是
  "日志什么时候打"，而是"`map→odom` 什么时候真的变了"。
* `before` 的 22.7 s **不是"上限"而是"点云停流的时刻"**：`t_cloud_stop − t_click = 22.0 s`，
  应用发生在停流后 0.63~0.71 s ⇒ **饱和期间这个订阅根本没被服务过**（与 §6.5 的 13.2 s / 14 s 同因）。
* `after` 的延迟分布 = "离下一帧开头还有多久" ⇒ 期望 ≈ 半个 align 周期、上界 = 一个 align 周期
  （本机 align 455~483 ms ⇒ 实测 0.24 s 正落在期望值上；目标"≤ 0.5 s"达成）。

### 4.2b 对照：轻载场景 与 当前工作区的 `Release(-O3)` 口径

同一天、同一台机器、同一个探针，只换"单帧 align 有多贵"：

| 运行 | 点云 | 云帧 | `~/pose` | `/tf` map→odom | handoff 日志 | apply 日志 | `~/pose` 生效 | `/tf` 生效 | 点云停流(点击后) | SIGINT |
|---|---|---|---|---|---|---|---|---|---|---|
| **before** 轻载（400 点云） | 400 点 @10 Hz | 127 | 115 | 801 | — | 0.040 | 0.041 | 0.060 | 8.0 | 0 |
| **after** 轻载（400 点云） | 400 点 @10 Hz | 126 | 114 | 801 | 0.0002 | 0.041 | 0.041 | 0.060 | 8.0 | 0 |
| **before**‑Release(-O3) | 4000 点 @10 Hz | 271 | 272 | 1602 | — | 0.000 | 0.000 | 0.020 | 22.0 | 0 |
| **after**‑Release(-O3) | 4000 点 @10 Hz | 268 | 269 | 1602 | 0.0001 | 0.121 | 0.121 | 0.137 | 22.0 | 0 |

* **轻载（400 点云，align 组基本空闲）**：before 0.040 s / after 0.041 s ⇒ 交接版本**没有引入额外开销**
  （差别在噪声量级；两者的 `/tf` 生效都比 apply 日志晚 ~20 ms，那是 50 Hz 定时器的 tick 间隔）。
* **`Release(-O3)`**（就是当前工作区 `colcon_defaults.yaml` 生效后用户会拿到的口径：
  align ≈ 13~18 ms/帧 ⇒ 组占用 ~15%）：
  * before **0.000 s**（组基本空闲，点击回调立刻被服务）——**说明这个 bug 会被 `-O3` 大幅掩盖**；
  * after **0.121 s**（apply 日志）/ **0.137 s**（`/tf`）——上限 = "到下一帧点云开头"，
    在"点云 10 Hz > 单帧 align 13 ms"时由**点云周期**决定（100 ms），而不是由 align 决定。
* ⇒ 修复后的延迟**与 align 成本解耦**：`after ≈ min(下一帧开始) ≤ max(点云周期, 一个 align 周期)`；
  修复前是**无上界**的（只要 align 周期 ≥ 点云周期就会饥饿，实测 12.5~22.8 s 甚至永不处理）。
  代价是 `-O3` 口径下典型的 0.00 s 变成 ~0.12 s（人眼不可辨），换来任何负载/任何构建口径下都不再出现
  "点一次没反应十几秒"。

### 4.3 构建口径（重要：为什么用隔离构建树 + 显式 `-DCMAKE_BUILD_TYPE=`）

* 本仓库现在的**工作区根目录**多了一个 `colcon_defaults.yaml`（由并行的"构建口径审计"agent 添加，
  本轮**没有**碰它），它让从仓库根执行的 `colcon build` 一律带上 `-DCMAKE_BUILD_TYPE=Release`
  ⇒ 本包会以 `-O3 -DNDEBUG` 编译。而本包**已验证的基线**是"`CMAKE_BUILD_TYPE` 为空 ⇒ 编译命令里
  一个 `-O` 都没有"，单帧 PCL align 因此是 400~490 ms（`-O3` 下只要 13~18 ms，
  见 `docs/gicp_backend_small_gicp.md` §5 与 `docs/build_optimization.md`）。
* **饥饿问题的严重程度直接由单帧 align 决定**，所以本轮 A/B **必须**在"已验证的无优化基线"下做，
  否则测不出问题本身。做法：
  * 不碰共享的 `build/`、`install/`（另一个 agent 正在重建它们，且它们现在是 Release 口径）；
  * 用**隔离构建树**：`colcon build --packages-select gicp_registration
    --build-base .tmp_cache/initpose_fix/build_{before,after}
    --install-base .tmp_cache/initpose_fix/install_{before,after}
    --cmake-args -DCMAKE_BUILD_TYPE=`（显式空 build type ⇒ 覆盖 `colcon_defaults.yaml` 的 Release）；
  * 复核：`flags.make` 的 `CXX_FLAGS = -fPIC -Wall -Wextra -Wpedantic -fPIC -fopenmp -std=gnu++17`（无 `-O`）、
    `CMakeCache.txt` 的 `CMAKE_BUILD_TYPE:STRING=`；
  * 运行时不 `source` 任何 `install/setup.bash`，并把**实际 dlopen 到的** `libgicp_registration.so`
    路径记进每次运行的 JSON（节点是 `rclcpp_components` 生成的 main，靠 `class_loader` 按名字加载 so
    ⇒ 必须显式把隔离 install 的 `lib/` 放进 `LD_LIBRARY_PATH`）。
* 构建结果：**0 error / 0 warning**（`-Wall -Wextra -Wpedantic`；日志
  `.tmp_cache/initpose_fix/logs/build_{before,after_final}.log`）。
* 与真实使用的差异（**已实测，见 §4.2b 的 Release 两行**）：用户按仓库惯例执行
  `colcon build --symlink-install --packages-select gicp_registration` 时会吃到 `colcon_defaults.yaml`
  的 Release ⇒ align 变成 13~18 ms/帧、align 组占用率 ~15% ⇒ **旧代码的饥饿被这个构建口径大幅掩盖**
  （before-Release 点击延迟 0.000 s），而修复后是 0.121 s（由 10 Hz 点云周期决定）。
  也就是说：**当前构建策略让问题暂时看不见，本修复让它不再依赖构建策略**
  （点云更密、`maximum_iterations` 调高、机器更忙、或去掉该 policy 文件时，旧代码会立刻回到十几秒）。

---

## 5. 契约回归（before / after 并排实测）

运行时 QoS 抽查（`ros2` 图内实测，`qos_check.py`，before/after **逐字段相同**）：

| 端点 | before | after |
|---|---|---|
| sub `/initialpose` | `RELIABLE/VOLATILE/KEEP_LAST(10)` | `RELIABLE/VOLATILE/KEEP_LAST(10)` ✅ |
| sub `/livox/lidar/pointcloud` | `BEST_EFFORT/VOLATILE/KEEP_LAST(1)` | 同 |
| pub `~/pose` | `RELIABLE/VOLATILE/KEEP_LAST(1)` | 同 |
| pub `~/fitness_score` / `~/converged` | `RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(1)` | 同 |

**(i) `use_initial_pose:=false`、无 `/initialpose` ⇒ 契约：**发** TF 且 = 恒等**

| 版本 | map→odom 条数 | `~/pose` 条数 | 其它 `/tf` 变换 | 首个匹配的应用值 (x,y,z,yaw°) | 运行末 map→odom (x,y,z,yaw°) | 预期应用值 T_map_base·T_odom_base | apply 日志延迟 | TF 缺失 WARN | exit |
|---|---|---|---|---|---|---|---|---|---|
| before | 701 | 23 | 无 ✓ | — | (-0.1475, 0.1462, 0.0007, 0.0221) | (2.5000, -1.5000, 0.0000, 10.0000) | — s | — s | 0 |
| after | 702 | 24 | 无 ✓ | — | (-0.1475, 0.1462, 0.0007, 0.0221) | (2.5000, -1.5000, 0.0000, 10.0000) | — s | — s | 0 |

**(ii) 无 `odom→base` TF、无 `/initialpose` ⇒ 契约：**0 条 map→odom****

| 版本 | map→odom 条数 | `~/pose` 条数 | 其它 `/tf` 变换 | 首个匹配的应用值 (x,y,z,yaw°) | 运行末 map→odom (x,y,z,yaw°) | 预期应用值 T_map_base·T_odom_base | apply 日志延迟 | TF 缺失 WARN | exit |
|---|---|---|---|---|---|---|---|---|---|
| before | 0 | 0 | 无 ✓ | — | — | (2.5000, -1.5000, 0.0000, 10.0000) | — s | — s | 0 |
| after | 0 | 0 | 无 ✓ | — | — | (2.5000, -1.5000, 0.0000, 10.0000) | — s | — s | 0 |

**(iii) 无 `odom→base` TF + 有 `/initialpose` ⇒ 契约：丢弃 + WARN，0 条 map→odom**

| 版本 | map→odom 条数 | `~/pose` 条数 | 其它 `/tf` 变换 | 首个匹配的应用值 (x,y,z,yaw°) | 运行末 map→odom (x,y,z,yaw°) | 预期应用值 T_map_base·T_odom_base | apply 日志延迟 | TF 缺失 WARN | exit |
|---|---|---|---|---|---|---|---|---|---|
| before | 0 | 0 | 无 ✓ | — | — | (2.5000, -1.5000, 0.0000, 10.0000) | — s | 0.204 s | 0 |
| after | 0 | 0 | 无 ✓ | — | — | (2.5000, -1.5000, 0.0000, 10.0000) | — s | 0.205 s | 0 |

**(iv) `odom→base` 带偏置 ⇒ 应用值必须 = `T_map_base·T_odom_base`（纯数学可逐位核对）**

| 版本 | map→odom 条数 | `~/pose` 条数 | 其它 `/tf` 变换 | 首个匹配的应用值 (x,y,z,yaw°) | 运行末 map→odom (x,y,z,yaw°) | 预期应用值 T_map_base·T_odom_base | apply 日志延迟 | TF 缺失 WARN | exit |
|---|---|---|---|---|---|---|---|---|---|
| before | 800 | 25 | 无 ✓ | (1.5915, -0.8655, 0.0500, 22.0000) | (1.5915, -0.8655, 0.0500, 22.0000) | (1.5915, -0.8655, 0.0500, 22.0000) | 8.420 s | — s | 0 |
| after | 801 | 11 | 无 ✓ | (1.5915, -0.8655, 0.0500, 22.0000) | (-0.1486, 0.1484, -0.0001, 0.0129) | (1.5915, -0.8655, 0.0500, 22.0000) | 0.100 s | — s | 0 |

**(v) 真跑 GICP 并采纳 ⇒ 应用后下一帧被精修，before/after 收敛值比对**

| 版本 | map→odom 条数 | `~/pose` 条数 | 其它 `/tf` 变换 | 首个匹配的应用值 (x,y,z,yaw°) | 运行末 map→odom (x,y,z,yaw°) | 预期应用值 T_map_base·T_odom_base | apply 日志延迟 | TF 缺失 WARN | exit |
|---|---|---|---|---|---|---|---|---|---|
| before | 1101 | 41 | 无 ✓ | (0.3500, -0.3500, 0.0000, 10.0000) | (0.3500, -0.3500, 0.0000, 10.0000) | (0.3500, -0.3500, 0.0000, 10.0000) | 14.497 s | — s | 0 |
| after | 1100 | 34 | 无 ✓ | (0.3500, -0.3500, 0.0000, 10.0000) | (-0.1479, 0.1488, 0.0001, 0.0079) | (0.3500, -0.3500, 0.0000, 10.0000) | 0.365 s | — s | 0 |

**逐条结论**：

1. **(i) `use_initial_pose:=false` + 无 `/initialpose`**：两版都按契约**发** TF（701/702 条）且最终值
   **逐位相同** `(-0.1475, 0.1462, 0.0007, 0.0221)`（= 恒等起步后 GICP 收敛值）
   ⇒ "参数被忽略、恒等起步"的语义未变。（该场景没有点击，所以"预期应用值"列不适用。）
2. **(ii) 无 `odom→base` TF + 无 `/initialpose`**：两版都是 **0 条 map→odom、0 条 `~/pose`**，
   日志每 2 s 一条"尚无 map→odom：**不发 TF**" ⇒ "没有初值不发 TF"未变。
3. **(iii) 无 `odom→base` TF + 有 `/initialpose`**：两版都**丢弃**这次初值、打印同一条 WARN
   （`/initialpose 收到，但 TF odom→base_link 查不到 …⇒ 忽略本次初值`）、**0 条 map→odom**；
   WARN 相对点击的时刻从 0.204 s 变成 0.205 s（= 应用被搬到下一帧开头，差别就是"一帧"）。
4. **(iv) 组合语义（`odom→base` 带 0.3/-0.2/0.05/7° 偏置）**：两版在 `/tf` 上出现的应用值**逐位相同**
   `(1.591542, -0.865539, 0.050000, 22.000000)`，且与 `T_map_base·T_odom_base` 的解析计算**完全一致**
   ⇒ "`T_map←odom = T_map←base·T_base←odom`"这条数值契约一字未改。
   （after 那行"运行末"的值是 GICP 随后把它精修回去的结果——因为这次点击是在**点云持续期间**生效的，
   后续采纳帧继续跟踪；before 那次点击发生在点云停流时刻，之后没有帧了，所以停在原地点击值上。）
5. **(v) 真跑 GICP + 采纳**：两版在 `/tf` 上被点击应用出的值都是 `(0.35, -0.35, 0, 10°)`（= 点击值，逐位相同）；
   延迟 before **14.497 s** → after **0.365 s**。
6. **只发 `map→odom`**：所有 22 次运行里 `/tf` 上的非 `map→odom` 变换都是 **0 条** ✓。
7. **SIGINT**：所有运行 `exit=0`（`signal_handler(SIGINT/SIGTERM)` → 正常退出）✓。

---

## 6. 考虑过但否决的方案

1. **把 `/initialpose` 单独放一个回调组，但回调里照旧做完全部工作**（TF 查询 + 组合 + 写 `map→odom` + `~/pose`）。
   * 表面最简单（只改一行 `callback_group`），但**破坏了"初值写入与 align 读改写串行"这条不变量**：
     点击写 `T_map_odom_` 与在飞的 align 结果写 `T_map_odom_` 会竞争，谁后写谁生效 ——
     用户点了新位置，却可能被一帧旧初值的 align 结果覆盖回去；而且 `mutex_` 只能保护"单次赋值"，
     保护不了"TF 查询 → 组合 → 赋值"这一整段的原子性（在飞的 align 会在中途改掉 `T_map_odom_`）。
   * 要修就得让 align 持锁跑完 400 ms（把 TF 定时器一起拖住）或引入版本号/序列号，
     复杂度与风险都远高于"交接 + 帧首应用"。
   * 否决。
2. **把交接槽的消费放到 TF 定时器（50 Hz）里**（"反正定时器一直在跑，顺手应用一下"）。
   * 违反"应用必须在 align 路径"这一前提：`map→odom` 的写会来自定时器组，与 align 组并发；
     还要在定时器组里做 TF 查询（0.05 s×2 档超时）⇒ 直接威胁"50 Hz 不被拖慢"的契约。
   * 否决。
3. **让 `/initialpose` 用 `SensorDataQoS`/BEST_EFFORT 降低排队成本**。
   * QoS 是契约（RViz 发 RELIABLE；BEST_EFFORT 会静默丢点击）；而且饥饿是调度问题，
     换 QoS 只是让样本**更容易被丢掉**，不是修好。
   * 否决。
4. **把点云订阅改成"处理完再取下一帧"之外的花样**（如降低点云频率、丢帧策略）。
   * 都会动到配准的输入契约与定位频率，属于另一个决定。
   * 否决。

---

## 7. 回退

| 想要的效果 | 操作 |
|---|---|
| 回退本次改动（回到"点击当场应用、与点云同组"） | `git revert <本次 commit>`（只动 `src/rm_localization/gicp_registration/**` 与本文档） |
| 只想让点击更"即时"、不要交接 | 不可行：把 `initialPoseCallback` 改回当场应用=重新引入饥饿（13.2 s 级） |
| 观察修复是否生效 | 看启动 banner 的 `并发: … 三个回调组 …`；点击后日志先出现 `…收到（handoff）…`，紧接着下一帧出现 `…更新初值…`；两条的时间差 ≈ 一个 align 周期 |

---

## 8. 没能验证的事（诚实清单）

1. **真人 RViz 点击**：本轮用"持久 RELIABLE 发布者 + 单条消息"模拟（比 `ros2 topic pub --once` 更接近 RViz），
   但**没有真的开 RViz 点 2D Pose Estimate**（RViz 的 publisher 长期存在、点击时可能连发几条）。
2. **整栈（Gazebo + nav2 + LIO + gicp）**：本文所有数字都是**单节点**测量（隔离 domain、无 nav2/Gazebo）。
   整栈下 CPU 争抢更严重（align 会更慢）⇒ 交接延迟仍应是 µs 级、应用延迟仍应是"一个 align 周期"，
   但**没有实测**。
3. **点云不来的场景**：按 §3.4 差异 ①，此时点击会排队到恢复点云后第一帧；本轮只做了"点云停流后
   before 才被处理"的对照（before 侧），**没有**专门测 after 侧"停流期间点击 → 恢复后第一帧生效"的延迟。
4. **`small_gicp` 后端**：本轮 A/B 只跑了 `backend=pcl`（默认，也是问题最严重的路径）。
   `small_gicp` 单帧更短（约 236 ms）⇒ 饥饿更轻，但修复对两个后端是同一段代码（交接/应用不在后端的路径上）。
5. **不同机器 / 更忙的整栈负载**：本文数字是本机（28 线程 i7-14700HX，load ≈ 3~5）；
   构建口径只测了两档（无优化基线 + `Release`，见 §4.2/§4.2b），**没有**在 Gazebo+nav2+LIO 同机跑时实测
   （那时 align 会更慢，修复后的延迟上界会随"一个 align 周期"一起变大，但仍是有界的）。
6. **`tf_lookahead_sec` / nav2 侧**：本次没有重跑 nav2 回归（`--goal -1.0 2.0`），
   只验证了 TF 契约本身（只发 `map→odom`、时间戳公式未变）。
