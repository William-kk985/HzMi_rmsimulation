# COD 实车帧实测（ROS 2 Humble）：`T_bl` 非单位阵，`odom→base_link` 却 ≈ 恒等

> **STATUS（2026-10-18 实测 / 2026-10 落盘）**：本文是**实车原始数据的唯一落盘处**。它证伪了
> [`cod_vs_ours_frame_architecture.md`](cod_vs_ours_frame_architecture.md) §2/§3.3 的「COD 的
> `base_link ≡ livox_frame`（C 型，`T_bl≡I`）」结论，判定为 **A 型**（约定分叉、`T_bl` 非单位阵），
> 并证明上游的**共轭**写法在实车上**真实生效**（车身姿态差 ≈46°）。其余内容（H2/H3 的源码对账、
> 可避免耦合清单）仍有效。
>
> **数据来源**：【实测】由用户本人在**实车**（ROS 2 Humble）上执行 §1 的命令并抄录终端输出；
> 时间戳为录制时的系统时间（epoch → 本地 CST 见 §1.0）。**本文数字逐字照抄，未做任何"修正"或平滑**；
> 所有推论都标了【推断】，可复算的算术在 §2 给出。
>
> **记号**（⚠️ 两个约定差一个逆，先钉死）：
> * `P := T(base_link ← livox_frame)` = **livox 在 base_link 里的位姿** = §1.1 `tf2_echo base_link livox_frame` **打印的那个变换**。本文按**实车打印方向**记账，也按任务书把这一条叫 `T_bl`。
> * `T_ol := T(odom ← livox_frame)` = LIO 输出的位姿。
> * 于是 `T_ob = T(odom ← base_link) = T_ol · P⁻¹`（**合成/正确**）；上游的"共轭"写法是 `P · T_ol · P⁻¹`。
> * ⚠️ [`frame_contract_v3.md`](frame_contract_v3.md) §2.2 把 `T_bl` 定义成 `lookupTransform(livox_frame, "base_link")` = **base 在 livox 里** = `P⁻¹`，并写合成 `T_ol·T_bl`、共轭 `T_bl⁻¹·T_ol·T_bl`。**两套写法是同一件事的两种记账**（本文的 `P` = v3 的 `T_bl⁻¹`）；后文只在"静止 ⇒ 共轭退化成恒等"这类与方向无关的结论上直接对比。

---

## §0 三行结论（先给答案）

| # | 结论 | 一句话证据 | 性质 |
|---|---|---|---|
| **1** | **A 型确认**：实车的 `base_link` 与 `livox_frame` **不是同一个系**，`T_bl` **不是单位阵** | `tf2_echo base_link livox_frame` 的静态输出：translation `(0.100, 0.000, 0.382)`、RPY **`(-46.238°, -1.684°, 0.000°)`**，`At time 0.0` 连打 3 次 ⇒ 静态边 | 【实测】 |
| **2** | **上游共轭 bug 在实车上真实生效**：静止时 `odom→base_link` 的 roll ≈ **0.030°**（≈ 恒等），而**正确的合成**应给 **≈ +46.25°**（= `P⁻¹` 的 roll）⇒ 车体姿态错 ≈46°，且**不会被任何"看起来正常"的里程计掩盖** | §1.5 四帧实测 roll `0.030 / 0.028 / -0.015 / -0.003`；共轭在 `T_ol≈I` 时**恒等于**恒等（与 `P` 无关），合成则必须给出 `P⁻¹` ⇒ 实测只与共轭自洽 | 【实测】+【推断】 |
| **3** | **实车 TF 树里没有重力对齐帧，也没有重定位**：`map→odom` 是**静态**（rate 10000、stamp `0.000000`）⇒ 无 AMCL/GICP 在改它；`camera_init` 帧**不存在**；全树唯一动态边是 `base_link→base_link_fake`（10.2 Hz）与 `odom→base_link`（10.2 Hz） | §1.3 `view_frames` 输出 + §1.4 `topic list` | 【实测】 |

---

## §1 命令与原始输出（逐字）

### 1.0 录制时刻

| 命令 | 时间戳（epoch） | UTC | 本地（CST, +0800） |
|---|---|---|---|
| ① `tf2_echo base_link livox_frame` | `1792322629.138551812` | 2026-10-18 11:23:49 | 2026-10-18 19:23:49 |
| ③ `view_frames` 记录时刻 | `1792322945.2855859` | 2026-10-18 11:29:05 | 2026-10-18 19:29:05 |
| ⑤ `tf2_echo odom base_link` 四帧 | `1792323072.66…` ~ `1792323075.66…` | 2026-10-18 11:31:12–15 | 2026-10-18 19:31:12–15 |

> 【实测】时间戳直接取自终端输出（TF stamp / `view_frames` 的"recorded at time"）。⇒ 全部数据来自**同一次会话、同一台车**（§4 第 5 条）。

### 1.1 命令 ①：`ros2 run tf2_ros tf2_echo base_link livox_frame`

```text
[INFO] [1792322629.138551812] [tf2_echo]: Waiting for transform base_link ->  livox_frame: Invalid frame ID "base_link" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [0.100, 0.000, 0.382]
- Rotation: in Quaternion (xyzw) [-0.393, -0.014, -0.006, 0.920]
- Rotation: in RPY (radian) [-0.807, -0.029, 0.000]
- Rotation: in RPY (degree) [-46.238, -1.684, 0.000]
- Matrix:
  1.000  0.021 -0.020  0.100
  0.000  0.692  0.722  0.000
  0.029 -0.722  0.691  0.382
  0.000  0.000  0.000  1.000
```

**两点现场观察**（【实测】）：

1. 同一块输出**连续打印了 3 次**，每次都是 `At time 0.0` ⇒ 这是**静态（latched）变换**，不是随 `/tf` 动态更新的边（动态边的 `At time` 会是具体的 epoch，见 §1.5）。
2. 第一行 `Invalid frame ID "base_link" ... frame does not exist` 是**第一次查询时静态 TF 还没到**（`tf2_echo` 会先 `canTransform` 一次再持续打印）；随后的 3 块输出证明它到了。⇒ **这条 INFO 不是配置错误**，是启动时序噪声。

### 1.2 命令 ②：第一次 `tf2_echo odom base_link` / `tf2_echo map odom` —— **失败**（时序）

| 命令 | 现场抄录到的错误片段（逐字） | 当时状态 |
|---|---|---|
| `ros2 run tf2_ros tf2_echo odom base_link` | `Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist` | LIO **尚未启动** |
| `ros2 run tf2_ros tf2_echo map odom` | `Invalid frame ID "map" ... frame does not exist` | 同上；`map` 更是要等重定位/SLAM 起来 |

> 【实测】两条命令在那一刻都报 `frame does not exist`；等 LIO 起来后**同一批查询成功**（§1.3/§1.5）。
> 【推断】错误全文与 §1.1 的 INFO 同款（`tf2_echo` 对 `target_frame` 不存在时固定这句），故不另行补全整行，只保留抄录片段。
> **教训**：`tf2_echo` 的 "does not exist" 有两种含义——**真的没有这条边**，或**还没起来**。必须与 `view_frames`/`topic list` 交叉确认（§3.3）。

### 1.3 命令 ③：`ros2 run tf2_tools view_frames`（监听 5 s，记录于 `1792322945.2855859`）

```text
livox_frame:   parent 'base_link',  broadcaster 'default_authority', rate 10000.000, most_recent 0.000000, oldest 0.000000, buffer 0.000
base_link:     parent 'odom',       broadcaster 'default_authority', rate 10.200,  most_recent 1792322945.263167, oldest 1792322940.263103, buffer 5.000
odom:          parent 'map',        broadcaster 'default_authority', rate 10000.000, most_recent 0.000000, oldest 0.000000, buffer 0.000
base_link_fake:parent 'base_link',  broadcaster 'default_authority', rate 10.200,  most_recent 1792322945.263167, oldest 1792322940.263103, buffer 5.000
```

> 说明：【实测】`rate 10000.000` + `most_recent/oldest = 0.000000` = **静态**边的特征（`view_frames` 把 latched/静态边记成 10000 Hz、stamp 0）；
> `rate 10.200` + 5 s 的 `oldest→most_recent` 窗口 = **动态**边，实际约 **10.2 Hz**。
> `broadcaster 'default_authority'` 是 `view_frames` 在拿不到节点名时的占位（本机没给 `--debug`/`/tf` publisher 归属）⇒ **不能**据此判定发布者是谁。

### 1.4 命令 ④：`ros2 topic list | grep -E "odom|cloud|tf$"`

```text
/pointcloud_to_laserscan
/waypoint_follower
/cloud_registered
/odom
/tf
```

> 【实测】命中 5 条（`/pointcloud_to_laserscan` 是因为含 `cloud` 子串）。⇒ `/odom` 与 `/cloud_registered` 存在；
> **`camera_init` 不存在**（连话题/帧都没有），这与 `small_point_lio` 一系"world 帧就叫 `odom`"的设计一致（【推断】，见 §3.3 第 3 条）。

### 1.5 命令 ⑤：`ros2 run tf2_ros tf2_echo odom base_link`（LIO 已启动）

```text
At time 1792323072.664835453  Translation [0.013, -0.019, -0.044]  Quaternion [-0.000, -0.001, 0.007, 1.000]  RPY(deg) [0.030, -0.106, 0.850]
At time 1792323073.665161371  Translation [0.017, -0.018, -0.044]  Quaternion [-0.000, -0.001, 0.008, 1.000]  RPY(deg) [0.028, -0.101, 0.906]
At time 1792323074.664659738  Translation [0.018, -0.016, -0.045]  Quaternion [-0.000, -0.001, 0.008, 1.000]  RPY(deg) [-0.015, -0.109, 0.867]
At time 1792323075.665017366  Translation [0.017, -0.013, -0.045]  Quaternion [-0.000, -0.001, 0.008, 1.000]  RPY(deg) [-0.003, -0.075, 0.908]
```

> 说明：【实测】四条 `At time` 相差 1.0003 s ⇒ **约 1 Hz 打印**（`tf2_echo` 的显示节流），而边本身由 `view_frames` 记为 **10.2 Hz**。
> 【推断】这四帧来自**同一段静止/近似静止**的时刻：平移在 `(0.013~0.018, -0.019~-0.013, -0.044~-0.045)` 内小幅走动。

---

## §2 读数与算术核对（自己重算一遍）

### 2.1 `T_bl` 的量值

| 量 | 值 | 核对 |
|---|---|---|
| translation | `(0.100, 0.000, 0.382)` m | 照抄 |
| **`\|t\|`** | **0.3949 m** | `√(0.1² + 0.382²) = √(0.01 + 0.145924) = √0.155924 = 0.394872` ✔ |
| RPY | `(-46.238°, -1.684°, 0.000°)` | 照抄；radian 版 `(-0.807, -0.029, 0.000)`：`-0.807×180/π = -46.24°` ✔、`-0.029×180/π = -1.66°`（printed RPY 的二级耦合，见 2.2） |
| quaternion | `(-0.393, -0.014, -0.006, 0.920)` | 照抄；`\|q\| = 1.000540` ⇒ **打印只保留 3 位小数**（见 2.2） |
| 矩阵 | `R = [[1.000, 0.021, -0.020], [0.000, 0.692, 0.722], [0.029, -0.722, 0.691]]` | 行范数 `1.00042 / 1.00007 / 0.99980`、`det = 1.00030` ⇒ 在打印精度内是**旋转矩阵** ✔；第 2–3 行的 `0.692/0.722` 与 `R_x(-46.2°)` 的 `cos=0.6917 / sin=0.7222` **逐位吻合** ⇒ 主项是**绕 x 的 −46° roll** ✔ |

### 2.2 四舍五入带来的两个小偏差（【推断】，不影响结论）

* 由打印的 quaternion 反算 RPY 得 `(-46.307°, -1.746°, -0.002°)`，与打印的 `(-46.238°, -1.684°, 0.000°)` 差 **0.07°/0.06°**——因为 quaternion 只印到 3 位小数（`|q|=1.00054`）。**下文一律用打印的 RPY 为准。**
* radian 的 `-0.029` 反算 `-1.66°` 与 degree 的 `-1.684°` 差 0.02°——同一原因（radian 也只印 3 位）。

### 2.3 静止时"正确"的 `odom→base_link` 应该是多少

【推断，链式法则】实车的 `odom` 就是 **LIO 初始化时刻的 LiDAR/IMU 壳体系**（§1.3：`base_link` 的父级直接是 `odom`，`/cloud_registered` 与 `livox_frame` 同源；§3.3）。于是在**静止、刚初始化**时 `T_ol ≈ I`，正确合成给

```
T_ob = T_ol · P⁻¹ ≈ P⁻¹     （P = §1.1 打印的 T(base_link ← livox_frame)）
```

| 预测 | roll / pitch / yaw（deg） | translation（m） | 说明 |
|---|---|---|---|
| **正确合成** `T_ol·P⁻¹`（`T_ol=I`） | **+46.25 / +1.16 / +1.22** | **(−0.111, +0.274, −0.262)** | `P⁻¹` 的严格 ZYX 反演（roll ≈ +46.24° 与"取反 −46.238°"一致；pitch/yaw 是 46° 与 −1.684° 的**二阶耦合**，【推断】） |
| **上游共轭** `P·T_ol·P⁻¹`（`T_ol=I`） | **0 / 0 / 0** | **(0, 0, 0)** | 代数上 `P·I·P⁻¹ = I`，**与 `P` 无关** ✔ |
| **实测**（§1.5，四帧） | roll **0.030 / 0.028 / −0.015 / −0.003**（跨度 0.045）；pitch −0.075…−0.109；yaw 0.850…0.908 | (0.013…0.018, −0.019…−0.013, −0.044…−0.045) | 只与**共轭**自洽；离合成预测差 **≈46° / ≈0.39 m** |

> 方向说明：`P` 是"**livox 在 base 里**"；`T_ob`（base 在 odom 里）在 `odom ≡ livox` 时就等于 `P` 的**逆**。
> 若按 v3 的记号（`T_bl = P⁻¹`）写，就是 `T_ob = T_bl`、共轭 `T_bl⁻¹·T_ol·T_bl` —— **同一个结论，符号相反**。
> 所以判据只用**绝对值**：**≈46° vs ≈0.03°**。

> 【推断】实测 pitch `−0.075…−0.109°`、yaw `0.850…0.908°` 也都在 1° 内（roll 之外的残差来自 LIO 静止漂移与打印精度），
> 相对 46° 的判决量**小两个数量级**。⇒ **判决不受"取哪一帧/取哪个轴"影响**。

### 2.4 平移的残差（诚实登记，不参与判决）

【实测】四帧平移均值 ≈ `(0.016, −0.017, −0.045)` m、极差 ≤ 6 mm。它**不是 0**（共轭在 `T_ol=I` 的严格预测），**也不是** `(−0.111, +0.274, −0.262)`（合成的预测）——但它离 0 只有 **4.6 cm**，离合成预测有 **0.39 m**。

【推断】两点解释，二者可并存：
1. `T_ol` 在采样时**并非严格单位阵**：从初始化到采样已经跑了几分钟（§1.0：11:23→11:31），静止漂移让 `t_ol` 有厘米级偏置。按共轭反解 `t_ol ≈ R_pᵗ·(实测平移)` 得 `t_ol ≈ (0.015, 0.021, −0.044)` m（【推断】3 个分量都在 5 cm 内，符合"静止漂移"而不是"外参没乘对"）。
2. 本仓 `small_point_lio_node.cpp` 的**姿态用合成、平移故意保留旧（共轭）写法**（`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:196-220`，【引用】）——若实车跑的是同一份代码，平移本来就不该等于 `P⁻¹` 的真值。

⇒ **主判决用旋转、不用平移**：46.2° vs 0.03° 是 **1500 倍**的差；平移则只有 0.39 m 量级的 12%。§4 第 4 条已把它列进 caveat。

---

## §3 判决

### 3.1 A 型：`T_bl` 非单位阵，实车不是"`base_link ≡ livox_frame`"

【实测】`T_bl`（= §1.1 打印的 `P`）平移 `|t| = 0.3949 m`、roll `−46.238°`、且是**静态**边 ⇒ 实车的 `base_link` 是**另一个系**（不是雷达壳体系），雷达相对它**斜装 ≈46°、偏置 ≈0.39 m**。

> 独立核对【推断】：打印矩阵的**第 3 列** = livox 的 z 轴在 base 里的方向 ≈ `(−0.020, 0.722, 0.691)`，与 base z 的夹角 `arccos(0.691) = 46.26°` ⇒ 与 RPY 的 −46.238° **同量级同符号约定** ✔。

### 3.2 上游共轭 bug 在实车上**真实生效**，车身姿态错 ≈46°

【实测+推断】静止时实测 roll ≈0.03°：
* 若 LIO 用**合成**（本仓 `lio_tf_adapter` 的 `T_ol·P⁻¹`、`small_point_lio_node.cpp:195` 修好后的写法），应输出 **+46.25°** 的 roll；
* 若用**共轭**（上游 `small_point_lio_node.cpp` 原写法，`P·T_ol·P⁻¹`，即 v3 记号的 `T_bl⁻¹·T_ol·T_bl`），静止时**必然是 0°**（与 `P` 无关）。

⇒ 实车跑的是**共轭**，且因为 `T_bl` 非单位阵，**姿态误差 ≈46°**（不是"零"，也不是"可忽略的几度"）。
⇒ 这正是本轮仿真侧判决过的 bug ③（假俯仰；仿真里 `urdf` 档实测 **4.890°**，`T_bl` 的倾角越大误差越大）在实车 × **46°** 斜装下的放大版（【引用】[`tilted_lidar_fidelity.md`](tilted_lidar_fidelity.md) §I.5.3/§J.1；[`cod_vs_ours_frame_architecture.md`](cod_vs_ours_frame_architecture.md) §3.3）。

**后果（【推断】，按 nav2 语义逐条）**：
* nav2 的 `robot_base_frame`/`base_link` 在 `map`/`odom` 里被**转了 ≈46°** ⇒ 代价图、`/scan` 的落点、`base_link_fake` 的偏航全部跟着错；
* **对纯 2D 导航而言，一个恒定的 roll 偏置不会像"平移错"那样立刻撞墙**，但任何"用车身水平面做的判断"（高度带、坡度、p2l 的 2D 投影、避障外廓）都建立在错的姿态上；
* 因为误差是**恒定**的（静止 0.03°），它**不会**在日志里表现为"漂移"或"抖动" ⇒ **静默**，与仿真侧的结论一致（"共轭只在 `T_bl≡I` 时不可观测"）。

### 3.3 TF 树的其余事实（【实测】）

| 边 | parent → child | 类型 | 速率（`view_frames`） | 含义 |
|---|---|---|---|---|
| `livox_frame` | `base_link → livox_frame` | **静态** | 10000.000（stamp 0） | 雷达安装外参；就是 §1.1 的 `T_bl` |
| `base_link` | `odom → base_link` | **动态** | 10.200 Hz | 里程计；内容 = 共轭（§3.2） |
| `odom` | `map → odom` | **静态** | 10000.000（stamp 0） | **没有重定位**：没有任何节点在改写 `map→odom` |
| `base_link_fake` | `base_link → base_link_fake` | **动态** | 10.200 Hz | 小陀螺/云台 yaw 替身（仅 yaw，【推断】同本仓 `fake_vel_transform` 语义） |

* **没有重力对齐帧**：全树 4 个帧名里没有 `odom_grav` / `world` / `world_grav` / `odom_gravity` 一类（§1.3 原文只有 `livox_frame`/`base_link`/`odom`/`base_link_fake`）⇒ 【实测】"插一个重力中间帧"不是这套实车的做法。
* **`camera_init` 不存在**（§1.4）⇒ 本仓 `tf_bridge_camera_init_to_map` 那种"全 0 桥"在这套实车上**连帧都没有**（【推断】它对 `small_point_lio` 一系本来就不适用，见 [`frame_contract_v3.md`](frame_contract_v3.md) §10.2）。
* `map→odom` 静态 ⇒ **`initial_pose:=continue`/重定位那套在实车上等于"永远信任出生点"**；`map` 与 `odom` 只差一个常数（【推断】若该常数非 0，则 `map` 不是出生点系；本次没记 `map→odom` 的数值 ⇒ §6 待补）。

### 3.4 与仿真侧的对照（一句话）

【推断】本仓仿真里 `robot11_mount:=urdf|sensor` 的 `T_bl` 是 `R_x(−30°)` + `|t|=0.2044 m`；实车是 **−46.2°** + **0.3949 m**。**形态相同、数值更大** ⇒ 仿真里那 4.890° 的假俯仰在实车上会放大到 **≈46°**（不是同一台车/同一支架，不能直接换算，但方向与机理一致）。

---

## §4 Caveats（诚实清单，按重要性）

1. **`/odom` 的 quaternion 与 `/cloud_registered` 的 `frame_id` 没抓到**【实测的缺口】：同一会话里两条 `ros2 topic echo` 命令**失败**——`ros2cli` 的 numpy 导入在该 shell 坏了，报 `Traceback ... import numpy ... KeyboardInterrupt`。⇒ 我们**没有**直接证据说明 `/odom` 的 `child_frame_id` 是 `base_link` 还是 `livox_frame`，也**没有** `cloud_registered` 的 `frame_id` 原文。§3.2 的判决只依赖 **TF 实测**，不依赖这两条。
2. **时序坑**：§1.2 的两次 `frame does not exist` 是 **LIO 还没起来**导致的，不是缺边；§1.1 的 `Invalid frame ID "base_link"` 是**静态 TF 晚到一步**。两类"假故障"都很容易被误读成配置错。
3. **只有一次会话、一台车**：所有数字来自 2026-10-18 19:23–19:31（CST）的一次上电；**没有重复测量**，没有第二台车/第二个支架的对照。⇒ 不能据此给出 `T_bl` 的**统计**误差（如 ±0.5°），只能给"这一次的量值"。
4. **平移残差**（§2.4）：四帧平移 ≈`(0.016, −0.017, −0.045)` 既不 0 也不等于 `t_bl`；它可能来自 LIO 静止漂移，也可能来自"姿态合成 + 平移共轭"的混搭实现。**判决只用旋转**。
5. **打印精度**：quaternion 与 radian 都只印 3 位小数（§2.2），反算有 0.06–0.07° 的差；**degree RPY 也是 3 位小数**（0.001° 分辨率）。
6. **`broadcaster 'default_authority'` 不能当发布者证据**（§1.3）：`view_frames` 没给出 `/tf` 的 publisher 归属；"谁发的某条边"在本文里全部是【推断】（依据帧名/话题/本仓同源代码），**没有** `ros2 node info` 的实测。
7. **没测的东西**：没有 `ros2 topic hz /tf`（边速率来自 `view_frames` 的 5 s 窗口）；没有 `tf2_echo map odom` 的**数值**（只知道它是静态）；没有重力方向/水平度实测；没有核对实车 URDF/CSV 的安装角与 `−46.238°` 是否同源（【推断】"−46.2° 与 30° 斜装不是同一台车"）。

---

## §5 这如何改变我们的文档

| 文档 / 结论 | 原说法 | 本实测后的说法 | 性质 |
|---|---|---|---|
| [`cod_vs_ours_frame_architecture.md`](cod_vs_ours_frame_architecture.md) §2、§3.3、§6.1 ①/② | COD 的 `base_link ≡ livox_frame`（**C 型**，`T_bl≡I`）；共轭被 `T_bl≡I` **掩盖**、不可观测 | **证伪**：实车 `T_bl` 非单位阵（`roll −46.238°`、`\|t\| 0.3949 m`）⇒ **A 型**（约定分叉）；共轭 bug **可见且已生效**（车身姿态错 ≈46°） | 【实测】+【推断】 |
| 同上 §3.6 / §8 选项 A（"全面采用 COD 约定"） | "COD 靠 `T_bl≡I` 免疫，学它成本低" | 【推断】前提不成立：**COD 的实车并不免疫**。选项 A 的"零成本"论据要重估——除非他们的 **nav 不用这条 `odom→base_link`**（未验证，见 §6） | 【推断】 |
| [`docs/README.md`](README.md) 取代表 #11 | "COD 的 `base_link ≡ livox_frame`（C 型，`T_bl≡I`）→ 实车实测 A 型" 但**本仓无落盘实测** | 本文即该实测的落盘；取代表 #11 的"⚠️ 该实测本仓尚无落盘文档"至此**已解决**（README 的待决 ⑤ 同步） | 【实测】 |
| [`frame_contract_v3.md`](frame_contract_v3.md) I2 / §13（实现状态） | "`small_point_lio` 槽位自己发 `odom→base_link`（已知例外）" | 【推断】实车上"自己发"的内容是**共轭**（错 46°）；这给 I2（adapter 唯一权威）提供了**实车侧**的动机，而不只是仿真洁癖 | 【推断】 |
| [`tilted_lidar_fidelity.md`](tilted_lidar_fidelity.md) §I.5.3/§J.1 | 仿真 `urdf` 档假俯仰 **4.890°**（`T_bl = R_x(−30°)`） | 实车 `T_bl = −46.2°` ⇒ 同类误差放大到 ≈46°；**机理同一**，数值不同车/不同支架 | 【引用】+【推断】 |
| [`odom_base_link_practice_survey.md`](odom_base_link_practice_survey.md) §9（18 变体） | "参数法不能把世界系摆正" | 实车侧补一条：**连"合成 vs 共轭"这一步都还没对**（比参数层更早的一层） | 【推断】 |

> **一句话**：本文把"COD 是 C 型"这条**推断**换成了"实车是 A 型"这条**实测**；`cod_vs_ours_frame_architecture.md`
> 的源码对账（H2/H3）与耦合清单仍然有效，但**凡是把"COD 不受影响"当归宿的段落都要按 A 型重读**。

---

## §6 未验证 / 待补（下一次上实车时按顺序做）

1. **`ros2 topic echo /odom --once`**：确认 `header.frame_id` / `child_frame_id`（本次 numpy 坏了没抓到）。期望 `odom`/`base_link`（若 `child_frame_id=livox_frame` 则 §3.2 的判决要重写）。
2. **`ros2 topic echo /cloud_registered --once --field header.frame_id`**：确认点云表达在哪个系（`livox_frame` vs `odom`）。
3. **`ros2 run tf2_ros tf2_echo map odom`**：记下 `map→odom` 的常数（本次只知道它静态）。
4. **`ros2 run tf2_ros tf2_echo base_link base_link_fake`**：确认"仅 yaw"（【推断】）以及 `spin_speed` 的实际值。
5. **`ros2 node info` 逐节点**：把 `default_authority` 换成真实发布者，验证 §1.3 的归属推断。
6. **`ros2 run tf2_ros tf2_echo odom base_link` 在"车摆正 + 刚初始化"时再采 10 帧**：确认 roll 是否严格 0（共轭）还是 46°（合成），并把 §2.4 的平移残差分离出来。
7. **对照实车 URDF/CSV**：`−46.238°` 与图纸/装配是否同源（【推断】本文只记录了 TF 读数，没有核对安装图纸）。
8. **水平度**：用车体水平仪/IMU 静态读数交叉验证 `−46.2°` 是"安装倾角"而不是"`base_link` 定义本身斜"。

---

## 附：本文的复现命令清单（原样）

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame          # ① 静态外参（等 3 块 At time 0.0）
ros2 run tf2_ros tf2_echo odom base_link                # ② 先失败（LIO 没起）→ ⑤ 成功（四帧）
ros2 run tf2_ros tf2_echo map odom                      # ② 先失败；起来后成功（本次未记数值）
ros2 run tf2_tools view_frames                          # ③ 5 s 监听 → frames_*.gv/.pdf
ros2 topic list | grep -E "odom|cloud|tf$"              # ④ 话题清单（camera_init 不存在）
```
