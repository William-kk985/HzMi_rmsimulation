# 机器人模型槽位：`robot:=<模型>`（含用户那份哨兵 URDF 的实测评估）

> 状态：**新增（2026-10-07）**，opt-in；`robot` 留空 = 本仓现行默认模型，**默认路径逐字节未改**。
> 一句话：用户给的 `hzmirmvision-master/configs/sentry_robot.urdf` 是一份**纯描述文件**
> （无惯性 / 无 gazebo / 无插件 / 无 IMU / 无 mesh），不能直接进 Gazebo；本仓把它接成
> `robot:=hzmirm` 槽位（运动学逐字保留 + 我们补的仿真件），**实测能跑通全链路**，
> 但**不建议**作为默认模型替换它 —— 它的雷达装在云台头上、离地 **0.80 m**，而 MID360 下视只有
> **−7.22°** ⇒ **6.3 m 以内的地面完全看不见**（实测 `/segmentation` 地面点数 = 0、`/scan` 近场全空）。
> 详细数字、机理与"要不要换/怎么补救"见 §3。
>
> 相关文档：`docs/README.md`（索引）、`docs/worlds.md` §5（无头实测口径）、
> `docs/lio_slots.md` §2（LIO 输入契约与杆臂）、`docs/params_ownership_checklist.md`（参数该放哪个包）、
> `docs/ground_segmentation_slots.md`（linefit/patchwork 槽位）、`docs/tf_interface_contract.md`（帧树契约）。

---

## 0. 快速入口

| 我想… | 看 |
|---|---|
| 一条命令试这个模型 | §6.1（`robot:=hzmirm`） |
| 知道它到底写了什么 / "雷达是不是斜放的" | §1.2 |
| 知道哪些是它的、哪些是我们补的 | §2.2 provenance 表 |
| 知道为什么"能跑但不好用" | §3.2（0.8 m + −7.22° ⇒ 近场盲环） |
| 知道换模型会让哪些配置失效 | §4（差异表 + 一条命令试新值） |
| 看旧/新模型的实测对照数字 | §5 |
| 出了事怎么退回去 | §6.2 |
| 哪些还没验 | §7 |
| **用户 `robot11` 的 12 个 STL 在哪 / 各自多少面 / 多大** | §9.1–§9.2（Phase 1）|
| **`base_link.STL`（208 万面）能不能当碰撞 / 代价多少** | §9.3（实测 RTF 0.203、峰值内存 760 MB）|
| **`robot11` 的足印/高度/轴距/雷达离地** | §9.4 |
| 雷达 30° 到底是 roll 还是 pitch | §9.5（**仰角测不出来**，只有方位能区分）+ §10.5 实测 |
| **怎么试 `robot:=robot11` / 它跑得怎么样** | §10.1（命令）+ §10.4（实测表）+ §10.8（结论）|
| **`robot:=robot11` 要不要手工设 `GAZEBO_MODEL_PATH`** | **§14：不用了**（2026-10-08 起 launch/包两侧都自动；实测见 `docs/gazebo_gui_troubleshooting.md` §5.1）|
| 它能不能当默认模型 | §10.8（**现在还不是**，三条实测理由）| 

---

## 1. 用户给的那份 URDF 是什么（逐字事实）

### 1.1 来源与校验

| 项 | 值 |
|---|---|
| 网页 | `https://github.com/William-kk985/hzmirmvision-master/blob/main/configs/sentry_robot.urdf` |
| 原始（curl 可用） | `https://raw.githubusercontent.com/William-kk985/hzmirmvision-master/main/configs/sentry_robot.urdf` |
| 字节 / 行数 | **3761 B / 156 行** |
| sha256 | **`9463265182aa320322996a08dc1cbf4c964a7e1528835f495290f9ea477da98f`** |
| 仓库形态 | 公开仓库、**只有一个分支 `main`、没有 tag**（`api.github.com/.../branches`、`/tags`）|
| 全仓文件数 | 1082（`git/trees/main?recursive=1`，`truncated=false`）|
| 全仓 URDF 数 | **1**（就是这一份）；无 xacro、无 `.gazebo`、无 SDF/world、无 mesh |
| 本仓逐字节副本 | `src/rm_nav_bringup/urdf/upstream/hzmirm_sentry_robot.urdf`（**只读、不参与运行**，只用于核对）|

取回时 `raw.githubusercontent.com` 有过间歇性 TLS 失败（`curl: (35) unexpected eof`），
`--retry 3` 后成功；上述 sha256 与远端逐字节核对过（`cmp` 一致）。
核对命令（**一条命令复现**）：

```bash
curl -s --retry 3 --retry-all-errors -o /tmp/up.urdf \
  https://raw.githubusercontent.com/William-kk985/hzmirmvision-master/main/configs/sentry_robot.urdf
sha256sum /tmp/up.urdf   # 期望 9463265182aa320322996a08dc1cbf4c964a7e1528835f495290f9ea477da98f
cmp /tmp/up.urdf src/rm_nav_bringup/urdf/upstream/hzmirm_sentry_robot.urdf && echo 逐字节相同
```

### 1.2 它写了什么（逐字）

机器人名 `sentry_robot`，**9 个 link / 8 个 joint**：

```
base_link ──(base_to_turret, fixed, xyz 0 0 0.3)──> turret_base
          └─(base_to_wheel_{fl,fr,rl,rr}, fixed, xyz ±0.22 ±0.25 0.08, rpy 1.5708 0 0)──> wheel_*
turret_base ──(turret_to_head, fixed, xyz 0 0 0.3)──> turret_head
turret_head ──(head_to_barrel, fixed, xyz 0.15 0 0.1, rpy 0 0 0)──> barrel
turret_head ──(head_to_lidar,  fixed, xyz 0 0 0.2,   rpy 0 0 0)──> lidar_link
```

几何（全部逐字）：底盘 `box 0.6×0.6×0.3 @z=0.15`；云台支架 `0.2×0.2×0.3 @0.45`；
云台头 `0.3×0.3×0.2 @0.7`；炮管 `cylinder r=0.03 l=0.3 @(0.35,0,0.7) rpy=0 1.5708 0`；
雷达 `cylinder r=0.05 l=0.05 @z=0.825`；四个轮 `cylinder r=0.08 l=0.05`。

**⚠️ 对"雷达是斜防/斜放的"这条猜想的更正（重要）**：

| 用户的说法 | 文件里的事实 | 出处 |
|---|---|---|
| 雷达"斜放" | `head_to_lidar` 的 `rpy="0 0 0"` —— **没有静态倾斜** | 上游 URDF 第 96–101 行 |
| 雷达装在会动的云台上 | `base_to_turret` 与 `turret_to_head` **都是 `type="fixed"`** ⇒ 在这份文件里云台**不会动**，雷达是**刚性固定在 `base_link` 上**的 | 上游 URDF 第 38–42、57–62 行 |
| （文件里唯一那个 1.5708） | 出现在 `head_to_barrel` 的 `rpy`（炮管放平），**不是雷达** | 上游 URDF 第 78–85 行 |

⇒ "斜放"这件事**在这份 URDF 里不存在**。但用户的直觉在物理上是**有道理**的：
MID360 的下视只有 −7.22°，装到 0.8 m 高就看不见近处地面（§3.2）——真车多半正是靠**下俯**来补。
所以本槽位把"斜放/云台"做成**可试的参数**（`turret_yaw_deg` / `turret_pitch_deg`，默认 0 = 与上游
数值等价），并实测了它的后果（§3.3）。

### 1.3 它**没写**什么（本栈需要的，全都没有）

| 缺的东西 | 后果（都是实测/可复现的） |
|---|---|
| **`<inertial>`：9 个 link 一条都没有** | Gazebo Classic 走 sdformat URDF2SDF：**无惯性的 link 会被整条丢掉**。实测 `gz sdf -p`：整份文件只剩 `<model name='sentry_robot'/>`（空模型）；父 link 有惯性、子 link 没有时，**子 link 连同它的关节一起消失**。⇒ **直接 spawn 进 Gazebo 是跑不起来的**（这是"它缺什么"里最硬的一条） |
| `<gazebo>` / `<sensor>` / `<plugin>` | 没有雷达、没有 IMU、没有底盘驱动 ⇒ 一个 ROS 话题都不会有 |
| `imu_link` | 本栈 LIO（FAST-LIO/Point-LIO/small_point_lio）与 `imu_complementary_filter` 都要它 |
| `livox_frame`（或任何雷达帧名） | 本栈的 `lidar_frame: livox_frame`、`base_link→livox_frame` 静态 TF 都必须存在 |
| 任何 mesh / `filename=` | 全文件 0 个 `filename=`；只有 box/cylinder 基本体 |
| world / launch / 插件配置 | 仓库里没有任何 Gazebo 相关文件 |
| `<collision>`（除 `base_link` 外） | 只有底盘那一个 box 有碰撞；轮子、云台、炮管、雷达**只有 visual** ⇒ 物理上就是"一个 0.6×0.6×0.3 的盒子在滑"（这解释了 §3.4 / §5 里的倾角现象） |

### 1.4 同仓其它相关材料（都取到了，结论：没有更完整的版本）

| 路径 | HTTP | 是什么 | 对本槽位的用处 |
|---|---|---|---|
| `simulation/README.md` | 200 | "哨兵导航模拟器"说明 | **无用**：它自述"激光来源 = **地图射线追踪**、里程计 = 模拟积分、定位 = AMCL" |
| `simulation/robot_simulator.py` | — | 假仿真（假激光 + 预设动态障碍） | **无用**：不涉及 Gazebo/URDF/插件 |
| `simulation/start_sim.sh` / `nav2_sim_params.yaml` | 200 | 启动脚本 / nav2 参数 | 只作参考 |
| `configs/sentry.yaml`、`configs/fastlio_mid360.yaml`、`configs/nav2_params.yaml` | 200 | 真机侧参数 | 真机参数，与 URDF 无耦合 |
| `configs/livox_mid360.json`、`configs/MID360_config.json` | 200 | Livox 驱动配置（IP/外参） | 与仿真无关 |
| `fast_lio/`、`third_party/livox_ros_driver2-master/`、`FAST-LIVO2/` | — | 上游 vendored 代码副本 | 与本仓已有的同类包重复，不引入 |

**还缺什么 / 需要用户提供什么**（如果确实存在"更完整的那一份"）：
1. 若真车用的是 **xacro**（含 `<gazebo>`/插件/云台关节/`imu_link`）——请给 **那个 `.xacro` 文件本身**
   （以及它 `include` 的其它文件）；
2. 若模型用了 **mesh**（`.stl`/`.dae`）——请给模型目录（含 `meshes/`），否则我们只能用基本体近似；
3. 若云台**真的会动**——请给云台的**关节名/类型/限位/控制话题**（本仓不猜控制接口，
   见 §3.1 为什么不做 (c)）；
4. `configs/sentry_robot.urdf` 若已更新——请给新的 sha256 或直接给文件（我们按 §1.1 的流程重新核对）。

---

## 2. 本仓怎么接的：一个 opt-in 槽位 + 逐元素 provenance

### 2.1 槽位

| 位置 | 参数 | 默认 | 取值 |
|---|---|---|---|
| `rm_nav_bringup/launch/bringup_sim.launch.py`（**正式入口**） | `robot` | `''` | `''` = 现行默认模型 `rm_nav_bringup/urdf/sentry_robot_sim.xacro`；`hzmirm` = `rm_nav_bringup/urdf/sentry_robot_hzmirm_sim.xacro` |
| 同上 | `turret_yaw_deg` / `turret_pitch_deg` | `0` / `0` | 只对 `robot:=hzmirm` 生效；单位度；0 = 与上游 URDF **数值等价** |
| `hzmi_rm_simulation/launch/rm_simulation.launch.py`（只起 Gazebo+模型） | `robot` | `''` | 同上（⚠️ 该文件不含感知/LIO ⇒ 选 `hzmirm` **不会**自动切标定，完整链路请走 `bringup_sim.launch.py`）|

**"默认路径没变"的三条证据**（都可复现）：
1. `robot` 留空时，`robot_description` 拼出的命令字符串与改造前**逐字符相同**：
   `xacro <share>/rm_nav_bringup/urdf/sentry_robot_sim.xacro xyz:="…" rpy:="…"`（单元级比对见 §8 工具②）；
2. 默认模型读的参数文件**还是** `linefit_ground_segmentation_ros/config/segmentation_sim.yaml`（那份文件
   **一个字都没改**）；默认模型跑的 `lio_tf_adapter` 分支**还是**原来那条（`xyz: [-0.12,0,-0.125]`）；
3. 只有 `robot:=hzmirm` 才会切换：linefit 参数文件 → `segmentation_sim_hzmirm.yaml`；
   `lio_tf_adapter` → 另一条互斥分支（`xyz: [0,0,-0.75]`）。两者都是"判据不变、只换一条边"。

槽位实现是**惰性**的（`Substitution.perform()` 里才求值）⇒ 不选这个槽位时零成本，
`--show-args` 与其它槽位组合都不受影响（与本文件里 `_PackageShareFile` 同一套设计原则）；
选错值会抛一条列出可用取值的 `RuntimeError`。

### 2.2 provenance 表（哪些是他们的、哪些是我们加的）

`tools/scripts/regress/check_hzmirm_urdf_provenance.py` **逐 link/joint 断言**：
上游 9 个 link / 8 个 joint 的名字、类型、父子、`xyz`、`rpy`、几何尺寸、材质颜色
在我们的模型里**数值相等**（容差 1e-9），差异只可能来自下表"我们加的"那些元素。

| 元素 | 来源 | 说明 |
|---|---|---|
| 9 个 link、8 个 joint（名字/类型/父子/xyz/rpy/几何/颜色） | **上游原样** | 一个字段都没改；`head_to_barrel` 的 `rpy 0 1.5708 0` 也照抄 |
| `base_to_turret` 的 yaw、`turret_to_head` 的 pitch | **我们参数化** | 写成 `${turret_yaw_rad}` / `${turret_pitch_rad}`；默认 0 ⇒ 数值等于上游的 `0 0 0`（脚本按数值断言，不看字符串） |
| **10 条 `<inertial>`**（9 个上游 link + `imu_link`） | **我们加的** | 不加就 spawn 不出来（§1.3 第一条）。质量取"与默认模型同一套占位值"：底盘 8.2、轮 0.5×4、云台 1.0×2、炮管 0.5、雷达 0.4、IMU 0.01 ⇒ Gazebo 里合成质量 **13.11 kg**（`gz sdf -p` 可核）；转动惯量一律 0.01（占位，**不是实测值**，见 §7） |
| `livox_frame` link + `livox_frame_joint`（fixed，`xyz 0 0 0`） | **我们加的** | 与上游 `lidar_link` **重合**：既保留"雷达在哪 = 上游说的位置"，又给出本栈要求的帧名。用本仓既有宏 `ros2_livox_simulation/urdf/mid360.xacro` 挂 **MID360 射线传感器**（100×360、10 Hz、0.1–200 m、σ=2 mm）+ `libros2_livox.so` 插件（话题 `/livox/lidar` = CustomMsg、`/livox/lidar/pointcloud` = PointCloud2；cloud 的 `frame_id` = 传感器名 = `livox_frame`） |
| `imu_link` + `imu_joint`（fixed，父 = `lidar_link`，`xyz 0 0 −0.05`） | **我们加的** | 位置**故意与默认模型同构**（IMU 在雷达下方 0.05 m）⇒ `small_point_lio`/`fast_lio` 参数里的 `extrinsic_T=[0,0,0.05]`（雷达在 IMU 系下）**不用改**。绝对高度 `base_link+0.75`；配 `gazebo_ros_imu_sensor`（100 Hz，`/livox/imu`，`frame_name=imu_link`） |
| `<plugin name="mecanum_controller" filename="libgazebo_ros_planar_move.so">` | **我们加的**（与默认模型逐字同款） | `cmd_vel→/cmd_vel_chassis`、`odom→/odom_ground_truth`、`publish_odom_tf=false`（T4：`odom→base_link` 归 LIO）⇒ **契约不变** |
| 13 个 `<gazebo reference=…><material>` | **我们加的** | 纯观感 |
| `<xacro:arg name="xyz"/"rpy">`（声明但不用） | **我们加的** | `bringup_sim.launch.py` 会无条件传平台外参；本模型的雷达位姿由上游关节链唯一决定，再叠一个外部平移就等于改上游几何 |

### 2.3 实测几何参数（本仓配置会依赖的量）

| 量 | hzmirm（本槽位） | 默认模型 | 怎么量的 |
|---|---|---|---|
| 车体 | **0.6 × 0.6 × 0.3** m（外接半径 **0.424** / 内切 **0.30**） | 0.2 × 0.3 × 0.1（半径 ≈0.18） | 上游 URDF 逐字 + TF 实测 |
| 炮管外伸 | 到 **x=+0.35 m**（3D 碰撞里没有它，只有底盘 box 有碰撞） | — | SDF（`gz sdf -p`）|
| 轴距 / 轮距 | **0.44**（x: ±0.22）/ **0.50**（y: ±0.25） | 0.20 / 0.26 | TF `base_link→wheel_fl` 实测 `[0.22, 0.25, 0.08]` |
| 底盘高 | 0.30 m（`base_link` 原点就是**最低点**，`z=0`） | 0.10 m（原点在轮心，轮半径 0.06） | URDF/SDF |
| **雷达离地** | **0.80 m**（`base_link+0.80`；实测地面峰 `z=−0.790`） | 0.235 m 几何 / **0.226 m 实测**（现行标定值） | ① TF `base_link→livox_frame = [0,0,0.8]`；② 点云 z 直方图地面峰 |
| IMU 离地 | 0.75 m（`base_link+0.75`） | 0.125 m | TF `base_link→imu_link` |
| 质量/惯量 | 13.11 kg（合成，占位惯量 0.01） | 8.2+4×0.5+0.01 ≈ 10.2 kg | `gz sdf -p` |
| 静止高度（实测） | RMUL2026 出生点 `z=0.0496`；RMUC2026 `z=−0.000` | `z=0.110` / `0.060` | `/odom_ground_truth`（= Gazebo 模型位姿）|
| 物理形态 | **固定关节全被 Gazebo 合并成 1 个 link**（`base_link`），碰撞体 = 底盘 box + IMU 小方块（2 个），**轮子没有碰撞** ⇒ 物理上"平底盒子在滑" | 5 个 link（4 轮 continuous 未合并）、6 个碰撞体 ⇒ 轮子着地 | `gz sdf -p` 的 `*_fixed_joint_lump__*` |

> 实测细节：RMUL2026 出生点 `(4.3,3.35)` 处**地面在 `z≈0.05`**（由两台车的静止高度反推：
> 默认模型 0.05+0.06=0.11、hzmirm 0.05+0=0.0496）；RMUC2026 出生点地面在 `z≈0`
> （默认模型 0.06、hzmirm −0.000）。

---

## 3. 雷达安装方式的决策与感知后果（本槽位的核心）

### 3.1 决策：选 (a)，并把 (b) 实现好；(c) 不做

| 方案 | 决定 | 理由 |
|---|---|---|
| **(a) 雷达固定在底盘上** | ✅ **采用（默认）** | **这不是"偏离上游"，而正是上游文件写的东西**：`base_to_turret`/`turret_to_head` 都是 `fixed`、`head_to_lidar` 的 `rpy=0` ⇒ 这份 URDF 里的雷达本来就是刚性固定、重力对齐的。所以本槽位**逐字保留**它的关节链（云台头、支架、炮管都还在，视觉上仍是"雷达在云台头上"），不改一个关节类型 |
| **(b) 挂在云台头上、但用重力对齐帧做感知** | ✅ **已实现并实测验证** | 本槽位的 linefit 参数文件把 `gravity_aligned_frame` 由 `""` 改成 `"base_link"`（上游语义：**只旋转、不平移**地把点云转到该帧姿态，见上游 README + `ground_segmentation_node.cc:114-136`）。默认（云台角 0）时这是恒等旋转 ⇒ 结果不变；一旦 `turret_pitch_deg≠0`，它就是**唯一**能让地面拟合继续工作的开关（A/B 见 §3.3）。代价 = 每次回调多一次 TF 查询（`base_link→livox_frame` 是静态 TF，永远可用） |
| **(c) 驱动云台让雷达保持水平** | ❌ 不做 | 需要新增一个控制节点/控制器（本仓"不加 watchdog/新节点"的纪律），而且上游 URDF **没有可动的云台关节**（没有类型/限位/控制话题可依）⇒ 属于"猜接口"，只会造出一个没人能验证的机制 |

**结论一句话**：**默认 = (a)**（忠实于文件，且感知是重力对齐的）；
**任何"斜放/云台"试验 = (b)**（`turret_pitch_deg` + companion YAML 的 `gravity_aligned_frame`）。

### 3.2 真正的后果不在"斜"，而在"高"：0.8 m + MID360 −7.22° ⇒ 近场盲环

机理（可算）：MID360 的垂直视场是 **−7.22° … +55.22°**（`ros2_livox_simulation/urdf/mid360.xacro:70-71`）。
雷达离地 `h` 时，**平坦地面的最近可见距离 = h / tan(7.22°) = 7.90·h**：

| 模型 | h | 地面最近可见距离 |
|---|---|---|
| 默认模型 | 0.226 m | **1.78 m** |
| hzmirm | **0.80 m** | **6.32 m** |

而 linefit 的 `sensor_height` 语义是"传感器系下**地面**的 z = −h"（`segment.cc:30`
`double cur_ground_height = -sensor_height_;`），**它假设地面在视野里**。0.8 m 时地面在 6.3 m 以外
⇒ 近处的地面点根本不存在 ⇒ 地面线拟合不出来。

**实测（同世界、同出生点、静止 20 s 窗口）**：

| 指标 | 默认模型 | hzmirm | 证据 |
|---|---|---|---|
| `/livox/lidar/pointcloud` 点/帧（中位） | 6270（RMUL）/ 5230（RMUC） | **1100 / 2308** | `probe.json` |
| 点云最小水平距离 | 0.97 m | **4.45 m**（RMUL）；RMUC 最近 `/scan` 2.60 m | 帧转储 `frames/*.csv` |
| z 直方图最大峰 | −0.23（RMUL，*= 地面*）/ −0.23（RMUC） | −0.37 / −0.59（RMUL）、−0.59 / −0.79（RMUC） | 同上（−0.79 那层**就是**地面，但只占 2.4%） |
| `/segmentation/ground` 点/帧（中位） | **2707 / 2816** | **0 / 0** | `probe.json` |
| `/segmentation/obstacle` 点/帧（中位） | 3554 / 2438 | 1100 / **2308（= 全部点）** | 同上 |
| `/scan` 有效波束（中位，共 1462） | 1168 / 891 | **567 / 752** | 同上 |
| `/scan` inf 占比 | 20.1% / 39.1% | **61.2% / 48.6%** | 同上 |
| `/scan` 最近回波 | **0.178 m / 0.561 m** | **2.904 m / 2.603 m** | 同上 |
| `/scan` 1 m 内波束（整窗口累计） | 34987 / 1562 | **0 / 1** | `scan.band_beams_total` |
| RTF | 0.80 / 0.81 | 0.92 / 0.86 | 同上 |

**这不是参数没标定对**，三条独立证据：

1. **linefit 自己**的统计话题说得很清楚（`ros2 topic echo --once /ground_segmentation/traversability_stats`）：
   `{"points":2950,"ground_in":0,"ground_out":0,"obstacle_out":2950,"demoted_ground":0,…}`
   —— `ground_in` 是**linefit 自己**判出的地面点数 = **0**，后面的"坡度/台阶判据"一点没降级
   （`demoted_ground=0`）⇒ 不是被后级吃掉的；
2. **离线复算**（`tools/scripts/regress/linefit_offline_probe.py`，把 linefit 的逐段直线拟合按同样门限
   在 Python 里重算）：同一帧点云，默认模型 **148 条地面线 / 32.8% 点判地面**
   （与 C++ 的实测 43% 同量级 ⇒ 复算口径可信），hzmirm **1 条线 / 1.8%**；
3. **换世界也一样**：RMUC2026（29×16 m 大平地，出生点 2.7 m 净空）里 hzmirm 的
   地面点数**同样 = 0**，而默认模型 2816/帧 ⇒ 与"场地太小/太挤"无关，是**安装高度**决定的。

**顺带解释"为什么连远处那层地面也没被拟合成地面线"**：0.8 m 时可见的那层地面在 6.3–10 m，
而 linefit 是**逐 1° 扇形、逐 0.415 m 径向桶**取"每桶最低点"再拟合直线，
要求**连续点间距 < `long_threshold`(1.0 m)** 才能成线（`segment.cc:64-83`）；
远场点稀疏 ⇒ 实测 hzmirm 一帧只有 **155/360** 个扇形里有任何点（默认模型 360/360），
成线条件基本不成立。**所以调 `sensor_height` 也救不回来**（离线复算：0.226 → 1 条线、
0.80 → 1 条线，两者都 ≈0；但 0.226 会让**真地面**永远碰不到
`max_start_height`(0.5) 这道门（|−0.79−(−0.226)|=0.57 > 0.5）⇒ 标定值仍然是**必须**改的）。

**结论**：
* 这份模型**能跑**（Gazebo + TF + LIO + linefit + p2l + nav2 全部起得来，见 §5），
  但**近场感知基本没有**：`/scan` 4.4 m 内全空、地面分割 0 点、代价图在车周围是空的；
* 要用这台车，**要么把雷达装低（<0.3 m，和默认模型一个量级），要么下俯 15–25°**
  （下俯 10° 的实测见 §3.3：地面最近可见距离 6.32 → 2.58 m，点云密度 ×2.7）。

### 3.3 "斜放"假设的实测（`turret_pitch_deg` / `turret_yaw_deg`）

**先说几何**：`turret_yaw_deg` 是绕 **z** 轴转 ⇒ 雷达 z 轴仍朝上，
**不破坏**"雷达重力对齐"这条前提，linefit/p2l 的重力对齐假设不受影响（这不是"猜"，
是 TF 直接可查：实测 `base_link→livox_frame` 的 `rpy`）。`turret_pitch_deg` 才破坏它。
（`yaw≠0` **没有专门跑一轮**，登记在 §7：它的结论由"绕 z 旋转不改 z 轴"这条几何事实给出。）

**`turret_pitch_deg:=10` 实测**（RMUL2026、`lio:=fastlio`、其余同 §5）：

| 指标 | pitch 0 | **pitch 10** |
|---|---|---|
| TF `base_link→livox_frame` | `[0,0,0.80] rpy [0,0,0]` | **`[0.035,0,0.797] rpy [0,10,0]`**（= 鼻朝下 10°，与 xacro 的 `rpy="0 ${pitch} 0"` 一致）|
| 点/帧（中位） | 1100 | **2928**（×2.7）|
| 点云 z 范围 | −0.793 … −0.367 | −0.699 … **+1.433**（斜着看 ⇒ 地面在传感器系里**不再等 z**）|
| `/scan` 最近回波 | 2.904 m | **2.231 m** |
| `/scan` 有效波束 / inf | 567 / 61.2% | **706 / 51.7%** |
| `/scan` 2–4 m 带（累计） | 36848 | **174372**（近场回来了）|
| RTF | 0.915 | 0.903 |

**`gravity_aligned_frame` 的 A/B**（离线复算同一帧，`linefit_offline_probe.py`）：

| 情形 | 拟合出的地面线 | 判为地面的点 | 其中**真是最低那层地面**的比例 |
|---|---|---|---|
| pitch 10°，**不做**重力对齐（= `gravity_aligned_frame: ""`） | 34 条 | 501 / 2874 = **17.4%** | **0.0%**（拟合到的是**别的面**）|
| pitch 10° + 旋转回 `base_link`（= 本槽位 YAML 的配置） | **133 条** | 1687 / 2874 = **58.7%** | **37.0%** |
| 同上、`sensor_height` 用几何值 0.8·cos10° = 0.788 | 133 条 | 58.7% | 38.1%（差异可忽略）|

⇒ **(b) 是有效的**：不做重力对齐时算法会"拟出一条斜的假地面"（0% 命中真地面），
做了之后才恢复正常。默认（pitch=0）时它退化成恒等旋转，**不改变任何结果**。

**连带问题（我们**没有**修，明确登记）**：`pointcloud_to_laserscan` 不做 TF
（`target_frame: ""`，见 `config/laserscan_params.yaml` 的 ★2026-09-23 注释），
它的 `min_height/max_height` 是在**点云自带帧**里切的 ⇒ 雷达一旦斜放，
这个"高度带"在世界里就变成一块**斜的板**，`/scan` 会带上不该带的点。
要治只能在那个槽位把 `target_frame` 打开（代价：引入 TF 按消息戳查询，
`docs/research_p2l_scan_stall.md` 记着那条路的风险）。⇒ **斜放的正经用法是配合
`sensor_height`+`gravity_aligned_frame` 的 linefit，并且接受 p2l 的高度带偏差**。

### 3.4 另一个实测到的现象：这台车会**点头**（body pitch）

RMUC2026 跑（含 10 s 直线行驶，0.3 m/s 指令）实测**车体最大俯仰 = 6.45°**（默认模型 **0.00°**）。
原因很直接：上游只有底盘那一个 box 有碰撞、**轮子没有碰撞**（§1.3 末行）⇒ 车是"平底盒子在滑"，
压过 0.2/0.3 m 台阶时整体点头。对感知的连带影响：
* `linefit` 的 `gravity_aligned_frame: base_link` 在**车体自己**俯仰时**也不是**真正的重力对齐
  （`base_link` 跟着车体一起斜）；真要保持重力对齐应当用 LIO 的世界帧（`odom`），
  但那会引入"按消息戳查 TF"的时序风险（本仓已在 `docs/slam_toolbox_scan_drops.md` 量化过）。
  **本槽位没做这个切换**（登记在 §7）。
* LIO 侧：10 s 直线段的**位移误差 20 mm / 1.94 m = 1.0%**（默认模型 22 mm / 1.99 m = 1.1%）
  ⇒ 这点点头没有毁掉里程计。

---

## 4. 几何差异 ⇒ **需要**跟着改的配置（默认值一个都没改）

> 规则：本次是"**试一试**"，目的是对照，所以**只提供差异与"一条命令试新值"的办法**，
> 绝不改默认值（改了就没法对照，也会影响默认模型的行为）。
> 下表中"本槽位已按模型切"的三项是**槽位内部**的，`robot` 留空时它们一概不生效。

| # | 配置（键） | 默认模型的值 | hzmirm 需要的值 | 现状 | 不改会怎样（实测/推算）|
|---|---|---|---|---|---|
| 1 | `linefit_ground_segmentation_ros/config/segmentation_sim.yaml` → `sensor_height` | `0.226` | **`0.80`** | ✅ **已按槽位切换**（`segmentation_sim_hzmirm.yaml`） | 0.226 会让真地面永远进不了 `max_start_height`(0.5) 的门（\|−0.79+0.226\|=0.57）⇒ 地面线更拟合不出来 |
| 2 | 同上 → `gravity_aligned_frame` | `""` | **`"base_link"`** | ✅ **已按槽位切换** | `turret_pitch_deg≠0` 时拟出"假地面"（§3.3 的 A/B：0% 命中真地面）|
| 3 | `lio_tf_adapter/config/lio_tf_adapter.yaml` → `xyz` | `[-0.12, 0, -0.125]` | **`[0, 0, -0.75]`** | ✅ **已按槽位切换**（launch 里互斥的另一条分支，参数回读实测 `[0.0,0.0,-0.75]`）| `base_link` 在 `odom` 里被抬到 z≈0.675；水平直行时 x/y/yaw 不受影响，一有俯仰/侧倾就按 0.675 m 的假杆臂放大成 x/y 误差 |
| 4 | `rm_navigation/params/nav2_params_sim_base.yaml` → `robot_radius`（第 147 行局部 / 第 295 行全局，各一处 `0.22`） | `0.22` | **≥0.30（内切）/ 0.424（外接）**；连带 `inflation_radius`（0.5 局部 / 0.55 全局）| ❌ **未改**（保持对照） | 车体 0.6×0.6 比 0.22 的圈大得多 ⇒ 规划器会把"贴着墙"当成可行（实测：该车在 RMUL2026 出生点前方 0.40 m 就被挡住，而默认模型能走到 0.54 m）|
| 5 | `rm_nav_bringup/config/traversability_criteria.yaml` → `speed_limit_lookahead_m` | `3.0`（注释写明"覆盖雷达能看见的地面：−7.22°、离地 0.226 m ⇒ 最近 1.78 m"）| **≥6.5**（并同步放宽 `speed_limit_corridor_half_width_m: 0.28`）| ❌ **未改** | **前瞻限速对本模型等于失效**：实测日志 `[slope_speed] … cells=1 d=[0.00,0.00] maxΔ坡=0.0 max台阶=0.000`（车前一格、距离 0）⇒ 它永远"不限速" |
| 6 | `hzmi_rm_simulation/config/measurement_params_sim.yaml` → `base_link2livox_frame` | `0.12 0.0 0.175` | **不用改**（本模型的外参由上游关节链唯一决定，xacro 里 `xyz/rpy` 参数**被忽略**）| — | 无影响（但别以为改了它就能挪雷达）|
| 7 | `small_point_lio/config/mid360_sim_tuned.yaml`、`FAST_LIO/config/fastlio_mid360_sim.yaml` → `extrinsic_T: [0,0,0.05]` | `[0,0,0.05]` | **不用改**（我们把 `imu_link` 放在 `livox_frame` 下方 0.05 m，刻意与默认模型同构）| — | 无影响 |
| 8 | `fake_vel_transform/config/fake_vel_params.yaml` → `spin_speed` | `5.0`（小陀螺）| **建议 `0.0`**（本模型云台是 fixed，"小陀螺"没有对应的物理量；`0.0` 时 `base_link_fake ≡ base_link`）| ❌ 未改（跑的时候用 `spin_speed:=0.0`）| `spin_speed≠0` 时 nav 看到的 `base_link_fake` 会**凭空转**，而雷达并没有转 ⇒ 规划与感知的取向不一致 |
| 9 | `pointcloud_to_laserscan/config/laserscan_params.yaml` → `min_height/max_height` | `-1.0 / 1.0`（传感器系；离地换算 [-0.774, +1.226] m）| **数值不用改**，但**离地含义变了**：`[-1.8, +1.8]` m | ❌ 未改 | 世界无悬挑（`docs/worlds.md` §6）⇒ 不会因此压进假障碍；但 §3.3 的"斜放 ⇒ 高度带被拧斜"这条对 p2l 依然成立 |

**"一条命令试新值"**（`.tmp` 里改副本、不动仓库，示例 = 第 4 项 nav2 的 `robot_radius`）：

```bash
# 只影响这一次跑：把默认 nav2 参数复制一份出来改，再把 params_file 指过去
cp src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml /tmp/nav2_hzmirm.yaml
sed -i 's/robot_radius: 0.22/robot_radius: 0.32/' /tmp/nav2_hzmirm.yaml     # 两处一起换
# ⚠️ 本 launch 的 nav2 参数是"base + planner + controller"三段叠加、且用包内路径拼的，
#    临时改一份的推荐做法是"改完直接 source 后按 §6.1 的命令跑"，或（更干净）
#    只在专用分支上改仓库文件；本文件不提供"运行时覆盖 nav2 参数"的新开关（不为一次试验加机制）。
```
第 5 项（`speed_limit_lookahead_m`）同理，但它是**单一真源**文件：改它 = 同时改默认模型与离线出图口径，
所以更推荐**先在专用分支上改、跑完 A/B 再决定**（本文件只登记差异，不替你改）。

---

## 5. 实测对照：默认模型 vs `robot:=hzmirm`

跑法（一次 bash 调用跑完一次隔离的无头仿真 + 探针，见 §8 工具①）：

```bash
# 默认模型（对照）
tools/scripts/regress/run_robot_model_probe.sh default2 --duration 20 --drive-seconds 10 \
  --drive-speed 0.2 --dump-cloud .tmp_robotslot/default2/frames --dump-scan .tmp_robotslot/default2/scans \
  -- world:=RMUL2026 mode:=mapping lio:=small_point_lio map_autocontinue:=False
# hzmirm（本槽位）
tools/scripts/regress/run_robot_model_probe.sh hzmirm2 --duration 20 --drive-seconds 10 \
  --drive-speed 0.2 --dump-cloud .tmp_robotslot/hzmirm2/frames --dump-scan .tmp_robotslot/hzmirm2/scans \
  -- world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=hzmirm map_autocontinue:=False
# 6 条结果并排打印
python3 tools/scripts/regress/compare_robot_model_probes.py \
  .tmp_robotslot/default2/probe.json .tmp_robotslot/hzmirm2/probe.json \
  .tmp_robotslot/rmuc_default/probe.json .tmp_robotslot/rmuc_hzmirm/probe.json
```

### 5.1 逐项对照（`mode:=mapping`，静止 20 s 窗口 + 10 s 直线行驶）

| 指标 | RMUL2026 默认 | RMUL2026 hzmirm | RMUC2026 默认 | RMUC2026 hzmirm |
|---|---|---|---|---|
| Gazebo spawn | ✅（一次"实体已入队但服务超时"的**瞬时竞态**，两台车都会出现，与世界加载慢有关，非模型问题）| ✅ 成功 | ✅ 成功 | ✅ 成功 |
| TF 树 | `base_link→{wheel_1..4, imu_link, livox_frame}` | **加** `turret_base/turret_head/lidar_link/barrel`：`turret_base(0,0,0.3) → turret_head(0,0,0.6) → lidar_link(0,0,0.8) → livox_frame(0,0,0.8) / imu_link(0,0,0.75)` 全部存在 ✅ | ✅ | ✅ |
| `/livox/lidar/pointcloud` | 10.00 Hz | 10.00 Hz | 10.00 Hz | 10.00 Hz |
| `/livox/imu` | 100.0 Hz | 99.99 Hz | 99.99 Hz | 100.0 Hz |
| `/scan` | 10.00 Hz | 10.00 Hz | 10.00 Hz | 10.00 Hz |
| 点/帧（中位） | 6270 | 1100 | 5230 | 2308 |
| `/segmentation/ground` 点/帧 | 2707 | **0** | 2816 | **0** |
| `/scan` 有效波束 / inf | 1168 / 20.1% | 567 / 61.2% | 891 / 39.1% | 752 / 48.6% |
| `/scan` 最近回波 | 0.178 m | 2.904 m | 0.561 m | 2.603 m |
| 真值位移（10 s 直线） | 0.545 m（出生点前方 0.5 m 被挡）| 0.400 m（0.6 m 车体更早被挡）| 1.997 m | 1.951 m |
| LIO 位移-真值位移（x, y） | (−0.021, +0.005) m | (−0.019, +0.007) m | (−0.021, +0.005) m | (−0.019, +0.007) m |
| **LIO 漂移（位移口径）** | 22 mm / 0.55 m 行程（被挡，样本短）| 20 mm / 0.40 m | **22 mm / 2.00 m = 1.1%** | **20 mm / 1.95 m = 1.0%** |
| LIO 路径长/真值路径长 | 1.54 | 2.56 | 1.10 | 1.15 |
| 车体最大俯仰/侧倾（真值） | 0.02° | 0.02° | **0.00°** | **6.45°**（§3.4）|
| RTF | 0.80 | 0.92 | 0.81 | 0.86 |
| 唯一发布者契约 | ✅ | ✅ | ✅ | ✅ |
| 墙钟总时长 | 34.1 s | 34.1 s | 34.0 s | 34.0 s |

契约检查（每次跑都做，`ros2 topic info -v` 逐条回读）：`/cmd_vel_chassis`、`/segmentation/obstacle`、
`/segmentation/ground`、`/map`、`/odom`、`/livox/imu` **publisher 全部 = 1**。

### 5.2 短导航冒烟（`mode:=slam_nav`，60 s，`spin_speed:=0.0`）

`RMUC2026 + lio:=small_point_lio + robot:=hzmirm`，实测：

* **nav2 全套起来了**：`controller_server`（RPP）/`smoother_server`/`planner_server`（NavfnPlanner）/
  `behavior_server`/`bt_navigator`/`waypoint_follower` 全部 `Configuring → Activating`（
  `lifecycle_manager_navigation` 逐条日志可查），两张 costmap `Subscribed to Topics: scan`；
* 20 个进程、**零 ERROR**（只有 headless 固有的 `gzclient died` 与 KDL 的"根 link 带惯性"警告
  —— 后者**默认模型也有**，是既有现象）；
* RTF **0.86**、`/scan` 10 Hz、有效波束中位 **763**、inf 47.8%；
* LIO 在 60 s 窗口内位移误差 **8 mm / 1.26 m = 0.6%**；
* ⚠️ **没有发 Nav2 目标点**（只直接发了 `/cmd_vel_chassis`）⇒ 这只能算"导航栈起得来、costmap 活着"，
  **不是**"导航能走通"。真正的 `mode:=nav` + 目标点验收**没做**（§7）。

### 5.3 结论（一句话）

**跑得通，但"看得见"这一项差得很远**：LIO/RTF/频率/TF/契约与默认模型同量级（漂移 1.0% vs 1.1%），
而 `/scan` 近场（<2.6 m）**全空**、地面分割 **0 点**、车周围代价图基本是空的。

---

## 6. 怎么试 / 怎么退

### 6.1 试（复制即可）

```bash
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash && source install/setup.bash

# ① 先构建（install/ 是**逐文件符号链接** ⇒ 新文件必须 build 才能被 xacro/节点看到）
colcon build --symlink-install --packages-select rm_nav_bringup linefit_ground_segmentation_ros

# ② 无头建图（推荐：与默认模型逐项对照）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=hzmirm \
  map_autocontinue:=False nav_rviz:=False lio_rviz:=False
# 换 RMUC2026（大平地，出生点 2.7 m 净空）、换 LIO、边建边导：把上面三个参数换掉即可
#   world:=RMUC2026 / lio:=fastlio / mode:=slam_nav（+ 建议 spin_speed:=0.0）

# ③ 试"雷达斜放/云台"假设（0 = 与上游 URDF 数值等价）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=mapping lio:=fastlio robot:=hzmirm \
  turret_pitch_deg:=10 map_autocontinue:=False nav_rviz:=False     # 鼻朝下 10°
#   turret_yaw_deg:=15 → 绕 z 转，几何上不影响重力对齐（§3.3）

# ④ 一条命令量齐（隔离的无头跑 + 探针 + 契约检查；结果落 .tmp_robotslot/<tag>/）
tools/scripts/regress/run_robot_model_probe.sh myrun --duration 20 --drive-seconds 10 \
  --dump-cloud .tmp_robotslot/myrun/frames --dump-scan .tmp_robotslot/myrun/scans \
  -- world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=hzmirm map_autocontinue:=False

# ⑤ 单独起 Gazebo+模型（不起感知/LIO；本文件不含槽位标定切换）
ros2 launch hzmi_rm_simulation rm_simulation.launch.py robot:=hzmirm world:=RMUC2026
```

### 6.2 退回去

| 想退掉什么 | 怎么做 |
|---|---|
| 只退回默认模型 | **什么都不用改**：去掉 `robot:=hzmirm`（或写 `robot:=''`）即可 —— 默认路径从未改动过 |
| 退回"雷达不斜" | 去掉 `turret_pitch_deg/turret_yaw_deg`（默认就是 0）|
| 退掉整套槽位（连新增文件一起）| `git revert <本主题的 commit>`（新增文件清单见 §8；它们**互不耦合**：删掉 `sentry_robot_hzmirm_sim.xacro`/`upstream/`/那两个 YAML/两个 regress 脚本后，`robot:=hzmirm` 会报一条可操作的错，默认模型不受影响）|
| 只想退回 linefit 的旧标定 | 把 `bringup_sim.launch.py` 里 `segmentation_params` 换回 `os.path.join(get_package_share_directory('linefit_ground_segmentation_ros'),'config','segmentation_sim.yaml')`（一行）|
| 只想退回 lio_tf_adapter 的旧杆臂 | 删掉 `lio_tf_adapter_hzmirm_node` 那条 Node（或把它的 `xyz` 改回 `[-0.12,0,-0.125]`）|

---

## 7. 未验证清单（诚实清单）

1. **`mode:=nav` + 真实 Nav2 目标点**没跑过：只做了 `mode:=mapping`（20 s 静止 + 10 s 直线）
   与 `mode:=slam_nav`（60 s，**未发目标点**）⇒ "规划/控制能不能把车开到目标"**未验**；
2. **`turret_yaw_deg≠0` 没有专门跑一轮**：结论由"绕 z 旋转不改变 z 轴"这条几何事实给出（§3.3），
   没有 `/scan` 实测；
3. **斜放（pitch≠0）只跑了 `lio:=fastlio` 一轮**（RMUL2026），没有在 RMUC2026 / small_point_lio 下复验；
4. **斜放时 p2l 高度带被拧斜的量化**没做（只证明机理与"我们没修"）；
5. **代价图/规划层面对"远处地面点进入 `/scan`"的反应**没量（`ground=0` ⇒ p2l 的输入是全部点，
   RMUC2026 里 >7 m 的波束 61659 vs 默认 31614，其中约 ≤50 束/帧是 6.3–10 m 的地面；
   究竟会不会让代价图/规划变差，**未验**）；
6. **车体点头（6.45°）对感知的影响**没量；`gravity_aligned_frame` 换成 `odom`（真正的重力对齐帧）
   **没试**（会引入按消息戳查 TF 的时序风险，见 §3.4）；
7. **`ground:=patchwork` + `robot:=hzmirm`** 未验（patchwork 自己的参数是按默认模型几何标定的）；
8. **其它 LIO 槽（`pointlio` / `cartographer`）** 与 `localization:=*` 各槽未验；
9. **质量/惯量是占位值**（0.01 的对角惯量），不是实测/辨识值 ⇒ 接触与点头的定量结论只能当**定性**；
10. **`rm_simulation.launch.py robot:=hzmirm`（单独起 Gazebo）没实跑**（只做了槽位解析的单元级验证）；
11. **`robot:=hzmirm` 与用户其它资产（RMUC2026 地图/PCD 先验、`localization:=gicp`）的配合**未验；
12. **长时间跑**（>60 s）与转弯工况的 LIO 漂移未验（本文件所有漂移数字都来自 10 s 直线 / 60 s 含 10 s 直线）。

---

## 8. 复现工具与证据文件

| 工具/文件 | 作用 |
|---|---|
| `src/rm_nav_bringup/urdf/sentry_robot_hzmirm_sim.xacro` | **本槽位的模型**：上游运动学逐字 + 我们补的 inertial/IMU/雷达/底盘（每个元素都带 `【上游原样】/【我们加的】` 标记与理由）|
| `src/rm_nav_bringup/urdf/upstream/hzmirm_sentry_robot.urdf` | 上游**逐字节副本**（只读、不参与运行），供 sha256/字段核对 |
| `tools/scripts/regress/check_hzmirm_urdf_provenance.py` | **provenance 断言**（§2.2）：sha256 + "上游 0 条 inertial" + 逐 link/joint 数值相等 + 列出我们加了什么。`python3 …` 即可，退出码 0 = 通过 |
| `tools/scripts/regress/run_robot_model_probe.sh` | **一次隔离的无头跑**（独立 HOME/DOMAIN_ID/GAZEBO_MASTER_URI、无 DISPLAY、跑完补零速、清 gzserver）+ 参数回读 + 契约检查 + 探针 |
| `tools/scripts/regress/robot_model_probe.py` | **只读探针**：TF/点云/IMU/分割/`/scan`/LIO vs 真值/RTF；可 `--dump-cloud/--dump-scan` 落帧供离线分析 |
| `tools/scripts/regress/compare_robot_model_probes.py` | 把若干份 `probe.json` 并排打印（§5.1 的表）|
| `tools/scripts/regress/linefit_offline_probe.py` | **离线复算 linefit** 的逐段地面线拟合（§3.2/§3.3 的 A/B）|
| `src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/config/segmentation_sim_hzmirm.yaml` | 本槽位的 linefit 参数（只改 `sensor_height` 与 `gravity_aligned_frame` 两键，逐条写了理由）|
| 原始证据（**未入库**，`.tmp_robotslot/<tag>/`）| `launch.log`（launch 全量日志）、`probe.json`（机器可读实测）、`probe.log`、`frames/*.csv`（点云帧）、`scans/*.csv`（`/scan` 帧）。tag：`default2`/`hzmirm2`（RMUL2026）、`rmuc_default`/`rmuc_hzmirm`（RMUC2026）、`tilt_pitch10`（斜放）、`hzmirm_nav`（60 s slam_nav）|

**每次都要先 `colcon build --symlink-install --packages-select rm_nav_bringup linefit_ground_segmentation_ros`**
（`install/` 是逐文件符号链接；新 xacro / 新 YAML 不 build 就看不到）。

---

## 9. Phase 1：用户 `robot11` 的 mesh 打包与量测（2026-10-07）

> 状态：**新增**。这一节只做"资产与量测"，**不改任何默认值、不接 URDF**（接入是 §10）。
> 素材：用户 2026-10-07 给的 12 个 STL（`base_link.STL` + `l2..l12.STL`，SolidWorks 导出）。
> 复现工具（全部只读、自带解析器、不装新包）：
> `tools/scripts/regress/robot11_mesh_inventory.py`（清单）、
> `robot11_make_collision_assets.py`（碰撞替代件）、
> `robot11_geometry.py`（正运动学 + 足印 + 地面）、
> `robot11_gz_collision_bench.py`（Gazebo 代价实测）。
> 机器可读产物全部落在 **包内** `src/rm_simulation/robot11_description/inventory/`。

### 9.1 包布局（为什么目录名与包名不一样）

| 项 | 值 |
|---|---|
| 包名 | **`robot11`**（`package.xml` 的 `<name>`）——**必须**叫这个：用户 URDF 里写死了 `package://robot11/meshes/<name>.STL`，而 `package://` 的解析键就是**包名** |
| 目录 | `src/rm_simulation/robot11_description/` —— 放在 `src/rm_simulation/` 下与其它**仿真资产包**（`hzmi_rm_simulation`、`livox_laser_simulation_RO2`）并列；目录名带 `_description` 只表达"这是描述包"，**与 `package://` 解析无关** |
| 内容 | `meshes/`（12 个原始 STL）+ `meshes/generated/`（12 个抽稀碰撞件）+ `inventory/`（4 份量测产物）+ `package.xml` / `CMakeLists.txt` |
| 安装 | `CMakeLists.txt` 装 `meshes/`，并**存在才装** `urdf/ config/ launch/`（Phase 2 会加 `urdf/`）|

```bash
colcon build --symlink-install --packages-select robot11
python3 -c "import ament_index_python as a; print(a.get_package_share_directory('robot11'))"
# 期望 .../install/robot11/share/robot11
```

**大文件怎么办（git 策略）**：本仓**本来就跟踪 mesh 与大二进制**（`mid360.stl` 4.2 MB、
`RMUL_2024.stl` 22 MB、`RMUL.pcd` 51 MB、`mapping_steves_apartment.gif` 38 MB），
所以按同一口径**把这 12 个 STL 全部入库**，包括 99.1 MiB 的 `base_link.STL`
（GitHub 单文件硬上限是 100 MiB ⇒ 它在限内，但只差 0.9 MiB；**如果远端拒收，退路是**
`git rm --cached` 掉它、只留抽稀件，并在本节记一条"本地拷贝步骤"）。
原始文件**一个字节都没改**：拷贝后 12 个 sha256 与附件逐一相同（附件目录名本身就是 sha256）。

### 9.2 mesh 清单（机器可读：`inventory/mesh_inventory.json`）

**12 个全部是 binary STL**，且 `84 + 50×三角形数 == 文件字节数` 逐一成立；
**单位：原始值就已经是米**（不是 mm——判据是"原始包围盒最大跨度 > 20 才当 mm"，实测最大 0.6）。
水密性用两套独立判据交叉校验（自带"每条边被几个面用到"统计 + `trimesh 4.11.0`），
两套结论一致：**没有一个水密**（`base_link` 尤其严重：53378 条非流形边、19432 条边界边、
**2733 个互不相连的实体**），所以下面所有"体积"只能当参考、**不能当质量/惯量来源**。

| mesh | 字节 | 三角形 | 尺寸 (m, x×y×z) | 包围盒 min … max (m) | 水密 | 非流形/边界边 | 连通体 |
|---|---|---|---|---|---|---|---|
| `base_link.STL` | 103,911,384 | 2078226 | 0.6000×0.6000×0.2609 | -0.3000,-0.3000,-0.0479 … 0.3000,0.3000,0.2130 | ❌ | 53378 / 19432 | 2733 |
| `l2.STL` | 4,477,884 | 89556 | 0.1164×0.1036×0.1246 | -0.1342,-0.0545,-0.0611 … -0.0179,0.0492,0.0635 | ❌ | 351 / 0 | 45 |
| `l3.STL` | 4,440,084 | 88800 | 0.1164×0.1098×0.1036 | -0.1342,-0.0635,-0.0545 … -0.0179,0.0463,0.0492 | ❌ | 351 / 0 | 44 |
| `l4.STL` | 4,561,784 | 91234 | 0.1175×0.1036×0.1098 | -0.1353,-0.0492,-0.0635 … -0.0179,0.0545,0.0463 | ❌ | 351 / 0 | 46 |
| `l5.STL` | 4,512,584 | 90250 | 0.1175×0.1098×0.1036 | -0.1353,-0.0463,-0.0492 … -0.0179,0.0635,0.0545 | ❌ | 351 / 0 | 45 |
| `l6.STL` | 1,221,884 | 24436 | 0.0452×0.1160×0.1160 | -0.0050,-0.0580,-0.0580 … 0.0402,0.0580,0.0580 | ❌ | 208 / 0 | 3 |
| `l7.STL` | 1,266,284 | 25324 | 0.0453×0.1160×0.1160 | -0.0051,-0.0580,-0.0580 … 0.0402,0.0580,0.0580 | ❌ | 208 / 0 | 4 |
| `l8.STL` | 1,266,284 | 25324 | 0.0453×0.1160×0.1160 | -0.0051,-0.0580,-0.0580 … 0.0402,0.0580,0.0580 | ❌ | 208 / 0 | 4 |
| `l9.STL` | 1,266,284 | 25324 | 0.0453×0.1160×0.1160 | -0.0051,-0.0580,-0.0580 … 0.0402,0.0580,0.0580 | ❌ | 208 / 0 | 4 |
| `l10.STL` | 1,882,484 | 37648 | 0.1470×0.1379×0.0706 | -0.1370,-0.0496,-0.0324 … 0.0100,0.0883,0.0382 | ❌ | 422 / 0 | 13 |
| `l11.STL` | 8,822,984 | 176458 | 0.1330×0.1724×0.3352 | -0.0260,-0.0437,-0.1291 … 0.1070,0.1287,0.2061 | ❌ | 412 / 0 | 54 |
| `l12.STL` | 3,437,084 | 68740 | 0.0717×0.0649×0.0601 | -0.0399,-0.0303,0.0000 … 0.0319,0.0345,0.0601 | ❌ | 0 / 500 | 16 |

**逐 mesh 的推断（与 URDF 关节值对得上）**：

| mesh | 它是什么 | 由 mesh 本身读出来的量 |
|---|---|---|
| `base_link` | 底盘（600×600 底板上再叠一段 553×550 的中段 + 285×337 的上塔，塔顶 z=0.2130）| 顶面 z=0.2130 **正好等于** `j10`（云台）origin 的 z ⇒ 云台坐在底盘顶面上（互校通过）|
| `l2..l5` | 四个**转向**模块（舵机/转向柱）| 每个都是"绕自身竖向轴转"的一坨；`l2` 的 mesh 在其原点 **−x 方向偏 76 mm** ⇒ 原点=转向轴、模块本体挂在轴的一侧（舵机偏置）|
| `l6..l9` | 四个**轮**子 | `l6` 的 y/z 双向对称 `±0.0580` ⇒ **半径 0.0580 m**、宽 0.0452 m、轴向 = mesh 的 x ⇒ 轮心就在 mesh 原点的 yz 轴上 |
| `l10` | 云台 **yaw** 段（`j10`，绕 z）| 147×138×71 mm |
| `l11` | 云台 **pitch** 段 + 发射机构（`j11`，绕 z→ 实际成为 pitch）| 133×172×335 mm，是整车最高的部件（顶点 z=0.2061 + j11 平移）|
| `l12` | **Livox MID-360** | 71.7×64.9×**60.1** mm，底面 z=0（= 安装面）⇒ 与 MID-360 规格高度 60.1 mm 完全一致（互校通过）|

### 9.3 `base_link.STL` 的碰撞策略（含 Gazebo 实测代价）

**结论：2,078,226 个三角面的 `base_link.STL` 不能直接当 `<collision>`。**
不是"慢一点"，而是**一有接触就掉到 0.2 倍实时**，并且**常驻内存 +587 MB**（下面有实测表）。

**三级方案（已全部生成、入库、可复现）**：

| 级别 | 做什么 | 产物 | 用在哪 |
|---|---|---|---|
| **(a) 视觉保留原网格** | 一个字节都不动 | `meshes/base_link.STL` | `<visual>`（本槽位**已采用**：视觉零损失）|
| **(b) 基本体碰撞（采用）** | 由 mesh 自身 z 分带自动求"最优 N 段包络 box"（5 mm 分带 → 动态规划取总体积最小的 N 段切分，N=4）| `inventory/collision_assets.json` 的 `boxes.base_link`（4 个 box）| `<collision>`（本槽位**已采用**）|
| **(c) 抽稀碰撞网格** | VTK `vtkQuadricDecimation`（`pyvista 0.46.4`；**meshlabserver 2020.09 在本机无 GL，`GLEW initialization failed` 跑不了**；`fast_simplification`/`open3d`/`pymeshlab` 未装、也没装新包）| `meshes/generated/*_collision.stl`（12 个，`base_link` 2,078,226 → **3,000** 面 / 147 KB）| 备选（实测与 (b) 同价，见下）|

**4 个 box 的客观保真度**（`--band 0.005 --merge-tol 0.012` 的 DP 最优 4 段）：

| 指标 | 值 | 含义 |
|---|---|---|
| 覆盖率 | **1.000**（1605/1605 个体素）| 真网格表面**没有一个点**在 box 外 ⇒ 不会"漏撞" |
| box 总体积 | 0.065121 m³ | 网格**凸包**体积 0.0595 m³ ⇒ 只比凸包胖 **9.4%** |
| 幻影体积占比 | 16.36%（pitch 30 mm）| box 内部格点里离真表面 >30 mm 的比例 ⇒ "凭空多出来的碰撞体积" |
| 最大内间隙 | 69.3 mm | 最坏情况的空腔 |

4 个 box（body 系，中心 / 尺寸）：`[0,0,-0.0054] 0.6000×0.6000×0.0850`、
`[0,0,0.0721] 0.5533×0.5495×0.0700`、`[0,-0.0063,0.1246] 0.5324×0.3345×0.0350`、
`[0,-0.0148,0.1775] 0.2850×0.3465×0.0709`。
（`l6..l9` 轮子用 `<cylinder r=0.058 l=0.0452>`，由轮 mesh 包围盒直接得到，比抽稀网格更便宜也更准。）

**Gazebo Classic 实测代价**（极简世界：ground_plane + sun，ODE quick/50，`max_step_size 1 ms`、
`real_time_update_rate 1000`；单模型 spawn；无头隔离 = `HOME=/tmp/gzhome-<tag>`、`ROS_DOMAIN_ID=77`、
专用 `GAZEBO_MASTER_URI`、`unset DISPLAY`；**只按 PID 杀，不用 `pkill -f`**；
产物 `inventory/gz_collision_bench.json`）：

| 变体 | 视觉 | 碰撞 | spawn 往返 (s) | 稳态 RTF | 稳态 RSS (MB) | **峰值 RSS (MB)** | 窗口内推进的仿真时间 (s) |
|---|---|---|---|---|---|---|---|
| **A 原网格** | 原网格 | **原网格** | 0.57 | 1.002 | 194 | **760** | 11.26 |
| **B 基本体（采用）** | 原网格 | **4 box** | 0.56 | 1.000 | 174 | **174** | 11.34 |
| **C 抽稀网格** | 原网格 | 3000 面 | 0.58 | 1.003 | 173 | **173** | 11.43 |
| **D 连视觉也抽稀** | 抽稀 | 4 box | 0.56 | 1.005 | 173 | **173** | 11.34 |
| **E 基线（无底盘）** | 无 | 无 | 0.58 | 1.000 | 172 | **172** | 11.33 |
| **A2 原网格 + 接触** | 原网格 | **原网格** | 0.56 | **0.200**（最低 0.04） | 187 | **759** | **0.49** |
| **B2 基本体 + 接触** | 原网格 | 4 box | 0.58 | 1.002 | 173 | **173** | 11.36 |
| **C2 抽稀 + 接触** | 原网格 | 3000 面 | 0.57 | 1.002 | 173 | **173** | 11.34 |

读法与结论（**四条，都是实测**）：

1. **稳态悬空时"原网格碰撞"并不慢**（A 的 RTF = 1.002）——因为底盘离地 54.5 mm、
   压根没有接触对被求值。**别用"稳态 RTF 正常"当作"原网格能用"的证据**。
2. **一有接触就崩**：A2（底盘贴地）RTF **0.203**、最低 **0.04** ⇒ 比实时慢 25–50 倍；
   同一工况下 4 个 box（B2）与 3000 面抽稀网格（C2）都是 **1.002**。
3. **常驻内存恒 +587 MB**：A/A2 峰值 **760 MB** vs 基线 172 MB —— 这是把 208 万面灌进 ODE
   trimesh 的代价，**跟有没有接触无关**。
4. 所以选 **(a)+(b)**：视觉留原网格（零损失），碰撞用 4 个 box。
   (c) 抽稀网格与 (b) **同价**，留作备选（想更贴形时换 `meshes/generated/base_link_collision.stl` 即可，
   保真度：抽稀件到原网格的单向误差 max 26 mm / p99 18 mm）。

> ⚠️ 本机两个坑（写下来省下一次踩）：① `gzserver --iters N` 在本机会**一直不退出**
> （世界在跑、进程不结束）⇒ 计时不能用它，改用固定墙钟窗口 + `gz stats`；
> ② 不设 `GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models` 时，`model://ground_plane`
> 会去连 `models.gazebosim.org` 并**永久挂住**（本机 DNS 把它解析到 198.18.0.83）。

### 9.4 整车几何（会喂给 `robot_radius` / `inflation_radius` / `sensor_height` / p2l 带偏移）

推导方式：**用户 URDF 的关节表** + **每个 mesh 的顶点**做零位正运动学（所有关节角 = 0），
再取变换后顶点的精确包围盒；足印取 XY 凸包（3 mm 体素抽稀后求，误差 ≤3 mm）。
机器可读：`inventory/geometry.json` / `geometry.md`。

| 量 | 值 | 怎么来的 |
|---|---|---|
| **地面平面** | body 系 **z = −0.102499 m** | 四个轮 link 的 mesh 顶点在 body 系下的**最低点**；轮 mesh 原点在轮轴上（r=0.0580）⇒ 轮底 = 地面 |
| **雷达离地** | **0.2595 m** | `body_to_livox` 的 z（0.157028）− 地面 z（−0.102499）|
| 轮心离地 | 0.1598 m | 轮心 body z = −0.0445（**不是** `j2` 的 origin z=0.05735，那是**转向关节**高度）|
| 底盘底离地间隙 | 0.0546 m | 底盘 mesh 最低点（−0.047944）− 地面 |
| 整车高（含云台/发射机构）| 0.5642 m | 所有 mesh 顶点 max z（0.461724）− 地面 |
| 底盘组高 | 0.3155 m | body + l2..l9 |
| **足印（底盘组）内切半径** | **0.300 m** | XY 凸包上离原点最近的边到原点距离 ⇒ nav2 `robot_radius` 的**下界**（能走多窄）|
| **足印（底盘组）外接半径** | **0.3565 m** | 凸包顶点里 \|xy\| 最大 ⇒ `robot_radius` 的**上界**（保证不撞）|
| 足印凸包面积 | 0.3390 m² | 600×600 底板四个角被切掉（纯方形是 0.36 m²）|
| 极坐标最大半径 | 0.3036（轴向）→ 0.3565（对角）| 每 15° 一档，见 `geometry.md` §3.1 |
| **轴距 / 轮距** | **0.36427 m / 0.36427 m** | `j2..j5` 的 origin（±0.1821345596729）|
| 质量合计 | **9.5521 kg** | URDF 12 条 `<inertial>` 的 `<mass>` 求和（body 6.9839 + 4×转向 0.9509 + 4×轮 0.6070 + yaw 0.1411 + pitch/发射 0.7290 + 雷达 0.1400）|

**mesh 实测 ↔ 关节值的互校（3 条，全部通过；不一致会在这里报出来）**：

| 检查 | mesh 侧 | 关节侧 | 差 |
|---|---|---|---|
| 底盘顶面 z ↔ `j10`（云台）origin z | 0.213000 | 0.213000 | **0.000000** |
| `l12` 底面 z ↔ `livox_frame` 原点（= 安装面）| 0.000000 | 0.000000 | **0.000000** |
| `l12` 高度 ↔ MID-360 规格高度 | 0.060097 | 0.0601 | 0.000003 |

> ⚠️ **一处"看起来像不一致、其实不是"**：`j2..j5` 的 origin z = 0.05735 与"轮心 0.1598 m 离地"
> 差 0.102 m。这不是错——`j2` 是**转向关节**（转向柱顶端），轮子挂在它下面 0.102 m 处。
> **踩坑提示**：Phase 2 里若要摆轮子，地平面必须按 **−0.102499** 算，不能按 `j2` 的 z 算
> （第一版 benchmark 就是这么错的，导致底盘"趴"在地上、量出来的 RTF 是蹭地代价）。

### 9.5 雷达 30° 安装：为什么"roll 还是 pitch"不能靠仰角测出来（Phase 2 的判据）

`body_to_livox` 的 30° 旋转轴有两份互相矛盾的来源（URDF 原文写 `rpy="0 -0.5236 0"` = pitch；
SolidWorks CSV 写 `Joint Origin Roll = −0.523598775598293` = roll）。
**先给出一个几何事实，能省掉一半的测量**：

| 候选 | 世界仰角范围 | **最近地面环半径** | 最朝下的方位（body 系）|
|---|---|---|---|
| roll（`rpy = −0.5236 0 0`）| −37.0° … +82.0° | **0.3444 m** | **+90°（即 +y）** |
| pitch（`rpy = 0 −0.5236 0`）| −37.0° … +82.0° | **0.3444 m** | **180°（即 −x）** |

⇒ MID-360 方位是 **360°**，所以"绕 x 转 30°"与"绕 y 转 30°"给出的**射线方向集合完全相同**
（只差一个绕 z 的旋转）⇒ **仰角谱、地面环半径、点数-距离分布都区分不了它们**。
**唯一能区分的判据是"最朝下的那一束指向 body 的哪个方位"**（roll → **±y**，pitch → **±x**），
以及由此导致的"方位角-仰角调制相位"。Phase 2 的点云实测就按这条判据做（§10.5）。

顺带一个重要结论：30° 下俯把雷达的**盲区**从"整片近场"缩到
**半径 0.3444 m 的圆**（= 0.2595 / tan 37°）——这正是 hzmirm（0.80 m 高、不斜放、盲区 6.3 m，
§3.2）缺的那件事。**这台车的雷达能看见近处地面。**

---

## 10. Phase 2：`robot:=robot11` 槽位接入 + 雷达 30° 的实测判定（2026-10-07）

> 状态：**新增（opt-in）**。`robot` 留空 = 默认模型，**默认路径逐字节未改**（见 §10.1 的证据）。
> 一句话：**接得进去、跑得起来（spawn ✅ / TF ✅ / 10 Hz ✅ / RTF 0.75），但"看得见"这条不成立** ——
> 这台车的雷达装在底盘**顶板的凹槽里**（雷达原点 z=0.157 < 底盘顶面 0.213），
> 实测 **75.5% 的点是"打到自己"的近距回波（r<0.12 m）**、`/segmentation/ground` 只剩 **529 点/帧**
> （默认模型 2626）、**LIO 连 `/odom` 都没发出来**。**不建议**拿它当默认模型（理由与数字见 §10.4–§10.7）。

### 10.1 槽位怎么用 / 加了哪些文件

```bash
# ① 构建（install/ 是逐文件符号链接；新 xacro 不 build 看不到）
colcon build --symlink-install --packages-select robot11 rm_nav_bringup hzmi_rm_simulation

# ② 试（无头建图；与默认模型逐项对照）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
  map_autocontinue:=False nav_rviz:=False lio_rviz:=False
# ③ 换"30° 绕哪个轴"（默认 roll = SolidWorks CSV；pitch = URDF 原文）
#   ... robot:=robot11 livox_tilt_axis:=pitch ...
# ④ 只起 Gazebo + 模型
ros2 launch hzmi_rm_simulation rm_simulation.launch.py robot:=robot11 world:=RMUC2026
# ⑤ 一条命令量齐（隔离无头跑 + 探针 + 契约检查）→ .tmp_robotslot/<tag>/
tools/scripts/regress/run_robot_model_probe.sh r11 --duration 20 --drive-seconds 10 \
  --dump-cloud .tmp_robotslot/r11/frames --dump-scan .tmp_robotslot/r11/scans \
  -- world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 livox_tilt_axis:=roll \
     map_autocontinue:=False
```

| 文件 | 是什么 |
|---|---|
| `src/rm_simulation/robot11_description/`（包名 **`robot11`**）| 12 个原始 STL + 12 个抽稀件 + 4 份量测产物（Phase 1，§9）|
| `src/rm_nav_bringup/urdf/upstream/robot11.urdf` | 上游 URDF **逐字节副本**（只读、不参与运行；sha256 `e3e322ac…a64fc`）|
| `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` | **生成物**（别手改）：上游运动学/inertial/visual 逐字 + 我们替换的 collision + 补的 IMU/雷达/底盘 |
| `tools/scripts/regress/robot11_make_sim_xacro.py` | 上者的**生成器**（provenance 是构造性的：可逐字节重算）|
| `tools/scripts/regress/robot11_geometry.py` | 零位 FK + 地面/足印/雷达倾角候选（§9.4/§9.5）|
| `tools/scripts/regress/robot11_gz_collision_bench.py` | Gazebo 碰撞代价实测（§9.3）|
| `bringup_sim.launch.py` | `robot` 的 `choices` 加 `'robot11'`；新增 `livox_tilt_axis`；新增互斥的 `lio_tf_adapter_robot11_node` |
| `hzmi_rm_simulation/launch/rm_simulation.launch.py` | `robot` 的 `choices` 加 `'robot11'`（只起 Gazebo）|

**"默认路径没变"的证据**（可复现，`_RobotXacroCommand` 的单元级比对，见 §10.7 工具）：
`robot` 留空时拼出的命令**仍是** `xacro …/sentry_robot_sim.xacro xyz:=… rpy:=…`（逐字符相同）；
`robot:=hzmirm` 也一字未变（仍追加两个云台角）。linefit / p2l / nav2 的**任何默认文件都没动**。

### 10.2 这个槽位里，哪些是用户的、哪些是我们补的/换的

| 元素 | 来源 | 说明 |
|---|---|---|
| 12 个 link / 12 个 joint（名字/类型/父子/xyz/rpy/axis）| **上游原样** | 由生成器逐字抄；`j2..j9` 保持 `continuous`（4 个转向 + 4 个轮），`j10/j11` 是云台（yaw→pitch，带发射机构）|
| 12 条 `<inertial>`（质量/惯量/质心）| **上游原样** | 这份 URDF **自带惯性**（与 hzmirm 那份不同）⇒ 不需要我们补；合计 **9.5521 kg** |
| 12 个 `<visual>`（`package://robot11/meshes/*.STL`）| **上游原样** | **原始 mesh，视觉零损失** |
| `<collision>` ×12 | **我们换掉** | 上游 12 个全是 mesh（`body` 208 万面）。换成：`body` = 4 个 DP box；`l2..l5/l10/l11` = 各自 mesh 包围盒的 box；`l6..l9` = cylinder(r=0.058, l=0.045)；**`livox_frame` 与 `imu_link` 故意不给碰撞**（理由见 §10.5）|
| `body_to_livox` 的 `rpy` | **上游原样 + 参数化** | 默认 = CSV 的 `-0.523598775598293 0 0`（roll）；`livox_tilt_axis:=pitch` 换成 `0 -0.523598775598293 0`（URDF 原文）|
| `imu_link` + `imu_joint`（fixed，挂 `livox_frame` 下方 0.05 m，**跟着雷达斜 30°**）| **我们加的** | 见 §10.3 的三条理由 |
| MID-360 射线传感器（`type="ray"`，挂在**上游自己的** `livox_frame` 上，sensor 名 = `livox_frame` ⇒ 点云 `frame_id` = `livox_frame`）| **我们加的** | **没有**用 `ros2_livox_simulation` 的 `mid360` 宏：宏会 new 一个同名 link + 关节，与上游的 `livox_frame` 冲突 ⇒ 把宏里那段 `<sensor type="ray">` 逐字抄过来、只改 sensor 名。参数与宏一致：100×360 / 10 Hz / 0.1–200 m / σ=2 mm / 垂直 −7.22°…+55.22° |
| `libgazebo_ros_planar_move.so` 底盘插件 | **我们加的**（与默认模型同款）| `cmd_vel→/cmd_vel_chassis`、`odom→/odom_ground_truth`、`publish_odom_tf=false` ⇒ 契约不变 |
| `gazebo_ros_imu_sensor`（100 Hz，`/livox/imu`，`frame_name=imu_link`）| **我们加的** | 与默认模型同款 |
| 13 条 `<gazebo><material>` | **我们加的** | 纯观感 |
| `<xacro:arg name="xyz"/"rpy">` | **我们加的**（声明但不用）| launch 会无条件传平台外参；本模型的雷达位姿由上游关节链唯一决定 |

### 10.3 IMU 为什么"跟着雷达一起斜"，而不是"挂在底盘上保持水平"

三条理由（这是本槽位最容易做错、也最容易把里程计搞歪的一处）：

1. **物理**：MID-360 的 IMU 与雷达本来就在**同一个壳**里，跟着雷达斜才是事实；
2. **改动面**：本栈三份 LIO 配置（`small_point_lio / FAST_LIO / point_lio`）的外参都是
   `extrinsic_T=[0,0,0.05]`（雷达在 IMU 系下、**无相对旋转**）。照抄"IMU 在雷达下方 0.05 m 且不转"
   就**一个字节都不用改**；若改挂到 `body` 上保持水平，就必须给三份 LIO 配置补一条 30° 的
   `extrinsic_R`（更大的改动面、且要同时改三处才自洽）；
3. **后果可控**：IMU 的 z 轴相对 `base_link` 斜 30° ⇒ 由 `lio_tf_adapter` 的 **`rpy`** 精确补偿
   （该节点本来就支持 rpy）。补偿量**不是手抄的**，是 launch 里的 `_LioAdapterRobot11` 从
   URDF 几何算出来的（`T_imu_link←base_link`，随 `livox_tilt_axis` 变）：

   | 候选 | `xyz`（T_imu←base_link）| `rpy` |
   |---|---|---|
   | roll | `[-0.000561701, -0.034862345, -0.151448296]` | `[0.523598776, 0, 0]` |
   | pitch | `[-0.079000533, -0.130915824, -0.085709533]` | `[0, 0.523598776, 0]` |

   两种候选下 IMU 离地都是 **0.216226 m**（= 0.2595 − 0.05·cos30°，由几何唯一决定）。

### 10.4 无头 A/B 实测（`mode:=mapping`、`lio:=small_point_lio`、RMUL2026、隔离无头跑）

口径：默认模型那轮是**静止 12 s**（`--drive-seconds 0`）；robot11 两轮是**静止 20 s + 直线 10 s**
（0.2 m/s），所以"位移"两列不可直接比，其余静止量可比。原始数据：`.tmp_robotslot/{r11_default,r11_roll,r11_pitch}/probe.json`。

| 指标 | 默认模型（对照） | `robot:=robot11` roll | `robot:=robot11` pitch |
|---|---|---|---|
| spawn | ✅ | ✅（同样出现那条"实体已入队但服务超时"的**瞬时竞态**，随后模型正常出现）| ✅ |
| TF 帧（实见） | 9 | **13**：`body→{l2,l3,l4,l5,l10,livox_frame}`、`l2→l6`/`l3→l7`/`l4→l8`/`l5→l9`、`l10→l11`、`livox_frame→imu_link` | 同左 |
| RTF | 0.804 | 0.747 | 0.734 |
| `/livox/lidar/pointcloud` | 10.0 Hz | 10.0 Hz | 10.0 Hz |
| `/livox/imu` | 100.0 Hz | 100.0 Hz | 100.0 Hz |
| `/scan` | 10.0 Hz | 10.0 Hz | 10.0 Hz |
| **点/帧（中位）** | **6373** | **19717** | **19049** |
| **`/segmentation/ground` 点/帧** | **2626** | **529** | **570** |
| `/scan` 有效波束（中位） | 1102 | 1108 | 1032 |
| `/scan` inf 比 | ~0.20 | 0.242 | 0.294 |
| **`/scan` 最近回波** | 0.755 m | **0.05 m** | **0.05 m** |
| **< 1 m 的波束（累计）** | —（未统计）| **346950** | **265675** |
| 1–2 m / 2–4 m / 4–7 m / >7 m | — | 8136 / 8201 / **0** / **0** | 17384 / 12958 / 20248 / **0** |
| 点云 `z∈[−0.02,0.03]` 的占比 | — | 极高（`z_hist` 前 3 名都是 0.00–0.02）| 同左 |
| **LIO `/odom`** | ✅ 有数据 | ❌ **有发布者（publishers=1）但探针窗口内一条消息都没有** | ❌ 同左 |
| 真值位移（10 s 直线）| 0.002 m（没行驶）| 0.536 m | 0.541 m |
| 车体最大俯仰/侧倾 | 0.02° | **0.11°** | 0.11° |
| 契约（`/cmd_vel_chassis`、`/odom`、`/livox/imu` 发布者）| 各 1 | 各 1 | 各 1 |

**"点头"没了**：hzmirm 槽位实测直线行驶时车体俯仰 **6.45°**（因为它的模型只有底盘 box 有碰撞、
轮子没有碰撞 ⇒ 平底盒子在滑）。robot11 **四个轮子是真 cylinder 碰撞、底盘离地 5.46 cm**
⇒ 实测最大俯仰 **0.11°**。这是本模型**相对 hzmirm 明确更好**的一条。

### 10.5 雷达 30° 的实测判定：**轴看得出来，但"哪个对"文件说了算**

**(1) 先说一条能省掉一半测量的几何事实（§9.5 已推、这里实测复核）**
MID-360 的方位是 **360°** ⇒ "绕 x 转 30°"与"绕 y 转 30°"给出的**射线方向集合完全相同**
（只差一个绕 z 的旋转）⇒ **世界仰角谱 / 地面环半径 / 盲区大小都区分不了它们**。实测：

| 分位数 | roll | pitch |
|---|---|---|
| p1 | −33.03° | −32.89° |
| p5 | −27.25° | −27.66° |
| p25 | −9.74° | −10.23° |
| min | **−37.20°** | −34.48° |

（p50 以上开始分叉，是因为两个候选把**同一台车**的**不同方位**采样进了视场，不是倾斜本身变了。）
两者降下界的**理论值都是 −37.22°**（= 本仓 mid360 宏的 −7.22° 再下俯 30°）⇒ 都**朝下**，
地面可见环半径都是 **0.3416 m**（= 0.2595/tan 37.22°）。

**(2) 唯一能区分的判据 = "最朝下的那一束指向 body 的哪个方位"，据此实测（离线分析 dump 的点云帧）：**

| 候选 | 世界仰角 < −34° 的回波方位峰 | 与几何预言 |
|---|---|---|
| **roll**（`rpy=-0.5236 0 0`）| **75° 与 105°**（中心 **+90° = +y**）| ✅ 完全一致（roll→最朝下为 ±y）|
| **pitch**（`rpy=0 -0.5236 0`）| **165° 与 195°**（中心 **180° = −x**）| ✅ 完全一致（pitch→最朝下为 ±x）|

复现：`python3 -c` 读 `.tmp_robotslot/r11_{roll,pitch}/frames/*.csv`（传感器系 x,y,z），
按候选旋转到 body 系后统计"仰角 < −34° 的点"的方位直方图。

**(3) 结论（诚实版）**：
* **仿真不能替你决定哪个是"真的"** —— 你写 roll 它就渲染 roll，写 pitch 就渲染 pitch，
  两者在"看得见多少地面 / 盲区多大 / `sensor_height` 该填多少"上**完全等价**；
* 所以**默认取 `roll`**：它来自 **SolidWorks 导出器的 CSV**（机器生成、`-0.523598775598293` 精确到 15 位），
  而 `rpy="0 -0.5236 0"` 是**手打的一行**（只有 5 位有效数字）⇒ 论"字面真相"CSV 更可信；
* **要真正定死，只能看实物/装配图**：雷达是往**左/右侧**歪（roll）还是往**前/后**歪（pitch）。
  这一条列进 §10.7 未验证清单。

**(4) 一个必须说清的坑（我们自己的 bug，已修，但结论受影响）**：
第一版给 `livox_frame` 和 `imu_link` 都加了碰撞体。**两者都正好在雷达原点上/正下方** ⇒
每一条朝下的射线先打到自己：实测 **48% 的点 r<0.05 m**、`/scan` 最近回波恒为 **0.05 m**、
点云基本全是自击。**已改成两者都不给碰撞**（默认模型与 hzmirm 槽位的雷达 link 同样没有碰撞）。

### 10.6 这台车真正的问题：**雷达装在底盘凹槽里**（不是倾斜的问题）

修掉上面那两个自击之后，**仍然有 75.5%（roll）/78.1%（pitch）的点在 r<0.12 m**。逐点归因（把点云
变换回 body 系再与 4 个 box 比对）：**80% 的近点落在第 4 个 box（body 上塔 z∈[0.142,0.213]）的面/内部**。
原因很直接：

* 雷达原点 **z = 0.157 m**，而**底盘顶面 z = 0.213 m** ⇒ **雷达在顶板下面 5.6 cm 的凹槽里**；
* 同时直接量 `base_link.STL`：雷达轴 0.06 m 内、z∈[0.09,0.15] 有 **14086 个顶点**（最近 **1.8 mm**），
  z∈[0.16,0.18] 也有顶点落在雷达自身 6 cm 的体积里 ⇒ **上游 mesh 本身就在雷达位置有结构**（安装座/凹槽壁）；
* 把 `body` 的碰撞换成**抽稀 mesh（表面）**理论上能让射线穿过开口。试了：
  **spawn 反而干净成功**（`SpawnEntity: Successfully spawned entity [robot]`，连那条瞬时竞态都没有），
  **但在 RMUL2026 里 100 s 内一条传感器数据都没出来**（`gzserver` 没死、插件都加载了）
  ⇒ 这条路**未验**（列进 §10.7），默认仍是 4 个 box。

**结论**：以现有上游数据，这台车的雷达**看不出去**——`/segmentation/ground` 只剩 529 点/帧、
`/scan` 的近场几乎全是自击、4 m 以外一个波束都没有（roll），**LIO 也因此初始化不出来（无 `/odom`）**。

### 10.7 换成 `robot:=robot11` 时**需要跟着改**的配置键（默认值一个都没改）

| # | 文件 → 键 | 现值（默认模型标定）| robot11 需要的值 | 现状 | 不改会怎样（实测）|
|---|---|---|---|---|---|
| 1 | `linefit_ground_segmentation_ros/config/segmentation_sim.yaml` → `sensor_height` | `0.226` | **`0.2595`** | ❌ 未改（该目录属别的任务，**没动**）| 差 3.4 cm，影响小；**不是**主要问题 |
| 2 | 同上 → `gravity_aligned_frame` | `""` | **`"base_link"`** | ❌ 未改 | 雷达斜 30° ⇒ 传感器系里的"地面"是**倾斜 30° 的斜面**，linefit 的极坐标地面线模型直接失效 —— **这才是 `ground` 从 2626 掉到 529 的主因之一** |
| 3 | `rm_navigation/params/nav2_params_sim_base.yaml` → `robot_radius`（局部/全局各一处）| `0.22` | **0.30（内切）/ 0.3565（外接）**；连带 `inflation_radius` | ❌ 未改（`params` 目录禁改）| 车体 0.6×0.6 比 0.22 的圈大得多 ⇒ 规划器把"贴着墙"当可行 |
| 4 | `pointcloud_to_laserscan/config/laserscan_params.yaml` → `min_height/max_height` | `-1.0 / 1.0`（传感器系）| 数值可不变，但**含义变了**：30° 倾斜把"高度带"拧斜（同 hzmirm §4 第 9 项）| ❌ 未改（`rm_perception` 禁改）| 近场自击点会进入 `/scan`（实测 `<1 m` 波束累计 346950）|
| 5 | `rm_nav_bringup/config/traversability_criteria.yaml` → `speed_limit_lookahead_m` | `3.0` | 视雷达实际可视距离（本模型几乎没有 4 m 以外的地面点）| ❌ 未改 | 前瞻限速无意义（本来就取不到远处地面）|
| 6 | `lio_tf_adapter` 的 `xyz`/`rpy` | `[-0.12,0,-0.125]` / `[0,0,0]` | **本槽位已按模型切**（含 30° 旋转补偿，见 §10.3 表）| ✅ **已切**（`robot:=robot11` 分支）| 不切 ⇒ `odom→base_link` 会带 30° 静态倾斜 |
| 7 | 三份 LIO 配置 → `extrinsic_T` / `extrinsic_R` | `[0,0,0.05]` / 无 | **不用改**（我们把 IMU 放在雷达下方 0.05 m 且不转，刻意保持这条几何）| — | 无 |
| 8 | `fake_vel_transform` → `spin_speed` | `5.0` | 建议 `0.0`（同 hzmirm，本模型云台 `j10/j11` 我们不加控制器）| ❌ 未改 | nav 看到的 `base_link_fake` 会凭空转，而雷达没转 |

**但请注意顺序**：在 §10.6 的"雷达看不出去"解决之前，**改第 1/2/4 项都救不了** ——
先把雷达从凹槽里"放出来"（要么用户确认雷达的真实安装高度/开口，要么用能穿过开口的碰撞表示）。

### 10.8 结论、回滚、未验证

**结论：`robot:=robot11` 是一个能跑通链路的 opt-in 槽位，但"当默认模型"这个问题的答案是 ❌（现在还不是）。**
支持它的：真 mesh 视觉、真 inertial（质量/惯量不是你拍的）、四轮真碰撞 ⇒ **点头消失（0.11°）**、
足印比 hzmirm 小（内切 0.30 / 外接 0.3565）、雷达离地 0.2595 m 与现行默认模型（0.226）同量级。
挡住它的：**雷达在凹槽里 ⇒ 75.5% 的点是自击、地面分割塌到 529 点/帧、LIO 发不出 `/odom`**，
以及 30° 倾斜轴的**上游自相矛盾**还没被实物定死。

| 想退掉什么 | 怎么做 |
|---|---|
| 只退回默认模型 | **什么都不用改**：去掉 `robot:=robot11` 即可（默认路径从未改动）|
| 退掉整套 robot11 槽位 | `git revert <本主题 commit>`；删掉 `robot11_description/`、`urdf/upstream/robot11.urdf`、`sentry_robot_robot11_sim.xacro`、`robot11_*.py` 后，`robot:=robot11` 会给一条可操作的错 |
| 只换倾斜轴 | 加 `livox_tilt_axis:=pitch`（或 `roll`）|
| 只换 `body` 的碰撞表示 | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision mesh`（重新生成 xacro；**这条路未验**，见 §10.6）|
| 重新生成 xacro | `python3 tools/scripts/regress/robot11_make_sim_xacro.py`（上游 URDF 改了就重跑）|

**未验证清单（本槽位新增）**：
1. **`livox_tilt_axis` 的最终真值**：仿真只能说"roll 的最朝下方位是 ±y、pitch 是 ±x"，**不能**判定哪个是实物
   （§10.5）。需要用户看实物/装配图，或让 SolidWorks 重新导出一次 CSV 核对；
2. **`body` 用抽稀 mesh 当碰撞**这条路没跑通（spawn 成功但 100 s 内无传感器数据）⇒ 上表中"地面能不能看见"
   的结论**只对 4-box 版本成立**；
3. **`/odom` 没出来**的原因没查到底（有发布者、无消息）；未排除是 LIO 配置/时序问题而非点云质量问题；
4. **`mode:=slam_nav` / `mode:=nav`** 没跑（连 `/odom` 都没有，跑了也说明不了什么）；
5. **`localization:=*` 各槽、`ground:=patchwork`、其它 LIO 槽**与 robot11 的配合未验；
6. **抽稀碰撞件（`meshes/generated/*.stl`）在小 link 上的可用性**：实测会让 Gazebo 插入模型时卡住/段错误，
   所以本槽位对 `l2..l5/l10/l11` 用的是 box ⇒ **那 11 个抽稀件目前只有 `base_link` 那个是验证过的**；
7. **`<1 m` 波束里自击与真障碍的占比**没分离（只有总数）；
8. 大文件（99.10 MiB 的 `base_link.STL`）**已入库并 push 成功**（GitHub 只给了 >50 MiB 的 warning），
   但**没有**用 LFS ⇒ 以后每次 clone 都要拖这 104 MB。

---

## 11. Phase 3：让雷达真的看得见 + 斜置感知链（2026-10-07）

> 状态：**新增**。`robot:=robot11` 从"能跑通但看不见"变成"**雷达看得见、LIO 出 /odom、
> 地面分割 5637 点/帧**"。默认模型与所有 launch/config 默认值**仍未改**（本节的每一项偏离都逐条登记）。
> 用户 2026-10-07 的两条判定（本节据此执行）：
> **(1) 30° 是绕 roll**（用户按实物确认 ⇒ `livox_tilt_axis` 默认 roll 从"CSV 更可信"升级为"实物判据"，
> 仍保留 `pitch` 开关对照）；**(2) 上游 URDF/mesh"跟实物基本一致、仿真侧可以改"** ⇒ 允许适配，
> 但每处偏离要写日期 + 中文理由 + provenance（见 §11.3）。

### 11.1 一句话结论

Phase 2 的"雷达在凹槽里 ⇒ 75.5% 自击、地面 529 点/帧、无 `/odom`"有**三个叠在一起的根因**，
本轮全部定位并解决（**A 方案胜出**）：

| # | 根因 | 证据 | 修法 |
|---|---|---|---|
| ① | `body` 的 4 个 DP box 是**实心包络**，而雷达原点 z=0.157 **在第 4 个 box 内部**（box 顶 0.213）⇒ 每条射线先打盒内壁 | 离线 30000 条与 Gazebo 同源射线：**80.5% 落在 <0.12 m**、看见地面 **0%**（Gazebo 实测 75.5% / 529 点每帧，差异 = Gazebo 丢掉 <0.1 m 回波） | **A**：`body` 碰撞 = DP box **减去雷达视锥**（锥半角 100.22°、半径 0.5 m、内清空 0.05 m）后分解成 76 个 box |
| ② | 模型根 link 叫 **`body`**，而全链路契约帧名是 `base_link`（`small_point_lio` 源码硬编码 `lookupTransform(lidar_frame,"base_link")`）⇒ **TF 断成两棵树** | Gazebo 日志：`Failed to lookup transform from base_link to livox_frame: ... not part of the same tree`（每帧）；`/odom` **0 条**（有发布者、无消息） | 生成时把根 link 改名 `body` → **`base_link`**（上游 SolidWorks CSV 里本来就叫 `base_link`） |
| ③ | linefit 的 `gravity_aligned_frame` 路径有 **C++ bug**：`Eigen::Affine3d tf;` 默认构造**不清零** | 最小 C++ 复现 + 回放实测（§11.4）；该键="" ⇒ ground 3969 点/帧，="base_link" ⇒ **恒 0 点** | **在源头对齐点云**：`livox_frame` 帧重力对齐（joint rpy=0），30° 倾角放进**插件新增的 `<tilt_rpy>`**（射线真的斜、点云表达在父 link 系）⇒ linefit/p2l 都不用开那个键 |

### 11.2 A / B / C 三条路：实现、实测、选谁

口径：无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、`unset DISPLAY`、
`nav_rviz:=False`）、`world:=RMUL2026 mode:=mapping lio:=small_point_lio`、
探针窗口 = 静止 20 s + 直行 8 s（0.2 m/s）+ 原地偏航 5 s（0.6 rad/s）。原始数据 `.tmp_robotslot/r11p3{a,b,c,def}/probe.json`。

| 指标 | Phase 2（4 个 box） | **A（挖视锥，采用）** | B（抬高 +0.10 m） | C（抽稀网格碰撞） | **默认模型（对照）** |
|---|---|---|---|---|---|
| spawn | ✅ | ✅ | ✅ | ❌ **服务 60 s 超时**（模型没进世界） | ✅ |
| **`/odom` 消息数** | **0** | **188** | **188** | 0（无数据） | **325** |
| `/livox/lidar/pointcloud` | 10 Hz | 10 Hz | 10 Hz | **0 Hz** | 10 Hz |
| 点/帧（中位） | 19717 | **11951** | 9957 | 0 | **6256** |
| **`/segmentation/ground` 点/帧** | **529** | **5637** | 5435 | 0 | **2739** |
| `/segmentation/obstacle` 点/帧 | 19232 | 6293 | 4493 | 0 | 3473 |
| `/scan` 帧数 / 有效波束 | 254 / 1108 | 188 / 833 | 188 / 809 | 0 | 325 / 1174 |
| **`/scan` >4 m 波束（每帧）** | **0** | **≈81** | ≈220 | 0 | **≈284** |
| **`/scan` <1 m 波束（每帧）** | 1366 | ≈521 | ≈268 | 0 | ≈145 |
| `/scan` 最近回波 | 0.05 m | 0.05 m | 0.05 m | — | 0.481 m（Phase 2 口径） |
| **自击 `r<0.12 m`（云点口径）** | **75.5%** | **29.0%** | 26.2% | — | **0.0%** |
| 自击 `r<0.05 m` | 48% | 27.4% | 24.8% | — | 0.0% |
| RTF | 0.747 | 0.459 | 0.456 | — | **0.79** |
| **LIO 漂移（直线段 8 s，位移差）** | 无 `/odom` | **0.024 m / 0.26°** | 0.027 m / 0.34° | — | **0.006 m / 0.03°** |
| **LIO 漂移（偏航段 5 s）** | 无 `/odom` | **0.0002 m / 0.004°** | — | — | **0.0008 m / 0.005°** |
| 车体最大俯仰/侧倾 | 0.11° | 0.28° / 0.12° | — | — | 0.51° / 0.02° |
| 探测窗口 | 25.4 s | 18.8 s（188 帧） | 18.8 s | — | 32.5 s（325 帧）|

⚠️ 两处读表提醒：① 默认模型的窗口更长（RTF 高 ⇒ 同样墙钟里跑出更多仿真帧），
所以"累计波束"必须换算成**每帧**再比（上表已换算）；② robot11 的 `r<0.12 m` 自击**不是遮挡**
（离线射线求交归因：近点 100% 落在**云台** `l10/l11` 上，中位距离 0.14 m —— 那是**真实存在**的
部件，离雷达只有 0.12~0.17 m；真车 360° 雷达也会照到它），而默认模型的雷达在车顶中间、
周围 0.3 m 内没有部件 ⇒ 0%。

**A 方案点云的物理复核**（`cloud_frame_01.csv`，11616 点）：body 仰角谱 **−40°…+85°**（= 真的斜 30° 下俯）、
**地面带（|z+0.2595|<0.06）占 46%**、地面距离 p5/p50/p95 = **0.39 / 0.74 / 2.49 m**（最近地面环 = 0.39 m，
与几何预言 0.2595/tan37.22° = 0.43 m 一致）、`r>4 m` 的点占 4.5%。

**A 方案的离线—在线一致性**（这是"离线筛方案、Gazebo 复核"这条纪律的验证）：
离线 30000 条射线预测"自击 <0.12 m = 0%、看见地面 25.2%"；Gazebo 实测自击 **29.0%**、地面 **46%**（云点口径）。
差异来自离线模型①只算了 `body` 的碰撞（Gazebo 还有云台/轮子）②按**射线**统计而 Gazebo 按**点数**统计。

#### 11.2.1 默认模型对照（同口径）

`.tmp_robotslot/r11p3def2/probe.json`（本轮补跑；`robot` 留空、其余参数与 A/B 完全一致）。
关键数字已并入 §11.2 主表：**`/odom` 325 条、点/帧 6256、地面 2739 点/帧、自击 0%、RTF 0.79、
直线漂移 0.006 m / 0.03°、偏航漂移 0.0008 m / 0.005°**。
> ✅ **共享代码改动的回归证据**：这一跑是在**插件 `<tilt_rpy>` 与 linefit 的 `Identity()` 修复都已经编译进
> 二进制之后**跑的，数字与 Phase 2 的默认模型（`default2`：6270 点/帧、2707 地面点/帧、自击口径同）
> 在噪声内一致（6256 / 2739）⇒ "缺省参数 = 单位阵 / 只动那条 if 分支"这两条**结构性论证**得到了实测支持。

⇒ 逐项结论：**A 方案的 robot11 在"看得见"上不输默认模型**（地面 5637 > 2739 点/帧、
点数/帧 11951 > 6256），代价是 **RTF 0.46 vs 0.79**、自击 29%（云台，物理真实）、
`/scan` 的远场波束更少（≈81/帧 vs ≈284/帧 —— 因为 360° 下俯 30° 后有相当一部分视场给了地面与自身）。

> ⚠️ 一次踩坑记录：同一 tag 连跑两次时，**上一次被中断的 launch 还在同一 domain/port 上活着**
> ⇒ 节点清单里同名节点出现两份、`/cmd_vel_chassis` 被两套栈抢、probe 只收到约 1/50 的消息
> （`counts` 只有 6 帧、漂移恒 0）。判据：`ros2 node list` 里出现重复名字。
> 处置：**换 tag**（tag 决定 domain/port）+ 跑前确认无同名残留；本次对照跑用的是 `r11p3def2`，有效。

#### 11.2.2 C 到底为什么不可用（根因，不是猜）

Phase 2 记的是"spawn 干净成功但 100 s 内无传感器数据"。本轮在 RMUL2026 里复现：
**`spawn_entity` 服务 60 s 超时**（`Spawn service failed. Exiting.`），而 `gzserver` 日志里
**插件该有的初始化全都在**（`LivoxPointsPlugin: load csv ... scan info size: 800000 / sample: 30000 /
tilt_rpy = ...`），只是**一帧都打不出来**（90 s 内 cloud/imu/scan 全 0）。⇒ 结论：
**不是"插件没挂上"、也不是"link 被丢掉"、更不是"射线被挡住"**，而是
**ODE 的 ray-vs-trimesh 求交代价是 O(三角面数)**：30000 条射线 × 3000 面 ≈ **9×10⁷ 次/帧 @10 Hz**，
Gazebo 在"打一帧射线"里出不来（spawn 阶段的空间重建也一起卡）。这条路**不可用**，且不是"参数问题"。

### 11.3 与上游的偏离清单（provenance：哪些是他们的、哪些是我们改的、为什么）

`src/rm_nav_bringup/urdf/upstream/robot11.urdf` **仍然逐字节未动**（sha256 `e3e322ac…a64fc`）。
生成物 `sentry_robot_robot11_sim.xacro` 由 `tools/scripts/regress/robot11_make_sim_xacro.py` 从上游 + 清单重算。

| # | 元素 | 上游 | 我们改成 | 日期 | 为什么（判据/实测） |
|---|---|---|---|---|---|
| 1 | 12 个 `<collision>` | 全是 mesh（`body` 208 万面） | `body` = 76 个 box（A 方案）；云台 `l10/l11` = 7/22 个 box；轮 = cylinder；其余 = 网格包围盒 box | 10-07 P1/P3 | Phase 1 实测：原网格一接触 RTF 0.20 / RSS +587 MB；Phase 3 实测：4 个实心 box 让 75.5% 的点变自击 |
| 2 | `body` 的根 link 名 | `body` | **`base_link`** | 10-07 P3 | 全链路契约帧名（LIO 源码硬编码）；**上游 SolidWorks CSV 里本来就叫 `base_link`**；不改名实测 TF 断树、`/odom` 0 条 |
| 3 | `body_to_livox` 的 `rpy` | `-0.523598775598293 0 0`（CSV 的 roll 形式） | **`0 0 0`**（帧重力对齐），倾角搬到插件参数 `<tilt_rpy>` | 10-07 P3 | linefit 的 `gravity_aligned_frame` 在本仓有 C++ bug（§11.4）；帧正、点云就正（插件把点发布在父 link 系）；**射线仍真的斜 30°** |
| 4 | `body_to_livox` 的 `origin.z` | `0.15702816968305` | `0.15702816968305 + $(arg livox_raise_m)`（**默认 0**） | 10-07 P3 | 只为 B 方案的实测对照；默认路径与上游逐字相同 |
| 5 | `imu_link` + `imu_joint` | 上游**没有** | 固定关节、沿 body −z 走 0.05 m（跟随雷达位置） | 10-07 P2 | 三份 LIO 配置的 `extrinsic_T=[0,0,0.05]`、`extrinsic_R=I` **逐字节不用改**；IMU 与雷达"只差 0.05 m 平移"这条几何没变（IMU 的**绝对**姿态从"跟着斜"变成"与 body 同姿态"，LIO 自己会估） |
| 6 | MID-360 射线传感器 | 上游**没有** | `<sensor type="ray" name="livox_frame">`（参数与 `mid360.xacro` 宏逐字一致）+ **新增 `<tilt_rpy>`** | 10-07 P2/P3 | 本栈需要 ≥10 Hz 的 `/livox/lidar`；`<tilt_rpy>` 装安装倾角（缺省单位阵 ⇒ 其它模型逐字节不变） |
| 7 | 底盘插件 / IMU 传感器 / 材质 | 上游**没有** | `libgazebo_ros_planar_move.so`（`cmd_vel→/cmd_vel_chassis`、`odom→/odom_ground_truth`、`publish_odom_tf=false`）、`gazebo_ros_imu_sensor`（100 Hz → `/livox/imu`）、13 条材质 | 10-07 P2 | 与默认模型同款 ⇒ 契约不变（单一 `/cmd_vel_chassis` 订阅、单一真值里程计） |
| 8 | `livox_points_plugin.cpp`（**仿真包，非上游 URDF**） | 发布点时不带安装姿态（`axis = ray·x̂`） | 新增 SDF 参数 `<tilt_rpy>`：射线方向按它偏、点云仍表达在**父 link 系** | 10-07 P3 | 让"物理上斜 30°"与"点云坐标/帧自洽"同时成立（§11.5）；**缺省 = 单位阵 ⇒ 默认模型/hzmirm 输出逐字节不变** |
| 9 | 云台 `l10/l11` 的碰撞 | mesh | 由网格推出的 7 / 22 个 box（体素 15 mm + 闭运算 + 填孔 + 贪心分解） | 10-07 P3 | 单包围盒离雷达只有 0.099/0.091 m 且**实心**，实测 34.7% 的射线打在它面上；真网格最近 0.116/0.139 m ⇒ 细盒把"碰撞近似造成的自击"降到接近 0 |

### 11.4 ★ linefit 的 `gravity_aligned_frame` 在本仓**不可用**（C++ bug，诚实更正 §3.3/§10.7）

**位置**：`src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`
（该目录属别的任务，本主题**没有改**）：

```cpp
Eigen::Affine3d tf;                 // ← Eigen 的默认构造**不清零**（只把仿射最后一行置 (0,0,0,1)）
tf.translate(Eigen::Vector3d(0, 0, 0));
tf.rotate(Eigen::Quaterniond(...)); // ← 在垃圾矩阵上叠一个旋转
pcl::transformPointCloud(cloud, cloud_transformed, tf);   // ⇒ 点全被塌到 ~1e-310
```

**三条独立证据**：

1. **最小 C++ 复现**（`g++ -I/usr/include/eigen3`，Eigen 3.4）：
   `Eigen::Affine3d tf; tf.rotate(q); (tf * Vector3d(0.1,0.2,0.3))` → `(7.5e-310, 7.5e-310, 7.5e-310)`；
   把第一行换成 `Eigen::Affine3d::Identity();` → `(0.1, 0.323, 0.160)`（正确）。
2. **离线复算**（`tools/scripts/regress/linefit_offline_probe.py`，本轮新增 `--roll-deg`）：
   同一帧、`sensor_height=0.2595`，把点云**先转 −30°**（= 该键生效时应有的效果）
   ⇒ linefit 判地面 **5100/11711 = 43.5%**，其中 81% 真的是最低那层地面；不转 ⇒ 23%（但那 23% 里只有 0.7% 是真地面）。
3. **回放活的 linefit 节点**（无 Gazebo：静态 TF + 以 10 Hz 发 dump 的那一帧；脚本 `.tmp_robot11/p3/replay_linefit.py`）：
   `gravity_aligned_frame=""` ⇒ `/segmentation/ground` **3969** 点/帧；
   `="base_link"` ⇒ **0** 点/帧（把输入点云预先转 −30/+30/−60/+60 都一样是 0 ⇒ 与"转多少"无关，只与"走没走那条分支"有关）。

**连带更正**：§3.3/§5.1 把 hzmirm 槽位"雷达 0.80 m + 下视 −7.22° ⇒ 地面分割全废"归因于**几何**。
几何那条**仍然成立**（0.8 m 高、下视 7.22° ⇒ 6.3 m 以内没有地面点），但 hzmirm 的 YAML 同时开了
`gravity_aligned_frame: "base_link"` ⇒ 它的 `/segmentation/ground` **恒为 0** 里，有一部分是**这个 bug**。
复现：`hzmirm`/`tilt_pitch10` 的 probe.json 里 `ground_points_per_frame.median` 都是 **0**。
⇒ **那条归因需要按"几何 + bug"两条并列重读**（本轮只做更正与登记，不改 hzmirm 的配置）。

**状态：已修（2026-10-07，经负责人授权）**。修法（一行 + 一段注释）：
`src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`
```cpp
Eigen::Affine3d tf = Eigen::Affine3d::Identity();   // 原来：Eigen::Affine3d tf;（未初始化）
```
**回归证据**（`tools/scripts/regress/linefit_replay_probe.py`，同一帧 `f_rot0.csv`、
同一个活的节点、无 Gazebo；BEST_EFFORT 订阅）：

| 配置 | 修前 | 修后 |
|---|---|---|
| `gravity_aligned_frame: ""`（默认路径） | ground **3969** 点/帧 | ground **3969** 点/帧（**逐字节口径一致**） |
| `gravity_aligned_frame: "base_link"` | ground **0** 点/帧 | ground **3969** 点/帧（obstacle 7742） |

结构性论证（与上面那张表互补）：该修复**只动 `if (!gravity_aligned_frame_.empty())` 分支内部**，
默认路径根本不进这块代码 ⇒ 默认模型行为不可能变化（用同一帧实测也确认了 3969 → 3969）。
⚠️ 本节记录这条修复**不改变** §11.3/§11.5 的 robot11 设计选择（点云在源头对齐 + 插件 `<tilt_rpy>`）：
那条路仍然更干净（p2l 不用建 tf2 MessageFilter）。修好之后"点云留在传感器系 +
`gravity_aligned_frame: base_link`"这条**更接近真机**的链路**重新可用**（见 §11.8 第 2 项）。

### 11.5 斜置感知链怎么处理的（本槽位专属，默认模型逐字节不变）

| 环节 | 之前（Phase 2） | 现在（Phase 3） | 为什么 |
|---|---|---|---|
| 点云坐标系 | 传感器系（斜 30°，`frame_id=livox_frame`） | **重力对齐**（`frame_id` 仍是 `livox_frame`；帧本身 rpy=0） | 本仓 linefit 的 `gravity_aligned_frame` 不可用（§11.4）⇒ 在**源头**对齐，等价于"驱动 + TF 正确投影"后的结果，且保住了 p2l 的 `target_frame=""` 加固（不建 tf2 MessageFilter） |
| linefit | `sensor_height: 0.226`、`gravity_aligned_frame: ""`（在斜系里拟合 ⇒ 假地面） | **`sensor_height: 0.2595`**（= 雷达离地实测）、`gravity_aligned_frame: ""`（点云已对齐） | `sensor_height` 的语义 = 雷达离地（linefit `segment.cc:30` `cur_ground_height = -sensor_height_`）；0.226 会让地面线整体高 3.35 cm |
| p2l | `target_frame: ""`、带 `min/max_height` 在**斜系**里量 | **不改**（`target_frame: ""`、`min_height −1.0 / max_height 1.0`） | 点云已重力对齐 ⇒ 高度带 = 真高度；**且不需要 TF**（2026-09-23 刻意避开的故障面保持关闭）。实测 `/scan` `frame_id=livox_frame`、10 Hz、>4 m 波束 15204 条 |
| LIO 外参 | `extrinsic_T=[0,0,0.05]`、`extrinsic_R=I` | **不改**（三份配置逐字节未动） | IMU 与雷达只差 0.05 m **平移**（`imu_joint` 沿 body −z 0.05 m）⇒ 这条几何本来就成立；30° 由 LIO 自己估（`odom→base_link` 由 `small_point_lio` 用 TF 精确换算） |
| `lio_tf_adapter` 杆臂 | 由 URDF 几何算出（含 30° rpy） | 由 launch **算出来**（现在是纯平移 `[-0.000562, -0.130916, -0.107028]`、rpy=0） | 帧变成重力对齐后，杆臂里不再有 30°；仍是"从模型几何算"、不是手抄 |

**验证（漂移口径 = 位移差，不是 odom 与真值的绝对差 —— 后者混着出生点偏移）**：
`tools/scripts/regress/robot_model_probe.py` 本轮新增 `--yaw-seconds/--yaw-rate` 与 `phase_drift`。

| 段 | A（robot11） | 默认模型（同口径） |
|---|---|---|
| 直行 8 s（0.2 m/s） | 0.024 m / 0.26° | 见 §11.2.1 |
| 原地偏航 5 s（0.6 rad/s） | 0.0002 m / 0.004° | 见 §11.2.1 |

### 11.6 `robot:=robot11` 的槽位专用配置增量（默认值一个都没改）

| # | 文件 → 键 | 默认模型 | robot11 | 为什么 | 副作用 |
|---|---|---|---|---|---|
| 1 | `linefit …/config/segmentation_sim_robot11.yaml`（**新增**）→ `sensor_height` | 0.226 | **0.2595** | 实测雷达离地（几何：0.157028+0.102499） | 无（逐键副本，只改这一个键） |
| 2 | 同上 → `gravity_aligned_frame` | `""` | `""`（**不改**） | 点云已在源头对齐；且该键在本仓有 bug | 无 |
| 3 | `pointcloud_to_laserscan/config/laserscan_params.yaml` | — | **不改** | 高度带随点云一起变成重力对齐 | 无 |
| 4 | 三份 LIO 配置 → `extrinsic_T/R` | `[0,0,0.05]` / I | **不改** | IMU 与雷达仍是纯平移关系 | 无 |
| 5 | launch `spin_speed` 默认值 | `5.0` | **`0.0`**（仅当 `robot:=robot11`） | 本模型四个轮子是真 cylinder、云台 `j10/j11` 我们没有加控制器 ⇒ "小陀螺"在仿真里既无执行机构也无实测依据 | 只对 robot11 生效（`--show-args` 可见 `5.0` 仍在其它槽位） |
| 6 | nav2 `robot_radius` / `inflation_radius` | 0.22 / 0.5(局部)·0.55(全局) | **建议 0.3565（外接）或 footprint 多边形；inflation ≥ 0.65** | 足印内切 **0.300** / 外接 **0.3565**（Phase 1 实测）；0.22 的圈比车小得多 | **未落地**（`src/rm_navigation/**/params` 属别的任务，本主题禁改）⇒ 见 §11.8 未验证清单 |
| 7 | `traversability_criteria.yaml` → `speed_limit_lookahead_m` | 3.0 | **不改（3.0 够用）** | 实测地面点覆盖 **0.39–5 m**（`ground_r_hist`），3.0 m 前瞻里有充足地面点；hzmirm 那种"雷达高+下视窄 ⇒ 前瞻里没有地面点"的问题在这台车上不存在 | 无 |

### 11.7 怎么试 / 怎么退

```bash
# ① 构建（install/ 是逐文件符号链接；新 xacro/新 YAML/改过的插件都必须 build）
colcon build --symlink-install --packages-select robot11 rm_nav_bringup \
    linefit_ground_segmentation_ros ros2_livox_simulation

# ② 一条命令量齐（隔离无头跑 + 探针 + 契约检查）→ .tmp_robotslot/<tag>/
tools/scripts/regress/run_robot_model_probe.sh r11 --duration 20 --drive-seconds 8 \
    --yaw-seconds 5 --yaw-rate 0.6 -- world:=RMUL2026 mode:=mapping \
    lio:=small_point_lio robot:=robot11 map_autocontinue:=False

# ③ 只看雷达"看得见什么"（离线，秒级，不用 Gazebo）
python3 tools/scripts/regress/robot11_livox_fov.py --compare          # mesh / 4 box / A 三选一对照
python3 tools/scripts/regress/robot11_livox_fov.py --emit-carved      # 重算 A 的 box 清单
python3 tools/scripts/regress/robot11_livox_fov.py --emit-turret-boxes # 重算云台细盒

# ④ 换 30° 绕哪个轴（默认 roll = 用户按实物确认）
#    ... robot:=robot11 livox_tilt_axis:=pitch ...

# ⑤ B 方案复现（抬高雷达）—— **必须同时改 linefit 的 sensor_height**
#    python3 tools/scripts/regress/robot11_make_sim_xacro.py --livox-raise-m 0.10   # 生成物
#    把 config/segmentation_sim_robot11.yaml 的 sensor_height 改成 0.3595（= 0.2595 + 0.10）
#    ... robot:=robot11 livox_raise_m:=0.10 ...

# ⑥ 退回默认模型：去掉 robot:=robot11 即可（默认路径从未改动）
```

| 想退掉什么 | 怎么做 |
|---|---|
| 退回默认模型 | 去掉 `robot:=robot11`（默认路径逐字节未改） |
| 退回 Phase 2 的"实心 4 box" | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision boxes` 后重建 |
| 退回 Phase 2 的"点云在传感器系" | 生成器里把 `body_to_livox` 的 rpy 换回 `$(arg livox_tilt_rpy)` 并去掉插件 `<tilt_rpy>`（**会立刻踩 §11.4 的 bug：地面分割 0 点**） |
| 整套 robot11 | `git revert <本主题 commit>` + 删 `robot11_description/`、`urdf/upstream/robot11.urdf`、`sentry_robot_robot11_sim.xacro`、`robot11_*.py`、`segmentation_sim_robot11.yaml` |

### 11.9 `robot:=robot11` 的 nav2 几何（槽位覆盖层，默认模型不动）

问题：nav2 的 `robot_radius` 是"实心圈"半径（本仓没有 footprint 多边形），
默认模型的 0.22 对 robot11 远远不够（足印内切 **0.300** / 外接 **0.3565**）。
约束：`src/rm_navigation/**/params` 属别的任务，`nav2_params_sim_base.yaml` **不能改**。

修法（**robot 槽位作用域**，与 linefit 的槽位文件同一套机制）：
新增 `src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`（**只含几何键**），
launch 里新增 `_Nav2ParamsForSlot`：`robot:=robot11` 时把这份覆盖层**深合并**到公共段之上、
写成一份临时 YAML 传给 nav2；**其它槽位原样返回公共段路径**（不合并、不写临时文件）。

| 键 | 默认模型 | robot11 | 为什么 |
|---|---|---|---|
| `local_costmap.robot_radius` | 0.22 | **0.3565** | 凸包外接半径（保证不撞的上界）；取内切 0.30 会让四个角扫到障碍 |
| `local_costmap.inflation_layer.inflation_radius` | 0.5 | **0.70** | 必须 ≥ robot_radius；软带厚度与默认模型同量级（0.34 m） |
| `global_costmap.robot_radius` | 0.22 | **0.3565** | 同上 |
| `global_costmap.inflation_layer.inflation_radius` | 0.55 | **0.75** | 同上（软带 0.39 m） |
| `cost_scaling_factor` | 3.0 / 2.5 | **不动** | 与半径无关 |

**单元级验证**（同一份代码、三种槽位，`ros2 param` 之外的最小复现）：

| `robot` | 传给 nav2 的公共段 | local `robot_radius` / `inflation` | global 同上 |
|---|---|---|---|
| `''`（默认） | `nav2_params_sim_base.yaml`（原路径） | 0.22 / 0.5 | 0.22 / 0.55 |
| `hzmirm` | 同上（原路径） | 0.22 / 0.5 | 0.22 / 0.55 |
| `robot11` | 合并后的临时 YAML | **0.3565 / 0.70** | **0.3565 / 0.75** |

**运行时验证（Gazebo `mode:=slam_nav`，无头隔离，一次跑；`.tmp_robotslot/r11p3nav/`）**：

| 检查 | 结果 |
|---|---|
| `ros2 param get /local_costmap/local_costmap robot_radius` | **0.3565** ✓（覆盖层真的生效） |
| `ros2 param get /global_costmap/global_costmap robot_radius` | **0.3565** ✓ |
| `ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius` | **0.7** ✓ |
| `/controller_server` / `/planner_server` lifecycle | 都是 **active [3]** ✓（配置+激活通过） |
| `NavigateToPose`（map 系，车前 1.5 m） | **被接受**（Goal accepted with ID …）✓ |
| **车有没有真的走过去** | ❌ **没有**：90 s 内真值位置只动了 1.8 cm（4.300→4.305, 3.350→3.368）。疑似"地图还没建起来/规划器没出路径"，本轮**没查到底**（会话被中断）⇒ 见 §11.8 第 1 项 |

⚠️ 也就是说：**覆盖层与栈的配置/激活链路已实测通过**，"能按新半径真的走一段"**尚未验证**。

### 11.8 未验证清单（Phase 3 新增，诚实清单）

1. **nav2 的运行时验证没跑完**：`robot_radius`/`inflation_radius` 的**槽位覆盖层已落地**
   （§11.9，单元级验证通过：默认/hzmirm 仍是 0.22，robot11 是 0.3565），但
   `mode:=slam_nav`/`mode:=nav` 的 configure/activate + 短目标**没有实测**（会话被中断）
   ⇒ 已补的冒烟证据：`robot_radius` 运行时确实是 0.3565、`controller/planner` active、
   `NavigateToPose` 被接受；**但车 90 s 内没动**（真值只挪 1.8 cm，疑似"地图/规划"环节，
   没查到根因）⇒ 真跑 nav 前必须把这一段走通；
2. ~~linefit 的 C++ bug 没有修~~ → **已修并回归**（§11.4 末尾，一行 `Identity()`）。
   仍**未做**的：把 robot11 切回"点云留在传感器系 + `gravity_aligned_frame: base_link`"这条
   更接近真机的链路（修好后已具备条件），以及 hzmirm 槽位在新代码下的重跑；
3. **hzmirm 槽位的结论未重跑**：§11.4 的更正只是"读数据 + 复现 bug"，没有重跑 hzmirm 验证
   "关掉 `gravity_aligned_frame` 后它的 ground 会不会从 0 变成别的值"（预期仍偏低，因为几何确实差）；
4. **`<1 m` 的近场点里"云台自击"与"真障碍"仍未分离**（本轮的归因是**离线射线求交**：
   近点里 100% 落在 `l10/l11` 上，中位距离 0.14 m ⇒ 但那是**零位关节角**下的云台；`j10/j11` 是
   `continuous`，仿真里没有控制器 ⇒ 云台姿态不会变）；
5. **`livox_raise_m≠0`（B 方案）与 linefit `sensor_height` 的同步是**手工**的**（launch 不替改 YAML）
   ⇒ 忘记改就会出现"地面线整体偏 h"的静默错误；本轮 B 的实测就是手工同步后跑的；
6. **插件 `<tilt_rpy>` 只验证了 roll 一档**（`-0.5236 0 0`）；`pitch` 档没跑 Gazebo（几何上等价，只差绕 z 一转）；
7. **IMU 的绝对姿态**从"跟着雷达斜 30°"改成"与 body 同姿态"（§11.3 第 5 项）：LIO 实测正常
   （漂移 0.024 m），但没有单独 A/B"IMU 跟着斜"的对照；
8. **`/scan` 的 98022 条 <1 m 波束里，云台自击占多少**没有单独统计（只有累计数）；
9. **插件改动没有跑默认模型/hzmirm 的回归**（论证是"缺省参数 = 单位阵 ⇒ 逐字节不变"，
   但**没有**实测对照）⇒ 收尾前应补一次默认模型的 A/B（本轮补跑了默认模型对照，见 §11.2.1）。

## 12. Phase 4：让 `robot:=robot11` 能带 GUI 用 + 自击不再污染限速 + 接线的**运行期**证据（2026-10-07）

> 状态：**新增**。触发 = 用户 2026-10-07 带 GUI 的一次实跑
> （`ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping lio:=small_point_lio
> robot:=robot11 nav_rviz:=True`，~13 s 后 Ctrl-C）暴露的三个缺陷：
> ① `gzclient` 被 SIGKILL（`exit code -9`）、rviz2 黑屏且 5 s 内退不掉；② 车**静止**时限速被钉在
> 0.60 m/s 地板；③ launch 横幅说"linefit 参数没有跟着切"，而代码里其实**已经切了**（横幅在骗人）。
> 默认模型与所有 launch/config 默认值**仍未改**（本轮每一项偏离都逐条登记；`robot:=robot11` 之外
> 的行为逐字节不变）。

### 12.1 一句话结论

| # | 缺陷 | 根因（实测） | 修法 | 现在的状态 |
|---|---|---|---|---|
| ① | GUI 被杀 / 黑屏 | 视觉是上游原始 STL：**12 个 mesh 合计 2,821,320 面 / 141.07 MB**，其中 `base_link.STL` 一项 99 MiB / **2,078,226 面**。实测 rviz2 峰值 RSS 随它在 **243 MiB ↔ 1444 MiB** 之间（+1.2 GB） | 新增 `robot11_visual:=decimated|full`（**默认 decimated**）：`<visual>` 复用 Phase 1 已入库的抽稀件 `generated/*_collision.stl`（**9,700 面 / 0.486 MB**，bbox 与原件差 ≤3 mm、单位/原点不变） | **可交互**（虚拟屏 + 软件 GL 下 gzclient/rviz2 都活满窗口、rviz2 有像素） |
| ② | 静止也被压到地板 | 雷达装在底盘凹槽里 ⇒ **26.9~29.0% 的点落在 r ≤ 0.09 m**，它们在前瞻走廊里造出 d=0.00 的粗格：**局部地面 = +0.000 m**（= 雷达自身高度！真值 −0.223 m）、台阶残差 **0.023~0.203 m**（帧间跳）、并把"参考坡度"钉成 **0.0°**（真值 ~2.9°） | 判据层新增**自击掩膜**（`self_mask_*`，**默认关**；只有 robot11 槽位的参数文件打开）：(a) 自身 collision 包络 113 个 AABB + (b) 近场死区 `r_xy ≤ 0.2416 m 且 z ≥ −0.2295 m`（上界 = 几何盲半径 0.3416 m − 10 cm） | **静止无假台阶**（近场 d<0.30 m 的 max 台阶 0.023~0.032 → **0.002~0.005 m**，低于 0.06 死区）；**真台阶仍限速**（2.6~2.8 m 的真特征照旧触发，见 §12.3） |
| ③ | 横幅说"参数没跟着切" | Phase 3 已经把 linefit 切成 `segmentation_sim_robot11.yaml`，但**横幅那一行写死了旧结论**（说的是"意图"不是"生效值"） | 横幅改成**运行时从真正递给节点的那份 YAML 里读出生效值**再打印；并补运行期 `ros2 param get` 证据 | **已修**：节点侧 `sensor_height=0.2595`、`gravity_aligned_frame=""`（§12.4） |

### 12.2 缺陷 ①：带 GUI 的代价（full vs decimated，**实测**）

口径：`world:=RMUC2026 mode:=mapping lio:=small_point_lio robot:=robot11 nav_rviz:=True`、车静止、
**虚拟屏 `Xvfb 3200x1200x24` + 软件 GL（llvmpipe / Mesa 23.2.1）**、RSS 每 1 s 采样一次、
窗口从"gzclient 与 rviz2 都起来"开始计 50~60 s。脚本：`tools/scripts/regress/robot11_gui_bench.sh`。
原始数据：`.tmp_robotslot/r11p4guiFULL{,2}/{rss.csv,shot-final.xwd,summary.txt}`、
`.tmp_robotslot/r11p4guiDEC{,2,3}/`。

| 指标 | `full`（上游原始 STL） | `decimated`（**默认**） | 差 |
|---|---|---|---|
| `<visual>` 三角形合计（12 个 mesh） | **2,821,320** | **9,700** | **−291×** |
| `<visual>` 文件字节合计 | **141.07 MB** | **0.486 MB** | −290× |
| 其中 `base_link` | 2,078,226 面 / 103.91 MB | 3,000 面 / 0.150 MB | −693× |
| `gzclient` 峰值 RSS | 474 MiB | 474 MiB | **0**（视觉档位**不动它**） |
| `gzserver` 峰值 RSS | 3,202 MiB（GUI）/ **3,112 MiB（无头对照，full）** | 3,202 MiB | 0（是 RMUC2026 世界的物理/网格，与视觉无关） |
| **`rviz2` 峰值 RSS** | **1,381 MiB**（另一跑 1,444） | **243 MiB**（两跑一致） | **−1,138 MiB（−82%）** |
| 三进程 RSS 合计 | ~5.06 GiB | ~3.92 GiB | −1.14 GiB |
| `gzclient` 活到窗口结束 | ✅ 50/50 s（60 s 那跑也满），**无死亡行** | ✅ 50/50 s（60 s 那跑也满），无死亡行 | — |
| `rviz2` 活到窗口结束 | ✅ | ✅ | — |
| 截图：**rviz2 窗口矩形内**非黑像素 / 颜色数 | **0.5768 / 1023** | **0.5768 / 1438** | 都真的在画（不是黑屏） |
| 截图：root 全屏非黑像素 | 0.3773 | 0.3773 | — |
| `/odom` 消息数（50 s 窗口内） | 90 | 115 | — |

**结论与诚实边界**：

1. **`robot11_visual:=decimated` 只换 `<visual>`**：`<collision>`（76 个 box + 云台细盒 + cylinder）、
   `<sensor>`、`<plugin>`、inertial 一个字节都没动 ⇒ **物理/感知/契约与全 mesh 版逐字节一致**。
   抽稀件是 Phase 1 用 VTK `vtkQuadricDecimation` 生成的**米制同原点**网格（`collision_assets.json`
   里逐个记了 `bbox_in/bbox_out` 与采样误差：`base_link` 出→参 max 26 mm / p99 18 mm）。
2. **本轮的 GUI 数字是"虚拟屏 + 软件 GL"上量的**：能测的是**进程存活、峰值 RSS、有没有画**；
   **不能**代表真 GPU 的渲染吞吐（llvmpipe 的帧率没有参考价值）。
3. ⚠️ **我们没能复现 `gzclient` 的 SIGKILL**：本机 31.9 GB 内存、空载可用 ~12~15 GB，两档都活满窗口、
   **都没有死亡行**。用户那次被杀最可能是**内存压力**（整栈 ~5 GiB：gzserver 3.1 + rviz2 1.4 + gzclient 0.5），
   而 `full` 档比 `decimated` 档**多 ~1.14 GiB**。⇒ 这是**推断**，不是实测（见 §12.7 第 1 项）。
   反过来说：**本轮也没有证据表明 `full` 档本身会让 GUI 崩**（两档都渲染正常）。
4. ⚠️ `gzclient` 的 RSS **不随视觉档位变**（464 vs 474 MiB，噪声内）⇒ "把 `base_link.STL` 换成抽稀件"
   **不会**降低 gzclient 的常驻内存；真正被降下来的是 **rviz2**（+1.2 GB → 243 MiB）。
   本次 GUI 里 gzclient 的内存主体是 RMUC2026 世界与 Gazebo 客户端本身。

### 12.3 缺陷 ②：自击掩膜（设计 + 前后实测）

#### 12.3.1 先更正一条 Phase 3 的归因（诚实更正）

§11.2 写的是"近点 100% 落在**云台** `l10/l11` 上，中位距离 0.14 m"。Phase 4 逐点复核后**要更正**：

* 实测近场点的**中位半径只有 0.021 m**（p95 = 0.048 m、max = 0.121 m），而不是 0.14 m；
* 它们**不在任何 `<collision>` 几何上**：把 113 个 collision AABB 逐个点数，"命中"的只有 **3 个点**；
* 它们是**射线插件把点重建成 `range·axis`** 的系统内移：射线实际从 `minDist·axis`（= 0.1 m）出发
  （`livox_points_plugin.cpp`：`start_point = minDist * axis`），而发布时按 `point = range * axis`
  算 ⇒ **每个点都朝传感器方向内移 0.1 m**。
* **独立证据（与掩膜无关）**：地面点的 z 随距离单调变化 —— r 0.25–0.35 → **−0.208**；
  r 0.35–0.50 → −0.219；r 0.5–1.0 → −0.237；r 2–4 → −0.253；几何真值 **−0.2595**。
  这正是"沿射线内移 0.1 m"应有的样子（近处仰角大 ⇒ 抬得高）。
  ⇒ 修那个插件会改变**所有模型**的点云（默认模型也移了 0.1 m）⇒ **不属本主题、且违反"默认逐字节不变"**，
  本轮只在**判据层**把这团近场剔掉。

#### 12.3.2 掩膜定义（两部分并集；默认两部分都关）

| 部分 | 定义 | 依据 |
|---|---|---|
| (a) 自身 collision 包络 | 本模型 12 个 link 的 `<collision>` 在 `livox_frame` 下的 **113 个 AABB**（逐个 ≥5 mm 膨胀；z 下限抬到地面以上 0.03 m） | Gazebo 的 ray sensor 走 ODE、**只与 `<collision>` 求交** ⇒ 真自击点必在某个 collision 体表面上 ⇒ 必在其 AABB 内；真障碍不可能在机器人自己的 collision 体内部 |
| (b) 近场死区 | `r_xy ≤ 0.2416 m` **且** `z ≥ −0.2295 m`（= 地面 +0.03 m） | 上界是**几何硬约束**：下俯 30° + FOV 下沿 7.22° ⇒ 最低射线的地面交点在 **0.3416 m**，即 **r < 0.3416 m 内不可能有地面回波**；取 0.3416 − 0.10 = **0.2416 m** 留余量 |

生成/验证工具：`tools/scripts/regress/robot11_self_mask.py`
（`--emit` 生成两份 YAML、`--check` 断言"YAML == 由模型重算的结果"、`--verify` 离线射线验证、
`--dump-fk` 打印零位 FK 供与运行期 TF 对照）。参数文件按 **robot 槽位**选
（`config/traversability_self_mask.yaml` = 默认**关**；`…_robot11.yaml` = 开 + 点表），
机制与 linefit 的 `segmentation_sim_<slot>.yaml` **完全同款**。

**离线验证（`--verify`，30000 条 MID-360 射线）**：

| 检查 | 结果 |
|---|---|
| 地面环（射线打到 z = 地面平面，7560 点，最近 **0.3457 m**）被掩点数 | **0** ✅ |
| 真 mesh 表面的 <0.75 m 命中被掩比例 | 0.2310 → **0.9992**（(b) 补上之后） |
| z 下限裁掉的 collision 形状数 / 最大裁掉量 | 8 个 / 0.035 m（= 轮子下沿，**故意的**地面保护） |

**在线验证（同一朵云、逐点判掩膜；3 帧实测）**：

| 帧 | 被掩点数 | 其中地面带(z<−0.20) | 走廊最近格 d | 该格 ground | 该格 台阶 |
|---|---|---|---|---|---|
| 无掩膜 | 0 | — | **0.00 m** | **+0.0000 m**（= 雷达自身高度） | 0.077~0.095 m |
| `robot11_self_mask_robot11.yaml` | 26.9~28.1% | **0** | **0.20 m** | **−0.2228 m**（真地面） | **0.006 m** |

#### 12.3.3 前后数字（同世界/同出生点/同参数；`mode:=mapping`、静止窗口）

`[slope_speed]` 与 `~/traversability_stats` 的**逐帧**统计（新增探针
`tools/scripts/regress/slope_speed_probe.py`；原始数据 `.tmp_robotslot/r11p4off/slope.log`（掩膜关）
与 `.tmp_robotslot/r11p4fin2/slope.log`（掩膜开））：

| 指标（静止） | BEFORE（掩膜关） | AFTER（掩膜开） |
|---|---|---|
| `self_masked` 点/帧（中位） | **0** | **3374**（≈ 全帧 11459 点的 29.4%） |
| 走廊最近格 `data_min_d` | **0.00 m**（37/37 帧） | **0.20 m**（46/46 帧） |
| 近场（d<0.30 m）max 台阶残差 | 0.023~0.032 m（该窗口）／日志里出现过 **0.128~0.203 m** | **0.002~0.005 m** |
| 近场（d<0.30 m）max 局部坡度 | 2.38~2.79° | 2.50~2.83° |
| `近坡`（参考坡度，取自最近一列） | **0.0°**（被自击格钉住） | **2.9~3.1°**（真地面坡度） |
| 限速落在地板 0.60 m/s 的帧数 | 12/129 条日志行出现 `d=0.00 m 台阶≈0.164 m`（其中 **9 条** `limit=0.60`）；另一窗口的逐帧统计 0/37 | **0/46**（最小 0.606 m/s） |
| 走廊 max 台阶（全走廊） | 0.104~0.189 m @ 2.6~2.8 m（**真场地特征**） | 0.075~0.191 m @ 2.6~2.8 m（同一真特征） |

**真台阶仍然限速（这一条是"没有把功能关掉"的证据）**：

* 静止窗口里 `why=step` 占 40/46 帧，触发格在 **2.3~2.8 m**、`台阶=0.180 m`（走廊 max 0.191 m）
  ⇒ 限速随"特征距离"变化：`d=2.34 m → 1.49 m/s`、`d=1.76 m → 1.33 m/s`、`d=1.14 m → 0.94 m/s`；
* 直行段（0.35 m/s × 8 s 向特征开过去）`why=slope_change 26 / slope 9 / step 1` 帧，
  限速 min 0.614 m/s、p50 1.13 m/s，走廊 max 台阶 p95 = 0.123 m ⇒ **接近特征时真的在减速**；
* 掩膜**只**剔"近场 + 自身 collision 体"里的点 ⇒ 2.6 m 处的特征一个点都没少（上表两列 max 台阶同源）。

⚠️ 一条**诚实更正**：日志里 `why=step d=0.00 m 台阶=0.164 m` 这个签名有**两个**来源 ——
(a) 上面那个近场自击格（Phase 4 已消除）；(b) **"已承诺特征"的距离前推**（`hold_decay_factor=1.0`
按"车以限速前进"扣减剩余距离）在**车其实没动**时会一路衰减到 0 ⇒ 即使特征在 2.6 m 也会被报成
`d=0.00`。本轮**没有**改 (b)（它不在本主题范围内、且改成"按真值里程计扣减"要动限速器的输入契约）
⇒ 见 §12.7 第 2 项。

### 12.4 缺陷 ③：接线（**运行期**证据，不是"我说配了"）

**修的是什么**：Phase 3 已经把 linefit 切到槽位文件（`_RobotSlotFile` 的 `slot_map`），
但 launch 横幅里那行字**写死了 Phase 2 的旧结论**（"linefit 参数**没有**跟着切（sensor_height 仍是 0.226…）"）
⇒ 用户照它排查就查错方向。根因是"横幅说的是**意图**，不是**生效值**"。

**修法**：新增 `_YamlKeysReadout` / `_SelfMaskReadout` 两个 Substitution —— 横幅在**运行时**把
**真正递给节点的那份 YAML** 读出来、把键值打进日志 ⇒ 横幅与"节点实际读到的文件"不可能再分叉。

**运行期证据（`robot:=robot11` + `lio:=small_point_lio`，一次跑里同时取）**：

| 证据 | 值 | 取自 |
|---|---|---|
| launch 横幅（运行时读文件） | `sensor_height=0.2595, gravity_aligned_frame="", input_topic="/livox/lidar/pointcloud", ground_output_topic="segmentation/ground" ← …/config/segmentation_sim_robot11.yaml` | `launch.log` 第 3 行 |
| 同上，掩膜那一路 | `self_mask_enable=true, 近场死区 r<=0.2416 m（z>=-0.2295）, 113 个 collision AABB ← …/traversability_self_mask_robot11.yaml` | 同上 |
| **节点侧**参数 | `ros2 param get /ground_segmentation sensor_height` → **`Double value is: 0.2595`** | `run_robot_model_probe.sh` |
| 同上 | `gravity_aligned_frame` → **`String value is:`**（空串，符合设计） | 同上 |
| 同上 | `self_mask_enable` → **`Boolean value is: True`**；`self_mask_boxes` → **678 个数**（113 盒） | 同上 |
| 节点自己的日志 | `自击掩膜（self_mask）：开：113 个 collision AABB（传感器系）…` | `launch.log` |
| 上游 linefit 的**输出**没变差 | `/segmentation/ground` **6641 点/帧**（中位；Phase 3 是 5637） | `probe.json` |

**`lio_tf_adapter` 的杆臂/旋转补偿在本配置里到底应不应当生效 —— 实测判定**：

* `lio:=small_point_lio` 时 `lio_tf_adapter` **按设计不启动**（同一时刻只能有一个 `odom→base_link`
  发布者）。运行期证据：`ros2 param get /lio_tf_adapter xyz` → **`Node not found`**；
  `ros2 node list` 里没有该节点。
* 这个配置下杆臂由 **LIO 自己**做：`small_point_lio_node.cpp` 用
  `lookupTransform(lidar_frame,"base_link")` 求 `T_bl←lidar`，再算
  `T_odom→base_link = T_bl←lidar⁻¹ · T_odom→lidar · T_bl←lidar`（相似变换 ⇒ **平移与旋转一起**处理）。
* **旋转补偿在本槽位不需要**：Phase 3 把 `body_to_livox` 的 rpy 改成 **0**（帧重力对齐、倾角搬进
  `<tilt_rpy>`）。运行期证据（`robot_state_publisher` 发的 TF，独立于任何离线生成器）：
  `ros2 run tf2_ros tf2_echo base_link livox_frame` → `Translation: [0.001, 0.131, 0.157]`、
  `RPY (radian) [0.000, -0.000, 0.000]` ⇒ **纯平移**。
* 离线生成器的零位 FK 与运行期 TF 一致（`robot11_self_mask.py --dump-fk`：
  `livox_frame livox_xyz=[0.000000 0.000000 0.000000]`、`l10 = [−0.000562, −0.130916, +0.055972]`）。
* ⇒ 所以横幅里那句"`lio_tf_adapter` 杆臂 = T_imu←base_link（含 30° 旋转补偿）"在本配置下是**误导**，
  已改成说明"本配置不启动该节点 + LIO 自己做相似变换"。

### 12.5 验收表（无头 + GUI）

口径：`world:=RMUC2026 mode:=mapping lio:=small_point_lio robot:=robot11 map_autocontinue:=False`、
静止窗口；无头 = `unset DISPLAY` + `HOME`/`ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI` 全隔离。
原始数据：`.tmp_robotslot/r11p4fin2/`（无头，掩膜开）、`.tmp_robotslot/r11p4base/`（无头，掩膜关）、
`.tmp_robotslot/r11p4guiDEC3/`、`r11p4guiFULL2/`（GUI）。

| 检查 | 无头（掩膜开，默认 decimated） | 无头（掩膜关，对照） | GUI（decimated + rviz2） | GUI（full + rviz2） |
|---|---|---|---|---|
| `/odom` 消息数 | **53**（53 帧）/ 静止窗口 | 67 | 115（60 s 窗口） | 103 |
| `/livox/lidar/pointcloud` | 10.0 Hz，**11405 点/帧** | 10.0 Hz，11188 点/帧 | 10 Hz | 10 Hz |
| `/segmentation/ground` 点/帧 | **6641** | 6644 | — | — |
| `/segmentation/obstacle` 点/帧 | 4766 | 4544 | — | — |
| `/scan` 有效波束/帧（中位） | 514 | 523 | — | — |
| `/scan` **>4 m** 波束/帧 | **304.2** | 283.6 | — | — |
| `/scan` **<1 m** 波束/帧 | **218.7** | 259.8 | — | — |
| 自击 `r<0.12 m`（**原始云点**口径） | **29.0%**（**故意不变**：近场回波是物理真实的；掩膜只作用于判据） | 28.6% | — | — |
| 其中**被掩膜剔出判据**的比例 | **29.4%（3374 点/帧）** | 0% | — | — |
| 静止时的假台阶（近场 d<0.30 m max 台阶） | **0.002~0.005 m** | 0.023~0.032 m（日志里出现过 0.128~0.203） | — | — |
| 静止时限速落在地板 0.60 的帧数 | **0/46**（min 0.606） | 9/129 条日志行 | — | — |
| RTF（`/clock` 口径） | **0.32~0.38**（本机与他任务并发，波动大；Phase 3 无头是 0.46） | 0.416 | 见下 | 见下 |
| `gzclient` 峰值 RSS / 存活 | 不启动（无 DISPLAY ⇒ 起不来） | 同 | **474 MiB / 活满 50 s** | 464 MiB / 活满 60 s |
| `gzserver` 峰值 RSS | **3112 MiB**（full 对照，无 DISPLAY） | — | 3202 MiB | 3202 MiB |
| `rviz2` 峰值 RSS / 存活 / 画面 | 不启动 | 不启动 | **243 MiB / 活满 / 窗口内非黑 0.5768** | **1381 MiB / 活满 / 窗口内非黑 0.5768** |
| `/cmd_vel_chassis` 发布者 | **1**（订阅者 1 = planar_move 插件） | 1 | 1 | 1 |
| `/segmentation/obstacle` 发布者 | **1** | 1 | 1 | 1 |
| `/segmentation/ground` 发布者 | **1** | 1 | 1 | 1 |
| `/map` 发布者 | **1** | 1 | 1 | 1 |
| `/odom` 发布者 | **1**（`small_point_lio`；`lio_tf_adapter` 未启动） | 1 | 1 | 1 |
| `/livox/imu` 发布者 | **1** | 1 | 1 | 1 |
| GPU 帧率 | — | — | **不可测**（llvmpipe 软件 GL，不代表真 GPU） | 同 |

**判定：`robot:=robot11` 现在 (a) 可以带 GUI 交互建图**（在"虚拟屏 + 软件 GL"上两档都活满窗口、
rviz2 有像素；`decimated` 把 rviz2 内存从 1381 → 243 MiB）。**未复现**用户那次的 SIGKILL，
所以"用户机器上还会不会被杀"仍属**未验证**（§12.7 第 1 项）。

### 12.6 作用域与回退（默认模型/其它槽位逐字节不变）

* (a)(b) 两处偏离都在 **robot 槽位作用域**内：`SelfMask::enable{false}` 是 C++ 兜底默认，
  默认槽位读到的 `traversability_self_mask.yaml` 也写死 `false` ⇒ `buildGrid()` 里那一行
  `if (self_mask_.contains(...))` 恒假（**没有**掩膜时的行为与 Phase 3 逐字节相同）。
* `robot11_visual` 的默认 `decimated` **只对 `robot:=robot11` 生效**（其它槽位的 xacro 根本不引用
  `visual_dir`/`visual_ext` 这两个 property；不传该 arg 时 xacro 默认 `false` = 上游原始 STL）。
* 判据层只**新增**了参数（`self_mask_*`），`traversability_criteria.yaml` 一个字节没改
  ⇒ `tools/scripts/regress/check_traversability_criteria.py` 仍然通过。
* **默认模型的回归证据**（`robot` 槽位留空，同世界同 LIO；`.tmp_robotslot/r11p4def/`）：
  运行期 `ros2 param get /ground_segmentation self_mask_enable` → **`Boolean value is: False`**、
  `self_mask_boxes` **空**、`sensor_height` 仍是 **0.226**；实测 **RTF 0.703、点/帧 5279、
  地面 2804 点/帧、自击 r<0.12 = 0.0%** —— 与 Phase 3 的默认模型对照（6256 / 2739 / 0%）在
  跑间波动内一致 ⇒ "新增一个默认关的掩膜"**没有**改变默认模型的任何行为。
* 契约不变：`/cmd_vel_chassis` / `/segmentation/*` / `map→odom` 的发布者个数全部实测 = 1；
  掩膜**不改**任何点云的标签（自击点仍在 `/segmentation/obstacle` 里，物理真实）。

| 想退掉什么 | 怎么做 |
|---|---|
| 退回"没有自击掩膜" | `config/traversability_self_mask_robot11.yaml` 的 `self_mask_enable: false`（或 launch 里撤掉 `self_mask_params` 那一路） |
| 退回"视觉用上游原始 STL" | `robot:=robot11 robot11_visual:=full`（= Phase 1~3 的逐字节行为） |
| 退回 Phase 2 的"实心 4 box"碰撞 | 见 §11.7 的同名行 |
| 整套 robot11 | 见 §11.7 最后一行 |

### 12.7 未验证清单（Phase 4 新增，诚实清单）

1. **"用户机器上 `gzclient` 为什么被 SIGKILL"没有复现**：本机（31.9 GB 内存）两档都活满窗口。
   本轮量到的是**内存代价差**（rviz2 1444 → 243 MiB），据此**推断**是内存压力所致 ——
   这是推断，不是实测。要证实需要在同类内存受限环境里跑一次 `full` 并看 OOM 记录。
   → **该问题已另案结清**（2026-10-07 追加）：见 `docs/gazebo_gui_troubleshooting.md` ——
   用户机内核日志（本次开机完整保留）与 cgroup `memory.events`（`oom_kill=0`）都表明
   那两次 `exit code -9` **不是 OOM**；**发送者无法指认**（无 `auditd`）—— ⭐ **用户 2026-10-10 回忆：
   是用户自己手动 `pkill -9` 的，因为当时"太卡了或者直接卡住了"**（另一个候选才是本仓 bench 脚本的
   全机 `pkill -9 -x gzclient`）；
   而"GUI 卡住不出来"另有真因：`robot11` 的 `model://` mesh 本地解析不到 ⇒
   回落到在线模型库并**同步阻塞**（用户 19:50 日志实测 48.03 s，且是被 Ctrl-C 打断的）。
2. **限速器的"承诺距离前推"在车不动时会衰减到 0**（`speed_limit_hold_decay_factor=1.0` 按"以限速前进"
   扣减剩余距离）⇒ 即使特征还在 2.6 m，日志也会出现 `d=0.00`。本轮**没有**改它（不在本主题范围，
   且"按真值里程计扣减"要动限速器的输入契约）⇒ `d=0.00` 这个签名以后仍有第二种来源。
3. **射线插件 `point = range * axis`（漏了 `start = minDist*axis`）的系统内移 0.1 m 没有修**：
   它是**共享代码**、改了会改变所有模型的点云（默认模型也移了 0.1 m）⇒ 不属本主题。
   本轮只是在判据层把这团近场剔掉。**未做**：量化"内移 0.1 m"对 LIO/代价图/`/scan` 的影响。
4. **`/scan` 的 <1 m 波束仍是 218.7 条/帧**（自击点仍在 obstacle 里）：本地代价图会不会被这圈
   "自障碍"填满、nav2 是否因此规划失败 —— **未验证**（§11.8 第 1 项"车 90 s 没动"的根因仍未定，
   本轮给出一个新候选：**局部代价图里恒有一圈 r≈0.02~0.09 m 的自身障碍**）。
5. **GUI 的渲染吞吐没有测**（llvmpipe 软件 GL）⇒ "带 GUI 时 RTF 是多少"在真 GPU 上仍未知；
   本轮只测了进程存活 / 峰值 RSS / 有没有画。
6. **`robot11_visual:=full` 下 rviz2 逐窗口像素只测了一次**（非黑 0.5769 / 1430 色），
   没有做"长时间（>5 min）连续建图时的内存增长"曲线。
7. **有头侧的默认值改了**（`robot11_visual` 默认 `decimated`）⇒ §11 里"visual = 原始 mesh、视觉零损失"
   这句话现在只在 `robot11_visual:=full` 下成立（已在 §12.2 写清）。
8. **掩膜盒子表是"生成物"**：模型改了（碰撞/关节/抬高）必须重跑
   `robot11_self_mask.py --emit`，否则 `--check` 会报错；本轮**没有**把它接进 CI/构建（只接了自检脚本）。

---

## 13. Phase 5：把 `<visual>` 真正做成"仿真视觉件"（QEM 抽稀 + 量化验证 + 三档代价实测）（2026-10-07）

> 状态：**新增**。触发 = 用户原话「还是卡住了，上网查资料，机器人模型放不进去仿真有啥意义」。
> Phase 1~3 修的是**碰撞（物理）**，Phase 4 修的是**判据/接线/GUI 档位开关**，
> 而**视觉件本身一直是上游 CAD 原始 STL**（整车 2,821,320 面 / 141.07 MB，其中 `base_link.STL`
> 一项 2,078,226 面 / 103.91 MB）。本相位把视觉件按公开的标准管线做成**真正的仿真 LOD**，
> 并把"几何没走样"量化成可复算的清单。
> 默认模型与所有 launch/config 默认值**仍未改**；`robot:=robot11` 之外逐字节不变。

### 13.1 一句话结论

| 问题 | 结论 | 证据 |
|---|---|---|
| CAD 原始网格能不能直接进仿真？ | **视觉可以（但有代价），碰撞绝对不行** | §13.2.1 的官方原文 + §9.3 本仓实测（有接触时 RTF 0.200 / 峰值内存 759 MB） |
| 那视觉呢？ | 原始 2,821,320 面是"给人看/给机床用"的档，不是"每帧渲染"的档；标准做法是**按预算抽稀 + 量化验证** | §13.2.2、§13.3 |
| 我做了什么 | QEM 二次误差边折叠抽稀：**2,821,320 → 163,995 面**（5.8%），磁盘 141.07 → 8.20 MB；逐 mesh 量化包围盒差 / 单向表面误差 / 三视剪影 IoU / 边界边 / 雷达近场自击，全部入库 | §13.3、`inventory/visual_decimation.json` |
| 开关 | `robot11_visual:=decimated`（**默认**）\| `full`；`decimated` 现在指向**专用视觉 LOD**（`meshes/decimated/`），不再是 Phase 4 那份碰撞档抽稀件 | §13.5 |
| GUI 到底行不行 | **本机（Xvfb+软件 GL）实测：两档都能起来、都活满窗口、都在画**；`gzclient` 473 MiB / `gzserver` 3202 MiB **两档完全相同**；`rviz2` 314 vs 322 MiB（差 8 MiB，噪声内） | §13.4（**实测**） |
| ⚠️ 那 GUI 的优势在哪 | **本相位没有测出"抽稀能省 GUI 内存"**（与 Phase 4 §12.2 的 rviz2 1,381↔243 MiB 结论**不一致**，见 §13.4.3 的诚实记录）。抽稀确定省下的是**网格解析/上传/绘制的工作量**（三角形少 17.2×）与**磁盘/网络/内存中的网格数据本身** | §13.4.3 |
| 还发现什么 | `/scan` 里**恒有 428 条/帧 <1 m 的障碍波束**（自击点仍在 obstacle 输出里，掩膜只管判据）⇒ 局部代价图可能被"自身障碍"填满 —— **未修**，见 §13.9 | §13.9（**实测**） |

### 13.2 上网查到的标准做法（**带 URL**）

> 抓取方式：本沙箱里 `web_fetch` 对部分主机返回 "non-public IP"，所以下面**全部用 `curl` 直取**；
> 取不到的会明确标注。原始抓取物落在 `.tmp_robot11/research/`。

#### 13.2.1 碰撞：为什么原始 CAD 网格不能当 `<collision>`

* **Gazebo Classic 官方教程「Make a model」**（<https://classic.gazebosim.org/tutorials?tut=build_model>，已取到）原文：
  > "Collision: A collision element encapsulates a geometry that is used for collision checking.
  > This can be a simple shape (**which is preferred**), or a triangle mesh
  > (**which consumes greater resources**). **A link may contain many collision elements.**"
  同页还有一句与本槽位直接相关：
  > "Try to reduce the number of links in your models in order to improve performance and stability."
  ⇒ **"简单形状优先"、"一个 link 可以有多个 collision"** 这两条不是我们发明的，是官方口径。

* **SDF 规范（`collision.sdf`，sdformat 仓库 1.9 分支）**
  （<https://github.com/gazebosim/sdformat/blob/sdf12/sdf/1.9/collision.sdf>，已取到）原文：
  > "The collision properties of a link. **Note that this can be different from the visual properties
  > of a link, for example, simpler collision models are often used to reduce computation time.**"
  ⇒ **`<collision>` 与 `<visual>` 用不同几何是规范明文鼓励的**。

* **SDF 1.9 的 `<geometry>`/`<mesh>`**（<https://github.com/gazebosim/sdformat/blob/sdf12/sdf/1.9/geometry.sdf>、
  <…/mesh_shape.sdf>，均已取到）：`<geometry>` 可选 box/capsule/cylinder/ellipsoid/heightmap/image/
  **mesh**/plane/polyline/sphere；`<mesh>` 在 **1.9 里只有 `uri` / `submesh` / `scale` 三个子元素**。
  ⚠️ **重要且容易踩**：`<convex_decomposition>`（V-HACD）**不在 SDF 1.9 里** —— 它是 1.10+（gz-sim 时代）
  才有的元素。**Gazebo Classic 11 用 SDF 1.9 ⇒ 它没有内置的 V-HACD 通道**；
  在 Classic 上做凸分解只能"离线算好、再写成多个 `<collision>` 的凸网格/基本体"。
  （本仓的 76 个 box 就是这条路的自制版本。）

* **ODE 官方手册 §10.7.6 Triangle Mesh Class**（<https://ode.org/ode-latest-userguide.html>，已取到）：
  > "Any triangle 'soup' can be represented… Triangle meshes can interact with spheres, boxes,
  > rays and other triangle meshes. **It works well for relatively large triangles.**"
  > "Trimesh/Trimesh collisions, perform quite well, but there are three minor caveats:
  > The stepsize you use will, in general, have to be reduced for accurate collision resolution.
  > **Non-convex shape collision is much more dependent on the collision geometry than primitive
  > collisions.** Further, the local contact geometry will change more rapidly (and in a more complex
  > fashion) for non-convex polytopes than it does for simple, convex polytopes such as spheres and cubes."
  ⇒ 官方措辞是"相对**大**三角形才好用"+"非凸接触比基本体敏感得多"+"步长要更小"；
  本仓量到的 **RTF 0.200 / 内存 +587 MB** 就是这三条在"208 万个小三角形"上的具体后果（§9.3）。

* **Gazebo Classic issue #514**（<https://github.com/gazebosim/gazebo-classic/issues/514>，已取到）：
  Bullet 引擎里 "**all meshes are loaded as `btConvexTriangleMeshShape`**"，"We need to support concave
  shapes" ⇒ 换物理引擎也不能"免费"拿到凹网格碰撞。

* **playerstage-gazebo 邮件列表「Trimesh collision」(2008)**
  （<https://sourceforge.net/p/playerstage/mailman/playerstage-gazebo/thread/60f134360806251625k2945846amb1b53718dd226e53%40mail.gmail.com/>，已取到）：
  > "It seems that gazebo trimesh collision only works with ode 0.8 built with the OPCODE library.
  > Also, unlike trimesh **visuals**, trimesh **collision** meshes are not automatically resized and
  > therefore should not need to be scaled in the world file."
  ⇒ 两条历史事实：网格碰撞要走额外的加速结构；**视觉网格会自动缩放、碰撞网格不会**（单位坑的来源之一）。

* **`ros-mesh-preprocessor`**（<https://github.com/ookkshirsagar/ros-mesh-preprocessor>，已取到 README）：
  一个专门做这件事的 ROS 2 工具包 —— "**QEM decimation, URDF visual/collision export, and
  Hausdorff-validated quality reports**"。⇒ "抽稀 + **用 Hausdorff 距离做质量报告**"是社区成型的做法，
  本仓的 `--err-*`（单向表面误差 p99/max）就是同一件事的自制版。

#### 13.2.2 视觉：预算、抽稀工具、"够用"的定义

* **《Blender Workflow for Cleaning CAD Meshes for ROS 2 / Gazebo》**
  （<https://mohammadrobot.github.io/ROS/Blender-CAD-Meshes/>，已取到）给出的**预算表**：
  > Mesh Budgets — **Visual: < 50k** / **Collision: < 5k**
  以及流程："Remove duplicate vertices (M → By Distance)" → "Fix normals（面朝向：蓝=正确、红=翻转）"
  → "**Triangulate + Decimate（Ratio 0.2–0.4）**：1,200,000 faces → 40,000 faces"
  → "Apply object transforms（Ctrl+A）" → "视觉导 DAE/OBJ、碰撞导 STL"。
  原文还点名两个坑：
  > "**Flipped normals** are a common cause of missing surfaces in simulation."
  > "**Slow simulation** ← collision mesh too detailed / **Unstable collisions** ← tiny CAD details in collision mesh
  > （remove screws/threads/fillets from collision mesh）"

* **《메쉬 경량화와 collision mesh 분리 기준》(ros2_gazebo_setup/docs/mesh-optimization.md)**
  （<https://raw.githubusercontent.com/Jinsun-Lee/ros2_gazebo_setup/refs/heads/master/docs/mesh-optimization.md>，
  已取到）给出**另一种口径与更完整的理由**：
  > · visual 메쉬는 매 프레임 렌더링된다 … 링크당 **약 100k 삼각형 이하**를 유지
  > · collision 메쉬는 매 물리 틱마다 질의된다 … 링크당 **약 1k 삼각형 이하**
  > · 실제 형상이 프리미티브에 가까우면 **프리미티브 도형**(`<box>`, `<cylinder>`, `<sphere>`)을 우선 사용
  > · 오목한 형상은 여러 개의 convex collision으로 분해한다 (**V-HACD** 또는 수동 분리)
  > · 경량화 도구: **Blender Decimate**, **MeshLab Quadric Edge Collapse Decimation**, **V-HACD**
  > · 验证清单："collision 是否覆盖 visual 全部范围 / spawn 后 **RTF ≥ 1.0** / 法线有没有翻"
  ⇒ 两篇独立来源的交集 = **每 link 视觉 ≤ 50k（严的那档）**，本相位取严的那档（≤50k/link、整车 ≤200k）。

* **抽稀算法本身**：
  * **MeshLab / PyMeshLab** 的 `Simplification: Quadric Edge Collapse Decimation`
    （<https://github.com/cnr-isti-vclab/PyMeshLab/blob/master/docs/filter_list.rst>，已取到）：
    > "Simplify a mesh using a **Quadric based Edge Collapse Strategy**; better than clustering but slower"
    参数（逐个取到原文）：`targetfacenum` / `targetperc` / `qualitythr`（"Quality threshold for penalizing
    bad shaped faces"）/ `preserveboundary` / `boundaryweight` / `preservenormal`（"Try to avoid face
    flipping effects"）/ `preservetopology`（"Avoid all the collapses that should cause a topology change
    (like closing holes, squeezing handles)"）/ `optimalplacement`（"Each collapsed vertex is placed in
    the position minimizing the quadric error"）/ `planarquadric`。
  * **Blender Decimate（Collapse）**（<https://docs.blender.org/manual/en/latest/modeling/modifiers/generate/decimate.html>，已取到）：
    > "Merges vertices together progressively, **taking the shape of the mesh into account**.
    > Ratio: The ratio of faces to keep after decimation."
  * **Open3D**：`open3d.geometry.TriangleMesh.simplify_quadric_decimation`
    （<https://www.open3d.org/docs/latest/python_api/open3d.t.geometry.TriangleMesh.html>，已取到）。
  * **trimesh**：`Trimesh.simplify_quadric_decimation`
    （<https://trimesh.org/trimesh.html>）—— ⚠️ **本机不可用**：它依赖 `fast_simplification`，
    而该包未安装（实测 `ModuleNotFoundError`）。`trimesh.org/trimesh.simplification.html` 这个
    具体页面 **404**（未能取到）。
  * **V-HACD**（<https://github.com/kmammou/v-hacd>）—— 本相位**没用到**（我们抽稀的是视觉件，
    碰撞件 Phase 1/3 已经用基本体解决）。

* **"够用"的判据**（本相位自定，但每条都对应上面某条来源）：
  1. **包围盒逐轴差 ≤ 1 mm**（单位/原点/尺寸不走样 —— 对应"Wrong robot size in simulation ← unapplied scale"）；
  2. **单向表面误差 p99 ≤ 3 mm、max ≤ 12 mm**（对应 ros-mesh-preprocessor 的 Hausdorff 报告思路）；
  3. **三视剪影 IoU ≥ 0.97**（轮廓没变 —— "视觉够用"的可量化代理）；
  4. **边界边不增长**（抽稀不该抽出新洞 —— 对应 `preservetopology` 的意图）；
  5. **雷达近场自击比例不变**（≤5 个百分点）—— 这条是**本槽位专属**的：这台车的雷达装在底盘凹槽里，
     抽稀如果把凹槽"封上"，视觉上就会看到一块盖住雷达的板（详见 §13.3.4）。
  ⇒ **不要求**"水密/流形"：视觉件只要看起来对；水密是对**碰撞/惯量**的要求（§13.2.3）。

#### 13.2.3 网格格式与质量坑

* **格式与单位**：SDF 1.9 的 `<mesh><scale>` 默认 `1 1 1`
  （`mesh_shape.sdf`，已取到）⇒ 缩放**不是**自动的；STL 本身不携带单位，Gazebo 按"米"读。
  ⇒ 从 mm/inch 导出的 CAD 通常要写 `scale="0.001"`；本仓 12 个 STL 实测**原始值就是米**
  （判据：原始包围盒最大跨度 0.6 < 20，见 §9.2），所以**一个 `scale` 都不写**（`check_robot11_visual_slot.py`
  的 D1 断言"两档都没有 `<mesh scale=...>`"）。
  格式本身：STL 无材质、二进制 STL 最小；DAE/OBJ 能带材质但要多解析一层
  （ros2_gazebo_setup 那篇建议"视觉用 DAE/OBJ 以保材质、碰撞用 binary STL"）。
* **非水密 / 非流形**：本仓 12 个 STL **没有一个水密**（`base_link` 有 53,378 条非流形边、
  19,432 条边界边、**2,733 个互不相连的实体**，见 §9.2）。它坏掉的是**体积/惯量**这条线
  （任何"从网格算质量/惯量"的工具都会给出无意义的数）⇒ 本槽位的 `<inertial>` **全部来自上游 URDF 的字面值**，
  不从网格算。它**不**影响渲染（视觉件允许有洞/有内部件），也**不**影响本仓的碰撞（碰撞是 box/cylinder）。
* **`<inertial>` 缺失 ⇒ link 被丢掉**：sdformat 的 URDF→SDF 转换 PR #1238
  （<https://github.com/gazebosim/sdformat/pull/1238>，已取到 diff）原文：
  > "Links without an `<inertial>` block will be considered to have **zero mass**."
  并会对"零质量 link"发 `LINK_INERTIA_INVALID` 警告（"no `<inertial>` block defined. " /
  "a mass value of less than or equal to zero. " / "Please ensure this link has a valid mass to
  prevent any …"）。urdfdom 的解析器侧也有对应的硬报错
  （<https://github.com/ros/urdfdom/blob/master/urdf_parser/src/link.cpp>，已取到：
  "Inertial element must have a mass element" / "Inertial element must have inertia element" /
  "Inertial: inertia element missing …"）。同一主题在 gz-sim 也有 issue
  （<https://github.com/gazebosim/gz-sim/issues/2815>）。
  ⇒ 这正是 §2/§10 里"hzmirm 那份 URDF 直接换上去 spawn 出来是空模型"的机制。

#### 13.2.4 GUI 代价（`gzclient` 为什么重）

* **Gazebo 核心开发者 Carlos Agüero 的 GUI profiling 报告**
  （<https://caguero.github.io/gz-profiling/2026-07-03/>，已取到）：
  > "**The GUI is render-thread-bound**; per-thread splits separate the OgreNext render thread from
  > the Qt main thread."；加载阶段的主导开销是 "**jetty mesh + material construction**"；
  > `3k_shapes_static` 世界下 GUI 平均占 **1.05 核**、其中渲染线程 **91%**。
  ⇒ GUI 进程的开销主要在**渲染线程**与**网格/材质构建**上 —— 与"视觉三角形数/网格数"直接相关。
  （注意：这份报告是**新版 Gazebo（gz-sim）**的，渲染后端是 OgreNext；Gazebo Classic 用 OGRE 1.x，
  同族但不同版本，所以我们只把它当"GUI 是渲染线程瓶颈 + 网格构建是加载期大头"的**方向性**引用。）
* **Gazebo Classic issue #805 "GUI topic visualization slow"**
  （<https://github.com/gazebosim/gazebo-classic/issues/805>，已取到）：
  > "Viewing a topic that produces large quantities of data causes **gzclient to be unresponsive**."
* **Gazebo Classic issue #777 "gazebo slow at loading multiple object instances"**
  （<https://github.com/gazebosim/gazebo-classic/issues/777>，已取到）—— 同类"实例/网格一多就慢"。
* **降代价的手段**（社区共识 + 本仓做法）：
  1. **抽稀视觉件**（本相位做的）；
  2. **不跑 gzclient**：`gzserver` 无头 + 只在需要时开 GUI（本仓所有 regress 跑法本来就是这样）；
  3. **不同时开 gzclient 与 rviz2**（本仓的 `nav_rviz/lio_rviz` 开关就是为这个存在的）；
  4. 关阴影（本仓的 bench 世界里一直是 `<shadows>false</shadows>`）；
  5. ⚠️ 我们**没有**找到"Gazebo Classic 用 OGRE 选项进一步降内存"的权威文档 ⇒ 不写进结论（见 §13.10）。

#### 13.2.5 其它 RoboMaster / ROS 队伍怎么做

* **RMOSS（RoboMaster 官方开源软件栈）的 `rmoss_gazebo`**
  （<https://github.com/robomaster-oss/rmoss_gazebo>、索引页 <https://index.rosdabbler.com/r/rmoss_gazebo/>，已取到）：
  它把"Gazebo 插件 + 机器人模型资源 + 场地模型"拆成**多个包**，其中资源单独成库
  （README："`rmoss_gz_resources` 主要包含资源文件，**体积较大，单独成库**"），
  而且**基于新版 Gazebo（Fortress）而不是 Classic**。⇒ 两条可借鉴：**重资产单独成包**、
  **新工程优先选新 Gazebo**（我们被本仓的 Classic 栈绑住，只能在 Classic 内做）。
* **湖北工业大学力创战队的哨兵/场地开源 `Hbut_LC_sentry`**
  （<https://github.com/HBUTHUANGPX/Hbut_LC_sentry>，已取到 README）——**与我们的问题最贴近的一条中文实践**：
  > "对 2023RMUC 地图 solidworks 文件进行开源…目标主要是为了在仿真中实现 Lidar-SLAM 和 Lidar 定位导航"
  > "**为了保证后续在仿真中不卡，秉持着保证最明显特征的原则，并没有作图出红蓝装饰带、视觉定位标签。
  > 请后面的贡献者务必遵守这条要求，尽可能的避免非必要特征。**"
  > 工具链："使用 solidworks 开源插件 **SolidWorks to URDF Exporter** 获得和 ros 兼容的包"
  ⇒ **"为了保证仿真不卡，CAD 里只保留最明显的特征、删掉装饰性细节"** —— 这正是"CAD 精简"这条标准做法
  在 RM 圈子里的口碑版本，也解释了为什么 CAD 原始件不能直接进仿真。
  （同一份 README 还诚实地写了"官方的仿真器应该是没有使用到 gazebo 的物理特性的"、
  "小车在 gazebo 中是上不了坡的" —— 说明这类项目普遍存在"模型/物理没对齐"的问题。）
* **SolidWorks→URDF 官方插件**：<https://github.com/ros/solidworks_urdf_exporter>
  （当前跳转到 `ros/solidworks_urdf_exporter`；它导出的正是"每个 link 一个 STL"的形态，
  也就是本仓 `robot11` 的来路）。

#### 13.2.6 够不到的来源（诚实清单）

| 想要的东西 | 结果 |
|---|---|
| `sdformat.org/spec?ver=1.9&elem=mesh` | 页面是 JS 应用，`curl` 只拿到同一个空壳（13 KB，两次请求字节数完全相同）⇒ 改从 **sdformat 仓库的原始 `sdf/*.sdf` 规范文件**取（已取到，见上） |
| `trimesh.org/trimesh.simplification.html` | **404**（改用 `trimesh.org/trimesh.html` 的类文档） |
| "Gazebo Classic OGRE 渲染选项降内存"的权威文档 | **没找到** ⇒ 不写结论 |
| Gazebo Classic 的"每帧三角形预算"官方数字 | **没有官方数字**（官方只说 "consumes greater resources"）⇒ 预算采用两篇第三方实践的交集（§13.2.2） |
| `meshlabstuff.blogspot.com` 的 QECD 介绍页 | **404**（改用 PyMeshLab 的 `filter_list.rst`，已取到且更权威） |

### 13.3 Phase 5 应用的流水线（工具 / 命令 / 预算 / 容差 / 逐 mesh 结果）

#### 13.3.1 工具与算法（**实际跑的那个**，不是"推荐的"）

| 项 | 值 |
|---|---|
| 工具 | `tools/scripts/regress/robot11_decimate_visuals.py`（生成 + 验证 + 出清单，一条命令） |
| 算法 | **二次误差边折叠（quadric error metric edge collapse, Garland–Heckbert 族）** = `vtkCleanPolyData`（只按坐标合并重复点，`tolerance=0`，不改几何）→ `vtkQuadricDecimation`（`TargetReduction`，`VolumePreservation=false`）→ `vtkTriangleFilter` |
| 生成命令 | `python3 tools/scripts/regress/robot11_decimate_visuals.py`（写 `meshes/decimated/` + `inventory/visual_decimation.json`） |
| 复核命令 | `python3 tools/scripts/regress/robot11_decimate_visuals.py --check` 或 `python3 tools/scripts/regress/robot11_decimate_visuals.py --only l12 --calibrate` |
| 输入 | `meshes/<name>.STL`（**只读**，一个字节都没改；sha256 记在清单里） |
| 输出 | `meshes/decimated/<name>.stl`（binary STL，**12 个 / 8.20 MB**） |

**为什么不用别的（都是本机实测，不是听说）**：

| 候选 | 本机实测 | 结论 |
|---|---|---|
| MeshLab `meshlabserver` 的 `Simplification: Quadric Edge Collapse Decimation` | `meshlabserver 2020.09` 在**无 GL** 的机器上直接抛 `MLException: GLEW initialization failed: Missing GL version`；加 `LIBGL_ALWAYS_SOFTWARE=1`、`GALLIUM_DRIVER=llvmpipe` 也一样（连 `-d` 列过滤器都还没走到就死在初始化） | **不能用**（工具在，但没有可用的 GL 上下文） |
| `trimesh.Trimesh.simplify_quadric_decimation` | `ModuleNotFoundError: No module named 'fast_simplification'` | **不能用**（缺依赖，且不打算为一个一次性任务装包） |
| `open3d.geometry.TriangleMesh.simplify_quadric_decimation` | `open3d` 未安装 | **不能用** |
| `pymeshlab` | 未安装 | **不能用** |
| Blender Decimate | 未安装 | **不能用** |
| **VTK `vtkQuadricDecimation`（pyvista 0.46.4）** | 可用、离线、无 GL 依赖 | **采用** |

> 三者（MeshLab QECD / Blender Collapse / VTK QuadricDecimation）是**同一族算法**：都用二次误差度量 + 边折叠 +
> 最优位置放置（MeshLab 文档原文见 §13.2.2）。差别只在"边界/法线/拓扑"这几个旋钮上；
> 本相位用"边界边不增长 + 剪影 IoU + 单向表面误差"三条把它量出来，而不是靠"用了哪个工具"取信。

#### 13.3.2 三角形预算（每个 link 的上限）

取 §13.2.2 两篇独立来源的交集：**每 link ≤ 50k**（Blender 那篇的 `<50k`；另一篇给 ≤100k，取严的），
**整车 ≤ 200k**。逐 link 再按"原来多大 / 画面里多显眼 / 小特征多少"分档：

| link | 是什么 | 预算 | 实测输出 |
|---|---|---|---|
| `base_link` | 底盘（最大最显眼） | 50,000 | 50,000 |
| `l11` | 云台 pitch + 发射机构（整车最高、细节最多） | 40,000 | 39,999 |
| `l2`..`l5` | 四个转向模块 | 12,000 ×4 | 12,000 / 11,999 / 11,998 / 12,000 |
| `l10` | 云台 yaw | 8,000 | 8,000 |
| `l12` | Livox MID-360 | 6,000 | 6,000 |
| `l6`..`l9` | 四个轮（几乎是纯圆柱） | 3,000 ×4 | 3,000 / 2,999 / 3,000 / 3,000 |
| **合计** | | **200,000** | **163,995** |

#### 13.3.3 逐 mesh 结果（**实测**，机器可读：`inventory/visual_decimation.json`）

容差：包围盒逐轴 ≤ **1.0 mm**；单向表面误差 `out→src` 的 p99 ≤ **3.0 mm** 且 max ≤ **12.0 mm**；
三视剪影 IoU ≥ **0.97**（2 mm 栅格）；边界边不得显著增长。

| mesh | 面数 前 → 后 | 磁盘 MB 前 → 后 | 包围盒差 (mm) | 误差 p99/max (mm) | 剪影 IoU 最差 | 边界边 前 → 后 | 判定 |
|---|---|---|---|---|---|---|---|
| `base_link` | 2,078,226 → **50,000** | 103.91 → 2.50 | **0.000** | 0.07 / 2.45 | 0.9964 | 19,636 → 2,380 | ✅ |
| `l11` | 176,458 → **39,999** | 8.82 → 2.00 | 0.022 | 0.00 / 0.20 | 0.9975 | 52 → 14 | ✅ |
| `l4` | 91,234 → **11,998** | 4.56 → 0.60 | 0.048 | 0.00 / 0.05 | 0.9956 | 4 → 34 | ✅ |
| `l5` | 90,250 → **12,000** | 4.51 → 0.60 | 0.048 | 0.00 / 0.12 | 0.9974 | 4 → 28 | ✅ |
| `l2` | 89,556 → **12,000** | 4.48 → 0.60 | 0.048 | 0.00 / 0.39 | 0.9960 | 4 → 20 | ✅ |
| `l3` | 88,800 → **11,999** | 4.44 → 0.60 | 0.048 | 0.00 / 0.16 | 0.9969 | 4 → 14 | ✅ |
| `l12` | 68,740 → **6,000** | 3.44 → 0.30 | 0.042 | 0.00 / 0.13 | 0.9987 | 500 → 76 | ✅ |
| `l10` | 37,648 → **8,000** | 1.88 → 0.40 | 0.109 | 0.00 / 0.09 | 0.9984 | 0 → 5 | ✅ |
| `l7` | 25,324 → **2,999** | 1.27 → 0.15 | 0.297 | 0.16 / 0.52 | 0.9919 | 0 → 48 | ✅ |
| `l8` | 25,324 → **3,000** | 1.27 → 0.15 | 0.297 | 0.16 / 0.53 | 0.9939 | 0 → 46 | ✅ |
| `l9` | 25,324 → **3,000** | 1.27 → 0.15 | 0.297 | 0.17 / 0.53 | 0.9929 | 0 → 48 | ✅ |
| `l6` | 24,436 → **3,000** | 1.22 → 0.15 | 0.297 | 0.18 / 0.52 | 0.9929 | 0 → 46 | ✅ |
| **合计** | **2,821,320 → 163,995**（−94.2%） | **141.07 → 8.20**（−94.2%） | max **0.297** | max p99 **0.18** / max **2.45** | min **0.9919** | — | **12/12 过** |

**误差口径的两句交代（诚实）**：
1. 默认口径是"在 dst 表面均匀撒点建 KD-tree，再查 src 的点"，误差上界 ≈ 采样间距 ≈ `sqrt(2·Area/N)`
   （实测间距 0.40~2.10 mm），并**已经减掉采样间距**（⇒ 报出来的是**保守偏低**的数）。
   为什么不用精确的 `vtkImplicitPolyDataDistance`：它在 dst 上建 cell locator，`base_link` 那种 208 万面的 dst
   **10 分钟都出不来**（本工具第一版就是死在这里，见 §13.10 第 6 项）。精确口径保留为 `--exact-err`。
   **两种口径的对照已实测**（`--only l12 --calibrate`，同一对网格、同一次运行）：

   | 口径 | `out→src` max | p99 | mean |
   |---|---|---|---|
   | 精确（`vtkImplicitPolyDataDistance`） | 0.295 mm | 0.204 mm | 0.050 mm |
   | 采样（KD-tree，间距 0.40 mm，已减间距） | 0.077 mm | 0.000 mm | 0.000 mm |
   | 差 | **−0.218 mm** | **−0.204 mm** | **−0.050 mm** |

   ⇒ 采样口径**偏低 ≤0.22 mm**（设计如此：宁可不虚报误差），比 3.0 mm 的判据小一个量级。
2. 误差**双向**都量了（`out→src` 与 `src→out`），判据卡的是 `out→src` 的 p99/max
   （= "抽稀件偏离原表面的量"）；`src→out` 另存清单，用于发现"整块小特征被抽没了"。

#### 13.3.4 还没完：三件"几何之外"的检查（都是本槽位专属的坑）

| 检查 | 为什么 | 结果 |
|---|---|---|
| **单位 / 原点 / scale** | 从 CAD 出来的件最容易死在 mm↔m 与"导出时忘了 apply transform"上 | 两档都**没有** `<mesh scale=...>`；包围盒差 max 0.297 mm（`l6`..`l9` 的轮，来自半径方向的离散化） |
| **`<visual><origin>` 逐字不变** | 换文件不能顺手把位置/朝向换了 | 13 个 link 的 `<visual><origin>` 两档**逐字相同**（`check_robot11_visual_slot.py` B1） |
| **雷达凹槽不能被"抽稀封上"** | 这台车的雷达在底盘**凹槽**里（§10.6：Phase 2 的 75.5% 自击就是这么来的）。抽稀如果把凹槽开口抹平，视觉上就是一块板盖住雷达 | 用 30,000 条 MID-360 射线打**视觉网格**，只在雷达原点 ±0.13 m 的局部盒内求交（**对 `t<0.12 m` 这个量是精确的**：更远的面不可能产生 <0.12 m 的命中）。`base_link` 自击 `t<0.12 m`：原件 **0.3833** → 抽稀 **0.3810**（Δ**−0.23 个百分点**）；`l11` 0.0031→0.0031；`l10` 0→0 |

> ⚠️ 最后一条是**外观检查**，不是**感知检查**：Gazebo 的射线传感器走 ODE、**只与 `<collision>` 求交**
> （本仓插件的实现：`livox_ode_multiray_shape.cpp` → `ODERayShape`；见 §11.2 与 §13.5）。
> 所以"视觉件抽稀"**不可能**改变点云 —— 这一条量的是"凹槽还在不在"。

### 13.4 三角形与代价：三档并排（**实测 + 一处明确标注的推断**）

#### 13.4.1 数据量（**实测**，来自清单与磁盘）

| 档位 | 指向的文件 | 12 个 mesh 面数合计 | 磁盘合计 | 相对 full |
|---|---|---|---|---|
| `full` | `meshes/<link>.STL`（上游 CAD 原件） | **2,821,320** | **141.07 MB** | 1× |
| `decimated`（**Phase 5 默认**） | `meshes/decimated/<link>.stl`（本相位的视觉 LOD） | **163,995** | **8.20 MB** | **1/17.2 面 / 1/17.2 字节** |
| （Phase 4 的旧 decimated 档） | `meshes/generated/<link>_collision.stl`（碰撞档抽稀件） | 9,700 | 0.486 MB | 1/291 |

> Phase 4 的 `robot11_visual:=decimated` 原本指向**碰撞档**抽稀件（`base_link` 3,000 面、`l11` 1,200 面）；
> **Phase 5 把它换成专用视觉 LOD**（`base_link` 50,000 面、`l11` 39,999 面）。
> 为什么换：碰撞档是"表面碰撞够用"的精度（`l11` 只有 1,200 面 ⇒ 云台/发射机构会糊成一团），
> 拿它当视觉件等于用"碰撞的精度"决定"看起来像不像"。旧档文件仍在库里、没删，
> 需要它的人把 `meshes/generated/` 直接指过去即可（launch 不支持这条，属于手工实验，见 §13.10）。

#### 13.4.2 GUI 内存：**本次实测**（Xvfb `3200x1200x24` + 软件 GL llvmpipe / Mesa 23.2.1）

口径：`world:=RMUC2026 mode:=mapping lio:=small_point_lio robot:=robot11 nav_rviz:=True`、车静止、
脚本 `tools/scripts/regress/robot11_gui_bench.sh <tag> --visual <decimated|full> --seconds <N> --rviz True`，
RSS 每 1 s 采样，窗口从"gzclient 与 rviz2 都起来"开始计。原始数据：`.tmp_robotslot/r11p5guiFULL/`、
`.tmp_robotslot/r11p5guiDEC/`、`.tmp_robotslot/r11p5guiFULL75/`。

| 指标 | `full`（40 s 窗） | `full`（**75 s 窗**） | `decimated`（40 s 窗，163,995 面） |
|---|---|---|---|
| `gzclient` 峰值 RSS | 473 MiB | 473 MiB | **473 MiB** |
| `gzserver` 峰值 RSS | 3,202 MiB | 3,202 MiB | **3,202 MiB** |
| `rviz2` 峰值 RSS | 322 MiB（75 s 窗同一跑）→ 见下 | **322 MiB（全程平稳，t≥15 s 后不再增长）** | **314 MiB** |
| `gzclient` 活满窗口 | ✅（无死亡行） | ✅ | ✅ |
| `rviz2` 活满窗口 | ✅ | ✅ | ✅ |
| 截图（rviz2 窗口矩形内）非黑比例 / 颜色数 | 0.5768 / 1,188 | — | 0.5768 / 1,188 |
| `/odom` 条数（窗口内） | 159 | — | 161 |
| 视觉三角形 | 2,821,320 | 2,821,320 | 163,995 |

**读法（三条，都是实测）**：
1. **两档都能起来、都活满窗口、都在画**（rviz2 窗口矩形内非黑 0.5768、1,188 种颜色）。
2. **`gzclient` / `gzserver` 的内存与视觉档位无关**：473 MiB / 3,202 MiB 两档**逐 MiB 相同**；
   这 3.2 GB 的主体是 **RMUC2026 世界本身**（Phase 4 的无头对照也是 3,112 MiB）。
3. **`rviz2` 也不随视觉档位变**：full 75 s 窗与 decimated 40 s 窗分别是 322 / 314 MiB，
   而且 full 那条曲线从 t≈15 s 起**完全平**（308→322 MiB 后不动）。

#### 13.4.3 ⚠️ 诚实记录：本相位**没有**测出"抽稀省 GUI 内存"（与 §12.2 不一致）

Phase 4 的 §12.2 报的是 **rviz2 峰值 1,381 MiB（full）↔ 243 MiB（decimated）**，差 **−1,138 MiB**；
本相位在**同一脚本、同一世界、同一组 flags、同一虚拟屏 + 软件 GL** 下重跑：

* `full`（40 s 窗）：rviz2 314 MiB；`full`（**75 s 窗**）：rviz2 322 MiB 且**平稳**；
* `decimated`（163,995 面，40 s 窗）：rviz2 314 MiB；
* 两者的**逐秒曲线形状几乎完全相同**（连 gzclient 的 218→481 MiB 爬升都一致）。

⇒ **我这一轮的结论是：视觉档位没有驱动 rviz2/gzclient 的内存**。可能的解释（**未证实**，逐条列着）：
1. `mode:=mapping` 用的 rviz 配置 `rm_navigation/rviz/nav2.rviz` 里那个 RobotModel display
   订的是 `/robot_description`，Durability 写的是 **Volatile**；若发布端是 transient_local，
   QoS 不兼容 ⇒ **RobotModel 根本收不到描述、也就不会去加载任何 robot mesh**
   （本轮 launch.log 里只看到 `/scan` 的 QoS 不兼容告警，没看到 `/robot_description` 的告警 ⇒ 证据不足，未定论）；
2. Phase 4 采样用的是 `ps -o rss= -C rviz2 | sort -rn | head -1` ⇒ 若**上一轮残留的 rviz2** 还活着，
   会被当成"本轮峰值"（他们的 `r11p4guiDEC` 那一跑 `rviz2=0`，说明 rviz2 起没起来在两次跑之间并不稳定）；
3. 两次跑的 `mode`/`rviz` 大小写组合不同（`nav_rviz=true` vs `True`），起没起 rviz2 本身就不一致。

**因此**：本报告**不主张**"抽稀视觉件能省下 GB 级 GUI 内存"。抽稀**确凿**省下的是：
三角形数 17.2×（相对 full）/ 291×（相对碰撞档）、磁盘 141.07 MB → 8.20 MB、
以及"渲染线程每帧要画的三角形数"与"网格解析/上传的工作量"（对应 §13.2.4 的
"GUI 是 render-thread-bound + 网格/材质构建是加载期大头"）。

#### 13.4.4 推断的部分（明确标注）

| 量 | 值 | 口径（**推断**，非实测） |
|---|---|---|
| 渲染端网格常驻足迹（OGRE + Gazebo `common::Mesh`） | full ≈ **0.5~0.7 GB**；decimated ≈ **30~40 MB**；碰撞档 ≈ **2 MB** | 模型：STL 三角汤 ⇒ 3 顶点/面；顶点 = 位置 3×f64 + 法线 3×f64 = 48 B ⇒ 144 B/面；索引 3×u32 = 12 B/面 ⇒ Gazebo 侧 ~156 B/面；OGRE 侧 vertex buffer 位置+法线各 3×f32 = 24 B/顶点 ⇒ 72 B/面 + 索引 12 B/面 ⇒ ~84 B/面。合计 ~240 B/面。**未实测**（`gzclient` 的 RSS 与档位无关这件事本身说明这个模型在本环境里量不到） |
| 真 GPU 上的帧率 | 未知 | 本轮只有 llvmpipe 软件 GL ⇒ **渲染吞吐没有参考价值**（与 Phase 4 §12.2 的边界一致） |
| 用户机器上那次 `gzclient` SIGKILL 的根因 | **仍未复现** | 本机 31.9 GB 内存、两档都活满窗口。整栈 ~3.9 GiB（gzserver 3.2 为主），若用户机器可用内存更小，"整栈 + 世界"这一块才是主因，而**不是** robot 的视觉档位 |

### 13.5 `robot11_visual:=decimated|full` —— 开关的最终语义（**xacro 现在由生成器拥有**）

| 项 | 值 |
|---|---|
| 开关 | `robot11_visual:=decimated`（**默认**）\| `full`（launch 侧 Phase 4 已加，本相位**没有改它**） |
| `decimated` 指向 | `package://robot11/meshes/decimated/<link>.stl` —— **本相位的视觉 LOD**（163,995 面 / 8.20 MB） |
| `full` 指向 | `package://robot11/meshes/<link>.STL` —— 上游 CAD 原件（2,821,320 面 / 141.07 MB） |
| 兼容别名 | `visual_decimated:=true|false`（Phase 4 引入的布尔别名）**仍然有效**：非空时它赢（保证"命令行参数优先于默认值"）。两个都不传 ⇒ `decimated` |
| 只影响 | `<visual>` 的 mesh **文件**。`<collision>`（76 个 carved box + 云台细盒 + 4 个轮 cylinder）、`<sensor>`、`<plugin>`、`<inertial>`、`<visual><origin>`、`<mesh scale>` **全部逐字节不变** |
| 生成方式 | `tools/scripts/regress/robot11_make_sim_xacro.py`（**生成器拥有**）。Phase 4 的手改块已按同一语义搬进生成器；`sentry_robot_robot11_sim.xacro` 的头部注释里写着"本文件是生成的，不要手改" —— **手改会在下次重跑时被覆盖，这是预期行为** |
| 生成前置 | 清单缺失或 `all_ok=false` ⇒ 生成器**直接报错**（不生成"看起来对但没人验证过"的模型） |
| 校验工具 | `tools/scripts/regress/check_robot11_visual_slot.py`（**真的跑 `xacro` 渲染两档**，再逐元素比） |

`check_robot11_visual_slot.py` 的断言（当前**全绿**，21 项；JSON 落 `.tmp_robot11/work/visual_slot.json`）：

| # | 断言 | 结果 |
|---|---|---|
| A0 | 两档 xacro 都渲染成功 | ✅ 50,180 B / 50,060 B |
| A1 | 两档 URDF「除 visual mesh 文件名外」**逐字节相同**（`<visual>` 的 filename 换成占位符后规范序列化比对） | ✅ |
| B1 | 每个 link 的 `<visual><origin>` 两档逐字相同 | ✅ 13 个 link |
| C1 | 两档引用的 mesh 文件全部存在 | ✅ |
| D1 | 没有任何 `<mesh scale=...>`（STL 单位 = 米） | ✅ |
| E1/E2/E3 | decimated 每 link ≤50k、整车 ≤200k（实测 163,995 = full 的 **5.8%**）；full 档整车 = **2,821,320**（与 §9.2 逐 mesh 清单求和一致） | ✅ |
| F1 | 清单里的抽稀件与磁盘**逐字节一致**（sha256 + 面数） | ✅ 12/12 |
| F2/F3/F4/F5 | 每条都过容差；最差 p99 **0.18 mm**、最差包围盒差 **0.297 mm**、最差剪影 IoU **0.9919** | ✅ |
| G1 | 上游质量合计 **9.5521 kg** | ✅ |
| G2 | `body_to_livox` 的 xyz 逐字不变（**0.000561701 0.130915824 0.157028170**） | ✅ |
| G3 | 四个转向关节 origin = ±0.1821345596729（轴距=轮距 0.36427 m） | ✅ j2..j5 |
| G4 | 四个轮的 `<collision>` 仍是 `cylinder r=0.058 l≈0.0452` | ✅ 4/4 |
| G5 | 两档 collision 元素个数相同（113 个） | ✅ |
| H1~H4 | 不传参默认 = decimated；`visual_decimated:=true/false` 分别 = decimated/full；两者冲突时**显式别名赢** | ✅ |

**构建注意**：`install/` 是**逐文件符号链接** ⇒ 新目录 `meshes/decimated/` 必须重新构建才可见：

```bash
colcon build --symlink-install --packages-select robot11 rm_nav_bringup
ls -l install/robot11/share/robot11/meshes/decimated/     # 期望 12 个软链
python3 tools/scripts/regress/check_robot11_visual_slot.py
```

回退：`robot11_visual:=full`（回到上游原始 STL）；或把生成器的 `visual-inventory` 指向旧清单重跑。
**删掉 `meshes/decimated/` 也可以**（`full` 档不引用它），但那样 `decimated`（默认）会因文件不存在而加载失败。

### 13.6 自击掩膜：本相位的**独立复测**（Phase 4 的设计见 §12.3，未改一行）

同世界（RMUL2026）、同出生点、`mode:=mapping`、`lio:=small_point_lio`、`nav_rviz:=False`，
探针 `tools/scripts/regress/slope_speed_probe.py`（114 帧静止 + 75 帧行驶）：

| 指标 | 静止（114 帧） | 行驶（75 帧） | 判据 |
|---|---|---|---|
| `self_masked` 点/帧（中位） | **3,374**（min 3,285 / max 3,480） | 4,460（2,915~8,721） | 掩膜真的在跑 |
| 被掩比例（`cloud_masked_frac`） | 0.2821（自击 `t<0.12 m` 实测 0.2822 —— **两者吻合**） | 0.2949 | — |
| 走廊最近格 `data_min_d` | **0.20 m（114/114 帧）** | 0.0~0.2 m | 静止时**不再出现 d=0.00** |
| 近场（d<0.30 m）max 台阶残差 | **0.002~0.005 m** | 0.002~1.695 m（真地形） | 静止时远低于 0.06 死区 ⇒ **无假台阶** |
| 近场 max 局部坡度 | 2.51~2.85°（= 真地面坡度） | — | 不再被自击格钉成 0.0° |
| **限速值 `limit`** | **1.0 m/s（114/114 帧）** | 0.6~2.0（中位 0.789） | 静止时**不在地板**（`limit_at_floor` = **0/114**） |
| 限速原因 `why` | `slope_change` 114 | **`step` 28 / `slope_change` 29 / `slope` 1 / `none` 17** | 行驶时**真台阶照样限速**（最低 0.6 m/s） |

⇒ 两条要求都满足：**静止不误限速**（114/114 帧 1.0 m/s、近场残差 ≤5 mm），
**真特征仍然限速**（行驶 75 帧里 28 帧因 `step` 限速、限速降到 0.6 m/s，走廊 max 台阶到 1.695 m）。

> ⚠️ 行驶窗口里 `data_min_d` 仍会出现 0.00 —— 那是 Phase 4 §12.7 第 2 项已经登记的**第二个来源**
> （"承诺距离前推"在车不动/慢速时衰减到 0），**不是**自击（同一帧 `self_masked` 仍有 2,915~8,721 点）。
> 本轮**没有**改它（不在本主题范围）。

### 13.7 接线证明（`sensor_height` 等**运行期**真值）

Phase 4 已经把"横幅说意图"改成"横幅运行时读真正递给节点的那份 YAML"（§12.4）。
本相位在**自己重建之后**又独立取证一次（同一次 bash 调用里起栈 + `ros2 param get`）：

| 键 | 运行期真值（`ros2 param get /ground_segmentation <k>`） | 期望 | 结论 |
|---|---|---|---|
| `sensor_height` | **0.2595** | 0.2595（雷达离地实测，§9.4） | ✅ **不是 0.226** |
| `gravity_aligned_frame` | `""` | `""`（点云在源头已重力对齐，§11.5） | ✅ |
| `max_dist_to_line` | 0.05 | 0.05 | ✅ |
| `input_topic` | `/livox/lidar/pointcloud` | 同 | ✅ |
| `self_mask_enable` | `True` | robot11 槽位 = True | ✅ |
| `self_mask_radius_m` / `self_mask_z_min_m` | 0.2416 / −0.22953 | 同 | ✅ |

launch 横幅（运行时从文件读出来的生效值）同一次跑的原文：

```
· linefit 参数**已跟着槽位切**（生效值由 launch 运行时读该文件打印）：
  sensor_height=0.2595, gravity_aligned_frame="", input_topic="/livox/lidar/pointcloud",
  ground_output_topic="segmentation/ground"
  ← install/linefit_ground_segmentation_ros/share/.../config/segmentation_sim_robot11.yaml
· 自击掩膜（只作用于判据/限速，不动点云与 /segmentation/* 标签）：
  self_mask_enable=true, 近场死区 r<=0.2416 m（z>=-0.2295）, 113 个 collision AABB
```

### 13.8 无头验收（本相位跑的一次，`robot11_visual` 默认 = decimated）

跑法（一次 bash 调用，隔离同 `run_robot_model_probe.sh`）：

```bash
tools/scripts/regress/run_robot_model_probe.sh r11p5 --duration 30 --drive-seconds 12 \
  --dump-cloud .tmp_robot11/work/probe/frames --dump-scan .tmp_robot11/work/probe/scans \
  --require-scan --require-odom -- \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
  map_autocontinue:=False lio_rviz:=False nav_rviz:=False
```

| 项 | 期望 | 实测 | 判定 |
|---|---|---|---|
| spawn | 模型进世界 | `spawn_entity` 报 "Entity pushed to spawn queue, but spawn service **timed out**…Exiting"，但模型**确实进了世界**（TF/点云/odom 全有） | ⚠️ 已知抖动（超时判定，不是失败） |
| TF `base_link` | 存在 | ✅ (0,0,0) rpy 0 | ✅ |
| TF `livox_frame` | 在 body 系 (0.00056, 0.1309, 0.1570) rpy 0 | ✅ **(0.0006, 0.1309, 0.157)** rpy (0,0,0) | ✅ **与约束逐位一致** |
| TF `imu_link` | (0.0006, 0.1309, 0.107) | ✅ | ✅ |
| `/odom` | 有、10 Hz | ✅ 210 条 / 46.07 s 墙钟 = **10.0 Hz** | ✅ |
| `/livox/lidar/pointcloud` | 10 Hz | ✅ 210 帧 / **11,974 点/帧**（中位） | ✅ |
| `/segmentation/ground` | 有点 | ✅ 210 帧 / **5,689 点/帧**（中位） | ✅ |
| `/segmentation/obstacle` | 有点 | ✅ 210 帧 / 6,273 点/帧 | ✅ |
| `/scan` | 有、>4 m 与 <1 m 都有 | ✅ 210 帧 / 有限波束中位 **830**；距离分带（210 帧合计）：<1 m **109,926**、1–2 m 39,500、2–4 m 42,810、4–7 m 16,301、**>7 m 218** ⇒ "既有 >4 m 也有 <1 m"**成立** | ✅（但见 §13.9） |
| 自击比例 | 与 §11.2 同量级 | ✅ `t<0.12 m` = **28.69%**（`t<0.05 m` = 26.54%） | ✅ |
| 自击掩膜 | 生效 | ✅ 中位 **3,374 点/帧**被剔（§13.6） | ✅ |
| 静止假台阶 | 无 | ✅ 近场 max 残差 **0.002~0.005 m**；限速 **1.0 m/s（114/114）** | ✅ |
| 真台阶仍限速 | 有 | ✅ 行驶 75 帧：`why=step` **28 帧**、限速最低 **0.6 m/s** | ✅ |
| RTF | — | **0.455**（整栈：感知 + LIO + slam_toolbox + 世界 RMUL2026，墙钟口径 46 s 窗口） | 记录 |
| 契约：`/segmentation/obstacle` | 1 个发布者 | ✅ 1 发布者 / 1 订阅者 | ✅ |
| 契约：`/segmentation/ground` | 1 个发布者 | ✅ 1 发布者 | ✅ |
| 契约：`/odom` | 1 个发布者 | ✅ 1 发布者 | ✅ |
| 契约：`/cmd_vel_chassis` | 1 个发布者（nav2/velocity_smoother）1 个订阅者（planar_move） | ✅ 1/1 | ✅ |
| 契约：`/map` | 1 个发布者 | ✅ 1 发布者 | ✅ |
| `gzclient`（无头跑） | 不参与 | 死亡行 `exit code -6`（无 DISPLAY 时 RenderEngine 起不来 ⇒ abort）—— **无头跑的预期行为**，不是缺陷 | 记录 |

### 13.9 ⚠️ 新发现（**实测，未修**）：`/scan` 里恒有一圈"自身障碍"

`/scan` 由 `pointcloud_to_laserscan` 从 `/segmentation/obstacle` 生成，而**自击点仍然在 obstacle 里**
（掩膜只在**判据/限速**那一层生效，**没有**动 `/segmentation/*` 的标签 —— 这是有意的：
近场回波是**物理真实**的，动标签等于改共享契约）。

单帧实测（`.tmp_robot11/work/probe/scans/scan_01.csv`，`robot11_visual:=decimated`）：

| 距离带 | 波束数 | 占该帧有限波束（833 条） |
|---|---|---|
| **< 0.3 m** | **132** | 15.8% |
| **0.3 ~ 1.0 m** | **296** | 35.5% |
| 1 ~ 2 m | 244 | 29.3% |
| 2 ~ 4 m | 116 | 13.9% |
| > 4 m | 45 | 5.4% |
| **< 1 m 合计** | **428** | **51.4%** |

三帧（`scan_01/02/03.csv`，每帧 1,462 条波束、有限 816~833 条）合计的分带比例：
<0.3 m **15.2%** / 0.3~1 m **34.7%** / 1~2 m 28.6% / 2~4 m 14.7% / >4 m 6.9%
⇒ **<1 m 占有限波束的 ~50%（三帧分别 51.4% / 49.8% / 48.4%）**，稳定复现。

* `nearest_range = **0.05 m**` ⇒ 确认是自击（与插件 `point = range·axis` 的系统内移一致，§12.3.1）。
* <1 m 的 428 条在方位上**集中在约 120° 的扇区**（30° 分箱：0/0/0/14/9/1/**119/90/91/96**/8/0）
  ⇒ 形状是"车体自己"，不是一圈均匀的环。
* 210 帧合计 **109,926** 条 <1 m ⇒ **523 条/帧**（Phase 4 §12.7 第 4 项报的是 **218.7 条/帧**；
  两次跑的世界/出生点/朝向不同，量级一致，**都是"自障碍"**）。

**风险**：nav2 的局部代价图（STVL / voxel / obstacle 层）会把这一圈 5 cm~1 m 的"障碍"画在车周围
⇒ 规划器可能认为车被围住（这正是 Phase 4 登记的那个"目标被接受但车不走"的新候选根因）。
**本轮没有改任何共享语义**（改 `/segmentation/*` 的标签会影响默认模型与其它槽位），
只把它量清楚、登记在这里。**下一步的候选修法**（未做）：在 `pointcloud_to_laserscan` 侧加
"自击掩膜"（同一份 `self_mask_*` 参数），或在插件侧修 `point = range·axis` 的内移。

### 13.10 本相位的未验证清单（诚实清单）

1. **`gzclient` 的 SIGKILL 仍未复现**：本机（31.9 GB）两档都活满窗口；本轮 GUI 数字是
   **Xvfb + 软件 GL（llvmpipe）**上量的 ⇒ 只能证明"进程活着、没黑屏、内存多少"，
   **不能**代表真 GPU 的渲染吞吐或真机内存压力。
2. **"抽稀省 GUI 内存"在本机没有被证实**（§13.4.3）：`gzclient` 473 MiB、`gzserver` 3,202 MiB、
   `rviz2` 314~322 MiB **两档都一样**，且与 Phase 4 §12.2 的 rviz2 1,381↔243 MiB **不一致**。
   我已把三条可能的原因列在 §13.4.3，但**没有定论** ⇒ 用户机器上的实测才是裁决。
3. **渲染端足迹（~240 B/面）是推断**，不是实测（§13.4.4）。
4. **`meshlabserver` 的 QECD 没跑起来**（无 GL）⇒ "两种实现给出同样的抽稀结果"这件事**没有对照过**；
   本相位只用"量化容差"而不是"工具身份"来取信。
5. **`--calibrate` 的对照已补做**（`--only l12`，见 §13.3.3 注 1）：精确口径 p99 0.204 mm /
   采样口径 0.000 mm ⇒ 采样**偏低 ≤0.22 mm**，比 3.0 mm 判据小一个量级。⚠️ 只在 `l12` 上对过
   （大网格的精确口径跑不出来，见下一条）⇒ "所有 mesh 都校准过"**不成立**。
6. **`base_link` 的精确误差口径跑不出来**（`vtkImplicitPolyDataDistance` 在 208 万面 dst 上 10 分钟未返回）
   ⇒ `base_link` 的 p99/max 只有采样口径的数。这是**已知的**方法学缺口。
7. **抽稀后的视觉在真 GPU 上"看起来够不够"没看过**：本相位只有数字（包围盒/误差/剪影/自击），
   **没有**一张真渲染截图做视觉确认（软件 GL 下的截图不能当"好不好看"的证据）。
8. **`l6`..`l9`（轮）的包围盒差 0.297 mm 全部来自半径方向**：3,000 面对一个 r=58 mm 的圆柱
   ⇒ 内接多边形半径损失 ~0.3 mm。**没有**换成 `<cylinder>` 视觉（那会改变 `<visual>` 的几何类型，
   超出"只换文件"的范围）。
9. **`decimated` 档与 `full` 档的物理/感知等价性**：本轮证明了两档 URDF 除 visual 文件名外逐字节相同
   （A1/G5），但**没有**做"两档各跑一次整栈、逐帧比点云/odom"的端到端对照（成本高；A1 已经在
   结构上排除了差异来源）。
10. **`meshes/decimated/` 的入库策略**：12 个文件 **8.20 MB**，已入库（理由：默认档直接依赖它，
    且原文本仓本来就跟踪 141 MB 的 CAD 原件；`git` 单文件上限 100 MiB，最大者 2.50 MB）。
    完全复现只需 `python3 tools/scripts/regress/robot11_decimate_visuals.py`（约 6 分钟）。
    若将来不想入库，把它改成构建期生成即可（**未做**：那会给构建加 pyvista/VTK 依赖）。
11. **`/scan` 的自障碍（§13.9）没有修**，也没有验证"它是不是 nav 不走的根因"。
12. **`robot11` 的 `model://` mesh 解析**（= "Gazebo 里车没有视觉 / GUI 卡住"那条）在 Phase 5 时
    还是"手工前缀 `GAZEBO_MODEL_PATH`"的 workaround —— **已由 §14（2026-10-08）自动化并实测**。

---

## 14. 2026-10-08：`robot:=robot11` 的 `model://` mesh 解析已**自动化**（不再需要手工 `GAZEBO_MODEL_PATH`）

> 一句话：**用户那条命令现在直接跑就行**，不用再前缀环境变量；**launch 侧那条腿**对默认模型 /
> 其它世界**一个字节都没动**（0 动作、0 日志；⚠️ 包侧那条腿是 env 上的一项增量，见 §14.6）。
> 机理、取证、三组对照实测的完整版在 **`docs/gazebo_gui_troubleshooting.md` §5.1**（本节只放结论与回滚）。

### 14.1 为什么需要（一句话机理）

URDF 的 `package://robot11/meshes/decimated/<link>.stl`（12 个 `<visual>`）会被 sdformat 在
URDF→SDF 时改写成 `model://robot11/meshes/decimated/<link>.stl`，而 gazebo 的 `model://` 解析根
只有 `$HOME/.gazebo/models` + `GAZEBO_MODEL_PATH`；解析不到时 `SystemPaths::FindFileURI()`
**无条件**回落到在线模型库并**同步阻塞**（实测 stall 48.03 / 76.34 / 99.67 s，不设上限），
随后 12 个 mesh 全部 `No mesh specified` ⇒ **车在 Gazebo 里没有视觉**。
RViz 不受影响（它走 `package://` + ament 索引）。详见 §13.9 与那份文档的 §3。

### 14.2 现在是什么机制（两条腿；文件 + 键 + 具体路径）

| 机制 | 文件 / 键 | 追加进 `GAZEBO_MODEL_PATH` 的目录 | 作用域 |
|---|---|---|---|
| **(a) 启动侧（本仓 launch）** | `rm_nav_bringup/launch/bringup_sim.launch.py` 与 `hzmi_rm_simulation/launch/rm_simulation.launch.py` 里的 `_gazebo_model_path_setup()`（`OpaqueFunction` + `AppendEnvironmentVariable`，**排在 include gzserver/gzclient 之前**） | `<install>/robot11/share` | 只有 `robot:=robot11` 才执行 ⇒ 默认模型 / `hzmirm` 的 env、日志、时序**逐字节不变**；追加不覆盖用户原值；用户已手工 export 过同一目录时幂等跳过 |
| **(b) 包侧（package.xml）** | `src/rm_simulation/robot11_description/package.xml` 的 `<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` | `<install>/robot11/share/robot11/..`（同一目录） | 任何 gazebo 入口（含裸 gzserver/gzclient）都吃得到；由 gazebo_ros 的启动脚本扫描得到 |

**哪条在实际运行里干活**：两条都会出现在 gzserver/gzclient 的 env 里（实测见那份文档 §5.1.3），
gazebo 按顺序**先命中 (b)**；但 **(a) 是"本仓 launch 一定可用"的保证**（不依赖 package.xml 是否重建过），
**(b) 负责我们 launch 之外的入口**。两者**各自单独就够**（隔离实测：只有 (a) → 0 错误 + 12 个 mesh
全部读进来；只有 (b) → 同样）。

**★ 2026-10-08 接手复核（无头、`gui:=False`、**不设任何** `GAZEBO_MODEL_PATH`）**：
`robot:=robot11` 时 gzserver 真身的 env 是**三项**（`…/share/robot11/..` **:** `…/hzmi…/meshes` **:**
`…/install/robot11/share`）—— 第一项来自 (b)、最后一项来自 (a)，`[Err] … No mesh specified` **0**、
`Waiting for model database update` **0**、spawn 成功、33 个节点；**单独**跑轻量入口只打印 1 条
`[gzmodel]`（"追加"）⇒ bringup 那条命令里第 2 条 `[gzmodel]`（"已存在，无需追加"）是**被 include 的
轻量入口**打印的幂等确认，**不是**同一入口跑了两遍。包侧 export 也补做了"真构建"验证
（含**非 symlink** 的独立 install 前缀）。完整表格与工具：那份文档 **§5.1.6**。

**★ 唯一的偏离（env 不是"零改动"，必须知道）**：(a) 只对 `robot:=robot11` 执行（其它槽位 0 动作、
0 日志）；但 (b) 是 `package.xml` 的 export ⇒ **任何** gazebo 入口（默认模型、其它 world、裸 gzserver）
的 `GAZEBO_MODEL_PATH` 都会多出 `…/install/robot11/share/robot11/..` 这一项。实测副作用：
无头跑**量不出来**（默认模型跑：`No mesh specified` 0、`Missing model.config` 0、节点集合与 robot11 跑
**逐个相同**）；有 GUI 时"插入模型"面板会多扫一个目录（`Missing model.config` 噪音 3 → 6 行）。
不想要这一项：删掉 `package.xml` 那一行并重建 robot11（§14.5），代价是裸 `gzserver`/`gz sim`
入口又会回落在线模型库。取证：那份文档 §5.1.6 与本仓 `docs/tilted_lidar_fidelity.md` §H。

### 14.3 实测（本次，隔离 + 黑洞代理 + Xvfb 软件 GL）

| 场景 | 「等在线模型库」 | `No mesh specified` | gzclient 事件数 | gzclient 读入（rchar） |
|---|---|---|---|---|
| 改造前（同一条命令，HEAD launch + 无 export） | **出现，stall 51.64 s**（拆掉黑洞才结束） | **34** | **275** | 5.67 MB（世界资产） |
| 现在（同一条命令，无任何手工环境变量） | **没有** | **0** | **27** | 5.67 MB（世界资产） |
| 改造前（轻量入口 `rm_simulation.launch.py`，同协议） | **出现，stall 35.46 s** | **45** | **374** | 4.21 MB |
| 现在（轻量入口，同协议） | **没有** | **0** | **28** | **12.47 MB**（+8.26 MB ≈ 12 个 mesh 的 8.20 MB；fd 扫描抓到 `…/decimated/l9.stl`） |
| 只有 (a) / 只有 (b)（隔离，正证据） | 没有 | 0 / 0 | 21 / 21 | **12.16 MB**（+8.26 MB） |

用户那条命令现在的实测：launch→spawn **8.06 s / 8.81 s**、`/odom` 34 条/20 s（RTF 0.17 限制）、
`/segmentation/ground` **6640 pts/帧**、`/scan` **1462 波束**（有限值 510，最远 7.9 m）、
节点集合与改造前**逐个相同**。

> ⚠️ `No mesh specified` 的次数不是常数（24 / 34 / 45），判据是"**是不是 0**"；
> 而"0 错误"**只有在同时看到 mesh 被读进来时**才算正面证据 —— 本沙箱里 gzclient 对
> "运行期插入的模型"处理不稳定（改造前后都有"整窗不建模型"的情况）⇒ 上表第 4/5 行的
> `rchar +8.26 MB` 才是"解析链真的通了"的证据。

### 14.4 用户那条命令（`robot:=robot11`，**不需要**任何环境变量前缀）

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav lio:=small_point_lio \
  localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 robot:=robot11 nav_rviz:=True
```

### 14.5 回滚

* 只想回退"启动侧"：删掉两个 launch 里的 `ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))`
  那一行（+ 可选删掉函数定义）⇒ 回到"必须手工前缀环境变量"的老状态（其余一切不变）。
* 只想回退"包侧"：删掉 `package.xml` 里的 `<gazebo_ros gazebo_model_path="${prefix}/.."/>` 一行，
  重新 `colcon build --symlink-install --packages-select robot11`。
* 两条都回退后，手工 workaround 依旧可用：
  `GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" ros2 launch …`。

### 14.6 本节未验证（诚实清单）

1. **真 GPU / 真显示器没跑过**：本沙箱只有 Xvfb + llvmpipe，"mesh 加载"是用**文件级证据**
   （`/proc/<gzclient>/io` 的 `rchar` 增量 8.26 MB + fd 扫描抓到 `.stl`）证的，不是看渲染像素
   ⇒ "车在真机屏幕上好不好看"仍未验证。
2. **本沙箱的 gzclient 对"运行期插入的模型"处理不稳定**：同样配置的多次跑里，有的会去建模型
   （于是能看到 12.47 MB 的 mesh 读入），有的整个窗口都不建（`rchar` 只有世界资产）。
   改造前后都有这个现象 ⇒ 它属于 gazebo classic 客户端在本沙箱的行为，**与本次修法无关**；
   但也因此，"**没有** `No mesh specified`"这句话只有在同一次跑里同时看到 mesh 读入时才成立
   （§14.3 第 4/5 行就是这种完整证据）。
3. **两个等价解析根会让 `InsertModelWidget` 多报 3 行 `Missing model.config` 噪音**
   （gzclient 事件数 21 → 27/28）：不影响加载/渲染，**没有**去消（要消得给 `install/robot11/share/robot11/`
   补 `model.config`，而那会让"不是模型目录"的语义变含糊）。
4. **`robot11_visual:=full` 档没有单独复测**解析链：两档只差 `<visual>` 的文件名/后缀，
   解析机制完全相同（`model://robot11/meshes/<link>.STL` vs `…/decimated/<link>.stl`），
   本次只对 `decimated`（默认档）做了端到端实测。

---

## 15. 2026-10-08：`robot11_mount:=plugin|urdf`（"斜 30° 到底斜在哪儿"）—— 指针

> 一句话：**默认没变**（`plugin` = §11 起的行为，去注释后生成物逐字节相同）；
> 新增一个 opt-in 档 `robot11_mount:=urdf`，把 30° 倾角**放回 URDF 关节**
> （= 上游/CSV 的字面值、**画出来的雷达 mesh 一起斜** = 实物的物理安装姿态），
> 插件 `<tilt_rpy>` 变单位阵。**该档不动任何感知参数** —— 就是为了看"倾斜放置最原始的效果"。

完整取证（A 什么斜了/什么没斜 · B 逐帧点账本与"有没有被裁"的定论 · C 开关用法 +
`urdf` 档实测与**坏在哪儿** · D 代价图/膨胀半径归因 · E 建议 · F 复现 · G 未验证）见
**`docs/tilted_lidar_fidelity.md`**。四条与本文件关系最大的结论：

1. **§11.3 第 3 项（`body_to_livox` 的 rpy 改成 `0 0 0`）仍然成立**，只是现在**可以按需切回**
   上游字面值（`robot11_mount:=urdf`）。两档的**世界射线方向逐条相同**（逐点差 ≤5×10⁻⁵ m）。
2. **§13.9 的"`/scan` 里恒有一圈自身障碍"有了归因**：它确实在 `/scan` 里（<1 m ≈ 一半波束），
   但它**不是**把车包住的那一圈 —— 实测代价图里 **离雷达 0.25 m / 离车心 0.39 m 以内
   没有任何 lethal 格**（自击团被 `obstacle_min_range: 0.1` 挡掉）。
3. **§11.8 第 1 项（"目标被接受但车 90 s 没动"）有了直接证据**：控制器
   `RegulatedPurePursuitController detected collision ahead` → `Controller patience exceeded`
   → 自旋恢复；真值 30 s 只走 0.18 m。根因是 **车被膨胀层包住**（车半径圆内 513/996 格 ≥99、
   free 仅 6 格），而"包住它的 lethal 格"来自 **0.26~0.40 m 的近场地面残留（经 `p2l` 的 2D 投影
   被标在雷达高度）+ 0.5~1.0 m 处 ~0.15 m 高的场地低矮件**，再乘上本槽位的
   `robot_radius 0.3565` + `inflation_radius 0.70/0.75`。**默认模型在同一场地是 free(0)、圆内 0/996。**
4. **§13.2/§12.7 的"点云少了很多"不是被裁**：一帧 30000 条射线里只有 **11960 条有回波（39.9%）**，
   其余在插件里就丢了；`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered`
   **每帧点数一致**（11960 / 11960 / 11968）⇒ 下游一级都没裁（`small_point_lio` 发布的云用的是
   `dense_point_deque`，不受 `min_distance 0.5`/`space_downsample` 影响）。

回退：去掉 `robot11_mount:=urdf`（默认就是 `plugin`）；`robot11_mount` 只对 `robot:=robot11`
生效，取值只能是 `plugin|urdf|sensor`（选错直接报错）。工具：
`tools/scripts/regress/run_robot11_mount_probe.sh` + `robot11_mount_probe.py`（本轮新增）。

> ★ **2026-10-09：新增第三档 `robot11_mount:=sensor` + 修掉两个 LIO 侧 bug** —— 详见
> **`docs/tilted_lidar_fidelity.md` §J**（施工记录 / A-B 表 / 配方 / 回退 / 未验证）：`sensor` = 与
> `urdf` 档逐字节相同的关节/插件渲染 + 插件 `<cloud_frame>sensor</cloud_frame>`（点云表达在
> **真·传感器系**、与 `frame_id` 自洽），且 launch **只在这一档**给 linefit 加
> `gravity_aligned_frame: base_link`、给 p2l 加 `target_frame: base_link`；同时修掉
> `/cloud_registered` 多乘一次 `T(base_link←livox_frame)`（bug ②：`urdf` 档 **30.970° → 1.054°**）
> 与 `odom→base_link` 的**假俯仰**（bug ③：**4.890° → 0.317°**，真值 0.003°）。
> `plugin`（默认）与其它模型/槽位的渲染、参数、行为**逐字节不变**（§J.3 静态证据 + §J.2 逐项 diff）。

### 15.1 ★ 2026-10-08 接手复核的补充（上面四条结论全部复现）

第二个人用新跑的数据重算了一遍（`.tmp_robotslot/r11m_plugin5`（静止）/`r11m_plugin6`（driven）/
`r11m_urdf4`（driven）/`r11m_urdf5`（静止）），并补了原文标"未验证"的两项 —— 完整表格见
**`docs/tilted_lidar_fidelity.md` §H**：

* 点云/平面拟合**逐位复现**（`(-0.01376, 0.0122, 0.99983)`、`d=0.26038`、夹角 1.054°、自击 3294、
  低于水平面 1770、`/cloud_registered` 0/11620）；`urdf` 档 `/cloud_registered` 地面 **30.618°**、
  低于水平面 **7511/11622 = 64.6%**；车半径圆内 `≥99` **453/1000**（原文 513/997、435/1001）
  ⇒ **"车一开始就在 inscribed 团里"三次独立复现**。
* **"画出来的雷达 mesh 平/斜"从推论升级为直接读数**：`gz model -m robot -i`（Gazebo transport，
  **不经 ROS/TF**）里被 lump 的视觉 `…fixed_joint_lump__livox_frame_visual_1`（mesh `l12.stl`）
  默认档 **roll 0.000°**、`urdf` 档 **roll −30.000°**。
  （原文说的 `/get_entity_state` 其实**在本仓 launch 里不存在** —— gzserver 没加载
  `libgazebo_ros_api_plugin.so`，不是"grep 过滤掉了"。）
* **LIO 漂移同协议 A/B 做了**（新增 `--drive`：直行 10 s ↔ 原地转 10 s）：driven 窗
  `plugin` 0.0094 m / `urdf` 0.0483 m，静止窗 0.0264 / 0.0134 m ⇒ **两种协议下排序相反，
  仍在噪声量级内**，不主张"哪个档漂得多"。
