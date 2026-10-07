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
