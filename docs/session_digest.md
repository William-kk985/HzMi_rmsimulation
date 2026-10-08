# 本工作期总汇总（先看这份）：`robot:=robot11` 全程 + 感知/LIO/导航/建图/资产

> **这份文档是什么**：把本工作期（**2026-10-05 → 2026-10-08**，按 `git log` 作者日期 **161 个提交**）
> 在 `docs/` 下散落的 **67 份文档**里的改动**合并成一份**。你想"只读一份就知道改了什么、为什么改、
> 怎么查出来的、现在的数字是多少、怎么退回去"——读这一份就够；要抠细节再按每条的"证据"列跳回原文档。
>
> **它是怎么造出来的（可核对）**：全部内容来自
> ① `git log`（提交是"改了什么"的唯一真值）；② 下列文档的原文（逐条引用到 `§`）：
> `README.md`、`robot_models.md`、`tilted_lidar_fidelity.md`、`gazebo_gui_troubleshooting.md`、
> `lio_divergence_no_impact.md`、`gicp_divergence_and_jitter.md`、`gicp_initialpose_latency.md`、
> `lio_drift_diagnosis.md`、`slope_speed_limiting.md`、`traversability_plan.md`、
> `path_clearance_and_contact.md`、`timestamp_construction_audit.md`、`slam_toolbox_scan_drops.md`、
> `slam_toolbox_tuning.md`、`slam_drops_loopclosure_odom.md`、`map_assets.md`、`mapping_2d_from_cloud.md`、
> `continue_mapping.md`、`nav2_vs_custom_planner.md`、`architecture.md`、`tf_interface_contract.md`、
> `sim_real_contract.md`、`params_ownership_checklist.md`、`issues_and_findings.md`、`worlds.md`、
> `ground_segmentation_slots.md`、`stvl_local_costmap.md`、`mapping_small_point_lio.md`、
> `lio_slots.md`、`localization_slots.md`。（**没有**读到的源，见 §5.3 的"源可核性"表。）
>
> **证据分级（全文逐条执行，不要混读）**：
>
> | 标记 | 含义 |
> |---|---|
> | **【实测】** | 有可复现产物：跑出来的 JSON/NPZ/CSV、`ros2 param get` 回读、源码 `file:line`、日志原文 |
> | **【推断】** | 由实测推导但**没有**直接测量（写出推断链与证伪条件） |
> | **【未验证】** | 明确登记为没做/没跑完/只有一个样本的事 |
> | **【来源】** | 上游文档/issue/规范/论文（带 URL） |
>
> **答案都是"实测"的**：本工作期的一大特点是**每条结论后面都有一次"判决性测量"**；
> 反过来，凡是**没有**测量支撑的，本文一律标 **【推断】/【未验证】**，**绝不升级成事实**。
>
> ⚠️ **一处日期口径差异（必须知道）**：`tilted_lidar_fidelity.md` 内部把 §I/§J 标为 "2026-10-09"、
> §K/§L/§M 标为 "2026-10-10"，但对应的提交（`a7497f0`、`2a5a8b6`、`18f2738`、`de3b78c`、`ef927e2`）
> 在 `git log` 里都是 **2026-10-07 / 2026-10-08**。**本文的时间线一律以 git 日期为准**，
> 引用时保留原文档的 `§` 编号（它比日期更稳定）。

---

# §0 一页速查（给"没时间"的人）

## §0.1 帧语义：谁发哪条边、谁是"假"的

**唯一合法的帧树（`tf_interface_contract.md` §三）**：

```
map ──(重定位：amcl | slam_toolbox-local | icp | gicp | small_gicp | cartographer)──► odom
      ──(里程计：LIO；或 lio:=cartographer 全包)──► base_link
      ──(URDF 固定关节，robot_state_publisher)──► livox_frame / imu_link / l2..l11 …
      ──(fake_vel_transform，20 Hz，哨兵云台机制)──► base_link_fake
```

| 帧 / 边 | 谁发（唯一发布者） | 语义要点 | 依据 |
|---|---|---|---|
| `map→odom` | **重定位模块之一**（互斥） | 全局锚定；**连续 25 帧被拒 ⇒ 停发**（宁可不发也不编造） | `gicp_divergence_and_jitter.md` §3.3 |
| `odom→base_link` | `lio_tf_adapter`（`lio:=fastlio\|pointlio`）／`small_point_lio` 自身／cartographer（`lio:=cartographer`） | **同一条边只能一个发布者**；Gazebo 真值 `publish_odom_tf=false` + remap 到 `/odom_ground_truth` | `tf_interface_contract.md` §一/§三、`issues_and_findings.md` #5 |
| `base_link→base_link_fake` | `fake_vel_transform`（20 Hz） | **不是脏帧**：哨兵云台载体；nav2 的 `robot_base_frame` 就是 `base_link_fake`，`/cmd_vel` 再旋到 `/cmd_vel_chassis` 下发。**T6"改回 base_link"已撤销** | `tf_interface_contract.md` §五 T6 说明 |
| `base_link→livox_frame` / `→imu_link` | `robot_state_publisher`（URDF 固定关节） | **矩阵值随 `robot:=` 槽位变**（3 档：`plugin`/`urdf`/`sensor`） | `tilted_lidar_fidelity.md` §C.1/§J.6.1 |
| `camera_init→body`、`aft_mapped*` | LIO 内部帧 | **不再泄漏到导航层**（T1–T5 已实施） | `tf_interface_contract.md` §五 |
| `/odom_ground_truth`（话题） | Gazebo `planar_move` 插件 | 真值里程计；**`.twist` 是指令回显，不是实测速度** | `tilted_lidar_fidelity.md` §M.9 ② |
| `map` 系原点 | — | **= 出生点相对系**（`(0,0,0)` = 出生点），不是场地绝对坐标 | `map_assets.md` §1、`continue_mapping.md` §0 |

### ★ "三个 `livox_frame`"——本工作期最贵的一课（`tilted_lidar_fidelity.md` §I.2）

同一个名字 `livox_frame` 在本仓被**三样不同的东西**共用，**混起来就会得出"物理错了"的错误结论**：

| # | 它是什么 | 由谁决定 | `plugin` 档 | `urdf` 档 | `sensor` 档 |
|---|---|---|---|---|---|
| ① | **物理传感器系**（实物安装姿态） | `body_to_livox` 关节 rpy | `0 0 0`（帧正） | `−30°`（真的斜） | `−30°`（真的斜） |
| ② | **点云数据实际表达的系** | 插件出点公式 `axis = sensor_rot·mount_rot_·ray`、原点 = 传感器原点 | **水平**（= 父 link 朝向） | **还是水平**（倾角没进数据！） | **斜 30°**（`<cloud_frame>sensor</cloud_frame>`） |
| ③ | **`header.frame_id` 字符串** | `<sensor name="livox_frame">` | `livox_frame` | `livox_frame` | `livox_frame` |

**判据（一眼分辨，不用猜）**：对点云做 RANSAC 地面平面拟合，看**法向与"它自称的那个帧"的 z 的夹角**：

* ≈0° ⇒ 帧与数据一致（`plugin` 实测 **1.054°**；`urdf` 也是 1.02°，**但那个帧自己斜了 30°** ⇒ 不自洽）；
* ≈30° ⇒ 数据真的在斜的传感器系里（`sensor` 实测 **29.092°**），此时**必须**经 TF 转到 `base_link` 才是重力对齐（实测 **1.035°**）。

**推论（§I.6 的 RViz 配方）**：`urdf` 档在 RViz 里"看着像平放扫到的东西被倾斜了"是**字面正确**的
—— 数据真的水平（两档传感器系点云刚体配准 **0.0004°** / 残差 p50 **0.074 mm**），斜的是那个帧；
**没有任何一个 `Fixed Frame` 能让"点云"和"车/代价图"同时看起来正**（这本身就是"帧与数据差 30°"的定义）。

## §0.2 两条代价图链（全局 `map` / 局部 `odom`）+ 3D→2D 的"高度信息死在哪一级"

```
/livox/lidar/pointcloud ──►[①地面分割 ground:=linefit|patchwork]──► /segmentation/{ground,obstacle}
        │                              │
        │                              └─[★判据层 rm_ground_traversability]（同一进程内，只降不升）
        │                                    · 坡度/台阶判据 ⇒ ground 降级成 obstacle
        │                                    · 自击掩膜 self_mask_*（默认关；robot11 开）
        │                                    · 近地剔除 obstacle_near_ground_m（默认 0；robot11 = 0.05）
        │                                    · 走廊连续量 ⇒ SlopeSpeedLimiter ⇒ /speed_limit
        │
        └──►[②pointcloud_to_laserscan]──► /scan（LaserScan：**没有 z**）
                                              │
                                              ├─►[③nav2 local_costmap/obstacle_layer.scan]（帧 = odom）
                                              │     projectLaser（z 强行置 0）→ TF 到代价图帧
                                              │     高度带 min/max_obstacle_height **在本帧里量**
                                              │
                                              ├─►[④nav2 local/global STVL（voxel）]（3D 点云直接进）
                                              │
                                              └─►[⑤inflation_layer]──► 代价梯度 ──► planner / controller

全局另一条腿：map/*.pgm|yaml ──►[⑥static_layer（map_server，先验图）]──► global_costmap
              PCD/*.pcd ──(pcd_to_nav2_map.py，离线)──► map/*.pgm|yaml（**2D 先验图从 3D 点云投影**）
```

| 环 | 在哪一节详述 | 本期改了什么（一句话） |
|---|---|---|
| ① 地面分割（linefit / patchwork） | 本文 §2.2.8；`ground_segmentation_slots.md` §3/§10/§11 | `th_dist 0.125→0.08`；接入"坡度/台阶"判据层；linefit 的 `gravity_aligned_frame` C++ bug 修掉 |
| ★ 判据层（坡度/台阶/自击/近地/限速） | 本文 §2.2.1/§2.2.7；`slope_speed_limiting.md` §1；`robot_models.md` §12.3 | 新增：连续量→速度上限（发 nav2 原生 `SpeedLimit`）、近地剔除、自击掩膜 |
| ② `pointcloud_to_laserscan` | 本文 §2.2.6；`ground_segmentation_slots.md` §11 | `max_height 0.1→1.0`（离地覆盖 **0.326 → 1.226 m**） |
| ③ `obstacle_layer.scan`（2D 链） | 本文 §2.2.1/§2.2.5；`tilted_lidar_fidelity.md` §K.1 | **结论是"不动"**：高度带在这条链上**没有逐点高度可判**（结构原因，见下） |
| ④ STVL（3D 链） | 本文 §2.2.9；`stvl_local_costmap.md` §2 | 局部代价图也上 STVL（槽位 `local_obstacle:=stvl`） |
| ⑤ `inflation_layer` | 本文 §2.2.2；`tilted_lidar_fidelity.md` §K.3 | robot11 槽位几何：`robot_radius 0.3565→0.300`（外接→内切）+ 膨胀 0.70/0.75→0.60/0.65 |
| ⑥ static_layer / 2D 先验 | 本文 §2.5.1；`map_assets.md` §3 | 默认 2D 先验提升到 `RMUC2026_v3` 会话产物（旧份留 `*.bak-20261007`） |

**★ 最反直觉、也最该记住的一条（`tilted_lidar_fidelity.md` §K.1）**：
**`obstacle_layer.scan` 的 `min/max_obstacle_height` 在这条 2D 链路上不是"障碍物高度闸"，而是"点云原点的代价图帧 z"的常量闸。**
因为 `LaserScan` 没有 z ⇒ `laser_geometry` 的 `projectLaser` 只能产出 `z ≡ 0` 的点，随后整体 TF 到代价图帧。
实测（robot11、静止）：**950 条波束的 odom z 全部落在 `[0.0564, 0.0815]`（跨度 25 mm），
落在这个带外的 = 0/950**，而同一跑 `odom→livox_frame` 的平移 z = **+0.064 m**（与 p50 吻合到 1 mm）。
⇒ **"按几何把高度带重新定基"在 2D 链路上没有可定的对象**（把闸门抬到任何值只是把整条平盘一起抬高）。
真正能用上"高度"的地方只有一处：**投影成 2D 之前**（判据层的"近地剔除"）。

## §0.3 参数归属表：每个高度/半径/距离参数**属于哪个帧**，写错会怎样

| 参数（键） | 住在哪个帧 | 默认 / robot11 取值 | 写错会怎样（**有实测的给数字**） |
|---|---|---|---|
| `linefit.sensor_height` | **传感器系**下"地面"的 z（= −离地高） | 0.226 / **0.2595** | 地面线整体偏移 h；robot11 用 0.226 ⇒ 偏 **3.35 cm**（`robot_models.md` §11.6） |
| `linefit.gravity_aligned_frame` | 分割前先把点云**只旋转**到这个帧 | `""` / `""`（robot11 仍空） | 本仓该键曾有 C++ bug：设 `base_link` ⇒ `/segmentation/ground` **恒 0 点**（已修，见 §2.2.4） |
| `p2l.min/max_height` | **点云自带帧**（`target_frame: ""` 时不建 TF） | −1.0 / **1.0**（原 0.1） | 旧值 ⇒ 离地 **>0.326 m** 的几何整条方位变 `inf` ⇒ 2D 图/代价图看不见护墙（`ground_segmentation_slots.md` §11） |
| `p2l.target_frame` | 是否把点云 TF 到别处再切片 | `""` / `""`（`sensor` 档 = `base_link`） | 一开就引入 TF/MessageFilter 故障面（2026-09-23 刻意避开）；`sensor` 档必须开 |
| `obstacle_layer.scan.min/max_obstacle_height` | **代价图帧**（local = `odom`） | 0.0 / 2.0（**本期不动**） | 按"障碍物高度"去调它 = 调错对象（§0.2 末）；`sensor` 档实测 **41.2%** 波束被丢 |
| `obstacle_layer.scan.obstacle_min_range` | 同帧，波束长度闸 | 0.1 | 挡掉 <0.1 m 自击波束（好）；同时"贴墙 0.1 m"也被挡（代价） |
| `obstacle_near_ground_m`（新增） | 判据层：`dz = z − 局部地面 g(x,y)` | 0.0（关） / **0.05** | 关 ⇒ 近场地面残留经 p2l 投成 lethal：车那格 **42（非 free）**、半径圆内 `≥99` **156**、free **239**；开 ⇒ **0 / 0 / 999**，到最近 lethal **0.531 → 1.534 m**（§K.2.3） |
| `self_mask_*`（新增） | **传感器系 + 重力对齐**：113 个 collision AABB ∪ 近场死区 | 关 / robot11 **开**（`r_xy ≤ 0.2416`、`z ≥ −0.2295`） | 关 ⇒ 近场自击格把"局部地面"钉成 **+0.000 m**（= 雷达自身高度）、"参考坡度"钉成 **0.0°**、静止也被压到地板 0.60 m/s（§12.3.3） |
| `local/global_costmap.robot_radius` | **它是 `inscribed_radius`**（本仓没有 footprint 多边形） | 0.22 / **0.300（内切）**（原 0.3565 外接） | 用外接值 ⇒ **全局**图上"车那格"被判 **99**、圆内 **0 格 free**（静止！）⇒ "车一开始就在膨胀团里"（§K.3.1） |
| `inflation_layer.inflation_radius` | 同帧；必须 ≥ `robot_radius` | local 0.5 / **0.60**；global 0.55 / **0.65** | 软带厚度决定"贴边"程度；`cost_scaling_factor` 与半径无关（本期不动） |
| `speed_limit_lookahead_m` | 传感器系的**朝向走廊**（帧 rpy=0 ⇒ +x = 车头） | 3.0（不改） | 再大也没用：平地最近只看得见 **1.78 m**（上坡面 ~2.6 m）（`slope_speed_limiting.md` §4） |
| LIO `extrinsic_T/R` | **点云实际表达的系**，不是"实物几何" | `[0,0,0.05]` / `I`（**三份配置一个字节没改**） | 光改外参补不了"点云不在 `livox_frame` 里"这个错（§I.8 原则 3） |
| `lio_tf_adapter.xyz/rpy` | `T_base_link←body` 的杆臂（`body` = 雷达/IMU 位置） | `[-0.12,0,-0.125]` / 按槽位算 | 丢了杆臂 ⇒ 原地转时"地图绕车画圆"（`issues_and_findings.md` #18） |
| 插件 `<tilt_rpy>` | 安装倾角（射线方向；点云仍表达在父 link 系） | 缺省 = 单位阵（其它模型逐字节不变） | 与关节 rpy **二选一**；两处都写 ⇒ 转 60° |
| 插件 `<cloud_frame>` | 点云表达在 `parent` 还是 `sensor` 系 | 缺省 `parent` / `sensor` 档 = `sensor` | 选 `sensor` 而下游不做重力对齐 ⇒ 地面在 odom/图里是 **30° 斜坡** |
| robot11 几何 | `livox_frame` z = **0.157028**；地面 = **−0.102499** ⇒ 雷达离地 **0.2595**；足印内切 **0.300** / 外接 **0.3565**；轮 r=0.058 | — | 这些数**是从 mesh 顶点 + 零位 FK 量出来的**（`robot_models.md` §9.4），不是抄的 |

## §0.4 现在能用的 / 还不能的（一句话一览）

| 能力 | 状态 | 证据 |
|---|---|---|
| 默认模型（`robot` 留空）全链路 | ✅ 能用；**本期没有改它的任何默认路径**（每个开关默认关 / 每处覆盖只在槽位文件里） | `robot_models.md` §11.2.1、§12.6、§J.2 |
| `robot:=robot11` 建图 / 感知 / 状态估计 | ✅ 雷达看得见（地面 **5689~6641 点/帧**）、`/odom` 10 Hz、RTF 0.32~0.46 | `robot_models.md` §11.2、§13.8 |
| `robot:=robot11` + GUI | ✅ 在"虚拟屏 + 软件 GL"下两档都活满窗口、都在画（`gzclient` 473 MiB、`gzserver` 3202 MiB **与视觉档位无关**） | §13.4.2 |
| `robot:=robot11` 短目标导航（≲0.4 m） | ✅ 能走到（`--goal-forward 0.5`：真值走 **0.2977 m**、残余 **0.1902 m** < 容差 0.25） | §L.7 |
| `robot:=robot11` 长目标 / 需要转向的目标 | ❌ **两个独立原因**：① 该出生点正前方 **0.42 m 就是实体障碍**（物理不可达）；② 反向 2.0 m 需要"先转头再走" ⇒ 依赖角速度通道，修前只执行 **0.55%** | §L.0、§L.2、§L.5 |
| 角速度通道（修后） | ✅ 整栈盲发 `wz=1.0`×12 s：真值 **+35.42°**（修前 +3.79°，**9.3×**）、执行率 **0.55% → 5.15%**、60 s 自由漂移 **+8.20° → +0.006°** | §M.7 |
| `robot11_mount:=sensor`（物理斜装 + 账也对） | ⚠️ 帧↔数据自洽（29.028° vs 1.004°）、下游重力对齐生效；**但代价图那一层仍坏**（odom 斜 30° ⇒ 41.2% 波束被高度带丢） | §J.7 第 3 条 |
| 坡度/台阶前瞻限速 | ⚠️ 有效但**没根治**：首击 **110.8 → 50.8 m/s²**（目标 ≲30 未达）；提速档把"0/4 到点"变成 **2/2 到点** | `slope_speed_limiting.md` §0/§3、`lio_divergence_no_impact.md` §3.3 |
| 目标朝向 | ❌ **本仓 nav2 不检查朝向**：`goal_checker_plugins` = `PositionGoalChecker`（只声明 `xy_goal_tolerance`）⇒ "原地转 180°"会**立即 SUCCEEDED**（`max|wz| = 0.0`、Δyaw 0.005°） | §M.7、§M.9 ① |
| 2D 先验图的"坡道面" | ❌ 仍然看不见（这是 2D 先验的固有上限：全场台阶边沿覆盖率只有 **35.98%~54.42%**，判据是 80%） | `map_assets.md` §3.1、`traversability_plan.md` §9.1 |
| 落空/负障碍、净空（限高） | ❌ 没做（`traversability_plan.md` §9 明确登记；抬到 1.226 m 之后"头顶结构"会成为假障碍，换场地要重估） | `traversability_plan.md` §9 |

## §0.5 本期最该记住的五句话

1. **"CAD 网格能不能进仿真"要分两问**：**视觉**可以（但要按预算抽稀 + 量化验证），**碰撞绝对不行**（一接触就掉到 0.2 倍实时、常驻 +587 MB）。
2. **帧是一等公民**：同一份数据配上错误的 `frame_id`，会让"物理正确的模型"在 RViz/nav2 里看起来全错；**判据要用矩阵/一致性，不要用 rpy 分量**。
3. **"看起来像"的观感往往是"刚性旋转"**：地面变 30° 斜坡、环状图案不变 = 帧的问题；一侧彻底没有地面、最近地面环从 0.31 m 变成 1.96 m = 物理倾角。两件事可以**同时**存在。
4. **先量后改**：本期每一个"修"前面都有一次判决性测量；被否掉的候选（紧门限、1000 Hz 插件、角速度限幅、只放松回环阈值、footprint 多边形…）都留下了数字。
5. **默认路径逐字节不变 + 一键回退**：所有改动都做成"槽位/开关/覆盖文件"，默认关；每个改动都有一行回退配方（§6.3）。

---

# §1 时间线：阶段、关键提交、以及"我们推翻过哪些结论"

## §1.1 阶段总览（按 git 作者日期）

| 阶段 | 日期 | 主题 | 关键提交（示例） | 产出文档 |
|---|---|---|---|---|
| **P0 场地与建图地基** | 10-05 | `world:=RMUC2026`（STL→world/2D/PCD）、`lio:=small_point_lio` 打通与调参、定位槽位（beluga / icp / gicp / small_gicp）、nav2 槽位（`nav:=mppi`、`planner:=smac2d`）、构建口径修复 | `7c0006b` `280888e` `eb7b811` `98baa55` `2ff42d4` `21fc301` `2c42339` `6ae2ada` | `worlds.md`、`lio_slots.md`、`localization_slots.md`、`algorithm_matrix.md`、`build_optimization.md` |
| **P1 重定位 / 时间戳 / 建图稳健性** | 10-05~10-06 | `/initialpose` 饥饿修复、1 ns 戳截断修复、2D 先验改从点云投影、`ground` 槽位、可通行性设计计划、LIO 漂移分段诊断、slam_toolbox 节点加密与丢帧裁决、续建/存档/守卫 | `c7bcd6d` `a281636` `7afbe01` `1a9ce8c` `fc3382d` `e866cfa` `5cb93cb` `0a9c5b5` `10f2a84` | `gicp_initialpose_latency.md`、`timestamp_construction_audit.md`、`mapping_2d_from_cloud.md`、`ground_segmentation_slots.md`、`traversability_plan.md`、`lio_drift_diagnosis.md`、`slam_toolbox_tuning.md`、`continue_mapping.md` |
| **P2 模型槽位（`robot:=`）** | 10-07 | opt-in 槽位机制、用户那份哨兵 URDF（`hzmirm`）逐字接入 + 补件、"0.8 m + −7.22° ⇒ 6.32 m 近场盲环"实测 | `d3940b5` `d6232ea` `a2c6349` `712741f` | `robot_models.md` §1–§8、`params_ownership_checklist.md` |
| **P3 `robot11` Phase 1–2** | 10-07 | 12 个 STL 打包与逐 mesh 量测、`base_link` 碰撞策略（ODE 代价实测）、槽位接入、**30° 是 roll 还是 pitch 的实测判定**、"雷达装在底盘凹槽里" | `f361f00` `599fc09` `0142bd9` | `robot_models.md` §9–§10 |
| **P4 Phase 3：让雷达看得见** | 10-07 | A 方案（碰撞挖视锥 = 76 个 box）、根 link 改名 `body→base_link`、插件新增 `<tilt_rpy>`、斜置感知链槽位化、nav2 几何覆盖层 | `d4e8232` `82e4946` `a1d9c15` `23c1dc8` | `robot_models.md` §11 |
| **P5 Phase 4–5：GUI / 自击 / 视觉件** | 10-07 | 自击掩膜、横幅改成"运行时读生效值"、`robot11_visual` 档位、**QEM 视觉 LOD（2,821,320 → 163,995 面）** | `1ca09eb` `cf60b84` `646277f` `4ec728e` | `robot_models.md` §12–§13 |
| **P6 斜装逐帧追查（§A–§I）** | 10-07~10-08 | 逐帧点账本（"有没有被裁"的定论）、代价图/膨胀半径归因、`robot11_mount:=plugin\|urdf` 开关、**三个 `livox_frame` 的判决**（H1/H2/H3）、Gazebo 侧 mesh 姿态直接读数、`model://` 解析自动化、gzclient 卡住取证 | `6c9ae96` `b7d625b` `a7497f0` `527fc10` `e2bcf3d` `96974db` `3b48c7c` | `tilted_lidar_fidelity.md` §A–§I、`gazebo_gui_troubleshooting.md`、`robot_models.md` §14–§15 |
| **P7 两个 LIO bug + `sensor` 档（§J）** | 10-08 | bug ②（`/cloud_registered` 多乘一次 TF）、bug ③（`odom→base_link` 用共轭发姿态）、新增 `robot11_mount:=sensor`（物理斜装 + 账也对） | `2a5a8b6` `1116b05` `7542973` | `tilted_lidar_fidelity.md` §J |
| **P8 代价图侧收尾（§K）** | 10-08 | 近地剔除（`obstacle_near_ground_m`）、`robot_radius` 外接→内切、插件 `point = range·axis` 漏起点修复、高度带"为什么不动" | `360d493` `1c89473` `1403f42` `18f2738` | `tilted_lidar_fidelity.md` §K |
| **P9 长目标归因 + 角速度修复（§L/§M）** | 10-08 | 六个候选逐条判决（根因 = 实体障碍 0.42 m）、可达性前置判据/闸门、**底盘塌成单刚体（角速度 1.03% → 16.2%）** | `94f0d6b` `de3b78c` `ef927e2` `9de6b72` `d7c4da0` `d3034aa` | `tilted_lidar_fidelity.md` §L–§M |
| **P10 撞击 / 贴边 / 限速** | 10-06~10-07 | 贴膨胀边→撞击→定位失效的链条归因、`w_smooth 0.4→0.0`、实时坡度/台阶判据、前瞻限速（含提速档）、"没撞击却飘"归因 | `1811149` `4a7996c` `6f0c27a` `ca89b69` `4adf0d8` `dad39c5` | `path_clearance_and_contact.md`、`slope_speed_limiting.md`、`lio_divergence_no_impact.md` |
| **P11 研究（未落地）** | 10-08 | 纯 C++ 规划库（`tdt-nav-kit`）接进 nav2 的三种形态、契约、效率账、落地坑 | `d4965bd` | `nav2_vs_custom_planner.md` |
| **P12 资产提升与规范** | 10-07 | 地图/先验资产规范、逐个清单、2D/3D 默认对提升到 `RMUC2026_v3`、退役资产进 `attic/` | `95846c3` `7f9e354` `2192024` `c8d65f9` | `map_assets.md`、`continue_mapping.md` §补 |

## §1.2 ★ 推翻过的结论："当时以为 → 后来的测量 → 更正"

> 这张表是本期最有价值的部分之一：**每一次更正后面都跟着一次判决性测量**。引用的 `§` 是原文档章节。

| # | 当时以为（错误认知） | 判决性测量（数字） | 更正后的结论 | 证据 |
|---|---|---|---|---|
| 1 | "点云少了很多，肯定被下游裁减了" | 一帧 30000 条射线里只有 **11960 条有回波（39.9%）**；`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered` 每帧点数 **11960 / 11960 / 11968** | **没有任何一级在裁**；规模由"有多少条射线真的打到东西"决定。少掉的是 Phase 2 的**假自击点**（75.5% → 28.5%） | `tilted_lidar_fidelity.md` §B |
| 2 | "URDF 里写的是 `rpy=0 -0.5236 0`（pitch），所以是 pitch" | MID-360 方位 360° ⇒ roll/pitch 的**射线方向集合完全相同**（仰角谱、地面环半径、盲区都一样）；唯一判据 = "最朝下的那一束指向 body 的哪个方位"（roll→±y 实测 75°/105°；pitch→±x 实测 165°/195°） | **仿真不能替用户决定哪个是真的**；默认取 **roll**（SolidWorks CSV 机器生成、15 位有效数字；URDF 那行只有 5 位）；用户后来**按实物确认 = roll** | `robot_models.md` §9.5/§10.5；`Fidelity` §I.3 |
| 3 | "近场点 100% 落在云台 `l10/l11` 上（中位距离 0.14 m）" | Phase 4 逐点复核：近场点**中位半径只有 0.021 m**（p95 0.048、max 0.121）；113 个 collision AABB 只"命中" **3 个点** | 真因是**插件 `point = range·axis` 漏了射线起点**（`start = minDist·axis`，minDist = 0.1 m）⇒ 每个点朝传感器内移 **0.1 m**；独立证据 = 地面点 z 随距离单调变化（r 0.25–0.35 → −0.208；r 2–4 → −0.253；几何真值 −0.2595） | `robot_models.md` §12.3.1；`Fidelity` §K.4.3 |
| 4 | "`hzmirm` 的地面分割全废 = 几何原因（0.8 m 高 + 下视 7.22° ⇒ 6.3 m 盲环）" | 几何那条**仍然成立**；但该槽位的 YAML 同时开了 `gravity_aligned_frame: "base_link"` ⇒ **`/segmentation/ground` 恒为 0** 里有一部分是这个 **C++ bug** | 归因要按"**几何 + bug**"两条并列重读 | `robot_models.md` §11.4 |
| 5 | "LIO 的 `odom` 帧被转了 30°（缺一条 LiDAR→IMU 外参，H1）" | 把 `/cloud_registered` 按 `T(base_link←livox_frame)` **反变换**回去：地面 **30.970° → 1.041°**；从发布的 `odom→base_link` **反解 LIO 状态** `R_ol` = `[0.086, 0.405, 9.105]°`（只有 yaw） | **odom 没有被转 30°，它水平到 0.1~0.4°**；30° 是**发布环节多转的一次**（bug ②） | `Fidelity` §I.1/§I.5 |
| 6 | "地图跟着车转 = 位姿图回环门槛太松，收紧就行" | 收紧后收尾日志变成 `Score histogram: Count: 0`（**一条回环都没命中**）⇒ 位姿图完全没有全局锚定 | **方向错了**；真因是 ① 插件**混用两套时间基**（`header.stamp` 仿真钟 / 逐点 `offset_time` 墙钟）② 先验给错源（喂 LIO 自己的 `/odom`：同源 + 晚到）⇒ 换成**独立底盘里程计** | `issues_and_findings.md` #19/#20/#21 |
| 7 | "本仓底盘对 yaw 有巨大死区：要 1.9 rad/s 得给 6 rad/s" | 用**同一行内的"指令/真值"配对**重测（n=75~109、符号一致率 1.00）：指令 0.34→实测 0.36、0.65→0.67、1.07→1.11、**2.83→2.83**（原地） | **≈1:1，底盘听得懂 yaw 指令**；旧表作废，**"要 1.9 得给 6"不要再引用** | `lio_drift_diagnosis.md` §2.3 的 2026-10-07 更正；`lio_divergence_no_impact.md` §2 |
| 8 | "`/scan` 丢帧是刚打开的**回环**（节点加密）造成的" | 同一份**完美里程计**（ATE = 0.000 m），只把 TF 晚发 **130 ms** ⇒ 丢帧 **0 → 62（4.26%）**；六格里**没有一条丢帧落在回环 ±2 s 内**（随机基线 1.5%） | 回环是**放大器不是开关**；直接因 = "这一帧的 `odom→base_link` 比它自己晚到、且晚过下一条 `/scan` 的到达间隔" | `slam_drops_loopclosure_odom.md` §0 |
| 9 | "把 `scan_queue_size` 调大就能消掉丢帧日志" | 队列 `2` 把 `queue is full` 从 **62 → 0**，但同一跑出现 **70 条另一个原因**的丢帧（`timestamp earlier than all the data in the transform cache`），位姿图节点数 **58 → 35** | **丢帧总量没减少，只是换了个名字** ⇒ 不推荐；正解是修根因（LIO 戳对齐） | 同上 §0 末 |
| 10 | "车 90 s 没动 = 地图没建好 / 规划器有问题" | 把 nav2 **完全摘掉**、只给底盘插件固定 `vx`：robot11 只能前进 **0.4199 m**（请求 6.25 m），默认模型 **0.5402 m** | 根因是**场地里 0.42 m 处有实体障碍** ⇒ `--goal-forward 2.0` **物理上不可达**；控制器侧那些日志都是**结果** | `Fidelity` §L.0/§L.2 |
| 11 | "短目标能走 ⇒ 长目标只是慢/更长" | `--goal-forward 0.5` 真值只走 **0.2977 m**、残余 **0.1902 m** < `xy_goal_tolerance 0.25` ⇒ `Goal succeeded` 是**容差成功**；且 §K.5 表里"0 条 `collision ahead`"**只对短目标成立**（长目标是 **135 条**） | 两件事要分开：短目标是"落在容差里"，长目标是"目标在障碍后面（不可达）"；**口径更正**已写进 §L.1.3 | `Fidelity` §L.1.3/§L.7 |
| 12 | "`gt_yaw_delta_deg` 报 473.416 / 251.651 度" | 真值其实是 **8.26° / 4.39°** | **工具 bug**：`_xy()` 的 yaw 已是度，代码又 `math.degrees()` 了一次 ⇒ 全部多乘 **57.2958**；已修，旧读数不要再引用 | `339e205`；`Fidelity` §L.1.3 |
| 13 | "robot11 角速度只执行 1.03% = 单位/缩放/mixing bug" | 源码级：`planar_move` 的 `OnUpdate` 就是 `model_->SetLinearVel/SetAngularVel`，而 `Model.cc:746-771` 对**每个 link 各设一遍同一个 `(v,ω)`**；同一插件在"极简单 link box"上能做到 **99.9%** | 通道本身没坏；根因是"**驱动器与模型结构不自洽**"（多刚体 + 关节树） | `Fidelity` §M.2/§M.3 |
| 14 | "把插件 `update_rate` 提到 1000 Hz 就能修角速度" | 角速度确实 1.03% → **7.8%**（台架 +53.80°），**但**纯物理正前方盲推从 **0.4077 m → 2.5652 m**、底盘 z 从 **0.1522 → 0.4005 m**（**爬上了实体障碍**），60 s 漂移 0.308 → 0.297°/s（没解决） | **不采用**（这是"必须做反向验收"的教科书例子） | `Fidelity` §M.5 |
| 15 | "抽稀视觉件能省下 GB 级 GUI 内存"（Phase 4：rviz2 **1381 → 243 MiB**） | Phase 5 在**同一脚本/世界/flags/虚拟屏**下重跑：`full` 314/322 MiB、`decimated` **314 MiB**，逐秒曲线形状**几乎完全相同** | **本相位没有测出这个收益**；三条可能原因已列但**没有定论**；抽稀**确凿**省下的是三角形（17.2×）、磁盘（141.07 → 8.20 MB）、网格解析/上传/绘制的工作量 | `robot_models.md` §13.4.3 |
| 16 | "`gzclient` 被 `exit code -9` = 内存不够（OOM）" | 用户机 `MemTotal 31.1 GiB`、内核日志**无 OOM 记录**、会话 cgroup `memory.events` 的 **`oom_kill = 0`**；而"GUI 卡住不出来"另有真因：`model://` 解析不到 ⇒ 回落在线模型库**同步阻塞**（实测 **48.03 s**，且是被 Ctrl-C 打断的） | `-9` 大概率是本仓 bench 脚本的**全机 `pkill -9 -x gzclient`**；卡住 = mesh 解析 + 在线模型库 | `gazebo_gui_troubleshooting.md` §0 |
| 17 | "回退档**逐字节相同**" | 精确核对的结论是：**模型体（去掉注释后）逐字节相同**（两份都是 35025 B / 35410 B，`diff` 为空），整份文件多出的是文件头说明 | 表述改成"模型体（去注释）逐字节相同" | `d3034aa` |
| 18 | "`/get_entity_state` 能拿到 Gazebo 眼里的 link 姿态，只是被 grep 过滤掉了" | `ros2 service list` 里 **0** 个 `/get_entity_state`；本仓 gzserver 只加载了 `libgazebo_ros_{init,factory,force_system}.so`，**没有** `libgazebo_ros_api_plugin.so` | "那个服务根本不存在"；改用 `gz model -m robot -i`（Gazebo transport，不经 ROS）拿到**直接读数** | `Fidelity` §H.4 |
| 19 | "`/odom_ground_truth.twist` 是实测速度" | 真值 yaw 12 s 只转了 **7.09°**，而该字段中位 = **57.296°/s = 1.0 rad/s**（= 刚设进去的指令值；`OnUpdate` 先 `SetAngularVel` 再在同一回调里读 `WorldAngularVel`） | **它是指令回显**；要量"到底转了多少"只能用**位姿差** | `Fidelity` §M.9 ② |
| 20 | "本仓 nav2 会检查目标朝向" | `goal_checker_plugins: ["general_goal_checker"]` = `PositionGoalChecker`（只声明 `xy_goal_tolerance`；1.1.20 里没有 `yaw_goal_tolerance`）⇒ "原地转 180°"**立即 SUCCEEDED**（`max|wz| = 0.0`、Δyaw 0.005°） | **不查朝向**；要真判转向得换 `SimpleGoalChecker`（本期**没改**，参数在别的任务目录） | `Fidelity` §M.9 ① |

---

# §2 逐项改动详述（主体）

## §2.0 读法与统一模板

每条改动都按同一个模板写（缺项写"文档未给"）：

> **现象**（用户看到 / 日志看到）→ **原因**（机理，必要时给公式 / `file:line`）→
> **过程**（怎么查的：**判决性测量**、被否掉的候选）→ **改动**（文件 / 参数 / 生成物 / 提交）→
> **原理**（为什么这样修是对的、通用教训）→ **效果**（before → after 的**实测数字**）→
> **回退**（一键怎么做）→ **证据**（原文档 `§` / 提交）

分层顺序：**模型层 → 感知/代价图层 → LIO 层 → 导航/限速层 → 资产/建图层 → 工具与方法论 → 研究结论**。

## §2.1 模型层（CAD → 仿真件、碰撞/视觉分离、model://、斜装、自击、单刚体）

### §2.1.1 CAD 网格**不能**直接当仿真件（碰撞与视觉要分开处理）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「机器人模型放不进去仿真有啥意义」；带 GUI 时 `gzclient` 消失 / rviz2 黑屏 5 s 退不掉；车"在 Gazebo 里没有" |
| **原因** | 上游给的 12 个 STL 是**给人看/给机床用**的档：整车 **2,821,320 面 / 141.07 MB**，其中 `base_link.STL` 一项 **2,078,226 面 / 103.91 MB**（99.1 MiB）。放进 ODE 的 trimesh 里，**ray-vs-trimesh 求交代价是 O(三角面数)**，碰撞每帧都要做；视觉件每帧都要渲染。**这是"用途不匹配"，不是"慢一点"** |
| **过程（判决性测量）** | Gazebo Classic 实测（极简世界、ODE quick/50、`max_step_size 1 ms`、单模型、无头隔离）：**A 原网格**：悬空 RTF 1.002、峰值 RSS **760 MB**；**A2 原网格 + 接触**：RTF **0.200**（最低 **0.04**）、窗口内只推进 **0.49 s** 仿真时间；**B 4 个 box**：1.002 / **174 MB**；**C 抽稀 3000 面**：1.002 / 173 MB；**E 无底盘基线**：172 MB ⇒ **常驻 +587 MB 与有没有接触无关** |
| **改动** | ① 视觉：保留原网格（`<visual>` 零损失）→ Phase 5 换成**专用 QEM 视觉 LOD**（见 §2.1.2）；② 碰撞：`base_link` = **4 个 DP box**（`--band 0.005 --merge-tol 0.012` 的最优 4 段）；轮 = `cylinder(r=0.058, l=0.0452)`；其余 mesh → 包围盒 box。产物 `inventory/collision_assets.json`＋`meshes/generated/*_collision.stl`（备选） |
| **原理** | **"视觉/碰撞/感知"三件是三种不同用途的几何，允许而且应该不同**（SDF 规范明文："simpler collision models are often used to reduce computation time"；Gazebo 官方："simple shape **which is preferred**"）【来源】。反过来：**别用"稳态悬空 RTF 正常"当作"原网格能用"的证据**（A 的 1.002 是因为底盘离地 54.6 mm、压根没有接触对被求值） |
| **效果** | 4 box 的客观保真度：**覆盖率 1.000**（1605/1605 体素，真网格表面没有一个点在 box 外 ⇒ 不漏撞）、box 总体积 **0.065121 m³**（只比凸包胖 **9.4%**）、幻影体积占比 **16.36%**（pitch 30 mm）、最大内间隙 **69.3 mm**。RSS 760 → **174 MB**；接触工况 RTF 0.200 → **1.002** |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision boxes`（回 Phase 2 的实心 4 box）／`--body-collision mesh`（抽稀网格碰撞，**未验**） |
| **证据** | `robot_models.md` §9.3（表）、§11.3 第 1 项、§13.2.1【来源】；提交 `f361f00` `d4e8232` |

### §2.1.2 视觉件的 QEM 抽稀：**2,821,320 → 163,995 面**（Phase 5）

| 项 | 内容 |
|---|---|
| **现象** | 同上（GUI 卡）；Phase 4 的 `decimated` 档当时指向的是**碰撞档抽稀件**（`l11` 只有 1,200 面 ⇒ 云台/发射机构糊成一团） |
| **原因** | 原始面数不是"每帧渲染"的档；标准做法 = **按预算抽稀 + 量化验证**（不是"换个工具就信"） |
| **过程（判决性测量）** | ① 先查标准做法（带 URL）：Blender/游戏与 Gazebo 侧"每 link ≤50k、整车 ≤200k"、`<50k`/`≤100k` 两篇独立来源取严；② 抽稀工具逐个试：**MeshLab `meshlabserver` 在本机无 GL，`GLEW initialization failed` 直接死**；`trimesh.simplify_quadric_decimation` 缺 `fast_simplification`；`open3d`/`pymeshlab`/Blender 都没装 ⇒ **VTK `vtkQuadricDecimation`（pyvista 0.46.4）采用**；③ 用"边界边不增长 + 三视剪影 IoU + 单向表面误差"三条把它**量出来** |
| **改动** | `tools/scripts/regress/robot11_decimate_visuals.py`（生成 + 验证 + 清单）；产物 `meshes/decimated/*.stl`（12 个 / **8.20 MB**）+ `inventory/visual_decimation.json`；`robot11_visual:=decimated`（**默认**）\| `full`；xacro 由**生成器拥有** |
| **原理** | 三条通用教训：① **抽稀要按"预算 + 容差"，不是按"工具身份"取信**；② 误差口径要**说清偏低/偏高**（本工具默认采样口径**偏低 ≤0.22 mm**，是设计如此：宁可不虚报）；③ **外观检查 ≠ 感知检查**（Gazebo 射线传感器走 ODE、**只与 `<collision>` 求交** ⇒ 视觉件抽稀**不可能**改变点云；"雷达凹槽还在不在"要单独量） |
| **效果** | 面数 **2,821,320 → 163,995（−94.2%）**、磁盘 **141.07 → 8.20 MB**；逐 mesh 全部过容差（最差包围盒差 **0.297 mm**、最差 p99 **0.18 mm**、最差剪影 IoU **0.9919**、12/12 过）；`base_link` 自击率 `t<0.12 m`：原件 0.3833 → 抽稀 **0.3810**（Δ**−0.23 pp**，凹槽没被抹平） |
| **回退** | `robot11_visual:=full`（回上游原始 STL）；或删 `meshes/decimated/`（但默认档会加载失败） |
| **证据** | `robot_models.md` §13.1/§13.3/§13.5；提交 `646277f` `4ec728e` `bf0353b` `15175ea` |

### §2.1.3 碰撞 **76 个 box 的由来**（让射线"挖"出视锥）+ 根 link 改名

| 项 | 内容 |
|---|---|
| **现象** | Phase 2 接入后：**75.5% 的点是 `r<0.12 m` 的近距回波**、`/segmentation/ground` 只剩 **529 点/帧**（默认模型 2626）、**LIO 连 `/odom` 都没发出来**（有发布者、0 条消息） |
| **原因** | 两条独立的根因：① `body` 的 4 个 DP box 是**实心包络**，而雷达原点 z=0.157 **正好在第 4 个 box 内部**（box 顶 0.213）⇒ 每条朝下的射线**先打盒内壁**；② 模型根 link 叫 **`body`**，而全链路契约帧名是 `base_link`（`small_point_lio` 源码硬编码 `lookupTransform(lidar_frame,"base_link")`）⇒ **TF 断成两棵树**，日志逐帧 `Failed to lookup transform from base_link to livox_frame: ... not part of the same tree` |
| **过程（判决性测量）** | ① 离线 30000 条与 Gazebo 同源射线求交：**80.5% 落在 <0.12 m、看见地面 0%**（与 Gazebo 实测 75.5% 同量级，差 = Gazebo 丢掉 <0.1 m 回波）；② `/odom` 消息数：Phase 2 = **0**、A 方案 = **188** |
| **改动** | ① `body` 碰撞 = DP box **减去雷达视锥**（锥半角 **100.22°**、半径 **0.5 m**、内清空 0.05 m）后分解成 **76 个 box**；云台 `l10/l11` = 7/22 个细 box；② 生成时把根 link 改名 `body → base_link`（**上游 SolidWorks CSV 里本来就叫 base_link**） |
| **原理** | **"碰撞体的形状"要服从"传感器必须看得见"这条功能约束**，而不只是"包住几何"；**最小包围 ≠ 可用碰撞**。改名这条是"**契约帧名是硬编码的，模型必须服从契约**" |
| **效果** | `robot11` 从"看不见"变成：地面 **5637 点/帧**（默认模型 2739）、点/帧 11951、`/odom` 188 条、`/scan` >4 m 波束 **≈81/帧**（Phase 2 = 0）；自击 75.5% → **29.0%**（剩下的 29% 是**物理真实**的云台回波，中位距离 0.14 m，真车 360° 雷达也会照到） |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --body-collision boxes`（回实心 4 box ⇒ 立刻变回"看不见"） |
| **证据** | `robot_models.md` §11.1/§11.2/§11.3 第 1–2、9 项；提交 `d4e8232` |

### §2.1.4 `model://` 解析 + "等在线模型库"的同步阻塞（48~100 s 卡顿）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「gazebo 加载不出来吗，rviz 倒是挺好的」；日志 `[Msg] Waiting for model database update to complete...` 之后长时间无输出，随后 `[Err] [Visual.cc:2956] No mesh specified`（车在 Gazebo 里**没有视觉**） |
| **原因** | URDF 的 `package://robot11/meshes/decimated/<link>.stl` 会被 sdformat 在 URDF→SDF 时改写成 **`model://robot11/...`**，而 gazebo 的 `model://` 解析根只有 `$HOME/.gazebo/models` + `GAZEBO_MODEL_PATH`；**解析不到时 `SystemPaths::FindFileURI()` 无条件回落到在线模型库 `http://models.gazebosim.org/` 并同步阻塞**（实测 stall **48.03 / 76.34 / 99.67 s**，不设上限） |
| **过程（判决性测量）** | ① 用户 19:50 那次日志：`t_cloud_stop − t_click` 不是问题，但"等模型库"实测 **48.03 s 且是被 Ctrl-C 打断的**（⇒ 再等也不会好）；② 先怀疑网络 ⇒ **受控实测把代理做成黑洞**，结论与直觉相反：**不是网络慢，是它一定会等**；③ 排除 OOM（见 §1.2 第 16 条） |
| **改动** | **两条腿**：**(a) launch 侧** `_gazebo_model_path_setup()`（`OpaqueFunction` + `AppendEnvironmentVariable`，排在 include gzserver/gzclient **之前**）⇒ 只对 `robot:=robot11` 执行；**(b) 包侧** `package.xml` 的 `<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` ⇒ 任何 gazebo 入口都吃得到。两者**各自单独就够** |
| **原理** | ① **"解析不到就上网找"是 gazebo 的设计**，所以"本地一定解析得到"是使用方责任；② 修法要**分作用域**：launch 侧对其它槽位 0 动作 0 日志，包侧是**全局 env 增量**（⚠️ 这是本期唯一的"不是零改动"处） |
| **效果** | 改造前 vs 现在（同一条命令、无任何手工环境变量）：「等在线模型库」**出现、stall 51.64 s → 没有**；`No mesh specified` **34 → 0**；gzclient 事件数 **275 → 27**；轻量入口的 `rchar` **4.21 → 12.47 MB（+8.26 MB ≈ 12 个 mesh 的 8.20 MB）** —— 这条"+8.26 MB 且 fd 扫描抓到 `.stl`"才是"解析链真的通了"的正证据 |
| **回退** | 删 launch 里 `ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))`（启动侧）／删 `package.xml` 那一行并重建（包侧）；手工 workaround 仍可用：`GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" ros2 launch …` |
| **证据** | `gazebo_gui_troubleshooting.md` §0/§3/§5.1/§5.1.6/§6.2–§6.4；`robot_models.md` §14；提交 `e2bcf3d` `96974db` `3b48c7c` `c7f57d8` |

### §2.1.5 雷达"斜 30°"的三种档位：`<tilt_rpy>` / `<cloud_frame>` / `robot11_mount`

| 项 | 内容 |
|---|---|
| **现象** | 用户：「雷达给我按我给你的放好…我要看到倾斜放置最原始的效果…点云少了很多…urdf/雷达/点云图像都是平放置的很奇怪」；后来（§I）：「点云看着像平放扫到的东西倾斜了，不是倾斜放置扫描到的东西」 |
| **原因** | **"倾角只能记在一个地方"**：记在**插件**（帧正、点云正、射线斜 —— 自洽但画不出斜的 mesh）**或**记在**关节**（帧斜、mesh 斜、射线斜 —— 但点云仍留在水平系）。两档的**世界射线逐条相同**（逐点差 ≤5×10⁻⁵ m），因为插件出点公式 `axis = sensor_rot·mount_rot_·ray` 里 `sensor_rot` 会跟着 link 姿态走 ⇒ 两种乘法乘出**同一个矩阵** |
| **过程（判决性测量）** | **H1**（odom 被转 30°）：反变换 `/cloud_registered` ⇒ **30.970° → 1.041°**；反解 `R_ol` 只有 yaw ⇒ **不成立**。**H2**（只斜了 label）：两档传感器系点云刚体配准 **0.0004°**、残差 p50 **0.074 mm**，而地面最近环 **0.3146 m** / 离地 0.2482 m ⇒ 最陡下俯 **38.27°**（水平安装不可能 < 1.96 m）⇒ **前半成立、后半不成立**。**H3**（右侧盲区 = roll 应有几何）：右侧两扇区 3586 点里 **0 个地面点**，最低仰角 **+21.89°/+19.67°**（解析下界 +19.11°）；左侧 2694 个地面点、最低仰角 **−36.87°**（下界 −37.22°）⇒ **成立，两档都是这样** |
| **改动** | ① 插件新增 SDF 参数 **`<tilt_rpy>`**（缺省单位阵 ⇒ 其它模型逐字节不变，`82e4946`）；② `robot11_mount:=plugin\|urdf`（`b7d625b`）；③ 插件新增 **`<cloud_frame>`**（缺省 `parent`；`sensor` ⇒ `axis = mount_rot·ray`）＋第三档 `robot11_mount:=sensor`（`1116b05`，`sensor` 档的 launch 只给它加 `gravity_aligned_frame: base_link` / `target_frame: base_link`） |
| **原理** | **五条原则**（`Fidelity` §I.8）：① 先分清**三个 `livox_frame`**（物理系/数据表达系/`frame_id` 字符串）；② 倾角**只能记一处**，然后把"点云表达 + `frame_id`"一起改到自洽；③ **外参要按"点云实际表达的系"给，不是按实物几何给**；④ **`odom` 不保证重力对齐**（`fix_gravity_direction: true` + 初始 `R=I` ⇒ odom ≡ 初始化那一刻的身体系）；⑤ **每个"高度"参数都属于某个具体的帧** |
| **效果** | 三档并排（静止、同世界/出生点）：`原始云地面法向 vs 自己的 frame_id` = **0.957° / 1.023° / 29.092°**；`vs base_link`（账对不对）= **0.957° / 30.920° / 1.035°**；`/scan` 波束在 odom 里超出 `[0,2] m` 的比例 = **0.000 / 0.000 / 0.404**；自击掩膜命中率三档都 **≈29.2%**（**不用重烘**：掩膜作用在旋转后的 `cloud_proc` 上，那份坐标两档逐点相同）；local lethal = **461 / 404 / 518**。⇒ **只有 `sensor` 档"数据在斜的传感器系里 + 账也对"两条同时成立** |
| **回退** | 去掉 `robot11_mount:=…`（默认 `plugin`）；只退"点云表达在传感器系" ⇒ 去掉 xacro 里的 `<cloud_frame>`；整套 ⇒ `git revert <对应 commit>` |
| **证据** | `tilted_lidar_fidelity.md` §A/§C/§I/§J；`robot_models.md` §15；提交 `82e4946` `b7d625b` `1116b05` `6c9ae96` `a7497f0` `527fc10` |

> ⚠️ **`urdf` 档坏在哪（逐条算式，`Fidelity` §C.4）**：① `/cloud_registered` 被多转 **30.72°**（64.6% 的点落到水平面以下）；
> ② `p2l` 高度带在 `urdf` 档"歪打正着仍然对"（点云没变），但 `/scan` 一旦被 TF 消费就变成**斜平面**
> `z_odom = −sin30°·r·sinφ = −0.5·r·sinφ` ⇒ `obstacle_layer` 把一半波束按高度丢掉（lethal 340 → 75）；
> ③ `linefit` 的 `sensor_height` 在那一档"幸运地对"——**是巧合不是设计**；
> ④ 自击掩膜的 113 个盒子是"帧正"假设下烘的，若将来点云真变成斜系表达**必须重烘**。

### §2.1.6 自击掩膜（`self_mask_*`）：把"近场自身回波"剔出限速判据

| 项 | 内容 |
|---|---|
| **现象** | 用户带 GUI 那次：**车静止**时限速被钉在 **0.60 m/s 地板**；日志 `why=step d=0.00 m 台阶≈0.164 m`（其中 9 条 `limit=0.60`）；`~/traversability_stats` 里"参考坡度"= **0.0°**（真值 ~2.9°） |
| **原因** | 雷达装在底盘凹槽里 ⇒ **26.9~29.0% 的点落在 `r ≤ 0.09 m`**（近场自击，是**物理真实**的回波）；它们在**前瞻走廊的第一个粗格**里造出 `d = 0.00`：**局部地面被算成 +0.000 m**（= 雷达自身高度！真值 −0.223 m）、台阶残差 **0.023~0.203 m**（帧间跳）、"参考坡度"被钉成 **0.0°** |
| **过程（判决性测量）** | ① 逐点复核：近场点中位半径 **0.021 m**、不在任何 collision 几何上（113 个 AABB 只命中 3 点）⇒ 与"内移 0.1 m"的插件 bug 吻合（§2.2.3 就是它的根因修复）；② 离线射线验证（`robot11_self_mask.py --verify`，30000 条）：**地面环（最近 0.3457 m）被掩 0 个** ✅、真 mesh 表面 <0.75 m 命中被掩比例 **0.2310 → 0.9992** |
| **改动** | `(a)` 本模型 12 个 link 的 `<collision>` 在 `livox_frame` 下的 **113 个 AABB**（逐个 ≥5 mm 膨胀，z 下限抬到地面以上 0.03 m）**∪** `(b)` 近场死区 **`r_xy ≤ 0.2416 m` 且 `z ≥ −0.2295 m`**；参数文件按 robot 槽位选（默认**关**、`robot11` 开） |
| **原理** | ① 掩膜**只作用于判据/限速**，**不动** `/segmentation/*` 的标签（动标签 = 改共享契约，影响默认模型）；② `(b)` 的上界是**几何硬约束**：下俯 30° + FOV 下沿 7.22° ⇒ 最低射线的地面交点在 **0.3416 m**，取 `0.3416 − 0.10 = 0.2416 m` 留余量；③ **两个掩膜盒子表是生成物**：模型改了必须重跑 `--emit`（`--check` 会报错） |
| **效果** | 静止窗口（掩膜关 → 开）：`self_masked` **0 → 3374 点/帧（29.4%）**；走廊最近格 `data_min_d` **0.00 → 0.20 m**（37/37 → 46/46 帧）；近场 max 台阶残差 **0.023~0.032（日志里出现过 0.128~0.203）→ 0.002~0.005 m**；"参考坡度" **0.0 → 2.9~3.1°**；限速落在地板的帧数 **9/129 → 0/46**（min 0.606 m/s）；**真台阶仍限速**：静止窗 `why=step` 40/46 帧、触发格 2.3~2.8 m、`台阶=0.180 m`；独立复测（114 帧静止 + 75 帧行驶）里行驶 75 帧中 **`why=step` 28 帧**、限速最低 **0.6 m/s** |
| **回退** | `config/traversability_self_mask_robot11.yaml` 的 `self_mask_enable: false`（或撤掉 launch 里 `self_mask_params` 那一路） |
| **证据** | `robot_models.md` §12.3/§12.6/§13.6；`Fidelity` §K.2.4（`dz` 三档实测：0.05 m 的闸有 **>2×** 余量）；提交 `1ca09eb` |

### §2.1.7 ★ 底盘**塌成单刚体**：`planar_move` 的 `Set*Vel` 只对单刚体自洽

| 项 | 内容 |
|---|---|
| **现象** | `robot:=robot11` **几乎不执行角速度指令**：盲发 `wz=1.0 rad/s` 持续 12 s（请求 **687.5°**）真值只转 **+3.79°**（**0.55%**）；反向 2.0 m 目标里 RPP 的 rotate-to-heading `wz=-0.75` **连发 45 s**、`vx` 恒 0、车只转 **6.03°**；**无指令 60 s 自由漂移 yaw +8.20°** |
| **原因** | 底盘驱动是 `libgazebo_ros_planar_move.so`，它的实现（`gazebo/physics/Model.cc:746-771`，本机读源码核对）是 **`SetLinearVel`/`SetAngularVel` 对模型里每一个 link 各设一遍同一个 `(v, ω)`**。这对**单刚体恰好自洽**（绕质心转时正是"每个 link 同一个 ω"），对**多刚体 + 关节树则不自洽**（第 i 个 link 的质心速度本应是 `ω × r_i`）⇒ 关节/接触约束求解器每步都要把它掰回来，**反作用把底盘的角速度吃掉**。`robot11` 是 **13 link / 12 joint**，默认模型只有 **7 link / 6 joint** ⇒ 前者被吃得更狠 |
| **过程（判决性测量）** | 先做**台架**（只 Gazebo + 底盘插件，摘掉 LIO/nav2/雷达，一次 <40 s、RTF≈1.00），可信度由三条外部锚点钉住（复现了 §L 的数）：正前方盲推极限 **0.4199 → 0.4077 m**、默认模型 **0.5402 → 0.5401 m**、原样 `wz` **+3.79°（墙钟）→ +7.09°（12 仿真秒）**。四个结构对照（同一世界/出生点、`wz=1.0` × 12 仿真秒）：极简单 link box（无接触）**+681.25°（99.9%）**／robot11 塌成单刚体 **+111.68…113.32°（16.2~16.5%）**／robot11 原样 **+7.09°（1.03%）**／全关节改 `fixed` **+21.31°（3.10%）**／仅转向关节 fixed **+16.69°（2.43%）**／零重力 **+128.02°（18.6%）**／默认模型 **+40.01°（5.82%）**（它自己零重力上限 48.1%） |
| **改动** | `tools/scripts/regress/robot11_weld_chassis.py`（生成器 `robot11_make_sim_xacro.py --chassis rigid` **默认调用**）：① 从 `base_link` 对 `j2…j11` 做 FK；② `l2…l11` 的 `<collision>/<visual>` pose **逐条**变换到根 link 系后挂上去（**几何一块都不改**）；③ `<inertial>` 用**平行轴定理**合成（`M = 9.4121 kg`、`COM = (0.003551, 0.007121, 0.073122)`、`Izz = 0.126766 kg·m²`）；④ 删掉 `j2…j11`，**保留** `livox_frame`/`imu_link` 及其固定关节；⑤ 去掉指向被塌掉 link 的 `<gazebo reference>` 材质块 |
| **原理** | ① **"没有执行机构的自由度"就是负担**：本栈**没有任何**轮子/云台控制器（`.xacro` 自己写着"上游 `j2..j9` 是 continuous 但我们不加轮子控制器"）⇒ 这 12 个关节在运行期唯一的作用就是**与 `planar_move` 较劲**；② **自证必须有"不依赖 FK 正确性"的物理锚点**：AABB 逐条相等这条会"两边一起错、检查照样通过"（实测踩过一次：`rpy="1.5708 -1.5708 0"` 被按"6 个数"解析 ⇒ 读成零向量），加上"**整车最低点必须落在轮子上 = −0.10250 m**"之后那次退化当场报错；③ **接触/摩擦是第二个独立限制项**（剂量-响应单调：`mu` 1.0/0.5/0.2/0.05/0.001 ⇒ `+7.09/+17.26/+54.75/+96.91/+126.80°`），但它**只解释 18.6% → 1.03% 那一段**，主因仍是结构不自洽 |
| **效果** | 台架：**1.03% → 16.2~16.5%（15.7×）**。整栈（同协议）：盲发 `wz=1.0`×12 s 真值 **+3.79° → +35.42°**（LIO +33.75°）、执行率 **0.55% → 5.15%**（**优于默认模型的 3.98%**）；**60 s 无指令自由漂移 +8.20° → +0.006°**（LIO −0.091°，与默认模型 −0.154° 同量级）；反向 **1.2 m** 目标：真值位移 **1.2102 m**（请求 1.2）、残余 0.0794 m、**`Goal succeeded`**。**被否掉的修法**：插件 `update_rate` 100 Hz → 1000 Hz 确实把角速度提到 7.8%，但**同一次改动让车"爬上"了实体障碍**（盲推 0.4077 → 2.5652 m、底盘 z 0.1522 → 0.4005 m）⇒ **不采用** |
| **回退** | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated`（生成物**模型体去注释后与 2026-10-09 的 HEAD 逐字节相同**，两份都是 35025 B、`diff` 为空）；塌陷工具本身 `rm -f tools/scripts/regress/robot11_weld_chassis.py` |
| **证据** | `tilted_lidar_fidelity.md` §L.5/§M.0–§M.12；提交 `ef927e2` `9de6b72` `d7c4da0` `d3034aa` |

## §2.2 感知 / 代价图层

### §2.2.1 近地剔除（`obstacle_near_ground_m`）：2D 链路上**唯一**能用上"高度"的那一级

| 项 | 内容 |
|---|---|
| **现象** | 用户：「那个膨胀半径感觉一直把地面当成障碍物算进去」；实测**车那格 = 42（非 free）**、车半径圆（0.3565 m）内 **`≥99` 156 格、free 只有 239 格**、到最近 lethal **0.531 m**、到最近 `≥99` **0.175 m** ⇒ `RegulatedPurePursuitController detected collision ahead` → `Controller patience exceeded` → 自旋恢复 |
| **原因** | 三层叠加（**逐条带数**，`Fidelity` §D.3）：① 近场地面残留（水平 **0.26~0.40 m**，linefit 因 `r_min 0.2` + `max_dist_to_line 0.05` 最难判的那一圈）；② 插件 `point = range·axis` 的**系统内移 0.1 m** 把近处地面整体抬高；③ `p2l` 把 3D 点**投成 2D**（LaserScan 没有高度）⇒ `obstacle_layer` 把它们标在**雷达高度**上 ⇒ **"地面"在局部代价图里就是货真价实的 lethal 格**。⚠️ 全局图那一路是 3D 的（STVL、`min_obstacle_height 0.0`）⇒ 同样的地面点（odom 里 z ≈ −0.07）会被滤掉（所以两张图的最近 lethal 略有差别 0.386 vs 0.400 m） |
| **过程（判决性测量）** | ① **先给"贴地点"一个判据**（在投影成 2D **之前**）：用判据层本来就有的局部地面 `g(x,y)`（0.20 m 粗格 → 4×4 个 0.05 m 细格"最低点"的 p05）算 `dz = z − g`；② **同一帧回放 A/B**（把一帧真点云 11568 点回放给**活的** `ground_segmentation_node`，只换阈值）：0.00/0.05/0.10 ⇒ ground 4683/**5985**/7286、obstacle 6885/**5583**/4282（**总数恒为 11568 = 不丢点**），`step_edge` 中位 **2011/2011/2011（逐字相同）** |
| **改动** | 新增参数 `obstacle_near_ground_m`（**0.0 = 关**），`robot:=robot11` 槽位 = **0.05**；"升"（obstacle→ground）显式写在两个分割节点里、**只在 `nearGroundEnabled()` 为真时**执行 ⇒ 默认路径连掩码都不填、行为逐字节不变；`~/traversability_stats` 多两个**私有**诊断字段 |
| **原理** | ① **信息一旦跨过 `p2l` 就永久丢失**（LaserScan 没有 z）⇒ 想用高度就必须在"投影之前"；② 阈值取 **0.05 m** 的三个实测依据：近场地面环 `dz` p05/p50/p95 = **+0.0035/+0.0057/+0.0095**（100% ≤ 0.05，余量 >5×）、水平 0.40–0.50 m = **+0.0018/+0.0089/+0.0125**、真障碍（~0.15 m 低矮件）在 0.50–0.75 m 的 `dz` p95 = **+0.1408**（只剔掉贴地那一层：`frac dz≤0.05 = 0.775`，**留下的正是 0.10–0.17 m 的本体**）；③ 与台阶判据**结构上互斥**：台阶闸 `step_height_threshold = 0.15 m` > 0.05 ⇒ 台阶点不可能被剔；④ `dz` 是"同一个量在同一个帧里相减"⇒ **与 `sensor_height` 标定、与 odom 原点都无关** |
| **效果** | 在线 A/B（robot11、静止、唯一变量 = 阈值）：车那格 **42 → 0（FREE）**；车半径圆内 `≥99` **156 → 0**、free **239 → 999**；到最近 lethal **0.531 → 1.534 m**、到最近 `≥99` **0.175 → 1.178 m**；径向剖面 0–0.3565 m 的 `≥99` 格 **全 0**；`/segmentation/obstacle` 中位 **6178.5 → 5851**、`/scan` 只少 **53 条/帧**（全是 0.30–0.50 m 的贴地回波） |
| **回退** | `config/traversability_near_ground_robot11.yaml` 的 `obstacle_near_ground_m: 0.0`（一个数），或去掉 launch 里 `near_ground_params` 那一路 |
| **证据** | `Fidelity` §K.2；提交 `360d493` `18f2738` |

### §2.2.2 `robot_radius`：**外接 0.3565 → 内切 0.300**（"能走多窄" vs "保证不撞"）

| 项 | 内容 |
|---|---|
| **现象** | "车一开始就在膨胀团里"：**全局**图上（静止！）外接那一档车那格 = **99**、车半径圆内 **0 格 free**、到最近 lethal 只有 **0.135 m** |
| **原因** | 本仓**没有 footprint 多边形** ⇒ `robot_radius` 在 nav2 里的真实语义 = **`inscribed_radius`**（"能走多窄"）。用**外接**半径（0.3565 = 凸包顶点最远）会把"车那格"在全局图上判成 inscribed；而**车那格非 free 的真因并不是半径** —— 先是"贴地点被标成 lethal 格"（§2.2.1），**再**被大 `inscribed_radius` 放大成"整个足迹非 free"（这条**纠正了 §D.3 的归因顺序**） |
| **过程（判决性测量）** | 干净静止三档 A/B（近地剔除都开、**唯一变量 = 半径/膨胀**）：① 外接 **0.3565/0.70**；② 内切 **0.300/0.60**；③ 0.22/0.50（默认模型那组，只为分离变量）。结果：**局部**三档都是"车那格 0 + 圆内 0 个 ≥99"（⇒ 局部问题已被近地剔除解决）；**全局**只有 ② 在静止时车那格 = 0（① 是 99、③ 是 77） |
| **改动** | `src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`（**只含几何键**，launch 用 `_Nav2ParamsForSlot` 深合并成临时 YAML ⇒ 其它槽位**原样返回公共段路径**）：local/global `robot_radius 0.3565 → 0.300`、local `inflation 0.70 → 0.60`、global `0.75 → 0.65`；`cost_scaling_factor` 不动 |
| **原理** | ① **半径的语义决定取值**：要"能走多窄" ⇒ 内切；要"保证不撞" ⇒ 外接或真 footprint；**两者不能用一个数同时满足**；② **折中必须写清**：内切圆会让**四个角在转弯时扫到** 0.300–0.3565 m 这一圈里的障碍（底盘是 600×600 切角，角点在 0.3565、边到原点 0.300）；兜底 = 场地没有更贴的窄缝 + 软带仍在 + 前瞻限速与碰撞监控仍在链路上 |
| **效果** | 采用 **0.300/0.60/0.65**：全局车那格 **99 → 0**、车半径圆内 **0 → 123 free**、到最近 lethal **0.135 → 0.804 m**；局部圆内 free **723/1008 → 871/1005**；软带 0.30/0.35 m 与默认模型（0.28/0.33 m）同量级。**真足印多边形：试了，没采用**（原因 1：nav2 的 `footprint` 参数声明成 `std::string`，本仓的运行时深合并写嵌套序列会让 `rcl` 报 `Sequences cannot be key at line 127` 并把**所有** nav2 节点一起 exit；原因 2：量到的收益只剩"转弯角不扫障碍"这一条） |
| **回退** | 改 `nav2_params_sim_robot11_costmap.yaml` 的两个键回去；或 `git revert 1c89473` 的该文件 |
| **证据** | `Fidelity` §K.3；`robot_models.md` §9.4（内切 0.300 / 外接 0.3565 的来源）、§11.9（覆盖层机制与运行时回读）；提交 `1c89473` |

### §2.2.3 插件 `point = range·axis` 漏了射线起点（0.1 m 的系统内移）

| 项 | 内容 |
|---|---|
| **现象** | 原始云的**"地面峰"随距离变化**（像斜面）：`r 0.25–0.35 → −0.208`、`0.35–0.50 → −0.219`、`0.5–1.0 → −0.237`、`2–4 → −0.253`（几何真值 **−0.2595**）；近场一团点（28.5% 落在 `r<0.12 m`）不在任何 collision 几何上 |
| **原因** | 源码级一行：射线起点是 `start = minDist·axis + offset.Pos()`（`minDist` = SDF `<range><min>` = **0.1 m**），而 `range` 是**从射线起点**量的距离 ⇒ 命中点应为 `range·axis + minDist·axis + offset.Pos()`，旧代码写的是 `point = range·axis` —— **既漏 `minDist·axis` 也漏 `offset.Pos()`**，于是每个点都朝传感器方向**内移 0.1 m**（近处仰角大 ⇒ 抬得高） |
| **过程（判决性测量）** | ① **可核对的代数事实**：`point = range·axis` 与 `InitializeRays()` 的 `start = minDist·axis + offset.Pos()` 用的是**同一个 `axis`**；② 跨模型 A/B（同一编译只差那一行）：默认模型的 `obstacle` **3716 → 3782（+66）**、`voxel_grid` **1129 → 1303（+174）**、`inscribed` 反而**降 580**；robot11 的**地面峰 −0.280 → −0.155 m**（旧值"随距离变"、新值是**尖峰**：`z ∈ [−0.16,−0.15]` 单桶 3283 点 = 1–4 m 环的 **76%**） |
| **改动** | `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`：`point = range_from_origin_ ? (range*axis + minDist*axis + sensor_offset.Pos()) : (range*axis)`；回退开关 SDF **`<range_from_origin>false</range_from_origin>`**（**缺省 = 修好的语义**）；启动日志打一行便于确认加载了哪一版 |
| **原理** | ① 这**只让坐标回到几何真值**（修复后 `point` 与 `start + range·axis` 逐位相等）；② 敢设成**默认**（而不是只给 robot11）的依据 = **默认模型没有任何一项变坏**；③ **代价必须登记**：它改的是**所有模型**的点云 ⇒ 任何"按旧点云烘出来的东西"都要重核（`traversability_self_mask_robot11.yaml` 的 113 个 AABB 与近场死区都是按旧点云烘的）——实测掩膜命中率**没有塌**（自击团离传感器只有 0.02~0.05 m，0.1 m 内移对它的**相对**位置影响最小）；⚠️ 若将来把 `minDist` 改大（>0.1 m），掩膜**必须重烘** |
| **效果** | robot11：**车半径圆内 `≥99` 13 → 0、free 601 → 851**（"修完这一行才和 §K.2.3 的圆内 0 个 ≥99 一致"）；`/scan` 有限波束 **864 → 772**（旧的 0.05–0.10 m 自击带里那批"被内移 0.1 m 后落进带内"的假波束消失）；局部 lethal/inscribed/free **338/11730/38277 → 231/8599/43960**（三项都往"更干净"走）；自击团点数 **3294 逐位相同**（⇒ 掩膜不用重烘的量化依据） |
| **回退** | 在对应模型的 xacro 的 `<plugin>` 里加 `<range_from_origin>false</range_from_origin>`（**逐字节**回旧行为，不需要回滚代码） |
| **证据** | `Fidelity` §K.4；`robot_models.md` §12.3.1；提交 `1403f42` `8e73c8b` |

### §2.2.4 linefit 的 `gravity_aligned_frame`：`Eigen::Affine3d` **未初始化**（点云被塌到原点）

| 项 | 内容 |
|---|---|
| **现象** | 该键一开（`base_link`）⇒ `/segmentation/ground` **恒 0 点/帧**（与"把输入点云预先转 −30/+30/−60/+60"无关 ⇒ 只与"走没走那条分支"有关） |
| **原因** | 源码（`ground_segmentation_node.cc`）：`Eigen::Affine3d tf;` —— **Eigen 的默认构造不清零**（只把仿射最后一行置 `(0,0,0,1)`），随后 `tf.translate(0,0,0)`、`tf.rotate(q)` 在**垃圾矩阵**上叠旋转 ⇒ `pcl::transformPointCloud` 把点全塌到 ~1e-310 |
| **过程（判决性测量）** | 三条独立证据：① **最小 C++ 复现**（Eigen 3.4）：`(tf * Vector3d(0.1,0.2,0.3))` → `(7.5e-310, 7.5e-310, 7.5e-310)`，换成 `Identity()` → `(0.1, 0.323, 0.160)`；② **离线复算**：同一帧先转 −30° ⇒ 判地面 **5100/11711 = 43.5%**（其中 81% 真是最低那层地面）；不转 ⇒ 23%（那 23% 里只有 0.7% 是真地面）；③ **回放活的节点**：`""` ⇒ **3969** 点/帧；`"base_link"` ⇒ **0** 点/帧 |
| **改动** | 一行（+注释）：`Eigen::Affine3d tf = Eigen::Affine3d::Identity();` |
| **原理** | ① **结构性论证**：该修复**只动 `if (!gravity_aligned_frame_.empty())` 分支内部**，默认路径根本不进这块代码 ⇒ 默认行为**不可能**变化（同一帧实测也确认了 3969 → 3969）；② 修好之后"点云留在传感器系 + `gravity_aligned_frame: base_link`"这条**更接近真机**的链路**重新可用**（本仓 `robot11` 仍然选择"在源头对齐"，因为 p2l 不用建 tf2 MessageFilter） |
| **效果** | 修前 → 修后：`gravity_aligned_frame: ""` ⇒ **3969 → 3969**（逐字节口径一致）；`= "base_link"` ⇒ **0 → 3969**（obstacle 7742） |
| **回退** | 把那一行改回 `Eigen::Affine3d tf;`（或 `git revert 0142bd9`） |
| **证据** | `robot_models.md` §11.4；提交 `0142bd9` |

### §2.2.5 2D 高度带"为什么不能重新定基"（结论 = **不动**，并把原因写进参数文件）

| 项 | 内容 |
|---|---|
| **现象** | 想让"近场地面残留"不被标成 lethal ⇒ 很自然想到调 `obstacle_layer.scan.min/max_obstacle_height` |
| **原因** | **结构原因**（三行可核对）：① `nav2_costmap_2d` 的 `laserScanCallback` 走 `projector_.transformLaserScanToPointCloud(...)` —— `laser_geometry` 的 `projectLaser` 对 LaserScan 只能产出 **z ≡ 0**；② `obstacle_layer.cpp` 的标记循环在**代价图帧**里量高度（本仓 `third_party/nav2/.../obstacle_layer.cpp:470,476`），而 `min_obstacle_height` 的 nav2 默认是 **0.0**（本仓 base 参数里根本没有这个键 ⇒ 走默认，运行期回读确认 = 0.0）；③ 合起来 ⇒ **每条波束在代价图帧里的 z = 那一帧"点云原点"的 odom z（一个常量）**，与它打到的三维点**无关** |
| **过程（判决性测量）** | robot11、`plugin` 档、静止：`/scan` 有限波束 **950** 条；波束在 odom 里的 z：min/p05/p50/p95/max = **+0.0564/+0.0637/+0.0652/+0.0807/+0.0815 m**（跨度只有 25 mm）；落在 `[0,2]` 之外的 **0/950**；同一跑 `odom→livox_frame` 的平移 z = **+0.064 m**（与 p50 吻合到 1 mm） |
| **改动** | **保持 0.0 / 2.0 不动**，并把"这两个键在 2D 链路上的真实语义 = 点云原点的代价图帧 z 的常量闸"写进 `nav2_params_sim_robot11_costmap.yaml` 的注释；对 `plugin`（默认）档：波束 z ∈ [0.056, 0.082]、`[0,2]` 全收 ⇒ **现状已经正确，改任何值都只会变坏**。对 `sensor` 档：波束 z 跨度 **−0.394 … +2.361 m** ⇒ 单靠一条 `[lo,hi]` **永远**只能救回半个环（真修得改代价图帧或 LIO 的 odom 重力对齐，两者都超出范围且会把默认模型一起拖下水） |
| **原理** | **"能调的参数"不等于"能改的语义"**：当一个键作用在错误的帧里时，调它的数值区间只是在移动一个常量，不会产生"逐点高度判别"的能力；正确的做法是**在信息还存在的那一级动手**（§2.2.1 的近地剔除） |
| **效果** | 验收表里"`/scan` 出带比例（`sensor` 档）"要求 ~0 而实测 **0.3960** ⇒ **未达成**，但这条被记成"2D 链路结构上定不了基"，替代修法（近地剔除）对**两档都成立**且不依赖"odom 的 z 是什么意思"；同时更正 §J.5 的一处口径（见 §2.3.4 与 §5.1） |
| **回退** | 无（未改）；若要试验：改这两个键 + 重跑 `run_tilt_mount_probe.sh` 即可（但**预期只是整盘平移**） |
| **证据** | `Fidelity` §K.1（含 §K.1.2/§K.1.3/§K.1.4） |

### §2.2.6 `pointcloud_to_laserscan` 高度带：`max_height 0.1 → 1.0`（离地覆盖 0.326 → 1.226 m）

| 项 | 内容 |
|---|---|
| **现象** | 场地里 **0.40 m 以上**的护墙/立柱在 `/scan` 里**整条方位是 `inf`**；`inf_is_valid=true` 时下游把 `inf` 当"10 m 处有回波"**沿射线清成 free** ⇒ 这才是真正危险的方向；也是"坡脚撞击点在图上看不见"的成因之一 |
| **原因** | `target_frame: ""` ⇒ z 过滤发生在**点云自带帧**（`livox_frame`，离地 0.226 m）⇒ `max_height 0.1` 的实际含义是"离地 **+0.326 m** 以内"。换算错了**对象**（以为是离地高度，实际是传感器系 z） |
| **过程（判决性测量）** | 同一批帧、同一朵 obstacle 云、**只换 z 过滤**，并排打印两套带：0.20–0.30 m 带 96.3% → 97.0%；0.30–0.40 m **83.3% → 98.5%**；**0.40–1.00 m 0.0% → 100%**；≥1.00 m **0.0% → 98.5%**（linefit；patchwork 同向） |
| **改动** | `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`：`max_height: 0.1 → 1.0`（`min_height: −1.0` **不动** —— 它的语义是"雷达下方 1.0 m 以内也要"，比场地最低几何（坑底 −0.2 m）更低 ⇒ 不丢东西） |
| **原理** | ① **`min_height` 保持不动的理由是"不丢东西"，不是"对称美观"**；② 与 COD 2026 的 `min 0.01 / max 1.00` **数字看着像、含义不同**：他们配 `target_frame: chassis`（地面高度附近的帧）⇒ 那组数是**离地**的；本仓不做 TF ⇒ 同样的 `1.0` 落在**离雷达 1.0 m = 离地 1.226 m**；③ **风险与回退必须成对写**：抬到 1.226 m 意味着**头顶结构**（悬挑/限高杆/桥）也会被投影到地面 ⇒ 假障碍；本场地没有这类几何（已用 `PCD/RMUC2026.pcd` 与 `RMUC2026.stl` 复核：driving 区内 z>1.2 m 的几何都是墙/立柱的**上方部分**，其正下方本来就有同一面墙）；换场地/真机若有悬挑**要重估** |
| **效果** | 实时链路同帧 A/B（202 帧）：`/scan` 有限束数 linefit **856.8 → 934.7（+77.8，+9.1%）**、patchwork **1113.8 → 1163.5（+4.5%）**；只在新带里出现的束 p50/max = **+83/+143**（linefit）；新带**丢掉**的束 = **0.0**；同一方位回波**距离**的变化 p50 = **0.000 m** ⇒ **纯增量**（老回波一个不动） |
| **回退** | 把 `max_height` 改回 `0.1`（`ground_segmentation_slots.md` §14 的回退表） |
| **证据** | `ground_segmentation_slots.md` §11/§12/§14；提交 `cb3a756` `97d0dbc` `819160c` |

### §2.2.7 地面分割槽位 `ground`：linefit ↔ Patchwork++（含 `th_dist 0.125 → 0.08`）

| 项 | 内容 |
|---|---|
| **现象** | 22~35° 的**缓坡/倒角**被 linefit 判成障碍（实测障碍率 **35.7~44.3%**）⇒ 坡面从 `/scan` 消失、2D 图里也看不见；反过来把 `th_dist` 放松又会"吃"掉低矮障碍 |
| **原因** | linefit 的 `max_slope ±0.4` / `max_dist_to_line 0.05 m` 对"缓坡"过于敏感（把坡面整片删成地面）；Patchwork++ 的 `th_dist` 是"点到拟合平面的距离小于它才算地面" = **地面层厚度**，任何竖直特征最下面 `th_dist` 那一段会被并进地面 |
| **过程（判决性测量）** | ① 上游考证：`url-kaist/patchwork-plusplus` pin **`3e6903a1`**（= tag v1.4.1）、根 `LICENSE` **BSD-2-Clause**；⚠️ 它的 `ros/package.xml` 写 **GPL-3.0** 而 `ros/LICENSE` 是 **MIT** ⇒ **许可不清 ⇒ 不 vendor ROS wrapper**，只 vendor `cpp/**` 自己写薄节点；② 参数扫描：`th_dist` 扫 0.05/0.06/**0.08**/0.10/0.125 五档（合成地形）——**0.125 时 0.15 m 薄墙在 p2l 口径下只剩 11.8% 覆盖**，0.08 时 0.15/0.30/0.40 m 薄墙 **100%** 覆盖、0.2/0.3 m 平台顶面 100% 覆盖；③ 整栈 smoke A/B（`bench/ground_slot_smoke.sh`，同脚本/同时序/同场地） |
| **改动** | 新增 `ground` 槽位（`linefit`（默认）\| `patchwork`）＋ vendored Patchwork++（只 `cpp/**`）＋自写薄 ROS 节点；`config/ground_segmentation_sim.yaml` 的 **`th_dist: 0.08`**（**唯一偏离上游默认的算法键**）。**默认仍然是 linefit** |
| **原理** | ① **"许可证不清就不 vendor"** 是一条独立于技术收益的决策；② **换算法前先给"硬要求"定判据**（"矮/低障碍要留成障碍"与"缓坡不该被判障碍"是**互相拉扯**的两个要求）⇒ 用它反推出参数；③ **不换默认要有理由**：本次只做了离线 + 建图链 smoke，**没做 `mode:=nav` 整栈回归** ⇒ "可以 A/B，但建议先别换默认" |
| **效果** | 缓坡：linefit 判障碍 **35.7~44.3% → patchwork 2.5~3.0%**；STL 倒角上 patchwork 判地面的比例是 linefit 的 **2.6 倍**（69% vs 27%）；`/scan` 里 **<1.5 m 的近距假回波 91.32 → 34.43（2.65×）**；`/map` 占用 **6544 → 7756（+18.5%）**、已知格 **+17.2%**；分割器 CPU **0.0836 → 0.0562 核（省 33%）**；真实帧中位 **0.52 ms vs 1.01 ms（1.94×）**。**新代价**：倒角附近平地上出现少量假障碍（8.8~10.8% vs linefit 1.0~1.2%）、−0.2 m 坑的下坡倒角反而更差（52.5% vs 8.1% 障碍率）、0.05~0.20 m 高度带方位覆盖率 94~100% → 84~93% |
| **回退** | `ground:=linefit`（默认，等于没改）；只回退 `th_dist` ⇒ 改回 `0.125`（坡面更好，但 **0.15 m 矮墙会从 `/scan` 消失**） |
| **证据** | `ground_segmentation_slots.md` §1/§3.1/§5.3/§6.1/§7/§8/§9；提交 `1a9ce8c` |

### §2.2.8 局部代价图也上 STVL（`local_obstacle:=stvl | stvl_both`）

| 项 | 内容 |
|---|---|
| **现象** | 局部图原本只靠 `/scan` 的 2D 投影 ⇒ **高度信息丢失**（同 §2.2.5）；想判断"这团点是不是障碍"在 2D 链路上没有依据 |
| **原因** | 局部图 `global_frame: odom`、`obstacle_layer.scan` 走 2D；而 STVL（spatio-temporal voxel layer）**直接吃 3D 点云**并带时间维（衰减） |
| **过程（判决性测量）** | 四次跑 A/B（`stvl_local_costmap.md` §3），参数来源 = COD `singlenav2_params.yaml:200-262`（逐键对账） |
| **改动** | `local_obstacle:=stvl`（局部替换）／`stvl_both`（两侧都上）；默认不变 |
| **原理** | **"降维发生在哪一层"是可选的**：把 3D 直接喂给局部图 ⇒ 多一条"带高度/带时间"的通路；代价是**多一路消费者**（一致性与算力）——这正是"一份点云被切成多份"的权衡（`architecture.md` §3.2.5） |
| **效果** | 见 `stvl_local_costmap.md` §3 的四次跑表（本文档未逐项转抄，属**可核对但本文未复算**的一类，见 §5.3） |
| **回退** | 槽位改回默认（`obstacle_layer` 那条路） |
| **证据** | `stvl_local_costmap.md`（§0 速查 / §2 机制 / §3 A/B / §4 回退 / §5 未验证）；提交 `4798ea2` `b2c8e1a` |

## §2.3 LIO 层

### §2.3.1 时间戳 1 ns 截断：`/scan` 丢帧 **116 → 10**、TF 等待 p90 **136 → 13 ms**

| 项 | 内容 |
|---|---|
| **现象** | `slam_toolbox` 持续刷 `Message Filter dropping message: frame 'livox_frame' at time <t> … 'discarding message because the queue is full'`，**每 2.5 仿真秒一条**（min=p50=p90=2.5 ⇒ 系统性）、约 **4.0~4.3%** 的 `/scan`，每次跑都有 |
| **原因** | **四步因果链**（每步都可单独验证）：① 插件用**仿真钟**盖戳（`rclcpp::Time`，整数纳秒，精确）；② `/scan` 那条链**不经过 LIO**、逐字段抄 header（`ground_segmentation_node.cc:135-136`、`pointcloud_to_laserscan_node.cpp:149`）⇒ `/scan` 的戳 == 插件戳；③ **另一条链（LIO）把插件戳转成 float64 秒再转回整数纳秒**：`nanosec = static_cast<uint32_t>((ts - floor(ts)) * 1e9)` —— **截断**，同一个 double 在 ~1e3 s 处的往返误差 ~1e-7 ns，方向随数值而变 ⇒ 约 **40%** 的帧落在 `X.99999999x` ⇒ 截断后变成 `X−1 ns`；④ tf2 的 `MessageFilter` 判据是"在**消息自己的戳**上 `canTransform`" ⇒ **低 1 ns 的同帧样本不合格**（被判成"要外推到未来"）⇒ 只能等**下一帧 TF**（~0.1 s）；队列深度 1 时新来的 `/scan` 把还在等的那条顶掉（`message_filter.hpp:411-419`） |
| **过程（判决性测量）** | ① 新增**逐纳秒戳差仪器**（探针加整数 ns 指标 + 离线复算器，`5410412`）；② 实测：**Δns = 0 的扫描一条都没丢（1658 条 / 0 丢），116 条丢帧的 Δns 全部 > 0**；③ `scan_queue_size` 1/5/10 的 A/B：**4.2% → 0** —— 但这条**不是**被采纳的修法；④ 上游考证：默认 1 由 PR #526（2022-08-24）引入，README 明写 "Should always be set to 1 in async mode"；维护者口径 = 这是 **TF/时间戳问题**不是 slam_toolbox 的 bug（#720/#777/#806/#834 全以"用户侧 TF 配置"关闭）；真正的上游修法在 `ros2/geometry2#544`（**2022-07 提出，至今 open**）；⑤ 可比项目（COD 2026 / TurtleBot 4 / uOttawa rover）**都没配** `scan_queue_size` |
| **改动** | **只改 LIO 的戳构造**（截断 → **四舍五入 + 进位**），让 TF 戳与点云戳**逐纳秒对齐**；`slam_toolbox` 的参数**一个都没改**（保持默认 `scan_queue_size=1`）；`timestamp_construction_audit.md` 顺带全仓扫了一遍：**`FAST_LIO` 有完全相同的 bug**（实测 Δ(TF戳−`/scan`戳) = −1 ns 占 **40.0%**、丢帧 **126/3003 = 4.2%**，与修复前的 `small_point_lio` 逐项吻合）；**`point_lio` 代码逐字相同**（判定同一 bug，但**没能实测**——该槽位在仿真里第 100 帧就被自己的 `pcd_save` 打死）；**全仓没有第 3 处**截断类构造 |
| **原理** | ① **根因修复 ≠ 队列补丁**：调大队列只是把"等待"吸收掉，被吸收的消息最终仍可能因超时/外推失败而丢（§1.2 第 9 条）；② **"谁的时间戳在骗人"要用逐纳秒的仪器去量**，不能靠"看起来对不对"；③ 真正把"1 ns"放大成"丢数据"的只有 `scan_queue_size=1`；costmap 侧是 `MessageFilter(queue=50)` + `transform_tolerance`，**只会多等一帧、不会丢** |
| **效果** | 验证跑 `fix01`：丢帧 **116 → 10（4.3% → 0.35%）**、**2.5 s 节律消失**、等待 TF 的 **p90 136 ms → 13 ms**；丢帧影响量化：被丢的 4% 里 **58.6% 被两个处理门证明本来就会忽略**，剩下 41.4% 的代价上界 = 被选中的帧晚 **0.1 仿真秒 / 2 cm** ⇒ 结论原本是"**不改默认**"，修完根因后连这条代价也基本消失 |
| **回退** | `git revert a281636`（回到截断语义 ⇒ 丢帧日志会回来）；**不要**用调大队列来"消日志" |
| **证据** | `slam_toolbox_scan_drops.md` §0/§6/§7/§8/§9；`timestamp_construction_audit.md`（结论先行 + §3/§4.4）；提交 `a281636` `5410412` `c73e9d0` `5c3d7af` |

### §2.3.2 bug ②：`/cloud_registered` 对"已经在 odom 的点"又乘了一次 `T(base_link←livox_frame)`

| 项 | 内容 |
|---|---|
| **现象** | `robot11_mount:=urdf` 档：`/cloud_registered` 里**地面变成 30.97° 的斜面**、**64.6% 的点落到水平面以下**（`plugin` 档是 0/11620）；RViz 固定帧取 `odom`/`map` 时整个场景像"被刚性转过" |
| **原因** | 回调收到的点来自 `small_point_lio.cpp:92-98` 的 `p_odom = R_ol·(extrinsic_R·p_lidar + extrinsic_T) + t_ol` —— **已经是 odom 系**；而 `small_point_lio_node.cpp:141` 又 `lookupTransform("base_link", lidar_frame)`、`:191` 做 `R*p+T`、`:159` 写 `frame_id="odom"` ⇒ **对已在 odom 的点再乘一次变换、还盖 odom 的标签** |
| **过程（判决性测量）** | ① **反变换检验**（同一条判据两个独立口径）：按 `T(base_link←livox_frame)` 反变换回去，地面从 **30.970° → 1.041°**（回到水平）；② **逐点残差**（最近邻配对）：`H_a`（`p_pub ≈ R(base←livox)·p_raw + t`）残差 p50 **2.67 cm**、82.4% 的点在 5 cm 内；`H_b`（不转）p50 **11.47 cm**、只有 10.2% 在 5 cm 内 ⇒ **H_a 赢 4.3 倍**；③ **地面法向分量**：`R(base←livox)·n_raw` 与实测发布的地面法向夹角 **0.088°** ⇒ 是**同一个刚体变换** |
| **改动** | 删掉那次 TF 查询与逐点刚体变换，直接把回调里的点写进消息（`*pointer = point.x()/y()/z()`）；`frame_id` 仍是 `odom`；文件里留日期 + 机制 + 影响面的中文注释 |
| **原理** | **`frame_id` 是承诺**：既然写 `odom`，数据就必须真的在 odom 里；**"默认档看不出来"不等于"没问题"** —— `plugin` 档那条 TF 是**纯平移**（`|t| = 0.2044 m`），所以它只把整朵云平移了 0.2 m，看不出"多转" |
| **效果** | `urdf` 档：**30.970° → 1.054°**；默认档：帧内几何（点数/地面倾角/扇区/自仰角谱）**逐项不变**、只把整朵云搬回真正的 odom 坐标（−t = −0.2044 m）；副作用检查：`/global_costmap/voxel_grid` 每帧点数 **761 → 791.0**（量级内）、`/segmentation/*`、`/scan` 点数不变 |
| **回退** | `git revert <J 的 LIO commit>`（默认档**逐位不变**，revert 后与 §I 那一轮等价） |
| **证据** | `Fidelity` §I.5.1/§I.5.3/§J.1.1/§J.2；提交 `2a5a8b6` `7542973` |

### §2.3.3 bug ③：`odom→base_link` 的姿态用了**共轭**而不是**合成**（假俯仰 4.890° → 0.317°）

| 项 | 内容 |
|---|---|
| **现象** | `urdf` 档：发布的 `odom→base_link` rpy = `[0.383, **4.890**, 7.701]°`，`/odom` 窗口内 pitch 跨度 **4.924°**，而真值 `/odom_ground_truth` 只有 **0.003°** ⇒ **nav2 以为车在 odom 里俯仰 4.9°** |
| **原因** | `small_point_lio_node.cpp:99` 取 `lookupTransform(lidar_frame,"base_link")`、`:109` 算 `T_ob = T_bl⁻¹·T_ol·T_bl` —— 这是一个**相似变换（共轭）**，不是坐标合成。正确写法由**链式法则**给出：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ `T_odom←base = T_ol·T_bl`；等价说法：位姿一致性 `T_ol = T_ob·T_bl⁻¹` ⇒ `T_ob = T_ol·T_bl`。**共轭只在 `T_bl` 是纯平移时"姿态碰巧对"** |
| **过程（判决性测量）** | ① 写了一个 **30 行 tf2 单测**（`tools/scripts/tiltmount/tf2_compose_order_test.cpp`，不进 colcon）把语义钉死：`tf2::Transform` 的 `A*B` 就是矩阵序 `R_A·R_B`；旧写法对一个"只有 yaw 9° 的水平姿态"给出 `(0.306, 4.486, 7.810)°`（**与实测那 4.9° 的符号/量级都对得上**）；② **正演** `R_bl·R_ol·R_blᵀ` 与实测 rpy **逐位相同**（0.383/4.890/7.701）⇒ 机理钉死；③ 按代码语义**反解** LIO 状态 `R_ol` = `[0.086, 0.405, 9.105]°`（水平，只有 yaw）⇒ 说明"物理正确值就是 `R_ol`"（在"点云其实表达在水平系"的现状下） |
| **改动** | 姿态取**合成** `T_ol · T_bl`；⚠️ **平移故意保留旧值**（不是漏改，理由见下） |
| **原理** | ① **"共轭/相似变换"与"坐标合成"是两件事**，只有当被夹在中间的那个变换是纯平移时才会重合；② **读表要用矩阵/一致性，不要用 rpy 分量**：`sensor` 档的 **−4.9° pitch 不是假俯仰**，而是"绕斜 30° 的 odom 转 yaw"在 ZYX 展开下**必然出现**的项（矩阵本身是对的：`R_ob == R_x(30)·Rz(Δ)`、`R_ob·R_bl⁻¹ == R_ol` 两条恒等式见 §J.3 单测与 §J.4.1）；③ 一条**记法更正**：§I.9 的 C++ 单测原把"正确合成"写成 `T_ol·T_bl⁻¹`（在 `T_bl := T(livox←base)` 的记法下差一次求逆），以 §J 为准 |
| **效果** | `urdf` 档：假俯仰 **4.890° → 0.317°**（< 0.5° 验收线；那个 30° 出现在 **roll** 上 = "odom 定义在斜的初始传感器系"这个物理事实）；默认档：**姿态与平移逐位不变**（`T_bl` 旋转 = 单位阵 ⇒ 合成与共轭姿态相同；平移显式取旧值） |
| **平移那半"故意不改"的实测代价** | **把平移也改成物理真值 ⇒ 会打断两张代价图**：`/scan` 盘面在 odom 里的 z（p50）**+0.063 m → −0.091 m**；落在 `obstacle_layer` 高度带之外的波束 **0/948 → 950/950**（robot11）、**1239/1239**（默认模型）；局部代价图 lethal/inscribed **461/16937 → 0/0**（整张图 62500 格全 free）、默认模型 **445/11029 → 0/0**。机理：本仓 `rm_simulation.launch.py` 把车生成在 `z=0.2`、**LIO 在坠落中就做了重力初始化** ⇒ odom 的 z=0 比"落定后的地面基准"高 **0.2~0.35 m**，而旧的共轭平移**恰好**补回了这 0.2044 m 的一部分（"歪打正着"），所以今天的默认路径"能用"。修它必须**分两步**（平移 + 高度带重新定基都在 `src/rm_navigation/**/params`，属别的任务目录） |
| **回退** | `git revert` 该 commit（默认档逐位不变）；只退 `sensor` 档 ⇒ 去掉 `<cloud_frame>` |
| **证据** | `Fidelity` §I.5.3/§J.1.2/§J.3/§J.5；`robot_models.md` §15 尾部；提交 `2a5a8b6` |

### §2.3.4 GICP 重定位：三道接受判据 + 环路内低增益滤波（治"自传播发散"与"到点后抖动"）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去然后重定位出问题」；「到了点还在飘/四处抖动」。复现到的失败签名：`map→odom` 在一次 12.7 m 行驶里**单调漂到 y=+0.97 m**，而 **fitness 一直是 0.0022~0.0029 m²、100% 被采纳**（**对既有健康判据完全不可见**）；机器人**静止**的 120 s 里定位误差仍在长（p50 14.4 cm → max **47.8 cm**）；跑 12.7 m 实际走了 **29.7 m**、`/cmd_vel` 角速度换向 **35 次**、**15 次 recovery**；用户那次更极端：`map→odom` 到 **(4.98, 62.1, 8.28)** |
| **原因** | **不是"GICP 算错了"，而是"地图没有分辨力"**：本场地先验图（大片地板 + 规则重复墙体）让**任何位姿都能配得很"好"** —— 离线自相似探针（与节点**同一份代码路径**）：25/25 收敛、**偏 0.07~2.5 m 的位姿 fitness 仍只有 0.002~0.10 m²**，**远低于接受阈值 `max_fitness_score = 0.3`**；只有真值本身是"完美"极小（0.00022）。⇒ **单靠 fitness 永远分不出"真值"和"差 0.5 m 的孪生位姿"**；"每帧采纳 + 用它做下一帧初值" = 在一个连续极小（自相似平台）上**随机游走**，而初值来自 LIO ⇒ LIO 的误差直接喂进配准、跳盆地。**yaw 方向是可观测的**（yaw 扫描 25/25 收敛、离真值最大 0.035 m ⇒ 误差 < 3°）⇒ 失败总是"位置在自相似方向上滑" |
| **过程（判决性测量 + 被否候选）** | **① 运动一致性门限**（创新 = 本帧配准结果 ⊖ 本帧初值 = "GICP 这一帧想施加的修正量"）：阈值 `gate_trans_rate 3.0 m/s`（= MPPI `vx_max 2.5` 的 4 倍余量；健康跑行驶段逐帧步长 p95 = 74 mm）、`step_min 0.10 m`、`step_max 0.6 m`、`gate_yaw_rate 60°/s`（`wz_max 2.5 rad/s = 143°/s`）；`dt` 用**点云戳差**不用墙钟（墙钟过载会把非法大跳变"合法化"）。**② 合理性/定义域**：判据作用在**结果蕴含的 `map→base_link`**（不是 `map→odom`，因为"车在图里的位置"才是物理量）：`plausible_margin_xy 1.0 m`（PCD 包围盒 30.2×18.4 m）、`z ∈ [−0.9, +0.7]`（实测 `map→base_link` z ≈ **−0.30 m**；用户那次 `map→odom` z 涨到 **+8.28**）。**③ 连续拒绝 ⇒ 定位失效**：`lost_after_rejections = 25` 帧（10 Hz ⇒ 2.5 s）⇒ `estimate_valid_ = false`、停止配准、ERROR、等 `/initialpose`（**绝不编造位姿**）。**④ 环路内低增益滤波**。**被否掉的候选**：**紧门限**（0.8 m/s / cap 0.12 m / 20°/s）—— 实测拒绝率 **18%** 且拒的是**合法修正**（创新 0.1226 m / 0.538°、fitness 0.0024），发布值被冻住而里程计继续走 ⇒ 创新 0.31→0.50→0.95→2.24 m ⇒ 连续 25 帧判失效（**死锁**：越拒越偏、越偏越拒）；**"只平滑输出、初值仍取上一帧测量"** —— 实测无效（静止 120 s 漂 40 cm 照旧）；**更紧的 `max_correspondence_distance`**（1.5→0.8~1.0 m）—— **未测，不写进默认值** |
| **改动** | `src/rm_localization/gicp_registration`：`GicpNode::pointcloudCallback` 的"接受/拒绝"这一步（**不改 GICP 数学、不改 TF 契约、不加任何节点/无 watchdog**）；发布值 `T_map_odom_` **同时是下一帧初值**，朝"最近采纳的测量"按 `α = 1 − exp(−dt/tau)` 推进（`tau = 1.0 s`；50 Hz 定时器 + align 帧末尾两条路径共用同一个 `steady_clock` 基准）；新键全部有默认值、非法值 WARN 回退；`gate_enable:=false` + `smoothing_enable:=false` = 改动前行为 |
| **原理** | ① **滤波必须在环路里**：配准答案在一小片连续极小里跳，"每帧把测量当新位姿"（增益 1）= 把跳变**积分**进去 ⇒ 静止也会漂；低增益 ⇒ 抖动 **~1/α**、随机游走 **~√α**；`tau=1.0 s` 的依据：采纳率 10 Hz ⇒ α≈0.095，而 LIO 慢漂移实测 1~3 cm/s ⇒ 滞后仅 1~3 cm（低于 LIO 自身噪声），跳变噪声被压 ~10 倍；② **门限只该切灾难性跳变，不该管噪声**（噪声交给滤波）；③ **"让失败可见"**：`~/converged` 现在 = `被采纳 && score ≤ warn` ⇒ 新判据拒掉的帧会让 `converged=false`；④ **高频运动仍由里程计提供**（本节点只发 `map→odom`，**绝不碰** `odom→base_link`） |
| **效果** | **静止 A/B（单变量、同一二进制，只切两个开关；150 s）**：`map→odom` **逐次更新步长 p95 0.0087 → 0.0005 m（17×）**、max 0.0543 → 0.0283；**@10 Hz 重采样 p95 0.0086 → 0.0005（17×）/ max 0.0161 → 0.0008（20×）**；yaw 逐次步长 p95 **0.127 → 0.008°（16×）**；**累计修正路程 4.085 → 0.247 m（16.5×）**；融合位姿逐样本跳变 p95 **0.0048 → 0.0020 m（2.4×）**、max 0.0129 → 0.0027（4.8×）；相对真值误差两次都稳定 **8.5 cm 且不随时间增长**（⇒ 改动前那 4 m 的修正路程是**均值回复的抖动**，真正的漂移发生在**行驶**时）。**行驶跑（a→b/c）的导航结局反而变差（SUCCEEDED → ABORTED）是上游 LIO 自己失控主导的，不能当验收**（真实收益 = "不再把假位姿喂给 nav2"）。**P0 回归**：PASS、采纳 **762/762**、**拒绝 门限 0 / 越界 0（连续 0）**、fitness 0.0027~0.0031、align 14~17 ms；附带收益："发目标前等 `map→odom` 稳定"的判据从旧文档的 **3.4 cm / 30 s** 降到 **0.0020 m / 5 s** |
| **回退** | `--gicp-kv "gate_enable: false" --gicp-kv "smoothing_enable: false"`（= 改动前行为），或 `git revert 4f29d35` |
| **证据** | `gicp_divergence_and_jitter.md` §0–§7；`path_clearance_and_contact.md` §2.5/§4.4；提交 `4f29d35` `642c174` `25e33ad` |

### §2.3.5 GICP 的 `/initialpose` 饥饿：一次点击 **22.7 s → 0.24 s**

| 项 | 内容 |
|---|---|
| **现象** | RViz 的 `2D Pose Estimate` 点下去"没反应"：实测一次点击 **13.2 s** 才被处理、另一次 **14 s 内完全没被处理**；本轮复现 **22.7 s / 22.6 s**，且**只在点云流停止后 0.6~0.7 s** 才被处理 ⇒ 饱和期间等于"永不处理" |
| **原因** | `/initialpose` 与点云订阅**同处一个 `MutuallyExclusive` 回调组**（align 组）；点云 10 Hz、PCL 后端单帧 align **400~490 ms** ⇒ 该组**接近 100% 占用**，点击回调要排在在飞的 align（以及后面排队的点云）之后 ⇒ **调度饥饿**（不是 RELIABLE 丢包：样本没超时、没丢） |
| **过程（判决性测量）** | ① 节点空转（无点云）时：**~15 ms**；② 饱和时：**≈13.2 s / 22.7 s**，并把"应用发生在停流后 0.63~0.71 s"对上 ⇒ 根因钉在**调度**上（同一探针、同一条 RELIABLE 消息，只换回调组）；③ 三个观测量（apply 日志 → `~/pose` → 下一个 `/tf` tick）互相吻合到 **~2 ms** ⇒ 量的不是"日志什么时候打"，而是"`map→odom` 什么时候真的变了" |
| **改动** | **handoff + apply 拆分**：点击回调只做 **POD 交接**（拷原始 pose/covariance/header + 置 `pending_initial_pose_valid_`）跑在**自己的第三个回调组**（不是 TF 组，也不是 align 组）；真正的"应用"（TF 查询 + `T_map←odom = T_map←base·T_base←odom` + 状态写入 + no-improve 清零 + 一条 `~/pose`）在**下一帧点云的帧首**执行 |
| **原理** | ① **"人手点"的话题不该与"高频数据"共享调度预算**：不放 TF 组是因为那个组的契约是"50 Hz TF/状态发布绝不被拖慢"（改前 `/tf` 掉到 2 Hz 就是它被拖慢的后果）；② **串行性不变量"保住且更强"**：写 `T_map_odom_` 的代码现在就在 `pointcloudCallback` 里（先应用、再配准，同一条回调序列），连"两个回调互相插队"的可能都不存在；③ **QoS 保持 RELIABLE**（改成 BEST_EFFORT 会在网络抖动时**静默丢点击**）——交接变便宜之后 RELIABLE 不再有"占着组"的代价 |
| **效果** | 饱和场景（PCL 400~490 ms/帧 @10 Hz）：**22.783 / 22.408 / 18.478 / 12.484 s → 0.162 / 0.303 / 0.263 / 0.197 s**（handoff 日志 **0.0003~0.0004 s**，`~/pose` 与 `/tf` 生效与 apply 差 ~20 ms）；轻载（400 点云）：before 0.040 s / after **0.041 s** ⇒ **没有引入额外开销**；`Release(-O3)` 口径：before **0.000 s**（bug 被 `-O3` 大幅掩盖）/ after **0.121~0.137 s**（上限 = "到下一帧点云开头"）⇒ **修复后的延迟与 align 成本解耦**，修复前是**无上界**的 |
| **回退** | `git revert c7bcd6d`（会回到"点一次没反应十几秒"）；注意这不是"性能问题"而是**可用性问题** |
| **证据** | `gicp_initialpose_latency.md` §1–§4.3/§6/§8；提交 `c7bcd6d` |

### §2.3.6 LIO 漂移诊断：正常工况 **1 cm/m**、唯一复现的失效模式 = **高角速度甩转（抛硬币）**

| 项 | 内容 |
|---|---|
| **现象** | 用户：「才跑一小会儿里程计就飘」；另有"续建之后位姿飘"的怀疑 |
| **原因（四个候选逐条判决）** | **(a) 正常累积** ❌ 解释不了"才一会儿"：直线 **0.0105 m/m**、原路返回 **0.0104 m/m**、窄沟 **0.0031 m/m**；整程 ATE RMSE **0.036 m** / max 0.096 m（51.4 m / 408 仿真秒）。**(b) 卡死/几何退化** ❌：窄沟段 `/scan` 有限束数最小 **115**（其余 ~850）、车速只有指令的 **62%**、两段都撞墙钟上限，但该段 ATE RMSE **0.024 m**、**0.0031 m/m**；逐秒误差增量 vs 束数 **r = 0.065**、vs RTF **r = −0.026**（≈0）。**(c) 转弯诱发** ✅ **唯一复现出来的失效模式**（**与地图冷热无关**）：温和转垃圾无害（180°@0.32 rad/s 增量 −0.003 m、慢转@0.162 −0.0004、快转@1.104 −0.008 m），**但进到高角速度工况就是抛硬币**：同一档甩转（指令 3.8~6、实际 0.8~1.9 rad/s）**6 次里坏 3 次**，坏起来从 LIO max **0.62 m** 到位置跑飞 **10.68 m**（`map→odom` 甩到 13.34 m）；**最冷的一次（启动后 15 s）反而最干净 0.042 m，最暖的一次（走过 34 m）坏了** ⇒ 不是"地图没长好"。**(d) 续建污染存档的假象** ❌：续建的 `map→odom` **全程恒等**（0.08, 0.02, −0.014）直到 426 s；每一段 ATE RMSE 0.014~0.053 m、每米 0.004~0.015 m，与从零建图**没有差别**。**污染是真的**（只读复测 PCD 的 z 到 **+23.10 m**），但它**不在实时位姿链路上** |
| **过程（判决性测量）** | ① 分段脚本化路线（6 类：直线/原路返回/慢转/快转/窄沟/角速度扫频）+ **50 Hz 同步时间轴** + 逐段 ATE；② 对齐口径：用**起步静止窗口**估一次常量刚体变换、**全程沿用**（**绝不逐段重对齐** —— 那会把漂移当坐标系差减掉）；③ **段序本身是实验设计的一部分**（破坏性段必须放最后：第一版把甩转放最前 ⇒ 后面所有"干净段"结论全部作废）；④ 两条跑法护栏（上一轮 `gzserver` 没死干净 ⇒ 量到的是上一轮那台车；不要同时跑两个仿真） |
| **改动** | **只加工具与文档**（`tools/scripts/diag/`），**没有改算法**；本报告的最大价值是一条**作废声明**（见下） |
| **原理** | ① **"抛硬币"型失效要按"必要条件/充分条件"拆**：冲击是必要条件，但决定成败的是"这一次有没有让扫描匹配连续失配"（`conv_false` 采样数：发散的三跑 384/948/966，没发散的 5 跑**全 0**）；② **采样饥饿会造出假结论**（单线程 `rclpy.spin_once()` 在 ~80 Hz `/tf` 洪水下被饿死 ⇒ "LIO 位移是真值 2.6 倍"的假结论）；③ **把"当时以为的"写进文档并标注作废**，比删掉更有用 |
| **效果** | 给用户的直接回答：**正常开就不该飘（1 cm/m）；出现高角速度原地旋转 ⇒ "很快就飘"完全合理**（6 次坏 3 次、0.6~10 m / 几十秒，而且 `map→base_link` 会跟着一起飘，**slam 救不回来**：LIO 崩时融合位姿与 LIO **1:1 同步**，比值 0.869）；**不要把"续建存档被污染"当成实时位姿飘的原因**（它坏的是 3D 点云资产） |
| **回退** | 不涉及（只加工具/文档）；作废的旧结论（"要 1.9 rad/s 得给 6 rad/s"）**不要再引用**（§1.2 第 7 条） |
| **证据** | `lio_drift_diagnosis.md` §0/§2.3/§4/§5/§6/§7；`slam_toolbox_scan_drops.md` §0 的 2026-10-06 后续；提交 `e866cfa` `7d03f16` |

### §2.3.7 slam_toolbox：**节点加密让回环真的触发**（0.797 → 0.313 m）+ 丢帧裁决

| 项 | 内容 |
|---|---|
| **现象** | 用户：「slam_toolbox 的配置怎么改，建图漂移能小一点 / 能自己纠回来？」；每次建图都有 `queue is full` 丢帧日志 |
| **原因** | ① **回环为什么常常不触发**（三道闸门，**主闸门是"节点太稀"**）：`loop_match_minimum_chain_size = 10` 要求 **10 个连续的旧节点**同时落在 `loop_search_maximum_distance = 3 m` 半径内；而实测**节点间距 0.80 m**（= 1.5×`minimum_travel_distance`）⇒ 10 个连续节点要跨 **≈8 m**，**几何上不可能塞进 3 m**；二级闸门 `LinkNearChains` 更紧（1.5 m 内要 10 个连续节点）。② **单帧能纠多少**：`correlation_search_space_dimension 0.5` ⇒ 平移搜索 **±0.25 m** ⇒ **LIO 一次跳 >0.25 m，逐帧匹配救不回来，只能等回环**。③ 丢帧的直接因**不是**回环（见 §1.2 第 8/9 条） |
| **过程（判决性测量）** | 四个配置的实测（同一路线 9.9 m 矩形环、同一速度）：**A2 现状**：位姿图 13 节点、环秩最多 1、全跑只有 1 次"疑似回环"、`err_map ≡ err_lio`（差 ≤3 cm）；**B（只把 chain 10→4）**：**环秩全程 0** —— 一次回环都没有；**C（只把节点加密：`minimum_travel_distance 0.5→0.2` 等）**：**32 节点 / 环秩 8 / 6 次疑似回环**，末段 `err_map 0.045` 优于 `err_lio 0.082`；**D（chain4 + 半径 4 + 阈值放松，但节点间距不变）**：环秩只到 **2**。⇒ **只有"加密节点"真的让回环触发** |
| **改动** | `mapper_params_online_async_sim.yaml`：`minimum_travel_distance 0.5 → 0.2`、`minimum_travel_heading 0.5 → 0.2`（该版本里其实是**死参数**，PR #888：`shouldProcessScan` 根本不看它；改它只为"应用的就是被测过的那一组"）、`minimum_time_interval 0.5 → 0.25`；**不碰** `loop_*` 全组、`correlation_search_space_dimension`、`scan_queue_size`、`max_laser_range`、`transform_*`、`map_update_interval`；也**不碰** `mapper_params_localization_sim.yaml` |
| **原理** | ① **先考证"这个参数到底有没有人读"**（`mode` 在 2.6.10 里是"半死"参数：唯一读它的是 Ceres 插件、只做一件事 `enable_fast_removal`；"建图/定位/终身建图"由**起哪个可执行文件**决定；本机**没有** `lifelong_slam_toolbox_node` 可执行文件 ⇒ COD 队的 `mode: lifelong` 实际跑的就是普通 async 建图）；② **漂不是配置被改坏了**：我们的 `mapper_params_online_async_sim.yaml` 与上游出厂版**逐项相同**（只差 `max_laser_range` 10 vs 20、`base_frame`，外加缺 4 个无关项）；③ **"没有回环 ⇒ 位姿图永远不做全局优化"**（`MapperGraph::CorrectPoses()` 全仓只有两个调用点：`TryCloseLoop` 与 RViz 手工回环） |
| **效果** | 节点间距 **0.797 → 0.313 m**；环秩 **1 → 8**、回环修正事件 **0~1 → 6**（同一路线/速度/3 分钟）；同路线地图占用格 **1613 → 1893**、已知格 **32549 → 36714**（墙面更实）；**代价测不出来**：RTF **0.80 → 0.76**、`gzserver` CPU% **52.6 → 52.7**。丢帧裁决（另一份报告）：**回环不是丢帧的原因**（六格里没有一条丢帧落在回环 ±2 s 内，随机基线 1.5%）；**"更好的里程计"买到的不是精度**（ATE RMSE 已 **0.021 m**，而真值 0.000 m）、**而是 TF 的时延与稠密度**（TF 比同戳扫描晚 **7.7 ms**，阈值 132 ms ⇒ **17 倍余量**；把它人为拖到迟到 **130 ms** ⇒ 丢帧 **0 → 4.26%**）；⚠️ **`scan_queue_size` 别当解药**（队列 2 ⇒ `queue is full` 62→0，但换出 70 条另一种丢帧、位姿图节点 58→35） |
| **回退** | 三个键改回 0.5/0.5/0.5；或 `git revert 5cb93cb` |
| **证据** | `slam_toolbox_tuning.md` §0/§1/§2/§3/§4；`slam_drops_loopclosure_odom.md` §0–§5；提交 `5cb93cb` `d3dde82` `3a2b54d` `d0fe0a0` `c7260da` |

## §2.4 导航 / 限速层

### §2.4.1 前瞻坡度/台阶限速：连续量 → 速度上限（**单一阈值源**、零新增节点）

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去」；实测**车以 ~1.7 m/s 冲上 `map(−3.6,3.1)` 的坡脚时 IMU \|a\| 冲到 79~90 m/s²（≈8~9 g）**（静止基线 ≈9.8）；这一下把 LIO 打散（`loc_err` 0.06 → 2.2 m 用 2.4 s，之后**车已停住**它仍继续跑到 2000 m+） |
| **原因** | ① 场地的**坡道面**（坡度 ≤22%）被 `linefit`（`max_slope ±0.4`/`max_dist_to_line 0.05`）当成"地面"整片删掉 ⇒ `/segmentation/obstacle` 没有它、`/scan` 没有它、2D 先验图也没有它（实测**静态图判"自由"而场地高度 >0.10 m 的格 88 576 个 = 自由区的 51.6%**）⇒ 规划器无从绕、控制器无从躲；② 物理：刚性底盘（`planar_move` 是**速度控制**、无悬架）在 ~2 ms 内被要求获得 `v·slope ≈ 0.17 m/s` 的垂向速度 ⇒ **`a ≈ v·tanθ/Δt`** |
| **过程（判决性测量）** | ① **标定锚点**：用实测 `v ≈ 1.7 m/s`、10% 坡、IMU **79~90**（取 86）反算 **`Δt_eff = v·tanθ/a = 1.98 ms`**；② **目标**：把预期峰值压到 **30 m/s²**（= 实测发散那一击 86 的 1/3）⇒ `v ≤ 30×0.00198/tanθ = 0.0593/tanθ`；③ 离线合成扫掠（`bench/slope_speed_offline.cpp`，不需要 Gazebo、同一份判据代码）定 `brake 0.4 m/s²`（0.8/0.5/**0.4**/0.3 ⇒ 过坡脚 1.18/0.90/**0.70**/0.65）；④ 整栈 A/B（`ssl_off` vs `ssl_on3`，同目标 `(-12.64,-0.31)`） |
| **改动** | ① 判据层扩展（header-only `rm_ground_traversability`）：同一帧的粗格缓存上再算**"车前/车后 1~3 m 走廊"**的连续量（局部坡度、**坡度变化** `dtan`、**台阶残差** `step`）；② `SlopeSpeedLimiter::update()` 三条规则取最紧 + 刹车距离界 + 速率限制；③ 发 **nav2 原生 `nav2_msgs/SpeedLimit`**（`controller_server` 订阅 `speed_limit_topic` → MPPI `setSpeedLimit()` **按比例缩放 `vx_max/vx_min/vy/wz` 四个约束**）；④ **单一真源** = `src/rm_nav_bringup/config/traversability_criteria.yaml` 的 `speed_limit_*` 一组键（另有一份同样的表在 `slope_speed_limit.hpp`，`check_slope_speed_table.py` 断言两处逐键一致 + 与 nav2 的 `vx_max`/`max_decel` 对账） |
| **原理** | ① **速度表**：`v(d) = sqrt(v_req² + 2·brake·d)` 是"以 `brake` 减速就正好在 d=0 降到 `v_req`"的运动学反解 ⇒ **是"过坡脚前开始减速"，不是"在坡脚上减速"**；② **必须"已承诺特征 + 距离前推"**：雷达**看不见车下与近处地面**（MID360 下视 −7.22°、离地 0.226 m ⇒ 平地最近看得见 **1.78 m**），所以测量到的特征距离**永远 ≥1.0~1.8 m** ⇒ 只用"距离刹车界"时上限永远降不到 `v_req`（离线扫掠：过坡脚仍 1.2~1.4 m/s）；做法 = 承诺（记住 `v_req` 与距离）→ 每帧按 `上一帧上限 × hold_decay_factor(1.0) × dt` 前推 → 前推到 0 才释放；③ **绝对坡度规则只在"没有承诺"时参与**（否则会把已承诺的减速顶回 ~1.4 m/s）；④ **速度地板不许配 0**（`speed_limit_floor_mps 0.45`，校验脚本会拦）；⑤ **前瞻用"朝向走廊"而不是"沿规划路径"**：这两个地面分割节点**不订阅 `/plan`、不查 TF**（那是它们"零额外故障面"的前提），而云本身在传感器帧里、livox 安装 rpy=0 ⇒ 帧的 +x 就是车头；⑥ **抗噪**：格内抬升取细格最高点的 **p90**（不是 max：实测 max 噪声可达 0.15 m）、定位型特征要过**支撑格数**（同一 0.40 m 距离带内 ≥2 格一致）、`Δ坡` 死区 4°、台阶死区 0.04 m |
| **效果** | 整栈 A/B（`ssl_off` → `ssl_on3`）：IMU 全程峰值 **110.8 → 50.8 m/s²（−54%）**；IMU>30/>50 样本 **7/3 → 1/1**；两次**都到点**（status=4，最近 0.25/0.23 m）；`|v|` p50/p90/max **1.23/1.82/1.88 → 0.71/1.36/1.87 m/s**；`|v|<0.6` 时长 **4.8 → 9.4 s**；限速器行为：362 行 `[slope_speed]`、limit p50 0.85 / min 0.45 / max 2.00、**不限速占比 9%**、`why` = slope_change 122 / step 197 / slope 12 / none 31；**平地回归**：限速开/关都 PASS、无 recovery，用时 **3.2 → 3.6 s**（+0.4 s）、真值 max\|v\| 1.54 → 1.19 m/s |
| **没走通 / 没帮上（别重复走）** | ① **"标障碍"这条路**：坡面本身可行驶，把它标成障碍是错的；实测只把第一击 86.4 → 68.2 m/s² ⇒ **必须限速，不能靠 occupancy**；② **只朝前看的走廊**：两次整栈跑里最重的一击都发生在**倒车**时（`v = −1.04/−1.00 m/s`），车尾后面的坡脚不在走廊里 ⇒ 第一版峰值 66.5/67.7/86.5；③ **"拿 `/odom` 的 `twist.linear.x` 符号判运动方向"这条路没走通**：机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` 一直打"前进" ⇒ **方向判据没有生效，根因未定位**（订阅建了吗？时间戳/时钟？符号约定？）⇒ 默认改成**双向窗口**（不依赖任何话题语义）；④ **调 nav2 的全局限速**（`velocity_smoother.max_velocity 2.0→1.0`）不是替代方案：它不看地形、平地也慢 |
| **回退（三种粒度）** | ① 只关限速：`speed_limit_enable: false`（注意本节点是**构造期**读参数，`ros2 param set` 运行时**不生效**，要重启节点或改文件）；② 连判据一起关：`traversability_enable: false`；③ **残留上限的手工恢复**（节点被 SIGKILL 时最后一次上限会留在 MPPI 里；正常退出会自己发回 `NO_SPEED_LIMIT`）：`ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"` |
| **证据** | `slope_speed_limiting.md` §0–§7；`traversability_plan.md` §9；`path_clearance_and_contact.md` §7.1；提交 `ca89b69` `bb0fe03` `71f6c87` `fc226bb` `72155e4` |

**★ 提速档（用户嫌慢之后的下一步，`lio_divergence_no_impact.md` §3/§4）**

| 键 | 改前 | 改后 | 依据（实测） |
|---|---|---|---|
| `speed_limit_peak_target_mps2` | 30.0 | **45.0** | 唯一直接由"用户嫌慢"推动的物理量；45 < 实测最小致损簇（>50 那一档）；实测峰值 50~88 出现在**限速已生效**的旧表跑里 ⇒ 30 这个目标值本来就没把峰值压到 30 附近 |
| `speed_limit_floor_mps` | 0.45 | **0.60** | 限速器 **96% 时间在触发** ⇒ 地板就是巡航速度下界 |
| `speed_limit_slope_change_deadband_deg` | 4.0 | **5.5** | `why=slope_change` 占限速行 **56~79%**（4~5.5° 的场地起伏被当特征）；真坡脚 Δ坡 实测 **9~16°**，不会漏 |
| `speed_limit_step_deadband_m` | 0.040 | **0.060** | `why=step` 占 13~33%；0.06 m 仍远小于判据自己的台阶闸 0.15 m |
| `speed_limit_slope_knots_mps` | `[1.66,1.10,0.83,0.66,0.55,0.45,0.45]` | **`[2.48,1.65,1.24,0.99,0.82,0.61,0.60]`** | 按 `v = round_down_2(45·Δt_eff/tanθ × 0.975)` 重算，反算峰值 43.4~43.9 ≤ 45 |
| `speed_limit_step_knots_m` / `_mps` | `[0.040]` / `[0.45]` | **`[0.060]` / `[0.60]`** | 与台阶死区同步（两处必须一致） |

**提速档的效果（最硬的一条 = 结局）**：旧表 **0/4 到点**（1 次 ABORTED + 3 次超时，`|v|<0.6` 占 61~73 s），
新表 **2/2 到点**（都在 ~27 s 里走完 22.6 m，计划 23.4 m），`|v|` p50 **0.05 → 1.16/1.19 m/s**、
`|v|<0.6` 时长 **61.6 s → 0.3/0.6 s**、recovery **22 → 0/4 次**、`conv_false = 0`、`mo_stale = 0`、
`lio_err max ≤ 0.54 m`、单帧 step ≤ 0.201 m（= 2.0 m/s，正好是底盘上限）。
**但"更快的表更安全"不能说满**：地形冲击簇**每跑都有**（峰值 57~192 m/s²），
`f2` 那次峰值 **192.2**（全部跑里最高）却到点且没发散，`nb3` 只有 67.2 却发散了
⇒ **分得开的是"GICP 有没有被打进拒绝级联"**（`conv_false`：发散的两跑 384/966 vs 没发散的全 0）。

### §2.4.2 `--goal-forward 2.0` 走不到的归因：六个候选逐条判决 + **可达性前置判据**

| 项 | 内容 |
|---|---|
| **现象** | `robot:=robot11` 短目标能走（0.5 m）、**长目标（2.0 m）不动**；控制器侧一堆 `detected collision ahead` / `Controller patience exceeded` / 自旋恢复 |
| **原因（判决：实体障碍 + 角速度通道，**两个独立机制**）** | ① **根因 = 正前方 0.42 m 处有实体障碍**：把 nav2 **完全摘掉**、只给底盘插件发固定 `vx=+0.25`（25 s、500 条指令、请求 6.25 m），robot11 **只能前进 0.4199 m**（之后 20 s 一动不动，最后一次 >2 mm 的运动在 t=5.32 s）；**默认模型同协议只有 0.5402 m** ⇒ **不是 robot11 特有，是场地/出生点的性质**。② **"0.5 m 能走"是容差成功**：真值只走 **0.2977 m**、停在离目标 **0.1902 m** < `xy_goal_tolerance 0.25`。③ **控制器侧全是结果不是原因**：`/plan` 10 条正常、`/cmd_vel` 真的发过 `vx=1.0`（89 条 >0.01）、车也真的动到 **0.4197 m** 就推不动（与盲推极限 0.4199 m 差 **0.2 mm**）。④ **地图侧两个真实现象（不是根因，但会改变失败的形状）**：`slam_nav` 下全局图只覆盖已建出来的那一块（目标在图外 ⇒ `off the global costmap`），而 `mode:=nav`（完整先验图）对同一个目标直接 **0 条 plan**（`failed to generate a valid path` ×10；目标格 = 31/inflated、窗内 23 个 lethal、到 ≥99 格 0.158 m）⇒ **完整地图也说不通** |
| **过程（判决性测量）** | **六候选逐条**（`Fidelity` §L.6）：① **地图/空间** ⇒ **部分成立但不是根因**（`slam_nav` 有 10 条 plan、末点精确落在目标上，只是路径**穿过尚未建出 lethal 的障碍**；`mode:=nav` 反而拒绝规划）；② **控制器/规划器参数**（`regulated_linear_scaling_min_radius 0.9`）⇒ **反驳**（实测 `/cmd_vel` 发到 **vx=1.0**，远高于任何"限速"；且**默认模型用同一套控制器参数也一样失败**：0.674 m 后 20 s 只动 5 mm）；③ **BT/恢复** ⇒ **反驳（是结果）**（FollowPath **进了** 45 次 EXECUTING、`number_of_recoveries` 最大 **7**、恢复序列跑满，但都发生在**车已经被推不动之后**）；④ **局部代价图/滚动窗口** ⇒ **反驳**（目标**在**局部图内 91/91；局部图前方 0.15–0.80 m 就有 ≥99/lethal 带，**与物理一致**）；⑤ **出生自转 ~10°** ⇒ **成立但换了归因**（robot11 的 yaw 在 60 s 内自由漂到 **+8.20°**（默认模型 −0.17°），而**指令角速度只有 0.55% 被执行** ⇒ "自转"是**不受控的漂移**）；⑥ **（本轮新增，决定性）场地里的实体障碍** ⇒ **成立** |
| **三处独立的几何自洽（让"实体障碍"从猜测变成测量）** | ① 前进极限 + 车体外接半径 = `0.4199 + 0.3565 = **0.776 m**` ≈ 在线代价图里 x≈0.78 的那条 lethal 列；② 后退极限 + 外接 = `1.5753 + 0.3565 = **1.932 m**` ≈ 先验图 `RMUL2026.yaml` 的西墙 x≈−1.9；③ **同一跑内部**：导航跑里车被推到的峰值真值位移 **0.4197 m** ≈ 纯物理推的极限 **0.4199 m**（差 0.2 mm） |
| **改动（新验收口径）** | `nav_goal_forensics.py` 新增：**`--preflight-reach`**（发目标**之前**先用绕过 nav2 的盲推量出"这个出生点物理上能走多远"）、**`--reach-gate`**（目标超出实测可达区就**不发目标**，在 JSON 里记 `goal_gate`）、**`--goal-yaw-only-deg`**（原地转目标）、**`--dump-traces`**（真值/里程计完整时间线 —— "无指令漂移"只能从它读）；新工具 `run_nav_goal_forensics.sh` / `nav_goal_report.py`（把 `forensics.json` 读成判决表） |
| **原理** | **"任何基于目标的验收，必须先跑一次盲推可达性探针"** —— 在**不可达的目标**上，"发目标 → 看车动不动"永远是红叉，而红叉的原因与控制器无关。判据（本出生点实测）：`reach = {fwd_m: **0.3992**, back_m: **1.4370**}` ⇒ 可达区是"前 0.40 m / 后 1.44 m"，`--goal-forward ±2.0` **都在区外** |
| **效果** | 反向 **1.2 m**（在可达区内、闸门放行）：真值位移 **1.2102 m**（请求 1.2）、残余 **0.0794 m**（< 容差）、**`Goal succeeded`**、`/cmd_vel_chassis` 的 `max|vx| = 0.9624`、`n(vx>0.01) = 121`、`collision ahead` 14 条、恢复 4 次；反向 **2.0 m** 加闸门 ⇒ `in_reach=false` ⇒ **不发目标**（0 plan / 0 条 `/cmd_vel` / 位移 0）；反向 2.0 m **不加**闸门 ⇒ 0 条 plan、8 次恢复、真值 0.0733 m、**Δyaw −179.7°**（恢复 Spin 这次真的把车转了 180° —— **角速度通道修好的直接后果**） |
| **回退** | 三个新工具是纯增量（**不进任何 launch/节点**）：`rm -f tools/scripts/tiltmount/{nav_goal_forensics.py,nav_goal_report.py,run_nav_goal_forensics.sh}`；`--preflight-reach/--reach-gate/--goal-yaw-only-deg/--dump-traces` **默认关**，回退不影响任何旧命令 |
| **证据** | `Fidelity` §L.0–§L.12（含 §L.1.3 的口径更正）、§M.8；提交 `94f0d6b` `de3b78c` `9de6b72` `339e205` |

### §2.4.3 两个"顺带查到"的缺陷（与上面的结论无关，但**必须登记**）

| # | 缺陷 | 判据性数字 | 现状 |
|---|---|---|---|
| ① | **本仓 nav2 不检查目标朝向**（`goal_checker_plugins: ["general_goal_checker"]` = `PositionGoalChecker`，只声明 `xy_goal_tolerance`；1.1.20 里没有 `yaw_goal_tolerance`） | 原地转 180° 目标 ⇒ nav2 **立即** `Reached the goal!`、`FollowPath SUCCEEDED`，`/cmd_vel` 与 `/cmd_vel_chassis` 的 `max|wz| = **0.0**`、真值位移 0.00012 m、**Δyaw 0.005°** | **没改**（参数在 `src/rm_navigation/**/params`，按规则只许走既有槽位覆盖）。要让"原地转"变成真判据需换 `nav2_controller::SimpleGoalChecker` |
| ② | **`/odom_ground_truth.twist` 是"指令回显"，不是实测速度** | `planar_move` 的 `OnUpdate` 挂在 `ConnectWorldUpdateBegin` 上：**先** `SetAngularVel`、**再**在同一个回调里 `UpdateOdometry()` 读 `model_->WorldAngularVel()` ⇒ 该字段**恒等于刚设进去的命令值**（实测：真值 yaw 12 s 只转 **7.09°**，而该字段中位 = **57.296°/s = 1.0 rad/s**） | **要量"到底转了多少"只能用位姿差**（`/odom_ground_truth.pose` 或 `/odom`），**不能信 twist** |

### §2.4.4 贴膨胀边 → 撞击 → 定位失效：**"这次不是调参能修的"**

| 项 | 内容 |
|---|---|
| **现象** | 用户：「他喜欢贴着膨胀半径走而且容易撞击进去然后重定位出问题」 |
| **原因（三件事分工）** | ① **贴边**：`SmacPlanner2D` 的平滑器**没有代价项**（`smoother.cpp:100-190`：更新式只有 `w_data·(x−y) + w_smooth·(y₊₁+y₋₁−2y)`，代价只做 `cost>252` 的硬回退），而本仓 `w_smooth 0.4 / w_data 0.1`（4:1）⇒ **它把 A\* 用 `cost_travel_multiplier=4.0` 买来的余量又往内切带外沿拉**（计划路径到内切带距离 p50 只有 **0.25 m**）；② **致命一击与贴边无关**：撞的是**地图看不见的坡道面**（§2.4.1 的原因①）；③ **最要命的是"停发之后"**：`map→odom` 冻结后 nav2 仍用**控制器**（不是 recovery）继续开车 —— 最坏一次**盲开 110 m**、撞到 **474 m/s²**、足迹余量 min **0.05 m** |
| **过程（判决性测量）** | **11 次跑 / 9 个配置**（单变量）：`base2`（用户同参）⛔ 卡死、健康段余量 p50 0.750 / p05 0.347 / min 0.304、IMU **86.4**（7 个 >30）、定位误差 max 2.21 m、停发后行程 0.0 m；**`v_nosmooth`（`w_smooth 0.4→0.0`）** ✅ SUCCEEDED、计划到带 p50 **0.25 → 0.335**、执行余量 p50 **0.57 → 0.85 m**（健康段 0.75 → **1.03**）；**`v_ginfl`（全局膨胀 0.55→0.85 + `cost_scaling_factor 8→4`）** ✅ SUCCEEDED 且**唯一一次全程没挨重击**（IMU max 27、>30 样本 0）—— **但同一配置复跑 `v_ginfl2` 就灾难性失败**（IMU 76、盲开 **110.5 m**、定位误差 **12 883 m**、15 次 recovery）；`v_navfn` ✅ 成功但健康段 IMU 仍 **72.9**；`v_combo`/`v_mul12`/`v_mppiclr` ⛔ 更差；`v_rpp` ⛔ 69 s 只走 2.6 m（**没跟起来**，另一种失败）；`v_nolost`（`lost_after_rejections 25→0`）✅ 到点且 `mo_stale = 0`，但它**根本没触发那条策略** ⇒ **不能**用来证明"永不放弃更好"，它真正的用处是**反证"撞击 → 必然丢定位"不成立**（同样是 ≥80 m/s² 的撞击，有的跑散了、有的没散） |
| **改动** | **已应用**：`planner:=smac2d` 的 `smoother.w_smooth 0.4 → 0.0`（提交 `4a7996c`）。**建议但未应用**：让 2D 图别再丢"坡道面/低矮几何"（⇒ 后来由 §2.4.1 的判据层 + §2.2.6 的高度带部分落地）、给"定位失效"加**停车互锁**、`ObstaclesCritic` 的膨胀参数与局部图对齐。**明确不做**：放松 GICP 判据 / 改成"永不放弃" |
| **原理** | ① **"贴边"与"卡死"不是同一件事**：`v_mul12` 把 A\* 余量做得最漂亮（健康段 min 0.350 m、边缘<0 为 0%）**结局照样卡死**（定位误差 16.2 m）⇒ 只治贴边不治链；② **撞击不必然致命，但它是这条链唯一的入口**：把 11 次跑按健康段 IMU 峰值分档，**9 次 ≥70 m/s²**，其中 3 次仍成功 ⇒ LIO 会不会被撞散是**概率性**的；③ **"停发 TF 之后没人踩刹车"是被放大的主因**（575/581 个样本有 `/cmd_vel_nav` 控制器指令，不是 recovery） |
| **效果** | 见上表（`v_nosmooth` 的余量与 `v_ginfl` 的"唯一一次没挨重击"）；另有一条**独立佐证**（`map_opt` p05 **−0.61 m**：雷达看见了图上更近的几何） |
| **回退** | 把 `w_smooth` 改回 0.4（或 `git revert 4a7996c`）；其余都是"未应用的建议" |
| **证据** | `path_clearance_and_contact.md` §0–§8；提交 `1811149` `4a7996c` `6f0c27a` `acb93c1` `2e7cf61` `4da000e` |

## §2.5 资产 / 建图层

### §2.5.1 2D 先验的 canonical 化（"当前默认是哪一份"）+ PCD→2D 的坑

| 项 | 内容 |
|---|---|
| **现象** | `map/` 与 `PCD/` 里同时躺着**合成图、`/scan` 累积图、点云投影图、多次会话产物**；"到底哪一份是当前的"靠人记；建图时"坡道在 3D 点云里清清楚楚、在 2D 栅格图上基本看不见" |
| **原因** | ① 2D 图来自 `/scan`，而 `/scan` 是 `linefit → pointcloud_to_laserscan` 的产物（**传感器系窄高度带，改前有效 ≈0.05–0.33 m**）⇒ **坡度够缓的斜面被判成地面 ⇒ 从 `/scan` 里彻底消失**（实测坡道段有限束数只有其余段的 31%）；② 没有命名/回滚规范 ⇒ "提升一份新图"这件事本身没有留痕 |
| **过程（判决性测量）** | ① **同一把尺子**（`verify_low_terrain.py`，真值 = `RMUC2026.stl` → 0.05 m"最高可站立面"高度图）量**四张 2D 候选**：`RMUC2026.yaml`（提升后）边沿被表示 **52.71%**、撞击窗 `map(-3.6,3.1)±0.6` **31/31 = 100%**、可行驶斜面被标占用 **5.21%**；`.bak-20261007`（提升前）**49.44% / 28/33 = 85% / 3.29%**；`_spl`（`/scan` 累积）**37.43% / 32/33 / 4.05%**（未知占 60%）；`_cloud`（点云投影，**已否决**）**35.98% / 30/33 / 7.23%**（超 5% 判据）。② **同窗口逐格对照**（`map_prior_ab.py`，公共窗口 `x∈[−24.80,3.60] y∈[−8.92,6.98]`）：新图边沿 **49.44% → 54.42%**（多表示 208 个真值边沿格）、未知格 **34819 → 24743（多"看见" 25.2 m²）**、新窗口**完全包含**旧窗口；代价 = "坡面中段凭空多出来的真障碍" **35 → 55 格（0.09 → 0.14 m²，差 0.05 m²）**、两图互换各有 3~4 千格占用/自由翻转（两次独立建图的正常噪声）。③ 3D 先验同尺子（GICP 自相似探针）：点数 **178,543 → 203,352（+14%）**、`xy` 足迹 **415.6 → 449.3 m²（+8%）** |
| **改动** | ① 默认对**就地提升**：`map/RMUC2026.pgm\|yaml` = `RMUC2026_v3` 那次会话的 2D 图（`577x326 @ 0.05 m`、`origin [−25.2,−9.09,0]`、占用 **9769** / 自由 **146198** / 未知 **32135**）、`PCD/RMUC2026.pcd` = 同一次会话的 3D 云（**203,352 点 / 6.51 MB**、8 字段、体素 0.10 m）⇒ 2D 与 3D **同源同场次**；旧份留 `*.bak-20261007`（人工命名，与工具自动的 `.prev-<时间戳>` **刻意区分**）；② 退役的合成资产进 `map/attic/`、`PCD/attic/`（**只搬不删**）；③ 命名规范写进 `map_assets.md` §1：`<world>_<tag>` 一个 `<tag>` = 一次会话的全部产物（`.posegraph/.data/.meta.yaml` + `PCD/<world>_<tag>.pcd`） |
| **原理** | ① **"哪一份是当前的"必须由文档回答，不能由文件时间回答**；② **"就地覆盖、不要 rm 再建"**：`install/` 是**逐文件软链**，删了重建会让软链悬空；③ 提升要**两个判据同时看**（"该表示的表示了" 与 "不该占的没占"）—— 只看前者会把图越描越胖；④ **2D 图有固有上限**：全场台阶边沿覆盖率四张都在 **35.98%~54.42%**、都够不到工具写的 80% 判据（激光扫不到台阶立面的下缘）⇒ 这是**性质**不是缺陷（`traversability_plan.md`） |
| **效果 / 回滚** | 回滚 = 把 `*.bak-20261007` 覆盖回 `map/RMUC2026.pgm\|yaml` 与 `PCD/RMUC2026.pcd`（`map_assets.md` §4.4 给了逐条命令）；`map_archive.sh restore` 走 `.prev-<ts>` 那条链 |
| **证据** | `map_assets.md` §0/§1/§2/§3/§4；`mapping_2d_from_cloud.md` §0/§6/§11；`worlds.md` §4；提交 `7f9e354` `2192024` `c8d65f9` `95846c3` |

**★ PCD→2D 的四个坑（都是实测）**

| # | 坑 | 数字 / 判据 | 处置 |
|---|---|---|---|
| 1 | **`/scan` 累积图里坡道 100% 是 unknown 或干脆在图外** | 坡面 **1680/1680 = 100% unknown**；走廊 **2688 格全在图外**（整条 6.72 m² 在窗口外） | 改用 `pcd_to_nav2_map.py`（点云投影）：坡面 **98.8% free**、走廊 **97.3% free** |
| 2 | **判据必须"相对局部地面"** | 0.05 m 栅格；占用 = 相对**局部地面**的高度超过阈值（默认 **0.15 m**，与 STL 管线同口径）**或**"又陡又高"；**缓坡（≤25°）保持 FREE**；法向 `k=8`、`--ground-cell 0.20 m`、p05；窗口/原点沿用**出生点相对系**（`amcl initial_pose=(0,0)`、gicp `initial_pose=[0,0,0]` **不用改**） | 规划器对坡道/走廊 5 个目标：`/scan` 图 **2/5**（3 个直接拒绝）→ 点云图 **5/5 全部给出路径** |
| 3 | **累积点云的"垂向拖影"会让 3D→2D 投影大面积误占** | `PCD/RMUC2026_lt.pcd` 在真值完全平坦（0.000±0.001 m）的 `map(−5,2)` 处：单帧累积点云的 **z 跨度 0.154 m**、单个 0.05 m 细格内 z 跨度中位 **0.085 / p90 0.169 m** ⇒ 重投影后 `occupied` **122.6 m²**（旧 `/scan` 图只有 22.7 m²），其中 **94.7% 的占用格真值局部高差 ≤0.10 m** | **实时链路不受影响**（判据跑在**单帧**上、没有累积拖影）；要重出先验图时，**更稳的路线 = 直接存"跑图那次的 `slam_toolbox` `/map`"**（对垂向拖影免疫，实测 **10.1 m² 占用 / 可行驶斜面误占 1.82%**） |
| 4 | **"图被掰歪的那一下"要能立刻报警** | `map_odom_jump_gate.py`（盯 `map→odom` 单步增量、超阈值打 `WARNING` + 发 latched `~/jumped`，**故意只报警不拦**）实测抓到：4 次跑共 **6 次单步跳变**，最大 **Δxy = 11.95 m、Δz = 7.36 m、Δyaw = 45.1°、Δmax-rot = 69.4°** | **只报警不拦**的理由：拦就得当**第二个 `map→odom` 发布者**（破坏"每条边恰好一个发布者"契约） |

### §2.5.2 续建 / 存档 / 会话身份：把"这次 launch 是谁"挂到 ROS 图上

| 项 | 内容 |
|---|---|
| **现象** | 用户：「能不能就是我在上次基础上继续建，手动指定一个地图名字去覆盖之类的」「以后换地图换场地会不会有干扰」；以及一次**真实事故**：「`save` 把位姿图写到**别人那套会话**的名字上」 |
| **原因** | ① `mode:=mapping` 起来时若同名存档已存在，slam_toolbox 会**反序列化并接着建**（`map_file_name` + `map_start_pose`），走完再 `save` 就是同名覆盖 ⇒ 缺的是"**场地/出生点守卫**"与"**存档名解析**"；② 事故的机理：`save` **信任了 `map/.session.yaml`**，而那个文件可能是**另一个还活着的会话**写的（同一台机上并行两套栈） |
| **过程（判决性测量）** | ① 换场地续建**默认拒绝并终止 launch**（除非显式 `map_allow_world_mismatch:=True`），依据 = sidecar `.meta.yaml` 记录的 world/spawn（`map` 系是**出生点相对系**，换 world 就是换原点 ⇒ 串味的地图叠在一起 = **假墙**）；② 续建后**一次性一致性检查**（`map_resume_check`：对不上就喊，`strict` 直接收栈）；③ 事故复盘留了原文与处置；④ 覆盖前**自动备份**（`.prev-<时间戳>`，默认留 **3** 代）+ `restore/backups` |
| **改动** | ① `mode:=mapping` 支持续建（`0a9c5b5`）＋ `cloud_accumulator` 让 **3D 先验也能跨会话续建**（map 系累积 + 同名存档 + 同一份场地守卫，`8b14aec`）；② `map_asset_guard.py` + `map_archive.sh`（`save/info/list/adopt`，`10f2a84`）；③ **会话播报器**（把"本次 launch 是谁"挂到 ROS 图上，`9d180ff`）；④ **`save` 严格解析会话名**（**活栈优先 / 多会话拒绝 / `--check`**，不再信任 `.session.yaml`，`02306fb`）；⑤ `cloud_accumulator` **数据卫生**（高度带 / 跳变速度闸 / TF 退化帧跳过）＋写前体检与备份（`59eb110`） |
| **原理** | ① **"同名覆盖"必须有守卫**：地图资产的价值在于"它属于哪个世界/哪个出生点"，这个信息必须在**文件里**（sidecar），不在人脑里；② **"谁写的"要有证据链**：会话播报器 + 严格解析 = 把"猜"换成"读 + 拒绝"；③ **续建 ≠ 拼接**：分块照建，但每块之间用**同一个存档名续上**，不需要手工拼（替代旧的"分块 + 手工拼接"建议）；④ **续建污染的边界要说清**：污染是真的（PCD z 到 +23.10 m），但它**不在实时位姿链路上**（§2.3.6） |
| **效果** | 续建日志（t=0 就打）会明确写出"从零建图 / 续建 + 绝对路径 + 存档 world/spawn"；`save` 在"多会话"情况下**拒绝**而不是猜；`map_archive.sh info/list` 可查每一代 |
| **回退** | 续建 = 删/改名存档后重新起栈；`save` 严格解析 = `git revert 02306fb`（**不推荐**：那会恢复"信任 `.session.yaml`"的事故面）；换场地允许 = 显式 `map_allow_world_mismatch:=True` |
| **证据** | `continue_mapping.md` §0/§3/§7/§10/§11；`map_assets.md` §1/§4；提交 `0a9c5b5` `8b14aec` `10f2a84` `15d5c57` `133742d` `9d180ff` `02306fb` `59eb110` |

### §2.5.3 世界资产 `RMUC2026`（由附件 STL 生成的全场）

| 项 | 内容 |
|---|---|
| **现象** | 需要一个"RMUC 级别全场"用来跑建图/导航；附件是 `easystl.stl`（毫米单位、Z 朝上、29.15×16.05 m） |
| **原因 / 改动** | 三处相对 `RMUL2026_world.world` 的改动：`<scale> **0.001 0.001 0.001**`（本 STL 原生单位 = **毫米**，bbox 29150×16050×1730；不缩放就是 1000 倍大）、姿态 **`0 0 1.6413436 0 0 0`（无旋转、z 抬 1.6413436 m）**、模型 pose **`0 0 0 0 0 0`**（让 **mesh 米制坐标 == world 坐标**，地图/PCD/出生点全部同一个系）；一条命令复跑：`tools/scripts/world/stl_to_world.py`（`worlds.md` §4.4） |
| **过程（判决性测量）** | 真实建图产物 vs STL 几何：把 `/map_save` 存的 22,947 点真实点云平移到 world 系后 bbox `x[−14.64,14.45] y[−6.19,9.56]`（STL 是 `x[−14.575,14.575] y[−6.401,9.649]`）；0.05 m 栅格上 **87.5% 的"竖直结构"格落在 STL 障碍格 5 cm 之内**（mean chamfer **0.019 m**）⇒ **毫米缩放、z 抬升、模型 pose 三件事都被实测证明是对的**；四种形态实测：**建图 ✅ / 2D+AMCL 导航 ✅ PASS / 3D+GICP 导航 ✅ PASS**（含一条"定位仍在缓慢收敛"的打折项） |
| **原理 / 场地对本车的坑** | ① **地图 free 237.8 m²，但 `clearance ≥ 0.22 m` 的只有 178.6 m²（同一连通域 176.3 m²）⇒ 真正能走的 ≈ 176 m²（占 free 的 74%）**；② **两个半场之间最窄处通道宽 0.45 m**（车直径 0.44 m）⇒ 余量 5 mm/边、**实际过不去**；A\* 给出的过场路线长 **41.7 m（绕场边）**；③ 台阶 **0.20 m 与 0.30 m**（轮半径 0.06 m ⇒ **物理上上不去**）；④ 目标点应当选在**出生点所在的半场**，跨半场的目标请当作"已知不可达" |
| **回退** | 世界/地图都是文件级资产：`map/attic/`、`PCD/attic/` 里留着历史版本；重跑一条命令即可再生成 |
| **证据** | `worlds.md` §0–§6；提交 `7c0006b` `280888e` `eb7b811` `7e7b823` |

## §2.6 工具与方法论（**本期可复用的部分**）

### §2.6.1 四条已经制度化的做法

| 做法 | 具体是什么 | 本期最有说服力的例子 |
|---|---|---|
| **① 判决性测量（先给判据，再造仪器）** | 面对"是不是 X 造成的"这类问题，先问"**如果 X 成立，我应该量到什么**"，然后造一个**只读**仪器去量那个量；测量结果必须能**否掉**候选 | "缺外参导致 odom 被转 30°"这条假设（H1）：判据 = "把 `/cloud_registered` 按那条 TF **反变换**回去，如果 odom 本来斜，反变换后会变成 60°"⇒ 实测 **1.041°** ⇒ **假设被否**（`Fidelity` §I.5.1） |
| **② 绕过被怀疑的那一层** | 当"控制器/规划器/代价图"都可能是原因时，**把 nav2 整个摘掉**只留物理 | `--push 0.25`：robot11 只能走 **0.4199 m** ⇒ 根因是实体障碍，不是控制器（§2.4.2） |
| **③ 反向验证（防止"修好了但修坏了别的"）** | 每个"改好一个指标"的改动，都要问"**它有没有让另一件事变坏**" | `update_rate 1000 Hz` 让角速度 1.03% → 7.8%，**但**盲推 0.408 → **2.565 m**（车爬上障碍）⇒ 否掉（§2.1.7）；限速开了以后**真台阶仍然限速**（`why=step` 28/75 帧）才算没把功能关掉（§2.1.6） |
| **④ 未验证清单制度** | 每份报告末尾**必须**有"未验证/诚实清单"，写明"哪些是单跑、哪些是无头、哪些只是推断"；**禁止**把推断写成事实 | 本文 §5 就是把各文档的未验证项去重合并的结果 |

### §2.6.2 "默认路径逐字节不变 + 一键回退"是怎么做到的（三种机制）

| 机制 | 怎么做 | 验证方式（不是"我说没变"） |
|---|---|---|
| **槽位覆盖文件**（`robot:=` / `ground:=` / `local_obstacle:=`） | 只为该槽位生成一份**逐键副本**或**只含几何键的覆盖层**，launch 深合并后传给节点；其它槽位**原样返回公共段路径**（不合并、不写临时文件） | ① 单元级：同一份代码三种槽位下"传给 nav2 的路径/值"逐项列出（默认/hzmirm 仍 0.22，robot11 = 0.3565）；② 运行期：`ros2 param get` 回读（`robot_radius` 真的是 0.3565） |
| **开关缺省 = 旧语义** | 插件 `<tilt_rpy>` 缺省单位阵、`<cloud_frame>` 缺省 `parent`、`<range_from_origin>` 缺省"修好的语义"、`self_mask_enable` 默认 false、`obstacle_near_ground_m` 默认 0.0、`gate_enable/smoothing_enable` 默认 true… | ① **结构性论证**（"缺省分支就是 2026-10-07 起逐字节相同的代码路径"）；② **实测对照**（默认模型跑一遍，逐项与改动前同量级：如 6256/2739 点/帧 vs 6270/2707）；③ **渲染 diff**（`xacro` 两档去注释后逐字节相同、35410 B） |
| **生成物由生成器拥有** | `sentry_robot_robot11_sim.xacro` 头部写"本文件是生成的，不要手改"；生成器 `robot11_make_sim_xacro.py` 可**逐字节重算** | `python3 … --out .tmp/…xacro && diff` **为空** ⇒ "生成器与入库的 xacro 逐字节一致"；清单缺失或 `all_ok=false` ⇒ 生成器**直接报错**（不生成"看起来对但没人验证过"的模型） |

### §2.6.3 跑法纪律（每一条都是踩过坑换来的）

1. **一条 bash 调用 = 一个 PID namespace**（本仓 `bwrap --unshare-pid --die-with-parent`）⇒ **启动 / 驱动 / 收尾必须塞进同一次调用**，否则进程全清、量到的是残局。
2. **隔离四件套**：`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、按 tag 哈希占用的 `GAZEBO_MASTER_URI`、`unset DISPLAY`（无头）。
3. **只杀自己的进程**：按 `/proc/<pid>/environ` 核对 master URI 后再 kill；**绝不做全机 `pkill -9 -x gzclient`**（用户那两次 `exit code -9` 就是这么来的，§1.2 第 16 条）。
4. **publisher 必须等机器人稳定后再注入**：在"spawn + 插件加载"期间出现 publisher 时，实测 `gzserver` segfault（exit −11）/ `gzclient` abort（exit −6）+ spawn 服务超时（`Fidelity` §H.6）。
5. **采样不能饿死**：单线程 `rclpy.spin_once()` 在 ~80 Hz `/tf` 洪水下会被饿死 ⇒ 造出"LIO 位移是真值 2.6 倍"的**假结论**；正确做法 = `MultiThreadedExecutor` + `ReentrantCallbackGroup` + 回调只做 O(1) 赋值 + **独立落盘定时器** + 每行带 `*_age_sim`（`lio_divergence_no_impact.md` §1.1）。
6. **单位与口径要写死**：① **"12 s"是墙钟还是仿真秒**（RTF 0.35 vs 1.00 会差 3 倍 ⇒ 执行率一律用**仿真时间**归一）；② **RTF 不可比**（同一批里可比、跨批不可比：0.23/0.36/0.69 排序不可复现）；③ **度/弧度**（`gt_yaw_delta_deg` 多乘 57.2958 那个 bug）；④ **分带百分比的分母**（`/scan` 用"有限波束 833 条"，不是全部 1462 个 bin）；⑤ **同一帧**（`ground`/`obstacle` 两朵云是异步到的、帧号不同 ⇒ 离线分析只用**唯一自洽**的那朵 `raw`）。
7. **破坏性实验放最后**：会**不可逆带坏状态**的段（高角速度甩转）排到路线末尾，否则前面所有"干净段"的结论全部作废（`lio_drift_diagnosis.md` §2.2）。
8. **一次只改一项**：把 5 项一起改的那次（`missing_data_ray_length` + `hit/miss` + `num_range_data` + `optimize_every_n_nodes`）留存率反而从 49% 掉到 30%，指向我多加的两项 ⇒ **回滚、一次一改**（`issues_and_findings.md` #26 七次结论）。

### §2.6.4 本期的工具地图（只列"能被别人复用"的）

| 目录 | 工具（举例） | 干什么 |
|---|---|---|
| `tools/scripts/regress/` | `robot_model_probe.py`、`run_robot_model_probe.sh`、`robot11_{mesh_inventory,geometry,gz_collision_bench,livox_fov,self_mask,make_sim_xacro,weld_chassis,decimate_visuals}.py`、`check_robot11_visual_slot.py`、`verify_low_terrain.py`、`check_slope_speed_table.py`、`check_traversability_criteria.py`、`analyze_slope_speed_ab.py`、`linefit_{offline,replay}_probe.py`、`map_prior_ab.py` | 模型资产量测 / FK 复算 / 断言 / 离线判据台架 / 低地形验收 / 限速真源对账 / A-B 指标提取 |
| `tools/scripts/tiltmount/` | `tilt_mount_probe.py`、`run_tilt_mount_probe.sh`、`tilt_mount_compare.py`、`tilt_chain_check.py`、`tilt_ab_table.py`、`tf2_compose_order_test.cpp`、`costmap_input_dump.py`、`near_ground_replay_probe.py`、`ground_leak_probe.py`、`p2l_forecast.py`、`nav_goal_forensics.py`、`nav_goal_report.py`、`run_nav_goal_forensics.sh`、`run_chassis_yaw_bench.sh`、`chassis_yaw_probe.py` | 斜装"帧/射线/外参/外方位"取证、逐帧点账本、代价图圆/分带、**绕过 nav2 的盲推**、可达性前置判据、底盘角速度台架 |
| `tools/scripts/diag/` | `nav_lio_timeline.py`、`run_nav_lio_timeline.sh`、`analyze_noimpact_divergence.py`、`map_odom_jump_gate.py`、`segment_drive.py`、`lio_drift_analyze.py`、`run_drift_diag.sh` | 50 Hz 只读时间轴 / 归因分析 / `map→odom` 跳变哨兵 / 分段驱动 + 逐段 ATE |
| `tools/scripts/mapping/` | `coverage_drive.py`、`map_archive.sh`、`map_asset_guard.py`、`map_resume_check`、`pcd_to_nav2_map.py`、`mapping_ab_run.sh`、`ramp_section_probe.py` | 跑图 / 存档与守卫 / 续建一致性检查 / 点云→2D / 建图 A-B |
| `tools/scripts/localization/` | `run_nav_smoke_regress.sh`、`nav_smoke_regression.py`、`compare_regress_snapshots.py`、`run_gicp_nav_ab.sh`、`gicp_nav_watch.py` | **P0 冒烟回归**（改任何东西之后都要跑）/ GICP A-B |
| `tools/scripts/world/` | `stl_to_world.py` | 场地 STL ⇒ world + 2D 图 + PCD 一条命令 |
| `tools/scripts/control/` | `improved_teleop.sh` | 键盘接管（⚠️ 松手必须显式发零速） |
| `bench/`（包内） | `slope_speed_offline.cpp`、`ground_seg_ab`、`gicp_selfsim_probe` | **不需要 Gazebo** 的离线台架（调参/验收的第一道闸） |

## §2.7 `tdt-nav-kit` 研究结论（**研究，未落地**）

> 触发 = 用户原话：「这个自己研制怎么自己研制呢，因为这个纯 c++ 吧，我们这个又算强依赖 nav2 吧，
> 做成 nav2 的插件吗，还是有办法保持纯 c++ 的形态…会不会影响原来纯 c++ 的运行效率呢」。
> 结论落在 `docs/nav2_vs_custom_planner.md`（**纯新增文档 + CMake 骨架，没有一行代码落地**，提交 `d4965bd`）。

### §2.7.1 三种接法与"该选谁"

| 维度 | **A. nav2 插件** | **B. 独立节点** | **C. 独立进程 + 共享内存/零拷贝 IPC** |
|---|---|---|---|
| 要写多少代码 | 一个薄适配类（Humble 的 `GlobalPlanner` 只有 **5 个纯虚函数**，其中 4 个可留空）+ pluginlib xml + 参数段 | 一个完整节点：TF/位姿、costmap 源、控制律、到达判据、恢复策略、生命周期 | B 的全部 + 共享内存/IPC 协议 + 进程编排 |
| 库还是纯 C++ 吗 | **是**（库一行不改，ROS 依赖只在适配层） | **是**（库编进 ROS 节点，但库本身无 ROS 头文件） | **是** |
| 每次规划的额外开销 | 一次虚函数调用 + action 收发（**都不是主要成本**）；costmap 是**指针原地读** | 取决于 costmap 从哪来：订 `costmap_raw` 会**序列化 + 拷贝**；自己跑 costmap 层则等价于 A | 一次 IPC 传输。**同进程**才能真零拷贝；跨进程只能靠 SHM/loaned message |
| 必须自己重实现 | 只在插件语义范围内（planner 只需 `createPlan`） | nav2 帮你做的一切（BT/恢复、目标管理、costmap 图层与 TF、`/cmd_vel` 独占、进度检查） | 同 B + IPC 编解码/一致性 |
| 与 nav2 生态 | 最好（RViz/`/plan`/BT/waypoint 全在） | 差（都要重新接线） | 中 |
| 何时选 | **默认选它** | 当 nav2 的**抽象与问题不匹配**（地形不是"空地/障碍"二值、底盘动力学不是差速/全向）时 | 只有在**已证明**规划耗时/抖动是瓶颈、且同进程不可行时（**本次没有找到任何论文/仓库用它跑规划器**） |

**推荐的一句话版**：把库保持原样（纯 CMake、无 ROS），在它旁边加一个 ament 包做薄适配；
**先做 `GlobalPlanner` 插件**（A\* 前端 + min-snap 后端，输出 `nav_msgs::Path`），
把"带时标的 min-snap 轨迹真正被执行"这件事留给后续的 Controller 插件或独立节点。

### §2.7.2 效率账（谁才是瓶颈）

| 项 | 量级 | 结论 |
|---|---|---|
| pluginlib `dlopen` + `configure()` | 生命周期期**一次** | 一次性开销，可忽略 |
| 每次规划的虚函数调用 + action 收发 | 纳秒~微秒级 | **不可能成为瓶颈** |
| **库自己的耗时**（上游公布） | A\* **平均 5.4 ms / 最大 14.8 ms**；min-snap **平均 3.9 ms / 最大 126.7 ms** | ⚠️ **min-snap 的最坏 126.7 ms 落在 20 Hz 控制环里会顶掉 2–3 拍** —— 这才是要担心的 |
| costmap 取用方式 | planner/controller 形态 = **指针原地读**；smoother 走 `CostmapSubscriber`（`nav2_msgs/Costmap` 的 `uint8[] data` = 560×300 = **168 000 B**，**每次更新都要序列化/反序列化一遍**） | 优先选"指针"那条路 |
| RMW/IPC、执行器抖动 | nav2 自己测过（文档给了来源） | 要 settle 必须在**我们机器上**测（文档 §3.6 直接给了实验设计清单） |

### §2.7.3 三条"接口语义级"的坑（**这条最值得记**）

1. **nav2 原生没有任何"带时间的轨迹"接口**（逐条证据）：① 消息层 `nav_msgs/Path` **只有 `header` + `poses[]`**，`nav2_msgs` 的 14 个消息里**没有 Trajectory 类**；② 动作层 `FollowPath` 只收 `nav_msgs/Path`；③ 接口层 `Controller` 只吐**一拍** `TwistStamped`，`Smoother` 是 `Path→Path`；④ **nav2 自己还会主动丢弃 Path 上的时间戳**：MPPI 的 path handler 把每个 pose 的 `header.stamp` **覆盖成当前位姿的时间戳**（`path_handler.cpp:71,86`）⇒ "把时标塞进 `poses[i].header.stamp`"这条路**在 nav2 自带控制器上会被无声抹掉**；⑤ 唯一"带时间"的东西是 MPPI **内部**的候选轨迹（对外只输出第一拍）；⑥ 别家怎么做：无人机栈干脆**不用 `Path`**（EGO-Planner 用自定义 `quadrotor_msgs/PositionCommand`，时间参数化直接进消息定义）。⇒ 本库的 `dt = 0.1 s` 是**隐式时标**（相邻点间距），一进 `Path` 就只剩"约定"。
2. **`0 = FREE` vs `0 = 障碍` 的值域陷阱**：nav2 的 master grid 是 `0 = FREE_SPACE / 253 = INSCRIBED / 254 = LETHAL / 255 = NO_INFORMATION`（`cost_values.hpp:42-46`），而本库的 `YAstar::setMap(int w, int h, u_char* mapData)` 把数据映射成 `x == 0 ? 1.f : 0.f` ⇒ **0 = 障碍、非 0 = 自由**（`yastar.cpp:404-417`，阈值 `occThs = 0.5f`）。**直接把 `getCharMap()` 喂进去 = 整张图取反**（自由变障碍）；另一个重载 `setMap(..., std::vector<int8_t>&)` 用 `x == 0 ? 0 : 1`，符合直觉但**会把 `−1`（unknown）当成障碍** ⇒ **必须自己写显式转换函数 + 一个单元测试**。
3. **`initCostMap()` 是硬前提**：库自己写着"调用 search 之前必须调用 `initCostMap()` 或其它设置 costmap 的函数"（`yastar.hpp:174-175`）—— **忘了不会报错，只会得到奇怪路径**（这类"静默失效"在本仓已多次出现，见 §3 的"静默失效"原则）。

---

# §3 原理与契约总纲（把"为什么这么改"提升成可复用的原则）

| # | 原则 | 一句话 | 本期最硬的例证 |
|---|---|---|---|
| **P1** | **帧是一等公民** | 同一份数据配上错误的 `frame_id`，会让"物理正确的模型"在 RViz/nav2 里看起来全错 | `urdf` 档：点云逐点相同（0.0004°）而 `/cloud_registered` 地面 30.970°（§2.3.2） |
| **P2** | **`frame_id` 是承诺，不是标签** | 写 `odom` 就必须真在 odom 里；写 `livox_frame` 就必须真在那个帧里 | 三个 `livox_frame` 的分辨（§0.1）；bug ② 的修复（§2.3.2） |
| **P3** | **契约 = 每条 TF 边 / 每个话题恰好一个发布者** | 多发布者 = 抖动 + 结论不可信；"拦一下"这类需求也不能靠抢发布权实现 | `map→odom` 单发布者、`base_link→base_link_fake` 双发布者被消除（P4）、跳变哨兵**只报警不拦**的理由（§2.5.1 坑 4） |
| **P4** | **参数归属 = 参数属于某个帧、某个原点、某一层** | "高度/半径/距离"这类参数**必须**说清"它属于哪个帧"；说不清就会调错对象 | `obstacle_layer` 高度带的真实语义（§2.2.5）；`p2l` 高度带跨帧换算（§2.2.6）；`robot_radius` 是内切还是外接（§2.2.2） |
| **P5** | **表示分离：视觉 / 碰撞 / 传感器 / 代价图是四种用途** | 同一个物体在不同用途下**应该**用不同几何/不同网格；把它们混成一个才是错 | CAD 原网格：视觉可以（但要抽稀）、碰撞绝对不行（§2.1.1）；视觉抽稀**不可能**改变点云（射线走 ODE、只与 `<collision>` 求交） |
| **P6** | **降维发生在哪一层是设计选择，但"信息丢失点"必须知道** | 一旦跨过某条边界（`p2l`）信息就永久丢失 ⇒ 想用那个信息就必须在边界之前动手 | 2D 链路上唯一能用"高度"的位置 = 近地剔除（§2.2.1）；`min_obstacle_height` 不是障碍物高度闸（§2.2.5） |
| **P7** | **刚体假设是物理引擎的前提，不是可选项** | `planar_move` 的 `Set*Vel` 只对单刚体自洽；"没有执行机构的自由度"是纯负担 | 底盘塌成单刚体：角速度 1.03% → 16.2%（§2.1.7） |
| **P8** | **先量后改；每个"修"前面要有一次判决性测量** | 判据先行（"如果 X 成立，我应该量到什么"），测量必须能**否掉**候选 | H1/H2/H3 三条假设各自被一次测量判决（§2.1.5）；六候选逐条判决（§2.4.2） |
| **P9** | **反向验证：改好一个指标不算完，要问"它有没有让别的变坏"** | 这正是"能跑通"与"没造假"的分界线 | `update_rate` 1000 Hz 让车爬上障碍（§2.1.7）；限速后**真台阶仍限速**才算没关功能（§2.1.6） |
| **P10** | **单变量 + 破坏性实验放最后 + 一次只改一项** | 多变量同改会让结论无法归因；不可逆状态必须先排到末尾 | 11 次跑 / 9 个配置的主表（§2.4.4）；甩转段放最后（§2.3.6）；五次同改那次的反例（§2.6.3 第 8 条） |
| **P11** | **唯一真源（single source of truth）** | 同一件事只在一处维护；"横幅/日志"必须**运行时读生效值**，不能写"意图" | `traversability_criteria.yaml` 的 `speed_limit_*` + `check_slope_speed_table.py` 断言两处逐键一致（§2.4.1）；横幅改成 `_YamlKeysReadout`（`robot_models.md` §12.4） |
| **P12** | **默认路径逐字节不变 + 一键回退 + 生成物有生成器** | 任何改动都要能回答"默认路径变了吗""怎么退回去" | 三种机制（§2.6.2）；`model://` 自动化里"包侧 export 是全局 env 增量"这条偏离被明确登记（§2.1.4） |
| **P13** | **仿真 ↔ 实车：先分三类再动手** | "改感知输出内容"（❌ 拒绝）／"消除仿真失真"（✅ 该做）／"世界本身的差异"（✅ 参数分家） | `sim_real_contract.md` §四/§五；"不许为了让仿真跑得动而改感知输出内容"（算力账要在仿真侧还） |
| **P14** | **优先怀疑"静默失效"，而不是"算错了"** | 本期的多数疑难都是"没报错但什么也没做/做错了" | 忘了 `initCostMap()` 只会得到奇怪路径（§2.7.3）；`gravity_aligned_frame` 一开就 0 点（§2.2.4）；`pure_localization` 写错层级 ⇒ 永不生效 + 析构 FATAL（`issues_and_findings.md` #15）；队列深度只是把"等待"吸收掉（§2.3.1） |
| **P15** | **证据分级与诚实边界** | 实测 / 推断 / 未验证要分开写；未验证清单是交付物的一部分 | 本文 §5；`Fidelity` §G/§I.10/§J.8/§K.8/§L.10/§M.10 |
| **P16** | **"观感"要翻译成可判定的量** | 用户说"斜了/飘了/走得慢"，都要先翻译成"哪个量、在哪个帧、什么口径" | "像平放扫到的东西被倾斜" → 刚性旋转 vs 覆盖图案（§0.1）；"慢" → 其实是"走不到"（0/4 到点，§2.4.1） |

---

# §4 总账表

## §4.1 所有 before → after 的实测数字（按层）

> 口径：除非注明，均为**无头隔离跑**、`lio:=small_point_lio`、同一世界/出生点；
> "证据"列的 `§` 指原文档章节（本文相应条目在 §2 里）。

| # | 指标 | 口径 / 条件 | 修前 | 修后 | 证据 |
|---|---|---|---|---|---|
| 1 | `base_link.STL` 碰撞的接触工况 RTF | ODE quick/50、1 ms 步长、有接触 | **0.200**（最低 0.04，窗口内只推进 0.49 s） | **1.002**（4 box） | `robot_models.md` §9.3 |
| 2 | 峰值 RSS（同一工况） | 同上 | **760 MB** | **174 MB** | 同上 |
| 3 | 视觉面数（12 个 mesh 合计） | 磁盘/清单 | **2,821,320** | **163,995（−94.2%）** | §13.3.3 |
| 4 | 视觉磁盘 | 同上 | **141.07 MB** | **8.20 MB** | 同上 |
| 5 | 抽稀误差（最差） | 包围盒 / `out→src` p99 / 剪影 IoU | — | **0.297 mm / 0.18 mm / 0.9919** | 同上 |
| 6 | 自击 `r<0.12 m` 占比 | robot11 原始云 | **75.5%**（Phase 2 实心 4 box） | **29.0%**（A 方案挖视锥；剩下的是物理真实云台回波） | §11.2 |
| 7 | `/segmentation/ground` 点/帧 | 同上 | **529** | **5637**（默认模型 2739） | 同上 |
| 8 | `/odom` 消息数 | 同一窗口 | **0**（有发布者） | **188** | 同上 |
| 9 | `/scan` >4 m 波束/帧 | 同上 | **0** | **≈81** | 同上 |
| 10 | `/scan` 有限束数 | 同帧 A/B（改 `max_height`） | **856.8**（linefit）/ 1113.8（patchwork） | **934.7（+9.1%）** / **1163.5（+4.5%）** | `ground_segmentation_slots.md` §11 |
| 11 | 0.40–1.00 m 高度带覆盖率 | 同一朵 obstacle 云 | **0.0%** | **100%** | 同上 |
| 12 | `/scan` 波束在 odom 里超出 `[0,2] m` 的比例 | `sensor` 档 | 0.404 | **0.3960**（❌ 未达成：2D 链路结构上定不了基） | `Fidelity` §K.1.3 |
| 13 | 车那格代价 | robot11 静止、近地剔除 | **42（非 free）** | **0（FREE）** | §K.2.3 |
| 14 | 车半径圆内 `≥99` / free | 同上（半径 0.3565） | **156 / 239** | **0 / 999** | 同上 |
| 15 | 到最近 lethal（车心） | 同上 | **0.531 m** | **1.534 m** | 同上 |
| 16 | **全局** 车那格 | 静止、三档几何 A/B | **99**（0.3565 外接） | **0**（0.300 内切） | §K.3.1 |
| 17 | 到最近 lethal（全局） | 同上 | **0.135 m** | **0.804 m** | 同上 |
| 18 | 原始云"地面峰" | robot11、插件 A/B | **−0.280 m**（随距离变，像斜面） | **−0.155 m**（尖峰） | §K.4.2 |
| 19 | 车半径圆内 `≥99`（插件修复后） | 同上 | **13** | **0** | 同上 |
| 20 | `gravity_aligned_frame: base_link` 的 ground 点/帧 | 同一帧、活的节点 | **0** | **3969**（默认路径 3969 → 3969 不变） | `robot_models.md` §11.4 |
| 21 | `/scan` 丢帧 | 同一条路线、`scan_queue_size` 保持 1 | **116（4.3%）**，2.5 s 节律 | **10（0.35%）**，节律消失 | `slam_toolbox_scan_drops.md` §9.4 |
| 22 | 等待 TF 的 p90 | 同上 | **136 ms** | **13 ms** | 同上 |
| 23 | `/cloud_registered` 地面倾角 | `urdf` 档 | **30.970°** | **1.054°** | `Fidelity` §J.1.1 |
| 24 | `odom→base_link` 假俯仰 | `urdf` 档（真值 0.003°） | **4.890°** | **0.317°** | §J.1.2 |
| 25 | `map→odom` 逐次更新步长 p95 | GICP 静止 A/B（同一二进制） | **0.0087 m** | **0.0005 m（17×）** | `gicp_divergence_and_jitter.md` §4.2 |
| 26 | `map→odom` 累计修正路程（150 s） | 同上 | **4.085 m** | **0.247 m（16.5×）** | 同上 |
| 27 | 融合位姿逐样本跳变 p95 | 同上 | **0.0048 m** | **0.0020 m（2.4×）** | 同上 |
| 28 | 发目标前"等定位稳定"判据 | P0 回归 | 旧文档 **3.4 cm / 30 s** | **0.0020 m / 5 s** | 同上 §4.4 |
| 29 | `/initialpose` 生效延迟 | 饱和（PCL 400~490 ms/帧 @10 Hz） | **22.7 s**（且只在停流后被处理） | **0.24 s**（= 一个 align 周期内） | `gicp_initialpose_latency.md` §4.2 |
| 30 | 位姿图节点间距 | 同路线/速度/3 分钟 | **0.797 m** | **0.313 m** | `slam_toolbox_tuning.md` §4.1 |
| 31 | 环秩 / 回环修正事件 | 同上 | **1 / 0~1** | **8 / 6** | 同上 |
| 32 | 受控实验：TF 晚发对丢帧 | 完美里程计（ATE 0.000 m） | 按时发 ⇒ **0** | 晚 130 ms ⇒ **62（4.26%）** | `slam_drops_loopclosure_odom.md` §0 |
| 33 | IMU 全程峰值 | 整栈 A/B（同目标） | **110.8 m/s²** | **50.8（−54%）** | `slope_speed_limiting.md` §3.1 |
| 34 | 结局（到点） | 同一张图 / 同一目标 | **0/4 到点**（1 ABORTED + 3 超时） | **2/2 到点**（~27 s / 22.6 m） | `lio_divergence_no_impact.md` §3.3 |
| 35 | `|v|` p50（只比没发散的那一对） | 同上 | **0.05 m/s** | **1.16/1.19 m/s** | 同上 |
| 36 | 平地回归用时 | 2.9 m 路线 | **3.2 s**（限速关） | **3.6 s**（限速开，+0.4 s，单次差值） | `slope_speed_limiting.md` §3.3 |
| 37 | 自击掩膜剔出的点/帧 | robot11 静止 | **0** | **3374（≈29.4%）** | `robot_models.md` §12.3.3 |
| 38 | 走廊最近格 `data_min_d` | 同上 | **0.00 m**（37/37 帧） | **0.20 m**（46/46 帧） | 同上 |
| 39 | 近场 max 台阶残差 | 同上 | **0.023~0.032 m**（日志里出现过 0.128~0.203） | **0.002~0.005 m** | 同上 |
| 40 | "参考坡度" | 同上 | **0.0°**（被自击格钉住） | **2.9~3.1°** | 同上 |
| 41 | 静止时限速落在地板的帧数 | 同上 | **9/129**（日志行） | **0/46** | 同上 |
| 42 | 计划路径到内切带距离 p50 / 执行余量 p50 | smac2d 平滑器 | **0.25 m / 0.57 m** | **0.335 m / 0.85 m**（健康段 0.75 → **1.03**） | `path_clearance_and_contact.md` §2.1/§3.1 |
| 43 | 角速度执行率（台架，`wz=1.0`×12 仿真秒） | 同一世界/出生点/命令 | **1.03%（+7.09°）** | **16.2~16.5%（+111.7…113.3°）** | `Fidelity` §M.3 |
| 44 | 角速度执行率（整栈） | 盲发 `wz=1.0`×12 s（请求 687.5°） | **0.55%（真值 +3.79° / LIO +3.86°）** | **5.15%（+35.42° / +33.75°）**（优于默认模型 3.98%） | §M.7 |
| 45 | 60 s 无指令自由漂移 yaw | 整栈 | **+8.20°** | **+0.006°**（LIO −0.091°；默认模型 −0.154°） | 同上 |
| 46 | 反向 **1.2 m** 目标 | 在可达区内、闸门放行 | 未测 | 真值位移 **1.2102 m**、残余 **0.0794 m**、**`Goal succeeded`** | 同上 |
| 47 | 纯物理盲推极限（正前 / 正后） | 绕过 nav2，25 s 腿 | **0.4199 / 1.5753 m** | **0.3992 / 1.4370 m**（可达性判据取值） | §M.7/§M.8 |
| 48 | `--goal-forward 0.5` 真值位移 / 残余 | `slam_nav` | 0.313 / 0.170 m（§K 口径） | **0.2977 / 0.1902 m**（同协议复跑） | §L.1.3 |
| 49 | 缓坡被判障碍的比例 | STL 真值、22~35° | linefit **35.7~44.3%** | patchwork **2.5~3.0%** | `ground_segmentation_slots.md` §5.3/§6 |
| 50 | `/scan` <1.5 m 近距假回波 | 整栈 smoke | **91.32**（linefit） | **34.43**（patchwork，−2.65×） | 同上 §6.1 |
| 51 | 分割器 CPU | 同上 | **0.0836 核** | **0.0562（−33%）** | 同上 |
| 52 | 0.15 m 薄墙覆盖（`th_dist`） | 合成台架 | **11.8%**（0.125） | **100%**（0.08） | 同上 §3.1/§5.3 |
| 53 | 2D 先验：全场台阶边沿被表示 | `verify_low_terrain.py` | **49.44%** | **52.71%**（同窗口对照 54.42%） | `map_assets.md` §3.1/§3.2 |
| 54 | 2D 先验：撞击窗 `map(−3.6,3.1)±0.6` | 同上 | **28/33 = 85%** | **31/31 = 100%** | 同上 |
| 55 | 2D 先验：可行驶斜面被标占用 | 同上 | **3.29%** | **5.21%**（其中"离真值边沿 >0.20 m 的真障碍" 0.57% → 0.90% = **+0.05 m²**） | 同上 |
| 56 | 3D 先验点数 / 足迹 | GICP 自相似探针 | **178,543 / 415.6 m²** | **203,352 / 449.3 m²** | 同上 §3.3 |
| 57 | 累积点云垂向拖影 → 重投影误占 | 真值平坦处 `map(−5,2)` | `/scan` 图 22.7 m² | 累积点云图 **122.6 m²**（94.7% 的占用格真值局部高差 ≤0.10 m） | `traversability_plan.md` §9.1 |
| 58 | 建图 `/map` 占用格 / 已知格 | 整栈 smoke | linefit 6544 / 184 537 | patchwork **7756（+18.5%）/ 216 322（+17.2%）** | `ground_segmentation_slots.md` §6.1 |
| 59 | LIO 正常工况漂移 | 分段路线（51.4 m / 408 仿真秒） | — | 直线 **0.0105 m/m**、窄沟 **0.0031 m/m**、整程 ATE RMSE **0.036 m** | `lio_drift_diagnosis.md` §0 |
| 60 | 高角速度甩转 | 6 次同档 | — | **6 次坏 3 次**（最坏位置跑飞 10.68 m、`map→odom` 13.34 m） | 同上 |
| 61 | PCL GICP 单帧 align | 构建口径修复（`colcon_defaults.yaml`） | **369.9 ms** | **11.3 ms（32.7×）** | `build_optimization.md`；提交 `6ae2ada` |
| 62 | 可通行面积（RMUC2026） | clearance ≥ 0.22 m | free 237.8 m² | **≈176 m²（74%）**（最窄通道 0.45 m，车直径 0.44 m） | `worlds.md` §6 |

## §4.2 ★ 被否掉的候选修法（连同否掉它的数字）

| # | 候选修法 | 为什么看起来合理 | **否掉它的数字** | 证据 |
|---|---|---|---|---|
| 1 | 把插件 `update_rate` 提到 **1000 Hz** | 每 10 ms 才重设速度，接触/约束有 10 个物理步去掰回来 | 角速度确实 → **7.8%**，**但**纯物理正前方盲推 **0.4077 → 2.5652 m**、底盘 z **0.1522 → 0.4005 m（爬上障碍）**、60 s 漂移 0.308 → 0.297°/s（**没解决**） | `Fidelity` §M.5 |
| 2 | GICP **紧门限**（0.8 m/s、cap 0.12 m、20°/s） | "按健康 p99 × 2 定阈值"是常见做法 | 拒绝率 **18%**，拒的是**合法修正**（创新 0.1226 m / 0.538°、fitness 0.0024）；发布值被冻住 ⇒ 创新 0.31→0.50→**0.95→2.24 m** ⇒ 连续 25 帧判失效（**死锁**） | `gicp_divergence_and_jitter.md` §5.1 |
| 3 | **只平滑输出**、初值仍取上一帧测量 | "输出平滑了就不抖了" | 静止 120 s 漂 40 cm、逐样本跳 16 cm **照旧**（测量链本身仍是"增益 1 的积分器"） | 同上 §5.2 |
| 4 | 用**真足印多边形**替代 `robot_radius` | 几何上最正确 | `rcl` 把嵌套序列当 key ⇒ `Couldn't parse params file … Sequences cannot be key at line 127` ⇒ **所有** nav2 节点一起 exit、整条导航链起不来（正确写法必须转成**字符串**）；且量到的收益只剩"转弯角不扫障碍" | `Fidelity` §K.3.4 |
| 5 | 调大 **`scan_queue_size`** 消丢帧日志 | 日志里就是"queue is full" | 队列 2 ⇒ `queue is full` **62 → 0**，但同一跑出现 **70 条另一种丢帧**，位姿图节点 **58 → 35** ⇒ **总量没减少，只是换了名字** | `slam_drops_loopclosure_odom.md` §0 |
| 6 | 只放松回环 **`chain_size`/半径/阈值** | "阈值太严所以不回环" | 只放松 chain 10→4 ⇒ **环秩全程 0**；chain4 + 半径 4 + 阈值放松（节点间距不变）⇒ 环秩只到 **2** | `slam_toolbox_tuning.md` §0/§4.1 |
| 7 | **收紧**回环阈值治"地图跟着车转" | "门槛太松 ⇒ 误回环" | 收紧后 `Score histogram: **Count: 0**`（一条回环都没命中）⇒ 位姿图完全没有全局锚定 ⇒ **方向错**；真因是时间基混用 + 先验给错源 | `issues_and_findings.md` #20/#21 |
| 8 | **角速度限幅**（`wz_max 2.5→1.4` 等三处） | `lio_drift_diagnosis` 说高角速度甩转会飘 | 因果方向相反：3 次分歧起步时指令 `|ω| ≤ 1.15/1.29/1.40 rad/s`，而 2.8~3.0 rad/s 的 recovery spin 出现在 LIO 已经飞到 **4.9~9.2 m/s 之后**；唯一一跑 `y1` 没发散且到点，但**比新表慢 4.5 s、p50 低 0.27 m/s**，且**一次跑分不开"限幅有效"与"这次本来就没磕重"** | `lio_divergence_no_impact.md` §4.2 |
| 9 | 用**标障碍**代替限速 | "把坡面标成障碍规划器就绕开了" | 坡面**本身可行驶**（标它是错的）；实测只把第一击 **86.4 → 68.2 m/s²** | `slope_speed_limiting.md` §5.1 |
| 10 | **只朝前**的走廊 | 车正常是往前开的 | 两次整栈跑里**最重的一击都发生在倒车时**（`v = −1.04/−1.00 m/s`）；第一版峰值 66.5/67.7/86.5 ⇒ 改成**双向窗口**才降到 50.8 | 同上 §5.2 |
| 11 | 用 `/odom` 的 `twist.linear.x` **符号判方向** | 物理上最直接的信号 | 机器人确实在倒车（`/cmd_vel` 与真值都是负的），但 `[slope_speed]` **一直打"前进"** ⇒ 判据没生效、**根因未定位** ⇒ 默认关闭、改用双向窗口 | 同上 §5.2/§7 第 1b 条 |
| 12 | 把 **`velocity_smoother.max_velocity 2.0→1.0`** 当限速替代 | 一行参数就能"开慢点" | 它**不看地形**，平地也慢 —— 而本机制的意义正是"只在低地形前慢" | 同上 §5.6 |
| 13 | bug ③ 的**平移也改成物理真值** | "修就修全" | `/scan` 盘面 z p50 **+0.063 → −0.091 m**；落在高度带之外的波束 **0/948 → 950/950**；局部代价图 lethal/inscribed **461/16937 → 0/0**（robot11 与**默认模型**都整张变空） | `Fidelity` §J.5 |
| 14 | C 方案：`body` 用**抽稀网格**当碰撞 | 比 4 个 box 更贴形 | **ODE 的 ray-vs-trimesh 求交代价是 O(三角面数)**：30000 条射线 × 3000 面 ≈ **9×10⁷ 次/帧 @10 Hz** ⇒ `spawn_entity` 服务 **60 s 超时**、90 s 内 cloud/imu/scan **全 0**。**不是参数问题** | `robot_models.md` §11.2.2 |
| 15 | MeshLab `meshlabserver` 的 QECD | 最"标准"的抽稀工具 | 本机**无 GL**：`MLException: GLEW initialization failed: Missing GL version`（连列过滤器都走不到） | `robot_models.md` §13.3.1 |
| 16 | 用**精确**误差口径（`vtkImplicitPolyDataDistance`） | 精度更高 | 在 dst 建 cell locator，`base_link` 那种 **208 万面**的 dst **10 分钟都出不来**；只在 `l12` 上校准过（精确 p99 0.204 mm vs 采样 0.000 mm ⇒ 采样**偏低 ≤0.22 mm**） | 同上 §13.3.3 注 1/§13.10 第 6 条 |
| 17 | `beluga` 槽位沿用 `laser_min_range: −1.0` | 别的槽位就这么写的 | beluga 对该键声明的是 `[0, DBL_MAX]` ⇒ **启动即崩** | `localization_slots.md`；提交 `c655d77` |
| 18 | cartographer：`use_odometry=false`（"干脆关掉先验"） | 先验同源冗余、晚到会撞 CHECK | 关掉后**近处地面/内部小墙留存曲线**坏、自由格=0（"栅格一点点出来很艰难/全灰"）；且**回环 Count: 0** 说明被调的门槛属于"已经不工作的东西" | `issues_and_findings.md` #20/#26 五次结论 |
| 19 | slam_toolbox `mode: lifelong`（在旧图里定位 + 顺便精修） | 名字就是"终身建图" | ① 没有任何代码读它；② 本机**没有 `lifelong_slam_toolbox_node` 可执行文件**（上游 `CMakeLists.txt` 故意没装）；③ 上游自己标 *highly experimental* ⇒ COD 队的 `mode: lifelong` **实际跑的就是普通 async 建图** | `slam_toolbox_tuning.md` §0 第 2 条 |
| 20 | 把 99.1 MiB 的大文件**排除**在仓库外 | GitHub 单文件硬上限 100 MiB，它只差 0.9 MiB | 实测**入库并 push 成功**（只有 >50 MiB 的 warning）；代价 = 以后每次 clone 都要拖这 104 MB（**没用 LFS**） | `robot_models.md` §9.1/§10.8 第 8 条 |

---

# §5 诚实边界（合并各文档的未验证清单，去重 + 按风险排序）

## §5.1 未验证 / 没做完（**不要当成已知**；按"影响下一步决策的程度"排序）

| # | 未验证项 | 风险 / 影响 | 出处 |
|---|---|---|---|
| 1 | **`robot:=robot11` 没有重跑"修前"的整栈基线**（§L 的 +3.79° / +8.20° 是直接引用；台架复现过、整栈"修后"有） | 严格同期对照缺失；要补：`--chassis articulated` 回退后再跑同一条命令 | `Fidelity` §M.10 1 |
| 2 | **反向 2.0 m 目标没有"修前 vs 修后"的同协议对照** | 只知道"修后"的形状（Δyaw −179.7°、0 条 plan） | §M.10 2 |
| 3 | **`wz` 剂量-响应没做完整扫描**（只有台架单点 `wz=1.0` 与整栈 `wz=1.0`；`wz=−0.75` 只由 RPP 给出） | "角速度通道好不好"缺一条曲线 | §M.10 3 |
| 4 | **单刚体丢掉了 12 个不受控自由度**（`j2…j11`）：将来若要给轮子/云台加真控制器，**必须**先回退到 `--chassis articulated` 再改造 | 影响未来扩展（生成器留了开关） | §M.10 4 |
| 5 | 塌陷后的接触变成"4 个 cylinder 刚体接触"，**平动的摩擦特性也变了**（正前方 0.4077 → 0.3992 m，−2%；反向 1.4370 与 §L 的 1.5753 同量级，差异来自 25 s 腿的时间上限 + 起始点不同） | **没有做系统标定** | §M.10 5 |
| 6 | **`robot11_mount:=sensor` 的"整条链端到端好用"没有任何意义被证明**；**代价图那一层仍然坏**（odom 斜 30° ⇒ `obstacle_layer` 实测 **41.2%** 波束被高度带丢掉） | 该档只能当"帧↔数据自洽"的示范，不能当"可用链" | §J.7 第 3 条、§J.4.2 第 (3) 条 |
| 7 | `sensor` 档**只测了 `lio:=small_point_lio`**；`fastlio`/`pointlio` 走 `lio_tf_adapter` 的杆臂只做了几何/单元级验证；`ground:=patchwork` + `sensor` 在 launch 里**直接报错**（设计选择，不是验证过的组合） | 组合覆盖不全 | §J.8 5/6 |
| 8 | **bug ③ 的"平移"与"代价图高度带重新定基"没实现**（只给了两个候选与实测代价） | 现状态是"歪打正着"（靠 0.2044 m 的巧合补回 odom z） | §J.5/§J.8 10 |
| 9 | **`sensor_height` 的 4.3 cm 疑问**：几何预测 **0.2595 m** vs 由原始云反推 **~0.302 m**（要查的是"轮 collision mesh 最低点 vs 关节 origin 的 FK"）；影响面 = `linefit.sensor_height`（别的任务的槽位文件）与 `p2l` 高度带的解释 | 若真偏 4 cm，地面线整体抬高 4 cm，与 `max_dist_to_line 0.05` 同量级 | §K.1 末尾/§K.8 6 |
| 10 | **`sensor` 档插件 A/B 没跑完**（`k2_sen_legacy` 那一跑 `probe.json` 的 `frames = {}`，而 linefit/限速都正常、TF 也 −30.000° ⇒ **取数时机/订阅侧的问题**，不是该档坏了）；要补：`bash tools/scripts/tiltmount/_batch_k2.sh`（≈15 min） | 该档的插件修复数字**不主张** | §K.8 3 |
| 11 | **全局代价图在静止时仍然偏脏**（0.300 档车那格 0、圆内 123 free，但整张图 unknown 仍占大头、`map` 还在建）；**没做**"跑几分钟之后再看"的对照 | 影响"长跑能不能干净" | §K.8 2 |
| 12 | `sensor` 档 `/scan` 出带比例仍是 **0.396**（2D 链路结构上定不了基；真修要改代价图帧或 LIO 的 odom 重力对齐） | 已知且**不打算在本期修** | §K.8 1 |
| 13 | **没测** `ground:=patchwork` + 近地剔除；**没测** `urdf` 档 + 近地剔除（那一档"半圈波束被高度带丢掉"仍在） | 组合覆盖 | §K.8 7/8 |
| 14 | **RTF / 内存没有做"同时间窗交替"的严格 A/B**（RTF 0.339~0.416 只能读作"同一批里可比"） | 凡是"哪个更慢"的结论都不成立 | §G 3、§K.8 9 |
| 15 | **真足印多边形只做到"知道怎么写"**（nav2 要**字符串**格式；没跑通那一版） | 转弯角扫障碍的折中仍在 | §K.8 4 |
| 16 | **`--goal-forward 2.0` 本身没修**：参数层面**没有可修的**（减小 `inflation_radius`、放大 `xy_goal_tolerance`、改 `regulated_linear_scaling_min_radius` 都**只会**让车更靠近障碍或更早宣布成功） | 要真做长距离验收必须换 spawn/世界，或先修角速度通道（已修） | §L.9 4、§L.0 |
| 17 | 先验图里正前方障碍位置（x≈0.5）与实测物理极限（0.776 m）**差 ~0.28 m 未归因**（候选：口径/尺度差，或两个不同物体）；反向那一侧是吻合的 ⇒ **不是整体平移** | 地图与物理的一致性有未解释残差 | §L.10 2 |
| 18 | **没做**"`slam_nav` 跑 5 分钟后再发同一个目标"这一格（本轮所有目标固定在 launch 后 60 s 发） | 建图成熟度的影响只有 `mode:=nav` 这个"图已建全"的对照 | §L.10 4 |
| 19 | **可达区形状只有两个方向的读数**（前 0.40 / 后 1.44 m），没有做"同一距离、不同方位角"的扫描 | 闸门只能用在这两个方向 | §L.10 6 |
| 20 | **"抽稀省 GUI 内存"在本机没有被证实**（`gzclient` 473 / `gzserver` 3202 / `rviz2` 314~322 MiB 两档都一样，且与 Phase 4 的 1381↔243 MiB **不一致**）；渲染端足迹（~240 B/面）是**推断**；**真 GPU 帧率未知**（只有 llvmpipe 软件 GL） | 用户机器上的实测才是裁决 | §13.4.3/§13.4.4/§13.10 2 |
| 21 | **`base_link` 的精确误差口径跑不出来**（208 万面 dst 上 10 分钟未返回）⇒ 它的 p99/max 只有采样口径的数；`--calibrate` 只在 `l12` 上对过 | 方法学缺口（已登记） | §13.10 5/6 |
| 22 | **抽稀后的视觉在真 GPU 上"看起来够不够"没看过**（没有一张真渲染截图做视觉确认） | "好不好看"未验 | §13.10 7 |
| 23 | `decimated` 与 `full` 两档**没有做端到端逐帧对照**（只在结构上排除了差异来源：A1/G5 断言） | 成本高 | §13.10 9 |
| 24 | **`/scan` 里那圈"自身障碍"没有修**，也没有验证"它是不是 nav 不走的根因" | 候选修法（在 p2l 侧加自击掩膜 / 在插件侧修内移）**未做** | §13.9/§13.10 11 |
| 25 | `model://` 解析：**真 GPU / 真显示器没跑过**（"mesh 加载"是用 `/proc/<gzclient>/io` 的 `rchar` + fd 扫描证的）；沙箱里 gzclient 对"运行期插入的模型"处理不稳定；`robot11_visual:=full` 档的解析链**没有单独复测** | "车在真机屏幕上好不好看"仍未验 | `robot_models.md` §14.6 |
| 26 | 用户机上"`gzclient` 还会不会被 `-9` 杀"**仍未复现**（本机 31.9 GB 两档都活满窗口）；两个等价解析根会让 `InsertModelWidget` 多报 3 行 `Missing model.config` 噪音（**没有去消**） | 需要在同类内存受限环境里跑 `full` 才能证实 | `gazebo_gui_troubleshooting.md` §9 |
| 27 | `hzmirm` 槽位**没有在新代码下重跑**（§11.4 的更正只是"读数据 + 复现 bug"）；`<1 m` 的近场点里"云台自击"与"真障碍"**仍未分离**（只有离线射线求交的归因）；B 方案（抬高雷达）与 `linefit.sensor_height` 的同步是**手工**的（忘记改就会出现"地面线整体偏 h"的静默错误） | 三个槽位对比的公平性 | `robot_models.md` §11.8 3/4/5 |
| 28 | 限速：**"第一击消除"没做到**（110.8 → 50.8，目标 ≲30；最可能原因 = 走廊里坡度/台阶估计仍稀（单帧 7~20 格）、4°/0.04 m 死区为压噪而设）；**承诺前推用"上限"当速度**（实车/打滑/被挡时**低估剩余距离、提前释放**）；**只验证了 `nav:=mppi`**（RPP/DWB/TEB 的 `setSpeedLimit()` 没测）；`ground:=linefit` 槽位只跑了默认槽；**只在"一张先验图 + 一个目标"上各一次跑**；`mode:=mapping` 没跑；真车未验；`/odom` 方向源在 LIO 发散时会跟着错；负障碍（坑/落空）只有部分覆盖 | 这一串直接决定"能不能上真车" | `slope_speed_limiting.md` §7 |
| 29 | 建图/资产：全场台阶边沿覆盖率（35.98%~54.42%）**够不到 80% 判据**，这是 2D 先验的**固有上限**（不是本次提升引入）；累积点云的垂向拖影会让 3D→2D 投影大面积误占；全局图"局部图 raytrace 还没清到"的残留仍在 | 换场地/换图前必读 | `map_assets.md` §3.1、`traversability_plan.md` §9.1 |
| 30 | LIO：唯一复现的失效模式（高角速度甩转）**门槛未定**、机理（"抛硬币"抛在 GICP 位姿门/内点上）只是**判据**不是**修复**；续建污染的 PCD 资产**没修**（只证明不影响实时位姿）；所有数字来自**单个世界 + 单出生点** | 不能外推 | `lio_drift_diagnosis.md` §11 |
| 31 | `tdt-nav-kit`：效率账里"必须在**我们机器上**测"的那几项（§3.6 的清单）**一项都没测**；`docs.nav2.org` 的 `/humble/` 目录 404，凡是契约都以本机 1.1.20 头文件为准 | 研究结论**未落地** | `nav2_vs_custom_planner.md` §0/§7 |
| 32 | 时间戳：`point_lio` 的同一 bug **判定但没能实测**（该槽位在仿真里第 100 帧就被自己的 `pcd_save` 打死）；两个 LIO **子模块的 bug 一行未动**（只报告 + 给补丁） | 上游 fork 里仍存在 | `timestamp_construction_audit.md` 结论先行/§4.4 |
| 33 | GICP：`localization:=small_gicp` 槽**没做行驶 A/B**（两条判据同样生效是结构性结论）；更紧的 `max_correspondence_distance`（1.5→0.8~1.0 m）**未测**；"沿墙方向不可观测"是**地图资产 + 场地几何**的性质，调参救不了 | 不要指望调参 | `gicp_divergence_and_jitter.md` §5.3/§7 |
| 34 | 3D 先验 + GICP 的"定位仍在缓慢收敛"是一条**打折项**（`worlds.md` §5）；`stvl_local_costmap.md` 的四次跑数字本文**未复算**（见 §5.3） | 读表要打折 | `worlds.md` §5、`stvl_local_costmap.md` §3 |

## §5.2 "这些结论是靠什么支持的"（单跑 / 单出生点 / 无头）

| 支持力度 | 结论 |
|---|---|
| **可复现三次以上 / 结构性论证 + 实测** | 车那格 inscribed 团（**513/997 → 435/1001 → 453/1000** 三次独立跑）；`plugin` 档点云/平面拟合**逐位复现**（`(−0.01376, 0.0122, 0.99983)`、`d=0.26038`、1.054°、自击 3294）；`a281636` 的截断修复（丢帧 116→10 单跑，但机理链四步都可单独验证） |
| **单变量 A/B（较强）** | GICP 静止 A/B（**同一二进制**、只切两个开关）；近地剔除同帧回放 A/B（同一帧点云）；插件 `range_from_origin` A/B（同一次编译只差那一行）；限速开/关（同一真源临时改、跑完还原 + diff 核对） |
| **单跑 / 单出生点 / 单目标** | 限速整栈 A/B（各一次跑，撞击量级本身是**概率性**的）；`robot11` 的绝大多数数字（**一个出生点**：RMUL2026 出生点）；`--goal-forward` 系列（各一次）；`sensor` 档三方 A/B（各 1 次静止 + 1 次目标 + 1 次固定动作） |
| **无头 / 软件 GL（不代表真机观感）** | 所有 GUI 数字（Xvfb 3200×1200×24 + llvmpipe/Mesa 23.2.1）；所有"mesh 加载"证据（文件级 `rchar` + fd 扫描）；**没有一次真 RViz 观察** |
| **纯推断（明确登记）** | 渲染端网格常驻足迹 ~240 B/面；用户机 `gzclient` SIGKILL 的内存压力解释；"包侧 export 会让默认模型的 env 多一项"的影响（无头量不出来，只有 GUI 噪音 3→6 行） |
| **概率性结论（不要当确定量）** | "撞击 ⇒ LIO 发散"（11 跑 / 9 配置里 9 次硬撞、3 次仍成功）；"高角速度甩转 ⇒ 飘"（6 次坏 3 次） |

## §5.3 源可核性：这份汇总"读了什么、没读什么"

**逐字/整节读完的源**（本文引用最密）：`tf_interface_contract.md`（全文）、
`slope_speed_limiting.md`（§0–§7）、`tilted_lidar_fidelity.md`（§A/§B/§C/§D.3–§D.4/§G/§H/§I/§J/§K/§L/§M 的关键节与全部表格）、
`robot_models.md`（§1.3/§3.2、§9–§15）、`gazebo_gui_troubleshooting.md`（§0/§1 + `robot_models.md` §14 的转述）、
`slam_toolbox_scan_drops.md`、`mapping_2d_from_cloud.md`、`slam_toolbox_tuning.md`、`slam_drops_loopclosure_odom.md`、
`map_assets.md`、`continue_mapping.md`、`lio_divergence_no_impact.md`、`gicp_divergence_and_jitter.md`、
`gicp_initialpose_latency.md`、`lio_drift_diagnosis.md`、`timestamp_construction_audit.md`、`path_clearance_and_contact.md`、
`ground_segmentation_slots.md`、`traversability_plan.md`、`nav2_vs_custom_planner.md`、`params_ownership_checklist.md`、
`sim_real_contract.md`、`worlds.md`、`README.md` 的**结论节 + 关键表**（各文档的 §0/速查 + 上表列到的具体表）。

**只读到"结论/摘要"层面、本文据此转述的源**（要逐行细节请回原文档，本文**没有**独立复算它们的数字）：
`stvl_local_costmap.md`（只读 §0 摘要 + 标题）、`mapping_small_point_lio.md`（只读 README 摘要 + 提交信息）、
`lio_slots.md` / `localization_slots.md`（只读 README 摘要 + 提交信息）、`algorithm_matrix.md`（只读 README 摘要）、
`smoke_test_runbook.md`（只读 README 摘要）、`issues_and_findings.md`（只读前 40 行 + #15/#18–#26 的摘录）、
`architecture.md`（只读标题结构与 §3.2.x 目录）、`glossary.md`（未读，术语表见本文 §8）、
`debug_fastlio_cartographer.md` / `cartographer_2d_occupancy_semantics.md`（只通过 `issues_and_findings.md` 与 README 间接引用）。

**明确不在本工作期范围、本文未读**：`cod_nav_*.md`（8 份，2026-10-04 以前的移植规划）、
`rm_algolab_plan.md` / `rm_algorithm_overview.md` / `rm_bench_refactor_plan.md` / `rm_algorithm_catalog.md`、
`scan_context_*.md`（3 份）、`mapping_relocalization_survey.md`、`nav_strategy_taxonomy.md`、
`slam_nav_goal_strategies.md`、`research_p2l_scan_stall.md`、`kinodynamic_esdf_mpc_notes.md`、
`build_optimization.md`、`3d_to_2d_survey.md`、`mapping/` 与 `package/` 两个子目录。
（它们的存在与定位见 `docs/README.md`；本文只在"P0 阶段"里提了它们对应的提交。）

## §5.4 文档之间的矛盾，以及本文采信了哪一版（**逐条给理由**）

| # | 矛盾 | 采信 | 理由 |
|---|---|---|---|
| 1 | `tilted_lidar_fidelity.md` 把 §I/§J 标为 2026-10-09、§K/§L/§M 标为 2026-10-10；但对应提交在 `git log` 里是 **2026-10-07 / 10-08** | **git 日期**（本文时间线），引用时保留原 `§` 编号 | 提交是"改了什么/什么时候"的唯一真值；文档内的"会话日"标签不稳定 |
| 2 | §K.5 表记"`detected collision ahead` = **0 条**"；§L.1.3 记长目标是 **135 条** | **§L**（0 条只对**短目标**成立） | §L 是同协议复跑 + 明确写了"口径更正" |
| 3 | §J.5 记 `sensor` 档局部代价图 **0 / 0（整张全 free）**；§K.1.3 复跑得 **495 / 16511** | **§K.1.3**（`0/0` 是"**平移也改成物理真值**"那一版的后果） | §K 明确把口径钉死，并给了独立复跑的数字 |
| 4 | §I.9 的单测把"正确合成"写成 `T_ol·T_bl⁻¹`；§J.1.2 用链式法则推出 `T_ol·T_bl` | **§J**（并保留 §I.8(f) 那句"会给出 rpy (30°,0,yaw)"是对的） | §J 是后一轮的**记法更正**，且给了单测断言 |
| 5 | `robot_models.md` §9.2 生成物文件头写整车面数 2,870,498；逐 mesh 求和是 **2,821,320** | **2,821,320** | 12 个 STL 逐项求和（提交 `15175ea` 就是修这个笔误） |
| 6 | §13.9 的 `/scan` 分带百分比分母用 693；实际有限波束 **833 条** | **833** | 提交 `e060a98` 的更正 |
| 7 | Phase 4 §12.2 报 rviz2 **1381 ↔ 243 MiB**；Phase 5 §13.4.3 同脚本重跑是 **314 / 322 MiB** | **两版都保留**，以 Phase 5 为"测量记录"，并把不一致登记为**未定论** | 两者是同脚本同环境的两次测量 ⇒ 属"测量冲突"，不能单方面宣布哪一个对（三条可能原因已列） |
| 8 | Phase 3 §11.2 说近场点"100% 落在云台 `l10/l11`，中位 0.14 m"；Phase 4 §12.3.1 复核为中位 **0.021 m**、AABB 只命中 3 点 | **Phase 4** | 更晚、更细的逐点复核，并给出了独立证据（地面 z 随距离单调变化） |
| 9 | `lio_drift_diagnosis.md` §2.3 的"巨大死区"表（要 1.9 rad/s 得给 6）；`lio_divergence_no_impact.md` §2 同批配对重测 **≈1:1** | **≈1:1**（旧表作废） | 旧表自认"大概率混进了闭环里 cmd 与被控量不同口径的问题"，且新测量给了 n 与符号一致率 |
| 10 | `hzmirm` 的 `/segmentation/ground = 0` 归因：§3.3 只讲几何；§11.4 追加"C++ bug" | **几何 + bug 并列** | 两条证据都成立（几何 6.32 m 盲环 + 该键一开 = 0 点） |
| 11 | `robot11_mount` 的取值：§15 说"只能是 `plugin\|urdf`"；§J 新增 `sensor` | **三档** | §J 是后一轮施工，且给了"选错直接报错"的语义 |
| 12 | §11 说"视觉 = 原始 mesh、视觉零损失"；Phase 5 把默认 `decimated` 指向"专用视觉 LOD" | **Phase 5**（"零损失"只在 `robot11_visual:=full` 时成立） | Phase 4 引入档位、Phase 5 改变了 `decimated` 的指向，文档自己也标了"这句话现在只在 full 档成立" |
| 13 | §C.4/§F 与 §K.5 对 `--goal-forward 0.5` 的读数（0.313/0.170 m）与 §L 的 0.2977/0.1902 m | **§L**（同协议复跑） | §L.1.3 明确写"复现，差异来自版本/时间" |
| 14 | "回退档逐字节相同"（多处）vs `d3034aa` 的精确表述 | **"模型体（去注释）逐字节相同"** | 后者是逐字节比对后的精确说法 |
| 15 | `path_clearance_and_contact.md` §3.2 说"没有任何单变量能可靠救这条链"，而 §4.1 仍然应用了 `w_smooth 0.4→0.0` | **两条都保留**：`w_smooth` 只治"贴边"（有计划/执行余量的量化收益），**不治链**（`v_mul12` 是反例） | 文档自己写明了"只治贴边不治链" |
| 16 | README 索引把 `robot_models.md` 的状态写成"（2026-10-07 新增）"，与其 §14/§15 的 2026-10-08 内容 | 以**文档正文**为准（README 只是索引） | 索引是"定位"不是"事实源" |

---

# §6 命令与回退速查

## §6.1 现在能用的启动命令（复制即可）

```bash
# ① 默认模型（本期没有改它的任何默认路径）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav \
    lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0

# ② robot11 的"今天的行为"（帧正、点云正、射线真的斜）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0

# ③ robot11 + 物理斜装 + 账也对（点云表达在真·传感器系；下游做重力对齐）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0

# ④ robot11 + 看"倾角记在关节上"的诊断档（RViz 里会看到 30° 斜坡 —— 那就是"帧与数据不一致"）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 robot11_mount:=urdf spin_speed:=0.0

# ⑤ 带 GUI（不需要任何环境变量前缀；model:// 解析已自动化）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=nav \
    lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d \
    spin_speed:=0.0 robot:=robot11 nav_rviz:=True

# ⑥ 无头 / 不再等在线模型库（两个都是 opt-in，默认 = 改造前行为）
… gui:=False              # 只起 gzserver，RViz 照旧
… gui:=False gazebo_offline:=True

# ⑦ 建图 / 续建（同名存档 = 续建；换场地续建默认拒绝）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
    lio:=small_point_lio mapper:=slam_toolbox map_name:=RMUC2026_home nav_rviz:=True spin_speed:=0.0

# ⑧ 只起 Gazebo + 模型（不含感知/LIO；注意：这条入口**不传** robot11_mount ⇒ 只能跑 plugin 档）
ros2 launch hzmi_rm_simulation rm_simulation.launch.py robot:=robot11 world:=RMUC2026

# ⑨ 换地面分割器 / 换定位槽 / 换局部障碍层（槽位，默认不变）
… ground:=patchwork      … localization:=small_gicp      … local_obstacle:=stvl
```

**构建（install/ 是逐文件符号链接 ⇒ 新文件必须 build 才可见）**：

```bash
# 改过插件/判据层/两个分割节点/launch/参数（最全的一条）
colcon build --symlink-install --packages-select ros2_livox_simulation rm_ground_traversability \
    linefit_ground_segmentation_ros patchwork_ground_segmentation rm_nav_bringup small_point_lio \
    pointcloud_to_laserscan robot11
```

## §6.2 验收命令（改完必须跑的那几条）

```bash
# ① robot11 槽位的 30 秒自检（跑起来之后，另开一个终端）
ros2 param get /ground_segmentation sensor_height              # 期望 0.2595（不是 0.226）
ros2 param get /ground_segmentation gravity_aligned_frame      # 期望 空串（sensor 档期望 base_link）
ros2 param get /ground_segmentation obstacle_near_ground_m     # 期望 0.05（robot11）/ 0.0（默认）
ros2 param get /ground_segmentation self_mask_enable           # 期望 True（robot11）/ False（默认）
ros2 param get /local_costmap/local_costmap robot_radius       # 期望 0.3（robot11）/ 0.22（默认）
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height   # 期望 0.0
ros2 param get /pointcloud_to_laserscan target_frame           # plugin 档期望 空串；sensor 档期望 base_link
ros2 run tf2_ros tf2_echo base_link livox_frame                # plugin: rpy 0；urdf/sensor: −30 0 0
ros2 topic echo /segmentation/obstacle --field width -n 1      # 期望 ~5500-5800（不是 ~6100-6900）
ros2 topic echo /ground_segmentation/traversability_stats -n 1 | grep -o '"near_ground":[0-9]*'

# ② P0 平地冒烟回归（改任何东西之后都要跑）
bash tools/scripts/localization/run_nav_smoke_regress.sh --tag regress_x --domain 151 --port 11821 \
     --localization gicp --nav mppi --planner smac2d --goal -1.0 2.0

# ③ 一条命令量齐 robot11（隔离无头 + 参数/TF 回读 + 契约 + 代价图圆 + 短目标）
tools/scripts/tiltmount/run_tilt_mount_probe.sh my_goal --variant plugin --settle 30 \
    --duration 30 --frames 3 --goal-forward 0.5 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False

# ④ 三层坐标系自检（离线，秒级）
python3 tools/scripts/tiltmount/tilt_chain_check.py --dir sensor:.tmp_tiltmount/j2_sensor
#   期望："地面法向 vs 自己 frame_id z" ≈ 30°；"经 TF 转到 base_link 后" ≈ 1°；自击掩膜命中 ≈ 29.2%

# ⑤ 只量近地剔除（同一帧回放，秒级，不用 Gazebo）
bash tools/scripts/tiltmount/_replay_ng.sh <一帧 raw_*.csv> 0.2595

# ⑥ 视觉档位 / 生成物 / 限速真源 的断言（都不用 Gazebo）
python3 tools/scripts/regress/check_robot11_visual_slot.py          # 21 项断言，期望全绿
python3 tools/scripts/regress/check_slope_speed_table.py            # 真源 + 表↔模型 + 与 nav2 对账
python3 tools/scripts/regress/check_traversability_criteria.py
python3 tools/scripts/regress/robot11_make_sim_xacro.py --out /tmp/x.xacro && diff /tmp/x.xacro \
    src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro          # 期望 diff 为空（生成物 = 生成器输出）

# ⑦ 可达性前置判据 + 目标闸门（新验收口径）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m10_r11_reach25 --settle 25 --duration 6 \
    --goal-wait 0 --preflight-reach 0.25 --preflight-time 25 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_nav_goal_forensics.sh m8_r11_rev12 --settle 30 --duration 30 \
    --goal-forward -1.2 --goal-wait 60 --reach-gate .tmp_tiltmount/m10_r11_reach25/forensics.json -- \
    world:=RMUL2026 mode:=nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False

# ⑧ 底盘角速度台架（变体矩阵，一次 <40 s）
#    先备好模型 SDF：xacro <模型>.xacro <args> | gz sdf -p /dev/stdin > /tmp/<模型>.sdf
tools/scripts/tiltmount/run_chassis_yaw_bench.sh t_r11_weld --model /tmp/robot11_welded.sdf \
    --variant baseline --wz 1.0 --push-wall 12 --settle 8 --add-jsp
```

## §6.3 回退速查表（每个改动**一键怎么退**）

| 想退掉什么 | 怎么做（一键） | 影响面 |
|---|---|---|
| 整套 `robot:=robot11` | 去掉 `robot:=robot11`（**默认路径从未改动**） | 零风险 |
| `robot11_mount` 任意档 | 去掉该参数（默认 `plugin`）；`sensor` 档还可只删 xacro 里的 `<cloud_frame>` | 只影响 robot11 |
| 单刚体底盘（模型行为） | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated` | 生成物**模型体去注释后逐字节等于** 2026-10-09 的 HEAD（两份 35025 B） |
| 视觉抽稀（回上游原始 STL） | `robot:=robot11 robot11_visual:=full` | 回到 Phase 1~3 的逐字节行为 |
| 视觉档位的默认值 | 生成器 `--visual-inventory` 指向旧清单重跑 | 默认 `decimated` 依赖 `meshes/decimated/` |
| 自击掩膜 | `config/traversability_self_mask_robot11.yaml` 的 `self_mask_enable: false`（或撤 launch 里 `self_mask_params`） | 回到"静止被压到地板 0.60" |
| 近地剔除 | `config/traversability_near_ground_robot11.yaml` 的 `obstacle_near_ground_m: 0.0`（或撤 `near_ground_params`） | 回到"车那格 42、圆内 156 个 ≥99" |
| 半径/膨胀（0.300/0.60/0.65 → 0.3565/0.70/0.75） | 改 `nav2_params_sim_robot11_costmap.yaml` 的两个键（或 `git revert 1c89473` 的该文件） | 只影响 robot11 槽位 |
| 插件 0.1 m 起点修复 | 在对应模型 xacro 的 `<plugin>` 里加 `<range_from_origin>false</range_from_origin>` | **逐字节**回旧行为（所有模型） |
| 两个 LIO bug 修复（②③） | `git revert 2a5a8b6`（默认档**逐位不变**；revert 后与 §I 那一轮等价） | 影响 `urdf`/`sensor` 档的账 |
| 时间戳四舍五入 | `git revert a281636` | 丢帧日志（116 条 / 4.3%）会回来 |
| linefit 的 `Identity()` 修复 | 改回 `Eigen::Affine3d tf;`（或 `git revert 0142bd9`） | `gravity_aligned_frame` 一开又变 0 点 |
| p2l 高度带 1.0 | 改回 `max_height: 0.1` | >0.326 m 的几何又从 `/scan` 消失 |
| 前瞻限速（只关限速） | `speed_limit_enable: false`（**构造期读参数**：`ros2 param set` 运行时不生效，要重启节点或改文件） | 速度回到 nav2 自己的 `vx_max`/smoother |
| 前瞻限速（连判据一起关） | `traversability_enable: false` | 回到"没有坡度/台阶判据"的旧链路 |
| 限速提速档（peak 45 / floor 0.60 / 死区 5.5°+0.06 m） | 六个键改回 `30 / 0.45 / 4.0 / 0.040 / 旧两张表`（或 `git revert 4adf0d8`） | 回到"0/4 到点" |
| MPPI 里残留的速度上限 | `ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit "{percentage: false, speed_limit: 0.0}"` | 节点被 SIGKILL 时的兜底 |
| smac2d 的 `w_smooth 0.4→0.0` | 改回 `0.4`（或 `git revert 4a7996c`） | 计划路径又会贴内切带 |
| GICP 三道判据 + 低增益滤波 | `gate_enable:=false` + `smoothing_enable:=false`（= 改动前行为） | 回到"每帧采纳 + 用测量当初值" |
| slam_toolbox 节点加密 | 三个键改回 `0.5/0.5/0.5`（或 `git revert 5cb93cb`） | 回环又基本不触发（环秩 1） |
| 地图默认对（2D/3D 先验） | 用 `*.bak-20261007` 就地覆盖回 `map/RMUC2026.pgm\|yaml` 与 `PCD/RMUC2026.pcd` | 见 `map_assets.md` §4.4 的逐条命令 |
| 会话存档的某一代 | `map_archive.sh restore`（走 `.prev-<时间戳>` 那条链） | ⚠️ 不要手工搬 `.prev-*` |
| `save` 的严格会话名解析 | `git revert 02306fb`（**不推荐**：会恢复"信任 `.session.yaml`"的事故面） | 存档安全 |
| `model://` 解析自动化（启动侧） | 删 launch 里 `ld.add_action(OpaqueFunction(function=_gazebo_model_path_setup))` | 回"必须手工前缀环境变量" |
| `model://` 解析自动化（包侧） | 删 `package.xml` 里 `<gazebo_ros gazebo_model_path="${prefix}/.."/>` 并重建 robot11 | ⚠️ 裸 `gzserver`/`gz sim` 入口又会回落在线模型库 |
| `gui` / `gazebo_offline` 开关 | 不传即可（默认 = 改造前行为） | 零风险 |
| 地面分割槽位 `ground` | `ground:=linefit`（默认） | 等于没改 |
| 局部 STVL | `local_obstacle` 槽位改回默认 | 零风险 |
| 本期的**工具**（`tools/scripts/tiltmount/` 等） | `rm -rf tools/scripts/tiltmount/`（**不进任何 launch/节点**） | 只会让 §6.2 的命令失效 |

---

# §7 提交索引（本工作期全部相关提交）

> 口径：`git log` 按**作者日期**取 `2026-10-05` 及以后（含 10-08），共 **161** 个提交，按时间倒序。
> "层"一列是由提交标题的关键字机械映射出来的**粗分类**（便于检索，不是严格归属）。
> ⚠️ 若用 `git log --since=2026-10-05 --until=2026-10-09`，你只会看到 **128** 个 ——
> `--since/--until` 按提交日期过滤，而本仓这一批提交的作者/提交日期虽然相同（核过 0 处不一致），
> 边界时刻的时区解释会让窗口比"作者日期 ≥ 2026-10-05"少一批。**以本文这张表为准。**

| hash | 日期 | 一句话（提交原文） | 层 |
|---|---|---|---|
| `d3034aa` | 2026-10-08 | docs(robot11): 把"回退档逐字节相同"说准 —— 是**模型体（去注释）**逐字节相同 | 模型/斜装 |
| `d7c4da0` | 2026-10-08 | docs(fidelity): §M —— robot11 角速度通道的归因与修复（逐候选判决 + 前后数字 + 新验收口径） | 模型/斜装 |
| `9de6b72` | 2026-10-08 | feat(tools): 可达性前置判据（--preflight-reach / --reach-gate）+ 原地转目标 + 角速度台架 | 工具 |
| `ef927e2` | 2026-10-08 | fix(robot11): 底盘塌成单刚体 —— planar_move 的 Set*Vel 只对单刚体自洽，角速度通道 1.03%→16.2% | 模型/斜装 |
| `de3b78c` | 2026-10-08 | docs(fidelity): §L —— `robot:=robot11` 的 `--goal-forward 2.0` 走不到：归因（实体障碍 + 角速度通道 0.55%），§K.8 第 5 项收口 | 模型/斜装 |
| `339e205` | 2026-10-08 | fix(tools): tilt_mount_probe 的 gt_yaw_delta_deg 把"度"又 degrees() 了一次（多乘 57.2958） | 模型/斜装 |
| `94f0d6b` | 2026-10-08 | feat(tools): §L 取证链 —— nav_goal_forensics 探针（plan/代价图/action 状态机/BT/速度 + 绕过 nav2 的 --push） | 导航/限速 |
| `6f6a014` | 2026-10-08 | chore(tools): _batch_k2.sh 每跑后立即校验 frames 非空并把 probe.json 另存 <tag>.done.json（批次中断也知道哪几跑是好的） | 工具 |
| `4964361` | 2026-10-08 | docs(fidelity): §K.8 补记 sensor 档插件 A/B 的数据缺口现象（probe frames={} 但 linefit/限速正常） | 模型/斜装 |
| `8e73c8b` | 2026-10-08 | docs(fidelity): §K.4.2 补上 robot11 的插件偏移 A/B（旧 vs 新：地面峰 -0.280 到 -0.155、圆内 >=99 13 到 0） | 模型/斜装 |
| `18f2738` | 2026-10-08 | docs(fidelity): §K 代价图侧收尾（高度带定基 / 近地剔除 / 半径与足印 / 插件 0.1 m 偏移）+ 取证工具 | 模型/斜装 |
| `1403f42` | 2026-10-08 | fix(livox_plugin): 点 = range*axis 漏了射线起点（minDist*axis + offset.Pos()），默认修好 + 回退开关 | 仿真/插件 |
| `1c89473` | 2026-10-08 | fix(nav2): robot11 槽位代价图几何 外接0.3565/0.70 改 内切0.300/0.60（+ 高度带"为什么不动"的注释） | 模型/斜装 |
| `360d493` | 2026-10-08 | feat(perception): 近地剔除（obstacle_near_ground_m，按槽位）—— 代价图 2D 链路唯一能用上高度的那一级 | 感知/代价图 |
| `d4965bd` | 2026-10-08 | docs(nav2): 纯 C++ 规划库接入 nav2 的三形态决策/契约/效率账 + CMake 骨架（纯新增） | 导航/限速 |
| `7542973` | 2026-10-08 | docs(fidelity): §J 两个 LIO bug 的修复 + robot11_mount:=sensor 的 A/B 实测（纯追加） | 模型/斜装 |
| `8375963` | 2026-10-08 | feat(tools): tiltmount 探针扩展（--drive/--goal/RTF/代价图圆/分带/契约）+ 两个新离线工具 | 模型/斜装 |
| `1116b05` | 2026-10-08 | feat(sim): 插件新增 <cloud_frame> 开关 + robot11_mount:=sensor（"物理斜装 + 账也对"，纯增量） | 模型/斜装 |
| `2a5a8b6` | 2026-10-08 | fix(small_point_lio): 修 bug②「点云多乘一次 T(base←livox)」+ bug③「odom→base 姿态用共轭」 | 仿真/插件 |
| `527fc10` | 2026-10-07 | feat(tools): 新增 tools/scripts/tiltmount/ —— 斜装雷达「帧/射线/外参/外方位」取证工具（纯增量） | 模型/斜装 |
| `a7497f0` | 2026-10-07 | docs(fidelity): §I 斜装雷达物理对不对/帧与外参账逐项判决（H1/H2/H3）+ RViz 配方 + 精确改法（中文） | 模型/斜装 |
| `c7f57d8` | 2026-10-07 | docs(gazebo): §5.1.6 无头二次独立复核 + §14/§15 两处更正与交叉链接 | 仿真/插件 |
| `6c9ae96` | 2026-10-07 | docs(fidelity): 新增《雷达"斜 30°"到底斜在哪儿》逐帧点账本 + 代价图归因 + mount 开关（中文） | 模型/斜装 |
| `e2bcf3d` | 2026-10-07 | feat(gazebo): robot:=robot11 的 model:// 解析自动化（launch 侧 + 包 export）+ 无头复核工具 | 模型/斜装 |
| `b7d625b` | 2026-10-07 | feat(robot11): 新增 robot11_mount:=plugin|urdf 安装方式开关（默认不变）+ 取证工具 | 模型/斜装 |
| `c12cdb8` | 2026-10-07 | docs(gazebo): 更正 §2.2 标题里的时间戳格式（是 `(秒 纳秒)`，不是微秒） | 仿真/插件 |
| `3737c23` | 2026-10-07 | docs(robot_models): §12.7 未验证项 ① 追加交叉链接（指向 gazebo GUI 另案结清） | 模型/斜装 |
| `96974db` | 2026-10-07 | docs(gazebo): 新增《gzclient 加载不出来》取证/机理/实测/修法（中文） | 仿真/插件 |
| `3b48c7c` | 2026-10-07 | feat(launch): 新增 `gui:=True|False` 与 `gazebo_offline:=True|False` 两个开关（纯增量） | 仿真/插件 |
| `e060a98` | 2026-10-07 | docs(robot11): §13.9 更正 `/scan` 分带百分比的分母（用"有限波束 833 条"，不是 693） | 模型/斜装 |
| `15175ea` | 2026-10-07 | fix(robot11): 生成物文件头里的整车面数笔误 2,870,498 → **2,821,320**（= 12 个 STL 求和，§9.2） | 模型/斜装 |
| `bf0353b` | 2026-10-07 | docs(robot11): §13 Phase 5 —— 上网查到的标准做法（带 URL）+ 抽稀流水线 + 三档代价实测 + 开关 + 验收 + 未验证 | 模型/斜装 |
| `4ec728e` | 2026-10-07 | feat(robot11): `decimated` 档改指**专用视觉 LOD** + 开关契约检查工具 + 三处不再骗人的横幅（Phase 5） | 模型/斜装 |
| `646277f` | 2026-10-07 | feat(robot11): `<visual>` 的 QEM 视觉 LOD —— 抽稀工具 + 12 个抽稀件 + 量化验证清单（Phase 5） | 模型/斜装 |
| `d2ac1cf` | 2026-10-07 | docs(robot11): §12 Phase 4 —— GUI 代价表（full vs decimated，实测）+ 自击掩膜前后数字 + 接线运行期证据 + 诚实更正 + 未验证清单 | 模型/斜装 |
| `cf60b84` | 2026-10-07 | fix(bringup): robot11 —— 更正骗人的横幅、补运行期接线证据、视觉档位开关（默认抽稀）、GUI 代价实测工具 | 模型/斜装 |
| `1ca09eb` | 2026-10-07 | feat(perception): 自击掩膜 —— 把近场自身回波剔出「坡度/台阶/限速」判据（默认关，robot11 槽位开） | 模型/斜装 |
| `23c1dc8` | 2026-10-07 | docs(robot11): §11.2.1 补一条共享代码回归证据（默认模型在插件+linefit 修复后与 Phase 2 数字一致） | 模型/斜装 |
| `a1d9c15` | 2026-10-07 | docs(robot11): §11.9 补 nav2 覆盖层的**运行时**证据（robot_radius 0.3565 生效 + controller/planner active + 目标被接受，但车未走 ⇒ 列入未验证） | 模型/斜装 |
| `d4e8232` | 2026-10-07 | feat(robot11): Phase 3 —— 让雷达真的看得见（A 方案：碰撞挖视锥）+ 斜置感知链槽位化 + nav2 几何覆盖 | 模型/斜装 |
| `82e4946` | 2026-10-07 | feat(sim): livox 射线插件支持"安装倾角"<tilt_rpy>（缺省单位阵 ⇒ 其它模型逐字节不变） | 模型/斜装 |
| `0142bd9` | 2026-10-07 | fix(perception): linefit 的 gravity_aligned_frame 会把点云塌到原点（Eigen::Affine3d 未初始化） | 感知/代价图 |
| `599fc09` | 2026-10-07 | feat(robot11): `robot:=robot11` opt-in 槽位（生成式 xacro）+ 雷达 30° 的实测判定 + 实测评估 | 模型/斜装 |
| `f361f00` | 2026-10-07 | feat(robot11): 用户哨兵 robot11 的 mesh 资产包 + 逐 mesh 量测 + base_link 碰撞策略（实测） | 模型/斜装 |
| `712741f` | 2026-10-07 | docs(robot): docs/robot_models.md —— 模型槽位 + 用户哨兵 URDF 的实测评估（+ 索引与 3 处交叉引用） | 模型/斜装 |
| `a2c6349` | 2026-10-07 | feat(tools): 机器人模型槽位的实测/复算/断言工具（无头跑·只读探针·离线 linefit·provenance） | 工具 |
| `d3940b5` | 2026-10-07 | feat(bringup): robot:=<模型> opt-in 槽位（默认 ''= 现行模型，逐字节行为不变）+ 按槽位切标定 | 模型/斜装 |
| `d6232ea` | 2026-10-07 | feat(robot): robot:=hzmirm 的机器人模型 —— 用户哨兵 URDF 逐字接入 + 我们补的 inertial/IMU/雷达/底盘 | 模型/斜装 |
| `95846c3` | 2026-10-07 | docs(map_assets): 地图/先验资产规范 + 逐个清单 + 实测对照 + 换图/回滚流程（+ 索引与 5 处文档同步） | 建图/资产 |
| `c8d65f9` | 2026-10-07 | chore(map): 退役的合成资产搬进 map/attic 与 PCD/attic（只搬不删）+ 两处 README | 建图/资产 |
| `2192024` | 2026-10-07 | feat(tools): map_prior_ab.py —— 两张 2D 先验候选的公共窗口/逐格/边沿对照（离线，无 ROS） | 建图/资产 |
| `7f9e354` | 2026-10-07 | feat(map): RMUC2026 默认先验对按实测提升到 v3 会话产物（旧份留 *.bak-20261007） | 建图/资产 |
| `6ca978e` | 2026-10-07 | docs(lio): 「没撞击却飘里程计」归因报告 + 限速/角速度 A/B + §2.3 更正 | 定位/LIO |
| `4adf0d8` | 2026-10-07 | feat(bringup,perception): 前瞻限速提速档（peak 45 / floor 0.60 / 死区 5.5°+0.06 m） | 感知/代价图 |
| `dad39c5` | 2026-10-07 | feat(tools,diag): 「没撞击却飘里程计」专用仪器 —— 只读时间轴 + 归因分析器 + 无头跑法 | 工具 |
| `72155e4` | 2026-10-07 | docs(slope_speed): §5 补一条没走通的路 —— 用 /odom 速度符号判方向没触发（根因未定位，默认改双向窗口） | 感知/代价图 |
| `fc226bb` | 2026-10-07 | docs(slope_speed): 前瞻坡度/台阶限速报告 + traversability_plan §9 状态更新 | 感知/代价图 |
| `71f6c87` | 2026-10-07 | feat(tools,regress): 限速真源/物理自检 + A/B 指标提取 + 离线合成扫掠台架 | 工具 |
| `bb0fe03` | 2026-10-07 | feat(bringup): 可通行性真源新增 speed_limit_* 一组键（前瞻限速的表与物理锚点） | 其它 |
| `ca89b69` | 2026-10-07 | feat(perception): 前瞻坡度/台阶限速 —— 连续量 → 速度上限（发 nav2 SpeedLimit，命令链零新增节点） | 感知/代价图 |
| `f78a685` | 2026-10-07 | docs(slots): §12 补一条更稳的重跑路线 —— 直接存跑图那次的 slam_toolbox /map（对点云垂向拖影免疫，本次实测 10.1 m² 占用 / 可行驶斜面误占 1.82%） | 建图/资产 |
| `819160c` | 2026-10-07 | docs: 缺陷①②③的修复报告 —— 实测前后、整栈 nav 复跑、重跑建图 recipe、回退 | 文档 |
| `97d0dbc` | 2026-10-07 | feat(tools,regress): 低矮地形实测/验收装置（同帧 A/B + 建图捕获 + 重跑建图 + 图侧验收） | 工具 |
| `cb3a756` | 2026-10-07 | fix(perception): p2l 高度带 max_height 0.1→1.0（离地 0.326→1.226 m）+ 离线判据改走真源 YAML | 感知/代价图 |
| `3ea5dd4` | 2026-10-07 | feat(perception): 实时「坡度/台阶」判据（rm_ground_traversability）+ 两个地面分割槽位接入 | 感知/代价图 |
| `4da000e` | 2026-10-06 | docs(clearance): §2.2 补一条独立佐证（map_opt p05 −0.61 m：雷达看见了图上更近的几何） | 感知/代价图 |
| `2e7cf61` | 2026-10-06 | docs(clearance): 跑次口径统一为 11 次跑 / 9 个配置（base 与 v_ginfl 各跑两次） | 感知/代价图 |
| `acb93c1` | 2026-10-06 | docs(clearance): §3 表两处数字校正（base 健康段 IMU 80.8/5、v_rpp 28.2/0）+ 按撞击量级重写读法 | 感知/代价图 |
| `4a7996c` | 2026-10-06 | fix(nav2): smac2d 平滑器 w_smooth 0.4→0.0（不再把 A* 的余量拉回内切带） | 导航/限速 |
| `6f0c27a` | 2026-10-06 | docs(clearance): 贴膨胀边/撞击/定位失效的实测归因报告（12 次跑 + 场地网格对账） | 感知/代价图 |
| `1811149` | 2026-10-06 | feat(tools,regress): 余量/接触/定位 三合一实测装置（贴膨胀边与撞击的量化基线） | 工具 |
| `25e33ad` | 2026-10-06 | docs(gicp): 发散与抖动复现/机制/修复/A-B 报告（+ 槽位与算法矩阵指针） | 定位/LIO |
| `642c174` | 2026-10-06 | feat(tools,gicp_bench): 发散/抖动 A/B 装置 + 场地自相似性探针 | 定位/LIO |
| `4f29d35` | 2026-10-06 | fix(gicp_registration): 接受判据三道 + 环路内低增益滤波 —— 治 map→odom 自传播发散与"到点后四处抖动" | 定位/LIO |
| `42370d7` | 2026-10-06 | docs(continue_mapping): §11 事故复盘②「save 把位姿图写到别人那套会话的名字上」 | 建图/资产 |
| `02306fb` | 2026-10-06 | fix(mapping): save 严格解析会话名（活栈优先 / 多会话拒绝 / --check），不再信任 .session.yaml | 建图/资产 |
| `9d180ff` | 2026-10-06 | feat(bringup): 会话播报器 —— 把「本次 launch 是谁」挂到 ROS 图上（存档事故修复之一） | 装配/参数 |
| `c7260da` | 2026-10-06 | docs(slam-toolbox): 丢帧 × 回环 × 里程计两假设的实测裁决（A 不成立 / B 买的是 TF 时延不是精度） | 建图/资产 |
| `d0fe0a0` | 2026-10-06 | feat(diag): 丢帧 × 回环 × 里程计 A/B 装置（只读仪器 + 真值里程计桥 + 归因器） | 工具 |
| `7d03f16` | 2026-10-06 | docs(diag): LIO 漂移分段诊断报告 —— 四个候选（正常累积/卡死退化/转弯/续建污染）逐条对账 | 工具 |
| `e866cfa` | 2026-10-06 | feat(diag): LIO 漂移分段诊断工具 —— 分段脚本化路线 + 50Hz 同步时间轴 + 逐段 ATE | 工具 |
| `3a2b54d` | 2026-10-06 | docs(slam-toolbox): §5 补三条实测反例（只放松 chain / 放宽 chain+半径+阈值 / 指望纠 LIO 大跳变），§7 修正路线复盘 | 建图/资产 |
| `5cb93cb` | 2026-10-06 | fix(slam-toolbox): 建图节点加密（min_travel 0.5→0.2 等），让回环真的能触发 | 建图/资产 |
| `d3dde82` | 2026-10-06 | docs(slam-toolbox): 建图漂移/回环调参考证 + 节点密度 A/B + sltune 实验工具 | 建图/资产 |
| `db457e1` | 2026-10-06 | docs(mapping): continue_mapping 补 数据卫生/覆盖前备份/续建一致性检查 + 事故复盘（含 2026-10-06 实测） | 建图/资产 |
| `133742d` | 2026-10-06 | feat(mapping): 续建后一次性一致性检查 map_resume_check（对不上就喊，strict 直接收栈） | 建图/资产 |
| `59eb110` | 2026-10-06 | feat(perception): cloud_accumulator 数据卫生（高度带/跳变速度闸/TF退化帧跳过）+ 写前体检与备份 | 感知/代价图 |
| `15d5c57` | 2026-10-06 | feat(mapping): 存档覆盖前自动备份 + restore/backups（+ save 后独立 PCD 体检） | 建图/资产 |
| `c8d8776` | 2026-10-06 | docs(mapping): 建图与重定位调研 —— 术语考据、会/不会用重定位的系统清单、本栈现状与缺口 | 建图/资产 |
| `adc2622` | 2026-10-06 | docs(mapping): 新增 docs/continue_mapping.md（续建/存档/场地隔离全流程）+ 三处指针 | 建图/资产 |
| `0a9c5b5` | 2026-10-06 | feat(mapping): mode:=mapping 支持「上次基础上继续建图」+ 换场地续建默认拒绝 | 建图/资产 |
| `8b14aec` | 2026-10-06 | feat(perception): cloud_accumulator —— 3D 先验也能跨会话续建（map 系累积 + 同名存档 + 同一份场地守卫） | 感知/代价图 |
| `10f2a84` | 2026-10-06 | feat(mapping): 地图存档守卫 map_asset_guard.py + map_archive.sh（save/info/list/adopt） | 建图/资产 |
| `5c3d7af` | 2026-10-06 | docs(timestamp): 全仓时间戳构造审计 —— 1 ns 截断类 bug 与戳/队列/QoS 风险 | 定位/LIO |
| `5410412` | 2026-10-06 | feat(diag): 逐纳秒戳差仪器 —— 探针加整数 ns 指标 + 离线复算器 | 工具 |
| `a281636` | 2026-10-06 | fix(small_point_lio): TF/odom 的戳由"截断"改"四舍五入"，修掉 /scan 被 tf2 QueueFull 顶掉 | 定位/LIO |
| `c73e9d0` | 2026-10-06 | docs(mapping): /scan 丢帧根因考证（1 ns 戳错位）+ 时间戳链实测 + scan_queue_size A/B | 建图/资产 |
| `fc3382d` | 2026-10-06 | docs(traversability): 新增 3D 派生可通行性设计计划（坡度/落空/净空，plan only） | 感知/代价图 |
| `b079175` | 2026-10-06 | docs(mapping): 补 §10「为什么 /scan 看不见坡道」——给 bench 使用者的链路解释 | 建图/资产 |
| `7afbe01` | 2026-10-06 | feat(mapping): 2D 先验图改从 3D 点云投影（pcd_to_nav2_map）+ 建图稳健性 A/B（丢帧/跳变哨兵/坡道段判定） | 导航/限速 |
| `1a9ce8c` | 2026-10-06 | feat(ground): 新增地面分割槽位 ground（linefit|patchwork，默认 linefit）+ vendored Patchwork++ | 感知/代价图 |
| `8170abe` | 2026-10-05 | docs(mapping): 明确 --pose-source gt 的含义/理由/代价（跑图用仿真真值闭环，LIO 质量另用 ATE 单独报） | 建图/资产 |
| `5bee8fb` | 2026-10-05 | feat(mapping): lio:=small_point_lio 建图打通 + RMUC2026 先验产物（2D+PCD，gicp 验证 PASS） | 定位/LIO |
| `f3e4c0b` | 2026-10-05 | docs(readme): docs/ 索引表补一行 worlds.md | 文档 |
| `eb7b811` | 2026-10-05 | feat(tools): 场地 STL ⇒ world + 2D 图 + PCD 一条命令（tools/scripts/world/stl_to_world.py） | 工具 |
| `7e7b823` | 2026-10-05 | docs(worlds): RMUC2026 世界说明 —— 资产/转换/出生点/地图来源/实测结论/坑 | 文档 |
| `280888e` | 2026-10-05 | feat(nav): RMUC2026 的 2D 栅格图与先验 PCD（从 STL 几何生成，非实跑建图） | 导航/限速 |
| `7c0006b` | 2026-10-05 | feat(sim): 新场地 world:=RMUC2026 —— 由附件 STL 生成的世界/模型/mesh + 出生点 | 模型/斜装 |
| `b2c8e1a` | 2026-10-05 | docs(stvl_local): 局部 STVL 的 A/B 实测（四次跑）+ 结论 + 回退 + 未验证清单 | 文档 |
| `4798ea2` | 2026-10-05 | feat(nav): local_obstacle 新增 stvl / stvl_both —— 局部代价地图也上 STVL | 导航/限速 |
| `d42a413` | 2026-10-05 | docs(small_point_lio): 全栈闭环实测（A/B）+ 调参结论 + launch 缺包修复，替换"调参中/全栈未验证" | 定位/LIO |
| `607dcab` | 2026-10-05 | fix(launch): 槽位缺包不再挡住所有组合 —— 包查询改惰性解析（small_point_lio/icp/gicp） | 定位/LIO |
| `830c5fb` | 2026-10-05 | docs(lio_slots): 扫参示例命令改用随仓库提交的 tools/lio_sweep_configs/（.tmp_sweep 是临时目录，不该写进文档） | 定位/LIO |
| `680a380` | 2026-10-05 | docs(lio_slots): 调参后重复性补第 4 次实测（6.88/6.90/6.92/6.93 m，末位置逐位一致） | 定位/LIO |
| `2ff42d4` | 2026-10-05 | tune(small_point_lio)+docs: 仿真参数调优 —— 只改 4 键，轨迹 30.2 m → 6.9 m（≈ FAST-LIO 参照） | 定位/LIO |
| `b3995b5` | 2026-10-05 | feat(lio)+tools: small_point_lio 离线扫参驱动（并行、多窗口）+ 判读脚本补 JSON/轨迹输出 | 定位/LIO |
| `2ba6625` | 2026-10-05 | docs(algorithm_matrix): §9.2.3 表改用 localization:=small_gicp 命名，§9.2.2 补历史口径说明 | 定位/LIO |
| `30b4e8e` | 2026-10-05 | docs(algorithm_matrix): 五路对照改用 localization:=small_gicp，删掉"临时改 YAML + git checkout"旧流程 | 定位/LIO |
| `2c42339` | 2026-10-05 | feat(localization)+docs: 新增 localization:=small_gicp 槽 —— backend 由 launch 注入，槽位层面与 gicp 并列 | 定位/LIO |
| `6bfea21` | 2026-10-05 | docs(lio): 登记 lio:=pointlio 的节点级复核事实 + lio:=small_point_lio 的真实状态（精度未达标） | 定位/LIO |
| `680a834` | 2026-10-05 | docs(localization): beluga SIGINT abort 的 respawn 审计（结论：不改代码）+ gicp 两后端并列 | 定位/LIO |
| `0a2e74d` | 2026-10-05 | feat(lio)+docs: 新增 lio:=small_point_lio 槽 + 节点级验证工具与实测记录 | 定位/LIO |
| `98baa55` | 2026-10-05 | vendor(small_point_lio): 引入 Yancey2023/small_point_lio@688d75c（MIT）+ timebase 兜底补丁 + 仿真参数 | 定位/LIO |
| `c2cb002` | 2026-10-05 | docs(algorithm_matrix)+§9.2: 新构建口径下的五路对照 —— 五路全 PASS、missed-rate 全 0、small_gicp align 快 5.4× | 定位/LIO |
| `20d1fb1` | 2026-10-05 | feat(regress): nav_smoke_regression.py 加 --localization <NAME> —— 让快照自证是哪一路定位 | 定位/LIO |
| `c7bcd6d` | 2026-10-05 | fix(gicp_registration)+docs: /initialpose 被"点云 + align"饿死 —— handoff/apply 拆分 + 第三个回调组（22.7 s → 0.24 s） | 定位/LIO |
| `6ae2ada` | 2026-10-05 | build+docs: 修构建口径 —— colcon 默认不设 CMAKE_BUILD_TYPE（6 个包整个 -O0），提交 colcon_defaults.yaml；实测 PCL GICP 369.9→11.3 ms（32.7×） | 文档 |
| `c655d77` | 2026-10-05 | fix(localization)+docs: localization:=beluga 启动即崩 —— 删掉 laser_min_range(-1.0)，beluga 对该键声明的是 [0, DBL_MAX] | 定位/LIO |
| `d60c21e` | 2026-10-05 | feat(gicp_registration)+docs: 可切换配准后端 backend=small_gicp（vendored koide3/small_gicp@v1.0.1）—— 默认仍是 pcl | 定位/LIO |
| `21fc301` | 2026-10-05 | feat(localization)+docs: 新增 localization:=beluga 槽（beluga_amcl 2.1.1 = AMCL 的现代实现） | 定位/LIO |
| `5c9c0a0` | 2026-10-05 | feat(analysis)+docs: Scan Context 同源性硬门（离线 bag 对拍）—— 同源性 PASS，严格判据 INCONCLUSIVE(偏 PASS) | 工具 |
| `b626f37` | 2026-10-05 | feat(analysis)+docs: Scan Context A1 资产探针与报告 —— 判定 GO（有条件） | 工具 |
| `413415d` | 2026-10-05 | docs: 新增 Scan Context 全局重定位方案（738 行，A~F） | 文档 |
| `34c1eb3` | 2026-10-05 | docs(algorithm_matrix §九): 记录本轮重定位线结果 + §9.1 三方对比协议 | 文档 |
| `4445fe3` | 2026-10-05 | feat(regress): 新增 compare_regress_snapshots.py —— 回归快照一键对比表 | 工具 |
| `708b35d` | 2026-10-05 | docs(localization_slots §7): 记录 gicp 验收结论（/tf 满速、fitness 2.3e-3 m²、PASS 3.2s/0 恢复）+ 五条遗留（settle 阈值偏严、总算力不足、iterations 不降 CPU、资产退化、三方对比暂缓） | 定位/LIO |
| `8b47ff9` | 2026-10-05 | fix(gicp): TF 定时器搬进独立回调组 + MultiThreadedExecutor（治 350 ms align 把 50 Hz TF 饿到 2.3 Hz ⇒ nav2 extrapolation） | 导航/限速 |
| `098078d` | 2026-10-05 | tune(gicp): 两级下采样 leaf 0.10/0.05 + max_corr 1.5（照抄 COD 2025 small_gicp_relocalization，治 map→odom 30 s 漂 8 cm） | 定位/LIO |
| `c8863f1` | 2026-10-05 | tune(gicp): tf_lookahead_sec 0.3 -> 0.45（覆盖 CPU 过载时 TF 定时器被饿死的最坏情况） | 定位/LIO |
| `629c971` | 2026-10-05 | feat(regress): nav_smoke_regression 发目标语义对齐人类 RViz（yaw auto=车当前朝向 / 等定位稳定 / 目标可用性预检） | 工具 |
| `36a71cf` | 2026-10-05 | fix(gicp_registration): map→odom/~/pose 盖 now+tf_lookahead_sec（≈AMCL transform_tolerance），修 nav2 "extrapolation into the future" | 导航/限速 |
| `28deaf1` | 2026-10-05 | feat(gicp_registration): 新增 GICP 重定位槽 localization:=gicp（只发 map→odom + 健康信号） | 定位/LIO |
| `47d7411` | 2026-10-05 | feat(params): goal checker 改 PositionGoalChecker（全向车位置-only） | 装配/参数 |
| `f033d96` | 2026-10-05 | fix(icp_registration): 点云订阅改用 SensorDataQoS(BEST_EFFORT)，/initialpose 保持 RELIABLE | 定位/LIO |
| `53f7a58` | 2026-10-05 | docs(定位槽位): 决策记录 —— 先试 icp；slam_toolbox 暂缓（保留入口，附客观补充：更新率不低、短板在资产维护与全局重定位弱）；gicp 提上日程 | 定位/LIO |
| `b467990` | 2026-10-05 | docs: 新增重定位槽位登记表（localization_slots.md） | 定位/LIO |
| `48703e6` | 2026-10-05 | fix(amcl): 高速跟踪参数 —— transform_tolerance 1.0→0.3 / update_min_d 0.25→0.05 / update_min_a 0.2→0.05 / recovery_alpha_slow 0.0→0.001 / recovery_alpha_fast 0.0→0.1 | 定位/LIO |
| `7097602` | 2026-10-05 | docs: 新增 Kinodynamic_esdf_mpc 调研笔记（270 行，含与 MPPI 的本质差异、引入成本与许可风险） | 文档 |
| `bbe7d43` | 2026-10-05 | refactor(params): nav2 参数从「一个组合一份 full copy」改为 base + 槽位文件（8/8 组合逐键 0 差异） | 导航/限速 |
| `419df04` | 2026-10-05 | feat(params): 补齐 planner×nav 组合文件（dwb/teb/mppi × smac2d） | 导航/限速 |
| `09578ed` | 2026-10-05 | docs: 补入 6 份调研文档（此前只写在磁盘、未入库） | 文档 |
| `effb443` | 2026-10-05 | fix(params): smac2d 变体同步 robot_radius 0.40 -> 0.22（与 dc03e11 对齐） | 导航/限速 |
| `dc116c0` | 2026-10-05 | feat(nav2): 新增 nav:=mppi 槽位（默认仍是 rpp ⇒ 现有已验证路径零影响）—— MPPI 控制器 + 逐键核实 1.1.20 + model_dt=1/controller_frequency | 导航/限速 |
| `50b51e9` | 2026-10-05 | docs(工单 §I): 卡住恢复链核查（只读，不改代码）—— 内置 BT/behavior_server/1.1.20 插件逐项核实 + 用户可执行的验证步骤 + 6 条缺口 | 文档 |
| `dc03e11` | 2026-10-05 | fix(nav2): robot_radius 0.40 → 0.22（回到真实车体半径）—— 消除 costmap 几何 ≠ 真实几何 导致的「一进带就卡死」 | 感知/代价图 |
| `2017f3a` | 2026-10-05 | docs(工单 §H): 记录 planner:=smac2d 实施（commit d4e662b、12 个已核实键、剔除 refinement_num 等）+ 新坑『新增 params 文件必须先 colcon build』+ 验收清单 | 导航/限速 |
| `d4e662b` | 2026-10-05 | feat(nav2): 新增 planner 槽位（默认 navfn ⇒ 现有已验证路径零影响）—— A/B COD 2026 风格 SmacPlanner2D | 导航/限速 |
| `c8f8868` | 2026-10-05 | docs(工单 §G): small_point_lio 排期约定 —— 可后面补且不阻塞 D3/D4；三条风险均为可控工程量；零成本预研动作=独立进程验证'喂得进去' | 定位/LIO |
| `2aca994` | 2026-10-05 | docs(工单 §F): 记录候选实验「两层 STVL」（全局+局部）—— 含他们参数、五个连带效应（时间维/吃原生点云/是删地面分割的前提/必须配裁剪盒/清除来源增加）、我们的现状与分两步尝试方案+判据+回退 | 文档 |
| `a4d7f40` | 2026-10-05 | docs(工单 §E): 决策记录 —— 静态 map→odom 不采用 / linefit 保留 / small_point_lio 提升为 lio 同级槽位（含落地清单与硬门槛） | 定位/LIO |
| `c3b0c96` | 2026-10-05 | docs: COD_NAV 思路整理（重点定位层特殊性）—— 逐层做法/理由/代价 + 定位层为何特殊 + 内在自洽性 + 对 D3/D4 的启示 | 文档 |

---

# §8 术语表（一页）与"接下来该做什么"

## §8.1 术语表（本仓口径）

| 术语 | 一句话定义 |
|---|---|
| **槽位（slot）** | launch 参数化的**可选实现**：`world:=` / `mode:=` / `lio:=` / `mapper:=` / `localization:=` / `nav:=` / `planner:=` / `ground:=` / `local_obstacle:=` / `robot:=` / `robot11_mount:=` / `robot11_visual:=`。**槽位的默认值 = 现行行为**，新增槽位不许影响别的槽位 |
| **`mode:=`** | `mapping`（建图/续建）／`slam_nav`（边建边导）／`nav`（用先验图导航） |
| **真值（gt）** | Gazebo 的世界位姿（`/odom_ground_truth`，**不含 TF**）；跑图时可用 `--pose-source gt` 闭环，把"建图质量"与"LIO 质量"分开量 |
| **RTF** | Real-Time Factor（`/clock` 口径）。⚠️ **只能同批比较**，跨批排序不可复现 |
| **ATE** | Absolute Trajectory Error（对齐后 vs 真值），本仓报 RMSE / max / 每米漂移 |
| **创新** | `本帧配准结果 ⊖ 本帧初值` = GICP 这一帧**想施加的修正量**（平移模长 + 等效 yaw） |
| **inscribed / circumscribed radius** | 内切半径（凸包边到原点最近距离，"能走多窄"）／外接半径（凸包顶点最远，"保证不撞"）。nav2 的 `robot_radius` 语义是**内切** |
| **lethal / inscribed / 膨胀** | nav2 `OccupancyGrid` 取值：**100 = lethal、99 = inscribed、0 = free、−1/255 = unknown**；1~98 是**膨胀层经过翻译的值**，**不要当 cost 读** |
| **高度带** | `p2l.min/max_height`（**点云自带帧**）／`obstacle_layer.min/max_obstacle_height`（**代价图帧**）。两者**帧不同、语义不同**（§0.3） |
| **近地剔除** | `obstacle_near_ground_m`：`dz = z − 局部地面 g(x,y)` ≤ 阈值的点从 obstacle **改判** ground（投影成 2D **之前**） |
| **自击（self-hit）** | 雷达打到自身结构的真实回波（robot11 约 **29%** 的点，中位半径 0.021 m）。**它是物理真实的**，掩膜只把它剔出**判据**，不动 `/segmentation/*` 标签 |
| **自击掩膜** | `self_mask_*` = 113 个 collision AABB ∪ 近场死区（`r_xy ≤ 0.2416` 且 `z ≥ −0.2295`） |
| **走廊 / 承诺特征 / 距离前推** | 限速器的前瞻区间（车前/车后 1~3 m，半宽 0.28 m + 张开 8°）；"承诺"= 记住某个特征的 `v_req` 与距离；"前推"= 每帧按 `上一帧上限 × hold_decay_factor × dt` 扣减剩余距离 |
| **单刚体 / articulated** | 底盘结构两档：`rigid`（把 `j2…j11` 塌成一个刚体，**`planar_move` 自洽**）／`articulated`（回退档，模型体去注释后逐字节等于旧版） |
| **QEM / 剪影 IoU / 单向表面误差** | 二次误差边折叠抽稀；三视剪影交并比（≥0.97 判据）；`out→src` 的表面误差 p99/max（抽稀件偏离原表面的量） |
| **判决性测量** | "如果假设 X 成立，我应该量到什么"——能**否掉**候选的那种测量 |
| **逐字节不变** | 默认路径的行为承诺（三种实现机制见 §2.6.2）；⚠️ 精确表述常常是"**模型体（去注释）**逐字节相同" |
| **未验证清单** | 每份报告末尾的诚实清单（本文 §5 是合并版） |
| **`.tmp_*`** | 跑出来的原始数据目录（**不入库**，已在 `.gitignore`）；引用文档时只能当"可复算的凭据"，不能当仓库资产 |
| **资产命名** | `map/<world>.pgm\|yaml`（当前 2D 先验）／`PCD/<world>.pcd`（当前 3D 先验）／`<world>_<tag>.{posegraph,data,meta.yaml}`（会话存档，一个 tag = 一次会话）／`.prev-<YYYYmmdd-HHMMSS>`（工具自动备份）／`.bak-<YYYYMMDD>`（人工提升备份）／`attic/`（只搬不删） |
| **哨兵 / 云台 / 小陀螺** | RM 哨兵机器人的云台（`j10/j11`）与"边转边打"行为；本仓用 `base_link_fake` + `fake_vel_transform`（`spin_speed`）实现。⚠️ robot11 的云台**没有控制器**（`spin_speed` 对它设 0.0） |
| **坡脚 / 台阶残差** | 坡度突变处（撞击来源）／"格内抬升(p90) − 该格坡度能解释的部分"（窄尺度尖台阶） |
| **SFC / min-snap** | 方形走廊（Safe Flight Corridor）／最小 snap 多项式轨迹（`tdt-nav-kit` 的后端，**研究未落地**） |

## §8.2 接下来该做什么（候选清单，按"阻塞程度"排序）

| 优先级 | 要做什么 | 为什么 | 从哪开始 |
|---|---|---|---|
| **P0** | **把"pitch 还是 roll"用实物/装配图彻底定死** | 目前默认 roll 的依据是"CSV 更可信 + 用户按实物确认"，但**证据仍是单点的**；若实物是 pitch，射线/盲区等价但"最朝下方位"完全不同（±x vs ±y） | `robot_models.md` §10.5 的方位直方图判据 + `livox_tilt_axis:=pitch` 对照跑 |
| **P0** | **换 spawn / 换世界做真正的"长距离 + 需要转向"验收** | 当前出生点**正前方 0.42 m 就堵死**，`--goal-forward 2.0` 永远红叉；角速度通道刚修好，还没有一次真实验收 | `--preflight-reach` 先量可达区，再选目标；或换 `world:=RMUC2026` |
| **P1** | **`sensor` 档链路可用化**：把代价图帧/odom 的重力对齐问题解决（或在代价图侧换重力对齐帧） | 现在 `sensor` 档"帧↔数据自洽"但**41.2% 波束被高度带丢掉**，代价图这一层仍坏 | `Fidelity` §J.5 的两个候选（LIO 侧 z 基准 / 代价图侧高度带或 `global_frame`），**必须单独 A/B 且会动别人的参数目录** |
| **P1** | **补三类缺失的对照跑**：① `bash tools/scripts/tiltmount/_batch_k2.sh`（sensor 档插件 A/B）② `--chassis articulated` 的整栈"修前"基线 ③ `wz` 剂量-响应扫描（0.3/0.6/1.0/1.5） | 这三个直接决定"§M 的修复到底值多少" | `Fidelity` §K.8 3 / §M.10 1、3 |
| **P1** | **`sensor_height` 的 4.3 cm 疑问** | 若真偏 4 cm，`linefit` 的地面线整体抬高 4 cm（与 `max_dist_to_line 0.05` 同量级） | §K.1 末尾/§K.8 6（要查"轮 collision mesh 最低点 vs 关节 origin 的 FK"） |
| **P2** | **真足印多边形**（字符串格式）落地 | 消掉"内切圆 ⇒ 转弯角扫障碍"这个已知折中；`rcl` 那条坑已知道怎么绕（必须写成**字符串**） | `Fidelity` §K.3.3/§K.3.4 |
| **P2** | **`/scan` 里那圈"自身障碍"的处置**（p2l 侧加自击掩膜，或修插件内移） | 它可能正是"局部图被自身障碍填满 ⇒ 规划器认为车被围住"的原因之一 | `robot_models.md` §13.9 的候选修法 |
| **P2** | **限速器的两处已知缺陷**：① 承诺前推改用**真实行程**（订阅 `/odom` 速度而不是用"上限"）② 在 RPP/DWB/TEB 上验证 `setSpeedLimit()` 语义 | 前者在实车/打滑时会**提前释放**；后者决定这套限速能不能换控制器 | `slope_speed_limiting.md` §7 第 2/3 条 |
| **P2** | **全局代价图"静止也偏脏"**：raytrace/原点附近清理；"跑几分钟再看"的对照 | 影响长跑可靠性 | `Fidelity` §K.8 2 |
| **P3** | **视觉/GPU 的真机复测**：真 GPU 上看抽稀够不够、GUI 内存到底省不省 | 本机只有 llvmpipe，"省内存"这条**没测出来** | `robot_models.md` §13.4.3/§13.10 |
| **P3** | **把 patchwork 变成可选默认**：补一次 `mode:=nav` 整栈回归 | 现在它只在离线 + 建图 smoke 上验证过 | `ground_segmentation_slots.md` §7.1/§9 |
| **P3** | **两个 LIO 子模块的时间戳补丁**要不要合（`FAST_LIO` 实测同一 bug；`point_lio` 判定未实测） | 现在只报告未改 | `timestamp_construction_audit.md` §4.4/§9 |
| **P4** | **`tdt-nav-kit` 落地第一版**：纯库 + 薄适配 + `GlobalPlanner` 插件（`planner:=tdt` 槽） | 研究已完成，但 §3.6 的自测清单**一项没测** | `nav2_vs_custom_planner.md` §6.1/§7.3 |
| **P4** | **负障碍（坑/落空）与净空（限高）判据** | `p2l` 高度带抬到 1.226 m 之后"头顶结构"会成为假障碍；换场地/真机必须面对 | `traversability_plan.md` §9 的 B/C 两行 |

---

## 附：这份汇总的自我说明

* **它是"汇总"，不是"新事实"**：本文没有跑任何仿真、没有改任何代码。
  所有数字都来自 §0 开头列的源文档与 `git log`；引用时给了 `§` 或提交 hash。
* **引用优先级**：① 提交（改了什么）→ ② 原文档的**表格与结论节**（数字）→ ③ 本文 §2 的转述。
  三者若冲突，以 ① 与 ② 为准，并按 §5.4 的方式记录口径。
* **维护约定**：以后每有一条新改动，**先在原文档里写全**（现象/原因/过程/改动/原理/效果/回退/证据），
  再在本文 §2 追加一条、§4 的总账表补一行、§7 的提交索引补一行。
  **本文永远保持"一份能读完的总入口"这个定位**，不复制原文档的长篇推导。
* **建议的阅读顺序**：§0（5 分钟）→ §1.2（推翻过什么）→ §3（原则）→ 你要找的那一层（§2.x）→ §5（别把这些当成已知）。
