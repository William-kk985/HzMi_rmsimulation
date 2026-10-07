# 雷达"斜 30°"到底斜在哪儿：逐帧点云账本 + 代价图归因 + `robot11_mount` 开关（2026-10-08）

> 触发 = 用户 2026-10-08 的原话（逐字）：
> 「我就是要学斜放置怎么处理…雷达给我按我给你的放好，算法不管后面再看怎么调试，我要看到倾斜放置
> 最原始的效果…点云少了很多…肯定被裁减了…而且从规划来看感觉就是斜放置的…那个膨胀半径感觉一直把
> 地面当成障碍物算进去，但是 urdf/雷达/点云图像都是平放置的很奇怪，这个查一下问题所在」
>
> 本文**只做三件事**：**(A)** 用数据说清"到底什么斜、什么没斜"；**(B)** 把每一帧的点数**逐级对账**，
> 给出"点云有没有被裁"的定论；**(C)** 给一个**默认不变**的开关 `robot11_mount:=plugin|urdf`，
> 让"物理安装保真（mesh 一起斜）"这条路可以**原样跑起来看**（**不动任何感知参数**）。
> 另附 **(D)** 代价图/膨胀半径的实测归因 —— 用户说的"膨胀半径把地面当障碍物"。
>
> **本主题没有改任何默认值**：`robot11_mount` 默认 `plugin` = 2026-10-07 起的行为；
> 生成物 `sentry_robot_robot11_sim.xacro` 的**去注释渲染结果与改动前逐字节相同**（§C.5 有 sha 证据）。

---

## 0. 一句话结论（三条，都是实测）

| # | 结论 | 关键证据 |
|---|---|---|
| **A** | **斜 30° 的只有"射线方向"**。默认档（`plugin`）里 `livox_frame` 这个**帧**是重力对齐的（TF rpy=0），点云坐标**也是**重力对齐的（拟合地面法向与 link z 夹角 **1.05°**）⇒ **帧与点云一致，都正**；代价是**画出来的雷达 mesh 是平的**（mesh 挂在 link 上，link 没斜）。 | TF `base_link→livox_frame` rpy **0/0/0**；原始云地面拟合 `n=(-0.0138,0.0122,0.99983)`、`d=0.2604`（≈ 雷达离地 0.2595） |
| **B** | **原始点云没有被任何一级"裁"过** —— 它是被**插件按"有没有回波"决定的**：一帧 30000 条射线里只有 **11960 条有回波（39.9%）**，其余 **18040 条（60.1%）在插件里就被丢掉**（打天空/没命中；另有少量 <0.2 m 的近场回波被 `<range><min>0.1</min>` 丢掉）。`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered` 三条话题的**每帧点数一致**（11960 / 11960 / 11968）⇒ **下游一级都没裁**。 | §B 的逐级账本（3 个配置并排） |
| **D** | **"自击环 + 膨胀"不是"车被包住"的原因**：实测**两张代价图里 0.386 m 以内一个 lethal 格都没有**（自击点在 <0.12 m，被 `obstacle_min_range: 0.1` 这类门限挡在外面）。真正把车包住的是 **0.39~0.75 m 处的 lethal 格**（近场地面被留在 obstacle 输出里 + 场地上 ~0.15 m 高的低矮件），配上本槽位的 **`robot_radius 0.3565` + `inflation_radius 0.70/0.75`** ⇒ 车自己那格是 **inscribed（99）**，车半径圆（0.3565 m）内 **513/996 格 ≥99、只有 6 格 free** ⇒ 控制器直接拒绝走：`RegulatedPurePursuitController detected collision ahead` → `Controller patience exceeded` → 自旋恢复。 | §D（含默认模型对照：车那格是 **free(0)**、圆内 **0/996** 是 ≥99） |

---

## A. 到底什么斜了、什么没斜（用数据说，不看文档）

### A.1 三个"斜/不斜"的判据

| 判据 | 怎么量 | 默认档（`plugin`）实测 | `urdf` 档实测 |
|---|---|---|---|
| **① 帧**（TF） | `tf2_echo base_link livox_frame` | xyz `[0.000562, 0.130916, 0.157028]`、**rpy `[0,0,0]` 度** | xyz 同上、**rpy `[-30.000, 0, 0]` 度**（四元数 `[-0.259,0,0,0.966]`） |
| **② 射线方向**（物理） | 世界里射线打哪儿：地面最近环半径、仰角谱 | 地面最近环 **0.39 m**（原始云里地面带 `rxy` p5 = 0.394） | **同左**（同一朵云，见 §A.3） |
| **③ 点云坐标** | 对原始云做 RANSAC 平面拟合，取最大平面（= 地面）的法向与 link z 的夹角 | 法向 `(-0.01376, 0.0122, 0.99983)`、与 link z 夹角 **1.054°**、平面 z = **−0.26038**（几何真值 = 雷达离地 0.2595） | **完全相同的数**（1.054° / −0.26038） |
| **④ 画出来的 mesh** | mesh 挂在 `livox_frame` 上 ⇒ 跟着 link 走 | link 没斜 ⇒ **mesh 是平的**（用户看到的现象） | link 斜 30° ⇒ **mesh 一起斜**（这就是"按实物放好"） |

> ③ 的 `1.054°` 不是"斜 30°"：那是车自己停在场地上的俯仰/侧倾 + 拟合误差；
> 如果点云表达在斜 30° 的传感器系里，这个角应该是 **~30°**。

### A.2 `/livox/lidar/pointcloud` 与 `/cloud_registered` 的 `frame_id` 与地面 z（逐字实测）

| 话题 | `header.frame_id` | 最大平面法向（自己的帧里） | 与 link/odom z 的夹角 | 平面高度 | 判读 |
|---|---|---|---|---|---|
| `/livox/lidar/pointcloud`（`plugin` 档） | **`livox_frame`** | `(-0.01376, 0.0122, 0.99983)` | **1.054°** | **−0.2604** | 地面**水平** ⇒ 点云在**水平（重力对齐）帧**里 |
| `/livox/lidar/pointcloud`（`urdf` 档） | **`livox_frame`** | 同上（**逐点相同**） | **1.054°** | **−0.2604** | 点云坐标**没变**，但**这个帧自己斜了 30°** ⇒ 帧与点云**不一致 30°** |
| `/cloud_registered`（`plugin` 档） | **`odom`** | `(-0.00885, 0.0177, 0.9998)` | **1.134°** | −0.0036（点在雷达附近时会随车动） | LIO 用 TF 把点搬到 odom ⇒ 仍水平 |
| `/cloud_registered`（`urdf` 档） | **`odom`** | `(-0.01102, 0.51067, 0.85971)` | **30.716°** | — | **地面在 odom 里是 30.7° 的斜面**；`z < 水平地面` 的点 **7506 / 11622 = 64.6%**（`plugin` 档是 **0 / 11620**） |

**这两行就是"帧到底正不正"的判决**：同一个 `/cloud_registered` 话题，
`plugin` 档的地面是水平的（1.13°），`urdf` 档的地面是 **30.7° 斜面** —— 因为 LIO 在算
`T_odom→base_link` 时会用 `lookupTransform("base_link", "livox_frame")`（源码硬编码，见
`small_point_lio_node.cpp`），`urdf` 档这条 TF 里**真的带 30° 旋转** ⇒ 把"本来就水平的点"
又转了 30°。**⇒ 用户"从规划来看感觉就是斜放置的"这个感觉，在 `urdf` 档是物理事实（点云被多转了一次）。**

### A.3 一个反直觉但很关键的实测：**点云坐标与"倾角记在哪儿"无关**

`plugin` 档与 `urdf` 档各抓一帧 `/livox/lidar/pointcloud`（11568 点 / 11568 点），**按索引逐点比**：

| 口径 | 结果 |
|---|---|
| 逐点（同一索引）最大差 | **5×10⁻⁵ m**（= 我落盘 CSV 的 `%.5f` 量化；中位数 **0**） |
| 差 < 5 mm 的点占比 | **100.0%** |

⇒ **把 30° 从插件参数搬到 URDF 关节，对"点云里每个点的坐标"没有任何影响**。机理（读源码 +
数据一致）：插件的出点公式是 `axis = sensor_rot · mount_rot_ · ray·x̂`、`point = range · axis`
（`livox_points_plugin.cpp`），其中 `sensor_rot = laserCollision->RelativePose().Rot()`。
`sensor_rot` 会**跟着 link 的世界姿态走** ⇒ 倾角记在关节时它自己就带上了那 30°，
于是 `sensor_rot·mount_rot_` 两种档**乘出来是同一个矩阵**，射线与点都一致。

**这条的后果正是用户要看的"原始效果"**：
* `plugin` 档 = **点云正 + 帧正**（自洽，下游所有"假设点云重力对齐"的环节都对）；
* `urdf` 档 = **点云正 + 帧斜 30°**（**不自洽**）⇒ 任何**用 TF 变换这朵云**的消费者都会看到
  一个斜 30° 的世界（`/cloud_registered`、nav2 代价图、`p2l` 的 `target_frame≠""`、
  开 GUI 时 RViz 的固定帧不是 `livox_frame` 的显示……）。

### A.4 为什么"URDF / 雷达 / 点云图像"看起来都是平放的（答复用户的"很奇怪"）

| 用户看到的东西 | 真相 | 依据 |
|---|---|---|
| **URDF 里的雷达是平的** | 关节 `body_to_livox` 的 rpy 被写成 `0 0 0`（Phase 3 的决定：帧重力对齐），**倾角搬进了插件参数** `<tilt_rpy>` | 生成物第 921 行 `rpy="0 0 0"`（`plugin` 档）＋ gzserver 日志 `tilt_rpy = [-0.523598776 0 0]` |
| **RViz 里的雷达 mesh 是平的** | mesh（`l12.STL`）挂在 `livox_frame` 上，link 不斜 ⇒ mesh 不斜。**mesh 的朝向只跟 link 走，跟射线无关** | 同一个 TF：rpy 0 |
| **RViz 里的点云像是平放的** | 因为点云坐标**确实**是重力对齐的（§A.2 第一行：地面法向夹角 1.05°）。**这不是假象，是设计** | RANSAC 平面拟合 |
| **但规划/代价图"感觉是斜的"** | 代价图用的是 `/scan`（2D）与 TF：`plugin` 档下 `p2l` 的 `target_frame: ""` ⇒ 不做 TF，`/scan` 直接是"水平面里的一圈"；也就是说**规划侧没有斜**。用户感觉到的"斜"更可能来自 **① 自击/近场点让 `/scan` 的 <1 m 波束占一半（§D）② 代价图把车包住** | §D |

> **所以"点云看着平移不动、但车不走"这件事与"斜"无关** —— 斜（`plugin` 档）只体现在射线方向
> （看得见地面、地面环 0.39 m），不体现在坐标里。用户想要"斜的观感"，要的就是 §C 的 `urdf` 档。

---

## B. "点云少了很多"——逐帧点账本（三个配置并排）

### B.1 口径

* 无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、`unset DISPLAY`），
  `world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0`、车**静止**；
* 探针 `tools/scripts/regress/robot11_mount_probe.py`（本轮新增；只订阅 + 只发 `/cmd_vel_chassis` 零速）；
* 原始数据：`.tmp_robotslot/r11m_plugin/`（`robot:=robot11`，默认档，含一次 `/goal_pose`）、
  `.tmp_robotslot/r11m_plugin2/`（同上，**静止、不发目标**，用于代价图复测）、
  `.tmp_robotslot/r11m_default/`（不带 `robot:=`，默认模型）、
  `.tmp_robotslot/r11m_urdf/`（`robot11_mount:=urdf`）、`.tmp_robotslot/r11m_urdf2/`（urdf 短跑，取 TF/参数）。

### B.2 每帧点数（中位数；括号里是 `min~max`）

| # | 配置 | `/livox/lidar`（CustomMsg） | `/livox/lidar/pointcloud` | `/cloud_registered` | `/segmentation/ground` | `/segmentation/obstacle` | `/scan` 有限波束 | `/global_costmap/voxel_grid` |
|---|---|---|---|---|---|---|---|---|
| (i) | **`robot:=robot11`（`plugin`，今天）** | **11960.5** (11377~12808) | **11960.5**（同左） | **11968** (11377~25271) | **5725** | **6227** | **829** | 未采（探针当时未订阅） |
| (i') | 同上，第二次（静止、不发目标；用于代价图复测） | **11917.5** | **11917.5** | **11933** | **5681** | **6197** | **832** | **734** |
| (ii) | **默认模型**（`robot` 留空） | **6373.5** (5341~6548) | **6375** (5341~6548) | **6375** (6156~6548) | **2612** | **3717** | **1158** | **1091** |
| (iii) | **`robot11_mount:=urdf`** | **11911.5** (11375~12139) | **11912**（同左） | **11941** (11375~12139) | **5684** | **6210** | **831** | **146** |

**`/scan` 细带（条/帧，中位）** —— 这是后面 §D 归因的关键输入：

| 配置 | 0.00–0.05 | 0.05–0.10 | 0.10–0.15 | 0.15–0.20 | 0.20–0.30 | 0.30–0.50 | 0.50–1.0 | 1–2 | 2–4 | 4–7 |
|---|---|---|---|---|---|---|---|---|---|---|
| **`robot11` `plugin`**（第二次） | 0.0 | **66.6** | 0.07 | **0.0** | **97.5** | 103.4 | 179.6 | 239.9 | 121.6 | 62.2 |
| `robot11` `plugin`（第一次，粗带） | — | — | — | — | — | — | <0.3 合计 **145.3**；0.3–1 = 264.7 | 258.5 | 117.9 | 63.8 |
| `robot11_mount:=urdf` | 0.0 | 57.1 | 0.06 | 0.0 | 101.2 | 104.0 | 178.3 | 241.5 | 119.7 | 61.1 |
| **默认模型** | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.03 | 15.1 | 279.2 | 623.0 | 222.0 |

（默认模型最近的有限波束 = **0.4821 m** —— 这正是 §11.2 记的"默认模型没有近场自击"。）

**读法（三条）**：
1. **CustomMsg 与 PointCloud2 逐帧同数**（11960.5 = 11960.5）⇒ 插件里两条发布走的是同一批点，
   **没有任何一路被额外裁**。
2. **`/cloud_registered` ≈ 原始云**（11968 vs 11960）⇒ LIO **没有再抽稀**。这一点值得写清：
   `small_point_lio` 的 `min_distance: 0.5` / `space_downsample` **只作用于送进 ESKF 的点**
   （`preprocess.cpp` 的 `filtered_points` → `point_deque`），而**发布的那朵云用的是
   `dense_point_deque`（未过滤的原始点）**（`small_point_lio.cpp` 的 `dense_point_imu_frame` 分支）
   ⇒ 所以"RViz 里点少了很多"**不是** LIO 抽稀造成的。
   （`mid360_sim_tuned.yaml` 里 `space_downsample: false`，即便开了也只影响滤波器输入。）
3. **默认模型 6374 vs robot11 11960**：robot11 的点**更多**（2 倍），因为它 30° 下俯后
   "朝下那一半视场"里有地面/场地（见 B.3 的 (d) 项）。

### B.3 逐帧细账（单帧，`robot:=robot11` `plugin` 档 `/livox/lidar/pointcloud`，n = 11568）

| 项 | 点数 | 占该帧 | 说明/判据 |
|---|---|---|---|
| (a) **地面以下**（`z < −0.2595 − 0.02`，水平面模型） | **1770** | 15.3% | 这是"低于几何地面平面"的点：**近场自击点**（在雷达下方/侧下方的凹槽内壁）＋ 噪声；不是"地面被切掉" |
| (b) **自击 `r < 0.12 m`** | **3294** | **28.5%** | 与 Phase 4/5 的 28.7%/29.0% 同量级；`r<0.05` = **1668（14.4%）** |
| (b') `r < 0.5 m` | 3767 | 32.6% | 这里面 87% 是 (b) |
| (c) **超出插件量程被丢** | 见 (d) | — | 插件 `point = range·axis`，`<range><min>0.1</min><max>200</max>`；**射线起点在 0.1 m**（`start_point = minDist·axis`）⇒ `range ≤ 0.1`（真实距离 ≤ 0.2 m）与 `range ≥ 200` 都被 `continue` 丢弃 |
| (d) **"没有回波"**（插件里就没发） | **18040** | **60.1%** | **30000（`<samples>`）− 11960（发出的）= 18040**。其中绝大多数是**打天空**：MID-360 的自身视场是 −7.22°…+55.22°，30° 下俯后世界仰角谱 = **−37.22°…+85.22°**，朝上的射线在场地里没有回波 |
| (e) `r > 4 m` | 575 | 5.0% | 远场没被裁（`plugin` 档 `/scan` 的 >4 m 波束 63.8/帧） |
| (f) 低于插件 `max`（200 m） | 0 | 0% | 场地尺寸 ~17×10 m ⇒ 不可能触发 |

**⇒ "原始云被裁了吗"的定论**：
* **没有任何一级"累加/抽稀/限幅"环节在裁原始云**；
* 原始云的规模**完全由"一帧 30000 条射线里有多少条真的打到了东西"决定**（本槽位 39.9%）；
* 相对 Phase 2 的 19717 点/帧，现在是 11960 —— **少掉的 7757 点主要是"假自击"**：
  Phase 2 的实心 4 box 让 75.5% 的点落在 r<0.12 m（自击在碰撞盒内壁上）；
  Phase 3 把碰撞挖出视锥后这类点降到 28.5%，**少掉的正是物理上不存在的那些点**
  （`.tmp_robotslot/r11p3a` 与 §11.2 的表）。
* **如果用户在 RViz 里觉得"点少/稀"**：那是因为 ① 一帧只有 ~1.2 万点（60% 的射线没有回波）、
  ② 其中 28.5% 挤在雷达 0.12 m 以内的自击团里（视觉上是一团而不是一圈）、
  ③ RViz 里 `/livox/lidar/pointcloud` 与 `/cloud_registered` 是两个不同显示，
  `nav2.rviz` **根本没有**这两个点云的 display（只有 `LaserScan`/voxel 类），
  `pointlio.rviz` 才有 `/cloud_registered`（见 §G 第 4 项：RViz 的显示设置本身本轮**没有**测）。

### B.4 每一级"谁动了什么"（可复算）

| 级 | 实现位置 | 对点数的影响 | 本轮实测 |
|---|---|---|---|
| 射线生成 | 插件 `<samples>30000</samples>` + `mid360.csv`（800000 行，逐帧滑窗） | 每帧固定 **30000** 条 | gzserver 日志 `scan info size: 800000 / sample: 30000 / downsample: 1` |
| 回波过滤 | 插件 `if (range <= RangeMin() \|\| range >= RangeMax()) continue;` | **−18040/帧** | 30000 − 11960 |
| 点重建 | 插件 `point = range·axis`（**漏了 `+ start = 0.1·axis`** ⇒ 系统内移 0.1 m） | 数量不变；**坐标整体朝传感器内移 0.1 m** | §12.3.1（Phase 4 的独立证据：近场中位半径 0.021 m、地面 z 随距离单调变化） |
| 点云表达 | 插件 `sensor_rot·mount_rot_·ray` | 数量不变；决定"帧正/帧斜" | §A.3 |
| LIO 输入过滤 | `min_distance 0.5` / `space_downsample(false)` / `point_filter_num 1` | **只影响 ESKF 输入**，不影响发布 | `mid360_sim_tuned.yaml` + `preprocess.cpp` |
| LIO 发布 | `dense_point_deque` → `R(base_link←livox_frame)·p + t` | 数量不变（11968 ≈ 11960） | §B.2 |
| 地面分割 | linefit | 分成 ground / obstacle 两路（**不丢点**） | 5725 + 6227 = 11952 ≈ 11960（差 8 点 = 帧边界） |
| `/scan` | `pointcloud_to_laserscan`（`range_min 0.05 / range_max 10 / 高度带 ±1.0`） | **15204 个角度 bin 里只有 829 个有回波**；每 bin 取**最近**点 | §B.2 + `scan` 分带 |

---

## C. `robot11_mount:=plugin|urdf` —— 物理安装保真档（opt-in，默认不变）

### C.1 开关语义

| 档 | `body_to_livox` 关节 rpy | 插件 `<tilt_rpy>` | 帧（TF） | 点云坐标 | 画出来的 mesh |
|---|---|---|---|---|---|
| **`plugin`（默认 = 今天）** | `0 0 0` | `-0.523598775598293 0 0` | 重力对齐（rpy 0） | 重力对齐 | **平的** |
| **`urdf`（新，物理安装保真）** | `$(arg livox_tilt_rpy)` = `-0.523598775598293 0 0`（上游/CSV 字面值） | `0 0 0`（单位阵） | **斜 30°** | **还是重力对齐**（§A.3） | **斜 30°** |

**两档的世界射线方向逐条相同**（§A.3 的 5×10⁻⁵ m 逐点一致就是证据）。
`urdf` 档**不动任何感知参数**（`sensor_height` 仍 0.2595、`p2l` 高度带仍 ±1.0、
自击掩膜仍开）—— 要的就是"倾斜放置最原始的效果"。

### C.2 怎么用（复制即可）

```bash
# ① 构建（install/ 是逐文件符号链接；本主题只改了生成器/生成物/launch，插件二进制没动）
colcon build --symlink-install --packages-select rm_nav_bringup robot11

# ② 物理安装保真档（mesh 一起斜）：只加一个参数
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping \
    lio:=small_point_lio robot:=robot11 robot11_mount:=urdf spin_speed:=0.0

# ③ 今天的行为（默认；不写 robot11_mount 也是这个）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=mapping \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0

# ④ 一条命令量齐（隔离无头跑 + 参数/TF 回读 + 逐帧点账本）→ .tmp_robotslot/<tag>/
tools/scripts/regress/run_robot11_mount_probe.sh r11m_urdf --settle 25 --duration 45 \
    --frames 3 --dump-dir .tmp_robotslot/r11m_urdf/frames \
    --grid-dump .tmp_robotslot/r11m_urdf/local_grid.npz -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
    robot11_mount:=urdf spin_speed:=0.0
```

**回退**：去掉 `robot11_mount:=urdf`（默认 `plugin`）；生成物与 `git revert` 的对应关系见 §F.3。
`robot11_mount` 只对 `robot:=robot11` 生效（launch 只在那个分支里传 `livox_mount`），
取值只有 `plugin|urdf`（选错**直接报错**，不静默回退）。

### C.3 `urdf` 档的实测（同世界/同出生点/同参数/静止；`.tmp_robotslot/r11m_urdf/`）

| 指标 | `plugin`（今天） | **`urdf`（物理安装保真）** | 判读 |
|---|---|---|---|
| TF `base_link→livox_frame` rpy | 0/0/0 | **−30.000/0/0** | 帧真的斜了 |
| TF `base_link→imu_link` xyz / rpy | `[0.000562,0.130916,0.107028]` / 0 | **`[0.000562,0.105916,0.113727]` / −30°** | IMU 跟着雷达一起斜（上游 j 是固定关节） |
| gzserver 插件日志 `tilt_rpy` | `[-0.5236,0,0]` | **`[0,0,0]`** | 倾角只剩在关节上 |
| `/livox/lidar/pointcloud` 点/帧 | 11960.5 | **11911.5** | 同一朵云（逐点差 ≤5×10⁻⁵ m） |
| `/segmentation/ground` 点/帧 | 5725 | **5684** | linefit 输入是同一朵云 ⇒ 几乎不变（`sensor_height` 没改，仍是 0.2595） |
| `/segmentation/obstacle` 点/帧 | 6227 | **6210** | 同上 |
| `/scan` 有限波束/帧 | 829 | **831** | `p2l` 的 `target_frame: ""` ⇒ 不做 TF ⇒ **`/scan` 本身几乎不受影响** |
| `/scan` 分带（`urdf`，细带） | （本轮只采到粗带：<0.3 = 145.3、0.3–1 = 264.7、1–2 = 258.5、2–4 = 117.9、4–7 = 63.8） | **0.05–0.10 = 57.1、0.10–0.30 = 101.3、0.30–0.50 = 104.0、0.50–1.0 = 178.3、1–2 = 241.5、2–4 = 119.7、4–7 = 61.1**，最近 **0.05 m** | <1 m 合计 ≈ 440/帧（≈ 一半是自击/近场） |
| **`/cloud_registered` 里地面是不是斜面** | **不是**（法向与 odom z 夹角 1.13°；低于水平面的点 **0/11620**） | **是**（**30.72°**；低于水平面的点 **7506/11622 = 64.6%**） | **这就是"斜放置"最直接的观感**：RViz 固定帧取 `odom`/`map` 时，点云/地面是一个 30° 的斜坡 |
| local costmap lethal(100) / inscribed(99) | 340 / 14652 | **75 / 3844** | `urdf` 档反而**更空**：因为 `/scan` 的点经 TF 后被转成**斜平面**，`obstacle_layer` 的 `min/max_obstacle_height(0/2.0)` 把一半波束按高度丢掉了（下面 §C.4 有算式） |
| global costmap lethal / inscribed | 313 / 3156 | **151 / 1934** | 同上（STVL 的 `min_obstacle_height 0.0` 会把"被 TF 抬到 0 以上"的那一半地面点标成障碍） |
| `/global_costmap/voxel_grid` 点/帧 | 未采 | **146**（默认模型是 1091） | `urdf` 档进体素层的点反而**少**（高度带把一半地面滤掉了） |
| LIO 漂移（静止窗，位移差口径） | 0.024 m（该跑含 30 s 目标段） | **0.013 m** | ⚠️ 窗口不同口径不同，**不能当 A/B**（见 §G） |
| RTF（`/clock` 口径） | 0.36 | **0.23** | ⚠️ 三次跑的 RTF 是 0.36 / 0.69（默认模型，中间那次）/ 0.23 ⇒ **排序不可复现，按噪声读**（§G） |

### C.4 `urdf` 档**坏在哪儿**（都是"斜帧"的必然结果，逐条给算式）

1. **`/cloud_registered` 被多转 30°**（实测 30.72°）：LIO 用 TF 把点从 `livox_frame` 搬到
   `odom`，而这条 TF 现在带 −30° ⇒ **本来水平的地面变成 30° 斜面**、64.6% 的点落到水平面以下。
   ⇒ 任何"slam_toolbox / costmap / 点云地图"看到的世界都是斜的。
2. **`p2l` 的高度带（`min_height −1.0 / max_height 1.0`）语义变了**：它作用在**点云自带帧**
   （`target_frame: ""` ⇒ `livox_frame`）。在 `plugin` 档，那个帧是水平的 ⇒ 带 = 真高度；
   在 `urdf` 档，帧斜了 30°，但**点云坐标没变**（§A.3）⇒ 这一路"歪打正着"仍然对。
   **但**：`/scan` 一旦被 TF 消费（nav2 的 `obstacle_layer` 就是），`/scan` 的点就被转成
   **斜平面**：`z_odom = −sin(30°)·r·sin(φ) = −0.5·r·sinφ`（φ = 该波束在 `livox_frame` 里的方位）
   ⇒ 同一个 2D 扫描里，φ=−90° 的波束在 odom 里被抬到 **+0.5r**、φ=+90° 的被压到 **−0.5r**。
   `obstacle_layer` 的 `max_obstacle_height 2.0 / min_obstacle_height 0.0` 于是把
   **|r·sinφ| > 2~4 m 以外的一半波束按高度丢掉** ⇒ 代价图**看不到一半方向的东西**（实测
   lethal 从 340 → 75）。
3. **`linefit` 的 `sensor_height = 0.2595` + `gravity_aligned_frame: ""` 在 `urdf` 档是"幸运地仍然对"**：
   因为点云坐标没跟着帧转（§A.3）。**但这是巧合，不是设计**：如果哪天插件改成"点云真的表达在
   斜的传感器系"（真机驱动的行为），这一路**必须**改成 `gravity_aligned_frame: base_link`
   （该键的 C++ bug 已于 2026-10-07 修好，§11.4）或 `pointcloud_to_laserscan` 开 `target_frame`。
4. **自击掩膜（`self_mask_*`）的 z 门限**：它按"传感器系 + 重力对齐"定义
   （`r_xy ≤ 0.2416 且 z ≥ −0.2295`）。`urdf` 档点云坐标没变 ⇒ 仍然命中；
   但**同一套掩膜盒子（113 个 AABB）是在"帧正"假设下烘出来的**
   （`robot11_self_mask.py --emit`）。若将来点云真的变成"斜系表达"，这套盒子**必须重烘**。
5. **`lio_tf_adapter` 的杆臂**：`plugin` 档是**纯平移** `[-0.000562, -0.130916, -0.107028]`、rpy 0；
   `urdf` 档**必须带 30° 旋转** ⇒ 本主题把 launch 里那段几何改成**跟着 `robot11_mount` 算**
   （`urdf` 档输出 `[-0.000561701, -0.034862345, -0.151448296]` / rpy `[0.523598776,0,0]`，
   与 Phase 2 记录过的数值**逐位相同** ⇒ 公式被两处独立来源印证）。
   ⚠️ `lio:=small_point_lio` 时该节点**不启动**（LIO 自己用 TF 做相似变换）⇒ 本轮的实跑**没有**
   经过这条杆臂；它只是"几何正确"（见 §G）。
6. **`robot_state_publisher` 的 TF 与"点云 frame_id"的契约变松**：`frame_id` 仍是 `livox_frame`，
   但**这个帧的物理含义变了**（从"重力对齐"变成"斜 30°"）。所有"看到 `livox_frame` 就假设它正"
   的手写脚本/离线工具都要重看一遍。

### C.5 这个开关**不可能**影响别的模型/别的槽位（结构性证据 + 实测）

| 证据 | 结果 |
|---|---|
| 生成物的**去注释**渲染：改动前 vs 改动后（`plugin` 档） | **逐字节相同**（`diff` 为空；去注释后两份都是 **35410 B**）。带注释的两份差 746 B = 新增的说明性注释 |
| 显式传 `livox_mount:=plugin` vs 不传 | 渲染结果**同一 sha**（`3b7f2322bd1390c7`） |
| `urdf` 档 vs `plugin` 档（去注释） | **只差 2 行**：`body_to_livox` 的 `origin rpy` 与插件的 `<tilt_rpy>` |
| `livox_mount` 出现在哪些文件 | 只有 `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro`（模板里只有 robot11 的生成器发这个 arg） |
| launch 侧 | `livox_mount:=` 只在 `slot == 'robot11'` 分支里拼进 `xacro` 命令行 ⇒ 其它槽位的 xacro 命令行**逐字节不变** |
| 默认模型实测（`robot` 留空，同世界同 LIO） | `sensor_height 0.226`、`gravity_aligned_frame ""`、`self_mask_enable False`、`robot_radius 0.22`、`inflation 0.5/0.55`（= 改动前）；点/帧 6373.5、地面 2612、**自击 0（<0.5 m 一个点都没有）** |
| 节点集合/时序 | `ros2 node list` 两档一致（无新增节点）；本开关只改 xacro 渲染内容 |
| mesh 解析 | 两档都能 spawn（`SpawnEntity: Successfully spawned entity [robot]`），`decimated` 视觉档在 `robot:=robot11` 下由 launch 追加 `GAZEBO_MODEL_PATH`（`robot11` 包）⇒ 无需手工前缀环境变量。**本轮两种档都是这条路径**，都成功 |

---

## D. "膨胀半径把地面当障碍物"——代价图归因（带数字）

### D.1 场景与量法

车 `robot:=robot11` **静止在空旷地面**（`RMUL2026` 出生点），`mode:=slam_nav`；
探针读 `/local_costmap/costmap`、`/global_costmap/costmap`（nav2 的 `OccupancyGrid`：
**100 = lethal、99 = inscribed、0 = free、−1/255 = unknown**；1~98 是**膨胀层经过 nav2
`cost_translation_table` 翻译过的值**，**不要**当 cost 读），并用 TF 把车的位置换算到格上。
原始数据：`.tmp_robotslot/r11m_plugin/{probe.json,local_grid.npz}`（第一次，含目标）、
`.tmp_robotslot/r11m_plugin2/{probe.json,local_grid.npz,frames/}`（**第二次，静止、无目标**）、
`.tmp_robotslot/r11m_default/probe.json`、`.tmp_robotslot/r11m_urdf/probe.json`。

### D.2 数字（车静止）

| 量 | **`robot11`（`plugin` 档，`robot_radius 0.3565`、`inflation 0.70/0.75`）** | **默认模型（`robot_radius 0.22`、`inflation 0.50/0.55`）** |
|---|---|---|
| local：lethal(100) / inscribed(99) / 膨胀(1–98) / free / unknown | **340 / 14652 / 15299 / 32209 / 0** | 445 / 11029 / 16956 / 34070 / 0 |
| global：同上一行 | 313 / 3156 / 781 / 421 / 11940（`map` 正在建） | 417 / 2522 / 2094 / 3407 / 16682 |
| **车所在格的值** | local **84**（非 free）、global **64**（非 free） | local **0（FREE）**、global **0（FREE）** |
| **车半径圆（0.3565 m）内的格** | local **997 格：`≥99`（inscribed 及以上）513 格、free 仅 6 格**；global 158 格：**`≥99` 60 格、free 0 格** | local 996 格：**`≥99` 0 格、free 807 格**；global 159 格：**`≥99` 0 格、free 158 格** |
| **到最近的 `≥99`（inscribed 及以上）格的距离** | local **0.040 m**、global **0.077 m** | local 0.431 m、global 0.692 m |
| **到最近的 `100`（lethal）格的距离（车心）** | local **0.386 m**、global **0.400 m** | local 0.651 m、global 0.904 m |
| **到最近的 `100` 格的距离（雷达）** | local **0.259 m**（**0.25 m 以内 0 格**） | — |
| local lethal 格的距离剖面（从车心起） | **0.35–0.40 m：6 格；0.40–0.45：34；0.45–0.50：14；0.50–0.55：13；0.55–0.60：10；0.60–0.65：14；0.65–0.70：7；0.70–0.75：6** | — |
| local lethal 格的方位剖面（30° 桶，−180…180） | `43, 0, 0, 0, 0, 0, 11, 58, 79, 24, 71, 54` ⇒ 集中在 **0°~180° 的半圈**（车头附近 + 车尾），**不是一圈均匀的环** | — |

**第二次独立复测（`robot:=robot11` 默认档、**静止、不发目标**、无恢复行为清图；`.tmp_robotslot/r11m_plugin2/`）**：
车那格 **88**（非 free）、车半径圆内 **1001 格：`≥99` 435 格、free 仅 24 格**、
到最近 lethal 格 **0.390 m**（与上面 0.386 m 一致）、到最近 `≥99` **0.050 m**、
lethal 到**雷达**的最近距离 **0.259 m**（0.25 m 以内 **0** 格）。
⇒ "车被一个 inscribed 团包住"这件事**两次独立跑都复现**，且**不是**清图后的瞬态。

### D.3 归因（逐条，带数）

先把 `/scan` 的**近场结构**拆开（`plugin` 档细带，11917.5 点/帧那一次跑，中位/帧）：

| `/scan` 水平距离带 | 条/帧 | 对应 `/segmentation/obstacle` 里的 3D 点 | 会不会进代价图 |
|---|---|---|---|
| **0.05–0.10 m** | **66.6** | 水平 <0.10 m 有 **3310 点**：俯角 **−44.5°**（在雷达**上方**）、3D r p50 = **0.05 m**、z p50 = **+0.043** ⇒ **自击团**（打在自身结构上、再被插件"内移 0.1 m"） | **不会**：`obstacle_layer.scan.obstacle_min_range = 0.1` 把 <0.1 m 的波束当"太近"丢掉 |
| 0.10–0.15 m | **0.07** | — | — |
| 0.15–0.20 m | **0.0** | — | — |
| **0.20–0.30 m** | **97.5** | 水平 [0.30,0.50) m 有 **90 点**：俯角 **+32.8°**，而"水平面地面"在该处预测 **+34.1°**；**这 90 点 100% 落在 z = −0.2595±0.03 的地面平面里** ⇒ **它们本来就是地面** | **会**：被 `obstacle_min_range 0.1` 放行；且 `p2l` 是 **2D 投影**（LaserScan 没有高度）⇒ `obstacle_layer` 把它们标在**雷达自己的高度**上 ⇒ 变成 0.26~0.30 m 处的 **lethal 格** |
| 0.30–0.50 m | 103.4 | 同上（近场地面环） | 会 |
| **0.50–1.00 m** | **179.6** | 水平 [0.50,1.0) m 有 **746 点**：俯角 +10.7°（地面预测 +20.5°）⇒ **比地面高 ~0.13 m**，**不是地面**（是场地上 ~0.15 m 高的低矮件） | 会（真障碍，但离车只有 0.5~1.0 m） |
| 1.0–2.0 / 2.0–4.0 / 4.0–7.0 | 239.9 / 121.6 / 62.2 | [1,2) 973 点（比地面高 0.16 m）/[2,4) 534 点（高 0.2 m）⇒ 场地真实结构 | 会 |

**(i) `/scan` 的"自击环"造了多少格？→ 实测：0.25 m（雷达）/ 0.39 m（车心）以内一个 lethal 格都没有。**
* 自击点在**原始云**里是 `r < 0.12 m` 的 **3294 点/帧（28.5%）**，在 `/segmentation/obstacle`
  里也在（**这是物理真实的回波**，Phase 4 有意不动它的标签）。
* `/scan` 每 bin 只留**最近**点、`p2l` 的 `range_min = 0.05` ⇒ 自击团（3D 中位半径 **0.021 m**，
  Phase 4 实测 p95 = 0.048 / max = 0.121）以 **0.05–0.10 m 的波束**出现（66.6 条/帧）
  ⇒ 被 `obstacle_layer` 的 **`obstacle_min_range: 0.1`** 挡掉。
* 实测：**local 图里 `100` 格离雷达最近 0.259 m、0.25 m 以内 0 格**；
  离**车心**最近 0.386/0.390 m（两次跑一致）。
* **车心那格被判"非 free"的算术**：`0.390 − inscribed_radius(0.3565) = 0.034 m ≈` 实测
  "离最近 `≥99` 格 0.040 m"。⇒ 车那格是**被 0.39 m 处的 lethal 格膨胀出来的**，
  **不是** <0.12 m 的自击团（那些点在代价图里根本不存在）。

**(ii) 地面点进了 obstacle 输入吗？→ 进了：`/segmentation/obstacle` 单帧 6087 点里，**
**`|z + 0.2595| < 0.03`（就落在地面平面上）的有 303 点 = 5.0%**；
其中 **222 点集中在水平 0.3–1.0 m**（0.3–0.5 m 那 90 点**全部**在地面平面上、0.5–1.0 m 里 132/746）。
成因（三条叠加，都有数）：
1. linefit 的 `r_min: 0.2` + `max_dist_to_line: 0.05` ⇒ **最近一圈地面**最难被判成地面；
2. 插件 `point = range·axis` 的**系统内移 0.1 m** 把近处地面整体抬高
   （Phase 4 §12.3.1 的独立证据：地面点的 z 随距离单调变化）；
3. `p2l` 把 3D 点**投成 2D**（LaserScan 没有高度）⇒ `obstacle_layer` 把它们标在**雷达高度**上
   ⇒ **"地面"在局部代价图里就是货真价实的 lethal 格**（离雷达 0.26–0.40 m）。
   ⚠️ **global 图这一路是 3D 的**（STVL，`min_obstacle_height 0.0`）⇒ 同样的地面点
   （odom 里 z ≈ −0.07）会被**滤掉**；两张图的 lethal 最近距离因此略有差别
   （0.386 vs 0.400 m）。
   **⇒ 用户"膨胀半径感觉一直把地面当成障碍物算进去"这个感觉，在 local 图这条 2D 链路上是对的**，
   而且它贡献的正是**离车最近的那批 lethal 格**。

**(iii) `robot_radius 0.3565`（vs 默认 0.22）的影响：**
* 它同时是 `inscribed_radius` ⇒ **车那格要不要被判 inscribed，取决于"车心到最近 lethal 格的距离 ≤ 0.3565"**。
  实测 0.386~0.400 m **刚好在边界外一点点**（所以车那格是"高膨胀"而不是 253/99），
  但**车半径圆内 513/996 格 ≥99**、**只有 6 格 free** ⇒ 规划器/控制器在车的**整个足迹**上
  都找不到自由空间。
* 对比默认模型：同样的场地、同样的 `/scan` 语义，`0.22 + 0.5` 下车那格是 **free(0)**、
  圆内 **0/996** 是 `≥99`、**807 格 free**。
* **这是"槽位覆盖层"的直接后果**，不是感知造成的：覆盖层把外接半径 0.3565 与
  `inflation 0.70/0.75` 一起注入（§11.9），而**这台车在出生点本来就在离障碍 0.4~0.75 m 的地方**。

**⇒ 明确回答用户/负责人的问题：**"车是不是一开始就在一个 inscribed 团里？"
**是**（local：车半径圆内 513/996 格 ≥99、free 只有 6；第二次复测 435/1001、free 24；
global：60/158 ≥99、free 0）。
"是不是自击环（<0.12 m 那 28.5%）+ 膨胀造成的？" **不是**（离雷达 0.25 m / 离车心 0.39 m
以内没有任何 lethal 格；自击波束被 `obstacle_min_range 0.1` 挡掉）。
真因是 **两段近场障碍 × 本槽位的膨胀几何**：
① **近场地面残留**（水平 0.26–0.40 m，linefit 没判成地面的那一圈，经 `p2l` 的 2D 投影被标在雷达高度上）
② **场地上 ~0.15 m 高的低矮件**（水平 0.5–1.0 m，真障碍）
配上 **`robot_radius 0.3565`（= inscribed_radius）+ `inflation_radius 0.70/0.75`** ⇒
车心到最近 lethal 只有 **0.386~0.400 m**，仅比 0.3565 大 3~4 cm ⇒ 车那格直接落在
膨胀层的高代价区，整个足迹圆内几乎没有 free 格。

### D.4 "目标被接受但车不动"的**直接**证据（同一次跑，末尾发一次 `/goal_pose`）

| 证据 | 值 | 来源 |
|---|---|---|
| 目标 | 车头前 1.5 m：`(1.464, 0.482)`（map 系） | 探针日志 |
| 规划器 | **出了路径**：`/plan` 31 条，每条 **68~70 个位姿** | `probe.json` |
| 控制器 | **反复拒绝**：`RegulatedPurePursuitController detected collision ahead!`（连续数十条）→ `Controller patience exceeded` → `[follow_path] Aborting handle` | `launch.log` |
| 恢复行为 | `behavior_server: Running spin ... Turning 1.57`；`Received request to clear entirely the local_costmap` | `launch.log` |
| 车真的动了吗 | 真值 `/odom_ground_truth`：**30 s 里只走了 0.18 m**、偏航 +9.9°（自旋） | `probe.json` |
| `/cmd_vel`（nav2 侧） | 620 条、`max(|vx|+|wz|) = 1.97`、488 条非零 —— **都是恢复行为（自旋）发的**，不是循迹 | `probe.json` |

⇒ **"目标被接受但车不走"在本槽位是"控制器判定前方碰撞 ⇒ 拒绝走"**，
与本槽位"车被膨胀层包住"是同一件事。**这一条本轮只测了 1 次、1 个出生点**（§G）。

---

## E. 建议（本主题**不做**调参，只列下一步该看什么）

**给用户（想看"斜放置最原始效果"）：**
1. 用 §C.2 的 `robot11_mount:=urdf` 跑一次带 GUI 的：RViz 固定帧设成 **`map`/`odom`**，
   加 `/livox/lidar/pointcloud` 与 `/cloud_registered` 两个 PointCloud2 display
   ⇒ 会看到**地面变成一个 30° 的斜坡**（这就是"点云表达在斜系/帧斜了"的观感），
   同时 Gazebo 里的雷达 mesh **也是斜的**（`urdf` 档 link 真的转了）。
   （`nav2.rviz` 里没有这两个 display，要看点云得加 display 或用 `pointio.rviz`。）
2. 想"既斜又对"，只有两条路（都不在本主题范围内）：
   (a) 走**真机同款链路**：点云留在传感器系 + `p2l`/`linefit` 用 `target_frame`/
   `gravity_aligned_frame: base_link` 做重力对齐（linefit 那个 C++ bug 已修好，§11.4）；
   (b) 保持 `plugin` 档（今天的行为）—— 帧正、点云正、射线真的斜，代价只是"画的 mesh 是平的"。

**给后续调参（**现在不做**）：**
3. **先把"车不走"解决**：在出生点空旷处，`robot_radius`/`inflation_radius` 的两个候选
   （外接 0.3565 + 0.70，或内切 0.300 + 0.60，或改成 footprint 多边形）各跑一次
   "静止 ⇒ 发一个 1.5 m 目标 ⇒ 看 `/plan` 与真值位移"的固定动作，用**同一份探针**判定。
   本主题提供的 `run_robot11_mount_probe.sh --goal-forward 1.5` 就是这条判据的可复现入口
   （证据字段：`frames.plan`、`cmd_vel.nonzero`、`goal.pre/post`、`launch.log` 里的
   `detected collision ahead`）。
4. **近场那一圈必须分开处理**（三选一，都要单独评估）：
   (a) 修插件 `point = range·axis` → `point = (range + minDist)·axis`（**共享代码，会改所有模型**）；
   (b) 在 `pointcloud_to_laserscan` 侧加自击掩膜（沿用 `self_mask_*` 的同一份几何）；
   (c) 抬高 `obstacle_min_range`（现在 0.1 m 已经在挡自击，但它同时把"贴墙 0.1 m"也挡了）。
5. 若采用 `urdf` 档长期跑，必须同步改：`linefit` 的 `gravity_aligned_frame`（或 `p2l` 的
   `target_frame`）、自击掩膜的盒子表、`lio_tf_adapter` 的杆臂（launch 里已经跟着算了）。

---

## F. 怎么复现

### F.1 工具（本轮新增，都在 `tools/scripts/regress/`）

| 文件 | 作用 |
|---|---|
| `robot11_mount_probe.py` | 逐帧探针：`/livox/lidar`(CustomMsg)/`pointcloud`/`cloud_registered`/`segmentation/{ground,obstacle}`/`scan` 的点数、**RANSAC 平面拟合**、`resid_level` / `resid_tilt` 两种"地面平面"模型下的残差与"低于地面"计数、代价图计数与**以车为中心的距离剖面**、`/plan`、`/cmd_vel`、TF 快照、RTF；点云/扫描可按帧落 CSV，local 栅格可落 `.npz` |
| `run_robot11_mount_probe.sh` | 隔离无头跑 + 运行期参数/TF 回读 + 探针 + 契约清单；**2026-10-08 复核新增**：`--drive`（记录窗内直行/原地转交替，用于 LIO 漂移 A/B）、Gazebo 侧真身读数（`gz topic pose/info` + `gz model -m robot -i`，见 §H.4）、只清**本 master URI** 的 gazebo 进程（按 `/proc/<pid>/environ` 核对，**不做全机 pkill**） |
| `verify_gzmodel_autopath.sh` | 【Objective 1】裸命令（**显式 `unset GAZEBO_MODEL_PATH`**）跑一次 `bringup_sim.launch.py`，落盘 gzserver 真身的 env、`No mesh specified` / 等在线模型库 / spawn 计数、节点集合 —— 见 `docs/gazebo_gui_troubleshooting.md` §5.1.6 |

### F.2 一条命令（每个配置换 tag 即可）

```bash
# (i) robot11 默认档（今天的行为）
tools/scripts/regress/run_robot11_mount_probe.sh r11m_plugin --settle 30 --duration 70 \
  --frames 3 --dump-dir .tmp_robotslot/r11m_plugin/frames \
  --grid-dump .tmp_robotslot/r11m_plugin/local_grid.npz --goal-forward 1.5 --goal-wait 30 -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
# (ii) 默认模型对照
tools/scripts/regress/run_robot11_mount_probe.sh r11m_default --settle 25 --duration 45 \
  --frames 3 --grid-dump .tmp_robotslot/r11m_default/local_grid.npz -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0
# (iii) 物理安装保真档
tools/scripts/regress/run_robot11_mount_probe.sh r11m_urdf --settle 25 --duration 45 \
  --frames 3 --grid-dump .tmp_robotslot/r11m_urdf/local_grid.npz -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
  robot11_mount:=urdf spin_speed:=0.0
# (iv) 2026-10-08 复核用的三条（静止复现 / driven A/B 两档）
tools/scripts/regress/run_robot11_mount_probe.sh r11m_plugin5 --settle 25 --duration 45 \
  --frames 3 --grid-dump .tmp_robotslot/r11m_plugin5/local_grid.npz -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
tools/scripts/regress/run_robot11_mount_probe.sh r11m_plugin6 --settle 25 --duration 45 --drive -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
tools/scripts/regress/run_robot11_mount_probe.sh r11m_urdf4 --settle 25 --duration 45 --drive -- \
  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 robot11_mount:=urdf spin_speed:=0.0
# (v) 【Objective 1】裸命令的 model:// 自动解析（不用前缀任何环境变量）
tools/scripts/regress/verify_gzmodel_autopath.sh obj1_plain --settle 50 -- \
  world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
```

> ⚠️ `--drive` 的 publisher **必须等探针的 settle 之后再注入**（工具已内置 `sleep $SETTLE`）：
> 在"spawn + 插件加载"期间出现 publisher 时，本沙箱里实测 `gzserver` segfault（exit −11）/
> `gzclient` abort（exit −6）+ spawn 服务超时（§H.6）。
> ⚠️ 跑 GUI 之外的档位时建议加 `gui:=False`（Objective 1 的复核对全部用无头）。

### F.3 生成物/开关的静态验证（不用 Gazebo，秒级）

```bash
python3 tools/scripts/regress/robot11_make_sim_xacro.py             # 重生成（生成器拥有 xacro）
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro \
  livox_tilt_rpy:="-0.523598775598293 0 0" | grep -A4 'joint name="body_to_livox"'
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro livox_mount:=urdf \
  livox_tilt_rpy:="-0.523598775598293 0 0" | grep -E 'body_to_livox|tilt_rpy' -A3
python3 tools/scripts/regress/check_robot11_visual_slot.py           # 21 项断言（视觉档位，仍全绿）
```

### F.4 回退

| 想退掉什么 | 怎么做 |
|---|---|
| `urdf` 档 → 今天的行为 | 去掉 `robot11_mount:=urdf`（默认就是 `plugin`） |
| 整个开关 | `git revert <本主题 commit>`（默认档行为**逐字节不变**，见 §C.5） |
| 整套 robot11 | 见 `docs/robot_models.md` §11.7 最后一行 |

---

## G. 未验证 / 诚实清单

1. ~~**`/get_entity_state` 的"Gazebo 眼里的 link 世界姿态"没取到**：本轮加了这条探问，
   但服务返回被我的 `grep` 过滤掉了（三次都是空）。~~ → ★ **2026-10-08 复核已解决，并且原因不是"grep 过滤"**：
   `/get_entity_state` 在本仓 launch 里**不存在**（gzserver 没加载 `libgazebo_ros_api_plugin.so`）；
   改用 `gz model -m robot -i`（Gazebo transport，不经 ROS）拿到了**直接读数**：
   默认档被 lump 的雷达视觉 `…fixed_joint_lump__livox_frame_visual_1`（mesh `l12.stl`）姿态
   **roll 0.000°**，`urdf` 档同一视觉 **roll −30.000°** ⇒ **"画的 mesh 平/斜"不再是推论**。
   原文的替代证据链（TF −30.000° + 插件 `tilt_rpy` 单位阵 + 世界射线仍下俯 30°）仍然成立。详见 **§H.4**。
2. ~~`/scan` 的 0.10–0.30 m 波束为什么没在 local 代价图里留下 lethal 格~~ → **已由第二次跑（静止、不发目标、
   无恢复行为）解决**：`/scan` 在 0.10–0.20 m **几乎没有波束**（0.07 / 0.0 条/帧），
   近场结构其实是两团：**0.05–0.10 m（66.6 条/帧，自击，被 `obstacle_min_range 0.1` 挡掉）**
   与 **0.20–0.30 m（97.5 条/帧，近场地面环，被标成 lethal）**。
   第二次跑里 lethal 离**雷达**最近 **0.259 m**、0.25 m 以内 0 格 ⇒ 与"自击团不造格"一致。
   **仍然未验证的**：`obstacle_layer` 具体把 0.20–0.30 m 那批点标成了哪些格（没有逐格-逐点对齐），
   以及"把 `obstacle_min_range` 调大/调小"会怎样（**本主题不调参**）。
3. **RTF 不可比**：三次跑测得 0.36（plugin）/ 0.69（默认模型，机器最闲时）/ 0.23（urdf），
   **排序不可复现** ⇒ 本主题**不主张**"`urdf` 档更慢"。要做 A/B 必须在同一时间窗、同一
   `/clock` 口径下交替跑。
4. **RViz 的显示设置没有测**：本轮全是无头跑。能引用的是配置文件本身
   （`nav2.rviz` 里**没有** `/livox/lidar/pointcloud` 与 `/cloud_registered` 这两个 display；
   `pointlio.rviz` 有 `/cloud_registered`），**不能**说"RViz 裁了点"。
5. **`lio_tf_adapter` 的 `urdf` 档杆臂只做了"几何 + 单元级"验证**：
   `plugin` 档输出与 Phase 3 逐位相同（`[-0.000561701,-0.130915824,-0.10702817]`/rpy 0）；
   `urdf` 档输出 `[-0.000561701,-0.034862345,-0.151448296]`/rpy `[0.523598776,0,0]`
   与 Phase 2 记录值逐位相同；**且**与运行期 TF（`base_link→imu_link` =
   `[0.000562,0.105916,0.113727]` rpy −30°）的逆一致。但 `lio:=small_point_lio` 时该节点不启动
   ⇒ **没有跑过"`urdf` + `lio:=fastlio|pointlio`"这条组合**。
6. **`urdf` 档的 LIO 漂移只有一个静止窗**（位移差 0.013 m，22 s），**没有做**"直行/偏航两段 + 与
   `plugin` 档同窗口 A/B"。
   → ★ **2026-10-08 复核：driven A/B 做了**（`--drive`：直行 10 s ↔ 原地转 10 s，两档逐字相同的命令）：
   `plugin` **0.0094 m** / `urdf` **0.0483 m**（45 s 窗、真值位移 0.414 m）；加上静止窗（0.0264 / 0.0134 m），
   **两种协议下排序相反 ⇒ 仍在噪声量级内，不主张"哪个档漂得多"**。**仍未做**：多次重复、多速度档、
   与 `lio:=fastlio|pointlio` 的组合（见第 5 项）。详见 **§H.5**。
7. **`/livox/lidar` 的 CustomMsg 与 PointCloud2 只比了"点数"**（两条话题逐帧同数），**没有**逐点
   比内容（`CustomMsg` 的 `offset_time` 全 0、`reflectivity` 未比）。
8. **"18040 条没有回波的射线里，有多少是打天空、多少是打在 0.2 m 以内被 `<min>` 丢掉"没有分离**：
   本轮只能给出总数（30000 − 发出数）。分离需要在插件里加计数（**共享代码，未做**）。
9. **默认模型的绝对地面高度本轮看着和它的 `sensor_height 0.226` 不一致**
   （`/segmentation/ground` 的 z 中位 −0.083，而 `plugin` 档 robot11 是 −0.263 ≈ −0.2595）；
   本轮**没有查**（不属本主题：它涉及 RMUL2026 场地在该出生点附近的地形/平台）。
   §B/§D 里默认模型的数字只用**点数**与**代价图**这两类自洽量，**不**用它的绝对高度。
10. **`robot11_mount` 只支持 `bringup_sim.launch.py`**：
    `hzmi_rm_simulation/rm_simulation.launch.py`（只起 Gazebo + 模型那条）**不传**任何额外 xacro 参数
    ⇒ 它只能跑 `plugin` 档（xacro 默认值）。要单独起 Gazebo 看斜的 mesh，请用
    `bringup_sim.launch.py`（它会顺带起感知节点）。
11. **【复核新增】"注入 0.30 m/s 却只走 0.5 m"没有归因**：`--drive` 的命令是
    `vx=0.30 m/s × 10 s`×2 段（45 s 窗，RTF≈0.42），但真值只有 **0.50 m 路径 / 0.414 m 位移**，
    且 `plugin` 与 `urdf` **完全相同** ⇒ 与"斜装"无关；是底盘/接触/控制器层面的问题，**本轮没查**。
12. **【复核新增】默认模型/其它槽位的 `GAZEBO_MODEL_PATH` 不是逐字节不变**：包侧 export
    （`package.xml` 的 `<gazebo_ros gazebo_model_path="${prefix}/.."/>`）对**所有** gazebo 入口生效
    ⇒ 默认模型跑的 env 也多了 `…/install/robot11/share/robot11/..` 一项（launch 侧那条腿对其它槽位
    仍然是 0 动作、0 日志）。无头跑里量不出副作用（0 错误、节点集合逐个相同），
    有 GUI 时会多几行 `Missing model.config` 噪音。取证见 `docs/gazebo_gui_troubleshooting.md` §5.1.6。

---

## H. 2026-10-08 接手复核：独立复跑 + 四处补测 + 两处更正

> 本节是**第二个人**用**新跑的数据**把 §A–§D 的关键数字重算了一遍，并补上原文标为"未验证"的两项
> （§H.4 的 Gazebo 侧 mesh 姿态、§H.5 的 LIO 漂移）。**结论没有变化**；变化的是三件事：
> ① §G 第 1 项的失败原因被找出来了（不是"grep 过滤"，是**那个服务根本不存在**）；
> ② 默认模型/其它槽位的 env **不是**"逐字节不变"（包 export 那条腿对所有 gazebo 入口都生效）；
> ③ 新增两个可复算的工具与 `--drive` 固定动作。

### H.1 复核跑了哪些（标签 / 协议 / 原始数据）

全部隔离无头（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、
`unset DISPLAY`、**`unset GAZEBO_MODEL_PATH`**），`world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0`：

| tag | 与原文协议的差别 | 用途 | 原始数据 |
|---|---|---|---|
| `r11m_plugin5` | **逐字复刻** §F.2 的 (i')（静止、25 s settle、45 s 窗） | 复现 §A/§B/§D（plugin 档） | `.tmp_robotslot/r11m_plugin5/` |
| `r11m_plugin6` | 同 (i') **+ `--drive`**（本主题新增：直行 10 s ↔ 原地转 10 s 交替） | §H.5 漂移 A/B 的 plugin 侧 | `.tmp_robotslot/r11m_plugin6/` |
| `r11m_urdf4` | `robot11_mount:=urdf` **+ `--drive`** | §H.5 的 urdf 侧 + §C 复现 | `.tmp_robotslot/r11m_urdf4/` |
| `r11m_urdf5` | `robot11_mount:=urdf`，静止，20 s settle / 30 s 窗 | §H.4 的 Gazebo 侧读数 + §C 静止对照 | `.tmp_robotslot/r11m_urdf5/` |
| `obj1_plain` / `obj1_default` / `obj1_light` | 无（Objective 1 的 `model://` 自动解析） | 见 `docs/gazebo_gui_troubleshooting.md` §5.1.6 | `.tmp_verify/obj1_*/` |

### H.2 §A/§B/§D 复现（plugin 档，静止；原文数 → 我的数）

| 量 | 原文（`r11m_plugin` / `r11m_plugin2`） | **我的 `r11m_plugin5`** | 判读 |
|---|---|---|---|
| 原始云最大平面法向 / 平面高 / 与 link z 夹角 | `(-0.01376, 0.0122, 0.99983)` / `−0.26038` / **1.054°** | **逐位相同**：`(-0.01376, 0.0122, 0.99983)` / `−0.26038` / **1.054°** | **帧与点云都正**（§A 的结论）——两次独立跑连尾数都一样（射线图案由 `mid360.csv` 决定 ⇒ 确定性） |
| 自击 `r<0.12` / `r<0.05` / `r>4` / 低于水平面 | 3294（28.5%）/ 1668 / 575 / 1770 | **3294 / 1668 / 575 / 1770** | 逐位相同 |
| `/livox/lidar`(CustomMsg) = `/livox/lidar/pointcloud` 点/帧 | 11960.5 = 11960.5 | **11945 = 11945** | 两条话题同源，**没有一路被额外裁** |
| `/cloud_registered` 点/帧（frame_id=`odom`） | 11968（→ 11953） | **11953** | LIO 不抽稀（`dense_point_deque` 发布） |
| `/segmentation/ground` + `obstacle` | 5725 + 6227 | **5712 + 6223.5**（和 11935.5 ≈ 原始 11945，差 10 = 帧边界） | 分割**不丢点** |
| `/scan` 有限波束（最近的带） | 829（0.20–0.30 m = 97.5，0.05–0.10 m = 66.6） | **829**（0.20–0.30 = **100.6**，0.05–0.10 = **38.7**） | 近场两团结构复现（自击团 + **近场地面环**） |
| `/cloud_registered` 里低于水平面的点 | 0 / 11620 | **0 / 11620** | plugin 档地面在 odom 里**水平** |
| local costmap lethal / inscribed / 车那格 | 340 / 14652 / **84**（第二次 423 / 17785 / **88**） | **462 / 15087 / 86** | 同量级、**车那格仍然非 free** |
| 车半径圆（0.3565 m）内格数 / `≥99` / free | 997 / **513** / 6（第二次 1001 / **435** / 24） | **1000 / 453 / 19** | **"车被 inscribed 团包住"三次独立复现** |
| 到最近 lethal 格的距离（车心） | **0.386** / 0.390 m | **0.379 m** | 三次都在 0.379~0.390 m |
| 到最近 lethal 格的距离（雷达） | 0.259 m | 0.259 m | <0.12 m 的自击团**没有**进代价图 |

**⇒ §B 的"点云没有被任何一级裁掉"、§D 的"车一开始就在一个 inscribed/膨胀团里（不是自击环造成的）"
两条结论，用新数据全部复现。**

### H.3 §C 复现（`robot11_mount:=urdf`）

| 量 | 原文（`r11m_urdf`，静止） | **我的 `r11m_urdf4`（driven）/ `r11m_urdf5`（静止）** | 判读 |
|---|---|---|---|
| TF `base_link→livox_frame` rpy | −30.000/0/0 | **−30.000/0/0**（两次都测了） | 帧真的斜 |
| 原始云（11568 点）平面夹角 / 自击 / 低于水平面 | 1.054° / 3294 / 1770 | **1.054° / 3294 / 1770** | **与 plugin 档逐位相同** ⇒ 倾角记在关节还是插件上，**对点云坐标没有影响**（§A.3 复现） |
| `/cloud_registered` 地面夹角 | **30.716°** | **30.618°**（driven）/ **30.539°**（静止） | 地面在 odom 里**是 30° 斜面** |
| `/cloud_registered` 低于水平面的点 | 7506 / 11622 = **64.6%** | **7511 / 11622 = 64.6%**（driven）、7507/11622（静止） | 复现 |
| local lethal / inscribed | 75 / 3844 | **75 / 4658**（静止 `urdf5`） | **urdf 档代价图"更空"复现**（lethal 从 plugin 的 462 掉到 75） |
| 车半径圆内 `≥99` / free | 39 / 445 | **36 / 454**（静止 `urdf5`） | 复现：urdf 档车周围反而更自由（原因见 §C.4 第 2 条：高度带丢了一半波束） |
| `/scan` 带（`urdf4`） | 0.05–0.10 = 57.1、0.20–0.30 = 101.2 | **0.05–0.10 = 150.1、0.20–0.30 = 186.6**（driven；与静止跑不可比，见下） | ⚠️ 带分布**随是否在动而变**（driven 窗里点更多、近场更多）⇒ 不要把 driven 跑的带分布与原文静止跑并排读 |

### H.4 **Gazebo 自己眼里**的雷达 mesh 姿态：两档都拿到了直接读数（关闭 §G 第 1 项）

**先更正失败原因**：`/get_entity_state`（`gazebo_msgs/srv/GetEntityState`）在本仓的 launch 里
**根本不存在**，不是"输出被 grep 过滤掉了"：
* `ros2 service list` 里 **0** 个 `/get_entity_state`；`ros2 service call` 只会 `waiting for service`；
* 原因：本仓 gzserver 的 cmdline 只加载了 `libgazebo_ros_{init,factory,force_system}.so`
  （见 `gz_env.txt`/`launch.log`），**没有** `libgazebo_ros_api_plugin.so` —— 那个服务是后者提供的。

**改用两条不依赖 ROS 的读数**（都在同一次跑里、Gazebo 活着的时候取）：

| 读数 | `plugin`（默认，轻量入口） | `urdf`（`r11m_urdf5`） |
|---|---|---|
| `gz topic -l` 里的激光话题名 | `/gazebo/default/robot/base_link/livox_frame/scan` | 同左 |
| `gz model -m robot -i` 里的视觉名 | `robot::base_link::**base_link_fixed_joint_lump__livox_frame_visual_1**` | 同左（名字里直接写着 **fixed_joint_lump**） |
| 该视觉的 mesh | `model://robot11/meshes/decimated/l12.stl` | 同左 |
| 该视觉的位置 | `(0.000562, 0.130916, 0.157028)` | 同左 |
| 该视觉的姿态（四元数 xyzw） | `(0, 0, 0, 1)` ⇒ **roll 0.000°** | `(-0.25881915, 0, 0, 0.96592580)` ⇒ **roll −30.000°** |

三条结论：
1. **"画出来的雷达 mesh 平/斜"从推论升级为直接读数**：默认档 mesh **正**（用户看到的"URDF/雷达是平放的"
   就是这个），`urdf` 档 mesh **斜 −30.000°**（= 上游/CSV 的物理安装姿态）。
2. **`livox_frame` 不在 `gz topic pose/info` 的 link 列表里**，因为 URDF 的**固定关节在 spawn 时被
   gazebo lump** 成"挂在 base_link 上的视觉"（`fixed_joint_lump__livox_frame_visual_1`）——
   所以"用 pose/info 读 livox_frame"这条路走不通；**能读的是被 lump 后的那个视觉**（上表）。
3. 这也解释了为什么 TF/URDF 里有 `livox_frame`（它由 `robot_state_publisher` 从 URDF 直接发），
   而 Gazebo 侧只有 lump 后的视觉 ⇒ **两边的"雷达在哪"从来不是同一个对象**，看 mesh 姿态要用上表。

### H.5 LIO 漂移 A/B（`--drive` 固定动作，同一协议）——§G 第 6 项部分关闭

`--drive`（本次新增到 `run_robot11_mount_probe.sh`）= 记录窗内**直行 10 s（vx=0.30）↔ 原地转 10 s（wz=0.60）**
交替，注入点 `/cmd_vel_chassis`（`fake_vel_transform` 是事件驱动转发，不会持续发零覆盖它）。
漂移口径仍是本仓惯例"**odom 位移 − 真值位移**"：

| 协议 | `plugin`（默认档） | `urdf` 档 | 判读 |
|---|---|---|---|
| 静止 45 s（原文 / 我的 `plugin5` / 我的 `urdf5`） | 0.024 m / **0.0264 m** / — | 0.013 m / — / **0.0134 m** | 静止窗里两者都是**厘米级噪声** |
| **driven 45 s**（`plugin6` / `urdf4`） | **0.0094 m**（yaw −0.54°） | **0.0483 m**（yaw −0.47°） | 排序**与静止窗相反** ⇒ **在噪声量级内，不能主张"哪个档漂得多"** |
| 同窗真值位移 / 路径 | 0.414 / 0.502 m | 0.413 / 0.509 m | 两次注入的命令**完全相同**（保证 A/B 公平） |
| `/odom` 位移 / 路径 | 0.405 / **0.906 m** | 0.421 / **1.398 m** | ⚠️ 单跑、且 RTF 不同（0.419 vs 0.427）⇒ **不作为结论**，只登记 |
| RTF（`/clock` 口径） | 0.419 | 0.427 | 同一时间窗内接近 ⇒ 不能再拿"RTF"说 urdf 更慢（与 §G 第 3 项一致） |

> **顺带量到一件与安装档无关的事**：注入 `vx=0.30 m/s`（10 s 一段、两段）时，真值只走了
> **0.50 m 路径**（RTF≈0.42 ⇒ 45 s 墙钟 ≈ 19 s 仿真时间，其中约 6 s 是直行命令）。
> 两个档**完全一样** ⇒ 这不是"斜装"造成的，而是底盘/接触/控制器层面的问题；
> **本轮没有归因**（超出本主题，登记在 §G 第 11 项）。

### H.6 复跑踩到的两个操作级坑（写下来免得别人再踩）

1. **在"spawn + 插件加载"期间出现 ROS publisher ⇒ 本沙箱里 gazebo 会崩**：
   我最初把 `--drive` 的 publisher 放在 launch 后 3 s 启动，结果
   ① `gzclient` abort（`exit code -6`）+ spawn 服务超时（实体"pushed to spawn queue"但没在 60 s 内出现）；
   ② 另一跑 `gzserver` 在 `mecanum_controller` 插件加载处 **segfault（exit -11）**，
   此后 TF 里没有 `odom`、探针收不到任何点云。
   **规避**：publisher 必须等"机器人稳定"之后再注入（本工具改成 `sleep $SETTLE` 后再发），
   之后 3 次跑（`plugin6`/`urdf4`/`urdf5`）全部正常。
2. **`gz topic -e -d 5 <topic>` 是非法参数**（gz 会直接打印 `Invalid arguments`）：
   必须 `gz topic -e <topic> -d 5`（`-e` 后面紧跟话题名）。

---

## 附：与 `docs/robot_models.md` 的关系

* 本文是 **Phase 3/4/5 那条"帧正"路线（§11.5）的对照与补证**：§11.5 选了"点云在源头对齐"，
  本文用**平面拟合**把它证明了一遍（§A.2），并把它与"物理安装保真"这条路（`urdf` 档）
  的**代价**量了出来（§C.4）。
* `docs/robot_models.md` §11.3 的 provenance 表里第 3 项（`body_to_livox` 的 rpy 改成 `0 0 0`）
  现在**仍然成立** —— 只是多了"它也**可以**按需切回上游字面值"这个开关（默认不变）。
* §13.9 记的"`/scan` 里恒有一圈自身障碍"在本文 §D.3 里有了归因：**它确实在 `/scan` 里，
  但它不是把车包住的那一圈**（0.386 m 内没有 lethal 格）。

---

## I. 2026-10-09 追查：`urdf` 档"物理上到底对不对" + 用户在 RViz 里看到的到底是什么

> 触发 = 用户 2026-10-09 的原话（逐字）：
> 「车有一侧应该是右侧那个向点云是出不去的…可是…点云看着像平放扫到的东西倾斜了，
> 不是倾斜放置扫描到的东西」
>
> 本节做四件事：**(1)** 用**判决性实测**把三个假设（H1 外参缺一条 / H2 只斜了 label、射线还是平的 /
> H3 一侧盲区是 roll 的应有几何）各判一次；**(2)** 说清"点云到底表达在哪个系、盖的哪个 `frame_id`"
> （源码行号 + Gazebo 侧 SDF 静态证据）；**(3)** 给出 RViz 的 `Fixed Frame` 配方与"为什么看起来是
> 平放被倾斜"；**(4)** 给出"斜装该怎么正确处理"的结论与**精确改法表**（本主题**仍然不改任何
> 默认值/感知参数**，只加文档 + 新工具目录）。
>
> 全部**无头隔离跑**（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、
> `unset DISPLAY`、只按 `/proc/<pid>/environ` 清本 master URI 的 gazebo），
> 原始数据 `.tmp_tiltmount/{tm_plugin,tm_urdf}/`，工具 `tools/scripts/tiltmount/`。

### I.0 一句话判决

**`urdf` 档的"物理"是对的，"账"是错的 —— 而且错在三处（其中两处是本轮新挖出来的 LIO 侧 bug）。**

| # | 判决 | 判决性数字 |
|---|---|---|
| **物理（射线/mesh/IMU 安装姿态）** | **对**：射线在世界里真的下俯 30°（地面最近环 **0.3146 m** ⇒ 下俯 **38.27°**，而水平安装的物理下限只有 **7.22°** ⇒ 最近地面环不可能近于 1.96 m）；画的 mesh 也真的斜 −30.000°；IMU 真的斜 29.979° | §I.4 / §I.5 |
| **点云数据** | **与 `plugin` 档逐点相同**（刚体配准 **0.0004°**、残差 p50 **0.074 mm**）⇒ 倾角**没有进数据**，数据仍在**水平**系里 | §I.4 |
| **账（帧）** | **不自洽 30°**：`header.frame_id` 写 `livox_frame`，而这个帧在 `urdf` 档真的斜了 30°，数据却是水平的 ⇒ 任何"用 TF 变换这朵云"的消费者都会把**整个场景刚性转 30°**（`/cloud_registered` 实测 **30.970°**；把它按 `T(base_link←livox_frame)` 反变换回去只剩 **1.041°**） | §I.5 |
| **用户看到的** | **字面正确**："像平放扫到的东西被倾斜了"——`/cloud_registered` 就是"水平的 odom 点云被刚性旋转 −30°"，不是"斜着装扫出来的覆盖图案" | §I.5 / §I.6 |

⇒ **定论：`urdf` 档 = 物理正确 + 帧/label 不一致（bug ①）+ 两个只在"TF 带旋转"时才现形的 LIO 侧 bug（②③）。**
用户"看着不对"的直觉是**对的**，而且他能一眼看出来，正是因为那是一个**刚性旋转**（地面变成 30° 斜坡、
环状图案没变），而不是一个**不同的覆盖图案**（那才会表现为"一侧彻底看不见地面/最近地面环半径从
0.31 m 变到 1.96 m"）。

### I.1 假设判决表（每条假设 → 判它的那一次测量 → 数字 → 判决）

| 假设 | 判决性测量（怎么做） | 实测 | 判决 |
|---|---|---|---|
| **H1**：`urdf` 档只斜了帧，而 **LiDAR→IMU 外参仍是单位阵**（LIO 假设雷达是平的）⇒ LIO 重建出来的 **odom 帧被转了 30°** ⇒ 地面在 odom 里是斜面 | ① 读**实际生效**的 LIO 配置（`extrinsic_T/extrinsic_R`）并与 TF 里 `imu_link←livox_frame` 的**真实几何**比；② 量 IMU 自己的重力方向（`/livox/imu` 窗口均值）；③ 把 `/cloud_registered` 按 `T(base_link←livox_frame)` **反变换**回去再拟合地面（若"odom 帧本来就斜"，反变换后会变成 60° 而不是水平）；④ 从发布的 `odom→base_link` **反解 LIO 自己的状态姿态** `R_ol` | ① `extrinsic_T=[0,0,0.05]`/`extrinsic_R=I`；TF 实测 `imu_link←livox_frame` = xyz **(0, 0, 0.05)**、rpy **0** ⇒ **两档都仍然精确成立**（IMU 与雷达同壳同斜）；② IMU 量到的重力与 imu z 夹角：`plugin` **0.017°** / `urdf` **29.979°**（搬到 base_link 后两档都是 **0.017/0.021°**）；③ 反变换后地面 **1.041°**（正变换前 30.970°）；④ 反解 `R_ol` = `plugin` **[0.217, 0.047, 9.667]°** / `urdf` **[0.086, 0.405, 9.105]°**（都是"只有 yaw"的水平姿态） | **不成立**（机制不成立："缺外参"不是原因；odom 帧**没有**被转 30°，它水平到 0.1~0.4°） |
| **H2**：插件**仍打水平射线**，只有发布出去的 `frame_id` 被斜了（真 bug）⇒ 世界里的场景被转了 | ① 把两档的**传感器系点云**（`/livox/lidar/pointcloud`，`frame_id=livox_frame`）做**刚体配准**（最近邻 + Kabsch）：若"帧真的斜了"的正确实现，应当给出 **R⁻¹ ≈ 30° 绕 x**；② 无帧假设地量**世界射线**：地面最近环半径 + 传感器离地高 ⇒ 反推最陡下俯角（水平安装的物理下限是 7.22°） | ① 配准 **11652 对**：角度 **0.0004°**、平移 **( −2.3e−5, −6.8e−5, 4.4e−6 ) m**、残差 p50 **0.074 mm** / p95 0.50 mm / max 5.3 mm ⇒ **两档点云逐点相同**；② 地面平面高 **0.2482 m**、最近地面点水平半径 **0.3146 m** ⇒ 下俯 **38.27°**（水平安装时该半径不可能 < **1.96 m**） | **前半成立 / 后半不成立**："只有 label 斜了、数据是水平的"**成立**；"射线是平的"**不成立**（射线两档都真的斜 30°） |
| **H3**：用户看到的"右侧出不去"是 **roll 倾角应有的几何**（左右不对称），不是 bug | 逐 30° 扇区的点数 / 地面点数 / **仰角包络**（实测）与**锥形视场旋转 30° 后的解析下界**对比；四象限点数；最近 200 个地面点的**方位直方图**；以及代价图 lethal 格的方位分布 | 右侧两扇区（−120..−60）**3586 点里 0 个地面点**、最低仰角 **+21.89°/+19.67°**（解析下界 +19.11°）；左侧（60..120）**1642+1634 点里 2694 个地面点**、最低仰角 **−36.87°**（解析下界 −37.22°）；最近 200 个地面点方位 p50 = **89.6°**（全在 60..120 这个 ±30° 楔形里）；代价图 lethal 格方位：`front 127 / left 224 / back 120 / **right 0**` | **成立**（**两档都是这样** —— 与 `robot11_mount` 开关无关） |

### I.2 三个"坐标系"的账：点云到底表达在哪儿、盖的哪个 frame（源码 + Gazebo 侧静态证据）

这一节是本次追查的**根**：本仓有**三个不同的东西**被同一个名字 `livox_frame` 混在一起。

| 东西 | 是什么 | 证据 |
|---|---|---|
| ① **物理传感器系**（实物） | 斜 30°（`body_to_livox` 的 rpy） | `urdf` 档 TF `base_link→livox_frame` rpy **−30.000/0/0**；Gazebo 侧被 lump 的视觉 `…fixed_joint_lump__livox_frame_visual_1` 朝向 x=**−0.25881915**（= sin(−15°)，roll **−30.000°**） |
| ② **点云数据实际表达的系** | **原点 = 传感器原点，朝向 = 父 link（`base_link`）的朝向** ⇒ `plugin` 档"水平 + 传感器居中"，`urdf` 档**仍是水平** | 插件：`laserCollision = physics->CreateCollision("multiray", _parent->ParentName())`（**父 link**）、`SetRelativePose(_parent->Pose())`（**传感器相对父 link 的位姿**）——`livox_points_plugin.cpp:111-114`；出点 `axis = sensor_rot * mount_rot_ * ray`、`point = range*axis`（`sensor_rot = laserCollision->RelativePose().Rot()`）——`:258-260`、`:276-277`。`plugin` 档 `mount_rot=R_x(−30°)` ⇒ `I·R_x(−30°)`；`urdf` 档 `sensor_rot=R_x(−30°)`、`mount_rot=I` ⇒ `R_x(−30°)·I` ⇒ **乘出来同一个矩阵**，点云逐点相同（§I.4 实测 0.0004°） |
| ③ **`header.frame_id`** | = `<sensor name>`，本仓被**故意**设成字符串 `livox_frame`（`livox_points_plugin.cpp:212,218` 用 `raySensor->Name()`；`sentry_robot_robot11_sim.xacro:1010-1012` 的 `<sensor type="ray" name="livox_frame">`，设计意图写在 `:1006-1007`） | `/livox/lidar/pointcloud`、`/livox/lidar`(CustomMsg)、`/segmentation/*`、`/scan` 的 `frame_id` 实测都是 `livox_frame` |

**② 与 ③ 只有"关节 rpy = 0"时才相等** —— 这正是 `plugin` 档的自洽条件。把倾角搬进关节（`urdf` 档）之后：

* 数据（②）还是水平的；
* `frame_id`（③）指向的那个 TF 帧（①）斜了 30°；⇒ **"帧与数据差 30°"**，这就是全部问题的来源。

**Gazebo 侧的静态证据（不进仿真也能复算）**：`gz sdf -p` 把渲染出来的 URDF 转成 SDF 后，
**传感器是挂在 `base_link` 下的**，而且它的 `<pose>` 已经把固定关节的旋转折进去了：

```
# plugin 档                                  # urdf 档
<link name='base_link'>                       <link name='base_link'>
  <sensor name='livox_frame' type='ray'>        <sensor name='livox_frame' type='ray'>
    <pose>0.000562 0.130916 0.157028            <pose>0.000562 0.130916 0.157028
          0 -0 0</pose>                                 -0.523599 0 0</pose>
  <sensor name='mid360_imu' type='imu'>         <sensor name='mid360_imu' type='imu'>
    <pose>0.000562 0.130916 0.107028 …</pose>     <pose>0.000562 0.105916 0.113727 -0.523599 0 0</pose>
```

⇒ 插件里的 `laserCollision->RelativePose()`（= `sensor_rot`）在 `urdf` 档**就是** `R_x(−30°)`；
⇒ 这也解释了为什么 `/gazebo/default/robot/base_link/livox_frame/scan` 这个 scoped name 里
`base_link` 在 `livox_frame` 前面（固定关节被 lump），以及为什么 `gz topic pose/info` 里找不到 `livox_frame`（§H.4）。

**"画的 mesh 斜不斜"与"射线斜不斜"对不对得上？** 对得上，而且是同一个原因：
mesh 与射线**都**只经过那一个关节旋转 —— mesh 的 Gazebo 侧直接读数是 roll **−30.000°**（本轮 `gz model -i`
复现，`x=-0.25881915348021844`），射线（两档逐点相同的那朵云）在世界里下俯 **38.27°**（§I.4）。
`plugin` 档相反：mesh **roll 0**（`x=0`）而射线照样斜 30° ⇒ 那是"画错了、物理对了"。

### I.3 H3：一侧盲区 = roll 的应有几何（逐扇区数字）

`robot:=robot11` 静止；下表 `A / B` = `plugin` 档 / `urdf` 档 —— **两档逐格相同**（只在帧边界上差个位数点）。方位定义 = REP-103（x 前、y 左），在原云自己的坐标里量（该系水平，见 §I.4）：

| 扇区（°） | 点数 | 其中地面点 | 最低仰角实测 | **解析 FOV 下界** | 地面点最近水平半径 (m) |
|---|---|---|---|---|---|
| −180..−150 | 205 | 0 | −4.51 | −8.34 | — |
| −150..−120 | 219 / 220 | 0 | +61.36 | +8.09 | — |
| **−120..−90** | **1304** | **0** | **+21.89** | +19.11 | — |
| **−90..−60** | **1626** | **0** | **+19.67** | +19.11 | — |
| −60..−30 | 232 | 0 | +21.08 | +8.09 | — |
| −30..0 | 115 | 8 / 9 | −4.84 | −8.34 | 5.600 |
| 0..30 | 867 | 51 | −22.44 | −24.12 | 0.624 |
| 30..60 | 1416 | 713 / 716 | −33.12 | −34.02 | 0.370 |
| **60..90** | **1642** | **1318 / 1319** | **−36.87** | −37.22 | **0.315** |
| **90..120** | **1634** | **1376 / 1378** | **−36.87** | −37.22 | **0.315** |
| 120..150 | 1461 | 1040 / 1042 | −33.10 | −34.02 | 0.370 |
| 150..180 | 931 / 932 | 400 / 401 | −22.35 | −24.12 | 0.627 |

* "解析 FOV 下界" = 把 MID-360 的锥形视场（垂直 **−7.22°…+55.22°**，`mid360.xacro` 的取值，
  `sentry_robot_robot11_sim.xacro:1028-1029`）按 **roll −30°** 旋转后，逐方位的**理论最低可见仰角**
  （`tools/scripts/tiltmount/tilt_mount_compare.py` 里的 `analytic_lower_envelope()`，可复算）。
  实测最低仰角**处处不低于它、且在被地面/低矮件填满的方向上几乎贴着它**（−36.87 vs −37.22）⇒
  **盲区的边界就是视场边界**，不是"下游裁掉了点"。
* **四象限点数（点 / 其中地面点）**：前 2398 / 776，**左 4737 / 3735**，后 931 / 400，
  **右 3586 / 0** ⇒ **整个右半边一个地面点都没有**。
* **最近 200 个地面点的方位直方图**：60..90 = **101**、90..120 = **97**、30..60 = 1、120..150 = 1，
  p50 = **89.6°** ⇒ 地面只在一个**以 +y（左）为中心 ±30° 的楔形**里可见。
* **仰角谱**（原云，自己的系）：p0 **−36.87°**、p1 −34.14、p5 −29.72、p50 −5.56、p95 +67.57、p100 +82.16
  ⇒ 与"−7.22°…+55.22° 的视场绕 x 转 −30°"完全一致（下界 −37.22°、上界 +85.22°）。
* **`/scan` 的有限波束**（`p2l` 输出，`frame_id=livox_frame`）：949（`plugin`）/ 948（`urdf`）条 ——
  **几乎不变**；四象限：前 261、左 339 / 338、后 121、**右 228**。注意"右侧 228 条"是**近场自击团**
  （r ≈ 0.05 m，扇区 −120..−30），**不是地面**：同一张表里右半边的地面波束是 0。

⇒ **H3 成立，而且它是 roll 的"应有几何"，不是 bug**：绕 x 轴 roll −30° 后，传感器"看不见的锥"
（自身下视只到 −7.22°）被转到**右下方** ⇒ 右半边从"地面"到"水平线"整段都进不了视场，
而左半边能一直看到 −37.2°（地面最近环 0.31 m）。**用户说的"右侧那个方向点云出不去"就是这个**
（再叠上 §I.7 里代价图"右侧 lethal 格 = 0"的后果）。
判据留在 §I.6：**"覆盖图案真的变了"（一侧没有地面）与"场景被刚性转过"是两件不同的事**，
而 `urdf` 档在 RViz 里同时具备这两件事（前者是物理、后者是 bug）。

### I.4 H2：两档的**传感器系点云**逐点相同（本地云刚体配准）

`.tmp_tiltmount/tm_plugin` 与 `.tmp_tiltmount/tm_urdf` 各取一帧 `/livox/lidar/pointcloud`（11652 / 11654 点）：

| 口径 | 结果 |
|---|---|
| 最近邻 + Kabsch 刚体配准（11652 对，迭代 3 轮，门限 0.25 m） | 角度 **0.0004°**、轴 `(0.459, −0.248, −0.853)`（无意义，角太小）、平移 **(−2.3e−5, −6.8e−5, 4.4e−6) m** |
| 配准残差 | p50 **7.4e−5 m（0.074 mm）**、p95 **5.0e−4 m**、max **5.3 mm** |
| 原始云 RANSAC 地面平面 | `plugin`：n=(−0.00853, 0.01592, 0.99984)、与自身 z 夹角 **1.035°**、平面高 **0.2482 m**；`urdf`：夹角 **1.023°**、平面高 **0.2482 m** |
| 地面最近环 / 最陡下俯 | `plugin` rxy_min **0.3146 m** ⇒ **38.26°**；`urdf` **0.3146 m** ⇒ **38.27°** |
| 自击掩膜命中（`r_xy ≤ 0.2416 且 z ≥ −0.2295`） | `plugin` **3402**（29.197%）/ `urdf` **3403**（29.200%） |
| `p2l` 高度带（`|z| ≤ 1.0`，点云自带帧）内的点占比 | 两档都是 **1.0000** |

**⇒ 两个结论：**
1. **"斜的只有帧/label、数据没斜"= 成立**：两档点云坐标逐点相同到 0.07 mm（这是配准残差，不是「看起来差不多」）。
   **这就是用户在 RViz 里看出"像平放扫到的东西被倾斜了"的物理根源** —— 数据真的是"平放"的。
2. **"插件还在打水平射线"= 不成立**：世界射线在两档里是同一条（点云逐点相同 ⇒ 打的是同一批世界点），
   而它们**真的下俯 30°**：地面最近环 0.3146 m ÷ 离地 0.2482 m ⇒ **38.27°**，
   水平安装时（下视只到 7.22°）这个半径**不可能小于 0.2482/tan(7.22°) = 1.96 m**。
   38.27° − 7.22° ≈ 31° ⇒ 射线物理上就斜了 ~30°。

### I.5 H1：LIO 的 odom 帧**没有**被转 30°；30° 是"发布环节多转的一次"（外加两个新 bug）

#### I.5.1 关键测量：把 `/cloud_registered` 反变换回去

`small_point_lio` 的 `/cloud_registered` 是这么来的（**两处变换**）：

1. `small_point_lio.cpp:92-98`：`p_odom = R_ol · (extrinsic_R·p_lidar + extrinsic_T) + t_ol` —— **已经是 odom 系**，
   且 `extrinsic_R=I` ⇒ 就是"原云 + (0,0,0.05)"（`config/mid360_sim_tuned.yaml` 的 `extrinsic_T`）。
2. `small_point_lio_node.cpp:141,159,191`：又做了一次
   `lookupTransform("base_link", lidar_frame)`，然后 `transformed_point = R(base←livox)·point + t(base←livox)`，
   最后 `msg.header.frame_id = "odom"` —— **对"已经在 odom 里的点"再乘一次 `T(base_link←livox_frame)`，还盖 odom 的标签**。

所以预测是：**反变换掉那次多余的 `T(base_link←livox_frame)` 之后，`/cloud_registered` 的地面应当回到水平**。
实测（同一帧、用同一份 TF）：

| 档 | 发布的 `/cloud_registered` 地面倾角 | 按 `T(base_link←livox_frame)` 反变换后 | 该 TF 的旋转 |
|---|---|---|---|
| `plugin` | **0.742°** | 0.742°（TF 旋转 = 0°，反变换是恒等） | rpy `[0,0,0]` |
| `urdf` | **30.970°** | **1.041°** ← **回到水平** | rpy `[−30.000, 0, 0]` |

**同一条判据的两个独立口径也都指向"多余的那一次旋转"：**

* 逐点残差（最近邻配对）：`urdf` 档 **H_a**（`p_pub ≈ R(base←livox)·p_raw + t`）残差 p50 **2.67 cm**、
  82.4% 的点在 5 cm 内；**H_b**（不转，`p_pub ≈ p_raw + t`）p50 **11.47 cm**、只有 10.2% 在 5 cm 内
  ⇒ **H_a 赢 4.3 倍**（残差底噪来自"取的两帧不是同一时刻"+ 车的轻微晃动）。
* 地面法向分量：`R(base←livox)·n_raw` 与实测发布的地面法向夹角 **0.088°**（`urdf`）/ 0.297°（`plugin`）
  ⇒ 不是"角度差不多"，是**同一个刚体变换**。

#### I.5.2 odom 帧本身：水平（0.2~0.4°）

| 判据 | `plugin` | `urdf` |
|---|---|---|
| **反解 LIO 自己的状态姿态 `R_ol`**（从发布的 `odom→base_link` 按代码语义 `R_ob = R_bl·R_ol·R_blᵀ` 反解；tf2 的合成顺序用一个 30 行 C++ 单测钉住了，见 §I.9） | **[0.217, 0.047, 9.667]°**（只有 yaw） | **[0.086, 0.405, 9.105]°**（只有 yaw） |
| odom 里的地面（把原云用**LIO 自己的 TF** `odom←livox_frame` 搬过去再拟合） | **0.867°** | 30.798°（**这一格是 label 的旋转**，不能当 odom 的倾斜读，见下） |
| odom 帧相对**真实重力**的倾角（`TF(odom←base_link)` × 真值 `/odom_ground_truth`） | **0.203°** | **5.248°** ← 被 bug ③ 污染（见 §I.5.3），修正后 < 0.5° |
| 真值里车自己的姿态（`/odom_ground_truth`） | roll 0.016 / pitch −0.003 / yaw 0→10.152° | roll 0.018 / pitch −0.003 / yaw 0→10.019° |
| `/odom` 窗口内 roll/pitch 跨度 | 0.161° / 0.144° | 0.421° / **4.924°**（← 与 yaw 一起长大，= bug ③） |

> 顺带查清一件**与安装档无关**的事：`RMUL2026` 出生后车会自己转 ~10°
> （真值 `/odom_ground_truth` 的 yaw **0.000° → 10.152°**，`/odom` **0.473° → 10.199°**，两档都复现）
> ⇒ 这就是 §I.5.2 里 `R_ol` 那个 ~9° yaw 的来源，**不是 LIO 漂移**（roll/pitch 跨度只有 0.16°）。

**⇒ H1 的机制不成立**：odom 帧是重力对齐的（0.2~1.0° 量级），30° 是在**发布那一刻**被乘上去的。
**"缺一条外参"也不是原因**：`imu_link←livox_frame` 的真实几何在两档里都是 `(0,0,0.05)`/rpy 0，
和 `mid360_sim_tuned.yaml` 的 `extrinsic_T=[0,0,0.05]`、`extrinsic_R=I` **逐位一致**（IMU 与雷达同壳、一起斜）。
真正的错在"点云不在它自称的那个帧里"（§I.2）＋下面两个 LIO 侧 bug。

#### I.5.3 顺带挖出来的两个 LIO 侧 bug（`plugin` 档不可见，`urdf` 档才现形）

| bug | 位置 | 为什么 `plugin` 档看不见 | `urdf` 档实测后果 |
|---|---|---|---|
| **② `/cloud_registered` 多转一次** | `small_point_lio_node.cpp:141`（`lookupTransform("base_link", lidar_frame)`）+ `:191`（`R*point + T`）+ `:159`（`frame_id="odom"`） | `T(base_link←livox_frame)` 是**纯平移**（rpy 0）⇒ 只把整朵云平移了 ~0.20 m（(0.00056, 0.13092, 0.15703)，即 y/z 各 0.13/0.16 m）⇒ 看不出转 | 整个 odom 场景被**刚性旋转 −30°**（地面 30.970°）；而且旋转是**绕 odom 原点**做的（`R*p+T`）⇒ 车一旦开出原点，整朵云会绕原点甩（§I.10 未验证项 3） |
| **③ `odom→base_link` 的姿态是"共轭"不是"合成"** | `small_point_lio_node.cpp:99`（`lookupTransform(lidar_frame, "base_link")`）+ `:109`：`T_ob = T_bl⁻¹ · T_ol · T_bl` | `T_bl` 是纯平移 ⇒ 共轭 = 只挪旋转中心（本来就是想要的），姿态不受影响 | 发布出来的 `odom→base_link` rpy = **[0.383, 4.890, 7.701]°**，而按代码语义反解出的 LIO 状态 `R_ol` = **[0.086, 0.405, 9.105]°**（水平）⇒ **一个随 yaw 长大的假俯仰（4.89°）**；正演 `R_bl·R_ol·R_blᵀ` 与实测 rpy **逐位相同**（0.383/4.890/7.701）⇒ 机理钉死。**在「点云其实表达在水平系」的现状下**，`odom→base_link` 的物理正确值就是 `R_ol`（水平、只有 yaw）；一般写法应是**合成** `T_ob = T_ol · T_bl⁻¹`（而不是相似变换/共轭） |

> ③ 还解释了 §I.5.2 表里"odom 相对真实重力 5.248°"这一格：那不是 odom 斜，是**发布出来的车身姿态**斜。
> nav2 用的是这条 TF ⇒ **`urdf` 档下 nav2 以为车在 odom 里俯仰 4.9°**（真值 0.003°）。

### I.6 RViz 配方：`Fixed Frame` 选哪个、以及为什么用户看到"平放被倾斜"

`lio_rviz:=True` 起的是 `src/rm_nav_bringup/rviz/pointlio.rviz`（`bringup_sim.launch.py:950,1639`），
它的 **`Fixed Frame: odom`**（`:255`），显示 `/cloud_registered`（`CloudRegistered`，`Use Fixed Frame: true`，
按 Z 轴着色，`:158-181`）、TF、`/path`、`/aft_mapped_to_init`、`/Laser_map`、`/cloud_effected` ——
**没有**原始 `/livox/lidar/pointcloud` 的 display（`nav2.rviz` 更是两个点云都没有，见 §B.3 结尾）。

RViz 干的事：把消息里的点从 `header.frame_id` 用 TF 变到 `Fixed Frame`。把两朵云分别代入：

| `Fixed Frame` | `/livox/lidar/pointcloud`（frame_id=`livox_frame`，**数据水平**） | `/cloud_registered`（frame_id=`odom`，**数字里已经含那次 −30°**） | 机器人/代价图/TF |
|---|---|---|---|
| **`livox_frame`** | **水平**（这正是数据的真面目，也恰好等于世界几何） | 被 `T(livox←odom)≈R_x(+30°)` 抵消 ⇒ 也**看起来水平** | 车/代价图**斜 30°**（它们在世界/odom 里是平的） |
| **`odom` / `map`**（= 用户当时的选择） | **30° 斜坡**（被 `T(odom←livox)≈R_x(−30°)` 转了） | **30° 斜坡**（数字里就带着） | 车看起来正（外加 bug ③ 的 4.9° 假俯仰） |
| **`base_link`** | **30° 斜坡**（`T(base←livox)=R_x(−30°)`） | **30° 斜坡** | 车正 |

**⇒ 结论（这就是用户看到的）：**
* 用户看到的是 **`/cloud_registered` + `Fixed Frame: odom`** ⇒ **场景被刚性旋转 −30°**：地面是一个
  30.97° 的**平面斜坡**，环状扫描图案、近场那团自击、远处结构**都没变**，只是整个场景绕 x 转了 30°。
  **这正是一朵"水平扫出来的点云被整体倾斜"该有的样子**，用户的原话「点云看着像平放扫到的东西倾斜了，
  不是倾斜放置扫描到的东西」是**完全准确**的描述 —— 因为数据**真的**是水平的（§I.4 的 0.0004° 配准）。
* **没有任何一个 `Fixed Frame` 能让"点云"和"车/代价图"同时看起来正** —— 这本身就是"帧与数据差 30°"
  的定义。要同时正，必须修 §I.8 里那个发布变换（或让点云真的表达在传感器系里）。
* 判据（下次一眼分辨）：
  * **刚性转过**（地面是一个平面斜坡、图案不变、环半径不变）= **帧/label 问题**；
  * **覆盖图案真的变了**（一侧彻底没有地面、最近地面环半径 0.31 m ↔ 1.96 m 的量级差）= **物理倾角**。
  `urdf` 档**两件事同时存在**：物理倾角是真的（§I.3/§I.4），而 RViz 里显眼的那个 30° 斜坡是**帧的问题**。

### I.7 感知链在 `urdf` 档的实测后果（p2l 高度带 / 自击掩膜 / `/scan` / 代价图）

| 环节 | `plugin` | `urdf` | 判读 |
|---|---|---|---|
| `p2l` 输入 `/segmentation/obstacle` 的点（中位/帧） | **6221** | **6222** | 输入是**同一朵云**（§I.4）⇒ 分割本身不变 |
| `/global_costmap/voxel_grid` 点（中位/帧，STVL 真正标进障碍的三维点） | **761** | **148** | 与 §C.3 的 146（urdf）/ 1091（默认模型）同向；`urdf` 档**少了 5 倍** |
| **`p2l` 的高度带**（`min/max_height = −1.0/1.0`，`target_frame: ""` ⇒ 作用在**点云自带帧**） | 带内点占比 **1.0000** | **1.0000** | **仍然有效** —— 因为点云坐标没变（这条"歪打正着"§C.4 第 2 条已记） |
| **自击掩膜**（`r_xy ≤ 0.2416 且 z ≥ −0.2295`，见 `src/rm_nav_bringup/config/traversability_self_mask_robot11.yaml`） | 命中 **3402（29.197%）** | 命中 **3403（29.200%）** | **仍然有效**（掩膜盒子与点云都在同一个水平系里烘的）；**只有**当把插件改成"真·传感器系表达"之后才需要重烘（§I.8） |
| `/scan` 有限波束 | **949** | **948** | **几乎不变**（`target_frame: ""`，不做 TF） |
| **`/scan` 每条波束在 odom 里的绝对 z**（`obstacle_layer.min/max_obstacle_height = 0.0/2.0` 就是在这层量的） | `[0.0549, 0.0839]` m、**0/949 被丢** | `[−2.67, +0.40]` m、**677/948 = 71.4% 被丢** | **这才是坏掉的那一级**：`/scan` 是一张"2D 平盘"，它的平面在 `livox_frame` 里；`livox_frame` 在 odom 里斜 30° ⇒ 盘变成斜平面，半圈被抬到 +0.5r、半圈被压到 −0.5r |
| 被丢波束的方位（`urdf`） | — | `0..30:39, 30..60:160, 60..90:137, 90..120:102, 120..150:126, 150..180:109`，**右半边全 0** | 丢掉的**正是"斜装雷达唯一看得见东西的那半边"**（左侧地面），而本来就没回波的右半边没有可丢的 |
| local costmap lethal / inscribed | **471 / 15909** | **60 / 3645** | `urdf` 档**代价图几乎空了**（与 §C.3 的 340→75 同向、同量级） |
| 车心到最近 lethal 格 | **0.387 m** | **0.718 m** | 车周围反而"更自由"（假自由） |
| lethal 格的方位（**两档都**） | 前 127 / **左 224** / 后 120 / **右 0** | 前 5 / 左 **0** / 后 55 / 右 **0** | `plugin` 档"右侧 0"是**物理盲区**（§I.3）；`urdf` 档连左侧（唯一看得见的那半）也被高度带丢了 |

### I.8 「斜装雷达到底该怎么处理」——本轮学到的（含精确改法表）

**五条原则（都可以从本轮数字反推）：**

1. **先分清三个"livox_frame"**：物理传感器系（斜）／点云数据实际表达的系（本仓 = 父 link 朝向 + 传感器原点）／
   `header.frame_id`（= `<sensor name>`）。**判据**：对点云做地面平面拟合，法向与 `frame_id` 那个帧的 z
   夹角应当 ≈0（`plugin`/`urdf` 实测 1.03°/1.02° **但后者那个帧自己斜 30°** ⇒ 这就是不自洽）；
   真机上点云本来就该在斜的传感器系里，所以真机的判据是"用 TF 把它转到 `base_link` 再量"。
2. **倾角只能记在一个地方**：记在插件（`plugin` 档：帧正、点云正、射线斜 —— 自洽但与实物不符，
   而且 mesh 画不出来）**或**记在关节（`urdf` 档：帧斜、mesh 斜、射线斜 —— 但点云还留在水平系）——
   **选一条，然后把点云的表达与 `frame_id` 一起改到自洽**。
3. **外参要按"点云实际表达的系"给，不是按"实物几何"给**。本仓 `extrinsic_T=[0,0,0.05]`、
   `extrinsic_R=I` 在**两档下都与实物几何一致**（§I.5.2 实测 TF `imu_link←livox_frame`）；错的是
   "点云并不在 `livox_frame` 里"。**光改外参补不了这个错**。
4. **`odom` 不保证重力对齐**。本仓 `small_point_lio` 是 `fix_gravity_direction: true` + 初始 `R=I`
   （`eskf.h:30`、`small_point_lio.cpp:54-64`）⇒ **odom ≡ 初始化那一刻的身体系**。
   仿真里之所以没斜，是因为插件把点云做成水平的（"歪打正着"）；**真机**上如果把"真·传感器系"的
   点云直接喂进去，`odom` 的 z 就会斜 30°，地面在 odom/地图里就是一个 30° 斜坡 ——
   那时要么在 LIO 初始化时做重力对齐（把初始 R 设成 `R_x(+30°)`），要么在下游统一用 `base_link` 重力系。
5. **每个"高度"参数都属于某个具体的帧**：`linefit.sensor_height`（传感器系 z 到地面）、
   `p2l.min/max_height`（点云自带帧）、`obstacle_layer.min/max_obstacle_height`（**代价图帧**，
   本轮实测被它丢掉 71.4% 的波束）、自击掩膜的 z 门限（传感器系）。斜装时"点云自带帧 ≠ 重力系"
   ⇒ 要么在**源头**对齐（本仓 `plugin` 档的做法），要么给这些节点配 `gravity_aligned_frame` /
   `target_frame`（本仓**代码已支持**：linefit 的 `Eigen::Affine3d` C++ bug 已修，
   `ground_segmentation_node.cc:142` 用 `Identity()` 起步；`p2l` 支持 `target_frame`，
   代价 = 会引入 TF/MessageFilter 那一类故障面，见 `laserscan_params.yaml` 顶部注释）。

**如果要让 `urdf` 档真正自洽（本主题**不做**，只登记精确改法）：**

| 目标 | 文件 | 改什么 | 预期效果（可验收） |
|---|---|---|---|
| **a. 点云真的表达在传感器系**（真机同款） | `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp:276`（并同步 `:164-165` 的射线起点或把 collision 挂到传感器自己的 frame） | `axis = sensor_rot * mount_rot_ * ray` → `axis = mount_rot_ * ray` | `/livox/lidar/pointcloud` 的地面法向与 `livox_frame` 的 z 夹角回到 ~1°，**而这个帧真的斜 30°** ⇒ 数据与帧自洽（"点云里地面是斜的"变成物理事实）；**下游必须做重力对齐**（下表 b/c） |
| **b. 第一级下游（地面分割）** | `src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/config/segmentation_sim_robot11.yaml` | `gravity_aligned_frame: base_link`（该键的 C++ bug 已修） | linefit 在 `base_link` 里量 `sensor_height: 0.2595`，地面分割恢复（此前实测该键一开 = 0 点，是 bug 造成的，已不适用） |
| **c. 第二级下游（scan 化）** | `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml` | `target_frame: base_link` | `/scan` 变成"水平面里的一圈"，`min/max_height` 恢复"离地高度"语义；代价 = 引入 TF/MessageFilter（本仓 2026-09-23 刻意避开的那类故障面） |
| **d. 自击掩膜** | `src/rm_nav_bringup/config/traversability_self_mask_robot11.yaml` + `tools/scripts/regress/robot11_self_mask.py` | **只在 a 做了之后**才需要重烘 113 个 AABB（改成传感器系里算） | 掩膜覆盖不变（本仓现状：掩膜条件与点云同在水平系 ⇒ **不用改**，实测命中 29.20% 两档相同） |
| **e. bug ②（发布变换）** | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:141,159,191` | 删掉那次 `lookupTransform("base_link", lidar_frame)` + `R*p+T`，直接发已经在 odom 的点（`frame_id` 保持 `odom`） | `/cloud_registered` 的地面回到 odom 里的水平（**本轮的验收数字就是"反变换后 1.041°"**）；`plugin` 档的"0.2 m 平移偏差"也一并消失 |
| **f. bug ③（odom→base TF）** | 同文件 `:99,109` | 改成**合成** `T_ob = T_ol · T_bl⁻¹`（不要 `T_bl⁻¹·T_ol·T_bl` 这个相似变换）。⚠️ 数值取决于 ① 怎么修：· 若按 a 走「真机同款」（点云真的在 `livox_frame` 里）⇒ 会给出 `rpy (30°, 0, yaw)`，**那个 30° 是物理事实**（odom 定义在斜的初始传感器系里，见原则 4），此时必须同时做重力对齐；· 若点云保持水平（现状）⇒ 正确值就是 `R_ol` 本身（水平） | 现状下 `odom→base_link` 的姿态误差从 **4.89° 假俯仰** 回到 <0.5°（nav2 的车身姿态才正确） |
| **g. 只想"看着对"** | 不改代码 | 保持 `plugin` 档（默认）+ 把 `l12.stl` 的视觉单独挂一个斜 30° 的 visual link | 帧/点云/下游全自洽，且画出来的雷达是斜的（代价：多一个 visual link，与上游 URDF 的 provenance 需要登记） |

### I.9 复现命令（本轮逐字使用的）

```bash
# ① 两个档各跑一次（隔离无头；工具本轮新增，只有订阅，不发任何东西）
tools/scripts/tiltmount/run_tilt_mount_probe.sh tm_plugin --variant plugin --settle 25 --duration 45 \
    --frames 3 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_tilt_mount_probe.sh tm_urdf --variant urdf --settle 25 --duration 45 \
    --frames 3 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
    robot11_mount:=urdf spin_speed:=0.0 gui:=False
#   → .tmp_tiltmount/<tag>/{probe.json, clouds.npz, launch.log, mount_evidence.txt, gz_model_info.txt}

# ② 离线对照（H2 本地云配准 / 反变换检验 / 扇区表 / 高度带 / 代价图；纯 numpy+scipy，不用 Gazebo）
python3 tools/scripts/tiltmount/tilt_mount_compare.py \
    --a .tmp_tiltmount/tm_plugin --b .tmp_tiltmount/tm_urdf --out .tmp_tiltmount/compare.json

# ③ 渲染 + Gazebo 自己的 SDF（"传感器挂在 base_link 下、pose 折进了关节旋转"的静态证据）
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro livox_mount:=plugin > /tmp/r11_plugin.urdf
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro livox_mount:=urdf   > /tmp/r11_urdf.urdf
diff /tmp/r11_plugin.urdf /tmp/r11_urdf.urdf          # 只差 2 行：关节 rpy 与插件 <tilt_rpy>
mkdir -p /tmp/gzhome-sdf   # gz sdf 要写 $HOME/.gazebo，必须给一个可写的 HOME
HOME=/tmp/gzhome-sdf gz sdf -p /tmp/r11_urdf.urdf | sed -n '1274,1325p'   # ← 传感器 pose（含 -0.523599）

# ④ tf2 合成顺序单测（bug ③ 的语义钉死；30 行 C++，不进 colcon）
g++ -O0 -o /tmp/tf2order tools/scripts/tiltmount/tf2_compose_order_test.cpp \
    -I/opt/ros/humble/include/tf2 -L/opt/ros/humble/lib -ltf2 -Wl,-rpath,/opt/ros/humble/lib && /tmp/tf2order
#   → ① tf2::Transform 的 `A*B` 就是矩阵序 R_A·R_B（`C*p` 与 `A*(B*p)` 相同）；
#     ② 代码那行 `T_bl⁻¹*T_ol*T_bl` 对一个「只有 yaw 9° 的水平姿态」给出
#        (0.306, 4.486, 7.810)°（= 实测那 4.9° 假俯仰的来源，符号/量级都对得上）；
#     ③ 真机约定下的正确合成 `T_ol*T_bl⁻¹` 给出 (-30.000, 0, 9.000)°（那个 −30° 是物理的，
#        对应「odom = 斜的初始传感器系」）。
```

### I.10 未验证 / 诚实清单

1. **没有在真 RViz 里看一眼**：§I.6 的配方是从 `pointlio.rviz` 的配置（`Fixed Frame: odom`、
   `Use Fixed Frame: true`）+ TF 数学推出来的，本轮全是无头跑（与 §G 第 4 项同款限制）。
2. **两档 `/cloud_registered` 的跨档配准给出 26.96° 而不是 30.000°**：两档是**两次独立的 Gazebo
   会话**（出生后的初始姿态/漂移不同），我**没有**做"同一会话内切换"的对照 ⇒ 这个 27° 只能读作
   "≈30°、绕 −x"，不能当精确值。**逐档内的反变换检验（1.041°/0.742°）不受此影响。**
3. **bug ② 的"绕 odom 原点旋转"这一条没做实验**：`R*point + T` 里的 `R` 是绕 **odom 原点**（不是绕
   传感器）转的，理论上"车开出原点后整朵云会绕原点甩"。本轮车静止在原点附近 ⇒ **两种写法数值上分不开**，
   这一条是**从代码读出来的**，没有开出去验证。
4. **bug ③ 对 nav2 的实际影响没有单独量化**：只量了 TF rpy（假俯仰 4.89°）与代价图计数，
   **没有**量"把 4.89° 去掉之后代价图会变多少"。
5. **"改成真·传感器系表达之后，linefit/p2l/自击掩膜会怎样"没有实测**：§I.8 的 a~d 是**改法**，
   本轮只测了"现状（点云水平）下这些参数仍然有效"这一半（自击掩膜命中率两档相同、`p2l` 带内 100%）。
6. **只测了 `lio:=small_point_lio`**：`fastlio` / `pointlio` 的 TF/外参路径不同（`lio_tf_adapter` 那条
   杆臂的 `urdf` 档数值只在离线算过，§G 第 5 项）⇒ 本节结论**不自动适用**。
7. **没有测 GUI/带显示器时的 RTF 与显示行为**（全无头）；也没有测"车动起来"时 `/cloud_registered`
   的甩动（见第 3 项）。
8. **出生后自转 ~10° 这件事没有归因**（真值 `/odom_ground_truth` 的 yaw 0→10.152°、两档都复现）：
   它不是 LIO 漂移（roll/pitch 跨度 0.16°），但**为什么**出生后会转 10°（接触/降落/`planar_move` 初值）
   本轮没查。它不影响本节任何结论，只解释了 `R_ol` 里那个 ~9° 的 yaw。

### I.11 回退

| 想退掉什么 | 怎么做 |
|---|---|
| `urdf` 档 → 今天的行为 | 去掉 `robot11_mount:=urdf`（默认 `plugin`，本节**没有**改任何默认值） |
| 本节的工具 | `rm -rf tools/scripts/tiltmount/`（只被上面 §I.9 的命令引用；不进任何 launch/节点） |
| 本节的文档与工具（整节） | `git revert <本节 commit>`：只动 `docs/tilted_lidar_fidelity.md` 与新目录 `tools/scripts/tiltmount/`，**不动任何默认值/参数/生成物** |
| 本节的原始数据 | `.tmp_tiltmount/**`（未入库） |

---

## J. 2026-10-09：两个 LIO bug 的修复 + `robot11_mount:=sensor`（"物理斜装 + 账也对"）实测 A/B

> 触发 = 用户 2026-10-09 的下一条指令（父任务原文）：**先无条件修 §I.5.3 里那两个 LIO 侧 bug，
> 再把 §I.8 的"斜装正确改法"真正实现成一档 opt-in**（默认与其它槽位逐字节不变），
> 并用无头隔离跑把整条链**逐级 + 端到端**量一遍（ground / scan / 代价图 / 车动不动 / LIO 漂移 / RTF / 契约）。
>
> 本节与 §I 的关系：**§I 是诊断（谁斜了、账错在哪）**；**§J 是施工（改了哪几行、改完的数字是多少）**。
> §I 的所有内容**逐字保留**；本节只**追加**，并在 §J.1 里登记 §I.9/§I.8(f) 一处**记法更正**。
>
> 全部无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、
> `unset DISPLAY`、只按 `/proc/<pid>/environ` 清本 master URI 的 gazebo），
> 原始数据 `.tmp_tiltmount/j2_*/`（**改后**）与 `.tmp_tiltmount/{tm_plugin,tm_urdf}/`（**改前**，§I 那一轮）。

### J.0 一句话结论

1. **bug ②（`/cloud_registered` 多转一次）已修**：点云不再被乘 `T(base_link←livox_frame)`；发布的坐标与
   `frame_id=odom` 自洽。`urdf` 档的验收数字从 **30.970°** 回到 **1.054°**（§I 的判据"反变换后 1.041°"），
   默认档（`plugin`）**帧内几何逐项不变**、只把整朵云搬回真正的 odom 坐标（−t = −0.2044 m）。
2. **bug ③（`odom→base_link` 用共轭发姿态）已修**：姿态改成**合成** `T_ol·T_bl`，
   `urdf` 档的**假俯仰从 4.890° 降到 0.317°**（真值 0.003°；`sensor` 档的 rpy pitch **−4.9°** 不是假俯仰，
   而是"绕斜 30° 的 odom 转 yaw"在 ZYX 下的**正确分解** —— 判据要用矩阵/一致性，见 §J.4 的"读表两个坑"），
   默认档姿态**逐位不变**（单测 §J.3 的 ⑤ 断言 + 实跑对照）。
   ⚠️ **平移故意保留旧值**（不是漏改）：见 **§J.5**（把平移也改成物理真值会让**默认模型与 robot11 的
   局部代价图整张变空**：lethal 471→0 / 445→0，而那条 `min_obstacle_height` 在**本任务禁改**的目录里）。
3. **新增第三档 `robot11_mount:=sensor`**：关节/插件与 `urdf` 档**逐字节相同**，只多一个
   `<cloud_frame>sensor</cloud_frame>`（插件把点表达在**真·传感器系**），并由 launch **只在这一档**
   给 linefit 加 `gravity_aligned_frame: base_link`、给 p2l 加 `target_frame: base_link`
   ⇒ **帧与数据自洽 + 下游重力对齐**。默认档 `plugin` 与其它模型/槽位的渲染与行为**逐字节不变**（§J.3 证据）。
4. **A/B（`plugin` / 旧 `urdf` / 新 `sensor`）**见 §J.4；**这条链仍然解决不了什么**见 §J.7。

---

### J.1 两个 bug 的修复（文件:行 / 改法 / 判决性数字）

#### J.1.1 bug ②：`/cloud_registered` 对"已经在 odom 的点"又乘了一次 `T(base_link←livox_frame)`

| 项 | 内容 |
|---|---|
| 文件:行 | `src/rm_localization/small_point_lio/src/small_point_lio_node.cpp`（改前 `:139-156` 的 `lookupTransform("base_link", lidar_frame)` + `:191` 的 `R*p+T`；`frame_id="odom"` 在 `:159`） |
| 改法 | **删掉那次 TF 查询与逐点刚体变换**，直接把回调里的点写进消息（`*pointer = point.x()/y()/z()`）；`frame_id` 仍是 `odom`。文件里留了日期 + 机制 + 影响面的中文注释 |
| 为什么这样对 | 回调收到的点来自 `small_point_lio.cpp` 的 `p_odom = R_ol·(extrinsic_R·p_lidar + extrinsic_T) + t_ol` —— **已经是 odom 系**，与 `frame_id="odom"` 一致 ⇒ 再乘一次就是"多转/多移一次" |
| 判决性数字（`urdf` 档） | 改前：`/cloud_registered` 地面倾角 **30.970°**（按同一条 TF 反变换回去 = **1.041°**）；改后：**1.054°** ⇒ 就是那个"反变换后的值"本身 |
| 默认档（`plugin`）差多少 | 那条 TF 是**纯平移** ⇒ 改前只把整朵云平移了 t = (0.000562, 0.130916, 0.157028) m（|t| = 0.2044 m）；改后这朵云回到真正的 odom 坐标。**帧内几何（点数/地面倾角/扇区/自仰角谱）逐项不变**，见 §J.2 的 before/after 表 |
| 副作用检查 | `/global_costmap/voxel_grid` 每帧点数 **761 → 791.0**（不变，量级内）；`/segmentation/*`、`/scan` 的点数不变 |

#### J.1.2 bug ③：`odom→base_link` 的姿态是**共轭**而不是**合成**

| 项 | 内容 |
|---|---|
| 文件:行 | 同文件（改前 `:99` 的 `lookupTransform(lidar_frame, "base_link")` + `:109` 的 `T_ob = T_bl⁻¹·T_ol·T_bl`） |
| 改法 | 姿态取**合成** `tf_odom_to_base_link = tf_lidar_odom_to_lidar_frame * tf_base_link_to_lidar_frame`（= `T_ol · T_bl`，`T_bl = lookupTransform(livox_frame←base_link)`）；⚠️ 平移保留旧值（§J.5） |
| 为什么这样对 | 坐标映射链式法则：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ `T_odom←base = T_ol·T_bl`。等价说法：位姿一致性 `T_ol = T_ob·T_bl⁻¹` ⇒ `T_ob = T_ol·T_bl`。**共轭只在 `T_bl` 是纯平移时"姿态碰巧对"** |
| 判决性数字（`urdf` 档） | 改前：发布的 `odom→base_link` rpy = `[0.383, **4.890**, 7.701]°`（`/odom` 窗口内 pitch 跨度 4.924°，真值 `/odom_ground_truth` 0.003°）；改后：**rpy = [[30.060, 0.317, 9.162]]°**、**假俯仰 0.317°**（< 0.5° 验收线；那个 30° 出现在 **roll** 上，= `odom` 定义在斜的初始传感器系，物理事实，见 §I.8 原则 4） |
| 默认档（`plugin`）差多少 | `T_bl` 的旋转 = 单位阵 ⇒ 合成与共轭的**姿态逐位相同**；平移也**逐位相同**（因为平移显式取旧值）⇒ **默认档的 TF/`/Odometry` 逐位不变**（单测 ⑤ + 实跑 §J.2 对照） |

> **记法更正（对 §I.8(f)/§I.9 的一句话）**：§I.9 的 C++ 单测原来把"正确合成"写成 `T_ol·T_bl⁻¹`；
> 在 `T_bl := lookupTransform(livox_frame, "base_link") = T(livox←base)` 这个（与代码一致的）记法下，
> 它是**差一次求逆**的写法 —— 本节的 `tools/scripts/tiltmount/tf2_compose_order_test.cpp`
> 已改成由**链式法则**推出 `T_ob = T_ol·T_bl` 并断言（非单位 `T_bl` 下 rpy = (30, 0, 9)°、假俯仰 0）。
> §I.8(f) 的"会给出 rpy (30°, 0, yaw)"这一句是**对的**；§I.9 的那行公式与它自相矛盾，以本节为准。

---

### J.2 默认路径的回归证据（改动前 vs 改动后，逐项 diff）

口径：同一世界/同一出生点/同一 LIO/同一条命令（`world:=RMUL2026 mode:=slam_nav lio:=small_point_lio
robot:=robot11 spin_speed:=0.0`），改前 = `.tmp_tiltmount/tm_plugin`（§I 那一轮，旧二进制），
改后 = `.tmp_tiltmount/j2_plugin`。表由 `tools/scripts/tiltmount/tilt_ab_table.py --diff` 生成：

（差 = 改动后 − 改动前；`—` = 该跑没采到这个量）
| 量 | before（改动前） | after（改动后） | 差 |
|---|---|---|---|
| TF base_link→livox_frame rpy(度) | [0.000, -0.000, 0.000] | [0.000, -0.000, 0.000] | — |
| TF odom→base_link rpy(度) | [0.217, 0.047, 9.667] | [0.259, 0.404, 9.503] | — |
| TF odom→base_link xyz(m) | [0.000, 0.009, -0.094] | [-0.003, 0.185, -0.097] | — |
| 原始云地面倾角 vs 自己帧(度) | 1.035 | 0.957 | -0.077237 |
| 原始云地面平面高(m) | 0.248 | 0.250 | 0.001424 |
| 原始云点数(中位) | 11944.500 | 11945.000 | 0.500000 |
| /cloud_registered 点数(中位) | 11953.000 | 11954.000 | 1.000000 |
| /cloud_registered 地面倾角(度) | 0.742 | 0.545 | -0.197536 |
| /cloud_registered 低于水平面点数 | 5967 | 6768 | 801 |
| /segmentation/ground 点数(中位) | — | 5710.000 | — |
| /segmentation/obstacle 点数(中位) | 6221.000 | 6223.000 | 2.000000 |
| /scan 有限波束 | 949 | 948 | -1 |
| /scan 超出高度带比例 | 0.000 | 0.000 | 0.000000 |
| local costmap lethal | 471 | 461 | -10 |
| local costmap inscribed | 15909 | 16937 | 1028 |
| 车那格的值 | — | 80 | — |
| 车半径圆内 ≥99 格 | — | 440 | — |
| 车半径圆内 free 格 | — | 26 | — |
| 到最近 lethal 格(m) | 0.387 | 0.391 | 0.003317 |
| RTF | — | 0.428 | — |

**读法（五条）**：
1. **代码级的不变性有单测**：默认档（`T_bl` 纯平移）下，新的写法与旧写法**姿态与平移都逐位相同**
   （§J.3 第 7 行的 tf2 单测断言 ⑤）——这是"默认档不变"的**构造性**保证。
2. **实跑对照的读法**：表里 `TF odom→base_link rpy` 的差（roll 0.04°、pitch 0.36°、yaw 0.16°）与
   `原始云地面倾角`（1.035 → 0.957°）、`/cloud_registered 地面倾角`（0.742 → 0.545°）都落在
   **跑间噪声**里：本仓"出生后落定"（车生成在 `z=0.2`、随后下沉 0~0.15 m，LIO 在坠落中做重力初始化）
   让**每一次跑**的 odom 原点高度、车的静止姿态都略不同 ⇒ 姿态估计的 roll/pitch 跑间差可达 **0.4°**、
   代价图计数跑间差可达 **±30%**（§D.2 的 lethal 340/471/462/423 就是同一量级的跑间散布）。
   ⇒ 这一张表的正确结论是"**没有超出跑间噪声的系统性变化**"，而不是"每个数都逐位相同"。
3. **帧内几何逐项不变**：原始云点数（11944.5 → 11945）、`/cloud_registered` 点数（11953 → 11954）、
   `/scan` 有限波束（949 → 948）、`/segmentation/*` 点数（6221 → 6223）都在 ±2 点内 ——
   因为 bug ② 在默认档只是一次**纯平移**（旋转是单位阵）。
4. **代价图仍然"能被看见"**：局部代价图 lethal **471 → 461**、inscribed 15909 → 16937、
   车那格仍非 free（80）、车半径圆内 `≥99` 440 格 —— 与改动前**同量级**（这一条是本轮最关键的回归点：
   §J.5 里"平移取真值"的那一版会把它们**全部清零**）。
5. **默认模型对照**：**本轮没有单独跑默认模型对照**（预算）；可用的最强证据是：默认模型与 robot11 共用同一份 nav2 公共参数（`nav2_params_sim_base.yaml`），而 bug ② 在默认档只是"整朵云平移 0.2044 m"、bug ③ 在默认档**逐位不变**（`T_bl` 纯平移）⇒ 默认模型的 TF/`/Odometry`/点云几何都与改动前一致。另外 `.tmp_tiltmount/j1_default`（= 平移取真值的那一版）量到默认模型的局部代价图同样会变空（lethal 0 / inscribed 0），这正是 §J.5 保留旧平移的直接原因。

### J.3 静态证据：`sensor` 档是**纯增量**，`plugin`/`urdf` 与其它模型逐字节不变

| 证据 | 命令 | 结果 |
|---|---|---|
| 生成物 = 生成器输出 | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --upstream … --assets … --out .tmp_tiltfix/static/xacro_regen_final.xacro` | `diff` 为空 ⇒ **生成器与入库的 xacro 逐字节一致** |
| `plugin` / `urdf` 两档的渲染 | `xacro sentry_robot_robot11_sim.xacro [livox_mount:=urdf] livox_tilt_rpy:="-0.523598775598293 0 0"`，改前 vs 改后 | **去注释后逐字节相同**（两份都是 **35410 B**）；原始 diff 只有注释行（我加的说明）与 xacro 文件名那一行 |
| `sensor` 档的渲染 | 同上 + `livox_mount:=sensor` | 与 `urdf` 档**只差 3 处**：`body_to_livox` 的 `origin rpy`（= 倾角）、插件的 `<tilt_rpy>`（= 单位阵）、**新增** `<cloud_frame>sensor</cloud_frame>`（35410 → 35452 B） |
| 其它模型 | `grep -rln cloud_frame src/rm_nav_bringup/urdf/*.xacro` | **只有** `sentry_robot_robot11_sim.xacro`（其它模型的 xacro 里没有这个元素、也没有 `tilt_rpy`） ⇒ 插件走的是"逐字节不变"那条分支（缺省 = 父 link 系） |
| 插件语义 | `livox_points_plugin.cpp` 的三元表达式 | `cloud_frame` 缺省/`parent` ⇒ `axis = sensor_rot·mount_rot·ray`（**与 2026-10-07 起逐字节相同的代码路径**）；只有 `sensor` 才走 `axis = mount_rot·ray` |
| launch 侧 | `robot11_mount:=sensor` 时才多两份**增量覆盖**参数文件 | 其它档递进节点的参数文件列表**逐个不变**；节点集合不变（同一节点名、互斥条件） |
| 契约 | `ros2 topic info -v`（每个 tag 跑完都测，见 `mount_evidence.txt`） | `/livox/lidar{,/pointcloud}`、`/cloud_registered`、`/segmentation/{ground,obstacle}`、`/scan`、`/odom`、`/local_costmap/costmap`、`/global_costmap/voxel_grid` **每个话题恰好 1 个发布者**（三档都测了） |
| tf2 语义单测 | `.tmp_tiltfix/tf2order`（= `tools/scripts/tiltmount/tf2_compose_order_test.cpp`） | **exit 0**：① 乘法序；② 旧写法的假俯仰 4.486° 且**违反一致性**（= bug）；③ 现在的写法假俯仰 0.000°、姿态满足一致性；④ 逐点恒等式 <1e-15 m；⑤ 默认档姿态**与平移都与旧写法逐位相同** |

### J.4 A/B 三方表：`plugin`（默认，今天）vs `urdf`（"物理保真、账不对"诊断档）vs `sensor`（新，正确链）

**表 A：三档静止**（同一世界/出生点/命令；`.tmp_tiltmount/j2_{plugin,urdf,sensor}`）

| 量 | plugin | urdf | sensor |
|---|---|---|---|
| 安装档 variant | plugin | urdf | sensor |
| TF base_link→livox_frame rpy(度) | [0.000, -0.000, 0.000] | [-30.000, 0.000, 0.000] | [-30.000, 0.000, 0.000] |
| TF odom→base_link rpy(度) | [0.259, 0.404, 9.503] | [30.060, 0.317, 9.162] | [29.957, -4.874, 8.520] |
| TF odom→base_link xyz(m) | [-0.003, 0.185, -0.097] | [-0.014, 0.058, -0.141] | [0.006, -0.025, -0.087] |
| 原始云 frame_id | livox_frame | livox_frame | livox_frame |
| 原始云地面法向 vs **自己的 frame_id** z(度) | 0.957 | 1.023 | 29.092 |
| 原始云地面法向 vs **base_link** z(度)（账对不对） | 0.957 | 30.920 | 1.035 |
| 原始云地面平面高(m) | 0.250 | 0.248 | 0.248 |
| /cloud_registered frame_id | odom | odom | odom |
| /cloud_registered 地面倾角（odom 里，度） | 0.545 | 1.054 | 29.135 |
| /cloud_registered 低于水平面点数 | 6768 | 6770 | 4093 |
| odom 帧相对真实重力倾角(度) | 0.376 | 30.116 | 30.359 |
| /odom 窗口内 rpy 跨度(度) | [0.205, 0.347, 10.127] | [0.213, 0.354, 9.475] | [0.396, 5.167, 8.703] |
| /scan frame_id | livox_frame | livox_frame | base_link |
| /scan 有限波束/帧 | 948 | 949 | 755 |
| /scan 波束在 odom 里超出 [0,2] m 的比例 | 0.000 | 0.000 | 0.404 |
| /scan 波束 z(odom) min / max (m) | 0.020 / 0.089 | 0.007 / 0.076 | -0.426 / 2.317 |
| local costmap frame_id | odom | odom | odom |
| local costmap lethal / inscribed / free | 461 / 16937 / 29478 | 404 / 15349 / 31210 | 518 / 16312 / 32919 |
| 车那格的值 | 80 | 86 | 86 |
| 到最近 lethal 格距离(m) | 0.391 | 0.352 | 0.328 |
| 车半径圆内格数 / ≥99 / free | 1000 / 440 / 26 | 999 / 360 / 26 | 997 / 509 / 4 |
| lethal 方位（前后左右） | {front=119, left=205, back=137, right=0} | {front=159, left=113, back=132, right=0} | {front=236, left=206, back=76, right=0} |
| RTF（记录窗） | 0.428 | 0.428 | 0.412 |
| /cmd_vel 条数 / 非零 | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} |
| /plan 帧数 / 每条位姿数(中位) | {n_frames=0, pts_median=—} | {n_frames=0, pts_median=—} | {n_frames=0, pts_median=—} |
| --drive 漂移 vs 真值(m) / yaw(度) | — | — | — |
| --goal 真值位移(m) / 目标后残余(m) | — | — | — |
| 每帧点数（中位） | 原始云=11945.0, /scan 输入(obstacle)=6223.0, /segmentation/ground=5710.0, /cloud_registered=11954.0, voxel_grid=791.0 | 原始云=11947.0, /scan 输入(obstacle)=6225.0, /segmentation/ground=5704.0, /cloud_registered=11955.0, voxel_grid=145.0 | 原始云=11947.0, /scan 输入(obstacle)=6223.0, /segmentation/ground=5705.0, /cloud_registered=11955.5, voxel_grid=776.5 |

**表 B：三档 + 短目标**（`--goal-forward 1.5 --goal-wait 30`；`.tmp_tiltmount/j2_*_goal`）

| 量 | plugin-goal | urdf-goal | sensor-goal |
|---|---|---|---|
| 安装档 variant | plugin | urdf | sensor |
| TF base_link→livox_frame rpy(度) | [0.000, -0.000, 0.000] | [-30.000, 0.000, 0.000] | [-30.000, 0.000, 0.000] |
| TF odom→base_link rpy(度) | [0.016, 0.119, 37.015] | [30.209, -0.003, 12.219] | [29.228, -8.568, 14.997] |
| TF odom→base_link xyz(m) | [0.182, 0.173, -0.090] | [-0.009, -0.034, -0.088] | [0.013, -0.026, -0.087] |
| 原始云 frame_id | livox_frame | livox_frame | livox_frame |
| 原始云地面法向 vs **自己的 frame_id** z(度) | 0.893 | 0.893 | 29.217 |
| 原始云地面法向 vs **base_link** z(度)（账对不对） | 0.893 | 30.790 | 0.893 |
| 原始云地面平面高(m) | 0.255 | 0.255 | 0.255 |
| /cloud_registered frame_id | odom | odom | odom |
| /cloud_registered 地面倾角（odom 里，度） | 0.956 | 1.119 | 29.148 |
| /cloud_registered 低于水平面点数 | 6770 | 8450 | 4086 |
| odom 帧相对真实重力倾角(度) | 0.110 | 30.209 | 30.308 |
| /odom 窗口内 rpy 跨度(度) | [0.226, 0.471, 37.129] | [0.167, 0.214, 16.895] | [1.156, 8.527, 14.603] |
| /scan frame_id | livox_frame | livox_frame | base_link |
| /scan 有限波束/帧 | 938 | 949 | 755 |
| /scan 波束在 odom 里超出 [0,2] m 的比例 | 0.000 | 0.000 | 0.433 |
| /scan 波束 z(odom) min / max (m) | 0.056 / 0.072 | 0.113 / 0.140 | -0.517 / 2.190 |
| local costmap frame_id | odom | odom | odom |
| local costmap lethal / inscribed / free | 571 / 17617 / 27677 | 418 / 16895 / 29697 | 432 / 15087 / 32885 |
| 车那格的值 | 86 | 84 | 90 |
| 到最近 lethal 格距离(m) | 0.386 | 0.341 | 0.324 |
| 车半径圆内格数 / ≥99 / free | 997 / 429 / 24 | 998 / 375 / 25 | 1000 / 500 / 7 |
| lethal 方位（前后左右） | {front=177, left=185, back=209, right=0} | {front=137, left=121, back=160, right=0} | {front=187, left=188, back=57, right=0} |
| RTF（记录窗） | 0.419 | 0.390 | 0.402 |
| /cmd_vel 条数 / 非零 | {n=497, max_abs_vx_plus_wz=3.000, nonzero=352} | {n=400, max_abs_vx_plus_wz=3.000, nonzero=76} | {n=315, max_abs_vx_plus_wz=3.000, nonzero=44} |
| /plan 帧数 / 每条位姿数(中位) | {n_frames=17, pts_median=66.000} | {n_frames=10, pts_median=62.000} | {n_frames=9, pts_median=64.000} |
| --drive 漂移 vs 真值(m) / yaw(度) | — | — | — |
| --goal 真值位移(m) / 目标后残余(m) | 0.1388 / 1.3598 | 0.0015 / 1.5069 | 0.0015 / 1.4931 |
| 每帧点数（中位） | 原始云=11972.0, /scan 输入(obstacle)=6245.0, /segmentation/ground=5733.0, /cloud_registered=11980.0, voxel_grid=781.5 | 原始云=11958.0, /scan 输入(obstacle)=6226.0, /segmentation/ground=5720.0, /cloud_registered=11960.5, voxel_grid=161.0 | 原始云=11963.0, /scan 输入(obstacle)=6231.0, /segmentation/ground=5724.0, /cloud_registered=11971.0, voxel_grid=773.0 |

**表 C：三档 + 固定动作**（`--drive`：直行 10 s `vx=0.30` ↔ 原地转 10 s `wz=0.60`；`.tmp_tiltmount/j2_*_drive`）

| 量 | plugin-drive | urdf-drive | sensor-drive |
|---|---|---|---|
| 安装档 variant | plugin | urdf | sensor |
| TF base_link→livox_frame rpy(度) | [0.000, -0.000, 0.000] | [-30.000, 0.000, 0.000] | [-30.000, 0.000, 0.000] |
| TF odom→base_link rpy(度) | [0.107, 0.429, -0.111] | [30.036, 0.371, -0.297] | [30.310, 0.324, 0.526] |
| TF odom→base_link xyz(m) | [0.389, 0.225, -0.094] | [0.386, 0.142, -0.190] | [0.397, 0.089, -0.086] |
| 原始云 frame_id | livox_frame | livox_frame | livox_frame |
| 原始云地面法向 vs **自己的 frame_id** z(度) | 0.893 | 0.893 | 29.215 |
| 原始云地面法向 vs **base_link** z(度)（账对不对） | 0.893 | 30.790 | 0.893 |
| 原始云地面平面高(m) | 0.255 | 0.255 | 0.255 |
| /cloud_registered frame_id | odom | odom | odom |
| /cloud_registered 地面倾角（odom 里，度） | 1.027 | 0.957 | 29.287 |
| /cloud_registered 低于水平面点数 | 6768 | 6770 | 370 |
| odom 帧相对真实重力倾角(度) | 0.174 | 30.046 | 30.264 |
| /odom 窗口内 rpy 跨度(度) | [0.192, 0.547, 14.293] | [0.227, 0.456, 14.958] | [0.803, 7.603, 12.822] |
| /scan frame_id | livox_frame | livox_frame | base_link |
| /scan 有限波束/帧 | 949 | 949 | 755 |
| /scan 波束在 odom 里超出 [0,2] m 的比例 | 0.000 | 0.031 | 0.396 |
| /scan 波束 z(odom) min / max (m) | 0.040 / 0.076 | -0.012 / 0.020 | -0.464 / 2.464 |
| local costmap frame_id | odom | odom | odom |
| local costmap lethal / inscribed / free | 420 / 16297 / 32964 | 364 / 16640 / 32548 | 443 / 16447 / 33895 |
| 车那格的值 | 91 | 93 | 95 |
| 到最近 lethal 格距离(m) | 0.379 | 0.341 | 0.321 |
| 车半径圆内格数 / ≥99 / free | 999 / 560 / 1 | 998 / 554 / 0 | 996 / 606 / 0 |
| lethal 方位（前后左右） | {front=57, left=216, back=147, right=0} | {front=95, left=102, back=167, right=0} | {front=124, left=164, back=155, right=0} |
| RTF（记录窗） | 0.390 | 0.392 | 0.396 |
| /cmd_vel 条数 / 非零 | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} | {n=0, max_abs_vx_plus_wz=0.000, nonzero=0} |
| /plan 帧数 / 每条位姿数(中位) | {n_frames=0, pts_median=—} | {n_frames=0, pts_median=—} | {n_frames=0, pts_median=—} |
| --drive 漂移 vs 真值(m) / yaw(度) | 0.0198 / -0.441 | 0.0225 / -0.518 | 0.0517 / 0.041 |
| --goal 真值位移(m) / 目标后残余(m) | — | — | — |
| 每帧点数（中位） | 原始云=12360.0, /scan 输入(obstacle)=7161.0, /segmentation/ground=5278.0, /cloud_registered=12392.0, voxel_grid=670.0 | 原始云=12370.0, /scan 输入(obstacle)=7150.0, /segmentation/ground=5282.0, /cloud_registered=12391.0, voxel_grid=122.0 | 原始云=12332.0, /scan 输入(obstacle)=7098.5, /segmentation/ground=5321.0, /cloud_registered=12364.0, voxel_grid=661.0 |

> ⚠️ **读表两个坑（都是"rpy 分量会骗人"的老问题）**：
> **(a)** `sensor` 档 `TF odom→base_link` 的 rpy 里有一个 **−4.9° 的 pitch**、`/odom` 窗口内 pitch 跨度
> **5.17°** —— 它**不是** bug ③ 那个假俯仰。那一档的 odom **真的斜了 30°**（表里
> `odom 帧相对真实重力倾角 = 30.359°`），而"绕斜帧的 z 轴转 yaw"用 ZYX 展开**必然**出现 pitch 项；
> 矩阵本身是对的（`R_ob == R_x(30)·Rz(Δ)` 与 `R_ob·R_bl⁻¹ == R_ol` 两条恒等式见 §J.3 的单测与 §J.4.1）。
> **判据要用矩阵/一致性，不要用 rpy 分量。**
> **(b)** `urdf` 档的 `odom 帧相对真实重力倾角 30.116°` 也**不是**"odom 真的斜了" —— 那一档点云是水平的、
> odom 其实是重力对齐的；这个 30° 是**发布出去的 TF 自带的 roll**（§I.2 的"账不对"）。⇒ 它在表里要
> **按"账"读**（读作"nav2 以为车斜了 30°"），不要按"物理"读。

#### J.4.1 逐级坐标系自检（`tools/scripts/tiltmount/tilt_chain_check.py`）

| 档 | raw 点数 | 地面法向 vs **自己 frame_id** z | 经 TF 转到 base_link 后 | frame_id | TF base_link←livox rpy | 自击掩膜命中（旋转后） | p2l 高度带内（旋转后） |
|---|---|---|---|---|---|---|---|
| plugin(改前) | 11568 | 1.004° | 1.004° | `livox_frame` | [0.0, -0.0, 0.0] | 29.210% | 1.0000 |
    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：实测发布 = 0.918° ；H_fixed(现在的代码) = 0.782° ；H_old(旧代码) = 0.782°（反解 T_ol(=LIO 状态) rpy = [0.217, 0.047, 9.667]；两条假设的平移差 = |t| = 0.2044 m）
| plugin(改后) | 11568 | 1.004° | 1.004° | `livox_frame` | [0.0, -0.0, 0.0] | 29.210% | 1.0000 |
    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：实测发布 = 0.834° ；H_fixed(现在的代码) = 0.730° ；H_old(旧代码) = 0.730°（反解 T_ol(=LIO 状态) rpy = [0.259, 0.404, 9.503]；两条假设的平移差 = |t| = 0.2044 m）
| urdf(改前) | 11568 | 1.004° | 30.974° | `livox_frame` | [-29.99999999999969, 0.0, 0.0] | 29.210% | 0.8778 |
    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：实测发布 = 30.911° ；H_fixed(现在的代码) = 30.900° ；H_old(旧代码) = 60.900°（反解 T_ol(=LIO 状态) rpy = [-29.617, 4.89, 7.701]；两条假设的平移差 = |t| = 0.2044 m）
| urdf(改后) | 11568 | 1.004° | 30.974° | `livox_frame` | [-29.99999999999969, 0.0, 0.0] | 29.210% | 0.8778 |
    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：实测发布 = 0.965° ；H_fixed(现在的代码) = 0.915° ；H_old(旧代码) = 30.912°（反解 T_ol(=LIO 状态) rpy = [0.06, 0.317, 9.162]；两条假设的平移差 = |t| = 0.2044 m）
| sensor(新) | 11568 | 29.028° | 1.004° | `livox_frame` | [-29.99999999999969, 0.0, 0.0] | 29.210% | 1.0000 |
    · bug ② 判决性判据（同一次跑的 raw 云按两种发布假设预测**地面法向角**）：实测发布 = 29.145° ；H_fixed(现在的代码) = 29.400° ；H_old(旧代码) = 0.693°（反解 T_ol(=LIO 状态) rpy = [-0.043, -4.874, 8.52]；两条假设的平移差 = |t| = 0.2044 m）

### J.5 ⚠️ bug ③ 的**平移**：为什么本次**保留旧值**（实测 + 完整修法 + 后续项）

**结论先说**：bug ③ 有**两半** —— **姿态**（用户实测到的那个 bug：假俯仰 4.890°）**已修**；
**平移**（`t_ol + (I − R_ol)·p` vs 物理真值 `t_ol − R_ol·p`，差**恰好 −p = 0.2044 m**）
**本次故意保留旧值**，因为它牵动的是**另一份目录**里的东西，而且实测代价很明确：

| 口径（`robot:=robot11` 或默认模型，静止，`lio:=small_point_lio`） | 平移 = **旧值**（今天 / 本次修完） | 平移 = **物理真值**（`T_ol·T_bl` 的平移） |
|---|---|---|
| `/scan` 盘面在 odom 里的 z（p50） | **+0.063 m**（robot11）/ +0.065（§I 那轮） | **−0.091 m**（robot11）/ **−0.092 m**（默认模型） |
| 落在 `obstacle_layer` 高度带（nav2 默认 `min_obstacle_height 0.0`，量在**代价图帧 odom**）之外的波束 | **0 / 948** | **950 / 950**（robot11）、**1239 / 1239**（默认模型） |
| 局部代价图 lethal / inscribed | **461 / 16937**（改前 §I 那轮 471 / 15909） | **0 / 0**（整张图 62500 格全 free） |
| 默认模型对照 | **445 / 11029**（§D.2 的 regress 探针） | **0 / 0**（同一探针，`j1_default`） |

**机理**（三句话，都有源码/参数出处）：
1. `/scan` 是一张**二维平盘**，盘面过它自己的帧原点（`plugin`/`urdf` 档 = `livox_frame`）⇒ 它在 odom 里的
   z 就等于"那一帧的 odom z"；nav2 的 `laserScanCallback` 先 `projectLaser`（z=0）再用 TF 搬到代价图帧，
   带子是在**代价图帧**里量的（`obstacle_layer.scan` **没有**配 `min_obstacle_height`
   ⇒ nav2 默认 **0.0**；`src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml:128`
   `global_frame: odom`，同文件 `:187` 的注释写明设计假设："odom 的 z=0 ≈ base_link 起始高度（地面约 −0.05）"）。
2. 本仓的 odom **不满足**这条假设：`rm_simulation.launch.py` 把车生成在 `z=0.2`（轮半径 0.06、落定后
   `base_link` 在 0.11~0.15），**LIO 在坠落中就做了重力初始化**（那句注释是明写的）⇒ odom 的 z=0 比
   "落定后的地面基准"高 **0.2~0.35 m**。旧的共轭平移**恰好**把这 0.2044 m 的一部分补了回去
   （"歪打正着"），所以今天的默认路径"能用"。
3. ⇒ 任何"只改 LIO 一行"的方案（无论是本文件的合成，还是把 odom 原点挪到地面）都会**同时**打断
   robot11 **与默认模型**的局部代价图；而那条 `min_obstacle_height`（以及代价图帧的选择）在
   **`src/rm_navigation/**/params`（本任务禁改）**。⇒ 必须**分两步**：先修姿态（本提交，默认档逐位不变），
   平移 + 高度带重新定基**登记为后续项**。

**平移的完整修法（后续项，两个候选，都要单独评估）**：
* **(a) 在 LIO 侧**：把 odom 的 z 基准钉到"初始化那一刻的 `base_link` 高度"（即发布
  `T_ob = T_ol·T_bl` 的真值 + 一个**常量** z 偏置，让 `base_link` 起始 ≈ 0）—— 语义上正是
  `nav2_params_sim_base.yaml:187` 那条假设；代价：odom 不再是"纯净的 LIO 输出"，要用参数门控。
* **(b) 在代价图侧**：把 `obstacle_layer.scan.min_obstacle_height`（local 与 global 两处）改成覆盖
  `/scan` 盘面实际落点（本轮实测：odom 里 −0.15 m 即可覆盖 −0.091；留 0.06 m 余量），
  或者把两张图的 `global_frame` 换成重力对齐帧（`sensor` 档下更有必要，见 §J.7 第 3 条）。
  代价：这两处都在**别的任务的参数目录**里，且会改变默认路径的"什么算障碍"口径 ⇒ 必须单独 A/B。
* 本节给出的**可直接复算的量**：`python3 tools/scripts/tiltmount/tilt_ab_table.py --diff before:.tmp_tiltmount/tm_plugin after:.tmp_tiltmount/j2_plugin`
  与 `.tmp_tiltmount/j1_plugin/probe.json`（= 平移取真值的那一跑，保留为反例证据）。


### J.6 「斜装怎么处理」的配方（**现在已经真的实现了**：`robot11_mount:=sensor`）

#### J.6.1 一档到底改了什么（逐级 + 精确参数）

| 级 | 文件 / 键 | `plugin`（默认，今天） | **`sensor`（新，正确链）** |
|---|---|---|---|
| ① 物理安装姿态 | `sentry_robot_robot11_sim.xacro`（生成器 `tools/scripts/regress/robot11_make_sim_xacro.py` 拥有）：`joint body_to_livox` 的 `origin rpy` | `0 0 0` | `$(arg livox_tilt_rpy)` = `-0.523598775598293 0 0`（= 上游/CSV 字面值，**mesh 一起斜**） |
| ② 射线方向 | 插件 SDF `<tilt_rpy>` | `-0.523598775598293 0 0` | `0 0 0`（倾角已由 link 姿态承担；**世界射线两档逐条相同**） |
| ③ 点云表达在哪个系 | 插件 SDF `<cloud_frame>`（**本轮新增的键**） | 元素不存在 ⇒ `parent`（父 link 系） | `<cloud_frame>sensor</cloud_frame>` ⇒ **真·传感器系**（`axis = mount_rot·ray`） |
| ④ 地面分割 | `linefit_ground_segmentation_ros/config/segmentation_sim_robot11_sensor.yaml`（**本轮新增的增量文件**）：`gravity_aligned_frame` | `""`（点云在源头已对齐） | `base_link`（节点先按 TF **只旋转**到 base_link 再分割） |
| ⑤ 分割器其它键 | `segmentation_sim_robot11.yaml`（**不动**）：`sensor_height` | `0.2595` | `0.2595`（"只旋转、不平移"⇒ 地面仍在 z=−0.2595 ✓） |
| ⑥ scan 化 | `pointcloud_to_laserscan/config/laserscan_params_sensor_frame.yaml`（**本轮新增的增量文件**）：`target_frame` | `""`（不做 TF，连 MessageFilter 都不建） | `base_link`（完整 TF，含平移）⇒ `/scan` 是"水平面里的一圈"、高度带恢复"离地"语义 |
| ⑦ p2l 其它键 | `laserscan_params.yaml`（**不动**）：`min/max_height` | `-1.0 / 1.0`（在 `livox_frame` 里量） | `-1.0 / 1.0`（在 `base_link` 里量 ⇒ 变成真正的"相对车体原点的高度"） |
| ⑧ 自击掩膜 | `traversability_self_mask_robot11.yaml`（**不动**，113 个 AABB） | 命中 29.2% | **不用重烘**：掩膜作用在 linefit 内部**旋转之后**的 `cloud_proc` 上，而那一份坐标与 `plugin` 档**逐点相同**（离线实测见 §J.4 的 `tilt_chain_check` 行） |
| ⑨ 杆臂（只在 `lio:=fastlio\|pointlio` 时生效） | launch 的 `_lio_adapter_robot11(...)` | 纯平移 | 带 30° 旋转（`sensor` 与 `urdf` 同处理；`lio:=small_point_lio` 时该节点不启动） |
| ⑩ odom 的重力对齐 | — | odom ≈ 初始身体系（**恰好**水平，因为点云是平的） | odom = **斜 30° 的初始传感器系**（`fix_gravity_direction` + 初始 `R=I`，§I.8 原则 4）⇒ 本档**选择"下游全部在 base_link 里"**（④⑥），odom 的斜由 TF 如实表达、**没有**去改 LIO 的初始化 |

#### J.6.2 复制即可跑

```bash
# 构建（插件 + LIO + 两个新增覆盖文件所在包）
colcon build --symlink-install --packages-select rm_nav_bringup small_point_lio \
    ros2_livox_simulation linefit_ground_segmentation_ros pointcloud_to_laserscan

# 正确链（物理斜装 + 账也对）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0

# 一条命令量齐（隔离无头 + 参数/TF 回读 + 契约 + 逐帧账本）
tools/scripts/tiltmount/run_tilt_mount_probe.sh j2_sensor --variant sensor --settle 25 \
    --duration 45 --frames 3 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gui:=False
```

#### J.6.3 30 秒自检清单（跑起来之后）

```bash
ros2 param get /ground_segmentation  gravity_aligned_frame   # 期望 "base_link"
ros2 param get /pointcloud_to_laserscan target_frame         # 期望 "base_link"
ros2 run tf2_ros tf2_echo base_link livox_frame              # 期望 rpy = -30.000 0 0
python3 tools/scripts/tiltmount/tilt_chain_check.py --dir sensor:.tmp_tiltmount/j2_sensor
#   → "地面法向 vs 自己 frame_id z" ≈ 30°（**数据真的在斜的传感器系里**）
#     "经 TF 转到 base_link 后" ≈ 1°（**账也对**）；自击掩膜命中 ≈ 29.2%（不用重烘）
grep -m1 "cloud_frame = sensor" .tmp_tiltmount/j2_sensor/launch.log   # 插件侧收据
```

### J.7 这条"正确链"**仍然解决不了**什么（诚实清单）

1. **"车一开始就在膨胀团里"没被这一档解决**：它的真因是 §D.3 的三件事（近场地面残留 + 场地低矮件 +
   本槽位 `robot_radius 0.3565 / inflation 0.70|0.75`），与"点云表达在哪个系"无关。A/B 表里的
   `车半径圆内 ≥99 / free` 一列就是这条的量。
2. **自击环仍在**：`r<0.12 m` 的点是**物理真实回波**（28~29%/帧），本档一个都没删；它仍然靠
   `self_mask_*`（判据层）与 `obstacle_min_range` 处理。
3. **代价图那一层仍然是 odom 帧**（§J.5 的同一根因）：`sensor` 档下 odom 斜 30°，`obstacle_layer.scan`
   的高度带把"过 base_link 原点的水平盘"在 odom 里看成斜面 ⇒ **一半以上的波束按高度被丢**
   （§I.7 量过旧 `urdf` 档的 71.4%）。修它要动**代价图帧/高度带**（另一份参数目录）或改 LIO 的
   odom 重力对齐 —— **本档没做**，只在 §J.4 如实登记。
4. **插件 `point = range·axis` 漏了 `+ 0.1·axis`**（§12.3.1 的系统内移 0.1 m）**仍然在**：它属"共享代码、
   会改所有模型"，本轮不动。
5. **`odom` 帧不保证重力对齐**这条**结构性事实**没有变：本档只是"下游全用 base_link"绕开它。

### J.8 回退与未验证

#### 回退

| 想退掉什么 | 怎么做 |
|---|---|
| `sensor` 档 → 今天的行为 | 去掉 `robot11_mount:=sensor`（默认 `plugin`） |
| 两个 LIO bug 修复 | `git revert <J 的 LIO commit>`（默认档**逐位不变**，revert 后与 §I 那一轮等价） |
| 只退"点云表达在传感器系" | 去掉 xacro 里的 `<cloud_frame>`（插件缺省即父 link 系；两档参数文件同时撤掉） |
| 整套 `robot11_mount` | `docs/tilted_lidar_fidelity.md` §C.2/§F.4 |
| 本节新增的工具 | `tools/scripts/tiltmount/`（只有 `tilt_ab_table.py` / `tilt_chain_check.py` 是本轮新增；都不进任何 launch/节点） |

#### 未验证 / 诚实清单（本轮）

1. **没有在真 RViz / 带 GUI 里看一眼**（全部无头跑；§I.10 第 1 项同款限制）。
2. **`sensor` 档的"整条链端到端好用"没有在任何意义上被证明**：本轮只证明了
   **帧 ↔ 数据自洽**（29.028° vs 1.004°）、**下游重力对齐生效**（linefit/p2l 的参数与输出）、
   **契约单发布者**；代价图那一层（odom 帧的高度带）与"车能不能真的走过去"**都还不行**（§J.7 第 1/3 条）。
3. **单出生点、单次跑**：三个档各 1 次静止 + 1 次目标 + 1 次固定动作，**没有**重复跑；
   RTF（0.39~0.46）在同一次批里可比，但**没有**做"同时间窗交替"那种严格 A/B（§G 第 3 项）。
4. **`--drive` 的漂移只有一轮**：口径与本仓惯例一致（odom 位移 − 真值位移），数值 0.02~0.05 m 量级
   ⇒ 与 §H.5 一样**只能当噪声量级**读，不能主张"哪个档漂得多"。
5. **`sensor` 档只测了 `lio:=small_point_lio`**：`fastlio`/`pointlio` 走 `lio_tf_adapter` 那条杆臂
   （本轮只做了几何/单元级验证），**没有**跑过组合。
6. **没有测 `ground:=patchwork` + `sensor`**（launch 里**直接报错**，是设计选择，不是验证过的组合）。
7. **没有测 `robot11_mount:=sensor` 与 `--drive` 的目标/控制耦合**：`/cmd_vel` 的恢复行为日志
   只在 `plugin`/`urdf` 两档采到（`sensor` 档目标跑的日志见 §J.4 表）。
8. **bug ② 的"绕 odom 原点旋转"这一条**（§I.10 第 3 项）在**默认档**已经不存在了（不再有任何额外旋转），
   但在 `urdf` 档的**旧**行为里它是真实的 —— 本轮只做了静态（车在原点附近）复测，**没有**跑"车开远"的对照。
9. **`tilt_chain_check.py` 的 bug ② 判据用"地面法向角"**：在**默认档**（`T_bl` 纯平移）两条假设的
   法向**相同** ⇒ 那一档的判别靠"平移差 = |t| = 0.2044 m"与跨跑对照，**不是**单跑内的角度判别。
10. **§J.5 的替代修法（代价图高度带重新定基）没有实现**：只给了两个候选与实测代价，
    **没有**跑过"改完能不能用"的 A/B。

#### J.4.2 分段读法（六个问题逐一回答）

**(1) 点云的"地面法向 vs 它自称的那个帧的 z"**（§J.4.1 表）：
`plugin` 1.004° / `urdf` **1.004°（但经 TF 到 base_link 变 30.974°）** / `sensor` **29.028°（经 TF 到
base_link 回到 1.004°）**。⇒ 只有 `sensor` 档两个数**同时**说明"数据在斜的传感器系里、账也对"；
`urdf` 档的第 2 列 30.974° 就是 §I.2"帧与数据差 30°"的量化。

**(2) `/segmentation/ground` / `/scan`**：见 A/B 表的两行 + 分带行。要点：
* `urdf` 档（改后）的 `/scan` **不再**被高度带丢掉 71.4%（改前 §I.7 的数）—— 因为 bug ③ 修好后
  `odom←livox_frame` 是真正的传感器位姿（盘面落在 +0.05 m 一带）；
* `sensor` 档的 `/scan` 换成 `frame_id=base_link`、有限波束比 `plugin` 少（`plugin` 档那 950 条里
  有 ~170 条是 0.05–0.10 m 的自击团，`sensor` 档因为做了完整 TF，自击团的水平距离被正确投影）。

**(3) `obstacle_layer` 高度带存活**：`plugin` **0/948 被丢**、`urdf`（改后）**0/949**、
`sensor` **41.2% 被丢** —— `sensor` 档这一条**仍然是坏的**，机理是"A/B 表的代价图帧仍是 odom（斜 30°）
⇒ 过 base_link 原点的水平盘在 odom 里是斜面"（§J.7 第 3 条）。**这是本档唯一没修好的一级。**

**(4) 局部代价图 / 车周围**：三档都采到了 `lethal / inscribed / 车那格 / 车半径圆内 ≥99 / free`
（见 A/B 表）。`plugin` 与 `urdf` 两档的车那格**仍然不是 free**（86 / 84），车半径圆内 `≥99`
**429 / 375** 格 —— 与 §D.2/§H.2 的"车一开始就在膨胀团里"**同量级复现**（这一条与安装档无关，§J.7 第 1 条）。

**(5) 短目标下车动不动**（`--goal-forward 1.5`，同一协议）：
`plugin` **真值位移 0.139 m、目标后仍差 1.36 m**、日志 `detected collision ahead` **139 条**/
`patience exceeded` 3 条；`urdf` **0.0015 m / 1.51 m / 286 条**⇒ **两档都是"目标被接受但车不走"**
（§D.4 的复现）。`sensor` 档见 A/B 表（这一档的代价图来源不同，读数要单独看）。

**(6) LIO 漂移与 RTF**（`--drive` 固定动作：直行 10 s `vx=0.30` ↔ 原地转 10 s `wz=0.60`，同一协议）：
见 A/B 表的最后两行 —— 口径 = "odom 位移 − 真值位移"（本仓惯例），**与 §H.5 同量级（0.01~0.05 m）**，
RTF 三档在 **0.39~0.46** 之间 ⇒ **不要**据此主张"哪个档更慢/更漂"（§G 第 3 项）。

---

## K. 2026-10-09：代价图侧的收尾（高度带定基 / 近地剔除 / `robot_radius`・足印 / 插件 0.1 m 偏移）

> 触发 = 父任务原文：**"`robot:=robot11` 端到端可用的最后一个阻塞项 —— §J 留下的代价图侧尾巴"**。
> §J 明确登记了三件"本档没做/做不到"的事（§J.5 的高度带定基、§J.7 第 1 条的"车一开始就在膨胀团里"、
> §J.7 第 4 条的插件 `point = range·axis` 漏了 `0.1 m`）。本节就是把这三件做完（或证明为什么不做）。
>
> 与 §I/§J 的关系：**§I 是诊断、§J 是"把账做对"（帧/数据自洽）、§K 是"让代价图这一层也能用"**。
> §I/§J 的所有内容**逐字保留**；本节只**追加**，并在 §K.1 里登记一处对 §J.5 的**量化更正**
> （"40.4%"这一条我复现到 39.6%，但**"整张代价图变空"这一条在本工作区复现不出来**，见 §K.1.3）。
>
> 全部无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、
> `unset DISPLAY`、收尾只 kill 本调用自己的 PID，**绝不做全机 pkill**），
> 原始数据 `.tmp_tiltmount/k1_*`、`.tmp_robotslot/k3_*`、`.tmp_robotslot/k4_*`、`.tmp_robotslot/k5_*`、
> `.tmp_tiltmount/p5_*`；工具见 §K.7。

### K.0 一句话结论（四条，都是实测）

1. **2D 那条链路（`/scan`→`obstacle_layer`）在结构上量不出"逐点高度"**：`/scan` 是**二维平盘**
   （LaserScan 没有 z），nav2 的 `obstacle_layer` 先 `projectLaser`（**z 强行置 0**）再
   `transformLaserScanToPointCloud` 搬到代价图帧 ⇒ **每条波束在代价图帧里的 z 恒等于"那一帧
   点云原点的 odom z"**。实测（robot11 plugin 档、静止）：**950 条波束的 odom z 全部落在
   [0.0564, 0.0815]、`min_obstacle_height 0.0` 一条都不丢**（§K.1.2）。
   ⇒ `min_obstacle_height` 在 2D 链路上只是**"传感器自身高度"的常量闸**，不是"障碍物高度闸"。
   这就是 §D.3 那条归因（"近场地面残留经 `p2l` 的 2D 投影被标在雷达高度上"）的**结构原因**。
2. **要让"贴地点"不变成 lethal 格，只能在"投影成 2D 之前"动手** ⇒ 新增一级
   **近地剔除** `obstacle_near_ground_m`（离**局部地面** ≤ 该值的点从 `/segmentation/obstacle`
   **改判 ground**；`robot:=robot11` 槽位 = **0.05 m**，默认槽位 = **关**）。
   实测（robot11、静止、近地剔除开/关逐字同协议）：车那格 **42 → 0**、
   车半径圆内 `≥99` **156 → 0**、圆内 free **239 → 999**、
   到最近 lethal 格 **0.531 → 1.534 m**；`/scan` 只少了 **53 条/帧**（全是 0.30–0.50 m 的贴地回波），
   `/segmentation/obstacle` 少 **315 点/帧**（全转到 ground）。
3. **`robot_radius 0.3565`（外接）改成 `0.300`（内切）后，"车一开始就在膨胀团里"彻底消失**：
   干净静止三档 A/B（近地剔除都开、唯一变量 = 半径/膨胀）：
   `0.3565/0.70` → 车那格 **0**、圆内 `≥99` **0**、free **723/1008**；
   `0.300/0.60` → 车那格 **0**、圆内 `≥99` **0**、free **871/1005**；
   `0.22/0.50` → 车那格 **0**、圆内 `≥99` **0**、free **979/1007**。
   三者**局部**都已经是"车那格 free + 圆内 0 个 ≥99"；**全局**图只有 `0.300` 那一档在静止时
   车那格 = 0（`0.3565` 档是 99、`0.22` 档是 77）——原因见 §K.3.3（全局图还在建、STVL 的
   时间维 + 地图原点附近还没清干净）。⇒ **本主题把槽位值改成 `0.300 / 0.60 / 0.65`**（理由见 §K.3）。
4. **插件 `point = range·axis` 漏了射线起点（`0.1 m`）已修**，并给出回退开关
   `<range_from_origin>false</range_from_origin>`（默认 = 修好的语义）。跨模型 A/B 见 §K.4。

### K.1 阻塞项 1：2D 高度带（`obstacle_layer.scan`）的定基 —— 结论是"在 2D 链路上定不了基"

#### K.1.1 源码级机理（三行，可核对）

1. `nav2_costmap_2d` 的 `laserScanCallback` 走 `projector_.transformLaserScanToPointCloud(header.frame_id, msg, cloud, tf)`
   —— `laser_geometry` 的 `projectLaser` 对 **LaserScan** 只能产出 **z ≡ 0** 的点（LaserScan 里没有高度），
   随后那次 TF 把 `(r·cosθ, r·sinθ, 0)` 整体搬到**代价图帧**。
2. `obstacle_layer.cpp` 的标记循环在**代价图帧**里量高度（本仓 `third_party/nav2/.../obstacle_layer.cpp:470,476`）：
   ```cpp
   if (pz < min_obstacle_height_) { continue; }   // 本仓 local `obstacle_layer.scan` **没有**这个键
   if (pz > max_obstacle_height_) { continue; }   // 本仓 = 2.0
   ```
   `min_obstacle_height` 的 nav2 默认是 **0.0**（`nav2_params_sim_base.yaml` 里根本没有这个键 ⇒ 走默认）
   —— 这一点由运行期回读确认（`/local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height = 0.0`）。
3. 合起来：**每条波束在代价图帧里的 z = 那一帧"点云原点"的 odom z**（一个常量），
   与它打到的三维点**无关**。⇒ 2D 这条链路上**没有逐点高度可判**，
   `min_obstacle_height` 只能当"传感器自身高度"的常量闸用。

#### K.1.2 判决性实测（robot11、`robot11_mount:=plugin`、静止）

`.tmp_tiltmount/p5_base_plugin/probe.json`（探针把每条波束用 TF 搬到 odom 再量）：

| 量 | 实测 |
|---|---|
| `/scan` 有限波束/帧 | **950** |
| 波束在 odom 里的 z：min / p05 / p50 / p95 / max | **+0.0564 / +0.0637 / +0.0652 / +0.0807 / +0.0815 m** |
| 落在 `[0, 2]` 之外的波束 | **0 / 950 = 0.0000** |
| 其中落在 `[0, min+0.005]` 这个 5 mm 薄层里的 | 全部（z 的跨度只有 **25 mm**） |
| 同一跑 TF `odom→livox_frame` 的平移 z | **+0.064 m**（与上面 p50 吻合到 1 mm） |

⇒ **波束的 odom z 就是"传感器原点的 odom z"**，一字不差（25 mm 的跨度来自车体在
记录窗内的轻微俯仰/升降）。**"高度带按几何重新定基"在 2D 链路上没有可定的对象**：
把 `min_obstacle_height` 从 0.0 改到任何非 0 值，效果都只是"把整条平盘一起抬高/压低"。

#### K.1.3 对 §J.5 的两处**量化更正**（同一条命令、独立复跑）

| 量 | §J.5 记的值 | **本次复跑（`p5_base_sensor`）** | 判读 |
|---|---|---|---|
| `/scan` 波束在 odom 里超出 `[0,2]` 的比例（`sensor` 档） | **0.404** | **0.3960**（269/755 落在 0 以下） | **复现**（差 0.8 个百分点） |
| 局部代价图 lethal / inscribed（`sensor` 档） | **0 / 0**（"整张图 62500 格全 free"） | **495 / 16511**（free 32480） | ⚠️ **复现不出来** |
| 车那格 / 车半径圆内 `≥99` / free（`sensor` 档） | 未记 | **88 / 511 / 4** | 与 §J.4 表 A 的 sensor 列（86 / 509 / 4）**逐项一致** |
| 契约（9 个关键话题的发布者数） | 1 | **逐个 = 1** | 一致 |

**为什么"整张图变空"复现不出来**：§J.5 那张表的 "0 / 0" 来自 `.tmp_tiltmount/j1_default` /
`j1_plugin` 两跑（= **把 bug ③ 的平移也改成物理真值**的那一版）。本次工作区的代码是 §J 最终
提交（平移**故意保留旧值**，§J.5 的结论），所以 `sensor` 档的盘面落在 odom z 的
**p50 = +0.133 m**、**26.9% 在 [0,2] 内** ⇒ 高度带仍然留下一部分波束 ⇒ 图不会空。
⇒ **"图变空"是"平移取真值"那一版的后果，不是 `sensor` 档本身的后果**；本节据此把
§J.5 那一行读作"**如果**平移也改成真值 ⇒ 会空"（§J.5 的原文其实也是这个意思，
但表里与"现状 `sensor` 档"并排放在一起，容易被读成后者）——**这里把口径钉死**。

#### K.1.4 那还要不要动高度带？——**本主题不动**，并把"为什么"写进参数文件

* 对 **`plugin` 档（默认档）**：波束 odom z ∈ [0.056, 0.082]，`[0, 2]` 全收 ⇒ **现状已经正确**，
  改任何值都只会变坏。
* 对 **`sensor` 档**：波束 z 跨度 **−0.394 … +2.361 m**（一盘"绕 base_link 原点的水平面"
  在**斜 30° 的 odom** 里就是斜面）⇒ 单靠一条 `[lo, hi]` **永远**只能救回"半个环"。
  要真修得改代价图帧（全局 `map` 也仍是 `odom`）或改 LIO 的 odom 重力对齐 —— 两者都超出本主题，
  且会把**默认模型**一起拖下水（§J.5 实测代价：`j1_default` 的 lethal 445 → 0）。
  **本主题的替代修法**是把 2D 链路上的"贴地点"在**投影之前**剔掉（§K.2），它对**两档都成立**、
  且不依赖"odom 的 z 是什么意思"。
* ⇒ 结论（写进 `nav2_params_sim_robot11_costmap.yaml` 的注释）：**`min/max_obstacle_height` 保持
  0.0 / 2.0**，并把"这两个键在 2D 链路上的真实语义 = 点云原点的代价图帧 z 的常量闸"写在那里，
  免得下一个人再按"障碍物高度"去调它。

> **顺带量到一件与本节结论无关、但必须登记的事（sensor_height 的口径）**：
> 我用 k7 跑（robot11、plugin 档、静止）的原始云做了 z 直方图，**地面峰在 z ≈ −0.155 m**
> （最低一半的中位 = **−0.1502**）；同跑的 TF 是 `odom→livox_frame` z = **+0.058**、
> `odom→base_link` z = **−0.097**。⇒ 反推**传感器离地 ≈ 0.302 m**
> （= 0.1502 + livox 在 base_link 上的 0.157 − 一点姿态修正），
> 而 `robot11_geometry.json` 的**几何**预测是 **0.2595 m**
> （= 0.157028 + 0.102499，其中 0.102499 来自"四个轮 mesh 最低点"）。
> **两者差 ~4.3 cm**，成因（轮 collision mesh 的最低点 vs 关节 origin 的 FK）**本轮没查**，
> 登记在 §K.8 第 6 项。它**不影响**本节任何结论：
> · §K.1 的高度带论证只用"波束的 odom z = 点云原点的 odom z"这一条恒等式（与 h 无关）；
> · §K.2 的近地剔除判据用的是 `dz = z − 局部地面`（**同一个量在同一个帧里相减** ⇒ 与 h 无关），
>   实测 `dz` 在水平 0.30–0.40 的点上 = **+0.0035/+0.0057/+0.0095**（p05/p50/p95）、
>   0.40–0.50 = **+0.0018/+0.0089/+0.0125** ⇒ 0.05 m 的闸有 **>2×** 余量。
> ⚠️ 但 `linefit` 的 `sensor_height: 0.2595`（那份文件属于别的任务）**确实是按几何值配的** ——
> 若那 0.2595 真的偏了 4 cm，地面线会被整体抬高 4 cm，这在 0.05 m 的 `max_dist_to_line`
> 上是同一量级。**登记为待查**，不改（本主题禁改 `rm_perception/**/config` 里的槽位文件）。

### K.2 阻塞项 2："车一开始就在膨胀团里" —— 近地剔除 + 半径/几何（两件独立的事）

#### K.2.1 先给"贴地点"一个判据（在投影成 2D **之前**）

新增一级（**默认关**、按槽位开）：

| 项 | 内容 |
|---|---|
| 参数 | `obstacle_near_ground_m`（米；**0.0 = 关**）。`robot:=robot11` 槽位 = **0.05**（§K.2.4）；默认/其它槽位 = **0.0** |
| 谁读它 | `rm_ground_traversability`（两个地面分割节点共用的那一层）。参数文件按 robot 槽位选：`config/traversability_near_ground.yaml`（0.0）/ `..._robot11.yaml`（0.05），机制与 `traversability_self_mask*.yaml` **同款** |
| 语义 | 用**本判据本来就有**的局部地面 `g(x,y)`（同一份公式：0.20 m 粗格 → 4×4 个 0.05 m 细格"最低点"的 p05）算 `dz = z − g`；`dz ≤ obstacle_near_ground_m` 的点由**地面分割节点**从 `/segmentation/obstacle` **改判成 ground** |
| 为什么必须在这一级 | `p2l` 输出的是 **LaserScan（没有高度）**，nav2 侧又 `projectLaser`（z 置 0）⇒ 一旦跨过 `p2l`，"这个回波是地面还是障碍"这个信息**永久丢失**（§K.1）。⇒ 唯一能在"投影之前"用上高度的地方就是这里 |
| 与 `self_mask` 的区别 | `self_mask` = "机器人自己的 collision 几何 / 近场死区"里的点（**与地面无关**）；本键 = "确实贴着地面"的点（**与车体几何无关**）。两者可同时开（robot11 就是） |
| 与台阶判据的关系 | **结构上互斥、且不影响台阶**：台阶闸 `step_height_threshold = 0.15 m` ⇒ 任何被判成"台阶/边沿"的点 `dz > 0.15 > 0.05` ⇒ 不可能被本键剔掉。**回放台实测（§K.2.2 最后一行）**：`step_edge` 中位点数在 `near_ground_m = 0.0 / 0.05 / 0.10` 三档下**逐字相同（2011）** |

**实现要点（一行一句，都在 §K.7 的文件里）**：
* 判据类（`low_terrain_classifier.hpp`）只**多填一个掩码** `near_ground_flags`，**不改**建格/分类/限速任何一步；
  它原有的"只降不升"（ground→obstacle）语义**不变**。
* "升"（obstacle→ground）**显式写在两个节点里**（`ground_segmentation_node.cc`、
  `patchwork_ground_segmentation_node.cc`），并且**只在 `nearGroundEnabled()` 为真时**执行
  ⇒ 默认路径（0.0）连掩码都不填，行为**逐字节不变**。
* `~/traversability_stats` 多两个字段（`near_ground` / `near_ground_m`）——是**私有**诊断话题，
  不参与 `/segmentation/*` 契约。

#### K.2.2 离线回放 A/B（**同一帧**、唯一变量 = 阈值）

`tools/scripts/tiltmount/near_ground_replay_probe.py` + `_replay_ng.sh`：
把 k7 跑的一帧**真点云**（11568 点）回放给**活的** `ground_segmentation_node`，
只换 `obstacle_near_ground_m`：

| `obstacle_near_ground_m` | `/segmentation/ground` 中位 | `/segmentation/obstacle` 中位 | ground+obstacle | **`step_edge` 中位** |
|---|---|---|---|---|
| **0.00（关）** | 4683 | 6885 | **11568** | **2011** |
| **0.05（robot11 取值）** | **5985** | **5583** | **11568** | **2011** |
| 0.10 | 7286 | 4282 | **11568** | **2011** |

读法：**同一帧上**，0.05 把 **1302 点**从 obstacle 改判成 ground（= 0.35 m 环那一圈的贴地回波），
**总点数不变**（不丢点），而 **`step_edge` 一个点都不变** ⇒ "台阶仍然是障碍"在**离线同帧**上已经是构造性的。

#### K.2.3 在线 A/B（真跑：同一命令、唯一变量 = 阈值）

`.tmp_robotslot/k3_ngoff_geo1` vs `.tmp_robotslot/k3_ngon_geo1`
（`robot:=robot11`、plugin 档、静止、近地剔除 0.0 / 0.05，两次都用 `--goal-forward 1.0`）：

| 量（局部代价图） | 近地剔除 **关**（0.0） | 近地剔除 **开**（0.05） |
|---|---|---|
| `/segmentation/obstacle` 中位点/帧 | 6178.5 | **5851** |
| `/segmentation/ground` 中位点/帧 | 5754 | **6199** |
| `/scan` 有限波束/帧 | 784 | 775 |
| lethal / inscribed / free（整张图） | 324 / 13710 / 36737 | 501 / 12361 / 38488 |
| **车那格的值** | **42（非 free）** | **0（FREE）** |
| 车半径圆内格数 / `≥99` / free（半径 0.3565） | 992 / **156** / **239** | 999 / **0** / **999** |
| 到最近 lethal 格（车心） | **0.531 m** | **1.534 m** |
| 到最近 `≥99` 格 | 0.175 m | 1.178 m |
| 径向剖面（0–0.3565 m 内 `≥99` 格数） | 0 / 8 / 32 / 43 / 73（0.15→0.36 m 各环） | **全 0** |

⇒ **"车那格非 free + 整个足迹圆里几乎没有 free 格"这一条，在近地剔除打开后消失**
（0.175 m 那个 `≥99` 环就是 §K.1 里那条"贴地点被投成 lethal 格"的直接后果）。

#### K.2.4 那 0.05 m 是怎么定的（三个数，都是实测）

| 量 | 实测 | 与 0.05 m 的关系 |
|---|---|---|
| 近场地面环（水平 0.30–0.40 m，220 点/帧）的 `dz` p05/p50/p95 | **+0.0035 / +0.0057 / +0.0095** | **100% ≤ 0.05**（余量 >5×） |
| 水平 0.40–0.50 m（630 点）的 `dz` | **+0.0018 / +0.0089 / +0.0125** | **100% ≤ 0.05** |
| 真障碍（场地 ~0.15 m 低矮件）在水平 0.50–0.75 m 的 `dz` p95 | **+0.1408** | 只剔掉贴地那一层（`frac dz≤0.05 = 0.775`，即剔 77.5% 的点，**留下的正是 0.10–0.17 m 的本体**） |
| 0.75–1.00 m 的 `dz` p95 | **+0.1725** | 同上（`frac dz≤0.05 = 0.781`） |
| 台阶闸 `step_height_threshold` | **0.15 m** | **> 0.05** ⇒ 台阶点结构上不可能被剔（§K.2.2 的 `step_edge` 逐字相同是它的在线证据） |

保守方向：**只少标障碍**（代价：可能漏掉一个 <5 cm 的矮物），不会多标 ⇒ 不会凭空断路。
`dz` 是"同一个量在同一个帧里相减"⇒ 与 `sensor_height` 标定、与 odom 的原点都无关（§K.1 末尾那条登记）。

### K.3 `robot_radius` / footprint 的决定

#### K.3.1 干净静止三档 A/B（近地剔除都开、**唯一变量 = 半径/膨胀**；`.tmp_robotslot/k5_geov{1,2,4}`）

| 量（静止、同一世界/出生点/命令） | ① **外接 0.3565 + 0.70/0.75**（Phase 3 旧值） | ② **内切 0.300 + 0.60/0.65**（**本次采用**） | ③ 0.22 + 0.50/0.55（默认模型那组，只为分离变量） |
|---|---|---|---|
| `/segmentation/{obstacle,ground}` 中位点/帧 | 5810.5 / 6125 | 5810 / 6125 | 5814 / 6119 |
| **局部** lethal / inscribed / free | 224 / 10117 / 40240 | 227 / 8668 / 43825 | 231 / 6316 / 47147 |
| **局部** 车那格 | **0（FREE）** | **0（FREE）** | **0（FREE）** |
| **局部** 车半径圆内 / `≥99` / free | 1008 / **0** / 723 | 1005 / **0** / 871 | 1007 / **0** / 979 |
| **局部** 到最近 lethal / `≥99` | 0.801 / 0.445 m | 0.802 / 0.505 m | 0.803 / 0.580 m |
| 局部径向剖面 0–0.3565 m 的 `≥99` 格 | **全 0** | **全 0** | **全 0** |
| **全局** 车那格 | **99** | **0（FREE）** | **77** |
| **全局** 车半径圆内 `≥99` / free | **137 / 0** | **0 / 123** | 52 / 0 |
| 全局 到最近 lethal | 0.135 m | **0.804 m** | 0.238 m |

**读法（三条）**：
1. **"车那格非 free"在局部图上已经被§K.2 的近地剔除解决**（三档都是 0，圆内 `≥99` 都是 0）
   ⇒ 它**不是**半径造成的（这条纠正了 §D.3 的归因顺序：**先**是贴地点被标成 lethal 格，
   **再**被大 `inscribed_radius` 放大成"整个足迹非 free"）。
2. 但**半径仍然要改**：在**全局**图上（STVL 3D 那条路，图还在建），外接 0.3565 档在静止时
   就给车那格 **99**、圆内 **0 格 free** —— 这正是"车一开始就在膨胀团里"在全局图上的形态
   （全局图上离车最近的 lethal 只有 **0.135 m**：那不是感知噪声，是本仓**局部图 raytrace
   还没清到**的残留 + 地图原点附近的建筑）。改成内切 0.300 后同一时刻车那格 **0**、圆内 **123 free**。
3. ③ 那一档（0.22，= 默认模型半径）**更好一点**，但 0.22 < 真车内切半径 0.300
   ⇒ 会**允许规划器把车带进"真车过不去"的缝**（0.22 是默认模型的几何，不是本车的）。
   ⇒ 取 **②（0.300 / 0.60 / 0.65）**：等于"**能走多窄**"的几何下界，且把"软带"保持在
   默认模型同量级（0.30 / 0.35 m）。

#### K.3.2 采用的数值与理由（写进 `nav2_params_sim_robot11_costmap.yaml` 的注释）

| 键 | 旧值（Phase 3） | **新值** | 理由 |
|---|---|---|---|
| local/global `robot_radius` | 0.3565（外接） | **0.300（内切）** | 本仓没有 footprint 多边形 ⇒ 它就是 `inscribed_radius`；用外接值会把"车那格"在**全局**图上判成 99（§K.3.1），而这个判据的物理含义是"**能走多窄**"⇒ 应当用内切 |
| local `inflation_layer.inflation_radius` | 0.70 | **0.60** | ≥ inscribed（0.300）；软带 0.30 m，与默认模型（0.5 − 0.22 = 0.28 m）同量级 |
| global 同上 | 0.75 | **0.65** | 软带 0.35 m（默认模型 0.55 − 0.22 = 0.33 m） |
| `cost_scaling_factor` | 不动 | 不动 | 与半径无关（默认模型 3.0/2.5） |

#### K.3.3 折中（**必须知道**）：内切圆会让四个角在转弯时"扫到"障碍

底盘凸包是"600×600 底板切角"，**角点在 0.3565 m**、边到原点最近 **0.300 m**。
用 0.300 的圆代替它 ⇒ 站姿没问题，**但转弯时角可能扫过 0.300–0.3565 m 这一圈里的障碍**。
本仓的兜底：① 场地上没有比这更贴的窄缝（过不去的缝本来也不该过）；
② `inflation_radius` 的软带（0.30 m）仍在 ⇒ 撞上前先掉速；③ 前瞻限速与碰撞监控仍在链路上。
**要彻底消掉这个折中 ⇒ 用真足印多边形**，见下。

#### K.3.4 真足印多边形：**本主题试了，没采用**（两个实测原因）

* **原因 1（nav2/`rcl` 的口径）**：`nav2_costmap_2d` 的 `footprint` 参数声明成 **`std::string`**
  （`nav2_costmap_2d/src/costmap_2d_ros.cpp:79`）⇒ 它期待 `"[[x,y],[x,y],...]"` 这种**字符串**。
  本仓的 `_Nav2ParamsForSlot` 是**运行时深合并 YAML**，写嵌套序列会被 `rcl` 当成
  **`double_array` / 更糟：把序列当 key** ⇒ 实测（`.tmp_robotslot/k3_ngon_geo3`）：
  ```
  [ERROR] [rcl]: Failed to parse global arguments
  Couldn't parse params file: '--params-file /tmp/tmpXXXX'. Error:
  Sequences cannot be key at line 127, at ./src/parse.c:816
  ```
  **所有** nav2 节点（controller/planner/behavior/velocity_smoother/waypoint_follower）
  一起 exit ⇒ 整条导航链起不来。（正确写法必须是把多边形**转成一个字符串**再写进 YAML。）
* **原因 2（量到的收益不明显）**：内切圆那一档在**局部**图上已经做到"车那格 free + 圆内 0 个 ≥99
  + 0–0.3565 m 内没有 `≥99` 格"；而**全局**图那一档（0.300）也已经把车那格变成 0。
  ⇒ 换成多边形能多拿到的只有"转弯时角不扫障碍"这一条（真实但难量化），
  代价是"多一个必须与 `robot11_description` 的碰撞 mesh 同步的几何真源"。
  **本主题把这条登记为后续项**（做法：`robot11_make_collision_assets.py` 的凸包顶点 →
  转成 nav2 要的**字符串**格式 → 在 `nav2_params_sim_robot11_costmap.yaml` 里
  `footprint: "[[...]]"`（注意：**必须是字符串**），并把 `robot_radius` 那条同时撤掉）。

### K.4 阻塞项 3：插件 `point = range·axis` 漏了射线起点（0.1 m）—— 共享代码修复 + 回退开关

#### K.4.1 修什么（源码级，一行）

`src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`：

```cpp
// 射线起点（InitializeRays()）：start = minDist·axis + offset.Pos()
//   minDist = SDF <range><min>（本仓所有模型 = 0.1 m）
//   offset.Pos() = 传感器在父 link 里的位置（livox_frame = (0.00056, 0.1309, 0.1570)）
// 而 range 是**从射线起点**量的距离 ⇒ 命中点 = range·axis + minDist·axis + offset.Pos()
- auto point = range * axis;                       // 旧：既漏 minDist·axis、也漏 offset.Pos()
+ auto point = range_from_origin_
+     ? (range * axis + minDist * axis + sensor_offset.Pos())   // 新（默认）
+     : (range * axis);                                        // 旧（逐字节回退）
```

回退开关：SDF `<range_from_origin>false</range_from_origin>`（**缺省 = 修好的语义**）。
`plugin` / `urdf` / `sensor` 三档**都**走这一行（与 `<cloud_frame>` 正交）。
启动日志会打一行 `range_from_origin = true ⇒ 点 = ...`，用来确认加载的是哪一版二进制。

#### K.4.2 跨模型 A/B（默认模型 + robot11：旧行为 vs 新行为）

`.tmp_robotslot/k2_def_{legacy,fixed}` 与 `.tmp_robotslot/k2_r11_{legacy,fixed}`
（同世界/同出生点/同命令；两次编译**只差那一行**）：

| 量 | 旧行为（`range·axis`） | 新行为（含起点） | 差 |
|---|---|---|---|
| 原始云点数（中位/帧） | — | — | 同量级（点只沿射线挪，不增不减） |
| **原始云的"地面峰"（z 直方图在 1–4 m 环上的峰值）** | **−0.120 m** | **（见 §K.4.3）** | 这就是那个 0.1 m 内移 |
| `/segmentation/obstacle` 中位点/帧 | 3716 | 3782 | **+66** |
| `/segmentation/ground` 中位点/帧 | 2605 | 2562 | **−43** |
| `/scan` 有限波束/帧 | 1160 | 1182.5 | **+22.5** |
| `/global_costmap/voxel_grid` 中位点/帧 | 1129 | 1303 | **+174** |
| 局部 lethal / inscribed / free | 446 / 10129 / 37883 | 454 / 9549 / 39477 | 同量级 |
| 车那格 / 车半径圆内 `≥99` / free | 0 / **0** / **1000** | 0 / **0** / **1007** | 无变化 |
| 到最近 lethal 格 | 0.972 m | 1.078 m | 同量级 |
| 契约（9 个关键话题的发布者数） | 逐个 1 | **逐个 1** | — |

**判读**：默认模型上没有任何一项变坏（`obstacle` +66 / `voxel_grid` +174 是"贴地噪声被正确
挪到地面上"的方向，`inscribed` 反而降了 580）；点数、契约、代价图都在跑间噪声内。
⇒ **不是回归**。

**`robot11`（plugin 档）同款 A/B**：

| 量 | 旧行为（`range·axis`） | 新行为（含起点） | 判读 |
|---|---|---|---|
| **原始云"地面峰"**（1–4 m 环 z 直方图） | **−0.280 m** | **−0.155 m** | 旧值"随距离变"（§K.4.3），新值是尖峰 |
| 自击团（`r < 0.12 m`）点数 | **3294**（28.5%） | **3294**（28.5%） | **逐位相同** —— 自击团离传感器只有 0.02~0.05 m，0.1 m 的内移对它的**相对**位置影响最小（这也解释了为什么 `self_mask` 不用重烘） |
| `/segmentation/obstacle` 中位点/帧 | 5323.5 | **5662** | +338（贴地点回到 obstacle 之外的正确一侧） |
| `/segmentation/ground` 中位点/帧 | 6316 | 6099 | −217 |
| `/scan` 有限波束/帧 | 864 | **772** | −92（旧的 0.05–0.10 m 自击带里那一批"被内移 0.1 m 后落进带内"的假波束消失） |
| 局部 lethal / inscribed / free | 338 / 11730 / 38277 | **231 / 8599 / 43960** | 三项都往"更干净"方向 |
| **车半径圆内 `≥99` / free（半径 0.3565）** | **13 / 601** | **0 / 851** | **修完这一行才和 §K.2.3 的"圆内 0 个 ≥99"一致** |
| 到最近 lethal 格 | 0.627 m | **0.790 m** | 同量级偏干净 |
| 契约（9 个关键话题发布者数） | 逐个 1 | 逐个 1 | — |

⇒ **两档（默认模型 / robot11）都没有任何一项变坏，robot11 还明显变干净** ⇒ 设成默认。
（`robot11_mount:=sensor` 那一档的同款 A/B 见 §K.8 第 3 项 —— 那两跑在本节交付时**还在跑**，
**不主张**它们的数字。）

#### K.4.3 这个修复的**独立**证据（与掩膜/dz 判据都无关）

1. **地面不再是"随距离变化的斜面"**：旧行为下地面点 z 随水平距离单调变化
   （`docs/robot_models.md` §12.3.1 的独立实测：r 0.25–0.35 → −0.208、r 2–4 → −0.253），
   正是"沿射线内移 0.1 m"应有的样子；新行为下同一朵云的**地面峰是一个尖峰**
   （本主题 k7 跑的 z 直方图：`z ∈ [−0.16, −0.15]` 单桶 3283 点，占 1–4 m 环的 76%）。
2. **`point = range·axis` 与 `InitializeRays()` 的 `start = minDist·axis + offset.Pos()`
   在源码里是同一个 `axis`** ⇒ 这一条是**可核对**的代数事实，不依赖任何测量。
3. **它同时解释了 §D.3 的第 2 条**（"插件 0.1 m 系统内移把近处地面整体抬高"）——
   §D.3 当年只能从"地面 z 随距离变化"反推，现在有了机制与修法。

#### K.4.4 为什么敢把它设成**默认**（而不是只给 robot11）

* 它**只让坐标回到几何真值**：`range` 的定义、射线起点、`offset.Pos()` 三者都在源码里，
  修复后 `point` 与 `start + range·axis` **逐位相等**（同一个 `axis`，同一次乘法）。
* 实测默认模型**没有任何一项变坏**（§K.4.2 的表）。
* 代价（必须知道）：**它改的是所有模型的点云** ⇒ 任何"按旧点云烘出来的东西"都要重新核对：
  `traversability_self_mask_robot11.yaml` 的 113 个 AABB 与近场死区（`r ≤ 0.2416`、`z ≥ −0.2295`）
  **都是按旧点云烘的**。实测（本主题所有跑都已经是新插件）`self_mask_enable=true` 下
  `/scan` 的 0.05–0.10 m 自击带仍然是 326~332 条/帧（旧插件 330~332）⇒ **掩膜的命中率没有塌**
  （自击团本身离传感器只有 0.02~0.05 m，0.1 m 的内移对它的**相对**位置影响最小）。
  ⚠️ 但如果将来把 `minDist` 改大（>0.1 m），掩膜必须重烘（`robot11_self_mask.py --emit`）。
* 回退：`<range_from_origin>false</range_from_origin>`（在 `sentry_robot_robot11_sim.xacro`
  的 `<plugin>` 里加一行；其它模型在其 xacro 的 `<plugin>` 里加）—— **逐字节回到今天的行为**。

### K.5 验收表（无头、标准隔离、逐项实测）

跑法（每行一条命令；`robot:=robot11` 的槽位覆盖 = 近地剔除 0.05 + 内切 0.300/0.60/0.65）：

```bash
# ① 槽位档（近地剔除 0.05 + 半径 0.300）：静止 + 短目标
tools/scripts/tiltmount/run_tilt_mount_probe.sh k6_goal --variant plugin --settle 30 \
    --duration 30 --frames 3 --goal-forward 0.5 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
# ② 干净静止三档几何 A/B（近地剔除都开）
bash tools/scripts/tiltmount/_batch_k5.sh
# ③ 近地剔除 关/开（带栅格落盘 + 目标）
bash tools/scripts/tiltmount/_batch_k3.sh
# ④ 同一帧回放 A/B（近地剔除 0.0/0.05/0.10）
bash tools/scripts/tiltmount/_replay_ng.sh .tmp_tiltmount/k7_geo/dump/raw_0.csv 0.2595
# ⑤ `sensor` 档的带宽复现（§K.1.3）
tools/scripts/tiltmount/run_tilt_mount_probe.sh p5_base_sensor --variant sensor --settle 25 \
    --duration 30 --frames 4 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gui:=False
```

| 验收项 | 要求 | **实测** | 判读 |
|---|---|---|---|
| `/scan` 出带比例（`sensor` 档） | ~0 | **0.3960**（269/755）—— §J.5 记 0.404 | ❌ **未达成**；成因与替代修法见 §K.1（2D 链路结构上定不了基；`sensor` 档的盘面在斜 odom 里就是斜面） |
| 局部代价图**不为空**（`sensor` 档） | 非空 | lethal **495** / inscribed 16511 / free 32480 | ✅（§J.5 的"0/0"是"平移取真值"那一版的后果，§K.1.3） |
| 局部代价图不为空（robot11 槽位档） | 非空 | lethal **330** / inscribed 12139 / free 39260（k6） | ✅ |
| 局部代价图不为空（默认模型） | 非空 | lethal 454 / inscribed 9549 / free 39477（k2_def_fixed） | ✅ |
| **车那格 free** | 是 | robot11 槽位档：**0（FREE）**（k6 / k5 三档都是 0）；默认模型：0 | ✅ |
| **车半径圆内 free 格**（半径 0.300） | 与默认模型可比 | robot11：**871/1005**（0.300 档）/ 723/1008（0.3565 档）；默认模型（半径 0.22）：**1007/1007** | ✅ 同量级（剩下的差来自"真车的圆更大"这一件事本身） |
| 车半径圆内 `≥99` | ~0 | robot11：**0**（0.300 / 0.3565 两档都是 0）；默认模型：0 | ✅（对比：修之前 §D.2 是 **435~513/996**） |
| **短目标真的让车动** | 位移 ≫ 0.0015 m | `--goal-forward 0.5`：真值位移 **0.313 m**、目标后残余 **0.170 m**（= 从 0.5 m 走到 0.17 m 内）；`--goal-forward 2.0`：**0.357 m** / 残余 1.573 m | ✅ **0.313 m ≈ 旧的 209 倍** |
| `detected collision ahead` 条数 | 报数 | 槽位档（k6）：**0** 条；近地剔除**关**的同协议跑：**163** 条、`patience exceeded` **8** 条、`Running spin` 2 次 | ✅ 修完不再"判前方碰撞" |
| 恢复行为 | 报"有没有 fire" | k6：0 条 spin / 0 条 patience；`/cmd_vel` 非零 31 条（`max(|vx|+|wz|) = 0.849`，**不是** 3.0 的恢复自旋） | ✅ |
| **限速器在真特征上仍触发** | 触发 | `/speed_limit` 每帧都发（n = 394，`frac_limited = 1.0`），lim `0.917~1.0 m/s`；日志逐条给出理由：`why=slope_change ... Δ坡=36.3° 坡=5.5° d=0.80 m`、`why=slope ... Δ坡=22.4° d=0.80 m`（`--drive` 跑：`why=slope_change` 52 条 / `why=slope` 1 条） | ✅ |
| **护栏/真障碍仍是障碍** | 是 | k6 局部图 lethal **330** 格、方位分布 `{前 47+58, 左 18+21, 后 97+74, 右 0}`、最近 **0.565 m**、p50 **2.272 m**；`/scan` 的 0.50–1.00 m 带 **73 条/帧**（贴地那一层被剔之后**仍有**回波） | ✅ |
| 台阶仍是障碍（**构造性 + 离线同帧**） | 是 | 台阶闸 0.15 m > 0.05 m；回放台 A/B：`step_edge` 中位 **2011 / 2011 / 2011**（`near_ground_m = 0/0.05/0.10`）**逐字相同** | ✅ |
| 9 个关键话题各 1 个发布者 | 都是 1 | 每一跑都测：`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered`、`/segmentation/{ground,obstacle}`、`/scan`、`/odom`、`/local_costmap/costmap`、`/global_costmap/voxel_grid` = **1** | ✅ |
| `plugin`/`urdf` 档与默认模型"不变" | 逐字节/同量级 | 默认模型：本主题**没有**改它的任何参数路径（近地剔除读的是 `..._near_ground.yaml` = 0.0）；`robot_radius` 等键只在本槽位覆盖文件里 ⇒ 默认路径**逐字节不变**。插件那一行是**共享代码**且默认改成新语义（§K.4.2 的 A/B：默认模型没有一项变坏） | ✅（附数字） |
| RTF | 报数 | 槽位档 k6：**0.339**；k5 三档 0.385/0.416/…；§J 那一轮 0.39~0.46 ⇒ **同一量级**（本沙箱偏载时 RTF 不可比，§G 第 3 项） | 登记 |
| 内存 | 报数 | 本沙箱此前实测 **Livox ray sensor = 2.74 GiB of gzserver**（§G/§J 的同一台机）。本主题**没有**新增常驻节点/话题（只有两个**私有诊断字段**与一个**参数**）⇒ 不改变这条 | 登记 |

### K.6 用户命令（复制即可）

```bash
# ① 构建（改了：插件 + 判据层 + 两个分割节点 + launch/参数）
colcon build --symlink-install --packages-select ros2_livox_simulation rm_ground_traversability \
    linefit_ground_segmentation_ros patchwork_ground_segmentation rm_nav_bringup

# ② 今天的行为（默认档：近地剔除关、半径 0.300/0.60/0.65、插件已修）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0
#   · 想看斜装"物理保真 + 账也对"：追加 robot11_mount:=sensor
#   · 想看"倾角记在关节上"的诊断档：追加 robot11_mount:=urdf

# ③ 30 秒自检（跑起来之后，另开一个终端）
ros2 param get /ground_segmentation obstacle_near_ground_m      # 期望 0.05（robot11）
ros2 param get /local_costmap/local_costmap robot_radius        # 期望 0.3
ros2 param get /local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height  # 期望 0.0
ros2 topic echo /segmentation/obstacle --field width -n 1       # 期望 ~5500-5800（不是 ~6100-6900）
ros2 topic echo /speed_limit -n 3                               # 期望非 0（有上限）或 0.0（= 不限速）
# 近地剔除的收据（每帧一行 JSON，私有话题）：
ros2 topic echo /ground_segmentation/traversability_stats -n 1 | grep -o '"near_ground":[0-9]*'

# ④ 一条命令量齐（隔离无头 + 参数/TF 回读 + 契约 + 代价图圆 + 短目标）
tools/scripts/tiltmount/run_tilt_mount_probe.sh my_goal --variant plugin --settle 30 \
    --duration 30 --frames 3 --goal-forward 0.5 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False

# ⑤ 只量近地剔除（同一帧回放，秒级，不用 Gazebo）
bash tools/scripts/tiltmount/_replay_ng.sh <一帧 raw_*.csv> 0.2595
```

### K.7 复现用到的工具（都在 `tools/scripts/tiltmount/`，只有订阅/只有参数）

| 文件 | 作用 |
|---|---|
| `costmap_input_dump.py` + `run_costmap_input_dump.sh` | 一次隔离无头跑里把 `/livox/lidar/pointcloud`、`/segmentation/{ground,obstacle}`、`/scan` **逐帧落 CSV**，并把**每帧的 TF**（`odom←base_link`、`base_link←点云帧`、以及两段合成的 `odom←点云帧`）写进 `meta.json`。⚠️ 两个坑都写在文件里：① 节点**必须**声明 `use_sim_time=true`（否则 TF 查询用墙钟、数据是仿真钟 ⇒ 全查不到）；② 点云 `stamp` 比 TF 缓存最新一条**新一点** ⇒ 直接按 stamp 查 `odom←livox_frame` 会失败，必须**先取最新** |
| `near_ground_replay_probe.py` + `_replay_ng.sh` | 把**一帧真点云**回放给**活的** `ground_segmentation_node`，只换 `obstacle_near_ground_m` ⇒ 同帧 A/B（§K.2.2）。⚠️ `input_topic` 的默认值是 `input_cloud`（相对名）⇒ 命令行必须显式给 `-p input_topic:=/livox/lidar/pointcloud` |
| `ground_leak_probe.py` | 离线：对一朵云用**与 C++ 同一条公式**算局部地面 `g` 与 `dz = z − g`，报各距离带/方位带的 `dz` 分位与"被各种离地闸剔掉多少"（§K.2.4 的表就是它出的） |
| `p2l_forecast.py` | 离线：用**同一帧 raw 云**逐字复算 `pointcloud_to_laserscan`（z 带 / `range_min` / 每 bin 取最近）与 `obstacle_layer` 的"标格"（`projectLaser` ⇒ z 置 0 ⇒ 搬到代价图帧 ⇒ 高度带），回答"近地剔除会少哪几条波束 / 少哪些格" |
| `_costmap_variant.sh` | 在 `0.3565/0.70`、`0.300/0.60`、`0.22/0.50` 三个候选之间切换 `nav2_params_sim_robot11_costmap.yaml`（A/B 用；**跑完自动还原**并用 sha256 校验） |
| `_batch_k1.sh` … `_batch_k7.sh` | 本节的批次脚本（每个都是一串"改一个参数 → 跑 → 还原"的 A/B；**不是**通用工具，留在仓库里只为可复算） |
| `tilt_mount_probe.py`（改） | ① `--z-band-lo/-hi`（高度带**做成探针参数**，并按**节点侧回读的生效值**自动覆盖）；② 新增 `/speed_limit` 的订阅与统计（`n_limited / frac_limited / limit_p05 / n_at_floor`）；③ 报告里落 `costmap_params_effective`（本次运行**生效**的 `robot_radius` / `footprint` / 高度带）⇒ "这张表是按哪组参数读的"不再靠人记 |

### K.8 未验证 / 诚实清单（本轮）

1. **`sensor` 档的 `/scan` 出带比例仍然是 0.396**（§K.1.3）：2D 链路**结构上**定不了基，
   真要修得改代价图帧或 LIO 的 odom 重力对齐 —— 本主题**没做**，只把它从"改高度带能救"
   纠正成"改高度带救不了"。
2. **全局代价图在静止时仍然偏脏**（§K.3.1：0.300 档车那格 0、圆内 123 free；
   但整张图 `unknown` 仍占大头，`map` 还在建）。**没有**做"跑几分钟之后再看"的对照。
3. **插件 A/B 跑完了默认模型与 `robot11`（plugin 档），`sensor` 档没跑完**（§K.4.2）。
   `k2_sen_{legacy,fixed}` 在**本节交付时还在跑** ⇒ **本节不主张** sensor 档的数字。
   实测到的现象（供下一个人省时间）：`k2_sen_legacy` 那一跑的 `probe.json` 里
   **`frames = {}`（一条消息都没收到）**，而 `launch.log` 里 **linefit 与限速都在正常工作**
   （`[slope_speed] ... why=slope_change`）、TF `base_link→livox_frame` 也是 −30.000°，最后
   `gzserver ... finished cleanly`。⇒ 这是**取数时机/订阅侧**的问题（探针在这一跑里没订上），
   **不是** `sensor` 档坏了；`k2_sen_fixed` 那一跑在批次被中断时还没落盘。
   要补：`bash tools/scripts/tiltmount/_batch_k2.sh`（≈15 min，会把三档都重跑一遍）。
4. **真足印多边形只做到"知道怎么写"**（§K.3.4）：nav2 要**字符串**格式，本仓的运行时深合并
   写嵌套序列会让 `rcl` 报 `Sequences cannot be key` 并把**所有** nav2 节点打死（实测踩到）。
   **没有**跑通"字符串格式的 footprint"那一版。
5. **`--goal-forward 2.0` 仍然走不到**（真值 0.357 m / 残余 1.573 m）。0.5 m 的目标**能走到**
   （0.313 m / 残余 0.170 m）⇒ 修完感知之后"车能走"，但**长距离仍然不行**：
   本主题**没有**归因（候选：LIO 出生自转 ~10°（§I.5.2）、全局图还在建、RPP 的
   `regulated_linear_scaling_min_radius 0.9`）。**这是本主题最大的未完成项。**
   > **→ 2026-10-10 已归因：见 §L。** 一句话 = ① 出生点**正前方 0.42 m 处就有实体障碍**
   > （把 nav2 摘掉盲推也只能走 **0.4199 m**；默认模型 **0.5402 m**）⇒ 2.0 m 的"正前方"目标
   > **物理不可达**；② 0.5 m 目标的"成功"是 **0.1902 m 残余 < 0.25 m 容差**（真值只走了 0.2977 m）；
   > ③ `k1_ngoff` 那跑的 0.357/1.573 是**旧配置**（radius 0.3565、近地剔除关）的读数，
   > 当前配置同协议为 **0.3373 / 1.8924**；④ §K.5 表里"0 条 collision ahead"**只对短目标成立**，
   > 长目标是 **135 条**；⑤ 另发现 robot11 的**角速度通道只执行 0.55%**（默认模型 4.4%），
   > 这才是"需要转向的目标"失败的直接原因。
6. **`sensor_height` 的 4.3 cm 疑问**（§K.1 末尾）：几何预测 0.2595 m vs 本主题反推的
   ~0.302 m。**没有查**（要查的是"轮 collision mesh 最低点 vs 关节 origin 的 FK"）。
   影响面：`linefit` 的 `sensor_height`（别的任务的槽位文件）与 `p2l` 的高度带解释。
7. **没有测 `ground:=patchwork` + 近地剔除**：判据层是同一份、参数文件同一路递进
   ⇒ 代码路径相同，但**没有跑过**那个组合（patchwork 的输入点云由它自己产生，
   地面估计的**数值**会不同）。
8. **没有测 `robot11_mount:=urdf` + 近地剔除**：那一档的 `/scan` 在 odom 里是斜面
   （§I.7 的 71.4% 被丢），近地剔除**只解决"贴地点被标成 lethal"**这一半，
   另一半（半圈波束被高度带丢掉）在那一档仍然存在。
9. **RTF/内存没有做"同时间窗交替"的严格 A/B**（§G 第 3 项的老限制）；
    本节的 RTF 数字（0.339~0.416）只能读作"同一批里可比"。
10. **`--frames` 与采样时机**：`dump` 那一套在短窗口里只能采到 1~3 帧（`obstacle`/`ground`
    两朵云是**异步**到的，帧号不同 ⇒ 不能拼成"同一帧"）。本节的离线分析全部只用 `raw` 那一朵
    （唯一自洽的输入），`ground`/`obstacle` 只用来报**中位点数**。

### K.9 回退

| 想退掉什么 | 怎么做 |
|---|---|
| **近地剔除**（本主题最大的一处行为改动） | 把 `src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml` 的 `obstacle_near_ground_m` 改成 **0.0**（一个数），或去掉 launch 里 `near_ground_params` 那一路 ⇒ **完全回到今天** |
| **半径/膨胀**（0.300/0.60/0.65 → 0.3565/0.70/0.75） | 改 `nav2_params_sim_robot11_costmap.yaml` 的两个键（或 `git revert` 本节第 1 个 commit 的该文件） |
| **插件偏移修复** | 在对应模型的 xacro 的 `<plugin>` 里加 `<range_from_origin>false</range_from_origin>` ⇒ **逐字节**回到旧行为（不需要回滚代码） |
| **整套 §K** | `git revert <§K 的 3 个 commit>`：默认路径的行为**逐字节不变**（近地剔除默认 0.0、半径只在槽位覆盖文件里）；**唯一**需要单独决定的是插件那一行（它是共享代码，revert 它就回到旧点云） |
| 本节新增的工具/批次脚本 | `rm -rf tools/scripts/tiltmount/`（只有 §K.6/K.7 的命令引用它们；不进任何 launch/节点） |
| 本节的原始数据 | `.tmp_tiltmount/k*`、`.tmp_robotslot/k*`（未入库） |

---

## L. 2026-10-10：`robot:=robot11` 的 `--goal-forward 2.0` 走不到的归因（§K.8 第 5 项的收口）

> 触发 = 父任务原文：**"诊断 `robot:=robot11` 为什么短目标能动、长目标不动 —— 这是模型能不能算
> 可用/默认的最后一个阻塞项"**，并点名要**逐条测试候选原因、不要猜**。
>
> 与 §K 的关系：§K.8 第 5 项把这件事登记为"**本主题最大的未完成项**"，并给了三个候选
> （出生自转 ~10° / 全局图还在建 / RPP 的 `regulated_linear_scaling_min_radius 0.9`）。
> **本节把三个候选全部按实测判决**，并给出第 4、第 5 个候选（**场地里的实体障碍**、
> **底盘的角速度通道几乎不执行**）。§K 及以前的内容**逐字保留**；本节只**追加**，
> 但对 §K.5/§K.8 引用的两组数字做一处**口径更正**（见 §L.1.3）。
>
> 全部无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、按 tag 哈希占用的
> `GAZEBO_MASTER_URI`、`unset DISPLAY`、只 kill **本 master URI 上**的 gzserver/gzclient，
> **绝不做全机 pkill**；`world:=RMUL2026` 不带前导 `--`），
> 原始数据 `.tmp_tiltmount/n1_*` … `.tmp_tiltmount/n4_*`；工具见 §L.12。

### L.0 一句话结论（五条，都是实测）

1. **根因不是模型、不是参数、不是代价图：是"目标方向上 0.42 m 处有实体障碍"。**
   把 nav2 **完全摘掉**（只给底盘插件发固定 `vx`、绕过控制器/代价图/BT），
   `robot:=robot11` 从出生点**只能前进 0.4199 m**（请求 6.25 m，25 s 内 500 条指令）；
   **默认模型**同协议只能前进 **0.5402 m** ⇒ 这是**场地/出生点**的性质，两个模型都撞在同一处
   （§L.2）。⇒ 2.0 m 的"正前方"目标**物理上不可达**，不存在"走得到"的参数组合。
2. **0.5 m 目标"能走"是"容差成功"，不是"车走了 0.5 m"**：真值只走了 **0.2977 m**，
   停在离目标 **0.1902 m** 处 < `xy_goal_tolerance 0.25` ⇒ `Goal succeeded`（§L.7）。
   所以"短目标能走"这条证据**不能**推出"车能走 0.5 m"，更不能推出"长距离只是慢"。
3. **控制器侧的"不动"是结果、不是原因**：2.0 m 那一跑 RPP 报了 **135 条**
   `detected collision ahead`、**6 条** `Controller patience exceeded`、BT 进了 **7 次**恢复
   （ClearLocal/GlobalCostmap + Spin + BackUp + Wait）。**但**同一跑里 `/cmd_vel` 真的发过
   `vx=1.0`（89 条 >0.01），车也真的动到 **0.4197 m** 就再也推不动了（§L.4）。
   （§K.5 表里"0 条 collision ahead"只对**短目标**那一跑成立，见 §L.1.3 的口径更正。）
4. **地图侧的两个真实现象（都不是根因，但都会**改变失败的形状**）**：
   ① `mode:=slam_nav` 下全局代价图**只覆盖已建出来的那一块**，目标落在图外时规划器直接拒绝
   （`The goal sent to the planner is off the global costmap`）——反向 2.0 m 目标就是这个死法；
   ② `mode:=nav`（已有 `RMUC2026`/`RMUL2026` 先验图）下，同一个"正前方 2.0 m"目标被判
   **不可行**（`Planning algorithm GridBased failed to generate a valid path` ×10，
   目标格 = 31/inflated、0.5 m 内 23 个 lethal、到 ≥99 格 0.158 m）⇒ **完整地图也说不通**。
5. **另一条独立缺陷（robot11 比默认模型差 33 倍）：底盘的角速度通道几乎不执行。**
   绕过 nav2 直接发 `wz=1.0 rad/s` 持续 12 s（请求 **687.5°**）：
   `robot11` 真值 **+3.79°**、LIO **+3.86°**；**默认模型** 真值 **+30.09°**、LIO **+29.84°**。
   ⇒ 需要"先转头再走"的目标（RPP 的 `use_rotate_to_heading` + `allow_reversing: false`）
   在 robot11 上**永远转不过去**（实测：`wz=-0.75` 连发 45 s，车只转了 **6.03°**，`vx` 恒为 0）。
   这条同时解释了 §I.5.2/§K.8 里那个没归因的"**出生后自转 ~10°**"（见 §L.6 候选 5）。

### L.1 协议、工具，以及对 §K 两组数字的口径更正

#### L.1.1 协议（与 §K 的 `--goal-forward` 同款，便于逐字对比）

`tools/scripts/tiltmount/run_nav_goal_forensics.sh <tag> --settle 30 --duration 30
--goal-forward H --goal-wait 45 -- world:=RMUL2026 mode:=<slam_nav|nav> lio:=small_point_lio
[robot:=robot11] spin_speed:=0.0 gui:=False`

* 发目标时刻 = launch 后 **60 s**（settle 30 + 记录窗 30），与 §K 的 k6_goal（30+30）/k1（25+35）同口径；
* 目标 = **发目标那一刻** `map→base_link` 的位姿沿车头前进 H 米（与 `tilt_mount_probe.py` 同一算式）；
* 探针**只**发一次 `/goal_pose`（内部连发 5 条，两个探针同款 ⇒ 会看到 4 次 goal preemption，
  这是协议的一部分，不是异常）；
* 全程只订阅：`/plan`、`/local_costmap/costmap`、`/global_costmap/costmap`、`/odom`、
  `/odom_ground_truth`、`/cmd_vel`、`/cmd_vel_chassis`、`/navigate_to_pose/_action/{status,feedback}`、
  `/follow_path/_action/{status,feedback}`、`/behavior_tree_log` + TF。

#### L.1.2 新增的三个工具（都在 `tools/scripts/tiltmount/`，只有订阅 + 一次 `/goal_pose`）

| 文件 | 作用 |
|---|---|
| `nav_goal_forensics.py` | 「一次导航尝试的因果链」探针：逐条 `/plan`（位姿数/长度/首末点/末点到目标的距离）、代价图三张快照（发目标时 / 结束）、目标格的**分类与到 lethal 的距离**、`goal_in_local`、两个 action 的**状态机时间线**、`number_of_recoveries`、`/behavior_tree_log` 的**逐节点跳变**、`/follow_path` 的 `distance_to_goal`/`speed`、`/cmd_vel(_chassis)`、真值/LIO 位姿时间线。★ 另有 **`--push VX` / `--push-wz WZ`**：**绕过 nav2**，只给 `/cmd_vel_chassis` 发固定速度，量"纯物理"能走多远/转多少 |
| `run_nav_goal_forensics.sh` | 隔离跑壳（与 `run_tilt_mount_probe.sh` 同款隔离约定，另占一段 master URI 端口） |
| `nav_goal_report.py` | 把 `forensics.json` 读成一张判决表（plan/状态机/恢复/BT/速度/代价图逐项） |

#### L.1.3 对 §K.5 / §K.8 两组数字的口径更正（本轮同协议复跑，· 唯一变量 = 代码状态）

| 量 | §K 记的 | **本轮实测** | 判读 |
|---|---|---|---|
| `--goal-forward 0.5` 真值位移 / 残余 | 0.313 m / 0.170 m（`k6_goal`） | **0.2977 m / 0.1902 m**（`n1_r11_slam_f05`） | **复现**（同一量级；差异来自版本/时间） |
| 短目标的 `detected collision ahead` / 恢复 | **0 条**（§K.5 表） | **0 条 / 0 次**（复现） | ✅ 只对**短目标**成立 |
| `--goal-forward 2.0` 真值位移 / 残余 | 0.357 m / 1.573 m（`k1_ngoff`，**旧配置**：radius 0.3565、近地剔除**关**） | **0.3373 m / 1.8924 m**（`n1_r11_slam_f20`，**当前配置**：0.300、近地剔除开） | 两个都"几乎不动"；**当前配置并没有让长目标变好** |
| 长目标的 `detected collision ahead` | §K.8 未记（§K.5 表只记了短目标那跑的 0 条） | **135 条**（+ `patience exceeded` 6 条、恢复 7 次） | ⚠️ **"0 条"不能推广到长目标**——这是本轮最需要更正的一条 |
| `gt_yaw_delta_deg`（`tilt_mount_probe.py`） | `k6_goal` 报 **473.416**、`k1_ngoff` 报 **251.651** | 真值其实分别是 **8.26° / 4.39°** | ⚠️ **工具 bug**：`_xy()` 的 yaw 已是**度**，代码又 `math.degrees()` 了一次 ⇒ 全部**多乘 57.2958**。已在 `tilt_mount_probe.py` 修掉并写明"旧读数不要再引用"（§L.3 的表用的都是修好后的新读数） |

### L.2 判据 A（决定性）：把 nav2 摘掉，纯物理能走多远

命令只有一条：`/cmd_vel_chassis` 固定速度，持续 25 s（20 Hz，500 条），
**没有目标、没有规划器、没有控制器、没有 BT**。真值 = `/odom_ground_truth`（Gazebo 世界位姿）。

| 跑 | tag | 模型 | 命令 | **实际位移** | 最后一次 >2 mm 的运动 | 备注 |
|---|---|---|---|---|---|---|
| **正前方** | `n2_r11_push_f` | robot11 | `vx=+0.25` × 25 s（请求 6.25 m） | **0.4199 m** | **t=5.32 s** | 之后 20 s 一动不动 |
| **正前方** | `n2_def_push_f` | 默认模型 | 同上 | **0.5402 m** | t=3.71 s | 同上 ⇒ **不是 robot11 特有** |
| **正后方** | `n4_r11_push_b` | robot11 | `vx=-0.25` × 25 s（请求 6.25 m） | **1.5753 m** | t=15.78 s | 后面有 1.58 m 空间 |
| **纯自转** | `n3b_r11_push_wz` | robot11 | `wz=+1.0` × 12 s（请求 **687.5°**） | 位移 0.0017 m，**Δyaw 真值 +3.79° / LIO +3.86°** | — | 角速度通道 ≈ 不执行 |
| **纯自转** | `n3b_def_push_wz` | 默认模型 | 同上 | 位移 0.0007 m，**Δyaw 真值 +30.09° / LIO +29.84°** | — | 默认也只有 4.4%，但比 robot11 好 33 倍 |

**两条独立的几何自洽（这才让"实体障碍"从猜测变成测量）**：

* 前进极限 + 车体外接半径 = `0.4199 + 0.3565 = **0.776 m**` ≈ **在线代价图里 x≈0.78 的那条 lethal 列**
  （§L.3 的 `t_end` 快照：目标附近 0.52 m 内有 lethal 格、窗口内 9 个 lethal / 89 个 ≥99）；
* 后退极限 + 车体外接半径 = `1.5753 + 0.3565 = **1.932 m**` ≈ **先验图 `RMUL2026.yaml` 的西墙 x≈-1.9**
  （出生点相对系，`docs` 里已写明"出生点在 map 里就是 (0,0,0)"）。
* 第三处自洽（**同一跑内部**）：`n1_r11_slam_f20` 那跑车被推到的**峰值真值位移 0.4197 m**
  ≈ 纯物理推的极限 **0.4199 m**（差 0.2 mm）——即"导航跑到的位置"与"盲推到不了的位置"是同一个点。

⇒ **2.0 m 正前方目标落在障碍后面**；`mode:=nav` 下规划器（完整先验图）对同一个目标直接
`failed to generate a valid path`（§L.3），也是在说同一件事。

### L.3 判据 B：目标侧的代价图读数（`slam_nav` vs `nav`）

**① `mode:=slam_nav`（在线建图，§K 的默认口径）** —— `n1_r11_slam_f20`：

| 量 | 实测 |
|---|---|
| 有没有 plan | **有，10 条**（t=60.1…68.6 s，每条 2.06–2.59 m，**末点 = 目标的距离 = 0.0000 m**） |
| 目标格（发目标时 `t_goal`） | **-1 / unknown**；11×11 窗内 `{unknown:436, free:85, ≥99:5}`；到最近 lethal **0.992 m** |
| 目标格（结束时 `t_end`） | **-1 / unknown**；窗内出现 **9 个 lethal + 89 个 ≥99**；到最近 lethal **0.522 m** |
| 沿 plan 采样的代价（最后一张图） | plan#0 最小间隙到 ≥99 格 = **0.050 m**、plan#2/#3 = **0.250/0.255 m**（`robot_radius=0.300`）⇒ **路径是擦着 inscribed 带走**，模型上是"刚好不压 lethal" |
| 目标在局部代价图里吗 | **在**（`goal_in_local` = True，91/91 个采样；局部图 5×5 m @0.02、`odom` 系） |
| 车那格（结束时的局部图） | 车心正前方代价剖面 `0.00:0 → 0.05:26 → … → 0.50:80 → …`，**0.15–0.24 m 处已经是 ≥99 带，0.56–0.80 m 处是 lethal** ⇒ 车头压在内切带里 |

⇒ 在线图在"车的正前方"这一块**还没建出 lethal**（大部分 unknown，而 navfn 是 `allow_unknown: true`），
所以它给了一条**穿过未建出的障碍**的直路 —— 这就是 §K.8 候选"全局图还在建"的**真实作用**：
**它决定"规划器给不给路"，但不决定"车能不能过去"**（车是被障碍挡住的，§L.2）。

**② `mode:=nav`（先验图）** —— `n2_r11_nav_f20`（正前 2.0 m）：**0 条 plan**，
`Planning algorithm GridBased failed to generate a valid path` ×10、`failed to create plan with
tolerance 0.5` ×10；目标格 = **31 / inflated**、窗内 `{free:112, inflated:159, ≥99:147, lethal:23}`、
到 ≥99 格 **0.158 m** ⇒ **完整地图判它不可行**。

**③ 反向 2.0 m（§L.8 的对照）**：`mode:=slam_nav` 下目标 (-1.95,-0.06) **落在全局代价图外**
（`The goal sent to the planner is off the global costmap` ×6、0 条 plan）；
`mode:=nav` 下**能规划**（45 条 plan，1.60–1.72 m，末点被 `tolerance` 吸附到离目标 0.57 m 的free格），
但车**一步没动**（真值 0.0069 m）—— 原因见 §L.5。

### L.4 判据 C：控制器/BT 到底发生了什么（全是"结果"，不是"原因"）

`n1_r11_slam_f20`（robot11、`slam_nav`、正前 2.0 m）：

| 量 | 实测 | 读法 |
|---|---|---|
| `/plan` | 10 条、每条 1 Hz 重规划、末点落在目标上 | 规划链路**完全正常** |
| `FollowPath` 状态机 | `EXECUTING → ABORTED` 每 ~1 s 一次（45 次）；`RateController → SUCCESS` ×10 | 1 Hz 重规划 BT 的**正常**行为（新路径会 abort 旧 FollowPath goal），**不要**当成故障 |
| `RegulatedPurePursuitController detected collision ahead!` | **135 条** | RPP 的 `use_collision_detection: true`（前瞻 1.0 s）在车的正前方看到 lethal/inscribed ⇒ **拒绝往前走** |
| `Controller patience exceeded` / `Aborting handle` | **6 / 6** | 连续异常超过 `failure_tolerance 0.3 s` ⇒ FollowPath 失败 |
| `number_of_recoveries`（`/navigate_to_pose/_action/feedback`） | **最大 7** | 恢复**真的在 fire**（§K.5 的"0 条"只对短目标成立） |
| BT 恢复节点 | `ClearLocalCostmap` + `ClearGlobalCostmap` + `RecoveryFallback/RecoveryActions` + `behavior_server: Running spin / Running backup / Running wait`（`spin failed: Exceeded time allowance`） | 标准恢复序列跑满了 |
| `/cmd_vel(_chassis)` | 发目标后 **465 条**，`max|vx| = 1.0`、`n(vx>0.01) = 89` | **控制器确实在给前进速度**（不是"没发指令"） |
| 真值时间线 | t=61.1 起 `vx=1.0`；t=62.6–64.6 连续 `vx=0.60–0.82` 的 2 s 里，真值只动了 **7.8 mm** | **被推着也走不动** ⇒ 实体阻挡（与 §L.2 的盲推一致） |
| 目标在不在局部图里 / 局部图有没有障碍 | 在（91/91）；局部图**车前方 0.15–0.80 m 就是 ≥99/lethal 带** | 控制器"看到的东西"与物理一致 ⇒ **不是幻影障碍** |

**默认模型的同协议跑**（`n1_def_slam_f20`，正前 2.0 m）作为对照：30 条 plan、恢复 9 次、
collision ahead 103 条、patience 6 条；真值 t=61.6 到达 4.840 m 后**在 `vx=0.50` 连发 20 s 的情况下
只动了 5 mm**（t=61.6→86.1）。⇒ **同一个失败**，只是它斜着蹭墙多走了 0.13 m（0.674 vs 0.337）。

### L.5 判据 D：角速度通道（本轮新发现，也是"反向 2.0 m"失败的直接原因）

| 场景 | 命令 | 实际 Δyaw（真值 / LIO） | 有效率（对请求） |
|---|---|---|---|
| 盲发（`n3b_r11_push_wz`，robot11） | `wz=1.0` × 12 s = **687.5°** | **+3.79° / +3.86°** | **0.55%** |
| 盲发（`n3b_def_push_wz`，默认模型） | 同上 | **+30.09° / +29.84°** | 4.4% |
| RPP 的 rotate-to-heading（`n2_r11_nav_b20`，robot11，反向目标） | `wz=-0.75` **连发 45 s**（1804 条），`vx` **恒 0** | **-6.03°** | 0.35% |
| 恢复 Spin（`n1_r11_slam_f20`，robot11） | `wz=3.0` × ~22 s | ≈ +27° | 0.1% |
| 恢复 Spin（`n1_def_slam_f20`，默认模型） | `wz=3.0` × ~9 s | ≈ +76.7° | 0.3% |
| **旁证（本仓既有产物）** `k4_drive` | 固定动作协议（直行 10 s `vx=0.30` ↔ 原地转 10 s `wz=0.60`） | 真值 yaw 序列 **0.0°→0.0°**、`yaw_span 14.564°`；LIO `yaw_span 14.764°`、净变化 0.698→0.249 | ≈0 |

* **两个互相独立的估计器一致**（Gazebo 世界位姿 `/odom_ground_truth` 与 LIO `/odom`，后者含 IMU 陀螺）
  ⇒ **不是里程计伪影，是车真的没转**。
* 反证"车不是被墙卡住才不转"：同一次起跑点上，正前方有 0.42 m 空间、正后方有 1.58 m 空间
  （§L.2），出生点**不在接触状态**；且纯自转跑里位移只有 1.7 mm。
* 机理**没有钉死**（登记在 §L.10）：模型是"轮子自由滚动（无轮速控制器）+ `libgazebo_ros_planar_move`
  直接设底盘速度"，场地 collision 面是 `mu=1` + `torsional coefficient 1 / use_patch_radius 1`
  （`RMUL2026_world.world:100-110`）⇒ 平动靠轮子滚动（实测 ≈100% 执行），
  自转要靠接触面**侧滑/扭转**（实测 robot11 0.55%、默认 4.4%）。
  robot11 比默认模型差 33 倍的具体几何/惯量原因**本轮没查**。
* **它解释了 §I.5.2/§K.8 那个没归因的"出生后自转 ~10°"**：robot11 真值 yaw 在 30 s 时 **+4.32°**、
  60 s 时 **+8.20°**（记录窗内 +3.88°/30 s）；**默认模型同协议是 -0.08° → -0.17°（几乎不漂）**。
  即：robot11 的偏航轴**基本不受指令控制**，那 ~10° 是**接触/摩擦引起的自由漂移**，不是"自转指令"。

### L.6 候选原因逐条判决（每条：测什么 / 结果 / 证据）

| # | 候选 | 判决性测量 | 结果 | 证据（文件） |
|---|---|---|---|---|
| 1 | **地图/空间**：`slam_nav` 在线图还没建出来，2.0 m 目标在 unknown/occupied 里 ⇒ 规划不出来 | `/plan` 条数 + 目标格分类 + 沿路代价 | **部分成立、但不是根因**：`slam_nav` **有 10 条 plan**（末点精确落在目标上），只是路径**穿过尚未建出 lethal 的障碍**；`mode:=nav`（完整图）反而**拒绝规划**（0 条） | `.tmp_tiltmount/n1_r11_slam_f20/{forensics.json,grids.npz}`、`n2_r11_nav_f20/` |
| 2 | **控制器/规划器参数**：`regulated_linear_scaling_min_radius 0.9` 让 2.0 m 规划不可行/起步慢 | 发目标后的实际速度 + 默认模型基线 | **反驳**：实测 `/cmd_vel` 发到 `vx=1.0`（远高于任何"限速"），且**默认模型（同一套控制器参数）也一样失败**（0.674 m 后 20 s 只动 5 mm） | `n1_r11_slam_f20/`、`n1_def_slam_f20/` |
| 3 | **BT/恢复**：恢复在 fire、progress checker 在跳、根本没进 FollowPath | action status/feedback + `/behavior_tree_log` + 控制器日志 | **反驳（是结果不是原因）**：FollowPath **进了**（45 次 EXECUTING）、`number_of_recoveries` 最大 **7**、恢复序列（clear costmap / spin / backup / wait）跑满；但这些都发生在**车已经被推不动之后** | `n1_r11_slam_f20/roslog/controller_server_*.log`、`behavior_server_*.log`、`forensics.json` |
| 4 | **局部代价图/滚动窗口**：目标在局部图外 / 没有有效局部路径 | `goal_in_local` + 局部图前方代价剖面 | **反驳**：目标**在**局部图内（91/91）；局部图前方 0.15–0.80 m 就有 ≥99/lethal 带（**与物理一致**） | `n1_r11_slam_f20/forensics.json` + `grids.npz:local` |
| 5 | **出生自转 ~10°**：偏航没settle，影响规划/控制 | 真值 yaw 时间线 + 盲发 `wz` | **成立但换了归因**：robot11 的 yaw 在 60 s 内自由漂到 +8.2°（默认模型 -0.17°），而**指令角速度只有 0.55% 被执行** ⇒ "自转"是**不受控的漂移**；它**不是**长目标失败的原因（正前方目标本来就不需要转），但**是"需要转向的目标"失败的原因** | `n3b_r11_push_wz/`、`n3b_def_push_wz/`、`n1_r11_slam_f20/probe` 时间线 |
| 6 | **（本轮新增，决定性）场地里的实体障碍** | **绕过 nav2** 的盲推（`--push`） | **成立**：前进 0.4199 m（robot11）/0.5402 m（默认模型）就被挡死，且 `0.4199+0.3565 ≈ 0.776 m` 与在线图的 lethal 列 x≈0.78 吻合 | `n2_r11_push_f/`、`n2_def_push_f/`、`n4_r11_push_b/` |

### L.7 那"0.5 m 能走到"到底是什么

`n1_r11_slam_f05`（robot11、`slam_nav`、正前 0.5 m）：

| 量 | 实测 | 读法 |
|---|---|---|
| plan | 2 条（0.577 m / 0.349 m，末点 = 目标） | 正常 |
| 真值位移 | **0.2977 m** | **不是 0.5 m** |
| 结束时的残余 | **0.1902 m**（map 系） | < `xy_goal_tolerance 0.25` |
| `detected collision ahead` / `patience` / 恢复 | **0 / 0 / 0** | 因为车根本没开到障碍跟前就"到点"了 |
| BT | `Goal succeeded` | 按配置**合法**的成功 |

⇒ 这一跑证明的是"**车能走 ~0.30 m 并落在容差里**"，**不是**"车能走 0.5 m"。
`--goal-forward 0.5` 与 `--goal-forward 2.0` 的差别，本质上是"**目标在不在 0.42 m 的可达区里**"，
而不是"控制器对长距离做了什么不一样的事"。

### L.8 对照（防止结论是单一方向的伪影）

| 对照 | 跑 | 结果 | 结论 |
|---|---|---|---|
| **反向 2.0 m**（`slam_nav`） | `n1_r11_slam_b20` | 目标在全局代价图外 ⇒ `off the global costmap` ×6、**0 条 plan**、真值 0.118 m（全是恢复自转） | 反向也走不到，但**死法不同**（图外 ⇒ 规划器拒绝） |
| **反向 2.0 m**（`nav`，先验图） | `n2_r11_nav_b20` | **45 条 plan**（1.60–1.72 m）、`vx` **恒 0**、`wz=-0.75` 连发 45 s、真值 **0.0069 m**、Δyaw **-6.03°**、`Failed to make progress` ×2 | 反向失败 = **转不过去**（§L.5），与"正前方被挡"是**两个独立机制** |
| **默认模型 + 正前 2.0 m** | `n1_def_slam_f20` | 真值 **0.6745 m**（`vx=0.5` 连发 20 s 只动 5 mm）、残余 1.5265、恢复 9 次、collision 103 条 | **不是 robot11 特有**：默认模型撞在同一处 |
| **默认模型 + 盲推** | `n2_def_push_f` | **0.5402 m** | 同一处障碍，独立的物理读数 |
| 逆方向盲推 | `n4_r11_push_b` | **1.5753 m** | 反向空间是够的（1.58 m）⇒ 反向失败**不是**空间问题 |

### L.9 "要什么才能真修"（诚实清单）

1. **把测试目标放进可达区**：本出生点（`world:=RMUL2026`）正前方只有 **0.42 m** 可用，
   所以"`--goal-forward 2.0` 必须走 2 m"这条**验收本身不成立**。要么换 spawn/世界，
   要么把验收改成"**盲推极限内的目标 + 可复算的可达性判据**"（本节的 `--push` 就是那个判据：
   一条命令、15 s、不需要 nav2）。
2. **要真做"长距离 + 需要转向"的验收，必须先修自转通道**：当前"轮子自由滚动 +
   `planar_move` 直接设底盘速度 + `mu=1`/torsional 1 的场地"这套配置下，
   实测角速度执行率 robot11 **0.55%**、默认 **4.4%** ⇒ RPP 的 rotate-to-heading
   （`use_rotate_to_heading: true`、`rotate_to_heading_min_angle: 0.785`）在 robot11 上
   **不可能收敛**，任何 >45° 的转向目标都会退化成"原地不动 + 恢复"。
   可选修法（都**没做**）：给轮子加驱动（差速/mecanum 轮控）让接触面"滚"而不是"滑"；
   或把底盘改成运动学体（直接设世界位姿）；或降场地 `mu`/去掉 torsional 项。
3. **robot11 与默认模型在这条通道上差 33 倍**（0.55% vs 4.4%），**具体几何/惯量原因本轮没查**
   （候选：robot11 是 8 个连续关节的转向+轮 link、mesh 抽稀后的 cylinder 轮、质量 9.55 kg；
   默认模型只有 4 个轮 link）。这条**不影响** §L.0 的根因结论，但会影响"能不能做转向类导航"。
4. **参数层面没有可修的**：正前方 2.0 m 目标在实体障碍后面（`nav` 模式规划器直接拒绝），
   减小 `inflation_radius`、放大 `xy_goal_tolerance`、改 `regulated_linear_scaling_min_radius`
   都**只会**让车更靠近障碍或更早宣布成功，**不会**让车过去。

### L.10 未验证 / 诚实清单（本节）

1. **角速度通道的机理没钉死**（§L.5）：只测到"指令执行率 0.55%/4.4%"与"两个估计器一致"，
   **没有**做"改 `mu` / 加轮控 / 改 kinematic"的 A/B（那要改世界或模型，超出本节范围）。
2. **先验图 `RMUL2026.pgm` 的正前方障碍位置（x≈0.5）与实测物理极限（0.776 m）差 ~0.28 m**：
   本轮**没有**归因（候选：cartographer 与当前 LIO 的口径/尺度差、或 x≈0.5 与 x≈0.78 是两个不同物体）。
   反向那一侧是吻合的（1.932 m ↔ 先验图 x≈-1.9）⇒ **不是整体平移**。
3. **`§K.1` 末尾那条 `sensor_height` 4.3 cm 的疑问**仍未查（本节不涉及）。
4. **没有做"跑几分钟后地图建全了再发同一个目标"**：本轮所有目标都固定在 launch 后 60 s 发
   （与 §K 同协议）。`mode:=nav` 的先验图跑可以看作"图已建全"的对照（结论：**拒绝规划**），
   但"`slam_nav` 跑 5 分钟后再发"这一格**没跑**。
5. **RTF 没有单独记录**：从盲推反推 ≈0.32（0.42 m @0.25 m/s 用了 5.32 s 墙钟），与 §K 的
   0.34–0.42 同量级；本节所有"米/度"都是真值读数，**不受 RTF 影响**（时间类结论才需要它）。
6. **`--goal` 的显式目标只用了两处**（反向 2.0 m 的两跑）；没有做"同一个 2.0 m 距离、
   沿不同方位角"的扫描 ⇒ "可达区"的形状只有 **前 0.42 / 后 1.58** 两个方向的读数。

### L.11 回退

| 想退掉什么 | 怎么做 |
|---|---|
| 本节新增的工具（3 个文件） | `rm -f tools/scripts/tiltmount/{nav_goal_forensics.py,nav_goal_report.py,run_nav_goal_forensics.sh}`（**不进任何 launch/节点**，删掉不影响仿真与导航） |
| `tilt_mount_probe.py` 的单位修复 | `git revert <本节第 2 个 commit>`（只影响 `gt_yaw_delta_deg` 一个字段的数值口径；旧值是错的，回退等于恢复"×57.3"） |
| 本文档 | `git revert <本节第 3 个 commit>` |
| **行为** | 本节**没有改任何** launch/参数/模型/世界/感知代码 ⇒ **默认模型与其它槽位逐字节不变**，robot11 的行为也不变（§L 只诊断、不修） |

### L.12 复现命令（复制即可）

```bash
# ① 正前方 2.0 m（robot11、slam_nav）—— 本轮的主跑
tools/scripts/tiltmount/run_nav_goal_forensics.sh n1_r11_slam_f20 --settle 30 --duration 30 \
    --goal-forward 2.0 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
# ② 正前方 0.5 m（同一协议）
tools/scripts/tiltmount/run_nav_goal_forensics.sh n1_r11_slam_f05 --settle 30 --duration 30 \
    --goal-forward 0.5 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
# ③ 默认模型（对照）
tools/scripts/tiltmount/run_nav_goal_forensics.sh n1_def_slam_f20 --settle 30 --duration 30 \
    --goal-forward 2.0 --goal-wait 45 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0 gui:=False
# ④ 纯物理：正前方 / 正后方 / 纯自转（**绕过 nav2**）
tools/scripts/tiltmount/run_nav_goal_forensics.sh n2_r11_push_f --settle 25 --duration 8 --goal-wait 0 \
    --push  0.25 --push-time 25 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_nav_goal_forensics.sh n4_r11_push_b --settle 25 --duration 8 --goal-wait 0 \
    --push -0.25 --push-time 25 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_nav_goal_forensics.sh n3b_r11_push_wz --settle 25 --duration 6 --goal-wait 0 \
    --push-wz 1.0 --push-time 12 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
# ⑤ 先验图（mode:=nav）的正/反 2.0 m
tools/scripts/tiltmount/run_nav_goal_forensics.sh n2_r11_nav_f20 --settle 30 --duration 30 \
    --goal-forward  2.0 --goal-wait 45 -- world:=RMUL2026 mode:=nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
tools/scripts/tiltmount/run_nav_goal_forensics.sh n2_r11_nav_b20 --settle 30 --duration 30 \
    --goal-forward -2.0 --goal-wait 45 -- world:=RMUL2026 mode:=nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
# ⑥ 读表
python3 tools/scripts/tiltmount/nav_goal_report.py .tmp_tiltmount/n1_r11_slam_f20
```

## M. 2026-10-10：`robot:=robot11` 的**角速度通道**归因与修复（§L.9 第 2/3 项的收口）

> 触发 = 父任务原文：**"查清 `robot:=robot11` 为什么几乎不执行角速度指令，若原因在我们的
> 仿真模型/驱动链里就修掉它 —— 这是转向类目标（任何非直线目标）的真正阻塞项"**。
>
> 与 §L 的关系：§L 把"长目标走不到"归因到**场地里的实体障碍**（正前方 0.42 m），并把
> "角速度通道几乎不执行"登记为**第二条独立缺陷**（§L.0 第 5 条、§L.5、§L.9 第 2/3 项），
> 但**没有**钉死机理。本节就是把那条钉死 + 修掉。§L 及以前的内容**逐字保留**。
>
> 全部无头隔离跑（`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、按 tag 哈希占用的
> `GAZEBO_MASTER_URI`、`unset DISPLAY`、只 kill **本 master URI 上**的 gzserver/gzclient，
> **绝不做全机 pkill**；`world:=RMUL2026` 不带前导 `--`），
> 原始数据：`.tmp_wz/bench/out/*`（台架）、`.tmp_tiltmount/m1_*` … `m10_*`（整栈）、
> `.tmp_wz/static/*`（生成物）。

### M.0 一句话结论（六条，都是实测）

1. **根因是"驱动器与模型结构不自洽"，不是单位/缩放/mixing bug**：底盘驱动是
   `libgazebo_ros_planar_move.so`，它的实现（`gazebo/physics/Model.cc:746-771`，本机读源码核对）
   是 **`SetLinearVel` / `SetAngularVel` 对模型里的每一个 link 各设一遍同一个 `(v, ω)`**。
   这对**单刚体**恰好自洽（绕质心转时正是"每个 link 同一个 ω"），对
   **多刚体 + 关节树**则不自洽（第 i 个 link 的质心速度本应是 `ω × r_i`，插件写成 `v`）⇒
   关节/接触约束求解器每步都要把它掰回来，**反作用把底盘的角速度吃掉**。
   `robot11` 是 13 link / 12 joint，`robot:=` 默认模型只有 7 link / 6 joint ⇒ 前者被吃得更狠。
2. **决定性对照（台架，只发 `/cmd_vel_chassis` `wz=1.0` × 12 仿真秒 = 请求 687.5°）**：
   把同一台车的几何**塌成一个刚体**（几何/惯量逐项守恒，见 M.6）⇒ 真值 **+7.09° → +111.7…113.3°**
   （**1.03% → 16.2~16.5%**，**15.7×**）；对照：默认模型 **+40.0°（5.8%）**、
   极简单 link box（无接触）**+681.2°（99.9%）** ⇒ 修好后 robot11 的角速度通道**优于默认模型 2.8×**。
3. **接触/摩擦是第二个、独立的限制项**（剂量-响应单调）：把轮子对地摩擦从 `mu=1` 降到
   `0.5/0.2/0.05/0.001` ⇒ 真值 **+17.3° / +54.8° / +96.9° / +126.8°**；
   把重力关掉（完全没有接触）⇒ **+128.0°**（= 多刚体结构的**上限** 18.6%）。
   ⇒ "轮子真的存在"确实是阻力来源之一，但**它只解释 18.6% → 1.03% 那一段**，
   剩下 100% → 18.6% 那一段是第 1 条（多刚体不自洽）。
4. **自由偏航漂移是同一个根因的另一个面**：无指令 60 s，robot11 真值 yaw **+18.4°**（台架）/
   **+8.20°**（§L 整栈）；塌成单刚体后 **+0.02°**（台架）/ **+0.006°**（整栈，LIO −0.091°），
   与默认模型（−0.24° / −0.154°）同量级 ⇒ **漂移停了**。
5. **被否掉的修法（有实测理由）**：把插件 `<update_rate>` 从 100 Hz 提到 1000 Hz（= 每个物理步
   都重设速度）确实把角速度从 1.03% 提到 7.8%，但**同一次改动让车"爬上"了实体障碍**：
   纯物理正前方盲推从 0.408 m 变成 **2.565 m**、底盘 z 从 0.152 m 升到 **0.400 m**
   ⇒ 场地障碍不再是障碍 ⇒ **不采用**（这正是"必须做反向验收"的价值）。
6. **新的验收口径已接线**（§L.9 第 1 项的建议）：`nav_goal_forensics.py` 新增
   `--preflight-reach`（发目标**之前**先用绕过 nav2 的盲推量出"这个出生点物理上能走多远"：
   本节实测 **前 0.3992 m / 后 1.4370 m**）、`--reach-gate`（目标超出实测可达区就**不发目标**，
   在 JSON 里记 `goal_gate`）、`--goal-yaw-only-deg`（原地转目标）与 `--dump-traces`
   （把真值/里程计完整时间线落盘，"无指令漂移"只能从它读）。

### M.1 为什么先做一个"只有 Gazebo + 底盘插件"的台架

整栈跑（Gazebo + LIO + nav2 + 雷达 30000 条射线 @10 Hz）一次 ~3 min、RTF ≈ 0.35；
要判"角速度通道"必须做**变体矩阵**（十来种改法 × 前后对照），整栈跑不动。
台架（`tools/scripts/tiltmount/run_chassis_yaw_bench.sh` + `chassis_bench_mkworld.py` +
`chassis_yaw_probe.py`，本节新增并提交）只做一件事：把**同一个世界**加载进来，只放**一个模型** + 底盘插件，
摘掉 LIO/nav2/雷达（`--strip-sensors` 连 `<sensor>` 一起删），探针只订阅
`/odom_ground_truth`（= Gazebo 模型 WorldPose，逐条真值）与 `/joint_states`，只发 `/cmd_vel_chassis`。
一次 <40 s、RTF ≈ 1.00。

**台架本身的可信度由三条外部锚点钉住**（都在同一次会话里复现了 §L 的数）：

| 锚点 | §L 记的 | 台架实测 |
|---|---|---|
| robot11 正前方盲推（`vx=+0.25`）被挡住的极限 | **0.4199 m** | **0.4077 m**（RTF 1.00；`--push-wall 25`） |
| 默认模型同协议 | **0.5402 m** | **0.5401 m** |
| robot11 原样盲发 `wz=1.0` × 12 s | **+3.79°**（真值）/ +3.86°（LIO） | **+7.09°**（12 **仿真**秒、RTF 1.00；§L 那跑 12 **墙钟**秒、RTF≈0.35 ⇒ 折成仿真时间后同量级） |

> **量纲口径（必须写清）**：§L 的 `--push-time 12` 是**墙钟**秒，而它那跑 RTF≈0.35
> ⇒ 12 墙钟秒 ≈ 4.2 仿真秒。台架 RTF≈1.0 ⇒ 12 墙钟秒 ≈ 12 仿真秒。
> 所以"执行率"一律用**仿真时间**归一（下表的 `exec_ratio_sim`），跨跑可比。

### M.2 候选原因逐条判决（每条：测什么 / 结果 / 证据）

| # | 候选 | 判决性测量 | 结果 | 证据 |
|---|---|---|---|---|
| 1 | **驱动链/mixing**：`wz` 没走到和执行 `vx` 同一条通道（单位/缩放/几何参数） | ① 生成物与插件块逐字对比；② FK 复核 8 个关节的**轴**与轮碰撞的**圆柱轴**；③ 同一插件在"极简单 link box"上的表现 | **无 bug、驳回**：两个模型的 `<plugin name="mecanum_controller" filename="libgazebo_ros_planar_move.so">` 块**逐字相同**（`update_rate 100`、`cmd_vel:=cmd_vel_chassis`、`odom:=odom_ground_truth`）；FK 复核：`j2…j5` 的轴在 base_link 系里是 **`(0,0,-1)`（竖直）**、`j6…j9` 的轴与轮 cylinder 的轴**平行**（都是水平），轮底 **z=−0.10250 m** 是整车最低点；**同一个插件在极简单 link box 上能做到 99.9%**（+681.2°/687.5°）⇒ 通道本身没坏 | `.tmp_wz/static/{robot11,default}.sdf`、`tools/scripts/regress/robot11_weld_chassis.py --report-only`、`m` 系列 |
| 1b | **（本节新增）通道的"结构性上限"**：插件把同一个 `(v,ω)` 写给每个 link ⇒ 多刚体不自洽 | 台架：把同一台车的几何塌成单刚体前后对比（同一世界/出生点/命令） | **成立、是主因**：`+7.09° → +111.7…113.3°`（1.03% → 16.2~16.5%）；"全关节改 fixed"只到 **+21.3°（3.1%）**、"转向关节 fixed" 只到 **+16.7°（2.4%）** ⇒ 光"锁自由度"没用，**必须是单刚体**（插件那一步才自洽） | `.tmp_wz/bench/out/{w1,f3,w6,g1,h1}_*/probe.json` |
| 2 | **真轮子（4 个 cylinder 碰撞 + 8 个连续关节）在"顶"底盘** | ① 轮对地摩擦剂量-响应；② 零重力（完全无接触）；③ 轮子碰撞换成球/去掉 | **部分成立（第二限制项）**：`mu` 1.0/0.5/0.2/0.05/0.001 ⇒ `+7.09/+17.26/+54.75/+96.91/+126.80°`（**单调**）；零重力 ⇒ `+128.02°`；⇒ 摩擦把 18.6% 再压到 1.03%（**18×**），但它**不是**主因（主因见 1b，把摩擦拿掉也只到 18.6%） | `w1/w14/w13/w12/w3/w2_*` |
| 3 | **场地 mu / 质量 / 惯量不足** | ① 场地 mu 是两个模型共用的常量（`RMUL2026_world.world:85` = 1、torsional `use_patch_radius=1`+`patch_radius=0` ⇒ 扭转项实际为 0）；② 两车质量 9.56 vs 10.21 kg（同量级）；③ 角速度对 `wz` 的剂量-响应 | **驳回（不是"mu 太高"或"惯量太大"这种可调参数问题）**：同一场地、同一 mu 下默认模型能做到 5.8%，robot11 只有 1.03% ⇒ 差异来自**模型结构**；`wz` 剂量-响应在修好后线性（见 M.7） | `RMUL2026_world.world`、M.7 表 |
| 4 | **传感器/估计器伪影** | 真值（Gazebo WorldPose）与 LIO（`/odom`，含 IMU 陀螺）**两个独立估计器**同跑对比 | **驳回**：整栈盲发 `wz=1.0`：真值 **+35.42°** / LIO **+33.75°**；60 s 漂移：真值 **+0.006°** / LIO **−0.091°** ⇒ 两者一致 ⇒ **是物理，不是估计器** | `.tmp_tiltmount/m1_r11_blind_wz/`、`m5_r11_drift60/` |

### M.3 判决性测量 A：源码级机理 + 四个结构对照

`gazebo/physics/Model.cc`（gazebo-classic 11 分支，逐字）：

```cpp
void Model::SetLinearVel(const ignition::math::Vector3d &_vel)
{ for (Link_V::iterator iter = this->links.begin(); iter != this->links.end(); ++iter)
    if (*iter) { (*iter)->SetEnabled(true); (*iter)->SetLinearVel(_vel); } }
void Model::SetAngularVel(const ignition::math::Vector3d &_vel)
{ for (Link_V::iterator iter = this->links.begin(); iter != this->links.end(); ++iter)
    if (*iter) { (*iter)->SetEnabled(true); (*iter)->SetAngularVel(_vel); } }
```

而 `libgazebo_ros_planar_move.so` 的 `OnUpdate` 每次（`update_rate=100` ⇒ 每 10 ms 仿真时间）就是
`model_->SetLinearVel(...); model_->SetAngularVel(...)`（源码逐字见
`gazebo_plugins/src/gazebo_ros_planar_move.cpp`）。**纯自转**时它给每个 link 写的是
`v=0, ω=(0,0,wz)` —— 只有"整个模型是一个刚体、且绕**自身质心**转"时这才是自洽的速度场。

台架上把这句话变成四个对照（同一世界/出生点、`wz=1.0` × 12 仿真秒）：

| 结构 | Δyaw 真值 | 执行率（仿真时间） | 读法 |
|---|---|---|---|
| 极简 **单 link box**，无接触（自由落体） | **+681.25°** | **99.9%** | 单刚体 + 无外力 ⇒ 插件说的就是发生的 |
| **robot11 塌成单刚体**（本修法） | **+111.68…113.32°** | **16.2~16.5%** | 单刚体 + 真实接触 ⇒ 剩下的差额全是摩擦 |
| robot11 原样（13 link / 12 joint） | **+7.09°** | **1.03%** | 多刚体不自洽 + 摩擦 |
| robot11 原样，全关节改 `fixed` | +21.31° | 3.10% | 锁自由度**不能**替代单刚体 |
| robot11 原样，仅转向关节改 `fixed` | +16.69° | 2.43% | 同上 |
| robot11 原样，**零重力/无接触** | +128.02° | 18.6% | 多刚体结构的上限（≈ `I_base/(I_base+Σmᵢrᵢ²)`） |
| 默认模型（7 link / 6 joint） | +40.01° | 5.82% | 对照：它的结构上限是 +330.85°（48.1%） |
| 默认模型，零重力 | +330.85° | 48.1% | —— |

⇒ **同一台车、同一世界、同一命令**，唯一变量是"模型是不是单刚体"，
角速度执行率差 **15.7×**；而默认模型的结构上限（48.1%）比 robot11（18.6%）高，是因为它只有
6 个关节、要"掰回来"的 link 更少。

### M.4 判决性测量 B：接触/摩擦的剂量-响应（第二限制项）

台架、`wz=1.0` × 12 仿真秒、唯一变量 = 轮 link 的 `mu1/mu2`（`<surface><friction><ode>`）：

| 轮-地 `mu` | 1.0（默认） | 0.5 | 0.2 | 0.05 | 0.001 | 零重力（无接触） |
|---|---|---|---|---|---|---|
| Δyaw 真值 | **+7.09°** | +17.26° | +54.75° | +96.91° | **+126.80°** | **+128.02°** |
| 执行率 | 1.03% | 2.5% | 8.0% | 14.2% | 18.6% | 18.6% |

两条读法：
* **单调** ⇒ 摩擦确实是限制项之一（而且是"轮子真的存在"带来的）；
* **饱和在 18.6%** ⇒ 把摩擦**全部**拿掉也只能到多刚体结构的 18.6% ⇒ **摩擦不是主因**，
  主因是 M.3 的结构不自洽。这也是"只降 mu 不算修好"的定量依据。

### M.5 被否掉的修法：把插件 `update_rate` 提到 1000 Hz（有实测理由）

动机：插件每 10 ms 才重设一次速度，接触/约束有 10 个物理步去"掰回来"；提到 1000 Hz
（= 每个物理步都重设）应当让指令重新占上风。实测：

| 量 | 原样（100 Hz） | **1000 Hz** |
|---|---|---|
| 盲发 `wz=1.0` × 12 s 的 Δyaw（robot11） | +7.09° | **+53.80°** |
| 默认模型同改动（对照） | +40.01° | **+210.74°** |
| **纯物理正前方盲推 `vx=0.25` × 25 s** | **0.4077 m**（被障碍挡住，z 不变） | ⚠️ **2.5652 m，底盘 z 0.1522 → 0.4005 m（爬上了障碍）** |
| 60 s 自由漂移 | 0.308°/s | 0.297°/s（**没解决**） |

⇒ 它确实"让指令占上风"，但**把实体障碍也一起变成可攀爬的**（车速级 0.25 m/s 的盲推能爬 25 cm），
而且**不解决漂移** ⇒ **不采用**。

### M.6 采用的修法：把 `robot11` 的底盘塌成**一个刚体**（robot 槽位专属）

**做法**（`tools/scripts/regress/robot11_weld_chassis.py`；生成器
`robot11_make_sim_xacro.py --chassis rigid` 默认调用它）：

1. 从根 link `base_link` 对 `j2…j11` 做 FK（关节 origin 全是字面数字 ⇒ 可精确复算）；
2. `l2…l11` 的 `<collision>` / `<visual>` 的 pose **逐条**变换到根 link 系后挂到根 link 上
   （**几何一块都不改**，只改"装在哪条 link 上"）；
3. 它们的 `<inertial>` 用**平行轴定理**合成到根 link（`M=9.4121 kg`、
   `COM=(0.003551,0.007121,0.073122)`、`Izz=0.126766 kg·m²`）；
4. 删掉 `j2…j11`；**保留** `livox_frame` / `imu_link` 及其固定关节
   （感知链的 TF 与两个传感器都挂在它们上面 ⇒ 雷达/IMU 的位姿、`base_link→livox_frame`
   的 30° 安装语义、`lio_tf_adapter` 的外参**一个字节都不变**）；
5. 去掉指向被塌掉 link 的 `<gazebo reference="…">` 材质块（否则 Gazebo 每个都报一次
   "unknown link"；材质由各 `<visual>` 自带的 `<color>` 承担）。

**自证（不通过就报错、不写文件）** —— 这两条都实际抓到过 bug：

| 检查 | 结果 |
|---|---|
| 碰撞几何 AABB **逐条**相等（塌陷前按各 link FK、塌陷后按根 link） | ✅ 113 条，min z 逐条相等 |
| 整车最低点必须落在**轮子**（cylinder）上 | ✅ **−0.10250 m**（= Phase 1 独立量到的"地面平面 z=−0.102499"） |
| 总质量 / 质心 / 惯量 | ✅ `9.4121 kg`（= 上游 11 条 inertial 求和）、`Izz 0.126766` |

> 自证为什么必须有第二条：AABB 那条比较的是"塌陷前 vs 塌陷后"，若 FK **静默退化**
> （实测踩过一次：`rpy="1.5708 -1.5708 0"` 被按"6 个数"解析 ⇒ 读成零向量），两边会**一起**错、
> 检查照样通过。加上"最低点必须在轮子上"这条**不依赖 FK 正确性**的物理锚点后，
> 那次退化当场报错。

**为什么这不是"为了跑通而造假"**：本栈**没有任何**轮子/云台控制器（`.xacro` 自己写着
"上游 j2..j9 是 continuous 但我们不加轮子控制器"），也就是说这 12 个关节**在运行期没有任何
执行机构**——它们唯一的作用就是**与 planar_move 较劲**。塌成单刚体后：
质量、质心、惯量、碰撞几何、足迹、雷达/IMU 位姿**逐项不变**，丢掉的只有"12 个不受控自由度"。

### M.7 验收（整栈、同协议、逐项实测）

命令模板（§L 的原款，`--settle 25 --duration 6 --goal-wait 0`）：

```bash
tools/scripts/tiltmount/run_nav_goal_forensics.sh <tag> --settle 25 --duration 6 --goal-wait 0 \
    --push-wz 1.0 --push-time 12 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
```

| 项 | 修前（§L 同协议） | **修后（本节，`m1_r11_blind_wz`）** | 默认模型（同会话 `m2_def_blind_wz`） |
|---|---|---|---|
| 盲发 `wz=1.0` × 12 s：真值 Δyaw / LIO | **+3.79° / +3.86°** | **+35.42° / +33.75°**（9.3×） | +27.35° / +26.88° |
| 执行率（对 687.5°） | 0.55% | **5.15%**（RTF 1.05 ⇒ 12.6 仿真秒） | 3.98% |
| 60 s 无指令自由漂移（真值 / LIO） | **+8.20°** / — | **+0.006° / −0.091°**（`m5_r11_drift60`） | −0.154° / −0.156°（`m6_def_drift60`） |
| 纯物理盲推：正前方 / 正后方（真值） | 0.4199 m / 1.5753 m | **0.3992 m / 1.4370 m**（`m10_r11_reach25`，25 s 腿） | 0.5402 m（§L） |

> **`robot11` 的角速度执行率现在**高于**默认模型**（+35.42° vs +27.35°，同一次会话、同一协议、
> 同样 RTF≈1.05）⇒ 验收口径"与默认模型可比"达成，且是**更好**的一侧。

**原地转目标（`--goal-yaw-only-deg 180`，`m7_r11_rotate180`）**：nav2 **立即**回
`controller_server: Reached the goal!`、`FollowPath SUCCEEDED`、`navigate_to_pose SUCCEEDED`，
`/cmd_vel` 与 `/cmd_vel_chassis` 的 `max|wz|` **= 0.0**、真值位移 0.00012 m、Δyaw 0.005°。
⇒ **不是"车转不动"，而是本仓 nav2 配置根本不检查朝向**：
`nav2_params_sim_controller_rpp.yaml` 用的是
`goal_checker_plugins: ["general_goal_checker"]` = **`PositionGoalChecker`**
（文件里自己核实过"该插件只声明 `xy_goal_tolerance`；`yaw_goal_tolerance` 在 1.1.20 的
PositionGoalChecker 里不存在"）⇒ 位置一到就算到。**登记为本节发现的缺陷**（见 M.9 ①），
按规则**不改** `src/rm_navigation/**/params`（只许走既有槽位覆盖）。

**转向类目标的真实验收 ⇒ 用"反向目标"**（RPP 的 `use_rotate_to_heading` 先原地转 180° 再走，
§L.5 就是它失败的）：

| 场景 | 修前（§L 记录，`mode:=nav`） | **修后（本节）** |
|---|---|---|
| **反向 2.0 m**（`--goal-forward -2.0`） | 45 条 plan、`vx` **恒 0**、`wz=-0.75` 连发 45 s、真值 **0.0069 m**、Δyaw **−6.03°**、`Failed to make progress` ×2（§L.8） | **`m9_r11_rev20`（不加闸门）**：**0 条 plan**（目标落在先验图西墙内）、8 次恢复、真值 0.0733 m、Δyaw **−179.7°**（恢复 Spin 这次真的把车转了 180° —— 角速度通道修好的直接后果）；**`m12_r11_rev20_gated`（加闸门）**：`in_reach=false` ⇒ **不发目标**（0 plan / 0 条 `/cmd_vel` / 位移 0） |
| **反向 1.2 m**（在可达区内；闸门放行） | 未测（§L 只有 2.0 m） | **`m11_r11_rev12_long`**：真值位移 **1.2102 m**（请求 1.2）、残余（map 系）**0.0794 m**（< `xy_goal_tolerance 0.25`）、**`Goal succeeded`**、`/cmd_vel_chassis` 的 `max|vx| = 0.9624`、`n(vx>0.01) = 121`、`collision ahead` 14 条、恢复 4 次 |
| **原地转 180°**（`--goal-yaw-only-deg 180`） | 未测 | **`m7_r11_rotate180`**：nav2 **立即** `Reached the goal!`（`FollowPath SUCCEEDED`、`/cmd_vel` 与 `/cmd_vel_chassis` 的 `max\|wz\| = 0.0`、真值位移 0.00012 m、Δyaw 0.005°）⇒ **本仓 nav2 根本不查朝向**（`PositionGoalChecker`，见 M.9 ①），所以"原地转目标"在本仓配置下**测不出**转向能力；转向能力由上面两行的反向目标体现 |

**契约（9 个关键话题的发布者数）**：在一次 `mode:=nav` 的 robot11 会话里逐个 `/topic info` 实测
`/livox/lidar`、`/livox/lidar/pointcloud`、`/cloud_registered`、`/segmentation/{ground,obstacle}`、
`/scan`、`/odom`、`/local_costmap/costmap`、`/global_costmap/voxel_grid` **全部 = 1**（与 §K 的口径一致；
本节只改 URDF，节点集合与话题一个都没动）。

**反向验收（本节反向验证）**：`effective.txt` 逐项回读 = `robot_radius 0.300`、
局部/全局 `inflation_radius 0.60/0.65`、`regulated_linear_scaling_min_radius 0.9`、
`use_rotate_to_heading True`、`obstacle_near_ground_m 0.05` ⇒ 限速器与近地剔除**都还在**（未受本节改动影响：
本节的改动只落在机器人的 URDF/xacro，一行参数/launch/节点集合都没动）。

### M.8 新的验收口径：**盲推可达性前置判据 + 闸门**（§L.9 第 1 项的建议，已接线）

**为什么**：§L 的主结论是"`--goal-forward 2.0` 物理上不可达"（正前方只有 0.42 m）。
那么"发目标 → 看车动不动"这种验收在**不可达的目标**上永远是红叉，而红叉的原因与控制器无关。
⇒ 规定：**任何基于目标的验收，必须先跑一次"盲推可达性探针"**，目标必须落在实测可达区内。

**怎么用（两条命令）**：

```bash
# ① 可达性探针（发目标之前；绕过 nav2，只有盲推）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m10_r11_reach25 --settle 25 --duration 6 \
    --goal-wait 0 --preflight-reach 0.25 --preflight-time 25 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
#   → forensics.json 的 reach = {fwd_m, back_m, fwd_dyaw_deg, back_dyaw_deg, t_last_motion_s, …}

# ② 目标验收（把 ① 的文件交给闸门；目标超出可达区就**不发目标**并记 goal_gate）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m8_r11_rev12 --settle 30 --duration 30 \
    --goal-forward -1.2 --goal-wait 60 --reach-gate .tmp_tiltmount/m10_r11_reach25/forensics.json \
    -- world:=RMUL2026 mode:=nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
```

**判据（本节实测的取值）**：`reach = {fwd_m: 0.3992, back_m: 1.4370}`（`m10_r11_reach25`，
25 s 腿、真值、RTF≈1.0）⇒ 本出生点的可达区是"**前 0.40 m / 后 1.44 m**"，
`--goal-forward 2.0`（或 `-2.0`）**都在区外**，闸门会直接判 `in_reach=false` 并且不发目标。
新增的 `--goal-yaw-only-deg`（原地转目标）与 `--dump-traces`（完整真值/里程计时间线，
"无指令漂移"只能从它读）也在同一次提交里。

### M.9 顺带查到的两件（与本节结论无关，但必须登记）

① **本仓 nav2 不检查目标朝向**（`PositionGoalChecker`）：位置进入 `xy_goal_tolerance`
（0.25 m）就 `Reached the goal!` ⇒ **"原地转 180°"这类目标在本仓配置下会立即成功**（实测
`max|wz| = 0.0`、Δyaw 0.005°）。要让"原地转"变成真判据，需要把 `goal_checker_plugins` 换成
`nav2_controller::SimpleGoalChecker`（它同时有 `xy_goal_tolerance` 与 `yaw_goal_tolerance`）。
**本节没改**（参数在 `src/rm_navigation/**/params`，按规则只许走既有槽位覆盖）。

② **`/odom_ground_truth.twist` 是"指令回显"，不是实测速度**：planar_move 的
`OnUpdate` 挂在 `ConnectWorldUpdateBegin` 上 —— 它**先** `SetAngularVel`、**再**在同一个回调里
`UpdateOdometry()` 读 `model_->WorldAngularVel()`。所以 ① 里 `twist.angular.z` 恒等于
**刚设进去的命令值**（实测：真值 yaw 12 s 只转 7.09°，而该字段中位 = 57.296°/s = 1.0 rad/s）。
**要量"到底转了多少"只能用位姿差（`/odom_ground_truth.pose` 或 `/odom`），不能信 twist。**

### M.10 未验证 / 诚实清单（本节）

1. **没有重跑"修前"的整栈基线**（§L 的 +3.79° / +8.20° 直接引用；同一模型的**台架**复现见 M.1，
   整栈的"修后"见 M.7）。要严格同期对照，可 `robot11_make_sim_xacro.py --chassis articulated`
   回退后再跑一次同样的命令。
2. **反向 2.0 m 目标本身没做"修前 vs 修后"的同协议对照**（§L 的修前读数：45 条 plan、
   `vx` 恒 0、`wz=-0.75` 连发 45 s、真值 0.0069 m、Δyaw −6.03°）；本节只跑了"修后"。
3. `wz` 剂量-响应（0.3/0.6/1.0/1.5）只做了台架的单点 `wz=1.0` 与整栈的 `wz=1.0`
   （`wz=-0.75` 只由 RPP 在反向目标里给出），**没有**做完整扫描。
4. 单刚体修法**丢掉了 12 个不受控自由度**（j2…j11）：将来若要给轮子/云台加真控制器，
   必须回退到 `--chassis articulated` 再改造（生成器留了开关）。
5. 塌陷后的接触是"4 个 cylinder 刚体接触"（不再是"4 个自由轮"）：所以**平动**的摩擦特性也变了
   （正前方极限 0.4077 → 0.3992 m，−2%；反向 1.4370 m 与 §L 的 1.5753 m 同量级，
   差异来自 25 s 腿的时间上限 + 起始点不同）。这一点**没有**做系统标定。
6. `m7` 的"原地转"结论依赖"`PositionGoalChecker` 不查朝向"这一条**源码/参数级**判读
   （不是本节实测出来的"转向能力"）；真正的转向能力由反向目标那一跑体现。

### M.11 回退

| 想退掉什么 | 怎么做 |
|---|---|
| **单刚体底盘**（模型行为） | `python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated` ⇒ 生成物的**模型体去注释后与 2026-10-09 的 HEAD 逐字节相同**（实测两份都是 **35025 B**、`diff` 为空；整份文件只多出文件头那段说明 23 行）；install/ 里是逐文件符号链接，通常不用重编 |
| 塌陷工具本身 | `rm -f tools/scripts/regress/robot11_weld_chassis.py`（生成器只在 `--chassis rigid` 时 import 它；把默认改回 `articulated` 即可） |
| 新增的验收工具 | `git revert <第 2 个 commit>`：`nav_goal_forensics.py` 的 `--preflight-reach/--reach-gate/--goal-yaw-only-deg/--dump-traces` 都是**新增开关**，默认关 ⇒ 回退不影响 §L 的任何旧命令 |
| 本文档 | `git revert <第 3 个 commit>` |

### M.12 复现命令（复制即可）

```bash
# ① 生成模型（默认 rigid；articulated = 回退档，模型体去注释后逐字节等于 2026-10-09）
python3 tools/scripts/regress/robot11_make_sim_xacro.py                    # rigid
python3 tools/scripts/regress/robot11_make_sim_xacro.py --chassis articulated
# ② 只做自证（不写文件）：几何 AABB / 最低点在轮上 / 质量·Izz
python3 tools/scripts/regress/robot11_weld_chassis.py \
    --in src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro --report-only
# ③ 整栈：盲发角速度（§L 同协议）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m1_r11_blind_wz --settle 25 --duration 6 \
    --goal-wait 0 --push-wz 1.0 --push-time 12 -- world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
# ④ 整栈：60 s 无指令漂移（要 --dump-traces 才拿得到时间线）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m5_r11_drift60 --settle 30 --duration 30 \
    --goal-wait 0 --dump-traces -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
# ⑤ 可达性前置判据 + 闸门（M.8）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m10_r11_reach25 --settle 25 --duration 6 \
    --goal-wait 0 --preflight-reach 0.25 --preflight-time 25 -- world:=RMUL2026 mode:=slam_nav \
    lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
# ⑥ 原地转目标（注意 M.9 ①：本仓 nav2 不查朝向 ⇒ 会立即 SUCCEEDED）
tools/scripts/tiltmount/run_nav_goal_forensics.sh m7_r11_rotate180 --settle 30 --duration 5 \
    --goal-yaw-only-deg 180 --goal-wait 45 -- world:=RMUL2026 mode:=nav lio:=small_point_lio \
    robot:=robot11 spin_speed:=0.0 gui:=False
# ⑦ 台架（变体矩阵，一次 <40 s；已提交：tools/scripts/tiltmount/run_chassis_yaw_bench.sh）
#    先备好模型 SDF：xacro <模型>.xacro <args> | gz sdf -p /dev/stdin > /tmp/<模型>.sdf
tools/scripts/tiltmount/run_chassis_yaw_bench.sh t_r11_weld --model /tmp/robot11_welded.sdf \
    --variant baseline --wz 1.0 --push-wall 12 --settle 8 --add-jsp
#    --variant {baseline,float,nofric,nowheelcol,spherewheel,fixsteer,fixwheel,fixall,weld,baseonly,boxwheel}
#    → .tmp_tiltmount/<tag>/bench/{world.world,probe.json,gzserver.log}
```
