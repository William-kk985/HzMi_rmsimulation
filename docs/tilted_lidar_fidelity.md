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
