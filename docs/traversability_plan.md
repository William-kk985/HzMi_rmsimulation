# 3D 派生可通行性（坡度 / 落空 / 净空）设计计划 —— **plan only，不含任何代码或参数改动**

> **本文是什么**：一份**设计文档**，回答"2D 占据栅格表示不了的三件事（斜坡、落空/坑、限高），
> 要用什么 3D 表示、怎么变成 nav2 能吃的层、怎么用我们的 STL 真值验证、怎么在 RViz 里看、
> 分几步做、先不做什么"。**本文不含任何代码/配置改动**，也没有改任何跑起来的文件。
>
> **本文不是什么**：不是实现记录，也不是实测报告。文中每个"数字"都标了出处：
> **[实测·已有]** = 本仓文档里已经量过的值（给文件 + 节号）；**[本次核对]** = 写本文时为对齐口径
> 现算的 STL 几何量（口径与命令都写出来，可复跑）；**[待测]** = 还没有这个数，写清"要量什么、怎么量"。
>
> **状态标记**：**【事实】** = 有出处可核对；**【推断】** = 我的判断（给出理由，但没有实测支撑）。
> 上下游背景见 `docs/mapping_2d_from_cloud.md`（3D→2D 判据与坡道段）、`docs/ground_segmentation_slots.md`
> （地面分割与 p2l 高度带）、`docs/worlds.md`（RMUC2026 场地资产）、`docs/algorithm_matrix.md`（维度决策）、
> `docs/architecture.md` §3.2.7（2.5D 落位）。

---

## 0. 一句话结论

**【推断】** 我们**不需要**先上"3D 地图"这个东西。需要的顺序是：

1. **2.5D 高程图（elevation map）先做** —— 因为我们要的三个量（坡面法向、相对局部地面的高差、每格净空）
   全都是"每个 (x,y) 一个或几个标量"的量，**本来就是 2D 网格**；nav2 的接口也是 2D 网格 ⇒ 转换成本最低。
2. **一个轻量"落空/负障碍"判据** 挂在同一张高程图上（不需要新的数据结构）。
3. **OctoMap 只在需要"真·三维体积判断"时才上**（多层/悬挑/隧道里的通过性检查）；
   ESDF 体素（voxblox / nvblox / FIESTA）**现在不做**（那是给 3D 规划器/MPC 用的，我们不上 3D 规划器）。
4. **动作** 分两档：**写 cost（可通行性）** 用 nav2 costmap 图层（现成机制）；
   **触发行为**（"看见了但我不走 / 我慢速试探"）用 BT 条件节点 + 自带行为树 XML（`bt_navigator` 现成机制）。
   两档**不要混在一个东西里**。
5. **我们台架真正的独有资产不是 3D 传感器，是 STL 真值**：`RMUC2026.stl` 可以把
   **坡度/净空/落差的真值图离线算出来**，与传感器派生层做**逐格比对**（TP/FP/FN 计数）。
   这在实车上**做不到**。

---

## 1. 问题定义：三件事、一张表、一条"二义性"论证

### 1.1 三件要"认出来并作出反应"的事

| # | 能力 | 要回答的问题 | 现在谁在回答（如果有） | 现状 |
|---|---|---|---|---|
| A | **坡度（可行驶斜面）** | 这一格是"能开上去的斜面"还是"墙 / 台阶沿"？坡度多少、该不该限速？ | `tools/scripts/mapping/pcd_to_nav2_map.py` 的 `--slope-limit 25°`（**离线、只用于出一张 pgm**） | 只有"free/occupied"的二值结果，**坡度值本身没有被发布出去** |
| B | **落空 / 负障碍（坑、边沿）** | 这一格脚下的地面是不是**没了 / 掉了**？ | 没有 | **空缺** |
| C | **净空高度（限高）** | 这一格**头顶**还有多少空间？够不够本车过？ | 没有 | **空缺** |

图例：A 现有的是"离线一次性判据"，B/C **完全空缺**（`docs/algorithm_matrix.md` §一.1 的 2.5D 空缺行：
"待建 `src/rm_perception/rm_elevation_map/` + `src/rm_navigation/rm_costmap_layers/`"）。

### 1.2 一张 2D OccupancyGrid **能**与**不能**表示什么

2D 占据栅格每格只有三种值：**占用 / 空闲 / 未知**（本仓口径 `0=占用 / 205=未知 / 254=空闲`，
`mode: trinary`、`free_thresh 0.25`、`occupied_thresh 0.65`，见 `docs/mapping_2d_from_cloud.md` §5.2）。**【事实】**

| 要表达的东西 | 2D 栅格能表示吗 | 为什么 / 证据 |
|---|---|---|
| 竖直墙、柱子、台阶立面 | ✅ 能（占用） | 高差大、法向陡 ⇒ 任何高度阈值判据都能抓住 |
| 0.2 / 0.3 m 台阶（"上不去"） | ✅ 能（占用） | 相对局部地面高差 0.2/0.3 m > 阈值 0.15 m（`docs/ground_segmentation_slots.md` §5.2：0.2/0.3 m 平台顶面 100% 判障碍）|
| **可行驶的斜坡（23°）** | ⚠️ **能表示，但一旦表示错就致命** | 坡度**没有**独立字段；只能被"降级"成二值。若判成**占用** ⇒ 规划器认为路被堵死；若判成**空闲** ⇒ 又丢掉了"这格有多陡、要不要限速" |
| **落空 / 负障碍（地面塌下去）** | ❌ **原理上不能** | 栅格记录的是"这一格 (x,y) 有没有障碍"，**不记录"脚下有没有地"**。一个 −0.2 m 的坑在俯视图上就是一块"没有回波的地" —— 与"墙后面看不见"、"超出量程"、"传感器盲区"**完全不可区分** |
| **净空高度（头顶空间）** | ❌ **原理上不能** | z 被压掉了。同一个 (x,y) 的"地面 + 头顶 1.2 m 处的横梁"与"地面 + 自由天空"在栅格里**是同一格** |
| 坡度值的连续量（10°/23°/35°） | ❌ **不能** | 只有三值，没有连续量；连"代价"都只能靠 inflation 间接表达 |

### 1.3 两条必须写清楚的"事实 + 后果"

**① 把可行驶的斜坡标成"占用"是错的（不是保守，是错）。**
**【事实】** `docs/mapping_2d_from_cloud.md` §4.1/§6.1：坡面区域在旧图（`/scan` 累积图 A）里
**1680/1680 格 = 100% unknown**，不是"判成占用"，是**根本没看到**；规划器对坡道/走廊的 5 个目标
**2/5 直接拒绝**。**【事实】** 同一份文档 §5.1/§10.4：新工具（点云投影 + 当地面法向闸）把坡面判成
**98.8% free**（1660/1680，其余 20 格是台阶沿），规划器 **5/5** 给出合法路径。
**【推断】** 所以"标成占用"与"标成未知"在这里是**同一类错误**——都让规划器**拒绝**这块地。
"保守"这个词在可通行性上是**误导**：把可走的地方标成不可走，等于**凭空删掉一条通路**。

**② "unknown 就是规划器拒绝的原因"——已经量化并已修。**
**【事实】** 坡面 100% unknown（1680/1680）→ 改成点云投影后 98.8% free → 规划器 2/5 → 5/5
（`docs/mapping_2d_from_cloud.md` §6.1 / §10.4 的三行表）。这就是"为什么必须有 3D 派生的信息"的
**已经付过账的证据**，不是设想。

**③ 为什么"认出 → 决定 → 行动"比一张 2D 图要多。**
**【推断】** 一张 2D 栅格能承载的只有"这一格能不能被压过去"。而这三件事需要的是**三段时间上不同的信息**：

```
感知（3D 点云） → 表示（2.5D 层：坡度/落差/净空） → 判据（阈值 + 上下文） → 决策（代价 or 触发） → 动作（规划/限速/BT）
```

2D 图只接在"决策 → 动作"这一段上。把判据塞进"离线出一张 pgm"（现在 `pcd_to_nav2_map.py` 的做法）
**能用，但只能选一次、且不能反映实时变化**（斜坡上有动态障碍、限高杆下面突然站了个人……）。
要"认出并作出反应"，缺的是**中间那段**（表示 + 判据 + 一条把判据送进 nav2 的路）。

---

## 2. 表示选型（带取舍表）+ 本机可用性核对 + 推荐

### 2.1 四个候选（以及"点云直投"这个非候选基线）

| 表示 | 存什么 | 能回答什么 | 成本 | 局限（**关键**） |
|---|---|---|---|---|
| **点云**（现在就有：`/livox/lidar/pointcloud`、`/segmentation/ground`、`PCD/<world>_spl.pcd`） | 一帧/一张点集，无拓扑、无免费空间 | "哪里有回波"；**法向、相对高度都可以现算** | **零**（已经在跑） | ① 无体素/无插值 ⇒ 稀疏处与盲区只能"未知"；② 每格统计要自己写；③ **无法回答"这一格是不是没有地"**（缺"反证"结构）；④ 密度随距离骤降 |
| **2.5D 高程图**（elevation map / grid_map） | 每个 (x,y)：`elevation`（地面高）、`elevation_variance`、可扩展层 `slope`/`roughness`/`free_height`/`traversability` | **坡度、粗糙度、台阶高差、净空**——我们三个需求里的 A 和 C，以及 B 的一半 | 中：一帧点云按 (x,y) 分桶 + 形态学/插值；`grid_map` 库现成 | ① **一个 (x,y) 只有一个地面高度 ⇒ 多层结构（桥 / 隧道 / 悬挑 / 头顶横梁）表达不了**（必须靠额外的"净空层"或第二张图打补丁）；② 需要**重力对齐的姿态**（否则"地面"跟着车滚）；③ 稀疏区必须外推 ⇒ 外推策略本身就是误报来源 |
| **OctoMap**（3D 概率八叉树） | 体素的**占据概率** + **射线清除**的自由空间 | **真·三维通过性**：任意 (x,y,z) 是否被占；**头顶有没有横梁**；多层结构；`projected_map` 直接给 2D | 高：8 Hz 点云做 ray casting + 树更新；内存随体积 | ① 分辨率越细代价越高；② **它回答的是"占据"，不是"可行驶"**——坡道在上面仍是一堆占据体素，**坡度/可走性还得另算**；③ 概率参数（hit/miss、clamping）是新的调参面 |
| **ESDF / TSDF 体素**（voxblox、nvblox、FIESTA） | 体素到最近表面的**距离场**（+ 梯度） | 连续距离、梯度 ⇒ 直接给 3D 规划器 / MPC / 主动探索 | 高（nvblox 要 GPU） | ① **它的消费者是 3D 规划器**，而我们的决策层明确不上 3D（`docs/algorithm_matrix.md` §8.2）；② 距离场同样不直接回答"坡度能不能上"；③ 又是一整套建图/融合/位姿依赖 |

**非候选基线（现在的实际做法）**：`tools/scripts/mapping/pcd_to_nav2_map.py` —— 离线把 PCD 投影成
一张 nav2 `pgm`。**【事实】** 判据是 `--normal-k 8` PCA 法向 ⇒ `--slope-limit 25°` 闸；
局部地面 = `--ground-cell 0.20 m` 粗格内可行驶面点的 `--ground-percentile 5`；
占用 ⟺ `z_max − 地面 > --height-threshold 0.15 m` **或**（坡度 > 25° 且 `z_max − 地面 > --slope-min-height 0.05 m`）；
`--ground-min-points 2`、`--ground-borrow-cells 1`、`--occ-min-neighbors 1`、`--occ-dilate-cells 1`、
`--fill-empty ground`、`--border unknown`（`--help` 与 §5.1 两处口径一致）。**【事实】**
它**只输出一张图**（没有把坡度/高度差留下来），而且**跑的时候才判**。

### 2.2 【事实】本机可用性核对（写本文时现场跑的）

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | wc -l                                  # 433 个包
ros2 pkg list | grep -iE 'octomap|grid|elevation|voxblox|nvblox|esdf|fiesta|voxel'
#  → 只有两行：nav2_voxel_grid、spatio_temporal_voxel_layer
apt-cache policy ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server \
                 ros-humble-octomap-rviz-plugins ros-humble-grid-map-ros \
                 ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin ros-humble-grid-map-costmap-2d
apt-cache search ros-humble-elevation          # → 无输出（Humble 没有 elevation_mapping 的 apt 包）
```

| 候选包 | `ros2 pkg list`（已装？） | apt 候选版本 | 结论 |
|---|---|---|---|
| `spatio_temporal_voxel_layer`（STVL，2.5D 体素） | ✅ **已装**（唯一在岗的 2.5D 环节） | — | **现成可用**，已做成槽位：`global_obstacle:=stvl`（默认）/`scan`/`none`、`local_obstacle:=stvl`/`stvl_both`/… |
| `ros-humble-grid-map-ros` + `-msgs` + `-costmap-2d` + `-rviz-plugin` | ❌ **未装** | **2.0.1-1jammy**（`ros-humble-grid-map` 系列一整排在源里，`apt-cache search ros-humble-grid-map` 命中） | **apt 一条命令可装**；Humble 有官方发布 |
| `ros-humble-octomap` / `-msgs` | ❌ **未装** | 1.9.8 / 2.0.1 | apt 可装 |
| `ros-humble-octomap-server` | ❌ **未装** | 2.3.1 | apt 可装（**注意**：包名叫 `octomap_server` 的那个 2.x 才是 ROS 2 的 `octomap_server` 节点包） |
| `ros-humble-octomap-rviz-plugins` | ❌ **未装** | 2.1.1 | apt 可装 |
| `elevation_mapping`（ANYbotics）/ `elevation_mapping_cupy`（leggedrobotics） | ❌ **未装、apt 也没有** | **无 apt 候选**（`apt-cache search ros-humble-elevation` 空） | **只能源码编译**；cupy 版还要 CUDA/GPU。本机是仿真工作站、无 GPU 相关依赖证据 ⇒ **不建议现在引** |
| voxblox / nvblox / FIESTA | ❌ 全无 | 无 apt 候选 | 只作**将来**候选登记 |

上游出处（写本文时逐个 `curl -o /dev/null -w %{http_code}` 复核过，均 **200**）：
[ANYbotics/grid_map](https://github.com/ANYbotics/grid_map)、
[ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping)、
[leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)、
[OctoMap/octomap_mapping](https://github.com/OctoMap/octomap_mapping)、
[OctoMap/octomap](https://github.com/OctoMap/octomap)、
[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)、
[nvidia-isaac/nvblox](https://github.com/nvidia-isaac/nvblox)、
[HKUST-Aerial-Robotics/FIESTA](https://github.com/HKUST-Aerial-Robotics/FIESTA)。
**这些上游描述本身是外部资料**（未经我们实测），只作"要装什么、有没有 ROS 2 分支"的依据。

### 2.3 引用到的外部来源（URL）

| 项 | URL | 我们关心的点 |
|---|---|---|
| grid_map 库本体 | <https://github.com/ANYbotics/grid_map> | 多层 2D 网格（`elevation`/`slope`/`roughness`…），Humble 有 apt 发布 |
| grid_map RViz 插件 | <https://index.rosdabbler.com/p/grid_map_rviz_plugin/> | `rviz_plugin` 单独一个包名 ⇒ 才知道要装哪个 |
| elevation_mapping_cupy（ROS 2 分支 README） | <https://raw.githubusercontent.com/leggedrobotics/elevation_mapping_cupy/ros2/README.md> | 有 `ros2` 分支、release v2.1.0（见 <https://github.com/leggedrobotics/elevation_mapping_cupy/releases/tag/v2.1.0>） |
| octomap_mapping（= `octomap_server` 节点所在） | <https://index.ros.org/r/octomap_mapping/#humble> | Humble 有条目 ⇒ 有 apt 发布 |
| octomap_rviz_plugins | <http://mirror-ap.wiki.ros.org/octomap_rviz_plugins.html> · <https://index.rosdabbler.com/r/octomap_rviz_plugins/> | 看八叉树要另外装这个插件包 |
| nvblox（GPU ESDF/TSDF） | <https://github.com/nvidia-isaac/nvblox> | 论文出处：nvblox, ICRA 2024 |

### 2.4 **推荐（带理由）**

**【推断】按这个顺序落地，且第一步不引入任何新依赖：**

1. **先做"2.5D 高程图 + 三个派生层"，但先只做离线**（Python + `numpy`/`scipy`，
   复用 `pcd_to_nav2_map.py` 已经调好的口径：0.05 m 格、0.20 m 粗格、p05 分位、`k=8` 法向）。
   理由：① 三个需求全都能在这套结构上表达；② 口径与**已经验收过的**工具一致 ⇒ 结果可对账；
   ③ 不装包、不动栈 ⇒ 风险≈0。
2. **数据结构用 `grid_map`（apt 2.0.1）当"格式"，不当"运行时"**：
   离线产出 `.npz`（自用）**并且**能导出 `grid_map_msgs/GridMap`（互操作用）。
   理由：将来别人（或 RViz）要看得懂，`grid_map` 是事实标准；但现在没有理由为了它引入一个运行时节点。
3. **实时只在"确认离线口径站得住之后"接**：优先走两条**已有**的路子，
   而不是新写 costmap 插件 ——（a）把离线层烘进一张 `<world>_slope.pgm` 当**第二个 `static_layer`**；
   （b）`local_obstacle:=cloud`（现成的 `obstacle_cloud_layer` 直投点云）+ 高度带参数。
   真正的新插件（`nav2_costmap_2d::Layer`）**等到离线层被 STL 真值验证过再写**。
4. **OctoMap 只在"真·三维体积判断"成为刚需时才装**：判据是"出现一个场景，2.5D 层无论怎么调都把
   可走的路判成不可走（多层/悬挑/隧道）"。**【事实】** 就 RMUC2026 这份网格而言，这个场景**不存在**
   （见 §5.3：整个场地**没有任何一个列有"头顶表面"**）。
5. **ESDF/MPC 不做**（理由见 §7.3）。

---

## 3. 三个派生层的设计（数据源 → 判据 → 输出 → 怎么进 nav2）

### 3.0 先说三个层共同的"现成积木"与"真正缺的东西"

**【事实】已有的积木**（都不用重写，用之前先核对一遍）：

| 积木 | 位置 | 对三层的作用 |
|---|---|---|
| 地面/障碍二分的**槽位** `<ground>`（`linefit` 默认 \| `patchwork`） | `bringup_sim.launch.py` 的 `ground` 参数；契约见 `docs/ground_segmentation_slots.md` §2 | 给"哪些点是地面"一个**官方来源**；`/segmentation/ground` 就是我们要的"地面点" |
| **代价图层槽位** `global_obstacle` / `local_obstacle` + `LOCAL_OBSTACLE_LAYER_TABLE` / `GLOBAL_OBSTACLE_LAYER_TABLE` | `src/rm_navigation/rm_navigation/launch/navigation_launch.py:50-62`（真值表）、`:86-106`（`build_param_substitutions` → `RewrittenYaml` 的 `enabled` 开关） | **加"一层数据源"的现成机制**：新层只要有自己的 `enabled` 键与参数段，就能挂进这张表；静态校验脚本 `tools/scripts/regress/local_obstacle_truth_table.py` 会复用同一份逻辑 |
| **STVL**（3D 体素 → 2D 代价） | apt 第三方；参数 `nav2_params_sim_*.yaml` 的 `stvl_layer`；`min_obstacle_height: 0.0`、`max_obstacle_height: 2.0` | 已经是"2.5D 环节"；**高度带就是它的语义边界** |
| `obstacle_layer`（吃 `/scan`） | 同上，`max_obstacle_height: 2.0`、`min_obstacle_height: -0.02` | 2D 主障碍源 |
| `obstacle_cloud_layer`（点云直投，不经 p2l） | 同上，槽位 `local_obstacle:=cloud\|both` | 已经能"绕过 p2l 高度带" |
| `pcd_to_nav2_map.py`（离线 3D→2D） | `tools/scripts/mapping/` | **三层判据的第一版全都可以先在这里试**（离线、零风险） |
| 膨胀层 | `inflation_radius` 局部 0.5 / 全局 0.55、`robot_radius 0.22` | 任何"致命格"最终都靠它变成可规划的代价梯度 |

**【事实】真正缺的只有一样：判据本身（把高程/法向/净空变成"代价或触发"的那段代码）。**
现在仓库里**没有**任何节点/插件在算坡度、落差、净空。`docs/algorithm_matrix.md` 把这两块登记为
"**（空缺）**…待建 `src/rm_perception/rm_elevation_map/` + `src/rm_navigation/rm_costmap_layers/`"，
`docs/architecture.md` §3.2.7 也给了同样的落位建议（生产端 `rm_elevation_map`、消费端 `rm_costmap_layers` 的
`nav2_costmap_2d::Layer` 插件）。**【推断】** 这两个包名是**既有约定**，应当沿用。

---

### 3.1 坡度层（slope / traversability）

| 项 | 设计 |
|---|---|
| **数据源** | **首选** `/segmentation/ground`（已是"地面点"，`linefit`/`patchwork` 同契约；`docs/ground_segmentation_slots.md` §2 的逐字段契约表）——但它**每帧只有约 700~900 点**（**【事实】** `docs/issues_and_findings.md` #25 的更正："`/segmentation/ground` … 实测 ground 只有 ~700 点（≈11 KB）"）⇒ 单帧太稀，必须**多帧累积**或**先验打底**。<br>**次选** `/livox/lidar/pointcloud`（原始点云，密度足够，但含墙/台阶，要靠法向闸分）<br>**离线首选** `PCD/<world>_spl.pcd` / `PCD/<world>.pcd`（已有，且 `pcd_to_nav2_map.py` 口径已调好）<br>**⚠ 共同前提**：必须在**重力对齐**的系里算（`livox_frame` 的 `rpy = 0 0 0`，`docs/ground_segmentation_slots.md` §3）或 map 系；**不能**用挂在倾斜车体上的本体 z |
| **判据** | 两个都要，因为它们回答不同的问题：<br>① **点级法向**（`k=8` PCA）与重力夹角 θ；<br>② **格级梯度** `‖∇z‖` 的 `atan`（对 0.05 m 格要做 3×3 平滑，否则量化噪声把坡度抬到 45°+）。<br>写进层里的应是**连续坡度值（deg）**，不是二值 |
| **阈值（**这里有个必须先拍板的问题**）** | **【事实】** 场地上真实的斜面：[实测·已有] 23°（`docs/mapping_2d_from_cloud.md` §4.1 的 `23–24°`）、12.8°（同表"浅坡 12.8°"）；`pcd_to_nav2_map.py --slope-limit` 默认 **25°**。<br>**【推断】** ⇒ **25° 这个闸只给 23° 留了 2° 余量**，"可行驶"与"不可行驶"的分界就压在真实特征上。**这不是一个能拍脑袋定的阈值**：它应该由"本车实测能上多少度"决定，而**这个数我们没有**（见 §8 第 4 条 **[待测]**）。<br>**【推断】建议的初始形态**：**三段式**而不是一刀切 —— `θ ≤ 15°` 正常通行；`15° < θ ≤ 25°` **高代价 / 限速**（保留可通行性，让代价梯度自己选平路）；`θ > 25°` 才致命。这样 23° 坡变成"能走但优先绕"而**不是"禁止"** |
| **输出** | （a）`slope` 层（float，deg）；（b）由它派生的 `cost` 层（0~252 的连续代价）或 `speed_limit` 层；（c）**决策点清单**（坡度 > 阈值的格，给 BT / 日志用） |
| **怎么进 nav2** | **路线 1（先做，零代码）**：离线把 `slope` 按阈值分档烘进一张 `<world>_slope.pgm` ⇒ 作为**第二个 `static_layer`**（nav2 的 `static_layer` 可以挂多个实例，各给一个 `map_subscribe_transient_local` 话题）；或用 `grid_map_costmap_2d`（apt 包，**现成**的 "GridMap → costmap_2d 格式"接口）<br>**路线 2（后做）**：写 `nav2_costmap_2d::Layer` 插件（落位 `src/rm_navigation/rm_costmap_layers/`），注册进 `local_costmap`/`global_costmap` 的 `plugins` 列表 + `plugin: "rm_costmap_layers::SlopeLayer"`；`enabled` 键按 `navigation_launch.py` 的 `_enabled_expr` 机制挂到 `local_obstacle`/`global_obstacle` 的真值表上（**不改契约**） |
| **已知的坑** | **【事实】** `pcd_to_nav2_map.py` §5.4 第 1 条：**不加法向闸、直接用"粗格内 z 的 p05"当局部地面 ⇒ 墙脚那些点的 p05 就是地面 ⇒ `dz≈0` ⇒ 整面墙被判 FREE**（实测 `/scan` 图里 1493 个占用格有 **1418 个**被这样吃掉）。**【推断】** 同一个坑在"坡度层"上会以另一种形式复现：**如果先用墙点估地面、再算坡度，得到的"地面"本身是墙**。⇒ 坡度层的**点级法向闸是必需项**，不是可选优化 |

---

### 3.2 负障碍 / 落空层（negative obstacle）

| 项 | 设计 |
|---|---|
| **数据源** | 同一张高程图（原始点云 + 局部地面）。**额外**需要"**反证**"：某一格**没有**地面回波，而**邻格有** |
| **判据（三选一或组合）** | ① **格内高差**：`z_max(格) − z_min(格) > Δ_drop` 且 **`z_min` 明显低于邻域地面** ⇒ 该格是"边沿/掉落"；<br>② **"无地面回波 + 邻域有地面"**：某格的局部地面估不出来（点数 < `--ground-min-points`），但半径 `r` 内邻格能估出地面，**且该格在激光视线内**（不是被遮挡）⇒ 判"地面缺失"；<br>③ **range image 上的遮挡/断裂**：把点云按激光的 (az, el) 展开成 range image，找"深度跳变 + 跳变后面没有回波"的模式（这是激光原理上最强的落空信号）。<br>**【推断】** 单一判据都会炸；建议**①做粗筛、②做确认、线上只用"①∧②"** |
| **为什么 2D 不能做** | 2D 栅格只知道"这一格有没有障碍"，**不知道"这一格有没有地"**。−0.2 m 的坑在俯视图上就是"没有回波" ⇒ 与"被墙挡住"、"超出量程"、"传感器盲区"**同形**。**【事实】** 佐证：`docs/mapping_2d_from_cloud.md` §6.3 —— 走廊条带 2688 格**整块落在 A 图窗口之外**（"不是判成占用，是图里根本没有这块地"）；A 图在它自己覆盖到的区域里也有 4587/4644 格是 unknown。**"没数据"与"没地"在 2D 里无法区分**，这正是负障碍层存在的理由 |
| **误报风险（我们量过的）** | ① **地图边界**：把窗口外沿当成"落空" ⇒ 车永远出不了图。**【事实】** `pcd_to_nav2_map.py` 用 `--border unknown|occupied` 区分这两种口径（默认 `unknown`）。负障碍层必须显式处理边界：**边界格按"未知"而不是"落空"**；<br>② **传感器噪声/稀疏**：0.10 m 体素下采样后的远场只有零星点（`docs/mapping_2d_from_cloud.md` §5.4 第 3 条）⇒ 空旷处每格都可能"没有地面回波"；<br>③ **45 cm 近场历史**：**【事实】** `p2l` 的 `range_min` 曾是 **0.45**（`docs/issues_and_findings.md` #24 / `docs/algorithm_matrix.md` §七 2026-09-21 行），会造成"贴身 45 cm 内既看不见又被反向清成 free"；**现在已是 0.05**（`config/laserscan_params.yaml` 里两次下调：`0.2 → 0.12 → 0.05`），且 `docs/stvl_local_costmap.md` §168/§232 实测"45 cm 盲区在当前配置下不存在（贴墙 0.35~0.40 m 仍标到墙）"。**【推断】** ⇒ **今天**不该再把 45 cm 当参数，但如果将来误报率压不住而把 `range_min` 退回 0.15/0.2，**近场就会重新变成"落空误报源"**，这条回退档位必须与负障碍层一起登记；<br>④ **有坡的地方天然有高差** ⇒ 判据 ① 会在 23° 坡的坡脚/坡顶密集触发。**【推断】** 必须先扣掉坡度分量（在"可行驶面"上做判据，而不是在全点上做） |
| **输出** | **【推断】建议 P0 只输出"独立的 no-go 掩膜 + 可视化"，不要直接写 lethal**：<br>（a）**保守**：误判成 lethal 的后果是"路被凭空切断"（§1.3 ①）；<br>（b）漏判的后果在这个场地上**很小**：0.2/0.3 m 台阶已经被 2D 静态层的**占用**格挡住了（`docs/worlds.md` §6：台阶高 0.20/0.30 m ⇒ 物理上上不去 ⇒ 2D 图把它们画成障碍）← **这层是"补第二道保险"，不是唯一防线**；<br>（c）真要用致命度：用 `nav2_costmap_2d::ObstacleLayer` 的 `marking`/`clearing` 语义最省事（沿用现有机制），**不要**新造一个"lethal"概念 |
| **怎么进 nav2** | 同 §3.1：先离线成层 ⇒ `static_layer` / `grid_map_costmap_2d`；后写 `Layer` 插件（`rm_costmap_layers::NegativeObstacleLayer`），挂进同一张 `enabled` 真值表 |

---

### 3.3 净空高度层（clearance）

| 项 | 设计 |
|---|---|
| **数据源** | 同一张高程图，但**必须保留"每格的上表面与下表面"两个高度**（或至少"头顶最近表面的高度"）。**【推断】** 这是 2.5D 表示**唯一必须打补丁**的地方：标准 elevation map 一个 (x,y) 只有一个 `elevation` ⇒ 要加一个 `ceiling_height` / `free_height` 层。这正是"2.5D 表达不了多层结构"那句限制的具体形态 |
| **本车真实尺寸（**【事实】** 从 URDF/xacro 读出来的，不是估的）** | `src/rm_nav_bringup/urdf/sentry_robot_sim.xacro`：<br>· `base_link` 碰撞盒 `box size="0.2 .3 .1"`，`origin xyz="0 0 0.05"` ⇒ z ∈ **[0, 0.10]**（相对 base_link）<br>· 4 个轮子 `cylinder length=0.05 radius=0.06`（`:5-6` 的 `xacro:property`），轮心在 z=0 ⇒ **轮半径 0.06 m**（= base_link 离地高度，`docs/worlds.md` §2.4 实测 `/odom_ground_truth` z = 0.0600）<br>· `imu_link` 碰撞盒 `box size="0.05 0.05 0.05"`，`origin (0.12, 0, 0.125)` ⇒ 顶到 **z = 0.15**<br>· Livox 装在 `base_link + (0.12, 0, 0.175)`（`xacro:arg xyz` 默认 = 实测值）；雷达本体 `box 0.1×0.06×0.06`（`src/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro`）⇒ 顶部到 **z ≈ 0.175 + 0.049 = 0.224 m**<br>⇒ **碰撞包络高度 ≈ 0.224 m（四舍五入取 0.23 m 做安全余量）**；横向 **0.2 × 0.3 m**（nav2 用的 `robot_radius: 0.22` 取的是"≈真实车体半径 0.2"的圆，见 `nav2_params_sim_base.yaml` 注释）<br>⚠ **真车不一样**：`sentry_robot_real.xacro` 里 Livox 在 `base_link + (0.0, 0.045, 0.49)` ⇒ **实车雷达面高 0.49 m、整车更高**。["哪个高度该进判据" 见 §8 第 3 条] |
| **判据** | 每格：`free_height = ceiling_z − ground_z`；`可过 ⟺ free_height > robot_height + margin`。<br>**【推断】** 初始建议 `robot_height = 0.23 m`（仿真包络四舍五入）、`margin = 0.10 m` ⇒ **阈值 0.33 m**；**但这个值必须在实车/新场地重新定**（§8 第 3 条） |
| **为什么 nav2 的 2D 层没有这个概念** | nav2 costmap 是纯 2D 栅格，**z 被压掉了**。现有参数里的 `max_obstacle_height: 2.0` **不是**"限高"：它的语义是"把 2 m 以上的点当噪声丢掉"（`nav2_params_sim_base.yaml` 里 `obstacle_layer` 与 `stvl_layer` 都是 2.0），**不控制机器人能不能从下面钻过去**。<br>**【事实】** `min_obstacle_height: 0.0` 是 2026-09-24 为"保住 0.4 m 矮墙"专门从 0.2 调低的（同文件注释），说明这些高度带是在"**把什么当地面上的障碍**"这一维上做取舍，**与"头顶"无关** |
| **输出** | `free_height` 层（float）+ 三档：`可过` / `高风险（触发减速或行为）` / `不可过（致命）` |
| **怎么进 nav2** | 同 §3.1/§3.2；【推断】这一层最值得走"**触发**"而不是"写 cost"（见 §4）：因为限高的失败模式是"车已经进去了才发现"，惩罚应当发生在**进入之前** |

---

## 4. 动作怎么接：**"写 cost" 与 "触发行为" 的边界**

### 4.1 两档的判据（**【推断】，但理由是可核对的**）

| 档 | 什么时候用 | 机制 | 为什么不能用另一档 |
|---|---|---|---|
| **写 cost（连续/致命代价）** | 决策是"**这一格值多少**"，而且**不需要改变导航流程**：坡道限速、落空禁行、净空不可过 | nav2 costmap 图层（`nav2_costmap_2d::Layer`）：`updateCosts()` 往 master grid 写 0~252 或 `LETHAL_OBSTACLE`/`NO_INFORMATION` | cost 只能表达"贵/不可走"；它**不能表达"停下来问一句"**，也不能改变行为树的分支 |
| **触发行为（BT 条件/动作）** | 决策是"**流程要变**"：进坡前减速试探、净空不足就换目标、落空边沿前先原地确认 | `bt_navigator` + BT XML（`default_bt_xml_filename`）+ 自定义 BT 节点（`plugin_lib_names`）；`behavior_server` 的插件（`behavior_plugins`） | 这些是**流程/服务层**，写进 cost 里就丢失了"原因"，也没法恢复 |

**一句话**：**cost 决定"走不走、走多贵"，BT 决定"走的姿势和走不成之后干什么"。**

### 4.2 现成的钩子（**【事实】**，都是本机已经在跑的机制）

| 钩子 | 位置（本仓证据） | 拓展方式（**设计级，不写代码**） |
|---|---|---|
| `bt_navigator.plugin_lib_names` | `src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml:90-121`（31 项，全是 `nav2_*_bt_node`） | **加一行**自己的 `librm_bt_nodes.so`（如 `rm_bt_is_ramp_ahead_condition_bt_node`）；节点用 `BT::ConditionNode` 派生 + `BT_REGISTER_NODES` 注册，库由 `nav2_behavior_tree` 的 `BT::SharedLibrary` 在运行期加载 |
| BT XML | **本仓现在没有**：两个 params 都没有 `default_bt_xml_filename` ⇒ 用的是**包内默认**（Humble 的 `navigate_to_pose_w_replanning_and_recovery.xml`） | 新增 `src/rm_navigation/rm_navigation/behavior_trees/*.xml`（把默认树复制过来，在 `RecoveryNode`/`Fallback` 里插入自定义条件），参数里给 `default_bt_xml_filename`。**【推断】这一步必须与"新增节点"一起做**——只加节点不改树，节点永不被调用，是典型的**静默失效** |
| `behavior_server` | 同文件 `:404-412`：`behavior_plugins: ["spin","backup","drive_on_heading","wait"]`；`costmap_topic` / `footprint_topic` / `max_rotational_vel 3.0` 等 | 加自定义插件（如"低速试探一段" `rm_behaviors/CreepForward`）⇒ 在 BT XML 里作为 recovery/主动动作调用。**【推断】** 这一档**现在不做**（见 §7.3），因为"怎么保证触发是安全的"还没有答案（§8 第 5 条） |
| 探测器 → BT 的**载体** | **没有**（仓库里没有任何自定义 BT 节点：`grep -rl "BT_REGISTER_NODES" src/` 为空 —— **【推断】** 这是从"params 里 plugin 列表全是 nav2 官方"反推的，写代码前应跑一遍 `grep -rn "BT_REGISTER_NODES\|bt_nodes" src/` 确认） | **【推断】推荐形态**：探测器发一个**状态话题**（如 `/traversability/status`，`std_msgs/String` 或自定义 msg：`{坡度档, 净空档, 落空档, 有效时间}`），BT 条件节点只做"读 latched 状态 + 与当前位姿/路径点比对"。**好处**：探测器与 BT 解耦，BT 里不跑重计算 |

### 4.3 **【推断】一个具体的、最小的触发闭环（设计，不含代码）**

```
rm_elevation_map（新，感知域）
    ├─ 发 /traversability/slope_grid      （2.5D 层，供 costmap 图层消费 = "写 cost" 那档）
    ├─ 发 /traversability/clearance_grid  （同上）
    └─ 发 /traversability/status          （latched，低维：{最近坡道方位+坡度, 本格净空, 落空标志}）
                 │
                 ├─→ rm_costmap_layers::SlopeLayer / ClearanceLayer   （"写 cost"）
                 └─→ rm_bt_nodes::IsRampAhead / IsClearanceOk          （"触发行为"，读 status）
                          └─ 在自定义 BT XML 里替换/包住 FollowPath
```

**【推断】必须守住的一条边界**：**BT 条件节点不许把"判据"重新实现一遍**。
判据只在 `rm_elevation_map` 里有一份（"同一件事只在一处维护"，`docs/README.md` 维护约定），
BT 只读结论。否则调一次阈值要改两处，且两处会漂。

---

## 5. 怎么验证（**我们的独特优势：STL 是地面真值**）

### 5.1 为什么这是"别人没有"的东西

**【事实】** `src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl`
就是 Gazebo **实际加载的那份几何**（`GAZEBO_MODEL_PATH` 指向 `<share>/hzmi_rm_simulation/meshes`，
见 `docs/worlds.md` §2.3），sha256 `cc723f3156999598127dfd34d1467767773708ecd46e78f6ad4c40de597d0846`。
**【事实】** 它的换算口径已经完全定死：**毫米单位**、`<scale>0.001`、`<visual>/<collision>` pose
`z = +1.6413436` ⇒ **地板顶面落 world z = 0**（mesh 局部 `z = −1.64134362793`，STL 顶点里有
**4412 个点**精确落在这个平面上，见 `docs/worlds.md` §2.1）。
⇒ **我们可以离线算出"坡度/净空/落差的真值图"，然后与传感器派生层逐格比对。**
**【推断】** 这在实车上**不可能**：真实场地没有 CAD 真值，也没有"同一次实验里既有真值又有传感器"的条件。
所以 **"真值 vs 派生"的逐格比对是我们台架最有价值、也最难被复制的产出**。

### 5.2 真值图怎么算（离线，纯几何，可复跑）

| 步骤 | 做法 | 口径/命令 |
|---|---|---|
| ① 读 STL（不经 Gazebo） | 二进制 STL = `80 B 头 + uint32 面数 + 每面 50 B`；每面 = `3 float32 法向 + 9 float32 顶点 + uint16 属性` | **【本次核对】** `45900` 面、文件 `2295084 B`（`84 + 45900×50 = 2295084` ✓） |
| ② 单位与抬升 | 顶点 `× 0.001`、`+ 1.64134362793` ⇒ **world 系，且 world == mesh 米制**（`model pose = 0 0 0 0 0 0`） | `docs/worlds.md` §2.1 |
| ③ **真值高程图** | 每 (x,y) 格取**最高**面高度（保守，保住薄墙 —— 与 `docs/worlds.md` §3 第 1 步同口径） | **【本次核对】** 0.02 m 栅格：`803 × 1458`，有限格 `1 166 859` |
| ④ **真值坡度图** | **面级**法向与竖直夹角（`atan2(‖n_xy‖, ‖n_z‖)`），按面积加权统计/落格 —— **不要**用高度栅格差分（0.02 m 量化会把 12.8° 抬成 45°） | **【本次核对】** 见 §5.3 表 |
| ⑤ **真值净空图** | 每 (x,y) **从下往上**求该列穿过的所有面高度，排序 ⇒ **最高面 = 地表**，**其上第一个面 = 头顶表面** ⇒ `clearance = z_ceiling − z_ground` | **【本次核对】** 用 §5.3 的脚本（**不用 ray 库**：本机 `rtree/embreex/numba` 都没有，`trimesh.ray` 会直接 `ModuleNotFoundError`；用逐三角形 XY bbox 粗筛 + 重心坐标求交，`46 400` 列 `7.9 s`） |
| ⑥ 配准 | 真值图在 **world** 系；传感器在 **odom/map** 系。平移量 = 出生点 `(10.925, 2.525)`（`docs/worlds.md` §3），**且 model pose 是 `0 0 0 0 0 0` ⇒ 没有旋转** | **【事实】** 交叉验证：合成图 `map/RMUC2026.yaml` 的 `origin = [-25.925, -9.425]` **正好等于**"场地左下角 `(-15.0, -6.9)` − 出生点 `(10.925, 2.525)`" ⇒ "生成了 map 系图"这条链**已经自洽** |
| ⑦ 指标 | **逐格混淆矩阵**：真值可行驶 vs 派生 free ⇒ `TP / FP / FN / TN` + 准确率/召回/精确率；**FP 与 FN 必须分开报**（这两种错误后果不对称：FP = 删掉一条通路，FN = 把车送进不可走的地方）；再报**连通性**（真值里可走的区域在派生层里是否仍连通、绕行长度比） | 复用 `tools/scripts/mapping/compare_2d_maps.py` 的"区域计数"思路（它已支持 `--box/--png/--json`）+ 新增逐格矩阵 |

### 5.3 **【本次核对】** 真值几何（写本文时从 STL 现算，口径如上）

**面积分布（0.02 m 顶视栅格，`lift = z + 1.64134362793`）**

| 层 | 面积 | 备注 |
|---|---|---|
| `lift = 0.00 m`（可行驶地面） | **216.0 m²** | 与 `docs/worlds.md` §6 的"场地可行驶地板 214.6 m²"同量级（栅格口径差） |
| `lift = +0.20 m`（0.2 m 台面） | **106.2 m²** | `docs/ground_segmentation_slots.md` §5.2 记 106.2 m² ✓ |
| `lift = +0.30 m`（0.3 m 台面） | **69.2 m²** | 同处记 70.7 m²（栅格口径差） |
| `lift = +0.35 / +0.40 / +0.90 / +0.96 / +1.53 m` | 3.8 / 8.2 / 9.7 / 3.3 / — m² | 塔、柱、护墙 |

**坡度分布（面级法向、面积加权）**

| 倾角区间 | 面积 | 与已有记录对照 |
|---|---|---|
| 0–5° | **1486.5 m²** | `docs/ground_segmentation_slots.md` §5.2 "85.5% within ±5 deg" ✓ |
| 5–10° | 6.43 m² | — |
| **10–15°** | **11.23 m²** | 同处记 **11.2 m²** ✓ |
| **15–22°** | **16.69 m²** | 同处记 **16.7 m²** ✓ |
| 22–35° | 7.52 m² | 同处记 **7.5 m²** ✓ |
| 35–60° | 7.36 m² | 同处记 **7.4 m²** ✓ |
| 60–90° | 203.28 m² | 同处记 22.1 m²（**口径不同**：它按"倾斜面"统计，这里把竖直面全算进来） |

**最大的一块斜面（= 我们一直在说的"23° 坡"）** —— **【本次核对】面级精确值**：

| 面 | 倾角 | 面积 | 顶点（world，`lift`） | 几何含义 |
|---|---|---|---|---|
| A | **23.0°** | 1.16 m² | `(-3.643, 8.002, 0.000)` → `(-4.218, 8.002, +0.200)` → `(-6.240, 4.293, 0.000)` | 边沿坡，**水平投影宽度 0.45 m**、升 0.20 m |
| A′ | **23.0°** | 1.01 m² | `(-4.218, 8.002, +0.200)` → `(-6.485, 4.764, +0.200)` → `(-6.240, 4.293, 0.000)` | 同一坡的另一半 |
| **镜像 A** | **23.0°** | 1.16 + 1.01 m² | `(3.643, -4.713, 0)` / `(4.218, -4.713, +0.200)` / `(6.240, -1.004, 0)` / `(6.485, -1.476, +0.200)` | **+x 半场（出生点那半场）的同一块坡** |
| 12.8° 倒角 | **12.8°** | 1.27 + 1.27 m² | `(-1.429, 7.054, +0.200)` → `(4.188, 7.054, +0.200)` → `(3.880, 6.614, +0.300)` | 连通 h=0.20 与 h=0.30 两级台面；水平投影 0.44 m |

⇒ **【本次核对】的结论**：
* **23.0° 是面级精确值**（`docs/mapping_2d_from_cloud.md` §4.1 记 "23–24°"，**一致**）；
* **平面投影宽度 0.45–0.47 m**（同上记 "≈0.45 m"，**一致**）；该坡的**升 = 0.20 m**，
  而同处记 "升 ≈0.18 m" —— **【推断】** 0.18 是**车实际走过的坡段口径**（车没走满整块面，或走了
  `map` 系里那 0.45 m 投影里的一段），与面级 0.20 m 不矛盾；
* **12.8°** 面级精确值 = `atan(0.10 / 0.44) = 12.8°`，同处记 "12.8°"，**一致**；
* **镜像坡**：map 系 `x[-7.29,-4.44] y[-7.23,-3.52]`（= world `(3.64..6.49, -4.71..-1.00)` 平移出生点）
  —— 车爬的就是它；文档里按 world 负半场叙述的那一块（map `x[-17.41,-14.57]`，**在 map 窗口之外**）
  是它的镜像。

**⚠ 两处必须写下来的"达不到"（否则 §7 的验收会设成做不到的事）**

1. **【本次核对】RMUC2026 的 STL 里没有"暴露出来的 −0.2 m 坑"。**
   mesh 里确实有一层 `lift = −0.20 m` 的水平面，面积 **466.25 m²**（`docs/ground_segmentation_slots.md`
   §5.2 记 "坑 z = −0.200 m（466 m²）"，数字一致）；但它的 XY 范围
   `x[-14.505, 14.545] y[-6.401, 9.649]` **几乎等于整个场地**，即它是**底板（0.20 m 厚的底座）的底面**。
   逐列检查：**每一列的"最上面那个面"都是 `lift ≥ 0`**，那一层永远在它下面。
   ⇒ 从任何传感器（也从任何俯视真值图）看，这块场地是**平的 0 m 地面**，**没有落差可测**。
2. **【本次核对】RMUC2026 的 STL 里没有任何"头顶表面"：46 400 列（0.10 m 栅格）里，
   `clearance < ∞` 的列 = 0。**
   即：**每一列的竖直方向只穿过一个面**（"地表"），其上什么都没有。
   与 `.tmp_cache/stl_view/README.md` 的 ray-parity 结论一致：">=0.25 m 净空的 subsurface 空洞只有
   **3.7 m²**、全是 ≤0.15 m 宽的缝"。`docs/worlds.md` §6 也写着"隧道 / 设计坡道：**没有**"。
   ⇒ **净空层在这份场地里没有可验证的真值**；`docs/mapping_2d_from_cloud.md` §4.1 说的"走廊**无顶棚**"
   也在本次核对里被证实（走廊剖面 `y ∈ [-5.90, -4.73]` 的每一列 `clearance = ∞`）。

**⇒ 【推断】由此得到两条对计划有直接影响的结论：**
* **落空层与净空层在这份场地上"造不出用例"，必须自建代理**：落空用 §5.2 的方式在几何上加一个
  −0.2 m 的坑（或复用 `docs/ground_segmentation_slots.md` §5.2 的 **7 个 STL 代理场景**：
  "0.2/0.3 m 平台 + 10/15/22° 倒角 + −0.2 m 坑"）；
  净空需要在 Gazebo 里**加一根横梁**（或用 `ground_seg_ab` 那种解析生成的合成点云）。
* **但"派生 vs 真值"的基础设施仍然现在就值得做**：坡度层可以**完全**用现有场地验证
  （两个 23° 坡 + 三个 12.8/17/10° 倒角 + 0.2/0.3 m 台阶），而这正是我们**唯一量过的真特征**。

### 5.4 三个已量特征**应当**被怎么分类（验收判据的来源）

| 特征（map 系） | 真值层应给 | 派生层应给 | 判据（**验收用**） |
|---|---|---|---|
| **23° 坡**<br>map `x[-7.29,-4.44] y[-7.23,-3.52]`<br>（world `(3.64..6.49, -4.71..-1.00)`） | 坡度 **23.0°**；可行驶面（面法向在限内） | 坡度层报 **23±2°**；**不得**被判成"不可行驶"（> 阈值档）；`pcd_to_nav2_map` 口径下 **FREE** | ① 真值坡度与派生坡度的**中位差 ≤ 3°**；② 派生层在该区域的 **free 占比 ≥ 95%**（对照：已验收的 `RMUC2026_cloud` 在坡面区域 **98.8% free**，见 `docs/mapping_2d_from_cloud.md` §6.1）；③ **规划器对坡顶/坡中目标的成功率 ≥ 4/5**（对照：旧 `/scan` 图 2/5、点云图 5/5，同处 §6.4）<br>⚠ **前置决策**：若最终阈值取 25°，23° 只有 2° 余量 ⇒ 这条例必须**先有 §3.1 的三段式阈值**才谈得上"通过" |
| **1.05 m 走廊（无顶棚）**<br>map `y[-8.43,-7.26] x[-8.93,-3.63]`<br>（world `y[-5.90,-4.73] x[2.0,7.3]`） | 面高 **+0.20 m**、面本身 **≈0°**；**净空 = ∞**（无顶棚）；**最窄净宽 1.00 ~ 1.05 m** | 坡度层报 **≈0°**；净空层**必须不触发**（因为它真没有顶）；落空层**不得**把两侧边沿误判成"掉下去" | ① 真值净宽量出来（**【本次核对】0.02 m 栅格：1.00 m**；`docs/mapping_2d_from_cloud.md` §4.1 **1.05 m**）—— **【待测】把这两个口径的差说清楚**（见 §8 第 8 条）；② **与本车宽度的对照要报出来**：车体 **0.2 × 0.3 m**（碰撞盒）／nav2 圆 **`robot_radius 0.22 m`** ⇒ 1.00 m 净宽**余量 0.28~0.30 m/侧**（按半径口径）⇒ 静态上**过得去**；③ **落空层在该走廊 8 邻域上的 FP 必须为 0**（这是"落空误报会切断通路"最容易发生的地方） |
| **0.2 / 0.3 m 台面** | 高差 **+0.20 / +0.30 m**，立面 **60–90°**，连接处是 **10/12.8/17/22° 倒角** | `pcd_to_nav2_map` 口径下**台面顶面必须占用**（高度闸 0.15 m）；倒角本身**可行驶** | ① 真值高差与派生高度差的**中位差 ≤ 0.05 m**；② 台面顶面的**占用召回 ≥ 95%**（对照：合成代理里 0.2/0.3 m 平台顶面 `linefit 99.53%` / `patchwork 100%`，`docs/ground_segmentation_slots.md` §5.2）；③ **12.8° 倒角不得被判成"不可行驶"**（对照：`linefit` 在 10~22° 倒角上有 **71.8~73.5% 判成障碍**，这是**已知缺陷**，patchwork 降到 **30.9~31.3%**，同处 §5.2 —— 所以这条例要注明"用哪个分割器"） |

**【推断】还有一条只有真值能给的判据**：**连通性**。
"真值里可行驶的 216.0 m²，在派生层里还是不是（近似）连通的？"——
`docs/worlds.md` §6 已经量过：`clearance ≥ 0.22 m` 的集合 178.6 m²、同一连通域 176.3 m²、
`clearance ≥ 0.30 m` 时**裂成两块 80.6 / 80.5 m²**（两个半场不再连通）。
⇒ **派生层不该比真值"更碎"**：如果派生层把 176 m² 的可走区切碎成几十块，那不管逐格指标多好看，都是 fail。

---

## 6. 怎么看（可视化）

### 6.1 RViz 配方（按"要不要装包"分档）

| 想看什么 | 怎么显示 | 需要额外装包？ |
|---|---|---|
| **原始/派生点云的形状**（最省事，**零依赖**） | `PointCloud2` 显示 + **Color Transformer = `AxisColor`（选 Z 轴）** 或 `Intensity`；固定视角 `TopDownOrtho` 看俯视、`Orbit` 看立体 | **不需要**（rviz2 自带） |
| **把派生层当"假彩色点云"看**（坡度/净空/落差） | 把每个 (x,y) 格变成一个点，`z` 放真实高程、把**层值塞进 `intensity` 或 `rgb`** ⇒ 一个普通 `PointCloud2` 就能看三层 | **不需要**（我们自己出点云即可） |
| **GridMap（2.5D 层的原生显示）** | `rviz2` 的 **`grid_map_rviz_plugin`** 提供 `GridMap` display：每个 layer 一个可视化开关、可调色标 | **要装** `ros-humble-grid-map-rviz-plugin`（+ 产 GridMap 话题要 `ros-humble-grid-map-msgs`）；本机**未装**，apt 候选 **2.0.1-1jammy** |
| **把层当 OccupancyGrid 看**（插件缺失时的替代） | GridMap → OccupancyGrid 转换后当普通 `Map` 显示；或用 **`grid_map_costmap_2d`**（apt，**现成**的"GridMap ↔ costmap_2d"接口） | 转换库要装 `ros-humble-grid-map-costmap-2d`（本机未装，候选 2.0.1） |
| **OctoMap 的八叉树** | `octomap_rviz_plugins` 的 `OccupancyGrid`/`OccupancyMap` display | **要装** `ros-humble-octomap-rviz-plugins`（本机未装，候选 2.1.1）+ `octomap_server`（候选 2.3.1） |
| **现有代价地图/图层本身** | `nav2.rviz`（`src/rm_navigation/rm_navigation/rviz/nav2.rviz`）里已有 costmap 显示；`Map` 显示看 `map` | 不需要 |

### 6.2 离线 PNG 路线（**现在就能用，不需要装任何东西**）

| 工具 | 产出 | 出处 |
|---|---|---|
| `tools/scripts/mapping/pcd_to_nav2_map.py --png <out.png>` | 俯视 PNG（带 `--report-box` 的统计框）；`--json` 同时落一份带全部面积统计与分区域分类的 manifest | `--help`；`docs/mapping_2d_from_cloud.md` §5.3 |
| `tools/scripts/mapping/compare_2d_maps.py --png` | 两张 2D 图的 A/B 对比图（灰/黑/品红/橙/红/绿六色差异图） | 同 §6.2（图在 `docs/img/ab_2d_compare.png`） |
| `.tmp_cache/stl_view/` 的渲染器 | **STL 的多视角 PNG**：`view_e_top_heightcolor.png`（按高度着色）、`view_f_levels_and_gaps.png`（离散层 + 每列竖直自由间隙）、`view_g_slices_solid_air.png`（8 个水平切片的 solid/air/roofed）、`view_i_measurements.png`（Z 直方图 + 坡度直方图 + 每层足迹） | `.tmp_cache/stl_view/README.md`；脚本 `render.py`（**纯 numpy z-buffer 软件渲染，不需要 OpenGL**）、`stl_lib.py`、`vox.py`、`parity.py`、`make_views.py` |
| **真值图渲染（本文建议新增，尚未有）** | 真值坡度/净空/落差图直接出 PNG（与派生层同色标、并排），做进 §5.2 的对比工具 | **【推断】这是第一步的交付之一** |

> ⚠ **`.tmp_cache/` 是未版本化的临时目录**（`git status` 里是 `??`）。要把 STL 渲染复用到
> "可复跑的工具"这一级，需要把用到的脚本**收敛成一个 `tools/scripts/world/` 下的工具**（别依赖 `.tmp_cache`）。

### 6.3 【事实】本机可视化能力核对（结论）

* **不需要装任何东西**：`PointCloud2` + Z 着色、`nav2.rviz` 的 costmap、`pcd_to_nav2_map.py --png`、
  `compare_2d_maps.py --png`、`.tmp_cache/stl_view` 的 STL 渲染。
* **要装才有的**：`GridMap` display（→ `ros-humble-grid-map-rviz-plugin`）、
  OctoMap 显示（→ `ros-humble-octomap-rviz-plugins`）、
  GridMap→costmap 转换（→ `ros-humble-grid-map-costmap-2d`）。
* **【推断】建议**：**第一步完全不装包**（用假彩色 `PointCloud2` + PNG）；等到"层要长期挂着看"时
  再把 `grid_map` 那三个包一次装齐（都是 apt、版本 2.0.1）。

---

## 7. 分阶段计划

### 7.1 排序原则

**【推断】** 按"价值 ÷ 成本"，并且**优先做能解锁后面所有工作的那一步**：
① 判据基础（真值 + 派生 + 对比）→ ② 坡度层进 nav2 → ③ 净空/落空。
理由：**没有第 1 步，"阈值定多少"永远只能拍脑袋**；而第 1 步**零风险**（全离线、不改栈、不装包）。

---

### 第一步：**离线判据基础**（真值图 + 派生层 + 对比工具）

| 项 | 内容 |
|---|---|
| **交付物** | ① `tools/scripts/world/` 下一个"由 STL 出真值图"的工具（高程/坡度/净空三张 + JSON manifest，含 sha256 与全部参数）；<br>② `tools/scripts/mapping/` 下一个"由 PCD 出 2.5D 层"的工具（**输出 `elevation / slope / height_diff` 三层**，而不是现在这一张 pgm；口径**默认值与 `pcd_to_nav2_map.py` 完全一致**）；<br>③ 一个"逐格对比"工具（TP/FP/FN + 连通性 + 并排 PNG）；<br>④ 一份结果记录（写进本文或新建 `docs/traversability_report.md`） |
| **不碰什么** | 不改任何 launch/param/节点；不装任何 apt 包；不改 `pcd_to_nav2_map.py` 的默认值（新工具独立） |
| **工作量【推断】** | 2~3 人日（真值工具 1 d，派生工具 0.5 d，对比 0.5 d，跑+写 1 d）。**【推断】** 真值工具的主要风险是"**本机没有 ray 库**"（`rtree`/`embreex`/`numba` 全无，`trimesh.ray` 直接报 `ModuleNotFoundError`）⇒ 用 §5.2 ⑤ 的"XY bbox 粗筛 + 重心坐标"纯 numpy 做法（**本次已经验证：46 400 列 7.9 s**） |
| **验收判据** | ① 真值工具自检：把 STL 的"地板顶面"算成 **`z = −1.64134362793 m`（mesh 系）**、面积与 `docs/worlds.md` §2.1 的"4412 个顶点精确落在该平面"口径对得上；<br>② 真值层能复现 §5.3 的四行坡度面积（1486.5 / 11.23 / 16.69 / 7.52 m²，容差 ±10%）；<br>③ **派生层能复现已验收的结果**：用 `PCD/RMUC2026_spl.pcd`（或 `_raw`）跑出的 2D 图与既有 `map/RMUC2026_cloud.pgm` 区域计数同量级（坡面 free ≥ 95%、走廊 free ≥ 95%）；<br>④ 对比工具能报出 §5.4 三条特征的分类结论，且 **FP/FN 分开列**；<br>⑤ **明确列出"这份场地验不了什么"**（落空无真值、净空无顶棚 —— §5.3 的两条） |

### 第二步：**坡度层进 nav2（写 cost）**

| 项 | 内容 |
|---|---|
| **前置** | 第一步通过；§3.1 的**三段式阈值**已定（**先用 15/25 两档，并显式标注"25° 来自 `pcd_to_nav2_map.py` 的旧默认值"**） |
| **交付物** | ① 离线烘出的 `<world>_slope.pgm`（三档）+ 复用既有 `static_layer` 机制挂上（**不改 launch**：`static_layer` 已在 params 里，加一个实例或加一张图）；<br>② 若"一张 pgm 不够"（要连续代价），再写 `src/rm_navigation/rm_costmap_layers/` 的 `SlopeLayer`（`nav2_costmap_2d::Layer`），`enabled` 挂进 `navigation_launch.py` 的真值表；<br>③ A/B + 回归 |
| **工作量【推断】** | 静态图路线 0.5~1 人日；写插件 + 挂槽位 + A/B **3~5 人日**（含 `local_obstacle_truth_table.py` 那类静态校验脚本的更新） |
| **验收判据** | ① **23° 坡在真值层与派生层都是"可行驶"**（§5.4 第 1 行三条）；<br>② `nav_smoke_regression.py` 的 **P0 回归 PASS**，且 `recoveries` / 用时与基线同级（对照：换图那次 `SUCCEEDED 54.2 s vs 57.9 s`、`recoveries` 都是 4，`docs/mapping_2d_from_cloud.md` §6.4）；<br>③ **规划器对 5 个坡道/走廊目标的成功率 ≥ 4/5**（基线 2/5 → 点云图 5/5）；<br>④ **不许把可走区切碎**：派生层里"从出生点可达的面积"不得比真值口径小 10% 以上（对照真值 176 m²，`docs/worlds.md` §6）；<br>⑤ 槽位互斥/真值表校验脚本仍然 PASS |

### 第三步：**净空层 + 落空层（触发与保险）**

| 项 | 内容 |
|---|---|
| **前置** | ① 需要一个**带特征的可验证场景**（§5.3 已证：RMUC2026 里**没有**顶棚、**没有**暴露的坑）⇒ 在 Gazebo 里加一根横梁 / 一个 −0.2 m 坑（或复用 `ground_seg_ab --synthetic` 的 7 个 STL 代理 + 3 个薄墙场景）；<br>② §8 第 5 条（触发安全性）有答案 |
| **交付物** | 净空层（判据 + 层 + 三档输出）；落空层（判据 + **独立 no-go 掩膜**，先不写 lethal）；`/traversability/status` 话题；**BT 条件节点 + 自带 BT XML**（§4.3）；A/B |
| **工作量【推断】** | 场景搭建 1~2 人日；净空判据+层 2~3 人日；落空判据 3~5 人日（**误报调参是最贵的一段**）；BT 节点+XML 2~3 人日；A/B 与验收 2 人日 ⇒ **合计 10~15 人日** |
| **验收判据** | ① **限高**：梁下 0.30 m（< 车高 0.224 + 余量）判"不可过"、梁下 0.60 m 判"可过"（对照 `robot_height ≈ 0.224 m`，§3.3）；<br>② **落空**：−0.2 m 坑被标 lethal（对照 `ground_seg_ab --synthetic` 的 "−0.2 m 坑" 代理场景）；<br>③ **误报**：在 1.05 m 走廊与 23° 坡上，落空层 FP = 0（§5.4 第 2 行第 ③ 条）；<br>④ **触发**：BT 在限高不足时**不进入**，日志里有可归因的状态（不是"莫名其妙停了"）；<br>⑤ 落空/净空层**默认关闭**（`enabled: false`），A/B 才开 —— 保证"不动已验证行为" |

### 7.2 **明确不做（现在不做，以及为什么）**

| 不做 | 为什么（**理由要可核对**） |
|---|---|
| **OctoMap** | ① 我们缺的是"坡度/净空/落差"这三个**标量场**，不是"体素占据概率"；<br>② **【事实】** 本场 STL 逐列检查：**没有任何头顶表面**（46 400 列中 `clearance < ∞` 的列 = 0）⇒ 上 OctoMap 在**这份场地**上换不来任何新信息；<br>③ 它是新的调参面（hit/miss、分辨率、ray casting 开销）而**当前栈的瓶颈不是感知**（`gzserver` 占 0.52 核，是分割器的 6~9 倍，`docs/ground_segmentation_slots.md` §6.1）；<br>④ 触发条件写死：**"出现一个场景，2.5D 层无论怎么调都把可走的路判成不可走（多层/悬挑/隧道）"** 才装 |
| **ESDF 体素 / MPC（voxblox / nvblox / FIESTA）** | ① 它们的消费者是 **3D 规划器/MPC**，而我们的决策层明确"继续 2D、不上 3D 规划器"（`docs/algorithm_matrix.md` §8.2）；<br>② 距离场不直接回答"这个坡度能不能上"；<br>③ nvblox 要 GPU；<br>④ **【推断】** 麦克纳姆底盘在 23° 坡上的运动学与平地差别不是"轨迹形状"而是"打滑/姿态"，那是**控制器限速问题**，不是 ESDF 能解决的 |
| **3D 规划器 / 3D 代价地图** | 同 ①：nav2 的插件契约（`nav_msgs/Path` + 2D costmap）就是 2D；哨兵是平地麦轮车，"必须做 3D 决策"的场景在这份场地上不存在 |
| **把 `ground` 默认从 `linefit` 换成 `patchwork`** | **【事实】** `docs/ground_segmentation_slots.md` §7.1 已经拍板"**不换**"，并列了 5 个触发条件（`mode:=nav` 整栈 A/B、P0 回归、第二个场地、两个新毛病确认、实车 `sensor_height` 重标定）。**【推断】** 本计划**不碰这个决策**；但第三步的落空层如果做，`patchwork` 在"−0.2 m 坑的下坡倒角"上**比 linefit 更差**（52.5% vs 8.1% 障碍率，同处 §5.2）这条必须重新看 |
| **专门做"1.05 m 走廊的通过优化"** | **【事实】** 车体 0.44 m 宽 / nav2 半径 0.22 m ⇒ 静态余量够（§5.4 第 2 行）；**【事实】** 实测卡死发生在**更窄的口**（`docs/worlds.md` §6：两个半场之间最窄 **0.45 m**，"余量 5 mm/边 ⇒ 实际过不去"），且 `map≈(0.5, 3~6)` 的北侧通道有**三次卡死**（`docs/mapping_small_point_lio.md` §9 第 1 条，**根因未定位**）。⇒ 走廊不是问题，"0.45 m 口"才是，而那是**几何上过不去**，不是感知问题 |
| **为"落空"新造 lethal 语义** | 用 nav2 现成的 `LETHAL_OBSTACLE` + 内切/膨胀即可；新造语义会与 `robot_radius 0.22` / `inflation_radius 0.5|0.55` 的既有语义打架 |
| **改 `pcd_to_nav2_map.py` 的默认阈值** | 它已经被验收（`docs/mapping_2d_from_cloud.md` §6），**改默认值会让已验收的结论失效**。新层用**新工具**，旧工具逐字节不动 |

### 7.3 **【推断】如果只能做一件事**

**做第一步。** 因为它同时产出三样别人拿不走的东西：
① 一份**真值图**（离线、可复跑、有 sha256）；② 一条**判据基线**（坡度/净空/落差的阈值从"拍"变成"量"）；
③ 一个**可复用的对比工具**（后面每加一层都靠它验收）。
而第二步、第三步都是"在判据之上加东西"——没有第一步，它们只能靠"看着 RViz 感觉还行"。

---

## 8. 开放问题 / 未验证

> 每条格式：**问题** → 我们知道什么 → **怎么确认（命令/做法）** → 影响哪一步。

1. **【待测】本车（仿真）实际能上多少度的坡？**
   * 已知：**坡度幅值**有实测（23°、12.8° 等，§5.3），**阈值**只有 `pcd_to_nav2_map.py` 的 `--slope-limit 25°`（一个判据参数，不是本车能力）。
   * 怎么量：Gazebo 里做**坡度阶跃**测试（用 `tools/scripts/world/stl_to_world.py` 搭带 5/10/15/20/25/30° 坡的世界；
     或直接用 `ground_seg_ab --synthetic --synthetic-long-ramp` 那类解析坡度场景的**物理版**），
     发固定 `linear.x` 看 `/odom_ground_truth` 的位移/打滑与 roll/pitch；
     同时用 `tools/scripts/mapping/ramp_section_probe.py` 的口径记录"发着速度但不动"的窗口
     （它对 20 航点路线实测过 `motion_checks_failed` 与 LIO `roll ptp 24.9°`）。
   * 影响：**第一步的阈值**与第二步的验收（§5.4 第 1 行）。
2. **【待测】p2l 的 `max_height 0.1` 把 0.4~1.0 m 的点全丢了，是谁的锅、要不要动？**
   * 已知：**【事实】** `docs/ground_segmentation_slots.md` §6.2 的高度带覆盖率表：**0.40~1.00 m 档覆盖率 0.0% / 7.3%**，
     并明确更正"**不是分割器的锅，是 `pointcloud_to_laserscan` 的 `max_height: 0.1`**"；
     后果：护墙上部只能靠 **STVL（`min_obstacle_height 0.0`）**那条路，`/scan` 里没有。
   * ⚠ **与我们三层的关系**：这条**只影响 `/scan`**，**不影响**走点云的坡度/净空层（点云没有高度带闸，`docs/mapping_2d_from_cloud.md` §10.3）。
   * 怎么确认：把 `p2l` 的 `max_height` 临时提到 1.0（与 `min_obstacle_height 0.0` 对齐）跑一次 `local_obstacle:=scan` vs `stvl` 的 A/B，
     看 `/scan` 的 0.4~1.0 m 档覆盖率是否从 ~0 升上去、且**近距假回波不增加**（对照：`docs/stvl_local_costmap.md` 的"贴墙 0.35~0.40 m 仍标到墙"）。
   * 影响：第三步的净空层若走 `/scan` 就**永远看不到 0.4 m 以上的横梁** ⇒ **必须走点云**。
3. **【待测】"车高"到底取哪个数？**
   * 已知：**【事实】** 仿真包络 ≈ **0.224 m**（§3.3，由 xacro 逐件加出来）；**真车** `sentry_robot_real.xacro` 的雷达在 **z = 0.49 m** ⇒ 整车更高。
   * 怎么定：① 对仿真用 `0.224 + margin`；② 对真车**重新量**（含云台/护板/天线的最高的那个点），
     并把结果写进 `docs/sim_real_contract.md` 那类契约文档；③ 用 `robot_radius: 0.22` 的语义作对照（它是**水平**半径，不是高度）。
   * 影响：净空层阈值（§3.3）与第三步验收（梁下 0.30 / 0.60 m 这两档）。
4. **【待测】本车的姿态极限（能不能在 23° 坡上保持可用定位/扫描？）**
   * 已知：**【事实】** 在 23° 坡段实测 LIO **pitch ptp 33.1°、roll ptp 24.9°**、坡道窗口内 `roll ptp 23.1°`
     （`docs/mapping_2d_from_cloud.md` §4.4），说明"底盘确实被顶起来/侧倾"；
     而 LIO 在这段的 `/scan` 有限束数只有其余段的 **31%**（同 §4.3）。
   * 怎么量：在"纯坡道"路线上用 `mapping_ab_run.sh --route …` 跑 5 次，报 `roll/pitch ptp`、
     `LIO ATE`、`/scan` 束数、`map→odom` 跳变哨兵的次数（工具已有）。
   * 影响：坡度层的**限速档**（15~25° 那档到底限到多少），以及"要不要在坡上触发"。
5. **【待测】动作层的触发怎么保证安全？（本文最没底的一条）**
   * 已知：**【事实】** 仓库里**没有**自定义 BT 节点、**没有** `default_bt_xml_filename`、**没有**自定义 `behavior_plugins`（§4.2）。
   * 开放点：① 触发是"**进入前**判"还是"**进入后**判"？（净空必须进入前，落空只能进入后）
     ② 触发之后**停在哪**：坡中/路口停车会不会更危险（**【事实】** 窄口处被物理卡死实测发生过 3 次，`docs/mapping_small_point_lio.md` §9）；
     ③ 状态过期怎么办（latched 话题 + 有效期字段 vs 每周期重算）。
   * 怎么确认：先在**离线回放**（`ros2 bag play` + 只起 `bt_navigator` + 假 costmap）里验 BT 分支，
     再 `mode:=nav` 单次跑，**最后**才 A/B。**【推断】** 这一步之前不动 BT。
6. **【待测】坑底/洼地的传感器覆盖（落空层的"看得见"前提）。**
   * 已知：**【事实】** MID360 的**下视 FOV 只有 −7.22°**（`src/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro` 的
     `<vertical><min_angle>${-7.22/180*M_PI}</min_angle>`），且 `docs/issues_and_findings.md` #26 的几何结论：
     "高 h 的墙需 `d ≥ (0.226 − h)/tan 7°`（h=0.15 m→0.62 m）⇒ **越近越看不见**"。
     **【推断】** 同一个几何对"坑"更狠：站在坑沿上，**坑底**在雷达的 −7.22° 以下 ⇒ **根本扫不到**。
   * 怎么量：在自建的 −0.2 m 坑代理场景里，把车停在坑沿不同距离（0.3/0.6/1.0/1.5 m），
     统计"坑底有回波的帧占比"与"坑沿被判落空的帧占比"；工具用现成的点云落盘 + `pcd_stats.py`。
   * 影响：**落空层的能力边界**（是"能看出这里掉下去了"还是只能"看出边沿"）。**【推断】** 大概率是后者 ⇒
     判据应当写成"**边沿/落差**检测"，而不是"坑底检测"。
7. **【待测】`p2l` 的 `range_min` 回退档位与落空层误报的耦合。**
   * 已知：**【事实】** 历史上 `range_min` 由 **0.45 → 0.2 → 0.12 → 0.05**（`docs/algorithm_matrix.md` §七 2026-09-21 行 +
     `config/laserscan_params.yaml` 的两次注释），当前 **0.05**；`docs/stvl_local_costmap.md` §168/§232 实测"45 cm 盲区已不存在"。
     `config` 里留的回退档位是 **0.15 / 0.2**。
   * 风险：**【推断】** 一旦回退到 0.15/0.2，车体半径 0.2 m 内的地面就没有回波 ⇒ **"无地面回波"这条判据会把车下一圈判成落空**。
   * 怎么确认：把落空层在"回退档位"下重跑同一段数据，报车周 0.5 m 环带上的 FP 计数。
   * 影响：落空层判据（§3.2）是否必须加"**车体半径内不判**"这条豁免。
8. **【待测】1.05 m 与 1.00 m 的走廊宽度差到底是什么口径？**
   * 已知：**【事实】** `docs/mapping_2d_from_cloud.md` §4.1 记"宽 **1.05 m**"（map 系 `y∈[-7.35,-8.35]` ⇔ world `y∈[-5.825,-4.825]`，即 **1.00 m 跨度**）；
     **【本次核对】** 0.02 m 栅格量到平台面（`lift=0.20`）在 x=2.5~7.3 上**恒为 1.00 m**；
     剖面显示 `y=-5.90` 是低处地面、`y=-5.85` 已到 0.20 m，`y=-4.73` 回到低处 ⇒ **墙到墙的净宽 ≈ 1.05~1.07 m**。
   * **【推断】** 差的是"薄墙/台阶沿自己的厚度"：0.05 m 栅格把"面"和"沿"各算了一格。**结论（供验收用）**：
     写验收时**必须写明口径**（"平台面宽度 1.00 m" 还是 "墙面到墙面 1.05 m"），否则 5 cm 的差会被当成 bug。
   * 怎么确认：用 0.05 m 栅格重算一次平台面宽度，与 0.02 m 的结果并列写下来。
9. **【未确认】仓库里到底有没有自定义 BT 节点 / 自带行为树 XML？**
   * 本文的结论（"没有"）是**从 params 反推**的（`plugin_lib_names` 31 项全是 `nav2_*`、无 `default_bt_xml_filename`）。
   * 怎么确认（**写代码前第一件事**）：`grep -rn "BT_REGISTER_NODES\|bt_nodes\|default_bt_xml" src/ --include=*.cpp --include=*.hpp --include=*.py --include=*.xml`。
   * 影响：§4.2 的"现成钩子"清单与 §4.3 的挂载方式。
10. **【未验证】`nav2` 的 `static_layer` 能不能挂两个实例（两张图）？**
    * 本文把它当作"零代码把坡度层送进 costmap"的主路线（§3.1 路线 1），但**没有实测**。
    * 怎么确认：看 Humble 版 `nav2_costmap_2d` 的 `StaticLayer` 实现是否对 `map_topic` 有全局假设；
      最直接的实验是**加一个实例、给不同 `map_topic`/不同 `plugin` 名**跑一次 `mode:=nav`，看两张图是否都生效。
    * 影响：第二步是否必须写插件（若两个实例不行，"零代码路线"不成立）。
11. **【未验证】`grid_map` 2.0.1 在 Humble 上与我们这份 nav2/`tf2` 版本是否真的兼容。**
    * 本文推荐"离线出层、必要时装 `grid_map` 三个包"，但**没有装、没有跑**。
    * 怎么确认：`sudo apt install ros-humble-grid-map-ros ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin ros-humble-grid-map-costmap-2d`
      后 `ros2 pkg list | grep grid_map`、起 rviz2 看有没有 `GridMap` display。**【推断】** 风险低（官方发布），但**未验就是未验**。
12. **【未验证（外部资料）】elevation_mapping / elevation_mapping_cupy 在本机的可行性。**
    * 已知：**【事实】** 本机**没有** apt 包（`apt-cache search ros-humble-elevation` 空），上游有 ROS 2 分支
      （[cupy 的 ros2 README](https://raw.githubusercontent.com/leggedrobotics/elevation_mapping_cupy/ros2/README.md)）。
    * 怎么确认：先**不装**。判据是"我们自己那份离线判据站不住、而它的机器人中心高程图语义明显更好"（例如需要
      **姿态补偿/多传感器融合/滚动窗口**）。到那时再评估要不要为它引入源码编译与 GPU 依赖。

---

## 附录 A：本文引用到的关键事实索引（一句话 + 出处）

| 事实 | 出处 |
|---|---|
| 坡面在 `/scan` 累积图里 **100% unknown**（1680/1680），走廊 **2688 格全在图外** | `docs/mapping_2d_from_cloud.md` §6.1 / §6.3 / §10.4 |
| 点云投影图：坡面 **98.8% free**、走廊 **97.3% free** | 同上 §6.1 |
| 规划器对坡道/走廊 5 个目标：**2/5 → 5/5** | 同上 §6.4 |
| `pcd_to_nav2_map.py` 判据：`k=8` 法向、`--slope-limit 25°`、`--ground-cell 0.20 m`、p05、`--height-threshold 0.15 m` | 同上 §5.1；`--help` 默认值 |
| 「墙会被自己当地面」：1493 个占用格里 **1418** 个被吸收 | 同上 §5.1 / §10.3 |
| `map/RMUC2026.yaml` origin `[-25.925, -9.425]` = 场地左下角 − 出生点 `(10.925, 2.525)` | `docs/worlds.md` §3 / §4.1 |
| STL：45900 面、sha256 `cc723f31…`、`<scale>0.001`、抬升 `+1.6413436`、地板顶面 mesh `z=-1.64134362793`（4412 个顶点） | `docs/worlds.md` §1 / §2.1 |
| 场地可行驶地板 214.6 m²、free 237.8 m²、`clearance≥0.22` → 176 m²、`≥0.30` 裂成 80.6/80.5 m² | `docs/worlds.md` §6 |
| 两个半场最窄 **0.45 m**，"余量 5 mm/边 ⇒ 实际过不去"，绕行 41.7 m | 同上 |
| 隧道/设计坡道：**没有** | 同上 |
| 地面分割：22~35° 缓坡 linefit 判障碍 **35.7~44.3%** → patchwork **2.5~3.0%** | `docs/ground_segmentation_slots.md` §6.2 / §7.1 |
| `p2l` 的 `max_height 0.1` ⇒ **0.40~1.00 m 档覆盖率 ≈0** | 同上 §6.2（重要更正段） |
| `th_dist 0.125→0.08` 的 A/B（0.15 m 薄墙覆盖率 11.8% → 100%） | 同上 §3.1 |
| `ground` 槽位默认 **linefit**、不换的 5 个条件 | 同上 §4.1 / §7.1 / §7.2 |
| `/segmentation/ground` 每帧只有 **~700 点**（≈11 KB） | `docs/issues_and_findings.md` #25 的更正 |
| `p2l range_min` 历史 `0.45 → 0.2 → 0.12 → 0.05`；"45 cm 盲区已不存在" | `docs/algorithm_matrix.md` §七（2026-09-21 行）；`docs/stvl_local_costmap.md` §168/§232 |
| MID360 下视 FOV **−7.22°**、"越近越看不见" | `src/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro`；`docs/issues_and_findings.md` #26 |
| 2.5D 空缺槽位：`rm_elevation_map` / `rm_costmap_layers` | `docs/algorithm_matrix.md` §一.1 / §六；`docs/architecture.md` §3.2.7 |
| 维度决策：导航层**继续 2D**、2.5D 只两处增量、3D 不进在线决策 | `docs/algorithm_matrix.md` §8.2 |
| 图层槽位真值表与 `enabled` 生成逻辑 | `src/rm_navigation/rm_navigation/launch/navigation_launch.py:50-62, 86-106` |
| nav2 参数：`robot_radius 0.22`、`inflation_radius` 0.5/0.55、`max_obstacle_height 2.0`、`min_obstacle_height 0.0`(2026-09-24 由 0.2)、`bt_navigator.plugin_lib_names` 31 项、`behavior_plugins` 4 项、**无** `default_bt_xml_filename` | `src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml`（:82-121 / :136-147 / :160-197 / :225-251 / :288-295 / :303-355 / :378-385 / :402-412） |
| 本车几何：轮半径 0.06、轮宽 0.05、`base_link` 碰撞盒 0.2×0.3×0.1、IMU 盒顶 z=0.15、Livox 在 `+(0.12,0,0.175)` | `src/rm_nav_bringup/urdf/sentry_robot_sim.xacro`（:5-6 / :22-40 / :155-186 / :262-264） |
| 真车 Livox 在 `+(0.0,0.045,0.49)` | `src/rm_nav_bringup/urdf/sentry_robot_real.xacro`（:16） |

## 附录 B：复跑本文"本次核对"的命令

```bash
# ① 包可用性（§2.2）
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -iE 'octomap|grid|elevation|voxblox|nvblox|esdf|fiesta|voxel'
apt-cache policy ros-humble-grid-map-ros ros-humble-grid-map-msgs ros-humble-grid-map-rviz-plugin \
                 ros-humble-grid-map-costmap-2d ros-humble-octomap ros-humble-octomap-server \
                 ros-humble-octomap-rviz-plugins
apt-cache search ros-humble-elevation      # 期望：空（Humble 无 apt 包）

# ② STL 面级坡度/面积 + 台面面积（§5.3）
#    二进制 STL：80 B 头 + uint32 面数 + 每面 50 B（3f 法向 + 9f 顶点 + uint16）
python3 - <<'PY'
import numpy as np, struct
p='src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl'
d=open(p,'rb').read(); n=struct.unpack('<I',d[80:84])[0]
arr=np.frombuffer(d,dtype=np.uint8,count=n*50,offset=84).reshape(n,50)
T=np.frombuffer(arr[:,12:48].tobytes(),dtype='<f4').reshape(n,3,3).astype(float)*0.001
base=-1.64134362793; zl=T[:,:,2]-base
A=0.5*np.linalg.norm(np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]),axis=1)
nn=np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]); nm=np.linalg.norm(nn,axis=1)
ang=np.degrees(np.arccos(np.clip(np.abs(nn[:,2])/np.where(nm>0,nm,1),0,1)))
for lo,hi in [(0,5),(5,10),(10,15),(15,22),(22,35),(35,60),(60,91)]:
    m=(ang>=lo)&(ang<hi); print(f'{lo:3d}-{hi:3d} deg  {m.sum():6d} tris  {A[m].sum():8.2f} m2')
PY

# ③ 真值"净空"（§5.3 第 2 条）：本机没有 rtree/embreex/numba ⇒ 别用 trimesh.ray
#    做法：对每个 (x,y) 用三角形 XY bbox 粗筛 + 重心坐标求交，收集该列所有面高，排序后取"最高面之上第一个面"
#    （本文用量：0.10 m 栅格 46 400 列，7.9 s）
python3 -c "import importlib;[print(m, (lambda: (importlib.import_module(m), 'OK')[1])() if True else '') for m in ['rtree','embreex','numba']]" 2>&1 | tail -3
```
