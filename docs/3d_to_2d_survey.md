# 3D → 2D 是怎么做的：各家实现对照（伪 2D 导航的共性手法）

> 结论先说：**"伪 2D"不是某一家的做法，而是整个生态的通用套路** —— 所有地面导航栈都在某个环节做
> "高度带筛选 + 水平投影"，差别只在**在哪一层做、用什么投影规则、自由空间怎么定**。
> 本文按系统里实际装了/源码在手的实现逐个对照（带源码位置），最后给出自己写这类模块的清单。
> **§二 先分清两种降维哲学（"几何 2D" vs "体素 2D"）**，§三 起才是共性旋钮与逐条对照。

---

## 一、对照表

| 实现 | 降维发生在哪一层 | 机制 | 关键参数（默认值） | 自由空间怎么定 |
|---|---|---|---|---|
| `pointcloud_to_laserscan`（本工程 + apt） | 传感器 → `/scan` | 高度带筛选 + 按**水平角度装箱**、每箱取**最小距离** | `min_height/max_height`（默认 ±inf）、`angle_min/max`、`range_min/max` | 不产 free，交给下游 |
| `depthimage_to_laserscan`（apt 已装） | 深度图 → `/scan` | 取若干**行**像素，按列取最小深度 | `scan_height`、`output_frame`、`range_min/max` | 同上（相机版同一思路） |
| nav2 `ObstacleLayer` | costmap 图层 | 点云源按**高度带**筛选后逐点 mark/clear | 源级 `min/max_obstacle_height`(0.0/0.0)、层级(0.0/2.0)、`obstacle_max_range`(2.5) | **raytrace 射线清除** |
| nav2 `VoxelLayer` | costmap 图层 | **3D 体素列**（每列 32 位）+ **计数阈值**决定 2D 格 | `z_voxels 10`、`z_resolution 0.2`、`origin_z 0.0`、`mark_threshold 0` | 体素内 raytrace |
| `spatio_temporal_voxel_layer`（apt，本工程 global costmap 用） | costmap 图层（**2.5D**） | 3D 体素 + **时间衰减** + `combination_method: max` 投影到 2D | `voxel_size 0.05`、`voxel_decay 0.5`、`min/max_obstacle_height 0.2/2.0`、`mark_threshold 0` | raytrace + 衰减 |
| **cartographer 2D** | **SLAM 内部** | 在**重力对齐系**里 `CropRangeData(min_z, max_z)` + voxel filter；**输入可以是 PointCloud2 也可以是 LaserScan** | `min_z = -0.8`、`max_z = 2.0`（⚠️ **相对传感器**，不是相对地面） | 射线插入（hit/miss 概率） |
| **cartographer 3D** | SLAM 内部（**3D 后端**）→ 输出时再投影一次 | 3D hybrid grid（high/low 两套分辨率的概率体素）+ 自适应体素滤波；对外仍发 2D 栅格：`Submap3D::AddToTextureProto` 做 **"X-ray view through the hybrid_grid, aligned to the xy-plane"** —— 按 (x,y) 列累积 `min_z/max_z/count`，**列厚度 < 3 个体素就判透明**，否则按平均占据概率出像素 | `TRAJECTORY_BUILDER_3D.num_accumulated_range_data`（backpack_3d: 160）、`submaps.high_resolution`、`high_resolution_max_range`；输入必须 `num_point_clouds ≥ 1`（PointCloud2） | 3D 概率体素（hit/miss 累积）；投影出的 2D 图里"透明"**不等于**射线测得的 free |
| `slam_toolbox` | — | **不做** 3D 处理，只吃 `LaserScan` | `scan_topic` | 2D 射线 |
| `nav2_amcl` | — | **不做** 3D 处理，只吃 `LaserScan` + 栅格图 | `scan_topic`、`laser_max_range` | 似然场 |
| `octomap_server`（**未安装**，需要时 apt 装） | 独立建图节点 | 八叉树 + 射线清除，直接发 `projected_map`（2D） | `occupancy_min_z/max_z`、`filter_ground` | **射线清除** |
| `pcl_ros`（apt 已装） | 预处理 | passthrough（高度带）/ voxel grid / statistical outlier | — | — |
| 本工程 `linefit_ground_segmentation` | 感知 | 3D 里按**高度 + 坡度**剔除地面 | `sensor_height 0.275`、`min/max_slope ±0.4`、`max_dist_to_line 0.1` | 决定"什么算可行驶地面" |
| 本工程 `tools/pcd_to_grid_map.py` | 离线工具 | 高度带 + 栅格化 + **洪泛求 free** | `--min-z/--max-z`、`--margin` | **洪泛（近似，无射线清除）** |

**源码位置（可自查）**：
- `src/rm_perception/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp` L191（高度带）、L225（每角度取最小距离）
- `third_party/nav2/nav2_costmap_2d/plugins/obstacle_layer.cpp` L470/L476（高度带 continue）
- `third_party/nav2/nav2_costmap_2d/plugins/voxel_layer.cpp` L213-224（体素标记）+ `third_party/nav2/nav2_voxel_grid/include/nav2_voxel_grid/voxel_grid.hpp` L99-116（列计数阈值）
- `third_party/cartographer/configuration_files/trajectory_builder_2d.lua` L19-20（`min_z/max_z`）+ `cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc` L54-66（裁剪）
- `third_party/cartographer/cartographer/mapping/3d/submap_3d.cc` `AddToTextureProto` / `ComputePixelValues`（3D → 2D 的 X-ray 投影；`kMinZDifference = 3.f`、`kFreeSpaceWeight = 0.15f`）
- `third_party/cartographer/cartographer/mapping/2d/submap_2d.cc`（2D 子图真身：概率栅格 + 射线插入）

---

## 二、"几何 2D" vs "体素 2D"：同一朵点云，两种降维哲学（**两道关，不是二选一**）

| | **几何 2D**（几何化：点云 → 射线） | **体素 2D**（体素化：点云 → 体素列） |
|---|---|---|
| 代表 | `pointcloud_to_laserscan`、`depthimage_to_laserscan` | nav2 `VoxelLayer`、STVL |
| 发生在**哪一层** | **感知层**（SLAM 之前） | **地图层**（costmap 图层，SLAM 之后） |
| 产物 | 一帧若干 `range`（`LaserScan`） | 每个 (x,y) 列一串 3D 体素 |
| 测量的语义 | **射线**：`range` 之前的空间必为 free | **体素**：列里哪些格子被占据 |
| 高度信息 | **销毁**（只剩 range/角度） | **保留**（点的 z 就是障碍自己的高度） |
| 投影规则 | 高度带 + **每角度取最小**（近处优先） | **计数阈值**（`mark_threshold`）/ **取最大**（STVL `combination_method: max`） |
| 自由空间 | **物理测量**（射线） | **推断**（raytrace 清除 / 时间衰减） |
| 时间维 | 无（帧间独立） | 有（`voxel_decay`、观察持久化） |
| 谁消费 | SLAM / 定位（slam_toolbox、AMCL、cartographer-2D、ICP）+ costmap | **只有 costmap**（SLAM/定位吃不到） |
| 代价 | 便宜（一帧几百个数，2D 匹配） | 贵（每帧 3D 点云写入体素网格；STVL 还带衰减） |
| 典型失效 | 一次决策、**全局承担、不可逆**：高度带切掉某类障碍，地图与 costmap 就都永远看不到它 | 每个消费者各做一遍 3D 处理；对建图/定位**零贡献** |

**一句话分工：几何 2D 决定"地图里有什么"，体素 2D 决定"路上躲什么"。**

在本工程里它们是**串联的两道关** —— 这正是 `global_obstacle:=scan|stvl` 只切第二道关、
不影响建图与定位的原因：

```
3D 点云 ──[第一道关：几何化]──> /scan ──┬──> SLAM / 定位（写进地图）
                                        └──> local/global costmap 障碍层
        └─[第二道关：体素化]──> STVL / VoxelLayer ──> costmap 障碍层（仅此一路）
```

**四个容易搞混的点**：

1. **不是所有 SLAM 都必须走几何 2D**：`slam_toolbox` / `nav2_amcl` **只**吃 `LaserScan`（必须过第一道关）；
   而 **cartographer 2D 两种输入都吃**（`num_laser_scans` 或 `num_point_clouds`，后者由它自己在重力对齐系里做高度带）
   —— "cartographer 能不能直接吃点云"的答案是"能"，但它一样会先降成 2D 再匹配。
2. **体素 2D 救不了建图**：地图是 SLAM 写的，而 SLAM 只看 `/scan`；体素层再精细也不进地图。
3. **不可逆性是本工程最容易踩的坑**：`p2l` 的高度带一次定死，之后 global/local costmap、AMCL、
   slam_toolbox 全在这个"结论"上工作 —— **高度准入（决策）做了、高度信息（数据）没留**
   （见 `architecture.md` §3.2.5）。
4. **扫描内运动补偿也在第一道关丢失**：`p2l` 把 `time_increment` 写成 0
   （`pointcloud_to_laserscan_node.cpp` L150），即不去畸变。仿真里无所谓（Gazebo ray sensor
   一次 update 的所有点用同一瞬时位姿，本来就不建模扫描内运动）；**真机**上 Livox 逐点扫描 +
   小陀螺旋转是真实畸变源 → 迁移时要么建图期 `spin_speed:=0.0`，要么在第一道关之前补去畸变
   （例：改用 LIO 去畸变后的点云当 `p2l` 输入，代价是感知链从此依赖 LIO）。

---

## 三、五个共性旋钮（"伪 2D"的通用配方）

| # | 旋钮 | 各家叫法 |
|---|---|---|
| 1 | **高度带（band-pass）** | `min/max_obstacle_height`、`min_z/max_z`、`min/max_height`、`occupancy_min_z/max_z`、`scan_height` |
| 2 | **投影规则** | 并集（band 内任一命中）/ **计数阈值**（nav2 `mark_threshold`）/ **取最大**（STVL `combination_method: max`） |
| 3 | **自由空间** | 射线清除（cartographer / nav2 raytrace / octomap）vs 洪泛（本工程离线工具） |
| 4 | **时间维** | 多帧累积（cartographer `num_accumulated_range_data`）、衰减（STVL `voxel_decay`）、观察持久化（obstacle layer `observation_persistence`） |
| 5 | **代价** | 由 2D 膨胀层生成（`252·exp(−scale·(d−r_in))`，即 2D 距离场） |

**只有第 3 条（自由空间）是真正的质量分水岭**：射线清除能给出可信的 free space，洪泛/并集只能近似。

---

## 四、三个反直觉但值得学的细节

1. **nav2 `VoxelLayer` 把低于 `origin_z` 的点"夹进最底层体素"**（`worldToMap3D(x, y, origin_z, ...)`），
   而不是丢弃 —— 地面附近的点不会凭空消失（`voxel_layer.cpp` L213-214）；
2. **cartographer 2D 在"重力对齐系"里裁剪高度**（先按姿态对齐再切 z），比在雷达原始系里切更合理 ——
   车辆倾斜时仍切在水平面上；
3. **`pointcloud_to_laserscan` 每个角度箱取"最小距离"**（`if (range < ranges[index])`），
   保证近处障碍不会被更远的点覆盖（不会出现"透过近处障碍看到远处"的假自由）。

---

## 五、自己写这类模块时的检查清单（照抄上面的做法）

1. 先定"**机器人碰撞体的高度范围**" → 得出高度带（下界=能跨越的高度，上界=车身净空）；
2. 选**投影规则**并明确"哪些高度算障碍"（并集 / 计数阈值 / 取最大）；
3. **自由空间必须用射线清除**（别用洪泛），否则未知区会被当成可走；
4. 加**时间维**（多帧累积或衰减）对抗噪点与动态物；
5. 最后交给膨胀层生成代价（"贴墙更贵"）。

---

## 六、对本工程的对照与可借鉴点

- 我们现在的链路与 cartographer/nav2 **同源**：`linefit`（3D 去地面）→ `p2l`（高度带切片→`/scan`）/ STVL（体素→max 投影）；
- **可借鉴的三点**：
  1. 离线工具 `pcd_to_grid_map.py` 补**射线清除**（用 LIO 的 `/path` 当射线原点），把"洪泛 free"升级为"射线 free"；
  2. 未来 2.5D 图层用**计数阈值**（需 N 个体素/帧才标记）而不是"任一命中"，提高抗噪；
  3. 高度带可以在**重力对齐系**里算（我们目前是在 `livox_frame` 系里切，车体倾斜时会有偏差）。

> 相关章节：`docs/architecture.md` §3.2.2（降维发生在哪一层）、§3.2.5（2D 为何能承载 3D）、
> `docs/algorithm_matrix.md` §一.1（维度归类，含三处"跨维度桥"）。

---

## 七、本工程里 3D→2D **分布在哪几个域**（为什么"看着很依赖 nav"）

**答案：降维发生两次，一次在感知域（我们自己的代码），一次在导航域（第三方 costmap 图层，我们只给参数）。**

| # | 位置 | 域 | 谁写的 | 输入 → 输出 | 配置在哪 |
|---|---|---|---|---|---|
| ① | `linefit_ground_segmentation_ros` | **感知** `src/rm_perception/` | 本仓 | 3D 点云 → 3D 障碍点云（**去地面，不算降维**） | 包内 `config/segmentation_sim.yaml` |
| ② | `pointcloud_to_laserscan` | **感知** `src/rm_perception/` | 本仓 | 3D 障碍点云 → **2D `/scan`**（高度带 + 每角度取最小） | 包内 `config/laserscan_params.yaml` |
| ③ | `global_costmap.stvl_layer` | **导航** `src/rm_navigation/` | **apt 第三方插件**（STVL），我们只配参 | 3D 障碍点云 → **2D 代价**（体素 + 高度带 + max 投影） | `params/nav2_params_sim_*.yaml` |
| ④ | `local_costmap.obstacle_layer` | **导航** `src/rm_navigation/` | nav2 官方插件，我们只配参 | `/scan`（已是 2D）→ 2D 代价（mark/clear） | 同上 |
| ⑤ | `tools/pcd_to_grid_map.py` | **工具**（离线） | 本仓 | 3D `.pcd` → `.pgm/.yaml` | CLI 参数 |

**所以"用到挺多 nav 的东西"是正常的**：nav2 的代价地图本身就是**图层化**架构，"
把传感器数据变成代价"这一层天然属于它 —— 只要用 nav2，nav 域里就必然有降维配置。

**是否需要依赖 nav？分两种情况**：
- **② `/scan` 这一路不依赖 nav**：把 nav2 整个去掉，`linefit` + `p2l` 照样发 `/scan`（只是没消费者）。
  它的消费者（AMCL、slam_toolbox、cartographer-2D、`local_costmap`）在 nav 域，所以"看起来依赖"；
- **③ STVL 这一路完全在 nav 域内完成**（它直接收 3D 点云）→ 这一路是真·依赖 nav2 的图层机制 + 第三方插件。

**为什么要有两路（不是重复）**：两类消费者的需求不同 ——
- `/scan` 要的是**标准 2D 传感器消息**（ROS 生态通用接口，能接一堆现成的 2D 定位/SLAM 算法）；
- STVL 要的是**保留高度与时间**（高度带可调、有时间衰减）。

**代价（本工程已经踩到）**：两处降维 → **阈值不一致**：
`local` 用 `/scan`（相对车顶 `z∈[-1.0, +0.1]`），`global` 用 STVL（`z∈[0.2, 2.0]`）→
同一个 0.1 m 矮台，local 会绕、global 视而不见（见 architecture §3.2.4 的一致性风险）。

**global 改用 `/scan` 后的注意点（2026-09 实测发现一个）**：

| 注意点 | 原因 | 处理 |
|---|---|---|
| **量程必须按场地尺度设** | global 是全图（RMUL 13×10 m），而 raytrace 只能清 `raytrace_max_range` 以内的旧标记 → 照抄 local 的 6 m 会累积幽灵障碍 | 与 `p2l` 的 `range_max` 对齐（**10 m**）：能看到多远，就能清多远 |
| 失去高度判别 | 高度带在 `p2l` 一处定死，global 无法独立判断多高算障碍 | 平地可接受；需要高度判别时用 `global_obstacle:=stvl` |
| 新增单点依赖 | global 的实时障碍也依赖 `p2l` 这一环（p2l 挂了 global 也没实时障碍） | 监控 `/scan` 频率 |

> **澄清一个常见误解**：`/scan` 路径并不是"没有高度处理"，而是——
> **高度准入（决策）✔ 已在 `p2l` 做完；高度信息（数据）✘ 没有保留**。
> 即"降维把 3D 判断固化成了结论，结论里不再有高度"（`docs/architecture.md` §3.2.5）。
> 对比 `stvl`：它收 3D 点云，每个点的 z 就是**障碍自己的高度**，所以能做高度筛选与分层。

**把"都用 `/scan`"后的依赖关系画清楚**（`global_obstacle:=scan` + local 也用 `/scan`）：

```
3D 雷达 → /livox/lidar/pointcloud
   ├─ linefit（感知域；只需点云，不需要 LIO/nav）      → /segmentation/obstacle（3D）
   │     └─ pointcloud_to_laserscan（感知域；只需点云 + URDF TF）→ /scan（2D）★降维终点
   ├─ LIO（定位域）→ odom→base_link（供 TF，不参与降维）
   └─ （可选）STVL（**nav 域插件**）→ 2D 代价   ← 只有 global_obstacle:=stvl 时才用

/scan → local_costmap.obstacle_layer   （nav 域：消费 2D）
/scan → global_costmap.obstacle_layer  （nav 域：消费 2D，即 scan 模式）
/scan → amcl / slam_toolbox / cartographer-2D（导航相关：消费 2D）
                    ↓
        planner / controller / behaviors（nav 域：纯 2D 决策）
```

结论：
1. **降维动作 100% 落在感知域**（`linefit` + `p2l`）；nav 域里**不再出现任何 3D 数据**，也就"不依赖 nav 做降维"；
2. **感知链本身也不依赖 LIO**：`linefit` 只要点云，`p2l` 只要点云 + URDF 的 `base_link→livox_frame` TF
   （所以 `mode:=mapping` 里它们照样在跑）；
3. nav 域剩下的只有"**2D 代价化（inflation）→ 规划 → 控制 → 行为**"，那是它的本职，不是降维；
4. 顺带收益：可以**完全不用 STVL**（少一个 apt 第三方插件 + 省掉最重的 CPU 层）。

**自己验证**（关掉 STVL 后，nav 域是否还碰 3D）：
```bash
ros2 param get /global_costmap/global_costmap.stvl_layer.enabled     # False
ros2 topic info /segmentation/obstacle --verbose | grep -c "SUBSCRIPTION"   # 只剩 p2l(+RViz)
ros2 topic info /scan --verbose | grep "Node name"                          # local/global costmap + amcl + rviz
```

**改进方向（低成本 → 彻底）**：
1. **阈值成对校准**并把"成对"写进注释；
2. **统一来源 —— 已实现为可切换槽位**（2026-09）：`global_obstacle:=stvl|scan|none`，
   默认 `stvl`（=方案 A，行为不变）；`scan` 即方案 C（global 与 local 同源，都吃 `/scan`，
   "什么算障碍"只在 `p2l` 一处决策）；`none` 即方案 B。两个图层都写进 `plugins`、
   只用 `enabled` 切换，因此**可以逐项 A/B**（甚至运行期 `ros2 param set ... enabled` 动态切换）；
3. **彻底分层**：把"环境表示"（栅格 + 2.5D 高程/净空 + 体素）全部放在感知域产出，nav 侧只留一个薄图层插件消费它 ——
   这正是 `src/rm_perception/rm_elevation_map/` + `src/rm_navigation/rm_costmap_layers/` 的规划（architecture §3.2.7）。
