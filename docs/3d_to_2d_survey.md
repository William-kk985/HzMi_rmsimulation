# 3D → 2D 是怎么做的：各家实现对照（伪 2D 导航的共性手法）

> 结论先说：**"伪 2D"不是某一家的做法，而是整个生态的通用套路** —— 所有地面导航栈都在某个环节做
> "高度带筛选 + 水平投影"，差别只在**在哪一层做、用什么投影规则、自由空间怎么定**。
> 本文按系统里实际装了/源码在手的实现逐个对照（带源码位置），最后给出自己写这类模块的清单。

---

## 一、对照表

| 实现 | 降维发生在哪一层 | 机制 | 关键参数（默认值） | 自由空间怎么定 |
|---|---|---|---|---|
| `pointcloud_to_laserscan`（本工程 + apt） | 传感器 → `/scan` | 高度带筛选 + 按**水平角度装箱**、每箱取**最小距离** | `min_height/max_height`（默认 ±inf）、`angle_min/max`、`range_min/max` | 不产 free，交给下游 |
| `depthimage_to_laserscan`（apt 已装） | 深度图 → `/scan` | 取若干**行**像素，按列取最小深度 | `scan_height`、`output_frame`、`range_min/max` | 同上（相机版同一思路） |
| nav2 `ObstacleLayer` | costmap 图层 | 点云源按**高度带**筛选后逐点 mark/clear | 源级 `min/max_obstacle_height`(0.0/0.0)、层级(0.0/2.0)、`obstacle_max_range`(2.5) | **raytrace 射线清除** |
| nav2 `VoxelLayer` | costmap 图层 | **3D 体素列**（每列 32 位）+ **计数阈值**决定 2D 格 | `z_voxels 10`、`z_resolution 0.2`、`origin_z 0.0`、`mark_threshold 0` | 体素内 raytrace |
| `spatio_temporal_voxel_layer`（apt，本工程 global costmap 用） | costmap 图层（**2.5D**） | 3D 体素 + **时间衰减** + `combination_method: max` 投影到 2D | `voxel_size 0.05`、`voxel_decay 0.5`、`min/max_obstacle_height 0.2/2.0`、`mark_threshold 0` | raytrace + 衰减 |
| **cartographer 2D** | **SLAM 内部** | 在**重力对齐系**里 `CropRangeData(min_z, max_z)` + voxel filter | `min_z = -0.8`、`max_z = 2.0` | 射线插入（hit/miss 概率） |
| cartographer 3D | SLAM 内部 | 自适应体素滤波 + range 限制 | `high_resolution_max_range` 等 | 3D 概率栅格 |
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

---

## 二、五个共性旋钮（"伪 2D"的通用配方）

| # | 旋钮 | 各家叫法 |
|---|---|---|
| 1 | **高度带（band-pass）** | `min/max_obstacle_height`、`min_z/max_z`、`min/max_height`、`occupancy_min_z/max_z`、`scan_height` |
| 2 | **投影规则** | 并集（band 内任一命中）/ **计数阈值**（nav2 `mark_threshold`）/ **取最大**（STVL `combination_method: max`） |
| 3 | **自由空间** | 射线清除（cartographer / nav2 raytrace / octomap）vs 洪泛（本工程离线工具） |
| 4 | **时间维** | 多帧累积（cartographer `num_accumulated_range_data`）、衰减（STVL `voxel_decay`）、观察持久化（obstacle layer `observation_persistence`） |
| 5 | **代价** | 由 2D 膨胀层生成（`252·exp(−scale·(d−r_in))`，即 2D 距离场） |

**只有第 3 条（自由空间）是真正的质量分水岭**：射线清除能给出可信的 free space，洪泛/并集只能近似。

---

## 三、三个反直觉但值得学的细节

1. **nav2 `VoxelLayer` 把低于 `origin_z` 的点"夹进最底层体素"**（`worldToMap3D(x, y, origin_z, ...)`），
   而不是丢弃 —— 地面附近的点不会凭空消失（`voxel_layer.cpp` L213-214）；
2. **cartographer 2D 在"重力对齐系"里裁剪高度**（先按姿态对齐再切 z），比在雷达原始系里切更合理 ——
   车辆倾斜时仍切在水平面上；
3. **`pointcloud_to_laserscan` 每个角度箱取"最小距离"**（`if (range < ranges[index])`），
   保证近处障碍不会被更远的点覆盖（不会出现"透过近处障碍看到远处"的假自由）。

---

## 四、自己写这类模块时的检查清单（照抄上面的做法）

1. 先定"**机器人碰撞体的高度范围**" → 得出高度带（下界=能跨越的高度，上界=车身净空）；
2. 选**投影规则**并明确"哪些高度算障碍"（并集 / 计数阈值 / 取最大）；
3. **自由空间必须用射线清除**（别用洪泛），否则未知区会被当成可走；
4. 加**时间维**（多帧累积或衰减）对抗噪点与动态物；
5. 最后交给膨胀层生成代价（"贴墙更贵"）。

---

## 五、对本工程的对照与可借鉴点

- 我们现在的链路与 cartographer/nav2 **同源**：`linefit`（3D 去地面）→ `p2l`（高度带切片→`/scan`）/ STVL（体素→max 投影）；
- **可借鉴的三点**：
  1. 离线工具 `pcd_to_grid_map.py` 补**射线清除**（用 LIO 的 `/path` 当射线原点），把"洪泛 free"升级为"射线 free"；
  2. 未来 2.5D 图层用**计数阈值**（需 N 个体素/帧才标记）而不是"任一命中"，提高抗噪；
  3. 高度带可以在**重力对齐系**里算（我们目前是在 `livox_frame` 系里切，车体倾斜时会有偏差）。

> 相关章节：`docs/architecture.md` §3.2.2（降维发生在哪一层）、§3.2.5（2D 为何能承载 3D）、
> `docs/algorithm_matrix.md` §一.1（维度归类，含三处"跨维度桥"）。
