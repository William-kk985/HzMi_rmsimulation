# rm_ground_traversability —— 地面分割之后的「坡度 / 台阶」判定（header-only）

> 归属：`src/rm_perception/rm_ground_traversability/`
> 消费者：`linefit_ground_segmentation_ros/ground_segmentation_node`（`ground:=linefit`）、
> `patchwork_ground_segmentation/patchwork_ground_segmentation_node`（`ground:=patchwork`）
> 阈值真源：`src/rm_nav_bringup/config/traversability_criteria.yaml`（**同一份**也被离线
> `tools/scripts/mapping/pcd_to_nav2_map.py` 读）

## 为什么有它

2D 先验图那一侧（离线 `pcd_to_nav2_map.py`）**一直有**"相对局部地面的高差 + 可行驶坡度"这套判据，
而**实时链路没有**：实时链路"什么算障碍"完全由 `linefit`（`max_slope ±0.4` ≈ 21.8°、
`max_dist_to_line 0.05 m`）或 `patchwork` 一家说了算。后果是**实时代价图与先验图口径不一致**：
`linefit` 会把 ≤22° 的坡面和 0.06~0.30 m 的低矮台面**整片判成地面删掉** ⇒
`/segmentation/obstacle` 里没有它、`/scan` 里没有它、局部代价图里没有它
⇒ 规划器从"看不见的坡脚/台阶"上开过去（实测 IMU 79~90 m/s²，见
`docs/path_clearance_and_contact.md` §2.2）。

## 判据（与 `pcd_to_nav2_map.py` 逐条同构）

```
局部地面 g(x,y)  = 该点所在 ground_cell(0.20 m) 粗格内、各 fine_cell(0.05 m) 细格"最低点"的 p05
可行驶(drivable) ⟺ z − g ≤ step_height_threshold(0.15 m)
                  且 不是（局部地面坡度 > drivable_slope_deg(25°) 且 z − g > slope_min_height(0.05 m)）
台阶/边沿(step_edge) ⟺ 判据命中（高度台阶 ∪ 超限坡面）
```

* **只降不升**：只把上一级判成 `ground` 的点降级为 `obstacle`，永不反向 ⇒ 不删任何已有障碍。
* **可行驶坡面必须保持 free**：`0.20 m × tan25° = 0.093 m < 0.15 m` ⇒ 任何 ≤25° 的坡都过不了高度闸；
  坡度判据另带 0.05 m 高度闸，24° 的长坡也不会被误判。
* **完备性**：每个输入点仍恰好落在 `ground` 或 `obstacle` 之一（与两个节点的原契约一致）。

## 契约

| 项 | 值 |
|---|---|
| 新增**诊断**话题 | `segmentation/step_edge`（`sensor_msgs/PointCloud2`，判据命中的点 = `obstacle` 的**子集**），可用 `publish_step_edge:=false` 关掉 |
| 新增**私有**话题 | `~/traversability_stats`（`std_msgs/String`，每帧一行 JSON：点数/降级数/耗时），可用 `publish_traversability_stats:=false` 关掉 |
| `/segmentation/ground` / `/segmentation/obstacle` | **发布者个数、消息类型、帧处理、QoS 全不变**（各 1 个发布者） |
| 参数不合法的处置 | 构造期直接抛异常拒绝启动（不允许"判据静默不生效"） |

## 参数（键名 = 真源 YAML / `pcd_to_nav2_map.py` 的 CLI）

| 键 | 默认 | 对应离线 CLI |
|---|---|---|
| `traversability_enable` | `true` | （离线没有开关；`--height-threshold` 巨大即等价） |
| `step_height_threshold` | `0.15` | `--height-threshold` |
| `drivable_slope_deg` | `25.0` | `--slope-limit` |
| `slope_min_height` | `0.05` | `--slope-min-height` |
| `ground_cell_m` | `0.20` | `--ground-cell` |
| `fine_cell_m` | `0.05` | `--resolution` |
| `ground_percentile` | `5.0` | `--ground-percentile` |
| `ground_min_points` | `2` | `--ground-min-points` |
| `publish_step_edge` | `true` | — |
| `publish_traversability_stats` | `true` | — |

一致性断言：`python3 tools/scripts/regress/check_traversability_criteria.py`（YAML ↔ C++ 默认值 ↔ Python 兜底）。

## 用法 / 验证 / 回退

见 `docs/ground_segmentation_slots.md` §10（判据）、§11（p2l 高度带）、§12（重跑建图 recipe）、
§13（整栈验证）、§14（回退表）。A/B 装置：`tools/scripts/regress/low_terrain_ab.py`。

## 许可

BSD-2-Clause（与本仓库其它自研包一致）。**没有 vendored 代码**：上游 Patchwork++ 的许可证考证见
`src/rm_perception/patchwork_ground_segmentation/thirdparty/patchwork-plusplus/VENDORING.md`。
