# 本目录是 **cartographer 核心的 fork**（不是上游 pristine 副本）

- 基线：`ros2/cartographer` tag **2.0.9004**（= 本机 `ros-humble-cartographer` 的对应源码）。
- 上游 pristine 参考副本仍在 `third_party/cartographer`（**未改动**，且被 `third_party/COLCON_IGNORE` 排除在构建外）。
- 之所以放进 `src/rm_localization/`：需要它覆盖 `/opt/ros/humble` 的系统包才能生效。

## 本 fork 唯一的改动：**清除豁免（clearance exemption）**

| 文件 | 改动 |
|---|---|
| `cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.proto` | 新增字段 `double min_probability_to_clear = 4;`（proto3，默认 0） |
| `cartographer/mapping/2d/probability_grid.h` / `.cc` | 新增 `ProbabilityGrid::GetProbabilityWithoutMarker()`（允许读取本轮已带 `kUpdateMarker` 的格子） |
| `cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc` | 新增 `SkipClearing()`；在 `CastRays` 的两处 miss 循环里跳过"P(occupied) ≥ 阈值"的格子；选项解析 + `CHECK_GE/CHECK_LT` |

语义：`min_probability_to_clear = 0` ⇒ **与上游逐字节同行为**；`= 0.80` ⇒ 已经（较）确信是障碍的格子
不再被"穿过"的射线写成 free，而空格子照旧被清成自由。原因与实测数据见
`docs/debug_fastlio_cartographer.md` §5.2.4（十三次修正）与 `tools/diag_map_loss.py`。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cartographer cartographer_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

依赖（本机已装）：`libceres-dev libprotobuf-dev libabsl-dev libcairo2-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libeigen3-dev libboost-iostreams-dev`。

## 副作用（必须知道）

动态障碍物一旦被判为占据（P ≥ 阈值）就**不会**再被"穿过"的射线清除，只能靠 `insert_free_space`
之外的机制（本场景无动态物）。实车/`pure_localization` 复用时若需要清除能力，把阈值调高（0.9）
或设回 0。
