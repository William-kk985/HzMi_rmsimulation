# Cartographer 2D occupancy-grid semantics — verbatim source analysis

Prepared for debugging "occupied cells keep getting erased" in this workspace.
Every excerpt below was copied verbatim from the real sources (fetched with web tools /
read from the vendored checkout). Line numbers are those of the quoted revision.

> **2026-10 合并说明**：本文 §1–§4 是逐字源码分析（原样保留）。仓库整理时，把同主题的**未入库**报告
> `cartographer_map_stability_report.md`（仓库根，455 行，2026-09-23，英文）中**本文没有的**证据并入
> **§5 合并附录**（来源标记、issue 调查、TSDF、替代方案、调参表），随后删除该未入库文件；§5.1 列了
> "原文哪些结论与本文重复、因此不再重抄"的对应表。**§1–§4 仍是本仓唯一的逐字源码依据；§5 是外部调研补遗。**

## Provenance (read first — two of the requested paths no longer exist)

| Item | Requested | What actually exists |
|---|---|---|
| 1 | `cartographer/mapping/internal/2d/range_data_inserter_2d.cc` / `.h` | **Renamed**: `cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc` / `.h` (class `ProbabilityGridRangeDataInserter2D`). Verified against `cartographer-project/cartographer` master, tree SHA `877157a0d91788a7700221d87232d412cb3c1ef4` (commit `877157a`). |
| 3 | `sensor_bridge.cc` → `HandleLaserScanMessage`, `ToRangeData`, `ToPointCloudWithIntensities` | `HandleLaserScanMessage` **is** in `sensor_bridge.cpp/.cc`, but the conversion itself is in `msg_conversion.cpp` in the ROS2 tree. **`ToRangeData` does not exist in any current branch** (checked `cartographer_ros` master, `ros2-dashing`, and the ROS2 fork) — it was removed years ago. |
| 4 | `occupancy_grid_node_main.cc` | That file contains **no** int8 conversion; it fetches submap textures and calls `CreateOccupancyGridMsg()` in `msg_conversion.cpp`. The int8 mapping lives there, plus the texture encoding in `cartographer/io/submap_painter.cc` and `cartographer/mapping/2d/probability_grid.cc`. |

**Version verification for this machine.** `dpkg` shows `ros-humble-cartographer 2.0.9004` and
`ros-humble-cartographer-ros 2.0.9002`. For ROS 2, rosdistro does **not** use
`cartographer-project/*` master; it uses:

* core: <https://github.com/ros2/cartographer> branch `ros2` (tag `2.0.9004`)
* ROS glue: <https://github.com/ros2/cartographer_ros> branch `ros2` (tag `2.0.9002`)

I cloned both tags and diffed against `cartographer-project` master / the ROS2 tip:
**all of `probability_grid_range_data_inserter_2d.{cc,h}`, `probability_grid.{h,cc}`,
`probability_values.{h,cc}`, `ray_to_pixel_mask.cc`, `local_trajectory_builder_2d.cc`,
`msg_conversion.cpp`, `sensor_bridge.cpp`, `occupancy_grid_node_main.cpp` are byte-identical**
(empty `diff`). So items 1–4 below are exactly what this workspace runs.

---

## 1. `ProbabilityGridRangeDataInserter2D` — hits, misses, free space

### (a) Verbatim

`cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc` lines 52–96:

```cpp
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
```

`kSubpixelScale = 1000` (line 30). Same file, lines 124–133:

```cpp
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}
```

`cartographer/mapping/2d/probability_grid.cc` lines 58–71 (the write itself):

```cpp
bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell_index,
                                       const std::vector<uint16>& table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16* cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}
```

`cartographer/mapping/2d/grid_2d.cc` lines 99–106:

```cpp
void Grid2D::FinishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}
```

### (b) Answers

* **Write order, exactly.** `GrowAsNeeded` (grow grid to cover origin + all `returns` + all
  `misses`) → **all hits** (`returns`, each immediately) → **if `insert_free_space` only**:
  the free-space ray of *each return* (origin → that hit's cell) → then the free-space ray of
  each element of `range_data.misses` → `FinishUpdate()` in `Insert()`. Hits are *not*
  interleaved with misses; **every** hit is applied before **any** miss.
* **Which cells get the "miss/free" write for a return.** The ray is
  `RayToPixelMask(begin, end, kSubpixelScale)` with `begin`/`end` expressed in the
  *superscaled* grid (cell size `resolution/1000` = 50 µm for a 5 cm grid); `RayToPixelMask`
  returns every **full 5 cm cell** the straight segment from origin to endpoint passes through,
  and it is **inclusive of both ends** (it pushes `scaled_begin / subpixel_scale` first and
  terminates after appending the cell containing `scaled_end`; see
  `cartographer/mapping/internal/2d/ray_to_pixel_mask.cc` lines 34–156). So the endpoint (hit)
  cell **is visited** by the miss loop — but its write is a no-op: the hit table entry always
  has `kUpdateMarker` set (`*cell = table[*cell]`, and every table entry is `value + kUpdateMarker`),
  so the later `ApplyLookupTable(..., miss_table)` hits `if (*cell >= kUpdateMarker) return false;`.
  Net effect: free space is written to `[origin cell … endpoint cell]` with the endpoint cell
  surviving as a hit. There is no geometric exclusion of the endpoint; the exclusion is the
  single-write-per-cell rule plus hit-first ordering.
* **"Hits have priority" is scoped to one `Insert()` call.** `FinishUpdate()` strips
  `kUpdateMarker` from every touched cell at the end of each `Insert()`. Therefore a cell that
  is a hit in scan *k* **can and will** be decreased by a miss in scan *k+1* (or in any later
  `Insert()` on the same submap). There is no persistence/weighting protection across scans.
  This is the mechanism by which "occupied cells get erased".
* **`range_data.misses` and `missing_data_ray_length`.** `misses` is populated in
  `LocalTrajectoryBuilder2D::AddRangeData`
  (`cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc` lines 163–185):
  every accepted point whose range exceeds `options_.max_range()` is *moved* to
  `origin + missing_data_ray_length / range * delta` and appended to `misses`; everything with
  `min_range <= range <= max_range` goes to `returns`; `range < min_range` is dropped.
  `missing_data_ray_length` therefore controls **only** the length of the "assumed empty"
  segment for beams that returned *beyond `max_range`*. It does **not** affect the free-space
  rays derived from `returns` (those always reach all the way to the hit cell).
* **Can `misses` be non-empty?** Both sensor types end up as `TimedPointCloudData` and go
  through the same code, so the answer is decided by `max_range`, not by the sensor type:
  * **PointCloud2**: yes, routinely — `ToPointCloudWithIntensities(PointCloud2)` applies
    **no range filter at all**, so any XYZ farther than `max_range` becomes a miss.
  * **LaserScan**: only if some beam survives `msg_conversion` with
    `range > options_.max_range()`. For **this workspace it can never happen**: the scan comes
    from `pointcloud_to_laserscan` with `range_max: 10.0`, `use_inf: true`
    (`src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`), while
    `TRAJECTORY_BUILDER_2D.max_range = 12.0` (`src/rm_localization/cartographer_ros/configuration_files/cartographer.lua`).
    p2l emits no-echo bins as `inf` (or `range_max + inf_epsilon` if `use_inf: false`), and both
    fail `first_echo <= msg.range_max` in the conversion → dropped before cartographer ever
    sees them. **Consequence: `range_data.misses` is always empty for your `/scan`, and
    `missing_data_ray_length` (0.5 → 0.05) is a no-op in your setup.** The observed clearing is
    caused entirely by the *return-ray* loop above.
* **`insert_free_space`.** `false` returns from `CastRays` before **both** miss loops, so no
  free-space write happens at all — neither from return rays nor from `range_data.misses`.
  It does **not** disable hits. This matches the measured result in your config comments
  (`insert_free_space=false` → 89 % retention but free cells = 0): "erasing old walls" and
  "marking free space" are literally the same write (`miss_table` on cells along a ray), so the
  switch cannot separate them. It is set with `true` as the default when the Lua key is absent
  (`HasKey(...) ? GetBool(...) : true`), and `CHECK_GT(hit_probability, 0.5)`,
  `CHECK_LT(miss_probability, 0.5)` are enforced.
* Extra filters between the sensor and the grid: `TransformToGravityAlignedFrameAndFilter`
  applies `sensor::CropRangeData(..., min_z, max_z)` to **both** returns and misses and then
  `VoxelFilter(voxel_filter_size)` to **both** (`local_trajectory_builder_2d.cc` lines 52–63).
  With `num_accumulated_range_data = 1`, `AddRangeData` inserts one `RangeData` per scan.

---

## 2. `ProbabilityGrid` / `probability_values` — what is stored, and the lookup tables

### (a) Verbatim

`cartographer/mapping/probability_values.h` lines 64–92:

```cpp
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;
...
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16 kUpdateMarker = 1u << 15;
...
// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}
```

`cartographer/mapping/probability_values.cc` lines 76–104 (verification added: this is the
probability-space twin; `ProbabilityGrid` uses the *correspondence-cost* one below):

```cpp
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}
```

`cartographer/mapping/probability_values.h` lines 32–45 / 48–57 (`BoundedFloatToValue`,
`Odds`, `ProbabilityFromOdds`, `ProbabilityToCorrespondenceCost`);
`cartographer/mapping/2d/probability_grid.cc` lines 27–31 and 39–49:

```cpp
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}
...
void ProbabilityGrid::SetProbability(const Eigen::Array2i& cell_index,
                                     const float probability) {
  uint16& cell =
      (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}
```

`cartographer/mapping/2d/probability_grid.h` lines 43–46 (comment) and `probability_grid.cc`
lines 77–82:

```cpp
// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. ...
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
...
float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  if (!limits().Contains(cell_index)) return kMinProbability;
  return CorrespondenceCostToProbability(ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}
```

`cartographer/mapping/2d/grid_2d.h` lines 68–71:

```cpp
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
               kUnknownCorrespondenceValue;
  }
```

### (b) Answers

* **Occupied or free?** The public model is **P(cell is occupied)**. The storage, however, is
  the **correspondence cost = 1 − P(occupied)**, held as a `uint16` in
  `correspondence_cost_cells_`. `GetProbability()` converts back to P(occupied);
  `GetCorrespondenceCost()` is 1 − P(occupied). `SetProbability()` takes P(occupied).
* **Exact numbers:** `kMinProbability = 0.1f`, `kMaxProbability = 0.9f` (`1.f - 0.1f`),
  `kMinCorrespondenceCost = 0.1f`, `kMaxCorrespondenceCost = 0.9f`,
  `kUnknownProbabilityValue = 0`, `kUnknownCorrespondenceValue = 0`,
  `kUpdateMarker = 1u << 15` = **32768**. The uint16 encoding range is `[1, 32767]`
  (`BoundedFloatToValue` clamps to the bound and maps linearly onto 32766 steps, `+1`), and it
  is independent of the update marker: while updating, values carry `+32768`.
  `kValueCount = 32768` table entries.
* **Unknown cells read as 0.1, not 0.5.** `ValueToCorrespondenceCost(0) = kMaxCorrespondenceCost = 0.9`,
  so `GetProbability` of an unknown cell returns `kMinProbability = 0.1`. The real "is unknown"
  test is `IsKnown()` / stored value `== 0`, never a 0.5 comparison. (Only `SetProbability`
  refuses to write to a non-unknown cell.)
* **How a table works.** The inserter builds two tables *once*, in the constructor:
  `hit_table_ = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(hit_probability))` and
  `miss_table_ = ... Odds(miss_probability)`, where `Odds(p) = p/(1-p)`. Each table has 32768
  `uint16` entries and is indexed by the *current* cell value; every entry already includes
  `kUpdateMarker`, so applying it both updates the probability and stamps the cell as touched.
  * index `0` (unknown): the cell is set to `ProbabilityFromOdds(odds)` — i.e. the **first**
    observation sets P directly to `hit_probability` (or `miss_probability`).
  * index `v >= 1`: `odds_new = odds * (p_v/(1-p_v))`, `P_new = ProbabilityFromOdds(odds_new)`,
    re-encoded via `CorrespondenceCostToValue`, which clamps into `[0.1, 0.9]`.
  * `DCHECK_EQ(table.size(), kUpdateMarker)` documents the 32768 entry count.
* **Saturation.** A repeatedly hit cell converges to **P = kMaxProbability = 0.9** (stored
  uint16 = **1**, i.e. correspondence cost 0.1). A repeatedly missed cell converges to
  **P = kMinProbability = 0.1** (stored uint16 = **32767**, correspondence cost 0.9). The clamp
  happens per update, so the log-odds random walk saturates and never leaves `[0.1, 0.9]`.
* Per-update log-odds increments are `ln(hit_odds)` and `ln(miss_odds)`. With upstream defaults
  (`hit = 0.55`, `miss = 0.49`) that is `+0.2007` vs `−0.0400` (≈5 misses cancel 1 hit); with
  your config (`hit = 0.68`, `miss = 0.40`) it is `+0.7538` vs `−0.4055` (≈1.9 misses cancel
  1 hit). A cell can receive at most **one** update per `Insert()` (the marker blocks the rest),
  so the "votes" per scan are at most one hit *or* one miss per cell.

---

## 3. LaserScan → `RangeData` classification (ROS 2 fork, `ros2/cartographer_ros` tag 2.0.9002)

### (a) Verbatim

`cartographer_ros/src/sensor_bridge.cpp` lines 156–162:

```cpp
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 117–175 (the actual classification):

```cpp
// For sensor_msgs::msg::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }
...
// For sensor_msgs::msg::LaserScan and sensor_msgs::msg::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
        ...
      }
    }
    angle += msg.angle_increment;
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 204–214 (the `ToPointCloudWithIntensities`
overloads — identical body for LaserScan and MultiEchoLaserScan) and lines 216–218 + 255–298
(PointCloud2 overload: **no range check anywhere**). Then
`cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc` lines 163–185:

```cpp
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
```

### (b) Answers

There are **two independent filters** with different bounds — this is the usual source of
confusion:

1. message-level (`msg.range_min`/`msg.range_max`, applied in `msg_conversion.cpp`);
2. trajectory-builder-level (`TRAJECTORY_BUILDER_2D.min_range`/`max_range`, applied in
   `local_trajectory_builder_2d.cc`; yours are `0.2` / `12.0`).

* `range_min <= range <= range_max` (message) → kept as a point at
  `(range·cos θ, range·sin θ, 0)`; otherwise **silently skipped** — it becomes neither a hit
  nor a miss (no free-space ray is created).
* **`inf`**: `inf <= msg.range_max` is true only when `msg.range_max` is itself `inf`. With a
  finite `range_max` (yours: 10.0) every `inf` ray is **dropped**. (If a driver *did* publish
  `range_max = inf`, the point would survive as an infinite `range`, then hit
  `range > max_range` → `missing_data_ray_length/range * delta` = `5/inf * inf` = **NaN** —
  a genuine hazard worth guarding against.)
* **NaN**: both `range_min <= NaN` and `NaN <= range_max` are false → **dropped**.
* **`range > range_max` (message)**: dropped. **`range < range_min` (message)**: dropped.
  (Yours: `range_min = 0.2`, `range_max = 10.0`.)
* **tb-level**: `range < min_range` → dropped; `min_range <= range <= max_range` → `returns`
  (hit); `range > max_range` → `misses` with the point placed at **exactly
  `missing_data_ray_length` metres from the origin along the beam direction**
  (`origin + (missing_data_ray_length / range) * delta`, defaults `min_range = 0.`,
  `max_range = 30.`, `missing_data_ray_length = 5.` in
  `configuration_files/trajectory_builder_2d.lua`; yours are `0.2`, `12.`, `0.05`).
  So the miss distance is **`missing_data_ray_length`** — not `range_max`, not the measured range.
* **Consequences for this workspace**: `/scan` comes from `pointcloud_to_laserscan`
  (`range_max: 10.0`, `use_inf: true`, `range_min: 0.2`) and cartographer's
  `max_range = 12.0`. Every no-echo bin is `inf` > 10.0 → dropped at step 1; nothing can exceed
  12.0 → **`range_data.misses` is always empty**, and `missing_data_ray_length` has **no
  effect whatsoever**. Lowering it 0.5 → 0.05 changed nothing observable (as expected). The
  only free-space writes in your maps are the origin→hit rays of step 1 of item 1.
* `HandleLaserScan` additionally splits the cloud into `num_subdivisions_per_laser_scan`
  subdivisions (yours = 1), shifts per-point times so the last point is 0, and advances the
  scan timestamp by the last point's time; `HandleRangefinder` transforms the cloud into the
  tracking frame (origin = `sensor_to_tracking->translation()`) and calls
  `trajectory_builder_->AddSensorData(... TimedPointCloudData ...)`. `misses` is created later,
  in the local trajectory builder — never in `sensor_bridge`.
* `PointCloud2` takes the same trajectory-builder filter but **no message-level range filter**,
  so with `num_point_clouds = 1` points beyond 12 m *do* become misses (and `min_z`/`max_z` are
  applied relative to the tracking frame, to returns and misses alike).

---

## 4. `/map` (`nav_msgs/OccupancyGrid`) int8 values

### (a) Verbatim

`cartographer_ros/src/occupancy_grid_node_main.cpp` lines 182–191 (all it does):

```cpp
void Node::DrawAndPublish() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_->publish(*msg_ptr);
}
```

`cartographer_ros/src/msg_conversion.cpp` lines 374–419 (the actual mapping — note: **no named
constants exist**):

```cpp
std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const rclcpp::Time& time) {
  ...
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}
```

How `color`/`observed` are produced — `cartographer/io/submap_painter.cc` lines 195–214
(`kCairoFormat` = `CAIRO_FORMAT_ARGB32`, declared in `cartographer/io/image.h:33`):

```cpp
  for (size_t i = 0; i < intensity.size(); ++i) {
    // We use the red channel to track intensity information. The green
    // channel we use to track if a cell was ever observed.
    const uint8_t intensity_value = intensity.at(i);
    const uint8_t alpha_value = alpha.at(i);
    const uint8_t observed =
        (intensity_value == 0 && alpha_value == 0) ? 0 : 255;
    cairo_data->push_back((alpha_value << 24) | (intensity_value << 16) |
                          (observed << 8) | 0);
  }
```

`cartographer/io/submap_painter.cc` line 106 (the canvas is filled **dark red** first):
`cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);`

`cartographer/mapping/2d/probability_grid.cc` lines 116–135 (the per-cell encode; `value` is
what becomes the red channel):

```cpp
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }
    ...
    const int delta =
        128 - ProbabilityToLogOddsInteger(GetProbability(xy_index + offset));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }
```

### (b) Answers

* **Where it happens**: not in `occupancy_grid_node_main.cc`. That node subscribes to
  `SubmapList`, fetches each submap's compressed texture, renders it with
  `cartographer::io::DrawTexture`, and `PaintSubmapSlices` composites all slices onto a canvas
  pre-filled with `rgba(0.5, 0, 0, 1)`. `CreateOccupancyGridMsg` then reads the canvas pixels.
* **Exact rule (no named constants)**: `value = observed == 0 ? -1 : RoundToInt((1 - color/255)*100)`.
  Here `color` is the **red** channel (`packed >> 16` truncated to `unsigned char`), and
  `observed` is the **green** channel (`packed >> 8` truncated). The texture's red channel is
  the `value` byte from `DrawToSubmapTexture` (`= max(0, 128 − ProbabilityToLogOddsInteger(P))`),
  and green is `255` for any cell that was ever observed, `0` only for never-observed cells.
  Never-observed pixels keep the dark-red background (green = 0) → `-1`.
* **`-1` = unknown.** There is **no `0`/`100` constant pair any more**; the old binary
  `kUnknownOccupancyGridValue/kFreeOccupancyGridValue/kOccupiedOccupancyGridValue` scheme is not
  present in any branch I checked (`cartographer_ros` master, `release-1.0`, `ros2-dashing`,
  ROS2 fork `2.0.9002`) — all of them use the proportional formula above.
* **The mapping is proportional, but of log-odds-derived intensity, not of the probability, and
  it is capped well below 100.** Because of the cairo compositing (the texture mixes an
  intentionally "invalid premultiplied" intensity with `A = 0`, which cairo/pixman then *adds*
  to the dark-red background), I replicated the whole chain (`libcairo` + the exact packing and
  formula) to get the true effective table:

  | P(occupied) | published int8 | | P(occupied) | published int8 |
  |---|---|---|---|---|
  | unknown (never observed) | **−1** | | 0.55 | 52 |
  | 0.10 | **0** | | 0.60 | 55 |
  | 0.15 | 11 | | 0.65 | 57 |
  | 0.20 | 18 | | 0.70 | 60 |
  | 0.25 | 25 | | 0.75 | 62 |
  | 0.30 | 31 | | 0.80 | **65** |
  | 0.35 | 36 | | 0.85 | 69 |
  | 0.40 | 41 | | 0.90 | **75** ← maximum reachable |
  | 0.45 | 45 | | | |
  | 0.50 | 50 | | | |

  So: unknown = −1; "free" only reaches 0 when the cell is saturated at P = 0.1; the most
  occupied a cell can ever be published as is **75** (`P = kMaxProbability = 0.9`). The
  `CHECK_GE(100, value)` confirms the author expected ≤ 100, but the compositing caps it at 75.
  A cell at exactly P = 0.5 encodes `(value = 0, alpha = 1)` — the `(value || alpha) ? alpha : 1`
  clause exists precisely so a *known* mid-probability cell is not encoded as `(0,0)` (= unknown).
* **Direct implications for this workspace's debugging** (all verified against your config:
  `hit_probability = 0.68`, `miss_probability = 0.40`, `insert_free_space = true`):
  * one hit on an unknown cell → P = 0.68 → published **58**; two hits → 0.8187 → **67**;
    three hits → 0.9 → **75**.
  * a saturated wall (P = 0.9) decays with each miss update to P = 0.857, 0.8, 0.727, 0.64, …
    → published 75, **70, 65, 61, 56**, 52, 44, 35, 26, 17, 8, 0. **Three miss updates drop a
    saturated wall below 65.**
  * a wall observed only once (P = 0.68 → 58) falls to 54 → 49 with two misses.
  * `tools/analyze_slam_bag.py` classifies `occ = G >= 65`, i.e. **P ≥ 0.80**. So a wall needs
    two consecutive hit updates to count as "occupied" at all, and only ~2 miss updates to stop
    counting. Any consumer that assumes `100 == occupied` (e.g. a nav2 threshold of exactly 100)
    will never see an obstacle from this publisher.
  * combined with item 3: in your `/scan` setup `missing_data_ray_length` is inert, so the miss
    writes that cause the decay come exclusively from the origin→hit rays (item 1, "Now add the
    misses" loop) — i.e. from beams that *did* return, whose rays pass through a cell that was
    hit at an earlier/later scan.

---

## 5. 合并附录：上游 issue 调查、TSDF、替代方案与调参（原 `cartographer_map_stability_report.md`）

> **本节是 2026-10 仓库整理时并入的**：原文件 `cartographer_map_stability_report.md` 在仓库根、**未跟踪**（455 行，2026-09-23，英文），与本文是同一批源码取证的两种写法。整理时**只把本文 §1–§4 没有的证据**搬进来（§5.1 列了重复项的指针），随后删除该未跟踪文件；原文的源码级结论仍以本文 §1–§4 为准（那里是逐字引文）。

### 5.0 来源与证据分级（provenance）

| 项 | 值 |
|---|---|
| 原文件 | `cartographer_map_stability_report.md`（仓库根，**untracked**，455 行，最后修改 2026-09-23；原标题 *"Cartographer 2D map-stability research report"*，版本依据 rosdistro humble：<https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml>） |
| 原文覆盖 | ROS 2 Humble + `ros-humble-cartographer 2.0.9004`（`ros2/cartographer` 分支 `ros2`）；Livox MID-360 → 地面剔除 → `pointcloud_to_laserscan` 窄高度带 → cartographer 2D；症状 = `/map` 里占据墙被擦掉/闪烁（保留率 ≈40%），`insert_free_space=false` → 89% 但无 free space |
| 合并口径 | **只并入本文没有的**证据；重复部分见 §5.1。原文的 `[SRC]`（上游源码/官方文档已核）本文记 **【引用】**；`[USER]`（GitHub issue / 工程博客的真实用户报告）记 **【引用·用户报告】**；`[SPEC]`（原报告作者的推断）记 **【推断】** |
| ⚠️ 性质 | 本附录是**外部仓库/issue 调研**，**不是本仓实测**。本仓的实测数字只有本文 §4(b) 末尾那几条（`hit 0.68 / miss 0.40` → 75/58/…）与 `cartographer.lua` 的当前取值；本附录的"推荐值"一律**没有**在本仓 A/B 过 |

### 5.1 与正文重复、因此不再重抄的部分（原文 §0 的主体）

| 原报告结论 | 本文已有位置 |
|---|---|
| 2D 没有"占据过就永不清除"的规则/开关；`CastRays` 先 hit、再 `insert_free_space` 才走 miss 射线 | §1(a)(b) |
| `insert_free_space=false` 是**早退** ⇒ 89% 保留但 free space = 0 | §1(b) «`insert_free_space`» |
| 每格 log-odds 更新、`kMinProbability/kMaxProbability = 0.1/0.9`、unknown 读作 0.1、`kUpdateMarker` | §2(b) |
| `missing_data_ray_length` 只缩短"无回波/超 `max_range`"的射线，**救不了**越顶光束（原文另引 issue #354 "missing_data_ray_length has no effect?"） | §1(b)、§3(b) |
| `/map` 是子图纹理混合、最大值 **75**（不是 100）、`CreateOccupancyGridMsg` 的取整公式 | §4(b) |
| 本仓 `/scan` 的 `range_data.misses` **恒空**、`missing_data_ray_length` 在本仓**无效** | §3(b) |

### 5.2 量化推论（原文 §0 的算术，正文未逐条列）

【引用】默认 `hit 0.55 / miss 0.49` 下：单次 hit 的 log-odds `ln(0.55/0.45) = +0.2007`、单次 miss `ln(0.49/0.51) = −0.0400` ⇒ **1 次 hit 恰好抵 5.02 次 miss**；从 unknown 到饱和需 **11 次 hit** 或 **55 次 miss**；clamp 到 `±2.1972`（`ln 9`）后，**已饱和的墙仍需 55 次 miss** 才会掉到"free"。

【引用·用户报告】`#1818` 的修复者把 `hit_probability` 提到 **0.75**（`miss_probability` 留 0.49）：hit odds 变 `+1.0986`、miss 仍 `−0.0400` ⇒ 比值变 **1 hit : 27.5 miss**；"被打过 3 次的墙"从 **15 次 miss** 变成 **82 次 miss** 才会掉到 P<0.5（本附录复算：`ln(3³)/0.04 = 82.4`）。

【推断】更激进的 `miss_probability = 0.499`：`ln(0.499/0.501) = −0.004` ⇒ 饱和墙要 **549 次 miss**、unknown 到 free 也要约 **549 帧 ≈ 55 s @10 Hz**——在小场地上可能"free space 出得太慢"，只在 0.49 不够时才用。

### 5.3 越顶光束（over-passing）：本文几何下的根因

【推断】一束从 **0.226 m** 高处出射、**越过 0.40 m 高的挡板**、打中远处墙（距离 R）的光束，在 2D 里与"穿过自由空间"**不可区分**：inserter 会把挡板所在的格（距离 `d << R`）当 free-space 擦掉，把远墙记为 hit。2D 管线里**没有任何东西知道"越顶"这件事**。

【引用·用户报告】MichaelGrupp（cartographer_ros #1457）对动态物体说的是同一机制：*"Dynamic objects are not explicitly modeled. But … a dynamic object can 'disappear' in the grid cell when later observations don't see an obstacle anymore at this point."*

【推断】**MID-360 让这件事更糟**：它是非重复扫描，方位/俯仰图案每帧都不同。对同一个方位角，带内最近的光束俯仰逐帧变化 ⇒ A 帧那条束打在挡板上（hit），B 帧稍高的一条束越过去、带内最近点变成远墙（hit ⇒ 擦挡板）⇒ **同一格上逐帧随机的 hit/miss 交替**，正是 5:1 的 log-odds 顶不住的图案。原文明确注明：**这是从 p2l 代码 + MID-360 规格推出来的，没有找到哪条 issue 明说**。

### 5.4 上游 issue 调查（原文 §1，本文正文没有）

| # | 结论 | 来源 |
|---|---|---|
| **1818** | 【引用·用户报告】**与我们的症状最像**："When I walk in one direction the tables are mapped as obstacles, however when i turn around … the tables disappear." 第二位用户（`marmik18`, 2024-11-12）同样复现，并**把 `hit_probability` 提到 0.75 + `miss_probability=0.49` 修好** | <https://github.com/cartographer-project/cartographer_ros/issues/1818> |
| **1338** | 【引用·用户报告】"Wall disappears in 2D SLAM"。MichaelGrupp：*"The occupancy grid node … just blends all submaps into an image, so it could be that it just **looks** as if the wall is gone. The submaps plugin … is better for checking."* 报告者随后发现墙只在 submap 0 里完整，且**把 `max_range` 降到 ~5 m 能缓解**（他想要 20 m）；另有人建议按**反射强度**过滤 | #1338 |
| **1678** | 【引用·用户报告】**`overlapping_submaps_trimmer_2d` 本身会擦数据**："sometimes, the trimming process will decrease the certainty of obstacles (**e.g., a wall may become free space**) or completely 'forget' parts of the map."；参数 `fresh_submaps_count=1, min_covered_area=2, min_added_submaps_count=5`；结论是内存/性能折衷 ⇒ 更保守或干脆别长期跑 SLAM | #1678 |
| **689** | 【引用·用户报告】"Map boundaries disappear over time" —— 被维护者以"RViz 对旧子图的**淡出显示**是 intended"关闭（PR #674）。⇒ 先确认看到的是 *Submaps* 显示的 fade，不是 `/map` | #689 |
| **1409** | 【引用·用户报告】反向问题（open）：障碍离开后 free space **从不**被改回来 —— 即没有可用的旋钮 | #1409 |
| **1845** | 【引用·用户报告】默认参数下 free space 要 ~5 分钟才在 `map.data` 里变成 `0`（open）；同机制的另一方向 | #1845 |
| **1538** | 【引用·用户报告】"How to tune the disappearing time of dynamic objects?"（open） | #1538 |
| **1457** | 【引用·用户报告】维护者对"格子为什么会被取消占据"的解释（§5.3 引的那段） | #1457 |

【引用】原文作者的**穷举检索**：在 `cartographer-project/cartographer{,_ros}` 里搜 `"free space"`、`wall disappear`、`map disappears`、`obstacle disappear`、`erase occupied`、`insert_free_space`（分别 28 / 23 条命中），**只有上面这些 issue**；ROS Answers / Discourse 没有额外信息；**没有任何维护者宣布的"修法"**。

### 5.5 从 3D 雷达跑 2D：`pointcloud_to_laserscan` 的语义与三条管线

#### 5.5.1 p2l 到底做了什么

【引用】`min_height`/`max_height` 是在 **`target_frame`** 里切的；然后每个点算
`index = (atan2(y,x) - angle_min)/angle_increment`，并且 **`if (range < ranges[index]) ranges[index] = range;`** ⇒ **带内最近的回波赢**。它**没有**逐束俯仰记账、**没有**"光束越过了什么"的概念。
`range_max` 也有影响：带内没有点的那一束发 `inf`（`use_inf=false` 时发 `range_max+1`），cartographer 再把 `> max_range` 的变成 `missing_data_ray_length` 的 miss 射线。

#### 5.5.2 三条管线

**选项 A（原生，不用 `pointcloud_to_laserscan`）：直接把 PointCloud2 喂给 cartographer 2D**（【引用】）

```lua
num_point_clouds = 1,          -- subscribe to PointCloud2 instead of LaserScan
num_laser_scans = 0,
TRAJECTORY_BUILDER_2D.min_z = -0.05   -- in tracking_frame, gravity aligned
TRAJECTORY_BUILDER_2D.max_z = 0.15    -- tight band around the 0.226 m scan plane
TRAJECTORY_BUILDER_2D.min_range = 0.3 -- kills ground ring + robot body
```

【引用】实现是 `local_trajectory_builder_2d.cc` 里的 `sensor::CropRangeData(sensor::TransformRangeData(...), min_z, max_z)`（另见 `range_data.cc`）。
【推断】相对 p2l 的好处：切带发生在**重力对齐之后** ⇒ 车体俯仰不会带着带子跑，也没有"整个 360° 里最近点赢"的 1°/束量化损失；但它**仍是 2D 投影**，越顶擦除照样存在。

**选项 B（已验证的真实配置：16 线 RoboSense RS16 → 2D）**（【引用·用户报告】：中文实践者的 `rs16_2d_outdoor.lua`，逐参数解释）

```lua
num_point_clouds = 1
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 100.
TRAJECTORY_BUILDER_2D.min_z = 0.2          -- "multi-line: prevents ground points polluting the map"
TRAJECTORY_BUILDER_2D.max_z = 1.4
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
POSE_GRAPH.optimize_every_n_nodes = 160.      -- ">= 2 x num_range_data"
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
```

【引用·用户报告】同一份配置还用 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`、`use_pose_extrapolator = false`；其调参规则：*"loop-closure thresholds — if unstable with false matches, raise these two; in a highly repetitive scene, lower them or turn loop closure off"*，并明确报告**重复走廊里的假回环会把图搞坏**。来源：<https://blog.csdn.net/xjblalala/article/details/125049642>。

**选项 C（本仓现状）：保留 `pointcloud_to_laserscan`，但把带子收紧到传感器高度附近并夹住 `range_max`**（【引用】原报告的示例值）

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_link
    min_height: -0.05     # metres, relative to base_link
    max_height: 0.10      # keep the top of the band well below the 0.4 m barrier top at range
    angle_increment: 0.00436   # 0.25 deg
    range_min: 0.3
    range_max: 8.0
    use_inf: true
```

【推断】几何：传感器 0.226 m、挡板顶 0.40 m 时，向上的光束只有**仰角 < `atan((0.40−0.226)/d)`** 才能越过挡板 ⇒ **2 m 处 4.9°、5 m 处 2.0°**。5 m 处 ±0.1 m 的带子 ≈ **±1.15°** ⇒ "紧、略低于中心"的带子 + 小 `range_max` 是扫描生成侧**唯一**的几何缓解手段。

#### 5.5.3 别人报的 free-space/越顶问题

【引用】cartographer issue 跟踪器里**没有**"over-passing"这个词；报的症状与我们相同（**视角一变障碍就消失**，#1818），维护者的回答是"取消占据是概率网格的设计"（#1457）。VLP-16 + cartographer 2D 可看 #725（无 IMU / TF 问题，用 `use_online_correlative_scan_matching` + TF 解决，而不是地图插入选项）。Livox MID-360 + nav2 在 ROS 2 有现成仓库，但是 **LIO + 重定位 + Nav2**，不是 cartographer-2D：<https://github.com/Ikunio/Lidar_nav2_ws>。

### 5.6 TSDF 子图模式：能不能解决？（结论：不能，别切）

【引用】原报告判决：**Humble 里 TSDF 是接通且能发 `/map` 的，但它不解决本问题，还带一个未修的漂移 bug。不要为这个切过去。**

**可用性**（【引用】，`ros2/cartographer` / `ros2/cartographer_ros` Humble 2.0.9004/2.0.9002）：

* `TSDF2D::DrawToSubmapTexture(...)` **存在**（`tsdf_2d.cc`）；
* `Submap2D::ToResponseProto()` 无条件调 `grid()->DrawToSubmapTexture(...)`（`submap_2d.cc`）；
* `occupancy_grid_node` 照旧经 `SubmapQuery` 取子图纹理并发 `nav_msgs/OccupancyGrid`；
* `CreateOccupancyGridMsg` 读同一套 premultiplied RGBA ⇒ TSDF 的 free/occupied/unknown 确实映到 `0 / ~75 / −1`，**没有 CHECK 或代码路径拒绝 TSDF**；
* `map_builder_test.cc` 把 `GlobalSlam2D` 与 `LocalizationOnFrozenTrajectory2D` 参数化到 `GridType::TSDF` ⇒ 上游**意图**是两者都能跑；
* `RealTimeCorrelativeScanMatcher2D`、`CeresScanMatcher2D` 都按 `grid.GetGridType()` 分支处理 TSDF，约束构建器对任意 `Grid2D` 建 `FastCorrelativeScanMatcher2D`。

**选项块**（【引用】，`ros2` fork 的 `trajectory_builder_2d.lua` 与上游相同）：

```lua
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter = {
  truncation_distance = 0.3,
  maximum_weight = 10.,
  update_free_space = false,          -- NOTE: default is false, and it is a *required* key (GetBool, no HasKey)
  normal_estimation_options = { num_normal_samples = 4, sample_radius = 0.5 },
  project_sdf_distance_to_scan_normal = true,
  update_weight_range_exponent = 0,
  update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
  update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
}
```

* 【引用】`update_free_space = false`（默认）⇒ **就是"没有 free space"那一档**：射线掩膜从 `origin + (1 - truncation/range)*ray` 开始，只碰 hit 前 `truncation_distance` 内的格子（`tsdf_range_data_inserter_2d.cc`：`const Eigen::Vector2f ray_begin = options_.update_free_space() ? origin : origin + (1.0f - truncation_ratio) * ray;`），与 `insert_free_space=false` 同结果。
* 【引用】`update_free_space = true` ⇒ **并不会**保护"射线沿途的障碍"：射线仍从原点出发，每格 `update_tsd = range - dist(cell,origin)` 夹到 `[−truncation, +truncation]`；只有**距 hit < 0.3 m（默认）**的格落在"表面带"，更前面的格全被夹到 `−truncation`，纹理把 `|TSD| = truncation` 渲染成 **free**。⇒ 3 m 外的一道矮挡板**照样被擦**。保护是"hit 周围 ±0.3 m 的圆盘"，不是"沿途每个障碍"。
* 【引用】权重：是**带权滑动平均**，每帧权重 `GaussianKernel(update_tsd, 0.5)`（表面 ≈0.80、夹紧带边缘 ≈0.67）⇒ 直射 hit 只比越顶 miss 高 **1.2×**；`maximum_weight` 只限制**存储**权重，**不阻止**均值移动（`updated_sdf` 用的是夹紧前的权重）。
* 【引用】纹理解码：`normalized_tsdf = pow(|TSD|/maxTSD, 0.5)`、`delta = weight*(normalized_tsdf*255-128)` ⇒ `+truncation` 与 `−truncation` **都**渲染成 free，只有 `|TSD| ≈ 0` 才渲染成 occupied。
* 【引用】**fast correlative scan matcher 对 TSDF 打分是错的**：`PrecomputationGrid2D` 用 `1 - |grid.GetCorrespondenceCost(cell)|` 建，TSDF 的 free 格 correspondence cost = 0.1 ⇒ 得分 **0.9**，即**自由空间拿到最高分**（与 cartographer#1625 "Why the score of unknown area is highest?" 同款）。
* 【引用】**官方没文档**：生成的配置参考里完全没有 TSDF 项，算法 walkthrough 以 `**TODO**: document TSDF configuration` 结尾。

**真实用户反馈（全是坏消息）**（【引用·用户报告】）

| # | 报告 | 来源 |
|---|---|---|
| 1518 | **open**：在**官方 bag** 上开 TSDF 就漂："I observed this drifting of map. It seems like some pose transformations are applied with wrong time stamp." | cartographer#1518 |
| 1648 | **open**：Gazebo 里 `grid_type="TSDF"`，"the state of yaw expressed in map coordinate rotates constantly as time goes on even though the robot is at stationary … the map becomes corrupted as time passes." | cartographer_ros#1648 |
| 1058 | closed，真实用户：默认参数下扫描相对（子）图 **"slide around"** 得厉害 | cartographer_ros#1058 |
| 1634 / 1842 | TSDF 位姿/优化不稳、初值漂移、yaw 不固定 | #1634 / #1842 |
| 1709 / 1632 / 1713 | "no matter how I configure Lua, I can't get the ideal result"；"Are there any reference … TSDF lua file?"（无人答）；仿真 bag 上调 TSDF | #1709 / #1632 / #1713 |
| 1466 | **open**：CI 里 `MapBuilderTestByGridType.GlobalSlam2D/1`、`LocalizationOnFrozenTrajectory2D/1`（正是 TSDF 参数化）失败，加上 TSDF inserter 测试不稳定（#1514） | #1466 |

【引用】原报告结论：**没有找到任何人**用 cartographer 2D TSDF 做出稳定 2D 占据图的报告。

### 5.7 其它旋钮：别人实际改成什么、换来什么（原文 §4）

【引用】为什么没有"永不清除占据"的开关：`probability_grid_range_data_inserter_options_2d.proto` **只有三个字段** `hit_probability`、`miss_probability`、`insert_free_space`；3D 侧有 `TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels`（"0 disables free space"），**2D 的对应物只有这个布尔**。原文 §6 还给了"今天可配置的保守 free-space 更新"四点清单：①提高 hit/miss 不对称 ②缩短擦除射线（`max_range`/`missing_data_ray_length`/`min_range`/窄 `min_z,max_z`）③提高每格 hit 次数（`num_accumulated_range_data`、`motion_filter`、更小的 `num_range_data`）④从 cartographer 外部供 free space（nav2 `track_unknown_space: false`）或换 slam_toolbox 的 hit-ratio 规则。**落地顺序见 §5.9。**

| 旋钮 | 用过的值 | 效果 / 来源 |
|---|---|---|
| `…probability_grid_range_data_inserter.hit_probability` | **0.75** | 【引用·用户报告】#1818 报告者"playing around with these 2 values"修好桌子/障碍消失 |
| `…miss_probability` | **0.49**（同一报告） | 【引用】比值变 1 hit : 27.5 miss；3-hit 的墙要 **82** 次 miss（而不是 15） |
| 同一对，更激进 | `miss_probability = 0.499` | 【推断】`ln odds −0.004` ⇒ 549 次 miss 才能擦掉饱和墙；free space 约 55 s 才饱和（小场地可能太慢） |
| `TRAJECTORY_BUILDER_2D.max_range` | **≈5 m** | 【引用·用户报告】#1338："setting max_range to around 5m reduces the problem"（他想要 20 m）。注意 `> max_range` 的回波会变 `missing_data_ray_length` miss 射线 ⇒ 减少长距离掠射束 |
| `TRAJECTORY_BUILDER_2D.missing_data_ray_length` | 默认 5 m；建议 **1–2 m** | 【推断】只缩短"无回波"射线；无害，减少推测性擦除。⚠️ 本仓该值**无效**（§3(b)） |
| `TRAJECTORY_BUILDER_2D.min_range` | 0.3 | 【引用·用户报告·CSDN RS16】去掉地面环/车体 ⇒ 少假 hit |
| `TRAJECTORY_BUILDER_2D.min_z/max_z` | 0.2/1.4（室外车）、`-0.8/2.0` 默认 | 【引用·用户报告·CSDN】"multi-line: prevents ground points polluting the map" |
| `TRAJECTORY_BUILDER_2D.voxel_filter_size` | 0.02–0.025 | 【引用·用户报告·CSDN】密度 vs CPU |
| `TRAJECTORY_BUILDER_2D.num_accumulated_range_data` | **10** | 【引用】官方 `backpack_2d.lua` 与 unicorn-racing 定位 lua（"accumulate scans (reduce noise)"） |
| `…motion_filter.max_distance_meters / max_angle_radians` | 0.1 / 0.004 | 【引用·用户报告·CSDN RS16】更密的插入（默认 0.2 / 1°） |
| `…submaps.num_range_data` | **80** | 【引用·用户报告·CSDN RS16】；经验式 `optimize_every_n_nodes ≈ 2 × num_range_data = 160` |
| `…ceres_scan_matcher.occupied_space_weight` | **10**（translation/rotation 各 1） | 【引用·用户报告·CSDN RS16】更信 scan/map 匹配。**但**【引用】它是 **Ceres 扫描匹配的 cost weight**，**不参与栅格更新** ⇒ 单靠它不能阻止擦除，只能避免"配准错了看起来像墙被擦"（调高只让位姿更贴图；人们常用 10–20、默认 1） |
| `POSE_GRAPH.constraint_builder.min_score` / `global_localization_min_score` | 0.48 / 0.60（`pose_graph.lua` 里 0.55/0.6） | 【引用·用户报告·CSDN】匹配不稳/错匹配就调高；**重复场景里假回环会把图搞坏** ⇒ 那里建议调低或关回环 |
| `POSE_GRAPH.optimize_every_n_nodes` | 160（建图）、**2**（定位）、**0**（诊断时关全局 SLAM） | 【引用】unicorn-racing；官方文档："Setting `optimize_every_n_nodes` to 0 is a convenient way of disabling global SLAM" |
| `POSE_GRAPH.overlapping_submaps_trimmer_2d` | `{fresh_submaps_count=1, min_covered_area=2, min_added_submaps_count=5}` | 【引用·用户报告】**不要为地图稳定性开它**：会把墙变 free、把图"忘掉"（#1678）；`pose_graph.lua` 默认注释掉 |
| `TRAJECTORY_BUILDER.pure_localization` + `pure_localization_trimmer.max_submaps_to_keep` | `= 3` | 【引用】冻结的先验子图**永不被改** ⇒ 先验图的 `/map` 完全稳定；滚动窗口只加 3 个局部子图 |
| `occupancy_grid_node` gflags | `-include_frozen_submaps`、`-include_unfrozen_submaps`、`-resolution`、`-publish_period_sec`、`-occupancy_grid_topic` | 【引用】用来判定"擦除发生在冻结子图还是活跃子图" |

【引用】**最便宜的决定性诊断**：用 cartographer RViz 的 *Submaps* 插件逐个看子图（**关掉 fade**），和/或让 occupancy grid node 跑 `-include_unfrozen_submaps=false`。若墙在冻结子图里还在 ⇒ 只有当前活跃子图被擦、闪烁是**混合伪影**；若子图里就没了 ⇒ 是**真的栅格更新**。

### 5.8 替代方案

#### 5.8.1 slam_toolbox：**没有** `prob_hit`/`prob_miss`，但更新规则更保守（原文 §5a）

【引用】**对"slam_toolbox 也有 prob_hit/prob_miss"这个前提的更正**：`grep -r "prob_hit"` 在 ros2 分支**无命中**。它的占据栅格由 `karto::OccupancyGrid` 建，**每格两个单调计数器**、按比值定占据：

```cpp
// karto_sdk/include/karto_sdk/Karto.h
virtual void UpdateCell(kt_int8u * pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt) {
  if (cellPassCnt > m_pMinPassThrough->GetValue()) {          // default 2
    kt_double hitRatio = cellHitCnt / (kt_double)cellPassCnt;
    if (hitRatio > m_pOccupancyThreshold->GetValue())         // default 0.1
      *pCell = GridStates_Occupied;
    else
      *pCell = GridStates_Free;
  }
}
```

* 【引用】每条穿过某格的束让 `passCnt` +1；每条**终止**在该格的束同时让 `hitCnt` +1（`RayTrace()`）；**两个计数器只增不减、不衰减**。
* ⇒ 一个被 hit 过 H 次的格子，要再积累约 **9×H 次 pass-through** 才会被擦（`H/(H+M) ≤ 0.1`）——是**整个运行期累计**，不是 cartographer 那种"十几次 miss"。
* 【引用】发布的地图在每次 `updateMap()` 时由**位姿图里的全部扫描确定性重算**：`karto::OccupancyGrid::CreateFromScans(mapper->GetAllProcessedScans(), resolution, min_pass_through, occupancy_threshold)`（`slam_mapper.cpp`）。⇒ 不会出现 cartographer 增量 log-odds 那种"逐帧闪"。

【引用】slam_toolbox 的稳定配方（参数名/默认值已核；**参数值是【推断】的保守化，不是社区验证的预设**）：

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    resolution: 0.05
    min_pass_through: 2          # default 2; 1 makes free space appear sooner
    occupancy_threshold: 0.05    # default 0.1 -> walls stickier (erase needs ~19x hits instead of ~9x)
    # scan admission: fewer, better-separated scans (map is rebuilt from graph scans only)
    minimum_time_interval: 0.5   # default
    minimum_travel_distance: 0.3 # default 0.5
    minimum_travel_heading: 0.3  # default 0.5
    # save the pose graph periodically / use serialization so a bad closure can be undone
    scan_buffer_size: 10
    loop_search_maximum_distance: 3.0
    correlation_search_space_dimension: 0.5
    correlation_search_space_smear_deviation: 0.1
```

* 【推断】代价：`occupancy_threshold` 调低也会让**偶发假 hit** 更容易被判 occupied。原报告明确写：**没有找到"低 3D 雷达 + 0.4 m 挡板 + 需要 free space"这个场景的公认配置**，上面只是"保守 free-space 更新"的源码等价物。
* 【推断】越顶的**几何问题在 slam_toolbox 里照样存在**（喂的是同一张 2D 扫描）；变的是更新规则——把"墙在 15 次 miss 后被擦"变成"墙在 9×hit 次 pass-through 后才被擦"。

#### 5.8.2 nav2 `static_layer` + `track_unknown_space: false`：让 `insert_free_space=false` 可用（原文 §5b）

【引用】`nav2_costmap_2d/plugins/static_layer.cpp` 的 `interpretValue`：

```cpp
unsigned char StaticLayer::interpretValue(unsigned char value) {
  if (track_unknown_space_ && value == unknown_cost_value_)       return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_) return FREE_SPACE;
  else if (value >= lethal_threshold_)                            return LETHAL_OBSTACLE;
  else if (trinary_costmap_)                                      return FREE_SPACE;
  double scale = value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}
```

* 【引用】`track_unknown_space` **默认就是 `false`**、`unknown_cost_value` 默认 `255 (=0xff)`，正好是 cartographer 的 `-1` 经 int8→uint8 转换后的值 ⇒ **cartographer 的 unknown 变成 FREE_SPACE**。
* 【引用】**坑**：`lethal_cost_threshold` 默认 **100**、`trinary_costmap` 默认 **true**；而 cartographer 的占据上限只有 **≈75**（本文 §4(b)）⇒ **用 nav2 默认值时，每一面 cartographer 墙都会变成 FREE_SPACE**。必须把 `lethal_cost_threshold` 设成 **60**（#628 的用户就按 60 卡）或更低。
* 【引用】对应 yaml：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: false     # unknown (-1) -> free
      lethal_cost_threshold: 60      # cartographer walls are ~57..90, never 100
      trinary_costmap: true
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true
```

* 【引用】原报告的判断：这是"字面问题最干净的纯配置答案"——cartographer 用 `insert_free_space=false`（墙稳定在 89%），让图 = 墙 + unknown，costmap 把 unknown 当 free。代价是失去"从未见过"与"已知 free"的区分（封闭场地通常可接受）。

#### 5.8.3 hector_slam（原文 §5c）：**未验证**

【引用】原报告**没能验证**（访问 hector_slam 仓库时网络中断）：`hector_mapping` 一般被认为暴露 `update_factor_free`/`update_factor_occupied`（log-odds 更新因子）与 `map_update_distance_thresh`/`map_update_angle_thresh`；**无法打开** `hector_mapping/include/hector_mapping/map/OccupancyGridMap.h` 确认擦除规则。**按未验证对待。**

【推断】定性上：hector_slam 没有回环 ⇒ 子图永不被重优化，消除了"优化后地图重排/闪"这一失效模式，但对越顶擦除**没有任何帮助**。

### 5.9 原文的建议动作顺序（原文 §7，【引用】值全部来自上面各来源）

```lua
-- 1. make hits dominate misses (biggest single lever; user-verified in #1818)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.75
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- 2. limit how much free-space carving each scan can do
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 12.       -- larger than the arena; try 6-8 if walls still get eaten
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.

-- 3. keep the height band tight & gravity-aligned (drop pointcloud_to_laserscan, feed the cloud directly)
num_laser_scans = 0
num_point_clouds = 1
TRAJECTORY_BUILDER_2D.min_z = -0.06
TRAJECTORY_BUILDER_2D.max_z =  0.12

-- 4. more hits per insertion, fewer redundant insertions
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- 5. make sure nothing else deletes data
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil   -- keep it commented out (default)
POSE_GRAPH.optimize_every_n_nodes = 2 * TRAJECTORY_BUILDER_2D.submaps.num_range_data   -- e.g. 180 for num_range_data=90
-- if the arena is repetitive: raise POSE_GRAPH.constraint_builder.min_score (0.55 -> 0.65) and
-- global_localization_min_score, or set optimize_every_n_nodes = 0 while diagnosing
```

【引用】nav2 侧：`track_unknown_space: false`、`lethal_cost_threshold: 60`、`trinary_costmap: true`。
【引用】若做完这些保留率仍不可接受：换 slam_toolbox（`occupancy_threshold: 0.05`），或把图冻结（cartographer `pure_localization` + `pure_localization_trimmer.max_submaps_to_keep = 3`）。**不要**切 `grid_type = "TSDF"`。

### 5.10 与本仓现状的差异（【推断】，别把附录当成本仓实测）

1. **本仓 `cartographer.lua` 是 `hit 0.68 / miss 0.40`**（本文 §2(b)/§4(b)），不是附录推荐的 0.75/0.49；0.75/0.49 是**外部用户**在 #1818 修好的值 ⇒ 想采纳必须在本仓做 A/B（`tilted_lidar_fidelity.md`/`debug_fastlio_cartographer.md` 的方法）。
2. 本仓这份 `src/rm_localization/cartographer/` 是 **fork**，多了 `min_probability_to_clear 0.80` 这类扩展（`docs/mapping/README.md` 有记）；**附录完全没有覆盖这个旋钮**，它与 `insert_free_space` 的交互**未验证**。
3. 本仓 `/scan` 的 `range_max: 10.0` / `max_range: 12.0`（本文 §3(b)）与附录假设一致 ⇒ §5.1 那几条"misses 恒空"的结论对附录同样成立。
4. 附录的"保留率 ≈40% / 89%"是 **2026-09-23 那次会话**的症状数字，本附录**不把它当本仓现行实测**；本文 §4(b) 的 75/58/… 才是可复算的本仓数字。

### 5.11 未验证 / 缺口（原报告自己声明的 + 本附录的）

* **hector_slam 源码没打开**（网络失败）⇒ §5.8.3 全部按"未验证"对待。
* **MID-360 逐帧随机的 hit/miss 交替是【推断】**（p2l 代码 + 非重复扫描规格），**没有** issue 明说。
* 所有 `[USER]` 结论都是**别人的机器/别人的配置**，本仓**没有复现**；参数值只能当"候选"，不能当"结论"。
* TSDF 的用户报告是 issue 文本，**未在本仓复现**；TSDF 的 scan-matcher 打分 bug 是源码推断（【引用】公式 + 代码路径），未跑实验验证。
* 附录**没有**覆盖本仓 fork 的扩展旋钮（`min_probability_to_clear` 等），也没有覆盖 `mode:=mapping` 下 `map→odom` 的静态语义（那属于 `docs/mapping/` 与 `debug_fastlio_cartographer.md` 的范围）。
