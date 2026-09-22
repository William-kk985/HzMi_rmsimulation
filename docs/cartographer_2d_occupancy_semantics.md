# Cartographer 2D occupancy-grid semantics — verbatim source analysis

Prepared for debugging "occupied cells keep getting erased" in this workspace.
Every excerpt below was copied verbatim from the real sources (fetched with web tools /
read from the vendored checkout). Line numbers are those of the quoted revision.

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
