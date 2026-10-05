# 局部代价地图也上 STVL（`local_obstacle:=stvl`）—— 实现 + A/B 实测

> 状态：**已实现 + 已实测（2026-10-05）**；结论见 §4。
> 一句话：给 `local_obstacle` 增加 `stvl` / `stvl_both` 两个取值，让**局部**代价地图也能跑
> SpatioTemporalVoxelLayer（COD 2026 的双图做法），然后用同栈同目标点 A/B 量化它的收益与代价。
> 相关文档：`docs/algorithm_matrix.md` §一（槽位表）/§八.4（实验清单 P1）、
> `docs/cod_nav_2026_deep_dive.md` §A8 + §4-C2、`docs/cod_nav_2026_params_review.md`、
> `docs/cod_nav_2026_migration_worksheet.md`、`docs/smoke_test_runbook.md` §0.3/§7.2、
> `docs/3d_to_2d_survey.md` §七。

---

## 1. 槽位设计（新增了什么）

| 取值 | 局部代价地图生效图层 | 说明 |
|---|---|---|
| `scan`（默认） | `obstacle_layer`(/scan) | **原行为，一字不动** |
| `cloud` | `obstacle_cloud_layer`(/segmentation/obstacle 点云直投) | 原行为，一字不动 |
| `both` | `obstacle_layer` + `obstacle_cloud_layer` | 原行为，一字不动 |
| **`stvl`（新）** | **`stvl_layer`**（3D 体素层，带时间衰减） | 镜像全局那张图的 STVL，= COD 2026 两个 costmap 都上 STVL 的做法 |
| **`stvl_both`（新）** | `stvl_layer` + `obstacle_layer` + `obstacle_cloud_layer` | 三路冗余（最多余、最贵）；**本次未做 bench**，只做了静态真值表校验 |

命名沿用本仓库约定"**一个来源一个名字、不藏子开关**"：`scan` / `cloud` / `stvl` 各只开一路，
`both` = scan+cloud，`stvl_both` = **stvl + both**（读作"stvl 加 both 那两路"）。

### 1.1 机制：为什么是 `enabled` 开关，不是按槽位重写 `plugins` 列表

局部 costmap 的三个障碍层**全部**写进 `plugins`，槽位只切每层的 `enabled`：

```yaml
# src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml（local_costmap 段）
plugins: ["obstacle_layer", "obstacle_cloud_layer", "stvl_layer", "inflation_layer"]
```

这与 global 的 `obstacle_layer` / `stvl_layer` **完全同款**，也是本仓库既有机制。之所以不能按槽位
重写 `plugins` 列表本身：`nav2_common` 的 `RewrittenYaml` 只替换**文件里已存在的叶子键**
（`rewritten_yaml.py:110-122`），且 `convert()` 只会把字符串转 bool/数字（`:175-190`）——
一个字符串值到不了节点（`plugins` 需要 `string[]`，类型不符会直接抛错）。

**唯一真值表**放在 `src/rm_navigation/rm_navigation/launch/navigation_launch.py` 的
`LOCAL_OBSTACLE_LAYER_TABLE` / `GLOBAL_OBSTACLE_LAYER_TABLE`，launch 由它**生成**
`RewrittenYaml` 的 `enabled` 重写表达式（`build_param_substitutions()`）。

### 1.2 `scan` / `cloud` / `both` 的行为保持

- **代价图语义**：这三个取值下 `stvl_layer.enabled = false`，STVL 在 `updateBounds()` 里直接返回
  ⇒ 不 mark、不 clear、不参与 `updateCosts`，对栅格内容零贡献。
- **改不掉的差别（如实说）**：`plugins` 列表多了 `stvl_layer` 一项 ⇒ 该图层会被**构造**并
  `onInitialize()`，即使 `enabled=false` 也会像 nav2 其它图层一样在 `activate()` 时订阅
  `/segmentation/obstacle`（`costmap_2d_ros.cpp:540-560` 对所有插件无条件 `activate()`）。
  **注意这不是新引入的类别**：改造前 `obstacle_cloud_layer` 就已经是"在 plugins 里但 enabled=false"
  的同一形态、且订阅同一个话题 —— 也就是说"局部 costmap 进程本来就订阅着
  `/segmentation/obstacle`"（运行期 `ros2 node info` 可见，见 §3.5）。
  实测代价：`scan` 那一跑的 RTF / `missed rate` 与历史基线同档（见 §3.1），RSS 无增长趋势（§3.5）。
- **静态校验**：`tools/scripts/regress/local_obstacle_truth_table.py` 用**真的 `RewrittenYaml`**
  跑真实 params 文件，断言 `scan|cloud|both` 的生效图层集合与改造前逐位一致。

---

## 2. 参数与来源（逐键）

局部 `stvl_layer` 整块**镜像 `nav2_params_sim_base.yaml` 里 global 的 `stvl_layer`**
（同源 `/segmentation/obstacle`、同高度带、同衰减），只有 3 处偏离，全部标注理由。

主来源两处：
- **COD 2026**：`third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml`
  —— 局部段 `200-262`、全局段 `290-352`（**两段数值逐键相同**，即"两个 costmap 同款 STVL"），
  下面简称 **S:行号**。旁证：`docs/cod_nav_2026_deep_dive.md` §A8 的对照表 + §4-C2。
- **我们自己的 global stvl 段**：`nav2_params_sim_base.yaml` 的 `global_costmap…stvl_layer`。

| 键 | 局部取值 | 来源 | 备注 |
|---|---|---|---|
| `plugin` | `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer` | S:201/291 | 与我们 global 同 |
| `voxel_decay` | `0.5` | **S:203/293** | 线性衰减 0.5 s（"人走过"的残影 0.5 s 内消失） |
| `decay_model` | `0`（线性） | **S:204/294** | 1=指数、2=持久 |
| `voxel_size` | `0.05` m | **S:205/295** | 比全局栅格 0.04 略细 |
| `observation_persistence` | `0.0`（只用最新一帧） | **S:207/297** | |
| `max_obstacle_height` | `2.0` m（layer 级） | S:208/298 | 与我们 global 同；COD **源级**另有 `1.0`（S:230/320），STVL 里**源级覆盖 layer 级** ⇒ COD 实际生效的是源级值 |
| `mark_threshold` | `0` | **S:209/299** | 体素高度阈值 |
| `update_footprint_enabled` | `true` | S:210/300 | |
| `combination_method` | `1`（max） | S:211/301 | |
| `origin_z` | `0.0` | S:213/302 | |
| `transform_tolerance` | `0.2` s | S:215/305 | |
| `mapping_mode` / `map_save_duration` | `false` / `60.0` | S:216-217/306-307 | 只用在线地图 |
| `track_unknown_space` | `true` | S:206/296 | 与我们 global 同 |
| **`publish_voxel_map`** | **`false`** | **S:214/304** | ★偏离我们的 global(`true`)：局部 5×5 m @10 Hz 发体素图纯开销；要看体素图用 global 那张 |
| `observation_sources` | `livox_mark livox_clear` | 我们 global 的写法 | COD 是**单源** marking+clearing（S:313-314）；我们拆两路让 clearing 走更宽的垂直 FOV（`livox_clear` 保留 `vertical_fov_angle 1.029 / vertical_fov_padding 0.05 / model_type 1 / decay_acceleration 5.0`） |
| `livox_mark.topic` | `/segmentation/obstacle` | 我们 global | COD 吃 `/livox/lidar_filtered`（S:311）——那是**车上的原始去畸变点云**，我们仿真里对应的就是 linefit 之后的 `/segmentation/obstacle` |
| `livox_mark.obstacle_range` | `3.0` m | 我们 global；COD 源级 `8.0`（S:316/319） | 取 3.0 = 与 local 的 `/scan` 源 6.0 m / 点云源 3.0 m 同档，避免局部图比别的槽位"看得更远" |
| **`livox_mark.min_obstacle_height`** | **`0.0`** | ★偏离 COD `0.15`（**S:229/319**） | 我们 2026-09-24 为 RMUL2026 的 **0.4 m 矮内墙**把 global 改成 0.0；local 若用 0.15 会出现"global 看得见、local 看不见"的分裂 |
| `livox_mark.max_obstacle_height` | `2.0` | ★偏离 COD `1.0`（S:230/320） | 同理，与 global 对齐 |
| `filter` / `voxel_min_points` | `"voxel"` / `0` | S:235-236/325-326 | |
| `expected_update_rate` | `0.0`（不判 stale） | 我们所有障碍源 | 见 `docs/issues_and_findings.md`：0.5 会把"缺一帧"变成 costmap 永久 `isCurrent()==false` |
| `clear_after_reading` | `true` | S:234/324 | |

**未能从 COD 代码里确认的**：COD 的 `stvl_voxel_layer.obstacle_range: 3.0`（layer 级，S:212/302）
与源级 `8.0` 同时存在，按 STVL 的参数层级**源级应覆盖 layer 级**（`docs/cod_nav_2026_deep_dive.md` §A8
"层级陷阱"一段），但我们**没有跑过 COD 的代码去实测**，所以这条只作为"我们为什么不用 8.0"的参考。

---

## 3. A/B 实测（headless，同栈同目标点）

### 3.1 两组实验、四次跑（全部 headless，同一个栈）

栈固定：`world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi
planner:=smac2d spin_speed:=0.0 nav_rviz:=False`，两次跑**只差 `local_obstacle` 一个词**。
回归：`python3 tools/scripts/regress/nav_smoke_regression.py --goal <G> --localization gicp+local_<标的>`。

- **A 组（任务口径目标点）** `--goal -1.0 2.0`：`scan`（控制组）vs `stvl`（实验组）
- **B 组（补充：贴墙目标点）** `--goal 0.25 -1.5`：机器人从 (0,0) 直着开到 x=+0.25，
  而 RMUL2026 的墙在 x≈+0.50~0.62 ⇒ **终点离墙 0.25 m**，用来回答"近距/盲区到底谁看得见"

| 跑 | goal | `local_obstacle` | PASS | status | 用时(s) | **RTF** | **missed rate** | d_min | recoveries | cmd 峰值 nav(v,w) | chassis(v,w) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| A-scan | -1.0 2.0 | `scan`（控制） | ✅ | SUCCEEDED | 3.4 | **0.75** | **0** | 0.0 | 0 | (1.45, 0.91) | (1.45, 0.91) |
| A-stvl | -1.0 2.0 | `stvl` | ✅ | SUCCEEDED | 3.2 | **0.73** | **0** | 0.0 | 0 | (1.42, 0.83) | (1.42, 0.83) |
| B-scan | 0.25 -1.5 | `scan`（控制） | ✅ | SUCCEEDED | 2.9 | **0.75** | **0** | 0.0 | 0 | (0.58, 0.50) | (0.58, 0.50) |
| B-stvl | 0.25 -1.5 | `stvl` | ✅ | SUCCEEDED | 2.9 | **0.75** | **0** | 0.0 | 0 | (0.55, 0.49) | (0.54, 0.47) |

四条全部 `status=SUCCEEDED`、`d_min=0.0 m`、`recoveries=0`、真值 `pose_delta` 与目标距离一致
（A 组 ≈2.72~2.76 m、B 组 ≈1.70~1.71 m）⇒ **没有一次恢复行为、没有假到达**。

运行期真值（`ros2 param get`，两跑都取）：

```
/local_costmap/local_costmap plugins = ['obstacle_layer','obstacle_cloud_layer','stvl_layer','inflation_layer']
A/B-scan : obstacle_layer=True   obstacle_cloud_layer=False  stvl_layer=False
A/B-stvl : obstacle_layer=False  obstacle_cloud_layer=False  stvl_layer=True
```

日志里 `local_costmap` 构造的图层（`Using plugin ...`）两跑都是四个 —— 即 `scan` 跑里
STVL 图层**被构造但 enabled=false**（见 §1.2 的说明）。

### 3.2 局部代价地图"看到了什么"：探针证据（只订阅，不改栈行为）

探针 `tools/scripts/regress/local_obstacle_probe.py`（2 Hz）对 `/local_costmap/costmap_raw`
（`nav2_msgs/Costmap`，254=真障碍）与 `/global_costmap/costmap_raw` 做环带统计，同时统计两个
**原始来源**（`/scan` 与 `/segmentation/obstacle`）在机器人附近的量。

| 指标 | A-scan | A-stvl | B-scan | B-stvl |
|---|---|---|---|---|
| 采样数 | 88 | 82 | 87 | 83 |
| 局部图 0~0.5 m 真障碍格 (p50/max) | 0 / 0 | 0 / 0 | **7 / 12** | **2 / 3** |
| 局部图 0.5~1.0 m 真障碍格 (p50/max) | **135 / 178** | **20 / 34** | 3 / 13 | 6 / 13 |
| 局部图车后扇区 0.3~1.5 m (p50/max) | 240 / 267 | 35 / 49 | 0 / 0 | 0 / 0 |
| 到最近真障碍的距离 min/p10/p50 (m) | 0.601 / 0.755 / 0.835 | 0.620 / 0.635 / 0.809 | **0.349 / 0.351 / 0.356** | **0.396 / 0.396 / 0.397** |
| 全局图 1 m 内真障碍格 (p50) 〔对照〕 | 58 | 56 | 81.5 | 81 |
| `/scan` 有效回波数 (p50) | 1218 | 1238 | 1091 | 1110 |
| `/scan` 在 0~0.5 m 的回波数 (max) | 0 | 0 | **35** | **25** |
| 点云在 0~0.5 m 的点数 (max) | 1 | 0 | **36** | **28** |
| 点云在 0~0.5 m **且落在 p2l 高度带外**的点数 (max) | 0 | 0 | **0** | **0** |

（完整工具：`python3 tools/scripts/regress/compare_local_obstacle_probe.py a=…jsonl b=…jsonl`）

读法：

1. **A 组这个目标点根本考不到"近距"**：整段导航里车到最近墙面的距离 p10 = 0.72 m（scan）/
   0.64 m（stvl），所以"0~0.5 m 环带里有没有障碍"两边都是 0 —— **用这个目标点无法区分**
   "局部 STVL 是否多看见东西"。这是本次实验最重要的**否定性**结论之一：任务书里"贴墙 0.3 m"
   的场景，用 `--goal -1.0 2.0` 是构造不出来的（要 B 组那种贴墙终点）。
2. **0.5~1.0 m 的墙体标记数差 ~7 倍**（135 vs 20，车后扇区 240 vs 35）：同一时刻、同一段墙、
   同一台车，**局部 STVL 在局部图上留下的"真障碍格"少得多**。原因见 §3.3 的栅格快照：
   STVL 的体素是 **0.05 m**，而局部栅格是 **0.02 m** ⇒ 一个体素只落到一个栅格 ⇒ 墙在局部图上
   变成**点状**而不是连续线；`/scan` 的波束角分辨率 0.0043 rad（0.8 m 处相邻回波间距 3 mm）⇒
   墙是**连续线**。全局图（0.04 m 栅格）那一列只是对照，它含 `static_layer`，不能用来比较
   STVL 的标记密度（这一点也是本次才想清楚：**全局图不能当局部 STVL 的对照组**）。
3. **B 组（贴墙）证明"45 cm 盲区"在当前配置下不存在**：`/scan` 在 0~0.5 m 有 25~35 个回波
   （p2l `range_min` 已在 2026-09-26 由 0.45 一路降到 0.05），两个配置的局部图都在
   **0.35~0.40 m** 处标到了墙 ⇒ 局部 STVL "消盲区"这条**动机已经不成立**（本地图 `scan` 也看得见）。
4. **点云并没有提供 p2l 高度带之外的信息**（B 组"高度带外的点 max = 0"）：`/segmentation/obstacle`
   的点在 livox_frame 下几乎全落在 p2l 的 z∈[-1.0, 0.1] 带内 ⇒ 在本场景里，
   "STVL 吃 3D 全高度、/scan 只吃一条带"这条**收益也是 0**（墙都是高的、地面已被 linefit 去掉）。

### 3.3 墙上标记形态：连续线 vs 点状（原始栅格快照）

B 组两跑都 dump 了"离障碍最近那一刻"的 `local_costmap_raw`（`--dump-grid`），对同一列（墙面
所在列 x≈0.52 m）逐格看 254：

| | B-scan（`/scan`） | B-stvl（STVL） |
|---|---|---|
| 该列 254 格数 / 跨度 | 137 / 192 格（3.84 m） | 70 / 173 格（3.46 m） |
| **占位率** | **71%** | **40%** |
| 连续段 | 5 段，最长 **110 格连续**（2.2 m） | **70 段，全部长度为 1** |
| 空隙 | 主要为 0（首尾各一段大空） | 69 个，长度 1~2 格 |
| 254 最近邻间距 p50 | **0.020 m**（= 1 格，连续） | **0.040 m**（隔 1~2 格，点状） |
| 253（膨胀带）格数 | 13801 | 13923（**几乎一样**） |

（另：两跑的局部栅格里 **255/NO_INFORMATION 格数都是 0** —— `track_unknown_space: true` 是我们
从 global 段镜像过来的值，在 `combination_method: 1`(max) 下它**本可以把整张局部图写成未知**，
实测没有触发；换 world/换 STVL 版本时值得复看这一项。）

⇒ 结论：**局部 STVL 的『致命核心』是周期 ≈0.05 m 的点状线**（0.05 m = `voxel_size`，与
"一个体素 → 一个 0.02 m 栅格"完全吻合），而 `/scan` 是连续线。**但 0.5 m 的膨胀带两者几乎
一样大**（13801 vs 13923 格）⇒ 规划器/MPPI 真正用的代价格几乎没变，这解释了为什么两跑都
一次过、没有任何恢复行为。**风险留给**：任何依赖"致命格连续性"的下游（例如把 lethal 当
碰撞判据的碰撞监控/自定义 critic），在局部 STVL 下会看到一条有 ~0.05 m 缝的墙。

### 3.4 代价：CPU / RTF / missed rate

| | A-scan | A-stvl | B-scan | B-stvl |
|---|---|---|---|---|
| RTF（回归脚本口径） | 0.75 | **0.73（-2.7%）** | 0.75 | 0.75（±0） |
| `Control loop missed its desired rate` 计数 | 0 | 0 | 0 | 0 |
| 话题频率 pcloud/imu/scan/odom (Hz) | 7.7/77.3/8.0/7.7 | 7.7/78.3/8.0/8.0 | 8.0/78.7/8.0/8.0 | 7.7/77.7/7.7/8.0 |
| controller_server RSS 首→末 (MB) | 59.2 → 71.0 | 52.8 → 71.6 | 60.2 → 70.3 | 58.3 → 71.8 |
| 局部图发布/接收（`costmap_raw` 条数） | 174 | 164 | 173 | 165 |

- **RTF 代价 ≈ 1~3%（本案 0.75→0.73）**，`missed rate` 两跑都是 0 —— 本机 28 核、RTF 只有
  ~0.75 时，局部再加一张 5×5 m 的 STVL 并没有把主循环压垮（全局那张 STVL 早就开着）。
- **RSS 没有"STVL 特有"的增长**：`scan` 跑里 STVL 图层虽然被构造（enabled=false），RSS 增长
  与 `stvl` 跑同档（+12 MB / +19 MB，四条都在同一量级）⇒ 没看到"禁用图层仍在无限囤点云"。
- ⚠️ 口径：探针订阅 `costmap_raw` 会让发布端多序列化一张图（两个跑都有，公平），
  所以绝对 RTF 与历史基线（同栈 `gicp-pcl` 的 0.76）**不完全可比**，只做横向比较。



---

## 4. 结论与建议

**结论：在本次的 sim 目标点集合与 RTF ≈0.73~0.75 下，局部 STVL 与 `/scan` 单源
「导航结果上不可区分，局部图内容上可区分且更稀疏」。**

分项：

| 维度 | 判定 |
|---|---|
| 能不能跑通 | ✅ 完全等价：A/B 两组四次跑全部 PASS/SUCCEEDED、`recoveries=0`、`d_min=0`、`missed=0` |
| 到达时间/速度包线 | 无可测差异（3.4 vs 3.2 s；cmd 峰值 1.45 vs 1.42 m/s，都在噪声内） |
| CPU/RTF | 代价很小：RTF **-2.7%**（A 组）~ **±0**（B 组），`missed rate` 恒 0；RSS 无异常增长 |
| 看得更多？ | ❌ **没有**：A 组两边的 0~0.5 m 环带都是空；B 组贴墙到 0.35~0.40 m 时**两边都看得见**（p2l `range_min=0.05`，45 cm 盲区早已不存在）；点云里"落在 p2l 高度带外"的点 = 0 |
| 看得更少？ | ⚠️ **是**：局部图的真障碍格数少 ~7 倍（135 vs 20），墙面 lethal 从"连续线"变成"周期 0.05 m 的点状线"（占位率 71% → 40%）；膨胀带几乎不变，所以对规划/控制**暂时无害** |
| 贴墙行为 | 贴墙终点姿态：`scan` 停在离墙 **0.349 m**、`stvl` 停在 **0.396 m**（后者略保守 4 cm）；A 组的最近距离反而 `stvl` 略小（p10 0.635 vs 0.717 m）⇒ **方向不一致，视为噪声**，但"STVL 让车更贴墙"没有证据 |
| 时间维（本次唯一没测到的收益） | ❓ sim 里**没有任何动态障碍**，`voxel_decay 0.5 s` 的收益（人走过/残影）在本实验里**根本无从体现** |

**建议**：

1. **默认值保持 `local_obstacle:=scan` 不变**（本次没有任何证据支持把它换掉；换掉还会把
   局部图的 lethal 核心变成点状线，属于"用一个已证实的隐患换一个未证实的收益"）。
2. **`local_obstacle:=stvl` 保留为可选槽位**：它已经能跑通且代价很小，是**将来做动态障碍/
   残影实验的载体**（那才是 STVL 的真正收益来源）；等实验台上出现动态障碍后再复测。
3. **若要用 `stvl`**：建议先把 `stvl_layer.voxel_size` 从 0.05 降到 **0.02**（= 局部栅格分辨率，
   消掉点状线），或直接用 `stvl_both`（STVL 提供时间维、`/scan` 提供连续线）。
   ⚠️ 这两条都是**本次未实测**的建议，见 §6。
4. **视觉/调试口径**：局部图变得"稀疏"是**正常现象**（体素量化），不要当成丢数据；
   真要看 STVL 的完整语义，看全局图（`publish_voxel_map: true`，`/global_costmap/voxel_grid`）。


---

## 5. 怎么跑 / 怎么回退

### 5.1 跑（原生 `ros2 launch`，无头）

```bash
cd ~/HzMi_rmsimulation
source /opt/ros/humble/setup.bash && source install/setup.bash

# 对照（原行为）：局部只吃 /scan
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False local_obstacle:=scan

# 实验：局部也上 STVL（另一个终端发同一个目标点）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=False local_obstacle:=stvl

# 目标点 + 回归（栈已起来后）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization gicp+local_stvl
```

一键 A/B（起栈 → 探针 → 回归 → 收尾 全在**同一个 bash 调用**里；本环境每条 bash 调用是独立
PID namespace，跨调用看不见彼此进程）：

```bash
bash .tmp_cache/stvl_local/run_ab.sh scan scan    # 控制组
bash .tmp_cache/stvl_local/run_ab.sh stvl stvl    # 实验组
```

§3.2/§3.3 那些数字是这么来的（**只订阅，不改栈的行为**；两跑都要挂同一个探针才可比）：

```bash
# 栈起来之后、发目标点之前挂上探针（--dump-grid 会存下"离障碍最近那一刻"的原始栅格）
python3 tools/scripts/regress/local_obstacle_probe.py \
    --out /tmp/scan.probe.jsonl --period 0.5 --dump-grid /tmp/scan.grid.npz &
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization gicp+local_scan
kill -INT %1        # 探针收到 SIGINT 会把 summary 写进 jsonl

# 出表（把四次跑的 jsonl 并排；--grid 可再画墙面 ASCII）
python3 tools/scripts/regress/compare_local_obstacle_probe.py \
    scan=/tmp/scan.probe.jsonl stvl=/tmp/stvl.probe.jsonl
```

> 本次用的"一次调用跑完 起栈→探针→回归→收尾"驱动脚本是 `.tmp_cache/stvl_local/run_ab.sh`
> （与既有 `.tmp_cache/five_way/run_one.sh` 同一套纪律：**同一 bash 调用内**完成，本环境每条
> bash 调用是独立 PID namespace，跨调用看不见彼此进程）。`.tmp_cache/` 不入库，所以上面把
> 关键命令都写全了；驱动脚本本身按需重写即可（隔离三件事：`HOME=/tmp/gzhome-<tag>`、
> 非默认 `ROS_DOMAIN_ID`、`unset DISPLAY` + 空闲的 `GAZEBO_MASTER_URI`）。

只看槽位到底开了哪些图层（**不用起栈**）：

```bash
python3 tools/scripts/regress/local_obstacle_truth_table.py --markdown
```

### 5.2 回退

- **最省事**：`local_obstacle:=scan`（默认值）—— `stvl_layer.enabled=false`，等价于今天。
- **彻底移除**：删掉 `nav2_params_sim_base.yaml`…`local_costmap` 段 `plugins` 里的
  `"stvl_layer"` 一项 + 删掉整段 `stvl_layer:`，再删 `navigation_launch.py` 两张表里
  `'stvl'/'stvl_both'` 两行与 arg 的 `choices` 两项。**改回后务必重跑**
  `local_obstacle_truth_table.py`。

---

## 6. 未验证 / 未能测量的清单

1. **`stvl_both` 没有跑过 bench**：只做了静态真值表校验（`local_obstacle_truth_table.py`
   断言它 = stvl+scan+cloud 三层全开），**没有实测**它与 `scan`/`stvl` 的差异、也没量它的 CPU。
2. **动态障碍/衰减完全没测**：sim 里没有会动的东西，`voxel_decay 0.5 s`、`decay_model 0` 的行为
   （残影多久消失、会不会把 0.4 m 矮墙衰减掉）**一次都没有被观测到**。
   `docs/cod_nav_2026_deep_dive.md` §9 第③条早就把"`voxel_decay 0.5` linear 会擦掉 0.40 m 矮墙"
   列为**未实测的推断**，本次也没能证伪或证实。
3. **点状线只在一个姿态上量过**：§3.3 的数字来自 B 组各自"离墙最近那一刻"的单帧
   （`best_grid`）。不同距离/角度的墙面占位率是否一样、以及"0.05 m 缝会不会真的让某个下游漏检"
   **没有测**（没有做碰撞/漏检实验，也没有看 MPPI 的 critic 内部）。
4. **`min_obstacle_height 0.0`(我们) vs `0.15`(COD) 的差别没测**：本场景地面已被 linefit 去掉，
   取 0.0 只是为了与 global 一致；会不会引入贴地幽灵障碍**在 sim 里没复现**（实车才可能暴露）。
5. **`publish_voxel_map: false` 是我们的选择**（COD 两图都 false，我们 global 是 true）：
   局部发体素图的开销**没有单独量化**。
6. **CPU 只有 RTF/频率代理，没有真正的 CPU% 采样**：`missed rate` 与 RTF 都受仿真 RTF 上限
   （0.73~0.75）压制 ⇒ 若把仿真跑得更快（更简单的 world、更少传感器），STVL 的相对代价会更大。
7. **只测了一个 world（RMUL2026）+ 一个控制器（mppi）**：`local_obstacle:=stvl` 与 RPP/DWB/TEB
   的组合、以及 RMUC/RMUL 场地**没跑**。
8. **数字口径**：探针订阅 `costmap_raw` 本身会让发布端多做一次序列化（两个跑都有）⇒ 与历史
   基线（不订阅探针的 0.76）不是严格同口径；`lc_min_r` 精度受"机器人是否恰在滚动窗中心"
   （nav2 取整到栅格，±1 cm）影响。

