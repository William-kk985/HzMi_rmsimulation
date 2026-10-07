# 地图 / 先验资产：规范、清单与实测（`docs/map_assets.md`）

> **一句话定位**：`src/rm_nav_bringup/map/` 与 `src/rm_nav_bringup/PCD/` 里**哪两个文件是"当前默认资产"**、
> 每个文件是什么 / 哪来的 / 实测多好 / 谁在引用它、以及**以后怎么换图、怎么回滚**。
> 动地图相关的东西（换先验、重跑建图、清理存档）先看这一份。
>
> 状态：**2026-10-07 首次建立**；本次执行了「按测量提升一对新资产」（§3、§4）。
> 上游依据：`docs/worlds.md`（世界资产怎么生成）、`docs/mapping_2d_from_cloud.md`（2D 图从哪来）、
> `docs/continue_mapping.md`（续建/存档/守卫）、`docs/lio_divergence_no_impact.md`（用户那次 nav 跑）。

---

## 0. 速查（当前默认对）

| 槽位 | 文件 | 谁在默认读它 | 当前内容（2026-10-07 提升后） |
|---|---|---|---|
| 2D 先验（nav2 栅格） | `src/rm_nav_bringup/map/RMUC2026.yaml` → `RMUC2026.pgm` | `mode:=nav` 的 map_server（launch 默认 `map_yaml:=<share>/map/<world>.yaml`） | `577 x 326 @ 0.05 m`，`origin [-25.2, -9.09, 0]`，占用 9769 / 自由 146198 / 未知 32135 |
| 3D 先验（点云） | `src/rm_nav_bringup/PCD/RMUC2026.pcd` | `localization:=gicp \| icp \| small_gicp`（launch 里 `pcd_path = <share>/PCD/<world>.pcd`） | **203,352 点 / 6.51 MB**，8 字段（x y z intensity + 4），体素 0.10 m |

* 两份都来自**同一次实跑建图会话** `RMUC2026_v3`（2026-10-07 00:46–01:15，`world:=RMUC2026`、
  `lio:=small_point_lio`、`mapper:=slam_toolbox`）⇒ 2D 与 3D **同源同场次**（这是本次提升的一个理由）。
* 提升前的旧一份（= 2026-10-06 19:14 那次"好"会话的产物）留在：
  `map/RMUC2026.pgm.bak-20261007`、`map/RMUC2026.yaml.bak-20261007`、`PCD/RMUC2026.pcd.bak-20261007`。
* `install/` 里是**逐文件软链**（`colcon build --symlink-install`）⇒ 覆盖源码树里的同名文件即可生效，
  不用重编；**但绝不能删了重建**（软链会悬空）—— 见 §5 的"就地覆盖"约定。
* 当前默认 2D 先验的预览图（左 = 栅格图与四档余量的规划路径，右 = STL 真值高度 + 红 = 台阶边沿）：
  `docs/img/RMUC2026_prior_20261007.png`（由 `verify_low_terrain.py --png` 生成）。

---

## 1. 命名规范（本仓库的约定）

| 类别 | 形式 | 谁读它 | 备注 |
|---|---|---|---|
| **2D 先验（当前）** | `map/<world>.pgm` + `map/<world>.yaml` | `mode:=nav` 的 map_server；`tools/scripts/regress/*`、`tools/scripts/mapping/*` 的 `--map-yaml` 默认值 | **两个文件是本仓库的"地图"，名字不允许改**（改了 launch 默认就找不到） |
| **3D 先验（当前）** | `PCD/<world>.pcd` | `localization:=gicp/icp/small_gicp` | 名字由 launch 拼出（`PCD/<world>.pcd`），**没有 launch 参数可改** |
| 会话存档（位姿图） | `map/<world>_<tag>.posegraph` + `.data` + `.meta.yaml` | `map_archive.sh` / launch 的续建守卫（`map_asset_guard.py`） | `<tag>` = 会话名（`v2` / `v3` / `good` / `cont` …）；**三者必须同在 `map/`** |
| 会话存档（点云） | `PCD/<world>_<tag>.pcd` + `PCD/<world>_<tag>.meta.yaml` | `cloud_accumulator` 的 `~/load` / `~/save`、`map_archive.sh info` | 与位姿图同名同标签 ⇒ 一个 `<tag>` 就是"一次会话的全部产物" |
| 自动备份 | `<文件>.prev-<YYYYmmdd-HHMMSS>` | `map_archive.sh restore/backups`（按通配找代）、`cloud_accumulator` 的 save | **工具自动生成，不要手工改名/搬走**（搬走 = `restore` 找不到那一代） |
| 人工提升备份 | `<文件>.bak-<YYYYMMDD>` | 人 | 本次提升产生（§4.4） |
| 历史合成资产 | `<文件>.synth.bak` | 无（历史） | 2026-10-05 的"由 STL 合成"那一版，见 `docs/worlds.md` §4；可复跑 `tools/scripts/world/stl_to_world.py` |
| 退役 / 凌乱 | `map/attic/`、`PCD/attic/` | 无 | **只搬不删**；每个 attic 里有一份 `README.md` 说明来龙去脉（§6） |

配套规矩：

1. **覆盖 `<world>.pgm|yaml` / `PCD/<world>.pcd` 之前必须先留一份备份**（`.bak-<日期>` 或 `.prev-<ts>`），
   并且**就地覆盖**（`cp -f` / 重写同一路径），不要 `rm` 再建。
2. **会话存档一律不入库**（几十 MB 的 `.posegraph/.data`）；`*.prev-*` 是回滚链，也不入库。
3. 备份代默认保留**最新 2~3 代**（`map_archive.sh` 的 `keep=3` 语义）；更老的可以搬 `attic/`（本次没有更老的）。
4. 存档的"属于哪个 world / 哪个出生点"由 sidecar `.meta.yaml` 记录，续建前由
   `src/rm_nav_bringup/scripts/map_asset_guard.py` 校验（`map` 系 = **出生点相对系**，
   换 world 就是换原点 ⇒ 串味的地图叠在一起 = 假墙）。

---

## 2. 清单（逐个文件）

### 2.1 `src/rm_nav_bringup/map/`

| 文件 | 大小 | mtime | 它是什么 / 哪来的 | 实测质量 | 处置 |
|---|---|---|---|---|---|
| `RMUC2026.pgm` + `.yaml` | 188117 / 126 B | 2026-10-07 12:38 | **当前 2D 先验**（= 原 `RMUC2026_v3_spl.*` 就地提升） | 全场边沿被表示 **52.71%**、撞击窗 **31/31**、可行驶斜面被标占用 5.21%（§3.1） | **提升（本轮）** |
| `RMUC2026.pgm.bak-20261007` + `RMUC2026.yaml.bak-20261007` | 180639 / 126 B | 2026-10-06 19:20 / 19:27 | 提升前的旧 2D 先验（= `RMUC2026_good.pgm` 的同一份，`md5 93021a5e…`；来源 = "好"存档 `RMUC2026_dropab_ab_g_long` 的 `/map`） | 全场边沿 49.44%、撞击窗 28/33、可行驶斜面误占 3.29%（§3.1） | 新增（回滚用） |
| `RMUC2026_v3_spl.pgm` + `.yaml` | 188117 / 133 B | 2026-10-07 01:17 | 用户最新一次建图（`RMUC2026_v3` 会话）产出的 2D 图；**提升的来源**；`tools/scripts/diag/run_nav_lio_timeline.sh` 的 `--map-yaml` 默认值就是它 | = 现 `RMUC2026.pgm`（逐字节相同，`md5 82e1742b…`） | **保留**（工具默认值 + 提升凭据） |
| `RMUC2026_good.{posegraph,data,meta.yaml}` + `.pgm/.yaml` | 33440928 / 23645218 / 861 B / 180639 / 131 B | 2026-10-06 19:14–19:20 | **受保护的"好"存档**（会话名 `RMUC2026_dropab_ab_g_long`，被用户改名为 `good`）；`.pgm/.yaml` 是它的 `/map` 快照 | 曾经的默认 2D 先验（= 上面那份 `.bak-20261007`） | **保留**（`docs/continue_mapping.md` §10 引用） |
| `RMUC2026_v2.{posegraph,data,meta.yaml}` | 9706842 / 4587794 / 789 B | 2026-10-06 18:00 | 用户 2026-10-06 晚上那次会话存档（`map_name:=RMUC2026_v2`） | world/出生点与当前一致（sidecar 可校验）；未做图面实测 | **保留**（`docs/continue_mapping.md`、`docs/slam_drops_loopclosure_odom.md` 引用） |
| `RMUC2026_v3.{posegraph,data,meta.yaml}` | 38084788 / 28259182 / 844 B | 2026-10-07 01:15 | **最近一次会话存档**（本次提升的两个资产都出自它）；`.session.yaml` 里 `archive_base` 就指它 | sidecar：`world=RMUC2026`、`spawn=(10.925,2.525)`、`resolution 0.05` | **保留**（可能是活会话的续建底图，只读） |
| `RMUC2026_v3.{posegraph,data,meta.yaml}.prev-20261007-005425` / `-011506` | 17.6/7.9 MB、24.3/14.6 MB、790/844 B | 2026-10-07 00:46 / 00:54 | `map_archive.sh save` 覆盖前自动留的**两代回滚** | — | **保留**（`restore` 靠它） |
| `RMUC2026_cont.{posegraph,data,meta.yaml}` | 6351884 / 1262032 / 885 B | 2026-10-06 15:06 | **agent 的续建测试存档**（"续建会不会被污染"那次实验留下的，污染云 z 到 +23.10 m 的那份） | `docs/continue_mapping.md` §7 有它的对照数字；**不是**可用先验 | **保留**（多份文档引用；若要清理，整组搬 `attic/`） |
| `RMUC2026.pgm.synth.bak` + `RMUC2026.yaml.synth.bak` | 204615 / 129 B | 2026-10-06 16:16 | **历史合成资产**：2026-10-05 由 `RMUC2026.stl` 直接生成的 2D 图（`600x341`、`origin [-25.925,-9.425]`），即 `docs/worlds.md` §4.1 描述的那一份；git 提交 `280888e` 里是它的原始版本 | 全场边沿被表示未重测（历史）；当时实测 free 237.8 m² | **移入 `map/attic/`** |
| `RMUC2026_spl.{pgm,yaml}` | 120727 / 129 B | 2026-10-06 15:45 | 早期"从 `/scan` 累积"的 2D 图（`small_point_lio` 建图那次的产物） | 全场边沿 **37.43%**、撞击窗 32/33、可行驶斜面误占 4.05%、**未知 60%**（窗口只覆盖 t 型区域） | **保留**（`docs/mapping_small_point_lio.md` 等 20+ 处引用） |
| `RMUC2026_cloud.{pgm,yaml}` | 195615 / 133 B | 2026-10-06 11:45 | **已被否决的点云投影图**（`pcd_to_nav2_map.py` 从 `PCD/RMUC2026_spl.pcd` 投影，`600x326`） | 全场边沿 **35.98%**、可行驶斜面被标占用 **7.23%**（超 5% 判据）、未知 37% ⇒ `docs/mapping_2d_from_cloud.md` §11 判定"不用它当先验" | **保留**（该文档 A/B 的对照物） |
| `empty_map.{pgm,yaml}` | 40015 / 126 B | 2026-09-09 | 空图（`bringup_sim.launch.py` 里 `empty_map_dir`，非 nav 模式占位用） | — | **保留**（launch 引用） |
| `.session.yaml` | 661 B | 2026-10-07 01:01 | 上一次 launch 写的会话状态（`map_name=RMUC2026_v3`、`world=RMUC2026`、launch cmdline、`session_pid`） | — | **保留**（`map/../.gitignore` 已忽略；`map_archive.sh` 的证据链之一） |
| `RMUC.{pgm,yaml,posegraph,data}`、`RMUL.{pgm,yaml,posegraph,data}`、`RMUL2026.{pgm,yaml,pbstream}`、`RMUL2026_world_backup.{pgm,yaml}` | 见目录 | 2026-02-04 / 2026-09-23 | **其它 world** 的资产（RMUC / RMUL / RMUL2026） | 未在本次范围内重测 | **保留**（`RMUC.yaml`/`RMUL*.yaml` 是各 world 的 nav 默认图） |

### 2.2 `src/rm_nav_bringup/PCD/`

| 文件 | 大小 | mtime | 它是什么 / 哪来的 | 实测质量 | 处置 |
|---|---|---|---|---|---|
| `RMUC2026.pcd` | 6507515 | 2026-10-07 12:38 | **当前 3D 先验**（= 原 `RMUC2026_v3.pcd` 就地提升）；`cloud_accumulator` save 于 2026-10-07 00:54 | 203,352 点；离线 GICP 探针 fitness 均值 0.00217 m²；整栈无头 `score≈0.0027`、`采纳 100%`（§3.3） | **提升（本轮）** |
| `RMUC2026.pcd.bak-20261007` | 5713627 | 2026-10-06 18:34 | 提升前的旧 3D 先验（= `RMUC2026_v2.pcd`，逐字节相同 `md5 9db987a4…`）；来自 2026-10-06 18:34 那次 `cloud_accumulator save` | 178,543 点；离线探针 fitness 均值 **0.00207 m²**（略优）、中位残差 0.023 m；整栈无头 `score p50 0.00290 m²`（§3.3） | 新增（回滚用） |
| `RMUC2026_v3.pcd` + `.meta.yaml` (+ `.prev-20261007-005427`) | 6507515 / 1168 B / 2577305 | 2026-10-07 00:54 | 提升的来源（`RMUC2026_v3` 会话的点云）；sidecar：`accumulated_points 44,522,931`、`voxel 0.1`、`z_span 2.358`、`health_warning false`、`skipped_frames_jump 0`、`tf_frames_ok 8734 / failed 55` | = 现 `PCD/RMUC2026.pcd`（逐字节相同 `md5 5b494a60…`） | **保留**（提升凭据 + `ground_segmentation_slots.md` §12 引用） |
| `RMUC2026_v2.pcd` + `.meta.yaml` (+ `.prev-20261006-183422`) | 5713627 / 1161 B / 2497273 | 2026-10-06 18:34 | 2026-10-06 那次会话的点云存档（= 上面那份 `.bak-20261007`） | 178,543 点（`pcd_stats` 复核） | **保留**（存档对，`map_archive.sh info --name RMUC2026_v2` 仍可用） |
| `RMUC2026_mapped.pcd` | 17400944 | 2026-10-06 15:52 | 一次全场地累积云（1,450,064 点），当"健康云"的标尺用 | `z ∈ [-0.53, 1.43]`、`xy` 足迹大；被 `pcd_bbox_health.py` / `cloud_accumulator` 注释当**实测依据**引用 | **保留**（工具/源码注释引用它） |
| `RMUC2026_spl.pcd` | 684633 | 2026-10-05 18:20 | `small_point_lio` 建图那次的点云（26,618 点同量级），`RMUC2026_spl` 2D 图的来源 | 点数/字段与既有资产同构 | **保留**（多份文档引用） |
| `RMUC2026_cont.pcd` + `.meta.yaml` | 3509531 / 1009 B | 2026-10-06 15:06 | agent 测试存档的点云（**含 LIO 退化污染**，z 到 +23.10 m） | `docs/continue_mapping.md` §7：污染云 z 跨度 27.41 m（健康云 1.96~2.02 m） | **保留**（多份文档引用；清理时整组搬） |
| `RMUC2026.pcd.synth.bak` | 852025 | 2026-10-06 16:16 | **历史合成点云**：由 STL 采样的"地面以上薄壳"（26,618 点 / 0.85 MB），`docs/worlds.md` §4.2 描述的就是它；git `280888e` | 与 `initial_pose=[0,0,0]` 语义精确对齐（地面 z=-0.06） | **移入 `PCD/attic/`** |
| `RMUC.pcd`、`RMUL.pcd`、`RMUL2026.pcd` | 18.2 / 50.9 / 1.7 MB | 2026-02-04 / 2026-09-23 | 其它 world 的 3D 先验 | 未重测 | **保留** |

### 2.3 仓库其它位置

| 文件 | 处置 |
|---|---|
| `good_map_preview.png`（仓库根，1136x636，旧默认 2D 先验的预览图） | **移入 `map/attic/`**（它预览的那张图已被本次提升取代；当前默认图的预览见 `docs/img/RMUC2026_prior_20261007.png`） |
| `.tmp_*`（`.tmp_mapassets/` 为本次的测试目录）、`log/`、`build/`、`install/` | 测试/构建产物，不入库；本次的测试目录用完即清（§7） |

---

## 3. 实测对照（2026-10-07）

口径全部是仓库现成工具，**同一命令、同一真值**（`RMUC2026.stl` → 0.05 m"最高可站立面"高度图）：

* 2D：`python3 tools/scripts/regress/verify_low_terrain.py --map <yaml> --label <x>`（本次 JSON/PNG 在 `.tmp_mapassets/`）
* 2D 同窗口对照 + 逐格对照：`python3 tools/scripts/mapping/map_prior_ab.py <旧>.yaml <新>.yaml`
  （**本次新增的工具**，离线、无 ROS；口径与 `verify_low_terrain.py` 同一套真值）
* 3D：`install/gicp_registration/lib/gicp_registration/gicp_selfsim_probe`（**与节点同一份代码路径**：
  同 `voxelDownsample` + 同 `RegistrationBackend` 参数），扫描 = 整栈无头跑里录的真实 `/livox/lidar/pointcloud`
* 整栈：`.tmp_mapassets/nav_pair_check.sh`（`mode:=nav localization:=gicp`，隔离 domain/端口/HOME，`nav_rviz:=False`）

### 3.1 四张 2D 候选图（各自窗口，`verify_low_terrain.py`）

| 图 | 栅格 / origin | 占用 / 自由 / 未知 | 全场台阶边沿被表示 | 撞击窗 `map(-3.6,3.1)±0.6` | 可行驶斜面被标占用 | 判据结论 |
|---|---|---|---|---|---|---|
| **`RMUC2026.yaml`（提升后 = 原 `v3_spl`）** | 577x326 / (-25.20,-9.09) | 9769 / 146198 / 32135 | **52.71%** (3052/5790) | **31/31 = 100%**，最近占用格 0.000 m | 5.21% | ⚠ 未达标（边沿覆盖率是历史遗留项；撞击点与斜面两项见 §3.2） |
| `RMUC2026.pgm.bak-20261007`（提升前） | 568x318 / (-24.80,-8.92) | 9096 / 136709 / 34819 | 49.44% (2822/5708) | 28/33 = 85% | **3.29%** | ⚠ 未达标 |
| `RMUC2026_spl.yaml` | 382x316 / (-15.80,-8.90) | 2569 / 45834 / **72309** | 37.43% (1484/3965) | 32/33 = 97% | 4.05% | ⚠ 未达标（窗口太小、60% 未知） |
| `RMUC2026_cloud.yaml`（已否决） | 600x326 / (-26.05,-8.80) | 7052 / 94952 / 93596 | 35.98% (2090/5808) | 30/33 = 91% | **7.23%** | ⚠ 未达标（斜面误占超 5%） |

「全场边沿被表示」两两都在 35%~53%、都够不到工具写的 80% 判据 —— 这是 **2D 先验的固有上限**
（激光扫不到台阶立面的下缘，见 `docs/traversability_plan.md`），**不是本次提升引入的**。

### 3.2 同窗口逐格对照（旧 → 新，`tools/scripts/mapping/map_prior_ab.py`）

把两张图裁到**公共物理窗口** `x∈[-24.80,3.60] y∈[-8.92,6.98]`（分母一致后才可比；两张图原点差
0.4/0.17 m，裁到整格后剩 ~0.5% 的边缘格对不齐，属工具已知量化效应）：

| 指标 | 旧（`bak-20261007`） | 新（`v3_spl`） | 谁好 |
|---|---|---|---|
| 全场台阶边沿被表示 | 49.44% | **54.42%** | **新**（+4.98 pp，多表示 208 个真值边沿格） |
| 可行驶斜面被标占用（总） | **3.29%** | 5.21% | 旧 |
| └ 其中离真值边沿 **≤0.10 m**（= 台阶旁的保守膨胀，**安全方向**） | 1.64% | 2.78% | （不算缺点） |
| └ 其中离真值边沿 **>0.20 m**（= 坡面中段凭空多出来的**真障碍**） | **0.57%（35 格 / 0.09 m²）** | 0.90%（55 格 / 0.14 m²） | 旧，但**差值只有 20 格 = 0.05 m²** |
| 未知格 | 34819 | **24743** | **新**（多"看见" 10076 格 = 25.2 m²） |
| 窗口包含关系 | — | 新窗口 `x[-25.20,3.65] y[-9.09,7.21]` **完全包含**旧窗口（四周都更大） | **新**（没有丢覆盖） |
| 重叠区真值边沿格**本身**被占用（以旧图栅格化为基准，5707 格） | 1118 = 19.6% | **1141 = 20.0%** | 新（微幅） |
| 同上（以新图栅格化为基准，4970 格） | 1147 = 23.1% | **1202 = 24.2%** | 新（微幅） |
| 各自窗口下真值边沿格本身被占用 | 1118 | **1301** | **新**（+16%） |
| 重叠区逐格（179,739 格） | — | 两图都自由 73.51%；都未知 12.23%；都占用 2.43% | — |
| └ 旧**自由**→新**占用** | — | 3193 格（1.78%）；其中 43.1% 落在真值边沿 0.10 m 内（= 该补的占用） | — |
| └ 旧**占用**→新**自由** | — | 4220 格（2.35%）；其中 38.5% 落在真值边沿 0.10 m 内（= 这一部分是**真损失**，其余是幽灵墙） | — |
| 旧图自由格在新图里 | — | 96.65% 仍自由；2.34% 变占用；1.02% 变未知 | — |
| 旧图占用格在新图里 | — | 46.4% 变自由（30.3% 真值是平地 ⇒ 抹掉的是幽灵墙） | — |

**净账**：新图在"真值边沿被表示"和"看不见的面积"两项上明显更好，撞击窗从 85% 变 100%（这正是
`verify_low_terrain.py` 当初为"坡脚撞车"造出来的判据）；代价是**坡面中段多了 20 格（0.05 m²）假障碍**、
以及两图互换时各有 3~4 千格占用/自由翻转（两次独立建图的正常噪声，双向都有，净额偏向新图）。

### 3.3 3D 先验对照（GICP，两把尺子）

| 尺子 | `PCD/RMUC2026.pcd.bak-20261007`（旧 = v2 云） | `PCD/RMUC2026.pcd`（新 = v3 云） | 谁好 |
|---|---|---|---|
| 点数 / 文件 | 178,543 / 5.71 MB | 203,352 / 6.51 MB | 新（+14%） |
| `xy` 足迹 / bbox | 415.6 m²；`x[-25.88,4.33] y[-9.75,8.66]` | 449.3 m²；`x[-26.73,4.49] y[-12.73,9.64]` | 新（+8% 覆盖） |
| 离线探针 fitness（6 帧真实点云、6 个真值位姿、`seed=0`） | **均值 0.00207 m²** / max 0.00255 | 均值 0.00217 m² / **max 0.00228** | 几乎并列（差 5%，在帧间起伏之内） |
| 离线探针 `\|残差 xy\|` | 中位 0.023 m / max 0.106 m | 中位 0.027 m / **max 0.032 m** | 并列（新图无离群帧） |
| 离线探针 竖直残差 | **+0.162 m**（系统性：扫描要被抬 16 cm 才贴合该图） | **-0.014 m**（1.4 cm 内） | **新**（与实时链路/世界几何一致） |
| 整栈无头（`mode:=nav localization:=gicp`，各自一次跑，停车静止） | 采纳 **763/764 = 99.9%**；`score` p50 **0.00290** / p95 0.00298 / max 0.00301 m²（n=514）；`map→odom (0.13,-0.02,-0.10)` | 采纳 **703/703 = 100%**、`拒绝 0`；`score` p50 **0.00272** / p95 0.00277 / max **0.00278** m²（n=517）；`map→odom (-0.064,-0.056,-0.076)` | 新（score −6.2%、更稳、修正量更小） |
| 整栈无头：静止时的定位误差（对 `odom_ground_truth`，**单样本**） | 0.072 m | 0.118 m | 旧（见 §3.4 的诚实说明） |

> 节点阈值 `max_fitness_score = 0.3 m²`，两者都远在阈值内；这个场地的先验**本身自相似**
> （`docs/algorithm_matrix.md` §四：偏 0.07~2.5 m 时 fitness 仍只有 0.002~0.10 m²），
> 所以**不要**只看 fitness 一个数——上表的"残差/覆盖率/一致性"才是分辨力所在。

### 3.4 结论：为什么提升的是 `v3_spl` + `v3`

1. 2D：边沿表示 +5 pp、撞击窗 100% vs 85%、未知面积 −25 m²、窗口严格包含旧窗口、重叠区真值边沿被占用 +12%；
   唯一代价是 0.05 m² 级别的坡面假障碍（20 格）——**这是"更敢在台阶旁标占用"的保守方向**。
2. 3D：fitness 与旧份并列（±5%），但**覆盖率 +8%**、**竖直一致性从 16 cm 偏到 1.4 cm**、
   整栈 `map→odom` 修正量更小（0.04 m vs 0.13 m），且与提升后的 2D 图**同一次会话**（同源）。
3. 旧份没有丢：三个 `.bak-20261007` 文件就是它（§4.4 一条命令回滚）。

**诚实说明（本轮唯一对新图不利的数）**：整栈无头跑停车静止时，"发布出去的位姿"对真值的 xy 误差是
**旧 0.072 m / 新 0.118 m**（各一个样本）。但同一件事在**控制得更好的离线探针**上（6 帧、同一套真值位姿、
同一 `seed`）结论相反：新图平均偏 **0.0247 m**、最大 0.032 m；旧图平均 **0.0343 m**、最大 **0.106 m**
（旧图那一帧 0.106 m 是离群）。考虑到这个场地先验**本身自相似**（`docs/algorithm_matrix.md` §四：
偏 0.07~2.5 m 时 fitness 仍只有 0.002~0.10 m²），单个静止样本的 4 cm 差异**不足以推翻其它五项证据**，
故仍提升；这一条列入 §7.2 未验证项，**下一次跑长距离导航时值得复核**（A/B 方法：`--map-yaml` 指到
`map/RMUC2026.yaml.bak-20261007` 拷贝 + `PCD/RMUC2026.pcd.bak-20261007` 换回，见 §4.4）。

---

## 4. 以后怎么换图（promote 流程）

### 4.1 先测量，再决定（不要直接覆盖）

```bash
# ① 2D 候选：同一口径验收（真值 = RMUC2026.stl）
python3 tools/scripts/regress/verify_low_terrain.py --map <候选>.yaml --label new \
    --json .tmp_mapassets/verify_new.json --png .tmp_mapassets/verify_new.png
# ② 与当前默认逐格对照（公共窗口 / 翻转格 / 边沿分桶 / 判据表）
python3 tools/scripts/mapping/map_prior_ab.py <当前默认>.yaml <候选>.yaml \
    --label-a canon --label-b new
#    ⚠ 默认那份的 yaml 是相对路径（image: RMUC2026.pgm）⇒ 想拿 .bak 做 A/B 时，
#      先把 .bak 那一对拷到一个临时目录再指过去（本次就是用 .tmp_mapassets/old_pair/ 做的）
# ③ 3D 候选：不换文件就能比 —— 离线探针（需要一帧真实点云；见 §3.3 怎么录）
install/gicp_registration/lib/gicp_registration/gicp_selfsim_probe \
    --pcd <候选>.pcd --scan <真实帧>.pcd --pose <x> <y> <yaw> --axis x --sweep 0:0:1
# ④ 最后再用整栈无头跑复核（隔离！见 §4.3）
```

### 4.2 覆盖（**就地**，先备份）

```bash
cd src/rm_nav_bringup
D=$(date +%Y%m%d)
cp -p map/RMUC2026.pgm  map/RMUC2026.pgm.bak-$D      # ① 备份（三条都要）
cp -p map/RMUC2026.yaml map/RMUC2026.yaml.bak-$D
cp -p PCD/RMUC2026.pcd  PCD/RMUC2026.pcd.bak-$D
md5sum map/RMUC2026.pgm.bak-$D PCD/RMUC2026.pcd.bak-$D   # ② 核对备份 == 提升前
cp -f <候选>.pgm map/RMUC2026.pgm                     # ③ 就地覆盖（不要 rm！）
printf 'image: RMUC2026.pgm\nmode: trinary\nresolution: 0.05\norigin: [<x>, <y>, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25' > map/RMUC2026.yaml
cp -f <候选>.pcd PCD/RMUC2026.pcd
ls -l map/RMUC2026.pgm map/RMUC2026.yaml PCD/RMUC2026.pcd   # ④ 三个都必须存在
```

> `yaml` 里只有 `image:` 与 `origin:` 允许变；`mode/resolution/negate/thresholds` 保持
> `trinary / 0.05 / 0 / 0.65 / 0.25`（与 `map_saver_cli`、`pcd_to_nav2_map.py` 的输出口径一致）。
> `origin` 必须与 pgm 的窗口自洽（`origin = 左下角在 map 系`；map 系 = 出生点相对系）。

### 4.3 整栈复核（隔离跑法，和本仓既有 A/B 一致）

用仓库现成的跑法即可（它在**自己那一条 bash 调用**里起栈 + 记录 + 收尾，隔离与本仓既有 A/B 完全一致）：

```bash
# 只录静止基线、不发目标（最省事的"换了图还能不能起来 + gicp 读数正不正常"）
bash tools/scripts/diag/run_nav_lio_timeline.sh --tag promoted --no-goal \
     --domain 95 --port 11595 --map-yaml "$PWD/src/rm_nav_bringup/map/RMUC2026.yaml"
# 想连导航一起验：把 --no-goal 去掉，加 --goal X Y（默认 = 用户那次撞坡脚的目标 -12.64 -0.31）
```

环境 = `HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、独立 `GAZEBO_MASTER_URI`、`unset DISPLAY`、
`nav_rviz:=False spin_speed:=0.0`，收尾发零 Twist 再杀进程组（细节见 `docs/map_assets.md` 引用的脚本头）。
本次实际用的是**临时脚本** `.tmp_mapassets/nav_pair_check.sh`（测试目录、不入库；等价于上面这条，
额外多打印了下面第 1、2 项证据）。要看的四项证据：

1. `map_server yaml_filename` = `<share>/map/RMUC2026.yaml`，且 `Read map …: W X H map @ 0.05 m/cell`；
2. `[status] … target=<点数> 点（leaf 0.100）` —— 证明 gicp 读的是新 PCD；
3. `采纳 N/M 帧`、`score=… m²`（判据：`score` 与提升前同量级、采纳率 100%）；
4. 没有 `[ERROR]`（`gzclient` 在无头下本来就会退，属正常）。

### 4.4 回滚（一条命令）

```bash
cd src/rm_nav_bringup
cp -f map/RMUC2026.pgm.bak-<日期>  map/RMUC2026.pgm
cp -f map/RMUC2026.yaml.bak-<日期> map/RMUC2026.yaml
cp -f PCD/RMUC2026.pcd.bak-<日期>  PCD/RMUC2026.pcd
md5sum map/RMUC2026.pgm map/RMUC2026.yaml PCD/RMUC2026.pcd   # 与 §4.4 记录的 md5 对得上即回滚成功
```

> 本次提升的回滚目标：`map/RMUC2026.pgm.bak-20261007`（`md5 93021a5ec6463cfd7c683f94ef4b4c01`）、
> `map/RMUC2026.yaml.bak-20261007`（`md5 aa71fa71f3ef2ae5f286c4e733e80d43`，
> `origin [-24.8,-8.92,0]`）、`PCD/RMUC2026.pcd.bak-20261007`（`md5 9db987a4ea83d4ff4227711ebedf9cc7`）。
> **提升后的当前值（回滚前/后对照用）**：`map/RMUC2026.pgm` = `md5 82e1742b54e2cf3e33ac8891ca90f7f5`
> （= `RMUC2026_v3_spl.pgm`）、`map/RMUC2026.yaml` = `md5 683b07a83120957aad1cbbd69d76bf43`
> （`origin [-25.2,-9.09,0]`）、`PCD/RMUC2026.pcd` = `md5 5b494a6016a6deb3276b2a6fb48ed1d1`（= `RMUC2026_v3.pcd`）。
> `map_archive.sh` 管的是**会话存档**（`<name>.posegraph/.data/.pcd`）的 `.prev-<ts>`，
> 与这里 `.bak-<日期>` 这一层**互不干涉**：前者是"续建/重存"的回滚，后者是"换默认先验"的回滚。

---

## 5. `install/` 树与"就地覆盖"约定

`colcon build --symlink-install` ⇒ `install/rm_nav_bringup/share/rm_nav_bringup/{map,PCD}/` 里
**每个文件一条软链**指向源码树。于是：

* **覆盖源码树里的同名文件 = 立刻生效**（本次提升没有重编，`ros2 param get /map_server yaml_filename`
  实测就指向 `install/.../map/RMUC2026.yaml`，`Read map …: 577 X 326`）；
* **一旦 `rm` + 重建同名文件，`install/` 那条软链就悬空**（launch 会报找不到图）；
  所以本文件与 §4.2 一律要求 `cp -f` 就地覆盖；
* `install/share/.../map|PCD/` 里目前还有 **6 条历史遗留的悬空软链**
  （`map/RMUC2026.{meta.yaml,posegraph,data,pgm.prior,yaml.prior}`、`PCD/RMUC2026.meta.yaml`）
  ——它们指向源码树里早已改名/移走的文件（如 `RMUC2026_good.*`）。本次**没动**它们
  （重编 `rm_nav_bringup` 会自动清掉；不动就能保证不影响在跑的会话）。
  本次搬 `attic/` 时被**顺带孤立**的 3 条（`map/RMUC2026.pgm.synth.bak`、`map/RMUC2026.yaml.synth.bak`、
  `PCD/RMUC2026.pcd.synth.bak`）已经**当场删掉那 3 条软链本身**（只删悬空链接，源码树里的文件是搬走而非删除）。

---

## 6. 已移动 / 保留（本次 tidy 的完整账）

**移入 `map/attic/`**（只搬不删；`map/attic/README.md` 有逐条说明）：
`RMUC2026.pgm.synth.bak`、`RMUC2026.yaml.synth.bak`、`good_map_preview.png`

**移入 `PCD/attic/`**：`RMUC2026.pcd.synth.bak`

**原地保留（有引用，或属于"存档/回滚链"这两个正规类别）**：其余全部 —— 逐个见 §2。
特别地：

* `*.prev-*`（`map/RMUC2026_v3.*` 两代、`PCD/RMUC2026_v3.*` 一代、`PCD/RMUC2026_v2.*` 一代）
  **一律留在原位**：`map_archive.sh restore/backups` 靠同目录通配找代，搬走等于砸掉回滚；
  且"保留最新 2~3 代"这条规矩本来就满足（没有更老的世代需要搬）。
* `map/RMUC2026_cont.*` / `PCD/RMUC2026_cont.*`（agent 测试存档）虽然"没用了"，
  但被 `docs/continue_mapping.md` / `docs/lio_drift_diagnosis.md` 多处以**路径**引用 ⇒ 留在原位，
  并在 §2 标注"若要清理，整组搬 `attic/`"（本次不动，避免让文档指向不存在的路径）。
* `map/RMUC2026_cloud.*` 虽然已被否决，但它是 `docs/mapping_2d_from_cloud.md` §11 那次
  A/B 的对照物 ⇒ 留在原位。

---

## 7. 入库策略（tracked / untracked）与未验证清单

### 7.1 入库策略

* **入库**：两个默认先验（`map/RMUC2026.pgm|yaml`、`PCD/RMUC2026.pcd`）、
  各 world 的 `map/<world>.pgm|yaml`、`PCD/<world>.pcd`、以及 `RMUC.* / RMUL.*` 那批历史资产
  （仓库**早就在跟踪**这几类二进制，本次只改已有路径的内容，不新增重量级二进制）；
  文档、`tools/`、`launch/`、小体积 sidecar（`.meta.yaml`、`*.yaml`）。
* **不入库**：会话存档（`*_<tag>.{posegraph,data}`，几十 MB）、`*.prev-*` 回滚代、
  `*.bak-*` / `*.synth.bak` / `attic/` 里的退役件、`.session.yaml`（`map/.gitignore` 已忽略）、
  以及本来就不该进的 `build/ install/ log/ .tmp_*/`。
* 依据：`docs/continue_mapping.md` §7「本次实测留下的存档**没有入库**（untracked）」这条既有惯例。
* ⚠ 本仓库**没有**为 `map/*.posegraph` / `PCD/*.pcd` 写通配 ignore（`map/.gitignore` 只忽略
  `.session.yaml`）⇒ 用 `git add -A` 时很容易把几十 MB 的存档带进去。**加档前先 `git status` 看清**。

### 7.2 未验证 / 已知限制

| 项 | 说明 |
|---|---|
| `RMUC2026_v3_spl.pgm` 的**确切生成命令** | 没找到当时的日志/脚本记录。能确定的是：它**不是** `pcd_to_nav2_map.py` 对 `PCD/RMUC2026_v3.pcd` 的投影（后者实测 68% 占用、0 未知，与本图 5.2% 占用、17% 未知完全不同），特征与 slam_toolbox 的 `/map`（`/scan` 累积 + 射线清除）一致；时间上紧接 `RMUC2026_v3` 建图会话。**列为未验证项。** |
| 2D 图的"全场边沿被表示"仍是 ~53% | 两张候选都够不到 80% 判据；这是 2D 表示的固有上限（`docs/traversability_plan.md`）。本次只是"选更好的那张"，**没有解决**这个上限。 |
| `RMUC2026_spl` / `RMUC2026_cloud` 的路径可达性 | 本次只跑了 `verify_low_terrain` 的图面判据，没跑 `--collision` 路径那一段的对比（它们本来就不是默认图）。 |
| 其它 world（RMUC / RMUL / RMUL2026） | 未重测、未整理（`RMUL2026_world_backup.pgm|yaml` 是"旧图备份"，有 4 处文档/源码引用 ⇒ 本次不动，留作以后整理）。 |
| 整栈无头复核的时长 | 两次跑（旧对 96 / 新对 95）都是**停车静止**基线，没有发导航目标；`score` 的对比因此是"同一位置、同一姿态"的干净对照，但**没有**覆盖"跑起来之后的定位跟踪"。 |
| `PCD/RMUC2026.pcd` 的新旧差异只在**出生点附近**验证充分 | 6 帧点云全部取自出生点静止段（真值位姿已知）；场地其它位置的对照没有做（缺真值位姿的实时数据）。 |
| **整栈静止时的定位误差**（对新图不利的那一项） | 旧 0.072 m / 新 0.118 m（**各一个样本**）。离线探针 6 帧的结论相反（新 0.0247 m 均值 vs 旧 0.0343 m）。⇒ 建议在**下一次长距离导航**时复核（方法：`--map-yaml` 指到旧图 + 把 `PCD/RMUC2026.pcd.bak-20261007` 换回去跑一次，见 §4.4）。 |
| 本次的测试产物 | 全在 `.tmp_mapassets/`（**测试目录、不入库**）：`verify_*.json/png`、`probe_ab.txt`、`canon_f0*.pcd`（6 帧真实点云，供复跑探针）、`canon.*` / `promoted.*`（两次整栈无头的日志与 `gicp_nav_watch` 记录）、`old_pair/`（旧图那一对的临时拷贝，用于 A/B）。|
