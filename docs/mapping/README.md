# `docs/mapping/` 建图专题（索引）

> ⚠️ **状态说明**：本目录里的多数文档是**当时那次会话的状态快照**（开头常写"✅ 已启动…"），
> **不是现行操作手册**。现行流程以 `docs/smoke_test_runbook.md` 为准：
> - 建图（slam_toolbox / cartographer）→ runbook **§1**（含落盘三件套 §1.1）
> - 边建图边导航 → runbook **§1.2**
> - cartographer 建图与纯定位 → runbook **§5 / §6**（纯定位需先生成 pbstream，三个场地目前都缺）

| 文件 | 内容 | 状态 |
|---|---|---|
| `建图操作指南.md` | 当时"已启动仿真"的操作快照（遥控建图流程） | 历史快照 → 看 runbook §1 |
| `立即开始建图.md` | 更简版的上手快照 | 历史快照 → 看 runbook §1 / §0.9 |
| `CARTOGRA_PBSTREAM_GUIDE.md` | Cartographer `.pbstream` 生成思路 | **需谨慎**：现行做法是"用 cartographer 建图后 `finish_trajectory` + `write_state` 导出"（见 runbook §5，三条原生 ros2 命令）；该文里的"由 `.png/.yaml` 转换"路线属绕过性尝试，未验证可作为纯定位输入 |
| `快速参考卡片.txt` | 实为**离线配置包**的快速参考卡片 | 分类归属应为 `docs/package/`（未移动，仅说明） |

> 落盘相关脚本：`tools/scripts/mapping/{save_grid_map.sh, save_pcd.sh}`（pbstream 不用脚本，见 runbook §5）。

## 落盘步骤（cartographer 建图，2026-09 现场确认可用）

前提：`mode:=mapping` + `lio:=fastlio` + `mapper:=cartographer` 跑完，RViz 里墙是连续深色线。

```bash
# ① 2D 栅格（nav2/AMCL 用）—— 它订阅 /map，cartographer 正好发在 /map
tools/scripts/mapping/save_grid_map.sh          # = map_saver_cli -f src/rm_nav_bringup/map/RMUL2026

# ② pbstream（cartographer 纯定位用；两条原生服务，见 runbook §5）
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/weicheng/HzMi_rmsimulation/src/rm_nav_bringup/map/RMUL2026.pbstream', include_unfinished_submaps: false}"

# ③ 3D 点云（ICP 纯定位用；需要 LIO 还在跑）
tools/scripts/mapping/save_pcd.sh               # = ros2 service call /map_save std_srvs/srv/Trigger
```

**⚠️ 坐标系必须注意**：cartographer 的 `map` 系是**出生点相对系**（`map→odom ≈ 恒等`），
而现有 `map/RMUL2026.pgm` 是世界系（`origin: [2.68, 0.228]`）——两者相差 ≈`(4.3, 3.35)`（出生点坐标）。
所以：
- 别直接覆盖 `RMUL2026.pgm`：先 `-f .../RMUL2026_carto` 另存，跟旧图比一比；
- 若确定用新图，把 `bringup_sim.launch.py` 里的 `amcl_init_x/y` 从 `4.3/3.35` 改为 **`0.0/0.0`**（新图原点=出生点）；
- 详见 `docs/issues_and_findings.md` §3.0。

**建图参数（本次定稿，见 `cartographer.lua`）**：`hit 0.68 / miss 0.49 / num_range_data 3000 /
min_probability_to_clear 0.80`（最后一项需要 `src/rm_localization/cartographer/` 这份 fork 核心）。

### 落盘踩坑：`Failed to write map ... Magick: Unable to open file (.../RMUL2026.pgm)`

**不是权限问题**（我已实测 `map/RMUL2026.pgm` 是 `-rw-rw-r--` 可写）。真因：`tools/scripts/{mapping,control}/*.sh`
里的 `PROJECT_ROOT` 少算了一层 —— 这些脚本原来在仓库根目录的 `scripts/` 下（`../..` 正好是根），
2026-09 合并进 `tools/scripts/` 后**深度多了一层**，`${SCRIPT_DIR}/../..` 算出来是 `.../HzMi_rmsimulation/tools`，
于是 `map_saver_cli -f src/rm_nav_bringup/map/RMUL2026` 实际指向 `tools/src/rm_nav_bringup/map/...`（不存在）
⇒ ImageMagick 报 `Unable to open file`，而 `map_saver` 只打 `Failed to save the map`。

**已修**（2026-09-23）：`start_sentinel.sh` / `improved_teleop.sh` / `save_pcd.sh` / `save_grid_map.sh`
的 `PROJECT_ROOT` 改成 `${SCRIPT_DIR}/../../..`（`tools/scripts/build.sh` 等深度 2 的脚本本来是对的）。
同样的原因还在 `tools/` 下留下了空的 `build/ install/ log/`（可删）。
