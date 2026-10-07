# `PCD/attic/` —— 退役/凌乱资产的存放处（**只搬不删**）

本目录里的东西**不是**当前默认资产，也不该被 launch / tools / docs 引用。
规矩：**只搬不删**；要恢复，`mv` 回上一级目录，并按 `docs/map_assets.md` §4 的流程测量后再用。

建立时间：2026-10-07（`docs/map_assets.md` 首次整理地图资产时）。

| 文件 | 原来在哪 | 它是什么 | 为什么退役 | 怎么复现 |
|---|---|---|---|---|
| `RMUC2026.pcd.synth.bak` | `PCD/` | 2026-10-05 由 `RMUC2026.stl` **合成**的 3D 先验：地面以上薄壳、**26,618 点 / 0.85 MB**、8 字段布局，地面放在 `z=-0.06` 以对齐 `initial_pose=[0,0,0]` 语义 —— 即 `docs/worlds.md` §4.2 描述的那一份 | 合成路线已被"实跑建图"取代（默认 3D 先验 2026-10-06 起是 `cloud_accumulator` 的实跑产物，2026-10-07 提升到 `RMUC2026_v3` 那次会话的 203,352 点云） | `python3 tools/scripts/world/stl_to_world.py --stl <STL> --world-name RMUC2026`；或 `git show 280888e:src/rm_nav_bringup/PCD/RMUC2026.pcd` |

⚠ 与它配套的 2D 合成图 + 旧图预览在 `map/attic/`。
存档（`RMUC2026_<tag>.pcd`）、提升备份（`*.bak-<日期>`）、`*.prev-*` 回滚代**都不在本目录**。
