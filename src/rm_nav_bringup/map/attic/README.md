# `map/attic/` —— 退役/凌乱资产的存放处（**只搬不删**）

本目录里的东西**不是**当前默认资产，也不该被 launch / tools / docs 引用。
规矩：**只搬不删**；要恢复某一份，直接 `mv` 回上一级目录并按 `docs/map_assets.md` §4 的流程测量后再用。

建立时间：2026-10-07（`docs/map_assets.md` 首次整理地图资产时）。

| 文件 | 原来在哪 | 它是什么 | 为什么退役 | 怎么复现 |
|---|---|---|---|---|
| `RMUC2026.pgm.synth.bak` | `map/` | 2026-10-05 由 `RMUC2026.stl` **合成**的 2D 先验（600x341 @0.05，`origin [-25.925,-9.425]`），即 `docs/worlds.md` §4.1 描述的那一份 | 合成路线已被"实跑建图"取代（默认先验 2026-10-06 起就是实跑产物，2026-10-07 又提升到 `RMUC2026_v3` 那次会话） | `python3 tools/scripts/world/stl_to_world.py --stl <STL> --world-name RMUC2026 --map-grid -15.0 -6.9 600 341`；或 `git show 280888e:src/rm_nav_bringup/map/RMUC2026.pgm` |
| `RMUC2026.yaml.synth.bak` | `map/` | 上面那张图的 yaml（`image: RMUC2026.pgm`、`origin [-25.925,-9.425,0]`） | 同上 | 同上 |
| `good_map_preview.png` | 仓库根目录 | 旧默认 2D 先验（= `map/RMUC2026_good.pgm`，现 `map/RMUC2026.pgm.bak-20261007`）的渲染预览图，1136x636 | 它预览的那张图 2026-10-07 已被取代；当前默认图的预览见 `docs/img/RMUC2026_prior_20261007.png` | `git log --diff-filter=D -- good_map_preview.png`（原为 untracked，见 `docs/continue_mapping.md` §10 的只读清单） |

⚠ 与它们配套的 `PCD/RMUC2026.pcd.synth.bak`（同一次合成路线的点云）在 `PCD/attic/`。
paired sidecar / 存档 / `*.prev-*` 回滚代**都不在本目录**（那些要留在原位，见 `docs/map_assets.md` §6）。
