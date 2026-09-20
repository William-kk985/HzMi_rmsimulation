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
| `CARTOGRA_PBSTREAM_GUIDE.md` | Cartographer `.pbstream` 生成思路 | **需谨慎**：现行做法是"用 cartographer 建图后 `finish_trajectory` + `write_state` 导出"（见 runbook §5 与 `tools/scripts/mapping/generate_cartographer_pbstream.sh`）；该文里的"由 `.png/.yaml` 转换"路线属绕过性尝试，未验证可作为纯定位输入 |
| `快速参考卡片.txt` | 实为**离线配置包**的快速参考卡片 | 分类归属应为 `docs/package/`（未移动，仅说明） |

> 落盘相关脚本：`tools/scripts/mapping/{save_grid_map.sh, save_pcd.sh, generate_cartographer_pbstream.sh}`。
