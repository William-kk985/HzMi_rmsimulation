# `docs/package/` 离线配置包专题（索引）

> 用途：把这套仿真系统**打包分发给别人**（不依赖 Git/Gitee）。
> **2026-10 合并**：原来 4 份同主题说明（`HZMI_RMSIMULATION_CONFIG_GUIDE` / `配置包使用说明` / `如何分发配置包` /
> `README_PACKAGE`）已**合并为一份**，原文 `git mv` 到 [`../archive/package/`](../archive/package/)（保留历史 + 顶部 `STATUS`）。

| 文件 | 内容 | 状态 |
|---|---|---|
| **`config_package_guide_merged.md`** | **合并版（唯一入口）**：使用（依赖/编译/运行/参数/地图）+ 分发（打包/排除/MD5/模板）+ FAQ + 检查清单 + §11 2026-10 路径核对 | ★ `现行（合并版）` |
| [`../archive/package/`](../archive/package/) | 4 份原文（`git mv` 保留历史，每份顶部有 `STATUS` 指回合并版） | `历史（已归档）` |

> 相关脚本（**现行位置**）：`tools/scripts/{create_config_package.sh, setup_from_package.sh}`。
> ⚠️ 这两份脚本的 `PROJECT_ROOT` 仍按"脚本在仓库根"计算（`${SCRIPT_DIR}/..`），2026-10 挪到 `tools/scripts/` 后**少一层**
> —— 详见合并版 §11.2（**本轮只记录、未改脚本**）。
