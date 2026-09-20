# `docs/` 文档索引（唯一入口）

> **先看这里**：本文件是 `docs/` 的总入口。每条给出「一句话定位 + 状态 + 什么时候看」。
> 约定：**不移动既有文件**（移动会破坏跨文档引用与外部链接），只在本文件与各子目录 `README.md` 里标注状态与主次。

---

## 快速入口（按你现在的目的找）

| 我想… | 看这份 | 跳到 |
|---|---|---|
| 把仿真跑起来、判断对不对 | `smoke_test_runbook.md` | §0.9 五分钟上手 → §9.1 就绪检查 |
| 知道"现在有哪些算法、能不能跑" | `algorithm_matrix.md` | §一 槽位 / §三 资产 / §四 实测状态 |
| 理解"为什么这样分层、为什么降维" | `architecture.md` | §3.2.1–§3.2.7 |
| 查某个名词是什么意思 | `glossary.md` | 按主题四组检索 |
| 排查某个诡异现象 | `issues_and_findings.md` + `smoke_test_runbook.md` §10.1 | 「现象 → 归属」表 |
| 改感知/代价图层、换 3D→2D 做法 | `3d_to_2d_survey.md` | §二 共性旋钮 / §六 本工程分布 |
| 挑算法（选型池） | `rm_algorithm_catalog.md` | §〇.0 前端 vs 系统 / 各章候选 |
| 改 TF/话题契约 | `tf_interface_contract.md` | T1–T5 已实施、T6 撤销 |

---

## 一、现行核心（维护中，是真值来源）

| 文档 | 定位 | **它是什么的真值来源** |
|---|---|---|
| `smoke_test_runbook.md` | **怎么跑 / 怎么判 / 错了怎么查**（原生命令，无封装脚本） | 操作步骤、判据、错误对照 |
| `algorithm_matrix.md` | **算法组合总表**：槽位 × 实现 × 资产 × 实测状态 × 阻塞 × 扩展位 | **算法现状（含实跑结果）的唯一真值** |
| `architecture.md` | 目录架构 + 分层概念（§3.2.1–3.2.7 是概念长文） | 目录/分层/参数清单 |
| `glossary.md` | 术语表：一句话定义 + 指向详述 | 名词定义 |
| `issues_and_findings.md` | 实际踩到的故障（根因/修复/证据）+ 静默失效坑 + 待办 | 故障根因库 |
| `3d_to_2d_survey.md` | 各家 3D→2D 实现对照（p2l / nav2 / STVL / cartographer / octomap） | 降维机制与参数 |
| `tf_interface_contract.md` | TF / 接口契约（**T1–T5 已实施，T6 撤销**） | 帧树与话题契约 |
| `params_ownership_checklist.md` | 参数归属清单（R1–R5 逐项状态） | 参数该放哪个包 |

## 二、选型与路线（参考）

| 文档 | 定位 | 状态 |
|---|---|---|
| `rm_algorithm_catalog.md` | 候选算法池（2D/3D 建图、重定位、相机；含 §〇.0 前端 vs 系统） | **现行**，加新候选算法时更新 |
| `rm_bench_refactor_plan.md` | 实验台改造路线（M0–M4 设计草案） | **部分完成**：M0/M1 相关项已在 `issues_and_findings.md` 落地；**未完成项看 `algorithm_matrix.md` §五/§六** |

## 三、专题子目录

| 目录 | 内容 | 索引 |
|---|---|---|
| `mapping/` | 建图相关操作指南（含 cartographer pbstream） | `mapping/README.md`（含"多处为状态快照，现行以 runbook §1/§5/§6 为准"的说明） |
| `package/` | 离线配置包（分发/使用）说明 | `package/README.md`（指明主文档与重复版本） |

## 四、历史草案（保留供回溯，**不要当现行依据**）

| 文档 | 说明 |
|---|---|
| `rm_algolab_plan.md` | 宏架构早期规划（2026-08/09 草案）；现行架构看 `architecture.md` |
| `rm_algorithm_overview.md` | 早期"角色 × 一体性 × 场景"总结；**已被 `architecture.md` §3.2.1 与 `algorithm_matrix.md` §一 取代**（保留其推理过程） |
| `mapping/建图操作指南.md`、`mapping/立即开始建图.md` | 写于当时会话的**状态快照**（开头写"✅ 已启动…"）；现行流程看 runbook §1/§1.1 |
| `mapping/快速参考卡片.txt` | 内容其实是**离线配置包**的快速参考（分类归属应为 `package/`，未移动） |

---

## 维护约定

| 变化 | 更新哪份 |
|---|---|
| 跑通一个组合 / 发现阻塞 | `algorithm_matrix.md` §四 / §五 |
| 修好一个 bug | `issues_and_findings.md`（根因/修复/证据）+ `smoke_test_runbook.md` §10（现象表） |
| 新增/替换算法、加装配级开关 | `algorithm_matrix.md` §一（+ §一.1 维度） |
| 改动目录结构、分层、参数语义 | `architecture.md` |
| 新名词 | `glossary.md` |
| 3D→2D 做法/阈值变化 | `3d_to_2d_survey.md` |
| TF 帧树/话题契约变化 | `tf_interface_contract.md` |

> 规则：**同一件事只在一处维护**（例如实测状态只在 `algorithm_matrix.md` §四），其他地方只放指针。
