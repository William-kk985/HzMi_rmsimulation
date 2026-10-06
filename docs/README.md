# `docs/` 文档索引（唯一入口）

> **先看这里**：本文件是 `docs/` 的总入口。每条给出「一句话定位 + 状态 + 什么时候看」。
> 约定：**不移动既有文件**（移动会破坏跨文档引用与外部链接），只在本文件与各子目录 `README.md` 里标注状态与主次。
>
> **调试记录命名约定**：按"**哪一套方案**"一份，文件名 = `debug_<lio>_<mapper>[_<mode>].md`，
> 例：`debug_fastlio_cartographer.md`（`lio:=fastlio` + `mapper:=cartographer`，默认 `mode:=mapping`）、
> 以后换方案就照此新增 `debug_pointlio_slam_toolbox.md` / `debug_fastlio_icp.md` …
> 内容固定包含：**启动命令 + 这套链路的节点/话题/文件清单 + 实测基线数字 + 症状→判据→处置 + 踩坑清单 + 未决项**，
> 见 `debug_fastlio_cartographer.md` §0.1 的模板。

---

## 快速入口（按你现在的目的找）

| 我想… | 看这份 | 跳到 |
|---|---|---|
| 把仿真跑起来、判断对不对 | `smoke_test_runbook.md` | §0.9 五分钟上手 → §9.1 就绪检查 |
| 知道"现在有哪些算法、能不能跑" | `algorithm_matrix.md` | §一 槽位 / §三 资产 / §四 实测状态 |
| 理解"为什么这样分层、为什么降维" | `architecture.md` | §3.2.1–§3.2.7 |
| 查某个名词是什么意思 | `glossary.md` | 按主题四组检索 |
| 排查某个诡异现象 | `issues_and_findings.md` + `smoke_test_runbook.md` §10.1 | 「现象 → 归属」表 |
| 改感知/代价图层、换 3D→2D 做法 | `3d_to_2d_survey.md` | §二 两种降维哲学 / §三 共性旋钮 / §七 本工程分布 |
| 挑算法（选型池） | `rm_algorithm_catalog.md` | §〇.0 前端 vs 系统 / 各章候选 |
| 改 TF/话题契约 | `tf_interface_contract.md` | T1–T5 已实施、T6 撤销 |
| 判断"这个改动会不会上实车 / 是不是为仿真妥协" | `sim_real_contract.md` | §一 分界点与替身 / §四 三分类判据 / §五 禁止清单 |
| 地图跟着车转 / 建图丢帧 / SLAM 不稳，想一步步查 | `debug_fastlio_cartographer.md` | §1 分层判读法 → §4 症状速查表 → §6 踩过的坑 |
| 换/加**里程计**（`lio` 槽）、想知道某个 LIO 吃什么话题什么时间戳 | `lio_slots.md` | §0 一句话结论 / §2 契约表 / §4.1 **launch 的"缺包不再挡住所有组合"修复** / §5 节点级验证 + **§5.9 全栈 A/B 实测** / §7 跑起来；**现状**：`small_point_lio` = **离线已调参（30.2 m → 6.9 m）+ 全栈已跑通**（PASS/SUCCEEDED，与 `lio:=fastlio` 同命令对照逐项同量级），⚠️ 但**全栈逐点精度、`spin_speed≠0`、其它场地/重定位槽组合仍未验**；`pointlio` 从未全栈验证（`algorithm_matrix.md` §一/§四） |

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
| `stvl_local_costmap.md` | **局部代价地图也上 STVL**（`local_obstacle:=stvl`）：槽位/机制、参数来源（COD `singlenav2_params.yaml:200-262`）、A/B 四次跑、代价与未验证清单 | 槽位取值、参数、实测数字 |
| `tf_interface_contract.md` | TF / 接口契约（**T1–T5 已实施，T6 撤销**） | 帧树与话题契约 |
| `params_ownership_checklist.md` | 参数归属清单（R1–R5 逐项状态） | 参数该放哪个包 |
| `sim_real_contract.md` | **仿真 ↔ 实车契约**：分界点/替身清单、共用接口、参数分家、三分类判据、禁止清单、验收判据 | **"改动会不会上实车 / 该不该为仿真动算法"的唯一真值** |
| `debug_fastlio_cartographer.md` | **调试手册（针对 `mode:=mapping`+`lio:=fastlio`+`mapper:=cartographer` 这套链路，§0.1 有范围声明）**：分层判读法（L0–L6）、诊断工具、实测基线数字、症状→判据→处置速查表、踩坑清单；**§5.2.4 = "留不住"的离线复现结论与参数扫描表** | **"SLAM/建图异常怎么查"的方法论真值** |
| `cartographer_2d_occupancy_semantics.md` | **cartographer 2D 占据栅格语义逐行考据**（2.0.9004/2.0.9002，与上游 master 逐字节相同）：命中/清除写入顺序与"同周期先写者胜"、`missing_data_ray_length` 只在超距回波分支生效、`kMin/kMaxProbability=0.1/0.9`、LaserScan 两级过滤（`inf` 被丢弃）、**`/map` 取值上限 75 的完整推导**（含 pycairo/C++ 两路复算） | 改 cartographer 参数前的**源码级依据**；§5.2.4 与 `cartographer.lua` 注释都引用它 |
| `mapping_small_point_lio.md` | **`lio:=small_point_lio` 的建图链**：接线审计（file:line + 运行时 `/map` 单发布者证据）/ 3D 先验怎么导（`/map_save` 存在、`save_pcd` 必须构造期打开、落盘路径硬编码、`/cloud_registered` 的常值杆臂错位实测）/ RMUC2026 实跑产物（2D 图 + PCD 统计）/ `localization:=gicp` 验证（fitness 0.0017 m²、880/880 帧、PASS）/ A/B / 回滚 / 未验证 | **small_point_lio 建图与产物的唯一真值**（含「仿真底盘会在窄口硬卡」等实测坑） |
| `mapping_2d_from_cloud.md` | **2D 先验图改从 3D 点云投影**（+ 建图稳健性 A/B）：`/scan` 丢帧量化与 四个变体 A/B（根因 = slam_toolbox `scan_queue_size=1`）/ `map→odom` 跳变哨兵（alert-only 的理由 + 实测抓到的跳变）/ 坡道段（23° 坡 + 1.05 m 无顶棚走廊）同时间轴证据与判定 / `pcd_to_nav2_map.py` 的判据与用法 / 两张 2D 先验的区域计数 A/B + 回归 / 跑图规程 / 回滚 / 未验证 | **「2D 图从哪来」与「坡道为什么在 2D 图里消失」的唯一真值** |
| `ground_segmentation_slots.md` | **地面分割槽位 `ground`（`linefit` \| `patchwork`）**：上游/许可证/pinned commit 考证（BSD-2-Clause，pin `3e6903a1` = v1.4.1；上游 `ros/**` 因 `package.xml`(GPL-3.0) 与 `LICENSE`(MIT) 矛盾而**不 vendor**）、与 linefit 的**逐字段契约表**、参数表（每个默认值的来源；`th_dist` 0.125→0.08 的实测理由）、`ground` 槽位 diff 与 `--show-args` 证据、**互斥真值表**（LaunchDescription 求值 + 运行期 `/segmentation/obstacle` 单发布者实证）、离线 A/B（真实帧 + RMUC2026.stl 实测几何代理）、整栈建图 smoke A/B、结论与回退 | **地面分割槽位的唯一真值**（含「22~35° 缓坡：linefit 判障碍 35.7~44.3% → patchwork 2.5~3.0%」与「p2l `max_height 0.1` 砍掉 0.33 m 以上护墙」两条实测量） |
| `worlds.md` | **整场世界资产（`RMUC2026`，由附件 STL 生成）**：资产清单 / mm→m 与 z 抬升的依据 / 出生点怎么选 / 2D 图与先验 PCD 怎么来 / 无头实测（建图·AMCL·GICP）/ 场地对本车的坑 + `tools/scripts/world/stl_to_world.py` 一条命令复跑（§4.4） | **场地世界资产与生成口径的唯一真值**（含"可通行 ≈176 m²"等实测量） |

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
