# `docs/` 文档索引（唯一入口）

> **2026-10 仓库整理**：覆盖**全仓 105 份自有 Markdown**（旧索引漏收 39 份）。每条 = 「链接 + 解决什么问题 + 状态」；上一轮只在 4 份顶部加了一行 `STATUS`。
> **2026-10 合并轮**：3 组重复/未跟踪文档已合并（`docs/package/` 4 份 → 1 份、`docs/mapping/` 2 份 → 1 份、未入库的 cartographer 报告 → 并入 `cartographer_2d_occupancy_semantics.md` §5），原文归档到 `docs/archive/`；`livox_viewer2_launch_notes.md` 已入库。
> 状态四值：`现行` / `历史（结论仍有效）` / `已被取代（见 X）` / `参考（非本仓结论）`。逐份明细见 [`inventory.md`](inventory.md)；不再成立的说法见 [取代表](#取代表不再成立的说法)。

## 先看这三份

| # | 文档 | 为什么先看它 |
|---|---|---|
| 1 | [`round_summary_gz_frames.md`](round_summary_gz_frames.md) | 上一轮（2026-10-18）四条线的合并总结 + **24 条缺口/未验证** —— 先知道"现在能用到哪一步" |
| 2 | [`frame_contract_v3.md`](frame_contract_v3.md) | **现行规范性契约**：三层参数 / 最终 TF 树（无 `odom_grav`）/ 不变式 I1–I7 / 重力契约 E1 / 重定位契约 |
| 3 | [`frame_contract_v3_measurements.md`](frame_contract_v3_measurements.md) | 该契约的**实测账**：13.75 vs 5.75 ms/帧、精度、逐位回归 —— 规范落地要付多少 |

> 3 分钟路径：这三份的 §0/§1 → 要动手看 [`smoke_test_runbook.md`](smoke_test_runbook.md) 与 [`map_assets.md`](map_assets.md) → 要改帧/TF 先读 [`tf_interface_contract.md`](tf_interface_contract.md) 与 [取代表](#取代表不再成立的说法)。

## 代码放哪：`rm_utils` / `tools/` / 功能包

> **2026-10 整理**：运行期「缝缝补补」已集中到 `src/rm_utils/`。
> - **`src/rm_utils/`** = 运行期胶水/诊断（存档守卫、续建体检、会话播报、LIO 外参断言、高度带守卫）；
>   分类表 + 每条的**删除条件**见 [`src/rm_utils/README.md`](../src/rm_utils/README.md)。
> - **`tools/scripts/`** = 离线工具（存档、回放、对账、bench），不进 ROS 图、不装成可执行节点。
> - **功能**（谁发布哪条 TF/话题、算法实现与参数）= **留在各自的功能包**，不搬进 `rm_utils`；
>   例如 `odom_level_tf` 是契约 §4.11 的静态重力边，仍属 `rm_nav_bringup`。
> 旧文档里的 `src/rm_nav_bringup/scripts/<name>.py` 是**历史坐标**；现行路径以
> `src/rm_utils/README.md` 为准（本轮按约束未改 §1–§43 正文）。

## ① 入门 / 总览

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`仓库根 README.md`](../README.md) | 仓库总入口：来源与致谢、话题接口、Docker/原生两种环境、启动示例 | `现行` |
| [`本索引 docs/README.md`](README.md) | 本文件 —— 全仓 md 的唯一索引（含取代表与待决清单） | `现行` |
| [`inventory.md`](inventory.md) | 本清单：逐份文档的行数 / 最后提交 / 一行定位（本索引的明细附表） | `现行` |
| [`round_summary_gz_frames.md`](round_summary_gz_frames.md) | 上一轮（10-18）四条线合并总结：外参守卫 / 机器卫生 / Fortress 后端 / COD 帧对账，含 24 条缺口 | `现行` |
| [`architecture.md`](architecture.md) | 目录结构 + 分层概念：为什么这么切 2D/2.5D/3D、参数在哪 | `现行` |
| [`glossary.md`](glossary.md) | 名词一句话定义 + 指向详述章节 | `现行` |
| [`algorithm_matrix.md`](algorithm_matrix.md) | 槽位 × 实现 × 资产 × 实测状态：算法现状的唯一真值 | `现行` |
| [`algorithm_axes.md`](algorithm_axes.md) | 按"正交轴"索引整个 bench：一次只动一个轴、轴的拓扑与选项 | `现行` |
| [`rm_algorithm_overview.md`](rm_algorithm_overview.md) | 早期"角色 × 一体性 × 两场景"总结（推理过程仍有价值） | `已被取代（见 architecture.md §3.2.1 / algorithm_matrix.md §一）` |
| [`slam_lio_primer.md`](slam_lio_primer.md) | 教材式讲义：KF→EKF→ESKF→预积分→LIO→SLAM→定位，每条公式落到本仓数字 | `现行` |
| [`kalman_slam_tutorial.md`](kalman_slam_tutorial.md) | 循序渐进读本：24 章 + 练习，从"为什么需要滤波"到 SLAM/AMCL/GICP | `现行` |
| [`mapping/README.md`](mapping/README.md) | `docs/mapping/` 子索引：建图快照合并版 + 现行落盘步骤 | `现行` |
| [`package/README.md`](package/README.md) | `docs/package/` 子索引：离线配置包合并版（4 份原文已归档） | `现行` |

## ② 契约 / 规范

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`frame_contract_v3.md`](frame_contract_v3.md) | **规范性**：三层参数、最终 TF 树（无 `odom_grav`）、I1–I7、重力契约 E1、重定位契约 | `现行` |
| [`frame_contract_v3_rationale.md`](frame_contract_v3_rationale.md) | 上述契约的理由记录：为什么只有 E1 让世界系 z 朝上、18 仓库 0 例、8 条自我更正 | `现行` |
| [`tf_interface_contract.md`](tf_interface_contract.md) | TF/话题契约（T1–T5 已实施、T6 撤销）：谁发哪条边、谁供哪个话题 | `现行` |
| [`sim_real_contract.md`](sim_real_contract.md) | 仿真↔实车契约：分界点/替身、三分类判据、禁止清单 | `现行` |
| [`params_ownership_checklist.md`](params_ownership_checklist.md) | 参数归属清单（R1–R5）：算法参数该放哪个包 | `现行` |
| [`odom_base_link_practice_survey.md`](odom_base_link_practice_survey.md) | `odom→base_link` 斜装/云台 15 行开源对照 + §9 重力初始化 18 变体实测（E1 的证据） | `现行` |
| [`lio_extrinsic_application_survey.md`](lio_extrinsic_application_survey.md) | 外参该在哪一步乘一次：四家源码对照 + 本仓两处变换判决 | `现行` |
| [`gravity_alignment_ownership_survey.md`](gravity_alignment_ownership_survey.md) | 重力对齐归谁（IMU/LIO/重定位/感知四层调研） | `已被取代（见 frame_contract_v3.md §2/§4）` |
| [`relocalization_practice_survey.md`](relocalization_practice_survey.md) | 全局重定位 vs 局部跟踪的工业实践 + `map→odom` 到底归谁 | `现行` |
| [`cod_vs_ours_frame_architecture.md`](cod_vs_ours_frame_architecture.md) | COD 实车 vs 本仓帧架构对账：base_link 约定分叉、共轭 bug 为何被掩盖 | `已被取代（§2「C 型」；实车实测见 cod_real_frame_measurement.md、取代表 #11）` |

## ③ 算法 / 原理

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`3d_to_2d_survey.md`](3d_to_2d_survey.md) | 各家 3D→2D 实现对照与两种降维哲学 | `现行` |
| [`mapping_relocalization_survey.md`](mapping_relocalization_survey.md) | 建图算法会不会用重定位、我们缺哪一级（回环/地点识别/多会话锚定） | `现行` |
| [`nav2_vs_custom_planner.md`](nav2_vs_custom_planner.md) | 纯 C++ 规划库接进 nav2 的三种接法、插件契约、效率账与落地坑 | `现行` |
| [`nav_strategy_taxonomy.md`](nav_strategy_taxonomy.md) | 导航策略全图：9 个设计轴 × 81 个族，本仓与 COD 放在同一张地图上 | `参考（非本仓结论）` |
| [`rm_algorithm_catalog.md`](rm_algorithm_catalog.md) | 候选算法池（2D/3D 建图、重定位、相机），含"前端 vs 系统"之辨 | `现行` |
| [`ground_segmentation_slots.md`](ground_segmentation_slots.md) | 地面分割槽位（linefit ↔ Patchwork++）：上游事实、逐字段契约、A/B | `现行` |
| [`lio_slots.md`](lio_slots.md) | 里程计槽位：各 LIO 吃什么话题/时间戳、small_point_lio 接线与实测 | `现行` |
| [`localization_slots.md`](localization_slots.md) | 重定位槽位登记表：一个槽位值一行，选一个就跑 | `现行` |
| [`mapping_small_point_lio.md`](mapping_small_point_lio.md) | `lio:=small_point_lio` 建图链：接线审计、3D 导出、RMUC2026 产物 | `现行` |
| [`gicp_backend_small_gicp.md`](gicp_backend_small_gicp.md) | gicp 两个配准后端（pcl / small_gicp）的对等支持 | `现行` |
| [`stvl_local_costmap.md`](stvl_local_costmap.md) | 局部代价地图上 STVL（`local_obstacle:=stvl`）：机制、参数来源、A/B | `现行` |
| [`slope_speed_limiting.md`](slope_speed_limiting.md) | 前瞻坡度/台阶限速：物理推导 + 整栈 A/B | `现行` |
| [`traversability_plan.md`](traversability_plan.md) | 3D 派生可通行性（坡度/落空/净空）设计计划，未实现 | `现行（plan only）` |
| [`scan_context_plan.md`](scan_context_plan.md) | Scan Context 类全局地点识别接入方案，未落地 | `现行（plan only）` |
| [`cartographer_2d_occupancy_semantics.md`](cartographer_2d_occupancy_semantics.md) | cartographer 2D 占据栅格语义逐行考据（含 `/map` 上限 75 的推导）+ **§5 合并附录**（issue 调查 / TSDF / slam_toolbox / nav2 阈值 / 调参表） | `现行` |
| [`livox_sensor_prior_art.md`](livox_sensor_prior_art.md) | 非重复扫描 Livox 在新 Gazebo 上的既有实现调研 + REUSE/ADAPT/WRITE 裁决 + §10 编译实测 | `现行` |
| [`kinodynamic_esdf_mpc_notes.md`](kinodynamic_esdf_mpc_notes.md) | `865749/Kinodynamic_esdf_mpc` 调研笔记（外部仓库） | `参考（非本仓结论）` |
| [`slam_nav_goal_strategies.md`](slam_nav_goal_strategies.md) | "目标怎么给"四种方式的外部调研 | `参考（非本仓结论）` |
| [`cod_nav_comparison.md`](cod_nav_comparison.md) | COD_NAV 2025 与本仓的逐项对照（两套架构之辨） | `参考（非本仓结论）` |
| [`cod_nav_2026_deep_dive.md`](cod_nav_2026_deep_dive.md) | COD_NAV `rmul2026` 分支深读：逐环节参数 + 可采纳/不采纳 | `参考（非本仓结论）` |
| [`cod_nav_2026_params_review.md`](cod_nav_2026_params_review.md) | cod_nav_2026 逐块参数评审（singlenav2/multiplenav2/approach/CSV） | `参考（非本仓结论）` |
| [`cod_nav_2026_integration_plan.md`](cod_nav_2026_integration_plan.md) | 把 COD(2026) 架构融进本 bench 的规划（未实施） | `历史（结论仍有效）` |
| [`cod_nav_2026_migration_worksheet.md`](cod_nav_2026_migration_worksheet.md) | COD(2026)→本仓的移植工单（只登记，不动手） | `历史（结论仍有效）` |
| [`cod_nav_macro_integration.md`](cod_nav_macro_integration.md) | 宏观融入分析：small_point_lio 溯源、静态 `map→odom` 深挖、能力地图 | `参考（非本仓结论）` |
| [`cod_nav_design_thinking.md`](cod_nav_design_thinking.md) | COD(2026) 思路整理，重点"定位层为什么特殊" | `参考（非本仓结论）` |
| [`cod_nav_nav_stack_inventory.md`](cod_nav_nav_stack_inventory.md) | COD 战队导航开源资料完整清单 | `参考（非本仓结论）` |
| [`cod_nav_open_source_materials.md`](cod_nav_open_source_materials.md) | COD 及兄弟仓库的公开资料清单（哪个真开源、在哪） | `参考（非本仓结论）` |

## ④ 测量 / 实验记录

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`frame_contract_v3_measurements.md`](frame_contract_v3_measurements.md) | 统一输出契约（选项 U）的效率+精度实测：13.75 vs 5.75 ms/帧、逐位回归 | `现行` |
| [`cod_real_frame_measurement.md`](cod_real_frame_measurement.md) | **COD 实车帧实测原文**（2026-10-18，ROS 2 Humble）：`T_bl` rpy `(−46.238°, −1.684°, 0)`/`\|t\| 0.3949 m`、静止 `odom→base_link` roll `0.030°` ⇒ A 型 + 共轭 bug 生效；5 条命令 + 原始输出 + 算术核对 + 7 条 caveat | `现行` |
| [`tilted_lidar_fidelity.md`](tilted_lidar_fidelity.md) | 雷达斜 30° 的逐帧点云账本 + 代价图归因 + bug①② 的判决性签名 | `现行` |
| [`lio_drift_diagnosis.md`](lio_drift_diagnosis.md) | LIO 漂移分段诊断："才跑一会儿就飘"是否合理（唯一复现 = 高角速度甩转） | `现行` |
| [`lio_divergence_no_impact.md`](lio_divergence_no_impact.md) | "没撞击里程计自己飘"的整栈归因 + 限速 A/B | `现行` |
| [`path_clearance_and_contact.md`](path_clearance_and_contact.md) | 贴膨胀边→撞进去→定位跟着坏：实测、归因、修复 | `现行` |
| [`gicp_divergence_and_jitter.md`](gicp_divergence_and_jitter.md) | GICP 重定位"发散 + 抖动"的复现、机制与修复 | `现行` |
| [`gicp_initialpose_latency.md`](gicp_initialpose_latency.md) | `/initialpose` 的延迟/饥饿问题：修复设计与实测 | `现行` |
| [`slam_drops_loopclosure_odom.md`](slam_drops_loopclosure_odom.md) | 建图丢帧 × 回环 × 里程计：两个假设的实测裁决 | `现行` |
| [`slam_toolbox_scan_drops.md`](slam_toolbox_scan_drops.md) | `/scan` 丢帧（MessageFilter queue full）的根因考证与 `scan_queue_size` A/B | `现行` |
| [`slam_toolbox_tuning.md`](slam_toolbox_tuning.md) | slam_toolbox 建图漂移调参：上游语义考证 + 本机 A/B | `现行` |
| [`mapping_2d_from_cloud.md`](mapping_2d_from_cloud.md) | 2D 先验图改从 3D 点云投影 + 建图稳健性 A/B（坡道段判定） | `现行` |
| [`scan_context_probe_report.md`](scan_context_probe_report.md) | A1 资产可行性探针：Scan Context 路线的准入判定 | `现行` |
| [`scan_context_samesource_gate_report.md`](scan_context_samesource_gate_report.md) | Scan Context 同源性硬门（真 `/scan` → PCD 库）实测 | `现行` |
| [`gazebo_sim_migration_plan.md`](gazebo_sim_migration_plan.md) | Classic→新 Gazebo 迁移 spike + §9 纯 CPU 射线 BVH 实测（含单位 bug 修正） | `历史（结论仍有效）` |
| [`livox_viewer2_launch_notes.md`](livox_viewer2_launch_notes.md) | LivoxViewer2 在 22.04 启动即崩的取证 + ROS2 驱动替代路线 | `现行` |

## ⑤ 历史 / 审计

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`session_digest.md`](session_digest.md) | 2026-10-05~08（161 提交）的总汇总 | `历史（结论仍有效）` |
| [`timestamp_construction_audit.md`](timestamp_construction_audit.md) | 全仓时间戳构造审计：1 ns 截断类 bug 及其它戳/队列/QoS 风险 | `历史（结论仍有效）` |
| [`build_optimization.md`](build_optimization.md) | 构建配置审计（colcon 默认不设 `CMAKE_BUILD_TYPE`）与实测 | `历史（结论仍有效）` |
| [`research_p2l_scan_stall.md`](research_p2l_scan_stall.md) | 旧链（livox→linefit→p2l）`/scan` 永久停发的调研（两条本仓缺陷） | `历史（结论仍有效）` |
| [`rm_algolab_plan.md`](rm_algolab_plan.md) | 宏架构早期规划（2026-08/09 草案） | `已被取代（见 architecture.md）` |
| [`rm_bench_refactor_plan.md`](rm_bench_refactor_plan.md) | 实验台改造路线 M0–M4 设计草案 | `历史（结论仍有效）` |
| [`mapping_session_snapshots_merged.md`](mapping/mapping_session_snapshots_merged.md) | **合并版**：同一会话两份建图快照（完整步骤 + 立即开始）→ 键位/路线/观察/排错/pbstream 保存 + §9 与现行流程差异 | `历史（结论仍有效；操作细节为 2026-09 快照）` |
| [`docs/archive/mapping/`](archive/mapping/) | 已合并归档：`建图操作指南.md`（325 行）+ `立即开始建图.md`（115 行），`git mv` 保留历史 + 顶部 `STATUS` | `历史（已归档）` |
| [`CARTOGRA_PBSTREAM_GUIDE.md`](mapping/CARTOGRA_PBSTREAM_GUIDE.md) | `.pbstream` 生成指南（含"需谨慎对待"的旧做法） | `历史（结论仍有效）` |
| [`config_package_guide_merged.md`](package/config_package_guide_merged.md) | **合并版**：离线配置包使用（依赖/编译/运行/参数/地图）+ 分发（打包/排除/MD5/模板）+ FAQ + 检查清单 + §11 2026-10 路径核对 | `历史（结论仍有效；v1.0.0 体系）` |
| [`docs/archive/package/`](archive/package/) | 已合并归档：`HZMI_RMSIMULATION_CONFIG_GUIDE.md` / `配置包使用说明.md` / `如何分发配置包.md` / `README_PACKAGE.md`，`git mv` 保留历史 + 顶部 `STATUS` | `历史（已归档）` |
| [`pbstream_生成完成报告.md`](../archive/pbstream_生成完成报告.md) | pbstream 生成成功报告（2026-09 归档） | `历史（结论仍有效）` |
| [`README.md`](../archive/reality_frozen/README.md) | 真车配置冻结快照说明（R5） | `历史（结论仍有效）` |
| [`FROZEN.md`](../archive/reality_frozen/FROZEN.md) | 该目录"已冻结"的六行声明 | `历史（结论仍有效）` |

## ⑥ 资产 / 流程

| 文档 | 看它解决什么问题 | 状态 |
|---|---|---|
| [`smoke_test_runbook.md`](smoke_test_runbook.md) | 怎么跑 / 怎么判 / 错了怎么查（原生命令，无封装脚本） | `现行` |
| [`map_assets.md`](map_assets.md) | `map/` 与 `PCD/` 里哪一份是当前默认资产 + 换图/回滚/清理 | `现行` |
| [`worlds.md`](worlds.md) | 场地世界资产（RMUC2026 由 STL 生成）：来源、出生点、坑与复跑脚本 | `现行` |
| [`robot_models.md`](robot_models.md) | 机器人模型槽位 `robot`：provenance、几何、安装决策、新旧对照、回滚 | `现行` |
| [`fortress_sim_backend.md`](fortress_sim_backend.md) | `sim_backend:=gz`（Fortress）后端的 opt-in 用法与验证 | `现行` |
| [`machine_hygiene_runbook.md`](machine_hygiene_runbook.md) | swap / zram / systemd-oomd 与收尾作用域的机器卫生 runbook | `现行` |
| [`gazebo_gui_troubleshooting.md`](gazebo_gui_troubleshooting.md) | gzclient 加载不出来的取证、机理与修法 | `现行` |
| [`debug_fastlio_cartographer.md`](debug_fastlio_cartographer.md) | 建图链路调试手册：分层判读 L0–L6 + 症状速查 + 踩坑 | `现行` |
| [`continue_mapping.md`](continue_mapping.md) | 在上次基础上续建 + 存档 + 场地隔离的操作流程 | `现行` |
| [`issues_and_findings.md`](issues_and_findings.md) | 实际踩到的故障根因库 + 静默失效坑 + 待办 | `现行` |
| [`README.md`](../tools/README.md) | 工具脚本根目录索引 | `现行` |
| [`ATTRIBUTION.md`](../tools/scripts/gzsim/ATTRIBUTION.md) | gzsim 后端的上游 pin 与许可证/署名 | `现行` |
| [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) | 第三方组件署名与许可证清单 | `现行` |
| [`README.md`](../third_party/README.md) | `third_party/` 的"只读对照"规则与当前内容 | `现行` |
| [`map/attic/README.md`](../src/rm_nav_bringup/map/attic/README.md) | `map/attic/` 退役资产规则（只搬不删） | `现行` |
| [`PCD/attic/README.md`](../src/rm_nav_bringup/PCD/attic/README.md) | `PCD/attic/` 退役资产规则（只搬不删） | `现行` |
| [`geometry.md`](../src/rm_simulation/robot11_description/inventory/geometry.md) | robot11 几何量测（逐 link 包围盒、地面/雷达高度），自动生成 | `现行` |
| [`mesh_inventory.md`](../src/rm_simulation/robot11_description/inventory/mesh_inventory.md) | robot11 mesh 清单（自动生成，勿手改） | `现行` |
| [`rm_livox_gz_sim/doc/README.md`](../src/rm_simulation/rm_livox_gz_sim/doc/README.md) | `sim_backend:=gz` 的 opt-in 后端资产用法与编译步骤 | `现行` |
| [`small_point_lio/README.md`](../src/rm_localization/small_point_lio/README.md) | vendored small_point_lio 的上游 README（含本仓联系方式补丁） | `参考（非本仓结论）` |
| [`rm_ground_traversability/README.md`](../src/rm_perception/rm_ground_traversability/README.md) | header-only 坡度/台阶判定包：为什么有它、判据是什么 | `现行` |

## 取代表（不再成立的说法）

> 一行一条「旧说法 → 新结论」。**旧文档不改写**，只在有必要的几份顶部加 `STATUS`。看到左列，按右列理解；有冲突时**以新文档/新 commit 为准**。

| # | 旧说法 | 出处（doc:line） | 新结论（doc § / commit） |
|---|---|---|---|
| 1 | **「插入一条冻结的重力中间帧（`odom_grav` / `world_grav`）是标准做法」** | `docs/gravity_alignment_ownership_survey.md:421`（§6.3 选项②）、`:516` | **18 个仓库/包 0 例**（`odom_base_link_practice_survey.md` §3.1；`frame_contract_v3_rationale.md` §3.1）。已被 **E1 估计器初始化对齐**取代：`frame_contract_v3.md` §2（无 `odom_grav`）/ §4；commit `952a6da` |
| 2 | **pb2025 式「只填 `gravity` 参数」可行** | `docs/odom_base_link_practice_survey.md:889`（§9.4(1)(3)） | **不转世界系**：参数对且稳的 `v2b` 仍 **30.670°**；喂 pb2025 字面值**发散 897 m**（同文 §9.3 #5/#7、§9.4(2)；`frame_contract_v3_rationale.md` §7②）。必须动**初始旋转** |
| 3 | **Autoware `imu_corrector` 会估 roll/pitch 并摆平 IMU** | 早期任务书前提；`docs/odom_base_link_practice_survey.md:365` 记为「对任务书前提的更正」 | **它不估、不做重力对齐**：只修 yaw rate 零偏/尺度 + 用 TF 把量测旋到 `base_link` + 协方差**只膨胀不旋转**（同文 R10/§3.4；`frame_contract_v3_rationale.md` §7③） |
| 4 | **共轭代码出自 COD_NAV** | `docs/frame_contract_v3.md:176`（仍写「上游 `qza36/COD_NAV` 的 `small_point_lio_node.cpp:75`」） | `qza36/COD_NAV` **master 不含** `small_point_lio`（master = FAST-LIO 栈，`cod_nav_nav_stack_inventory.md:58`）；写法属上游 **`Yancey2023/small_point_lio@688d75c`**，COD 只是 vendor（`frame_contract_v3_rationale.md` §7④；本仓 `2a5a8b6` 修为合成） |
| 5 | **LIO-SAM 的 `odom→base_link` 合成在 `src/transformFusion.cpp`** | 二手资料/早期稿的引用（`frame_contract_v3_rationale.md` §7⑤ 收口） | 现行 `master` **已无该文件**（只剩 `featureExtraction/imageProjection/imuPreintegration/mapOptmization` 四个 cpp），`TransformFusion` 类并入 **`imuPreintegration.cpp:46-51/124-130`**（`odom_base_link_practice_survey.md:754`） |
| 6 | **波束出带率 41.2%** | `docs/session_digest.md:132`（亦见 `:155`、`:863`、`:1300`） | **39.6 / 40.4**：`sensor` 档 p2l 不丢束（<2%），出带的是**代价图高度带**；独立复跑 0.396（`tilted_lidar_fidelity.md` §K.1.3:1510）、首测 0.404（§J.5:1155）；`frame_contract_v3_rationale.md` §7⑥ |
| 7 | **`0.348 µs/ray`（RayQuery 探针）** | `docs/gazebo_sim_migration_plan.md:1342`（§9.1.2 / 附录 A） | **无效**：`hit=0%` + mm/m 单位 bug（同文 §9.1.2、§9.6 #5 明写「不要引用」）。有效值 = CPU BVH **0.417–0.978 µs/ray**（§9.1.4）。⚠️ 该数字**不在** `livox_sensor_prior_art.md` §10（见待决 ⑤） |
| 8 | **§J 那轮地面倾角 ≈1.05°（绝对值）** | `docs/tilted_lidar_fidelity.md:23`（§A）、`:85` | 世界/出生点在 2026-10-09 后已改，**该绝对值未复现**（`round_summary_gz_frames.md` §4 U7:380；`frame_contract_v3_rationale.md` §8 末）。只保留判决性签名：最大平面角 **0.046°** vs bug② 预测 **29.977°**（`tilted_lidar_fidelity.md:2961`） |
| 9 | **`robot11_mount_axis` 参数** | 任务书命名；`docs/frame_contract_v3.md:105`、§10.1:577 记为「本仓不存在」 | 本仓**不存在**（全仓 grep 无命中）；实际是 **`livox_tilt_axis`**（`bringup_sim.launch.py:1407-1416`；v3 的 `L:1257-1268` 已于 2026-10 批量校正为 `L:1407-1416`）。**2026-10-21 该参数已删除并冻结 `roll`**（见 `frame_contract_v3_measurements.md` §28） |
| 10 | **swap 已满（`SwapFree` ≈1 MiB）** | `docs/gazebo_gui_troubleshooting.md:155`（亦见 `:727`、`:824`、`:851`） | 实测 **used 194 / free 3901 MiB**（总 4095），旧快照来自**上一次开机**（`machine_hygiene_runbook.md:23`、`:35`） |
| 11 | **COD 的 `base_link ≡ livox_frame`（C 型，`T_bl≡I`）** | `docs/cod_vs_ours_frame_architecture.md:420`（§6.1）、§3.3:202 | 实车实测 **A 型**：`T_bl` rpy ≈ **(−46.238°, −1.684°, 0)**、`odom→base_link` ≈ **0.030°** ⇒ **共轭 bug 真实生效**，不再被 `T_bl≡I` 掩盖。原始输出与算术核对见 [`cod_real_frame_measurement.md`](cod_real_frame_measurement.md)（2026-10 落盘） |
| 12 | **`small_point_lio` 更快（所以用它的理由）** | `docs/cod_nav_macro_integration.md:124`（F3）、`docs/cod_nav_2026_deep_dive.md:71` | 本仓调参下实测 **慢 2.4×**：**13.75 vs 5.75 ms/帧**；但两者**工作量不同**（spl 进 EKF ≈3500 点/帧 vs pointlio 发布 146 点）⇒ **该比较不公平、不能说明核心快慢**；**同工作量 A/B（spl 改前 vs 改后）进行中**（`frame_contract_v3_measurements.md` §1、§3.1、§6.2） |
| 13 | **「`fix=false` 组 6 例中 4 例发散」** | `952a6da` 的 commit message | **8 个 `fix=false` 变体：5 发散 + 2 云被拖花**，唯一稳的是把 init 推迟的 `v2b`（`0ee92b6`；`frame_contract_v3_rationale.md` §7⑦；`odom_base_link_practice_survey.md` §9.3 末） |

## 待决（2026-10 合并轮：标 ✅ 的条目已处理，其余未动）

**① 未跟踪的 md —— ✅ 已清零（2026-10 合并轮）**
- `cartographer_map_stability_report.md`（仓库根，455 行英文）→ 已并入 [`cartographer_2d_occupancy_semantics.md`](cartographer_2d_occupancy_semantics.md) **§5 合并附录**（重复部分在 §5.1 列指针），随后删除该未跟踪文件。
- [`livox_viewer2_launch_notes.md`](livox_viewer2_launch_notes.md)（255 行）→ 已 `git add` 入库（内容未改）。

**② 近重复（同一件事的多份文档）**
- ✅ 已合并：`docs/package/` 四份说明 → [`package/config_package_guide_merged.md`](package/config_package_guide_merged.md)（原文归档 `docs/archive/package/`）。
- ✅ 已合并：`docs/mapping/建图操作指南.md` + `立即开始建图.md` → [`mapping/mapping_session_snapshots_merged.md`](mapping/mapping_session_snapshots_merged.md)（原文归档 `docs/archive/mapping/`）。
- `cod_nav_nav_stack_inventory.md` 与 `cod_nav_open_source_materials.md` —— 同一批 COD 仓库清单（**未合并**，本轮不做）。
- `cod_nav_comparison.md` 是 `cod_nav_design_thinking.md` 的素材，内容大幅重叠。

**③ 主题已被更新文档覆盖（保留供回溯，不当现行依据）**
- `rm_algorithm_overview.md` → [`architecture.md`](architecture.md) §3.2.1 + [`algorithm_matrix.md`](algorithm_matrix.md) §一；`rm_algolab_plan.md` → [`architecture.md`](architecture.md)
- `docs/mapping/` 两份快照（已合并归档）→ [`smoke_test_runbook.md`](smoke_test_runbook.md) §1/§5；`gravity_alignment_ownership_survey.md` 的 `world_grav` 推荐 → [`frame_contract_v3.md`](frame_contract_v3.md) §4

**④ 入链情况**
- 旧索引漏收 **39 份**（`cod_nav_*` 9、`gicp_*` 3、`scan_context_*` 3、`package/` 4、`continue_mapping` / `localization_slots` 等）；本次**全部收录**，本索引无孤儿。
- 绝对零入链 4 份：`archive/pbstream_生成完成报告.md`、`archive/reality_frozen/FROZEN.md`、`robot11_description/inventory/mesh_inventory.md`、`tools/scripts/gzsim/ATTRIBUTION.md` —— 现已在 §⑤/§⑥ 收链。

**⑤ 需要补的证据（是仓库缺数据，不是文档格式问题）**
- ✅ 已补：取代表 **#11** 的实车实测原始数据已落盘为 [`cod_real_frame_measurement.md`](cod_real_frame_measurement.md)（5 条命令 + 原始输出 + 算术核对 + 7 条 caveat）。
- ✅ 已修：`frame_contract_v3.md` / `frame_contract_v3_rationale.md` 的 `L:xxxx` 行号已按 `8e5e928`（2853 行）**逐条复核**（`livox_tilt_axis` 由 `L:1257-1268` 更正为 `L:1407-1416`），两份文档顶部都加了"行号基线"说明。
- 取代表 **#7** 的引用位置：`livox_sensor_prior_art.md` §10 **没有**该数字，真实位置是 `gazebo_sim_migration_plan.md` §9.1.2 / §9.1.4（**未处理**，本轮不做）。

## 附录：清单与口径

**105** 份自有（docs 90（含 `docs/archive/` 6 份已归档）/ 根 2 / tools 2 / archive 3 / src 自有 7 / third_party 索引 1）· **119** 份第三方与子模块（src vendored 27 + `third_party/**` 92，**不进索引**，清单见 [`inventory.md`](inventory.md) 附录 B）· **124** 份临时/构建产物（`.tmp_*`、`.venv`、`install/`，**不计入**）。

## 维护约定

> **同一件事只在一处维护**，其它地方只放指针：跑通/阻塞 → `algorithm_matrix.md` §四/§五；修 bug → `issues_and_findings.md`；契约 → `tf_interface_contract.md`（规范性以 `frame_contract_v3.md` 为准）；推翻旧结论 → 本文件 [取代表](#取代表不再成立的说法) 加一行 + 旧文档加 `STATUS`；新增文档 → 本文件分组 + 重新生成 `inventory.md`。
