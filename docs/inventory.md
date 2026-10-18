# 全仓 Markdown 清单（inventory）

> **口径**：`git ls-files '*.md'` + `find`（含未跟踪/子模块内部），排除 `.git/`。
> **行数** = `wc -l`；**最后提交** = `git log -1 --format='%h %ad' --date=short -- <file>`（`未入库` = 尚无提交）。
> 生成于 2026-10 仓库整理；本清单是 [`README.md`](README.md) 的明细附表，**不移动/不改写任何文档**。

## 计数

| 类别 | 份数 | 处置 |
|---|---:|---|
| 本仓自有（docs / 根 / tools / archive / src 自有 / third_party 索引） | **102** | 收入本清单 A 表 + `README.md` 索引 |
| 第三方 vendored + 子模块内部 | **119** | 附录 B（**不进索引**，只列出） |
| 临时 / 构建产物（`.tmp_*`、`.venv`、`install/`） | **124** | 附录 C（不计入仓库文档） |
| 合计 | **345** | — |

## A. 本仓自有文档（102 份，按索引分组）

### ① 入门 / 总览（12 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `README.md` | 362 | `51ba382 2026-09-20` | 仓库总入口：来源与致谢、话题接口、Docker/原生两种环境、启动示例 |
| `docs/README.md` | 199 | `本次整理` | 本文件 —— 全仓 md 的唯一索引（含取代表与待决清单） |
| `docs/round_summary_gz_frames.md` | 512 | `68d936f 2026-10-18` | 上一轮（10-18）四条线合并总结：外参守卫 / 机器卫生 / Fortress 后端 / COD 帧对账，含 24 条缺口 |
| `docs/architecture.md` | 658 | `b2c8e1a 2026-10-05` | 目录结构 + 分层概念：为什么这么切 2D/2.5D/3D、参数在哪 |
| `docs/glossary.md` | 76 | `aa64817 2026-09-21` | 名词一句话定义 + 指向详述章节 |
| `docs/algorithm_matrix.md` | 587 | `25e33ad 2026-10-06` | 槽位 × 实现 × 资产 × 实测状态：算法现状的唯一真值 |
| `docs/algorithm_axes.md` | 184 | `7e7f8a6 2026-09-25` | 按"正交轴"索引整个 bench：一次只动一个轴、轴的拓扑与选项 |
| `docs/rm_algorithm_overview.md` | 151 | `84f3de6 2026-09-20` | 早期"角色 × 一体性 × 两场景"总结（推理过程仍有价值） |
| `docs/slam_lio_primer.md` | 3064 | `00172e6 2026-10-09` | 教材式讲义：KF→EKF→ESKF→预积分→LIO→SLAM→定位，每条公式落到本仓数字 |
| `docs/kalman_slam_tutorial.md` | 3606 | `7c6cd9f 2026-10-10` | 循序渐进读本：24 章 + 练习，从"为什么需要滤波"到 SLAM/AMCL/GICP |
| `docs/mapping/README.md` | 96 | `b158525 2026-09-23` | `docs/mapping/` 子索引：现行落盘步骤 + 旧快照清单 |
| `docs/package/README.md` | 13 | `84f3de6 2026-09-20` | `docs/package/` 子索引：离线配置包多版本的主次 |

### ② 契约 / 规范（10 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `docs/frame_contract_v3.md` | 700 | `8e5e928 2026-10-19` | **规范性**：三层参数、最终 TF 树（无 `odom_grav`）、I1–I7、重力契约 E1、重定位契约 |
| `docs/frame_contract_v3_rationale.md` | 603 | `23bebb8 2026-10-19` | 上述契约的理由记录：为什么只有 E1 让世界系 z 朝上、18 仓库 0 例、8 条自我更正 |
| `docs/tf_interface_contract.md` | 153 | `489947f 2026-09-21` | TF/话题契约（T1–T5 已实施、T6 撤销）：谁发哪条边、谁供哪个话题 |
| `docs/sim_real_contract.md` | 144 | `34c64a6 2026-09-22` | 仿真↔实车契约：分界点/替身、三分类判据、禁止清单 |
| `docs/params_ownership_checklist.md` | 95 | `712741f 2026-10-07` | 参数归属清单（R1–R5）：算法参数该放哪个包 |
| `docs/odom_base_link_practice_survey.md` | 970 | `c996365 2026-10-19` | `odom→base_link` 斜装/云台 15 行开源对照 + §9 重力初始化 18 变体实测（E1 的证据） |
| `docs/lio_extrinsic_application_survey.md` | 371 | `92992ab 2026-10-18` | 外参该在哪一步乘一次：四家源码对照 + 本仓两处变换判决 |
| `docs/gravity_alignment_ownership_survey.md` | 515 | `3017355 2026-10-19` | 重力对齐归谁（IMU/LIO/重定位/感知四层调研） |
| `docs/relocalization_practice_survey.md` | 456 | `e065107 2026-10-19` | 全局重定位 vs 局部跟踪的工业实践 + `map→odom` 到底归谁 |
| `docs/cod_vs_ours_frame_architecture.md` | 607 | `057294c 2026-10-18` | COD 实车 vs 本仓帧架构对账：base_link 约定分叉、共轭 bug 为何被掩盖 |

### ③ 算法 / 原理（27 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `docs/3d_to_2d_survey.md` | 217 | `aa64817 2026-09-21` | 各家 3D→2D 实现对照与两种降维哲学 |
| `docs/mapping_relocalization_survey.md` | 424 | `c8d8776 2026-10-06` | 建图算法会不会用重定位、我们缺哪一级（回环/地点识别/多会话锚定） |
| `docs/nav2_vs_custom_planner.md` | 726 | `d4965bd 2026-10-08` | 纯 C++ 规划库接进 nav2 的三种接法、插件契约、效率账与落地坑 |
| `docs/nav_strategy_taxonomy.md` | 680 | `09578ed 2026-10-05` | 导航策略全图：9 个设计轴 × 81 个族，本仓与 COD 放在同一张地图上 |
| `docs/rm_algorithm_catalog.md` | 323 | `d00f8e3 2026-09-21` | 候选算法池（2D/3D 建图、重定位、相机），含"前端 vs 系统"之辨 |
| `docs/ground_segmentation_slots.md` | 790 | `f78a685 2026-10-07` | 地面分割槽位（linefit ↔ Patchwork++）：上游事实、逐字段契约、A/B |
| `docs/lio_slots.md` | 781 | `712741f 2026-10-07` | 里程计槽位：各 LIO 吃什么话题/时间戳、small_point_lio 接线与实测 |
| `docs/localization_slots.md` | 853 | `6f0c27a 2026-10-06` | 重定位槽位登记表：一个槽位值一行，选一个就跑 |
| `docs/mapping_small_point_lio.md` | 460 | `7d03f16 2026-10-06` | `lio:=small_point_lio` 建图链：接线审计、3D 导出、RMUC2026 产物 |
| `docs/gicp_backend_small_gicp.md` | 454 | `680a834 2026-10-05` | gicp 两个配准后端（pcl / small_gicp）的对等支持 |
| `docs/stvl_local_costmap.md` | 340 | `b2c8e1a 2026-10-05` | 局部代价地图上 STVL（`local_obstacle:=stvl`）：机制、参数来源、A/B |
| `docs/slope_speed_limiting.md` | 348 | `95846c3 2026-10-07` | 前瞻坡度/台阶限速：物理推导 + 整栈 A/B |
| `docs/traversability_plan.md` | 652 | `fc226bb 2026-10-07` | 3D 派生可通行性（坡度/落空/净空）设计计划，未实现 |
| `docs/scan_context_plan.md` | 738 | `413415d 2026-10-05` | Scan Context 类全局地点识别接入方案，未落地 |
| `docs/cartographer_2d_occupancy_semantics.md` | 624 | `d398cf3 2026-09-23` | cartographer 2D 占据栅格语义逐行考据（含 `/map` 上限 75 的推导） |
| `docs/livox_sensor_prior_art.md` | 508 | `03c8498 2026-10-18` | 非重复扫描 Livox 在新 Gazebo 上的既有实现调研 + REUSE/ADAPT/WRITE 裁决 + §10 编译实测 |
| `docs/kinodynamic_esdf_mpc_notes.md` | 270 | `7097602 2026-10-05` | `865749/Kinodynamic_esdf_mpc` 调研笔记（外部仓库） |
| `docs/slam_nav_goal_strategies.md` | 418 | `09578ed 2026-10-05` | "目标怎么给"四种方式的外部调研 |
| `docs/cod_nav_comparison.md` | 79 | `700f25d 2026-09-26` | COD_NAV 2025 与本仓的逐项对照（两套架构之辨） |
| `docs/cod_nav_2026_deep_dive.md` | 531 | `372d923 2026-09-27` | COD_NAV `rmul2026` 分支深读：逐环节参数 + 可采纳/不采纳 |
| `docs/cod_nav_2026_params_review.md` | 773 | `09578ed 2026-10-05` | cod_nav_2026 逐块参数评审（singlenav2/multiplenav2/approach/CSV） |
| `docs/cod_nav_2026_integration_plan.md` | 188 | `3449d9a 2026-10-04` | 把 COD(2026) 架构融进本 bench 的规划（未实施） |
| `docs/cod_nav_2026_migration_worksheet.md` | 463 | `48703e6 2026-10-05` | COD(2026)→本仓的移植工单（只登记，不动手） |
| `docs/cod_nav_macro_integration.md` | 508 | `09578ed 2026-10-05` | 宏观融入分析：small_point_lio 溯源、静态 `map→odom` 深挖、能力地图 |
| `docs/cod_nav_design_thinking.md` | 57 | `c3b0c96 2026-10-05` | COD(2026) 思路整理，重点"定位层为什么特殊" |
| `docs/cod_nav_nav_stack_inventory.md` | 225 | `09578ed 2026-10-05` | COD 战队导航开源资料完整清单 |
| `docs/cod_nav_open_source_materials.md` | 208 | `09578ed 2026-10-05` | COD 及兄弟仓库的公开资料清单（哪个真开源、在哪） |

### ④ 测量 / 实验记录（15 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `docs/frame_contract_v3_measurements.md` | 344 | `8152e57 2026-10-19` | 统一输出契约（选项 U）的效率+精度实测：13.75 vs 5.75 ms/帧、逐位回归 |
| `docs/tilted_lidar_fidelity.md` | 2987 | `fe0dc31 2026-10-18` | 雷达斜 30° 的逐帧点云账本 + 代价图归因 + bug①② 的判决性签名 |
| `docs/lio_drift_diagnosis.md` | 598 | `6ca978e 2026-10-07` | LIO 漂移分段诊断："才跑一会儿就飘"是否合理（唯一复现 = 高角速度甩转） |
| `docs/lio_divergence_no_impact.md` | 505 | `95846c3 2026-10-07` | "没撞击里程计自己飘"的整栈归因 + 限速 A/B |
| `docs/path_clearance_and_contact.md` | 615 | `819160c 2026-10-07` | 贴膨胀边→撞进去→定位跟着坏：实测、归因、修复 |
| `docs/gicp_divergence_and_jitter.md` | 404 | `6ca978e 2026-10-07` | GICP 重定位"发散 + 抖动"的复现、机制与修复 |
| `docs/gicp_initialpose_latency.md` | 369 | `c7bcd6d 2026-10-05` | `/initialpose` 的延迟/饥饿问题：修复设计与实测 |
| `docs/slam_drops_loopclosure_odom.md` | 581 | `c7260da 2026-10-06` | 建图丢帧 × 回环 × 里程计：两个假设的实测裁决 |
| `docs/slam_toolbox_scan_drops.md` | 604 | `7d03f16 2026-10-06` | `/scan` 丢帧（MessageFilter queue full）的根因考证与 `scan_queue_size` A/B |
| `docs/slam_toolbox_tuning.md` | 598 | `3a2b54d 2026-10-06` | slam_toolbox 建图漂移调参：上游语义考证 + 本机 A/B |
| `docs/mapping_2d_from_cloud.md` | 608 | `95846c3 2026-10-07` | 2D 先验图改从 3D 点云投影 + 建图稳健性 A/B（坡道段判定） |
| `docs/scan_context_probe_report.md` | 338 | `b626f37 2026-10-05` | A1 资产可行性探针：Scan Context 路线的准入判定 |
| `docs/scan_context_samesource_gate_report.md` | 270 | `5c9c0a0 2026-10-05` | Scan Context 同源性硬门（真 `/scan` → PCD 库）实测 |
| `docs/gazebo_sim_migration_plan.md` | 1862 | `057294c 2026-10-18` | Classic→新 Gazebo 迁移 spike + §9 纯 CPU 射线 BVH 实测（含单位 bug 修正） |
| `docs/livox_viewer2_launch_notes.md` | 255 | `未入库` | LivoxViewer2 在 22.04 启动即崩的取证 + ROS2 驱动替代路线 |

### ⑤ 历史 / 审计（17 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `docs/session_digest.md` | 1323 | `51d2240 2026-10-11` | 2026-10-05~08（161 提交）的总汇总 |
| `docs/timestamp_construction_audit.md` | 571 | `5c3d7af 2026-10-06` | 全仓时间戳构造审计：1 ns 截断类 bug 及其它戳/队列/QoS 风险 |
| `docs/build_optimization.md` | 552 | `6ae2ada 2026-10-05` | 构建配置审计（colcon 默认不设 `CMAKE_BUILD_TYPE`）与实测 |
| `docs/research_p2l_scan_stall.md` | 523 | `9c487ad 2026-09-23` | 旧链（livox→linefit→p2l）`/scan` 永久停发的调研（两条本仓缺陷） |
| `docs/rm_algolab_plan.md` | 128 | `84f3de6 2026-09-20` | 宏架构早期规划（2026-08/09 草案） |
| `docs/rm_bench_refactor_plan.md` | 205 | `84f3de6 2026-09-20` | 实验台改造路线 M0–M4 设计草案 |
| `cartographer_map_stability_report.md` | 455 | `未入库` | 英文报告：cartographer 2D 地图"墙被擦掉/不稳"的源码级根因 |
| `docs/mapping/建图操作指南.md` | 325 | `84f3de6 2026-09-20` | 当时会话的建图操作步骤（状态快照，现行看 runbook §1） |
| `docs/mapping/立即开始建图.md` | 115 | `84f3de6 2026-09-20` | 当时会话的"立即开始"简明指南（状态快照） |
| `docs/mapping/CARTOGRA_PBSTREAM_GUIDE.md` | 366 | `371d2ff 2026-09-21` | `.pbstream` 生成指南（含"需谨慎对待"的旧做法） |
| `docs/package/配置包使用说明.md` | 468 | `ee6600c 2026-09-08` | 离线配置包使用说明（2026-09-08 状态） |
| `docs/package/如何分发配置包.md` | 340 | `ee6600c 2026-09-08` | 如何把项目打包分发（2026-09-08） |
| `docs/package/HZMI_RMSIMULATION_CONFIG_GUIDE.md` | 323 | `e8a07ca 2026-09-15` | 配置包使用指南（另一版本） |
| `docs/package/README_PACKAGE.md` | 214 | `ee6600c 2026-09-08` | 配置包 README（另一版本） |
| `archive/pbstream_生成完成报告.md` | 204 | `ee6600c 2026-09-08` | pbstream 生成成功报告（2026-09 归档） |
| `archive/reality_frozen/README.md` | 11 | `e8a07ca 2026-09-15` | 真车配置冻结快照说明（R5） |
| `archive/reality_frozen/FROZEN.md` | 6 | `e8a07ca 2026-09-15` | 该目录"已冻结"的六行声明 |

### ⑥ 资产 / 流程（21 份）

| 路径 | 行数 | 最后提交 | 解决什么问题 |
|---|---:|---|---|
| `docs/smoke_test_runbook.md` | 1014 | `adc2622 2026-10-06` | 怎么跑 / 怎么判 / 错了怎么查（原生命令，无封装脚本） |
| `docs/map_assets.md` | 328 | `95846c3 2026-10-07` | `map/` 与 `PCD/` 里哪一份是当前默认资产 + 换图/回滚/清理 |
| `docs/worlds.md` | 408 | `712741f 2026-10-07` | 场地世界资产（RMUC2026 由 STL 生成）：来源、出生点、坑与复跑脚本 |
| `docs/robot_models.md` | 2162 | `b2caff3 2026-10-11` | 机器人模型槽位 `robot`：provenance、几何、安装决策、新旧对照、回滚 |
| `docs/fortress_sim_backend.md` | 937 | `0991302 2026-10-18` | `sim_backend:=gz`（Fortress）后端的 opt-in 用法与验证 |
| `docs/machine_hygiene_runbook.md` | 701 | `a137bf2 2026-10-18` | swap / zram / systemd-oomd 与收尾作用域的机器卫生 runbook |
| `docs/gazebo_gui_troubleshooting.md` | 869 | `51d2240 2026-10-11` | gzclient 加载不出来的取证、机理与修法 |
| `docs/debug_fastlio_cartographer.md` | 899 | `219d2a2 2026-10-04` | 建图链路调试手册：分层判读 L0–L6 + 症状速查 + 踩坑 |
| `docs/continue_mapping.md` | 822 | `95846c3 2026-10-07` | 在上次基础上续建 + 存档 + 场地隔离的操作流程 |
| `docs/issues_and_findings.md` | 248 | `20a0de5 2026-09-24` | 实际踩到的故障根因库 + 静默失效坑 + 待办 |
| `tools/README.md` | 36 | `eb7b811 2026-10-05` | 工具脚本根目录索引 |
| `tools/scripts/gzsim/ATTRIBUTION.md` | 78 | `50a7e1a 2026-10-18` | gzsim 后端的上游 pin 与许可证/署名 |
| `THIRD_PARTY_NOTICES.md` | 62 | `1a9ce8c 2026-10-06` | 第三方组件署名与许可证清单 |
| `third_party/README.md` | 25 | `d030ca1 2026-09-09` | `third_party/` 的"只读对照"规则与当前内容 |
| `src/rm_nav_bringup/map/attic/README.md` | 15 | `c8d65f9 2026-10-07` | `map/attic/` 退役资产规则（只搬不删） |
| `src/rm_nav_bringup/PCD/attic/README.md` | 13 | `c8d65f9 2026-10-07` | `PCD/attic/` 退役资产规则（只搬不删） |
| `src/rm_simulation/robot11_description/inventory/geometry.md` | 96 | `f361f00 2026-10-07` | robot11 几何量测（逐 link 包围盒、地面/雷达高度），自动生成 |
| `src/rm_simulation/robot11_description/inventory/mesh_inventory.md` | 270 | `f361f00 2026-10-07` | robot11 mesh 清单（自动生成，勿手改） |
| `src/rm_simulation/rm_livox_gz_sim/doc/README.md` | 43 | `057294c 2026-10-18` | `sim_backend:=gz` 的 opt-in 后端资产用法与编译步骤 |
| `src/rm_localization/small_point_lio/README.md` | 101 | `98baa55 2026-10-05` | vendored small_point_lio 的上游 README（含本仓联系方式补丁） |
| `src/rm_perception/rm_ground_traversability/README.md` | 67 | `3ea5dd4 2026-10-07` | header-only 坡度/台阶判定包：为什么有它、判据是什么 |

## 附录 B. 第三方 / 子模块文档（119 份，不进索引）

规则：`third_party/` 与 `src/` 下的上游副本**只读对照**，本仓不维护其内容；此处只登记，便于判断"某个 md 改动该不该我们管"。

### B.1 vendored 与子模块内部（119 份）

```
src/rm_driver/livox_ros_driver2/README.md
src/rm_driver/livox_ros_driver2/src/CHANGELOG.md
src/rm_localization/FAST_LIO/README.md
src/rm_localization/FAST_LIO/include/ikd-Tree/README.md
src/rm_localization/cartographer/.github/PULL_REQUEST_TEMPLATE.md
src/rm_localization/cartographer/CONTRIBUTING.md
src/rm_localization/cartographer/PATCH_README.md
src/rm_localization/point_lio/Log/guide.md
src/rm_localization/point_lio/README.md
src/rm_localization/point_lio/include/IKFoM/README.md
src/rm_localization/slam_toolbox/.github/ISSUE_TEMPLATE/bug_report.md
src/rm_localization/slam_toolbox/.github/ISSUE_TEMPLATE/feature_request.md
src/rm_localization/slam_toolbox/.github/PULL_REQUEST_TEMPLATE.md
src/rm_localization/slam_toolbox/README.md
src/rm_navigation/costmap_converter/README.md
src/rm_navigation/costmap_converter/costmap_converter/include/costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/README.md
src/rm_navigation/costmap_converter/costmap_converter/src/costmap_to_dynamic_obstacles/multitarget_tracker/README.md
src/rm_navigation/fake_vel_transform/README.md
src/rm_navigation/teb_local_planner/README.md
src/rm_perception/linefit_ground_segementation_ros2/README.md
src/rm_perception/patchwork_ground_segmentation/README.md
src/rm_perception/patchwork_ground_segmentation/thirdparty/patchwork-plusplus/CHANGELOG.md
src/rm_perception/patchwork_ground_segmentation/thirdparty/patchwork-plusplus/README.md
src/rm_perception/patchwork_ground_segmentation/thirdparty/patchwork-plusplus/VENDORING.md
src/rm_perception/patchwork_ground_segmentation/thirdparty/patchwork-plusplus/cpp/README.md
src/rm_perception/pointcloud_to_laserscan/README.md
src/rm_simulation/livox_laser_simulation_RO2/README.md
third_party/cartographer/.github/PULL_REQUEST_TEMPLATE.md
third_party/cartographer/CONTRIBUTING.md
third_party/cod_nav_2026/README.md
third_party/cod_nav_2026/src/fake_vel_transform/README.md
third_party/cod_nav_2026/src/pb_nav2_plugins/CONTRIBUTING.md
third_party/cod_nav_2026/src/pb_nav2_plugins/README.md
third_party/cod_nav_2026/src/pb_omni_pid_pursuit_controller/CONTRIBUTING.md
third_party/cod_nav_2026/src/pb_omni_pid_pursuit_controller/README.md
third_party/cod_nav_2026/src/pointcloud_to_laserscan/README.md
third_party/cod_nav_2026/src/ros2_simple_serial/README.md
third_party/cod_nav_2026/src/small_point_lio/README.md
third_party/cod_nav_2026/src/waypoint_editor/README.ja.md
third_party/cod_nav_2026/src/waypoint_editor/README.md
third_party/cod_nav_2026/src/waypoint_editor/USAGE_WITH_NAV2.md
third_party/cod_nav_2026/src/waypoint_editor/USAGE_WITH_NAV2.zh.md
third_party/fast_lio/Log/guide.md
third_party/fast_lio/README.md
third_party/nav2/.devcontainer/caddy/srv/nav2/index.md
third_party/nav2/.github/ISSUE_TEMPLATE.md
third_party/nav2/.github/PULL_REQUEST_TEMPLATE.md
third_party/nav2/README.md
third_party/nav2/doc/README.md
third_party/nav2/doc/development/codespaces.md
third_party/nav2/doc/process/PreReleaseChecklist.md
third_party/nav2/doc/requirements/_template_requirement.md
third_party/nav2/doc/requirements/requirements.md
third_party/nav2/doc/use_cases/README.md
third_party/nav2/doc/use_cases/_template_use_case.md
third_party/nav2/doc/use_cases/collision_avoidance_use_case.md
third_party/nav2/doc/use_cases/indoor_localization_use_case.md
third_party/nav2/doc/use_cases/indoor_navigation_use_case.md
third_party/nav2/doc/use_cases/keep_out_zones_use_case.md
third_party/nav2/doc/use_cases/multi-story-building_use_case.md
third_party/nav2/doc/use_cases/outdoor_localization_use_case.md
third_party/nav2/doc/use_cases/outdoor_navigation_use_case.md
third_party/nav2/nav2_amcl/README.md
third_party/nav2/nav2_behavior_tree/README.md
third_party/nav2/nav2_behavior_tree/groot_instructions.md
third_party/nav2/nav2_behaviors/README.md
third_party/nav2/nav2_bringup/README.md
third_party/nav2/nav2_bt_navigator/README.md
third_party/nav2/nav2_collision_monitor/README.md
third_party/nav2/nav2_constrained_smoother/README.md
third_party/nav2/nav2_controller/README.md
third_party/nav2/nav2_core/README.md
third_party/nav2/nav2_costmap_2d/README.md
third_party/nav2/nav2_dwb_controller/README.md
third_party/nav2/nav2_graceful_controller/README.md
third_party/nav2/nav2_lifecycle_manager/README.md
third_party/nav2/nav2_map_server/README.md
third_party/nav2/nav2_mppi_controller/LICENSE.md
third_party/nav2/nav2_mppi_controller/README.md
third_party/nav2/nav2_msgs/README.md
third_party/nav2/nav2_navfn_planner/README.md
third_party/nav2/nav2_planner/README.md
third_party/nav2/nav2_regulated_pure_pursuit_controller/README.md
third_party/nav2/nav2_rotation_shim_controller/README.md
third_party/nav2/nav2_route/README.md
third_party/nav2/nav2_route/graphs/scripts/README.md
third_party/nav2/nav2_simple_commander/README.md
third_party/nav2/nav2_smac_planner/README.md
third_party/nav2/nav2_smac_planner/lattice_primitives/README.md
third_party/nav2/nav2_smoother/README.md
third_party/nav2/nav2_system_tests/README.md
third_party/nav2/nav2_system_tests/models/dynamic_obstacle/README.md
third_party/nav2/nav2_system_tests/models/dynamic_world/README.md
third_party/nav2/nav2_system_tests/models/empty_room/README.md
third_party/nav2/nav2_system_tests/src/behaviors/README.md
third_party/nav2/nav2_system_tests/src/localization/README.md
third_party/nav2/nav2_system_tests/src/planning/README.md
third_party/nav2/nav2_system_tests/src/system/README.md
third_party/nav2/nav2_system_tests/src/system_failure/README.md
third_party/nav2/nav2_system_tests/src/updown/README.md
third_party/nav2/nav2_system_tests/src/waypoint_follower/README.md
third_party/nav2/nav2_theta_star_planner/README.md
third_party/nav2/nav2_util/README.md
third_party/nav2/nav2_velocity_smoother/README.md
third_party/nav2/nav2_voxel_grid/README.md
third_party/nav2/nav2_waypoint_follower/README.md
third_party/nav2/tools/planner_benchmarking/README.md
third_party/nav2/tools/smoother_benchmarking/README.md
third_party/point_lio/Log/guide.md
third_party/point_lio/README.md
third_party/point_lio/include/IKFoM/README.md
third_party/small_gicp/.github/ISSUE_TEMPLATE/bug_report.md
third_party/small_gicp/.github/ISSUE_TEMPLATE/feature_request.md
third_party/small_gicp/BENCHMARK.md
third_party/small_gicp/CODE_OF_CONDUCT.md
third_party/small_gicp/CONTRIBUTING.md
third_party/small_gicp/README.md
third_party/small_gicp/docs/index.md
third_party/small_gicp/docs/paper.md
```

## 附录 C. 临时 / 构建产物（124 份，不计入）

`.tmp_*/`（历史调研草稿与复现工作区）、`.venv/`（Python 依赖自带的 LICENSE/README）、`install/`（colcon 安装树，是 `src/` 与 `share/` 的拷贝）——**均非本仓文档**，不索引、不修改。

| 区域 | 份数 |
|---|---:|
| `.tmp_*` | 109 |
| `.venv` | 8 |
| `install` | 7 |
