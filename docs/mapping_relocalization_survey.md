# 建图算法会用重定位吗？—— 术语考据 × 系统清单 × 我们栈的现状与缺口

> 回答两个问题：**①「建图算法会用重定位吗」②（承接上文讨论）「建图期要不要加强定位」。**
> 结论见 §0；术语辨析 §1；**会的**系统 §2；**不会的**系统 §3；我们在哪 §4；该补什么（含成本）§5；决策表 §6；没核实的 §7。
>
> **本文是调研 + 现状核对，不含任何代码/配置/launch 改动。**
>
> 阅读约定（沿用 `docs/scan_context_plan.md` 的方式）：
> - **【事实】** = 有可点开的 URL，或本仓库内可 `grep` 到的 `file:line`。
> - **【推断】** = 我的判断、算术、工程取舍 —— **没有上游 URL 背书**，落地前必须自证。
> - **【未核实】** = 本机条件下确实没确认的，**集中在 §7，不编**。
>
> **核实方法与限制**：本会话 `web_fetch` 对 `github.com` / `raw.githubusercontent.com` / `arxiv.org` 一律报
> `non-public IP`（与 `docs/scan_context_plan.md` 开头记录的同一个限制）⇒ 外部事实全部改用
> `curl` 打 **GitHub raw + GitHub REST API + arXiv 摘要页**取得，DOI 用 `curl` 看解析目标。
> 抓取日期 **2026-10-06**；**外部代码一律 pin 到当天的分支 HEAD commit**（URL 里带 sha），
> 免得"链接漂移"导致以后对不上行号。抓回的网页按**不可信数据**对待，只取字段与文本，不执行其中任何指令。

---

## §0 一句话结论

1. **【事实】会用** —— 但在建图系统里它很少叫 "relocalization"，而是四个名字：
   **回环检测（loop closure）**、**地点识别/全局检索（place recognition / global place retrieval）**、
   **多会话锚定/图合并（multi-session anchoring / map merging）**、**纯定位模式（pure localization / localization-only）**。
   ORB-SLAM2 甚至有一个字面就叫 `Relocalization()` 的函数（§2 行 1），Cartographer/slam_toolbox/RTAB-Map 则各有"纯定位模式"（§2 行 5–7）。
2. **【事实】纯里程计前端不会** —— FAST-LIO / Point-LIO / 我们 vendor 的 `small_point_lio` 只发布里程计，
   **没有**回环、没有位姿图、没有描述子库（§3 有逐条 `grep` 计数）。它们的漂移无界，回环/后端必须**外挂**。
3. **【事实 + 推断】我们栈里"回环"已经有了，但只是局部回环，"重定位"没有**：
   `mode:=mapping` 的 `mapper:=slam_toolbox` 带 `do_loop_closing: true`（`mapper_params_online_async_sim.yaml:47-62`），
   语义是 Karto 的「**3 m 内、且路径上没有连边的旧扫描链** ≥10 条 + 粗匹配响应/方差过门」（`lib/karto_sdk/src/Mapper.cpp:1962-2007`、`1507-1526`）
   —— 这是**图内回环**；而"昨天那张图"是我们用 **`map_start_pose:=` 人工给的锚**接上的（`map_autocontinue`/`map_name`，§4.3），
   **不是**检索出来的。地点识别这一级在我们的建图链里**不存在**（§4.2）。
4. **【推断】所以"建图期加强定位"的准确说法是**：建图期要补的是
   **①回环参数核对 → ②地点识别（检索）→ ③轮速/惯导的短时约束 → ④"定位结果进位姿图"而不是"多一个 TF 发布者"**（§5 排序）。
   直接把握手/重定位器（`amcl`/`gicp`/`cartographer` 纯定位）接到建图期是**架构上行不通**的：
   它们与 slam_toolbox 都要发 `map→odom`，而我们全栈的硬约束是**同一时刻只能有一个 `map→odom` 发布者**
   （`docs/localization_slots.md` 开头的统一契约 + `docs/tf_interface_contract.md:47-50` 的角色表）。

---

## §1 术语：同一个机制，五个名字（外加一条定义）

**【事实】同一个"全局位姿恢复"能力，在不同系统里叫不同名字**；名字取决于**它在哪张图上工作**：

| # | 名字 | 典型场合 | 出处（URL / file:line） |
|---|---|---|---|
| A | **relocalization** | **跟踪丢失**后的恢复（kidnapped robot / 被搬动 / 快速旋转丢跟踪） | ORB-SLAM2 `Tracking::Relocalization()`，注释原文 *"Relocalization is performed when tracking is lost"*，由状态机在 `eTrackingState::LOST` 分支调用：<https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/Tracking.cc#L1341>（调用点 `:320` `:329` `:366`；状态枚举 `include/Tracking.h:81-89` 同 sha）；ORB-SLAM3 同函数：<https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L3609> |
| B | **loop closure（回环）** | **图内**新老帧/子图之间加约束，修正累计漂移 | ORB-SLAM2 `LoopClosing::DetectLoopCandidates`（DBoW2 词袋）：<https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/LoopClosing.cc#L141>；LIO-SAM `loopClosureThread()` / `detectLoopClosureDistance()`：<https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L503>（`:610` 半径搜索） |
| C | **place recognition / global place retrieval（地点识别）** | **不知道自己在哪**时先在库里检索"我来过这"，再进行局部配准 | 综述把全局定位拆成 *"the combination of global place retrieval and local pose estimation"*：<https://arxiv.org/abs/2302.07433>（摘要）；描述子清单见 §2.2 |
| D | **multi-session / map merging / anchoring（多会话、图合并、锚定）** | **跨会话**：把今天的会话接到昨天那张图上 | slam_toolbox `.posegraph` 反序列化续建 + `merge_maps_kinematic` 节点（§2 行 7）；RTAB-Map `Memory::saveOptimizedPoses`（注释 *"for next session"*）：<https://github.com/introlab/rtabmap/blob/aa95581cd2f0adca514e0fa6747ea4d7ce53d962/corelib/include/rtabmap/core/Memory.h#L260>；ORB-SLAM3 `LoopClosing::MergeLocal()/MergeLocal2()`：<https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LoopClosing.cc#L1215> |
| E | **pure localization / localization-only（纯定位模式）** | **先验图上只定位、不再改图**（= 我们 `mode:=nav` 的 `localization` 槽做的那件事） | Cartographer `PureLocalizationTrimmer`（注释 *"to implement localization without mapping"*）：<https://github.com/cartographer-project/cartographer/blob/877157a0d91788a7700221d87232d412cb3c1ef4/cartographer/mapping/pose_graph_trimmer.h#L67>；slam_toolbox `localization_slam_toolbox_node`（§2 行 7） |

**【事实】权威一点的措辞**：SLAM 领域综述（Cadena et al., *Past, Present, and Future of SLAM: Towards the Robust-Perception Age*, arXiv 1606.05830v4）
把这件事称为 **"Metric Relocalization"**，定义为 *"estimating the relative pose with respect to the previously built map"*
（原文见 <https://arxiv.org/abs/1606.05830>；本会话用 `pdftotext` 抽的正文，第 746 行附近）。

**【推断】一条辨析（本文最有用的那句）**：

> **回环 = 图内的约束**（新帧 ↔ 历史帧/子图，加的边和基准都在同一张图里）；
> **重定位 = 当前观测 ↔ 已有地图/别的会话**（基准在图外）。
> 两者**共用同一套零件**（描述子检索 → 候选 → 配准 → 位姿图优化），差别只在**"约束加到哪张图、谁是基准"**。
> 这也正是"建图算法用不用重定位"这个问题的答案会是"**用，但通常不叫这个名字**"的原因。

**【事实】我们自己的词汇表里已经有这三格**：`docs/nav_strategy_taxonomy.md`
§2.4「先验图上的扫描匹配 / 粒子滤波（重定位）」、§2.5「地点识别 / 回环」、§2.9「不做重定位（静态 `map→odom`）」；
选型目录里也把"回环/纯定位"作为对照项：`docs/rm_algorithm_catalog.md:18-38`。

---

## §2 会用的系统（表 + 出处）

> 表里的"证据"一列，外部仓库都给 **pin 到 commit 的 URL**；我们仓库给 `file:line`。
> 抓取/核对日期 **2026-10-06**。

| # | 系统 | 建图期的"重定位"以什么形态出现 | 证据（参数 / 函数名） | 出处 |
|---|---|---|---|---|
| 1 | **ORB-SLAM2**（视觉） | ① 跟踪状态机里有**显式的 `Relocalization` 状态**（丢失即全局重定位）② 独立 `LoopClosing` 线程用 **DBoW2 词袋**做地点识别 | `bool Tracking::Relocalization()`；`mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame)`；`LoopClosing::DetectLoopCandidates(mpCurrentKF, minScore)` | <https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/Tracking.cc#L1341> ；`.../src/LoopClosing.cc#L141` ；论文 <https://arxiv.org/abs/1610.06475> |
| 2 | **ORB-SLAM3**（视觉惯性 + 多地图） | 同上，且**多地图合并**：检索到与"旧地图"的重叠就 `MergeLocal/MergeLocal2` 把两张图焊在一起 | `mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap())`；`mbMergeDetected`；`MergeLocal();` / `MergeLocal2();` | <https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L3609> ；`.../src/LoopClosing.cc#L181`（调用）`#L1783`（定义）；论文标题即含 "Multi-Map"：<https://arxiv.org/abs/2007.11898> |
| 3 | **LIO-SAM**（激光惯性） | 里程计 + **位姿图优化（GTSAM iSAM2）+ 回环线程**：按"半径 + 时间差"找历史关键帧，ICP 过了就加回环因子 | `loopClosureThread()`（`:503`）、`performLoopClosure()`（`:529`）、`detectLoopClosureDistance()`（`:610`，`radiusSearch(..., historyKeyframeSearchRadius, ...)`）；参数 `loopClosureEnableFlag: true`、`loopClosureFrequency: 1.0`、`historyKeyframeSearchRadius: 15.0`（m）、`historyKeyframeSearchTimeDiff: 30.0`（s）、`historyKeyframeFitnessScore: 0.3` | <https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L503> ；`.../config/params.yaml:82-88` ；论文 <https://arxiv.org/abs/2007.00258> |
| 4 | **SC-LIO-SAM**（激光惯性 + Scan Context） | 在 LIO-SAM 的回环之上**加一级全局地点识别（Scan Context）**：`SC` 回环修大漂移、`RS`（半径）回环做细缝 | README：*"We used two types of loop detections (i.e., radius search (RS)-based … and Scan context (SC)-based global revisit detection)"*、`performSCLoopClosure` / `performRSLoopClosure`、回环因子用 Cauchy 核；描述子 API 只有 `makeAndSaveScancontextAndKeys` / `detectLoopClosureID` | <https://github.com/gisbi-kim/SC-LIO-SAM/blob/d43ca00d97a756303c10975e32e8d66bfabb337d/README.md> ；`.../SC-LIO-SAM/src/mapOptmization.cpp`（GTSAM `ISAM2`、`loopPoseQueue`）；`.../SC-LIO-SAM/include/Scancontext.h` |
| 5 | **FAST-LIO-SAM**（= FAST-LIO2 前端 + LIO-SAM 式后端） | FAST-LIO2 只做里程计，**回环与位姿图在后端**：关键帧 → iSAM2 优化 → 检测到回环就加因子 | README：*"a SLAM implementation combining FAST-LIO2 with pose graph optimization and loop closing based on LIO-SAM paper"*；正文 *"keyframe detection → add to pose graph"*、*"detect loop → if loop, add to pose graph"* | <https://github.com/engcang/FAST-LIO-SAM/blob/6ec8b8f537aa33162856462f17b09f1f91e0eafc/README.md> |
| 6 | **Cartographer**（2D/3D 图 SLAM） | ① 全局 SLAM 用**分支定界扫描匹配**算回环约束；② 提供**纯定位模式**（加载冻结的 `.pbstream`，不再建图） | 约束构建：`ConstraintBuilder2D` 用 `FastCorrelativeScanMatcher2D` + `global_localization_min_score` / `min_score`（`constraint_builder_2d.cc:180,215,228`）；`pose_graph.lua` 里 `branch_and_bound_depth = 7`、`global_localization_min_score = 0.6`、`min_score = 0.55`；纯定位：`PureLocalizationTrimmer` + `map_builder.cc:56-73`；配置 `backpack_2d_localization.lua`（`pure_localization_trimmer.max_submaps_to_keep = 3`）+ `-load_state_filename` + `load_frozen_state`（默认 true） | 源码 <https://github.com/cartographer-project/cartographer/blob/877157a0d91788a7700221d87232d412cb3c1ef4/cartographer/mapping/pose_graph_trimmer.h#L67>、`.../mapping/map_builder.cc#L56`、`.../configuration_files/pose_graph.lua`；ROS 侧 <https://github.com/cartographer-project/cartographer_ros/blob/c138034db0c47fe0ea5a2abe516acae02190dbf5/cartographer_ros/configuration_files/backpack_2d_localization.lua>、`.../cartographer_ros/node_main.cc:35-45`；官方文档「Pure localization」：<https://google-cartographer-ros.readthedocs.io/en/latest/demos.html#pure-localization>；论文 W. Hess et al., *Real-Time Loop Closure in 2D LIDAR SLAM*, ICRA 2016, pp.1271-1278：<https://doi.org/10.1109/ICRA.2016.7487258> |
| 7 | **RTAB-Map** | ① 词袋回环（它的看家本领）；② **localization 模式**（`Mem/IncrementalMemory=false`：STM/WM 冻结，只更新 `getLastLocalizationPose()`）；③ **多会话**：位姿图与"上次定位位姿"落盘，下次会话载入继续；GUI 有 Multi-session localization 组件 | `Parameters::kMemIncrementalMemory()`、`isIncremental()`、`getLastLocalizationPose()`、localization 模式下"pose priors fixing the map nodes"（`kRGBDLocalizationPriorError`）、`Memory::saveOptimizedPoses(...)`/`loadOptimizedPoses(...)`（注释 *"for next session"*） | 头文件 <https://github.com/introlab/rtabmap/blob/aa95581cd2f0adca514e0fa6747ea4d7ce53d962/corelib/include/rtabmap/core/Rtabmap.h#L169>、`...#L113`、`...#L359`；`.../corelib/include/rtabmap/core/Memory.h#L260`；GUI `guilib/include/rtabmap/gui/MultiSessionLocWidget.h`；论文（JFR 36(2):416-446, 2019）：<https://arxiv.org/abs/2403.06341> |
| 8 | **slam_toolbox** | ① **回环**：Karto 的近距离链搜索 + 粗匹配门（参数化）；② **本地化模式**：`localization_slam_toolbox_node` 加载 `.posegraph`，"弹性位姿图定位"，并订阅 `/initialpose` 以便像 AMCL 那样**手工重定位**；③ **续建/长时**：`serialize/deserialize` 把位姿图存下来下次接着建，`lifelong_slam_toolbox_node` 做"真·lifelong"（可删节点）；④ **图合并**：`merge_maps_kinematic` 服务加载多张 `.posegraph` 手工/交互式拼成一张 | 参数 `do_loop_closing` / `loop_search_maximum_distance` / `loop_match_minimum_chain_size` / `loop_match_maximum_variance_coarse` / `loop_match_minimum_response_coarse` / `loop_search_space_dimension`（上游 `config/mapper_params_online_async.yaml`）；`mode` 只被 **Ceres solver** 读了两个值里的一个：`if (mode == std::string("localization"))` ⇒ 只开 `enable_fast_removal`；节点形态由 executable 决定（`localization_slam_toolbox_node` / `lifelong_slam_toolbox_node` / `async_slam_toolbox_node`） | README（lifelong §"LifeLong Mapping"、localization §"Localization mode consists of 3 things"、Map Merging、事件话题 `/slam_toolbox/loop_closure_event`）：<https://github.com/SteveMacenski/slam_toolbox/blob/33841d040dfff86f9467000bc022336995d56f20/README.md> ；`.../solvers/ceres_solver.cpp#L67`（读参）`#L183`（唯一分支）；`.../src/slam_toolbox_localization.cpp#L81`（`initialpose` 订阅）；`.../src/merge_maps_kinematic.cpp#L51`；`.../launch/lifelong_launch.py#L56`、`.../launch/localization_launch.py#L57`；JOSS 论文 <https://joss.theoj.org/papers/10.21105/joss.02783> |
| 9 | **hdl_graph_slam** | 3D 图 SLAM：NDT 里程计 + **回环检测** + 位姿图（g2o），另可加 GPS/IMU/地面约束 | README：*"it performs loop detection and optimizes a pose graph"*，特性列表里有 *"Loop closure"* | <https://github.com/koide3/hdl_graph_slam/blob/95b8dce41c10667dcf0e3e9801dc22754b3bd525/README.md> |
| 10 | **interactive_slam** | 把"回环/合并"做成**交互式后端**：手动+自动回环、手工平面校正、**多张地图合并** | README 特性表：*"[Manual & Automatic] Loop closing"*、*"[Manual] Multiple map merging"* | <https://github.com/koide3/interactive_slam/blob/4e3cb87edc0ab7c735e0aa85e1fd985b2aaa6841/README.md> |
| 11 | **maplab / maplab 2.0**（视觉惯性，多会话） | 多会话建图 + 在先验图里定位（"map merging"与"localization in a prior map"是它的两大卖点） | 论文（标题即含 Mapping and Localization）：<https://arxiv.org/abs/1711.10250>；maplab 2.0：<https://arxiv.org/abs/2212.00654> | 同左（**只核到标题/摘要页，正文未读**，见 §7） |
| 12 | **Kimera-Multi**（多机分布式） | 多机器人各自建图 + **机器人间地点识别** + **分布式位姿图优化/地图合并** | 论文标题 *"Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems"*：<https://arxiv.org/abs/2106.14386>（ICRA 2021 版：<https://arxiv.org/abs/2011.04087>） | 同左（**只核到标题/摘要页**，见 §7） |

### 2.1 那"在昨天的图上继续建图"到底是什么问题？【推断，机制来自上面各系统的代码事实】

把上面第 2/7/8/10/11 行的共性抽出来，就是一句话：

```
昨天的图（位姿图/子图集合）  ──┐
                              ├─ 需要知道一个"相对变换 T" ─→ 才能把今天的第一帧锚进去
今天开机时的第一帧观测      ──┘
```

求这个 `T` 只有三条路（现实系统通常混用）：

1. **人工给定**：告诉系统"我现在在旧图的 (x, y, θ)" —— slam_toolbox 的 `map_start_pose` / `START_AT_GIVEN_POSE`
   （`loadPoseGraphByParams()` 把 `map_file_name`+`map_start_pose` 变成一次 `DeserializePoseGraph` 请求：
   <https://github.com/SteveMacenski/slam_toolbox/blob/33841d040dfff86f9467000bc022336995d56f20/src/slam_toolbox_common.cpp#L607>），
   或"我从上次停下的地方/码头开机"（`map_start_at_dock` / `START_AT_FIRST_NODE`）。**这就是我们现在的做法**（§4.3）。
2. **地点识别 + 配准**：在旧图库里检索"这地方我来过"，得到候选位姿，再用 ICP/GICP 精配准（Scan Context/STD/BTC/DBoW/NetVLAD 都是这一级）。
3. **外部绝对定位**：GPS/RTK、UWB、二维码、动捕等直接给出全局位姿。

⇒ **【推断】"多会话建图"= 地点识别（或外部定位）求出 `T` + 把它作为位姿图的锚（一元/先验因子）**。
两条都缺，就只能靠第 1 条人工给定 —— 这正是我们当前的风险点：**同场地 + 同出生点这个前提一旦不成立，图就会叠错**（§4.3）。

### 2.2 地点识别的描述子清单（§5(ii) 要用）

| 描述子 | 类型 | 出处 |
|---|---|---|
| **Scan Context**（IROS 2018，G. Kim & A. Kim） | 3D 点云的**极坐标距离图**（egocentric），旋转不变（列移位）+ ring key 两级检索 | DOI（解析到 IEEE 8593698）：<https://doi.org/10.1109/IROS.2018.8593698>；bibtex 见 SC-LIO-SAM README；**原参考实现仓库 `irapkaist/scancontext` 现已 404**（见 §7） |
| **Scan Context++**（T-RO 2021/2022） | 上面那篇的期刊扩展（对旋转/横向位移更鲁棒） | <https://arxiv.org/abs/2109.13494> ；代码 <https://github.com/gisbi-kim/scancontext_tro> |
| **STD**（ICRA 2023，HKU MARS） | 三角形描述子（边长/夹角对刚体变换不变） | <https://arxiv.org/abs/2209.12435> ；代码 <https://github.com/hku-mars/STD>（README 明写 ICRA2023 + arXiv 链接） |
| **BTC**（T-RO，STD 的期刊扩展：Binary + Triangle Combined） | 三角形 + 二值化，检索更快 | DOI（解析到 IEEE 10388464）：<https://doi.org/10.1109/TRO.2024.3353076> ；代码 <https://github.com/hku-mars/btc_descriptor> |
| **M2DP**（IROS 2016） | 3D 点云投影到多平面后的 2D 签名 | DOI（解析到 IEEE 7759260）：<https://doi.org/10.1109/IROS.2016.7759260> ；代码 <https://github.com/LiHeUA/M2DP> |
| **NetVLAD**（CVPR 2016，视觉） | 学习式（CNN + VLAD 池化）地点识别 | <https://arxiv.org/abs/1511.07247> |
| **PointNetVLAD**（CVPR 2018，点云） | 学习式 3D 检索 | <https://arxiv.org/abs/1804.03492> |

> **本仓已有的取舍**：`docs/scan_context_plan.md` §A.1–§A.7 已经把上面这些逐个核过（含许可证结论），
> §E.4 的结论是**不 vendor 上游 3D 描述子（许可证风险，CC BY-NC-SA 一类），照 Scan Context 的思路自写一个 2D 版本**
> —— 因为我们手上的查询是 2D `/scan`、先验是"接近平的地面切片"。

---

## §3 不会用的系统（以及"为什么这是设计，不是缺陷"）

| 系统 | 有没有回环/重定位代码 | 证据（本会话的 `grep` / 仓库树 / README） |
|---|---|---|
| **FAST-LIO / FAST-LIO2** | **没有**（上游本体） | 仓库树 74 个 blob，路径里匹配 `loop|closure|pose_?graph|gtsam|g2o|place` 的文件 **0 个**（<https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9>）；README 只在 "Related Works" 里列出**第三方**扩展 `FAST-LIO-LOCALIZATION`：*"The integration of FAST-LIO with **Re-localization** function module"* ⇒ 重定位是**别人加的**：<https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/README.md> |
| **Point-LIO** | **没有** | README 自述定位是里程计：*"As a LiDAR odometry, Point-LIO could be used in various autonomous tasks…"*（README 全文检索 `loop|closure|relocal|pose graph|backend` **0 命中**）：<https://github.com/hku-mars/Point-LIO/blob/point-lio-with-grid-map/README.md#L18> |
| **small_point_lio**（我们 vendor 的那份） | **没有** | ① 节点只发两个话题：`/Odometry`、`/cloud_registered`（`src/rm_localization/small_point_lio/src/small_point_lio_node.cpp:62-63`）+ 一个 `/map_save` 点云落盘服务（`:70-71`）；② 全包检索 `loopClosure|loop_clos`、`relocal|Relocal`、`pose_?graph`、`gtsam|g2o|ceres`、`DetectLoop` —— **全部 0 命中**（含 `.md`；唯一带 "loop" 的字符串是 `src/small_point_lio/preprocess.cpp:68` 的 `"imu loop back"` 日志，与回环无关）；③ README 的第三方清单只有 Eigen / unordered_dense / small_gicp / Open3D，**没有** gtsam/g2o/描述子库（`src/rm_localization/small_point_lio/README.md`）；④ 它的"存图"= `save_pcd` + `/map_save` 存 **3D 点云**，不是位姿图（我们这边的接线与坑见 `docs/mapping_small_point_lio.md:12-16,76-85`） |
| **我们的 `localization:=''` 回退** | **没有**（连回环也没有） | `map≡camera_init` 静态桥 ⇒ "LIO 漂多少就是多少，无回环、无先验图约束"：`docs/localization_slots.md` §1 表末行 |

**【推断】为什么"纯里程计"是合理的设计**（不是偷懒）：

1. **分工**：里程计要 4k–8kHz、低延迟、退化场景稳（Point-LIO README 的卖点就是"高频 + 抗 IMU 饱和"）；
   位姿图优化是**全局、非线性、可能跳变**的批处理 —— 把它塞进前端会毁掉前端的实时性与可预测性。
2. **跳变代价**：回环修正会让 `map→odom` 一次性跳几厘米~几十厘米；如果这发生在导航中，代价地图/控制会抖。
   分层（前端连续、后端跳变、中间用 TF 缓冲）是工程上的标准答案：见我们自己的 `docs/architecture.md` 分层与
   `docs/rm_algorithm_catalog.md` §〇.0「前端 vs 系统」。
3. **代价**：没有回环 ⇒ 长时/大范围**必然漂**（这是 FAST-LIO 的 README 直接把 `FAST-LIO-LOCALIZATION` 列为"扩展应用"的原因）。

---

## §4 我们栈的现状

### 4.1 `mode:=mapping` 的真实接线（先认清在跑什么）

**【事实】** `mode:=mapping` = `lio:=<fastlio|pointlio|small_point_lio|none|cartographer>`（默认 `fastlio`）
＋ `mapper:=<slam_toolbox|cartographer>`（默认 `slam_toolbox`）
（`src/rm_nav_bringup/launch/bringup_sim.launch.py:431`（mode）、`:460-473`（lio choices）、`:521-525`（mapper choices））；
选 `mapper:=slam_toolbox` 时起的是 **`async_slam_toolbox_node`，节点名 `slam_toolbox`**，
参数文件 `src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml`
（`bringup_sim.launch.py:1156-1162`），其中：

| 键（我们文件里的值） | 值 | 位置 | 语义（【事实】来自我们 vendor 的 Karto 源码） |
|---|---|---|---|
| `mode` | `mapping` | `mapper_params_online_async_sim.yaml:17` | **只被 Ceres solver 读一个分支**：`if (mode == "localization")` ⇒ `enable_fast_removal=true`，其它值（含 `lifelong`）等价于默认 mapping（`src/rm_localization/slam_toolbox/solvers/ceres_solver.cpp:143`；上游同逻辑见 §2 行 8） |
| `do_loop_closing` | `true` | `:48` | 回环总开关（`src/slam_mapper.cpp:179` → Karto `m_pDoLoopClosing`） |
| `loop_search_maximum_distance` | `3.0` (m) | `:47` | 候选半径：把"**与当前帧距离 < 3 m、但路径上还没有连边**"的旧扫描攒成候选链（`lib/karto_sdk/src/Mapper.cpp:1962-1996`） |
| `loop_match_minimum_chain_size` | `10` | `:49` | 候选链**至少 10 条**才允许闭合（`Mapper.cpp:2001`；本仓另有 `LinkNearChains` 的同一判断 `:1646`） |
| `loop_match_minimum_response_coarse` | `0.35` | `:51` | 粗匹配响应下限（`Mapper.cpp:1523`） |
| `loop_match_maximum_variance_coarse` | `3.0` | `:50` | 粗匹配协方差上限（`Mapper.cpp:1524-1525`，对角元各自过门） |
| `loop_search_space_dimension` / `_resolution` / `_smear_deviation` | `8.0` / `0.05` / `0.03` | `:60-62` | 回环用的相关搜索空间（比顺序匹配的 `correlation_search_space_dimension: 0.5` 大得多，`:55-57`） |
| `enable_interactive_mode` | `true` | `:36` | 打开 RViz 交互插件 ⇒ **可以手动拖节点做"人工回环"**（上游 README：interactive mode 在 localization/lifelong 模式会被强制关掉） |
| `scan_buffer_size` / `scan_buffer_maximum_scan_distance` / `minimum_travel_distance` | `10` / `10.0` / `0.5` | `:43-44` / `:41` | 回环与本地点链的原料（扫描缓存/建节点步长） |

**【事实】上游参数清单出处**：<https://github.com/SteveMacenski/slam_toolbox/blob/33841d040dfff86f9467000bc022336995d56f20/config/mapper_params_online_async.yaml>
（与我们这份同结构；localization 档把 `loop_match_minimum_chain_size` 调成 `3`：`.../config/mapper_params_localization.yaml:45`）。

**【推断】一个**重要的尺度判断：我们的候选半径是 **3 m**，而 RMUL2026 场地是 **12.45 m × 8.65 m**
（`docs/scan_context_probe_report.md` §0 表；对角 ≈15.2 m）。也就是说：
**只有"我回到自己 3 m 以内"才可能触发回环**（同一区域绕圈够用），而"从场地一头走到另一头再回来"这类**大回环**能不能闭，
取决于中途是否有 ≥10 帧落在某个 3 m 邻域的旧链上 —— 这一点**我们从未实测过**（没有任何 `/slam_toolbox/loop_closure_event` 计数证据，见 §7）。

### 4.2 表：有什么 / 缺什么 / 依据

| 能力 | 我们有什么 | 我们缺什么 | 依据 |
|---|---|---|---|
| **单会话局部回环** | ✅ 有。`do_loop_closing: true` + Karto 近邻链 + 粗匹配门；另可 RViz 手动回环 | ①**没有任何一次"回环到底发生过几次"的实测**（无事件计数）②候选半径 3 m / 链长 10 的**场地适配性未验** | `mapper_params_online_async_sim.yaml:47-62`；`lib/karto_sdk/src/Mapper.cpp:1962-2007,1507-1526`；上游事件话题 `/slam_toolbox/loop_closure_event`（README） |
| **全局地点识别（retrieval）** | ❌ 无节点、无描述子库、无检索服务（全仓 `grep -liE "scan_?context|place_?recognition|dbow|netvlad|descriptor"` 在 `src/` 下只命中一个**无关**的 `teb_local_planner/graph_search.h`） | **整级缺失**：无法回答"我在旧图的哪里" | 本会话全仓 grep；`docs/scan_context_plan.md` 标题即「**纯方案文档，未落地**」 |
| **多会话续建（锚定）** | ✅ 有：`map_autocontinue`（默认 True）+ `map_name` + `map_start_pose` + world/spawn 守卫；`.posegraph/.data` 落盘与 `map_archive.sh save/info/adopt/backup/restore` | ①**"锚在哪"是人给的**（`map_start_pose` 默认 `[0,0,0]`），不是检索出来的 ②换出生点/换场地起手就会叠错图（守卫只能拦 world/spawn 不一致，拦不住"位姿错"） | `bringup_sim.launch.py:558-586`（四个参数）、`:1120-1130`、`:1152-1162`（注入 `map_file_name`+`map_start_pose`）；`tools/scripts/mapping/map_archive.sh`（save 用 `/slam_toolbox/serialize_map`；脚本自带写前备份与 `restore` 回滚）；上游机制 `src/slam_toolbox_common.cpp:607`（`loadPoseGraphByParams`→`DeserializePoseGraph`，`START_AT_GIVEN_POSE`/`START_AT_FIRST_NODE`） |
| **丢失找回（tracking lost / 被搬动）** | ⚠️ 部分：`localization` 槽里 `slam_toolbox` 纯定位**订阅 `/initialpose`**、AMCL 也有 `/initialpose` ⇒ 人工给一次初值即可恢复 | 自动恢复能力 = 0：没有"全局检索→自动注入初值"的闭环 | `docs/localization_slots.md` §1（`slam_toolbox` 行、`gicp`/`icp` 行均要求初值）；上游 `/initialpose`：`src/slam_toolbox_localization.cpp:81` |
| **先验图上的连续定位（导航期）** | ✅ 7 个槽：`amcl` / `beluga` / `slam_toolbox`(纯定位) / `icp` / `gicp` / `small_gicp` / `cartographer`(纯定位) | 资产/验收不均：`slam_toolbox` 纯定位缺 RMUL2026 的 `.posegraph`；`cartographer` 纯定位**待实跑** | `docs/localization_slots.md` §1 表（含 ✅/⏸/⚠️ 状态列）；`docs/algorithm_matrix.md:25-27` |
| **短时漂移约束（轮速/惯导 EKF）** | ❌ 无 `robot_localization`/EKF 节点；`lio:=none` 这个"接外部 odom"的语义槽在，但**没有外部 odom 提供者** | 轮式里程计融合整条链路 | `docs/algorithm_matrix.md:198`；`docs/algorithm_matrix.md:135`、`docs/architecture.md:628,648`（`lio:=none` 语义）；`docs/nav_strategy_taxonomy.md` §2.1/§2.2（已收录 `robot_localization` 与 Moore & Stouch 出处） |
| **3D 先验（PCD）跨会话续建** | ⚠️ 有但默认关：`cloud_accumulator:=True` 才起（另一个 agent 正在该目录工作，本文不引用其细节） | 未做长跑验收（launch 注释自己写明） | `bringup_sim.launch.py:588-596`（"默认 False：…属新增路径、还没做长跑验收"） |
| **`map→odom` 单发布者约束** | ✅ 已确立为契约（T1–T5）：重定位槽只发 `map→odom`，LIO 只发 `odom→base_link` | 正因如此，**建图期不能同时再挂一个重定位器** | `docs/localization_slots.md` 开头「统一契约」；`docs/tf_interface_contract.md:47-50`（角色表：重定位"不得发布 `odom→base_link`"） |

### 4.3 【重点】我们的"续建" ≠ 重定位

- **事实**：续建 = 若 `map/<map_name>.posegraph` 存在且守卫通过 ⇒ 给 slam_toolbox 注入
  `map_file_name=<base>` + `map_start_pose=[0,0,0]`（`bringup_sim.launch.py:1152-1156`），
  节点起来时 `loadPoseGraphByParams()` 反序列化并**接着建**（上游 `slam_toolbox_common.cpp:607`）。
- **事实**：`map_start_pose` 的语义是"机器人现在在**那张旧图**的哪个位姿"，
  我们的默认值 `[0.0, 0.0, 0.0]` 成立的**前提**是"本工程 `map` 系 = 出生点相对系 ⇒ 机器人在出生点重生"
  （launch 参数说明 `:573-578` 原文就是这个前提）。
- **推断**：因此我们的续建能力 = **"人工锚定"**（§2.1 路径 1），**不是**"重定位"（路径 2/3）。
  它成立的条件很窄：**同场地 + 同出生点 + 上一张图本身没歪**。
  后一条我们已经被咬过一次（`bringup_sim.launch.py:597-604` 那段"2026-10-06 事故当天追加"的注释：
  续建进来的 `.posegraph` 里已含一段 LIO 退化，ATE max 1.15 m ⇒ 图与真实场地不一致）。
- **推断**：要把它变成"重定位"，缺的正是 §2.1 路径 2 的**地点识别 + 精配准**；
  而精配准我们**已经有了并且验收过**（`localization:=gicp`：fitness 0.0017 m²、880/880 帧、PASS，
  见 `docs/mapping_small_point_lio.md` 与 `docs/localization_slots.md` §1）⇒ 缺口其实只有"检索"这一级。

### 4.4 重定位槽（仅 `mode:=nav`）与"建图期加强定位"的冲突点

- **事实**：`localization` 槽**只在 `mode:=nav` 生效**（launch `:444-447` 的 description 原文："仅 mode:=nav 生效"）；
  这些槽的职责被定义为**只发 `map→odom`**，且**同一时刻只能有一个发布者**。
- **事实**：`mode:=mapping` 时 `map→odom` 由 **slam_toolbox（mapper）自己**发布（它同时在算位姿与建图）。
- **推断**：所以"建图期把 `localization:=gicp` 也打开"在现架构下**不可行**（两个 `map→odom` 发布者 ⇒ TF 树非法，
  这正是 `docs/tf_interface_contract.md:32` 记录的 P1 类事故）。
  正确形态是 §5(iv) 的"**定位结果 → 位姿图先验因子**"，而不是"第二个 TF 发布者"。

### 4.5 更正一条：COD 2026 的 `mode: lifelong` 其实是个**无效值**

- **事实**：`third_party/cod_nav_2026/src/cod_bringup/params/mapper_params_online_async.yaml:19` 写的是 `mode: lifelong #localization`；
  但同一份 bringup 起的是 **`executable='async_slam_toolbox_node'`**
  （`third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:85-94`）。
- **事实**：读 `mode` 的只有 Ceres solver，而且只有**一个分支**：`if (mode == std::string("localization"))`
  （上游 `solvers/ceres_solver.cpp:183`；**我们 vendor 的 2.6.10 也是同一套逻辑**：`src/rm_localization/slam_toolbox/solvers/ceres_solver.cpp:143`）。
  ⇒ `lifelong` 这个字符串**既不报错也不改变行为**，等价于默认的 mapping。
- **事实**：真正的 lifelong 是**换 executable**：`lifelong_slam_toolbox_node`
  （上游 `launch/lifelong_launch.py:56` 与 `src/experimental/slam_toolbox_lifelong_node.cpp`；
  我们这份 vendor 里同样带着 `launch/lifelong_launch.py:13` + `src/experimental/slam_toolbox_lifelong.cpp`，**但我们从没起过它**）。
   它的语义是"**允许删节点**的真·lifelong"（上游 README：*"true lifelong mapping that does support the method for removing nodes over time as well as adding nodes"*），
  另外还有一堆 `lifelong_*` 参数（`lifelong_search_use_tree` / `lifelong_minimum_score` / `lifelong_node_removal_score` …，`src/experimental/slam_toolbox_lifelong.cpp:56-101`）。
- **推断**：所以"COD 用了 `mode: lifelong`"这句话**不能当作他们的能力证据**；他们实际跑的是 async 建图（与我们同一条路）。
  顺带一提：真要跑 lifelong，代价是它在**删节点**，`map` 会随会话变形 —— 与"稳定的先验图 + AMCL/纯定位"是两条路线（上游 README 也建议
  "先建完图，再切到 pose-graph 定位模式"）。**我们当前不需要它**（§6）。

---

## §5 该补什么（排序 + 成本 + 出处）

> 排序原则：**先"确认已有的有没有在工作"，再补"整级缺失的那一块"，最后才动架构。**
> 成本是【推断】（按本仓的验证文化估：改动本身小、验收占大头）。

### (i) 先核对/量化 slam_toolbox 的回环参数 —— 成本 ≈ 0.5–1 天（**纯测量，不改代码**）

- **做什么**：跑一次 `mode:=mapping`（`lio:=fastlio` 与 `lio:=small_point_lio` 各一次），用上游事件话题
  **`/slam_toolbox/loop_closure_event`（只在回环被处理时触发）** 与 `/slam_toolbox/pose_graph`（上游 README：**只在回环事件时发布全图**）
  数"这一趟到底闭合过几次"；再决定动不动 `loop_search_maximum_distance`(3.0) / `loop_match_minimum_chain_size`(10)。
  出处：<https://github.com/SteveMacenski/slam_toolbox/blob/33841d040dfff86f9467000bc022336995d56f20/README.md>（事件话题表）；
  参数清单同 README 与 `config/mapper_params_online_async.yaml`。
- **为什么排第一**：这是**唯一"不改任何东西就可能拿到收益"**的一项，而且它的结果决定后面几项的必要性
  （若回环在 12.45×8.65 m 场地里几乎不触发，那么 (ii) 的优先级立刻上升）。
- **我们的既有经验**：同类"上游默认值要不要改"的题目我们做过一次（`docs/slam_toolbox_scan_drops.md`：
  `scan_queue_size` 的 1/5/10 A/B，结论**不改默认**）—— 建议照那个格式写结论（改/不改 + 数字 + 回滚）。
- **风险【推断】**：把半径放大（例如 8–10 m）会同时放大"**近邻但不同地点**"的误闭合风险；
  Karto 的保护是 `loop_match_minimum_response_coarse`(0.35) + `loop_match_maximum_variance_coarse`(3.0) 这道粗门
  （`Mapper.cpp:1523-1525`），**没有任何"回环后图变差了"的自动检测** ⇒ 必须靠"回环计数 + 事后图检查"两个人看。

### (ii) 地点识别（Scan Context 类）—— 成本：离线 A2 约 1–2 天；在线检索节点 + 槽位集成约 3–5 天【推断】

- **为什么是"整级缺失"里的头号**：它是 §2.1 路径 2 的上半段；下半段（精配准）我们**已验收**（`localization:=gicp`）。
  用途三合一：① **真正的多会话/跨出生点续建**（替掉 `map_start_pose=[0,0,0]` 这个人工前提）
  ② **丢失找回**（检索 → `/initialpose` → 已验收的 GICP 槽，零改动复用现成话题）
  ③ 未来"先验图里任意摆位"的导航起手。
- **我们已有的证据链（直接接着做即可）**：
  - 方案与候选对照（含许可证结论："不 vendor，照思路自写 2D 版"）：`docs/scan_context_plan.md`（§0 结论、§A.1–§A.7、§D.1–§D.3、§E.4）
  - A1 资产探针：**GO（有条件）** —— 墙线闭合、墙长 56.8–180.2 m、互/自中位比 5.77×；但"库带 vs 查询带"的高度带必须对齐：
    `docs/scan_context_probe_report.md`（§0 判定表）
  - **同源硬门**：**PASS** —— 真实 `/scan` 与 `PCD/<world>.pcd` 生成的库是同一份几何（逐 bin 平均差 0.269 m vs 错误地点 1.023 m）；
    但"§D.2 严格判据"只到 **INCONCLUSIVE（偏 PASS）**（67.2% / 78.8%，缺口全来自 1.0 m 候选网格量化）⇒ **库要用 0.5 m 网格**：
    `docs/scan_context_samesource_gate_report.md`（§0 判定表）
- **外部出处**：Scan Context（§2.2 行 1）、Scan Context++、STD、BTC —— 以及"SC 回环修大漂移、RS 回环做细缝"的工程经验
  （SC-LIO-SAM README：<https://github.com/gisbi-kim/SC-LIO-SAM/blob/d43ca00d97a756303c10975e32e8d66bfabb337d/README.md>）。
- **风险【推断】**：误检索 = 把"以为回到 A 点"的初值注入 GICP ⇒ 静默收敛到错的地方。
  本仓已有"验收门"设计（plan §B.4）与同源门报告 §6 的量化缺口，**必须先过门再上实时**。

### (iii) 轮式里程计 / EKF 融合 —— 成本：中（新节点 + 契约改动，3–7 天【推断】）

- **作用**：**短时**漂移约束与雷达退化兜底（长走廊、空旷场地），也让"LIO 突然跳一下"不至于直接传到 `odom→base_link`。
- **它不能替代 (i)/(ii)**：EKF 是**局部**平滑，没有绝对基准 ⇒ 不解决回环、不解决跨会话锚定。**推断**。
- **当前障碍（事实）**：我们没有外部轮速源接入；`lio:=none` 这个语义槽留了位置但"没有外部 odom 提供者"
  （`docs/algorithm_matrix.md:198`）。另外若引入 EKF，它会与 LIO 争 `odom→base_link`
  ⇒ 必须二选一：**要么 EKF 在 LIO 之前（轮速+IMU→`odom→base_link`，LIO 退成传感器/建图输入），要么 LIO 在前、EKF 只在仿真外用**
  （单发布者约束：`docs/tf_interface_contract.md:47-50`）。
- **出处**：`robot_localization`（EKF/UKF，ROS 2）：<https://github.com/cra-ros-pkg/robot_localization> ；
  方法出处 Moore & Stouch, *A Generalized Extended Kalman Filter Implementation for the Robot Operating System*：
  <https://link.springer.com/chapter/10.1007/978-3-319-27146-0_25>（本仓已在 `docs/nav_strategy_taxonomy.md` §2.1/§2.2 收录）。
- **《sim_real_contract》提醒**：轮速在仿真里是"干净"的，实车打滑就变成噪声源 —— 引入前先按
  `docs/sim_real_contract.md` 的三分类判据过一遍"这是算法改进还是为仿真妥协"。

### (iv) "把会话锚到先验图"的架构改造（**为什么这是架构改动，不是加个开关**）—— 成本：1–2 周【推断】

- **冲突点（事实）**：`map→odom` **同一时刻只能有一个发布者**（`docs/localization_slots.md` 开头统一契约；
  `docs/tf_interface_contract.md:47-50` 的角色表；`:32` 记录了历史上"多父边/帧树闭环"的事故 P1）。
  建图期这个发布者是 mapper（slam_toolbox）；导航期是 `localization` 槽。**两者不能同时开**。
- **业界正确形态（事实，两家都有源码背书）**：把外部定位当成**位姿图上的先验/一元因子**，而不是第二个 TF 发布者：
  - Cartographer：`POSE_GRAPH.optimization_problem` 里专门有 `fixed_frame_pose_translation_weight` / `fixed_frame_pose_rotation_weight`
    （`configuration_files/pose_graph.lua`）—— 外部坐标系（如动捕/GPS）作为**软先验**进优化。
  - RTAB-Map：localization 模式下"optimization is run on a sub-graph … with pose priors fixing the map nodes"
    （`kRGBDLocalizationPriorError` 控制先验方差）：<https://github.com/introlab/rtabmap/blob/aa95581cd2f0adca514e0fa6747ea4d7ce53d962/corelib/include/rtabmap/core/Rtabmap.h>（见 §2 行 7 的精确链接）。
- **改造内容【推断】**：新增"锚定因子"通道：`localization` 结果（位姿+协方差）→ 建图后端加一条**一元先验因子**（锚），
  同时**只有 mapper 发 `map→odom`**；并且要定义 ①坐标基准（同一 `map` 系）②协方差/权重（对应 cartographer 那两个 weight）
  ③拒绝策略（检索/配准没过门就不加因子）。
- **排最后的原因**：它只有在 (ii) 落地之后才有意义（没有"锚在哪"的来源，就没有可供锚定的观测）；
  且它是全栈契约级改动，验收成本远大于 (i)(ii)。

---

## §6 决策表：你的目标 → 该用什么

| 你的目标 | 该用的机制 | 我们现在的状态 | 具体怎么走 |
|---|---|---|---|
| **短时单场建图**（几分钟、绕一圈、回出生点，当场导航） | 局部回环 + 里程计（够用） | ✅ 有（`do_loop_closing` + Karto 近邻链；另可 RViz 手动回环） | 直接用 `mode:=mapping`；先做 §5(i) 的"回环计数"测量 |
| **长时 / 多次建图**（今天建一半，明天接着建；跨天累积） | **地点识别 + 位姿图锚定**（真正的多会话） | ⚠️ 半有：续建锚定在（`map_autocontinue`/`map_start_pose`），**地点识别不在** | 现状要求"同场地 + 同出生点"；要摆脱这个前提 ⇒ §5(ii) → 再 §5(iv) |
| **换出生点 / 未知道路重入 / 被搬动后继续** | **全局地点识别 + 精配准**（重定位） | ❌ 无自动路径（只能人工 `/initialpose`） | §5(ii)：A2 写 2D 描述子 → 离线自检索 → 接 `/initialpose` 到已验收的 `localization:=gicp`（`docs/scan_context_plan.md` §B.3 就是"零代码改动"的最小接法） |
| **建图中丢失找回**（tracking lost / 打滑 / 快速旋转） | 同上一行（ORB-SLAM2 的 `Relocalization`、slam_toolbox 的 `/initialpose`、AMCL 的初值都是这个东西） | ❌ 自动 = 0；人工 = 有（`/initialpose` 在 `slam_toolbox`/AMCL 槽都通） | §5(ii)；并注意建图期"谁发 `map→odom`"的约束（§4.4） |
| **导航期稳定重复定位**（同一张图反复跑、要可重复） | **先验图上的重定位**（AMCL/GICP/纯定位） | ✅ 7 个槽已登记，`amcl`/`beluga`/`icp`/`gicp`/`small_gicp` 已验收或已有整栈数据；`slam_toolbox`/`cartographer` 纯定位待跑/缺资产 | `docs/localization_slots.md` §1 按资产选槽（不是本文范围） |
| **3D 先验（PCD）复用 / 3D 建图续建** | 里程计 + 3D 先验累积（`cloud_accumulator`）+（可选）描述子 | ⚠️ 有入口、默认关、未长跑验收 | 见 `bringup_sim.launch.py:588-596` 与 `docs/continue_mapping.md`（另一任务在维护该文） |
| **只要 2D 栅格图 / 先验图** | 2D 图 + 2D 重定位（AMCL/纯定位） | ✅ 已有 | `docs/localization_slots.md`、`docs/mapping_2d_from_cloud.md` |
| **"我就是要 COD 那样的 lifelong"** | `lifelong_slam_toolbox_node`（允许删节点的真 lifelong） | ❌ 我们从未起过它（vendor 里代码与 launch 都在） | **【推断】不建议现在做**：它会删节点、`map` 随会话变形，与"稳定先验图 + 纯定位"路线冲突；上游 README 自己也建议"建完图后切到 pose-graph 定位模式" |

---

## §7 未验证 / 未能确认（**不编**）

1. **COD 那份 `mode: lifelong` 在他们实际使用的 slam_toolbox 版本里的行为**：我只核了
   ①上游 `ros2` 分支 HEAD（`33841d04`，2026-10-06）②我们 vendor 的 2.6.10 —— 两者都只有 `mode == "localization"` 一个分支。
   若他们的版本更旧或被打过补丁，§4.5 的结论要重核。
2. **"kidnapped robot problem" 这个英文术语的权威出处**：本次没能取到含该词的原文
   （Cadena 2016 用的是 "Metric Relocalization"；LiDAR 全局定位综述用的是 "global localization" / "global place retrieval"）。
   §1/§6 里我按"业界通称"使用，**没有直接出处**。
3. **`irapkaist/scancontext` 原仓库不可达**（`api.github.com/repos/irapkaist/scancontext` 返回 `Not Found`，
   与 `docs/scan_context_plan.md` §F.1 的记录一致）；**Scan Context 的 IROS 2018 正文我没读**，
   只核了 DOI 解析目标（IEEE 8593698）与 SC-LIO-SAM README 里的 bibtex。
4. **BTC 论文正文未读**：只核了 DOI 解析到 IEEE 10388464 与仓库存在（`hku-mars/btc_descriptor`，默认分支 `master`）；
   它的 `master/README.md` 取回 404 ⇒ **README 内容与输出形式未确认**（`docs/scan_context_plan.md` §F.3/F.4 也把它列为未核实）。
5. **M2DP 论文正文未读**：只核了 DOI 解析到 IEEE 7759260 与仓库存在（`LiHeUA/M2DP`）。
6. **maplab / maplab 2.0 / Kimera-Multi**：只核到 arXiv 标题与摘要页（`1711.10250`、`2212.00654`、`2106.14386`），
   **正文未读** ⇒ §2 表里"多会话/map merging"是按标题与摘要转述，没有逐条代码证据。
7. **我们自己的回环从未被量化过**：没有任何 `/slam_toolbox/loop_closure_event` 计数、也没有"闭合前后 `map` 差多少"的记录。
   §4.1/§5(i) 的判断全部是**读参数与源码语义**得出的，不是实测。
8. **`loop_search_maximum_distance: 3.0` 与 12.45×8.65 m 场地的相互作用**：§4.1 末段是【推断】，
   依据是 Karto 的 `FindPossibleLoopClosure` 语义，**没有实测支撑**。
9. **本文没有任何运行期观测**：没有起 ROS、没跑仿真、没读 bag。所有"我们栈"的结论都来自仓库文件与既有文档记录的数字
   （文档里的实测数字都已注明出处）。因此"回环到底有没有生效"这类问题，本文只能给判据、不能给答案。
10. **RTAB-Map 头文件取自 `master`（`aa95581c`）**：其注释详略可能与该仓库的发布版（0.21/0.22）不同；
    我引用的是**参数名与 API**（`Mem/IncrementalMemory`、`getLastLocalizationPose`、`kRGBDLocalizationPriorError`），不是注释原文的措辞。
11. **slam_toolbox 的 `mode` 在 ROS 1 版本（`kinetic-devel` 分支）里的语义未核**：ROS 1 里可能存在别的取值分支，
    这不影响 §4.5 对 ROS 2 侧（上游 HEAD + 我们 2.6.10）的结论。
12. **`enable_interactive_mode: true` 的实际内存代价未量**：上游 README 说 localization/lifelong 模式会强制关掉它
    （"valid for either mapping or continued mapping modes"），我们建图期开着 ⇒ 会保留扫描缓存；**多大开销未测**。

---

## 附录 A：外部参考链接清单（2026-10-06 抓取，代码类 URL 均 pin 到 commit）

**SLAM 系统源码/文档**

- ORB-SLAM2（`@f2e6f51`）：<https://github.com/raulmur/ORB_SLAM2/tree/f2e6f51cdc8d067655d90a78c06261378e07e8f3> ·
  `Tracking.cc#L1341`（`Relocalization`）· `Tracking.h#L81`（状态枚举）· `LoopClosing.cc#L141`（DBoW2 候选）· 论文 <https://arxiv.org/abs/1610.06475>
- ORB-SLAM3（`@4452a3c`）：<https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4> ·
  `Tracking.cc#L3609` · `LoopClosing.cc#L1215`（`MergeLocal`）· `#L1783`（`MergeLocal2`）· 论文 <https://arxiv.org/abs/2007.11898>
- LIO-SAM（`@0be1fbe`）：<https://github.com/TixiaoShan/LIO-SAM/tree/0be1fbe6275fb8366d5b800af4fc8c76a885c869> ·
  `src/mapOptmization.cpp#L503` · `config/params.yaml#L82` · 论文 <https://arxiv.org/abs/2007.00258>
- SC-LIO-SAM（`@d43ca00`）：<https://github.com/gisbi-kim/SC-LIO-SAM/tree/d43ca00d97a756303c10975e32e8d66bfabb337d> ·
  `SC-LIO-SAM/include/Scancontext.h` · `SC-LIO-SAM/src/mapOptmization.cpp`
- FAST-LIO（`@7cc4175`）：<https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9>（README 里的第三方 `FAST-LIO-LOCALIZATION`）
- FAST-LIO-SAM（`@6ec8b8f`）：<https://github.com/engcang/FAST-LIO-SAM/tree/6ec8b8f537aa33162856462f17b09f1f91e0eafc>
- Point-LIO：<https://github.com/hku-mars/Point-LIO/blob/point-lio-with-grid-map/README.md>
- Cartographer（`@877157a`）：<https://github.com/cartographer-project/cartographer/tree/877157a0d91788a7700221d87232d412cb3c1ef4> ·
  `mapping/pose_graph_trimmer.h#L67` · `mapping/map_builder.cc#L56` · `configuration_files/pose_graph.lua` ·
  `mapping/internal/constraints/constraint_builder_2d.cc#L180`
- cartographer_ros（`@c138034`）：<https://github.com/cartographer-project/cartographer_ros/tree/c138034db0c47fe0ea5a2abe516acae02190dbf5> ·
  `cartographer_ros/configuration_files/backpack_2d_localization.lua` · `cartographer_ros/node_main.cc#L35`
- Cartographer 官方文档：<https://google-cartographer.readthedocs.io/en/latest/>（论文引用）·
  <https://google-cartographer-ros.readthedocs.io/en/latest/demos.html#pure-localization>（纯定位跑法）·
  <https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html#localization-only>（Localization only / `use_odometry`）
- Cartographer 论文 DOI：<https://doi.org/10.1109/ICRA.2016.7487258>
- RTAB-Map（`@aa95581`）：<https://github.com/introlab/rtabmap/tree/aa95581cd2f0adca514e0fa6747ea4d7ce53d962> ·
  `corelib/include/rtabmap/core/Rtabmap.h` · `corelib/include/rtabmap/core/Memory.h` · 论文（JFR 2019 / arXiv 版）<https://arxiv.org/abs/2403.06341>
- slam_toolbox（`@33841d0`）：<https://github.com/SteveMacenski/slam_toolbox/tree/33841d040dfff86f9467000bc022336995d56f20> ·
  README（lifelong / localization / map merging / 事件话题）· `config/mapper_params_online_async.yaml` ·
  `config/mapper_params_localization.yaml` · `solvers/ceres_solver.cpp#L67`、`#L183` ·
  `src/slam_toolbox_common.cpp#L607` · `src/slam_toolbox_localization.cpp#L81` · `src/merge_maps_kinematic.cpp#L51` ·
  `launch/lifelong_launch.py#L56` · `launch/localization_launch.py#L57` · 论文（JOSS）<https://joss.theoj.org/papers/10.21105/joss.02783>
- hdl_graph_slam（`@95b8dce`）：<https://github.com/koide3/hdl_graph_slam/tree/95b8dce41c10667dcf0e3e9801dc22754b3bd525>
- interactive_slam（`@4e3cb87`）：<https://github.com/koide3/interactive_slam/tree/4e3cb87edc0ab7c735e0aa85e1fd985b2aaa6841>

**多会话 / 多机 / 综述**

- maplab：<https://arxiv.org/abs/1711.10250> · maplab 2.0：<https://arxiv.org/abs/2212.00654>
- Kimera-Multi（T-RO）：<https://arxiv.org/abs/2106.14386> · （ICRA 2021）：<https://arxiv.org/abs/2011.04087>
- Cadena et al. 2016（"Metric Relocalization"）：<https://arxiv.org/abs/1606.05830>
- LiDAR 全局定位综述（"global place retrieval + local pose estimation"）：<https://arxiv.org/abs/2302.07433>

**地点识别描述子**

- Scan Context：<https://doi.org/10.1109/IROS.2018.8593698>（bibtex 见 SC-LIO-SAM README）
- Scan Context++：<https://arxiv.org/abs/2109.13494> · <https://github.com/gisbi-kim/scancontext_tro>
- STD：<https://arxiv.org/abs/2209.12435> · <https://github.com/hku-mars/STD>
- BTC：<https://doi.org/10.1109/TRO.2024.3353076> · <https://github.com/hku-mars/btc_descriptor>
- M2DP：<https://doi.org/10.1109/IROS.2016.7759260> · <https://github.com/LiHeUA/M2DP>
- NetVLAD：<https://arxiv.org/abs/1511.07247> · PointNetVLAD：<https://arxiv.org/abs/1804.03492>

**轮式里程计 / EKF**

- `robot_localization`：<https://github.com/cra-ros-pkg/robot_localization>
- Moore & Stouch（EKF for ROS）：<https://link.springer.com/chapter/10.1007/978-3-319-27146-0_25>

## 附录 B：本文引用的本仓文件

- `src/rm_nav_bringup/launch/bringup_sim.launch.py`（槽位与续建参数：`:431` `:444` `:460-473` `:521-525` `:558-596` `:597-604` `:1120-1162`）
- `src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml`（`:17` mode、`:36` interactive、`:47-62` 回环参数）
- `src/rm_localization/slam_toolbox/config/mapper_params_localization_sim.yaml`（`:15` `mode: localization`）
- `src/rm_localization/slam_toolbox/solvers/ceres_solver.cpp:143`；`src/slam_mapper.cpp:172-273`；`lib/karto_sdk/src/Mapper.cpp:1507-1526,1646,1962-2007`
- `src/rm_localization/slam_toolbox/launch/lifelong_launch.py:13`；`src/experimental/slam_toolbox_lifelong.cpp:56-101`；`src/merge_maps_kinematic.cpp:51`
- `src/rm_localization/small_point_lio/`（`README.md`、`src/small_point_lio_node.cpp:62-71`、`src/small_point_lio/preprocess.cpp:68`）
- `tools/scripts/mapping/map_archive.sh`
- `third_party/cod_nav_2026/src/cod_bringup/params/mapper_params_online_async.yaml:19`；`.../launch/multiplenav_launch.py:85-94`
- 文档：`docs/localization_slots.md`、`docs/tf_interface_contract.md`、`docs/algorithm_matrix.md`、`docs/lio_slots.md`、
  `docs/mapping_small_point_lio.md`、`docs/mapping_2d_from_cloud.md`、`docs/slam_toolbox_scan_drops.md`、
  `docs/architecture.md`、`docs/rm_algorithm_catalog.md`、`docs/nav_strategy_taxonomy.md`、`docs/sim_real_contract.md`、
  `docs/continue_mapping.md`、`docs/scan_context_plan.md`、`docs/scan_context_probe_report.md`、`docs/scan_context_samesource_gate_report.md`、
  `docs/cod_nav_comparison.md`
