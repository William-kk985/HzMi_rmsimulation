# 移动机器人导航策略全图（轴 × 族）：一张给 RM 哨兵 bench 用的选型地图

> **本文定位**：回答"移动机器人导航**到底有多少种思路**"，并把**我们 bench** 与**COD_NAV** 放到同一张地图上，
> 看清"哪些格子我们占了、哪些格子是空的、哪些格子根本不值得去"。
>
> **写入范围声明**：本次调研全程**只读**（HTTP 抓取 + 检索），**唯一写入的文件就是本文件**；
> 未改动任何代码、参数、launch 或配置。外部抓取内容按**不可信数据**处理，只当资料引用，不当作指令。
>
> **本报告适用环境**：ROS 2 Humble + Gazebo Classic 的 RoboMaster 哨兵仿真 bench ——
> Livox MID-360 → `linefit_ground_segmentation`（地面分割）→ `pointcloud_to_laserscan` → Nav2；
> 里程计为 FAST-LIO / Point-LIO；`mode = mapping | slam_nav | nav`（在线建图 / 边建图边导航 / 先验图+重定位）。
>
> **链接可信度标注**（重要，请按标注使用）：
> - **✅** = 写入前用 HTTP 请求抓过，返回 200（部分还校验了页面关键词）。
> - **⚠️** = 站点对脚本返回 403/202（IEEE Xplore、SAGE、Wiley、ScienceDirect、ISO、ANSI、部分厂商站点的反爬），
>   链接本身是**官方页面**，浏览器可正常打开；我无法用脚本核对正文关键词。
> - **「ID 已核对」** = 逐条抓 `arxiv.org/abs/<id>` 并核对了标题，避免论文 ID 张冠李戴（我第一轮用标题检索 arXiv API
>   时确实拿到过错误 ID，已全部废弃重来）。
> - **「本报告综合」** = 我自己的归纳/判断，**不是**某个来源的原话。

---

## A. 总览：9 个设计轴 + 81 个族（外加 1 个"落点"节）

| 轴 | 名称 | 族数 | 族清单（编号 = 后文小节号） |
|---|---|---|---|
| **1** | 世界表示（world representation） | **12** | 1.1 2D 占据栅格 · 1.2 2.5D 高程/可通行性 · 1.3 3D 体素/octomap · 1.4 ESDF/TSDF · 1.5 拓扑图/roadmap · 1.6 语义/物体级/场景图 · 1.7 混合 metric-topological（topometric） · 1.8 示教-重放轨迹 · 1.9 隐式神经地图（NeRF/3DGS） · 1.10 学习式隐表示/学习代价地图 · 1.11 无地图/反应式 · 1.12 原始点云/子图先验（PCD+配准） |
| **2** | 定位与状态估计 | **9** | 2.1 航位推算/轮式里程计 · 2.2 轮+IMU EKF/UKF · 2.3 LIO/VIO · 2.4 先验图上的扫描匹配/粒子滤波 · 2.5 地点识别/回环 · 2.6 GNSS/RTK · 2.7 UWB/信标/二维码/磁条 · 2.8 语义/地标锚点 · 2.9 **不做重定位**（静态 `map→odom` / 手工初值） |
| **3** | 全局路径 / 任务级规划 | **10** | 3.1 图搜索（Dijkstra/A\*/D\* Lite/Theta\*） · 3.2 Hybrid A\*/state lattice（Smac） · 3.3 NavFn/势场栅格 · 3.4 采样式（RRT/RRT\*/PRM/BIT\*） · 3.5 轨迹优化（CHOMP/STOMP/TrajOpt/GPMP2/TEB） · 3.6 覆盖规划 · 3.7 前沿/探索规划 · 3.8 拓扑路由 · 3.9 多目标/TSP 排序 · 3.10 MPC 式全局规划 |
| **4** | 局部规划与控制 | **11** | 4.1 Pure Pursuit · 4.2 Stanley · 4.3 Regulated Pure Pursuit · 4.4 DWA/DWB · 4.5 TEB · 4.6 MPPI · 4.7 MPC（acados/casadi/`mpc_local_planner`） · 4.8 RL 局部策略 · 4.9 速度平滑/指令整形 · 4.10 运动学模型（差速/全向/Ackermann/腿足） · 4.11 反应式避障（势场/VFH） |
| **5** | 任务指定方式 | **10** | 5.1 无目标（探索） · 5.2 单目标 · 5.3 航点序列/through-poses · 5.4 巡逻环线 · 5.5 覆盖任务 · 5.6 示教-重放 · 5.7 语义/自然语言指令 · 5.8 示教学习（LfD/IL） · 5.9 行为树/FSM 任务层 · 5.10 任务/车队协议 |
| **6** | 不确定性与安全 | **7** | 6.1 未知区策略 · 6.2 重规划与恢复行为 · 6.3 碰撞监控/安全监控/限速 · 6.4 风险感知与机会约束 · 6.5 主动 SLAM/不确定性驱动探索 · 6.6 安全标准与"安全链独立" · 6.7 安全型传感器与安全 PLC |
| **7** | 学习 / 端到端 / 基础模型 | **8** | 7.1 RL 导航 · 7.2 基准与仿真器 · 7.3 模仿学习 · 7.4 Sim-to-real · 7.5 基础模型导航 · 7.6 世界模型 · 7.7 扩散策略 · 7.8 LLM/VLM 任务规划 |
| **8** | 工业 AGV/AMR 范式 | **8** | 8.1 磁条/磁带 · 8.2 二维码/标记 · 8.3 反射板激光三角定位 · 8.4 示教-重放 AGV · 8.5 SLAM AMR · 8.6 车队管理 · 8.7 VDA5050 · 8.8 为什么工业偏好固定路径+拓扑图 |
| **9** | 多机/协同 | **6** | 9.1 编队控制 · 9.2 任务分配 · 9.3 多机 SLAM/地图融合 · 9.4 多机前沿探索 · 9.5 通信约束下的协同 · 9.6 ROS 2 多机部署 |
| **10** | **RM 哨兵生态落点**（不是设计轴，是"坐标系落点"） | 10 个仓库 | 见 §B.10 |
| | **合计** | **9 轴 / 81 族** | |

**先给结论，避免误读（本报告综合）**：
轴与族是**"词汇表"**，不是**"可自由排列组合的菜单"**。81 个族里绝大多数组合在工程上不成立
（例如"磁条导引 + NeRF 建图"、"无重定位 + 覆盖规划 + 高速动态避障"）。
真实产品/强队配置收敛到 **5~8 种标准组合**（§D），其余族要么是研究前沿，要么是某个组合里的**可替换插槽**。
`docs/algorithm_axes.md` 里"120 组合"的说法也是同一个意思：**组合数是插槽相乘的假象，不是 120 个独立方案**。

### 计数小结（直接回答"到底有多少种思路"）

- **轴数**：**9 个设计轴**（世界表示 / 定位与状态估计 / 全局规划 / 局部控制 / 任务指定 / 不确定性与安全 / 学习与基础模型 / 工业 AGV-AMR 范式 / 多机协同），
  外加第 10 节作为"**RM 哨兵生态落点**"（不是设计轴，是别人已经站在哪一格）。
- **族数**：**81 族**（12 + 9 + 10 + 11 + 10 + 7 + 8 + 8 + 6 = 81；可用 `grep -cE '^\*\*[0-9]+\.[0-9]+ ' docs/nav_strategy_taxonomy.md` 复核，
  本文件当前正好 81 条族条目）。
- **"能实际选的路"数**：**5~8 种标准产品化组合**（§D 给出 8 种；其中 D1–D3 是量产/强队主流，去掉研究向的 D6/D7 与多机 D8，剩下的就是 **5 种**）。
- **单机比赛可用的"现实自由度"（本报告综合）**：约 **2~3 种组合 × 3~4 个插槽** ——
  即"在线建图边建边导航（D4）"或"先验图+重定位（D2/D3）"二选一，再在【里程计 / 重定位 / 全局规划器 / 局部控制器 / 任务层】里换实现。
  **不是 81 选 1，也不是任意排列组合。**

---

## B. 逐轴展开

> 每族的固定四行：**是什么** / **何时用** / **ROS(2) 实现** / **来源**。
> 「来源」里每条链接都带 ✅/⚠️ 标注；族内只要 ≥1 条可点链接即满足"每族有权威来源"。

### B.1 轴 1：世界表示（12 族）

**1.1 2D 占据栅格（occupancy grid / costmap）**
- **是什么**：把世界压成 `resolution≈0.03~0.05 m` 的二维概率栅格（free / occupied / unknown），再叠加膨胀、体素投影、禁行区等"代价层"；导航栈的事实标准。
- **何时用**：场地平坦、障碍物贴地、机器人是 2D 运动学；**几乎所有 2D 导航栈的默认选择**，也是 RM 场地（平地+矮墙）最划算的表示。
- **ROS 2 实现**：`nav2_costmap_2d`（static/obstacle/inflation/voxel 层）+ `nav2_map_server`；ROS 1 对应 `map_server`+`costmap_2d`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/map_server/configuring_map_server/>

**1.2 2.5D 高程 / 可通行性图（elevation / traversability）**
- **是什么**：2.5D = 每个 2D 格子存一个高度（或高度带、坡度、台阶高度、粗糙度），从"障碍/非障碍"升级为"能不能过、以什么代价过"。
- **何时用**：有斜坡/台阶/减速带/可越障矮台；腿足与越野机器人必备；RM 场地里对应"高地、斜坡、能量机关台阶"。
- **ROS 2 实现**：`elevation_mapping`（ANYbotics）、`elevation_mapping_cupy`（GPU 版）、`grid_map`；RM 生态常用 `terrain_analysis` / `terrain_analysis_ext`（把离地高度写进点云 `intensity`，再喂 costmap 层）。
- **来源**：✅ <https://github.com/ANYbotics/elevation_mapping> ；✅ <https://github.com/leggedrobotics/elevation_mapping_cupy> ；✅ <https://github.com/ANYbotics/grid_map> ；✅ <https://github.com/HongbiaoZ/autonomous_exploration_development_environment> ；✅ <https://github.com/fabioruetz/wild_visual_navigation>（学习式可通行性，与 1.10 共用）；✅ arXiv 2305.08510「ID 已核对」

**1.3 3D 体素 / octomap（含时间维）**
- **是什么**：八叉树或体素栅格存三维占据概率，能表达悬空结构（横梁、桌下）、并能按高度带切层投影给 2D 规划器。
- **何时用**：三维结构重要、点云多源（雷达+深度相机）、需要"障碍随时间消失"（动态障碍残影）。
- **ROS 2 实现**：`octomap`（3D 概率八叉树）；Nav2 `voxel_layer`（costmap 的 3D 体素层）；`spatio_temporal_voxel_layer`（STVL，带时间衰减，COD 与我们都在用）。
- **来源**：✅ <https://octomap.github.io/> ；✅ <https://github.com/octomap/octomap> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/> ；✅ <https://github.com/SteveMacenski/spatio_temporal_voxel_layer>

**1.4 ESDF / TSDF（欧氏符号距离场 / 截断符号距离场）**
- **是什么**：不存"是否占据"，而存"到最近障碍的距离与梯度"，规划器可直接做梯度下降、碰撞代价连续可微。
- **何时用**：需要平滑、可微、考虑机器人尺寸的轨迹优化（无人机、机械臂、腿足）；纯 2D 栅格导航用不上。
- **ROS 2 实现**：`voxblox`（TSDF→ESDF，ROS 1 为主，有 ROS 2 分支/衍生）、`nvblox`（NVIDIA GPU 版 TSDF/ESDF，ROS 2）、`FIESTA`（增量式 ESDF，无人机）。
- **来源**：✅ <https://github.com/ethz-asl/voxblox> ；✅ <https://github.com/nvidia-isaac/nvblox> ；✅ <https://github.com/HKUST-Aerial-Robotics/FIESTA> ；✅ <https://github.com/ethz-asl/mav_voxblox_planning> ；✅ arXiv 1611.03631（voxblox 论文，**ID 已核对**）

**1.5 拓扑图 / roadmap（图结构表示）**
- **是什么**：世界被抽象成"节点+边"（路口、房间、增益点）+ 可通行代价，不存几何细节；PRM 是最经典的 roadmap 构造法。
- **何时用**：大范围、结构化环境；需要"任务级"决策（去哪、按什么顺序）；地图维护成本低。
- **ROS 2 实现**：OMPL（PRM/PRM\*/RRT 家族的库，也被 MoveIt 等复用）；Nav2 `nav2_route` 的 route graph；SBPL（搜式规划库，lattice/图搜索）。
- **来源**：✅ <https://ompl.kavrakilab.org/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/sbpl/sbpl>

**1.6 语义 / 物体级地图 / 3D 场景图**
- **是什么**：地图里带上"这是什么"（门、箱子、增益点、敌人）以及物体间关系（场景图），供语言指令与任务推理使用。
- **何时用**：需要按语义找目标（"去补给点"）、人机协作、长期运维；算力与标注成本高，比赛里通常只在研究 Demo 阶段。
- **ROS 2 实现**：`Hydra`（实时场景图）、`Kimera-Semantics`（度量-语义）、`ConceptGraphs`（开放词表 3D 场景图）。
- **来源**：✅ <https://github.com/MIT-SPARK/Hydra> ＋ ✅ arXiv 2201.13360（Hydra 论文，**ID 已核对**）；✅ <https://github.com/MIT-SPARK/Kimera-Semantics> ＋ ✅ arXiv 1910.02490；✅ <https://github.com/concept-graphs/concept-graphs> ＋ ✅ arXiv 2309.16650；✅ arXiv 2501.05750（室内具身语义建图综述，**ID 已核对**）

**1.7 混合 metric-topological（topometric）**
- **是什么**：底层保留度量栅格（局部避障、精确到位），上层挂拓扑图/路线（全局决策、跨区导航）——"两层地图"。
- **何时用**：大场景 + 需要重复精度；工业 AMR 与"示教路线"产品的常见内部结构；也是 Nav2 route graph 的设计哲学。
- **ROS 2 实现**：Nav2 `nav2_route`（route graph + costmap 混合）；STRANDS `topological_navigation`（ROS 1 经典实现，ROS 2 生态仍有移植）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/strands-project/strands_navigation> ；✅ arXiv 2501.05750（topometric 术语与综述语境）；⚠️ <https://www.annualreviews.org/content/journals/10.1146/annurev-control-032724-020548>（"Local Maps Are All You Need: A Review of Topometric Teach and Repeat Navigation"，Annual Review of Control, Robotics, and Autonomous Systems）

**1.8 示教-重放轨迹（teach-and-repeat route representation）**
- **是什么**：地图不是栅格，而是**一条或多条被示教过的轨迹/路线**（含关键帧、局部子图、路口标记），运行时沿路线重放并在局部做避障。
- **何时用**：路线固定、环境外观会变（光照/季节/人流）、不想做全局度量建图；巡检、农业、工业送料。
- **ROS 2 实现**：`norlab-ulaval/wiln`（激光 teach-and-repeat 框架，面向恶劣天气户外）；ROS 1 时代有 `teach_repeat_*`、route-following 系列包；Nav2 里可用 `nav2_route` + 录制路径近似实现。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；⚠️ Annual Reviews topometric teach-and-repeat 综述（同上）；⚠️ <https://onlinelibrary.wiley.com/doi/10.1002/rob.20342>（Furgale & Barfoot, "Visual Teach and Repeat for Long-Range Rover Autonomy", JFR 2010）

**1.9 隐式神经地图（NeRF / 3D Gaussian Splatting）**
- **是什么**：用神经网络或 3D 高斯基元表示场景的密度/颜色/几何，可渲染出照片级视图，并从中导出安全走廊/占据查询。
- **何时用**：研究阶段；需要视觉逼真重建 + 在其中做规划（无人机穿越、室内语义导航）；实时性与内存是硬约束。
- **ROS 2 实现**：Splat-Nav（3DGS 地图上的安全实时导航，论文+仓库）；MonoGS（单目高斯泼溅 SLAM）；NeRF 本身只解决重建与渲染，不直接给规划接口。
- **来源**：✅ arXiv 2403.02751（Splat-Nav，**ID 已核对**）；✅ <https://github.com/muskie82/MonoGS> ；✅ arXiv 2003.08934（NeRF 原始论文，**ID 已核对**）

**1.10 学习式隐表示 / 学习代价地图**
- **是什么**：不显式定义"障碍"，而是用学习模型输出代价/可通行性/隐状态（latent），供规划器或策略消费；常以自定义 costmap 层或独立评分网络落地。
- **何时用**：语义/社交场景（人的舒适距离、草地可通行性）、传感器噪声大、规则难以手写代价时；可解释性与安全验证是弱点。
- **ROS 2 实现**：Nav2 插件机制里写自定义 costmap layer / controller critic（官方有插件教程）；研究实现如 Wild Visual Navigation（自监督可通行性）、Learning Social Cost Functions（人机共处代价函数）、DreamerV3（隐空间世界模型）。
- **来源**：✅ <https://github.com/fabioruetz/wild_visual_navigation> ＋ ✅ arXiv 2305.08510；✅ arXiv 2407.10547（Learning Social Cost Functions，**ID 已核对**）；✅ arXiv 2301.04104（DreamerV3，**ID 已核对**）

**1.11 无地图 / 反应式（mapless / reactive）**
- **是什么**：不维护任何全局地图，只靠当前传感器做反应式决策：势场法、VFH 直方图、Bug 系列算法、沿墙走。
- **何时用**：极低成本、极简场景（扫地机早期、循线车）、或作为大栈的兜底安全层；没有全局最优性，会困在局部极小/长死胡同。
- **ROS 2 实现**：Nav2 层面可用 `nav2_rotation_shim_controller` + 简单控制器近似；ROS 1 生态有 `voxel_grid`/`VFH` 类包；多数团队直接自写。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/027836498600500106>（Khatib 1986 人工势场，IJRR）；⚠️ <https://ieeexplore.ieee.org/document/88148>（Borenstein & Koren, VFH, IEEE T-RA 1991）；⚠️ <https://dl.acm.org/doi/abs/10.1007/BF01840369>（Lumelsky & Stepanov, Bug 算法, Algorithmica 1987）

**1.12 原始点云 / 子图先验（PCD + 配准，不经栅格化）**
- **是什么**：先验地图就是一份（或几份）**点云/PCD**，定位靠直接配准（GICP/ICP/NDT），导航再临时投影出局部栅格。
- **何时用**：有离线建好的 3D 点云但不想维护 2D 图；需要 3D 精度；也是 RM 强队常见套路（"PCD 先验 + ICP 重定位"）。
- **ROS 2 实现**：`small_gicp`（快速 GICP，常被 RM 队用作重定位后端）、PCL 的 NDT/ICP、`icp_registration`（社区常用 PCD 配准节点）。
- **来源**：✅ <https://github.com/koide3/small_gicp> ；✅ <https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization> ；✅ <https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html>（PCL NDT 官方教程）；✅ <https://github.com/PointCloudLibrary/pcl>

### B.2 轴 2：定位与状态估计（9 族）

**2.1 航位推算 / 轮式里程计**
- **是什么**：由编码器转速积分出位姿；误差随时间无界增长（打滑、轮径误差）。
- **何时用**：短时局部控制、EKF 的预测项、LIO 失效兜底；**绝不能单独承担全局定位**。
- **ROS 2 实现**：`ros2_control` 的 `diff_drive_controller`（发布 `odom→base_link` 与 `/odom`）。
- **来源**：✅ <https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>

**2.2 轮 + IMU 的 EKF/UKF 融合**
- **是什么**：以 EKF/UKF 融合轮速、IMU、（可选）GPS/视觉，输出平滑的 `odom→base_link` 与协方差。
- **何时用**：没有 LIO/VIO 或需要高频鲁棒里程计时；工业 AMR 的标配，也是 LIO 的对照基线。
- **ROS 2 实现**：`robot_localization`（`ekf_node` / `ukf_node` / `navsat_transform_node`）。
- **来源**：✅ <https://github.com/cra-ros-pkg/robot_localization> ；✅ <https://link.springer.com/chapter/10.1007/978-3-319-27146-0_25>（Moore & Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System"）

**2.3 LIO / VIO（激光/视觉惯性里程计）**
- **是什么**：紧耦合 LiDAR/相机 + IMU，直接配准建图并输出高频高精度里程计；RM 哨兵当前主流（FAST-LIO / Point-LIO / Small Point-LIO）。
- **何时用**：需要 3D 精度、点云配准、无 GPS 的室内外场景；退化场景（长走廊、空旷场地）与 IMU 初始化是主要风险点。
- **ROS 2 实现**：FAST-LIO / FAST-LIO2、Point-LIO、LIO-SAM、VINS-Fusion（视觉惯性）、OpenVINS。
- **来源**：✅ <https://github.com/hku-mars/FAST_LIO> ＋ ✅ arXiv 2107.06829（FAST-LIO2，**ID 已核对**）；✅ <https://github.com/hku-mars/Point-LIO> ；✅ <https://github.com/TixiaoShan/LIO-SAM> ＋ ✅ arXiv 2007.00258；✅ <https://github.com/HKUST-Aerial-Robotics/VINS-Fusion> ；✅ <https://github.com/rpng/open_vins>

**2.4 先验图上的扫描匹配 / 粒子滤波（重定位）**
- **是什么**：在**已有地图**上求 `map→odom`：粒子滤波（AMCL）、点云配准（NDT/GICP/ICP）、或图优化式定位（Cartographer 纯定位、slam_toolbox localization 模式）。
- **何时用**：有先验图、要重复精度、要"关机再开机仍在同一坐标系"；是 `mode:=nav` 的核心环节。
- **ROS 2 实现**：Nav2 `nav2_amcl`；`cartographer_ros`（`-load_state` 纯定位）；`slam_toolbox`（localization 模式）；`small_gicp` / `icp_registration`（3D 配准）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/> ；✅ <https://google-cartographer-ros.readthedocs.io/en/latest/> ＋ ✅ <https://github.com/cartographer-project/cartographer_ros> ；✅ <https://github.com/SteveMacenski/slam_toolbox> ＋ ✅ <https://joss.theoj.org/papers/10.21105/joss.02783>（slam_toolbox 论文, JOSS）；✅ <https://github.com/koide3/small_gicp>

**2.5 地点识别 / 回环（place recognition & loop closure）**
- **是什么**：用外观/结构描述子判断"我来过这里"，触发回环约束修正累计漂移；也是"重定位到先验图某处"的基础设施。
- **何时用**：大规模、长时间、重复路线；单次短程比赛可不用，但**没有它就没有全局一致性**。
- **ROS 2 实现**：DBoW2/DBoW3（词袋，视觉）、Scan Context / Scan Context++（3D 点云描述子）；常嵌在 Cartographer/LIO-SAM/slam_toolbox 内部。
- **来源**：✅ <https://github.com/dorian3d/DBoW2> ；✅ <https://github.com/gisbi-kim/scancontext_tro> ＋ ✅ arXiv 2109.13494（Scan Context++，**ID 已核对**）

**2.6 GNSS / RTK**
- **是什么**：卫星定位（RTK 固定解可达厘米级），提供全局绝对坐标，但室内/遮蔽环境失效。
- **何时用**：户外大场景、园区物流；室内比赛场地无信号，只能当"可选传感器槽"。
- **ROS 2 实现**：`robot_localization` 的 `navsat_transform_node`（把经纬度转 `map` 系并融合进 EKF）；Nav2 的 GPS 跟随教程（把 GPS 路径点转成 Nav2 目标）；RTKLIB（开源 RTK 解算）。
- **来源**：✅ <https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/> ；✅ <https://github.com/cra-ros-pkg/robot_localization> ；✅ <http://www.rtklib.com/>

**2.7 UWB / 信标 / 二维码 / 磁条**
- **是什么**：靠**人工布置的基础设施**定位：UWB 基站测距、二维码/标签（ArUco/AprilTag）绝对位姿、磁条/磁钉循迹。
- **何时用**：室内工业、可改造场地、需要"绝对且可认证"的定位（反射板/磁条可做安全型定位）；缺点是**必须布场**。
- **ROS 2 实现**：`apriltag_ros` / OpenCV ArUco（视觉标签）；各厂商 UWB/磁条驱动多为私有；磁条与反射板见 §B.8。
- **来源**：✅ <https://github.com/AprilRobotics/apriltag> ；⚠️ <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>（OpenCV ArUco 官方教程）；磁条/反射板官方资料见 8.1/8.3

**2.8 语义 / 地标锚点**
- **是什么**：把地图里的语义物体（门牌、标牌、充电桩、增益点）当作长期地标，用检测结果约束位姿。
- **何时用**：几何退化（长走廊、空旷）时提供额外约束；与 1.6 语义地图配套。
- **ROS 2 实现**：无"官方一站式"实现；通常自写节点把检测位姿作为 `PoseWithCovariance` 送进 EKF/因子图（GTSAM/Ceres）。
- **来源**：✅ arXiv 1606.05830（Cadena et al., "Past, Present, and Future of SLAM"，**ID 已核对**，语义 SLAM 章节）；✅ <https://github.com/AprilRobotics/apriltag> ；✅ <https://github.com/concept-graphs/concept-graphs>

**2.9 不做重定位：静态 `map→odom` / 手工初值** ⭐（COD_NAV 2026 就在这一格）
- **是什么**：不估计 `map→odom`，而是**发布一个静态变换**（或依赖上一次位姿/手工对齐），把 `odom` 当成 `map` 用。
- **何时用**：在线 SLAM 一直开着（`map` 随车实时重建，坐标原点固定在同一处）、单场次短时任务、或"每局重开+人工摆位"；**代价是无法恢复丢失的全局位姿**。
- **ROS 2 实现**：`tf2_ros static_transform_publisher`（COD `rmul2026` 分支正是如此）；Nav2 侧无 `amcl`/无 localization 节点。
- **来源**：✅ <https://github.com/qza36/COD_NAV> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation>（README 明确"slam_toolbox 同时导航建图"+静态 TF 设计）

### B.3 轴 3：全局路径 / 任务级规划（10 族）

**3.1 图搜索（Dijkstra / A\* / D\* Lite / Theta\*）**
- **是什么**：在离散图上做最优搜索；A\* 用启发式加速，D\* Lite 支持增量重规划（环境变化时只修局部），Theta\* 允许任意角度（不贴格）。
- **何时用**：栅格全局规划的基本盘；D\* Lite 适合地图持续变化；Theta\* 适合"讨厌锯齿路径"的场合。
- **ROS 2 实现**：Nav2 `nav2_navfn_planner`（NavFn，Dijkstra/势场式）、`nav2_theta_star_planner`（Theta\*）；ROS 1 `global_planner`（A\*/Dijkstra 可切换）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/> ；✅ arXiv 1401.3843（Theta\*，**ID 已核对**）；✅ <http://www.cs.cmu.edu/~maxim/files/dlite_icra02.pdf>（D\* Lite 作者页 PDF）

**3.2 Hybrid A\* / state lattice（Nav2 Smac 家族）**
- **是什么**：在满足车辆运动学（最小转弯半径、前进/倒车）的离散动作空间里搜索，输出**可被底盘跟踪**的平滑路径；state lattice 是其推广（预生成动作图）。
- **何时用**：Ackermann/差速非全向机器人、需要倒车/掉头、路径必须可执行；全向底盘可退化用 2D 版本。
- **ROS 2 实现**：Nav2 `SmacPlanner2D` / `SmacPlannerHybrid`（Dubins/Reeds-Shepp）/ `SmacPlannerLattice`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner>

**3.3 NavFn / 势场式栅格规划**
- **是什么**：在 costmap 上传播"导航函数"（势场/波前），从目标反向扩散，路径沿梯度下降；实现简单、稳定、参数少。
- **何时用**：平地、障碍不复杂、要"够用就行"；缺点是路径贴障碍、转弯生硬、狭窄处易贴边。
- **ROS 2 实现**：Nav2 `NavfnPlanner`（`nav2_navfn_planner`）；ROS 1 `navfn` / `global_planner`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner>

**3.4 采样式规划（RRT / RRT\* / PRM / BIT\* / Informed RRT\*）**
- **是什么**：随机采样 + 图/树连接，高维与复杂约束下最通用；RRT\* 系列保证渐近最优，BIT\* 用启发式批处理加速收敛。
- **何时用**：高维（机械臂、腿足）、复杂约束、非结构化空间；2D 平地导航里通常不如 A\*/Smac 划算。
- **ROS 2 实现**：OMPL（RRT/RRT\*/PRM/BIT\* 等的参考实现库）；Nav2 默认不含采样式全局规划器（需自写插件或用 MoveIt 那套）。
- **来源**：✅ <https://ompl.kavrakilab.org/> ；✅ arXiv 1105.1186（RRT\*，**ID 已核对**）；✅ arXiv 1404.2334（Informed RRT\*，**ID 已核对**）；✅ arXiv 1405.5848（BIT\*，**ID 已核对**）

**3.5 轨迹优化式规划（CHOMP / STOMP / TrajOpt / GPMP2 / TEB）**
- **是什么**：把路径当连续轨迹，最小化"平滑+避障+动力学"代价（梯度法/随机法/凸优化/高斯过程）。
- **何时用**：机械臂与需要高阶平滑的移动平台；**移动机器人上最常见的落地形态其实是局部 TEB/MPC（见 4.5/4.7）**，全局轨迹优化较少见。
- **ROS 2 实现**：MoveIt 2（CHOMP/STOMP 等规划器，主要面向机械臂）、TrajOpt（tesseract 生态）、GTSAM（GPMP2 所在库）、`teb_local_planner`（移动端）。
- **来源**：✅ <https://moveit.picknik.ai/main/index.html> ；✅ <https://github.com/moveit/moveit2/tree/main/moveit_planners/stomp> ；✅ <https://github.com/tesseract-robotics/trajopt> ；✅ <https://github.com/borglab/gtsam> ；✅ <https://github.com/rst-tu-dortmund/teb_local_planner>

**3.6 覆盖规划（boustrophedon / 细胞分解 / 完整覆盖）**
- **是什么**：保证把可通行区域"扫一遍"（弓字形/牛耕式、细胞分解、生成树），目标不是最短而是**覆盖率+重叠率**。
- **何时用**：清扫、消杀、巡检、农业；RM 里对应"巡逻/清点区域"这类任务（若真要做）。
- **ROS 2 实现**：Nav2 **Coverage Server**（官方覆盖服务器）；`opennav_coverage`（Nav2 兼容的完整覆盖任务服务器/导航器/BT 工具，Open Navigation 项目）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/> ；✅ <https://github.com/open-navigation/opennav_coverage>

**3.7 前沿 / 探索规划（frontier exploration）**
- **是什么**：把"已知与未知的边界"（frontier）当目标，边建图边导航，直到没有前沿；常配信息增益/距离的效用函数。
- **何时用**：无先验图、场地未知（第一次进场）；也是"无图模式"的标准答案。
- **ROS 2 实现**：`m-explore-ros2`（`explore_lite` 的 ROS 2 版）、`m-explore`（ROS 1 原版）。
- **来源**：✅ <https://github.com/robo-friends/m-explore-ros2> ；✅ <https://github.com/hrnr/m-explore>

**3.8 拓扑路由（route graph / 路线跟随）**
- **是什么**：预先/在线构建"路线图"（节点=关键位姿，边=可通行路段）并在图上做路由，落到具体几何路径再交给局部控制器。
- **何时用**：路线相对固定、需要"任务级"语义（巡逻、去增益点、回补给）；与 1.7 混合地图、5.4 巡逻环线天然配套。
- **ROS 2 实现**：Nav2 **Route Server**（`nav2_route`，支持 route graph 文件、图搜索、route 上的 BT 节点）；STRANDS `topological_navigation`（ROS 1 经典）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_route> ；✅ <https://github.com/strands-project/strands_navigation>

**3.9 多目标 / TSP 排序（waypoint sequencing）**
- **是什么**：给定一组目标点，决定**访问顺序**（TSP/VRP），再逐个/连续下发；也可"多点一次规划"（through poses）。
- **何时用**：巡逻、清点、多点补给；COD 的"手工示教 CSV 航点序列"就是**人肉 TSP**（顺序由人定，不求解）。
- **ROS 2 实现**：Nav2 `waypoint_follower`（`FollowWaypoints` + `NavigateThroughPoses` + 航点任务执行器插件）；顺序求解可用 Google OR-Tools。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower> ；✅ <https://developers.google.com/optimization/routing/tsp>

**3.10 MPC 式全局规划**
- **是什么**：把"整段路径"当最优控制问题滚动求解（含动力学/障碍约束），介于全局规划与局部控制之间。
- **何时用**：高速、动力学强耦合（赛车、无人机）；移动机器人上通常只在**局部**层用 MPC（4.7），全局用图搜索。
- **ROS 2 实现**：acados（实时最优控制求解器，ROS 2 有社区封装）、`mpc_local_planner`（移动底盘 MPC 局部规划器）；Nav2 的 MPPI 属于"采样式 MPC"（4.6）。
- **来源**：✅ <https://docs.acados.org/> ；✅ <https://github.com/rst-tu-dortmund/mpc_local_planner> ；✅ arXiv 1509.01149（MPPI，**ID 已核对**）

### B.4 轴 4：局部规划与控制（11 族）

**4.1 Pure Pursuit（纯跟踪）**
- **是什么**：在前方固定/自适应前视距离取一个点，算圆弧曲率追过去；经典、极简、只控转向/角速度。
- **何时用**：差速/Ackermann 低速跟踪、路径平滑；急弯与近目标处会切角/震荡。
- **ROS 2 实现**：无独立 Nav2 插件（被 RPP 取代）；ROS 1 `base_local_planner`/自写节点常见；学术与工业代码里到处都是。
- **来源**：✅ <https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf>（Coulter, CMU TR 1992, "Implementation of the Pure Pursuit Path Tracking Algorithm"）

**4.2 Stanley 控制器**
- **是什么**：前轮转角 = 航向误差 + 前轴横向误差的反正切项，DARPA Grand Challenge 冠军方案的核心。
- **何时用**：Ackermann 高速路径跟踪、道路场景；对横向误差敏感，需与前视/滤波配合。
- **ROS 2 实现**：无 Nav2 官方插件；ROS 2 社区/自研节点常见（`stanley_controller` 类包多为民码）。
- **来源**：⚠️ <https://onlinelibrary.wiley.com/doi/10.1002/rob.20147>（Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge", JFR 2006）

**4.3 Regulated Pure Pursuit（RPP）**
- **是什么**：纯跟踪 + 曲率/障碍/接近目标时的**速度调节**，专为服务机器人设计，参数少、行为可预测。
- **何时用**：室内服务机器人、窄通道、需要"平滑但保守"；我们 bench 的默认控制器。
- **ROS 2 实现**：Nav2 `nav2_regulated_pure_pursuit_controller`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/> ；✅ arXiv 2305.20026（RPP 论文，**ID 已核对**）；✅ <https://link.springer.com/article/10.1007/s10514-023-10097-6>（Autonomous Robots 正式版）

**4.4 DWA / DWB**
- **是什么**：在速度空间采样可行轨迹（考虑加速度极限），用多目标评价函数（避障、贴路径、朝向、速度）打分选优；DWB 是 DWA 的 Nav2 重写与插件化版本。
- **何时用**：动态障碍、需要局部绕行；参数多、调参成本高，低速下容易抖动。
- **ROS 2 实现**：Nav2 `nav2_dwb_controller`（DWB）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/> ；⚠️ <https://ieeexplore.ieee.org/document/580977>（Fox, Burgard, Thrun, DWA, IEEE RAM 1997）

**4.5 TEB（Timed Elastic Band）**
- **是什么**：把轨迹当成"橡皮筋"做多目标优化（时间最优 + 避障 + 运动学），支持全向、倒车、狭窄空间。
- **何时用**：需要"贴着障碍快速通过"、需要倒车/全向机动；调参最贵、CPU 最高、易发散。
- **ROS 2 实现**：`teb_local_planner`（ROS 1 原版，ROS 2 有分支/移植，我们 bench 已集成 + `costmap_converter`）。
- **来源**：✅ <https://github.com/rst-tu-dortmund/teb_local_planner>

**4.6 MPPI（Model Predictive Path Integral，采样式 MPC）**
- **是什么**：对大量控制序列采样、按轨迹代价指数加权，无需梯度即可处理非凸/非可微代价；Nav2 里支持差速与全向（Omni）运动学。
- **何时用**：高速、动态障碍、需要激进机动与平滑控制；CPU 开销大（`batch_size × time_steps`），需要速度包络与 critic 调参。
- **ROS 2 实现**：Nav2 `nav2_mppi_controller`（Humble 起可用；**我们已安装但尚未接入 `nav` 槽**）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller> ；✅ arXiv 1509.01149（MPPI 论文，**ID 已核对**）

**4.7 MPC（acados / casadi / mpc_local_planner）**
- **是什么**：显式求解带约束的最优控制问题（每周期解一次 QP/NLP），可显式编码动力学、速度、障碍约束。
- **何时用**：需要"可证明满足约束"的场合（安全、高速、Ackermann）；实现与实时性门槛高（求解器、模型、Warm start）。
- **ROS 2 实现**：`mpc_local_planner`（rst-tu-dortmund，ROS 1/ROS 2 代码基础）、acados（求解器）、CasADi（建模）；Nav2 官方无 MPC 插件。
- **来源**：✅ <https://github.com/rst-tu-dortmund/mpc_local_planner> ；✅ <https://docs.acados.org/>

**4.8 RL 局部策略（learned local policy）**
- **是什么**：用神经网络直接输出速度指令（端到端），或输出局部代价/评分供采样器使用；训练在仿真里完成。
- **何时用**：规则难写（人群、越野）、或作为高层决策层；**安全性与可解释性是硬伤**，比赛里几乎没人把底层交给 RL。
- **ROS 2 实现**：无官方插件；通常自写 `nav2_core::Controller` 插件或在 `cmd_vel` 前挂一个推理节点。
- **来源**：见 §B.7（7.1/7.5）；✅ <https://github.com/ros-navigation/navigation2>（Nav2 插件接口）

**4.9 速度平滑 / 指令整形**
- **是什么**：对 `cmd_vel` 做加速度/加加速度限幅、平滑、超时清零，避免"下位机收到不可能的速度"或原地抖动。
- **何时用**：**几乎总是应该开**（底盘保护、仿真↔实车一致）；我们已开 `velocity_smoother`，COD 也做了 Savitzky-Golay 平滑。
- **ROS 2 实现**：Nav2 `nav2_velocity_smoother`、`nav2_smoother`（Savitzky-Golay / constrained smoother，作用于路径而非速度）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/smoother_plugins/savitzky_golay_smoother/configuring_savitzky_golay_smoother/>

**4.10 运动学模型（差速 / 全向 / Ackermann / 腿足）**
- **是什么**：控制器与规划器必须匹配底盘可行运动集：差速（不可横移）、全向/麦轮（可横移+自旋）、Ackermann（最小转弯半径）、腿足（步态+地形）。
- **何时用**：总是——**运动学不匹配是"控制器输出对但车不动/画龙"的第一嫌疑**（我们踩过 `fake_vel_transform` 把角速度清零的坑）。
- **ROS 2 实现**：`ros2_control` 的 `diff_drive_controller` / `mecanum_drive_controller` / `ackermann_steering_controller`；Nav2 的 MPPI 有 Omni 模式、Smac Hybrid 用 Reeds-Shepp/Dubins；腿足用 elevation map + 专用步态控制器。
- **来源**：✅ <https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html> ；✅ <https://github.com/ros-controls/ros2_controllers/tree/master/mecanum_drive_controller> ；✅ <https://github.com/ros-controls/ros2_controllers/tree/master/ackermann_steering_controller> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/> ；⚠️ <https://www.science.org/doi/10.1126/scirobotics.abk2822>（Miki et al., 腿足感知式运动, Science Robotics 2022）

**4.11 反应式避障（势场 / VFH）**
- **是什么**：不规划完整路径，只根据当前障碍分布产生排斥速度/方向（势场法）或选择"障碍最少的方向扇区"（VFH）。
- **何时用**：极简平台、兜底安全层；局部极小值与震荡是固有缺陷。
- **ROS 2 实现**：无 Nav2 官方插件；ROS 1 生态有 `voxel_grid`/VFH 类包，多数团队自写。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/027836498600500106>（Khatib 1986）；⚠️ <https://ieeexplore.ieee.org/document/88148>（VFH 1991）

### B.5 轴 5：任务指定方式（10 族）

**5.1 无目标（探索驱动）**
- **是什么**：不给目标，由探索策略自己生成（前沿、信息增益）；结束条件是"没有未知区域/时间到"。
- **何时用**：第一次进场、场地未知；比赛里风险高（耗时不可控）。
- **ROS 2 实现**：`m-explore-ros2`（`explore_lite`）+ 在线 SLAM（slam_toolbox / cartographer）。
- **来源**：✅ <https://github.com/robo-friends/m-explore-ros2> ；✅ <https://github.com/SteveMacenski/slam_toolbox>

**5.2 单目标（single goal pose）**
- **是什么**：给一个 `PoseStamped`，走 BT `NavigateToPose` 全流程（规划→控制→恢复）。
- **何时用**：调试、单点验证、比赛里的"去某个点"。
- **ROS 2 实现**：Nav2 `nav2_bt_navigator` + `NavigateToPose` action；`nav2_simple_commander` 的 `BasicNavigator.goToPose()`。
- **来源**：✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://docs.nav2.org/rolling/getting_started/navigation_concepts/behavior_trees/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_bt_navigator>

**5.3 航点序列 / through-poses** ⭐（COD 的"多点导航"在这一格）
- **是什么**：一次给一串航点：`NavigateThroughPoses`（一次规划穿越多点）或 `FollowWaypoints`（逐个下发、每个航点可带任务插件）。
- **何时用**：巡逻、按顺序打卡；COD 用手工示教的 CSV 航点表 + 多点导航实现。
- **ROS 2 实现**：Nav2 `nav2_waypoint_follower`（`FollowWaypoints` + `WaitAtWaypoint` 等插件）；`NavigateThroughPoses` 由 BT navigator 提供。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/waypoint_follower/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_waypoint_follower> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation>（csv 航点 + 单点/多点导航）

**5.4 巡逻环线（patrol loop）**
- **是什么**：把路线做成闭环，反复执行；可与"事件触发换路线"（比分、血量、裁判系统信号）结合。
- **何时用**：哨兵/安防的常态任务；RM 哨兵的"巡逻+增益点+回防"本质上是**带条件的巡逻环线**。
- **ROS 2 实现**：Nav2 Route Server（route graph 支持环路）+ waypoint follower 循环调用；上层由 BT/FSM 决定何时换路线。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_route>

**5.5 覆盖任务（coverage mission）**
- **是什么**：任务是"覆盖这块区域"，由覆盖服务器生成完整覆盖路径并执行。
- **何时用**：清扫/巡检/搜索；RM 里对应"扫点/清点"类需求。
- **ROS 2 实现**：Nav2 Coverage Server；`opennav_coverage`（含 BT 节点与导航器）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_coverage_server/> ；✅ <https://github.com/open-navigation/opennav_coverage>

**5.6 示教-重放（teach & repeat playback）**
- **是什么**：人工示教一遍（推着走/遥控），系统记录路线，之后自动重放；局部避障可叠加。
- **何时用**：路线固定、场地外观变化大、不想做全局定位；工业 AGV 与巡检机器人常见。
- **ROS 2 实现**：`wiln`（激光 teach-and-repeat）；Nav2 里可用 route graph + 录制路径近似。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；⚠️ Annual Reviews topometric teach-and-repeat 综述（<https://www.annualreviews.org/content/journals/10.1146/annurev-control-032724-020548>）

**5.7 语义 / 自然语言指令（VLN 家族）**
- **是什么**：用自然语言（"去红方补给区"）或视觉语言目标（"找到沙发"）指定任务，由 VLM/LLM 规划子目标。
- **何时用**：研究与人机交互场景；比赛里可靠性/算力/延迟都不达标（**本报告综合**）。
- **ROS 2 实现**：无官方栈；研究项目（VLFM、NaVid、LM-Nav、NavGPT、SayCan）多为仿真或特定硬件 Demo。
- **来源**：✅ arXiv 1711.07280（R2R VLN 基准，**ID 已核对**）；✅ arXiv 2312.03275（VLFM）；✅ arXiv 2402.15852（NaVid）；✅ arXiv 2207.04429（LM-Nav）；✅ arXiv 2305.16986（NavGPT）；✅ arXiv 2204.01691（SayCan）——以上 ID 均已核对

**5.8 示教学习（LfD / 模仿学习）**
- **是什么**：从人类示范（遥操作/轨迹）学策略，而不是回放轨迹本身；可分行为克隆、逆强化学习、DAgger 交互式修正。
- **何时用**：规则难写、示范易得；分布偏移（covariate shift）是主要失败模式。
- **ROS 2 实现**：无官方栈；常见做法是录 bag → 离线训练 → 以插件/节点形式接回 `cmd_vel`。
- **来源**：✅ arXiv 1011.0686（DAgger，**ID 已核对**）；⚠️ <https://www.sciencedirect.com/science/article/pii/S0921889009000974>（Argall et al., "A Survey of Robot Learning from Demonstration", RAS 2009）

**5.9 行为树 / FSM 任务层**
- **是什么**：把"任务"写成可组合的行为树或状态机（规划→控制→恢复→重试→换路线），Nav2 自带 BT 导航器。
- **何时用**：**只要任务不止"到一个点"，就该有这一层**；COD 用 BT + RateController + IsStuck→BackUp，我们目前是 stock BT + 自研分段工具。
- **ROS 2 实现**：`nav2_bt_navigator` + BehaviorTree.CPP（可自定义 XML 与节点）；SMACH/PlanSys2 用于更经典的任务规划。
- **来源**：✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://www.behaviortree.dev/> ；✅ <https://github.com/PlanSys2/ros2_planning_system> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree>

**5.10 任务 / 车队协议（VDA5050 / Open-RMF / MassRobotics）**
- **是什么**：机器人与上位调度系统之间的**标准消息协议**（订单、路径、状态、错误码），让多厂商车队混跑。
- **何时用**：多机园区、仓储；单车比赛用不上，但**它定义了"工业界怎么描述任务"**，值得抄它的抽象。
- **ROS 2 实现**：Open-RMF（完整车队管理框架）、VDA5050 参考仓库、MassRobotics AMR 互操作标准。
- **来源**：✅ <https://github.com/open-rmf/rmf> ＋ ✅ <https://www.open-rmf.org/> ；✅ <https://github.com/VDA5050/VDA5050> ＋ ✅ <https://www.vda.de/dam/jcr:f0c9c019-1506-4dee-998a-e92723fbf025/EN-VDA5050-V200.pdf>（VDA5050 v2.0.0 官方规范 PDF）；✅ <https://www.massrobotics.org/what-is-the-massrobotics-amr-interoperability-standard/>

### B.6 轴 6：不确定性与安全（7 族）

**6.1 未知区策略（unknown-space policy）**
- **是什么**：三件套参数：`track_unknown_space`（是否区分未知与空闲）、规划器 `allow_unknown`（能否穿未知区）、膨胀层 `inflate_unknown`（未知区是否当障碍膨胀）。
- **何时用**：**永远是必答题**。在线建图时大量 unknown，若 `allow_unknown=true` 就会出现"穿空白/把窄缝当走廊"（我们与 COD 都踩过，见 `docs/cod_nav_comparison.md §3`）。
- **ROS 2 实现**：`nav2_costmap_2d` 参数 + 各规划器参数（NavFn/Smac/Theta\*）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/>

**6.2 重规划与恢复行为（recovery）**
- **是什么**：卡住/路径失效时的兜底动作：清代价地图、自旋、后退、等待、按航向直行；BT 决定重试次数与顺序。
- **何时用**：所有真实部署；没有恢复行为的栈在动态场景里"一次失败就永久失败"。
- **ROS 2 实现**：Nav2 Behavior Server（`spin`/`backup`/`drive_on_heading`/`wait`/`assisted_teleop`）+ BT 恢复分支（`ClearEntireCostmap` 等）。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/> ；✅ <https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_behaviors>

**6.3 碰撞监控 / 安全监控 / 限速**
- **是什么**：独立于规划器的"最后一道软件闸门"：按多边形/圆形监测区判断即将碰撞则减速/停车；以及速度限幅与超时清零。
- **何时用**：有人/有对抗的动态环境；**工业上这一层通常还要被安全 PLC/安全激光雷达取代或冗余（见 6.6/6.7）**。
- **ROS 2 实现**：Nav2 `nav2_collision_monitor`（collision detector + monitor 节点，可作为 `cmd_vel` 中继）；`nav2_velocity_smoother`。
- **来源**：✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/> ；✅ <https://github.com/ros-navigation/navigation2/tree/main/nav2_collision_monitor> ；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/>

**6.4 风险感知与机会约束规划（risk-aware / chance-constrained）**
- **是什么**：不追求"零碰撞"，而是约束**碰撞概率**（机会约束）或优化风险泛函（CVaR、风险敏感 MPC）。
- **何时用**：不确定性大（定位漂移、动态障碍预测）、需要量化安全边界；计算复杂，工程落地少。
- **ROS 2 实现**：无官方插件；通常在自研 MPC/采样器里加风险项（学术代码为主）。
- **来源**：✅ arXiv 2307.08024（"Bayesian inference for data-efficient, explainable, and safe robotic motion planning: A review"，**ID 已核对**；含不确定性/风险规划章节）

**6.5 主动 SLAM / 不确定性驱动探索**
- **是什么**：把"定位/地图的不确定性"作为探索目标（最大化信息增益、最小化位姿协方差），而不仅是"探索未知区域"。
- **何时用**：地图质量决定后续任务成败、且场地允许自由探索；比赛场景通常时间不够（**本报告综合**）。
- **ROS 2 实现**：无官方栈；`m-explore` 类包只用几何前沿，不含不确定性项；主动 SLAM 以研究代码为主。
- **来源**：✅ arXiv 2207.00254（"A Survey on Active Simultaneous Localization and Mapping"，**ID 已核对**）

**6.6 安全标准与"安全链独立"（ISO 3691-4 / ANSI-A3 R15.08 / ISO 13849 / IEC 61508）**
- **是什么**：无人工业车辆（ISO 3691-4）、工业移动机器人（ANSI/A3 R15.08 系列）、机械安全控制系统（ISO 13849-1）、功能安全通用（IEC 61508）等标准，规定安全功能、性能等级（PL/SIL）与验证方法。
- **何时用**：**产品要出货、要过认证、要进工厂就必须**；比赛不用，但它解释了"为什么工业产品不敢让导航计算机决定停车"。
- **ROS 2 实现**：标准不提供实现；工程上的结论是**安全链（安全激光雷达/安全 PLC/急停/安全限速）与导航计算机物理/逻辑分离**——导航只出"建议速度"，安全链有权否决（**本报告综合 + 厂商文档佐证**，见 6.7 与 8.8）。
- **来源**：⚠️ <https://www.iso.org/standard/70660.html>（ISO 3691-4:2020 官方目录页）；⚠️ <https://www.iso.org/standard/69883.html>（ISO 13849-1）；⚠️ <https://webstore.ansi.org/standards/ria/ansia3r15082026>（ANSI/A3 R15.08-3-2026）；⚠️ <https://www.automate.org/standards/industrial-mobile-robots>（A3 工业移动机器人标准页）；✅ <https://www.iec.ch/functionalsafety>（IEC 功能安全总览）

**6.7 安全型传感器与安全 PLC（safety-rated sensing）**
- **是什么**：通过认证的安全激光扫描仪、安全编码器、安全控制器（可做安全限速、安全区域、安全停机）。
- **何时用**：与 6.6 绑定；典型形态是"安全扫描仪覆盖车体前后 + 安全 PLC 直接切动力"。
- **ROS 2 实现**：安全设备一般**不接 ROS 图**（接安全 PLC）；ROS 侧只接收状态位（如"安全停"）用于状态机。
- **来源**：✅ <https://www.sick.com/us/en/catalog/products/safety/safety-laser-scanners/microscan3/c/g187235>（SICK microScan3 安全激光扫描仪产品页）；⚠️ <https://www.pilz.com/en-JP/company/press/messages/articles/239220>（Pilz：AMR 安全导航方案新闻稿，厂商资料）；✅ <https://www.sick.com/media/docs/3/43/143/operating_instructions_nav350_laser_positioning_sensor_en_im0040143.pdf>（SICK NAV350 操作手册，含反射板定位与安全相关说明）

### B.7 轴 7：学习 / 端到端 / 基础模型（8 族）

**7.1 RL 导航（DQN / DDPG / SAC / PPO 系）**
- **是什么**：把导航建成 MDP，用深度 RL 学策略（目标驱动视觉导航、连续控制避障）。
- **何时用**：仿真里可大规模采样、奖励可设计（人群避让、越野）；真机迁移与安全性是门槛。
- **ROS 2 实现**：无官方栈；典型是"仿真训练 → 导出 ONNX → ROS 2 推理节点"。
- **来源**：✅ arXiv 1312.5602（DQN）；✅ arXiv 1509.02971（DDPG）；✅ arXiv 1801.01290（SAC）；✅ arXiv 1707.06347（PPO）；✅ arXiv 1609.05143（Target-driven 视觉导航）——以上 ID 均已核对

**7.2 基准与仿真器（Habitat / Isaac Lab / Arena-Rosnav / iGibson）**
- **是什么**：为导航/具身智能提供场景、传感器仿真、任务与评测协议（成功率、SPL 等）。
- **何时用**：做学习型导航的对照实验；**我们的 Gazebo Classic bench 属"工程验证"而非"学习基准"**，两者互补。
- **ROS 2 实现**：Habitat（非 ROS，Python 生态）、Isaac Lab（GPU 并行仿真，有 ROS 2 桥）、Arena-Rosnav（ROS 2 社交导航基准）、iGibson。
- **来源**：✅ <https://aihabitat.org/> ＋ ✅ <https://github.com/facebookresearch/habitat-lab> ；✅ <https://isaac-sim.github.io/IsaacLab/> ＋ ✅ <https://github.com/isaac-sim/IsaacLab> ；✅ <https://github.com/Arena-Rosnav/arena-rosnav> ；✅ <https://github.com/StanfordVL/iGibson>

**7.3 模仿学习（IL / 行为克隆 / DAgger）**
- **是什么**：从专家轨迹学策略；DAgger 通过"边跑边问专家"缓解分布偏移。
- **何时用**：示范数据易得、奖励难设计；长程导航里常与拓扑图/子目标结合。
- **ROS 2 实现**：无官方栈；录 bag → 训练 → 推理节点。
- **来源**：✅ arXiv 1011.0686（DAgger）；⚠️ ScienceDirect Argall LfD 综述（同 5.8）

**7.4 Sim-to-real（域随机化 / 迁移）**
- **是什么**：让仿真里学到的策略在真机可用：域随机化、系统辨识、噪声建模、teacher-student 蒸馏。
- **何时用**：任何"仿真训练 + 真机部署"的路线；**对我们的 bench 尤其相关：仿真↔实车契约本身就是 sim-to-real 的工程化版本**（见 `docs/sim_real_contract.md`）。
- **ROS 2 实现**：Isaac Lab 内置域随机化；Gazebo 侧靠插件与参数扰动自己搭。
- **来源**：✅ arXiv 2009.13303（Sim-to-Real 综述）；✅ arXiv 1703.06907（域随机化）——ID 均已核对

**7.5 基础模型导航（VLFM / NaVid / NoMaD / ViNT-GNM / π0 系）**
- **是什么**：在大规模数据上预训练的导航模型：视觉导航基础模型（ViNT/GNM/NoMaD）、零样本语义前沿（VLFM）、视频 VLM 导航（NaVid）、VLA 动作模型（π0/OpenVLA）。
- **何时用**：跨场景零样本、语言/图像目标；算力、实时性、可解释性都远离比赛要求（**本报告综合**）。
- **ROS 2 实现**：研究代码为主（多数是 Python + 仿真/自有硬件），需要自己包成 ROS 2 节点。
- **来源**：✅ arXiv 2312.03275（VLFM）；✅ arXiv 2402.15852（NaVid）；✅ arXiv 2310.07896（NoMaD）；✅ arXiv 2306.14846（ViNT，含 GNM）；✅ arXiv 2410.24164（π0）；✅ arXiv 2406.09246（OpenVLA）；✅ <https://visualnav-transformer.github.io/> ；✅ <https://pku-epic.github.io/NaVid/>；✅ <https://github.com/Physical-Intelligence/openpi>——arXiv ID 均已核对

**7.6 世界模型（world models）**
- **是什么**：学环境的隐动力学模型，在"想象"里做规划/训练（Ha & Schmidhuber 的 VAE+RNN；DreamerV3 的隐空间 actor-critic）。
- **何时用**：样本效率要求高、奖励稀疏；对导航的工程价值仍在研究中。
- **ROS 2 实现**：无官方栈。
- **来源**：✅ arXiv 1803.10122（World Models，**ID 已核对**）；✅ arXiv 2301.04104（DreamerV3，**ID 已核对**）

**7.7 扩散策略（diffusion policy / planning）**
- **是什么**：用扩散模型建模多模态动作分布（Diffusion Policy）或直接生成轨迹（Diffuser）；天然表达"多峰"行为。
- **何时用**：多模态示范数据、需要避障与示教风格兼容；推理频率与实时性是限制。
- **ROS 2 实现**：研究代码；通常作为高层策略，输出给底层控制器。
- **来源**：✅ arXiv 2303.04137（Diffusion Policy）；✅ arXiv 2205.09991（Diffuser）；✅ arXiv 2403.03954（3D Diffusion Policy）；✅ <https://diffusion-policy.cs.columbia.edu/> ——ID 均已核对

**7.8 LLM/VLM 任务规划（SayCan / Code as Policies / VoxPoser / LLM-Planner）**
- **是什么**：LLM/VLM 把语言指令分解成可执行子目标或代码，再由经典栈执行。
- **何时用**：任务层需要灵活性、可解释的中间表示；对"是否能落地到比赛"目前答案是"不能直接落地，但可借它的任务分解思路"（**本报告综合**）。
- **ROS 2 实现**：无官方栈；常见做法是 LLM 输出 JSON/航点序列交给 Nav2 action。
- **来源**：✅ arXiv 2204.01691（SayCan）；✅ arXiv 2209.07753（Code as Policies）；✅ arXiv 2307.05973（VoxPoser）；✅ arXiv 2212.04088（LLM-Planner）；✅ arXiv 2305.16986（NavGPT）——ID 均已核对

### B.8 轴 8：工业 AGV/AMR 范式（8 族）

**8.1 磁条 / 磁带导引**
- **是什么**：地面贴磁带/磁条，车载磁传感器循迹；位置精度靠标记点（RFID/磁钉）修正。
- **何时用**：路线极固定、成本敏感、环境要求"绝对可预测"；改路线要重新贴磁条。
- **ROS 2 实现**：通常不由 ROS 主导（PLC/专用控制器），ROS 只做上层调度与 HMI。
- **来源**：✅ <https://www.mhi.org/blog/126126/agv-amr-navigation-systems-floor-based-vs-lidar>（MHI 行业博客：floor-based vs LiDAR 导引对比，**行业资料**）；✅ <https://iopscience.iop.org/article/10.1088/1755-1315/1285/1/012023>（AGV 导航技术综述类会议论文）

**8.2 二维码 / 标记导引**
- **是什么**：地面二维码阵列，车载下视相机读码得到绝对位姿，码间靠里程计；仓储机器人经典方案。
- **何时用**：高密度货架、需要厘米级停靠；地面必须保持洁净可贴码。
- **ROS 2 实现**：`apriltag_ros`、OpenCV ArUco；工业上多为厂商私有实现。
- **来源**：✅ <https://github.com/AprilRobotics/apriltag> ；⚠️ <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html> ；⚠️ <https://ieeexplore.ieee.org/document/11324329>（磁条-二维码混合导航论文，IEEE 目录页）

**8.3 反射板 / 激光三角定位**
- **是什么**：墙上/柱上装反光板，激光扫描仪测角（三角/三边）解算绝对位姿；工业 AGV 的"可认证"定位方案。
- **何时用**：需要长期稳定、可做安全型定位、场地可改造；需要布置反射板并做标定。
- **ROS 2 实现**：厂商方案为主（SICK NAV350、Kollmorgen/NDC8 等）；ROS 侧通常只做上层接口。
- **来源**：✅ <https://www.sick.com/media/docs/3/43/143/operating_instructions_nav350_laser_positioning_sensor_en_im0040143.pdf>（SICK NAV350 官方操作手册）；⚠️ <https://ndcsolutions.com/zh-hans/ndc8-40-including-our-new-generation-of-laser-navigation/>（NDC8 激光导航系统，厂商页）；⚠️ <https://www.kollmorgen.com/en-us/products/autonomous-mobile-solutions>（Kollmorgen AGV/AMR 方案）

**8.4 示教-重放 AGV**
- **是什么**：人工带车走一遍，系统记录路径（磁条/激光/视觉均可），之后自动重放。
- **何时用**：路线稳定、部署时间紧、现场工程师不想碰 SLAM 参数；与 1.8/5.6 是同一族在工业里的名字。
- **ROS 2 实现**：`wiln`；工业上多为示教盒 + 专有控制器。
- **来源**：✅ <https://github.com/norlab-ulaval/wiln> ；✅ MHI 行业博客（同 8.1）

**8.5 SLAM AMR（自然导航）**
- **是什么**：不靠地面基础设施，靠激光 SLAM + 先验图定位"自然导航"；部署灵活，但需要环境特征与调参。
- **何时用**：场地多变、改造受限、要与人类混行；代表产品为仓储/服务 AMR。
- **ROS 2 实现**：Nav2 全栈（AMCL + NavFn/Smac + DWB/MPPI）、Cartographer、slam_toolbox。
- **来源**：✅ MHI 行业博客（floor-based vs LiDAR，含自然导航对比）；✅ <https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/> ；✅ <https://github.com/SteveMacenski/slam_toolbox>

**8.6 车队管理（fleet manager）**
- **是什么**：多台 AGV/AMR 的交通管制、任务派发、充电调度、电梯/门禁联动。
- **何时用**：≥2 台车、共享通道时**必须**有（否则死锁）；调度层与导航层解耦。
- **ROS 2 实现**：Open-RMF（开源车队管理框架，含交通调度与任务编排）；商用如 MiR Fleet / OTTO Fleet Manager（私有）。
- **来源**：✅ <https://www.open-rmf.org/> ＋ ✅ <https://github.com/open-rmf/rmf> ；✅ <https://www.massrobotics.org/what-is-the-massrobotics-amr-interoperability-standard/>

**8.7 VDA5050 协议**
- **是什么**：AGV/AMR 与上位系统之间的标准 JSON/MQTT 接口（订单、即时动作、状态、可视化、错误），支持多厂商混跑。
- **何时用**：产品要接入第三方 WMS/MES/车队系统；**它是"任务层"的工业标准答案**（对应 5.10）。
- **ROS 2 实现**：VDA5050 官方参考仓库；社区有 `vda5050_connector` 类 ROS 2 桥（部分仓库已迁移/私有化，需自行确认）。
- **来源**：✅ <https://github.com/VDA5050/VDA5050> ；✅ <https://www.vda.de/dam/jcr:f0c9c019-1506-4dee-998a-e92723fbf025/EN-VDA5050-V200.pdf>

**8.8 为什么工业偏好固定路径 + 拓扑图**
- **是什么/为什么（本报告综合，基于 8.1–8.7 与 6.6 的来源）**：① 安全认证要求可预测、可验证的行为，自由规划难以论证"不会撞人"；② 安全型定位（磁条/二维码/反射板）比 SLAM 更容易过认证；③ 客户要的是**可重复、可审计**的节拍，而不是最优路径；④ 拓扑图/固定路线让调度、交通管制、验收测试都变简单；⑤ SLAM 的失效模式（动态环境、长走廊）在工厂里代价极高。
- **何时用**：工业落地/需要认证的场合；反过来说，**比赛/研究场景的"自由规划"优势在工厂里往往不是优势**。
- **ROS 2 实现**：这一族本身是"设计取向"而非算法，落地组合见 §D.1。
- **来源**：✅ MHI 行业博客（floor-based vs LiDAR）；⚠️ <https://www.pilz.com/en-JP/company/press/messages/articles/239220>（Pilz AMR 安全导航，厂商资料）；⚠️ ISO 3691-4 官方目录页（同 6.6）

### B.9 轴 9：多机 / 协同（6 族，简述）

**9.1 编队控制（formation control）**
- **是什么**：多机保持几何构型（队形）运动：领航-跟随、虚拟结构、基于位移/距离的编队。
- **何时用**：表演、协同搬运、护航；RM 里几乎用不到（规则上通常单兵）。
- **ROS 2 实现**：无官方栈；研究代码 + 自研。
- **来源**：⚠️ <https://www.sciencedirect.com/science/article/pii/S0005109815001038>（Oh, Park, Ahn, "A survey of formation control of multi-agent systems", Automatica 2015）

**9.2 任务分配（MRTA）**
- **是什么**：把任务集合分配给多台机器人（拍卖、市场、匈牙利算法、CBBA），目标是最小化完成时间/代价。
- **何时用**：多机巡检、搜索；与 3.9 的单机 TSP 是同一问题的多机版。
- **ROS 2 实现**：研究代码为主；Open-RMF 的任务编排可视为工程化实现。
- **来源**：⚠️ <https://journals.sagepub.com/doi/10.1177/0278364904045564>（Gerkey & Matarić, IJRR 2004, MRTA 分类学）；✅ <https://github.com/open-rmf/rmf>

**9.3 多机 SLAM / 地图融合**
- **是什么**：多机各自建图 + 机间回环/地图合并（分布式 PGO），得到全局一致地图。
- **何时用**：大场地多机同时建图；通信带宽与时间同步是门槛。
- **ROS 2 实现**：`Kimera-Multi`（分布式度量-语义）、`Swarm-SLAM`（稀疏分布式协作 SLAM）。
- **来源**：✅ <https://github.com/MIT-SPARK/Kimera-Multi> ；✅ <https://github.com/MISTLab/Swarm-SLAM> ＋ ✅ arXiv 2301.06230（**ID 已核对**）

**9.4 多机前沿探索**
- **是什么**：把前沿分配给不同机器人，避免重复探索；经典工作是"协调式多机探索"（基于效用/代价分配）。
- **何时用**：灾难搜救、大范围未知环境。
- **ROS 2 实现**：`m-explore` 系列有多机分支/扩展；多在研究代码里。
- **来源**：✅ <https://github.com/hrnr/m-explore> ；✅ <https://github.com/robo-friends/m-explore-ros2> ；⚠️ <https://ieeexplore.ieee.org/document/1389414>（Burgard et al., "Coordinated Multi-Robot Exploration", IEEE T-RO 2005）

**9.5 通信约束下的协同**
- **是什么**：考虑带宽、丢包、断连（间歇通信）的协同策略：何时通信、共享什么、断连时如何自治。
- **何时用**：真实无线环境、地下/遮蔽场景；仿真里常被忽略，真机上是大坑。
- **ROS 2 实现**：ROS 2 多机通信（DDS 域、`ROS_DOMAIN_ID`、Discovery Server）；工程参考 OSRF《ROS 2 多机器人系统》书。
- **来源**：✅ <https://osrf.github.io/ros2multirobotbook/> ；✅ <https://docs.ros.org/en/jazzy/p/example_multi_robot/>

**9.6 ROS 2 多机部署**
- **是什么**：命名空间、TF 前缀、多 `map` 系、DDS 配置等工程手段，让 N 台车共用一套栈。
- **何时用**：多机开发/仿真；RM 有队伍用 namespace 为未来多机留位（如 SMBU PolarBear 的 `pb2025_sentry_nav`）。
- **ROS 2 实现**：`example_multi_robot`（ROS 2 官方示例）、Isaac Sim ROS 2 多机导航教程、Nav2 多机 bringup 实践。
- **来源**：✅ <https://docs.ros.org/en/jazzy/p/example_multi_robot/> ；✅ <https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_multi_navigation.html> ；✅ <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>

### B.10 轴 10：RM 哨兵生态落点（1 行/仓库）

> 说明：以下"用了哪些族"是根据各仓库 **README / 文件清单 / 我们自己的只读深读文档**归纳的（**本报告综合**），
> 未 clone、未运行；仓库可能在赛季间继续变动。「ID 已核对」类论文引用同前。

| 项目 | 链接 | 它落在这张地图的哪些格子 |
|---|---|---|
| **SMBU-PolarBear `pb2025_sentry_nav`**（深圳北理莫斯科大学北极熊，2025 赛季） | ✅ <https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav> | 2.3 Point-LIO（自维护分支）+ 2.4 small_gicp 重定位 + 1.2/2.5D `terrain_analysis(_ext)` 地形分析（离地高度写进 intensity）+ 1.3 STVL/点云层 + 3.3 Nav2 默认全局规划器 + 4.x `pb_omni_pid_pursuit_controller` 全向 PID 跟踪 + 5.3 航点 + 9.6 namespace 多机预留；`pointcloud_to_laserscan` 仅在 SLAM 模式启用 |
| **PolarisXQ `SCURM_SentryNavigation`**（SCURM 火锅战队 24–25 赛季） | ✅ <https://github.com/PolarisXQ/SCURM_SentryNavigation> | 2.3 改造版 FAST-LIO（**自带重定位模式**，省一套重定位算力）+ 2.4 ICP 重定位 + 1.2 `terrain_analysis` 地形分析 + 自研 **costmap intensity 层**（1.10/1.2 的工程化）+ 6.2 增强版 back_up 恢复行为（朝无碰撞方向退）+ 5.9 BehaviorTree.CPP |
| **XDU-IRobot `SentryNav2026_XDU`**（西安电子科技大学 IRobot，2026 赛季） | ✅ <https://github.com/XDU-IRobot/SentryNav2026_XDU> | 2.3 Super-LIO 里程计 + 2.4 **Lightning-LM 重定位** + 1.12/2.4 点云裁剪与配准 + 1.3/1.2 深度相机（D435）点云 + 3.x Nav2 导航栈 + 5.9 自主行为树决策 + 裁判系统/CAN 通信（任务层）。注：仓库名不是 `sentry_navigation`（该名 404），`sentry_navigation` 是其**包目录**；RM2024 版本开源贴见 ✅ <https://bbs.robomaster.com/article/374512> |
| **`laohao78/ROS2_RM_Navigation`** | ✅ <https://github.com/laohao78/ROS2_RM_Navigation> | 2.3 FAST-LIO / Point-LIO + 2.4 **SLAM-Toolbox / AMCL / ICP 多选重定位** + 2.1 轮式里程计 + 3.3 NavFn 类全局规划 + 4.5 TEB 局部规划 + `mapping`/`nav` 双模式 + Gazebo Classic 仿真（与我们 bench 结构最接近的公开仓库） |
| **COD 战队：GitHub `qza36/COD_NAV` + Gitee `cod_-rm2026_-navigation`** | ✅ <https://github.com/qza36/COD_NAV> ；✅ <https://gitee.com/codnavgation/cod_-rm2026_-navigation> | 2.3 FAST-LIO fork / Small Point-LIO + **2.9 静态 `map→odom`（不重定位）** + 1.3/1.1 两张 costmap 都上 STVL（时间维）+ 1.12 车体点云裁剪盒 + 3.x Smac2D（2025）/MPPI 侧全局 + 4.6 **MPPI（全向，高速）** + 5.3 **CSV 航点序列（多点导航）** + 4.9 Savitzky-Golay 平滑；RM2026 开源贴 ✅ <https://bbs.robomaster.com/article/1882897> |
| **`Yancey2023/small_point_lio`**（东莞理工 ACE，RM26） | ✅ <https://github.com/Yancey2023/small_point_lio> | 2.3 LIO（Point-LIO 的 2–3× 加速变体，ESKF + iVox，**无回环无 PGO**，见我们 `docs/cod_nav_2026_deep_dive.md` 的只读核对）——它是"里程计插槽"里的一个实现，不是完整导航方案 |
| **`Xiancaijiang/rm2025_sentry_nav`**（PolarBear 2025 的另一公开副本/前身） | ✅ <https://github.com/Xiancaijiang/rm2025_sentry_nav> | 与 `pb2025_sentry_nav` 同源；可作为"同一套设计在不同赛季的演化"样本 |
| **`HongbiaoZ/autonomous_exploration_development_environment`** | ✅ <https://github.com/HongbiaoZ/autonomous_exploration_development_environment> | 1.2 **2.5D 地形分析**（`terrain_analysis`/`terrain_analysis_ext`）与 3.7 探索式导航的参考实现；被多个 RM 队直接借用（PolarBear、SCURM 均引用它） |
| **`SMBU-PolarBear-Robotics-Team/small_gicp_relocalization`** | ✅ <https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization> | 1.12 + 2.4：把先验 PCD 与实时点云做 GICP 配准求 `map→odom`（RM 生态里"PCD 先验重定位"的标准件） |
| **`icp_registration`**（我们 bench 内 `src/rm_localization/icp_registration`） | 本地包（无外部 URL） | 1.12 + 2.4：与上者同类，作为我们 `localization:=icp` 槽的实现 |

---

## C. 决策表：如果你要 … → 推荐族 + 代价

> 「族」编号对应 §B；「代价」一列是**必须一起接受的代价**，不是可选项。

| # | 如果你要 … | 推荐族（编号） | 代价 / 注意 |
|---|---|---|---|
| 1 | **无图 / 第一次进场（场地未知）** | 1.1 + 1.3（在线 SLAM 出图）+ 3.7 前沿探索 + 6.2 恢复行为；里程计用 2.3 | 无重复精度；地图质量随圈数变化；**未知区策略（6.1）必须先定**，否则"穿空白/窄缝当走廊"；探索耗时不可控 |
| 2 | **有图、要重复精度（同场地反复跑）** | 1.1 + 2.4 重定位（AMCL / GICP / Cartographer 纯定位 / slam_toolbox localization）+ 3.2 Smac + 4.3 RPP 或 4.6 MPPI | 要维护地图资产与坐标系一致性；场地一改图就作废；**重定位失效没有兜底就会"整体跑飞"**（要有 2.5 回环/多假设或 2.9 兜底策略） |
| 3 | **换场地免重标定** | 1.8 示教-重放 / 1.5+3.8 拓扑路由 + 2.3 LIO 里程计（不依赖先验图）；或每次重跑 1.1+3.7 | 路线要重教/重录，人工成本转移到赛前；没有全局最优路径；示教质量直接决定上限 |
| 4 | **要安全认证 / 要出货** | 6.6 + 6.7（安全链独立）+ 8.1–8.3 固定路径与安全型定位 + 6.3 碰撞监控与限速 | 硬件成本（安全扫描仪/PLC）；自由规划被大幅限制；ROS 侧只出"建议速度"；认证周期长 |
| 5 | **要高速动态避障** | 4.6 MPPI（全向）+ 4.10 全向运动学 + 1.3 STVL（时间衰减）+ 4.9 速度平滑 + 6.3 碰撞监控 | CPU（`batch_size×time_steps`）、critic 调参量大；**速度包络必须从低速标起**；`robot_radius/footprint` 不诚实会直接撞 |
| 6 | **要开源可改 / 生态最厚** | 3.x/4.x/5.x/6.x 的 **Nav2 实现**（插件化：planner/controller/costmap layer/BT 节点都可自写）+ 3.6 `opennav_coverage` + 3.8 `nav2_route` | Nav2 是 2D 接口为中心（3D 规划要自写插件）；Humble 与最新文档存在版本差异；插件多了以后"契约问题"比"算法问题"更常见（我们已踩过 6 个） |
| 7 | **窄缝 / 贴边通过（场地小、障碍密）** | 1.1 诚实 footprint + 3.2 Smac2D（`cost_travel_multiplier` 把路径推离膨胀层）+ 4.3 RPP（保守）或 4.6 MPPI + 6.1 `inflate_unknown` 收紧 | 与"高速"直接冲突；膨胀半径/代价乘子是"窄缝当走廊"的正面对策，但过紧会无解；要接受更慢的平均速度 |
| 8 | **长走廊 / 几何退化（雷达看不到特征）** | 2.3 多源 LIO（FAST-LIO + Point-LIO 可选）+ 2.2 轮+IMU EKF 兜底 + 2.4 先验图配准（GICP/反射板） | 复杂度上升；需要"谁在什么时候发 `map→odom`"的严格契约（我们已有 `tf_interface_contract.md`）；退化时任何单一来源都会漂 |
| 9 | **要 2.5D/3D 地形能力（斜坡、台阶、高地）** | 1.2 高程/可通行性（`terrain_analysis` 或 `elevation_mapping`）+ 1.4 ESDF（若要做平滑三维轨迹）+ 4.10 匹配的运动学 | 需要自写 costmap 层/规划器（Nav2 规划接口是 2D）；调参与标定量显著上升；仿真里"斜坡"很难做得像真的 |
| 10 | **语言/语义任务（"去增益点"式指令）** | 1.6 语义地图 + 7.5/7.8 基础模型与 LLM 任务层 + 3.8 拓扑路由（把语义落成节点） | 算力与延迟；可靠性不足以承担比赛；**现实做法是"人写拓扑节点 + 语义标签"，而不是端到端 VLM**（本报告综合） |
| 11 | **多机协同** | 9.2 任务分配 + 9.3 多机 SLAM 或各自独立建图 + 8.6 车队管理/8.7 VDA5050（工程化）+ 9.6 namespace 部署 | 通信是最大风险（9.5）；调试复杂度非线性上升；比赛规则通常不允许 |

---

## D. 标准产品化组合（5~8 种）与各自代表

> 这一节回答"到底有几种**真在用**的思路"。结论：**8 种**（其中 1–3 是量产主流，4–8 各有明确适用面）。

| # | 组合名 | 用到的族 | 代表实现 / 产品 | 一句话适用面 |
|---|---|---|---|---|
| **D1** | **固定路径 AGV**（基础设施导引 + 拓扑路线 + 安全链） | 8.1/8.2/8.3 + 1.5/3.8 + 6.6/6.7 + 4.9 | SICK NAV350、Kollmorgen NDC8（⚠️ 厂商页 / ✅ 手册 PDF） | 路线固定、要认证、要可审计的工厂物流 |
| **D2** | **经典 AMR / Nav2 默认栈**（先验 2D 图 + AMCL + 图搜索 + DWA/RPP） | 1.1 + 2.4(AMCL) + 3.1/3.3 + 4.3/4.4 + 6.1/6.2 | Nav2 bringup 默认（AMCL + NavFn + DWB/RPP）；ROS 1 时代 `move_base` 同构 | 室内服务机器人、教学、绝大多数 ROS 项目起点 |
| **D3** | **高速全向竞速栈 / RM 强队栈**（LIO + 配准重定位 + Smac/MPPI + 时间维代价图） | 2.3 + 2.4(GICP/ICP) + 3.2 + 4.6 + 1.3(STVL) + 4.9 | COD_NAV（MPPI Omni + Small Point-LIO + STVL）、`pb2025_sentry_nav`（Point-LIO + small_gicp + 全向 PID 跟踪） | 场地固定/半固定、要高动态、可接受调参成本 |
| **D4** | **无图探索栈**（在线 SLAM + 前沿探索 + 恢复） | 1.1/1.3 + 2.3 + 3.7 + 6.2 + 5.1 | `m-explore-ros2`/`explore_lite` + slam_toolbox | 第一次进场、未知场地、救援/巡检 |
| **D5** | **示教-重放栈**（topometric 路线跟随，无全局度量图） | 1.8 + 5.6 + 2.1/2.3（局部里程计）+ 4.x 局部控制 | `wiln`（激光 teach-and-repeat）；工业示教 AGV | 路线固定但环境外观变化大、部署时间紧 |
| **D6** | **学习式端到端 / 基础模型栈**（视觉导航模型直接出动作或子目标） | 7.5 + 7.1/7.3/7.7 + 1.10 | ViNT/GNM/NoMaD（visualnav-transformer）、VLFM、NaVid、π0/openpi | 研究、Demo、跨场景零样本；**离比赛可靠性尚远** |
| **D7** | **语义/语言任务栈**（LLM/VLM 分解任务 + 经典栈执行） | 7.8 + 1.6 + 3.8 + 5.7 | SayCan、LM-Nav、Code as Policies、NavGPT | 人机交互研究、服务机器人原型 |
| **D8** | **多机 / 车队栈**（多机协同 + 调度协议） | 9.1–9.6 + 8.6 + 8.7 + 5.10 | Open-RMF、VDA5050 生态、Kimera-Multi/Swarm-SLAM | 园区、仓储、多机协同作业 |

**为什么不是 81 种（本报告综合）**：族之间有四类硬约束——① **互斥**（2.4 与 2.9 都在发 `map→odom`；1.1 与 1.9 是不同世界的表示）；
② **依赖**（3.7 探索必须有在线建图；5.5 覆盖必须有覆盖服务器；D1 必须有安全链）；
③ **成本不叠加**（同时上 3.2+4.5+4.6 只会在"调参地狱"里互相对冲，收益不叠加）；
④ **场景筛掉大半**（RM 场地是平地、单机、强对抗、每局重开 ⇒ 1.2/1.9/9.x 基本不选）。
所以真正的自由度是"**在 8 种组合里选 1 种，再在 3~4 个插槽里换实现**"。

---

## E. 我们 vs COD 在这张地图上的位置

**（4 句结论）** 我们和 COD 落在**同一格 D3（高速全向竞速栈）**，差别主要在"地图/重定位这一层"：
COD（`rmul2026`/Gitee RM2026）走的是 **2.9 静态 `map→odom` + 在线 slam_toolbox + 手教 CSV 航点（5.3）+ MPPI（4.6）**，
即"**不重定位、靠每局当场重建地图，任务顺序由人写死**"；我们走的是 **2.4 四选一重定位（AMCL/ICP/slam_toolbox/Cartographer 纯定位）+ `mode` 三态（mapping / slam_nav / nav）**，
即"**重定位是可换插槽、地图是资产**"，但我们的全局规划器仍是 **3.3 NavFn**、控制器只接了 **4.3/4.4/4.5**（**4.6 MPPI 与 3.2 Smac / 3.1 Theta\* / 3.8 nav2_route / 6.3 collision_monitor 都已在 `/opt/ros/humble` 装好但未接入**）。
所以在地图上：**我们"覆盖的格子"更宽（表示层、定位层的可替换性更强），COD"在所选格子里的深度"更深（MPPI 参数配方、STVL 时间维、速度包络已跑到 7.5 m/s 级）**；
我们最短的补课路径不是加更多算法，而是把**已装未接**的 Smac/MPPI/collision_monitor 接进槽位，并补上 5.3/5.4 的航点与巡逻任务层（COD 已有、我们目前偏薄）。

**支撑细节（可选读）**：

| 轴 | 我们（本 bench，自查） | COD_NAV（rmul2026 / Gitee RM2026，只读核对） |
|---|---|---|
| 1 世界表示 | 1.1 2D costmap（static+obstacle+inflation）+ 1.3 STVL（仅 global） | 1.1 + 1.3 **local 与 global 都上 STVL**（`voxel_decay`），1.12 车体点云裁剪盒 |
| 2 定位 | 2.3 FAST-LIO/Point-LIO + **2.4 四选一**（AMCL/ICP/slam_toolbox/Cartographer 纯定位） | 2.3 Small Point-LIO + **2.9 静态 `map→odom`（无重定位）** |
| 3 全局规划 | 3.3 NavFn（Smac/Theta\* 已装未接） | Smac2D（2025 master）/ MPPI 侧全局（2026） |
| 4 局部控制 | 4.3 RPP（默认）/4.4 DWB/4.5 TEB；**4.6 MPPI 已装未接** | **4.6 MPPI Omni @50 Hz**，速度包络到 7.5 m/s；4.9 SG 平滑 |
| 5 任务 | stock BT + 自研分段工具；5.3 waypoint_follower 已启动但未用于成体系巡逻 | **5.3 CSV 航点 + 单点/多点导航** + `waypoint_editor` |
| 6 安全/恢复 | 6.2 stock 恢复；6.3 collision_monitor 已装未接 | 6.2 BT：RateController 3 Hz + IsStuck→BackUp + 10 次重试 |

---

## F. 未能找到权威来源的地方 / 存疑项（请按此打折使用）

1. **"WVN world representation" 无法确权**。任务描述里提到"learned latent/occupancy（e.g. WVN world representation）"，
   我用多组检索都没有找到与导航地图表示对应的、可核对的 WVN 论文或仓库（检索命中的都是无关内容）。
   ⇒ 1.10 我改用**可核对的三个来源**（Wild Visual Navigation、Learning Social Cost Functions、DreamerV3）替代，**WVN 未写入**。
   如果这是某篇内部/最新论文，请补一个链接，我再补进 1.10。
2. **Nav2 官方文档路径在 2025 年后改版**：旧的 `docs.nav2.org/configuration/packages/...` 全部 404，
   现行结构是 `docs.nav2.org/rolling/configuration_and_development/configuration_guide/...`。我**逐条抓到 200 才写入**；
   但我们 bench 是 **Humble**，Humble 时代的参数键名/默认值与 rolling 文档可能有差异——**以 `/opt/ros/humble/share/...` 里的实际插件与参数为准**（本报告只保证"文档页存在"，不保证"键名与 Humble 完全一致"）。
3. **标准正文无法直接引用**：ISO 3691-4 / ISO 13849-1 / ANSI-A3 R15.08 的正文是**付费且对脚本返回 403**，
   我只能给官方目录页 + IEC 功能安全总览页。第 6.6 节里"**安全链必须独立于导航计算机**"这一表述是
   **我基于标准框架 + 厂商资料（Pilz/SICK）的综合**，**不是**逐条引用的标准原文；要用于产品认证请以标准正文与认证机构意见为准。
4. **部分厂商页面对脚本返回 403**（Kollmorgen、NDC Solutions、Pilz、ANSI、automate.org、SAGE/Wiley/IEEE/ScienceDirect）：
   这些链接是**官方页面**，但我无法用脚本校验正文内容，只能确认"域名/路径来自官方检索结果"。
   特别地，`ndcsolutions.com` 与 `kollmorgen.com` 的 AGV/NDC8 页面属**厂商宣传资料**，不是中立来源。
5. **AGV 导航范式的"权威综述"只有会议/行业级来源**：磁条/二维码/反射板这几族（8.1–8.4）我找到的是
   IOP 会议论文页（✅ 200 但为会议页）与 MHI 行业博客（✅ 200，**行业协会博客，非同行评议**）；
   **没有**找到近 5 年内、可自由访问、覆盖"磁带 vs 二维码 vs 反射板 vs SLAM"的中立对比综述。8.8 的结论因此标注为"本报告综合"。
6. **RM 生态仓库只做了 README/文件清单级核对**：§B.10 的"用了哪些族"来自各仓库 README 与我们既有的只读深读
   （`docs/cod_nav_2026_deep_dive.md` 等），**未 clone、未编译、未运行**；赛季分支仍可能变化。
   另外：用户提到的 XDU `sentry_navigation` 作为**仓库名**在 GitHub 上是 404，实际活跃仓库是 **`XDU-IRobot/SentryNav2026_XDU`**
   （`sentry_navigation` 是其中的包目录），我按实际存在的仓库写。
7. **Hydra 论文 ID**：我先抓到的 `2105.07664` 与 Hydra 无关（已废弃），改用 ✅ `2201.13360`（标题已核对）。
   **MonoGS 的 arXiv ID 我未能确权**，因此 1.9 只给仓库链接，不给论文 ID。
8. **少数族只有"技术报告/书籍"级来源**：4.1（Pure Pursuit 用 CMU 技术报告 PDF）、9.1（编队控制综述为 Automatica 付费页）、
   9.2（MRTA 为 IJRR 付费页）、9.4（多机探索为 IEEE 付费页）——**可点、权威，但正文需机构订阅**。
9. **未做的事**：我没有逐一复现任何算法的性能数字；本报告里的"代价"判断（例如 MPPI 的 CPU 开销、TEB 的调参成本）
   部分来自我们 bench 的既有实测文档（`docs/algorithm_matrix.md`、`docs/issues_and_findings.md`）与来源页面的定性描述，
   **不是**本次新做的基准测试。
