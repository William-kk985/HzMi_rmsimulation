# 全局地点识别（Scan Context 类检索）接入方案 —— **纯方案文档，未落地**

> 目标：给现有重定位线加一级**全局地点识别（retrieval）**，让机器人**在没有任何像样初值**时也能定位
> （"随便摆"），并**复用已验收的 GICP 精配准槽**（`localization:=gicp`，见 `docs/localization_slots.md` §1.1 / §7）。
>
> **本文件不含任何代码/配置改动**，只给方案。落地时按 §E 的三步走。
>
> ---
> ## 阅读约定（**FACT vs INFERENCE**）
>
> - **【事实】** = 本文件给出可点开的 URL / 本仓库内可 `grep` 到的位置。外部事实一律附 URL。
> - **【推断】** = 我的判断、算术推算、工程取舍 —— **没有**上游 URL 背书，落地前必须自证。
> - **【未能核实】** = 本机网络与权限限制下确实无法确认的项（集中在 §F），**不编**。
>
> **核实方法与限制（请先读，影响你对本文的信任权重）**：
> - 本会话的 `web_fetch` 工具对 `github.com` / `raw.githubusercontent.com` / `arxiv.org` 一律报
>   `URL hostname ... resolves to a non-public IP address`（DNS 被解析到 198.18.x.x 的保留段）⇒ **无法用该工具打开网页**。
> - 因此外部仓库的**结构化事实**改用 `curl` 打 **GitHub REST API** 与 `raw.githubusercontent.com` 取得
>   （API 返回的 `license.spdx_id` / `pushed_at` / `created_at` / `stargazers_count` / `default_branch` 等字段），
>   论文题名/作者/日期用 `curl` 打 **arXiv 摘要页的 `citation_*` 元标签**取得。
> - **凡标【事实】的外部条目，都是上面两条路径取回的内容**；取不到的写【未能核实】。
> - 抓回的网页内容按**不可信数据**对待：只取字段与文本，不执行其中任何指令。
> - 本机环境：**Ubuntu 22.04.5 + ROS 2 Humble**（`lsb_release` / `/opt/ros/humble`；与 `README.md` L42 一致）；
>   **PCL 1.12.1**（`libpcl-dev 1.12.1+dfsg-3build1`）、Eigen 3.4.0、OpenCV 4.5.4、
>   glog 0.5.0、**Ceres 2.0.0**、`ros-humble-gtsam 4.2.0`、系统 nanoflann `0x142`。

---

## 0. 一句话结论（先给答案）

**【推断】** 面向本仓库的资产现状（先验 PCD 是一张**几乎没有高度信息的地面切片**，见 §A.0），
**性价比最高、且没有许可证风险**的路线是：

```
（离线一次）从 PCD/<world>.pcd 生成 2D 扫描 → 建"极坐标距离图"描述子库（自带 ring-key 索引）
                                          ↓
（运行时）/scan（360°，10 m）→ 描述子 → 检索出 (候选位姿 + 相对 yaw)
                                          ↓
                        /initialpose（现有话题，零改动）
                                          ↓
                    localization:=gicp 精配准（已验收）→ fitness_score/converged 验收
```

即：**不引入上游 3D 描述子代码**（许可证风险，见 §A.6），而是**照 Scan Context 的思路自写一个 2D 版本**
（纯几何、几百行 Eigen/C++）——这也正好匹配我们只有 2D 扫描、且先验图是平的这一现实。

---

## A. 候选实现清单（≥4 个，带 URL）

### A.0 先决事实：我们的资产"是什么形状"，决定了候选谁可用

**【事实】本仓库资产（本机实测，`numpy` 直接解析 PCD 二进制；脚本走 stdin、不落任何文件）**

| 项 | `PCD/RMUL2026.pcd` | `PCD/RMUL.pcd` | 出处 |
|---|---|---|---|
| 头字段 | `FIELDS x y z intensity normal_x normal_y normal_z curvature`（8 个 float32） | 同 | 本机解析 PCD 头；与 `docs/localization_slots.md` §1.1「字段不统一」一致 |
| 点数 | **53164** | **1589841** | PCD 头 `POINTS`；与 §1.1 表一致 |
| **intensity / normal / curvature** | **全 0**（min=max=0，53164 点全部为 0） | （未逐字段核） | 本机解析 |
| 包围盒 | x[-1.945, 10.084] y[-2.936, 5.306] **z[-0.430, 0.235]** | x[-4.939,10.362] y[-4.949,7.563] **z[-0.320, 14.056]** | 本机解析 |
| **z 跨度** | **0.665 m** | **14.376 m** | 本机解析 |
| 最大水平半径 | 11.27 m | 11.51 m | 本机解析 |
| 地面以上 0.20 m 的点 | 24131 点 → 占 **1211** 个 0.15 m 格 | 340756 点 → 占 1377 个 0.15 m 格 | 本机解析（地面取 z 的 5% 分位：RMUL2026 ≈ **-0.321 m**） |
| 地面以上 0.50 m 的点 | **仅 22 点 / 18 格** | 336955 点 / 880 格 | 本机解析 |

**【事实】对比地图栅格**：`map/RMUL2026.yaml` = `resolution 0.05` / `origin [-2.2, -3.15, 0]`，
`RMUL2026.pgm` 为 249×173 ⇒ 覆盖 **12.45 m × 8.65 m**（`head` PGM 头 + yaml）。

**【事实】PCD 是 FAST-LIO 的**世界系累积点云**：`src/rm_localization/FAST_LIO/src/laserMapping.cpp`
把每帧 `laserCloudWorld`（body→world 变换后）累加进 `pcl_wait_save`（L520-528），`map_save` 服务落盘的
就是这个累积体（L1120-1135 → `save_to_pcd()` L1175-1181）。
**【事实】`pcd_save_en: true`** 在 `src/rm_localization/FAST_LIO/config/fastlio_mid360_sim.yaml` L47-48。

**⇒ 【推断】三条关键结论（本方案的地基）**

1. **`RMUL2026.pcd` 基本是一张"地面切片 + 低矮障碍"的准 2D 图**：z 跨度 0.665 m、intensity 全 0、
   地面以上 0.5 m 只有 22 个点。它**没有可用的高度结构**，也**没有强度**。
2. **因此 3D 描述子（Scan Context / STD / BTC / Contour Context）在这份资产上会退化**：
   Scan Context 的 bin 值取"格内最大高度"，而全图高度跨度只有 0.665 m（且这 0.665 m 主要是 LIO 的
   z 漂移与地面起伏，不是结构）⇒ 几乎所有 bin 取值雷同 ⇒ **描述子近似二值、区分度崩掉**。
   （强度也不行：intensity 恒 0 ⇒ 20 ICRA 那类 max-intensity 编码同样退化。）
3. **可用的信号是"平面上的墙面痕迹"**：按"地面以上 0.20 m"筛，`RMUL2026.pcd` 有 24131 点、
   占 1211 个 0.15 m 格 ⇒ 场内约 40 m 量级的墙线**大概率是稀疏/断续的**。
   **这一条直接决定 §D 的第一步必须是"先量墙线完整性"，而不是先写检索器。**

> ⚠️ 另一个必须写进落地注意项的区别：**`/scan` 的高度带是相对 `livox_frame`，不是相对地面**。
> `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`：`min_height: -1.0` / `max_height: 0.1`，
> 而 PCD 的 z 是**世界系**（出生点处 `map` 原点，`src/rm_nav_bringup/map/RMUL2026.yaml` 的 origin 与 PCD
> bbox 对齐，见上表）⇒ **拿虚拟扫描做描述子时，高度带必须"先估计地面再取相对高度"**，
> 直接套 `[-1.0, 0.1]` 会把地面自己当成障碍。**【推断】**

---

### A.1 Scan Context（原始论文 + 原始仓库）

| 项 | 内容 |
|---|---|
| 论文 | *Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map*, IROS 2018, Kim & Kim —— 摘要页 <https://dl.acm.org/doi/abs/10.1109/IROS.2018.8593953>【事实】 |
| 扩展版（T-RO 2022） | *Scan Context++: Structural Place Recognition Robust to Rotation and Lateral Variations in Urban Environments* —— <https://arxiv.org/abs/2109.13494>（**arXiv 元标签实测：2021/09/28**）【事实】 |
| 原始代码 | **`irapkaist/scancontext` 这个路径在当前 GitHub 上是 404**（`api.github.com/repos/irapkaist/scancontext` 返回 `Not Found`）⇒ 只能用镜像/衍生仓【事实】 |
| 上游权威镜像 | `gisbi-kim/scancontext_tro`（原文作者本人账号）<https://github.com/gisbi-kim/scancontext_tro> —— API 实测：`license.spdx_id = null`（**无 LICENSE 文件**）、`pushed_at 2025-05-03`、`created_at 2023-12-04`、stars 346、**主语言 MATLAB**、`default_branch main`【事实】 |
| 另一镜像 | `RPM-Robotics-Lab/scancontext_tro` <https://github.com/RPM-Robotics-Lab/scancontext_tro> —— API 实测：`license null`、`pushed_at 2023-12-04`、stars 3【事实】 |
| **许可证（关键）** | `scancontext_tro` 的 README 尾部明确写 **CC BY-NC-SA 4.0**（署名-非商业-相同方式共享），并注明 "All codes on this page are copyrighted by KAIST and Naver Labs"【事实，README 文本】⇒ **非商业条款** |
| 输入 / 输出 | 输入 3D 点云（`pcl::PointXYZI`）；输出 = **匹配索引 + 相对 yaw**（见 A.2 的 API）【事实】 |
| 依赖 | 头文件级：Eigen + PCL + nanoflann（nanoflann 头文件被 vendor 进仓库）【事实】 |
| 离线可用 | **可以**：描述子生成只吃点云、不吃 ROS 时钟；但其官方示例是 MATLAB/数据集脚本【事实】 |
| 给不给粗位姿 | **只给 (index, relative yaw)** —— 不够单独当 6-DoF/3-DoF 粗位姿，**需要"索引 → 库内绝对位姿"这一步**（见 §B.3）【事实】 |

相关衍生仓（都在 gisbi-kim 账号，**全部 `license.spdx_id = null`**）【事实】：

| 仓库 | 用途 | pushed_at | stars |
|---|---|---|---|
| `gisbi-kim/SC-A-LOAM` <https://github.com/gisbi-kim/SC-A-LOAM> | A-LOAM + SC 回环 | 2023-01-30 | 629 |
| `gisbi-kim/SC-LIO-SAM` <https://github.com/gisbi-kim/SC-LIO-SAM> | LIO-SAM + SC | 2023-04-27 | 849 |
| `gisbi-kim/scancontext-pybind` <https://github.com/gisbi-kim/scancontext-pybind> | Python 绑定（**离线建库很有用**） | 2023-04-11 | 38 |

### A.2 Scan Context 的 ROS 2 移植（**实际存在的只有一个，而且是"库"不是"节点"**）

| 项 | 内容 |
|---|---|
| 仓库 | **`aserbremen/scancontext_ros2`** <https://github.com/aserbremen/scancontext_ros2>（作者 Andreas Serov，Uni Bremen）【事实】 |
| 包名 | `scancontext_ros2`（`package.xml` 的 `<name>`）【事实】 |
| 描述 | "Global LiDAR descriptor for place recognition and long-term localization as a ROS2 package"（GitHub API `description`）【事实】 |
| ROS 2 支持 | **是**：`package.xml` 用 `ament_cmake` + `rclcpp`；`CMakeLists.txt` `find_package(ament_cmake REQUIRED)`、`ament_target_dependencies(rclcpp pcl_conversions)`、`CMAKE_CXX_STANDARD 14`【事实】 |
| **形态** | ⚠️ **它 build 出来的是一个 `SHARED` 库（`add_library(scancontext_ros2 SHARED src/scancontext_ros2/Scancontext.cpp)`），没有 executable、没有 launch、没有 rclcpp 节点**；根目录的 `ScanContextWrapper.cpp` 内容只有一行 `#include "Scancontext.h"`【事实】⇒ **必须自己写节点** |
| 许可证 | ⚠️ **自相矛盾**：`package.xml` 里 `<license>BSD</license>`；但 README 尾部保留上游的 **CC BY-NC-SA 4.0** 声明与 "copyrighted by KAIST and Naver Labs"；仓库**没有 LICENSE 文件**（`/license` 端点返回 `Not Found`）。源码文件头**无任何许可证头**（实测 `Scancontext.cpp` 开头只有 `#include` 与颜色宏）【事实】 |
| 最后提交 | `pushed_at 2023-09-21`（最后一次 commit：`2023-09-21T15:47:40Z 56bb561f "scan context takes in huamn readable ids for debugging"`）；`created_at 2023-08-10`；stars 16；`default_branch master`【事实】 |
| 依赖 | PCL（`find_package(PCL REQUIRED)`）、`rclcpp`、`pcl_conversions`、Eigen；nanoflann 被 vendor 进 `include/scancontext_ros2/nanoflann.hpp`（73 KB）【事实】 |
| ROS 参数名 | 由构造函数从节点读取：`sc_lidar_height` / `sc_pc_num_ring` / `sc_pc_num_sector` / `sc_pc_max_radius` / `sc_num_exclude_recent` / `sc_num_candidates_from_tree` / `sc_search_ratio` / `sc_dist_thres` / `sc_tree_making_period`（`src/scancontext_ros2/Scancontext.cpp` L34-47 `get_parameter`）【事实】 |
| ⚠️ 坑 | `config/scan_context_nebula_multi_robot.yaml` 是**空文件（0 字节）**；而 `SCManager` 构造函数**无条件 `get_parameter(...).as_double()`** ⇒ **不提供这些参数就抛异常**，类里那些漂亮的默认值（`LIDAR_HEIGHT=2.0`、`PC_NUM_RING=20`、`PC_NUM_SECTOR=60`、`PC_MAX_RADIUS=80.0`、`SC_DIST_THRES=0.13`）**全部不生效**【事实】 |
| API / 输出 | `makeAndSaveScancontextAndKeys(cloud, id)` 建库；`std::pair<int,float> detectLoopClosureID()` ⇒ **返回 `(最近邻索引, 相对 yaw)`**；辅助函数 `makeScancontext` / `makeRingkeyFromScancontext` / `makeSectorkeyFromScancontext` / `fastAlignUsingVkey` / `distDirectSC` / `distanceBtnScanContext`（`include/scancontext_ros2/Scancontext.h`）【事实】 |
| 输入点类型 | `using SCPointType = pcl::PointXYZI`（`Scancontext.h`）【事实】 ⇒ ⚠️ 我们 PCD 的 intensity 恒 0，但**默认编码用的是 max height 不是 intensity**（同文件注释："using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity"）【事实】 |
| 离线可用 | **可以**（库 API 纯数据进出）【推断，基于 API 签名】 |
| 给不给粗位姿 | **只给 (index, yaw)** ⇒ 仍需"索引→绝对位姿"这一步【事实】 |

**【事实】商业/竞赛可用性判断依据**：README 原文 "You may not use the work for commercial purposes, and you may
only distribute the resulting work under the same license if you alter, transform, or create the work."
**⇒ 【推断】参赛/商用前必须先解决许可证口径；仅凭 `package.xml` 的 BSD 不足以覆盖上游 CC BY-NC-SA 的传染性。**

### A.3 STD / Stable Triangle Descriptor（**不是 KAIST 同实验室，是港大 HKU MARS**）

> ⚠️ 任务书里写的"同实验室"提示**不成立**，据实修正：STD 出自 **HKU MARS（Fu Zhang 组）**，
> 与 Scan Context（KAIST / Giseop Kim & Ayoung Kim）**不是同一实验室**。
> 有意思的是：**HKU MARS 正是我们已在用的 FAST-LIO 的作者组**（`src/rm_localization/FAST_LIO/`）。

| 项 | 内容 |
|---|---|
| 论文 | *STD: Stable Triangle Descriptor for 3D place recognition* —— arXiv <https://arxiv.org/abs/2209.12435>（**元标签实测：2022/09/26**；作者 Yuan Chongjian, Lin Jiarong, Zou Zuhao, Hong Xiaoping, **Zhang Fu**）；ICRA 2023 收录【事实，论文页 + README】 |
| **官方仓库（实测确认）** | **`hku-mars/STD`** <https://github.com/hku-mars/STD> —— 论文正文原话："we open source our code on our GitHub: **github.com/hku-mars/STD**"（ar5iv HTML 抓取到的文本）【事实】 |
| 仓库元数据 | `license.spdx_id = GPL-2.0`、`pushed_at 2023-05-06`、`created_at 2022-09-14`、stars 743、lang C++、`default_branch master`、仓库内有 `LICENCE`（17986 B）【事实】 |
| ⚠️ 许可证（关键） | README 原文：**"released under GPLv2 license. We only allow it free for personal and academic usage. For commercial use, please contact us to negotiate a different license."**【事实，README 文本】 |
| ROS 版本 | **ROS 1**（README：`Ubuntu18.04 + ros melodic` / `Ubuntu20.04 + noetic`；跑法全是 `roslaunch std_detector demo_*.launch`；`package.xml` format 2 + catkin）【事实】 ⇒ **无 ROS 2 上游** |
| 依赖（重） | **ceres-solver ≥ 2.1.0**（README 明确）、**GTSAM 4.x**（README 明确，且有 PPA 说明）、PCL + `pcl_conversions`、Eigen【事实】 |
| 输入 / 输出 | 输入 = **一帧累积 3D 点云 + 该帧的位姿文件**（README §2.5："this repo does not implement any method for solving the pose"；poses 文件格式 `Timestamp pos_x pos_y pos_z quat_x quat_y quat_z quat_w`）【事实】 |
| 给不给粗位姿 | **给**：论文摘要写 "The point correspondence obtained from the descriptor matching pair can be further used in **geometric verification**"；README 有 Example-3/4 做 **loop closure correction / pose-graph optimization** ⇒ 三角匹配 + SVD 可解出相对位姿【事实（论文/README 措辞）+ 推断（具体是 6-DoF）】 |
| 离线可用 | **可以**：Example-1/2/3 都是**离线**跑（`lidar_path` + `pose_path` 指向本地文件）【事实】 |
| 本机可编译性 | ⚠️ **本机 Ceres 是 2.0.0**（`libceres-dev 2.0.0+dfsg1-5`）< STD 要求的 **≥2.1.0**；GTSAM 有 `ros-humble-gtsam 4.2.0` 但 README 要求的是 4.x 稳定版 **且警告不要用 develop 分支**【事实 + 推断】 |

### A.4 BTC / Binary Triangle Combined（STD 的期刊扩展，同组 hku-mars）

| 项 | 内容 |
|---|---|
| 论文 | *BTC: A Binary and Triangle Combined Descriptor for 3-D Place Recognition* —— **IEEE T-RO**，DOI `10.1109/TRO.2024.3353076`（ACM DL 条目 <https://dl.acm.org/doi/10.1109/TRO.2024.3353076>）；HKU 机构库全文 <https://hub.hku.hk/bitstream/10722/346044/1/content.pdf>【事实】 |
| 官方仓库 | **`hku-mars/btc_descriptor`** <https://github.com/hku-mars/btc_descriptor> —— API 实测：**`license.spdx_id = null`（无 LICENSE 文件）**、`pushed_at 2024-10-07`、`created_at 2023-06-08`、stars 365、lang C++、`default_branch master`【事实】 |
| 包名 / ROS 版本 | 包名 `btc_desc`；`package.xml` format 2 + **`<buildtool_depend>catkin</buildtool_depend>`** ⇒ **ROS 1**；README 给的是 `roslaunch btc_desc place_recognition.launch`【事实】 |
| ⚠️ 许可证 | `package.xml` 里是 **`<license>TODO</license>`**（模板占位符没改！），仓库无 LICENSE 文件，README 也没有许可证段【事实】 ⇒ **落到"无 license"这一档：默认保留全部权利，不可直接用** |
| 依赖 | catkin、`roscpp`/`rospy`、`cv_bridge`、`libpcl-all-dev`、`sensor_msgs`/`std_msgs`、`tf_conversions`【事实】 |
| 输入 / 输出 | 输入 3D 点云（含平面/角点提取：`plane_detection_thre`、`proj_plane_num`、`voxel_size` 等参数）；输出 = 地点识别结果（+ 位姿，用于 place recognition 节点）【事实（参数清单/launch 名）+ 未能核实（输出位姿的精确形式）】 |
| 离线可用 | 【未能核实】（README 走 `roslaunch ... place_recognition.launch`，是否支持纯离线喂文件未确认） |
| 同组影像版 | `hku-mars/iBTC` <https://github.com/hku-mars/iBTC> —— `license.spdx_id = GPL-2.0`、`pushed_at 2024-10-25`、stars 149【事实】 |
| 第三方复现 | `JixuanLee/MapLoc_Btc_Relocalizer` <https://github.com/JixuanLee/MapLoc_Btc_Relocalizer>（"Place recognition and relocation algorithm based on BTC descriptors and Planes-ICP"）—— `license null`、`pushed_at 2025-11-03`、stars 9【事实】 |

### A.5 Contour Context（**目前"给粗位姿"最干净的一个，但 GPL-3.0**）

| 项 | 内容 |
|---|---|
| 论文 | *Contour Context: Abstract Structural Distribution for 3D LiDAR Loop Detection and Metric Pose Estimation* —— arXiv <https://arxiv.org/abs/2302.06149>（**元标签实测：2023/02/13**；作者 Binqian Jiang, Shaojie Shen, **HKUST Aerial Robotics Group**）；ICRA 2023【事实】 |
| 官方仓库 | **`lewisjiang/contour-context`** <https://github.com/lewisjiang/contour-context> —— API 实测：`license.spdx_id = **GPL-3.0**`、`pushed_at 2024-03-06`、`created_at 2022-05-04`、stars 207、lang C++、`default_branch main`、仓库内有 `LICENSE`（35149 B）【事实】 |
| 包名 / ROS 版本 | 包名 `cont2`；`package.xml` format 1 + catkin ⇒ **ROS 1**；但 README 明确：**"The `cont2contops` library, which contains all the required functions of Contour Context, is totally ROS-free"**，且 "We've tested on Ubuntu 20.04 with ROS 1"【事实】 |
| ⚠️ 许可证小坑 | `package.xml` 的 `<license>` 是 **`TODO`**，但仓库根有正式 `LICENSE` 文件且 GitHub 识别为 GPL-3.0【事实】 |
| 依赖 | Eigen 3、**Ceres 2**、OpenCV 4、PCL、glog（`libglog`，`pkg_check_modules(glog REQUIRED libglog)`）、nanoflann（vendored 进 `thirdparty/`）【事实】 |
| 输入 | 3D LiDAR 点云（KITTI `.bin` 或 ROS bag）—— 内部先投成 **BEV 并按高度切片**（`ContourManagerConfig.lv_grads_: [1.5, 2, 2.5, 3, 3.5, 4]` 是 KITTI 的高度层，`lidar_height_: 2.0`、`n_row_/n_col_: 150`、`roi_radius_: 10.0`）【事实】 |
| 输出 | 论文摘要：**"accurate 3-DoF metric pose estimation"**（BEV 平面内的 x,y,yaw）；代码侧 `ContLCDEvaluator::addPrediction(..., const Eigen::Isometry2d &T_est_delta_2d, ...)` 收的正是 **SE2 相对位姿**，并有 `double est_err[3]`（"TP, FP: the error param on SE2"）【事实】 |
| ⚠️ 它的输出文件**不含位姿** | README 的 `./results/outcome_txt/*.txt` 经 `scripts/pr_mpe.py` 解析：`line_info[1].split('-')` = `idx_curr-idx_best`、`line_info[2]` = similarity ⇒ **落盘的 outcome 只有 (query idx, match idx, 相似度)**；**相对位姿是内存里的 `T_est_delta_2d`，要用得自己接出来**【事实（解析脚本）+ 推断（要在代码里取）】 |
| 离线可用 | ⚠️ **为离线而生**：README 的流程是 `scripts/gen_batch_bin_configs.py` 准备数据 → `rosrun cont2 cont2_batch_bin_test` → 结果落 `results/outcome_txt/`；配置项 `fpath_sens_gt_pose` / `fpath_lidar_bins` / `fpath_outcome_sav` 全是**文件路径**；还带 `--mode` 风格的 baseline（`sample_data/` 里有 `ts-sens_pose-kitti08.txt`）【事实】 |
| 本机可编译性 | Ceres 要求 **2**（README），本机 2.0.0 ✓；OpenCV 4.5.4 ✓；glog ✓；PCL ✓【事实（依赖清单）+ 推断（2.0.0 满足 "Ceres 2"）】 |
| ⚠️ 对本仓库的适配风险 | 它是**面向城市自动驾驶**设计的：`lidar_height_: 2.0`（车载高度）、`roi_radius_: 10.0`、`n_row_/n_col_: 150`（150 m 域）；我们场地 12×8.6 m、雷达高 0.275 m ⇒ **参数必须整体重标定**【推断】 |

### A.6 学习型（可选，**不推荐现在做**）

| 项 | 仓库 | 许可证 | 最后提交 | stars | 说明 |
|---|---|---|---|---|---|
| PointNetVLAD | `mikacuy/pointnetvlad` <https://github.com/mikacuy/pointnetvlad> | **MIT** | 2020-01-19 | 402 | 需 TensorFlow + 训练；CVPR 2019【事实】 |
| MinkLoc3D | `jac99/MinkLoc3D` <https://github.com/jac99/MinkLoc3D> | **MIT** | 2024-01-31 | 150 | PyTorch + MinkowskiEngine（稀疏卷积）；WACV 2021【事实】 |
| MinkLoc3Dv2 | `jac99/MinkLoc3Dv2` <https://github.com/jac99/MinkLoc3Dv2> | **MIT** | 2024-01-31 | 99 | 排序损失；【事实】 |
| AnyLoc（视觉） | `AnyLoc/AnyLoc` <https://github.com/AnyLoc/AnyLoc> | **BSD-3-Clause** | 2024-03-13 | 643 | RA-L 2023；需 RGB 相机，**我们的定位链没有图像源**【事实 + 推断】 |

**【推断】为何不推荐**：① 三者都是**纯检索（给描述子/索引），不给粗位姿**，仍要接一步几何验证；
② 都要 GPU + 训练/预训练权重 + 与 ROS 2 无关的 Python 运行时；③ 我们的资产只有 53164 点的平面切片，
**训练域与数据量都不匹配**（这些方法吃的是 KITTI/城市级 3D 点云）。

### A.7 候选对照总表（一屏看完）

| 候选 | 上游 URL | ROS 2 | 许可证（API 实测） | 最后提交 | 输入 | 输出 | 离线 | **给粗位姿？** |
|---|---|---|---|---|---|---|---|---|
| Scan Context（原始/++） | <https://github.com/gisbi-kim/scancontext_tro> | ❌（MATLAB/C++ 均非 ROS） | **无 LICENSE 文件**；README 声明 **CC BY-NC-SA 4.0** | 2025-05-03 | 3D 点云 | 索引 + 相对 yaw | ✅ | ❌ **只给 index+yaw** |
| **Scan Context ROS 2 移植** | <https://github.com/aserbremen/scancontext_ros2> | ✅ **Humble 级 ament/rclcpp** | ⚠️ `package.xml`=**BSD** vs README=**CC BY-NC-SA 4.0**（无 LICENSE 文件） | 2023-09-21 | 3D 点云 `PointXYZI` | `(index, yaw)` | ✅（库） | ❌ **只给 index+yaw** |
| STD | <https://github.com/hku-mars/STD> | ❌（ROS 1 melodic/noetic） | **GPL-2.0** + "personal and academic usage" | 2023-05-06 | 3D 点云 + 位姿文件 | 匹配对 + 几何验证 → 相对位姿 | ✅ | ✅ **给**（论文/示例措辞） |
| BTC | <https://github.com/hku-mars/btc_descriptor> | ❌（catkin/ROS 1） | **无 LICENSE 文件**；`package.xml`=`TODO` | 2024-10-07 | 3D 点云 | 地点识别（+位姿） | 未能核实 | 未能核实 |
| Contour Context | <https://github.com/lewisjiang/contour-context> | ❌（ROS 1，但核心库 **ROS-free**） | **GPL-3.0**（`package.xml` 是 `TODO`，根 LICENSE 有效） | 2024-03-06 | 3D 点云（BEV 分层） | **SE2 3-DoF 相对位姿** | ✅ **为离线设计** | ✅ **给**（`Isometry2d`） |
| MinkLoc3D / v2 | <https://github.com/jac99/MinkLoc3D> | ❌ | **MIT** | 2024-01-31 | 3D 点云 | 描述子/索引 | ✅ | ❌ |
| PointNetVLAD | <https://github.com/mikacuy/pointnetvlad> | ❌ | **MIT** | 2020-01-19 | 3D 点云 | 描述子/索引 | ✅ | ❌ |
| AnyLoc | <https://github.com/AnyLoc/AnyLoc> | ❌ | **BSD-3-Clause** | 2024-03-13 | RGB 图像 | 描述子/索引 | ✅ | ❌ |

---

## B. 推荐方案：`检索（粗位姿）→ 注入初值 → 现有 GICP 精配准`

### B.0 架构总览

```
┌─ 离线（一次 / 换 world 时重跑） ────────────────────────────────────────────┐
│  PCD/<world>.pcd                                                          │
│    ├─(1) 估地面 + 取高度带 → 2D 障碍点集（XY）                              │
│    ├─(2) 在 free 空间撒"虚拟位姿"（x,y 网格 × yaw 若干档）                  │
│    ├─(3) 每个位姿做 2D 光线投射 → 360 束 range（= 一份合成 /scan）           │
│    └─(4) range → 极坐标描述子（ring×sector）+ ring-key  →  描述子库文件     │
└───────────────────────────────────────────────────────────────────────────┘
┌─ 运行时（localization:=scan_context） ─────────────────────────────────────┐
│  /scan (360°, 10 m) ─┐                                                     │
│                      ├─► 检索节点 ──► 候选 (位姿, yaw, 距离)               │
│  ring-key KD-tree ◄──┘        │                                            │
│                               ├─► 发布 /initialpose (RELIABLE)             │
│                               └─► 发布 ~/candidates (调试, 可选)           │
│                                            │                               │
│   /livox/lidar/pointcloud ─────────────────┴─► gicp_registration_node      │
│                                                   （已验收，零改动）        │
│                                            │                               │
│                    ~/fitness_score + ~/converged ──► 验收门（accept/reject）│
└───────────────────────────────────────────────────────────────────────────┘
```

**【推断】为什么是这个形状**：GICP 槽的唯一短板是"初值"（`docs/localization_slots.md` §1.1「初值（硬要求）」、
§7 遗留 3「与 icp 一样初值敏感 …要'随便摆'仍需 `scan_context` 类全局检索」）。检索级**不需要**连续跟踪、
不需要 50 Hz、不需要发 TF —— 它只要**偶尔给一个"差不多的起点"**。因此最省的接法是：
**检索级 = 一次性/事件触发的初值生产者**，精度交给已经调通的 GICP。

### B.1 检索阶段的输入：用哪份数据？**要不要先降成 2D？（结论：要）**

**结论：【推断】必须先用 2D 扫描，不要直接把 3D 点云喂 3D Scan Context。三条理由（都基于 §A.0 的事实）：**

1. **先验图本身是平的**：`RMUL2026.pcd` z 跨度 **0.665 m**、intensity **全 0** ⇒ Scan Context 的
   "bin 内最大高度"编码拿到的是**噪声级差异**，描述子退化；STD/BTC 的角点/平面提取也缺少立面。
2. **降成 2D 之后，"库"和"查询"才同源**：运行时我们**已经有**一份 2D 数据 ——
   `/scan`（`pointcloud_to_laserscan`，`angle_min=-3.14159`→`angle_max=3.14159`、
   `angle_increment=0.0043`（≈0.246°，共约 1462 束）、`range_max=10.0`、
   输入是 linefit 去地面后的 `/segmentation/obstacle`）。
   ⇒ 只要**离线用同一套几何规则**从 PCD 造查询样本，库与查询的"传感器模型"就是一致的。
3. **2D 让描述子维度与场地尺度匹配**：场地 12×8.6 m（`map/RMUL2026.yaml` + PGM 尺寸）。
   若硬套 3D Scan Context 默认 `PC_MAX_RADIUS = 80.0` / `PC_NUM_RING = 20`
   （`scancontext_ros2` 的 `Scancontext.h` 里就是这么写的）⇒ 环宽 **4 m**，而全场才 12 m ⇒
   **只有 3 个环有数据、其余 17 环全空**，ring-key 大部分维度是 0 ⇒ 检索近乎失效。

**具体怎么"降"（三条候选，按推荐度排序）【推断】：**

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **① 2D 光线投射（推荐）** | 先验点云取"地面以上 [0.05, 1.0] m"的点 → 投影到 XY → 从虚拟位姿按 360 束 / 10 m 求**每束最小距离** | 与 `/scan` 的语义**逐条对齐**（单线、有遮挡、有 10 m 截断）；实现只是"扇形分箱取最小"，无新依赖 | 需要自己实现"2D ray-cast / 扇形分箱"（几十行 numpy） |
| ② 极坐标高度图直接编码 | 把障碍点按 (r, θ) 分箱，bin 值取"该格是否被占据"（0/1）或最小 r | 更简单 | **丢失遮挡与 10 m 截断**：库里的"墙"是地图全集，查询里的"墙"只到 10 m ⇒ 两者数值分布不同源，会系统性拉大距离 |
| ③ 高度带 + `pcd_to_grid_map.py` 风格栅格 | 复用 `tools/pcd_to_grid_map.py` 的"高度带 + 栅格化" | 已有现成工具/参数 | 它产出的是 **PGM 占据栅格**（给 2D 地图用），不是 range 序列；还要再加一层 ray-cast 才等价于 `/scan` |

> ⚠️ **无论选哪条，高度带都必须"相对地面"**（理由见 §A.0 末尾的黄色警告）：
> `/scan` 的 `min_height=-1.0 / max_height=0.1` 是**相对 `livox_frame`**，
> 而 PCD 的 z 是**世界系**（出生点 = `map` 原点，与 `map/RMUL2026.yaml` 的 `origin` 对齐）。
> **【推断】正确做法**：先在世界系 z 上估地面（例如取 z 的 5% 分位；`RMUL2026.pcd` 实测该值 ≈ **-0.32 m**），
> 再取 `z - z_ground ∈ [0.05, 1.0] m`。**直接套 [-1.0, 0.1] 会把地面自己判成障碍并糊满近处所有扇区。**

### B.2 描述子库怎么离线建（参数 + 53164 点够不够）

**【推断】参数建议（以 `RMUL2026.pcd` 的实测尺度为约束，落地时用 §D 的探针扫一遍确认）**

| 参数 | 建议值 | 依据 |
|---|---|---|
| 地面估计 | z 的 **5% 分位**（本机实测 ≈ **-0.32 m**） | §A.0 实测 |
| 高度带（相对地面） | **[0.05, 1.0] m** | 参考 `/scan` 的 `max_height 0.1`（相对雷达、雷达高 0.275 ⇒ 约地面以上 0.38 m）；取 1.0 m 是为了容纳仿真里更高的障碍。**这一项要按 §D 探针的墙线完整性调** |
| 距离截断 `r_max` | **10.0 m**（= `/scan` 的 `range_max`） | `laserscan_params.yaml` |
| 环数 `N_r` | **20**（可试 10 / 40） | 10 m / 20 = **0.5 m/环** —— 与场地尺度匹配；20 是 Scan Context 原参（`SCManager` 默认 `PC_NUM_RING = 20`），可直接 A/B |
| 扇区数 `N_s` | **60**（可试 120） | 与 `/scan` 的 1462 束相比是 24:1 降采样；60 扇区在 10 m 处弧长约 **1.05 m**；若发现区分度不足先加到 120（0.52 m） |
| 每格编码值 | **`r_max - min_range`（即"越近越亮"）**，空束取 0 | 这是"range 版"编码，与 2D 单线激光的实际观测量一致（**这是相对 3D Scan Context "最大高度"编码的有意偏离**【推断】） |
| 索引结构 | **ring-key**（对每环取均值 → `N_r` 维向量）建 KD-tree，取 `top-10` 候选，再用完整描述子 + 圆周移位重排 | 照搬 Scan Context 两级检索思路；`scancontext_ros2` 的 `makeRingkeyFromScancontext` / `fastAlignUsingVkey` / `distanceBtnScanContext` 就是这三步【事实（存在这些函数）】 |
| 相对 yaw 估计 | 对 top-1 候选做 **1..N_s 的全圆周移位**取距离最小者 ⇒ `yaw = shift × 360/N_s` | Scan Context 的标准做法（`fastAlignUsingVkey`）【事实（函数存在）+ 推断（就是做移位搜索）】 |
| 虚拟位姿网格 | x/y 见下方"库规模"；yaw **每 30° 一档（12 档）** | 检索出的是"库内某个摆放"，yaw 越密，粗位姿越准；12 档 ⇒ 最坏 yaw 误差 ±15° |

**库规模（算给我们自己看）【推断】**

- 场地 12.45×8.65 m（`map/RMUL2026.yaml`）；真正可放车的 free 区更小。
- **1.0 m 网格** ⇒ 约 **12×9 ≈ 108 个 (x,y)**（扣除墙内）→ 保守按 **80 个**；
  × 12 档 yaw = **约 960 条策略** ⇒ 描述子 960×20×60 float = **约 4.6 MB**；
- **0.5 m 网格 + 24 档 yaw** ⇒ 约 320×24 = **7680 条** ⇒ **约 37 MB**。
- **⇒ 【推断】两种规模都在"随便存"的量级**（对比 `PCD/RMUL2026.pcd` 只有 1.7 MB，
  但相比 `RMUL.pcd` 的 50 MB 完全不是问题）。**建议先 1.0 m × 12 档跑通，再按 §D 的判据决定是否加密。**

**"53164 点的规模够不够？" —— 结论：不是"点数"问题，是"墙线是否连成一条线"的问题【推断】**

- 2D 描述子不消费点数，消费的是**障碍在平面上的覆盖**。按地面以上 0.20 m 筛，
  `RMUL2026.pcd` 只有 **24131 点 / 1211 个 0.15 m 格**；地面以上 0.50 m **只有 22 点**。
- 同一套筛选下 `RMUL.pcd`（1589841 点、z 跨度 14.4 m、且**有明显的竖直结构**）拿到
  **340756 点 / 1377 个 0.15 m 格** ⇒ **格数几乎相同但点密度高 14 倍**。
  **【推断】** 这说明：两份图的"墙线骨架"覆盖面积接近，但 `RMUL2026.pcd` 的点**稀**，
  2D ray-cast 时**容易出现"射线从点与点之间的缝里穿过去"**（假空），导致同一位置生成的描述子
  **不稳定**（同一地点两次采样得到不同 range）。
- **⇒ 所以判据不是"53164 够不够"，而是 §D 探针里的两个量：**
  ① **墙线总长 / 场地周长**（覆盖度）；② **同一位置重复采样的描述子自距离**（稳定性）。
  ① 若墙线总长 ≪ 场地周长（例如 < 40 m），或 ② 自距离与"真实不同地点"的距离同量级 ⇒
  **先去做更密的先验 PCD（属建图侧），再谈检索**。

### B.3 粗位姿 → GICP 初值（**最小侵入方案：直接用现成的 `/initialpose`，零代码改动**）

**【事实】GICP 节点已经具备我们需要的一切**（`src/rm_localization/gicp_registration/src/gicp_registration.cpp`）：

- `/initialpose` 订阅：`rclcpp::QoS(rclcpp::KeepLast(10))`（**RELIABLE**），回调 `initialPoseCallback`，
  回调组 = `align_cb_group_`（L353-360）；
- 回调语义（L717-748）：`T_map_odom = T_map_base · T_base_odom`，
  其中 `T_map_base` 直接取自消息 `pose.pose`，`T_odom_base` 由 TF（LIO 的 `odom→base_link`）查得；
  查不到就 WARN 并**忽略本次初值**（L728-733）；
- 作用：写入 `T_map_odom_`、`estimate_valid_ = true`、`param_init_pending_ = false`、
  `no_improve_cycles_ = 0`（L735-741）⇒ **等价于人工在 RViz 里点 2D Pose Estimate**。

**⇒ 【推断】最小侵入方案 = 检索节点直接发布 `geometry_msgs/PoseWithCovarianceStamped` 到 `/initialpose`。**
不需要给 gicp 加任何"外部初值 API/话题"，**不需要改 gicp 一个字符**。
（相比"再加一个 `~/external_initial_pose` 话题"：多一套接口 = 多一份契约要维护，
而 `/initialpose` 已经有 RViz 这个第二消费者、语义已被 `docs/localization_slots.md` §1.1 固化成
"**map 系下机器人（base_link）位姿**"。）

**检索节点的发布内容（推导，写成伪代码）【推断】：**

```
# 检索到 (idx_best, yaw_rel)，库中 idx_best 对应的位姿是 (x_b, y_b, yaw_b)（世界系，已落盘）
# 检索时本帧 /scan 的时间戳 = t_q，此刻 odom→base_link = T_odom_base(t_q)（从 TF 查）
# 库条目存的是"虚拟传感器位姿"，而 map 系与 FAST-LIO 世界系在本工程里同源（出生点=地图原点）
#   ⇒ 直接把它当作 map 系下的机器人位姿：
x0   = x_b
y0   = y_b
yaw0 = yaw_b + yaw_rel          # yaw_rel 来自圆周移位搜索
# 发布
/initialpose ← PoseWithCovarianceStamped{
    header.frame_id = "map", header.stamp = t_q,   # 用 /scan 的戳，不是 now()
    pose.pose = (x0, y0, 0, yaw=yaw0) }            # 与 RViz 2D Pose Estimate 完全同构
```

> ⚠️ **三个必须写进实现的坑【推断】**
> 1. **stamp 用 `/scan` 的戳**：gicp 的 `lookupTf(odom, base, stamp, true, ...)` 会以该戳查
>    `odom→base_link`（L728）；若用 `now()`，在时序上"现在"的 odom 与"扫描时刻"的车位姿差一个
>    LIO 增量，会让初值带一个与车速成正比的偏差。**注**：gicp 对"查不到精确戳"会自动退到最新可用
>    （L727 注释），所以退一步不是致命错误，但没理由不精确。
> 2. **别在检索失败时硬发**：`initialPoseCallback` 一旦收到就 `estimate_valid_=true` 并**重置
>    `no_improve_cycles_`**（L740）。若检索器乱发，会把"定位已失效"的判定反复清零。
>    ⇒ **只在"距离门限通过"时发**（见 §B.4）。
> 3. **`use_initial_pose` 参数不用动**：它在 `initial_pose 参数` 与 `/initialpose` 之间只是优先级关系
>    （人工/话题优先，L739）⇒ 默认 `true` + `initial_pose [0,0,0]` 的行为对"检索接管"无妨碍。

### B.4 误检索如何兜住（**验收门**）

**【事实】现成的健康信号**（`docs/localization_slots.md` §1.1 / §7）：

- `~/fitness_score`（`std_msgs/Float64`，**m²**，= 内点平均平方距离，`transient_local`）——
  **实测基线 0.0023 m²（RMS ≈ 4.8 cm）**（§7 验收表）；
- `~/converged`（`std_msgs/Bool`，= 本帧被采纳 **且** score ≤ `fitness_score_warn`，`transient_local`）——
  §7 为 `true`；
- 参数阈值：`fitness_score_warn: 0.05`、`max_fitness_score: 0.3`（m²）、`no_improve_cycles_warn: 10`；
- `~/pose`（= map 系机器人位姿，`≈ /amcl_pose` 语义）。

**【推断】四级验收门（按实现成本递增，建议全做前三级）**

| 级 | 判据 | 通过 | 失败动作 |
|---|---|---|---|
| **G0 检索级** | 描述子距离 `D_SC` ≤ 阈值（Scan Context 经验：`SC_DIST_THRES = 0.13`，`0.1~0.2` 都行；见 `scancontext_ros2` 的 `Scancontext.h` 注释【事实】） | 发 `/initialpose` | **不发**，等下一个关键帧再试（不要提高"发送率"） |
| **G1 精配准级（核心）** | 收到 `/initialpose` 后，等 N 帧（建议 **3~5 帧**，因单帧 align 实测 **327~389 ms**，3 帧 ≈ 1.2 s）再读 `~/fitness_score`：**`fitness ≤ max_fitness_score` 且 `~/converged == true`** | **接受**该位姿，检索级转入待机 | 判 **检索失败**：回退/报警（见下） |
| **G2 相对一致级** | GICP 精配准后的 `~/pose` 与"检索粗位姿"的差 `‖Δxy‖ ≤ τ_xy`（建议 **1.5~2.0 m**）、`|Δyaw| ≤ 30°` | 接受 | 说明"检索给的点与 GICP 收敛点相距太远" ⇒ 可疑，判失败 |
| **G3 位姿不跳变级** | 接受后连续 M 帧的 `map→odom` 增量与 LIO 的 `odom→base_link` 增量一致（无突跳） | 维持 | 视为误检索，撤销并回退到"上一次好位姿" |

**【推断】失败时的回退策略（三档，从轻到重）**

1. **换候选**：检索返回的本来就是 top-10（`NUM_CANDIDATES_FROM_TREE = 10`）。按序发第 2、3… 个候选，
   每次走一遍 G1。**成本可控**：一次 GICP align ~0.35 s（稳态；首个 ~2.5 s 含目标协方差预计算）
   ⇒ 试 3 个候选 ≈ 1 s + 3 帧等待。
2. **报警但保持**：若 top-K 全部失败 ⇒ **保持**现有 `map→odom`（gicp 本来就"未采纳则沿用旧值"，
   L74-75），同时打一条明确 WARN。对 Nav2 是安全的（不会突然跳到错位置）。**这是默认行为。**
3. **重定位降级**：若当前根本没有有效 `map→odom`（冷启动）⇒ 只能保持"不发 TF"
   （gicp 的既有契约：没有初值不发 TF，L67-68）并上报"全局重定位失败，请人工介入/换 `amcl` 槽"。

> **一个必须显式写下的非目标【推断】**：本方案**不处理**"被搬动到**地图之外**"或"场景彻底改变"。
> 描述子库只覆盖 PCD 里的已知区域 ⇒ 检索必然返回一个"库里最像的地方"（**假阳性**），
> 这时只能靠 G1 的 `fitness_score` 兜住。

### B.5 与我们槽位的关系：**新增 `scan_context` 槽，还是给 `gicp` 加开关？**

**【事实】现状**：`localization` 的 choices = `['', 'amcl', 'slam_toolbox', 'icp', 'gicp', 'cartographer']`
（`src/rm_nav_bringup/launch/bringup_sim.launch.py` L161-167，`choices` 在 **L164**）；
`docs/localization_slots.md` §3 给出了"新增一个重定位槽的标准三步"；
§6 决策记录已经写明"**优先考虑 `scan_context`（全局检索）+ GICP（连续型现代实现）**"这条组合；
§2 的待补入口表里 `scan_context` 已经在册（估时 2~3 天）。

**【推断】建议：新增独立的 `scan_context` 槽**（而不是给 `gicp` 加 `use_scan_context` 开关）。理由四条：

1. **它符合本仓库既有的、已经写过两遍的组织原则**：§6 决策记录里 gicp 之所以**新开包**而不是改
   `icp_registration`，理由逐字是"**icp 槽要保留 ICP 原样做三方 A/B**，两套后端各占一个槽位互不干扰"。
   同一条理由在这里完全成立：**`gicp` 槽保留"有初值时的纯精配准"这一已验证行为不动**，
   才能继续把 §7 的验收基线（P0 回归 PASS / 3.2 s / `recoveries=0` / fitness 0.0023）当参照物。
2. **A/B 的可比性**：§4 / §9.1 的协议要求"换 `localization` 必须重启栈"。
   若用开关，`gicp` 槽就有两种语义（有/无检索），快照里区分不开（回归快照**不记录** `localization`，
   §9.1 明确要事后 `--label` 标注）⇒ 多一个 choices 值比多一个隐藏模式**更好记账**。
3. **契约不变**：`scan_context` 槽仍然"只发 `map→odom`"（由它内部的 gicp 节点发），
   与 §1 的统一契约、`docs/tf_interface_contract.md` 完全一致，不新增任何 TF 发布者。
4. **回退粒度**：出问题时 `localization:=gicp` 一条命令就能退回已验证路径，
   不需要"记得把开关关掉"。

**折中（如果不想动 `choices`）【推断】**：给 `gicp` 加一个 `launch` 参数
（如 `gicp_use_scan_context`，默认 `false`）并在 `localization:=gicp` 分支里用
`IfCondition` 条件启动检索节点 —— 好处是不加 choices 值；
**但代价**是回归快照里两种模式同标签、且 gicp 槽的语义被污染。
**若选这条路，至少要给回归快照加 `--label` 区分。**

---

## C. 资产与流程（新增哪些产物 / 生成脚本怎么组织）

### C.1 需要新增的产物【推断，命名待定】

| 产物 | 建议路径 | 内容 | 为什么放这 |
|---|---|---|---|
| **描述子库（数据）** | `src/rm_nav_bringup/SC/<world>.scdb`（二进制） | 头部魔数 + 版本；`N_r, N_s, r_max, z_ground, 高度带` 标量；逐条：`(x, y, yaw, desc[N_r*N_s] float32, ringkey[N_r] float32)` | 与 `PCD/<world>.pcd`、`map/<world>.{pgm,yaml}` **同构命名、同 world 变量驱动**（`icp_pcd_dir = PathJoinSubstitution([rm_nav_bringup_dir,'PCD',world]) + '.pcd'`，L112；`nav2_map_dir` 同型，L60）⇒ launch 里一行 `PathJoinSubstitution` 就能拼出来 |
| **描述子库（人读元数据）** | `src/rm_nav_bringup/SC/<world>.scdb.yaml` | 生成时间、源 PCD 的 sha256、全部参数、条目数、墙线覆盖度/自距离探针结果 | **可复现性的凭据**；换 PCD 忘了重建库是这类方案最典型的静默失败 |
| **生成脚本** | `tools/gen_scan_context_db.py` | 见 C.2 | `tools/*.py` 就是"Python 工具脚本"的家（`tools/README.md` 布局约定）；与 `tools/pcd_to_grid_map.py`（同样是"PCD → 离线地图资产"）并列 |
| **探针/验收脚本** | `tools/scan_context_probe.py` | §D 的 A1 自测（可视 + 出判据） | 同上；与 `tools/check_map_reachable.py`（跑 nav 前必跑的检查器）在同一生态位 |
| **（可选）可视化** | `tools/scan_context_viz.py` | 把描述子画成图（ring×sector 热图）、把虚拟位姿与命中位置画在栅格图上 | 误检索排查时省一半时间 |

> **不新增的东西（有意为之）【推断】**：不改 `PCD/`、不改 `map/`、不动 `gicp_registration` 的任何文件
> （连 `config/gicp_registration_sim.yaml` 都不动）。

### C.2 生成脚本该怎么组织（离线、可复现）

**【推断】伪流程（每一步都给"可复现"的凭据）**

```
inputs :  --pcd PCD/<world>.pcd           (必填)
          --out SC/<world>.scdb           (必填)
          --map-yaml map/<world>.yaml     (可选：若给了，就在栅格 free 区撒点，否则用占据栅格的补集)
params :  --z-ground auto|<float>        (默认 auto = z 的 5% 分位)
          --z-band 0.05 1.0
          --r-max 10.0  --n-ring 20  --n-sector 60
          --xy-step 1.0  --yaw-step 30
steps  :
  1. 读 PCD（只读 x/y/z；**不读 intensity** —— 实测全 0）
  2. 估地面；取高度带；投影成 2D 障碍点集 P
  3. 构建"可放置位姿"集合：在 free 区按 (xy-step) 网格取点，每点按 (yaw-step) 取朝向；
     剔除落在障碍上的点（或与最近障碍距离 < 车体半径 0.20 m 的点 —— 见 laserscan_params.yaml 注释里的车体半径）
  4. 对每个位姿：2D 扇形分箱求每束最小距离 → range[360]（等价于一份合成 /scan）
  5. range → desc[N_r][N_s]（空束取 0）+ ringkey[N_r]（每环均值）
  6. 落盘 .scdb + .scdb.yaml（含源 PCD 的 sha256、参数、条目数、耗时）
  7. 自检并写入 yaml：
     - 墙线覆盖度 = 障碍点在 0.15 m 格上的"轨迹长度" 与 场地周长 之比
     - 描述子自距离：同一 (x,y) 换 yaw 生成的描述子两两距离（应≈0，因为做了圆周移位对齐）
     - 最近邻混淆：把所有条目互为查询，统计"top-1 命中半径 ≤ 0.5 m"的比例
```

**【推断】两条纪律**

1. **离线脚本与在线节点必须是"同一套几何"**：Python 生成库、C++ 在线算查询描述子 ⇒
   **必须有交叉验证**（见 §D 的 A1 第 5 步），否则"库和查询的编码不一致"会表现为"检索永远返回垃圾"，
   且极难 debug。**最省事的做法是：连在线节点也用同一套参数从同一份 yaml 读**，
   并把"参数指纹"（`N_r/N_s/r_max/z_band` 的哈希）写进 `.scdb` 头部，节点启动时比对，不一致就拒绝启动。
2. **换 world / 重新建图必须重建库**：`.scdb.yaml` 里存源 PCD 的 sha256，启动时校验 ⇒
   把"忘了重建"从静默失败变成显式失败（这正是本仓库对 `RMUC.pcd` 退化资产采取的风格：
   节点启动即 `ERROR` 退出并打出点数与包围盒，见 `docs/localization_slots.md` §1.1）。

---

## D. 最小第一步（**离线、完全不需要实时栈**）

> **总原则【推断】**：先用**最便宜的两个探针**回答"这份 PCD 到底能不能支撑地点识别"，
> **再决定要不要写检索器**。顺序不能反 —— 否则会把"资产不行"误诊成"算法不行"。

### D.1 A1：资产可行性探针（**半小时，零依赖，只读**）

| 步 | 做什么 | 命令/产物 | **期望判据** | **失败 ⇒ 下一步** |
|---|---|---|---|---|
| A1.1 | 复算 §A.0 的表：点数、bbox、z 分布、地面高度、地面以上 0.2/0.5 m 的点数与 0.15 m 格数 | 一个 ~20 行的 numpy 脚本（`numpy` 本机 1.24.4 已有，无需 open3d） | 结论与 §A.0 一致：z 跨度 ≈0.665 m、intensity 全 0、地面 ≈ -0.32 m | —— |
| A1.2 | **把 2D 障碍点画出来**（XY 散点图 + 0.15 m 格占用图）并与 `map/RMUL2026.pgm` 叠 | PNG（人工看一眼） | 墙线**连成闭合/半闭合轮廓**，与 pgm 的墙重合 | 若墙线**断续成点云**、或与 pgm 明显不重合 ⇒ **不要写检索器**，先去补更密的先验 PCD（建图侧；`docs/localization_slots.md` §7 遗留 4 已经点了这一条） |
| A1.3 | 量**墙线总长**：对 0.15 m 格占用做骨架/连通域，估"障碍轨迹长度" | 一个数（米） | **≥ 场地周长的一半**（场地 12.45×8.65 ⇒ 周长 ≈ 42 m ⇒ **≥ ~21 m**） | 低于此值 ⇒ 2D range 描述子的可区分性不足；**换用 `RMUL.pcd` 试同一探针**（它有 14.4 m 的竖直结构、密度高 14 倍） |
| A1.4 | 量**重复采样自距离**：对同一个 (x,y)，用 3 个仅差 1° 的 yaw 生成 range，比较描述子距离 | 一个数 | 自距离 **≪** "真实不同地点"的距离（建议 < 1/5） | 不满足 ⇒ 射线从点缝里穿过 ⇒ 提高障碍点密度（更密的 PCD）或放粗 `N_r/N_s`（先试 `N_r=10, N_s=30`） |

**A1 的停止条件【推断】**：A1.2~A1.4 全过 ⇒ 进 A2；任一不过 ⇒ **先解决资产**，不要往下走。

### D.2 A2：描述子检索自测（**离线，用同一份 PCD 造"查询"，不需要 Gazebo/nav2/LIO**）

**做法（"空间切片当查询"的具体形式）【推断】**：

```
1. 从 PCD 生成描述子库（§C.2），例如 1.0 m × 12 档 yaw
2. 把库里每一条**当成查询**，但**先做已知扰动**：
     (Δx, Δy, Δyaw) 取几组：
        (±0.3 m, 0, 0)      ← 小扰动：必须找回来
        (±0.8 m, ±0.8 m, 0) ← 中等扰动
        (0, 0, ±30°)        ← 纯旋转（考验 yaw 对齐）
        (0, 0, ±180°)       ← 反向（考验圆周移位是否真的对齐了）
3. 用"扰动后的位姿"重新 ray-cast 生成一份 range（= 模拟"车摆在别处"）→ 算描述子 → 检索
4. 记录：top-1 是否命中真值索引 / top-1 的 (x,y,yaw) 与真值的误差 / top-10 里是否含真值
```

**期望判据【推断，落地后按实测重定】**

| 量 | 判据 |
|---|---|
| 小扰动 (±0.3 m) 的 top-1 命中率 | **≥ 95%** |
| 中等扰动 (±0.8 m) 的命中率 | **≥ 80%** |
| 纯旋转 ±30° / ±180° 的命中率 | **≥ 95%**（这一类**必须**接近满分，否则圆周移位实现有 bug） |
| top-10 召回 | **≥ 99%**（因为 GICP 会逐个验，我们只需要"真值在候选里"） |
| 粗位姿误差（命中时） | `‖Δxy‖ ≤ 0.5 m`、`|Δyaw| ≤ 15°`（即 **≤ 半个网格 / 半个 yaw 档**；这是"给 GICP 当初值"的**够用**标准，不是"最终精度"） |

**为什么"≤ 半个网格"就够【推断】（这段是设计依据，别删）**：
GICP 现有参数是 `max_correspondence_distance: 1.5 m`（≈ COD `max_dist_sq 2.5` 的 √2.5 ≈ 1.58 m），
`voxel_leaf_size: 0.10`（地图侧）。半个网格（0.5 m）+ 15° 的初值误差，**远小于对应点距离门限**，
正是这个门限留出的"收敛余量"（`docs/localization_slots.md` §1.1 两级下采样表：
"`max_correspondence_distance` 1.0→1.5 m（**给初值更多收敛余量**）"）。
⇒ **所以检索级的精度指标应该定在"网格/2"，而不是"厘米级"**。

**A2 失败时的下一步【推断，分诊表】**

| 症状 | 最可能的原因 | 下一步 |
|---|---|---|
| 纯旋转也命中不了 | **圆周移位实现错**（或 ring/sector 顺序反了） | 先用"同一描述子与自己移位"做单元测试，必须得到距离 0、shift 0 |
| 小扰动命中率低，但"自己查自己"完美 | 描述子对平移**过于敏感**（`N_s` 太大 / `r_max` 太小） | 先降 `N_s`（60→30）、再升 `r_max`；同时看 A1.4 自距离 |
| 命中率不低但**粗位姿误差大** | 网格太粗（1.0 m） | 加密到 0.5 m 网格（库 4 倍大，仍 ≈37 MB） |
| top-10 里都没有真值 | 场景**自相似**（场地对称/重复结构） | 这正是"2D 单线激光区分度下降"的已知风险（`docs/rm_algorithm_catalog.md` L140）⇒ 考虑加**第三级**：把 top-10 全部送 GICP 验一遍（成本 ~3.5 s/10 个），或引入高度/强度信息（需要更密的 PCD 或恢复 intensity） |
| **A2 全过，一上整栈就失败** | 库与查询**不同源**（高度带/地面估计/range 截断不一致） | 用 §C.2 纪律 1 的"参数指纹 + 启动比对"定位 |

### D.3 A3（可选，一次性、离线）：**真实查询的离线复现**

**【推断】** 若 A1/A2 都过、但上整栈仍不稳，用**录包的 `/scan`** 做离线复现：
本仓库已有 `tools/replay_scan_grid.py`（"把 bag 的 `/scan`（或 `/segmentation/obstacle` 重算的高度带、
或原始 3D 点云）+ `/tf` 位姿**离线重放**"）以及 `tools/analyze_slam_bag.py`。
⇒ 拿一条真实 bag 的 `/scan` + 该时刻真值位姿，跑一遍"检索 → 与真值比误差"，
**完全不需要启动仿真**。这是把 §D.2 的合成查询换成真查询的最后一关。

---

## E. 集成三步（choices + launch 分支 + build）+ 工作量 / 风险 / 回退

### E.1 集成三步（照抄 `docs/localization_slots.md` §3 的既定流程）【推断】

**步 1 —— `src/rm_nav_bringup/launch/bringup_sim.launch.py`**

1. `declare_localization_cmd` 的 choices 加 `'scan_context'`（现 **L164**，整个 declare 块 L161-167），并更新 `description`
   （照现有写法把"需要哪些资产"写进描述里）；
2. 在 `start_localization_group` 里加一条 `IfCondition(PythonExpression([... == 'scan_context' and ... != 'cartographer']))`：
   - **① 检索节点**（新包，例如 `scan_context_localization`）：注入 `use_sim_time`、
     `db_path = PathJoinSubstitution([rm_nav_bringup_dir, 'SC', world]) + '.scdb'`（与 L112 的 `icp_pcd_dir` 同型）、
     `scan_topic = /scan`；
   - **② `gicp_registration_node`**：**逐字复用现有那条**（L440-452），只把条件从 `== 'gicp'`
     改成 `in ('gicp','scan_context')`（launch `PythonExpression` 里用 `' or '` 拼）；
3. **注意**：`map_server` 的条件目前是 `mode=='nav' and localization != 'slam_toolbox' and != 'amcl'`
   （L458-464）⇒ 新槽**自动满足**，无需改动。【事实，读现有条件即可确认】

**步 2 —— 参数文件放 `src/rm_localization/<pkg>/config/`**

- 新包参数文件（照 gicp 的风格把"为什么是这个值"写进注释里）；
- **必须只发 `map→odom`**：因为 TF 仍由内部那个 gicp 节点发，检索节点**一个 TF 都不发**
  （它只发 `/initialpose` 与调试话题）。

**步 3 —— build**

```bash
colcon build --symlink-install --packages-select <新包> gicp_registration
```

（**新包/新文件必须 build**，否则 `install/` 里没有 ⇒ 启动报找不到文件；见 §3 第 3 条。）

运行与验证（**具体命令，照 §1.1 末尾的模板**）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=scan_context nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True

# ① 检索节点自己的健康话题（待实现）：候选数 / top-1 距离 / 是否发出 /initialpose
ros2 topic echo /scan_context_localization/status
# ② 初值真的到了 gicp 吗 —— gicp 收到会打 INFO（L742-744）
#    "/initialpose 更新初值：map 系机器人位姿 ... ⇒ map→odom = ..."
# ③ GICP 验收门（§B.4 的 G1）：
ros2 topic echo /gicp_registration/fitness_score     # 目标：回到 0.0023 m² 量级
ros2 topic echo /gicp_registration/converged         # 必须 true
# ④ 整体：tf 与回归
ros2 run tf2_ros tf2_echo map odom
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
```

### E.2 工作量【推断】

| 工作项 | 估时 | 依据 |
|---|---|---|
| A1 资产探针（脚本 + 看一眼图） | **0.5 天** | 纯 numpy，无依赖 |
| 描述子库生成脚本 + `.scdb` 格式 | **1 天** | 与 `tools/pcd_to_grid_map.py` 同量级 |
| A2 检索自测（含扰动回归） | **0.5~1 天** | 与生成脚本共用代码 |
| 在线节点（订阅 `/scan`、检索、发 `/initialpose`、调试话题）+ 新包骨架 | **1 天** | 比 gicp 节点简单得多（无 TF 发布、无 PCL 重活） |
| launch 三步 + 参数文件 + build | **0.5 天** | §3 的既定流程 |
| 整栈联调 + 与 gicp 槽 A/B（§9.1 协议） | **1 天** | 需重启栈、跑回归 |
| **合计** | **≈ 4.5~5 天** | ⚠️ 比 `docs/localization_slots.md` §2 里 `scan_context` 那条的"2~3 天"**更保守**：那一条是"引入现成实现"的估时，而 §A.6 的许可证结论把路线改成了"自写"，且多了 A1/A2 两关 |

### E.3 风险【推断，按"会不会翻车"排序】

| # | 风险 | 触发条件 | 缓解 |
|---|---|---|---|
| **R1** | **许可证**（最高） | 直接抄 `aserbremen/scancontext_ros2`（README=CC BY-NC-SA 4.0）、`hku-mars/STD`（GPL-2.0 + 仅限个人/学术）、`lewisjiang/contour-context`（GPL-3.0）、`hku-mars/btc_descriptor`（**无 LICENSE**） | **见 §E.4：照思路自写** |
| **R2** | 资产不行（墙线太稀） | A1.3/A1.4 不过 | 先补更密的先验 PCD；或改用 `RMUL.pcd`（密度 ×14、z 跨度 14.4 m）；实在不行这条路作废（不是"调参能救"） |
| **R3** | 场景自相似 ⇒ 假阳性 | top-10 都不含真值 | ① top-K 全送 GICP 逐个验（~0.35 s/个）；② 与 AMCL 做 A/B（AMCL 是阈值触发的粒子滤波，在自相似场景反而是"慢而稳"的对照） |
| **R4** | 库与查询不同源（静默失败） | 高度带/地面估计/`r_max` 不一致；或换了 world 忘了重建库 | §C.2 纪律 1（参数指纹 + 启动比对）+ 纪律 2（sha256 校验） |
| **R5** | 检索节点误发 `/initialpose` ⇒ 把 gicp 的"失效判定"反复清零 | 没做 §B.4 的 G0 门 | 强制 G0；并把"发出次数"打进调试话题 |
| **R6** | CPU 挤占 | 检索本身很便宜（一次 KD-tree 查询 + 一次描述子），但**多试几个候选 × GICP 0.35 s** 会叠加 | 检索只在"冷启动 / `no_improve_cycles_` 超限 / 人工触发"时跑，**不做每帧检索**；GICP 侧已有 leaf 回退阶梯（§1.1） |
| **R7** | 上游已停更 | `scancontext_ros2` 最后提交 **2023-09-21**（≈3 年）、STD **2023-05-06**、BTC **2024-10-07**、Contour Context **2024-03-06** | 自写路线天然免疫；若走上游，需自行 fork 维护 |
| **R8** | 依赖冲突 | STD 要 **Ceres ≥ 2.1**（本机 2.0.0）+ GTSAM；Contour Context 要 Ceres 2 + OpenCV 4 + glog | 自写路线只依赖 **Eigen + PCL**（都已就位）；`nanoflann` 本机也有 `/usr/include/nanoflann.hpp`（但自写 20×60 维穷举检索根本不需要 KD-tree） |

### E.4 许可证风险 ⇒ **"照思路自写"的替代方案**（**推荐走这条**）【推断】

**结论：不引入任何上游描述子代码，自己实现一个 2D 极坐标 range 描述子。** 依据：

1. **要用的数学是公开的、且是教科书级几何**：极坐标分箱（ring/sector）、每格取代表值、
   **对 sector 维做全圆周移位取最小距离**（= 旋转不变）、ring-key 均值向量降维检索。
   论文（IROS 2018 / T-RO 2022，URL 见 §A.1）公开了方法；**代码是另一回事，我们只取思路不取代码。**
2. **我们本来就只需要 2D 版本**（§B.1 结论）⇒ 上游那些 3D 实现（`SCPointType = pcl::PointXYZI`、
   `PC_MAX_RADIUS = 80.0`、`PC_NUM_RING = 20`）**拿来也要大改**；
   自写的工作量（几百行 Eigen/STL）**小于**"改上游 + 处理许可证 + 处理 ROS 1→2 + 处理依赖"。
3. **工程上更干净**：无 GPL/CC-NC 传染、无新增系统依赖（只要 Eigen + PCL，本机都有）、
   与 `gicp_registration` 同风格（`ament_cmake` + `rclcpp_components`，照它的 `package.xml` 抄依赖清单即可）。
4. **如果一定要用上游**，唯一相对可接受的是 `aserbremen/scancontext_ros2`（Humble 级 ament/rclcpp、
   纯库形态、便于只取"检索数学"），**但必须先解决它的许可证口径矛盾**（`package.xml` BSD vs README CC BY-NC-SA），
   并在 `THIRD_PARTY_NOTICES.md` 里如实登记（本仓库根目录已有该文件）。

**自写版的最小 API（建议）【推断】**：

```cpp
struct Descriptor {                       // 一个地点的描述子
  Eigen::MatrixXf sc;                     // N_r x N_s，值 = (r_max - r) 或 0
  Eigen::VectorXf ringkey;                // N_r
  Eigen::VectorXf sectorkey;              // N_s
};
struct DbEntry { float x, y, yaw; Descriptor d; };

class PolarRangeDescriptor {              // 纯计算，无 ROS 依赖（可被离线工具复用）
  Descriptor make(const std::vector<float>& ranges /*360 束*/, float angle_min, float angle_inc) const;
  // 旋转不变的描述子距离：对 sector 维做 1..N_s 的全移位取最小
  std::pair<float,int> distance(const Descriptor& q, const Descriptor& c) const;  // (距离, 相对 yaw 档)
};
```

> **注意**：把"描述子数学"与"ROS 节点"**分开成两层**（上层纯 C++/Eigen，下层 rclcpp），
> 是让 §D.2 的离线自测能直接链接同一份代码、从而做到"库与查询同源"的关键。

---

## F. 未能确定项（**不许编**）

以下都是本会话**确实没核实到**的，落地前必须自己确认：

1. **`irapkaist/scancontext` 的现状**：`api.github.com/repos/irapkaist/scancontext` 返回 `Not Found`
   （同类 `irapkaist/SC-LeGO-LOAM` 也是 `Not Found`）⇒ **我无法确认原始仓库是"改名/转移/删除/设为私有"中的哪一种**，
   也无法确认原始仓库的许可证文件内容（只能通过 `gisbi-kim/scancontext_tro` 的 README 转述 CC BY-NC-SA 4.0）。
2. **`aserbremen/scancontext_ros2` 的许可证到底是哪个**：`package.xml` 写 BSD、README 写 CC BY-NC-SA 4.0、
   无 LICENSE 文件、源码无许可证头 ⇒ **无法判定**；这是必须**问作者/法务**才能定的事，本文只能如实并列。
3. **`hku-mars/btc_descriptor` 的输出形式**：README 未写"是否输出 6-DoF 位姿"，
   我只确认了参数清单与 `roslaunch btc_desc place_recognition.launch`；**输出位姿的精确形式未核实**。
4. **BTC 论文的正式出处细节**：只确认了 DOI `10.1109/TRO.2024.3353076`（T-RO）与 ACM DL 条目页；
   **未能打开 IEEE Xplore 正文**，因此论文里的运行时间/描述子尺寸等**量化指标一概未引用**。
5. **STD 论文里的量化指标**（每帧耗时 ms、描述子尺寸、KITTI/NCLT 上的 precision 数值）：
   本会话只取到 arXiv 摘要（`2209.12435`），**摘要里没有这些数字**；ar5iv 全文页只用于定位
   "官方仓库 = `hku-mars/STD`"这一句，**未逐节核对实验数据** ⇒ 不引用。
6. **Contour Context 对本场地的适配性**：`lidar_height_: 2.0`、`roi_radius_: 10.0`、
   `n_row_/n_col_: 150` 这套参数是 KITTI/城市规模；**降到 12×8.6 m 的室内场地后还能不能工作，未验证**。
   另外它落盘的 outcome 文件**不含相对位姿**（要用得自己从 `T_est_delta_2d` 接出来），
   **具体接哪一行代码我没读到**（只读了 `evaluator.h` 的 `addPrediction` 签名与 `pr_mpe.py` 的解析脚本）。
7. **各上游仓库的"最新 commit 哈希"**：本文的"最后提交"一律用 GitHub API 的 `pushed_at`（仓库级），
   只有 `scancontext_ros2` 我另外列了 commit 列表（最新 `56bb561f`，2023-09-21）；其余仓库**未逐仓列 commit**。
8. **`RMUL2026.pcd` 的"墙线总长"**：本文给的是**推断阈值**（"≥ 场地周长一半 ≈ 21 m"），
   **不是实测值** —— 实测要靠 §D.1 的 A1.3 探针。同样，**A1.2 的"墙线是否连成轮廓"我也没有做**（只算了格数）。
9. **`RMUL.pcd` 的字段完整性**：我只核了它的点数/bbox/z 跨度，**没有像 `RMUL2026.pcd` 那样逐字段核 intensity/normal/curvature 是否全 0**。
10. **在线检索节点的实际单帧耗时**：本文只有"很便宜"的**推断**（一次描述子 + 一次穷举/KD-tree 检索），
    **没有实测**；GICP 侧 0.35 s/帧是**已有实测**（§1.1）。
11. **`map→odom` 与"检索粗位姿"的坐标系完全同源这一前提**：本文依赖"FAST-LIO 的世界系 = `map` 系
    （出生点 = 地图原点）"，依据是 `map/RMUL2026.yaml` 的 `origin` 与 PCD bbox 对齐 + 代码里
    `laserCloudWorld` 的语义 + §1.1 的注释。**没有做一次端到端实测来证明这两系之间只有平移/无旋转偏移**；
    若存在固定外参/旋转，§B.3 的 `x0/y0/yaw0` 推导需要补一个 `T_map_world`。
12. **ROS 2 移植的"其他"候选**：我用 GitHub 搜索 API 的多组关键词（`scan context ros2`、
    `place recognition ros2 lidar`、`triangle descriptor place recognition` 等）返回的条目很有限
    （部分查询 `total_count = 0`，疑与 API 速率限制/查询语法有关），
    **不能断言"只有 `aserbremen/scancontext_ros2` 一个"** —— 只能说"我核实到的只有这一个"。
13. **网络限制导致的核实缺口**：`web_fetch` 工具对 github/arxiv 域名不可用（DNS 解析到 198.18.x.x），
    我改用 `curl` + GitHub REST API。**因此凡是需要"读网页正文"才能确认的东西（如 README 里的
    性能数字、issue 里的已知 bug、release notes）都有缺口**；本文所有外部事实都限定在
    "API 字段" 与 "raw 文件内容" 两类可 `curl` 到的证据上。

---

## 附：本文引用到的外部 URL 一览

**论文**
- Scan Context (IROS 2018)：<https://dl.acm.org/doi/abs/10.1109/IROS.2018.8593953>
- Scan Context++ (T-RO 2022)：<https://arxiv.org/abs/2109.13494>
- STD (ICRA 2023)：<https://arxiv.org/abs/2209.12435>
- BTC (T-RO 2024)：<https://dl.acm.org/doi/10.1109/TRO.2024.3353076> ／ 全文 <https://hub.hku.hk/bitstream/10722/346044/1/content.pdf>
- Contour Context (ICRA 2023)：<https://arxiv.org/abs/2302.06149>
- AnyLoc (RA-L 2023)：<https://arxiv.org/abs/2308.00688>

**代码**
- `gisbi-kim/scancontext_tro`（SC++ 代码）：<https://github.com/gisbi-kim/scancontext_tro>
- `RPM-Robotics-Lab/scancontext_tro`：<https://github.com/RPM-Robotics-Lab/scancontext_tro>
- **`aserbremen/scancontext_ros2`（唯一的 SC ROS 2 移植）**：<https://github.com/aserbremen/scancontext_ros2>
- `gisbi-kim/SC-A-LOAM`：<https://github.com/gisbi-kim/SC-A-LOAM> ／ `gisbi-kim/SC-LIO-SAM`：<https://github.com/gisbi-kim/SC-LIO-SAM> ／ `gisbi-kim/scancontext-pybind`：<https://github.com/gisbi-kim/scancontext-pybind>
- **`hku-mars/STD`**：<https://github.com/hku-mars/STD>
- **`hku-mars/btc_descriptor`**：<https://github.com/hku-mars/btc_descriptor> ／ `hku-mars/iBTC`：<https://github.com/hku-mars/iBTC>
- `JixuanLee/MapLoc_Btc_Relocalizer`：<https://github.com/JixuanLee/MapLoc_Btc_Relocalizer>
- **`lewisjiang/contour-context`**：<https://github.com/lewisjiang/contour-context>
- `mikacuy/pointnetvlad`：<https://github.com/mikacuy/pointnetvlad> ／ `jac99/MinkLoc3D`：<https://github.com/jac99/MinkLoc3D> ／ `jac99/MinkLoc3Dv2`：<https://github.com/jac99/MinkLoc3Dv2> ／ `AnyLoc/AnyLoc`：<https://github.com/AnyLoc/AnyLoc>
- nanoflann（SC/STD/Contour Context 都用到）：<https://github.com/jlblancoc/nanoflann>

**本仓库内依据（供自查）**
- `src/rm_nav_bringup/launch/bringup_sim.launch.py`：L60 `nav2_map_dir`、L112 `icp_pcd_dir`、
  L161-167 `localization` declare（**choices 在 L164**）、L268-280 p2l 节点、
  L419-431 `icp` 分支 / L434-452 `gicp` 分支、L458-465 `map_server` 条件
- `src/rm_localization/gicp_registration/src/gicp_registration.cpp`：L353-360 `/initialpose` 订阅（RELIABLE）、
  L717-748 `initialPoseCallback`（语义 `T_map_odom = T_map_base · T_base_odom`）
- `src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml`：两级 leaf 0.10/0.05、
  `max_correspondence_distance 1.5`、`tf_lookahead_sec 0.45`、`fitness_score_warn 0.05`、`max_fitness_score 0.3`
- `src/rm_localization/FAST_LIO/src/laserMapping.cpp`：L520-528 累积 `laserCloudWorld`、L1120-1135 `map_save`、L1175-1181 `save_to_pcd`
- `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml`：`min_height -1.0` / `max_height 0.1` /
  `angle_min -3.14159` / `angle_max 3.14159` / `angle_increment 0.0043` / `range_max 10.0` / `range_min 0.05`
- `src/rm_nav_bringup/map/RMUL2026.yaml`：`resolution 0.05` / `origin [-2.2,-3.15,0]`
- `docs/localization_slots.md` §1.1 / §2 / §3 / §4 / §6 / §7；`docs/algorithm_matrix.md` §九 / §9.1；
  `docs/3d_to_2d_survey.md`；`docs/rm_algorithm_catalog.md` L140 / L145 / L158 / L174；
  `docs/glossary.md` L23；`tools/README.md`（`pcd_to_grid_map.py` / `replay_scan_grid.py` / `analyze_slam_bag.py`）
