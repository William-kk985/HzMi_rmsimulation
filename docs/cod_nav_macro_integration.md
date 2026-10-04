# COD_NAV（RMUL2026 成品版）宏观融入分析：small_point_lio 溯源 + 静态 `map→odom` 深挖 + 能力地图

> **本文定位**：回答两个具体问题（Q1 `small_point_lio` 的出处与 COD 换它的理由；Q2 COD 为什么用「静态 `map→odom`」），
> 并在此基础上给出**宏观能力地图（§C）**与**融入我们架构的宏观视图（§D）**。
> **本文是本次唯一被创建/修改的文件**；全程只读调研，未执行任何上游代码，未改动任何配置。
> **引用约定**：本地路径一律给 `file:line`；外部事实一律给 URL。**FACT（事实）与 INFERENCE（推断）在每一节显式分离**，
> 推断一律带「**推断**」标记与依据；读不到/无法确证的东西集中在 §E。
>
> **版本基准（关键，先说清）**：
> | 对象 | 版本 | 证据 |
> |---|---|---|
> | 本地参考栈 `third_party/cod_nav_2026/` | **Gitee `codnavgation/cod_-rm2026_-navigation` @ `master`，HEAD `0127e200`（2026-04-06）** —— 即 RMUL2026 **赛后成品**，10 个包、190 个文件 | `git -C third_party/cod_nav_2026 log -1` → `0127e2007d16a53343db557beee7524a45cb5d98 2026-04-06 fix: 更新正确的地图文件名`；`git -C third_party/cod_nav_2026 remote -v` → `https://gitee.com/codnavgation/cod_-rm2026_-navigation.git` |
> | GitHub `qza36/COD_NAV` 分支 `rmul2026` | HEAD `485f2ebfa`（2026-03-10）—— **季前**状态，包名是 `nav_bringup` 不是 `cod_bringup` | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100> |
>
> ⚠️ 两者是**同一作者的先后两版**：Gitee 成品把静态桥的 yaw 从 `0.0` 改成 `-0.5`、把 MPPI 从 7.5 m/s 收敛到 2.5 m/s、
> 并新增 `goal_approach_controller`（详见 `docs/cod_nav_nav_stack_inventory.md` §B2）。
> **凡两者不一致处，本文以 Gitee 成品为准并同时标出 GitHub 季前值。**
> 既有文档（`docs/cod_nav_comparison.md`、`docs/cod_nav_2026_deep_dive.md`、`docs/cod_nav_2026_integration_plan.md`、
> `docs/cod_nav_nav_stack_inventory.md`、`docs/cod_nav_2026_params_review.md`）已覆盖的内容，本文**不重复展开**，
> 只做**宏观判定**并指向它们。

---

## A. `small_point_lio` 是什么 + COD 为什么换它

### A0. 任务里给的 BBS 帖（813022）—— 正文提取成功，摘要如下

- **URL**：<https://bbs.robomaster.com/article/813022?source=4>（`?source=4` 与不带参数返回同一份 1,763,733 B 页面）
- **提取方法（FACT）**：正文**不在 SSR HTML 里**，而在页面尾部的 `window.__NUXT__` payload（`<script>` 内联，位于页面偏移 ≈1,713,688 B）中的 `article.htmlContent` 字段；
  它是**转义后的 HTML 串**（`\u003C` 等），需先做 JS 反转义再剥标签。本文提取到正文 **8,142 字符（带标签）/ 约 4,000 汉字（纯文本）**。
- **标题（FACT，取自 `<title>` 与 payload 的 `title` 字段）**：
  「【RM26赛季定位算法开源-东莞理工学院-ACE战队】Small Point-LIO，一个比 Point-LIO 快 2 - 3 倍的里程计」
- **作者（FACT）**：帖内自述「本人大二时负责 25 赛季哨兵导航，目前大三，在 26 赛季同样在做导航相关技术研发」；
  标题署名**东莞理工学院 ACE 战队**；`small_point_lio/package.xml:7,9` 的 maintainer/author 是 **Yingjie Huang `1709185482@qq.com`**，
  `LICENSE.txt` 头行 `Copyright (c) 2025 Yingjie Huang`。⇒ 作者 = ACE 战队的 Yingjie Huang（Yancey）。
- **项目性质（FACT，原文）**：「Small Point-LIO 是 Point-LIO[1] 的**重制版**，以 **MIT 协议开源**。在相同参数下，性能大约**比原代码快 2 - 3 倍**。」
- **仓库 URL（FACT，原文给了两个）**：
  - GitHub：<https://github.com/Yancey2023/small_point_lio>
  - 国内镜像：<https://www.yanceymc.cn/gitea/Yancey/small_point_lio>
  - QQ 群 1070252119。
  - **API 确权（FACT）**：`https://api.github.com/repos/Yancey2023/small_point_lio` → `full_name=Yancey2023/small_point_lio`、
    `license=MIT`、`default_branch=ros2`、`created_at=2025-10-09`、`pushed_at=2026-08-31`、`stargazers_count=121`、`language=C++`。
    ⇒ `docs/cod_nav_2026_deep_dive.md:71` 里「同名公开仓库**很可能是** `Yancey2023/small_point_lio`（推测）」这条**本次已从推断升级为事实**。

#### A0.1 帖子里**逐条说了什么**（原文要点，均为 FACT；「→」后是本文的机制解读，标 INFERENCE）

| # | 帖子原文要点 | 机制解读（INFERENCE） |
|---|---|---|
| 1 | **数据预处理重构**：原 Point-LIO 有 4 种过滤同时进行（`tag` 置信度、`blind`/`det_range` 距离、`point_filter_num` 抽点、`filter_size_surf` 体素），且逻辑**分散在代码各处** | 这是它**逐点（per-point）**处理的直接后果——过滤必须在点进入队列前做完 |
| 2 | 「原代码包含了 IMU 预处理，这部分代码原本是 **Fast LIO 用来做点云去畸变的**，但是 **Point-LIO 是逐点更新的，不需要进行点云去畸变**」 | **它不做 scan de-skew**；去畸变由「每个点各自带时间戳 + 逐点 ESKF 预测」隐式替代。⇒ 与 FAST-LIO 的「一帧内 IMU 积分去畸变 + 帧级 iterated-EKF」是**两条不同路线** |
| 3 | 「原代码缓存了**点云队列**，但 Point-LIO 是逐点更新的，不应该使用点云队列，而应该使用**点队列**」；「接收到点云时就过滤，把点和 IMU 存储在**按时间排序的点队列和 IMU 数据队列**中」 | 时间序由**逐点时间戳队列**保证 ⇒ IMU 与点的融合是「按时间戳归并两个队列」，**对 IMU 频率不敏感**（下面是它的证据引用） |
| 4 | 体素降采样「在 **small_gicp** 的降采样代码基础上修改，使降采样之后的点**保留时间戳属性**，且性能比 PCL 快 1.3 倍」；并**额外保留降采样前的稠密点队列**，在后续流程做去畸变后**发布到话题** | 内部地图用降采样点（省算力），**对外可发去畸变后的稠密点云**——两者是不同的队列 |
| 5 | **地图改进（它自称性能优秀的关键）5 条**：①邻居候选列表**超 5 个时才算距离**（省 40–60%）；②自研哈希 `hash_position_index`（纯位运算，省 30–40%）；③`std::unordered_map` → `ankerl::unordered_dense::map`（省 10–20%）；④每栅格**只存一个点**；⑤LRU 缓存**命中也要更新**以提升稳定性 | 地图是 **iVox 哈希体素**（Faster-LIO 路线），不是 FAST-LIO 的 ikd-Tree。作者自己说「我们一般会把里程计地图分辨率设置在 **0.1 m 以上**」 |
| 6 | **滤波器改进**：原 Point-LIO 「相同时间的点会一起更新……观测矩阵维度不固定」；作者发现 Mid-360 过滤后**同一时刻的点极少**，于是「把相同时间的点**分多次更新**，把观测矩阵维度改为**固定值**」（`Eigen::Dynamic` → 具体数字，省 20–30%）；且「**没有使用 IKFoM 框架**，而是重头实现」 | 逐点更新 + **定维观测**是可读性与速度的双重收益 |
| 7 | **编译选项**：`-march=native`、`-ffast-math`、`-fno-math-errno`；并声明「我之前的性能对比都是**与加了该编译选项的 Point-LIO** 进行对比的」 | 2–3× 的对比基线是**公平的**（不是拿未优化的上游比） |
| 8 | 结语：动机是「Point-LIO 原代码中有很多**不合理的地方**」，要「让大家不再被原代码所折磨」 | 该项目的定位是**工程重写 + 提速**，不是新算法 |

#### A0.2 帖子**没有**说的东西（FACT，用于纠偏）

按关键词在提取出的正文里逐一计数，以下**全部 0 命中或与提问无关**：
`twist`(0)、`save_pcd`(0)、`timestamp`(0)、`map_resolution`(0)、`topic`(0)、`外参`(0)、`重定位`(0)、`回环`(0)、`loop`(0)、`deskew`(0)、`license`(0)。
「发布」只出现 2 次、「话题」1 次（都是第 4 条那句「发布到话题」）、「MIT」1 次。

⇒ 因此：
- **帖子没有列「published topics」清单**——`/Odometry`、`/cloud_registered`、`odom→base_link` TF、`/map_save` 服务这些**只能从仓库源码取证**（见 A2）。
- **帖子没有给地图分辨率默认值**——`map_resolution: 0.5` 只在 `small_point_lio/config/mid360.yaml:25`（见 A2）。
- **帖子没有谈「高速/自旋鲁棒性」**，也没有和 FAST-LIO 做任何对比；唯一对比对象是 **Point-LIO 原代码**，指标是**速度**（2–3×）。

### A1. 与 FAST-LIO / Point-LIO 的路线差异（以本地源码为证，FACT + 推断分离）

| 维度 | FAST-LIO（我们的默认槽 `lio:=fastlio`） | `small_point_lio`（COD 2026） | 证据 |
|---|---|---|---|
| 处理粒度 | **帧级**：一帧点云 + 帧内 IMU 积分 → iterated-EKF 更新一次 | **点级**：`while` 循环里按时间戳在「点队列 / 稠密点队列 / IMU 队列」三路归并，**逐点** `predict_state` + `update_point` | `third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio/small_point_lio.cpp:86-144`（循环与三个分支）；BBS 第 2/3/6 条 |
| 去畸变 | 需要（IMU 预处理 + 帧内积分），FAST-LIO 参数里有 `blind`/`filter_size_surf` 一整套 | **不做**；稠密点队列在逐点流程里被搬到 odom 帧后发话题（对外是「已去畸变」的稠密云） | BBS 第 2 条；`small_point_lio.cpp:90-100`（稠密点搬 odom）、`:101-124` |
| 地图结构 | **ikd-Tree**（增量 k-d 树） | **iVox 哈希体素**（`SmallIVox`，`add_point` 增量插入；`map_resolution`） | `estimator.h:22`（`std::shared_ptr<SmallIVox> ivox`）；`small_point_lio.cpp:51,122`；BBS 第 5 条 |
| 观测方程 | 帧级 scan-to-map 残差 | 逐点找近邻 **5 点**拟合平面取最小特征向量做法向（`NUM_MATCH_POINTS=5`） | `docs/cod_nav_2026_deep_dive.md:72`（对 `estimator.cpp` 的逐行读取） |
| IMU 速率处理 | 有 buffer / 外推参数（`max_iterations`、`filter_size_*` 等） | **无外推参数**：IMU 回调与点云回调**各调一次** `handle_once()`；`publish_odometry_without_downsample: false` 时 odom 在「三队列都非空」的循环尾部发布 ⇒ **≈ IMU 频率** | `small_point_lio_node.cpp:197-211`（两个回调都调 `handle_once`）；`small_point_lio.cpp:146-156`；`config/mid360.yaml:50`；`docs/cod_nav_2026_deep_dive.md:106` |
| 回环 / PGO | **无**（FAST-LIO 本身也没有；我们靠 mapper 槽补） | **无**：`src/` 全目录 grep `loop_clos|pose_graph|pgo|scan_to_map|ikd` **0 命中** | `grep -rn -i "loop_clos\|pose_graph\|pgo\|scan_to_map\|ikd" third_party/cod_nav_2026/src/small_point_lio/src/` → 无输出 |
| 内部是否估速度 | FAST-LIO 的 `/Odometry.twist` 有值 | **内部有**：`common::Odometry` 带 `velocity` 与 `angular_velocity` 字段，`publish_odometry()` 里被填好 | `src/common/common.h:13-19`、`small_point_lio.cpp:169-175` |
| 对外是否发速度 | — | **不发**：ROS 侧 `odometry_msg.twist.twist.*` **五行全被注释**，留 `// TODO it is lidar_odom->lidar_frame, we need to transform it to odom->base_link` ⇒ **twist ≡ 0** | `small_point_lio_node.cpp:90-96` |
| 地图分辨率 | 我们 3D 图 0.1~0.2 m 量级（PCD） | `map_resolution: 0.5`（iVox 体素）；`save_pcd` 时另用 `PointcloudMapping(0.02)` 存 2 cm 全量云 | `config/mid360.yaml:25`；`small_point_lio_node.cpp:32` |
| `save_pcd` | 我们不常用 | **默认能力存在**：`save_pcd: true` 时在线累积；`ros2 service call /map_save std_srvs/srv/Trigger` 落盘 **`ROOT_DIR + "/pcd/scan.pcd"`**，`ROOT_DIR` 由 `include/param_deliver.h.in` 经 `configure_file` 编译期注入源码目录 | `small_point_lio_node.cpp:31-51`、`include/param_deliver.h.in`；`docs/cod_nav_2026_deep_dive.md:107`；Gitee 成品已把 `config/mid360.yaml:7` 改成 `save_pcd: false` |
| 发布/订阅（FACT，源码） | — | **订阅** `/livox/lidar`（PointCloud2，`SensorDataQoS`）、`/livox/imu`（`SensorDataQoS`）；**TF 查询** `lookupTransform(lidar_frame, "base_link")`（查不到就 `return`，**丢帧且不发 TF/odom**）。**发布** `/Odometry`（queue 1000）、`/cloud_registered`（**仅当订阅数 > 0 才发**）、TF `odom→base_link`（**节点自己发**，帧名**硬编码**）、服务 `map_save` | `small_point_lio_node.cpp:26-27,64-69,78-99,101-102`；`src/lidar_adapter/livox_pointcloud2.h:20-47` |
| `tag`/`timestamp` 适配（FACT，**移植硬门槛**） | — | `LivoxPointCloud2Adapter` 用 `PointCloud2ConstIterator<uint8_t>(msg,"tag")` 与 `<double>(msg,"timestamp")`，**只保留 `(*tag & 0b00111111) == 0` 的点**，时间戳**乘 1e-9 转秒**。字段不存在 ⇒ 迭代器构造即失败 | `src/lidar_adapter/livox_pointcloud2.h:25-37` |

### A2. 上游仓库与许可证（FACT）

| 项 | 值 | 证据 |
|---|---|---|
| COD 分支里的形态 | **vendored 目录**（含自己的 `.github/workflows/`、`.clang-format`、`.clang-tidy`、`3rdparty/`），**不是 submodule** | `third_party/cod_nav_2026/src/small_point_lio/`（42 个文件）；仓库根无 `.gitmodules` |
| 上游仓库 | <https://github.com/Yancey2023/small_point_lio>（MIT，默认分支 `ros2`，121★ / 22 fork，2025-10-09 建、2026-08-31 最后 push） | `https://api.github.com/repos/Yancey2023/small_point_lio` |
| 上游 README 关键句 | 「delivering a **2-3x speed improvement** over the original」「The default branch is for ROS2. If you want to run it without ros, please checkout the `main` branch」「If you want to know why it so fast, please read [this]」(链到 BBS 813022) | `third_party/cod_nav_2026/src/small_point_lio/README.md:3,5,7` |
| 许可证（上游） | **MIT**，`Copyright (c) 2025 Yingjie Huang` | <https://github.com/Yancey2023/small_point_lio> 的 `license.spdx_id = MIT`；`third_party/cod_nav_2026/src/small_point_lio/LICENSE.txt:1-3`；`package.xml:8` `<license>MIT</license>` |
| 第三方致谢 | Eigen(MPL2.0) / `ankerl::unordered_dense`(MIT) / `small_gicp`(MIT) / Open3D(MIT) | `README.md:90-95` |
| 注意（FACT） | **`small_gicp` 只在 CI 里被 apt/源码安装，`CMakeLists.txt` 并不 `find_package(small_gicp)`** ⇒ 运行期**不是依赖**；那份体素降采样是**抄进来的代码**（`src/util/voxelgrid_sampling.*` 文件头写明来自 koide3/small_gicp） | `docs/cod_nav_2026_deep_dive.md:72,325,494` |
| 拿到的另一种实现 | 上游仓库 README 另行给出「不用 ROS 请 checkout `main`」⇒ 上游按 ROS/非 ROS 分两支 | `small_point_lio/README.md:5` |

### A3. COD 为什么换掉 FAST-LIO —— FACT / INFERENCE 分离

**先立时间线（FACT，全部来自提交史 API）**：

| 时间 | commit | 消息 | 说明 |
|---|---|---|---|
| 2025-08-02 | `54dc95af4` | Merge PR #2 from `dyx-dev` | `master`（2025 赛季线）在此冻结；**master 用 FAST-LIO v2 fork + patchwork++ + small_gicp 重定位**（见 `docs/cod_nav_comparison.md:11`） |
| 2025-11-18 | **`9d54b5ab8`** | **`feat:replace fastlio2`** | **第一次删除 FAST-LIO**（删 `FAST_LIO/`），**同一批没有加入新 LIO** |
| 2025-11-18 | `093d13a66` | `feat:lio interface pub odom2base_link` | 让 LIO 接口发布 `odom→base_link`（即「LIO 自己发 TF」这个契约） |
| 2025-11-18 | `ffc4e24e3` | `feat:rmul2026 slam navgation param` | slam_toolbox 参数（含 `mode: lifelong`） |
| 2025-12-19 | `9856cff66` | `remove livox driver` | 继续做减法 |
| 2025-12-19 | `9dfbeabdd` / `6e03b83e0` | `feat:use stvl instead of voxel_layer` / `chore:add stvl to global map` | 双图 STVL |
| 2025-12-28 | **`d1c027958`** | **`add small point lio and ferfect mppi`** | **真正落地 `small_point_lio`（同时引入 MPPI）** |
| 2026-03-04 | `632fbb4eb` | `稳定在先验地图下到达增益点` | 砍掉全部重定位（删 `bringup.launch.py`、`nav2_params_amcl.yaml`、carto lua/pbstream），加 `nav.launch.py`(GitHub)/`singlenav_launch.py`(Gitee) |
| 2026-03-10 | `a7299b60a` → `485f2ebfa` | MPPI 落地 + 消震荡三轮 | GitHub `rmul2026` HEAD |
| 2026-04-06 | `0127e200` | `fix: 更新正确的地图文件名` | **Gitee 成品 HEAD（本地 `third_party` 快照）** |

来源：<https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>；
Gitee：`https://gitee.com/api/v5/repos/codnavgation/cod_-rm2026_-navigation/commits`（本地 `git log` 亦得 `0127e200`）。

#### A3.1 FACT（能直接引用到证据的）

| # | FACT | 证据 |
|---|---|---|
| F1 | COD 的 `master`（2025）确实用 FAST-LIO fork；`rmul2026` 与 Gitee 成品都用 `small_point_lio`；Gitee 成品 10 个包里**没有** `FAST_LIO` | `docs/cod_nav_comparison.md:11-12`；`docs/cod_nav_2026_integration_plan.md:150`；`ls third_party/cod_nav_2026/src/` |
| F2 | 换 LIO 的**提交消息只写了 `replace fastlio2`**，没有任何理由描述；落地是 40 天后另一个 commit | `9d54b5ab8`、`d1c027958`（同上 API） |
| F3 | 仓库内**唯一**解释 `small_point_lio` 价值的文字是上游 README 的「**2-3x speed improvement**」，以及 BBS 帖标题「比 Point-LIO 快 2-3 倍的里程计」 | `small_point_lio/README.md:3`；BBS 813022 标题 |
| F4 | COD 的 `README.md` 把该包的作用写成 **`small_point_lio  #point_lio提供里程计，odom->base_link`** —— 定位是「**里程计源**」，不是「建图/定位一体」 | `third_party/cod_nav_2026/README.md`（仓库结构段） |
| F5 | 上游的收益口径**只有速度**，且基线是**同样开了 `-march=native` 的 Point-LIO**；帖子里**没有任何**「鲁棒性/去畸变/高动态/自旋」的对比 | BBS 813022 第 7 条 + §A0.2 的 0 命中统计 |
| F6 | `small_point_lio` **不做去畸变**（作者把 IMU 预处理称为「原本是 Fast LIO 用来做点云去畸变的……Point-LIO 逐点更新不需要」） | BBS 813022 第 2 条 |
| F7 | `small_point_lio` **没有回环/PGO**；COD 用 slam_toolbox 在 2D 侧补闭环 | `grep` 0 命中（A1 表）；`cod_bringup/launch/multiplenav_launch.py:85-94`（启动 `async_slam_toolbox_node`） |
| F8 | COD 把 LIO 的点云**降维后**才用于导航：`pointcloud_to_laserscan` 把 `/livox/lidar` 变成 `/scan`（`range 0.5~20.0`、`angle ±3.1416`、`increment 0.0087`），3D 云只经裁剪盒喂 STVL | `cod_bringup/launch/multiplenav_launch.py:65-84`；`singlenav2_params.yaml:221`（`topic: /livox/lidar_filtered`） |
| F9 | 他们的 LIO 消费的是 `livox_ros_driver2` 的 **PointCloud2（含 `tag`/`timestamp` 字段）**，这正是 Livox 官方驱动原生就带的字段 | `config/mid360.yaml:5` `lidar_type: livox_pointcloud2`；`livox_pointcloud2.h:28-29`；COD README 要求先装 <https://github.com/Livox-SDK/livox_ros_driver2.git> |
| F10 | `package.xml:22` 声明 `<depend>livox_ros_driver2</depend>`，但 `LIVOX_DRIVER` 只在 `lidar_type == "livox_custom_msg"` 时才需要（编译期 `HAVE_LIVOX_DRIVER`，找不到只 warning） | `small_point_lio/package.xml:22`；`small_point_lio_node.cpp:178-187`；`docs/cod_nav_2026_deep_dive.md:74` |

#### A3.2 INFERENCE（我的判断，逐条给依据与反证边界）

| # | 推断 | 依据 | 反证/边界 |
|---|---|---|---|
| I1 | **主因是「算力/帧率」而非「算法质量」**：7.5 m/s（GitHub 季前）到 2.5 m/s（Gitee 成品）的高速哨兵 + 机载算力，需要更省 CPU 的 LIO；2–3× 的**同基线**提速直接换算成可用的点云/IMU 频率 | F3 + F5 + `docs/cod_nav_2026_deep_dive.md:336`（B1 条同样推断） | **不能反证**：仓库里没有任何 CPU 占用/bag/基准数据；此推断只有文本证据 |
| I2 | **次因是「维护面收敛」**：2026 是一次整体做减法（删 submodule、删 livox driver、删地面分割、删重定位），换一个**自带完整工程配置**（PCH/OpenMP/CI/`.clang-format`）的包符合同一策略 | A3 时间线；`docs/cod_nav_2026_integration_plan.md:151` | 无直接文档，属合理外推 |
| I3 | **不是**因为在「高动态/自旋」下 Point-LIO 族更强而选的：他们**从不自转**（`spin_speed` 全分支无一处非零），且帖子不谈鲁棒性 | §A0.2 0 命中；`docs/cod_nav_2026_deep_dive.md:292`、`docs/cod_nav_comparison.md:60` | — |
| I4 | **不是**因为「需要 PCD 先验/重定位」而选的：`save_pcd` 在 Gitee 成品里被改成 `false`，且整个 2026 路线**没有重定位** | `config/mid360.yaml:7`；`cod_bringup/launch/localization_launch.py:44`（`lifecycle_nodes = ['map_server']`，无 amcl） | — |
| I5 | **不是因为去畸变质量**：它明确不做 de-skew，靠逐点时间戳隐式补偿；在 7.5 m/s + 100 Hz IMU 下这恰好够用（每点都有 IMU 预测），但在**帧率低/IMU 稀疏**时会退化 | F6 + `small_point_lio.cpp:86-144` | 属机理解读，**未实测** |
| I6 | **`livox_pointcloud2` 适配器的字段要求在他们那里不是成本**（官方驱动本来就发 `tag`/`timestamp`），但**对我们仿真是硬门槛** | F9 vs 我们 `/livox/lidar` 是 CustomMsg、`/livox/lidar/pointcloud` 是普通 PointCloud2（`docs/debug_fastlio_cartographer.md` §10.0 雷达话题行） | 见 §A4 |

### A4. 结论：我们要不要 vendor `small_point_lio`？

| 判定 | 内容 | 依据 |
|---|---|---|
| **不能直接融（硬门槛）** | `LivoxPointCloud2Adapter` 要求 `tag`(uint8) + `timestamp`(float64) 字段，否则迭代器构造失败；我们仿真 `/livox/lidar` 是 **CustomMsg**、`/livox/lidar/pointcloud` 是**普通 PointCloud2**（无这两个字段） | `livox_pointcloud2.h:25-37`；`docs/debug_fastlio_cartographer.md` §10.0 |
| **不必融（同族已存在）** | 我们已有 `lio:=pointlio`——`src/rm_localization/point_lio/`（仓库内已 checkout 的可编译源码，含 `CMakeLists.txt`/`config/`），其上游在 `.gitmodules` 里登记为 `https://github.com/William-kk985/Point-LIO.git`（即 hku-mars Point-LIO 的自有 fork）。`small_point_lio` 是**同一族**（Point-LIO）的工程提速版，**算法结论不会不同**，只是 CPU 开销不同 | `docs/algorithm_axes.md:48-54`；`.gitmodules`（`[submodule "src/rm_localization/point_lio"]`）；`ls src/rm_localization/point_lio/` |
| **若真要融，三条路** | ① 编译期装 `livox_ros_driver2` 走 `livox_custom_msg` 适配器（`-DHAVE_LIVOX_DRIVER`）；② 给仿真点云**补 `tag`/`timestamp` 字段**（自研小节点，把 CustomMsg 转成带这两字段的 PointCloud2——**这与我们已修好的 CustomMsg 链路直接冲突，不建议**）；③ 只当参考实现读 | `small_point_lio_node.cpp:178-187`；`CMakeLists.txt`（`find_package(livox_ros_driver2 QUIET)`） |
| **不该融的部分** | `save_pcd` 的落盘路径 = `ROOT_DIR + "/pcd/scan.pcd"` 且 `ROOT_DIR` 编译期写死源码目录；`map_resolution: 0.5` 比我们 0.05 粗 10 倍；**`twist ≡ 0`**（任何依赖 `/Odometry.twist` 的模块会读到 0） | `small_point_lio_node.cpp:31-51,90-96`；`config/mid360.yaml:25` |

**一句话（A 部分）**：`small_point_lio` 是东莞理工 ACE 战队 Yingjie Huang 对 Point-LIO 的 **MIT 工程重写**（逐点 ESKF + iVox + 定点/定维观测 + 编译期加速，官方口径**快 2–3×**，且基线是同编译选项的原版）。
COD 2026 换它，**有据可查的只有「速度」这一条**（README/帖标题）；「算力优先 + 做减法」是合理推断；「高动态鲁棒性/去畸变更好」**被证据反证**（它不做去畸变，帖子也不谈鲁棒性）。
对我们：**结论不变——同族用 `lio:=pointlio` 就够，不必 vendor**（与 `docs/cod_nav_2026_integration_plan.md:172` 一致，本文补上了 BBS 原文与上游确权两处证据）。

---

## B. 核心问题：COD 为什么用「静态 `map→odom`」

### B1. 证据清单（全部 `file:line` / URL，FACT）

#### B1.1 「谁在发 `map→odom`」——答案是**唯一一个静态发布器**

| # | 事实 | 证据（本地 Gitee 成品） | 证据（GitHub 季前，作对照） |
|---|---|---|---|
| E1 | **单点导航**：唯一发布者是 `tf2_ros static_transform_publisher`，节点名 `map_to_odom`，参数 `--x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw **-0.5** --frame-id map --child-frame-id odom` | `third_party/cod_nav_2026/src/cod_bringup/launch/singlenav_launch.py:67-89`（yaw 在 `:82-83`） | `nav_bringup/launch/nav.launch.py` 同款（<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/nav.launch.py>） |
| E2 | **多点导航（在线 SLAM）**：唯一发布者是同一个静态节点，但 `--yaw **0.0**`（z 仍 0.05） | `cod_bringup/launch/multiplenav_launch.py:95-116`（yaw 在 `:109-110`） | `nav_bringup/launch/slam.launch.py:101-120`（**yaw 0.0**，<https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py>） |
| E3 | 全仓只有 **3 处** `static_transform_publisher`：上面的 singlenav、multiplenav，以及 `small_point_lio/launch/small_point_lio.launch.py:24-45` 的 `base_link→livox_frame`（**而该 launch 从未被 cod_bringup 引用**，各 launch 是直接起 `small_point_lio_node`） | `grep -rn "static_transform_publisher" third_party/cod_nav_2026/src/`（3 命中） | `docs/cod_nav_2026_deep_dive.md:70`（该分支无 `.gitmodules`） |
| E4 | `slam_toolbox` **结构上不可能**发 `map→odom`：`publishTransformLoop()` 第一行就是 `if (transform_publish_period == 0) { return; }`；而他们的参数正是 **`transform_publish_period: 0.0`** | `src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:251-259`（这是**本机同版本源码**，与我们/他们都在用的 slam_toolbox 一致）；他们的参数 `cod_bringup/params/mapper_params_online_async.yaml:30` | `nav_bringup/params/mapper_params_async.yaml` 同值（`docs/cod_nav_2026_deep_dive.md:119`） |
| E5 | **没有 AMCL**：定位 launch 的 `lifecycle_nodes = ['map_server']`；`singlenav2_params.yaml` 从第 1 行 `bt_navigator:` 开始，**全文件无 `amcl:` 段** | `cod_bringup/launch/localization_launch.py:44`；`cod_bringup/params/singlenav2_params.yaml:1`（且 `grep -i amcl` 两套 params 均 0 命中） | `docs/cod_nav_2026_deep_dive.md:116-117`（对 GitHub 版同结论）；`632fbb4eb` 删除了 `nav2_params_amcl.yaml`、`carto.config.lua`、`map/*.pbstream` |
| E6 | **launch 里没有任何初始位姿注入**：`singlenav_launch.py` 全文没有 `initialpose`、没有 `amcl_initial_pose`、没有 `map_start_pose` | `cod_bringup/launch/singlenav_launch.py`（154 行全文） | `docs/cod_nav_2026_deep_dive.md:123`（「`nav.launch.py` 并没有任何初始化位姿的东西」） |
| E7 | `localization_launch.py` 默认地图 = **`cod_bringup/maps/rmul2026.yaml`**（Gitee 成品）；`singlenav2_params.yaml:360` 里的 `map_server.yaml_filename` 是一条**写死作者机器路径**的死值（`/home/cod-sentry/dyx_ws/...`），会被 launch 的 `map` 参数覆写 | `cod_bringup/launch/localization_launch.py:76-79`（default_value）与 `:56-58`（`param_substitutions` 把 `yaml_filename` 换成 `map_yaml_file`）；`singlenav2_params.yaml:355-360` | GitHub 版写死 `/home/cod-sentry/qza_ws/...`（`docs/cod_nav_2026_deep_dive.md:121`） |

#### B1.2 先验地图 / 在线建图的参数（FACT）

| 参数 | 值 | 证据 |
|---|---|---|
| 先验图 `image` / `resolution` / `origin` | `rmul2026.pgm` / **0.05** / **`[-1.756, -7.036, 0]`**（yaw=0） | `third_party/cod_nav_2026/src/cod_bringup/maps/rmul2026.yaml:1-7` |
| 先验图栅格尺寸 | PGM header `P5 255 177 255` ⇒ **12.75 m × 8.85 m** | `python3` 解析 `cod_bringup/maps/rmul2026.pgm` 头（本文实测）；`docs/cod_nav_nav_stack_inventory.md:132`（255×177 ⇒ ≈12.75×8.85 m） |
| `slam_toolbox` 模式 | `mode: lifelong`（注释里还留着 `#localization`） | `cod_bringup/params/mapper_params_online_async.yaml:19` |
| `slam_toolbox` 更新节流 | `map_update_interval: 1.0`、`minimum_time_interval: 0.3`、`minimum_travel_heading: 0.1`、`max_laser_range: 10.0`、`min_laser_range: 0.2`、`resolution: 0.05` | 同上 `:31-34`（GitHub 季前是 `2.5 / 0.5 / 0.5 / 5.0 / 0.01`，见 `docs/cod_nav_2026_deep_dive.md:133`） |
| `map_start_pose` | **不存在**（被注释） | `cod_bringup/params/mapper_params_online_async.yaml:24-26`（`# map_start_pose: [0.0, 0.0, 0.0]`） |
| slam_toolbox 的 `base_frame` | `base_link`（**不是** `base_link_fake`） | 同上 `:16` |
| Nav2 的 `robot_base_frame` | `base_link_fake`（**全部** nav2 节点：`bt_navigator`、两张 costmap、smoother、behavior） | `singlenav2_params.yaml:5`（bt_navigator）、`:184`（local）、`:274`（global）、`:411`（smoother）、`:443`（behavior） |
| Nav2 的 odom 话题 | bt_navigator `odom_topic: Odometry`；velocity_smoother `odom_topic: "Odometry"`（Gitee 成品已修好拼写，GitHub 季前是 `"odomety"`） | `singlenav2_params.yaml:6,482`；`docs/cod_nav_2026_deep_dive.md:348` |
| 全局 planning frame | `bt_navigator.global_frame: map`；global_costmap `global_frame: map` + **`rolling_window: true`**（25×25 m @0.04） | `singlenav2_params.yaml:4,273,269-279` |
| 局部参考系 | local_costmap `global_frame: odom` + `rolling_window: true`（10×10 @0.05） | `singlenav2_params.yaml:183,186-189` |

#### B1.3 作者本人的原话（**这是 Q2 最直接的证据**）

BBS 帖 **1882897**《【RM2026-哨兵机器人导航、决策、自瞄、下位机控制全开源】辽宁科技大学-COD战队》
（<https://bbs.robomaster.com/article/1882897>，同样从 `window.__NUXT__` 的 `htmlContent` 提取，正文 18,304 字符（带标签）/ ≈6,079 汉字）「建图」段原文（FACT，逐字）：

> 「在没有先验 pgm 的情况下进行导航，nav2 官方的方案是边 slam 边导航：slamtoolbox + nav2
> 只要提供了 map->odom->baselink 的 tf，以及 /map 上有地图数据，nav2 就可以正常运行
> 所以：small pointlio 提供 odom->baselink 以及高频里程计，slamtoolbox 提供 /map 数据，**map 到 odom 静态发布 ‘0’,‘0’,'0'**;
> **由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了纯 lio 的定位方法**」

同一帖还有两句对本次分析很关键的话（FACT）：
> 「NOTE：一定要注意的是，不能一味去调上位机的速度，先要确保电控那边能正确把控制器的速度转换成轮速」
> 「这是我修改过的 small point lio，**可以输出速度**，感谢 ACE 战队的开源，真的非常鲁棒！」

⇒ **三点直接结论**：
1. 作者说的静态值是 **`0,0,0`**（帖子里写的是「地图到 odom」的三个平移量），与 `multiplenav_launch.py:99-110`（x=y=0、yaw=0）一致；
   而 `singlenav_launch.py:82-83` 的 **`yaw -0.5`** 是**成品里改出来的、帖子里没提的**一个额外偏航。
2. **作者明确给出了动机**：「环境简单 + 面积小 + 定位误差容忍度大」⇒ 直接回答了「为什么敢不用重定位」。
3. 作者说他们「**修改过的 small point lio 可以输出速度**」——这句话必须与源码对照：**COD 仓库里的 `small_point_lio_node.cpp:90-96` 仍然是注释掉的 twist**。
   ⇒ **FACT：COD 公开仓库里发布出来的 `/Odometry.twist` 恒为 0**；作者所说的「能输出速度」要么是**仓库外的改动**、要么是**指内部速度被用于其它目的**、要么是**RViz 里画的是内部速度**。
   **本文只能确证「公开快照的 twist 为 0」**，其真机是否另有版本**无法确定**（记入 §E）。

### B2. 动机假设分析：逐条判「支持 / 推测 / 被反证」

> 判定口径：**SUPPORTED** = 有作者原话或可核对的机制证据；**PLAUSIBLE** = 与证据一致但无直接证据；**CONTRADICTED** = 有反证。

| # | 候选原因 | 判定 | 证据与理由 |
|---|---|---|---|
| **(a)** | **纯在线 SLAM、无先验地图 ⇒ `map` 原点天然 == 启动位姿，静态桥「不花任何代价」** | **SUPPORTED** | ①`multiplenav_launch.py` 里没有 `map_server`（它只 include `navigation_launch.py`，`:138-147`），也没有 `localization_launch.py` ⇒ 在线模式下 `/map` 的唯一来源是 slam_toolbox（`:85-94`）；②静态桥 yaw=0、x=y=0（`:99-110`）⇒ map≡odom；③slam_toolbox 的首帧位姿在 karto 里就是原点。**这是 (a) 的完整闭环**，且作者原话「在没有先验 pgm 的情况下……只要提供了 map->odom->baselink 的 tf …… 静态发布 0,0,0」正是这个意思 |
| **(b)** | **避免 slam_toolbox 回环/位姿图修正带来的 `map→odom` 跳变扯动 global costmap / MPPI / 自转底盘** | **CONTRADICTED（对「避免位姿跳变」这半边）+ 部分 SUPPORTED（对「不希望 SLAM 管位姿」这半边）** | **被反证的部分**：`transform_publish_period: 0.0`（`mapper_params_online_async.yaml:30`）+ `slam_toolbox_common.cpp:255-257` ⇒ slam_toolbox **根本不发** `map→odom`，所以**不存在「SLAM 的位姿跳变」这个现象可被避免**。真正跳变的是 **`/map` 栅格本身**（`static_layer` 订阅 `/map`，`map_update_interval: 1.0` ⇒ 每秒可能重排一次栅格），而这条**静态桥并没有解决**。<br>**支持的部分**：把 `transform_publish_period` 设为 0 意味着「让 SLAM 的修正**只体现为地图变化、不体现为位姿变化**」——这确实是一种「不让 SLAM 动 TF」的设计，但它省掉的是**位姿连续性**而不是**图一致性**。<br>**另一条被反证的子假设**：他们「为了自转底盘不跳」——他们**从不自转**（`spin_speed` 全仓无一处非零，见 `docs/cod_nav_comparison.md:60`、`docs/cod_nav_2026_deep_dive.md:292-293`），所以「避免扯动自转底盘」这个动机**不成立**（底盘自转是下游 `cod_serial_ul26` 的事，与 TF 无关） |
| **(c)** | **主动砍掉 AMCL/重定位，以消掉一整类失败源** | **SUPPORTED（动机）+ SUPPORTED（事实）** | 作者原话「由于联盟赛的环境较为简单且面积较小，定位的误差容忍度很大，所以采用了**纯 lio 的定位方法**」；`632fbb4eb`「稳定在先验地图下到达增益点」同批删掉 `bringup.launch.py` / `nav2_params_amcl.yaml` / carto lua / `*.pbstream`（`docs/cod_nav_2026_deep_dive.md:118`）；`localization_launch.py:44` 只留 `map_server`。<br>⇒ **动机是「容忍度大 + 不值得调」**（**推断**：AMCL/ICP 的调参时间成本 > 收益），而**不是**「他们试过 AMCL 失败」——后者无证据 |
| **(d)** | **简单/人手/赛季带宽** | **SUPPORTED（README 自述）+ 部分为推断** | README 原文：「此导航程序的设计原理保持 **Keep It Simple Stupid** 的原则，**发布坐标系静态转换比运用 urdf 维护更简单易操作**」（`third_party/cod_nav_2026/README.md`「注意」段）。这解释了**为什么用静态 TF 而不是 URDF/动态 TF**，但**没有**说明为什么不要重定位；「人手/带宽」属**推断**（`docs/cod_nav_2026_deep_dive.md:337-338` 亦为推断） |
| **(e)** | **配置气味 / 潜在冲突（slam_toolbox 本来也会发 `map→odom`，静态桥是「抢发布者」）** | **部分成立，但与被反证的那半边不同** | **FACT**：slam_toolbox 的 `transform_publish_period: 0.0` **正是**为了「不发」，所以**没有双发布者冲突**——这一点他们做对了（对比我们自己的 `docs/tf_interface_contract.md` §二 P1：我们曾经真的踩过「静态桥 + AMCL」多父边）。<br>**但**：①`singlenav_launch.py:83` 的 `yaw -0.5` 是一个**没有任何注释/文档解释的魔数**；②`multiplenav_launch.py` 同时启动 slam_toolbox（`:85-94`）**和**静态桥（`:95-116`），一旦有人把 `transform_publish_period` 改成非 0 就**立刻**变成双发布者；③`singlenav2_params.yaml:360` 的 `map_server.yaml_filename` 写死作者机器路径 ⇒ **复制配置到新机器会静默加载失败（或加载错图）**；④`localization_launch.py:76-79` 的默认图是 `maps/rmul2026.yaml`，**但它在多点模式下从不被启动** ⇒「先验图」这条路在成品里只服务单点模式。<br>⇒ 判定：**「静态桥」本身不是气味；但「静态桥 + 无注释魔数 + 写死路径 + 依赖 0.0 这一个参数维持唯一发布者」的组合是脆弱配置** |

### B3. 后果与风险（静态 `map→odom` 的代价，FACT 主导 + 明确标推断）

| # | 后果 | 机制（FACT） | 对我们的具体含义（INFERENCE，标出） |
|---|---|---|---|
| R1 | **`map` 系完全继承 LIO 的长期漂移；SLAM 的回环修正无法回头修正机器人位姿 ⇒ 漂移变成「地图畸变」** | 静态桥 ⇒ 机器人 map 位姿 ≡ LIO odom 位姿（`singlenav_launch.py:67-89` + `small_point_lio_node.cpp:78-99`）；slam_toolbox 的位姿图优化改了 `map_to_odom_`，但它**不发那条 TF**（`slam_toolbox_common.cpp:255-257`） | ⇒ 长距离跑动后，**方向/距离误差只能靠 `/map` 栅格不断重排来「吸收」**，表现为「地图在机器人脚下平移/旋转」。**推断**：这是他们能接受的原因之一——联盟赛场地小、单局时间短 |
| R2 | **机器人必须从「先验图的位姿/朝向起点」出发**（单点模式）；否则整张图在 map 系里整体错位 | `singlenav_launch.py` 无任何 `initialpose`/`map_start_pose`（E6）；静态桥固定 `x=y=0, z=0.05, yaw=-0.5`（`:72-83`）；先验图 `origin [-1.756, -7.036, 0]`（`maps/rmul2026.yaml:4`） | ⇒ **对我们的先验图同样成立**：我们 `src/rm_nav_bringup/map/RMUL2026.yaml:4` 的 `origin` 是 **`[-2.2, -3.15, 0]`**，PGM 是 **249×173**（12.45 m × 8.65 m，本文实测）。若要用静态桥，机器人**必须**在「采集该图时的起点 + 朝向」启动，且 odom 的 yaw 零点要对齐 |
| R3 | **`yaw -0.5` 与地图 `origin` 的关系无法从仓库自证** | `maps/rmul2026.yaml:4` 的 `origin` 第三项是 **0**（原点处朝向 = map 系 +x）；静态桥给 `odom→map` 之前先转 **-0.5 rad（≈-28.6°）**（`singlenav_launch.py:82-83`，全仓无注释） | **推断**：该 -0.5 是**手工标定常量**，用于补偿「先验图并非从 `origin` 处 yaw=0 起飞」或「场地摆放朝向」；因为**成品是单点模式**（用先验图），而**作者帖子只提 0,0,0**，二者矛盾只能在真机标定中解释。**我们无法确定其物理含义**（记入 §E）。<br>⚠️ 另注：GitHub 季前的 `t1.yaml` origin 是 `[-3.84, -6.05, 0]` 而 yaw 也是 0（`docs/cod_nav_2026_deep_dive.md:120-121`）⇒ **换了地图却没改 yaw=0**；到成品换 `rmul2026.yaml` 时才变成 -0.5。这条**支持「-0.5 是跟图走的标定值」**的推断 |
| R4 | **Nav2 目标语义被绑死在「图的原点系」**：`/goal_pose` 的坐标是 map 坐标，而 map 与 odom 同源 ⇒ 目标点实际上是**相对启动点**的坐标 | `bt_navigator.global_frame: map`（`singlenav2_params.yaml:4`）；global_costmap `global_frame: map`（`:273`）；航点 CSV 存的是 map 坐标（`cod_bringup/wps/*.csv`，6 组） | ⇒ 先验图一旦换（场地/赛季/重建图），**所有 CSV 航点全部失效**（必须重打）。我们的 `waypoint_through_nav2` 若走同路线，也要接受同一约束 |
| R5 | **「机器人被搬走 / 被抱回起点 / 比赛脚本传送」后必须重启栈** | LIO 的 odom 是增量积分，没有绝对观测（无 AMCL、无 ICP、无重定位）；静态桥不会自动纠正 | ⇒ 我们的仿真里 `gz model -m ... -x ...` 传送、或 `nav_smoke_regression.py` 重置仿真后**必须重启 bringup**；实车「被撞大位移」后同样无法自恢复 |
| R6 | **换图 = 换定位基准；`map_server` 的 yaml 路径写死会静默加载旧图** | `singlenav2_params.yaml:360`（`/home/cod-sentry/dyx_ws/.../rmul2026.yaml`）；`localization_launch.py:56-58,76-79`（launch 的 `map` 参数会覆写它，但**只有在走 `localization_launch.py` 时**） | ⇒ 多点模式**根本不起 `map_server`** ⇒ 静态层无图可用，全局 costmap 的 `static_layer` 只能吃 `track_unknown_space: true` 的未知区（`singlenav2_params.yaml:280`；`always_send_full_costmap: False`）。**推断**：这解释了为什么他们要多点模式**放大全局滚动窗到 50×50**（`docs/cod_nav_nav_stack_inventory.md:167`） |
| R7 | **「位姿连续」被换来的是「图不连续」**：`static_layer` 吃 `/map`，而 `/map` 每秒可能被重排（`map_update_interval: 1.0`） | `singlenav2_params.yaml:282-285`（`static_layer` + `map_subscribe_transient_local: True`）；`mapper_params_online_async.yaml:31` | ⇒ **推断**：这是他们真正面对的现象——**机器人不抖，图会「跳」**。对高速 MPPI 而言，「图跳」的影响小于「TF 跳」（TF 跳会让 `getRobotPose()` 出现大位移，而他们恰好把这点彻底删掉了） |
| R8 | **`z=0.05` 的静态抬升是 3D 遗留，2D 下无害但也无用** | `singlenav_launch.py:76-78`、`multiplenav_launch.py:103-105` 都是 `--z 0.05` | ⇒ 若我们要用，**必须写 0**（我们 2D costmap 的 `robot_base_frame: base_link_fake` 与 odom 的 z 关系由 `lio_tf_adapter` 全程负责，见 `docs/tf_interface_contract.md` §三）。**推断**：他们保留 0.05 是因为 LIO 的 z 起点/雷达安装高度 |
| R9 | **`spin_speed == 0` 让 MPPI 的 `wz` 被丢弃**（与本题间接相关，但属同一套「减法」） | `fake_vel_transform.cpp` 里 `aft_tf_vel.angular.z = spin_speed_`（默认 0.0，全仓无一处非零）；`singlenav2_params.yaml:101,105` 仍声明 `wz_max 1.5 / wz_min -1.5` | ⇒ **守恒性检查**：他们「只控位置」的自洽性依赖 `goal_checker` 不管朝向——Gitee 成品单点用 **`PositionGoalChecker`**（`xy 0.2`，**无 yaw 键**，`singlenav2_params.yaml:79-83`），多点用 `SimpleGoalChecker` + **`yaw_goal_tolerance: 6.28`**（`multiplenav2_params.yaml:77-79`）。**我们若采用，必须同批改容差**，否则永远判不到「到点」 |

### B4. **对我们是否适用** —— 逐条结论

> 我们的现状基线（FACT，`file:line`）：
> - `map→odom` 的发布者：**AMCL / slam_toolbox(localization) / icp_registration** 三选一，或 **cartographer**（纯定位或全包形态）；
>   唯一允许静态桥的情形是 **`mode:=nav` + `localization:=''` + `lio∈{fastlio,pointlio,none}`**（即「拿 LIO 当绝对定位」的回退用法）。
>   证据：`src/rm_nav_bringup/launch/bringup_sim.launch.py:444-468`（`icp_frame_bridge_condition` + `tf_bridge_node`/`tf_bridge_node2` 两条 `camera_init→map`、`body→odom` 静态桥）；
>   同文件 `:445` 的注释原文：「**amcl / slam_toolbox / icp_registration 三者都会自行发布 map→odom，绝不能再叠加静态桥（否则 map/odom 多父边）**」。
> - `docs/tf_interface_contract.md:17-19`：slam_toolbox / amcl / icp_registration 各发 `map→odom`；
>   `:44` 目标契约「`map ──(重定位：AMCL / slam_toolbox / ICP 之一发布)──► odom ──(里程计：LIO 发布)──► base_link`」。
> - `mode` 三态：`mapping` / `slam_nav` / `nav`（`bringup_sim.launch.py:124-131`）；`slam_nav` 的语义是「在线 SLAM 直接把 `/map` 与 `map→odom` 喂给 costmap；不加载磁盘地图、不起任何重定位模块」（`:130-131`）。

| 子问题 | 结论 | 证据/推理 |
|---|---|---|
| **(i) 我们哪些模式可以安全地用静态桥？** | **只有两个「本来就是静态」的角落**：<br>①`mode:=nav` + `localization:=''` + `lio≠cartographer`——**代码里已经这么做了**（`:444-468`），语义是「LIO 当绝对定位，静态桥补帧」；<br>②`localization:=icp`——`icp_registration` 的 `map→odom` 本质是「滑窗锚定的静态偏移」（`docs/tf_interface_contract.md:19`：「100 Hz 专用线程」），行为上**接近静态桥但会缓慢修正**；<br>**其余一律不行**：`amcl` / `slam_toolbox(localization)` / `cartographer(localization)` / `cartographer(全包)` 都自己发 `map→odom`。 | `bringup_sim.launch.py:444-468`；`docs/tf_interface_contract.md:17-19,44,126-128`；`docs/algorithm_axes.md:63`（A2 轴） |
| **(ii) 我们会失去什么？** | ①**先验图重定位能力**（`mode:=nav` 存在的意义）；②**长时漂移被修正的能力**（回环修正不再进入位姿）；③**「换图不用重打航点」的能力**（R4）；④**「被搬动后可恢复」的能力**（R5）。<br>⇒ 换句话说：静态桥不是「一种更简单的定位」，而是**把定位问题整体删掉**——只有在「场地固定、起点固定、单局时间短」时才是等价解。 | R1–R6；`docs/cod_nav_2026_integration_plan.md:175`（同结论） |
| **(iii) 能不能靠调 `slam_toolbox.transform_publish_period` 代替静态桥？** | **不能**，但**值得改**：<br>①`transform_publish_period` 只控制**发布频率**，不控制**是否有修正**（`slam_toolbox_common.cpp:255-259`：非 0 就按该周期把当前 `map_to_odom_` 发出去）；设成大值（如 1~5 s）只是把「连续小抖」变成「低频大跳」，**对 `getRobotPose()` 更糟**（nav2 的 TF 超时窗口是 0.1~0.2 s，见 `singlenav2_params.yaml:117` `transform_tolerance: 0.1`、我们 `nav2_params_sim_rpp.yaml` 的 `transform_tolerance`）。<br>②设成 0（COD 的做法）**只与静态桥配套**：等于宣布「SLAM 不参与定位」。<br>③**真正该调的是另外两个**：`map_update_interval`（栅格重排周期）与 `minimum_time_interval`（位姿图优化节流）。我们当前是 **`map_update_interval: 5.0`、`minimum_time_interval: 0.5`**（`src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:29,32`），COD 成品是 **`1.0` / `0.3`**（`mapper_params_online_async.yaml:31,35`）。 | `slam_toolbox_common.cpp:74-83,251-275`；我们的参数见上；`docs/cod_nav_2026_integration_plan.md:174`（C20 同方向） |
| **(iv) 我们 `slam_nav` 已经踩到的「在线图滞后 / 缝」，静态桥会更好还是更坏？** | **会更坏（结论），而且我们踩的坑与静态桥无关**：<br>**FACT（我们的症状链）**：`docs/debug_fastlio_cartographer.md:867` ——「nav2 激活期 `base_link_fake→odom` 超时 → cartographer `Ignored subdivision`（零戳）→ 拒收扫描、**无 `map→odom`** → 全局路径只在已知区边界就停 → 贴墙时 `/scan` 丢墙 → **缝被当走廊 ⇒ 穿墙撞墙**」；`:870` 遗留①「缝：`range_min` 改小**没解决**」；`:869` 实测「近场（<1.5 m）障碍点 `z_map ≤ 0.196`、远场 `z_map ≈ 0.40`（墙高≈0.40 m）；`/scan` 的 `inf` 占 22~30%（主因 `range_max 10 m` < 场地 15×28 m）」；`:872`「`robot_radius 0.40`」。<br>**为什么静态桥更坏**：静态桥会让 `map` 与 `odom` 永久同源 ⇒ **在线 SLAM 的闭环修正永远进不了位姿** ⇒ 「图漂移」不再被位姿吸收，而是**直接叠在规划用的那张栅格上**（R1/R7）。我们的 `slam_nav` 已经是「图不可信 + 感知有缝」，再叠加「图不会被修正」，只会让 planner 更无从判断。<br>**真正的对策方向**（证据支持）：①**把 `map_update_interval` 从 5.0 降下来**（对齐 COD 的 1.0）——我们的「帧→栅格」延迟是 5 s 量级，而 `transform_publish_period` 是 0.02 s（`mapper_params_online_async_sim.yaml:28-29`）⇒ **50 Hz 的位姿 + 0.2 Hz 的图**，这个错配本身就是「滞后」的一种解释；②**把感知缝隙当成独立问题修**（`range_max 10→20`、裁剪盒、局部 STVL：`docs/cod_nav_2026_integration_plan.md` P0 的三条）；③**不要在 `slam_nav` 里引入静态桥**。 | 见左栏逐条 `file:line` |

**B 部分一句话**：COD 用静态 `map→odom` 的**真原因是 (a)+(c)+(d)**——「无先验图 ⇒ map≡odom 零成本」「环境简单/面积小 ⇒ 定位容忍度大 ⇒ 纯 LIO」「KISS 原则」；
「避免回环跳变扯动 costmap/MPPI/自转底盘」这半边**被证据反证**（他们根本不让 slam_toolbox 发 TF；且他们从不自转）；
代价是**漂移变图畸变 + 起点/朝向被绑死 + 换图即失效 + 被搬动不可恢复**。
**对我们**：不应作为默认；我们**已经**在 `mode:=nav + localization:=''` 这个回退角落里有了同样能力（`bringup_sim.launch.py:444-468`），
额外能做的只有「调 `slam_toolbox` 的 `map_update_interval`」这一件事。

---

## C. 宏观大对比：能力地图

> 表格口径：「判定」= **我们强 / 平 / 他们强**（相对我们 bench 现有资产，不看谁赚了钱）；
> 「一句话原因」必须能落到参数或文件。COD 侧证据以 **Gitee 成品**为准，GitHub 季前值在括号里注出。

| 维度 | COD（RMUL2026 成品） | 我们 | 判定 | 一句话原因 |
|---|---|---|---|---|
| **1. 传感器** | Livox MID-360 + `/livox/imu`（`config/mid360.yaml:3-6`）+ **RealSense D435i 真接进 STVL**（`singlenav2_params.yaml:218,241` `observation_sources: livox_source realsense_source`） | Gazebo 旋转仿真 Mid360（`/livox/lidar` CustomMsg + `/livox/lidar/pointcloud`）+ linefit 地面分割 + 无深度相机（`docs/debug_fastlio_cartographer.md` §10.0） | **他们强** | 他们多一路真实 3D 相机观测源（`model_type: 0`、`obstacle_range 3.0`、`min_obstacle_height 0.18`，`:248-262`）；⚠️ 但**我们仿真链路的雷达是旋转式的、更接近真车策略**，且我们有独立地面分割（见第 3 行） |
| **2. LIO** | **`small_point_lio`**（逐点 ESKF + iVox，`map_resolution 0.5`，无回环，**无 de-skew**，`twist≡0`） | `fastlio`（帧级 iterated-EKF + ikd-Tree）+ 可选 `pointlio`（同族）+ `lio_tf_adapter` 统一到 `odom→base_link`/`/odom`（`docs/tf_interface_contract.md` T2/T3） | **我们强** | 我们有**统一契约层**（`:74-75`）与两种可 A/B 的 LIO；他们是「一个 LIO 直接发硬编码帧名」（`small_point_lio_node.cpp:61-62`） |
| **3. 地面分割** | **没有**（2026 整条砍掉，`patchwork`/`linefit` grep 0 命中）；用 p2l **硬高度带**取代（`min_height 0.10`、`max_height 1.00`，`multiplenav_launch.py:72-73`） | `linefit_ground_segmentation` → `/segmentation/obstacle` + `/segmentation/ground`（`docs/debug_fastlio_cartographer.md` §10.0） | **我们强** | 硬高度带对坡道/矮墙不鲁棒；linefit 是**可 A/B、可诊断**（z-profile 工具）的资产 |
| **4. 2D 建图** | `slam_toolbox` async **`mode: lifelong`**，`map_update_interval 1.0`、`minimum_time_interval 0.3`、`resolution 0.05`、`max_laser_range 10.0`（`mapper_params_online_async.yaml:19,31-35`）；另有 `auto_save_map.launch.py` 每 30 s 存一次图（**路径写死 `/home/cod-sentry/...`**，`:13-15`） | `cartographer`（**打了核心补丁 `min_probability_to_clear: 0.80`**，`docs/debug_fastlio_cartographer.md` §10.1）+ `slam_toolbox`（`map_update_interval 5.0`、`minimum_time_interval 0.5`，`mapper_params_online_async_sim.yaml:29,32`） | **我们强（图质量）/ 他们强（更新率）** | 我们的图是**唯一被用户验收过的资产**（§10.1「这个地图是可以的」）；但**他们的栅格更新比我们快 5 倍**（1.0 vs 5.0 s）⇒ 这一条要单独抄（§D 槽位） |
| **5. 定位/重定位** | **零**（无 amcl、无 icp、无 carto、无先验图重定位）；`localization_launch.py:44` 只留 `map_server`；静态 `map→odom` | **五种**：`amcl` / `slam_toolbox(localization)` / `icp`(3D PCD) / `cartographer`(pbstream 纯定位) / `cartographer`(全包形态兼任里程计)；`docs/tf_interface_contract.md:17-19,126-128`；`bringup_sim.launch.py:136` | **我们强** | 这是**代差**：他们把一个失败类整体删除，我们把它做成可 A/B 的四个槽位 |
| **6. TF 契约** | `odom→base_link` 硬编码在 LIO 节点内（`small_point_lio_node.cpp:61-62`）；`map→odom` 静态桥（`singlenav_launch.py:67-89`）；`base_link→base_link_fake` 由 `fake_vel_transform` 发；**依赖外部提供 `base_link→livox_frame`**（LIO 查不到就丢帧，`:64-69`） | **显式契约文档 + 每边恰好一个发布者**：`docs/tf_interface_contract.md:44`（目标帧树）、`:47-53`（角色/禁止事项）、T1–T5 已实施（`:73-77`）；`lio_tf_adapter` 统一 `/odom`；Gazebo 关 `publish_odom_tf` | **我们强** | 他们的帧名是**编译期常量**、TF 来源**不在仓库内**（`docs/cod_nav_2026_deep_dive.md:495`）；我们的契约被文档化并**踩过坑后修好**（`docs/tf_interface_contract.md` §二 P1–P6） |
| **7. 全局规划** | **`SmacPlanner2D`** + `cost_travel_multiplier 4.0` + `tolerance 0.5` + `allow_unknown true` + `smooth_path: true` + 内建 `smoother{w_smooth 0.4, w_data 0.1, do_refinement true}`（`singlenav2_params.yaml:375-405`） | **`NavfnPlanner`** + `allow_unknown: true`（`nav2_params_sim_rpp.yaml:357-362`）；`planner` 槽**目前只有 navfn** | **他们强** | `cost_travel_multiplier` 是把路径推向代价谷底的**直接对策**（`singlenav2_params.yaml:384` 注释原文），我们正是「贴缝走」的受害者 |
| **8. 局部控制** | **`goal_approach_controller` 包 MPPI(Omni)**，50 Hz、`time_steps 80`、`model_dt 0.02`、`batch_size 1500`、`vx/vy ±2.5`、`temperature 0.25`、`gamma 0.015`；近目标限速 + **2 m 内绕开 MPPI 直驱**（GAC:96-126）；`velocity_smoother` 30 Hz / OPEN_LOOP（`singlenav2_params.yaml:471-482`） | `rpp`（默认，`desired_linear_vel: 0.5`）/ `dwb` / `teb` 三槽；**MPPI 包已装但未接**（`docs/cod_nav_2026_params_review.md` §0.4） | **他们强** | 差的是**控制器族**（采样式 omni + 近目标直驱）；⚠️ 但他们的 `vx_max 2.5` 是**实车收敛值**，可直接当我们 A/B 的首版（`docs/cod_nav_nav_stack_inventory.md:168`） |
| **9. 代价图与障碍** | **两张图都挂 STVL**（`voxel_decay 0.5/1.0`、`decay_model 0`、`voxel_size 0.05`、源级 `obstacle/raytrace 8/9 m`、`model_type 1`、`vertical_fov_angle 2.00`、`filter: voxel`），并**用裁剪盒去掉车体点云**（`multiplenav_launch.py:35-49`：x±0.2 / y−0.2~0.4 / z−0.1~0.2 / `negative True`），**全局也 `rolling_window: true`**（25×25@0.04 单点 / 50×50@0.04 多点） | global = `static + obstacle + stvl + inflation`（STVL 在 global，`nav2_params_sim_rpp.yaml:258,284-288`，`voxel_decay 0.5`）；local = `obstacle + obstacle_cloud + inflation`（**无时间维**，`:182`）；`robot_radius 0.40`（`:178,252`）；**无裁剪盒**（`/segmentation/obstacle` 直接进 local cloud 层，`:213-224`） | **他们强（近距/时间维）/ 我们强（诚实几何）** | 他们的「裁剪盒 + 双图 STVL + 源级 8/9 m」直接打我们「近距盲区 / 幽灵障碍」两个痛点；我们的 `robot_radius 0.40` 是**按 0.2 车体 + 0.2 近距盲区**算出来的诚实值（`:173`），他们单点只有 **0.1 m 方框**（`singlenav2_params.yaml:190-191`） |
| **10. 任务层** | BT（自研 XML：`RateController 3 Hz` 重规划、`RecoveryNode` 分层重试 10 次、`RemovePassedGoals radius 0.7`）+ **6 组语义航点 CSV**（`cod_bringup/wps/`）+ **仓内 `waypoint_editor`**（RViz 打点）+ 决策在**另一个仓库**（`cod_-rm2026_-behavior-tree`，9 分支） | stock BT + 自研分段工具（`tools/scripts/nav/segment_goal_navigator.py`）；航点图/前沿探索**未做**（`docs/debug_fastlio_cartographer.md` §10.5 遗留⑥） | **他们强（成品度）** | 他们有**从打点到执行到决策的完整闭环**（CSV → `waypoint_through_nav2` → BT → 裁判串口），我们只有 stock BT + 工具脚本 |
| **11. 仿真** | **GitHub `rmul2026` 分支没有任何仿真**（无 worlds/无 gazebo launch）；作者另外维护 **`COD_loopback_sim`**（把 Nav2 `loopback_sim` 回移 Humble + pixi）用于「不启 Gazebo 也能跑决策/控制」 | **一等公民**：Gazebo Classic 场地（RMUC/RMUL/RMUL2026）+ 旋转 Mid360 插件 + 多算法 A/B（`docs/architecture.md` §一） | **我们强** | 我们有**物理仿真 + 真值 odom + 场景可切换**；他们的 loopback 是**刻意去掉物理**的（README 自述「physics-engine artifacts 是干扰」，`docs/cod_nav_nav_stack_inventory.md:59`） |
| **12. 测试与工程** | CI **仅编译**（`skip-tests: true`，且路径拼写 `ws_liovx` 疑似一直失败）；提交了 `.idea/`、`launch/__pycache__/*.pyc`；`auto_save_map.launch.py:13-15` 与 `singlenav2_params.yaml:360` 写死 `/home/cod-sentry/...`；`map_server.yaml_filename` 是死值 | 有回归闸门（`tools/scripts/regress/nav_smoke_regression.py`）、链路体检（`tools/scripts/diag/watch_startup_chain.py`）、`docs/` 体系、无硬编码 | **我们强** | 工程卫生是**我们唯一的「碾压项」**（`docs/cod_nav_comparison.md:36`） |
| **13. 决策与下位机** | **完整**：决策库 `cod_-rm2026_-behavior-tree`（BT 4.8 + Groot2，10 棵 XML，裁判系统 0x0001…0x020D 消息表）；下位机 `rmcod2026_-sentry`（舵轮解算 + RLS 功率拟合 + 变档小陀螺 + 超电 + 热量控制）；串口帧 `0xA5 + 3×float + checksum = 15 B`（`ros2_simple_serial/src/cod_serial.cpp`）；**唯一导航侧契约**是 `/aft_cmd_vel` | **无**：仿真 bench，无下位机、无裁判系统、无自瞄；契约只在文档层（`docs/sim_real_contract.md`） | **他们强** | 这是**体系差距**，但也是**我们不该追**的部分（我们没有车）；我们该抄的只有**「上位机速度语义必须与轮速换算对齐」这一条契约**（作者原话 NOTE，见 §B1.3） |

**C 部分一句话**：我们强在 **契约（TF/参数/文档）/ 重定位资产 / 地面分割 / 二维图质量 / 仿真 / 工程卫生**；
他们强在 **控制器与规划器族（MPPI+wrapper、Smac2D）/ 近距与时间维障碍表示（裁剪盒+双图 STVL）/ 任务层成品度 / 决策-下位机体系 / slam_toolbox 更新率**。

---

## D. COD 融入我们架构的宏观视图

### D1. 目标架构图（文字版；默认链 = 一个字不动，新增 = 候选分支）

```
[传感器]  Gazebo 旋转 Mid360 ──> /livox/lidar(CustomMsg) ─┬─> /livox/lidar/pointcloud(PointCloud2)
                                                          │
                        ┌─────────────────────────────────┴──────────────┐
                        │  ★新增候选：rm_cloud_crop（裁剪盒）             │
                        │     输出 /livox/lidar_filtered                  │
                        └─────────────────────────────────┬──────────────┘
                                                          │（只喂 3D 代价图层，/scan 链不动）
[E 感知]  linefit_ground_segmentation ─> /segmentation/obstacle ─┬─> local: obstacle_cloud_layer（候选）
                                                                 └─> global: stvl_layer（现状）
          pointcloud_to_laserscan ─> /scan ─┬─> local: obstacle_layer（现状默认）
                                            ├─> global: obstacle_layer（候选 global_obstacle:=scan）
                                            └─> mapper / localization

[A0 LIO]  lio:=fastlio(默认) | pointlio | none | cartographer
          ★新增候选：lio+=smallpointlio？—— **不新增**（见 §A4：tag/timestamp 硬门槛 + twist≡0）
          └─> /odom ──> lio_tf_adapter ──> TF odom→base_link
[A1 建图] mapper:=cartographer(默认) | slam_toolbox
          ★新增候选：mapper+=slam_toolbox_lifelong（参数集：map_update_interval 1.0 / minimum_time_interval 0.3
                                                   / minimum_travel_heading 0.1 / mode: lifelong）
                                                    ⚠️ 它必须自己发 map→odom（transform_publish_period≠0）
[A2 定位] localization:=amcl | slam_toolbox | icp | cartographer（仅 mode:=nav）
          ★不新增：静态 map→odom 桥（现状只在 mode:=nav + localization:='' 这条回退里存在，保持不变）
          └─> TF map→odom（**每条边恰好一个发布者**：docs/tf_interface_contract.md:44）

[B 规划] planner:=navfn(默认) ──> ★新增候选：planner+=smac2d（tolerance 0.5 / cost_travel_multiplier 4.0 /
                                                  allow_unknown true / smooth_path true + 内建 smoother）
          smoother := simple(默认) ──> ★新增候选：smoother+=savgol（**必须在我们自研 BT 里显式加 <SmoothPath>**）

[C 控制] nav:=rpp(默认) | dwb | teb ──> ★新增候选：nav+=mppi（Omni；model_dt 必须 = 1/controller_frequency）
                                                    └─ 可选第 5 个候选：mppi_approach（= 上游 goal_approach_controller 同思路：
                                                       近目标限速 + direct_approach 直驱，**需自研或 vendor 并 colcon build**）

[F 底盘]  /cmd_vel ──> velocity_smoother ──> fake_vel_transform ──> /cmd_vel_chassis ──> Gazebo 麦轮
          spin_speed==0 直通（默认，我们已修） | spin_speed!=0 小陀螺（覆盖 angular.z，**必须同批放宽 yaw_goal_tolerance**）
```

### D2. 槽位映射表（我们每个槽位新增哪些取值）

> 「成本」= 改动量级；「判定」= ✅直接融 / △需改造 / ❌不融（与 `docs/cod_nav_2026_integration_plan.md:162-178` 对齐，本文补上**为什么**与**依赖顺序**）。

| 槽位 | 新增取值 | 来源（file:line / URL） | 判定 | 依赖/前提 |
|---|---|---|---|---|
| `lio` | **不新增**（`smallpointlio` 不落地） | `livox_pointcloud2.h:25-37`；`small_point_lio_node.cpp:90-96` | ❌ | 已有 `pointlio`（同族）；见 §A4 |
| `planner` | `planner+=smac2d` | `singlenav2_params.yaml:375-405` | ✅ | 无需改包（`nav2_smac_planner` 已装）；**不要抄** Hybrid 专用键（`motion_model_for_search`/`angle_quantization_bins`/`minimum_turning_radius`/`analytic_expansion_*`/`rotation_penalty`/`lookup_table_size`/`allow_reverse_expansion`，对 `SmacPlanner2D` 惰性） |
| `smoother` | `smoother+=savgol` | `singlenav2_params.yaml:407-420` | △ | **必须在我们自研 BT 里显式加 `<SmoothPath>`**，否则是死代码（他们的 BT 就没有它：`docs/cod_nav_2026_deep_dive.md:236`） |
| `nav` | `nav+=mppi`（Omni）；可选 `nav+=mppi_approach` | `singlenav2_params.yaml:84-176`；GAC `:96-126` | ✅（mppi）/ △（wrapper） | **硬约束三条**（原文见 `docs/debug_fastlio_cartographer.md:888`）：①`model_dt == 1/controller_frequency`（否则 `optimizer` 抛异常、controller_server 起不来）；②`CostCritic.consider_footprint: true` 在**只有 `robot_radius` 的圆形 costmap** 上会抛异常 ⇒ 先加 footprint 或设 false（我们现状：`nav2_params_sim_rpp.yaml:178,252` 只有 `robot_radius: 0.40`、无 footprint）；③`min_y_velocity_threshold` 必须降到 0.001（我们现状 `nav2_params_sim_rpp.yaml:107` 是 `0.5`，会把 Omni 的侧向反馈抹零）。另见 `docs/cod_nav_2026_params_review.md:94,704` |
| `local_obstacle` | `local_obstacle+=stvl`（带 `voxel_decay`） | `singlenav2_params.yaml:200-262` | ✅ | 包已装；**先只加时间维这一条**，其余层不动 |
| `global_obstacle` | （我们已有 `stvl`）补 `model_type: 1` + `vertical_fov_angle: 2.00` + `filter: voxel` | `singlenav2_params.yaml:237-240` | ✅ | 我们 global 的 stvl 已开，但**这三个键是最容易漏的**（`model_type: 0` 会按 0.7 rad 垂直 FOV 裁切） |
| `crop_box`（新节点槽） | `crop_box:=on|off` | `multiplenav_launch.py:35-49`；`cpp_lidar_filter/src/filter_node.cpp` | ✅ | **必须在 `livox_frame` 下重标定**（`CropBox` 不做 TF，而 marker 画在 `base_link`）；`leaf_size` 是死参（`VoxelGrid` 整段注释） |
| `mapper` | `mapper+=slam_toolbox_lifelong` | `mapper_params_online_async.yaml:19,31-35` | △ | 必须**让它发 `map→odom`**（把 `transform_publish_period` 从 0.0 改成非 0）；必须**与 `localization:=*` 互斥**（否则双发布者） |
| `localization` | **不新增静态桥**；`mode:=nav + localization:=''` 的回退保持现状 | `bringup_sim.launch.py:444-468` | — | 已存在 |
| 任务层 | `waypoint_through_nav2`（CSV → `navigate_through_poses`） | `waypoint_editor/USAGE_WITH_NAV2.zh.md`；`cod_bringup/wps/*.csv` | △ | 需 vendor/自研桥节点；`waypoint_editor` 与 Nav2 **都会启 `map_server`**（冲突）⇒ 二选一 |
| 哨兵语义 | （我们已有 `spin_speed`）语义分档成文 | `fake_vel_transform.cpp`；`singlenav2_params.yaml:101-105` | ✅（认知项） | `spin_speed!=0` 时 MPPI 的 `wz` 被丢弃 ⇒ **必须同批放宽 `yaw_goal_tolerance`**（他们多点用 6.28） |
| 决策/下位机 | **不融** | — | ❌ | 体系不同；只借「cmd_vel→轮速一致性检查」这一条契约（BBS 1882897 的 NOTE） |

### D3. 依赖顺序（谁必须先做、为什么）

```
P0-1  裁剪盒（crop_box）           ── 独立节点、独立话题、可随时停用 ⇒ 最先做，直接打「近距/自身点云」
  │                                  （但它需要先有一份稳定的 base_link→livox_frame TF：我们已有，见 tf_interface_contract §一）
  ├─> P0-2  global stvl 三键修正（model_type 1 / vertical_fov 2.00 / filter voxel）
  │         ── 与 P0-1 同批：新点云进来后，视锥必须正确，否则「看到却不标记」
  └─> P0-3  local_obstacle+=stvl（先只加 decay）
            ── 必须在 P0-1/P0-2 之后：否则「近距点云 + 错误视锥 + 无时间维」会互相掩盖现象

P1     planner+=smac2d ── 与感知无耦合，可并行；但要先有 P0 的图才看得出「路径是否居中」

P2     nav+=mppi ── 必须在「costmap 已有可用梯度」之后（inflation 梯度 + 时间维），
        否则会复现他们「整个膨胀区都当碰撞 ⇒ MPPI 没梯度」的老问题；
        且必须与 `spin_speed` 语义一起定（wz 归属），不能两个都开
  └─> P2b  mppi_approach（wrapper）── 最后做：它只在 MPPI 已经能跑之后才有意义

P3     mapper+=slam_toolbox_lifelong ── 最后做：
        ①它是唯一会引入「第二个 map→odom 候选」的改动 ⇒ 必须在 launch 里加互斥条件；
        ②它的价值（更新率）只有在 P0 把图/感知修好之后才可判定；
        ③⚠️ 但它里面有一条**可以提前做且零风险**：把我们现网 slam_toolbox 的
          `map_update_interval 5.0 → 1.0`（对齐 COD）——这条只有一行，且能直接检验「缝」是否与栅格更新率有关
```

### D4. 冲突清单（必须显式互斥/同批）

| # | 冲突 | 事实/机制 | 处置（launch 或参数层） |
|---|---|---|---|
| C1 | **`map→odom` 双发布者** | `localization:=amcl/slam_toolbox/icp/cartographer` 各自发布（`docs/tf_interface_contract.md:17-19`）；`mapper:=slam_toolbox_lifelong` 若把 `transform_publish_period` 设为非 0 也会发布（`slam_toolbox_common.cpp:255-259`）；`mode:=nav+localization:=''` 时还有 T1 静态桥（`bringup_sim.launch.py:444-468`） | **必须在 launch 的条件里做四路互斥**（现有代码已对 amcl/slam_toolbox/icp 做了：`:303-406`；新增 `mapper` 取值时要把它算进去） |
| C2 | **`wz` 归属** | `fake_vel_transform` 在 `spin_speed!=0` 时**无条件覆盖** `aft_tf_vel.angular.z`（`fake_vel_transform.cpp`；我们已改成 `0 直通`，**语义与他们不同**）；MPPI 会输出 `wz`（`singlenav2_params.yaml:101,105`） | 二者**互斥**：要么 `spin_speed=0` 让 MPPI 控朝向，要么自转 + `yaw_goal_tolerance` 放宽（他们多点 6.28）/ 单点用 `PositionGoalChecker`（`singlenav2_params.yaml:79-83`） |
| C3 | **无 footprint 的 MPPI** | 我们只有 `robot_radius 0.40`、**没有 footprint 多边形**（`nav2_params_sim_rpp.yaml:178,252`）；`CostCritic.consider_footprint: true` 在圆形 costmap 上会抛异常（`docs/debug_fastlio_cartographer.md` §10.4-6） | 接 MPPI 前：**加一个 footprint 多边形**（按车体几何）或把 `consider_footprint` 设 false |
| C4 | **`spin_speed` 语义** | 我们：`spin_speed==0` **直通**（保留 `wz`）；他们：`spin_speed==0` ⇒ **`wz≡0`**（无条件覆盖）。同一个参数名，两种语义 | 移植任何「他们的小陀螺参数」时必须**按我们的语义重算**，不能把 `spin_speed 0.0` 当成「自转关、朝向交给 Nav2」（在他们那里恰好相反） |
| C5 | **`model_dt` × `controller_frequency`** | MPPI 要求 `model_dt == 1/controller_frequency`，否则 `controller_server` 直接 configure 失败（`docs/debug_fastlio_cartographer.md` §10.4-6） | 抄他们的 `model_dt 0.02`（单点）**就得同时把 `controller_frequency` 设 50**；我们若用 20/30 Hz，必须改成 `0.05/0.0333` |
| C6 | **`smoother_server` 是死代码** | 他们的 BT 里**没有 `<SmoothPath>`**（`docs/cod_nav_2026_deep_dive.md:236`） | 我们若上 `smoother+=savgol`，**必须同步改自研 BT**，否则白起一个节点 |
| C7 | **`waypoint_editor` × `map_server`** | 两者都会启 `map_server`（`waypoint_editor/USAGE_WITH_NAV2.zh.md`，`docs/cod_nav_nav_stack_inventory.md:60`） | 二选一，或用不同 namespace |
| C8 | **`local_obstacle+=stvl` × 我们的 `scan` 默认** | 我们 local 默认 `scan`（`bringup_sim.launch.py:169-172`）；STVL 吃 3D 点云/`/livox/lidar_filtered` | 加 STVL 是**加一层**不是换层；要同时决定「谁来 clearing」（STVL 的 raytrace 与 obstacle_layer 的 raytrace 会互相打架） |

### D5. 「最终形态」描述

1. **默认路径一个字不动**：`lio:=fastlio` + `mapper:=cartographer` + `localization:=amcl`（`mode:=nav`）+ `planner:=navfn` + `nav:=rpp` + `linefit` + `local_obstacle:=scan`。
   证据：`bringup_sim.launch.py:136,143,176`；`docs/cod_nav_2026_integration_plan.md:19`（「只增槽，不换默认」）。
2. **新增候选全部可 A/B、可回退**：每个新取值都只对应「一个新 params/一个新节点 + 一个 launch 参数值」，
   回退 = 把该参数改回默认（`nav:=rpp` / `planner:=navfn` / 停裁剪盒节点 / 删 STVL 层）。
3. **契约不变量仍然成立**：`map→odom` 每条边**恰好一个发布者**；`odom→base_link` 只由 `lio_tf_adapter` 或 `lio:=cartographer` 发布；
   `base_link→base_link_fake` 只由 `fake_vel_transform` 发布（`docs/tf_interface_contract.md:44-53,88`）。
4. **每一个候选都有独立判据**：P0 用 `cloud_z_profile.py` / `costmap_marking_check.py` / `nav_smoke_regression.py`；
   P1/P2 用「路径居中程度 / 震荡幅度 / 到达时间 / CPU」写回 `docs/algorithm_matrix.md §四`（判据与验收方式沿用 `docs/cod_nav_2026_integration_plan.md:44-48` 与 `:129-131`）。

---

## E. 结论

### E1. 可融入比例（按「组件个数」与「价值」两个口径）

| 分类 | 组件 | 个数 |
|---|---|---|
| **直接融（✅）** | ①裁剪盒 ②global STVL 三键（`model_type 1`/`vertical_fov 2.00`/`filter voxel`）③`local_obstacle+=stvl` ④STVL 源级 `obstacle/raytrace 8/9 m` ⑤`planner+=smac2d`（9 个有效键 + 内建 smoother）⑥`nav+=mppi`（骨架 + critic 键名 + `temperature/gamma/reset_period`）⑦inflation 梯度（`cost_scaling_factor 5.0`，半径按 `robot_radius 0.40` 重算）⑧`p2l.range_max 10→20` ⑨`velocity_smoother` 键名对齐 ⑩slam_toolbox 更新率（`map_update_interval 1.0`） | **10** |
| **需改造（△）** | ①`smoother+=savgol`（要先改 BT）②`nav+=mppi_approach`（要 vendor/自研 wrapper，且它把 `angular.z` 置 0）③`mapper+=slam_toolbox_lifelong`（要加互斥 + 要让它发 TF）④`goal_checker`（单点 `PositionGoalChecker` / 多点 `yaw 6.28`，要同批改）⑤`waypoint_through_nav2` 任务层 ⑥高度带（**禁止照抄**，必须按我们 z 剖面重算）⑦裁剪盒数值（必须按 `livox_frame` 重标） | **7** |
| **不融（❌）** | ①`small_point_lio`（tag/timestamp 硬门槛 + `twist≡0` + `map_resolution 0.5` + `ROOT_DIR` 写死）②静态 `map→odom` 作为默认（我们四种重定位是资产）③去掉 linefit 改硬高度带 ④5 m/s+ 速度包线 ⑤50×50@0.04 `always_send_full_costmap` ⑥`auto_save_map.launch.py`（写死路径）⑦RealSense 链路（我们无相机）⑧决策/下位机体系 | **8** |

⇒ **「直接融 10 / 需改造 7 / 不融 8」**，且**没有任何一条要求改我们的架构**（都是「加一个槽位取值」或「改一个参数」）。
这印证了 `docs/cod_nav_2026_integration_plan.md:158-188` 的结论：**能融，但正确形状不是搬架构，而是把他们的组件变成我们槽位的新取值**。

### E2. 收益最大的一条

**`planner+=smac2d` 的 `cost_travel_multiplier: 4.0`（+ 内建 `smooth_path`）。**
理由：它**直接对准我们唯一被用户验收过、但一直没解决的规划症状**——「贴缝走 / 把缝当走廊 ⇒ 穿墙撞墙」
（`docs/debug_fastlio_cartographer.md:867,870`）。它是一条**已有包的参数级改动**（`nav2_smac_planner` 已装）、
不引入新节点、不改变契约，且 `cost_travel_multiplier` 的语义（`singlenav2_params.yaml:384` 注释「较大的值将更精确地放置在通道的中心」）
正是「把路径推离高代价区」的**直接对策**。

> 次优：**裁剪盒 + 双图 STVL**（打「近距盲区 / 幽灵障碍」），但它引入新节点与标定成本；
> `nav+=mppi` 收益高但**必须先解决 C2（wz 归属）与 C3（无 footprint）**，否则起不来/不收敛。

### E3. 风险最大的一条

**「照抄他们的 yaml」这一动作本身**（不是某一条参数）。
两条硬证据：①他们的 yaml 大量使用 **nav2 Jazzy/main 才有的键**，在本机 1.1.20 上**静默失效或直接抛异常**
（`docs/cod_nav_2026_params_review.md` §0.1、§0.4：`vy_min/wz_min/ax_*/ay_*/az_*`、`publish_critics_stats`、
`path_length_tolerance`、`CostCritic.trajectory_point_step`、planner 段 16 个 Hybrid 专用键等）；
②更危险的是**「看起来生效其实生效了、但语义不同」的参数**——`spin_speed` 就是活例（C4）：
在他们是「`wz≡0`」，在我们是「直通」，同名反义。
⇒ **风险控制办法**：任何新槽位落地前先 `ros2 param list <node>` 逐键核对存在性（`docs/debug_fastlio_cartographer.md` §10.4-4），
并把「语义是否相同」单独写一行。

### E4. 如果只做三件事，做哪三件

| 优先级 | 做什么 | 为什么是它 | 验收/回退 |
|---|---|---|---|
| **第一件** | **`planner:=navfn` → 新增 `planner:=smac2d`**，抄 `tolerance 0.5`、`allow_unknown true`、`cost_travel_multiplier 4.0`、`max_planning_time 4.5`、`smooth_path true` + 内建 `smoother{max_iterations 10000, w_smooth 0.4, w_data 0.1, do_refinement true}`；**不抄** Hybrid 专用键 | 收益最大（E2）、风险最低、零新依赖、直击「贴缝走」 | 判据：同一目标的路径是否居中于代价谷底、`velocity_smoother` 饱和次数下降、规划耗时 < `max_planning_time`；回退 `planner:=navfn`（`docs/cod_nav_2026_integration_plan.md:51-54`） |
| **第二件** | **裁剪盒 + 双图 STVL 三件套**：新增 `/livox/lidar_filtered`（`CropBox`，`negative`，数值**在 `livox_frame` 下重标**）→ 只喂 **global STVL**（补 `model_type: 1`、`vertical_fov_angle: 2.00`、`filter: "voxel"`、源级 `obstacle_range 8.0 / raytrace_range 9.0`）与 **local 新增 STVL 层**（`voxel_decay 0.5`、`decay_model 0`、`voxel_size 0.05`） | 打我们两个最疼的感知痛点（近距盲区、幽灵障碍），且是他们**真正在用的**组合（`multiplenav_launch.py:35-49` + `singlenav2_params.yaml:200-262`） | 判据：`cloud_z_profile.py` 的 0–0.5 m 桶不再空、贴墙时 `inf` 探针有回波、`costmap_marking_check.py` 近距标记不下降；回退 = 停裁剪盒节点 + 删 STVL 层（`docs/cod_nav_2026_integration_plan.md:44-48`） |
| **第三件** | **`nav+=mppi`（Omni）+ 先把三个硬约束解掉**：①加 footprint 多边形（或 `CostCritic.consider_footprint: false`）②`model_dt ≡ 1/controller_frequency` ③`min_y_velocity_threshold 0.5 → 0.001`；同时**明确 `wz` 归属**（`spin_speed:=0` ⇒ MPPI 控朝向；要自转 ⇒ 同批放宽 `yaw_goal_tolerance` / 换 `PositionGoalChecker`） | 收益第二（控制器族换代 + 现成的战训参数），但**只有把三个硬约束解决掉才起得来**，所以必须作为一个整体做 | 判据：贴墙最小距离、窄缝通过率、震荡幅度、到达时间、CPU（三方对照写回 `docs/algorithm_matrix.md §四`）；回退 `nav:=rpp` |

> **明确不做（前三件之外）**：`small_point_lio`、静态 `map→odom` 作为默认、`slam_toolbox_lifelong`（除非先有互斥条件）、
> `smoother:=savgol`（除非先改 BT）、RealSense 链路、5 m/s+ 速度包线、决策/下位机体系。

### E5. 本次**未能确定**的东西（明确列出）

1. **`singlenav_launch.py:82-83` 的 `yaw = -0.5` 的确切物理含义**：仓库内无注释、无文档；Gitee 只有 1 个提交（本地 clone 是 `--depth` 浅克隆，`git log -S'--yaw'` 也只得 HEAD），
   **无法从历史确定它何时/为何被引入**。作者帖子只说 `0,0,0`。⇒ **无法确定**。
2. **他们真机的 `/Odometry.twist` 是否有值**：公开快照 `small_point_lio_node.cpp:90-96` 是注释掉的；但 BBS 1882897 作者说「这是我修改过的 small point lio，**可以输出速度**」。
   ⇒ 两者矛盾，**仓库外版本无法验证**。这可能影响他们 `velocity_smoother` 的 `feedback: "OPEN_LOOP"`（`singlenav2_params.yaml:475`）是否真的不读 odom。
3. **`base_link → livox_frame` 的实际外参来源**：Gitee 成品仓内**只有 `small_point_lio/launch/small_point_lio.launch.py:24-45` 这一处静态 TF**，
   而 cod_bringup 的 launch 都**直接起 `small_point_lio_node`、不 include 它**；仓库内也没有 `robot_state_publisher` 的启动。⇒ **他们实际从哪加载 URDF/TF 无法确定**
   （GitHub 季前同样如此，见 `docs/cod_nav_2026_deep_dive.md:495`）。
4. **`singlenav_launch.py` 里 RealSense 的 TF/帧是否真的被 STVL 消费**：`realsense_source` 的 `topic: /camera/camera/depth/color/points`（`singlenav2_params.yaml:244`）由驱动 `publish_tf: true`（`singlenav_launch.py:116`）供帧，
   但**我们无法验证其外参与 `base_link_fake` 的关系**，也未验证 `model_type: 0` 的视锥是否与 D435i 实际安装一致。
5. **静态 `map→odom` 与先验图 `origin` 是否真的自洽**：`maps/rmul2026.yaml:4` 的 `origin` yaw 为 0，而静态桥 yaw 为 -0.5；
   二者是否对应「同一物理起点」**只有真机标定能证明**（R3）。
6. **R7「`/map` 栅格跳变 vs `map→odom` 位姿跳变哪个更伤」的定量比较**：本次只做了机制分析（`slam_toolbox_common.cpp:255-259` + `static_layer` 订阅 `/map`），
   **未做实验**（本文禁止运行任何代码）。⇒ 对我们是否有益**未能确定**。
7. **我们 `slam_nav` 里 `map→odom` 的实际发布者**：`docs/debug_fastlio_cartographer.md:867` 记录的症状里明确写「**无 `map→odom`**」，
   而该模式用 `mapper` 槽 + `lio` 槽（`bringup_sim.launch.py:486-510`），**本应**由 slam_toolbox / cartographer 发布。
   ⇒ **这是一个我们自己的未解症状**（怀疑与 cartographer 零戳/`Ignored subdivision` 同源，`:870` 遗留②），本文**只能转述、无法定位**。
8. **我们 `slam_toolbox` 的两份参数对 `base_frame` 取值不一致（本文实测 grep）**：
   `src/rm_navigation/rm_navigation/params/mapper_params_online_async.yaml:15` = **`base_frame: livox_frame`**；
   `src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:15` = **`base_frame: base_link`**。
   而 `bringup_sim.launch.py:56` 在仿真里加载的是**后者**（`get_package_share_directory('slam_toolbox')/config/mapper_params_online_async_sim.yaml`）。
   ⇒ 仿真路径下取 `base_link`（与 `docs/tf_interface_contract.md:44` 的契约一致），但仓库里**仍留着一份 `livox_frame` 的旧值**；
   两者语义不同（`livox_frame` 相对 `base_link` 有 z≈0.3 偏移，见 `docs/cod_nav_2026_deep_dive.md:66`）。
   **本文未确证那份 `livox_frame` 是否在别的入口（如非 sim 的 launch）下生效** ⇒ **未确定**。
9. **上游 `small_point_lio` 在自己仓库（`ros2` 分支）与我们 `third_party` 里的快照是否完全一致**：本文只核对了 `LICENSE`/`package.xml`/`README` 三处元数据，
   **未做文件级 diff**（上游 2026-08-31 仍在更新，而我们手上的快照来自 COD 的 vendored 目录）。

---

### 附：本文所有引用的本地 `file:line` 与 URL（便于复核）

| 用途 | 引用 |
|---|---|
| 本地 COD 快照身份 | `git -C third_party/cod_nav_2026 log -1` → `0127e200`（2026-04-06）；`remote` = `https://gitee.com/codnavgation/cod_-rm2026_-navigation.git` |
| 静态 `map→odom`（单点，yaw −0.5） | `third_party/cod_nav_2026/src/cod_bringup/launch/singlenav_launch.py:67-89` |
| 静态 `map→odom`（多点，yaw 0） | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:95-116` |
| 裁剪盒内联参数 | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:35-49` |
| p2l（`/scan`）参数 | `third_party/cod_nav_2026/src/cod_bringup/launch/multiplenav_launch.py:65-84` |
| slam_toolbox 参数 | `third_party/cod_nav_2026/src/cod_bringup/params/mapper_params_online_async.yaml`（`:19,30,31-35`） |
| Nav2 参数（单点/多点） | `third_party/cod_nav_2026/src/cod_bringup/params/singlenav2_params.yaml`（483 行）、`multiplenav2_params.yaml`（476 行） |
| 定位 launch（只有 map_server） | `third_party/cod_nav_2026/src/cod_bringup/launch/localization_launch.py:44,76-79` |
| 先验图 | `third_party/cod_nav_2026/src/cod_bringup/maps/rmul2026.yaml:1-7`；PGM header `P5 255 177` |
| 航点 CSV | `third_party/cod_nav_2026/src/cod_bringup/wps/{go_gain,go_home,patrol_center,patrol_front,patrol_behind,preload_patrol}.csv` |
| `small_point_lio` 源码关键行 | `src/lidar_adapter/livox_pointcloud2.h:25-37`；`src/small_point_lio_node.cpp:26-27,61-62,64-69,90-96,101-102,178-187,197-211`；`src/small_point_lio/small_point_lio.cpp:43-156,167-177`；`src/small_point_lio/estimator.h:22`；`src/common/common.h:13-19`；`config/mid360.yaml:3-7,25,50`；`package.xml:7-9,22`；`README.md:3,5,7,90-95`；`LICENSE.txt:1-3` |
| slam_toolbox 语义 | `src/rm_localization/slam_toolbox/src/slam_toolbox_common.cpp:251-259`（本机同版本源码） |
| goal_approach_controller | `third_party/cod_nav_2026/src/goal_approach_controller/src/goal_approach_controller.cpp:96-126,143-146`；`goal_approach_controller_plugin.xml` |
| 我们的模式/槽位/契约 | `src/rm_nav_bringup/launch/bringup_sim.launch.py:52-56,124-181,303-406,444-468,486-510` |
| 我们的 nav2 参数 | `src/rm_navigation/rm_navigation/params/nav2_params_sim_{rpp,dwb,teb}.yaml`（rpp: `:1-40`(amcl)、`:178,182-235,241-242,252-288,357-362`） |
| 我们的 slam_toolbox 参数 | `src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml:13-17,28-32` |
| 我们的地图 | `src/rm_nav_bringup/map/RMUL2026.yaml:1-7`（`origin [-2.2,-3.15,0]`）；PGM `P5 249 173` |
| 我们的 TF 契约 | `docs/tf_interface_contract.md:17-22,44-53,73-77,126-128` |
| 我们的调试结算 | `docs/debug_fastlio_cartographer.md:867,869-870,872`（§10.2）、`:838-899`（§10） |
| 既有 COD 文档 | `docs/cod_nav_comparison.md:11-12,60,29`；`docs/cod_nav_2026_deep_dive.md:70-74,106-107,116-123,236,292-293,348,495`；`docs/cod_nav_2026_integration_plan.md:19,150-154,158-188`；`docs/cod_nav_nav_stack_inventory.md:16,59-60,95-101,132,168-175`；`docs/cod_nav_2026_params_review.md` §0.1/§0.4 |
| BBS 813022（small_point_lio 原理） | <https://bbs.robomaster.com/article/813022?source=4>（正文在 `window.__NUXT__` → `article.htmlContent`） |
| BBS 1882897（COD 2026 全开源技术报告） | <https://bbs.robomaster.com/article/1882897>（同上提取方式） |
| 上游仓库 | <https://github.com/Yancey2023/small_point_lio>；<https://api.github.com/repos/Yancey2023/small_point_lio>；镜像 <https://www.yanceymc.cn/gitea/Yancey/small_point_lio> |
| COD 提交史 | <https://api.github.com/repos/qza36/COD_NAV/commits?sha=rmul2026&per_page=100>；<https://gitee.com/api/v5/repos/codnavgation/cod_-rm2026_-navigation/commits> |
| COD 分支树 | <https://api.github.com/repos/qza36/COD_NAV/git/trees/rmul2026?recursive=1> |
| GitHub 季前 launch | <https://raw.githubusercontent.com/qza36/COD_NAV/rmul2026/nav_bringup/launch/slam.launch.py>；`.../nav.launch.py` |
| COD 导航仓（成品） | <https://gitee.com/codnavgation/cod_-rm2026_-navigation> |
| COD 决策仓 / 下位机 | <https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree/tree/feature%2Ffinish>；<https://gitee.com/cod_-control/rmcod2026_-sentry> |
| COD loopback sim | <https://github.com/qza36/COD_loopback_sim> |
