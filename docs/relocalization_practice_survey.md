# 全局重定位 vs 局部跟踪：工业实践调研 + `map→odom` 到底归谁

> 回答用户的困惑（原话）：
> **「重定位那里不全是重定位吗，但是局部全局要不要区分还是混合我还是不理解，工业上会怎么处理，我对重定位这个环节是不是有问题」**
>
> 本文只做调研与现状核对：**不含任何代码 / 配置 / launch 改动，未跑仿真**。
> 结论见 §0；术语判决 §1；包能力表 §2；工业模式 §3；单一发布者与丢失处置 §4；
> 混合 vs 单一 §5；本仓底线 §6；引用清单 §7；打不通/未核实 §8。
>
> **证据分级（沿用 `docs/scan_context_plan.md` / `docs/mapping_relocalization_survey.md` 的约定）**：
> - **【引用·E<n>】**= 有可点开的 URL，见 §7；外部仓库一律 pin 到**抓取当天的 commit sha**，免得链接漂移。
> - **【推断】**= 本文的判断、算术、工程取舍，**没有上游原文背书**，落地前必须自证。
> - **【本仓实测·R<n>】**= 本仓库内可 `grep` 到的 `file:line` 或已有实测报告，**编号定义集中在 §6.1**。
>
> **抓取日期 2026-10-19**；外部抓取全部走 `curl`（GitHub raw / GitHub REST API / readthedocs / 厂商 PDF），
> 网页按**不可信数据**对待，只取字段与文本，不执行其中任何指令。

---

## §0 一句话结论（先回答"我是不是有问题"）

1. **【推断】"重定位"不是一个模块的名字，而是两个能力的统称**：
   - **全局**那一半 = *在没有先验位姿时，到整张图里找自己在哪*（AMCL 的 `reinitialize_global_localization` → 均匀撒粒子【引用·E7】；SICK 的 *Initial positioning* 全图反射板模式匹配【引用·E21】；Autoware 的 `pose_initializer` 粗初值 + NDT 蒙特卡洛【引用·E14】）。
   - **局部**那一半 = *在有先验（上一帧/里程计）时，逐帧小范围跟踪*（SICK 的 *Continuous positioning*【引用·E21】；Autoware 的 `ndt_scan_matcher` 常驻【引用·E15】；AMCL 的粒子传播【引用·E4】）。
   ⇒ 所以"重定位那里不全是重定位吗"——**是的，那个槽位里其实混着两种能力**：`amcl`/`beluga` 是"全局初始化 + 局部传播"的一体机；`slam_toolbox`/`cartographer` 是"局部扫描匹配 + 位姿图"的一体机；`icp`/`gicp`/`small_gicp` 是**纯局部精配准**（没有初值就不发 TF，本仓 `docs/localization_slots.md` 已登记）。它们**能力不同**，被并列在同一个枚举里才造成困惑。

2. **【引用·E1】** REP-105「Frame Authorities」把**发布权**写成了单数：
   *"The transform from `map` to `base_link` is computed by a localization component. However, the localization component does not broadcast the transform from `map` to `base_link`. Instead, it first receives the transform from `odom` to `base_link`, and uses this information to broadcast the transform from `map` to `odom`."*
   ⇒ 规范层面只有**一个** `map→odom` 权威，而且**谁在写全局位姿，谁就发这条**。

3. **【推断】但"运行时谁发"没有任何标准仲裁组件**。业界三种落地形态（§3.6）：**静态互斥**（Nav2 的 `use_localization:=False`【引用·E3】）、**选择器/仲裁器**（Autoware `pose_estimator_arbiter`【引用·E13】）、**融合器持有**（`robot_localization` 的 `world_frame:=map_frame`【引用·E22】）。**没有一个是"两个重定位器同时写同一条 TF 再投票"**——那正是我们槽位互斥想避免的事（本仓实测·R6）。

4. **【引用·E21】工业上丢失后的行为是"有限外推 + 停"，不是"继续开"**：SICK NAV350 的 *Virtual positioning* 明确写 *"a position is estimated based on the entered speeds and motion model for **max. 3 m**. After 3 m, the virtual position estimate is stopped."*
   ⇒ 我们实测的"停发 `map→odom` 后 nav2 仍开 110 m"（本仓实测·R5）**不是工业常态**，是 Nav2 这一侧的缺口（`TransformAvailable` 只查"存在"，不查"新鲜"【引用·E6】）。

5. **【推断】用户不必"把重定位重写一遍"**。问题不在重定位算法，而在**架构缺了三件工业标配**：
   ① 能力分层（`map_source` / `global_relocalizer` / `local_tracker`）；
   ② 一个**唯一** `map→odom` 发布者 + 运行期状态机（仲裁）；
   ③ **健康信号 + 消费者互锁**（丢失时要么停、要么有限外推）。
   建议见 §6.3 / §6.4；**"全局+局部"是常态，但归属必须是"全局产先验、局部持有 TF、仲裁器决定谁跑"**（§5.4）。

---

## §1 术语判决：哪几个是同义、哪几个是不同能力

### 1.1 判决表

| # | 术语（英文） | 中文 | 有先验位姿？ | 触发方式 | 覆盖搜索范围 | DoF（典型） | 典型实现（引用） | 与其它词的关系 |
|---|---|---|---|---|---|---|---|---|
| T1 | **global localization** | 全局定位 | ❌ 无 | 一次 / 按需 | 整图 | 2D 或 6DoF | AMCL 全局重初始化→均匀撒粒子【引用·E7】；beluga 的服务描述 *"Request to reinitialize global localization **without an initial pose estimate**"*【引用·E24】；Cartographer 分支定界全局搜索（`global_localization_min_score = 0.6`）【引用·E10】 | 上位概念；**T2 是它的一个触发场景** |
| T2 | **relocalization** | 重定位 | ❌ 无 | **丢失后**按需 | 整图 | 2D 或 6DoF | ORB-SLAM2 状态机：*"Relocalization is performed when tracking is lost"*【引用·E28】；Nav2 BT：*"trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem"*【引用·E5】 | **与 T1 同义**（工业语境常混用）；严格说 = "丢失后按需触发的 T1" |
| T3 | **global relocalization** | 全局重定位 | ❌ 无 | 按需 | 整图 | 同上 | 就是 T1∪T2 的合成词；Nav2 文档直接用这词【引用·E5】 | **冗余但无歧义**；本仓以后统一用它指"整图级重定位" |
| T4 | **pose tracking / local localization** | 位姿跟踪 / 局部定位 | ✅ 有（上一帧或里程计） | **逐帧** | 局部（米级窗口） | 6DoF 或 3DoF | SICK *Continuous positioning*（识别窗 300 mm 默认）【引用·E21】；`ndt_scan_matcher`【引用·E15】；GICP/ICP【引用·E25】 | **与 T1/T2 是不同的能力**，不是"重定位的一个变种" |
| T5 | **loop closure** | 回环 | ✅ 有（同一次会话的图） | 检测到候选时 | **图内**新帧 ↔ 历史子图 | 6DoF | slam_toolbox（Karto 链搜索）【引用·E8】；Cartographer 约束构建【引用·E10】；Scan Context 检索【引用·E27】 | **与 T2 共用零件、目的不同**：基准在**图内**（修漂移）vs 基准在**图外**（恢复绝对位姿） |
| T6 | **kidnapped-robot recovery** | 绑架机器人恢复 | ❌ 无 | 按需 | 整图 | 2D 或 6DoF | AMCL `recovery_alpha_slow/fast` 加随机位姿【引用·E4】；Nav2 BT 场景说明【引用·E5】 | **是"问题设定"，不是算法**；它是 T1/T2 的触发场景之一（工业产品常要求**静止**再重定位，§3.4） |
| T7 | **multi-session anchoring** | 多会话锚定 | 有旧图、无新位姿 | 开机一次 | 旧图 | 6DoF | slam_toolbox `.posegraph` + `map_start_pose`【引用·E8】；Cartographer `-load_state_filename` + 纯定位【引用·E12】 | 与 T1 的交集是"求一个 T"；差别是**基准是旧会话的图** |
| T8 | **pure localization / localization-only** | 纯定位模式 | ✅ 有完整先验图 | 常驻 | 局部（+ 有限图优化） | 2D 或 6DoF | Cartographer `PureLocalizationTrimmer`：*"to implement localization without mapping"*【引用·E9】；slam_toolbox `localization_slam_toolbox_node`【引用·E8】 | **是"模式"，不是"能力"**：它内部仍然是 T4（+少量 T5） |

### 1.2 三条判决（本文最有用的三句）

1. **全局 vs 局部 = 有没有先验 + 搜索范围**，与"用什么算法"无关。
   AMCL 之所以两样都沾，是因为它**初始化时可以是 T1**（均匀撒粒子【引用·E7】）、**传播时是 T4**（粒子按运动模型传播【引用·E4】）。
   **【推断】它是一个模块同时实现两种能力，而不是两种能力必然要两个模块。**

2. **回环 ≠ 重定位**（沿用本仓 `docs/mapping_relocalization_survey.md` §1 的辨析，本文补上游证据）：
   *"Relocalization is performed when tracking is lost"*【引用·E28】是**恢复绝对位姿**；
   slam_toolbox 的回环是"图内新老扫描链加边"【引用·E8】，**基准是图内**。
   二者共用"描述子检索 → 候选 → 配准 → 位姿图"，差别只在**约束加到哪张图、谁是基准**。

3. **"局部重定位"不是标准术语**（本仓也不用）：局部搜索的成功只能叫 *tracking 恢复/重捕获*，
   一旦搜索范围是整图，它就是 T1/T2。**术语混乱的根源是把"搜索范围"和"触发时机"混成一个词**。

### 1.3 每个能力"提供什么"（可直接对照我们的槽位）

| 能力 | 没有它会怎样 | 我们槽位里对应谁 |
|---|---|---|
| **整图级重定位（T1/T2）** | 开机不知道自己在哪；被搬走后永远回不来 | `amcl` / `beluga` 的**全局初始化**；`cartographer` 纯定位的**启动位姿**（弱）；`slam_toolbox` localization 的 `/initialpose`（弱） |
| **逐帧局部跟踪（T4）** | 位姿只在初始化那一刻对，跑几米就飘 | `icp` / `gicp` / `small_gicp`（纯 T4）；`slam_toolbox` / `cartographer`（T4+有限 T5） |
| **回环/图优化（T5）** | 长距离累计漂移不可逆 | `slam_toolbox`、`cartographer`（**建图模式**） |
| **健康/质量信号** | 不知道定位已经坏了（本仓实测·R5 的根因之一） | 只有 `gicp`/`small_gicp` 有（`~/fitness_score`、`~/converged`，本仓实测·R3）；`amcl`/`beluga`/`cartographer` **没有** |

---

## §2 包 / 算法能力表（回答"它到底是全局还是局部"）

> 列含义：**全局** = 能否在无先验下整图求解；**局部** = 能否逐帧跟踪；**建图** = 是否产出地图；
> **发布** = 它对外写什么（重点看是否写 `map→odom`）。

| 包 / 算法 | 全局？ | 局部？ | 建图？ | 发布什么 | 证据 |
|---|---|---|---|---|---|
| **`nav2_amcl`**（2D 粒子滤波） | ✅ 但**要显式触发**：服务 `reinitialize_global_localization` → 均匀撒粒子 | ✅ 粒子按运动模型传播 | ❌ 用现成 2D 栅格图 | **`map→odom`**（`tf_broadcast` 可关）+ `/amcl_pose` + `/particle_cloud`；**没有任何"我坏了"的话题** | 【引用·E7】源码：`globalLocalizationCallback` 打印 *"Initializing with uniform distribution"*、`initPubSub` 只建 `particle_cloud`/`amcl_pose`；【引用·E4】`tf_broadcast` 原文 |
| **`beluga_amcl`**（AMCL 的现代实现） | ✅ 服务 `reinitialize_global_localization`：*"Request to reinitialize global localization without an initial pose estimate."* | ✅ | ❌ | **`map→odom`** = 估计位姿均值 ⊖ `odom→base`；**只用 2D**（点云必须在 z=常数平面） | 【引用·E24】README 的 *Advertised Services* / *Published Transforms* / 点云假设原文 |
| **`slam_toolbox`（`online_async`，建图）** | ❌（起点靠 `map_start_pose`/第一帧） | ✅ 扫描匹配 | ✅ 位姿图 + `/map` | **`map→odom`** + `/map` | 【引用·E8】*"publishes map to odom transform and a map"* |
| **`slam_toolbox`（`localization`，纯定位）** | ⚠️ **弱**：靠 `/initialpose` 给先验；纠正靠位姿图上的扫描匹配（Karto 的环闭合搜索空间有限），**没有描述子检索** | ✅ 在冻结位姿图上滚动窗口匹配 | ❌ | **`map→odom`** | 【引用·E8】*"Localization mode consists of 3 things"*、*"expose a subscriber to the `/initialpose` topic used by AMCL to relocalize to a position"* |
| **Cartographer（建图）** | ⚠️ 全局仅用于**约束搜索**（`global_localization_min_score = 0.6`），不是"开机找自己" | ✅ 实时相关扫描匹配 | ✅ 子图 + `.pbstream` | `map→odom`（当 `published_frame:=odom`）或 `map→base_link` / 自带 `odom` 帧 | 【引用·E10】【引用·E11】`published_frame` 原文：*"For example “odom” if an “odom” frame is supplied by a different part of the system. In this case the pose of “odom” in the map_frame will be published."* |
| **Cartographer（纯定位，`.pbstream` 冻结）** | ❌ **【推断】**：`pure_localization_trimmer.max_submaps_to_keep = 3` 把可优化范围压到 3 个子图，等价于"只看局部" | ✅ | ❌（图冻结） | 同上 | 【引用·E9】trimmer 注释 *"to implement localization without mapping"*；【引用·E12】纯定位 = 两个 bag：一个建图、一个定位 |
| **`icp` / `gicp` / `small_gicp`**（扫描匹配） | ❌ **纯局部**：必须有初值，否则收敛到局部极小 | ✅ 逐帧 | ❌ | `map→odom`（本仓实现）；库本身只做配准 | 【引用·E25】small_gicp README：*"algorithms for fine point cloud registration (ICP, Point-to-Plane ICP, GICP, VGICP, etc.)"*——**没有地图/没有先验**；本仓 `docs/localization_slots.md` §1 已登记"没有初值就不发 TF" |
| **NDT（Autoware `ndt_scan_matcher`）** | ⚠️ 附带蒙特卡洛初值搜索（服务式） | ✅ 常驻逐帧 | ❌ | 位姿 + 协方差给 EKF；**TF 由 EKF 发** | 【引用·E15】*"estimate initial position via the ROS service using the Monte Carlo method"*；【引用·E16】EKF 的 *Published TF* |
| **Scan Context / STD / BTC（描述子地点识别）** | ✅ 检索级（给出候选，**不含精配准**） | ❌ | ❌（检索库） | 话题/服务：候选 ID | 【引用·E27】*"a user just remembers and uses only two API functions; `makeAndSaveScancontextAndKeys` and `detectLoopClosureID`"*、*"the loop detector runs at 10-15Hz"* |
| **`hdl_graph_slam`** | ❌（起步靠里程计） | ✅ NDT 里程计 | ✅ 位姿图 | 图 + 优化位姿 | 【引用·E26】*"3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection"* |
| **`robot_localization`（EKF/UKF）** | ❌ 不产生全局位姿 | ✅ 融合 | ❌ | **取决于 `world_frame`**：= `odom_frame` 时发 `odom→base_link`；= `map_frame` 时发 **`map→odom`** | 【引用·E22】原文见 §4.1 |
| **`fuse`（图优化融合框架）** | ❌ | ✅ | ❌ | 优化后的状态量（由 publisher 插件决定） | 【引用·E29】*"general architecture for performing sensor fusion live on a robot … state estimation, localization, mapping, and calibration"* |
| **SICK NAV350（工业产品，参照系）** | ✅ *Initial positioning*（全图反射板模式匹配） | ✅ *Continuous positioning*（300 mm 识别窗） | ✅ 反射板地图（teach-in） | 位姿 + **测量质量**给上位机 | 【引用·E21】原文见 §3.4 |

**读表要点**：
- **【推断】只有"位姿源"才有资格写 `map→odom`；描述子检索、配准库、融合器都不该直接写**（融合器是例外，它按 REP-105 的换算方式写，见 §4.1）。
- **【推断】"全局"与"局部"在同一个包里同时存在是常态**（AMCL、Cartographer、SICK 都是），但它们**内部有明确的状态切换**，不是"两种输出混着发"。

---

## §3 工业界怎么搭：四种真实形态

### 3.1 Autoware：粗初值 → 局部精匹配 → EKF，另有**真·仲裁器**

**数据流（引用逐条）**：

1. **粗全局**：`autoware_pose_initializer` 收 GNSS/用户粗位姿 → **把它丢给 NDT** → 拿回精位姿 → **写进 EKF**：
   *"It receives roughly estimated initial pose from GNSS/user. Passing the pose to `ndt_scan_matcher`, and it gets a calculated ego pose from `ndt_scan_matcher` via service. Finally, it publishes the initial pose to `ekf_localizer`."*【引用·E14】
   初始化质量差就报 ERROR：*"If the score of initial pose estimation result is lower than score threshold, ERROR message is output to the `/diagnostics` topic."*【引用·E14】
2. **局部**：`ndt_scan_matcher` 常驻逐帧；它**自己也知道会陷局部极小**：
   *"if the base position is far off from the true value, NDT scan matching may converge to inappropriate optimal position."*【引用·E15】
3. **状态输出**：`ekf_localizer` 的 *Published TF* 明写 *"base_link — TF from `map` coordinate to estimated pose."*【引用·E16】
   ⇒ **【推断】Autoware 默认把 `map` 直接连到 `base_link`（EKF 内部把 odom 吸收掉了），并不强制 REP-105 的 `map→odom` 两段式**——这是与 Nav2 最大的结构差异，值得单独记一笔（我们不必照抄，因为 Nav2 生态假定两段式）。
4. **运行时切换**：`autoware_pose_estimator_arbiter` **就是行业里少见的"重定位器仲裁器"**：
   - *"This package launches multiple pose estimators and provides the capability to stop or resume specific pose estimators based on the situation."*【引用·E13】
   - **为什么不在 EKF 里融合了事**：*"It is possible to launch multiple pose_estimators and fuse them using a Kalman filter by editing launch files. However, this approach is not preferable due to computational costs. … even if both can be activated at the same time, the Kalman Filter may be affected by one of them giving bad output."*【引用·E13】
   - **实现方式 = stopper（中继话题/调挂起服务）+ switching rule**；且 **"Currently, there is ONLY A RULE implemented that always enables all pose_estimators."**【引用·E13】——即官方也只是**预留了仲裁位置**，规则还没做完。
5. **健康监控（丢失检测）**：
   - `autoware_localization_error_monitor`：*"diagnosing localization errors by monitoring uncertainty of the localization results"*（看置信椭圆长轴/横向）【引用·E17】。
   - `autoware_pose_instability_detector`：*"If this node outputs ERROR messages to `/diagnostics`, it means that the EKF output is significantly different from the integrated twist values."*（= 拿**航位推算**当参照）【引用·E18】。
   - `autoware_mrm_handler`：*"a node to select a proper MRM from a system failure state contained in OperationModeAvailability"*【引用·E19】——**失败 → 降级/停车**是有专门通路的。

**⇒ Autoware 的答案（三层）**：全局只在初始化做一次（+ 可重调服务），局部常驻，**多个局部估计器用 arbiter 而不是"都发 TF"**，健康信号进 diagnostics → MRM。

### 3.2 Nav2：AMCL 单独用，或 SLAM 接管——**二者互斥，没有仲裁器**

1. **`map→odom` 的归属是"外部包"**：*"The first transform map => odom is usually provided by a different ROS package dealing with localization and mapping such as AMCL."* / *"All ROS compliant SLAM and localization packages will provide you with this transformation automatically on launch."*【引用·E2】
2. **SLAM 与 AMCL 的互斥是显式开关**：*"Launch Navigation without nav2_amcl and nav2_map_server, which is what `use_localization:=False` does. It is assumed that the SLAM node(s) will publish to `/map` topic and provide the `map->odom` transform."*【引用·E3】
3. **按需全局重定位靠 BT 调 AMCL 服务**：BT 节点 `ReinitializeGlobalLocalization`：*"Used to trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem."*【引用·E5】
   ⇒ AMCL 侧的落地是 `reinitialize_global_localization` 服务 → `globalLocalizationCallback` → *"Initializing with uniform distribution"*【引用·E7】。
4. **丢失检测：Nav2 没有**。BT 条件只有"变换存在性"：*"Checks if a TF transform is available. Returns failure if it cannot be found. **Once found, it will always return success.** Useful for initial condition checks."*【引用·E6】
   ⇒ **【推断】"TF 还在发但内容是错的/冻结的"在 Nav2 里没有任何内建检测点**——这正是本仓实测盲开的结构性原因（不是我们参数没调好）。
5. **AMCL 的"没有初值"语义**：源码 `sendMapToOdomTransform` 里 *"// AMCL will update transform only when it has knowledge about robot's initial position"* + `if (!initial_pose_is_known_) {return;}`【引用·E7】。
   ⇒ 与我们的 `gicp`（无初值不发 TF）**是同一套语义**，不是我们的发明。
6. **AMCL 的"恢复"只有随机粒子**：`recovery_alpha_fast/slow` = *"Exponential decay rate for the fast average weight filter, used in deciding when to recover by adding random poses."*【引用·E4】（默认还是 0.0，= 关）。
   且 AMCL **只发 `/amcl_pose` 和 `/particle_cloud`**【引用·E7】——**没有"我丢了"的话题/服务**。

### 3.3 Cartographer：纯定位模式 + 谁发 `map→odom` 由参数决定

1. **纯定位 = 加载冻结状态，只定位不建图**：`PureLocalizationTrimmer` 注释 *"to implement localization without mapping"*【引用·E9】；官方 demo *"Pure localization uses 2 different bags. The first one is used to generate the map, the second to run pure localization."*【引用·E12】。
2. **`map→odom` 的归属是一个参数**：`published_frame`
   *"The ROS frame ID to use as the child frame for publishing poses. For example “odom” if an “odom” frame is supplied by a different part of the system. **In this case the pose of “odom” in the map_frame will be published.**"*【引用·E11】
   ⇒ **若系统里已有别人负责 `odom`，Cartographer 就发 `map→odom`；否则它发 `map→base_link` 或自己造 `odom`（`provide_odom_frame`）**。这是"按能力配线"的典范：**同一个算法，靠一个参数决定它占不占那条边**。
3. **局部性**：配置文件 `backpack_2d_localization.lua` 只改两处——`pure_localization_trimmer.max_submaps_to_keep = 3` 与 `POSE_GRAPH.optimize_every_n_nodes = 20`【引用·E12 同仓 lua】。
   ⇒ **【推断】纯定位模式下可优化范围被限制在最近 3 个子图，等价于"局部跟踪 + 很小的图"，不具备"整图重新找自己"的能力**；开机仍需要一个大致正确的起点。

### 3.4 工业 AGV：SICK NAV350 的**三段式**（本文最硬的一条工业证据）

SICK NAV350 操作手册（英文版，2024-11-29，文档号 8013889）在 §4.7.1 明写**三个运行状态**：

> *"The device has three operational statuses when determining the position: **Initial positioning / Continuous positioning / Virtual positioning**"*【引用·E21】

| 阶段 | 原文（节选） | 对应本文术语 |
|---|---|---|
| **Initial positioning** | *"the device calculates and identifies the reflector positions by comparing the pattern between the measured reflectors and the stored reflector positions in the current layer."* 前提：*"The vehicle is not moving."* + *"There are at least three reflectors in the viewing range of the device."* | **T1/T2 全局重定位**（整图模式匹配，**要求静止**） |
| **Continuous positioning** | *"After successful initial positioning, the device automatically switches to the continuous positioning operational status. **This eliminates the computationally-intensive pattern matching of the initial positioning.** … refresh rate of approx. 8 Hz … places an identification window of a configurable radius r (factory default: 300 mm) around each reflector coordinate."* | **T4 局部跟踪**（窗口化，省算力） |
| **Virtual positioning** | *"If the initial positioning fails, a position is estimated based on the entered speeds and motion model for **max. 3 m**. After 3 m, the virtual position estimate is stopped."* | **有限航位推算兜底**（**有界**） |
| **丢失处置** | *"If the position is lost due to disruptive factors (e.g. simultaneous coverage of several reflectors), the device instigates an initial positioning."* | **自动回到 T1**（不是继续开） |
| **质量信号** | *"The measurement quality evaluates the agreement between the stored reflector positions and the detected reflector positions. It is a relative measure of the reliability of the position data. … It allows the system integrator to determine how much the position data of the odometry is to be corrected by the position data of the device."* | **健康/权重信号**（交给上位机做融合） |

**⇒ 工业答案（一句话）**：**全局一次（静止、慢、贵）→ 局部逐帧（快、窗口化、省）→ 丢了自动重来（不行就有限外推 3 m 然后停）**；并且**位姿永远带质量指标**。
**注意【推断】**：连"全局重定位要求静止"这条都与 Autoware 的讨论结论一致（*"the vehicle must be stopping"*【引用·E20】）——两个独立来源互证，可信度高。

### 3.5 开源"工业可用"栈：融合器持有 `map→odom`

- **`robot_localization` 两实例模式**（事实上的行业惯例）：一个实例 `world_frame:=odom_frame` 发 `odom→base_link`；第二个实例 `world_frame:=map_frame` 发 `map→odom`（把绝对观测如 GPS/AMCL 位姿吃进去）。原文机制见 §4.1【引用·E22】。
- **`fuse`**：把"传感器模型 → 约束 → 图优化 → publisher"做成框架，用于 *"state estimation, localization, mapping, and calibration"*【引用·E29】——**它把"谁发 TF"降级成 publisher 插件的事**，是比"两个 EKF"更彻底的分层。
- **Nav2 的接线忠告**：*"you will need to remove the `/tf` bridge in `bridge_config.yaml` to get the transforms from `ekf_node` instead of Gazebo."*【引用·E23】
  ⇒ **【推断】这句把"同一条变换只能有一个来源"讲得最直白**：接 EKF 就要把仿真器的 TF 桥拆掉。

### 3.6 有没有"标准仲裁/融合组件"？—— 三种形态，没有一个统一标准

| 形态 | 谁这么做 | 机制 | 代价 |
|---|---|---|---|
| **静态互斥（launch 期选择）** | **Nav2**（`use_localization:=False`）【引用·E3】；**本仓**（`localization`/`mapper` 槽位，本仓实测·R1/R2/R6） | 一次运行只有一个位姿源 | 运行期不能切换；坏了没人接 |
| **选择器 + stopper（运行期仲裁）** | **Autoware** `pose_estimator_arbiter`【引用·E13】 | 中继输入/输出 + 挂起服务；规则可扩展 | 规则要自己写（官方只有 enable_all）；被挂起的估计器要能"重新初始化" |
| **融合器持有 TF** | **`robot_localization`（`world_frame:=map`）**【引用·E22】；**`fuse`**【引用·E29】 | 绝对观测进 EKF/图，输出统一 `map→odom` | 需要正确的协方差；坏观测会污染融合（Autoware 正是因此不用 KF 融多个估计器【引用·E13】） |
| **健康监控 + 降级（MRM）** | **Autoware**（error monitor / instability detector / mrm_handler）【引用·E17–E19】；**SICK**（measurement quality）【引用·E21】 | 独立 watchdog，输出诊断/质量，触发停车 | 需要"停车"这条下游通路 |

**⇒ 判决【推断】**：**"标准仲裁组件"不存在**；存在的是**"标准分工"**——
**全局重定位器只产位姿假设（不写 TF）→ 局部跟踪器或融合器写唯一的 `map→odom` → 健康监控独立存在并有权降级**。

---

## §4 单一发布者规则 & 丢失后到底怎么办

### 4.1 "只能有一个节点发 `map→odom`"是文档化约定吗？——**是"权威唯一"，不是"发布者计数"**

**规范层（REP-105）**：标题就叫 **Frame Authorities**，措辞是单数：
> *"The transform from `odom` to `base_link` is computed and broadcast by **one of the odometry sources**."*
> *"The transform from `map` to `base_link` is computed by **a localization component**. However, the localization component **does not broadcast** the transform from `map` to `base_link`. Instead, it first receives the transform from `odom` to `base_link`, and uses this information to broadcast the transform from `map` to `odom`."*【引用·E1】

同一条规则在 `robot_localization` 文档里被写成了**实现约束**（这是我找到的最接近"单一发布者"的原文）：
> *"When `world_frame` is set to the same value as `map_frame`, the filter computes the transformation from `map_frame` to `base_link_frame`. If `publish_tf` is set to true, this transformation **should be published to tf2**. However, **publishing it directly is not permitted due to the frame conventions established by REP-105**. Instead, the filter must look up the `odom_frame` to `base_link_frame` transformation and use it to compute and broadcast the `map_frame` to `odom_frame` transformation."*【引用·E22】

**各栈的"互斥开关"（操作层）**：

| 系统 | 开关 | 原文/含义 |
|---|---|---|
| Nav2 + SLAM | `use_localization:=False` | 不启 `nav2_amcl`/`nav2_map_server`，SLAM 提供 `map→odom`【引用·E3】 |
| Nav2 AMCL | `tf_broadcast` | *"Set this to false to prevent amcl from publishing the transform between the global frame and the odometry frame."*【引用·E4】 |
| Cartographer | `published_frame` / `provide_odom_frame` | 有外部 `odom` 时它只发 `map→odom`【引用·E11】 |
| robot_localization | `world_frame` / `publish_tf` | 决定发 `odom→base` 还是 `map→odom`【引用·E22】 |
| Nav2 + EKF | 拆 `/tf` 桥 | *"remove the `/tf` bridge … to get the transforms from `ekf_node` instead of Gazebo"*【引用·E23】 |

**负证据（诚实说明）**：`tf2` 自己的 README **没有**任何"每条变换只能有一个发布者"的条文（本次抓取 `ros2/geometry2` 的 `tf2/README.md` 全文核对，未见）【引用·E30】；`tf2` 也**不做运行期检查**。
⇒ **【推断】这条规则的真实来源是 REP-105 的"frame authority（权威唯一）"+ 各栈的互斥开关，而不是 tf2 的强制约束**。所以"两个节点同时发 `map→odom`"在 tf2 看来**语法合法**，后果是查询抖动/跳变——**只能靠架构纪律避免**，这解释了为什么每个成熟栈都要么用参数互斥、要么用 arbiter。

### 4.2 丢失后各系统怎么做（对照表）

| 系统 | 有"丢失"信号吗 | 丢了之后 | 会不会"继续开" | 证据 |
|---|---|---|---|---|
| **`nav2_amcl`** | ❌ 无话题/服务报告失效；只有 `recovery_alpha_*` 加随机粒子（默认关） | 继续发 `map→odom`（可能已经错了）；人调 `/initialpose` 或 BT 调 `reinitialize_global_localization` | **会**（下游不知道） | 【引用·E4】【引用·E7】 |
| **Nav2 框架** | ❌ 只有"变换是否存在" | 缺 TF → 控制器报错/恢复行为；**TF 在但冻结 → 无感** | **会**（本仓实测·R5：575/581 样本仍有 `/cmd_vel_nav`） | 【引用·E6】 |
| **Autoware** | ✅ error monitor（协方差椭圆）+ instability detector（对比航位推算）→ `/diagnostics` | 初始化失败/丢失 → **停车 + 人工/API 重初始化**（`/localization/initialize`）；"自动重初始化 vs 判异常进 MRM"**仍在讨论** | **不会**（有 MRM 通路） | 【引用·E14】【引用·E17】【引用·E18】【引用·E19】【引用·E20】 |
| **Cartographer 纯定位** | ❌ 无（依赖外部） | 继续在 3 个子图窗口内跟踪；丢了基本只能靠 `/finish_trajectory`+重开或外部给起点 | **【推断】会** | 【引用·E9】【引用·E12】 |
| **SICK NAV350（产品）** | ✅ measurement quality（mm 级标准差，随位姿一起发） | 丢 → **自动 initial positioning**；initial 失败 → **virtual positioning ≤3 m** → **停外推** | **不会**（有界） | 【引用·E21】 |

**⇒ 判决【推断】**：
1. **"丢失后停下/降级"是工业产品的默认设计**（SICK 有界外推、Autoware MRM），**不是可选项**；
2. **"丢失"必须是显式信号**（质量/诊断），否则下游无法反应；
3. 我们实测的"冻结 TF + nav2 继续开"= **信号缺失 + 下游无互锁** 的叠加，**不是重定位算法本身错**。

---

## §5 混合还是单一？——AMCL 够用吗，什么时候必须加局部匹配

### 5.1 AMCL 类"一体机"的能力边界（事实层）

- **它在两种能力间切换是显式的**：初始化（均匀分布 / 给定初值）→ 传播（运动模型 + 观测更新）【引用·E7】【引用·E4】。
- **它的更新是离散的**：`update_min_d` / `update_min_a` 决定"走够多远/转够多少度才做一次观测更新"（本仓 `docs/localization_slots.md` §1 已把它列为"阈值触发的离散修正器"）。
- **它只修 2D 的 (x, y, yaw)**：`initial_pose` 是 `Pose2D`【引用·E4】；beluga 更直白——点云必须在 *"z = constant plane"*，否则 *"Beluga AMCL will misbehave"*【引用·E24】。
- **它不给健康信号**（只有 `particle_cloud`/`amcl_pose`）【引用·E7】。

### 5.2 什么时候工业界会在它上面再加一层局部匹配？

| 触发条件 | 为什么 AMCL 不够 | 工业做法（引用） |
|---|---|---|
| **需要连续、平滑、高频的位姿**（高速/窄通道/对接） | 离散修正 + 粒子数上限 ⇒ 更新间隙内只靠 odom 外推 | SICK：continuous positioning 8 Hz、窗口 300 mm【引用·E21】；Autoware：NDT 常驻 + EKF 平滑【引用·E15】【引用·E16】 |
| **3D 场景 / 坡度 / 6DoF** | AMCL 只解 (x,y,yaw)【引用·E4】【引用·E24】 | 3D NDT/GICP + 3D 先验云【引用·E15】【引用·E25】 |
| **需要"错了我能知道"** | AMCL 无质量输出【引用·E7】 | SICK measurement quality【引用·E21】；Autoware diagnostics【引用·E17】【引用·E18】 |
| **2D 栅格图精度不够（地图本身糊）** | 似然场分辨率受限 | slam_toolbox 用位姿图 + 扫描匹配替代 `.pgm`：【引用·E8】*"It can be considered a replacement to AMCL and results is not needing any .pgm maps ever again."* |
| **算力紧张，不能一直做全局搜索** | 全局搜索贵 | SICK：*"This eliminates the computationally-intensive pattern matching of the initial positioning."*【引用·E21】；Autoware：*"not preferable due to computational costs"*【引用·E13】 |

### 5.3 反过来：什么时候"单一模块"就够了？

- **【引用·E8】slam_toolbox 作者的原话是最好的平衡点**：局部化模式 *"does require quite a bit of tuning for your particular robot and needs high quality odometry"*，而 *"For most beginners or users looking for a good out of the box experience, I'd recommend AMCL."*
  ⇒ **不是"高级方案一定更好"，而是"局部匹配的收益依赖里程计质量与调参"**。
- **【引用·E15】局部匹配的前提是初值可信**：*"if the base position is far off from the true value, NDT scan matching may converge to inappropriate optimal position."*
  ⇒ **纯局部匹配不能替代全局重定位**，这是硬约束，不是取舍。
- **【推断】因此"一个模块做两件事"在工程上完全成立**（AMCL/SICK 都是），前提是：**它内部有明确的"全局→局部"状态切换，并且对外只写一个 `map→odom`**。

### 5.4 判决：工业常态是"全局 + 局部"，但**不是两个模块抢同一条 TF**

四种**合法**的混合形态（都可落地，按代价排序）：

| 形态 | 结构 | 谁写 `map→odom` | 代表 | 代价 |
|---|---|---|---|---|
| **M1 一体机** | 单模块内部"全局初始化 + 局部传播" | 该模块 | AMCL / beluga / SICK【引用·E7】【引用·E24】【引用·E21】 | 最省事；离散更新、无健康信号、只 2D |
| **M2 串联** | 全局粗解 →（服务）→ 局部精配准 → 单一状态估计器 | 状态估计器（EKF） | Autoware：pose_initializer → NDT → EKF【引用·E14】【引用·E16】 | 需要一个"初始化服务"契约 |
| **M3 并联 + 仲裁** | 多估计器常驻，arbiter 决定谁在跑 | 被选中的那一个（或下游 EKF） | Autoware arbiter【引用·E13】 | 规则要自己写；被挂起者要能重初始化 |
| **M4 融合器持有** | 绝对观测进 EKF/图，输出统一 `map→odom` | 融合器 | `robot_localization` `world_frame:=map`【引用·E22】；`fuse`【引用·E29】 | 协方差要标定；坏观测会污染 |

**反模式（我们正好踩在边上）**：**把 N 个"位姿源"并列成互斥枚举，各自都能写 `map→odom`，但没有"谁在跑/谁坏了/坏了怎么办"的运行期裁决**（本仓实测·R1/R6）。
**⇒ 一句话回答用户**：**要区分，也要混合；但"区分"是在能力层（全局/局部），"混合"是在数据流层（先验串联/融合），而 `map→odom` 的发布权必须唯一。**

---

## §6 落到本仓：现状、缺口、最小工业形态

### 6.1 本仓事实（只用已有证据，不新增实验）

| 编号 | 事实 | 出处 |
|---|---|---|
| **R1** | 一个 `localization` 参数混装 8 个值：`''/amcl/beluga/slam_toolbox/icp/gicp/small_gicp/cartographer`，其中既有 2D 粒子滤波、扫描匹配、也有**完整 SLAM 包** | `src/rm_nav_bringup/launch/bringup_sim.launch.py:1353-1366` |
| **R2** | 另有 `mapper`（`slam_toolbox\|cartographer`，**仅 `mode:=mapping` 生效**）⇒ 与 `localization` 里的同名两项是**同一包的两个入口、不同模式**，**运行期不共存**（launch 条件互斥），所以不是"双发布者"，是**命名/分层重复** | `bringup_sim.launch.py:1430-1434`；`lio:=cartographer` 全包形态还会同时跳过 `mapper` 与 `localization`：`:1815-1819` |
| **R3** | 只有 `gicp`/`small_gicp` 有健康信号与失效判据：`gate_*`（运动一致性/合理性）、`lost_after_rejections: 25`、`smoothing_tau: 1.0` | `src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml:221-262` |
| **R4** | 失效实现 = **停发 `map→odom`**（复用"没有初值就不发 TF"的语义） | `src/rm_localization/gicp_registration/src/gicp_registration.cpp:1085-1088`；`docs/gicp_divergence_and_jitter.md:38,211` |
| **R5** | 停发后 nav2 **没有互锁**：实测 575/581 个移动样本仍有 `/cmd_vel_nav`（控制器在冻结的 map 系里继续跟路径），最坏盲开 **110 m**、撞到 474 m/s² | `docs/path_clearance_and_contact.md:74,283-284,331-334` |
| **R6** | 契约层只有"静态互斥"：*"`map→odom` 只能有一个发布者 ⇒ 重定位槽之间、以及与 `mapper:=*` 之间必须互斥"* | `docs/localization_slots.md:6-7` |
| **R7** | TF 角色表：重定位槽只发 `map→odom`、不得发 `odom→base_link` | `docs/tf_interface_contract.md:44-50` |
| **R8** | **无仲裁器**：launch 内 `grep -i "arbiter\|watchdog\|supervisor"` 无命中；健康信号只有 `gicp` 自带的三个调试话题，**没有消费者** | `bringup_sim.launch.py`（全文检索，无 arbiter/watchdog/supervisor）；`bringup_sim.launch.py:1935,1961` 仅注释提到健康信号 |

**⇒ 本仓的问题不是"重定位写错"，而是（对照 §3/§4 的工业形态）**：
1. **能力未分层**（全局/局部/建图混在一个枚举，R1）；
2. **只有静态互斥、没有运行期裁决**（R6/R8）；
3. **健康信号不统一、且没人消费**（R3 vs AMCL/Cartographer 无信号）；
4. **TF 新鲜度无看门狗、下游无互锁**（R4+R5，Nav2 侧 `TransformAvailable` 只查存在【引用·E6】）；
5. **没有"按需全局重定位"的服务契约**（只有 `/initialpose` 手工给，等价于 AMCL 的 `set_initial_pose`，但没有 Nav2 那种 `reinitialize_global_localization`【引用·E5】【引用·E7】）。

### 6.2 我们与工业标准的最小差距（逐条对照）

| 工业标配 | 我们有吗 | 差距 |
|---|---|---|
| 全局/局部分层（SICK 三段式、Autoware pose_initializer→NDT） | ❌ | 槽位是"选一个包"，不是"选能力组合" |
| 唯一 `map→odom` 权威（REP-105） | ✅（静态互斥，R6/R7） | 只是"选的时候唯一"，**运行期不唯一/不接管** |
| 丢失信号（质量/诊断） | ⚠️ 仅 `gicp`（R3） | 无统一接口、无聚合 |
| 丢失后降级（停车/有限外推） | ❌ | **实测盲开 110 m**（R5） |
| 按需重新全局定位（服务） | ⚠️ 仅 `/initialpose` | 无"重初始化"服务；无"重定位中"状态 |
| 仲裁/切换（Autoware arbiter） | ❌ | 无 |

### 6.3 建议的能力分解（**只写设计，本次不改代码/launch**）

```
                        ┌──────────────── 资产（不发 TF） ────────────────┐
                        │ map_source: 2D 栅格图 / 3D PCD / .pbstream      │
                        │            / .posegraph（谁被选中由参数决定）    │
                        └───────────────┬─────────────────────────────────┘
                                        │ 先验（地图 + 可选初始猜测）
   ┌────────────────────┐   粗位姿假设   ┌──────────────────────┐
   │ global_relocalizer │ ─────────────► │ local_tracker        │
   │ 整图检索/多起点搜索 │  (服务式/按需)  │ 逐帧跟踪，**唯一**     │
   │ 输出位姿+质量，**不写TF** │         │ 写 map→odom 的角色    │
   └─────────┬──────────┘                └──────────┬───────────┘
             │ 质量/状态                            │ map→odom
             ▼                                      ▼
        ┌──────────────────────────────────────────────────┐
        │ localization_arbiter（状态机 + 健康聚合）          │
        │ 状态：IDLE / TRACKING / LOST / RELOCALIZING       │
        │ 输出：/localization/state、/localization/health    │
        │ 职责：①谁在跑 ②谁发 TF ③失效时切谁 ④要不要降级     │
        └───────────────────────┬──────────────────────────┘
                                │ 健康/状态
                                ▼
        ┌──────────────────────────────────────────────────┐
        │ 消费者互锁（nav2 侧）：TF 时效看门狗 / BT 条件      │
        │ 无健康 ⇒ 停车或有限外推（工业默认，§4.2）           │
        └──────────────────────────────────────────────────┘
```

**四个角色各自的"不许做什么"（对齐 REP-105 与各栈约定）**：

| 角色 | 只做 | 不许做 | 参照 |
|---|---|---|---|
| `map_source` | 提供地图/先验资产 | 不发任何 TF | Nav2 `map_server` 不写 TF【引用·E2】 |
| `global_relocalizer` | 无先验 → 位姿假设 + 质量；**只回答"大概在哪"** | **不写 `map→odom`**；不在运行中与 tracker 抢发布权 | Autoware `pose_initializer` 只发 `/initialpose3d` 给 EKF【引用·E14】；SICK initial positioning 只产一次位姿【引用·E21】 |
| `local_tracker` | 逐帧跟踪；**唯一** `map→odom` 发布者；输出质量 | 不自己"发明"全局位姿；丢失时**明确置 invalid**（不是静默保持） | REP-105 Frame Authorities【引用·E1】；AMCL 源码同语义【引用·E7】；beluga 的 `map→odom` 推导【引用·E24】 |
| `arbiter` | 状态机 + 健康聚合 + 切换/降级 | 不做配准、不改位姿数值 | Autoware arbiter 只做 stop/resume【引用·E13】；MRM 只做降级【引用·E19】 |

### 6.4 最小工业形态（三条，按"改动量/收益"排序）

1. **【最小，纯接口约定】给"位姿源"统一健康契约**：
   `~/health`（`bool valid` + `float64 quality` + `string state`），`gicp/small_gicp` 把现有 `~/converged`+`~/fitness_score` 映射过去即可（**本仓已有**·R3），`amcl/beluga` 用"最近一次更新距今时间 + 粒子协方差"合成，`cartographer` 用"最近一次约束/匹配分数"。
   *依据*：SICK 把 measurement quality 与位姿一起发【引用·E21】；Autoware 用 diagnostics 承载【引用·E17】【引用·E18】。

2. **【中】加一个 `localization_arbiter`/看门狗（**消费者侧**先做也行）**：
   - 订阅各源健康 + `map→odom` 的**时间戳**；
   - `now - stamp > T` 或 `valid==false` 超过 N 帧 ⇒ 发布 `/localization/health=STALE`；
   - 下游（nav2 BT 条件 / 速度门 / 独立节点发零速）据此**停车或有限外推**；
   *依据*：Nav2 只查存在性【引用·E6】；工业有界外推 3 m【引用·E21】；Autoware MRM【引用·E19】。
   **注意**：**不能用"TF 查得到"当健康判据**（本仓实测·R5 正是这种情况）。

3. **【大】把 `localization` 槽位拆成"能力"而不是"包名"**：
   `map_source`（`2d_grid|pcd|pbstream|posegraph`）× `global_relocalizer`（`none|amcl_global|pcd_multistart|descriptor`）× `local_tracker`（`gicp|ndt|slam_toolbox_localization|cartographer_localization`）。
   *依据*：Autoware 的 `pose_source` 组合 + arbiter【引用·E13】；SICK 的三段式是产品内建【引用·E21】；REP-105 的权威划分【引用·E1】。
   **收益**：`mapper` 与 `localization` 的"同名两项"（R1/R2）自然消解为"建图 vs 定位的两种 `map_source` 用法"。

### 6.5 直接回答用户的三个疑问

1. **"重定位那里不全是重定位吗？"**
   **不是。** 那个槽位里有一半是**局部跟踪**（`icp/gicp/small_gicp` 纯局部；`slam_toolbox/cartographer` 局部+有限回环），另一半才带**全局**能力（`amcl/beluga` 的均匀初始化）。**混在一个枚举里是命名问题，不是算法问题。**
2. **"局部全局要不要区分还是混合？"**
   **要区分（能力层），也要混合（数据流层）**：全局只在"开机/丢失/被搬走"时用一次，产出一个**先验**；局部逐帧跟踪并在**全程**持有 `map→odom`。§5.4 的 M2（串联）是代价最低、与我们现状最接近的形态。
3. **"我对重定位这个环节是不是有问题？"**
   **算法没错，架构缺三件**：能力分层、唯一发布者+运行期状态机、健康信号+互锁。**最该先补的是第 3 件**——因为它才是"冻结 TF 还继续开 110 m"（R5）的直接原因。

---

## §7 引用清单（全部本次抓取并核对原文）

> 抓取日期 **2026-10-19**。外部仓库 URL 均 pin 到当日 commit sha；文档站（readthedocs / docs.nav2.org / 厂商 PDF）无 sha，记录文档号/版本。

### 7.1 规范与 Nav2

| 编号 | 来源 | URL |
|---|---|---|
| **E1** | REP-105 *Coordinate Frames for Mobile Platforms*（`ros-infrastructure/rep@11ca24a`） | https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst |
| **E2** | Nav2 *Setting Up Transformations*（rolling） | https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/ |
| **E3** | Nav2 *Navigating while Mapping (SLAM)*（`use_localization:=False`） | https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/ |
| **E4** | Nav2 *Configuring AMCL*（`tf_broadcast`/`recovery_alpha_*`/`transform_tolerance`） | https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/ |
| **E5** | Nav2 BT 节点 `ReinitializeGlobalLocalization` | https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/ReinitializeGlobalLocalization/ |
| **E6** | Nav2 BT 条件 `TransformAvailable` | https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/conditions/TransformAvailable/ |
| **E7** | `nav2_amcl` 源码（`navigation2@cea9ed9`）：`globalLocalizationCallback` / `sendMapToOdomTransform` / `initServices` / `initPubSub` | https://github.com/ros-navigation/navigation2/blob/cea9ed9d2e55b4e084be705a495a721368021f19/nav2_amcl/src/amcl_node.cpp |
| **E23** | Nav2 *Smoothing Odometry using Robot Localization*（拆 `/tf` 桥） | https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/ |

### 7.2 2D/3D 定位与 SLAM 包

| 编号 | 来源 | URL |
|---|---|---|
| **E8** | `slam_toolbox` README（`@6efdc33`）：*"Localization mode consists of 3 things"*、*"publishes map to odom transform and a map"* | https://github.com/SteveMacenski/slam_toolbox/blob/6efdc33c4a45a83d6e0e0dd4490f4c891aff04d6/README.md |
| **E9** | Cartographer `pose_graph_trimmer.h#L68`（`@877157a`）：*"to implement localization without mapping"* | https://github.com/cartographer-project/cartographer/blob/877157a0d91788a7700221d87232d412cb3c1ef4/cartographer/mapping/pose_graph_trimmer.h#L68 |
| **E10** | Cartographer `configuration_files/pose_graph.lua`（`@877157a`）：`min_score = 0.55`、`global_localization_min_score = 0.6`、`branch_and_bound_depth = 7` | https://github.com/cartographer-project/cartographer/blob/877157a0d91788a7700221d87232d412cb3c1ef4/configuration_files/pose_graph.lua |
| **E11** | Cartographer ROS *Configuration*（`published_frame` / `provide_odom_frame`） | https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html |
| **E12** | Cartographer ROS *Demos*（Pure localization）+ `backpack_2d_localization.lua`（`cartographer_ros@c138034`） | https://google-cartographer-ros.readthedocs.io/en/latest/demos.html · https://github.com/cartographer-project/cartographer_ros/blob/c138034db0c47fe0ea5a2abe516acae02190dbf5/cartographer_ros/configuration_files/backpack_2d_localization.lua |
| **E24** | `beluga_amcl` README（`Ekumen-OS/beluga@30b3e7c`）：*Advertised Services* 的 `reinitialize_global_localization`、*Published Transforms* 的 `map to odom`、点云 z=常数平面假设 | https://github.com/Ekumen-OS/beluga/blob/30b3e7c8f6bad4d9173026760a4b038c7fc46e41/beluga_amcl/README.md |
| **E25** | `small_gicp` README（`@fa0cfc9`） | https://github.com/koide3/small_gicp/blob/fa0cfc983417c044b6036f71dfa448038bb9c820/README.md |
| **E26** | `hdl_graph_slam` README（`@95b8dce`） | https://github.com/koide3/hdl_graph_slam/blob/95b8dce41c10667dcf0e3e9801dc22754b3bd525/README.md |
| **E27** | Scan Context（Scan Context++ / T-RO）README：两个 API、10–15 Hz | https://github.com/gisbi-kim/scancontext_tro |
| **E28** | ORB-SLAM2 `Tracking.cc#L1341-1346`（`@f2e6f51`）：*"Relocalization is performed when tracking is lost"* | https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/src/Tracking.cc#L1341-L1346 |
| **E29** | `fuse` README（`locusrobotics/fuse@8e3f0a1`） | https://github.com/locusrobotics/fuse/blob/8e3f0a1fef6f3b8af16a50f60bf4c11843bf4d81/README.md |
| **E30** | `tf2` README（`ros2/geometry2` rolling）——**负证据**：全文无"单发布者"条文 | https://github.com/ros2/geometry2/blob/rolling/tf2/README.md |

### 7.3 Autoware

| 编号 | 来源 | URL |
|---|---|---|
| **E13** | `autoware_pose_estimator_arbiter` README（`autoware_universe@66e0169`） | https://github.com/autowarefoundation/autoware_universe/blob/66e016944aba7c1297956454189b17be8da7d16d/localization/autoware_pose_estimator_arbiter/README.md |
| **E14** | `autoware_pose_initializer` README（`autoware_core@75c5793`） | https://github.com/autowarefoundation/autoware_core/blob/75c5793614518f83167c61ba7ee9a494f9bddc83/localization/autoware_pose_initializer/README.md |
| **E15** | `autoware_ndt_scan_matcher` README（`autoware_core@75c5793`） | https://github.com/autowarefoundation/autoware_core/blob/75c5793614518f83167c61ba7ee9a494f9bddc83/localization/autoware_ndt_scan_matcher/README.md |
| **E16** | `autoware_ekf_localizer` README（`autoware_core@75c5793`）：*Published TF* | https://github.com/autowarefoundation/autoware_core/blob/75c5793614518f83167c61ba7ee9a494f9bddc83/localization/autoware_ekf_localizer/README.md |
| **E17** | `autoware_localization_error_monitor` README（`autoware_universe@66e0169`） | https://github.com/autowarefoundation/autoware_universe/blob/66e016944aba7c1297956454189b17be8da7d16d/localization/autoware_localization_error_monitor/README.md |
| **E18** | `autoware_pose_instability_detector` README（同上 sha） | https://github.com/autowarefoundation/autoware_universe/blob/66e016944aba7c1297956454189b17be8da7d16d/localization/autoware_pose_instability_detector/README.md |
| **E19** | `autoware_mrm_handler` README（同上 sha） | https://github.com/autowarefoundation/autoware_universe/blob/66e016944aba7c1297956454189b17be8da7d16d/system/autoware_mrm_handler/README.md |
| **E20** | Autoware Discussion #4870 *"Reinitialization when NDT-matching failed"*（维护者答复：需停车 + 手工/服务初始化；reinit vs MRM 仍在讨论） | https://github.com/orgs/autowarefoundation/discussions/4870 |

### 7.4 工业 AGV 与融合

| 编号 | 来源 | URL |
|---|---|---|
| **E21** | SICK **NAV350** *Operating Instructions*（英文，文档号 **8013889/1QA2/2024-11-29**）§4.7.1：Initial / Continuous / Virtual positioning、丢失处置、measurement quality | https://www.sick.com/media/docs/3/43/143/operating_instructions_nav350_laser_positioning_sensor_en_im0040143.pdf |
| **E22** | `robot_localization` *State Estimation Nodes*（`rolling-devel@7dfb6aa`）：`world_frame:=map_frame` 时必须换算成 `map→odom` | https://github.com/cra-ros-pkg/robot_localization/blob/7dfb6aa97b2082185d2fac3420888ae8474bfc1a/doc/state_estimation_nodes.rst |

### 7.5 本仓证据（R 编号 → 文件）

| 编号 | 文件 |
|---|---|
| R1/R2/R8 | `src/rm_nav_bringup/launch/bringup_sim.launch.py:1353-1366`、`1430-1434`、`1815-1819`、`1935`、`1961` |
| R3 | `src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml:221-262` |
| R4 | `src/rm_localization/gicp_registration/src/gicp_registration.cpp:1085-1088`；`docs/gicp_divergence_and_jitter.md:38,211` |
| R5 | `docs/path_clearance_and_contact.md:74,283-284,331-334` |
| R6 | `docs/localization_slots.md:6-7`（另见 §1 槽位表 `:25-32`） |
| R7 | `docs/tf_interface_contract.md:44-50` |

---

## §8 打不通 / 没核实的东西（诚实清单）

1. **`docs.nav2.org` 旧 URL 全部 404**：`/configuration/packages/configuring-amcl.html`、`/setup_guides/transformation/setup_transforms.html` 等（站点已整体迁到 `/rolling/` 前缀）。本文 Nav2 引用一律用迁移后的路径；**旧链接以后会失效**。
2. **Autoware 官方文档站的 localization 设计页 404**（`autoware-documentation/main/design/autoware-architecture/localization/`）；因此 Autoware 侧**全部改用各包 README + 一个 GitHub Discussion**，没有引用"设计文档"级别的表述。
3. **M2DP 没取到一手来源**：arXiv 上无该文（`export.arxiv.org` 查询仅返回不相关结果），IEEE DOI 页面未抓正文；本文只把它当作"描述子家族"名称提及，**未给结论性引用**。
4. **只核到一家工业厂商的原始手册**（SICK NAV350）。Pepperl+Fuchs 只取到**产品目录**（`PF-Positioning-Systems-Catalogue.pdf`，营销级，无定位机制条款），**不作为证据**；BlueBotics / 其他 AGV 厂商手册未尝试（时间/权限）。
5. **`fuse` 与 `map→odom` 的关系未核实**：README 只说明它是融合框架，**没有**"谁发 `map→odom`"的条款；本文未断言 fuse 能发该变换（§2 表中标注为"由 publisher 插件决定"）。
6. **`slam_toolbox` 与 AMCL 的精度对比没有上游定量数据**：只有作者定性表述（*"needs high quality odometry"*、*"I'd recommend AMCL"*）【引用·E8】；**未做自己的 A/B**（本轮禁止跑仿真）。
7. **Cartographer 纯定位"能否整图重定位"是推断**（依据 `max_submaps_to_keep = 3` + frozen state），**没有**上游明文说"pure localization 不支持全局重定位"。
8. **Autoware 是否真的不发 `map→odom`**：EKF README 写的是"TF from `map` to estimated pose"，**但**我没有核到 `autoware_launch` 里是否有别处补 `map→odom` 静态桥；§3.1 第 3 条已按"文档字面 + 推断"标注，**不要当成已验证的结论**。
9. **"静止才能全局重定位"是两家独立来源的共识**（SICK 明写；Autoware 讨论答复明写），但不是所有系统都这样（例如 AMCL 的均匀撒粒子并不要求静止）——**本文按"工业产品/AV 栈更保守"来解释，未做统计**。
