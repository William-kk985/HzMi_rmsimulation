# COD 实车（30° 斜装雷达）vs 本仓仿真：帧架构对账与"到底是谁的缺陷"

> **任务性质**：只读代码取证（`grep` / `read` / `git log`），**未运行任何仿真**，未修改
> `third_party/cod_nav_2026/**`、任何 launch/config/源码，也未改其它文档（本文件是本次唯一写入）。
> **本地 COD 源码**：`third_party/cod_nav_2026/`（Gitee `codnavgation/cod_-rm2026_-navigation@master`，
> 含 `.git/`，190 文件，5.4 MB；**不是 submodule**，全部可读）。下文 COD 行号一律指这份本地副本。
> **本文与既有分析的关系**：`cod_nav_nav_stack_inventory.md` / `cod_nav_2026_deep_dive.md` /
> `cod_nav_2026_params_review.md` / `cod_nav_design_thinking.md` / `cod_nav_comparison.md` 已被读；
> 本文**不重复**它们的参数搬运清单，只做"帧约定 + TF 契约 + 代价图供给"这一条纵切，
> 并**就地更正**其中两处与本地源码不符的记述（§3.1、§4.5）。
> **前置**：本仓侧的事实基线见 `docs/tilted_lidar_fidelity.md` §A/§I/§J/§N、`docs/tf_interface_contract.md`、
> `docs/robot_models.md`、`docs/architecture.md`。

---

## 0. 五条结论（先给答案）

| # | 结论 | 一句话证据 |
|---|---|---|
| **1** | **COD 的 `base_link` 就是雷达壳体系**（= `livox_frame`），**不是**重力对齐的车体系。他们不是"把 30° 吸收掉了"，而是**把 `base_link` 定义成斜的那个系**，于是 30° 在 TF 里**根本不存在**。 | `small_point_lio_node.cpp:75` 的共轭公式只在 `T(base_link←livox)=I` 时成立；上游 launch 声明的正是单位 extrinsic（`small_point_lio.launch.py:24-45`）——**且 COD 的 launch 从不发布这条边**（§2.1）。 |
| **2** | **H2 更正**：**COD 用的就是我们 vendored 的同一份 `small_point_lio`**（同一上游 `Yancey2023/small_point_lio@688d75c`）。此前"COD 用的是另一套 LIO（FAST-LIO）所以不受影响"的假设**是错的**。COD 不炸的原因**不是换了 LIO**，而是**换了 `base_link` 的定义**。 | `third_party/cod_nav_2026/src/small_point_lio/` 与 `src/rm_localization/small_point_lio/` 同源；`docs/tilted_lidar_fidelity.md` §N.1 已给出 sha256 逐字节相同的取证。 |
| **3** | **H2 的技术判决**：COD **有** `odom→base_link` 的等价处理，但**不是** `lio_tf_adapter` 式的**合成**，而是写死在 LIO 节点里的**共轭（相似变换）**：`T_ob = T_bl⁻¹·T_ol·T_bl`（`:75`）。它在 `T_bl=I` 时**退化成恒等**，所以"正确"只是**约定带来的巧合**。我们的 `lio_tf_adapter` 反而是本仓唯一写对**合成**的实现（`lio_tf_adapter_node.cpp:66-72`），却被 `bringup_sim.launch.py:2028-2033` 明确排除。 | §3.3 / §3.5 |
| **4** | **H3 判决**：COD 的 nav2 **完全没有 `obstacle_layer`、完全没有 `/scan`**（两份 params 里 `obstacle_layer` 命中数 **0**，`/scan` 命中 **0**）。两张代价图都是 `["static_layer","stvl_voxel_layer","inflation_layer"]`，观测源是 **3D 点云** `/livox/lidar_filtered`。**所以"每束 z 由扫描原点 z 伪造"这条失效路径在 COD 里根本不存在** —— 不是"他们绕过了"，是"那条代码路径没被启用"。 | `singlenav2_params.yaml:192,200-263,281`；§4.1 |
| **5** | **架构判决**：我们的架构 **(b) 本身不残缺，但默认值/约定与 COD 分叉，并额外引入了若干可避免的耦合**（不是 (c) 真缺陷）。"斜装雷达"本身固有的破点只有 2 个；我们把它放大成 6 个，其中 4 个是我们自己造的。 | §6 / §7 |

---

## 1. 证据表：用户 5 问 → COD 证据 → 我们证据 → 判决

| Q | 问的是什么 | COD 文件:行 | 我们文件:行 | 判决 |
|---|---|---|---|---|
| **Q1** | `base_link` 该不该和雷达正方向对齐 | `small_point_lio_node.cpp:75`（共轭，仅 `T_bl=I` 时对）· `small_point_lio.launch.py:24-45`（单位 extrinsic 的声明，`:47` 未启用）· `filter_node.cpp:56`（marker 发在 `base_link`，而裁剪作用在输入点云系）· `multiplenav_launch.py:70`（p2l `target_frame: base_link` 而输入是 `/livox/lidar`） | `tf_interface_contract.md:44`（契约链 `…→base_link→livox_frame`）· `robot_models.md:173-174`（base_link 原点=底盘最低点，雷达 +0.80 m）· `tilted_lidar_fidelity.md:22/§A.1`（默认档 `base_link→livox_frame` rpy=0） | **COD 用"传感器壳体系"当 base_link（违反 REP-103 语义）；我们用"车体/重力系"。两种都自洽，代价落点不同 —— 见 §2.3。** |
| **Q2** | COD 跑 `small_point_lio` 时的 TF 处理是否正确/等价 | `small_point_lio_node.cpp:52-100`（**节点自己**发 `odom→base_link`，无外部 adapter）· `:75`（**共轭**）· `:127`+`:159`（`/cloud_registered` 自称 `odom` 却对已配准点再乘一次 `T(base_link←livox)`） | `bringup_sim.launch.py:2021-2026`（理由注释）· `:2028-2033`（`lio!=small_point_lio` 才启动 adapter）· `lio_tf_adapter_node.cpp:66-72`（**合成**：`q_lio*q_offset`、`p+R·t`） | **等价但不等价于"正确"**：COD 靠 `T_bl≡I` 让共轭退化成恒等；我们排除了唯一写对合成的节点，改用上游的共轭（已于 `2a5a8b6` 修为合成，属上游潜伏 bug，见 §N.1）。 |
| **Q3** | COD nav2 用哪些帧、代价图吃什么、有没有高度带 | `singlenav2_params.yaml:183-192`（local：`odom`/`base_link_fake`/三插件）· `:200-263`（STVL，源=`/livox/lidar_filtered` PointCloud2）· `:229-230`（`min 0.15`/`max 1.0`）· `:273-281`（global：`map`/`base_link_fake`）· `:319-320`（global 源的同一带） | `nav2_params_sim_base.yaml:158-184`（local 四插件，`obstacle_layer`=`/scan` 默认开）· `:180`（`max 2.0`，**`min` 未设⇒0.0**）· `:301-323`（global）· `:354-355`（STVL `0.0/2.0`）· `nav2_params_sim_robot11_costmap.yaml:34-44`（伪造 z 的机理+实测） | **COD 有高度带（0.15/1.0），但量的是真 3D z；我们的 `obstacle_layer` 带量的是"传感器原点 z"。COD 没有这条路径 ⇒ 没有该失效模式。** |
| **Q4** | "COD 也是 nav2，为啥它没事" | `grep -c obstacle_layer` = **0 / 0**；`grep /scan` = **0**；`singlenav_launch.py` 全文件**没有** `pointcloud_to_laserscan` | 我们 local 默认 `local_obstacle:=scan`（`bringup_sim.launch.py:1416-1425`） | **COD 用的是 nav2，但没用 nav2 的 `obstacle_layer`+`LaserScan`。见 §5 的"它没用的 7 个特性"。** |
| **Q5** | 现在这个仿真架构有缺陷吧 | — | §6 并排表 | **(b) 约定分叉 + 可避免耦合；不是 (c) 真缺陷。** |

---

## 2. H1：COD 的 `base_link` 约定到底是什么

### 2.1 COD 侧证据链（四条，互相独立）

**(a) 上游 launch 声明的 `base_link→livox_frame` 是单位阵 —— 而且它从不被启用。**

`third_party/cod_nav_2026/src/small_point_lio/launch/small_point_lio.launch.py:24-45`：

```python
24:    static_base_link_to_livox_frame = Node(
25:        package="tf2_ros",
26:        executable="static_transform_publisher",
27:        arguments=[
28:            "--x", "0.0",
30:            "--y", "0.0",
32:            "--z", "0.0",
34:            "--roll", "0.0",
36:            "--pitch", "0.0",
38:            "--yaw", "0.0",
40:            "--frame-id", "base_link",
42:            "--child-frame-id", "livox_frame",
```
```python
47:    return LaunchDescription([small_point_lio_node])          # ← 只返回节点，静态 TF 未加入
```

> ⚠️ **这是本仓与 COD 的`small_point_lio` 的**唯一**一处差异**：我们的 vendored 副本
> `src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` 是
> `LaunchDescription([small_point_lio_node, static_base_link_to_livox_frame])`（`git blame` = 引入提交
> `98baa55`），**COD 那份没有**。也就是说：**那条"单位 extrinsic"在 COD 里是死代码**。
> 这条对 §2.1(c) 的推理很关键 —— COD 真机的 `base_link→livox_frame` **必须来自仓库之外**
> （他们自己的 URDF / 实车 bringup），且**必须是单位（或纯平移）**，否则 §3.3 的共轭公式会立刻算错。

**(b) COD 的 `cod_bringup` 里根本没有任何 `base_link→livox_frame` 发布者。**
全仓 `grep -rn "static_transform_publisher"` 只有 3 处：`singlenav_launch.py:69`（`map→odom`）、
`multiplenav_launch.py:97`（`map→odom`）、`small_point_lio.launch.py:26`（上面那条死代码）。
`robot_state_publisher` **未被启动**（只有 `navigation_launch.py:54`、`localization_launch.py:49`
两处注释性 URL 提及）。⇒ **这份仓库自己跑不起来 `odom→base_link`**，必须由外部 URDF 提供。

**(c) 节点内的换算式只在 `T(base_link←livox) = I` 时成立。** 见 §3.3 的逐行推导。

**(d) 兄弟节点的行为也把 `base_link` 当 `livox_frame` 用。**

| 节点 | 行 | 做的事 | 只有 `base_link≡livox_frame` 才自洽 |
|---|---|---|---|
| `cpp_lidar_filter` | `filter_node.cpp:81-119` | `pcl::CropBox` 直接作用在**输入点云自身坐标系**（=`livox_frame`），**不做 TF**；`:117` `output.header = msg->header` 原样保留输入帧 | 它却把可视化 marker 发在 `base_link`（`:56`）⇒ 注释/README 里的"base_link 裁剪盒"成立的前提就是两个帧相同 |
| `pointcloud_to_laserscan` | `multiplenav_launch.py:67-73` | 输入 `/livox/lidar`（帧 = `livox_frame`），却设 `target_frame: base_link`、`min_height: 0.1 / max_height: 1.00` | 高度带的语义是"离 base_link 的 z"，只有在两帧相同时才等于"离雷达的 z" |

**结论**：COD 的帧约定是 **`base_link` ≡ 雷达壳体系**。它的原点是雷达，它的 z 轴随雷达一起斜 30°。
这在 REP-103 的意义上**不是**一个合格的 `base_link`（`base_link` 应是车体水平投影中心、z 朝上），
但它**内部自洽**：所有消费者（LIO、裁剪盒、p2l、nav2）都按同一个斜系记账，账就对得上。

### 2.2 我们的约定（完全不同）

```
docs/tf_interface_contract.md:44
map ──(重定位：AMCL / slam_toolbox / ICP 之一发布)──► odom ──(里程计：LIO 发布)──► base_link ──(URDF 固定关节)──► livox_frame / imu_link …
```

| 项 | 我们 | 证据 |
|---|---|---|
| `base_link` 原点 | 底盘**最低点**、z=0（车体几何中心投影） | `robot_models.md:173`「底盘高 0.30 m（`base_link` 原点就是**最低点**，`z=0`）」 |
| `base_link` 姿态 | **重力对齐**（z 朝上），与雷达无关 | 同上 + `tilted_lidar_fidelity.md:22` 的实测：`base_link→livox_frame` rpy `0/0/0` |
| `livox_frame` | **独立子帧**，由 URDF 固定关节提供 | 默认模型 `[0,0,0.80] rpy 0/0/0`；robot11 `[0.000562,0.130916,0.157028] rpy 0/0/0`（§A.1）；`robot11_mount:=urdf|sensor` 时 `rpy=[-30,0,0]` |
| 谁提供这条边 | `robot_state_publisher`（URDF） | `bringup_sim.launch.py:1784-1786`「它**需要** `base_link→livox_frame` 这条静态 TF…仿真里由 robot_state_publisher 从 URDF 提供」 |

### 2.3 30° 必须被吸收在某个地方 —— 谁买单

| | **COD** | **我们** |
|---|---|---|
| 30° 记在哪 | **帧的定义里**：`base_link` 就是斜的。TF 里**没有**这个 30°，URDF 里也**没有**（这份仓库没有 URDF）。 | **一条显式 TF 边上**：`base_link→livox_frame` 的 `rpy`（`urdf`/`sensor` 档），**或**插件的 `<tilt_rpy>`（`plugin` 档，此时 TF rpy=0） |
| 谁"付钱" | **帧语义**付钱：`base_link` 不再是车体系 ⇒ 任何按 REP-103 理解它的消费者都错；高度带全部在斜系里量；渲染出来的 TF 树对物理挂装**说谎** | **整条栈**付钱：每个做 TF 合成、做高度带、做地面拟合的环节都必须对这个 30° 正确 —— 而其中有一环（上游 `small_point_lio`）**不正确** |
| 隐蔽性 | **极高**：没有一行代码写"30°"，grep 不到，review 也看不出来 | **低**：`grep -rn "rpy"` 一行就能找到 |
| 出错时的表现 | 只在"换个消费者"时暴露（且通常表现为精度略降，不是崩） | 立刻表现为 `/cloud_registered` 地面 30.7° 斜面、代价图把 30° 当世界（`tilted_lidar_fidelity.md:49` 实测 `7506/11622 = 64.6%` 的点"低于水平地面"） |
| 谁更难维护 | 换雷达/换挂装 ⇒ 全栈重新标定，且**没有任何地方记录了旧值** | 值集中、可算、可测（`_LioAdapterRobot11` 从 URDF 几何算，`bringup_sim.launch.py:2073`） |

> **所以"谁买单"的准确答案**：COD 把账记在**约定**上（一次性、零成本、但语义是错的、且不可见）；
> 我们把账记在**代码/配置**上（每次都要对、对不上就炸、但对得上就是对的）。

### 2.4 直接回答"base_link 不就应该和雷达正方向对齐吗"

**不应该 —— REP-103 的 `base_link` 是车体系，不是传感器系。** 但这里有个真实的两难：

* 若 `base_link` = 车体系（我们的选择），则 30° 必须由某条 TF 边承载，**所有**消费者都得处理它；
* 若 `base_link` = 雷达系（COD 的选择），则"承载"这一步消失了，但**所有**消费者都被悄悄搬进一个斜 30° 的世界。

**COD 之所以看起来"base_link 和雷达对齐了"，是因为他们干脆把 `base_link` 定义在雷达上。**
这不是"更正确的做法"，而是"把不变式从代码挪到了约定里"。对 2D nav2（几乎不关心 pitch，§4.3）
这个trade 很划算；对一个要同时服务 3D SLAM / ICP / cartographer / 多种重定位的**算法对比 bench**，
这个 trade 不划算 —— 因为 bench 的价值恰恰在于"每个环节的坐标语义都写下来、都可 A/B"。

---

## 3. H2：`small_point_lio` 的 TF 处理 —— 含**明确更正**

### 3.1 更正声明（必须显式写下）

> **此前的假设是错的。** 我们曾经推断："COD 实车不受 30° 斜装影响，是因为**它用的不是我们 vendored 的这套 LIO**
> （或用的是修过的版本）。"
>
> **事实**：COD 2026 成品（本地 `third_party/cod_nav_2026/`，Gitee `cod_-rm2026_-navigation@master`）
> 与 GitHub `rmul2026` **用的就是 `small_point_lio`**，并且与我们 vendored 的是**同一份上游**
> （`Yancey2023/small_point_lio@688d75c`，`ros2` 分支）——`docs/tilted_lidar_fidelity.md` §N.1 已用
> `git blame` + `sha256` + 与上游 `diff` 为空三条独立取证。本仓对该 `.cpp` 的**唯一**改动是
> `a281636`（1 ns 截断）与 `2a5a8b6`（修 bug ②③），**没有一行是"我们写错的"**（§N.0 第 1 条）。
>
> **COD 不炸的真原因不是"换了 LIO"，而是"换了 `base_link` 的定义"**：`T_bl ≡ I` ⇒ 上游那两个
> 变换 bug **同时退化**（§3.3、§3.4）。
>
> 顺带更正两处既有文档与本地源码不符的记述：
> * `docs/cod_nav_comparison.md:27`「全局 costmap … 只吃 `/scan`」——那是 2025 `master`/GitHub
>   `rmul2026` 的形态；**Gitee 2026 的两张图都不吃 `/scan`**（§4.5）。
> * `docs/cod_nav_comparison.md:43`「他们的带在雷达面**之上** 0.01~1.0」——Gitee 2026 的源级带是
>   **`0.15 / 1.0`** 且 `max` 是"哨兵高度以上不关心"（`singlenav2_params.yaml:229-230`）。

### 3.2 COD：谁发布 `odom→base_link`？—— **不是 adapter，是 LIO 节点内联**

```
third_party/cod_nav_2026/src/small_point_lio/src/small_point_lio_node.cpp:61-62
61:            transform_stamped.header.frame_id = "odom";
62:            transform_stamped.child_frame_id = "base_link";
...
98:            tf_broadcaster->sendTransform(transform_stamped);
```

* 发布者 = `small_point_lio` 节点本身，**硬编码**帧名，**无参数可关**（`:61-62` 是字面量，不是 `declare_parameter`）。
* COD **没有**任何 `lio_tf_adapter` 之类的桥接包：`ls third_party/cod_nav_2026/src/` 只有
  `small_point_lio` / `cpp_lidar_filter` / `pointcloud_to_laserscan` / `fake_vel_transform` /
  `cod_bringup` / `goal_approach_controller` / `pb_nav2_plugins` / `pb_omni_pid_pursuit_controller` /
  `ros2_simple_serial` / `waypoint_editor` 十个包，**没有 adapter 类构件**。
* LIO 的输入/输出帧约定：`mid360.yaml:6` `lidar_frame: livox_frame`；`:30-33`
  `extrinsic_T=[-0.011,-0.02329,0.04412]`、`extrinsic_R=I`（**MID-360 出厂 IMU↔LiDAR 杆臂**）。

### 3.3 共轭还是合成？—— 逐行推导，判决是**共轭**

```cpp
:63:  geometry_msgs::msg::TransformStamped base_link_to_lidar_frame_transform;
:65:      base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg);
:70:  tf2::Transform tf_lidar_odom_to_lidar_frame;                       // T_ol
:75:  tf2::Transform tf_odom_to_base_link = tf_base_link_to_lidar_frame.inverse()
:75:                                    * tf_lidar_odom_to_lidar_frame
:75:                                    * tf_base_link_to_lidar_frame;  // T_bl⁻¹ · T_ol · T_bl
```

记法（与代码一致）：`T_bl := lookupTransform(livox_frame, "base_link")` = **base_link 在 livox 系里的位姿**
（tf2 语义：`lookupTransform(target, source)` 返回 `T_target←source`）。`T_ol` = LIO 输出的 body(=livox) 位姿。

* **正确的合成**（链式法则）：`p_odom = T_ol·p_livox = T_ol·(T_bl·p_base)` ⇒ **`T_ob = T_ol·T_bl`**。
* **COD 的写法**：`T_bl⁻¹·T_ol·T_bl` = **共轭 / 相似变换**。把它展开：

| 分量 | 正确值（合成） | COD（共轭） | 何时相同 |
|---|---|---|---|
| 旋转 | `R_ol·R_bl` | `R_bl⁻¹·R_ol·R_bl` | 仅 `R_bl = I`（因为 `R_ol·R_bl` 与 `R_bl⁻¹R_olR_bl` 一般不等） |
| 平移 | `p_ol` | `R_bl⁻¹·p_ol` | 仅 `R_bl = I` |

⇒ **只有当 `T_bl = I`（单位阵）时，共轭才与合成逐位相等**，此时它退化成一个恒等操作，
`odom→base_link` 就等于 LIO 的原始输出。

> 我们的 `docs/tilted_lidar_fidelity.md` §J.1.2 已经独立得出同一结论：
> 「**共轭只在 `T_bl` 是纯平移时"姿态碰巧对"**」，并给了判决性数字
> （`urdf` 档改前假俯仰 **4.890°**、改后 **0.317°**）。本文的增量是：
> **COD 的 `T_bl` 恰好就是 I**，所以这个 bug 在他们的部署里**不可观测**。

### 3.4 是否对"已经配准的点"再变换了一次？—— **是**，同样被 `T_bl≡I` 掩盖

```
:101:  small_point_lio->set_pointcloud_callback([...](const std::vector<Eigen::Vector3f> &pointcloud) {
:127:                msg.header.frame_id = "odom";
:159:                    transformed_point = lidar_frame_to_base_link_R * point + lidar_frame_to_base_link_T;
```

而回调拿到的点**已经是 odom 系**——`small_point_lio.cpp:98`：

```cpp
98:  pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());
```

⇒ 数据自称 `odom`（`:127`），却又被乘了一次 `T(base_link←livox)`（`:159`）。
这与本仓 **bug ②** 是同一处（§N.1 第 6 行，blame `98baa55` = 上游原样）。
`|t| = 0.2044 m` 的纯平移在 COD 里表现为"整朵云平移 20 cm"（`extrinsic_T` 只有 4.4 cm，
而 COD 的实车 `base_link→livox_frame` 若为纯平移，这个平移量就是它）——**肉眼与单帧统计都看不出**。

### 3.5 我们：`lio_tf_adapter` 是**正确的合成**，却被排除掉了

```
src/rm_localization/lio_tf_adapter/src/lio_tf_adapter_node.cpp:66
66:    const tf2::Quaternion q_out = q_lio * q_offset;
67:    const tf2::Vector3 t_offset(xyz_[0], xyz_[1], xyz_[2]);
68:    const tf2::Vector3 t_rotated = tf2::quatRotate(q_lio, t_offset);
70-72: tf_msg.transform.translation.{x,y,z} = msg->pose.pose.position.{x,y,z} + t_rotated.{x,y,z}();
```

即 `T_out = T_lio ∘ T_offset`（`q = q_lio·q_off`、`p = p_lio + R_lio·t_off`）—— **这是链式法则的正确写法**，
也正是 §N.0 第 2 条所说的"本仓唯一写对合成的桥接"。

**但它在 `lio:=small_point_lio` 时被明确排除**（三处，互斥）：

```
src/rm_nav_bringup/launch/bringup_sim.launch.py:2028-2033
'", LaunchConfiguration('lio'), "' != 'none' and '",
'", LaunchConfiguration('lio'), "' != 'cartographer' and '",
'", LaunchConfiguration('lio'), "' != 'small_point_lio' and '",       # ← 排除
'", LaunchConfiguration('robot'), "' != 'hzmirm' and '",
'", LaunchConfiguration('robot'), "' != 'robot11'"])))
```
（`:2051-2055` hzmirm 分支、`:2076-2080` robot11 分支带**同样的** `!= 'small_point_lio'` 条件。）

理由注释在 `:2021-2026`，其中一句是准确且重要的：

```
:2025:  #   杆臂由它自己处理（⚠️ 该换算的写法有瑕疵，见 docs/lio_slots.md §6 第 2 条）。
```

⇒ **我们当时就知道那处"有瑕疵"，但选择"排除 adapter"而不是"修 LIO"**（后来 `2a5a8b6` 才修 LIO）。
后果：`robot:=robot11` 的 30° `rpy` 补偿（`_LioAdapterRobot11('rpy')`，`:2088`）
**在 `lio:=small_point_lio` 时没有任何载体**——它只对 `lio:=fastlio|pointlio` 生效。

### 3.6 H2 判决

| 问题 | 答案 |
|---|---|
| COD 跑的是同一个 `small_point_lio` 吗？ | **是**，同一上游 commit，逐字节相同（§N.1）。**"COD 换了 LIO"这个前提是错的。** |
| COD 有 `odom→base_link` 的等价处理吗？ | **有**，但是**节点内联的共轭**（`:75`），不是 `lio_tf_adapter` 式的合成，也**没有**独立 adapter 节点。 |
| 那个处理"正确"吗？ | **仅在 `T(base_link←livox) = I` 时正确**——而这正是 COD 的约定（§2.1）。所以它是**"约定正确"，不是"数学正确"**。 |
| 我们排除 `lio_tf_adapter` 是对的吗？ | **当时不得已、今天仍不对称**：两次发布者冲突是真实约束（`:2021-2022`），但排除掉的恰好是本仓唯一写对合成的实现，而需要那个合成的正是 30° 斜装槽位。**§8 的选项 B 就是收这一条。** |

---

## 4. H3：COD 的 nav2 用哪些帧、代价图吃什么、有没有高度带

### 4.1 COD 的两张代价图（`singlenav2_params.yaml`，单点模式 = 比赛模式）

| 项 | local_costmap | global_costmap |
|---|---|---|
| `global_frame` | **`odom`**（`:183`） | **`map`**（`:273`） |
| `robot_base_frame` | `base_link_fake`（`:184`） | `base_link_fake`（`:274`） |
| `rolling_window` | `true`（`:186`）· 10×10 @0.05（`:187-189`） | **`true`**（`:275`）· 25×25 @0.04（`:269-270,279`） |
| `robot_radius` / footprint | `0.1` / `[±0.1]²`（`:190-191`） | 同（`:277-278`） |
| **插件列表** | **`["static_layer","stvl_voxel_layer","inflation_layer"]`（`:192`）** | **`["static_layer","stvl_voxel_layer","inflation_layer"]`（`:281`）** |
| **`obstacle_layer`** | **无** | **无** |
| 观测源 | `observation_sources: livox_source realsense_source`（`:218`）· `livox_source.topic = /livox/lidar_filtered`、`data_type: PointCloud2`（`:220-221`） | 同（`:308,311`） |
| **高度带** | **源级** `min_obstacle_height: 0.15` / `max_obstacle_height: 1.0`（`:229-230`）；RealSense `0.18/1.5`（`:252-253`） | **源级** 同一组（`:319-320`、`:342-343`） |
| 层级其它 | `max_obstacle_height 2.0`（`:208`）、`origin_z 0.0`（`:213`）、`voxel_decay 0.5`（`:203`）、`decay_model 0`（`:204`）、`model_type 1`（`:237`）、`vertical_fov_angle 2.00`（`:238`） | 同（`:298-300` 等，逐项镜像本表左列） |

**多模式（`multiplenav2_params.yaml`）**：帧与插件结构完全相同（`:177-186` local、`:267-275` global），
高度带 `0.1/1.0`（`:223-224`、`:313-314`），另外多了 `pointcloud_to_laserscan` → `/scan` → `slam_toolbox`
（`multiplenav_launch.py:65-94`）—— **`/scan` 只喂建图，不进任何 costmap**（§5 第 6 条）。

### 4.2 我们（对照）

| 项 | local_costmap | global_costmap |
|---|---|---|
| `global_frame` | **`odom`**（`nav2_params_sim_base.yaml:128`） | **`map`**（`:284`） |
| `robot_base_frame` | `base_link_fake`（`:129`） | `base_link_fake`（`:285`） |
| 插件列表 | `["obstacle_layer","obstacle_cloud_layer","stvl_layer","inflation_layer"]`（`:158`） | `["static_layer","obstacle_layer","stvl_layer","inflation_layer"]`（`:301`） |
| **默认启用** | **`obstacle_layer` = `/scan`**（`local_obstacle:=scan`，`bringup_sim.launch.py:1416-1425`） | **`stvl_layer`**（`global_obstacle:=stvl`，`:1408-1414`） |
| `obstacle_layer` 高度带 | `max_obstacle_height: 2.0`（`:180`）；**`min` 未设 ⇒ nav2 默认 `0.0`**（`obstacle_layer.cpp:142`） | 同（`:323`） |
| STVL 高度带 | `min 0.0 / max 2.0`（`:250-251`），**注释显式登记这是对 COD `0.15` 的偏离**（`:218-222`） | `min 0.0 / max 2.0`（`:354-355`），同样登记（`:350-355`） |
| STVL 源 | `/segmentation/obstacle`（`:246`） | `/segmentation/obstacle`（`:346`） |

### 4.3 关键区别：两种"高度"是**不同物理量**

这是整个 H3 的枢纽。

**(i) COD 的路径：点云 → STVL，`z` 是点的真实 3D 坐标。**
STVL 把 `PointCloud2` 的每个点按其 (x,y,z) 落到体素里，`min/max_obstacle_height` 在**代价图帧**的 z 上过滤。
每个点的 z 是 LIO 配准出来的**真实几何高度**（相对代价图帧）。

**(ii) 我们的路径：`/scan` → nav2 `obstacle_layer`，`z` 是**伪造**的。**

```
third_party/nav2/nav2_costmap_2d/plugins/obstacle_layer.cpp:337
337:        projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
```
`LaserScan` 没有高度；`transformLaserScanToPointCloud` 把每条波束按 `range/angle` 展开成
`z = 0`（该帧内）的点，然后：

```
third_party/nav2/nav2_costmap_2d/src/observation_buffer.cpp:143-144
143:      if ((*iter_z) <= max_obstacle_height_ &&
144:        (*iter_z) >= min_obstacle_height_)
```
这段过滤发生在 `global_frame_cloud`（已变换到**代价图帧**）上。而"`z=0` 的点从扫描帧搬到代价图帧"
只会带上一个平移量 ⇒ **每条波束在代价图帧里的 z 恒等于"那一帧扫描原点的 z"**，
与它实际打到什么三维结构**完全无关**。

我们自己的实测（`src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml:34-41`，
以及 `traversability_near_ground_robot11.yaml:17-21`）：

> `/scan` 是**二维平盘**（LaserScan 没有高度），nav2 的 `obstacle_layer` 先 `projectLaser`（**z 强行置 0**）
> 再 `transformLaserScanToPointCloud` 搬到代价图帧 ⇒ **每条波束在代价图帧里的 z 恒等于"那一帧点云原点的 odom z"**，
> 与它打到的三维点无关。实测（robot11 plugin 档、静止）：950 条波束的 odom z 全在 **[0.0564, 0.0815]**（跨度 25 mm），
> `[0,2]` 之外 **0 条**。⇒ 这两个键在 2D 链路上**只是"传感器自身高度"的常量闸**。

⇒ **"把扫描面挪 0.2 m ⇒ 950/950 条波束全丢"** 这个现象，是 (ii) 这条路径的**必然**：
整个平盘一起被移出 `[min,max]` 带。它在 (i) 里**不可能**发生 —— 因为 (i) 里每个点的高度是它自己的。

### 4.4 COD 有没有那个失败模式？—— **没有，而且连"可能没有"的余地都没有**

因为 **COD 完全没有启用 `obstacle_layer`**：

```
$ grep -c "obstacle_layer" src/cod_bringup/params/singlenav2_params.yaml      → 0
$ grep -c "obstacle_layer" src/cod_bringup/params/multiplenav2_params.yaml    → 0
$ grep -n "/scan\|scan:" src/cod_bringup/params/singlenav2_params.yaml        → (无输出)
```
```
$ grep -rn "static_transform_publisher" src/                                        → 3 处，全是 map→odom（+1 处死代码）
$ ls src/ | grep -c pointcloud_to_laserscan                                          → 1，但只在 multiplenav_launch.py 被 include
$ grep -n "pointcloud_to_laserscan" src/cod_bringup/launch/singlenav_launch.py       → (无输出)
```

⇒ **单点（比赛）模式的 COD 不存在任何 `LaserScan` 进入代价图的通道**；`/scan` 这个类型在它的
nav2 参数里一次都没出现。**"每束 z 由扫描原点伪造"这条代码路径没有被执行过。**

### 4.5 但 COD **也有**一个"高度带在斜系里量"的问题（诚实登记）

COD 的 `odom` **不是重力对齐的**，而是"**初始 LiDAR 系**"。取证：

```
src/small_point_lio/src/small_point_lio/eskf.h:30
30:        Eigen::Matrix<value_type, 3, 3> rotation = Eigen::Matrix<value_type, 3, 3>::Identity();    // 旋转
```
```
src/small_point_lio/src/small_point_lio/small_point_lio.cpp:54-64
54:                if (parameters.fix_gravity_direction) {
55:                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
57:                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
59:                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
60:                    estimator.kf.x.gravity *= scale;
64:                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
```

`R_W_B(0) = I`（`:30`）且 `gravity` 直接取"前 200 帧 IMU 加速度之和"（`:55-60`，**在 body 系里量的**）
⇒ 重力被当成一个**世界系的自由 3 维向量**（`:64` 用 `-gravity` 抵消），
**世界系（`odom`）被定义为初始 body 系**。所以 30° 斜装 ⇒ **`odom` 的 z 轴也斜 30°**，
`gravity` 状态里存着一个斜的重力向量。

**后果**（我们的判断，标记为推断，见 §9）：
1. STVL 的 `min/max_obstacle_height 0.15/1.0` 量的是**沿斜 30° 的 z**，不是真高度 —— 一个 0.85 m 宽的斜板；
2. `pointcloud_to_laserscan` 的 `min_height 0.1 / max_height 1.0`（`multiplenav_launch.py:72-73`）
   在 `target_frame: base_link`（= 斜系）里切片 ⇒ 也是一个斜板；
3. nav2 的 footprint、`robot_radius`、yaw 全在斜系里 ⇒ 与地面真投影差一个 `cos30°` 量级的因子。

**但这三条与"950/950 全丢"不是同一类失效**：
前者的失效是"**漏掉一部分本应看见的障碍**"（连续、可容忍、可以靠加宽带子掩盖）；
后者是"**整帧观测归零**"（离散、致命、且看起来像"没有障碍物"⇒ 直接撞）。
COD 靠"带子宽 + 量真值 + 不在回路里做 2D→3D 反投影"把前者压到了不可见。

---

## 5. "cod 那个也是 nav2 阿，为啥那个 nav2 没问题" —— 它**没用**/改配了这 7 条

**是的，COD 用的就是 nav2（`cod_bringup/launch/navigation_launch.py` 起 nav2 全家桶），
而且它踩的坑和我们是同一批 nav2 代码。差别全在"用了哪几个特性"：**

| # | nav2 特性 | **COD** | **我们** | 这一条是否解释了"我们不炸它炸" |
|---|---|---|---|---|
| 1 | **`nav2_costmap_2d::ObstacleLayer` + `LaserScan`** | **完全不用**（两份 params `grep` = 0） | **local 默认开**，global 可选 | ✅ **主因**。这条路径的 z 是伪造的（§4.3-ii） |
| 2 | **`/scan` 作为代价图源** | **不用**（`/scan` 只喂 `slam_toolbox`，仅多模式） | 是 local 的**唯一** 2D 源（`local_obstacle:=scan`） | ✅ 同上 |
| 3 | **3D→2D 反投影（p2l）在导航回路里** | **不在**（单点模式无 p2l；多模式 p2l 只喂建图，且 `map→odom` 是静态的） | **在**（`linefit→/segmentation/obstacle→p2l→/scan→costmap`） | ✅ 我们的"高度带"因此天然有两个frame |
| 4 | **`robot_base_frame` 的父帧语义** | `base_link_fake` 的父 `base_link` **= 雷达系**（无 30° 记账） | `base_link_fake` 的父 `base_link` **= 车体系**（有 30° 记账） | ✅ §2 的全部内容 |
| 5 | **`map→odom` 的来源** | **静态 TF**（`singlenav_launch.py:67-89`，`z=0.05, yaw=-0.5`；多模式 `yaw=0`）—— 没有任何定位节点（`localization_launch.py` `lifecycle_nodes=['map_server']`） | **6 选 1**（amcl/beluga/slam_toolbox/icp/gicp/small_gicp/cartographer） | ⚠️ **不是**原因，但决定了"帧树错了会不会被发现"：COD 静态桥没有反馈回路能暴露误差 |
| 6 | **`pointcloud_to_laserscan`** | 只在多模式、只喂 `slam_toolbox`（`multiplenav_launch.py:65-94`） | **核心环节**（p2l 是感知域的唯一 2D 出口） | ✅ 同 #3 |
| 7 | **全局代价图是滚动窗** | `rolling_window: true`、25×25（`singlenav2_params.yaml:275`） | `map` 系、固定、`static_layer` | ⚠️ 影响"全局图能不能把错误摊薄"，非主因 |

**一句话**：**"COD 也用 nav2"这句话是对的，但它只用了 nav2 的
`planner / controller / bt / behavior / STVL / static / inflation`，
把 `obstacle_layer + LaserScan` 这条最容易踩帧坑的路径整个弃用了。**
它是"用 nav2 的骨架 + 自己的 3D 感知"，我们是"用 nav2 的全套 2D 感知"。

---

## 6. 架构判决

### 6.1 并排表（任务要求的 8 行）

| 维度 | **COD**（Gitee 2026 成品） | **我们**（本仓默认） | 谁更"稳" |
|---|---|---|---|
| **① TF 契约链** | `map --(静态 TF，z=0.05,yaw=-0.5)--> odom --(small_point_lio 内联)--> base_link --(fake_vel_transform，**仅 yaw**)--> base_link_fake`；**`livox_frame` 与 `base_link` 是同一个系，无边** | `map --(6 选 1 重定位)--> odom --(lio_tf_adapter **或** LIO 内联)--> base_link --(**URDF 固定关节**)--> livox_frame/imu_link`；`base_link --(fake_vel_transform，仅 yaw)--> base_link_fake` | COD 更短 ⇒ 更少出错点；我们更长 ⇒ 语义更全 |
| **② `base_link` 定义** | **= 雷达壳体系**（原点在雷达、z 随雷达斜） | **= 车体系**（原点在底盘最低点、z 朝上/重力对齐） | 我们对（REP-103）；COD 稳（无记账） |
| **③ 30° 记在哪** | **帧定义 + ESKF 的 `gravity` 状态**（`eskf.h:30` + `small_point_lio.cpp:54-64`）；TF/URDF 里**没有** | **一条显式 TF 边**（`urdf`/`sensor` 档的 `livox_frame` rpy）**或插件 `<tilt_rpy>`**（`plugin` 档） | COD 不可见；我们可见可测 |
| **④ 谁发 `map→odom`** | `static_transform_publisher`（**无定位**） | amcl / beluga / slam_toolbox / icp / gicp / small_gicp / cartographer（+ `lio:=none` 的静态回退桥） | 我们强得多（且这正是 bench 的资产） |
| **⑤ costmap 吃什么** | local/global **都是** `static + STVL(PointCloud2)`；源 = `/livox/lidar_filtered` + RealSense | local：`obstacle_layer(/scan)` **默认开** + cloud + STVL(可选)；global：`static + STVL(/segmentation/obstacle)` | COD 单源单帧、我们多源多帧（bench 需要，但默认值把它暴露在 §4.3-ii 上） |
| **⑥ "height" 参数住在哪** | **一处**：STVL 源级 `min/max_obstacle_height`（`0.15/1.0`），量**真 3D z** | **四处**：nav2 `obstacle_layer` 源级（`min` 默认 0.0，量**伪造 z**）、STVL 源级（`0.0/2.0`，量真 z）、`linefit` 的 `sensor_height`+`gravity_aligned_frame`、p2l 的 `min/max_height`+`target_frame`、自击掩膜的 z 门限 | **COD 一处、我们四处** ⇒ 这是最该收的一条 |
| **⑦ 3D→2D 转换步骤** | **不在导航回路里**（单点无 p2l；多模式 p2l 只喂建图，且 `map→odom` 静态 ⇒ 不回灌 costmap） | **在回路里且是唯一 2D 出口**：`linefit/patchwork → /segmentation/obstacle → p2l → /scan → costmap` | COD 天然免疫 |
| **⑧ 斜装雷达会破几个地方** | **2 个**（STVL 斜板、p2l 斜板）+1 个语义债（`base_link` 不再是车体系）；**都不在致命路径上** | **6 个**（下表 §7.2）：TF 合成 · LIO 共轭 · linefit 重力系 · p2l 高度带 · nav2 伪造 z · 自击掩膜 z | **2 vs 6** |

### 6.2 判决：**(b) 约定分叉 + 可避免耦合；不是 (c) 真缺陷**

| 选项 | 判定 | 理由 |
|---|---|---|
| **(a) 架构没问题，只是文档不够** | ❌ 不成立 | 光靠文档解决不了 §7.2 的 6 条 —— 其中 2 条是真 bug（上游共轭、双重变换），2 条是**没有唯一所有者**的配置（高度带、`lio_tf_adapter` 排除），1 条是我们把上游死代码启用了。文档是必要条件，不是充分条件。 |
| **(b) 架构没问题，但默认值/约定与 COD 分叉，并额外引入了可避免的耦合** | ✅ **成立（主判决）** | 我们的契约（REP-103 base_link、显式 TF 边、多重重定位）对一个**算法对比 bench** 是**更正确**的选择，不该向 COD 看齐。真正该修的是：**(b1)** 让"高度"只有一个所有者；**(b2)** 让 `lio_tf_adapter` 成为所有 LIO 槽的**唯一** `odom→base_link` 权威；**(b3)** 把 `robot11_mount` 三档收敛成一档默认 + 一档保真；**(b4)** 清掉那条被启用的上游单位 TF。 |
| **(c) 架构真缺陷** | ❌ 不成立 | "多一层显式 TF 边"不是缺陷，是 REP-103 的**要求**。同样的 30° 若放进 COD 的约定里，代价是"所有高度语义静默偏 30°"，那才是更深的缺陷 —— 只是不可见。真正属于缺陷级的只有**上游 `small_point_lio` 的共轭/双重变换**，而它是**第三方 bug**（§N.1：blame `98baa55` = 上游逐字节原样），且我们已修（`2a5a8b6`）。 |

> **换个说法（给用户的一句话）**：不是"我们的仿真架构有缺陷"，而是
> **"我们把一个 COD 藏在约定里的 30°，写进了一条显式 TF 边；显式的东西会被检查，于是我们查出了 6 个问题，
> 其中 2 个是上游 bug、4 个是我们自己的默认值/接线问题。"**
> 查出 6 个不是架构的错，是**架构更诚实**的代价；但"6 个里有 4 个是我们自己造的"这部分，必须修。

---

## 7. 可避免耦合清单

### 7.1 先列"固有的"（**不是我们的错，别去修**）

| # | 固有项 | 为什么固有 | 证据 |
|---|---|---|---|
| I1 | 斜装雷达**必须**被吸收在某处 | 物理事实。要么改 `base_link` 语义（COD），要么写一条 TF 边（我们）。没有第三条路 | §2.3 |
| I2 | `small_point_lio` 的**共轭**与**双重变换**是上游 bug | 本仓引入时与上游 `688d75c` 逐字节相同；`git blame` 全在 `98baa55` | `tilted_lidar_fidelity.md` §N.0 第 1 条、§N.1 第 3/6 行 |
| I3 | nav2 `obstacle_layer` 的 **z 伪造** | 上游设计：`LaserScan` 里没有高度信息，`projectLaser` 只能给 `z=0`。任何 2D 雷达消费者都逃不掉 | `obstacle_layer.cpp:337`、`observation_buffer.cpp:143-144` |
| I4 | `pointcloud_to_laserscan` 的 `min/max_height` 天然在 **target_frame** 里量 | 上游语义 | `multiplenav_launch.py:70-73` 与我们对 p2l 的用法同构 |
| I5 | 我们选 REP-103 的 `base_link`（车体系） | **有意的契约选择**，bench 必须同时服务 3D SLAM / ICP / cartographer / 4 种重定位 | `tf_interface_contract.md:44` |
| I6 | `base_link_fake` 这层"假帧" | **两边都有**（同源北极熊模式）。而且这个 trade 上 **COD 更疼**：它的 `base_link_fake` 继承了 30° pitch，我们的不会（我们 `base_link` 是平的） | COD `fake_vel_transform.cpp:44-51`（只做 yaw，pitch 透传）；我们 `tf_interface_contract.md:82-88`（T6 撤销说明） |

### 7.2 再列"我们的"（**可避免，且是我们自己引入的**）

| # | 耦合 | 证据 | 怎么来的 | 该不该修 |
|---|---|---|---|---|
| **O1** | **高度阈值住在 4 个 frame 里、没有唯一所有者**：nav2 `obstacle_layer` 源级（`min` 默认 `0.0`，量**伪造 z**）、STVL 源级（`0.0/2.0`，量真 z）、`linefit` 的 `sensor_height` + `gravity_aligned_frame`、p2l 的 `min/max_height` + `target_frame`、自击掩膜 z 门限 | `nav2_params_sim_base.yaml:180,196-197,250-251,354-355`；`traversability_self_mask_robot11.yaml`；`laserscan_params*` | 每加一个感知槽就加一组高度键，没有"谁拥有地面/高度语义"的裁决 | ✅ **修**（这是 §8 选项 B 的一半） |
| **O2** | **`lio_tf_adapter` 被 `lio:=small_point_lio` 排除** ⇒ 本仓唯一写对**合成**的实现，恰好在最需要它的槽位上不跑；`robot11` 的 30° `rpy` 补偿没有载体 | `bringup_sim.launch.py:2028-2033`、`:2051-2055`、`:2076-2080`（三处 `!= 'small_point_lio'`）、`:2087-2088` | 为了消除"两个 `odom→base_link` 发布者"（真实约束），选择了"排除 adapter"而不是"统一到 adapter" | ✅ **修**（§8 选项 B） |
| **O3** | **`robot11_mount` 三档并存**（`plugin` 默认 / `urdf` **诊断档·账是错的** / `sensor` 物理保真） | `bringup_sim.launch.py:1287-1301`（声明与描述）、`:507-533`（横幅） | 为了"证明物理斜装的后果"造了 `urdf` 档，之后没退役 | ✅ **修**（§8 选项 C：收敛为 1 默认 + 1 保真，`urdf` 不作一等选项） |
| **O4** | **我们启用了上游那条单位 `base_link→livox_frame`** | `src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` vs COD 的同文件 `:47`（**唯一差异**）；blame = `98baa55` | 上游自己 `return LaunchDescription([small_point_lio_node])`（死代码），我们把它**加进了返回值** | ✅ **修**（一行：删掉，或加注释"与 URDF 30° 关节互斥"）。**当前 bringup 不走这个 launch**（`bringup_sim.launch.py:1794-1806` 自建 Node）⇒ 是**潜伏**坑，不是活的 |
| **O5** | local costmap 的**默认值**把 `/scan` 当唯一 2D 源（`local_obstacle:=scan`），而传感器槽位允许 30° 斜装 | `bringup_sim.launch.py:1416-1425`；`nav2_params_sim_base.yaml:158-184` | 默认值是历史行为（"原行为，链路单点"），斜装槽位是后来加的 | ⚠️ **半修**：bench 需要这个槽位（A/B 要用），但**默认值**应随 `robot11_mount` 一起切，或至少在 `robot:=robot11` 时把 `min/max_obstacle_height` 的语义在文档/参数里显式声明为"传感器高度闸"（`nav2_params_sim_robot11_costmap.yaml:34-41` 已写了注释，但默认值没动） |
| **O6** | 30° 的**方向**（roll vs pitch）有两份互相矛盾的来源，且 `livox_tilt_axis` 默认 `roll` | `robot_models.md:653`（URDF 原文 `rpy="0 -0.5236 0"`= pitch vs SolidWorks CSV= roll）、`§9.5`（方位 360° ⇒ **仰角**测不出来） | 上游 URDF 与实物 CSV 冲突，我们选了 CSV | ⚠️ **保留但写死**：它只影响 `_LioAdapterRobot11` 的 rpy 分解，不影响辐条方向（§9.5 已证）；但**必须在一个地方写清"默认 roll，理由=X"** |

**小结：6 个可避免耦合，4 个明确该修（O1/O2/O3/O4），2 个半修（O5/O6）。**

---

## 8. 三个选项（按 成本/风险 排序）

> 目标锚定：**"算法对比 bench"，不是 COD 克隆**。所以判据不是"更像 COD"，而是
> "**每个坐标语义只有一个所有者、可 A/B、可回归**"。

### 选项 C（先做，成本 S，风险低）—— 三档收敛成"一默认 + 一保真"，删掉诊断档

**做法**
1. `robot11_mount` 的取值从 `plugin|urdf|sensor` 收敛为 **`plugin`（默认，唯一支持的常规档）** + **`sensor`（物理保真档）**；
   `urdf` 保留为**文档里的反例**（`tilted_lidar_fidelity.md` §C/§I 已有完整数字），
   **从 `_LIVOX_MOUNTS`（`bringup_sim.launch.py:172`）与 `choices=list(_LIVOX_MOUNTS)`（`:1303`）里移除**，
   并让 `_validate_livox_mount`（`:175-181`）给出可操作报错。
2. 在同一处声明里写清"`plugin` = `R_bl ≡ I` + 倾角在插件 `<tilt_rpy>`"是**唯一支持的默认**（这正是用户 Q6 选项 A 的字面内容，见下）。
3. 清 O4：`src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47` 去掉 `static_base_link_to_livox_frame`。

**修哪几条**：Q1（把"我们用哪种 30° 记账"钉死）、Q5 的 O3/O4/O6。
**成本**：改 1 个 launch 的 choices + 1 行 vendored launch + 文档；**不碰任何算法**。≤ 1 人日。
**风险**：低。`urdf` 档从来不是默认，去掉不影响任何已验证路径（§J.3 已证 `plugin`/`sensor` 之外逐字节不变）。
**验收测试**
```bash
# ① 档位只剩两个，且缺省仍是 plugin
ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args | grep -A2 robot11_mount
# ② 选 urdf 必须给出可操作报错（而不是静默跑）
ros2 launch ... robot:=robot11 robot11_mount:=urdf   # 期望：非零退出 + 明确提示
# ③ 静态：vendored launch 不再包含单位 base_link→livox_frame
grep -c static_base_link_to_livox_frame src/rm_localization/small_point_lio/launch/small_point_lio.launch.py   # 期望 0（或仅注释）
# ④ 回归：默认档行为逐字节不变
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5   # 期望 PASS/SUCCEEDED
```

---

### 选项 B（主体，成本 M，风险中）—— 让 `lio_tf_adapter` 成为**所有** LIO 槽的 `odom→base_link` 唯一权威 + 高度语义单一所有者

**B1. `odom→base_link` 单一权威（修 O2）**
- `lio:=small_point_lio` 时也启动 adapter；为消除"双发布者"，把 LIO 节点那条广播**关掉**。
  两条实现路径：
  * **B1a（推荐，需协调 agent A）**：给 `src/rm_localization/small_point_lio/**` 加一个参数
    `publish_tf: true|false`（默认 `true`，保持 §J/§N 的默认路径逐字节不变），bringup 传 `false`。
    ⚠️ **该目录当前归 agent A 所有，本任务不可改**，需跨 agent 协调。
  * **B1b（零改上游）**：保留 LIO 的广播，但把它作为 **`odom→base_link_lio`**（改 `child_frame_id` 需改代码，同样要 B1a 的权限）；
    或者**接受现状**并把 adapter 的 `base_frame` 设成 `base_link_fake`（下策，会绕开 `base_link` 语义，不推荐）。
- 之后 `_LioAdapterRobot11('rpy')` 的 30° 补偿对**所有** LIO 槽生效，**同一个数**只写一次。

**B2. 高度语义单一所有者（修 O1）**
- 明确裁决：**"什么算障碍"的最终裁决权在感知域（`/segmentation/obstacle`），代价图不再自己判高度。**
- 具体：`obstacle_layer`（`/scan` 路径）的 `min/max_obstacle_height` **要么删掉、要么在注释里定义为"传感器高度闸"**，
  并把"地面剔除"的唯一责任压到 `obstacle_near_ground_m`（`nav2_params_sim_robot11_costmap.yaml:42-44` 已指出这条路）。
- STVL 的 `0.0/2.0` 保持（它量真 z，且已登记是对 COD `0.15/1.0` 的有意偏离）。

**修哪几条**：Q2 全部、Q3 的一半（高度带所有权）、Q5 的 O1/O2。
**成本**：B1 需改 `small_point_lio`（1 个 `declare_parameter` + 1 个 `if`）+ launch 3 处条件 + adapter 参数；B2 是 config + 注释。**M（2–3 人日，含回归）**。
**风险**：中。B1 动的是"谁发 `odom→base_link`"这条契约的**唯一性**，必须配静态守卫（见验收 ①）。
**验收测试**
```bash
# ① 单一发布者不变式（对 (lio, robot, robot11_mount) 的笛卡尔积做静态检查）
python3 tools/scripts/regress/<新增>_tf_publisher_uniqueness.py   # 期望：每组恰好 1 个 odom→base_link 发布者
# ② 合成正确性：发布的 T_ob 与 /odom + base_link→livox_frame 用链式法则复算一致
ros2 run tf2_ros tf2_echo odom base_link            # 与 /odom 的 rpy 误差 < 0.5°（§J.1.2 的验收线）
# ③ 30° 补偿真的生效（robot:=robot11 + lio:=small_point_lio，sensor 档）
#    期望：/cloud_registered 地面倾角 < 1.5°（§A.2 判据），odom→base_link pitch 不随 30° 走
# ④ 端到端
python3 tools/scripts/regress/nav_smoke_regression.py --goal 2.0 -2.5   # PASS/SUCCEEDED
```

---

### 选项 A（不推荐，成本 L，风险高）—— 全面采用 COD 约定

> ⚠️ **先说清一件事：用户提的"A = 对齐 `base_link` 与传感器壳 / 发 `R_bl = I` + 把倾角搬进插件"
> —— 后半句我们**已经是这样了****（`robot11_mount:=plugin`：`base_link→livox_frame` rpy = 0/0/0，
> 倾角由插件 `<tilt_rpy>` 承担，`tilted_lidar_fidelity.md:22`、`:82`）。
> 所以 A 若只做到"R_bl = I + 倾角在插件"，**它已经实现了**；A 的**真正增量**只剩
> "把 0.13/0.157 m 的杆臂也折进 `base_link`（即把 `livox_frame` 的原点搬到 `base_link`）"。

**做法**：`base_link` 原点搬到雷达安装面、`base_link→livox_frame ≡ I`、`odom` 由"初始雷达系"定义。

**会坏掉什么（逐条）**
1. `robot_models.md:173` 的"`base_link` 原点 = 底盘最低点、z=0"整条几何基线作废 ⇒
   `robot_radius` / footprint / inflation 全部要按"原点在 0.26 m 高的雷达"重算；
2. `linefit` 的 `gravity_aligned_frame`、`p2l` 的 `target_frame`、自击掩膜、STVL 高度带
   **5 个消费者**的"重力对齐"假设全部要改成"斜系"（或全部显式加 `target_frame`）；
3. `tf_interface_contract.md:44` 的契约链要重写；`base_link_fake` 会继承 30° pitch（COD 的现状）；
4. cartographer / AMCL / ICP / GICP / slam_toolbox **6 个重定位槽**都按 REP-103 的 `base_link` 工作 ⇒ 全部要验；
5. `docs/` 里已实测的十几个基线（§A/§B/§D/§K/§L/§M 的数字）全部失去可比性。

**修哪几条**：Q1（改成 COD 的答案）、**Q2/Q3 反而变差**（把 §3.3 的共轭风险变成"必须靠约定"）、Q5 的 O1 会**更糟**（高度语义更不可见）。
**成本**：**L（≥ 1 周 + 全部基线重跑）**；**风险：高**。
**结论**：**不推荐**。它把"显式可测"换成"隐式不可见"，与"算法对比 bench"的目标相反。
**验收测试**（若真做）：`view_frames` 无 `livox_frame` 边 + 全部 6 个重定位槽的 `nav_smoke_regression` PASS — 成本极高，收益是"更像 COD"。

---

### 排序一览

| 排 | 选项 | 修 Q1 | 修 Q2 | 修 Q3 | 修 Q5 | 成本 | 风险 | 建议 |
|---|---|---|---|---|---|---|---|---|
| **1** | **C**（三档收敛 + 清 O4） | ✅ 钉死约定 | — | — | O3/O4/O6 | S | 低 | **先做** |
| **2** | **B**（adapter 唯一权威 + 高度单一所有者） | — | ✅ 全部 | ✅ 一半 | O1/O2 | M | 中 | **主体** |
| 3 | A（照抄 COD） | ✅ 但换成错答案 | ❌ 更差 | ❌ 更差 | ❌ | L | 高 | **不做** |

---

## 9. 未验证 / 诚实清单

| # | 未验证项 | 为什么没验 | 影响 |
|---|---|---|---|
| 1 | **COD 真机 URDF 里 `base_link→livox_frame` 的真实数值** | 本地 checkout 只有 `src/`（Gitee 2026 仓），**没有 `urdf/`**；GitHub `rmul2026` 的 `urdf/` 不在本地 | §2/§3 的整个推理链依赖"它≈单位/纯平移"。**这是推断，不是实测**。反证：若它不是单位，COD 的 `odom→base_link` 按 §3.3 会明显错，其 nav2 不可能正常工作 ⇒ **旁证很强但不等于取证** |
| 2 | **COD 实车的雷达是否真的斜 30°、绕哪个轴、朝哪个方向** | 同上（无 URDF/无标定文件）；`docs/robot_models.md:653` 记录**我们**的 URDF 与 CSV 冲突，COD 的情况未知 | 影响 §4.5 的"斜板"方向与量级；**不影响 §4.4**（无 `obstacle_layer` 是结构事实） |
| 3 | **COD 的 `odom` 是否真的随雷达斜 30°** | 从 `eskf.h:30` + `small_point_lio.cpp:54-64` **推**出来的，**没有在 COD 数据上实测**（没有 bag、没有他们的 TF 录制） | §4.5 的三条后果是**推断**。若 `gravity` 状态被后续观测强烈拉回真垂直，则 `odom` 会渐进重力对齐，结论要改 |
| 4 | **COD 的 `base_link→livox_frame` 若缺失，`:65` 的 `lookupTransform` 会抛异常 ⇒ 直接 `return`（`:68`）** | 需要实跑才知道他们的图里到底有没有这条边 | 若真缺失：COD 实际是"没有任何 `odom→base_link`"，那 nav2 靠别的东西（例如他们仓库外的静态桥）—— 这会推翻 §2.1(c) 的表述顺序，但不推翻结论 |
| 5 | **`cpp_lidar_filter` 的裁剪盒数值究竟是按哪个系标的** | `filter_node.cpp` 作用在输入点云系、marker 发在 `base_link`（`:56` vs `:117`），源码自相矛盾；上游 README 也自相矛盾（`cod_nav_nav_stack_inventory.md:169` 已登记） | 只影响"他们有没有一个隐藏的 30° 记账"这一条旁证 |
| 6 | **COD 是否另有仓库外的启动项提供了 `base_link→livox_frame` 或 `urdf`** | 他们实车的 `nav.sh` / 上层 ws 不在本地 | 同 #4 |
| 7 | **`robot:=robot11` + `lio:=small_point_lio` 组合今天的实际数字** | `src/rm_localization/small_point_lio/**` 正在被 **agent A** 改（`2a5a8b6` 已修 ②③ 的姿态，**平移按 §J.5 有意保留旧值**）；本任务**不跑仿真** | §3.5/§8-B 的"今天是否已经对"取决于 agent A 的收尾状态。本文按**当前 HEAD** 的工作树读源码（未提交改动不算） |
| 8 | **本文引用的 `docs/tilted_lidar_fidelity.md` §J/§N 行号** | 该文件正被 **agent A** 编辑，行号可能已漂移 | 引用时以**节号**（§J.1.2 / §N.0 / §N.1）为准，行号仅作参考 |
| 9 | **选项 B1a 的可行性** | 需要改 `src/rm_localization/small_point_lio/**`，该目录**不属于本任务权限**（并发 agent A） | B1a 是"实现建议"，落地需跨 agent 协调；本任务只给方案与验收 |
| 10 | **COD 的 `map→odom` 静态 TF 的 `yaw=-0.5` 从哪来** | 无注释、无标定文件 | 与本文主题无关，仅登记 |

---

## 附：本文引用的本地 COD 文件清单（全部可读，非 submodule）

```
third_party/cod_nav_2026/src/
├── small_point_lio/
│   ├── launch/small_point_lio.launch.py            ← 单位 base_link→livox_frame 是死代码（:47）
│   ├── config/mid360.yaml                          ← lidar_frame=livox_frame，extrinsic_R=I
│   └── src/
│       ├── small_point_lio_node.cpp                ← odom→base_link 的**共轭**（:75）+ 点云双重变换（:159）
│       └── small_point_lio/{eskf.h,small_point_lio.cpp}   ← R_W_B(0)=I（eskf.h:30）、gravity 取 body 系测量（:54-64）
├── cod_bringup/
│   ├── launch/{singlenav,multiplenav,localization,navigation}_launch.py
│   └── params/{singlenav2,multiplenav2}_params.yaml ← obstacle_layer 命中 0、/scan 命中 0
├── cpp_lidar_filter/src/filter_node.cpp             ← CropBox 不做 TF，marker 却发 base_link
├── fake_vel_transform/src/fake_vel_transform.cpp    ← base_link→base_link_fake 仅 yaw
└── pointcloud_to_laserscan/                         ← 多模式才用，target_frame=base_link
```
