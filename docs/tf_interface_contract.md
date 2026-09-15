# TF / 接口契约设计（草案，待评审）

> 目标：理清"谁发布哪条 TF 边、谁提供哪个话题"，把现在用于"硬凑帧树"的静态桥替换成正式契约。
> 状态：**设计文档，尚未改代码**。本文所有"现状"结论均来自对源码的实测（附文件/行号）。
> 关联：`docs/architecture.md`（总览）、`docs/rm_bench_refactor_plan.md`（§6 待修 #4 TF 桥）、`THIRD_PARTY_NOTICES.md`。

---

## 一、现状：谁在发布什么（实测）

| 发布者 | TF 边 | 话题 | 证据 |
|---|---|---|---|
| Gazebo 底盘插件 `mecanum_controller` | **`odom → base_link`**（`publish_odom_tf=true`） | `/odom` | `rm_nav_bringup/urdf/sentry_robot_sim.xacro`（`odometry_frame=odom`、`robot_base_frame=base_link`、`publish_odom_tf=true`） |
| `fast_lio` | **`camera_init → body`** | `/Odometry`（header `camera_init` / child `body`） | `FAST_LIO/src/laserMapping.cpp` L630-631、L648-658 |
| `point_lio` | **`camera_init → aft_mapped`** | `aft_mapped_to_init`（header `camera_init` / child **`body`**） | `point_lio/src/laserMapping.cpp` L245-246、L258-259 |
| `slam_toolbox`（localization） | **`map → odom`** | `/map`、`/scan` | `mapper_params_localization_sim.yaml`（`map_frame=map`、`odom_frame=odom`、`base_frame=base_link`） |
| `amcl` | **`map → odom`** | `/amcl_pose` | `nav2_params_sim.yaml`（`global_frame_id=map`、`odom_frame_id=odom`、`base_frame_id=base_link`） |
| `icp_registration` | **无 TF**（`map→odom` 广播整段被注释） | 无 | `icp_registration.cpp` L118-125（`timer_` 与 `sendTransform` 全被注释） |
| `fake_vel_transform` | **`base_link → base_link_fake`**；还做 `/cmd_vel` → `/cmd_vel_chassis` 速度变换 | `/cmd_vel`、`/cmd_vel_chassis` | `fake_vel_transform/src/*.cpp` L11-12、L67、L88-89 |
| `bringup_sim` 三条**静态桥** | `camera_init→map`、`body→odom`、`base_link→base_link_fake` | — | `bringup_sim.launch.py`（LIO 启用时启动） |
| `robot_state_publisher` | URDF 固定关节（`base_link`↔`livox_frame`/`imu_link` 等） | `/tf_static` | `sentry_robot_sim.xacro` |

**消费方期望**：nav2 `global_costmap`：`global_frame=map`、`robot_base_frame=**base_link_fake**`；`local_costmap`：`global_frame=odom`、`robot_base_frame=base_link_fake`；AMCL/slam_toolbox 都以 `base_link` 为底盘帧。

---

## 二、问题清单（都是实测出来的）

| # | 问题 | 后果 |
|---|---|---|
| **P1** | 静态 `camera_init→map` 与 AMCL/slam_toolbox 的 `map→odom` **并存** → `map`/`odom` 出现多个父边（帧树闭环/非法） | TF 树冲突、`view_frames` 报警、定位/代价地图偶发丢帧 |
| **P2** | 两套 LIO 输出不一致：话题 `‎/Odometry` vs `aft_mapped_to_init`；TF 子帧 `body` vs `aft_mapped` | 下游无法用统一接口消费，只能靠帧桥硬凑 |
| **P3** | `icp_registration` **不发布 `map→odom`**（广播被注释） | `localization:=icp` 时 `map↔odom` 只能靠静态桥凑，ICP 的配准结果实际没进 TF 树 |
| **P4** | `base_link→base_link_fake` 有**两个发布者**：`fake_vel_transform` + 静态桥 | 同一条边双发布，TF 抖动/覆盖 |
| **P5** | sim 里 **Gazebo 也发 `odom→base_link`**，与 LIO 定位争夺同一条边（若把 LIO 改名为 odom→base_link 必然冲突） | 定位与真值混用，仿真验证结论不可信 |
| **P6** | nav2 用 `base_link_fake` 作为 `robot_base_frame`，而非标准 `base_link` | 多一层"假帧"，与 URDF/TF 语义脱节 |

---

## 三、目标契约（标准 ROS 导航帧树）

```
map ──(重定位：AMCL / slam_toolbox / ICP 之一发布)──► odom ──(里程计：LIO 发布)──► base_link ──(URDF 固定关节)──► livox_frame / imu_link …
```

| 角色 | 唯一职责 | 禁止事项 |
|---|---|---|
| 里程计（`fast_lio` / `point_lio`） | 发布 **`odom→base_link`** + 统一话题 **`/odom`** | 不得发布 `map→*`；不得用 `camera_init`/`body`/`aft_mapped` 这些内部帧名对外 |
| 重定位（AMCL / slam_toolbox / ICP） | 发布 **`map→odom`** | 不得发布 `odom→base_link` |
| 底盘仿真（Gazebo 插件） | 只发布 `/cmd_vel` 的执行与**可选**调试用真值（默认**不发 TF**） | 不与 LIO 争 `odom→base_link` |
| `fake_vel_transform` | 只做 `/cmd_vel` → `/cmd_vel_chassis` 速度变换 | **不再发布任何 TF** |
| 装配层（bringup） | 只做 remap / 参数注入 | **不再用静态 TF 桥凑帧树** |

---

## 四、候选方案

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）适配节点重命名** | 新增自研小组件 `lio_tf_adapter`：订阅 LIO 里程计（bringup 侧把两套话题 remap 成 `/odom`），按其位姿发布标准 `odom→base_link`；删除三条静态桥；启用 ICP 的 `map→odom`；sim 关闭 Gazebo `publish_odom_tf`；nav2 `robot_base_frame` 改回 `base_link` | 不改上游 LIO 代码；一次解决 P1–P6；契约清晰 | 多一个小节点；需回归 nav2 参数（footprint/膨胀） |
| **B（彻底）** | 在自有 fork 里把两个 LIO 的帧名/话题**参数化**，直接输出 `odom→base_link`、`/odom` | 最干净、无适配层 | 要改并维护两个上游 fork；合并上游更新成本上升 |
| **C（过渡，最小改动）** | 保留静态桥，但**按 `localization` 值条件化**：`amcl`/`slam_toolbox` 模式不启动 `camera_init→map`、`body→odom`（只 icp 模式启动）；并删掉重复的静态 `base_link→base_link_fake` | 立刻消除 P1/P4 冲突；改动极小 | 不解决 P2/P3/P5/P6 |

**建议路径**：先落 **C**（当天可完成，消除最危险的帧树冲突）→ 再做 **A**（正式契约）→ 远期若维护成本可接受再看 **B**。

---

## 五、分阶段实施（每步可独立验收）

| 阶段 | 内容 | 验收 |
|---|---|---|
| **T1 ✅（2026-09 已实施）** | 帧桥改为**仅 `mode:=nav` + `localization:=icp` + 启用 LIO** 时启动；删除重复的静态 `base_link→base_link_fake`（由 `fake_vel_transform` 20Hz 独占发布） | P1/P4 消除：amcl/slam_toolbox 模式不再与静态桥争 `map`/`odom` |
| **T2 ✅（2026-09 已实施）** | 统一里程计话题：bringup 把 `fast_lio` 的 `/Odometry`、`point_lio` 的 `aft_mapped_to_init` 都 **remap 为 `/odom`** | `ros2 topic hz /odom` 在两种 LIO 下都连续；TEB 变体的 `odom_topic: /odom` 生效 |
| **T3** | 新增 `lio_tf_adapter`，输出标准 `odom→base_link`；删除全部静态桥 | `tf_echo odom base_link` 连续；帧树只剩标准边 |
| **T4** | sim URDF 关闭 Gazebo `publish_odom_tf`（真值仅保留 `/odom` 话题供对比） | 无"双发布者"警告；LIO 定位成为唯一位姿源 |
| **T5** | 启用 `icp_registration` 的 `map→odom` 广播（修好被注释的定时器/或改为每次配准后发布） | `localization:=icp` 时 `map→odom` 由 ICP 提供 |
| **T6** | nav2 `robot_base_frame` 由 `base_link_fake` 改回 `base_link`（并回归 footprint/速度参数） | nav2 各组合实跑通过 |

---

## 六、验收清单（每次改动后跑）

```bash
ros2 run tf2_tools view_frames.py          # 检查：无多父、无闭环、边符合 §三
ros2 run tf2_ros tf2_echo odom base_link   # 连续、无跳变
ros2 run tf2_ros tf2_echo map odom         # 由重定位模块提供
ros2 topic hz /odom                        # 与 LIO 频率一致
ros2 lifecycle get /controller_server      # nav2 生命周期 active
```

组合烟测：`mapping/nav × fastlio/pointlio × amcl/slam_toolbox/icp` 以及 `lio:=none` + cartographer。

---

## 七、风险与回滚

| 风险 | 说明 | 回滚 |
|---|---|---|
| 改 sim URDF（关 `publish_odom_tf`） | 会影响所有依赖真值 odom 的调试 | 恢复 `publish_odom_tf=true`（一行） |
| 改 nav2 `robot_base_frame` | 影响 footprint/膨胀/TEB 行为 | 恢复 `base_link_fake` + 静态桥 |
| 新增适配节点 | 引入一个新的失败点（但逻辑极小） | 停用该节点，恢复静态桥 |

> 注：`camera_init`/`body`/`aft_mapped` 是两套 LIO 的**内部帧名**，改造后它们只应存在于 LIO 进程内部，不再泄漏到导航层。
