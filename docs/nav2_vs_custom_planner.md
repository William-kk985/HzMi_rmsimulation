# 纯 C++ 规划库接进 nav2：三种接法、契约、效率账与落地坑

> **本文回答的问题**（用户原话）：「这个自己研制怎么自己研制呢，因为这个纯 c++吧，我们这个又算强依赖 nav2 吧，做成 nav2 的插件吗，还是有办法保持纯 c++ 的形态，如果要做成 nav2 的插件或者融入 nav2 会不会影响原来纯 c++ 的运行效率呢」。
>
> **被评估的库**：`T-DT-Algorithm-2026/tdt-nav-kit`（RM 哨兵导航前后端：`YAstar` = A\*/Kinodynamic A\*，`MinimumSnapOsqp` = min-snap + 方形走廊（SFC）轨迹优化）。纯 CMake、**无 ROS、无 `package.xml`**，依赖 Eigen3 + OSQP/OsqpEigen，OpenCV 只在 `main.cpp` 示例里用。
> **我们的栈**：ROS 2 Humble + nav2 **1.1.20**，槽位化（`planner:=navfn|smac2d`、`nav:=rpp|dwb|teb|mppi`、`localization:=…`、`lio:=…`），已有 A/B 回归 harness（`tools/scripts/regress/`）。
>
> **证据标注约定**（全文逐条执行）：
> - 【源码】= 直接读到的代码/消息定义，给文件:行（本地安装的 Humble 头文件、`humble` 分支源码、上游仓库源码）；
> - 【文档】= 官方文档页面（给 URL）；
> - 【第三方】= 第三方仓库/论文（给 URL）；
> - 【我们的实测/文档】= 本仓库 `docs/`、`tools/` 里的既有结论；
> - 【无来源】= **没有找到可引用来源**，只给"要settle它必须测什么"；
> - 【推算】= 由消息定义/常数算出来的量级，不是实测。
>
> **版本警告（很重要）**：`docs.nav2.org` 已被重组为**按发行版分目录**，目前只有 `/rolling/` 与 `/jazzy/`（`/humble/` 返回 404，见 §9 未核实项）。因此 **Rolling 文档里的插件接口与 Humble 1.1.20 有实质差异**（例如 Rolling 的 `createPlan()` 多两个参数、参数键从 `planner_plugins` 改成 `plugins`、Controller 用 `newPathReceived()` 取代 `setPlan()`、多了 path handler 插件类型）。**本文凡涉及契约，一律以本机 `/opt/ros/humble/include/` 的头文件与 `humble` 分支源码为准**，Rolling 文档只用于"语义解释/命名来源"，差异处逐一标注。

---

## 0. 一页结论

### 0.1 决策表（三种接法）

| 维度 | **A. nav2 插件**（planner / smoother / controller） | **B. 独立节点**（自己吃 costmap、自己发 `/cmd_vel`） | **C. 独立进程 + 共享内存/零拷贝 IPC** |
|---|---|---|---|
| 要写多少代码 | 一个薄适配类（Humble 下 `GlobalPlanner` 只有 5 个纯虚函数，其中 4 个可留空）+ pluginlib xml + 参数段 | 一个完整节点：TF/位姿获取、costmap 源、控制律、到达判据、恢复策略、生命周期 | B 的全部 + 共享内存/自定义 RMW 或自定义 IPC 协议 + 进程编排 |
| 库是否还是纯 C++ | **是**。库代码一行不改，ROS 依赖只出现在适配层（§7.3 的 CMake 骨架） | **是**。库被编进一个 ROS 节点，但库本身仍无 ROS 头文件 | **是**（库甚至可以编成独立可执行文件/服务） |
| 每次规划的额外开销 | 一次虚函数调用 + action server 收发的消息（见 §4：**这些都不是主要成本**）；costmap 是**指针原地读**（planner/controller 形态） | 取决于 costmap 从哪来：订阅 `costmap_raw` 会**序列化+拷贝**；自己在进程内跑 costmap 层则等价于 A | 一次 IPC 传输。**同进程**才能真零拷贝；跨进程只能靠 SHM/loaned message（§4.5） |
| 一次性开销 | pluginlib/`dlopen` + `configure()`（生命周期期一次，见 §1.1） | 进程启动、DDS 发现 | 进程启动、SHM 段建立、发现 |
| 必须自己重实现的东西 | 只在插件语义范围内：planner 只需 `createPlan`；controller 需要自己写跟踪律与到达判据 | nav2 帮你做的一切：行为树/恢复、目标管理、costmap 图层与 TF 维护、`/cmd_vel` 独占、进度检查 | 同 B + IPC 编解码/一致性 |
| 与 nav2 生态的兼容 | 最好：RViz/`/plan`/BT/waypoint/`nav2_simple_commander` 全都还在 | 差：nav2 的 BT、waypoint follower、碰撞监控、速度平滑都需要重新接线 | 中：nav2 侧保留，只是规划在别的进程 |
| 我们栈的接入成本 | **最低**：新增 `planner:=tdt` 槽位、一个 yaml 段，直接复用现有 A/B harness | 中：需要新 launch、新的 `/cmd_vel` 归属决策（§6.1） | 高：需要自定义中间件/消息，收益未被任何来源证明 |
| 何时选它 | **默认选它**：想要库的算法 + 保留 nav2 的工程能力（恢复、waypoint、可视化、参数化） | 当 nav2 的**抽象与问题不匹配**（地形不是"空地/障碍"二值、底盘动力学不是差速/全向）时；这是 RM 圈的现实选择（§5.2 HWSentryNav26 作者原话） | 只有在**已证明**规划耗时/抖动是瓶颈、且同进程方案不可行时。**本次没有找到任何论文/仓库用它来跑规划器**（§9） |

### 0.2 对本次这个库的具体建议（一句话版）

**把库保持原样（纯 CMake、无 ROS），在它旁边加一个 ament 包做薄适配；先做 `GlobalPlanner` 插件（A\* 前端 + min-snap 后端，输出 nav2 的 `nav_msgs::Path`），把"带时标的 min-snap 轨迹真正被执行"这件事留给后续的 Controller 插件或独立节点**——因为 nav2 **原生没有任何"带时间的轨迹"接口**（§3），min-snap 的时标在 `Path` 里必然丢失。CMake 骨架见 §7.3，分阶段计划见 §7.4。

**效率结论（详见 §4）**：插件化本身的开销（dlopen 一次性、虚函数调用每次）在量级上**远小于**这个库自己的耗时（A\* 平均 5.4 ms / 最大 14.8 ms；min-snap 平均 3.9 ms / **最大 126.7 ms**）。真正要担心的不是"插件 vs 直调"，而是三件事：① **min-snap 的最坏 126.7 ms** 落在 20 Hz 控制环里会顶掉 2–3 拍；② costmap 的**取用方式**（指针原地读 vs 订阅 `costmap_raw` 拷贝 168 KB 级数据）；③ **执行器/回调组**导致的抖动与 `setPlan` 之后的重算节奏。

---

## 1. 问题一：nav2 插件契约，精确版（以本机 1.1.20 为准）

### 1.1 三个接口的逐字签名（Humble 1.1.20）

`nav2_core::GlobalPlanner`（本机 `/opt/ros/humble/include/nav2_core/global_planner.hpp`），5 个纯虚函数：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;   // 4 个入参
virtual void cleanup() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual nav_msgs::msg::Path createPlan(                            // 注意：Humble 只有 2 个入参
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) = 0;
```

> **与 Rolling 文档的差异**：Rolling 教程表述为"base class provides 5 pure virtual methods…`createPlan()` 带 4 个入参（start、goal、viapoints、取消检查函数）"【文档】<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>。**Humble 没有 viapoints 参数**——viapoints 由 `planner_server` 自己在 `computePlanThroughPoses()` 里拼接（`nav2_planner/src/planner_server.cpp` 的 `computePlanThroughPoses`，本机 Humble 分支源码）。

`nav2_core::Smoother`（本机 `.../nav2_core/smoother.hpp`）：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,   // ← 注意：不是 Costmap2DROS
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;
virtual void cleanup() = 0;  virtual void activate() = 0;  virtual void deactivate() = 0;
virtual bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) = 0;  // in-out + 时间上限
```

`nav2_core::Controller`（本机 `.../nav2_core/controller.hpp`）：

```cpp
virtual void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
virtual void cleanup() = 0;  virtual void activate() = 0;  virtual void deactivate() = 0;
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(   // 返回值是 TwistStamped
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker) = 0;                        // 3 个入参
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

> **与 Rolling 文档的差异**：Rolling 把 `setPlan()` 换成了 `newPathReceived()`，并且 `computeVelocityCommands()` 变成 5 个入参（多了 `transformed_global_plan` 与 `global_goal`），由 path handler 负责把全局路径变换/裁剪【文档】<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/>。Humble 里 **`setPlan()` 是唯一入口**，路径变换要控制器自己做（我们栈里 TEB/RPP/MPPI 都是自己 `transformGlobalPlan`）。

`GoalChecker` / `ProgressChecker`（本机 `.../nav2_core/goal_checker.hpp`、`progress_checker.hpp`）：`initialize(parent, plugin_name, costmap_ros)` / `reset()` / `isGoalReached(query_pose, goal_pose, velocity)` / `getTolerances(...)`；progress checker 是 `initialize(parent, plugin_name)` / `check(PoseStamped&)` / `reset()`。**两个 checker 都不需要 `configure/activate/deactivate`**——它们不是生命周期插件。

### 1.2 服务端怎么加载、什么时候调用（决定了"一次性 vs 每次"）

`nav2_planner/src/planner_server.cpp`（Humble 分支，本地已下载核对）：

- 构造：`gp_loader_("nav2_core", "nav2_core::GlobalPlanner")` —— **pluginlib 的"包名"参数是 `nav2_core`，不是 `nav2_planner`**（`planner_server.cpp` 构造函数）。
- 参数：`declare_parameter("planner_plugins", default_ids_)`、`declare_parameter("expected_planner_frequency", 1.0)`；**只有**在 `planner_plugins == 默认值` 时才补声明 `default_ids_[i] + ".plugin"`（`planner_server.cpp` 约 54–62 行）。真实插件类型由 `nav2_util::get_plugin_type_param(node, <id>)` 读取，它读的就是 **`<id>.plugin`**，**读不到会 `exit(-1)` 直接杀进程**（本机 `/opt/ros/humble/include/nav2_util/node_utils.hpp:135-149`）。
- 加载时机：`on_configure()` 里逐个 `createUniqueInstance(type)` → `planner->configure(node, id, tf_, costmap_ros_)`（`planner_server.cpp` 约 110–122 行）。**`dlopen` + 构造 + `configure` 都只发生一次**；`activate()/deactivate()` 只在生命周期切换时调用；**每次规划只有一次 `planners_[id]->createPlan(start, goal)` 虚函数调用**（`getPlan()`，约 518–546 行）。
- 计时口径：`result->planning_time = this->now() - start_time`，`start_time` 在动作回调开头取，**包含**等 costmap（`waitForCostmap()` 里 `while(!costmap_ros_->isCurrent()) r.sleep()`）、TF 变换（`transformPosesToGlobalFrame`）、路径校验（`validatePath`）、以及插件调用（约 419–437、270–280 行）。所以 `planning_time` **不是**插件内部耗时，做 A/B 时不要拿它当"库耗时"。
- 频率守卫：`expected_planner_frequency` > 0 时 `max_planner_duration_ = 1/f`，超时打 warning（约 133–141 行）。

`nav2_controller/src/controller_server.cpp`（Humble 分支）：

- 三个 loader：`progress_checker_loader_("nav2_core","nav2_core::ProgressChecker")`、`goal_checker_loader_("nav2_core","nav2_core::GoalChecker")`、`lp_loader_("nav2_core","nav2_core::Controller")`。
- 参数键（**精确**）：`controller_frequency`、`progress_checker_plugin`（**单数，字符串，不是 vector**）、`goal_checker_plugins`（复数 vector）、`controller_plugins`（vector）、`min_x/y/theta_velocity_threshold`、`failure_tolerance`、`publish_zero_velocity`、`speed_limit_topic`（约 39–63 行）。
  > ⚠️ 第三方参数文件里常见 `progress_checker_plugins: [...]`（例如 PolarBear 的 Jazzy 配置，见 §5.1），**Humble 1.1.20 不认这个键**——写了不会报错，只是被忽略、回落到默认 `progress_checker`。
- 控制环：`rclcpp::WallRate loop_rate(controller_frequency_)`（**墙钟**），每拍 `updateGlobalPath()` → `computeAndPublishVelocity()` → 到达判定 → `loop_rate.sleep()`；miss 掉目标频率会打 "Control loop missed its desired rate" 并 `loop_rate.reset()`（约 386–421 行）。
- `setPlan()` 的调用时机：`FollowPath` 动作**开始时**调一次（`setPlannerPath(action_server_->get_current_goal()->path)`，约 382 行），或**被抢占时**调一次（`updateGlobalPath()`，约 538–563 行）。**不是每拍都调**。
- `/cmd_vel` 归属：`vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1)`（约 198 行）；发布时还会**再拷贝一次** `velocity.twist` 进新 Twist（`publishVelocity()`，约 567–574 行），并且**只在有订阅者时**才发。异常/到达时会 `publishZeroVelocity()`。

`nav2_smoother/src/nav2_smoother.cpp`（Humble 分支）：参数 `smoother_plugins`（vector）、`costmap_topic`（默认 `global_costmap/costmap_raw`）、`footprint_topic`（默认 `global_costmap/published_footprint`）、`robot_base_frame`、`transform_tolerance`；`on_configure()` 里建 `CostmapSubscriber`/`FootprintSubscriber`/`CostmapTopicCollisionChecker`，再 `loadSmootherPlugins()`；动作回调里 `smoothers_[id]->smooth(result->path, goal->max_smoothing_duration)`，`was_completed` 与 `smoothing_duration` 回填结果。

### 1.3 action / 消息契约（本机 `nav2_msgs` 定义逐字）

`nav2_msgs/action/ComputePathToPose.action`：

```
#goal
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start        # false 时用机器人当前位姿
---
#result
nav_msgs/Path path
builtin_interfaces/Duration planning_time
---
#feedback            # 空
```

`nav2_msgs/action/SmoothPath.action`：goal = `nav_msgs/Path path` + `string smoother_id` + `builtin_interfaces/Duration max_smoothing_duration` + `bool check_for_collisions`；result = `nav_msgs/Path path` + `builtin_interfaces/Duration smoothing_duration` + `bool was_completed`。

`nav2_msgs/action/FollowPath.action`：goal = `nav_msgs/Path path` + `string controller_id` + `string goal_checker_id`；result = `std_msgs/Empty result`；feedback = `float32 distance_to_goal` + `float32 speed`。**注意 result 是空的**——控制器不回报轨迹，只回报"跟随结束"。

`nav_msgs/msg/Path`（本机）：`std_msgs/Header header` + `geometry_msgs/PoseStamped[] poses`——**只有几何，没有时间**（详见 §3）。

### 1.4 costmap 是怎么交到插件手里的（关键）

| 接口 | 拿到的东西 | 类型/生命周期 | 能否原地读 |
|---|---|---|---|
| `GlobalPlanner::configure` | `std::shared_ptr<nav2_costmap_2d::Costmap2DROS>` | 由 `planner_server` 在 `on_configure` 里 `make_shared` 并**常驻**；`planner_server` 还把它放进自己的 `nav2_util::NodeThread costmap_thread_` 里 spin（`planner_server.cpp` 约 65、94 行）⇒ **costmap 的更新在另一个线程** | ✅ `costmap_ros->getCostmap()` 返回 **`Costmap2D *`（裸指针，无拷贝）**，本机头文件注释："Same as calling `getLayeredCostmap()->getCostmap()`"（`costmap_2d_ros.hpp:222-227`） |
| `Controller::configure` | 同上（`Costmap2DROS` 共享指针） | 同上（`controller_server` 也有 `costmap_thread_`） | ✅ 同上 |
| `Smoother::configure` | `CostmapSubscriber` + `FootprintSubscriber` | 订阅 **`nav2_msgs/msg::Costmap`** 话题（默认 `global_costmap/costmap_raw`），内部持有一份 `std::shared_ptr<Costmap2D> costmap_` 和 `nav2_msgs::msg::Costmap::SharedPtr costmap_msg_` | ⚠️ **是消息、是拷贝**：`CostmapSubscriber::getCostmap()` 先 `toCostmap2D()`（含 `resizeMap`）再返回内部那份共享 `Costmap2D`（`nav2_costmap_2d/src/costmap_subscriber.cpp` Humble 分支，`getCostmap`/`toCostmap2D`/`resizeMap` 三处） |

主栅格怎么读（本机 `nav2_costmap_2d` 头文件）：

- `Costmap2D::getCharMap()` → `unsigned char *`，**底层那张 master grid**（`costmap_2d.hpp:232`）；
- `getSizeInCellsX()/getSizeInCellsY()`、`getResolution()`、`getOriginX()/getOriginY()`（`costmap_2d.hpp:238/244/274/262/268`）；
- `getCost(mx,my)` / `getCost(index)`（`costmap_2d.hpp:149/156`）；
- **同步原语**：`typedef std::recursive_mutex mutex_t; mutex_t * getMutex()`（`costmap_2d.hpp:361-365`）。nav2 自己就读它：`planner_server::isPathValid()` 里 `std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));` 然后逐点取 `getCost()`；`costmap_2d_ros.cpp` 在装图层时同样"持有 costmap 锁再初始化插件"，注释写着 *"lock the costmap because no update is allowed until the plugin is initialized"*（Humble 源码）。
- 值的语义（本机 `cost_values.hpp:42-46`）：`NO_INFORMATION = 255`、`LETHAL_OBSTACLE = 254`、`INSCRIBED_INFLATED_OBSTACLE = 253`、`FREE_SPACE = 0`。⚠️ **这与 `nav_msgs/OccupancyGrid` 的语义（0 空闲 / 100 占用 / −1 未知）完全不同**，也不是 `nav2_msgs/Costmap.msg`（`uint8[] data`）。
- **膨胀层没有独立 API**：膨胀是 `inflation_layer` 直接写进同一张 master grid；想拿到"图层对象"只能 `costmap_ros->getLayeredCostmap()->getPlugins()`（返回 `std::vector<std::shared_ptr<Layer>> *`，`layered_costmap.hpp:133`）再 `dynamic_pointer_cast`。**没有任何稳定的、带版本保证的"读膨胀层参数"接口**——所以规划器通常只读 master grid 的代价梯度，而不是去问膨胀层。

### 1.5 你要写的那几行"注册样板"（Humble 精确版）

`nav2_navfn_planner` 是最小可抄样本（本机 `/opt/ros/humble/share/nav2_navfn_planner/`）：

```xml
<!-- global_planner_plugin.xml -->
<library path="nav2_navfn_planner">
  <class name="nav2_navfn_planner/NavfnPlanner" type="nav2_navfn_planner::NavfnPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description></description>
  </class>
</library>
```

```cmake
# CMakeLists.txt（nav2_navfn_planner 的原始写法）
add_library(${library_name} SHARED ...)
ament_target_dependencies(${library_name} ...)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
install(FILES global_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
```

```xml
<!-- package.xml -->
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/global_planner_plugin.xml" />
</export>
```

```cpp
// 源文件末尾
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
```

【源码】`nav2_navfn_planner/global_planner_plugin.xml`、`CMakeLists.txt`、`src/navfn_planner.cpp` 末行（本机安装 + Humble 分支）；【文档】同样的四件套也写在插件教程里 <https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>。**`<class name>` 用 `包名/类名`（斜杠）是 Humble 的惯例；Rolling 是 `包名::类名`**（第三方参数文件里那句注释 *"In Iron and older versions, `/` was used instead of `::`"* 是同一件事的现场记录，见 §5.1 引用的 PolarBear yaml）。

> **`pluginlib`/`class_loader` 是什么**：`class_loader` 是 "ROS-independent package for loading plugins during runtime"，"utilizes the host operating system's runtime loader to open runtime libraries (e.g. .so/.dll/.dylib files), introspect the library for exported plugin classes"【文档】<https://github.com/ros/class_loader>（`ros2` 分支 README）。`pluginlib` 是它的 ROS 封装。**换句话说：插件机制本身就只是 `dlopen`+`dlsym`+虚表，不引入新的运行期抽象。**

---

## 2. 问题二的答案（先说结论）

**nav2 原生不支持"带时间的轨迹"（min-snap/min-jerk 那种"每段有持续时间、带速度/加速度约束"的对象）。** 证据（逐条可查）：

1. **消息层：没有这种消息。** 本机 `nav_msgs/msg/Path.msg` 全文只有：
   ```
   std_msgs/Header header
   geometry_msgs/PoseStamped[] poses
   ```
   没有任何 duration/time-allocation 字段（`geometry_msgs/PoseStamped` 自带 `header.stamp`，但见第 4 条）。本机 `nav2_msgs/msg/` 下的全部 14 个消息里**没有 Trajectory 类消息**（`BehaviorTreeLog, BehaviorTreeStatusChange, CollisionMonitorState, CostmapFilterInfo, CostmapMetaData, Costmap, EdgeCost, ParticleCloud, Particle, RouteEdge, Route, RouteNode, SpeedLimit, VoxelGrid`）。带时间的轨迹消息在 ROS 2 里是 `trajectory_msgs/JointTrajectory`（关节）与 `trajectory_msgs/MultiDOFJointTrajectory`（无人机），**nav2 一个都不用**。
2. **动作层：`FollowPath` 只收 `nav_msgs/Path`。** 见 §1.3。
3. **接口层：Controller 只吐一拍控制量。** `computeVelocityCommands(...) → geometry_msgs::msg::TwistStamped`；`Controller` 接口里**没有任何**返回轨迹的方法（本机 `controller.hpp` 全文）。Smoother 是 `Path → Path`（`smooth(nav_msgs::msg::Path &, Duration) → bool`）；Planner 是 `pose,pose → Path`。三个接口全部以 `Path` 为枢纽，而 `Path` 无时标。
4. **nav2 自己会主动丢弃 Path 上的时间戳。** MPPI 的 path handler 把全局路径里**每个 pose 的 `header.stamp` 覆盖成当前位姿的时间戳**：
   ```cpp
   transformed_plan.header.stamp = global_pose.header.stamp;      // path_handler.cpp:71
   global_plan_pose->header.stamp = global_pose.header.stamp;     // path_handler.cpp:86
   ```
   【源码】`nav2_mppi_controller/src/path_handler.cpp`（Humble 分支）。⇒ **把时标塞进 `poses[i].header.stamp` 这条路，在 nav2 自带控制器上会被无声抹掉**。
5. **唯一"带时间"的东西是 MPPI 的内部实现细节**：MPPI 在自己的滚动时域里维护一条 `time_steps × model_dt` 的候选轨迹并做优化，但对外**只输出第一拍** `TwistStamped`（`nav2_mppi_controller`，`computeVelocityCommands`）。这条内部轨迹不是可插拔接口、也不给别人消费。
6. **对照：别家怎么带时间。** 无人机栈干脆**不用 `Path`**：EGO-Planner 的 `traj_server.cpp` 发布自定义消息
   ```cpp
   pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
   ```
   而 `quadrotor_msgs/PositionCommand.msg` 带 `position / velocity / acceleration / yaw / yaw_dot / trajectory_id / trajectory_flag`——**时间参数化直接进消息定义**【第三方】<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp`、`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）。RM 侧的 HWSentryNav26 则把"轨迹 + 速度剖面"做成自己的结构体（见 §5.2）。

### 2.1 那 min-snap 后端该放哪？（三种放法，代价不同）

| 放法 | 怎么做 | 丢什么 | 什么时候选 |
|---|---|---|---|
| **放进 Planner**（A\* + min-snap 一步到位，输出按 `dt` 采样的 `Path`） | `createPlan()` 里跑 A\* → min-snap → 按库的 `dt` 采样成 `geometry_msgs/PoseStamped` 序列 | **时标**（`Path` 装不下）；下游控制器只能用几何跟踪（RPP/MPPI 都只看几何） | **首选**。语义最干净：min-snap 本质是"把一条几何路径整形+限速"，输出仍是路径 |
| **做成 Smoother**（navfn/smac 出 path → min-snap 优化） | 实现 `nav2_core::Smoother`，`smooth(Path&, Duration)` 原地改写 | 同样丢时标；**另外**：Smoother 的 costmap 走 `CostmapSubscriber`（拷贝 + §6.4 的并发 bug 面），且 `SmoothPath` 只在 BT 里被显式 tick（不是每个规划周期自动跑） | 想"复用 nav2 前端 + 只换后端"、并且能接受"平滑是 BT 里显式一步"时 |
| **做成 Controller**（`setPlan()` 时把 `Path` 变成带时标的内部轨迹，每拍按时间采样出 `Twist`） | 实现 `nav2_core::Controller`：`setPlan()` 里跑 min-snap，`computeVelocityCommands()` 里 `t = now - t0` 查表 | 要自己写：路径跟踪、到达判据（交给 `GoalChecker`）、速度限幅、失败恢复（抛 `nav2_core::PlannerException` 会被 server 兜住） | **只有当真要按 min-snap 的时标执行时**才选它。此时 `Path` 只是"任务的几何输入"，时标由自己维护 |

**我们的库属于哪一种？** 上游的输出形态很关键（本次从源码核实）：

- `MinimumSnap::SolveOutput` = `{ std::vector<Eigen::Vector2f> path; std::vector<PointPair> corridor; int iter; double time; bool success; }`——**返回的是"路径点 + 走廊"**，多项式系数与段时长是内部量（`evaluateEquation`/`lineDecoder` 都是 `protected`）。【源码】`src/MinimumSnapOsqp/minimumSnap.hpp:179-186, 256-259`。
- 但这条 `path` **是按时间等间隔采样出来的**：内部 `while(tres > dt){ tres -= dt; … op.push_back(getfx(i, time)); }`，末尾还按 `tres > dt*0.1` 补一个点（`minimumSnap.cpp:541-551`）；`dt` 默认 0.1 s（`minimumSnap.hpp` 的 `float dt = 0.1`），上游 README 的运行参数表也写着"采样时间 0.1s：MinimumSnap结果函数按照分配时间、速度进行采样"【源码】+【第三方】README。
  ⇒ **时标是"隐式"的（相邻点 ≈ `dt`），一旦落进 `nav_msgs::Path` 就只剩"约定"，不是数据。**
- 段时长由 `trapezoidalTimeAllocation(path, maxSpeed, maxAcc)` + `setTimeAllocated()` 决定，`maxSpeed/maxAcc` **只是自动时间分配用的参数，不是求解期的硬约束**（`Usage.md` 原话："该参数只影响未显式设置 timeAllocated 时的时间分配，不是求解阶段的硬速度约束"）【第三方】`doc/Usage.md`。
- `KinodynamicAstar::Result` 反而**显式带时间**：`struct Sample{ State state; Eigen::Vector2d acceleration; double time; }`（`State = [x,y,vx,vy]`），`Config::sampleTime = 0.1`【源码】`src/YAstar/kinodynamicAstar.hpp`。

**因此对本文这个库**：先做 Planner（把 min-snap 的 `path` 输出成 `Path`，把 `dt` 变成**显式参数**并在文档里写成契约）；等真需要按时标跑（例如要卡"过洞/上台阶"的时间窗）再做 Controller 或独立节点。

---

## 3. 效率：插件 vs 直调，逐项拆开算

> 口径：本节把开销分成 **one-off（一次性）** 与 **per-call（每次规划/每拍）**，并区分 **有来源的数字** 与 **【无来源】必须自己测的数字**。

### 3.1 one-off（生命周期/加载期）

| 项 | 量级/来源 | 说明 |
|---|---|---|
| `dlopen` + 类构造 + `configure()` | **机制有据，数字【无来源】** | `class_loader`/`pluginlib` 用宿主 loader 打开 `.so`【文档】<https://github.com/ros/class_loader>。`planner_server` 只在 `on_configure()` 里做一次（见 §1.2）。**没有找到任何公开的"pluginlib 加载耗时"实测数字**——搜索到的只有机制说明与教程，没有任何 ms/µs 级测量（§9）。 |
| TF buffer / costmap 共享指针 | 由 server 提供，不额外付 | 你的插件只是拿引用（§1.4）。 |
| costmap 首次分配 560×300 | 168 000 B（= 560×300×1 B）【推算，算术】 | 与我们仿真里 map 尺寸一致（上游基准就是 560×300 @0.05 m）。 |

**结论**：一次性开销在整个生命周期里只出现一次（除非服务器崩溃 respawn），**与"每次规划快不快"无关**。它唯一可能咬人的场景是"每次规划都重新 `createInstance`"——而 nav2 的 server 不这么做。

### 3.2 per-call（每次规划/每拍）

| 项 | 有来源的数字 | 判断 |
|---|---|---|
| 一次虚函数调用 | 【无来源】 | 量级上是几 ns 级（间接跳转），**与库自身 5.4 ms/126.7 ms 相比可忽略**——这是量级推理，不是引用。 |
| action server 收发（`ComputePathToPose`） | 【无来源，只有工具链】 | 走同一进程/跨进程取决于 `use_composition`（§3.4）。要 settle：`ros2_tracing`（见 §3.5）。 |
| **我们自己库的耗时（决定一切的那一项）** | **有来源**（见 §3.3） | A\* 平均 5.4 ms / 最大 14.8 ms；min-snap 平均 3.9 ms / **最大 126.7 ms**。 |
| `planner_server::publishPlan()` 的路径拷贝 | 【源码】每次 `auto msg = std::make_unique<nav_msgs::msg::Path>(path);`——**整条路径深拷贝一次**，且只在有订阅者时发布 | 300 点 ≈ 21–24 KB 拷贝【推算，按消息字段算：每 `PoseStamped` ≈ 72–80 B CDR】。相对 5 ms 级规划可忽略。 |
| Controller 侧每拍拷贝 | 【源码】`publishVelocity` 把 `velocity.twist` 拷成 `Twist` 再发；`nav2_velocity_smoother` 每拍再处理一次 | 每拍几十字节，可忽略；**但 velocity_smoother 是一个额外节点/额外一次 DDS 往返**，属于"要不要进链路"的取舍。 |

### 3.3 这个库自己公布的性能（必须进决策表）

上游 README「性能表现」表（1000 次，AMD 7735H @4.2 GHz / Ubuntu 22.04 / GCC 11.2.0，场地 560×300 @0.05 m）【第三方】<https://github.com/T-DT-Algorithm-2026/tdt-nav-kit>（`README.md`）：

| 地图 | 算法 | 平均 | 最大 |
|---|---|---|---|
| RMUC2024 | Astar | 7627.44 µs | 21806 µs |
| RMUC2024 | Minimum Snap | 1255.42 µs | 14493.1 µs |
| RMUC2024 | 生成代价地图 | 1604.13 µs | 6870.61 µs |
| RMUC2024 | 化简路径 | 34.0191 µs | 242.632 µs |
| **RMUC2026** | **Astar** | **5446.45 µs** | **14761.2 µs** |
| **RMUC2026** | **Minimum Snap** | **3901.7 µs** | **126698 µs** |
| **RMUC2026** | **生成代价地图** | **1588.52 µs** | **3613.08 µs** |
| **RMUC2026** | **化简路径** | **26.0119 µs** | **117.99 µs** |

上游 `doc/MinimumSnap.md` 另有一张"1000 次、全图随机取点"的表，其中 `RMUC2025 / OSQPCorridor / astar+势场` 平均 6052.95 µs、**最大 204404 µs**；`Close`（闭式）后端平均 694.975 µs / 最大 6757.24 µs【第三方】`doc/MinimumSnap.md`。

**这张表的含义**：
- **后端不是"便宜的那个"**：OSQP 走廊后端的最坏值（126.7 ms / 204 ms）比 A\* 前端最坏值还大一个数量级；闭式后端快且稳（最大 6.8 ms）但作者自己说"闭式求解不建议使用 jps 作为前端，因为质量很差基本不可优化"。
- 在 20 Hz（50 ms）控制环里，**126.7 ms 会顶掉 2–3 拍**；在 1–2 Hz（`expected_planner_frequency`）的全局规划器里则完全可接受。⇒ **这决定了 min-snap 更适合放在 Planner/Smoother（低频、可等待），而不是 Controller（高频、不可阻塞）**。
- 上游没有给出分位数（P50/P95/P99），只有 avg/max；而"最大 126.7 ms"这种长尾才是实时性真正的风险 ⇒ **必须自己补测分位数**（§3.6）。

### 3.4 costmap：原地读 vs 拷贝，以及 nav2 自己测过的 RMW/IPC 账

**（a）原地读（planner/controller 形态）**
`getCostmap()` 是裸指针（§1.4），`getCharMap()` 直接给 `unsigned char*`。「零拷贝」在这里是真的：**不经过 DDS、不序列化、不 new**。

**（b）消息拷贝（smoother 形态 / 独立节点订阅）**
smoother 拿的是 `CostmapSubscriber`，订阅 `nav2_msgs/msg::Costmap`，`getCostmap()` 内部 `toCostmap2D()`（含 `resizeMap`）。数据量：`uint8[] data` = 560×300 = **168 000 B**（+ `CostmapMetaData`），**每次更新都要序列化/反序列化一遍**。

**（c）ROS 2 intra-process 的官方口径（很关键，反直觉）**
ROS 2 的 intra-process 设计文档（`design.ros2.org`，Crystal 时代写就、至今仍是权威描述）明确说：

- "from the latency and CPU utilization point of view, **it is convenient to use intra-process communication only when the message size is at least 5KB**"【文档】<https://design.ros2.org/articles/intraprocess_communications.html>；
- "the performance of a single process ROS 2 application with intra-process communication enabled are **still worst than what you could expect from a non-ROS application sharing memory between its components**"——**同进程的 ROS 2 也打不过"非 ROS 的共享内存"**；
- intra-process 仍需通过 RMW 发"meta-message"，因此"performance … heavily dependent on the chosen RMW implementation"。

**（d）nav2 官方实测的 CPU 账（TurtleBot4 仿真，rolling commit，2025-12）**【文档】<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>（"Performance in ROS 2: RMW, Node Composition, Intra-process Communication, and QoS"）：

| Middleware | 配置 | CPU |
|---|---|---|
| Zenoh | 基线 | 4.5% |
| Zenoh | IPC (SharedPtr) | 6.0% |
| Zenoh | IPC + ConstSharedPtr | 5.8% |
| CycloneDDS | 基线 | 18.3% |
| CycloneDDS | IPC (SharedPtr) | 17.8% |
| CycloneDDS | IPC + ConstSharedPtr | 16.1% |
| FastDDS | 基线 | 6.8% |
| FastDDS | IPC (SharedPtr) | 7.8% |
| FastDDS | IPC + ConstSharedPtr | 7.8% |

nav2 maintainer 的结论是：**`use_composition: true` 强烈推荐**（"reduces process overhead, and generally improves system efficiency"），而 **intra-process 要看 RMW**：FastDDS/Zenoh 开 IPC 反而更费 CPU，CycloneDDS 才降。`use_intra_process_comms` 默认 `false`。

⇒ 对我们的直接含义：**"把 costmap 从 nav2 交给规划器"最省的形态是"同一个进程 + 指针原地读"（也就是 planner/controller 插件 + composition），而不是"跨节点发消息再开 IPC"**。开 IPC 不是免费的。

### 3.5 rclcpp 发布/订阅延迟与执行器抖动（有来源的部分）

| 结论 | 来源 |
|---|---|
| rclcpp 相对裸 DDS 的通信开销**最多 50%** | ros2_tracing 论文转引：*"Kronauer et al. investigated the end-to-end latency of communications and found that its overhead is up to 50% compared to directly using DDS"*【第三方/论文】<https://arxiv.org/abs/2201.00393>（该论文本身是 nav2 官方 profiling 教程推荐的工具链） |
| 打开全部 ROS 2 instrumentation 后，端到端消息延迟开销**平均 0.0033 ms（3.3 µs）** | 同上（ros2_tracing 摘要） |
| rclcpp 的三种 executor "work well for most applications, but there are some issues that make them **not suitable for real-time applications**, which require well-defined execution times, determinism, and custom control over the execution order" | ROS 2 官方文档【文档】<https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html>（正文源：`ros2/ros2_documentation@humble` `source/Concepts/Intermediate/About-Executors.rst`） |
| 回调组语义：`MutuallyExclusive`（同组串行）/`Reentrant`（可并行）；不同组之间**总是**可并行；配错会死锁 | 【文档】<https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html> |
| nav2 自己的缓解手段：`nav2_velocity_smoother` 的 `use_realtime_priority` ——"Adds soft real-time prioritization to the controller server … set the controller's execution thread to a higher priority than the rest of the system (`90`) to meet scheduling deadlines to have **less missed loop rates**" | 【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/> |
| iceoryx（真零拷贝共享内存）宣称"**latency of less than 1 µs** for transferring a message … this latency is constant as size doesn't matter" | 【厂商宣称】<https://iceoryx.io/classic/v2.0.0/getting-started/what-is-iceoryx/> |
| Fast DDS 有 SHM transport（同 host，每 participant 一个 shared memory segment） | 【文档】<https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html> |
| `rclcpp::LoanedMessage` 存在（borrow/return loaned message），但能否真零拷贝取决于 RMW 与消息类型 | 【源码】<https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/loaned_message.hpp> |

**没有找到的**：ROS 2 Humble + rmw_fastrtps 在**环回**上的端到端 pub/sub 延迟的权威实测数字（µs 级）；把"规划器放进独立进程 + 共享内存"与"同进程直调"对比过的**规划耗时**实测。【无来源】——见 §3.6 的测量方案。

### 3.6 要 settle 它，必须在**我们机器上**测这几项（列表即实验设计）

1. **pluginlib 一次性加载**：给 `planner_server` 加 `--log-level debug` 或用 `ros2_tracing`/`perf` 量 `dlopen` 到 `configure()` 返回的墙钟；对照"独立节点启动到 ready"。预期：**一次性、只影响起动时间**。（无来源 ⇒ 必须自测）
2. **虚函数 vs 直调**：同一份库代码，(a) 在插件 `createPlan()` 里调，(b) 在纯 C++ benchmark 里调（上游 `main.cpp` 的 `benchTime` 就是现成模板），跑 1000 次取 **P50/P95/P99/max**。判据：差值若 < 库自身 P50 的 1%，则"插件税"在本问题下不可见。
3. **costmap 取用方式**：(a) `getCostmap()->getCharMap()` 原地 + 值域转换，(b) 订阅 `global_costmap/costmap_raw` 反序列化。量两侧的**每次同步耗时**与**最坏值**（168 KB 量级，见 §3.4）。
4. **`Path` 构建+发布**：先用 `ros2 topic hz/delay` 与 `ros2_tracing` 量 300 点 `Path` 的发布端到端延迟；同时用纯 C++ 侧计时把"构造 Path"与"发送"分开。
5. **执行器抖动**：`planner_server`/`controller_server` 在 (a) 默认单线程 executor、(b) composition + 多线程 executor + 不同回调组 下的**规划回调间隔抖动**（P99 − P50）。判据：把 min-snap 的最坏 126.7 ms 放进时间线，看它是否让 `controller_frequency` 掉拍（server 自己会打 "Control loop missed its desired rate"）。
6. **`setPlan` 频率与重算节奏**：确认我们的 BT 多久 re-tick 一次 `ComputePathToPose`（`PipelineSequence` 语义），以及 `planner_server.planning_time` 与"库内部耗时"的差（§1.2 已说明它包含等待/校验）。
7. **分位数**：上游只给 avg/max，**必须自己补 P95/P99**（尤其 OSQP 走廊后端）。
8. **工具链**：`ros2_tracing`（`ros2 trace` + LTTng + `babeltrace`/`tracetools_analysis`）是 nav2 官方 profiling 教程指定的工具，且它自己的开销平均 3.3 µs【文档+论文】<https://docs.nav2.org/rolling/tutorials/general_tutorials/get_profile/get_profile/>、<https://arxiv.org/abs/2201.00393>。

---

## 4. 问题四：真实世界的三种接法（含出处与作者自述的取舍）

### 4.1 模式 A：包成 nav2 插件（**先把"外部纯 C++ 库"包成插件的先例**）

| 例子 | 包的是什么外部库 | 可核实的证据 |
|---|---|---|
| `nav2_smac_planner` | **OMPL**（纯 C++、无 ROS 的采样式规划库） | `CMakeLists.txt: find_package(ompl REQUIRED)`；`package.xml: <depend>ompl</depend>`；`include/nav2_smac_planner/node_hybrid.hpp:28 #include "ompl/base/StateSpace.h"`，`:125 ompl::base::StateSpacePtr state_space;`【源码】ros-navigation/navigation2 `humble` 分支 |
| `nav2_constrained_smoother` | **Ceres Solver**（纯 C++ 优化库） | `package.xml`: `<description>Ceres constrained smoother</description>`、`<depend>libceres-dev</depend>`（第 6、20 行）；`CMakeLists.txt:4` 甚至写着 `set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup`——**nav2 自己承认外部求解器的编译类型对性能有显著影响**【源码】ros-navigation/navigation2 `humble` 分支 |
| `spatio_temporal_voxel_layer`（STVL） | **OpenVDB**（纯 C++ 体素库），作为 costmap 图层插件 | 已被 nav2 官方插件目录收录：*"Spatio-Temporal Voxel Layer — Steve Macenski — Maintains temporal 3D sparse volumetric voxel grid with decay through sensor models"*【文档】<https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/>。我们自己的 global costmap 就在用它【我们的实测/文档】`docs/architecture.md:137` |
| `pb_nav2_plugins`（SMBU PolarBear 战队） | **他们自己的 C++ 行为/图层**，独立仓库、第三方维护 | README：*"`pb_nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库"*；`costmap_plugins.xml` 注册 `pb_nav2_costmap_2d::IntensityVoxelLayer`（`base_class_type="nav2_costmap_2d::Layer"`）；`CMakeLists.txt: pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)`【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins> |
| `pb_omni_pid_pursuit_controller`（同战队） | **他们自己的全向 PID 跟踪律**，做成 `nav2_core::Controller` 插件 | README 列出全部参数（`translation_kp/ki/kd`、`lookahead_dist`、`curvature_min/max`…）；在 `pb2025_sentry_nav` 的 `nav2_params.yaml` 里被写成 `plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"`【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller> |

**这些作者的取舍（可引用的部分）**：
- nav2 官方 tuning guide 明确推荐 `use_composition: true` 以省 CPU/内存，并且**插件是官方推荐的扩展方式**（插件目录本身就是它的产品面）【文档】<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>。
- PolarBear 的做法是"**nav2 骨架 + 自研插件**"：`pb2025_sentry_nav` 的 `pb2025_nav_bringup/package.xml` 同时依赖 `navigation2`、`pb_nav2_plugins`、`pb_omni_pid_pursuit_controller`、`terrain_analysis`、`terrain_analysis_ext`——**自研的"地形分析"节点与 nav2 并存**，而"控制器/图层"则以插件形式进入 nav2【第三方】<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>。注意他们的 `robot_base_frame: gimbal_yaw` + `fake_robot_base_frame: gimbal_yaw_fake`（我们栈的 `base_link_fake` 是同一套思路）【第三方】`pb2025_nav_bringup/config/simulation/nav2_params.yaml:98,153,154,272,389,439`。

### 4.2 模式 B：独立节点（自己吃地图、自己发底盘指令）

**（i）HWSentryNav26 —— 浙江大学 Hello World 战队 26 赛季轮腿哨兵导航**（C++，135 ⭐）【第三方】<https://github.com/Polyacetone/HWSentryNav26>

这条最有参考价值，因为它**就是"纯 C++ 规划栈 + ROS 2 外壳、不用 nav2"的现成答案**：

- **不用 nav2**：README 原话 *"导航状态机采用扁平化有限状态结构…状态机为本项目从零设计，**未采用 Navigation2 行为树等现成方案**"*；`nav_executor/package.xml` 的依赖是 `rclcpp, rclcpp_components, tf2, tf2_ros, cv_bridge, tf2_geometry_msgs, visualization_msgs, sensor_msgs, nav_msgs, std_msgs, interfaces, common_libs`——**没有任何 nav2 依赖**（`map_server` 同理）。
- **作者对"自研 vs 框架"的判断（可直接引用）**：*"判断标准不是'别人都用'，而是'**现成框架的抽象是否和你的问题对齐**'…如果你的机器人是普通四轮、平地行驶，用它比较省心。但一旦你的需求超出它的抽象——比如地形不是'空地/障碍'二值、底盘动力学不是普通舵轮/全向轮——你就要在它的框架里打补丁，**补丁越多，越不如自己搭**。"*（`TUTORIAL.md`）
- **他们的轨迹表示（与我们的 min-snap 问题同构）**：`AnnotatedPath` = 一次规划的**不可变路径包**：`MincoTrajectory` + `PathSpeedProfile` + 台阶段 + 代价图层 + `goal_id`（用于丢弃过期结果）+ `planning_performance`；`PathSpeedProfile` 的原子状态是 `{arc_length, time, velocity}`，并注明 *"区间内 v² 对弧长线性，因此切向加速度恒定"*【第三方】`nav_executor/include/nav_executor/common/trajectory/annotated_path.hpp`、`path_speed_profile.hpp`。
- **几何与时标解耦（关键设计判断）**：*"关键不变量：**几何与时标解耦**——MINCO 的参数时长只是几何优化的内部坐标，不进入执行。原因：执行时标完全由速度剖面决定（见阶段 6）"*；并且统一用**弧长**做坐标，使"规划线程不可变地生成、执行线程无歧义地消费"【第三方】`DESIGN.md`。
- **控制频率的取法**：*"`nav_executor` 不设独立控制定时器，而是以底盘反馈（`ChassisStatus`，20 Hz）的每次到达作为一拍的触发源。这样控制周期与底盘反馈天然同步，**不存在两个时钟的相位混叠**"*【第三方】`DESIGN.md`。
- **代价图的处理（与"读不读 nav2 膨胀层"直接相关）**：`map_server` 离线做**物理距离膨胀**，理由是 *"下游的 MINCO（L-BFGS）与 MPC（FDDP）都是基于梯度的优化器，如果代价只在障碍格内跳变到 255、格外为 0，采样得到的代价场在边界附近梯度急剧变化甚至不连续，线搜索容易失败"*【第三方】`DESIGN.md`。⇒ **梯度型后端要的是"连续可导的距离场"，不是 nav2 那套 0/253/254/255 的阶梯代价**。这条直接适用于我们的 min-snap（OSQP 走廊后端虽然不需要梯度，但走廊生成同样依赖"哪些格是障碍"的干净语义）。

**（ii）无人机栈：Fast-Planner / EGO-Planner 家族**：规划 + `traj_server` 是**独立 ROS 节点**，通过**自定义消息**（`quadrotor_msgs/PositionCommand`：position/velocity/acceleration/yaw/yaw_dot + trajectory_id/flag）把**带时标的轨迹**交给控制器，话题 `/position_cmd`【第三方】<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp:240`；`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）。这套接法**不能直接搬到 nav2**——因为 nav2 的 Controller 接口只吃 `TwistStamped`、不吃位置/速度前馈（§2）。

### 4.3 模式 C：独立进程 + 共享内存 / 零拷贝 IPC

- **iceoryx**：宣称 <1 µs 常数延迟、真零拷贝（厂商口径，§3.5）；ROS 2 侧有对应 RMW 集成，但 **本次没有找到任何"把规划器放进独立进程、用 SHM 喂 costmap"的真实机器人项目**（§9）——公开的都是中间件性能测试（如 IEEE "Latency Analysis in Internal Layers of Shared Memory Communication in ROS2"）而不是规划器案例。
- **Fast DDS SHM**：DDS 内建、同 host 自动生效（segment/segment buffer 机制）【文档】<https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>。注意：**它仍然要经过序列化与 DDS 语义**，与 §3.4(c) 的 intra-process 结论合起来看：**"跨进程零拷贝"在这条链路上不是免费的**。
- **nav2 自己的经验数据**（§3.4(d)）说明：即使全在同进程（composition），开 IPC 也会让 FastDDS/Zenoh 的 CPU 上升。⇒ **不建议为"省规划器通信开销"走到模式 C**；要省，先走"同进程 + 指针原地读"。

### 4.4 三种模式的公开证据强度（诚实评估）

| 模式 | 证据强度 |
|---|---|
| A 插件 | **强**：nav2 官方插件目录 + 多个外部库（OMPL/Ceres/OpenVDB）先例 + 第三方战队的插件仓库 |
| B 独立节点 | **强**：HWSentryNav26 全仓库可查；PolarBear 的 `terrain_analysis*` 与 nav2 并存；无人机栈 |
| C 独立进程+SHM | **弱**：只有中间件层面的性能文档/论文，**没有找到规划器案例** |

---

## 5. 问题五：落地坑（逐条挂到我们自己的实测/文档）

### 5.1 `/cmd_vel` 是单发布者契约，接之前先决定"谁拥有它"

- nav2 内部：`controller_server` 是唯一发布 `cmd_vel` 的 server（`vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1)`）【源码】`controller_server.cpp:198`；`nav2_behaviors`（Spin/BackUp…）自身也会发速度指令，所以"谁能发速度"本来就是 nav2 的配置问题。
- **我们栈的实际链路**（【我们的实测/文档】`src/rm_navigation/rm_navigation/launch/navigation_launch.py`）：
  `controller_server`（`cmd_vel → cmd_vel_nav`，:237）→ `nav2_velocity_smoother`（`cmd_vel → cmd_vel_nav`、`cmd_vel_smoothed → cmd_vel`，:298）→ `/cmd_vel` → `fake_vel_transform`（订阅 `/cmd_vel`，发布 `/cmd_vel_chassis`，并以 20 Hz 发布 `base_link→base_link_fake`）【我们的实测/文档】`src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:11-14`。
- **⇒ 若走独立节点形态，必须显式二选一**：要么不启 `controller_server`（并把 `/cmd_vel` 让给它），要么让它只发 `/plan`、由 nav2 控制器跟踪。**两条链路同时发 `/cmd_vel` = 不定态**。
- 另外 `fake_vel_transform` 的 `spin_speed≠0` 会让 nav 看到的 `base_link_fake` **凭空旋转**而雷达不转（我们已记录：`docs/robot_models.md:321` 建议 `spin_speed:=0.0`；`spin_speed==0` 时 `base_link_fake ≡ base_link`）——**独立节点若直接用 `odom` 系做规划/控制，会绕过这套坐标系，行为与 nav2 侧不可比**。

### 5.2 生命周期顺序与"configure 期只做一次"的约束

- `nav2_lifecycle_manager`："take in a set of **ordered** nodes to transition one-by-one into the configuration and activate states… then bring down the stack in the **opposite order**"，并建立 bond 心跳，任一 server 失联就把整栈拉下去【文档】<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/>。⇒ **新插件的宿主 server 必须在 `node_names` 里，且顺序要保证 costmap 先起来**。
- 插件拿到 `Costmap2DROS` 共享指针后，**不要在 `configure()` 里假设 costmap 已经 available**：`planner_server::on_configure()` 先 `costmap_ros_->configure()` 再 `planner->configure(...)`；**`activate()` 阶段才 `costmap_ros_->activate()`**（`planner_server.cpp` 约 84–85、176 行）。真正的"地图有效"要等 `costmap_ros_->isCurrent()`——server 自己会 `waitForCostmap()`。
- **`get_plugin_type_param()` 读不到 `<id>.plugin` 会 `exit(-1)`**（§1.2）——参数写错不是"回落默认"，而是**整个 server 进程退出**。

### 5.3 costmap 的 frame 语义：高度/几何参数是**在 costmap 的 global frame 里量的**（本次从源码逐字核实）

`nav2_costmap_2d` 的 `ObservationBuffer` 把所有观测**先变换到 costmap 的 global frame**，然后才用 z 做高度带过滤：

```cpp
tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);   // observation_buffer.cpp:116
...
if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) { ... } // :141-145
```

`global_frame_` 来自 `layered_costmap_->getGlobalFrameID()`【源码】`obstacle_layer.cpp:122` + `observation_buffer.cpp`（Humble 分支）。

⇒ **含义（对本文这个库直接相关）**：
- 我们的 **global costmap 的 global frame 是 `map`，local costmap 是 `odom`**；nav2 侧 `robot_base_frame` 在 sim 槽位里是 **`base_link_fake`**（【我们的实测/文档】`src/rm_navigation/rm_navigation/params/nav2_params.yaml:172-173,239-240`（`global_frame: odom` / `map`）、`nav2_params_sim_base.yaml:86,129,285,414`（三处 costmap + behavior 均为 `robot_base_frame: base_link_fake`，注释写明"与 local/global costmap、controller 保持一致（哨兵云台系）"）；同族外部证据 `docs/cod_nav_macro_integration.md:187-188`）。⇒ **高度带、`robot_radius`/footprint、origin 全部是"该图自己那个 frame 下的量"**；把 global 的高度带参数抄到 local（或反之）本身就换了参考系。
- 我们已有同族教训：`docs/architecture.md:303` 把 `min/max_obstacle_height` 定性为"机器人碰撞体的高度范围"，而 local 的 `/scan` 是**相对车顶 `livox_frame` 的 z∈[−1.0, +0.1] 切片**；`docs/tilted_lidar_fidelity.md` 整篇记录的是"点云不在它自称的那个帧里 / odom 帧被转 30°"这个坑（§H1、§I.2）。**把库接进来时，"地图原点/分辨率/朝向"必须从 `costmap_ros` 读，不能从参数里另抄一份。**
- **对上游库的具体换算**（本次从源码核实）：`YAstar::transformPos(pos) = (pos - originPos) / mapping`（`yastar.cpp:883-886`）⇒ 库的 `originPos` **就是"格 (0,0) 在世界系里的坐标"**，和 nav2 的 `getOriginX()/getOriginY()` 语义**一致**；数组是 RowMajor `(height, width)`、按 `[y][x]` 索引（`isInObstacle(x,y)` 读 `occMap(y,x)`，`yastar.cpp:936-939`），和 nav2 `char map` 的 `map[y*size_x + x]` **几何上兼容**。
- ⚠️ **但值域不兼容（这是最容易踩的一脚）**：`YAstar::setMap(int w, int h, u_char* mapData)` 会把数据映射成 `x == 0 ? 1.f : 0.f`，即 **0 = 障碍、非 0 = 自由**（`yastar.cpp:404-417`），阈值 `occThs = 0.5f`（`yastar.hpp:309`）；而 nav2 的 master grid **0 = `FREE_SPACE`、254 = `LETHAL_OBSTACLE`、255 = `NO_INFORMATION`**（§1.4）。**直接把 `getCharMap()` 喂进去 = 整张图取反**（自由变障碍）。另一个重载 `setMap(..., std::vector<int8_t>& data)` 用 `x == 0 ? 0 : 1`（`yastar.cpp:418-430`），它符合"0 空闲"的 occupancy 直觉，**但会把 `-1`（unknown）当成障碍**。⇒ **必须自己写显式转换函数 + 一个单元测试**（§7.3 骨架里给了 `0/1` 映射）。
- `SfcSquare::Map = Eigen::Matrix<unsigned char, Dynamic, Dynamic, RowMajor>`，其 `setMap(int w,int h,u_char* data, mapping, originPos)` 也在**同一个 `u_char*` 约定**下工作（`_getBound`/`_getBoundSquareInplace` 里 `map(yint,xint) == 0` 被当作"该点不可扩展"，`sfcSquare.cpp:148,162-203`）——**也就是说 A\* 与 SFC 两侧的 0/1 约定必须一致地喂**，否则走廊会反着长。
- 还要注意**双重膨胀**：nav2 的 `inflation_layer` 已经把代价写进 master grid（253/254），而库自己还会 `initCostMap()`/`setCostField(radius, decay)` 再造一层势场（【源码】`yastar.hpp:175-185`、`README` 势场函数 `f(x)=0.5/(0.1+x)+1.0`）。**要么把 nav2 的膨胀算作"输入"，要么让库自己膨胀**——两次叠加会得到"墙比真实更厚"的行为，且与我们 `docs/` 里"`inflation 若调到 0.55/0.75，必须 ≥ robot_radius`"那条注意同源。

### 5.4 线程安全：读 costmap 与"图层在更新"是**真的并发**

- costmap 有**自己的更新线程**：`Costmap2DROS::on_configure()` 里 `map_update_thread_ = std::make_unique<std::thread>(...)`（【源码】`costmap_2d_ros.cpp:295`），循环在 `mapUpdateLoop()`（:456-481）里按 `update_frequency` 调 `updateMap()`；而 `planner_server`/`controller_server` 又把 costmap 放进 `nav2_util::NodeThread` 单独 spin（§1.4）。⇒ **规划回调与 costmap 更新天然是两个线程。**
- 正确做法：读主栅格时拿 `Costmap2D::getMutex()`（`std::recursive_mutex`）——nav2 自己就这么干（`isPathValid()`）、装图层时也这么干（"no update is allowed until the plugin is initialized"）。**"我先拷一份再算"是简单可靠的替代**（拷 168 KB 的时间远小于一次 A\*）。
- **上游已有的并发 bug（值得读，别重犯）**：nav2 issue **#6429 "Concurrent Access Bugs"**（Jazzy 分支 + ASan 复现）列出 4 个 race，其中 *Bug1* 就是 **`CostmapSubscriber` 只维护一份共享 `Costmap2D`：spin 线程用 `getCost()` 读，而 executor 线程处理新 costmap 时若尺寸变了就 `resizeMap()` 释放并替换同一个数组 ⇒ `heap-use-after-free`**；*Bug3* 是 `planner_server` 里"costmap 几何 + `planner_->potarr`"的 use-after-free【第三方/GitHub】<https://github.com/ros-navigation/navigation2/issues/6429>。
  - ⚠️ **Humble 是否同样中招：本次未能确证**。但 Humble 源码里 `CostmapSubscriber::getCostmap()`/`toCostmap2D()`/`resizeMap()` 的结构与 Jazzy 相同（`nav2_costmap_2d/src/costmap_subscriber.cpp` Humble 分支），**所以"通过 `CostmapSubscriber` 拿图（也就是 Smoother 形态）天然带着这个风险面"** ⇒ 若采用 Smoother 形态，**必须自己再加锁/加拷贝**，不要假设 nav2 帮你兜住。
- **锁序反转**：issue **#6461**（Lyrical 分支，ThreadSanitizer）报告 `costmap_2d_ros.cpp` 的 `_dynamic_parameter_mutex`（`mapUpdateLoop()` 持锁调 `LayeredCostmap::updateMap()` 的组合锁）、插件初始化持 costmap 锁、`updateParametersCallback()` 之间形成锁序环，可能死锁【第三方/GitHub】<https://github.com/ros-navigation/navigation2/issues/6461>。**Humble 1.1.20 源码里 `_dynamic_parameter_mutex` 与 `dynamicParametersCallback` 都存在，`mapUpdateLoop()` 同样持该锁调 `updateMap()`（`costmap_2d_ros.cpp:475-479`）**；但该 TSan 报告是针对新分支的，**不能据此声称 Humble 已验证有死锁**——只能说"同类代码结构在，风险面相同"。
- **实践建议**：插件里**不要在 `configure()` 里缓存 `Costmap2D*` 的坐标几何后长期使用**（issue #6429 的 Bug3 就是几何与内部数组不匹配），每次 `createPlan()` 现读 `getSizeInCellsX/Y()`、`getResolution()`、`getOriginX/Y()`。

### 5.5 `setPlan` 的重算节奏

- `setPlan()` 只在 `FollowPath` 动作开始/被抢占时调一次（§1.2）；**"多久重算一次全局路径"由行为树决定**（`ComputePathToPose` 被再 tick），不是由 controller 决定。nav2 的 smoother 教程里也提示了这一点：在 `PipelineSequence` 里如果把平滑结果写回 `{path}`，`FollowPath` 返回 `RUNNING` 时会重 tick 上游节点把 `path` 覆盖掉，建议平滑结果写到另一个黑板变量【文档】<https://docs.nav2.org/rolling/tutorials/general_tutorials/adding_smoother/adding_smoother/>。
- ⇒ 我们若把 min-snap 放在 Planner 里，**重算节奏 = BT 的重规划节奏**；放在 Controller 里则要注意：库里最坏 126.7 ms 的求解**不能放在每拍 `computeVelocityCommands()` 里**（会直接把控制环拖垮），必须在 `setPlan()` 时算好或用后台线程 + 双缓冲。
- `planner_server.planning_time` 的口径见 §1.2（含等待/校验），做 A/B 时要额外埋点区分。

### 5.6 保持"纯 C++"形态的 CMake 结构（有现成先例）

**先例 1（库 ROS-free + ROS 包 `find_package` 它）**：`ompl` 在本机 Humble 里就是**普通 CMake 包**（`/opt/ros/humble/share/ompl/package.xml` 里 `<build_type>cmake</build_type>`），而 `nav2_smac_planner` 用 `find_package(ompl REQUIRED)` + `<depend>ompl</depend>` 直接用它（§4.1）。本机共有 **10 个** ROS 2 包声明 `build_type=cmake`：`backward_ros cartographer foonathan_memory_vendor gmock_vendor gtest_vendor ignition_cmake2_vendor ignition_math6_vendor ompl ros_workspace urdfdom`【本机可复核】。
**先例 2（上游自己的做法）**：tdt-nav-kit 用 `scripts/setup.sh` 把 OSQP/OsqpEigen 构建安装到 `3rd/install`，`CMakeLists.txt` 用 `find_package(osqp REQUIRED)`/`find_package(OsqpEigen REQUIRED)` 消费【第三方】`README.md`、`CMakeLists.txt`。**同一套"前缀安装 + find_package"的套路就能用在它自己身上。**

⇒ 推荐结构（**库一行不改**）：

```
tdt_nav_kit/                     # 上游仓库（保持纯 CMake、无 ROS；可作为 submodule 或 vendored 目录）
  CMakeLists.txt                 # add_library(tdt_nav_kit STATIC ...) + install(EXPORT) + CTest
  src/YAstar/…  src/MinimumSnapOsqp/…
  test/  bench/                  # 纯 C++ 单测 + 上游 main.cpp 风格的 benchmark（无 ROS 也能跑）

tdt_nav_kit_ros/                 # 我们新增的 ament 包（唯一带 ROS 的地方）
  package.xml                    # <depend>tdt_nav_kit</depend> + <nav2_core plugin="..."/> 导出
  CMakeLists.txt                 # find_package(tdt_nav_kit REQUIRED) + 一个 SHARED 插件库
  global_planner_plugin.xml      # pluginlib 描述（base_class_type="nav2_core::GlobalPlanner"）
  include/tdt_nav_kit_ros/tdt_global_planner.hpp
  src/tdt_global_planner.cpp     # 薄适配：costmap→库、库→nav_msgs::Path
  src/standalone_node.cpp        # 可选：同一份适配代码再包一个独立节点（模式 B 的对照组）
```

### 5.7 三个"接口语义"级别的坑（本库特有）

1. **值域取反**（§5.3）：`getCharMap()` 必须显式映射，不能直传。
2. **`dt` 是隐式时标**（§2）：库的输出点间距 ≈ `dt = 0.1 s`（默认），一进 `Path` 就没有这个信息。要么把 `dt` 变成参数并在文档里写死契约，要么走 Controller 形态自己维护。
3. **`initCostMap()` 是硬前提**：库自己写着 *"调用 search 之前必须调用 `initCostMap()` 函数，或者其他的设置 costmap 的函数"*（`yastar.hpp:174-175`）——**忘了这一步不会报错，只会得到奇怪路径**。

---

## 6. 给我们的 bench 的推荐（含接入方式）

### 6.1 推荐形态：**"纯库 + 薄适配"两 target，先上 `GlobalPlanner` 插件**

理由（逐条对上证据）：
1. **契约最便宜**：Humble 的 `GlobalPlanner` 只有 `createPlan()` 一个函数有实际工作，其余 4 个可以留空（官方教程原话："The remaining methods are not used but it's mandatory to override them… we did override all but left them blank"）【文档】。
2. **costmap 拿的是指针**（不像 Smoother 走 `CostmapSubscriber` 拷贝 + 并发风险面，§1.4、§5.4）。
3. **库的输入形态天然匹配**：A\* 要"宽×高 + `u_char*` + 分辨率 + 原点"，正好是 `getSizeInCellsX/Y()` + `getCharMap()` + `getResolution()` + `getOriginX/Y()`（§5.3 已逐字核对语义与索引顺序）。
4. **纳秒~微秒级的插件开销 vs 毫秒~百毫秒级的库耗时**：插件形态不可能成为瓶颈（§3.2）。
5. **能直接复用我们已有的 A/B 能力**：新增 `planner:=tdt` 槽位（与 `planner:=navfn|smac2d` 并列），参数文件写成 `nav2_params_sim_planner_tdt.yaml`，用 `tools/scripts/regress/`（`nav_smoke_regression.py`、`plan_probe.py`、`nav_clearance_probe.py`、`compare_regress_snapshots.py`）做同一套判据的对照【我们的实测/文档】`tools/scripts/regress/`。

### 6.2 接入形态与代码骨架（Humble 精确签名）

```cpp
// tdt_nav_kit_ros/include/tdt_nav_kit_ros/tdt_global_planner.hpp
#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "YAstar/yastar.hpp"                  // 纯 C++，无 ROS
#include "MinimumSnapOsqp/minimumSnap.hpp"    // 纯 C++，无 ROS

class TdtGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(                 // ← Humble：只有 start/goal
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void syncCostmapToLibrary();                    // 锁 + 值域转换 + setMap

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;                              // 插件 id，用于 <id>.xxx 参数
  rclcpp::Logger logger_{rclcpp::get_logger("TdtGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  YAstar astar_;                                  // 库对象：只在这里出现
  MinimumSnap min_snap_;
  std::vector<unsigned char> grid_;               // 每次同步复用的缓冲（避免每帧 new）
  double sample_dt_{0.1};                         // 库的隐式时标，变成显式参数
  double max_speed_{3.0}, max_acc_{1.0};
  double simplify_eps_{0.1};
  unsigned int cached_w_{0}, cached_h_{0};
};
```

```cpp
// 关键：costmap → 库（Humble 语义，逐条对应 §5.3 的坑）
void TdtGlobalPlanner::syncCostmapToLibrary()
{
  auto * cm = costmap_ros_->getCostmap();          // Costmap2D*，无拷贝（costmap_2d.hpp:224）
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*(cm->getMutex()));  // 与更新线程互斥

  const unsigned int w = cm->getSizeInCellsX();
  const unsigned int h = cm->getSizeInCellsY();
  const unsigned char * master = cm->getCharMap();

  if (grid_.size() != static_cast<size_t>(w) * h) { grid_.assign(static_cast<size_t>(w) * h, 0xFF); }

  for (size_t i = 0; i < grid_.size(); ++i) {
    const unsigned char c = master[i];
    // nav2: 0=FREE_SPACE, 253=INSCRIBED, 254=LETHAL, 255=NO_INFORMATION  (cost_values.hpp:42-46)
    // 库的 u_char 重载: 0 == 障碍、非 0 == 自由  (yastar.cpp:404-417)
    const bool blocked =
      (c == nav2_costmap_2d::LETHAL_OBSTACLE) ||
      (c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ||
      (c == nav2_costmap_2d::NO_INFORMATION && treat_unknown_as_obstacle_);
    grid_[i] = blocked ? 0x00 : 0xFF;
  }

  astar_.setMapping(static_cast<float>(cm->getResolution()));
  astar_.setOriginPos(static_cast<float>(cm->getOriginX()),
                      static_cast<float>(cm->getOriginY()));   // 库 = 格(0,0) 的世界坐标（yastar.cpp:883）
  astar_.setMap(static_cast<int>(w), static_cast<int>(h), grid_.data());
  astar_.setOccThs(0.5f);          // 库内部 occMap ∈ {0,1}
  astar_.initCostMap(true);        // 库自己的势场膨胀：别和 nav2 inflation 叠两次（§5.3）
  min_snap_.setMap(astar_.getMap(), static_cast<float>(cm->getResolution()),
                   cm->getOriginX(), cm->getOriginY());  // A* 与 SFC 的 0/1 约定必须一致
}
```

```cpp
// createPlan：前端 → 化简 → 后端 → nav_msgs::Path
nav_msgs::msg::Path TdtGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();   // 不要硬编码 "map"
  path.header.stamp = clock_->now();

  syncCostmapToLibrary();
  try {
    std::vector<Eigen::Vector2f> raw = astar_.search(
      {static_cast<float>(start.pose.position.x), static_cast<float>(start.pose.position.y)},
      {static_cast<float>(goal.pose.position.x),  static_cast<float>(goal.pose.position.y)});
    if (raw.empty()) { return path; }                        // 空 Path ⇒ server 会判无效
    auto simplified = astar_.simplifyPath(raw, static_cast<float>(simplify_eps_));

    MinimumSnap::SolveInput in;
    in.setPath(simplified);
    in.setTimeAllocated(MinimumSnap::trapezoidalTimeAllocation(
      simplified, static_cast<float>(max_speed_), static_cast<float>(max_acc_)));
    in.setInitVel({0.f, 0.f});
    in.setBackend(MinimumSnap::Backend::OSQPCorridor);       // fallback = Close
    auto out = min_snap_.solve(in);
    const auto & pts = out.success ? out.path : simplified;  // 后端失败就退回前端路径

    path.poses.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i].x();
      ps.pose.position.y = pts[i].y();
      ps.pose.orientation.w = 1.0;
      // 可选：把库的隐式时标显式写出来（nav2 自带控制器会覆盖它，见 §2 第 4 条）
      // ps.header.stamp = path.header.stamp + rclcpp::Duration::from_seconds(i * sample_dt_);
      path.poses.push_back(ps);
    }
  } catch (const std::exception & e) {
    throw nav2_core::PlannerException(std::string("tdt planner failed: ") + e.what());
  }
  return path;
}
```

```cpp
// 文件末尾
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(TdtGlobalPlanner, nav2_core::GlobalPlanner)
```

参数段（Humble 键名，`planner_plugins` + `<id>.plugin`）：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    expected_planner_frequency: 1.0        # 库最坏 126.7 ms ⇒ 别设 20 Hz
    GridBased:
      plugin: "tdt_nav_kit_ros/TdtGlobalPlanner"   # Humble 用 '/'；Rolling 用 '::'
      sample_dt: 0.1
      max_speed: 3.0
      max_acc: 1.0
      simplify_eps: 0.1
      treat_unknown_as_obstacle: true
```

> **注意**：`sample_dt/max_speed/...` 是我们自己声明的参数，**必须**在用 `nav2_util::declare_parameter_if_not_declared(node, name_ + ".xxx", …)` 之后再读（官方教程的写法：`name_ + ".interpolation_resolution"`，`<插件id>.<参数>` 就是插件的命名空间）【文档】。

### 6.3 min-snap 后端"放哪"的最终建议（分阶段）

| 阶段 | 做什么 | 为什么 |
|---|---|---|
| **P0** | 库原样保留，加 `tdt_nav_kit` 的纯 CMake target + CTest + 上游风格 benchmark（**无 ROS 也要能跑**）；**Release/-O2 必须固定住** | 保住"库自己的测试/基准"，也是"纯 C++ 形态没被破坏"的可验证证据。上游 `CMakeLists.txt` 本身就写死 `set(CMAKE_BUILD_TYPE "Release")` + `-O2`；nav2 对 Ceres 也做同样的事（`# significant Ceres optimization speedup`，§4.1 引用） |
| **P1** | `GlobalPlanner` 插件（A\* 前端 + min-snap 后端 → `Path`），`sample_dt` 显式化 | 契约最便宜、可直接 A/B（`planner:=tdt`） |
| **P2** | A/B：`tdt` vs `navfn` vs `smac2d`，指标用现成 harness + §3.6 的埋点（**务必补 P95/P99**） | 结论必须来自我们机器上的数，而不是上游那张 avg/max 表 |
| **P3（可选）** | 若真需要"按时标执行"（过洞/上台阶时间窗）→ 再写 `nav2_core::Controller` 或独立节点，复用 P1 的适配层（同一份 `syncCostmapToLibrary`） | 见 §2.1：只有 Controller/独立节点能消费时标 |

**不建议**：一上来就做独立进程 + 共享内存（§4.4 证据最弱、复杂度最高、收益未被任何来源证明）。

---

## 7. 未能核实 / 明确"没有来源"的清单

1. **`docs.nav2.org/humble/` 不存在**（返回 404 的 "This page moved" 页）；`navigation.ros.org` 在新文档上线后不可用（本机 `curl` 连接失败/仅剩代理响应）。⇒ **Humble 版本的插件接口只能以本机 `1.1.20` 头文件 + `humble` 分支源码为依据**，本文已如此处理。
2. **pluginlib/`dlopen` 的具体加载耗时**：搜索到的只有机制说明（`class_loader` README、插件教程），**没有找到任何 µs/ms 级实测数字** ⇒ 列为"必须自测"（§3.6 第 1 条）。
3. **虚函数调用 vs 直接调用的实测差值**：无来源（量级推理已给，但未引用）。
4. **`nav_msgs::Path`（数百点）发布/序列化的实测耗时**：无来源；本文只给了按消息字段推算的 ≈72–80 B/pose、300 点 ≈21–24 KB【推算】。
5. **ROS 2 Humble + Fast DDS 环回 pub/sub 的权威延迟数字**：未取到。已知可引用的只有：rclcpp 比裸 DDS 最多 +50%（转引）、ros2_tracing 3.3 µs 自身开销、iceoryx 厂商宣称 <1 µs。
6. **nav2 issue #6429 / #6461 在 Humble 1.1.20 上是否可复现**：未验证。#6429 是 Jazzy + ASan，#6461 是 Lyrical + TSan；本文只核对了 **Humble 源码里同类结构存在**（`CostmapSubscriber` 的 `resizeMap`、`mapUpdateLoop` 持 `_dynamic_parameter_mutex` 调 `updateMap`），**不声称 Humble 已中招**。
7. **colcon 对 `package.xml` 里 `<build_type>cmake</build_type>` 的官方说明页**：未取到（`colcon.readthedocs.io` 的若干路径 404）。但**本机可复核**：Humble 安装里 10 个包（含 `ompl`、`cartographer`）就是这么声明的 ⇒ 结论仍成立。
8. **Rolling 文档与 Humble 的完整差异清单**：本文只标注了影响契约的几处（`createPlan` 参数、`plugins` vs `planner_plugins`、`setPlan` vs `newPathReceived`、path handler 插件类型）。未做逐页 diff。
9. **"独立进程 + SHM 跑规划器"的真实项目案例**：未找到（只有中间件层论文/文档）。因此 §0.1 里模式 C 的"何时选它"写成了"只有在已证明必要时"。
10. **上游库的 P95/P99 分位数**：上游只公布 avg/max（1000 次），无分位数。
11. **`MinimumSnap` 的时标到底"严格等间隔"吗**：本次读到的是"按 `dt` 累减采样 + 段末补点"（`minimumSnap.cpp:541-551`），⇒ **段边界处最后一个采样点可能短于 `dt`**；段长由 `timeAllocated` 决定。所以"点 k 的时刻 = k·dt"只是**近似约定**，要精确用必须自己按 `timeAllocated` 重建时标（或让上游开一个"输出 `(pos, t)` 对"的接口）。**这一点建议在 P0 阶段用单元测试钉死。**
12. **本文的"我们的实测/文档"引用均为读取现有 `docs/`、`src/`、`tools/` 文件所得；本次未新跑任何仿真，未改任何代码/配置/launch/其它文档。**

---

## 8. 参考来源汇总（按类型）

**上游库（第三方）**
- 仓库与 README（含性能表、运行参数表）：<https://github.com/T-DT-Algorithm-2026/tdt-nav-kit>（`README.md`）
- `src/YAstar/yastar.hpp|yastar.cpp`、`src/YAstar/kinodynamicAstar.hpp`、`src/MinimumSnapOsqp/minimumSnap.hpp|minimumSnap.cpp`、`src/MinimumSnapOsqp/sfcSquare.hpp|sfcSquare.cpp`、`main.cpp`、`CMakeLists.txt`、`doc/MinimumSnap.md`、`doc/Astar.md`、`doc/Usage.md`（同仓库 `main` 分支）

**nav2 源码（Humble 分支 = 我们的 1.1.20 同族）**
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_planner/src/planner_server.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_smoother/src/nav2_smoother.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_subscriber.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp>
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_navfn_planner/src/navfn_planner.cpp>、`.../nav2_navfn_planner/{CMakeLists.txt,global_planner_plugin.xml}`
- <https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/{CMakeLists.txt,include/nav2_smac_planner/node_hybrid.hpp}>、<https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/path_handler.cpp>
- 并发问题：<https://github.com/ros-navigation/navigation2/issues/6429>、<https://github.com/ros-navigation/navigation2/issues/6461>

**本机安装的 nav2 1.1.20（`/opt/ros/humble/`，逐字核对，版本最准）**
- `include/nav2_core/{global_planner,smoother,controller,goal_checker,progress_checker,exceptions}.hpp`
- `include/nav2_costmap_2d/nav2_costmap_2d/{costmap_2d,costmap_2d_ros,layered_costmap,costmap_subscriber,cost_values}.hpp`
- `include/nav2_util/node_utils.hpp`
- `share/nav2_msgs/{action,msg}/*`、`share/nav_msgs/msg/{Path,OccupancyGrid}.msg`、`share/geometry_msgs/msg/*`、`share/trajectory_msgs/msg/*`
- `share/{nav2_navfn_planner,nav2_smac_planner,nav2_theta_star_planner}/*.xml`、`share/cartographer/package.xml`、`share/ompl/package.xml`（`build_type=cmake` 的 10 个包）

**nav2 官方文档（Rolling；Humble 页面已下线，差异见 §1）**
- 插件教程（planner / controller / smoother）：<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/>、<https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/>、<https://docs.nav2.org/rolling/tutorials/general_tutorials/adding_smoother/adding_smoother/>
- 插件总目录：<https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/>
- **性能章节（RMW/组合/IPC 的 CPU 实测表）**：<https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/>
- 服务端参数：<https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/>、`.../configuring_smoother_server/`、`.../configuring_velocity_smoother/`、`.../configuring_lifecycle_manager/`
- profiling 教程：<https://docs.nav2.org/rolling/tutorials/general_tutorials/get_profile/get_profile/>

**ROS 2 官方**
- class_loader / pluginlib 机制：<https://github.com/ros/class_loader>
- Executors（含"不适合实时"的原文）：<https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html>
- Callback groups：<https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html>
- intra-process 设计（≥5 KB 规则、仍不如非 ROS 共享内存）：<https://design.ros2.org/articles/intraprocess_communications.html>
- real-time 背景：<https://design.ros2.org/articles/realtime_background.html>
- `rclcpp::LoanedMessage`：<https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/loaned_message.hpp>
- 论文：ros2_tracing <https://arxiv.org/abs/2201.00393>；ROS 2 通信实时性评估 <https://arxiv.org/abs/1809.02595>、<https://arxiv.org/abs/1808.10821>
- 中间件：Fast DDS SHM <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>；iceoryx <https://iceoryx.io/classic/v2.0.0/getting-started/what-is-iceoryx/>

**真实工程案例（第三方）**
- HWSentryNav26（浙大 Hello World，**不用 nav2** 的哨兵导航）：<https://github.com/Polyacetone/HWSentryNav26>（`README.md`、`DESIGN.md`、`TUTORIAL.md`、`nav_executor/include/nav_executor/common/trajectory/{annotated_path,path_speed_profile}.hpp`、`nav_executor/package.xml`、`map_server/package.xml`）
- pb2025_sentry_nav（深北莫 PolarBear，**nav2 + 自研插件/节点**）：<https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav>（`pb2025_nav_bringup/package.xml`、`pb2025_nav_bringup/config/simulation/nav2_params.yaml`、`terrain_analysis/package.xml`）
- pb_nav2_plugins：<https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins>；pb_omni_pid_pursuit_controller：<https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller>
- EGO-Planner（无人机栈的自定义带时标消息）：<https://github.com/ZJU-FAST-Lab/ego-planner>（`src/planner/plan_manage/src/traj_server.cpp`、`src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg`）
