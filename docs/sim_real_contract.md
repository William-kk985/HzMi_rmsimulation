# 仿真 ↔ 实车契约（区分"共用算法层"与"只为仿真存在的替身"）

> **本文回答一类反复出现的问题**：这个改动会不会上实车？这行是不是为 Gazebo 妥协？该不该为了仿真性能去调算法？
> 判据、分界、清单、禁止项都在这里。
> 相关：`architecture.md` §3.2.4（谁吃什么、消费树）、`issues_and_findings.md`（坑与根因）、
> `tf_interface_contract.md`（帧/话题契约）、`params_ownership_checklist.md`（参数归属 R1–R5）。

---

## 〇、一句话结论

**分界点只有一个：数据源。** 实车是 Livox 驱动，仿真是一个假装成它的 Gazebo 插件。
从话题往下（感知 → LIO → 定位 → 建图 → 导航）两边是**同一份代码**，差异**只允许出现在参数文件里**。

因此：
- 「感知层是不是为仿真服务的？」——**不是**。它是实车也要跑的算法层，仿真只是换了个数据源喂它。
- 「为仿真存在的东西在哪？」——集中在 **数据源替身 / 执行机构替身 / 仿真独有观测**（§一）。
- 「症状出现在感知层」≠「根因在感知层」。这一轮踩的坑（插件时间基、RTF、spawn 高度）全部落在数据源侧，
  但症状（`/scan` 掉到 2.8 Hz、地图跟着车转）显示在感知层的输出上。

---

## 一、分界点：一棵树，两类替身

```
实车: Livox MID360 硬件 ──livox_ros_driver2──┐
                                             ├─→ /livox/lidar(CustomMsg) /livox/imu /livox/lidar/pointcloud
仿真: Gazebo ──livox_laser_simulation_RO2────┘        │
                    ▲ 仿真替身①                       ├─→ FAST-LIO / Point-LIO        → /odom
                                                     ├─→ complementary_filter       → /imu/data
                    ▲ 仿真替身②                       └─→ linefit → p2l              → /segmentation/obstacle → /scan
       gazebo_ros_planar_move（底盘运动 + 真值 odom）          ↓
                                              slam_toolbox / cartographer / nav2 costmap
```

| 角色 | 实车（`archive/reality_frozen/bringup_real.launch.py` 实测） | 仿真（`bringup_sim.launch.py`） | 性质 |
|---|---|---|---|
| 雷达数据源 | `livox_ros_driver2_node` + `MID360_config.json` | `livox_laser_simulation_RO2`（Gazebo 插件） | **替身①** |
| IMU 预处理 | `complementary_filter_node` | 同 | **共用** |
| 地面分割 | `ground_segmentation_node`（linefit） | 同 | **共用** |
| 2D 激光 | `pointcloud_to_laserscan_node` | 同 | **共用** |
| 车体运动 | 下位机 + CAN | `gazebo_ros_planar_move` | **替身②** |
| 真值里程计 | 无（实车拿不到真值） | `/odom_ground_truth`（`publish_odom_tf=false`，10 Hz） | **仿真独有观测** |
| 云台 / 小陀螺 | 真实机构 | `fake_vel_transform`（纯软件模拟旋转补偿） | 半替身（节点两边都有，仿真缺机构） |
| LIO | `fastlio_mapping` / `pointlio_mapping` | 同 | **共用** |
| `odom→base_link` | 实车快照用 `static_transform_publisher`（旧） | `lio_tf_adapter`（新，含杆臂补偿） | **已分叉** ⚠️ 见 §六 |
| 定位 / 建图 | `slam_toolbox` / `icp_registration` | 同 **+ `cartographer`**（实车快照里没有） | 部分分叉 |
| 导航 | nav2 | 同（参数分 sim/real 变体） | 共用 |

---

## 二、共用接口契约（改这里=两边都改，必须先拆变体）

| 话题 | 类型 | 生产者 | 消费者 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 驱动 / 插件 | FAST-LIO、Point-LIO |
| `/livox/lidar/pointcloud` | `sensor_msgs/PointCloud2` | 驱动 / 插件 | linefit |
| `/livox/imu` → `/imu/data` | `sensor_msgs/Imu` | 驱动+互补滤波 / 插件+互补滤波 | FAST-LIO |
| `/segmentation/obstacle` | `sensor_msgs/PointCloud2` | linefit | p2l、STVL、`local_obstacle:=cloud` |
| `/scan` | `sensor_msgs/LaserScan` | p2l | cartographer、slam_toolbox、nav2 |
| `/odom` | `nav_msgs/Odometry` | LIO（经 T2 remap） | cartographer、nav2、`lio_tf_adapter` |
| `/odom_ground_truth` | `nav_msgs/Odometry` | `planar_move` | 对比用；cartographer 全包形态的先验 |
| `/cmd_vel` → `/cmd_vel_chassis` | `geometry_msgs/Twist` | nav2 / 遥控 | `fake_vel_transform` → `planar_move` |

帧契约（`livox_frame` / `imu_link` / `base_link` / `base_link_fake` / `odom` / `map`）见 `tf_interface_contract.md`。

---

## 三、参数分家清单（允许不同，而且**必须**不同）

| 参数 | sim | real | 为什么不同 |
|---|---|---|---|
| linefit `sensor_height` | `0.275` | `0.59` | 雷达离地高度是**物理事实**，两边不同 |
| p2l `scan_time` | `0.1` | `0.3333` | 传感器出帧周期；sim 侧已按 10 Hz 修正 |
| p2l `range_min` | `0.2` | `0.45` | sim 侧修正值（0.45 会造成贴身盲区 + `inf` 被清成 free）；**实车快照未同步** ⚠️ |
| nav2 / costmap 参数 | `nav2_params_sim_*.yaml` | `nav2_params_real.yaml` | 场地尺寸/传感器噪声/速度上限不同 |
| LIO 外参 | sim URDF 的 `livox_frame`/`imu_link` | `MID360_config.json` + `measurement_params_real.yaml` | 安装尺寸不同 |
| **运动先验来源**（cartographer `use_odometry`） | `/odom_ground_truth`（`gazebo_ros_planar_move`，**无打滑的理想值**，10 Hz） | 下位机轮速 odom（会打滑、有延迟、有噪声） | 对应关系成立，但**仿真的先验"太好"** → 台架在这项上比实车容易；要复现实车退化需注入打滑/延迟（见 `issues_and_findings.md` §六 退化测试槽位） |

> ⚠️ **实现方式不一致 = 漂移风险**：linefit 用 `segmentation_sim.yaml` / `segmentation_real.yaml` 两份文件；
> 而 p2l 的参数在实车栈里是**内联写在 launch 里**的（`bringup_real.launch.py:174-187`），
> 结果就是 sim 修好的值没有回流。**实车解冻时应把 p2l 参数改成 `laserscan_params_sim.yaml` / `_real.yaml` 变体。**

---

## 四、三分类判据（唯一标准）

问一句：**这个改动改变的是「算法看到的世界」，还是「仿真对算法的失真」？**

| 类别 | 判定 | 例子 | 处理 |
|---|---|---|---|
| **改感知/传感器输出内容** | 降级 / 妥协 | `samples 30000→12000`、`update_rate 10→5`、`downsample>1`、为了省 CPU 调粗 `n_segments` / `angle_increment` / `n_bins` | ❌ **拒绝**：会污染台架结论，实车上不成立 |
| **消除仿真引入的失真** | 修复 | 插件逐点时间基（墙钟→仿真钟）、时间戳同轴、TF 多父边、spawn 高度、世界网格缺失 | ✅ **该做**：让台架结论可信（`issues_and_findings.md` #19） |
| **世界本身的差异** | 参数分家 | `sensor_height`、场地尺寸、噪声/延迟标定 | ✅ 正常：两边各一份配置（§三） |

**两类"看起来像妥协"但要分清的东西：**
- QoS / 队列深度 / 线程数 / CPU 亲和性 / 去掉恒等 TF 过滤器 = **管道类**：不改变感知输出语义 → 随便改，零 sim2real 风险；
- 反过来，**不要**把"消除仿真伪影"当借口去改算法语义。

---

## 五、禁止清单

1. **不许为了让仿真跑得动而改感知输出内容**（§四第一类）。仿真的算力账应该在**仿真侧**还（少打射线、关 GUI、换更省的链路），不是在算法侧还。
2. **不许把仿真专属结论当实车结论**。已确认的例子：cartographer 的 `use_odometry=false`
   —— 它是因为**仿真里** `/odom` 与 `/scan` 同源同戳、且 LIO 的 odom 晚到会撞 collator 的时间序 CHECK（`exit -6`）；
   实车上轮速/IMU odom 有自己的时间戳来源，先验是**正资产**，必须重新评估。
3. **不许改共用文件却不拆变体**（§三末的 p2l 反例）。
4. **不许只凭"仿真里好用"就定参数**：任何在仿真里定下来的参数，若它属于共用算法层，必须标注
   「在什么条件下成立」（例：搜索窗 30° 的成立条件是 *相邻 scan 间隔 × 角速度 ≤ 30°*）。

---

## 六、实车冻结快照的落后项（解冻时必须同步）

| 项 | 实车快照现状 | 仓库当前值 / 结论 | 为什么必须同步 |
|---|---|---|---|
| `odom→base_link` 杆臂补偿 | 用 `static_transform_publisher`，无 `lio_tf_adapter` | `lio_tf_adapter` 带 `xyz: [-0.12, 0, -0.125]` | 不补偿杆臂 → 原地转时"地图绕车画圆"（同根因见 `issues_and_findings.md` #18 / nav2 #4135） |
| p2l `range_min` / `scan_time` | `0.45` / `0.3333` | `0.2` / `0.1` | 贴身盲区 + `inf` 被当自由空间清掉 |
| 时间同步判据 | 未记录 | 戳与逐点 offset **必须同源同轴**；`/odom` 不得出现未来戳 | 缺 PPS / `time_sync_en=false` 时触发**同一个失效模式**：假去畸变 → 转圈时整帧被拧 |
| cartographer | 实车栈里没有 | bench 里有（`cartographer.lua`） | 若要上实车，先验来源应换成**轮速 odom**（不是 LIO 自己的 `/odom`） |

---

## 七、sim2real 验收判据（这几条可以**直接**搬到实车用）

1. **时间基同轴**：`ros2 topic echo /odom --field header.stamp --once` 与 `/scan` 的戳同一条时间轴；
   `/odom` 的戳不得超前于 `/clock`（不允许"未来戳"）。
2. **静止 + 原地自转**：点云不分层、`tf2_echo map base_link` 的 x/y 不画圆（杆臂/时间基都对的联合判据）。
3. **丢帧判读**：看 `/scan` 的 `min/max` 间隔 —— **`min` 能到满速（1/传感器率）说明是"卡顿成串"，不是"算力均匀不足"**；
   并套用不等式：**相邻 scan 间隔 × 角速度 ≤ 扫描匹配搜索窗**（当前 30°；实车把"间隔"换成实际出帧间隔）。
4. **建图验收**：回环必须**真的命中** —— cartographer 收尾 `Score histogram` 不能是 `Count: 0`
   （一条回环都没有 = 位姿图无全局锚定，地图只剩 local SLAM）。
5. **先验必须独立**：喂给优化器的运动先验不应由**被优化的同一份传感器**产生
   （LIO 的 `/odom` 来自同一份点云 → 反馈回路；应该用轮速/底盘 odom）。

---

## 八、变更流程（本仓库约定）

1. 动手前先归类：这次改动落在 **数据源替身 / 共用算法层 / 参数分家** 的哪一类（§一/§四）；
2. 共用层的改动：必须同时给出 **sim 与 real 两侧结论**，否则先拆 `*_sim` / `*_real` 变体；
3. 记录落点：坑与证据 → `issues_and_findings.md`；接口/帧变更 → `tf_interface_contract.md`；
   参数归属 → `params_ownership_checklist.md`；**本文件只维护"分界与判据"**，不重复记录具体故障。
