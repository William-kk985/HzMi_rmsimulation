# rm_livox_gz_sim —— Gazebo Fortress（`sim_backend:=gz`）的 opt-in 后端资产

本包**只装我们自己的东西**：SDF 世界/模型（SDF ≤ 1.9）、`ros_gz_bridge` 配置、
`sim_backend:=gz` 的 launch 入口、以及"把本仓 URDF 派生成 Fortress SDF"的生成器。

**第三方传感器本体不在这里、也不在本仓库里**（许可卫生：只描述、不再分发）。它由

```bash
tools/scripts/gzsim/fetch_and_patch_mid360.sh
```

按固定 revision 取下来、打上我们自己的三个补丁（`tools/scripts/gzsim/patches/`），
编到独立前缀（默认 `<repo>/.tmp_gzlivox2/gzsim_ws/install`，可用 `GZSIM_WS` 改）。

## 一条命令

```bash
# 1) 编后端（一次性）
tools/scripts/gzsim/fetch_and_patch_mid360.sh

# 2) 起栈（整栈 + 端到端：linefit -> /scan + LIO）
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
    world:=RMUC2026 mode:=slam_nav lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0
```

只想看传感器（不开感知/LIO）：

```bash
ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=False     # 台架世界
```

## 话题（与 Classic 的对应关系）

| 话题 | 类型 | 来源 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 插件实例 #1，**进程内** rclcpp 发布（不经桥） |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | 插件实例 #2，同姿态、同一次求交结果 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 插件把 gz 的 IMU 转成 ROS（`frame_id=imu_link`） |
| `/clock` | `rosgraph_msgs/msg/Clock` | **唯一**经 `ros_gz_bridge` 的东西 |

详表（`frame_id` 约定、QoS、点云字段布局、性能对照、会坏的地方、回退方法）见
仓库文档 `docs/fortress_sim_backend.md`。
