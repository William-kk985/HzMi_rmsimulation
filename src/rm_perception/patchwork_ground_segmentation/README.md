# patchwork_ground_segmentation

`bringup_sim.launch.py` 的地面分割槽位 **`ground:=patchwork`** 的实现：把
[Patchwork++](https://github.com/url-kaist/patchwork-plusplus) 包成**与 `linefit_ground_segmentation_ros`
同契约**的 ROS 2 节点，让下游（`pointcloud_to_laserscan` → `/scan` → costmap / STVL / SLAM）零改动。

**先读 [`docs/ground_segmentation_slots.md`](../../../docs/ground_segmentation_slots.md)** —— 那是槽位的唯一真值：
上游/许可证/pinned commit 考证、契约表、参数表（每个默认值的来源）、槽位用法、A/B 实测、结论与回退。

## 用法

```bash
# 槽位形态（正常用法；默认仍是 linefit ⇒ 现有行为不变）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox ground:=patchwork

# 只起分割器（不要 Gazebo/LIO，方便对着 ros2 bag play 调试）
ros2 launch patchwork_ground_segmentation patchwork_ground_segmentation.launch.py
```

## 这个包里有什么

| 路径 | 是什么 |
|---|---|
| `src/patchwork_ground_segmentation_node.cpp` | 节点本体（唯一契约实现；与 linefit 逐字段对齐） |
| `config/ground_segmentation_sim.yaml` | 31 个参数，每个都标了来源（上游默认 / 上游 ROS / 本仓实测 / 本仓决定） |
| `launch/patchwork_ground_segmentation.launch.py` | 单独启动用 |
| `thirdparty/patchwork-plusplus/**` | vendored 上游库（BSD-2-Clause，pin `3e6903a1…` = v1.4.1）；**见其 `VENDORING.md`** |
| `bench/ground_seg_ab.cpp` | 离线 A/B 台架：同一批点上跑两个库，输出逐点一致率、按高度/倾角/真值类别的保留率与方位覆盖率、单帧耗时 |
| `bench/export_bag_frames.py` | 把 bag 里的点云帧导成裸 `float32 .bin`（供台架） |
| `bench/ground_slot_smoke.sh` | 整栈建图 smoke 一轮（headless 隔离 + 脚本化行驶 + 零 Twist 停车 + 只读指标采集 + 录点云） |
| `bench/smoke_metrics.py` | 上面的只读指标采集器（频率 / `/scan` 内容 / `/map` 格数 / RTF / CPU） |
| `bench/verify_ground_slot.py` | **互斥真值表**：把 LaunchDescription 真跑出来求值 condition，断言恰好 1 个分割器被启用 |

## 快速自检

```bash
source install/setup.bash
python3 src/rm_perception/patchwork_ground_segmentation/bench/verify_ground_slot.py   # 期望 PASS
./install/patchwork_ground_segmentation/lib/patchwork_ground_segmentation/ground_seg_ab --synthetic
```

## 已知限制（详见文档 §9）

- **只在 `mode:=mapping` 下做过整栈 smoke**；`mode:=nav` / `slam_nav` 与各 `localization` 槽位的组合**未跑**。
- 参数**没有调优**（只扫过 `th_dist` 5 档）；`sensor_height` 是仿真标定值（0.226 m），实车必须重标。
- 语义差异：Patchwork++ 会把**坡度 ≤45° 且整体在传感器水平面之下**的面判成地面（这是它的设计），
  代价是 35~60° 的陡面比 linefit 更容易被判成地面。
