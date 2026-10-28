# scan_context_relocalizer

`global_relocalizer:=scan_context` 槽的实现：**Scan Context 全局地点检索 → `/initialpose` 粗位姿**。
契约见 `docs/frame_contract_v3.md` §4.16，实测见 `docs/frame_contract_v3_measurements.md` §42。

## 它做什么 / 不做什么

* 做：取一帧 `/cloud_registered`（odom），用 TF 摆平到 `odom_level`，减去 `base_link` 原点 →
  SC 描述子 → 查 `PCD/<world>_sc_db.npz` → 闸门（`score`/`margin`）通过后发
  `/initialpose`（map 系机器人位姿）+ `~/health`（≥1 Hz）+ 可选 `~/candidates`。
* **不做**：不发任何 TF；`map→odom` 的唯一发布者仍是 `localization:=gicp|small_gicp|icp|amcl|beluga`。
* 失败不静默：DB 缺 → launch 在起 Gazebo 前报错；匹配差 → `health.valid=false/state=NO_MATCH`，
  **不发** `/initialpose`。

## 建库（离线，一次）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run scan_context_relocalizer build_sc_db -- \
  --bag <mapping-run>/bag/raw --world <world> \
  --pcd <rm_nav_bringup>/PCD/<world>.pcd \
  --out <rm_nav_bringup>/PCD/<world>_sc_db.npz
colcon build --symlink-install --packages-select rm_nav_bringup
```

输入只需建图跑的 `/cloud_registered` + `/tf` + `/tf_static`（无需真值）。格式：
`descriptors (K,20,60)`、`ring_keys (K,20)`、`poses (K,4,4)=T_map_base`、
`level_yaws (K,)`、`stamps (K,)`、`meta(JSON)`。

**为什么需要 `level_yaws`**：SC 描述子表达在**重力对齐 level 帧**里（世界朝向），
跨会话时两个 level 帧的 yaw 不同（LIO 初始朝向），必须用
`yaw_map = level_yaws[k] + Δ_shift + yaw(T_level←base)` 换算；只用 `yaw_k + Δ` 会差一个
LIO 初始朝向（实测 54°）。

## 运行

launch 槽位（只加这一条参数；默认 `none` 行为不变）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2027 mode:=nav lio:=small_point_lio localization:=gicp \
  global_relocalizer:=scan_context ...
```

单独跑节点 / 手工查库：

```bash
ros2 run scan_context_relocalizer scan_context_relocalizer_node --ros-args \
  -p db_path:=/abs/path/<world>_sc_db.npz -p use_sim_time:=true
ros2 service call /scan_context_relocalizer/relocalize std_srvs/srv/Trigger "{}"
ros2 topic echo /scan_context_relocalizer/health --once
```

参数见 `config/scan_context_relocalizer_sim.yaml`；`~` 下的健康/候选话题在节点名
`/scan_context_relocalizer` 下。

## 测试

```bash
python3 -c 'from scan_context_relocalizer import scan_context as s; print(s.selftest())'
```

`scan_context.py` 是 `third_party/eval/sc.py` 的复制（仅去掉 `eval_lib` 依赖 + 加
`query_yaw_from_shift()`），算法本体未改；selftest 保留（24/25）。
