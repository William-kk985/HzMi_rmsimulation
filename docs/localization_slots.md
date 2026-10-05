# 重定位槽位登记表（localization slots）

> 入口：`bringup_sim.launch.py` 的 `localization` 参数（**仅 `mode:=nav` 生效**）
> 当前 choices：`['', 'amcl', 'slam_toolbox', 'icp', 'cartographer']`
> **统一契约**：重定位槽**只负责发布 `map→odom`**；`odom→base_link` 由 LIO（+ `lio_tf_adapter`）负责；Nav2 用 `base_link_fake`。
> **`map→odom` 只能有一个发布者** ⇒ 重定位槽之间、以及与 `mapper:=*` 之间必须互斥。

## 1. 已可用（现成入口）

| 槽位值 | 节点 / 包 | 参数文件 | 需要的资产（现状） |
|---|---|---|---|
| `amcl` | `nav2_amcl` | `rm_navigation/params/nav2_params_sim_base.yaml`（已调：`transform_tolerance 0.3` / `update_min_d,a 0.05` / `recovery_alpha_*` 已打开，见工单 §K） | 2D 栅格图 `rm_nav_bringup/map/RMUL2026.pgm|.yaml` ✅ |
| `icp` | `icp_registration/icp_registration_node`（**我们自己的包**，可改） | `icp_registration/config/icp_registration_sim.yaml`（含 `pcd_path`） | 先验点云 `rm_nav_bringup/PCD/RMUL2026.pcd` ✅（RMUC/RMUL 也有） |
| `slam_toolbox` | `slam_toolbox/localization_slam_toolbox_node` | `slam_toolbox/config/mapper_params_localization_sim.yaml` + launch 注入 `map_file_name=map/<world>`、`map_start_pose=[0,0,0]` | **序列化位姿图 `map/RMUL2026.posegraph(+.data)` ❌ 缺**（RMUC/RMUL 有）⇒ 需先建图并 `serialize_map` |
| `cartographer` | `cartographer_node`（纯定位） | cartographer 配置 | `map/RMUL2026.pbstream` ✅ |
| `''`（留空） | 无重定位：LIO 当绝对定位 + 静态桥补帧 | — | — |

运行示例（把 `localization:=` 换成上表任一项）：
```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
  lio:=fastlio localization:=icp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
```

## 2. 待补入口（**已登记、未实现**）

| 计划槽位值 | 用什么 | 依赖 / 资产 | 落地要点 | 估时 |
|---|---|---|---|---|
| `gicp` | **small_gicp / fast_gicp**（GICP 取代现有 ICP） | PCD 资产已有；GICP 库要引入 | **两条路**：① **在 `icp_registration` 内把配准后端从 PCL ICP 换成 GICP**（复用现有话题/参数/资产，改动集中，**推荐**）；② vendor 上游 `small_gicp_relocalization`（须核许可） | 0.5~1 天 |
| `beluga` | `beluga_amcl`（AMCL 的现代化实现，接口兼容） | 需装 `beluga`/`beluga_amcl`；2D 栅格图已有 | 新增槽位值 + 参数文件（可先沿用 AMCL 参数语义做 A/B） | 0.5~1 天 |
| `scan_context` | Scan Context **全局检索** + ICP/GICP **精配准**（两级） | 需引入 Scan Context 实现 + 用 PCD 建描述子库 | 解决"**车随便摆 / 被搬动**"（ICP 类天生初值敏感）；检索出粗位姿 → 现有精配准 | 2~3 天 |
| `fastlio_loc` | LIO + 先验 PCD 做配准得 `map→odom`（一体化配方） | PCD 已有 | 与我们 `lio_tf_adapter` 有职责重叠 ⇒ 作为**对照实现** | 1~2 天 |
| `teaserpp` | TEASER++ 无初值全局配准 | 需引入 TEASER++ + 特征 | 远期；开销大 | 远期 |

## 3. 新增一个重定位槽的**标准三步**（与 planner/controller 槽同构）

1. `bringup_sim.launch.py` 的 `localization` `choices` 加一项 + 写对应 launch 分支（Node + 参数文件）；
2. 参数文件放 `src/rm_localization/<pkg>/config/`，并保证**只发 `map→odom`**（绝不碰 `odom→base_link`）；
3. `colcon build --symlink-install --packages-select <pkg>`（**新文件/新包必须 build**，否则 install 里没有 → 启动报找不到文件）。

## 4. A/B 协议（怎么比才公平）

- 同一次运行里**比不了**（`map→odom` 只能一个发布者）⇒ **分开跑**，保证 **同一 world / 同一目标 / 同一路线 / 同一 `nav`+`planner`**；
- 指标：
  1. `python3 tools/scripts/diag/record_tf_monotonic.py` → `map→odom` 的**跳变次数/幅度/单调性**；
  2. 位姿话题频率（`ros2 topic hz /amcl_pose` 或对应话题）；
  3. **P0 回归 PASS/FAIL + 到达误差 + 用时**（`tools/scripts/regress/nav_smoke_regression.py`）；
  4. CPU / RTF。
- 记录：`algorithm_matrix.md §四` 加一行 + 本表状态列更新。

## 5. 契约与经验提醒

1. **AMCL 是阈值触发的离散修正器**（`update_min_d/a` 决定更新时机）；**slam_toolbox 定位 / GICP / NDT 是连续型**（每帧匹配、`transform_publish_period` 可到 0.02 s = 50 Hz）⇒ **高速下连续型更"跟手"**（详见工单 §K）；
2. **`localization:=icp/gicp` 类方法初值敏感** ⇒ 常需"固定起点"或"上次位姿"作为初值；要"随便摆"就得配全局检索（`scan_context`）或粒子类（`amcl`/`beluga`）；
3. **恢复行为直接发 `/cmd_vel`**（绕过 velocity_smoother）；**`spin_speed != 0` 时 `fake_vel_transform` 会替换 `angular.z`** ⇒ 高速自转下定位更容易被拖偏，A/B 时把 `spin_speed` 固定为 0.0；
4. **切换重定位后先看 `map→odom`**：`ros2 run tf2_ros tf2_echo map odom`（是否台阶式跳变/是否长期不更新）。
