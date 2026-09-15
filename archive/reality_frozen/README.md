# reality_frozen —— 真车配置冻结快照（2026-09 归档）

本目录是 `rm_nav_bringup`（原 `config/reality/` 与 `bringup_real.launch.py`）的**归档快照**，原因：

- 本仓库定位为**仿真实验台**，只维护 simulation 侧配置；
- 按改造规则 R5，真车（reality）配置**冻结**、不再与仿真双份同步；
- 真车部署配置将在**部署导出阶段**（见 `docs/rm_bench_refactor_plan.md` §4.6）按"选定组合的包清单"另行生成。

**归档内容**：fastlio / pointlio / icp / segmentation / slam_toolbox / nav2 的 real 参数、`MID360_config.json`（真车雷达 IP 与外参）、`measurement_params_real.yaml`、以及真车总入口 `bringup_real.launch.py`。

**使用注意**：这些文件已不参与 colcon 构建/安装，直接 `ros2 launch` 会因路径失效而不可用；仅作**参考基线**（例如核对真车雷达参数、迁移到部署导出工作区时对照）。
