# tools —— 工具脚本根目录

布局约定：
- `tools/*.py`（本目录根）＝ **Python 工具脚本**（评测、数据后处理、打包导出等）
- `tools/scripts/` ＝ **Shell 脚本**（原仓库顶层 `scripts/` 合并至此）

```
tools/
├── *.py            # Python 工具（按需新增）
└── scripts/        # sh 脚本
    ├── build.sh
    ├── control/        # 启动/控制（start_sentinel.sh、improved_teleop.sh 方向键遥控）
    ├── mapping/        # 建图工具（存图、存 pcd）
    ├── create_config_package.sh
    └── setup_from_package.sh
```

> 历史：原顶层 `scripts/` 于 2026-09 合并入 `tools/scripts/`，旧路径引用已同步更新。

## 工具清单

| 工具 | 用途 | 判读要点 |
|---|---|---|
| `check_map_reachable.py` | 栅格地图**可达性/目标点**检查（nav2 选点前必跑） | 连通域被切块 = 地图有细墙/幽灵墙 |
| `pcd_to_grid_map.py` | LIO 的 3D `.pcd` 切层投影成 2D `.pgm/.yaml`（第三条地图来源） | 与已有图比对覆盖度 |
| `monitor_map_odom.py` | **实时监视 `map→odom` 曲线并判读**（只读诊断，需运行时） | ①平滑漂移（正常矫正）/ ②锯齿（局部 SLAM 走偏 + 全局事后矫正 → 留残影）/ ③高频抖动（查局部匹配/时间戳），见文件头与 `docs/smoke_test_runbook.md` §5 |
| `analyze_slam_bag.py` | **离线分析 bag**（先 `ros2 bag record`，再跑它）：时间戳体检 / IMU 启动零值帧 / 两条里程计是否一致 / `map→odom` 是否在甩 | ⚠️ 判"在甩"看**速率抖动 P95**，不是逐样本 Δyaw（机器人真在转时后者本来就大）；与 `monitor_map_odom.py`（在线）互补 |
| `seg_bench_offline.cc` | **linefit 单帧耗时离线基准**（把录到的点云直接喂核心库，不经 ROS/DDS） | 用来判"感知链卡顿是算力还是交付"：实测 **1.01 ms/帧** ⇒ 不是算力（详见 `docs/issues_and_findings.md` #25） |
| `replay_scan_grid.py` | **★ 把 bag 的 `/scan`（或 `/segmentation/obstacle` 重算的高度带、或原始 3D 点云）+ `/tf` 位姿离线重放成 cartographer 2D 概率栅格**：`--mode events` 存"每周期命中/只清不命"事件矩阵，`--mode sweep` 秒级扫参数，`--mode diag` 出"墙留住的 vs 化掉的"两组票数 | 写入顺序逐条照抄 `probability_grid_range_data_inserter_2d.cc`（命中先写、同周期先写者胜、**跨帧无保护**）。**校验**：对 ret4 同轨迹，真 `/map` 末态 40.5% ↔ 复现 43.3%、留存+10 帧 52.9% ↔ 52.8% ⇒ 相对排序可信。⚠️ **跨 bag 比"留存百分比"无意义**（ret/ret2/ret3/ret4 是四条不同路线/时长的 bag），只在同一条 bag 上比。⚠️ 位姿用 `--pose tf`（`/odom_ground_truth` 是世界坐标，与 `map` 系差一个出生点平移） |
| `diag_wall_passes.py` | **量化"到底是谁把墙格子擦掉的"**：抽样真 `/map` 的墙格子，逐帧统计"命中 vs 被更远回波压过"，并给出"远多少"的分布 | 结论：墙格子每帧只有 ~12% 命中、~17% 被压过（净票≈0）；压过它的回波 **82% 在 50cm 以内**（同一条墙的邻格，"锯齿+掠射"）⇒ 不是"墙看不见了"。见 `docs/debug_fastlio_cartographer.md` §5.2.4 |

> 前两个是**离线**工具（吃文件），第三个需要仿真/实车在跑（吃 `/tf`）。
