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

> 前两个是**离线**工具（吃文件），第三个需要仿真/实车在跑（吃 `/tf`）。
