# tools —— 工具脚本根目录

布局约定：
- `tools/*.py`（本目录根）＝ **Python 工具脚本**（评测、数据后处理、打包导出等）
- `tools/scripts/` ＝ **Shell 脚本**（原仓库顶层 `scripts/` 合并至此）

```
tools/
├── *.py            # Python 工具（按需新增）
└── scripts/        # sh 脚本
    ├── build.sh
    ├── control/        # 启动/控制（start_sentinel.sh、improved_teleop.sh）
    ├── mapping/        # 建图工具（cartographer 快捷、存图、存 pcd）
    ├── create_config_package.sh
    └── setup_from_package.sh
```

> 历史：原顶层 `scripts/` 于 2026-09 合并入 `tools/scripts/`，旧路径引用已同步更新。
