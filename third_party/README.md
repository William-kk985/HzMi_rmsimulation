# third_party —— 官方原版第三方库（只读对照）

本目录存放**网上最官方、最源头**的第三方算法库原版（fresh 上游），用途：

1. **原版对照**：`src/` 里使用的版本（含本地改动/适配）与原版 diff，方便看清"我们改了什么"；
2. **上游同步**：上游更新时对比、合入的基准；
3. **选型参考**：评测新候选算法前，先在此拉原版看接口/配置/许可。

## 规则（重要）

- **只读**：不要在本目录内做任何修改/调参；需要改动的版本放 `src/` 对应角色目录。
- 以 **git submodule** 固定到具体 commit（见下），方便随时 fetch 上游新版本对比。
- 新增候选库：`git submodule add <官方URL> third_party/<名字>`，并在下表登记。

## 当前内容

| 目录 | 官方上游 | 用途对照（src 里的使用版） | 锁定的 commit |
|---|---|---|---|
| `third_party/fast_lio` | https://github.com/hku-mars/FAST_LIO | `src/rm_localization/FAST_LIO` | 7cc4175 (main) |
| `third_party/point_lio` | https://github.com/hku-mars/Point-LIO | `src/rm_localization/point_lio` | 4b86a46 |
| `third_party/cartographer` | https://github.com/cartographer-project/cartographer | （ROS2 版由 apt 提供：ros-humble-cartographer*） | 877157a (2.0.0) |

> 备注：cartographer 本体为 ROS1 时代官方仓库；本工程 ROS2 用法走 apt 的 ros-humble-cartographer/cartographer_ros，此处仅为"最源头"参考。其它用 apt 的库（nav2、slam_toolbox 等）同样无需源码，不入本目录。
