# gzsim —— Gazebo Fortress（`sim_backend:=gz`）后端的取用与补丁

这个目录里**没有任何第三方源码**（除了 `patches/*.patch` 里引用的上游代码上下文行，
那是补丁格式的必然产物，不构成再分发）。我们只做三件事：

1. 把上游**钉在固定 revision**；
2. 打**我们自己的**补丁（每个补丁头部写明改了什么、为什么、对应哪条实测）；
3. 把结果编到独立前缀（默认 `<repo>/.tmp_gzlivox2/gzsim_ws/install`）。

## 一条命令

```bash
tools/scripts/gzsim/fetch_and_patch_mid360.sh          # 获取 + 打补丁 + 编译 + 产物自检
tools/scripts/gzsim/fetch_and_patch_mid360.sh --no-build
GZSIM_WS=/opt/gzsim_ws tools/scripts/gzsim/fetch_and_patch_mid360.sh   # 换前缀
```

编完 `source <GZSIM_WS>/install/setup.bash`（本仓的 launch 会自动找到前缀，也接受
`GZSIM_PREFIX=/path/to/install`）。

## 上游 pin 与许可

| # | 上游 | pin | 许可证 | 我们动了什么 |
|---|---|---|---|---|
| ① | <https://github.com/uos/rmagine> | tag **`2.4.3`**（`2962242278193dcf8d63697ab3c93900a5513a23`） | **BSD-3-Clause**（`LICENSE`） | **零改动**。raycasting 引擎本体（Embree 后端） |
| ② | <https://github.com/uos/rmagine_gazebo_plugins> | commit **`c6dc04e2570fe64742d2e6a54e0641b0e5bf7fde`** | **BSD-3-Clause**（`LICENSE`） | `0001`：+35 行，加「按 model 实体自机排除」的运行时 API（见下） |
| ③ | <https://github.com/ashduwihch/mid360_gazebo_harmonic> | commit **`497149a82b5fcf7ca0645995ba0b3b937fe033a6`** | **MIT**（`LICENSE`） | `0002`（Fortress 化，+71/−20 行）+ `0003`（ROS 契约，+47/−6 行） |

> 上游 ③ 的仓库里也**自带**了 ①② 的副本（`packages/rmagine`、`packages/rmagine_gazebo_plugins`）。
> 本脚本**只取它的 `packages/mid360_simulation_plugin_ros2` 一个包**，①② 用各自上游的
> 钉住版本 —— 避免"跟着一个第三方仓库的 vendored 快照漂移"。
> 已知后果：③ 自带的那份 `rmagine_gazebo_plugins` 是个改过的 fork（它建 BVH 走
> `<collision>`、支持多 mesh Collada）。我们用**上游**那份（走 `<visual>`），在本仓的
> 资产（STL、visual≈collision）上等价；这一点在 `docs/fortress_sim_backend.md`
> 的「与之前的区别」表里有对照。

### 补丁清单（都在 `patches/`，`git apply -p1` 可直接打）

| 文件 | 目标上游 | 内容 |
|---|---|---|
| `0001-rmagine-gazebo-plugins-self-exclusion.patch` | ② | `MapRegistry` 增加 `AddIgnoredModelEntity` / `RemoveIgnoredModelEntity` / `IsIgnoredModelEntity`；`RmagineEmbreeMapSystem::IsIgnoredVisual()` 在祖先链上多查一次。作用：**自机排除** —— 传感器插件把"自己所属的 model 实体"登记进去，建 BVH 时整棵子树被剔除 ⇒ 机器人看不见自己的车体。为什么不用上游已有的 `<ignore_model>名字</ignore_model>`：那是**按 model 名字**匹配，同一个模型被 include 两次时 Gazebo 会加 `_0` 后缀，名字匹配不可靠。 |
| `0002-mid360-fortress-compat.patch` | ③ | Fortress（ignition-gazebo6）化：CMake 包名/目标名（`gz-sim8`→`ignition-gazebo6` 等 4 个）、注册宏（`GZ_ADD_PLUGIN`→`RMAGINE_GZ_ADD_PLUGIN`）、`components/RaycastData` 的 `#if`（gz-sim6 没有这个组件，physics 后端整个关掉，只留 Embree）、以及 **`gzerr/gzwarn/gzmsg/gzdbg` → `ignerr/…` 的 4 个 `#define`**（gz-common4 只定义 ign* 拼法，gz* 拼法是 gz-common5+ 才加的）。 |
| `0003-mid360-ros-contract.patch` | ③ | 与本仓 ROS 契约对齐：① `<qos_reliability>`（默认 `best_effort` = `SensorDataQoS`；`reliable` 只作复现开关）；② `<node_name>`（双实例不能共用 node 名）；③ `<publish_imu>`（IMU 只能有一个实例发）；④ `<imu_frame_id>`（Classic 的 `/livox/imu` 是 `imu_link`，不是 `livox_frame`）；⑤ `<ignore_self>`（默认 true；false 只用于"自机排除到底有没有用"的对照实验）。 |
| `0004-mid360-custommsg-tag.patch` | ③ | `/livox/lidar`（CustomMsg）每点 `tag` 从写死的 `0x10` 改成 `<custom_msg_tag>`、**默认 0**。理由：`small_point_lio` 的适配器要求 `(tag & 0x3f) == 0`，`0x10` 会让它把**每帧每个点都丢掉** ⇒ 永不初始化（无 `odom→base_link`、无 `/cloud_registered`）；Classic 的 `libros2_livox.so` 从不写 `tag`（默认 0）。`0x00` 仍在 FAST-LIO / Point-LIO 的白名单里（它们只要求 `(tag & 0x30) ∈ {0x00,0x10}`），所以三个槽位都收。传 `16` 可复现上游行为。 |

**补丁行的计数**：`git diff --stat` 在打完后是
`0001: 2 files, +35`、`0002: 2 files, +71/−20`、`0003: 1 file, +47/−6`、
`0004: 1 file, +13/−1`。

## 构建期依赖：Embree 4（Apache-2.0，构建期下载）

`rmagine` 的 CMake 用 `FetchContent` 从 GitHub 拉**预编译**的
`embree-4.4.0.x86_64.linux.tar.gz`（apt 只有 3.12，**不可用**）。所以：

* 首次 configure **需要联网**（本机实测 13 s 下完、整包 13.4 s 编完）；
* 离线/CI：把 tarball 预置到 CMake 的下载缓存，或
  `-DRMAGINE_DEPENDENCY_MODE=Local` 指到系统里的 Embree ≥ 4；
* Embree 是 **Apache-2.0**，我们从不再分发它（只下载、只链接）。

## 自检（脚本第 4 步会跑）

```
ok   librmagine_embree_map_system.so  (592K)
ok   libmid360_system.so  (704K)
ok   链到 ignition-gazebo6（不是 gz-sim8）
ok   导出 IgnitionPluginHook（Fortress 注册宏生效）
```

> ⚠️ 写自检时踩到一个坑，留在这里免得后人重踩：**不要**写成
> `nm … | grep -q 符号`。`grep -q` 一命中就关管道，写端吃 SIGPIPE(141)，
> 在 `set -o pipefail` 下整条管道被判成失败 —— 明明有符号也会报「没有」。

## 同目录其它工具

| 文件 | 用途 |
|---|---|
| `gz_livox_probe.py` | 契约/性能探针：两路话题一起收，出「仿真频率/墙钟频率/点数/`offset_time`/`line`/`frame_id`/字段布局/两实例逐帧一致性」，并支持 `--slow-ms`（慢消费者）与 `/clock` 观测 |
| `launch_increment_diff.py` | 证明 `bringup_sim.launch.py` 的改动是**纯增量**：用同一组参数构建两个版本的整棵 Action 树，做多重集 + 保序比较 |
