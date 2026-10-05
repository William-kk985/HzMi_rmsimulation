# gicp_registration 的可切换配准后端：`pcl`（默认） vs `small_gicp`

> 2026-10-05 · 对应改动：`gicp_registration` 新增 `backend` 参数 + vendored `third_party/small_gicp`
> 本文只讲这一个包（`src/rm_localization/gicp_registration/`）。
> 契约（只发 `map→odom`、`tf_lookahead_sec` 盖戳、健康话题、"无初值不发 TF"、两级 leaf 0.10/0.05、
> `max_correspondence_distance` 1.5 m、`pcl::PointXYZ`）**一个字都没改**，见 `docs/localization_slots.md`。

---

## 0. 一句话结论（先看这个）

1. **`small_gicp` 后端可用，且更快**：在**同一批帧、同一份参数、同一个构建配置**下，
   仓库当前默认构建（`CMAKE_BUILD_TYPE` 为空 = 无 `-O`）里
   `pcl` 稳态 **386.7 ms/帧**（median）→ `small_gicp`（4 线程）**205.5 ms/帧**，≈ **1.9×**；
   打开 `-O3` 后 `pcl` 12.5 ms → `small_gicp` **2.9 ms**，≈ **4.3×**。
2. **但真正的大头不是换后端，而是"这个包一直在无优化编译"**：`colcon`/`ament` 默认**不设置**
   `CMAKE_BUILD_TYPE` ⇒ 编译命令里**没有任何 `-O`**。用**未改动的 HEAD 代码**只加
   `--cmake-args -DCMAKE_BUILD_TYPE=Release`，同一台机器同一条合成扫描：
   **`pcl` 后端 395 ms → 13 ms（≈30 倍）**，fitness（0.00123 m²）与 `map→odom` 数值完全一致。
   ⇒ 详见 §5，建议团队优先评估这一条（它比换后端便宜得多，且不需要引入任何新依赖）。
3. `~/fitness_score` 的语义**没有**被偷偷改：两个后端都发"**内点平均平方距离（m²）**"；
   `small_gicp` 的原生 `RegistrationResult::error`（Mahalanobis 加权、无量纲）另发在
   **新话题** `~/small_gicp_error`，且**只在 `backend: small_gicp` 时存在**。
4. 回退：`backend: "pcl"`（默认值本来就是它），或整体 `git revert` 引入本次改动的那个 commit。

---

## 1. small_gicp 是什么（已核实的事实，附出处）

| 项 | 值 | 核实方式 |
|---|---|---|
| 上游 | <https://github.com/koide3/small_gicp>（作者 Kenji Koide，AIST） | GitHub REST API `repos/koide3/small_gicp` |
| 许可证 | **MIT**（`LICENSE`："MIT License / Copyright (c) 2024 Kenji Koide"） | `curl https://raw.githubusercontent.com/koide3/small_gicp/v1.0.1/LICENSE` |
| **本仓 pin** | commit **`57c1106daf83c2c79ee0c58a9c7ed0032298ff4e`** = tag **v1.0.1**（2026-06-13） | `api.github.com/repos/koide3/small_gicp/tags`、`.../commits/<sha>` |
| 上游最新 release | v1.0.0（2024-08-09）；v1.0.1 是更新的 tag；master HEAD `fa0cfc98…`（2026-09-29） | 同上。**我们故意 pin tag 而不是 master**（可复现） |
| 形态 | **header-only**（README 原文："You can just download and drop it in your project directory to use it"）；`BUILD_HELPER=ON` 时另外编一个 helper 库 `libsmall_gicp.so`（可选） | 上游 `README.md`、`CMakeLists.txt` |
| 强制依赖 | **Eigen**（唯一必须）+ 自带的 nanoflann/Sophus 源码 | `README.md` §Dependencies |
| 可选依赖 | **OpenMP**（`BUILD_WITH_OPENMP=auto`，默认"找到就用"）、TBB（`BUILD_WITH_TBB=OFF`）、PCL（只给 test/benchmark 用） | 上游 `CMakeLists.txt` 的 `option()` |
| 其它构建开关（**真实名字**，任务书里的 `-DSMALL_GICP_USE_*` 在上游并不存在） | `BUILD_HELPER`(ON) · `BUILD_WITH_OPENMP`(auto) · `BUILD_WITH_TBB`(OFF) · `BUILD_WITH_MARCH_NATIVE`(**OFF，保持关闭**：开了会把二进制绑死在编译机) · `BUILD_TESTS/BUILD_EXAMPLES/BUILD_BENCHMARKS/BUILD_PYTHON_BINDINGS`(OFF) | 同上 |
| 本机依赖现状 | Eigen 在 `/usr/include/eigen3`（`Eigen3Config.cmake` 存在，**不会**触发上游的 FetchContent 联网下载）；OpenMP = GCC 的 `libgomp`（`find_package(OpenMP)` 命中） | 本机构建日志 `compile_commands.json` |

### 1.1 我们实际用到的 API（都在 pinned commit 里核实过源码）

```cpp
#include <small_gicp/points/point_cloud.hpp>              // small_gicp::PointCloud（points/normals/covs）
#include <small_gicp/ann/kdtree.hpp>                      // UnsafeKdTree<>（不持有所有权）/ KdTree<>（shared_ptr 持有）
#include <small_gicp/ann/kdtree_omp.hpp>                  // KdTreeBuilderOMP(num_threads)
#include <small_gicp/util/normal_estimation_omp.hpp>      // estimate_covariances_omp(cloud[, tree], k, threads)
#include <small_gicp/factors/gicp_factor.hpp>             // GICPFactor（distribution-to-distribution）
#include <small_gicp/registration/reduction_omp.hpp>      // ParallelReductionOMP{ num_threads }
#include <small_gicp/registration/registration.hpp>       // Registration<Factor, Reduction, ...>::align()
```

* `Registration<GICPFactor, ParallelReductionOMP>::align(target, source, target_tree, init_T)`
  → `RegistrationResult{ converged, iterations, num_inliers, H, b, error, T_target_source }`
  （`registration_result.hpp`；`T_target_source` 与 PCL `getFinalTransformation()` 同语义）。
* 参数一一对应：`rejector.max_dist_sq`←`max_correspondence_distance²`、
  `criteria.translation_eps`←`transformation_epsilon`、`criteria.rotation_eps`←`rotation_epsilon`、
  `optimizer.max_iterations`←`maximum_iterations`、`optimizer.max_inner_iterations`←`maximum_optimizer_iterations`、
  `reduction.num_threads`←`small_gicp_num_threads`、协方差 KNN←`correspondence_randomness`。
* **也核实过、但本次没用**：`small_gicp::align(...)`（`registration_helper.hpp`，需要链接 helper 库）、
  `voxelgrid_sampling/_omp`（`util/downsampling*.hpp`）、
  `IncrementalVoxelMap`（`ann/incremental_voxelmap.hpp`）。本节点的先验地图是**静态**的，
  不需要增量插入/LRU 淘汰 ⇒ 用静态 `KdTree` 更简单也更快。
* ⚠ **体素下采样我们仍然用 PCL 的 `pcl::VoxelGrid`**（`voxelDownsample()`，两个后端共用）：
  只有源/目标点云**逐点一致**，`align ms` 与 `fitness` 才可比；
  另外 `voxelgrid_sampling_omp` 上游自己注明"多线程下采样有轻微 run-to-run 不确定性（点数最多 +10%）"，
  会让 A/B 的源点数每帧抖动，这也是不用它的原因。

### 1.2 COD 2025 的 ROS wrapper（`koide3/small_gicp_relocalization`）

存在（<https://github.com/koide3/small_gicp_relocalization>），是 small_gicp 作者给的 ROS 1/2 重定位示例。
**我们没有 vendor 它**，只借用了它的参数口径（本仓库两级 leaf 的出处）：

| 它的参数 | 值 | 我们的对应 |
|---|---|---|
| `num_threads` | 8 | `small_gicp_num_threads`（我们用 4，见 §4 的线程扫描） |
| `global_leaf_size` | 0.25 | `voxel_leaf_size`（**我们取 0.10**：0.25 时 RMUL2026 只剩 2438 个 target 点，实测会漂） |
| `registered_leaf_size` | 0.05 | `voxel_leaf_size_scan`（0.05，照抄） |
| `max_dist_sq` | 2.5 | `max_correspondence_distance` 1.5 m（√2.5≈1.58） |

---

## 2. 怎么用（切换后端）

配置文件：`src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml`

```yaml
/gicp_registration:
  ros__parameters:
    backend: "pcl"                # pcl | small_gicp；默认 pcl = 改动前的已验证路径
    small_gicp_num_threads: 4     # 仅 backend=small_gicp 生效（OpenMP 线程数）
```

三种切法（任选）：

```bash
# ① 改配置文件（持久）：把 backend 改成 "small_gicp"
# ② 运行时覆盖（bringup_sim.launch.py 的 localization:=gicp 槽，加在节点参数上）：
ros2 run gicp_registration gicp_registration_node --ros-args \
  --params-file install/gicp_registration/share/gicp_registration/config/gicp_registration_sim.yaml \
  -p pcd_path:=$PWD/src/rm_nav_bringup/PCD/RMUL2026.pcd \
  -p backend:=small_gicp -p small_gicp_num_threads:=4
# ③ bringup launch 里透传（该文件由另一位同学维护，这里只给形状）：
#    ... Node(..., parameters=[params, {'backend': 'small_gicp'}])
```

启动 banner 第一行就会打印当前后端与目标侧预处理耗时（A/B 时第一眼确认自己在跑哪条路）：

```
===== gicp_registration 启动 =====
  ★ 配准后端 backend=small_gicp（参数原值 'small_gicp'；可选 pcl | small_gicp，默认 pcl）
     · pcl        = pcl::GeneralizedIterativeClosestPoint（单线程，已验证路径）
     · small_gicp = koide3/small_gicp（MIT，vendored，pinned 57c1106/v1.0.1；OpenMP 多线程，num_threads=4）
     目标侧一次性预处理：268.1 ms（small_gicp：建 KdTree + 估协方差，构造期一次）
```

**非法取值不会静默换路**：`backend: "foo"` ⇒ 一条 WARN + 退回 `pcl` 并把参数原值打进 banner。

### 2.1 CMake 怎么消费它（含 `COLCON_IGNORE` 的澄清）

* 源码以 **vendored 形态**放在 `third_party/small_gicp/`（去掉了 `.git`，随本仓库一起提交）。
* `gicp_registration/CMakeLists.txt` 用 **`add_subdirectory(${SMALL_GICP_VENDOR_DIR} …)`**，
  链接它导出的 `small_gicp` 目标（PUBLIC 带上 `include/`、`Eigen3::Eigen`、`OpenMP::OpenMP_CXX`）；
  可选路径可用 `-DSMALL_GICP_VENDOR_DIR=/path/to/small_gicp` 覆盖。
* ⚠ **`third_party/COLCON_IGNORE` 只让 colcon 不去发现 `third_party/**` 里的 `package.xml`**
  （small_gicp 自带 `package.xml`，`<build_type>cmake</build_type>`），**对 CMake 毫无影响** ——
  `add_subdirectory` 照样能进；反过来说，删掉 `COLCON_IGNORE` 会让 colcon 把 small_gicp 当成一个
  独立包去 build（并可能和本包的 `add_subdirectory` 抢同一个 install 前缀）。两件事互不相干。
* 为什么不 `find_package(small_gicp)`：那需要系统级 `sudo make install`，破坏"克隆即可构建"。
* 编译告警契约：本包 `-Wall -Wextra -Wpedantic` 必须 0 warning，而上游头文件在 `-Wextra` 下有 2 处
  自身告警（`rejector.hpp:23` unused parameter ×5、`kdtree_omp.hpp:62` sign-compare）。
  试过把它变成 `-isystem`（两种写法都失败：只设 `INTERFACE_SYSTEM_INCLUDE_DIRECTORIES` 时 CMake
  仍按普通 `-I` 生成；清掉 `INTERFACE_INCLUDE_DIRECTORIES` 又直接找不到头文件）⇒
  最终在**唯一 include 它的翻译单元** `src/registration_backend.cpp` 里用
  `#pragma GCC diagnostic push/ignored … pop` 局部屏蔽（只覆盖 include 那几行，本包自己的代码照旧受管）。
* `add_subdirectory` 的**另一个副作用已修掉**（重要，见 §5）：上游 `CMakeLists.txt` 会把
  `CMAKE_BUILD_TYPE` FORCE 成 `Release`，从而把**本包所有目标**变成 `-O3 -DNDEBUG`。
  本包在 `add_subdirectory` 前后做了备份/还原，保证编译标志与改动前**逐字一致**。

### 2.2 复现 vendored 源码（与我们提交的内容逐字节一致）

```bash
cd /home/weicheng/HzMi_rmsimulation/third_party
git clone https://github.com/koide3/small_gicp.git small_gicp
cd small_gicp && git checkout 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e   # = tag v1.0.1
rm -rf .git                                                             # vendored：随本仓提交
# 许可证文件：third_party/small_gicp/LICENSE（MIT，Copyright (c) 2024 Kenji Koide）
# 署名登记：THIRD_PARTY_NOTICES.md §二
```

---

## 3. 指标语义（**本次最容易被搞错的地方**）

| 话题 | 含义 | 实现位置 |
|---|---|---|
| `~/fitness_score` | **不变**：内点平均平方距离（m²）。"内点" = 用最终变换把 source 投到 target 后，最近邻**平方**距离 ≤ `max_correspondence_distance²`（1.5²=2.25 m²）的 source 点；无内点 ⇒ `nan` | `pcl`：`gicp.getFitnessScore(r²)`（PCL 1.12.1 `impl/registration.hpp:140-165`）；`small_gicp`：`registration_backend.cpp` 的 `pclEquivalentFitnessScore()` **逐点等价复算** |
| `~/converged` | 后端自己的收敛标志（`pcl`: `hasConverged()`；`small_gicp`: `RegistrationResult::converged`）；节点再与"评分达标"相与后发布（**这段逻辑没改**） | `gicp_registration.cpp` 的 `publishHealth()` |
| `~/small_gicp_error`（**新增，仅 `backend=small_gicp`**） | `small_gicp::RegistrationResult::error` **原值** = Σ 0.5·rᵀ·(C_target + T·C_source·Tᵀ)⁻¹·r（**Mahalanobis 加权、无量纲**，不是 m²） | `registration_backend.cpp` |

为什么不直接把 `error` 当 fitness：它的量纲/量级与 m² 完全不同（实测同一帧：`error ≈ 51.0` vs
`fitness = 0.00123 m²`）。若拿它去比 `max_fitness_score: 0.3`，**每一帧都会被判为不达标**（静默失效）——
这正是必须**另开话题**而不是改写 `~/fitness_score` 的原因。

两个后端指标仍有**细微**差异（如实记录）：PCL 的 KdTreeFLANN 用 `float` 存平方距离、
最终变换是 `Matrix4f`；small_gicp 全程 `double` ⇒ 恰好落在门限边界上的点归属可能不同（影响 <1e-6 m²）。
实测：`pcl` 0.00123~0.00124 m²，`small_gicp` 0.00122~0.00123 m²。

---

## 4. 实测 A/B（同一台机器、同一批帧、同一份参数）

### 4.0 测量口径（可复现）

* **机器**：i7-14700HX（28 线程），Ubuntu 22.04 + ROS 2 Humble，PCL 1.12.1；测量时 `load ~3`。
* **隔离**：单节点（**不启** Gazebo / nav2 / LIO），`ROS_DOMAIN_ID=88`（非默认域），
  `ROS_LOG_DIR` 落在工作区内（`.tmp_cache/gicp_ab/…`）。
* **数据**：`src/rm_nav_bringup/PCD/RMUL2026.pcd`（原始 53164 点 → 去 NaN 53164 → 体素 0.10 m 后
  **target 12450 点**）；合成实时点云 = 从同一 PCD 等间隔抽 **4000 点**、**只含 x/y/z**（无 intensity）、
  注入 `(+0.15, −0.15, 0) m` 偏移 → 节点体素 0.05 m 后 **source 3701 点**（台架 3710 点）。
  真值 `T_target←source = Trans(−0.15, +0.15, 0)`。
* **参数**：`max_correspondence_distance 1.5`、`maximum_iterations 16`、`transformation_epsilon 5e-4`、
  `rotation_epsilon 2e-3`、`correspondence_randomness 20`、`maximum_optimizer_iterations 20`、`max_fitness_score 0.3`。
* **两个台子**（走**同一份代码**：`RegistrationBackend` + `voxelDownsample`）：
  1. `install/gicp_registration/lib/gicp_registration/gicp_backend_bench`（离线，逐帧打印，可给 median/p95/max）；
  2. 真节点 + 合成点云/静态 TF 探针（`.tmp_cache/gicp_ab/probe2.py` + `ab_run.sh`，可看 `/tf` 频率与健康话题）。

### 4.1 A 表：**仓库当前默认构建**（`CMAKE_BUILD_TYPE` 为空 ⇒ 无 `-O`）

`gicp_backend_bench`（12 帧稳态，第 0 帧单列，因为它含 PCL 的目标协方差预计算）：

| 后端 | 首帧 align | 稳态 median | mean | p95 | **max** | fitness（median） |
|---|---|---|---|---|---|---|
| `pcl`（单线程） | 1511.8 ms | 386.7 ms | 408.3 | 632.7 | **632.7 ms** | 0.00122 m² |
| `small_gicp` threads=**1** | — | 559.5 ms | 553.0 | 640.0 | 640.0 ms | 0.00122 m² |
| `small_gicp` threads=2 | — | 412.0 ms | 372.6 | 442.3 | 442.3 ms | 0.00122 m² |
| `small_gicp` threads=**4**（默认） | 237.0 ms | **205.5 ms** | 218.7 | 260.1 | **260.1 ms** | 0.00122 m² |
| `small_gicp` threads=8 | — | 205.0 ms | 229.5 | 318.1 | 318.1 ms | 0.00122 m² |

> `small_gicp` 的**目标侧一次性预处理**（转点云 + 建 KdTree + 估协方差，target 12450 点）在构造期做一次：
> **268.1 ms**（-O3 时 6.8 ms）。PCL 的目标协方差是**惰性**的，算在**第一帧**里（上表 1511.8 ms）。
> **单线程的 small_gicp 比 PCL 还慢**（559 vs 387 ms）——它在这个构建配置下的收益**全部来自多线程**。

真节点（14 s，`[status]` 行采样）：

| 后端 | align ms（实测区间，median） | `ros2 topic hz /tf` | `~/pose` 帧率 | `~/fitness_score` | `~/converged` true |
|---|---|---|---|---|---|
| `pcl` | 404~482（≈450） | **50.00 Hz**（min 0.019 / max 0.021 s） | 24 条 /14 s ≈ 1.7 Hz | 0.00124 m² | 24/24 = 100% |
| `small_gicp` t=4 | 226~278（≈236） | **50.00 Hz**（min 0.019 / max 0.021 s） | 57 条 /14 s ≈ 4.1 Hz | 0.00123 m² | 49/53、51/55、55/57 = **92~96%**（见 §6.2） |

### 4.2 B 表：**同一个包、只加 `-DCMAKE_BUILD_TYPE=Release`**（`-O3`）

**A/B 的两个后端都重新编**，且额外用**未改动的 HEAD 代码**做对照（证明 30× 不是本次重构带来的）：

| 配置 | 稳态 median | max | fitness | 说明 |
|---|---|---|---|---|
| `pcl`，**未改动的 HEAD**，无 `-O` | 395.4 ms（节点 [status]） | — | 0.00123 m² | 改动前的"已验证口径" |
| **`pcl`，未改动的 HEAD，`-O3`** | **12.7~18.4 ms**（节点 [status]） | — | 0.00123 m² | **≈30×**，数值一字不差 |
| `pcl`，本次改动，`-O3` | 12.5 ms（台架 19 帧） | 14.9 ms | 0.00122 m² | 与未改动代码一致 ⇒ **重构没改变 PCL 路径的成本** |
| `small_gicp` t=4，本次改动，`-O3` | **2.9 ms**（台架 19 帧） | 3.0 ms | 0.00122 m² | 比 `-O3` 的 PCL 再快 **4.3×**；首帧 3.6 ms、目标预处理 6.8 ms |

`/tf` 频率在两种构建里都是 **50.00 Hz**（TF 由定时器驱动，与 align 耗时无关——这正是 2026-10-05
"MT executor + 独立回调组"那次修复的效果）。

### 4.3 坏初值容忍度（**要求项**：+0.5 m / +10°）

台架（确定性；初值 = 真值 ∘ 偏差）：

| 后端 | 初值偏差 | converged | 采纳 | fitness | **最终位姿误差** | align |
|---|---|---|---|---|---|---|
| `pcl` | +0.5 m / +10° | true | 是 | 0.001235 m² | **5 mm / 0.06°** | 2670 ms（无 -O；-O3 时 32 ms） |
| `small_gicp` t=4 | +0.5 m / +10° | true | 是 | 0.001221 m² | **3 mm / 0.03°** | 502 ms（无 -O；-O3 时 4.5 ms） |
| `pcl` | +1.0 m / +20° | true | 是 | 0.001251 m² | 8 mm / 0.08° | 4754 ms（-O3 时 43 ms） |
| `small_gicp` t=4 | +1.0 m / +20° | true | 是 | 0.001221 m² | 3 mm / 0.04° | 660 ms（-O3 时 5.2 ms） |

真节点（`-p initial_pose:=[0.35, 0.15, 0, 0, 0, 0.174533]` = 真值 +0.5 m/+10°，**无** `/initialpose` 注入）：

* `pcl`：第 1 条 [status] 就已经在真值上（`x=-0.149 y=0.151 yaw=-0.01°`），23/23 帧采纳，0.00123 m²；
* `small_gicp`：同样第 1 条 [status] 已在真值（3/3 采纳），51/55 帧采纳，0.00123 m²，align 185~270 ms。

**结论：两个后端都能从 +0.5 m/+10°（乃至 +1.0 m/+20°）的坏初值里恢复；small_gicp 的最终误差还略小
（3 mm vs 5 mm，量级差异，别过度解读）。没有出现"新后端更不耐坏初值"的证据。**

### 4.4 契约回归（两个后端都跑，全部通过）

| 检查项 | `pcl` | `small_gicp` | 证据 |
|---|---|---|---|
| **没有初值就不发 TF**（有点云、无 `odom→base` TF、无 `/initialpose`） | ✅ 0 条 `map→odom`、0 条 `~/pose`、`~/converged` 全 false、日志每 2 s 一条 WARN | ✅ 同 | `run_noinit_*/probe.log`（116 帧点云） |
| **只发 `map→odom`**（`/tf` 上非 `map→odom` 的变换数） | ✅ 0 条 | ✅ 0 条 | 探针统计（`/tf` 上其余变换计数 = 0） |
| `tf_lookahead_sec` 盖戳 / `~/pose` 与 TF 同戳 | ✅ 未改动（同一段代码） | ✅ 同 | 代码路径未变 |
| SIGINT 干净退出（退出码 0） | ✅ 所有 run | ✅ 所有 run | `[ab] node SIGINT exit=0` |
| `~/fitness_score` = 内点平均平方距离（m²） | ✅ 0.00123~0.00124 | ✅ 0.00122~0.00123（**另算**，非原生 error） | §3 |
| 构建 `0 error / 0 warning`（`-Wall -Wextra -Wpedantic`） | ✅ | ✅ | 全量重编日志 `grep -ciE "warning|error"` = 0 |
| `yaml.safe_load` 配置 | ✅ 25 个参数键解析通过 | ✅ 同 | `python3 -c "import yaml; …"` |

---

## 5. ★ 顺带发现：这个包一直在**无优化**编译（比换后端更值钱）

* `colcon`/`ament_cmake` **不会**替你设置 `CMAKE_BUILD_TYPE`（实测 `CMakeCache.txt: CMAKE_BUILD_TYPE:STRING=`），
  `flags.make` 里就是 `-fPIC -Wall -Wextra -Wpedantic -std=gnu++17` —— **没有 `-O2/-O3`**。
* 用**未改动的 HEAD 代码**、只改构建参数：

  ```bash
  colcon build --symlink-install --packages-select gicp_registration \
    --cmake-args -DCMAKE_BUILD_TYPE=Release      # 只影响这一个包
  ```

  ⇒ PCL GICP 单帧 **395 ms → 13 ms（≈30×）**，`fitness`（0.00123 m²）与 `map→odom` **完全一致**。
* 为什么本次**没有**顺手打开它：① 它改变的是"已验证路径"的构建配置（还附带 `-DNDEBUG` 关掉
  assert），属于**另一个决定**；② 一旦 `add_subdirectory(small_gicp)` 带进上游那句
  `set(CMAKE_BUILD_TYPE "Release" CACHE … FORCE)`，本包会**静默**变成 `-O3 -DNDEBUG`
  （而且 `CMAKE_BUILD_TYPE` 一旦被写进 cache，后续增量构建也一直生效）——这正是本次改动里
  **必须显式还原**它的原因（`CMakeLists.txt` 里 `_gicp_registration_build_type_backup` 那几行）。
* **建议**：把"是否给 gicp_registration 开 `-O3`"作为一个独立议题评估（收益 30× 且零代码改动、
  零新依赖；风险是数值路径/断言行为变化，需要重跑一次 `nav_smoke_regression.py`）。
  在开之前，本文 A 表就是"当前口径"的基准。

---

## 6. 风险与行为差异（**上生产前请逐条看**）

### 6.1 算法/数值差异（无法消除，只能记录）

| 差异 | 说明 | 影响 |
|---|---|---|
| 优化器不同 | PCL GICP = **BFGS**（`maximum_optimizer_iterations` 内层）；small_gicp 默认 **LM**（Levenberg–Marquardt） | 收敛轨迹/迭代次数不逐帧一致；坏初值实测都能恢复（§4.3） |
| 协方差正则化 | PCL 对平面做特征值钳制（`cov = R·diag(1e-3,1,1)·Rᵀ`，`impl/gicp.hpp` 里 `computeCovariances`）；small_gicp 用原始协方差直接求逆 | 退化场景（纯平面、极稀疏）下两者行为可能分叉；**本仓库资产实测无差异**（fitness 同一量级、同一真值） |
| 距离/变换精度 | PCL 侧 `Matrix4f` + `float` 距离；small_gicp 全程 `double` | 门限边界点的归属可能不同（<1e-6 m²）；`~/fitness_score` 末位有 1e-5 级差异 |
| 下采样 | **同一个 `voxelDownsample()`（pcl::VoxelGrid）**，逐点一致 | 无差异（这是 A/B 可比的前提） |

### 6.2 `~/converged` 会**抖动**（small_gicp 特有，实测 4~8%）

* 现象：`small_gicp` 在**已经处于极小点**的跟踪帧上，偶尔返回 `converged=false`
  ——LM 的 lambda 试探无法再降低误差 ⇒ `optimizer.hpp:170` 的 `if (!success) break;`，
  而**同一帧的 `~/fitness_score` 仍是最优的 0.00123 m²**。
* 实测：`~/converged` true 49/53、51/55、55/57（**92~96%**）；`pcl` 后端 100%（PCL 的
  `hasConverged()` 两个循环出口都置 true ⇒ 实际上"永远 true"，只有异常才 false）。
* 后果：这些帧**被既有接受判据（`converged && score≤max`）丢掉** ⇒ `map→odom` 沿用上一次估计
  （**位姿不受影响**，只是更新频率略降；`[status]` 行的"采纳 N/M"能直接看到）。
* 若要"不抖"：把 `registration_backend.cpp` 里的 `Registration` 第 3~5 个模板参数写成
  `small_gicp::NullFactor, small_gicp::DistanceRejector, small_gicp::GaussNewtonOptimizer`
  并删掉 `max_inner_iterations` 那一行（**实测 GN 变体：`~/converged` 100% true、align 200~278 ms、
  fitness 0.00123、坏初值同样恢复**）。我们**故意保留上游默认 LM**（重定位场景下阻尼更稳健）。

### 6.3 线程超订（与我们的 MT executor 叠加）

* 节点是 `MultiThreadedExecutor` + 两个 `MutuallyExclusive` 回调组：**同一时刻只有一个 align 在跑**
  （`align_cb_group_`），所以 `small_gicp_num_threads` 只在这**一个** align 内部并行；
  但 TF 定时器组（50 Hz）与它并行，且整栈里还有 Gazebo / nav2 / LIO。
* 线程扫描（A 表）：1→559 ms、2→412 ms、4→**205 ms**、8→205 ms（无进一步收益）。
  ⇒ 默认 **4** 是"整栈同机"时的稳妥值；COD 用 8 是因为它的机器/负载不同。
* `num_threads <= 0` 会被判非法并退回 4（OpenMP `num_threads(0)` 是未定义行为）。

### 6.4 启动期多花的时间

`backend=small_gicp` 时，构造节点会多花 **268 ms**（无 `-O`；`-O3` 时 6.8 ms）做目标预处理
（转点云 + 建 KdTree + 估协方差）。**它换来的是"首帧不再有 1.5 s 的 PCL 目标协方差开销"**，
启动总时间基本不变。banner 会打印这个耗时。

### 6.5 顺带观测到的既有问题：`/initialpose` 可能被"点云 + align"饿死

（**不是本次改动引入的**，但团队应该知道；这两条订阅按既有设计**故意同组**，见 `gicp_registration.cpp` 顶部注释）

* 空闲时（无点云）：`ros2 topic pub --once /initialpose` ⇒ 节点 **~15 ms** 就收到并应用 ✅
* align 组被点云占满时（PCL 400~490 ms/帧 vs 10 Hz 点云）：
  * 探针（RELIABLE，5 条）的 `/initialpose` 直到**点云停下来的那一刻**才被处理（延迟 ≈ **13.2 s**）；
  * 另一次用 `ros2 topic pub --once` 在饱和期发的样本，**14 s 内完全没有被处理**。
* 未做根因定位（MT executor 的 MutuallyExclusive 组调度饥饿？RELIABLE 样本在组忙碌期间的排队/丢弃？
  两者可能都有）。**本次改动没有触碰回调组结构**；`small_gicp` 把单帧从 ~450 ms 降到 ~236 ms
  ⇒ 组占用率减半，风险随之下降，但没有消失。
* 若团队要根治：把 `/initialpose` 拆到**第三个**回调组 —— 但那会破坏"人工初值与在飞的 align 串行"
  这条不变量（旧注释里明确写了为什么故意同组），属于**另一个决定**。
  > **2026-10-05 后续更新（已落地）**：确实拆到了第三个回调组，但**同时**把"写 `map→odom`"这一步
  > 搬进了**点云回调的帧首**（`consumePendingInitialPose()`，仍在 align 组内）⇒ 交接（handoff）与
  > 应用（apply）分离，串行性不变量**更强**（写入与 align 是同一回调序列）。
  > 实测（无优化基线、PCL 400~490 ms/帧 @10 Hz）：点击生效延迟 **22.7 s → 0.24 s**；
  > 回到 Release(-O3) 口径是 **0.000 s → 0.121 s**（旧代码的饥饿被 `-O3` 掩盖）。
  > 设计、实测表、契约回归与被否决的方案见 **`docs/gicp_initialpose_latency.md`**。

---

## 7. 复现命令（把本文所有数字重新量一遍）

```bash
cd /home/weicheng/HzMi_rmsimulation
source /opt/ros/humble/setup.bash

# ① 构建（本包，0 error / 0 warning）
colcon build --symlink-install --packages-select gicp_registration

# ② 离线台架：两个后端、同一批帧、逐帧 align ms + fitness + 坏初值
./install/gicp_registration/lib/gicp_registration/gicp_backend_bench \
  --pcd src/rm_nav_bringup/PCD/RMUL2026.pcd --backend both --frames 30 --threads 4
#   线程扫描：把 --threads 换成 1/2/4/8

# ③ 真节点（单节点、非默认域、工作区内日志）：
#    .tmp_cache/gicp_ab/ab_run.sh <TAG> <时长s> <pcl|small_gicp> <none|tfon|good|bad> [--ros-args 覆盖]
bash .tmp_cache/gicp_ab/ab_run.sh ab_pcl     14 pcl        tfon  '-p initial_pose:=[-0.15,0.15,0,0,0,0]'
bash .tmp_cache/gicp_ab/ab_run.sh ab_small   14 small_gicp tfon  '-p initial_pose:=[-0.15,0.15,0,0,0,0]'
bash .tmp_cache/gicp_ab/ab_run.sh ab_bad_pcl 14 pcl        tfon  '-p initial_pose:=[0.35,0.15,0,0,0,0.174533]'
#   none = 不发静态 TF、不发 /initialpose ⇒ 验"无初值不发 TF"

# ④ 用户最常用的三条（整栈起来之后）
ros2 topic hz /tf                                   # 期望 ~50 Hz（两个后端都一样）
ros2 topic echo /gicp_registration/fitness_score    # m²；当前口径 0.0012 量级
ros2 topic echo /gicp_registration/converged        # small_gicp 会有少量 false（§6.2）
ros2 topic echo /gicp_registration/small_gicp_error # 仅 backend=small_gicp；原生 error，**不是 m²**
python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
```

---

## 8. 回退

| 想要的效果 | 操作 |
|---|---|
| 立刻回到已验证的 PCL 路径 | `backend: "pcl"`（**默认值就是它**），或运行时 `-p backend:=pcl` |
| 整包回退到本次改动之前 | `git revert <本次 commit>`（`third_party/small_gicp/` 会一并移除；`docs/` 与 `THIRD_PARTY_NOTICES.md` 的登记行也一起回退） |
| 只想去掉 vendored 依赖但保留 `backend` 参数 | 不可行：`small_gicp` 路径需要 `third_party/small_gicp`（`find_package` 需要系统安装；`add_subdirectory` 需要源码） |

---

## 9. 本次**没能**验证的事（诚实清单）

1. **整栈（Gazebo + nav2 + LIO + gicp）下的端到端表现**：本文所有数字都是**单节点**测量；
   `backend: small_gicp` 还没跑过 `nav_smoke_regression.py`、没跑过比赛场次。**首跑必须看**：
   `~/converged` 的 false 比例（§6.2）、`[status]` 的采纳率、`ros2 topic hz /tf`、
   `Control loop missed its desired rate` 是否消失。
2. **`-O3` 那条 30× 的结论只验证了"单帧 align + fitness 不变"**；没有在 `-O3` 下跑整栈回归，
   也没评估 `-DNDEBUG`（关 assert）的副作用。
3. **`~/converged` 抖动的根因**只定位到"LM 在极小点提前 break"这一层（源码 + 实测一致），
   没有逐帧插桩确认每一帧的退出分支。
4. **§6.5 的 `/initialpose` 饥饿**只做了现象观测（2 次实验），没有根因定位；
   空闲场景（~15 ms）与饱和场景（≥13 s / 未处理）差异很大，**真人 RViz 点击是否受影响未验证**
   （RViz 的 publisher 长期存在，与 `ros2 topic pub --once`/探针的行为可能不同）。
5. **没有跑 `ament_lint_auto` 的 cpplint/copyright**（沿用既有配置跳过）；没有跑单测（本包无单测）。
6. **没有在真车/真实 Livox 点云上验证**（只有 PCD 合成的 x/y/z 点云；真点云的噪声/密度分布不同）。
7. **`small_gicp_num_threads` 只在 28 线程的 i7-14700HX 上扫过**；换机器（尤其小核多的 CPU）需要重扫。
8. **上游 master 的新提交没有评估**（我们刻意 pin 在 v1.0.1；升级需重新过一遍本文 A/B）。
