# VENDORING —— 本目录里的 Patchwork++ 是怎么进来的

> 本目录 = **url-kaist/patchwork-plusplus 的忠实子集**（未改一个字节的上游源码）+ 一个我们写的构建入口。
> 权威登记表在仓库根 `THIRD_PARTY_NOTICES.md`；本文件只记"怎么复现这个目录"。

## 1. 上游与 pin

| 项 | 值 |
|---|---|
| 仓库 | <https://github.com/url-kaist/patchwork-plusplus> |
| 论文 | Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud, IROS 2022 |
| **pinned commit** | **`3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`** |
| 该 commit 的 tag | **`v1.4.1`**（= 抓取时的 `master` HEAD，提交日期 2026-05-23，提交信息 `chore(release): v1.4.1 (#101)`） |
| 许可证 | **BSD-2-Clause**（`LICENSE`，`Copyright (c) 2024, Urban Robotics Lab. @ KAIST`）；GitHub REST API `license.spdx_id = "BSD-2-Clause"` |
| 上游版本号 | `cpp/CMakeLists.txt` 里 `project(patchworkpp VERSION 1.4.1)` |

复现（逐文件抓取，不用 `git clone` 以免带进 `.git`）：

```bash
COMMIT=3e6903a1d5537a4cc2ace897b0bbb98a92d6014c
BASE=https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/$COMMIT
mkdir -p cpp/common/include/patchwork cpp/common/src cpp/patchworkpp/include/patchwork cpp/patchworkpp/src
for f in LICENSE README.md CHANGELOG.md; do curl -sSf "$BASE/$f" -o "$f"; done
for f in cpp/README.md \
         cpp/common/include/patchwork/plane_fit.h cpp/common/include/patchwork/types.h \
         cpp/common/src/plane_fit.cpp \
         cpp/patchworkpp/include/patchwork/patchworkpp.h cpp/patchworkpp/src/patchworkpp.cpp ; do
  mkdir -p "$(dirname "$f")" && curl -sSf "$BASE/$f" -o "$f"
done
```

## 2. 拿了什么 / 没拿什么

**拿了**（= 编 `patchworkpp_ground_seg` 这个静态库所需的**全部**文件）：

| 路径 | 是什么 |
|---|---|
| `cpp/common/include/patchwork/types.h` | `PointXYZ`（带 `idx` 原始索引！）、`PCAFeature`、`PatchStatus` |
| `cpp/common/include/patchwork/plane_fit.h` | `estimate_plane()`、`xy2theta()`、`xy2radius()`、`point_z_cmp()` |
| `cpp/common/src/plane_fit.cpp` | 上面的实现（SVD 平面拟合） |
| `cpp/patchworkpp/include/patchwork/patchworkpp.h` | `patchwork::Params` + `PatchWorkpp` 类 |
| `cpp/patchworkpp/src/patchworkpp.cpp` | 主算法：RNR / CZM / R-VPF / R-GPF / GLE / TGR |

**没拿**（并说明原因）：

| 上游路径 | 为什么不要 |
|---|---|
| `cpp/CMakeLists.txt`、`cpp/cmake/eigen.cmake` | 构建期 `FetchContent` 从 gitlab 下载 Eigen 3.4.0（见 `CMakeLists.txt` 顶部注释①）；本仓库用系统 `libeigen3-dev` |
| `cpp/patchwork/**`（经典 Patchwork） | 我们只用 Patchwork++；它需要 TBB，白编一个库 |
| `python/**`、`cpp/patchworkpp/examples/**` | 需要 Open3D / pybind11，仿真机上不装 |
| **`ros/**`（上游自带 ROS 2 wrapper）**明确不 vendor** | `ros/package.xml` 写 `<license>GPL-3.0</license>`，而同目录 `ros/LICENSE` 是 **MIT**（`Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss`，明显是从 KISS-ICP 抄来的模板）—— **两处声明互相矛盾**。我们无法判定哪个有效 ⇒ 按"许可证不清就不 vendor"处理，改为**自己写一个薄 ROS 2 节点**（`src/` 下），只依赖 BSD-2-Clause 的 `cpp/**`。 |
| `data/*.bin`（KITTI 样例） | 与我们的 A/B 无关 |

## 3. 我们改了什么

**上游源码：一个字节都没改。** 三个动作都属于"目录整理"，不触碰 `.h/.cpp`：

1. 删掉上游的 4 个 `CMakeLists.txt`（`cpp/`、`cpp/common/`、`cpp/patchworkpp/`、`cpp/patchwork/`），换成同目录我们写的 `CMakeLists.txt`；
2. 删掉 `cpp/patchwork/` 空目录（未取源码）；
3. 增加本文件（`VENDORING.md`）。

核对方法（应为空输出）：与上游同 commit 逐文件 diff ⇒ 见 `docs/ground_segmentation_slots.md` §1.3 的核对命令。

## 4. 告警

上游代码在本仓库的 `-Wall -Wextra -Wpedantic` 口径下会报少量告警（未使用参数、有符号/无符号比较等）。
我们**不改上游源码**，改为在 `CMakeLists.txt` 里对本目标 `PRIVATE` 关掉这几类：
`-Wno-unused-parameter -Wno-sign-compare -Wno-unused-variable`。
调用方（本包节点/台架）自己的 TU **仍然**吃全量告警。

## 5. 已知的上游行为（用之前要知道的）

- `patchworkpp.h` 在**全局作用域**写了 `using namespace std;`（上游如此）⇒ include 它的 TU 会被污染。
  本包把"include patchworkpp.h"限制在**唯一一个**翻译单元（`src/patchwork_ground_segmentation_node.cpp`）
  以及台架里，避免扩散。
- `reflected_noise_removal()`（RNR）**需要第 4 列 intensity**：`if (cloud_in.cols() < 4) { cout << "RNR requires intensity information !"; return; }`。
  我们的仿真点云是 `pcl::PointXYZ`（无 intensity）⇒ **`enable_RNR` 默认关**，否则每帧往 stdout 打一行字且什么都不做。
- `estimateGround()` 会给**每个输入点**一个归属：落在 `min_range < r <= max_range` 的进 CZM 分区，
  其余直接进 `cloud_nonground_`（第 626-628 行）⇒ 与 linefit"越界点算障碍"的语义一致。
  唯一边角：`pc2czm()` 第 600 行 `if (z == std::numeric_limits<float>::min()) continue;` 会把 z 恰好等于
  `FLT_MIN`（≈1.18e-38）的点**两个列表都不放**（RNR 用它当"已删除"标记）。本包节点对这类点按**障碍**兜底。
- `update_elevation_thr()` 会**在线改写 `params_.sensor_height`**（`sensor_height = -update_mean`，第 360 行）
  ⇒ 配置里的 `sensor_height` 只是**前几帧的初值/种子**，之后由观测自适应。这既是它的卖点也是它的风险
  （见 `docs/ground_segmentation_slots.md` §6）。
