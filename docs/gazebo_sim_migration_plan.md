# Gazebo Classic 11 → 新 Gazebo（`gz sim` / `ros_gz`）迁移可行性调研与迁移清单

> **文档性质：可行性 spike（探针）+ 迁移清单**。本文**不改任何功能性代码/配置/launch/world**。
> 所有实测都在 `TMPDIR` 等价的未跟踪临时目录 `.tmp_gzsim/` 里完成（`Xvfb` 复用 `.tmp_gzgui/tools/usr/bin/Xvfb`）。
>
> **一句话结论**：**这条迁移值得试，而且没有想象中贵 —— 但它换不来"MID-360 的 CSV 逐射线图案"。**
> 新 Gazebo 的 `gpu_ray` 是**光栅化**实现，30000 条射线的内存代价从 Classic 的 **2.74 GiB 塌到 ≈6 MiB**
> （本机实测，2880 → 30000 条只涨 5.8 MiB）、内存/射线的量级差 **≈435×**，而且**无 GPU 机器上照样跑得动
> （含 GUI）**；代价是**传感器图案的自由度从"任意 3 万条射线"退化成“只有水平×垂直规则网格”**，
> 要保住 MID-360 就必须**自己写一个 gz system**。
>
> **裁决速查**
>
> | 问题 | 结论 | 置信度 |
> |---|---|---|
> | Humble 官方配哪个新 Gazebo | **Fortress（`ignition-gazebo6` / gz-sim6）** | 高（官方配对表 + 本机已装） |
> | 本环境能不能装 | **能，而且已经装好了**（`ros-humble-ros-gz` 0.244.26 + `libignition-gazebo6` 6.18.0） | 高（apt 实测） |
> | 能不能用 Garden/Harmonic | 能拿到包，但 **`apt install ros-humble-ros-gzharmonic` 会 `Remv` 掉 Gazebo Classic + `ros-humble-gazebo-ros-pkgs` + `ros-humble-ros-gz*`** ⇒ **会摧毁现有基准台** | 高（`apt-get -s` 实测） |
> | 新 Gazebo 支持任意逐射线图案（MID-360 CSV）吗 | **不支持**。SDF `<lidar>` 规范全文只有规则网格；连 Jetty 全新的 CPU `cpu_lidar` 也只生成规则网格 | **很高**（规范全文 + 上游源码逐行） |
> | 无 GPU 支持软件渲染吗 | **支持，且本机实测可用**（Mesa llvmpipe / Xvfb，GL 4.5 core） | 高（本机实测） |
> | 没有 DISPLAY 呢 | **`ign gazebo -s --headless-rendering` 无 DISPLAY 也能出完整传感器数据** | 高（本机实测，点云逐字节同尺寸） |
> | 有"完全不碰渲染器"的纯 CPU 射线吗 | **只有 Jetty（gz-sim10）有**（`gz-sim-cpu-lidar` + `gz-physics` 射线），而 **Jetty 没有 Humble 的 ros_gz 包** | 高（上游源码 + apt 实测） |
> | GUI 能留吗 | **能**。GUI 是独立进程 + 独立 gz-rendering 实例，`gui:=` 槽位语义可以完整保留 | 高（实测窗口 + 上游架构） |
> | 30k 射线代价是"消除"还是"搬家" | **真的消除**（不是搬家）：从 **每射线一个 ODE 几何体**（O(rays) 内存/CPU）变成 **一张光栅化纹理**（O(1) 内存） | 中高（本机标定 + 上游实现） |

---

## 0. 状态、范围与证据分级

| 项 | 说明 |
|---|---|
| 本文测的机器 | 28 核 / 31 GiB / **无 `/dev/dri`、无加载的 NVIDIA 驱动**（`nvidia-smi` 失败）⇒ **本身就是一台"没有 GPU 的电脑"**，正好用来回答用户提的第三个问题 |
| 已执行 | apt 可用性、`apt-get download`、**Fortress PoC（带 GUI / 无头 / 无 DISPLAY 三种）**、30000 射线标定、源码级规范核对 |
| 未执行 | 真机世界（RMUC2026/RMUL2026 STL）迁移、`ros_gz_bridge` 端到端、nav2 整栈 A/B、Harmonic 实测（见 §8 诚实清单） |
| 证据分级 | ①**本机实测**（可复现，附录 A 给命令）②**本机安装物**（apt 包 / 头文件 / 已装规范文件）③**上游源码/规范**（带 URL）④**官方文档**（带 URL） |

---

## 1. 版本配对：ROS 2 Humble 到底配哪个新 Gazebo

### 1.1 官方配对矩阵（一手引用）

官方页 <https://gazebosim.org/docs/latest/ros_installation/> 给出的兼容表（原文照抄关键行）：

|  | **GZ Fortress (LTS)** | **GZ Harmonic (LTS)** | Gz Ionic | Gz Jetty (LTS) |
|---|---|---|---|---|
| ROS 2 Rolling | ❌ | ⚡ | ⚡ | ✅ |
| ROS 2 Kilted | ❌ | ⚡ | ✅ | ❌ |
| ROS 2 Jazzy (LTS) | ❌ | ✅ | ❌ | ❌ |
| **ROS 2 Humble (LTS)** | **✅** | **⚡** | ❌ | ❌ |

- `✅` = 推荐组合；`⚡` = 可以，但"**use with caution**，需要一些功夫"；`❌` = 不可能。
- **所以：Humble 的官方答案就是 Fortress。** 同页明确写："Usually, the latest version of Gazebo is available at
  the beginning of each ROS release cycle (for example **Gazebo Fortress for ROS 2 Humble**)."
- ⚠️ **Garden 对 Humble 是 `❌`**（不是 ⚡）。虽然 `packages.osrfoundation.org` 上确实存在
  `ros-humble-ros-gzgarden`，但官方配对表把它排除在 Humble 之外 —— 这条容易踩。
- Humble 想用 Harmonic 的话，官方给的唯一路径是**非官方二进制包**
  `ros-humble-ros-gzharmonic`（来自 `packages.osrfoundation.org`），并且原文警告：
  "These packages **conflict with `ros-humble-ros-gz*` packages**"。

### 1.2 各版本 EOL（一手引用）

官方 EOL 表 <https://gazebosim.org/docs/all/releases/>：

| 名称 | 发布 | **EOL** | 备注 |
|---|---|---|---|
| Jetty | 2025-09 | **2031-05** | LTS |
| Ionic | 2024-09 | 2026-12 | |
| Harmonic | 2023-09 | **2029-05** | LTS |
| Garden | 2022-09 | **2024-11** | **已 EOL** |
| **Fortress** | **2021-09** | **2027-05** | **LTS** |
| Edifice / Dome / Citadel | 2021-03 / 2020-09 / 2019-12 | 均已 EOL | |

ROS 2 侧：REP-2000 <https://www.ros.org/reps/rep-2000.html> 写 **"Humble Hawksbill (May 2022 – May 2027)"**。

> **一个很好用的巧合**：**Fortress EOL = 2027-05，Humble EOL = 2027-05**。
> 也就是说，**如果目标只是"跟着 Humble 生命周期走完"，Fortress 的生命周期刚好够，不需要为了续命而跳到 Harmonic。**

### 1.3 `ros_gz` 版本号 ↔ Gazebo 版本

**本机实测**（`apt-cache policy`）：已装 `ros-humble-ros-gz-sim 0.244.26-1jammy.20260907.235058`，
其依赖里明确写 `libignition-gazebo6-dev`。

```
$ apt-cache depends ros-humble-ros-gz-sim | grep ignition
  依赖: libignition-msgs8
  依赖: libignition-transport11
  依赖: libignition-gazebo6-dev        <-- Fortress = ign-gazebo 6
  依赖: libignition-math6-dev
```

⇒ **`ros-humble-ros-gz` `0.244.x` ≡ Fortress（gz-sim6）**。这是硬证据，不是推测。

各发行版对应的 `ros_gz` 包名与底层库名（**本机 apt 实测 + 上游命名**）：

| Gazebo | gz-sim | Humble 上的 ROS 包名 | 底层 apt 库 | 命令 |
|---|---|---|---|---|
| **Fortress** | gz-sim6 | `ros-humble-ros-gz`（+ `-sim` / `-bridge` / `-interfaces` / `-image`） | `libignition-gazebo6-dev` | **`ign gazebo`** |
| Garden | gz-sim7 | `ros-humble-ros-gzgarden*` | `libgz-sim7-dev` | `gz sim` |
| Harmonic | gz-sim8 | `ros-humble-ros-gzharmonic*` | `libgz-sim8-dev` | `gz sim` |
| Ionic | gz-sim9 | **无 Humble 包** | — | `gz sim` |
| Jetty | gz-sim10 | **无 Humble 包** | — | `gz sim` |

本机实测的候选版本：
`ros-humble-ros-gz → 0.244.26`（ROS 源）、`ros-humble-ros-gzgarden → 0.244.11`（OSRF 源）、
`ros-humble-ros-gzharmonic → 0.244.12`（OSRF 源）；
`libgz-sim7-dev → 7.9.0`、`libgz-sim8-dev → 8.15.0` 在 `packages.osrfoundation.org/gazebo/ubuntu-stable` 里都存在。

### 1.4 Ignition → Gazebo 改名与命令差异（会真实咬人的一条）

上游在 **Garden（gz-sim7）** 做了 Ignition→Gazebo 改名（见上方命名表；Fortress 仍用 `ignition-*` 库名与 `ign gazebo`；
`libignition-*` 与 `gz-*` 在 Fortress 的头文件里是**同一份**，本机 `/usr/include/ignition/gazebo6/gz/sim/System.hh`
就是 `#include <gz/sim/System.hh>` 的 shim —— 实测）。

**本机实测的一个坑**：

```
$ gz sim --versions
Invalid arguments
$ ign gazebo --versions
6.18.0
```

⇒ **Fortress 上不存在 `gz sim` 这个子命令**（`ign-tools 1.5.0` 只提供 `ign gazebo`）。
**所有从网上抄来的 `gz sim …` 命令在 Fortress 上会直接报错**，
而 `gz topic` / `ign topic` 两种写法都能用。迁移脚本里必须处理这个分支。

### 1.5 结论

- **首选 Fortress**，理由三条：①官方 ✅ 配对；②**本机已经装好**（零安装成本）；
  ③EOL 与 Humble 同步（2027-05），生命周期不是短板。
- **Harmonic 是"以后再说"的选项**，因为它在 Humble 上会与 Classic 打架（§2.3 实测）。
- **Jetty 不用考虑**：它是唯一有纯 CPU 射线的版本，但 **Humble 没有它的 ros_gz 包**（§2.1 实测）。

---

## 2. 环境实测：这台机器上到底有什么

### 2.1 apt 可用性（一手实测）

```bash
$ apt-cache policy ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
ros-humble-ros-gz:          候选 0.244.26-1jammy.20260908.015605   已安装：(无)   [mirrors.aliyun.com/ros2]
ros-humble-ros-gz-sim:      候选 0.244.26-1jammy.20260907.235058   已安装：同版本 [aliyun / dpkg]
ros-humble-ros-gz-bridge:   候选 0.244.26-1jammy.20260907.225444   已安装：同版本 [aliyun / dpkg]
```

**结论：`ros-humble-ros-gz-sim` 与 `ros-humble-ros-gz-bridge` 本来就已经装在这台机器上了**
（只有 meta 包 `ros-humble-ros-gz` 没装）。已装的核心库：

| 包 | 版本 |
|---|---|
| `libignition-gazebo6` / `-dev` / `-plugins` | **6.18.0** |
| `libignition-gui6` / `-dev` | 6.9.0 |
| `libignition-rendering6` / `-core-dev` / **`-ogre1`** / **`-ogre2`** | 6.6.4 |
| `libignition-msgs8` / `libignition-transport11` / `libignition-common4` / `libignition-fuel-tools7` | 8.7.1 / 11.4.2 / 4.9.0 / 7.3.1 |
| `sdformat12`（含 `sdformat12-sdf`） | 12.9.0 |
| Gazebo **Classic** `gazebo` / `libgazebo11` | **11.10.2** ← 与 Fortress **并存** |

已装的 Fortress system 插件：**147 个 `.so`**（`dpkg -L libignition-gazebo6-plugins`）。

> **注意一个易混点**：机器上同时有 Gazebo **Classic 11** 与 **Fortress**。
> 命令层面 `gazebo`/`gzserver`/`gzclient` = Classic，`ign gazebo` = Fortress。二者**不冲突**。

**没有的**（实测）：

```bash
$ apt-cache policy ros-humble-ros-gzjetty ros-humble-ros-gzionic
ros-humble-ros-gzjetty:  <不存在>
ros-humble-ros-gzionic:  <不存在>
```

⇒ **Jetty / Ionic 在 Humble 上没有官方 ros_gz 二进制包**。这条直接掐死了"用 Jetty 的纯 CPU 射线"这条路
（除非自己编 `ros_gz` + 自己编 gz-sim10，代价远超本 spike 范围）。

### 2.2 `apt-get download`（无 root）实测

**成功**。5 个 Fortress ros_gz deb 全部下到 `.tmp_gzsim/dl_fortress/`，**共 4.0 MiB**：

| 文件 | 大小 |
|---|---|
| `ros-humble-ros-gz-bridge_0.244.26-…deb` | 3,566,464 B |
| `ros-humble-ros-gz-sim_0.244.26-…deb` | 276,070 B |
| `ros-humble-ros-gz-interfaces_0.244.26-…deb` | 273,824 B |
| `ros-humble-ros-gz-image_0.244.26-…deb` | 55,936 B |
| `ros-humble-ros-gz_0.244.26-…deb` | 7,078 B |

⇒ **无 root 的 `apt-get download` 在本机完全可用**，所以"拿包来研究/验证"这件事没有障碍。
（`apt-get download` 只是下载 `.deb`，不会安装，不碰系统。**未做任何 `dpkg -i`**。）

### 2.3 ⚠️ Harmonic 路径的致命冲突（**这条决定"能不能保住现有基准台"**）

`apt-get -s install ros-humble-ros-gzharmonic`（**只模拟，不执行**）：

```
127 packages, 35.1 MiB 待下载
Remv gazebo [11.10.2+dfsg-1]
Remv ros-humble-gazebo-dev / -plugins / -ros / -ros-pkgs [3.9.0-1jammy]
Remv ros-humble-gazebo-ros2-control
Remv ros-humble-velodyne-simulator / -gazebo-plugins
Remv ros-humble-desktop-full
Remv ros-humble-ros-gz-sim / -bridge / -image / -interfaces / -sim-demos        <-- 连 Fortress 一起删
Remv ros-humble-ros-ign-image
```

⇒ **`apt install ros-humble-ros-gzharmonic` 会卸载 Gazebo Classic 11 与整个 `gazebo_ros_pkgs`。**
也就是说，**它和"不能丢掉现有可工作的 Classic 基准台"是直接冲突的**。

> **含义**：Harmonic（以及任何非 Fortress 的版本）**必须走"另一台机器 / 容器 / docker"**，
> 不能和现有 bench 装在同一套 `/opt/ros/humble` 里。
> 这也反过来把 **Fortress 推成唯一"零风险并行共存"的选项** —— 因为它**已经装好了**，
> 且没有对 Classic 做任何改动。

### 2.4 GPU / 显示现实（**本机就是一台无 GPU 机器**）

```bash
$ ls /dev/dri
ls: 无法访问 '/dev/dri': 没有那个文件或目录        # 没有 DRM 设备节点
$ nvidia-smi -L
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver.
$ lspci | grep -i vga
01:00.0 VGA compatible controller: NVIDIA Corporation Device 2860 (rev a1)   # 有卡但驱动没起
```

**但软件渲染栈是完整的**（在 Xvfb 上实测 `glxinfo -B`）：

```
OpenGL vendor string:   Mesa
OpenGL renderer string: llvmpipe (LLVM 15.0.7, 256 bits)
Accelerated:            no
OpenGL core profile version string: 4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4
OpenGL ES profile version string:   OpenGL ES 3.2 Mesa 23.2.1
```

⇒ **无 GPU 但仍然拿到 OpenGL 4.5 core 上下文**，靠的是 Mesa `llvmpipe`。
本机 `/usr/lib/x86_64-linux-gnu/dri/` 里 `swrast_dri.so` / `kms_swrast_dri.so` 都在，
`libEGL_mesa.so.0`、`libgbm.so.1` 也都在。

---

## 3. 传感器分析（**最关键的一节**）

### 3.1 新 Gazebo 的传感器：不是"一个系统一个传感器"，而是**"一个 Sensors 系统管全部"**

**上游事实（源码目录，实测 404/200 探测 + GitHub 树负载）**：

| 版本 | `src/systems/` 里的传感器系统 | 说明 |
|---|---|---|
| **Fortress (gz-sim6)** | `sensors-system`（唯一），`imu-system`、`air-pressure-system`、`altimeter-system`、`magnetometer-system`、`logical-camera-system`、`thermal-system`、`contact-system` | **没有任何 lidar 专用系统** |
| Garden (gz-sim7) | `sensors`（只有 `Sensors.cc`） | 同上 |
| Harmonic (gz-sim8) | `sensors`（只有 `Sensors.cc`） | 同上 |
| Ionic (gz-sim9) | `sensors`（只有 `Sensors.cc`） | 同上 |
| **Jetty (gz-sim10)** | `sensors` **+ `src/systems/cpu_lidar`** ← **唯一新增** | 见 §3.7 |

**本机 Fortress 实测（一手，比上网查更硬）**：

```bash
$ dpkg -L libignition-gazebo6-plugins | grep '\.so$' | grep -iE 'lidar'
libVisualizeLidar.so                     # 只是 GUI 可视化插件，不是传感器系统
$ ls /usr/include/ignition/gazebo6/gz/sim/ | grep -i lidar
(空)                                      # 头文件里也没有 lidar-system
```

⇒ **Fortress 里 LiDAR 只有一条路：`<sensor type="gpu_ray">`，由 `Sensors` 系统驱动，底层是 gz-rendering。**

各 SDF `type=` 与系统插件的对应（Fortress 实测 + 上游）：

| SDF `type` | 系统插件 | 是否需渲染器 |
|---|---|---|
| `camera` / `depth_camera` / `rgbd_camera` / `segmentation_camera` / `thermal_camera` / `wide_angle_camera` | `ignition-gazebo-sensors-system` | **是** |
| `gpu_ray` / `gpu_lidar` | `ignition-gazebo-sensors-system` | **是**（光栅化） |
| `ray` / `lidar`（CPU） | **Fortress 无实现** ← 见 §3.5 | —（本就不存在） |
| `imu` | `ignition-gazebo-imu-system` | 否 |
| `air_pressure` / `magnetometer` / `altimeter` / `contact` / `force_torque` / `navsat` / `logical_camera` | 各自的 `*-system` | 否 |

### 3.2 **SDF `<lidar>` 规范全文**（这就是"图案自由度"的全部）

**本机安装物**（一手、权威、无需联网）：`/usr/share/sdformat12/1.9/lidar.sdf`（Fortress 用的 sdformat12 规范，
**全文 78 行**）。它的**全部**元素如下（这是逐行枚举，不是摘抄）:

```xml
<element name="lidar">
  <element name="scan" required="1">
    <element name="horizontal" required="1">
      <element name="samples"    type="unsigned int" default="640" required="1"/>
      <element name="resolution" type="double"       default="1"   required="1"/>
      <element name="min_angle"  type="double"       default="0"   required="1"/>
      <element name="max_angle"  type="double"       default="0"   required="1"/>
    </element>
    <element name="vertical" required="0">
      <element name="samples"    type="unsigned int" default="1" required="1"/>
      <element name="resolution" type="double"       default="1" required="0"/>
      <element name="min_angle"  type="double"       default="0" required="1"/>
      <element name="max_angle"  type="double"       default="0" required="1"/>
    </element>
  </element>
  <element name="range" required="1">
    <element name="min" type="double" default="0" required="1"/>
    <element name="max" type="double" default="0" required="1"/>
    <element name="resolution" type="double" default="0" required="0"/>
  </element>
  <element name="noise" required="0">
    <element name="type"   type="string" default="gaussian" required="1"/>
    <element name="mean"   type="double" default="0.0" required="0"/>
    <element name="stddev" type="double" default="0.0" required="0"/>
  </element>
  <element name="visibility_mask" type="unsigned int" default="4294967295" required="0"/>
</element>
```

**整个规范里没有任何一个元素能表达"第 i 条射线的方向"**。水平/竖直各自只有一个
`min_angle`/`max_angle`/`samples` 的三元组 ⇒ **只能是等间距规则网格（矩形 FOV 的均匀采样）**。

补充实测：在整套 SDF 规范里检索"图案类"元素 ——

```bash
$ grep -rlniE '<element name="(pattern|directions|rays|ray_directions|csv|points)"' \
      /usr/share/sdformat12/1.9/ /usr/share/sdformat9/1.7/
(无输出)
```

⇒ **不存在** `pattern` / `directions` / `ray_directions` / `csv` 之类的元素。
（对比：Classic 11 用的 `sdformat9/1.7/lidar.sdf` 是同样的结构 —— 也就是说，
**我们现在的 MID-360 图案从来不是 SDF 给的，而是我们自己的插件绕过了 SDF 采样**。）

### 3.3 证据 A：`gz::rendering::GpuRays` 的 API **只有规则网格**

**本机安装头文件**（一手）：`/usr/include/ignition/rendering6/gz/rendering/GpuRays.hh`。
它的**全部** setter（逐行枚举）：

```cpp
SetClamp(bool)                    Clamp()
SetIsHorizontal(bool)             IsHorizontal()
SetRayCountRatio(double)          RayCountRatio()
SetAngleMin(double)               AngleMin()            // 水平起始角
SetAngleMax(double)               AngleMax()            // 水平结束角
SetRayCount(int)                  RayCount()            // 水平采样数
SetVerticalRayCount(int)          VerticalRayCount()    // 竖直采样数
SetVerticalAngleMin(double)       VerticalAngleMin()
SetVerticalAngleMax(double)       VerticalAngleMax()
SetHorizontalResolution(double)   HorizontalResolution()
SetVerticalResolution(double)     VerticalResolution()
ConnectNewGpuRaysFrame(...)       Data() / Copy(float*)
```

⇒ **`GpuRays` 没有任何"给我一组任意射线方向"的入口。** 它只能表达"水平 N 条 × 竖直 M 条、等间距"。
（上游同一份：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/GpuRays.hh>）

`RayQuery` 倒是**逐条任意方向**（本机 `/usr/include/ignition/rendering6/gz/rendering/RayQuery.hh`）：

```cpp
class RayQuery : public virtual Object {
  virtual void SetOrigin(const math::Vector3d &) = 0;
  virtual void SetDirection(const math::Vector3d &) = 0;
  virtual void SetFromCamera(const CameraPtr &, const math::Vector2d &) = 0;
  virtual RayQueryResult ClosestPoint() = 0;      // {distance, point, objectId}
};
```

**但它是从"场景"里造出来的**：`Scene.hh` 里 `virtual RayQueryPtr CreateRayQuery() = 0;`
—— 即 `RayQuery` 依附于一个 `Scene`，而 `Scene` 来自某个渲染引擎 ⇒ **它不是"免渲染器"的离线射线。
（这是 §4.4 的关键分界线。）**

### 3.4 证据 B：**连 Jetty 全新的 CPU `cpu_lidar` 也只生成规则网格**

Jetty（gz-sim10）新增了 `src/systems/cpu_lidar/`，插件名 `gz-sim-cpu-lidar-system`，
类 `gz::sim::systems::CpuLidar`，**由 `gz-physics` 做射线求交、完全不走渲染器**。
但它生成射线的代码是（`gz-sensors10/src/CpuLidarSensor.cc`，上游源码）：

```cpp
// Load(): 预计算 unit vector —— 双层 for，纯规则网格
for (unsigned int v = 0; v < vSamples; ++v) {
  const double inclination = vMin + v * vStep;
  for (unsigned int h = 0; h < hSamples; ++h) {
    const double azimuth = hMin + h * hStep;
    this->dataPtr->unitVectors.emplace_back(
        std::cos(inclination) * std::cos(azimuth),
        std::cos(inclination) * std::sin(azimuth),
        std::sin(inclination));
  }
}

// GenerateRays(): 只会把上面那批 unit vector 拉成 (rMin, rMax) 线段
std::vector<std::pair<Vector3d, Vector3d>> CpuLidarSensor::GenerateRays() const {
  ...
  for (const auto &unitVec : this->dataPtr->unitVectors)
    rays.emplace_back(unitVec * rMin, unitVec * rMax);
  return rays;
}
```

而且：

```cpp
// gz-sensors10/include/gz/sensors/CpuLidarSensor.hh
public: std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
  GenerateRays() const;                    // <-- 注意：没有 virtual
```

⇒ **双重否定**：①射线集来自规则网格；②`GenerateRays()` **不是 `virtual`**，
`unitVectors` 是私有 `dataPtr` 成员 ⇒ **子类也无法覆写图案**。
（上游：<https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/src/CpuLidarSensor.cc>、
<https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems/cpu_lidar>）

### 3.5 证据 C：Fortress **根本没有 CPU `ray` 传感器**（运行期一手证据）

PoC 世界（`.tmp_gzsim/poc_fortress.sdf`）里同时放了一个 `type="gpu_ray"` 和一个 `type="ray"`。
启动日志立刻给了答案：

```
[Wrn] [SdfEntityCreator.cc:917] Sensor type LIDAR not supported yet.
                                Try using a GPU LIDAR instead.
```

并且订阅 `/poc/cpu_ray` **超时**（`exit=124`，20 s 一条都没有），而 `/poc/gpu_lidar` 正常出 360×8 的
`LaserScan` 与 `PointCloudPacked`。

> **这一条是本调研里最有实践价值的发现**：Fortress（**Humble 的官方配对**）**没有纯 CPU 的 LiDAR 路径**。
> 所以在 Fortress 上，**LiDAR ⇔ 渲染器 ⇔ GL 上下文**，没有第二条路。
> 想"完全不碰渲染器做 LiDAR"必须上 Jetty，而 Jetty 没有 Humble 包（§2.1）。

### 3.6 社区侧交叉印证（上游 issue）

| issue | 标题 | 说明 |
|---|---|---|
| [gz-sim#1958](https://github.com/gazebosim/gz-sim/issues/1958) | **Custom lidar sensor** | 用户问怎么自定义 LiDAR |
| [gz-sim#2645](https://github.com/gazebosim/gz-sim/issues/2645) | **Build my own Lidar Sensor with custom scanning shape** | 用户问**自定义扫描形状** —— 正是我们的 MID-360 问题 |

（这两个 issue 的正文在本沙箱里没能完整抓到——GitHub 只回了导航框架、正文被截断；
**它们的存在与标题可见，但结论未逐句核对**，已列入 §8 诚实清单。）

### 3.7 **如果要做：自定义 gz system 必须实现什么**（精确接口）

#### 3.7.1 系统插件接口（**本机 Fortress 头文件逐行**）

`/usr/include/ignition/gazebo6/gz/sim/System.hh` 全文（去注释）：

```cpp
namespace ignition { namespace gazebo {
class System { public: System() = default; public: virtual ~System() = default; };

class ISystemConfigure {
  public: virtual void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr) = 0;
};

class ISystemConfigureParameters {
  public: virtual void ConfigureParameters(
      ignition::transport::parameters::ParametersRegistry &_registry,
      EntityComponentManager &_ecm) = 0;
};

class ISystemPreUpdate {
  public: virtual void PreUpdate(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm) = 0;
};

class ISystemUpdate {
  public: virtual void Update(const UpdateInfo &_info,
                              EntityComponentManager &_ecm) = 0;
};

class ISystemPostUpdate {
  public: virtual void PostUpdate(const UpdateInfo &_info,
                                  const EntityComponentManager &_ecm) = 0;  // 注意 const
};
}}
```

**要点**：
- `PreUpdate` 拿**可变** `EntityComponentManager&`（可以 `CreateComponent`）；
  `PostUpdate` 拿 **`const EntityComponentManager&`**（只能读）。
  ⇒ **"写传感器数据"必须发生在 `PostUpdate`**（读 `RaycastData` 结果 → 发布）。
- `UpdateInfo` 成员（本机 `Types.hh`）：`simTime`、`realTime`、`dt`、`paused`、`iterations`、`residualStepSize`。
- 插件用宏自注册：

```cpp
GZ_ADD_PLUGIN(CpuLidar, System, CpuLidar::ISystemPreUpdate, CpuLidar::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(CpuLidar, "gz::sim::systems::CpuLidar")
```

（Fortress 里对应 `IGNITION_ADD_PLUGIN`；Jetty 的 `cpu_lidar` 实测就是上面这两行。）

#### 3.7.2 传感器侧的必需件

| 需要什么 | Fortress 对应物 | 本机路径 / 引用 |
|---|---|---|
| 传感器工厂 | `sensors::SensorFactory` | `gz/sensors/SensorFactory.hh`（gz-sensors6） |
| 传感器抽象 | `gz::sim::Sensor`（`Entity()` / `Name()` / `Pose()` / `Topic()` / `Parent()`） | `/usr/include/ignition/gazebo6/gz/sim/Sensor.hh` |
| 组件 | `components::Sensor`、`components::SensorTopic`、`components::ParentEntity`、`components::WorldPose` | `/usr/include/ignition/gazebo6/gz/sim/components*` |
| 拿渲染场景 | `gz/sim/rendering/RenderUtil.hh`、`SceneManager.hh`、`Events.hh` | `/usr/include/ignition/gazebo6/gz/sim/rendering/` |
| 发射 gz-transport 话题 | `gz::transport::Node::Advertise<gz::msgs::LaserScan>` / `PointCloudPacked` | gz-transport11（本机已装） |

#### 3.7.3 **Jetty 给出的"正确答案"就是一份可直接照抄的参考实现**

Jetty 的 `CpuLidar` 把"CPU LiDAR"拆成了**三个组件 + 一个物理后端**，这个架构才是最省事的落点：

```
CpuLidar::PreUpdate(ecm)                       // 系统侧
  ├─ 对新传感器: CreateComponent(SensorTopic)
  ├─ rays = sensor->GenerateRays()             // ① 生成射线集
  ├─ CreateComponent(entity, components::RaycastData{rays})
  └─ CreateComponent(entity, components::NeedsRaycast(true))

（gz-physics 的 raycast 系统读 RaycastData/NeedsRaycast，做求交，写回 results）
                                                 // ② 求交：跟渲染器完全无关

CpuLidar::PostUpdate(ecm)
  ├─ Each<CpuLidar, RaycastData, WorldPose>
  │    → sensor->SetPose(worldPose)
  │    → sensor->SetRaycastResults(results)      // ③ 填 ranges/intensities
  └─ sensor 自己发布 LaserScan / PointCloudPacked
```

**所以要支持 MID-360 的 CSV 图案，最小改动点是 ①**：把 `unitVectors` 的来源从"规则网格"
换成"读 `mid360.csv`"。**但**因为 `GenerateRays()` 非虚 + `unitVectors` 私有，
**不能靠子类**，只能在下面三条里选一条：

| 路线 | 做法 | 工作量 | 风险 |
|---|---|---|---|
| **R1（务实）** | **自己写一个 gz system**，不复用 `gz-sensors::CpuLidarSensor`：自己读 CSV → 自己持有一组方向 → 自己在 `PostUpdate` 里用 `RayQuery` 逐条打，或自己接进 `RaycastData` 那条链 | **L** | 需自己实现发布（`LaserScan`/`PointCloudPacked`）、时间戳、`frame_id`、`<topic>` 语义 |
| R2（上游补丁） | **fork `gz-sensors`**，给 `CpuLidarSensor` 加一个"从 CSV/显式方向表载入 `unitVectors`"的口子（把 `GenerateRays()` 改虚或加 `SetRayDirections()`） | **M**（改动小但**要维护 fork**） | 版本升级要重放补丁；且必须让上游接受才可持续 |
| R3（渲染器侧） | fork `gz-rendering` 的 OGRE2 `GpuRays`，把它那张"射线方向打包纹理"换成 CSV 烘焙的任意方向表（上游文件 `ogre2/src/Ogre2GpuRays.cc` + `media/` 下的 GLSL） | **L+** | 最深的改动，但**唯一能保住"光栅化 O(1) 内存"优势**的路线；每次渲染库升级都要重做 |

> **本 spike 不做 R1/R2/R3 中的任何一个** —— 只把"必须做什么"钉死，供后续单独立项。

**还有一个"小便宜"路线（R0，值得单独提）**：
如果 MID-360 的**非重复性**不是刚性需求，可以只用 `gpu_ray` 的规则网格去逼近：
`<horizontal><samples>100</samples></horizontal>` × `<vertical><samples>300</samples></vertical>`
= 30000 条，**零代码改动**（本机 1000×30 已实测跑通，RTF≈1.0）。代价是**图案变了** —— 属于
**感知契约变更**，必须另开任务做量化验收（同 `docs/gazebo_gui_troubleshooting.md` §6.4.1 末尾对
"降 `<samples>`" 的口径：**用户已否决降采样，必须量化验收**）。R0 只作为"想要先把链路跑通"的临时手段。

### 3.8 传感器结论

1. **新 Gazebo 的官方传感器体系不支持任意逐射线图案 —— 一个都不支持（含 Jetty）。**
   依据是 SDF `<lidar>` 规范全文（4 个角度参数 + 2 个采样数）+ `GpuRays` API 全集 + Jetty `cpu_lidar` 源码。
2. 我们现在的 MID-360 图案**之所以存在，正是因为 Classic 上我们用了自己的插件绕过 SDF**。
   迁移到新 Gazebo **等于把这个"绕过去"的能力交回去** ⇒ **必须自己再写一个 system 才能拿回来**。
3. **Fortress 连 CPU `ray` 都没有** ⇒ LiDAR 与渲染器强绑定（这是 §4 的支点）。

---

## 4. 无 GPU 分析

### 4.1 三条路径的定义与实测归属

| 路径 | 机制 | 需要 DISPLAY？ | 需要 GPU？ | 本机实测 |
|---|---|---|---|---|
| **A. X + llvmpipe** | Xvfb 虚拟屏 + Mesa 软件光栅化（GLX） | 需要（虚拟屏即可） | **不需要** | ✅ **已跑通（含 GUI 窗口）** |
| **B. `--headless-rendering`** | EGL 无窗口上下文（软件 EGL） | **不需要** | **不需要** | ✅ **已跑通（无 DISPLAY）** |
| **C. 纯 CPU 射线（不走渲染器）** | `gz-physics` 射线求交 | 不需要 | 不需要 | ❌ **Humble 上拿不到**（只有 Jetty 有，无 Humble 包） |

`--headless-rendering` 的语义在**本机 `ign gazebo --help` 里有一手文字**：

```
--headless-rendering     Run rendering in headless mode
--render-engine [arg]    Ignition Rendering engine plugin to load for both the server and the GUI.
                         Gazebo will use OGRE2 by default. (ogre2)
--render-engine-gui      … for the GUI
--render-engine-server   … for the server
-s                       Run only the server (headless mode). This overrides -g, if it is also present.
-g                       Run only the GUI.
```

⇒ 渲染引擎可**分别**为 server / GUI 指定（`--render-engine-server` / `--render-engine-gui`），
这给了"GUI 用贵的引擎、传感器用便宜的引擎"这种调优空间。
本机可用的引擎插件只有 **`ogre`（OGRE 1.9）与 `ogre2`（OGRE 2.1+）**，
**没有 `optix`**（`ls /usr/lib/x86_64-linux-gnu/ | grep ignition-rendering6-optix` 为空）。

### 4.2 路径 A 实测：**无 GPU 机器上 GUI 能起来并渲染**

在 Xvfb `:99`（1280×800×24）+ `LIBGL_ALWAYS_SOFTWARE=1` 下：

```bash
$ DISPLAY=:99 xwininfo -root -children
     5 children:
     0x200012 "Gazebo": ("ign-gazebo-gui" "Gazebo GUI")  1000x845+0+0  +0+0
     0x200014 "Gazebo GUI": ()  1x1+0+0  +0+0
     0x400001 "ign gazebo gui": ("ign gazebo gui" "Ign gazebo gui")  10x10+10+10  +10+10
     0x200004 "Qt Selection Owner for ign-gazebo-gui": ()  3x3+0+0
```

GUI 插件全部加载成功（`3D View` / `MinimalScene` / `World control` / `World stats` / `Entity Tree` …），
日志里唯一的渲染相关警告是：

```
[Wrn] [Ogre2RenderTarget.cc:574] Anti-aliasing level of '8' is not supported;
                                 valid FSAA levels are: [ 0 4 ]. Setting to 0
```

⇒ **llvmpipe 的 FSAA 只有 0 和 4 两档**，`gui.config` 默认要 8 ⇒ 被静默降级。这是无 GPU 机器上的
**画质降级点**（不是功能故障）。

**一个有趣的重复发现**：本仓 `docs/gazebo_gui_troubleshooting.md` §6.3 早就量到 Classic 的
`gzclient` 在软件光栅化下要 **444–609 % CPU** —— 与本次 Fortress GUI 量到的 **1269 % CPU**
是同一个现象（**软件光栅化是多线程且极吃 CPU**），只是 Fortress 的 GUI 更重。

### 4.3 路径 B 实测：**连 DISPLAY 都不需要**

`ign gazebo -r -v 3 -s --headless-rendering <world>`，**显式 `unset DISPLAY`**：

- 启动日志**零错误**（无 `EGL`、`Failed`、`Unable` 字样）；
- `/world/poc/stats` 里 **`real_time_factor: 1`**；
- `/poc/gpu_lidar/points` **正常出数据**，且与"有 Xvfb"的那次**逐字节同尺寸**：

| 运行 | `points.txt` 字节数 | `width` × `height` |
|---|---|---|
| `-s` + Xvfb + llvmpipe | **3,624,688** | 1000 × 30 |
| `-s --headless-rendering` + **无 DISPLAY** | **3,624,688** | 1000 × 30 |

⇒ **"无 GPU + 无显示器"的服务器上，`--headless-rendering` 的软件 EGL 路径可用，
而且传感器数据是真的（不是空点云）。** 这是"无 GPU 也可以做 CI/无头回归"的直接依据。

### 4.4 路径 C：**纯 CPU 射线在新 Gazebo 里只有 Jetty 有，而 Humble 拿不到**

- 机制（Jetty）：`components::RaycastData` + `components::NeedsRaycast` 交给 **`gz-physics`**，
  由物理引擎对碰撞几何求交 —— **完全绕过 gz-rendering**。这确实是"不需要渲染器的 CPU 射线"。
- **但**：①它 **只有 gz-sim10（Jetty）** 有（§3.1 表）；②`apt-cache policy ros-humble-ros-gzjetty`
  → **不存在**（§2.1 实测）；③Jetty 也不支持任意图案（§3.4）。
- **`RayQuery` 不是路径 C**：它由 `Scene::CreateRayQuery()` 创建（本机 `Scene.hh:1124`），
  依附于渲染引擎 ⇒ 仍然要一个 GL（或软件 GL）上下文。
  ⇒ **"不碰渲染器的纯 CPU 射线"在 Humble 可行方案里没有。**

### 4.5 性能代价（无 GPU 的代价到底在哪）

| 项 | 有/无 GPU | 实测（本机，无 GPU） |
|---|---|---|
| `llvmpipe` 峰值 × 满 3D GUI | 无 GPU | **`ign gazebo gui` 1269 % CPU**（≈12.7 核） |
| 纯 server（无 GUI 窗口） | 无 GPU | **`ign gazebo server` 22.9–23.5 % CPU** |
| 传感器是否可用 | 无 GPU | **可用**（`gpu_ray` 出真数据） |
| 画质 | 无 GPU | FSAA 8→0（被降级） |
| 有无 GPU 的差距 | — | **本机无法对照量**（这台机器没 GPU）⇒ 已列入 §8 |

⇒ **无 GPU 的代价几乎全在 GUI 上**：把 GUI 关掉（`gui:=False`），server 侧只要 ~23 % 单核；
**留着 GUI 就得多付十几核**。但这正是用户要的"GUI 必须留着"的代价，且**它不阻塞传感器**。

### 4.6 无 GPU 结论

1. **支持软件渲染路径，而且是双保险**：路径 A（Xvfb + llvmpipe）与路径 B（`--headless-rendering` 软件 EGL）
   **本机都实测跑通**，且路径 B **连显示器都不要**。
2. **无 GPU 不等于无传感器**：`gpu_ray` 在无 GPU 机器上**出真数据**（点云字节数逐字节一致）。
3. **没有可用的"纯 CPU 射线"退路**：路径 C 只在 Jetty，Humble 拿不到；`RayQuery` 仍要 GL 上下文。
4. **代价**：GUI 在软件光栅化下极吃 CPU（十几核），且 FSAA 从 8 降到 0；
   server 侧很便宜（~23 % 单核）。
5. ⇒ **"没有 GPU 的电脑" 不是迁移的阻碍**，但**"想同时开大 GUI + 30000 射线" 在无 GPU 机器上会很吃力**（§5.5）。

---

## 5. PoC 实测（**已执行**）

### 5.1 环境与方法

| 项 | 值 |
|---|---|
| 引擎 | **Fortress `ign gazebo` v6.18.0**（`ros_gz_sim` 0.244.26 已装但 PoC 直接用 `ign gazebo` CLI） |
| 世界 | `.tmp_gzsim/poc_fortress.sdf`（1 个 ground plane + 2 面墙 + 1 个静止 sensor rig），以及生成器 `.tmp_gzsim/gen_world.py` 产出的 `w_2880.sdf` / `w_30k.sdf` |
| 传感器 | `gpu_ray`（水平×竖直规则网格）、`ray`（CPU，用于证伪）、`imu` |
| 虚拟屏 | `.tmp_gzgui/tools/usr/bin/Xvfb :99 -screen 0 1280x800x24`（**复用既有 scratch 里的 Xvfb**，未新装） |
| 环境 | `LIBGL_ALWAYS_SOFTWARE=1`；**`HOME` 重定向到 `.tmp_gzsim/home`**（见 §5.2 的坑） |
| 脚本 | `.tmp_gzsim/run_poc.sh`（GUI）、`.tmp_gzsim/run_scale.sh`（标定，三种模式） |

> **⚠️ 一个很有价值的踩坑（会原样搬到真迁移里）**：沙箱里 `$HOME` 只读，Fortress 的 GUI 会因为
> ```
> [Err] [Filesystem.cc:471] Failed to create directory [/home/weicheng/.ignition/gazebo]: Read-only file system
> [Err] [Gui.cc:86] Failed to create the default config folder […/gazebo/6]
> terminate called after throwing an instance of 'std::logic_error'
>   what():  basic_string::_M_construct null not valid
> ```
> **直接 abort**，而且 `ign gazebo` 会把 server 一起 `SIGKILL`。
> ⇒ **`$HOME/.ignition/` 必须可写**（或把 `HOME` 指到别处）。
> 这条对"CI/容器里跑 `ign gazebo`"是硬前提。

### 5.2 结果 1：**GUI 起得来，有真窗口**（回答"GUI 能不能留"）

见 §4.2：`xwininfo` 里有 `("ign-gazebo-gui" "Gazebo GUI") 1000x845`，GUI 插件全部加载，
`[Msg] Loaded plugin [3D View] / [MinimalScene] / [World control] / [World stats] …`。

**⇒ GUI 能留。** 而且在**没有 GPU** 的机器上也能留。

### 5.3 结果 2：传感器发布正常

`ign topic -l` 打出来的话题（节选）：

```
/clock
/gazebo/resource_paths
/gui/camera/pose
/poc/gpu_lidar                 <-- LaserScan
/poc/gpu_lidar/points          <-- PointCloudPacked（Fortress 同时发两种）
/poc/imu
/sensors/marker
/stats
/world/poc/{clock,pose/info,dynamic_pose/info,scene/info,state,stats}
```

`/poc/gpu_lidar` 内容（实测）：`angle_min: -3.14159`、`angle_max: 3.14159`、
`angle_step: 0.017501894150417828`、`count: 360`、`vertical_count: 8`、
`frame: "sensor_rig::link::gpu_lidar_3d"`。

`/poc/imu` 内容（实测）：`linear_acceleration { z: 9.8 }`、`orientation { w: 1 }`。
（`z: 9.8` 是重力项，符合预期。）

**⚠️ 话题命名与 Classic 不同**：Fortress 默认 `frame_id` 形如
`<model>::<link>::<sensor>`（`::` 分隔），而 Classic 是 `<link>_<sensor>` 之类。
**`ros_gz_bridge` 的配置和下游 `frame_id` 假设必须跟着改**（§6.7）。

### 5.4 结果 3：CPU `ray` 在 Fortress 不存在（证伪）

```
[Wrn] [SdfEntityCreator.cc:917] Sensor type LIDAR not supported yet. Try using a GPU LIDAR instead.
```
+ `/poc/cpu_ray` 订阅超时（`exit=124`）。

### 5.5 结果 4：**30000 射线标定（本调研最硬的一个数字）**

三组运行，`server-only`（`-s`，无 GUI 窗口）以隔离传感器成本，采样 `/world/poc/stats`：

| 运行 | 射线数 | 模式/环境 | **RTF** | **RSS** | **CPU** | 点云 |
|---|---|---|---|---|---|---|
| `rays2880_server` | 360×8 = **2,880** | `-s` + Xvfb + llvmpipe | **1.0001** | **505,492 KB ≈ 494 MiB** | 23.5 % | 360×8，`points.txt` 347,555 B |
| `rays30k_server` | 1000×30 = **30,000** | `-s` + Xvfb + llvmpipe | **0.9999** | **511,396 KB ≈ 499 MiB** | 23.5 % | 1000×30，`points.txt` 3,624,688 B |
| `rays30k_headless_nox` | 1000×30 = **30,000** | `-s --headless-rendering`，**无 DISPLAY** | **1** | — | — | 1000×30，3,624,688 B（同上） |

点云元数据证明射线真的打了（不是空跑）：

```
# rays30k_server/points.txt
width: 1000
height: 30
point_step: 32
row_step: 32000
```

**关键 delta**：射线数 **×10.4**（2,880 → 30,000），
- **RTF 从 1.0001 → 0.9999（无变化）**
- **RSS 只涨 5,904 KB ≈ 5.8 MiB**

⇒ **≈0.218 KiB / 条射线**。

**与 Classic 对照**（本仓 `docs/gazebo_gui_troubleshooting.md` §6.4.1 的实测：
2.74 GiB / 30000 条 = **≈94.8 KiB/条**，根因是 `livox_ode_multiray_shape.cpp` 的 `AddRay()`
**每条射线新建一个 `ODECollision` + 一个 `ODERayShape`**）：

| 实现 | 内存/条射线 | 机制 |
|---|---|---|
| Classic 自研插件（现状） | **≈94.8 KiB** | 每射线一个 ODE 几何体（`[heap]` 2.78 GiB） |
| **Fortress `gpu_ray`** | **≈0.218 KiB** | 光栅化：一张射线方向纹理 + 一张深度图 |
| **倍数** | **≈435×** | |

> **⚠️ 必须诚实标注的口径差异（否则这个倍数会被误读）**：
> ①Classic 的 94.8 KiB/条 是在**真机世界 + `robot11` + 一个完整栈**下量的；
> ②本次 0.218 KiB/条 是在**只有静态几何、无物理动力学**的微型世界里量的，
> 而且射线数是 **1000×30 规则网格**，**不是 MID-360 的 30000 条任意方向**；
> ③两者 RTF 不可直接比（Classic 0.32–0.46 含 SLAM/LIO 整栈；本次 ≈1.0 是世界几乎不干活）。
> **可以安全下的结论**是：**"每射线一个碰撞体"→"一张纹理"这个架构变化，
> 使射线数不再线性地放大内存与单线程世界更新成本**；**不能**下的结论是"整栈 RTF 会从 0.4 变成 1.0"。

### 5.6 结果 5：无 DISPLAY 的 `--headless-rendering`（见 §4.3）

无 DISPLAY、零报错、RTF=1、点云与有屏那次**逐字节同尺寸**。

### 5.7 PoC 数字总表

| 场景 | GUI 窗口 | RTF | RSS(gzserver) | RSS(gzclient) | CPU(gui/server) | 传感器 |
|---|---|---|---|---|---|---|
| GUI + 2880 射线（Xvfb+llvmpipe） | ✅ 1000×845 | 0.79–0.85 | 523 MiB | **582 MiB** | **1269 %** / 22.9 % | ✅ |
| `-s` + 2880 射线 | — | 1.0001 | 494 MiB | — | — / 23.5 % | ✅ |
| `-s` + 30000 射线 | — | 0.9999 | 499 MiB | — | — / 23.5 % | ✅ |
| `-s --headless-rendering` + 30000 射线，无 DISPLAY | — | 1 | — | — | — | ✅ |

### 5.8 PoC 的边界（**明确没测什么**）

- 没测**真机世界**（RMUC2026 / RMUL2026 STL，几十 MB 级 mesh）—— 世界加载耗时与内存未量。
- 没测**机器人动力学**（本次全 `static`，物理成本≈0）。
- 没测 `ros_gz_bridge` 端到端（`/scan`、`/livox/lidar` 到 ROS 侧）。
- 没测 **MID-360 的 30000 条任意方向**在新 Gazebo 上到底什么代价（因为**做不到**，§3）。
- 没测 **Harmonic**（会摧毁 Classic，§2.3）。
- 没测 **有 GPU 机器的对照**（本机无 GPU）。
- 没测 `ogre`（OGRE 1.9）引擎与 `ogre2` 的差异。

---

## 6. 迁移清单（逐类，**路径/数量来自本仓实测**）

规模基线：本仓 tracked 文件 **1,646**；`src/rm_simulation` **143** 个文件；`src/rm_nav_bringup` **94** 个文件；
`tools/scripts` 非 `__pycache__` 文件 **125** 个。

### 6.0 总表

| # | 类别 | 涉及文件/条目 | 工作量 | 风险 | 能否渐进/加开关 |
|---|---|---|---|---|---|
| 1 | URDF/xacro `<gazebo>` 插件块 | **5** 个 xacro（3 个 sentry robot + 1 个 waking robot + 1 个 mid360） | **M** | 中 | ✅ 可（新 slot） |
| 2 | **自定义 Livox 插件** | **12** 文件（`livox_laser_simulation_RO2/`）+ **17 MB / 800,001 行** `mid360.csv` | **L** | **极高** | ⚠️ 难（核心能力） |
| 3 | 驱动插件 `libgazebo_ros_planar_move.so` | 出现在 **4** 个 xacro（共 6 处）+ 里程计/TF 契约 | **L** | 高 | ✅ 可（新 slot） |
| 4 | `.world` → SDF | **5** 个 `.world` + **18** 个 `model.sdf` | **M** | 中 | ✅ 可（新世界目录） |
| 5 | 障碍物 C++ 插件 | **14** 个 `obstacle*.cc`（Gazebo Classic C++ API） | **L** | 中高 | ✅ 可（只影响 `RMUL2024_world_dynamic_obstacles`） |
| 6 | launch 文件 | `bringup_sim.launch.py`（1300+ 行）、`rm_simulation.launch.py` | **M** | 中 | ✅ 可（`sim_backend:=` 新槽） |
| 7 | 话题/TF/`ros_gz_bridge` | 现存 **0** 个 bridge 配置；`base_link_fake` 出现在 **67** 处 | **M** | **高** | ✅ 可（bridge 独立进程） |
| 8 | 感知链（linefit→p2l→`/scan`） | 应**零改动**（话题契约由 bridge 维持） | **S** | 低 | ✅ |
| 9 | nav2 params | **18** 个 `nav2_params*.yaml`（多数只受话题名影响） | **S** | 低 | ✅ |
| 10 | 地图资产 | `map/` 242 MB + `PCD/` 118 MB，**格式与后端无关** | **—** | 无 | ✅ 完全不动 |
| 11 | `tools/scripts/**` 实验台 | **57 / 125** 个非 pyc 脚本含 `gazebo|gzserver|gzclient` | **M** | 中 | ✅ 可（双后端分支） |

### 6.1 URDF/xacro 的 `<gazebo>` 插件块

**实测清单**（含 `libgazebo_ros_*` 或 `<gazebo` 的**我们的**文件，共 5 个）：

| 文件 | 行数 | 关键内容 |
|---|---|---|
| `src/rm_nav_bringup/urdf/sentry_robot_sim.xacro` | 266 | `libgazebo_ros_planar_move.so`（`cmd_vel:=cmd_vel_chassis`、`odom:=odom_ground_truth`）、`libgazebo_ros_imu_sensor.so`（`~/out:=/livox/imu`） |
| `src/rm_nav_bringup/urdf/sentry_robot_hzmirm_sim.xacro` | 398 | 同上（"与默认模型逐字同款"） |
| `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` | **892** | **内联了一段 `<sensor type="ray" name="livox_frame">`**（`samples 100`×`360`、`<samples>30000</samples>`、`<topic>/livox/lidar</topic>`）+ `planar_move` + `imu_sensor` |
| `src/rm_simulation/hzmi_rm_simulation/urdf/simulation_waking_robot.xacro` | 259 | `planar_move`（`odom:=odom`）、`imu_sensor` |
| `src/rm_simulation/livox_laser_simulation_RO2/urdf/mid360.xacro` | 106 | **`<sensor type="ray">` + `libros2_livox.so` + CSV 路径 + `<samples>30000</samples>`** |

**本仓用到的 `gazebo_ros` 插件频次**（实测）：

| 插件 | 出现次数 | 新 Gazebo 对应 |
|---|---|---|
| `libgazebo_ros_imu_sensor.so` | **10** | `ignition-gazebo-imu-system` + `ros_gz_bridge` |
| `libgazebo_ros_ray_sensor.so` | **6** | `sensors-system` (`gpu_ray`) + bridge |
| `libgazebo_ros_planar_move.so` | **6** | **无直接对应** → `ignition-gazebo-diff-drive-system`（语义不同，见 §6.3） |
| `libgazebo_ros_joint_state_publisher.so` | **6** | `ignition-gazebo-joint-state-publisher-system` |
| `libgazebo_ros_diff_drive.so` | **6** | `ignition-gazebo-diff-drive-system` |
| `libgazebo_ros_camera.so` | 3 | `sensors-system` + bridge |
| `libgazebo_ros_p3d.so` | **12**（12 个 obstacle `model.sdf`） | **无直接对应** → 需替换为 `PosePublisher`/自研 |
| `libros2_livox.so`（我们的） | 2 | **无对应** → §6.2 |

**工作量 M / 风险中 / 可渐进**：新增一份 `*_gz_sim.xacro`（或 `sim_backend:=gz` 分支），
**不动现有 xacro** ⇒ Classic 基准台零影响。

### 6.2 ⚠️ 自定义 Livox 插件（**最高风险项**）

**现状（本仓实测）**：

| 文件 | 作用 |
|---|---|
| `src/rm_simulation/livox_laser_simulation_RO2/src/livox_ode_multiray_shape.cpp` + `include/ros2_livox/livox_ode_multiray_shape.h` | **ODE 多射线形状**：`AddRay()` **每条射线** `new` 一个 `ODECollision` + `ODERayShape` ← **2.74 GiB 的根因** |
| `src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp` + `include/ros2_livox/livox_points_plugin.h` | `gazebo::RayPlugin`：读 SDF、读 CSV、`CreateCollision("multiray")`、`reserve(samplesStep/downSample)`、逐条 `AddRay`、发布点云 |
| `include/ros2_livox/csv_reader.hpp` | 读 `mid360.csv` |
| `src/csv_main.cpp` | CSV 工具（离线） |
| `scan_mode/mid360.csv` | **800,001 行 / 17 MB** 的原始图案表 |
| `urdf/mid360.xacro` | `<samples>30000</samples>`、`<downsample>1</downsample>`、`<csv_file_name>…/mid360.csv</csv_file_name>`、`<topic>mid360</topic>` |
| `CMakeLists.txt` / `package.xml` | 依赖 `gazebo` + `gazebo_ros` |

**技术事实**：SDF `<lidar>` 规范不支持任意方向（§3.2），`GpuRays` 无相应 API（§3.3），
`RayQuery` 要 GL 上下文（§4.4），Jetty 的 `cpu_lidar` 也只用规则网格（§3.4）。
⇒ **必须自己写一个 gz system**；三条路线的取舍与接口细节见 **§3.7.3（R0/R1/R2/R3）**。

**工作量 L / 风险极高 / 渐进性差**：这是**唯一的"功能净损失"项**，
也是**决定"这次迁移是等价替换还是降级"的那一项**。

### 6.3 驱动插件（`libgazebo_ros_planar_move.so`）

现状：`<plugin name="mecanum_controller" filename="libgazebo_ros_planar_move.so">`，
`<remapping>cmd_vel:=cmd_vel_chassis</remapping>`、`<remapping>odom:=odom_ground_truth</remapping>`。

**为什么是 L（不是 S）**：
- `planar_move` 是**平面假运动学**（直接给速度、产生真值里程计），**不是**轮式动力学；
- 新 Gazebo 最接近的是 `ignition-gazebo-diff-drive-system`（**差速轮**）——
  对**麦轮底盘**语义不对（本仓是 `mecanum_controller`，另有 `mecanum_drive-system` 可用）；
- 而且 **`odom` 话题名 / `cmd_vel_chassis` 重映射 / TF 广播行为（`odom→base_link`）**
  都要在新系统里重新对齐 —— 本仓的 TF 契约是
  **`map→odom→base_link→base_link_fake`**，`base_link_fake` 在 **67** 处被引用（实测），
  **单发布者规则**（`docs/tf_interface_contract.md`）必须保持。

⇒ **工作量 L / 风险高 / 可渐进**（新 slot + 桥接 `cmd_vel_chassis`）。

### 6.4 `.world` → SDF

**实测：5 个 `.world` + 18 个 `model.sdf`**（全在 `src/rm_simulation/hzmi_rm_simulation/`）：

| 世界 | 体积 | 内容 |
|---|---|---|
| `world/RMUC2024_world/RMUC2024_world.world` | 8.0 K / 152 行 | 单一 STL：`model://RMUC2024_world/meshes/RMUC_2024.stl` |
| `world/RMUC2026_world/RMUC2026_world.world` | 12 K / 266 行 | 单一 STL：`model://RMUC2026_world/meshes/RMUC2026.stl` |
| `world/RMUL2024_world/RMUL2024_world.world` | 12 K / 309 行 | STL `RMUL_2024.stl` |
| `world/RMUL2024_world/RMUL2024_world_dynamic_obstacles.world` | 12 K / 359 行 | **`libobstacle{1..4}.so`** + `model://obstacles/obstacle*`（见 §6.5） |
| `world/RMUL2026_world/RMUL2026_world.world` | 12 K / 253 行 | STL `RMUL2026.stl` |

**好消息**：这些世界**几乎没有 Gazebo Classic 专有插件**（只有 dynamic_obstacles 那个有），
主要就是 `<include>` STL + `<physics>` + `<light>`。**主要工作量是 SDF 版本迁移与 `<plugin>` 头改写**
（`libgazebo_ros_*` → `ignition-gazebo-*`），**不是重写**。

**另一个坑（本仓已踩过，会原样复发）**：`model://` 的解析根在 Classic 靠
`gazebo_ros` 扫各包 `package.xml` 的 `<export><gazebo_ros gazebo_model_path=…/></export>` 拼出
`GAZEBO_MODEL_PATH`（见 `docs/gazebo_gui_troubleshooting.md` §3、`robot11_description/package.xml` 的长注释）。
**新 Gazebo 走 `GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH`，机制完全不同**
⇒ `model://RMUC2026_world/meshes/RMUC2026.stl` 这类 URI 在迁移后**默认解析不到**，必须重新配环境变量。
（本次 PoC 用的是 `ign gazebo` CLI，日志里确实出现了 `/gazebo/resource_paths` 话题 —— 说明资源路径是独立一套。）

**工作量 M / 风险中 / 可渐进**（新世界目录，不动旧 `.world`）。

### 6.5 障碍物 C++ 插件（**容易被漏掉的一块**）

**实测 14 个 `obstacle*.cc`**（`src/rm_simulation/hzmi_rm_simulation/meshes/obstacles/obstacle_plugin/`），
外加 12 个 `obstacles/obstacle*/model.sdf`（引用 `libgazebo_ros_p3d.so`）。

它们**直接 include Gazebo Classic 的 C++ API**（一手）：

```cpp
#include <ignition/math.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo { class Obstacles: public ModelPlugin { ... } }
// 用到的 Classic 类型：PoseKeyFrame ×14、PoseAnimationPtr ×14、PoseAnimation ×14 …（实测计数）
```

⇒ **这些必须重写为 `gz::sim::System`**（或干脆用新 Gazebo 的声明式动画/`TrajectoryFollower` 替代）。
**只影响 `RMUL2024_world_dynamic_obstacles` 这一个世界**，所以**可以完全延后**。

**工作量 L / 风险中高 / 可渐进**（世界级开关，与主链路解耦）。

### 6.6 launch 文件

| 文件 | 现状要点（实测） |
|---|---|
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | **1300+ 行**。槽位 `world:=`（默认 `RMUL2026`）、`robot:=`、`gui:=`（**默认 `True`**）、`gazebo_offline:=`（默认 `False`）、`use_sim_time`、`lio_rviz`、`nav_rviz` + `lio/nav/localization/ground/planner/nav` 等。**`gui:=` 的语义是"起 gzclient"，RViz 由 `nav_rviz`/`lio_rviz` 单独控**（注释里明确：`gui:=False` = 只起 gzserver） |
| `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py` | 世界/机器人启动、`_gazebo_model_path_setup`（OpaqueFunction，**必须在 include gzserver/gzclient 之前**跑） |
| `src/rm_nav_bringup/launch/cartographer_sim.launch.py` | 建图 slot |

上游入口：`/opt/ros/humble/share/gazebo_ros/launch/{gazebo,gzserver,gzclient}.launch.py`
→ 迁移后改用 `ros_gz_sim` 的 `gz_sim.launch.py` + `create`（`/opt/ros/humble/lib/ros_gz_sim/create` 已存在，实测）。

**关键映射**：

| Classic 概念 | 新 Gazebo |
|---|---|
| `gzserver.launch.py` | `ros_gz_sim` 的 `gz_sim.launch.py`（默认起 server+GUI；`gz_args` 控制） |
| `gzclient.launch.py` | 同一 `gz_sim` 进程树的 GUI（或 `gz sim -g` 独立进程） |
| `spawn_entity.py` | `ros_gz_sim` 的 `create` 可执行（实测存在） |
| `gui:=True/False` | **可原样保留**（对应"起不起 GUI"），语义不变 |
| `gazebo_offline:=` | 需重做（新 Gazebo 的 model 解析与 Fuel 机制不同） |
| `GAZEBO_MODEL_PATH` | **`GZ_SIM_RESOURCE_PATH` / `IGN_GAZEBO_RESOURCE_PATH`** |

**工作量 M / 风险中 / 可渐进**：建议加 **`sim_backend:=classic|gz`** 槽，
`classic` 为默认值 ⇒ **现有命令 `ros2 launch rm_nav_bringup bringup_sim.launch.py` 逐字不变。**

### 6.7 话题名 / TF / `ros_gz_bridge`（**风险最高的一类**）

**实测：本仓现存 `ros_gz_bridge` 配置文件 = 0 个**（`grep -rl ros_gz_bridge src/` 无命中）
⇒ 这部分是**纯新增**，不是改写。

**必须跨过的差异**：

| 项 | Classic 现状 | 新 Gazebo | 桥接动作 |
|---|---|---|---|
| LiDAR 点云 | `/livox/lidar/pointcloud`（自研插件发 `PointCloud2`） | `/poc/gpu_lidar/points`（`PointCloudPacked`）或 `/livox/lidar` | `ros_gz_bridge` 加 `PointCloudPacked`↔`PointCloud2` |
| `/scan` 来源 | `pointcloud_to_laserscan`（`src/rm_perception/pointcloud_to_laserscan/`） | **不变**（p2l 仍订阅 ROS `PointCloud2`） | 保持桥后话题名一致 ⇒ **p2l 零改动** |
| `frame_id` | `<link>_<sensor>` 类 | `<model>::<link>::<sensor>`（**实测**） | ★ **`frame_id` 改写或 TF 补帧**，否则 `tf2` 全线报"frame 不存在" |
| `/clock` | `gazebo_ros` 发 | gz-transport `/clock` | 必须桥（`use_sim_time` 依赖它） |
| `/livox/imu` | `libgazebo_ros_imu_sensor.so`（`~/out` 重映射） | `imu-system` + bridge | 话题名对齐 |
| `cmd_vel_chassis` / `odom_ground_truth` | `<remapping>` | bridge | **`ros_gz_bridge` 不做 remapping**，需用 ROS 侧 `topic_tools`/节点重映射 |
| TF | **`map→odom→base_link→base_link_fake`**（`base_link_fake` **67 处引用**）| 新 Gazebo 的 `odom` TF 由 drive system 发 | **单发布者规则**（`tf_interface_contract.md`）**必须逐条复核**，否则 TF 树出现双发布 |

`ros_gz_bridge` 的用法（上游，`ros_gz_bridge` 包已装）：
```bash
ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@GZ_MSG
# 例：/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
（`/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge` 实测存在。）

**工作量 M / 风险高 / 可渐进**（bridge 是独立进程，可加可撤）。
**这是"最容易静默出错"的一类**：话题桥上了但 `frame_id` 不对 ⇒ RViz 看着"什么都没有"，
和本仓 `docs/gazebo_gui_troubleshooting.md` §2 描述的"现象①"是同一类坑。

### 6.8 感知链（linefit → pointcloud_to_laserscan → `/scan`）

- `src/rm_perception/linefit_ground_segementation_ros2/`、`src/rm_perception/patchwork_ground_segmentation/`
  —— 都是 **ROS 侧节点**，吃 `PointCloud2`、吐 `/segmentation/obstacle`，**与仿真后端无关**。
- `src/rm_perception/pointcloud_to_laserscan/config/{laserscan_params,laserscan_params_sensor_frame}.yaml`
  + `launch/pointcloud_to_laserscan_launch.py` —— 同上。
- `bringup_sim.launch.py:1652` 起 `pointcloud_to_laserscan_node`。

⇒ **只要 bridge 后话题名与 `frame_id` 保持契约，感知链应该零改动。**
**工作量 S / 风险低 / 天然可渐进** —— 这也是**选择"用 bridge 保持话题契约"而不是"改下游"的核心理由**。

### 6.9 nav2 params 与地图资产

- **18 个 `nav2_params*.yaml`**（`src/rm_navigation/rm_navigation/params/` 17 个 +
  `src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml`）。
  其中 **10 个左右直接含 `base_link_fake`**（实测）。
  ⇒ 只要 TF 帧名与话题名不变，**这些文件应零改动**。**工作量 S / 风险低**。
- **地图资产**：`src/rm_nav_bringup/map/` **242 MB**、`src/rm_nav_bringup/PCD/` **118 MB**，
  格式是 `.pgm/.yaml/.pcd/.posegraph/.data` —— **与仿真后端完全无关**。
  **工作量 — / 风险无 / 完全不动**（守 `docs/map_assets.md` 的规则）。

### 6.10 `tools/scripts/**` 实验台

- `tools/scripts` 下非 `__pycache__` 文件 **125** 个；其中 **57** 个含 `gazebo|gzserver|gzclient`（实测）。
- 典型需改的：`tools/scripts/mapping/run_mapping_headless.sh`、
  `tools/scripts/localization/run_*_ab.sh`、`tools/scripts/diag/*`、
  `tools/scripts/control/{start_sentinel,watch_stack}.sh`、`tools/scripts/regress/*`。
- 本仓已知的**一个具体隐患**（`bringup_sim.launch.py:1169` 注释原话）：
  "本仓多个 bench/ab 脚本收尾会 `pkill -9 -x gzclient`（**全机**范围，不看归属）"。
  迁移后进程名变成 `ign-gazebo-gui` / `ign gazebo server`，
  **旧 `pkill` 会失效（安静地不杀）或误杀**；反之新脚本写 `pkill -x gz` 也可能误伤。

**工作量 M / 风险中 / 可渐进**：建议**所有 harness 加 `SIM_BACKEND` 环境变量分支 + 进程匹配改成"按 PID"**，
不要引入新的全机 `pkill`。**本次任务严格遵守了"只杀自己的 PID"**（`.tmp_gzsim/run_*.sh`
用 `ps` 前后差分求 PID 集合，只 kill 该集合，实测留下的遗留进程为 `(none)`）。

### 6.11 **"保持不动"清单**（**保住现有基准台的关键**）

| 类别 | 具体 | 理由 |
|---|---|---|
| Gazebo Classic 11 全部 | `gazebo` 11.10.2、`ros-humble-gazebo-ros-pkgs`、所有 `.world`/xacro | 与 Fortress **并存**（实测），不动就是零风险 |
| `ros-humble-ros-gz*` | 已装 0.244.26 | **已经装好，不需要动** |
| `.world` / `model.sdf` 全部 23 个 | 新建 `*_gz/` 目录另放 | 新世界与旧世界同源不同目录，可 A/B |
| 全部 xacro | 新增 `*_gz_sim.xacro` | `robot:=` 槽位加新值，旧值逐字不变 |
| 地图资产 | `map/` 242 MB + `PCD/` 118 MB | 后端无关 |
| nav2 params | 18 个 yaml | 只要契约不变 |
| 感知链全部包 | linefit / patchwork / pointcloud_to_laserscan | ROS 侧，与后端无关 |
| `libros2_livox.so` 与 `mid360.csv` | 原样保留 | **在 Fortress 路径上被替换之前，Classic 路径仍要用它** |
| `.gitignore`、`tools/scripts/{mapping,diag,regress}` 功能脚本、`src/rm_localization/**`、`src/rm_navigation/**/params`、`docs/{map_assets,session_digest,kalman_slam_tutorial,tilted_lidar_fidelity}.md` | — | **本任务明确不碰**（其它任务在改） |

---

## 7. 分阶段推荐计划（P0 → P4）

### 7.0 总体形状

```
P0 可行性(本 spike 已完成大部分) → P1 机器人 → P2 传感器系统 → P3 桥接 → P4 A/B 验收
```

**贯穿全过程的硬约束**：
- **默认值永不改**：新增 `sim_backend:=classic|gz`，`classic` 为默认 ⇒ 现有命令与基准台逐字不变。
- **新世界/新 xacro 放新目录、新 slot**，旧资产**只读不改**。
- **Harmonic/Jetty 一律进容器**，不与 `/opt/ros/humble` 混装（§2.3 实测会 `Remv` Classic）。

---

### P0 — 可行性（**本 spike 已覆盖大半**）

| 项 | 内容 |
|---|---|
| **入口判据** | 有 Fortress 可用（`ign gazebo --versions` 出 6.18.0）—— **本机已满足** |
| **出口判据** | ①`gz-gui` 能起窗口（无 GPU 也行）②`gpu_ray` 能出发真数据 ③知道任意图案"能不能"（**不能**，§3）④知道无 GPU 代价（§4） |
| **本 spike 状态** | ✅ **① ② ③ ④ 全部达成**（§4/§5） |
| **剩余待做** | 真机世界（RMUC2026 STL）加载一次，量加载耗时与 RSS；`ros_gz_bridge` 冒烟一条 `/clock` |
| **工作量** | 小（1–2 天） |
| **风险** | 低 |
| **不许做** | 不碰任何既有文件 |

### P1 — 机器人（把车放进新 Gazebo，**先不管 LiDAR 图案**）

| 项 | 内容 |
|---|---|
| **入口判据** | P0 全部出口达成 |
| **做法** | 新增 `src/rm_nav_bringup/urdf/sentry_robot_*_gz_sim.xacro`：`<gazebo>` 块换成 `ignition-gazebo-{imu,diff-drive|mecanum-drive,joint-state-publisher}-system`；LiDAR 先用**规则网格 `gpu_ray`（R0）**占位；`ros_gz_bridge` 先把 `/clock`、`/livox/imu`、`odom`、`cmd_vel_chassis` 桥起来 |
| **出口判据（必须全部满足）** | ①`ign gazebo` 起得来且 GUI 有窗口 ②TF 树仍是 **`map→odom→base_link→base_link_fake`**，**每个帧单发布者**（`ros2 run tf2_tools view_frames` + 发布者计数）③`/livox/imu` 频率≈100 Hz ④`cmd_vel_chassis` 能驱动底盘、`odom_ground_truth` 有合理数值 ⑤`/clock` 桥通、`use_sim_time` 一致 |
| **工作量** | 中（1–2 周） |
| **风险** | 中（TF 单发布者 / `cmd_vel` 重映射语义） |
| **可逆** | ✅ 全程 `sim_backend:=gz`，默认仍 `classic` |

### P2 — 传感器系统（**决定"等价迁移"还是"降级"**）

| 项 | 内容 |
|---|---|
| **入口判据** | P1 出口全部达成（车已经在新 Gazebo 里跑起来，只是 LiDAR 图案不对） |
| **必须做的决策** | **R0（规则网格，零代码，改图案）vs R1/R2/R3（自研 system，保图案）** —— 见 §3.7.3 |
| **若选 R1（推荐先做）** | 新增独立包（如 `rm_livox_gz`）：读 `mid360.csv` → 一组方向 → 每帧求交（`RayQuery` 或接 `RaycastData`）→ 发 `LaserScan` + `PointCloudPacked`；**`base_link_fake`/`frame_id`/`<topic>` 语义必须与 Classic 逐字对齐** |
| **出口判据** | ①**点云条数 = 30000**、10 Hz、`frame_id` 与 Classic 一致 ②**与 Classic 同一位姿下的点云做几何比对**（本仓已有 `tools/scripts/regress/linefit_replay_probe.py`、`robot11_livox_fov.py` 这类探针可复用）③**RSS/RTF 不劣于 Classic**（Classic 基线：gzserver 3.2 GiB、RTF 0.32–0.46） |
| **工作量** | **L**（R1）／L+（R3） |
| **风险** | **极高** —— 这是整个迁移的成败点 |
| **建议** | **先花 1 天做 R1 的"只出 30000 个方向、不算物理"的最小骨架**，确认"能发对格式 + 能读 CSV"再谈性能 |

### P3 — 桥接（全面对齐话题/TF 契约）

| 项 | 内容 |
|---|---|
| **入口判据** | P2 出真点云 |
| **做法** | 写 `ros_gz_bridge` 配置：`PointCloudPacked↔PointCloud2`、`LaserScan`、`Imu`、`Clock`、`Twist`、`Odometry`、`TFMessage`；对照 `docs/tf_interface_contract.md` 逐条复核 |
| **出口判据** | ①**下游一行不改**就能收到 `/scan`、`/livox/lidar`、`/livox/imu`、`odom`/TF ②`base_link_fake` **仍然只有一个发布者** ③`pointcloud_to_laserscan` → `/scan` 频率与 Classic 一致 ④RViz 里 TF 树与点云都正常（**不是"看着什么都没有"**） |
| **工作量** | 中 |
| **风险** | **高**（静默失败：桥上了但 `frame_id` 不对） |
| **可逆** | ✅ bridge 是独立进程 |

### P4 — A/B 验收（**证明不劣于基准台**）

| 项 | 内容 |
|---|---|
| **入口判据** | P3 出口全部达成 |
| **做法** | 复用 `tools/scripts/**` 既有 harness（**加 `SIM_BACKEND` 分支，不用全机 `pkill`**），同世界/同机器人/同参数跑 classic vs gz |
| **出口判据（量化）** | ①**RTF**：gz ≥ classic（classic 基线 0.32–0.46 @ robot11）②**RSS**：gz 显著优于 classic 的 3.2 GiB ③**建图/定位产物可比**：同一条 `mode:=mapping` 路线的 ATE / `/scan` 丢帧率 / GICP fitness 在容差内 ④**GUI 可用性**：在无 GPU 机器上开 GUI 不崩、操作不卡到不可用 ⑤**感知等价性**：点云几何比对通过 |
| **工作量** | 中 |
| **风险** | 中（口径不一致会得出假结论——见 §5.5 的口径警告） |
| **交付** | 一份带数字的迁移验收报告 + 默认值是否切换的决策 |

---

### 7.5 GPU-less 路径结论（**用户第三个问题**）

| 场景 | 可行方案 | 代价 |
|---|---|---|
| **有 GPU 的机器** | 原生（`ogre2` + GPU） | 基线 |
| **没有 GPU，但要 GUI** | **方案 A：`Xvfb` + `LIBGL_ALWAYS_SOFTWARE=1`（Mesa llvmpipe）** —— **本机实测跑通并出真数据** | **GUI 软件光栅化极吃 CPU：`ign gazebo gui` 实测 1269 %（≈12.7 核）；FSAA 从 8 降到 0** |
| **没有 GPU，也不要 GUI（CI/无头回归）** | **方案 B：`ign gazebo -s --headless-rendering`（软件 EGL）** —— **本机实测：无 DISPLAY、零报错、点云逐字节同尺寸** | server 侧很便宜（实测 **23.5 % 单核**） |
| **想要"完全不碰渲染器"** | **方案 C：只有 Jetty 有（`gz-sim-cpu-lidar` + `gz-physics`），但 Humble 没有 Jetty 的 `ros_gz` 包** | **本环境不可用** |

**⇒ 对"可能有的电脑没有 GPU"的答复**：
**能跑，不需要 GPU**；两条软件路径（虚拟屏 + 软件 GL / `--headless-rendering` 软件 EGL）**本机都实测通过**。
**真正的代价在 GUI 的 CPU 上（十几核）**，而不在"能不能跑"。
如果某台机器连软件 GL 都紧张，**`gui:=False` 只留 RViz** 是现成的降级档（Classic 那边同一个开关已在用）。
👉 **但注意：因为 Fortress 没有 CPU `ray`（§3.5），LiDAR ⇒ 渲染器 ⇒ 软件 GL 这条依赖在无 GPU 机器上是刚性的**，
**不存在"关掉渲染器只留 LiDAR"的省法**（除非上 Jetty，而 Humble 拿不到）。

### 7.6 **两个必答问题**

**Q1：迁移真的能消掉 30000 射线的成本，还是只是搬家？**

**答：真的消掉，不是搬家。** 机制上，
- **Classic 现状**：`livox_ode_multiray_shape.cpp` 的 `AddRay()` **每条射线**创建一个 `ODECollision` + 一个 `ODERayShape`，
  落在 glibc 堆上（`[heap]` 2.78 GiB），且**世界更新是单线程**的，射线求交必须在同一个线程里逐条做
  ⇒ **成本是 O(rays)，而且是串行的**（这正是本仓记录的"瓶颈是 Classic 世界更新单线程"）。
- **新 Gazebo `gpu_ray`**：射线方向被打包成一张纹理，**一次光栅化 pass** 出整张深度图
  ⇒ **内存 O(1)**（与射线数基本无关），**成本不随射线数线性增长**。
- **本机实测支持这个判断**：射线数 ×10.4，**RSS 只涨 5.8 MiB**，**RTF 从 1.0001 到 0.9999（不动）**。

⚠️ **诚实的限定**：**这不等于"整栈 RTF 会从 0.4 变成 1.0"。**
本仓的 0.32–0.46 是在**完整栈（SLAM/LIO/nav2 + 真机世界 + robot11）**下量的；
本次 PoC 是**静态微型世界**（物理成本≈0）。**能安全说的是**：
"**射线数不再线性放大内存与单线程世界更新成本**"；
"**整栈 RTF 提升多少**"必须靠 **P4 的 A/B 真跑**才能给数字。
另外：**如果最后必须走 R1/R2/R3 自研 system，那么"消除"的结论要重新评估** ——
自研 CPU 射线（R1）本质上是**把 ODE 的活搬到自己手里**，很可能**又变成 O(rays) 的 CPU 成本**
（虽然能省掉"每射线一个碰撞体"的内存，因为可以只存方向向量）；
**只有 R3（改造 OGRE2 `GpuRays` 的方向纹理）才保得住"光栅化 O(1)"这个优势**。
**这是 P2 阶段最重要的技术判断。**

**Q2：GUI 能留下吗？**

**答：能，而且不需要任何妥协。**
- **架构上**：新 Gazebo 的 GUI 是**独立进程**（实测：`ign gazebo gui` 与 `ign gazebo server` 是两个 PID），
  各持**独立的 gz-rendering 实例**，可以分别指定引擎
  （`--render-engine-gui` / `--render-engine-server`）。**"GUI 必须留着"这条诉求在新 Gazebo 上是原生支持的**，
  不存在"无头替代 GUI"的被迫选择。
- **实测上**：本机（**无 GPU**）在 Xvfb + llvmpipe 下拿到了 **1000×845 的真实 GUI 窗口**，
  GUI 插件（3D View / Entity Tree / World control / World stats / Transform control …）全部加载。
- **可保留的开关语义**：现有 `gui:=True/False` 可以**逐字保留**
  （`gui:=False` = 只起 server，与 Classic 的"只起 gzserver"语义一一对应）。
- **代价（要提前说清）**：无 GPU 机器上 GUI 的软件光栅化要吃 **十几核 CPU**（实测 1269 %），
  画质上 FSAA 从 8 被迫降到 0。**本仓 Classic 的 `gzclient` 本来就在软件光栅化下吃 444–609 % CPU**
  （`docs/gazebo_gui_troubleshooting.md` §6.3），所以这不是新问题，只是**更贵一些**。

---

## 8. 诚实：**本次没能验证的东西**

| # | 未能验证 | 原因 / 现状 |
|---|---|---|
| 1 | **Gazebo Classic 11 的精确 EOL 日期** | 官方 releases 页只列新 Gazebo（Fortress 起）。**未抓到 Classic 11 EOL 的一手 URL** ⇒ 本文不引用该日期。 |
| 2 | **gz-sim#1958 / #2645 的正文与结论** | GitHub issue 页在本沙箱只返回导航框架、正文被截断（`web_fetch` 返回 truncated）。**只确认了标题与存在性**，未逐句核对上游对"自定义扫描形状"的回复。 |
| 3 | **有 GPU 与无 GPU 的性能对照** | 本机**没有 GPU**（无 `/dev/dri`、NVIDIA 驱动未加载）⇒ 无对照基线。所有"无 GPU 数字"都是绝对值，不是"损失了多少"。 |
| 4 | **Harmonic 的实际表现** | 按任务要求"不丢现有 bench"，**没有安装**（`apt-get -s` 显示会卸载 Classic）。**未实测**。 |
| 5 | **真机世界（RMUC2026 / RMUL2026 STL）在新 Gazebo 里的加载时间与内存** | PoC 用的是微型世界。世界 mesh 2.19 MB 级，加载耗时未量。 |
| 6 | **机器人动力学（有质量/关节/摩擦）下的 RTF** | PoC 世界全 `static`，物理成本≈0。**P1 才能量**。 |
| 7 | **`ros_gz_bridge` 端到端（含 `PointCloudPacked` ↔ `PointCloud2`、`frame_id` 改写）** | 未搭。这是 P3 的核心工作。 |
| 8 | **`RayQuery` 在软件 GL（llvmpipe）下的逐条射线吞吐** | 未测。**这是 R1 路线可行性的关键未知数**——如果要自己写 system 用 `RayQuery` 打 30000 条，需要单独基准。 |
| 9 | **`ogre`（OGRE 1.9）与 `ogre2` 的差异** | 两者本机都装了，只用了 `ogre2`（Fortress 默认）。 |
| 10 | **`gz sim` 与 `ign gazebo` 的命令差异全集** | 只验证了 `gz sim` 在 Fortress 上不可用（`Invalid arguments`）。其它子命令（`gz topic` / `ign topic`）只验证了可用。 |
| 11 | **13 个 obstacle C++ 插件的行为** | 只做了静态清单与 API 依赖分析，**未运行**（只在 `RMUL2024_world_dynamic_obstacles` 用到）。 |
| 12 | **`base_link_fake` 的 67 处引用逐条复核** | 只统计了数量，未逐条判断哪些会受迁移影响。 |
| 13 | **`mid360.csv` 的 800,001 行与 `<samples>30000</samples>` 的精确对应关系** | 只确认了 CSV 行数/体积与插件读 `samples`/`downsample` 的代码位置，**未逐行读 `csv_reader.hpp` 验证抽样规则**。 |
| 14 | **`--headless-rendering` 是否真的走了 EGL** | 实测"无 DISPLAY 能跑通 + 出真数据"是硬事实；但**没抓 EGL 调用栈**证明它用的是 EGL 而非别的机制。 |
| 15 | **上游 issue 中"是否有人在改任意图案支持"** | 未找到明确的 roadmap/PR。 |

---

## 附录 A 复现命令（全部在 `.tmp_gzsim/`，全程不动功能代码）

```bash
cd /home/weicheng/HzMi_rmsimulation

# ---- 环境事实 ----
apt-cache policy ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
apt-cache policy ros-humble-ros-gzgarden ros-humble-ros-gzharmonic libgz-sim8-dev
apt-get download --print-uris libgz-sim8-dev ros-humble-ros-gzharmonic   # 只看 URI/大小
apt-get -s install ros-humble-ros-gzharmonic | grep -i Remv              # ★ 看会不会删 Classic
ign gazebo --versions; gz sim --versions        # 后者在 Fortress 上 "Invalid arguments"
glxinfo -B                                       # 需要 DISPLAY

# ---- 规范全文（一手） ----
grep -nE '<element name=|<attribute name=' /usr/share/sdformat12/1.9/lidar.sdf

# ---- PoC：带 GUI ----
bash .tmp_gzsim/run_poc.sh gui 50                # 内部起 Xvfb :99 + LIBGL_ALWAYS_SOFTWARE=1
cat .tmp_gzsim/run_gui/windows.txt               # 应看到 "Gazebo GUI" 1000x845
cat .tmp_gzsim/run_gui/stats.txt                 # real_time_factor
cat .tmp_gzsim/run_gui/topics.txt                # /poc/gpu_lidar, /poc/gpu_lidar/points, /poc/imu

# ---- PoC：30000 射线标定 + 无 DISPLAY 无头 ----
python3 .tmp_gzsim/gen_world.py 360  8 .tmp_gzsim/w_2880.sdf
python3 .tmp_gzsim/gen_world.py 1000 30 .tmp_gzsim/w_30k.sdf
bash .tmp_gzsim/run_scale.sh rays2880_server        .tmp_gzsim/w_2880.sdf server   xvfb 70
bash .tmp_gzsim/run_scale.sh rays30k_server         .tmp_gzsim/w_30k.sdf  server   xvfb 70
bash .tmp_gzsim/run_scale.sh rays30k_headless_nox   .tmp_gzsim/w_30k.sdf  headless nox  60

# ---- 结果核对 ----
grep -E '^(width|height|point_step)' .tmp_gzsim/scale_rays30k_server/points.txt   # 1000 x 30
tail -3 .tmp_gzsim/scale_rays30k_server/stats.txt                                 # RTF ~1.0
```

**注意**：
- 沙箱里 `/tmp` 是**每次 bash 调用独立**的 ⇒ **Xvfb 与 `ign gazebo` 必须在同一次调用里起**，脚本已按此写。
- `$HOME/.ignition` **只读会导致 GUI 直接 abort**（§5.1）⇒ 脚本把 `HOME` 重定向到 `.tmp_gzsim/home`。
- 脚本收尾用 **`ps` 前后差分求 PID 集合**只 kill 自己的进程，**不做全机 `pkill`**（实测遗留 = `(none)`）。

## 附录 B 引用 URL 清单

**官方文档**
- 版本配对矩阵：<https://gazebosim.org/docs/latest/ros_installation/>
- Gazebo 各版本 EOL：<https://gazebosim.org/docs/all/releases/>
- 传感器教程（`gz sim`）：<https://gazebosim.org/docs/latest/sensors/>
- ROS 2 各发行版 EOL（REP-2000）：<https://www.ros.org/reps/rep-2000.html>

**上游源码 / 规范**
- gz-sim10 `cpu_lidar` 系统：<https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems/cpu_lidar>
- gz-sim10 `CpuLidar.cc`：<https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/cpu_lidar/CpuLidar.cc>
- gz-sensors10 `CpuLidarSensor.cc`（`GenerateRays` / `unitVectors`）：<https://github.com/gazebosim/gz-sensors/blob/gz-sensors10/src/CpuLidarSensor.cc>
- gz-rendering6 `GpuRays.hh`（只有规则网格 API）：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/GpuRays.hh>
- gz-rendering6 `RayQuery.hh`（逐条任意方向，但依赖 `Scene`）：<https://github.com/gazebosim/gz-rendering/blob/gz-rendering6/include/gz/rendering/RayQuery.hh>
- gz-sim6 `System.hh`（`ISystemConfigure/PreUpdate/PostUpdate`）：<https://github.com/gazebosim/gz-sim/blob/gz-sim6/include/gz/sim/System.hh>
- SDFormat `<lidar>` 规范：<https://sdformat.org/spec?ver=1.9&elem=lidar>（该页是 JS 渲染，**权威文本用本机 `/usr/share/sdformat12/1.9/lidar.sdf`**）

**上游 issue（自定义图案）**
- <https://github.com/gazebosim/gz-sim/issues/1958> Custom lidar sensor
- <https://github.com/gazebosim/gz-sim/issues/2645> Build my own Lidar Sensor with custom scanning shape
- <https://github.com/gazebosim/gz-sim/issues/2595> Gazebo in Windows Docker cannot use Nvidia GPU, falls back to using CPU

**本仓内部真值**
- `docs/gazebo_gui_troubleshooting.md` §6.3 / §6.4.1（Classic 基线：gzclient 483.7 MiB / 444–609 %；
  gzserver 3199.9 MiB / 66.9 %；Livox 插件 +2.74 GiB ≈ 94.8 KiB/条，根因 `AddRay()`）
- `docs/tf_interface_contract.md`（TF 契约、单发布者）
- `docs/worlds.md`（场地资产）、`docs/robot_models.md`（模型槽位）、`docs/ground_segmentation_slots.md`（感知链）

---

## 9. 第二枪（`.tmp_gzsim2/`）：R0/R1/R3 收口 —— 修掉一个"看起来通过"的假数字、量出 CPU 射线、回答 GUI 与 10 Hz 的关系

> 本节是 §5（`.tmp_gzsim/`，第一枪）的续篇。第一枪用的是**微型世界 + 规则网格**，只回答了
> "Fortress 能不能跑、30000 条射线贵不贵"。第二枪换到**真机世界**（`RMUC2026.stl`）+ **真 MID-360 图案**
> （`mid360.csv`）+ **真机器人位姿**，并补上第一枪明确没测的四件事（§8 的 #5 / #6 / #8 / #13）：
>
> 1. **R1（纯 CPU 逐射线 BVH）到底多少 µs/ray**，1/2/4/8/16 线程怎么摊，30k@10 Hz 够不够；
> 2. **R0（Fortress `gpu_ray` 规则网格）在 30k / 2880 × headless / GUI 下的真实数字**；
> 3. **GUI 把点云从 10 Hz 拖到 8.35 Hz 的机制**（是 RTF 掉了，还是 publisher/时钟耦合错了）；
> 4. **R3（改造 OGRE2 `GpuRays` 的方向表）在源码层是否现实**、要动几个包、能不能上游。
>
> 一句话结论：**R1 在"没有 GPU 的电脑"上是可行的（30k 条 12.5–29.4 ms/帧 @1 线程，
> 4 线程 5.6–13.6 ms，预算 100 ms 有 3.4–18× 余量）；R0 已经能跑且便宜，但它给不了 MID-360 的图案；
> R3 技术上成立、但必须 fork 2–3 个包且精度受立方图重采样限制；GUI 与"10 Hz 点云"在 sim 时间里
> 完全兼容，代价是 wall-clock 吞吐掉到 0.82–0.84×。**
>
> ⚠️ **同时必须记一笔**：第一枪留下的 R1 数字（`us_per_ray=0.348`、`fits_10Hz=YES`）是
> **无效的**——世界 STL 是毫米、探针按米算，**0% 命中**却"通过"了。见 §9.1.2。

### 9.0 本节与 §5 的口径差异（**别把两节的数字混着比**）

| | §5（第一枪，`.tmp_gzsim/`） | 本节（第二枪，`.tmp_gzsim2/`） |
|---|---|---|
| 世界 | 微型世界（1 个地面 + 2 面墙） | **真机世界** `RMUC2026.stl`（2,295,084 B，45,900 三角面，137,700 顶点，1 个 submesh） |
| 世界真实尺寸 | 未量 | **29.150 × 16.050 × 1.730 m**（mesh 原始 bbox 29150×16050×1730 **mm**） |
| 传感器图案 | 规则网格 1000×30 | **真 `mid360.csv` 的 30,000 行滚动窗口**（方位 0–360° 均匀、俯仰 −7.16°…+52.16°，85.3% 在地平线以上） |
| 测的东西 | 端到端 RTF / RSS / CPU（`gpu_ray`） | ①**射线求交本身**（CPU BVH，R1）②端到端（`gpu_ray`，R0）③源码（R3） |
| 能否互换 | **不能**：本节 R1 是"求交成本"，§5 是"整机世界更新成本" | |

### 9.0.1 本节关闭 / 未关闭的 §8 条目

| §8 # | 第一枪未验证 | 本节状态 |
|---|---|---|
| #5 | 真机世界（RMUC2026 STL）的加载时间与内存 | **关闭（部分）**：`gz-common::MeshManager` 加载 **16.4–16.9 ms**（45,900 面 / 2.29 MB），BVH 建树 **22.1–24.0 ms**；world 真实尺寸 **29.150×16.050×1.730 m**。整机 RSS 见 §9.2。 |
| #6 | 有质量/关节/摩擦的动力学下的 RTF | **关闭（部分）**：P1-lite（含 `DiffDrive` + `Physics` + `IMU`）headless RTF **0.999**（§9.3.1）。完整栈（SLAM/LIO/nav2）仍未测。 |
| #8 | `RayQuery` 在 llvmpipe 下的逐条射线吞吐 | **未关闭**：第一枪那组 `us_per_ray=0.348 / engine_ms=117.3` 与 CPU BVH 的 0-hit 数字**同源**（mm/m 单位错），**同样无效**；本轮没重测（见 §9.6 #5）。**但 R1 的答案已由 CPU BVH 给出**（§9.1），`RayQuery` 不再是 R1 的唯一选项。 |
| #13 | `mid360.csv` 的 800,001 行与 `<samples>30000</samples>` 的对应关系 | **关闭**：`csv_reader.hpp:20-45` 逐行读三元组；`livox_points_plugin.cpp:407-427` 是**滚动窗口** `[currStartIndex, currStartIndex+30000)`，`index = k % 800000` ⇒ **26.7 帧一个循环**。本节用 `RAY_START` 复现了第 0/10/20 帧（命中率 16.70/17.05/16.72 %）。 |

### 9.1 R1：纯 CPU 逐射线 BVH（**本节最硬的数字**）

#### 9.1.1 方法（被测对象 / 坐标 / 射线 / 线程 / 重复次数）

**探测器**：`.tmp_gzsim2/cpu_ray_bvh.cc`（438 行 C++17，依赖 `ignition-common4` 的 `MeshManager`
加载 STL、`bullet` 的 `btBvhTriangleMeshShape` 做求交、`ignition-msgs8` 只为复用常量）。
它**不碰渲染器、不需要 GL/DISPLAY**，是 R1 路线（"自己写一个 gz system 逐条打射线"）的成本上界估计。

**世界坐标变换**（与 `world/RMUC2026_world/model.sdf:35-40` 完全一致）：

```
world = mesh_local × 0.001 + (0, 0, +1.6413436)
```

- `0.001`：STL 是**毫米**；
- `+1.6413436`：mesh 局部里**可行驶地面顶面**在 z = −1.64134362793，抬起来让地面落在 world z = 0
  （mesh bbox 的 z 是 −1.84134…−0.11134 m，即底板 0.2 m 厚 + 场地高 1.53 m）；
- 变换后实测 bbox：`(-14.575, -6.401, -0.200) … (14.575, 9.649, 1.530)`，即 **29.150 × 16.050 × 1.730 m**。

**传感器位姿**：`RMUC2026` 的出生点是 `(10.925, 2.525, 0.2)`（`rm_simulation.launch.py:134-139`），
`base_link → livox_frame` 是 `xyz="0.12 0 0.175"`（`simulation_waking_robot.xacro:17`），落地后
雷达光心约在 **0.26 m**。所以标称位姿取 **(10.925, 2.525, 0.26)**，另测 4 个场景（§9.1.4）。
`max_range` 取 **200 m**（Classic 的 `mid360.xacro:9` 就是 `laser_max_range=200.0`），
另做 10/30/70/200 m 敏感性（§9.1.4 末）。

**射线**：直接读 `src/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv`，
方向约定与 Classic 插件**逐字一致**（`livox_points_plugin.cpp:420-423`）：

```
el = 90° − zenith_csv ,  dir = (cos el·cos az, cos el·sin az, sin el)
```

**只取连续 30,000 行**，因为 Classic 插件是**滚动窗口**（`livox_points_plugin.cpp:407-427`：
`end_index = currStartIndex + samplesStep; currStartIndex += samplesStep; index = k % maxPointSize`），
80 万行 / 3 万 = **26.7 帧一个循环**。本探针用 `RAY_START` 复现第 0/10/20 帧（§9.1.4）。
⚠️ **不能只取前 4000 行**：CSV 是**按时间排序**的，前 4000 行几乎全在高仰角带
（前 300 行的 zenith 是 37.8°–46.2°），据此算命中率会得到 0% 的假结果——这一点我在扫描位姿时踩过。

**线程**：每个线程各自一份 `btCollisionDispatcher/btDbvtBroadphase`，**共享只读的
`btBvhTriangleMeshShape`**（BVH 只建一次，22–24 ms）。每档跑 5 次取**最小值**（min 对噪声最稳）。

**构建**：

```bash
g++ -O2 -std=c++17 -Wall cpu_ray_bvh.cc -o cpu_ray_bvh \
    $(pkg-config --cflags --libs ignition-common4 ignition-common4-graphics ignition-msgs8 ignition-utils1) \
    $(pkg-config --cflags --libs bullet) -lpthread
```

（`ignition-common4-graphics` 必须显式加：`MeshManager` 在 graphics 库里，`ignition-common4.pc`
的 `Libs` 不含它；`ignition-utils1` 是 `Mesh.hh` 里 `ImplPtr.hh` 的依赖。）

#### 9.1.2 ⚠️ 上一枪的 R1 数字**无效**：mm/m 单位 bug，以及"0 命中会静默通过"

第一枪跑的是 `scale=1.0`（把毫米 STL 当米），于是：

```
SCENE bounds=(-14575.00 -6400.66 -1841.34)-(14575.00 9649.34 -111.34) origin=(0.00 1624.34 -1841.04)
SANITY ray0 dir=(-0.011 -0.613 0.790) hit=0 dist=-1.000
RESULT threads=1 rays=3000 done=3000 hit=0.0% total_ms=1.04 us_per_ray=0.348 frame_30k_ms=10.4 fps_30k=95.8 fits_10Hz=YES
```

逐条对账：

| 症状 | 根因 |
|---|---|
| `bounds` 是 `±14575`（不是 ±14.575） | **世界是 29.15 mm 宽**（不是 29.15 m） |
| `origin.z = -1841.04` | 传感器"离地 0.26"落在 mm 世界里 = 离地 0.26 mm |
| `max_range=70.0` 但几乎打不到东西 | 70 单位 = **70 mm**，比场地小 400 倍 |
| `hit=0.0%` 却打印 `fits_10Hz=YES` ⚠️ | **没有"几何有效性"断言**：射线全不中 ⇒ 求交立即返回 ⇒ 计时**异常地快** ⇒ 看起来"通过" |
| 唯一命中的那次 `dist=1233.000` | 在"把原点抬到 AABB 上方"的实验里打到 1233 **mm**，正好说明单位是毫米 |

**这就是为什么必须把"几何正确"变成硬闸门**：本节的探针在打印任何计时之前先打一组 sanity 射线，
**任何一条不满足就 `exit 9` 并拒绝给结论**：

| sanity 射线 | 期望 | 作用 |
|---|---|---|
| 正下方 `(0,0,-1)` | **命中，距离 ≈ 传感器高度 ±0.15 m** | 一举抓住 mm/m、z 偏移、mesh 未变换三类错误 |
| 正上方 `(0,0,+1)` | **不命中**（这个场地 mesh 没有天花板） | 反向断言：`expect=miss` 也是断言，避免"全 miss"被当成正常 |
| 俯角 5° | 命中 ≈ `z/sin5°`（z=0.26 ⇒ 2.983 m），允许 −30% 以内（可能有台阶/矮物） | **与距离相关的**标尺校验，独立于墙在哪 |
| 四个水平方向 | ≥1 个命中且距离在 0.5–40 m（这个场地南边是敞开的，允许逃逸） | 抓整体尺度/朝向错误 |
| `RAY_ENCLOSED=1` | 只放宽"上方必 miss / 5° 必打地面"两条（封闭场景本来就有顶和近墙） | **正下方的断言永不放宽** |

> **教训（值得写进任何 benchmark 的规范）**：**"0 命中"的探针会静默通过每一条性能断言。**
> 任何"每帧 N 条射线"的 benchmark，必须同时断言**命中率非零**且**命中距离几何合理**
> （至少一个"向下打地面 = 传感器高度"的标尺）。否则单位错 1000 倍也能得到漂亮的 µs/ray。

#### 9.1.3 修正后的有效性证据（sanity + 独立暴力求交交叉验证）

标称位姿 `(10.925, 2.525, 0.26)`、200 m、30k 窗口 0：

```
SCENE mesh=.../RMUC2026.stl submeshes=1 verts=137700 tris=45900 load_ms=16.4 bvh_ms=22.7 scale=0.001000 zoff=1.641344
WORLD bounds_m=(-14.575 -6.401 -0.200)-(14.575 9.649 1.530) size_m=(29.150 16.050 1.730)
SENSOR pose=(10.925 2.525 0.260) max_range=200.0 reps=5
SANITY down       dir=(+0.000 +0.000 -1.000) hit=1 dist=0.260 expect=hit~sensor_z PASS=YES
SANITY up         dir=(+0.000 +0.000 +1.000) hit=0 dist=-1.000 expect=miss(no ceiling) PASS=YES
SANITY graze-5deg dir=(+0.996 +0.000 -0.087) hit=1 dist=2.751 expect=hit~z/sin(5) PASS=YES
SANITY +x         hit=0  SANITY -x hit=1 dist=6.925  SANITY +y hit=0  SANITY -y hit=1 dist=3.808
SANITY overall=PASS fails=0
HITDIST threads=1 p01=2.17 p05=2.49 p50=3.74 p95=7.41 max=26.64 zero_lt_0.1m=0
```

- **正下方 = 0.260 m**，与传感器高度**完全一致** ⇒ 尺度、z 偏移、mesh 变换三者同时被钉住；
- 命中距离分布 `p05=2.49 / p50=3.74 / p95=7.41 / max=26.64 m` ⇒ 打的是 2–7 m 的地面/结构 + 26.6 m 的远角，
  **没有一条 < 0.1 m**（`zero_lt_0.1m=0`）⇒ 不是"传感器埋在实体里"的退化命中；
- 5° 俯角打 2.751 m（理论 2.983 m，差 7.8%，是场地上一处矮台阶/坡），在 ±30% 容差内；
- 水平 2/4 命中（+x / +y 打空，因为这个场地 0.26 m 高度上南/东边界是敞开的），符合目视几何。

**独立交叉验证**（防止"BVH 和我自己的射线构造一起错"）：用 Python + numpy 写一份**暴力
Möller–Trumbore**（45,900 个三角面 × 35 条射线，同一 STL、同一变换、同一 CSV 行），与 C++ 的
`ClosestRayResultCallback` 结果逐条比对：

```
   idx   C++ dist    py dist    delta
  6156     5.1178     5.1178  0.00000
  ...
 12599     6.8006     6.8006  0.00000
     0    -1.0000    -1.0000  0.00000   （未命中，两边一致）
mismatches: 0 of 35      （30 条命中 + 5 条未命中，最大 |Δ| = 1e-5 m）
```

#### 9.1.4 结果：4 个场景 × 1/2/4/8/16 线程（**有效数字**）

全部：`mid360.csv` 连续 30,000 行（第 0 帧）、`max_range=200 m`、z = 0.26 m、min-of-5。
`µs/ray` = 该线程数下总时间 ÷ 30,000；`frame_30k_ms` = 一帧 30k 条射线的墙钟时间；
`fits` 判据 = 预算 100 ms/帧（10 Hz）。

| 场景 | 位姿 (x,y,z) | **命中率** | 1 线程 µs/ray → 帧 ms | 2 线程 | 4 线程 | 8 线程 | 16 线程 |
|---|---|---|---|---|---|---|---|
| **A1 开阔（南边界）** | (4, −6, 0.26) | **9.82 %** | 0.484 → **14.52 ms** | 0.279 / 8.36 ms (1.74×) | 0.201 / 6.03 ms (2.41×) | 0.113 / 3.40 ms (4.27×) | 0.066 / 1.99 ms (7.30×) |
| **A2 开阔（西侧）** | (−10, 0, 0.26) | **18.08 %** | 0.471 → **14.14 ms** | 0.294 / 8.83 ms (1.60×) | 0.207 / 6.20 ms (2.28×) | 0.113 / 3.39 ms (4.17×) | 0.068 / 2.05 ms (6.91×) |
| **B 标称出生点** | (10.925, 2.525, 0.26) | **16.70 %** | 0.417 → **12.50 ms** | 0.261 / 7.84 ms (1.59×) | 0.185 / 5.56 ms (2.25×) | 0.107 / 3.21 ms (3.90×) | 0.063 / 1.89 ms (6.61×) |
| **C 结构化（命中重）** | (12, 8, 0.26) | **42.81 %** | 0.578 → **17.34 ms** | 0.338 / 10.13 ms (1.71×) | 0.265 / 7.95 ms (2.18×) | 0.152 / 4.56 ms (3.80×) | 0.089 / 2.68 ms (6.46×) |
| **D 封闭上限（不可达）** | (10, −2, 0.26) | **100.00 %** | 0.978 → **29.35 ms** | 0.628 / 18.84 ms (1.56×) | 0.452 / 13.57 ms (2.16×) | 0.246 / 7.39 ms (3.97×) | 0.140 / 4.21 ms (6.97×) |

> **D 的定位**：`(10,−2)` 的顶只有 0.14 m 高（`SANITY up hit=1 dist=0.140`），**真机（含雷达外壳）塞不进去**，
> 所以它是"全部命中"的**成本上限**，不是可用位姿；它的 sanity 按设计会 FAIL（`RAY_STRICT=0` 放过）。
> 本场地的**可用位姿**命中率上限就是 **C 的 42.8 %**（再高只能钻进 0.4 m 高的空腔）。

**线程伸缩（30k 短批 vs 10 帧摊销）**：

| 线程 | A1 30k | C 30k | A1 10 帧（300k 射线） | C 10 帧 |
|---|---|---|---|---|
| 1 | 1.00× | 1.00× | 1.00× (0.455 µs/ray) | 1.00× (0.593 µs/ray) |
| 2 | 1.74× | 1.71× | 1.89× | 1.91× |
| 4 | 2.41× | 2.18× | 2.73× | 2.98× |
| 8 | 4.27× | 3.80× | 4.77× | 4.27× |
| 16 | 7.30× | 6.46× | **8.36×** | **7.25×** |

- **4 线程 5–8 ms/帧、8 线程 3–4.6 ms/帧**，即 10 Hz 预算的 **12–30 倍余量**；连**单线程**都有 3.4–8× 余量。
- 伸缩**亚线性**是三个原因叠加：①30k 一帧只有 12–17 ms，16 线程的**线程创建/回收 ~0.8 ms** 占比可观
  （10 帧摊销后 16 线程从 6.6–7.0× 升到 7.3–8.4×）；②本机是 **i7-14700HX（8 P-core + 12 E-core，28 线程）**，
  超过 8 线程就落到 E-core；③每线程一个 `btDbvtBroadphase` + 共享 BVH 的**内存带宽**瓶颈。
  ⇒ 真做 R1 时应当用**常驻线程池**（不要每帧建线程），4–8 个工作线程就够。

**敏感性**（标称位姿、1 线程）：

| `max_range` | 10 m | 30 m | 70 m | 200 m |
|---|---|---|---|---|
| µs/ray | 0.391 | 0.405 | 0.433 | 0.427 |
| 命中率 | 16.48 % | 16.70 % | 16.70 % | 16.70 % |

⇒ **射程几乎不影响成本**（BVH 对逃逸射线剪枝很快），所以"为了省钱把 200 m 改小"没有意义；
反过来，**200 m 的配置不用改**。

| CSV 滚动窗口 | 第 0 帧 | 第 10 帧（start=300000） | 第 20 帧（start=600000） |
|---|---|---|---|
| 命中率 | 16.70 % | 17.05 % | 16.72 % |
| µs/ray（1 线程） | 0.424–0.462 | 0.426–0.431 | 0.437–0.456 |

⇒ **窗口之间命中率稳定在 16.7–17.1 %**，不是"第 0 帧特殊"。

#### 9.1.5 命中率不是成本的主因；"命中射线 vs 未命中射线"分账（1 线程）

| 场景 | 混合 µs/ray | **只打命中的射线** | **只打未命中的射线** | 命中/未命中 |
|---|---|---|---|---|
| A1（9.82 %） | 0.484 | **1.985** (n=2,945) | 0.326 (n=27,055) | 6.1× |
| A2（18.08 %） | 0.471 | **1.382** (n=5,423) | 0.281 (n=24,577) | 4.9× |
| B（16.70 %） | 0.417 | **1.392** (n=5,009) | 0.264 (n=24,991) | 5.3× |
| C（42.81 %） | 0.578 | **0.973** (n=12,844) | 0.369 (n=17,156) | 2.6× |
| D（100 %） | 0.978 | **1.008** (n=30,000) | —（0 条） | — |

三条结论，**和"命中越多越慢"的直觉不完全一样**：

1. **单条命中射线比未命中射线贵 2.6–6.1 倍**：`ClosestRayResultCallback` 只在**命中**时才做
   精确三角面求交（未命中只走 AABB 剪枝）；
2. 但**整帧成本并不由命中率单调决定**：A1 的命中率只有 9.82 %，成本（0.484）却**高于**命中率 18.08 % 的 A2（0.471）。
   原因是 A1 的命中是**长距离掠射地面**（`p95=12.48 m`，见 HITDIST），穿过的 BVH 节点更多、
   命中的三角面更多；A2 的命中集中在 1.9–4 m。**成本 ≈ Σ(节点访问 + 三角面测试)**，不是命中条数；
3. 所以**"命中重"和"未命中重"两个场景都要报**（本节 A1/A2 vs C vs D）：可用位姿区间
   **9.8 % → 42.8 % 命中**对应的成本是 **0.417–0.578 µs/ray（1 线程）**，只差 1.4 倍；
   真正的成本上限（100 % 命中、0.978 µs/ray）**比标称位姿贵 2.3 倍**。

#### 9.1.6 R1 的判定

**✅ 可行。** 在"真世界 + 真 30k 图案 + 真位姿"下：**1 线程 12.5–29.4 ms/帧**，
**4 线程 5.6–13.6 ms/帧**，**预算 100 ms 有 3.4–18× 余量**，且**完全不碰渲染器**
（⇒ 无 GPU 也能跑；连 `--headless-rendering`/EGL 都不需要，CI 里不会再出现
"渲染器起不来 ⇒ LiDAR 没数据"的耦合）。

**必须一起说的三条边界**：

1. 这里测的是**射线求交**，不含 30k 点/帧的**消息构造 + 序列化 + 传输**
   （480 KB LaserScan + 960 KB PointCloudPacked，见 §9.2）。R1 与 R0 共用同一条发布路径，
   所以 **R1 相对 R0 的增量就是这 0.5–13 ms**；
2. BVH 是**静态世界建一次**（22–24 ms）。**动态障碍**要 refit/rebuild，本节**没测**；
3. 只算**世界 mesh**，射线会穿过机器人自身（真系统必须处理 self-hit / 传感器安装件）。

### 9.2 R0：30k / 2880 的真实数字（headless 与 GUI）

三个运行都在**同一个盒子**上，世界是 `.tmp_gzsim2/w_30k.sdf` / `w_2880.sdf`
（规则网格 `gpu_ray`，1000×30 与 360×8），`-r -v 3`，各跑 ≈25 s。RTF 有两个口径：
**报**= `/stats` 的 `real_time_factor`（瞬时/窗口值，末次采样）；**均**= `sim_time / real_time`
（含世界加载与 spawn，所以偏低）。两者都列，别混用。

| 运行 | 射线 | 模式 | **RTF 报** | sim/real（均） | 迭代 | **CPU** | **RSS** | **scan Hz** | **points Hz** | `/clock` Hz |
|---|---|---|---|---|---|---|---|---|---|---|
| `run_r0_30k_headless` | 1000×30 = 30,000 | `-s --headless-rendering` | **0.9994–1.0** | 23.228 / 25.406 = 0.914 | 23,228 | **24.7 %**（单 ruby） | **375 MB** | **10.064** | 25.218 ⚠️ | **1000.01** |
| `run_r0_30k_gui` | 30,000 | `-r -v 3`（server+GUI，Xvfb 1280×800） | **0.77–0.88**（末次 0.8325） | 19.745 / 25.348 = 0.779 | 19,745 | **1321 %**（GUI 1295.8 + server 21.5 + Xvfb 4.0） | server 363 MB + **GUI 588 MB** = 1.09 GB | **8.542** | 20.910 ⚠️ | **841.53** |
| `run_r0_2880_headless` | 360×8 = 2,880 | `-s --headless-rendering` | **1.0** | 23.234 / 25.394 = 0.915 | 23,234 | **23.4 %** | **365 MB** | **10.000** | 28.168 ⚠️ | 999.83 |

载荷与元数据（**扫描与点云是同一份数据的两种编码**）：

| 话题 | 类型 | 字节/帧 | 结构 | `frame_id`（gz 侧） | 桥到 ROS 后 |
|---|---|---|---|---|---|
| `/poc/gpu_lidar` | `ignition.msgs.LaserScan` | **480,193**（480 KB） | `count=1000 vcount=30 ranges=30000` | `sensor_rig::link::gpu_lidar` | `sensor_rig/link/gpu_lidar` |
| `/poc/gpu_lidar/points` | `ignition.msgs.PointCloudPacked` | **960,143**（960 KB） | `width=1000 height=30 point_step=32 row_step=32000` | `frame_id=sensor_rig::link::gpu_lidar` | 同上 |
| `/livox/lidar`（P1-lite） | LaserScan | 480,221 | `count=1000 vcount=30 ranges=30000` | `p1_robot::base_link::livox_lidar` | `p1_robot/base_link/livox_lidar` |
| `/livox/lidar/points`（P1-lite） | PointCloudPacked | 960,148 | `width=1000 height=30 point_step=32` | 同上 | 同上 |

⇒ 10 Hz 下 **≈14 MB/s** 进 `ros_gz_bridge`（480 KB + 960 KB 每帧，两路都订阅时），
`ros_gz_bridge` 进程本身实测 **8.9 % CPU / 70 MB**（`p1_gui/cpu.txt`，第一枪）——**这不是瓶颈**。

**`frame_id` 约定**（迁移必改项，§6.7 的实测补充）：Fortress 默认是
`<model>::<link>::<sensor>`，**`ros_gz_bridge` 把 `::` 换成 `/`**（实测：
`frame_id: p1_robot/base_link/livox_lidar`）。所以 TF 树里的 frame 名必须从 Classic 的
`livox_frame`/`livox_lidar` 改成 `p1_robot/base_link/livox_lidar`（或在 bridge 侧重写）。

⚠️ **表里 points Hz 那三列不要当真**：`25.218 / 20.910 / 28.168` 都是**另一个 15 s 窗口**里
顺序测的（`run_r0.sh:55-57` 是先 points 后 scan，不是同时），而且今天复现出的
"每周期多发 2.2–2.6 条"现象会把点云速率抬高（见 §9.3.4）。**同一时刻**同时订阅两路的 P1-lite
实测是 **10.001 / 10.001 Hz**（`p1_headless/hz_gz_both.txt`），那才是"一帧一路"的真实关系。

### 9.3 "GUI 必看"与"10 Hz 点云"能不能同时成立

第一枪留下的疑问：P1-lite 的 GUI 跑出 **8.350 / 8.428 Hz**（不是 10 Hz），且 `dt_max=0.217 s`。
**是 RTF 掉了，还是 publisher/sim-clock 耦合错了？** 这决定了"GUI 必看"是不是与"10 Hz 点云"冲突。

#### 9.3.1 受控实验（同世界、同传感器、只改渲染模式）

`.tmp_gzsim2/run_rate.sh`：`p1_lite.sdf`（1000×30 `gpu_ray` + IMU + DiffDrive），
**server 与 GUI 分成两个进程启动**（`ign gazebo -s` / `ign gazebo -g`）以便**精确归属 CPU**；
每段 `timeout` 硬上限；每次跑用**独立 `GZ_PARTITION`**（这台机器是共享的，别的任务可能同时在跑 sim，
同一 partition 上的同名 topic 会互相污染）；`HOME` 每跑一个独立可写目录
（只读的 `$HOME/.ignition` 会让 GUI 起不来并 SIGKILL server）。

| 模式 | `/clock` Hz | `/stats` RTF | scan **墙钟 Hz** | scan **sim 时间 Hz** | server CPU | GUI CPU | server RSS | GUI RSS |
|---|---|---|---|---|---|---|---|---|
| **headless**（`-s --headless-rendering`，无 DISPLAY） | **1000.01** | **0.999** | 22.13 | 22.9 | **≈49 %** | — | 387 MB | — |
| **GUI 1280×800** | **833.64** | **0.819** | 21.61 | 28.4 | **≈51 %** | **1112–1130 %** | 392 MB | **610 MB** |
| **GUI 640×480** | **835.43** | **0.879** | 21.28 | 28.4 | **≈51 %** | **1214–1220 %** | 393 MB | **611 MB** |

（CPU/RSS 取 `cpu_samples.txt` 里**真正的 `ign gazebo` 进程**那一行；脚本用 `setsid` 起独立进程组，
样本里还会混进 `timeout` 包装进程（≈0 %/2 MB），**把两者平均会得到 24 %/195 MB 这种假数**，别用。
三个模式的 **server CPU（49→51 %）与 RSS（387→393 MB）基本不变** ⇒ **GUI 没有加重 server 的负担**，
它抢的是整机的核：**GUI 一个进程就吃 11–12 个核**。）

#### 9.3.2 机制：**sim 时钟被整体拖慢**（不是 publisher 的错）

证据链（每一环都是独立测量）：

1. **`/clock` 从 1000.01 Hz 掉到 833.6 Hz**（×0.834）——`/clock` 是"每个 1 ms 步进一条"，
   所以这直接就是**步进频率**掉了；`/stats` 的 RTF 0.999 → 0.819 独立复现同一结论。
2. **server 的 CPU 没变**（49 % → 51 %）而 **GUI 一个进程吃 1112–1220 %**（≈11–12 核）⇒ 抢 CPU 的是 GUI 的软件光栅化
   （llvmpipe，本机无 GPU，FSAA 8 被强制降到 0：`Ogre2RenderTarget.cc:574` 的警告）。
3. **传感器是按 sim 时间定频的**：gz-sensors 的 `Sensor::Update(_now)` 拿 `_now` 和
   `nextUpdateTime` 比（`sensors6_Sensor.cc:458-483`，`nextUpdateTime += 1/updateRate`），
   `GpuLidarSensor::PublishLidarScan(_now)` 用它当 stamp（`sensors6_GpuLidarSensor.cc:283`）。
   所以 **墙钟 Hz = sim 时间 Hz × RTF**。
4. **决定性的一步在"每周期恰好 1 条"的那次运行里**：第一枪 `p1_gui` 8.350 Hz / `p1_gui2` 8.428 Hz
   对 headless 10.001 Hz ⇒ 比值 **0.835 / 0.843**；而 `/clock` 的比值是 **0.834**（本轮）
   / **0.842**（R0 的 `841.53/1000.01`）。**三个独立量在 1 % 内一致** ⇒ **就是 RTF 掉，
   不是 publisher/时钟耦合错**。
5. `dt_max=0.217 s` 也对得上：`100 ms / 0.834 ≈ 120 ms` 的正常周期 + 一次丢帧（≈2 个周期）⇒ 0.217 s。
6. **今天的墙钟速率不能用来验证这条传递链**：今天每周期发 2.2–2.6 条（§9.3.4），所以墙钟
   `22.13 → 21.61` 几乎没掉，而 sim 时间速率 `22.9 → 28.4` 反而偏高（倍数不稳）。**倍数把
   RTF 的效应掩盖了。** 但 `/clock`（1000.01 → 833.64）和 `/stats`（0.999 → 0.819）已经把
   "整场仿真变慢"这件事直接测出来了，第一枪那组 1.0× 的数据则把"传感器跟着变慢"补上了。

**⇒ 谁在拖慢？** 不是"点云算不动"（server CPU 没变），而是 **GUI 的 llvmpipe 把整机内存带宽/CPU
抢到 gz-sim 的 1 ms 步进跑不满 1000 步/秒**。这同时解释了 R0 的 GUI：世界更重（有阴影 + 30k），
GUI 吃 **1296 %**、RTF 0.83、`/clock` 841.5 Hz、scan 8.542 Hz（比值 0.849）。

**⇒ 缩窗口救不回来**：640×480 只把 RTF 从 0.819 抬到 0.879（GUI CPU 反而从 1112 涨到 1216 %）⇒
GUI 的成本是**固定开销**（场景/合成器/llvmpipe 管线），不是像素数。想省只能**关 GUI** 或换有 GPU 的机器。

#### 9.3.3 对"GUI 必看 + 10 Hz 点云"的裁决

| 问法 | 答案 |
|---|---|
| GUI 能不能留 | **能**（独立进程、独立 gz-rendering 实例，§7.6 Q2 已证） |
| 开了 GUI 还有 10 Hz 点云吗 | **在 sim 时间里是 10 Hz**（SDF 的 `<update_rate>10</update_rate>` 没变；第一枪实测 headless `10.001 Hz` → GUI `8.35–8.43 Hz`，算法侧开 `use_sim_time` 看到的仍是 10 Hz 周期） |
| 那 8.35 Hz 是什么 | **墙钟频率**：10 Hz × RTF 0.82–0.88。整场仿真一起变慢，**不是点云单独变慢** |
| 代价是什么 | **墙钟吞吐 0.82–0.88×**：一次 40 s 的跑要 47–49 s；实时性/回归时长要留 ~20 % 余量；`dt_max` 会到 0.2 s（下游 `use_sim_time` 的 TF/odom 也一起变慢，**相对关系不变**） |
| 要 10 Hz 墙钟怎么办 | **关 GUI**（headless `/clock` 1000.0 Hz、RTF 0.999），或上 GPU |

#### 9.3.4 ⚠️ 必须标注的口径异常：**今天所有模式都多发 2.2–2.6 条/周期**

今天在**所有**模式下都测到"传感器一个周期发了 2–3 条消息"：

| `update_rate` | 1 | 5 | 10 | 20 |
|---|---|---|---|---|
| 实测墙钟 Hz | **2.47** | **11.20** | **22.13** | **46.21** |
| 倍数 | 2.47× | 2.24× | 2.21× | 2.31× |

而判据显示"周期本身是对的"：`dt_max ≈ 1/update_rate`（1 Hz → 1.0034 s；10 Hz → 0.1026 s），
`dt_min ≈ 0`（成对出现），**单一 publisher**（`ign topic -i` 只有一行），**`dup_stamps=0`**
（成对的消息 sim stamp 不同），**与射线数无关**（把 30k 降到 300 条，速率仍是 23.6 Hz），
**与是否 `--headless-rendering` 无关**（`-s` 与 `-s --headless-rendering` 都是 22.2 Hz）。

⇒ **判定**：`<update_rate>` 并没有被当成绝对频率执行（每周期 2–3 次 publish）。
第一枪在同一世界测到的是 **1.0×**（10.001 Hz，`dt_max=0.1018`），所以这个倍数**依赖环境/负载，不可复现**。
**因此**：

- **绝对 Hz 数字（本节的 22 Hz、第一枪的 10 Hz）都不能当作"传感器频率"引用**；
  可信的是 **比值**（GUI/headless）、**`/clock` Hz**、**`/stats` RTF**，以及**同时订阅两路时的 1:1 关系**；
- 这也正好解释了 §9.2 表里 `points` 那几个 20–28 Hz 的怪值（顺序探测 + 不同窗口）；
- 机制未查明（见 §9.6 #3）。**在下游用 ROS 侧 `ros2 topic hz` 做验收**，别信 gz 侧的绝对数。

### 9.4 R3：能不能把任意 CSV 图案塞进 OGRE2 的 `GpuRays`

**问题**：Fortress 的 `gpu_ray` 只能配规则网格（`<ray><scan>` 的水平/垂直 samples），
所以拿不到 MID-360 的逐射线图案。能不能**只换"方向表"**、保留"光栅化 + 一次 pass"的优势？

#### 9.4.1 结论先行

**技术上成立，而且比想象中轻——因为上游代码里"方向表"本来就已经是一张纹理了。**
但它是**对 `gz-rendering` 公共 API 的扩展**，落地必然是 **fork（2 个包，带 SDF 管道则 3 个）**，
并且**精度受立方图重采样的硬上限约束**。详见下面。

#### 9.4.2 源码里"方向表"到底在哪（file:line）

上游的实现是**两遍**：

| 环节 | 位置 | 作用 |
|---|---|---|
| 第 1 遍：**立方图** | `Ogre2GpuRays.cc:136-207`（`cubeCam[6]`、`kCubeCameraCount=6`）、`Ogre2GpuRays.cc:563-622`（按 FOV 算每个面要多少采样） | 6 个 90° 相机把**整个球面**渲染成 6 张深度图 |
| 深度 → 距离 | `media/materials/scripts/gpu_rays_1st_pass_fs.glsl:83-86`（`viewSpacePos = cameraDir * d; l = length(viewSpacePos)`） | 每个像素一个"沿该像素方向的射程" |
| **第 2 遍：方向表** | **`Ogre2GpuRays.cc:661` `CreateSampleTexture()`**，填充循环 **`Ogre2GpuRays.cc:722-745`** | 建一张 **RGBA32F 纹理**，**每个输出像素**打包 `(u, v, faceIdx)`：`R=u`、`G=v`、`B=cube face`（注释在 `Ogre2GpuRays.cc:678-682`） |
| 方向 → cube UV | **`Ogre2GpuRays.cc:633-660` `SampleCubemap()`** | 世界方向向量 → 面号 + uv |
| 第 2 遍的着色器 | `media/materials/scripts/gpu_rays_2nd_pass_fs.glsl:26`（`uniform sampler2D cubeUVTex`）、**`gpu_rays_2nd_pass_fs.glsl:66-95`** | `data = texture(cubeUVTex, uv)` → 取 `faceIdx` → 从对应面取 `range/retro` |

**关键**：`Ogre2GpuRays.cc:722-745` 那个双重循环就是"规则网格 → 方向 → cube UV"的**唯一**入口：

```cpp
double v = vmin;
for (unsigned int i = 0; i < h2nd; ++i) {          // 垂直行
  double h = min;
  for (unsigned int j = 0; j < w2nd; ++j) {        // 水平列
    math::Vector3d ray(0, 0, 1);
    math::Quaterniond pitch(math::Vector3d(1,0,0), -v);
    math::Quaterniond yaw  (math::Vector3d(0,1,0), -h);
    math::Vector3d dir = yaw * pitch * ray;
    math::Vector2d uv = this->SampleCubemap(dir, faceIdx);   // ← 方向表
    pDest[index++] = uv.X(); pDest[index++] = uv.Y();
    pDest[index++] = faceIdx; pDest[index++] = 1.0;
    h += hStep;
  }
  v += vStep;
}
```

⇒ **任意 CSV 图案 = 把 `(i,j)` 换成 CSV 的第 k 行**：`az = csv_az[k]`、`el = 90°−csv_zen[k]`，
`h2nd` 从 30 改成 1、`w2nd` 从 1000 改成 30000。**不需要改第 1 遍，也不需要改着色器。**

#### 9.4.3 要改哪些地方、多少行（估算）

| # | 包 / 文件 | 改动 | 估算 |
|---|---|---|---|
| 1 | **`gz-rendering` 公共 API** `include/gz/rendering/GpuRays.hh:121-156`（当前只有 `SetRayCount/SetVerticalRayCount/SetAngleMin/Max`） | 加 `SetRayDirections(const std::vector<math::Vector3d>&)` + `RayDirectionCount()`；**空 = 现状**（后端兼容） | **30–50 行** |
| 2 | `gz-rendering` `BaseGpuRays.{hh,cc}` + `Ogre2GpuRays.{hh,cc}` | 存方向表；`CreateSampleTexture()` 里"有方向表就按表填、没有就走原双重循环"；`UpdateFOV` 里 `w2nd = N, h2nd = 1`；`cubeFaceIdx` 照旧收集（决定要渲染哪几个面） | **60–100 行**（**复用** 661–790 的纹理上传代码） |
| 3 | **`gz-sensors`** `src/GpuLidarSensor.cc:186-240`（`CreateLidar()` 里配网格）+ `src/Lidar.cc:150-163`（`set_count/set_vertical_count`）、`Lidar.cc:233-280`（`PublishLidarScan` 的双层循环） | 读 CSV（`Time,Azimuth,Zenith`），算方向，调新 API；消息改成 `count=N, vertical_count=1`；点云 `width=N, height=1`；`FillPointCloudMsg` 走一维 | **120–200 行** |
| 4 | **`gz-sim`**（可选） | SDF `<sensor type="gpu_ray">` **没有** CSV 字段（`sdformat-9.7/sdf/Lidar.hh:104-260` 只有 `horizontal/vertical` 的 samples/min/max/resolution）⇒ 要么扩 schema + `Sensors` system（**fork `gz-sim6`**），要么**自己写一个 gz system 直接建传感器**（不动 gz-sim，但仍要 fork gz-sensors 才能定制 `GpuLidarSensor`） | **40–150 行** |
| | **合计** | **2 个包（必需）/ 3 个包（SDF 顺手）** | **≈250–400 行 + 打包/版本钉子** |

**注意**：给 `GpuRays` 基类**加虚函数会破坏 ABI**（vtable 布局变），所以"装一个 patch 进现成
`libignition-rendering6`"是不可行的——必须是**整包重编译的 fork**（colcon overlay）。
这对我们是可接受的（迁移本来就要重建工作区），但**不是"改个配置"**。

#### 9.4.4 精度上限（**这条决定 R3 值不值**）

第 1 遍的立方图分辨率是硬编码 clamp：

```cpp
// Ogre2GpuRays.cc:615-622
unsigned int samples1stPass = std::clamp(v, min1stPassSamples /*128*/, max1stPassSamples /*1024*/);
this->Set1stTextureSize(samples1stPass, samples1stPass);
```

⇒ **每个立方图面最多 1024²，即 90°/1024 ≈ 0.088°/texel**。任意方向是从这张图里
**最近邻重采样**出来的，于是：

- **角度量化**：每条射线的方向被吸到 0.088° 的网格上。MID-360 的 30k 图案铺在
  360°×59°（≈21,240 deg²）上，**平均点间距 ≈0.84°**（`sqrt(21240/30000)`），
  立方图 texel 比它**细约 10×** ⇒ **量化误差小于图案间距，可接受**；
- **边缘漏检/重影**：细杆、薄板、栏杆这类目标，命中与否取决于立方图 texel 是否覆盖它
  ⇒ **同一个世界、同一条射线，R3 与 Classic/R1 可能给出不同答案**（Classic 是精确三角面求交）；
  这直接影响"感知等价性"验收（§7.4 的判据 ⑤）；
- **无法保留逐点时间偏移**：CSV 第一列是 `Time`，Classic 插件虽然当前 `offset_time=0`
  （`livox_points_plugin.cpp:381`），但真机是有帧内运动畸变的；R3 和 R0 一样是"一帧一个位姿"。

⇒ **R3 的保真度是"方向对、距离是重采样的近似"**，介于 R0（方向都不对）和 R1（逐射线精确）之间。

#### 9.4.5 要不要 fork / 能不能上游

> **2026-10-18 补充**：R1 的「外面是否已经有人做了」已单独调研，见 [`livox_sensor_prior_art.md`](livox_sensor_prior_art.md) —— **存在 MIT/BSD-3 的现成实现**（`ashduwihch/mid360_gazebo_harmonic` = rmagine/Embree CPU BVH + MID-360 CSV + `CustomMsg`；`uos/rmagine_gazebo_plugins` = BSD-3 且明示 Fortress 已测），**R1 可降级为「移植 + 适配」（S–M）**；关键新事实：Fortress **没有** `components/RaycastData`（那是 `gz-sim8+`），所以"让物理引擎批量求交"在 Fortress 不存在，BVH 必须自己持有。

| 问 | 答 |
|---|---|
| 要 fork 吗 | **要**：`gz-rendering`（必需）+ `gz-sensors`（必需）+ 可选 `gz-sim`；加虚函数 ⇒ 必须整包重编译 |
| 能上游吗 | **技术上可以**（内部结构已经支持"方向表"，PR 是"把表从规则网格换成外部数组"），但要**跨 3 个 repo + SDF schema 变更 + 一个新的 sensor 配置入口**，且会引入"重采样精度"的产品决策 ⇒ 现实中是**几个月级的多 PR**，赶不上我们的迁移窗口 |
| 风险 | **高**：①fork 的 ABI/版本钉子（Fortress 一旦打补丁就要 rebase）②重采样精度影响感知等价性验收 ③llvmpipe 下 30k 方向表的纹理带宽未实测（第 1 遍的分辨率不能随便调大） |
| 与 R1 比 | R1 **不动任何上游包**（只加一个自己的 system）、**逐射线精确**、**纯 CPU**；R3 省的是 CPU（光栅化），但把成本换成"fork + 精度损失" |

#### 9.4.6 R3 判定

**⚠️ 现实但重。** 如果本项目**长期押注 gz**、并且能接受"距离是 0.088° 立方图重采样"的近似，
R3 是唯一能同时拿到"任意图案 + 光栅化 O(1) 内存"的路线；
但**对"这一轮迁移"**，它的性价比低于 R1（见 §9.5）。**不建议把它排进 P2。**

> 顺带说明：第一枪下载的 **`igngz6_Sensors.cc`（1037 行）和 `igngz6_RenderUtil.cc`（3838 行）
> 都不在这条改动路径上**——前者是 gz-rendering 的 sensor 容器（无 `GpuRays`），后者是 OGRE2 场景
> 构建（材质/可视），全文 grep 不到 `GpuRays`。**`gzsim6_Sensors.cc` 那次下载失败（0 字节）也不影响结论**：
> 真正要改的是 `Ogre2GpuRays.cc` 的 `CreateSampleTexture()` 和 `GpuRays` 的公共 API。

### 9.5 决策表与推荐

#### 9.5.0 先回答 §7.6 Q1 留下的开放问题（"自研 R1 会不会又变成 O(rays)"）

第一枪的判断是：**"R1 本质上是把 ODE 的活搬到自己手里，很可能又变成 O(rays) 的 CPU 成本"**，
并说这是 P2 最重要的技术判断。**本节把它量化了**：

- **确实是 O(rays)**：单线程 0.417–0.978 µs/ray，30k 条 = **12.5–29.4 ms/帧**；
- **但这个"O(rays)"在无 GPU 的 28 线程机器上完全付得起**：4 线程 **5.6–13.6 ms**、8 线程 **3.2–7.4 ms**，
  10 Hz 预算（100 ms）有 **7–30× 余量**；
- **内存优势保住了**：R1 只需要存方向向量（30k×3×8 B ≈ 720 KB）+ 一份静态 BVH，
  **不需要"每射线一个碰撞体"**（Classic 的 2.74 GiB 根因）；
- **而且它不碰渲染器** ⇒ 顺手解掉"LiDAR 依赖软件 GL"这条刚性耦合（§7.5 的"没有省法"结论对 R1 不成立）。

⇒ **§7.6 Q1 的答案是分层的**：**"R0 已消除射线成本"成立；"R1 会把成本搬回来"也成立，
但搬回来的量级是 0.5–13 ms/帧，不是 Classic 的 2.74 GiB + 单线程世界更新。**

#### 9.5.1 四条的对照（保真度 × 吞吐 × 无 GPU × 工作量/风险）

| 方案 | **保真度**（能否复现 MID-360 图案 / 距离是否精确） | **吞吐**（30k@10 Hz，本机实测） | **无 GPU 可用** | **工作量 / 风险** | 结论 |
|---|---|---|---|---|---|
| **Classic 11（现状）** | **最高**：逐射线 ODE 求交，图案任意 | **最差**：RTF 0.32–0.46（整栈）、RSS 2.74 GiB（每射线一个 ODE 体） | ✅（但吃 444–609 % CPU 的 gzclient） | 0 / **EOL 风险** | 基准台，别动 |
| **R0：Fortress `gpu_ray` 规则网格** | **低**：方向只能是规则网格，**拿不到非重复图案**；距离精确（光栅化） | **最好**：RTF 0.999（headless）/ 0.82（GUI），RSS 375 MB，CPU **24.7 %**（R0 世界，生命周期均值） | ✅（llvmpipe/EGL，实测） | **0（已跑通）** / 低 | **近期可用的默认档** |
| **R1：自研 CPU BVH system** | **最高**：CSV 逐射线、三角面精确求交；还能加逐点时间戳/噪声 | **够用**：1 线程 12.5–29.4 ms/帧；4 线程 5.6–13.6；8 线程 3.2–7.4（预算 100 ms） | ✅✅ **纯 CPU，连渲染器都不用** | **中高** / 中（自维护 system + 动态世界 refit） | **P2 推荐** |
| **R3：fork `gz-rendering`+`gz-sensors` 喂方向表** | **中高**：方向精确，**距离是 0.088° 立方图重采样**（薄物可能漏/重） | **最好**（同 R0，O(1) 内存） | ✅ | **高**（2–3 个 fork，250–400 行 + ABI 钉子） / **高** | **长期选项，不进 P2** |

#### 9.5.2 推荐（**含"保真 vs 速度"这个必须由用户拍板的取舍**）

**我的推荐是 R1（自研 CPU 射线 system）作为 P2 的主线，R0 作为开发/CI 的默认档，R3 只留作长期议题。**
理由按重要性排序：

1. **保真度是这个仓库的核心资产**：本项目已经有一整套"感知保真度"审计
   （`docs/tilted_lidar_fidelity.md`、`docs/session_digest.md`、`small_point_lio` 的变换 bug 审计）。
   R0/R3 都会改变点云的空间分布（R0 连方向都不对，R3 的距离是重采样的），
   **等于把已经校准好的感知基准作废**；R1 是唯一能保持"同一条射线打到同一个三角面"的路线。
2. **R1 的成本已经被量出来，而且不贵**：最坏（100 % 命中）单线程 29.4 ms/帧，4 线程 13.6 ms；
   标称位姿单线程 12.5 ms、4 线程 5.6 ms。**10 Hz 预算（100 ms）下的余量：单线程最坏 3.4×、
   标称 8.0×；4 线程 7.4–17.9×；8 线程 13.5–31×**，而且这是在一台
   **没有 GPU** 的机器上、用**纯 C++/Bullet** 得到的。
3. **R1 是纯加法**：一个 `gz-sim` system 插件（编译成 `.so`，在 world 里 `<plugin>` 加载）
   ⇒ 不动 `bringup_sim.launch.py` 默认值、不动 xacro、不动 `rm_perception/**`、不动 nav2 params，
   与 §6.11"保持不动清单"完全兼容。**不 fork 任何上游包**。
4. **R3 的收益（省 CPU）在本机并不稀缺**：server 侧的 30k `gpu_ray` 只吃 **24.7 %**（R0 世界，
   生命周期均值；P1-lite 的渲染期样本 ≈49 %），
   而 GUI 一开就吃掉 1112–1296 %。**真正的瓶颈是 GUI 的软件光栅化（11–13 核），不是射线**。R3 省的那点 CPU
   解决不了"无 GPU 机器上 GUI 贵 12 倍"这个主要矛盾。
5. **R1 顺带解掉一个架构耦合**：Classic 和 R0 都是"LiDAR ⇒ 渲染器 ⇒ 软件 GL"，渲染器起不来就没点云；
   R1 把这条依赖**彻底切断**（CI/无头回归再也不会因为 EGL/llvmpipe 挂掉而丢数据）。

**必须由用户拍板的取舍（我不替你决定）**：

| 取舍 | 选 A | 选 B |
|---|---|---|
| **保真 vs 速度** | **A：要 MID-360 的逐射线图案** ⇒ 走 **R1**（多写一个 system，约 1–2k 行；每帧多 0.5–13 ms CPU；换来感知基准可迁移） | **B：只要"有个 30k 点的 3D LiDAR"** ⇒ 走 **R0**（零成本，今天就能跑；但点云分布与真机不同，所有感知阈值/标定要重做） |
| **GUI vs 10 Hz 墙钟** | **A：GUI 必看** ⇒ 接受 RTF 0.82–0.88（40 s 的跑变 48 s），算法侧 `use_sim_time` 看到的仍是 10 Hz | **B：要 10 Hz 墙钟** ⇒ 关 GUI（headless），调试另开一台/另起一个 GUI 连同一 server |
| **长期 vs 近期** | **A：长期押 gz** ⇒ 值得为 R3 排一个 fork 的预算（2–3 包） | **B：先把迁移做完** ⇒ R3 只写进 roadmap，P2 用 R1 |

**一句话**：**如果"点云必须和现在一模一样"——答案是 R1，而且它划算；
如果"点云长得不一样也行"——R0 已经躺在 `.tmp_gzsim2/` 里了，今天就能用。
R3 是"两个都要"的选项，但它把成本换成了 fork + 重采样精度，不该压在 P2 上。**

### 9.6 诚实清单（本节**新增**的未验证项，接 §8）

| # | 未验证 | 原因 / 现状 |
|---|---|---|
| 1 | **R1 接进完整栈后的 RTF** | 本节只测了**射线求交**（12.5–29.4 ms/帧 @1 线程）。不含：从 gz-sim 抽世界几何、消息构造（480 KB + 960 KB/帧）、传输、以及 SLAM/LIO/nav2 的负载。**必须等 P4 的 A/B**。 |
| 2 | **动态障碍下的 BVH 成本** | 探针把世界 BVH **建一次**（22–24 ms）。移动障碍需要 refit/rebuild，**未测**（Fortress 的 `btBvhTriangleMeshShape` 支持 refit，但代价未知）。 |
| 3 | **"每周期多发 2.2–2.6 条消息"的机制** | 已排除：多 publisher、重复 stamp、射线数、`--headless-rendering`、窗口大小。**未定位**到具体代码路径（缺 `gz-sim6` 的 `Sensors.cc`，且本沙箱**无外网**：`raw.githubusercontent.com` 解析到非公网 IP，无法下载）。**只有比值可用**。 |
| 4 | **R3 的原型与重采样精度** | **没有写任何代码**。0.088°/texel 的量化、薄物漏检率、llvmpipe 下方向表的带宽，全部是**推算**，未实测。 |
| 5 | **`gz::rendering::RayQuery`（第一枪的另一个尝试）** | 它的 `us_per_ray=0.348 / engine_ms=117.3` 与 CPU BVH 的 0-hit 数字**同源**（mm/m 单位错），**同样无效**；本轮**没有重测**（需要 GL 上下文 + 用米制 mesh 重跑）。**不要引用那两个数字。** |
| 6 | **有 GPU 机器上的对照** | 本机无 `/dev/dri`、NVIDIA 驱动未加载 ⇒ 所有数字都是"无 GPU"的绝对值。有 GPU 时 R0/R3 的成本会大幅下降，R1（纯 CPU）**不变**——这反而让 R1 的相对优势在有 GPU 的机器上变小。 |
| 7 | **`gz-sim6` 的 `Sensors.cc` 源码** | 上一枪下载失败（0 字节，`.tmp_gzsim2/gzsim6_Sensors.cc`）；本节对 SDF 管道的分析只到 **schema 层**（`sdformat-9.7/sdf/Lidar.hh`）。 |
| 8 | **16 线程以上的伸缩** | 只测到 16 线程（28 个逻辑核）。E-core/P-core 混合拓扑下的最优线程数未系统扫。 |
| 9 | **`guismall` 的"窗口真的变小了吗"** | Xvfb 屏幕设成 640×480，但 `ign gazebo -g` 的窗口未必跟着缩（Qt 会 clamp/裁切）；**GUI CPU 没变（1112→1216 %）说明"按像素省钱"这条路本来就不通**，但"窗口实际多大"未取证。 |

---

## 附录 C 第二枪复现命令（全部在 `.tmp_gzsim2/`，**不动任何功能代码**）

```bash
cd /home/weicheng/HzMi_rmsimulation
D=.tmp_gzsim2
MESH=src/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/meshes/RMUC2026.stl
CSV=src/rm_simulation/livox_laser_simulation_RO2/scan_mode/mid360.csv

# ---- 0) 编译（R1 探针需要 ignition-common4-graphics 与 ignition-utils1） ----
g++ -O2 -std=c++17 -Wall $D/cpu_ray_bvh.cc -o $D/cpu_ray_bvh \
    $(pkg-config --cflags --libs ignition-common4 ignition-common4-graphics ignition-msgs8 ignition-utils1) \
    $(pkg-config --cflags --libs bullet) -lpthread
g++ -O2 -std=c++17 -Wall $D/hz_probe2.cc -o $D/hz_probe \
    $(pkg-config --cflags --libs ignition-msgs8 ignition-transport11 ignition-utils1) -lpthread

# ---- 1) R1：先看 sanity（不 PASS 就不会给计时） ----
RAY_REPS=3 $D/cpu_ray_bvh "$MESH" "$CSV" 30000 200.0 1 2 4 8 16
#   RAY_SCALE / RAY_ZOFF / RAY_POSE="x,y,z" / RAY_START（CSV 滚动窗口）/ RAY_ENCLOSED=1
#   RAY_DUMP=path 可导出逐射线命中距离，用于和 Python 暴力求交交叉验证

# ---- 2) R1：全场景 × 全线程（结果写 $D/r1_final.log） ----
$D/run_r1.sh

# ---- 3) R0：30k / 2880 × headless / GUI（会写 run_r0_*/ 并做 `ign stats`） ----
$D/run_r0.sh                      # 第一枪的脚本，产物在 $D/run_r0_*
#   观察点：stats.txt 的 real_time_factor、cpu.txt、hz_scan.txt / hz_points.txt / hz_clock.txt

# ---- 4) GUI vs 10 Hz：同世界同传感器，headless / GUI 1280x800 / GUI 640x480 ----
$D/run_rate.sh headless headless
$D/run_rate.sh gui      gui 1280x800
$D/run_rate.sh guismall gui 640x480
#   产物：rate_<label>/{hz_both,hz_clock,hz_stats,pub,cpu_samples}.txt
#   要点：server 与 GUI 分进程启动（set sid 进程组）以便精确归属 CPU；
#         每次跑独立 GZ_PARTITION；每段硬 timeout；只 kill 自己的进程组

# ---- 5) 位姿扫描（找"命中重/未命中重"场景；用 30k 行，别用 4k，CSV 是时间排序的） ----
$D/sweep_poses.sh                 # 2 m 网格，写 $D/r1_pose_sweep.csv

# ---- 6) 收尾检查（只查自己的进程；本沙箱是独立 PID namespace，看不到也不影响别人的进程） ----
pgrep -a -f "ign gazebo" ; pgrep -a -f Xvfb ; echo "leftovers: (none) 期望为空"
#   每个脚本都 set sid 起自己的进程组，exit trap 里 `kill -9 -<pgid>` 只杀自己那一组
```
