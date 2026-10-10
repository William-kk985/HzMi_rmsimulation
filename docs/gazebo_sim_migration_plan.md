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
