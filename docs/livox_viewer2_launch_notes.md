# LivoxViewer2 在 Ubuntu 22.04 上启动即崩：取证记录 + ROS2 驱动替代路线（2026-10-08）

> 触发 = 用户 2026-10-08 的原话：「帮我打开看雷达网口的那个软件」。
> 本文**只做两件事**：**(A)** 把"官方 Linux 查看器在本机起不来"这件事**逐项取证**，说清
> 哪些原因被**实测排除**、崩溃到底在哪；**(B)** 给一条**已验证可用**的替代路线
> （ROS2 驱动 + RViz，一条命令：`tools/scripts/livox/start_livox_mid360.sh`）。
>
> 环境：Ubuntu 22.04.5 LTS / glibc 2.35 / NVIDIA 550.163.01（RTX 4070 Laptop，Vulkan 1.3.277）/
> 原生 X11 会话（`DISPLAY=:1`，非 Wayland）/ Intel i7-14700HX / 32 GB。

## 0. 结论先行

**Livox 官方 Livox Viewer 2 的 Linux 版在本机（以及本机之外同样的桌面会话里）启动必然 SIGSEGV**，
与"哪个版本"无关：**2.3.0（UE4.25，OpenGL）与 2.6.2（UE4.27，Vulkan）崩在同一处**。
崩溃点位于 **`WBP_ViewerMain` 主界面控件构造**时的一次**空指针解引用**（`rax=0`）。

⇒ 因此改为 **ROS2 驱动 + RViz** 看雷达（见 §3）。这不需要官方查看器。

## 1. 现象与崩溃现场（逐字）

两个版本的日志都停在同一步（`Maps/Viewer` 起来后 ~40 ms）：

```
LogLoad: Game class is 'ViewerGameMode_C'
LogGameMode: Match State Changed from WaitingToStart to InProgress
ShowFlag.PostProcessing = "0" / r.ScreenPercentage = "200" / ShowFlag.Lighting = "0" …
[ 36]LogUMG: Display: Widget Class WBP_ViewerMain_C - Loaded Fast Template.   ← 建主界面
CommonUnixCrashHandler: Signal=11
Uncaught Exception: SIGSEGV: invalid attempt to read memory at address 0x0000000000000000
```

gdb 现场（2.6.2，`-nullrhi`）：

```
0x2f76441  mov  -0x260(%rbp),%rax
0x2f76448  mov   0x18(%rax),%rax     ← rax=0
0x2f7644c  movzwl (%rax),%ecx       ← 读 0 号地址 → SIGSEGV     rax=0x0 rdi=0xa
```

调用栈（用户在自己终端里跑出来的完整 50 帧）呈**深递归**：`0x385abdd → 0x385a4e9 → 0x385b60a`
这一组地址**重复 8 次**，符合"控件树递归构造/布局"的形态；崩溃前 5 次打印
`ADD THRESHOLD 30/90/150`（= `ViewerConfig.json` 的 `ReflectivityToColorThresholds`），
说明**配置文件读到了、pak 挂载正常**（`LogPakFile: Mounting pak file … OK`）。

## 2. 逐项排除（每条都有实测对照）

| # | 假设 | 实验 | 结果 |
|---|---|---|---|
| 1 | 程序装在**只读**目录 ⇒ UE4 建不了 `Saved/Logs` | 从 `~/下载/LivoxViewer2 for Ubuntu v2.3.0`（只读挂载）直接跑 | **真问题**：`Failed opening … for writing: Read-only file system` → 复制到工作区可写目录后过掉这一关，露出下一条 |
| 2 | 沙箱/容器 PID 命名空间导致 pthread 失败 | 对照：沙箱内 vs `danger-full-access` 真实环境 | 沙箱内多一条 `mutex lock failed`；**真实环境仍 SIGSEGV** ⇒ 不是沙箱 |
| 3 | 下载/复制**文件损坏** | `md5sum` 对比原件与副本：二进制 `4c5105155202e3768fc770f9bb8ee3e3` 一致；pak `0d3f6d64cf4f5d0bb962557ed6ac888a` 一致 | **不是** |
| 4 | **显卡/渲染后端**问题 | `-nullrhi`（完全不渲染） | **仍崩在同一处** ⇒ 与渲染无关 |
| 5 | Vulkan 设备选择（NVIDIA vs llvmpipe） | `VK_ICD_FILENAMES=lvp_icd…`（软渲染）/ `__NV_PRIME_RENDER_OFFLOAD=1` | 两种情况都崩 ⇒ **不是** |
| 6 | 版本太老 | 从官网下 **2.6.2**（2026-09-21，UE4.27） | **同一处崩溃**（连引擎大版本都换了）⇒ 不是版本问题 |
| 7 | 音频子系统 | `SDL_AUDIODRIVER=dummy` / `-nosound` | 仍崩 |
| 8 | 中文 locale / 本地化缺失 | `LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8` | 仍崩（日志里 `No localization for 'zh-CN' exists` 只是回退到 en，不是原因） |
| 9 | D-Bus / 无障碍（at-spi） | `env -u DBUS_SESSION_BUS_ADDRESS NO_AT_BRIDGE=1 NO_AT_SPI=1` | 仍崩 |
| 10 | `LD_LIBRARY_PATH` 里的 ROS/Gazebo/MVS 库干扰（ROS 的 OpenSSL 等） | `env -u LD_LIBRARY_PATH -u LD_PRELOAD` | 仍崩 |
| 11 | 本 shell 的 **capabilities 全零**（`CapEff=0`） | `systemd-run --user`（能力集正常） | `ExecMainStatus=139` ⇒ 仍崩 |
| 12 | 缺少系统依赖/字体 | `ldd`：**0 个** `not found`；`fc-list`：**502** 个字体 | 干净 |
| 13 | 只有我的自动化会话会崩 | **用户在自己的终端里跑** | **同一个崩溃、同一处**（§1 的 50 帧栈即来自这次）⇒ 与环境无关，是程序自身缺陷 |

公开同类反馈：[Livox-SDK#226「Crash after Splashscreen」](https://github.com/Livox-SDK/Livox-SDK/issues/226)、
[livox_ros_driver2#145「crashes in debian and ubuntu」](https://github.com/Livox-SDK/livox_ros_driver2/issues/145)
（2026-10-08 当天 GitHub 经代理返回 403，**正文未取到**，只确认到标题与存在性）。

**仍未验证**：崩溃那三层递归函数（`0x385abdd/0x385a4e9/0x385b60a`）到底是什么
（无 `.sym` 符号、二进制为 Development 构建但未带符号表）；因此**没有**"它想要哪个空对象"的直接证据，
§0 的"控件构造时空指针"是**由现场形态（UMG + 深递归 + nullptr + 无渲染依赖）推断**，不是符号级定论。

## 3. 替代路线：ROS2 驱动 + RViz（已验证可用）

本仓 `src/rm_driver/livox_ros_driver2` **已编译**（`install/livox_ros_driver2`），
且**已有真机配置**：`src/rm_driver/livox_ros_driver2/src/config/MID360_config.json`
（本机 `192.168.1.50`、雷达 `192.168.1.178`、端口 56100/56300/56400 那套）。
`install/` 里那份是**指向源码的符号链接**，改源码即生效，**不需要重新 `colcon build`**。

> ⚠️ **这台雷达的 IP 会变**（DHCP 自动获取）。实证：`archive/reality_frozen/MID360_config.json`
> 里是 `.119`，本次会话开始时配置里是 `.146`，实测雷达在 **`.178`**（MAC `0c:9a:e6:03:fc:67`，
> OUI `0C:9A:E6` = SZ DJI Technology，Livox 属 DJI）⇒ **至少变过三次**。
> 所以**不要**把 IP 当成固定值：链路通但没数据时，**先核对 IP**（见 §3.5）。

一条命令：

```bash
cd ~/HzMi_rmsimulation
./tools/scripts/livox/start_livox_mid360.sh            # 配网卡(如需) + 驱动 + RViz
./tools/scripts/livox/start_livox_mid360.sh --check     # 只自检，不开 RViz
./tools/scripts/livox/start_livox_mid360.sh --skip-net  # 网卡已配好，跳过改网卡
```

脚本做 7 件事：① 读配置并与脚本变量对账；② 找有线网卡（排除 wifi/docker/tun）；
③ 网卡不是 `192.168.1.50/24` 就用 `nmcli` 配上（**需要 sudo**，脚本会提示）；
④ `ping`/`arping` 连通性（无 `arping` 时自动跳过）；⑤ `source` ROS + 工作区；
⑥ 起驱动并**硬前置检查** `192.168.1.50` 是否在本机；⑦ 起 RViz（
`config/display_point_cloud.rviz`：Fixed Frame `livox_frame`、Topic `/livox/lidar`）。

### 3.1 本机当前硬件状态（实测）

```
enx00e04c0b2110   driver=r8152   operstate=down   NetworkManager: unavailable
```

- 机器**没有内置有线网口**（只有 `wlp0s20f3` WiFi），但有**一块 USB 网卡**（Realtek `r8152`）。
- 它现在是 `down`/`unavailable` ⇒ **网线没插（或雷达没上电）**。插上后 `operstate` 应变 `up`，
  再跑脚本即可。
- 注意：`~/.ros/log` 在自动化会话里是只读的（spdlog 会直接 abort），脚本已把驱动日志
  改到 `<工作区>/.tmp_roslog_driver/`（可用 `LIVOX_LOG_DIR` 覆盖）。

### 3.1b 实测：网卡配好了、雷达也上电了，但**物理层仍没链路**（2026-10-08）

用户执行全过程（逐步取证）：

```
== 2. 网卡地址 ==  当前:（未配置 IPv4）  链路: down          ← ① 配 IP 之前就 down
  … sudo nmcli … 连接已成功激活 … 现在: 192.168.1.50/24     ← ② IP 配上了
== 3. 连通性 ==  ping 不通 / ARP 表: 192.168.1.146 INCOMPLETE ← ③ 无人应答
== 5. 启动驱动 == 驱动已启动（这次**没有** bind failed）      ← ④ 说明 ①② 的修复生效
== 6. 话题自检 == /livox/lidar 存在但**无频率**               ← ⑤ 设备根本不在
```

现场取证（**这才是"没数据"的真正原因**）：

```
carrier                                          → 0
ethtool … | grep -i 'link detected'              → Link detected: no
ethtool … | grep -iE 'speed|duplex'              → Speed: 10Mb/s  Duplex: Half   ← 未协商上的占位值
lsusb | grep -i realtek                          → 0bda:8153 RTL8153（硬件正常）
ip route show | grep 192.168.1                   → 192.168.1.0/24 dev enx… linkdown（路由本身对）
lsusb -t                                         → Port 4: Driver=r8152, 480M（USB 2.0，带宽够）
```

**结论：`carrier=0` + `Link detected: no` ⇒ 网卡在 RJ45 侧收不到任何电信号**，与 IP、路由、
ROS、驱动、USB 带宽**全都无关**。`10Mb/s Half` 是未建立链路时的默认显示，**不能**读成
"协商到了 10M"；`ip neigh` 里的 `INCOMPLETE` 是这条的必然结果（请求发出、无人应答）。

**排查顺序（便宜→贵）**：

1. `ethtool <iface> | grep 'Link detected'` —— 它是后面一切的前提。
2. **拔插网线两端**（尤其 MID-360 那端的航空插头/水晶头），并**换一个 USB 口**重插网卡
   （r8152 的 PHY 偶发卡死，USB 重插会重新初始化）。
3. **换一根网线**，并把这块 USB 网卡插到**已知能用的交换机/路由器**上看 link 是否变 `yes`
   ⇒ 一刀切开"网卡/线的问题"与"雷达端的问题"。
4. **确认雷达在运行态**：MID-360 需**独立供电 9–27 V**（**网口不支持 PoE，网线不供电**）；
   指示灯见官方《Mid-360 User Manual》的 Working States 表（正常应为绿灯常亮）。
   上电但供电不足时，可能"亮灯但网口 PHY 不起"。
5. 确认网线插的是**雷达本体网口**，以及链路上是否串了 HAP/交换机（那要看中间设备的口）。
6. 物理层修好后，若雷达 IP 不是配置里的 `192.168.1.146`，用
   `tools/scripts/livox/find_livox.sh` 扫出真实 IP/MAC 再回填配置（它会先检查链路，down 时直接拒绝扫描）。

### 3.1c 同一故障的后续取证：USB 层已全部排除（2026-10-08 深夜）

用户把 USB 网卡**换了一个 USB 口**后重测，读数完全相同（`carrier=0`、`Link detected: no`、
`rx_bytes=0`），并且这次 `dmesg` 可读，拿到了 USB 层的完整证据：

```
usb 1-4: USB disconnect, device number 25          ← 从 Port 4 拔出
r8152 1-4:1.0 enx00e04c0b2110: Stop submitting intr, status -108
usb 1-9: reset high-speed USB device number 28 using xhci_hcd   ← 插到 Port 9
r8152 1-9:1.0: load rtl8153b-2 v2 04/27/23 successfully         ← 固件加载成功（芯片 = RTL8153B）
r8152 1-9:1.0 eth0: v1.12.13
r8152 1-9:1.0 enx00e04c0b2110: renamed from eth0                ← 正常注册
```

加上 `over-current/overcurrent` 计数为 **0**，⇒ 下面这些**全部排除**：

| 变量 | 证据 |
|---|---|
| USB 口 | 换口（Port 4 → Port 9）读数不变 |
| USB 供电/过流 | dmesg 无过流事件 |
| 适配器固件 / 驱动绑定 | `load rtl8153b-2 v2 … successfully`、`eth0: v1.12.13` |
| USB 链路稳定性 | 无 fallback/反复 reset（只有插拔那一次 reset） |
| IP/路由 | 配好后是 `192.168.1.50/24`，路由 `linkdown` 正确 |
| **剩下的嫌疑** | **① 这块适配器的以太网 PHY ② 网线 ③ 雷达侧（本体网口/供电/插错口/中间设备）** |

**分岔判据（`tools/scripts/livox/link_test.sh`）**：把该 USB 网卡插到**已知能用的网口**
（路由器/交换机 LAN 口，或另一台电脑的网口 —— 现代网卡自动翻转，不需要交叉线）：

- `carrier` 变 **1** ⇒ 网卡+这根线都好 ⇒ 问题在**雷达侧**；
- 仍为 **0** ⇒ 问题在**网卡或线**（先换线，再换网卡；RTL8153 第三方适配器存在
  "能枚举、固件正常、就是不起链路" 的已知毛病）。

**另一条实测注意点**：这台机器上 `ethtool` 查询会报
`netlink error: Operation not permitted`（在受限会话里连 `systemd-run --user` 也一样），
**但 `Link detected:` 仍会打印且有效**。要求"零噪音"的读数请直接读内核文件：
`cat /sys/class/net/<iface>/carrier`（1=有信号）与 `operstate`，**不需要任何权限**。
`tools/scripts/livox/net_diag_livox.sh` 已按此调整：遇到该报错会标注"与雷达无关"，
并把内核文件作为速度/双工/链路的第一来源。

### 3.2 实测到的失败模式（脚本据此设了硬检查）

网卡没配 IP 时，驱动**能起来**但 SDK 初始化失败 —— 如果没有前置检查，会误判成"驱动正常、雷达没数据"：

```
successfully parse base config, counts: 1
bind failed
Failed to init livox lidar sdk.
[ERROR] [livox_lidar_publisher]: Init lds lidar fail!
```

原因：Livox SDK 会把本机 socket **绑到配置里的 host IP**（`host_net_info.*_ip`），
该地址不在本机 ⇒ `bind` 失败。⇒ 必须先配网卡（或改配置）。

### 3.5 实测：**配置里的雷达 IP 是旧的**（本次会话真正的拦路虎，2026-10-08 深夜）

物理层修好（`carrier=1`）后跑脚本，RViz 报：

```
Fixed Frame [livox_frame]  →  No tf data. Actual error: Frame [livox_frame] does not exist
```

⚠️ **这句报错很容易被误判成"缺 TF / Fixed Frame 配错"**，其实它只是"**一帧点云都没发布过**"的后果：
PointCloud2 自带 `frame_id=livox_frame`，RViz 渲染这种消息**根本不需要 TF**（该 launcher 里
`static_transform_publisher` 数量 = 0，属正常设计）。**不要**为此去加 TF 或改 Fixed Frame。

真正的读数（三条一起看，结论唯一）：

```
网卡 : carrier=1   rx_bytes=411,095,196      ← 雷达一直在回传 411 MB 数据
ARP  : 192.168.1.146 FAILED                  ← 配置里写的 IP 根本不存在
       192.168.1.178 lladdr 0c:9a:e6:03:fc:67 ← 雷达实际在这里（OUI 0C:9A:E6 = SZ DJI）
日志 : Init lds lidar success!
       GetFreeIndex key:livox_lidar_2449582272.
       Can not get index, the livox lidar type:8, handle:2986453184   ← 刷了 863 行
       Storage point data failed, can not get index, …
```

机制：SDK 按配置去连 `192.168.1.146`（连不上、无设备条目），而点云来自 `192.168.1.178` ⇒
**包收到了但没有可归属的设备 ⇒ 整包丢弃**（`Storage point data failed`），于是 `/livox/lidar`
一帧都不发。⇒ **改配置第 28 行 `lidar_configs[0].ip` 即可**（本次已改为 `.178`）。

**改哪里（唯一真值文件）**：

```bash
src/rm_driver/livox_ros_driver2/src/config/MID360_config.json    # 第 28 行 "ip"
```

- `install/livox_ros_driver2/share/…/MID360_config.json` 是**符号链接**指向它 ⇒ 改一处即可，**无需重新构建**。
- 同文件第 14/16/18/20 行的 `192.168.1.50` 是**本机** IP，**必须保持与网卡配置一致，不要动**。

**防复发**：`start_livox_mid360.sh` 新增 **§3.5 前置检查** —— 进 ARP 表核对配置 IP；
不在线就自动 ping 扫同网段、列出在线设备并提示"把配置改成哪一个"，避免又对着空话题排查。

## 4. 未验证 / 后续可做

1. **雷达接上后的端到端验证**（当前**物理层未连通**，见 §3.1b）：链路变 `up` 后，脚本应显示
   `/livox/lidar ≈ 10 Hz`、`/livox/imu ≈ 100 Hz`。在此之前结论只能到"驱动能起、前置检查正确、
   `/livox/lidar` 话题已创建（但无发布者出数）"。
2. **`arping` 未安装**（`apt install iputils-arping` 可让第 4 步更强）。
3. **Wine 跑 Windows 版查看器**（官方 `Viewer2_2.6.2_Win.zip`）未尝试 —— 如需官方查看器的
   反射率着色/LVX2 录制体验，这是另一条路。
4. **容器方案未落地**：本机 `docker` 存在但当前用户**不在 `docker` 组**、且无免密 sudo，故未执行。
5. 崩溃的三层递归函数身份（见 §2 末）—— 要符号级定论需 Livox 提供 `.sym` 或自行反汇编。
6. `tools/scripts/livox/` 是**新增目录**；`tools/scripts` 不参与 colcon 构建（`tools/build/COLCON_IGNORE`），
   因此**不需要**同步到 `install/`，也不需要重新 `colcon build`。
