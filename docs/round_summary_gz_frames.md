# 本轮总结（2026-10-18）：外参守卫 · 机器卫生 · Fortress 后端 · COD 帧架构对账

> **状态**：四条线**全部完成**，本文没有"进行中"的条目。
>
> **写作口径**：**只汇总、不重算**——每个数字后面都挂着原文档的 `§` 或 commit。原文与本文
> 若有冲突，以原文的实测记录为准；已经发现的 **4 处口径冲突**逐条写在 **§4.0**，并说明取了哪一版、为什么。
>
> **证据分级**（全文逐条标注）：
> - **【实测】** 本轮真跑出来的（日志 / 探针 / bag 重放 / 命令行输出）；
> - **【推断】** 读源码、几何或文档推出来的，本轮**没有**数据；
> - **【未验证】** 没做，**不要当结论用**（汇总在 §4）。
>
> **覆盖提交**：
> `fe0dc31`（LIO 外参守卫）· `a137bf2`（机器卫生 runbook + 14 个脚本的收尾作用域）·
> `9152d90`（gz 文档收尾；gz 的**代码与主文档**实际随 `057294c` 一并入库）·
> `da5c2fe`（`model.config` 资产修正）。
>
> **主要来源**：`docs/tilted_lidar_fidelity.md` §O · `docs/machine_hygiene_runbook.md` ·
> `docs/fortress_sim_backend.md` · `docs/gazebo_sim_migration_plan.md` §11 ·
> `docs/cod_vs_ours_frame_architecture.md`。

---

## 0. 一句话结论

**这轮把四件事各自收口，且没有一件是靠"应该没问题"收的。**

| 线 | 一句话 | 现在能不能用 |
|---|---|---|
| **L1 外参守卫** | `small_point_lio` 的两个 latent 变换 bug（共轭 / 双重变换）从"文档里的设计"变成**开机可见 + 每帧可查**：外参有旋转会 WARN/ERROR，等式被破坏会限速 ERROR；**只诊断、不停机、不改任何数值**【实测】 | ✅ 能用；默认行为逐位不变 |
| **L2 机器卫生** | 把"swap 快满了"这个传闻**证伪**（真的没满），把真风险定位到 **oomd 整组杀**；并把 14 个脚本的收尾从"全机 pkill"改成"只杀自己这次起的"【实测】 | ✅ 能用；剩 `start_sentinel.sh` 一处 |
| **L3 Fortress 后端** | 多了一条 opt-in 的 `sim_backend:=gz`（**Classic 仍是默认、默认路径逐字节不变**），传感器/话题/帧/QoS 契约对齐【实测】 | ⚠️ **半可用**：世界/点云/`/scan` 能跑，**LIO 在 gz 侧不出 odom** |
| **L4 COD 帧对账** | 更正了一个**前提性错误**（COD 用的就是我们那份 `small_point_lio`），判决为 **(b) 约定分叉 + 6 条可避免耦合（其中 4 条明确该修）**，给出 C/B/A 三选项【实测源码 + 推断】 | ✅ 结论可用；选项 C 未实施（等拍板） |

**现在能不能用**（按用途分）：

| 用途 | 能不能 | 依据 |
|---|---|---|
| Classic 默认路径（跑仿真、LIO、nav2、建图） | ✅ **能**，且与改动前**逐字节相同** | launch 增量 `+2/−0`（`fortress_sim_backend.md` §9）；LIO 默认档重放 **0 差异**（`tilted_lidar_fidelity.md` §O.4a） |
| gz 后端做**传感器/感知/点云**验收 | ✅ 能（10.0000 Hz、RTF 0.97、`/scan` 通、自机排除生效） | `fortress_sim_backend.md` §5/§6/§8 |
| gz 后端做**定位/建图闭环** | ❌ **不能**：`small_point_lio` 持续 `imu loop back`、不发 `odom→base_link` | `fortress_sim_backend.md` §8.1 |
| gz 后端跑**移动**闭环（nav2 控制） | ❌ **不能**：底盘在 SDF 里是 `<static>`，`/cmd_vel_chassis` 无人消费 | `fortress_sim_backend.md` §10 第 1 条 |
| 真机 | — 本轮**没碰**（无 MID-360、无实车） | `tilted_lidar_fidelity.md` §O.5 第 1 条 |

**还差什么**（全部展开在 §4）：gz 侧 LIO（首要）· gz 底盘运动 · 同轮 Classic A/B · 真机 `R_bl` 来源 ·
COD 的 `urdf/` 本地副本 · `robot11_mount:=urdf` 去留（等拍板）。

---

## 1. 四条线 + 状态表

| 线 | 做了什么 | commit | 关键实测数字 | 剩余缺口 | 状态 |
|---|---|---|---|---|---|
| **L1 LIO 前提断言** | 新增独立 TU `extrinsic_guard.cpp`：启动期外参 readout（#1）+ 每帧 `T_ob·T_bl⁻¹≈T_ol` 断言（#2）；单测第 6 条 | `fe0dc31` | 成本 **24.4 ns/帧**（+~5 ns 拷贝）；默认档重放 **30/30、31/31、31/31 全 0 差异**（第二袋 33/34/34）；斜装档启动 ERROR `rpy=[30.000000,-0.000000,0.000000]°`、`\|t\|=0.204443 m`；注入共轭 ⇒ `0.207297637 m / 30.000000°`、计数 **1→15→29**（间隔 5.0–5.2 s）；反向对照 **0 违例**；假俯仰 **0.082°**（修前 4.890°）；最大平面角 **0.046/0.367/0.420°**（bug② 预测 29.977°） | `1.05°` 绝对角未复现（世界/出生点已变，§4 U7） | ✅ 完成 |
| **L2 机器卫生** | swap/zram/oomd 体检 runbook（逐条标 已执行/需 root/未执行）+ 新增 `scoped_cleanup.sh` + **14 个脚本**收尾作用域化 | `a137bf2` | swap **194 / 3901 MiB**（测试前 `pswpout` 仅 **652 页 ≈2.5 MB**）；`vm.swappiness=60`；zram **不可用**；oomd `kill`+`Limit=50%`（曾杀 **25 个进程**）；历史全局 OOM `async_slam_tool` **11.85 GB**；decoy **10 个** PASS + **3 真脚本 ×5 decoy** PASS；`pkill -9 -x gz` **18 行 → 0** | `start_sentinel.sh` 仍是全机 `pkill -f ros` 等 4 行 | ✅ 完成 |
| **L3 Fortress 生产化** | `fetch_and_patch_mid360.sh`（pin 上游、幂等、不 vendor）+ 新包 `rm_livox_gz_sim` + `sim_backend:=classic\|gz`（默认 classic） | `9152d90`（内容随 `057294c`） | 仿真 **10.0000 Hz** / 墙钟 **9.72 Hz** / RTF **0.97**；中位 **8018** 点；`offset_time` 最大 **99,996,667 ns** 单调；求交 **0.325 ms/实例 ⇒ ≈0.65 ms/帧**；双实例 BVH **建一次**、**313,721 点 atol=0**；RSS **328/441/652 MB**；首帧 **3.00 s**；`/scan` **418 束 2.75–8.09 m** `frame_id=base_link`；`--strip-conditions` **+2/−0** | gz 侧 LIO 未通过；底盘 `<static>`；无同轮 A/B | ✅ 完成（opt-in，功能有边界） |
| **L4 COD 帧架构对账** | 只读源码取证，出对账文档：H1/H2/H3 + 可避免耦合 O1–O6 + 三选项 | `057294c` | COD `obstacle_layer` 命中 **0/0**、`/scan` 命中 **0**；`small_point_lio` 与上游 `688d75c` **sha256 逐字节相同**；我们与 COD 的 vendored launch **唯一差异** = `:47` 多一个节点；`lio_tf_adapter` 被 **3 处** `!= 'small_point_lio'` 排除 | COD 本地无 `urdf/` ⇒ `T_bl≈I` 是推断 | ✅ 完成 |

### 1.1 L1：守卫具体落了什么（`fe0dc31`）

**文件与插入点**（全部有 `file:line`，见 `tilted_lidar_fidelity.md` §O.1）：

| 项 | 位置 | 内容 |
|---|---|---|
| 阈值 | `src/rm_localization/small_point_lio/src/extrinsic_guard.cpp:36-52` | WARN `1e-3 rad`（≈0.057°）、ERROR `0.05 rad`（≈2.9°）、旋转容差 `1e-6 rad`、平移容差 `1e-4 m` |
| #1 启动 readout | 同上 `:66-109` | `rpy(T_bl)`（度）+ `\|t\|`，**每次启动恰好 1 条**，级别按 `max\|rpy\|` 取 INFO/WARN/ERROR |
| #2 每帧断言 | 同上 `:131-176`（违例分支起 `:157`，计数 `:158`） | `T_ob·T_bl⁻¹` vs `T_ol`；**只计数 + 限速 ERROR（1 条/5 s，墙钟 `steady_clock`）** |
| #2 取数点 | `small_point_lio_node.cpp:193-194` | 14 个 double 的 POD 快照（odom 回调末，**不引入任何新的浮点表达式**） |
| #2 检查点 | `small_point_lio_node.cpp:208-209` | 点云回调首、`get_subscription_count()` 分支**之外** ⇒ 有没有订阅者都查 |
| 构建 | `src/rm_localization/small_point_lio/CMakeLists.txt:74-78` | 显式 `list(APPEND SOURCE src/extrinsic_guard.cpp)`（GLOB 在 configure 时求值，不显式列会在**已有 build/** 的工作区里 undefined symbol） |
| 单测第 6 条 | `tools/scripts/tiltmount/tf2_compose_order_test.cpp:155-201` | ⑥a/⑥b/⑥c |

**为什么必须独立 TU（本轮最反直觉的一处，实测逼出来的）**【实测】：

| 守卫放法（同一输入流 bag 重放，30 条 `/cloud_registered` / 31 条 `/odom` / 31 条 TF） | `/cloud_registered` | `/odom` + `odom→base_link` |
|---|---|---|
| 内联在 odom 回调里 | 30/30 逐位相同 | **31/31 有差，最大 1.11e-16（1 ulp）** |
| 同 TU、但在点云回调里调用（odom 回调只留 14 个 double 拷贝） | 逐位相同 | **31/31 有差，最大 1.11e-16** |
| **独立 TU `extrinsic_guard.cpp` + 同一取数点** | **逐位相同** | **逐位相同（0 差异）** |
| 语义中性对照：只加一行整数自增 | 逐位相同 | **0 差异** |
| 语义中性对照：只加 14 个 double 成员拷贝 | 逐位相同 | **0 差异** |

读法：不是"任何改动都会差"，而是**这段浮点代码在 odom 回调所在 TU 里就会让 GCC 的 IPA/内联预算变**
（本包 `-ffast-math -march=native`，`tf2::Transform::operator*` 的 FMA 收缩随之变）⇒ 1 ulp。
反过来，"守卫只诊断、不动数值"是**可逐位验证**的，不是承诺。

**平移为什么用"双合法值白名单"而不是严格断言**：本仓按 §J.5 **有意**保留旧平移 ⇒ 等式在平移上
**永远**不成立（与合成真值差 `|t_pub−t_composed| = 0.204267 m`）。所以判据是
`min(|t_pub−t_composed|, |t_pub−t_legacy|) <= 1e-4 m`：两条合法值都由本仓代码显式算出，
**谁写了第三套平移公式**才会触发。

**成本**：独立微基准 2e6 次、非恒定输入、`-O3 -march=native -ffast-math` ⇒ **24.4 ns/次**（两次 24.4/24.7）；
加 odom 回调里 14 个 double 的拷贝（~5 ns），量级 **<0.1 µs/帧**，相对点云回调（几十 µs 级）可忽略。
**无新 TF 查询、无新订阅/定时器/线程**（§N.5 明确反对"在点云回调里再查一次 TF"——那正是 bug ② 的形状）。

**三段回归**【实测】：

| 段 | 判据 | 实测 |
|---|---|---|
| (a) 默认档 `plugin`（`R_bl=I`） | 同一输入流重放，改动前 vs 改动后 | 袋1：`/cloud_registered` 30/30、`/odom` 31/31、`/tf` 31/31 **全 0 差异**；袋2（最终二进制）：33/34/34 **全 0 差异**；差异只有多出来的那条 INFO 守卫日志 |
| (b) 斜装档 `robot11_mount:=urdf`（`T_bl=Rx(30°)`） | 守卫**真的会喊** | 启动 ERROR `rpy=[30.000000, -0.000000, 0.000000]° (=0.523599 rad)、\|t\|=0.204443 m`；故障注入（把合成改回共轭）⇒ 3 条限速 ERROR，平移 `0.207297637 m`、旋转 `30.000000°`，计数 **1→15→29**，间隔 5.0–5.2 s；**反向对照**（正确代码 + 同一 30° 外参）⇒ **0 条违例** ⇒ 守卫不是"见 30° 就喊"，是"等式被破坏才喊" |
| (c) 两个老 bug 仍修好 | 同一次 `gq_urdf` live 跑的 `probe.json` | 假俯仰 `-0.082°`（**<0.5°**，修前 4.890°）；`/cloud_registered` 最大平面角 `0.046/0.367/0.420°`，而 bug ② 假设应为 **29.977°**；Kabsch(raw→published) 旋转 **0.087°**、平移 `[-0.0069, 0.0031, 0.0504] m` |

**⚠️ 一处诚实偏离**：§J 那轮的"地面倾角 ≈1.054°"本轮**没有复现**。原因已查明：本轮最大平面到传感器
的高度 `d = 0.103 m`，而 §J 那轮是 `d = 0.26 m`（真地面）；本轮出生朝向真值 yaw **0.01°**、§J 是 **10.12°**
⇒ **两次跑的"最大平面"不是同一块面、出生姿态也不同**（世界/出生点在 10-09 之后被改过；本轮未动
`src/rm_simulation/**`）。所以本轮**不主张"绝对角 = 1.05°"**，只主张**判决性签名**：bug ② 的 30° 签名不存在。

### 1.2 L2：机器卫生与收尾作用域（`a137bf2`）

**结论反转（第一件事先把传闻证伪）**【实测】：

| 之前的说法 | 本轮实测 | 判决 |
|---|---|---|
| "`SwapFree ≈ 1 MiB`，swap 快满了" | swap **4095 MiB 总 / 194 MiB 已用 / 3901 MiB 空闲**；`MemAvailable` **16.8 GiB** | **没耗尽** |
| 同上 | 本次开机（10-16 16:06 起）在测试**之前** `pswpout` 只有 **652 页 ≈ 2.5 MB**；若这次开机真被换满 4 GB，`pswpout` 会是 **~100 万页** 量级 | ⇒ 那个快照属于 **上一次开机**（换页统计随重启清零），**已被取代** |

**swap 里装的是谁**（cgroup `memory.swap.current` 口径，合计 ≈194 MB，与 `free` 吻合）：
firefox **102.35 MB** · Chrome **34.98 MB** · gnome-software **21.27 MB** · packagekit **9.65 MB** ·
VS Code 1.99 MB …；顶层 `user.slice` **181.5 MB**、`system.slice` **9.9 MB** ⇒ **是桌面应用，不是 gazebo**。

**系统面**：`vm.swappiness = 60`（默认）；zram **不可用**（`/dev/zram*` 不存在、模块未加载、无 `zram-generator`；
工具与模块都在）；`sudo` 在沙箱里被 `no-new-privs` 挡死 ⇒ **所有需 root 的命令一条都没执行**。

**真风险不是"swap 满"，是"oomd 整组杀"**【实测 + 历史日志】：

| 日期 | 谁杀的 | 判据 | 被杀单位 |
|---|---|---|---|
| 10-01 21:44 | **systemd-oomd（用户态）** | user slice PSI **72.74% > 50%** 持续 **20 s** 且有 reclaim | 整个 `vte-spawn-*.scope`（**25 个进程**，含终端里所有 job） |
| 10-06 11:30 | **内核 OOM killer**（`global_oom`，调用者是 oomd） | 全机真的分配不出页（`async_slam_tool` 单进程 anon-rss **11.85 GB**） | 单进程 `async_slam_tool` |

"50%" 的来源是发行版 drop-in `/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf`
（`ManagedOOMMemoryPressure=kill` + `ManagedOOMMemoryPressureLimit=50%`，`systemctl show` 里显示成
`2147483648`），**不是** `/etc/systemd/oomd.conf` 的 60%（那份只有注释）。

**脚本收尾的作用域改造**【实测】：

* 新增 `tools/scripts/lib/scoped_cleanup.sh`（205 行）。三条判据，**任一**成立才算"自己的"：
  ① `setsid` 进程组（`PGID=$!`，`kill -INT -- "-$PGID"`）；
  ② `/proc/<pid>/environ` 里 `GAZEBO_MASTER_URI` 与本 run **逐字相等**；
  ③ PID 是 `$$` / `$LPID` / `$LAUNCH_PID` 的**后代**。
  名字/命令行**只用来挑候选**，绝不单独作为杀的理由；自带自伤保护（`$$` 及所有祖先永不杀）。
* **14 个脚本**接进去；`run_robot_model_probe.sh` / `robot11_gui_bench.sh` 原来没有 `setsid`，这次补上。
* 删掉一类"看着像保护、其实永远匹配不到"的写法：`pkill -f "gzserver.*$PORT"`（gzserver 的 cmdline 里
  **没有端口**，端口在 `GAZEBO_MASTER_URI` 环境变量里）⇒ 那是**空操作**。
* 证据：10 个合成 decoy 的判定矩阵 **PASS**（别人的 gzserver/gzclient/launch/slam 全活、本 URI 的全死、
  进程组兜底生效、自伤保护生效）；3 个**真脚本** × 5 decoy **PASS**（`run_nav_smoke_regress`、
  `run_remap_lowterrain`、`run_robot_model_probe` 连跑 3 次）；跑前残留清理也只清本 URI 的。
* 最终 `grep -rn 'pkill -9 -x gz' tools/` = **0**（改前 **18 行**）。

**剩余风险（明确留着）**：`tools/scripts/control/start_sentinel.sh:165/166/169/172` 仍是
`pkill -f gzserver` / `gzclient` / **`pkill -f ros`** / `rviz2` —— 其中 `pkill -f ros` 会杀掉机器上
**所有** ROS 进程（包括别人的）。

### 1.3 L3：Fortress 生产化（`9152d90`，内容随 `057294c`）

**交付物**：

* `tools/scripts/gzsim/fetch_and_patch_mid360.sh`：取三个上游并打补丁，pin 死在
  **rmagine `2.4.3`** / **rmagine_gazebo_plugins `c6dc04e`** / **mid360_gazebo_harmonic `497149a`**，
  幂等，编到独立前缀（默认 `$REPO/.tmp_gzlivox2/gzsim_ws`，`GZSIM_PREFIX` 可覆盖），
  **不 vendor 第三方代码**、不装进 `/opt/ros`、不碰系统库。
* 新包 `src/rm_simulation/rm_livox_gz_sim/`（launch / worlds / models / scripts / config）。
* `bringup_sim.launch.py` 的 `sim_backend:=classic|gz`（**默认 `classic`**）。

**默认路径零改动的证据**【实测】（`fortress_sim_backend.md` §9，工具 `launch_increment_diff.py` 比较**整棵
Action 树**的每个字段 + condition + 嵌套子 Action）：

| 口径 | 结果 |
|---|---|
| Action 数量 | 旧 **61** → 新 **63** |
| 变化点 | **3 处**：① 新增 `DeclareLaunchArgument name=sim_backend`；② 新增 `IncludeLaunchDescription`（gz，`condition=…=='gz'`）；③ 原 `start_rm_simulation` **内容一字未改**，只多了 `condition=…=='classic'`（默认恒真） |
| `--strip-conditions` 后 | **+2 新增 / 0 删除 / 顺序一致** ⇒ **纯增量** |
| `ros2 launch … --show-args` | 默认值全不变，只多一个 `'sim_backend': … (default: 'classic')` 与 `Valid choices are: ['classic', 'gz']` |

**三个缺口**（这是 L3 全部"不能"的来源）：

1. **gz 侧 LIO 未通过**【实测】：`small_point_lio` 持续打印 `imu loop back`（`preprocess.cpp:67` 的
   `imu_msg.timestamp < last_timestamp_imu`），**没有发出 `odom→base_link`**，`view_frames` 里
   **没有 `odom`/`map` 帧**，`/cloud_registered` 空。**发布端已被排除**：rclpy 直收 `/livox/imu`
   是严格 `+10 ms` 单调（381 条，`min=max=10,000,000 ns`，0 条负差），且 `Publisher count: 1`。
2. **底盘 `<static>`**【实测】：Fortress 没有 `planar_move` 的对应物 ⇒ `/cmd_vel_chassis` 不驱动底盘，
   nav2 规划/控制链路仍会跑但车不动。
3. **无同轮 Classic A/B**【未验证】：Classic 列填的是既有文档基线（见 §3 与 §4.0 冲突 1）。

### 1.4 L4：COD 帧架构对账（`057294c`）

**必须显式写下的更正**：**COD 用的就是我们 vendored 的同一份 `small_point_lio`**（同一上游
`Yancey2023/small_point_lio@688d75c`，`sha256` 逐字节相同，`tilted_lidar_fidelity.md` §N.1 已三条独立取证）。
"COD 换了 LIO 所以不受影响"**是错的**。COD 不炸的真原因是**换了 `base_link` 的定义**。

| 事实 | 证据 | 性质 |
|---|---|---|
| COD 的 `base_link ≡ livox_frame`（传感器壳体系，原点在雷达、z 随雷达斜 30°） | `small_point_lio_node.cpp:75` 的共轭只在 `T_bl=I` 时成立；`small_point_lio.launch.py:24-45` 声明单位 extrinsic 而 `:47` **只 return 节点**（死代码）；`cod_bringup` 全仓 3 处 `static_transform_publisher` **全是 `map→odom`**；`filter_node.cpp:56`（marker 发 `base_link`）vs `:81-119`/`:117`（CropBox 作用在输入点云系、`header` 原样保留）；`multiplenav_launch.py:70`（`target_frame: base_link` 而输入是 `/livox/lidar`） | 四条独立证据 ⇒ **推断（强旁证）**，因为本地 checkout **没有 `urdf/`** |
| COD 节点里是**共轭**（相似变换）`T_bl⁻¹·T_ol·T_bl` | `:75` | 【实测源码】 |
| COD 对"已配准的点"**又乘了一次外参** | `:159`（`transformed_point = R*point + T`），而 `small_point_lio.cpp:98` 已经是 `odom` 系 | 【实测源码】 |
| 两者在 `T_bl=I` 时**同时退化成恒等** ⇒ 在 COD 里**不可观测** | 上面的旋转/平移分量分析 | 【推断】 |
| ⚠️ **我们与 COD 的唯一差异**：我们的 vendored launch `:47` **把那条单位 TF 加进了返回值** | `src/rm_localization/small_point_lio/launch/small_point_lio.launch.py:47`，blame **`98baa55`** | 【实测源码】⇒ **潜伏陷阱**（当前 bringup 不走这个 launch，所以还没咬人） |
| COD **完全没有 `obstacle_layer`** | 两份 params `grep -c obstacle_layer` = **0 / 0**；`/scan` 命中 **0**；两张图都是 `["static_layer","stvl_voxel_layer","inflation_layer"]`，源 `/livox/lidar_filtered`，`min/max_obstacle_height 0.15/1.0` | 【实测源码】 |
| COD 的 `/scan` 只喂 `slam_toolbox`（多模式），`map→odom` 是**静态** TF（没有定位节点） | `multiplenav_launch.py:65-94`；`singlenav_launch.py:67-89`；`localization_launch.py` 的 `lifecycle_nodes=['map_server']` | 【实测源码】⇒ nav2 的"每束 z 由扫描原点伪造"这条代码路径**从未被执行** |
| 本仓 `lio_tf_adapter` 是**唯一写对合成**的实现，却在 `lio:=small_point_lio` 时被**排除** | `lio_tf_adapter_node.cpp:66-72`（`q_lio*q_off`、`p+R·t`）；`bringup_sim.launch.py:2028-2033`、`:2051-2055`、`:2076-2080` 三处 `!= 'small_point_lio'`；`:2025` 注释自认"该换算的写法有瑕疵" | 【实测源码】 |

**判决**：**(b) 约定分叉 + 可避免耦合**，不是 (c) 真缺陷。可避免的 **6 条**（O1–O6）vs 固有的 **6 条**（I1–I6）；
其中 **4 条明确该修**（O1 高度阈值住在 4 个 frame 里 / O2 adapter 被排除 / O3 三档并存 + `urdf` 账是错的 /
O4 我们启用了上游那条单位 TF），**2 条半修**（O5 `local_obstacle:=scan` 默认值 / O6 roll-pitch 双来源）。

**三选项排序**：

| 排 | 选项 | 内容 | 成本 | 风险 | 建议 |
|---|---|---|---|---|---|
| 1 | **C** | `robot11_mount` 三档收敛成 `plugin`（默认）+ `sensor`（保真），`urdf` 从 `choices` 移除只留文档反例；删 vendored 的单位 TF；修 `cod_nav_comparison.md:27/:43` | **S（≤1 人日）** | 低 | **先做**（等拍板，见 §8） |
| 2 | **B** | `lio_tf_adapter` 成为**所有** LIO 槽的 `odom→base_link` 唯一权威（给 `small_point_lio` 加 `publish_tf:=false`）+ 高度语义单一所有者 | **M（2–3 人日）** | 中 | 主体 |
| 3 | A | 全面采用 COD 约定（`base_link` 搬到雷达） | L（≥1 周 + 全部基线重跑） | 高 | **不做** |

---

## 2. ⭐ 与之前的区别（重点）

> 这张表 = `fortress_sim_backend.md` §2 的 11 条（去重后 14 行）**+** 本轮其它三条线带来的 3 类变化。
> "之前" 一律指 **Classic 11 / 本轮之前**；"现在" 一律指 **gz 后端 / 本轮之后**。

### 2.1 总表

| 组 | # | 维度 | 之前（Classic / 本轮之前） | 现在（gz / 本轮之后） | 为什么重要 / 影响 |
|---|---|---|---|---|---|
| gz | 1 | **传感器实现** | URDF `<sensor type="ray">` + `libros2_livox.so` | 模型级 `<plugin filename="libmid360_system.so">`（gz **system** 插件） | 传感器从"URDF 里的一条 ray 传感器"变成"挂在 model 上的系统插件"；SDF 里 `livox_frame` 是一个真 `<link>` |
| gz | 2 | **求交后端** | **ODE 逐射线**打 `<collision>` | **Embree BVH**（rmagine `O1DnSimulatorEmbree`，建一次、逐帧复用）；Fortress 无 `components/RaycastData` ⇒ physics 后端编译期关闭 | 从"ODE 逐射线"变成"CPU 批量 BVH"⇒ **求交第一次可以计时**（0.325 ms/实例，§3）；也是 RSS 降一个量级的来源 |
| gz | 3 | **点云是否过桥** | **不过桥**：`libros2_livox.so` 在 gzserver 进程内直接发 ROS 话题 | **也不**过桥：插件在 gz 进程内用 rclcpp 直发；`ros_gz_bridge` **只桥 `/clock`** | ⇒ 没有 `PointCloudPacked` 中转、**`frame_id` 不会变成 gz 的 `model::link::sensor` 名字**（迁移计划 §6.7 点名的最高风险项**没有发生**）、少一次序列化 |
| gz | 4 | **字段布局** | `x,y,z`（f32）+ **`rgb`**（f32），**`point_step=32`** | `x,y,z`(f32, 偏移 0/4/8) + `intensity`(f32, 12) + `tag`(u8, 16) + `line`(u8, 17) + `timestamp`(f64, 18)，**`point_step=26`**，**无 `rgb`**，`is_dense=true` | 本仓**没有**消费者硬依赖 rgb：grep 只 2 处命中（Classic 插件生产端 + `point_lio` 一个**没人用**的 typedef）；linefit/p2l/patchwork/gicp/探针全按**字段名**取 xyz ⇒ **节点不坏**。唯一实际影响是**外部工具**：RViz `Color Transformer=RGB8` 会全黑（改 `Intensity`） |
| gz | 5 | **`frame_id`** | `/livox/lidar*`=`livox_frame`；`/livox/imu`=`imu_link` | **同**（插件 `<cloud_frame>` / `<imu_frame_id>` 钉住） | 不钉 `imu_frame_id` 会退回 `livox_frame`，与 Classic 不一致 |
| gz | 6 | **`offset_time`（逐点时间）** | **恒 0**（帧内无运动模型） | `i·1e9/point_rate`，**单调**、最大 **99,996,667 ns = 99.9967 ms**（`i=29999`） | 去畸变第一次有真的时间维；也是"两实例同 stamp 同点数"的前提 |
| gz | 7 | **QoS** | 点云 `SensorDataQoS`（best_effort + keep_last 5）；IMU reliable | **同**（上游原值 `KeepLast(20).reliable()` 被补丁 **0003** 改掉）；IMU **保持 reliable** | 详见 **§2.2**：IMU 必须 reliable；全仓 9 个点云订阅者**全是** best_effort ⇒ 换写者不会"发了个寂寞" |
| gz | 8 | **双发布** | **一个插件一次** publish 两个消息 | **两个插件实例**（同 link、同姿态），各发一路；IMU 只由 #1 发（`<publish_imu>`） | 两实例**共用一棵 BVH**（`1 supported visuals` 只 1 次）、**313,721 点 atol=0 逐点相同**；代价只有**求交 2×**（≈0.65 ms/帧），无内存/一致性代价 |
| gz | 9 | **30° 记在哪** | `plugin` 档：插件键 **`<tilt_rpy>`**（帧是平的、数据是斜的） | SDF **`<link><pose>`** 的 roll **`-0.523598775598293`**，**且要求 URDF 关节也斜**（`robot11_mount:=sensor`）⇒ 帧与数据自洽 | gz 后端**只实现 sensor 语义**："平帧 + 斜数据"是自相矛盾的组合，没有移植价值；代价是 `plugin` 档在 gz 侧**不存在**（§2.9 第 2 条） |
| gz | 10 | **自机排除机制** | `livox_frame` **故意不给 collision** ⇒ "不打自己"靠**几何缺失** | 插件把所属 model 登记进 `MapRegistry` ⇒ 整车 **113 个碰撞体**不进 BVH；`<ignore_self>true\|false</ignore_self>` 可 A/B | A/B 判决：`true` ⇒ 中位 **8040.5** 点、`r_min 0.4292 m`、`r<0.30 m` 的点 **0**；`false` ⇒ **0 点（全盲）**，因为传感器被自己的碰撞体**整个包住** ⇒ 这条是**必需**，不是优化 |
| gz | 11 | **世界资产要求** | Classic 容忍空 `<sdf>`、SDF 版本宽松 | SDF **≤ 1.9**（`sdformat12` 上限；写 1.10 ⇒ `Unable to convert from SDF version 1.10 to 1.9`，**一帧点云都不出**）；`model.config` 的 `<sdf>` **不能为空**（否则 `Failure to detect an sdf tag`） | 两条都是"静默失败/整世界起不来"级；本仓两份 `world/model.config` 已在 **`da5c2fe`** 修好（2 行）；`ign sdf -k` 可预检 |
| gz | 12 | **`HOME` 与插件路径** | 无要求 | gz 要**可写** `$HOME`（`~/.ignition`）；本机 `$HOME` 只读 ⇒ **GUI abort 并 SIGKILL server**；`GZ_SIM_SYSTEM_PLUGIN_PATH` **和** `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` **都**要设 | launch 自动把 gz 子进程的 `HOME`/`XDG_*` 重定向到运行期临时目录（`gz_home`）；裸 shell 里手起 `ign gazebo` 要自己重定向 |
| gz | 13 | **启动方式 / 参数** | `bringup_sim.launch.py …` 一条路 | 同名 launch 多一个 **`sim_backend:=classic\|gz`**，**默认 `classic`**；另有 `gz_world` / `gz_qos` / `gz_ignore_self` / `gz_home` / `gz_csv` / `gz_runtime_dir` | 3 处变化、净 **+2** 个 Action（61→63），`--strip-conditions` = **+2/−0**；不传该参数时节点集合/命令行/时序**逐字节不变** |
| gz | 14 | **底盘 / 真值里程计** | `libgazebo_ros_planar_move.so` 订阅 `/cmd_vel_chassis`，并发 `/odom_ground_truth` | **都没有**：机器人在世界文件里是 **`<static>true</static>`**；`/odom_ground_truth` **不存在** | ⇒ gz 只能做"静止的感知/定位/建图"验收，**不能**跑 nav2 移动闭环；依赖真值里程计的调试脚本在 gz 档拿不到数据（契约上不要求，`odom→base_link` 归 LIO） |
| 其它 | 15 | **新增断言**（L1） | 两个 latent bug 只能靠**离线文档**看 | 启动 readout（1 条/次）+ 每帧 `T_ob·T_bl⁻¹≈T_ol` 断言（**只计数 + 限速 ERROR 1/5 s**） | 成本 **24.4 ns/帧**；默认档**逐位 0 差异**；斜装档会喊；**不抛、不中止、不改发布、无 latched 话题** ⇒ 不改任何契约 |
| 其它 | 16 | **收尾作用域**（L2） | 全机 `pkill -9 -x gzserver/gzclient` + `pkill -9 -f 'ros2 launch'` + 空操作 `pkill -f "gzserver.*$PORT"` | `scoped_cleanup.sh`：**进程组 / `/proc/environ` 的 `GAZEBO_MASTER_URI` / 血缘**，**14 个脚本**接入 | 名字只用来挑候选、绝不单独作为杀的理由；同机有兄弟任务时不再误杀；`pkill -9 -x gz` **18 行 → 0** |
| 其它 | 17 | **资产修正**（`da5c2fe`） | 两份 `world/model.config` 是 `<sdf version="1.7"></sdf>`（**标签内容为空**） | `<sdf version="1.7">model.sdf</sdf>`（**2 个文件、2 行**） | Classic 侥幸容忍；sdformat12 不容忍 ⇒ 没有这 2 行，gz 世界**根本进不去** |

### 2.2 QoS：本轮唯一一条"结论与直觉相反"的

| 话题 | Classic | gz（默认） | 为什么 |
|---|---|---|---|
| `/livox/lidar`、`/livox/lidar/pointcloud` | `SensorDataQoS`（**best_effort** + keep_last 5） | **同**（`<qos_reliability>best_effort</qos_reliability>`） | 与 Classic **逐字对齐** |
| `/livox/imu` | 默认 **reliable** | **reliable 保留** | **FAST-LIO 用默认 depth-10（reliable）订阅 IMU**；best-effort 写者 + reliable 读者 **QoS 不兼容 ⇒ 一条都收不到**。IMU ~250 B @100 Hz，且发布发生在 gz transport 回调线程（不在 `PostUpdate`）⇒ 不构成背压源 |

**"全仓订阅者都是 best_effort"是这次敢改的前提**【实测 grep，`fortress_sim_backend.md` §4.2】：
`FAST_LIO`、`small_point_lio`、`point_lio`（`/livox/lidar`）、`linefit_ground_segmentation`、
`pointcloud_to_laserscan`、`patchwork_ground_segmentation`、`gicp_registration`、
`cloud_accumulator`、`tools/scripts/**` 探针 —— **9 个订阅者全是 `SensorDataQoS`/`BEST_EFFORT`**。
⇒ 换 best-effort 写者**不会**造成"收不到"。`reliable` 仍可一键复现（`gz_qos:=reliable`），**只用于对照实验**。

**诚实结论（这一条被任务书要求标出来）**：慢消费者实验里 **best_effort 9.711 Hz vs reliable 9.689 Hz**，
把订阅者 `SIGSTOP` 后 **best_effort 9.740 Hz vs reliable 9.660 Hz** —— **reliable 在本机也没有被堵**。
⇒ 这条改动的收益是**契约对齐 + 按定义永不阻塞的保证**，**不是本机测出来的性能提升**；
Classic 那次"写者阻塞"的机理牵涉"写者与读者在同一进程/同一执行器"，gz 插件是"PostUpdate 里 publish、
DDS 线程发"，**结构上就不同**。⚠️ Classic 那次"停更且不自恢复"的完整链条，本轮**没有**在 gz 侧复现，
也**没有**在 Classic 侧复跑对照（§4 U6）。

### 2.3 这 17 条可以分成三类（读法）

| 类别 | 条目 | 含义 |
|---|---|---|
| **物理/上游必然**（改不了，只能适应） | 1、2、4、6、11 | 传感器换了引擎、字段由插件决定、sdformat12 的版本上限 …… 这些不是"我们选错了" |
| **我们的选择**（有理由，但要能回退） | 3、5、8、9、12、13、14 | 不过桥、钉 `frame_id`、双实例、SDF `<pose>` 承担 30°、HOME 重定向、`sim_backend` 默认 classic、底盘静态 |
| **修 bug / 补护栏** | 7、10、15、16、17 | QoS 补丁 0003、实体级自机排除（Classic 靠几何缺失，不够用）、外参守卫、收尾作用域、`model.config` |

### 2.4 刻意**没有**变的东西（别误读）

| 契约 | 状态 |
|---|---|
| 话题名与类型（`/livox/lidar` CustomMsg、`/livox/lidar/pointcloud` PC2、`/livox/imu` Imu、`/scan`、`/cloud_registered`、`/Odometry`） | **一致**（下游节点一行未改） |
| TF 链（`map→odom→base_link→livox_frame/imu_link`）与单发布者约定 | **一致**（gz 侧不发任何 odom TF） |
| 谁发 `odom→base_link`（LIO）/ 谁发 `map→odom`（按 `mode`/`localization`） | **一致** |
| LIO 配置（`extrinsic_R=I`、`frame_id` 名字） | gz 档**不需要改** |
| Classic 的 world / xacro / config / 感知 / nav2 文件 | **一字未动**（`fortress_sim_backend.md` §2.11） |

---

## 3. 实测数字总表

> **口径警告（必读）**：**Classic 列是"既有文档里的历史基线"，不是本轮同机同时刻新测的** ⇒
> 没有做同轮 A/B，"变化"列**只能看量级**。Classic 的数字来源是 `livox_sensor_prior_art.md`
> §1/§10.3 与 `tilted_lidar_fidelity.md` §J（详见 §4.0 冲突 1）。

### 3.1 gz vs Classic 并排

| 指标 | **Classic 11**（⚠️ 历史基线，非同轮） | **gz / Fortress**（本轮实测） | 变化 / 口径 |
|---|---|---|---|
| RTF（`/clock` 仿真时间 / 墙钟） | **0.32–0.46**（`robot11` 全栈，迁移计划 §1/§9） | **0.97**（台架）/ **0.97**（`mode:=mapping` 整栈） | **≈ +2.4×**（按 0.40 中值算） |
| server RSS | **3.2 GiB**（prior_art §10.3 对照行） | **328 MB**（台架，整棵进程树）/ **441 MB**（含 robot11 + TF 发布者）/ **652 MB**（mapping 整栈） | **−80% … −90%** |
| 逐帧求交耗时 | 无（ODE 逐射线，未插桩） | **0.325 ms / 30000 rays**（单实例中位；min 0.307 / max 0.358，28 次采样）；双实例合计 **≈0.65 ms/帧** | 绝对量 <1 ms |
| 每帧点数（中位） | ≈ **11,568**（`tilted_lidar_fidelity.md` §J 的 `plugin` 档） | **8018.5**（台架，7854–8167）/ **8040.5**（robot11，sensor 档） | 差异来自**命中率与 `max_range`**，不是丢点（30000 束 26.7–27.2% 命中） |
| 启动 → 首帧点云 | 未记录 | **3.00 s**（launch → 日志 `Loaded 800000 scan directions`） | — |
| 仿真频率 | 10 Hz 名义（未记录实测回归） | **10.0000 Hz**（`CustomMsg` 戳线性回归，120 帧，误差 <1e-12） | — |
| 墙钟频率 | — | **9.72 Hz**（台架 headless）/ **9.48 Hz**（整栈） | 台架 RTF ≈0.97；整栈只记了 9.48 Hz，未单列 RTF |
| `offset_time` | **恒 0** | 最大 **99,996,667 ns**，**全帧单调** | 新增时间维 |
| `point_step` / `rgb` | **32** / **有** | **26** / **无** | 无本仓消费者硬依赖 |
| 双发布 | 1 次 publish 2 消息 | 2 实例；**119/119 帧同 stamp 同点数**、**313,721 点 atol=0** | 细表 §3.2 |
| 自机排除 | 靠几何缺失 | A/B：**8040.5 点 vs 0 点**，`r_min 0.4292 m` | 细表 §3.2 |
| `/scan` | 链路存在（历史） | **418 条有限波束、2.75–8.09 m**、`frame_id=base_link` | linefit → p2l 在 gz 档**打通** |
| LIO（`small_point_lio`） | 可用（§J/§N 有全栈数字） | ❌ **未通过**：`imu loop back`、无 `odom→base_link` | 见 §4 U1 |
| QoS 慢消费者 | 历史上有"写者阻塞"（Classic 那次） | best_effort **9.711 Hz** / reliable **9.689 Hz**（**都没被堵**） | 见 §2.2 / §4 U6 |

### 3.2 三个专项实验（原始数字）

**(a) 双实例：BVH 共享 + 逐点一致性**【实测，台架世界 `rmuc2026_gz_bench.sdf`，headless，120 帧】

| 判据 | 实测 | 结论 |
|---|---|---|
| BVH 是否只建一份 | 日志 `Embree map bootstrapped with 1 supported visuals` **只出现 1 次**（不是 2 次）；两实例 `<map_key>default</map_key>` 相同 ⇒ 同一 `EmbreeMapPtr` | ✅ 共用一棵 |
| 逐帧求交 | 单实例 min 0.307 / median **0.325** / max 0.358 ms @30000 rays；双实例合计 ≈0.65 ms | ≈2×，符合预期 |
| 内存 | 328 MB（只有传感器）/ 441 MB（含 robot11） | 没有"BVH 两份"的台阶 |
| `frame_id` / 时间戳 / 点数 | 两路都是 `livox_frame`；**119/119 同 stamp**；`n_delta_max = 0` | ✅ |
| 逐点坐标 | **313,721 个点 xyz 完全相同（atol=0，比特级）** | ✅ 两个实例算的是同一次求交结果 |

**(b) 自机排除 A/B**【实测，整栈世界 + robot11 的 113 个真碰撞体，同一位姿只切一个开关】

| 量 | `ignore_self:=true`（生产档） | `ignore_self:=false`（对照档） |
|---|---|---|
| 建树日志 | `… with 1 supported visuals` | `… with 14 supported visuals` |
| 每帧点数（中位） | **8040.5** | **0**（`n_max` 也是 0） |
| 最近点 `r_min` | **0.4292 m**（> 0.6×0.6 底盘半对角 0.42 m） | —（没有点） |
| `r<0.12 m` / `r<0.30 m` 的点 | **0 / 0** | — |
| RTF | 0.9686 | 0.9985 |

⇒ 对照档是 **0 点而不是"看见自己"**：传感器被自己的碰撞体整个包住，每条射线都从几何体内部出发 ⇒
单面几何判不到有效交点 ⇒ **全盲**。这比"看见自己"更糟，也更能说明这条排除是**必需**的。
（`r_min=0.429` 只说明"没有近场回波"，`ignore_self` 的 A/B 才是"看不见车体"的证据。）

**(c) QoS 慢消费者矩阵**【实测，2 s/帧的慢订阅者 + 快探针 + `/clock`】

| 场景 | 写者 QoS | 快探针墙钟频率 | `/clock` 相邻最大仿真跳变 |
|---|---|---|---|
| 基线（无慢消费者） | best_effort | 9.743 Hz | 18 ms |
| **慢消费者在时** | **best_effort** | **9.711 Hz** | 18 ms |
| 慢消费者退场后 | best_effort | 9.705 Hz | 21 ms |
| 基线（无慢消费者） | reliable | 9.703 Hz | 26 ms |
| **慢消费者在时** | **reliable** | **9.689 Hz** | 32 ms |
| 慢消费者退场后 | reliable | 9.730 Hz | 22 ms |
| **`SIGSTOP`（不 ack）** | **best_effort** | **9.740 Hz** | — |
| **`SIGSTOP`（不 ack）** | reliable | 9.660 Hz | — |

⇒ **两种 QoS 在本机都没被堵**。收益是契约与保证，不是实测提速（§2.2）。

### 3.3 怎么读这三张表（三条限制）

1. **没有同轮 A/B**：Classic 列是历史基线，机器上同时还有别的 agent 在跑 ⇒ 单个数字有 ±3% 抖动。
2. **"永不阻塞"没有在本机复现**：所以不要把 9.711 vs 9.689 当成"best_effort 更快"的证据。
3. **gz 侧的端到端只覆盖 `mode:=mapping`**：`mode:=slam_nav` / `mode:=nav` 未验证；且因为 LIO 未通过，
   `/cloud_registered` 一路没有数据（§4 U1/U5）。

---

## 4. 缺口与未验证（四份文档去重后 **24 条**）

### 4.0 先说 4 处口径冲突（本文取了哪一版、为什么）

| # | 冲突 | 本文取 | 为什么 |
|---|---|---|---|
| 1 | `fortress_sim_backend.md` §2.8 **抬头**写"Classic 的数字是本机同轮实测"，同节**表头/表尾**与 **§10 第 8 条**都说"Classic 是既有基线、**没有**同轮 A/B" | **取后者**（历史基线） | §10-8 是"未验证清单"里的自述，且**指名了来源**（`livox_sensor_prior_art.md` §1/§10.3），还给了理由（本轮时间全花在 gz 侧）。抬头那句应是笔误 |
| 2 | 本轮任务摘要说 gz 新增"**+3 Actions**"，`fortress_sim_backend.md` §9 说"**61→63**（净 +2）" | **取"3 处变化 / 净 +2"** | 两者不矛盾但口径不同：3 处 = 1 个新增 `DeclareLaunchArgument` + 2 个 `IncludeLaunchDescription`，其中 classic 那条是**原 Action 加了 condition**（不是新增）⇒ 净 +2。`--strip-conditions` 的 +2/−0 是同一件事的另一种数法 |
| 3 | `cod_vs_ours_frame_architecture.md` §0-5 说"放大成 6 个，其中 **4 个**是我们自己造的"，§7.2 小结说"**6 个**可避免耦合，**4 明确该修 + 2 半修**" | **取 §7.2** | §7.2 有 O1–O6 逐条证据与"该不该修"的裁决；§0 那句是摘要，措辞把 O5/O6 归到了"半修"之外 |
| 4 | `machine_hygiene_runbook.md` §6 说"已清掉 **9 个**脚本里的这类写法"，§8.2 逐文件表是 **14 个** | **取 14** | 9 = 最初点名的清单；实际改的是 14 个（§8.6 另说"18 行里 11 行在点名的 9 个脚本，另 7 行在同族 4 个脚本里，也一并改了"） |

### 4.1 高风险：阻断闭环 / 使结论不可用（7 条）

| ID | 缺口 | 证据 / 现状 | 影响 | 下一步 |
|---|---|---|---|---|
| **U1** | **gz 侧 LIO 未通过** | `small_point_lio` 持续 `imu loop back`（`preprocess.cpp:67`）、无 `odom→base_link`、`view_frames` 无 odom/map 边、`/cloud_registered` 空；发布端已排除（`+10 ms` 严格单调、`Publisher count: 1`） | gz 侧**不能**做定位/建图闭环；`map→odom→base_link` 的**单发布者契约没有被真正考到** | 先 `lio:=fastlio` 复跑（FAST-LIO 的 IMU 订阅是默认 reliable，与我们的写者兼容）；再给 IMU 回调加"前 20 条戳"临时日志 |
| **U2** | **gz 底盘是 `<static>`** | Fortress 无 `planar_move` 对应物；`/cmd_vel_chassis` 无消费者 | gz 侧**不能**跑 nav2 移动闭环 | 写一个 gz 侧的 `cmd_vel` → 位姿插件（未做） |
| **U3** | **没有同轮 Classic A/B** | Classic 列来自既有文档（§4.0 冲突 1） | §3 的"变化"列**只能看量级**，不能当精密对照 | 同机同刻跑一次 Classic 同参数（fortress §10 第 8 条） |
| **U4** | **gz 只支持 1 场地 + 1 机器人** | `world:=` 只 `RMUC2026`；`robot:=` 只 `robot11` | 其它场地/`hzmirm` 在 gz 档**直接报错**（明确报错，不静默） | 加 `worlds/<name>.sdf`（SDF ≤1.9）并登记 `WORLD_MAP`；其它机器人要从 URDF 派生 |
| **U5** | **gz 未验证 `mode:=slam_nav` / `mode:=nav`** | 端到端只跑了 `mode:=mapping` | nav2 在 gz 档的状态未知 | 等 U1 修好后再跑 |
| **U6** | **QoS "写者被堵"未复现** | best_effort 与 reliable **都没被堵**（§3.2c） | 这条改动的收益只能表述为"契约 + 按定义永不阻塞的保证"，**不能**表述为"实测提速" | 保留 `gz_qos:=reliable` 一键开关，将来复现 |
| **U7** | **§J 的 `1.05°` 未复现** | 本轮最大平面 `d = 0.103 m`、出生 yaw **0.01°**；§J 那轮 `d = 0.26 m`、yaw **10.12°** ⇒ 不是同一块面/同一姿态（世界在 10-09 后被改过；本轮未动 `src/rm_simulation/**`） | **不主张"绝对角 = 1.05°"**，只主张判决性签名（bug ② 的 30° 签名不存在：0.046° vs 29.977°） | 回到 §J 那个世界/出生点版本才能逐数对齐 |

### 4.2 中风险：结论必须标"推断"（8 条）

| ID | 缺口 | 证据 / 现状 | 影响 |
|---|---|---|---|
| **U8** | COD 本地 checkout **没有 `urdf/`** | `third_party/cod_nav_2026/` 只有 `src/`（190 文件） | "`T_bl ≈ I`"是**推断**（强旁证：若它不是单位，COD 的 `odom→base_link` 会明显错，nav2 不可能正常工作） |
| **U9** | COD 的 `odom` 是否真随雷达斜 30° | 由 `eskf.h:30`（`R_W_B(0)=I`）+ `small_point_lio.cpp:54-64`（`gravity` 取 body 系测量的和）**推出**的，**没有在 COD 数据上实测** | COD 文档 §4.5 的三条后果（STVL 斜板、p2l 斜板、footprint 差 `cos30°`）都是**推断**；若 `gravity` 被后续观测拉回真垂直，结论要改 |
| **U10** | COD 若缺 `base_link→livox_frame`，`:65` 的 `lookupTransform` 会抛异常 ⇒ `:68` 直接 `return` | 需要实跑才知道他们的图里到底有没有这条边 | 若真缺失：COD 实际是"没有任何 `odom→base_link`"⇒ 推翻 COD 文档 §2.1(c) 的表述顺序，但**不推翻结论** |
| **U11** | COD 选项 B1a 需改 `src/rm_localization/small_point_lio/**`（跨 agent 权限） | 本轮只给方案与验收，**未落地** | 选项 B 的成本里含一次跨 agent 协调 |
| **U12** | 无真机、无 MID-360 | 所有数字来自本仓仿真 + bag 重放 | 真机上 `R_bl` 由谁提供（URDF 关节 vs 用户 `static_transform_publisher`）**未核实** |
| **U13** | 微基准不是 `perf` 级剖析 | 24.4 ns 是纯算术热缓存；"14 个 double 拷贝 + 成员读取"的额外开销按量级估 `<0.1 µs/帧` | 成本结论是"量级可忽略"，不是"精确 24.4+5 ns" |
| **U14** | gz 侧**动态障碍 refit** 未走到 | 世界全静态、机器人在 gz 侧也是静态；`delta_trans/delta_rot` 分支未被触发 | 动态场景下 BVH 的增量更新**未验证** |
| **U15** | gz **GUI** 无成体系数字 + 无 CPU 剖析 | 整树 RSS 910 MB、点云探针没采到数据；CPU 只有 0.5 s 采样的累计 tick | GUI 路径与 CPU 峰值都不能引用 |

### 4.3 低风险：覆盖率与流程（9 条）

| ID | 缺口 | 说明 |
|---|---|---|
| **U16** | 一致性断言只在"**有点云输出**的帧"上跑 | 某帧 `pointcloud_odom_frame` 为空则该帧不做断言（`/odom` 仍照发）⇒ **覆盖率的缺口**，不是数值风险 |
| **U17** | 没有 latched `UInt64` 违例计数话题（**有意**） | 加话题会改话题契约（§J.3 的"每话题恰好 1 个发布者"会多一条）；计数写进同一条限速 ERROR |
| **U18** | 故障注入只在 `.tmp_guard/` 的临时副本里做过 | 仓库源码里没有共轭写法、也没有任何注入开关 |
| **U19** | `R_bl` **时变**（云台/机械臂）未验证 | 守卫按"固定外参"设计；`T_bl` 每帧重查但 `#1` 只打一次；时变时 `#2` 会喊（这是对的） |
| **U20** | **沙箱限制**：root 命令 **0 条执行**；无宿主机 PID 视图 | `sudo` 被 `no-new-privs` 挡死 ⇒ swapfile/zram/swappiness/oomd 调参**全未执行**（命令原样给出）；"每进程 VmSwap top15"原理上取不到（只有 4 个 PID），改用 cgroup `memory.swap.current` |
| **U21** | 14 个改造脚本里 **11 个**只做了 `bash -n` + 合成判定 | 没有逐个跑完整流程（动辄 5–10 分钟）；3 个真跑过（`run_nav_smoke_regress` / `run_remap_lowterrain` / `run_robot_model_probe`） |
| **U22** | `start_sentinel.sh` 仍是全机 pkill | `:165` gzserver / `:166` gzclient / **`:169` `pkill -f ros`** / `:172` rviz2 —— 一键启动人工工具，会杀掉别人的 ROS |
| **U23** | 资产/规模覆盖不足 | `ign sdf -k` **未扫全仓** SDF（只做了 2 个世界）；多实例 **>2**、多传感器同世界**未测**；Classic 的 `meshes/obstacles/*` **没有**进 gz 世界 |
| **U24** | COD 侧三条外围未知 | `cpp_lidar_filter` 的裁剪盒到底按哪个系标（源码自相矛盾）；`map→odom` 静态 TF 的 `yaw=-0.5` 从哪来（无注释）；是否另有**仓库外**启动项提供 `base_link→livox_frame`/`urdf` |

**计数**：**24 条**（高 7 / 中 8 / 低 9）。另有 4 处**口径冲突**（§4.0，不是缺口，是"两份材料说法不同"）。

---

## 5. 回退表

| 线 | 回退动作（按"最小 → 彻底"） | 影响 | 注意 |
|---|---|---|---|
| **L1 外参守卫** | ① **只想静音**：不动代码，日志过滤（`grep -v 外参守卫`）② **彻底**：`git revert fe0dc31`；或手工删 `src/extrinsic_guard.cpp` + `small_point_lio_node.cpp` 的 2 处调用（`:193-194` 快照、`:208-209` 调用）+ `small_point_lio_node.hpp:44-63` 成员 + `CMakeLists.txt:74-78` 的 `APPEND` + 单测 ⑥（`tf2_compose_order_test.cpp:155-201`） | 只丢日志/计数；**发布数值逐位不变**（§O.4(a) 已逐位证明） | 别只删 `.cpp` 不删调用点（链接期 undefined symbol） |
| **L2 收尾作用域** | ① **单脚本回退**：把该脚本的 `sc_*` 调用换回原来的 `pkill`（每处改动都留了注释）② **彻底**：`git revert a137bf2`（同时删 `tools/scripts/lib/scoped_cleanup.sh` 与 runbook） | 回退即恢复"全机 pkill"⇒ 会误杀同机别人的栈 | **不要**只删 `scoped_cleanup.sh` 却留着 `source`（14 个脚本全部报错） |
| **L3 gz 后端** | ① **最省事**：**不传 `sim_backend`** ⇒ Classic（默认路径零改动）② **代码**：删 `src/rm_simulation/rm_livox_gz_sim/` + 回退 `bringup_sim.launch.py` 的 3 个 Action（`declare_sim_backend_cmd` `:1223`、classic 的 `condition` `:1586`、gz 的 `IncludeLaunchDescription` `:1591-1606`）③ **构建前缀**：`rm -rf .tmp_gzlivox2/gzsim_ws`（没装进 `/opt/ros`、不碰系统库；换一份用 `GZSIM_PREFIX` 指） | ①②③ 之后回到 **100% 原状** | **不建议** `git revert 057294c`：gz 的代码内容随它入库，整提交 revert 会**连 COD 文档一起撤掉** |
| **L4 资产修正** | `git revert da5c2fe`（两份 `world/model.config` 的 `<sdf>` 回到空） | Classic 不受影响；**gz 世界再也起不来**（`Failure to detect an sdf tag`） | 这 2 行是 gz 的**硬前提**，不是可选优化 |
| **L5 文档** | 删 `docs/round_summary_gz_frames.md` + 撤 `docs/README.md` 里那一行 | 无 | 纯文档 |

---

## 6. 用法速查

### 6.1 gz 后端（两步）

```bash
# ① 一次性：取上游 + 打补丁 + 编译（源码不进本仓库，编到独立前缀）
tools/scripts/gzsim/fetch_and_patch_mid360.sh

# ② 整栈：与 Classic 同一条 launch，只多一个参数
ros2 launch rm_nav_bringup bringup_sim.launch.py sim_backend:=gz \
    world:=RMUC2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0
```

**两个硬要求**（缺了会明确报错，不静默跑错）：

| 要求 | 为什么 |
|---|---|
| `robot:=robot11` | 只有这个槽位在 gz 侧有 Fortress 化的模型生成路径（从同一份 URDF 派生 SDF） |
| `robot11_mount:=sensor` | 倾角必须在 **URDF** 里，`robot_state_publisher` 发的 `base_link→livox_frame` 才与物理一致（gz 的 SDF `<link><pose>` 也真的带 roll −0.5236） |

> `robot11_mount:=urdf` 在 gz 侧**也能跑**（`fortress_sim_backend.md` §2.7），但那一档"账是错的"
> （COD 文档 O3），只适合做反例 ⇒ 生产用 `sensor`。

**台架（只看传感器，不开感知/LIO）**：

```bash
ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=False
# 带 GUI（无 GPU 机器要 Xvfb；本仓不替你起）
xvfb-run -s "-screen 0 1280x800x24" ros2 launch rm_livox_gz_sim gz_sim.launch.py robot:=none gui:=True
```

**Classic 原命令不变**：去掉 `sim_backend`（或不写）就是 Classic 默认路径。

### 6.2 其它常用

```bash
# 看守卫有没有喊（启动 readout / 每帧断言）
ros2 launch … 2>&1 | grep -E '外参守卫|位姿一致性断言失败'

# 单测第 6 条（不进 colcon）
g++ -O0 -o /tmp/tf2order tools/scripts/tiltmount/tf2_compose_order_test.cpp \
  -I/opt/ros/humble/include/tf2 -L/opt/ros/humble/lib -ltf2 -Wl,-rpath,/opt/ros/humble/lib && /tmp/tf2order

# 脚本收尾排障：看每个 own/skip 的原因
SC_SCOPE_VERBOSE=1 bash tools/scripts/…

# 默认路径零改动的自检
python3 tools/scripts/gzsim/launch_increment_diff.py --old /tmp/old.py \
  --new src/rm_nav_bringup/launch/bringup_sim.launch.py
```

> ⚠️ 本机 `$HOME` 只读，`ros2 launch --show-args` 会在 `~/.ros/log` 上 OSError（**与本次改动无关**）。
> 跑这类命令前把 `HOME`/`ROS_LOG_DIR` 指到可写目录。

---

## 7. 文档索引（这轮新增/更新的）

| 文档 | 变化 | 行数 | 看它解决什么问题 |
|---|---|---|---|
| [`tilted_lidar_fidelity.md`](tilted_lidar_fidelity.md) | 追加 **§O**（+157 行） | 2987 | **守卫为什么这么写**：阈值、插入点、独立 TU 的 A/B、24.4 ns 成本、单测 ⑥、三段回归、诚实清单 |
| [`machine_hygiene_runbook.md`](machine_hygiene_runbook.md) | **新增** | 701 | **swap 到底满没满**（没满）、该不该动、oomd 为什么杀整组、哪些命令需 root（一条没跑）、脚本收尾为什么只杀自己 |
| [`fortress_sim_backend.md`](fortress_sim_backend.md) | **新增** | 547 | **gz 后端怎么用**、与 Classic 的 11 条差异（§2）、契约逐条实测（§5）、自机排除（§6）、端到端（§8）、会坏的地方（§2.9）、回退（§2.11） |
| [`gazebo_sim_migration_plan.md`](gazebo_sim_migration_plan.md) | 追加 **§11**（+25 行） | 1862 | **P0–P4 计划的最终状态**：哪项完成、哪项部分完成（P1 底盘、P4 A/B、LIO） |
| [`cod_vs_ours_frame_architecture.md`](cod_vs_ours_frame_architecture.md) | **新增** | 607 | **COD 为什么没事**（`base_link≡livox_frame`）、我们哪 4 条耦合是自己造的（O1–O4）、C/B/A 三选项与验收命令 |
| [`round_summary_gz_frames.md`](round_summary_gz_frames.md) | **新增（本文）** | 512 | **本轮四条线的合并入口**：一页看懂做了什么、与之前的区别、数字、缺口、回退、待办 |

---

## 8. 待办

| # | 待办 | 谁定 / 成本 | 依据 |
|---|---|---|---|
| **1** | **`robot11_mount` 的 `urdf` 去留**：选项 C 建议从 `choices`（`bringup_sim.launch.py:172`、`:1303`）移除，只留 `plugin`（默认）+ `sensor`（保真），`urdf` 降级为**文档里的反例** | **等你拍板**（≤1 人日，含验收 4 条） | COD 文档 §8 选项 C / O3 |
| **2** | **`start_sentinel.sh` 的全机 pkill**：`:169` 的 **`pkill -f ros`** 会杀掉整机所有 ROS 进程（还有 `:165` gzserver、`:166` gzclient、`:172` rviz2） | 低（把它也改成 `setsid` + 本 URI 核对） | `machine_hygiene_runbook.md` §8.7 第 1 条 |
| **3** | **`cod_nav_comparison.md:27/:43` 更正**：`:27`「全局 costmap 只吃 `/scan`」对 Gitee 2026 **不成立**（两张图都不吃 `/scan`）；`:43`「带在雷达面**之上** 0.01~1.0」实际是 **`0.15 / 1.0`** | 低（2 行） | COD 文档 §3.1 |
| **4** | **gz 侧 LIO 验证**（最高优先）：先 `lio:=fastlio` 复跑，确认 gz 后端本身能不能喂出 odom；再决定要不要给 `small_point_lio` 的 IMU 回调加临时日志 | 中（需跑仿真） | `fortress_sim_backend.md` §8.1 / §10 第 5 条 |
| **5** | **`fortress_sim_backend.md` §10 的 item 8 / item 9**：**8 = Classic vs gz 的同轮对照性能**（现在"变化"列只能看量级）；**9 = gz GUI（Xvfb）的成体系数字**（现在只有整树 RSS 910 MB，点云探针没采到数据） | 8 需要同机同刻跑两遍；9 需要 GUI 环境 | 同上 §10 第 8/9 条 |
| **6** | 其它（成本更高，先记）：COD 的 `urdf/` 取证（U8）· `R_bl` 时变（U19）· 全仓 SDF `ign sdf -k` 预检（U23）· 选项 B（adapter 唯一权威，M 级，需跨 agent） | 高 | §4 |

---

## 附：一句话记住这轮

**我们查出的 6 个"斜装雷达问题"里，2 个是上游 bug（已修 + 已加守卫）、4 个是我们自己的默认值/接线；
COD 之所以看起来"没事"，是因为它把 30° 藏进了 `base_link` 的定义里 —— 不是它更对，是它更看不见。**
（`cod_vs_ours_frame_architecture.md` §6.2）
