# 统一输出契约（选项 U）的**效率 + 精度实测**（2026-10-20）

> 对象：`lio:=small_point_lio` 从"自己发 `odom→base_link`"改成"**adapter 唯一发布 + E1 初始化对齐**"
> （契约 `docs/frame_contract_v3.md` §2.1/§4/§13）。核心问题有两个，本文只回答这两个：
> **1）这么改，`small_point_lio` 变慢了吗？** **2）对齐之后它还比 `pointlio` 快吗？**
> （用户的判据原话："这个 lio 设计出来就是为了提速优化，不然不如 pointlio"。）
>
> 机器：**Intel i7-14700HX / 28 线程**，无 GPU 参与，`gui:=False`，同一世界 `RMUL2026`、同出生点、
> 同一 `spin_speed:=0.0`（静止）、`robot:=robot11 robot11_mount:=sensor`（**诚实斜装链**，30° 倾角在
> URDF 关节里、点云真的表达在传感器系）、`mode:=mapping`（无导航栈）。
> 原始数据：`.tmp_uc/<tag>/`（gitignored：`proc.json` / `probe.json` / `clouds.npz` / `launch.log` /
> `publishers.txt` / `params.txt` / `meta.txt`）。采集脚本**已提交**：
> `tools/scripts/tiltmount/{uc_efficiency_bench.sh, uc_procstats.py, uc_replay_identity.sh, uc_odom_compare_recorder.py}`
> （测量当时以 `.tmp_uc/` 下的同名副本运行，逻辑相同）。

---

## §1 一句话结论

1. **改成"adapter 唯一发布 + E1 对齐"没有可测到的开销**：`small_point_lio` 的窗口 CPU 从
   **13.85 ms/帧（基线）→ 13.75 ms/帧（对齐、adapter 在跑）**，差值在跑间噪声内；
   多出来的 adapter 进程只有 **0.18 % CPU / 0.45 ms/帧**。
2. **但在本仓这份调参下，`small_point_lio` 比 `pointlio` 慢 2.4 倍**：**13.75 vs 5.75 ms/帧**、
   **5.6 % vs 2.5 % CPU**。→ 用户那句"不然不如 pointlio"在**本 bench 的这份配置上成立**，
   原因与"改契约"无关，是**逐帧处理点数**与**初始化门**的差别（§6 有分解，含未证明部分）。
3. **对齐把残余地面倾角从 30.1° 打到 0.38°（hybrid）/0.61°（urdf）**，`odom→base_link` 静止 rpy 从
   `[30.04, −0.23, −0.05]°` 变成 `[0.27, −0.20, 0.04]°`，**全部过 §4.5 的 <0.7° 线**；
   `pointlio` 同场景 0.11–0.27°（E1 文档记 0.166°）。
4. **默认路径（`gravity_align_mode:=off`）零改动**：launch 增量证明（节点集合/参数/顺序不变）+
   **同一份输入流重放：before/after 两个二进制输出逐位相同（141/141 行全等）**。
5. 一处**必须记住的口径**：`pointlio` 的 `/cloud_registered` 只有 **146 点/帧**，`small_point_lio` 是
   **11864 点/帧**（81×）。两者"每帧"的工作量**不同**，`ms/帧` 不是算法核心的等价比较（§6.2）。

---

## §2 方法（可复现；口径写死在这里）

### 2.1 三次跑 × 三个二进制档位

| 列 | tag | 二进制 | launch 关键参数 |
|---|---|---|---|
| **baseline**（改动前） | `uc_base_b1` / `uc_base_b2` | **before** = `git show 23bebb8:...` 编译的 `libsmall_point_lio.so`（md5 `b4baa05b…`） | `... lio:=small_point_lio`（`gravity_align_mode` 缺省 = `off`） |
| **aligned**（改动后启用） | `uc_align_a1` / `uc_align_a2` / `uc_align_a3` | **after**（md5 `95508487…`） | `... gravity_align_mode:=hybrid`（⇒ `publish_tf:=false` + `odom_convention:=sensor` + adapter） |
| aligned（`urdf` 档对照） | `uc_align_urdf` | after | `gravity_align_mode:=urdf` |
| **pointlio**（参考） | `uc_pointlio1` / `uc_pointlio2` | after（**该列不用到任何新代码**） | `lio:=pointlio` |
| 默认路径（改动后自证） | `uc_base_a1` / `uc_base_a2` | after | 同 baseline（`off`） |

**二进制换档方式**：`build/small_point_lio/{libsmall_point_lio.so, small_point_lio_node}` 成对替换
（本工作区是 `--symlink-install`，`install/…` 是指向 `build/…` 的软链），换完校验 md5 并写进
`meta.txt`；**同一个 launch、同一份参数文件，唯一变量 = 二进制**。

### 2.2 窗口口径（为什么不是"整跑平均"）

每次跑都做：`起栈 → 等 LIO 出第一帧 /cloud_registered → 再等 **20 s 仿真**（settle）→ 取 **20 s 仿真**的窗口`。
窗口内同时量：

* **CPU**：每 50 ms 轮询 `/proc/<pid>/stat` 的 **utime+stime（字段 14/15，USER_HZ=100）**，在窗口两端求差：
  * `cpu_s`（单核秒）、`%CPU = cpu_s / 窗口墙钟秒 × 100`（= top 的单核口径）
  * **`ms/frame` = cpu_s × 1000 / 窗口内 `/cloud_registered` 帧数**（该帧数 = LIO 实际处理的雷达帧数）
  * `cpu/sim_s = cpu_s / 窗口仿真秒`（仿真时间归一化，便于跨 RTF 比较）
  * 另一套"**全程**"口径：进程自启动到窗口结束的累计 CPU ÷ 自首个 `/clock` 起的仿真秒（**含初始化/200 帧门**）
* **RSS**：`/proc/<pid>/status` 的 `VmHWM`（峰值）与 `VmRSS`
* **速率**：`/cloud_registered`、`/odom`（= `/Odometry` 的 remap 名）、`/scan` 的条数 ÷ 窗口仿真秒
* **RTF** = 窗口仿真秒 / 窗口墙钟秒；**gzserver** 的 CPU/RSS 同口径（系统性对照：若 gzserver 也变了，说明不是 LIO 的事）
* **初始化耗时**：首个 `/clock`（≈世界开跑）→ 首个 `/odom`（LIO 出数）的**仿真秒**与**墙钟秒**（墙钟以 `ros2 launch` 启动时刻为 0）

### 2.3 精度口径（与 E1 实验**逐字相同**，便于跨文档比较）

* **主指标**：把**同一朵原始云** `/livox/lidar/pointcloud`（~12.0k 点、`livox_frame`，传感器系）经 TF 搬进 LIO 的世界系
  `p_world = T_world←base · T_base←livox · p_raw`，在 `r∈[0.8,8] m`、平面离原点 **0.05–0.60 m** 的点上做
  RANSAC（700 次、0.03 m）+ 内点 SVD 精修，**残差 = 地面法向 vs 世界系 +z 的夹角**。
  实现：`.tmp_gravtest/grav_probe.py`（采集）+ `.tmp_gravtest/raw_world_fit.py`（拟合）。
* **次指标**：`/cloud_registered` 自己 `frame_id` 上的同类拟合（`probe.json → residual_deg`）。
  `pointlio` 该云只有 146 点 ⇒ **拟合不出**（与 E1 文档一致）。
* **独立证人**：`tf2_echo odom base_link` 的 rpy（静止）、`/imu` 加速度均值在 `base_link` 里的方向（应 ≈0°）。

### 2.4 一次跑的命令（原文）

```bash
B=tools/scripts/tiltmount/uc_efficiency_bench.sh
# baseline（改动前的二进制；"before/after" 是 build/small_point_lio/ 下的 .so+可执行文件成对换档）
$B uc_base_b1 before world:=RMUL2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor spin_speed:=0.0
# aligned（hybrid）
$B uc_align_a1 after world:=RMUL2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid
# pointlio 参考（注意 watch 名：pointlio 的 comm 被内核截断成 pointlio_mappin）
WATCH=pointlio,gzserver,lio_tf_adapter $B uc_pointlio1 after world:=RMUL2026 \
    mode:=mapping lio:=pointlio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0
# 主指标（残差）：把 probe.json/clouds.npz 放进 .tmp_gravtest/<tag>/ 再跑（E1 实验的拟合脚本）
mkdir -p .tmp_gravtest/uc_align_a1 && cp .tmp_uc/uc_align_a1/{probe.json,clouds.npz} .tmp_gravtest/uc_align_a1/
python3 .tmp_gravtest/raw_world_fit.py uc_base_b1 uc_align_a1 uc_pointlio1
```

⚠️ **可复现性缺口（诚实写在前面）**：精度探针 `grav_probe.py` / 拟合脚本 `raw_world_fit.py`
是 **E1 实验的产物，仍在 gitignored 的 `.tmp_gravtest/`**（未在本提交里）。
`uc_efficiency_bench.sh` 用 `GRAV_PROBE_DIR=<dir>` 指向它们；找不到时**跳过精度探针**（效率部分照跑），
不会静默给出错的数。

`run_uc.sh` 内含隔离：`HOME=$REPO/.tmp_uc/home-<tag>`、按 tag 散列的 `ROS_DOMAIN_ID`、
探测空闲端口后占用 `GAZEBO_MASTER_URI`、`unset DISPLAY`、`setsid` 起栈（进程组 = 本 run）、
收尾用 `tools/scripts/lib/scoped_cleanup.sh` **只清本进程组/本 master URI**（**不做全机 pkill**）。
⚠️ 本机 `/tmp` 是**每条命令一份私有 tmpfs**，所以 HOME 与结果都落在工作区 `.tmp_uc/`（跨命令可见）。

---

## §3 三列效率 + 精度表（用户要看的那张）

### 3.1 主表：`small_point_lio`(aligned, adapter) vs `small_point_lio`(baseline) vs `pointlio`

格式：**中位数（逐跑区间）**；`n=2` 时区间 = 两次跑；aligned 列 `n=3`。全部 `settle=20 s / window=20 s(仿真)`。

| 指标（窗口 20 s 仿真） | **small_point_lio·aligned**（hybrid, adapter） | **small_point_lio·baseline**（改动前） | **pointlio**（参考） |
|---|---|---|---|
| **ms/帧**（`cpu_s×1000/帧数`） | **13.75**（13.75–13.87） | **13.85**（13.85–13.92） | **5.75**（5.75–5.80） |
| **%CPU**（单核） | **5.57**（5.54–5.62） | **5.80**（5.79–5.80） | **2.54**（2.47–2.54） |
| **CPU 秒 / 仿真秒** | **0.1375**（0.1375–0.1380） | **0.1385**（0.1385） | **0.0578**（0.0575–0.0580） |
| 全程 ms/帧（含初始化） | 13.47（13.38–13.47） | 13.55（13.55–13.73） | 6.12（6.10–6.12） |
| **RSS 峰值 VmHWM** | **33.8 MB**（33.1–35.0） | **34.8 MB**（34.7–34.8） | **83.0 MB**（83.0–83.6） |
| **RTF** | 0.405（0.401–0.409） | 0.419（0.418–0.419） | 0.442（0.427–0.442） |
| gzserver %CPU | 79.5（79.3–79.9） | 78.9（78.9–79.9） | 77.3（77.3–78.5） |
| gzserver RSS | 3075 MB | 3075 MB | 3075 MB |
| **`/cloud_registered`** | **10.00 Hz / 11864 点/帧** | 10.00 Hz / 11864 点/帧 | **10.00 Hz / 146 点/帧** |
| **`/Odometry`** | 10.00 Hz | 10.00 Hz | 10.00 Hz |
| `/scan` | 10.00 Hz、1462 束、**668–669 有限**、跨度 5.688 m | 同（1462 / 668 / 5.688） | 同（1462 / 677 / 5.666） |
| **残余地面倾角（主：原始云→TF）** | **0.384°**（0.346–0.488） | **30.158°**（30.158–30.220） | **0.19°**（0.111–0.267） |
| 残余（次：`/cloud_registered`） | 0.418°（0.389–0.506） | 30.110°（30.110–30.173） | 拟合不出（146 点） |
| **`tf2_echo odom base_link` rpy（静止）** | **[0.27, −0.20, 0.04]°** | **[30.04, −0.23, −0.05]°** | [0.26, 0.06, −0.06]° |
| **初始化**（仿真 / 墙钟） | 2.10 s / 11.6 s | 2.20 s / 11.4 s | **0.70 s / 8.0 s** |
| 额外 adapter 进程 | 0.18 % CPU / 0.45 ms/帧 / 15.7 MB | —（该节点不启动） | 0.15–0.18 % CPU |

**读这张表要知道的三件事**：

1. **契约改动本身：0 成本**。aligned 与 baseline 的 `ms/帧`/`%CPU`/RSS/RTF 全部落在跑间噪声内
   （噪声量级：ms/帧 ±0.15、%CPU ±0.2、RSS ±1 MB、RTF ±0.02）。多出来的是**另一个进程**
   （adapter，0.18 % CPU），不是 LIO 变慢。
2. **`small_point_lio` 在本配置下比 `pointlio` 慢 2.4×**（13.75 vs 5.75 ms/帧），**内存反而省 2.4×**
   （33.8 vs 83.0 MB），**初始化多 1.4 s 仿真**（2.10 vs 0.70 s，200 帧门 vs `Set_init`）。
3. **RTF 是 gzserver 的瓶颈**（gzserver 吃 77–80 % 单核，LIO 只吃 2.5–5.6 %）⇒ 三个档的 RTF
   都在 0.40–0.44，**LIO 快慢几乎不影响 RTF**。所以"提速"在本 bench 上只体现在 CPU 占用/发热/余量，
   不体现在仿真快慢。

### 3.2 `urdf` 档（对照，不进主表）

| 指标 | `gravity_align_mode:=urdf` |
|---|---|
| 残余（主 / 次） | **0.613°** / 0.581° |
| `tf2_echo odom base_link` rpy | [0.46, −0.23, 0.02]° |
| ms/帧 · %CPU · RSS | 14.14 · 5.80 · 34.2 MB |
| 日志 | `mode=urdf src=urdf R_world_body rpy=[-30.000 0.000 0.000]`（`Δ(URDF,IMU)` 不参与判决） |

`hybrid` 与 `urdf` 的残差差 0.23°（0.384 vs 0.613），**跑间噪声 ±0.15°** ⇒ 本 bench 上两者
**分不出高下**；与 E1 文档（0.474 vs 0.581，`urdf` 略优）方向相反，属噪声量级（见 §7 第 1 条）。

### 3.3 对齐效果的可复核证据（三个独立证人）

1. **估计器日志**（`launch.log`）：
   ```
   gravity_align_init_hybrid: Δ(URDF,IMU)=0.006° < 2° ⇒ 用 URDF 先验
   gravity_align_init: mode=hybrid src=urdf R_world_body rpy(deg)=[-30.000 0.000 0.000]
                       Δ(URDF,IMU)=0.006° world_gravity=[-0.001 -0.001 -9.810] body_source=imu_mean200
   ```
   三次 hybrid 跑的 Δ = **0.006° / 0.024° / 0.007°** ⇒ 全部走 INFO/URDF 分支（E1 记 0.011°）。
   ⚠️ `urdf` 档那一跑的 `Δ` 打印成 `-1.000°` = 该档**不计算互检**的哨兵值，不是"差 −1°"。
2. **TF 与 `/Odometry` 的合成恒等式**（sensor 档的定义式，实测逐项对上）：
   `/Odometry`（= `T_ol`，child `livox_frame`）rpy = **[−29.735, −0.200, 0.042]°**；
   adapter 的 `rpy` = **+30°**（URDF）；TF `odom→base_link`（probe 的 `lookupTransform`，与
   `tf2_echo` 同一条边）rpy = **[0.265, −0.200, 0.042]°**。
   ⇒ 姿态逐项相加成立（−29.735 + 30 = 0.265），yaw/pitch 原样透传 ⇒ **合成只做了一次**。
   baseline 同一跑里 `/Odometry` 与 TF **逐位相同**（都是 `[30.039, −0.234, −0.051]°`）⇒ 默认契约未变。
3. **IMU 独立证人**：`/imu` 加速度均值在 `base_link` 里与 +z 夹角 = **0.012–0.020°**（所有跑）⇒
   车真的水平、URDF 的 30° 链是真的；所以"对齐后 base_link 的 roll/pitch ≈0"不是拟合出来的假象。

---

## §4 默认路径（`gravity_align_mode:=off`）的回归证明

### 4.1 静态：launch 增量（`tools/scripts/gzsim/launch_increment_diff.py`）

```bash
python3 tools/scripts/gzsim/launch_increment_diff.py \
  --old .tmp_uc/old/bringup_sim.launch.py \          # = git show HEAD:...（改动前）
  --new src/rm_nav_bringup/launch/bringup_sim.launch.py \
  --arg mode=mapping --arg lio=small_point_lio --arg robot=robot11 \
  --arg robot11_mount=sensor --arg gravity_align_mode=off --arg world=RMUL2026 --arg localization=
# → actions: OLD 63 / NEW 66；共有 61；新增 5；消失/变成另一份 2；共有 Action 相对顺序：一致 ✅
```

| 变化 | 是什么 | 影响 |
|---|---|---|
| **+1** | `DeclareLaunchArgument gravity_align_mode`（新参数，默认 `off`） | 只多一个 `--show-args` 条目（98 → 99 个） |
| **+1** | 组合检查 `GroupAction(condition=mode!='off')`（内含 `OpaqueFunction`） | `off` 档 condition 为假 ⇒ **不执行、零动作** |
| **+1** | `lio:=small_point_lio` 的**第二份 Node 定义**（契约档） | `off` 档 condition 为假 ⇒ 不启动 |
| **+1** | `lio_tf_adapter_spl_node` | `off` 档 condition 为假 ⇒ 不启动 |
| **±1** | robot11 横幅 `LogInfo` 文本更新（加 `gravity_align_mode=` 收据） | 纯文本 |
| **±1** | SPL `GroupAction` 容器（里面多了上面那个 Node） | 容器哈希变了，**子 Action 内容未变** |

**逐字段证明**：把两棵树的 SPL 组取出来比较 —— 组内**原有那份 SPL Node 的序列化内容完全相同**
（参数表、remap、output 逐字段相同，唯一新增的是 `condition`），`LogInfo(参数文件=…)` 也逐字段相同。
⇒ `off` 档：**同一节点集合、同一参数、同一 Action 顺序**，只多了一个不执行的 Node + 一个新参数声明。

### 4.2 动态：**同一输入流重放，两个二进制输出逐位相同**（最强证据）

两次独立仿真跑**不可能**逐位相同（出生位姿/落定/初始化窗口不同：实测基线残差 30.110° vs 30.173°、
TF rpy 差 0.07°）。所以先录一份**输入流** bag，再分别喂给两个二进制：

```bash
# ① 录输入（只录输入，不录 LIO 输出）：68 帧 /livox/lidar(CustomMsg) + 676 /livox/imu + 1 /tf_static
.tmp_uc/record_input_bag.sh bagrec   # ⚠️ 该脚本未提交（与 uc_efficiency_bench.sh 同一套隔离口径，可照抄）
# ② 换二进制 → 起裸节点（同一份 mid360_sim_tuned.yaml，use_sim_time:=false）→ 重放 → 逐条记录发布值
tools/scripts/tiltmount/uc_replay_identity.sh .tmp_uc/bagrec/bag_in
# ③ 记录内容：ODOM（/Odometry 的 7 个 double + frame_id/child）、CLOUD（戳/frame/点数/md5(data)）、
#    TF（odom=>base_link 的 7 个 double），float 一律 %r = 17 位有效数字
```

**结果**：

```
before.txt 141 行（ODOM 47 / CLOUD 47 / TF 47）
after.txt  141 行（ODOM 47 / CLOUD 47 / TF 47）
排序后 diff → 0 行           # 逐位相同（每个发布数值 17 位有效数字全等，点云整段 md5 全等）
未排序 diff → 58 行          # **只是到达顺序**：TF/ODOM/CLOUD 三个话题在两个进程里的投递交错不同
```

⇒ **默认路径的发布数值与改动前逐位相同**（不是"在噪声内"，是**同一个字节序**）。

### 4.3 两跑噪声（用于判断上面的表里哪些差是噪声）

| 量 | 观测到的跑间噪声（同档两跑） |
|---|---|
| 主指标残余倾角 | **±0.15°**（基线 30.110/30.173；aligned 0.346/0.488） |
| `tf2_echo odom base_link` roll | ±0.09°（基线 30.04/30.11；aligned 0.20–0.30） |
| ms/帧 | ±0.15（基线 13.85/13.92；after-off 13.95/14.15） |
| %CPU | ±0.2 |
| RSS VmHWM | ±1 MB |
| RTF | ±0.02 |
| 初始化（仿真/墙钟） | ±0.1 s / ±0.3 s |
| 点云点数 | ±1 点（11864） |

---

## §5 行为变化 / 稳定性（逐项）

| 项 | 观测 | 判决 |
|---|---|---|
| `/cloud_registered` 点数 | 11864 帧帧一致（改动前后、对齐前后都 11863–11864） | **无变化** |
| `/cloud_registered` 帧率 | 9.90–10.00 Hz（随 RTF 抖动，三档同） | 无变化 |
| `/Odometry` 帧率 | 9.95–10.00 Hz（= 每帧一次，`publish_odometry_without_downsample: false`） | 无变化 |
| `/scan` 波束 | 1462 束、**668–669 有限**、跨度 5.688 m（baseline 与 aligned 各 668/669） | 差 **1 束（0.15 %）**，在噪声内；`pointlio` 677 束（+1.3 %） |
| 初始化耗时 | baseline 2.20 s 仿真 / 11.4 s 墙钟；aligned 2.10 s / 11.6 s；`pointlio` 0.70 s / 8.0 s | `off↔hybrid` **无差异**（200 帧门支配）；`pointlio` 快 1.4 s 仿真（无 200 帧门） |
| RSS | 33–35 MB（spl，含 adapter 的 15.7 MB 是**另一个进程**）；pointlio 83 MB | 无变化 |
| 发散 / 不稳 | **0 例**。10 次跑全部稳定：odom 位置停在出生点（`|p|` ≤ 0.2 m）、地面拟合内点 3513–5260、p95 0.002–0.007 m | 无发散 |
| `位姿一致性断言失败`（守卫 #2） | 所有 `body` 档跑 = **0** | 无违例 |
| `publish_tf:=false` + `odom_convention:=body`（launch 不用，但参数契约允许） | 单独验证过：节点起得来、`/Odometry`/`/cloud_registered` 照发、**`/tf` 一个 publisher 都没有**（`ros2 node info` 只有 `/Odometry`/`/cloud_registered`），日志明确写"需要另一个节点（lio_tf_adapter）来发 odom→base_link" | **可用**（`tf_broadcaster` 在该档根本不创建 ⇒ 无空指针风险） |
| 已知无关噪声 | 每次跑都有 `joint_state_publisher` 与 `map_session_announcer.py` 退出（本机 `/var/crash` 只读）；部分跑有一次 `spawn_entity: Spawn service failed`（"Entity pushed to spawn queue, but spawn service timed out" —— 实体随后仍然生成，点云/TF 正常） | **与本改动无关**（baseline 跑同样出现） |

---

## §6 与 E1 实验的对照 + "为什么 spl 比 pointlio 慢"

### 6.1 与 `docs/odom_base_link_practice_survey.md` §9 的数字对照

| 量 | E1 文档（2026-10-19，同世界同 spawn） | 本文（2026-10-20） | 一致性 |
|---|---|---|---|
| baseline 主指标 | 30.110° / 30.129°（打补丁/未打补丁） | **30.158° / 30.220°**（before），30.078° / 30.379°（after-off） | ✅ 同量级（+0.05…+0.25°，跑间噪声内） |
| `urdf` | 0.474° | **0.613°** | ✅ 同量级（+0.14°） |
| `hybrid` | 0.581° | **0.384°**（0.346–0.488） | 方向略优 0.2°，在噪声边缘 |
| `pointlio` | 0.166° | **0.19°**（0.111–0.267） | ✅ |
| TF rpy（baseline） | [30.03, −0.09, −0.13]° | [30.04, −0.23, −0.05]° | ✅ |
| 初始化（`fix=true`） | 7.6–8.2 s 墙钟 | **11.2–11.7 s 墙钟**（2.1–2.2 s 仿真） | ❌ 差 ~3.5 s：本轮包含 `mode:=mapping` 的完整栈（slam_toolbox/linefit/p2l/自击掩膜）与 28 线程机上的 RTF 差异；**同一轮内三档可比**，跨轮不可比 |
| RSS（spl / pointlio） | 33–35 / 83 MB | 33–35 / 83 MB | ✅ |

### 6.2 `small_point_lio` 2.4× 慢的来源（**能证**与**不能证**分开写）

**能证的部分**：

1. **每帧处理点数差一个量级**。`small_point_lio` 的配置（`mid360_sim_tuned.yaml`）是
   `space_downsample: false` + `point_filter_num: 1` + `min_distance: 0.5` ⇒ 每帧**约 3500 点**进逐点更新
   （该文件头写明：关降采样后"每帧 ~3500 点"；打开 0.2 m 体素降采样会掉到 ~620 点，但**轨迹从 7.4 m 变 30.2 m = 发散**）。
   而 `pointlio` 是 `point_filter_num: 3` + `filter_size_surf/map: 0.5` ⇒ 进 EKF 的点少一个量级
   （旁证：它发布的 `/cloud_registered` 只有 **146 点/帧**，spl 是 **11864 点/帧**）。
   ⇒ **ms/帧 不是"同等工作量"的比较**：spl 在这个 bench 上被**稳定性逼着**处理最多的点。
2. **初始化门的 1.4 s**：spl 等 200 帧 IMU（2.0 s 仿真）才初始化；`pointlio` 的 `Set_init` 不等
   （0.70 s 仿真就出数）。这与 §4.4 的结论一致（删门 = 5/8 发散，**不能删**）。
3. **`pointlio` 的 CPU 只占 2.5 %**，gzserver 占 77–80 % ⇒ 在本 bench 上"LIO 更快"换来的只是**余量**，
   不改变 RTF。

**不能证的部分（诚实边界）**：

* 没有量到两者**实际参与 EKF 更新的点云数**（只在配置里读到 `point_filter_num`/体素尺寸，以及发布点数这个
  间接指标）⇒ **"每点成本谁高"没有结论**。若要判"small_point_lio 的核心是否真的更快"，
  必须做**同工作量的 A/B**：把 pointlio 也调成不降采样（`point_filter_num: 1`、`filter_size_surf: 0`）、
  或把 spl 调成与 pointlio 同点数的档位（已知后者在本 bench **发散**）。**本轮没做**。
* 上游"small_point_lio 比 Point-LIO 快 2–3×"的说法**在本 bench 没有复现**（我们得到相反方向）。
  可能的原因（**未验证**）：① 上游比的是**关闭降采样前的默认配置**下的核心耗时；② 上游用自定义
  MID-360 驱动（`custom_mid360_driver`）而我们走 `livox_custom_msg`；③ 我们的 tuned 参数把
  `imu_meas_omg_cov` 调到 0.2（更"信 IMU"、每帧点更新迭代更多）。
* 本轮 `mode:=mapping`（含 slam_toolbox 等）——**导航栈在跑时的相对开销没测**（LIO 的占比只会更低，
  因为 nav2 会再抢 CPU，但没数据）。

---

## §7 未能验证 / 诚实边界

1. **精度结论的噪声**：残余倾角跑间 ±0.15°，`hybrid` (0.384°) 与 `urdf` (0.613°) 的差 0.23° 只比噪声大一倍；
   与 E1 文档的排序（urdf 更优）**方向相反**。样本 n=3（hybrid）/1（urdf）⇒ **不足以断言哪个档更准**；
   E1 的 n=8 档结论仍应视为更可信。
2. **`robot11_mount:=plugin` 档没测**：该档倾斜在插件里、`gravity_align_rpy` 自动算出 = `[0,0,0]`
   ⇒ E1 是**数学上的 no-op**（`hybrid` 的 Δ 会是 ~0，因为 IMU 与帧都水平）。本轮只跑了 `sensor` 档
   （**唯一"物理斜装 + 账也对"**的档，也是 E1 实验用的档）。
3. **`imu` 档没跑**（E1 实测 0.519–0.564°）；**错轴/无 IMU 的互检分支**（E1 的 `v8/v8b`）没复跑。
4. **导航栈（`mode:=nav`/`slam_nav`）下的行为没测**：
   * `nav2` 的 `min_obstacle_height` 高度带是按"`global_frame: odom`、地面≈−0.05"标定的；
     `odom` 一旦真的重力对齐，**scan 盘面在 odom 里的 z 会变**（本轮实测 aligned 档 `/Odometry` 的 z ≈ **−0.045 m**，
     baseline ≈ −0.08 m）。这对代价图是**更好还是更糟**，本轮**没测**（§J.5 那个"整张图变空"的坑要在 nav 档复测）。
   * `localization:=''` 时 `map≡odom` 静态桥仍在（`tf_bridge_spl_map_to_odom_node`），本改动没动它；
     **nav + 重定位槽 + 本契约**的组合没跑。
5. **`/Odometry` 语义变化的下游影响没测**：`publish_tf:=false` + `odom_convention:=sensor` 后，
   `/odom` 话题的内容从"合成 `T_ob`"变成"`T_ol`"。本仓 nav2 **主要走 TF**、且 `mode:=mapping` 下没有
   消费者读 `/odom` 的位姿；但没有做"谁订阅了 `/odom` 并把它当 `base_link` 用"的全仓审计。
6. **`odom_convention:=sensor` 下没有位姿一致性守卫**（守卫 #2 被跳过）。等价保障依赖
   "adapter 的 xyz/rpy 由 URDF 算出"这一点，**没有逐帧断言**（§13.2）。
7. **`rpy` 的 5 cm 口径**：spl 的 `/Odometry` 严格说是 `odom←IMU body`，但 `child_frame_id` 写的是
   `lidar_frame`（历史约定）；adapter 的 `xyz` 用 `T_imu←base_link`（含 0.05 m）。
   ⇒ 合成链里 livox↔imu 的 0.05 m **只算一次**，但**没有**做"逐点坐标恒等式"的复核
   （E1/§O 那套 `p_pub = T_{label←src}·p_src` 本轮没跑）。
8. **`check_contract` 不存在**（本任务明确不做）。所以 I2 仍只能靠"节点集合 + `ros2 node info`"证明，
   不是静态矩阵证明。
9. **CPU 数字的机器依赖性**：单机、单次、无 CPU 绑核/无 governor 控制；`%CPU` 是**单核百分比**。
   同档两跑的 ms/帧 差 ≤0.2 ms，但**跨机不可比**。
10. **`spawn_entity` 的偶发超时**（"Entity pushed to spawn queue… timed out"）在本轮部分跑出现，
    实体最终仍然生成（点云/TF 正常）。它可能让**该跑的出生位姿**与其它跑略有不同（= 残差 ±0.15° 的一部分来源）。

---

## §8 一句话给用户

> **改契约不需要付性能代价**（13.85 → 13.75 ms/帧，噪声级；多一个 0.18 % CPU 的 adapter 进程），
> **对齐也不需要**（同一跑里 0 成本），**并且把 30.1° 的残余倾角打到了 0.38°**。
> 但**"`small_point_lio` 比 `pointlio` 快"这件事在本 bench 上不成立**：13.75 vs 5.75 ms/帧、
> 5.6 % vs 2.5 % CPU —— 差别主要来自**每帧处理的点数**（我们为了不发散关掉了降采样）与
> **200 帧初始化门**，而不是本次契约改动。要真正回答"值不值得用 spl"，下一步应该做
> **同工作量的 A/B**（两边都关降采样/都开同尺寸体素），而不是继续比 `ms/帧`。
MD
wc -l docs/frame_contract_v3_measurements.md