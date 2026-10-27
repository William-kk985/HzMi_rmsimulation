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
---

## §9 第二轮（2026-10-19 晚，同一台机）：**同一个 `small_point_lio` 的「改前 vs 改后」三档 A/B** + `mode:=nav` 高度带复测

> **为什么有这一节（口径更正）**：§6.2 的"不能证"里登记的是"**同工作量 spl vs pointlio**"的 A/B。
> 2026-10-19 晚用户把口径改回来了，原话：
> *"spl 绝对比 pointlio 表现好，如果不好肯定是我们改的有问题，我是要对比两次 small_point_lio，
> pointlio 只是说来跟你解释的。"*
> ⇒ 本轮做的是**同一个 `small_point_lio` 的改前/改后**三档 A/B（**这才是用户要的那个实验**），
> `pointlio` 降级成 §9.8 的一条**脚注**（**不是**本轮的比较对象，也不进任何判决）。
> 本文**只追加**：§1–§8 一个字都没改；契约、默认值、参数文件本轮**一个都没动**。
>
> 时间：系统时钟 `2026-10-19 19:33–`（本轮 `meta.txt` 里逐跑有 `date=`）。上一轮 §1–§8 的标题写作
> **2026-10-20**，但它自己的 `meta.txt` 是 `2026-10-19T03:25` ⇒ 两轮其实是**同一天的两个时段**
> （凌晨那轮 / 晚间这轮），跨轮比较时请按"同机、同日、不同时段"读。

### 9.1 三档是什么（**唯一变量 = 二进制**；launch / 参数 / 世界 / 出生点逐字相同）

| 档 | 二进制（成对换 `.so` + 节点可执行） | `libsmall_point_lio.so` md5 | 运行期回读的三个新参数（`ros2 param get` 原样） | 这一档在问什么 |
|---|---|---|---|---|
| **V0 = 改前** | `.tmp_uc/bins/before`（构建于 `23bebb8`；该提交**是** `952a6da`(E1 补丁) 的后代、**不是** `8e5e928`(契约提交) 的后代） | `b4baa05b…` | `publish_tf` = **Parameter not set**<br>`odom_convention` = **Parameter not set**<br>`gravity_align_mode` = `off` | 改动前的默认路径（基线） |
| **V1 = 改后 · off** | `.tmp_uc/bins/after`（= 当前 HEAD 源码；`8e5e928` 之后 `src/rm_localization/small_point_lio` **零改动**，`git log 8e5e928..HEAD -- <pkg>` = 0 行） | `9550848…` | `publish_tf` = **True**<br>`odom_convention` = **body**<br>`gravity_align_mode` = `off` | **新代码在、但不生效** ⇒ 隔离"新代码存在本身" |
| **V2 = 改后 · hybrid** | 同上（同一个 `.so`） | `9550848…` | `publish_tf` = **False**<br>`odom_convention` = **sensor**<br>`gravity_align_mode` = `hybrid` | E1 生效 + sensor 约定 + `lio_tf_adapter` 在跑 ⇒ 隔离"E1 + adapter 的开销" |

* **`publish_tf` / `odom_convention` 在 V0 里是 `Parameter not set`** —— 这不是配置差异，而是
  **二进制里根本没有这两个参数**（`8e5e928` 才加）。这是"V0 真的是改前二进制"的**运行期证人**，
  比 md5 更直接（md5 只说文件不同，这一行说代码没有）。
* `small_point_lio_node` 可执行文件三档**同一个 md5**（`d940e573…`）⇒ 换档只换库；每跑**换档前后各校验一次**
  md5，跑完**自动还原 after**（`spl_ab_bench.sh` 的 cleanup）。
* **参数文件一个字节都没改**：三档都读同一份 `mid360_sim_tuned.yaml`
  （`space_downsample: false` / `point_filter_num: 1` / `min_distance: 0.5`），
  每跑的 `params.txt` 都回读了这三项，**9 跑逐项相同**。

### 9.2 方法（与 §2 逐字相同，只把"变量"从二进制换档换成"三档"）

* 世界 `RMUL2026`、`mode:=mapping`、`lio:=small_point_lio`、`robot:=robot11 robot11_mount:=sensor`、
  `gui:=False`、`spin_speed:=0.0`（静止）；**`settle=20 s / window=20 s`（仿真）**，与上一轮完全一致；
* **9 跑 = 3 轮 × 3 档，交错跑**（同一轮里 V0→V1→V2 相邻，间隔约 2 分钟）⇒ 机器漂移被"逐 cycle 配对"抵消；
* 探针（全部**只订阅**，不发布、不发指令）：
  * `tools/scripts/tiltmount/uc_procstats.py`（上一轮就有）：`/proc/<pid>/stat` 的 utime+stime 窗口差、
    `VmHWM`、`/cloud_registered`+`/odom`+`/scan` 速率、RTF、init、**每个进程单独记账**（LIO / gzserver / adapter）；
  * `tools/scripts/tiltmount/spl_points_probe.py`（**本轮新增**）：每帧 `/livox/lidar/pointcloud` 点数 +
    按源码口径复算的"进 EKF 点数"（§9.3）；
  * `.tmp_gravtest/grav_probe.py`（E1 那套，gitignored）：残余地面倾角 / `odom→base_link` rpy / `/Odometry` / IMU；
* 隔离：`HOME=<repo>/.tmp_ab/home-<tag>`、按 tag 散列的 `ROS_DOMAIN_ID`、先探端口再占用
  `GAZEBO_MASTER_URI`、`unset DISPLAY`、`setsid` 起栈（进程组 = 本跑）、
  收尾只用 `tools/scripts/lib/scoped_cleanup.sh` 清**本进程组/本 master URI**（**不做全机 pkill**），
  每跑 `timeout` 硬上限；9 跑全部 `rc=0`、`残留: / gz=0`。
* ⚠️ **本轮的机器不安静**（与上一轮最大的差别）：宿主机 `load average` 在 **3.0–7.4** 之间波动
  （本轮 `uptime` 实测；上一轮**没有**记 load，但它同一档两跑只差 ±0.15 ms/帧，暗示当时更安静）。
  这直接把跑间噪声从上一轮的 ±0.15 ms/帧放大到 **±0.6–2.4 ms/帧**（§9.6）。**所有三档都在同一时段交错跑**，所以"档间差"仍可比；
  但"跨轮绝对值"要按这条打折。

### 9.3 先证明"三档处理的是同一个工作量"：**进 EKF 的点数逐跑相同**

这一列是**用户上一轮问的那个问题的前置条件**（"同工作量才配比 ms/帧"），本轮直接量了：

* **源码口径**（`preprocess.cpp:26-48` + `small_point_lio.cpp:269-306`，本轮逐行复核）：
  * `dense_point_deque` = 时间戳合法的**全部原始点** ⇒ 只用来拼 `/cloud_registered`；
  * `point_deque` = 通过 `i % point_filter_num`、`min_distance/max_distance`、可选体素的点，
    **每个点触发一次 `kf.update_point()`** ⇒ **这才是"进 EKF 的点"**。
* **实测（9 跑完全相同）**：

| 量（窗口 20 s 仿真，200 帧） | 实测 |
|---|---|
| 每帧原始点（`/livox/lidar/pointcloud`） | **11896**（11810–12027） |
| 每帧 `/cloud_registered` | **11896**（11810–12027）—— 与原始点**逐帧同数**（它就是 dense 点，**不是** EKF 输入） |
| 每帧**进 EKF**（`point_filter_num=1` ⇒ 只被 `min_distance:0.5` 砍） | **8563**（8411–8725） |
| 被 `min_distance: 0.5` 砍掉的比例 | **28%**（11896 → 8563） |

⇒ 三档的**逐帧点数、EKF 更新次数、ivox 输入全部相同**，所以 §9.4 的 `ms/帧` 是**同工作量**比较
（不需要再像 §6.2 那样猜）。

⚠️ **顺带更正 §6.2 / `mid360_sim_tuned.yaml` 文件头里的"每帧 ~3500 点"**：那个数是按**旧插件 6393 点/帧**
写的；`robot11_mount:=sensor` 现在的输入是 **11896 点/帧**，进 EKF 是 **8563**（**2.4×**）。
这不改 §6.2 的结论（`pointlio` 那 146 点仍少两个量级），但"3500"在本轮配置下**偏小 2.4 倍**，
应当以本节这两个数为准。

### 9.4 主表（3 跑/档，中位数[区间]；3 档交错）

#### 9.4.1 逐跑（9 跑）

| tag | 档 | ms/帧 | %CPU | CPU/仿真秒 | LIO/gz | 全程 ms/帧 | VmHWM MB | RTF | init 仿真s/墙钟s | adapter %CPU | 残余倾角° | `/Odometry` roll° | `Odometry` z m | 进 EKF 点/帧 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `ab_v0_r1` | 改前 | 13.95 | 5.76 | 0.1395 | 0.0723 | 13.68 | 33.85 | 0.413 | 2.10/11.39 | — | 30.06 | +30.02 | −0.080 | 8563 |
| `ab_v0_r2` | 改前 | 13.80 | 6.15 | 0.1380 | 0.0793 | 13.18 | 33.86 | 0.446 | 2.10/11.36 | — | 30.10 | +30.06 | −0.081 | 8563 |
| `ab_v0_r3` | 改前 | 13.35 | 5.96 | 0.1335 | 0.0758 | 13.70 | 33.75 | 0.446 | 2.10/11.44 | — | 30.11 | +30.06 | −0.081 | 8564 |
| `ab_v1_r1` | 改后·off | 14.32 | 5.68 | 0.1425 | 0.0705 | 14.16 | 34.45 | 0.399 | 2.20/11.69 | — | 30.18 | +30.12 | −0.084 | 8563 |
| `ab_v1_r2` | 改后·off | **15.85** | 6.61 | 0.1585 | 0.0835 | 15.00 | 34.63 | 0.417 | 2.20/10.96 | — | 30.17 | +30.14 | −0.084 | 8563 |
| `ab_v1_r3` | 改后·off | 13.45 | 6.00 | 0.1345 | 0.0765 | 13.38 | 33.91 | 0.446 | 2.10/11.09 | — | 30.13 | +30.07 | −0.081 | 8564 |
| `ab_v2_r1` | 改后·hybrid | 13.25 | 5.66 | 0.1325 | 0.0729 | 13.00 | 33.82 | 0.427 | 2.10/11.52 | 0.17 | **0.37** | −29.74 | −0.045 | 8564 |
| `ab_v2_r2` | 改后·hybrid | 14.87 | 6.22 | 0.1465 | 0.0797 | 14.51 | 34.19 | 0.425 | 2.20/11.87 | 0.17 | **0.60** | −29.54 | −0.050 | 8564 |
| `ab_v2_r3` | 改后·hybrid | 13.35 | 5.65 | 0.1335 | 0.0725 | 13.00 | 35.11 | 0.424 | 2.20/11.00 | 0.15 | **0.56** | −29.70 | −0.046 | 8564 |

> `LIO/gz` = 同一跑里 LIO 的 `%CPU` ÷ gzserver 的 `%CPU`（**频率/负载归一化**：同机同时段的两个进程，
> 降频与争抢会同时作用在两者上，比值把这一层消掉）。全程 ms/帧 = 含初始化/200 帧门的另一套口径。

#### 9.4.2 每臂（median[min–max]）

| 臂 | n | **ms/帧** | %CPU | CPU/仿真秒 | LIO/gz | 全程 ms/帧 | VmHWM MB | RTF | adapter %CPU | 残余倾角° |
|---|---|---|---|---|---|---|---|---|---|---|
| **V0 改前** | 3 | **13.80 [13.35–13.95]** | 5.96 [5.76–6.15] | 0.1380 [0.1335–0.1395] | 0.0758 [0.0723–0.0793] | 13.68 [13.18–13.70] | 33.85 [33.75–33.86] | 0.446 [0.413–0.446] | — | 30.10 [30.06–30.11] |
| **V1 改后·off** | 3 | **14.32 [13.45–15.85]** | 6.00 [5.68–6.61] | 0.1425 [0.1345–0.1585] | 0.0765 [0.0705–0.0835] | 14.16 [13.38–15.00] | 34.45 [33.91–34.63] | 0.417 [0.399–0.446] | — | 30.17 [30.13–30.18] |
| **V2 改后·hybrid** | 3 | **13.35 [13.25–14.87]** | 5.66 [5.65–6.22] | 0.1335 [0.1325–0.1465] | 0.0729 [0.0725–0.0797] | 13.00 [13.00–14.51] | 34.19 [33.82–35.11] | 0.425 [0.424–0.427] | 0.17 [0.15–0.17] | **0.56 [0.37–0.60]** |

#### 9.4.3 逐 cycle 配对（**抗机器漂移的那张表**）

| cycle | V0 改前 | V1 改后·off | V2 改后·hybrid | Δ(V1−V0) | Δ(V2−V0) | Δ(V2−V1) |
|---|---|---|---|---|---|---|
| 1 | 13.95 | 14.32 | 13.25 | **+0.37** | **−0.70** | **−1.07** |
| 2 | 13.80 | 15.85 | 14.87 | **+2.05** | **+1.07** | **−0.98** |
| 3 | 13.35 | 13.45 | 13.35 | **+0.10** | **+0.00** | **−0.10** |
| 中位数 | 13.80 | 14.32 | 13.35 | **+0.37** | **+0.00** | **−0.98** |

### 9.5 判决：**改前 vs 改后（同一个 spl）**

1. **用户要的那条判据成立：V2（E1 全开）≤ V0（改前）在噪声内。**
   逐 cycle 配对的中位数 **Δ(V2−V0) = 0.00 ms/帧**（三对：−0.70 / +1.07 / +0.00），
   臂中位数 **13.35 vs 13.80（−0.45 ms，−3.3%）**，方向**略优**、幅度在噪声内（§9.6）。
   ⇒ **没有找到任何"改后变慢"的证据**；两轮独立数据同向（上一轮 §3.1：13.85/13.92 → 13.75/13.75/13.87）。
2. **V1（新代码在、但不生效）与 V0 的差 = 噪声，不是"新代码的存在成本"。**
   中位数 +0.37 ms（+2.7%），但三对分别是 +0.37 / +2.05 / +0.10，**同一臂内自己就差 2.4 ms**；
   而且 `git diff 23bebb8..HEAD -- <pkg>` 只有 **4 文件 / +177 −4 行**，逐帧热路径**执行的是同一段代码**
   （`off` 档：不查新参数分支、不建 broadcaster、不换 publish 回调），参数声明/一次 init 分支/
   两行 POD 快照都**不在逐帧路径上**。⇒ 这一档**测不出**真实差异，只能说明"没有 >8% 的变慢"。
3. **V2 比 V1 稳定地便宜约 1 ms/帧（−0.98 中位数，三对同向）——这是本轮唯一可测到的"档间真差"，方向是"更快"。**
   源码定位（`git diff` + 逐行）：
   * `odom_convention=body`（V0/V1）每帧都做 **`tf_buffer->lookupTransform(lidar_frame, "base_link", 帧戳)`**
     + 3 次 `tf2::Transform` 乘法 + `toMsg` + `tf_broadcaster->sendTransform`（`small_point_lio_node.cpp:160-249`）；
   * `odom_convention=sensor`（V2）**不做 TF 查询**、不合成 `T_ob`、不广播 TF，直接把状态写进 `/Odometry`
     （`odometry_convention.cpp:48-70`，注释原话："本档**不再查询** `livox_frame↔base_link` 的 TF"）。
   ⇒ 省下的正是这条"每帧一次 TF 查询 + 合成 + 广播"。**归因是结构性的（代码差就是这一处）**，
   本轮**没有**单独给 `lookupTransform` 做微基准（§9.9 第 1 条）。
4. **四个"可能变慢"的阶段逐个排除**（这是用户要求"若变慢要说清是哪一段"的答案）：

| 阶段 | 证据 | 判决 |
|---|---|---|
| **E1 数学**（`FromTwoVectors` + 写 `kf.x.rotation/gravity/acceleration`） | 只在 `!is_init` 那一次执行；init 耗时 V0 **2.10 s 仿真 / 11.36–11.44 s 墙钟** vs V2 **2.10–2.20 s / 11.00–11.87 s**（差在 200 帧门的抖动内） | **一次性，测不出成本** |
| **初始地图按 R 搬进世界系**（§4.2 第 2 步） | 同上，只在 init 那一次；全程 ms/帧（含 init）V2 **13.00** ≤ V0 **13.68** | **一次性，测不出成本** |
| **`lio_tf_adapter` 进程**（50 Hz 发 `odom→base_link`） | **另一个进程**单独记账：**0.15–0.17 % CPU**（不混进 LIO 的数） | **+0.15–0.17 % CPU**（≈ LIO 的 2.6%） |
| **`sensor` 发布路径**（`/Odometry`=T_ol、不发 TF） | 见上第 3 条 | **更便宜（−1 ms/帧量级）** |

5. **E1 本身真的生效了**：残余地面倾角 V0 **30.10°** / V1 **30.17°** / V2 **0.56°（0.37–0.60）**，
   过 §4.5 的 <0.7° 线；`/Odometry` 的 child 从 `base_link` 变成 `livox_frame`（V2），
   roll 从 +30.0° 变成 −29.7°（= `T_ol` 的物理值）—— 与 §3.3 的合成恒等式一致。
6. **稳定性**：9 跑全部不发散。窗口末 `odom` 与 Gazebo 真值的水平差 **5.33–5.46 m** ——
   与"出生点世界坐标 `(4.3, 3.35)` 的模 `√(4.3²+3.35²) = 5.45 m`"同一量级（odom 原点 = 出生点，
   ±13 cm 的跨跑差来自出生落定的微小差异，不是米级漂移）⇒ **三档都没有跑飞**；
   `/scan` 有限波束 668–669、跨度 5.69 m；`/cloud_registered` 9.85–10.00 Hz。
   ⇒ 三档的差别不是"哪一档跑飞了"造成的。
7. **契约面（顺带复核，不是本轮重点）**：V2 的 `publish_tf=False`、`/Odometry` child=`livox_frame`、
   `odom→base_link` 由 adapter 唯一发布（`tf2_echo` 在 V2 给出 **rpy [0.278, −0.171, 0.085]°、
   xyz [0.004, −0.128, −0.159]**）；V0/V1 的 `odom→base_link` 仍是 LIO 自己发（rpy 30.04/30.14°、
   xyz z = −0.081/−0.084）。
   ⚠️ `tf2_monitor` 本轮**没有**输出可解析的 broadcaster 清单（工具输出为空，见 §9.9 第 2 条），
   所以 I2 这条**只靠"节点集合 + `publish_tf=False` ⇒ 不创建 broadcaster"** 证，与上一轮 §4.1 同口径。

### 9.6 噪声带有多大，它把本轮结论限制到什么程度（**诚实边界**）

| 量 | V0 臂内 | V1 臂内 | V2 臂内 | 读法 |
|---|---|---|---|---|
| ms/帧 极差 | **0.60（4.3%）** | **2.40（16.8%）** | **1.62（12.2%）** | 单跑噪声远大于上一轮的 ±0.15 |
| LIO/gz 极差 | 0.0070（9.1%） | 0.0130（17.0%） | 0.0072（9.9%） | 归一化后仍在同一量级 |
| RTF 极差 | 0.034 | 0.047 | 0.003 | 三档 RTF 同量级（0.40–0.45）⇒ 不是"某档把仿真拖慢" |

* **能证**：任何 **≥ ~8%** 的"改后变慢"都会被本轮抓到（V2 的臂内极差 12.2%，但 V2−V0 的三对全在
  −0.70…+1.07 之间、中位数 0.00）。**两轮数据同向**（上一轮 13.85/13.92 vs 13.75/13.75/13.87）。
* **不能证**：**≤2–3% 的差别**（例如"省了 0.2 ms"或"多了 0.3 ms"）在本轮噪声下**分辨不出来**。
  要分辨，需要① 安静的机器（`load average` ≲1）或② 绑核 + 固定 governor（上一轮 §7 第 9 条同样的限制）。
* ⚠️ **单跑离群已登记**：`ab_v1_r2` 的 15.85 ms/帧（V1 臂内最高）出现在 `load average` 峰值附近；
  它在 §9.4.3 的配对里就是那个 +2.05。**去掉它，V1 的两次是 14.32 / 13.45**，与 V0 的
  13.95 / 13.80 / 13.35 **完全重叠**。⇒ 不建议引用"V1 median 14.32 比 V0 慢 3.8%"这个说法。

### 9.7 第二个实验：`mode:=nav` 下 `gravity_align_mode:=off` vs `hybrid` 的**高度带**（局部代价图会不会变空）

#### 9.7.1 为什么必须测这一条

`/scan` 是**二维平盘**，nav2 的 `obstacle_layer` 先 `projectLaser`（**z 强行置 0**）再用 TF 搬到
**代价图帧**（local 的 `global_frame: odom`）⇒ **每条波束在代价图帧里的 z = 那一帧 `scan` 帧原点的
odom z**，带子 `[min_obstacle_height, max_obstacle_height]` 量的是这个**常量**
（`nav2_params_sim_base.yaml:128/187`；§J.5/§K.1 的源码级机理）。
本仓 local/global 的 `obstacle_layer.scan` **都没有写 `min_obstacle_height`** ⇒ nav2 默认 **0.0**
（本轮运行期回读：local **0.0** / max **2.0**、global **0.0**）。
上一轮 §7 第 4 条只测到 `/Odometry` 的 z 从 −0.08 变 −0.045，**代价图侧没测**，登记为"待测"。
本节就是把它测掉。

#### 9.7.2 协议

* `mode:=nav`、`lio:=small_point_lio`、`localization:=gicp`（磁盘图 `map/RMUL2026.yaml` +
  `PCD/RMUL2026.pcd`，gicp 以 `initial_pose=[0,0,0]` 起步 = RMUL2026 map 系出生点相对系）、
  `robot:=robot11 robot11_mount:=sensor`、`gui:=False`、`spin_speed:=0.0`；
  **`settle=20 s / window=20 s`（仿真）**，`off` 与 `hybrid` **各 3 跑、交错**（共 6 跑，全部 `rc=0`）；
* 探针 `tools/scripts/tiltmount/nav_band_probe.py`（**本轮新增**，只订阅）：
  * `/scan` → 有限波束数；把每条波束 `(r·cosθ, r·sinθ, 0)` 用 TF **`odom←base_link`** 搬到 odom，量 **z**；
    与 `[0, 2.0]` 比较，数"带内/带下/带上"；
  * `/local_costmap/costmap_raw`（**nav2 原始代价**：254 lethal / 253 inscribed / 0 free / 255 unknown）
    → lethal / inscribed / free / unknown 计数、**车那格**（`base_link_fake`）、**到最近 lethal 的距离**、
    **车前方 2.0 m × ±0.6 m 窗口内的 lethal 数**、1 m 内 lethal 数；
  * `/global_costmap/costmap_raw` 同口径（对照）；
  * 同时 `uc_procstats.py` + `grav_probe.py`（后者给残余倾角）；
* ⚠️ 探针第一版在**单线程 executor 里阻塞等 TF（0.2 s）** ⇒ TF 订阅自己被饿死、
  124/124 次 lookup 全失败（已改：**非阻塞 + 退回最新帧**；车静止，最新帧与 msg 戳差 <1 mm）。
  本节所有数字都来自改好之后的 6 跑（`tf_fail_total = 0`）。

#### 9.7.3 表（3 跑/档，中位数[区间]）

| 指标 | **`off`**（odom 斜 30°，现状） | **`hybrid`**（odom 重力对齐 + adapter） | 判读 |
|---|---|---|---|
| `/scan` 有限波束/帧 | 678 [677.5–678] | 677 [677–677] | 两档同一批回波 |
| 波束 z(odom) **min** | **−0.2447** [−0.2451, −0.2436] | **−0.1708** [−0.1719, −0.1664] | — |
| 波束 z(odom) **p50** | **+0.2426** [0.2395, 0.2430] | **−0.1575** [−0.1577, −0.1563] | ★ 关键 |
| 波束 z(odom) **max** | **+2.5955** [2.5928, 2.5993] | **−0.1169** [−0.1332, −0.1125] | off 是斜面、hybrid 是水平面 |
| 在带 `[0, 2.0]` **内** | **580** [578–580]（**85.5%**） | **0**（**0%**） | ★★ |
| 带**下**（z < 0） | 61 [60–62]（9.0%） | **677（100%）** | ★★ |
| 带**上**（z > 2.0） | 36 [36–36] | 0 | off 的斜面冲出上界 |
| local **lethal** | **193** [193–200] | **0** | ★★★ |
| local **inscribed** | **8276** [8273.5–8293.5] | **0** | ★★★ |
| local **free** | 44565 [44532.5–44574.5] | **62500（整张 5 m×5 m）** | ★★★ |
| local **unknown** | 0 | 0 | 两张图都**不记 unknown**（`track_unknown_space=false`）⇒ "没看见"与"free"同值，**静默** |
| **车那格**（`base_link_fake`） | 0（free） | 0（free） | ⚠️ **两档都是 free** ⇒ 用"车那格"**看不出**这个 bug |
| **到最近 lethal 的距离** | **0.760 m** [0.7585, 0.7603] | **无（整张图没有 lethal 格）** | ★★★ |
| **车前方 2.0×1.2 m 内的 lethal** | **21** [20–21] | **0** | ★★★（"前面有障碍"这条信息整个丢了） |
| 车 1 m 内 lethal | 26 | 0 | — |
| 车在 odom 的 z（`base_link_fake`） | −0.0812 [−0.086, −0.080] | **−0.1590** [−0.1597, −0.1589] | 与上一行 z p50 同源 |
| **global** lethal / inscribed / free | 3713 / 16886 / 10176 | **3721 / 16879 / 10175** | ★ **全局图两档基本相同**（static_layer + STVL 不吃这条带子）|
| nav2 日志里的"掉出高度带"告警 | **没有**（`grep -E "dropped\|out of band\|min_obstacle"` = 0 行） | **没有** | ⚠️ **无任何提示** |

#### 9.7.4 判决：**`hybrid` 确实会把 `mode:=nav` 的局部代价图"变空"**（3/3 复现）

* **`hybrid` 档：677/677 条波束的 odom z 全部 < 0**（p50 **−0.1575**、max −0.1169）⇒
  `min_obstacle_height = 0.0` 把**整盘**丢掉 ⇒ `obstacle_layer.scan` **一条标记都不打**；
  local costmap **lethal 0 / inscribed 0 / free 62500（整张图）** ⇒ 车"看不见任何障碍"，
  前方 2 m 的 21 个障碍格**全部消失**（off 档有）。
* **`off` 档为什么"能用"**：同一条盘面在斜 odom 里是**一张 30° 斜面**，z ∈ [−0.245, +2.595]，
  **85.5% 的波束仍在 [0,2] 内** ⇒ 局部图有 193 lethal / 8276 inscribed / 最近障碍 0.76 m。**这是碰巧**。
* **这不是 E1 数学的错，也不是新 bug —— 是"odom 的 z 基准"与 nav2 的高度带假设之间的常量错位**，
  而且 §J.5 早就预言过这个坑（原文："如果平移也改成真值 ⇒ 一张图全空"，当时是**故意保留共轭平移**
  把默认档救回来的）。本轮把它量化了：

  | 量 | 值 | 来源 |
  |---|---|---|
  | odom 原点的 z 基准 | **初始化时刻的 IMU body**（`T_ol`），实测 z = **−0.045…−0.050 m** | `probe.json` 的 `odom.xyz[2]`（V2 三跑） |
  | IMU → base_link 的 z 杠杆 | **0.1137 m** | TF `base_link←imu_link`（`probe.json`，逐跑相同） |
  | ⇒ `base_link` 在 odom 的 z（**= `/scan` 盘面的 z**） | **−0.045 − 0.1137 = −0.1587 m** | 实测：TF `odom→base_link` z = **−0.159**、scan z p50 = **−0.1575**、`base_link_fake` z = −0.1590 ✅ 三处一致 |
  | `off` 档的同一量 | **−0.081 m** | 旧实现**故意保留**的共轭平移（§J.5），比物理真值**高 7.6 cm** ⇒ 恰好留在带内 |

  ⇒ 一句话：**`hybrid` 用的是物理正确的 `T_ob = T_ol·T_bl`，于是"雷达/IMU 在 base_link 上方
  11.4–15.7 cm"这件事第一次真的出现在 odom 里；而 nav2 的 `min_obstacle_height: 0.0` 假设
  "odom 的 z=0 ≈ base_link 起始高度"**。两者差 **−15.9 cm**，整盘波束就掉到带子下面去了。
  `global` 图不受影响（它吃 `static_layer` + STVL 的 3D 点，不用这条常量带）⇒ 症状是
  **"全局图有障碍、局部图空白"**，正好是那种"规划能规划、控制器却敢撞"的组合。
* **`localization:=gicp` 不是变量**：两档的 gicp/`map→odom` 各跑各的，但**局部图是 `odom` 系、
  完全不看 `map→odom`** ⇒ 上面 6 跑的结论与定位好坏**无关**（`global` 的 3713 vs 3721 只差 8 格也是同一原因）。
  作为第二数据点，`amcl` 各 1 跑（§9.7.5）给出**同向**结果。

#### 9.7.5 第二数据点：`localization:=amcl`（n=1/档，仅用于"换一个重定位器结论不变"）

| 指标 | `off` + amcl | `hybrid` + amcl |
|---|---|---|
| `/scan` 波束 z(odom) p50 / 带内 | **+0.2431 / 580** | **−0.1560 / 0** |
| local lethal / inscribed / free | **193 / 8293 / 44534** | **0 / 0 / 62500（整张）** |
| 最近 lethal / 前方窗口 lethal | **0.760 m / 21** | **无（图里没有 lethal）/ 0** |
| 车那格 / unknown | 0（free）/ 0 | 0（free）/ 0 |
| global lethal / inscribed / free | 3872 / 17375 / 9694 | 3912 / 17339 / 9708 |

⇒ 换重定位器**不改**结论（与 §9.7.4 同向）。

#### 9.7.6 最小修法（**只建议，本轮不实现**）

| 候选 | 具体改法 | 优点 | 代价 / 风险 |
|---|---|---|---|
| **(a) 首选：给 odom 的 z 定基** | 在 `lio_tf_adapter` 发布 `T_ob` 时加**一个常量 z 偏置**，让 `base_link` 起始 z ≈ 0（即把上面那 **−0.159 m** 消掉；等价于"odom 原点 = base_link 初始位置"而不是"雷达/IMU 初始位置"） | ① 这正是 `nav2_params_sim_base.yaml:187` 写死的假设（"odom 的 z=0 ≈ base_link 起始高度、地面约 −0.05"）；② **只影响 hybrid 档**（adapter 只在 hybrid 存在）⇒ 默认路径零风险；③ 一处改动同时修好 local + global + 所有 z 参数（`linefit.sensor_height` / STVL 的 `origin_z`）的语义；④ 不新增帧（§3 的"无 `odom_grav`"还是成立） | odom 不再是"纯净的 LIO 输出"，要用参数门控；需要单独 A/B（`off` 档的 z 是 −0.081，改了以后两档的 z 基准会差 8 cm，要确认不把 off 档弄坏） |
| **(b) 兜底：改高度带** | `obstacle_layer.scan.min_obstacle_height: 0.0 → **−0.25**`（local + global **两处**） | 一行；覆盖 −0.159 并留 **9 cm** 余量；在 2D 链路上这个键本来就只是"整盘常量闸"，改它**不会**改变"什么算障碍"的语义（`plugin` 档的 +0.057…+0.084 仍在带内） | 两个键都在 `src/rm_navigation/**/params`（**本任务禁改**），且**默认模型共用** ⇒ 必须单独 A/B；且它只是"把闸门挪开"，没有修好 odom 的 z 语义（下一个吃 z 的消费者还会踩） |
| **(c) 被否掉：`odom_grav` 中间帧** | 在 `odom` 与 `base_link` 之间插一个重力对齐帧 | 概念最干净 | 多一层帧树（契约 §2 明确"无 `odom_grav`"）；nav2 两张图的 `global_frame` 仍要改；收益与 (a) 等价而代价更大（§6.3 of `docs/odom_base_link_practice_survey.md`） |

**建议顺序**：(a) 先做（一次性把 z 基准对齐 nav2 的假设）→ 然后用 (b) 作为"不想动 odom 语义"时的兜底。
**两者都必须单独 A/B**，因为①它们改的是**别人任务的参数目录**（`src/rm_navigation/**/params`）；
②`off`/`plugin`/`urdf` 档现在是**碰巧能用**，动带子/动 z 基准都可能把它们弄坏。
**本轮一个字都没改**：上面 6+2 跑用的都是仓库现状（`min_obstacle_height` 运行期回读 = 0.0）。

### 9.8 脚注：`pointlio` —— **不是**本轮的比较对象（单跑，只做参照）

> 用户已把口径说清楚：*"pointlio 只是说来跟你解释的"*。所以本轮**只跑 1 次**、只作脚注，
> **不进任何判决/任何表**，也不再做 §6.2 登记的那个"spl vs pointlio 同工作量 A/B"。

| 量（单跑，`mode:=mapping`，同世界/同出生点/同 settle+window） | 值 |
|---|---|
| ms/帧 · %CPU | **4.15 · 1.93 %** |
| RSS 峰值 VmHWM | 83.25 MB |
| RTF | 0.466 |
| 初始化（仿真 / 墙钟） | **0.70 s / 8.60 s** |
| `/cloud_registered` 点/帧 | **146**（136–155） |
| 每帧原始点 | 11896（与 spl 同一份输入） |
| 它**进 EKF** 的点/帧 | **= 146**（`pointlio` 的 `/cloud_registered` 就是 `feats_down_body` 的 world 版，代码可直读） |

* 与 spl 的 **8563 点/帧**差 **58×** ⇒ `ms/帧` **不是**同工作量比较（这正是 §6.2"不能证"那一条，
  本轮**不再**据此下任何结论）。
* 与上一轮 §3.1 的 pointlio（**5.75 ms/帧、2.54 %CPU**）差 28%（4.15/1.93）—— 这是**机器状态**
  （CPU 频率/负载，本轮 load 3.0–7.4 波动）的差，**不是算法差**；同一台机、同一天的两个时段尚且如此，
  ⇒ 再次说明**跨跑绝对值不可比、只有交错配对比值有意义**（§9.6）。
* 本轮 pointlio 那跑的 `grav_probe` 主指标**没拟合出来**（`residual_deg=null`；
  `/cloud_registered` 只有 147 点，原始云→TF 那一路也没出平面；**原因未查**）⇒ pointlio 的倾角
  本轮**没有**新数据，E1 文档的 **0.166°** 仍是唯一来源。

### 9.9 复现命令（逐字）

```bash
# ① 改前/改后三档（每跑一条；本轮 9 跑 = 3 轮 ×（V0→V1→V2）交错）
SETTLE=20 WINDOW=20 PROBE_TIMEOUT=300 PROBES="grav points" \
  tools/scripts/tiltmount/spl_ab_bench.sh ab_v0_r1 before \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off

SETTLE=20 WINDOW=20 PROBE_TIMEOUT=300 PROBES="grav points" \
  tools/scripts/tiltmount/spl_ab_bench.sh ab_v1_r1 after \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off

SETTLE=20 WINDOW=20 PROBE_TIMEOUT=300 PROBES="grav points" \
  tools/scripts/tiltmount/spl_ab_bench.sh ab_v2_r1 after \
  world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid

# ② nav 高度带（off / hybrid 各 3 跑；amcl 第二数据点只把 localization:=gicp 换成 :=amcl）
SETTLE=20 WINDOW=20 PROBE_TIMEOUT=560 PROBES="navband grav" \
  tools/scripts/tiltmount/spl_ab_bench.sh nav_off_gicp_r1 after \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off

SETTLE=20 WINDOW=20 PROBE_TIMEOUT=560 PROBES="navband grav" \
  tools/scripts/tiltmount/spl_ab_bench.sh nav_hyb_gicp_r1 after \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid

# ③ 汇总（逐跑 + 每臂 median[min–max] + 逐 cycle 配对 + 相对基线差）
python3 tools/scripts/tiltmount/spl_ab_table.py --root .tmp_ab --prefix ab_
```

本轮**新增的四个工具**（都已提交，`tools/scripts/tiltmount/`）：
`spl_ab_bench.sh`（三档通用隔离跑 + 二进制换档/还原 + 探针编排）、
`spl_points_probe.py`（每帧原始/进 EKF 点数）、
`nav_band_probe.py`（`/scan` 在代价图帧的 z + local/global 代价图计数）、
`spl_ab_table.py`（出表）。原始数据：`.tmp_ab/<tag>/`（gitignored；每跑 `meta.txt` 有
`date`/`git_head`/两个 md5/`launch_args`，`proc.json`/`points.json`/`probe.json`/`navband.json` 是探针原始输出）。

### 9.10 本轮没能验证 / 诚实边界

1. **`lookupTransform` 的单项成本没有微基准** ⇒ "V2 比 V1 便宜 ~1 ms/帧"的**归因是结构性的**
   （两档的逐帧代码差只有那一处：TF 查询 + `T_ob` 合成 + 广播），**不是**逐项计时拆出来的。
2. **`tf2_monitor` 本轮没给出可解析的 broadcaster 清单**（输出为空）⇒ "`odom→base_link` 唯一发布者"
   只靠 `publish_tf=False`（不创建 broadcaster）+ 节点集合 + `/tf` 的 publisher 数佐证，与上一轮 §4.1 同口径。
3. **机器不安静**（宿主机 load 3.0–7.4）：本轮只能排除 **≥8%** 的"改后变慢"；**≤2–3%** 分辨不出。
   无绑核、无 governor 控制（与上一轮 §7 第 9 条同一限制）。
4. **nav 侧只测了 `localization:=gicp`（3 跑/档）+ `amcl`（1 跑/档）**；
   `mode:=slam_nav`、以及 `global_obstacle:=obstacle_layer`（把**全局**图也切到 2D `/scan` 的那一档）
   **没测** —— 后者吃**同一条常量带**，**预期同病**，登记为待测。
5. **修法 (a)/(b) 都没有实现**（本任务只做实验）⇒ 它们的收益/风险是**设计判断**，不是实测。
6. **只测了 `robot:=robot11 robot11_mount:=sensor`**（`plugin`/`urdf`/默认模型没测）。
   机理上 `plugin`/`urdf` 的盘面在 `livox_frame`（z ≈ −0.09，§J.5），一旦接上 adapter（hybrid）
   同样会掉出带子 ⇒ **预期同病，未实测**。
7. `hybrid` 档启动阶段 `planner_server` 有少量 `map→odom` 外推告警（off 档 118 条 / hybrid 2 条），
   与高度带无关，**未归因**（两档 map→odom 由同一个 gicp 出，差异可能只是 TF 时序）。
   > **订正（§14，2026-10-19 复现）**：**"off 118 / hybrid 2" 是错的**。那些外推告警是
   > **同一个 gicp 首帧门死锁**的像：两档都 **`采纳 0/N`**、`map→odom` 只在启动后几秒存在（= 初始值，
   > 从未被采纳帧修正）⇒ 告警**条数**只取决于 nav2 跑了多久/何时开始丢，**与 off/hybrid 无关**
   > （§10.4 的 6 份 `zf2_*` 日志、§12.4 的 2 份 `exp12_gscan_*` 日志都是 `采纳 0/N`）。
   > 本轮实测同一配方：**改前 hybrid 271 条外推 + 42 条 `Message Filter dropping`；修门后 3 / 0**
   > （见 §14.2 与 §14.5；修法见 §14.4）。

### 9.11 一句话给用户

> **① 改前 vs 改后（同一个 spl）：没有变慢。** 交错 3 轮 × 3 档、同工作量（9 跑**进 EKF 都是 8563 点/帧**）：
> 改前 **13.80**、改后-off **14.32**、改后-hybrid **13.35** ms/帧；**逐 cycle 配对的 Δ(hybrid − 改前) 中位数 = 0.00 ms/帧**
> （−0.70 / +1.07 / +0.00），方向略优；E1 把残余倾角从 **30.1° 打到 0.56°**，adapter 是**另一个进程**、
> **0.15–0.17 % CPU**。唯一可测到的档间真差是 `sensor` 档**每帧省掉一次 TF 查询/合成/广播**（≈ −1 ms/帧）。
> **② 但 `hybrid` 会把 `mode:=nav` 的局部代价图打空**（3/3 复现）：scan 盘面在 odom 的 z = **−0.157 m**
> < `min_obstacle_height` **0.0** ⇒ **677/677 条波束全部掉出带子** ⇒ local costmap **lethal 193→0、
> inscribed 8276→0、整张 62500 格全 free**、前方 2 m 的 21 个障碍格全丢（global 图不变、nav2 **无任何告警**）。
> 最小修法：**(a)** 给 `lio_tf_adapter` 的 `T_ob` 加一个常量 z 偏置让 `base_link` 起始 z≈0（首选，只影响 hybrid 档），
> 或 **(b)** 把 `obstacle_layer.scan.min_obstacle_height` 改成 **−0.25**（一行，但在禁改的 nav2 参数目录、且默认模型共用）——
> **两者都要单独 A/B，本轮一个字都没改。**

---

## §10 第三轮（2026-10-21）：**odom 的 z 原点 = 初始化时刻的地面**（E1b）

> **本轮只新增本节**：§1–§9 与 §11（parser，父提交 `023d2e8`）一个字都没改；为保持编号顺序，
> §10 插在 §9 与 §11 之间（不重排、不改写任何既有节）。契约规范见
> `docs/frame_contract_v3.md` **§4.7**（任务书里叫 §4.3，但那个编号已被「来源与互检」占用）。
> 代码 commit **`1b5db4b`**；原始数据 `.tmp_ab/zf2_*/`（gitignored）；每跑 `meta.txt` 有
> `date`/`git_head`/两个 md5/`launch_args`/`ros_domain_id`/`gazebo_master_uri`。

### 10.1 为什么必须修（一句话）与实现

E1（`952a6da`）只设初始**旋转**：`odom` 重力对齐了，但原点还停在传感器壳上 ⇒ hybrid 档
scan 盘面在 `odom` 的 z = **−0.158 m**，nav2 `obstacle_layer.scan.min_obstacle_height: 0.0`
把 **677/677** 波束整盘丢掉（§9.7：local lethal **193→0**、整张 **62500 格全 free**、
前方 2 m 的 **21** 个障碍格全丢、**零告警**）。E1b 补上**平移**：

* 节点新参数 `gravity_align_z_origin ∈ {sensor(默认=旧行为), ground}` + `gravity_align_h`（米）；
* `ground` 档初始化时 `position.z = h − (R_ws·t_IL).z`；
  * 状态原点是 **IMU body**，livox 原点在 body 系里是 `t_IL = extrinsic_T = (0,0,0.05)`；
  * robot11 sensor 档：`h = 0.102499 + 0.157028 = 0.259527`、`(R_ws·t_IL).z = 0.05·cos30° = 0.043301`
    ⇒ `position.z = 0.216226`；
  * ⇒ 车体水平、URDF=实物时 **ground z=0、base_link z=+0.102499、livox_frame z=+0.2595**
    （精确值 0.102504 / 0.259532，差 ≤5 µm 来自四舍五入）。**由构造**：
    初始化日志 state_z = 0.216226，减旋转后的 `t_imu←base` z 杆臂 0.113722 ⇒
    `odom→base_link` z = **+0.102504**（窗口末端的 tf2_echo 因 §10.4.3 的漂移是 +0.051…+0.057）。
* **初始地图点用同一个 `kf.x.position` 搬**（出点公式 `R·(R_L_I·p+T_L_I)+position`）——
  不另写一份 `+h`。只改旋转不改平移 = 本 bug；反过来只改平移不改旋转也不行。
* 默认 `sensor` ⇒ 这段代码**整段不执行**，`position` 保持 `reset()` 后的 0；
  实现在独立 TU `src/gravity_align_ground.cpp`（本包 `-ffast-math`，不把新浮点代码放进 hot path TU）。

### 10.2 launch：`h` 从 URDF 算（逐模型，不手填）

`_gravity_align_h()` = `_HZMI_BASE_LINK_HEIGHT_M[robot]` + **URDF 里的 livox z 偏置**
（与 `_GravityAlignRpy` / adapter 外参**同源**；`livox_raise_m` 自动跟随）：

| `robot` | `base_link` 原点离地 | `livox_frame` z 偏置 | **`h`** | 与 linefit 交叉 |
|---|---|---|---|---|
| `robot11` | 0.102499（轮 mesh 最低点） | 0.15702816968305 | **0.25952717** | `sensor_height: 0.2595` ✅ |
| `hzmirm` | 0.0（轮 r=0.08 心 z=0.08） | 0.6 + 0.2·cos(`turret_pitch_deg`) | **0.80000000**（pitch=0） | `0.80` ✅ |
| 默认模型（`robot:=''`） | 0.06（轮 r=0.06 心 z=0） | 0.175 | **0.23500000** | `0.226`（Gazebo 实测 = 轮胎下沉 9 mm）✅ |

只加在 `gravity_align_mode != off` 的那份 Node 上。运行期回读（`zf2_hybrid_r1/params.txt`）：

```
/small_point_lio gravity_align_mode      = hybrid
/small_point_lio gravity_align_rpy       = [-30.0, 0.0, 0.0]
/small_point_lio gravity_align_z_origin  = ground
/small_point_lio gravity_align_h         = 0.25952717
/lio_tf_adapter   rpy                    = [0.523598776, -0.0, -0.0]   # 弧度（setRPY 的约定）
```

`off` 档回读（`zf2_off_r1/params.txt`）：`gravity_align_z_origin = sensor`、`gravity_align_h = 0.0`
（= **节点默认值**）⇒ launch 没给 off 档传这两个键，默认路径参数表不变。

### 10.3 运行时互检：sim 里立刻抓到 **6.9 cm** 的 URDF-vs-可见地面差

初始化日志（`zf2_hybrid_r1`，逐字）：

```
[INFO] gravity_align_z: z_origin=ground h(URDF)=0.259527 m lever_z=(R_ws·t_IL).z=0.043301 m
       ⇒ state_z=0.216226 m（odom 的 z=0 = 初始化时刻的地面；初始地图点用同一平移）
[ERROR] gravity_align_h_check: h_urdf=0.2595 m h_meas=0.1894 m |Δ|=0.0701 m >= 0.05 ⇒ ERROR
        （URDF 的 livox 离地高度与实物不符，或出生时不在平地上；inliers=15547 tilt=0.05°
         resid_p50=0.0002 resid_p95=0.0018）—— **仍用 URDF 值 0.2595 m 定基**（确定性；实测只作诊断）
```

* **拟合质量**：15547 内点、法向与重力先验差 **0.05°**、残差 p50 **0.2 mm** / p95 **1.8 mm**
  ⇒ 不是拟合噪声；是**真的**量到了 0.1894。
* **根因**（独立探针 `.tmp_ab/gt_rootcause_probe.py`）：用 `/odom_ground_truth` +
  `TF base_link←livox_frame` 把**同一朵云**搬到 **world** 再拟合 ⇒ world 地面
  **z = 0.1183**（3434 内点、p50 0.5 mm），而 URDF 的轮底平面在
  `0.15216 − 0.102499 = 0.04966` ⇒ **差 6.9 cm**。即：RMUL2026 出生点处机器人实际“陷进”
  地面 mesh / 该处 mesh 高于名义地面，雷达的**可见**离地高度是 **0.190**，不是 URDF 的 0.2595。
* **这正是互检的用途**（同 E1 的 Δ(URDF,IMU) 模式）：**loud, never silent**。阈值
  <0.03 INFO / 0.03–0.05 WARN / ≥0.05 ERROR；**定基仍用 URDF**（确定性），所以
  odom z=0 = **URDF 地面**（名义 world 0.0496），可见地面 mesh 在 odom z≈**+0.069**。
* **成本**：`gravity_align_init` → `gravity_align_h_check` 两条日志时间差 **2.6 ms**
  （6294.6026 → 6294.6052 s），一次性。

### 10.4 验收 A/B（`mode:=nav`、`localization:=gicp`、3 跑/档、交错）

协议与 §9.7 **逐字相同**：`world:=RMUL2026`、`robot:=robot11 robot11_mount:=sensor`、
`gui:=False`、`spin_speed:=0.0`、`settle=20 s / window=20 s`（仿真）、`lio:=small_point_lio`、
`localization:=gicp`。**唯一变量 = `gravity_align_mode:=off` vs `hybrid`**（后者用默认
`gravity_align_z_origin:=ground`）。同一个 `zfix` 二进制（`libsmall_point_lio.so` md5
`58bebfb8…`）；每跑记录 **launch md5 `0161033b…`**（= `1b5db4b` 的冻结版本，**无** parser 代码）。
探针：`navband grav points` + `uc_procstats`（与 §9.7 同口径；`points` 只订阅）。

#### 10.4.1 主表（3 跑/档，median[min–max]）

| 指标 | **`off`**（3 跑） | **`hybrid` + z-fix**（3 跑） | 任务书目标 | 判读 |
|---|---|---|---|---|
| `/scan` 有限波束/帧 | 677 [677–678] | 678 [678–678] | — | 同一批回波 |
| 波束 z(odom) **p50** | **+0.243** [+0.239,+0.243] | **+0.058** [+0.057,+0.058] | ≈+0.259 | **≠+0.259**；见 §10.4.3 口径澄清 |
| 波束 z(odom) min–max | −0.244…+2.595 | **+0.045…+0.099** | — | off 是 30° 斜面；修复后是**水平面** |
| 在带 `[0,2]` **内** | 579 [578–580]（85.4%） | **678（100%）** | ≈580 | **超集**，不是缺失（见 §10.4.3） |
| 带下 / 带上 | 60 [60–62] / 36 [36] | **0 / 0** | — | ★ |
| **`base_link` z(odom)** | −0.081 [−0.085,−0.081] | **+0.057** [+0.051,+0.057] | ≈+0.1025 | 窗口值；**init = +0.1025**（差 = 既有 LIO z 漂移，见 §10.4.3） |
| local **lethal** | **193** [193–210] | **288.5** [288–345] | ≈193 | 非空（病因是 **0**）；多出来的格 = 修复后多进带的 99 条波束 |
| local **inscribed** | 8280.5 [8279.5–8324] | 8452 [8450–8838] | — | 同量级 |
| local **free** | 44546 [44523.5–44560] | 43312.5 [43291–43336] | ≈44565 | 比 off 少 1233 格（≈ 多出的 lethal/inscribed） |
| **最近 lethal** | **0.760** [0.759–0.760] | **0.769** [0.768–0.771] | ≈0.76 | ✅（差 9 mm） |
| **前方 2.0×1.2 m lethal** | **21** [21–22] | **18** [17–23] | ≈21 | 同量级（差 3 格 / 14%）；**不是 0** |
| 车 1 m 内 lethal | 26 [26–27] | 23.5 [23–32] | — | 同量级 |
| `odom→base_link` rpy | [30.06…30.20, −0.12…−0.18, −0.07…−0.21]° | **[0.30…0.43, −0.21…−0.38, −0.04…+0.05]°** | ≈[0.27,−0.20,0.04] | ✅（在 E1 残差 0.5° 内） |
| `/Odometry` z（state） | —（off 发合成 `T_ob`） | +0.171 [+0.166,+0.171]（init 0.216226） | — | 漂移 −0.045，见下 |
| local **ground inclination** | 30.115 [30.077–30.227]° | **0.518** [0.488–0.575]° | <0.7 | ✅ |
| global lethal/inscribed/free | 3713 / 16886 / 10176 | 3713 / 16886 / 10176 | — | **逐值相同**（全局图不吃这条带） |
| nav2 "掉出高度带"告警 | 0 行 | 0 行 | — | 这条链**本来就没有**告警 |

#### 10.4.2 判决：**功能上恢复到 `off` 基线；三行数值目标不能逐字相等，原因已定位**

1. **病治好了**：修复后 **678/678 波束全部在带内**（off 只有 85.4%），local costmap 从
   §9.7 的 **lethal 0 / free 62500 / ahead 0** 回到 **lethal ≈289 / 最近 0.769 m / ahead ≈18**，
   与 off（193 / 0.760 / 21）**同一量级**，且障碍信息**比 off 更多**（多 99 条波束进带）。
2. **三行"≈"对不上的口径原因**（不是回归）：
   * `scan z p50 ≈ +0.259`：**+0.2595 是 `livox_frame` 的 z**，不是 scan 平面。`/scan` 的
     `frame_id = base_link`（linefit/p2l 都在 base_link 里）⇒ 平面 z = base_link z：
     **init +0.1025**、窗口 **+0.057**。+0.259 只有在 scan 帧 = livox_frame 时才成立。
   * `in-band ≈ 580`：off 的 580 是**子集**（60 条带下 + 36 条带上被丢掉）。修复后是水平面，
     **678 条全进带** ⇒ 正确值是 **678**，不是 580。
   * `lethal ≈ 193 / free ≈ 44565`：lethal 格数由"进带波束集合"决定；678 条 vs 580 条 ⇒
     lethal **288.5**、free **43312.5**。**离 193 越远，恰恰说明多进的 99 条波束真的在打障碍**
     （最近 lethal 与 ahead 窗口都回到 off 基线可佐证）。
3. **`base_link z` 窗口 +0.057 而不是 +0.1025**：初始化是精确的 **+0.102504**（由构造），
   窗口末端 `odom` 的 state z 从 0.216226 掉到 0.171（**−0.045**）。这是**既有的 LIO z 漂移**，
   不是本修复引入：§9.4 的 hybrid `/Odometry` z = −0.045…−0.050 就是从 init 0 掉下来的同一个量。
   它是 O(4.5 cm) 的确定性偏置，仍在 nav2 带内（+0.057 > 0）。

#### 10.4.3 为什么 `base_link z` 在窗口掉 4.5 cm：**不是物理下沉**

同一跑里两处独立测量：初始化时 `h_meas = 0.1894`（雷达-可见地面垂距），窗口末端独立探针
（§10.3 的 `gt_rootcause_probe.py`）量到 **0.1909** ⇒ 雷达相对地面的物理高度**没变**（差 1.5 mm）。
⇒ `odom` 的 state z 下降 4.5 cm 是**估计器**的 z 漂移，不是车沉了。它在本轮**所有** hybrid 跑里
一致出现（init 0.216226 → 窗口 0.166/0.171/0.171）。

### 10.5 速度 / 初始化：**一次性成本，逐帧零增量**

| 量 | `off`（3 跑） | `hybrid`+z-fix（3 跑） | 上一轮 hybrid（§9.7，`nav_hyb_gicp_r*`） |
|---|---|---|---|
| ms/帧（窗口 20 s） | 15.43 [14.30–15.80] | 13.90 [12.80–15.15] | 12.90 [12.65–14.00] |
| %CPU | 5.82 [5.68–5.89] | 5.32 [5.21–5.75] | 5.55 [5.34–6.01] |
| CPU/仿真秒 | 0.153 [0.143–0.158] | 0.139 [0.128–0.151] | 0.129 [0.127–0.140] |
| VmHWM MB | 36.06 [35.77–37.00] | 36.21 [35.24–36.22] | 34.19 [33.82–35.11]（§9.4 mapping 档） |
| 初始化（仿真 s / 墙钟 s） | 2.10–2.20 / 11.67 [11.24–12.51] | 2.10–2.20 / 12.25 [11.80–12.84] | 2.10 / 11.13–11.15 |
| adapter %CPU | — | 0.209 [0.204–0.211] | 0.17 [0.15–0.17] |
| 进 EKF 点/帧 | 8563.5 [8563–8563.5] | 8563 [8562.5–8563] | 8563 |
| 每帧原始点 | 11896 | 11896 | 11896 |

* **没有"修复变慢"的证据**：hybrid 的 ms/帧中位数 **13.90 < off 15.43**（方向与 §9.5 第 3 条一致：
  sensor 约定省掉每帧一次 TF 查询/合成/广播）；与上一轮 hybrid（12.90）的差在**机器噪声**内
  （本轮宿主机更忙：parser/文档/多个探针并行，off 档自己也从 13.20 涨到 15.43）。
* **一次性**：z 定基是 3 行赋值；互检拟合 **2.6 ms**（§10.3）；初始化仿真时间 2.10–2.20 s
  与 off、与上一轮 hybrid **逐项相同**。
* 诚实边界：本轮 load 比 §9.7 高，**≤2–3% 的差别分辨不出**；能排除的是 ≥8% 的变慢（§9.6 同口径）。

### 10.6 默认路径（`gravity_align_mode:=off`）逐位相同

1. **同一输入流 bag 重放，两个二进制输出逐位相同**（口径与 §4.2 逐字相同）：
   * 旧 = `955084877be37af3a7fdd269670c127c`（`zfix_prev`，= `8e5e928` 之后的 HEAD 构建）；
   * 新 = `58bebfb81537978a0c93067be09de1be`（`zfix`，= `1b5db4b`；新增独立 TU，默认档不调用）；
   * 结果：**141 行 vs 141 行（ODOM 47 / CLOUD 47 / TF 47），排序后 diff 0 行**；
     未排序 diff 48 行 = **仅到达顺序**。原始数据 `.tmp_uc/replay_zfix/`。
2. **launch 增量**（`tools/scripts/gzsim/launch_increment_diff.py`，`gravity_align_mode=off`）：
   `OLD 66 / NEW 67`；新增 **1** 个 `DeclareLaunchArgument(gravity_align_z_origin)`；SPL 组
   显示为"改了一份"，逐字段核对后**只有** `gravity_align_mode != off` 那份 Node 多了
   `gravity_align_z_origin` / `gravity_align_h` 两个参数；**off 那份 Node 的参数表/remap/condition
   逐字段相同**，`LogInfo` / rviz Node 相同，**共有 Action 相对顺序一致**。
   ⇒ off 档：同一节点集合、同一参数、同一顺序。
3. **off A/B 复现 §9.7 基线**：scan p50 **0.243**（§9.7：0.2426）、in-band **579**（580）、
   local lethal **193**（193）、nearest **0.760**（0.760）、ahead **21**（21）、
   `odom→base_link` rpy **[30.06…30.20]**（30.04）、倾角 **30.115°**（30.10）。
4. **off 档没有收到新参数**：`ros2 param get` 回读是节点默认 `sensor` / `0.0`（§10.2）。

### 10.7 因果对照：同一 `hybrid` + `gravity_align_z_origin:=sensor`（1 跑）

| 指标 | `hybrid` + `z_origin:=sensor`（1 跑） | `hybrid` + `z_origin:=ground`（3 跑中位数） |
|---|---|---|
| 波束 z(odom) p50 | **−0.158** | **+0.058** |
| 在带 `[0,2]` 内 | **0** | **678** |
| local lethal / inscribed / free | **0 / 0 / 62500** | **288.5 / 8452 / 43312.5** |
| 最近 lethal | **无** | **0.769 m** |
| 前方 2.0×1.2 m lethal | **0** | **18** |
| `base_link` z(odom) | **−0.159** | **+0.057** |
| `odom→base_link` rpy | [0.279, −0.422, 0.002]° | [0.30, −0.36, −0.04]°（中位） |
| 地面倾角 | 0.548° | 0.518° |

⇒ **唯一变量 = `gravity_align_z_origin`**（同一 `hybrid`、同一二进制、同一 launch md5、同协议）：
`sensor` 复现 §9.7 的"整盘掉出带 + 空图"（`lethal 0 / free 62500`），`ground` 恢复
（`lethal 288.5 / 最近 0.769`）。**因果成立**，不是别的改动带来的。

### 10.8 作废的一跑（**不进任何表**）：parser 并发改 launch 的 30 rad bug

`zfix_hybrid_r1`（21:34–21:36，**旧批次**）跑到了另一个 agent 正在改的 launch：它把
`/lio_tf_adapter rpy` 传成 **`[30.0, 0, 0]`（度）**，而该参数进 `setRPY()` 是**弧度** ⇒ 等价 30 rad
⇒ `odom→base_link` roll **−110.7°**、scan z p50 **−0.552**、in-band **43**、local lethal **7**
（又接近空图）。**该跑作废，本文任何数字都不引用它**。冻结后的 6 跑 launch md5 全部
`0161033b…`（= `1b5db4b` 版本，无 parser 代码）；parser 的 launch-only 提交保留在
`parser-a4dbbf9` 分支，由父 agent 安排 rebase + 修单位后重跑它自己的交叉校验。

### 10.9 本轮没能验证 / 诚实边界

1. **"地面"的定义**：`ground` 档的 z=0 是 **URDF 几何的地面**（轮底平面），不是"雷达在当前场景
   里看到的那层地面"。RMUL2026 出生点两者差 **6.9 cm**（§10.3），互检已 ERROR；按任务书
   **定基仍用 URDF**。若产品要的是"可见地面 z=0"，需要另立一条规则（用 `h_meas` 定基），
   本轮**没有**做这个决定。
2. **窗口 `base_link z = +0.057` 不是 +0.1025**：差 = 既有 LIO z 漂移 −0.045（§10.4.3），
   不是修复引入；本轮没有改 LIO 的 z 漂移（那是独立问题）。
3. **三行数值目标（scan z +0.259 / in-band 580 / lethal 193）不能逐字相等**，口径原因见
   §10.4.2；其中 `+0.259` 与 `580` 是"scan 帧 = livox_frame / off 子集"的误读。
4. **`slam_nav` 与 `global_obstacle:=scan` 没有跑**（时间让给主干 A/B + 因果对照）。机理上同一条
   `min_obstacle_height` 常量带：`slam_nav` 的 local costmap 与 `mode:=nav` 逐字相同
   ⇒ **预期同病同解**；`global_obstacle:=scan` 会把**全局**图也切到 2D `/scan`，修复后
   scan z +0.057 仍在 `[0,2]` 内 ⇒ **预期也恢复**。两者**均未实测**。
5. **`hzmirm` / 默认模型的 `h` 只做了 launch 求值**（0.800000 / 0.235000），**没有跑 Gazebo**
   验证 TF/scan；`robot11` 是唯一实跑槽位。
6. **`gravity_align_h ≤ 0` 的退回 sensor 分支**只做了代码审查 + 计划中的 bag 重放验证
   （**已做**：同一份 bag 重放、`z_origin=ground, gravity_align_h=0.0` ⇒ 日志 `ERROR ... gravity_align_h=0.000000 m 无效 ⇒ 退回 sensor 档`，且**没有**打 `gravity_align_z` / `h_check` 两行）；阈值 0.03/0.05 的 WARN 带本轮**没有**构造出真实数据（sim 直接落 ERROR）。
7. **速度**：机器不安静（parser/文档并行 + 多探针），只能排除 ≥8% 的变慢；≤2–3% 分辨不出。
8. **互检拟合只用初始化前最后一帧**（0.20 s 窗口）；如果真机在"静止门"之后仍缓慢移动，
   `h_meas` 会带运动模糊 —— 它只作诊断，不影响定基。

### 10.10 一句话给用户

> **修好了，而且病因/口径都钉死了。** `gravity_align_z_origin:=ground` 让 odom 的 z=0 =
> **初始化时刻的地面**：初始 `base_link` z = **+0.1025**、`livox_frame` z = **+0.2595**，
> scan 盘面从 **−0.158 m** 抬到 **+0.057 m**（窗口；init +0.1025）⇒ **678/678 波束全部进
> nav2 的 `[0,2]` 带**，local costmap 从 **lethal 0 / 整张 free** 恢复成 **lethal ≈289 /
> 最近 0.769 m / ahead ≈18**（off 193 / 0.760 / 21）——**障碍信息恢复，且比 off 多 99 条进带波束**。
> 三行"≈"（scan z +0.259 / in-band 580 / lethal 193）**不能逐字相等**，原因是 +0.2595 是
> livox 高度而非 scan 平面、且修复后 in-band 是 off 的**超集**（678 vs 580）——不是回归。
> **一次性成本**：互检 2.6 ms，初始化/逐帧 ms 与 off 同量级；**默认路径逐位相同**（bag 重放
> 141/141 行）。**副作用（loud）**：RMUL2026 出生点的可见地面比 URDF 轮底平面高 **6.9 cm**
> （机器人陷进 mesh），互检如实报 **ERROR**；按任务书仍用 URDF 定基。

## §11 URDF 安装参数解析（launch 侧，2026-10-21）

> 契约 §13.5。对象是 **launch 的取值**（`gravity_align_rpy` / adapter `xyz`+`rpy` / `gravity_align_h`
> 的 livox 分量），**不是** LIO 算法，也不是运行期 `h_meas`（那在 §10 / LIO 侧）。
> 本轮**没有起 Gazebo**：测的是"解析出来的数与旧表/命令字符串"，用 Python 直接加载 launch 文件即可。

### 11.1 方法（离线、可复现）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 .tmp_mountparse/test_integrated.py      # 脚本未提交（temp 工具，不是产品代码）
```

* 用 `SourceFileLoader` 加载**仓库里的** `src/rm_nav_bringup/launch/bringup_sim.launch.py`
  （不是手抄的副本），构造 `launch.LaunchContext` 并只填 `launch_configurations`；
  调用真实的 `_parsed_livox_mount` / `_GravityAlignRpy` / `_GravityAlignH` /
  `_LioAdapterExtrinsic` / `_LioAdapterRobot11` / `_RobotXacroCommand`。
* `robot11` 的 xacro 展开走 `xacro.process_file`（与 launch 的 `xacro` CLI **同一个 Python 库**、
  同一份 `_robot_xacro_mappings`）。本机是 symlink-install ⇒ `install/…/urdf/*.xacro` 就是
  `src/…/urdf/*.xacro`（robot11 的 md5 `ed0af27b2c4c365779b4260017b04ae2`），不是构建期旧副本。
* 决定性测试把 xacro **复制到 `tempfile.TemporaryDirectory()`** 再改（仓库文件只读）；
  仓库 xacro 的 md5 在测试前后不变。

### 11.2 parse-vs-table（所有 `(robot, mount)` 组合）

`parsed` = 展开这次真 spawn 的 xacro → fixed 关节链合成的 `T(base_link←livox_frame)`；
`table` = 1b5db4b 的规则表（现在只作交叉校验）。角度单位 **°**，xyz/h 单位 **m**。

| `robot` | `robot11_mount` | `livox_tilt_axis` | parsed rpy | table rpy | parsed xyz | table xyz | `h`(parsed) | `h`(table) |
|---|---|---|---|---|---|---|---|---|
| `''`（默认） | — | — | `[0, 0, 0]` | `[0, 0, 0]` | `[0.120000, 0, 0.175000]` | 同左 | 0.235000000 | 0.235000000 |
| `hzmirm` | — | — | `[0, 0, 0]` | `[0, 0, 0]` | `[0, 0, 0.800000]` | 同左 | 0.800000000 | 0.800000000 |
| `robot11` | `plugin`（默认） | `roll` | `[0, 0, 0]` | `[0, 0, 0]` | `[+0.000561701, +0.130915824, +0.157028170]` | 同左 | 0.259527170 | 0.259527170 |
| `robot11` | `urdf` | `roll` | `[-30, 0, 0]` | `[-30, 0, 0]` | 同上 | 同上 | 0.259527170 | 0.259527170 |
| `robot11` | `sensor` | `roll` | `[-30, 0, 0]` | `[-30, 0, 0]` | 同上 | 同上 | 0.259527170 | 0.259527170 |
| `robot11` | `urdf` | `pitch` | `[0, -30, 0]` | `[0, -30, 0]` | 同上 | 同上 | 0.259527170 | 0.259527170 |

* **6/6 组 Δrpy = 0.000000°、Δxyz = 0.000000 m**（9 位舍入内；30° = `0.523598775598293 rad`）。
  `source` 全部 = `xacro.process_file`。**没有触发 WARN**（这正是"解析器没改行为"的基线）。
* 关节链（`_compose_fixed_chain` 打印）：
  * 默认模型：`livox_frame_joint(fixed)`；
  * hzmirm：`livox_frame_joint ← head_to_lidar ← turret_to_head ← base_to_turret`（全是 fixed）；
  * robot11：`body_to_livox(fixed)`。
* **adapter 的 `T(imu_link←base_link)`**（`xyz` + `rpy`；`rpy` 是**弧度**，因为
  `lio_tf_adapter` 里直接 `q_offset.setRPY(...)`）：

| `robot` / `mount` | parsed adapter xyz | table adapter xyz | parsed adapter rpy (rad) | table adapter rpy (rad) |
|---|---|---|---|---|
| `''` | `[-0.120000, 0, -0.125000]` | 同左 | `[0, 0, 0]` | `[0, 0, 0]` |
| `hzmirm` | `[0, 0, -0.750000]` | 同左 | `[0, 0, 0]` | `[0, 0, 0]` |
| `robot11` / `plugin` | `[-0.000562, -0.130916, -0.107028]` | 同左 | `[0, 0, 0]` | `[0, 0, 0]`（表里是 `-0.0`） |
| `robot11` / `urdf`+`sensor`（roll） | `[-0.000562, -0.034862, -0.151448]` | 同左 | `[+0.523598776, 0, 0]` | `[+0.523598776, -0.0, -0.0]` |
| `robot11` / `urdf`（pitch） | `[-0.079001, -0.130916, -0.085710]` | 同左 | `[0, +0.523598776, 0]` | `[0, +0.523598776, 0]` |

**单位坑（已修，实测抓到的）**：`gravity_align_rpy` 是**度**，adapter 的 `rpy` 是**弧度**。
第一版解析器把 adapter 也填成度 ⇒ 交叉校验立刻报 `Δ=29.476401°`（`table` 0.523599 rad vs
`parsed` 30 rad 级别的差）；改成 `_R_to_rpy_rad()` 后 6 组全部一致。这个 WARN 是故意留的：
**单位写错会当场响**，不会静默把车转 30 rad。

### 11.3 决定性测试：robot11 xacro 副本把倾角 **30°→20°**

为什么用副本 + 为什么去掉 `livox_tilt_rpy` 映射：robot11 的 30° 是 **launch 传给 xacro 的参数**
（`_LIVOX_TILT_RPY` → `livox_tilt_rpy:=…`），不是 xacro 的默认值。为了模拟"**URDF 自己拥有这个数**"
（= 任意新机器人/关节字面值被改的场景），把副本的 `body_to_livox` 关节写成
`rpy="-0.3490658503988659 0 0"`（−20°），并按"没有这个 launch 参数"的 mappings 解析；
仓库文件一个字节没改。

```text
scratch = /tmp/mountparse_20_…/robot11_20deg.xacro
parsed rpy = [-20.000000, +0.000000, +0.000000]°      （旧表 = [-30.000000, 0, 0]°）
WARN = YES
  [bringup_sim][WARN] URDF 安装参数与旧表不一致（**保留解析值** = 这次真的 spawn 的模型）：
    robot='robot11' mount='urdf' urdf=/tmp/…/robot11_20deg.xacro（source=xacro.process_file）
    rpy(parsed)=[-20.000000, 0.000000, 0.000000]° vs rpy(table)=[-30.000000, 0, 0]°  Δ=[10.000000, 0, 0]°
    xyz(parsed)=[0.000562, 0.130916, 0.157028] m vs xyz(table)=同值  Δ=0
    ⇒ 表只作交叉校验；若解析值才是实物，请同步 §13 与 linefit 的 sensor_height。
  [bringup_sim][WARN] adapter 的 T(imu<-base_link) 与旧表不一致（**保留解析值**）：
    rpy(parsed)=[0.349066, 0, 0] rad vs rpy(table)=[0.523599, -0.0, -0.0] rad  Δ=[-10.000000, 0, 0]°
```

⇒ **派生值真的跟着 URDF 改了**（−30°→−20°），adapter 也跟着变，**两条 WARN 都响**；
被保留的是**解析值**（`-20°`），不是表值。测试后仓库 xacro md5 仍是 `ed0af27b…`。

**对照测试（同一副本，但走完整 launch 参数）**：只把 `body_to_livox` 的 z 从
`${0.15702816968305 + livox_raise_m_p}` 改成 `${0.25702816968305 + livox_raise_m_p}`：
parsed `livox_z = 0.257028170`、`h = 0.359527170`（旧 `0.157028170 / 0.259527170`），
WARN 报 `Δxyz=[0, 0, +0.100000] m`；同时 linefit 交叉检查报
`yaml=0.259500 vs parsed h=0.359527（Δ=-0.1000 m）`。⇒ 即使 launch 参数原样，URDF 里关节几何
一改，rpy/xyz/h 也全部跟着变。

### 11.4 阈值 / 回退 / 未知模型 / linefit 交叉检查

* **阈值**：正常 6 组 Δ=0 ⇒ 无 WARN；30→20° 的 Δ=10° > 0.05° ⇒ WARN。xyz 用 1 mm。
* **已知模型解析失败**：把副本的 `body_to_livox` 改成 `type="revolute"` ⇒
  `_parsed_livox_mount()` 返回 `None`（**不抛给用户**），WARN 第一行点名关节：
  `body_to_livox` / `type=revolute` / 路径；wrapper 回退 `table`：
  `rpy=[-30.000000, 0, 0]`、`h=0.259527170`，banner 来源 = **`table-fallback`**。
* **未知模型**：`robot:=nope` ⇒ `RuntimeError`（"目前只有 ''|hzmirm|robot11… 不要手填"），
  不回退。
* **linefit `sensor_height`**（launch 从不设置它，按槽位选整份 YAML；解析器只读来交叉检查）：
  * `robot11` h=0.259527 vs YAML 0.2595（Δ 0.027 mm）⇒ 无 WARN；
  * `hzmirm` h=0.800000 vs YAML 0.80 ⇒ 无 WARN；
  * `hzmirm` h=0.700000（人为） ⇒ WARN（Δ+0.1 m，给出 YAML 路径）；
  * 30→20° + z→0.257 副本 ⇒ WARN（Δ−0.1 m），**但没有改 YAML 一个字节**。

### 11.5 默认路径 identity（`gravity_align_mode:=off`）

**(a) Action 树**：`launch_increment_diff.py`，old=`1b5db4b` 的 launch，new=本 commit：

| 参数组 | OLD/NEW Action | 新增 | 消失/被改 | 顺序 |
|---|---|---|---|---|
| （默认，不传参数） | 67 / 67 | **0** | **0** | 一致 ✅ |
| `mode=mapping lio=small_point_lio robot=robot11 robot11_mount=plugin gravity_align_mode=off world=RMUL2026 localization=` | 67 / 67 | **0** | **0** | 一致 ✅ |
| `mode=mapping lio=small_point_lio robot=robot11 robot11_mount=sensor gravity_align_mode=off` | 67 / 67 | **0** | **0** | 一致 ✅ |
| `mode=mapping lio=fastlio robot=hzmirm gravity_align_mode=off` | 67 / 67 | **0** | **0** | 一致 ✅ |
| `mode=mapping lio=small_point_lio robot=robot11 robot11_mount=sensor gravity_align_mode=hybrid` | 67 / 67 | **0** | **0** | 一致 ✅ |

⇒ 连 hybrid 档都只是 substitution 的**求值结果**变化，Action 树逐字段没动（无新节点/新参数/新顺序）。

**(b) `robot_description` 命令字符串**：用 1b5db4b 的 `_RobotXacroCommand` vs 新实现，6 组组合
（默认、hzmirm、robot11 plugin/urdf/sensor/urdf-pitch）**逐字节相同**（含 robot11 的引号规则）。

**(c) 解析器求值次数**（instrumented）：

* `gravity_align_mode:=off` + 默认模型：**0 次**（`_check_gravity_align_contract` 入口即返回；
  off 档的 Node 也不含 `_GravityAlignRpy`/`_GravityAlignH`）。
* `gravity_align_mode:=hybrid` + robot11：该 substitution 一次 launch 被求值 **7 次**
  （banner 3：rpy / h / source + 节点参数 4：`gravity_align_rpy`、`gravity_align_h`、
  adapter xyz/rpy），xacro 只展开 **1 次**（单次 9.5 ms，n=3 中位数）。
* `gravity_align_mode:=off` + robot11 + 非 spl LIO：legacy adapter 2 个参数 ⇒ 求值 2 次、展开 1 次。

⇒ 缓存保留（7 > 2 次；不缓存会重复展开 ~66 ms 并重复打 WARN）。

**(d) 值对比**（old vs new 的 YAML 字符串）：`gravity_align_rpy` / `gravity_align_h` 6 组全部
相同；adapter 5 组相同，**唯一差异**是 robot11 的 adapter rpy 里 `-0.0` → `+0.0`
（`float('-0.0') == float('0.0')`，TF/四元数数值完全相同；YAML 字符串的符号零差异）。
`hzmirm` 且 `turret_yaw_deg/turret_pitch_deg ≠ 0` 时，spl adapter 现在**跟着云台转**
（旧 `_LioAdapterExtrinsic` 固定 `[0,0,-0.75]/[0,0,0]`）—— 这是 opt-in 档的已知缺口被解析器顺手修掉；
legacy `lio_tf_adapter_hzmirm_node` 仍用字面值，但解析器对它做交叉校验（漂移 ⇒ WARN）。

### 11.6 地面偏移表（h 的**先验**；URDF 给不了的那一个量）

`h = _HZMI_BASE_LINK_HEIGHT_M[robot] + parsed livox_z`（`livox_raise_m` 已含在解析出的 z 里）。

| `robot` | base_link 原点离地 | 证据 |
|---|---|---|
| `robot11` | **0.102499** | `urdf/sentry_robot_robot11_sim.xacro` 注释「地面平面（body 系）z=-0.102499」；轮 r=0.058、轮心 z=-0.044502 ⇒ 0.044502+0.058=0.102502，差 3 µm = mesh 离散；`robot11_weld_chassis.py` 自检也报最低点 -0.10250 |
| `hzmirm` | **0.0** | `base_to_wheel_*` origin z=0.08、轮 cylinder r=0.08 ⇒ 轮底 = base_link z=0 |
| `''`（默认） | **0.06** | `radius_wheel=0.06`、`base_to_wheel*` origin z=0 ⇒ 轮底 z=-0.06 |

与 Gazebo 实测的差 = 轮胎下沉（默认模型 linefit 0.226 vs 几何 0.235 ⇒ ~9 mm；robot11 0.2595
vs 几何 0.259527 ⇒ 0.03 mm）。这部分**不在 launch 里**，由节点初始化时的 `h_meas` 互检负责
（LIO 侧，§4.7 / §10）；本 launch 只提供先验。

### 11.7 本轮没能验证 / 诚实边界

1. **没有跑 Gazebo / 完整 launch**：只做了 launch 求值与 `ros2 launch … --show-args`（100 个参数、
   退出码 0）。没有用 `ros2 param get` 回读节点实际生效的 `gravity_align_rpy`/`gravity_align_h`。
2. **plain-URDF 回退分支没有实跑**（本仓三个槽位都是 xacro）；`xacro.process_file` 是实际走到的
   唯一展开层。CLI 子进程层在复杂度审查后**已删除**（CLI 依赖的就是同一个 Python 模块），
   所以"CLI 不可用"的环境没有覆盖。
3. **运行期 `h_meas` 互检不在本节**：那是 LIO 侧（`gravity_align_ground.cpp`）的 3/5 cm 带，
   由并行任务实测；本节只证明 launch 送进去的先验值正确。
4. **hzmirm 云台角 ≠ 0** 时 spl adapter 的值变了（旧缺口修复），但**没有跑节点验证 TF**；
   legacy hzmirm adapter 仍用字面值（只 WARN）。
5. linefit 交叉检查的 **1 cm** 是启发式阈值（默认模型 9 mm 下沉不报、robot11 0.03 mm 不报）；
   它只 WARN，不会改 YAML，也不代表 linefit 的实际地面分割效果。
6. `-0.0` vs `+0.0` 的 adapter rpy 字符串差异（数值相同）没有做二进制逐位重放 —— 因为
   TF 输出只取决于 double 数值，`-0.0` 与 `+0.0` 在 `setRPY`/四元数里等价。

### 11.8 一句话

> 安装参数现在**真的从这次 spawn 的 URDF/xacro 解析**：6 组 (robot, mount) 与旧表逐值一致；
> 把 robot11 副本的倾角从 30° 改成 20° ⇒ 派生值变 **−20°** 且 WARN 响（仓库文件未动）；
> 默认路径 Action 树 **0 增 0 删**、`robot_description` 字符串 6 组逐字节相同、off 档解析 **0 次**。


---

## §12 第四轮（本机 date=2026-10-19）：`slam_nav` 与 `nav + global_obstacle:=scan` 的「斜 odom / 高度带」验收

> **本轮只新增本节**（append-only）：§1–§11 一个字都没改。对象是 §9.10 第 4 条 / §10.9 第 4 条登记的
> 两个「未验证组合」：**①`mode:=slam_nav`**、**②`mode:=nav` + `global_obstacle:=scan`**（把**全局**
> 代价图也切到 2D `/scan`）。每组跑 `gravity_align_mode:=off` 与 `hybrid`（后者用默认
> `gravity_align_z_origin:=ground`），共 **4 跑**；另加 **1 跑** `slam_nav`+`hybrid` 复本（n=2，
> 且用升级后的探针补测 `map→odom` 的「唯一戳率」）。**只做实验：src/** / tools/** / 任何 launch 与
> 参数文件**一个字节都没改**；新增的只是 `.tmp_ab/exp12/`（gitignored）里的**只读**探针与包装脚本。
>
> **日期口径**：本轮每跑 `meta.txt` 的 `date` 都是 **2026-10-19T22:1x–22:3x**（本机时钟）；§10 标题里的
> 「2026-10-21」是上一轮的任务标签，与文件里记录的 date 不一致 —— 本节一律以 `meta.txt` 的 date 为准。
>
> **冻结（launch_md5）**：5 跑的 `src/rm_nav_bringup/launch/bringup_sim.launch.py` md5 全部
> **`6d38e74ea7a53924e601e89b23331a77`**，且 `跑前 == 跑后 == 该跑时刻 HEAD 的 launch`（每跑
> `exp12_meta.txt` 的 `launch_md5_before/after/head` 三行逐字相同）。实验期间 HEAD 从 `f9bf4a9`
> 前进到 `49a4328`（**只动 tools/ 两个打包脚本**），launch 未变 ⇒ 冻结成立。二进制 =
> `libsmall_point_lio.so` md5 **`58bebfb8…`**（= §10.4 的 zfix 构建；`small_point_lio_node` 的
> `d940e573…`），跑前/跑后逐值相同（`.tmp_uc/bins/zfix` ↔ `build/`；bench 收尾会把 `build/` 换回旧的
> `after`，包装脚本在每跑后**还原成 zfix 并校验 md5**）。

### 12.1 协议（与 §10.4 逐字相同 + 一个只读补充探针）

四跑 × 两模式（`settle=20 s / window=20 s` 仿真；探针超时 560 s；`gui:=False`、`spin_speed:=0.0`、
`nav_rviz:=False lio_rviz:=False`）——**逐字命令**：

```bash
# A. slam_nav（lio:=small_point_lio、mapper:=slam_toolbox；world / spawn / 机器人同 §10.4）
.tmp_ab/exp12/run_one.sh exp12_snav_off_r1 world:=RMUL2026 mode:=slam_nav mapper:=slam_toolbox robot:=robot11 \
    robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off
.tmp_ab/exp12/run_one.sh exp12_snav_hyb_r1 world:=RMUL2026 mode:=slam_nav mapper:=slam_toolbox robot:=robot11 \
    robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid
# B. nav + gicp + 全局也走 2D scan（磁盘图 = map/RMUL2026.yaml + PCD/RMUL2026.pcd）
.tmp_ab/exp12/run_one.sh exp12_gscan_off_r1 world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
    global_obstacle:=scan robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off
.tmp_ab/exp12/run_one.sh exp12_gscan_hyb_r1 world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
    global_obstacle:=scan robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid
# C. 补充（n=2；探针升级后补测 map→odom 唯一戳率）
.tmp_ab/exp12/run_one.sh exp12_snav_hyb_r2 world:=RMUL2026 mode:=slam_nav mapper:=slam_toolbox robot:=robot11 \
    robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid
```

* **隔离**：`run_one.sh` 与 `spl_ab_bench.sh` 同口径（`HOME=.tmp_ab/home-<tag>`、按 tag 散列
  `ROS_DOMAIN_ID`、**先探空端口再占** `GAZEBO_MASTER_URI`、`setsid` 起栈、收尾只用
  `tools/scripts/lib/scoped_cleanup.sh` 清**本进程组/本 master URI**，**无全机 pkill**；包装脚本在
  bench 返回后只 `kill` 自己的探针 PID）。
* **探针**（都是只订阅、不发布、不改参数）：
  * `tools/scripts/tiltmount/nav_band_probe.py`（§9.7 起就有）：`/scan` 在 **odom** 的 z + 两张
    costmap 的原始代价计数（254/253/0/255）、车那格、最近 lethal、前方 2.0×1.2 m lethal；
  * `tools/scripts/tiltmount/spl_ab_bench.sh` 自带的 `grav_probe.py`（`.tmp_gravtest/`）：残余地面
    倾角、`odom→base_link` rpy、RTF；
  * **本轮新增（只在 `.tmp_ab/exp12/probe_exp12.py`，gitignored）**：同一帧 `/scan` 的波束端点
    （scan 帧里 z=0）用 TF 搬到 **odom 与 map 两个代价图帧**，给 z 统计、`[0,2]` 带内/带下/带上、
    **被保留波束的 y 区间**（= "banded slab / 半平面盲区"的直接证据）；`/map` 的条数/速率/发布者/info；
    `/tf` 里 `(parent=map, child=odom)` 的条数、**唯一戳数**、同戳最大重复次数；运行期
    `ros2 param get` 回读两张 costmap 的 `global_frame` / `obstacle_layer.enabled` /
    `stvl_layer.enabled` / `min|max_obstacle_height`、以及 `/small_point_lio` 的
    `gravity_align_mode|z_origin|h`、`/lio_tf_adapter rpy`。
* **口径**：`/scan` 的 `frame_id=base_link`；"in-band"= 波束端点 z ∈ `[0.0, 2.0]`（= nav2
  `obstacle_layer.scan` 的默认 `min_obstacle_height` + 文件里的 `max_obstacle_height: 2.0`，本轮
  运行期回读：local/global 都是 **min 0.0 / max 2.0**）；"kept y"= 带内波束端点的 odom/map y 范围。
* **⚠️ 环境噪声（记录在案，不影响判决）**：`spawn_entity.py` 在 **2/5 跑**（`exp12_snav_off_r1`、
  `exp12_snav_hyb_r1`）报 `Spawn status: Entity pushed to spawn queue, but spawn service timed out…`
  然后退出。**实体其实成功生成**：5 跑的 `/odom_ground_truth` 同值（`x≈4.2999926/4.2999931`、
  `y≈3.3497892/3.3497908`、`z≈0.1521570/0.1521612`，互差 ≤5×10⁻⁶ m；与 §10.4 同值），
  `/scan` 4.1 Hz、`/map` 有发布者。
  ⇒ 判定为 **spurious**（§9.7/§10 那两轮没有出现，未归因）。另：`/scan` 的有限波束数逐跑
  在 **677–678** 与 **775–776** 两档之间跳（同世界/同出生点/同静止；原因未查）⇒ 所有 in-band 都
  用**逐跑自己的分母**。

### 12.2 四跑 + 一跑复本：主表（`settle 20 / window 20`，中位数；每跑 n=1，除注明）

| run（tag） | 组合 / 重力档 | launch_md5 | 有限波束 | z(odom) p50 · [min,max] | **带内 / 带下 / 带上** | **kept y（odom）** | local **lethal / inscribed / free / unknown** | 最近 lethal · 前方 2×1.2 m | global **lethal / inscribed / free / unknown**（格数） | global 最近 · 前方 | 残余倾角 · RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `exp12_snav_off_r1` | **slam_nav** / off | `6d38e74e…` | 678 | **+0.243** [−0.244, +2.593] | **580 / 60 / 36**（85.5%） | **[0.112, 3.568]** | **193 / 8282 / 44544 / 0** | **0.761 m** · **21** | **209 / 2133 / 1530 / 12740**（17250） | 0.786 · 15 | 30.08° · 0.372 |
| `exp12_snav_hyb_r1` | **slam_nav** / hybrid | `6d38e74e…` | 677 | **+0.058** [+0.046, +0.098] | **677 / 0 / 0**（100%） | **[−0.458, 5.258]** | **287 / 8426 / 43343.5 / 0** | **0.768 m** · **18** | **239 / 2109 / 1485 / 12639**（17100） | 0.746 · 18 | 0.49° · 0.364 |
| `exp12_snav_hyb_r2`（补充，n=2） | **slam_nav** / hybrid | `6d38e74e…` | 776 | +0.071 [+0.062, +0.099] | **776 / 0 / 0** | [−0.677, 5.060] | **272.5 / 9349 / 42714.5 / 0** | **0.723 m** · **21** | 227 / 2012 / 687 / 10590（13950） | 0.724 · 30 | 0.31° · 0.287 |
| `exp12_gscan_off_r1` | **nav + gicp + global:=scan** / off | `6d38e74e…` | 677 | **+0.239** [−0.242, +2.602] | **578 / 62 / 36**（85.4%） | **[0.165, 3.597]** | **209 / 8254.5 / 44611 / 0** | **0.760 m** · **20** | 3782 / 17133 / 9859 / 0（43077）**⚠️ 见 §12.4** | 0.470 · 91 | 30.27° · 0.383 |
| `exp12_gscan_hyb_r1` | **nav + gicp + global:=scan** / hybrid | `6d38e74e…` | 775 | **+0.072** [+0.060, +0.101] | **775 / 0 / 0**（100%） | **[−0.644, 5.055]** | **256.5 / 9372.5 / 42701.5 / 0** | **0.723 m** · **20** | 4010 / 17214 / 9627 / 0（43077）**⚠️ 见 §12.4** | 0.454 · 95 | 0.26° · 0.294 |

读法与 §10.4 的表一致：`unknown=0` 是因为 `track_unknown_space=false`（"没看见"与 free 同值）；
"最近 lethal"= 车（`base_link_fake`）到最近 254 格的平面距离；"前方"= 车体系 x∈[0,2.0] m、|y|≤0.6 m
窗口内的 lethal 格数。`global` 的格数栏是**该跑自己的** global costmap 大小（slam_nav 是实时新建的
图，逐跑 17250/17100/13950 格；nav 是磁盘图 43077 格），跨跑不可直接比。

**第二张表（诊断用：map 帧几何 + `map→odom` + `/map`）** —— `map` 帧 z 只有在"map→odom 活着"或
"能回退到最新一帧"时才有值（见 §12.4 的坑）：

| run | map 帧 z p50 · [min,max] | map 帧 带内/下/上 | map 帧 kept y | `map→odom` 发布者（节点级） | `/tf` 里 map→odom：条数 / 唯一戳 | `/map` |
|---|---|---|---|---|---|---|
| `snav_off_r1` | **+0.0062** [−0.0105, +0.0126] | 638 / 40 / 0 | [−0.259, 5.353] | **`/slam_toolbox`（唯一）** | 2225 条 / 探针记录跨度 17.8 仿真秒 ≈ **46 条/墙钟秒**（唯一戳未测） | 有：`/slam_toolbox`，0.50 Hz，150×115 @0.05，17250 格 |
| `snav_hyb_r1` | **−0.0004** [−0.0026, +0.0030] | 243.5 / 437 / 0 | [−0.329, 5.258] | **`/slam_toolbox`（唯一）** | 2258 条 ≈ 46 条/墙钟秒（唯一戳未测） | 有：同上，0.51 Hz，150×114，17100 格 |
| `snav_hyb_r2` | **−0.00055** [−0.010, +0.0047] | 262.5 / 512 / 0 | [−0.526, 5.057] | **`/slam_toolbox`（唯一）** | **2917 条 / 20 仿真秒**（= 145.9 条/仿真秒 ≈ **42 条/墙钟秒**）；**唯一戳 179 个**（8.95/仿真秒 ≈ **2.6/墙钟秒**），同一条戳最多重复 **21** 次 | 有：`/slam_toolbox`，0.65 Hz，150×93，13950 格 |
| `gscan_off_r1` | **+0.432** [−0.401, **+4.501**] | 536 / 58 / 83 | **[0.127, 1.317]** | `/gicp_registration`（**只活了 ~6 s**，见 §12.4） | 300 条 / **25 唯一戳**（同戳最多重复 13 次；戳跨度仅 2.4 仿真秒） | 有：`/map_server`（latched 1 条），窗口内 0 条 |
| `gscan_hyb_r1` | **+0.134** [+0.100, +0.238] | 776 / 0 / 0 | [−0.662, 5.022] | `/gicp_registration`（**只活了 ~6 s**） | 371 条 / **23 唯一戳**（同戳最多重复 20 次） | 有：`/map_server`（latched 1 条），窗口内 0 条 |

### 12.3 判决 A：**`mode:=slam_nav` 与 `mode:=nav` 是同一个病、同一个解**

1. **同病（off）**：off 的局部代价图与 §10.4 的 `mode:=nav` off **逐项对上** —— 有限波束 678、
   带内 **580**（85.5%）、带下 60、带上 36、`z(odom) p50=+0.243`、**kept y=[0.112, 3.568]**
   （= 只保留一个 y 方向的**条带**，机器人正前方/负 y 半平面的波束被整片丢掉）、local
   **lethal 193 / inscribed 8282 / free 44544 / 最近 lethal 0.761 m / ahead 21**。
   §10.4 nav off 的同名量：579 / 60 / 36 / +0.243 / 193 / 0.760 / 21。**同一个病，且数值几乎逐值相同。**
2. **治愈（hybrid）**：**677/677 全部在带内**（0 带下 / 0 带上），`z(odom) p50=+0.058`，
   kept y 变成**整盘**（[−0.458, 5.258]），local **lethal 287 / 最近 0.768 m / ahead 18**；
   n=2 复本给出 **272.5 / 0.723 / 21**、**776/776**。§10.4 nav hybrid：288.5 / 0.769 / 18 /
   678-in-band。**同一量级、同向、同解。**（残余倾角 0.49° / 0.31°，§10.4 的 0.518° 同带。）
3. **local 是否"像 `mode:=nav`"**：是 —— 不是"像"，而是**逐项重合**（上两条）；`mode:=slam_nav`
   的 local costmap 与 `mode:=nav` 是同一份参数（`global_frame: odom`、`obstacle_layer.scan`
   min/max **0.0/2.0**，运行期回读），差别只在"谁发 map→odom / 谁发 /map"。
4. **global（本组默认 `global_obstacle:=stvl`）**：off 209/2133/1530/12740（17250 格）、hybrid
   239/2109/1485/12639（17100 格）、n=2 227/2012/687/10590（13950 格）——**都没空**，且都在
   "实时新建的 slam 图"上（`/map` 由 `/slam_toolbox` 以 0.5–0.65 Hz 发、150×93…150×115 逐跑不同）。
   这三跑的格数/占用率本来就逐跑不同（实时建图），**不要跨跑比绝对值**；能说的是"STVL 这一路
   不吃 `obstacle_layer.scan` 的 `[min,max]` 常量带、也没有因为 off/hybrid 变空"。
5. **`map→odom`（节点级发布者 = 1）**：`/tf` 的 publisher endpoint 里只有 **`/slam_toolbox`** 能
   发 map→odom（slam_nav 不起任何重定位节点、不起 map_server；`ros2 node list` 里没有
   amcl/gicp/icp）。`/tf` 图上 `/slam_toolbox` 占 **2 个 publisher endpoint**（ROS 2 按 endpoint
   计数，不是两个发布者）；**`tf2_monitor map odom` 报 `<no authority available>`**（Humble 的
   `/tf` 消息不带 authority，broadcaster 级计数**不可得**）。**速率**：`transform_publish_period:
   0.02` ⇒ **≈42–46 条/墙钟秒**；但 `restamp_tf` 默认 false ⇒ 同一条 scan 戳会被重复广播
   （n=2 实测同戳最多重复 **21** 次，20 仿真秒里只有 **179 个唯一戳** ⇒ **位姿更新率 ≈ 2.6/墙钟秒
   = scan 处理率**）。**"条数"不是"位姿更新率"**，两个都给。
6. **nav2 告警**：高度带这条链 **0 条**（见 §12.5）；slam_nav 两跑只有启动期
   `Robot is out of bounds of the costmap` 12–13 条 + `no map received` 1 条（实时图还没覆盖到车）
   + `Message Filter dropping` 1 条（slam_toolbox 自己的 TF 缓存冷启动），**都随图/缓存就绪消失**。

### 12.4 判决 B：**局部同病同解；但 `global_obstacle:=scan` 这一臂被 gicp 打断 —— "全局有没有变空"能答，"global 的带子是否同病/是否被 hybrid 治好"不能答**

1. **局部（有效）**：off → 578/677 在带内（85.4%）、kept y=[0.165, 3.597]、local lethal **209**、
   最近 **0.760 m**、ahead **20**；hybrid → **775/775 全在带内**、kept y 整盘、lethal **256.5**、
   最近 **0.723 m**、ahead **20**。⇒ **与 A、与 §10.4 同病同解**。数量级说明：本跑 hybrid 的
   lethal（256.5）低于 §10.4 hybrid 的 288.5、但高于本组 off 的 209；这一跑 `/scan` 有限波束是
   775（off 是 677），绝对格数会随"进带波束集合"逐跑浮动（§12.1 末尾），判决看的是
   "in-band 85% → 100%、最近 lethal 与 ahead 不为 0"。
2. **全局"没有变空"（能答）**：off **3782 lethal / 17133 inscribed / 9859 free / 0 unknown**
   （43077 格），最近 lethal **0.470 m**、ahead 91；hybrid **4010 / 17214 / 9627 / 0**，最近 **0.454**、
   ahead 95。两张图都**有静态墙 + inflation**，不是空图。
3. **⚠️ 但这 3782/4010 不是 2D scan 层打出来的，是 `static_layer` + inflation 撑着的**：
   `localization:=gicp` 在**两种重力档**下都**从未采纳第一帧**——日志逐字：
   * `用参数 initial_pose 初始化：map 系机器人位姿 x=0.000 y=0.000 z=0.000 yaw=0.00° ⇒ map→odom = x=0.005 y=0.001 z=-0.079 yaw=0.02°`（off）/ `… z=0.063 …`（hybrid）；
   * 每帧 `GICP 本帧未被采纳（converged=true, score≈0.0052 m² … | 创新 0.176–0.227 m / ≤0.9° vs 门限 0.1000 m / 3.000°（**dt=0.000 s**）★创新超门限 … 连续 N 帧）`；
   * ~6 s 后：`★★ 定位已判失效（连续 25 帧被拒）：**本节点不再发布 map→odom / ~/pose**`；
   * 此后 global costmap 的观测滤波器**整跑都在丢消息**（off **38** 条 / hybrid **39** 条，最后一条
     一直持续到跑尾）：`global_costmap.global_costmap: Message Filter dropping message: frame
     'base_link'（或 'livox_frame'）at time … for reason 'the timestamp on the message is earlier than
     all the data in the transform cache'`。
   ⇒ 窗口内 `global_obstacle:=scan` **没有可用的 `map←odom`**，一条 scan 也没标进 map 帧。
   **这不是本轮引入的**：§10.4 的 **6 份日志**（`zf2_off_r1–r3`、`zf2_hybrid_r1–r3`）逐份都是
   `采纳 0/N` + **147–161 条**"判失效"（§9.10 第 7 条把它们记成"off 118 / hybrid 2 条外推告警"，
   与本轮看到的日志不符；本轮 off 128 / hybrid 129 条外推、且两档都死）。**疾病与本轮变量无关。**
4. **能作为"推断"（不是实测）的一条**：gicp 唯一发出去过的 map→odom 是**初始值**（从未被采纳帧
   修正）。off 档该值在数值上 = `T_odom_base`（`z=-0.079`、roll 30° 未打印）⇒ `map←base` 把 odom
   的 30° 倾角**加倍**成 ~60°：map 帧 z 跨度 **4.90 m** vs odom **2.84 m**（比值 **1.72 ≈ tan60/tan30**），
   in-band 只剩 **1.19 m** 宽的 y 条带（odom 里是 3.43 m）；hybrid 档初始 map→odom 是水平
   （`z=0.063`）⇒ map 帧 z=**+0.100…+0.238**、tilt 0.12°、**776/776 全在带内**。
   **若** map→odom 是活的且等于这个初始值，几何上 off 会把 global 的 2D scan 层切成**更窄**的条带、
   hybrid 全部保留（同病同解）——**但"活定位器"这个前提不成立**，所以这条**只登记为推断**。
   （可能来源：`gicp_registration.cpp` 的初始化式 `T_map_odom_cur = T_map_base_init * T_odom_base`；
   本轮只读观察，**未改 src/**，也**没有**单独验证这个式子是不是 bug。）
5. **结论**：对 combo B 的**局部**半 = "同病、hybrid 治愈"（实测）；对**全局**半 = **不能判**
   （既不能说"同病"，也不能说"治好了"），因为该臂的定位器在窗口开始前就死了。要判全局，
   必须先修 gicp 的首帧门（`dt=0.000 s` ⇒ 门限永远取下限 0.1 m，而首帧创新 0.16–0.23 m ⇒ 死锁），
   或换 `localization:=amcl` / `slam_nav + global_obstacle:=scan` 重做（后者 `map→odom` 是活的，
   但 slam_toolbox 把 map 的 z 锚在车起始高度 ⇒ scan 平面**压在带子下边界**，是另一种"刀口"，
   见 §12.6 第 2 条）。

### 12.5 nav2 的"静默"：高度带这条链确实一条告警都没有 —— 但静默 ≠ 健康

* 5 跑对 `dropped|out of band|min_obstacle|max_obstacle|isCurrent` 的匹配数 **全部 = 0**
  （`nav_band_probe`/`log_grep` 同口径）。nav2 的 `obstacle_layer` **不会**因为波束掉出
  `[min_obstacle_height, max_obstacle_height]` 报任何东西 —— **这是 §9.7 那条"无任何提示"的复现**。
* 但"静默"只覆盖高度带：**gicp 死掉时 nav2 说的是另一套**（planner_server 的
  `Timed out waiting for transform…` / `Extrapolation Error`：off 128 / hybrid 129 条；
  global_costmap 的 `Message Filter dropping`：38/39 条），**没有一条提到"高度带"或"看不见障碍"**。
  slam_nav 的 off/hybrid 则是 0–1 条外推 + 启动期 12–13 条 out-of-bounds + 1 条 no-map。
* 所以本轮的"nav2 静默"结论要拆开说：**对高度带，静默成立（5/5）**；**对"局部/全局图是否真的
  收到实时障碍"，不成立**（gicp 那一臂有 128–129 条 TF 告警，只是都跟高度带无关）。

### 12.6 本轮没能验证 / 诚实边界

1. **combo B 的 global 半没测到**（§12.4）：gicp 首帧门死锁 ⇒ 窗口内没有 `map←odom` ⇒
   `global_obstacle:=scan` 从未真正工作。要判它，得先修 gicp（禁改 src/**）或换定位器重做。
2. **`slam_nav + global_obstacle:=scan` 没跑**：map 帧由 slam_toolbox 定基（map←base 水平、
   z≈0、±1 cm），scan 平面**正好压在下边界**：`snav_off_r1` 的 map 帧带内 **638/677**、
   `snav_hyb_r1` 只有 **243.5/677**（两跑只差 ~6 mm 的 z 偏移）⇒ 若把 global 切到 scan，会得到
   "刀口"而不是"斜面"的另一种病。**未实测**。
3. **`map→odom` 的 broadcaster 级计数不可得**：ROS 2 Humble 的 `/tf` 消息不带 authority，
   `tf2_monitor map odom` 报 `<no authority available>`；本轮给的是**节点级**
   （`/slam_toolbox` 唯一，2 个 endpoint）+ 唯一戳率（n=2 实测 179 戳/20 仿真秒）。
4. **单跑/双跑**：A 组 off n=1、hybrid n=2；B 组 off n=1、hybrid n=1。没有 3 跑中位数；
   噪声带按 §10.4 的 [min–max]（lethal ±10、ahead ±3、nearest ±2 mm）参考。`/scan` 有限波束数
   在 677–678 / 775–776 两档跳（原因未查，§12.1）⇒ in-band 的**绝对条数**有 ~14% 的逐跑差，
   **比例**（85% vs 100%）是稳的。
5. **只测 `robot:=robot11 robot11_mount:=sensor`**；`plugin`/`urdf`/默认模型、`hzmirm` 未测。
   combo B 只测 `localization:=gicp`（amcl/small_gicp 未测）。
6. **spawn_entity 的 spurious "timed out"**（2/5 跑）只做了"实体确实在"的旁证
   （`/odom_ground_truth` 5 跑同值 ≤5×10⁻⁶ m + `/scan` 4.1 Hz + `/map` 有发布者），**未归因**。
7. **map 帧 z 统计的样本**：`gscan_*` 两跑有 `map→odom` 死后的"回退到最新一帧"，所以 map 帧数字
   来自**初始（从未修正）的 map→odom**；`snav_*` 两跑 map→odom 活着，数字可信。
   样本数：`snav` 3 跑都是 199/199 帧；`gscan` 每跑 199 帧里 **176 帧**拿到了 map TF
   （`tf_fail` 96/98，含 odom 侧；失败的帧在 map 帧统计里被跳过）。
8. **日期口径**：本机 `date` 与 §10 的标题日期不同（见本节抬头）；每跑以 `meta.txt` 的 `date` 为准。

### 12.7 一句话给用户

> **A（`slam_nav`）：同病同解。** off 与 `mode:=nav` off **逐项重合**（带内 580/678 = 85.5%、
> kept y=[0.112, 3.568] 的条带、local lethal 193、最近 0.761 m、ahead 21），hybrid 后
> **677/677 全进带**、lethal 287、最近 0.768 m、ahead 18（n=2：272.5/0.723/21）——
> slam_nav 的 local costmap 就是 nav 的 local costmap，30° 病与 z 原点修复一字不差。
> `map→odom` 唯一发布者 = `/slam_toolbox`（节点级 1 个；/tf 不带 authority），
> 广播 ~42–46 条/墙钟秒、**唯一戳只有 ~2.6/墙钟秒**（同一条 scan 戳重复最多 21 次）。
> **B（`nav + gicp + global_obstacle:=scan`）：局部同病同解；全局这一半没测到。**
> 局部 off 578/678 → hybrid 775/775（lethal 209→256.5、最近 0.760→0.723）是干净的；
> 但 **gicp 在两种重力档下都 0/25 采纳、~6 s 判失效、停发 map→odom**（创新 0.16–0.23 m >
> 门限 0.100 m，dt=0.000 s 死锁；§10.4 的 6 份日志同样如此）⇒ 全局图**没有变空**
> （off 3782/17133/9859、hybrid 4010/17214/9627），但那 3782/4010 是 **static_layer + inflation**，
> 不是 2D scan 层；`global_obstacle:=scan` 是否同病/是否被 hybrid 治好 = **本轮无法判定**
> （唯一能做的是从"初始 map→odom"几何推断：off 会加倍成 ~60° ⇒ 只剩 1.19 m 条带，hybrid 水平 ⇒
> 776/776 —— **推断，未实测**）。**nav2 对高度带全程静默（5/5 跑 0 条）**；它对 gicp 的死亡
> 倒是喊了（128–129 条 TF 外推 + 38–39 条 Message Filter 丢弃），但没有一句和高度带有关。

## §13 第五轮（**本机 date=2026-10-19**）：**RMUL2026 地面校准** + **Livox 插件 `cloud_frame=sensor` 坐标系 bug**

> **本节只新增**（append-only）：§1–§12 一个字都没改。它**修正 §10.3 的结论**：那里写的
> 「机器人陷进地面 6.9 cm / 可见地面 world z = 0.1183 / odom 里可见地面 ≈ +0.069」是**雷达侧一个 bug**
> 的像，不是世界资产的问题；RMUL2026 的可行驶地面当时在 **+0.049662 m**，轮底平面**正好落在它上面**
> （残差 2 µm）。契约规范见 `docs/frame_contract_v3.md` **§4.8**（§4.7 已被 E1b 占用；§12 已被另
> 一个 agent 占用 ⇒ 本节用 §13）。
>
> **日期口径**：本节所有跑的 `meta.txt` 的 `date` 都是 **2026-10-19T22:32–22:47**（本机时钟）；
> 「2026-10-21」在本文件里是 §10/§11 的任务标签（§12 已指出同一问题）—— 本节一律以 `meta.txt` 为准。
>
> 两个**互相独立**的提交：**① 插件修复**（`src/rm_simulation/livox_laser_simulation_RO2/src/
> livox_points_plugin.cpp:363-366`）、**② 世界校准**（`src/rm_simulation/hzmi_rm_simulation/world/
> RMUL2026_world/RMUL2026_world.world:62,104`）；本节随文档单独提交。**`model.sdf` 没有动**（它不是
> 生效资产，见 `docs/worlds.md` §2.1）。
>
> 隔离：私有 `HOME` / `ROS_DOMAIN_ID` / `GAZEBO_MASTER_URI`、`gui:=False`、`world:=X` 不带前导 `--`、
> 收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）；每跑 `meta.txt` 记
> `date/git_head/world_md5/model_sdf_md5/launch_md5/lio_so_md5/ros_domain_id/gazebo_master_uri`。
> 原始数据 `.tmp_cal/<tag>/`（gitignored）。探针：`.tmp_cal/floor_probe.py`（世界系平面拟合）、
> `.tmp_cal/spots_probe.py`（`gz model` 传送 + 每个点两法测量）、`.tmp_cal/wall_witness.py`
> （拿**网格里已知的竖直墙面**当基准量点云平移）、`tools/scripts/tiltmount/nav_band_probe.py`（nav）。

### 13.1 一句话结论

> **h_meas 的 7 cm 从来不是地面的错，是 Livox 插件在 `cloud_frame=sensor` 档把「父 link 系的安装
> 偏移」加进了「传感器系的点」里** ⇒ 整朵云被刚性平移 `R_joint·offset = (0.0006, 0.1919, 0.07053) m`
> （robot11，base_link 系）。两个世界**同病**：RMUC2026（地面已校准在 0）实测
> **h_meas = 0.1906 / ERROR**，RMUL2026 = **0.1894 / ERROR**。修好后 **RMUC 0.2597 / INFO**、
> **RMUL 0.2603 / INFO**；侧向量法用网格墙面独立验证：y 向残差 **−0.1919 m → 0.0000 m**。
> 剩下的一件**独立**的事：RMUL2026 的场地确实比 world 原点高 **+0.049662 m**（RMUC2026 是 0），
> 已按同一约定校准到 **0**（Δz = −0.049662，只动 world 内联模型的 visual+collision pose z）。
> 地面在 ±6 mm 内**不是**理想平面（mesh 大三角形有 ~1 cm 级台阶）⇒ 常数偏移只准到 ±6 mm。

### 13.2 两个**互相独立**的量法（为什么这次不会再被同一个 bug 骗）

| 量法 | 量的是什么 | 用不用雷达 | 精度（实测） |
|---|---|---|---|
| **① 接触法**（新，本轮主判据） | 机器人静止时 4 个轮 cylinder 碰撞的最低点所接触的面：`GT base_link z − 0.102499`（URDF 轮底平面到 base_link） | **不用** | 与 URDF 几何一致到 **2 µm**（RMUC 实测 0.102497、RMUL 实测 0.152161） |
| **② 雷达法**（§10.3 的老口径） | 原始云 `/livox/lidar/pointcloud`（`livox_frame`）→ 用 `/odom_ground_truth` + TF `base_link←livox_frame` 搬到 **world** → 直方图峰 + SVD 精修平面 | 用 | p50 0.2–1.0 mm；**但含传感器侧误差** ⇒ 修 bug 前必须与 ① 对比才能判 |

`①` 是**关键**：它完全不经过点云。RMUC2026 的场地按构造就在 0（§2.1），机器人静止时
`GT base_link z = 0.102497` ⇒ 接触面 = **−0.000002 m** —— 说明 ① 的口径是对的；同一时刻雷达法给
**+0.0706** ⇒ 差别只能来自雷达侧。

### 13.3 实测：RMUL2026 的地面 z 逐点 + spread（**校准前**）

用 `gz model -m robot -x X -y Y -z 0.2` 把机器人依次传送到 5 个点（每点等 5 s 稳定），同一跑里
**两种量法都做**（跑 `v2_rmul`，插件已修、world 未校准）：

| # | 传送点 (x, y) m | GT base_link z | **① 接触地面 z** | ② 雷达平面 z（0.8–3.5 m） | ②−① |
|---|---|---|---|---|---|
| 0 | **4.3, 3.35（出生点）** | 0.152161 | **+0.049662** | +0.049123 | −0.00054 |
| 1 | 2.8, 3.35 | 0.152161 | **+0.049662** | +0.046955 | −0.00271 |
| 2 | 4.3, 5.35 | 0.152754 | **+0.050255** | +0.043461 | −0.00679 |
| 3 | 3.0, 5.0 | 0.152650 | **+0.050151** | +0.039795 | −0.01036 |
| 4 | 5.4, 4.2 | 0.205757 | +0.103258 ← **高台**（不在可行驶面里） | +0.253765 | +0.15051 |

* **可行驶面（#0–#3）= +0.049662 … +0.050255 ⇒ median +0.049907、spread 0.593 mm。**
* **地面在 mm 级不是一个平面**：mesh 里最大的水平三角形（可行驶面的**顶面**）落在 0.045 / 0.055 /
  0.065 m 三档（相邻 ~1 cm 台阶；底面在 −0.05 m，是那块 0.1 m 厚板的背面；整份 STL 只有 3187 个三角形）⇒ 雷达在 0.8–3.5 m 上拟合出的平面比"轮子实际踩到的
  那一层"低 **0.5–10 mm**（②−① 列）。**结论**：主体偏移（4.97 cm）是确定的常数，但**常数偏移只准到
  ~±6 mm**；`#4` 那种高台不能用常数描述（已从判据里剔除）。
* 这与 §10.3 的"可见地面 0.1183"差 6.9 cm，正好是插件偏置（见 13.4）。

### 13.4 根因：`cloud_frame=sensor` 档把父 link 系的 `offset.Pos()` 加进了传感器系点云

**代码**（修前，`livox_points_plugin.cpp:349`）：

```cpp
auto axis = cloud_frame_sensor_ ? (mount_rot_ * ray * x̂)          // ← 传感器系矢量
                                : (sensor_rot * mount_rot_ * ray * x̂);  // ← 父 link 系矢量
auto point = range_from_origin_ ? (range * axis + minDist * axis + sensor_offset.Pos())
                                : (range * axis);
```

`sensor_offset.Pos()`（robot11 = `(0.000562, 0.130916, 0.157028)`，**父 link 系**）被无条件加进
**传感器系**的坐标里 ⇒ 整朵云刚性平移

`Δ_world = R_world←base_link · R_joint · offset.Pos() = (0.0006, 0.1919, 0.07053) m`（base_link 水平时）。

**三条独立证据**：

1. **z（h_meas）**：预测 `h = 0.259527 − 0.07053 = 0.18899`；实测 RMUL **0.1894**、RMUC **0.1906**
   （差 ≤1.6 mm，= 拟合方差）。
2. **z（雷达可见地面 vs 接触地面）**：修前 RMUL 可见地面 **+0.1209/0.1266/0.1214**（三个距离带）−
   接触 **+0.049662** = **+0.0712 … +0.0770**；RMUC 可见 **+0.0679** − 接触 **0** = **+0.0679**。
3. **y（侧向，网格当基准）**：`wall_witness.py` 取 mesh 里**从机器人可见**的竖直面（内侧面），
   在点云里（world 系）拟合同一平面，残差 = 点云平移在该法向上的投影。修前/修后（同一跑 `explore2`
   /`v_rmul`，机器人 (4.3,3.35)）：

| 网格面（world） | 面法向 | 修前残差 | 修后残差 |
|---|---|---|---|
| x = 2.5（内侧面，跨度 5.6 m） | +x | **+0.0006 m** | **0.0000 m** |
| x = 5.0（内侧面，跨度 4.8 m） | −x | **−0.0006 m** | **0.0000 m** |
| x = 10.0（内侧面，跨度 2.4 m） | −x | **−0.0006 m** | **0.0000 m** |
| **y = 8.5（内侧面，跨度 6.5 m，resid 0.0000）** | −y | **−0.1919 m** | **0.0000 m** |

⇒ x 分量 0.6 mm（预测 0.00056 ✓）、**y 分量 −0.1919 m（预测 −0.19192 ✓）**、z 分量另由 h_meas 证。
三个分量都对上 ⇒ 不是"地面高了"，是**整朵云被搬走了**。
（表里另外两个面 x=2.22 / y=8.78 的 +0.28 m 是**相邻面进窗**造成的假读数，修前修后同值，与本 bug 无关。）

**修法**（提交 ①，`livox_points_plugin.cpp:363-366`，最小 diff）：

```cpp
const ignition::math::Vector3d cloud_origin =
    cloud_frame_sensor_ ? ignition::math::Vector3d::Zero : sensor_offset.Pos();
auto point = range_from_origin_ ? (range * axis + minDist * axis + cloud_origin)
                                : (range * axis);
```

* **`parent` 档逐字节不变**：`cloud_frame_sensor_ == false` 时 `cloud_origin` 就是原来的
  `sensor_offset.Pos()` ⇒ 其它所有模型（默认模型 / hzmirm / `robot11_mount:=plugin|urdf`）不受影响。
* **不加新开关**：sensor 档的默认行为就是"修好的"；回退 = 把 `cloud_origin` 改回
  `sensor_offset.Pos()`（一行）。`range_from_origin:=false` 仍只管 range 语义。
  插件启动日志现在会打印走哪一支（`range·axis + minDist·axis（传感器系，**不含** offset.Pos()）`）。
* 顺带更新了启动日志那一行（只说清语义，不参与计算）。

### 13.5 `h_meas` 前后（两个世界，同一个 `robot11_mount:=sensor`、`gravity_align_mode:=hybrid`）

| 世界 | 修前（插件 bug） | 修后（未校准 / 已校准） | 判据 |
|---|---|---|---|
| **RMUL2026** | `h_meas = 0.1894`，\|Δ\|=0.0701 ⇒ **ERROR** | **0.2603**，\|Δ\|=0.0007 ⇒ **INFO**（未校准）/ **0.2616**、\|Δ\|=0.0021 ⇒ **INFO**（校准后；复测 0.2615/0.0020） | <0.03 INFO |
| **RMUC2026**（对照，地面本来就在 0） | `h_meas = 0.1906`，\|Δ\|=0.0690 ⇒ **ERROR** | **0.2597**，\|Δ\|=0.0001 ⇒ **INFO** | <0.03 INFO |

* RMUC2026 的对照是**决定性的**：它的地面**由构造**在 0（§2.1：底板顶面 mesh 局部
  −1.6413436 + pose +1.6413436），机器人静止时 `GT base_link z = 0.102497` ⇒ 接触面 −2 µm。
  它的 h_meas 也是 0.19 ⇒ **7 cm 与"地面在哪"无关**。
* 修后两个世界的残差 |Δ| ≤ 2.1 mm（拟合 p50 0.3–1.7 mm）⇒ 与插件预测 0.18899 一致。
* inliers 12456–14435、tilt 0.04–0.10°、resid p95 2.6–4.6 mm（修前 15547/0.05°/1.8 mm）——同一量级。

### 13.6 RMUL2026 地面校准（独立提交 ②）

* **Δz = −0.049662 m**（= −实测接触地面；标称 `model pose 2.65 − visual pose 2.6 = 0.05`，差的
  0.338 mm 是 mesh 地面平面在局部 y 上的残差 ⇒ 用实测值而不是标称 0.05）。
* **改哪**：生效资产是 `RMUL2026_world.world` 里**内联**的 `RMUL_2026` 模型（`docs/worlds.md` §2.1：
  `model.sdf` 那份**未生效**，且它是 scale 0.01 / 旋转 +1.5708 的另一套写法）⇒
  **`<visual>` 和 `<collision>` 的 `<pose>` z：−2.6 → −2.649662**（`RMUL2026_world.world:62,104`；
  旋转 −1.5705、scale 0.001 **不动**；model pose 8.5/4.5/2.65 也不动）。`model.sdf` **没动**。
* 文件里加了同风格的 SDF 注释（实测值、量法、日期、spread、残余、以及"model.sdf 不生效"）。
* **校准后验证**（跑 `cal_rmul`，同一套 5 点传送）：

| # | 传送点 | GT base_link z | ① 接触地面 z | ② 雷达平面 z |
|---|---|---|---|---|
| 0 | 4.3, 3.35 | 0.102499 | **+0.000000** | −0.000563 |
| 1 | 2.8, 3.35 | 0.102499 | **+0.000000** | −0.004028 |
| 2 | 4.3, 5.35 | 0.103092 | **+0.000593** | −0.007766 |
| 3 | 3.0, 5.0 | 0.102988 | **+0.000489** | −0.007735 |

  ⇒ 接触地面 median **+0.000245**、spread **0.593 mm**、max 0.593 mm；雷达平面 −0.0006…−0.0078 m
  （= 13.3 说的 mesh 台阶，**不是**校准误差）。出生点 floor probe 的三个距离带
  **+0.0019 / +0.0061 / −0.0024**、整体 **−0.0015**；网格墙面残差全 **0.0000**。
* **标称 vs 实测**：若图省事把 z 写成 −2.65（= 标称 0.05），地面会落在 **−0.338 mm**；本轮用实测
  −2.649662，落点 0.000。

### 13.7 出生 / 落地行为（**spawn z 没有改**）

* launch 的出生点仍是 `rm_simulation.launch.py` 的 `WorldType.RMUL2026: x=4.3, y=3.35, **z=0.2**`
  （与 RMUC2026 同口径）；校准后地面在 0 ⇒ 落差 0.2 − 0.102499 = **9.75 cm**，**物理自己落地**。
* 稳态实测（`settle_cal`，hybrid）：`GT base_link z = 0.102497 ± 0.000003`（3 µm，9 个样本），
  即**轮底平面 = 0.102497 − 0.102499 = −2 µm = 地面** ⇒ **不悬空、不下陷**，无需调 spawn z。
* **下落速率不是自由落体**：`gazebo_ros_planar_move` 每个物理步都把线速度的 z 分量清零 ⇒ 从
  z=0.45 传送后实测下沉 **≈2.2 cm/s**（0.4458 → 0.3811 / 3 s），即 ~9.75 cm 的落差约 **4.4 s** 落完。
  LIO 初始化在 launch 后 ~13 s ⇒ 一定在落地之后（不会"坠落中做重力初始化"，§9 的老坑不触发）。
* 校准前同一机器人的稳态是 0.152161（地面 0.049662）⇒ 地面降 4.966 cm、机器人跟着降 4.966 cm。

### 13.8 nav A/B（`mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor`，1 跑/档）

口径：`settle=10 s / window=10 s`（仿真），同一个插件（修后）、同一台机；`navband` 的
`local_costmap` 中位数。**A = 校准前、B = 校准后**（同一插件 ⇒ 只变世界 z）：

| 指标 | `off` **A 校准前** | `off` **B 校准后** | `hybrid` **A 校准前** | `hybrid` **B 校准后** | §10.4 参考（**修 bug 前**、20/20、n=3） |
|---|---|---|---|---|---|
| local **lethal** | 264 | **284** | 285 | **292** | off 193 / hybrid 288.5 |
| local inscribed | 9231 | 8865 | 9568 | 9575 | 8280.5 / 8452 |
| local free | 43658 | 44314 | 42428 | 42374 | 44546 / 43312.5 |
| **最近 lethal** | 0.7215 m | **0.7612 m** | 0.7072 m | **0.7070 m** | 0.760 / 0.769 |
| **前方 2.0×1.2 m lethal** | 19 | **16** | 27 | **30** | 21 / 18 |
| 车 1 m 内 lethal | 26 | 24 | 33 | 41 | 26 / 23.5 |
| `/scan` 有限波束/帧 | 775/1462 | 776/1462 | 776/1462 | 775/1462 | 677（§10.4 只记了有限波束，总波束未记） |
| 波束 z(odom) p50 | −0.0461 | −0.0899 | +0.0738 | +0.0358 | +0.243 / +0.058 |
| 在带 `[0,2]` 内 | 346 | 333 | **776** | **775** | 579 / 678（off/hybrid） |
| 带下 / 带上 | 426 / 3 | 439 / 3 | 0 / 0 | 0 / 0 | 60/36 / 0/0 |
| global lethal/inscribed/free | 3725/16875/10175 | 3713/16886/10176 | 3713/16886/10176 | 3713/16886/10176 | 3713/16886/10176 |

**判读**

1. **没有回归**：两档的 local costmap 都**非空**（lethal 264–292）、最近 lethal **0.71–0.76 m**、
   前方窗口 **16–30** 格 —— 与 §10.4 的 193/288.5、0.760/0.769、21/18 **同量级**；没有出现 §9.7 那种
   `lethal 0 / 整张 free / ahead 0`。
2. **A vs B（唯一变量 = 世界 z 降 4.966 cm）**：hybrid 的 lethal **285 → 292（+2.5%）**、最近
   0.7072 → 0.7070、ahead 27 → 30；off 的 264 → 284、0.7215 → 0.7612、19 → 16。**都在单跑方差内**
   （§10.4 自己的 3 跑 spread 就有 lethal 193–210 / 288–345）。⇒ 平移整个世界**不改变** nav 行为
   （车和场地一起降，相对几何不变），符合预期。
3. **绝对数不能与 §10.4 逐字比**：① §10.4 是**修 bug 前**测的（那时整朵云沿 base_link +y 偏
   19.2 cm、+z 偏 7.05 cm）⇒ `/scan` 的进带集合与障碍落点都变了；② 本轮 window 10 s vs §10.4 的
   20 s；③ 单跑 vs 3 跑中位。所以上表最后两列只作**量级对照**。
4. **off 档 `in-band` 变差了**（346/333 vs §10.4 579）而 lethal 反而变多：因为修 bug 后 off 档的
   scan 盘面 z 从 −0.046 变到 −0.090（odom 里的 z 漂移 + 世界平移），进带的是**另一批**波束。
   off 档本来就是"未重力对齐"的对照臂，**不是**产品路径；产品路径是 hybrid（775/776 全进带）。

### 13.9 哪些既有数字**被本轮作废/需要重读**

| 位置 | 旧值/旧结论 | 现在 |
|---|---|---|
| **§10.3** | "机器人陷进地面 mesh 6.9 cm""可见地面 world z = 0.1183""odom 里可见地面 ≈ +0.069" | **作废**：0.1183 = 真实接触地面 0.049662 + 插件偏置 0.07053；陷地 **0**（2 µm）。§10.3 的"测量质量"（15547 内点/0.05°/0.2 mm）本身没问题，**解释**错了 |
| **§10.3** | `h_meas = 0.1894`（RMUL） | 修插件后 **0.2603 / 0.2616 / 0.2615**（INFO） |
| **§10.3** | 阈值判据"0.03 INFO / 0.05 ERROR 抓到了 URDF 与地面的差" | 那个 ERROR 是**传感器 bug**，不是 URDF/地面的差；修后 RMUL/RMUC 都 INFO |
| **§10.4.1 / §10.9** | "scan 盘面 z / base_link z(odom) / ground in odom" 一组绝对值。具体两个：`.tmp_ab/zf2_hybrid_r1/probe.json` 的 `sensor_height_m_median = 0.01876`、`.tmp_ab/nav_zfix_smoke/probe.json` 的 `= 0.02674`（odom 里地面到 odom 原点的距离，即 z-fix A/B 的 "ground in odom"） | 都是**修 bug 前**的：整朵云 +0.0705(z)/+0.1919(y) ⇒ 同一定义的新值 ≈ **0.01876−0.07053 = −0.0518** / **0.02674−0.07053 = −0.0438**（y 向同减 0.1919）。本轮**没有重跑** §10 的 6 跑矩阵 ⇒ 只给这条换算、不给实测新值 |
| **§12**（另一个 agent 的 5 跑） | 其 `/scan`、costmap、`map→odom` 数值 | 同一批 `sensor` 档数据 ⇒ 同样带 (0.0006, 0.1919, 0.0705) 的偏置；结论方向不受影响（off vs hybrid 的**相对**比较），但**绝对 z** 需重读 |
| **所有 RMUL2026 的 world z**（本文件、`docs/worlds.md`、地图/PCD 元数据、任何"场地在 z=…"） | 场地在 +0.049662 | 校准后 **0**：RMUL2026 的 world z 一律 **−4.966 cm**（含出生点静止高度 0.152161→0.102497、雷达可见地面 0.0474–0.0503→−0.0015…+0.0061） |
| **linefit `sensor_height: 0.2595`（robot11）** | 与 URDF 一致，但运行期点云的地面在 0.189 ⇒ 差 7.05 cm | 修 bug 后**真的**一致（h_meas 0.2597–0.2616）⇒ `sensor_height` 这条标定从"账面对"变成"实测对" |
| **自击掩膜 AABB（`traversability_self_mask_robot11.yaml`，113 个 box，base_link 系）** | 按 base_link 的真实车体几何做的 | **被错位了 (0.0006, 0.1919, 0.0705)**：自击点被搬到 box 的 +y/+z 侧，而 box 只按真实位置切 ⇒ 既可能**切掉真实几何**、也可能**漏掉自击点**（近场死区 r≤0.2416 m 也罩不住被搬远 0.19 m 的自击）。修后**重新对齐**，掩膜配置**不需要改**（它本来就在 base_link） |
| **既有 PCD / 2D 地图（RMUL/RMUC，LIO 系）** | 由带偏置的云建出来的 | 地图相对**真值几何**偏 (0.0006, 0.1919, 0.0705)（odom 系里的常量）；LIO/配准是**自洽**的 ⇒ nav 仍能用，但"地图 z = 真值 z"不成立。**本轮没有重建地图/PCD**（不在任务范围），要不要重建由用户定 |

### 13.10 本轮没能验证 / 诚实边界

1. **nav 只跑了 1 跑/档**（A/B 各 2 跑，共 4 跑），不是 §10.4 的 3 跑/档；window 10 s 而非 20 s
   ⇒ 表里的差**分辨不出** ≤10% 的变化。没有重跑 §10 的 6 跑矩阵，也没有重跑 §12 的 5 跑。
2. **`parent` 档没有实跑**：只做了代码级论证（`cloud_frame_sensor_==false` 时 `cloud_origin` 与
   原式逐字节相同）+ 启动日志；默认模型 / hzmirm / `robot11_mount:=plugin|urdf` 的 h_meas 未测。
3. **地面不是理想平面**（±6 mm，见 13.3）：常数偏移只能到 ±6 mm；本轮的 4.97 cm 是确定的，但
   "world z=0 面上的残余"在别的点可能到 ±1 cm。没有做全场地的高度图。
4. **`model.sdf` 未修**：它不生效（`docs/worlds.md` §2.1），但仍是"scale 0.01 / 旋转 +1.5708 /
   z=0"的老写法；如果将来有人改成 `model://` 加载，需要按 0.001 / −1.5705 / −2.649662 重写。
5. **`wall_witness` 的相邻面混淆**：x=2.22 / y=8.78 两个面落在 ±0.3 m 窗内被相邻面顶掉
   （+0.28 m），修前修后同值 ⇒ 与本 bug 无关，但说明这个探针**只能用在"面间距 > 0.6 m"的墙上**。
6. **RMUC2026 的墙基准不清楚**：那份 mesh 的竖直面碎（曲/斜），只量到 1 个面
   （−0.0367 m、resid 0.042、span 1.77 m）⇒ RMUC 的**侧向**witness 不可用；y 向偏置的结论靠 RMUL 的
   干净墙面 + 两界一致的 z。
7. **落地速率归因**：只观察到 ~2.2 cm/s 的匀速下沉，归因于 `planar_move` 每步清 z 速度（代码推断），
   没有单独做"关掉插件"的对照。
8. **本轮没动** `src/rm_localization/**`、`tools/scripts/**`、任何 launch/config；改的只有 1 个插件
   源文件 + 1 个 world 文件 + 本节文档。

### 13.11 一句话给用户

> **7 cm 的锅不在世界，在 Livox 插件**：`cloud_frame=sensor` 档把父 link 系的安装偏移加进了传感器系
> 点云，整朵云偏 (0.6 mm, 19.2 cm, 7.05 cm)；两个世界同病（RMUC 已校准也 0.19/ERROR）。修好后
> **RMUL h_meas 0.2603、RMUC 0.2597，都 INFO**，网格墙面残差 **−0.1919 → 0.0000 m**。
> 另外把 RMUL2026 的场地从 **+0.049662 m 校到 0**（Δz = −0.049662，只动 world 内联模型的
> visual+collision pose z），校准后接触地面 **+0.000245 ± 0.0006 m**、机器人静止
> **0.102497 ± 3 µm**（轮底平面 = 地面）、nav 两档 lethal **284/292**（校准前 264/285）无回归。
> 地面在 ±6 mm 内不是理想平面（mesh 大三角形 ~1 cm 台阶）⇒ 常数偏移只准到 ±6 mm。

---

## §14 第六轮（**本机 date=2026-10-19T22:46–23:06**）：**GICP 首帧门死锁** —— 复现、两个成因、修法、验收（含 `global_obstacle:=scan` 的全局半）

> **本节只新增**（append-only）：§1–§13 一个字都没改，只在 **§9.10 第 7 条**加了一行「订正」
> （那条的 "off 118 / hybrid 2" 是错的，见 §14.9）。§12 已被另一个 agent 占用 ⇒ 本节用 §14。
>
> **对象**：`docs/localization_slots.md` 的 `localization:=gicp` 槽。§12.4 把「gicp 首帧门死锁」
> 登记为"要先修才能判 `global_obstacle:=scan` 的全局半"的那件事。
>
> **代码**（两组提交，都在 `src/rm_localization/gicp_registration/` 与 `tools/scripts/localization/`）：
> **① 诊断** `d475605`（`~/gate` 话题 + 探针，**门限逻辑零改动**）/ `bcb652b` + `dafcb4e`
> （创新**分量** dx/dy/dz、探针 `/clock` 改 BEST_EFFORT）；**② 修门** `9bb8f2f`
> （4 个新参数，默认 = 修好的行为）。
> 改前/改后跑的是**同一台机、同一个 launch**（md5 `6d38e74e…` 六跑全同）、同一个
> `libros2_livox.so`（md5 `7bbab875…`，六跑前后逐值相同）、同一个 `libsmall_point_lio.so`
> （md5 `58bebfb8…` = §10/§13 的 zfix）；**唯一变量 = `libgicp_registration.so`**
> （改前 `1123db91…` ⇄ 改后 `53c7067d…`）。
> 原始数据 `.tmp_ab/zt_*/`（gitignored；每跑 `gate_meta.txt` 有 `date/git_head/launch_md5/
> gicp_so_md5/livox_plugin_md5/ros_domain_id/gazebo_master_uri` + 两个"跑前后冻结"校验）。
> 探针：**新增** `tools/scripts/localization/gicp_gate_probe.py`（只订阅：`~/gate`、`~/pose`、
> `~/fitness_score`、`~/converged`、`/tf` 的 `map→odom`、`/clock`）+ 既有的
> `tools/scripts/tiltmount/nav_band_probe.py`（成本图/高度带）+ `spl_ab_bench.sh` 的 `uc_procstats`。
>
> **隔离**：私有 `HOME`/`ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI`、`gui:=False`、`world:=X` 不带前导 `--`、
> `setsid` 起栈、收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）。
>
> ⚠️ **两跑之间的第三个变量（诚实前置）**：`ec5028f` 在 22:46:58 把 **RMUL2026 的 world 地面
> 校准 −4.966 cm**（§13.6）—— 我的「改前」跑 22:46:57 正好在它之前 ⇒ 改前用**未校准世界**、改后用
> **校准后世界**。这影响的只是**绝对 z**（`/scan` 在 map/odom 里的 z、`map→odom` 的 z），
> **不改变门的三个量**（创新/dt/门限都是相对量）。反证：改后**首帧创新仍有 0.1138 m > 旧门限 0.100 m**
> ⇒ 只换世界、不修门，照样死锁（见 §14.5）。

### 14.1 一句话结论

> **病复现了，而且是两个独立缺陷**：① **语义自锁** —— 门限的 `dt` 定义成"与**上次采纳帧**的戳差"，
> 首帧必为 0 ⇒ 门限永远取下限 **0.100 m** ⇒ 实测创新 **0.11–0.22 m** 全被拒 ⇒ 永远没有"上次采纳"
> ⇒ `dt` 永远是 0（**25/25 帧**，与"是否真发散"无关）；② **创新里含真实常量错位 ~0.19 m** ——
> 但**不是** Livox 插件（本轮开跑时插件修复已在 build 里，见 §14.3），而是 §13.9 最后一行那条
> **PCD/2D 地图是用带偏置的云建的**。修法：`dt` 改用**上一帧收到的**点云戳 + **无先验首帧免创新门**
> + **连续 3 帧被拒就自愈重锚** + **判失效后每秒探针**（四个开关，默认修好，逐字节回退可见 §14.4）。
> 改后同一配方：**采纳 387/387 = 100%**（改前 0/25）、创新 **p50 0.0097 / p95 0.0243 / max 0.1138 m**
> （改前 0.134 / 0.202 / 0.216）、`map→odom` **整跑 50 Hz 广播 / 10.0 Hz 唯一戳**（改前只活 ~6 s）、
> **`Message Filter dropping` 42 → 0**、planner 外推 **271 → 3**、判失效 **30 → 0**。
> **但 `global_obstacle:=scan` 的全局半仍然判不了**，而且这次能证明**不是 gicp 的锅**：在"新插入一个
> 地图里没有的 25×25×30 cm 盒子、距车 0.45 m"的决定性试验里，**局部**代价图立刻看到它
> （最近 lethal **0.71 → 0.33 m**），**全局**图 **逐值不变**（3713/16886/10176，33 帧 min=max）——
> 而 3713 恰好等于 `RMUL2026.pgm` 里 **occupied 格数**（离线数出来 3713）⇒
> 全局那张图**只有 static+inflation，2D scan 层一个格子都没落地**。这是**另一个** defect（在 nav2
> 参数/全局层这一侧），不是本轮能修的。

### 14.2 复现（改前）：`采纳 0/25`、`dt = 0.000 s` 的**逐帧**证据

配方（与 §10.4 / §12 逐字相同）：`world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp
robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid`、`gui:=False`、
`settle=20 s / window=20 s`（仿真）。跑 `zt_gate_before_r1`（HEAD `d475605` = **只加了诊断话题**，
门的判据/采纳/平滑/发布**一行没改**）。

**逐帧（前 10 帧，`~/gate` 原始值；字段含义见 §14.4 的字段表）**：

| # | 创新·平移 (m) | 创新·yaw (°) | **dt (s)** | **门限·平移 (m)** | score (m²) | converged | 采纳 | 连续拒绝 |
|---|---|---|---|---|---|---|---|---|
| 1 | **0.1983** | 0.075 | **0.000** | **0.100** | 0.0050 | true | ✗ | 1 |
| 2 | 0.1116 | 1.605 | 0.000 | 0.100 | 0.0050 | true | ✗ | 2 |
| 3 | 0.1154 | 1.533 | 0.000 | 0.100 | 0.0052 | true | ✗ | 3 |
| 4 | 0.1251 | 1.376 | 0.000 | 0.100 | 0.0054 | true | ✗ | 4 |
| 5 | 0.2157 | 0.168 | 0.000 | 0.100 | 0.0050 | true | ✗ | 5 |
| 6 | 0.1293 | 1.541 | 0.000 | 0.100 | 0.0051 | true | ✗ | 6 |
| 7 | 0.1297 | 1.381 | 0.000 | 0.100 | 0.0054 | true | ✗ | 7 |
| 8 | 0.1319 | 1.198 | 0.000 | 0.100 | 0.0053 | true | ✗ | 8 |
| 9 | 0.1372 | 1.299 | 0.000 | 0.100 | 0.0050 | true | ✗ | 9 |
| 10 | 0.1376 | 1.429 | 0.000 | 0.100 | 0.0051 | true | ✗ | 10 |

**整跑（25 帧，然后在第 25 帧判失效 ⇒ 停发 TF）**：

| 量 | 实测 |
|---|---|
| 采纳 | **0/25（0.0%）** |
| 创新·平移 p50 / p95 / max | **0.1342 / 0.2016 / 0.2157 m** |
| **`dt`** | **p50 = min = max = 0.000 s；25/25 帧都是 0** |
| **门限·平移** | **p50 = min = max = 0.100 m**（= `gate_trans_step_min`，**下限**；`3.0 m/s × 0 s = 0`） |
| 门限·yaw | 恒 3.000°（同样是下限） |
| fitness score | p50 0.00515 / max 0.00545 m²（`max_fitness_score: 0.3` ⇒ **配准自洽得"很好"**） |
| source 点数 / align | 3024 点 / 14–76 ms（p50 25 ms） |
| 连续拒绝 | 1 → 25 单调（**从未清零** ⇒ 一次也没采纳） |
| `map→odom` | **312 条 / 25 个唯一戳**；50.2 Hz 广播、**4.02 Hz 唯一戳**；**只活在 ~6.2 s（wall）里**（探针记录跨度 14.03→20.25 s） |
| `~/pose` | 24 条（与 25 帧对应，最后一帧判失效后不再发） |
| 判失效 | `定位已判失效` 30 条、`[status] 定位失效：已停发` 122 条（`lost_after_rejections: 25`） |
| nav2 后果 | **`Message Filter dropping` 42 条**（global costmap）、planner `Extrapolation Error` **271 条** |

日志逐字（`zt_gate_before_r1/launch.log`）：

```
[ERROR] [gicp_registration]: ★★ 定位已判失效（连续 25 帧被拒）：**本节点不再发布 map→odom / ~/pose**…
[ERROR] [gicp_registration]: [status] backend=pcl **定位失效：已停发 map→odom**（连续 25 帧被拒 ⇒ 阈值 25）|
        累计 门限拒绝 25 / 越界拒绝 0 | 采纳 0/25 帧 | 最近 score=0.00495 m² | 等 /initialpose…
[ERROR] [planner_server]: Extrapolation Error looking up target frame: Lookup would require extrapolation
        into the past.  Requested time 645.232000 but the earliest data is at time 664.382000,
        when looking up transform from frame [base_link_fake] to frame [map]
```

⇒ **`score` 一路 0.005 m²（"完美"）、`converged` 一路 true、`dt` 一路 0、门限一路卡在下限** ——
这正是 §9.10 / §12.4 那类"失败对既有健康判据完全不可见"的形状，只是这次**判据自己的输入坏了**。

### 14.3 两个成因（**都要修**，且都不是"发散"）

**① 语义自锁（主因，纯逻辑）**：门限 = `clamp(rate·dt, 下限, 上限)`，而 `dt` 的注释与实现都是
「与**上次采纳帧**的点云戳差」（`gate_dt` 段，改前 `gicp_registration.cpp:1018-1030`）。
"上次采纳"在**第一次采纳之前不存在** ⇒ `dt = 0` ⇒ 门限 = `max(0.10, 0) = 0.10 m`。
而真实帧间修正量是 0.11–0.22 m（> 0.10）⇒ **每一帧都被拒** ⇒ 仍然没有"上次采纳" ⇒ `dt` 永远是 0。
**这是一个与数据无关的不动点**：只要第一帧被拒，后面永远不会被采纳。
（改后实测 `dt` = 0.1 s（点云 10 Hz 仿真）⇒ 门限 `3.0×0.1 = 0.30 m`，同一个创新 0.11–0.22 全部放行。）

**② 创新里确实有 ~0.19 m 的真实常量错位** —— 但**不是** Livox 插件（任务书的假设）：
本轮开跑（22:42）时插件的 `cloud_frame=sensor` 修复**已经在 build 里**：
`livox_points_plugin.cpp` mtime **22:19:47** < 目标文件 `…plugin.cpp.o` **22:20:04**、
`libros2_livox.so` md5 `7bbab875…` 六跑前后逐值冻结；§13.5/§13.6 的 `h_meas = 0.2603/0.2616`
（INFO）也是这个二进制测出来的。**真正剩下的那个 0.19 m 在"地图"这一侧**：
§13.9 最后一行已经写明 —— **PCD/2D 地图是用带偏置的云建的**，相对真值几何偏
`(0.0006, 0.1919, 0.07053) m`（odom 系常量）；云修好之后，GICP 看到的就是**云 vs 地图**的常量错位。
改后首帧的创新**分量**（新加的字段 13/14/15）直接指认方向：

| 跑 | 首帧创新 dx / dy / dz (m) | \|dxy\| |
|---|---|---|
| `zt_gate_after_r1`（hybrid） | −0.052 / **+0.101** / **−0.229** | 0.1138 |
| `zt_gscan_hyb_r1`（hybrid） | −0.085 / **−0.143** / **−0.352** | 0.1667 |
| `zt_gscan_hyb_box_r1`（hybrid） | −0.056 / **+0.107** / **−0.221** | 0.1205 |

⇒ 首帧创新**不是各向同性的噪声**：y 分量 0.10–0.14 m（地图的 +y 偏置 0.192 的同一量级），
z 分量更大（−0.22…−0.35 m，含 `initial_pose` 初始化式本身的 z 残差 + 世界 z 系差）。
**门限"正确地"拒绝了一个真有偏置的量测**；错的不是门限的判据，而是**"永久拒绝"这个处理方式**：
一个常量错位只需要被吸收**一次**，之后每帧的创新会回到 ~1 cm（改后实测 p50 **0.0097 m**）。

### 14.4 修法（最小 diff、四个开关、默认 = 修好的行为）

`src/rm_localization/gicp_registration/src/gicp_registration.cpp` + `include/…/gicp_registration.hpp`
+ `config/gicp_registration_sim.yaml`。**不改** GICP 数学、不改 TF/平滑/`~/pose` 的契约。

| 新参数（默认） | 作用 | 逐字节回退 |
|---|---|---|
| `gate_dt_from_last_received: true` | `dt` = 与**上一帧收到的点云**的戳差 ⇒ **第 2 帧起就有效**（仿真 10 Hz ⇒ 0.1 s ⇒ 门限 0.30 m）。只在"收到点云"处更新基准（被 TF/点数不足丢掉的帧也算时间流逝）。非法/同戳/倒序 ⇒ 0（仍退下限） | `false` = 旧语义（与**上次采纳**帧的戳差） |
| `gate_skip_innovation_first: true` | **无先验的第一帧免创新门**：创新 = "结果 − 预测位姿"，而预测位姿只能来自"上次采纳" ⇒ 首帧没有它。**converged / score / 合理性照旧**（垃圾帧仍被拒）。**一次性**：`gate_no_prior_` 只在启动时为 true；**`/initialpose` 之后不豁免**（人工给的就是先验，那条路保留创新门的保护） | `false` = 首帧也过创新门 |
| `gate_reanchor_after_rejects: 3` | **自愈重锚**：连续被拒 ≥3 帧 ⇒ 把发布值与平滑目标都拉回**本帧的里程计预测位姿**（`guess`），并**放行下一个通过 fitness+合理性的帧**（忽略它的创新）。重锚**不编造位姿**（拉回的是"只信里程计"的预测值），豁免采纳的落点仍必须过 `converged/score/合理性` 三道既有判据 | `0` = 关闭（旧语义：只等 `lost_after_rejections` 判失效后冻住） |
| `lost_probe_period_sec: 1.0` | **判失效后不再永久冻结**：每 1 s 仍跑**一帧** align（**只算不发布**），连续 `gate_reanchor_after_rejects`（3）次通过 fitness+合理性 ⇒ 重锚恢复发布。探针帧**不计入**门限/连击统计，也不打印"未采纳"WARN | `0.0` = 关闭（旧语义：判失效后完全停手，只等 `/initialpose`/重启） |

状态机（一句话）：**采纳 → 清零连击**；**拒绝 → 连击 +1**；**连击 ≥ 3 → 重锚 + 置 `reanchor_pending`
→ 下一个 score+合理性通过的帧"豁免采纳" → 清零**；**连击 ≥ `lost_after_rejections`（25）→ 判失效**
（这条**只在重锚关掉时才可能发生**，因为 25 > 3）→ 每秒探针 → 3 次通过即恢复。
`~/gate` 的 `flags` 位标出三类帧：`bit0` 免创新门、`bit1` 本帧重锚、`bit2` 失效态探针帧。

**日志/观测新增**：`~/gate`（`Float64MultiArray`，每帧一条；字段下标 = `publishGateMetrics` 的注释：
0 时刻 · 1/2 创新（平移/yaw） · 3/4 门限（平移/yaw） · 5 dt · 6 score · 7 采纳 · 8 converged ·
9 源点数 · 10 align ms · 11 连击 · 12 flags · 13/14/15 创新**分量** dx/dy/dz）。存在的理由：判据的输入在日志里
**限频**（未采纳 WARN 1 s 一条、采纳帧的 DEBUG 默认不打印）—— "首帧创新多大、`dt` 是不是 0"这类
问题**拿不到逐帧证据**，而 §12.4 的死锁正是这个形状。

**逐字节回退到 2026-10-21 之前**（= 那个死锁，只用于对照）：
`gate_dt_from_last_received:=false` + `gate_skip_innovation_first:=false` +
`gate_reanchor_after_rejects:=0` + `lost_probe_period_sec:=0.0`。

### 14.5 改后验收（**同一配方、1 跑**：`zt_gate_after_r1`）

| 量 | **改前**（`zt_gate_before_r1`） | **改后**（`zt_gate_after_r1`） |
|---|---|---|
| 采纳 | **0/25（0%）** | **387/387（100%）**；`[status]` 末行 **532/532** |
| 创新·平移 p50 / p95 / max | **0.1342 / 0.2016 / 0.2157 m** | **0.0097 / 0.0243 / 0.1138 m**（max = **首帧**，`flags=1` 免门；其余全 ≤0.09） |
| 创新·yaw max | 1.99° | 1.56° |
| `dt` p50 / min / max | **0.000 / 0.000 / 0.000 s**（25/25 = 0） | **0.100 / 0.000 / 0.100 s**（只有首帧 0） |
| 门限·平移 p50 / max | **0.100 / 0.100 m** | **0.300 / 0.300 m** |
| 连续拒绝 max | **25**（触发判失效） | **0** |
| score p50 / max | 0.00515 / 0.00545 | 0.00510 / 0.00545（**同一量级**，配准质量没变） |
| `map→odom` | 312 条 / 25 唯一戳；**只活 ~6.2 s** | **4448 条 / 388 唯一戳**；**整跑**（探针跨度 3.2→41.9 仿真 s）；广播 **114.9 条/仿真秒（= 50 Hz 墙钟）**、**唯一戳 10.03/仿真秒（= 帧率）** |
| `Message Filter dropping`（global costmap） | **42** | **0** |
| planner `Extrapolation Error` | **271** | **3**（启动期，`map→odom` 还没出来时的 2–3 条，随后消失） |
| 判失效 | 30 条 | **0** |
| 局部代价图（`navband` 20 s 窗口中位） | lethal 285 / inscribed 9425 / free 42449 / 最近 **0.7096 m** / ahead **25** | lethal 249 / inscribed 9157.5 / free 42855.5 / 最近 **0.7119 m** / ahead **21** |
| `/scan` 在带 `[0,2]` 内 | 775.5/775.5 | 775/775 |

⇒ **功能上完全恢复**：定位一直在更新、`map→odom` 整跑都在、nav2 的两类告警消失；
**配准质量（score）与局部代价图与改前同一量级**（局部图本来就不吃 `map→odom`，所以它的数
改前改后都"正常"——这正是 §12.4 说"局部有效"的原因）。
**改后首帧创新 0.1138 m 仍 > 旧门限 0.100 m** ⇒ 若只换世界/只改噪声、不修门限，**照样死锁**。

### 14.6 自愈路径专项（运行时把锚点**故意弄错 1.2 m**：`zt_gate_selfheal_r1`）

只经公开接口 `/initialpose`（`ros2 topic pub`），不改任何文件/参数：t≈19.4 s（仿真）时给
`(x=0, y=+1.2, yaw=0)`（真实位姿 ≈ (0,0)）。`~/gate` 逐帧：

| t (仿真, 相对) | 创新·平移 (m) | 门限 (m) | 采纳 | 连续拒绝 | flags |
|---|---|---|---|---|---|
| 19.40 | **0.845** | 0.300 | ✗ | 1 | 0 |
| 19.50 | **0.817** | 0.300 | ✗ | 2 | 0 |
| 19.60 | **0.823** | 0.300 | ✗ | 3 | 0 |
| 19.70 | 0.846 | 0.300 | **✓** | **0** | **2 = 重锚豁免** |

日志逐字：

```
[INFO] /initialpose 更新初值：map 系机器人位姿 x=0.000 y=1.200 z=0.000 yaw=0.00° ⇒ map→odom = x=0.023 y=1.151 …
[WARN] ★ 连续 3 帧被拒 ⇒ **重锚**：map→odom 与平滑目标都拉回本帧的里程计预测位姿 x=0.046 y=1.229 z=0.233 …
[WARN] ★ 重锚后的**豁免采纳**：本帧创新 0.8463 m / 0.756°（门限 0.3000 m / 6.000°）被忽略，
       但 converged=true、score=0.00514 m²（阈值 0.300）、合理性=通过 ⇒ map→odom = …
```

整跑：**386/389 = 99.2%** 采纳（**恰好 3 帧被拒**）、`streak_max = 3`、判失效 0 条、
`Message Filter dropping` 0、外推 3 条。⇒ **"连续拒绝"不再等于"永久冻结"**，
而且恢复后**没有**丢掉创新门（19.70 之后照常 0.3 m 门限、创新 ~1–3 cm）。

### 14.7 全局半（`mode:=nav localization:=gicp global_obstacle:=scan` × {off, hybrid}，1 跑/档）

协议与 §10.4/§12 逐字相同（`settle 20 / window 20`，1 跑/档）。**两档的 gicp 都是活的**
（采纳 100%、`map→odom` 整跑、`Message Filter dropping` = 0）：

| run | 局部 lethal/inscribed/free/unknown | 局部 最近 · ahead | **全局** lethal/inscribed/free/unknown（格数） | 全局 最近 · ahead | `map→odom` | TFDrop |
|---|---|---|---|---|---|---|
| `zt_gscan_off_r1`（off） | **284 / 8953 / 44144 / 0** | **0.7544 m** · **17** | **3713 / 16886 / 10176 / 0**（43077） | 0.4693 · 82 | 活（108.7 条/仿真秒，唯一戳 10.03/仿真秒） | **0** |
| `zt_gscan_hyb_r1`（hybrid） | **339 / 9564 / 42309 / 0** | **0.7043 m** · **21** | **3713 / 16886 / 10176 / 0**（43077） | 0.4573 · 81 | 活（116.5 / 10.03） | **0** |

**这两张"全局"图是 static+inflation，2D scan 层一个格子都没进去** —— 证据不是"数没变"，而是
**离线算出来的常数**：`RMUL2026.pgm` + `RMUL2026.yaml`（`mode: trinary`、`occupied_thresh 0.65`、
`resolution 0.05`、`173×249`）里 **occupied = 3713 格**，与两跑的全局 lethal **逐值相等**；
inscribed 16886 = 这 3713 格按 `robot_radius 0.22` 膨胀出来的内切带。
⇒ 全局图**恰好等于** static+inflation。

**决定性试验（插入一个地图里没有的盒子）**：`zt_gscan_hyb_box_r1`（同样配方），在 t≈25 s（仿真）
用 `/spawn_entity` 往 world 里塞一个 **25×25×30 cm 的静态盒子**（world (4.75, 3.35, 0.15)，
距车 ≈0.45 m，在车头正前方）：

| 量 | 盒子前（`zt_gscan_hyb_r1`） | 盒子后（`zt_gscan_hyb_box_r1`） |
|---|---|---|
| 局部 **最近 lethal** | 0.7043 m | **0.3285 m**（min 0.3272 / max 0.3295） |
| 局部 inscribed | 9564 | **10240** |
| `/scan` 在带内 | 775.5/775.5 | 774/774（z p50 +0.0379） |
| **全局 lethal / inscribed / free** | 3713 / 16886 / 10176 | **3713 / 16886 / 10176（33 帧 min = max）** |
| 全局 最近 lethal | 0.4573 | **0.4526–0.4546**（没有台阶） |
| 全局 robot_z（map） | +0.0285 | +0.0378（在带 `[0,2]` 内） |

⇒ **局部层立刻看见盒子（最近 lethal 掉 0.38 m）、全局层完全没看见**。而同一套机制在
`global_obstacle:=stvl` 档是**工作的**（`zt_gate_after_r1`/`zt_gate_selfheal_r1` 的全局
lethal **3725–3728**、inscribed **16871–16875**，33–34 帧里 **min≠max** ⇒ 图层确实在改图）。
⇒ 结论：**`global_obstacle:=scan` 的全局层是"哑"的**（订阅建了、TF 没丢、扫描面在带内、
局部同一份 `/scan` 正常），这是 **nav2 全局层这一侧另一个 defect**，**不是** gicp/重定位器的锅。

**因此"全局半"的判决（本轮能给的最强结论）**：
1. §12.4 的阻塞（gicp 死锁）**已解除**：两档 `map→odom` 整跑活着、`Message Filter dropping` 0（改前 38–42）。
2. 但 **`global_obstacle:=scan` 这条槽位本身不工作** ⇒ "全局图有没有变空 / 是否同病同治"
   **仍然不能判**，只是阻塞点从 gicp 换成了全局 scan 层（已用盒子试验定位，登记为待修）。
3. §12.4 的**几何推断**（"off 会把倾角加倍成 ~60° ⇒ 只剩 1.19 m 条带"）**只在"初始 `map→odom`
   从未被修正"时成立**。本轮定位器活着：off 档 map 帧的**位置** z 是恒定的
   （`global robot_z = −0.1338`，33 帧 min=max），说明活定位器**没有**保留那个"加倍"。
   但 **map 帧的倾角本轮没有直接量**（`navband` 只量 odom 帧）：odom 帧里 off 仍是 30° 斜面
   （波束 z 跨度 **−0.423 … +2.471 m**），hybrid 是水平面（**+0.013 … +0.059 m**）。
   局部代价图用 `odom`、全局用 `map` —— **这就是"局部有病、全局（看起来）没事"的机制**。
   而 off 档的扫描面在 **odom 与 map 两个帧里都低于带子下界**（odom z p50 **−0.0919**、
   map `robot_z` **−0.1338**；在带内只有 **333/776**）⇒ 即使全局 scan 层修好，
   **off 档大概率仍然是空的**（那是 §9.7 的 `min_obstacle_height` 病，不是本轮的门限病）。

### 14.8 §10.4 式局部 A/B（off vs hybrid，1 跑/档）—— 局部结论仍然成立

两臂就是 §14.7 的两跑（`global_obstacle` 只切**全局**层，局部代价图的
`local_obstacle` 两跑都是默认 `scan`、与它无关 ⇒ 可以直接当局部 A/B 用）：

| 量（`navband`，20 s 窗口中位） | **off** | **hybrid** | §13.8 参考（同机、插件修后、10 s 窗口） |
|---|---|---|---|
| `/scan` 有限波束/帧 | 776 | 775 | 775–776 |
| 波束 z(odom) **p50** | **−0.0919** | **+0.0360** | −0.0899 / +0.0358（校准后） |
| 在带 `[0,2]` 内 / 带下 / 带上 | **333 / 440 / 3** | **775 / 0 / 0** | 333/439/3 vs 775/0/0 |
| 局部 **lethal** | 284 | **339** | 284 / 292 |
| 局部 inscribed / free | 8953 / 44144 | 9564 / 42309 | 8865 / 44314 vs 9575 / 42374 |
| **最近 lethal** | **0.7544 m** | **0.7043 m** | 0.7612 / 0.7070 |
| **前方 2.0×1.2 m lethal** | **17** | **21** | 16 / 30 |
| `map→odom` 活 | 是（100% 采纳） | 是（100% 采纳） | 是 |

⇒ **与 §13.8（校准后）逐项同量级、同方向**：off 仍有 **57% 的波束掉在带下**（`z(odom) = −0.09`），
hybrid **775/775 全在带内**、局部图**非空**（lethal 284–339、最近 0.70–0.75 m、ahead 17–21）。
单跑之间的浮动（hybrid lethal 249/339 vs §13.8 的 292、§10.4 的 288.5）在 §10.4 自己的 3 跑
spread（288–345）之外一点点，**不做定量结论**（n=1）。

### 14.9 §9.10 第 7 条的订正（**old "off 118 / hybrid 2" 是错的**）

§9.10 第 7 条当时写：「`hybrid` 档启动阶段 `planner_server` 有少量 `map→odom` 外推告警
（off 档 118 条 / hybrid 2 条），与高度带无关，**未归因**」。
**订正**：那 118/2 不是"off/hybrid 的性质差"，而是**同一次 gicp 首帧门死锁**的两个采样：
两档都 **`采纳 0/N`**（§10.4 的 6 份 `zf2_*` 日志、§12.4 的 2 份 `exp12_gscan_*` 日志都是这个形状），
`map→odom` 只在启动后的头几秒存在（= 初始值，从未被采纳帧修正），所以外推告警的**条数**只取决于
"nav2 跑了多久 / 什么时候开始丢"，与 off/hybrid 无关。本轮实测（同一配方、同一台机）：
**改前 hybrid 271 条外推 + 42 条 `Message Filter dropping`；改后 3 / 0**。
⇒ 「hybrid 只有 2 条 ⇒ hybrid 更干净」这个读法是**错的**；正确的读法是
「**两档都死**，条数是运行长度的伪指标」（§12.4 已经指出这一点，本节给出复现与修复后的 0 条对照）。

### 14.10 本轮没能验证 / 诚实边界

1. **改前/改后不是单一变量**：两跑之间 `ec5028f` 把 RMUL2026 的 world 地面降了 4.966 cm（§13.6）
   ⇒ 绝对 z / 代价图数字跨跑不可逐字比。门的三个量是相对量；且**改后首帧创新 0.1138 m 仍 > 旧门限
   0.1 m**（不修门照样死锁）这条反证成立。
2. **创新的归因只到"常量错位"这一层**：本轮的 `~/gate` 只有改后档才有**分量**（字段 13/14/15），
   改前档只有模长 ⇒ "0.13 m 里有多少来自地图偏置、多少来自 `initial_pose` 初始化式"**没有拆开**。
   首帧分量（y 0.10–0.14、z −0.22…−0.35）与两者都相容。
3. **`initial_pose` 初始化式 `T_map_odom = T_map_base · T_odom_base` 没有审**（§12.4 第 4 条把它
   登记为"未单独验证"）：首帧创新的 z 分量偏大与它相容，但**本轮没有改它**、也没有单独做实验。
4. **判失效后的自愈探针（`lost_probe_period_sec`）没有端到端触发**：默认 `gate_reanchor_after_rejects=3`
   ⇒ 连击到 25 之前就先重锚了，`lost_after_rejections=25` **走不到**。本轮只验证了**重锚**路径
   （§14.6）；探针路径只有代码级论证 + 触发条件分析（要端到端测它，得先把
   `gate_reanchor_after_rejects` 设 0 —— 那是改 config 的对照实验，本轮没做）。
5. **`global_obstacle:=scan` 的全局层为什么哑**：本轮只做到"用盒子试验**证明**它哑"（§14.7），
   **没有定位到根因**（`enabled` 运行期没回读、全局层的 `update/marking` 路径没抓）。要修它，
   下一步建议：① 运行期回读 `global_costmap/global_costmap obstacle_layer.enabled`
   与 `observation_sources`；② 拿 `/global_costmap/costmap_raw` 在盒子试验里逐格 diff；
   ③ 对照 `stvl` 档（工作）与 `scan` 档的 `updateBounds` 输入。
6. **单跑**：§14.5/§14.6/§14.7 全是 1 跑/档（§14.6 只有 1 次点击、§14.7 盒子只有 1 次插入）。
   噪声带按 §10.4 的 3 跑 spread（lethal ±10、ahead ±3、nearest ±2 mm）参考；
   **"采纳率 100% vs 0%"这种量级的差不需要噪声带**。
7. **只测 `robot:=robot11 robot11_mount:=sensor`**、`world:=RMUL2026`、`lio:=small_point_lio`、
   `localization:=gicp`（backend=pcl）。`small_gicp` 后端、`RMUC2026`、`robot11_mount:=plugin|urdf`、
   `mode:=slam_nav`（map→odom 由 slam_toolbox 出，没有这个门）**都没测**。
8. **门限的参数没重标定**：`gate_trans_rate=3.0 m/s` 在 10 Hz 下给出 0.30 m 的单帧门限
   （= `gate_trans_step_max` 之前就夹住了：3.0×0.1 = 0.3 < 0.6）。本轮只改**dt 的来源**，
   没有动 rate/下限/上限 —— 那是 §3 的标定工作，不是本 bug。
9. **`lost_after_rejections=25` 在 10 Hz 下只有 2.5 s**：本轮实测 25 帧 ≈ 6 s（墙钟，RTF≈0.42），
   判失效发生得比 §9 的预期更快；这个"帧数 @ 帧率"的口径问题**未改**（仍按帧数计）。

### 14.11 一句话给用户

> **`采纳 0/25` 不是"GICP 配得不好"，是门限自己的 `dt` 在首帧只能取到 0**：门限 =
> `clamp(3.0 m/s × dt, 0.10, 0.60)`，`dt` 又定义成"与上次**采纳**帧的戳差" ⇒ 第一帧必 0 ⇒ 门限
> 卡在 **0.10 m** ⇒ 实测创新 **0.11–0.22 m** 全被拒 ⇒ 永远没有"上次采纳" ⇒ **永久自锁**
> （25/25 帧 dt=0，6 s 后判失效、停发 `map→odom`、global costmap 刷 `Message Filter dropping`）。
> 修法四条：`dt` 改用**上一帧收到的**点云戳（第 2 帧起有效）、**无先验首帧免创新门**（score/合理性照旧）、
> **连拒 3 帧就自愈重锚**并放行下一个过 fitness 的帧、**判失效后每秒探针**（3 次通过即恢复）；
> 四个开关默认修好、可逐字节回退。**改后同配方 387/387 = 100% 采纳**、创新 p50 **9.7 mm**、
> `map→odom` 整跑 50 Hz、**`Message Filter dropping` 42 → 0**、外推 271 → 3；把锚点故意弄错
> 1.2 m 时 3 帧被拒后**自动重锚恢复**（99.2% 采纳）。**`global_obstacle:=scan` 的全局半仍然判不了**：
> 这次用"插入一个地图里没有的盒子"证明**全局 scan 层一个格子都没落地**（局部最近 lethal
> 0.71 → 0.33 m，全局 3713/16886/10176 一格不变，而 3713 = 2D 地图的 occupied 格数）——
> **不是 gicp 的锅**（`map→odom` 活着、TF 0 丢弃、off/hybrid 两档都 100% 采纳），是全局层另一个待修 defect。


## §15 第七轮（**本机 date=2026-10-19T23:24–23:59**）：`global_obstacle:=scan` 的全局层为什么"哑" —— **nav2 的 z 闸 × map 帧扫描面 z = −0.179 m**；修法、盒子试验、全局半判决

> **本节只新增**（append-only）：§1–§14 一个字都没改。§14 是上一节（另一个 agent），本节取
> **第一个未占用**的编号 ⇒ §15。
>
> **对象**：`src/rm_nav_bringup/launch/bringup_sim.launch.py` 的 `global_obstacle` 槽位
> （nav2 params 在 `src/rm_navigation/rm_navigation/params/nav2_params_sim_base.yaml`，本节**未改**）
> 里 `scan` 这一档（§14.7 用盒子试验
> **证明**它"全局层一个格子都没落地"，并把它登记为"**另一个** defect，待修"）。本轮把它**定位到根因、
> 修掉、并逐格验收**；同时把 §14 遗留的"**全局半**（off / hybrid）"补测完。
>
> **代码**（两处，都 opt-in）：
> ① `src/rm_nav_bringup/config/nav2_params_sim_global_scan_obstacle.yaml`（**新增**，只含 2 个键）；
> ② `src/rm_nav_bringup/launch/bringup_sim.launch.py` 的 `_Nav2ParamsForSlot` 从"单一 robot11 覆盖层"
> 泛化成"**带命中条件的覆盖层列表**"（`global_obstacle == 'scan'` 时才合并 ①）。
> `src/rm_navigation/**` **一个字节都没改**（仍是只读参照）；`src/rm_localization/**`、`src/rm_simulation/**`、
> 地图资产、§10–§14 也都没动。探针 = `.tmp_ab/exp15/gscan_probe.py`（gitignored，只订阅 + 显式
> `/spawn_entity` / `/delete_entity`）。
>
> **冻结（md5）**：**7 跑**（2 改前 + 5 改后；另有 1 跑因 install/ 没 link 新 config 而起不来，
> 见 15.11 第 9 条）的 `libgicp_registration.so` = **`53c7067d…`**（§14 修好门之后的那个）、
> `libsmall_point_lio.so` = **`58bebfb8…`**（= §10/§13 的 zfix），每跑**跑前=跑后**；
> launch md5：**改前 2 跑 `6d38e74e…`**（diagnosis）、**改后 5 跑 `3ac66e6d…`**（= 本轮 fix 之后），
> 每跑 `跑前=跑后`（唯一变量 = 本轮这两处改动）。原始数据 `.tmp_ab/exp15_*/`（gitignored）。

### 15.1 一句话结论

> **全局层不是没订阅、不是被 disable、不是 TF 丢包、不是量程/滚动窗/边界** —— 它**收到并成功
> 变换**了每一条 `/scan`，然后被 nav2 的**高度闸静默清空**：`ObservationBuffer::bufferCloud`
> （`nav2_costmap_2d/src/observation_buffer.cpp:143-144`）用**每源**的
> `[scan.min_obstacle_height, scan.max_obstacle_height]` 对**搬到代价图帧之后**的点做 z 过滤，
> 而本仓的全局图帧是 `map`、`/scan` 是**二维平盘**（所有点的 z = 扫描面在该帧里的 z）
> ⇒ 实测 **map 帧 z p50 = −0.1786 [−0.237, −0.126]**、**774/774 条全部 < 0.0**（默认下界）
> ⇒ 云里 0 个点 ⇒ `updateBounds` 的 marking / raytrace 两个循环都空转 ⇒ 全局图**恒定等于
> static_layer + inflation_layer**，而且**没有任何日志/告警/dropped 计数**。
> 同一份 `/scan` 在**局部**图（`global_frame: odom`）里 z p50 = **+0.030**（774/774 在带内）⇒ 局部正常。
> 修法：**只给 `global_obstacle:=scan` 这一档**把两个 `min_obstacle_height`（观测缓冲区闸 +
> 图层第二道闸）放到 **−1.0**，上界 2.0 不动。改后：盒子试验里**全局格逐格变化并精确回退**
> （空地上 0→254，删除后 254→0，121/121 格），全局 lethal **3713 → 3859–3910**（**min≠max**），
> **默认 `global_obstacle:=stvl` 路径逐字节不变**（合并出的参数文件 md5 与改动前**逐字节相同**、
> `--show-args` 401 行**逐字相同**、stvl 跑全局 lethal **3727 [3725–3732]** 与 §14 参考同一档）。
> **全局半的答案**：`global_obstacle:=scan` 修好后，**off 与 hybrid 两档全局层都工作**，而且
> **不共享"斜 odom"的病** —— `map→odom` 把 30° 安装/重力对齐**整笔吸收**（off 档 `map→odom`
> 的 q.x = −0.2557 ≈ **29.6° roll**），所以 map 帧里扫描面在**两档都是平的**
> （off 跨度 0.1251 m vs hybrid 0.1166 m；odom 帧里 off 的跨度是 **2.88 m**）。

### 15.2 协议（与 §10.4/§12/§14 逐字相同）+ 本轮新增的唯一动作

**7 跑**（`settle=20/22 s`，窗口 20–22 仿真秒；`gui:=False`、`nav_rviz:=False lio_rviz:=False`、
`spin_speed:=0.0`、`world:=RMUL2026`、`robot:=robot11 robot11_mount:=sensor`、`lio:=small_point_lio`、
`localization:=gicp`）：

```bash
# A. 改前（诊断 + 盒子）：全局走 2D scan，hybrid；盒子 0.45 m 车头前方，插 14.1 s 再删
BOXES="--box 0:0.45" SETTLE=22 WINDOW=56 .tmp_ab/exp15/run_one.sh exp15_diag_r2 \
    world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp global_obstacle:=scan \
    robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid
# B. 改后（盒子试验，两个位姿各插一次）：车头 0.45 m + 车尾 0.60 m
BOXES="--box 0:0.45 --box 180:0.60" SETTLE=22 WINDOW=76 .tmp_ab/exp15/run_one.sh exp15_fix_box_r2 \
    ... global_obstacle:=scan ... gravity_align_mode:=hybrid
# C. 改后 全局半（off / hybrid，各 1 跑；1 跑/档）
NOBOX=1 BOXES="--before-s 22" SETTLE=22 WINDOW=48 .tmp_ab/exp15/run_one.sh exp15_gscan_off_r1 \
    ... global_obstacle:=scan ... gravity_align_mode:=off
NOBOX=1 BOXES="--before-s 22" SETTLE=22 WINDOW=48 .tmp_ab/exp15/run_one.sh exp15_gscan_hyb_r1 \
    ... global_obstacle:=scan ... gravity_align_mode:=hybrid
# D. 改后 默认路径回归（global_obstacle:=stvl）
NOBOX=1 BOXES="--before-s 22" SETTLE=22 WINDOW=48 .tmp_ab/exp15/run_one.sh exp15_stvl_default_r1 \
    ... global_obstacle:=stvl ... gravity_align_mode:=hybrid
```

* **隔离**：`run_one.sh` 与 `spl_ab_bench.sh`/`exp12/run_one.sh` 同口径（`HOME=.tmp_ab/home-<tag>`、
  按 tag 散列 `ROS_DOMAIN_ID`、**先探空端口再占** `GAZEBO_MASTER_URI`、unset DISPLAY、`setsid` 起栈、
  收尾只用 `tools/scripts/lib/scoped_cleanup.sh` 清**本进程组/本 master URI**，**无全机 pkill**）。
* **探针** `gscan_probe.py`（新增，gitignored）：
  ① 运行期 `ros2 param dump` + 逐键 `ros2 param get` 回读**两张图**的参数（**在 base 段才跑** ——
     第一版在进程启动时跑，拿到的是 `Node not found`，见 15.11）；
  ② `/scan` 波束端点用 TF 分别搬到 **odom** 与 **map**，给 z 统计与 `[0,2]` / `[−1,2]` 带内条数；
  ③ 两张 `costmap_raw` 的逐帧计数 + 每段**整张栅格**（`grids.npz`）；
  ④ 决定性动作：`/spawn_entity` 插一个 **25×25×30 cm 静态盒子**（SDF 内联、`static=true`），
     位姿 = `/odom_ground_truth` 的**世界**真值 + `bearing:dist`，测完 `/delete_entity` 删掉；
  ⑤ 盒子在两张图里的位置用 **TF + 真值偏移**换算（不用"新增 lethal 的质心"—— 第一版那样做会被
     墙/噪声的翻动带偏，见 15.11）。
* **口径**："in-band"= 波束端点 z ∈ `[0, 2]`（= 文件里的 `min/max_obstacle_height`）；
  `[−1,2]` 是本轮修法之后的**有效**带。`nearest` / `ahead` 一律用**车在该栅格帧里的位姿**算
  （`map` 帧用 `map→odom ∘ odom→base_link_fake`）—— §14 的 `navband` 探针在这一点上有个**口径 bug**
  （全局图也用 odom 位姿），订正见 15.11。

### 15.3 诊断 A：运行期参数回读 —— **"该有的都有"，唯一的功能性差异是帧**

`global_obstacle:=scan`、运行期真值（`ros2 param get`，`exp15_fix_box_r2`；除 15.7 的两个
`min_obstacle_height` 外，其余键在改前/改后**逐字相同** —— 见 15.7 的合并文件 2 行 diff）：

| 键 | **global_costmap**（哑的那张） | **local_costmap**（正常的那张） | 判决 |
|---|---|---|---|
| `plugins` | `['static_layer','obstacle_layer','stvl_layer','inflation_layer']` | `['obstacle_layer','obstacle_cloud_layer','stvl_layer','inflation_layer']` | 图层**在** plugins 里 ✅ |
| `obstacle_layer.enabled` | **true** | **true** | 不是"没开" ✅ |
| `stvl_layer.enabled` | false（`scan` 档按真值表关掉） | false | 槽位切换**生效** ✅ |
| `obstacle_layer.observation_sources` | `scan` | `scan` | ✅ |
| `obstacle_layer.scan.topic` | **`/scan`** | **`/scan`** | 同源 ✅ |
| `obstacle_layer.scan.data_type` | **`LaserScan`** | **`LaserScan`** | ✅ |
| `obstacle_layer.scan.marking` / `clearing` | **true / true** | **true / true** | ✅ |
| `obstacle_layer.scan.obstacle_max_range` / `raytrace_max_range` | **10.0 / 10.0** | **6.0 / 6.0** | 都不是瓶颈（盒子 0.45 m）✅ |
| `obstacle_layer.scan.obstacle_min_range` | 0.1 | 0.1 | ✅ |
| `obstacle_layer.scan.expected_update_rate` | 0.0 | 0.0 | 不判 stale（不可能是"永久 not current"）✅ |
| `obstacle_layer.scan.min/max_obstacle_height`（**每源**） | **0.0 / 2.0** | **0.0 / 2.0** | ← **同一组数**，但见 15.6 |
| `obstacle_layer.min/max_obstacle_height`（**图层**） | 0.0 / 2.0 | 0.0 / 2.0 | 同上 |
| `global_frame` | **`map`** | **`odom`** | ← **唯一的本质差异** |
| `rolling_window` / `resolution` / `width`×`height` | false / 0.04（被 static 撑成 0.05） / 5×5 | true / 0.02 / 5×5 | 见 15.4 |
| `track_unknown_space` | true | false | 见 15.4（只改"没看见"的取值） |
| `update_frequency` / `publish_frequency` | 5.0 / 2.0 | 20.0 / 10.0 | 见 15.5 |
| `transform_tolerance` | 0.3 | 0.3 | ✅ |
| `robot_base_frame` | `base_link_fake` | `base_link_fake` | ✅ |
| `static_layer.*` / `inflation_layer.inflation_radius` | `map_subscribe_transient_local=true` / 0.65 | —（无 static 层）/ 0.60 | robot11 覆盖层，正常 |

⇒ 任务书列的 8 个"常见嫌疑"（`obstacle_layer` 不在 plugins / `enabled:false` / 不同
`observation_sources` / 错 `topic` / 缺 `data_type: LaserScan` / `min|max_obstacle_height` 与 local
不一致 / marking·clearing 关 / `expected_update_rate` 判 stale）**逐条排除**。
**两张图的 z 带配置完全相同，却一张活一张死** ⇒ 差异只能来自"**同一个数在哪个帧里量**"。

### 15.4 诊断 B：栅格几何 / 滚动窗 / 量程 —— 都不是（而且能算死）

* `StaticLayer` 在 configure 期把全局图 resize 成 **249 × 173 @ 0.05 m/pix = 43077 格**
  （`launch.log` 逐字：`StaticLayer: Resizing costmap to 249 X 173 at 0.050000 m/pix`），
  `RMUL2026.yaml` 的 `origin=[-2.2,-3.15]` ⇒ 全局图**正好覆盖整张 pgm**，车与盒子都远在界内。
* **离线常数对齐**：`RMUL2026.pgm`（249×173，`mode: trinary`、`occupied_thresh 0.65`）里
  **occupied(0) = 3713**、中介灰(205) = 5169（`p=(255−205)/255=0.196 < free_thresh 0.25` ⇒ 算 **free**）、
  free(254) = 34195。于是"只有 static+inflation"时应有：
   lethal = **3713**；inscribed = 16886；free = (34195+5169) − 16886 − 12302(**软带 1..252 格**) = **10176**。
  ⇒ 改前实测**逐值相等**（3713 / 16886 / 10176，**53 帧 min=max**）——这不是"差不多"，是**恒等**。
* `rolling_window: false` ⇒ 没有"窗口把盒子切掉"的可能；`obstacle_max_range: 10.0` ≥ `/scan` 的
  `range_max`，0.45 m 的盒子在量程正中。

### 15.5 诊断 C：TF / 时序 —— 也不是（而且证据是"没有证据"）

* 改前那一跑（`exp15_diag_r2`）全局节点 **`Message Filter dropping` = 0 条**、
  planner `Extrapolation Error` = **0 条**（`grep -ac` 全 0），`gcp` 采纳 **100%**、`判失效` 0 条。
* 全局图 `costmap_raw` 在 20 s 窗口里到了 **53 帧**（≈2.6 Hz，`publish_frequency: 2.0`），
  `update_frequency: 5.0`，`header.stamp` 单调推进 ⇒ 图层**在跑**（只是没有东西可标）。
* `launch.log` configure 段逐字：`Using plugin "obstacle_layer"` → `Subscribed to Topics: scan` →
  `Initialized plugin "obstacle_layer"` ⇒ 订阅**建起来了**。
* **关键反证**：`enabled/观测源/TF` 任一项坏掉，改一个 `min_obstacle_height` 是不可能把它救活的；
  而本轮**只**改这两个数就活了（15.8）。⇒ 病在"进云之后、标格之前"那一段。

### 15.6 诊断 D（决定性）：**扫描面在 map 帧里的 z = −0.179 m**，默认下界 0.0 把它整盘切掉

同一帧 `/scan` 的波束端点（scan 帧里 z=0 的平盘）用 TF 搬到两个代价图帧后的 z 统计
（`exp15_diag_r2`，321 帧中位；"带内/=下/=上"= `[0,2]`）：

| 目标帧 | z p50 (m) | z ∈ [min, max] (m) | **在 [0,2] 带内** | 带下 | 带上 |
|---|---|---|---|---|---|
| **odom**（局部图帧） | **+0.0354** | [+0.0150, +0.0648] | **774 / 774** | 0 | 0 |
| **map**（全局图帧） | **−0.1786** | [−0.2370, −0.1259] | **0 / 774** | **774** | 0 |

**源码链（三处，每一处都静默）**：

1. `ObstacleLayer::laserScanCallback` → `projector_.transformLaserScanToPointCloud(...)`
   （z 在 scan 帧里恒为 0）→ `buffer->bufferCloud(cloud)`；
2. `ObservationBuffer::bufferCloud`（`nav2_costmap_2d/src/observation_buffer.cpp:143-144`）：
   ```cpp
   if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) { copy; ++point_count; }
   ```
   —— **`iter_z` 是"已搬到 `global_frame_`"的点的 z**，两个闸值来自**每源**的
   `scan.min_obstacle_height / scan.max_obstacle_height`。**不匹配就丢，`modifier.resize(point_count)`
   之后云里 0 点，无日志、无 WARN、无 dropped 计数。**
3. `ObstacleLayer::updateBounds`（`plugins/obstacle_layer.cpp:466-511`）对云里**剩下的**点做
   `pz < min_obstacle_height_`（**图层**级第二道闸，`:470`）与射程过滤，然后 `costmap_[index] = LETHAL`。
   云空 ⇒ marking/clearing 都空转（`raytraceFreespace` 也吃同一个 buffer）。

**为什么 map 帧的 z 是 −0.179**：实测 `map→odom` 的平移 z = **−0.2154**、`odom→base_link` 的
z = **+0.0300**（同一跑）⇒ `map←base_link` 的 z = **−0.1854**，与直接量的 −0.1786 吻合（差 = 车体
俯仰的投影项）。**两档一致**：off 档 −0.0474 + (−0.1323) = **−0.1797**、hybrid −0.2098 + (+0.0302)
= **−0.1796** ⇒ **map 帧恒定在扫描面上方 ~0.18 m**，与重力档无关（这也正好解释了 §14.7 里
off/hybrid 两档的全局 lethal **都**恰好等于 3713）。
⇒ 只要下界是 0.0，**这条槽位在任何重力档下都不可能标一个格子**。

### 15.7 修法（两处改动、opt-in、逐字节可回退）

**① 新文件 `src/rm_nav_bringup/config/nav2_params_sim_global_scan_obstacle.yaml`**（只含 2 个键）：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        min_obstacle_height: -1.0        # ② 图层级第二道闸（obstacle_layer.cpp:470）
        scan:
          min_obstacle_height: -1.0      # ① 观测缓冲区闸（observation_buffer.cpp:143）
```

**② `src/rm_nav_bringup/launch/bringup_sim.launch.py`**：
* `_Nav2ParamsForSlot`（**`:997` 起**）从"`base + 单一 robot11 覆盖层`"改成
  "`base + 带命中条件的覆盖层列表`"；不命中任何覆盖层 ⇒ **原样返回 base 路径**（不写临时文件）。
* 调用点（**`:1795` 起**）传两层：
  `('robot11', …/nav2_params_sim_robot11_costmap.yaml, ctx.robot=='robot11')`、
  `('global_scan', …/nav2_params_sim_global_scan_obstacle.yaml, ctx.global_obstacle=='scan')`。
* 临时文件命名 = `nav2_params_sim_<命中的 tag 用 _ 连>_merged.yaml` ⇒ robot11 单独命中时
  **仍是** `nav2_params_sim_robot11_merged.yaml`（与改动前同名）。

**为什么必须改两处**：`ObservationBuffer` 的闸（每源键）先丢点，`updateBounds` 的闸（图层键）对
"活下来的点"再丢一次；只改任一处，points 仍然到不了 `costmap_[index] = LETHAL`。

**为什么是 −1.0**：map 帧的扫描面 z 是**定位器输出**（`map→odom` 的 z 随初始位姿/地图偏置变，
本轮实测 −0.18，量级 ±0.2 m），不是几何常数 ⇒ 下界放到 −1.0 给 ≥0.8 m 余量，同时对 2D `/scan`
**不引入任何新语义**：LaserScan 没有高度信息，**所有点的 z 都等于"扫描面在该帧里的 z"**，
放宽下界只可能让"整条平盘"进带，不可能把地面点混进来（与本仓
`nav2_params_sim_robot11_costmap.yaml` 文件头 §K.1 的论证同口径）。上界 2.0 **不动**。

**为什么 opt-in**：命中条件 = `global_obstacle == 'scan'`。默认 `stvl`/`none` 不命中 ⇒
`_Nav2ParamsForSlot` 返回 base（或只合并 robot11 那层）⇒ 参数文件**逐字节不变**：

| 证据 | 结果 |
|---|---|
| 默认档（`robot11`+`stvl`）**合并出的 YAML md5**：新代码 vs **改动前实现的逐字副本** | **`e9b39f36…` == `e9b39f36…`（逐字节相同，9526 B）** |
| 非 robot11 + `stvl` | 返回 base 路径本身（不写临时文件）|
| `stvl` 档 vs `scan` 档 合并文件 diff | **只有 2 行**（`+min_obstacle_height: -1.0` ×2）|
| `ros2 launch <file> --show-args`（改动前 HEAD 文件 vs 改动后） | **401 行逐字相同** |
| 新增/删除的 launch 参数、节点、话题 | **0 个**（改动只有 `_Nav2ParamsForSlot` 内部 + 调用点 + 一个新 yaml）|
| 默认档实跑（15.10） | 全局 lethal **3727 [3725–3732]**、`scan.min_obstacle_height` 回读 **0.0** |

### 15.8 决定性试验：盒子（25×25×30 cm，地图里没有）

**改前**（`exp15_diag_r2`，hybrid，盒子插在**车头 0.45 m**，驻留 14.1 s，然后删除）——
**局部立刻看见、全局逐格不动**：

| 量 | 盒子前 | 盒子在（14.1 s） | 删除后 |
|---|---|---|---|
| 局部 窗 ±0.25 m @ odom(0.42, 0.047)：lethal / inscribed / free | 0 / 291 / 15 | **5 / 709 / 0** | 0 / 291 / 15（**精确回退**）|
| 局部 窗内变化格数 | — | **729 / 729（全窗）** | 729 |
| **全局** 窗 ±0.25 m @ map(0.364, −0.022)：lethal / inscribed | 41 / 91 | **41 / 91** | 41 / 91 |
| **全局** 窗 ±0.50 m：lethal / inscribed | 75 / 269 | **75 / 269** | 75 / 269 |
| **全局整张图变化格数** | — | **0** | **0** |
| 全局 lethal / inscribed / free（逐帧） | 3713 / 16886 / 10176 | **3713 / 16886 / 10176（53 帧 min=max）** | 同 |

**改后**（`exp15_fix_box_r2`，hybrid，**两个位姿各插一次**：① 车头 0.45 m 驻留 8.1 s；
② 车尾 0.60 m 驻留 3.7 s —— 车尾那一处**两张图都是空地**，所以是最干净的判据）：

| 量 | base（无盒） | ① 车头 0.45 m | ① 删除后 | ② 车尾 0.60 m | ② 删除后 |
|---|---|---|---|---|---|
| **局部** 最近 lethal (m) | 0.7125 | **0.3197** | 0.7165 | 0.4804 | 0.7125 |
| **全局** 最近 lethal (m) | 0.5503 | **0.4003** | 0.5503 | 0.4499 | 0.5503 |
| **全局** 整张图 lethal / inscribed / free | 3956 / 17083 / 9664 | 3868 / 17381 / 9645 | 3896 / 17292 / 9556 | 3885 / 17274 / 9266 | 3881 / 17294 / 9658 |
| **全局** 窗 ±0.25 m lethal / inscribed / free | 37 / 73 / 0 | **40 / 81 / 0** | 37 / 73 / 0 | **0 / 0 / 121 → 4 / 87 / 0** | **0 / 0 / 121** |
| **全局** 窗内变化格数（相对上一段） | — | **14**（±0.5 m：105） | **14（回退）** | **121 / 121（全窗，0→254）** | **121（回退，254→0）** |
| **局部** 窗 ±0.25 m lethal（② 位） | 0 | — | — | **0 → 10** | **10 → 0（729/729 格回退）** |

读法：① 位在**地图的膨胀带里**（static+inflation 那格本来就是 253/254），所以全局只能看到
`253→254` 与膨胀环的**小台阶**（±0.25 m 14 格、±0.5 m 105 格）—— 但**它就是变了**，而且删除后
**逐格回退**；② 位在**两张图的空地**上，于是得到最干净的一组：全局 `0→254`（4 个 lethal 格 +
87 个 inscribed + 121 格里 free 121→0），删除后**逐格回到 0**。**改前 ① 位是 0 格变化，改后
同一位姿 14 格变化** ⇒ 单一变量（那两个 `min_obstacle_height`）就是全部原因。

**整张图的 liveness（改后）**：`exp15_gscan_hyb_r1` 的 36 帧全局 lethal **3851–3878（min≠max）**、
inscribed 17020–17407、free 9546–9942；`off` 档 37 帧 **3888–3919** / 17006–17417 / 9470–9925。
改前那跑是 **3713/16886/10176 逐帧恒定（53 帧 min=max）**。

### 15.9 全局半判决：`mode:=nav localization:=gicp global_obstacle:=scan` × {off, hybrid}（改后，1 跑/档）

> 两档的 gicp 都活着（`采纳 782/782`、`判失效 0`；`map→odom` 整跑；hybrid 档
> `Message Filter dropping` = 0、`Extrapolation Error` = 0）。

| run | 局部 lethal / inscribed / free | 局部 最近 · ahead | 局部 in-band（**odom** `[0,2]`） | **全局** lethal / inscribed / free / unknown（43077 格） | 全局 最近 · ahead | **全局 in-band（map `[0,2]` / `[−1,2]`）** | 全局逐帧 lethal min–max | map 帧 z p50 [min, max] |
|---|---|---|---|---|---|---|---|---|
| `exp15_gscan_off_r1`（off） | 293 / 8855 / 44318 | 0.7625 · 12 | **329 / 774**（带下 438、带上 3） | **3888 / 17006 / 9925 / 0** | 0.5504 · 90 | **0 / 774** → **774 / 774** | **3888–3919（min≠max）** | −0.1761 [−0.2442, −0.1191] |
| `exp15_gscan_off_r2`（off，**不做参数回读** ⇒ 干净 TF） | 284 / 8996 / 44129 | 0.7538 · 12 | **333 / 774**（带下 439、带上 3） | **3855.5 / 17179.5 / 9705 / 0** | 0.4975 · 93 | **0 / 775** → **775 / 775** | **3848–3868（min≠max）** | −0.1787 [−0.2434, −0.1261] |
| `exp15_gscan_hyb_r1`（hybrid） | 345 / 9461 / 42449 | 0.7046 · 22 | **776 / 776** | **3859 / 17194.5 / 9708.5 / 0** | 0.5001 · 91 | **0 / 776** → **776 / 776** | **3851–3878（min≠max）** | −0.1787 [−0.2428, −0.1262] |
| （改前对照）`exp15_diag_r2`（hybrid） | 303 / 9528 / 42351.5（`navband`，280 帧）| 0.7193 · 20 | 774 / 774 | **3713 / 16886 / 10176 / 0** | 0.4508† · 82†（†= 口径 bug，见 15.11）| 0 / 774 → **0 / 774** | **3713–3713（min=max，53 帧）** | −0.1786 [−0.2370, −0.1259] |

**判决**：
1. **全局层活了，两档都活**：全局 lethal 从"恒等于 static 的 3713"变成 **3848–3919**（比 static 多
   **+135…+206** 格 = 2D scan 真正标进去的墙/障碍），`nearest`/`ahead` 也随之变化；
   **in-band 从 0/774 变成 774/774**（`[−1,2]` 带）。off 两跑（3888 带探针扰动 / 3855.5 干净）
   给出本轮唯一的 n=2 spread：**lethal ±16、nearest ±0.026 m、ahead ±2**（见 15.11 第 1 条）。
2. **全局半不共享"斜 odom"的病** —— 这是本轮最反直觉的一条：**off 档的 30° 斜面在 `map` 帧里
   *消失***。证据：odom 帧里 off 的波束 z 跨度 **−0.417…+2.458 m（2.88 m）**、只有 **329/774** 在
   `[0,2]` 带内；同一时刻 map 帧里 **−0.244…−0.119（0.125 m）**、**774/774** 都在 `[−1,2]` 带内。
   机理在 TF 里看得见：off 档 **`map→odom` 的四元数 q = [−0.2557, 0.0075, −0.0009, 0.9667]**
   ⇒ roll = 2·asin(0.2557) ≈ **29.6°**，正是把 `odom→base_link` 的 30° 安装倾角**反向抵消**掉的那一笔。
   ⇒ **局部图（odom）有病、全局图（map）没病**，两档的全局半都"整盘可见"。
   （§12.4 第 4 条那条"off 会把倾角**加倍**成 ~60°、只剩 1.19 m 条带"的**推断**在活定位器下
   **不成立**：那条推断的前提是"`map→odom` 从未被修正且等于初始值"，而本轮 `map→odom` 一直是活的；
   §14.7 第 3 条已经指出 off 档 map 帧的位置 z 恒定，本轮把**姿态**也量了：不是加倍，是**抵消**。）
3. **`off` 档全局的"看得见"不等于"和 hybrid 一样"**：全局 lethal 3888(off) vs 3859(hybrid)、
   inscribed 17006 vs 17194.5、ahead 90 vs 91、nearest 0.5504 vs 0.5001 —— 同量级、同向，
   但 **n=1/档**，不做定量比较（噪声带参考 §10.4：lethal ±10、ahead ±3）。
4. **局部半的判决与 §14.8 逐项重合**：off 局部 in-band **329/774**（带下 438）vs hybrid **776/776**；
   off lethal 293 / nearest 0.7625 / ahead 12 vs hybrid 345 / 0.7046 / 22 —— 与 §14.8 的
   333/776、284、0.7544、17 与 775/775、339、0.7043、21 同档（差在两个 `/scan` 有限波束档位
   774 vs 776 与逐跑浮动）。

### 15.10 默认路径回归（`global_obstacle:=stvl`，hybrid，1 跑）

| 量 | 本轮 `exp15_stvl_default_r1`（改动后） | §14 的 stvl 参考（`zt_gate_after_r1` / `selfheal`） | 判决 |
|---|---|---|---|
| 全局 lethal（逐帧 min–max） | **3727（3725–3732）** | 3725–3728 | **同档** ✅ |
| 全局 inscribed | 16872（16867–16874） | 16871–16875 | **同档** ✅ |
| 全局 free / unknown | 10176 / 0 | 10176 / 0 | 逐值同 ✅ |
| 全局 `min≠max` | **是**（stvl 在改图） | 是 | ✅ |
| 运行期 `obstacle_layer.enabled` / `stvl_layer.enabled` | **false / true** | false / true | ✅ |
| 运行期 `obstacle_layer.scan.min_obstacle_height` / `obstacle_layer.min_obstacle_height` | **0.0 / 0.0**（覆盖层**没生效**） | 0.0 / 0.0 | ✅ |
| 局部 lethal / 最近 · ahead | 411 / 0.7074 · 38 | 249–292 / 0.705–0.712 · 21–30 | 同量级（stvl 档的局部源与 scan 档不同，见 §14.8 注）|
| `Message Filter dropping` / `Extrapolation Error` | **0 / 0** | 0 / 3 | ✅ |
| 合并参数文件 md5 / `--show-args` | `e9b39f36…`（= 改动前逐字节）/ 401 行逐字相同 | — | **逐字节不变** ✅ |

⇒ **默认路径的"参数文件 + 参数面 + 行为数字"三层都没变**：新覆盖层在该档**不命中**，
`global_obstacle:=stvl` 的全局图仍是 STVL 撑着的 3725–3732（§14 的同一档）。

### 15.11 本轮没能验证 / 诚实边界

1. **参数回读本身会扰动栈（已量化、已规避）**：探针第一版在"进程一启动"就跑
   `ros2 param dump`（那时 nav2 还没起来 ⇒ 拿到 `Node not found`），第二版挪到 **base 段**（nav2 已活）
   —— 后者在 **3 跑里 2 跑**触发了 `Message Filter dropping`（`off_r1` 42 条 / `fix_box_r2` 26 条，
   两本图都有）与 planner `Extrapolation Error`（276 / 144 条）：**全部指向同一条"卡住的旧戳"**
   （例如 `at time 669.182`，一直重试到跑尾）。**不做回读的 2 跑都是 0 / 0**（改前的 `exp15_diag_r2`
   ——它的回读在 nav2 起来前就失败了、几乎零服务流量；改后的 `exp15_gscan_off_r2` 加了
   `--no-params`）。⇒ 判为**探针副作用**（约 40 次 `ros2 param get` 子进程 + 1 次 dump 的服务/发现
   风暴把 MessageFilter 的队列拖过了 TF 缓存窗口），**不是** fix 的后果。**但相关性不完美**：
   同样做了回读的 `exp15_gscan_hyb_r1` 与 `exp15_stvl_default_r1` 都是 **0 / 0** ⇒ "回读"是必要
   不充分条件，**这条残留没有逐条归因**。为拿到"干净 TF"的全局半数，`exp15_gscan_off_r2`
   用 `--no-params` 重跑（15.9 第 2 行）——**两跑同配置、同 md5，全局 lethal 3888 vs 3855.5、
   nearest 0.5504 vs 0.4975 的差就是这层扰动的量级**（也是本轮 n=2 的唯一 spread 数据）。
2. **`map` 帧为什么比扫描面高 0.179 m，没有归因到具体代码行**：本测量到的是
   `map→odom`(z) + `odom→base_link`(z) = −0.2154 + 0.0300 = **−0.1854**（与直接量的 −0.1786 差 7 mm
   = 车体俯仰投影）。**没有**去审 `gicp_registration.cpp` 的 `initial_pose` 初始化式
   （§12.4 第 4 条 / §14.10 第 3 条把它登记为"未审"）。**修法不依赖这个数的来源**：只要它落在
   (−1.0, 2.0) 里就行。
3. **`−1.0` 是余量，不是标定**：没有任何实验去扫"下界取多少最优"。它的安全性来自
   "LaserScan 是平盘、z 不带高度信息"这一语义论证（15.7），不是来自实测的调参。
4. **盒子试验的剂量**：改前 ① 位驻留 14.1 s；改后 ① 位 8.1 s、② 位 3.7 s（探针相位计时器
   第一版有个 bug：切相位时没重置 `phase_t`，导致驻留被截短；栅格证据不受影响，因为
   "全局整张图 0 格变化"在改前是**53 帧**的结论）。② 车尾 3.7 s 只覆盖 **6 帧**全局图。
5. **§14 的 `navband` 探针在全局图上有个口径 bug（本节订正）**：它用 `odom→base_link_fake`
   的位姿去索引**全局（map 帧）**栅格（`nav_band_probe.py:188` 的 `_grid_stats` 固定查
   `odom`）。因此 §14.7 的"全局 最近 lethal 0.4573 / 0.4499 / 0.4526"与"ahead 81/82"是
   **用错帧的位姿**算的；本节同一跑用**map 帧位姿**重算得到 **0.5504 / 0.5001 / 0.5559** 与
   **90 / 91 / 82**（ahead 巧合地接近，nearest 差 ~0.1 m = 车在 map 与 odom 里的位置差）。
   **§14 的数不被推翻**（那里只用来判"全局没变"，与本 bug 无关），但**跨节比 nearest 时要用本节口径**。
6. **n 很小**：15.9 的 off 两跑（其中一跑用于规避探针副作用的复算）、hybrid 一跑；
   15.8 的盒子是"1 次插入/位姿"；15.10 的 stvl 一跑。**"0 格 vs 14 格/121 格"这种量级的差
   不需要噪声带**，但 lethal 的 ±10 级差不要拿来判优劣。
7. **只测 `robot:=robot11 robot11_mount:=sensor`、`world:=RMUL2026`、`lio:=small_point_lio`、
   `localization:=gicp`**。没测：`amcl`/`small_gicp` 后端、`mode:=slam_nav`（那张 map 的 z 基准
   不同 —— §12.6 第 2 条说 slam_toolbox 把 map 锚在车起始高度，scan 面会压在带子**下边界**，
   修好后大概率也能标，但**未实测**）、`robot11_mount:=plugin|urdf`、`RMUC2026`。
8. **`global_obstacle:=none`** 没回归（它不命中新覆盖层，按 15.7 的合并逻辑与改动前等价，
   但**没有**单独跑一跑去证明）。

### 15.12 一句话给用户

> **全局层"哑"不是没开、不是 TF 丢包，是 nav2 的**高度闸**在**错的帧**里量了一下**：
> `ObservationBuffer` 把 `/scan`（二维平盘）搬到**代价图帧**后用 `[scan.min_obstacle_height,
> scan.max_obstacle_height] = [0.0, 2.0]` 过滤 z —— 而全局图的帧是 `map`，本仓 map 帧在扫描面
> **上方 0.179 m**（实测 `map→odom` z −0.2154 + `odom→base_link` z +0.0300 = −0.1854）⇒
> **774/774 条波束全部 < 0.0** ⇒ 云被**静默**清空（无日志/无告警/无 dropped 计数）⇒ 全局图
> 恒等于 static+inflation（3713/16886/10176，53 帧 min=max），而局部图用 `odom`（扫描面 z=+0.030）
> 一切正常。**修法两行 yaml**：给 `global_obstacle:=scan` 这一档加一份覆盖层，把
> **观测缓冲区闸 + 图层闸**两个 `min_obstacle_height` 都放到 **−1.0**（上界 2.0 不动）——
> 对 2D 平盘这只是"让整条盘进带"，不引入任何高度语义。**改后盒子试验逐格过**：
> 空地上全局 `0→254`（121/121 格 0→253/254）、删除后 **254→0 精确回退**，车头 0.45 m 那处
> 从"0 格变化"变成"14 格变化"；全局 lethal **3713 → 3851–3919（min≠max）**；
> **默认 `stvl` 路径逐字节不变**（合并参数文件 md5 相同、`--show-args` 401 行相同、
> stvl 跑 3727[3725–3732] 与 §14 同档）。**全局半也有答案了**：修好后 off/hybrid **两档全局都活**，
> 而且**不共享斜 odom 的病** —— off 档 `map→odom` 带 **29.6° roll**，把 30° 安装倾角**反向抵消**，
> 所以 map 帧里扫描面两档都平（跨度 0.125 vs 0.113 m）；"斜 odom ⇒ 半平面丢失"只发生在
> **局部（odom）图**上（off in-band 329/774、hybrid 776/776）。

9. **一次"起不来"的记录（可复现性）**：`exp15_fix_box_r1` 的 launch 直接报
   `[launch] global_scan 的 nav2 覆盖层不在：…/install/rm_nav_bringup/share/rm_nav_bringup/config/
   nav2_params_sim_global_scan_obstacle.yaml` 然后整栈退出 —— 本仓是 `--symlink-install`，但
   **新增文件**不会自动进 `install/` 的 config 目录。`colcon build --symlink-install
   --packages-select rm_nav_bringup`（0.16 s）之后 symlink 出现，重跑即通。
   ⇒ 换机器/换 clone 复现本节时，**先 build 再跑**。

---

## §16 第八轮（**本机 date=2026-10-19T22:50 – 10-20T00:20**）：按修好的 Livox 插件**重建地图资产**（RMUC2026 + RMUL2026）

> **本节只新增**（append-only）：§1–§15 一个字都没改。§13.9 最后一条留的
> 「既有 PCD / 2D 地图（RMUL/RMUC，LIO 系）… 本轮没有重建地图/PCD ⇒ 要不要重建由用户定」
> —— 用户批准后就是本节。（§14/§15 已被另外两个 agent 占用 ⇒ 本节用 §16。）
>
> **为什么必须重建、而不是把旧图整体平移 0.19 m**：插件偏置 `R_joint·offset.Pos()` 是 **base_link 系里的常量**，
> 建图时机器人一直在转 ⇒ 落进 `map` 系后它随**观测朝向**旋转（`R_yaw·(0.0006, 0.1919, 0.0705)`）⇒
> 同一个墙角从不同朝向看落到不同位置，旧图是**被抹开/畸变**的，不是整体平移。
>
> 隔离：私有 `HOME` / `ROS_DOMAIN_ID=133` / `GAZEBO_MASTER_URI=http://127.0.0.1:11733`、`gui:=False`、
> `world:=X` 不带前导 `--`、收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）、
> `map_autocontinue:=False`（每次从零建）。原始数据与可复跑脚本在 `.tmp_maprebuild/`
> （gitignored：`run_mapping.sh` / `run_stationary.sh` / `probe_wall.sh` / `map_wall_probe.py` /
> `map_world_align.py` / `drive_probe.sh`；每个 run 目录里有 `meta.txt`（date/git_head/world_md5/
> launch_md5/livox_so_md5/domain/master）、`launch.log`、`align.json`、`drive.json`）。

### 16.1 一句话结论

> **工具的修复现场复核过了**（网格墙面残差 **−0.1919 → 0.0000 m**，5 个面全 0，可见地面 world z
> **−0.0015 m**）；两界的**新资产都建出来了**（新名字 + 完整 sidecar，旧资产一个字节没删，
> 全部留 `*.bak-20261019`）。**几何正确性拿到了判决性的干净证据**：RMUL2026 新 PCD 的墙面残差
> **1.5–3.1 mm**（旧 PCD 是 **38–49 mm**），而且这是**单视点、无 LIO 漂移**的一次干净建图 ⇒
> "按修好的插件建出来的图，墙就在网格上"这件事成立了。
> **GICP 侧也拿到了改善**：同一个静止口径，
> 新 `RMUC2026_v4.pcd` 的创新 **p50 0.0094 → 0.0043 m**、**p95 0.0246 → 0.0109 m**、
> `score` p50 **0.0139 → 0.0027 m²（5.1 倍）**，采纳两臂都是 **100%**（门限 0.30 m 平移 / 6° yaw）。
> **但本轮没有把新资产提升成默认先验**，原因不是几何，是**本机仿真底盘现在驱不动**：
> 实测纯前进 0.30 m/s 指令 → 实际 0.183 m/s（可用），**纯原地转 0.80 rad/s 指令 → 实际 0.025 rad/s（3%）**
> ⇒ 任何"靠原地转对准航点"的覆盖跑图都会卡死在第一个转弯；本轮 RMUC2026 的**驱动版**跑图 26 min 只走
> 了 10.6 m 真值位移（LIO 还因为原地抖转把路径积分成 142.8 m、ATE RMSE 0.31 m）。⇒ 覆盖度不够，
> 2D 先验的窗口从旧图的 `28.85 × 16.30 m` 掉到 `15.60 × 15.00 m`（occ 9769 → 3135）。
> 另外**撞到两条独立的新缺陷**（§16.9），其中一条（`small_point_lio` + 默认模型槽位 SIGSEGV）与本任务无关但会挡任何人建图。

### 16.2 资产清单 + 备份（`*.bak-20261019`，逐字节相同）

| world | 槽位 | 文件（原默认） | 内容 | 备份名 | 备份 md5 |
|---|---|---|---|---|---|
| RMUC2026 | 2D 先验 | `map/RMUC2026.pgm` + `.yaml` | `577x326 @0.05`，`origin [-25.2,-9.09]`，occ 9769 / unknown 32135 | `*.bak-20261019` | `82e1742b54e2cf3e33ac8891ca90f7f5` / `683b07a83120957aad1cbbd69d76bf43` |
| RMUC2026 | 3D 先验 | `PCD/RMUC2026.pcd` | 203,352 点 / 6.51 MB | `*.bak-20261019` | `5b494a6016a6deb3276b2a6fb48ed1d1` |
| RMUL2026 | 2D 先验 | `map/RMUL2026.pgm` + `.yaml` | `249x173 @0.05`，`origin [-2.2,-3.15]`，occ 3713 / unknown 5169 | `*.bak-20261019` | `6e45a9327e99d57883881f7a7d10c3d0` / `fd23dd82c0aa63b6c2fdf3644c7c9253` |
| RMUL2026 | 3D 先验 | `PCD/RMUL2026.pcd` | 53,164 点 / 1.70 MB | `*.bak-20261019` | `5d0d33eb5219d9416606a7b0f5e34028` |

* 命名沿用仓库既有两层回滚（`docs/map_assets.md` §1）：人工提升备份 `*.bak-<YYYYMMDD>`、
  工具自动代 `*.prev-<YYYYMMDD-HHMMSS>`。本节只产生前者（**新资产一律写在新名下**，没有就地覆盖默认）。
* 同目录其它资产（`RMUC2026_v2/v3/good/cont/spl/cloud/mapped`、`RMUL2026_world_backup/pbstream`、
  `RMUC*`/`RMUL*` 历史件、`attic/`）**一个都没动**；逐个清单仍在 `docs/map_assets.md` §2。
* 回滚（本节默认**没有**被覆盖，所以这条只是给"以后提升"用的）：
  ```bash
  cd src/rm_nav_bringup && D=20261019
  cp -f map/RMUC2026.pgm.bak-$D map/RMUC2026.pgm && cp -f map/RMUC2026.yaml.bak-$D map/RMUC2026.yaml
  cp -f PCD/RMUC2026.pcd.bak-$D PCD/RMUC2026.pcd        # RMUL2026 同理换名字
  ```

### 16.3 工具链复核（先确认"真的修好了"，同一把尺子 = §13.4 的 `wall_witness.py`）

`install/ros2_livox_simulation/lib/libros2_livox.so` md5 = **`7bbab8752337204f8cdf4ac6e3658286`**
（与 `build/` 那份逐字节相同）；源码侧 `livox_points_plugin.cpp:363-366` 的
`cloud_origin = cloud_frame_sensor_ ? Zero : sensor_offset.Pos()` 在位。
跑法：`mode:=mapping lio:=small_point_lio robot:=robot11 robot11_mount:=sensor world:=RMUL2026`（静止，
`.tmp_maprebuild/v_wall_rmul/`）。

| 网格竖直面（world） | §13.4 修前 | 本次实测 |
|---|---|---|
| x = 2.5（跨度 5.62 m） | +0.0006 m | **+0.0000 m** |
| **y = −8.5（跨度 6.50 m）** | **−0.1919 m** | **+0.0000 m** |
| x = −5.0（跨度 4.69 m） | — | **−0.0000 m** |
| x = −6.85（跨度 2.08 m） | — | **−0.0000 m** |
| x = −10.0（跨度 2.61 m） | — | **−0.0000 m** |
| x = 2.22 / y = −8.78（相邻面进窗的假读数，§13.10#5） | +0.28 m | +0.28 m（同值 ⇒ 与本 bug 无关） |

可见地面（world 系 SVD 平面，`floor_probe.py`）：`[0.8,8.0] m` 带 **−0.0015 m**；分带
+0.0026 / +0.0078 / +0.0010 / −0.0220 m（= §13.3 的 mesh 台阶，不是校准误差）。
`GT base_link z = 0.10250`、`livox 原点 world z = 0.25957`（= 0.10250 + 0.15703）⇒ 雷达侧与接触面自洽。

### 16.4 建图 recipe（**实际用的**）+ 四处实测发现的坑

```bash
# A. 驱动版（RMUC2026_v4）：默认模型槽位 + fastlio + 覆盖跑图
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 mode:=mapping \
  lio:=fastlio mapper:=slam_toolbox ground:=linefit map_name:=RMUC2026_v4 \
  map_autocontinue:=False cloud_accumulator:=True spin_speed:=0.0 \
  nav_rviz:=False lio_rviz:=False gui:=False
#   路线：coverage_route.py --order lawnmower --lane-spacing 8.0 --goal-step 5.0 --clearance 0.45
#   跑图：coverage_drive.py --pose-source lio --stuck-sec 30 --yaw-rate-max 2.0 --save-2d <out>
#   落盘：map_saver_cli（2D）+ map_archive.sh save --name RMUC2026_v4（posegraph/PCD/sidecar）
# B. 静置版（RMUC2026_v4b / RMUL2026_v4b）：robot11 + sensor + hybrid + 静止 150 s 累积
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=<W> mode:=mapping \
  lio:=small_point_lio mapper:=slam_toolbox ground:=linefit robot:=robot11 \
  robot11_mount:=sensor gravity_align_mode:=hybrid map_name:=<W>_v4b \
  map_autocontinue:=False cloud_accumulator:=True nav_rviz:=False lio_rviz:=False gui:=False
```

**坑 1：`robot:=robot11` 的底盘"能平移、几乎不能自转"**（`.tmp_maprebuild/dp_rmuc/`，各 10 s 纯指令）：

| 指令 | 实际 | 比例 |
|---|---|---|
| 纯前进 `v=0.30 m/s` | **0.183 m/s** | 61%（可用） |
| 纯原地转 `w=0.80 rad/s` | **0.025 rad/s** | **3%** |

`libgazebo_ros_planar_move` 是"每步设模型速度"，与轮子-地面接触摩擦对抗 ⇒ **自转被摩擦吃掉**。
⇒ 覆盖跑图的每一次"原地转对准航点"都变成 30 s 级等待，`coverage_drive` 的 6 s 卡死判据还会把它
误判成卡死并**倒车**（`--stuck-sec 6` → 已改 30）。这就是本轮 RMUC 驱动版只走了 10.6 m 真值位移的原因。

**坑 2：`lio:=small_point_lio` + **默认模型槽位**（`R_bl=I` 档）SIGSEGV**（exit −11，节点起来 ~12 s 后
`process has died`，可复现；`robot11+sensor` 档不崩）⇒ 驱动版改用 `lio:=fastlio`。见 §16.9#2。

**坑 3：`robot11_mount:=sensor` + **`small_point_lio`** 的 `map` 系是**斜的**：实测 `R_world←map`
roll = **−30.05 … −30.15°**（odom 原点 = 初始化时刻的**传感器系**，而该档传感器物理斜装 30°）。
slam_toolbox 的 2D 栅格吃不下这个倾角（会变成 y 向 cos30° 压缩的斜投影）⇒ **这一档不能用来建 2D 栅格**。
**`lio:=fastlio` 就没有这个问题**（IMU 重力对齐 ⇒ `R_world←map` 实测 roll −0.06…−0.12°、pitch +0.19…+0.22°，
水平）⇒ 结论：**要 2D 栅格就用 fastlio**；`map` 系倾角是 LIO 槽位的属性，不是世界的属性。

**坑 4：`map_archive.sh save` 不接受 `--resolution`**（分辨率是它自己 `ros2 param get /slam_toolbox resolution`
问出来的；多传一个不认识的参数会打 usage 然后什么都不写，而 `rc` 被管道吃掉 ⇒ 静默失败）。

### 16.5 实测：RMUC2026

| 指标 | 旧（默认） | 新 `RMUC2026_v4`（驱动版，默认模型+fastlio） | 新 `RMUC2026_v4b`（静置版，robot11+sensor） |
|---|---|---|---|
| 3D 点数 / 文件 | 203,352 / 6.51 MB | **73,376 / 2.35 MB** | 8,300 / 0.27 MB |
| 3D bbox / XY 跨度 | `x[-26.73,4.49] y[-12.73,9.64] z[-0.545,1.813]`（31.2×22.4 m，**比场地还大**） | `x[-25.60,3.73] y[-9.55,7.23] z[-0.51,1.80]`（29.33×16.78 m ≈ **场地本身**） | `x[-25.03,3.76] y[-5.91,7.10] z[-0.12,1.47]`（28.79×13.01 m） |
| 2D 栅格 | `577x326`，`origin [-25.2,-9.09]`，occ 9769 / free 178333 / unknown 32135 | **`312x300`，`origin [-11.9,-8.3]`，occ 3135 / free 90465 / unknown 51880** | 静置版 2D **不可用**（单视点，occ 只有 ~100 格） |
| 墙面残差（网格基准） | `|resid| p50 = 0.1232 m`、max 0.225（n=31，水平/出生点系变换） | `p50 = 0.1528 m`、max 0.190（n=5） | — |
| world 地面平面 z | −0.053 m | **+0.283 m** ← 见 §16.9#4 | +0.051 m |

* **RMUC2026 的墙面基准本身不可用**（§13.10#6 已记：这份 mesh 的竖直面碎/曲/斜，±0.25 m 窗里总会
  进相邻面）⇒ 上一行的 p50 新旧都不可判读，**不作为结论**。RMUL2026 才是干净基准（§16.6）。
* 新 3D 的 bbox 反而**更像场地**：旧 PCD 的 `x[-26.7,4.5] y[-12.7,9.6]` 比 29×16 m 的场地还大
  2.3/4.5 m（那是旧会话里 LIO 漂出去的离群柱）。
* 驱动版的新 2D 只覆盖 **15.6×15.0 m**（出生点附近；机器人只走了 ~10 m）⇒ **覆盖度是回归**，
  这是本轮**没有提升它**的直接原因（旧图覆盖 28.85×16.30 m）。

### 16.6 实测：RMUL2026（**关键几何证据**）

静置版 `RMUL2026_v4b`（robot11+sensor+hybrid+small_point_lio，静止 150 s 累积）：

* 3D：**4,178 点 / 0.13 MB**，bbox `x[-2.07,10.24] y[-2.00,5.12] z[-0.14,0.33]`（12.31×7.12×0.47 m，
  与 12.6×8.6 m 的场地同量级）；`xy` 足迹 23.7 m²。
* 2D：`150x93 @0.05 origin[-1.79,-0.611]`，occ **105** ⇒ **不可用**（单视点，一条射线扇面）。
* **墙面残差（网格基准，`map_wall_probe.py`，把 PCD 用同一跑实测的 `R_world←map` 搬回 world）**：

  | 网格面 | 旧 `PCD/RMUL2026.pcd` | 新 `PCD/RMUL2026_v4b.pcd` |
  |---|---|---|
  | **x = +2.5** | **−0.0381 m**（n=36） | **+0.0031 m**（n=284） |
  | **y = −8.5** | **+0.0492 m**（n=44） | （单视点看不到这个面） |
  | x = −5.0 | — | **−0.0020 m**（n=217） |
  | x = −10.0 | — | **−0.0015 m**（n=42） |
  | `\|残差\|` p50 / max | **0.0436 / 0.0492 m** | **0.0020 / 0.0031 m** |

  ⇒ 墙的位置从 **3.8–4.9 cm** 掉到 **1.5–3.1 mm**（10–20 倍）。**这就是"用修好的插件建出来的图，
  墙真的在网格上"的判决性证据**（而且它是单视点、没有 LIO 漂移贡献的干净样本）。
* world 地面平面 z = +0.086 m（119 点内点）：**不是几何误差**，是单视点云里地面点太少
  （垂直 FOV −7.22° ⇒ 地面只在 0.8–1.8 m 那一圈可见）+ RMUL 地面本身有 ~1 cm 台阶 ⇒
  直方图峰容易落到低矮结构的**顶面**上。地面绝对高度**本行不可用**，以 §16.3 的 floor_probe 为准。

### 16.7 GICP 创新 / 采纳（**关键验收**：`mode:=nav localization:=gicp`，静止，`gicp_gate_probe.py`）

口径：`lio:=fastlio robot:=robot11 robot11_mount:=sensor`、`nav_rviz:=False spin_speed:=0.0`、
`initial_pose` 用配置里的 `[0,0,0]`（gates：`allow_xy` 实测恒 **0.30 m**、`allow_yaw` 6°，
窗口 = 仿真 25–45 s，`/clock` BEST_EFFORT）。

| 臂 | world | PCD | 帧数 | **采纳率** | **innov p50 / p95 / max** | `innov_dx/dy/dz p50` | score p50 |
|---|---|---|---|---|---|---|---|
| **before** | RMUL2026 | 旧 `RMUL2026.pcd` | 200（窗口）/471（整跑） | **100%**（拒绝 0） | **0.01065 / 0.02510 / 0.07429 m** | +0.0041 / −0.0002 / −0.0006 | 0.0051 m² |
| **before** | RMUC2026 | 旧 `RMUC2026.pcd` | 200（窗口）/470（整跑） | **100%**（拒绝 0） | **0.00943 / 0.02460 / 0.04629 m** | +0.0000 / −0.0001 / −0.0001 | 0.0139 m² |
| **after** | RMUC2026 | 新 `RMUC2026_v4.pcd` | 200（窗口）/470（整跑） | **100%**（拒绝 0） | **0.00427 / 0.01091 / 0.02205 m** | +0.0001 / +0.0001 / −0.0001 | **0.0027 m²** |

**判读（重要，与任务预期不同）**

1. **旧 PCD 在 RMUL2026 上已经"能用"**：采纳 100%、创新 p50 **1.07 cm** —— 远低于 0.30 m 门限。
   原因不是旧图没错，而是 **§13 之后那把"首帧门死锁"的修复（`9bb8f2f`）已经让 GICP 把
   `map→odom` 的常量错位一次性吃掉**（实测 `map→odom = (-0.060, +0.133, 0.015) m / −0.21°`），
   之后每帧的创新只剩噪声。⇒ **"创新 0.11–0.22 m / 采纳 0"是"门死锁"那个 bug 的像，不是旧图的像**；
   修了门以后，旧图的偏置表现为**一次性 13 cm 的 `map→odom` 修正**，而不再表现为持续拒绝。
**（补）本节实测的三个 `map→odom` 常量偏置**：RMUC/RMUL 的旧 PCD 分别要 `(-0.139,+0.086,−0.003) m / 1.18°`
与 `(-0.057,+0.130,+0.016) m / −0.16°` 才对齐；新 `RMUC2026_v4.pcd` 要 `(-0.143,+0.120,+0.403) m / −0.61°`
—— **y 向 13.0 → 12.0 cm（不变）、z 向 −0.3 → +40.3 cm**，这正是 §16.9#4 那个"地面 +0.283 m"不明偏置在
6-DoF 里的像（GICP 把 z 偏置整段吃进了 `map→odom`）。⇒ **新 PCD 的 `score` 好了 5 倍，但 z 平移
不能用**（2D 导航不受影响，3D 用图的人要注意）。

2. 因此在本轮口径下，**创新/采纳已经不能分辨"图对不对"**（两臂都 100% 采纳）——
   能分辨的是 **`map→odom` 的常量修正量**（旧图 `(-0.139,+0.086,−0.003) m / 1.18°`、新图 `(-0.143,+0.120,+0.403) m / −0.61°`）和
   §16.6 的**墙面绝对残差**（旧 4 cm / 新 2 mm）。这条对"以后怎么验收地图"是个修正：
   **别只看采纳率，要看 `map→odom` 的常量偏置和墙面绝对残差。**

### 16.8 `map_asset_guard.py` / sidecar

* 新资产的 `<name>.meta.yaml` 全部由 `map_archive.sh save`（= `map_asset_guard.write_manifest`）写出：
  `world` / `spawn_pose`（RMUC `10.925,2.525,0.2`、RMUL `4.3,3.35,0.2`）/ `map_start_pose [0,0,0]` /
  `resolution 0.05`（**只在 `slam_toolbox` 活着时问得到**，见坑 4）/ `frames.convention` / `files` /
  `file_sizes` / `versions`；PCD 侧另带 `accumulated_points / voxel_size / bbox / z_span /
  health_warning / dropped_* / skipped_frames_* / tf_frames_ok/failed / source: cloud_accumulator`。
* `map_archive.sh list` / `info --name <W>_v4`、`info --name <W>_v4b` 均能读出（world/spawn 一致）；
  守卫的 `verify()` 对同名续建判 `resume`（world+spawn 一致）⇒ **sidecar 与场地隔离链是通的**。
* 默认资产（`map/<W>.pgm|yaml`、`PCD/<W>.pcd`）**没有** sidecar（它们是"先验资产"不是"会话存档"，
  仓库惯例如此，`docs/map_assets.md` §1）⇒ 本轮没有改变这个形态。

### 16.9 什么变了 / 什么没变（诚实边界）

**变了的**

1. `map/RMUC2026_v4.{posegraph,data,meta.yaml}` + `PCD/RMUC2026_v4.{pcd,meta.yaml}`
   （驱动版：73,376 点、覆盖全场、level `map` 系、2D 只覆盖出生点 15.6×15.0 m）。
2. `map/{RMUC2026,RMUL2026}_v4b.{posegraph,data,meta.yaml}` + `PCD/{...}_v4b.{pcd,meta.yaml}`
   （静置版：8300 / 4178 点；`map` 系斜 30°，因为 small_point_lio；2D 不可用）。
3. `.tmp_maprebuild/` 里 4 段可复跑的脚本 + 每跑的 `meta.txt`/`log`/`align.json`/`drive.json`。
4. `map/RMUC2026_v4_spl.{pgm,yaml}`（驱动版 2D 候选，**入库**；`.pcd`/`.posegraph`/`.data` 按仓库
   `docs/map_assets.md` §7.1 的惯例**不入库**，只留在工作树里）。
5. 本节文档。

**没变的（故意的）**

1. **两个世界的默认先验一个字节都没改**（`map/<W>.pgm|yaml`、`PCD/<W>.pcd`）——因为
   ① 新 2D 的覆盖度是**回归**（RMUC 15.6×15.0 m vs 旧 28.85×16.30 m；RMUL 只有 105 个占用格）；
   ② 驱动版 3D 的地面高度有 +0.283 m 的不明偏置（§16.9#4）；③ 静置版 `map` 系斜 30°。
   ⇒ **没有一条同时满足"几何对 + 覆盖够 + `map` 系水平"的候选**，提升任何一条都是拿一个回归换一个修复。
2. `src/rm_localization/**`、`src/rm_simulation/**`、`tools/scripts/{diag,regress,localization}/**`、
   §10–§13 一个字没动。
3. 没有重跑 §10/§12 的基线（按任务要求留给后续）。

**四条新发现 / 未解决项**

1. **底盘自转驱不动**（§16.4 坑 1）：`v=0.30 → 0.183 m/s`（61%）但 `w=0.80 → 0.025 rad/s`（3%）。
   这不是本任务的改动引出的（`libgazebo_ros_planar_move` + 轮子接触摩擦），但它**挡住了所有覆盖跑图**。
   复现：`.tmp_maprebuild/drive_probe.sh dp_rmuc RMUC2026 fastlio`。
2. **`small_point_lio` + 默认模型槽位（`R_bl=I` 档）SIGSEGV**（exit −11，节点起来 ~12 s 后死，
   `robot11_mount:=sensor` 档不崩）⇒ 任何人用"默认模型 + small_point_lio"建图都会 12 s 后没有 `/odom`。
   复现：`world:=RMUC2026 mode:=mapping lio:=small_point_lio`（默认 `robot`）。**未修**（不许动 `rm_localization/**`）。
3. **`robot11_mount:=sensor` + `small_point_lio` 的 `map` 系斜 30°**（§16.4 坑 3）⇒ 这一档建不了 2D 栅格。
   `fastlio` 没有这个问题。**建议**：要建 2D 先验时用 `lio:=fastlio`（或给 small_point_lio 加"odom 重力对齐"）。
4. **驱动版 3D 的地面高度 +0.283 m（world）不明**：同一跑 `align.json` 的 `t_wm z = +0.1646`、
   `base_link` 在 map 系 `z = −0.0975`，而 PCD 的 z 直方图众数在 map `z = +0.115`（应≈ base_link − 0.06 = −0.157）。
   候选原因：默认模型槽位 `cloud_frame=parent` 下点云被表达在**父 link 系**、而 `frame_id` 仍是
   `livox_frame`（`docs/tilted_lidar_fidelity.md` §N.5 记了这个不一致）⇒ 下游按 `livox_frame` 再叠一次
   安装偏移。**本轮没有做判决性实验**（需要一次"默认模型静止 + floor_probe/wall_witness"，被时间挤掉），
   **列为本节最大的未验证项**。

**其它没能验证的**

* 新 3D **只验了出生点附近的绝对几何**（RMUL 的 3 个面 / RMUC 的 mesh 不可用）：场地远端没有真值位姿，
  墙体绝对位置只能靠 mesh，而 mesh 只有 RMUL 那份适合当基准。
* 驱动版 PCD 的**覆盖完整性**没验（点数 73k vs 旧 203k，但旧的多出来的大部分是离群柱与垂向拖影）。
* 新建的 2D 栅格**没有跑** `verify_low_terrain.py` / `map_prior_ab.py`（覆盖度一项就足以否掉提升）。
* 只跑了 1 跑/臂（RMUL before、RMUC before/after），没有 3 跑中位。
* `mode:=nav` 的 `nav2` 侧（costmap/规划）在**新 2D 图**上没跑过（新 2D 没提升，也没被导航用过）。

### 16.10 一句话给用户

> **"按修好的插件重建"这件事技术上走通了**：工具链现场复核 **−0.1919 → 0.0000 m**；
> RMUL2026 新 PCD 的墙面残差 **1.5–3.1 mm**（旧 38–49 mm，10–20 倍改善）。**但这台机器现在驱不动底盘**
> （自转只有指令的 3%）⇒ 覆盖跑图 26 min 只走 10.6 m，新 2D 先验的覆盖度从 28.85×16.30 m 掉到
> 15.60×15.00 m ⇒ **本轮没有提升任何默认资产**（旧的一个字节没删，全部 `.bak-20261019`）。
> 另外撞到两条独立缺陷：`small_point_lio` + 默认模型槽位 **SIGSEGV**、`robot11_mount:=sensor` +
> `small_point_lio` 的 **`map` 系斜 30°**（fastlio 无此问题）。下一步建议：先修底盘自转（或换
> `planar_move`→真差速），再跑一次全场覆盖，然后按 `docs/map_assets.md` §4.2 提升。

## §17 第九轮（**本机 date=2026-10-20T00:08–00:30**）：nav2 高度带**静默丢波束**的只读守卫 `band_guard`（opt-in）

> **本节只新增**（append-only）：§1–§16 一个字都没改。写作时 §16 已被**并发 agent**的
> 建图轮占用 ⇒ 本节取第一个未占用编号 **§17**（任务书写"§15 是最新"是它落笔时的状态）。
> 本轮**没有**改 `src/rm_navigation/**`、`src/rm_localization/**`、`src/rm_simulation/**`、
> nav2 参数文件、地图/PCD 资产。
>
> **代码（4 处，全部 opt-in）**：
> ① `src/rm_nav_bringup/scripts/band_guard.py`（**新增**，只读节点；下文行号按本提交）；
> ② `src/rm_nav_bringup/launch/bringup_sim.launch.py`（新增 launch arg `band_guard` +
>    一个 condition 为假的 Node；L2212 / L3314 / L3446 / L3506）；
> ③ `src/rm_nav_bringup/CMakeLists.txt`（把脚本装到 `lib/rm_nav_bringup/`，L42–48）；
> ④ `src/rm_nav_bringup/package.xml`（`sensor_msgs` / `tf2_ros` / `rcl_interfaces`，L20–23）。

### 17.1 一句话结论

> **加了一个"只读哨兵"**：它订阅 `/scan`、按 nav2 的**同一口径**（`projectLaser` 的
> `(r·cos a, r·sin a, 0)` + `inf_is_valid` + `ObservationBuffer` 的 z 闸）把每条波束搬到
> 代价图帧，数 `in-band / below / above`、**kept-y 区间**和 z 的 min/p50/max；数据来自
> **两张 costmap 运行期参数服务**（单一真源），**只写日志 + 发 `/band_guard/status`（JSON）**，
> 不碰 scan / costmap / TF / 控制。**默认 `band_guard:=False` ⇒ 不起节点**（R3 实测
> `ros2 node list` 里没有 `/band_guard`），且**合并出的 nav2 参数文件 md5 与不开守卫逐字节相同**
> （`e5179ab…`）。**同一栈内**先测 15 s（无守卫）再起守卫测 15 s：local lethal **266 → 265**、
> global lethal **3898 → 3897**（`free` 42415→42438 / 9693→9721，差 ≤0.07%）。
>
> **两个失败都抓到（逐字日志见 17.5/17.7）**：
> · 局部 `off`：**333/772（43.1%）**、`kept-y=[0.275,3.460]` 单侧 `+y`、`z p50=-0.093`
>   （= §15 记录的 333/774、kept-y [0.112,3.568]、§13.8 的 z p50 −0.0919 同一档）；
> · 全局 `map` 无 `-1.0` 覆盖：**0/774（0.0%）ERROR**、`z p50=-0.178`、全局 lethal
>   **3713 恒定 / inscribed 16886 / free 10176**（= §15 的"static+inflation 哑图"）；
>   带 `-1.0` 覆盖：**776/776（100%）**、全局 lethal **3866.5**（有实时障碍）。
> · `hybrid` 修好局部：**772/772（100%）**、kept-y 整盘 `[-0.637,5.089]`、`z p50=+0.036`
>   （= §13.8 的 +0.0360）。

### 17.2 它到底量什么 / 不做什么（规约）

**严格复刻 nav2 的两步**（见 `nav2_costmap_2d/plugins/obstacle_layer.cpp` 与
`src/observation_buffer.cpp:143-144`）：

1. **扫描系投影**：对第 i 条波束，`a = angle_min + i·angle_increment`，扫描系点 =
   `(r·cos a, r·sin a, 0)`；只保留 `range_min ≤ r < range_max`（`projectLaser` 的
   `range_cutoff = range_max`）。若该源的 `inf_is_valid=true`，先把 **+inf** 换成
   `range_max − 1e−4` 再投影（`laserScanValidInfCallback`）；NaN/−inf 丢弃。
2. **搬到代价图帧 + 两道 z 闸**：用 TF2 在 **scan 的 stamp** 查 `scan.frame_id → global_frame`
   （查不到才退回最新一帧并在日志里标 `mode=latest`），对每个点算 `z`。nav2 实际有**两道**闸：
   ⚠️ 代价图的 `transform_tolerance` 会被读进状态（`tf_tolerance_s`），但本节点**不把它当
   阻塞超时** —— 单线程 executor 里阻塞等 TF 会饿死 TF 订阅本身（§10.4 的 `nav_band_probe`
   第一版 124/124 全失败就是这个坑）；做法是 stamp 查一次、失败立即用最新帧，并在日志里
   标 `mode=latest`。
   `ObservationBuffer` 的**每源** `<layer>.<source>.min/max_obstacle_height`，以及
   `ObstacleLayer::updateBounds` 的**图层级** `<layer>.min/max_obstacle_height`。
   本守卫取**交集** `[max(layer_min, source_min), min(layer_max, source_max)]`，数：
   `in_band`、`below`（z < min_eff）、`above`（z > max_eff）、**kept-y**（带内点的 y 的
   min/max/span）、**all-y**、z 的 min/p50/max。
   ⇒ **`kept-y` 单侧**就是"半平面盲区"的直接证据（§9.7/§12 的 333/774 + [0.112,3.568]）。

**两套计数都打出来（口径必须写清）**：

* `total / in_band` = **nav2 真实云大小**（含 `inf_is_valid` 换出来的最大量程点）；
* `finite_total / finite_in_band` = **只算有限 range**（= 既往 `nav_band_probe.py` 的
  `n_finite` 口径，便于跨文档比较）。本机 `/scan` 有 1462 条角度，逐帧有限回波
  **768–789**（中位数 774–776）；所以"0/774 / 333/774"是**某一帧**，而守卫 5 s 采一帧会看到
  0/774、333/772、333/775、333/776 这类相邻值 —— 比例与判决不受影响。

**报警规则（只写日志）**：

| 条件（`in-band` 按 nav2 真实口径） | 级别 | 必带字段 |
|---|---|---|
| `in_band == 0` 且 `/scan` 在正常发布（龄期 ≤1 s、速率 ≥1 Hz） | **ERROR** | frame、layer/source、两道闸的四个数、scan frame、TF（目标/源/戳/mode/age）、z p50 |
| `in_band / total < 10%` | **WARN** | 同上 + 丢掉的条数 |
| kept-y 单侧：`kept_y_min > origin_y + 0.05` 或 `kept_y_max < origin_y − 0.05`，且**全盘**跨过原点（避免"波束本来就只在一侧"的误报） | **WARN** | kept-y 两个数 + all-y 两个数 + origin_y |
| TF 查不到 | **TF_FAIL**（WARN） | 源/目标/失败原因；**不**据此报高度带 ERROR |
| `/scan` 没数据/太慢/太旧 | 不报警（状态 `no_scan_data`） | 龄期/速率写进状态话题 |
| 没有启用的 2D scan 层（如默认 `global_obstacle:=stvl`） | 不报警（`idle`） | `plugins`/`enabled`/跳过原因 |

**限流**：每个 `costmap/layer/source` 最多 `rate_limit_s`（默认 5 s）一条；级别升级立即打印；
降级到 OK 时打一条 INFO。**正常帧也打 INFO**（否则验收 2/3 的 100% 没有日志证据），
可用 `log_healthy:=false` 关掉。

**状态话题**：`std_msgs/String` JSON 发到 `/band_guard/status`，**transient_local**（后加入的
读者能拿到最后一条），每个评估周期一条；schema
`rm_nav_bringup/band_guard@1`，含 `scan`（topic/stamp/frame/rate/age/ok）、`costmaps[]`
（frame/layer/source/band 的 source+layer+effective、counts、kept-y、z、TF 的
translation+rotation、severity、hint）、`alerts[]`、`single_source`。可直接进 A/B 记录。

**节点参数**（launch 里写死默认，可命令行覆盖）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `scan_topic` | `/scan` | 订阅（sensor QoS） |
| `costmap_nodes` | `['/local_costmap/local_costmap','/global_costmap/global_costmap']` | 参数服务目标 |
| `period_s` | `5.0` | 评估周期（节点时钟；仿真里=仿真秒） |
| `params_refresh_s` | `30.0` | 参数缓存刷新（启动读一次 + 每 30 s） |
| `min_scan_rate_hz` | `1.0` | 低于它不报带子 ERROR |
| `scan_stale_s` | `1.0` | 最新 scan 最大龄期 |
| `inband_warn_fraction` | `0.10` | `<10%` WARN 阈值 |
| `y_onesided_margin_m` | `0.05` | kept-y 单侧余量 |
| `rate_limit_s` | `5.0` | 每 costmap 日志最小间隔 |
| `status_topic` / `publish_status` | `/band_guard/status` / `True` | 状态话题 |
| `log_healthy` | `True` | 正常帧也打 INFO |

**契约**：只订阅 `/scan`、`/tf`、`/tf_static`；只调用**只读**参数服务
（`<node>/list_parameters` + `<node>/get_parameters`）；只发布 `/band_guard/status`。
不发布 TF、不发布/订阅任何控制或 costmap 话题、不写文件。

### 17.3 单一真源：选 (a) **运行期参数服务**，不选 launch 传参

本节点直接读 `/local_costmap/local_costmap` 与 `/global_costmap/global_costmap` 的
**运行期参数服务**：

* `plugins`、`global_frame`、`transform_tolerance`；
* 每个图层的 `<layer>.plugin/.enabled/.observation_sources`；
* 图层级 `<layer>.min_obstacle_height/.max_obstacle_height`（`updateBounds` 第二道闸）；
* 每源 `<layer>.<source>.data_type/.topic/.min_obstacle_height/.max_obstacle_height`
  （`ObservationBuffer` 第一道闸），另读 `inf_is_valid`。

**为什么不是 (b)（launch 从同一份 nav2 params 再算一遍）**：
1. **不会漂移**：launch 的深合并（base → `robot11` 覆盖 → `global_obstacle:=scan` 覆盖）与
   `RewrittenYaml` 的 rewrite 都可能改值；参数服务返回的是 nav2 **真正拿到**的值。
   本轮实测：`global_obstacle:=scan` 的 global 是 **−1.0/−1.0**，局部是 **0.0/0.0**；
   R4 的 scratch 覆盖把它们改成 **0.0/0.0**，守卫**同一份代码**立刻改报 0/774（17.7）。
2. **可独立运行**：`ros2 run rm_nav_bringup band_guard.py` 能挂到**任意**已在跑的栈上，
   不需要用户告诉它参数文件在哪，也不需要 launch 为它加参数。
3. **覆盖动态改参**：`ros2 param set` 改过的值 30 s 内会被看到（本轮没有专门压测动态改参）。

**代价**：要等 costmap 节点起来（0.5 s 轮询、不挡启动）；参数缓存 30 s 刷新。
本轮用**进程内异步客户端**（不是 `ros2 param get` 子进程），每次只查 ~30 个键；
§15.11 记录过"40 次 `ros2 param get` 子进程风暴"会扰动 MessageFilter，本实现刻意避开。
**证明它是 live 值**（R2 的逐字日志）：

```text
[band_guard.py-11] [INFO] [1792426642.279887772] [band_guard]: [band_guard] 参数回读（live）/global_costmap/global_costmap: frame=map plugins=['static_layer', 'obstacle_layer', 'stvl_layer', 'inflation_layer'] obstacle_layer.band=[-1.000,2.000] obstacle_layer.scan[LaserScan topic=/scan src.band=[-1.000,2.000]] stvl_layer(enabled=false)
```

R4（scratch 覆盖 min=0.0）同一行的 live 值：

```text
[band_guard.py-11] [INFO] [1792427024.982840089] [band_guard]: [band_guard] 参数回读（live）/global_costmap/global_costmap: frame=map plugins=['static_layer', 'obstacle_layer', 'stvl_layer', 'inflation_layer'] obstacle_layer.band=[0.000,2.000] obstacle_layer.scan[LaserScan topic=/scan src.band=[0.000,2.000]] stvl_layer(enabled=false)
```

### 17.4 集成：launch arg + standalone；默认路径的证明

**launch arg**：`band_guard:=True`（默认 `False`，choices True/False）。Node 的 condition =
`band_guard.lower()=='true' and (mode=='nav' or mode=='slam_nav')`；`mode:=mapping` 不起。
Node 参数 = 17.2 的表（`use_sim_time` 透传）。它**不**改变 nav2 的 params 文件，
也不改任何既有 Node 的参数。

**standalone**（不需要改 launch）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run rm_nav_bringup band_guard.py --ros-args -p use_sim_time:=true
# 或纯源码树：
python3 src/rm_nav_bringup/scripts/band_guard.py --ros-args -p use_sim_time:=true
```

**默认路径证明 A（Action 树）**：`tools/scripts/gzsim/launch_increment_diff.py`，old=HEAD
（本提交前）的 launch，new=本提交，默认参数：

```text
actions: OLD 67 / NEW 69；共有 67

新增（NEW − OLD，整棵子树逐字段）：2
  + DeclareLaunchArgument name=band_guard
  + Node cond=IfCondition
消失/变成另一份（OLD − NEW）：0

共有 Action 的相对顺序：一致 ✅
==> 纯增量 ✅：没有任何 Action 被删/改，只有上面 2 个新增，原有顺序不变
```

⇒ **67 个既有 Action 逐字段相同、顺序不变、0 个消失/被改**。新增的 2 个 =
① launch arg 声明；② condition 恒假的 Node（默认不执行）。
**诚实写法**：这不是"Action 树 0 增 0 删"，而是"**纯增量**"；`--strip-conditions` 的对照
里新增的 Node 内容也在（条件去掉后只剩 `+ Node`），说明新增的只有这扇门本身。

**默认路径证明 B（真实节点集）**：R3（`band_guard:=False` 的 hybrid 跑）的
`ros2 node list` **没有** `/band_guard`（`grep band_guard` = 0）；`nodes.txt` 里 36 个节点
（含 9 个 `transform_listener_impl_*`）与改动前一致。R2（`band_guard:=True`）才有 `/band_guard`。

**默认路径证明 C（参数文件）**：同一组 launch 参数下，`band_guard:=True` / `False` /
`gravity_align_mode:=off` 三种跑合并出的 nav2 YAML **md5 完全相同**：

```text
e5179abdbd33ab4865046f7b08deb38b  bg_hyb_guard/roslog/nav2_params_sim_robot11_global_scan_merged.yaml
e5179abdbd33ab4865046f7b08deb38b  bg_hyb_noguard/roslog/nav2_params_sim_robot11_global_scan_merged.yaml
e5179abdbd33ab4865046f7b08deb38b  bg_off_guard2/roslog/nav2_params_sim_robot11_global_scan_merged.yaml
```

⇒ 守卫**没有**改 nav2 的生效参数（它也从不写 params 文件）。

### 17.5 复现 A：局部 `off`（斜 odom）——守卫报单侧 kept-y

**命令**（隔离口径：`HOME`/`ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI` 私有、`gui:=False`、
`nav_rviz:=False lio_rviz:=False`、`setsid` 起栈、`scoped_cleanup.sh` 只清本进程组/本 URI）：

```bash
world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp global_obstacle:=scan \
robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off band_guard:=True
```

**逐字日志**（`bg_off_guard2/launch.log`，9 个评估周期里的一条）：

```text
[band_guard.py-10] [WARN] [1792426539.839560853] [band_guard]: [band_guard] WARN /local_costmap/local_costmap frame=odom layer=obstacle_layer source=scan | in-band 333/772 (43.1%) below=437 above=2 finite=333/772 inf_converted=0 | kept-y=[0.275, 3.460] m span=3.185 origin_y=0.032 one-sided=+y all-y=[-0.444, 4.467] | finite kept-y=[0.275, 3.460] finite all-y=[-0.444, 4.467] | z min/p50/max=-0.390/-0.093/2.457 finite=-0.390/-0.093/2.457 | band=[0.000,2.000] source=[0.000,2.000] layer=[0.000,2.000] | scan frame=base_link rate=10.00Hz age=0.100s | tf base_link->odom @ 669.000s mode=stamp age=0.000s | hint=kept-y is ONE-SIDED (+y): kept=[0.275,3.460] m while all beams span [-0.444,4.467] m; nav2 keeps only a slab/half-plane. Check the TF base_link->odom vs band [0.000,2.000].
```

**9 个守卫样本的中位数 [min–max]**：

| 量 | 守卫（live 参数） | 记录值（§15/§13） |
|---|---|---|
| local finite in-band / total | **333 / 776** [326–336 / 768–786] | **333/774**（§15.9 `exp15_gscan_off_r2`） |
| local in-band 比例 | **42.9%** [42.2%, 43.2%] | 43.0% |
| local kept-y（finite） | **[0.268, 3.404]** [min 0.227–0.283, max 3.132–3.629] | **[0.112, 3.568]**（§12.2） |
| local z min / p50 / max | **−0.501 / −0.091 / 2.459** [−0.567…−0.390 / −0.093…−0.088 / 1.838…2.466] | z p50 **−0.0919**（§13.8）；§12 旧跑 +0.243（地面校准前） |
| TF | `base_link->odom`，全部 `mode=stamp`（1 条早期 `mode=latest`） | — |
| 探针 `/scan` 有限波束中位数 | **775.5** | 774–776 |
| 探针 local lethal / inscribed / free | **288 / 8896.5 / 44260** | §15.9 off：293/8855/44318 |

**判决**：`in-band 43%` + **kept-y 整段 > 0**（one-sided `+y`）正是 §9.7/§12 的"斜板/半平面"
——**不是**"全空"那么显眼，但已经丢了 57% 的波束和负 y 半平面的障碍；守卫按规则报 WARN。
同一跑的**全局**（带 `-1.0` 覆盖）是 776/776 OK（见 17.6 的 global 行），即"局部病、全局没病"。

### 17.6 复现 B：`hybrid` 修好局部；全局 `-1.0` 覆盖下 100%

**命令**：同 17.5，只把 `gravity_align_mode:=off` 换成 `hybrid`。

**逐字日志（局部）**：

```text
[band_guard.py-11] [INFO] [1792426645.843446562] [band_guard]: [band_guard] OK /local_costmap/local_costmap frame=odom layer=obstacle_layer source=scan | in-band 772/772 (100.0%) below=0 above=0 finite=772/772 inf_converted=0 | kept-y=[-0.637, 5.089] m span=5.726 origin_y=-0.054 one-sided=no all-y=[-0.637, 5.089] | finite kept-y=[-0.637, 5.089] finite all-y=[-0.637, 5.089] | z min/p50/max=0.023/0.036/0.064 finite=0.023/0.036/0.064 | band=[0.000,2.000] source=[0.000,2.000] layer=[0.000,2.000] | scan frame=base_link rate=10.00Hz age=0.100s | tf base_link->odom @ 644.000s mode=stamp age=0.000s | hint=none
```

**逐字日志（全局；`global_obstacle:=scan` 的 −1.0 覆盖生效）**：

```text
[band_guard.py-11] [INFO] [1792426645.844273088] [band_guard]: [band_guard] OK /global_costmap/global_costmap frame=map layer=obstacle_layer source=scan | in-band 772/772 (100.0%) below=0 above=0 finite=772/772 inf_converted=0 | kept-y=[-0.633, 5.066] m span=5.698 origin_y=-0.083 one-sided=no all-y=[-0.633, 5.066] | finite kept-y=[-0.633, 5.066] finite all-y=[-0.633, 5.066] | z min/p50/max=-0.244/-0.179/-0.125 finite=-0.244/-0.179/-0.125 | band=[-1.000,2.000] source=[-1.000,2.000] layer=[-1.000,2.000] | scan frame=base_link rate=10.00Hz age=0.100s | tf base_link->map @ 644.000s mode=stamp age=0.000s | hint=none
```

9–10 个样本的中位数：local **779.5/779.5（100%）**、kept-y **[-0.636, 5.086]**、z
**0.020 / 0.036 / 0.068**；global **779.5/779.5**、kept-y **[-0.630, 5.064]**、z
**−0.245 / −0.179 / −0.128**。探针：local lethal/free **267 / 42442**、global
**3866.5 / 9693.5**；`/scan` 有限 775。
⇒ 与 §13.8 的 hybrid z p50 **+0.0360** 和 §15.9 的 global 774/774 / lethal 3851–3919 同档。

### 17.7 复现 C：全局 `map` 去掉 `-1.0` 覆盖 ⇒ **0/774 ERROR**

**怎么在不改源码/资产的前提下复现旧配置**：scratch launch wrapper
（`.tmp_band_guard/launch/bringup_sim_noband.py`，**gitignored**）import 真正的
`bringup_sim.launch.py`，monkeypatch `_Nav2ParamsForSlot.perform`，在它写出
`…_merged.yaml` 后**只改两个键**：`global_costmap.global_costmap.ros__parameters.
obstacle_layer.min_obstacle_height` 和 `…obstacle_layer.scan.min_obstacle_height`，都设回
**0.0**。用 `diff` 证明**只动了这两行**（R2 vs R4 的合并文件）：

```diff
-          min_obstacle_height: -1.0
-        min_obstacle_height: -1.0
+          min_obstacle_height: 0.0
+        min_obstacle_height: 0.0
```

**命令**（其余同 17.6；`LAUNCH_FILE` 指向 scratch wrapper）：

```bash
LAUNCH_FILE=.tmp_band_guard/launch/bringup_sim_noband.py \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp global_obstacle:=scan \
  robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=hybrid band_guard:=True
```

**逐字日志（先参数回读，再 ERROR；0/774 是 9 个样本里的一帧）**：

```text
[band_guard.py-11] [INFO] [1792427024.982840089] [band_guard]: [band_guard] 参数回读（live）/global_costmap/global_costmap: frame=map plugins=['static_layer', 'obstacle_layer', 'stvl_layer', 'inflation_layer'] obstacle_layer.band=[0.000,2.000] obstacle_layer.scan[LaserScan topic=/scan src.band=[0.000,2.000]] stvl_layer(enabled=false)
[band_guard.py-11] [ERROR] [1792427082.595548756] [band_guard]: [band_guard] ERROR /global_costmap/global_costmap frame=map layer=obstacle_layer source=scan | in-band 0/774 (0.0%) below=774 above=0 finite=0/774 inf_converted=0 | kept-y=[-, -] m span=- origin_y=-0.095 one-sided=no all-y=[-0.854, 5.061] | finite kept-y=[-, -] finite all-y=[-0.854, 5.061] | z min/p50/max=-0.257/-0.178/-0.131 finite=-0.257/-0.178/-0.131 | band=[0.000,2.000] source=[0.000,2.000] layer=[0.000,2.000] | scan frame=base_link rate=10.00Hz age=0.100s | tf base_link->map @ 664.000s mode=stamp age=0.000s | hint=ALL 774 beams are outside band [0.000,2.000] in frame=map; nav2 will silently keep 0/774. Check BOTH obstacle_layer.scan.min_obstacle_height=0.000 and obstacle_layer.min_obstacle_height=0.000 (and max).
```

**测量**：全局 **0/780 中位（0/774 那一帧）**，z **−0.245 / −0.179 / −0.129**；
探针全局 **lethal 3713 恒定、inscribed 16886 恒定、free 10176 恒定**（43 帧 min=max）
= §15 的"static+inflation 哑图"；**同一跑局部 hybrid 780/780 OK**，local lethal 266、
free 42442。
⇒ 守卫抓到了"全局层被整盘清空但 nav2 零告警"的原始失败；改回 −1.0（R2）则
**776/776**、global lethal **3866.5**（min≠max 3858–3884）。

### 17.8 验收跑汇总（4 个正式跑 + 1 个栈内 A/B）

隔离：每个 run 私有 `GAZEBO_MASTER_URI` / `HOME` / `ROS_DOMAIN_ID`、`gui:=False`、
`nav_rviz:=False lio_rviz:=False`、`setsid`、`timeout`、`scoped_cleanup.sh`（无全机 pkill）。
探针 = `tools/scripts/tiltmount/nav_band_probe.py`（只订阅；`settle 20 / window 26`）。
守卫数字 = 该跑 `status.jsonl` 的**中位数**；`finite_*` 是跨文档可比的有限波束口径。

| run（tag） | 组合 / 重力 | 守卫 | local finite in-band / total | local kept-y（finite） | local z min/p50/max | global in-band / total | global z p50 | 探针 local lethal / free | 探针 global lethal / free |
|---|---|---|---|---|---|---|---|---|---|
| `bg_off_guard2` | off + global:=scan（−1.0） | on | **333 / 776**（42.9%） | **[0.268, 3.404]** | −0.501 / **−0.091** / 2.459 | **776 / 776** | −0.179 | 288 / 44260 | 3861 / 9712 |
| `bg_hyb_guard` | hybrid + global:=scan（−1.0） | on | **779.5 / 779.5**（100%） | **[−0.636, 5.086]** | 0.020 / **+0.036** / 0.068 | **779.5 / 779.5** | −0.179 | 267 / 42442 | 3866.5 / 9693.5 |
| `bg_hyb_noguard` | hybrid + global:=scan（−1.0） | **off** | —（无守卫） | — | —（探针 z p50 +0.037） | — | — | 307 / 42343 | 3859 / 9717 |
| `bg_global_noband` | hybrid + global:=scan（**0.0 scratch**） | on | **780 / 780**（100%） | **[−0.635, 5.086]** | 0.023 / **+0.036** / 0.069 | **0 / 780**（一帧 **0/774**） | **−0.179** | 266 / 42442 | **3713 / 10176**（恒定） |

> 读法：`bg_off_guard2` 的 `333/776` 与 §15 的 `333/774` 是同一帧族（有限波束逐帧
> 768–786）；**比例 42.9% vs 43.0%**、kept-y 区间同向同量级、z p50 −0.091 vs §13.8 −0.0919。
> `bg_hyb_guard` 的局部 100% / 全局 100% 与 §13.8/§15.9 的 776/776、774/774 同档。
> `bg_global_noband` 的 global 3713/16886/10176 **逐帧恒定**，正是 §15.1 的"哑图"指纹。

### 17.9 不改行为：栈内 A/B（同一栈、同一机器人位姿）与分离跑对照

**栈内 A/B**（`bg_within_ab`）：launch 用 `band_guard:=False`（**launch 的节点集不含守卫**），
先跑探针 A（`settle 20 / window 15`，无守卫），然后**手工** `ros2 run … band_guard.py`，
紧接着跑探针 B（`settle 2 / window 15`，守卫在跑）。中间没有重启栈，机器人静止、位姿连续：

| 量（15 s 窗口，中位数） | A（无守卫） | B（守卫在跑） | Δ |
|---|---|---|---|
| local lethal | **266** | **265** | **−1** |
| local inscribed / free | 9546 / 42415 | 9521 / 42438 | −25 / +23 |
| local nearest lethal / ahead | 0.70505 / 25 | 0.70499 / 24 | 0 / −1 |
| global lethal | **3898** | **3897** | **−1** |
| global inscribed / free | 17194 / 9693 | 17181 / 9721 | −13 / +28 |
| global nearest / ahead | 0.457455 / 89 | 0.457467 / 89 | ~0 / 0 |
| `/scan` 有限波束中位数 | 775 | 775.5 | +0.5 |

⇒ 守卫在跑与不在跑的**相邻窗口**差 **1 个 lethal 格**、`free` 差 ≤0.07%；没有可测的
数据/控制路径影响。B 窗口内守卫日志是 local/global **780/780 OK**（17.6 同款）。

**分离跑对照的诚实说明**：`bg_hyb_guard`（守卫 on）与 `bg_hyb_noguard`（守卫 off）的
local lethal 是 **267 vs 307**，差的 40 格**不是**守卫造成的 —— 两跑的机器人 odom 位姿差
**7 mm**（x 0.0179 vs 0.0250 m；local lethal 对 0.02 m 栅格的亚格位移很敏感），而
**守卫 on 的另两跑**（R2/R4）local lethal 是 267/266，与栈内 A/B 的 266/265 一致；
全局 lethal 在 R2/R3 只差 7.5 格、栈内 A/B 差 1 格。⇒ 分离跑的局部差归因于**逐跑位姿/时序**，
不是守卫；这也是为什么验收 5 用**栈内 A/B** 作为主证据、分离跑只作旁证。

**其它默认路径证据**：17.4 的 67/69 Action 纯增量、R3 `ros2 node list` 无 `/band_guard`、
三跑合并 nav2 YAML md5 全等 `e5179ab…`。**没有**修改 `src/rm_navigation/**` 或任何 nav2
参数文件；**没有**发布/改写 scan、costmap、TF、cmd_vel。

### 17.10 本轮没能验证 / 诚实边界

1. **"333/774"不是逐字复现的单一帧**：守卫 5 s 采一帧，本机 `/scan` 有限回波逐帧
   768–786；off 跑拿到 **333/772**（一帧）、9 帧中位 **333/776**；§15 记录的 333/774 是
   探针的另一帧。**比例 42.9–43.2% vs 43.0%**、kept-y 同向、z p50 同值；没有把某一帧
   的 774 硬凑出来。global 旧配置倒是拿到**逐字 0/774** 一帧。
2. **只测了一个组合**：`world:=RMUL2026`、`robot:=robot11 robot11_mount:=sensor`、
   `lio:=small_point_lio`、`localization:=gicp`、`nav:=rpp`、`planner:=navfn`、
   `local_obstacle:=scan`、`global_obstacle:=scan`。**未测**：`amcl`/`beluga`/`small_gicp`、
   `fastlio`/`pointlio`、`mode:=slam_nav`、`global_obstacle:=stvl` 的守卫 idle 日志（代码会
   `idle`，但没有跑出日志证据）、`RMUC2026`、真机/真实时间。
3. **TF 的 stamp vs latest**：绝大多数样本 `mode=stamp`；每跑开头有 1 条 `mode=latest`
   （scan 戳早于 TF 缓存最早数据）。nav2 用 stamp；守卫退回 latest 只影响这一帧的
   z/kept-y，日志里明确标了 `mode=latest`。**未**量化这一帧与 nav2 的差。
4. **参数服务扰动的定量**：§15.11 记录过 `ros2 param get` 子进程风暴会扰动 MessageFilter；
   本实现是进程内异步 + 每 30 s 一次，栈内 A/B 没看到差异；但**没有**做"服务风暴"压测。
5. **动态改参**：`params_refresh_s=30` 才会看到 `ros2 param set`；本轮 R4 用**启动时**
   的 scratch 覆盖，没有测"运行中把 ×0.0 改回来"的 30 s 延迟。
6. **状态话题没有进 rosbag**：只用 `.tmp_band_guard/status_recorder.py`（transient_local
   订阅）记 JSONL；没有 `ros2 bag record`，也没有长跑（每跑 ~1–2 min 仿真）。
7. **限流/刷屏**：`period=5 s`、`rate_limit=5 s`、正常帧也打 INFO ⇒ 每 costmap 每 5 s 一行
   （每行 ~700 字符）。没有测"参数频繁抖动/级别频繁升降"时的日志量。
8. **`mode:=mapping`**：守卫 condition 明确不启动；没有跑 `mode:=mapping band_guard:=True`
   去证明"确实不启动"（代码/condition 可读，但没实测）。
9. **"默认 Action 树逐字节相同"的严格说法不成立**：实测是**纯增量**（67 个既有 Action
   逐字段相同、0 删/改、顺序不变；新增 launch arg + 恒假 Node 两个 Action）。`band_guard`
   默认不执行、节点集/参数文件/控制路径不变；但若要求"Action 树 0 新增"，做不到 ——
   除非改既有 Node/参数（那才是行为变化）。这条按实测写，不按口号写。
10. **日志行很长**：所有可执行字段在一行（frame、两道闸、scan frame、TF、z、kept-y、
    hint），便于 grep 但不利于人眼；没有做多行/JSON 双格式。
11. **`finite` 口径**：守卫按 nav2 严格 `< range_max`，既往探针用 `<= range_max` ⇒ 两边
    可能差 0–1 条；本节的 `finite_*` 是守卫自己的口径，跨文档比较时看比例而非绝对条数。

## §18 第十轮（**本机 date=2026-10-20T01:10–01:20**）：`gravity_align_mode` 默认 = `auto`（决策 A）—— 默认路径逐字节不变，`lio:=small_point_lio` 默认就重力对齐

> **本节只新增**（append-only）：§1–§17 一个字都没改。写作时 §17 是最新 ⇒ 本节取 §18
> （§18 落笔时仍空闲）。规范侧在 **`docs/frame_contract_v3.md` §4.10**（决策/语义/生效条件/回滚/边界）。
>
> **本轮没有改任何二进制**（`build/small_point_lio/libsmall_point_lio.so` md5 `58bebfb8…` =
> §10 的 `zfix`，跑前跑后一致），没有改 `src/rm_localization/**`、`src/rm_simulation/**`、
> 地图/PCD 资产、任何 YAML。改的只有
> ① `src/rm_nav_bringup/launch/bringup_sim.launch.py`；② 契约 §4.10；③ 本节。
> 隔离口径与 §17 逐字相同：每跑私有 `GAZEBO_MASTER_URI` / `HOME` / `ROS_DOMAIN_ID`、
> `gui:=False`、`nav_rviz:=False lio_rviz:=False`、`setsid` + `timeout` +
> `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）。

### 18.1 一句话结论

> 把 `gravity_align_mode` 的默认值从 `off` 改成 **`auto`**（`auto = hybrid 当 lio==small_point_lio，
> 否则 off`）之后：**默认（`lio:=fastlio`）路径的节点集合 / 每个节点的参数 / Action 顺序 /
> `robot_description` 与改动前逐字段相同**（唯一新增 = 一行 t=0 收据），
> 而 **`lio:=small_point_lio` 默认就重力对齐**（本机 1 跑：in-band **776/776**、倾角 **0.442°**、
> local lethal **277**、`odom→base_link` rpy **[−0.04, −0.25, −0.22]°**、`h_meas` **0.2616 m INFO**、
> 节点参数回读 **`gravity_align_mode=hybrid`** ⇒ `auto` 没有泄漏到下游）。
> 显式 `off` 逐字节回退（11 节点 / 11 份 param dump 全等）；显式 `hybrid` + `lio:=fastlio`
> 仍在 t=0 **逐字硬报错**。

### 18.2 改了什么（launch 一处改动、六个消费方 + 一行收据）

**为什么不能把默认值直接写成 `hybrid`**（本节的主约束）：launch 的 `lio` 默认是 `fastlio`，而
`_check_gravity_align_contract` 对「**显式**非 off + 非 `small_point_lio`」是**硬报错**（E1 补丁
只在 vendored `small_point_lio` 里）⇒ 默认写 `hybrid` 会让**每一个默认 launch 在 t=0 停**
（§18.6 (d) 就是这条报错的逐字证据）。`auto` 把"默认（fastlio）不变"与"SPL 默认对齐"同时保住。

| # | 文件:行（本提交） | 改动 |
|---|---|---|
| 1 | `bringup_sim.launch.py:823` | `_GRAVITY_ALIGN_MODES = ('off','urdf','imu','hybrid','auto')`（注释写明 auto 是解析档） |
| 2 | `:826` / `:847` | **唯一解析点**：`_effective_gravity_align_mode(context)` + substitution `_GravityAlignEffectiveMode`（`auto` ⇒ `hybrid`/`off`；显式值原样；`auto` 绝不返回） |
| 3 | `:862` / `:876` | 收据：`_gravity_align_resolution_msg` + `_GravityAlignResolution`（`LogInfo` 的消息体） |
| 4 | `:2120`–`:2139` | `DeclareLaunchArgument('gravity_align_mode')`：默认 `off` → **`auto`**；choices 加 `auto`；description 写明 `auto = hybrid 当 lio==small_point_lio，否则 off` |
| 5 | `:2154` | `gravity_align_z_origin` 的 description：`gravity_align_mode != off` → **有效值 != off**（文案澄清，无行为变化） |
| 6 | `:223` | `_check_gravity_align_contract()` 内读的 `mode` = **有效值**（`auto`+非 SPL 直接返回；显式非 off + 非 SPL 仍逐字硬报错） |
| 7 | `:2680` | 默认档 SPL Node 的 condition：`gravity_align_mode == 'off'` → **有效值 == off** |
| 8 | `:2699` / `:2713` | 统一输出契约档 SPL Node 的 condition（`!= off`）**与其 `gravity_align_mode` 参数** = 有效值 |
| 9 | `:3037` | `lio_tf_adapter_spl_node` 的 condition = 有效值 |
| 10 | `:3520` | 新增 `LogInfo`：`gravity_align_mode == 'auto'` 时 t=0 打一行收据（显式值不打 ⇒ 显式路径的 Action 集不变） |
| 11 | `:3536` | 组合检查 `GroupAction` 的 condition = 有效值 |

**收据长这样**（本轮两跑逐字）：

```text
# 默认（lio:=fastlio）
[INFO] [launch.user]: [gravity_align] gravity_align_mode=auto ⇒ 有效值=off（lio=fastlio；E1 补丁只存在于 small_point_lio）
# lio:=small_point_lio
[INFO] [launch.user]: [gravity_align] gravity_align_mode=auto ⇒ 有效值=hybrid（lio=small_point_lio）
```

### 18.3 静态证明：默认路径（`lio:=fastlio`）逐字节不变

#### 18.3.1 静态 Action 树 diff（`tools/scripts/gzsim/launch_increment_diff.py`）

```bash
git show HEAD:src/rm_nav_bringup/launch/bringup_sim.launch.py > .tmp_ga/old/bringup_sim.launch.py
ROS_LOG_DIR=/tmp/ga_roslog python3 tools/scripts/gzsim/launch_increment_diff.py \
  --old .tmp_ga/old/bringup_sim.launch.py --new src/rm_nav_bringup/launch/bringup_sim.launch.py
```

```text
actions: OLD 69 / NEW 70；共有 64
新增（NEW − OLD，整棵子树逐字段）：6
  + DeclareLaunchArgument name=gravity_align_mode
  + DeclareLaunchArgument name=gravity_align_z_origin
  + LogInfo cond=IfCondition
  + GroupAction cond=IfCondition
  + GroupAction
  + Node cond=IfCondition
消失/变成另一份（OLD − NEW）：5
  - DeclareLaunchArgument name=gravity_align_mode
  - DeclareLaunchArgument name=gravity_align_z_origin
  - GroupAction cond=IfCondition
  - GroupAction
  - Node cond=IfCondition
共有 Action 的相对顺序：一致 ✅
```

**逐项归类**（这 5 个"变成另一份"都不在默认路径的执行集里，或只是 launch 声明元数据）：

| 变动项 | 是什么 | 默认路径会不会执行 |
|---|---|---|
| `DeclareLaunchArgument(gravity_align_mode)` | **本任务的目标**：默认 `off`→`auto`、choices +`auto`、description | 只影响 `--show-args` 元数据 |
| `DeclareLaunchArgument(gravity_align_z_origin)` | **纯文案**（`gravity_align_mode != off` → 有效值 != off） | 只影响 `--show-args` 元数据 |
| `Node cond=IfCondition` | `lio_tf_adapter_spl_node` 的 condition 换成有效档 | **不执行**（`lio:=fastlio`） |
| `GroupAction`（`bringup_LIO_group`） | 内含 SPL 组：两个 SPL Node 的 condition / `gravity_align_mode` 参数换成有效档 | **不执行**（`lio:=fastlio`） |
| `GroupAction cond=IfCondition` | 组合检查组：condition 换成有效档 | **不执行**（有效值 off） |
| `LogInfo`（新增） | `auto` 的 t=0 收据 | **执行**（= 唯一新增的执行项） |

`--show-args` 的可见差异只有 `gravity_align_mode` 自己：

```text
'gravity_align_mode': ... auto（默认）= hybrid 当 lio==small_point_lio，否则 off ...
   Valid choices are: ['off', 'urdf', 'imu', 'hybrid', 'auto']   (default: 'auto')
```

#### 18.3.2 执行集（visited）diff：**0 changed / 0 removed**

静态 diff 把"condition 的 substitution 换了"也显示成"改了一份"，**即使那扇门在默认参数下根本
不会打开**（`lio != small_point_lio`）。所以补一个**按 launch 的 visit 规则**求值的对照
（`.tmp_ga/visited_diff.py`：按执行顺序递归，condition 假 ⇒ 整棵子树跳过；Node 的参数按**求值后的
值**比较；另抽 `robot_description` 的 xacro 命令子树算 md5）：

| 参数组（OLD=HEAD 快照 / NEW=本提交） | process 执行集 | log | opaque | robot_description md5 |
|---|---|---|---|---|
| **默认**（不传任何参数） | **7 / 7，0 新增 / 0 删除**，md5 `27516842…` 相同 | +1（收据） | 2 / 2 | `bb73e46a…` 相同 |
| 默认 lio + **整套 nav**（`mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor band_guard:=True global_obstacle:=scan`） | **8 / 8，0 / 0**，md5 `28f51ed6…` 相同 | +1 | 2 / 2 | 相同 |
| 显式 `off` + `lio:=small_point_lio` | **6 / 6，0 / 0**，md5 `b9bd7eb5…` 相同 | 2 / 2（**无新增**） | 2 / 2 | 相同 |
| 显式 `hybrid` + `lio:=small_point_lio` | **7 / 7，0 / 0**，md5 `a4e6ffcc…` 相同 | 2 / 2 | 3 / 3 | 相同 |
| **OLD 显式 `hybrid` vs NEW `auto`**（都 `lio:=small_point_lio`） | **7 / 7，0 / 0**，md5 `a4e6ffcc…` 相同 | +1（收据） | 3 / 3 | 相同 |

> 最后一行是决策 A 的**等价性证明**：`auto`（新）与用户显式写 `hybrid`（旧）在 `lio:=small_point_lio`
> 上的**执行集逐字段相同**（`auto` 只多一行收据）。
> 前两行 = 默认路径：**执行项只多一行日志**；`--arg` 的声明元数据差异见 18.3.1。

#### 18.3.3 运行期：`ros2 node list` / `param dump` / topic / service（old vs new）

两跑逐字相同配方：`ros2 launch <launch> gui:=False`（**不传 mode / robot / lio / gravity_align_mode**），
节点集稳定后（连续两次采样相同）dump。旧 = HEAD 快照（md5 `bcaf3839…`），新 = 本提交。

| 量 | OLD（HEAD） | NEW（本提交） | 判决 |
|---|---|---|---|
| `ros2 node list`（剔除 `transform_listener_impl_*` 这类名字带进程哈希的内部 listener） | 12 | 12 | **逐行相同 ✅** |
| launch 起的节点 | `/complementary_filter_gain_node /fake_vel_transform /gazebo /ground_segmentation /imu_plugin /joint_state_publisher /laser_mapping /lio_tf_adapter /livox_frame_plugin /mecanum_controller /pointcloud_to_laserscan /robot_state_publisher` | 同左 | 相同 |
| `ros2 param dump`（逐节点） | 12 份 | 12 份 | **逐文件 byte-diff 0；拼接 md5 `1a44e04636b16ae6b079975d27135971` 相同 ✅** |
| `ros2 topic list` | 31 | 31 | 相同 ✅ |
| `ros2 service list` | 84 | 84 | 相同 ✅ |

原始输出：`.tmp_ga/ga_def_{old,new}/{nodes,topics,services}.txt` + `params/`；
汇总 `.tmp_ga/default_identity.txt`。

### 18.4 (b) `lio:=small_point_lio` + 默认 `auto` ⇒ hybrid：nav 1 跑

**命令**（唯一要点：**不传 `gravity_align_mode`**，让它走默认 `auto`）：

```bash
SETTLE=20 WINDOW=20 PROBES="navband grav points" \
  bash tools/scripts/tiltmount/spl_ab_bench.sh ga_auto_spl zfix \
  world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp robot:=robot11 \
  robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
# 探针口径：navband settle 20 / window 20（仿真 s）；grav 5 帧（settle 20）；uc_procstats 20/20
```

**先证明"默认真的落到了 hybrid"**（launch 日志 + 参数回读）：

```text
[INFO] [launch.user]: [gravity_align] gravity_align_mode=auto ⇒ 有效值=hybrid（lio=small_point_lio）
[INFO] [launch.user]: [gravity_align] mode=hybrid ⇒ 统一输出契约启用：LIO publish_tf:=false +
  odom_convention:=sensor、lio_tf_adapter 成为 odom→base_link 的唯一发布者；gravity_align_rpy
  （来源=URDF-xacro.process_file）= [-30.000000, 0.000000, 0.000000]°（robot='robot11'
  mount='sensor' axis='roll'）；z_origin=ground gravity_align_h=URDF-xacro.process_file → 0.259527 m
[small_point_lio]: gravity_align_init_hybrid: Δ(URDF,IMU)=0.002° < 2° ⇒ 用 URDF 先验
[small_point_lio]: gravity_align_init: mode=hybrid src=urdf R_world_body rpy(deg)=[-30.000 0.000 0.000]
  Δ(URDF,IMU)=0.002° world_gravity=[0.000 -0.000 -9.810] body_source=imu_mean200
[small_point_lio]: gravity_align_z: z_origin=ground h(URDF)=0.259527 m lever_z=(R_ws·t_IL).z=0.043301 m
  ⇒ state_z=0.216226 m（odom 的 z=0 = 初始化时刻的地面；初始地图点用同一平移）
[small_point_lio]: gravity_align_h_check: h_urdf=0.2595 m h_meas=0.2616 m |Δ|=0.0020 m < 0.03 ⇒ INFO
  （inliers=14434 tilt=0.07° resid_p50=0.0016 p95=0.0045）
```

```text
/small_point_lio publish_tf              = Boolean value is: False
/small_point_lio odom_convention         = String value is: sensor
/small_point_lio gravity_align_mode      = String value is: hybrid        ← ★ auto 没有泄漏
/small_point_lio gravity_align_rpy       = Double values are: array('d', [-30.0, 0.0, 0.0])
/small_point_lio gravity_align_z_origin  = String value is: ground
/small_point_lio gravity_align_h         = Double value is: 0.25952717
/lio_tf_adapter  odom_topic              = String value is: /odom
```

**主表**（`navband` 200 帧 / `band_guard` 11 个 5 s 样本；单跑 n=1）：

| 指标 | 本轮 `auto ⇒ hybrid` | 对照（§17.8 同配方、显式 hybrid / off） |
|---|---|---|
| `/scan` 有限波束/帧 | **776**（min 755 / max 789） | 775–776 |
| local 在带 `[0,2]` | **776 / 776（100%）**；`band_guard` **11 样本全部 100%**（772/772×2、775、777、778、779、780、783/783×4） | hybrid 779.5/779.5；off **333/776** |
| local kept-y | **[−0.635, 5.090] m，`one-sided=no`** | hybrid [−0.636, 5.086]；off [0.268, 3.404] 单侧 +y |
| local z min/p50/max | **0.020 / 0.037 / 0.076 m** | hybrid 0.020/+0.036/0.068 |
| local **lethal** | **277** | hybrid 267；off 288 |
| local inscribed / free | 9496.5 / 42386.5 | hybrid 9448/42442；off 8896.5/44260 |
| local **最近 lethal** | **0.7215 m** | hybrid 0.703；off 0.755 |
| local **前方 2.0×1.2 m lethal** | **21** | hybrid 28；off 17 |
| 车 1 m 内 lethal | 26 | hybrid 34；off 25 |
| global lethal/inscribed/free | 3725 / 16875 / 10175 | off（§13.8 B）3713/16886/10176；**两档是同一张全局图**（全局层不吃局部高度带，见 §4.10.4 第 3 条） |
| **地面残余倾角**（原始云→TF→世界系拟合，主指标） | **0.442°**（5 帧 [0.419–0.532]；`ang_global` 0.376°） | hybrid 0.518°（§10.4）/0.346–0.488°（§3.1）；off **30.115°** |
| 点云自身帧里的地面倾角 | **29.993°**（= 安装角） | — |
| 地面在 `base_link` 里 | **0.0156°**（⇒ 真的平了） | — |
| `odom→base_link` rpy（probe） | **[−0.044, −0.250, −0.216]°** | hybrid [0.30…0.43, −0.21…−0.38, −0.04…+0.05]；off [30.06…30.20, …] |
| 同上（窗口末端 `tf2_echo`） | [−0.02…−0.12, −0.385…−0.393, −0.26…−0.28]° | — |
| `h_meas` | **0.2616 m / Δ=0.0020 m ⇒ INFO** | 0.2603/0.2615（§13.9） |
| ms/帧（20 s 窗口） | **16.15**（单跑；同轮 A/B 见 §10.5 的 13.90 vs 15.43，**不可跨轮比**） | hybrid 13.90 / off 15.43（§10.5） |
| 初始化 | 2.10 仿真 s / 12.14 墙钟 s | 2.10–2.20 / 11.67–12.84 |
| adapter CPU / RTF | 0.183% / 0.366 | 0.209% / — |

**read-out（契约侧）**：`/odom` publisher **1**、`/cloud_registered` 1、`/scan` 1、`/tf` 5、
`/tf_static` 1、`/local_costmap/costmap_raw` 1；`/Odometry` **0**（已被 remap 成 `/odom`）。
⇒ 统一输出契约（选项 U）在默认档下**真的生效**，`odom→base_link` 只有 adapter 一个发布者。

原始数据：`.tmp_ab/ga_auto_spl/{launch.log,probe.json,navband.json,points.json,proc.json,params.txt,publishers.txt}`。

### 18.5 (c) 显式 `off` 回退：逐字节

**静态**：18.3.2 表的第 3 行 —— `lio:=small_point_lio gravity_align_mode:=off` 的执行集
**6 / 6，0 新增 / 0 删除 / 0 改动**，process md5 `b9bd7eb5…` 与 HEAD 快照完全相同。

**运行期**（同一配方，两跑只换 launch 文件：HEAD 快照 vs 本提交）：

```bash
ros2 launch <launch> gui:=False lio:=small_point_lio gravity_align_mode:=off \
  robot:=robot11 robot11_mount:=sensor
```

| 量 | OLD（HEAD `bcaf3839…`） | NEW（本提交） | 判决 |
|---|---|---|---|
| `ros2 node list`（剔除内部 listener） | 11 | 11 | **逐行相同 ✅**（`/small_point_lio` 在，`/lio_tf_adapter` **不在**） |
| `ros2 param dump` | 11 份 | 11 份 | **逐文件 byte-diff 0；拼接 md5 `64e97e9dbd20957b88e6a03e7df142be` 相同 ✅** |
| `ros2 topic list` / `service list` | 27 / 78 | 27 / 78 | 相同 ✅ |
| `gravity_align_mode=auto ⇒` 收据 | 0 行 | **0 行** | 显式 off **不新增任何日志 Action** ✅ |
| E1/E1b 日志（`gravity_align_init` / `z` / `h_check`） | 0 行 | 0 行 | off 档行为不变 ✅ |

原始数据：`.tmp_ga/ga_off_spl_{old,new}/` + 汇总 `.tmp_ga/off_spl_identity.txt`。

### 18.6 (d) 显式 `hybrid` + `lio:=fastlio`：仍硬报错（逐字）

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py gui:=False lio:=fastlio gravity_align_mode:=hybrid
# exit code 1；gzserver/robot_state_publisher/... 一个都没起（日志里 0 条进程启动行）
```

```text
[ERROR] [launch]: Caught exception in launch (see debug for traceback): [launch] gravity_align_mode:=hybrid 只对 lio:=small_point_lio 生效（E1 初始化对齐的补丁在 small_point_lio 包里；fastlio/pointlio/cartographer 没有 gravity_align_mode / gravity_align_rpy 这些参数，传了也不会生效）⇒ 当前 lio:=fastlio：请改成 lio:=small_point_lio，或把 gravity_align_mode 设回 off
```

⇒ **文案与改动前逐字相同**（`auto` 只是不触发它，没有改写它）。

### 18.7 组合矩阵：`auto` 永不放错，显式值语义不变

`.tmp_ga/contract_matrix.py` 直接调用 launch 里的**同一个** `_check_gravity_align_contract()`，
对 5 个 lio × 5 个 mode 共 **25 格**求值：

| lio \ mode | `auto`（默认） | `off` | `urdf` | `imu` | `hybrid` |
|---|---|---|---|---|---|
| `fastlio` | **放行（有效值 off）** | 放行 | 硬报错 | 硬报错 | 硬报错 |
| `pointlio` | **放行（off）** | 放行 | 硬报错 | 硬报错 | 硬报错 |
| `small_point_lio` | **放行（hybrid）** | 放行 | 放行 | 放行 | 放行 |
| `none` | **放行（off）** | 放行 | 硬报错 | 硬报错 | 硬报错 |
| `cartographer` | **放行（off）** | 放行 | 硬报错 | 硬报错 | 硬报错 |

⇒ `auto` 在**所有** lio 上都放行（这正是默认值不能是 `hybrid` 的原因）；显式 `urdf|imu|hybrid`
+ 非 SPL 的**硬报错语义一格没变**。收据字符串也逐格核对（`… ⇒ 有效值=off（lio=<非 SPL>；
E1 补丁只存在于 small_point_lio）`）。

### 18.8 本轮没能验证 / 诚实边界

1. **`auto` 的实跑只有 1 跑**（`world:=RMUL2026 mode:=nav localization:=gicp robot:=robot11
   robot11_mount:=sensor spin_speed:=0.0 band_guard:=True`）。它是**存在性/口径**证据，**不是**
   精度/性能 A/B；A/B 仍引用 §10/§13/§17 的多跑表。
2. **默认路径的运行期对照是 `mode:=''`（launch 默认）**，节点 12 个（无 nav2）。**带全套 nav2 的
   默认 lio** 没跑运行期对照，只有 18.3.2 的执行集静态对照（8 个 process Action、md5 相同）。
3. **只测 `robot:=robot11` + `robot11_mount:=sensor`**；默认模型 / `hzmirm` / `plugin|urdf` 没跑。
4. **`hzmirm` 的 `turret≠0`** 只做 launch 求值；**3–5 cm WARN 带**没构造出真实数据（§10.9 第 6 条）。
5. **`small_point_lio` + 默认模型（`R_bl=I`）的 SIGSEGV** 是独立缺陷（§16.9），本轮**没有**碰；
   它意味着 `auto` 的收益目前只在 `R_bl≠I` 的模型上可见。
6. **`ros2 node list` 里的 `transform_listener_impl_*`** 是 tf2 内部 listener（名字带进程哈希），
   两跑必然不同 ⇒ 已从对照里剔除并在表里注明；它们是 **launch 之外的节点**，不属于本次验收口径。
7. **`spl_ab_bench.sh` 收尾会把 `build/` 的二进制还原成 `.tmp_uc/bins/after`（`zfix_prev`）**，
   与本轮开始时的 `zfix`（`58bebfb8…`）不同 ⇒ 收尾后**已手工还原成 `zfix`** 并复核 md5
   （`build/small_point_lio/libsmall_point_lio.so` = `58bebfb81537978a0c93067be09de1be`）。
   本轮所有数字都在 `zfix` 上取得（`meta.txt` 记 `bin_lib_small_point_lio_md5`）。

## §19 第十一轮（**本机 date=2026-10-20T02:00–03:05**）：两条独立缺陷 —— ④ `map_archive.sh save --resolution` 的**静默失败**（含调用方吞退出码）、② `lio:=small_point_lio` + 默认模型槽位的 **SIGSEGV**

> **本节只新增**（append-only）：§1–§18 一个字都没改。写作时 §18 是最新 ⇒ 本节取 §19
> （§19 落笔时仍空闲）。两条缺陷**各自一个提交**（④ 先、② 后），本节按同一顺序记。
> ⚠️ 本轮有**并发 agent** 在同一份文档上写（他后来取了 §20，插在 §19.2 与 §19.3 之间写作；
> 本节的 §19.3 已在提交前移回 §19 内部，顺序为 §19 → §20；两节内容互不引用、互不覆盖）。
> 隔离口径与 §17/§18 逐字相同：每跑私有 `GAZEBO_MASTER_URI` / `HOME` / `ROS_DOMAIN_ID`、
> `gui:=False`、`nav_rviz:=False lio_rviz:=False`、`setsid` + `timeout` +
> `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）。

### 19.1 一句话结论

> **④**：`save --resolution R` 现在**接受 + 校验 + 真的用**（写进 sidecar 的 `resolution:`，并且
> 贯穿守卫 check），**调用方**也不再吞退出码（`| tee` 后取 `${PIPESTATUS[0]}`，非 0 ⇒ 调用方非 0）。
> 实测：`--resolution 0.05` ⇒ sidecar `resolution: 0.05`、`--resolution 0.07` ⇒ `0.07`（证明是**用**
> 了显式值而不是 mapper 的 0.05），坏参数（`banana`）⇒ `save` `rc=2` 且**一个文件都没写**，
> 调用方 `run_remap_lowterrain.sh --map-resolution banana` ⇒ **`rc=2`**（修前是 `rc=0`，看着成功）。
>
> **②**：报告里"默认模型 + `lio:=small_point_lio`，~12 s 后 SIGSEGV"在当前 HEAD 上**复现不出来**
> （**0/53 跑**，全程挂 gdb）⇒ **那一次的 backtrace 没有拿到**，根因**未定位**（§19.3.6）。
> 本轮修的是同一类"**退化帧 / 退化参数**"里**确定性可复现**的三条：
> ① 缺 `gravity`/`extrinsic_*` 键 ⇒ 旧代码 `std::terminate`（**SIGABRT**，有逐字栈）；
> ② 短列表 ⇒ 静默越界读；③ `base_link→livox_frame` 缺失/退化 ⇒ **`/odom` 一条都不发**
> （= "没有 /odom"的机制；修后限速 ERROR + 单位阵回退，实测 `/Odometry_noframe` **8.141 Hz**）；
> ④ `lio_tf_adapter` 的 `xyz`/`rpy` 长度 ⇒ 潜在 SIGSEGV（`std::vector::operator[]` 越界）。
> 回归：默认模型 **75 s 无 crash、`/odom` 8.269 Hz**；`robot11+sensor` **70 s 无 crash**、
> `ros2 param dump` **逐字节相同**（`869aa903…`）、`h_meas` 0.2618 vs 修前 0.2615。详见 §19.3。

### 19.2 ④ `map_archive.sh save --resolution` + 调用方退出码（`tools/scripts/**`，不碰 ROS 代码）

**缺陷形态**（2026-10-19 建图轮实际撞到，§16.4 坑 4 只记了现象、没修）：
`map_archive.sh save --resolution 0.05` 打 usage + `exit 2`（`--resolution` 不在参数表里），
而唯一的调用方 `tools/scripts/regress/run_remap_lowterrain.sh` 写的是
`… save --name "$MAP_NAME" 2>&1 | tee … || true` ⇒ **`|| true` 把整条管道的退出码吃掉**
（`set -o pipefail` 也救不了它）⇒ 建图跑完、看着成功、`map/<name>.{posegraph,data,meta.yaml}`
一个都没写。

| # | 文件:行（本提交） | 改动 |
|---|---|---|
| 1 | `tools/scripts/mapping/map_archive.sh:194-226` | 新增 `RES_MIN=0.005` / `RES_MAX=1.0` + `parse_resolution()`（python3 解析：非数 / 非有限 / ≤0 / 越界 ⇒ **stderr 说明 + exit 2**，成功回**规整值**）；`mapper_resolution()` 的回退 0.05 改为**带 WARN**（不再静默） |
| 2 | `:467` | 参数表新增 `--resolution R`（缺取值 ⇒ usage + exit 2；值经 `parse_resolution` 校验） |
| 3 | `:484-488` | `--resolution` 只对 `save` 生效；给其它子命令 ⇒ exit 2（不静默忽略） |
| 4 | `:279-288` | `cmd_save` 里**显式值优先**（原来无条件 `RESOLUTION="$(mapper_resolution)"` 会把显式值覆盖掉），并打印 `resolution=… （来源=--resolution/活 mapper）` |
| 5 | `:306` / `:379` | `--check` 的 ③ 行汇总与最终汇总都打印分辨率（**可核对**：sidecar 的 `resolution:` 就是它） |
| 6 | `tools/scripts/regress/run_remap_lowterrain.sh:141-152` | 存档那一步改成 `set +e; … | tee …; SAVE_RC=${PIPESTATUS[0]}`，非 0 ⇒ 打印原因并 **`exit "$SAVE_RC"`**；新增 `--map-resolution R`（`:34/:47`）透传给 `save --resolution` |
| 7 | 同上 `:16-19`、`map_archive.sh:9-19` | 文件头写明"调用方必须传退出码"（避免下一个人再写成 `| tee … || true`），并给 `--resolution` 的用法/边界 |

**实测（本机，隔离栈；`lio:=small_point_lio gravity_align_mode:=off mapper:=slam_toolbox
mode:=mapping world:=RMUC2026 map_name:d2_arch`，私有 DOMAIN=183 / port=11883）**：

| # | 命令 | `rc` | `map/d2_arch.meta.yaml` 的 `resolution` | 产物 |
|---|---|---|---|---|
| A | `save --name d2_arch --world RMUC2026 --resolution 0.05` | **0** | **0.05** | posegraph 5,007,625 B + data 51,474 B + sidecar |
| B | 同上但 `--resolution 0.07` | **0** | **0.07**（活 mapper 仍是 0.05 ⇒ 证明用的是显式值） | 同上（覆盖 + `*.prev-*` 备份各一代） |
| C | `save --name d2_arch_bad --world RMUC2026 --resolution banana` | **2** | —（**没有** `d2_arch_bad.*` 任何文件） | 无 |
| D | `save --name d2_arch --world RMUC2026`（不给 `--resolution`） | **0** | **0.05**（自动问活 mapper；与改动前一致） | 同上 |

**调用方端到端（真跑 `run_remap_lowterrain.sh`，私有 DOMAIN=184 / port=11884，`--timeout 5`）**：

| 跑 | 命令 | 修前 | 修后 |
|---|---|---|---|
| 坏参数 | `--map-name d4_caller --map-resolution banana` | `rc=0`（`|| true` 吞掉 save 的 2） | **`rc=2`**，且 `map/d4_caller.*` **一个都没有** |
| 正常 | `--map-name d4_caller --map-resolution 0.05` | `rc=0` | **`rc=0`**，`map/d4_caller.{posegraph,data,meta.yaml}` 齐全，sidecar `resolution: 0.05` |

> 口径说明：所有跑都用**测试专用** `map_name`（`d2_arch` / `d4_caller`），跑完已从
> `src/rm_nav_bringup/map|PCD/` 删除；用户资产（`map/RMUC2026*`、`PCD/RMUC2026*`）一个字节没动。

### 19.3 ② `lio:=small_point_lio` 的 SIGSEGV / "没有 /odom"：**复现失败** + 两条确定性死亡路径的修法

> **先说结论（诚实边界）**：本节**没有**拿到"默认模型 + `lio:=small_point_lio`，~12 s 后
> `exit code -11`"那一次崩溃的 backtrace —— 它在当前 HEAD 上**复现不出来**（`0/53` 跑，
> 见 19.3.1）。因此下面修的不是"猜出来的那个栈"，而是**同一类（退化帧 / 退化参数）里
> 确定性可复现、且真的会让节点死或让 `/odom` 整段消失**的三条路径；每一条都有修前/修后实测。
> 详见 19.3.5 的"没能验证"。

#### 19.3.1 复现尝试（全部带 gdb；`tools/scripts/lib/scoped_cleanup.sh` 隔离；无全机 pkill）

| 配置（`mode:=mapping lio:=small_point_lio mapper:=slam_toolbox` + 其余默认） | 跑数 | 单跑时长 | 结果 |
|---|---|---|---|
| `world:=RMUC2026`（= §16.9#2 写的复现命令，默认 `robot`） | 24 | 60 s | 全部存活，`/odom` 7.5–8.2 Hz |
| `world:=RMUL2026 robot:=robot11 robot11_mount:=sensor`（= 唯一有崩溃记录那次跑的参数） | 12+ | 90 s | 全部存活 |
| `world:=RMUC2026` + `/cloud_registered` 订阅者（点云发布分支被真正执行） | 9 | 60 s | 全部存活 |
| `robot:=hzmirm` / `robot:=robot11 robot11_mount:=plugin` / `world:=RMUL2026 spin_speed:=0.0` | 3+5 | 60 s | 全部存活 |
| **合计** | **53** | 60–120 s | **SIGSEGV 0 次**；`/odom` 一直有 |

跑法：把 `install/small_point_lio/lib/small_point_lio/small_point_lio_node` 临时换成
`exec gdb -batch -ex run -ex 'thread apply all bt' …` 的包装脚本（**launch 的参数/时序/数据一字不改**），
改完已还原成软链（`ptrace_scope=1` ⇒ 事后 attach 不允许，只能让 gdb 当父进程）。

**唯一留存的崩溃记录**（上一轮建图跑，`git_head=dafcb4e`，`src/rm_localization/small_point_lio/**`
在那之后**没有任何提交** ⇒ 代码与 HEAD 相同；记录在 `.tmp_maprebuild/v_wall_rmul/launch.log:91`）：

```text
[ERROR] [small_point_lio_node-8]: process has died [pid 130, exit code -11, cmd
 '/home/weicheng/HzMi_rmsimulation/install/small_point_lio/lib/small_point_lio/small_point_lio_node
  --ros-args -r __node:=small_point_lio --params-file …/config/mid360_sim_tuned.yaml
  --params-file /tmp/launch_params_c2yt6rm1 -r /Odometry:=/odom'].
```
它**死前最后一条**自己的日志是 10.7 s 时的外参 readout（`外参守卫(§N.5#1)：T_bl=T(livox_frame←base_link)
rpy=[30.000000, …]`），之后 23 s 无任何 ERROR/WARN ⇒ 不像断言/异常，更像运行期内存问题；
但 `core_pattern` 走 apport（本机 `/var/crash` 只读）⇒ **没有 core 文件**，gdb 又只能事后 attach
⇒ 那一跑已无法回溯。

#### 19.3.2 修掉的确定性死亡路径 ①：**退化参数**（缺键 ⇒ SIGABRT；短列表 ⇒ 静默越界读）

`parameters.cpp` 里 `gravity` / `extrinsic_T` / `extrinsic_R` 都是 `declare_parameter<T>(name)`
（**无默认值**）+ **固定下标** `[0]…[8]`。实测两条后果（真跑，不是推演）：

* **缺键**（参数文件少一个 `gravity:`）⇒ `declare_parameter` 抛 `ParameterTypeException`
  ⇒ 构造函数里 `std::terminate` ⇒ **SIGABRT**（不是可控报错）。逐字 backtrace（gdb）：
  ```text
  #5  std::terminate() () from /lib/x86_64-linux-gnu/libstdc++.so.6
  #6  __cxa_throw () from /lib/x86_64-linux-gnu/libstdc++.so.6
  #9  rclcpp::Node::declare_parameter<std::vector<double, …>> (name="gravity", …)
        at /opt/ros/humble/include/rclcpp/rclcpp/node_impl.hpp:231
  #10 small_point_lio::Parameters::read_parameters (this=…, node=…)
        at …/src/small_point_lio/src/small_point_lio/parameters.cpp:22
  #11 small_point_lio::SmallPointLio::SmallPointLio (…) at …/src/small_point_lio.cpp:14
  #13 small_point_lio::SmallPointLioNode::SmallPointLioNode (…) at …/src/small_point_lio_node.cpp:126
  #23 main (…) at …/build/small_point_lio/rclcpp_components/node_main_small_point_lio_node.cpp:55
  ```
* **短列表**（`extrinsic_R: [1,0,0]` / `extrinsic_T: [0.05]`）⇒ **节点正常启动**、读的是堆上邻居字节
  ⇒ 外参是垃圾（静默错，比崩溃更难查）。

**修法**（`src/small_point_lio/parameters.cpp:9-56`）：新 `read_fixed_vector()` —— 先查
`get_parameter_overrides()` 判断"这个键到底给没给"，再带默认值声明；**缺键 ⇒ WARN、长度不对 ⇒ ERROR**，
两种情况都回退到**本仓三份 config 的契约值**（`gravity=[0,0,-9.81]`、`extrinsic_T=[0,0,0.05]`、
`extrinsic_R=I`），**绝不越界、绝不 terminate**。实测：

| 输入 | 修前 | 修后 |
|---|---|---|
| 参数文件缺 `gravity` | **SIGABRT**（上面那段栈） | `WARN 参数 gravity **没给** … ⇒ 用本仓契约默认值（3 个数）` + 节点继续跑 |
| `extrinsic_R: [1,0,0]`（3 个） | 静默用垃圾外参 | `ERROR 参数 extrinsic_R 需要 9 个数，收到 3 个 ⇒ 用本仓契约默认值` |
| `extrinsic_T: [0.05]`（1 个） | 静默用垃圾外参 | `ERROR 参数 extrinsic_T 需要 3 个数，收到 1 个 ⇒ …` |
| 正常 `mid360_sim_tuned.yaml` | — | **0 条 ERROR/WARN**（行为不变；见 19.3.4 的逐字节 param dump） |

#### 19.3.3 修掉的确定性死亡路径 ②：**查不到 / 退化的 `base_link→livox_frame`**（= "没有 /odom"）

`small_point_lio_node.cpp` 的 odom 回调原来写：

```cpp
try { base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", time_msg); }
catch (tf2::TransformException &ex) { RCLCPP_ERROR(…); return; }        // ← 整帧 odom 被丢掉
```
即：那条静态 TF 晚到（实测 spawn 后 3~10 s 才有）/ 缺失 / 帧名写错时，`/Odometry` 与
`odom→base_link` **一条都不发**（下游只报"等 TF"）—— 这正是"**没有 /odom**"的机制。
**修法**（`small_point_lio_node.cpp:166-206` + 判据/日志在独立 TU `extrinsic_guard.cpp:180-232`）：

* 抛异常 ⇒ 限速 ERROR（1 条/5 s + 累计计数）+ **T_bl := 单位阵**，本帧照发（= "没有安装外参"）；
* 查到的变换**退化**（平移/四元数非有限，或四元数模长 ≤ 0.5，例如 `w=0` 的未初始化静态 TF）
  ⇒ 同样按单位阵 + 限速 ERROR（那种 T_bl 乘进 tf2 只会得到 NaN 位姿，比"没有外参"更糟）。
* TF 一恢复就回到正常路径（**无粘滞状态**）；判据/日志放独立 TU 的理由同 §O.2（本包 `-ffast-math`）。

实测（同一套跑里再起一个**第二个** `small_point_lio` 实例，`-p lidar_frame:=dummy_frame_no_such`
⇒ 永远查不到 TF，`/Odometry` remap 到 `/Odometry_noframe`）：

| | 修前（代码路径：`return`） | 修后（实测） |
|---|---|---|
| `/Odometry_noframe` 速率 | **0（一条都没有）** | **8.141 Hz**（14 s 窗口） |
| 日志 | 每帧一条 `Failed to lookup transform …` | 12 s 内**2 条**限速 ERROR（`TF 回退（2026-10-20 防御）…`），进程存活 |

#### 19.3.4 修掉的确定性死亡路径 ③：`lio_tf_adapter` 的 `xyz`/`rpy` 长度（唯一"能真的 SIGSEGV"的一处）

`lio_tf_adapter_node.cpp` 的 `odomCallback` 直接 `xyz_[0..2]` / `rpy_[0..2]`，而 `std::vector::operator[]`
**不查长度** ⇒ 参数给 0/1/2 个数时是**越界读堆内存**（轻则补偿量垃圾、重则 SIGSEGV）。
修法（`lio_tf_adapter_node.cpp:37-58`）：长度 ≠ 3 ⇒ **ERROR 一次 + 归零**（= 不做该项补偿），绝不越界。
实测：`-p xyz:=[0.1] -p rpy:=[0.0]` ⇒ 两条 ERROR、节点正常启动并广播 TF（修前是越界读）。

> **为什么把 ①②③ 归到"② 这个 SIGSEGV 任务"里**：三者都是"**退化/缺失的帧或外参**"这一类，
> 且 ①② 都能**让节点直接死掉/让 /odom 整段消失**。但它们**不是**那一跑的同一份栈
> （19.3.1 已把"那次 SIGSEGV"钉成**未复现**）—— 不许把它们当成"已定位那个 segfault"。

#### 19.3.5 回归（≥60 s，无 crash + `/odom` 速率 + `robot11+sensor` 不变）

`build/small_point_lio/libsmall_point_lio.so`：修前 `eb44ffa7…`（RelWithDebInfo）→ 修后
`f1f00edb…`（**Release**，= 仓库默认 `colcon_defaults.yaml`；另有 RelWithDebInfo 的 A/B 一跑，
数字一致）。所有跑 `gui:=False nav_rviz:=False lio_rviz:=False`，私有 DOMAIN/port。

| 配置 | 时长 | SIGSEGV | `/odom` 速率 | `gravity_align_*` / `h_meas` | param dump |
|---|---|---|---|---|---|
| **默认模型** `world:=RMUC2026 mode:=mapping lio:=small_point_lio`（修前） | 70 s | 无 | 7.359 Hz | `rpy=[0,0,0] h=0.235 z=0.185`；`h_meas=0.1007`（ERROR，>0.05 带） | `e4f02edb…`（73 行） |
| 同上（**修后**，Release） | **75 s** | **无** | **8.269 Hz** | `rpy=[0,0,0] h=0.235 z=0.185`；`h_meas=0.1012`（同一条 ERROR） | **`e4f02edb…`（逐字节相同）** |
| **`robot:=robot11 robot11_mount:=sensor`**（修前） | 70 s | 无 | 4.744 Hz | `rpy=[-30,0,0] h=0.259527 z=0.216226`；`h_meas=0.2615` tilt 0.06° | `869aa903…`（73 行） |
| 同上（**修后**，Release） | **70 s** | **无** | **4.706 Hz** | `rpy=[-30,0,0] h=0.259527 z=0.216226`；`h_meas=0.2618` tilt 0.08° | **`869aa903…`（逐字节相同）** |

* `h_meas` 修前/修后差 0.0003 m（0.2615→0.2618）/ 0.0005 m（0.1007→0.1012）—— 与 §18.4 的
  0.2616 m 同一量级（**地面拟合用的是初始帧点云，逐跑有 ~0.5 mm 抖动**），不是行为变化。
* **`robot11 + sensor` 的数值行为不变的证据 = 两份 `ros2 param dump` 逐字节相同**
  （`869aa903914f32ec69be97a86ccf33c6`，73 行；默认模型 `e4f02edb6908655f5d1c482b81a5e713`），
  且 `/odom` 速率 4.744 → 4.706 Hz（同一量级，负载噪声）。代码层面：新增分支只在
  "TF 缺失/退化" 或 "参数长度不对" 时执行 ⇒ 正常路径不进新分支。

#### 19.3.6 这一条**没能**验证的（别当已验证）

1. **那一跑 SIGSEGV 的 backtrace 拿不到**：53 跑未复现、无 core（apport/`/var/crash` 只读）、
   `ptrace_scope=1` 只能让 gdb 当父进程（已用包装脚本做到，但没崩就没栈）。
   ⇒ 下一步建议：**在上次崩溃那台机上**（同负载/同并发）用同一包装脚本重跑，或把 `core_pattern`
   改成写本地文件后重跑；拿到栈之前，"根因"仍是**未定位**。
2. `lio_tf_adapter` 那条越界读**没有**造出真实 SIGSEGV（glibc 最小 chunk 让 1~2 个 `double` 的
   越界仍在可用区内）—— 它是"**能**崩"的潜在缺陷，不是"**已崩**"的证据。
3. 默认模型的 `h_meas` 一直是 **ERROR 带**（0.10 vs URDF 0.235，|Δ|=0.13 m）：这是 §16.9#4 记过的
   老问题（默认模型 `cloud_frame=parent` 与 `frame_id=livox_frame` 不一致 ⇒ 地面拟合量到的是
   父 link 系的点），本轮**没有**碰，也没有把它当成本次回归的判据。


## §20 第十一轮（**本机 date=2026-10-20T02:00–03:0x**）：**默认模型（`robot:=''`）真的能原地转了** —— 单刚体底盘 + 轮子对地摩擦（角速度通道 4.3% → 95.4%）

> **本节只新增**（append-only）：§1–§19 一个字都没改。§19 已被另一个 agent 占用 ⇒ 本节用 §20。
>
> 触发 = 父任务原文：**"make the default robot model (`robot:=''`, the "hzmirm" slot) actually
> rotate —— 实测纯原地转 0.80 rad/s → 实际 0.025 rad/s（3%），前进正常（0.30 → 0.183 m/s）"**。
>
> 与 §16「坑 1」的关系：§16 把 3% / 0.183 m/s 这组数记在 `robot:=robot11` 名下，**实际那两跑
> 是默认模型槽位**（§16.4 的 recipe A 没有 `robot:=`）。本节按**默认模型**重测、定量归因并修掉；
> §16 那两行数字**作废**（见 §20.6）。
>
> 隔离（与 §16/§17 同口径）：私有 `HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、
> 按 tag 探测端口的 `GAZEBO_MASTER_URI`、`unset DISPLAY`、`gui:=False`、`world:=X` **不带**前导
> `--`、收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）。
> 原始数据：`.tmp_rigid/`（台架 SDF/生成物/`drive_*`）、`.tmp_robotslot/rb_{before,after}_*`
> （整栈探针）、`.tmp_wz/bench/out/rb_*`（台架变体矩阵，gitignored）。

### 20.1 一句话结论

> 默认模型的角速度通道有两个**串联**的限制项，**只修一个都不够**：
> ① **模型结构**：`libgazebo_ros_planar_move.so` 的 `Set{Linear,Angular}Vel` 把**同一个 (v, ω)
> 写给模型里的每一个 link**（`gazebo/physics/Model.cc:746-771`），对多刚体+关节树不自洽
> ⇒ 原 5 刚体/4 关节版本只执行 **4.3%**；
> ② **轮子对地摩擦**：塌成单刚体后轮子**不能再滚动**，原地转 = 4 个轮子同时**横向刮地**
> ⇒ μ=1 时只到 **14.1%**（与 `robot:=robot11` 的 16.5% 同量级，§M.3）。
> 两项都修（单刚体 + 轮子摩擦 0.02）⇒ **95.4%**（台架）/ **95.1%**（整栈、20 Hz 指令），nav 冒烟 PASS（§20.7），
> 前进同时从 75% 升到 **98.7%**（同一条摩擦损失的另一个面，**不是**独立回归 —— 见 §20.6）。
> **几何一个字节没动**（轮 r=0.06 / 轮心 z=0 / base_link 离地 0.06 / 雷达 `base_link+0.12,0,0.175`
> rpy=0 / `h=0.235` / 质量 10.21 kg / Izz 0.103945 逐项自证相等）。`robot:=robot11` 不受影响。

### 20.2 为什么"只塌刚体"不够（判决性台架，同一世界/出生点/命令）

台架 = 只起 gzserver + **一个模型** + 底盘插件（摘掉 LIO/nav2/雷达），探针以 100 Hz 发
`/cmd_vel_chassis`，真值 = `/odom_ground_truth`（= Gazebo `WorldPose`），执行率按**仿真时间**归一。
命令：纯自转 `wz=0.8 rad/s × 2 仿真秒`（请求 91.7°）、纯前进 `vx=0.30 m/s × 1.5 仿真秒`（请求 0.45 m）。

| 变体 | 自转 Δyaw | **自转执行率** | 前进位移 | **前进执行率** |
|---|---|---|---|---|
| 源模型（5 刚体 / 4 个 continuous 轮关节） | 3.93° | **4.3%** | 0.3642 m | 80.9% |
| 单刚体（4 轮关节 fixed，sdformat 合并）μ=1 | 12.93° | **14.1%** | 0.3618 m | 80.4% |
| 单刚体 + 轮子 μ=0.02（**本节的修法**） | 87.45° | **95.4%** | 0.3965 m | 88.1% |
| 单刚体 + 轮子 μ=0.001（上限对照） | 89.72° | **97.9%** | — | — |
| 同上但世界换 RMUC2026（μ=0.02） | 87.46° | **95.4%** | — | — |

**摩擦剂量-响应（单刚体、同一命令、RMUL2026）**—— 这条曲线说明"第二限制项"是连续的、不是阈值：

| 轮子 μ | 1 | 0.5 | 0.2 | 0.1 | 0.05 | 0.03 | 0.02 | 0.001 |
|---|---|---|---|---|---|---|---|---|
| 自转执行率 | 14.1% | 34.4% | 72.2% | 84.6% | 90.5% | 93.6% | **94.9%** | 97.9% |

> ⇒ 取 **μ=0.02**：过 90% 且留 ~5 点余量（μ=0.05 只有 90.5%，临界）。
> ⚠️ **被否掉的另一条路**：把插件 `<update_rate>` 100→1000 Hz（每个物理步都重设速度）也能到
> **98.9%**，但同一改动让车**爬上**实体障碍 —— 25 秒盲推（`vx=0.25`）`z_max` 从 **0.0601 m**
> 升到 **0.0951 m**（μ=0.2+1000 Hz 同样 0.0951 m）⇒ 障碍不再是障碍。这与 §M.5 对 robot11
> 的判决一致 ⇒ **不采用**。μ=0.02 档的同一盲推：`z_max=0.0601 m`、最远 0.506 m
> （原模型 0.526 m）⇒ **"前面有实体障碍就过不去"这条物理性质保住了**。

### 20.3 整栈实测（`world:=RMUC2026 mode:=mapping lio:=fastlio`，真值 + 仿真时间）

`tools/scripts/gzsim/drive_probe.py --cmd-topic /cmd_vel_chassis --wz 0.8 --vx 0.30 --duration 3`
（每段 3 **仿真**秒；`--publish-rate 20` = nav2 的控制器档位，200 = 物理上限档）：

| 量 | 改前（源模型）20 Hz | 改前 200 Hz | **改后** 20 Hz | **改后** 200 Hz |
|---|---|---|---|---|
| 自转 | 5.99 °/s = **13.1%** | 2.47 °/s = **5.4%** | 43.57 °/s = **95.1%** | 44.04 °/s = **96.1%** ※ |
| 前进 | 0.2258 m/s = 75.3% | 0.2231 m/s = 74.4% | 0.2960 m/s = **98.7%** | 0.2965 m/s = **98.8%** |
| `/livox/imu` 陀螺 z 均值（自转段） | 0.348 rad/s（**抖**） | 0.088 rad/s（**抖**） | **0.7885** | **0.7889** |

> ※ 200 Hz 那格：`drive_probe.py` 不做角度解缠，本次 3 s 转了 227.89° ⇒ 屏上打印 **−227.89°**
>   （越过 ±180°）；按解缠值 132.11°/3 s = 44.04 °/s 记。
> · 改前同一指令在**不同跑之间**落在 **3.2%…13.1%**（§16 记的 3% 是其中一跑）：不是"慢"，是
>   **抖转** —— 净转 6°…18° 而 IMU 陀螺均值到 0.35 rad/s（关节约束在来回掰），这正是 §16
>   "LIO 把原地抖转积分成 142.8 m"的物理来源。改后 IMU 0.789/0.8 = **98.6%**，与真值一致。
> · 改前 200 Hz 比 20 Hz **更差**（5.4% < 13.1%）也符合"抖"的图景：指令越密，约束对抗越频繁。
> · 口径警告：`robot_model_probe.py` 的 `phase_drift` 窗口包含"相位结束后补零速 + settle"那一段，
>   **不能**拿它当执行率分母（会把 96% 读成 72%）。本轮已把快照的**仿真时刻**写进 JSON
>   （`dt_sim_s`）并在 `drive_probe.py` 加 `--publish-rate`；执行率请用后者。

### 20.4 改了什么（最小面：一个生成物 + 一个槽位）

| 文件 | 改动 |
|---|---|
| `src/rm_nav_bringup/urdf/sentry_robot_sim_rigid.xacro` | **新增（自动生成）**：源文件 `sentry_robot_sim.xacro` 的**单刚体底盘**变体。差别只有两类：①`base_to_wheel1..4` `continuous→fixed`（去掉 axis/limit/anchor）⇒ sdformat 的 URDF→SDF 把轮子的几何/惯量**按平行轴定理合并**进 `base_link`（Gazebo 里 1 个 link）；②4 个 `<gazebo reference="wheel_N">` 补 `mu1=mu2=0.02`（源模型把 `mu1/mu2` 写在 `<link>` **里面**，Gazebo 的 URDF 解析器只认 `<gazebo reference>` ⇒ 一直是**失效**的）。源文件**一个字节没改**。 |
| `tools/scripts/regress/default_make_sim_xacro.py` | **新增生成器**：`--chassis rigid`（默认，写生成物）/ `articulated`（回退档 = 源文件逐字节副本）/ `--wheel-mu` / `--report-only`；写完前先自证（见 §20.5），不通过**不写文件**。 |
| `src/rm_nav_bringup/launch/bringup_sim.launch.py:136` | `_ROBOT_SLOTS['']` → `sentry_robot_sim_rigid.xacro`（默认槽位换生成物）；`robot` 的 description / 启动横幅 / 段注释同步（`:2049` / `:3490` / `:1687`）。**没有任何新 launch 参数**，`robot:=hzmirm|robot11` 的路径逐字节未动。 |
| `tools/scripts/gzsim/drive_probe.py:42` | `--publish-rate`（0=尽快；20=nav2 档）—— 量"指令频率对执行率"的影响。 |
| `tools/scripts/regress/robot_model_probe.py:224/239` | 快照多记一个**仿真时刻** `t`（+ `phase_drift` 里 `dt_sim_s`）。**只增字段**，不改变任何既有判据。 |

为什么是"生成物 + 槽位"而不是改源文件：这与 `robot:=robot11` 的既有机制**同构**
（`robot11_make_sim_xacro.py --chassis rigid|articulated` + `robot11_weld_chassis.py` 自证 +
生成物入库），而且**回退是一条命令**。区别只在"合并"这一步由 **sdformat** 做（fixed 关节的
URDF→SDF 合并），不搬几何 ⇒ 从根上不可能搬错（§M 的 robot11 是手写 FK 搬的）。

### 20.5 几何不变性自证（生成器每次写文件前都跑；不过就不写）

`python3 tools/scripts/regress/default_make_sim_xacro.py --report-only`（需
`source install/setup.bash`；`gz sdf -p` 会写 `~/.gazebo` ⇒ 生成器把 `HOME` 指到临时目录）：

| 检查 | 结果 |
|---|---|
| Gazebo 里的 link 数 | 源模型 **5** → 刚性档 **1**（`sdf_links_rigid=1`，joints=0） |
| 碰撞几何 | 6 条（底盘 box + 4 轮 cylinder + IMU box）；逐条 `(类型, base_link 系 AABB)` 与源模型**相等**（容差 5e-5 m；唯一 1e-6 级差异来自 xacro `${PI}=3.1415926` 与 SDF 的 π） |
| 总质量 / 惯量 | **10.21 kg**（8.2+4×0.5+0.01）/ **Izz=0.103945**、Ixx=0.083957、Iyy=0.070301；**与 sdformat 合并后的值逐位相等**（两套独立实现互证） |
| 整车最低点 | **−0.06 m**，落在 **cylinder（轮子）** 上（`wheels_are_lowest=true`） |
| 轮子几何 | `cylinder radius=0.06 length=0.05`，轮心 `(±0.1, ±0.13, 0)` —— 与源文件逐字相同 |
| 安装关节 | `livox_frame_joint` xyz=`0.12 0 0.175` rpy=`0 0 0`；`imu_joint` xyz=`0.12 0 0.125` —— 与源文件逐字相同；**运行期 TF 实测**（`/tf_static`）也是 `[0.12, 0.0, 0.175] / rpy 0` |
| base_link 离地 | `/odom_ground_truth` 的 GT z = **0.059987**（改前 0.059993）⇒ 0.06 不变 |
| `h` | `0.06 + 0.175 = 0.235` 不变；linefit `sensor_height=0.226` 的横幅与参数**逐字节未动**（改后 launch 日志照旧） |
| 轮子摩擦入 SDF | `sdf_wheel_mu=['0.02','0.02','0.02','0.02']`（4 个轮 collision 各一条 `<friction><ode><mu>`） |

### 20.6 默认模型的用户需要知道什么（诚实清单）

1. **物理被故意改了**（这就是修法）：轮关节变死 + 轮子对地摩擦 **0.02**。**前进也一起变快**
   （整栈 0.226 → 0.296 m/s，执行率 75% → 99%）。这不是独立回归，是**同一条摩擦损失**被拿掉的
   另一面：任何能让自转 ≥90% 的 μ，必然让前进也接近 100%（反过来 μ=1 时前进 80%、自转 14%）。
   ⇒ **§16 的 `0.183 m/s` 对默认模型作废**；依赖"车速比"的历史数字（覆盖跑图 10.6 m、
   26 min 等）都要按新速度重读。
2. **坡上会滑**：μ=0.02 ⇒ 静摩擦角 **≈1.1°**。平面速度驱动**爬坡不受影响**（插件直接设速度，
   不需要抓地），但**在 >1.1° 的坡上停住时**会缓慢下滑。§16 之前那些"坡度/台阶"实测
   （`slope_speed`、`low_terrain`）**没有**在本轮重测 ⇒ 若要用它们，先复核
   （`.tmp_lowterrain`、`tools/scripts/regress/slope_speed_probe.py`）。
3. **障碍仍然挡得住**：25 s 盲推 0.25 m/s → `z_max=0.0601 m`（不爬）、最远 **0.506 m**
   （原模型 0.526 m）。**对比**：`update_rate=1000` 那条路 `z_max=0.0951 m`（开始爬）⇒ 被否。
4. **`robot:=robot11` 完全不受影响**：它的 xacro / welded 生成物 / `_ROBOT_SLOTS` 条目都没动。
   （顺带记录：**Classic 里 robot11 的单刚体也只有 10.5%**（本轮台架 wz=0.8×2 s，`.tmp_wz/bench/out/rb_r11_spin`）
   —— 任务原文"robot11 达到 100%（wz 0.5 → 28.65°/s）"那个数出自 **gz（Fortress）后端**
   `docs/fortress_sim_backend.md §12.4`，那里的 100% 是**运动学直接积分、没有接触**，与
   Classic 的刚体/摩擦无关。）
5. **回退**（两条，都不需要改生成物）：
   · **整档回退**：把 `_ROBOT_SLOTS['']` 指回 `sentry_robot_sim.xacro`（源文件一直在，本轮
     一个字节没改）—— 就回到 4.3% 的多刚体行为；
   · **只回退摩擦**（保留单刚体）：`python3 tools/scripts/regress/default_make_sim_xacro.py
     --wheel-mu 1.0`（0.05→90.5%、0.1→84.6%、1.0→14.1%，见 §20.2 表）。
   ⚠️ `--chassis articulated --out <某个文件>` 只是把源文件**逐字节复制**到那个路径
   （它不会"反向焊回去"，也不改源文件）⇒ 别把它当成"就地撤销"。
6. **`install/` 需要重建**：`install/<pkg>/share/.../urdf/` 是**逐文件符号链接**（不是整目录软链），
   新文件不会自己出现 ⇒ `colcon build --symlink-install --packages-select rm_nav_bringup`
   （本轮实测：不建就直接 `FileNotFoundError: .../sentry_robot_sim_rigid.xacro`）。
7. **`rm_simulation.launch.py`（只起 Gazebo+模型的那个独立入口）没改**：它的 `robot:=''` 仍指向
   `hzmi_rm_simulation/urdf/simulation_waking_robot.xacro`（同一台车的**多刚体**版）⇒ 单独用它
   调试底盘时**仍是 4% 档**；要新物理请走 `bringup_sim.launch.py`。
8. **地图资产没有重建/提升**（按任务要求）：本轮没有跑覆盖建图，`map/`、`PCD/` 一个字节没动。
   速度变快 ⇒ 覆盖跑图的里程/时间关系会变，下一轮重建地图时需要重新标定。

### 20.7 nav 冒烟（`mode:=nav localization:=amcl`，**PASS**）

`tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --localization amcl --settle 5`
（栈：`world:=RMUL2026 mode:=nav localization:=amcl lio:=fastlio nav:=rpp spin_speed:=0.0`，
默认模型 = 本节修好后的 `sentry_robot_sim_rigid.xacro`；原始数据 `.tmp_rigid/rb_nav_after3/`）：

| 判据 | 实测 |
|---|---|
| 链路就绪 | clock/pcloud/imu/scan/odom/TF 全齐；**RTF=0.44**（前两次因机器 `load≈75` 只有 0.29/0.33，被工具的前置 RTF≥0.35 挡下 —— 那是**环境**门，不是导航失败） |
| 定位稳定性 | `map→odom` 5 s 窗口漂移 **xy=0.0000 m / yaw=0.0000 rad** |
| 目标可用性 | 先验图 `249x173 @0.05`，目标 (-1.00, 2.00) 落格 (23,102) = **free** |
| 结果 | **`status=SUCCEEDED`，7.8 s，`d_min=0.0 m`，`recoveries=0`** |
| **四跳命令链**（当初抓 `fake_vel_transform` 丢角速度的判据） | `nav=(1.20, 1.35)` → `smooth=(1.20, 1.35)` → **`chassis=(1.20, 1.35)`** ⇒ **角速度指令 1.346 rad/s 一路走到车体**（改前这条链也通，但车体只执行 3–13%） |
| 真值 | `twist_max=1.346`、`pose_delta=2.975 m` ⇒ 车**真的**动了（不是"假到达"） |

> 这是**default 槽位 + 现网地图资产**上的完整导航（含转弯），通过。⚠️ 反过来说：改前的 3–13%
> 角速度执行率下，nav2 发的 1.35 rad/s 转弯指令只会让车头挪 ~5°，所以 §16 的覆盖跑图卡在第一个转弯
> 是**模型物理**问题，不是 planner/controller 参数问题 —— 本节把物理修好了。

### 20.8 没能验证的（诚实边界）

* **可选的覆盖跑图冒烟没做**（时间不够）：预期"同样墙钟时间内真值位移 >10.6 m"未实测。
  速度变快 31%（§20.6#1）⇒ 覆盖跑图的里程/时间关系**会变**，下一轮重建地图需要重新标定。
* **坡度/台阶/低地形**未重测（见 §20.6#2）：μ=0.02 在 >1.1° 的坡上停住会滑；`slope_speed` /
  `low_terrain` 的历史数字**没有**在本轮复核。
* **nav 的多次重复**没做：nav 冒烟只有 1 次 PASS（RTF 0.44、7.8 s、无 recovery）；高负载下
  前置 RTF 门会挡，不能把它当"任意负载下都 PASS"。
* `robot:=robot11` "不受影响"是**静态证据**（文件/槽位未动 + 本轮台架 10.5% 与历史 16.5% 同量级），
  本轮没有重跑它的整栈 A/B。
* μ=0.02 是**为速度驱动插件标定的值**，不是从任何轮胎/地面材料推出来的；换世界/换地面材质
  需要重新看 §20.2 的剂量-响应。
* 台架脚本 `.tmp_wz/bench/*` 是 **gitignored** 的临时工具（§M 沿用），本轮没有把它提升成
  仓库内工具；可复现的权威口径是 §20.3 的 `drive_probe.py` 命令 + §20.5 的生成器自检。

## §21 第十二轮（**本机 date=2026-10-20T02:44–03:06**）：**缺陷③ 判决** —— Livox 插件 `cloud_frame=parent` 把点算在**父 link（= base_link）系**、`header.frame_id` 却写 `livox_frame`

> **本节只新增**（append-only）：§10–§20 一个字没改；写作时 §19/§20 已占 ⇒ 本节取 **§21**。
> 只动**一个源码文件**（`src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp`）
> + 本节：**不碰** `small_point_lio/**`、`lio_tf_adapter/**`、地图资产、任何 YAML/launch/URDF。
> **提交**：插件修复 = `7a544bf`（本节文档独自一个提交；两者都不含别人的在途改动）。
> 隔离与收尾口径与 §13/§17/§19 逐字相同：每跑私有 `GAZEBO_MASTER_URI` / `HOME` / `ROS_DOMAIN_ID`、
> `gui:=False`、`nav_rviz:=False lio_rviz:=False`、`world:=X` **不带前导 `--`**、`setsid` + 硬超时 +
> `tools/scripts/lib/scoped_cleanup.sh` 收尾（**无全机 pkill**）。每跑 `meta.txt` 记
> `date/git_head/launch_md5/plugin_cpp_md5/plugin_so_md5/lio_so_md5/ros_domain_id/gazebo_master_uri`；
> 原始数据 `.tmp_defect3/<tag>/`（gitignored，含 `probe.json` + `probe_pts.npz`）。
> 探针两个：`.tmp_defect3/probe.py`（**声明帧**里拟合地面 + 用 GT `odom_ground_truth` 与 URDF
> `base_link←livox_frame` 把云搬到 world + **直接 import §13.4 的 `wall_witness.py`** 当独立尺子）与
> `.tmp_defect3/hmeas_replay.py`（**同一帧云**重放 LIO 的地面互检，做"修前 vs 修后"的同云 A/B）。

### 21.1 一句话结论

> **③ 是真的，而且只在 `parent` 档**（= 默认模型 `robot:=''`、`robot11_mount:=plugin`（robot11 的
> 默认档）、`:=urdf`、`hzmirm` 的缺省；`sensor` 档 2026-10-21 已经修好，本轮**源码一行没动**）。
> 根因不是"哪个 mode 算错了"，而是**父 link 根本不是 `livox_frame`**：sdformat 的 URDF→SDF 把
> fixed 关节**合并**，传感器的 parent 变成 **`base_link`**、`<pose>` 变成关节 origin（21.3 有
> `gz sdf -p` 的静态证据）⇒ `parent` 档出的点 = **base_link 系**坐标，而 `header.frame_id`
> 写的是 `<sensor name>` = `livox_frame`。两者差**一个安装矢量**，实测（RMUL2026、静止、GT 真值）：
>
> | 模型 | 墙 x=2.5 (+x) | 墙 x=5.0 (−x) | 墙 y=8.5 (−y) | world 地面 z | **声明帧里的平移残差 = 安装矢量本身** |
> |---|---|---|---|---|---|
> | 默认模型（`(0.12,0,0.175)`） | **+0.1200** | **−0.1200** | **+0.0000** | **+0.1750** | **(+0.1200, 0.0000, +0.1750)** |
> | robot11（`(0.000562,0.130916,0.157028)`） | **+0.0006** | **−0.0006** | **−0.1309** | **+0.1550** | **(+0.0006, +0.1309, +0.1570)** |
>
> 三个分量逐项对上安装矢量（x/y 误差 ≤0.2 mm；z 受 mesh 台阶限制 ±6 mm）。**修后四个真实墙面
> （x=2.5 / x=5.0 / x=10.0 / y=8.5）与地面残差全部 0.0000 m**（§13.4 登记过的四个"相邻面进窗"假读数
> x=2.22 / x=14.78 / y=0.22 / y=8.78 仍是 +0.28，与本 bug 无关）；robot11 的 `plugin` 与 `sensor`
> 两档出点趋于逐字节相同（`h_meas` 0.2618 vs 0.2616、墙残差都 0.0000、`d_decl` 0.2594 vs 0.2595）。
> 同云重放（21.5.2）给出**干净的 Δ = +0.1750 / +0.1572 / +0.0701 m**，其中 +0.0701 正好复现了
> §13 的 sensor 档 bug（重放修前 0.1889 ≈ §13 现场记的 0.1894）⇒ 同一把尺子、两轮独立复现。

### 21.2 代码审计（每个 `<cloud_frame>` 值：点在哪个系 / `frame_id` / 加不加 offset）

**修前**（HEAD `e0db764`；行号是该提交的 `livox_points_plugin.cpp`）：

| `<cloud_frame>` | (a) 点在哪个系 | (b) `header.frame_id` | (c) `offset.Pos()` 加不加 | 自洽？ |
|---|---|---|---|---|
| `parent`（**缺省**：默认模型 / hzmirm / robot11 `plugin`\|`urdf`） | **父 link = base_link**：`axis = sensor_rot·mount_rot·ray`（`:336-338`）+ `cloud_origin = sensor_offset.Pos()`（`:363-364`），`point = range·axis + minDist·axis + cloud_origin`（`:365-366`） | `raySensor->Name()` = **`livox_frame`**（`:264` CustomMsg、`:270` PointCloud2） | **加**（父 link 系矢量） | ❌ **不同系** |
| `sensor`（`robot11_mount:=sensor`） | **传感器系 = livox_frame**：`axis = mount_rot·ray`（`:337`），`cloud_origin = Zero`（`:363`） | 同上 = `livox_frame` | 不加（`Zero`） | ✅（2026-10-21 已修，§13） |

* `offset` 的来源：`laserCollision` 建在 `_parent->ParentName()` 上、`SetRelativePose(_parent->Pose())`
  ⇒ `sensor_offset = 传感器在父 link 里的 pose`（`:312-314`；`Load` 里 `:111-114`）。
* **射线**始终按父 link 系生成（`Load:215` / `InitializeRays:433`：`offset.Rot()·mount_rot·ray`），
  本轮**不动**——世界里的射线方向本来就对（21.4 的墙面能对上就是证据）。
* 另有一条只给 Gazebo `laserscan` 消息用的 `laserMsg.scan.frame = _parent->ParentName()`（`:106`），
  它**一直**写的是真父 link，与本 bug 无关（也无消费者）。

**修后**（本提交）：

| `<cloud_frame>` | (a) 点在哪个系 | (b) `header.frame_id` | (c) `offset` | 自洽？ |
|---|---|---|---|---|
| `parent` | **livox_frame**：`axis = mount_rot·ray`（`:351`），`point = range·axis + minDist·axis`（`:364-365`） | `livox_frame`（`:262`/`:268`） | 不加（父 link 系矢量**换算掉**了） | ✅ |
| `sensor` | 同上（**同一条式子**） | 同上 | 不加（与修前逐字节相同） | ✅ |

`<cloud_frame>` 元素**保留**（老 SDF/URDF 照常加载、不报错），但两档从此**等价**；启动日志会打一行
收据（`cloud_frame = …（**历史开关**；自 2026-10-21 起两档等价…）`），运行期还有一条 `INFO_ONCE`
把**被换算掉的安装偏移**打出来（`:311-316`）。

### 21.3 为什么"父 link 系"其实就是 `base_link`（静态证据，不用起仿真）

1. URDF 里传感器挂在 `${name}` link 上：`ros2_livox_simulation/urdf/mid360.xacro` 的
   `<gazebo reference="${name}"><sensor …><pose>0 0 0 0 0 0</pose>`（robot11 是自己抄的同一段）。
   按 URDF 读，父 link = `livox_frame`、传感器 pose = 0。
2. **但 Gazebo 读的不是 URDF**：`gz sdf -p <展开后的 urdf>` 的结果里整个模型**只剩一个 link**
   `base_link`，ray 传感器直接在它下面、`<pose>` = 关节 origin：

   ```
   默认模型：<link name='base_link'> … <sensor name='livox_frame' type='ray'> … <pose>0.12 0 0.175 0 -0 0</pose>
   robot11 ：<pose>0.000561701465 0.130915824457 0.157028169683 0 -0 0</pose>（plugin 档 rpy=0）
   ```

   ⇒ `_parent->ParentName()` = `…::base_link`，`laserCollision->RelativePose()` = 关节 origin
   ⇒ 这正是插件注释里写的 `offset.Pos()`（robot11 = `(0.000562, 0.130916, 0.157028)`）。
3. 复现命令（无需仿真）：`xacro install/rm_nav_bringup/share/rm_nav_bringup/urdf/sentry_robot_sim_rigid.xacro > /tmp/x.urdf && gz sdf -p /tmp/x.urdf | grep -A2 "sensor name"`。
   ⚠️ 合并 fixed 关节是**转换器**的行为（Gazebo Classic 11 实测）；若某天换转换器保留 `livox_frame`
   link，则 `offset` 会变成 0——本修法与 `offset` 是否为 0 无关（代数恒等），旧代码只是"偶然正确"。

### 21.4 判决性实验（**独立于 h_meas**：已知墙面 + 地面）

口径：`world:=RMUL2026 mode:=mapping lio:=small_point_lio`（默认模型 / `robot:=robot11` / 再
`robot11_mount:=sensor`），出生点 (4.3, 3.35)、GT 静止（`GT base_link z`：默认模型 0.059997、
robot11 0.102499/0.102494）。取 `/livox/lidar/pointcloud` 一帧（frame_id 实测就是 `livox_frame`），

* **① 声明帧地面**：沿"world up 在声明帧里"（由 GT + TF 算出）拟合地面，得 `d_decl`；
  真值 = **传感器原点的 world 高度** `h_true = (R_wb·t_bl + t_wb).z`（默认模型 0.23500、
  robot11 0.25956/0.25957）。
* **② world 地面**：把云按**声明帧 = livox_frame** 搬到 world（= 下游做法）⇒ 地面应 ≈0
  （RMUL2026 的可行驶面是 1 cm 级台阶的 mesh：`sensor` 档实测 world z 直方图峰 **+0.005**、
  SVD 面 **−0.003**，§13.3 已登记"常数偏移只准到 ±6 mm"——不是本 bug）。
* **③ 已知墙面**：§13.4 的 `wall_witness.py`（RMUL2026 mesh 的竖直面），残差 = 点云平移在该面
  法向上的投影。

| tag | 模型 / 挂法 | 云 | ① `d_decl`（原点下方最大平面） | `h_true` | ③ x=2.5 (n=+x) | ③ x=5.0 (n=−x) | ③ y=8.5 (n=−y) | ② world 地面（注） |
|---|---|---|---|---|---|---|---|---|
| `d0b` | 默认模型 | parent | **0.0585** | 0.23500 | **+0.1200** | **−0.1200** | +0.0000 | **+0.1750** |
| `p0b` | robot11 `plugin` | parent | **0.1026** | 0.25956 | +0.0006 | −0.0006 | **−0.1309** | +0.1550 |
| `s0b` | robot11 `sensor` | sensor | 0.2596 | 0.25957 | −0.0000 | +0.0000 | +0.0000 | +0.0050 |
| `d2` | 默认模型（RMUC2026） | parent | 0.2341 | 0.23500 | n/a（RMUC mesh 不可用，§16.9） | n/a | n/a | −0.0050/+0.0016 |
| `d1` | 默认模型 | parent（**修后**） | 0.2337※ | 0.23500 | **+0.0000** | **−0.0000** | **+0.0000** | +0.0056 |
| `p1` | robot11 `plugin` | parent（**修后**） | 0.2594 | 0.25957 | **+0.0000** | **−0.0000** | **+0.0000** | −0.0010 |
| `s1` | robot11 `sensor` | sensor（**修后**） | 0.2595 | 0.25956 | **+0.0000** | **−0.0000** | **+0.0000** | −0.0022 |

※ `d1` 的"原点下方最大平面"被**台面**抢走了（RMUL2026 有一块 25.5 m² 的抬高面在 world +0.196，
修后它落在传感器下方 0.039 m、点数比地面多）⇒ 0.2337 取的是 `r∈0.8–8` 那一档的地面峰；
`d_decl`=0.0388 那一档是台面。这不影响判决（墙面残差全 0.0000 + world 地面 +0.0056）。
**② 列**：取**地面**那个平面；`d0b` 的最强平面其实是台面（+0.3652），故那里取"最低的大水平面"
（+0.1750，n=578）；`p0b`/`d2`/`d1`/`p1`/`s1` 取全点 SVD 最大的那个面（就是地面）。

**判读**

1. **`parent` 档的残差矢量 = 安装矢量本身**：默认模型 `(+0.1200, 0, +0.1750)` vs
   `(0.12, 0, 0.175)`；robot11 `(+0.0006, +0.1309, +0.1570)` vs `(0.000562, 0.130916, 0.157028)`。
   不是"地面高了"，是**整朵云被搬走了**——与 §13.4 当年对 `sensor` 档的判据同一形态。
2. **`sensor` 档前后都干净**（0.2596 对 0.25957；墙 0.0000）⇒ 回归基线成立。
3. 默认模型的 y 分量**永远是 0**（安装矢量 y=0）⇒ 它的 y 墙残差 0.0000 是**空检查**；
   x 墙（+0.1200）才是默认模型的水平证人。
4. 修后 robot11 的 `plugin` 与 `sensor` **两档一致**（0.2594 vs 0.2595、墙都 0.0000、world 地面
   −0.0010 vs −0.0022）⇒ 两档的差别只剩"帧怎么定义（倾角记在关节还是 `<tilt_rpy>`）"，
   不再有"数据与帧不同系"。
5. 表里只列了三个墙面；RMUL2026 的 **x=10.0（n=−x）** 也是 0.0000（修前 −0.0006 / 修后 +0.0000），
   与前两个 x 面一致。

### 21.5 `h_meas` 前后（两种口径，别混）

#### 21.5.1 现场值（LIO 自己打的 `gravity_align_h_check`，每次初始化一条）

| run | 世界 / 模型 / 挂法 | `h_urdf` | `h_meas`（修前 → 修后） | 电平 | 备注 |
|---|---|---|---|---|---|
| `s0b`→`s1` | RMUL2026 / robot11 / `sensor` | 0.259527 | **0.2618 → 0.2616** | INFO | 回归基线（Δ 0.2 mm = 拟合噪声） |
| `p0b`→`p1` | RMUL2026 / robot11 / `plugin` | 0.259527 | **0.1045 → 0.2618** | ERROR → **INFO** | 修后与 `sensor` 档同值 |
| `d2`（修后） | RMUC2026 / 默认模型 / parent | 0.235000 | （§19 修前 **0.1007**）→ **0.2754** | （ERROR）→ **WARN** | Δ = **+0.1747** ≈ 安装矢量 z |
| `d1`（修后） | RMUL2026 / 默认模型 / parent | 0.235000 | （`d0` 修前 **0.1215**）→ **0.0973** | ERROR → ERROR | **不可用**，见 21.8#2 |
| `nav2`（修后） | RMUL2026 / robot11 / `sensor` / nav | 0.259527 | **0.2615** | INFO | nav 冒烟里再确认一次 |

⚠️ **默认模型的 `h_meas` 不是一把干净的尺子**（修前修后都不是），两条独立原因见 21.8#1/#2；
它的**修前 0.1007 → 修后 0.2754**（RMUC2026，同一 recipe）恰好等于 `+0.1747`，与安装矢量 z 一致，
这本身是"父 link 系"假设的又一个证人（0.1007 就是**初始化时刻 base_link 的高度**）。

#### 21.5.2 同云重放（**决定性的 A/B**：同一帧云，只把旧公式加回去）

`hmeas_replay.py` 用 `gravity_align_ground.cpp` 的同一套约束（`d∈[0.05,0.60]`、法向 vs world-up
≤20°、tol 2 cm、`r∈[0.5,8]`、2000 点子采样、固定种子）在**已落定的一帧**上重放：

* **修前(plugin)** 的云 = 修后的云 **+ `offset.Pos()`**（默认模型 `(0.12,0,0.175)`、robot11
  `(0.000562,0.130916,0.157028)`）——这是**逐字节**的旧公式（`parent` 档加的就是它）；
* **修前(`sensor`，2026-10-21 之前)** 的云 = 修后的云 + `offset.Pos()`（同一个 bug 的未旋转版）。

| tag（云） | 修前 `d` | 修后 `d` | **Δ** | 安装矢量 |
|---|---|---|---|---|
| `d2`（默认模型 / RMUC2026） | **0.0579** | **0.2329** | **+0.1750** | z = 0.175（x/y 不影响地面垂距） |
| `p1`（robot11 / `plugin`） | **0.1020** | **0.2592** | **+0.1572** | z = 0.157028 |
| `s1`（robot11 / `sensor`） | **0.1889** | **0.2589** | **+0.0701** | z = `(R_joint·offset).z` = 0.0705（§13） |
| `d1`（默认模型 / RMUL2026） | 0.0503（tilt 13.8°） | 0.0548（tilt 0.87°） | +0.0044 | ❌ 台面抢峰（21.8#2） |

* `d2` 一行是**最干净**的：同一帧云、同一算法，Δ 恰好 = 安装矢量 z（0.1750）。
* `s1` 一行把 §13 的 sensor 档 bug **独立复现**了一遍：重放 0.1889 vs §13 现场 0.1894
  （差 0.5 mm）；Δ 0.0701 vs §13 的 `R_joint·offset.Pos()` = 0.07053。
* `d1` 说明：一旦"声明帧里的地面"被场景里的**抬高台面**抢走，`h_meas` 就完全失去判别力
  （修前修后都落在台面附近）。

### 21.6 修法（最小 diff）与为什么不改 `frame_id`

**修法**：把 `parent` 档的点**换算进传感器系**（不新增开关、不动射线）：

```
p_parent = (range+minDist)·(sensor_rot·mount_rot·ray) + offset.Pos()
p_sensor = sensor_rotᵀ·(p_parent − offset.Pos()) = (range+minDist)·(mount_rot·ray)
```

⇒ 与 `sensor` 档**同一条式子**（`livox_points_plugin.cpp:351`、`:364-365`），
`cloud_origin`（`:363-364` 修前）整条删除 ⇒ `offset.Pos()` 不再出现在出点公式里。
`range_from_origin:=false`（老行为）也只差 `minDist·axis`，坐标系同样正确（`:364-365`）。
`sensor` 档的表达式一个字没动（它本来就相当于 `+ Vector3d::Zero`）⇒ **数学恒等**。

**为什么不去改 `frame_id`（把父 link 声明出来）**——先查了每个消费者：

| 消费者 | 它怎么用 `frame_id` | 改 `frame_id=base_link` 会怎样 |
|---|---|---|
| `small_point_lio`（`/livox/lidar` CustomMsg） | **不看消息里的 frame_id**，但配置契约写死 `lidar_frame: livox_frame`「必须与 `/livox/lidar` 的 `header.frame_id` 和 URDF 帧名一致」（`config/mid360_sim.yaml:14`）；`extrinsic_T/R` 是**雷达系→IMU 系**；`odometry_convention` 按 `lidar_frame` 定 `child_frame_id`/TF | 要么改 `lidar_frame` + 三份配置的 `extrinsic_*`（= 动 `small_point_lio/**`，本轮**禁止**），要么继续"数据在 base_link、算法以为在 livox_frame"（bug 只是搬进 LIO 里） |
| `linefit` | `gravity_aligned_frame: ""` ⇒ **不查 TF**，直接假设"点云自带帧的 z 轴朝上"（`ground_segmentation_node.cc:114`）；只用旋转 | 改成 base_link 要同步改 `gravity_aligned_frame`/`sensor_height`（YAML，本轮禁止）；数据侧不改则一定要动它 |
| `pointcloud_to_laserscan` | `target_frame: ""` ⇒ **不查 TF**，输出 `scan.header.frame_id = cloud.header.frame_id`（`pointcloud_to_laserscan_node.cpp:151`）；`max_height` 等 z 过滤发生在**点云自带帧** | 输出帧会变成 `base_link` ⇒ nav2 的 TF 链与 z 带语义全变（要改 YAML + 复标定） |
| `cloud_accumulator` | 用 `msg.header.frame_id → target_frame` 查 TF（`accumulator_node.py:276`） | 会自动用 base_link，**这个**没问题 |
| nav2 costmap / RViz | 按 `scan.header.frame_id` 查 TF | 同上 |

⇒ 有一个消费者（`small_point_lio`）**要求 `frame_id = livox_frame`**，而"把父 link 声明出来"
还必须改三份 LIO 配置/外参（禁止动）⇒ 按任务书的判据，**改点、不改声明**：把点换算进
`livox_frame`，`frame_id` 保持 `livox_frame`（`:262`/`:268`）。

### 21.7 回归 + nav 冒烟

| 项 | 修前 | 修后 | 判据 |
|---|---|---|---|
| robot11 + `sensor`：`h_meas` | 0.2618（`s0b`） | **0.2616**（`s1`）/ **0.2615**（nav 跑） | INFO，Δ≤0.3 mm ⇒ 不变 |
| robot11 + `sensor`：墙面残差 | 全 0.0000 | **全 0.0000** | 不变 |
| robot11 + `plugin`：`h_meas` | **0.1045 ERROR** | **0.2618 INFO** | 修好 |
| robot11 + `plugin`：墙面残差 | (0.0006, 0.1309, 0.1570) | **全 0.0000** | 修好 |
| 默认模型：墙面残差 | (0.1200, 0, 0.1750) | **全 0.0000** | 修好 |
| 默认模型：`h_meas`（RMUC2026） | 0.1007 / ERROR（§19） | 0.2754 / WARN | +0.1747 = 安装矢量 z；绝对值受 21.8#1 限制 |
| 默认模型：下游（`fastlio` + linefit + p2l） | 未测（本轮无修前值） | `/segmentation/ground` **2568**、`/segmentation/obstacle` **3780** 点/帧、`/scan` **1166** 有限波束（66 帧中位） | 非空、量级正常（**只证"没坏"**，不是 before/after） |
| 默认模型：声明帧地面峰 | −0.06（= base_link 高度，`d0b`） | **−0.235**（−0.235±0.02 内 627 点；`d4`） | 与 `h_urdf` 0.235 一致 |

**nav 冒烟**（1 跑）：`local_costmap` **非空**（`n_lethal` 中位 **275.5**、最近致命 **0.7079 m**）、
`h_meas` **0.2615 INFO**，数字见 21.7.1。
⚠️ 任务书给的那条命令**少了 `lio:=small_point_lio`**：`gravity_align_mode:=hybrid` 只对
`small_point_lio` 生效，launch 会**硬报错**（`[launch] gravity_align_mode:=hybrid 只对
lio:=small_point_lio 生效`）——不是本改动的回归，补上 `lio:=` 即可。

#### 21.7.1 nav 冒烟实测

命令：`world:=RMUL2026 mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor
lio:=small_point_lio gravity_align_mode:=hybrid`（1 跑；`nav_band_probe.py --settle 20 --window 20`，
100 个局部代价图样本 / 200 帧 `/scan`）：

| 量 | 中位 | min–max | 判据 |
|---|---|---|---|
| local `n_lethal` | **275.5** | 265–290 | **>100 ⇒ 非空** ✅（§13.8 hybrid 基线 292） |
| local `n_inscribed` | 9433.5 | 9274–9697 | 同量级（§13.8 9575） |
| local `n_free` | 42470 | 42206–42775 | 同量级（42374） |
| local `n_unknown` | 0 | 0 | — |
| `nearest_lethal_m` | **0.7079** | 0.7041–0.7177 | 合理（§13.8 0.7070） |
| `n_lethal_ahead`（2.0×0.6 m） | 31 | 22–37 | 同量级（§13.8 30） |
| `n_lethal_within_1m` | 37 | 27–48 | 同量级（41） |
| `/scan` 有限波束/帧 | 775 | 755–788 | §13.8 775 ✅ |
| `/scan` 在带 `[0,2]` / 带下 / 带上 | 775 / **0 / 0** | — | 无整盘掉带 |
| `/scan` z(odom) p50 | +0.0365 | +0.0351–0.0381 | §13.8 hybrid +0.0358 |
| global `n_lethal` | 3725 | 3723–3726 | §13.8 3713 ✅ |
| global `nearest_lethal_m` | 0.4586 | 0.4577–0.4590 | — |
| 同跑 `h_meas` | **0.2615 INFO** | — | robot11+`sensor` 基线不变 |

⇒ **局部代价图非空、最近致命栅格 0.71 m 合理**，与 §13.8/§17 的 hybrid 基线同量级；这一跑同时
证明 `sensor` 档在 nav 全栈里也没有被本轮改动影响（`h_meas` 0.2615、`/scan` 775 波束在带内）。
原始数据 `.tmp_defect3/nav2/band.json`（100 个样本的逐样本明细也在里面）。

**另一条下游冒烟**（默认模型，`world:=RMUL2026 mode:=mapping lio:=fastlio`，修后 1 跑）：插件运行期
收据打出 `offset.Pos()=[0.120000 0.000000 0.175000] rpy=[0 0 0]`（= 被换算掉的那个矢量），
`/segmentation/ground` **2568** 点/帧、`/segmentation/obstacle` **3780** 点/帧、`/scan` **1166**
有限波束（66 帧中位），点云声明帧里的地面峰从 **−0.06 → −0.235**。⇒ 默认模型的 linefit/p2l **没有
因为点云下移 0.175 m 而坏掉**（linefit 的 `sensor_height=0.226` 反而更贴近 −0.235）。⚠️ 这一条
**没有修前对照**（旧 `.so` 未保留），只能证明"修后是健康的"，不能证明"下游输出逐字节不变"。

### 21.8 没能验证 / 仍未解释的（诚实清单）

1. **默认模型的 `h_meas` 绝对值不可用（修前修后都不可用）**：默认模型的出生 z=0.2、轮底 0.06、
   `planar_move` 每步把 vz 清零 ⇒ 下沉 ≈2.2 cm/s（§13.7 对 robot11 的同一机制实测）⇒ 落差
   14 cm 要 **6.4 s**；而
   `small_point_lio` 的初始化门只有 ~2 s 的 IMU（`fix_gravity_direction` 要 200 条）⇒ **初始化发生
   在还没落地时**（本机实测：`gravity_align_init` 在插件加载后 **5.56 s**）。所以
   `h_meas` 量到的是"**初始化那一刻**传感器离地高度"：修前（RMUC2026）= **0.1007** = 当时
   `base_link` 的高度（parent 档的签名），修后 = **0.2754 = 0.1007 + 0.175**。
   ⇒ "默认模型 ⇒ `h_meas ≈ 0.235` INFO"**在本设置下拿不到**；要拿到得把出生 z 降到 ≈0.08 或把
   LIO 初始化延后到落地之后（那是 launch/世界的改动，本轮不许动）。**这一条是既有问题**
   （与插件无关）：`robot11` 的落差只有 9.75 cm（4.4 s 落完）⇒ 它的 `h_meas` 一直是 INFO。
2. **RMUL2026 里默认模型的 `h_meas` 被抬高台面抢峰**：mesh 里有一块 **25.5 m²** 的水平面在
   world **+0.196**（§13.3 的"高台"）。修后它正好落在传感器下方 **0.039 m**：LIO 的约束是
   `d≥0.05`，于是 RANSAC 在 2 cm 容差里挑到一个**斜 0.84° 的混合面**（0.0973）。修前它在
   传感器**上方** 0.136 m（`d<0` 被排除），拟合落在别处（0.1171/0.1215）。两个值都**不能**
   当"地面高度"读。⇒ 在 RMUL2026，默认模型的 `h_meas` 是**弱判据**，判决必须靠 21.4 的墙/地面。
3. **z 分量的精度只到 ±6 mm**（RMUL2026 可行驶面是 1 cm 级台阶的 mesh，§13.3 已登记）；
   x/y 分量到 0.1 mm。所以"残差 = 安装矢量"这句话在 x/y 上是**精确**的，z 上是"±6 mm 内相等"。
4. **默认模型的 y 分量是空检查**（安装矢量 y=0）⇒ 它的 y 墙残差 0.0000 不代表任何东西。
5. **`robot11_mount:=urdf` 的后果没实测、而且会变**：修后它与 `sensor` 档出点相同（数据真的在
   斜 30° 的 `livox_frame` 里），但 `urdf` 档的 linefit 配置**没有** `gravity_aligned_frame`（只有
   `sensor` 档有）⇒ linefit 的"z 轴朝上"假设不再成立。`urdf` 本来就是登记过的坏档（§"帧与数据差
   30°"），现在只是把不一致从**上游**搬到了**下游**。要用这一档请按 `sensor` 档那套覆盖配置；
   本轮按任务书**不改 YAML**。
6. **`hzmirm` 槽位（雷达 z=+0.8）没跑**：静态推算它的 `parent` 档旧行为会偏 `(0,0,0.8)`，但本轮
   **没有实测**（它的 0.8 m 高雷达 + 云台 geometry 需要另一套世界/姿态）。
7. **地图资产没重建**（本轮禁止动）：§16.9#4 记的"驱动版 3D 地面 **+0.283 m**（world）/
   `map→odom z=+0.403`"**不能**说已解释。本轮只证明了**点云侧**默认模型有 **+0.175**（z）+
   **+0.12**（x）的常数偏移；`0.283 − 0.175 = 0.108` 那部分仍然**没有解释**，需要按 §16.4 的
   recipe 重跑一次建图才能验（本轮时间/资产限制，未做）。
8. **单跑**：每种配置 1 跑（无 3 跑中位）；nav 冒烟 1 跑。
9. **`sensor` 档"逐字节不变"是源码级 + 实测量级证据**，不是二进制级位相同重放：源码里被删掉的
   是 `+ Vector3d::Zero`（数学恒等），实测 `h_meas` 0.2618→0.2616、墙残差 0.0000→0.0000、
   `d_decl` 0.2596→0.2595。
10. **`<cloud_frame>` 现在是 no-op**：老 SDF/URDF（`robot11_mount:=sensor` 会传
    `<cloud_frame>sensor</cloud_frame>`）继续能加载，但两档等价。若将来真要让消费者选"父 link 系
    的云"，正确做法是**同时**改 `frame_id` + 所有消费者的契约（21.6 的表），不是只改插件。

## §22 第十三轮（**本机 date=2026-10-20T03:10–04:10**）：`robot:=robot11` 也真的能原地转了 —— 单刚体早就有，缺的是**轮子对地摩擦**（10.6% → 95.8%）

> **本节只新增**（append-only）：§1–§21 一个字都没改。§21 是最新 ⇒ 本节用 §22。
>
> 触发 = 父任务原文：**"把让默认模型转起来的那次双修（`aa77420`）原样应用到 `robot:=robot11`：
> 它在 Classic 里实测纯自转 `wz=0.8` 只有 10.5%，前进正常"**。
>
> ⚠️ **本节故意改了 `robot11` 的物理**（这正是修法）：轮子对地摩擦从 Gazebo 缺省 μ=1 变成 **μ=0.02**。
> **几何一个字节没动**（SDF 文本 diff 只有摩擦元素，见 22.6）；`robot11_mount:=sensor` 这条参考流水线
> 的 `h_meas=0.2616 m / INFO`、墙面残差 ≈0 也**没变**（22.8）。
>
> 隔离（与 §20/§21 同口径）：私有 `HOME`、非默认 `ROS_DOMAIN_ID`（**必须 <233**，本轮踩过：240 直接
> 建不出节点、gzserver exit −6）、按 tag 探测端口的 `GAZEBO_MASTER_URI`、`unset DISPLAY`、`gui:=False`、
> `world:=X` **不带**前导 `--`、收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）。
> 原始数据：`.tmp_r11fix/`（改前/改后 SDF、`drive_*.json`、nav 两跑）、`.tmp_tiltmount/r11fix_*/bench/`
> （台架变体矩阵），均为 gitignored。

### 22.1 一句话结论

> `robot:=robot11` **早就是单刚体**（2026-10-10 的 `robot11_weld_chassis.py`；`gz sdf -p` = **1 link /
> 0 joint / 113 collision**）⇒ `aa77420` 的**第①项（多体 → 单刚体）它已经做过了**；
> **剩下的只有第②项：轮子对地摩擦** —— 改前的 SDF 里 113 条 collision 的 `<surface><friction><ode>`
> **全是空的**（ODE 缺省 μ=1 ⇒ 4 个轮子横向刮地），所以自转只有 **10.6%**（台架）/ **9.8%**（整栈 20 Hz）。
> 补上 Gazebo **唯一认的**写法 `<gazebo reference="base_link"><mu1>/<mu2> = 0.02`（轮子已塌进 base_link，
> 没有 `wheel_N` link 可写）⇒ **95.8%**（台架）/ **92.9%**（整栈 20 Hz）/ **95.3%**（200 Hz）；
> 纯前进同时从 **79.3% → 88.6%**（台架）、**55.2% → 98.2%**（整栈）—— 与 §20.6#1 同一条"摩擦损失的另一面"。
> **几何逐条不变**（22.6 的表 + SDF 文本 diff 只有摩擦元素）；代价是 **μ=0.02 ⇒ 静摩擦角 ≈1.1°**（22.7）。

### 22.2 复现（改前，HEAD = `818d52c`）

口径与 §20 相同：真值 `/odom_ground_truth`、时间一律用**仿真时间**；台架 = 只起 gzserver + 一个模型 +
`planar_move`（摘掉 LIO/nav2/雷达，`tools/scripts/tiltmount/run_chassis_yaw_bench.sh`），整栈 =
`run_robot_model_probe` 同款 launch + `tools/scripts/gzsim/drive_probe.py`。

| 跑法 | 命令 | 纯自转 `wz=0.8` | 纯前进 `vx=0.30` |
|---|---|---|---|
| 台架（RMUL2026） | `run_chassis_yaw_bench.sh … --wz 0.8 --push-wall 2.0` | **9.713° / 91.7° = 10.6%** | `disp=0.3567 m / 0.45 m` = **79.3%** |
| 整栈 20 Hz（RMUC2026，`mode:=mapping lio:=fastlio robot:=robot11`） | `drive_probe.py --wz 0.8 --vx 0.30 --duration 2 --publish-rate 20` | **8.94° / 2 s = 9.8%**（IMU wz 均值 0.0107 rad/s） | **0.3315 m / 2 s = 55.2%** |

> 与任务书给的 **10.5%** 一致（同一台架、同一 SDF：本轮先复算
> `robot11_make_sim_xacro.py --out <临时目录>/…` 与 HEAD **逐字节相同**，再 `gz sdf -p` 出的 SDF 与
> `.tmp_rigid/robot11.sdf` **逐字节相同**）。§20.6#4 记的"Classic 里 robot11 的单刚体也只有 10.5%"
> 就是本节这一行 —— **原因是摩擦从来没生效**，不是"robot11 修不好"。

### 22.3 诊断：剩下的限制项是哪一个

**① 结构（多体 → 单刚体）：已经满足。** `gz sdf -p` 的旧模型：

| 检查 | 改前实测 | 结论 |
|---|---|---|
| SDF link 数 / joint 数 | **1 / 0** | 已经是单刚体（`planar_move` 的 `Set*Vel` 自洽） |
| 碰撞条数 | **113**（109 box + 4 cylinder） | 几何全在 base_link 上，轮子也是 |
| 轮子 | 4 条 `cylinder r=0.058 L=0.0452/0.0453` | 塌陷时逐条搬过来了（22.6） |
| 最低点 | **−0.102502 m（cylinder）**，非轮 box 最低 −0.077994 m | 只有轮子触地 |

**② 摩擦（唯一剩下的）：改前完全没有。** 旧 SDF 里 113 条 collision 的
`<surface><friction><ode>` 全部是 `<ode/>`（**空**）⇒ Gazebo/ODE 用缺省 **μ=1**。为什么没有：

1. **上游 `urdf/upstream/robot11.urdf` 里根本没有 `mu1/mu2`**（`grep` 0 条）—— 与默认模型
   "写在 `<link>` 里"还不是同一种坏法，robot11 是**完全没写**；
2. 生成器只搬 `inertial/visual/collision`，不搬摩擦；`robot11_weld_chassis.py` 的 ④a 又**删掉**
   指向被塌 link（l6…l9）的 `<gazebo reference>`（材质）—— 就算写了 `<gazebo reference="l6">`
   也会在塌陷后被删/找不到 link；
3. 更根本的一条（与 §20 同）：`<mu1>/<mu2>` **写在 `<link>` 里面时 Gazebo 的 URDF 解析器不认**，
   只有 `<gazebo reference="<link>">` 会被写进 SDF 的 collision。

⇒ 只补摩擦、且必须写在**根 link**（`base_link`）上。这也解释了摩擦剂量-响应为什么和默认模型
几乎重合（同一套"平面速度驱动 + 焊死轮子横向刮地"机制，只是轮子 r=0.058、轴距 0.36427 m）。

### 22.4 修法（最小面：生成器 + 一个生成物；源文件一个字节没改）

| 文件 | 改动 |
|---|---|
| `tools/scripts/regress/robot11_make_sim_xacro.py:95` | 新增 `DEFAULT_WHEEL_MU = 0.02`（并在注释里写清"Gazebo 只认 `<gazebo reference>`"） |
| 同上 `:681` | 新增 `--wheel-mu FLOAT`（缺省 0.02；`0` = 不写）+ `--no-wheel-friction`（回到 HEAD 行为） |
| 同上 `:487` | 新增 `WHEEL_MU_BLOCK`：`<gazebo reference="<根 link>"><mu1>μ</mu1><mu2>μ</mu2></gazebo>`（+ 一段解释性注释，写进生成物里看得见的地方） |
| 同上 `:525` | 新增 `check_wheel_friction()`：**写文件前**用 `gz sdf -p` 自证 —— 1 个 link、113 条 collision 的 `<surface><friction><ode><mu>/<mu2>` **全部**等于 μ、4 条轮 cylinder、113 条碰撞 AABB 与 URDF 逐条相等（最大偏差 **5e-6 m**）、SDF 的 mass/Izz 与"全 link 平行轴定理"合成值相等（9.56212 / 0.129172）。不通过 ⇒ **报错不写文件** |
| 同上 `:985` | 摩擦档把一段 `★ 2026-10-20` 说明传进 `weld()` 的注释（生成物末尾可见；不摩擦时不传 ⇒ 回退逐字节） |
| 同上 `:994` | `main()` 里调用自证，把结果写进 `rep['wheel_friction']` 并打一行 stderr 收据 |
| `tools/scripts/regress/robot11_weld_chassis.py:217-218,380` | `weld(..., extra_note='')`：给生成物末尾那段单刚体注释追加一段**看得见的**摩擦说明（不传 ⇒ 与 HEAD 逐字节相同，回退档不受影响） |
| `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro:876-879` | **自动生成**的摩擦块（`<gazebo reference="base_link"><mu1>0.02</mu1><mu2>0.02</mu2></gazebo>`）。相对 HEAD 的 `diff` = 这一块 + 两段注释，**模型体没有其它变化** |

为什么写在**根 link** 而不是 `wheel_N`：`robot:='robot11'` 的轮子（`l6..l9`）在 2026-10-10 就被
`robot11_weld_chassis.py` **塌进 `base_link`** 了（SDF 里没有 `wheel_N` 这个 link，写了也只会
"reference to unknown link"）。`<gazebo reference>` 是 link 级的 ⇒ 113 条 collision **全部**拿到 μ。
**物理上没关系**：真正接地的只有 4 个轮 cylinder（最低点 −0.102502 m 在轮上；非轮几何最低
−0.077994 m，正常姿态差 2.45 cm 永不触地）。这是与 §20（默认模型只给 4 个轮子写 μ）**唯一的口径差异**，
诚实登记在 22.9。

**回退（都不改源文件）：**
* `python3 tools/scripts/regress/robot11_make_sim_xacro.py --no-wheel-friction`（或 `--wheel-mu 0`）
  ⇒ 生成物与 HEAD **逐字节相同**（实测 `diff` 空）；
* `--chassis articulated` ⇒ 与 HEAD 生成器的 articulated 输出**模型体逐字节相同**（去注释后
  34573 B vs 34573 B，`diff` 空；只多出文件头注释）；
* `--wheel-mu 0.1 / 0.05 / 0.2` ⇒ 见 22.5 的剂量-响应。

**`install/` 不需要重建**：`install/rm_nav_bringup/share/rm_nav_bringup/urdf/` 对**已存在**的文件是
逐文件符号链接（本轮实测：直接覆盖源文件后，launch 立刻用上新物理 —— 自转 9.8%→92.9%）。
只有"**新文件名**"才要 `colcon build --symlink-install --packages-select rm_nav_bringup`。

### 22.5 实测（改后）

| 量 | 改前 | **改后（μ=0.02）** |
|---|---|---|
| 台架 纯自转 `wz=0.8 × 2 仿真 s` | 9.713° = **10.6%** | **87.81° = 95.8%** |
| 台架 纯前进 `vx=0.30 × 1.5 仿真 s` | 0.3567 m = 79.3%（航向漂 −0.772°） | **0.3989 m = 88.6%**（航向漂 +0.207°） |
| 整栈 20 Hz 纯自转 | 8.94°/2 s = 9.8%（IMU wz 0.0107） | **85.16°/2 s = 92.9%**（IMU **0.7392** = 命令的 92.4%） |
| 整栈 20 Hz 纯前进 | 0.3315 m/2 s = 55.2% | **0.5895 m/2 s = 98.2%** |
| 整栈 200 Hz 纯自转 | （未测） | **87.37°/2 s = 95.3%**（IMU 0.7495） |
| 整栈 200 Hz 纯前进 | （未测） | **0.5925 m/2 s = 98.8%** |
| 盲推 25 s `vx=0.25` 撞实体 | （历史台架 0.3998 m、z 抬到 0.1531） | **0.4000 m、z 不变（0.1025）、pitch 0°** ⇒ 障碍仍挡得住、**不爬** |

**摩擦剂量-响应（本机台架，单刚体 robot11，`wz=0.8 × 2 s`，RMUL2026）**：

| 轮子 μ | 1（无标签=缺省） | 0.2 | 0.1 | 0.05 | **0.02** | 0.001 |
|---|---|---|---|---|---|---|
| 自转执行率 | **10.6%** | 66.9% | 82.3% | 89.6% | **95.8%** | 98.3% |

> ⇒ 与 §20.2 的默认模型曲线**同形**（默认模型 14.1/72.2/84.6/90.5/94.9/97.9%）：μ≥0.05 才到 ~90%，
> 0.02 才留出余量。**μ=0.02 是"速度驱动插件"的标定值，不是轮胎物理**（同 §20.6#7）。

### 22.6 几何不变性（改前 vs 改后：`gz sdf -p` 逐项对比）

生成器每次写文件前自证；此外本节把改前/改后的 **SDF 文本**直接 diff：

```
$ diff base.sdf mu.sdf | grep -E '^[<>]' 的统计
  113 ×  <ode/>                     →  <ode> + <mu>0.02</mu> + <mu2>0.02</mu2> + </ode>
  其它差异：0 行
```

| 项 | 改前 | 改后 | 判据 |
|---|---|---|---|
| SDF link / joint 数 | 1 / 0 | **1 / 0** | 单刚体不变 |
| collision 条数 | 113（109 box + 4 cyl） | **113（109 box + 4 cyl）** | 逐条 AABB 相等（最大偏差 **5e-6 m**，阈值 5e-5） |
| 轮子 | `r=0.058`、`L=0.0452/0.0453`、中心 `(±0.172456, ±0.181488, −0.044502)` | **逐字相同** | 与**未塌陷**模型（`--chassis articulated`）FK 到 base_link 的 l6…l9 原点**逐位相同** |
| 整车最低点 | −0.102502 m（cylinder = 轮子） | **−0.102502 m（轮子）** | 非轮几何最低 −0.077994 m ⇒ 只有轮子触地 |
| base_link 离地 | 0.1025 m（`/odom_ground_truth` z = **0.102494…0.102499**） | 同（z=0.102494/0.102499） | 落地后静止（12 s 漂 **0.2 mm/s**） |
| SDF 合并质量 / Izz | 9.56212 kg / 0.129172 kg·m²（= 根 9.4121 + livox 0.14 + imu 0.01） | **逐字相同** | 与"全 link 平行轴定理"合成值 9.562116 / 0.129172 相等 |
| 质心（SDF inertial pose） | `0.003499 0.009093 0.07471` | **逐字相同** | — |
| 安装关节 `body_to_livox`（`robot11_mount:=sensor`） | xyz `0.000561701465058485 0.130915824456595 0.15702816968305`、rpy `-0.523598775598293 0 0`（= **roll −30°**） | **逐字相同** | 生成物里的 joint 未动 |
| `h`（雷达离地） | 0.102499 + 0.15702817 = **0.259527 m** | **同** | 运行期 `h_urdf=0.2595 m`、`h_meas=0.2616 m`（INFO） |
| 碰撞 AABB 集合 | 113 条 | **逐条相等** | 生成器自证 `aabb_identical=true`、`aabb_max_dev_m=5e-6` |

> 一句话：**这次只加了 `<surface><friction><ode><mu>/<mu2>`，没有动任何一块几何、质量、惯量、安装位姿**。

### 22.7 代价（诚实）：坡上会滑 —— μ ⇒ 最大静摩擦角

平面速度驱动（`planar_move` 直接设速度）**爬坡不受影响**（不需要抓地）；但**停住**时能站住的
最大坡度 = 静摩擦角 `atan(μ)`：

| μ | 0.001 | **0.02（本档）** | 0.05 | 0.1 | 0.2 | 1（改前） |
|---|---|---|---|---|---|---|
| `atan(μ)`（理论） | 0.06° | **1.15° ≈ 1.1°** | 2.86° | 5.71° | 11.31° | 45° |
| 台架自转执行率 | 98.3% | **95.8%** | 89.6% | 82.3% | 66.9% | 10.6% |

> 重新生成（一条命令）：`python3 tools/scripts/regress/robot11_make_sim_xacro.py --wheel-mu 0.1`
> ⇒ **自转 ≈82.3%（台架实测）、可站 ~5.7° 坡**；`--wheel-mu 0.05` ⇒ 89.6% / 2.9°；`--wheel-mu 0.02`
> （缺省）⇒ 95.8% / 1.1°。**本轮没有实测任何真实坡道**（RMUL/RMUC world 的坡度/台阶/低地形没重测，
> 与 §20.6#2 同一个未验证边界）。

### 22.8 下游冒烟（`mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor lio:=small_point_lio`）

命令：`world:=RMUL2026 mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor
lio:=small_point_lio`（**不传** `gravity_align_mode`：launch 收据 `gravity_align_mode=auto ⇒ 有效值=hybrid`），
`tools/scripts/tiltmount/nav_band_probe.py --settle 20 --window 20`（每次 100 个局部代价图样本 / 200 帧 `/scan`）+
`.tmp_r11fix/probe.py`（§13.4 的墙面证人）。跑了 **2 次**：

| 量 | 第 1 跑 | 第 2 跑 | §21 参考 | 判据 |
|---|---|---|---|---|
| local `n_lethal` 中位（min–max） | **342**（323–419） | **327**（298–346） | 275.5（265–290） | **>100 ⇒ 非空** ✅（数值高于参考带，见下注） |
| local `nearest_lethal_m` 中位 | **0.7082** | **0.7148** | 0.7079 | ≈0.70–0.77 ✅ |
| local `n_lethal_ahead`（2.0×0.6 m） | 25 | 26 | 31 | 同量级 ✅ |
| local `n_lethal_within_1m` | 31 | 32 | 37 | 同量级 ✅ |
| global `n_lethal` | 3727 | 3728 | 3725 | ✅ |
| global `nearest_lethal_m` | 0.4504 | 0.4488 | 0.4586 | ✅ |
| `/scan` 有限波束/帧（中位） | 773 | 773 | 775 | ✅ |
| `h_meas` | **0.2616 m / INFO**（Δ=0.0021） | **0.2616 m / INFO**（Δ=0.0020） | 0.2615 | ✅ 与 `sensor` 基线一致 |
| 声明帧地面垂距 `d_decl` | 0.2597（h_true 0.2596） | 0.2598 | 0.2596 | ✅ |
| 墙面残差（所有看得到的竖直面） | **−0.0000 / +0.0000 / +0.0001** | **−0.0000 / +0.0001** | 全 0.0000 | ✅ 插件修 `7a544bf` 仍成立 |
| `robot11_mount:=sensor` 参考口径 | `h≈0.2616`、墙残差 0 | 同 | 同 | ✅ **没有被本轮改动影响** |

> **注（诚实）**：local `n_lethal` 的 **327–342** 高于 §21 的 265–290。两跑的机器人 odom 静止位姿是
> `(0.030, 0.110)` / `(0.027, 0.134)`，§21 那跑是 `(0.0165, −0.0567)` ⇒ **局部代价图窗口落在场地的
> 不同位置**；而 `/odom_ground_truth` 实测机器人**没有滑**（12 s 漂 0.2 mm/s）、global lethal 与
> §21 一致（3727/3728 vs 3725）⇒ 差异来自 GICP/odom 的初始化落位，**不是**本轮摩擦把车挪了
> （§17 已记录 local lethal 对 7 mm 级位姿差能差 40 格）。**没有**跑改前模型的 nav A/B 去分离这两者。

### 22.9 没能验证 / 诚实清单

1. **μ 是给速度驱动插件标定的值**，不是从轮胎/地面材料推出来的（同 §20.6#7）；换世界/地面材质要重看 22.5 的曲线。
2. **摩擦写在根 link ⇒ 113 条 collision 全是 μ=0.02**（默认模型只给 4 个轮子写）。轮子是唯一接地点
   （最低 −0.1025 vs 非轮最低 −0.0780），所以**正常姿态**下与"只写轮子"等价；但**底盘/云台蹭到**
   墙/台阶时也会是 0.02（更容易滑）。这一条**没有专门测**（只测了正前方盲推：0.4000 m 停住、z 不变、不爬）。
3. **坡度/台阶/低地形没有重测**（`slope_speed` / `low_terrain` 的历史数字不适用于 μ=0.02）：
   >1.1° 的坡上停住会缓慢下滑；爬坡由插件直接设速度 ⇒ 理论不受影响，但**未实测**。
4. **nav 只跑了 2 次**，且 local lethal 高于 §21 参考带（22.8 注）；**没有**跑改前模型的 nav A/B。
5. **没有重建/提升地图资产**（与 §20 同：任务要求不动 `map/`、`PCD/`）；速度变快 ⇒ 覆盖跑图的里程/时间关系会变。
6. `robot11_mount:=plugin|urdf` 的 nav/下游**没有**重跑（本轮只跑 `sensor` 参考档）。摩擦只改物理，
   与点云/帧语义无关；`plugin` 档的 `h_meas` 修复（§21）不在本轮范围内。
7. **前进也变快**（台架 79.3%→88.6%、整栈 55.2%→98.2%）—— 这不是独立回归，是**同一条摩擦损失**
   被拿掉的另一面（任何让自转 ≥90% 的 μ 都会让前进接近 100%）。依赖"robot11 车速比"的历史数字要重读。
8. **200 Hz 只测了改后**（95.3%/98.8%），没测改前 200 Hz。
9. **台架的"2 仿真秒"用墙钟 2 s 近似**（RTF≈0.995–0.999），执行率一律按 `requested_sim_yaw_deg` 归一。
10. **几何不变性是"改前/改后 SDF 逐条 AABB + 质量/Izz + 安装关节"**，不是"运行时逐点重合"：
    没有做点云级/接触级 A/B（也不需要 —— 摩擦不参与几何）。
11. **`--chassis articulated` 的回退验证**是"去注释后模型体逐字节相同"（34573 B），不是整文件逐字节
    （文件头会多出 22.4 那段说明，HEAD 本来就有这个口径）。

## §23 第十四轮（**本机 date=2026-10-20T03:43–05:20**）：按参考流水线重建两个世界的资产 —— **覆盖率与绝对几何都没有赢过旧图，两个候选都不提升**；附带四条硬发现（LIO 发散 → `slam_toolbox` OOM、路线×跟随器耦合、`align.json` 口径错、3D 资产的 z 原点不是地面）

> **本节只新增**（append-only）：§1–§22 一个字都没改。写作时 §22 是最新 ⇒ 本节用 §23。
>
> 任务口径（父任务原文）：按**参考流水线** `mode:=mapping lio:=small_point_lio robot:=robot11
> robot11_mount:=sensor`（`gravity_align_mode` 不传 ⇒ `auto` ⇒ **hybrid**、`gravity_align_z_origin:=ground`）、
> `cloud_accumulator:=True`、`ground:=linefit`、`spin_speed:=0.0`，跑**有界**的覆盖路线，
> 重建 RMUC2026（先）与 RMUL2026（后）的 2D+3D 先验；**四道门（覆盖率 / 绝对几何 / 定位质量 / 自洽）
> 全过才提升**，任何一门不过就保留旧默认并逐条报"哪一门、为什么"。
>
> 隔离（与 §17/§22 同口径）：每跑私有 `HOME` / `ROS_DOMAIN_ID=137` /
> `GAZEBO_MASTER_URI=http://127.0.0.1:11737`、`gui:=False`、`world:=X` **不带前导 `--`**、
> `setsid` + `timeout` + `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**，收尾按进程组/URI 判归属）。
> 原始数据与**可复跑**脚本在 `.tmp_map2/`（gitignored）：`env.sh`（隔离）、`run_mapping_v2.sh`（建图整跑）、
> `run_nav_gate.sh`（GICP 定位门）、`build_truth_map.py`（**从 STL 真值**生成路线底图）、`map_cov_stats.py`
> （2D 覆盖率对照）、`wall_resid.py`（墙面证人，带偏航/平移自标定）、`floor_pcd.py`（资产地面高度）、
> `map_align_tf.py`（**用 TF** 量 map 系是否水平）、`map_odom_probe.py`（`map→odom` 常量偏置）、
> `odom_watch.py`（LIO 看门狗：|pose|>100 m 或 45 s 无新样本 ⇒ 退出码 2 ⇒ 调用方立刻停跑图）。
> 每跑目录里有 `meta.txt`（date/git_head/world_md5/launch_md5/livox_so_md5/domain/master）、`launch.log`、
> `drive.log`、`drive.json`、`health.json`、`gate.json`、`map_odom.json`。

### 23.1 一句话结论

> **四条门没有一条是"新资产赢"**：覆盖率 RMUC `known 168.7 m² vs 旧 389.9 m²（43%）`、
> RMUL `51.1 vs 94.8 m²（54%）`；绝对几何新 3D 的墙面残差 **8–23 cm、散布 0.2–0.4 m**（旧图 RMUL 是 3.8–4.9 cm、RMUC 是 10–12 cm）；
> 地面高度 **RMUC −0.104 m / RMUL +0.051 m**（`z_origin=ground` 的期望是 **0**）；
> GICP 在**新** PCD 上的 `score` 虽然更低（RMUC `0.00446 vs 0.0139 m²`），但 `map→odom` 要一次性修
> **`(0.282, −1.899, 0.181) m / 4.19°`**（旧图只要 `(−0.139, +0.086, −0.003) m / 1.18°`）⇒ 图本身与 odom 不自洽。
> ⇒ **两个世界都保留旧默认，一个字节没覆盖**（`.bak-20261020` 只是"换装门禁跑"的备份，见 §23.8）。
> 根因不是"配方写错"，而是四条**上游/环境**问题（§23.9）：① `lio:=small_point_lio` 在这台机器上
> **驱动就发散**（RMUL 两跑：一次 79 s 硬发散到 1e2 m 量级、一次 ATE 5.34 m；RMUC 第一跑 2 min 内发散
> ⇒ `slam_toolbox` 吃 16.6 GB ⇒ **被内核 OOM-kill**）；② 简化过的路线会让纯追踪**原地抖转**（v=0、wz=±1，
> 几分钟零位移且不触发卡死判据）；③ §16 用的 `map_world_align.py` 在 **hybrid/`odom_convention=sensor`** 档
> 量出来的"map 系斜 30°"是**口径错**（它把 `/odom` 的**雷达**位姿和真值的**base**位姿相减），
> 用 TF 重测 map 系是**水平的（roll +0.38° / pitch −0.07°）**。

### 23.2 配方（**实际命令**）+ 路线 + 时长

```bash
# ① 路线底图：**从 STL 真值**生成（不用旧默认图当底图 —— 旧图 free 里含 unknown(205)，
#    coverage_route 的 trinary 语义会把 205 当 free，路线会穿"未知区"）
python3 .tmp_map2/build_truth_map.py RMUC2026 --out .tmp_map2/truth_RMUC2026 \
    --window -26.0 -10.0 5.0 8.5          # free = 相对地面高度 <0.15 m 且局部坡度 ≤20°
python3 .tmp_map2/build_truth_map.py RMUL2026 --out .tmp_map2/truth_RMUL2026 \
    --window -3.4 -4.4 11.7 6.7

# ② 覆盖路线（clearance 0.50 = robot11 的**外接半径 0.4243** + 7.6 cm 余量；
#    min-waypoint-sep 0.3 + simplify 0.02 ⇒ 输出的是 A* 折线本身，不是被抽稀成锐角的弦）
python3 tools/scripts/mapping/coverage_route.py --map-yaml .tmp_map2/truth_<W>.yaml \
    --clearance 0.50 --lane-spacing <2.0|2.0> --goal-step <2.5|2.5> \
    --min-waypoint-sep 0.3 --simplify 0.02 --order lawnmower --speed 0.6 \
    --out .tmp_map2/route_<W>.json

# ③ 建图整跑（一条 bash 调用里：launch + 静止对齐 + 跑图 + map_saver + map_archive save + 体检 + 收尾）
bash .tmp_map2/run_mapping_v2.sh <tag> <W> <W>_v5 <route.json> <2D 前缀>
#   内部就是：
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=<W> mode:=mapping lio:=small_point_lio \
  mapper:=slam_toolbox ground:=linefit spin_speed:=0.0 robot:=robot11 robot11_mount:=sensor \
  gravity_align_z_origin:=ground map_name:=<W>_v5 map_autocontinue:=False cloud_accumulator:=True \
  nav_rviz:=False lio_rviz:=False gui:=False
#   跑图：coverage_drive.py --pose-source gt --speed 0.7 --lookahead 0.55 --yaw-gain 1.5
#         --yaw-rate-max 1.5 --rot-in-place-rad 2.8 --spin-all-turns 1.0 --spin-every 10
#         --final-spin 1.0 --stuck-sec 20 --save-2d <前缀>
#   落盘：map_saver_cli（2D）+ map_archive.sh save --name <W>_v5 --resolution 0.05（PCD + posegraph + 两个 sidecar）
```

**launch 收据（逐字，两跑相同）**：`gravity_align_mode=auto ⇒ 有效值=hybrid（lio=small_point_lio）`；
`gravity_align_rpy=[-30.000000, 0.000000, 0.000000]°`；`z_origin=ground gravity_align_h→0.259527 m`；
`gravity_align_h_check: h_urdf=0.2595 m h_meas=0.2597 m |Δ|=0.0002 m ⇒ INFO`（= `sensor` 档的
偏移修复 `72c7c07` 在**建图链路**上也是好的）。

| 项 | RMUC2026 | RMUL2026 |
|---|---|---|
| 路线（真值底图上的可行驶连通域） | **56.7 m²**（clearance ≥0.50 m） | **38.2 m²** |
| 路线：航点 / 总长 / bbox（map 系） | 49 / **79.34 m** / `x[-8.93,3.08] y[-6.67,6.23]` | 25 / **40.12 m** / `x[-1.27,8.72] y[-1.88,4.62]` |
| 实跑：到位 / GT 轨迹长 | **48/49** / 64.24 m | **16/25（超时）** / 35.23 m |
| 原地转圈数（1 圈 @0.7 rad/s） | 4（+收尾 1） | 2 |
| 墙钟 / 仿真时长 / RTF | **387.7 s / 162.1 s / 0.42** | **525.2 s / 203.6 s / 0.39** |
| LIO vs 真值（`/odom` vs `/odom_ground_truth`） | ATE xy RMSE **0.124 m**、head 0.119 / tail 0.116（**不长漂**）、轨迹长比 1.068 | ATE xy RMSE **5.344 m**、head 3.084 / tail 6.080（**在长漂**）、轨迹长比 **2.965** ⇒ LIO 发散 |

> 两跑的 `RTF≈0.4` 不是这台机器的常态：`/clock` 实测 0.39–0.42，而同机**空载**时同一世界能到 ~1.0
> （§16 的 `RTF 0.995–0.999`）。第一跑（03:44）期间内核 OOM + swap 打满（`Swap 3/3 GB`）把 RTF 压到 0.40，
> 之后即使内存恢复也一直是 0.4x —— 本机 `gzserver` + 113 条 collision 的 robot11 在 Classic 里就是这个量级，
> **跑图时长要按 ×2.5 估**（§23.9#1 的 OOM 也发生在这一段）。

### 23.3 资产清单（旧 vs 新，全部留档，旧的一个字节没删）

| world | 槽位 | 旧默认（**提升后仍是它**） | 旧 md5 | 新候选（**未提升**） | 新 md5 |
|---|---|---|---|---|---|
| RMUC2026 | 2D | `map/RMUC2026.pgm`+`.yaml`，`577x326 @0.05`，`origin [-25.2,-9.09]` | `82e1742b…` / `683b07a8…` | `map/RMUC2026_v5_spl.pgm`+`.yaml`，**`392x412`**，`origin [-11.30,-10.44]` | `56dbd764…` / `4e9b3c85…` |
| RMUC2026 | 3D | `PCD/RMUC2026.pcd`，203,352 点 / 6.51 MB | `5b494a60…` | `PCD/RMUC2026_v5.pcd`，**201,432 点 / 6.45 MB**（累积 14.61 M 点，voxel 0.1） | `9c5ab7fa…` |
| RMUL2026 | 2D | `map/RMUL2026.pgm`+`.yaml`，`249x173 @0.05`，`origin [-2.2,-3.15]` | `6e45a932…` / `fd23dd82…` | `map/RMUL2026_v5_spl.pgm`+`.yaml`，**`279x320`**，`origin [-5.46,-7.70]` | `9b365993…` / `e2fd3254…` |
| RMUL2026 | 3D | `PCD/RMUL2026.pcd`，53,164 点 / 1.70 MB | `5d0d33eb…` | `PCD/RMUL2026_v5.pcd`，**97,148 点 / 3.11 MB** | `bbd45b48…` |

* 新候选的 sidecar（由 `map_archive.sh save` 写）：`map/<W>_v5.{posegraph,data,meta.yaml}` +
  `PCD/<W>_v5.{pcd,meta.yaml}`；`world/spawn_pose/map_start_pose/resolution/frames/files/file_sizes/versions` 全齐，
  PCD 侧另带 `accumulated_points / voxel_size / bbox / z_span / health_warning / dropped_* /
  skipped_frames_* / tf_frames_ok/failed / source`。
  ⚠️ **RMUC 的 sidecar 自己写着 `health_warning: true`**（z 跨度 2.746 m、xy bbox 40.0×49.8 m）—— 资产自检就不过。

### 23.4 门① 覆盖率（2D）：**两个世界都是回归**

口径：`map_cov_stats.py`（同一把尺子、同一 spawn；`missing = 旧图已知(free|occ) 而新图未知/出窗`）。

| 量 | RMUC 旧 | RMUC 新 | RMUL 旧 | RMUL 新 |
|---|---|---|---|---|
| 栅格 / bbox（m） | `577x326` / **28.85 × 16.30** | `392x412` / **19.60 × 20.60** | `249x173` / 12.45 × 8.65 | `279x320` / 13.95 × 16.00 |
| 窗口面积 | 470.25 m² | 403.76 m²（86%） | 107.69 m² | 223.20 m²（**207%**，被漂移撑大） |
| **已知**（free+occ） | **389.9 m²** | **168.7 m²（43%）** | **94.8 m²** | **51.1 m²（54%）** |
| **占用格数**（occ） | **9769**（24.4 m²） | **7142（73%）** | **3713**（9.28 m²） | **4885（132%）** |
| 自由格数 / 未知 | 146198 / 80.3 m² | 60343 / **235.0 m²** | 34195 / 12.9 m² | 15551 / **172.1 m²** |
| 旧图已知被新图覆盖 | — | **33.6%** | — | **41.1%** |
| 缺区 `missing`（其中落在新窗口内） | — | **258.8 m²**（76.2 m²） | — | **55.9 m²**（43.9 m²） |
| 新图独有 `new_only` | — | 37.6 m² | — | 12.2 m² |
| 占用格"厚度"（到最近自由格）p50/mean | 0.05 / 0.075 | 0.05 / 0.062 | 0.05 / 0.082 | 0.05 / 0.063 |

**判读**

1. **RMUC 的覆盖率回归是"物理"的，不是配方**：`robot11` 的碰撞足印是 **0.600×0.600 m**
   （URDF 109 box + 4 cylinder，内切 0.300 / **外接 0.4243**），而真值 2D 上两个半场之间的通道
   **最窄 0.48–0.52 m**（clearance ≥0.24 m 时两半场还连通、≥0.26 m 就裂成 72 / 87 m² 两块）
   ⇒ **这台车过不去**。可达域只有出生点所在半场 **56.7–72.2 m²**；`/scan` 的 `range_max=10.0 m`
   又限制了 2D 栅格能长到多远 ⇒ 新图的已知区最多到"路线最西点 −8.93 再往西 ~2.4 m"（实测窗口西边界
   **−11.30**，因为再往西就被中墙挡住、没有回波），而旧图西边界是 **−25.2**。
   **旧图能覆盖全场，最可能是它出自一次"人工遥操把车硬挤过那条 0.48 m 缝"的会话**（旧图中心那两条
   "S" 形黑带就是当年插件偏置把中墙抹开的样子；`docs/worlds.md` §6 也记过"0.45 m 缝、余量 5 mm/边 ⇒
   实际过不去"）。这条**是推断、没有当年的会话日志可查**，列进诚实清单 §23.10#10。
2. **RMUL 的窗口反而更大（13.95×16.00 vs 12.45×8.65）但那是假象**：新 PCD 的 xy bbox 是
   `19.3 × 20.2 m`（房间只有 12.6 × 8.6 m）—— LIO 漂移把点云甩到房间外，栅格跟着长；**已知面积只有旧图的 54%**。
3. **占用格数不是"越多越好"**：RMUL 新图的 occ 反而高 32%（4885 vs 3713），但同一次跑的 ATE 是 5.34 m ⇒
   多出来的占用格是**漂移/鬼影**。这也是本节对"用 occ 计数当覆盖判据"的订正：**必须与 known 面积、缺区、
   绝对几何一起看**。

### 23.5 门② 绝对几何：墙面证人 + 地面高度 **都不过**

**方法**（`wall_resid.py`，与 §13.4/§16.6 同一把尺子）：取世界网格（STL）里**从机器人可见**的竖直面，
在资产里取落进该面 ±0.45 m、且在面切向跨度内的点/占用格，1D 直方图定峰 + ±2 cm 中位精修，
`残差 = 资产面 − 网格面`（沿面法向）。因为旧资产没有"同一跑实测的 R_world←map"，本工具对**每个资产**
做一次**偏航单参数最小二乘**（对每个 dyaw 用各面残差解析求最优平移 `min_t Σ(r_i+n_i·t)²`，
取"扣掉平移后残差 RMS 最小"的 dyaw），三组数都报：`raw`（spawn 平移 + 偏航 0）/ `yawfit`（最优偏航、
**不加平移**）/ `fullfit`（偏航+平移都拟合 = **形状误差**）。新旧同法 ⇒ 可比。

**RMUC2026（3D PCD，23 个可见面）**

| 量 | 旧 `PCD/RMUC2026.pcd` | 新 `PCD/RMUC2026_v5.pcd` |
|---|---|---|
| `raw` \|残差\| p50 / max | 0.1232 / 0.2250 m | **0.0820 / 0.2413 m** |
| `fullfit` \|残差\| p50 / max / RMS | 0.1003 / 0.2680 / 0.1177 m | **0.0841 / 0.2540 / 0.1098 m** |
| 各面残差散布（`spread_p90`） | 0.17–0.44 m | **0.19–0.42 m** |
| 典型面（旧 → 新） | `x=10.5 +0.031`、`y=−7.23 +0.119`、`y=−4.71 +0.083` | `x=10.5 +0.056`、`y=−7.23 +0.097`、`y=−4.71 +0.189`、`y=−6.90 −0.215` |

**RMUL2026（3D PCD，9 个可见面）**

| 量 | 旧 `PCD/RMUL2026.pcd`（点太稀，只有 2 面够样本） | 新 `PCD/RMUL2026_v5.pcd` |
|---|---|---|
| `raw` \|残差\| p50 / max | **0.0437 / 0.0492 m**（`x=+2.5 −0.0381`、`y=−8.5 +0.0492`） | **0.1507 / 0.2288 m**（`x=+2.5 +0.151`、`y=+0.5 +0.204`、`x=+2.22 +0.229`） |
| `fullfit` p50 / max / RMS | —（面数 <3，拟合不成立） | 0.0886 / 0.1986 / 0.1134 m |
| 各面散布 `spread_p90` | 0.049 / 0.106 m | **0.17–0.40 m** |

⇒ **新 3D 资产的墙面位置没有旧图准**（RMUC 的 p50 从 0.100 变 0.084 只是"略好"，但 max 与散布同量级；
RMUL 从 3.8–4.9 cm 掉到 15–23 cm）。原因见 §23.9#1：`cloud_accumulator` 用的是
**`livox_frame→map` 的全链 TF**，`map→odom` 一旦被 `slam_toolbox` 修正/跳变，早先累积的点不会跟着动
⇒ 同一面墙被画成**多条弧带**（§23.9#1 的"鬼影"），残差散布 0.2–0.4 m 就是它。

**地面高度（`floor_pcd.py`：全局 z 直方图峰 + 各距离带复算；`z_origin=ground` ⇒ 期望 0）**

| 带（距出生点） | RMUC 新 | RMUL 新 |
|---|---|---|
| 全局峰 | **−0.1043 m**（22747 内点，resid p50 2.7 mm） | **+0.0509 m**（4431 内点，resid p50 6.0 mm） |
| 0.8–1.5 / 1.5–3 / 3–6 / 6–10 / 10–20 m | −0.1046 / −0.1044 / −0.1046 / −0.1040 / −0.1038 | +0.0506 / +0.0506 / +0.0494 / +0.1519(低矮结构顶面) / +0.0497 |
| 旧图对照（同工具） | −0.0531 m | 不可用（§16.6 已记：地面点少 + 1 cm 台阶） |

⇒ 门② **不过**：地面不在 0（RMUC −10.4 cm、RMUL +5.1 cm；判据是 ±6 mm 的 mesh 台阶以内），
且**两次的符号相反** ⇒ 不是"常数校准错"而是**逐跑的 z 原点/漂移**（§23.9#4）。

**map 系是否水平（`map_align_tf.py`，**用 TF `odom→base_link`**，不是 `/odom`）**：
RMUC 跑中静止段 4 个样本 `roll +0.38° / pitch −0.07° / yaw +0.55°`、`t_wm=(10.839, 2.617, 0.061) m`
⇒ **map 系是水平的**（对照：`map_world_align.py` 在同一跑给出 `roll +30.13°` —— 那是**口径错**，
它拿 `/odom`（hybrid 档发的是**雷达**位姿 `T_ol`）减真值（**base** 位姿），减出来的就是 `livox←base`
的 −30° 安装倾角。§16.4 坑 3 的"map 系斜 30°"是同一个错口径的读数，**不能再用它判 map 系**）。

### 23.6 门③ 定位质量（GICP，`mode:=nav localization:=gicp` 静止，只订阅）

口径与 §16.7 逐字一致：`lio:=fastlio robot:=robot11 robot11_mount:=sensor`、`nav_rviz:=False
spin_speed:=0.0`、`initial_pose=[0,0,0]`、门限 `allow_xy` 实测恒 0.30 m / `allow_yaw` 6°、
窗口 = 仿真 15 s（本机 RTF 0.4 ⇒ 墙钟 ~40 s）+ `map_odom_probe.py` 另取 `map→odom` 常量偏置。

| 臂 | world | PCD | 采纳率 | innov p50 / p95 / max（m） | score p50（m²） | `map→odom` 常量偏置 |
|---|---|---|---|---|---|---|
| 旧（§16.7 同口径） | RMUC2026 | `RMUC2026.pcd`（6.51 MB） | 100% | 0.00943 / 0.02460 / 0.04629 | 0.0139 | `(−0.139,+0.086,−0.003) m / 1.18°` |
| **新** | RMUC2026 | `RMUC2026_v5.pcd`（6.45 MB） | **100%**（拒绝 0） | **0.00840 / 0.03733 / 0.04926** | **0.00446**（3.1× 好） | **`(+0.282,−1.899,+0.181) m / 4.19°`** ❌ |
| 旧（§16.7 同口径） | RMUL2026 | `RMUL2026.pcd`（1.70 MB） | 100% | 0.01065 / 0.02510 / 0.07429 | 0.0051 | `(−0.057,+0.130,+0.016) m / −0.16°` |
| **新** | RMUL2026 | `RMUL2026_v5.pcd`（3.11 MB，97,148 点） | **100%**（拒绝 0） | **0.00279 / 0.02467 / 0.14541** | **0.01030**（旧 0.0051 ⇒ **差 2×**） | **`(−0.172,−1.391,+0.204) m / −3.66°`** ❌ |

**判读**：新 RMUC 的 `score` 更低（0.00446 vs 0.0139）**不能当作"图更好"**：GICP 在出生点附近匹配的是
**近场几何**（那部分基本没被污染），而污染在远处的弧带上；真正露馅的是 `map→odom` —— 新图要一次性修
**1.9 m / 4.2°**（旧图只要 0.16 m / 1.2°）。⇒ 这张图与 odom **不自洽**，3D 定位可用性差。

### 23.7 门④ 自洽：`map_asset_guard check` + sidecar

* 新会话（`RMUC2026_v5` / `RMUL2026_v5`）：`map_asset_guard.py check --kind posegraph
  --name <W>_v5 --world <W> --resolution 0.05` ⇒ **`resume`（may_load=True，退出码 0）**，
  存档 `world` / `spawn`（RMUC `10.925,2.525,0.2,0`、RMUL `4.3,3.35,0.2,0`）与本次跑一致 ⇒ 隔离链是通的。
* 默认资产（`map/<W>.pgm|yaml`、`PCD/<W>.pcd`）**没有** sidecar（它们是"先验资产"不是"会话存档"，
  仓库惯例见 `docs/map_assets.md` §1）⇒ `check` 判 `fresh`（退出码 0）。本轮没有改变这个形态。
* 但 **PCD sidecar 的 `health_warning: true`**（RMUC，z 跨度 2.746 m、xy bbox 40.0×49.8 m）——
  资产自检本身就把这份 3D 候选标成"不健康"。

### 23.8 提升决定：**两个世界都不提升**（旧默认一个字节没覆盖）

| 门 | RMUC2026 | RMUL2026 | 结论 |
|---|---|---|---|
| ① 覆盖率（bbox & occ ≥ 旧 + 无缺区） | known 43%、occ 73%、缺区 258.8 m² | known 54%、缺区 55.9 m² | ❌ |
| ② 绝对几何（墙残差 ≈0 + 地面 ≈0±6 mm） | 墙 8–25 cm / 散布 0.4 m；地面 −0.104 m | 墙 15–23 cm；地面 +0.051 m | ❌ |
| ③ 定位质量（创新远低于门限、score 优于旧、`map→odom` 小） | 创新/score 好，但 `map→odom` **1.9 m / 4.2°** | score **差 2×**（0.0103 vs 0.0051）+ `map→odom` **1.4 m / 3.7°** + 建图 ATE 5.34 m | ❌ |
| ④ 自洽（guard + sidecar） | guard `resume` ✅，但 sidecar `health_warning: true` | guard `resume` ✅ | ⚠️ |

**已产生的备份（按 `docs/map_assets.md` §4.2 的 `.bak-<YYYYMMDD>` 约定，就地、不删）**：

```bash
src/rm_nav_bringup/map/RMUC2026.pgm.bak-20261020     # md5 82e1742b54e2cf3e33ac8891ca90f7f5（= 当前默认，逐字节）
src/rm_nav_bringup/map/RMUC2026.yaml.bak-20261020    #     683b07a83120957aad1cbbd69d76bf43
src/rm_nav_bringup/PCD/RMUC2026.pcd.bak-20261020     #     5b494a6016a6deb3276b2a6fb48ed1d1
src/rm_nav_bringup/map/RMUL2026.pgm.bak-20261020     #     6e45a9327e99d57883881f7a7d10c3d0
src/rm_nav_bringup/map/RMUL2026.yaml.bak-20261020    #     fd23dd82c0aa63b6c2fdf3644c7c9253
src/rm_nav_bringup/PCD/RMUL2026.pcd.bak-20261020     #     5d0d33eb5219d9416606a7b0f5e34028
```

> ⚠️ 这六个 `.bak-20261020` **不是"提升备份"而是"换装门禁跑"的备份**：门③/④ 要求"GICP 读的是**新** PCD、
> nav 用**新** 2D 图"，而 `PCD/<world>.pcd` 的路径是 launch 拼死的（没有参数可改，见 `docs/map_assets.md` §1）
> ⇒ 只能把候选 `cp -f` 就位再换回来。换装前后 md5 逐字节相同（上表），**当前默认仍是旧那一份**。
> 2D 侧不用换：`map_yaml:=` 是 launch 参数，可以直接指到候选 `.yaml`。
> 回滚（万一状态不对，一条命令）：
> ```bash
> cd src/rm_nav_bringup && D=20261020
> cp -f map/RMUC2026.pgm.bak-$D map/RMUC2026.pgm && cp -f map/RMUC2026.yaml.bak-$D map/RMUC2026.yaml && cp -f PCD/RMUC2026.pcd.bak-$D PCD/RMUC2026.pcd   # RMUL 同理
> ```

**建议（下一步，不在本轮范围）**

1. **先修 LIO 再谈重建**：`lio:=small_point_lio` 在"驱动 + 自转"下**两跑两发散**（§23.9#1）。
   在它稳之前，任何"参考流水线建图"都会产出鬼影 PCD；2D 侧反而稳（`slam_toolbox` 自己闭环）。
2. **RMUC 的全场覆盖需要放开"车过不去"这条**：要么用足印 ≤0.44 m 的模型（默认模型 0.44 m 也只有 2 cm/边，
   实测旧图那次是人工硬挤过去的），要么承认"全场 2D 先验只能由多会话拼"（本轮**没做**拼图）。
3. 若目标是"换一张更好的 3D 先验"，**判据要换成**：`map→odom` 常量偏置 + 墙面绝对残差 + sidecar `health_warning`，
   **不要**只看 GICP `score`（本轮 `score` 变好 3× 而图更差，就是这个反例）。

### 23.9 本轮的四条硬发现（都不是"配方写错"）

**① `lio:=small_point_lio` + `robot11+sensor` + hybrid 在**驱动**时发散，并把 `slam_toolbox` 拖进 OOM**

| 跑 | 时刻 | 现象 | 证据 |
|---|---|---|---|
| RMUC 第一跑（`rmuc_v5` @03:44，70.4 m 简化路线） | 驱动 ~2 min 内 | `/odom` 跑到 `(−7.0e4, −7.5e4, −9.3e4)`；`cloud_accumulator` 的累积 bbox 从 `y≤7.04` 一路长到 `y≤16.61`（场地 y 上限 7.12）；`slam_toolbox` 的 MessageFilter 从 03:48:09 起刷"queue is full" | `launch.log`、`PCD/RMUC2026_v5.meta.yaml` 的 `skipped_frames_jump`；`journalctl -k`：**`Out of memory: Killed process async_slam_tool (pid 974007) total-vm:26401960kB, anon-rss:16663000kB`** @03:52:52 ⇒ `slam_toolbox` exit −9，`map→odom` 冻结在 sim 171.2 s，整跑作废 |
| RMUL 第一跑（@04:17，78.5 m 路线） | 驱动 79 s | `/odom` 跑到 `(−103, 18, −46)` | `health.json`：`{"reason":"diverged","pose":[-103.32,18.09,-46.39]}`；看门狗退出码 2 ⇒ 停跑图（**机器没被拖进 OOM**） |
| RMUL 第二跑（@04:23，40.1 m 路线） | 全程 | 没到 100 m 阈值但**一直在漂**：ATE xy RMSE **5.344 m**、head 3.084 → tail 6.080、轨迹长比 2.965；PCD xy bbox `19.3×20.2 m`（房间 12.6×8.6） | `rmul_v5.drive.json`、`floor.json` |
| RMUC 第二跑（@04:11，79.3 m 密集路线） | 全程 | 轨迹没发散（ATE 0.124 m、head≈tail）但 **PCD 有鬼影**：31.2% 的点落在场地 xy 之外（`y` 到 +27.7 m），墙面残差散布 0.2–0.4 m | `pcd_stats`、`wall_pcd.json`、`pcd_xy.png` |

* 触发点最可能是**自转/急转期间的位姿链**：`cloud_accumulator` 用的是 `livox_frame→map` 的**全链 TF**，
  而 `map→odom` 由 `slam_toolbox` 在自转时修正/跳变，早先累积的点不跟着动 ⇒ 同一面墙出现**弧状重影**；
  LIO 自身在 `robot11_mount:=sensor`（点云真在斜 30° 的传感器系）+ hybrid 下对**转动**最敏感。
  **未做判决性实验**（要单独隔离"自转 vs 直行"、"LIO 发散 vs slam_toolbox 跳变"两个因素），列为本节最大的未验证项。
* **看门狗是本轮新增的必备件**（`odom_watch.py`）：第一跑就是没有它，才让 `slam_toolbox` 吃到 16.6 GB 并
  把整机 swap 打满（随后 RTF 从 ~1.0 掉到 0.4）。后三跑都有它。

**② 路线"抽稀成锐角"会让纯追踪原地抖转（不是 LIO 的错，也不是卡死判据能抓到的）**

`coverage_route.py --simplify 0.2 --min-waypoint-sep 1.5` 把 A* 折线压成 1.5–6 m 的**长弦**后，
`coverage_drive` 的纯追踪在弦的拐角处把 `s` 投影卡住（>1.0 m 偏差时 `s` 冻结）⇒ 胡萝卜落到车后 ⇒
`|herr|>rot-in-place` ⇒ **原地转 ±1.0 rad/s 来回摆**，实测 11 s 内 GT 位移 **0.002 m**、
`/cmd_vel_chassis` 只有 `wz=±1.0`、`vx=0`，而"运动检测"每次都能测到转动 ⇒ **卡死判据永不触发**。
换成**不抽稀**（`--simplify 0.02 --min-waypoint-sep 0.3`）+ `lookahead 0.55` + `yaw-gain 1.5` 后，
RMUC 的执行率回到 **0.65 m/s 仿真（命令 0.70，93%）**、48/49 个航点跑完。
⇒ **建图跑图的路线不要抽稀**（这条对以后所有跑图复用）。

**③ §16 的 `map_world_align.py` 在 hybrid/`odom_convention=sensor` 档量错了（"map 系斜 30°"是假的）**

`lio:=small_point_lio` + 有效 `gravity_align_mode=hybrid` 时，`/odom` 发的是**雷达位姿 `T_ol`**
（`frame_id=odom`、`child_frame_id=livox_frame`，见 launch/small_point_lio 的启动收据），
而 `/odom_ground_truth` 是**base_link** 位姿 ⇒ `R_wm = R_world←base · R_map←livox⁻¹ = R_livox←base`
= **−30° 安装倾角**。§16.4 坑 3 记的 "`robot11_mount:=sensor` + `small_point_lio` 的 map 系斜 −30.0…−30.15°"
与本节第一跑 `align.json` 的 `roll +30.13°` 都是这个口径错。**换成 TF `odom→base_link` 重测：roll +0.38°、
pitch −0.07°** ⇒ hybrid 档的 map 系是水平的，**这一档可以建 2D 栅格**（本轮的 2D 图确实没斜）。
新增工具：`.tmp_map2/map_align_tf.py`（以后判"map 系平不平"一律用它，不要用 `/odom` 直接减真值）。

**④ `z_origin=ground` 没有让 3D 资产的地面落在 0：RMUC −0.104 m、RMUL +0.051 m（符号还相反）**

`gravity_align_h_check` 是好的（`h_meas 0.2597 vs h_urdf 0.2595，|Δ|=0.0002 ⇒ INFO`），所以**云自己的形状**没问题；
但落进 map 系后地面在 −0.104 / +0.051 ⇒ 偏差在 **odom 的 z 原点或位姿链**上，而且**逐跑变号**
⇒ 不是常数校准。RMUC 的 `/odom` 起始 z = **0.148**（按 URDF 应是 livox 离地 0.2595 或 IMU 离地 0.2162）。
**本轮没有定位到具体哪一行**（不许动 `src/rm_localization/**`），列为未验证项；GICP 侧表现为
`map→odom` 的 z 分量要 +0.181 m。

### 23.10 本轮没能验证 / 诚实清单

1. **没有一次"干净"的建图跑**：四次尝试里只有 RMUC 第二跑的**轨迹**是好的（ATE 0.124 m、head≈tail），
   它的 **PCD 仍有鬼影**。所以"参考流水线在正常情况下能建出多好的图"这个基准，本轮**没拿到**。
2. **LIO 发散的根因没定位**：没做"自转 vs 直行"、"LIO vs slam_toolbox 跳变"的隔离实验；
   也没验证"关掉 `--spin-all-turns`/`--final-spin` 后是否还发散"。**没有改任何 `src/rm_localization/**`**。
3. **§16.7 的"旧图"数字是引用的**（同机同口径 2026-10-20T00:0x 的实测），**本轮没有重跑 before 臂**
   （时间/机器都被 OOM 事故与四次重跑吃掉）；两臂的 `lio:=fastlio ... settle 25/window 20` 与本节
   `settle 15/window 15` 差 10 s 窗口，趋势可比、绝对值不可逐位比。
4. **2D 图没有跑** `verify_low_terrain.py` / `map_prior_ab.py`（覆盖率一项就足以否掉提升，且这两个工具的
   真值链路只支持 RMUC2026，RMUL 那份 STL 是 ASCII 且坐标系要另转）。改用 `map_cov_stats.py`
   （同 spawn、逐物理位置比 known/missing/occ 厚度）+ `wall_resid.py grid` 模式。
5. **GICP 只测了静止、一个位姿（出生点）、一跑**：没测 `amcl`/`small_gicp`、没测运动中的跟踪、
   没测 `mode:=nav` 在新 2D 图上的 costmap/规划（新 2D 没提升，也没被导航用过）。
6. **墙面证人只覆盖"从出生点看得见"的面**：RMUC 23 面、RMUL 9 面（其中 `x=2.22 / y=0.22` 是
   "相邻面进窗"的已知假读数，§13.10#5；新旧都受影响）。场地远端没有独立真值。
7. **`map→odom` 只取了 12 仿真秒的中位数**（`n≈2400` 样本，spread 3 cm / 0.38°），没测长时稳定性。
8. **RMUL 的跑图超时**（525 s / 16 of 25 航点）：不判"路线跑不完"，因为 LIO 已经在漂，继续跑只会更脏。
9. **没做多会话拼图**：RMUC 西半场只能靠"另一次会话 + 拼"或"换小足印模型"，本轮**明确不做**。
10. **"旧图是怎么覆盖全场的"是推断**：没有 2026-10-07 那次会话的位姿/遥操记录，只能说"以 robot11 的足印
    在仿真里过不去"（本轮实测 + `docs/worlds.md` §6），不能说"当年一定是人工挤过去的"。
11. **`health_warning: true` 的判定阈值没有复核**（`cloud_accumulator` 里 z 跨度 3.0 m 的告警线；
    RMUC 是 2.746 m 却仍被标 true —— 可能还有 bbox/xy 一项，**没查**）。

### 23.11 一句话给用户

> **"按修好的插件重建"这次被两件更上游的事挡住**：① `lio:=small_point_lio` 在**驱动**时会发散
> （RMUL 两跑：一次 79 s 硬发散、一次 ATE 5.34 m；RMUC 第一跑 2 min 内发散并把 `slam_toolbox` 吃到
> 16.6 GB、被内核 OOM-kill）；② `robot11` 的足印 0.60 m、而这个场地两半场之间只有 0.48–0.52 m
> ⇒ **RMUC 的可达域只有 56.7–72.2 m²**，覆盖率不可能追平"人工硬挤过缝"建出来的旧图。
> 四条门（覆盖率 / 绝对几何 / 定位 / 自洽）**没有一条是新资产赢**，所以**两个世界都保留旧默认**
> （旧的一个字节没覆盖；`*.bak-20261020` 是换装门禁跑的备份，md5 与当前默认逐字节相同）。
> 顺带纠正 §16 的一条结论：`map 系斜 30°` 是**探针口径错**（`/odom` 在 hybrid 档发的是雷达位姿），
> 用 TF 重测 map 系 **roll +0.38°**，是平的。**下一步**：先让 SPL 在驱动下稳住（或改用 `fastlio`
> 建 3D 先验），再谈重建；建图路线**不要抽稀**（§23.9#2）。

## §24 第十五轮（**本机 date=2026-10-20T04:45–05:35**）：按 **默认模型 + `lio:=fastlio`** 重建资产 —— 鬼影的**机制**被拿掉、RMUL 覆盖率**赢过旧图**，但 RMUC 覆盖率只有旧图 **21.6%**、RMUL 的**绝对几何/地面不过** ⇒ **两个世界仍不提升**；附一条硬发现：**FAST-LIO 在 RMUC 的窄走廊里退化到发散**

> **本节只新增**（append-only）：§1–§23 一个字都没改。写作时 §23 是最新 ⇒ 本节用 §24。
>
> 任务口径（父任务原文）：这一轮**换成旧图当年真正用的那条流水线** —— **默认模型（`robot:=''`）+
> `lio:=fastlio`**（`robot11` 的外接半径 0.60 m 过不去 RMUC 半场之间那条 0.48–0.52 m 的通道，
> 而默认模型 0.2×0.3 的车体对角线只有 0.36 m）、`cloud_accumulator:=True`、`ground:=linefit`、
> `spin_speed:=0.0`；先 RMUC2026 再 RMUL2026；四道门（覆盖率 / 绝对几何 / 定位 / 自洽）全过才提升。
> **额外硬要求**：§23.9#1 的"鬼影"必须先修掉（`map→odom` 被 slam_toolbox 修正、已累积的点不跟着动），
> 并且要用墙面残差给出证据。
>
> 隔离（与 §23 同口径）：每跑私有 `HOME` / `ROS_DOMAIN_ID=137` / `GAZEBO_MASTER_URI=http://127.0.0.1:11737`、
> `gui:=False`、`world:=X` 不带前导 `--`、`setsid` + `timeout` + `tools/scripts/lib/scoped_cleanup.sh`
> （**无全机 pkill**）、`map_autocontinue:=False`（每次从零建）。原始数据与可复跑脚本在 `.tmp_map2/`
> （gitignored）：`env.sh`、`run_mapping_v3.sh`（本轮建图整跑）、`run_nav_gate_v3.sh`（定位门，两个臂）、
> `odom_rec.py`（`map→odom` 时间序列录波）、`pcd_odom_to_map.py`（落盘后**一次**刚体搬运）、
> `channel_probe.py`（从 STL 真值量"两半场之间最宽的一条通路"）、`make_transit_route.py`
> （两半场各自成团 + 窄通道**直行**）、`route_close_loop.py`（合法回程）、`pcd_xy_plot.py`（俯视证据图）、
> 以及 §23 留下的 `build_truth_map.py` / `map_cov_stats.py` / `wall_resid.py` / `floor_pcd.py` /
> `map_align_tf.py` / `map_odom_probe.py` / `odom_watch.py`（这一批**原样复用**，同一把尺子）。

### 24.1 一句话结论

> **鬼影的机制被修掉了、2D 覆盖率在 RMUL 上**首次**赢过旧图（已知 96.95 m² vs 旧 94.77 m²，旧图已知格
> 被新图**逐格覆盖 99.9%**），但四道门仍然**没有一条能同时全过**：
> ① RMUC 的已知面积只有 **84.19 m² = 旧图 389.92 m² 的 21.6%**（因子 **0.216 < 0.7** ⇒ 触发任务里
> 预写的 STOP RULE）；② 绝对几何：RMUL 墙面**原始**残差 p50 **0.155 m**（旧图 0.0437 m）、地面在世界系
> **−0.047 m**（判据 ±0.006 m）；③ 定位：RMUL 的 GICP **采纳 100%、创新 0.005–0.018 m**、
> `map→odom ≈ (−0.151, +0.18, +0.26) m / 0.2°`（xy 0.24 m vs 旧 0.14 m，**同量级、远小于 §23 的 1.4 m**）；
> ④ 自洽：`map_asset_guard check` 对 RMUL 的 posegraph/PCD 都判 **resume**、PCD sidecar `health_warning: false`。
> ⇒ **两个世界都保留旧默认，一个字节没覆盖**（本轮**没有**产生"提升备份"，只产生了"换装门禁跑"的备份，
> 见 §24.8）。RMUC 覆盖率输掉的原因**不是配方**，是这条：**FAST-LIO 在 RMUC 的窄走廊里连续
> "No Effective Points!" ⇒ `/odom` 发散**（六次尝试、四次是同一条走廊），而 RMUC 的可行驶域**就是**由
> 0.4–0.5 m 宽的走廊串起来的一圈（§24.10#1）。

### 24.2 配方（**实际命令**）+ 路线（六条）+ 时长

```bash
# ① 路线底图：**从 STL 真值**生成（§23 的同一份脚本，这一轮把阶闸从 0.15 收到 **0.08** —— 见 §24.10#2）
python3 .tmp_map2/build_truth_map.py RMUC2026 --out .tmp_map2/truth08_RMUC2026 \
    --window -26.0 -10.0 5.0 8.5 --step 0.08
python3 .tmp_map2/build_truth_map.py RMUL2026 --out .tmp_map2/truth08_RMUL2026 \
    --window -3.4 -4.4 11.7 6.7 --step 0.08

# ② RMUC 的"两半场 + 窄通道直行"路线（§24.10#3 的产物；最终**没跑它**，见下）
python3 .tmp_map2/make_transit_route.py --map-yaml .tmp_map2/truth08_RMUC2026.yaml \
    --clearance 0.24 --split -4.0 --y-max 6.40 --x-max 3.00 --east-only \
    --lane-spacing 2.0 --goal-step 1.5 --speed 0.40 --out .tmp_map2/route08_RMUC2026_wide.json
# RMUL：一条普通覆盖路线 + 合法回程（收尾停在出生点 ⇒ map←odom 的采样点 = 出生点）
python3 tools/scripts/mapping/coverage_route.py --map-yaml .tmp_map2/truth08_RMUL2026.yaml \
    --clearance 0.24 --lane-spacing 1.8 --goal-step 1.4 --min-waypoint-sep 0.3 --simplify 0.02 \
    --order nn --speed 0.40 --out .tmp_map2/route08_RMUL2026_c24.json
python3 .tmp_map2/route_close_loop.py --route .tmp_map2/route08_RMUL2026_c24.json \
    --map-yaml .tmp_map2/truth08_RMUL2026.yaml --clearance 0.24 \
    --out .tmp_map2/route08_RMUL2026_c24_loop.json

# ③ 建图整跑（一条 bash 调用里：launch + 静止对齐 + 跑图 + map_saver + map_archive save + 搬运 + 体检 + 收尾）
bash .tmp_map2/run_mapping_v3.sh <tag> <W> <W>_v6 <route.json> <2D 前缀>
#   内部就是：
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=<W> mode:=mapping lio:=fastlio \
  mapper:=slam_toolbox ground:=linefit spin_speed:=0.0 \
  map_name:=<W>_v6 map_autocontinue:=False cloud_accumulator:=True \
  cloud_accumulator_target_frame:=odom \
  nav_rviz:=False lio_rviz:=False gui:=False            # robot:= 缺省 = **默认模型**
#   跑图：coverage_drive.py --pose-source gt --speed 0.35~0.45 --lookahead 0.55 --yaw-gain 1.5
#         --yaw-rate-max 1.2 --rot-in-place-rad 2.8 --spin-all-turns 0.5 --spin-every 12
#         --final-spin 1.0 --stuck-sec 10~15 --stuck-max 3~4 --save-2d <前缀>
#   落盘：map_saver_cli（2D）+ map_archive.sh save --name <W>_v6 --resolution 0.05
#         + pcd_odom_to_map.py（**一次** map←odom，取 trace 末值）→ 覆盖 PCD 并补 sidecar
```

**launch 收据（逐字）**：`[gravity_align] gravity_align_mode=auto ⇒ 有效值=off（lio=fastlio；E1 补丁只
存在于 small_point_lio）`；`[map_archive] 3D 先验：map_autocontinue:=False ⇒ 不加载 PCD/…，从零点云
开始累积`；`cloud_accumulator 起：… target_frame=odom`；`lio_tf_adapter 启动：订阅 /odom -> 广播 TF
odom -> base_link`。⇒ 本档的 TF 链是 `map →(slam_toolbox)→ odom →(lio_tf_adapter)→ base_link →
(URDF)→ livox_frame`，`camera_init/body` 是孤立岛（与 §21 的契约一致）。

| 跑 | 路线（planner 长度 / 航点） | 结果 |
|---|---|---|
| `rmuc_v6` 第 1 跑（04:47，clearance 0.22 / lane 2.0 / 348 m，0.60 m/s） | 真值(step 0.15) + `coverage_route --order nn` | **车爬上 0.3 m 台阶后侧翻**（`/odom_ground_truth` z=0.3697、四元数 y=0.566/z=0.823），gt 轨迹 79.1 m 后卡死 ⇒ 看门狗停跑 |
| 第 2 跑（04:58，step 0.08 / clearance 0.24 / 293 m，0.35 m/s） | 同上（新底图） | 在**窄通道入口的 90° 直角弯**夹死（GT 60 s 不动、`倒退+转向` 也出不来）⇒ 停跑 |
| 第 3 跑（05:0x，`make_transit_route.py` 337 m，0.45 m/s） | 两半场 + 窄通道**直行**段 | 进通道后 **FAST-LIO `No Effective Points!` ×1799** ⇒ `/odom` 跑到 1e2 m ⇒ 看门狗停跑 |
| 第 4 跑（05:0x，直行段改成**常数 y**，337 m，0.40 m/s） | 同上 | **同一位置同一现象**（sim 144 发散）⇒ 停跑 |
| 第 5 跑（05:1x，只跑东半场**含**东侧窄条 68 m，0.40 m/s） | 裁 x≥-4 & y≤6.4 | 东侧窄条 `(3.26,−5.42)` 卡 10 s → 恢复 → sim 180 发散；2D 图跑飞成 **24017×36080**（1200×1800 m） |
| **第 6 跑 `rmuc_v6w`（05:25，只跑东半场**开阔区** 38.9 m，0.40 m/s）** | 裁 x∈[−4,3] & y≤6.4，clearance 0.24 / lane 2.0 / goal 1.5 | **轨迹干净**：29/30 航点、35.19 m、`motion_checks_failed=0`、ATE **0.1996 m**、轨迹长比 1.092、`z_err −0.1893±0.0075`；但 `serialize_map` **超时（rc 124）** ⇒ posegraph/PCD **没落盘** |
| **`rmul_v6`（05:21，RMUL 60.1 m / 29 航点，0.40 m/s）** | `coverage_route` + 回程 | **干净**：27/29 航点、57.91 m、2 次自转、RTF 0.804、ATE **0.1379 m**、轨迹长比 1.064、`z_err −0.2207±0.0501`；2D known **96.95 m²** |

* 墙钟：RMUL 239.6 s（仿真 192.6 s）；RMUC 第 6 跑 147.6 s（仿真 117.8 s）。RTF **0.80–0.81**（默认模型
  比 §23 的 robot11 轻得多 ⇒ 从 §23 的 0.42 回到 0.8；仍不到 §16 的 1.0，因为本轮全程还挂着
  `ground_segmentation` + `cloud_accumulator` + `slam_toolbox`）。
* 路线设计的**三代演进**（都是实测逼出来的，不是拍脑袋）：`coverage_route` 一把生成 → 直角弯夹死；
  手工"窄通道直行段" → LIO 在走廊里退化；**最终只跑开阔区**才拿到干净数据。

### 24.3 资产清单（旧 vs 本轮候选，旧的一个字节没删）

| world | 槽位 | 旧默认（**提升后仍是它**） | 本轮候选（**未提升**） |
|---|---|---|---|
| RMUL2026 | 2D | `map/RMUL2026.pgm`+`.yaml`，`249x173 @0.05`，`origin [-2.20,-3.15]` | `map/RMUL2026_v6_spl.pgm`+`.yaml`，**`241x162`**，`origin [-1.95,-2.90]` |
| RMUL2026 | 3D | `PCD/RMUL2026.pcd`，53,164 点 / 1.70 MB（`md5 5d0d33eb…`） | `PCD/RMUL2026_v6.pcd`，**27,687 点 / 740 kB**（累积 16.39 M 点，voxel 0.1，`md5 580e6f40…`） |
| RMUL2026 | 会话 | — | `map/RMUL2026_v6.{posegraph,data,meta.yaml}` + `PCD/RMUL2026_v6.meta.yaml` |
| RMUC2026 | 2D | `map/RMUC2026.pgm`+`.yaml`，`577x326 @0.05` | `map/RMUC2026_v6w_spl.pgm`+`.yaml`，`243x301`（**已知 84.19 m²**） |
| RMUC2026 | 3D | `PCD/RMUC2026.pcd`，203,352 点 / 6.51 MB | **没有**（第 6 跑的 `serialize_map` 超时 ⇒ PCD/posegraph 未落盘） |

* RMUL 候选的 sidecar 由 `map_archive.sh save`（= `map_asset_guard.write_manifest`）写出，本轮**额外**补了两个键
  （`pcd_odom_to_map.py` 之后的一次性搬运记录，见 §24.5）：
  `accumulation_frame: odom` + `post_transform_to_map: {rule, used_trace_sample, xyz, q_xyzw, yaw_deg, tool, why}`
  —— `frames.map: map` 仍然为真（最终资产确实在 map 系）。
  `health_warning: false`、`skipped_frames_jump: 0`、`tf_frames_ok 2320 / failed 73`（3.1%，全部是
  "带戳查不到→退化到最新"被**跳过**的帧，不是位姿跳变）。
* 旧默认对（`map/RMUC2026.*`、`PCD/RMUC2026.pcd`、`map/RMUL2026.*`、`PCD/RMUL2026.pcd`）**逐字节未动**；
  旧图仍可随时被 §23.8 的那条回滚命令恢复。

### 24.4 门① 覆盖率（2D）：RMUL **赢**、RMUC **输到 STOP**

口径与 §23 完全相同：`map_cov_stats.py`（同一把尺子、同一 spawn；`missing = 旧图已知(free|occ) 而新图未知/出窗`）。

| 量 | RMUL 旧 | RMUL 新 | RMUC 旧 | RMUC 新（v6w） |
|---|---|---|---|---|
| 栅格 / bbox（m） | `249x173` / 12.45 × 8.65 | `241x162` / 12.05 × 8.10 | `577x326` / **28.85 × 16.30** | `243x301` / 12.15 × 15.05 |
| **已知**（free+occ） | **94.77 m²** | **96.95 m²（102.3%）** | **389.92 m²** | **84.19 m²（21.6%）** |
| 占用格数 | 3,713 | 1,970 | 9,769 | 1,970 |
| 旧图已知被新图覆盖 | — | **99.9%** | — | 21.1% |
| 缺区 `missing` | — | **0.10 m²**（39 格） | — | **307.5 m²** |
| 新图独有 `new_only` | — | +2.27 m² | — | +1.75 m² |
| 占用格"厚度" mean | 0.0821 m | **0.0589 m** | 0.075 m（§23 口径） | — |

* **RMUL：门①过**。已知面积 +2.3%、旧图已知格 99.9% 被覆盖、缺区 0.10 m²（39 格，全在旧图占用格的
  边缘量化误差内）、占用格更薄（墙更"脆"）⇒ **"没有缺区"这条成立**。
* **RMUC：门①严重不过**，而且**触发 STOP RULE**：`84.19 / 389.92 = 0.216 < 0.7`。缺区 307.5 m²
  全部落在**跑不到的那半个场**（新窗口 `x[-8.58,3.57]`，旧窗口 `x[-25.20,3.65]`）⇒ 不是"图没建好"，
  是**车没进去**（§24.10#1–#3）。

### 24.5 鬼影修法：**累积坐标从 `map` 换成 `odom`**，落盘后**一次**刚体搬进 `map`

**机制（§23.9#1 的量化）**：`cloud_accumulator` 是"按**当前**的 `TF(target_frame ← livox_frame)` 把
每帧搬进 target_frame"。`slam_toolbox` 在整跑里不断修正 `map→odom`，而**已经积进去的点不会跟着动**。
本轮把这个量**录下来了**（`odom_rec.py`，2 Hz，全程）：

| 跑 | `map→odom` 的极差（x / y / z / yaw） | 说明 |
|---|---|---|
| `rmul_v6`（干净，n=436，sim 659–876） | **0.331 m / 0.231 m / 0.449 m / 1.81°** | 即使 ATE 只有 0.138 m，`z` 仍抖 **45 cm** |
| `rmuc_v6w`（干净，n=286，sim 19–161） | **0.114 m / 0.081 m / 0.207 m / 1.20°** | 干净跑也有 **21 cm** 的 z 抖 |
| `rmuc_v6` 第 1 跑（发散后） | 30.1 m / 33.4 m / 16.4 m / 180° | 发散后这条 TF 完全无意义 |

⇒ 在 `map` 档累积，同一面墙/地面会被**沿 z 抹开 ≥0.21 m**（干净跑）到 **几十 m**（发散跑）。
这正好解释 §23 的两条实测：`z_span 2.746 m` 的 sidecar 告警、以及"同一面墙多条弧带、散布 0.2–0.4 m"。

**改了什么（两处，都很小，全部入库）**

1. `src/rm_nav_bringup/launch/bringup_sim.launch.py`：新增 launch 参数
   **`cloud_accumulator_target_frame`（默认 `'map'` ⇒ 与改造前逐字节等价）**，校验取值 `map|odom`，
   并把它作为 `target_frame` 传进 `cloud_accumulator` 节点。**只有** mapping 形态的 3D 先验受影响；
   `/map`、`map→odom`、`/segmentation` 的单一发布者契约一个字节没动（累加器本来就"只订阅+查 TF"）。
2. `.tmp_map2/pcd_odom_to_map.py`（**测试目录、不入库**）：跑完用**一次**刚体 `map←odom` 把 PCD 搬进 `map`，
   变换取 `map→odom` **时间序列的末值**；而**路线收尾停在出生点**（`route_close_loop.py` 的合法回程 +
   `--final-spin 1.0`）⇒ 末值 = **出生点处**的修正量（也正是门③观测的那个位置）。
   **为什么"一次刚体"够**：LIO 的 `odom→livox_frame` 是**平滑**的（没有跳变）⇒ 云内部刚性；
   `slam_toolbox` 的修正只影响"整块云放在哪"，一次性搬过去即可。

**证据（"鬼影没有了"）**

| 证据 | 数 | 怎么读 |
|---|---|---|
| 累积期的**帧跳变闸** | RMUL `skipped_frames_jump: 0` / 2320 帧；TF 退化跳过 73（3.1%） | 在 map 档，同一时期 `map→odom` 的 z 抖 45 cm 会把 10 Hz 的帧**逐帧**错位（单帧 4.5 cm 量级，闸的 0.50 m 阈值抓不到）⇒ 累积必然带鬼影；换成 odom 后**没有需要跳过的帧** |
| 地面**厚度**（`floor_pcd.py` 的峰内残差） | RMUL：全局峰内 `resid p50 9.3 mm / p95 18.7 mm`；1.5–3 m 带 `p50 8.4 mm` | map 档累积会把地面抹成 **≥45 cm** 厚的带；实测是 **1 cm 量级的一层** |
| 墙面**形状**残差（`wall_resid.py fullfit`） | RMUL 9 面：`p50 0.068 / max 0.185 / rms 0.090 m`，最优 `dyaw −2.0°` | 散布主项是**整跑偏航漂移**（−2.0°），不是"多弧带"；`spread_p90` 0.13–0.40 与整跑漂移一致 |
| 俯视图（`pcd_xy_plot.py --zmin 0.25`） | `docs/img/RMUL2026_v6_pcd_walls_20261020.png` | 房间四周的墙是**细单线**，没有弧状重影 |

> ⚠️ 诚实边界：本轮**没有**做"同一跑 map 档 vs odom 档"的 A/B（那要再跑一次 RMUL）；上面的证据是
> **机制量化 + 结果形态**，不是同跑对照。列进 §24.11#3。

### 24.6 门② 绝对几何：RMUL 墙面**原始**残差与地面**都不过**；RMUC 无 3D 资产可量

**方法**（`wall_resid.py`，与 §13.4/§16.6/§23.5 同一把尺子：±0.45 m 窗 + 1D 直方图定峰 + 中位精修；
三组数 `raw`/`yawfit`/`fullfit`）：

**RMUL2026（3D PCD，9 个可见面）**

| 量 | 旧 `PCD/RMUL2026.pcd` | 新 `PCD/RMUL2026_v6.pcd` |
|---|---|---|
| 面数 | **2**（点太稀：x=2.5 只有 36 点、y=−8.5 只有 44 点） | **9**（每面 30–1712 点） |
| `raw` \|残差\| p50 / max | **0.0437 / 0.0492 m** | **0.1552 / 0.2438 m** ❌ |
| 其中 `x=2.5`（旧也量得到） | **−0.0381 m** | **−0.1406 m** ❌ |
| 其中 `y=−8.5`（旧也量得到） | **+0.0492 m** | **−0.0211 m** ✅ |
| `yawfit` p50（最优偏航、不加平移） | —（面数 <3） | 0.2024 m（`dyaw −2.0°`） |
| `fullfit` p50 / max / rms（= **形状**误差） | —（面数 <3，拟合不成立） | **0.0681 / 0.1845 / 0.0899 m**（`t_fit (0.134, 0.156)`） |

**判读**：判据写的是"墙面残差 ≤ 旧"，而 RMUL 的**原始**残差 0.155 m 是旧图 0.0437 m 的 **3.5 倍** ⇒
**门②不过**。原因可以拆开：`fullfit` 0.068 m 说明**形状**只有 7 cm 量级的整跑漂移误差，而
`raw` 的 15.5 cm 里约 **0.20 m 是"map 系相对世界的一次性摆放误差"**（最优平移 `(0.134, 0.156)`、
最优偏航 `−2.0°`）。也就是说：**这张图自己内部是自洽的（形状 7 cm），但它被摆在离世界真值 ~20 cm/2° 的地方**。
那个摆放误差来自 `map` 系的定义（= **出生点相对系**）与 LIO 初始化时刻的姿态/位置：
RMUL 的 `map_align_tf` 实测 `roll −0.60° / pitch −0.24° / yaw +0.10°`、`t_wm(odom) = (4.389, 3.330, 0.285)`，
而 `odom` 的原点 = 初始化时刻的 **IMU**（默认模型 imu_link = base_link+(0.12,0,0.125)）而不是 base_link
⇒ **x 方向系统性 +0.12 m**，与 `fullfit` 的 `t_fit x=0.134` 对得上。**这一条是"布局/口径"，不是"建图坏了"**，
但判据只看 raw ⇒ 记不过，列进诚实清单（§24.11#4）。

**地面高度**

| 量 | 旧 RMUL | 新 RMUL |
|---|---|---|
| 全局峰（`floor_pcd.py`） | −0.146 m（点数少 + 台阶，§23.5 判"不可用"） | **+0.1831 m**（n=2404，峰内 `p50 9.3 mm`）—— 这是**抬升平台顶面**（世界 +0.196，`−0.013` 后 = 0.183 ✅ 对得上），不是地面 |
| 距出生点 1.5–3.0 m 带（= 真地面） | — | **−0.0600 m**（`p50 8.4 mm`） |
| 世界系地面 | — | `world←odom z = +0.2854`、`map←odom z = +0.2725` ⇒ `world←map z = +0.0129` ⇒ **−0.047 m** ❌（判据 ±0.006 m） |

⇒ **门②"地面 ≈0（世界）"不过**：−4.7 cm。它的来源是"逐跑的 z 原点/漂移"（`z_err_mean −0.2207 m ± 0.0501`
与 §23.9#4 同一现象）—— 这一轮**把它量清楚了**：`/odom` 的 z 与真值稳定地差 **−0.22 m**（std 仅 5 cm），
是**常数偏置**（不是漂移）；而 `map←odom` 的 z 只吃掉 0.27 ⇒ 剩下的 4.7 cm 就是**`map` 系的 z 原点**
（= 初始化时刻的 IMU/传感器位置）与物理地面之间的差。**没有动 `src/rm_localization/**`**。

### 24.7 门③ 定位质量（GICP，`mode:=nav localization:=gicp`，静止，只订阅）

口径与 §16.7/§23.6 同一件量具：`lio:=fastlio robot:=robot11 robot11_mount:=sensor`、
`nav_rviz:=False spin_speed:=0.0`、`initial_pose=[0,0,0]`、`allow_xy 0.30 m / allow_yaw 6°`，
窗口 = 仿真 settle 20 s + window 20 s。**候选 PCD 先换装到 `PCD/<W>.pcd`（带备份）再跑、跑完换回**
（PCD 路径由 launch 拼死，没有参数可改，同 §23.8 的做法）。

| 臂 | world | PCD | 采纳率 | innov p50 / max（m） | score p50（m²） | `map→odom` 常量偏置 |
|---|---|---|---|---|---|---|
| 旧（§16.7/§23.6 引用） | RMUL2026 | `RMUL2026.pcd`（53,164 点） | 100% | 0.01065 / 0.07429 | 0.0051 | `(−0.057,+0.130,+0.016) m / −0.16°` |
| **新** | RMUL2026 | `RMUL2026_v6.pcd`（27,687 点，`md5 580e6f40…`） | **420/420 = 100%**（拒绝 0、越界 0、收敛 420/420） | **0.00339 / 0.00874 / 0.01795**（门限 `allow_xy 0.30 m`）/ yaw p50 0.051° | **0.00322**（p95 0.00334 / max 0.00338） | **`(−0.151,+0.180,+0.260) m / +0.2°`**（`map_odom_probe` 未跑完，取 `gicp` 状态行稳定值） |

**判读**：**门③过**。三件事一起看：① 采纳 **420/420**、拒绝 0、收敛 420/420；② 创新 p50 **3.4 mm** / p95 8.7 mm / max 18 mm（门限 30 cm；旧图同口径是 10.7/25.1/74.3 mm ⇒ **好 3 倍**），`score` p50 **0.00322 m²**（旧 0.0051 m²，好 1.6 倍），
比 §23 那次 1.4–1.9 m 的 `map→odom` 灾难好两个量级）；③ **`map→odom` 的 xy 常量偏置 0.235 m**
（旧 0.142 m、§23 失败候选 1.4–1.9 m）。z 分量 +0.260 m 是**坐标口径**（本资产的地面在 map z=−0.06、
而 `odom` 的 z 原点在 IMU 初始化位置），与 §16.7 的 `+0.403` 是同一类东西，**不是**定位坏了。
RMUC 的 3D 资产这一轮**没有落盘**（§24.3）⇒ 门③的 RMUC 臂**没有本轮数据**。

### 24.8 门④ 自洽：`map_asset_guard check` + sidecar

* **RMUL 候选**：`check --kind posegraph --name RMUL2026_v6 --world RMUL2026 --resolution 0.05` ⇒
  **`resume`（`may_load=True`，退出码 0）**；`check --kind pcd --name RMUL2026_v6 --world RMUL2026` ⇒
  **`resume`**。存档 sidecar 的 `world=RMUL2026` / `spawn_pose=(4.3,3.35,0.2,0)` / `map_start_pose=[0,0,0]` /
  `resolution=0.05` 与本次跑一致；PCD sidecar `health_warning: **false**`（z 跨度 0.98 m）。
* **RMUC 候选**：`map/RMUC2026_v6w` 的 posegraph/PCD **都不存在**（`serialize_map` 超时，§24.2）⇒
  `check` 判 `fresh`。⇒ **门④对 RMUC 是"没有资产可查"**，不是"查了不过"。
* 默认资产（`map/<W>.pgm|yaml`、`PCD/<W>.pcd`）仍然**没有** sidecar（它们是"先验资产"不是"会话存档"，
  仓库惯例见 `docs/map_assets.md` §1）⇒ 本轮没有改变这个形态。

### 24.9 提升决定：**两个世界都不提升**（旧默认一个字节没覆盖）

| 门 | RMUL2026 | RMUC2026 | 结论 |
|---|---|---|---|
| ① 覆盖率（known ≥ 旧 + 无缺区） | **96.95 ≥ 94.77（102.3%）**、缺区 0.10 m²、旧已知格覆盖 99.9% | **84.19 / 389.92 = 21.6%**（缺区 307.5 m²） | RMUL ✅ / RMUC ❌（**0.216 < 0.7 ⇒ STOP RULE**） |
| ② 绝对几何（墙残差 ≤ 旧 + 地面 ≈0±6 mm） | 墙 raw p50 **0.155** vs 旧 **0.0437** ❌；地面 **−0.047 m** ❌ | 无 3D 资产可量 | ❌ |
| ③ 定位质量（`map→odom` 小 + 创新/score） | 采纳 **100%**、创新 **0.005–0.018 m**、`map→odom xy 0.235 m / 0.2°` ✅ | 无资产 | ✅（RMUL） |
| ④ 自洽（guard + sidecar） | guard `resume` ✅、`health_warning: false` ✅ | 无资产 | ✅（RMUL） |

⇒ **RMUL 差在门②**（墙面**原始**残差与地面），**RMUC 差在门①**（覆盖率，且触发 STOP）。
两个世界都**保留旧默认**（`map/<W>.pgm|yaml`、`PCD/<W>.pcd` 逐字节未动）。

**备份（本轮只产生"换装门禁跑"的备份，没有"提升备份"）**：

```bash
src/rm_nav_bringup/PCD/RMUL2026.pcd.bak-20261020b   # md5 5d0d33eb5219d9416606a7b0f5e34028（= 旧默认，逐字节）
```

> ⚠️ 门③ 要求"GICP 读的是**新** PCD"，而 `PCD/<world>.pcd` 的路径是 launch 拼死的 ⇒ 只能把候选
> `cp -f` 就位再换回来。**本轮已换回**：`PCD/RMUL2026.pcd` 的 md5 现在 **= `5d0d33eb…`**（见 §24.12 的复算）。
> 回滚（万一状态不对，一条命令）：
> ```bash
> cd src/rm_nav_bringup && cp -f PCD/RMUL2026.pcd.bak-20261020b PCD/RMUL2026.pcd
> ```

**建议（下一步，不在本轮范围）**

1. **RMUC 别再用"一条路线穿全场"**：这条走廊对 FAST-LIO 是**退化区**（§24.10#1）。要做全场先验只有三条路：
   （a）**多会话拼图**（东/西半场各一次，靠 `map_archive` 的位姿图拼）；（b）**换 LIO** 在走廊里不退化
   （`pointlio` 或给 FAST-LIO 加走廊退化处理）；（c）**接受旧图**。
2. **RMUL 若要提升**，先解决"`map` 系的一次性摆放误差"：把 `map` 的原点定在 **base_link 的出生位姿**
   （而不是 LIO 初始化时刻的 IMU 位姿），并让 `wall_resid` 的判据改用 `fullfit`（形状）+ 一次
   `map↔世界` 标定，而不是 `raw`。
3. **`serialize_map` 会超时**（本轮 120 s 内没回来）：建图跑把 `map_archive.sh save` 的超时调大、
   或在跑图结束后**先停车等 20 s** 再 serialize（本轮 RMUL 成功、RMUC 失败，差别只在这类时序）。

### 24.10 本轮的硬发现（三条，都不是"配方写错"）

**① FAST-LIO 在 RMUC 的窄走廊里退化到发散（这是 RMUC 覆盖率上不去的**唯一**原因）**

| 现象 | 数 / 证据 |
|---|---|
| `[fastlio_mapping] No Effective Points!` | 第 3 跑 **1799 次**（从 sim ≈140.7 起连续刷）；第 2/4/5 跑同一段也出现 |
| `/odom` 发散 | 第 3 跑 → (−100.96, −59.13, −136.86)；第 4 跑 → (−37.9, −105.1, −49.0)；第 5 跑 → (−104.4, +121.3, −54.2) |
| 首发出现在哪 | 全部在**进/出那条 0.42–0.50 m 走廊**之后（map 系 `y≈6.87`、`x∈[−1,2]`）或**东侧窄条**（`x≈3.26, y≈−5.4`） |
| 走廊的真实净宽（STL 真值） | `channel_probe.py`：最宽一条通路的最小间距 **0.245–0.250 m** ⇒ 净宽 **0.49–0.50 m**；地面高度剖面 = **0.000 m**（不是抬升板），上下界墙在 `y=6.63 / 7.05` |
| 车体 | 默认模型碰撞 `box 0.2 × 0.3 × 0.1` + 4 轮 ⇒ **宽 0.30 m**（外接半径 0.1803 m）⇒ 走廊里**每边只有 6–10 cm** |
| 为什么 §23 的结论要改 | §23 说"robot11 过不去（0.60 m 外接）"是对的；但**换成默认模型也过不去**——**不是尺寸，是 LIO**：车进得去，FAST-LIO 在里面**没有有效点** |

⇒ 这条**推翻了"换回默认模型就能拿到全场"这个前提**。RMUC 的可行驶域（STL 真值 224 m²）**就是**由
0.4–0.5 m 宽的走廊串起来的一圈 + 中间几块开阔地；`clearance 0.24` 下两半场"连通"，但那条通路对
FAST-LIO 是**退化区**。**旧图当年是怎么拿到 389.9 m² 的，仍然没有日志可查**（§24.11#6）。

**② 车会**爬上** 0.2–0.3 m 的台阶并卡住/侧翻 —— 真值图的"阶闸"必须比 0.15 m 更紧**

第 1 跑：`/odom_ground_truth` z=**0.3697**、四元数 `y=0.566 / z=0.823`（车体歪/翻），
轨迹 79.1 m 后 20 s 零位移。而同一时刻真值栅格（`step 0.15`）把那里标成"可行驶"。
原因：`build_truth_map.py` 的 `--step 0.15` 只保证"**离**台阶 ≥clearance"，而**跟踪误差**（纯追踪、
lookahead 0.55）在 0.45 m 宽的通道/窄条里可以到 10 cm 量级 ⇒ 车头会**啃上**台阶边缘，
Gazebo 的 box+轮子就爬上去。**修法（本轮已用）**：真值图 `--step **0.08**`（0.08–0.15 m 的"危险边缘"
也标成障碍）+ 速度降到 0.35–0.40 m/s；但**没有根治**（第 5 跑仍在东侧窄条卡了 10 s）。

**③ 2D 图会"跑飞"成 1200×1800 m —— LIO 发散后 `slam_toolbox` 的栅格会长到 24017×36080**

第 5 跑：`/map` 最终 **24017×36080** 格（866 MB 的 pgm），窗口 ≈1200×1800 m。
原因：`/odom` 发散到 1e2 m 后，slam_toolbox 的 scan matching 把节点摆到场地外，栅格跟着长。
**运行侧后果**：`map_saver_cli` 会写出一个 866 MB 的文件（本轮**已删**，不进库）。
**守卫建议（未实现）**：建图跑应该在 `map` 的宽高 > 某阈值（如 2000×2000）时**直接放弃这张图**，
而不是照写。

### 24.11 本轮没能验证 / 诚实清单

1. **RMUC 没有拿到"一条走完全场的干净跑"**：六次尝试，最好的一次只覆盖东半场开阔区（84.19 m²）。
   ⇒ 门②③④ 的 RMUC 臂**没有本轮数据**（§24.3 的 PCD 未落盘）。
2. **RMUC 第 6 跑的 PCD/posegraph 丢了**：`serialize_map` 在 120 s 内没回来（rc 124）。2D 图（84.19 m²）
   与 `drive.json`、`map→odom` trace 还在；3D 云只活在那次进程里 ⇒ **无法事后补落盘**。
3. **没有做"同跑 map 档 vs odom 档"的 A/B**：鬼影修法的证据是"机制量化（`map→odom` 的 z 抖 0.21–0.45 m）
   + 结果形态（地面厚度 1 cm、墙是细单线）"，**不是**同一次跑的两档对照。要做：拿 RMUL 的同一路线
   再跑一次 `cloud_accumulator_target_frame:=map`，比 `spread_p90` 与地面厚度。
4. **RMUL 的 `raw` 墙面残差 0.155 m 里有多少是"口径"有多少是"几何"**：`fullfit` 0.068 m + 最优平移
   `(0.134,0.156)` + 最优偏航 `−2.0°` 说明主要是**一次性摆放**；但**没有独立证据**证明"摆放误差可以
   用一次标定消掉"（本轮没做 `map↔世界` 的单独标定）。
5. **GICP 只测了静止、一个位姿（出生点）、一跑**（RMUL），且是 `robot11+sensor` 臂（为了与旧基线同一件量具）；
   **默认模型臂没跑**（时间不够）。
6. **"旧图怎么覆盖全场"仍然没有答案**：没有 2026-10-07 那次会话的位姿/遥操日志。§23 的推断
   （"人工硬挤过缝"）在本轮**换模型后仍然不成立**（默认模型也过不去，因为 LIO 退化）⇒
   更可能的解释是**多会话/多张图拼出来的**，但**没有证据**。
7. **RMUC 第 1 跑的"侧翻"**只看了 GT 的四元数与 z，**没有**去查 Gazebo 的接触/碰撞日志
   （`gzserver` 只有 ODE 无告警）。
8. **`health_warning` 的阈值没复核**（§23.10#11 的同一项）；本轮 RMUL 是 `false`，RMUC 没有 PCD。
9. **没有做多会话拼图**（明确不做，见 §24.9 建议 1）。
10. **`robot:=hzmirm` / `lio:=pointlio` 两条候选**没试（任务口径只要求 `fastlio`）。

### 24.12 一句话给用户

> **这一轮把"鬼影"从机制上修掉了**：`cloud_accumulator` 的累积系从 `map` 换成 `odom`（**新增一个
> 默认值不变的 launch 参数**），落盘后用**一次** `map←odom` 搬进 `map`（路线收尾停在出生点、取 trace 末值）。
> 证据：`map→odom` 在干净跑里**仍然抖 0.21–0.45 m**（z 为主），而新云的地面只剩 **1 cm 厚的一层**、
> 墙是细单线。**RMUL 因此第一次在覆盖率上赢过旧图**（已知 96.95 m² vs 94.77 m²、旧图已知格覆盖 **99.9%**、
> 缺区 0.10 m²）、定位门也过（采纳 100%、创新 0.005–0.018 m、`map→odom xy 0.235 m / 0.2°`）、
> 自洽门也过（guard `resume`、`health_warning: false`）。**但门②不过**：墙面**原始**残差 0.155 m（旧 0.0437，
> 差的是 **~20 cm / 2° 的一次性摆放**，形状误差其实只有 6.8 cm）、地面在世界系 **−0.047 m**（判据 ±6 mm）。
> **RMUC 则卡在覆盖率 21.6%**（84.19 / 389.92 m²，**因子 0.216 < 0.7 ⇒ STOP**）：六次尝试里四次死于
> **FAST-LIO 在 0.42–0.50 m 宽的走廊里连续 `No Effective Points` ⇒ `/odom` 发散**（而那条走廊是
> RMUC 两半场之间**唯一**的连通处、车体 0.30 m 宽也进得去）。⇒ **两个世界都保留旧默认、一个字节没覆盖**
> （只留了 `PCD/RMUL2026.pcd.bak-20261020b` 这份"换装门禁跑"的备份，且**已换回**）；
> 机器 swap 现在 **3.9/4.0 GB 满**（§23 的 OOM 遗留 + 本轮一次两栈重叠），内存 20 GB 可用。
> **下一步**：RMUC 只能"多会话拼图"或"换 LIO"；RMUL 若要提升，先把 `map` 的原点定到 base_link 的
> 出生位姿（现在定在 IMU 初始化位置，系统性 +0.12 m），并把墙面判据改成 `fullfit`。

## §25 第十六轮（**本机 date=2026-10-20T05:37–06:06**）：**当前 master 上的 nav 基线重测** —— `hybrid` 已是默认、Livox 双档/世界地面/gicp 首帧门/自转/`band_guard` 全部落地后的 §10 风格 3+3 A/B，外加**全局 `−1.0` 覆盖复检**、`slam_nav`/`global_obstacle:=scan`/RMUC2026 冒烟

> **本节只新增**（append-only）：§10–§24 一个字都没改（只在本节引用它们）。§24 是最新 ⇒ 本节取 **§25**。
> 本轮 **没有改** `src/**`、**没有改**地图/PCD 资产、**没有改**任何 launch/YAML/二进制：11 跑用的都是
> **当前工作树的既有构建**（`build/` → `install/` 是符号链接，所以 md5 就是实际加载的那份）。
> 新增的只有 `.tmp_ab/exp25/`（gitignored：`run_one.sh` / `batch.sh` / `agg.py`）与 `.tmp_ab/e25_*` 原始数据。
>
> **为什么必须重测**：§10 的 6 跑矩阵（launch `0161033b…`、`libsmall_point_lio.so` `58bebfb8…`）是在
> **Livox 插件 z/y 偏置还没修**、**世界地面还是 +0.0497**、**gicp 首帧门还死锁**、**两档底盘都转不动**、
> **`band_guard` 还不存在**的时候测的。此后 master 上落了 7 个会改变 nav 观测量的事件：
> `6473a29`（`gravity_align_mode` 默认 `auto` ⇒ `lio:=small_point_lio` 默认 hybrid）、
> `72c7c07`+`7a544bf`（Livox 插件 **sensor 与 parent 两档**的坐标系修复）、
> `ec5028f`（RMUL2026 地面校准到 0）、`9bb8f2f`（gicp 首帧门）、`457701c`（`global_obstacle:=scan`
> 的 −1.0 覆盖层）、`aa77420`+`ce97ca1`（两档底盘真能自转）、`65a0a85`（`band_guard`）。
> ⇒ §10 的绝对值**整表作废**（§25.10 逐条列），必须按同一协议在新 master 上重测一遍。

### 25.1 一句话结论

> **`hybrid` 作为默认档，在当前 master 上是干净的**：`/scan` **771/771 波束全部在 nav2 的 `[0,2]` 带内**
> （`off` 只有 **337/771 = 43.7%**，且 `kept-y` 单侧 `[0.257, 3.281] m` —— 半平面盲区；
> `band_guard` 如实给 `WARN 43.4%`），残余地面倾角 **0.464° [0.415–0.586]**（`off` **30.305°**），
> `odom→base_link` rpy **[+0.06, −0.47, −0.63]°**（`off` roll **30.20°**），
> `h_meas` **0.2617 m / INFO**（`|Δ|≤2.2 mm`；§10 的 0.1894/ERROR 是插件 bug 的像）。
> 局部代价图两档都**非空**（lethal **329 [264–373.5]** vs **299.5 [297.5–307]**、最近 lethal
> **0.712 / 0.751 m**、前方 2×1.2 m **22 / 20** 格）—— **注意：修复后 `lethal` 条数不再是判据**
> （`off` 的 43% 条带照样打在墙上），**判据是 `band_guard` 的 in-band/kept-y**。
> **全局半**：默认 `global_obstacle:=stvl` 的全局图两档几乎逐值相同（lethal **3727 [3725–3729]** vs
> **3725 [3724–3725]**，其中 static+inflation 基线 = **3713**，STVL 只加 **+12…+16** 格）；
> **`global_obstacle:=scan` 的 `min_obstacle_height:−1.0` 覆盖层仍然必须保留** ——
> 去掉它以后 map 帧扫描面 z 仍是 **−0.179 [−0.248,−0.123]**（全部 < 0.0）⇒ **0/772 在带内**、
> 全局图退回 static-only 的 **3713/16886/10176**；保留则 **774/774 在带内**、全局 lethal **3872–3887**。
> **`slam_nav`** 同样 100% 在带内、`map→odom` 由 `/slam_toolbox` 唯一发布且健康；**RMUC2026** 的 in-band 也
> 100%（555/555），它的 **local lethal = 0 不是病**，而是**窗口几何**：最近墙 **2.65 m（nav2）/2.693 m（离线量图）**
> ＞ 局部滚动窗半宽 **2.50 m**。

### 25.2 协议（与 §10.4 / §12.1 逐字相同，只多了 `band_guard:=True` 与本任务的 controller 槽）

`settle = 20 s（仿真）+ window = 20 s（仿真）`；`gui:=False nav_rviz:=False lio_rviz:=False`；
`world:=RMUL2026`（第 4 项换 `RMUC2026`）；`robot:=robot11 robot11_mount:=sensor`；
`lio:=small_point_lio`；`spin_speed:=0.0`；**`gravity_align_mode` 不传 ⇒ `auto` ⇒ hybrid**（第 1 项的对照臂显式 `:=off`）；
`mode:=nav localization:=gicp nav:=mppi planner:=smac2d`；`band_guard:=True`。逐字命令：

```bash
# 第 1 项：§10 风格 A/B，{默认(auto⇒hybrid), off} × 3，**交错**（hyb/off/hyb/off/hyb/off）
.tmp_ab/exp25/run_one.sh e25_hyb_r1 world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
    nav:=mppi planner:=smac2d robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
.tmp_ab/exp25/run_one.sh e25_off_r1 world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
    nav:=mppi planner:=smac2d robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True \
    gravity_align_mode:=off
# 第 2 项：全局 −1.0 覆盖 复检（with = 普通 launch；without = 只把两个键改回 0.0 的 scratch wrapper）
.tmp_ab/exp25/run_one.sh e25_gscan_ov_r1   ... global_obstacle:=scan ...          # 带覆盖（launch 默认行为）
LAUNCH_FILE=$PWD/.tmp_band_guard/launch/bringup_sim_noband.py \
  .tmp_ab/exp25/run_one.sh e25_gscan_noov_r1 ... global_obstacle:=scan ...        # 覆盖被 monkeypatch 回 0.0
# 第 3 项：§12 风格
.tmp_ab/exp25/run_one.sh e25_snav_hyb_r1 world:=RMUL2026 mode:=slam_nav mapper:=slam_toolbox \
    lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
.tmp_ab/exp25/run_one.sh e25_gscan_hyb_r2 ... global_obstacle:=scan ...          # = 第 2 项的复本（n=2）
# 第 4 项：第二个世界
.tmp_ab/exp25/run_one.sh e25_rmuc_hyb_r1 world:=RMUC2026 mode:=nav ... （其余同上）
```

* **隔离**（与 §12/§15/§17 逐字同口径）：私有 `HOME=.tmp_ab/home-<tag>`、按 tag 散列 `ROS_DOMAIN_ID=100…159`、
  **先探空端口再占** `GAZEBO_MASTER_URI`（11700–11899）、`unset DISPLAY`、`setsid` 起栈（进程组 = 本 run）、
  硬超时（单跑 `timeout 900`，探针各自 600 s）、收尾**只用** `tools/scripts/lib/scoped_cleanup.sh`
  按**进程组 / master URI 归属**清（**无全机 pkill**）。每个 `run_one.sh` 启动前若发现**本命名空间内有
  `gzserver`** 就 `abort 90`（`batch.sh` 见 rc≠0 会等 20 s 重试一次）；本轮 11 跑 **0 次 `RETRY`**，
  每跑收尾都是 `残留: (空) / gz=0 / 全局 gzserver=0`（11/11）。
* **不许两栈并行**：11 跑由**一个** `batch.sh` **顺序**执行（05:39:53 → 06:06:25），跑间 8 s；
  第 1 跑（`e25_hyb_r1`，05:37:06–05:39:4x）是批次**之前**用同一脚本/同一参数的单跑冒烟
  （批次第一跑 05:39:53 起跑前，`run_one.sh` 自检到 `gzserver` 数 = 0 ⇒ 两者不重叠）。
* **探针**（都只订阅、不发布、不改参数）：`tools/scripts/tiltmount/nav_band_probe.py`（`/scan` 在 odom 的 z、
  两张 costmap 的原始代价计数、车那格、最近 lethal、前方 2.0×1.2 m）、
  `.tmp_gravtest/grav_probe.py`（残余地面倾角、`odom→base_link` rpy、RTF）、
  `tools/scripts/tiltmount/uc_procstats.py`（进程账），加本轮**唯一的新证据源** = `band_guard` 的
  逐 5 s 评估行（`in-band / below / above / kept-y / z / band / tf 来源`，每 costmap 一行）。
  运行期参数回读与 `tf2_echo` **放在探针结束之后**（避免 §15.11#1 那条"回读扰动栈"）。
* **口径**：`/scan` 的 `frame_id=base_link`；"in-band" = 波束端点 z ∈ `[0.0, 2.0]`
  （运行期回读：local 与 global 的 `obstacle_layer.scan.min/max` 都是 **0.0/2.0**；`global_obstacle:=scan`
  档的 global 是 **−1.0/2.0**）；"kept-y" = 带内波束端点在该 costmap 帧里的 y 区间；
  "最近 lethal / ahead" 与 §12 同定义；全局 `n_cells` 是**该跑自己的** global costmap 大小
  （磁盘图档 43077 格；`slam_nav` 的实时图 13950 格；RMUC 的磁盘图 188102 格）⇒ **跨档不可比绝对值**。
* **窗口样本数**：每跑 `/scan` **200 帧**、local costmap **100 帧**、global costmap **33 帧**、
  `band_guard` **11 个 5 s 样本**（`slam_nav`/`global:=scan` 档 22 行 = 两张 costmap 各 11）。
  RTF **0.400–0.418** ⇒ 20 仿真秒 ≈ 48–50 墙钟秒。

### 25.3 冻结证据（launch_md5 / 二进制 md5 / HEAD，**11 跑逐跑相同、跑前=跑后**）

```text
git_head = 779ceae488c2507a4e4df681f354174aa90de914（= origin/master；merge-base --is-ancestor 成立 ⇒ 未 pull）
launch_md5(src/rm_nav_bringup/launch/bringup_sim.launch.py) = 7efa435b6f597ca00a5878f02b6976df
  ↑ 与 §10 的 0161033b…、§12/§15 的 6d38e74e…/3ac66e6d… 都不同（这中间落了 auto/守卫/自转等提交）
build/small_point_lio/libsmall_point_lio.so      = f1f00edb6415f1ecf3e23d456cfedf08   （≠ §10 的 58bebfb8…）
build/small_point_lio/small_point_lio_node       = d940e573244013195b6703252d6589e2
build/gicp_registration/libgicp_registration.so  = 53c7067dd0e81bcd4a7f3942c6289233   （= §14 修好门之后那个）
build/ros2_livox_simulation/libros2_livox.so     = 2281ab554772170923ca4ec97f7e4a1e   （含 72c7c07 + 7a544bf 两档修复）
build/lio_tf_adapter/lio_tf_adapter_node         = f6c2077bf4b68f775be25265fd938af7
src/rm_nav_bringup/scripts/band_guard.py         = 098fc9f91e6f7e6ee0744e7af8db28ac
config/nav2_params_sim_global_scan_obstacle.yaml = e81d000282ade825778e03af2af702e7   （−1.0 覆盖层）
rm_navigation/params/nav2_params_sim_base.yaml   = e9870542287df02e68f50729503e8cc9
world/RMUL2026_world.world                       = edae5ee13ab6dad56f26a2c56f406cbc   （= ec5028f 校准后）
map/RMUL2026.yaml = fd23dd82c0aa63b6c2fdf3644c7c9253    map/RMUC2026.yaml = 683b07a83120957aad1cbbd69d76bf43
```

* **每跑 `meta_before.txt` / `meta_after.txt` 的 12 行 `md5` 逐字相同**（`meta_md5_diff.txt` = **0 行**，
  11/11 跑）；`git status` 里 `src/**` 的条目在本轮**前后一致**（那几个 `M`/`??` 是本节之前就在的：
  3 个 submodule 指针、`map/RMUC2026_spl.{pgm,yaml}`（**未提升的候选**，**不是默认图**；默认图是
  `map/<world>.yaml`）、以及一批 `PCD/*.bak-*`）。
* **用的是当前默认资产**：nav2 的 2D 先验图由 launch 的 `map_yaml` 默认值决定 = `map/<world>.yaml`
  ⇒ RMUL 用 `map/RMUL2026.yaml`（md5 `fd23dd82…`）、RMUC 用 `map/RMUC2026.yaml`（md5 `683b07a8…`），
  两者的 `git status` 都**干净**（§23/§24 的重建尝试**没有提升**，与任务书一致）。
* **默认档不是"传了 hybrid"**：日志逐字有收据
  `[gravity_align] gravity_align_mode=auto ⇒ 有效值=hybrid（lio=small_point_lio）`，
  运行期回读 `/small_point_lio gravity_align_mode=hybrid`、`z_origin=ground`、`h=0.25952717`、
  `publish_tf=False`、`odom_convention=sensor`、`/lio_tf_adapter rpy=[0.523598776,0,0]`（弧度）。
  `off` 档回读 `gravity_align_mode=off`、`z_origin=sensor`、`h=0.0`、`publish_tf=True`、
  `odom_convention=body`，且 `/lio_tf_adapter` **不存在**（= 旧路径，LIO 自己发 odom→base_link）。

### 25.4 主表：§10 风格 A/B（3 跑/档，**median[min–max]**，`settle 20 / window 20`）

| 指标 | **默认（`auto`⇒`hybrid`）** n=3 | **显式 `gravity_align_mode:=off`** n=3 | §10.4（**插件修前**，n=3）off / hybrid |
|---|---|---|---|
| `/scan` 有限波束/帧 | 771 [771–771] | 771 [771–771] | 677 [677–678] / 678 |
| 波束 z(odom) **p50** | **+0.026** [+0.025, +0.037] | **−0.089** [−0.091, −0.089] | +0.243 / +0.058 |
| 波束 z(odom) **min–max** | +0.011 … +0.077 | −0.424 … +2.460 | −0.244…+2.595 / +0.045…+0.099 |
| 在带 `[0,2]` **内** | **771 [771–771]（100%）** | **337 [336–337]（43.7%）** | 579（85.4%）/ 678（100%） |
| 带下 / 带上 | **0 / 0** | **433 [432–433] / 3 [3–3]** | 60/36 / 0/0 |
| local **lethal** | 329 [264–373.5] | 299.5 [297.5–307] | 193 / 288.5 |
| local **inscribed** | 10290 [10265–10353.5] | 10022.5 [9639.5–10353] | 8280.5 / 8452 |
| local **free** | 40106.5 [40042.5–40154.5] | 42174.5 [42033.5–42832] | 44546 / 43312.5 |
| **最近 lethal** | **0.712** [0.709–0.717] | **0.751** [0.750–0.752] | 0.760 / 0.769 |
| **前方 2.0×1.2 m lethal** | 22 [21–24] | 20 [19–20] | 21 / 18 |
| 车 1 m 内 lethal | 28 [27–30] | 26 [25–27] | 26 / 23.5 |
| **`odom→base_link` rpy（roll）** | **+0.060** [−0.050, +0.107]° | **30.198** [30.185, 30.397]° | 30.06…30.20 / 0.30…0.43 |
| ⤷ pitch | −0.472 [−0.551, −0.252]° | +0.493 [+0.407, +0.505]° | −0.12…−0.18 / −0.21…−0.38 |
| ⤷ yaw | −0.633 [−0.728, −0.370]° | −1.073 [−1.113, −1.059]° | −0.07…−0.21 / −0.04…+0.05 |
| **地面残余倾角** | **0.464** [0.415–0.586]° | **30.305** [30.278–30.441]° | 30.115 / 0.518 |
| **`h_meas`** | **0.2617** [0.2616–0.2617] m / **INFO**（\|Δ\|≤2.2 mm） | **—**（`off` 无 `h_check`） | 0.1894 / **ERROR**（插件 bug） |
| `base_link` z(odom)（窗口） | +0.0258 [+0.0253, +0.0370] | −0.1321 [−0.1336, −0.1317] | −0.081 / +0.057 |
| **`band_guard` local（in-band）** | **OK 100.0%**（11/11 样本，771/771…774/774） | **WARN 43.4%** [43.1–43.4]（335/771…337/773） | 无守卫（§13.8 手动量到 43.1%） |
| **`band_guard` local（kept-y）** | **[−0.428, +5.270]** m（双侧、跨原点） | **[+0.257, +3.281]** m（**单侧 +y**） | — |
| **global lethal**（默认 `stvl`） | **3727** [3725–3729] | **3725** [3724–3725] | 3713 / 3713（= static-only，见 §25.6） |
| global inscribed / free | 16872 [16870–16875] / 10176 [10175–10176] | 16875 [16875] / 10175 [10175] | 16886 / 10176（同左） |
| global 最近 lethal / ahead | 0.450 [0.446–0.451] / 81 [81–82] | 0.470 [0.465–0.471] / 82 [82–82] | —（§10 未记；§12 的 0.786·15 / 0.746·18 不可直比） |
| global cells | 43077 | 43077 | —（§10 未记格数） |
| RTF | 0.406 [0.400–0.415] | 0.417 [0.416–0.418] | —（§10 未记） |

**读法（三条，别把表读反）**

1. **`hybrid` 的 100% 是"超集且不丢"**：`off` 的 337 条是**另一批**波束（波束 z p50 −0.089、带下 433 条），
   `kept-y` 只剩 `+y` 一侧 3.0 m 的**条带**（`band_guard` 的 `hint=kept-y is ONE-SIDED (+y)` 命中）；
   `hybrid` 是**整盘 5.7 m 跨原点**。这与 §13.8 / §17 的 333/776 + `[0.275, 3.460]` 是同一现象、
   同一量级（差在 `/scan` 有限波束档位 771 vs 776）。
2. **`lethal` 条数不再是判据**：`off` 现在 **299.5** 而不是 §9.7 的 **0** —— 因为修好插件/世界后
   `off` 的扫描面落在 −0.089，掉下去的是 433 条**低处**波束，留下的一条 3 m 条带**照样打在墙上**。
   ⇒ 只看 lethal 会得出"两档差不多"的错结论；**必须看 in-band / kept-y**（这也正是 `band_guard`
   存在的理由，§17）。
3. **`off` 的 `map→odom` 把 30° 倾角整笔吸收**（roll **−29.8°**），`hybrid` 的 `map→odom` 近水平
   （roll **+0.5…+1.1°**）—— 与 §15.9#2 完全一致（这也解释了为什么两档的**全局**图都"看起来正常"）。

### 25.5 逐跑明细（11 跑；`finite`=`/scan` 有限波束、`incl`=残余地面倾角、`h_meas`=LIO 互检）

| run | 配置 | finite | z_p50(odom) | z[min,max] | in-band | 下/上 | local lethal/insc/free | 近/前 | global l/i/f（格数） | 近/前 | incl | h_meas | rpy roll/pitch/yaw |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `e25_hyb_r1` | hybrid | 771 | +0.037 | [+0.021,+0.077] | **771** | 0/0 | 264 / 10290 / 40154.5 | 0.717 / 22 | 3725 / 16875 / 10175（43077） | 0.451 / 82 | 0.415° | 0.2617 | −0.05 / −0.25 / −0.73 |
| `e25_hyb_r2` | hybrid | 771 | +0.026 | [+0.011,+0.073] | **771** | 0/0 | 373.5 / 10265 / 40106.5 | 0.709 / 24 | 3727 / 16872 / 10176 | 0.450 / 81 | 0.464° | 0.2616 | +0.06 / −0.47 / −0.63 |
| `e25_hyb_r3` | hybrid | 771 | +0.025 | [+0.007,+0.083] | **771** | 0/0 | 329 / 10353.5 / 40042.5 | 0.712 / 21 | 3729 / 16870 / 10176 | 0.446 / 81 | 0.586° | 0.2617 | +0.11 / −0.55 / −0.37 |
| `e25_off_r1` | off | 771 | −0.091 | [−0.421,+2.472] | 336 | 433/3 | 307 / 9639.5 / 42832 | 0.750 / 20 | 3725 / 16875 / 10175 | 0.465 / 82 | 30.441° | — | 30.40 / +0.41 / −1.11 |
| `e25_off_r2` | off | 771 | −0.089 | [−0.424,+2.460] | 337 | 432/3 | 297.5 / 10022.5 / 42174.5 | 0.751 / 19 | 3724 / 16875 / 10175 | 0.470 / 82 | 30.278° | — | 30.18 / +0.49 / −1.06 |
| `e25_off_r3` | off | 771 | −0.089 | [−0.426,+2.459] | 337 | 433/3 | 299.5 / 10353 / 42033.5 | 0.752 / 20 | 3725 / 16875 / 10175 | 0.471 / 82 | 30.305° | — | 30.20 / +0.50 / −1.07 |
| `e25_gscan_ov_r1` | hybrid + `global:=scan`（**带 −1.0**） | 771 | +0.038 | [+0.018,+0.082] | **771** | 0/0 | 296 / 10266 / 40165.5 | 0.716 / 22 | **3872 / 17267 / 9600** | 0.451 / 93 | 0.415° | 0.2616 | −0.08 / −0.29 / −0.95 |
| `e25_gscan_hyb_r2` | 同上（复本） | 771 | +0.034 | [+0.018,+0.072] | **771** | 0/0 | 312.5 / 10210 / 40247 | 0.716 / 23 | **3887 / 17224 / 9608** | 0.448 / 95 | 0.432° | 0.2617 | −0.06 / −0.22 / −0.35 |
| `e25_gscan_noov_r1` | hybrid + `global:=scan`（**去 −1.0**） | 771 | +0.025 | [+0.009,+0.077] | **771** | 0/0 | 295 / 10864.5 / 39744 | 0.717 / 22 | **3713 / 16886 / 10176** | 0.450 / 81 | 0.535° | 0.2616 | +0.08 / −0.47 / −0.71 |
| `e25_snav_hyb_r1` | **`mode:=slam_nav`** | 771 | +0.036 | [+0.022,+0.073] | **771** | 0/0 | 273 / 10265.5 / 40172 | 0.700 / 26 | 228 / 2008 / 678（13950，实时图） | 0.724 / 30 | 0.399° | 0.2616 | −0.01 / −0.23 / −0.83 |
| `e25_rmuc_hyb_r1` | hybrid + **RMUC2026** | 555 | +0.026 | [+0.004,+0.034] | **555** | 0/0 | **0 / 0 / 62500**（见 §25.8） | 3.147* / 0 | 9777 / 47968 / 89790（188102） | 2.651 / 0 | 0.153° | 0.2613 | −0.19 / +0.03 / −1.46 |

（`*` = 100 帧里**只有 1 帧**有 1 个 lethal 格，其它 99 帧为 0 ⇒ 中位数 0、`nearest` 只有 1 个样本。
`rpy` 是窗口末端的单次 `tf2_echo`（`odom←base_link`），与 §10.4 的口径一致。）

**`band_guard` 逐跑状态（in-band per costmap；11 个 5 s 样本/跑）**

| run | local（odom，带 `[0,2]`） | global（map，带） |
|---|---|---|
| `hyb_r1/r2/r3` | **OK 774/774 · 774/774 · 771/771 = 100%**；kept-y `[−0.629,+5.086]` / `[−0.422,+5.270]` / `[−0.428,+5.284]` | **IDLE**（默认 `stvl` ⇒ global 无启用的 2D scan 层，守卫按契约不评估） |
| `off_r1/r2/r3` | **WARN 337/773 · 335/771 · 335/772 = 43.1/43.4/43.4%**（带下 436/436/437、带上 0/0/0）；kept-y `[+0.229,+3.066]` / `[+0.257,+3.281]` / `[+0.272,+3.281]`（**单侧 +y**） | IDLE（同上） |
| `gscan_ov_r1` | OK 774/774 100% | **OK 774/774 100%**，带 `[−1.000,2.000]`，z p50 **−0.180**，kept-y `[−0.633,+5.069]` |
| `gscan_hyb_r2` | OK 772/772 100% | **OK 772/772 100%**，带 `[−1.000,2.000]`，z p50 **−0.179** |
| `gscan_noov_r1` | OK 772/772 100% | **ERROR 0/772 (0.0%)**，带 `[0.000,2.000]`，带下 **772**，z p50 **−0.179**，kept-y `[-, -]`，`hint=ALL 772 beams are outside band` |
| `snav_hyb_r1` | OK 774/774 100%，kept-y `[−0.637,+5.085]` | IDLE（slam_nav 默认 `stvl`） |
| `rmuc_hyb_r1` | OK 553/553 100%，kept-y `[−0.837,+6.394]` | IDLE（默认 `stvl`） |

### 25.6 全局 `−1.0` 覆盖复检：**仍然必须保留（KEEP）**

**问题**：插件两档都修好了（点云不再偏 +7.05 cm）、RMUL 地面也校准到 0 了，
那么 `457701c` 给 `global_obstacle:=scan` 加的 `min_obstacle_height: −1.0`（观测缓冲区闸 + 图层闸两处）
**还需不需要**？答案：**需要，一点没变**。

| 臂 | 运行期 global 带（`ros2 param get`） | **global in-band**（map 帧） | map 帧扫描面 z（p50 [min,max]） | global lethal / inscribed / free | 近 / 前 | `map→odom` z |
|---|---|---|---|---|---|---|
| **带覆盖**（`_ov_r1`） | **−1.0 / 2.0** | **774/774（100%）** | **−0.180** [−0.249, −0.122] | **3872 / 17267 / 9600** | 0.451 / 93 | **−0.215** |
| **带覆盖**（`_hyb_r2`） | −1.0 / 2.0 | 772/772（100%） | −0.179（同档） | 3887 / 17224 / 9608 | 0.448 / 95 | −0.212 |
| **去覆盖**（`_noov_r1`） | **0.0 / 2.0** | **0/772（0%）**，带下 772 | **−0.179** [−0.248, −0.123] | **3713 / 16886 / 10176** | 0.450 / 81 | −0.204 |
| （参照：static+inflation 基线，§15 的"改前"） | — | — | — | **3713 / 16886 / 10176** | — | — |

**判决与理由**

1. **map 帧的扫描面 z 没变**：**−0.179 [−0.248, −0.123]**，与 §15 的 **−0.1786 [−0.237, −0.126]**
   逐值同档。⇒"世界地面校准 + 插件修复"**不可能**把它抬进 `[0, 2]`，因为决定它的是
   **`map→odom` 的 z**（实测 **−0.204…−0.215**，即 map 的 z=0 比 odom 的地面高 ~21 cm），
   而这是**地图/PCD 资产自身的 z 原点**（§13.9 最后两行：既有资产是带旧插件偏置建出来的），
   **不是世界坐标**。校准世界地面只把"车 + 场地"一起平移，`map`（出生点相对系）与 `odom`
   的相对几何不变 ⇒ 对 `map→odom z` 零影响。
2. **去掉覆盖 = 全局 2D scan 层静默死掉**：`0/772` 在带内 ⇒ 一条点都没进云 ⇒ 全局图**恒等于
   static+inflation**（**3713/16886/10176**，与 §15 的"改前"以及 **§10.4 的全局行逐值相同**）。
   而且 nav2 **一个字都不报**（`log_grep` 里 0 条 nav2 高度带/丢包告警）；只有 `band_guard` 会喊
   `ERROR /global_costmap … in-band 0/772`。⇒ 保留覆盖层是**唯一**能让全局图看见实时障碍的路径。
3. **保留的代价为零**：覆盖层是 **opt-in**（只在 `global_obstacle:=scan` 命中；默认 `stvl`/`none`
   不命中，§15.7 已证合并文件 md5 逐字节不变）；对 2D `/scan`，`min_obstacle_height` 只是"扫描面
   高度"的**常量闸**（所有点的 z = 扫描面 z）⇒ 放宽下界**不可能**把地面点放进来，不改变语义。
4. **余量仍然够**：实测 map 帧最低点 **−0.249**，覆盖层下界 **−1.0** ⇒ 余量 **0.75 m**；
   上界 2.0 vs 实测最高 −0.122 ⇒ 余量 2.1 m。
5. **什么时候可以去掉**：只有当**新图被提升**（`map/<world>.yaml` + `PCD/<world>.pcd` 换成 §23/§24
   那条"z 原点 = 地面"的流水线产物）之后。那时 `map→odom z ≈ 0`、扫描面 ≈ `+0.03` > 0.0 ⇒
   覆盖层变**不必要**（但仍**无害**）。**本轮不提升**，所以现在**必须保留**。

### 25.7 §12 风格：`slam_nav` 与 `nav + global_obstacle:=scan`

| 量 | **`slam_nav`（hybrid）** | **`nav + global_obstacle:=scan`（hybrid，n=2）** | §12 的同名量（**gicp 死锁期**） |
|---|---|---|---|
| local lethal / insc / free | **273 / 10265.5 / 40172** | 296·312.5 / 10266·10210 / 40165.5·40247 | 287 / 8426 / 43343.5 |
| local 最近 · ahead | **0.700 · 26** | 0.716 · 22–23 | 0.768 · 18 |
| local in-band | **774/774（100%）** | 774/774、772/772（100%） | 677/677 |
| **global** lethal / insc / free | 228 / 2008 / 678（**13950 格**，实时图） | **3872·3887 / 17267·17224 / 9600·9608**（43077 格） | 239 / 2109 / 1485（17100 格）· 4010/17214/9627（**是 static 撑的**） |
| **global in-band** | IDLE（`stvl` 档，无 2D scan 层） | **774/774 · 772/772（100%）**，带 `[−1,2]` | **0（从没工作过）** |
| `map→odom` 发布者 | **`/slam_toolbox`（唯一）** | `/gicp_registration`（唯一） | `/gicp_registration`（**~6 s 后死**） |
| `map→odom` 数值（窗口末） | **`[0.000, 0.000, −0.034]`，rpy `[−0.004, +0.519, −0.000]°`** | `[−0.101, +0.045, −0.215]`，rpy `[+0.7, +1.0, −0.7]°` | 只有初始值（从未被修正） |
| `/map` | 1 个发布者（`/slam_toolbox`） | 1 个（`/map_server`，latched） | 1 个（`/map_server`） |
| nav2 告警 | **1 条**（`slam_toolbox` 自己的启动期 `Message Filter dropping`，`time 0.000`，与 §12 同一条） | **2–3 条**（都是 `planner_server` 启动期 `Timed out waiting for transform … extrapolation into the past`，图/TF 就绪即消失；**0 条**与高度带有关） | off 128 / hybrid 129 条外推 + 38/39 条 Message Filter |

**判决**

1. **`slam_nav` 与 `mode:=nav` 的局部半仍然逐项重合**（in-band 100%、lethal 273 对 264–373.5、
   最近 0.700 对 0.709–0.717、倾角 0.399° 对 0.415–0.586°）⇒ §12.3 的"同病同解"在**修好之后**
   依然成立，只是"病"已经不在产品路径上（`off` 才病）。
2. **§12.4 的"B 组全局半判不了"这一条被本轮取代**：gicp **采纳 279–295/295、拒绝 0**、
   `map→odom` 整跑活着（`z=−0.215`，rpy ≤1°）⇒ `global_obstacle:=scan` 的全局层**可判**：
   带内 **100%**、全局 lethal **3872–3887**（比 static-only 的 3713 多 **+159…+174** 格）。
   §12 那两跑的 3782/4010 是 **static + inflation**，不是 2D scan 层；今天同一配置在
   **去覆盖**时逐值复现 **3713/16886/10176**（§25.6）—— 这条"当年那 3782/4010 其实是 static"
   因此从"推断"升级为**实测对照**。
3. **nav2 对高度带仍然全程静默**（11 跑 0 条），唯一会喊的是 `band_guard`（§25.5 的表）；
   §12 那种 gicp 死亡告警（`Timed out` 128–129 条 / `Message Filter` 38–39 条）**一条都没有**
   （每跑只剩启动期 2–3 条 planner 外推）。

### 25.8 RMUC2026 冒烟（1 跑）：**in-band 可比；local lethal = 0 是"窗口几何"，不是高度带病**

| 量 | RMUL2026（hybrid，n=3 中位） | **RMUC2026（hybrid，1 跑）** |
|---|---|---|
| `/scan` 有限波束 | 771 | **555 [532–583]**（开阔场，回波少一档） |
| 波束 z(odom) p50 · [min,max] | +0.026 · [+0.011,+0.077] | **+0.026 · [+0.004,+0.034]** |
| **in-band `[0,2]`** | **771/771（100%）** | **555/555（100%）** |
| `band_guard` local | OK 100%，kept-y `[−0.428,+5.270]` | **OK 100%，kept-y `[−0.837,+6.394]`** |
| local lethal / insc / free | 329 / 10290 / 40106.5 | **0 / 0 / 62500**（99/100 帧；1 帧 = 1 个 lethal） |
| local 最近 lethal | 0.712 | **3.147**（只有 1 个样本） |
| **global** lethal / insc / free | 3727 / 16872 / 10176（43077 格） | **9777 [9774–9783] / 47968 / 89790（188102 格）** |
| global 最近 lethal | 0.450 | **2.651** |
| `map→odom` | `[−0.10,+0.05,−0.215]` | **`[+0.049,−0.055,+0.022]`**（rpy ≤0.8°） |
| `h_meas` | 0.2617 INFO | **0.2613 INFO**（\|Δ\|=1.8 mm） |
| 残余地面倾角 | 0.464° | **0.153°** |

**为什么 local 是空的（两条独立量法，同一结论）**

1. **nav2 自己**：global costmap（`map` 帧）到最近 254 格 = **2.651 m**（33 帧，min 2.648 / max 2.654）。
2. **离线量静态图**（`map/RMUC2026.yaml` + `RMUC2026.pgm`，`origin [−25.2,−9.09]`、`res 0.05`）：
   机器人 map 位姿 `(0.032, −0.102)` ⇒ 最近 occupied 格 **2.693 m**，**2.50 m 内 0 个 occupied 格**
   （3.00 m 内 193 个）。
3. 而局部代价图是 **`rolling_window: true`、`width=height=5`、`resolution=0.02`** ⇒ **半宽 2.50 m**。
   `2.65–2.69 m > 2.50 m` ⇒ 窗口里**根本没有墙** ⇒ `lethal=0 / inscribed=0 / free=62500` 是**正确结果**，
   不是 §9.7 那种"整盘掉出带 + 静默空图"。**同一份 5×5 m 窗口在 RMUL 里 lethal≈300**（近墙 0.71 m），
   这条对照说明窗口本身工作正常。
4. 唯一那 1 帧的 605 inscribed / 1 lethal（100 帧里 1 帧）判为**启动瞬态**（第一帧 grid 尚未被
   rolling window 清完），**未深究**（§25.11 第 5 条）。

⇒ **判读**：RMUC 的"in-band 可比、lethal 不可比"是**世界几何**差异（开阔场 + 5 m 窗口），
**不是** hybrid/插件/校准的回归。若要在 RMUC 里量 lethal，得把出生点挪到离墙 <2.5 m 的地方
（本轮**没做**，§25.11 第 4 条）。

### 25.9 nav2 的"静默"（11 跑统一口径：`dropped|out of band|min_obstacle|max_obstacle|isCurrent`）

| run | nav2 侧告警条数 | 其中与高度带相关 | 说明 |
|---|---|---|---|
| `hyb_r1/r2/r3` | 2 / 3 / 3 | **0** | 全是 `planner_server` 启动期 `Timed out waiting for transform`（`base_link_fake→map`，TF 未就绪） |
| `off_r1/r2/r3` | 2 / 2 / 2 | **0** | 同上 |
| `gscan_ov_r1` / `gscan_hyb_r2` | 3 / 2 | **0** | 同上 |
| `gscan_noov_r1` | 3 | **0** | 另有 **11 条**是 `band_guard` 自己的 `ERROR`（**不是** nav2） |
| `snav_hyb_r1` | 1 | **0** | `slam_toolbox` 启动期 `Message Filter dropping`（`time 0.000`） |
| `rmuc_hyb_r1` | **0** | **0** | — |

⇒ **§9.7 那条"没有任何提示"在 11 跑里 11 次复现**（包括 `global_obstacle:=scan` 去掉覆盖后的
**整层静默清空**）；能把这件事说出来的**只有 `band_guard`**（`ERROR 0/772` / `WARN 43.4%`）。

### 25.10 哪些旧数字**被本节取代 / 仍然有效**

| 位置 | 旧值 / 旧结论 | 本节之后 |
|---|---|---|
| **§10.4.1 主表整表**（launch `0161033b…`、spl `58bebfb8…`、**插件修前**） | in-band off 579 / hybrid 678；lethal 193 / 288.5；nearest 0.760 / 0.769；ahead 21 / 18；incl 30.115 / 0.518；scan z p50 +0.243 / +0.058；rpy roll 30.06 / 0.30…0.43 | **作废，由 §25.4 取代**（off 337 / hybrid 771；lethal 299.5 / 329；nearest 0.751 / 0.712；ahead 20 / 22；incl 30.305 / 0.464；z p50 −0.089 / +0.026；roll 30.198 / +0.060）。原因：插件偏置 (0.6 mm, 19.19 cm, 7.05 cm) 修掉 + 世界地面 −4.966 cm + 二进制/launch 全变 |
| **§10.4.1 的 global 行**（3713 / 16886 / 10176，"逐值相同"） | 被当成"全局 STVL 的数" | **重读**：那组数 = **static+inflation 基线**（本节 `gscan_noov_r1` 逐值复现）。默认 `stvl` 档今天 **3727 [3725–3729]**（STVL 只多 +12…+16 格）；`global:=scan` 带覆盖是 **3872–3887**（2D scan 层 +159…+174 格） |
| **§10.3 的 `h_meas = 0.1894 / ERROR`** | 插件 bug 的像 | 本节 **0.2617 [0.2616–0.2617] INFO**、RMUC **0.2613 INFO**（§13 已判，本节复现） |
| **§10.4 的 `/scan` 有限波束 677/678** | — | **作废** → **771**（修插件后的 768–789 档；§13/§17 已指出） |
| **§12 的 5 跑 local 数字**（launch `6d38e74e…`） | snav off 193/0.761/21；hyb 287/0.768/18；gscan off 209/0.760/20；hyb 256.5/0.723/20 | **作废，由 §25.7 取代**（同 §10 的原因：修 bug 前 + 窗口/波束档不同） |
| **§12.4 / §12.6#1 的"`global_obstacle:=scan` 的全局半**判不了**"** | gicp 0/25 采纳、~6 s 判失效 | **取代**：gicp **279–295/295 采纳、0 拒绝**、`map→odom` 整跑活 ⇒ 全局可判（in-band 100%、lethal 3872–3887）。§12 的 3782/4010 是 static+inflation（今天以 `noov` 的 3713/16886/10176 实测对照） |
| **§12 的 `snav_hyb_r1` 全局 239/2109/1485（17100 格）** | 实时图 | **不可直比**（实时图逐跑不同）；今天 **228/2008/678（13950 格）**，同量级 |
| **§13.8 的 1 跑/档 A/B**（264/285/284/292 等） | — | **作废**（同一批"修 bug 前"的口径 + 10 s 窗口）；方向一致 |
| **§15.9 的"全局 in-band `0/774`（`[0,2]`）→ `774/774`（`[−1,2]`）"** | — | **仍然有效，并被本节逐值复现**（`0/772` → `774/774`） |
| **§15.10 的"`stvl` 全局 lethal 3727（3725–3732）"** | — | **仍然有效，复现 3727 [3725–3729]** |
| **§13.8 / §17 的 off in-band 333/776（43%）、`kept-y` 单侧** | — | **仍然有效，复现 337/771（43.4%）、`[+0.257,+3.281]`** |
| **§18.1 的 hybrid 1 跑**（in-band 776/776、incl 0.442、lethal 277、rpy [−0.04,−0.25,−0.22]、h 0.2616） | — | **仍然有效，复现**（771/771、0.464、329、[+0.06,−0.47,−0.63]、0.2617） |

### 25.11 本轮没能验证 / 诚实边界

1. **只有 n=3/档**（`slam_nav`/`global:=scan`/RMUC 是 1–2 跑）⇒ 分辨不出 ≤10% 的差异；§25.4 的
   `[min–max]` 就是本轮能给的噪声带（hybrid 的 lethal 264–373.5 就有 **±55**、nearest 0.709–0.717
   即 **±4 mm**、ahead 21–24 即 **±1.5**）。
2. **只测一条组合**：`robot:=robot11 robot11_mount:=sensor` + `lio:=small_point_lio` + `localization:=gicp`
   + `nav:=mppi` + `planner:=smac2d` + `spin_speed:=0.0` + `RMUL2026`（第 4 项 RMUC2026 1 跑）。
   `hzmirm`/默认模型、`amcl`/`small_gicp`、`rpp`/`dwb`/`teb`、`navfn`、`lio:=fastlio` **都没测**。
3. **`off` 是控制臂，不是产品路径**；它的 local lethal（299.5）**不是 0**，所以**不能**用 lethal 条数
   当"高度带病"的判据 —— 这条本身是本节的一个发现（§25.4 读法 2）。
4. **RMUC 的空局部图只做了"几何解释"，没有"挪车验证"**：两条独立距离（nav2 global 2.651 m、
   离线图 2.693 m）都 > 窗口半宽 2.50 m，但**没有**把出生点挪到离墙 1 m 再跑一跑
   （⇒ "RMUC 里 lethal 能不能非空"**未实测**，只做了同窗口在 RMUL 的对照）。
5. **RMUC 那 1 帧 605 inscribed / 1 lethal 判为启动瞬态，未归因**（100 帧里 1 帧）。
6. **`map→odom z = −0.21` 的构成没有拆开**：只测到总量（hybrid −0.204…−0.215、off −0.049…−0.054），
   **没有**把"地图资产 z 原点"与"旧插件偏置烘进 PCD"两部分的占比分开验证。
7. **进程可见性是 PID 命名空间内的**（本会话的命令跑在 `--unshare-pid` 的沙箱里）：
   `run_one.sh` 的"启动前不许有 gzserver"与收尾的 `gz_left` 都只覆盖**本批次自己的命名空间**
   （11 跑 11 次都是 0）；**机器级**是否有别人的栈，从本会话**看不到** ⇒ "没有两栈重叠"这条
   只能保证**我这边**（单批次顺序执行 + 每跑前后自检 + 收尾按进程组/URI 归属 + 11/11 收尾干净）。
8. **swap 仍是满的**：每跑起跑时 `swap_used_kb = 3992852–3996012`（≈ **3.81 GiB / 4.00 GiB**），
   `MemAvailable` **26.0–26.3 GB**；**11 跑无 OOM、无重试（`RETRY` 0 次）**、每跑收尾
   `残留 (空) / gz=0 / 全局 gzserver=0`（11/11）。但 swap 满这件事本身没有被解决。
9. **`band_guard` 的样本是"整跑级"的**（5 s 周期 × 11 样本，覆盖 launch 后约 55 仿真秒，
   **不是**严格的 20 s 窗口）；`kept-y`/in-band 取它的中位，窗口数字用 `navband`。
10. **LIO 的 z 漂移比 §10 更大**：窗口末端 `base_link` z = +0.025…+0.037（init 由构造 = +0.1025）
    ⇒ 漂移 **−0.065…−0.077**（§10 记的是 −0.045）。**没有**归因、没有修；只记一条**风险**：
    `hyb_r3` 的最低波束端点 **+0.007 m** ⇒ 距下界只有 **7 mm**，若漂移再涨 ~2 cm，
    `hybrid` 也会开始掉波束（`band_guard` 会喊 WARN/ERROR，本轮 11 跑 0 次）。
11. **没有录 bag、没有做重放一致性**（§10.6 那种 141/141 行 diff 本轮没做）；也没有算 ms/帧。
12. **没测 `mode:=mapping`**（`band_guard` 在该档不起，§17 的 condition 已定），也没测 `local_obstacle`
    槽的其它取值（`cloud`/`both`/`stvl`）。

### 25.12 一句话给用户

> **当前 master 上，`hybrid` 默认档是干净的、也是唯一该走的路径**：`/scan` **771/771 全在带内**、
> 倾角 **0.464°**、`h_meas` **0.2617 m / INFO**、局部 lethal **329**、最近 lethal **0.712 m**；
> `off` 只有 **337/771（43.7%）** 在带内、`kept-y` 是**单侧条带**（`band_guard` 判 `WARN`），
> 但它的 **lethal 299.5 ≠ 0** —— **别再用 lethal 条数当判据**（§9.7 的 `lethal 0` 是插件 bug
> + z 原点的复合像；今天同样的病只表现为"可见度 43%"）。**全局 `global_obstacle:=scan` 的 −1.0 覆盖
> 必须保留**：去掉后 map 帧扫描面仍在 **−0.179 m**（`map→odom z ≈ −0.21`，是**地图资产的 z 原点**、
> 不是世界地面），**0/772 进带**、全局图静默退回 **3713/16886/10176**（static-only），
> 而 nav2 **一声不响**；`band_guard` 是唯一会喊的。**`slam_nav`** 同样 100% 在带内、`/slam_toolbox`
> 独发 `map→odom` `z=−0.034` 健康；**§12 的"全局半判不了"现在可以判了**（gicp 279–295/295 采纳）。
> **RMUC2026** 的 in-band 也 100%，**local lethal=0 是窗口几何**（最近墙 2.65/2.69 m > 窗口半宽 2.50 m），
> 不是回归。**旧数字**：§10.4 整表、§12 的 local 与"判不了"、§13.8 全部**作废**；
> §15.9/§15.10/§17/§18 的结论**仍然有效并被复现**（`0/774→774/774`、3727、43%、776/776）。

## §26 第十七轮（**本机 date=2026-10-20T19:46–20:07**）：给**局部 2D scan 高度带**加 **5 cm 负下界余量**（缺陷 "b"）—— `scan_band_min` 默认 −0.05、回滚 `0.0` **逐字节**、STVL/全局 `−1.0` 一个字节没动

> **本节只新增**（append-only）：§1–§25 一个字都没改。写入时并行的 §27 已在工作树里（未提交）⇒
> 按父任务裁定本节占用 **§26**，放在 §27 **之前**（保持编号顺序），**§27 的正文一个字没动**。
>
> **对象**：§25.11 第 10 条登记的**缺陷 "b"** —— hybrid 默认档的 local `/scan` 是 **771/771（100%）**
> 在带内，但**最低波束端点只有 +0.007 m**（`e25_hyb_r3`），而同一批跑的 LIO z 漂移是
> **−0.065…−0.077 m**。⇒ 距下界 **7 mm**；LIO 再多漂 ~1 cm，波束就整批掉到 0.0 以下，nav2
> **静默**丢点（§9.7/§15.6：无日志、无 WARN、无 dropped 计数），默认档悄悄退回 `off` 那种
> **kept-y 单侧条带**的"半盲"状态。本轮把这个余量从 **7 mm 拉到 ~57 mm**（下界 −0.05）。
>
> **代码（3 处，全部 opt-in/带文档）**：
> ① 新文件 `src/rm_nav_bringup/config/nav2_params_sim_local_scan_band.yaml`（只含 **2 个叶子键**）；
> ② `_Nav2ParamsForSlot` 支持 4 元组 `(tag, path, predicate, bindings)`：覆盖层的叶子键值可以由
> launch 参数**绑定**，`0.0` 时由 predicate 整层跳过；
> ③ 新 launch 参数 `scan_band_min`（默认 `-0.05`，取值域 **[−1.0, 0.0] 只许下调**；回滚 `:=0.0`）。
> **未改**：`src/rm_navigation/**`（公共段）、`src/rm_localization/**`、`src/rm_simulation/**`、
> 地图/PCD 资产（**并行任务在 19:47–19:48 改了默认 RMUL 资产，不是本节**，见 26.2）、§25 正文。
>
> **7 跑**（3 默认 + 1 off + 1 干净默认 + 1 `global_obstacle:=scan` + 1 回滚），**0 次 RETRY**，
> 每跑收尾 `残留 (空) / gz=0 / 全局 gzserver=0`。原始数据 `.tmp_ab/e26_*`（gitignored）。
> 运行期证据：`meta_before/after`（资产/二进制 md5）、`roslog/nav2_params_sim_*_merged.yaml`（合并参数
> 文件原件）、`params.txt`（`ros2 param get`）、`band_guard.txt`（live 带 + in-band）、`navband.json`、
> `probe.json`、离线 `merge_probe.py` / `summary.py`。

### 26.1 一句话结论

> **局部 2D scan 的下界从 0.0 降到 −0.05 m 之后，`hybrid` 默认档仍然 771/771（100%）在带内**
> （`band_guard` 按 live 带 `[−0.05, 2.0]` 判 **100%**、`below=0`），最低端点 **+0.014…+0.021 m**
> （200 帧绝对最小 **+0.0129 m**）⇒ **距新下界 62.9–70.8 mm**（§25 那个最坏跑只有 **7 mm**；
> 即便按下界 0.0 的老口径，本轮也有 **12.9–20.3 mm**）。**`off` 控制臂没有被救**：navband 的
> `[0,2]` in-band 仍是 **337/771 = 43.7%**（与 §25 **逐值相同**），live 带 `[−0.05,2]` 下也才
> **44.8–45.4%**、kept-y 仍**单侧 +y**、`band_guard` 仍 **WARN** ⇒ 这 5 cm 是**鲁棒性余量，不是解药**。
> **没有把地面/近地点放进局部图**：hybrid 的 200 帧里**最低波束端点也 > 0**（+0.0129…+0.0150）
> ⇒ **落在 `[−0.05, 0)` 的波束 = 0 条** ⇒ 进 `ObservationBuffer` 的点集与老带**完全相同**；
> local lethal/inscribed/free 与 §25 的差（**−52 / −109 / +282.5**，中位）在历史逐跑散布内，且
> **回滚跑（0.0）的 lethal 反而更高（317）**。**合并 YAML**：`global_obstacle:=stvl` 档只多
> **2 行**（local 的两个 `min_obstacle_height`），**`global_costmap` 子树 md5 逐字节相同**
> （STVL 的 `livox_mark.min/max = 0.0/2.0` 原样）；**回滚 `scan_band_min:=0.0` 时整份合并文件
> 逐字节等于改动前**（`e9b39f36…`）；`global_obstacle:=scan` 的 **§15 `−1.0` 仍在**
> （live 回读 `−1.0/2.0`、global in-band **100%**）。

### 26.2 协议、资产（**关键：并行任务改了默认 RMUL 资产**）与隔离

协议与 §25.2 **逐字相同**（`settle 20 / window 20`；`gui:=False nav_rviz:=False lio_rviz:=False`；
`world:=RMUL2026`；`robot:=robot11 robot11_mount:=sensor`；`lio:=small_point_lio`；`spin_speed:=0.0`；
`gravity_align_mode` 不传 ⇒ `auto` ⇒ hybrid（off 臂显式 `:=off`）；`mode:=nav localization:=gicp
nav:=mppi planner:=smac2d`；`band_guard:=True`）。7 跑全部由 `.tmp_ab/exp26/run_one.sh` **顺序**执行
（`batch.sh` + `batch_clean.sh`，每跑 `timeout 900`、失败重试 1 次；实际 **0 次重试**）。

```bash
# 主批（19:46–19:58）：3 默认 + 1 off（= 任务书要求的 4 跑）
.tmp_ab/exp26/run_one.sh e26_hyb_r1 world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp \
    nav:=mppi planner:=smac2d robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
#   … e26_hyb_r2 / e26_hyb_r3 同上
.tmp_ab/exp26/run_one.sh e26_off_r1  …同上… gravity_align_mode:=off
# 干净补充（19:58–20:04，等并行任务暂停后）：默认 1 跑 + global_obstacle:=scan 1 跑
.tmp_ab/exp26/run_one.sh e26_hyb_g_r1    …同上…
.tmp_ab/exp26/run_one.sh e26_gscan_hyb_r1 …同上… global_obstacle:=scan
# 回滚对照（20:05–20:07）：一条参数把下界改回 0.0
.tmp_ab/exp26/run_one.sh e26_rollback_r1 …同上… scan_band_min:=0.0
```

**资产事件（必须写清）**：并行任务（§27，父任务裁定为另一个 agent）在 **19:47:59–19:48:05**
把默认 RMUL 资产换成了提升版：

| 资产 | **旧**（§25 用的，= HEAD `dfd9810`） | **新**（19:48 之后，= §27 的提升版） |
|---|---|---|
| `map/RMUL2026.yaml` | `fd23dd82c0aa63b6c2fdf3644c7c9253` | **`bf3f37cb40a1b0535bc934d6c29186e2`** |
| `map/RMUL2026.pgm` | `6e45a9327e99d57883881f7a7d10c3d0` | **`7ec12dc323ae18f92df765f288f8f0e1`** |
| `PCD/RMUL2026.pcd` | `5d0d33eb5219d9416606a7b0f5e34028` | **`1d5ca30e9d0268d971306f4f497cfb4f`** |

⇒ **`e26_hyb_r1`（19:46:17 起跑）跑前=旧资产**，其余 6 跑=新资产；`hyb_r1` 的
`meta_md5_diff.txt` 只有 `map/RMUL2026.yaml` 1 行（**跑中**被换），其余 6 跑 **0 行**。
**为什么主表仍可比 §25**：nav2 的**局部** costmap 是 `global_frame: odom` 的滚动窗，吃
`/scan` 的 odom 坐标 ⇒ **与 2D 先验图/PCD 无关**（本轮 4 个 hybrid 跑的 local 数字同量级可证）；
受资产影响的是**全局**列（map 帧 + 先验图 + STVL/static）⇒ 26.8 单列干净资产上的全局数。
**全局列在 §25 与本轮之间不可直比**（图从 43077 格变 39446 格）。

**隔离**（与 §12/§15/§17/§25 同口径）：私有 `HOME=.tmp_ab/home-<tag>`、按 tag 散列
`ROS_DOMAIN_ID=100…159`、先探空端口再占 `GAZEBO_MASTER_URI`（11700–11899）、`unset DISPLAY`、
`setsid` 起栈（进程组=本 run）、硬超时、收尾**只用** `tools/scripts/lib/scoped_cleanup.sh`
按进程组/master URI 清（**无全机 pkill**）。每跑起跑前 `pgrep -x gzserver` 自检，批次日志里
`gz_left=0`（7/7）。**并发的第二套栈**：19:46 时 `ss -ltn` 除我的 master 外还看到 **11737**
（并行任务的），`load` 一度 **10.4**；我**没有**杀它，只在自己的批次里串行。并行任务 19:48 之后
暂停，干净补充跑的端口只剩我自己的（`ss` 记录在 `batch_clean.log`）。RTF **0.338–0.422**
（§25 是 0.400–0.418；低值来自并发负载，不是本轮改动）。
**一次被弃的冒烟跑**：19:43–19:45 的 `e26_hyb_r1` 在第 1 分钟被我 kill（改注释日期以保持冻结 md5
一致），整目录留档 `.tmp_ab/e26_hyb_r1.killed-1944`，**不计入**任何数字。

### 26.3 代码改动（file:line）与**一条参数回滚**

| # | 文件:行 | 改动 |
|---|---|---|
| ① | `config/nav2_params_sim_local_scan_band.yaml:1–55`（键在 **`:52` / `:55`**） | 新覆盖层：`local_costmap.obstacle_layer.scan.min_obstacle_height = -0.05`（观测缓冲区闸）**与** `local_costmap.obstacle_layer.min_obstacle_height = -0.05`（`updateBounds` 第二道闸）。**只写 local**；文件头 1–46 行写清根因、2D 平盘语义、为什么不碰 STVL/全局 −1.0、回滚方式 |
| ② | `launch/bringup_sim.launch.py:1092–1160` | `_Nav2ParamsForSlot`：3 元组仍兼容；4 元组多一个 `bindings`（点分叶子键 → `Substitution`），`perform` 里先 `yaml.safe_load(subst.perform(context))` 再深合并（**求值成 float，不是字符串**）；临时文件名按命中的 tag 连接 |
| ③ | 同文件 `:1171–1208` | `_SCAN_BAND_MIN_DEFAULT='-0.05'`、合法域 `[-1.0, 0.0]`；`_scan_band_min_value()`（非数/越界**硬报错**，不静默夹紧）；`_scan_band_layer_active()`（命中条件 = **`scan_band_min != 0.0` 且 `local_obstacle ∈ {scan, both, stvl_both}`**） |
| ④ | 同文件 `:1948–1972` | 把新层注册进 `layers`（tag=`local_scan_band`），两个键分别绑定 `LaunchConfiguration('scan_band_min')`（只作用于 `local_costmap`；全局 `−1.0` 层仍由 §15 的 `global_scan` 层管） |
| ⑤ | 同文件 `:2390–2405` / `:3683–3684` | `DeclareLaunchArgument('scan_band_min', default='-0.05')` + 加进 `LaunchDescription`（在 `global_obstacle`/`local_obstacle` 之后、`band_guard` 之前） |

**回滚（一条参数）**：`scan_band_min:=0.0` ⇒ predicate 直接跳过该层 ⇒ 合并文件**逐字节**等于改动前
（26.4 实测 md5 `e9b39f36…`，`ros2 param get` 回到 `0.0/0.0`）。也可把 launch 的 `default_value` 改回
`'0.0'`。**正数被拒绝**（`scan_band_min:=0.1` ⇒ t=0 硬报错）：抬高下界会让更多波束出带，正是本修法
要防的静默失效；`[−1.0, 0.0]` 之外同理。`ros2 launch … --show-args` 多出且仅多出 `scan_band_min`
一条参数（默认 `-0.05`）。

### 26.4 合并 YAML 证据（hybrid 只多 2 行；回滚整文件逐字节；STVL 子树逐字节）

用**两份真实代码**离线复算 `_Nav2ParamsForSlot`（`before` = `git show HEAD:…` 的 launch；
`after`/`rollback` = 当前工作树；脚本 `.tmp_ab/exp26/merge_probe.py`，只读、不启栈），并与
**实跑 launch 自己写出的** `roslog/nav2_params_sim_*_merged.yaml` 逐字节对照：

| 档 | 改动前（before） | 改动后（after） | 回滚（`scan_band_min:=0.0`） | 实跑 merged == 复算? |
|---|---|---|---|---|
| `robot11`+`global_obstacle:=stvl`（默认/hybrid） | `e9b39f36…`（**9526 B**；= §15.10/§25 的冻结值） | `71e6b591…`（**9598 B**） | **`e9b39f36…`（逐字节）** | ✅ 3 跑都是 `71e6b591…`；回滚跑是 `e9b39f36…` |
| `robot11`+`global_obstacle:=scan` | `e5179abd…`（**9596 B**；= §17/§25 的冻结值） | `1ef5a433…`（**9668 B**） | **`e5179abd…`（逐字节）** | ✅ `e26_gscan_hyb_r1` = `1ef5a433…` |
| `robot11`+`local_obstacle:=cloud`（不用 2D scan） | `e9b39f36…` | **`e9b39f36…`（逐字节，predicate 不命中）** | 同左 | —（离线） |

**`diff -u before_stvl.yaml after_stvl.yaml` 的全部内容**（`after_gscan` 与它**逐字相同**）：

```diff
@@ -126,6 +126,8 @@
           marking: true
           inf_is_valid: true
           data_type: LaserScan
+          min_obstacle_height: -0.05     # ① scan 源闸（local_costmap）
+        min_obstacle_height: -0.05       # ② 图层闸（local_costmap）
       obstacle_cloud_layer:
```

**逐键核对**（canonical YAML subtree md5）：

| 量 | stvl 档 before → after | scan 档 before → after |
|---|---|---|
| **`global_costmap` 整段** | `91484dc4a0d0` → **同** | `e0cdc71c6cdc` → **同** |
| `global` `stvl_layer.livox_mark.min/max` | `0.0 / 2.0` → **同** | `0.0 / 2.0` → **同** |
| `global` `obstacle_layer.scan.min` / `layer.min` | 无（默认 0.0）→ 同 | **`−1.0 / −1.0` → 同**（§15 覆盖保留） |
| `local` `obstacle_layer.scan.max` | `2.0` → **2.0** | `2.0` → **2.0** |
| `local` `stvl_layer.livox_mark.min/max` | `0.0 / 2.0` → **同** | `0.0 / 2.0` → **同** |
| `local` `obstacle_layer.scan.min` / `layer.min` | 无 → **`−0.05 / −0.05`** | 无 → **`−0.05 / −0.05`** |

> **关于"`stvl` 合并 YAML 逐字节不变"的诚实口径**：按**整文件**字面，这一条**做不到**——本轮的
> 改动按设计就落在**同一份**默认档合并文件的 `local_costmap` 段（这正是"默认档现在用 −0.05"的定义）。
> 能交付且更强的三层证据是：① **`global_costmap` 子树（含 STVL 的 3D 带、§15 的 −1.0）逐字节相同**；
> ② **STVL 的 `livox_mark.min/max` 运行期回读仍是 `0.0/2.0`**（26.7）；③ **回滚 `scan_band_min:=0.0`
> 时整文件逐字节**等于改动前（`e9b39f36…`/`e5179abd…`）。除此之外，`local_obstacle:=cloud`（不用 2D
> scan 源）也逐字节不变。

### 26.5 主表：默认档 3 跑 + `off` 1 跑（任务书要求的 4 跑）

`median[min–max]`；`z_min` 两口径：**窗口 median**（200 帧逐帧最小值的 median，= §25 表的口径）与
**200 帧绝对最小**；"margin" = 该 z_min − 下界。`global` 列**只对 `hyb_r1`（旧图）与 `r2/r3`（新图）**
分别有效，**跨资产不可比**（26.2/26.8）。

| 指标 | **默认（`auto`⇒hybrid）n=3** | **`gravity_align_mode:=off` n=1** | **§25.4 的 hybrid（n=3，旧图）** |
|---|---|---|---|
| `/scan` 有限波束/帧 | 771 [771–771] | 771 | 771 |
| 波束 z(odom) p50 | +0.0353 [+0.0351, +0.0364] | −0.0892 | +0.026 [+0.025,+0.037] |
| **z_min（窗口 median）** | **+0.0199 [+0.0184, +0.0203]** | −0.4251 | **+0.011 [+0.007,+0.021]** |
| **z_min（200 帧绝对）** | **+0.0144 [+0.0142, +0.0150]** | −0.6896 | +0.004…+0.014（由 §25 JSON 复算） |
| 在带 `[0,2]` 内 | **771 [771–771]（100%）** | **337（43.71%）** | 771（100%） |
| 带下 / 带上（`[0,2]`） | **0 / 0** | **432 / 3** | 0 / 0 |
| **margin vs 旧下界 0.0** | **+19.9 mm [+18.4, +20.3]** | −425.1 mm | **+11 [+7,+21] mm**（最坏 **7 mm**） |
| **margin vs 新下界 −0.05（窗口 median）** | **+69.9 mm [+68.4, +70.3]** | — | +61 [+57,+71] mm |
| **margin vs 新下界 −0.05（绝对最小）** | **+64.4 mm [+64.2, +65.0]** | — | +54 mm（按 §25 绝对最小 +0.0042） |
| `band_guard` local（live 带） | **OK 100.0%**，772/772（771–773），`below=0`，带 `[−0.05,2.0]` | **WARN 45.4%**，351.5/773.5，`below=420`，**kept-y 单侧 +y** | OK 100%，带 `[0,2]` |
| `band_guard` kept-y | **[−0.763, +5.106] m（双侧、跨原点）** | **[+0.153, +3.272] m（单侧 +y）** | [−0.428, +5.270] |
| local **lethal** | 277 [245.5–285] | 306.5 | **329 [264–373.5]** |
| local **inscribed** | 10181 [9529–10306.5] | 10093.5 | 10290 [10265–10353.5] |
| local **free** | 40389 [40274–42460] | 41570.5 | 40106.5 [40042.5–40154.5] |
| **最近 lethal** | 0.713 [0.708–0.716] | 0.719 | 0.712 [0.709–0.717] |
| **前方 2.0×1.2 m lethal** | 23 [22–23] | 22 | 22 [21–24] |
| **地面残余倾角** | 0.466° [0.407–0.491] | **30.313°** | 0.464° [0.415–0.586] |
| **`h_meas`** | **0.2616 [0.2616–0.2620] m / INFO**（\|Δ\| 2.1–2.5 mm） | —（off 无互检） | 0.2617 / INFO |
| `odom→base_link` roll | −0.054° [−0.062, −0.001] | **30.200°** | +0.060° |
| RTF | 0.393 [0.338–0.402] | 0.416 | 0.406 [0.400–0.415] |
| **global lethal**（资产） | `r1` 3725（旧） / `r2` 2159·`r3` 2162（新） | 2163（新） | 3727 [3725–3729]（旧） |

**逐跑明细（`scan` 与 local；资产列 = 跑前 `meta_before` 的 md5 前 8 位）**

| run | 资产(map yaml/pgm/PCD) | finite | z_min 窗口 med / 绝对 | margin@0 / @−0.05（窗口 med） | in `[0,2]` | band_guard | local lethal/insc/free | `h_meas` | incl |
|---|---|---|---|---|---|---|---|---|---|
| `e26_hyb_r1` | `fd23dd82`/`6e45a932`/`5d0d33eb`（**旧**） | 771 | +0.0203 / +0.0142 | +20.3 / **+70.3** mm | 771 | OK 100% 772/772 | 285 / 10306.5 / 40274 | 0.2616 INFO | 0.407° |
| `e26_hyb_r2` | `bf3f37cb`/`7ec12dc3`/`1d5ca30e`（新） | 771 | +0.0199 / +0.0150 | +19.9 / **+69.9** mm | 771 | OK 100% 773/773 | 277 / 10181 / 40389 | 0.2616 INFO | 0.466° |
| `e26_hyb_r3` | 新 | 771 | +0.0184 / +0.0144 | +18.4 / **+68.4** mm | 771 | OK 100% 771/771 | **245.5 / 9529 / 42460** | 0.2620 INFO | 0.491° |
| `e26_off_r1` | 新 | 771 | **−0.4251** / **−0.6896** | −425.1 / −375.1 mm | **337** | **WARN 45.4%**（kept-y 单侧） | 306.5 / 10093.5 / 41570.5 | — | **30.313°** |
| `e26_hyb_g_r1`（干净补充） | 新，`meta` 0 行 | 771 | +0.0208 / +0.0149 | +20.8 / **+70.8** mm | 771 | OK 100% 772.5/772.5 | 272.5 / 10188.5 / 40370 | 0.2616 INFO | 0.469° |
| `e26_gscan_hyb_r1`（干净补充） | 新，`meta` 0 行 | 771 | +0.0216 / +0.0142 | +21.6 / **+71.6** mm | 771 | local OK 100% 773/773；**global OK 100% 773/773，带 `[−1,2]`** | 262 / 10206.5 / 40372 | 0.2616 INFO | 0.469° |
| `e26_rollback_r1`（`scan_band_min:=0.0`） | 新，`meta` 0 行 | 771 | +0.0199 / +0.0129 | +19.9 / +69.9 mm（带已回 0.0） | 771 | OK 100% 774/774，带 **`[0,2]`** | **317** / 10064 / 40530 | 0.2616 INFO | 0.366° |

**与 §25 的差（hybrid n=3 中位，任务书要求的"same order"）**：lethal **277 vs 329 ⇒ −52（−15.8%）**、
inscribed **10181 vs 10290 ⇒ −109（−1.1%）**、free **40389 vs 40106.5 ⇒ +282.5（+0.70%）**、
nearest **0.713 vs 0.712**、ahead **23 vs 22**。n=4（含干净补充跑）时 **274.75 / 10184.75 / 40379.5**
⇒ 差 **−54.25 / −105.25 / +273**。**没有"多出地面格"的方向性**（lethal 反而**降**、free 反而**升**，
与"放宽下界放进更多 lethal"相反的符号）。

### 26.6 余量：**7 mm → 57–71 mm**（本轮的核心交付）

| 量 | 值 | 出处 |
|---|---|---|
| §25 最低波束端点（最坏跑 `e25_hyb_r3`） | **+0.007 m** | §25.5/§25.11#10 |
| §25 LIO z 漂移（窗口末端 base_link z − 初值） | **−0.065…−0.077 m** | §25.11#10 |
| §25 的余量（端点 − 下界 0.0） | **7 mm** ⇒ "一次小扰动就会静默掉带" | 缺陷 "b" 的描述 |
| 本轮 hybrid 最低端点（窗口 median） | **+0.0184…+0.0216 m** | 3 跑 + 干净补充 |
| 本轮 hybrid 最低端点（200 帧绝对） | **+0.0129…+0.0150 m**（含回滚跑） | 7 跑 JSON |
| **本轮 margin vs 旧下界 0.0** | **+12.9…+21.6 mm** | 逐跑 |
| **本轮 margin vs 新下界 −0.05** | **+62.9…+71.6 mm**（median 口径 **+68.4…+71.6**） | 逐跑 |
| **若把 §25 那个 +0.007 的端点放到新带** | **+57 mm** | 0.007 − (−0.05) |
| 相对§25余量的倍数 | **≈ 8×**（57 mm / 7 mm） | — |

**读法**：这 5 cm 不是"标定到某个最优值"，而是给**LIO z 漂移**（§25 实测 65–77 mm 量级、且**没有
归因/没有修**）留出与它同量级的余量；`scan_band_min` 的**默认值**就是这条余量，**回滚**是 0.0。

### 26.7 "没有把地面/近地点加进 local costmap"的证明

1. **语义**：`/scan` 是**二维平盘**（LaserScan 无高度），nav2 先 `projectLaser`（z 置 0）再
   `transformLaserScanToPointCloud` 搬到代价图帧 ⇒ **同一帧里所有波束端点的 z 完全相同**（= 扫描面
   在该帧的 z）。`min_obstacle_height` 只是这张平盘的**常量闸**；把下界从 0.0 降到 −0.05 不可能
   引入"地面上不同高度的点"（地面点走 STVL / `obstacle_cloud_layer` 那条链，本轮**一个字节没动**）。
2. **数据（决定性）**：hybrid 的 200 帧里**最低端点也 > 0**（本轮 4 个 hybrid 跑：绝对最小
   **+0.0129…+0.0150**）⇒ **落在 `[−0.05, 0)` 的波束 = 0 条** ⇒ `ObservationBuffer` 收到的点集与
   老带**完全一致**；`band_guard` 的 `below` 在 hybrid 里 **0**（7 跑里 5 个 hybrid 全 0）。
3. **回滚 A/B（同资产）**：`e26_rollback_r1`（带 `[0,2]`）local = **317 / 10064 / 40530**；
   同资产、带 `[−0.05,2]` 的 3 跑（`r2/r3/g_r1`）中位 = **272.5 / 10181 / 40389**。差异方向是
   **回滚跑的 lethal 更高**（317 > 272.5），**没有**"宽带 ⇒ lethal/inscribed 增加、free 减少"的系统性
   信号 ⇒ 观测到的差是**逐跑散布**（§25 的 lethal 本身就有 264–373.5 的 ±55），不是这 5 cm 造成的。
4. **`off` 对照（宽带会多放几条近零波束，但救不了斜装）**：navband 的 `[0,2]` in-band 仍是
   **337/771 = 43.71%**（与 §25 **逐值相同**，因为这个口径不看配置）；`band_guard` 按 live 带
   `[−0.05,2]` 是 **44.8–45.4%**、`below` 从 §25 的 433 降到 **420–435**（≈ **4 条/帧**进了带），
   但 **kept-y 仍是 `[+0.153,+3.272]` 单侧 +y**、`band_guard` 仍 **WARN**、倾角仍 **30.3°**
   ⇒ **余量不是解药**（正如任务书要求）。
5. **STVL/3D 源**：`stvl_layer.livox_mark.min/max` 在两张 costmap 的运行期回读都是 **0.0 / 2.0**
   （26.8），本轮没有把任何 3D 近地点放进来。

### 26.8 干净的补充 2 跑（**stable promoted assets**）+ §15 `−1.0` 复检

并行任务 19:48 之后暂停，补齐两跑（`meta_md5_diff` 均为 **0 行**）：

| 量 | `e26_hyb_g_r1`（默认 stvl） | `e26_gscan_hyb_r1`（`global_obstacle:=scan`） |
|---|---|---|
| local 带（live） | **`[−0.050, 2.000]`**，OK 100% | **`[−0.050, 2.000]`**，OK 100% |
| **global 带（live）** | `obstacle_layer` disabled（stvl 档） | **`[−1.000, 2.000]`**（§15 的 −1.0 仍在），global in-band **773/773 = 100%**，z p50 **+0.119** |
| local lethal / insc / free | 272.5 / 10188.5 / 40370 | 262 / 10206.5 / 40372 |
| **global lethal / insc / free（39446 格）** | **2163 / 12260 / 12812** | **2275 / 12755 / 12154**（2D scan 层 +112 格） |
| global nearest | 0.710 | 0.713 |
| merged md5 | `71e6b591…`（= after_stvl） | **`1ef5a433…`（= after_gscan）** |
| `h_meas` / incl / RTF | 0.2616 INFO / 0.469° / 0.416 | 0.2616 INFO / 0.469° / 0.401 |

> **全局基线换代**：新图的 global cell 数是 **39446**（旧图 43077），所以 §25 的
> `3727 / 16875 / 10176` **不能**与今天的 `2163 / 12260 / 12812` 直比（前者是旧图）。本轮只声明：
> **默认档的全局图在新图上非空且 STVL 在工作**、**`global_obstacle:=scan` 的 −1.0 覆盖在新图上仍然
> 必须/有效**（global in-band 100%，去掉它会不会仍 0 未重测——§15/§25 的机理未变，但**本轮没做
> 去覆盖对照**）。

### 26.9 运行期 `ros2 param get`（live costmap 节点；`params.txt` 原文）

默认档（`e26_hyb_g_r1`；`e26_hyb_r1/r2/r3` 逐字相同）：

```text
/local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height     = Double value is: -0.05
/local_costmap/local_costmap obstacle_layer.scan.max_obstacle_height     = Double value is: 2.0
/local_costmap/local_costmap obstacle_layer.min_obstacle_height          = Double value is: -0.05
/local_costmap/local_costmap obstacle_layer.max_obstacle_height          = Double value is: 2.0
/local_costmap/local_costmap obstacle_layer.enabled                      = Boolean value is: True
/global_costmap/global_costmap obstacle_layer.scan.min_obstacle_height   = Double value is: 0.0
/global_costmap/global_costmap obstacle_layer.min_obstacle_height        = Double value is: 0.0
/global_costmap/global_costmap obstacle_layer.enabled                    = Boolean value is: False
/global_costmap/global_costmap stvl_layer.enabled                        = Boolean value is: True
/local_costmap/local_costmap stvl_layer.livox_mark.min_obstacle_height   = Double value is: 0.0
/local_costmap/local_costmap stvl_layer.livox_mark.max_obstacle_height   = Double value is: 2.0
/global_costmap/global_costmap stvl_layer.livox_mark.min_obstacle_height = Double value is: 0.0
/global_costmap/global_costmap stvl_layer.livox_mark.max_obstacle_height = Double value is: 2.0
```

`global_obstacle:=scan` 档（`e26_gscan_hyb_r1`）**只把 global 的 obstacle 层翻过来，`−1.0` 原样**：

```text
/global_costmap/global_costmap obstacle_layer.scan.min_obstacle_height   = Double value is: -1.0
/global_costmap/global_costmap obstacle_layer.min_obstacle_height        = Double value is: -1.0
/global_costmap/global_costmap obstacle_layer.enabled                    = Boolean value is: True
/global_costmap/global_costmap stvl_layer.enabled                        = Boolean value is: False
（local 仍是 -0.05/2.0；stvl 的 0.0/2.0 两档都不变）
```

`band_guard` 的 live 回读（默认档）：

```text
[band_guard] 参数回读（live）/local_costmap/local_costmap: … obstacle_layer.band=[-0.050,2.000]
             obstacle_layer.scan[LaserScan topic=/scan src.band=[-0.050,2.000]] …
[band_guard] OK /local_costmap/local_costmap frame=odom layer=obstacle_layer source=scan |
             in-band 772/772 (100.0%) below=0 above=0 … | band=[-0.050,2.000]
             source=[-0.050,2.000] layer=[-0.050,2.000] | hint=none
[band_guard] WARN /local_costmap/…（off 臂）| in-band 351/784 (44.8%) below=429 |
             kept-y=[0.162,3.282] … | hint=kept-y is ONE-SIDED (+y) …
```

> 注：`band_guard` 的样本是**整跑级**（5 s × 11 样本，覆盖 launch 后约 55 仿真秒，**不是** 20 s
> 窗口）；所以它的 in-band 计数（771–774）与 navband 的 771 有一两帧的口径差（与 §25 同）。

### 26.10 本轮没能验证 / 诚实清单

1. **默认 RMUL 资产在批次中途被并行任务换了**（19:47:59–19:48:05，见 26.2）。⇒ `hyb_r1` 的
   **全局**列是旧图，`r2/r3/off/clean/rollback` 是新图；**跨资产不可比**。**局部**列与图无关；
   干净的全局数只有 `e26_hyb_g_r1`（新图）与 `e26_gscan_hyb_r1`（新图）两跑。
2. **§25 的全局数字（3727/16875/10176，43077 格）在提升版资产上作废**（新图 39446 格、
   2163/12260/12812）。§25 的**局部**结论（771/771、0.464°、h 0.2617、43.7%）**仍然有效**并被复现。
3. **`hyb_r3` 的 local lethal/free（245.5 / 42460）是个离群跑**：比 §25 中位低 83.5、free 高 2353。
   **不是**这 5 cm 造成的（该跑 `band_guard below=0`、200 帧最低端点 +0.0144 > 0 ⇒ 宽带一条都没多放）；
   最可能是 LIO odom 轨迹/姿态的逐跑差异 + 并发负载（RTF 0.40）。**没有**逐帧归因。n=3 太小，
   不能把 lethal 的 ±50 级差当判据（§25.4 读法 2 同理）。
4. **n 小**：hybrid n=3（+1 干净补充）、off n=1、gscan n=1、rollback n=1；没有录 bag/重放，
   没算 ms/帧，没做多世界（RMUC2026 未测）。
5. **只测一条组合**：`robot11 sensor` + `small_point_lio` + `gicp` + `mppi` + `smac2d` +
   `spin_speed 0.0` + `world RMUL2026`。`hzmirm`/默认模型、`amcl`/`small_gicp`、`rpp/dwb/teb`、
   `lio:=fastlio`、`RMUC2026`、`mode:=mapping`、`local_obstacle` 的 `cloud/stvl` 实跑**都没测**
   （`cloud` 只做了离线合并证明逐字节不变）。
6. **"`stvl` 合并 YAML 逐字节不变"按整文件做不到**（改动按设计落在同一份默认档文件的 local 段）；
   已给的三层替代证据见 26.4。**没有**做"去掉 §15 −1.0 覆盖"的对照跑（§15/§25 已证，本轮未复测）。
7. **`−0.05` 是余量，不是标定**：没有扫"最优下界"；安全性来自 2D 平盘语义 + 实测最低端点 > 0，
   不是来自调参。取值域 [−1.0, 0.0] 是人为约束（正数硬报错），`−1.0` 之外不测。
8. **正数/非数的硬报错只做了代码路径审查**（`_scan_band_min_value`），**没有**单独跑一次
   `scan_band_min:=0.1` 的 launch 去截图报错。
9. **并发**：19:46 时另一套栈的 11737 与我的栈并存（`load 10.4`、RTF 一度 0.338）；我**没有**杀它，
   只保证自己批次内串行 + 每跑前后 `gz_left=0`。机器级"没有第二套栈"这条在本会话的 PID 命名空间里
   **看不到**（同 §25.11#7 的限制）；19:48 之后并行任务暂停，干净补充跑的 `ss` 只剩我自己的 master。
10. **`scan_band_min` 也作用于 `off` 臂**（同一份 local 配置）⇒ off 的 `below` 从 433 降到 420–435
    （≈4 条/帧进带），但 in-band 仍 43.7–45.4%、kept-y 仍单侧、WARN 仍触发 ⇒ 不构成"救活"。
11. **回滚只证明了参数路径**（`0.0` ⇒ 合并文件逐字节 = 改动前、live 回读 0.0），**没有**在回滚档
    跑满 3 跑做 A/B；单跑对照见 26.7#3。
12. **§27 的资产提升未提交**（工作树里是并行任务的未提交改动）。本节的提交**只含**本节代码/文档，
    **不包含** map/PCD 资产，也不包含 §27 正文。

### 26.11 一句话给用户

> **局部 2D scan 的下界现在是 −0.05 m（`scan_band_min`，默认生效、一条参数回滚 0.0）**：
> `hybrid` 默认档仍 **771/771（100%）在带内**，最低端点 **+0.014…+0.022 m** ⇒ 距新下界
> **63–72 mm**（§25 的 7 mm 余量 ×8），`h_meas` **0.2616 INFO**、倾角 **0.466°**、
> local lethal/free **277/40389**（§25 是 329/40106.5，差在逐跑散布内，且**宽带没有多放一条
> 负 z 波束**）。**`off` 仍 43.7%（navband [0,2]）/ 45.4%（live 带）、kept-y 单侧 +y、WARN** ——
> 5 cm 是鲁棒性余量，**救不了**斜装。**STVL 的 0.0/2.0、`global_obstacle:=scan` 的 §15 −1.0、
> `max 2.0` 一个字节没动**；合并文件只在 local 段多 2 行，`global_costmap` 子树逐字节相同，
> **回滚时整文件逐字节回到 `e9b39f36…`**。**诚实边界**：批次中途并行任务换了默认 RMUL 资产
> （局部数字不受影响、全局列按资产分列，新图全局基线 2163/12260/12812），n 小，`off` 不是产品路径。

## §27 第十七轮（**本机 date=2026-10-20T19:39–20:17**）：给 §24 的 **v6 资产做一次刚体后处理** —— §24 的"`dyaw −2.0°` 一次性摆放错"其实是**峰估计器的 z 窗假象**（真值只有 **+0.141 m 的纯平移**），搬完之后**四门全过** ⇒ **RMUL 提升**（2D+3D 一起换）；RMUC 不动（§24 已判死：覆盖率 21.6%）

> **本节只新增**（append-only）：§1–§25 一个字都没改。写作时 **§26 已被并行的"缺陷 b"（局部 2D-scan 带 −0.05）任务占用**（HEAD `dfd9810` 的 launch 注释就写着"缺陷 b；§26"）⇒ 按父任务裁定，本节用 **§27**。
>
> 任务口径（父任务原文）：缺陷 **a** —— §24 的 RMUL2026 v6 图"差两道门"：覆盖率 102.3%、gicp 420/420 采纳、
> 创新 p50 3.4 mm、score 0.00322、自洽 OK，但**墙面 raw 残差 p50 0.155 m**（旧 0.0437）、**地面 −0.047 m**
> （判据 ±6 mm）。§24.6 的诊断是"形状好（`fullfit` p50 0.068）+ **一次性整体摆放错**（`dyaw ~−2.0°`、
> `t_fit (0.134,0.156)`）+ 一个地面 z 偏移" ⇒ 本轮的活就是：**只做一步刚体后处理**、**重跑四道门**、
> **四门全过才提升 RMUL**（RMUC 保持旧图 —— 它的重建被"FAST-LIO 窄走廊退化"物理挡住，§24.10#1）。
>
> 隔离：与 §23/§24 同口径（私有 `HOME` / `ROS_DOMAIN_ID=137` / `GAZEBO_MASTER_URI=http://127.0.0.1:11737`、
> `gui:=False`、`world:=X` 不带前导 `--`、`setsid` + `timeout` + `tools/scripts/lib/scoped_cleanup.sh`、
> **无全机 pkill**；跑前查 `free`/`swap` 与 `ss -ltn`）。原始数据与可复跑脚本在 `.tmp_align/`（gitignored）：
> `fit_faces.py`（面级峰残差 + 面级 LS 刚体）、`floor_fit.py`（**真值高度图分区**的地面/平台锚点）、
> `fit_dt.py`（第二种解算：真值竖直面栅格 + 距离变换）、`apply_tf.py`（PCD 逐点 / 2D 最近邻重栅格化）。
> 复用 §23/§24 的同一把尺子：`wall_resid.py` / `floor_pcd.py` / `map_cov_stats.py` / `map_odom_probe.py` /
> `map_asset_guard.py` / `run_nav_gate_v3.sh`。

### 27.1 一句话结论

> **RMUL2026 提升**：把 §24 的 `PCD/RMUL2026_v6.pcd` + `map/RMUL2026_v6_spl.{pgm,yaml}` 施加**一次刚体**
> **`dyaw = 0.000°`、`t = (+0.1408, −0.0147, +0.0461) m`**（map 系内、绕 map 原点；**只有平移，没有旋转**），
> 四道门**全过**：① 已知 **96.95 m² = 旧图 102.3%**、旧已知格 **98.75%** 被覆盖（缺区 1.19 m²，其中
> **405 格是旧图自己那道"画到 world x≈2.40"的西墙**）；② 墙面 **raw p50 0.01392 m**（旧默认 0.04365，
> **同一把尺子**）、地面**世界系 −0.0006 m**（判据 ±6 mm）；③ GICP **420/420 采纳**、创新 p50 **3.27 mm**、
> score **0.00314 m²**、`map→odom` **`(−0.026,+0.184,+0.325) m / +0.28°`**（x 从 −0.151 掉到 −0.026；
> **剩下的 +0.184 m 是 `robot11_mount:=sensor` 的安装横向偏置 `+0.1919`，不是图的错**，见 §27.6）；
> ④ `map_asset_guard` 对 `RMUL2026_v6`（posegraph/PCD）与新写的 `RMUL2026_v6al` 全判 **resume**、
> `health_warning: false`。⇒ **装 `map/RMUL2026.{pgm,yaml}` + `PCD/RMUL2026.pcd`，三个文件都有
> `.bak-20261020-promote` 备份（md5 与提升前逐字节相同）**。RMUC2026 **一个字节没动**。
>
> **顺带纠正 §24.6 的一条诊断**：`dyaw −2.0°` **不存在**。真值（STL）里 RMUL 的**竖直面全部只有 0.40 m 高**
> （`z∈[0,0.40]`），而 §24 的墙面证人用的是 `zrange 0.15,1.6` —— 这个窗口把 **0.4 m 以上、离墙面 0.20 m
> 的"高 z 抹影"** 也当成墙面点了（该跑 `map→odom` 的 z 极差 **0.449 m**，§24.5）。把窗口收回到墙的真实
> 高度后，**四个周界面的原始残差 = −0.1406 / +0.1410 / +0.0152 / −0.0141**，互相一致到 **0.5 mm**，
> 最优刚体就是**纯平移**；独立用**墙线斜率**量也给出 **|yaw| ≤ 0.35°**（不是 2°）。

### 27.2 口径与起点（**只动后处理，不动流水线**）

* 输入 = §24 的 v6 资产，**逐字节**：`PCD/RMUL2026_v6.pcd`（27,687 点，`md5 580e6f40…`）、
  `map/RMUL2026_v6_spl.{pgm,yaml}`（`241x162 @0.05`、`origin [-1.95,-2.90]`）。**不重跑建图**。
* 变换形式写死为 **map 系内一步刚体 + 一个 z 平移**：`p' = R(dyaw)·p + (dx, dy, dz)`，`R` 绕 **map 原点**
  （= 出生点）。理由与 §24.5 相同：云内部在 `odom` 里是**刚性**的（LIO 的 `odom→livox` 平滑），
  所以"整块放哪"永远可以只用一步刚体表达；不能表达的是**逐时间的漂移**（那是形状残差，见 §27.4）。
* 判据（§23/§24 逐字）：① `known ≥ 旧 94.77 m²` 且旧已知格被覆盖；② 墙面 **raw** p50 ≤ 旧 **0.04365**、
  地面世界 z ∈ **±0.006 m**；③ GICP `map→odom` 常量偏置"小"（与旧默认 `(−0.057,+0.130,+0.016)/−0.16°`
  同量级）且创新/score 不退化；④ `map_asset_guard check` = `resume`、PCD sidecar `health_warning: false`。

### 27.3 怎么拟合的（三步 + 两条独立交叉校验）

**尺子**：`wall_resid.py` / `map_wall_probe.mesh_faces()` —— 取 STL 里**从出生点可见**的竖直面，
资产点落在面法向 ±0.45 m 窗、面切向跨度内，**1D 直方图定峰 + ±2 cm 中位精修**，
`残差 = 资产面 − 网格面`（沿法向）。本轮新增 `.tmp_align/fit_faces.py`：**同一把峰尺**，但
（a）可以指定**面子集**，（b）把"扫 dyaw"的目标函数写成**子集内 |残差| 的 RMS**，
（c）对每个 dyaw 用**面级解析最优平移** `min_t Σ(r_i + n_i·t)²`。

**第一步：先收 z 窗（这是本轮最关键的一步）**。真值里 RMUL 的 6 个内墙面**全部 `z∈[0,0.40]`**
（`map_wall_probe.py faces --world RMUL2026`：`x=2.5/x=14.5/y=0.5/y=8.5/x=5.0/x=10.0` 都写着
`z[-0.00,0.40]`），所以 `zrange 0.15,1.6` 里 **0.40 m 以上的点根本不是墙面点**：

| 面 | `z∈[0.15,0.60)` 峰残差 | `z∈[0.15,1.60)` 峰残差 | 变化 |
|---|---|---|---|
| `x=2.50`（n=+x） | **−0.1406** | −0.1406 | 0（该窗 z>0.6 处 **0 点**） |
| `x=14.50`（n=−x） | **+0.1410** | **+0.1943** | **+0.053**（该窗 z>0.6 有 **1401 点**，峰在 +0.20） |
| `y=0.50`（n=+y） | **+0.0152** | +0.0214 | +0.006 |
| `y=8.50`（n=−y） | **−0.0141** | −0.0211 | −0.007 |

⇒ 宽窗把东墙的峰**拉走 5.3 cm**（那 1401 个"高 z"点离墙面 0.20 m，是**该跑 z 漂移 0.449 m 把墙抹上去
的影**，见 §27.4）；四个面一旦不再互相一致，`dyaw` 就被拟合器拿去"解释"这个不一致 ⇒ §24 的 −2.0°。

**第二步：四个周界面做面级 LS**（`fit_faces.py --subset ABCD --zrange 0.15,0.6`）：

* `dyaw = 0.000°`（扫 0.05° 网格，RMS 面在 ±0.1° 内是平的）、`t = (+0.1408, −0.0147) m`、
  拟合后逐面残差 **+0.0002 / +0.0002 / +0.0005 / +0.0005 m**（RMS **0.0004**）。
* **交叉校验 ①（加一面内部墙）**：子集 `{ABCD, x=10.0}` ⇒ `dyaw +0.10°`、`t (+0.1477, −0.0223)`、
  6 面残差全 ≤ **0.0085 m** ⇒ 与 ABCD 解差 **<1 cm / 0.1°**。
* **交叉校验 ②（墙线斜率，完全不看残差峰）**：对每个面按切向分箱取中位中心、再拟合直线
  （`x=2.5 −0.099°`、`x=14.5 −0.225°`、`y=0.5 +0.352°`、`y=8.5 +0.180°`、`x=5 −0.348°`、`x=10 +0.306°`）；
  在"整图有一个刚体 yaw θ"的模型下，x 法向面给 `θ = +0.10/+0.23°`、y 法向面给 `θ = +0.35/+0.18°`
  ⇒ **|θ| ≤ 0.35°**，而且**同号**（≈ +0.2°，在估计器噪声内）⇒ **不是 −2°**。
* **失败的交叉校验（诚实记录）**：`.tmp_align/fit_dt.py` 用"真值竖直面栅格 + 距离变换"做全结构对齐，
  数值最优给 `dyaw −0.33°、t (+0.04,−0.01)`，但**不采用**：STL 的 RMUL 周界墙是 **0.3 m 厚**（内面 2.5 /
  外面 2.2 都在竖直面集合里），栅格化后是 0.32 m 厚的带 ⇒ 该代价函数对 cm 级平移**不敏感**（对齐前后
  `p50 距离` 都是 0.000）。⇒ 记"该法在本世界上不可用"，不作证据。

**第三步：z（地面）**。§24 用的是 `floor_pcd.py` 的"全局峰 / 离出生点 1.5–3 m 环带"，而 **RMUL 的真值里
除了 `z=0` 的地面，还有 +0.200 m 的抬升平台（25.5 m²）与 0.38–0.42 m 的墙顶（≈47 m²）** ——
"全局峰"必然是平台顶（§24.6 已记）。本轮改用 `.tmp_align/floor_fit.py`：**先按真值高度图分区**
（`build_truth_map.py` 的 `.rel.npy`：ground `rel<0.05` / platform `0.15<rel<0.25`），
再在每个区里对**水平点**（局部法向 `|n_z|>0.9`，k=12 邻域 PCA）取 z 峰：

| 区 | 水平点 | 资产峰 z（**对齐后**） | 真值 | 差 |
|---|---|---|---|---|
| **地面**（rel<0.05） | 12,044 | **−0.0135**（对齐前 **−0.0590**） | 0.000 | → world **−0.0006 m** |
| 抬升平台（0.15<rel<0.25） | 5,207 | **+0.2292**（对齐前 +0.1842） | +0.200 | **+0.0292 m** ❌ |
| 墙顶（0.30<rel<0.48） | 164 | 峰不成立（样本太少） | +0.400 | — |

* **`dz` 怎么定的**：`dz = 0 − (地面在 map 系的 −0.0590) − (world←map 的 z = +0.0129) = +0.0461 m`。
  其中 `world←map z = +0.0129` 来自**同一跑**的实测（`align.json` 的 `t_wm_med z = 0.28538` 减
  `map→odom` trace 末值 `0.27247`，§24.6 的同一个数）。⇒ 地面在 **world 系 −0.0006 m**（判据 ±6 mm ✔）。
* ⚠️ **诚实的形状残差**：`地面 ↔ 平台` 的**间距**在资产里是 **0.2432 m**、真值是 **0.2000 m**
  ⇒ **+43 mm 的竖直形状误差**（同一跑 `map→odom` z 极差 0.449 m 的残差，§24.5）。
  **一个 dz 不可能同时把地面与平台都归零**；本轮按判据（判据写的是"地面"）锚在**地面**，
  平台因此高出 **+29 mm**（world +0.229 vs 真值 +0.200）。这一条进 §27.11。

### 27.4 拟合结果 + 逐面残差（before → after，**同一把尺子**）

**施加的刚体（最终值，写进 sidecar）**：`dyaw = 0.000°`、`t = (+0.1408, −0.0147, +0.0461) m`
（`map` 系、绕 map 原点；`dz` 见 §27.3 第三步）。

**3D：`PCD/RMUL2026_v6.pcd` → `RMUL2026_v6al.pcd`（`wall_resid.py pcd`，`zrange 0.15,0.6`）**

| 面（world） | 法向 | 点数 | **raw 残差 before** | **raw 残差 after** | 判读 |
|---|---|---|---|---|---|
| `x=2.50` | +x | 712 | −0.1406 | **+0.0021** | ✅ 进拟合 |
| `x=14.50` | −x | 64 | +0.1410 | **−0.0020** | ✅ 进拟合 |
| `y=0.50` | +y | 746 | +0.0152 | **−0.0010** | ✅ 进拟合 |
| `y=8.50` | −y | 1046 | −0.0141 | **+0.0031** | ✅ 进拟合 |
| `x=10.00` | −x | 881 | +0.1544 | **+0.0139** | ✅ 交叉校验（未进拟合） |
| `x=5.00` | −x | 1386 | −0.1799 | **−0.1224** | ⚠ **抹影面**：横向散布 0.5 m、沿 0.1 m 空腔成串（体素格点混叠），峰在不同 z 窗下在 −0.18/+0.02 之间跳 ⇒ **不进拟合** |
| `x=2.22` | +x | 718 | +0.1394 | +0.1696 | ❌ **假读数**：对齐后正好读成它与真墙的 **0.28 m** 间距（§13.4 的"相邻面进窗"） |
| `y=0.22` | +y | 577 | +0.2438 | +0.2359 | ❌ 同上（0.28 m 假读数） |
| `y=8.78` | −y | 663 | +0.2391 | +0.2433 | ❌ 同上 |
| **9 面 \|残差\| p50 / max / rms** | | | **0.1424 / 0.2438 / 0.1694** | **0.01392 / 0.2433 / 0.1328** | 判据 ≤ **0.04365** ✅ |
| （对照）§24 的宽窗 `zrange 0.15,1.6` | | | 0.15516 / 0.24378 / 0.16686 | — | §24.6 原值，**本轮原样复现** |
| （新口径）4 个可靠周界面 | | | 0.1406 / 0.1410 / 0.0152 / 0.0141 | **0.0002/0.0002/0.0005/0.0005** | 一致到 0.5 mm |

> 三个"假读数面"在**对齐之后**分别读 **+0.170 / +0.236 / +0.243** —— 恰好是它们到真墙的几何间距
> （0.28 / 0.28 / 0.28 m 减去量化）。这是"它们量的是隔壁那道墙"的**判决性证据**，也说明
> **9 面 p50 里那 3 个值不该被当成图的误差**；判据用的 p50 对这 3 个值不敏感（稳健统计），
> 但**逐面表必须逐个标注**。

**2D：`RMUL2026_v6_spl` → `RMUL2026_v6al_spl`（`wall_resid.py grid`，占用格中心）**

| 面 | 旧默认 `map/RMUL2026.yaml` | v6（未处理） | **v6al（处理后）** |
|---|---|---|---|
| `x=2.50` | −0.075 | −0.100 | **+0.025** |
| `x=14.50` | **+0.225** | +0.125 | **−0.025** |
| `y=0.50` | +0.075 | +0.025 | **+0.025** |
| `y=8.50` | +0.075 | −0.025 | **−0.025** |
| `x=5.00` | +0.175 | +0.075 | **−0.075** |
| `x=10.00` | +0.225 | +0.075 | **−0.075** |
| `x=2.22` | +0.205 | +0.180 | （该面未成立） |
| **\|残差\| p50 / max** | **0.1750 / 0.2250** | 0.0750 / 0.1800 | **0.0250 / 0.0750** |

⇒ **2D 不是"本来就好的"**（§23 说的 0.021 m 那条是**旧 RMUL 的 3D PCD**，不是这张栅格图；
这张图的四周界自己就差 **0.10–0.13 m**、`x=14.5` 那道旧默认图差 **0.225 m**）
⇒ **决定：2D 与 3D 都用同一个刚体重栅格化/搬运**（两张资产同属 `map` 系，必须互相一致；
若只搬 PCD，导航用的栅格墙会与 GICP 摆出来的位姿差 0.14 m）。处理后四个周界面全部 **±0.025 m = 半格**。

**2D 重栅格化的做法**（`.tmp_align/apply_tf.py grid`）：**destination-driven 最近邻** —— 对输出每个格中心
做逆变换到源格取值，所以**不会出现空洞**；输出窗口 = 源窗口四角变换后的 bbox（对齐到 0.05 格）
⇒ `241x162, origin [-1.95,-2.90]` → **`242x163, origin [-1.85,-2.95]`**；
`resolution/mode/negate/thresholds` 一个字节没动；占用/自由格数 `1970/36809` **逐格不变**
（纯平移不改变内容，只改变"放哪"），未知格 263 → 667（多出来的 404 格是新窗口的边界外）。

**2D 的实际平移量是量化的**：`t=(+0.1408,−0.0147)` 落在 0.05 格上 = **+0.15 / −0.05 m**（3 格 / 1 格），
所以 2D 与 3D 的内容差 ≤ **1 cm / 1 格** —— 这是"用同一份刚体、但栅格只能整格搬"的固有量化，
写进 §27.11。

### 27.5 施加的变换与落盘（可复跑）

```bash
# ① 拟合（离线，无 ROS）
python3 .tmp_align/fit_faces.py --pcd src/rm_nav_bringup/PCD/RMUL2026_v6.pcd \
        --subset ABCD --zrange 0.15,0.6 --json .tmp_align/ff_ABCD.json
python3 .tmp_align/floor_fit.py  --pcd src/rm_nav_bringup/PCD/RMUL2026_v6.pcd \
        --tf .tmp_align/tf_abcd.json --json .tmp_align/floor_v6.json     # 对齐前
# ② 施加（PCD 逐点；2D 最近邻重栅格化）
python3 .tmp_align/apply_tf.py pcd  --in src/rm_nav_bringup/PCD/RMUL2026_v6.pcd \
        --out .tmp_align/RMUL2026_v6al.pcd --dyaw 0.0 --dx 0.1408 --dy -0.0147 --dz 0.0461
python3 .tmp_align/apply_tf.py grid --in .tmp_map2/rmul_v6/RMUL2026_v6_spl.yaml \
        --out .tmp_align/RMUL2026_v6al_spl.yaml --dyaw 0.0 --dx 0.1408 --dy -0.0147
# ③ 复测
python3 .tmp_map2/wall_resid.py pcd  --world RMUL2026 --pcd .tmp_align/RMUL2026_v6al.pcd \
        --spawn 4.3 3.35 --zrange 0.15,0.6
python3 .tmp_map2/wall_resid.py grid --world RMUL2026 --map-yaml .tmp_align/RMUL2026_v6al_spl.yaml \
        --spawn 4.3 3.35
python3 .tmp_align/floor_fit.py --pcd .tmp_align/RMUL2026_v6al.pcd
python3 .tmp_map2/map_cov_stats.py --a src/rm_nav_bringup/map/RMUL2026.yaml \
        --b .tmp_align/RMUL2026_v6al_spl.yaml --spawn 4.3 3.35
```

**装到哪（就三条 `cp -f`，见 §27.7）**：`PCD/RMUL2026.pcd` ← `RMUL2026_v6al.pcd`、
`map/RMUL2026.pgm` ← `RMUL2026_v6al_spl.pgm`、`map/RMUL2026.yaml`（`image: RMUL2026.pgm`、
`origin [-1.85,-2.95,0]`）。候选件以 `PCD/RMUL2026_v6al.pcd`（+ sidecar）与
`map/RMUL2026_v6al_spl.{pgm,yaml}` 留在仓里作凭据。

### 27.6 四道门（逐条：数字、旧 vs 新）

| 门 | 旧默认 | §24 的 v6（未处理） | **v6al（处理后）** | 判据 | 判 |
|---|---|---|---|---|---|
| ① 已知面积 | **94.77 m²** | 96.948（102.3%） | **96.948（102.3%）** | ≥ 旧 | ✅ |
| ① 旧已知格被覆盖 | — | 99.9%（缺 0.10 m²） | **98.75%（缺 1.19 m²）** | 无缺区 | ⚠（见下） |
| ① 占用格数 / 厚度 mean | 3713 / 0.0821 m | 1970 / 0.0589 | **1970 / 0.0589** | — | — |
| ② 墙面 raw p50（9 面） | **0.04365**（只能量 2 面） | 0.1424（宽窗口径 0.15516） | **0.01392** | ≤ 旧 | ✅ |
| ② 墙面 raw p50（4 可靠周界面） | — | 0.1406 | **0.0004**（max 0.0005） | — | ✅ |
| ② 地面（world） | −0.146（工具判不可用） | **−0.0470** | **−0.0006 m** | ±0.006 | ✅ |
| ③ GICP 采纳 | 100% | 420/420 | **420/420（拒绝 0、越界 0）** | 100% | ✅ |
| ③ 创新 p50 / max | 10.65 / 74.29 mm | 3.39 / 17.95 mm | **3.27 / 11.31 mm** | ≪300 mm | ✅ |
| ③ score p50 | 0.0051 m² | 0.00322 | **0.00314 m²** | ≈旧或更好 | ✅ |
| ③ `map→odom` | `(−0.057,+0.130,+0.016)/−0.16°` | `(−0.151,+0.180,+0.260)/+0.2°` | **`(−0.026,+0.184,+0.325)/+0.28°`** | "小" | ✅（xy 0.186 vs 旧 0.142，见下） |
| ④ guard（posegraph / PCD） | fresh | resume / resume | **resume / resume**，`RMUL2026_v6al` 也 resume | resume | ✅ |
| ④ sidecar `health_warning` | — | false | **false** | false | ✅ |

**门①的 1.19 m² 缺区是什么**（逐格定位过）：**475 格里 400 格是一列 3 格宽、位于 `map x≈−1.9`
（world `x≈2.40`）的** —— 那是**旧图自己那道西墙**（旧 2D 图的 `x=2.5` 面读 **−0.075**，它把墙画到了
world 2.42）。我们把新图**按真值摆正**以后，旧墙那一列落在新图的"已知区"之外 ⇒ 记成缺区。
**它不是"新图少了一块"**：新图那一片是**场外**（真值西墙在 world 2.50）。另有 70 格是旧新两张图本来
就不同的自由区（对齐前就有 39 格）。⇒ 门①判 **过**，但把"旧已知格覆盖 99.9% → 98.75%"如实记账。

**门③的 y = +0.184 m 是什么**（**这不是图的错**，必须写死在这里）：`robot11` + `robot11_mount:=sensor`
的 LiDAR 在 `base_link` 系里的安装偏置是 `(0.000562, 0.130916, 0.157028)`，30° 斜装后**在世界系**是
`Δ = (0.0006, +0.1919, +0.0705) m`（§13.4 的实测/预测对着 ±1.6 mm 的那一组）。GICP 报的 `map→odom`
是**odom 原点（= LIO 初始化时刻的 IMU/雷达位）在 map 里的位置**，所以它天然含这个横向偏置：

* 预测 `y = +0.1919`，实测 **+0.184**（差 8 mm）；
* 把 `dy = −0.0147` 加上去之后 `y` **没有**跟着动（0.180 → 0.184）⇒ 它对**资产**的 y 不敏感，只对量具敏感；
* x 分量则**跟着资产走**：−0.151 → −0.026（= +0.1408 的平移落在 x 上）。
  ⇒ **对齐后剩下的 xy 残差是"传感器安装口径"，不是图的摆放误差**；旧默认的 `(−0.057,+0.130)` 同理
  （它 = 同一个安装偏置 + 旧 PCD 自己的 −0.04/+0.05 摆放误差）。z 分量 `+0.325` = 本轮 `dz +0.046`
  + 同一跑里"odom 的 z 原点在传感器高度"的老口径（§24.7 已把 z 归为口径）。
  **§27.8 又给了一个跨臂旁证**：**同一张提升后的图**，换成 `hybrid + small_point_lio` 链，
  `map→odom` 的 y 变成 **+0.014**、z 变成 **+0.081** ⇒ 那 +0.184/+0.325 **只跟 odom 的口径有关**。

### 27.7 提升决定：**RMUL2026 提升**（2D + 3D 一起换），RMUC2026 **不动**

| 世界 | 门① | 门② | 门③ | 门④ | 决定 |
|---|---|---|---|---|---|
| **RMUL2026** | ✅ 102.3% / 98.75% | ✅ 墙 **0.01392**（旧 0.04365）、地面 **−0.0006 m** | ✅ 420/420、创新 **3.27 mm**、`map→odom` x **−0.026** | ✅ resume / `health_warning: false` | **提升** |
| RMUC2026 | ❌ 84.19/389.92 = 21.6%（§24） | 无 3D 资产可量（§24：`serialize_map` 超时） | 无资产 | 无资产 | **不动**（§24.10#1 的物理阻塞没解） |

**备份（就地、先备份、不删；`*.bak-<日期>-promote` 这一代是本轮新增，§23/§24 的 `.bak-20261019/20`
原样保留）**：

```bash
src/rm_nav_bringup/map/RMUL2026.pgm.bak-20261020-promote     # 6e45a9327e99d57883881f7a7d10c3d0（= 提升前，逐字节）
src/rm_nav_bringup/map/RMUL2026.yaml.bak-20261020-promote    # fd23dd82c0aa63b6c2fdf3644c7c9253
src/rm_nav_bringup/PCD/RMUL2026.pcd.bak-20261020-promote     # 5d0d33eb5219d9416606a7b0f5e34028
# 另有同 md5 的更早备份：*.bak-20261019（三件）、PCD/RMUL2026.pcd.bak-20261020b
```

**提升后（当前默认对，三个文件都要在）**：

| 槽位 | 文件 | md5 | 尺寸 / 内容 |
|---|---|---|---|
| 2D 先验 | `map/RMUL2026.pgm` | **7ec12dc323ae18f92df765f288f8f0e1** | `242x163 @0.05`，occ 1970 / free 36809 / unknown 667 |
| 2D 先验 | `map/RMUL2026.yaml` | **bf3f37cb40a1b0535bc934d6c29186e2** | `image: RMUL2026.pgm`、`origin [-1.85,-2.95,0]`、`trinary/0.05/0/0.65/0.25` |
| 3D 先验 | `PCD/RMUL2026.pcd` | **1d5ca30e9d0268d971306f4f497cfb4f** | 27,687 点 / 886,233 B（8 字段 binary，与旧 `RMUL2026.pcd` **同构**） |

* **候选凭据留在仓里**（未删）：`PCD/RMUL2026_v6al.pcd`（同 md5 `1d5ca30e…`）+ **`PCD/RMUL2026_v6al.meta.yaml`**
  （sidecar：`frames.map: map`、`world/spawn/map_start_pose` 齐、`health_warning: false`，并新增
  `post_transform_align` 一整块记录本轮施加的刚体、墙面证人的 before/after、地面/平台的两个锚点）
  + `map/RMUL2026_v6al_spl.{pgm,yaml}`（`7ec12dc3…` / `501b454b…`）。原 `*_v6.*` 一个字节没动。
* `map/RMUL2026.yaml` 的 `image:` 指回 **`RMUL2026.pgm`**（`docs/map_assets.md` §4.2 的约定：默认对的 yaml
  必须自洽，不能依赖候选文件活着）。`yaml` 的 md5 因此与候选 `501b454b…` 不同、而**指向的内容相同**。
* **回滚（一条命令）**：
  ```bash
  cd src/rm_nav_bringup && D=20261020-promote && \
    cp -f map/RMUL2026.pgm.bak-$D map/RMUL2026.pgm && \
    cp -f map/RMUL2026.yaml.bak-$D map/RMUL2026.yaml && \
    cp -f PCD/RMUL2026.pcd.bak-$D  PCD/RMUL2026.pcd && \
    md5sum map/RMUL2026.pgm map/RMUL2026.yaml PCD/RMUL2026.pcd   # 应为 6e45a932… / fd23dd82… / 5d0d33eb…
  ```
* ⚠ **后续要补的一处文档维护**：`docs/map_assets.md` §2.1 里 `RMUL2026.pgm|yaml` 仍然写着
  "43092 B / 2026-09-23 / 其它 world 的资产 … 未在本次范围内重测 ⇒ 保留" —— 提升后已不成立
  （现在是 `39461 B / 242x163 / origin [-1.85,-2.95]`）。本轮**没有改那份文档**（父任务口径：
  只追加本节 + 换资产），列在这里以免下次整理时漏掉。
* `install/` 是**逐文件软链**（§5），三处都是 `cp -f` **就地覆盖** ⇒ 不用重编、软链不悬空；
  本轮实跑已证明生效：`map_server` 读的是 `242 X 163`（新的），`gicp` 读的 `PCD/RMUL2026.pcd`
  = `md5 1d5ca30e`（新 PCD，见 §27.8 的跑头）。

### 27.8 提升后的 nav 冒烟（RMUL2026，`hybrid` 默认档，`band_guard:=True`）—— **通过**

**跑法**（一条 bash 调用里起栈 + 采证据 + 收尾；脚本 `.tmp_align/nav_smoke_v27.sh`，隔离与 §23/§24/§25 同口径）：

```bash
bash .tmp_align/nav_smoke_v27.sh smoke_v27_rmul
#   内部：ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUL2026 mode:=nav \
#     localization:=gicp lio:=small_point_lio ground:=linefit spin_speed:=0.0 \
#     robot:=robot11 robot11_mount:=sensor band_guard:=True nav_rviz:=False lio_rviz:=False gui:=False
#   （gravity_align_mode 默认 auto ⇒ lio=small_point_lio ⇒ 有效值 = hybrid）
#   证据：跑头 md5 / local_obstacle_probe.py（两图环带真障碍格）/ band_guard 的 live 回读与 OK 行 /
#         h_meas / map_odom_probe.py / map_server 读的图尺寸
#   ⚠ 产物落盘在 .tmp_map2/<tag>/（脚本里 env.sh 会把 OUT 改到 .tmp_map2/<tag>，HOME 也在那下面），
#     不是 .tmp_align/ —— 本节的每个数字都能在 .tmp_map2/{smoke_v27_rmul,ovr_with,ovr_without}/ 里查到。
```

**跑头（逐字，这一跑 = 本节的量具快照）**：

```text
date=2026-10-20T20:11:11+08:00   git_head=8d35350（= §26 的局部带 −0.05 已在内）
pcd=1d5ca30e…  pgm=7ec12dc3…  yaml=bf3f37cb…            ← 提升后的三个默认文件
launch_md5=69316fe388adc7adbb2fffd374003992              ← 8d35350 之后（含 scan_band_min）
livox_so_md5=2281ab55…  cfg_base=e9870542…  cfg_global_scan=e81d0002…  cfg_local_band=25d9a836…
[map_server] Read map …/map/RMUL2026.pgm: 242 X 163 map @ 0.05 m/cell
```

**五条判据**：

| 判据 | 实测 | 判 |
|---|---|---|
| **local costmap 非空** | `local_obstacle_probe.py` 49 个采样：机器人 **1 m 内 lethal 格 = 26.78 [24, 30]**（全在 0.5–1.0 m 环带；0–0.5 m 内 0 格 = 开阔地，正常），最近 lethal **0.716 m [0.713, 0.721]**，车后 0.3–1.5 m 扇区 **0** 格 | ✅ |
| **`h_meas` INFO** | `h_urdf=0.2595 m h_meas=0.2616 m |Δ|=0.0020 m < 0.03 ⇒ INFO`（inliers 14432、tilt 0.06°、resid p50 1.7 mm） | ✅ |
| **`band_guard` OK，带 = `[-0.05, 2.0]`** | 7 个样本 **全部 100%**：`772/772, 782/782, 781/781, 769/769, 763/763, 771/771, 774/774`，`below=0 above=0`，`kept-y` 跨度 **4.04–6.06 m**、`one-sided=no`（不是半平面盲区），`band=[-0.050,2.000] source=[-0.050,2.000] layer=[-0.050,2.000]` | ✅ |
| **GICP 健康** | 整跑 **327/327 帧采纳**、拒绝 0、越界 0，score 最新 0.00315 m²，创新 0.0040 m / 0.069° | ✅ |
| **`map→odom` 常量偏置** | **`(−0.0118, +0.0138, +0.0812) m / +0.90°`**（n=2160，span 3.0 / 5.8 / 1.4 mm、0.08°） | ✅ |

**这一跑还给了 §27.6 那条"y 是量具口径"一个决定性旁证**：同一个提升后的图，**换成
`hybrid` + `small_point_lio`** 这条链，`map→odom` 的 **y = +0.014 m、z = +0.081 m**；
而 `fastlio`（`gravity_align=off`）那条链是同图的 **y = +0.184、z = +0.325**（§27.6）。
**同一个资产、两张不同的 odom 口径** ⇒ 那 +0.184/+0.325 **只跟量具（odom 原点在斜装雷达/IMU 上）有关**，
与图的摆放无关。（hybrid 档 odom 的 z 原点 = 初始化时刻的地面，§10，所以 z 只剩 +0.081。）

**诚实注**：三跑都出现 `spawn_entity: Spawn service failed. Exiting.`
（`Entity pushed to spawn queue, but spawn service timed out waiting for entity to appear … [robot]`）。
**它是启动时序的假报警**：同一跑里 `/scan` 有 763–784 条波束、`h_meas` INFO、GICP 327–328 帧全采纳、
`map→odom` 稳定 ⇒ 机器人**确实在仿真里**（与 §23.2 的 `gzclient` 无头退出同类）。列进 §27.10#13。

### 27.9 全局 `−1.0` 覆盖（§15）还需要吗？—— **判决：不再必需，但仍保留（KEEP）**

**为什么这次要重判**：§15 的根因是"**扫描面在 map 帧的 z = −0.179 m**，被 nav2 默认下界 `0.0` 整盘切掉"，
所以给全局 `obstacle_layer[.scan].min_obstacle_height` 放了 `−1.0`（`global_obstacle:=scan` 覆盖层）。
那个 **−0.179 m 是"旧资产 + 旧跑"的 map 系 z 原点**（旧图的地面根本不在 0，§23.5/§24.6）。
提升后的地图**地面在 map 系 −0.0135 m（world −0.0006）** ⇒ 扫描面**应当**回到 `[0, 2]` 内。
本轮把这件事**直接测了**（单变量 = 那两个键，其余逐字相同，三跑同一套 md5，见 §27.8 跑头）。

**A/B（`global_obstacle:=scan`，`hybrid` 默认档；`localization:=gicp lio:=small_point_lio`）**

```bash
bash .tmp_align/nav_smoke_v27.sh ovr_with    global_obstacle:=scan       # 两个键 = −1.0（默认覆盖层）
...apply_nav_kv.py 把两个键改 0.0（记录 md5 e81d0002… → 66e5a03b…，跑完 cp -f 还原回 e81d0002…）
bash .tmp_align/nav_smoke_v27.sh ovr_without global_obstacle:=scan       # 两个键 = 0.0
cp -f .tmp_align/global_scan_obstacle.yaml.orig src/rm_nav_bringup/config/…
```

| 量（`band_guard` 的 live 回读 + `local_obstacle_probe.py`） | **with `−1.0`** | **without `0.0`** | 差 |
|---|---|---|---|
| 全局层生效带（回读） | `[-1.000, 2.000]` | `[0.000, 2.000]` | 下界差 1 m |
| **全局 in-band** | **100%**：`782/782, 781/781, 769/769, 764/764, 772/772, 774/774` | **100%**：`784/784, 783/783, 766/766, 775/775, 766/766, 768/768, 771/771` | **一模一样** |
| 全局 `below` / `above` | **0 / 0** | **0 / 0** | 无 |
| **扫描面在 map 帧的 z（min / p50 / max）** | **0.009 / 0.119 / 0.422 m** | **0.004…0.012 / 0.119 / 0.418…0.513 m** | 无（同一量） |
| 全局 kept-y 跨度 / 单侧 | 5.67 m / `one-sided=no` | 5.66 m / `one-sided=no` | 无 |
| **全局 lethal（机器人 1 m 内）** | **36.21 [35, 37]** | **36.43 [35, 37]** | +0.6%（n=7 样本内噪声） |
| 全局最近 lethal 距离 | 0.730 m [0.719, 0.733] | 0.723 m [0.717, 0.735] | 无 |
| 局部（对照，逐字不动） | 100%，lethal 26.37 | 100%，lethal 25.76 | 无 |

**判决：保留（KEEP），但理由从"必需"变成"兜底"**：

1. **在提升后的图上 `−1.0` 已经不是必需**：`0.0` 臂的全局 in-band 与 lethal **与 `−1.0` 臂逐样本相同**
   ⇒ §15 的病灶被**换图**治好了（扫描面从 **−0.179 → +0.119 m**）。这是本轮最直接的一条收益：
   以后任何"全局层为什么是空的"的问题，**先看地图的 z 原点**，不要先动 nav2 闸。
2. **但余量只有 4–12 mm**：`0.0` 臂里扫描面 z 的**逐样本最小值 = +0.004…+0.012 m**，
   即"下界 0.0 只剩 4–12 mm 的余量"。对比局部带 `−0.05` 给的 **57 mm**（§26），这是 **5–14 倍更薄**。
   任何一件小事都能把它翻负：`dz` 再错 4 cm（本轮 `dz` 的量级）、换 world/换模型、LIO z 漂移
   （§24.5 实测逐跑极差 0.207–0.449 m）、或车开到场地的低洼处。
   一旦翻负，nav2 **一声不响**地清空全局 scan 层 —— 那正是 §15/§17 的静默失败形态，
   而 `band_guard` 虽然会喊（in-band==0 ⇒ ERROR），但它只是诊断、不修数据。
3. **`−1.0` 无害**：它只放宽下界（`min_obstacle_height` 在 2D 链路上只是"传感器自身高度"的常量闸，
   §25.6），不改变任何其它键、不动 STVL 的 3D 带（`0.0/2.0`）、不动 `max 2.0`；
   且只在 `global_obstacle:=scan` 这一 opt-in 档生效（默认 `stvl` 逐字节不受影响）。
   ⇒ **按父任务口径"拿不准就保留"** ⇒ **保留**。
4. **这条判决与 §25.6 的关系**：§25.6 的结论（KEEP）**仍然有效**，但**理由要改**——
   §25.6 的依据是"扫描面在 −0.179 m"，那在提升后的资产上**已经不成立**；
   新的依据是"只剩 4–12 mm 余量 + 静默失败代价太高"。§25.10 的"哪些旧数字被取代"里应记这一条。

### 27.10 本轮没能验证 / 诚实清单

1. **竖直形状残差没法用一步刚体消掉**：资产里`地面 ↔ 抬升平台`的间距 **0.2432 m**，真值 **0.2000 m**
   （**+43 mm**）。本轮按判据把 `dz` 锚在**地面**（world −0.0006 m ✔），代价是**平台顶面高出 +29 mm**
   （world +0.2292 vs 真值 +0.2000）。来源是该跑 `map→odom` 的 z 极差 **0.449 m**（§24.5）——
   `cloud_accumulator` 在 `odom` 档累积，而 odom 自己在跑里漂。**没有**做"按时间分段的多刚体/弹性校正"。
2. **墙面抹影仍是形状误差的主项**：`x=5.0` 那道内墙上点沿法向摊开 **0.5 m**（0.1 m 间隔的体素格点
   混叠），峰在不同 z 窗下 −0.18/+0.02 之间跳 ⇒ **不进拟合**，对齐后它读 **−0.122 m**。
   `x=10.0` 读 +0.0139（好）。**"9 面 p50" 这件事本身就不该被当成图的精度**（3 个假读数面在里面）。
3. **三个"假读数面"没有从工具里删**（`x=2.22 / y=0.22 / y=8.78`）：它们按 §13.4 是"相邻面进窗"，
   对齐后正好读成 0.28 m 的几何间距 ⇒ 本轮**只标注、不改判据**（改了就没法与 §23/§24 逐位比）。
   以后要做的是把 `wall_resid.py` 的窗从 ±0.45 收到 ±0.12（或按面去重），**本轮没做**（不动既有工具）。
4. **`dyaw = 0.000°` 是最优值不是"证明没有旋转"**：面级 LS 的 RMS 曲面在 ±0.1° 内是平的；
   墙线斜率给 **+0.2° ± 0.15**。也就是说"整图相对真值可能有 **≤0.2°** 的残余偏航"仍未被排除
   （在 10 m 上 = 3.5 cm）。§24 的 −2.0° 则**确定是假象**（宽 z 窗）。
5. **GICP 只测了静止、一个位姿（出生点）、一跑（420 帧）**；没测运动中的跟踪，也没测 `amcl`/`small_gicp`。
   门③的 `map→odom` 只取了 12 仿真秒的中位数（n=2768 样本，spread 4 mm / 0.18°）。
6. **`map→odom` 的 z 分量（+0.325）没有独立真值**：它等于本轮 `dz +0.046` + 该跑 odom 的 z 原点口径
   （§24.7 把 z 归为口径）。**没有**做"把 dz 关掉再跑一次 gicp"的对照（那一跑能直接把 dz 从
   `map→odom` 里减出来）——**这是最该补的一个对照，本轮没做**（时间/机时被并行任务占住）。
7. **2D 重栅格化的量化**：`t=(+0.1408,−0.0147)` 落在 0.05 格上变成 **+0.15/−0.05**（3 格/1 格），
   所以 2D 与 3D 的内容差 ≤ **1 cm / 1 格**。没有做"亚格插值（三线性/超采样）"那一版。
8. **nav 冒烟与全局 −1.0 的 A/B 都跑了，但都只是"静止、出生点、一个位姿、n=1"**（§27.8/§27.9）：
   * **没发导航目标**（只订阅、只采 costmap/probe），所以"能不能规划到目标"**未验证**；
   * `map→odom` 的 z 分量 +0.081（hybrid 臂）里有多少是本轮的 `dz +0.046` **没有独立拆开** ——
     要拆就得再跑一次"dz 关掉"的 gicp 臂，**本轮没做**（这是最该补的一个对照）；
   * 全局 A/B 只覆盖 `global_obstacle:=scan` 这一 opt-in 档（默认 `stvl` 逐字节不受影响），
     且只测了**出生点附近**：场地远端（尤其抬升平台顶面、`x=14` 一侧）的扫描面 z 没逐点量，
     **"4–12 mm 余量在场地其它位置是否更薄"未验证** —— 这也是保留 `−1.0` 的另一个理由。
9. **旧 `PCD/RMUL2026_v6.meta.yaml` 的 `file_sizes` 与文件不符**：sidecar 写 `886233`，磁盘上是
   **740525**（ascii）—— 说明 §24 的 `pcd_odom_to_map.py` 在 `map_archive save` **之后**把 PCD 重写成
   ascii 了。本轮没有改那份 sidecar（只在新 `RMUL2026_v6al.meta.yaml` 里写对了 886233，因为新文件是
   8 字段 binary）。**这是既有的 sidecar 不一致，记账在此。**
10. **RMUC2026 一个字节没动**（§24 已判死）。本轮**没有**再尝试"多会话拼图 / 换 LIO"（§24.9 的建议 1）。
11. **`fit_dt.py` 那种"全结构距离变换"在本世界上不可用**（周界墙 0.3 m 厚 ⇒ 代价函数对 cm 级平移不敏感），
    记录在此，避免以后有人再走一遍。
12. **没有重跑建图**（任务口径本来就不要）：所有结论都建立在 §24 那一次 `rmul_v6` 的产物上；
    它的 ATE 是 0.1379 m、`z_err −0.2207±0.0501`——本轮只搬了整体，没有改那次跑的漂移。
13. **`spawn_entity` 三跑都报 ERROR（假报警）**：`Entity pushed to spawn queue, but spawn service
    timed out waiting for entity to appear … [robot]`。机器人**确实在**（`/scan` 763–784 束、
    `h_meas` INFO、GICP 327–328 帧全采纳、`map→odom` 稳定），是启动时序的服务超时。
    **没有**去查 `spawn_entity.py` 的 `-timeout` 该调多大（与 §23.2 的 `gzclient` 无头退出同类），
    但**它是本轮三跑的公共噪声**，写在这里避免以后拿它当"栈没起来"的证据。
14. **三跑的 `localization:=gicp` 都是 `robot11 + robot11_mount:=sensor + small_point_lio`**，
    与**建图那次（默认模型 + `fastlio`）不是同一台车/同一条链**。所以 §27.8 的 `map→odom`
    （y +0.014 / z +0.081）与 §27.6 的（y +0.184 / z +0.325）**不能逐位比**，只能当"同一张图、
    两种 odom 口径"的旁证用。**没有**跑"默认模型 + fastlio"的 nav 臂。
15. **`band_guard` 的 `/band_guard/status` JSON 本轮没解析成功**（`ros2 topic echo --once` 的
    transient_local 输出被 `data:` 前缀和截断拌在一起 ⇒ `json.loads` 报 `Expecting ',' delimiter`）。
    所有 in-band / 带 / kept-y / scan-z 数字取自 **`band_guard` 的 launch 日志 OK 行**（同一份 live 回读、
    同一批样本，字段逐字相同）⇒ 结论不受影响，但"JSON 通道可用"这条**没验证**；脚本里的解析代码要修。

### 27.11 一句话给用户

> **RMUL2026 换图了（2D+3D 一起），RMUC2026 没动**。做法只有一步：给 §24 的 v6 资产施加**一次刚体**
> **`dyaw = 0.000°`、`t = (+0.1408, −0.0147, +0.0461) m`**。关键发现是 **§24 那个"`dyaw −2.0°` 一次性摆放错"
> 不存在** —— 真值里 RMUL 的墙只有 **0.40 m 高**，而 §24 的墙面证人用 `zrange 0.15,1.6`，把 0.4 m 以上、
> 离墙 0.20 m 的"z 漂移抹影"也当成墙面点了（该跑 `map→odom` z 极差 **0.449 m**）；把窗口收回到墙高之后，
> 四个周界面的原始残差是 **−0.1406 / +0.1410 / +0.0152 / −0.0141**，互相一致到 **0.5 mm**，
> 最优刚体就是**纯平移**。搬完之后**四道门全过**：已知 **96.95 m²（旧图 102.3%）**、墙面 raw p50
> **0.01392 m**（旧 **0.04365**）、地面 **world −0.0006 m**（判据 ±6 mm）、GICP **420/420 采纳 / 创新 p50
> **3.27 mm** / score **0.00314** / `map→odom` **`(−0.026,+0.184,+0.325)/+0.28°`**（x 从 −0.151 掉到 −0.026；
> 剩下的 **+0.184 m 是 `robot11_mount:=sensor` 的安装横向偏置 +0.1919**，**不是图的错**）、
> `map_asset_guard` 全判 **resume**、`health_warning: false`。**备份三件 `.bak-20261020-promote`
> （md5 与提升前逐字节相同）**，一条命令可回滚。**两处诚实缺口**：① 地面↔平台间距比真值多 **43 mm**
> （z 漂移的形状误差，一个 dz 只能锚一个）；② `x=5.0` 那道内墙被抹开 0.5 m、对齐后仍读 −0.122 m，
> 以及 3 个"相邻面进窗"的假读数面只能标注、不能当精度。**提升后的冒烟与全局 `−1.0` 的 A/B 也跑完了**：
> `hybrid` 默认档 **local costmap 非空**（1 m 内 24–30 个 lethal 格、最近 0.72 m）、`h_meas` **INFO**
> （0.2616 / |Δ|0.0020）、`band_guard` **7/7 样本 100% 在 `[−0.05,2.0]`**、`one-sided=no`、
> GICP **327/327**、`map→odom` **`(−0.012,+0.014,+0.081)/+0.90°`**；**全局 `−1.0` 判决 = 保留（KEEP）**，
> 但理由变了：**在提升后的图上 `0.0` 与 `−1.0` 逐样本相同**（扫描面 z p50 从 **−0.179 → +0.119 m**，
> §15 的病灶被换图治好了），**但余量只剩 4–12 mm** ⇒ `−1.0` 从"必需"降级为"兜底"，保留。


---

## §28 第十八轮（**本机 date=2026-10-21T05:50–06:10**）：删除两个冗余 launch 参数 `livox_tilt_axis` / `livox_raise_m` —— 值冻结、默认路径逐字节不变

> **本节只新增**（append-only）：§1–§27 一个字都没改。任务口径：这两个参数都是"**生成模型的输入**"，
> 不是运行时真值 —— 安装 rpy/xyz 自 `f9bf4a9` 起由 `_parsed_livox_mount()` 从**这次真要 spawn 的 URDF**
> 解析（`gravity_align_rpy` / `gravity_align_h` / `lio_tf_adapter` 的 xyz,rpy 全走它）。删掉 launch 开关
> 后，只要**冻结值**与删除前默认路径传的值相同，生成的 `robot_description` 就应逐字节不变。

### 28.1 删了什么、冻结成什么

| 参数 | 原声明 | 冻结值 | 处理 |
|---|---|---|---|
| `livox_tilt_axis` | `default='roll'`，`choices=['roll','pitch']` | **`roll`**；传给 xacro 的 `livox_tilt_rpy` 字符串 = **`-0.523598775598293 0 0`**（SolidWorks CSV 的 Joint Origin Roll；用户 2026-10-07 按实物确认） | launch 里**删除**声明/校验/`_robot_xacro_args` 分支/`_urdf_livox_mount_table`/`_adapter_table`/`_check_gravity_align_contract` 日志/横幅；`_LIVOX_TILT_RPY` 双值字典 → 单值 `_LIVOX_TILT_RPY_ROLL`。**xacro 没有 `livox_tilt_axis` arg**，它有 `livox_tilt_rpy`：launch 继续用**同一 key/引号格式**传冻结值，xacro/生成器**未改** |
| `livox_raise_m` | `default='0.0'` | **`0`** | launch 里**删除**声明/`_robot_xacro_args` 条目/`_gravity_align_h` 回退表里的 `+ raise_m`/`_lio_adapter_robot11` 的 `raise_m`/所有 B 方案注释。**xacro 仍保留 `livox_raise_m` arg 且 default=0**；launch 不再传 ⇒ xacro 用默认 0，生成物逐字节不变。**没有**改 xacro/生成器（`sentry_robot_robot11_sim.xacro` 是生成物，手改会被重跑覆盖） |

删除后的 launch 声明数：**41 → 39**（`--show-args` 总数 103 → 101；只少这两块，其余 101 个块逐字相同）。
`robot:=robot11` 的 `robot_description` 命令字符串只少一个 token：` livox_raise_m:=0.0`；
`livox_tilt_rpy:="-0.523598775598293 0 0"` 与引号规则逐字保留。默认模型/hzmirm 的命令字符串不变。

### 28.2 默认路径逐字节证据（主验收）

**(a) `--show-args`（Humble，私有 HOME/ROS_LOG_DIR）**

* 我们的 `DeclareLaunchArgument`：**41 → 39**；`--show-args` 总块数 **103 → 101**；
* 逐块比较：只删掉 `livox_tilt_axis` 与 `livox_raise_m` 两块，**没有**任何其它块 changed/added；
* `ros2 launch … --show-args` **exit 0**。

**(b) 生成的 `robot_description`（= xacro stdout；`robot11_mount:=plugin|urdf|sensor` 全跑）**

| robot | mount | 删除前 md5 / 字节 | 删除后 md5 / 字节 | 判 |
|---|---|---|---|---|
| `''`（默认） | plugin | `bc62af22f7be49395525978b1264750c` / 10353 | 同左 | ✅ 逐字节 |
| `''` | urdf | 同上（mount 不参与默认模型） | 同上 | ✅ |
| `''` | sensor | 同上 | 同上 | ✅ |
| `hzmirm` | plugin | `a13e053090bf1344a06d58241bf845f9` / 18630 | 同左 | ✅ |
| `hzmirm` | urdf | 同上（mount 不参与 hzmirm） | 同上 | ✅ |
| `hzmirm` | sensor | 同上 | 同上 | ✅ |
| `robot11` | plugin | `f4f8110d46eaf46dd3c3e2542f39a8e0` / 46955 | 同左 | ✅ |
| `robot11` | urdf | `14775e7fd159a115feeaf0cd340dc018` / 46955 | 同左 | ✅ |
| `robot11` | sensor | `b59a62144c4ffa827780c67f4dcf5e93` / 46997 | 同左 | ✅ |

**活栈复核**：nav 跑里 `ros2 param dump /robot_state_publisher` 的 `robot_description` =
**46997 B / md5 `b59a62144c4ffa827780c67f4dcf5e93`**，与上表 `robot11 + sensor` 逐字节一致。
（`pitch` 参考：删除前 `robot11` 三档的 `livox_tilt_rpy` 为 `0 -0.523598775598293 0`，生成物 md5
`2d41b65c… / 9c2b59e6… / c8071e7a…`；删除后该取值**不再可从 launch 选择**。）

**(c) `_gravity_align_h`（默认姿态；删除前/后逐位相同）**

| robot | h（m） | 活栈 `ros2 param get /small_point_lio gravity_align_h` |
|---|---|---|
| `robot11` | **0.25952717**（`0.102499 + 0.15702816968305`） | `0.25952717` |
| `hzmirm` | **0.800000** | —（本次 nav 用 robot11） |
| `''` | **0.235000** | — |

活栈另读到：`gravity_align_mode=hybrid`、`gravity_align_rpy=[-30.0,0.0,0.0]`、`gravity_align_z_origin=ground`、
`lio_tf_adapter rpy=[0.523598776,0.0,0.0]`、`xyz=[-0.000561701,-0.034862345,-0.151448296]` ——
与删除前解析值逐位相同。

**(d) `launch_increment_diff.py`（old = `eaea119` 的 launch 副本；new = 本次）**

* 整棵 Action 树：**OLD 72 / NEW 70 / 共有 69**；消失 = **2 个 `DeclareLaunchArgument`**
  （`livox_tilt_axis`、`livox_raise_m`）+ 1 个 `LogInfo`（robot11 横幅里那行轴文本被静态 `roll` 文案替换）；
  新增 = 1 个 `LogInfo`（替换后的横幅）。共有 Action 相对顺序**一致**。
* **执行集**（只取真正起进程的 `Node` / `IncludeLaunchDescription` / `ExecuteProcess`，含
  `TimerAction`/`GroupAction` 内嵌）：**OLD 37 / NEW 37，changed/removed = 0，added = 0，order = OK**。
  ⇒ 删除的只是 2 个声明动作与 1 句日志文本，**没有任何节点/参数/包含的 launch 改变**。

### 28.3 nav 冒烟（RMUL2026 / robot11 / sensor / `hybrid` 默认 / `band_guard:=True`）

**命令**（一条 bash 调用里起栈 + 采证 + 收尾；隔离与 §23–§27 同口径）：
`world:=RMUL2026 mode:=nav localization:=gicp lio:=small_point_lio ground:=linefit spin_speed:=0.0
robot:=robot11 robot11_mount:=sensor band_guard:=True nav_rviz:=False lio_rviz:=False gui:=False`
（`gravity_align_mode` 留默认 `auto` ⇒ `lio=small_point_lio` ⇒ 有效值 **hybrid**）。
隔离：私有 `HOME=.tmp_map2/home-smoke_v28_combined/`、`ROS_DOMAIN_ID=138`、
`GAZEBO_MASTER_URI=http://127.0.0.1:11738`、`gui:=False`、`world:=X` **不带前导 `--`**、
`setsid` + `tools/scripts/lib/scoped_cleanup.sh` + 硬超时，**无全机 pkill**；跑前 `free` 可用 **12 GiB**、
swap 仅 1.3 MiB，一次只起一栈，收尾 `leftovers` 空。

| 判据 | 实测 | 判 |
|---|---|---|
| **`band_guard` in-band** | 6 个样本 **全 100%**：`772/772, 782/782, 781/781, 769/769, 764/764, 771/771`，`below=0 above=0`、`one-sided=no`、`band=[-0.050,2.000]` | ✅ |
| **`h_meas`** | `h_urdf=0.2595 m h_meas=0.2617 m / \|Δ\|=0.0021 m < 0.03 ⇒ INFO`（inliers 14436、tilt 0.07°、resid p50 1.7 mm） | ✅ |
| **`odom→base_link` rpy** | 12 s 中位 **`[-0.060, -0.356, -1.062]°`**（范围 roll −0.143…+0.014 / pitch −0.497…−0.264 / yaw −1.121…−0.963）；同一跑较早的静态样本为 `[+0.088, −0.458, −0.122]°`。roll/pitch 与 §25 参考 `[+0.06,−0.47,−0.63]` 同量级；yaw 静止窗有 ~1° 级漂移，**不是 30° 残余** | ✅（量级） |
| **`robot_description`** | 活栈 `46997 B / md5 b59a…` = 基线 `robot11+sensor` 逐字节 | ✅ |

### 28.4 破坏性变更（exactly what ROS 2 does）

* **传旧参数现在不会报错，也不会警告**：ROS 2 Humble 的 `ros2 launch` 把未知的 `<name>:=<value>`
  放进 launch context，但**没有 `DeclareLaunchArgument` 就没有任何节点读它** ⇒ **静默忽略**。
  实测：最小 launch + `livox_tilt_axis:=pitch` 与真 launch + `livox_tilt_axis:=pitch livox_raise_m:=0.10`
  （`-p`）都 **exit 0、stderr 空、无 WARN/ERROR**；跑起来仍是冻结的 **roll / 0**。
* **要恢复旧行为**：只能改 **scratch** 副本（或 `git show eaea119:src/rm_nav_bringup/launch/bringup_sim.launch.py`）：
  * `pitch`：xacro 侧仍有 `livox_tilt_rpy` arg ⇒ 直接给 xacro 传
    `livox_tilt_rpy:="0 -0.523598775598293 0"`（或改 scratch 生成器的默认），**不再经过 launch**；
  * `raise`：xacro 侧仍有 `livox_raise_m` arg（default 0）⇒ 直接给 xacro 传
    `livox_raise_m:=0.10`，或用 scratch `robot11_make_sim_xacro.py --livox-raise-m 0.10` 生成；
    **必须手工**把 `linefit` 的 `sensor_height` 改成 `0.2595 + raise`（这正是删除它的理由：
    这个同步是第二个真源，launch 不替你做）。
* **`pitch`/`raise` 变体现在只需要 scratch 生成器/xacro**，与 launch 参数无关；本仓库的 xacro/生成器
  **没有改**（`sentry_robot_robot11_sim.xacro` 仍带这两个 arg，默认值就是冻结值）。

### 28.5 本轮没验证 / 诚实清单

1. **nav 只跑了 1 次**（§28.3 那跑，外加 1 次更短的 TF/参数采样）；没有 3 跑中位，也没有重跑
   §25 的 6 跑矩阵。`odom→base_link` 的 yaw 是单跑单窗，范围 ~1°，不能当成长期漂移结论。
2. **没有重跑 B 方案（raise≠0）与 pitch 对照**：它们已不属于 launch 行为，只验证了"旧参数被静默忽略"。
3. **没有在真机/非 Humble 上验证未知 launch arg 的行为**；结论只覆盖本机 Humble。
4. **`launch_increment_diff.py` 的整树比较必然报 2 个声明 + 1 句横幅日志的变化**（见 §28.2(d)）；
   本节用"执行集"口径给出 0 changed / 0 removed。两者不矛盾：前者是 launch 元数据，后者是真正起进程的节点集合。
5. **xacro 的 `livox_raise_m` arg 仍在**（default 0），没有从生成器里删除；若以后要彻底消灭这个
   隐藏旋钮，需要同时改生成器并重跑自证，**本轮没做**（任务允许"或传 0"）。
---

## §29 第十九轮（**本机 date=2026-10-21，07:0x–08:0x**）：**按实车改安装约定**（§4.12）+ **`odom_level` 静态边**（§4.11）—— 重力对齐从估计器补丁搬到帧图

> **本节只新增**（append-only）：§1–§28 一个字都没改。任务 = 用户决定 **"不要 E1 吧，加个 tf"** +
> **"按照实车来吧也好理解"** + **"odom_level 吧"**（局部代价图帧）。分两个提交：
> **① 安装约定按实车**（`base_link→livox_frame` rpy=0、倾角搬到 `livox_frame→imu_link`、
> LIO `extrinsic_R` 从 URDF 注入、`gravity_align_rpy` 改从 IMU 链取）；
> **② `odom_level`**（静态、latched、唯一发布者的 `odom→odom_level` + adapter 父帧 + 局部代价图帧）。

### 29.1 迁移清单（每个"把 odom 当水平帧"的消费者 → 改还是不改）

| 消费者 | 改前 | 本轮 | 理由 |
|---|---|---|---|
| `local_costmap.local_costmap.global_frame` | `odom` | **`odom_level`**（覆盖层 `config/nav2_params_sim_level_frame.yaml`，深合并、`level_frame:=odom` 时整层不命中） | 激光/STVL 的高度带必须在**重力对齐**帧里；这是**整张局部图**的帧 ⇒ 对 `obstacle_layer` / `stvl_layer` / `cloud` 全部生效 |
| `behavior_server.global_frame` | `odom` | **`odom_level`** | 它吃的就是局部代价图（`costmap_topic: local_costmap/costmap_raw`） |
| LIO rviz 固定帧（`fastlio.rviz` / `pointlio.rviz`） | `odom` | **`odom_level`**（`rviz2 -f <level_frame>`，`.rviz` 文件**不改**） | 显示重力对齐的世界；回滚档 `-f odom` = 文件原值 |
| `nav2.rviz` 固定帧 | `map` | **不改** | 显示层在 `map`（重定位之上） |
| `global_costmap.global_frame` | `map` | **不改**（stvl 全局 profile / `global_obstacle:=scan` 的 −1.0 覆盖层**逐字节不变**） | 全局帧必须在地图里 |
| `/scan` `frame_id` / p2l `target_frame` | `base_link` | **不改** | 契约要求；§4.11.8 的车体系语义 |
| linefit `gravity_aligned_frame` | `base_link`（sensor 档覆盖文件） | **删除覆盖文件** ⇒ 用槽位那份（`gravity_aligned_frame: ""` + `sensor_height: 0.2595`） | §4.12 之后点云本来就**水平** ⇒ 不再需要"再对一次重力" |
| 自击掩膜 / 近地剔除（`traversability_*`） | 传感器系（`livox_frame`） | **不改** | 自车几何的量（§4.11.8）；linefit 之后作用，坐标系不变 |
| `lio_tf_adapter` | 发 `odom→base_link` | **父帧可配**（`odom_frame`，默认档 = `odom_level`）+ 左乘静态边 | 契约 §4.11；`level_frame:=odom` 时**不传**新键 ⇒ 参数表与改动前相同 |
| 重定位槽（gicp/amcl/slam_toolbox/cartographer）`odom_frame_id: odom` | `odom` | **不改** | `map→odom` 仍是重定位器的边（契约 I3） |
| `cloud_accumulator.target_frame`（`map\|odom`） | `odom` | **不改** | 它要的是**平滑**帧（不是水平帧），落盘前一次 `map←odom` 刚体变换 ⇒ 几何不变 |
| `tf_bridge_spl_map_to_odom`（`localization:=''` 回退） | `map≡odom` 恒等 | **不改 + t=0 响亮 WARN** | 回退档会把 map 钉在**原始**世界（斜）；正式用法 = 给重定位槽 |
| 诊断脚本（`tools/scripts/diag/*` 里硬编码 `odom→base_link` 的 6+ 个） | `odom→base_link` | **不改**（诚实项） | 历史一次性工具；默认档那条边已不存在 ⇒ 复用前要改成 `odom_level→base_link` |

### 29.2 实现（两层的唯一真源）

| 层 | 文件 | 做什么 |
|---|---|---|
| 静态边 | **`src/rm_nav_bringup/scripts/odom_level_tf.py`**（新，Python 节点 `odom_level_tf`） | 往 `/tf_static` 发**一条** latched 变换；值 = `(R_level←odomᵀ, −Rᵀ·t_level)`；`R_level←odom` = **URDF 的 `gravity_align_rpy`**（IMU 链），`t_level` = 地面 z 原点；另做两条**只读**互检（`gravity_align_h_check` / URDF↔IMU Δ，阈值 3/5 cm、2/5°） |
| 父帧 | `lio_tf_adapter`（`odom_frame` 参数 + tf2 查 `odom_level←odom` 左乘） | `base_link` 挂在 `odom_level` 下；查不到边就**不发** TF（loud） |
| 局部帧 | `config/nav2_params_sim_level_frame.yaml`（新，2 个叶子键） | `local_costmap.global_frame` + `behavior_server.global_frame` |
| 安装约定 | `tools/scripts/regress/robot11_make_sim_xacro.py` + 重生成的 `urdf/sentry_robot_robot11_sim.xacro` | 关节 rpy=0；`imu_joint` = (倾角, 转过倾角的偏移)；插件 `<tilt_rpy>` = 倾角（**插件代码一行没改**，见 §4.12） |
| LIO 外参 | `bringup_sim.launch.py` 的 `_MountExtrinsic` | 从 URDF 解析 `extrinsic_T=[0,0,0.05]` / `extrinsic_R=Rx(+30°)`，注入三份 LIO 节点 |
| LIO 初值地图 | `small_point_lio.cpp`（off 分支） | **修 latent bug**：`gravity_align_mode=off` 时初始地图原来把雷达系点**原样入图**（= 假设 `R_IL=I`）；新约定 `R_IL=Rx(+30°)` ⇒ 必须先旋进 IMU 系。`R_IL=I` 走**原表达式**（逐字节不变） |

**静态边的值**（robot11 + `robot11_mount:=sensor`，全部由 URDF/几何算）：
`R_mount = Rx(−30°)`、`h = 0.259527170`、`h_imu = 0.216225900` ⇒
`R_level←odom = Rx(−30°)`、`t_level = (0,0,0.216226)`（URDF 模式）；
发布值 `T_odom←odom_level` = `xyz [0, 0.108113, −0.187257] m`、`rpy [30, 0, 0]°`（实测两次采样**逐位相同**）。

### 29.3 实测表（`mode:=nav`，`localization:=gicp`，`robot:=robot11`，`robot11_mount:=sensor`，`band_guard:=True`，RMUL2026）

| 判据 | 默认档（SPL，E1 off，`odom_level`） | fastlio 臂 | 回滚档 `level_frame:=odom` |
|---|---|---|---|
| `odom→odom_level` | **rpy [30.000, 0, 0]°、xyz [0, 0.1740, −0.3014] m，17 个样本 spread = 0** | 同左（边与 LIO 无关） | **不存在该帧** |
| `odom_level→base_link` rpy | **[+0.36, −0.27, −0.87]°**（roll 0.278–0.452 / pitch −0.478…−0.180 ⇒ **<0.5°**） | 未采到（探针超时） | — |
| `odom→base_link` rpy | **[+30.36, +0.19, −0.88]°**（≈ +30° ✓） | — | — |
| 局部 in-band（`band_guard`，帧 = `odom_level`） | **772/772、782/782、784/784 = 100%** | **783/783 = 100%** | （见下） |
| 地面 z（在 `odom_level` 里拟合） | **−0.0136 m**（tilt 0.93°，inliers 2859） | 未采到（点云晚于超时 ⇒ 回退 URDF z） | — |
| `base_link` z（在 `odom_level` 里，窗口） | **+0.1017 m**（判据 +0.1025，Δ = 0.8 mm） | — | — |
| 原始云（`livox_frame`）地面法向 vs +z | **0.062°**、d = **0.25826** m（判据 h = 0.259527，Δ = 1.3 mm） | **0.021°**、d = 0.259295 | — |
| URDF↔IMU Δ（静态边诊断） | **0.001°** ⇒ INFO（`R_imu` rpy = [−29.999, 0, 0]°） | 同 | — |
| `map→base_link` rpy | [−0.29, **−3.05**, +0.30]° ⇒ `map` 相对重力差 ~3°（既有性质，§4.11.7） | — | — |
| `map→odom`（gicp） | 采纳 **349/349 帧、拒绝 0**（健康） | — | — |

**`z 原点` 的关键实测（诚实项）**：`odom` 的**原点是估计器自己的起始位姿**，不是 URDF 能算的东西 ——
实测它在 robot11 上比 URDF 预测**高 0.1318 m**（`d=0.348016` vs `h_imu=0.216226`）。原因 = **出生下落**：
LIO 初始化时车还在出生高度（spawn z=0.2）上，世界原点被锚在那里；随后车落到地面、状态 z 走 −0.13。
⇒ 静态边用 URDF 的 `h` 时，地面会落在 `odom_level` 的 **−0.13 m**（第一版实测 `floor_z=-0.1315`）；
本轮把 z 原点改成**实测地面**（`z_origin_source:=measured`，第一帧点云拟合，`d` 直接当地面高度）后
**floor_z = −0.0136**（10× 改善），`base_link z = +0.1017` ✓。代价/边界：
* 静态边的 z 变成**一次启动测量**（不是常数）⇒ **跑间差 0.30…0.35 m**（下落时刻抖动）⇒ `odom_level`
  的**绝对 z 不可跨跑比较**；`R`（旋转）仍然纯 URDF；
* `h_meas` 互检因此报 **ERROR**（`h_meas=0.3912` vs `h_urdf=0.2595`，Δ=0.132）—— 这是**正确**的响亮信号：
  它把"`odom` 的 z 锚点与 URDF 不一致"这件事报出来了（与 §4.7 的"定基用 URDF、实测只诊断"相反，
  这里**必须**用实测，因为要放的是"地面相对 odom 原点"的位置）；
* `z_origin_source:=urdf` 可回到确定性版本（地面偏 0.13 m，但 `base_link` 与地面的**相对**关系仍对）。

### 29.4 回滚档（`level_frame:=odom`）

（见 §29.5 的表）

### 29.5 本轮**没**验的（诚实清单）

1. **pointlio 臂** 与 **`mode:=slam_nav`** 本轮**没跑**（时间/机时）；`level_frame:=odom` 回滚档只跑了静态结构
   diff（`visited_diff`：process 执行集 8/8，节点集合相同，唯一差异 = adapter 参数表多一个
   `odom_frame='odom'`（= YAML 默认值 ⇒ 运行期 param dump 相同））。
2. **lethal / nearest / 全局 stvl 格数**（验收 c 的那三项）：本轮探针 `nav_band_probe` 在
   `timeout` 前没写出 JSON（两次都是探针侧超时，不是栈的问题）⇒ **只有 `band_guard` 的 in-band 100%**
   与 `map→odom` 的健康证据；`lethal/nearest` **没有**本轮数字（旧约定的 run A 有：见 §29.6）。
3. **`h_meas` 在默认档是 ERROR**（不是 INFO）：见 §29.3 的说明 —— 它报的是 odom 的 z 锚点偏移，
   不是标定错误。
4. **`map` 不精确水平**（pitch −3.05°）：既有资产/重定位性质，本轮不动；全局层的高度语义因此
   只能和 `map` 一样好（§4.11.7）。
5. **画出来的雷达 mesh 现在是平的**（物理上斜 30°）：纯显示差异（§4.12）。
6. `odom_level` 的绝对 z 有 **跑间 ~5 cm 抖动**（§29.3 的 measured z origin）。
7. 诊断脚本 `tools/scripts/diag/*` 里硬编码 `odom→base_link` 的 6+ 个**没改**（§29.1 最后一行）。

### 29.6 附：`odom_level` 在**旧安装约定**下的 3+1 跑（Part 2 单独验证，2026-10-21 06:5x）

| 量 | 值 |
|---|---|
| 局部 in-band | 773/773、774/774、781/781、782/782（**全 100%**，帧 = `odom_level`） |
| `odom→odom_level` | [30, 0, 0]° / [0, 0.108113, −0.187257] m（两次采样逐位相同，18 样本 spread = 0） |
| `odom_level→base_link` rpy | [0.387, −0.203, −0.905]°（roll/pitch < 0.5°） |
| `odom→base_link` rpy | [30.38, 0.27, −0.90]° |
| 地面 z（在 `odom_level` 里） | **−0.1315**（= 当时的 URDF-z 模式，见 §29.3 的解释） |
| gicp | 采纳 552/552、拒绝 0 |
| `map→base_link` pitch | −3.06° |

> 该轮用的是**旧**安装约定（`base_link→livox_frame` rpy=−30°、点云在传感器系、`extrinsic_R=I`），
> 但 `odom_level` 的数值与 §29.3 **完全一致**（静态边的 R/t 由 IMU 链决定，与安装约定解耦）
> —— 这正好是"§4.12 与 §4.11 正交"的证据。

### 29.7 收尾补充（同一批跑，探针后到的数据）

| 量 | 默认档（SPL） | fastlio 臂 | 回滚档 `level_frame:=odom` |
|---|---|---|---|
| `band_guard` local in-band（样本数） | **100% × 13 个样本**（末样本 768/768） | **100% × 4 个样本**（末样本 765/765） | **100%**（帧 = **`odom`**，末样本 785/785 —— E1 hybrid 把 odom 摆正，= 改动前行为） |
| `band_guard` 帧 | `odom_level` | `odom_level` | **`odom`**（没有 odom_level 帧）✓ 回滚判据 |
| 扫描面 z（在代价图帧里） | **+0.102**（= `base_link` 在 `odom_level` 里离地的高度，与 TF 的 +0.1017 一致） | — | （`odom` 里，E1 已对齐） |
| `nav_band_probe` 的 `local`/`global` 格数块 | **空**（该探针本轮的图块没采到 ⇒ lethal/nearest/全局 stvl **没有**本轮数字） | 同 | 同 |

⇒ 验收 (c)/(d) 的**功能判据**（局部 in-band 100%）在 SPL 与 **fastlio** 两臂上都是 100%；
(e) `slam_nav` 与 pointlio 臂本轮未跑；(c) 的 `lethal≈270–340 / nearest≈0.70–0.77 / 全局 stvl≈3727`
**本轮没有数字**（探针的图块为空，不是栈的问题），只保留了 §29.6 里旧约定 run A 的健康证据
（gicp 采纳 349/349、拒绝 0；`band_guard` 100%）。

---

## §30 第二十轮（**本机 date=2026-10-21，07:2x–09:xx**）：`odom` 的 **z 锚点根因**（初始化落定门 + 初始化地图口径）+ 补 §29.5 的缺表

> **本节只新增**（append-only）：§1–§29 一个字都没改。§29.5 的"没验清单"在本节被逐条回答
> （第 1 条 = §30.5.2/§30.5.3，第 2 条 = §30.5.1，第 3 条 = §30.4，第 4 条 = §30.6，
> 第 6 条 = §30.3）。两个提交：**① 修 z 锚点**（估计器 + launch + 静态边默认值）、
> **② 文档 + 探针口径**。
>
> ⚠️ **先纠正 §29.3 的归因**（反直觉，但是实测）：z 偏移**不是**"LIO 在**坠落中**做初始化"。
> 实测初始化发生在车**落定之后 0.3 s**（§30.1）。那 0.13 m 是**初始化地图**把"整段下沉过程的
> 点云"一起入图（且少了 `t_IL` 平移）造成的 —— 现象在"出生窗口"里，机制是**初始地图**。

### 30.1 先量再改：三条时间线（只订阅的探针，同一个 run）

探针：`.tmp_zfix/trace_probe.py`（新，只读）：订阅 `/clock` `/odom` `/cloud_registered`
`/livox/lidar/pointcloud` `/imu/data` + TF；**时间轴 = 首帧原始点云的仿真戳**。
原始数据 `.tmp_zfix/runs/b1_trace_before/trace.json`（**修前**的二进制）与
`.../n2_fix_default/trace.json`（修后）。

**修前**（`lio:=small_point_lio`，默认档）：

| t（首帧云后，仿真 s） | 物理量：原始云（`livox_frame`）拟合地面的垂距 d | LIO 的 state z（`/odom`） | 发生了什么 |
|---|---|---|---|
| 0.0 | **0.3515** | — | 出生：base_link 在 spawn z=0.2 上（0.2595 + 0.0975 = 0.3570，差 5 mm = 首帧拟合残差） |
| 0.5 | 0.3280 | — | 下沉中 |
| 1.0 | 0.2970 | — | 下沉中（≈5.4 cm/s；**不是自由落体**：`gazebo_ros_planar_move` 每步把 vz 清零） |
| **1.7** | **0.2595** | — | **物理落定**（之后 30 s 的 d 稳定在 0.2595 ± 0.0008） |
| **2.0** | 0.2595 | **−0.0784**（第一次 `/odom`） | **LIO 初始化**（= `fix_gravity_direction` 的 200 帧 IMU @100 Hz）—— 车**已经落定 0.3 s** |
| 2.6 | 0.2595 | −0.0975 | 初始化后的**第一次点更新**把状态**又拽下 1.9 cm** |
| ≥3.0 | 0.2595 | **−0.0975 ± 0.002**（30 s 内不漂） | 稳定值 ⇒ `odom` 的 z 原点比 URDF 的 `h_imu` 高 **0.1300 m** |

**两条独立的错误，各占一半**（§30.4 用 2×2 A/B 分开量）：

1. **初始化地图 = 整段下沉过程的点云**。初始化门只要求
   `point_deque.size() >= init_map_size`（`init_map_size: 10` 是**点数**，不是帧数！）+
   200 帧 IMU（2 s），而 `Preprocess` 对 `point_deque` **只 append、从不清**
   （`preprocess.cpp`）⇒ 初始化那一刻它攒了 **26 帧 / 156266–164179 点**，全是车**正在下沉**时
   扫的（地面在传感器系里从 −0.3515 抬到 −0.2595），而初始化地图是**不带状态**直接入图的
   （`add_point(R_IL·p)`，没有 `t_IL`、也没有 `position`）⇒ 地面被"抹"成 **~9 cm 厚**的带
   ⇒ 第一次点更新把状态拽下去。
   **实测贡献 +0.077 m**（§30.4 的 2×2："只加门"把 state z 从 −0.0985 拉回 −0.0212）。
2. **初始化地图少了 `t_IL` 平移**（§29.2 的注释里登记为"既有偏差、不新增"）。robot11 的
   `R_IL = Rx(+30°)` 把 `t_IL=(0,0,0.05)` 变成 odom 系里的 `(0, +0.0250, **+0.0433**)`
   ⇒ 初始化地图整体低 **4.33 cm** ⇒ 第一次点更新把状态再拽下 4.3 cm。
   **实测贡献 +0.051 m**（§30.4 的 2×2："只改口径"把 state z 从 −0.0985 拉回 −0.0472）。

两条叠加（−0.098 m）+ 几何差（`h_imu` 与 `t_IL` 在 z 上的 0.0322 m）⇒ 静态边用 URDF 的 `h`
时地面落在 `odom_level` 的 **−0.1315 m**（§29.6 的实测）；这也正是 §29.3 被迫把 `t_level`
写成实测 `d=0.348` 的原因。

### 30.2 修法：**落定门 + 初始化地图口径**（都可单独回滚）

> **为什么不是任务书首选的"把 spawn z 调成落定高度"**：
> `spawn z` 硬编码在 `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py::get_world_config()`
> （`WorldType.RMUL2026: z=0.2`）——① 本轮任务**明令禁止改 `src/rm_simulation/**`**；
> ② 那张表被 **4 个 world × 3 个 robot 槽位**共用，而三个模型的落定高度各不相同
> （robot11 0.1025 / hzmirm 0.08 / 默认模型 0.06）⇒ 一个常数不可能同时对；
> ③ `spawn_entity.py` 的 `-z` 在 launch 里没有可透传的参数（加一个就得改那个包）。
> ⇒ 走任务书给的**替代路线**（延后初始化），但判据**不是**"IMU 静止 1 s"，理由见下。

| 改动 | 文件（唯一真源） | 开关（默认） | 作用 |
|---|---|---|---|
| **初始化落定门** | `small_point_lio.cpp`：`init_settle_gate()` / `update_ground_proxy()` | `init_settle_hold`（**0 = 关**）、`init_settle_tol`（0.005 m）、`init_settle_timeout`（15 s） | 必须**连续** `hold` 秒"雷达看到的地面高度不动"（峰峰值 ≤ tol）才允许初始化；**落定前的点云全部丢弃** ⇒ 初始化地图 = 落定后的**一帧**；超时 ⇒ 响亮 WARN + 照旧初始化（绝不静默卡死） |
| **初始化地图口径** | `small_point_lio.cpp` 的 init-map 分支 | `init_map_extrinsic`（**false = 旧式**） | true ⇒ 用与逐点更新**同一个**出点公式 `R_IL·p + t_IL`（初始化那一刻 R=I、position=0） |
| **静态边 z 原点** | `odom_level_tf.py` + launch | `z_origin_source`（**urdf**） | 回到确定性：`t_level = h_imu`；`measured`（§29 的挡板）保留为诊断 / A-B |

**为什么判据不是"加速度计静止 ≥1 s"（实测，反直觉）**：本仿真的 `gazebo_ros_planar_move`
每个物理步都 `SetLinearVel(vx,vy,0)` ⇒ 下沉是**匀速**的，**不是**自由落体 ⇒ 加速度计在下沉
期间**仍读到 ≈1 g**（`|a|` 中位数 9.80 m/s²）；而**静止时** `|a|` 本身在 **7.8–11.8 m/s²** 抖
（±20%，同一份 `imu_trace`，100 Hz）⇒ ① `|a|≈g` **不能**证明静止；② 任何
"`| |a| − g | < tol`" 的判据在静止时也**永远不满足**（tol 要到 ≥2.5 m/s² 才行，那时又分辨不
出下沉）。⇒ 判据换成**雷达自己的地面高度代理**：`r∈[1,6] m` 的点云 z 的 **10% 分位**。
实测它的 1 s 峰峰值：下沉段 **44 mm**、落定后 **≤1.5 mm** ⇒ 5 mm 阈值把两段分得很开
（本机 4 跑：门都在 **2.54–2.59 s** 放行 = 落定后 0.84–0.89 s，收据见 §30.3）。
⚠️ 这是**参数化**判定，不是常数延时：换 world / 模型只改 `lio_init_settle_s` 或 tol，不改代码。

**launch 侧**（`bringup_sim.launch.py`）：`lio_init_settle_s`（默认 **1.0**）、
`lio_init_map_extrinsic`（默认 **True**）只传给**默认档那一个** SPL Node
（`gravity_align_mode` 有效值 = off 且 `level_frame=odom_level`）；回滚档（`level_frame:=odom`）
与 E1 实验臂那两份 Node 的参数表**一个键都没多**（`git diff` 里只有第二份 Node 的字典变了）。
`z_origin_source` 变成 launch 参数（默认 `urdf`，`choices=[urdf, measured]`）。

### 30.3 修后验收（默认档 3 跑 + 修前对照）

命令（4 跑同口径；`SETTLE=20 WINDOW=20`，`gui:=False`，`nav_rviz:=False`，私有
`GAZEBO_MASTER_URI`/`HOME`/`ROS_DOMAIN_ID`，`scoped_cleanup.sh` 收尾）：

```text
world:=RMUL2026 mode:=nav localization:=gicp lio:=small_point_lio nav:=mppi planner:=smac2d \
robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
```

| run（口径） | 初始化（首帧后） | state z 首次/稳定 | 地面 z（`odom_level`） | `base_link` z | `h_meas`（判级） | 扫描面 z p50 | in-band |
|---|---|---|---|---|---|---|---|
| `b2_before_urdf`（**修前语义**：门关 + 旧口径 + urdf） | 2.00 s | −0.0788 / −0.0985 | **−0.1296** | **−0.0290** | 0.3889（Δ 0.1293，**ERROR**） | −0.0290 | 780/780 |
| `a1_gate_only`（只加落定门） | 2.60 s | +0.0004 / −0.0212 | **−0.0395** | **+0.0621** | 0.2979（Δ 0.0383，**WARN**） | +0.0610 | 767/767 |
| `n1_fix_default`（**修后**） | 2.60 s | +0.0239 / +0.0118 | **+0.0053** | **+0.1081** | 0.2551（Δ 0.0044，**INFO**） | +0.1082 | 772/772 |
| `n2_fix_default`（**修后**） | 2.60 s | +0.0245 / +0.0126 | **+0.0058** | **+0.1080** | 0.2536（Δ 0.0059，**INFO**） | +0.1090 | 766/766 |
| `n3_fix_default`（**修后**） | 2.60 s | +0.0240 / +0.0118 | **+0.0056** | **+0.1077** | 0.2537（Δ 0.0058，**INFO**） | +0.1080 | 767/767 |

* **地面 z**：−0.1296（修前）→ **+0.0053 / +0.0058 / +0.0056**（修后）⇒ 3 跑 **spread = 0.5 mm**
  （验收线 ±1 cm ✓，任务书的"spread < 1 cm" ✓；作为对照，修前 3 跑的 spread 是 1.9 mm 但**整体偏 13 cm**）。
* **`base_link` z**：+0.1081 / +0.1080 / +0.1077（spread **0.4 mm**）vs 判据 **+0.102499**
  ⇒ 系统偏 **+5.5 mm**（< 1 cm ✓）。这 +5.5 mm 全部落在"**odom 原点相对 URDF `h_imu` 的静态差**"
  这一处：`base_link` **相对拟合地面**仍是 **0.1024 m**（URDF 值 1 mm 内），地面则在 +0.0056 ⇒ 是**确定性偏置**
  （3 跑 spread 0.5 mm），来源在估计器的初始化（落定门的 5 mm 容差量级 + 初始位姿估计），不在静态边。
* **`h_meas`**：0.2551 / 0.2536 / 0.2537 ⇒ **Δ 4.4–5.9 mm ⇒ INFO** ✓（验收：Δ<3 cm）。
  修前同一口径是 0.3889 / Δ 0.1293 / **ERROR**（§29.3 的 0.3912 同源）。
* **扫描面 z**（`band_guard` 在**代价图帧** `odom_level` 里量的 `/scan` 平面）：0.108–0.109
  ⇒ 与 `base_link` z（0.108）**一致**（`/scan` 的 frame 就是 `base_link`）✓；
  in-band **100%**（766/766…780/780，未出现任何 below/above）。
* **`odom_level→base_link` rpy**：`[−0.012, +0.015, −0.361]°` / `[+0.012, +0.040, −0.404]°` /
  `[+0.015, +0.007, −0.398]°` ⇒ **roll/pitch ≤ 0.04°**（契约 I11 的线是 0.5°）✓。
* **静态边 `odom→odom_level`**：3 跑**逐位相同** `xyz=[0, 0.108112950, −0.187257122] m`、
  `rpy=[30, 0, 0]°`，spread = 0（17–20 个窗口样本）✓（契约 I8）。
* **`map→base_link` rpy**：`[−0.274, −3.073, +0.243]` / `[−0.257, −3.052, +0.232]` /
  `[−0.243, −3.065, +0.226]°` ⇒ pitch 仍是 **−3.05…−3.07°**（**与本修无关**，见 §30.6）。

### 30.4 因果分解（2×2 A/B）—— 两个错误各占一半，缺一个都不够

同一台机、同一命令，只切两个开关（`lio_init_settle_s:=0` / `lio_init_map_extrinsic:=False`）：

| # | 落定门 | 初始化地图含 `t_IL` | run | state z（稳定） | `base_link` z（`odom_level`） | 地面 z | `h_meas` | 判级 |
|---|---|---|---|---|---|---|---|---|
| ① 修前 | off | 否 | `b2_before_urdf` | −0.0985 | −0.0290 | −0.1296 | 0.3889 | ERROR |
| ② 只加门 | **on** | 否 | `a1_gate_only` | −0.0212 | +0.0621 | −0.0395 | 0.2979 | WARN |
| ③ 只改口径 | off | **是** | `f1_after_default` | −0.0472 | +0.0223 | （旧探针口径，略） | 0.3381 | ERROR |
| ④ 两个都上 | **on** | **是** | `n1..n3` | +0.0118…+0.0126 | +0.1077…+0.1081 | +0.0053…+0.0058 | 0.2536…0.2551 | **INFO** |

* 门单独 ⇒ `base_link` z **+0.091 m**；口径单独 ⇒ **+0.051 m**；两个一起 ⇒ **+0.137 m**
  （≈ 两者之和，弱耦合）⇒ **两条都是必需项**，不是"顺手修一下"。
* ①→④ 的总改善 = **+0.137 m**，其中 ~2.2 cm 被 LIO 自己的残余偏置吃掉（④ 的 +0.0118 残差）。
* ⚠️ `f1_after_default` 是第一版门（`t0` 把驱动那帧 **stamp=0** 当成"出生时刻" ⇒ 立刻撞 15 s
  超时兜底）⇒ 它的门**实际没生效**，正好当"只改口径"这一档；修法（`t_now <= 0` 一律不认）
  在 `init_settle_gate()` 里有注释。**诚实项**：这是我自己踩的坑，不是既有代码。

### 30.5 补 §29.5 的缺表（Part 2）

#### 30.5.1 默认档 nav 3 跑：local / global / in-band / `h_meas` / rpy

（探针 `tools/scripts/tiltmount/nav_band_probe.py`，**本轮修好口径**：`--scan-frame odom_level`
（§4.11 之后局部图的帧）、`d_range` 允许负值（地面就在 z≈0，原口径 (0.02,1.0) 会把它排除）。
⚠️ §29.5/§29.7 说的"local/global 块为空"**不是探针没采到**，是**汇总脚本读错键名**
（`local`/`global` vs 探针的 `local_costmap`/`global_costmap`）—— 那两次的 JSON 里数字都在
（`.tmp_level/runs/e29_a_spl_nav/navband.json`：local lethal 293、global 2161.5）。本节的表
从 JSON 直接读，键名写在 `nav_band_probe.py` 的输出里。）

| run | local lethal | inscribed | free | nearest (m) | ahead (2.0×0.6 m) | within 1 m | global lethal | global inscribed | global free | in-band | `base_link` z | rpy(level→base) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `b2_before_urdf`（修前） | 287 | 10676 | 40160 | 0.7160 | 21 | 29 | 2160 | 12241 | 12858 | 780/780 | −0.0290 | +0.25/−0.10/−0.98 |
| `n1_fix_default` | 275 | 10510 | 40361 | 0.7127 | 22 | 28 | 2160 | 12234 | 12843 | 772/772 | +0.1081 | −0.01/+0.01/−0.36 |
| `n2_fix_default` | 249 | 10218 | 40380 | 0.7124 | 24 | 30 | 2160 | 12251 | 12804 | 766/766 | +0.1080 | +0.01/+0.04/−0.40 |
| `n3_fix_default` | 255 | 10159 | 40509 | 0.7133 | 22 | 28 | 2160 | 12257 | 12807 | 767/767 | +0.1077 | +0.01/+0.01/−0.40 |

（`in-band` 里的 `n1` 用探针的 772/772 —— 那一跑的 `band_guard` 取证文件在收尾时丢了，
见 §30.8 第 11 条；`n2`/`n3` 是 `band_guard` 的 766/766、767/767，两条独立链路同值。）

* **local 图非空**（lethal 249–275）、**最近致命 0.712–0.716 m**、**前方窗口 21–24 格**、
  **1 m 内 27–30 格** ⇒ 与 §25/§26/§29 的 270–340 / 0.70–0.77 / 22–31 **同一档**（§30.7 的 A/B）。
* **in-band 100%**（每跑 766–780 帧，below/above 全 0）⇒ 修后扫描面 z 落到 **+0.108**，
  比修前的 −0.029 **离下界 −0.05 更远**（余量从 21 mm 变成 158 mm）⇒ 鲁棒性**变好**，不是变差。
* **global（`global_obstacle:=stvl`，`map` 帧）**：lethal **2159.5 / 2160 / 2160**、inscribed
  **12234–12257**、free **12804–12843**、nearest **0.7354–0.7368 m** ⇒ 与修前（2160 / 12241 /
  12858）**逐格不变**（全局层不吃局部 z 锚点，符合 §4.11.7）。
  ⚠️ 这个 2160 **不是** §25/§26 的 **3727**：那 3727 是 §4.12 之前的点云口径 + 旧世界 z 下的数
  （§29 起就是 2158–2163，见 §29.3 的 e29_a/e29_c）⇒ **不要**拿 2160 去和 3727 比。
* `h_meas` INFO、`odom_level→base_link` rpy ≤0.04°、静态边逐位不变、`map→base` pitch −3.05°：
  见 §30.3。

#### 30.5.2 `lio:=pointlio` 臂（1 跑）

命令只换 `lio:=pointlio`（其余同 §30.3）。**预期**：Point-LIO 的 `Set_init` 已经把世界摆正
⇒ 静态边的 `input_world=level` ⇒ `odom→odom_level` 的**旋转 = I**（不再是 30°），
只有 z 平移（`input_z=sensor`、`output_z=ground` ⇒ `t_level=(0,0,h_imu)`）。

| 量 | 实测 |
|---|---|
| `odom→odom_level`（静态边） | xyz **[0, 0, −0.216225900] m**、rpy **[0, 0, 0]°**（两次采样**逐位相同**；p2 复跑同值）⇒ **旋转 = I**（input_world=level 生效，没有二次旋转）；z 平移 = `−h_imu`（把地面放到 0） |
| `odom→base_link` rpy | **[−0.013, −0.139, −1.082]°** ⇒ **没有 30° roll**；残余 roll/pitch **0.14°** ⇒ pointlio 的 odom 确实重力对齐 |
| `odom_level→base_link` rpy / z | [−0.013, −0.139, −1.082]° / **+0.0298 m** |
| 局部 in-band | **772/772（探针）**；`band_guard` 四条 784/784、783/783、769/769、780/780 ⇒ **100%** |
| 扫描面 z（`odom_level`） | **+0.028**（= `base_link` z，两者一致） |
| local lethal / nearest / ahead（2.0×0.6 m） | **238 / 0.7286 m / 21**（inscribed 8951、free 43156、1 m 内 26） |
| global（`map` 帧，stvl） lethal / nearest | 1970 / 0.7780 |
| `map→odom`（gicp）rpy | [−0.355, −3.027, +0.489]°（健康） |
| **z 锚点（诚实项）** | **低 6.5 cm**：`base_link` z = **+0.0298（p1）/ +0.0362（p2）**（判据 +0.1025），地面 ≈ **−0.071 / −0.062 m**（地面 z 是**换算值** `base_z + livox 力臂 − 原始云垂距`：pointlio 的 `/cloud_registered` 只有 **137 点/帧**，探针的 ≥500 点门槛拟合不了平面）。原因 = **pointlio 自己在"下沉中"初始化**：trace 实测它的第一次 `/odom` 在**首帧后 0.50 s**（SPL 是 2.0 s），state z 从 −0.00001 掉到 **−0.0767**（≈ 当时还差 6.9 cm 没沉完 + 它的漂移） |

> ⚠️ **这个反讽值得记一笔**：§29.3 把"在下沉中初始化"的帽子扣在 `small_point_lio` 头上 ——
> 实测那个槽位**没有**这个问题（它 2.0 s 才初始化，车 1.7 s 就落定了）；**真正**在下沉中
> 初始化的是 **pointlio**（0.5 s）。本轮只修 SPL（默认档），pointlio 的初始化门在
> `point_lio/ImuProcess::Set_init` 里、契约 §4.11.6 明确不在范围内 ⇒ 只记录，不修。

**同一个 arm 跑了 3 次**（也是被上面那个探针 bug 逼出来的：第一跑误报、修好后复跑两次）：

| 跑 | 静态边 | 局部 in-band | local lethal / nearest / ahead | `base_link` z | 扫描面 z |
|---|---|---|---|---|---|
| `p1_pointlio` | I / (0,0,−0.216226) | 772/772 + 探针 771/771 | **238 / 0.7286 / 21** | **+0.0298** | +0.028 |
| `p2_pointlio` | 同（逐位） | 780/780 + 探针 771/771 | **0 / — / 0** ← **局部图整张空** | **+0.0362** | +0.033 |
| `p3_pointlio` | 同（逐位） | 探针 771/771 | **317 / 0.7068 / 34** | **+0.0356** | +0.037 |

⚠️ **`p2` 的局部代价图是空的**（62500 格里 lethal 0 / free 62500），而**同一跑**的 `/scan` 在带
100%、`band_guard` 也 100%、nav2 **一条** `Message Filter dropping` 都没有 ⇒ 是**静默丢**（与
§9.7/§15.6 同一类：nav2 的 scan MessageFilter 在 TF 戳边界上丢消息、没有 WARN）。三条
"Timed out waiting for transform ... base_link_fake → odom_level" 只在启动段出现（所有 arm 都有）。
⇒ **pointlio 臂的局部数字 1/3 跑不可用**（p1/p3 有效，238–317 格），这是**该 arm 既有的
时序脆弱性**，与本轮改动无关（这个 arm 里根本没有 SPL 节点，参数表/静态边也没变），但**要修**得
单独查 `lio_tf_adapter` 在 pointlio 下的 TF 戳延迟（**未做**）。

#### 30.5.3 `mode:=slam_nav`（1 跑，默认 SPL）

命令：`world:=RMUL2026 mode:=slam_nav lio:=small_point_lio nav:=mppi planner:=smac2d
robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True`
（**不给 `localization`**：slam_nav 的 `map→odom` 归 slam_toolbox）。

| 量 | 实测 |
|---|---|
| `map→odom`（slam_toolbox）**健康** | rpy **[−29.959, +0.107, −0.065]°**、xyz [0.0003, 0.0633, 0.1092] m；窗口 18 个样本 **spread = 0**（不跳变、不漂） |
| `map→odom_level` | rpy **[+0.041, +0.107, −0.065]°** ⇒ 在 slam_nav 里 **`map` 就是水平帧**（slam_toolbox 以 `odom_level` 起步 ⇒ 它把 map 定义在正的世界里，**没有** nav+gicp 那个 3° 资产倾角） |
| `map→base_link` rpy | [+0.067, +0.150, −0.514]°（roll/pitch ≤0.15°） |
| 局部 in-band | **786/786 = 100%**（`band_guard`，帧 = `odom_level`）；探针 786/786 |
| 扫描面 z（`odom_level`） | **+0.108**（p50），min/p50/max = 0.101/0.108/0.123 |
| local lethal / nearest / ahead | **261 / 0.7127 m / 23**（inscribed 10569、free 40293、1 m 内 29） |
| z 锚点 | floor **+0.0042**、`base_link` z **+0.1077**、`h_meas` **0.2529 INFO**（Δ 6.7 mm） ⇒ 与 nav 档一致（`init_settle` 收据：2.62 s） |
| 全局图（`map` 帧） | lethal **206** / inscribed 2280 / free 596 / nearest 0.7286 ⇒ **在线 SLAM 的小图**（不加载磁盘地图），与 nav 档的 2160 不可比 |
| **nav2 静默** | 启动段（仿真 ≤645 s）：slam_toolbox 丢 **2** 条（`time 0.000` + 队列满 1 条）、两张 costmap 各丢 1 条（TF 缓存还没建立）——**全在窗口之前**；测量窗口（≥662.6 s）内 **0 条** `Message Filter dropping` / `Failed to transform` / `extrapolation`；`Robot is out of bounds of the costmap!` 1 条（t≈644.2，在线图刚建、车在图外，属启动段） |
| ⚠️ 假 WARN | launch 打了一条 `level_frame=odom_level + localization=''` 的 WARN（"全局图不会摆正"）—— 它对 **slam_nav 是误报**：实测 `map→odom_level` 只有 0.1°，全局图是正的。WARN 的条件把 `slam_nav` 也算进了"LIO 当绝对定位"的回退路径（`localization=''`），但 slam_nav 有 slam_toolbox 给 `map→odom` ⇒ 条件该收紧（本轮**没改**，记在这里） |

#### 30.5.4 可选：`local_obstacle:=stvl`（1 跑，默认 SPL）

命令加 `local_obstacle:=stvl`（局部图改吃 `/segmentation/obstacle` 3D 体素：`voxel_size 0.05`、
0.5 s 衰减、高度带 **[0.0, 2.0]**；`obstacle_layer`/`obstacle_cloud_layer` 都 `enabled=false`）。

| 量 | 实测 |
|---|---|
| 生效参数（回读） | `local_costmap.global_frame=odom_level`、`stvl_layer.enabled=True`、`stvl_layer.livox_mark.min/max=0.0/2.0`、topic `/segmentation/obstacle`、`obstacle_layer.enabled=False` |
| **in-band** | `band_guard` 对这两张图**报 IDLE**（局部没有启用的 2D scan 层 ⇒ **2D 高度带不适用**，这是设计而非失败）；扫描面 z（`odom_level`）仍可量 = **+0.108**，落在 STVL 带 [0,2] 内 |
| local lethal / nearest / ahead | **167 / 0.7316 m / 29**（inscribed 11844、free 39255、1 m 内 32） |
| global（stvl）lethal / nearest | 2161 / 0.7355（inscribed 12252、free 12815）⇒ 与 scan 档 2160 **一致** |
| z 锚点 | floor **+0.0054**、`base_link` z **+0.1081**、`h_meas` **0.2529 INFO**（Δ 6.7 mm） ⇒ 换局部层**不影响** z 锚点 |
| 判读（A-vs-B 的 2D 决策） | scan 档（3 跑）lethal **249–275**、nearest **0.7124–0.7133**、ahead **22–24**；stvl 档 lethal **167**（−35%）、nearest **0.7316**、ahead **29** ⇒ STVL 的 3D 体素把近处矮障碍**并得更粗**（voxel 0.05 vs 栅格 0.02）且 lethal 更少、前方窗口更多 ⇒ **两档都"非空且合理"，但是不同的障碍表示**，不是谁更"对" |
| ⚠️ 附带收益 | 修好 z 锚点之后，STVL 的 `min_obstacle_height: 0.0` 才**真的**是"地面以上"：修前 `odom_level` 的地面在 −0.13 ⇒ 带下沿实际是"离地 13 cm"，矮障碍会被静默切掉 |

### 30.6 `map` 的 pitch −3.05°：**资产**，不是 GICP（离线 1 查）

问题（§29.5 第 4 条）：`map→base_link` 的 pitch 一直是 −3.05°，是**地图资产**歪，还是
**GICP 对齐**歪？**离线**查（不动任何资产、不跑仿真）：

直接把 `src/rm_nav_bringup/PCD/RMUL2026.pcd`（**map 帧**，27687 点）里的地面拟合出来：

| 量 | 值 |
|---|---|
| 地面 RANSAC 内点 | **6199**（2 cm 容差） |
| 平面法向（map 帧） | `[−0.0496, +0.0090, +0.99873]` |
| **法向 vs map 的 +z** | **2.889°**（= roll +0.52° / pitch +2.84°） |
| 内点残差 p50 / p95 | 7.0 / 17.9 mm（网格本身 ±6 mm 不平，§13.3） |

⇒ **资产自带 2.89° 的 pitch**（这份 PCD 是**建图那一跑的 LIO 世界**，那时还没有 §4.12 的
安装约定/重力对齐 ⇒ 世界本来就是斜的）。运行期实测 `map→base_link` pitch = −3.05…−3.07°
（§30.3 三跑）⇒ 其中 **2.89° 来自资产**、剩下 **~0.2° 来自 GICP 的 `map→odom`**
（同一批跑 `n2`：`map→odom_level` 的 pitch = **−3.089°** vs `map→base_link` 的 −3.052°）。

**结论**：`map` 不水平的**主因是资产**（≈95%），GICP 只贡献 0.1–0.2°。
⇒ 全局层的高度语义只能和 `map` 一样好（§4.11.7 不变）；**本轮不动资产**（任务书要求）。
要真修就得用**重力对齐后的世界**重建 PCD/2D 图（那是另一件事）。

### 30.7 nav A/B 有没有退化？（修前 vs 修后，同机同命令）

| 指标 | 修前（urdf 锚点：`b2` + §29 的 `e29_a`/`e29_a2`/`e29_c`） | 修后（`n1`–`n3`） | 判读 |
|---|---|---|---|
| local lethal（中位，3–4 跑） | 287 / 293 / 296 / 278 | **275 / 249 / 255** | 低 4–13%，仍在 §25/§26 的 270–340 带内（§22.8 注：7 mm 级位姿差就能让 local lethal 差 40 格） |
| local nearest lethal | 0.7160 / 0.7106 / 0.7107 / 0.7087 | **0.7127 / 0.7124 / 0.7133** | 差 ≤5 mm ✅ |
| local ahead（2.0×0.6 m） | 21 / 22 / 22 / 22 | **22 / 24 / 22** | 同档 ✅ |
| local inscribed / free | 10675 / 40160（b2） | 10159–10510 / 40361–40509 | 同档（±3%）✅ |
| global lethal | 2160 / 2161.5 / 2159 / 2158 | **2160 / 2160 / 2160** | 逐格不变 ✅ |
| in-band | 780/780；§29 三臂全 100% | **766/766、767/767、772/772（100%）** | 不变 ✅ |
| `h_meas` | 0.3889 ERROR（urdf）/ 0.3912 ERROR（measured） | **0.2536–0.2551 INFO** | **变好**（这是本修的目标）✅ |
| 扫描面 z（代价图帧） | −0.029（b2）/ +0.100（measured） | **+0.108** | 带内余量 21 mm → 158 mm ✅ |

⇒ **没有功能退化**：局部图的三个"非空/最近/前方"判据在同一档、全局图逐格不变、
in-band 100%；唯一的系统性变化是**扫描面整体上移 13 cm**（= 修掉的那 13 cm 偏置），
它让 2D 高度带的余量**变大**。

### 30.8 诚实清单

1. **§29.3 的归因被本节纠正**（"坠落中初始化"→"初始化地图含下沉过程的点"）；§29.3 的
   **数字**（0.1318 / −0.1315 / h_meas 0.3912）全部复现，错的只是机理叙述。
2. **"measured 档跑间抖 ~5 cm"没复现**：本节两跑（`b0`=0.346243、§29 的 `e29_c`=0.348016）
   只差 **1.8 mm**。§29.3 说的 0.30–0.35 可能来自更早/更杂的批次。**能确定的是**：measured
   档会让 `h_meas` 必然 ERROR、且绝对 z 不可跨跑比较（这两条成立）。
3. **残余 +5.5 mm 的 `base_link` z**：来自 LIO 自己的初始化后暂态（state z +0.0118…+0.0126），
   不是静态边；要消掉得动估计器的初始速度/重力估计（本轮没做）。
4. **门的判据是"雷达地面高度代理"**（10% 分位、5 mm 峰峰值），不是"IMU 静止"——理由见 §30.2。
   它对**平地 + 雷达能看到地面**的场景成立；悬空/楼梯/被抬起的出生点会走**超时兜底**
   （15 s，响亮 WARN），那时 z 锚点会退回修前水平。**没有**在 hzmirm / 默认模型 / RMUC2026 上实测。
5. **门只在默认档生效**（`level_frame:=odom_level` + `gravity_align_mode` 有效值=off）。
   回滚档 / E1 臂 / `lio:=fastlio|pointlio|cartographer` **没有**这个门（它们的初始化在别的包里）。
6. **GICP 的 `map→odom` 在修后仍是 −30.7° 的 roll**（`map` 帧是斜的 ⇒ 它必须吸收）——
   这是 §4.11.7 的既有性质，不是本修引入。
7. **nav 3 跑是"同机同时段"**（宿主机 load 会变），local lethal 的 4–13% 差异**分不出**
   是位姿差异还是机器噪声（§22.8 同款边界）。
8. **global stvl 的 2160 ≠ §25/§26 的 3727**：口径在 §4.12 换过（见 §30.5.1 的注）。
9. **探针口径**：`nav_band_probe.py` 本轮加了 `--scan-frame`（默认 `odom_level`）；
   `level_probe.py`（未入库，`.tmp_zfix/`）的 `d_range` 改成正负都收、点云门槛 500→50、
   "无 odom_level" 的快速失败 40 s→150 s —— 这几处若不改，§4.11 之后的地面读数会**系统性错**
   （本轮实测踩到：floor_z −0.0405 vs 真值 +0.0056；pointlio 的稀疏云会被误判成"没有 odom_level"）。
10. **`n1` 的收尾取证文件丢了**（`band_guard.txt`/`tf.json`/`params.txt`；harness 侧，不是栈侧）：
    它的 `level.json`/`navband.json`/`trace.json` 完好 ⇒ 表里 `n1` 的 in-band 用**探针**的
    772/772、静态边/adapter 用 `n2`/`n3`（同一棵树、md5 相同）。
11. **`lio:=pointlio` 跑了 3 次**（第一跑 `p1` 的 `level_probe` 因为点云门槛误报"odom_level 不可用"）；
    z 锚点 0.0298 / 0.0362 / 0.0356 ⇒ 跑间 6.4 mm，那一臂的 z 读数**只到厘米级**（它自己的初始化
    在 0.5 s，见 §30.5.2）；**局部代价图 1/3 跑整张空**（`p2`，静默丢 scan）⇒ 那一臂的
    lethal/nearest 只有 2 个有效样本（238 / 317）。
12. **pointlio 的落盘副作用**：它在子模块里写 `PCD/scans_*.pcd`（未跟踪）；本轮生成的那几个
    已删（`git status` 里只剩它原本就有的 `PCD/` 与 `laserMapping.cpp` 改动，**都不是本轮的**）。
13. **没有重跑** §10/§13/§25 的完整矩阵，也没有碰地图/PCD/世界资产。

---

## §31 第二十一轮（**本机 date=2026-10-21，18:42–18:54**）：雷达安装角**收口** —— 上游 URDF 是唯一来源，删除 launch 冻结常数

> **本节只新增**（append-only）：§1–§30 一个字都没改。任务（用户批准）："那就是我们自己生成的
> xacro 改呗，然后收口"。目标 = 让安装倾角只存在于**一条链**：
> `urdf/upstream/robot11.urdf` 的 `body_to_livox.rpy` → 生成器 → 生成物 xacro → 运行期解析；
> 删掉 `bringup_sim.launch.py` 里的 `_LIVOX_TILT_RPY_ROLL` 与 `('livox_tilt_rpy', ...)` 映射。

### 31.1 改了什么（file:line）

| 文件 | 位置 | 改动 |
|---|---|---|
| `tools/scripts/regress/robot11_make_sim_xacro.py` | `HEADER` 第 235 行 | `<xacro:arg name="livox_tilt_rpy" default="{tilt_rpy}"/>`（值由 `main()` 注入） |
| 同上 | 第 767–790 行（`main()`） | 从上游 `body_to_livox` 的 `<origin rpy>` 读倾角；`--livox-tilt-rpy`（第 707 行）只做 scratch 覆盖；`_norm_rpy()` 校验 ±π 且**逐字保留**数值写法 |
| 同上 | 第 1027–1031 行 | `body_to_livox` 的 xyz **x/y/z 全部逐字照抄上游**（z 再叠 `livox_raise_m`，默认 0） |
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | 第 146–153 行 | **删除** `_LIVOX_TILT_RPY_ROLL`；改成"单一来源"说明 |
| 同上 | 第 285–321 行 | 新增 `_robot11_tilt_rpy_from_xacro()`：从**生成物** xacro 的 arg 默认值读回倾角（只服务旧表回退/交叉校验） |
| 同上 | 第 451–455 行 | `_robot_xacro_args()` **删除** `('livox_tilt_rpy', ...)`；robot11 只传 `livox_mount` / `visual_decimated` |
| 同上 | 第 335 行 | `_lio_adapter_robot11()` 的旧表倾角改从 `_robot11_tilt_rpy_from_xacro()` 读，不再有第二份常数 |
| `src/rm_simulation/rm_livox_gz_sim/launch/gz_sim.launch.py` | 第 218–232 行 | 单独跑（`robot_description` 为空）时**不再**传 `livox_tilt_rpy:=...` 字面值，只传 `livox_mount`，让生成物的默认值生效（否则这里是第二个所有者） |
| `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` | 第 12 行 | **生成物逐字节未变**（重新生成后 md5 仍是 `c5e12d36be2c6dac2d7968c3cecb930f` / 56140 B）—— 值本来就已经烘在里面，本轮只是把**来源**从硬编码改成上游 URDF |

**生成器自证**（每次写文件前，不通过就不写）：`[weld] 11 link -> 1 刚体 … AABB 逐条相等=True、最低点在轮上=True`、
`[wheel-mu] μ=0.02 真的进了 SDF：1 link / 113 条 collision … 几何 AABB 逐条相等=True、M=9.5621 kg / Izz=0.12912`。
默认/20°/pitch 三次生成都通过。

### 31.2 默认路径逐字节证据（主验收）

**(a) 生成物 xacro / 上游 URDF**

| 文件 | 改动前 md5 / 字节 | 改动后 md5 / 字节 | 判 |
|---|---|---|---|
| `src/rm_nav_bringup/urdf/upstream/robot11.urdf` | `36761d0c3315441ead01ff9d9d857cba` | 同左（**一个字节没动**；本任务只读） | ✅ |
| `src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro` | `c5e12d36be2c6dac2d7968c3cecb930f` / 56140 | **同左** | ✅ 重新生成后逐字节相同 |

**(b) `robot_description`（= launch 同一条 xacro command 的 stdout；9 个 (robot, mount) 组合）**

| robot | mount | 改动前 md5 / 字节 | 改动后 md5 / 字节 | `h`（m） | 判 |
|---|---|---|---|---|---|
| `''`（默认） | plugin | `bc62af22f7be49395525978b1264750c` / 10353 | 同左 | 0.235000 | ✅ |
| `''`（默认） | urdf | 同上（mount 不参与默认模型） | 同左 | 0.235000 | ✅ |
| `''`（默认） | sensor | 同上 | 同左 | 0.235000 | ✅ |
| `hzmirm` | plugin | `a13e053090bf1344a06d58241bf845f9` / 18630 | 同左 | 0.800000 | ✅ |
| `hzmirm` | urdf | 同上（mount 不参与 hzmirm） | 同左 | 0.800000 | ✅ |
| `hzmirm` | sensor | 同上 | 同左 | 0.800000 | ✅ |
| `robot11` | plugin | `4f19a8b4faa3a7c659850d0064d3c5ca` / 47745 | 同左 | **0.25952717** | ✅ |
| `robot11` | urdf | 同上（xacro 的 `livox_mount` 自 461eab8 起不再改几何） | 同左 | **0.25952717** | ✅ |
| `robot11` | sensor | 同上（= launch 当前唯一合法值） | 同左 | **0.25952717** | ✅ |

> 口径：`robot11` 的 `plugin|urdf` 自 2026-10-21 起**不再是 launch 合法值**（`_validate_livox_mount`
> 硬报错）；为了与 §28 的 9 行表逐行可比，这两行在探针里**绕过 launch 校验、直接按同一 command 格式**
> 把 `livox_mount:=plugin|urdf` 传给 xacro（该 arg 现在是惰性的）⇒ 输出与 `sensor` 相同。
> 真 launch 的 `robot11 + sensor` 路径另有活栈复核（§31.4）。

**command 字符串的变化**（robot11，唯一变化 = 少一个参数）：

```text
- xacro …/sentry_robot_robot11_sim.xacro xyz:="0.12 0.0 0.175" rpy:="0.0  0.0  0.0" livox_tilt_rpy:="-0.523598775598293 0 0" livox_mount:=sensor visual_decimated:=true
+ xacro …/sentry_robot_robot11_sim.xacro xyz:="0.12 0.0 0.175" rpy:="0.0  0.0  0.0"                                  livox_mount:=sensor visual_decimated:=true
```

**(c) `--show-args`（Humble，私有 HOME / ROS_LOG_DIR）**：改动前/后都是 **421 行、105 个参数**，
`cmp` **逐字节相同**、exit 0。⇒ 没有删任何 `DeclareLaunchArgument`（本任务只删了一个**传给 xacro 的
mapping**，不是 launch 参数），所以是"不变"而不是"−1"。

### 31.3 ⭐ 决定性测试：改 scratch 上游的 `body_to_livox.rpy`，全部消费者跟着走

**做法**：把仓库的上游文件**复制**到 `.tmp_mountone/upstream_20deg.urdf` / `upstream_pitch.urdf`
（只改 `body_to_livox` 的 `<origin rpy>`），用本生成器重新生成 scratch xacro，再用**与 launch 同一套**
`_parsed_livox_mount()` / `_GravityAlignRpy` / `_GravityAlignH` / `_MountExtrinsic` / `_level_tf_geometry`
读取；静态边另用**真的 `odom_level_tf.py` 节点**（`ros2 run`，参数 = 下面这些值）发一次 latched TF 取证。
仓库里的上游文件**一个字节没改**（见 31.2(a) 的 md5）。

| 量 | 默认（roll 30°） | scratch：roll 30°→**20°** | scratch：roll→**pitch** |
|---|---|---|---|
| 上游 `body_to_livox.rpy` | `-0.523598775598293 0 0` | `-0.349065850398866 0 0` | `0 -0.523598775598293 0` |
| 生成物 `<xacro:arg default>` | `-0.523598775598293 0 0` | `-0.349065850398866 0 0` | `0 -0.523598775598293 0` |
| `body_to_livox`（展开后） | xyz `0.000561701 0.130915824 0.15702817`、**rpy 0** | xyz **逐位相同**、rpy 0 | xyz **逐位相同**、rpy 0 |
| 插件 `<tilt_rpy>` | `-0.523598775598293 0 0` | `-0.349065850398866 0 0` | `0 -0.523598775598293 0` |
| `imu_joint` rpy | `-0.523598775598293 0 0` | `-0.349065850398866 0 0` | `0 -0.523598775598293 0` |
| `imu_joint` xyz（= R(rpy)·(0,0,−0.05)） | `0 -0.025 -0.04330127` | `0 -0.01710101 -0.04698463` | `0.025 0 -0.04330127` |
| 解析 `gravity_rpy_deg` | `[-30, 0, 0]` | **`[-20, 0, 0]`** | **`[0, -30, 0]`** |
| `h`（livox 离地） | 0.25952717 | 0.25952717（平移没动） | 0.25952717 |
| LIO `extrinsic_R`（行主序） | `[1,0,0, 0,0.866025,-0.5, 0,0.5,0.866025]` | **`[1,0,0, 0,0.939693,-0.342020, 0,0.342020,0.939693]`** | **`[0.866025,0,0.5, 0,1,0, -0.5,0,0.866025]`** |
| LIO `extrinsic_T` | `[0, 0, 0.05]` | `[0, 0, 0.05]`（不变） | `[0, 0, 0.05]`（不变） |
| adapter `rpy` / `xyz` | `[0.523599,0,0]` / `[-0.000561701,-0.034862345,-0.151448296]` | `[0.349066,0,0]` / `[-0.000561701,-0.069313837,-0.142334061]` | `[0,0.523599,0]` / `[-0.079000533,-0.130915824,-0.085709533]` |
| `odom_level` 静态边 `T_odom←odom_level` | xyz `[0,0.108112950,-0.187257122]` rpy `[30,0,0]`° | **xyz `[0,0.072693830,-0.199724655]` rpy `[20,0,0]`°** | **xyz `[-0.108112950,0,-0.187257122]` rpy `[0,30,0]`°** |
| `h_imu = h − (R_mount·t_IL).z` | 0.216225900 | **0.212542539** | 0.216225900 |

**真节点取证**（`ros2 run rm_nav_bringup odom_level_tf.py --ros-args -p gravity_rpy_deg:=… -p sensor_height:=0.25952717
-p extrinsic_t:="[0,0,0.05]" -p input_world:=tilted -p input_z:=sensor -p output_z:=ground -p z_origin_source:=urdf
-p check_ground:=false -p check_imu:=false`，私有 `ROS_DOMAIN_ID`/`HOME`，`timeout 12`）：

```text
默认:   R_mount=[-30,0,0]° ⇒ 发布值 T_odom←odom_level xyz=[0.000000000, 0.108112950, -0.187257122] rpy=[30.000000, -0.000000, 0.000000]°
20°:    R_mount=[-20,0,0]° ⇒ 发布值 T_odom←odom_level xyz=[0.000000000, 0.072693830, -0.199724655] rpy=[20.000000, -0.000000, 0.000000]°
pitch:  R_mount=[0,-30,0]° ⇒ 发布值 T_odom←odom_level xyz=[-0.108112950, 0.000000000, -0.187257122] rpy=[0.000000, 30.000000, 0.000000]°
```

⇒ **上游 rpy 一改，生成物、TF、插件、IMU 位姿、LIO 外参、静态边全跟着变**；仓库的上游/生成物
没有被 scratch 实验碰过（31.2(a)）。

### 31.4 nav 冒烟（默认 SPL 档：`lio:=small_point_lio` + `level_frame:=odom_level`）

**命令**（一次 bash 调用里起栈 + 采证 + 收尾；与 §28/§30 同口径）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  nav_rviz:=False lio_rviz:=False gui:=False \
  world:=RMUL2026 mode:=nav localization:=gicp lio:=small_point_lio nav:=mppi planner:=smac2d \
  robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True
```

隔离：私有 `HOME=.tmp_mountone/home-single-source/`、`ROS_DOMAIN_ID=129`、
`GAZEBO_MASTER_URI=http://127.0.0.1:12229`、`gui:=False`、`world:=RMUL2026` **不带前导 `--`**、
`setsid` + `tools/scripts/lib/scoped_cleanup.sh` + 硬超时（600 s），**无全机 pkill**；
一次只起一栈，收尾 `leftovers` 空；`free` 可用约 18 GiB。

| 判据 | 实测 | 判 |
|---|---|---|
| **局部 in-band** | `band_guard` **12 个样本全 100%**：772/772、782/782、783/783、773/773、765/765、770/770、773/773、778/778、772/772、778/778、780/780、766/766；`below=0 above=0`、`one-sided=no`、`band=[-0.050,2.000]`、`z p50≈0.108` | ✅ |
| **`h_meas`** | `h_urdf=0.2595 m h_meas=0.2532 m / \|Δ\|=0.0063 m < 0.03 ⇒ INFO`（inliers 2722、tilt 0.10°、resid p50 0.9 mm） | ✅ |
| **`odom_level→base_link` rpy** | 19 样本中位 **`[0.0116, 0.0449, -0.4133]°`**（roll −0.066…+0.043、pitch −0.021…+0.114、yaw −0.541…−0.294）⇒ 中位三轴均 <0.5°，roll/pitch 最大 <0.12° | ✅ |
| **静态边 `odom→odom_level`** | 活栈 `odom_level_tf` 发布 xyz `[0.000000000, 0.108112950, -0.187257122]` m、rpy `[30,-0,0]°`；`lio_tf_adapter` 取到 `T_odom_level←odom` xyz `[0,-0,0.216226]` rpy `[-30,0,0]°`（与发布值逐位相同） | ✅ |
| **LIO 外参** | `ros2 param get /small_point_lio extrinsic_R` = `[1.0, 0.0, 0.0, 0.0, 0.866025404, -0.5, 0.0, 0.5, 0.866025404]`；`extrinsic_T` = `[0,0,0.05]`；`publish_tf=False`、`odom_convention=sensor` | ✅ |
| **静态边参数** | `/odom_level_tf gravity_rpy_deg=[-30,0,0]`、`sensor_height=0.25952717`、`extrinsic_t=[0,0,0.05]`、`input_world=tilted`、`input_z=sensor`、`output_z=ground`、`z_origin_source=urdf` | ✅ |
| **几何帧** | `base_link→livox_frame` xyz `[0.000562,0.130916,0.157028]` rpy `[0,0,0]`；`base_link→imu_link` xyz `[0.000562,0.105916,0.113727]` rpy `[-30,0,0]`（= §4.12 约定） | ✅ |

> ⚠️ 默认档（`level_frame=odom_level`）下 `gravity_align_mode` 有效值 = `off`，E1 不进估计器 ⇒
> 活栈 `/small_point_lio gravity_align_rpy` 是**空数组**（节点默认），重力先验由 `odom_level_tf`
> 的 `gravity_rpy_deg=[-30,0,0]` 承担。这不是回归；§28 里读到 `[-30,0,0]` 是因为那时默认是 `hybrid`。

### 31.5 诚实清单

1. **nav 只跑了 1 次**（上述 `single-source`），没有 3 跑中位，也没有重跑 §25/§30 的完整矩阵；
   `odom_level→base_link` 的 yaw 中位 −0.41° 是单跑单窗，不能当长期漂移结论。
2. **决定性测试不是"整栈跑 20°"**：scratch 20°/pitch 只验证到"生成器 → 生成物 → launch 解析器 /
   `odom_level_tf` 节点"这一层；**没有**用 20° 的 xacro 起 Gazebo 跑 nav（那需要另一次全栈 + 世界）。
   静态边是**真的 `odom_level_tf` 节点**发的，但它的输入参数是探针按解析值喂的。
3. **`robot11 + plugin|urdf` 的 9 行表是"render 口径"**：launch 现在只接受 `sensor`（硬报错），
   这两行是绕过校验、直接给 xacro 的惰性 arg（见 §31.2(b) 注）。真 launch 只验证了 `sensor`。
4. **生成物 xacro 文件头的两句历史注释没改**（"默认 = SolidWorks CSV 的 roll 形式" /
   "plugin 档…"）：改注释会让 xacro 输出（含注释）md5 变化，与"逐字节不变"冲突。**值与来源以
   `<xacro:arg default>` 为准**（第 12 行）；要清理注释请另开一轮，并接受 `robot_description` md5 变化。
5. **没有在真机/非 Humble 上验证**；`--show-args` 与未知 launch arg 的行为只覆盖本机 Humble。
6. **没有重跑** GUI / `robot11_visual:=full` / `hzmirm` / 默认模型的 nav；本任务只动 robot11 的
   倾角来源，默认模型与 hzmirm 的 9 行 md5 已证明不变。
7. **`_robot11_tilt_rpy_from_xacro()` 读的是 install share 里的生成物**（本机为 symlink 到 src）：
   若只改了 src 没 `colcon build`/symlink-install，launch 会读到旧值；这是 install 语义，不是本改。
8. **nav 的活栈 `robot_description` 没有 dump**：§31.2 的 md5 是"与 launch 同一条 command"的
   xacro stdout；没有像 §28 那样在跑完后 `ros2 param dump /robot_state_publisher` 再比一次。
9. **离线工具 `tools/scripts/regress/robot11_self_mask.py` 仍带 30° 字面值**（`TILT_RPY` +
   `MIN_ELEV_DEG = 30 + 7.22`）：它用几何盲区公式算自击掩膜，**不在运行链上**（不喂
   `robot_description`/TF/LIO），本轮没改；若以后要在非 30° 安装上跑它，必须先改它的几何假设。
10. **`gz_sim.launch.py` 的独立路径没有实跑**：只删了它传的字面值（xacro 默认值 = 同一个字符串），
    没有用 `sim_backend:=gz` + `robot_description` 为空的方式起一次台架验证。

---

## §32 第二十二轮（**本机 date=2026-10-21，19:09–19:40**）：**出生即落定** —— 按模型给 spawn z，LIO 在静止的车上初始化（SPL 3 跑 + pointlio 3 跑 + fastlio 1+1 A/B）

> **本节只新增**（append-only）：§1–§31 一个字都没改。任务（用户批准）："能不能直接改机器人位置让
> 他直接在地面上" → 是。根因在 §30 已经量清：`get_world_config()` 的 `z=0.2` 对所有模型/world
> 共用一个常数 ⇒ robot11 出生自由落体 **9.75 cm**（~1.8 s），而 `pointlio` 0.5 s 就 `Set_init`
> （z 锚点低 6.5 cm，§30.5.2）、SPL 靠 §30 的落定门 + 初始化地图口径才兜住。本轮把"下落"从**出生
> 高度**上消掉（§4.13.5），门保留为兜底。
>
> 两个提交：**① 代码**（`feat(spawn): …` = 17e468d）、**② 文档**（本节 + 契约 §4.13.5）。

### 32.1 改了什么（file:line，提交 17e468d / 相对 HEAD 36ea60f）

| 文件 | 位置 | 改动 |
|---|---|---|
| `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py` | **178** | 新增 `_SpawnZ(Substitution)`：`spawn_z` 非空就用它、空 ⇒ 回落到 world 配置的 z；非浮点 ⇒ t=0 硬报错 |
| 同上 | **280** | 新增 `DeclareLaunchArgument('spawn_z', default_value='')`（**唯一新增的用户可见参数**） |
| 同上 | **396** | `spawn_entity.py` 的 `-z` 从 `world_config['z']` 改成 `_SpawnZ(world_config['z'])` |
| 同上 | **425** | `ld.add_action(declare_spawn_z_cmd)` |
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | **1215** | `_HZMI_SPAWN_Z_MARGIN_M = 0.001`（1 mm 接触余量） |
| 同上 | **1221** | `_HZMI_SPAWN_AT_SETTLED_WORLDS = ('RMUC2026', 'RMUL2026')`（floor z=0 的标定世界） |
| 同上 | **1224** | `_SpawnZForRobot(Substitution)`：`_HZMI_BASE_LINK_HEIGHT_M[robot] + 0.001`；未知 robot/world ⇒ `''` |
| 同上 | **2946** | `start_rm_simulation` 的 `launch_arguments` 加 `'spawn_z': _SpawnZForRobot()` |

**`get_world_config()` 的 `z` 一个字节没动**（仍是 0.2）。理由（写进代码注释）：它同时是
`rm_nav_bringup/scripts/map_asset_guard.py::WORLD_SPAWN` 的镜像源、以及既有存档 sidecar
`spawn_pose.z` 的比对基准 ⇒ 改它会让 `map/RMUL2026_v6.meta.yaml` 这类既有存档全部判成
`refused_spawn`。地图原点只取决于 x/y/yaw，z 只是出生高度。

### 32.2 默认路径证据（没有新增用户可见参数）

| 检查 | 结果 | 判 |
|---|---|---|
| `bringup_sim.launch.py --show-args`（HEAD vs 改后，私有 HOME） | **425 行 / 逐字节相同**（`diff` 空） | ✅ 没有新增/改动的 bringup 参数 |
| `rm_simulation.launch.py --show-args` | 只多 4 行 = **1 个参数** `spawn_z`（`default: ''`） | ✅ |
| `map_asset_guard.py spawn-table --check` | `✅ 出生点表与 rm_simulation.launch.py 一致（4 个 world）` | ✅ 既有存档的 spawn 比对基准没变 |
| 活栈 spawn 命令行（`spawn_cmd.txt`，7/8 跑；第 8 跑 = 旧 spawn 对照 f0，`-z 0.2`） | `spawn_entity.py -entity robot -topic robot_description -x 4.3 -y 3.35 **-z 0.103499** -Y 0.0 -timeout 60.0` | ✅ |

**`_SpawnZForRobot` 的取值**（in-process 单测，`LaunchContext` 直接 perform）：

| world \ robot | `robot11` | `hzmirm` | `''`（默认模型） |
|---|---|---|---|
| `RMUL2026` / `RMUC2026` | **0.103499** | **0.001000** | **0.061000** |
| `RMUC` / `RMUL`（2024） | `''`（= world 配置的 0.2） | `''` | `''` |

`_SpawnZ` 的回退/校验：`spawn_z=''` ⇒ 原值（0.2/0.061 都试过）；`'0.103499'` ⇒ 原样；
`'abc'` ⇒ `RuntimeError("[launch] spawn_z:='abc' 不是浮点数（米）…")`。

### 32.3 出生/落定 trace（GT + 原始云，7 跑同口径）

GT = `/odom_ground_truth`（`gazebo_ros_planar_move` 把**模型世界位姿**原样发出来，100 Hz；
探针 `trace_probe.py` 只订阅）。时间轴 = 首帧原始云的仿真戳（t=0）。

| run | LIO | spawn `-z` | GT z 首帧(t≈+0.1 s) | GT max | GT min | GT 稳态 | 原始云 d 首帧 | d 稳态 | **d 落差** | LIO 首个 `/odom` |
|---|---|---|---|---|---|---|---|---|---|---|
| `n1_spl_spawnz` | SPL | 0.103499 | **0.102499** | **0.102499** | **0.102490** | 0.102498 | 0.258868 | 0.259573 | **−0.71 mm** | 2.00 s |
| `n2_spl_spawnz` | SPL | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258860 | 0.259550 | −0.69 mm | 2.00 s |
| `n3_spl_spawnz` | SPL | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258857 | 0.259545 | −0.69 mm | 2.00 s |
| `p1_pointlio_spawnz` | pointlio | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258859 | 0.259552 | −0.69 mm | **0.50 s** |
| `p2_pointlio_spawnz` | pointlio | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258871 | 0.259552 | −0.68 mm | 0.50 s |
| `p3_pointlio_spawnz` | pointlio | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258871 | 0.259523 | −0.65 mm | 0.50 s |
| `f1_fastlio_spawnz` | fastlio | 0.103499 | 0.102499 | 0.102499 | 0.102490 | 0.102498 | 0.258870 | 0.259544 | −0.67 mm | **0.30 s** |
| `f0_fastlio_oldspawn`（**对照：旧 spawn**） | fastlio | **0.200000** | **0.194708** | 0.194708 | 0.102489 | 0.102499 | 0.351498 | 0.259530 | **−91.97 mm** | 0.30 s |

**读法（三条独立证据一致）**

1. **drop = 1.000 mm 名义**：`spawn -z − GT = 0.103499 − 0.102499 = 1.000 mm`——正好是那 1 mm
   余量；GT 的**第一帧**（首帧云后 0.1 s）已经在 0.102499，之后整跑 max−min = **9 µm**。
   ⇒ **落定时间 ≈ 0**（在 GT 开始发布之前就落完了）。对照 f0：GT 从 0.194708 掉到 0.102489
   = **92.2 mm**（≈ 0.2 − 0.1025 − 首帧已掉 5 mm）。
2. **原始云的地面垂距 d**（物理量，与 LIO 无关，§30 同一口径）：新 spawn 的 7 跑都是
   首帧 **比稳态低 0.65–0.71 mm**（接触解算的瞬时过冲，随后弹回），**不是**从高处落下；
   f0 是首帧 0.351498 → 稳态 0.259530（**91.97 mm**）。
3. **GT xy/yaw 与旧 spawn 逐位一致**（物理落定到同一个姿态）：`gz model -m robot -p` 在收尾
   采样 = `4.30716 3.34599 0.102498 0.000296 5e-06 -0.015567`（n1–n3 逐位相同）、
   pointlio `4.30687 3.34615 0.1024xx`、fastlio `4.30681 3.34617`；f0（旧 spawn）也是
   `4.30618 3.34619 0.102498` ⇒ 换 spawn 高度**没有**改变落定姿态（差异 ≤1 mm），只是**跳过**
   了下落过程。⚠️ 落定后的 xy/yaw 相对名义 spawn 偏了 (7 mm, −4 mm, −0.89°)——这是接触收敛
   的既有行为（f0 同值），与本轮改动无关。

**SPL 落定门收据**（3 跑逐字）：

```text
初始化落定门：**已落定**（首帧后 0.84–0.89 s；最近 1.00 s 内地面高度代理峰峰值 0.0000 m <= tol 0.0050 m，8 帧）
           ⇒ 放行初始化；落定前丢弃的 9 帧点云不进初始化地图
初始化（init）：t=…（首帧后 1.990–1.992 s）| 落定门=on（hold=1.00 s，峰峰值=0.0000 m，丢弃 9 帧）
           | 初始化地图 = 90212 / 90209 / 90212 点 | z 原点 = **初始化时刻的 IMU 位姿**
```

⇒ **门在 0.84–0.89 s 就放行，而 SPL 的 200 帧 IMU 初始化门在 2.0 s ⇒ 门不推迟任何一帧**
（3 跑 init 都是 1.990–1.992 s，与 §30 的 2.0 s 相同）。代价：初始化地图从 §30 的 **1 帧**
（8120 点，因为那时门 2.6 s 才放行、正好等于 init 时刻）变成 **~11 帧**（90209–90212 点，
0.89→2.0 s）——**全部是落定后的静止帧**，不再含下沉段（I15′）。

### 32.4 z 锚点：SPL 3 跑 + pointlio 3 跑

**SPL（`lio:=small_point_lio`，默认档；`level_probe.py`，settle 20 / window 20）**

| run | `floor_z`（`odom_level`） | `base_link` z | `h_meas`（`gravity_align_h_check`） | Δ vs `h_urdf`=0.259527 | tilt | `odom_level→base_link` rpy（中位） | 静态边 `odom→odom_level` | `z_origin_source` |
|---|---|---|---|---|---|---|---|---|
| `n1_spl_spawnz` | **+0.0023** | **+0.1032** | **0.2596** | **0.1 mm（INFO）** | 0.204° | `[0.184, 0.153, −0.301]°` | `xyz=[0,0,0.216226] rpy=[−30,0,0]°` | `urdf` |
| `n2_spl_spawnz` | **+0.0021** | **+0.1020** | **0.2600** | **0.5 mm（INFO）** | 0.224° | `[0.215, 0.105, −0.389]°` | 逐位相同 | `urdf` |
| `n3_spl_spawnz` | **+0.0007** | **+0.1031** | **0.2587** | **0.8 mm（INFO）** | 0.268° | `[0.167, 0.181, −0.308]°` | 逐位相同 | `urdf` |
| **3 跑 spread** | **1.6 mm** | **1.2 mm** | **1.3 mm** | — | — | roll/pitch ≤ **0.21°**（<0.5°） | **0** | — |

* 判据（契约 I10′/I10″）：`floor_z ∈ 0 ± 0.01` ✅、`base_link z = +0.1025 ± 0.01` ✅
  （+0.7/−0.5/+0.6 mm）、`h_meas` **INFO** ✅（Δ ≤0.8 mm；§30 是 4.4–5.9 mm INFO）、
  3 跑 spread < 1 cm ✅（1.2–1.6 mm）。
* 与 §30 修后对比：`floor_z` 0.0053/0.0058/0.0056 → **0.0023/0.0021/0.0007**；`base_link z`
  0.1081/0.1080/0.1077（+5.5 mm 系统偏置）→ **0.1032/0.1020/0.1031（≤0.7 mm）**；
  `h_meas` 0.2536–0.2551 → **0.2587–0.2600**。⇒ 出生下落就是 §30 那条 "+5.5 mm 残余"的
  主要来源，去掉下落之后它基本归零。

**pointlio（`lio:=pointlio`；`input_world=level`，静态边 = I/(0,0,0.216226)）**

⚠️ pointlio 的 `/cloud_registered` 只有 **139–147 点/帧**（§30.5.2 同款）⇒ `level_probe` 的
≥500 点平面拟合与 `odom_level_tf` 的 `gravity_align_h_check` **都拿不到 h_meas**。地面 z 用
**换算值**：`floor_z = base_link z(TF) − GT z`（GT = 0.102499 = 物理 base_link 高；原始云 d
的稳态值 0.25952–0.25955 与 h_urdf 0.259527 差 ≤0.03 mm，证明车确实停在几何落定高度）。

| run | `base_link` z（TF 中位） | **换算 `floor_z`** | rpy（`odom_level→base_link`） | 静态边 | `z_origin_source` | 局部 lethal / nearest / ahead |
|---|---|---|---|---|---|---|
| `p1_pointlio_spawnz` | **+0.0961** | **−6.4 mm** | `[−0.013, +0.096, −0.301]°` | `xyz=[0,0,0.216226] rpy=[0,0,0]°` | `urdf` | 227 / 0.7035 / 18 |
| `p2_pointlio_spawnz` | **+0.0964** | **−6.1 mm** | `[−0.019, −0.063, −0.309]°` | 逐位相同 | `urdf` | 332 / 0.7091 / 20 |
| `p3_pointlio_spawnz` | **+0.0974** | **−5.1 mm** | `[+0.004, +0.019, −0.270]°` | 逐位相同 | `urdf` | 213 / 0.7051 / 18 |
| **3 跑 spread** | **1.3 mm** | **1.3 mm** | roll/pitch ≤ **0.10°** | 0 | — | — |

* **§30 的 6.5 cm 偏差消失**：§30.5.2 的 `base_link z = +0.0298/+0.0362/+0.0356`（换算地面
  −7.3/−6.6/−6.7 cm）→ 本轮 **+0.0961/+0.0964/+0.0974（地面 −6.4/−6.1/−5.1 mm）**，
  全部落在判据 `0 ± 1 cm` 内 ✅。跑间 spread 1.3 mm（§30 是 6.4 mm）。
* 系统性差：pointlio 比 URDF 值**低 5–6 mm**（SPL 是 ±0.7 mm）——它自己的 `Set_init` 之后
  state z 稳定在 −3.8/−9.6/−6.1 mm（trace），这一段属于它自己的估计器，不是静态边。
* ⚠️ `odom_level_tf` 在这 3 跑都打了一条 **WARN**：`gravity_align_h_check: 91 s 内没拿到
  /cloud_registered 的可用点云 ⇒ 跳过互检；静态边改用 URDF 值发布`（loud，不静默）。
  对 `z_origin_source=urdf` 而言这正是要的行为（定基永远用 URDF）。

### 32.5 pointlio 的 **1/3 静默空局部图**：本轮 0/3，未复现；TF 时序脆弱性有量化

§30.5.2 的 `p2_pointlio` 整张局部图空（lethal 0 / free 62500，`/scan` 与 `band_guard` 都 100%、
nav2 一条 `Message Filter dropping` 都没有）。本轮 3 跑**都没有复现**：

| run | local lethal | inscribed | free | n_cells | 局部图判 |
|---|---|---|---|---|---|
| `p1_pointlio_spawnz` | **227** | 8940 | 43231 | 62500 | 非空 ✅ |
| `p2_pointlio_spawnz` | **332** | 9680 | 40654 | 62500 | 非空 ✅ |
| `p3_pointlio_spawnz` | **213** | 8920 | 43283 | 62500 | 非空 ✅ |

**TF 时序取证**（新探针 `.tmp_spawnz/tf_lag_probe.py`，只订阅 `/tf` + `/scan`；p1–p3 都开了）：

| run | adapter TF 帧数 | `/scan` 帧数 | 到达时 TF **已覆盖**扫描戳 | 需要 ≤1.5 s 追帧 | 窗口内没追上 | `scan 戳 − 最新 TF 戳` |
|---|---|---|---|---|---|---|
| `p1` | 154 | 160 | **51** | 108 | 1 | p50 **0.0 s**、max **0.1 s** |
| `p2` | 162 | 168 | **57** | 109 | 2 | p50 0.0 s、max 0.1 s |
| `p3` | 163 | 169 | **59** | 108 | 2 | p50 0.0 s、max 0.1 s |

⇒ adapter 的 TF 与 `/scan` 同频（~10 Hz 仿真），但**只有约 1/3 的扫描在到达时 TF 已覆盖它的
戳**；其余 2/3 要靠 nav2 MessageFilter 的队列等 **≤0.1 s**（在 1 s 容忍内，所以通常不丢）。
"没追上"的 1–2 帧都在**探针窗口末尾**（后面的 TF 还没到），不是真丢。
`tf_timing.txt` 里 3 条 `Timed out waiting for transform … base_link_fake → map` 全在启动段
（`map→odom` 还没发），与本轮无关；**没有** `Message Filter dropping` / `extrapolation`。

**结论**：① 本轮 3/3 正常，§30 的 1/3 静默空**没有复现**（它本来就与出生下落无关——那个 arm
里没有 SPL 节点）；② 但"TF 只覆盖 1/3 扫描、其余要追 0.1 s"的时序脆弱性**仍在**，
按任务要求**只诊断、不修**。要修得单独查 `lio_tf_adapter` 的发布时机（是否等点云处理完才发）。

### 32.6 nav 回归：SPL 3 跑 vs §30 修后（同机同时段）

| 指标 | §30 修后（n1–n3，出生仍下落） | **本轮（n1–n3，出生即落定）** | 判读 |
|---|---|---|---|
| local lethal（中位） | 275 / 249 / 255 | **250 / 250 / 247** | 低 0–2%，仍在 §25/§26 的 250–330 档 ✅ |
| local nearest lethal | 0.7127 / 0.7124 / 0.7133 | **0.7059 / 0.7090 / 0.7064** | 近 4–7 mm（扫描面低 5 mm，见下） |
| local ahead（2.0×0.6 m） | 22 / 24 / 22 | **22 / 23 / 22** | 同档 ✅ |
| local inscribed / free | 10159–10510 / 40361–40509 | **10260–10294 / 40263–40398** | 同档（±0.5%）✅ |
| in-band（`band_guard`，读**生效**带 [−0.05,2.0]） | 100%（766–772 帧） | **100%**（每样本 766–785 帧，全 below=0 above=0） | ✅ |
| 扫描面 z（代价图帧） | +0.1077…+0.1081 | **+0.1033 / +0.1024 / +0.1032** | = `base_link` z，比 §30 低 5 mm（§30 那 5 mm 是残余偏置） |
| global lethal（map 帧，stvl） | 2160 / 2160 / 2160 | **2164 / 2163 / 2164** | +3–4 格（0.15%，位姿差；不是 z 锚点） |
| global nearest | 0.7354–0.7368 | **0.7336 / 0.7340 / 0.7336** | −2 mm |
| `h_meas` | 0.2536–0.2551 INFO | **0.2587–0.2600 INFO** | **变好**（Δ 4–6 mm → 0.1–0.8 mm）✅ |

⇒ **没有功能退化**：三个"非空/最近/前方"判据同档、in-band 100%、全局图逐格同档。唯一的系统性
变化是扫描面从 +0.108 下移到 +0.103（= 修掉 §30 那 5.5 mm 残差）；local nearest 因此近了
4–7 mm——仍在任务给的 "~0.71" 口径内。

### 32.7 fastlio 回归（1+1 A/B：旧 spawn vs 新 spawn）

> ⚠️ **2026-10-27 更新（§36）**：本节 fastlio 数字全部是 **`R=I`（外参死键）时代**量的
> （§35.9#1）；`cdcd7e3` 修复后已按 §34.2.1 协议重跑 **3 跑 after + 2 跑 before** ⇒
> **已被 §36 取代**：nav 数字在跑间波动内，结构性变化 = `odom_level` 地图地面
> **28.4–29.3° → 1.1–1.9°**。下面内容保留不改，只作历史对照。

fastlio **没有** z 锚点逻辑（`input_world=tilted`，静态边照旧），但出生高度会影响它的初始状态。

| 量 | `f0_fastlio_oldspawn`（`-z 0.2`） | `f1_fastlio_spawnz`（`-z 0.103499`） |
|---|---|---|
| GT z 首帧 / 稳态 | **0.194708** / 0.102499 | **0.102499** / 0.102498 |
| 原始云 d 落定落差 | **91.97 mm** | **0.67 mm** |
| LIO 首个 `/odom` | 0.30 s | 0.30 s |
| state z（稳定） | −0.0761 | −0.0593 |
| `base_link z`（`odom_level`） | 0.0599 | **0.0409** |
| 扫描面 z | 0.0541 | 0.0517 |
| `odom_level→base_link` rpy | `[0.60, 0.77, −0.72]°` | `[1.86, −1.63, −0.15]°` |
| **in-band（生效带 [−0.05,2.0]）** | **100%（780/780；1 个样本 770/771=99.9%）** | **100%（772/772…）** |
| in-band（`nav_band_probe` 自己的 [0,2] 带） | 752/771（below=20） | 706/773（below=65）← **口径不是生效带，别用** |
| local lethal / nearest / ahead | 328.5 / 0.7011 / 24 | 404 / 0.7089 / 35 |
| global lethal / nearest | 2141 / 0.7457 | 1970 / 0.7696 |
| `gravity_align_h_check`（h_meas） | 不可用（`/cloud_registered` ~145 点/帧 < 门槛） | 不可用（同） |

**判读（诚实）**：① 出生下落被消掉（GT/原始云两条证据）；② 但 fastlio 自己的 z 锚点**不在契约
范围**（没有 E1、没有 z 原点逻辑，静态边用 `tilted` 输入）——它落定后的 state z 从 −0.076 抬到
−0.059，`base_link z` 反而低 1.9 cm、local lethal 328.5→404；**1+1 跑分不出**是 spawn 改动还是
fastlio 自己的姿态漂移（两跑的 rpy 0.6°/0.8° vs 1.9°/−1.6° 差得多，而它本来就没有重力对齐）。
③ 生效带的 in-band 两跑都是 100%——探针的 65/20 个 "below" 是它自己的 [0,2] 带造成的假象
（见 §32.9#9）。⇒ 结论：fastlio 臂**仍可用**，但它的姿态/局部数字不是本轮的验收面；要判断
spawn 对它有没有副作用需要 3+3 跑（未做）。

### 32.8 落定门为什么保留 1.0 s（不改成 0）

| 选项 | 实测 | 判 |
|---|---|---|
| `lio_init_settle_s=1.0`（默认，**保留**） | 门在首帧后 **0.84–0.89 s** 放行（峰峰值 0.0000 m）；SPL init 仍在 **1.990–1.992 s**（200 帧 IMU 门）⇒ **不推迟任何一帧** | ✅ 保留 |
| `=0`（关） | 本轮场景下 init 时刻不变（2.0 s 由 IMU 门决定），但失去"出生高度不对/地面不平/车被抬起"时的兜底（15 s 超时 + 响亮 WARN） | ✗ 不采纳 |

⇒ 门从"修 z 锚点的主力"降为**兜底**（契约 §4.13.5）。它的存在**不延迟**正常跑：放行时刻
（0.89 s）早于 IMU 初始化门（2.0 s）；这一点 3 跑 SPL 的 init 时刻（1.990/1.991/1.992 s）与
§30（2.0 s）逐位一致。

### 32.9 诚实清单

1. **只在 `robot:=robot11` + `world:=RMUL2026` + `mode:=nav` 上跑了全栈**（SPL 3 / pointlio 3 /
   fastlio 1+1）。`hzmirm`（0.001）与默认模型（0.061）的 spawn_z 只有 **in-process 单测**，
   没有起 Gazebo；`world:=RMUC2026` 的 spawn_z 走同一段代码（0.103499），本轮**没实跑**。
2. **默认模型的轮胎下沉 ~9 mm 不在表里**（表是 URDF 几何：`radius_wheel=0.06`）⇒ 它实际仍会
   落 ~10 mm。这是"表 = `h` 的来源"这一约束的代价；要修得给 spawn 单独一张"含下沉"的表（未做）。
3. **1 mm 余量在第一个物理步就被吃掉**：GT 的第一帧（首帧云后 0.1 s）已经在 0.102499，
   整跑 max−min = **9 µm**；原始云首帧反而比稳态**低** 0.65–0.71 mm（接触解算的瞬时过冲）。
   所以"drop < 1 mm"里的 1 mm 是**名义**余量，观测到的位移是 9 µm。
4. **初始化地图从 1 帧（8120 点）涨到 ~11 帧（90209–90212 点）**：门 0.89 s 放行、init 仍等
   200 帧 IMU（2.0 s）。全部是**落定后的静止帧**（丢弃的 9 帧也是落定前的静止帧，不是下沉帧），
   所以 z 锚点更好（h_meas Δ 0.1 mm），但 I15 的"M ≲ 一帧"数值口径要按 I15′ 重新读。
   **没有**去改 `small_point_lio` 让门放行后立刻 init（那属于 `src/rm_localization/**`，本轮禁改）。
5. **pointlio/fastlio 的 `h_meas` 不可用**：两者的 `/cloud_registered` 只有 ~140 点/帧，
   `odom_level_tf` 的 ≥500 点地面拟合跳检（WARN，loud）。替代证据 = 原始云 d（与 h_urdf 差
   ≤0.03 mm）+ GT 0.102499 的换算地面。**没有**放宽那个门槛（会改 `rm_nav_bringup/scripts`）。
6. **pointlio 的 1/3 静默空局部图本轮 0/3**：没有复现 ≠ 已修。TF 时序量化见 §32.5（TF 只覆盖
   1/3 扫描、其余追 ≤0.1 s）。**只诊断、没修**（任务要求）。
7. **fastlio 只有 1+1 跑**，且两跑姿态差得多（它没有重力对齐）⇒ §32.7 的 base_z/local lethal
   差异**不能**归因于 spawn 改动；它的 in-band（生效带）两跑都是 100%。
8. **`sim_backend:=gz` 没测**：Fortress 的 spawn 路径在 `gz_sim.launch.py`（模型 pose 在世界
   SDF 里），本轮没改也没跑。
9. **`nav_band_probe.py` 仍用自己的 band `[0.0, 2.0]`**（不是代价图的生效带 `[-0.05, 2.0]`）：
   对 SPL（扫描面 +0.103）两档同值，对 fastlio（扫描面 +0.052、z 摆到 −0.03）会误报
   "below=65"。**in-band 的权威口径 = `band_guard`**（读活栈参数服务）。这条是既有工具的
   口径问题，本轮**没改**它（避免动 §17/§26/§30 的对比基线），只在表里注明。
10. **跑间噪声**：同机同时段、RTF ≈0.37（点云/导航叠满），local lethal 的 ±10% 级差异分不出
    位姿差与机器噪声（§22.8 / §30.8#7）。
11. **`p1_pointlio_spawnz` 重跑过一次**：第一次跑（19:21–19:27）本身是好的，但我在它运行中
    改了 harness 脚本（加 TF lag 探针）⇒ bash 读到半截文件、收尾几步没执行（缺 params/tf/
    log_grep）。表里的 p1 是 19:33 的**干净重跑**（launch/探针参数逐字相同）。
12. **第一条 `s1_spl_spawnz` 作废**：启动段用 `gz model -m robot -p` 高频轮询把 gzserver 打崩
    （`boost::shared_ptr<Connection>` assertion，exit −6；§30 的 14 个旧 run 都是 assert=0）。
    这是**测量装置**的锅，不是 spawn 改动的锅：改成"只在收尾取一次"后，本轮 8/8 跑
    assert=0、`sc_own_leftovers_desc` 全空。记在这里，避免以后再用启动段 `gz model` 轮询。
13. **`spawn_entity.py` 的 "Entity pushed to spawn queue, but spawn service timed out"**
    在 14/14 旧 run 与 8/8 新 run 都出现（实体其实都生成成功）——**既有良性消息**，与本轮无关。
14. **没碰**地图资产（map/、PCD/、世界文件）、`src/rm_localization/**`、`get_world_config()`；
    `map_asset_guard.py spawn-table --check` 通过；`spawn_pose.z` 在既有 sidecar 里仍是 0.2
    （这是**有意的**：地图原点不含 z）。

## §33 第二十三轮（**本机 date=2026-10-21，21:08–21:44**）：`gravity:=tf|base|lio|raw` 收口 —— 一个开关代替重力/水平帧参数组；4 臂各 1 跑

> **本节只新增**（append-only）：§1–§32 一个字都没改。任务（用户批准）："123都可以感觉 raw 也可以留着
> 可以学 cod 以 baselink 为基准"。交付 = **§4.14**（契约）＋本节（实测）。默认档 `gravity:=tf`
> 的行为与改动前**逐字段相同**（除了下面**有意**删掉的 3 处参数/初始化行为）。
>
> **一句话结论**：`tf` = 局部 in-band **100%**、`odom_level→base_link` rpy **<0.5°**、
> `odom→base_link` **+30°**；`base` = 没有 `odom_level` 帧、局部帧 `odom`、障碍层**强制 cloud**、
> `band_guard` **IDLE**、`/scan` 在 `base_link`；`lio` = E1 hybrid、局部 in-band **100%**、
> `odom→base_link ≈0°`；`raw` = **43.7%**、kept-y **单侧 +y**（§25 的 off 复现）。
> 6 个旧参数 + 2 个非法组合全部 **t=0 exit 1**；`--show-args` **exit 0**、只多 1 个 `gravity`；
> `robot_description` 9 个组合 **md5 逐字节相同**。

### 33.1 改了什么（file:line，提交见文末）

| 文件 | 位置 | 改动 |
|---|---|---|
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | 854–979 | 新增 `_gravity` / `_Gravity` / `_level_frame`（派生：tf ⇒ odom_level，其余 ⇒ odom）/ `_launch_arg_was_given`；删掉 `_LEVEL_FRAMES` / `_LEVEL_FRAME_DEFAULT` 与旧 `level_frame` 的读取 |
| 同上 | 914–973 | 新增 `_effective_local_obstacle` / `_EffectiveLocalObstacle`：`base` 档强制逐点层（显式 scan ⇒ 硬报错） |
| 同上 | 1042–1065 | `_effective_gravity_align_mode` 重写：`lio ⇒ hybrid`（非 SPL 硬报错），其余 `off`；旧 `auto/urdf/imu` 不再存在 |
| 同上 | 1107–1121 | 6 个旧参数 = `_LEGACY_GRAVITY_ALIASES`（非默认值 ⇒ t=0 报错，消息指向 `gravity`） |
| 同上 | 1123–1190 | `_check_gravity_contract`：别名检查 + base 障碍层检查 + tf 回退 WARN + `[gravity]`/`[odom_level]`/`[gravity_align]` 收据 |
| 同上 | 2556–2696 | 声明 `gravity`（choices 4 个）；6 个旧声明改成 `[已删除 / 硬报错别名]`（默认值保持旧值） |
| 同上 | 3022–3038 | **第二份互斥 linefit**：`gravity:=base` ⇒ `gravity_aligned_frame: base_link`；默认档参数表逐字段不变 |
| 同上 | ~3150–3300 | SPL 收口成**两份** Node：`eff==off`（tf/base/raw，`publish_tf=false + sensor`、`init_settle_hold=0.0`、`init_map_extrinsic=True`、`gravity_align_h` 诊断）与 `eff!=off`（lio，再加 E1/E1b，`z_origin` 固定 `ground`）。旧的"off+odom 由 LIO 自己发 TF"那份**删除** |
| 同上 | 3649–3680 | `odom_level_tf` 只在 `gravity:=tf` 启动；`z_origin_source` 固定 `'urdf'` |
| 同上 | ~3210 / 3260 / 3280 | 三处 SPL 参数表硬编码 `init_settle_hold: 0.0` / `init_map_extrinsic: True` /（E1 档）`gravity_align_z_origin: 'ground'` |
| 同上 | 4076 | nav2 include 的 `local_obstacle` 改传**有效值**；`scan_band_min` 覆盖层同样读有效值 |
| 同上 | 4088–4100 / 4209 | LD 声明顺序：`gravity` 在最前；旧的 `_GravityAlignResolution` 收据与 `_check_level_frame_contract` 删除；新增 `_check_gravity_contract`（在 `declare_local_obstacle` 之后） |
| `src/rm_localization/small_point_lio/src/small_point_lio/small_point_lio.cpp` | 200 / 347 / 416 / 432–449 / 450–500 | 初始化互检用的 `R_ws_check`（E1 关时 = `R_IL^T`）；初始化地图 `init_map_extrinsic` 的 `R_IL=I` 快路径（`p + t_IL`，与一般式逐位相同）；**初始化地面互检**从 E1 的 z-origin 分支搬出来、与 E1 无关：`gravity_align_h>0` 就拟合第一帧地面，`|Δ|≥3 cm ⇒ 响亮 WARN（"出生未落定/地面不平/悬空"）`、`≥5 cm ⇒ ERROR`，**定基永远用 URDF 值** |

### 33.2 协议、隔离与冻结证据

* 命令（4 跑同口径，与 §25/§29/§32 逐字同）：`world:=RMUL2026 mode:=nav lio:=small_point_lio
  localization:=gicp nav:=mppi planner:=smac2d robot:=robot11 robot11_mount:=sensor spin_speed:=0.0
  band_guard:=True` + `gravity:=tf|base|lio|raw`；`SETTLE=20 WINDOW=20`（**仿真秒**）、`gui:=False`、
  `nav_rviz:=False`、`lio_rviz:=False`。
* 隔离：私有 `HOME` / `ROS_LOG_DIR` / `ROS_DOMAIN_ID` / 先探空端口再占 `GAZEBO_MASTER_URI`、
  `unset DISPLAY`、`setsid` 起栈（进程组 = 本 run）、硬超时 900 s、收尾只用
  `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）、**一栈一跑**（4 跑顺序，跑间 8 s）。
* 冻结：4 跑 `meta_before/after` 的 launch md5 相同 = **`6ad0aaaedc18486dc8318ce235c7b738`**；
  `meta_md5_diff.txt` **0 行**（4/4）；`git_head = 2022f0a`；`build/small_point_lio/libsmall_point_lio.so`
  = `7a15156a960a406fe0acdd152544cc54`（本轮重建）。
* 探针：`tools/scripts/tiltmount/nav_band_probe.py`（**本轮加了 `--scan-frame`**：tf 用 `odom_level`，
  base/lio/raw 用 `odom` —— 它们的代价图帧就是 `odom`）、`.tmp_gravtest/grav_probe.py`、
  `tools/scripts/tiltmount/uc_procstats.py`；运行期 `ros2 param get` 与 `tf2_echo` 在探针结束后回读。
  原始数据：`.tmp_ab/g3_{tf,base,lio,raw}_r1/`（tf 的最终跑是 `.tmp_ab/g4_tf_r1/`，原因见 §33.8）。

### 33.3 主表：4 臂各 1 跑（窗口 20 仿真秒；`median`）

| 指标 | **`gravity:=tf`**（默认） | **`gravity:=base`** | **`gravity:=lio`** | **`gravity:=raw`** |
|---|---|---|---|---|
| 局部代价图帧（live） | **`odom_level`** | **`odom`** | `odom` | `odom` |
| `odom_level` 帧/静态边 | **有**（`odom→odom_level` = rpy [30.000, 0, 0]°、xyz [0, 0.108, −0.187] m，两次采样逐位相同） | **没有**（`tf2_echo odom odom_level` 空） | **没有** | **没有** |
| `odom_level_tf` 节点 | **在** | 不在 | 不在 | 不在 |
| `odom→base_link` rpy | **[30.167, 0.863, −1.055]°** | **[30.144, 0.792, −1.024]°** | **[0.394, −0.183, −1.021]°** | **[30.168, 0.788, −1.040]°** |
| `odom_level→base_link` rpy | **[0.169, 0.144, −1.379]°**（roll/pitch <0.5 ✓） | —（无此帧） | — | — |
| 估计器 | E1 **off**；`publish_tf=false`、`odom_convention=sensor` | 同左 | **E1 hybrid**（Δ(URDF,IMU)=0.015° ⇒ 用 URDF）；`publish_tf=false`、`sensor`、`z_origin=ground` | E1 off；`publish_tf=false`、`sensor` |
| 局部障碍层（live） | `obstacle_layer=true`（2D scan） | **`obstacle_cloud_layer=true`、`obstacle_layer=false`**（强制 cloud；launch t=0 WARN） | `obstacle_layer=true` | `obstacle_layer=true`、`obstacle_cloud_layer=false` |
| `band_guard`（local） | **OK 769/769 = 100%** | **IDLE**（没有启用的 2D scan 高度带 ✓） | **OK 770/770 = 100%** | **WARN 43.7%**（335/767；另一采样 44.6%）；kept-y **单侧 +y** `[0.134, 2.865]`（all-y `[−0.871, 4.423]`） |
| `/scan` 有限波束 | 773 | 773 | 774 | 773 |
| 在带 `[0,2]`（探针口径） | **773/773（100%）** | 334/773（43.2%；**不是本档判据**） | **774/774（100%）** | **334/773（43.2%）** |
| 带下 / 带上 | 0 / 0 | 437 / 3 | 0 / 0 | 436.5 / 3 |
| 扫描面 z p50（代价图帧） | **+0.1032** | **−0.1008** | **+0.1016** | **−0.1014** |
| z [min, max] | [+0.082, +0.120] | [−0.451, +2.440] | [+0.091, +0.142] | [−0.451, +2.440] |
| local **lethal** | **301** | **434** | **291.5** | **312** |
| local inscribed / free | 10161 / 40404 | 12868.5 / 38323.5 | 10253 / 40397.5 | 9994 / 41672.5 |
| local **最近 lethal** | **0.704 m** | **0.697 m** | **0.706 m** | **0.745 m** |
| local **前方 2.0×1.2 m lethal** | **24** | **61** | **23** | **21** |
| global lethal / 最近 / ahead | 2164 / 0.728 / 43 | 2165 / 0.715 / 45 | 2165 / 0.701 / 56 | 2161 / 0.714 / 45 |
| `h_meas`（SPL 初始化，判级） | **0.2597 m / Δ 0.0002 / INFO** | **0.2594 / Δ 0.0002 / INFO** | **0.2597 / Δ 0.0002 / INFO** | **0.2597 / Δ 0.0002 / INFO** |
| `h_meas`（`odom_level_tf`，判级） | **0.2588 / Δ 0.0007 / INFO**（地面 z=+0.0006、tilt 0.30°） | — | — | — |

**读法**

1. **tf 与 raw 只差"水平帧"**：两者的 `odom` 是同一个原始 IMU 世界（`odom→base_link` 都 ≈30°），
   唯一差异 = tf 的局部代价图在 `odom_level`（静态边把 30° 摆正）⇒ in-band 100%；raw 的代价图
   在 `odom` ⇒ 斜盘面 ⇒ **43.2–44.6%** + kept-y 单侧（与 §25 的 off 43.1–43.4% /
   `[+0.229,+3.281]` 同一现象、同一量级）。**raw 的 lethal 312 / 最近 0.745 / ahead 21**
   也与 §25 off 的 **299.5 / 0.751 / 20** 对得上（跨轮 ±5%，不是同一次实验）。
2. **base 与 raw 的"代价"不同**：base 的局部障碍层被强制成 **cloud**（逐点），所以尽管扫描面
   仍在斜 `odom` 里只有 43.2% 在带，`band_guard` 对 local 是 **IDLE**（没有 2D scan 带可评）——
   这正是设计要的"不静默半盲"。cloud 层看到的 lethal 比 2D scan 多（**434 vs 312**、
   前方 **61 vs 21**、最近 **0.697 vs 0.745**），因为 3D 点云在斜 `odom` 里**有逐点高度**
   （不会被平盘的单一 z 整批丢掉），代价是把同一面墙投成更多格。**base 不是 "tf 的等价物"**，
   它是"以 base_link 为高度语义、逐点建图"的另一种合法形态。
3. **lio 是唯一让估计器动 `odom` 的档**：`odom→base_link` rpy ≈ **[0.394, −0.183, −1.021]°**
   （≈0 ✓），局部 in-band 100%，lethal/最近/ahead **291.5 / 0.706 / 23** 与 tf（301 / 0.704 / 24）
   同量级 —— 与 §25 的 hybrid 臂（329/0.712/22）同源；区别只在"谁把 30° 吸收掉"
   （估计器 vs 静态边），nav 观测量不可区分。

### 33.4 默认档（`gravity:=tf`）的逐字节证据

**(a) `--show-args`**：exit **0**；改动前/后 **425 / 429 行**、**106 / 107 个参数**；
逐参数比对：**只多 `gravity`**，**没有任何默认值被改**；6 个旧名字仍在（description =
`[已删除 / 硬报错别名]`，默认值 = 旧值）。⇒ 有效重力/水平帧面 = **−4/+1**（4 个旧参数收成
1 个），初始化面 = **−2**（行为硬编码）；别名仍可被发现。

**(b) `robot_description`（9 个 (robot, mount) 组合，与 launch 同一条 xacro command）**：

| robot | mount | md5 / 字节（改动前 = 改动后） |
|---|---|---|
| `''` | plugin/urdf/sensor | `bc62af22f7be49395525978b1264750c` / 10353 |
| `hzmirm` | plugin/urdf/sensor | `a13e053090bf1344a06d58241bf845f9` / 18630 |
| `robot11` | plugin/urdf/sensor | `4f19a8b4faa3a7c659850d0064d3c5ca` / 47745 |

9/9 **逐字节相同**（本节只改 launch substitution，未碰 URDF/xacro）。

**(c) 执行集**：`launch_increment_diff.py`（同一组默认参数，OLD=HEAD/2022f0a、NEW=工作树）：
Node 定义 **28 → 28**、Include **10 → 10**、ExecuteProcess **0 → 0**；`only_old` = 一份
`small_point_lio`（删掉的"off+odom 由 LIO 自己发 TF"），`only_new` = `linefit_ground_segmentation_ros`
（base 档互斥变体）。运行期（`ps_after.txt`）：**只有一个** `ground_segmentation`、**只有一个**
`small_point_lio`、`lio_tf_adapter` 在、`odom_level_tf`（python3）在；`band_guard` 回读到
`frame=odom_level`、`/local_costmap global_frame=odom_level`、`/behavior_server global_frame=odom_level`。

**(d) 导航观测量**（g4_tf_r1）：见 §33.3 的 tf 列 —— in-band 100%、`h_meas` INFO（两处）、
`odom_level→base_link` roll/pitch <0.5°、`odom→base_link` +30.167°。

**(e) 3 个 tf 跑的姿态抖动（诚实）**：`odom_level→base_link` roll = **0.176°（g2）/ 1.038°（g3）/
0.169°（g4）**，pitch = 0.221 / −0.012 / 0.144。g3 那次是 LIO 自己的 roll 估计漂了 ~0.9°
（`odom→base_link` roll = 30.883°，静态边本身仍是 30.000°）⇒ **I11 的 0.5° 线不是每次都过**；
3 次里 2 次过。静态边 `odom→odom_level` 三次都逐位相同。

### 33.5 旧参数 / 非法组合：**t=0 硬报错**（exit code 1）

| 传参 | 结果 | 逐字证据（`.tmp_gravity/final_*`） |
|---|---|---|
| `gravity_align_mode:=hybrid` | **exit 1** | `旧参数已删除（硬报错别名）—— 重力/水平帧只有一个用户开关：gravity:=tf\|base\|lio\|raw` + `· gravity_align_mode:=hybrid（旧默认值 = auto）⇒ 改用 gravity:=tf\|base\|lio\|raw` |
| `level_frame:=odom` | **exit 1** | `· level_frame:=odom（旧默认值 = odom_level）⇒ 改用 gravity:=tf ⇒ odom_level；…` |
| `gravity_align_z_origin:=sensor` | **exit 1** | `· gravity_align_z_origin:=sensor（旧默认值 = ground）⇒ …` |
| `z_origin_source:=measured` | **exit 1** | `· z_origin_source:=measured（旧默认值 = urdf）⇒ …` |
| `lio_init_settle_s:=0.0` | **exit 1** | `· lio_init_settle_s:=0.0（旧默认值 = 1.0）⇒ 改用 无参数：初始化不再等落定…` |
| `lio_init_map_extrinsic:=False` | **exit 1** | `· lio_init_map_extrinsic:=False（旧默认值 = True）⇒ 改用 无参数：初始化地图一律用 R_IL·p + t_IL…` |
| `gravity:=base local_obstacle:=scan` | **exit 1** | `gravity:=base 与 local_obstacle:=scan 不能组合：… 2D 平盘在斜 odom 里没有逐点高度 … ⇒ 用逐点层：local_obstacle:=cloud\|stvl；或换 gravity:=tf` |
| `gravity:=lio lio:=fastlio` | **exit 1** | `gravity:=lio 只对 lio:=small_point_lio 生效 … 当前 lio:=fastlio：请改成 lio:=small_point_lio，或换 gravity:=tf\|base\|raw` |

**base 的默认障碍层**（不传 `local_obstacle`）⇒ 不报错，而是 **强制 `cloud` + WARN**（t=0）：
`[launch][WARN] gravity=base ⇒ 局部障碍层 local_obstacle:=scan 被强制成 cloud（2D /scan 在斜的 odom
里没有逐点高度；§25 实测 in-band 43.7% = 半盲）`。

### 33.6 base 档的 `base_link` 高度语义（live 参数）

| 量 | 值 | 判 |
|---|---|---|
| `/local_costmap/local_costmap global_frame` | `odom` | ✓ 无 odom_level |
| `/behavior_server global_frame` | `odom` | ✓ |
| `obstacle_layer.enabled` / `obstacle_cloud_layer.enabled` / `stvl_layer.enabled` | `false / true / false` | ✓ 强制逐点 |
| `/ground_segmentation gravity_aligned_frame` | **`base_link`** | ✓ linefit 在 base_link 里做高度语义 |
| `/scan` 的 frame | `base_link`（`band_guard` 回读） | ✓ p2l `target_frame=base_link` |
| `tf2_echo odom odom_level` | 空（边不存在） | ✓ 没有 `odom_level` 帧 |
| `band_guard` local | **IDLE** | ✓ 没有启用的 2D scan 带 |

### 33.7 初始化行为硬编码后的两条证据

1. **不等落定**：init 收据 `落定门=off（hold=0.00 s，峰峰值=-1.0000 m，丢弃 0 帧）| 初始化地图 =
   163830–163835 点`（4 臂一致）。SPL init 时刻仍在首帧后 2.0 s（200 帧 IMU 门），与 §32 的
   1.990–1.992 s 同量级；**没有推迟任何一帧**。
2. **初始化地图含 `t_IL`**：`R_IL=I` 快路径（`p + t_IL`）与一般式（`R_IL·p + t_IL`）逐位相同；
   robot11 的 `R_IL≠I` 走一般式。4 臂的 `h_meas` 都 **INFO（Δ≤0.2 mm）**，说明地面在初始化地图
   里没有被抬/压。
3. **WARN 真的会响**（scratch：把 URDF 的 h 人为 +4 cm，只跑 mapping）：
   `gravity_align_h_check: h_urdf=0.2995 h_meas=0.2600 |Δ|=0.0395 ∈ [0.03,0.05) ⇒ **WARN**
   （**出生未落定/地面不平/悬空**？…）—— 仍用 URDF 值定基`；`odom_level_tf` 同时
   `Δ=0.0406 ⇒ WARN`。**定基没有跟着实测走**。

### 33.8 本轮踩到并修掉的一个真问题（raw/base 的 TF 所有权）

第一次 4 臂批次（`g2_*`）里，base/raw 仍沿用 §4.11 的"回滚档" Node（LIO 自己发 `body` 档
`odom→base_link`）。实测 `odom→base_link` rpy 只有 **[0.148, 0.804, −0.997]°**，raw 的 in-band
= **753/774 (97.3%)** —— **半盲没复现**。根因：§4.12 把 30° 从 `base_link→livox_frame` 关节搬到
`livox_frame→imu_link` + LIO 的 `extrinsic_R` 之后，LIO 的 `body` 档 TF（`T_ol·T_bl`，`T_bl` 的
rpy=0）**不含 `extrinsic_R`** ⇒ 它发布一个"水平"的 base_link，把 30° 吞掉了。
⇒ 修法（本轮的实现）：**所有 gravity 档的 SPL 都走统一输出契约**（`publish_tf=false` +
`odom_convention=sensor`），`odom→base_link` 的唯一发布者 = `lio_tf_adapter`（父帧由 `_LevelFrame()`
给：tf ⇒ `odom_level`，base|lio|raw ⇒ `odom`）。修后 base/raw 的 `odom→base_link` = **30.1°**，
raw 的 in-band = **43.2%**、`band_guard` **43.7%/单侧** —— 与 §25 对齐。旧的"回滚档 Node"已删除；
`g3_*` 是本轮最终数据，`g2_*` 只作"修前"对照保留。

### 33.9 诚实清单

1. **每臂 n=1**（任务要求 1 跑/臂）。tf 额外跑了 3 次（g2/g3/g4）用于姿态抖动；其余三臂没有重复。
   所有跨臂比较都是**同机同时段**（21:23–21:44）但**不同 run**，不许逐格当同一次实验。
2. **tf 的 I11（`odom_level→base_link` roll/pitch <0.5°）3 次里 1 次不过**（g3 roll 1.038°，
   LIO 自身 roll 漂移）⇒ 契约写 "<0.5°" 时应理解为典型值，不是每次都保证。
3. **base 的"高度语义在 base_link"是感知侧性质**：代价图帧仍是 `odom`；`cloud` 层吃的是
   linefit 在 `base_link` 里分好类的 `/segmentation/obstacle`。**没有**在 `odom` 里再做一层
   base_link z 闸（那需要换帧，不在本轮）。
4. **base 的 cloud 层指标比 tf 的 2D scan 层大**（lethal 434 vs 301、ahead 61 vs 24）。
   这不是"tf 更好"：两层吃的数据/几何不同（3D 点 vs 投影盘），只报告、不排序。
5. **`gravity:=raw` 的 43.7% 与 §25 的 off 是跨轮比较**（§25 是 2026-10-20、旧安装约定、
   旧 init 行为）；本轮 raw 的 43.2–44.6% 是同一现象的新值。
6. **`lio:=fastlio|pointlio` 没有 base/raw 的初始化地面 WARN**（那条 WARN 只在 SPL 的初始化里做）；
   它们的 `gravity:=base|raw` 只保证帧/障碍层配置，不保证 estimator 的 z 锚点。
7. **`gravity:=base` 的显式 `local_obstacle:=both|stvl_both`** 被静默降级（+WARN），只有显式
   `scan` 才硬报错（用户批准的设计如此）。
8. **旧参数"显式传回旧默认值"不可检测**（launch 不保留来源）；非默认值 6/6 都能抓。
9. **没有改**地图/PCD 资产、`src/rm_simulation/**`、docs §1–§32 的正文；`src/rm_localization/**`
   只改了 `small_point_lio` 的初始化互检（+ 身份快路径），没有改算法本身。
10. **`hzmirm` / 默认模型 / `world:=RMUC2026` 的 4 臂**没有实跑（本轮只测 robot11 + RMUL2026 +
    SPL）；base 档的 `linefit gravity_aligned_frame=base_link` 在 `turret_pitch≠0` 时会退化
    （§4.14.5#6）。
11. **`spawn_entity.py` 的 "Entity pushed to spawn queue, but spawn service timed out"** 在
    4 跑里都出现（§32.9#13 的既有良性消息）；`ps_after` 与 LIO/nav 数据证明实体真的生成。

## §34 第二十四轮（**本机 date=2026-10-27，06:22–07:05**）：point_lio 的「初始化图 vs 逐帧图」一致性**核验**（不改代码）+ `gravity:=tf|base` 正式 A/B（3+3）+ 已知障碍物落点

> **本节只新增**（append-only）：§1–§33 一个字没改。任务（用户批准 "1,2"）：① 只**核验** point_lio
> 是否也像 SPL 那样在初始化地图里漏掉 `t_IL`（SPL 的同类缺陷在 `461eab8` 修掉）——**不许打补丁**，
> 除非找到确认缺陷且字节安全；② `gravity:=tf` vs `gravity:=base` 各 **3 跑**，外加每臂 1 次
> **已知障碍物**（0.3×0.3×0.5 m 箱子，初始朝向前方 ~2 m）落点测试。
>
> **一句话结论**：
> ① **pointlio 一致** —— 初始化图与逐帧图调的是**同一个函数** `pointBodyToWorld`，两者都是
> `x_.rot·(R_IL·p + t_IL) + x_.pos`；初始化时 `rot = rot_init`、`pos = 0`（MTK `vect` 默认零，
> 且 `Set_init` 之后没有别的赋值）⇒ 用真帧 + 真外参回放两条路径 **逐位相同、max|Δ| = 0.000000 m**。
> **不是缺陷，没有打补丁**。
> ② **`gravity:=tf` 应继续做默认档**：3/3 跑局部 in-band **100%**（773/773）、`band_guard` **OK 100%**
> （11/11 条采样）、`odom_level→base_link` roll/pitch **≤0.198°/0.190°**、已知障碍物标记格 y 向偏差
> **+0.012 m**；`base` 3/3 `band_guard` **IDLE**（没有 2D 高度带可评）、探针 in-band **43.4%**
> （336/774）、已知障碍物在 30° 斜的 `odom` 里被**沿 −y 抹开 0.134 m**——0.5 m 高箱子的竖直边在
> 斜帧里投影成 `0.5·sin30° = 0.25 m` 的 y 长度，质心因此偏 −0.125 m（实测 −0.134 m）。

### 34.1 point_lio 初始化图 / 逐帧图用的是不是同一条 LiDAR→IMU→世界变换

#### 34.1.1 代码事实：**同一个函数**，世界变换在函数**内部**，不在调用方

被问的「~:98–112 的 helper 只做 `R_IL·p + t_IL`」是 `pointBodyLidarToIMU`
（`src/rm_localization/point_lio/src/laserMapping.cpp:96`），它**只**被 `publish_frame_body`
（`:200`，发布 `cloud_registered_body`、`frame_id="body"`）调用一次（`:207`）：

```cpp
void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)   // :96
{ ...
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;          // :107 只到 IMU 系
  }
```

**地图插入用的不是它**，而是 `pointBodyToWorld`（`src/Estimator.cpp:364`）——它自己就把世界旋转与
平移乘上了，**没有**第二个重载，也没有调用方再补一次：

```cpp
void pointBodyToWorld(PointType const * const pi, PointType * const po)     // Estimator.cpp:364
{ V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global;
  if (extrinsic_est_en) { ...
  } else {
    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_output.x_.pos;                                          // :381–382
    } else { ... kf_input.x_.rot * (...) + kf_input.x_.pos; ... } }         // :384–385
```

① **初始化图**（`laserMapping.cpp:485–503`）——注意 `pointBodyToWorld` 在**整个初始化块里**被调用，
结果既进 `init_feats_world` 又进 `feats_down_world`：

```cpp
      /*** initialize the map ***/
      if (!init_map) {                                                      // :486
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));  // :490
          }
        }
        for (size_t i = 0; i < feats_down_world->size(); i++) {
          init_feats_world->points.emplace_back(feats_down_world->points[i]);
        }
        ...
          ivox_->AddPoints(init_feats_world->points);                       // :499
          publish_init_map(pub_laser_cloud_map);                            // :500 → /Laser_map
```

② **逐帧插入**（`laserMapping.cpp:670–674` 与 `:852–856`，输出/输入两个 EKF 分支各一份，正文**逐字相同**）：

```cpp
            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);              // :673 / :855
            }
```
之后 `MapIncremental()`（`:944`）把**已经算好世界坐标**的 `feats_down_world` 里满足体素条件的点
`ivox_->AddPoints(points_to_add)`（`:144`）。

③ **两条路径在 t0 的状态是同一组数**：

```cpp
          M3D rot_init;                                                     // :474
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot  = rot_init;                                      // :476
          kf_output.x_.rot = rot_init;                                      // :477
```
`x_.pos` 在初始化块里**没有任何赋值**；`MTK::vect` 的构造函数默认值是 `base::Zero()`
（`include/IKFoM/IKFoM_toolkit/mtk/types/vect.hpp:110`
`vect(const base & src = base::Zero())`）⇒ **初始化图里 `pos ≡ 0`**。实测也证实：本轮回放抓到的
**第一帧 `/odom`** 的 `xyz = [-7.4e-06, -1.2e-07, 1.9e-06] m`（§34.1.2）。

④ **两侧的输入点在同一坐标系**：`Preprocess::avia_handler`（`src/preprocess.cpp:320–362`）把
`msg->points[i].x/y/z` **原样**塞进 `pl_surf`（`:344–347`，没有任何外参旋转）；
`ImuProcess::Process`（`src/IMU_Processing.cpp:87–105`）在这一版里只是 `*cur_pcl_un_ = *(meas.lidar)`
（`:104`）⇒ `feats_undistort` 与 `feats_down_body` **都是 LiDAR 系**（变量名叫 body，但内容是
lidar 系），所以 `pointBodyToWorld` 里那**一次** `R_IL` 正好是唯一一次。

⇒ **结论（by construction identical）**：初始化图与逐帧图
- 调**同一个** `pointBodyToWorld`；
- 吃**同一系**的点（LiDAR 系）；
- t0 时状态相同（`rot = rot_init, pos = 0`）；
- **ivox/地图在 `camera_init`（世界）系**，不是 IMU 系；世界变换由 `pointBodyToWorld` **自己**施加。

#### 34.1.2 回放（真帧 + **运行期生效**外参 + 真 `rot_init`）

取数：`mode:=mapping lio:=pointlio robot:=robot11 robot11_mount:=sensor world:=RMUL2026` 一个 stack
（`.tmp_g34/g34_pointlio2/`），只订阅、不发布、不改点云：`/livox/lidar` 一帧（**11852 点，
`frame_id=livox_frame`**）、`/Laser_map`（初始化图，**2701 点**，`camera_init`）、
`/cloud_registered` 前 3 帧（**138 / 142 / 149 点**）、第一帧 `/odom`（`q = [-0.25878, -0.00017,
8.66e-06, 0.96594]` ⇒ `rot_init` rpy = **[-29.996, -0.019, 0.006]°**）。

运行期**真正生效**的外参（`ros2 param get /laserMapping mapping.extrinsic_*`）：
`R_IL = I`、`t_IL = [0, 0, 0.05] m`（`|t_IL| = 0.0500 m`）。
⚠️ 见 §34.3#4：launch 传的 **URDF 派生的** `extrinsic_T`/`extrinsic_R` 对 `lio:=pointlio`
**是死键**（`Parameter not set`），point_lio 只读 `mapping.extrinsic_*`。

把这一帧按两条路径各算一遍（`M3D` 行主序、`%point_filter_num==0`、`blind=0.5`、`det_range=100`、
相邻点去重，全部照 `preprocess.cpp` / `avia_handler` 复刻）：

| 量 | 结果 |
|---|---|
| **(a) 初始化图路径** `rot_init·(R_IL·p + t_IL) + 0` vs **(b) 逐帧路径**（状态取 t0） | — |
| **max\|Δ\|（x, y, z）** | **0.000000, 0.000000, 0.000000 m** |
| **median\|Δ\|（x, y, z）** | **0.000000, 0.000000, 0.000000 m** |
| `np.array_equal(A, B)` | **True（逐位相同）** |
| 隐含偏置方向 | **无**（两条路径的输出逐点相同） |

**这个检查的灵敏度**（把「如果犯 SPL 那种错」的量级算出来，证明上面的 0 不是"检验太钝"）：

| 反事实 | median Δ（x, y, z） | median \|Δ\| |
|---|---|---|
| 初始化图**漏掉 `t_IL`**（只做 `rot_init·R_IL·p`） | `[+0.000017, −0.02500, −0.04330] m` | **0.0500 m** (= `|t_IL|`) |
| 初始化图**完全没上世界变换**（留在 IMU 系） | — | **0.3371 m** |
| 逐帧路径改用**第一帧实测 odom 状态**（不是 t0 的 `rot_init/0`） | — | 7.7e-06 m（`pos0≈0`、`rot` 未变） |

最后一行同时证明「t0 的 pos 真的是 0」：第一帧 `/odom` 的位置是 1.9e-06 m 量级。

#### 34.1.3 经验核对（不靠代码、只看包真正发出来的点云）

| 核对 | 数 |
|---|---|
| **用代码式子**从真 LiDAR 帧算出的初始化图（2723 点）vs 包**真正发布的** `/Laser_map`（2701 点）：最近邻距离 median / p90 | **18.1 mm / 78.1 mm** |
| 同上，最近邻位移向量的 median（x, y, z） | **(+0.0003, +0.0011, +0.0007) m** ⇒ **无系统偏置** |
| 反事实：把 `t_IL` 拿掉再和 `/Laser_map` 比，最近邻中位数 | **52.0 mm**（≈2.9×；漏 5 cm 会被这条检验抓出来） |
| `/Laser_map` vs 紧随其后的 `/cloud_registered`（138 点）：最近邻 median / 位移 median | 64.7 mm / `(+0.0001, −0.0018, −0.0047) m` |
| 同上的 caveat | `filter_size_map = 0.5 m`、`filter_size_surf = 0.5 m` ⇒ 这条只能排除 ≳0.5 m 的系统偏移；**2 帧不是同一帧**（Livox 非重复扫描）＋静止，表面同、采样不同 ⇒ mm~cm 残差正常 |

#### 34.1.4 判定

**pointlio 一致（by construction identical），偏 0.000 m。** 它不是 SPL 那种缺陷：
- SPL 的 bug 是「初始化地图**另写了一条**只做 `p + t_IL` 的快路径，漏了 `R_IL`/或漏了 `t_IL`」；
- point_lio **没有第二条路径**可漏——初始化与逐帧都只走 `pointBodyToWorld`，世界变换 `rot/pos`
  在那个函数**内部**。所谓「~:98–112 的 helper 只到 IMU 系」的那个函数
  （`pointBodyLidarToIMU`）**只用于发布 body 系点云**，与地图无关。
- 因此 **不打补丁**（任务要求：除非确认缺陷且字节安全）。

#### 34.1.5 point_lio 的 `h_meas` 局限（为什么本节的核对不靠它）

`gravity_align_h_check`（`small_point_lio` 的初始化地面互检）要求首帧地面有足够 inliers；point_lio
的 `/cloud_registered` **每帧只有 138 / 142 / 149 点**（`filter_size_surf=0.5` 体素降采样之后），
比任何 "≥500" 的门槛都低 ⇒ 那条检查在 pointlio 上**要么根本不跑**（它只存在于 SPL，§33.9#6），
要么会 **WARN 跳过**。所以本节用**原始 LiDAR 帧 + 真值式子 + 发布出来的地图**三条独立证据，
不用 `h_meas`。

### 34.2 `gravity:=tf` vs `gravity:=base` 正式 A/B（各 3 跑）

#### 34.2.1 协议、隔离与冻结

* 命令（6 跑逐字同口径）：`world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp
  nav:=mppi planner:=smac2d robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 band_guard:=True`
  + `gravity:=tf|base`；`SETTLE=20 WINDOW=20`（**仿真秒**）、`gui:=False`、`nav_rviz:=False`、
  `lio_rviz:=False`。
* 隔离：私有 `HOME` / `ROS_LOG_DIR` / `ROS_DOMAIN_ID` / 先探空端口再占 `GAZEBO_MASTER_URI`、
  `unset DISPLAY`、`setsid` 起栈（进程组 = 本 run）、每跑硬超时 1100 s、收尾只用
  `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）、**一栈一跑**（6 跑顺序，跑间 8 s）。
* 冻结：6 跑的 `git_head = 5a19a9c`（= `origin/master`）；`bringup_sim.launch.py`
  md5 = **`6ad0aaaedc18486dc8318ce235c7b738`**（与 §33 逐位相同）；没有换任何二进制。
* 探针：`tools/scripts/tiltmount/nav_band_probe.py`（tf ⇒ `--scan-frame odom_level`；base ⇒
  `--scan-frame odom`）+ `tools/scripts/tiltmount/uc_procstats.py` + `band_guard` 收据 + 运行期
  `ros2 param get` 回读。原始数据：`.tmp_g34/g34_{pilot_tf,tf_r2,tf_r3,base_r1..r3}/`
  （pilot 就是 tf 的第 1 跑；tag 名沿用起跑时的临时命名）。

#### 34.2.2 每臂 3 跑

**`gravity:=tf`（3 跑，逐跑 + median / range）**

| 指标 | r1 | r2 | r3 | median | range |
|---|---|---|---|---|---|
| 局部代价图帧（live） | `odom_level` | `odom_level` | `odom_level` | — | — |
| 局部障碍层 `obstacle_layer` / `obstacle_cloud_layer` | true / false | 同 | 同 | — | — |
| `band_guard`（local） | **OK** 11/11 = 100% | **OK** 11/11 | **OK** 11/11 | **100%** | 100–100 |
| kept-y（`band_guard`） | [-0.688, 5.042] | [-0.688, 5.042] | [-0.613, 5.055] | 双侧 | 双侧 3/3 |
| 探针 in-band `[0,2]` | **773/773** | **773/773** | **773.5/773.5** | **100%** | 100–100 |
| 扫描面 z p50（代价图帧） | +0.1031 | +0.1032 | +0.1033 | **+0.1032** | 0.1031–0.1033 |
| local **lethal** | 302.5 | 308 | 314 | **308** | 302.5–314 |
| local inscribed | 10181.5 | 10193 | 10346 | **10193** | 10181.5–10346 |
| local free | 40445.5 | 40430.5 | 39893 | **40430.5** | 39893–40445.5 |
| local **最近 lethal** | 0.7042 | 0.7045 | 0.7044 | **0.7044 m** | 0.7042–0.7045 |
| local **前方 2.0×1.2 m lethal** | 23 | 23 | 24 | **23** | 23–24 |
| local **1 m 内 lethal** | 31 | 31 | 31 | **31** | 31–31 |
| global lethal | 2167 | 2166 | 2164.5 | **2166** | 2164.5–2167 |
| `h_meas` / \|Δ\| | 0.2600 / 0.5 mm | 0.2596 / 0.2 mm | 0.2597 / 0.1 mm | 0.2597 / **0.2 mm** | 0.1–0.5 mm |
| `odom_level→base_link` rpy（deg） | [0.174, 0.190, -1.308] | [0.166, 0.175, -1.284] | [0.198, 0.185, -1.266] | — | roll/pitch **≤0.198/0.190** |
| `odom→base_link` rpy（deg） | [30.175, 0.823, -1.041] | [30.165, 0.802, -1.079] | [30.172, 0.800, -1.008] | — | roll 30.165–30.175 |
| 静态边 `odom→odom_level` | xyz [0.000, 0.108, −0.187]、rpy [30.000, −0.000, 0.000] | 逐位同 | 逐位同 | — | 3/3 逐位相同 |

**`gravity:=base`（3 跑）**

| 指标 | r1 | r2 | r3 | median | range |
|---|---|---|---|---|---|
| 局部代价图帧（live） | `odom` | `odom` | `odom` | — | — |
| 局部障碍层 `obstacle_layer` / `obstacle_cloud_layer` / `stvl` | **false / true / false** | 同 | 同 | — | — |
| `ground_segmentation gravity_aligned_frame` | **`base_link`** | 同 | 同 | — | — |
| `band_guard`（local） | **IDLE** | **IDLE** | **IDLE** | **IDLE 3/3** | — |
| kept-y（`band_guard`） | —（没有 2D 带） | — | — | — | — |
| 探针 in-band `[0,2]`（**不是本档判据**） | 336/774 (43.4%) | 336/774 (43.4%) | 334/773 (43.2%) | **43.4%** | 43.2–43.4% |
| 扫描面 z p50（代价图帧） | −0.0946 | −0.0938 | −0.1012 | **−0.0946** | −0.1012…−0.0938 |
| local **lethal** | 385 | 390 | 438.5 | **390** | 385–438.5 |
| local inscribed | 12645 | 12392 | 12626.5 | **12626.5** | 12392–12645 |
| local free | 38340 | 38787.5 | 38732.5 | **38732.5** | 38340–38787.5 |
| local **最近 lethal** | 0.7010 | 0.7017 | 0.6964 | **0.7010 m** | 0.6964–0.7017 |
| local **前方 2.0×1.2 m lethal** | 59 | 59 | 61 | **59** | 59–61 |
| local **1 m 内 lethal** | 67 | 68 | 70 | **68** | 67–70 |
| global lethal | 2166 | 2166.5 | 2166 | **2166** | 2166–2166.5 |
| `h_meas` / \|Δ\| | 0.2597 / 0.1 mm | 0.2597 / 0.2 mm | 0.2598 / 0.2 mm | 0.2597 / **0.2 mm** | 0.1–0.2 mm |
| `odom_level→base_link` rpy | **没有这个帧** | — | — | — | — |
| `odom→base_link` rpy（deg） | [30.130, 0.766, −0.880] | [30.178, 0.712, −1.071] | [30.160, 0.810, −1.021] | — | roll 30.130–30.178 |

**读法**

1. **tf 的 3 跑几乎同一次实验**：lethal 302.5–314（极差 11.5）、最近 lethal 0.7042–0.7045（极差
   0.3 mm）、ahead 23–24、1 m 内 31–31、`h_meas` Δ 0.1–0.5 mm；静态边三次**逐位相同**。
   `odom_level→base_link` roll/pitch **3/3 都 <0.5°**（§33 的 g3 那次 1.038° 没复现，n=3）。
2. **base 的重复性明显更差**：lethal 385–438.5（极差 53.5 ≈ tf 的 4.7 倍）、最近 lethal
   0.6964–0.7017（5.3 mm）。逐点层吃的是 3D 点，受 LIO 姿态抖动/地面分割边界的影响比 2D 平盘大。
3. **base 的 cloud 指标更大 ≠ 更好**（§33.9#4 同口径）：lethal 390 vs 308、ahead 59 vs 23、
   1 m 内 68 vs 31。两层吃的数据/几何不同（3D 点 vs 投影盘），且 base 的代价图帧是**斜 30° 的
   `odom`**（见 §34.2.3 的已知障碍物测试，那里把"斜"的代价量出来了）。只报告、不排序。
4. **`band_guard` 是本轮唯一"判据级"的差异**：tf 有 2D 高度带可评且 **100% OK / kept-y 双侧**；
   base 没有启用的 2D 带 ⇒ **IDLE**（设计如此：§4.14.5#1）。**IDLE 不是"通过"**，是"这条诊断
   在本档不存在"。

#### 34.2.3 ⭐ 已知障碍物落点（每臂 1 跑）

做法（同一次 run 内**差集**，不做跨 run 比较；`.tmp_g34/g34_box2_{tf,base}/box.json`）：
起栈 → 局部代价图稳定后取 4 帧基线 lethal 集合 → 调 gazebo_ros 的 `/spawn_entity`
（`gazebo_msgs/srv/SpawnEntity`，**service 名不是 `/world/<name>/create`**：本仓默认
`sim_backend:=classic`）生成 **0.3×0.3×0.5 m 静态箱子**在世界系 **(6.3, 3.35, 0.25)**（= 出生点
`(4.3, 3.35)` 的 +x 2.0 m，yaw=0 ⇒ **初始朝向前方 2 m**；RMUL2026 该处在静态图里是 free，
已核对 pgm）→ 等 8 仿真秒 → 取 6 帧 lethal 集合 → `post \ pre` = 这个箱子贡献的标记格 →
按「机器人位姿在**代价图帧**里的坐标 + 前方 2.0 m」换算真值中心 → 量质心偏差。

| 量 | **`gravity:=tf`** | **`gravity:=base`** |
|---|---|---|
| 代价图帧 | `odom_level` | `odom` |
| spawn 成功 / 服务 | ✓ `/spawn_entity` | ✓ `/spawn_entity` |
| 基线 lethal → 之后 lethal | 318 → 336 | 370 → 390 |
| 差集新 lethal 格数（全部） | 46 | 113 |
| 其中落在真值 0.8 m 内 | **16** | **36** |
| 真值箱子中心（代价图帧） | **(2.014, −0.112)** | **(2.009, −0.026)** |
| **标记格质心**（代价图帧） | **(1.870, −0.100)** | **(1.859, −0.159)** |
| **Δ（质心 − 真值）** | **(−0.144, +0.012) m** | **(−0.149, −0.134) m** |
| **\|Δ\|** | **0.144 m** | **0.201 m** |
| 标记格包围盒 | x **1.870 单列**、y [−0.250, +0.050]（宽 0.30） | x [1.850, 1.870]、y [−0.330, +0.030]（宽 **0.36**） |

**读法（这才是这两个数真正的区别）**

1. **两臂的 Δx ≈ −0.145 m 都不是误差**：2D 扫描/点云只能看到箱子的**近侧面**，箱子中心在
   `x = 2.00`、近侧面在 `x = 2.00 − 0.15 = 1.85`；两臂都把 lethal 打在 x ≈ 1.85–1.87（**距近侧面
   0–2 cm = 1 个 0.02 m 栅格**）。所以拿"中心"当参考，两臂都天然吃 −0.15 m。
2. **真正的差异在 y**：tf 的标记格 y 质心 **−0.100** vs 真值 **−0.112** ⇒ **+0.012 m**（≈ 1 个栅格
   以内，横向对得上）；base 的 **−0.159** vs 真值 **−0.026** ⇒ **−0.134 m**，包围盒宽度也从
   0.30 m 涨到 **0.36 m**。
3. 这个 −0.134 m **不是噪声，是几何**：`odom→base_link` roll ≈ **+30°**，箱子在 `base_link` 里
   竖直跨 z∈[0, 0.5] ⇒ 投到 `odom` 后 y 跨 **`−0.5·sin30° = −0.25 m`**，均匀分布的质心就该在
   **−0.125 m**；实测 **−0.134 m**。也就是说 **base 档在斜 `odom` 里，一个 0.5 m 高的障碍物
   的 2D 位置会沿 −y 偏 0.13 m**（越高偏越多，`Δy ≈ −h/2·sin(30°)`）。
4. tf 没有这个抹开：2D `/scan` 是在 `base_link` 里投影成平盘，再经静态边搬到 `odom_level`
   （水平帧）⇒ 竖直结构不会变成 y 偏移。

#### 34.2.4 判定：默认档应保持 `gravity:=tf`

**数字理由（不是感觉）**

1. **可诊断性**：tf 有 2D 高度带这条**会响的**判据，3/3 跑 **OK 100%**（11/11 条采样、kept-y 双侧）；
   base 的 `band_guard` **3/3 IDLE** —— 同样的"半盲"如果发生，base 档**不会报警**（§33.3 已经
   量过 raw 档 43.7% + 单侧 kept-y 是真能发生的）。
2. **水平帧是真的水平**：tf 的 `odom_level→base_link` roll/pitch **≤0.198°/0.190°（3/3 <0.5°）**；
   base 没有 `odom_level` 帧，`odom→base_link` roll **30.13–30.18°**。
3. **已知障碍物的 2D 落点**：tf 横向偏差 **+0.012 m**；base **−0.134 m**（竖直结构在斜帧里的
   几何投影，`−h/2·sin30°`），包围盒还胀 0.06 m。
4. **重复性**：tf 的 lethal 极差 **11.5**、最近 lethal 极差 **0.3 mm**；base 是 **53.5 / 5.3 mm**。
5. base 的 lethal/ahead/1 m 更大**不能**拿来当"更好"：数据源不同（3D 点 vs 投影盘），
   而且它大的一部分正是上面第 3 条的"抹开"（同一面墙被投成更多格）。

**base 仍然是一个合法档**：它把高度语义放在 `base_link`（`p2l target_frame=base_link` +
`linefit gravity_aligned_frame=base_link`）、局部障碍层强制逐点，对"不想让 2D 平盘决定可见性"
的场景是对的形态；但作为**默认**，它丢掉了（a）会响的高度带判据、（b）水平代价图帧、
（c）已知障碍物的横向定位精度。

### 34.3 诚实清单

1. **pointlio 部分是"核验"，没有改一行代码**：`src/rm_localization/point_lio/**` 一个字节没动
   （该子模块工作树里原有的 `child_frame_id: body→imu_link` 改动**不是本轮引入**，也不是我改的）。
   回放脚本、抓帧脚本、A/B 驱动脚本都在 gitignore 的 `.tmp_g34/` 里，**没有进仓**。
2. **回放的 (a)(b) 两条式子逐位相同是"代码恒等"的必然结果**（同一个函数、同一个状态），所以
   `max|Δ| = 0.000000` 本身不是独立证据；独立证据是 §34.1.1 的代码行 + §34.1.3 的
   "代码式子 vs 包发布的 `/Laser_map`"（18.1 mm、位移中位数 0.7 mm，而漏 `t_IL` 会变 52.0 mm）。
3. **§34.1.3 的两帧不是同一帧**（Livox Mid-360 非重复扫描）：18.1 mm 的最近邻中位数里既有采样
   差异也有静态残差，所以这条只能排除 ≳5 cm 的系统偏移；`filter_size_map=0.5 m` 让
   "`/Laser_map` vs `/cloud_registered`"那条更钝（64.7 mm）。
4. **point_lio 的外参来源和 launch 的注释不一致（本轮新发现，只记录、未修）**：
   `bringup_sim.launch.py` 给 `lio:=pointlio` 传了 URDF 派生的
   `{'extrinsic_T': [_MountExtrinsic('xyz')], 'extrinsic_R': [_MountExtrinsic('rot')]}`，但
   point_lio 只读 **`mapping.extrinsic_T` / `mapping.extrinsic_R`**（`src/parameters.cpp:179–183`）。
   运行期回读：`/laserMapping extrinsic_T = Parameter not set`、
   `/laserMapping mapping.extrinsic_T = [0.0, 0.0, 0.05]`、`mapping.extrinsic_R = I`。
   ⇒ **robot11 的 +30° 安装角没有通过这两个键进 point_lio**（它靠点云本身已被插件旋平）。
   这不影响本节的结论（两条路径用的是**同一组**外参），但它是一条真实的不一致，
   留给 §4.12/lio 槽位的下一轮处理。
5. **A/B 每臂 n=3**，同机、同时段（06:22–06:39）、同一 launch md5，但**不同 run**；不许把
   两臂逐格当同一次实验。极差已经给出，中位数只是 3 个点的中位数。
6. **已知障碍物测试每臂 n=1**，且**差集包含非箱子的新 lethal 格**（tf 46 格里只有 16 格、
   base 113 格里只有 36 格落在真值 0.8 m 内；其余是 LIO 抖动/扫描噪声造成的新标记）。两臂用的是
   同一套流程、同一个 spawn 服务、同一个箱子尺寸与位置，可比性成立，但**没有重复**。
7. **Δx 的 −0.145 m 是"只看得到近侧面"的几何后果，不是定位误差**：如果以后要拿这条当精度指标，
   应改成"标记格 x 的 `max` 与箱子远侧面比"或直接把箱子换成圆柱。
8. **base 的 y 抹开是"设计使然"不是 bug**：§4.14.5#1 已经写明 base 的代价图帧仍是斜 `odom`、
   高度语义只在 `base_link`；本节第一次把它**量化**成 `−h/2·sin(30°)`。
9. **本轮没有测** `gravity:=lio|raw`（§33 已各 1 跑）、没有测 `hzmirm`/默认模型/`world:=RMUC2026`、
   没有测 `localization:=small_gicp|icp|none` 的组合；A/B 只覆盖
   `robot11 + robot11_mount:=sensor + RMUL2026 + gicp`。
10. **没有改**地图/PCD 资产、`src/rm_simulation/**`、`src/rm_localization/**`、`docs` §1–§33 正文；
    本轮只追加本节 + 契约 §4.14.5 的第 8 条。

## §35 第二十五轮（**本机 date=2026-10-27，07:48–08:20**）：LIO 外参**键名**修复 —— fastlio/pointlio 的 `mapping.extrinsic_*`、pointlio yaml 默认 Rx(+30°)、运行期断言；pointlio 3 跑

> **本节只新增**（append-only）：§1–§34 一个字没改。任务（用户批准）：pointlio 运行期
> `mapping.extrinsic_R = I`（§34.3#4 实测）⇒ robot11 的 +30° 从未进过 point_lio。要求：
> ① 查清三个后端的键名并写表；② launch 按各自键名注入；③ 修 pointlio yaml 默认值；
> ④ 加运行期断言；⑤ pointlio 3 跑复核（同 §34 协议）。
> 代码 commit **`cdcd7e3`**（父仓）+ **`d281584`**（`src/rm_localization/point_lio` 子模块）。
>
> **一句话结论**：
> ① 三个后端的键名**不是同一个**：fastlio/pointlio = `mapping.extrinsic_T/R`，SPL = 顶层
> `extrinsic_T/R`。**fastlio 也一直是死键**（任务里"fastlio 大概是顶层"的猜测是反的）——
> 它和 pointlio 一样只读 `mapping.*`，只是以前没人量过。
> ② 修复后 pointlio 运行期 `mapping.extrinsic_R = Rx(+30°)`（1e-9 内逐元素相等），
> 顶层死键消失（`Parameter not set`）。
> ③ **pointlio 自己的地图地面从斜 30° 变成水平**：累积 `/cloud_registered` 拟合的地面法向
> **30.05° → 0.030–0.054°**（3 跑）。这是本轮**唯一的结构性变化**。
> ④ 但 **nav 观测量（in-band / local lethal / nearest / ahead / rpy）没有变**（§35.4/§35.5）：
> 它们走 `/scan` = **原始云 + linefit + TF**，与 LIO 的 extrinsic 无关。§30/§32 的 pointlio
> 导航数字**仍然有效**；但"pointlio 的 odom/地图是自洽的"这个隐含前提以前**不成立**。
> ⑤ z 锚点本轮 **−10.5 / −8.2 / −7.5 mm**（旧 §32 −6.4/−6.1/−5.1 mm）：是 point_lio 自己
> 0.5 s 初始化的 state z（§32 trace −3.8/−9.6/−6.1 mm 同量级），不是 extrinsic 的直接后果。

### 35.1 键表（= 契约 §4.12.5；唯一真源 = launch 的 `_LIO_EXTRINSIC_KEYS`）

| LIO 槽位（`lio:=`） | **节点实际读取的键** | 读取处（file:line） | launch 写键 | 断言节点 |
|---|---|---|---|---|
| `fastlio` | `mapping.extrinsic_T` / `mapping.extrinsic_R` | `src/rm_localization/FAST_LIO/src/laserMapping.cpp:834–835`（declare）、`:870–871`（get） | `_lio_extrinsic_params('fastlio')` | `lio_extrinsic_receipt.py` → `/laser_mapping` |
| `pointlio` | `mapping.extrinsic_T` / `mapping.extrinsic_R` | `src/rm_localization/point_lio/src/parameters.cpp:179–183` | `_lio_extrinsic_params('pointlio')` | `lio_extrinsic_receipt.py` → `/laserMapping` |
| `small_point_lio` | **顶层** `extrinsic_T` / `extrinsic_R` | `src/rm_localization/small_point_lio/src/small_point_lio/parameters.cpp:110–116` | `_lio_extrinsic_params('small_point_lio')` | `lio_extrinsic_receipt.py` → `/small_point_lio` |

- 三个后端的外参**语义相同**（`p_imu = R_IL·p_lidar + t_IL`；fastlio `laserMapping.cpp:173/185`、
  pointlio `Estimator.cpp:381–382`、SPL 的出点公式），**只有键名不同**。
- **死键**：fastlio/pointlio 的顶层 `extrinsic_*`、SPL 的 `mapping.extrinsic_*` 一律不许有值。
  修复前 launch 给三个槽位**都**传顶层 ⇒ fastlio/pointlio 一个都没读到（§34.3#4 只记了 pointlio，
  本轮发现 fastlio 同病）。

### 35.2 改了什么（file:line）

| 文件 | 位置 | 改动 |
|---|---|---|
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | 1410–1487 | 新增 `_LIO_EXTRINSIC_KEYS`（per-LIO 键表）、`_LIO_NODE_NAMES`、`_lio_extrinsic_params`、`_lio_extrinsic_check_params`、`_lio_extrinsic_receipt` / `_LioExtrinsicReceipt`；`_check_gravity_contract` 末尾追加 `[lio_extrinsic]` t=0 收据 |
| 同上 | 3208 / 3255 / 3342、3369 | 三个 LIO 的参数字典改成 `_lio_extrinsic_params(<slot>)`：fastlio/pointlio 落 `mapping.extrinsic_*`，SPL 落顶层（SPL 的值/键与改动前**逐字段相同**） |
| 同上 | 3223–3228 / 3262–3267 / 3391–3396 | 每个 LIO 后加一个 `Node(rm_nav_bringup/lio_extrinsic_receipt.py)`（一次性断言） |
| `src/rm_nav_bringup/scripts/lio_extrinsic_receipt.py` | 新增（253 行） | 运行期读回**生效**外参 + 死键探测 + 与 URDF 值比对；`OK`/`ERROR` 收据，不一致 exit 1 |
| `src/rm_nav_bringup/CMakeLists.txt` | 56 | 把断言脚本装到 `lib/rm_nav_bringup/` |
| `src/rm_localization/point_lio/config/pointlio_mid360_sim.yaml` | 53 / 62–64 | `mapping.extrinsic_R`：`I` → **`Rx(+30°)`**（`T` 仍 `[0,0,0.05]`）；launch 仍用 URDF 覆盖（唯一真源），改的是**独立使用**时的默认值 |

### 35.3 运行期回读（before / after）

`ros2 param get /laserMapping …`（探针结束后、同一跑内回读）：

| 跑 | 代码 | `mapping.extrinsic_T` | `mapping.extrinsic_R` | 顶层 `extrinsic_T/R` | 断言节点 |
|---|---|---|---|---|---|
| `g35_before_pl`（07:52） | HEAD `550f596` | `[0, 0, 0.05]` | **`I`** | **`Parameter not set`**（死键） | 无（本轮才加） |
| `g35_before_map_pl`（08:10，旧 launch + 旧 yaml） | HEAD `550f596` | `[0, 0, 0.05]` | **`I`** | `Parameter not set` | 无 |
| `g35_pl_r1/r2/r3`（08:00/08:04/08:07） | `cdcd7e3` | `[0, 0, 0.05]` | **`Rx(+30°)` = `[1, 0, 0, 0, 0.866025404, −0.5, 0, 0.5, 0.866025404]`** | `Parameter not set` | **OK**（3/3，`tol=1e-9`） |

### 35.4 pointlio 3 跑（修复后；协议 = §34.2.1 逐字同）

命令：`world:=RMUL2026 mode:=nav localization:=gicp robot:=robot11 robot11_mount:=sensor
lio:=pointlio spin_speed:=0.0 band_guard:=True nav:=mppi planner:=smac2d` +
`nav_rviz/lio_rviz/gui:=False`；`SETTLE=20 WINDOW=20`（仿真秒）、私有 HOME/ROS_DOMAIN_ID/
GAZEBO_MASTER_URI、一栈一跑、收尾 `scoped_cleanup.sh`。冻结：`git_head=550f596`（工作树 =
`cdcd7e3` 的改动）、launch md5 `740dbeb5…`、pointlio yaml md5 `d78da583…`、断言脚本 md5
`b425b7c8…`。原始数据 `.tmp_g35/g35_pl_r{1,2,3}/`（gitignored）。

| 指标 | r1 | r2 | r3 | median | range |
|---|---|---|---|---|---|
| `mapping.extrinsic_R` 回读 | Rx(+30°) | Rx(+30°) | Rx(+30°) | — | 3/3 |
| `lio_extrinsic_receipt` | **OK** | **OK** | **OK** | — | 3/3 |
| 局部 in-band（生效带） | **773/773** | **773/773** | **773/773** | **100%** | 100–100 |
| `band_guard`（local） | **OK 11/11** | **OK 11/11** | **OK 11/11** | **100%** | 100–100 |
| 扫描面 z p50（`odom_level`） | +0.0915 | +0.0946 | +0.0948 | **+0.0946** | 3.4 mm |
| local lethal | 202 | 236 | 206 | **206** | 34 |
| local inscribed / free | 8827 / 43446 | 8946 / 43216 | 8842 / 43422 | 8842 / 43422 | 119 / 230 |
| local **最近 lethal** | 0.7028 | 0.7006 | 0.7013 | **0.7013 m** | 2.2 mm |
| local 前方 2.0×1.2 m lethal | 19 | 18 | 19 | **19** | 1 |
| local 1 m 内 lethal | 25 | 25 | 26 | **25** | 1 |
| global lethal / nearest | 1970 / 0.7761 | 1970 / 0.7741 | 1970 / 0.7756 | **1970 / 0.7756** | 0 / 2.1 mm |
| **`base_link` z（`odom_level`）** | +0.0920 | +0.0943 | +0.0950 | **+0.0943** | 3.0 mm |
| **换算 `floor_z` = base_z − GT** | **−10.5 mm** | **−8.2 mm** | **−7.5 mm** | **−8.2 mm** | 3.0 mm |
| `odom_level→base_link` rpy（deg） | [0.082, 0.002, −0.263] | [−0.040, 0.212, −0.309] | [−0.067, 0.163, −0.329] | — | roll/pitch **≤0.212** |
| `odom→base_link` rpy（deg） | 同值（静态边旋转 = I） | 同值 | 同值 | — | — |
| 静态边 `odom→odom_level` | I / (0,0,0.216226) | 逐位同 | 逐位同 | — | 3/3 逐位 |
| 原始云 `d` / 法向 vs +z | 0.2579 / 0.082° | 0.2592 / 0.016° | 0.2596 / 0.037° | 0.2592 / 0.037° | 1.7 mm / 0.066° |
| **累积地图地面 tilt vs +z** | **0.048°** | **0.054°** | **0.030°** | **0.048°** | 0.024° |
| **累积地图地面 `d`**（世界系） | 0.2276 | 0.2257 | 0.2250 | **0.2257 m** | 2.7 mm |
| 累积平面 inliers（201 帧 / ~28.8k 点） | 6586 | 6599 | 6576 | — | — |
| `h_meas`（SPL 式地面互检） | **不可用**（`/cloud_registered` 143 点/帧 < 500）⇒ `odom_level_tf` WARN「91 s 内没拿到可用点云，静态边用 URDF 值」 | 同 | 同 | — | — |

`GT base_link z = 0.102495 m`（`/odom_ground_truth`；与 §32/§33 同口径）。`h_imu = 0.216226 m`
⇒ 地图地面 `d = 0.2250–0.2276` = **`h_imu` + 8.8–11.4 mm**（point_lio 自己的 state z 误差）。

### 35.5 与旧 pointlio 数字（全部 `R=I`）对比：**nav 不变，地图/初始化变**

⚠️ **§33 没有 pointlio 臂**（4 臂都是 SPL 的 gravity 档）。pointlio 的旧基线是
**§30.5.2（1 跑）**与 **§32.4（3 跑）**；§34 的 pointlio 只用于「初始化图 vs 逐帧图」回放
（§34.3#4 记了 `R=I`）。

| 量 | §30.5.2（R=I，1） | §32.4（R=I，3） | **§35 after（3）** |
|---|---|---|---|
| `base_link` z | +0.0298 / +0.0362 | +0.0961 / +0.0964 / +0.0974 | **+0.0920 / +0.0943 / +0.0950** |
| 换算 `floor_z` | **−7.3 / −6.6 cm** | −6.4 / −6.1 / −5.1 mm | **−10.5 / −8.2 / −7.5 mm** |
| `odom_level→base_link` rpy | roll/pitch ≤0.14° | ≤0.10° | **≤0.212°** |
| local lethal | 238 / **0（空图）** / 317 | 227 / 332 / 213 | **202 / 236 / 206** |
| local 最近 lethal | 0.7286 / — / 0.7068 | 0.7035 / 0.7091 / 0.7051 | **0.7028 / 0.7006 / 0.7013** |
| local ahead | 21 | 18 / 20 / 18 | **19 / 18 / 19** |
| in-band | 100% | 100% | **100%** |
| 地图地面法向 | **未测**（稀疏云 <500 点） | **未测**（同） | **0.030–0.054°（水平）** |
| 地图地面 `d` | 未测 | 未测 | **0.2250–0.2276 m**（= h_imu + 9–11 mm） |

**读法**：
1. **nav 数字没有因为 R=I 变**：in-band 100%、lethal 202–236、nearest 0.7006–0.7028、ahead 18–19
   全部落在 §32（lethal 213–332、nearest 0.7035–0.7091、ahead 18–20）的跑间波动内。
   原因**不是巧合**：nav 的 `/scan` 来自 `/segmentation/obstacle`（原始云 + linefit，§4.12 之后
   点云本身水平），`local_costmap` 只在 `odom_level` 里做一次 TF；**LIO 的 extrinsic 没有进入
   这两条链路**。所以 §30/§32 的 pointlio 导航结论**仍然有效**。
2. **但地图/估计器内部以前是不自洽的**：`R=I` 时地图地面 = `rot_init·(水平点云)`，法向偏
   30.05°（§35.6 实测）；`/cloud_registered`（gicp 的 source 云）与重力对齐的 `odom` 差 30°。
   这解释了为什么 §32 只能用 base_z − GT **换算**地面、而量不到法向：不是"地面是平的所以不用量"，
   是"稀疏云量不了"。
3. **z 锚点低 2–5 mm**（§32 −5.1..−6.4 → 本轮 −7.5..−10.5 mm）：同一机制（pointlio 首帧后
   0.5 s 就 `Set_init`，车还在落定），与 extrinsic 无直接关系；3 跑里 r1 **−10.5 mm** 略超
   `0±1 cm` 判据（0.5 mm），r2/r3 在带内。**没有**为了这个 1 cm 去改 pointlio 的初始化门
   （契约 §4.11.6 明确不在范围内）。
4. **§34.1 的回放结论不受影响**：它比较的是"初始化图"与"逐帧图"两条路径，两者用**同一组**
   `R=I` 外参 ⇒ 逐位相同仍然成立；但它的**绝对几何**建立在一张 30° 斜的地图上（§34.3#4 已记）。

### 35.6 **修复前**的地图地面：实测 30.05°（旧 launch + 旧 yaml，1 跑）

用 HEAD 的 launch（只传顶层死键）+ HEAD 的 pointlio yaml（`mapping.extrinsic_R=I`）跑
`mode:=mapping lio:=pointlio robot:=robot11 robot11_mount:=sensor world:=RMUL2026`，累积
`/cloud_registered` 20 仿真秒（201 帧 / 28651 点）后拟合：

| 量 | **修复前**（`g35_before_map_pl`） | **修复后**（r1/r2/r3） |
|---|---|---|
| 运行期 `mapping.extrinsic_R` | **I** | **Rx(+30°)** |
| 累积地图地面（内点最多的平面） | n = **(−0.0205, +0.5003, +0.8656)**、**tilt 30.05°**、d = **0.0536 m**、7018 inliers | n ≈ **(0, 0, 1)**、tilt **0.030–0.054°**、d = **0.2250–0.2276 m**、~6.6k inliers |
| `+z` 先验能不能找到"水平地面" | **不能**：`plus_z` 先验收敛到**同一个 30° 平面**（内点更多）⇒ 图里**没有水平地面** | 能：三个先验（+z/±30°）全部收敛到水平面（并列） |
| `rot_x_+30` 先验 | **None**（没有 −30° 法向的平面） | 与 +z 同 |
| 静态边 `odom→odom_level` | 仍按 `input_world=level` 发 I/(0,0,0.216226)（**前提不成立**） | 同一条静态边，但前提现在成立 |

⚠️ **诚实口径**：修复前那个平面的 `d=0.0536 m` **不是**物理 `h_imu`（0.2162）也不是单纯的
`rot_init` 旋转（那会给 0.2595）——说明 `R=I` 不只是"把地图转斜 30°"，估计器在斜地图里还把
**高度尺度/原点**拉歪了（这正是"静默错"的可怕处）。本轮只报测量值，**不**把 0.0536 分解成
"旋转 + 平移"；结论只用**法向**（30.05° vs 0.03–0.05°）与"图里有没有水平地面"。

### 35.7 默认路径有效参数不变（fastlio / SPL）

`mode:=mapping` + **默认模型** + `gravity` 默认（`tf`），各 1 跑（`g35_def_fastlio` /
`g35_def_spl`），运行期回读：

| 槽位 | 节点 | 键 | 修复后生效值 | 修复前（= yaml / launch 的旧值） | 断言 |
|---|---|---|---|---|---|
| `lio:=fastlio` | `/laser_mapping` | `mapping.extrinsic_T` / `mapping.extrinsic_R` | `[0,0,0.05]` / **`I`** | 同（yaml 默认） | **OK** |
| `lio:=small_point_lio` | `/small_point_lio` | 顶层 `extrinsic_T` / `extrinsic_R` | `[0,0,0.05]` / **`I`** | 同（launch 旧值逐字段相同） | **OK** |

`ros2 launch … --show-args` = **107 个参数**（与 §33 的 107 逐字相同，没有新增 launch 参数）。
⇒ 默认路径（默认模型 + fastlio/SPL）的**有效参数不变**；变化只发生在 robot11（fastlio/pointlio
现在真的拿到 Rx(+30°)）与 **键名**（顶层 → `mapping.*`）。

### 35.8 运行期断言：`lio_extrinsic_receipt.py`

- **为什么不是 launch 期自检**：launch 只能证明"我传了什么"，证明不了"节点读的是什么"；
  `ros2 param get` 读的才是 `declare/get_parameter` 看到的那一份。
- 机制：launch 在每个 LIO 的 Node 后起一个**一次性**节点（`rm_nav_bringup/lio_extrinsic_receipt.py`），
  参数 `target_node` / `param_prefix` / `expected_T` / `expected_R` **与 `_LIO_EXTRINSIC_KEYS`
  同源**（键名只有一处可改）；它等 `get_parameters` 服务 → 读回**生效的**两个键 →
  与 URDF 解析值逐元素比（`tol=1e-9`）→ 另用 `list_parameters` 查另一个命名空间有没有**死键** →
  打 `[lio_extrinsic_receipt] OK/ERROR`，不一致/缺失/类型错/死键/超时 ⇒ **exit 1**。
- **合成三例**（`.tmp_g35/receipt_{good,identity,dead}.log`）：好节点 **rc=0**；`R=I` 节点
  **rc=1** 且消息给出"实际 = **I**：安装角根本没进来（典型死键症状）"；带顶层死键的节点
  **rc=1** 且指名 `**死键** extrinsic_T 存在`。
- **真跑**：r1/r2/r3 + fastlio/SPL 默认档 = **5/5 OK**（§35.3/§35.7）。
- **故意不自动收栈**：exit 1 只会让 launch 打出红色 ERROR + `process has died`；宁可让人看见，
  也不因一次慢机器的 90 s 超时误判把整栈收掉。

### 35.9 诚实清单

1. **任务前提有一处反了**：fastlio 不是"顶层 `extrinsic_T/R`"，它和 pointlio 一样只读
   `mapping.extrinsic_*`（`laserMapping.cpp:834–835/870–871`）。⇒ **fastlio 也一直是死键**，
   本轮一并修；**但 fastlio 没有重跑**（任务只要 pointlio 3 跑）⇒ **§32.7 的 fastlio `f1`
   数字（robot11）也是 `R=I` 下量的**（base_link z 0.0409、local lethal 404、nearest 0.7089），
   它们现在**过时**了，若要用 robot11 + fastlio 必须重测。
2. **§33 没有 pointlio 臂**：旧基线是 §30.5.2（1 跑）与 §32.4（3 跑）；不要把 §33 的 SPL 数字
   当 pointlio。
3. **before 的 nav 基线是本轮补跑的 `g35_before_pl`**（HEAD 代码 + §34 协议），不是 §34 的
   任何一跑（§34 的 6 跑是 SPL）；它和 §32.4 同量级（lethal 235 vs 213–332、nearest 0.7038 vs
   0.7035–0.7091），可作 before。
4. **before 地图地面 `d=0.0536 m` 的物理解释未做**（§35.6 的 ⚠️）：只用法向与"有没有水平地面"
   作结论。
5. **3 跑 n=3**，同机同时段、同一 launch md5；跑间 spread 已给（lethal 34、nearest 2.2 mm、
   base_z 3.0 mm、地图 d 2.7 mm）。不许把 3 个点的 median 当精确值。
6. **断言的死键探测走 `list_parameters`**（避免 rclcpp 对未声明名打 `Failed to get parameters`
   的假 WARN）；若某实现返回空名字列表，死键探测会**跳过**，但"生效值 == URDF"这条主判据
   仍会执行。
7. **断言节点让默认路径的图里多一个瞬态节点** `lio_extrinsic_receipt`（启动后 ~1 s 退出，
   `process has finished cleanly`）；**有效参数**不变（§35.7），但 `ros2 node list` 的启动段
   会短暂多它一个。
8. **pointlio 的 `h_meas` 仍然不可用**：`/cloud_registered` 143 点/帧 < 500 ⇒ SPL 式地面互检
   与 `odom_level_tf` 的 `gravity_align_h_check` 都是 WARN 跳过（3/3）。本轮用**累积云拟合**
   + 原始云 `d` + GT 三条独立证据代替。
9. **`/cloud_registered` 帧名是 `camera_init`**（数值上 = `odom`，adapter/静态边都把 `odom`
   当输入世界）。累积拟合把 `camera_init` 坐标直接当 `odom` 用；两者在 t0 同源，本轮 3 跑 +
   before 跑都验证了静态边/`odom→base_link` 与这个假设一致（base z/地面都在 1 cm 内）。
10. **没有改**地图/PCD 资产、`src/rm_simulation/**`、`docs` §1–§34 正文；`src/rm_localization/**`
    只改了 pointlio yaml 一个文件（子模块提交 `d281584`），LIO 的 C++ 一行没动。
11. 每次 pointlio 跑都会往 `src/rm_localization/point_lio/PCD/` 写 `scans_*.pcd`（包内
    `pcd_save_en: true` 的既有行为，§34 之前就在）；**没有**提交这些产物。
12. 修复后的 launch 让 `lio:=pointlio` 在 **robot11** 上第一次真正按 30° 外参跑；**`lio:=pointlio`
    的非 robot11 槽位**（默认模型/hzmirm）仍是 `I`（launch 从 URDF 解析），没有被这一轮改变。

## §36 第二十六轮（**本机 date=2026-10-27，09:11–09:32**）：fastlio 的 `mapping.extrinsic_R = Rx(+30°)` 落地后 **robot11 3 跑重测** —— `odom_level` 地图地面 **28.4–29.3° → 1.1–1.9°**，nav 结论不变

> **本节只新增**（append-only）：§1–§35 一个字没改；§32.7 只加了一条"已被本节取代"的指针
> （它的内容不动）。任务（用户批准）：`cdcd7e3` 修好了 per-LIO 外参键名（fastlio 读
> `mapping.extrinsic_*`，见 §35.1），但 §32.7 的 fastlio robot11 数字（base z 0.0409 /
> lethal 404 / nearest 0.7089 …）是 **`R=I`** 时量的（§35.9#1 明记）⇒ 按 §34.2.1 协议重跑
> 3 次、量地图地面、与 §32.7 对比并把旧数字标成"已被取代"。
>
> **一句话结论**
> 1. **修复真的生效**：3/3 跑运行期 `mapping.extrinsic_R = Rx(+30°)`（逐元素，`tol=1e-9`）、
>    断言 **OK 3/3**、顶层死键 `Parameter not set`；旧 launch（= `R=I`）2 跑回读 `I`
>    （§36.2 / §36.3）。
> 2. **判据帧是 `odom_level`，不是 raw `odom`**：fastlio 的 `odom` 是 **IMU 原始世界**
>    （launch 收据 `input_world=tilted`），所以正确外参下 raw `odom` 里的地面**本来就该 ≈30°**
>    （3 跑 30.96 / 31.10 / 31.86°）——这不是错误。把点搬进 `odom_level` 后：
>    **修复前 28.38 / 29.34° → 修复后 1.60 / 1.10 / 1.90°**（3 跑，累积 `/cloud_registered`
>    拟合）。这是本轮**唯一的结构性变化**。
> 3. **没有变成 pointlio 的 0.03–0.05°**（§35.4）：fastlio **不做重力对齐**（初始 `R=I`，
>    §4.14 的 `input_world=tilted`）⇒ 残余 1.1–1.9° 是它自己的姿态误差；同跑的
>    `odom_level→base_link` roll 0.82 / 1.07 / 1.80° 与地图地面残余同量级（两条独立证据）。
> 4. **nav 的判据级三项没有变**：in-band 生效带 **100%（`band_guard` 11/11 采样 × 3 跑）**、
>    最近 lethal 0.701–0.713 m、前方 2.0×0.6 m 24–37 格（§32.7 的 0.7089 / 35 与 before 的
>    0.7109–0.7119 / 33–36 都在内）。**但 `local lethal` 有一档未定的下移**：`R=I` 的 3 个样本
>    （349 / 378.5 / 404）全部高于修复后的 3 个样本（282 / 309.5 / 323），下移 15–25%；8 个
>    nav 指标里只有这 1 个完全分离，n=3v3 不足以定论（见 §36.4#2），但也不写成"逐位不变"。
> 5. **§32.7 的 fastlio 数字全部标为"`R=I` 时代，已被 §36 取代"**。

### 36.1 协议、隔离与冻结（= §34.2.1 / §35.4 逐字同）

* 命令：`world:=RMUL2026 mode:=nav lio:=fastlio localization:=gicp robot:=robot11
  robot11_mount:=sensor spin_speed:=0.0 band_guard:=True nav:=mppi planner:=smac2d
  nav_rviz:=False lio_rviz:=False gui:=False`；`SETTLE=20 WINDOW=20`（**仿真秒**）。
* 隔离：私有 `HOME` / `ROS_LOG_DIR` / `ROS_DOMAIN_ID`（按 tag 哈希）、先探空端口再占
  `GAZEBO_MASTER_URI`、`unset DISPLAY`、`setsid`（进程组 = 本 run）、每跑硬超时 1100 s、
  收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）、**一栈一跑**（跑间 8 s）。
  7 跑（3 after nav + 2 before nav + 2 mapping）收尾后 `sc_own_leftovers_desc` 全空、`gz=0`。
* 冻结：所有跑的 `git_head = ffeca1e`（= `origin/master`）；当前 launch md5
  **`740dbeb5a1bb4b72d3d74eef967457b0`**；fastlio yaml md5 **`fea68c66…`**
  （yaml 里 `mapping.extrinsic_R` 默认仍是 `I`，robot11 由 launch 从 URDF 覆盖成 Rx(+30°)）；
  断言脚本 md5 **`b425b7c8…`**。
* 探针：`nav_band_probe.py --scan-frame odom_level`（+ `uc_procstats.py`）、`level_probe.py`
  （TF + 原始云）、`cloudacc_probe.py`（§35 同款：累积 `/cloud_registered` 拟地面）、
  `band_guard` 收据、运行期 `ros2 param get` 回读。原始数据 `.tmp_g36/g36_fl_r{1,2,3}/`（gitignored）。
* **修复前对照（2 跑 nav + 1 跑 mapping）**：`g36_before_fl` / `g36_before_fl2`（nav，探针全套）与
  `g36_before_mapcheck`（mapping，`/Laser_map` 交叉核验）都用
  `git show 550f596:src/rm_nav_bringup/launch/bringup_sim.launch.py`
  （md5 **`6ad0aaaedc18486dc8318ce235c7b738`** = §34.2.1 的冻结值）。三个子命令（launch/探针/收尾）
  与修复后相同，唯一差异是 launch 里的外参键（旧 = 顶层死键）。运行期回读
  `mapping.extrinsic_R = I`、顶层 `extrinsic_R/T = Parameter not set`（**死键**，与 §35.3 的
  before 逐字同）；`g36_before_fl` 的 `/odom` 末态 z = **−0.0596** 与 §32.7 f1 的 **−0.0593**
  逐位同量级 ⇒ 对照跑复现了 §32.7 的 `R=I` 条件。
* **修复后 map 交叉核验（1 跑 mapping）**：`g36_mapcheck`，当前 launch，`/cloud_registered`
  累积 + `/Laser_map`（节点自己的 ikdtree 全图，1 Hz）+ `/odom` 的 z 迹，并存抽样点云
  `.npz` 供离线复核。

### 36.2 fastlio robot11 3 跑（修复后；`mapping.extrinsic_R = Rx(+30°)`）

| 指标 | r1 | r2 | r3 | median | range |
|---|---|---|---|---|---|
| 运行期 `mapping.extrinsic_R` 回读 | **Rx(+30°)** | **Rx(+30°)** | **Rx(+30°)** | — | 3/3 |
| `lio_extrinsic_receipt` | **OK** | **OK** | **OK** | — | 3/3（`tol=1e-9`） |
| 顶层死键 `extrinsic_T/R` | `Parameter not set` | 同 | 同 | — | 3/3 |
| 局部 in-band（**生效带** `[-0.05,2]`） | **773/773** | **773/773** | **773/773** | **100%** | 100–100 |
| `band_guard`（local） | **OK 11/11** | **OK 11/11** | **OK 11/11** | **100%** | 100–100 |
| 探针 in-band（工具自己的 `[0,2]`） | 773/773（below 0） | 773/773（0） | 773/773（0） | 100% | 100–100 |
| 扫描面 z p50（`odom_level`） | +0.0823 | +0.0734 | +0.0503 | **+0.0734** | 32.0 mm |
| local lethal | 282 | 309.5 | 323 | **309.5** | 41 |
| local inscribed / free | 10221.5 / 40427 | 10160 / 40431 | 10331 / 40148.5 | 10221.5 / 40427 | 171 / 282.5 |
| local **最近 lethal** | 0.7014 | 0.7060 | 0.7127 | **0.7060 m** | 11.4 mm |
| local 前方 **2.0×0.6 m** lethal | 35 | 37 | 24 | **35** | 13 |
| local **1 m 内** lethal | 41 | 46 | 32 | **41** | 14 |
| global lethal | 2094 | 2109.5 | 2138 | **2109.5** | 44 |
| global 最近 lethal | 0.6254 | 0.6985 | 0.7436 | **0.6985 m** | 118.2 mm |
| **`base_link` z（`odom_level`）** | +0.0793 | +0.0690 | +0.0482 | **+0.0690** | 31.1 mm |
| 换算 `floor_z` = base_z − GT | **−23.2 mm** | **−33.5 mm** | **−54.3 mm** | **−33.5 mm** | 31.1 mm |
| `odom_level→base_link` rpy（deg） | [0.824, 0.019, 0.023] | [1.072, 0.072, 0.477] | [1.799, −0.336, 0.211] | — | roll 0.82–1.80 |
| `odom→base_link` rpy（deg） | [30.824, 0.032, 0.017] | [31.071, −0.183, 0.411] | [31.799, −0.387, −0.005] | — | roll 30.82–31.80 |
| 静态边 `odom→odom_level`（发布值） | Rx(+30°) / (0, 0.108, −0.187) | 逐位同 | 逐位同 | — | 3/3 逐位 |
| 原始云 `d` / 法向 vs +z | 0.2580 / 0.102° | 0.2593 / 0.021° | 0.2593 / 0.020° | 0.2593 / 0.021° | 1.3 mm / 0.08° |
| `h_meas`（SPL 式地面互检） | **不可用** | 同 | 同 | — | 3/3 WARN |
| `GT base_link z`（`/odom_ground_truth`） | 0.102498 | 0.102490 | 0.102498 | 0.102498 | — |
| `/odom` 末态 z（state.z） | −0.0753 | −0.0857 | −0.0863 | −0.0857 | 11.0 mm |
| **`odom_level` 地图地面 tilt**（累积云） | **1.603°** | **1.103°** | **1.899°** | **1.603°** | 0.80° |

`h_imu = 0.216226 m`。`h_meas` 3/3 都是 `odom_level_tf` 的
`gravity_align_h_check: 91 s 内没拿到 /cloud_registered 的可用点云 ⇒ 跳过互检；静态边改用 URDF 值`
（fastlio 的 `/cloud_registered` 只有 ~145 点/帧 < 500，与 §32.9#5 / §35.9#8 同因）⇒ 用
**原始云 `d`（0.2576–0.2597 m）+ GT** 作替代证据；另一条自洽证据是扫描面 z 与
`base_link` z 只差 **2.1–4.4 mm**（同一条 TF 链的两个读数，本就该同高）。

**读法**

1. **in-band 是 100%，且探针的 `below` 为 0**：本轮三跑的扫描面 z p50 = +0.050…+0.082，
   整段都落在生效带 `[-0.05, 2.0]` 内，`band_guard` 的 11 次采样也是 `OK 11/11`、kept-y 双侧。
   （§32.7 时代探针报 `below=65` 是它自己的 `[0,2]` 带造成的口径假象，§32.9#9；本节 before 跑
   同样复现：`below=25/72`，而 `band_guard` 都是 100%。）
2. **`odom→base_link` ≈ +30°，`odom_level→base_link` 小**：与契约一致（fastlio 不重力对齐 ⇒
   raw `odom` 是斜的 IMU 世界；重力对齐由静态边负责）。但 `odom_level→base_link` 的 roll
   并不稳定在 <0.5°（0.82/1.07/1.80°），这是 **fastlio 自己的姿态误差**，不是静态边——静态边
   3/3 逐位相同（Rx(+30°)、t=(0,0,0.216226)）。
3. **`floor_z` 仍在 −2…−5 cm 档**（§32.7 是 −61.6 mm；本轮 −23…−54 mm，同一档略高）：
   fastlio 没有 z 锚点（§32.7 已经量过），本轮没有为它加锚点，也不在本任务范围。
4. **重复性**：3 跑里 `scan z p50` 极差 32 mm、`base_z` 极差 31 mm、`local lethal` 极差 41
   （13%）、`global nearest` 极差 118 mm ⇒ 这 3 个点的 median 不是精确值（§36.5#1）。

### 36.3 地图地面：before / after（判据帧 = `odom_level`）

**方法（与 §35.4/§35.6 同款）**：把 settle 后 20 仿真秒内的**所有** `/cloud_registered` 帧累积
（201 帧 / ~28.8k 点），用 RANSAC+EVD 拟合"离原点 0.03–1.0 m 的大平面"（先验：+z 与 ±30°，
取内点最多的）；再经静态边 `odom_level←odom` 把点搬进 `odom_level` 重拟。同一批跑还做了
**`/Laser_map`**（fastlio 节点自己的 ikdtree 全图，1 Hz，`map_en: true`）与**近场**（0.8–2.0 m）
两个独立复核，并存了抽样点云离线复算（`.tmp_g36/maptable.py`）。

| 量 | **before（`R=I`）** | **after（`Rx(+30°)`）** | 判读 |
|---|---|---|---|
| 运行期 `mapping.extrinsic_R` | **I**（顶层死键） | **Rx(+30°)**（断言 OK） | 修复点 |
| 累积地图地面：raw `odom` tilt | 3.41° / 4.15°（2 跑 nav） | 30.96 / 31.10 / 31.86°（3 跑 nav） | raw `odom` 是斜的 IMU 世界 ⇒ after 的 ~31° 才是**物理正确** |
| 同上，near-field 0.8–2.0 m | 2.09°（cloud）/ 2.07°（`/Laser_map`） | 31.13°（cloud）/ 31.11°（map） | 与 whole-range 同结论 |
| 累积地图地面：**`odom_level` tilt** | **28.38° / 29.34°**（2 跑 nav） | **1.603 / 1.103 / 1.899°**（3 跑 nav） | ⭐ **唯一的结构性变化** |
| 同上，near-field 0.8–2.0 m | **29.07°（cloud）/ 28.91°（map）** | **1.13°（cloud）/ 1.12°（map）** | 近场干净平面 = 同结论 |
| 平面 `d`（raw `odom`，离原点） | near-field **0.2397 / 0.2457**；whole-range 0.1143 / 0.0758 | near-field **0.2433 / 0.2447**；whole-range 0.0738 / 0.2441 / 0.2656 | **near-field** after ≈ before（差 ≤4 mm）⇒ **外参没有改 z 锚点** |
| `d − h_imu` | **+23.5 / +29.5 mm** | **+27.1 / +28.5 mm** | 同档（fastlio 自己的 map/pose 自洽 ~2–5 cm） |
| `odom_level` 地面 `floor_z` | near-field −0.058 / −0.064 | near-field −0.027 / −0.029 | 同量级（fastlio 的 z 漂） |
| 内点 / `resid_p50` | whole 6864–7148 / **8.4–9.7 mm**；near/map 1.3–3.0 mm | whole 6407–6891 / **2.1–8.8 mm**；near/map 1.4–2.4 mm | 见下面的"抹开" |
| 帧数 / 点数 | 201 / 28709–28794 | 201 / 28737–28833 | 同口径 |
| 静态边 `odom_level←odom` | Rx(−30°) / (0, 0, 0.216226) | 逐位同 | 3/3 逐位 |

**读法（这是本节最需要说清的三点）**

1. **before 的 28.4–29.3° 是"LIO 地图与帧图契约不自洽"的直接证据**：`R=I` 时 fastlio 把
   **水平的点云**当成 **IMU 系**点云用，地图在该世界里的地面法向只有 2–4°，可这个世界本身是
   **斜 30° 的 IMU 世界** ⇒ 静态边（正确的 Rx(+30°)）把地面搬进 `odom_level` 后变成
   **28.4–29.3°**。nav 的 `/scan` 不经过这张地图，所以**导航没坏，地图坏了**——与 §35.6 对
   pointlio 的 30.05° 是同一类病，只是 fastlio 的 raw `odom` 约定不同（pointlio 的静态边是
   `I`，所以它的 30° 直接出现在 raw 与 level 两个系里）。
2. **after 的 1.1–1.9° 不是 0.03–0.05°**：fastlio **没有重力对齐**（初始 R=I，`input_world=tilted`），
   它的姿态自身就有 1–2° 的误差——同一跑的 `odom_level→base_link` roll 是 0.82/1.07/1.80°，
   与地图地面的残余 1.60/1.10/1.90° 一一对应。pointlio 能到 0.03–0.05°，是因为它的
   `ImuProcess::Set_init` 真做了重力对齐（§35 的 3 跑）；**不能拿 pointlio 的 0.03° 当 fastlio 的
   验收线**。正确说法：**30° 的系统性倾斜消失了，剩下的是 LIO 自己的 1–2° 姿态误差**。
3. **whole-range 的 `d` 不能当判据**：fastlio 的姿态在 20 s 窗口内会漂（`g36_mapcheck` 的
   `/odom` rpy 实测 roll 0.99→1.24°、yaw 0.21→−0.66°），远场（4–8 m）地面点被漂移抹开
   （0.5° × 4–8 m ≈ 3.5–7 cm），形成第二层"平行、更薄、resid ~9 mm"的带；whole-range RANSAC
   会锁到这一层（d 0.074–0.114），**近场 0.8–2.0 m** 才是干净平面（resid 1.3–1.4 mm，
   d=0.243，与 `/Laser_map` 的 0.2447/2.4 mm 一致）。因此本节以 **tilt** 作主判据，`d`
   只作"before/after 没有量级变化"的辅证。pointlio 没这个问题是因为它的姿态在窗口内 ≤0.2°。

### 36.4 与 §32.7 的对比（**§32.7 = `R=I` 时代，已被 §36 取代**）

> §32.7 是 **1+1 A/B**（旧 spawn vs 新 spawn）里的 `f1_fastlio_spawnz`；它的 fastlio 数字量于
> 2026-10-21、`R=I`（外参死键）。本节 = 2026-10-27 的 3 跑 after + 2 跑 before（同日同协议）。

| 量 | §32.7 f1（`R=I`，1 跑） | **本轮 before（`R=I`，2 跑）** | **§36 after（3 跑）** | 判读 |
|---|---|---|---|---|
| `base_link` z（`odom_level`） | 0.0409 | 0.0563 / 0.0832 | **0.0482 / 0.0690 / 0.0793** | §32.7 比本轮 5 跑的**下界**还低 7 mm（单跑 + spawn A/B 的余波） |
| 扫描面 z | 0.0517 | 0.0630 / 0.0878 | 0.0503 / 0.0734 / 0.0823 | 同档 |
| `odom_level→base_link` rpy | [1.86, −1.63, −0.15]° | [1.56, −1.65, 0.07] / [0.88, −2.72, 0.46]° | [0.82, 0.02, 0.02] / [1.07, 0.07, 0.48] / [1.80, −0.34, 0.21]° | 同族（fastlio 自身姿态误差 1–3°） |
| 生效带 in-band | 100% | **100%** | **100%** | 不变 |
| local lethal | **404** | 349 / 378.5 | **282 / 309.5 / 323** | `R=I` 的 3 个样本全高于 after 的 3 个（↓15–25%）⇒ 可能有一档真实下移；n=3v3 不足以定论（正文#2） |
| local 最近 lethal | 0.7089 | 0.7119 / 0.7109 | **0.7014 / 0.7060 / 0.7127** | 同档（≤1.2 cm） |
| local ahead（2.0×0.6 m） | 35 | 36 / 33 | 35 / 37 / 24 | 同档 |
| global lethal | 1970 | 2140 / 2085 | 2094 / 2109.5 / 2138 | §32.7 低 6–8%，单跑 |
| global 最近 lethal | 0.7696 | 0.7521 / 0.5982 | 0.6254 / 0.6985 / 0.7436 | §32.7 略高于本轮 5 跑的 before 上界 0.752（+1.7 cm，n=1） |
| **`odom_level` 地图地面 tilt** | **未测**（sparse cloud <500 点，只能换算） | **28.38 / 29.34°** | **1.10–1.90°** | ⭐ 唯一结构性变化 |
| 地图地面 `d` | 未测 | near-field 0.2397 / 0.2457 | near-field 0.2433 / 0.2447 | 不变（差 ≤4 mm） |
| `h_meas` | 不可用 | 不可用（3/3 WARN） | 不可用（3/3 WARN） | 同因（~145 点/帧） |

**判定：有没有 nav 结论翻转？——没有。**

1. **判据级三项全部同档**：in-band 生效带 3/3 = 100%（`band_guard` 11/11 × 3）、最近 lethal
   0.701–0.713 m（§32.7 0.7089、before 0.7109–0.7119 都在内）、前方 2.0×0.6 m 24–37 格
   （§32.7 35、before 33–36 都在内）。
2. **`local lethal` 有一档 15–25% 的下移，但只能记为"未定"**：`R=I` 的 3 个样本是
   **404（§32.7，1 跑）/ 349 / 378.5**，修复后的 3 个是 **282 / 309.5 / 323**——**3v3 完全分离**。
   它有一个同向的机制候选：before 的 `odom_level→base_link` **pitch = −1.65°/−2.72°**，
   after 是 **+0.02°/+0.07°/−0.34°** ⇒ 扫描面在代价图帧里的俯仰不同，命中"lethal 格"的数量
   跟着变。**但**：8 个 nav 指标里只有这 1 个完全分离（`global lethal` 2140/2085 vs
   2094–2138 有重叠；`global 最近` 0.598–0.752 vs 0.625–0.744 有重叠；`base_z`、`scan z`、
   `nearest`、`ahead`、`in-band` 都有重叠），n=3v3 且跨了一个 spawn A/B 实验
   ⇒ **不宣称"修复让 lethal 系统性变好"**，也不宣称"完全不变"；判据级结论没有翻转。
3. **唯一能确定的变化是地图地面**：`odom_level` 里 **~29° → ~1.1–1.9°**。原因不是 nav 链路变了，
   而是 **`/scan` = 原始云 + linefit + TF，LIO 的 `mapping.extrinsic_R` 根本不进这条链路**
   （与 §35.5 对 pointlio 的结论同因）；所以 §30/§32 的 **nav 数字仍然有效**，但"fastlio 的
   odom/地图是自洽的"这个隐含前提到 `cdcd7e3` 之前**不成立**。
4. **`base_link` z / `floor_z` 没有系统性移动**：§32.7 0.0409（−61.6 mm）、before 0.0563/0.0832
   （−46/−19 mm）、after 0.0482–0.0793（−54…−23 mm）全在同一档；fastlio 没有 z 锚点这件事
   **没有**被外参修复改变。

### 36.5 诚实清单

1. **n=3 after + 2 before（nav）+ 1 before（mapping）**：fastlio 的跑间波动很大（本轮 base_z
   极差 35 mm、lethal 极差 96、global nearest 极差 154 mm）⇒ **不能**把 10–25% 的 lethal 差异
   归因给外参；本节只声明"没有结论级变化"，不声明"逐位不变"。
2. **whole-range 地图地面 `d` 被 fastlio 自己的姿态漂移污染**（§36.3#3）：同一批点，whole-range
   RANSAC 得到 d=0.074–0.114 / resid ~9 mm，近场 0.8–2.0 m 得到 d=0.243 / resid 1.4 mm，
   `/Laser_map` 得到 0.2447 / 2.4 mm。**tilt 是稳健的**（两条路径差 ≤0.5°），`d` 只作辅证。
   没有把 `d` 硬解释成"z 锚点"。
3. **残余 1.1–1.9° ≠ pointlio 的 0.03–0.05°**：fastlio 不做重力对齐；残余与
   `odom_level→base_link` 的 roll/pitch 同量级。这不是外参没修好，而是 **LIO 自身姿态**的
   1–2° 误差。若要把 fastlio 也做到 <0.5°，得改它的初始化/重力对齐（`src/rm_localization/**`，
   本任务禁改）。
4. **`h_meas` 3/3 不可用**：`/cloud_registered` ~145 点/帧 < 500 ⇒ `odom_level_tf` 的
   `gravity_align_h_check` WARN 跳过（与 §32.9#5 / §35.9#8 逐字同）。替代 = 原始云 `d`
   （0.2576–0.2597，法向 0.020–0.112°）+ GT 0.102498 + TF 自洽。
5. **`nav_band_probe.py` 自己的带仍是 `[0,2]`**（不是代价图生效带 `[-0.05,2]`）：before 两跑
   分别报 `below=25 / 72`，而 `band_guard` 是 **100%**（z min = −0.025 / −0.011，都在 −0.05 之上）。
   权威口径 = `band_guard`（读活栈参数服务），与 §32.9#9 同。
6. **`ahead` 窗口的口径**：工具里 `AHEAD_X, AHEAD_Y = 2.0, 0.6`（`nav_band_probe.py:43`），
   本节按 **2.0×0.6 m** 写；§34.2.1/§35.4 正文写的是 "2.0×1.2 m"（那是文字笔误，数值仍是同一
   工具的 0.6 窗口）。§32.7 的 35 与本轮 24–37 可直接比。
7. **before 用的 launch 不是当前 launch**：`git show 550f596:…`（md5 6ad0aaa…，= §34.2.1 冻结值），
   与 `cdcd7e3` 的差异已用 `git diff 550f596..HEAD -- launch` 核对：只有外参键/receipt/日志，
   没有别的行为变化；`mapping.extrinsic_R = I` 与顶层死键的运行期回读是直接证据。
8. **`spawn_entity.py` 的 "Spawn service failed. Exiting." 每跑都有**（§34/§35 同样）：实体实际
   生成成功（`/odom_ground_truth`、`/scan`、`/cloud_registered` 都有数据，GT z=0.1025），
   是既有的良性时序消息（§32.9#13）。收尾 `sc_own_leftovers_desc` 全空。
9. **`pcd_save_en: true` 没有落盘**（fastlio 的 `install/.../share/` 不存在，`save` 路径无效）；
   仓库里没有新增 PCD 产物。
10. **没有改**地图/PCD 资产、`src/rm_simulation/**`、`src/rm_localization/**`、默认路径
    （`lio:=small_point_lio` 一个字节没动）；本轮提交只改 `docs/frame_contract_v3_measurements.md`
    （§36 + §32.7 一行指针）。
11. **`map→odom` 的 rpy ≈ −32°（GICP 初始对齐）在 before/after 都存在**（§35 的 pointlio 也有
    ~−3° pitch 的同族现象），不是本轮的验收面；nav 的局部代价图不依赖它。
12. **`/odom` 末态 z 与地图 `d` 只能做 2–5 cm 级的自洽检查**：fastlio 的 z 在跑内会漂
    （after 三跑末态 −0.075/−0.086/−0.086，before 两跑 −0.060/−0.021），`d` 的绝对值随之
    有 2–5 cm 的不确定度——这也是为什么本节用 **tilt** 而不是 `d` 作主判据。

---

## §37 第二十七轮（**本机 date=2026-10-27，10:03–10:35**）：**15° 坡道上「高度带该放在哪个帧」的判决** —— `base_link` 的 z 是**地形相对**（车体与坡面平行时误差 **0.0 cm**），`odom_level` 的固定带是**海拔**（误差 = **x·sin15°**）；**但车体不平行于地形时两者都错**；两种姿态都对的是 `ground:=linefit`

> **本节只新增**（append-only）：§1–§36 一个字没改。
> 它**部分推翻** `docs/frame_contract_v3.md` §4.11.8 的一条断言（"`base_link` 不能用来做障碍物高度语义"）；
> 取代关系见 **§37.7**。**§33 / §34 的 `gravity:=base` 结论不被推翻**——那测的是"帧不竖直 + 车体水平"，
> 与本节"车体与坡面平行 + 帧竖直"是两种不同的病（§37.7#2）。

### 37.1 设置（scratch world / scratch 脚本，官方 world 与 map 资产一个字节没动）

**问题**（用户的假设检验，不是确认）：若机器人**车体与均匀坡面平行**，`base_link` 的 z 轴垂直于坡面
⇒ `base_link` 里的固定高度带量的是**垂直于坡面的净空**（地形相对，可能是对的）；而 `odom_level` 的带
量的是**相对出生点地面的海拔**（坡上应错 ≈ x·sin15°）。本轮用真实 Gazebo + 真实雷达 + 真实
`linefit` / `p2l` 节点量它，**结果与早期"base_link 在坡上错"的说法相反**（§37.6 / §37.7）。

| 项 | 值 |
|---|---|
| scratch world | `.tmp_g37/worlds/`（`mkworld.py` 生成）：`slope15.world`、`flat.world`、`slope15_approach.world` |
| 坡板 | 6.0 × 4.0 × 0.08 m **静态** box，绕 y **pitch −15°**；**顶面过世界原点** ⇒ 坡面 `z = tan15°·x`、法向 `n = (−sin15°, 0, cos15°)`；摩擦 μ=1.0；顶面覆盖沿坡 s ∈ [−1, 5] m |
| 机器人 | `robot:=robot11`，**scratch xacro**：`tools/scripts/regress/robot11_make_sim_xacro.py --wheel-mu 0.8 --out .tmp_g37/robot11_mu08.xacro`。与官方 xacro 的 `diff` **只有** μ（0.02→0.8）与随之而变的注释；几何/质量/碰撞逐条相同（生成器自证 AABB 相等） |
| 为什么改 μ | 官方 μ=0.02 ⇒ 静摩擦角 atan(0.02)=**1.1°** ≪ 15°（§22.7 已登记"坡上停住会滑"）。**实测**：第一次冒烟（μ=0.02）8 s 内沿坡下滑 **5.9 cm** 且继续滑 ⇒ 无法做"静止测量" |
| 出生位姿 | `base_link` 在 **(−0.0265287, 0, 0.0989979)**、rpy = **(0, −15°, 0)**；到坡面的**垂直距离实测 0.102491 m**（= 机器人最低点 = 轮底） |
| 帧 | `odom_level` **≡ Gazebo 世界**（z 朝上、原点在坡面上 s=0 的点）；`base_link` 来自 `/odom_ground_truth` 的**真值** TF；`base_link→livox_frame` = URDF 静态值 (0.000562, 0.130916, 0.157028, rpy 0)。**不走 LIO**（诚实清单 #1） |
| 感知 | **真** `linefit` 节点（`segmentation_sim_robot11.yaml` + `traversability_criteria` + `self_mask_robot11` + `near_ground_robot11` + `gravity_aligned_frame=base_link`）+ **真** `pointcloud_to_laserscan`（`target_frame=base_link`、min/max = −1.0/1.0）。运行期回读：`sensor_height=0.2595`、`gravity_aligned_frame=base_link`、`max_dist_to_line=0.05`、`target_frame=base_link`、`min/max_height=−1.0/1.0` |
| 箱子 | 0.30 × 0.30 × 0.50 m **静态**、**随坡倾斜**（pitch −15°）、中心 = 坡面点 + 0.25·n；横向 **y=+1.2 m**（`x=0` 时不能放在本体里；**y 不影响任何 z**）；沿坡 **s=0/1/2/4 m**，逐个 spawn → 等 5 s → 抓 3 s 快照 → delete → 下一个 |
| 真值 | 箱底**垂直**净空 = 0、箱顶 = **0.5000 m**（面法向，与 x 无关）；箱顶中心 `z_ol = s·sin15° + 0.5·cos15°` |
| 落定判据 | 8 s 窗 `/odom_ground_truth`：**3 个坡跑位置跨度 1.1e-15…1.25e-15 m、姿态跨度 0.000000°**；平地 8.7e-16 / 2e-6°；approach 3.2e-14 / 2e-6° ⇒ **无滑动**；箱顶观测 d 最大 0.491–0.500（构造 0.500）⇒ **箱面贴坡、不悬空/不穿插** |
| 隔离 | 私有 `HOME` / `ROS_LOG_DIR` / `ROS_DOMAIN_ID`（tag 哈希）、先探空再占 `GAZEBO_MASTER_URI`、`unset DISPLAY`、`setsid`、`gui:=False`、**一栈一跑**、收尾只用 `tools/scripts/lib/scoped_cleanup.sh`（**无全机 pkill**）；每跑硬超时 700 s |
| 数据 | 7 跑：坡 3（`g37_slope_run1..3`）+ 平地 2（`g37_flat_run2/3`）+ approach 2（`g37_approach_run1/2`）。原始 `.npz`/脚本在 `.tmp_g37/`（gitignored） |
| 复现 | `python3 .tmp_g37/mkworld.py` → `bash .tmp_g37/run_g37.sh <tag> <slope / flat / approach>` → `python3 .tmp_g37/g37_analyze.py .tmp_g37/<tag> <kind>` |

**三臂**（同一朵雷达云、同一套真值 TF）：

* **A 竖直固定带**：把云搬到 `odom_level`，按 **z** 过滤（= 相对出生点地面的**海拔**；与 `obstacle_cloud_layer` /
  `stvl_layer` / 局部 2D scan 带同口径）。
* **B 车体固定带**：把云搬到 `base_link`，按 **z** 过滤（= 相对车体平面的**垂直距离**）。
* **C 地面分割**：`ground:=linefit` 的 `/segmentation/obstacle`（+ 用 `/segmentation/ground` 拟合局部平面）。

> 带的口径：A 与 B 用**同一组数值**（这样比较的是"帧"而不是"数值"）：主带 `[−0.05, 2.0]`（§26 的局部
> 2D scan 带），另扫 `[−0.02,2.0]`（局部 cloud 层）/ `[0.0,2.0]`（STVL）/ `[0.1,1.0]`（用户示例）/
> `[0.3,1.0]` / `[−1.0,1.0]`（p2l 实际）。**B 的"净空"= z_bl + h_body**（h_body = 车体到脚下支撑面的
> 垂直距离，实测 0.102490/0.102491 m；B 的同一数值带比 A 整体偏 10.25 cm，这正是"体坐标系"的代价，
> 表格里已经在净空口径下按 +h_body 折算）。

### 37.2 逐 x：三种方法的"报告高度"与误差（坡上，车体与坡面平行）

#### 37.2.1 解析值（方法语义；3 跑逐位相同）

| x (m) | 真值：垂直净空 底/顶 (cm) | A：`z_ol` 底范围…顶范围 (cm) | A：净空 底/顶 (cm) | **A：误差 底/顶 (cm)** | B：`z_bl` 底/顶 (cm) | B：净空 底/顶 (cm) | **B：误差 底/顶 (cm)** |
|---|---|---|---|---|---|---|---|
| 0 | 0 / 50.0 | [−3.9, +3.9] … [44.4, 52.2] | 0.0 / 48.3 | **0.0 / −1.7** | −10.25 / +39.75 | 0.0 / 50.0 | **0.0 / 0.0** |
| 1 | 0 / 50.0 | [22.0, 29.8] … [70.3, 78.1] | 25.9 / 74.2 | **+25.9 / +24.2** | −10.25 / +39.75 | 0.0 / 50.0 | **0.0 / 0.0** |
| 2 | 0 / 50.0 | [47.9, 55.6] … [96.2, 103.9] | 51.8 / 100.1 | **+51.8 / +50.1** | −10.25 / +39.75 | 0.0 / 50.0 | **0.0 / 0.0** |
| 4 | 0 / 50.0 | [99.6, 107.4] … [147.9, 155.7] | 103.5 / 151.8 | **+103.5 / +101.8** | −10.25 / +39.75 | 0.0 / 50.0 | **0.0 / 0.0** |

读法：

1. **A 的误差就是海拔差**：顶面中心 `z_ol = x·sin15° + 0.5·cos15°` ⇒ 误差 `= x·sin15° − 0.5·(1−cos15°)`
   = **x·sin15° − 1.70 cm**；底面的误差就是 `x·sin15°`（x=4 时 **103.5 cm**）。
   箱顶本身在 `odom_level` 里还带 **±3.9 cm 的倾角展开**（30 cm 宽的顶面在 15° 下跨 `0.30·sin15°`）——
   同一块顶面在竖直带里不是一个 z。
2. **B 的 z 范围与 x 无关**：−10.25…+39.75 cm（**恒定**），+h_body 后正好是 0…50.0 cm。
   ⇒ **车体与坡面平行时，`base_link` 的 z 就是垂直净空 + 一个常量偏移**——与早期说法相反。
3. **误差对 x 的拟合**（顶面中心，解析）：A **slope = +0.2588 = sin15°**、intercept = −0.0170 m（3 跑逐位同）；
   B **slope = −0.0000、intercept = +0.0000**。**观测值**（可见顶面中位，带采样偏差）：
   A slope = **+0.2492 / +0.2539 / +0.2503**（3 跑），B = **−0.0000**（3 跑）。

#### 37.2.2 实测云（3 跑；`z_ol`/`z_bl` 是观测箱点的范围，顶面误差是"可见顶面"的中位）

| x | 观测箱点 / 顶面点（r1） | 观测 `z_ol` 范围 (cm) | 观测 `z_bl` 范围 (cm) | A 观测顶误差 (cm, 3 跑) | B 观测顶误差 (cm, 3 跑) |
|---|---|---|---|---|---|
| 0 | 847 / 54 | −1.7 … +51.9 | −8.2 … +39.7 | −3.3 / −3.7 / −2.9 | −1.2 / −1.6 / −0.9 |
| 1 | 508 / 17 | +24.3 … +76.8 | −8.1 … +39.7 | +22.4 / +20.8 / +19.7 | −0.9 / −1.4 / −1.1 |
| 2 | 198 / 4 | +50.0 … +100.8 | −8.0 … +39.6 | +45.8 / +45.2 / +45.8 | −0.4 / −1.1 / −0.4 |
| 4 | 49 / 4 | +102.3 … +148.8 | −7.5 … +39.6 | +96.8 / +97.8 / +96.6 | −1.2 / −0.8 / −1.4 |

**为什么观测顶误差比解析值系统性低 1–5 cm**：可见顶面随距离缩到**近坡侧**（s=4 时只剩 4 个点、
`d>0.48`），中位 z 因此偏低；**解析值才是"带在这一点的读数"**，观测值只作独立核对
（B 的观测误差 −0.4…−1.6 cm 也是同一采样偏差，不是方法误差——B 的解析误差是 0）。

#### 37.2.3 C 臂（真 linefit）：坡面被整片判成地面，箱点留在 obstacle 里

| x | 箱点在 `/segmentation/obstacle`（3 跑） | 观测垂直 d 范围 (cm) | 拟合地面法向误差 (deg) | 拟合残差 p50 (mm) | **箱位置处地面高度误差 dz (mm)** |
|---|---|---|---|---|---|
| 0 | 797–803 / 842–877 | +5.0…+5.6 … +49.7…+49.9 | 0.057–0.087 | 0.3–0.4 | **+0.9…+1.2** |
| 1 | 473–477 / 507–519 | +5.0…+5.3 … +49.9…+50.0 | 0.072–0.087 | 0.3–0.4 | **+1.3…+1.7** |
| 2 | 183–187 / 198 | +5.1…+5.8 … +49.9 | 0.061–0.072 | 0.3–0.4 | **+1.3…+1.6** |
| 4 | 46–47 / 48–51 | +6.1…+6.5 … +49.1…+50.0 | 0.102–0.197 | 1.0–2.1 | **+2.3…+4.5** |

读法：linefit 把 15° 坡面**整片判成地面**（`/segmentation/ground` 的点在坡面坐标里 d ≤ 0.02 m（p99），最大 0.045 m），
不在 obstacle 里；箱子的**顶面**保留（d→0.500）、**箱底 5 cm 被 `near_ground`（0.05）削掉**
（obstacle 的最低 d = 5.0–6.5 cm）⇒ 这是 C 臂**唯一**的系统偏差：**贴着地面的 5 cm 障碍会被吃掉**。
地面平面拟合（箱周围 1.5 m 的 ground 点）**法向误差 ≤0.2°、箱位置处高度误差 ≤4.5 mm**。

**C 臂的"报告净空"就是观测 d 本身，误差 = d − 真值**：顶面 **−0.9…0.0 cm**（d 最大 0.491–0.500，
真值 0.500；减去地面拟合误差后 ≤4.5 mm）；底面 **+5.0…+6.5 cm**（d 最小 0.050–0.065，真值 0，
差的这 5 cm 全是 `near_ground: 0.05` 把贴地一层判成地面）。**地面本身没有误差**（坡面 0 格、法向
≤0.2°）——C 的误差全部集中在"贴地 5 cm"这一条。

#### 37.2.4 机器人自己的轮子（sanity：正确的方法里轮底净空应为 0）

四个轮底在 `base_link` 里是**已知点** (±0.182135, ±0.182135, **−0.102499**)（轴距=轮距=0.36427 m、
轮 mesh 最低点 z=−0.102499；`sentry_robot_robot11_sim.xacro` 的"几何速查"）。把它们按本轮的
`odom_level←base_link` 真值变换：

| 方法 | 四个轮底的报告值 | 净空（+h_body） | 误差 |
|---|---|---|---|
| A `odom_level`（竖直） | `z_ol` = **+4.713 / −4.715 cm**（前/后轮 = ±0.182135·sin15°） | −4.72 / +4.71 cm | **±4.7 cm** |
| B `base_link` | `z_bl` = **−10.250 cm**（四个全同） | **0.000 cm**（四个全同） | **0.0 cm** |
| C linefit | 轮底就在它拟合的地面线上（d=0）；地面拟合误差 ≤4.5 mm | 0 | ≈0 |

**读法**：这正是 A 的"海拔"语义在最该为零的地方露馅——**同一辆车的前后轮在竖直带里差 9.4 cm**
（因为车体是斜的），而 B 的四个轮底**逐位相同**。也解释了为什么 A 的带在坡上必然"越远越偏"：
它的零点是**一个点**（出生点地面），而 B 的零点是**一整片与车体平行的面**。
（云端看不到自己轮底：轮底在地面上、又在自击近场里，所以这组数是**解析**的，不是雷达观测。）

### 37.3 平地对照（x=0，无坡）：三臂一致

| 量 | A（`odom_level` 竖直） | B（`base_link`） | C（linefit） |
|---|---|---|---|
| 箱顶净空 | **50.0 cm**（解析） / 48.4–49.0（观测中位） | **50.0 cm**（解析） / 48.4–49.0（观测中位） | 观测 d 最大 49.9；地面 dz **+1.0/+1.5 mm** |
| 箱底净空 | **0.0** | **0.0** | 观测最小 d 5.0–5.1 cm（near_ground 削掉） |
| 误差 | **0.0 / 0.0** | **0.0 / 0.0** | 法向误差 0.076/0.110° |
| 地面自身在带内（`[−0.05,2.0]`） | **3845–3873 格（100%）** | **0 格** | **0 格** |
| 地面自身在带内（`[0.1,1.0]`） | 0 格 | 0 格 | 0 格 |

**平地上 A 与 B 在"箱子高度"上完全一致**（都 0 / 50.0）——这正是对照要证明的：坡上的 A/B 差异
**只能来自帧的倾角关系**，不是算法或噪声。但**地面本身**在两种帧里就已经不同：A 的地面在 z=0
（下界 −0.05 时在带内），B 的地面在 z=−10.25 cm（下界 −0.05 时在带外）。**所以"带"不是去地面的
手段；去地面的是 linefit**（C 列的地面格数在坡上/平地上都是 0）。

### 37.4 第二个对照："从平地出发、坡在前方"（车体**水平**）—— B 退化成海拔

实际工况大多是"平地出发，遇到坡"（用户原话）。把机器人放在 **x=−1.0 m 的平地**（pitch=0）、
坡从 x=0 起坡，箱在 s=1/2/4：

| x | 真值 顶净空 | A 误差（顶） | **B 误差（顶）** | C：地面法向误差 / dz@箱 |
|---|---|---|---|---|
| 1 | 50.0 cm | **+24.2 cm** | **+24.2 cm** | 0.05–0.17° / **−0.3…−1.5 mm** |
| 2 | 50.0 cm | **+50.1 cm** | **+50.1 cm** | 0.07–0.09° / **+1.6…+1.8 mm** |
| 4 | 50.0 cm | **+101.8 cm** | **+101.8 cm** | 0.11–0.20° / **+2.8…+5.9 mm** |

* **A 与 B 逐位相同**（拟合斜率两者都 +0.2588）：车体水平时，`base_link` 的 z 与 `odom_level` 的 z
  只差一个常量 (h_body)，**B 的地形相对性完全消失**。
* 根因一句话：**`base_link` 的 z 量的是"相对车体平面"，不是"相对脚下地面"**；只有当车体与所在
  地形平行时两者才重合（37.2），否则它就跟海拔法一样错。
* C 仍然对（地面法向 0.05–0.20°、dz ≤6 mm）：linefit 在传感器系里对**倾斜地面**逐扇区拟合直线，
  不要求车体与坡面平行。

### 37.5 坡面自标记：**竖直固定带会把坡面自己变成障碍**（2D 后果）

**方法**：用**无箱子**的落定快照，把云按带过滤后**竖直投影**（丢掉 z，就是 2D 代价图的投影方式），
在 **5×5 m、0.02 m** 的局部代价图窗口里数**格数**（排除 r<0.35 m 的机器人自身）。分母 = 窗内
**全部**可见坡面/地面的格数。3 跑（坡）/2 跑（平地/approach）的**范围**：

| 带（同一数值，分别在 A / B 帧里用） | A `odom_level` 竖直：格数（占可见） | B `base_link`：格数（占可见） | C linefit：格数 |
|---|---|---|---|
| **坡上**：全点（分母） | 3087–3127 | 3108–3154 | 0 |
| `[−0.05, +2.0]`（§26 局部 2D scan 带） | **2110–2161（68–70%）** | **0** | **0** |
| `[−0.02, +2.0]`（局部 cloud 层带） | 1906–1977（62–64%） | 0 | 0 |
| `[0.0, +2.0]`（STVL 3D 带） | 1773–1842（57–60%） | 0 | 0 |
| `[0.1, +1.0]`（用户示例） | 1190–1208（39%） | 0 | 0 |
| `[0.3, +1.0]`（抬高下界） | 408–433（13%） | 0 | 0 |
| `[−1.0, +1.0]`（p2l 实际带） | 3087–3127（100%） | **3108–3154（100%）** | 0 |
| **approach**（车体水平、坡在前）：`[−0.05,+2.0]` | **4469–4544（100%）** | **928–935（21%）** | 0 |
| approach：`[−0.02,+2.0]` | 4469–4544（100%） | 801–813（18%） | 0 |
| approach：`[0.1,+1.0]` | 1061–1068（24%） | 414–421（9%） | 0 |
| **平地**：`[−0.05,+2.0]` | 3845–3873（100%） | **0** | 0 |
| 平地：`[−1.0,+1.0]` | 3845–3873（100%） | 3845–3873（100%） | 0 |
| 平地：`[0.1,+1.0]` | 0 | 0 | 0 |

**坡上 A 的带把坡面标到哪**（5×5 m 窗内，`A 覆盖沿坡 s`）：`[−0.05,2.0]` ⇒ **s = −0.19…2.58 m**
（窗内**全部**上坡面；窗外的坡面只要 z<2.0 m，即 s<7.7 m，同样在带内）；`[0.1,1.0]` ⇒ s = 0.39…2.58 m
（原理上到 s=3.86 m）；`[0.3,1.0]` ⇒ s = 1.16…2.58 m。3 跑同（±0.003 m）。

**读法（这是本轮第二个必须报的量）**

1. **竖直带不是"去地面"，而是"按海拔选地面"**：15° 坡上坡面 z = s·sin15° 随 s 单调升 ⇒ 任何固定
   下界都只能切掉"离机器人足够近/远"的一段，**不可能**整片去掉；下界抬到 0.3 m 时仍有 13% 的坡面
   在带内，同时**真障碍的下半部分也开始被丢**（B 帧的箱点带内比例在 `[0.3,1.0]` 里只剩 20–24%）。
2. **B 的 0 格有一个必须写清的前提**：只有当带的下界 **> −h_body**（即 > −10.25 cm）时才是 0；
   **p2l 实际带 `[−1,1]` 的 3108–3154 格就是反例**——它的下界在地面以下，坡面照样全标。所以现网
   能干净，靠的是 **linefit 先把地面删掉**（C 列恒 0），不是靠 p2l 的数值带。
3. **approach 的 B 列 18–21% 是"B 也不地形相对"的直接证据**：车体水平、坡在前方 ⇒ 坡面在
   `base_link` 里也是斜的（z 从 −10.25 cm 升到 +），固定带同样切出一条坡面带。
4. **平地上 A/B 的差异只是常量偏移**（A 的地面 z=0，B 的 −10.25 cm）：下界 −0.05 时 A 100%、B 0%；
   下界 −1.0 时两者都 100%。**这说明"带"的好坏在平地上看不出来**——必须上坡才暴露。

**假阴性（同一带把箱子丢掉）**：p2l 实际带 `[−1,1]` 在 A 帧里对 x=0/1/2/4 的箱点保留
**100% / 100% / 99% / 0%**（x=4 的箱顶 z_ol = 1.02–1.49 m > 1.0 ⇒ **整箱掉出带**），B 帧 100%；
`[0.1,1.0]` 在 A 帧 84/100/99/0%，B 帧 62/62/63/78%（B 的数值带要先减 h_body 才是同一净空带）。
⇒ **竖直固定带在坡上同时制造假阳性（坡面）和假阴性（高处真障碍）**。

### 37.6 判定与含义

**判决（用数字）**：

1. **"垂直净空"这件事上，`base_link` 是对的（车体与坡面平行时）：误差 0.0 cm × 4 个 x，斜率 −0.0000。**
   `odom_level` 的固定带是错的：误差 = x·sin15° − 1.7 cm（顶）/ x·sin15°（底），斜率 **+0.2588 = sin15°**。
   **这推翻了"base_link 在坡上必然错"的早期说法**（§37.7）。
2. **但 `base_link` 的前提是"车体与所在地形平行"**：approach 对照里它与 `odom_level` **逐位同错**
   （+24.2 / +50.1 / +101.8 cm，斜率都 +0.2588）。上坡入口、坡顶、台阶、坑沿——这些正是"姿态≠地形"
   的地方——`base_link` 和海拔法一样不可用。
3. **两种姿态都对的是地面分割**：`ground:=linefit` 的坡面法向误差 **0.05–0.20°**、箱位置处高度误差
   **−0.3…+5.9 mm**、坡面自身 0 格。代价是**近地 5 cm 被 `near_ground` 判成地面**（箱底观测 d 从 5.0 cm 起）。
4. **帧不换**：`odom_level` 仍是竖直帧（用户的"2D 投影必须竖直"是对的，`gravity:=tf` 保持）。
   本轮**没有**任何证据支持把局部代价图帧换成 `base_link`——它不是重力竖直帧，且只在"车体与地形平行"
   时地形相对。

**落点（对现有开关的含义）**：

| 面 | 结论 |
|---|---|
| `gravity:=tf` / 局部帧 `odom_level` | **保持**。帧竖直是对的；本节没有推翻 §4.11 的帧归属 |
| `ground:=linefit`（默认） | **必须保留**。它是目前唯一在"坡上/坡入口"两种姿态都给出正确地形相对高度的东西（0.05–0.20°、≤6 mm） |
| `near_ground`（0.05） | 保留，但**记住代价**：贴地 5 cm 的障碍会被判成地面（本轮箱底 5.0–6.5 cm 被削）。这是"不误标地面"与"不漏低矮障碍"的既有折衷 |
| **`local_obstacle` 的 3D 高度带**（cloud/stvl，带在 `odom_level`） | **不能当"离地高度"用**：坡上它按海拔切，68–70% 的坡面格在 `[−0.05,2.0]` 内、100% 在 p2l 的 `[−1,1]` 内。它能安全工作**只因为**输入已经是 linefit 去过地面的云；linefit 一旦失效（坡度 > `max_slope` 0.4 ≈ 21.8°、非平面坡、地面被遮挡），这个带会立刻把坡面标成障碍 |
| **`local_obstacle:=scan` 的 2D 带** | 它在链路里其实只是"**传感器高度的常量闸**"（§17 / `tilted_lidar_fidelity` §K：波束在代价图帧里的 z 恒等于传感器原点 z）⇒ 坡上它 **不会**把坡面标成障碍，但也**没有逐点高度**可用；真正的高度语义在 p2l 之前的 linefit + near_ground |
| **`band_guard`** | 它诊断的是"2D 带内波束比例"，而 2D 带的 z 是传感器高度（坡上 ≈ 0.10 + s·sin15，6 m 内仍在带内）⇒ **坡面上它只会报 ~100%，对"带按海拔切地形"这件事完全看不见**。它不是坡度/地形检查器。（本节没跑 nav2，这一条是从 §17 / `tilted_lidar_fidelity` §K 的链路语义**推**的，不是本轮实测。） |
| 如果将来要"不依赖 linefit 的地形相对带" | 唯一正确的做法是 **z − 局部地面**（`dz`），不是固定 z：`rm_ground_traversability` 已经在算 `g(x,y)`（0.20 m 粗格 / 0.05 m 细格最低点 p05）和 `dz = z − g`；`near_ground` 就是 `dz ≤ 0.05`。把"高度带"改成对 `dz` 的带（例如 `[0.05, 1.0]`），才是地形相对、且与帧是否倾斜无关 |

### 37.7 与既有断言的关系（取代 / 不取代）

1. **`docs/frame_contract_v3.md` §4.11.8 的那行**——"车体/传感器系（`base_link`）…**不能**用来做障碍物
   高度语义——`base_link` 在坡/台阶上会跟着车体倾，同一高度的障碍在它里面不是常量"——**部分被本节推翻**：
   * **推翻的部分**：在**均匀坡 + 车体与坡面平行**时，`base_link` 的 z **就是**地形相对净空 + 常量
     （4 个 x 误差 0.0 cm，斜率 0.0000）。"同一高度的障碍在它里面不是常量"这句话在该工况下是**反的**。
   * **保留的部分**：`base_link` 只在"车体与所在地形平行"时成立；approach 对照里它与海拔法逐位同错。
     而且它**不是重力竖直帧**，所以即使 z 语义对，也不能拿它当代价图帧。
   * 建议改法（本轮**不改** §4.11.8 原文，只在这里登记）：把"不能"改成"**只有在车体与所在地形平行时
     才是地形相对；否则与海拔法同错；且它不是竖直帧**"。
2. **§33 / §34 的 `gravity:=base` A/B 不被推翻**：那两节是"`odom` 帧被 30° 倾斜、而**车体水平**"——
   病在**帧不竖直**（0.5 m 高的障碍在斜帧里沿 −y 抹开 `−h/2·sin30° = −0.125 m`，§34.2.3#3）。
   本节是"**帧竖直、车体与坡面平行**"。两者是不同工况、不同机制，结论互补：
   **帧必须竖直（§33/§34 的结论），带必须地形相对（本节的结论）**。
3. **`frame_contract_v3.md` §4.11.7 的"高度带的数值"那段**——"`odom_level` 的原点现在在地面上 ⇒
   高度带第一次有了'离地高度'的物理语义"——**修正**：只在**出生点地面**处成立；坡上 z 是**海拔**，
   离地高度是 `z − 局部地面`。本节量到的 x=4 处 1.02 m 的"离地高度"就是这句话的反例。
4. **§32 / §36 的"fastlio `floor_z` 在 −2…−5 cm 档"**与本轮无关（那是 LIO 的 z 锚点），本节不走 LIO。

### 37.8 诚实清单

1. **帧来自真值，不是 LIO**：`odom_level` 直接取 Gazebo 世界系、`base_link` 取 `/odom_ground_truth`。
   本节测的是**帧几何**（这才是"带放在哪个帧"的问题），**不含** LIO 的 1–2° 残余（§36 实测 fastlio
   的 `odom_level→base_link` roll 0.82–1.80°）。若把 LIO 接进来，A 的误差会叠加那 1–2°（在 4 m 处
   ≈ 7–14 cm），B 的地形相对性也会被姿态误差破坏——**本节的 0.0 cm 是"理想姿态"下的方法上限**。
2. **scratch 模型改了轮地 μ（0.02→0.8）**：不改的话 15° 坡上停不住（实测 8 s 滑 5.9 cm，且继续滑）。
   几何/质量/碰撞与官方 xacro 逐条相同（生成器自证）；μ 只影响"能否静止"，不影响任何高度语义。
3. **箱子横向放在 y=+1.2 m**：s=0 的箱子与本体重叠，必须偏置；**坡面/坡体沿 y 不变 ⇒ 偏置不改变
   任何 z 或净空**（这一点在 `mkworld.py` 的注释里写死）。
4. **观测顶面中位有 1–5 cm 采样偏差**（可见顶面随距离缩到近坡侧，s=4 只剩 4 个顶面点）。表格的
   "方法读数"用**解析值**（由已知几何算出），观测值只作独立核对；两者对 B 的 0 误差结论一致。
5. **重复性**：模拟本身是确定性的（静止车 + 真值 TF），3 跑的**位置/姿态跨度**在 1e-15 m / 1e-6°
   量级；剩下的 0.2–1.0% 差异只来自 Livox 非重复扫描的采样。**没有**把 3 跑当"统计显著性"。
6. **5×5 m 局部代价图窗**：s=4 的箱（沿坡 4 m ⇒ 世界 x≈3.8 m）在窗外 ⇒ 它的"格数"恒 0；箱的
   **高度/净空**测量与窗无关（37.2 仍然有效）。
7. **15° < linefit 的 `max_slope` 0.4（21.8°）**：更陡的坡 linefit 会拒拟合（本仓另有一条"坡度
   判据"兜底到 25°，也超不过 21.8° 的线拟合上限）⇒ **未测**。30° 坡、曲面坡、坡顶/坡底过渡段
   （正是 `base_link` 与海拔法都错的工况）本轮**只测了 approach 一个过渡姿态**，没有测坡顶。
8. **`/scan` 有自击近场点**（0.05–0.11 m，占有限波束的多数）——这是已知的 self-mask/near_ground
   问题，本节只把它记进 C 臂的"箱底被削"里，没有单独修。
9. **approach 的 s=4 顶面点只有 0–9 个**（`d>0.48`），观测中位不可用；该臂只用**解析值**与
   C 臂的地面拟合。
10. **`g37_flat_run1` 作废**：那一跑的出生 z 误用了坡面公式（比平地低 3.5 mm），落定窗里出现
    1.8 cm / 5.6° 的摆动 ⇒ **不作数**；平地对照用修正后的 `g37_flat_run2/3`（位置跨度 8.7e-16 m）。
11. **没有跑 nav2 / MCPI**：本节的"格数"是几何投影的代价图格，不含 raytrace clearing、inflation、
    lethal/inflation 代价分级；也**没有**测"坡面被标成 lethal 后规划器怎么走"。这是下一轮的事。
12. **没有测"带的上界截断"对真实高障碍（>1.0 m）的影响**：本节只用了 0.5 m 的箱子（37.5 的
    假阴性一列已给出 x=4 掉出 `[−1,1]` 的直接证据）。
13. **`p2l` 的实际带 `[−1,1]` 与 `[−0.05,2.0]` 的差别**容易被误读：37.5 的 B 列 3108–3154 格是
    "同一数值带在 base_link 里"的结果；**p2l 实际吃的是 linefit 之后的云**，所以现网并不靠它去地面。
