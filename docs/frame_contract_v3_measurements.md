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
