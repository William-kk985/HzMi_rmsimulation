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
