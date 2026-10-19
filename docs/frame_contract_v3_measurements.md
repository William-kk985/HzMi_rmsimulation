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
