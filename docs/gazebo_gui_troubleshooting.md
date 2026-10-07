# Gazebo GUI（gzclient）加载不出来：取证、机理与修法

> 现象（用户 2026-10-07）：「gazebo加载不出来吗，rviz倒是挺好的看着」「我感觉电脑资源很丰富阿，
> 不应该加载不出来」；之前还看到过
> `[ERROR] [gzclient-1]: process has died [pid …, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so']`。
>
> 本文回答：**为什么 RViz 一切正常、Gazebo 就是不出来**；**`-9` 到底是谁杀的**；
> 机器 32 GB 内存是不是真的不够；以及**一条命令的无头跑法**。
>
> 结论先行（三句话）：
> 1. **不是内存问题。** 用户机器 `MemTotal = 31.1 GiB`、28 核；内核日志（本次开机 16:35:56 起
>    完整保留）**没有任何 OOM 记录**，用户会话 cgroup 的 `memory.events` 里 `oom_kill = 0`。
> 2. **`gzclient` 真的会"卡住不出来"**：`robot:=robot11` 的 `<visual>` mesh URI 在本地解析不到，
>    gazebo 于是**回落到在线模型库** `http://models.gazebosim.org/` 并**同步阻塞**。
>    用户 19:50 那次日志里实测卡了 **48.03 s**，而且**是在用户按 Ctrl-C 的那一刻才结束的**
>    —— 也就是说再等下去也不会好。同一条链还导致**车在 Gazebo 里没有 mesh**（RViz 看不到这个问题）。
> 3. **`exit code -9` 大概率是本仓自己的 bench/ab 脚本杀的**：那些脚本收尾做的是
>    `pkill -9 -x gzclient`（**全机**范围，不看 `GAZEBO_MASTER_URI`），而用户 16:55 / 16:56
>    那两次被杀时，同机上确实有兄弟任务在跑 bench。

---

## 0. 想直接照做：三行命令

```bash
# ① 治好"Gazebo 卡住不出来 / 车在 Gazebo 里没有"（根因：robot11 的 model:// mesh 本地解析不到）
#    ★ 2026-10-08 起**已经自动化**：不需要再手工前缀 GAZEBO_MODEL_PATH（机制见 §5.1）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11

# ② 干脆不看 Gazebo 窗口（一条命令无头跑；RViz 照旧）
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False

# ③ 再叠一层"不再等在线模型库"的兜底
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False gazebo_offline:=True
```

`gui` / `gazebo_offline` 都是**opt-in 参数，默认值 = 改造前的行为**（`gui=True`、
`gazebo_offline=False`），不动任何节点的集合与时序。实测三种组合都**PASS**（§5.3）。

> ①里的那条命令在 2026-10-08 之前需要写成
> `GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" ros2 launch …`。
> 那个手工写法**仍然有效**（§5.1.4），但已不再是必需的。

---

## 1. 症状分类（先定位你是哪一种）

| 症状 | 典型日志/表现 | 本文对应章节 |
|---|---|---|
| **A. 窗口在，但世界/模型不出来（卡住）** | `[Msg] Waiting for model database update to complete...` 之后长时间无输出 | §3、§4.1 |
| **B. 窗口出来了，场地有、车没了** | `[Err] [Visual.cc:2956] No mesh specified` | §4.2 |
| **C. 进程直接死：`exit code -9`（SIGKILL）** | launch 里**没有** `escalating to 'SIGKILL'` 那几行 | §5 |
| **D. 进程死：`-6` / `-11` / `-2`** | `-6` = abort、`-11` = 段错误（内核日志有 `segfault`）、`-2` = 自己退出 | §5.3 |
| **E. 关窗口关不掉，5 s / 10 s 后被 SIGTERM/SIGKILL** | `failed to terminate '5' seconds after receiving 'SIGINT'` | §5.2（这是**正常**的收尾行为） |
| **F. 黑屏（进程活着、就是不画）** | 见 §7 的像素判据 | §7 |

---

## 2. 用户机器上的取证（只读，含原始行）

取证范围：`~/.ros/log/**`（298 个 run 目录）、`~/.gazebo/`、`journalctl`（含内核日志）。

### 2.1 `~/.ros/log/2026-10-07-19-50-41-310829-weicheng-289578/launch.log`

```
1791373843.7415378 [INFO] [gzclient-1]: process started with pid [289621]
1791373848.7416131 [spawn_entity.py-4] [INFO] [1791373848.741388767] [spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [robot]
1791373896.7838168 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
1791373901.7941954 [ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
1791373901.8401017 [ERROR] [gzclient-1]: process has died [pid 289621, exit code -15, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
```

**注意两个时间戳只差 8 毫秒**：`spawn_entity` 报"成功生成 robot"是 `…848.741`，
下一行 `gzclient` 的日志（见下）是 `…848.749`。

### 2.2 `~/.gazebo/client-11345/default.log`（gzclient 自己写的，时间戳格式 = `(秒 纳秒)`）

```
(1791373848 749596472) [Msg] Waiting for model database update to complete...
(1791373896 784088121) [Wrn] [Event.cc:61] Warning: Deleting a connection right after creation. ...
(1791373897 593144202) [Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://robot11/meshes/decimated/l2.stl]
(1791373897 593158806) [Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://robot11/meshes/decimated/l2.stl]
(1791373897 593170043) [Err] [Visual.cc:2956] No mesh specified
```

读法：

* 这些时间戳的格式是 `(秒 纳秒)`，**不是微秒** —— 自己写解析脚本时别踩
  （本任务踩过：按微秒除 1e6 会得到"启动后 −1791372xxx 秒"这种鬼数字）。
* `848.749 → 896.784` = **48.03 s** 卡在"等在线模型库"。
* `896.784` 和 launch 里 `user interrupted with ctrl-c` 的 `896.7838168` 相差 **0.27 ms**
  ⇒ **这 48.03 s 不是自己走完的，是用户按 Ctrl-C 把它打断的**（gzclient 装了 SIGINT 处理，
  所以它又活了 5 s 才被 launch 升级成 SIGTERM）。**再等下去不会自己好。**
* 卡完之后立刻就是 `model://robot11/meshes/decimated/l2.stl` **解析失败** ⇒ 视觉 mesh 没加载。

### 2.3 三次连续的失败尝试（16:55 / 16:56 / 16:59）

`~/.ros/log/2026-10-07-16-55-26-711094-weicheng-70931/launch.log`：

```
1791363329.1647146 [INFO] [gzclient-1]: process started with pid [70986]
1791363355.8760724 [ERROR] [gzclient-1]: process has died [pid 70986, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
1791363357.7234998 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
```

`~/.ros/log/2026-10-07-16-56-18-753168-weicheng-73460/launch.log`：

```
1791363381.1753340 [INFO] [gzclient-1]: process started with pid [73500]
1791363392.9428306 [ERROR] [gzclient-1]: process has died [pid 73500, exit code -9, cmd 'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'].
1791363394.6979272 [WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
```

| run | gzclient pid | 起 | 死 | 活了 | exit code | 用户按 Ctrl-C |
|---|---|---|---|---|---|---|
| 16:55:26 | 70986 | 16:55:29.16 | 16:55:55.88 | **26.7 s** | **-9** | 死后 **+1.85 s** |
| 16:56:18 | 73500 | 16:56:21.18 | 16:56:32.94 | **11.8 s** | **-9** | 死后 **+1.76 s** |
| 16:59:37 | 77009 | 16:59:39.70 | —（用户 16:59:21 先按了 Ctrl-C） | ~41 s | **-15** | 死前 |

**关键：前两次 `-9` 之前，launch 里没有任何 `failed to terminate … escalating to 'SIGKILL'`**
⇒ 这不是 launch 自己杀的，是**外部 SIGKILL**。第三次是用户先 Ctrl-C、launch 正常 SIGTERM（`-15`）。

### 2.4 `.tmp_robot11/`（同机的兄弟任务 bench）在那个时间窗里确实在跑

```
2026-10-07 16:46:09  .tmp_robot11/p3_run_def.log
2026-10-07 16:47:35  .tmp_robot11/p3_run_def2.log
2026-10-07 16:51:33  .tmp_robot11/p3_nav.log
2026-10-07 16:51:43  .tmp_robot11/p3_nav2.log
2026-10-07 17:02:18  .tmp_robot11/p4/baseline_run.log
```

### 2.5 **OOM 有证据吗？没有**

| 检查 | 结果 |
|---|---|
| `journalctl -k -b`（本次开机 16:35:56 → 现在，1309 行，含完整的 16:55–17:00 窗口） | **0 条** OOM / `Killed process` / `Memory cgroup out of memory` |
| `/sys/fs/cgroup/user.slice/user-1000.slice/user@1000.service/memory.events` | `oom 0` / **`oom_kill 0`** / `oom_group_kill 0` |
| `journalctl -u systemd-oomd`（本次开机） | 只有启动/停止，**没有任何 `Killed … due to memory pressure`** |
| 历史上真出现过的 OOM（**不是**这次） | `10月 06 11:30:36 kernel: systemd-oomd invoked oom-killer: … oom-kill:constraint=CONSTRAINT_NONE,…,global_oom,…,task=async_slam_tool,pid=264550`；`10月 01 21:44:25 systemd-oomd[802]: Killed …/vte-spawn-….scope due to memory pressure for /user.slice/user-1000.slice/user@1000.service being 72.74% > 50.00% for > 20s with reclaim activity` |
| `dmesg` | 无权限（`读取内核缓冲区失败: 不允许的操作`）；但 `journalctl -k` 可读且已覆盖该窗口 |

> **结论：16:55 / 16:56 那两次 `gzclient` 的 `exit code -9`，没有任何 OOM 证据。**
> 反面证据是齐全的（内核日志完整且干净 + cgroup `oom_kill=0` + oomd 无记录）。
>
> 但要诚实说明两点：
> * 机器上**确实存在** OOM 历史（10-06 11:30 一次真正的 global OOM，任务转储里
>   `gzserver` 的 RSS 是 74.7 万页 ≈ **2.85 GiB**，是当时最大的进程之一），
>   而且**交换分区已经 99.97% 用满**（`SwapTotal 4095 MiB / SwapFree 1 MiB`）——
>   这台机器的内存余量没有"32 GB"听起来那么宽裕，值得清一下 swap 占用大户。
> * 没有 `auditd`（`systemctl is-active auditd` = inactive），所以**无法从日志里指认是谁发的
>   SIGKILL**。下面的 §5 给出的是"排除法 + 时间线吻合"的结论，不是抓现行。

---

## 3. 机理（一）：为什么 gzclient 会卡在"等在线模型库"

### 3.1 代码路径（gazebo 11 / gazebo-classic，带出处）

1. `sdformat` 的 URDF→SDF 转换会把 `package://<pkg>/...` **改写成 `model://<pkg>/...`**。
   本机可复现（`gz sdf -p`，见 §6.1）：
   ```
   URDF: filename="package://robot11/meshes/decimated/base_link.stl"
   SDF : <uri>model://robot11/meshes/decimated/base_link.stl</uri>
   ```
2. `gazebo/common/SystemPaths.cc` 的 `SystemPaths::FindFileURI()`：
   ```cpp
   if (prefix == "model")
   {
     for (auto iter = this->modelPaths.begin(); iter != this->modelPaths.end(); ++iter)
     { path = boost::filesystem::path(*iter) / suffix; if (exists(path)) {...break;} }

     // Try to download the model from models.gazebosim.org if it wasn't found.
     if (filename.empty())
       filename = ModelDatabase::Instance()->GetModelPath(_uri, true);   // ← forceDownload = true
   }
   ```
   ⇒ **本地找不到就无条件去网上找**（`_forceDownload=true`）。
3. `gazebo/common/ModelDatabase.cc` 的 `GetModelPath()` → `HasModel()` → `GetModels()`：
   ```cpp
   boost::mutex::scoped_try_lock lock(this->dataPtr->updateMutex);
   if (!lock)
   {
     gzmsg << "Waiting for model database update to complete...\n";
     boost::mutex::scoped_lock lock2(this->dataPtr->updateMutex);   // ← 就在这里同步阻塞
   }
   ```
   而抓取线程干的是 `GetDBConfig()` → `GetManifestImpl()` → **`curl_easy_perform()`，
   全程没有设 `CURLOPT_TIMEOUT` / `CURLOPT_CONNECTTIMEOUT`**
   ⇒ 遇到"代理在但不回包"就是**无上限等待**。
4. `ModelDatabase::GetURI()` 用的是环境变量 `GAZEBO_MODEL_DATABASE_URI`，没设就用编译期默认
   `http://models.gazebosim.org/`。
5. 网络路径最后还是失败 → `SystemPaths::FindFile()` 打印
   `File or path does not exist [""] [model://robot11/...]` → `gazebo/rendering/Visual.cc:2956`
   打印 `No mesh specified` ⇒ **这个 `<visual>` 没有几何，车在 GUI 里就是"没有"**。

### 3.2 为什么"只有 robot11 会中招"

| 槽位 | `<mesh>` 视觉数 | `package://` 引用数 | 会不会走 §3.1 的回落 |
|---|---|---|---|
| `''`（本仓默认 `simulation_waking_robot.xacro`） | **0** | 0 | 不会（全是 box/cylinder） |
| `hzmirm` | **0** | 0 | 不会 |
| `robot11` | **12** | **14** | **会** ← 用户 2026-10-07 换的就是这个 |

⇒ 这解释了用户的对比感受：**RViz 一切正常**（RViz 用 `package://` 直接走 ament 索引，
不经过 gazebo 的 `model://` 回落），**只有 Gazebo 卡/不显示**，
而且**是从换成 `robot:=robot11` 之后才开始**。

### 3.3 为什么"本地解析不到"（真正缺的东西）

`gazebo/common/SystemPaths.cc` 里 `model://` 的解析根是
`$HOME/.gazebo/models` **加上** `GAZEBO_MODEL_PATH` 的每一项。
本项目里 `GAZEBO_MODEL_PATH` 来自各包 `package.xml` 的
`<export><gazebo_ros gazebo_model_path="${prefix}/…"/>`（实现见
`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py`），实测只有一项：

```
GAZEBO_MODEL_PATH = <install>/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes
```

`model://robot11/meshes/decimated/l2.stl` 要解析，需要某个解析根 `R` 满足
`R/robot11/meshes/decimated/l2.stl` 存在。而实际文件在：

```
install/robot11/share/robot11/meshes/decimated/l2.stl      ← 存在（symlink 到 src）
```

⇒ **只差一个解析根 `install/robot11/share`**。而 `install/robot11/share/robot11` 里
**没有 `model.config`**，也没有任何包把 `install/robot11/share` 写进 `GAZEBO_MODEL_PATH`
（本仓没有任何一个 `package.xml` 导出 `gazebo_model_path` 指向它）。

> 顺带：`~/.gazebo/client-11345/default.log` 里那两条
> `[Err] [InsertModelWidget.cc:403] Missing model.config for model "…/meshes/RMUL2024_world"`
> 与 `"…/meshes/obstacles"` 是**同一个根因的另一个表现**：把非模型目录（`meshes/` 下的兄弟
> 目录）放在了 `GAZEBO_MODEL_PATH` 上，Gazebo 会一个个去读 `model.config` 并报错。
> 它**不是**世界加载失败的原因（世界 `model://RMUC2026_world/...` 是能解析的，
> `meshes/RMUC2026_world/model.config` 存在）。

### 3.4 沙箱里的受控复现

见 §6.2 的实测表（`blackhole` 配置 = 用一个"只 accept、永不回包"的本地 TCP 黑洞当代理，
精确复现"代理在但不干活"）。

---

## 4. 机理（二）：`exit code -9` 是谁杀的

### 4.1 先排除"launch 自己杀的"

launch 自己杀会在前面留下证据（三条一组）：

```
[ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'
[INFO]  [gzclient-1]: sending signal 'SIGTERM' to process[gzclient-1]
[ERROR] [gzclient-1]: process[gzclient-1] failed to terminate '10.0' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'
```

用户 298 个 run 里 `exit code -9` 一共 **202 条**，**绝大多数都是这一种**（关窗口时的正常升级）。
**只有 16:55:55（pid 70986）和 16:56:32（pid 73500）这两条 gzclient 的 `-9` 前面没有这组证据**
⇒ 外部 SIGKILL。

### 4.2 最可能的来源：本仓 bench/ab 脚本的全机 `pkill -9`

仓库里有**多处**收尾是这么写的（不看 `GAZEBO_MASTER_URI`、不看是谁起的）：

```
tools/scripts/diag/run_drift_diag.sh:130:          pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/run_mapping_headless.sh:133: pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/mapping_ab_run.sh:110:       pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/nav_map_ab_run.sh:66:        pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/mapping/sltune/run_sltune_ab.sh:104: pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/regress/run_remap_lowterrain.sh:79:  pkill -9 -x gzserver; pkill -9 -x gzclient
tools/scripts/regress/run_nav_clearance_ab.sh:134: pkill -9 -x gzserver
tools/scripts/localization/run_nav_smoke_regress.sh:44,60: pkill -9 -x gzserver
```

而且紧邻的几行还有 **`pkill -9 -f 'ros2 launch'`** —— 这会把**别人**的 `ros2 launch` 一起杀掉。

两个细节值得注意：

* `tools/scripts/regress/run_robot_model_probe.sh` 与 `robot11_gui_bench.sh` 想"只杀自己那个
  master 的"，写的是 `pkill -f "gzclient.*$GZ_PORT"` —— 但 **`gzclient` 的命令行里根本没有端口**
  （`gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so`），所以这个"限定"**永远匹配不到**，
  等于没保护；`gzserver` 那条能匹配（`gzserver <world>` 里也没端口……同样匹配不到）。
* 时间线吻合：16:46–17:02 同机有兄弟任务的 bench（§2.4），两次 `-9` 落在 16:55:55 与 16:56:32。

> **诚实边界**：没有 `auditd`，无法"抓现行"证明是这几行 `pkill` 干的。
> 但可以确定的是：**它不是 OOM，也不可能是资源不足。**

### 4.3 其余退出码

| code | 含义 | 用户日志里的例子 |
|---|---|---|
| `-6` | abort（`SIGABRT`） | `10月 07 16:59:xx` 附近多次；`.tmp_robot11/p4/baseline_run.log` 里也有 `[gzclient-1] … exit code -6` |
| `-11` | 段错误 | `10月 06 20:50:18 kernel: gzclient[3600071]: segfault at 18 ip … in libc.so.6[7b4cc4028000+195000] likely on CPU 24` |
| `-2` | 自己退出（`SIGINT` 默认动作） | 多条历史记录 |
| gzserver `-11` | `10月 07 14:35:59 / 14:39:11 kernel: gzserver[1622157/1626342]: segfault at 158 ip … in libgazebo_ode.so.11.10.2` | 物理引擎侧段错误，与 GUI 无关 |

---

## 5. 修法

> 分级：**【修法】** = 消除根因；**【绕过】** = 不改根因但让你能干活；**【环境】** = 机器/环境层面。

### 5.1 【修法·本项目｜★ 2026-10-08 已自动化】让 `model://robot11/...` 本地可解析

> **现在一条命令就够**（不需要任何手工环境变量）：
> ```bash
> ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 mode:=nav \
>   lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d spin_speed:=0.0 nav_rviz:=True
> ```
> 手工前缀环境变量的老办法**仍然有效**，但只作 fallback（§5.1.4）。

#### 5.1.1 现在是谁在修：两条腿，互为兜底

| # | 机制 | 落点（文件 / 键） | 贡献的解析根 | 生效范围 |
|---|---|---|---|---|
| (a) **启动侧** | `rm_nav_bringup/launch/bringup_sim.launch.py` 与 `hzmi_rm_simulation/launch/rm_simulation.launch.py` 里的 `_gazebo_model_path_setup()`（`OpaqueFunction`，**必须排在 include gzserver/gzclient 之前**），用 `AppendEnvironmentVariable('GAZEBO_MODEL_PATH', …)` | `<install>/robot11/share`（= `os.path.dirname(get_package_share_directory('robot11'))`） | 只有 `robot:=robot11`（真带 `<mesh>` 视觉的槽位）会执行；默认模型 / `hzmirm` 一条都不加 ⇒ 它们的 env / 日志 / 时序**逐字节不变**；追加不覆盖用户原值；用户已手工 export 过同一目录时幂等跳过 |
| (b) **包侧** | `src/rm_simulation/robot11_description/package.xml`：`<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` | `<install>/robot11/share/robot11/..`（同一个目录，字面量带 `..`） | **任何** gazebo 入口都吃得到：本仓 launch、裸 `ros2 launch gazebo_ros gzserver.launch.py` / `gzclient.launch.py`、`gz sim`（实现：`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py` 扫描各包 package.xml 后把 `${prefix}` 换成本包 share，再拼进 `ExecuteProcess` 的 `additional_env`） |

**哪条在实际运行里干活？** 实测（§5.1.3 的 env dump）一次 `robot:=robot11` 的 launch 跑起来后，
gzserver/gzclient 的 env 是：

```
GAZEBO_MODEL_PATH=/…/install/robot11/share/robot11/.. : /…/install/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes : /…/install/robot11/share
                  └────── (b) 包 export 贡献 ──────┘   └──────── 改造前就有（world/obstacle 的 model://）────────┘   └── (a) launch 贡献 ──┘
```

* gazebo 是按 `modelPaths` 顺序找的 ⇒ **先命中的是 (b) 那条**（`boost::filesystem::exists()` 会把字面量里的 `..` 解开，实测可用）；
* 但 **(a) 才是"本仓 launch 一定可用"的保证**：它不依赖那份 package.xml 是否重建过、能不能被
  `catkin_pkg` 解析，只要 `install/robot11/share` 在就一定加进去；把 package.xml 拿开（模拟"没重建过的旧 install"）
  后 (a) 单独实测同样 0 错误、12 个 mesh 全部读进来（§5.1.2 的 P2）；
* (b) 的价值在"**我们 launch 之外的 gazebo 入口**"：裸 gzserver/gzclient、直接开 world 文件、`gz sim`
  都只认 package.xml 那条。不设任何环境变量、只留 (b) 时实测同样 0 错误（§5.1.2 的 P3）。

#### 5.1.2 三组对照实测（2026-10-08，Xvfb + 软件 GL，黑洞代理）

协议（= §7 / §6.3 的同一套，**只有这条协议会让 gzclient 去建模型并解析 mesh**）：
起 gzserver → 等 `Init world` → `spawn_entity.py -file`（`robot11_visual:=decimated`）→ **再起 gzclient**。

| | (a) 启动侧根 | (b) 包 export 根 | 「等在线模型库」 | `No mesh specified` | `SystemPaths.cc:459` | `FuelModelDatabase.cc:313` | gzclient 事件数 | gzclient 读入 `rchar` |
|---|---|---|---|---|---|---|---|---|
| `asis`（改造前） | ✗ | ✗ | **出现，stall 13.84 s**（我把黑洞代理拆掉它才结束，**不设上限**） | **24** | **36** | **36** | **184** | 3.90 MB（只有世界资产，**没读** robot11 的 mesh） |
| **只有 (a)** | ✓ | ✗ | 没有 | **0** | 0 | 0 | **21** | **12.16 MB**（+8.26 MB） |
| **只有 (b)** | ✗ | ✓ | 没有 | **0** | 0 | 0 | **21** | **12.16 MB**（+8.26 MB） |

读法（重要）：

* `No mesh specified = 0` **只有在"确实读了 mesh"的前提下才是证据** —— 加载成功时 gazebo
  **一个日志都不打**，所以必须同时看"客户端到底读进来多少字节"。上表：12.16 − 3.90 =
  **8.26 MB ≈ 12 个 decimated mesh 的 8.20 MB**（`stat` 求和 = 8,200,758 B），且 0.1 s 一次的
  fd 扫描抓到了正在打开的 `…/meshes/decimated/l5.stl` / `l11.stl` ⇒ 这是**正证据**，不是"没报错"。
* `asis` 那行的 **24 / 36 / 36 / 184** 与 §6.3（2026-10-07 量的 24 / 36 / 36 / 183）**逐项相同**
  ⇒ 两组日期的复现口径一致（事件数差 1 是收尾时多一条 `Event.cc` 告警）。
* 两条根**各自单独就够**，所以 §5.1.1 的"哪条在干活"不是"二选一"，而是"先命中 (b)，缺 (b) 时 (a) 顶上"。
* 代价（诚实说明）：两个解析根是**等价目录**，`InsertModelWidget` 扫 `GAZEBO_MODEL_PATH` 时会各扫一遍
  ⇒ 带 GUI 的 robot11 跑，`Missing model.config` 这类**噪音**从 3 行变 6 行（gzclient 事件数
  21 → 27，见 §5.1.3）。它只是"插入模型"面板扫目录时的告警，**不影响加载/渲染**。

#### 5.1.3 用户那条命令（`robot:=robot11`，无任何手工环境变量）实测

| 项 | 实测值 |
|---|---|
| 命令 | `ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 gui:=True gazebo_offline:=False robot:=robot11 mode:=nav lio:=small_point_lio localization:=gicp nav:=mppi planner:=smac2d nav_rviz:=True spin_speed:=0.0` |
| launch 日志里那一行 | `[gzmodel] robot:=robot11 需要 robot11 的本地解析根：GAZEBO_MODEL_PATH 追加 /…/install/robot11/share（追加，不覆盖原值：<空>；…）` |
| gzserver / gzclient 的 `GAZEBO_MODEL_PATH` | 三条（见 §5.1.1 的展开），**两条 robot11 根都在** |
| launch → spawn | **8.06 s / 8.81 s / 9.08 s**（三次；launch 自己的 `spawn_entity.py` 成功） |
| `Waiting for model database update to complete...` | **没有出现**（黑洞代理仍然挂着 ⇒ 与网络无关） |
| `No mesh specified` / `SystemPaths.cc:459` / `FuelModelDatabase.cc:313` | **0 / 0 / 0** |
| gzclient 事件数 | **27**（含 8 行 `Missing model.config` 噪音；改造前同一条命令是 **275**，且含 stall） |
| `/odom` | 34 条/20 s（1.7 Hz；另一次 72 条/30 s = 2.39 Hz）—— 速率的限制是 **RTF 0.17**（软件 GL + 30000 条 ODE 射线），不是里程计本身 |
| `/segmentation/ground` | 6640 pts/帧（中位；另一次 6648） |
| `/scan` | 1462 波束/帧，其中有限值 510（另一次 504），最远 7.9 m |
| 节点集合 | 完整且与改造前**逐个相同**（`small_point_lio`、`gicp_registration`、`map_server`、`controller_server`、`planner_server`、`bt_navigator`、`rviz`…；只有 rviz 内部 `transform_listener_impl_<随机 hex>` 这种带随机后缀的伪节点名不同） |

**同一条命令的 before/after 对照（改造前那侧 = HEAD 的 launch + 拿掉包 export，其余逐项相同）：**

| | 改造前（`A1b`） | 现在（`F1b`） |
|---|---|---|
| launch → spawn | 8.57 s | 9.08 s |
| 「等在线模型库」 | **出现，stall = 51.64 s**（我把黑洞拆了它才结束） | 没有 |
| `No mesh specified` / `SystemPaths` / `Fuel` | **34 / 56 / 56** | **0 / 0 / 0** |
| gzclient 事件数 | **275** | **27** |

**轻量入口（`rm_simulation.launch.py`，同一条 robot11 路径）也有同一组对照**，
而且这一组抓到了**正面证据**（`gzclient` 真的把 12 个 mesh 读进来了）：

| | 改造前（`L1r`） | 现在（`L4r`） |
|---|---|---|
| 「等在线模型库」 | **出现，stall = 35.46 s** | 没有 |
| `No mesh specified` / `SystemPaths` / `Fuel` | **45 / 78 / 78** | **0 / 0 / 0** |
| gzclient 事件数 | **374** | **28** |
| gzclient `rchar`（全程） | 4.21 MB（只有世界资产） | **12.47 MB**（+8.26 MB ≈ 12 个 mesh 的 8.20 MB） |
| fd 扫描抓到的 `.stl` | 无 | `…/robot11_description/meshes/decimated/l9.stl`（T+4.60 s） |

> ⚠️ **一处必须知道的口径**：`No mesh specified` 的次数**不是一个固定常数**
> （2026-10-07 那次 24、本次 before 侧 34 / 45），它 = "客户端处理了多少轮场景消息 × 12 个 mesh × 2"，
> 随窗口长度/时机变；**判据是"是不是 0"**，不是"等不等于 24"。
> 同理，"修好之后 0 错误"**只有在同时看到 mesh 被读进来时才构成正面证据** ——
> 本沙箱里 gzclient 对"运行期插入的模型"处理**不稳定**（同样配置的几次跑里，有的会去建模型、
> 有的整个窗口都不建；改造前后都有这个现象）⇒ 上表 `L4r` 那一列（12.47 MB + fd 命中）才是
> "解析链真的通了"的证据，`F1b` 那一列只是"**没有**再出现那条回落路径"。
> 这属于 gazebo classic 客户端在本沙箱的行为，与本次修法无关（同一个不确定性在 `asis` 上也存在）。

#### 5.1.4 手工做法（fallback，仍然有效）

**(a) 启动时追加一个 `GAZEBO_MODEL_PATH` 解析根**（不改任何文件）

```bash
GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" \
ros2 launch rm_nav_bringup bringup_sim.launch.py robot:=robot11 world:=RMUC2026 ...
```

（launch 侧的 `_gazebo_model_path_setup` 认得这个目录：已经在 `GAZEBO_MODEL_PATH` 里就**不重复追加**，
日志里会打 `[gzmodel] … 已存在（无需追加）`。）

**(b) 包侧 export**（持久，已经进了仓库）

```xml
<gazebo_ros gazebo_model_path="${prefix}/.."/>
```

`${prefix}` = 该包的 share 目录 = `<install>/robot11/share/robot11`，所以解析根 = `${prefix}/..`
= **`<install>/robot11/share`**（要往上一级，因为 `model://robot11/meshes/…` 要求解析根 R 满足
`R/robot11/meshes/…` 存在）。这条**已实测**（§5.1.2 的 P3：拿掉 (a) 后仅靠它，0 错误 + mesh 全读进来）。

**(c) 一条命令无头跑，完全不碰 GUI** ⇒ 见 §5.3 的 `gui:=False`。

#### 5.1.5 静态验证（不用起 Gazebo，三行）

```bash
# ① 本地解析应当命中（两个拼法都指同一个目录）
ls -l install/robot11/share/robot11/meshes/decimated/l2.stl
ls -l install/robot11/share/robot11/../robot11/meshes/decimated/l2.stl
# ② 确认 sdformat 确实把 package:// 改写成了 model://（应当数出 12 条）
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro robot11_visual:=decimated > /tmp/r11.urdf
gz sdf -p /tmp/r11.urdf | grep -c 'uri>model://robot11'
# ③ 包侧的 export 是否真的进了 install/（重建后才会有）
grep gazebo_model_path install/robot11/share/robot11/package.xml
# ④ 起 GUI 后应当**看不到**这两行
grep -c "No mesh specified\|Waiting for model database" ~/.gazebo/client-*/default.log
```

#### 5.1.6 无头二次独立复核（2026-10-08 接手复核；**不开 Xvfb、不开 GUI**）

上面 §5.1.2/§5.1.3 走的是"Xvfb + 软件 GL + 黑洞代理"那条协议（因为要复现 **gzclient** 的卡顿）。
复核只需要回答"**裸命令能不能解析到 mesh**"与"**哪条机制在干活**"⇒ 无头 `gui:=False` 就够，
而且更干净：没有 gzclient 的事件数/字节数这类不稳定量，`No mesh specified` 直接看 gzserver。

一条命令（新工具，只清**本 master URI** 上的 gazebo 进程）：

```bash
tools/scripts/regress/verify_gzmodel_autopath.sh obj1_plain --settle 50 -- \
    world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
```

| 项 | `robot:=robot11`（裸命令，`GAZEBO_MODEL_PATH` 启动前**未设置**） | 默认模型（`robot` 留空） |
|---|---|---|
| `gzserver` 真身的 `GAZEBO_MODEL_PATH` | `…/install/robot11/share/robot11/..` **:** `…/install/hzmi_rm_simulation/share/hzmi_rm_simulation/meshes` **:** `…/install/robot11/share`（**三项，robot11 根出现 2 次**） | `…/install/robot11/share/robot11/..` **:** `…/install/hzmi_rm_simulation/share/…/meshes`（**两项**） |
| 条目 [0] 的来源 | (b) 包 export（`${prefix}/..`，**排在 modelPaths 最前 ⇒ 先被命中**） | 同左（(b) 对**所有** gazebo 入口都生效） |
| 最后一项的来源 | (a) launch 的 `AppendEnvironmentVariable` | **没有**（(a) 只对 `robot:=robot11` 执行） |
| `[Err] … No mesh specified` | **0**（日志里另一处 "No mesh specified" 是我们自己 `[gzmodel]` 说明文字，见下） | **0** |
| `Waiting for model database update` / `SystemPaths.cc:459` / `FuelModelDatabase` | **0 / 0 / 0** | **0 / 0 / 0** |
| `SpawnEntity: Successfully spawned entity [robot]` | **1** | **1** |
| `ros2 node list` | **33** 个 | **33** 个，与左列**逐个相同**（只差 `transform_listener_impl_<随机 hex>` 这种 ROS 内部伪节点名） |
| `[gzmodel]` 日志行 | **2** 条（解释见下） | **0** 条 |

**关于"日志应当只有一条"**：`robot:=robot11` 时确实会打印 **2** 条 `[gzmodel]`，
但**只有一条真的追加**：
* 第 1 条（`bringup_sim.launch.py`，日志第 4 行）= "**追加** …/install/robot11/share"；
* 第 2 条（日志第 5 行）= 紧跟其后被 include 的 **`hzmi_rm_simulation/rm_simulation.launch.py`**
  自己那份 `_gazebo_model_path_setup()`，它看到的 env **已经**有这条根 ⇒ 打印
  "**已存在（无需追加）**"并不动 env（幂等分支，这正是设计要的行为；
  `bringup_sim.launch.py:2371` 的 `OpaqueFunction` 与 `:2372` 的
  `ld.add_action(start_rm_simulation)` 相邻，而 `start_rm_simulation` 就是那个 include）。
* 证据（可复算）：**单独**跑轻量入口 `ros2 launch hzmi_rm_simulation rm_simulation.launch.py
  robot:=robot11 world:=RMUC2026 gui:=False`（不带 bringup）⇒ `[gzmodel]` **1 条**（"追加"）、
  `Successfully spawned entity` 1、`No mesh specified` **0**、env 与上表**逐项相同**（三项）。
  ⇒ 两次打印是"两个入口各说一句话"，不是"同一个入口跑了两遍"。

**包侧 export 在"真构建"之后确实生效（本次补测，之前一直没做）**：

| 测法 | 结果 |
|---|---|
| `colcon build --symlink-install --packages-select robot11 rm_nav_bringup` 后直接问 gazebo_ros 自己那个函数（`/opt/ros/humble/lib/gazebo_ros/gazebo_ros_paths.py: GazeboRosPaths.get_paths()` —— `gzserver.launch.py`/`gzclient.launch.py` 用的就是它） | 返回 `…/install/robot11/share/robot11/..`（`${prefix}` 已被替换成本包 share） |
| **非 symlink** 的真安装（`colcon build --packages-select robot11 --build-base /tmp/rb --install-base /tmp/ri` ⇒ `share/robot11/package.xml` 是**真实拷贝** 3270 B，不是符号链接） | 同样返回 `…/tmp/ri/robot11/share/robot11/..` ⇒ export 是随 `ament_package()` 装进去的，**不是**"只因为 install/ 里是符号链接" |
| `gz sdf -p` 生成的 SDF | 12 条 `<uri>model://robot11/meshes/decimated/*.stl</uri>`（`<uri>` 总数 25，另外 13 条是 gazebo 材质），`package://` **0** 条 |
| 逐条 `model://` 落到磁盘 | 12/12 命中（`realpath(root + '/robot11/' + rest)` 存在） |
| 生成物与 HEAD 的对照（`livox_mount` 默认档） | 去注释后**逐字节相同**（两份都是 35410 B、`sha256` 相同）；显式 `livox_mount:=plugin` 与不传**同一 sha** |

**⇒ 哪条是"权威"？** 不是二选一，但优先级是明确的：**运行的这条命令里是 (b) 先命中**
（它在 `GAZEBO_MODEL_PATH` 的第 0 项）；**(a) 的价值是"不依赖那份 manifest"**
（没重建过/解析不了也照样加）；**(b) 的价值是"launch 之外的 gazebo 入口"**。

> ⚠️ **一处必须说的偏离**（§5.1.1 与 `robot_models.md` §14.2 里"一个字节都没动"这句只对 (a) 成立）：
> (b) 是 `package.xml` 的 export ⇒ **任何** gazebo 入口（含默认模型、其它 world、裸 gzserver）
> 的 `GAZEBO_MODEL_PATH` 都会多出 `…/install/robot11/share/robot11/..` 这一项 —— 这是
> "包侧兜底"的必然代价，**不是** 0 改动。实测它的副作用**在无头跑里量不出来**
> （默认模型跑：`No mesh specified` 0、`Missing model.config` 0、节点集合与 robot11 跑逐个相同）；
> 有 GUI 时它会让"插入模型"面板多扫一个目录（`Missing model.config` 噪音 3 → 6 行，§5.1.2 尾部）。
> 想彻底去掉这条：删掉 `package.xml` 的那一行并重建 robot11，此时只剩 (a)，
> 代价是裸 `gzserver`/`gz sim` 入口又会回落在线模型库。

### 5.2 【修法·环境】不要再让 gzclient 等在线模型库

**不要**用 `GAZEBO_MODEL_DATABASE_URI=""`（空串）：`ModelDatabase::GetURI()` 里有
`result[result.size()-1]`，对空串 `size()-1` 会下溢成 `SIZE_MAX` ⇒ **越界读（UB）**。
用一个"必然立刻失败"的地址：

```bash
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1/"     # 连接立刻被拒 ⇒ 不等
# 或者干脆在 /etc/hosts 里钉死（需要 root）
#   127.0.0.1 models.gazebosim.org
```

本项目已把它做成开关（默认关 = 行为不变）：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py gazebo_offline:=True ...
```

> 注意：**这条只是"不再等"，不会让车出现**（车不出现是 §5.1 的事）。

### 5.3 【项目新增】`gui:=True|False` —— 一条命令无头跑

`bringup_sim.launch.py` 与 `rm_simulation.launch.py` 都新增了（**纯增量，默认值 = 改造前行为**）：

```bash
# 默认：和以前逐字一样，起 gzclient
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11

# 无头：只起 gzserver，RViz 照旧由 nav_rviz/lio_rviz 控制
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False

# 无头 + 不等在线模型库
ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 gui:=False gazebo_offline:=True

# 只看 Gazebo 起没起来（不含感知/导航/LIO），用 rm_simulation
ros2 launch hzmi_rm_simulation rm_simulation.launch.py world:=RMUC2026 robot:=robot11 gui:=False
```

**「杀掉 GUI、留下 gzserver」的等价手工做法**（老版本/别的入口）：

```bash
pkill -x gzclient                    # 默认 SIGTERM，够用
# ⚠️ 千万不要在别人还在看 GUI 的时候跑 pkill -9 -x gzclient（全机范围）
# ⚠️ 想只动自己那一份：先查 PID，再按 PID kill
pgrep -a -x gzclient                 # 看命令行/父进程确认是自己的
kill "$(pgrep -x gzclient)"          # 只杀这一个
```

RViz 侧不受影响：`nav_rviz:=False` / `lio_rviz:=False` 各自独立。

**端到端验证（本次实测，不是读代码）**：

| 用例 | 判定依据 | 结果 |
|---|---|---|
| `hzmi_rm_simulation rm_simulation.launch.py world:=RMUC2026 robot:=robot11 gui:=False` | gzclient 在 50 s 窗口内出现 **0/50** 秒；gzserver 50/50 | **PASS** ✅ |
| 同上但**不带** `gui`（默认 True） | gzclient 出现 **48/50** 秒；gzserver 50/50 | **PASS** ✅ |
| `rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 robot:=robot11 mode:=mapping lio:=none nav_rviz:=False gui:=False` | gzclient **0/60** 秒；gzserver 58/60；本 run 的 `launch.log` 里只有 `gzserver-4` 的 `process started`，**没有** `gzclient-*` | **PASS** ✅ |

> ⚠️ 判定别用机器级 `pgrep -x gzclient`：第一次测的时候，上一轮的 gzclient 因为
> `pkill -f "gzclient.*$PORT"`（**永远匹配不到**，见 §4.2）没被清掉，把结果污染成假 "up"。
> 现在改成"开跑前按名字清干净 + 以本 run 的 `launch.log` 为准"。

### 5.4 【修法·本仓工具链】别再全机 `pkill -9 -x gzclient`

把 §4.2 那几处改成"只杀自己起的进程组"：

```bash
# 反例（现在）：全机杀，会把别人正在看的 GUI 打成 exit code -9
pkill -9 -x gzserver; pkill -9 -x gzclient

# 正例：脚本自己用 setsid/进程组起 gzserver，收尾只杀自己那个组
setsid gzserver ... & PGID=$!
trap 'kill -9 -- "-$PGID" 2>/dev/null' EXIT
```

（这一项本次**没有改动**那些脚本 —— 见 §9 未做清单。）

### 5.5 【绕过】给 gzclient 加超时前先确认是不是真的卡

```bash
# 现场看 gzclient 卡在哪（等价于本文 §2.2 的取证）
tail -f ~/.gazebo/client-*/default.log
# 有 "Waiting for model database update to complete..." 且长时间不动 ⇒ §5.2
# 有 "No mesh specified" ⇒ §5.1
```

---

## 6. 实测

> 全部在**软件渲染**（Xvfb 私有 display + llvmpipe / Mesa 23.2.1，无 GPU——本沙箱
> `nvidia-smi` 报 "couldn't communicate with the NVIDIA driver"）上量的。
> 隔离：`HOME=<repo>/.tmp_gzgui/<tag>/gzhome`、非默认 `ROS_DOMAIN_ID`、
> 专用 `GAZEBO_MASTER_URI`、专用 X display，退出时清理。
> ⚠️ 本沙箱 `/tmp` 每次 bash 调用都是新 tmpfs ⇒ `HOME` 不能放 `/tmp`。

### 6.1 静态事实

| 项 | 值 |
|---|---|
| `gz sdf -p`（robot11 URDF→SDF） | `package://robot11/meshes/decimated/base_link.stl` → `<uri>model://robot11/meshes/decimated/base_link.stl</uri>` |
| `model://RMUC2026_world/meshes/RMUC2026.stl` | **可**解析（`GAZEBO_MODEL_PATH` 上的 `meshes/RMUC2026_world/` 有 `model.config`） |
| `model://robot11/meshes/decimated/*.stl` | **不可**解析（没有任何 `GAZEBO_MODEL_PATH` 项能满足 `R/robot11/meshes/…`） |
| 沙箱到 `models.gazebosim.org` | `http=200, time_total≈0.93 s, 9598 B`（走代理）⇒ 默认环境**不会**卡 |

### 6.2 网络假说 · 受控实测（**结论与直觉相反，所以更要说清**）

四组配置都**没有** spawn 机器人（探针的 `spawn_entity.py` 参数当时写错了，车没生成）——
这反而成了一个**关键的阴性对照**：

| # | `GAZEBO_MODEL_DATABASE_URI` | 代理 | 有车? | 出现 X 窗口 | 首帧非黑 | 「等在线模型库」 | gzclient 峰值 RSS |
|---|---|---|---|---|---|---|---|
| 1 | 未设 | 可用（`database.config` 回 200，0.93 s） | ✗ | 0.7 s | 3.1 s | **无** | 450 MiB |
| 2 | 未设 | **黑洞**（accept 后永不回包） | ✗ | 0.7 s | 3.1 s | **无** | 444 MiB |
| 3 | `http://127.0.0.1:1/` | 黑洞 | ✗ | 0.7 s | 3.2 s | 无（`Unable to connect … [http://127.0.0.1:1//database.config]`） | 450 MiB |
| 4 | `""`（空串） | 黑洞 | ✗ | 0.8 s | 3.1 s | 无（`Unable to connect … [//database.config]`） | 449 MiB |

**读法**：光有"代理挂了"**并不会**让 GUI 卡住（第 2 行：窗口 0.7 s、首帧 3.1 s、全程没提过模型库）。
⇒ **在线模型库本身不是根因**；它只有在"GUI 线程也需要模型清单"时才会变成根因。
（第 3/4 行说明：把 URI 指到必败地址并不会让启动变慢，也不会崩 —— 可以放心当兜底开关用。）

**三次"真卡住"的观测**（都在**有** `robot:=robot11` 的 mesh 视觉时出现）：

| 场景 | 「等在线模型库」起 → 止 | stall | 结束原因 |
|---|---|---|---|
| 用户 2026-10-07 **19:50**（`~/.gazebo/client-11345/default.log`） | `1791373848.749596` → `1791373896.784088` | **48.03 s** | launch 的 `user interrupted with ctrl-c` 在 `1791373896.7838168`，**与"止"相差 0.27 ms** ⇒ **是 Ctrl-C 打断的，不是自己好的** |
| 同机 **20:13** 另一次 `bringup_sim … robot:=robot11`（**代理可用**，不是本任务的探针） | `1791375197.611883` → `1791375273.952294` | **76.34 s** | 同一 run 的 Ctrl-C 在 `1791375273.9524972`（**相差 0.5 ms**）⇒ 同样是打断的 |
| 本任务黑洞复现（`mesh_asis`） | `1791375142.358570` → `1791375242.031...` | **99.67 s** | 我把黑洞进程拆掉的那一刻（探针窗口 100 s 到头） |

⇒ **stall 不设上限**，48 / 76 / 99.7 s 三个数字都只是"人先受不了"或"我把代理拆了"。
为什么"网络明明通"也要几十秒：`ModelDatabase::UpdateModelCacheImpl()` 拿到 `database.config`
之后，会**对清单里每一个模型再发一次 HTTP 请求**去取它的 `model.config`
（`this->dataPtr->modelCache[fullURI] = ModelDatabase::GetModelName(fullURI);` ⇒
`GetModelConfig()` ⇒ `curl_easy_perform`）。清单里是几百个模型 ⇒ 几百次串行请求。
而这条路径**全程没有超时**，所以"代理通、但慢/不稳"也照样能卡几十秒到几分钟。

### 6.3 mesh 链路 · 根因与修法对照（**有**车）

同一个世界（RMUL2026）、同一个 `robot11` URDF（`robot11_visual:=decimated`）、同一个黑洞代理，
只差一个环境变量：

| 项 | `asis`（默认 `GAZEBO_MODEL_PATH`） | `fixed`（追加 `<install>/robot11/share`） |
|---|---|---|
| `model://robot11/meshes/decimated/l2.stl` 本地可解析 | **no** | **yes** |
| 出现「等在线模型库」 | **是**，stall = **99.67 s**（不设上限） | **没有** |
| `[Err] [Visual.cc:2956] No mesh specified` | **24 次** | **0** |
| `[Wrn] [SystemPaths.cc:459] File or path does not exist` | **36 次** | **0** |
| `[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel` | **36 次** | **0** |
| `[Wrn] [ModelDatabase.cc:340] Getting models from[…]`（重试风暴） | **36 次** | **0** |
| 解析失败的 `model://` URI | **12 个**（`base_link,l2…l12` 全部） | 0 |
| gzclient 自己的日志跨度 | 183 个事件 / 101.2 s | **20 个**事件 / 71.2 s |

`asis` 那 12 个 mesh 的**逐条原始行**（黑洞代理下，节选）：

```
(1791375142 358570479) [Msg] Waiting for model database update to complete...
(1791375242 031      ) [Wrn] [ModelDatabase.cc:212] Unable to connect to model database using [http://models.gazebosim.org//database.config]. Only locally installed models will be available.
(1791375242 31210947 ) [Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://robot11/meshes/decimated/l2.stl]
(1791375242 31227369 ) [Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://robot11/meshes/decimated/l2.stl]
(1791375242 31246268 ) [Err] [Visual.cc:2956] No mesh specified
```

⇒ **`fixed` 一改，两件事同时消失**（不再等模型库 + mesh 全部加载）。
`fixed` 的代价只是 `InsertModelWidget` 多 3 条 `Missing model.config` 的**噪音**
（`ament_index` / `colcon-core` / `robot11` 三个子目录没有 `model.config`），
它们是"插入模型"面板扫目录时的告警，**不影响加载**。

### 6.4 资源画像与 gzserver 内存记账

**一次干净的分阶段记账**（`world=RMUC2026`、`robot=robot11`、`robot11_visual=decimated`、
带 GUI；每 2 s 采一次 `/proc/<pid>/status`，探针开跑前把机器上的 `gzserver`/`gzclient` 按名字清干净）：

| 阶段 | 采样 | 时长 | gzserver RSS 首 → 末（峰值） | 平均 CPU | 斜率 | 相对上一阶段的增量 |
|---|---|---|---|---|---|---|
| **P0** 只加载世界（RMUC2026，含 2.19 MB 的 `RMUC2026.stl`） | 20 | 39 s | **267.8 → 267.8 MiB**（267.8） | 20.9 % | +0.03 MiB/min | — |
| **P1** spawn `robot11`（含 Livox ray 插件） | 20 | 39 s | **3198.0 → 3198.0 MiB**（3198.0） | 37.7 % | +0.04 MiB/min | **+2930.1 MiB (+2.86 GiB)** |
| **P2** 静默（GUI 同时在线） | 165 | **333 s** | **3199.8 → 3199.9 MiB**（3199.9） | 66.9 % | **+0.02 MiB/min** | +1.8 MiB |

其它峰值：

| 进程 | 峰值 RSS | 峰值 CPU（软件 GL，仅参考） |
|---|---|---|
| `gzclient` | **483.7 MiB** | 444–609 %（多线程软件光栅化） |
| `gzserver` | **3199.9 MiB** | 66.9 %（均） |
| `rviz2` | 本次未起（`--gui yes` 只起 Gazebo） | — |

> **结论 1（~3.2 GB 是谁的）**：世界只占 **268 MiB**；**2.86 GiB 全部来自 spawn `robot11` 那一步**，
> 而且是一次性的固定分配（P1 内 39 s 一动不动）。世界 mesh（2.19 MB）**不是**原因，
> 视觉档位也不是（与既有实测"full/decimated 的 gzserver RSS 完全相同"一致）。
>
> **结论 2（有没有泄漏）**：P2 静默 **333 s（>5 min）** 里 gzserver 只涨了 **1.8 MiB**
> ⇒ **+0.02 MiB/min，没有可测的泄漏**。所以"跑久了越来越卡"不是内存泄漏。
>
> **结论 3（OOM 在用户机器上可不可信）**：单看 gzserver 3.2 GiB + gzclient 0.48 GiB + rviz2 0.3 GiB
> + nav2/LIO 十几个节点，整栈峰值大约 6~8 GiB；用户机器 `MemTotal = 31.1 GiB`
> ⇒ **单纯从容量看，OOM 完全不该发生**（这也印证了 §2.5：内核日志里确实没有 OOM）。
> 但要注意 **swap 只剩 1 MiB**（§8 第 4 条）：容量够不代表没有内存压力抖动。

#### 6.4.1 那 2.86 GiB 具体是**哪个东西**（同一世界、同一 URDF，只剥掉一段）

把 robot11 URDF 里 `<gazebo reference="livox_frame"> … </gazebo>` 整段（= `type="ray"` 传感器 +
`libros2_livox.so` 插件）**删掉**，其余（12 个 mesh 视觉、115 个 collision、IMU、mecanum 插件）
逐字节不变，再跑同一套记账：

| 配置 | P0 只有世界 | P1 + robot11 | P1 的增量 |
|---|---|---|---|
| RMUC2026 + robot11（**含** Livox ray 插件） | 267.8 MiB | **3198.0 MiB** | **+2930.1 MiB** |
| RMUC2026 + robot11（**剥掉** Livox ray 插件） | 179.4 MiB | **307.4 MiB** | **+128.0 MiB** |

（两次 P0 不同是因为前一次带 GUI、后一次无头 —— 但**增量是同一次 run 内量的**，可以直接比。）

⇒ **2.74 GiB（2930 − 128）来自那段 Livox ray 插件**；机器人本身（12 个 mesh 视觉 +
115 个 collision + IMU + 底盘控制器）只有 **128 MiB**。

内存**形态**也对得上（`smaps` 排序）：

| | 最大的映射 |
|---|---|
| 含插件 | `rw-p [heap]` **2776.0 MiB** rss（虚拟 2 843 900 MiB） |
| 剥掉插件 | `rw-p [heap]` **46.5 MiB** rss |

⇒ 2.78 GiB 全在 **glibc 堆上**，是**海量小对象**，不是一块大 buffer。
按 `sample=30000` 折算：**≈ 94.8 KiB / 条射线**。成因在插件源码里能对上：
`livox_laser_simulation_RO2/src/livox_ode_multiray_shape.cpp` 的 `AddRay()` **每条射线**都新建
一个 `ODECollision` + 一个 `ODERayShape`（再各自落到 ODE 的 `dRay` geom / space 上），
而 `livox_points_plugin.cpp` 里是 `rayShape->RayShapes().reserve(samplesStep / downSample)` =
**reserve(30000)** 后真的 `AddRay` 30000 次。`<visualize>false</visualize>` 已经关掉了
（所以**不是**射线可视化），代价纯粹是"30000 条 ODE 射线"的结构体开销。

> **给主线的含义**：如果哪天需要把这 3.2 GB 压下来，**唯一有效的旋钮是
> `<plugin><samples>`**（以及 `<downsample>`）—— 把 30000 降到 10000 预计能省 ~1.8 GiB，
> 但会改变点云密度 ⇒ 属于感知契约变更，**必须**另开一个带量化验收的任务，
> 本文不动它（见 §9）。

---

## 7. 怎么复现（不依赖本文的脚本）

```bash
# 0) 一个虚拟屏（本沙箱没有 Xvfb 包，用 apt-get download 解到本仓；有 X 的机器跳过）
#    Xvfb :79 -screen 0 1600x1000x24 -nolisten tcp &
#    export DISPLAY=:79

# 1) 隔离
export HOME=$PWD/.tmp_gzgui/rep/gzhome ROS_LOG_DIR=$PWD/.tmp_gzgui/rep/roslog
export ROS_DOMAIN_ID=177 GAZEBO_MASTER_URI=http://127.0.0.1:12500
mkdir -p "$HOME" "$ROS_LOG_DIR"

# 2) 精确复现"代理在但不干活"
python3 - <<'PY' &
import socket; s=socket.socket(); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(('127.0.0.1',13500)); s.listen(64)
while True: s.accept()      # accept 但永不回包
PY
export http_proxy=http://127.0.0.1:13500 https_proxy=$http_proxy
export no_proxy=localhost,127.0.0.1

# 3) 起世界 + spawn robot11
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch gazebo_ros gzserver.launch.py world:=$PWD/install/hzmi_rm_simulation/share/hzmi_rm_simulation/world/RMUC2026_world/RMUC2026_world.world &
sleep 10
xacro src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro robot11_visual:=decimated > .tmp_gzgui/rep/robot.urdf
ros2 run gazebo_ros spawn_entity.py -entity robot -file .tmp_gzgui/rep/robot.urdf -x 0 -y 0 -z 0.25 -timeout 90

# 4) 起 GUI，看它自己的日志
ros2 launch gazebo_ros gzclient.launch.py &
sleep 60; tail -5 ~/.gazebo/client-*/default.log
# 预期：Waiting for model database update to complete... 然后长时间不动
#       最后 No mesh specified

# 5) 加解析根再试一次 ⇒ 预期不再出现上面两行
kill %3 %2 %1 2>/dev/null
GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$PWD/install/robot11/share" \
  ros2 launch gazebo_ros gzclient.launch.py &
sleep 30; grep -c "No mesh specified\|Waiting for model database" ~/.gazebo/client-*/default.log   # 期望 0
```

---

## 8. 用户侧还需要自己在真机（真 GPU / 真显示器）上确认的

本沙箱**没有 GPU、没有真显示器**，以下都**没有实测**：

1. **真 GPU 路径**：所有渲染数字都来自 llvmpipe 软件渲染。真机上 OGRE 走 NVIDIA GL，
   首帧时间、RSS、CPU 都会不一样（软件渲染下 gzclient 峰值 CPU 到过 581%）。
2. **窗口管理器/合成器**：本沙箱无 WM（截图里 `gzclient` 的窗口是 `+0+0`）。
   真机上如果 Gazebo 窗口被摆到屏幕外、或被合成器黑屏，需要自己确认。
3. **`/etc/hosts` 或代理对 `models.gazebosim.org` 的实际行为**：请实测
   ```bash
   time curl -sS -o /dev/null -w '%{http_code} %{time_total}\n' http://models.gazebosim.org//database.config
   ```
   * 秒回 ⇒ 不是本文 §3 的路径；
   * 卡住/超时 ⇒ 就是它，按 §5.2 处理。
4. **swap 占用**：`SwapFree` 只剩 1 MiB。请自己看 `free -m` 与
   `systemd-cgtop`/`ps` 找占用大户（本文没有权限去动它）。
5. **`exit code -9` 的发送者**：需要 `auditd` 或 `bpftrace` 才能抓现行。建议装/开
   `auditd` 后复现一次：
   ```bash
   sudo auditctl -a always,exit -F arch=b64 -S kill -F a1=9 -k whokills
   sudo ausearch -k whokills -i | tail -20
   ```

---

## 9. 未验证 / 未做清单（诚实边界）

* **没有** 在真 GPU / 真显示器上跑过（本沙箱无 GPU）。
* **没有** 指认 16:55:55 / 16:56:32 两次 SIGKILL 的发送者（无 `auditd`）——
  §4.2 是排除法结论。
* **没有** 修改 §4.2 列出的那些 bench 脚本（`tools/scripts/mapping/**`、
  `tools/scripts/diag/**` 等属于别的任务范围）；只给出改法。
* **没有** 验证 (b) 方案（`package.xml` 里加 `gazebo_model_path="${prefix}/.."`）在
  `colcon build` 后的实际效果 —— 沙箱里验证的是 (a) 的等价做法（直接给 `GAZEBO_MODEL_PATH`）。
  §6.3 的 `fixed` 配置走的是 (a)。
* **没有** 处理"加了解析根之后 `InsertModelWidget` 多 3 条 `Missing model.config` 噪音"：
  修法是给 `install/robot11/share/robot11/` 补一个 `model.config`，但 `ament_index` /
  `colcon-core` 两个子目录的告警来自 ament 的 share 布局本身，**消不掉**（只是噪音，不影响加载）。
* **没有** 动 `<samples>30000</samples>`（§6.4.1 指出它是 3.2 GB 的唯一旋钮）：
  改它会改变点云密度 ⇒ 属感知契约变更，需要单独的量化验收任务（本仓 RULES 也禁止在本任务里改）。
* **没有** 在真机上验证 `gazebo_offline:=True`（只在沙箱的黑洞代理 + `baduri` 配置下验证）。
* 交换分区已满（`SwapFree` 只剩 1 MiB）**没有**做任何清理动作（用户数据只读）。
* ⚠️ **数据完整性披露**：`~/.gazebo/client-11345/default.log` 是**默认端口 + 默认 HOME**
  才会写的文件；本文 §2.2 引用的那 6 行是在本任务**开始时**（19:5x）读到的原文。
  之后 **20:13** 有一次同机并发的 `bringup_sim … robot:=robot11` 运行（**不是本任务的探针** ——
  本任务所有探针都用自己的 `HOME` 与专用端口）把它覆盖了。若要复核 §2.2 的原文，
  请用 20:13 那次同口径的日志（§6.2 第 2 行，stall 76.34 s），或在用户真机上重跑一次复现。
* ⚠️ **同机并发披露**：本任务运行期间（20:06 / 20:13 / 20:17…）机器上还有别的任务在跑
  `bringup_sim.launch.py`。为了拿到干净的内存数字，本任务做过按名字的
  `pkill -9 -x gzserver/gzclient`，**有可能**误伤并发运行（这正是 §4.2 描述的同一个坑）。
  另外 §6.4 的记账第一次也因为并发/残留进程拿到过 123 MiB 的**假**数字，
  已在干净隔离下重测（表里是重测值）。

---

## 10. 相关文档

* `docs/robot_models.md`（robot11 槽位、`robot11_visual` 档位）
* `docs/worlds.md`（RMUC2026 世界资产）
* `docs/issues_and_findings.md`
