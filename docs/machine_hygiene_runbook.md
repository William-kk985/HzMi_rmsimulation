# 机器卫生 runbook：swap / zram / systemd-oomd / 收尾作用域

> 适用机器：`weicheng` 的 32 GB / 28 核工作站（kernel `6.8.0-146-generic`，systemd `249.11-0ubuntu3.23`，
> root 文件系统 `ext4`，`/swapfile` 4 GiB）。
> 本文分两部分：
> **Part 1**（§0–§7）交换区体检 + 安全处置 + **不该做什么**；
> **Part 2**（§8）本仓 bench/ab 脚本的**收尾作用域**改造（只杀自己的进程）与实测证据。
>
> **每一条命令都标了状态**：
> - **【已执行】** —— 本次在沙箱里真跑过，输出见正文；
> - **【未执行·需 root】** —— 会改系统状态且要 root，**没有跑**，命令原样给出；
> - **【未执行·会打扰用户】** —— 会动用户的进程/窗口，**没有跑**；
> - **【沙箱限制】** —— 本会话的 bash 跑在 `bwrap --unshare-pid` 的独立 PID 命名空间里，
>   只能看见 4 个进程（`ls -d /proc/[0-9]* | wc -l` → 4），**看不到宿主机进程**，故这类量测只能给命令。

---

## 0. 结论速查（2026-10-18 04:23 CST 实测）

| 项 | 实测值 | 命令状态 |
|---|---|---|
| `MemTotal` / `MemAvailable` | 31.1 GiB / **16.8 GiB** | 【已执行】 |
| swap 总量 / 已用 / 剩余 | 4095 MiB / **194 MiB** / 3901 MiB | 【已执行】 |
| swap 用量（本次测试**前**，04:09） | **4.3 MiB**（`pswpout=652` 页 ≈ 2.5 MB） | 【已执行】 |
| `vm.swappiness` | **60**（默认） | 【已执行】 |
| zram | **不可用**：`/dev/zram*` 不存在、`zram` 模块**未加载**、无 `zram-generator` | 【已执行】 |
| root/sudo | 账号在 `sudo` 组，但**沙箱内无法 sudo**（`no new privileges`） | 【已执行】 |
| `systemd-oomd` | **active**；user 会话被发行版设为 `ManagedOOMMemoryPressure=kill` + **`Limit=50%`** | 【已执行】 |
| 历史 OOM | 10-06 11:30:37 `async_slam_tool`（anon-rss **11.85 GB**，全局 OOM）；10-01 21:44:25 `vte-spawn-*.scope`（**25 个进程**被 oomd 整组杀） | 【已执行】 |

**三句话**：
1. **当前 swap 没有耗尽**（194 MiB / 4095 MiB，`MemAvailable` 16.8 GiB）。这次实测里
   **`swap` 从 4.3 MiB 涨到 194 MiB**，是在我跑了 6 次真实 gazebo 短测（Part 2 的验证）期间发生的
   —— 即**桌面应用（firefox / chrome / gnome-software / packagekit）被换出**，不是 gazebo 常驻占着。
2. 用户此前看到的 **“`SwapFree ≈ 1 MiB`” 这个快照本次复现不出来**。硬证据：本次开机
   （2026-10-16 16:06 起，已 1 天 12 小时）在我跑测试**之前** `pswpout` 只有 **652 页（2.5 MB）**
   —— 如果这台机器在这次开机里真的被换满 4 GB，`pswpout` 会是 **~100 万页**量级。
   ⇒ 那个快照应属于 **10-16 16:06 那次重启之前**的开机（换页统计随重启清零）。
3. **真正该防的不是“swap 满”，是“oomd 整组杀”**：10-01 那次是 oomd 因为 user slice 的
   **内存压力 72.74% > 50% 持续 20 s** 把整个 `vte-spawn-*.scope`（25 个进程）一起杀的；
   10-06 那次是**真·全局 OOM**（`async_slam_tool` anon-rss 11.85 GB），oomd 只是恰好是触发分配的任务。
   两者的处置完全不同（见 §5）。

---

## 1. 怎么量：先看全机，再找"谁在 swap 里"

### 1.1 全机口径【已执行】

```bash
free -m
swapon --show
cat /proc/sys/vm/swappiness
grep -E 'Swap(Total|Free|Cached)|Zswap|Zswapped' /proc/meminfo
grep -E 'pswpin|pswpout' /proc/vmstat        # 本次开机累计换入/换出页数（1 页=4 KiB）
cat /proc/pressure/memory                    # PSI：oomd 判据的原始输入
```

本次输出（04:09 与 04:23 两次，中间跑过 6 次 gazebo 短测）：

```
# 04:09（测试前）
内存： 31866 total / 14309 used / 3282 free / 14274 buff-cache / 16756 available
交换：  4095 total /     4 used / 4091 free
NAME      TYPE SIZE USED PRIO
/swapfile file   4G 4.3M   -2
swappiness = 60
SwapTotal: 4194300 kB / SwapFree: 4189948 kB / SwapCached: 2280 kB / Zswap: 0 kB
pswpin 80 / pswpout 652

# 04:23（6 次短测之后）
交换：  4095 total /   194 used / 3901 free
pswpin 80 / pswpout 49041          # 多换出 48389 页 ≈ 189 MB
```

> **怎么读 `pswpout`**：它是**本次开机**累计值。判断“swap 有没有被换满过”，这条比
> `free` 的瞬时值可靠得多。

### 1.2 每进程 `VmSwap` top 15【未执行·沙箱限制（无宿主机 PID 视图）】

在**你自己的终端**（不是本 agent 的沙箱）里跑：

```bash
# ① 每进程 VmSwap（kB → MB），top 15；带 PID 与进程名
for p in /proc/[0-9]*/status; do
  pid=${p#/proc/}; pid=${pid%/status}
  awk -v pid="$pid" '
    /^Name:/{n=$2}
    /^VmSwap:/{ if ($2+0 > 0) printf "%9.2f MB  pid=%-8s %-24s\n", $2/1024, pid, n }
  ' "$p" 2>/dev/null
done | sort -rn | head -15

# ② 顺带看 RSS（谁在吃内存）与 swap 的对比
for p in /proc/[0-9]*/status; do
  pid=${p#/proc/}; pid=${pid%/status}
  awk -v pid="$pid" '
    /^Name:/{n=$2}
    /^VmRSS:/{r=$2}
    /^VmSwap:/{ if ($2+0 > 0) printf "%9.2f MB swap / %9.2f MB rss  pid=%-8s %-24s\n", $2/1024, r/1024, pid, n }
  ' "$p" 2>/dev/null
done | sort -rn | head -15
```

本次在沙箱里跑 ①（同一段 python 逻辑）得到的结果是 **0 行**：

```
# 【已执行】沙箱内 /proc 只有 4 个 PID（bwrap 自己 + 本次 bash + 它的子进程）
$ ls -d /proc/[0-9]* | wc -l
4
$ <扫描 /proc/*/status 的 VmSwap>
processes with VmSwap>0: 0    sum VmSwap = 0.0 MB
```

⇒ “top 15 VmSwap 表”在当前沙箱里**在原理上就取不到**（不是 0，而是**看不见**）。
**替代口径**见 §1.3（cgroup 级，能真正回答“谁在 swap 里”）。

### 1.3 按 cgroup 的 swap（可用的替代口径）【已执行】

cgroup v2 的 `memory.swap.current` 是**宿主机**视角的，且能一路下钻到 scope/unit：

```bash
for f in $(find /sys/fs/cgroup -maxdepth 7 -name memory.swap.current -readable 2>/dev/null); do
  s=$(cat "$f" 2>/dev/null || echo 0)
  [ "${s:-0}" -gt 0 ] || continue
  printf '%9.2f MB swap %8.0f MB mem  %s\n' \
    "$(echo "scale=2;$s/1048576" | bc)" \
    "$(echo "$(cat $(dirname "$f")/memory.current 2>/dev/null || echo 0)/1048576" | bc)" \
    "$(dirname "$f" | sed 's|/sys/fs/cgroup||')"
done | sort -rn | head -15
```

本次输出（04:23，合计 ≈ 194 MB，与 `free` 的 194 MiB 吻合）：

| swap | 所在 cgroup（= 谁的） | 进程侧内存 |
|---:|---|---:|
| **102.35 MB** | `.../app.slice/snap.firefox.firefox-*.scope` | 816 MB |
| **34.98 MB** | `.../app.slice/app-com.google.Chrome-6144.scope` | 231 MB |
| **21.27 MB** | `.../app.slice/app-gnome-gnome-software-*.scope` | 111 MB |
| **9.65 MB** | `/system.slice/packagekit.service` | 480 MB |
| 4.26 MB | `.../app.slice/app-gnome-google-chrome-6144.scope` | 2750 MB |
| 1.99 MB | `.../app.slice/app-code-7884.scope`（VS Code） | 6682 MB |
| 1.49 MB | `.../session.slice/org.gnome.SettingsDaemon.Datetime.service` | 1 MB |
| 1.04 MB | `.../session.slice/org.gnome.SettingsDaemon.Keyboard.service` | 6 MB |
| 1.02 MB | `.../session.slice/pipewire.service` | 0 MB |

顶层：`user.slice` **181.5 MB**、`system.slice` **9.9 MB**、`init.scope` 0 MB。

> **结论**：这次 swap 里装的是**桌面应用**（firefox / Chrome / gnome-software / packagekit），
> 不是 gazebo。要想知道“具体是哪个 PID”，必须在**非沙箱**终端用 §1.2 的脚本 —— 或者直接
> `systemctl status <scope>` 看这个 scope 里有哪些进程。

### 1.4 zram 可用性【已执行】

```bash
ls -l /dev/zram*            # 本次：No such file or directory
command -v zramctl          # 本次：/usr/sbin/zramctl（工具在）
modinfo zram                # 本次：模块在（/lib/modules/6.8.0-146-generic/.../zram.ko）
lsmod | grep '^zram'        # 本次：空（未加载）
ls /usr/lib/systemd/systemd-zram-setup /usr/lib/systemd/system/zram-generator* 2>/dev/null
                            # 本次：都没有 ⇒ 没装 systemd-zram-generator
```

⇒ **zram 现在不可用**（模块在、工具在，但没加载、没有设备、也没有开机自动配置它的 generator）。
启用见 §3.2（需 root）。

### 1.5 root / sudo【已执行】

```bash
id -u                       # 1000
id                          # uid=1000(weicheng) gid=1000(weicheng) 组=1000(weicheng),65534(nogroup)
sudo -n true; echo $?       # 1
getent group sudo           # sudo:x:27:weicheng   ← 账号本身在 sudo 组
```

`sudo` 的真实报错：

```
sudo: /etc/sudo.conf is owned by uid 65534, should be 0
sudo: The "no new privileges" flag is set, which prevents sudo from running as root.
```

> **【沙箱限制】**：这是 bwrap 沙箱的 `no-new-privs` + ID 映射（`nobody/nogroup`）造成的，
> **不代表真机没有 sudo**。真机上请在**自己的终端**确认：
> `sudo -n true && echo HAVE_ROOT || echo NO_PASSWORDLESS_SUDO`。
> 本文所有 “需 root” 的命令**我一条都没有执行**。

### 1.6 systemd-oomd 现状 + 两次历史 OOM 的原始行【已执行】

```bash
systemctl is-active systemd-oomd
systemctl show user@1000.service -p ManagedOOMSwap -p ManagedOOMMemoryPressure -p ManagedOOMMemoryPressureLimit
systemctl cat user@1000.service | grep -n -A2 oomd
systemd-analyze cat-config systemd/oomd.conf
journalctl --no-pager -k | grep -iE 'Out of memory: Killed|oom-kill' | tail
journalctl --no-pager | grep -iE 'oomd.*(kill|Killed)' | tail
```

本次输出：

```
active
ManagedOOMSwap=auto
ManagedOOMMemoryPressure=kill
ManagedOOMMemoryPressureLimit=2147483648        # systemd 的百分比编码：2147483648 = 50%
ManagedOOMPreference=none

# 发行版 drop-in（这是"50%"的来源，不是 /etc/systemd/oomd.conf 的 60%）：
/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf
  [Service]
  ManagedOOMMemoryPressure=kill
  ManagedOOMMemoryPressureLimit=50%

# /etc/systemd/oomd.conf 里只有注释（编译期默认值）：
#SwapUsedLimit=90%
#DefaultMemoryPressureLimit=60%
#DefaultMemoryPressureDurationSec=30s
# 另有 /usr/lib/systemd/oomd.conf.d/10-oomd-defaults.conf:
[OOM]
DefaultMemoryPressureDurationSec=20s
```

历史 OOM 原始行（`journalctl`）：

```
10月 01 21:44:25 systemd-oomd[802]: Killed /user.slice/.../vte-spawn-aabb8d4b-....scope
  due to memory pressure for /user.slice/user-1000.slice/user@1000.service
  being 72.74% > 50.00% for > 20s with reclaim activity
10月 01 21:44:25 systemd[2156]: vte-spawn-aabb8d4b-....scope: systemd-oomd killed 25 process(es) in this unit.

10月 06 11:30:36 kernel: systemd-oomd invoked oom-killer: ... oom_score_adj=-900
10月 06 11:30:37 kernel: oom-kill:constraint=CONSTRAINT_NONE,...,global_oom,
  task_memcg=/user.slice/.../vte-spawn-964a1bbf-....scope,task=async_slam_tool,pid=264550,uid=1000
10月 06 11:30:37 kernel: Out of memory: Killed process 264550 (async_slam_tool)
  total-vm:18841480kB, anon-rss:11854480kB, file-rss:1568kB, shmem-rss:0kB, UID:1000
10月 06 11:30:36 systemd[1]: user@1000.service: A process of this unit has been killed by the OOM killer.
```

**两次的性质不同**（要点）：

| 日期 | 谁杀的 | 判据 | 被杀单位 |
|---|---|---|---|
| 10-01 21:44 | **systemd-oomd（用户态）** | user slice PSI > **50%** 持续 **20 s** 且**有 reclaim 活动** | 整个 `vte-spawn-*.scope`（**25 个进程**，含终端里所有 job） |
| 10-06 11:30 | **内核 OOM killer**（`global_oom`；调用的任务是 oomd） | 全机真的分配不出页（`async_slam_tool` 单进程 anon-rss **11.85 GB**） | 单进程 `async_slam_tool` |

⇒ 10-01 那种“终端整组没了”，是 **oomd 的粒度**（scope 级）导致的，不是内存真不够；
10-06 那种是**真不够**，换 swap/zram 只能拖延，救不了。

---

## 2. 安全地"释放 swap"（不动别人的进程、不 `swapoff`）

### 2.1 先判断“要不要动”

```bash
free -m | tail -1
pswpout=$(awk '/pswpout/{print $2}' /proc/vmstat); echo "本次开机累计换出 $((pswpout*4/1024)) MB"
```

- `SwapFree` 还有一半以上 ⇒ **不用动**（换出几百 MB 是内核正常行为，反而省内存）。
- `SwapFree` < 200 MB 且 `MemAvailable` 也不多 ⇒ 按 §2.2 处理，**不要** §2.3 的 `swapoff`。

### 2.2 正解：重启**那一个**长命进程

swap 里的页只有两种消失方式：**进程退出**（页被丢弃）或 **swapoff**（页被换回内存）。
所以“释放 swap”= 让持有它的长命进程**正常重启**：

```bash
# ① 找到是谁（cgroup 级，可在沙箱/普通终端跑）
systemctl status <scope 名，例如 app-com.google.Chrome-6144.scope> --no-pager | head -20

# ② 找到同 scope 里具体哪个 PID 的 VmSwap 最大（真机终端，见 §1.2）

# ③ 只重启/关闭那一个东西（用户自己的会话操作，本 agent 不会替你做）
#    - 桌面应用：正常关窗口（不要 kill -9，浏览器会丢会话）
#    - 我们自己的栈（gzserver/slam_toolbox/...）：重跑那个 bench 脚本即可，
#      脚本收尾现在只会清"自己这次起的"进程（见 Part 2 §8）
```

> **重启整个 bench 栈的推荐做法**：用 Part 2 改造后的脚本重跑（或直接 `Ctrl-C` 让它走 trap）。
> **不要**用 `pkill -9 -x gzserver` —— 见 §6。

### 2.3 只有在这种条件下才考虑 `swapoff`【未执行·会打扰用户 / 有 OOM 风险】

`swapoff` 要把 swap 里的**全部**页换回 RAM；`SwapUsed` 必须**明显小于** `MemAvailable`，
并留足余量（经验：`SwapUsed_MB * 2 < MemAvailable_MB`），且**没有任何 gazebo/建图在跑**
（否则一换回就把刚空出来的内存又吃掉，直接 OOM）：

```bash
# 手动、在真机终端、确认上面的条件后（本 agent 没有执行，也不建议脚本化）
free -m
sudo swapoff /swapfile && sudo swapon /swapfile
free -m
```

**绝对不要**在 `SwapFree` 只剩几 MB 时跑 `swapoff -a`：4 GB 换回需要 4 GB 空闲内存，
不够就是一轮 OOM（10-06 那种 `global_oom` 会重演）。

---

## 3. 扩容/加 swap、开 zram（**需 root，一条都没有执行**）

### 3.1 加一个 swapfile【未执行·需 root】

```bash
# 0) 看空间与文件系统（本次实测：ext4，/ 上还有 138 GB 空闲）
df -T / ; df -h /
# 1) 建 8 GiB（ext4 用 fallocate 即可；btrfs 要先 chattr +C 且不要用 fallocate）
sudo fallocate -l 8G /swapfile8
sudo chmod 600 /swapfile8
sudo mkswap /swapfile8
# 2) 启用（优先级给低一点，让内核优先用原来的 /swapfile 也行；这里只加容量）
sudo swapon /swapfile8
sudo swapon --show
free -m
# 3) 持久化
echo '/swapfile8 none swap sw,nofail 0 0' | sudo tee -a /etc/fstab
# 4) 回滚
sudo swapoff /swapfile8 && sudo rm -f /swapfile8   # 再从 /etc/fstab 删掉那一行
```

**注意**：

- 不要放在 `/tmp`（本机 `/tmp` 在部分运行环境里是 tmpfs，等于放内存里）、不要放 NFS。
- NVMe 上写 swap 有寿命成本；4 GiB → 8 GiB 只解决“换出容量”，不解决“anon 11.85 GB”那种峰值。
- 加完之后 `vm.swappiness` 仍是 60，内核会更积极换出（见 §4）。

### 3.2 开 zram（压缩内存块设备）【未执行·需 root】

```bash
# 一次性（重启失效）
sudo modprobe zram
sudo zramctl --find --size 8G --algorithm zstd      # 设备名通常是 /dev/zram0
sudo mkswap /dev/zram0
sudo swapon --priority 100 /dev/zram0
zramctl ; swapon --show ; free -m
# 回滚
sudo swapoff /dev/zram0 && sudo zramctl --reset /dev/zram0 && sudo modprobe -r zram
```

```bash
# 持久化（推荐；本机**没装** systemd-zram-generator）
sudo apt-get install -y systemd-zram-generator
sudo tee /etc/systemd/zram-generator.conf >/dev/null <<'EOF'
[zram0]
zram-size = min(ram / 2, 8192)
compression-algorithm = zstd
swap-priority = 100
EOF
sudo systemctl daemon-reload
sudo systemctl start systemd-zram-setup@zram0.service
zramctl ; swapon --show
```

**注意**：

- zram **不增加**总内存，只是把换出的页压缩后放回 RAM（典型 2–3×）。它适合“换出量小但频繁”，
  **不适合** 10-06 那种单进程 11.85 GB 匿名内存的峰值。
- zram 里放 swap 时，社区通常把 `vm.swappiness` 调到 100–180（更愿意压缩换出）；**本机没开 zram，
  所以这条先别照抄**。
- 已有一个 4 GiB 的 `/swapfile`；两者可以并存（zram 优先级更高即可）。

---

## 4. `vm.swappiness` 怎么给

现状：**60**（内核默认）。含义：内存紧张时，内核在“丢 page cache”与“换出匿名页”之间的相对权重。

| 场景 | 建议 | 理由 |
|---|---|---|
| 桌面/交互（本机） | **10–20** | 少换出匿名页 ⇒ 切窗口/编译不卡；代价是可用内存变少、page cache 更早被丢 |
| 服务器/批处理 | 60（默认） | 更愿意换出冷页，保住 cache |
| 用了 zram 做 swap | 100–180 | 换出=压缩，仍在本机内存里，代价低 |
| `0` | **不建议** | 现代内核仍会在必要时换出；`0` 容易造成直接 reclaim 卡顿，且不会减少 OOM |

```bash
# 临时（重启失效）【未执行·需 root】
sudo sysctl -w vm.swappiness=10
# 永久【未执行·需 root】
echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-swappiness.conf
sudo sysctl --system
cat /proc/sys/vm/swappiness
```

> 提醒：**调 swappiness 不解决 swap 满**。swap 满 = 匿名内存 > RAM + swap；
> 唯一正解是减少常驻（关掉不用的栈/浏览器标签）或加容量（§3）。

---

## 5. `systemd-oomd` 调参（先看它现在有多激进）

### 5.1 它现在为什么这么激进【已执行】

- user 会话被发行版 drop-in 设成 **`ManagedOOMMemoryPressure=kill` + `ManagedOOMMemoryPressureLimit=50%`**
  （`/usr/lib/systemd/system/user@.service.d/10-oomd-user-service-defaults.conf`）；
- 触发条件：**user slice 的 PSI > 50% 持续 20 s 且有 reclaim 活动** ⇒ **杀整个 scope**；
- 10-01 21:44 就是这么把 `vte-spawn-*.scope` 里 **25 个进程**一起杀的（用户当时正在终端里干活）。
- 另一条触发线是 `ManagedOOMSwap=auto` + `oomd.conf` 的 `SwapUsedLimit=90%`：
  **swap 用到 90% 时，oomd 会在 user slice 里挑一个 cgroup 整组杀**。本机 swap 只有 4 GiB，
  一旦被桌面应用 + gazebo 填到 90%（≈3.6 GB），这条线就会启动。

### 5.2 三个可调项（按“改动范围从小到大”）

**(a) 只把 user 会话的阈值放宽（最小改动）**【未执行·需 root】

```bash
sudo mkdir -p /etc/systemd/system/user@1000.service.d
sudo tee /etc/systemd/system/user@1000.service.d/50-oomd.conf >/dev/null <<'EOF'
[Service]
# 保留 oomd 兜底，但不再 50% 就整组杀；80% 才动手
ManagedOOMMemoryPressure=kill
ManagedOOMMemoryPressureLimit=80%
EOF
sudo systemctl daemon-reload
systemctl show user@1000.service -p ManagedOOMMemoryPressure -p ManagedOOMMemoryPressureLimit
# 期望：kill / 3435973836（=80%）
```

**(b) 让 user 会话不再被 oomd 主动杀（更彻底，但失去这层保护）**【未执行·需 root】

```bash
sudo tee /etc/systemd/system/user@1000.service.d/50-oomd.conf >/dev/null <<'EOF'
[Service]
ManagedOOMMemoryPressure=auto
ManagedOOMSwap=auto
EOF
sudo systemctl daemon-reload
systemctl show user@1000.service -p ManagedOOMMemoryPressure   # 期望：auto
```

> `auto` 的语义是“由上层/默认策略决定”，对 `user@.service` 来说等于**不再由这条 drop-in 主动 kill**。
> 是否真的完全关闭，以 `systemctl show` 的结果为准（别只看文件）。

**(c) 改 oomd 全局默认（影响所有 unit，包括系统服务）**【未执行·需 root】

```bash
sudo tee /etc/systemd/oomd.conf >/dev/null <<'EOF'
[OOM]
SwapUsedLimit=95%
DefaultMemoryPressureLimit=80%
DefaultMemoryPressureDurationSec=30s
EOF
sudo systemctl restart systemd-oomd
systemd-analyze cat-config systemd/oomd.conf | tail -20
systemctl status systemd-oomd --no-pager | head -8
```

### 5.3 调参的代价（务必知道）

- oomd 的粒度是 **cgroup**：它杀的是**整个 scope/unit**，不是单个进程。放宽它 ⇒ **终端/整个栈一起死**
  的概率下降；但内存真的不够时，内核 OOM killer 会接手，它的粒度是**单进程**、选择更“随机”
  （10-06 挑中的就是 `async_slam_tool`，而它恰好是干活的那个）。
- `ManagedOOMMemoryPressure=kill` 是**兜底**。如果直接删掉，swap 满时可能先经历长时间卡顿
  （直接 reclaim / thrashing）再被内核杀 —— **“卡死”比“被杀”更难恢复**。
- 想保护某个具体 scope（比如正在跑的 bench 栈）：

  ```bash
  systemctl set-property --runtime <scope>.scope ManagedOOMPreference=avoid
  ```

  这是**提示**不是保证（10-01 被杀的那个 scope 没有任何 preference）。

---

## 6. **不要做**的事（血泪清单）

| ❌ 不要 | 为什么 |
|---|---|
| **`swapoff -a` / `swapoff /swapfile`（swap 快满时）** | 要把 4 GB 页换回 RAM；没有 4 GB 空闲 ⇒ 直接触发全局 OOM（10-06 那种）。**本 agent 一条都没跑** |
| **全机 `pkill -9 -x gzserver` / `gzclient`** | 不看 `GAZEBO_MASTER_URI`，会杀掉同机**别人**正在跑/正在看的栈（也正是本仓旧脚本的问题，Part 2 已改） |
| `pkill -9 -f 'ros2 launch'` / `pkill -f ros` | 连别人的 launch / 所有 ROS 进程一起杀；本仓旧脚本里有，Part 2 已清掉 9 个脚本里的这类写法 |
| `pkill -f "gzserver.*$PORT"` 当成“只杀自己” | **空操作**：`gzserver` 的 cmdline 里只有 world 文件，端口在 `GAZEBO_MASTER_URI` 环境变量里 ⇒ 永远匹配不到（`gzclient` 同理） |
| 因为 Gazebo 卡就手动 `pkill -9` | 用户 10-06/10-07 的 `gzclient exit code -9` 就是这么来的（外部 SIGKILL），而“卡”的根因是 §5.4/§4.2 里那两类（模型库阻塞 / RTF 低）。要停就 `Ctrl-C` 让 launch 走正常收尾 |
| `echo 3 > /proc/sys/vm/drop_caches` “清内存” | 只丢 page cache，**不释放 swap**；之后所有东西重新读盘，反而更慢，还可能引发换入风暴 |
| `rm /swapfile` 而不先 `swapoff` | 内核仍占着被删的 inode，swap 不可回收，直到重启 |
| 把 swapfile 建在 `/tmp`、tmpfs、NFS 上 | 不生效/极慢/占内存 |
| `systemd-run --scope` 里跑 gazebo 后又 `systemctl kill` 整个 scope | 会连带杀掉该 scope 里别人的进程；杀之前先 `systemctl status` 看清 |

---

## 7. 本次“执行 / 未执行”总表

| 命令/动作 | 状态 | 备注 |
|---|---|---|
| `free -m`、`swapon --show`、`/proc/meminfo`、`/proc/vmstat`、`/proc/pressure/memory` | **已执行** | §1.1 |
| 扫 `/proc/*/status` 的 `VmSwap` | **已执行（沙箱内 0 行）** | 只有 4 个 PID 可见 ⇒ §1.2 的限制 |
| 扫 `cgroup memory.swap.current` | **已执行** | §1.3，给出了真正“谁在 swap 里” |
| `id -u`、`id`、`sudo -n true`、`getent group sudo` | **已执行** | §1.5 |
| `ls /dev/zram*`、`command -v zramctl`、`modinfo zram`、`lsmod` | **已执行** | §1.4 |
| `systemctl show user@1000.service ...`、`systemctl cat`、`systemd-analyze cat-config` | **已执行** | §5.1 |
| `journalctl` 查历史 OOM | **已执行** | §1.6 |
| `modprobe zram` / `zramctl --find` / `mkswap` / `swapon` | **未执行·需 root** | §3.2 |
| `fallocate` 新 swapfile / `mkswap` / `swapon` / 改 `/etc/fstab` | **未执行·需 root** | §3.1 |
| `sysctl -w vm.swappiness=...` / 写 `/etc/sysctl.d` | **未执行·需 root** | §4 |
| 写 oomd drop-in / 改 `/etc/systemd/oomd.conf` / `systemctl restart systemd-oomd` | **未执行·需 root** | §5.2 |
| `swapoff`（任何形式） | **未执行**（有 OOM 风险） | §2.3 |
| 重启/杀**用户的**任何进程 | **未执行**（只清理过本 agent 自己起的测试栈，见 Part 2） | §2.2 / §8 |

---

## 8. Part 2：本仓脚本收尾的**作用域**改造（只杀自己起的进程）

> 背景：旧脚本收尾是**全机** `pkill -9 -x gzserver` / `gzclient` + `pkill -9 -f 'ros2 launch'`，
> 不看 `GAZEBO_MASTER_URI`。同机有兄弟任务在跑时，这会杀掉**别人**的栈
> （`docs/gazebo_gui_troubleshooting.md` §4.2 / §5.4 记录过这条候选路径）。
> 另有一类"看着像保护、其实永远匹配不到"的写法：`pkill -f "gzserver.*$GZ_PORT"` /
> `pkill -f "gzclient.*$GZ_PORT"` —— **gzserver/gzclient 的 cmdline 里没有端口**
> （端口在 `GAZEBO_MASTER_URI` 环境变量里）⇒ 这两条是**空操作**，等于没保护。

### 8.1 判定规则（三条，**任一**成立才算"自己的"）

实现：**`tools/scripts/lib/scoped_cleanup.sh`**（新增，14 个脚本 `source` 它）。

| # | 判据 | 怎么做的 |
|---|---|---|
| ① | **进程组**（首选，无需名字匹配） | 起栈用 `setsid ros2 launch ... &` ⇒ `PGID=$!`；收尾 `kill -INT -- "-$PGID"` / `kill -9 -- "-$PGID"` |
| ② | **环境变量** | 读 `/proc/<pid>/environ`，`GAZEBO_MASTER_URI` 必须与本 run **逐字相等** |
| ③ | **血缘** | PID 是 `$$`（本脚本）/ `$LPID` / `$LAUNCH_PID` 的**后代** |

- **名字/命令行只用来挑候选**（`comm` 是 `gzserver`/`gzclient`，或 cmdline 匹配某 ERE），
  **绝不单独作为杀的理由**。
- 自伤保护：`$$` 本身、`$BASHPID`、以及 `$$` 的**所有祖先**永不杀
  （防 `bash -c "... pkill ..."` 那种把自己/调用者杀掉的老坑）。
- 对外 API（脚本里就这 5 个，其余是内部实现）：

  | 函数 | 作用 |
  |---|---|
  | `sc_own_gz_pids` | 列出本 run 的 `gzserver`/`gzclient` PID（comm 精确匹配 + 归属判定） |
  | `sc_kill_own_gz <SIG>` | 只杀上面这些 |
  | `sc_kill_own_pattern <SIG> <ERE>` | 只杀"本 run 里 cmdline 匹配"的进程（节点名走这条） |
  | `sc_kill_own_ros2_launch <SIG> [ERE]` | 只杀本 run 的 `ros2 launch`（默认 ERE=`ros2 launch`） |
  | `sc_own_leftovers_desc [ERE]` / `sc_own_gz_count` | 收尾报告用（**作用域内**的残留，不再把别人的进程报成"残留"） |

- 排障开关：`SC_SCOPE_VERBOSE=1` 会把每个"own/skip + 原因"打到 stderr。

### 8.2 逐文件改动

> 说明：`setsid`/`PGID` 这套**本来就有**的脚本只替换了"按名字全机杀"的那几行；
> `run_robot_model_probe.sh`、`robot11_gui_bench.sh` 原来没有 `setsid`，这次一并补上。

| 文件 | 行（改前） | 改成了什么 |
|---|---|---|
| `tools/scripts/diag/run_drift_diag.sh` | 129–134 | `pkill -9 -f 'ros2 launch'` → `sc_kill_own_ros2_launch 9`；`pkill -9 -x gzserver/gzclient` → `sc_kill_own_gz 9`；两条节点 `pkill -9 -f` → `sc_kill_own_pattern 9`；残留复查改为作用域内计数 |
| `tools/scripts/diag/run_nav_lio_timeline.sh` | 169–172、185–188 | 同上；跑前"残留守卫"由 `pgrep -a -x gzserver` + 全机 `pkill` → `sc_own_gz_pids` + `sc_kill_own_gz 9`（**只清本 URI 的**） |
| `tools/scripts/localization/run_gicp_nav_ab.sh` | 92–96、115–118 | 同上（跑前守卫 + 收尾） |
| `tools/scripts/localization/run_nav_smoke_regress.sh` | 44、60–62 | 跑前"残留 gzserver"改为 `sc_own_gz_pids`/`sc_kill_own_gz`；收尾三条 `pkill`（含 `ros2 launch`）→ `sc_kill_own_gz` + `sc_kill_own_pattern` + `sc_kill_own_ros2_launch` |
| `tools/scripts/mapping/mapping_ab_run.sh` | 106、109–112 | `pkill -f 'ramp_section_probe|map_odom_jump_gate'`（TERM）→ `sc_kill_own_pattern 15`；其余三条 → `sc_kill_own_ros2_launch` / `sc_kill_own_gz` / `sc_kill_own_pattern` |
| `tools/scripts/mapping/nav_map_ab_run.sh` | 65–68 | 三条 `pkill` → `sc_kill_own_ros2_launch` / `sc_kill_own_gz` / 两条 `sc_kill_own_pattern` |
| `tools/scripts/mapping/run_mapping_headless.sh` | 132–137 | 同上 + 残留复查改为作用域内 |
| `tools/scripts/mapping/sltune/run_sltune_ab.sh` | 104–107 | `pkill -9 -x gz*` → `sc_kill_own_gz`；节点（原来把 `ros2 launch` 混在 alternation 里）→ `sc_kill_own_pattern` + 单独 `sc_kill_own_ros2_launch`；残留复查改为作用域内 |
| `tools/scripts/mapping/sltune/run_dropab.sh` | 148–151 | 同上（含 `truth_odom` 残留复查） |
| `tools/scripts/regress/run_remap_lowterrain.sh` | 64、78–82 | 跑前守卫 + 收尾整段（含 `pkill -9 -f 'ros2 launch rm_nav_bringup'`）→ 作用域化 |
| `tools/scripts/regress/run_nav_clearance_ab.sh` | 134–137、150–155 | 跑前守卫 + 收尾整段（`ros2 launch` 从 alternation 里拆出来单独 scope） |
| `tools/scripts/regress/run_low_terrain_capture.sh` | 61–64、77、81–85 | 跑前守卫；`pkill -INT -f "bag record"` → `sc_kill_own_pattern 2 'bag record'`；收尾三条 → 作用域化 |
| `tools/scripts/regress/run_robot_model_probe.sh` | 79–80、83、88–89 | **删掉空操作** `pkill -f "gzserver.*$GZ_PORT"` / `gzclient.*`，改 `sc_kill_own_gz 9`；跑前清理同上；残留计数由"永远 0"的 `pgrep -fa "gzserver.*$GZ_PORT"` 改成 `sc_own_gz_count`；**补 `setsid` + `PGID`**，收尾再加 `sc_kill_own_ros2_launch` / `sc_kill_own_pattern`（组杀之后兜底） |
| `tools/scripts/regress/robot11_gui_bench.sh` | 102–105、110、129–143 | **删掉空操作**的 `pkill -f "gzserver.*$GZ_PORT"`/`gzclient.*`，改 `sc_kill_own_gz 9`；`pkill -f rviz2` → `sc_kill_own_pattern 9 'rviz2'`；起窗口/采样时的 `pgrep -x gzclient|rviz2`（全机口径，会把别人的窗口当成"起来了"）→ `sc_own_pids_by_pattern` |

每个改动点都留了一行注释说明作用域规则（`grep -rn '作用域' tools/scripts/*/` 可看全）。

**行为其它方面不变**：参数、输出路径、退出码、trap 时机、`sleep`/等待次数都没有改；
唯一"多出来"的行为是：**跑前也会清掉同 URI 的上一次残留**（原来全机 `pkill` 意外做到的这件事，
现在按 URI 做，仍然做得到）。

### 8.3 证据 A：合成判定矩阵（10 个 decoy，最快、最直接）

脚本：`.tmp_scopetest/harness.sh`（未入库，**只有 decoy + 一个 source 了 lib 的小 SUT**）。
它模拟"同机别人"的 5 类进程，然后让 SUT 用**自己的 URI** 跑一遍收尾：

```bash
$ bash .tmp_scopetest/harness.sh
=== 收尾前（这是 SUT 之外、模拟同机别人的进程）===
  A_other_uri_gz    pid=13  comm=gzserver  uri=http://127.0.0.1:11990
  B_same_uri_gz     pid=14  comm=gzserver  uri=http://127.0.0.1:11999
  C_plain_sleep     pid=15  comm=sleep      uri=http://127.0.0.1:11990
  D_other_launch    pid=16  comm=sleep      uri=http://127.0.0.1:11990   # cmdline="ros2 launch ..."
  E_same_launch     pid=17  comm=sleep      uri=http://127.0.0.1:11999   # cmdline="ros2 launch ..."
  F_other_slam      pid=18  comm=sleep      uri=http://127.0.0.1:11990   # cmdline="slam_toolbox ..."
  G_same_slam       pid=19  comm=sleep      uri=http://127.0.0.1:11999   # cmdline="slam_toolbox ..."
  H_other_gzclient  pid=20  comm=gzclient   uri=http://127.0.0.1:11990
  [J] 自伤测试：自己还活着（pid=67）

=== 收尾后 ===
  ✅ A_other_uri_gz       pid=13   alive（期望 alive）    # 别人的 gzserver：不同 URI ⇒ 活
  ✅ H_other_gzclient     pid=20   alive（期望 alive）    # 别人的 gzclient
  ✅ B_same_uri_gz        pid=14   dead （期望 dead）     # 本 URI 的 gzserver（非后代）⇒ 按 environ 认领
  ✅ C_plain_sleep        pid=15   alive（期望 alive）    # 无关进程
  ✅ D_other_launch       pid=16   alive（期望 alive）    # 别人的 ros2 launch
  ✅ E_same_launch        pid=17   dead （期望 dead）     # 本 URI 的 ros2 launch
  ✅ F_other_slam         pid=18   alive（期望 alive）    # 别人的 slam_toolbox
  ✅ G_same_slam          pid=19   dead （期望 dead）     # 本 URI 的 slam_toolbox
  ✅ I_own_group_gz       pid=21   dead （期望 dead）     # 本 run 进程组里、URI 是别人的 ⇒ 按**进程组**认领
  ✅ J_self_shell         pid=66   alive（期望 alive）    # cmdline 含 slam_toolbox 的调用者自己 ⇒ 自伤保护

RESULT: PASS
```

> 关键点：**B/E/G 被清掉**证明"`/proc/environ` 核对**真的能匹配**"（旧脚本的
> `pkill -f "gzserver.*$PORT"` 在这里会是**空的**）；**A/D/F/H/C 全活**证明不误伤；
> **I 被清掉**证明进程组这条兜底有效；**J 活着**证明自伤保护有效。

### 8.4 证据 B：3 个真实脚本（真 gazebo + 真 ros2 launch + decoy）

做法（`verify_real.sh`）：先起 5 个 decoy（别人的 gzserver/gzclient/"ros2 launch"/"slam_toolbox" +
无关 sleep），再**真跑**目标脚本；等它自己的 gzserver 起来（按 URI 认），给它发 **SIGTERM**
（等价"跑一半被打断"⇒ 走 trap 收尾），然后查"自己的栈是否清干净 / decoy 是否全活"。

```bash
# ① 最便宜：nav_smoke_regress（单次 launch + 回归）
$ bash .tmp_scopetest/verify_real.sh navsmoke3 http://127.0.0.1:11898 150 \
    tools/scripts/localization/run_nav_smoke_regress.sh --tag scv_nav3 --domain 198 --port 11898 --settle 1 --goal 0.0 0.0
[navsmoke3] 本 run 的 gzserver 起来=1（3 s）
[navsmoke3] 起来时本 run 的 gz: 200 208
[navsmoke3] decoys 存活: alive alive alive alive alive
  ✅ 本 run 剩余 gzserver/gzclient  0
  ✅ 本 run 剩余 ros2 launch        0
  ✅ 本 run 剩余节点进程            0
  ✅ decoy 存活数（应为 5/5）       5
[navsmoke3] RESULT: PASS

# ② run_remap_lowterrain（先算覆盖路线再起建图栈）
$ bash .tmp_scopetest/verify_real.sh remap2 http://127.0.0.1:11897 180 \
    tools/scripts/regress/run_remap_lowterrain.sh --tag scv_remap2 --domain 197 --port 11897
[remap2] 本 run 的 gzserver 起来=1（5 s）
  ✅ 本 run 剩余 gzserver/gzclient  0
  ✅ 本 run 剩余 ros2 launch        0
  ✅ 本 run 剩余节点进程            0
  ✅ decoy 存活数（应为 5/5）       5
[remap2] RESULT: PASS

# ③ run_robot_model_probe（原来用**永远匹配不到**的 pkill -f "gzserver.*$PORT"；连跑 3 次）
$ for i in 1 2 3; do bash .tmp_scopetest/verify_real.sh probe$i http://127.0.0.1:1188$i 240 \
    tools/scripts/regress/run_robot_model_probe.sh scv_probe_r$i --duration 5; done
[probe1] gzserver 起来=1（4 s） … [probe1] RESULT: PASS
[probe2] gzserver 起来=1（4 s） … [probe2] RESULT: PASS
[probe3] gzserver 起来=1（4 s） … [probe3] RESULT: PASS
```

> 说明：`probe` 第一次跑时曾出现 1 个 `ros2 launch` + 1 个节点残留（launch 被 SIGKILL 太早、
> 有节点脱组）——这就是为什么给 `run_robot_model_probe.sh` 在组杀之后**又加了**
> `sc_kill_own_ros2_launch` / `sc_kill_own_pattern` 兜底；加完连跑 3 次全 PASS。

### 8.5 证据 C：**跑前残留清理**也只清自己的

场景：先放一个 **同 URI** 的 gzserver（模拟"上一次跑崩了、残留占着同一个端口"）+ 4 个**别的 URI** 的
decoy，然后跑脚本：

```bash
$ bash .tmp_scopetest/verify_startup.sh startup_nav http://127.0.0.1:11895 \
    tools/scripts/localization/run_nav_smoke_regress.sh --tag scv_st --domain 195 --port 11895 --settle 1 --goal 0.0 0.0
[startup_nav] 跑前：本 URI 残留=11 别人=12 13 14 15 16
[startup_nav] 8s 后：本 URI 残留=dead  别人的 gzserver=alive gzclient=alive launch=alive slam=alive sleep=alive
[startup_nav] 收尾后：别人的 gzserver=alive gzclient=alive launch=alive slam=alive sleep=alive
[startup_nav] RESULT: PASS
# 脚本自己的日志也印证了"只认本 URI"：
[regress] 残留 gzserver（本 master URI）：11
```

### 8.6 证据 D：不再有全机 gazebo 清理

```bash
$ grep -rn "pkill -9 -x gz" tools/ scripts/ 2>/dev/null
$ echo "matches=$(grep -rn 'pkill -9 -x gz' tools/ scripts/ 2>/dev/null | wc -l)"
matches=0
```

（本仓没有 `scripts/` 目录，`grep` 会对它报 "No such file or directory" 并返回 2；
上面用 `2>/dev/null` + 计数，`tools/` 下命中 **0** 条。）

**改前 → 改后**（同样口径）：

| 站点（`git grep … HEAD -- tools/` 口径） | 改前 | 现在 |
|---|---:|---:|
| `pkill -9 -x gzserver` / `gzclient` | **18** 行（其中 **11** 行在本次点名的 9 个脚本里；另 7 行在同族的 4 个脚本里，也一并改了） | **0** |
| `pkill -9 -f '…ros2 launch…'`（含混在 alternation 里的） | **11** 行 | **0** |
| `pkill -f "gzserver.*$GZ_PORT"` / `"gzclient.*$GZ_PORT"`（**空操作**） | **7** 行可执行（`run_robot_model_probe.sh` 4 + `robot11_gui_bench.sh` 3；另 2 行是**注释**，保留作为"此路不通"的说明） | **0** |
| `pkill -f rviz2`（全机） | **1** 行 | **0**（改成 `sc_kill_own_pattern`） |

### 8.7 诚实的边界（本 part **没有**验证到的）

1. **`tools/scripts/control/start_sentinel.sh` 没改**（不在本次 9 个脚本清单里，是"一键启动"人工工具）。
   它仍然是：

   ```
   165:  pkill -f gzserver
   166:  pkill -f gzclient
   169:  pkill -f ros          ← 这条会杀掉机器上**所有** ROS 进程（包括别人的）
   172:  pkill -f rviz2
   ```

   ⇒ **建议**：人工启动一次前先确认没有别人的栈在跑；要彻底解决，把它也改成
   "`setsid` + 本 URI 核对"（本次按"不动范围外文件"处理）。
2. `robot11_gui_bench.sh` 里 `ps -o rss= -C gzclient|gzserver|rviz2` 的**采样**仍是全机口径
   （**只读、不杀**）。它需要真显示器，本沙箱验证不了，所以只改了"起窗口判定"和"清理"。
3. 除上面 3 个真跑脚本外，其余 11 个只做了 **`bash -n` 语法检查 + 合成判定**，
   **没有逐个跑完整流程**（它们动辄 5–10 分钟）。逻辑改动是同一套 `sc_*` 调用，风险面一样。
4. **沙箱限制**：本 agent 的 bash 在独立 PID 命名空间里，**看不到宿主机进程** ⇒
   无法在"宿主机上真的有一个别人的 gazebo"的条件下演戏；证据是"同命名空间内、不同 URI 的 decoy 全活"，
   加上规则本身**只按进程组/环境变量/血缘**判定（不依赖"谁的机器"）。
5. 两个**同脚本**并发跑会共用同一个 `GAZEBO_MASTER_URI`（端口硬编码），仍会互相抢端口 ——
   这是端口独占问题，不是 kill 作用域问题（脚本本来就会在端口占用时退出 3）。
6. 所有测试都在本 agent 自己的 PID 命名空间里，**跑完进程随命名空间销毁**；
   `ps`/`pgrep` 复查残留：**(none)**。
