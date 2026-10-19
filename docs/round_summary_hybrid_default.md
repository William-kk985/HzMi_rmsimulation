# 本轮总结：从「z 原点」到「hybrid 成为 nav 默认」

> 范围：`d36dbc1..95ae404`（24 个提交，30 文件 +5935/−209）。
> 一句话：**把 `odom` 的"朝向 + 原点"两件事一次修干净，并顺路挖出四个静默失效的真实缺陷；最后 `hybrid` 成为 `lio:=small_point_lio` 的默认重力对齐档。**
> 标注约定：**【实测】** = 本仓跑出来的数；**【推断】** = 未实测；**【引用】** = 外部资料。

---

## 一、这轮定下的五件事（结论 + 判据）

| # | 结论 | 关键证据 |
|---|---|---|
| 1 | **`odom` 的 z=0 = 初始化时刻的地面**（`gravity_align_z_origin:=ground`，仅在重力对齐档生效；`h` 由 URDF 解析 + 运行时地面互检） | init 时 URDF 地面 0 / `base_link` +0.1025 / `livox` +0.2595（精确）；`h_meas` Δ ≤ 2 cm ⇒ INFO |
| 2 | **安装参数真的从 URDF 读**（`gravity_align_rpy` / adapter `xyz,rpy` / `h` 先验），旧表降为交叉校验 | 6 组 parse-vs-table **Δ=0**；scratch 把 30°→20° ⇒ 派生值变 **−20°** 且 **WARN**；默认路径 **执行集 0 改 0 删** |
| 3 | **Livox 插件 `cloud_frame=sensor` 档的坐标 bug 修好**（点云曾被整体平移 `(0.6, 191.9, 70.5) mm`） | 拿网格已知墙面当基准：y=−8.5 残差 **−0.1919 → 0.0000 m**；`h_meas` **0.189 → 0.2603/0.2597（两世界都 INFO）** |
| 4 | **nav2 的高度带会在四个地方静默丢数据**（局部半平面 / 全局整层清空 / gicp 门自锁 / `map→odom` 反向注入倾角），现已有三处修复 + 一条只读守卫 | 见第二节 |
| 5 | ⭐ **`hybrid` 成为 nav 默认**（`gravity_align_mode:=auto`：`lio==small_point_lio` ⇒ `hybrid`，否则 `off`） | 默认 `fastlio` 路径**运行时逐字节不变**；`off`+SPL **逐字节回退**；显式 `hybrid`+`fastlio` 仍**逐字报错** |

### 为什么是 `auto` 而不是直接把默认写成 `hybrid`

launch 的默认 `lio` 是 **`fastlio`**，而 E1/E1b 的补丁只存在于 `small_point_lio`；`_check_gravity_align_contract` 对"非 off + 非 SPL"是**硬报错**。
⇒ 默认直接写 `hybrid` 会让**所有默认跑法在 t=0 报错**。`auto` 让两种默认都成立。

---

## 二、四类"静默失效"及其修复

| # | 位置 | 症状（都**无日志**） | 修复 |
|---|---|---|---|
| 1 | **局部** `obstacle_layer`（`odom` 帧） | 扫描面倾斜 ⇒ 高度带只收一条带：**in-band 333/776**、kept-y **[0.112, 3.568]**（单侧）⇒ **半平面盲区**（局部图 53.2% 面积永不 lethal） | `hybrid` 使盘面水平 ⇒ **776/776**、kept-y 双侧 |
| 2 | **全局** `obstacle_layer`（`map` 帧） | `map` 里扫描面 z ≈ **−0.1786** ⇒ **0/774** 在带内 ⇒ 全局图恒等于 static+inflation（`3713` = pgm occupied 格数，53 帧 min=max） | `457701c`：opt-in 覆盖层把**两处** `min_obstacle_height` 放到 **−1.0**；盒子试验：全局 **0 格 → 14/121 格**变化，删除后**逐格回退** |
| 3 | **gicp 首帧门** | `dt` 取"与上次**采纳**帧的戳差"，从未采纳 ⇒ `dt=0` ⇒ 门限卡下限 0.1 m ⇒ **采纳 0/25** ⇒ 6 s 后判失效、停发 `map→odom` | `9bb8f2f`：`dt` 改"上一帧收到" + 首帧免创新门 + 连续 3 拒**自愈重锚** + 失效态探针 ⇒ **采纳 100%**、创新 p50 **0.134 → 0.0097**、`TFDrop 42 → 0`、外推 `271 → 3` |
| 4 | **`map→odom` 的倾角** | `off` 档 `map→odom` 带 **29.75° roll**，把 30° 安装倾角**反向抵消** ⇒ `map` 帧反而是平的 | **不是缺陷而是"活定位器"的正常结果**；它解释了为什么**全局半不吃倾斜病的亏**，也**推翻了 §12.4 的"加倍成 60°"推断** |

**只读守卫 `band_guard`**（`65a0a85`，opt-in）：把 1、2 两种失效**当场喊出来**（`in-band / kept-y / z / frame / band` + 检查建议）；同栈 A/B 证明它**不改变运行结果**（local lethal 266→265、global 3898→3897）。

---

## 三、提交清单（24 个）

### A. 帧契约与 LIO（6）
| commit | 内容 |
|---|---|
| `1b5db4b` | **E1b**：`odom` 的 z 原点 = 地面（`gravity_align_z_origin` / `gravity_align_h` / `h_meas` 互检 / 独立 TU `gravity_align_ground.cpp`） |
| `4987dc4` | 契约 §4.7（z 原点规范）+ measurements §10（验收 A/B） |
| `f9bf4a9` | **安装参数从 URDF/xacro 解析**（+ 弧度守卫 `_assert_radians`；旧表降为交叉校验） |
| `023d2e8` | 契约 §13.5 + measurements §11（解析对照 + 30°→20° 决定性测试） |
| `6473a29` | ⭐ **`gravity_align_mode` 默认 `auto`** + 唯一解析点 + 契约 §4.10 + measurements §18 |
| `95ae404` | §18.4 守卫样本数订正 |

### B. 物理/世界（2）
| commit | 内容 |
|---|---|
| `72c7c07` | **修 Livox 插件 sensor 档**：不再叠加父 link 系 `offset.Pos()`（`parent` 档逐字节不变） |
| `ec5028f` | **RMUL2026 地面校准**到 world z=0（Δz = −0.049662） |

### C. 定位/代价图（4）
| commit | 内容 |
|---|---|
| `d475605` / `bcb652b` / `dafcb4e` | gicp `~/gate` 逐帧观测量 + 创新分量（dx/dy/dz）+ 探针 |
| `9bb8f2f` | ⭐ **修 gicp 首帧门死锁**（4 个新参数，默认=修好；逐字节回退四开关） |
| `555eb96` | measurements §14（复现/成因/修法/验收/全局半）+ §9.10 订正 |
| `457701c` | ⭐ **修全局 `obstacle_layer` 被 z 闸静默清空**（opt-in 覆盖层） |
| `bd18eaa` | §15 抬头措辞订正 |

### D. 工具/守卫/地图（4）
| commit | 内容 |
|---|---|
| `acae43f` / `49a4328` | 打包脚本：项目根少一级、`tar`/`mv` 目标、分发包 setup 根目录自适应（真跑通过：398 MB / 2187 条目） |
| `b1434b8` | **地图重建**（新名字 + sidecar，默认未提升）+ measurements §16 |
| `65a0a85` / `d881147` | **band_guard**（opt-in 只读守卫）+ measurements §17 |
| `9835957` | measurements §12（`slam_nav` 与 `nav+global:=scan` 验收） |

### E. 文档（1）
| commit | 内容 |
|---|---|
| `38a2d94` / `c616d69` / `c0b3803` | measurements §13（世界校准 + 插件 bug、作废值换算、日期口径） |

---

## 四、被**作废/修正**的旧数字（引用前必看）

| 旧说法 | 现在 | 原因 |
|---|---|---|
| 「机器人陷进地面 **6.9 cm** / 可见地面 0.1183 / odom 里 +0.069」 | ❌ **作废**：真实接触地面 0.049662，**陷地 = 0（2 µm）** | 0.1183 = 真实接触地面 + **插件 0.07053** |
| `h_meas` **0.1894 / 0.1906**（ERROR） | 修插件后 **0.2603 / 0.2597（INFO）**；校准后 **0.2616** | 插件坐标 bug |
| §10.4 的 ground-in-odom **0.01876 / 0.02674** | 同定义新值 ≈ **−0.0518 / −0.0438**（各减 0.07053；y 减 0.1919） | 同上 |
| §9.10「off 118 / hybrid 2」 | ❌ **错**：两档都 `采纳 0/N`（门死锁），条数只取决于跑了多久；修门后 **3 / 0** | gicp 门自锁 |
| §12.4「off 会加倍成 **60°**、只剩 1.19 m 条带」 | ❌ **不成立**：活定位器下 `map→odom` 带 −29.75° roll 抵消倾角，`map` 帧两档都平 | **推断**当成了实测 |
| 「39.6/40.4% 出带」 | ⚠️ **与 14.2% 不是同一个量**：前者是另一次 `sensor` 档 mapping 跑（755 波束），后者是本次 nav A/B（96/678）；**可复用的是机理（斜面 ⇒ 定向丢），不是百分比** | 跨跑混用 |
| 「创新 0.11–0.22 / 采纳 0 = 旧图偏置」 | ❌ **错**：那是**门死锁**的像；门修好后旧图也 **100% 采纳**，地图偏置表现为**一次性 `map→odom` 常量修正** | 归因错 |
| 「修出生点抬 6.9 cm」（1c 方案） | ❌ **取消**：车根本没陷进去 | 诊断被推翻 |
| §4.1 / §13.2 / §13.4 的「默认 off」 | ⚠️ **已被 §4.10 取代**（正文保留，§4.10 声明取代） | 默认已改 `auto` |

---

## 五、仍未解决 / 未验证（诚实清单）

### A. 四条硬化（已知、未做）
| # | 问题 | 影响 |
|---|---|---|
| 1 | **默认模型（hzmirm，13 link）底盘自转只有 3%**（0.80 → 0.025 rad/s；`robot11` 焊接版是 100%） | **建图覆盖不够**（26 min 只走 10.6 m）⇒ 新地图没提升 |
| 2 | **`small_point_lio` + 默认模型（`R_bl=I`）SIGSEGV**（~12 s，可复现，无 `/odom`） | **默认模型 + SPL** 这条组合直接崩 |
| 3 | **`cloud_frame=parent` 档疑似 `frame_id` 与点云所在系不一致**（驱动版地面 +0.283 m、`map→odom z=+0.403`） | 若成立 ⇒ **默认挂法**有偏移；**未证实** |
| 4 | `map_archive.sh save` **静默忽略 `--resolution`** | 存图传分辨率会**无声不写盘** |

### B. 基线/资产
- **仍在用旧地图**（带插件偏置；gicp 吸收成常量修正，nav 可用）。新候选：`RMUC2026_v4.pcd`（墙面残差 p50 **0.0436 → 0.0020 m**、gicp score **5.1×**）等，**未提升**。
- **`§10` / `§12` 基线未在"当前 master + 修好插件 + 校准世界"上重跑**（旧表是混合条件下测的）。
- **全局 `−1.0` 覆盖可能只是旧地图的拐杖**（`map→odom` z 差 0.113 m ≈ 插件 0.0705 + 世界 0.0497）⇒ **地图重建后应复测"还需不需要"**。

### C. 未测组合
`hzmirm` / 默认模型在 Gazebo 实跑 · `hzmirm` 云台≠0 · `robot11_mount:=plugin|urdf` · `h_meas` 的 **3–5 cm WARN 带** · `amcl` / `beluga` / `small_gicp` / `fastlio` / `pointlio` 配新默认 · `slam_nav` 配新门 · `RMUC2026` 配全局 scan 修复 · **gz backend** 配以上全部 · 真机。

### D. 文档与运维
- `docs/README.md` 的取代表需要为**新的数字口径**（14.2% / 39.6–40.4% / 41.2% 三者关系）加一行。
- `spl_ab_bench.sh` 收尾会把 `build/` 二进制还原成 `.tmp_uc/bins/after`（zfix_prev）⇒ **跑完要手工还原 zfix `58bebfb8…`**。
- `band_guard` 默认路径新增 **2 个恒假 Action**（不是严格"0 新增"；节点不启动、参数/控制路径不变）。
- **KISS-Matcher 全局重定位调研未做**（曾提议，未派）。

---

## 六、怎么跑（当前正确配方）

```bash
# nav（hybrid 现在自动生效；不再需要显式 gravity_align_mode:=hybrid）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=small_point_lio localization:=gicp \
  nav:=mppi planner:=smac2d spin_speed:=0.0 robot:=robot11 nav_rviz:=True \
  robot11_mount:=sensor band_guard:=True

# 只想回退到旧行为（逐字节）
... gravity_align_mode:=off
```

- `gravity_align_mode:=auto`（默认）⇒ `lio==small_point_lio` 时取 `hybrid`，否则 `off`；t=0 会打一行收据说明解析结果。
- `band_guard:=True` 建议在**回归/改帧/改高度带/改代价图配置**时开。
- 地图：**仍用默认资产**；想试新候选按 `docs/map_assets.md` §4.2 走。

---

## 七、文档索引导航（本轮新增/更新）

| 文档 | 内容 |
|---|---|
| `docs/frame_contract_v3.md` **§4.7** | z 原点规范 |
| 同上 **§4.8** | 世界地面校准是前提 |
| 同上 **§4.9** | `band_guard` 登记 |
| 同上 **§4.10** | ⭐ **默认档决定**（取代 §4.1/§13.2/§13.4 的"默认 off"） |
| `docs/frame_contract_v3_measurements.md` **§10–§18** | 依次：z 原点验收 · URDF 解析 · slam_nav/global-scan · 世界校准 + 插件 bug · gicp 门 · 全局层 · 地图重建 · 守卫 · 默认 auto |
