# 世界资产：RMUC2026（由附件 STL 生成的全场地图）

> 入口：`ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC2026 ...`
> 本文回答：这个场地是什么、怎么从 STL 变过来的、出生点怎么选的、地图/PCD 怎么来的、
> 实测跑通了什么、以及**哪些坑是真的**（台阶/通道 vs 我们 0.22 m 半径的车）。

---

## 0. 一句话

把附件 `easystl.stl`（毫米单位、Z 朝上、29.15 x 16.05 m 的 RMUC 级别全场）按
**scale 0.001 + z 抬 1.6413436 m** 变成 Gazebo 世界 `RMUC2026`，并配套生成
2D 栅格图与先验 PCD，四种形态实测：**建图 ✅ / 2D+AMCL 导航 ✅ PASS / 3D+GICP 导航 ✅ PASS
（含一条"定位仍在缓慢收敛"的打折项）**。
**场地本身可跑**，但场地几何对我们这台车不友好：0.20/0.30 m 台阶上不去、两个半场之间
只有 0.45~0.50 m 宽的通道（车直径 0.44 m）⇒ 实际可通行区 ≈ **176 m²（占图上游离区 237.8 m² 的 74%）**，
且基本被限制在出生点所在的那半个场（见 §6）。

---

## 1. 资产清单与来源

| 文件 | 说明 |
|---|---|
| `src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl` | **实际被 Gazebo 加载的那份**（`GAZEBO_MODEL_PATH` 指向 `<share>/hzmi_rm_simulation/meshes`，见 §2.3）。2,295,084 B |
| `src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world/model.{config,sdf}` | `model://RMUC2026_world` 的独立入口（与 world 文件里内联的那份等价） |
| `src/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/RMUC2026_world.world` | 世界文件（结构镜像 `RMUL2026_world.world`） |
| `src/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world/model.{config,sdf}` + `meshes/RMUC2026.stl` | 与上面对称的第二份目录树（镜像 RMUL2026 的既有结构；这份 `meshes/` 目前**不被任何路径引用**，只是照抄结构，占 2.2 MB） |
| `src/rm_nav_bringup/map/RMUC2026.{pgm,yaml}` | 2D 栅格图。⚠ **当前这一份不是本文 §4.1 的合成产物** —— 2026-10-06 起换成实跑建图的 `/map`，2026-10-07 又提升到 `RMUC2026_v3` 那次会话的图（`577x326`、`origin [-25.2,-9.09]`）。**当前默认对 / 换图 / 回滚见 `docs/map_assets.md`**；§4.1 描述的那份合成图现在在 `map/attic/RMUC2026.pgm.synth.bak` |
| `src/rm_nav_bringup/PCD/RMUC2026.pcd` | 先验点云。⚠ 同上：当前是实跑建图产物（2026-10-07 提升，**203,352 点**），§4.2 描述的那份合成点云现在在 `PCD/attic/RMUC2026.pcd.synth.bak` |
| `src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py` | +`WorldType.RMUC2026` 一项（出生点）与对应 GroupAction |
| `src/rm_nav_bringup/launch/bringup_sim.launch.py` | `amcl_init_x/y` 字典 +`'RMUC2026': 0.0` |

**STL 出处（只读输入，未修改）**：
sha256 `cc723f3156999598127dfd34d1467767773708ecd46e78f6ad4c40de597d0846`
（原附件 `/home/weicheng/.dsh/attachments/v1/files/cc/cc723f31.../easystl.stl`，
工作副本 `.tmp_cache/stl_view/easystl.stl`）。仓库里两份拷贝的 sha256 与原件**逐个相同**（`sha256sum` 复核）。

---

## 2. 网格转换（mm → m、z 抬升、碰撞）

### 2.1 三处改动（相对 `RMUL2026_world.world`）

| 项 | RMUL2026 | RMUC2026 | 为什么 |
|---|---|---|---|
| `<scale>` | `0.001`（world 内联）/ `0.01`（model.sdf，未生效） | **`0.001 0.001 0.001`** | 本 STL 原生单位 = **毫米**（bbox 29150 x 16050 x 1730），不缩放就是 1000 倍大 |
| 姿态旋转 | `0 0 -2.6 -1.5705 0 0`（绕 X 转 -90°） | **`0 0 1.6413436 0 0 0`（无旋转）** | `RMUL2026.stl` 是 **Y 朝上**（bbox y 只有 520 mm）所以那份要转；本 STL 的 z 只有 1730 mm ⇒ **本来就是 Z 朝上**，转了反而躺倒 |
| 模型 pose | `8.5 4.5 2.65`（把场地挪到世界一角） | **`0 0 0 0 0 0`** | 让 **mesh 米制坐标 == world 坐标**，地图/PCD/出生点全部同一个系，少一层换算 |

z 抬升量的来历：底板（可行驶地面）**顶面**在 mesh 局部坐标 `z = -1.64134362793 m`
（不是栅格拟合值 —— STL 顶点里有 4412 个点精确落在 `-1641.34362793 mm` 这个平面上），
所以 `<visual>/<collision>` 的 pose 抬 `+1.6413436 m`，场地地面正好落在 `world z = 0`。
**不抬的话机器人会在 z=0 出生、地面在 -1.64 ⇒ 直接悬空 1.6 m 掉下去**（RMUL 那份 world 的
`z` 参数注释里记过同类事故：坠落中做重力初始化会让 LIO 地图倾斜）。

### 2.2 世界级设置：**一个都没改**

`gravity / magnetic_field / atmosphere / physics(ode, 0.001 s, RTF 1, 1000 Hz) / scene / audio / wind /
spherical_coordinates` 与 `RMUL2026_world.world` **逐字节同值**；那份 world 里没有 `<plugin>`，
本份也没有。唯一动过的是 5 盏灯的 pose：类型/颜色/衰减/方向全同，位置整体平移
`(-8.5,-2.88)`（RMUL2026 场地中心 `(8.5,4.5)` → RMUC2026 场地中心 `(0,1.62)`），
高度取 RMUL2026 world 的 `<state>` 段实际生效值 `z=7~14`（那份定义处写 `z=1`、运行时被 state 覆盖）。

### 2.3 `model://` 是怎么解析到的（踩坑记录）

`GAZEBO_MODEL_PATH` 由 `gzserver.launch.py → GazeboRosPaths.get_paths()` 在**启动时**扫描
所有包的 `package.xml` 里 `<export><gazebo_ros gazebo_model_path=...>` 拼出来（本仓库 =
`<share>/hzmi_rm_simulation/meshes`）。所以：
`model://RMUC2026_world/meshes/RMUC2026.stl` → `<share>/hzmi_rm_simulation/meshes/RMUC2026_world/meshes/RMUC2026.stl`。
⇒ 新资产加完后**必须 `colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup`**
（`install/` 是逐文件符号链接，不 build 新文件不会出现），然后 `source install/setup.bash`。

### 2.4 网格质量与碰撞：实测能用

已知（`.tmp_cache/stl_view/README.md`）：45,900 三角形 / 137,700 顶点、**非水密**、352 个独立 body、
绕向不一致、有大量重合/近重合薄片。
**做法**：`<collision>` 直接用同一份 raw mesh（和 RMUL2026 同款写法），不额外做简化凸包。
**实测量到的证据**：
* `spawn_entity` 报 `Successfully spawned entity [robot]`；
* 机器人静止时 `/odom_ground_truth`（Gazebo 平面底盘插件的真值）**z = 0.0600 m**，
  正好等于轮半径 0.06 ⇒ **站在地面上，既没沉下去也没悬空**；整场跑动中（2.6 m 直行）z 恒为 0.0600；
* 45,900 面静态 trimesh 没有产生任何 ODE/碰撞告警（`grep -E "\bODE\b|trimesh|collision"` 空）；
* LiDAR 打到台阶立面的点云与 STL 几何重合度 87.5%（§5.1）。
⇒ **不需要简化碰撞体**。若以后要省 CPU，可只把"可行驶地板 + 台阶立面"做成简化碰撞，但**没有证据表明需要**。

---

## 3. 出生点：怎么选的

推导（脚本 `.tmp_cache/rmuc2026/build_assets.py`，全部可复跑）：

1. 用 `.tmp_cache/stl_view/heightmap.npz`（0.02 m 顶视高度栅格）聚合到 **0.05 m** 栅格，
   取每格**最高**面高度（保守，保住薄墙）；
2. 相对底板 `z=-1.6413436` 抬升 > **0.15 m** 的格 = **障碍**（0.20/0.30 m 台阶、塔、柱、墙全落入），
   ≤ 0.15 m 的格 = **可行驶**；场地外沿（栅格无数据）也标成障碍 ⇒ 规划器不会往空处走；
3. 在**严格平台面**（抬升 ≤ 0.05 m）上做距离变换，要求 `clearance ≥ robot_radius(0.22) + margin(0.08) = 0.30 m`；
4. 取出生点所在连通域内**离障碍最远**的点：

   | | world (mesh 米制) | clearance | 该点 ±0.55 m 邻域高度起伏 |
   |---|---|---|---|
   | **选定** | **(+10.925, +2.525)** | **2.704 m** | ≤ 0.004 m（真平） |
   | 左半场对称点 | (-10.925, +0.775) | 2.706 m | ≤ 0.004 m |

   两半场镜像、数值等价（差 2 mm）；**取 +x 半场**只为与 RMUL2026 的出生点象限一致、便于叙述。
   实测复核：`/odom_ground_truth` 出生位姿 = **(10.9249, 2.5215, 0.06)**，与设定值差 5 mm 量级。

5. 出生 **z = 0.2**（与 RMUL2026 同口径）：轮半径 0.06 ⇒ 落地后 base_link 停在 0.06，留 0.14 m 余量，
   既不长时间悬空（避免 LIO 在坠落中做重力初始化），也不会插进地面。

launch 里的写法（`rm_simulation.launch.py::get_world_config`）：

```python
WorldType.RMUC2026: {
    'x': '10.925', 'y': '2.525', 'z': '0.2', 'yaw': '0.0',
    'world_path': 'RMUC2026_world/RMUC2026_world.world'
}
```

---

## 4. 地图与 PCD 是怎么产出的

> **⚠ 2026-10-07 现状（先读这一行）**：本章 §4.1/§4.2 记录的是 **2026-10-05 的"由 STL 合成"路线**，
> 它**已经不是当前默认资产** —— 那两份现在分别在 `map/attic/RMUC2026.pgm.synth.bak`（+`.yaml.synth.bak`）
> 与 `PCD/attic/RMUC2026.pcd.synth.bak`（`git show 280888e:…` 也有原始版本），可随时用 §4.4 的工具复跑。
> **当前默认对 = 实跑建图产物**（2026-10-07 提升到 `RMUC2026_v3` 那次会话的 2D + 3D）：
> 清单、实测对照、换图与回滚全在 **`docs/map_assets.md`**。本章保留原样，作为"合成路线怎么做的"的存档。

两条路线都试了：**当初资产用 (a) 从 STL 合成**，(b) 真实建图只作**交叉验证证据**（§5.1）；
**现在反过来** —— (b) 实跑建图是当前默认资产，(a) 退到 `attic/`（见上面的现状说明）。

### 4.1 `map/RMUC2026.pgm` + `.yaml`（路线 a；**现已在 `map/attic/`，历史存档**）

* 由 §3 的 0.05 m 栅格直接出图：`free → 254`、`occupied/外沿 → 0`，
  **阈值 0.15 m**（相对底板抬升）。取 0.05 m 而不用 0.03：0.03 m 时 free 面积 217.1 m²、
  0.15 m 时 237.8 m²，差的 20 m² 全是台阶根部的栅格过渡带；两种阈值下"能被 0.22 m 车走的部分"
  几乎一样（都被 inflation 吃掉），取 0.15 与"台阶算墙"的语义一致。
* 尺寸/原点：`resolution 0.05`、`600 x 341`、**`origin = [-25.925, -9.425, 0]`**
  = 网格左下角 `(-15.0, -6.9)` 减去出生点 `(10.925, 2.525)`。
* **map 系 = 出生点相对系**（与 RMUL2026 同口径）：出生点在 map 里就是 `(0,0)`，
  所以 `bringup_sim.launch.py` 的 `amcl_init_x/y` 给 `0.0`；`gicp`/`icp` 的
  `initial_pose: [0,0,0]` 也**不用改**（这就是选"出生点系"而不是"world 系"的原因：
  世界系的话 `initial_pose` 得按 world 注入，而 launch 并不注入它）。
* 面积：free **237.8 m²**、occupied 229.4 m²、场地外沿 44.4 m²（也标占用 = 围墙）。

### 4.2 `PCD/RMUC2026.pcd`（路线 a；**现已在 `PCD/attic/`，历史存档**）

* 几何：把 STL 采样成"**地面以上薄壳**"（镜像 `PCD/RMUL2026.pcd` 的性格 —— 那份 z 跨度 0.665 m、
  含地面切片 + 低矮障碍），高度带 **`[地面-0.08, 地面+0.60] m`**；
* 采样：按面积均匀采 3,000,000 点 → 留带内 2,036,997 点 → **按法向分两档做体素下采样**：
  * 近水平面（`|nz|>0.7`，地面/台阶顶面）：**0.25 m** ⇒ 11,124 点（只提供 z 约束，稀疏即可，对应 RMUL 的"地面切片"）
  * 近竖直面（台阶立面/塔/柱/墙）：**0.10 m** ⇒ 15,494 点（提供 x/y 约束，要密）
  * 合并 **26,618 点 / 0.85 MB**，字段与 RMUL2026.pcd 同构（8 x float32：x y z intensity + 4 个零法向/曲率）
* **坐标 = map 系**，其中**地面放在 `z = -0.06`**：这是"`initial_pose: [0,0,0]` 语义下地面该在的位置"
  —— 初值把 base_link 放在 map 原点，而车静止时 base_link 离地 0.06 m，所以地面 = `-0.06`，
  初值**精确正确**（对照：`RMUL2026.pcd` 的地面在 `-0.32`，因为那份存的是 LIO `camera_init` 系，
  与初值语义差 0.26 m，靠 GICP 的 `max_correspondence_distance=1.5 m` 吸收）。
* 体量对照：节点日志 `原始 26618 → 体素 0.100 m 后 21161`（RMUL2026 是同 leaf 下 12450）——
  场地面积是 RMUL2026 的 2.4 倍，按面积折算密度相当。**若 align 太慢**，回退阶梯：
  `voxel_leaf_size 0.10 → 0.15/0.20`（或把本 PCD 的水平面 0.25 → 0.35）。

### 4.3 现有 `map/RMUC.pgm` 能不能复用？**不能**（有证据）

`RMUC.pgm` 尺寸确实和本 STL 接近（28.85 x 15.05 m vs 29.15 x 16.05 m），但几何对不上：

* 用与 §5.1 同一把尺子（chamfer：`RMUC.pgm` 的占用格 → STL"可行驶地板轮廓"）在 **±4 m 平移**里搜：
  最好 **mean 0.525 m / 命中率 10.7%**，而全窗口命中率**中位数 7.8%**（≈随机基线），
  峰值显著性为负（最好点落在搜索窗边缘，不是峰）；
* 两个先验几何假设下更直接：`STL+(14.575, 6.401)`（左下角对齐）mean **0.732 m / 12.3%**、
  `STL+(14.425, 7.424)`（中心对齐）mean **0.718 m / 11.3%** —— **都在随机水平**；
* 语义上也对不上：`RMUC.pgm` 的 free 有 **366.8 m²**，本 STL 的可行驶地板只有 **237.8 m²**
  ⇒ RMUC2024 那张图是"几乎全平"的 2024 场地，与这张带 0.20/0.30 m 大台阶的 2026 场地
  **不是同一套布局**（与早先 FFT 配准"回到随机水平"的结论一致）。
⇒ 结论：**不复用**，2D 图与 PCD 都从本 STL 现生成（§4.1/§4.2）。

### 4.4 一条命令复跑：`tools/scripts/world/stl_to_world.py`

§3 / §4.1 / §4.2 的做法已整理成一个脚本（约定全部参数化，**换 STL 不用改代码**）：

```bash
# 复现已提交的 RMUC2026 资产（--map-grid 是当初手工取的整数边界，见下面的差异 ②）
python3 tools/scripts/world/stl_to_world.py \
    --stl .tmp_cache/stl_view/easystl.stl --world-name RMUC2026 \
    --spawn 10.925 2.525 --map-grid -15.0 -6.9 600 341

# 换一份 STL：出生点自动选（严格平台面上离障碍最远），其余全默认
python3 tools/scripts/world/stl_to_world.py --stl /path/new_field.stl --world-name RMUL2027

# 试跑/回归：写到临时树，完全不碰仓库里的资产
python3 tools/scripts/world/stl_to_world.py --stl ... --world-name RMUC2026_TEST --out-root /tmp/world_test
```

产出四样 + 两样附带：`meshes/<NAME>_world/`（`meshes/<NAME>.stl` 逐字节拷贝 + `model.sdf` +
`model.config`，这是 Gazebo 的 `model://` 解析根）、`world/<NAME>_world/`（`<NAME>_world.world`
+ 上面两份 + `meshes/`，镜像既有目录结构）、`map/<NAME>.pgm|yaml`、`PCD/<NAME>.pcd`，
外加**打印摘要**（bbox / 地面高度 / 网格 / free-occupied 面积 / 出生点 + clearance / 点数）与
**JSON manifest**（默认 `<out-root>/.tmp_cache/world/<NAME>.manifest.json`，含 STL sha256 与全部参数）。
**launch 文件不自动改**：摘要里会打印该加的那两行（格式同 §3），照抄即可。
`--help` 有全部开关；`--robot-radius / --clearance-margin / --band / --voxel-* / --map-pad /
--seed` 等都有与 RMUC2026 一致的默认值。

**回归证据**（`--world-name RMUC2026_TEST` 写进临时目录，比对完即删、**未提交**）：

| 检查项 | 工具输出 | 已提交资产 | 结论 |
|---|---|---|---|
| STL 拷贝 sha256 | `cc723f31…` | `cc723f31…` | 逐字节相同 |
| 场地 bbox | x[-14.575,14.575] y[-6.401,9.649] z[-1.841,-0.111] | 同 | 相同 |
| 自动求出的地面 | mesh z = **-1.6413436**（202.9 m² 的最大水平面） | 抬升 1.6413436 | 相同 |
| `map/<NAME>.pgm` | 600x341 @0.05，free **237.8** / occupied **229.4** / 外沿 **44.4** m² | 同 | **逐字节相同** |
| `.yaml` origin | `[-25.925, -9.425, 0]` | 同 | 除 `image:` 名外相同 |
| 出生点 | (10.925, 2.525) clearance **2.704 m** | 同（§3 表） | 相同 |
| PCD 点数 | **26,582**（0.25 m→11,146 ＋ 0.10 m→15,436） | **26,618**（11,124 ＋ 15,494） | −36 点（−0.14%），见差异 ① |
| PCD bbox（map 系） | x[-25.500,3.650] y[-8.926,7.124] z[-0.140,0.540] | 同 | 相同 |
| world / model.sdf | 去注释后 251 / 67 行 | 同 | 只有 `<model name>` 与 mesh URI（含世界名）不同 |

两处**已知的、刻意的**差异：

① **PCD 点数 ±0.1%**：已提交那份是 `trimesh` 默认（无种子）随机采样出来的；工具默认固定
`--seed 0` 以便逐次可复跑，于是"带内点数"（2,037,263 vs 2,036,997）与体素点数差几十个
（体素下采样本身确定，变的是哪些体素被采到）。bbox、两档 leaf、密度口径都一致。
② **地图网格默认值**：工具默认取"场地 bbox 外扩 `--map-pad 0.5 m` 再向外对齐整格" ⇒ **604x342**；
已提交那张是当初**手工取的整数边界 600x341**（30.00 x 17.05 m，x 比 bbox+0.5 少 0.075 m）。
两者的 free/occupied 面积完全一样（多出来的只是外沿一圈"占用"）；要逐字节复现旧图就显式给
`--map-grid -15.0 -6.9 600 341`（上面第一条命令已经这么写了）。

**另外：`.tmp_cache/rmuc2026/` 里现存的脚本与本次工具不一致，以工具为准**（资产两处都对得上）：

* `gen_assets.py` 把地图网格、出生点、地面高度（`-1.641343627929688`）全写成常量；工具全部从
  STL 自己算（地面 = 面积最大的水平面，本 STL 逐位等于那个常量）。
* `build_assets.py` 的**末尾版本**把出生点距离变换做在 `free`（阈值 0.15 m）上，会给出
  **(7.975, 2.025) / 3.200 m** —— 那**不是**已提交的出生点；能复现 §3 表与 launch 注释里那组数
  （对称两区各 ~71.4 m²、+x 点 2.704 m / −x 点 2.706 m）的是**严格平台面（抬升 ≤ 0.05 m）**版本，
  工具默认用的就是后者（`--plateau-threshold 0.05`）。即：**脚本漂移过，资产没漂**。

---

## 5. 无头实测（headless）

环境（与 `docs/smoke_test_runbook.md` / `.tmp_cache/five_way/run_one.sh` 同一套隔离）：
`HOME=/tmp/gzhome-*`（沙箱里 $HOME 只读，gzserver 建 `~/.gazebo` 会 SIGABRT）、
非默认 `ROS_DOMAIN_ID`、`GAZEBO_MASTER_URI=http://127.0.0.1:1140x`、`unset DISPLAY`、`nav_rviz:=False`、
**所有步骤在同一个 bash 调用内**（PID namespace 一结束进程就被清掉）。日志在 `.tmp_cache/rmuc2026/`。

### 5.1 `mode:=mapping lio:=fastlio mapper:=slam_toolbox`（建图）

| 检查项 | 结果 |
|---|---|
| Gazebo 加载世界 | ✅ 无 mesh/URI 报错；`gzserver` 正常起 |
| 机器人出生在地面上 | ✅ `/odom_ground_truth` = `(10.9249, 2.5215, 0.0600)`；整程 3 组 `tf2_echo odom base_link` z 稳定在 **-0.243**（不沉不飘） |
| 话题 | ✅ `/livox/lidar`、`/livox/lidar/pointcloud`、`/livox/imu`、`/scan`、`/odom`、`/odom_ground_truth`、`/map` 全部出现 |
| 频率（RTF≈0.5 时） | pointcloud 7.8~8.0 Hz、imu 77~79 Hz、scan 7.6~8.0 Hz、odom 7.7~7.9 Hz（标称 10/100，按实时率折算正常） |
| `/map` 会随运动增长 | ✅ 驱动前 `known 20.8 m²` → 驱动后 `58.5 m²`（另一次跑到 74.2 m²） |
| 驱动方式 | ⚠️ 见下 |
| LIO 是否发散 | ✅ **无发散**：LIO 估计位移 / Gazebo 真值位移 = **2.623 / 2.619 m = 1.002**（`/odom` 话题口径 2.510/2.619 = 0.958），逐帧曲线全程贴合（详见下面的"测量陷阱"） |
| 真实建图产物 vs STL 几何 | ✅ 把 `/map_save` 存下的 22,947 点真实点云平移到 world 系后：bbox `x[-14.64,14.45] y[-6.19,9.56]`（STL 是 `x[-14.575,14.575] y[-6.401,9.649]`）；0.05 m 栅格上 **87.5% 的"竖直结构"格落在 STL 障碍格 5 cm 之内**（mean chamfer 0.019 m）⇒ **毫米缩放、z 抬升、模型 pose 三件事都被实测证明是对的** |

**驱动方式（说清楚）**：`/cmd_vel_chassis`（Gazebo 平面底盘插件的执行话题）直接发 `Twist`，
`linear.x=0.4, angular.z=0`，持续 6 s ⇒ 真值前进 2.42~2.64 m（与 0.4 m/s 相符）。
**为什么不走 nav 侧**：`mode:=mapping` 不起 nav2，`/cmd_vel_nav` 没有发布者；
`/cmd_vel → /cmd_vel_chassis` 那条 fake_vel_transform 通路在 `spin_speed:=0.0` 下也可用，
但直接发执行话题最省事、且能排除中间环节。**注意插件没有命令超时**：
发布停止后车会**保持最后速度继续走**（实测停发后仍在缓慢前进），测完必须显式发零速或直接收栈。

**⚠️ 一个测量陷阱（写下来省得别人再踩）**：用 `rclpy.spin_once()` 每 0.2 s 采一次
`lookup_transform('odom','base_link')` 时，`/tf` 有 ~80 msg/s、采样循环只 ~5 Hz ⇒
tf2 Buffer 永远在处理积压 ⇒ "latest" 拿到的是十几秒前的旧值，看起来就像
**"LIO 滞后 13 s / 估计位移是真值的 2.6 倍"**。换成 `MultiThreadedExecutor`（或 `tf2_echo` CLI）
后同一场景立刻变成 **1.002**。**LIO 没问题，是测量工具的锅**。

### 5.2 `mode:=nav localization:=amcl nav:=rpp planner:=navfn`（2D 图 + AMCL）

命令见 §7。结果（`nav_smoke_regression.py --goal -6.0 3.5 --localization RMUC2026-amcl`）：

```
P0 回归：✅ PASS
  · 地图 0.050 m/格 600x341 origin=(-25.93, -9.43)；目标落格 (398, 258) = free
  · 结果 status=SUCCEEDED 用时=91.4s d_min=0.0m recoveries=6
  · 命令链 max|v|,|w|: nav=(0.50,0.75) smooth=(0.50,3.00) chassis=(0.50,3.00)
  · 真值: twist_max=3.000 pose_delta=8.715m
  · RTF=0.49 hz={pcloud 8.0, imu 79.3, scan 8.0, odom 7.7} tf_age=0.1
  · "Control loop missed its desired rate" 出现 1 次
```

⇒ **加载地图 ✅ / AMCL 定位稳定 ✅（map→odom 3 s 窗口漂移 0.0000 m）/ 规划-控制-底盘链路 ✅ / 到点 ✅**。

⚠️ **过程不干净（要如实说）**：91.4 s 里 `controller_server` 报了 **4 次 `Failed to make progress`**，
`behavior_server` 的一次 `Spin` 恢复还**超时失败**（`Exceeded time allowance before reaching the Spin goal
- Exiting Spin`），合计 `recoveries=6`；`pose_delta 8.715 m`（直线只有 7.45 m）说明它绕了路。
即：**"能到点"成立，"走得顺"不成立** —— 与 §6 的"台阶当墙 + 通道窄"完全一致。

### 5.3 `mode:=nav localization:=gicp`（PCD + 3D 重定位）

```
P0 回归：✅ PASS
  · 结果 status=SUCCEEDED 用时=94.1s d_min=0.0m recoveries=6
  · 命令链 max|v|,|w|: nav=(0.50,0.75) smooth=(0.50,3.00) chassis=(0.50,3.00)
  · 真值: twist_max=3.000 pose_delta=9.155m
  · RTF=0.78 hz={pcloud 8.0, imu 78.0, scan 8.0, odom 8.0}
  · ⚠️ 稳定性：settled=False 等待=30.0s 漂移 xy=0.053 m yaw=0.0129 rad（阈值 0.02 m / 0.01 rad）
```

GICP 侧健康指标（`nav_gicp.log` 的 `[status]` 行）：

```
地图点数（GICP target）：原始 26618 → 去 NaN 26618 → 体素 0.100 m 后 21161
地图包围盒：min=(-25.50,-8.93,-0.14) max=(3.65,7.12,0.54)
[status] 采纳 1082/1082 帧 | 最近 score=0.01788 m² | align=21.1 ms | source=3143 点 → target=21161 点
         map→odom x=-0.119 y=-0.029 z=0.225 yaw=-0.65°
```

⇒ **PCD 被正确加载 ✅ / 100% 帧被采纳 ✅ / fitness 0.018 m²（warn 阈值 0.05）✅ / 单帧 align 21 ms ✅（比 RMUL2026 的 327~389 ms 还快，完全不构成 CPU 压力）/ 到点 ✅**。
⇒ 与 §4.2 的设计一致：**`initial_pose: [0,0,0]` 直接就是对的**（收敛后的 `map→odom` 只有 -0.12 m / -0.65° 的修正量），
说明"map 系 = 出生点系 + 地面放 z=-0.06"这套约定是自洽的。
⚠️ 唯一打折项：`map→odom` 在 30 s 窗口里仍在缓慢收敛（0.053 m / 0.0129 rad，约 1.8 mm/s），
没达到回归脚本的 `settle` 判据 ⇒ **不要把这次结果当作"定位收敛后"的干净读数**；
按 GICP"每帧都在精配准"的性质，这是持续微调而不是发散（100% 帧采纳 + score 稳定在 0.018）。
`recoveries=6`、4 次 `Failed to make progress` 与 AMCL 那次**完全同型**。

---

## 6. 坑：这张场地对我们这台车意味着什么（都是量出来的）

| 事实 | 数值 | 对 0.22 m 半径车（直径 0.44 m）的含义 |
|---|---|---|
| 场地可行驶**地板**（0.02 m 栅格，严格平面） | **214.6 m²** | 地面本身 |
| 地图 free（阈值 0.15 m，含台阶根部过渡带） | 237.8 m² | 规划器看到的"白区" |
| clearance ≥ 0.22 m 的集合 | 178.6 m²（同一连通域 176.3 m²） | **真正能走的 ≈ 176 m²（占 free 的 74%）** |
| clearance ≥ 0.30 m | 162.8 m²，但**裂成两块 80.6 / 80.5 m²** | 0.30 m 余量下两个半场不再连通 |
| 两个半场之间最窄处 | **通道宽 0.45 m**（world (±10.58,+9.33) 与 (-14.18,+9.22) 一带）；0.50 m 处在 (-14.22,+5.5) | 余量 5 mm/边 ⇒ **实际过不去**；A* 给出的过场路线长 **41.7 m**（绕场边） |
| 台阶高度 | **0.20 m 与 0.30 m**（另有 1 mm 级装饰面），高度突变 60~90° | 轮半径 0.06 m ⇒ **物理上上不去**；因此 2D 图把它们画成障碍、PCD 里保留（LiDAR 确实能看到） |
| 隧道 / 设计坡道 | **没有**（`.tmp_cache/stl_view/` 的 ray-parity 结论：≥0.25 m 净空的 subsurface 空洞只有 3.7 m²、全是 ≤0.15 m 宽的缝） | 不要指望"钻洞"或"爬坡"路线 |
| 0.15 m 高处的自由空间 / 走廊中位宽 | 240.9 m² / **0.25 m**；28 处 < 0.70 m 的卡点 | 这些卡点大多在两个角部结构内部 ⇒ 里侧基本是死区 |

**结论**：场地能用（地图正确、能定位、能规划到点），但**它是一场"全场级"障碍赛**，
不是我们平时跑的 12 x 8 m 半场。想在这个 world 里做有意义的导航考核，
目标点应当选在**出生点所在的半场**（约 176 m² 可达），跨半场的目标请当作"已知不可达"。

---

## 7. 怎么自己跑（复制即可）

```bash
cd /home/weicheng/HzMi_rmsimulation
source /opt/ros/humble/setup.bash && source install/setup.bash

# 0) 资产是新加的 ⇒ 先 build（install/ 是逐文件符号链接，不 build 看不到新文件）
colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup
source install/setup.bash

# 1) 建图（Gazebo + FAST-LIO + slam_toolbox；带 RViz 看）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=mapping lio:=fastlio mapper:=slam_toolbox nav_rviz:=False lio_rviz:=True

# 2) 导航 + AMCL（2D 栅格图路线，实测 PASS）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=fastlio localization:=amcl nav:=rpp planner:=navfn spin_speed:=0.0

# 3) 导航 + GICP（3D 先验点云路线）
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUC2026 mode:=nav lio:=fastlio localization:=gicp nav:=rpp planner:=navfn spin_speed:=0.0

# 4) 无头跑一次 P0 回归（另开一个终端；栈已在跑）
python3 tools/scripts/regress/nav_smoke_regression.py --goal -6.0 3.5 --localization RMUC2026-amcl
```

`--show-args` 里 `world` 是自由字符串，新值不需要额外声明：

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py --show-args | grep -A2 "'world'"
```

**回滚**：本 world 的全部改动都是**新增文件 + 两处 launch 加行**，没有改任何既有资产：

```bash
git rm -r src/rm_simulation/hzmi_rm_simulation/world/RMUC2026_world \
           src/rm_simulation/hzmi_rm_simulation/meshes/RMUC2026_world \
           src/rm_nav_bringup/map/RMUC2026.pgm src/rm_nav_bringup/map/RMUC2026.yaml \
           src/rm_nav_bringup/PCD/RMUC2026.pcd docs/worlds.md
git checkout -- src/rm_simulation/hzmi_rm_simulation/launch/rm_simulation.launch.py \
                src/rm_nav_bringup/launch/bringup_sim.launch.py
colcon build --symlink-install --packages-select hzmi_rm_simulation rm_nav_bringup
```
（等价于 `git revert <本次 commit>`；`world:=RMUC2026` 消失后其余 world 完全不受影响。）

---

## 8. 未验证 / 已知未做

* `localization:=beluga / small_gicp / icp / slam_toolbox / cartographer` 在 RMUC2026 下**都没跑**：
  `beluga` 缺 `.pgm` 之外的 apt 包依赖验证、`slam_toolbox`/`cartographer` 需要 `.posegraph`/`.pbstream`
  （本 world 没有建），`icp`/`small_gicp` 与 `gicp` 共用同一份 PCD（大概率同上，但**未实测**）。
* `mode:=slam_nav`、`nav:=dwb/teb/mppi`、`planner:=smac2d`、`global_obstacle/local_obstacle` 各槽位
  在 RMUC2026 下**未做 A/B**。
* 跨半场（41.7 m 绕行）的目标**没有实测**，只有几何上的"最窄 0.45 m"证据。
* 场地里的 **0.20/0.30 m 台阶能否被更小的车（半径 < 0.2 m）通过**、以及"能不能爬上去"，
  没有做参数化实测。
* STL 的**非水密/重复面**对 ODE 的长期影响（长时间接触、多个接触点）没有做压力测试，
  只跑了 3 次短程（< 3 min/次）。
* 世界文件里 5 盏灯的**实际照度**没有校验（只保证与 RMUL2026 同参数、位置平移）。
* `tools/scripts/world/stl_to_world.py` 的产物只做了**文件级回归**（pgm 逐字节、world/model 去注释逐行、
  PCD 点数与 bbox，见 §4.4），**没有**再起一次 Gazebo 验证 `--world-name RMUC2026_TEST` 能 spawn；
  `RMUC2026` 本身的无头实测见 §5。
