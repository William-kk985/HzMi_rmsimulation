# HzMi_rmsimulation 功能包 / 算法包规划

> 项目已由 `pb_rmsimulation` 更名为 `HzMi_rmsimulation`（文件夹/git 仓库名）。
> ROS2 包名亦已由 `pb_rm_simulation` 更名为 `hzmi_rm_simulation`（遵循 ROS 包名仅小写+下划线的命名规范）。

## 一、目标

构建一个「建图 / 重定位 / 规划导航」×「2D / 3D」的多组合算法实验平台，用**宏编译架构**实现：
- 一套代码 → 多套编译产物（组合矩阵）
- 编译期零开销切换算法变体，未选中的不参与编译（错误隔离）
- 可复现、可对比的算法实验

## 二、核心原则

| 层 | 现状 | 结论 |
|---|---|---|
| 建图 / 里程计 / 重定位 | 基本全是现成打包好的开源方案（LIO、Cartographer、AMCL、ICP…） | **复用，不重写** |
| rm_algolab | 只做「选型 + 编排 + 调参 + 对比」 | 薄适配层（每个算法 ~50 行） |

真正自研的只有小块胶水：`fake_vel_transform`（速度补偿）、`icp_registration`（配准节点，基于开源库）、将来的适配器。

## 三、宏体系（config.hpp 四层）

```
优先级从高到低：

① 重定位 LOCAL_*        ← 主调试对象，必选且互斥
     LOCAL_2D_AMCL
     LOCAL_2D_SLAM_TOOLBOX
     LOCAL_3D_ICP
     (LOCAL_3D_NDT 预留)

② 建图   MAPPING_*      ← 高优先级（跑新场地时切）
     MAPPING_2D_CARTO / MAPPING_3D_LIO

③ 规划导航 NAV_*        ← 中优先级（对比 TEB/DWB）
     NAV_2D_TEB / NAV_2D_DWA

④ 里程计 ODOM_*         ← 低频开关，默认 LIO
     ODOM_FAST_LIO (默认) / ODOM_POINT_LIO / ODOM_WHEEL(预留)
```

互斥校验统一用 `#if defined(A)+defined(B) > 1 → #error`。
调试宏 `DEBUG_PLOT / DEBUG_DUMP / DEBUG_BENCH_LOG` 发布前全注释。

## 四、功能包 / 算法包清单

### A. 新建包

| 包 | 定位 | 状态 |
|---|---|---|
| `rm_algolab` | ⭐ 宏架构实验台（核心）| M1 待建 |
| `rm_bench`（可选）| 组合对比/评估工具 | M3 |

### B. 改造现有自研包（引入宏体系）

| 包 | 改造 |
|---|---|
| `fake_vel_transform` | 加 config.hpp：`PLATFORM_SIM / PLATFORM_REAL` |
| `icp_registration` | 加 config.hpp：算法变体宏（作 LOCAL_3D_ICP 后端）|

### C. 复用不动（算法本体现成）

`FAST_LIO`、`Point-LIO`、`Cartographer`、`Nav2`(Navfn+DWB)、`TEB`、`AMCL`、`slam_toolbox`

### D. 远期预留

- 3D 导航（`NAV_3D` 宏预留）
- LVI 融合（R3LIVE 等）
- 点云降维统一封装

## 五、现有算法盘点

### 建图
| 算法 | 类型 | 输出 |
|---|---|---|
| Cartographer | 2D SLAM | .pbstream / 栅格图 |
| slam_toolbox | 2D SLAM | .pgm + .posegraph |
| FAST-LIO | 3D LIO | 点云 .pcd |
| Point-LIO | 3D LIO | 点云 .pcd |

### 定位 / 重定位
| 算法 | 类型 | 依赖 |
|---|---|---|
| FAST-LIO / Point-LIO 里程计 | 持续定位 | 无 |
| AMCL | 2D 粒子滤波 | 栅格图 |
| slam_toolbox(localization) | 2D | .posegraph |
| icp_registration | 3D 配准 | .pcd |

### 规划导航
| 算法 | 层级 | 类型 |
|---|---|---|
| Nav2 NavfnPlanner | 全局 | A* 类 |
| Nav2 Controller(FollowPath) | 局部 | DWB 类 |
| TEB Local Planner | 局部 | 时间弹性带 |

### 感知支撑
pointcloud_to_laserscan（3D→2D 降维）、linefit_ground_segmentation（地面分割）、imu_complementary_filter（IMU 滤波）、fake_vel_transform（速度补偿）

## 六、里程计选型

| 优先级 | 算法 | 频率 | 场景 |
|---|---|---|---|
| ⭐ 主力 | FAST-LIO | ~10Hz | 默认 |
| ⭐ 备选 | Point-LIO | 100Hz+ | 高帧率/平滑控制 |
| 🧪 备份 | 轮式里程计 | 高频 | 激光退化兜底 |
| 🧪 实验 | 帧间 ICP | 10~20Hz | 无 IMU 对照 |

## 七、落地路线

```
M1  骨架：建 rm_algolab + config.hpp 四层宏 + CMake option 矩阵 + 空适配器 + launch 模板
M2  两条代表管线：
    组合1 纯2D   = CARTO + AMCL + TEB
    组合2 3D+2D  = LIO + AMCL + TEB（最实用，MID360 降维）
M3  扩展矩阵：slam_toolbox / ICP / DWA + bench.sh 对比工具
```

## 八、待用户确认

- [ ] rm_algolab 包名确认
- [ ] M2 优先组合（建议：LIO + AMCL + TEB）
- [ ] 是否开始搭建 M1 骨架
- [ ] 远程 gitee 仓库是否同步改名
