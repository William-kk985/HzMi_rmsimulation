#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stl_to_world.py —— 一条命令：场地 STL ⇒ Gazebo world + 2D 栅格图 + 先验 PCD

背景
    `RMUC2026` 这套世界资产最初是在 `.tmp_cache/rmuc2026/` 里用几个一次性脚本拼出来的
    （`build_assets.py` 出生点 + `gen_assets.py` 地图/PCD，world/model 是手改 RMUL2026 的）。
    本工具把它们整理成**可复跑的一条命令**，约定与 RMUC2026 已实测通过的那套逐项一致
    （见 `docs/worlds.md` §2~§4）；给另一份 STL 时不需要再改任何一行代码。

产出（相对 `--out-root`，默认 = 本仓库根；`--out-root /tmp/x` 可写到临时目录做回归）
    1. `src/rm_simulation/hzmi_rm_simulation/meshes/<NAME>_world/`
           `meshes/<NAME>.stl`（输入的逐字节拷贝）+ `model.sdf` + `model.config`
           ← 这是 Gazebo 真正解析的那棵树：`GAZEBO_MODEL_PATH=<share>/hzmi_rm_simulation/meshes`
    2. `src/rm_simulation/hzmi_rm_simulation/world/<NAME>_world/`
           `<NAME>_world.world`（内联模型 + 5 盏灯 + state + gui）+ `model.sdf` + `model.config`
           + `meshes/<NAME>.stl`（镜像既有目录结构；`world` 里内联的那份才是生效的）
    3. `src/rm_nav_bringup/map/<NAME>.pgm` + `.yaml`（nav2/AMCL 的 2D 栅格图）
    4. `src/rm_nav_bringup/PCD/<NAME>.pcd`（GICP/ICP 的先验点云）
    5. 打印摘要（bbox / 出生点 + clearance / free-occupied 面积 / 点数）+ JSON manifest

几何/坐标约定（**硬编码的只有约定本身，没有任何 world 专属常量**）
    * 输入 STL 按 `--scale 0.001` 从毫米变米，Z 轴朝上、无旋转；模型 pose 保持 0
      ⇒ **mesh 米制坐标 == world 坐标**（map/PCD/出生点全用同一个系，少一层换算）。
    * 可行驶地面（底板顶面）的高度**从 STL 自己算**：0.02 m 顶视 max-z 栅格上面积最大的
      那个水平面（本 STL = mesh z `-1.6413436279296876` m）；visual/collision 抬 `-z_floor`
      ⇒ 地面落在 **world z = 0**（不抬的话机器人会在 z=0 出生、地面在 -1.64 ⇒ 悬空坠落）。
      算错时用 `--floor-z` 手工兜底。
    * 2D 栅格图：细栅格按格取**最高**面高度（保守，保住薄墙）→ 聚合到 `--resolution`；
      相对地面抬升 `> --floor-threshold`（默认 0.15 m）的格 = 占用，≤ = 自由，
      场地外沿（栅格无数据）**也算占用**（当围墙，规划器不会往空处走）。
    * 出生点：⚠️ 距离变换做在**严格平台面**上（相对地面抬升 ≤ `--plateau-threshold`，默认 0.05 m，
      即"真平的地板"，与占用阈值 0.15 m 是两回事）——台阶根部/坡道的过渡带不参与，
      否则出生点会被推到半场中央。要求 `clearance ≥ robot_radius + margin`
      （默认 0.22 + 0.08 = 0.30 m），取**最大连通域里离障碍最远**的点（`--auto-spawn`），
      或用 `--spawn X Y` 指定。180° 对称的场地会得到一对镜像解，本工具取数值最大者
      （RMUC2026 落在 −x 半场；已提交的 +x 出生点是对称位，用 `--spawn` 指定）。
    * **map 系 = 出生点相对系**：`yaml origin = 场地网格左下角 − 出生点` ⇒ 出生点在 map 里就是
      `(0,0)`，`amcl_init_x/y` 与 GICP/ICP 的 `initial_pose [0,0,0]` 都不用改。
    * PCD：STL 表面均匀采样 → 取高度带 `[地面-0.08, 地面+0.60] m` 的点 → **按法向分两档体素下采样**
      （近水平 `|nz|>0.7`：0.25 m，只提供 z 约束；近竖直：0.10 m，提供 x/y 约束）→ 转到 map 系，
      其中**地面放在 `z = -0.06`**（车静止时 base_link 离地 0.06 m ⇒ `initial_pose [0,0,0]` 精确正确）。

用法
    # 复现已提交的 RMUC2026 资产（TM 见 docs/worlds.md §4.4；--map-grid 是当初手工取的整数边界）
    python3 tools/scripts/world/stl_to_world.py \
        --stl .tmp_cache/stl_view/easystl.stl --world-name RMUC2026 \
        --spawn 10.925 2.525 --map-grid -15.0 -6.9 600 341

    # 换一份 STL：出生点自动选，其余默认
    python3 tools/scripts/world/stl_to_world.py --stl /path/new_field.stl --world-name RMUL2027

    # 回归/试跑：写到临时树，不碰仓库里的资产
    python3 tools/scripts/world/stl_to_world.py --stl ... --world-name RMUC2026_TEST \
        --out-root /tmp/world_test

依赖：`numpy` + `trimesh`（`--auto-spawn` 还需要 `scipy`）。RMUC2026 那次的完整推导、
实测结论与坑见 `docs/worlds.md`。
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# ------------------------------------------------------------------ 默认约定（与 RMUC2026 一致）
SCALE_DEFAULT = 0.001            # STL 原生单位 = 毫米
RASTER_RES_DEFAULT = 0.02        # 细栅格：顶视 max-z（保住薄墙）
MAP_RES_DEFAULT = 0.05           # nav2 栅格图分辨率（本仓库既有图统一 0.05）
FLOOR_THRESHOLD_DEFAULT = 0.15   # 相对地面抬升 > 此值 = 障碍（0.20/0.30 m 台阶、塔、柱、墙）
PLATEAU_THRESHOLD_DEFAULT = 0.05  # "严格平台面"：出生点距离变换只在这个面上做（排除台阶根部过渡带）
ROBOT_RADIUS_DEFAULT = 0.22      # 本车半径（RMUL2026/RMUC2026 同款）
CLEARANCE_MARGIN_DEFAULT = 0.08  # 出生点余量 ⇒ 要求 clearance ≥ 0.30 m
MAP_PAD_DEFAULT = 0.5            # 场地 bbox 外扩（米），再向外对齐整格
BAND_DEFAULT = (-0.08, 0.60)     # PCD 高度带（相对地面；镜像 RMUL2026.pcd 的 0.665 m 跨度）
VOXEL_H_DEFAULT = 0.25           # 近水平面体素（只提供 z 约束，稀疏即可）
VOXEL_V_DEFAULT = 0.10           # 近竖直面体素（提供 x/y 约束，要密）
NORMAL_Z_DEFAULT = 0.7           # |nz| > 此值算"近水平"
BASE_HEIGHT_DEFAULT = 0.06       # 车静止时 base_link 离地高度 ⇒ PCD 地面 z = -0.06
SPAWN_Z_DEFAULT = 0.2            # launch 里的出生 z（与 RMUL2026 同口径）
POINTS_DEFAULT = 3000000         # PCD 表面采样点数（按面积均匀）
SEED_DEFAULT = 0                 # 采样随机种子（固定 ⇒ 逐次可复跑）
FLOOR_TOL = 1e-6                 # 水平面聚类容差（米）：重合薄片/插值误差会差最后几位
MAX_CONTACTS_DEFAULT = 10

# 世界级设置（physics/scene/audio/wind/spherical_coordinates）与灯光模板**全部照抄**
# `world/RMUL2026_world/RMUL2026_world.world`：这些与具体场地无关，只有灯位要平移到新场地中心。
# 灯位取那份 world 的 `<state>` 段（实际运行值；定义处写 z=1，运行时被 state 覆盖）。
RMUL2026_FIELD_CENTER = (8.5, 4.5)      # RMUL2026 world 里场地中心的 pose
LIGHT_TEMPLATE = [
    # (name, type, pose(x, y, z), cast_shadows, direction)
    ("user_directional_light_0", "directional", (7.32231, 3.81549, 13.0), 1, "0.1 0.1 -0.9"),
    ("user_point_light_0", "point", (7.45, 3.9, 14.0), 0, "0 0 -1"),
    ("user_spot_light_0", "spot", (2.78166, 1.19644, 7.0), 0, "0 0 -1"),
    ("user_spot_light_1", "spot", (11.9339, 6.3566, 7.0), 0, "0 0 -1"),
    ("user_spot_light_2", "spot", (7.45, 3.9, 5.0), 0, "0 0 -1"),
]
CAMERA_RPY = "-2.7e-05 1.0 2.2"          # RMUL2026/RMUC2026 同款朝向
AUTHOR = ("Lihan Chen", "lihanchen2004@163.com")


def die(msg):
    sys.exit("[stl_to_world] 错误: %s" % msg)


# ------------------------------------------------------------------ STL → 高度栅格 → 地面高度
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def rasterize_max(V, F, res):
    """顶视 max-z 栅格化（与 `.tmp_cache/stl_view/stl_lib.py::rasterize` 同算法）。

    每个三角形按其 XY 包围盒取格**心**，做重心坐标插值后取最大值 ⇒ 每格 = 该列最高面高度
    （保守，细柱子/薄墙不会被漏掉）。格心 = `lo + (i+0.5)*res`，`lo` = 米制 bbox 下角。
    返回 (Z, x0, y0)：Z 形状 (ny, nx)，NaN = 该列没有任何面。
    """
    lo = V.min(axis=0)
    hi = V.max(axis=0)
    nx = int(np.ceil((hi[0] - lo[0]) / res))
    ny = int(np.ceil((hi[1] - lo[1]) / res))
    Z = np.full((ny, nx), -np.inf)
    T = V[F]
    tx0, tx1 = T[:, :, 0].min(axis=1), T[:, :, 0].max(axis=1)
    ty0, ty1 = T[:, :, 1].min(axis=1), T[:, :, 1].max(axis=1)
    t0 = time.time()
    for i in range(len(T)):
        if i and i % 20000 == 0:
            print("  ... 栅格化 %d/%d 三角形 (%.0fs)" % (i, len(T), time.time() - t0), flush=True)
        p = T[i]
        i0 = max(int(np.floor((tx0[i] - lo[0]) / res)), 0)
        i1 = min(int(np.floor((tx1[i] - lo[0]) / res)), nx - 1)
        j0 = max(int(np.floor((ty0[i] - lo[1]) / res)), 0)
        j1 = min(int(np.floor((ty1[i] - lo[1]) / res)), ny - 1)
        if i1 < i0 or j1 < j0:
            continue
        xs = lo[0] + (np.arange(i0, i1 + 1) + 0.5) * res
        ys = lo[1] + (np.arange(j0, j1 + 1) + 0.5) * res
        gx, gy = np.meshgrid(xs, ys)
        x0, y0 = p[0, 0], p[0, 1]
        x1, y1 = p[1, 0], p[1, 1]
        x2, y2 = p[2, 0], p[2, 1]
        den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(den) < 1e-15:                      # 竖直面：投影成线段，不贡献高度
            continue
        a = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / den
        b = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / den
        c = 1.0 - a - b
        inside = (a >= -1e-9) & (b >= -1e-9) & (c >= -1e-9)
        if not inside.any():
            continue
        zz = a * p[0, 2] + b * p[1, 2] + c * p[2, 2]
        sub = Z[j0:j1 + 1, i0:i1 + 1]
        np.maximum(sub, np.where(inside, zz, -np.inf), out=sub)
        Z[j0:j1 + 1, i0:i1 + 1] = sub
    Z[np.isinf(Z)] = np.nan
    return Z, float(lo[0]), float(lo[1])


def find_floor(Z, res, tol=FLOOR_TOL, topk=6):
    """从细栅格找出**可行驶地面**的高度：面积最大的那个水平面。

    同一块平面会给出最后几位不同的浮点值（插值误差）⇒ 先按 `tol` 聚类再比面积。
    返回 (floor_z, plane_area_m2, levels)：levels = [(z, area_m2, n_exact), ...] 面积降序。
    """
    vals = Z[np.isfinite(Z)]
    uniq, cnt = np.unique(vals, return_counts=True)
    cell = res * res
    clusters = []                                  # [z, cells, n_exact]
    for i in np.argsort(-cnt):                     # 从面积大的开始，保证代表值是众数
        z = float(uniq[i])
        for c in clusters:
            if abs(c[0] - z) <= tol:
                c[1] += int(cnt[i])
                break
        else:
            clusters.append([z, int(cnt[i]), 1])
    clusters.sort(key=lambda c: -c[1])
    levels = [(c[0], c[1] * cell, c[2]) for c in clusters]
    return levels[0][0], levels[0][1], levels[:topk]


# ------------------------------------------------------------------ 0.05 m 占用栅格 + 出生点
def aggregate_grid(Z, x0, y0, res_raster, xg0, yg0, nx, ny, res):
    """细栅格 → 地图栅格：每格取**最高**值（与 RMUC2026 的 build_assets.py 同一做法）。"""
    xe = x0 + np.arange(Z.shape[1] + 1) * res_raster
    ye = y0 + np.arange(Z.shape[0] + 1) * res_raster
    cx = np.floor((xe[:-1] + res_raster / 2 - xg0) / res).astype(int)
    cy = np.floor((ye[:-1] + res_raster / 2 - yg0) / res).astype(int)
    okx = (cx >= 0) & (cx < nx)
    oky = (cy >= 0) & (cy < ny)
    flat = np.full(ny * nx, -np.inf)
    sub = Z[np.ix_(oky, okx)]
    idx = (cy[oky][:, None] * nx + cx[okx][None, :]).ravel()
    vals = np.where(np.isnan(sub), -np.inf, sub).ravel()
    np.maximum.at(flat, idx, vals)
    flat[~np.isfinite(flat)] = np.nan
    return flat.reshape(ny, nx)


def grid_extent(lo, hi, res, pad, explicit=None):
    """地图网格范围：默认 = 场地 bbox 外扩 `pad` 米后**向外对齐整格**；explicit 直接给定。"""
    if explicit is not None:
        xg0, yg0, nx, ny = explicit
        nx, ny = int(round(nx)), int(round(ny))
        if nx <= 0 or ny <= 0:
            die("--map-grid 的 NX/NY 必须为正")
        return float(xg0), float(yg0), nx, ny
    xg0 = np.floor((lo[0] - pad) / res) * res
    yg0 = np.floor((lo[1] - pad) / res) * res
    xg1 = np.ceil((hi[0] + pad) / res) * res
    yg1 = np.ceil((hi[1] + pad) / res) * res
    return float(xg0), float(yg0), int(round((xg1 - xg0) / res)), int(round((yg1 - yg0) / res))


def distance_to_obstacle(mask, res):
    """mask 内每格到最近非 mask 格的距离（米）。

    出生点用 `严格平台面` 作 mask（台阶根部过渡带 ⇒ 障碍，见 docs/worlds.md §3 步骤 3）。
    """
    try:
        from scipy import ndimage as ndi
    except ImportError:
        die("--auto-spawn / clearance 计算需要 scipy（pip install scipy），"
            "或用 --spawn X Y 显式给出生点")
    D = ndi.distance_transform_edt(mask) * res
    D[~mask] = 0.0
    return D, ndi


def choose_spawn(plateau, D, xg0, yg0, res, need, explicit=None):
    """返回 (sx, sy, clearance, note, components)。

    explicit 给定 ⇒ 用它（clearance 取最近格心处、在严格平台面距离场上的值）；
    否则 ⇒ 在 `clearance ≥ need` 的最大连通域里取离障碍最远的点。
    """
    nx, ny = plateau.shape[1], plateau.shape[0]
    if explicit is not None:
        sx, sy = float(explicit[0]), float(explicit[1])
        ix = int(np.floor((sx - xg0) / res))
        iy = int(np.floor((sy - yg0) / res))
        if not (0 <= ix < nx and 0 <= iy < ny):
            die("--spawn (%g, %g) 落在地图网格之外" % (sx, sy))
        clr = float(D[iy, ix])
        note = "显式指定"
        if not plateau[iy, ix]:
            note += "；⚠️ 该格不在严格平台面上（有台阶/坡道/障碍）"
        elif clr < need:
            note += "；⚠️ clearance %.3f < 要求 %.3f（车可能过不去）" % (clr, need)
        return sx, sy, clr, note, []

    _, ndi = distance_to_obstacle(plateau, res)
    reg = plateau & (D >= need)
    lab, n = ndi.label(reg, structure=np.ones((3, 3)))
    if n == 0:
        die("没有任何格的 clearance ≥ %.2f m —— 场地太窄或阈值太高" % need)
    sizes = ndi.sum(reg, lab, range(1, n + 1)) * res * res
    order = np.argsort(-sizes)
    comps = []
    for k in order[:5]:
        ys, xs = np.where(lab == k + 1)
        comps.append({"area_m2": float(sizes[k]),
                      "bbox": [float(xg0 + xs.min() * res), float(yg0 + ys.min() * res),
                               float(xg0 + (xs.max() + 1) * res), float(yg0 + (ys.max() + 1) * res)],
                      "centroid": [float(xg0 + (xs.mean() + 0.5) * res),
                                   float(yg0 + (ys.mean() + 0.5) * res)]})
    main = lab == (order[0] + 1)
    Dm = np.where(main, D, 0.0)
    iy, ix = np.unravel_index(np.argmax(Dm), Dm.shape)
    sx = float(xg0 + (ix + 0.5) * res)
    sy = float(yg0 + (iy + 0.5) * res)
    return sx, sy, float(D[iy, ix]), "自动（最大连通域内离障碍最远）", comps


# ------------------------------------------------------------------ 输出文件
def write_text(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def write_pgm(path, img):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    return path


def write_pcd(path, pts):
    """8 x float32 的 binary PCD，字段与 `PCD/RMUL2026.pcd` 同构（后 5 个字段留 0）。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rec = np.zeros((len(pts), 8), np.float32)
    rec[:, :3] = pts
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\n"
              "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
              "SIZE 4 4 4 4 4 4 4 4\n"
              "TYPE F F F F F F F F\n"
              "COUNT 1 1 1 1 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
              % (len(pts), len(pts)))
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(rec.tobytes())
    return path


def model_sdf(name, z_shift, scale, max_contacts, desc):
    """独立 model 入口（两棵树内容相同；注释里带上生成参数，便于回溯）。"""
    uri = "model://%s_world/meshes/%s.stl" % (name, name)
    return """<?xml version='1.0'?>
<sdf version='1.7'>
  <!-- {name} 场地模型（独立 model 入口，结构与 world/{name}_world/{name}_world.world 里内联的那份等价）。
       {desc}
       · mesh 单位按 scale={scale:g} 换算成米，Z 轴朝上、无旋转
       · 底板（可行驶地面）顶面在 mesh 局部 z={zfloor:.8g} m ⇒ pose 抬 +{zs:.8g} m，地面落在 world z=0
       本文件由 tools/scripts/world/stl_to_world.py 生成，勿手改。 -->
  <model name='{mname}'>
    <static>true</static>
    <allow_auto_disable>true</allow_auto_disable>

    <link name='base_link'>
      <inertial>
        <mass>1000.0</mass>
        <inertia>
          <ixx>100.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>100.0</iyy>
          <iyz>0.0</iyz>
          <izz>100.0</izz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
      </inertial>

      <pose>0 0 0 0 0 0</pose>
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <kinematic>false</kinematic>
      <enable_wind>false</enable_wind>

      <visual name='visual'>
        <pose>0 0 {zs:.8g} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{uri}</uri>
            <scale>{scale:g} {scale:g} {scale:g}</scale>
          </mesh>
        </geometry>
        <material>
          <lighting>true</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.01 0.01 0.01 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <transparency>0</transparency>
        <cast_shadows>true</cast_shadows>
      </visual>

      <collision name='collision'>
        <laser_retro>0</laser_retro>
        <max_contacts>{max_contacts}</max_contacts>
        <pose>0 0 {zs:.8g} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{uri}</uri>
            <scale>{scale:g} {scale:g} {scale:g}</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1</mu>
              <mu2>1</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
""".format(name=name, mname="%s_world" % name, zs=z_shift, zfloor=-z_shift, scale=scale,
           uri=uri, max_contacts=max_contacts, desc=desc)


def model_config(name, description):
    return """<?xml version="1.0" ?>
<model>
\t<name>{mname}</name>
\t<version>1.0</version>
\t<sdf version="1.7"></sdf>

\t<author>
\t\t<name>{an}</name>
\t\t<email>{ae}</email>
\t</author>

\t<description>{desc}</description>
</model>
""".format(mname="%s_world" % name, an=AUTHOR[0], ae=AUTHOR[1], desc=description)


def world_file(name, z_shift, scale, max_contacts, center, camera, gen_note):
    """世界文件：世界级设置/灯光模板照抄 RMUL2026_world.world，只有模型与灯位随 STL 变。"""
    shift = (center[0] - RMUL2026_FIELD_CENTER[0], center[1] - RMUL2026_FIELD_CENTER[1])

    def lpose(p):
        return " ".join("%g" % v for v in (p[0] + shift[0], p[1] + shift[1], p[2]))

    lights, state_lights = [], []
    for lname, ltype, pose, shadows, direction in LIGHT_TEMPLATE:
        if ltype == "directional":
            body = """      <direction>{d}</direction>
      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>{s}</cast_shadows>
""".format(d=direction, s=shadows)
        elif ltype == "point":
            body = """      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>{s}</cast_shadows>
      <direction>{d}</direction>
""".format(s=shadows, d=direction)
        else:  # spot
            body = """      <direction>{d}</direction>
      <attenuation>
        <range>20</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>{s}</cast_shadows>
      <spot>
        <inner_angle>0.6</inner_angle>
        <outer_angle>1</outer_angle>
        <falloff>1</falloff>
      </spot>
""".format(d=direction, s=shadows)
        lights.append("""    <light name='{n}' type='{t}'>
      <pose>{p} 0 -0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
{body}    </light>
""".format(n=lname, t=ltype, p=lpose(pose), body=body))
        state_lights.append("""      <light name='{n}'>
        <pose>{p} 0 -0 0</pose>
      </light>
""".format(n=lname, p=lpose(pose)))

    uri = "model://%s_world/meshes/%s.stl" % (name, name)
    return """<sdf version='1.7'>
  <world name='default'>
    <!-- {note} -->
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <!-- =====================================================================
         {name} 场地模型
         · mesh 单位 = 毫米（scale={scale:g}）；Z 轴朝上 ⇒ 不需要旋转
         · 底板（可行驶地面）顶面在 mesh 局部 z={zfloor:.8g} m ⇒ visual/collision pose 抬 +{zs:.8g} m，
           场地地面正好落在 world z = 0（不抬的话机器人会在 z=0 悬空坠落）
         · 模型 pose 保持 (0,0,0)：mesh 米制坐标 == world 坐标，map/PCD/出生点都按这个系写
         本文件由 tools/scripts/world/stl_to_world.py 生成，勿手改。
         ===================================================================== -->
    <model name='{name}'>
      <link name='link_3'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <pose>0 0 -3e-06 0 -0 0</pose>
        <gravity>1</gravity>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <enable_wind>0</enable_wind>
        <visual name='visual'>
          <pose>0 0 {zs:.8g} 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{uri}</uri>
              <scale>{scale:g} {scale:g} {scale:g}</scale>
            </mesh>
          </geometry>
          <material>
            <lighting>1</lighting>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <shader type='pixel'>
              <normal_map>__default__</normal_map>
            </shader>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.01 0.01 0.01 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
          <transparency>0</transparency>
          <cast_shadows>1</cast_shadows>
        </visual>
        <collision name='collision'>
          <laser_retro>0</laser_retro>
          <max_contacts>{max_contacts}</max_contacts>
          <pose>0 0 {zs:.8g} 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>{uri}</uri>
              <scale>{scale:g} {scale:g} {scale:g}</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>1</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
              <bullet>
                <split_impulse>1</split_impulse>
                <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
              </bullet>
            </contact>
          </surface>
        </collision>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose>0 0 0 0 0 0</pose>
    </model>
    <!-- 灯光：类型/颜色/衰减照抄 RMUL2026_world.world，位置整体平移到新场地中心
         ({cx:g}, {cy:g})（平移 {sx:g}, {sy:g}），高度取那份 world 的 <state> 段实际生效值 z=5~14。 -->
{lights}    <state world_name='default'>
      <sim_time>0 0</sim_time>
      <real_time>0 0</real_time>
      <wall_time>0 0</wall_time>
      <iterations>0</iterations>
      <model name='{name}'>
        <pose>0 0 0 0 0 0</pose>
        <scale>1 1 1</scale>
        <link name='link_3'>
          <pose>0 0 -3e-06 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
{state_lights}    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>{camera}</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
""".format(note=gen_note, name=name, scale=scale, zs=z_shift, zfloor=-z_shift, uri=uri,
           max_contacts=max_contacts, lights="".join(lights),
           state_lights="".join(state_lights), camera=camera,
           cx=center[0], cy=center[1], sx=shift[0], sy=shift[1])


# ------------------------------------------------------------------ 主流程
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="场地 STL ⇒ Gazebo world + 2D 栅格图 + 先验 PCD（约定见 docs/worlds.md §2~§4）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--stl", required=True, help="输入 STL（毫米单位、Z 朝上）")
    ap.add_argument("--world-name", required=True,
                    help="世界名（如 RMUC2026）：决定目录名、world/model 名、map/PCD 文件名")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--auto-spawn", action="store_true",
                   help="自动选出生点（默认行为；不写 --spawn 时生效）")
    g.add_argument("--spawn", nargs=2, type=float, metavar=("X", "Y"),
                   help="显式出生点（world/场地系米制，应落在可行驶地面上）")
    ap.add_argument("--floor-threshold", type=float, default=FLOOR_THRESHOLD_DEFAULT,
                    help="相对地面抬升超过此值 ⇒ 占用（米）")
    ap.add_argument("--plateau-threshold", type=float, default=PLATEAU_THRESHOLD_DEFAULT,
                    help="严格平台面阈值（米）：出生点的距离变换只在这个面上做，"
                         "排除台阶根部/坡道过渡带")
    ap.add_argument("--resolution", type=float, default=MAP_RES_DEFAULT,
                    help="2D 栅格图分辨率（米/格）")
    ap.add_argument("--out-root", default=REPO_ROOT,
                    help="输出根目录（默认仓库根；回归试跑可指到临时目录）")
    ap.add_argument("--manifest", default=None,
                    help="JSON manifest 路径（默认 <out-root>/.tmp_cache/world/<NAME>.manifest.json）")
    ap.add_argument("--preview", default=None, help="可选：另存一张 PNG 预览图（需 Pillow）")
    # --- 下面这些都有与 RMUC2026 一致的默认值，一般不用动
    ap.add_argument("--scale", type=float, default=SCALE_DEFAULT, help="STL 原生单位 → 米")
    ap.add_argument("--raster-res", type=float, default=RASTER_RES_DEFAULT,
                    help="细高度栅格分辨率（米）")
    ap.add_argument("--floor-z", type=float, default=None,
                    help="手工指定可行驶地面在 mesh 系的高度（米）；默认自动取面积最大的水平面")
    ap.add_argument("--robot-radius", type=float, default=ROBOT_RADIUS_DEFAULT, help="车半径（米）")
    ap.add_argument("--clearance-margin", type=float, default=CLEARANCE_MARGIN_DEFAULT,
                    help="出生点额外余量（米）")
    ap.add_argument("--map-pad", type=float, default=MAP_PAD_DEFAULT,
                    help="地图网格相对场地 bbox 的外扩（米），随后向外对齐整格")
    ap.add_argument("--map-grid", nargs=4, type=float, metavar=("X0", "Y0", "NX", "NY"),
                    help="直接给定地图网格（场地系左下角 + 格数）；默认按 --map-pad 推")
    ap.add_argument("--band", nargs=2, type=float, default=list(BAND_DEFAULT),
                    metavar=("LO", "HI"), help="PCD 高度带（相对地面，米）")
    ap.add_argument("--voxel-horizontal", type=float, default=VOXEL_H_DEFAULT,
                    help="近水平面体素 leaf（米）")
    ap.add_argument("--voxel-vertical", type=float, default=VOXEL_V_DEFAULT,
                    help="近竖直面体素 leaf（米）")
    ap.add_argument("--normal-z", type=float, default=NORMAL_Z_DEFAULT,
                    help="|nz| 大于此值算近水平面")
    ap.add_argument("--base-height", type=float, default=BASE_HEIGHT_DEFAULT,
                    help="车静止时 base_link 离地高度 ⇒ PCD 里地面 z = -此值")
    ap.add_argument("--points", type=int, default=POINTS_DEFAULT, help="PCD 表面采样点数")
    ap.add_argument("--seed", type=int, default=SEED_DEFAULT, help="采样随机种子（固定可复跑）")
    ap.add_argument("--spawn-z", type=float, default=SPAWN_Z_DEFAULT,
                    help="launch 摘要里给的出生 z（米）")
    ap.add_argument("--max-contacts", type=int, default=MAX_CONTACTS_DEFAULT)
    ap.add_argument("--camera-pose", nargs=6, type=float, default=None,
                    metavar=("X", "Y", "Z", "R", "P", "Y"),
                    help="gui 相机位姿；默认按场地中心与尺寸取整")
    a = ap.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", a.world_name):
        die("--world-name 只允许字母/数字/下划线/点/连字符：%r" % a.world_name)
    if not os.path.isfile(a.stl):
        die("找不到 --stl：%s" % a.stl)
    if a.resolution <= 0 or a.raster_res <= 0:
        die("--resolution / --raster-res 必须为正")
    return a


def main(argv=None):
    t_start = time.time()
    a = parse_args(argv)
    name = a.world_name
    out = os.path.abspath(a.out_root)
    stl = os.path.abspath(a.stl)
    try:
        import trimesh
    except ImportError:
        die("需要 trimesh（pip install trimesh）")

    res = a.resolution
    stl_sha = sha256_file(stl)
    stl_bytes = os.path.getsize(stl)
    print("[stl]  %s\n       sha256 %s  %.3f MB  scale %g" % (stl, stl_sha, stl_bytes / 1e6, a.scale))

    # ---------------------------------------------------------------- 网格
    mesh = trimesh.load(stl, process=False)
    V = np.asarray(mesh.vertices, dtype=np.float64) * a.scale
    F = np.asarray(mesh.faces)
    lo, hi = V.min(axis=0), V.max(axis=0)
    print("[bbox] x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f] m  (%.2f x %.2f x %.2f m)"
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))

    Z, rx0, ry0 = rasterize_max(V, F, a.raster_res)
    print("[raster] %d x %d @ %.3f m  x0=%.4f y0=%.4f  (每格取最高面; %.0fs)"
          % (Z.shape[1], Z.shape[0], a.raster_res, rx0, ry0, time.time() - t_start))

    if a.floor_z is not None:
        floor_z, plane_area, levels = float(a.floor_z), float("nan"), []
        floor_src = "手工 --floor-z"
    else:
        floor_z, plane_area, levels = find_floor(Z, a.raster_res)
        floor_src = "自动（面积最大的水平面）"
    z_shift = -floor_z
    frac = (floor_z - lo[2]) / (hi[2] - lo[2])
    print("[floor] mesh z = %.8g  (平面面积 %.1f m², %s) ⇒ z 抬升 %+.8g m，地面落在 world z = 0"
          % (floor_z, plane_area, floor_src, z_shift))
    for z, ar, ne in levels[:4]:
        print("        水平面候选 z=%+.4f m  面积 %6.1f m²" % (z, ar))
    if a.floor_z is None and frac > 0.35:
        print("        ⚠️ 这个「地面」落在场地高度的 %.0f%% 处，不像底板顶面 —— 请用 --floor-z 复核"
              % (frac * 100))

    # ---------------------------------------------------------------- 地图网格 + 出生点
    xg0, yg0, nx, ny = grid_extent(lo, hi, res, a.map_pad, a.map_grid)
    Zg = aggregate_grid(Z, rx0, ry0, a.raster_res, xg0, yg0, nx, ny, res)
    inside = ~np.isnan(Zg)
    dz = Zg - floor_z
    occ = inside & (dz > a.floor_threshold)
    free = inside & ~occ
    cell = res * res
    print("[grid] %.2f m/格  %d x %d  ｜ field 左下角 = (%.3f, %.3f)  ｜ 外沿(无数据)也标占用"
          % (res, nx, ny, xg0, yg0))
    print("[map]  free %.1f m² ｜ occupied %.1f m² ｜ 场地外沿 %.1f m²（阈值 %.2f m）"
          % (free.sum() * cell, occ.sum() * cell, (~inside).sum() * cell, a.floor_threshold))

    plateau = inside & (dz <= a.plateau_threshold)          # 严格平台面：只在这里选出生点
    print("[plateau] 严格平台面（抬升 ≤ %.2f m）%.1f m²（占 free 的 %.0f%%）"
          % (a.plateau_threshold, plateau.sum() * cell, 100.0 * plateau.sum() / max(1, free.sum())))
    D, _ = distance_to_obstacle(plateau, res)
    need = a.robot_radius + a.clearance_margin
    sx, sy, clr, note, comps = choose_spawn(plateau, D, xg0, yg0, res, need, a.spawn)
    print("[spawn] x=%.3f y=%.3f  clearance=%.3f m（严格平台面上的距离变换；要求 ≥ %.2f = %.2f + %.2f）  %s"
          % (sx, sy, clr, need, a.robot_radius, a.clearance_margin, note))
    for i, c in enumerate(comps):
        print("        连通域%d: %.1f m²  x[%.2f,%.2f] y[%.2f,%.2f] 质心 (%.2f,%.2f)"
              % (i + 1, c["area_m2"], c["bbox"][0], c["bbox"][2], c["bbox"][1], c["bbox"][3],
                 c["centroid"][0], c["centroid"][1]))

    # ---------------------------------------------------------------- pgm + yaml
    img = np.zeros((ny, nx), np.uint8)          # 先全填占用（含场地外沿）
    img[free] = 254
    map_dir = os.path.join(out, "src", "rm_nav_bringup", "map")
    pgm_path = os.path.join(map_dir, "%s.pgm" % name)
    yaml_path = os.path.join(map_dir, "%s.yaml" % name)
    write_pgm(pgm_path, img[::-1])              # PGM 第 0 行 = 最高 y
    ox, oy = xg0 - sx, yg0 - sy                 # map 系 = 出生点相对系
    write_text(yaml_path, "image: %s.pgm\n"
                          "mode: trinary\n"
                          "resolution: %.2f\n"
                          "origin: [%.6g, %.6g, 0]\n"
                          "negate: 0\n"
                          "occupied_thresh: 0.65\n"
                          "free_thresh: 0.25" % (name, res, ox, oy))
    print("[map]  写出 %s + .yaml ｜ map 原点（= 场地左下角 − 出生点）= [%.6g, %.6g, 0]  ⇒ 出生点即 (0,0)"
          % (pgm_path, ox, oy))

    if a.preview:
        try:
            from PIL import Image
            rgb = np.zeros((ny, nx, 3), np.uint8)
            rgb[free] = (255, 255, 255)
            rgb[occ] = (0, 0, 0)
            rgb[~inside] = (60, 60, 200)
            Image.fromarray(rgb[::-1]).resize((nx * 2, ny * 2), Image.NEAREST).save(a.preview)
            print("[preview] %s" % a.preview)
        except Exception as ex:                 # 预览是可选功能，失败不影响资产
            print("[preview] 跳过：%s" % ex)

    # ---------------------------------------------------------------- PCD
    P, Fi = trimesh.sample.sample_surface(mesh, a.points, seed=a.seed)
    P = np.asarray(P, dtype=np.float64) * a.scale
    P[:, 2] += z_shift                          # → world/地面系（地面 z=0）
    N = np.abs(np.asarray(mesh.face_normals)[Fi])
    band = (P[:, 2] >= a.band[0]) & (P[:, 2] <= a.band[1])
    P, N = P[band], N[band]
    horiz = N[:, 2] > a.normal_z
    print("[pcd]  采样 %d 点 → 高度带 [%+.2f, %+.2f] m 内 %d 点（%.0f%% 近水平面）"
          % (a.points, a.band[0], a.band[1], len(P), horiz.mean() * 100))

    def voxel_ds(pts, leaf):
        key = np.floor(pts / leaf).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)   # 每体素留首个样本（确定性）
        return pts[np.sort(idx)]

    Pf = voxel_ds(P[horiz], a.voxel_horizontal)
    Pv = voxel_ds(P[~horiz], a.voxel_vertical)
    Pm = np.vstack([Pf, Pv]) if len(Pv) else Pf
    Pm = Pm.copy()
    Pm[:, 0] -= sx
    Pm[:, 1] -= sy
    Pm[:, 2] -= a.base_height                   # 地面 → z = -0.06（initial_pose [0,0,0] 语义）
    if len(Pm) == 0:
        die("高度带内没有点，请检查 --band / --scale / --floor-z")
    pcd_path = os.path.join(out, "src", "rm_nav_bringup", "PCD", "%s.pcd" % name)
    write_pcd(pcd_path, Pm)
    pcd_bbox = [float(Pm[:, 0].min()), float(Pm[:, 1].min()), float(Pm[:, 2].min()),
                float(Pm[:, 0].max()), float(Pm[:, 1].max()), float(Pm[:, 2].max())]
    print("[pcd]  近水平面 %.2f m leaf → %d 点 ＋ 近竖直面 %.2f m leaf → %d 点 = **%d 点**（%.2f MB）"
          % (a.voxel_horizontal, len(Pf), a.voxel_vertical, len(Pv), len(Pm),
             os.path.getsize(pcd_path) / 1e6))
    print("       map 系 bbox x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]（地面 z = %.2f）"
          % (pcd_bbox[0], pcd_bbox[3], pcd_bbox[1], pcd_bbox[4], pcd_bbox[2], pcd_bbox[5],
             -a.base_height))

    # ---------------------------------------------------------------- world / model 树
    center = (round((lo[0] + hi[0]) / 2, 2), round((lo[1] + hi[1]) / 2, 2))
    if a.camera_pose:
        camera = " ".join("%s" % float(v) for v in a.camera_pose)
    else:
        camera = "%s %s %s %s" % (float(round(0.75 * (hi[0] - lo[0]) + center[0])),
                                  float(round(0.75 * (hi[1] - lo[1]) + center[1])),
                                  float(round(0.70 * (hi[0] - lo[0]))), CAMERA_RPY)
    gen_note = ("%s —— 由 STL 生成的场地世界（world=%.2f x %.2f m；生成工具 "
                "tools/scripts/world/stl_to_world.py，参数与依据见 docs/worlds.md）。"
                % (name, hi[0] - lo[0], hi[1] - lo[1]))
    desc = ("%s world built from %s (sha256 %s..., %d B, scale %g + z shift %.8g)."
            % (name, os.path.basename(stl), stl_sha[:8], stl_bytes, a.scale, z_shift))
    mesh_tree = os.path.join(out, "src", "rm_simulation", "hzmi_rm_simulation", "meshes",
                             "%s_world" % name)
    world_tree = os.path.join(out, "src", "rm_simulation", "hzmi_rm_simulation", "world",
                              "%s_world" % name)
    tree_cfg = model_config(name, desc + " 注意：本目录是 Gazebo 的 model:// 解析根"
                                   "（GAZEBO_MODEL_PATH=<share>/hzmi_rm_simulation/meshes），"
                                   "世界文件里的 model://%s_world/meshes/%s.stl 指向本目录下的 meshes/。"
                                   % (name, name))
    world_cfg = model_config(name, desc + " 本目录与 meshes/%s_world 对称（映射既有目录结构）；"
                                   "实际生效的是 %s_world.world 里内联的那份模型，"
                                   "本目录的 meshes/ 不被任何路径引用。" % (name, name))
    sdf = model_sdf(name, z_shift, a.scale, a.max_contacts, desc)
    world_txt = world_file(name, z_shift, a.scale, a.max_contacts, center, camera, gen_note)

    files = []
    for tree, sdf_txt, cfg_txt in ((mesh_tree, sdf, tree_cfg), (world_tree, sdf, world_cfg)):
        os.makedirs(os.path.join(tree, "meshes"), exist_ok=True)
        shutil.copyfile(stl, os.path.join(tree, "meshes", "%s.stl" % name))
        write_text(os.path.join(tree, "model.sdf"), sdf_txt)
        write_text(os.path.join(tree, "model.config"), cfg_txt)
        files += [os.path.join(tree, "meshes", "%s.stl" % name),
                  os.path.join(tree, "model.sdf"), os.path.join(tree, "model.config")]
    world_path = write_text(os.path.join(world_tree, "%s_world.world" % name), world_txt)
    files.append(world_path)
    files += [pgm_path, yaml_path, pcd_path]
    print("[world] %s\n[world] %s" % (world_path, os.path.dirname(pgm_path)))

    # ---------------------------------------------------------------- 摘要 + manifest
    launch = {
        "rm_simulation.launch.py": (
            "WorldType.%s = '%s'，并在 get_world_config 里加：\n"
            "        WorldType.%s: {\n"
            "            'x': '%.3f', 'y': '%.3f', 'z': '%g', 'yaw': '0.0',\n"
            "            'world_path': '%s_world/%s_world.world'\n"
            "        }" % (name, name, name, sx, sy, a.spawn_z, name, name)),
        "bringup_sim.launch.py": (
            "amcl_init_x / amcl_init_y 的字典各加一项 '{name}': 0.0（本图是出生点相对系）；\n"
            "  GICP/ICP 的 initial_pose [0,0,0] 不用改".format(name=name)),
    }
    print("[launch] 本工具不改 launch 文件；接进仿真要各加一处（格式见 docs/worlds.md §3）：")
    print("         rm_simulation.launch.py: WorldType.%s ｜ x=%.3f y=%.3f z=%g ｜ world_path="
          "%s_world/%s_world.world" % (name, sx, sy, a.spawn_z, name, name))
    print("         bringup_sim.launch.py:   amcl_init_x/y 字典 + '%s': 0.0" % name)

    manifest = {
        "tool": "tools/scripts/world/stl_to_world.py",
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "world_name": name,
        "stl": {"path": stl, "sha256": stl_sha, "bytes": stl_bytes, "scale": a.scale},
        "bbox_m": {"min": [float(v) for v in lo], "max": [float(v) for v in hi],
                   "extent": [float(hi[i] - lo[i]) for i in range(3)]},
        "floor": {"mesh_z": floor_z, "z_shift": z_shift, "plane_area_m2": plane_area,
                  "source": floor_src, "height_frac": float(frac),
                  "candidates": [{"z": z, "area_m2": ar} for z, ar, _ in levels]},
        "map": {"resolution": res, "size": [nx, ny], "grid_lower_left_field": [xg0, yg0],
                "origin_map": [ox, oy], "floor_threshold": a.floor_threshold,
                "plateau_threshold": a.plateau_threshold,
                "plateau_m2": float(plateau.sum() * cell),
                "floor_raster_res": a.raster_res,
                "free_m2": float(free.sum() * cell), "occupied_m2": float(occ.sum() * cell),
                "border_m2": float((~inside).sum() * cell),
                "free_cells": int(free.sum()), "occupied_cells": int(occ.sum()),
                "border_cells": int((~inside).sum()),
                "map_grid_explicit": a.map_grid is not None},
        "spawn": {"field_xy": [sx, sy], "clearance_m": clr,
                  "clearance_mask": "strict plateau (dz <= %.2f m)" % a.plateau_threshold,
                  "required_clearance_m": need, "robot_radius": a.robot_radius,
                  "margin": a.clearance_margin, "mode": note,
                  "components": comps},
        "pcd": {"points_sampled": a.points, "seed": a.seed, "band": list(a.band),
                "points_in_band": int(len(P)),
                "horizontal_points": int(len(Pf)), "vertical_points": int(len(Pv)),
                "points": int(len(Pm)), "bytes": os.path.getsize(pcd_path),
                "voxel_horizontal": a.voxel_horizontal, "voxel_vertical": a.voxel_vertical,
                "normal_z": a.normal_z, "base_height": a.base_height,
                "bbox_map": pcd_bbox},
        "world": {"field_center": list(center), "camera_pose": camera,
                  "z_shift": z_shift, "model_pose": [0, 0, 0, 0, 0, 0]},
        "files": [os.path.relpath(p, out) for p in files],
        "launch": launch,
        "elapsed_s": round(time.time() - t_start, 1),
    }
    mpath = a.manifest or os.path.join(out, ".tmp_cache", "world", "%s.manifest.json" % name)
    write_text(os.path.abspath(mpath), json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print("[manifest] %s" % os.path.abspath(mpath))
    print("[done] %s 用时 %.1fs  ⇒ %d 个文件" % (name, time.time() - t_start, len(files) + 1))


if __name__ == "__main__":
    main()
