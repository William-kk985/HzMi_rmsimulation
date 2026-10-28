#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""perimeter_fence.py —— 给 STEP 生成的场地世界加一圈**静态围栏**（立柱 + 横杆）

背景（取证见 `docs/worlds_2027.md` §9）
    两个 2027 场地都是从 STEP 三角化来的，**只有地面和一堆矮结构**：
    `RMUL2027` 的几何最高点 0.42 m（就是场地自带的边界矮墙），`RMUC2027` 除了中央龙门/灯塔
    没有任何东西把场地围起来。而真实赛场的点云里，1~3 m 高度上有一圈明显的**边界环**
    （`PCD/RMUL.pcd` 的 z 1.0~1.6 m 环 + 到 3 m 的墙 + 3~14 m 的顶棚），
    3D 全局重定位（GICP/KISS/Scan Context）就靠这些竖直面。本工具把这一圈补回来。

几何约定
    · 围栏**中心线** = 场地可行驶区/几何外沿的矩形（由调用方给定，必须是从 STL 量出来的）；
    · 立柱 `0.06 x 0.06`，顶面在 `z = +height`（默认 1.50），**向下埋 0.10 m**（`--embed`）
      保证地面起伏 ≤3 cm 时立柱不会悬空；
    · 横杆 3 根 `0.05 x 0.05`，中心在 `--rails`（默认 0.30/0.70/1.10），沿每条边通长；
    · 立柱间距：每条边 `n = round(L/spacing)` 段、`L/n` 一根（默认 ≈2.0 m，均分边长）；
      四角共用立柱；
    · 一整圈**连续、不留门**（理由见文档：仿真里机器人从场内出生，连续一圈对配准更有利；
      真要开门：跳过落在缺口区间内的立柱，并把该边的横杆拆成两段即可，本工具故意不做参数化）；
    · `<static>true</static>`、单一 link、每个盒子一个 collision + 一个 visual、`mu=1`。

产出（两棵树，与 `_world` 目录同构）
    src/rm_simulation/hzmi_rm_simulation/meshes/<NAME>/model.config + model.sdf   ← model:// 解析根
    src/rm_simulation/hzmi_rm_simulation/world/<NAME>/model.config  + model.sdf   ← 结构镜像

用法
    perimeter_fence.py --name RMUC2027_fence --xmin -14 --xmax 14 --ymin -5.85 --ymax 9.15
    perimeter_fence.py --name RMUL2027_fence --xmin -6.30 --xmax 6.30 --ymin -4.30 --ymax 4.30
"""

import argparse
import math
import pathlib
import sys

HEADER = """<?xml version='1.0'?>
<sdf version='1.7'>
  <!-- {name}：场地四周的静态围栏（由 tools/scripts/world/perimeter_fence.py 生成，勿手改）。
       中心线矩形 x [{xmin:.3f}, {xmax:.3f}] × y [{ymin:.3f}, {ymax:.3f}]（{sx:.2f} × {sy:.2f} m，周长 {perim:.1f} m）
       立柱 {post:.2f}×{post:.2f} m，顶面 z={height:.2f}，向下埋 {embed:.2f} m；间距 {spacings}
       横杆 3 根 {rail:.2f}×{rail:.2f} m，中心 z = {rails}
       一共 {npost} 根立柱 + {nrail} 根横杆 = {nbox} 个盒子（每个 collision + visual）
       为什么需要它 / 实测数字见 docs/worlds_2027.md §9；坐标与场地地面的关系见 §3。 -->
  <model name='{name}'>
    <static>true</static>
    <allow_auto_disable>true</allow_auto_disable>
    <pose>0 0 0 0 0 0</pose>
    <link name='fence'>
      <inertial>
        <mass>1000.0</mass>
        <inertia><ixx>100.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>100.0</iyy><iyz>0.0</iyz><izz>100.0</izz></inertia>
        <pose>0 0 0 0 0 0</pose>
      </inertial>
      <pose>0 0 0 0 0 0</pose>
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <kinematic>false</kinematic>
      <enable_wind>false</enable_wind>
"""

BOX = """      <collision name='{name}_col'>
        <max_contacts>10</max_contacts>
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <surface>
          <friction><ode><mu>1</mu><mu2>1</mu2><fdir1>0 0 0</fdir1><slip1>0</slip1><slip2>0</slip2></ode></friction>
          <bounce><restitution_coefficient>0</restitution_coefficient><threshold>1e+06</threshold></bounce>
        </surface>
      </collision>
      <visual name='{name}_vis'>
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <lighting>true</lighting>
          <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
      </visual>
"""

FOOTER = """    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0" ?>
<model>
\t<name>{name}</name>
\t<version>1.0</version>
\t<sdf version="1.7">model.sdf</sdf>

\t<author>
\t\t<name>Lihan Chen</name>
\t\t<email>lihanchen2004@163.com</email>
\t</author>

\t<description>{desc}</description>
</model>
"""


def split_len(length, spacing):
    """按 ≈spacing 均分一条边，返回 (段数, 实际间距)。"""
    n = max(1, int(round(length / spacing)))
    return n, length / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--xmin', type=float, required=True)
    ap.add_argument('--xmax', type=float, required=True)
    ap.add_argument('--ymin', type=float, required=True)
    ap.add_argument('--ymax', type=float, required=True)
    ap.add_argument('--post', type=float, default=0.06)
    ap.add_argument('--height', type=float, default=1.50, help='立柱顶面 z')
    ap.add_argument('--embed', type=float, default=0.10, help='立柱向下埋深')
    ap.add_argument('--spacing', type=float, default=2.0)
    ap.add_argument('--rail', type=float, default=0.05)
    ap.add_argument('--rails', default='0.30,0.70,1.10')
    ap.add_argument('--out-root', default='.')
    a = ap.parse_args()

    rails = [float(v) for v in a.rails.split(',')]
    sx = a.xmax - a.xmin
    sy = a.ymax - a.ymin
    n_x, sp_x = split_len(sx, a.spacing)     # x 方向边（沿 x 走的两条边，长度 sx）
    n_y, sp_y = split_len(sy, a.spacing)

    boxes = []       # (name, x, y, z, size_x, size_y, size_z)
    zc_post = a.height - (a.height + a.embed) / 2.0
    post_sz = a.height + a.embed
    # 四条边上的立柱（四角共用 ⇒ 只建一遍）
    def add_post(i, x, y):
        boxes.append(('post_%03d' % i, x, y, zc_post, a.post, a.post, post_sz))

    idx = 0
    for k in range(n_x + 1):                 # y = ymin / ymax 两条边，沿 x 分布（四角都建）
        x = a.xmin + k * sp_x
        for y in (a.ymin, a.ymax):
            add_post(idx, x, y); idx += 1
    for k in range(1, n_y):                  # x = xmin / xmax 两条边，去掉四角
        y = a.ymin + k * sp_y
        for x in (a.xmin, a.xmax):
            add_post(idx, x, y); idx += 1

    # 横杆：每条边一根通长（与立柱重叠，静态无妨）
    rail_idx = 0
    cx, cy = (a.xmin + a.xmax) / 2.0, (a.ymin + a.ymax) / 2.0   # ★ 矩形不居中于原点时必须用中心
    for h in rails:
        for (x, y, dx, dy) in ((cx, a.ymin, sx, a.rail), (cx, a.ymax, sx, a.rail),
                               (a.xmin, cy, a.rail, sy), (a.xmax, cy, a.rail, sy)):
            boxes.append(('rail_%03d' % rail_idx, x, y, h, dx, dy, a.rail)); rail_idx += 1

    body = ''.join(BOX.format(name=n, x=x, y=y, z=z, sx=dx, sy=dy, sz=dz)
                   for (n, x, y, z, dx, dy, dz) in boxes)
    txt = HEADER.format(name=a.name, xmin=a.xmin, xmax=a.xmax, ymin=a.ymin, ymax=a.ymax,
                        sx=sx, sy=sy, perim=2 * (sx + sy), post=a.post, height=a.height,
                        embed=a.embed, rail=a.rail, rails='/'.join('%.2f' % r for r in rails),
                        npost=idx, nrail=rail_idx, nbox=len(boxes),
                        spacings='%.3f m（%d 段）/ %.3f m（%d 段）' % (sp_x, n_x, sp_y, n_y)) + body + FOOTER

    root = pathlib.Path(a.out_root)
    outs = [root / 'src/rm_simulation/hzmi_rm_simulation/meshes' / a.name,
            root / 'src/rm_simulation/hzmi_rm_simulation/world' / a.name]
    for d in outs:
        d.mkdir(parents=True, exist_ok=True)
        (d / 'model.sdf').write_text(txt, encoding='utf-8')
        (d / 'model.config').write_text(MODEL_CONFIG.format(
            name=a.name,
            desc=('Static perimeter fence generated by tools/scripts/world/perimeter_fence.py: '
                  'centerline x [%.3f, %.3f] y [%.3f, %.3f], posts %.2f m to z=%.2f every ~%.2f m, '
                  '3 rails at z=%s. See docs/worlds_2027.md §9.'
                  % (a.xmin, a.xmax, a.ymin, a.ymax, a.post, a.height, a.spacing,
                     '/'.join('%.2f' % r for r in rails)))), encoding='utf-8')

    bx = (a.xmin - a.post / 2, a.xmax + a.post / 2, a.ymin - a.post / 2, a.ymax + a.post / 2)
    print('[%s] 中心线 x[%.3f,%.3f] y[%.3f,%.3f]  %.2f x %.2f m  周长 %.1f m' %
          (a.name, a.xmin, a.xmax, a.ymin, a.ymax, sx, sy, 2 * (sx + sy)))
    print('  立柱 %d（0.06x0.06，顶面 z=%.2f，埋 %.2f）+ 横杆 %d（0.05x0.05 @ %s）= %d 盒' %
          (idx, a.height, a.embed, rail_idx, '/'.join('%.2f' % r for r in rails), len(boxes)))
    print('  间距：x 边 %.3f m x %d；y 边 %.3f m x %d' % (sp_x, n_x, sp_y, n_y))
    print('  几何 AABB：x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]' %
          (bx[0], bx[1], bx[2], bx[3], -(a.embed), a.height))
    print('  写出：%s' % ' ; '.join(str(d) for d in outs))
    return 0


if __name__ == '__main__':
    sys.exit(main())
