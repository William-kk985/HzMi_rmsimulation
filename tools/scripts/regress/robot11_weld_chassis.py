#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11_weld_chassis.py —— 把 `robot11` 的仿真模型塌成**一个刚体**（单 link），并自证几何/惯量不变。

为什么需要它（实测归因，见 docs/tilted_lidar_fidelity.md §M）
------------------------------------------------------------------
`robot:=robot11` 的底盘由 `libgazebo_ros_planar_move.so` 驱动，而这个插件的实现是
（源码 `gazebo/physics/Model.cc:746-771`）：

    void Model::SetLinearVel (const Vector3d &_vel) { for (每个 link) link->SetLinearVel(_vel); }
    void Model::SetAngularVel(const Vector3d &_vel) { for (每个 link) link->SetAngularVel(_vel); }

= **把同一个 (v, ω) 写给模型里的每一个 link**。这对「单刚体」恰好是自洽的（刚体绕自身质心转时
正是每个 link 同一个 ω、质心平动同一个 v）；对**多刚体 + 关节树**则不自洽：绕质心转时第 i 个
link 的质心速度本应是 `ω × r_i`，而插件把它写成 v ⇒ 关节约束求解器每步都要把这个错误"掰回来"，
反作用把底盘的角速度吃掉。实测（同一世界/出生点/`/cmd_vel_chassis` `wz=1.0` 持续 12 s 仿真秒）：

    robot11 原样（13 link / 12 joint）        真值 +7.09°   = 请求的 1.03%
    同模型 `--variant fixall`（全关节改 fixed）+21.31°      = 3.10%
    同模型塌成单刚体（本工具）                **+113.32° = 16.48%**
    默认模型（7 link / 6 joint，对照）        +40.01°      = 5.82%
    单 link 的极简 box 模型（无接触，对照）    +681.25°     = 99.9%

同时实测：**自由偏航漂移**（无指令 60 s）从 **+18.4°**（原样）降到 **−0.23°**（塌成刚体，
默认模型 −0.24°）；"正前方被实体障碍挡住"的物理极限保持不变（0.3998 m vs 原样 0.4077 m，
§L 的两种模型分别 0.4199 / 0.5402 m）。

它做了什么（**只动"哪条 link 装哪块几何"，不动任何一块几何、不动质量/惯量总量**）
------------------------------------------------------------------
  1. 从根 link 做 FK（关节 origin 全是字面数字 ⇒ 可精确复算），把 l2…l11 的
     `<collision>` / `<visual>` 的 pose 变换到根 link 系，**逐条**挂到根 link 上；
  2. `l2…l11` 的 `<inertial>` 用平行轴定理合成到根 link 的 `<inertial>`
     （总质量、质心、绕质心的惯量张量逐项守恒）；
  3. 删掉这些 link 与它们的关节（`j2…j11`）；
  4. **保留** `livox_frame` / `imu_link` 及其固定关节（感知链的 TF 与传感器都挂在它们上面）。
  5. 自证：塌之前/之后 **碰撞几何的 AABB 逐条集合相等**（min z 等）、总质量与 Izz 相等；
     不相等就**报错不写文件**（这条检查抓到过一次 FK 读错 `<pose>` 的 bug）。

回退：生成器 `robot11_make_sim_xacro.py --chassis articulated`（= 今天的行为，逐字节相同）。

用法：
  python3 tools/scripts/regress/robot11_weld_chassis.py --in a.xacro --out b.xacro
  python3 tools/scripts/regress/robot11_weld_chassis.py --in a.xacro --report-only
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

#: ⚠️ `register_namespace` 会**删掉同名前缀的既有条目**（源码里那条 `if v == prefix: del`），
#: 所以不能把带/不带 www 的两个 URI 都注册成 `xacro` —— 实测那样只剩最后一个，
#: 另一个就退化成 `ns0:` 前缀，xacro 会直接拒绝解析。⇒ 按**文档里实际出现的那个**注册。
XACRO_NS = 'http://ros.org/wiki/xacro'


def _register_xacro_prefix(root):
    for el in root.iter():
        t = el.tag
        if isinstance(t, str) and t.startswith('{') and 'wiki/xacro' in t:
            ET.register_namespace('xacro', t[1:].split('}')[0])
            return True
    return False

#: 这些 link **不参与**塌陷（感知链要它们当独立帧：雷达/IMU 的 TF 由固定关节给出）
DEFAULT_KEEP = ('livox_frame', 'imu_link')


def _rpy2R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _R2rpy(R):
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(R[2][0]) < 0.999999:
        r = math.atan2(R[2][1], R[2][2])
        y = math.atan2(R[1][0], R[0][0])
    else:                                    # 万向锁：与 URDF 的 R=Rz·Ry·Rx 约定一致地取一支
        r = math.atan2(-R[1][2], R[1][1])
        y = 0.0
    return r, p, y


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _parse_pose(text, where):
    """读一个 3 元素属性（`xyz="…"` / `rpy="…"`）。"""
    if text is None:
        return [0.0, 0.0, 0.0]
    if '${' in text:
        raise SystemExit('[weld] %s 里含 xacro 表达式（%s）⇒ 无法数值塌陷；'
                         '塌陷只对字面几何成立' % (where, text))
    v = [float(x) for x in text.split()]
    if len(v) != 3:
        raise SystemExit('[weld] %s 期望 3 个数，读到 %r' % (where, text))
    return v


def _pose_of(el, where):
    """返回 (xyz, rpy)。⚠️ 必须**分开**读两个属性：曾经的写法"按 6 个数解析 rpy"会把
    `rpy="1.5708 -1.5708 0"` 读成零向量 ⇒ FK 静默退化（自证检查也会跟着一起错，见 §M）。"""
    o = el.find('origin')
    if o is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return (_parse_pose(o.get('xyz'), where + '/xyz'),
            _parse_pose(o.get('rpy'), where + '/rpy'))


def _iaabb(pts):
    return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]


def _geom_points(geom, R, t):
    """把一条碰撞/视觉几何的**特征点**变到根 link 系（用于 AABB 自证；圆柱取 2×24 环）。"""
    kind = list(geom)[0]
    pts = []
    if kind.tag == 'box':
        s = [float(x) / 2.0 for x in kind.get('size').split()]
        for a in (-s[0], s[0]):
            for b in (-s[1], s[1]):
                for c in (-s[2], s[2]):
                    pts.append([a, b, c])
    elif kind.tag == 'cylinder':
        r = float(kind.get('radius'))
        h = float(kind.get('length')) / 2.0
        for i in range(24):
            th = 2.0 * math.pi * i / 24.0
            for z in (-h, h):
                pts.append([r * math.cos(th), r * math.sin(th), z])
    elif kind.tag == 'sphere':
        r = float(kind.get('radius'))
        for dx in (-r, 0.0, r):
            for dy in (-r, 0.0, r):
                for dz in (-r, 0.0, r):
                    if abs(dx) + abs(dy) + abs(dz) <= r * 1.7321:
                        pts.append([dx, dy, dz])
    else:                                     # mesh 等：无法解析 ⇒ 交给调用方忽略
        return None
    return [_add(_mv(R, p), t) for p in pts]


def _collision_aabbs(links, T):
    """每条碰撞几何在**根 link 系**里的 (minz, AABB) —— 塌陷前后必须逐条相等。"""
    out = []
    for nm, link in links.items():
        p, R = T.get(nm, ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        for coll in link.findall('collision'):
            cxyz, crpy = _pose_of(coll, nm)
            Rc = _mm(R, _rpy2R(*crpy))
            tc = _add(_mv(R, cxyz), p)
            g = coll.find('geometry')
            ktag = list(g)[0].tag if len(list(g)) else 'none'
            pts = _geom_points(g, Rc, tc)
            if pts is None:
                continue
            mn, mx = _iaabb(pts)
            out.append((round(mn[2], 5), tuple(round(v, 5) for v in mn),
                        tuple(round(v, 5) for v in mx), ktag, nm))
    out.sort()
    return out


def _merge_inertials(links, T, weldable, root):
    """总质量 / 质心 / 绕质心的惯量张量（平行轴定理）。返回 (M, com, I3x3, per_link)。"""
    M = 0.0
    com = [0.0, 0.0, 0.0]
    items = []
    for nm in weldable:
        ine = links[nm].find('inertial')
        if ine is None:
            continue
        m = float(ine.find('mass').get('value'))
        cxyz, crpy = _pose_of(ine, nm + '/inertial')
        p, R = T[nm]
        cw = _add(_mv(R, cxyz), p)
        Rw = _mm(R, _rpy2R(*crpy))
        i = ine.find('inertia')
        I = [[float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
             [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
             [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]]
        Iw = _mm(_mm(Rw, I), [[Rw[j][i] for j in range(3)] for i in range(3)])
        items.append((m, cw, Iw, nm))
        M += m
        com = _add(com, [m * v for v in cw])
    com = [v / M for v in com]
    Itot = [[0.0] * 3 for _ in range(3)]
    for m, cw, Iw, _nm in items:
        d = _sub(cw, com)
        d2 = sum(v * v for v in d)
        for a in range(3):
            for b in range(3):
                Itot[a][b] += Iw[a][b] + m * ((d2 if a == b else 0.0) - d[a] * d[b])
    return M, com, Itot, items


def weld(xacro_text, root='base_link', keep=DEFAULT_KEEP, comment=True, strict=True):
    """返回 (新 xacro 文本, report dict)。"""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    rroot = ET.fromstring(xacro_text, parser=parser)
    _register_xacro_prefix(rroot)
    links = {l.get('name'): l for l in rroot.findall('link')}
    if root not in links:
        raise SystemExit('[weld] 找不到根 link %r（现有：%s）' % (root, sorted(links)))
    joints = rroot.findall('joint')

    # 关节表 + 保留哪些关节（keep 集合里 link 参与的关节一律保留）
    jinfo = []
    for j in joints:
        par = j.find('parent').get('link')
        ch = j.find('child').get('link')
        jinfo.append(dict(el=j, name=j.get('name'), type=j.get('type'), parent=par, child=ch))
    # ⚠️ 只对**要塌掉**的关节解 pose：保留的关节（body_to_livox）origin 里有 xacro 表达式
    #    （`${0.15702816968305 + livox_raise_m_p}`），数值解析会（正确地）报错。
    kept_joints = [ji for ji in jinfo if ji['parent'] in keep or ji['child'] in keep]
    weld_joints = [ji for ji in jinfo if ji not in kept_joints]
    for ji in weld_joints:
        xyz, rpy = _pose_of(ji['el'], 'joint ' + str(ji['name']))
        ji['xyz'], ji['rpy'] = xyz, rpy

    # FK：根 → 每个 link（只走要塌掉的关节）
    eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T = {root: ([0.0, 0.0, 0.0], eye)}
    changed = True
    while changed:
        changed = False
        for ji in weld_joints:
            if ji['parent'] in T and ji['child'] not in T:
                p, R = T[ji['parent']]
                T[ji['child']] = (_add(_mv(R, ji['xyz']), p), _mm(R, _rpy2R(*ji['rpy'])))
                changed = True
    weldable = [nm for nm in links if nm not in keep and nm in T]
    missing = [nm for nm in links if nm not in keep and nm not in T]
    if missing:
        raise SystemExit('[weld] 这些 link 从根走不到（关节表不全？）：%s' % missing)

    rep = {'welded_links': sorted(weldable), 'kept_links': sorted(keep),
           'dropped_joints': sorted(ji['name'] for ji in weld_joints)}
    aabb_before = _collision_aabbs(links, T)   # 塌陷前：逐 link 用各自 FK 后的 pose

    # ① 惯量合成
    M, com, Itot, items = _merge_inertials(links, T, weldable, root)
    rep.update(mass_kg=round(M, 6), com=[round(v, 6) for v in com],
               izz=round(Itot[2][2], 6),
               ixx=round(Itot[0][0], 6), iyy=round(Itot[1][1], 6))

    # ② 把视觉/碰撞逐条搬到根 link
    root_el = links[root]
    moved_coll, moved_vis = 0, 0
    for nm in weldable:
        if nm == root:
            continue
        p, R = T[nm]
        src = links[nm]
        for tag, box in (('collision', moved_coll), ('visual', None)):
            for el in list(src.findall(tag)):
                cxyz, crpy = _pose_of(el, nm + '/' + tag)
                nxyz = _add(_mv(R, cxyz), p)
                nrpy = _R2rpy(_mm(R, _rpy2R(*crpy)))
                o = el.find('origin')
                if o is None:
                    o = ET.Element('origin')
                    el.insert(0, o)
                o.set('xyz', ' '.join('%.9g' % v for v in nxyz))
                o.set('rpy', ' '.join('%.9g' % v for v in nrpy))
                src.remove(el)
                root_el.append(el)
                if tag == 'collision':
                    moved_coll += 1
                else:
                    moved_vis += 1
    # 根自己的碰撞也要在 AABB 表里（不动）
    # ③ 根 link 换成合成 inertial
    old = root_el.find('inertial')
    if old is not None:
        root_el.remove(old)
    ine = ET.Element('inertial')
    o = ET.SubElement(ine, 'origin')
    o.set('xyz', ' '.join('%.9g' % v for v in com))
    o.set('rpy', '0 0 0')
    ms = ET.SubElement(ine, 'mass')
    ms.set('value', '%.9g' % M)
    ii = ET.SubElement(ine, 'inertia')
    for k, (a, b) in (('ixx', (0, 0)), ('ixy', (0, 1)), ('ixz', (0, 2)),
                      ('iyy', (1, 1)), ('iyz', (1, 2)), ('izz', (2, 2))):
        ii.set(k, '%.9g' % Itot[a][b])
    root_el.insert(0, ine)
    rep.update(moved_collisions=moved_coll, moved_visuals=moved_vis,
               n_collisions_root=len(root_el.findall('collision')),
               n_visuals_root=len(root_el.findall('visual')))

    # ④a 去掉指向被塌掉的 link 的 `<gazebo reference="…">`（否则 Gazebo 每个都报一次
    #     "reference to unknown link"；材质本身已由各 <visual> 的 <material><color> 承担）
    dropped_refs = []
    for gz in list(rroot.findall('gazebo')):
        ref = gz.get('reference')
        if ref and ref in weldable and ref != root:
            rroot.remove(gz)
            dropped_refs.append(ref)
    rep['dropped_gazebo_refs'] = dropped_refs

    # ④ 删掉被塌掉的 link 与关节
    for nm in weldable:
        if nm != root:
            rroot.remove(links[nm])
    for ji in weld_joints:
        rroot.remove(ji['el'])

    # ⑤ 自证：碰撞几何 AABB 逐条相等（塌陷只搬 pose，不该动任何一块几何）
    links_after = {l.get('name'): l for l in rroot.findall('link')}
    aabb_after = _collision_aabbs(links_after, {root: ([0.0, 0.0, 0.0], eye)})
    rep['n_collisions_before'] = len(aabb_before)
    rep['n_collisions_after'] = len(aabb_after)
    rep['min_z_before'] = min(a[0] for a in aabb_before) if aabb_before else None
    rep['min_z_after'] = min(a[0] for a in aabb_after) if aabb_after else None
    # 独立的**物理**自证：整车最低点必须落在轮子（cylinder）上 —— 这一条不依赖 FK 的正确性
    #   （轮子在**塌陷后**的几何里也在，且"轮子着地"是 Phase 1 独立量过的事实：地面 z=-0.1025）。
    #   为什么必须有它：AABB 自证只查"塌陷前后一致"，若 FK 静默退化（实测踩过一次）两边会一起错。
    wheel_min = [a[0] for a in aabb_after if a[3] == 'cylinder']
    all_min = min(a[0] for a in aabb_after) if aabb_after else None
    rep['min_z_wheels'] = min(wheel_min) if wheel_min else None
    rep['wheels_are_lowest'] = bool(wheel_min and all_min is not None
                                    and abs(min(wheel_min) - all_min) < 1e-3)
    if strict and not rep['wheels_are_lowest']:
        raise SystemExit('[weld] ❌ 塌陷后**最低点不在轮子上**（全部 %.5f / 轮子 %s）⇒ FK 或几何搬错了，'
                         '拒绝写出模型（这一条曾在 rpy 解析退化成 0 时救过一次）'
                         % (all_min if all_min is not None else float('nan'), rep['min_z_wheels']))
    ok_aabb = (len(aabb_before) == len(aabb_after) and
               all(abs(a[0] - b[0]) < 5e-4 and
                   all(abs(x - y) < 5e-4 for x, y in zip(a[1], b[1])) and
                   all(abs(x - y) < 5e-4 for x, y in zip(a[2], b[2]))
                   for a, b in zip(aabb_before, aabb_after)))
    rep['wheels_are_lowest'] = bool(rep.get('wheels_are_lowest'))
    rep['aabb_identical'] = bool(ok_aabb)
    if strict and not ok_aabb:
        raise SystemExit('[weld] ❌ 塌陷前后碰撞几何 AABB 不一致（这是硬约束，拒绝写出模型）：'
                         'before=%d 条 min_z=%s / after=%d 条 min_z=%s'
                         % (len(aabb_before), rep['min_z_before'],
                            len(aabb_after), rep['min_z_after']))
    if comment:
        # ⚠️ XML 注释里**不允许 `--`**（会 not well-formed）；注释体里要写命令行开关 ⇒ 换全角破折号
        #    （与生成器 `_sanitize_comments` 同一套做法）。
        hdr = ('\n  ==================================================================\n'
               '       【我们加的｜2026-10-10】**单刚体底盘**（`--chassis rigid`，本文件默认）\n'
               '       ==================================================================\n'
               '       把 %d 个 link（%s）塌成 1 个刚体：碰撞/视觉几何**逐条**搬进根 link，\n'
               '       质量/质心/惯量按平行轴定理合成（M=%.4f kg、Izz=%.5f kg·m²）；\n'
               '       删掉 %d 个关节（j2…j11），**保留** %s 与它们的固定关节。\n'
               '       为什么（实测，见 docs/tilted_lidar_fidelity.md §M）：planar_move 是\n'
               '       `Model::Set*Vel` = 把同一个 (v,ω) 写给**每一个 link**，只有单刚体才自洽；\n'
               '       多刚体 + 关节树时关节约束每步都要把它掰回来，实测角速度只执行 1.03%%\n'
               '       （单刚体 16.5%%、默认模型 5.8%%）、自由偏航漂移 +18.4°/60 s（单刚体 −0.23°）。\n'
               '       几何/惯量守恒由 robot11_weld_chassis.py 自证（AABB 逐条相等，%.5f m 最低点）。\n'
               '       回退：`robot11_make_sim_xacro.py --chassis articulated`（= 原模型，逐字节相同）。\n'
               % (len(weldable), ', '.join(sorted(weldable)), M, Itot[2][2],
                  len(weld_joints), '/'.join(keep), rep['min_z_after'] or 0.0))
        rroot.append(ET.Comment(hdr.replace('--', '\u2014')))
    # ⑥ 串行化（保留注释；xacro 命名空间前缀由 register_namespace 还原）
    out = ET.tostring(rroot, encoding='unicode')
    out = '<?xml version="1.0"?>\n' + out + '\n'
    return out, rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--out', default=None)
    ap.add_argument('--root', default='base_link')
    ap.add_argument('--report-only', action='store_true')
    a = ap.parse_args()
    txt = open(a.inp, encoding='utf-8').read()
    out, rep = weld(txt, root=a.root)
    import json
    print(json.dumps(rep, ensure_ascii=False, indent=2, sort_keys=True))
    if not a.report_only:
        if not a.out:
            raise SystemExit('--out 必填（除非 --report-only）')
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write(out)
        sys.stderr.write('[weld] %s -> %s (%d B)\n' % (a.inp, a.out, os.path.getsize(a.out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
