#!/usr/bin/env python3
"""chassis_bench_mkworld.py —— 把一份模型 SDF 塞进 RMUL2026 世界，做一个「只有 Gazebo + 底盘插件」
的快速台架（**角速度通道变体矩阵**用；§M）。

配合：
  tools/scripts/tiltmount/run_chassis_yaw_bench.sh <tag> --model <sdf> [--variant …] [--wz …]
  tools/scripts/tiltmount/chassis_yaw_probe.py（探针本体，由上面那个 shell 调用）

为什么要台架（而不是每次都起整栈 nav2+LIO+雷达）：整栈一次 ~3 min、RTF≈0.35，
而"角速度通道"必须做**变体矩阵**（十来种改法 × 前后对照）。台架只加载同一个世界 + 一个模型 +
底盘插件，摘掉 LIO/nav2/雷达，一次 <40 s、RTF≈1.00。

用法:
  mkworld.py --model <urdf|sdf> --out <world> [--variant baseline|float|nofric|nowheelcol|spherewheel|fixsteer]
             [--spawn 4.3 3.35 0.2] [--strip-sensors] [--keep-visual] [--wheel-mu M] [--add-jsp]

为什么不走 bringup：本台架只回答一个问题 —— **给定 /cmd_vel_chassis，底盘转不转**。
它把 LIO / nav2 / 雷达（30000 条射线 @10 Hz）全摘掉 ⇒ 单步 RTF 高、一跑 <40 s，便于做变体矩阵。
最终结论仍用仓库原生工具（run_nav_goal_forensics.sh）复核。
"""
import argparse
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
BASE_WORLD = os.path.join(REPO, 'src/rm_simulation/hzmi_rm_simulation/world/'
                                'RMUL2026_world/RMUL2026_world.world')
WHEELS = ('l6', 'l7', 'l8', 'l9')
STEER = ('j2', 'j3', 'j4', 'j5')
ALLJOINTS = ('j2', 'j3', 'j4', 'j5', 'j6', 'j7', 'j8', 'j9')

JSP = """      <plugin name='joint_state_publisher' filename='libgazebo_ros_joint_state_publisher.so'>
        <ros><remapping>~/out:=joint_states</remapping></ros>
        <update_rate>200</update_rate>
%s      </plugin>
"""


def _rpy2R(r, p, y):
    import math as _m
    cr, sr = _m.cos(r), _m.sin(r)
    cp, sp = _m.cos(p), _m.sin(p)
    cy, sy = _m.cos(y), _m.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _R2rpy(R):
    import math as _m
    p = _m.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.999999:
        r = _m.atan2(R[2, 1], R[2, 2])
        y = _m.atan2(R[1, 0], R[0, 0])
    else:
        r = _m.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def _pose_of(el, tag):
    """读 <origin>（URDF）或 <pose>（SDF）—— 两种写法都要认，否则 FK 会退化成单位阵
    （这个坑实测踩过：weld 出来的模型几何全叠在根 link 原点，轮子凭空短了 4 cm）。"""
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    e = el.find(tag)
    if e is None:
        e = el.find('pose')
    if e is not None and e.text:
        v = [float(x) for x in e.text.split()]
        xyz = np.array(v[:3])
        if len(v) >= 6:
            rpy = np.array(v[3:6])
    return xyz, _rpy2R(*rpy)


def weld_into_root(model):
    """把模型塌成**一个刚体**：所有 link 的 inertial/collision 折算进根 link，删掉全部 joint 与其它 link。

    语义 = 「底盘由 planar_move 直接驱动 ⇒ 轮子不应作为独立刚体与它较劲」。碰撞几何逐条保留
    （只把 pose 变换到根 link 系），质量/惯量按平行轴定理合成 ⇒ 总质量与惯量张量不变。
    """
    links = {l.get('name'): l for l in model.findall('link')}
    root = model.find('link')
    root_name = root.get('name')
    joints = []
    for j in model.findall('joint'):
        pe, ce = j.find('parent'), j.find('child')
        par = pe.get('link') or (pe.text or '').strip()
        ch = ce.get('link') or (ce.text or '').strip()
        xyz, R = _pose_of(j, 'origin')
        joints.append((par, ch, xyz, R))
    # FK: root -> link
    T = {root_name: (np.zeros(3), np.eye(3))}
    changed = True
    while changed:
        changed = False
        for par, ch, xyz, R in joints:
            if par in T and ch not in T:
                p, Rl = T[par]
                T[ch] = (p + Rl @ xyz, Rl @ R)
                changed = True
    M = 0.0
    com = np.zeros(3)
    items = []
    for name, L in links.items():
        ine = L.find('inertial')
        if ine is None:
            continue
        m = float(ine.find('mass').text)
        c, Rc = _pose_of(ine, 'pose')
        Ie = ine.find('inertia')
        I = np.array([[float(Ie.find('ixx').text), float(Ie.find('ixy').text), float(Ie.find('ixz').text)],
                      [float(Ie.find('ixy').text), float(Ie.find('iyy').text), float(Ie.find('iyz').text)],
                      [float(Ie.find('ixz').text), float(Ie.find('iyz').text), float(Ie.find('izz').text)]])
        p, Rl = T[name]
        cw = p + Rl @ c
        Rw = Rl @ Rc
        items.append((m, cw, Rw @ I @ Rw.T))
        M += m
        com += m * cw
    com /= M
    Itot = np.zeros((3, 3))
    for m, cw, Iw in items:
        d = cw - com
        Itot += Iw + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    # 碰撞折算
    moved = []
    for name, L in links.items():
        p, Rl = T[name]
        for coll in L.findall('collision'):
            po = coll.find('pose')
            cxyz = np.zeros(3)
            Rc = np.eye(3)
            if po is not None and po.text:
                v = [float(x) for x in po.text.split()]
                cxyz = np.array(v[:3])
                if len(v) >= 6:
                    Rc = _rpy2R(*v[3:6])
            nxyz = p + Rl @ cxyz
            nR = Rl @ Rc
            r, pi, y = _R2rpy(nR)
            if name != root_name:
                L.remove(coll)
                moved.append((nxyz, (r, pi, y), coll))
    for nxyz, (r, pi, y), coll in moved:
        po = coll.find('pose')
        if po is None:
            import xml.etree.ElementTree as _E
            po = _E.SubElement(coll, 'pose')
            coll.remove(po)
            coll.insert(0, po)
        po.text = '%.9g %.9g %.9g %.9g %.9g %.9g' % (nxyz[0], nxyz[1], nxyz[2], r, pi, y)
        root.append(coll)
    # 根 link 的 inertial 换成合成值
    ine = root.find('inertial')
    if ine is not None:
        root.remove(ine)
    import xml.etree.ElementTree as _E
    ine = _E.Element('inertial')
    po = _E.SubElement(ine, 'pose')
    po.text = '%.9g %.9g %.9g 0 0 0' % (com[0], com[1], com[2])
    ms = _E.SubElement(ine, 'mass')
    ms.text = '%.9g' % M
    Ie = _E.SubElement(ine, 'inertia')
    for k, v in (('ixx', Itot[0, 0]), ('ixy', Itot[0, 1]), ('ixz', Itot[0, 2]),
                 ('iyy', Itot[1, 1]), ('iyz', Itot[1, 2]), ('izz', Itot[2, 2])):
        e = _E.SubElement(Ie, k)
        e.text = '%.9g' % v
    root.insert(0, ine)
    # 删掉所有 joint 与其它 link
    for j in list(model.findall('joint')):
        model.remove(j)
    for name, L in links.items():
        if name != root_name:
            model.remove(L)
    print('[weld] 合成刚体：%d link → 1（质量 %.4f kg，COM %s，Izz %.5f）'
          % (len(links), M, np.round(com, 4), Itot[2, 2]))


def low_friction(coll, mu):
    surf = coll.find('surface')
    if surf is None:
        surf = ET.SubElement(coll, 'surface')
    fr = surf.find('friction')
    if fr is None:
        fr = ET.SubElement(surf, 'friction')
    ode = fr.find('ode')
    if ode is None:
        ode = ET.SubElement(fr, 'ode')
    for k in ('mu', 'mu2'):
        e = ode.find(k)
        if e is None:
            e = ET.SubElement(ode, k)
        e.text = str(mu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='urdf 或 sdf（urdf 会先经 gz sdf -p 转换，需已转换好则给 .sdf）')
    ap.add_argument('--out', required=True)
    ap.add_argument('--variant', default='baseline')
    ap.add_argument('--spawn', nargs=4, type=float, default=[4.3, 3.35, 0.2, 0.0])
    ap.add_argument('--strip-sensors', action='store_true')
    ap.add_argument('--keep-visual', action='store_true')
    ap.add_argument('--wheel-mu', type=float, default=None)
    ap.add_argument('--all-mu', type=float, default=None)
    ap.add_argument('--add-jsp', action='store_true')
    ap.add_argument('--wheel-radius', type=float, default=0.058)
    ap.add_argument('--name', default='robot')
    ap.add_argument('--ur', type=float, default=None,
                    help='覆盖 planar_move 插件的 <update_rate>（Hz）：= 物理步长时它每个 physics step 都重设速度')
    ap.add_argument('--plugin-json', default=None,
                    help='直接把 <plugin> 的 XML 片段整体换掉（给 measure 用）')
    a = ap.parse_args()

    sdf = ET.parse(a.model).getroot()
    model = sdf.find('model')
    assert model is not None, 'no <model> in %s' % a.model

    if a.strip_sensors or not a.keep_visual:
        for link in model.findall('link'):
            for tag in ('sensor',):
                if a.strip_sensors:
                    for e in list(link.findall(tag)):
                        link.remove(e)
            if not a.keep_visual:
                for e in list(link.findall('visual')):
                    link.remove(e)

    if a.variant == 'float':
        # 全 link 关重力 ⇒ 完全没有接触（用来判决「接触是不是阻力来源」）
        for link in model.findall('link'):
            g = ET.SubElement(link, 'gravity')
            g.text = '0'
    elif a.variant == 'nofric':
        for link in model.findall('link'):
            if link.get('name') in WHEELS:
                for coll in link.findall('collision'):
                    low_friction(coll, 0.001)
    elif a.variant == 'nowheelcol':
        for link in model.findall('link'):
            if link.get('name') in WHEELS:
                for coll in list(link.findall('collision')):
                    link.remove(coll)
    elif a.variant == 'spherewheel':
        for link in model.findall('link'):
            if link.get('name') in WHEELS:
                for coll in link.findall('collision'):
                    geo = coll.find('geometry')
                    for e in list(geo):
                        geo.remove(e)
                    sph = ET.SubElement(geo, 'sphere')
                    r = ET.SubElement(sph, 'radius')
                    r.text = str(a.wheel_radius)
                    po = coll.find('pose')
                    if po is not None:
                        po.text = '0 0 0 0 0 0'
    elif a.variant == 'fixwheel':
        for j in model.findall('joint'):
            if j.get('name') in ('j6', 'j7', 'j8', 'j9'):
                j.set('type', 'fixed')
                ax = j.find('axis')
                if ax is not None:
                    j.remove(ax)
    elif a.variant == 'fixsteer':
        for j in model.findall('joint'):
            if j.get('name') in STEER:
                j.set('type', 'fixed')
                ax = j.find('axis')
                if ax is not None:
                    j.remove(ax)
    elif a.variant == 'fixall':
        for j in model.findall('joint'):
            j.set('type', 'fixed')
            ax = j.find('axis')
            if ax is not None:
                j.remove(ax)
    elif a.variant == 'weld':
        weld_into_root(model)
    elif a.variant == 'baseonly':
        # 只留根 link：用来量「单刚体」时 planar_move 的角速度执行上限（对照多 link 结构）
        for j in list(model.findall('joint')):
            model.remove(j)
        for link in list(model.findall('link')):
            if link.get('name') != 'base_link':
                model.remove(link)
    elif a.variant == 'boxwheel':
        # 用一个小 box 代替轮子（box 与 trimesh 的接触是 ODE 里最稳的一条）
        for link in model.findall('link'):
            if link.get('name') in WHEELS:
                for coll in link.findall('collision'):
                    geo = coll.find('geometry')
                    for e in list(geo):
                        geo.remove(e)
                    b = ET.SubElement(geo, 'box')
                    s = ET.SubElement(b, 'size')
                    s.text = '0.0452 0.116 0.116'
    elif a.variant != 'baseline':
        raise SystemExit('unknown variant %s' % a.variant)

    if a.wheel_mu is not None:
        for link in model.findall('link'):
            if link.get('name') in WHEELS:
                for coll in link.findall('collision'):
                    low_friction(coll, a.wheel_mu)
    if a.all_mu is not None:
        for link in model.findall('link'):
            for coll in link.findall('collision'):
                low_friction(coll, a.all_mu)

    if a.ur is not None:
        for pl in model.findall('plugin'):
            if pl.get('filename', '').endswith('libgazebo_ros_planar_move.so'):
                ur = pl.find('update_rate')
                if ur is None:
                    ur = ET.SubElement(pl, 'update_rate')
                ur.text = str(a.ur)

    model.set('name', a.name)
    pose = ET.Element('pose')
    pose.text = '%g %g %g 0 0 %g' % (a.spawn[0], a.spawn[1], a.spawn[2], a.spawn[3])
    model.insert(0, pose)

    if a.add_jsp:
        jsp = JSP % ''.join('        <joint_name>%s</joint_name>\n' % j for j in ALLJOINTS)
        # 放在第一个 plugin 前面（顺序无所谓；同一个 model 下多个 plugin 都会加载）
        model.append(ET.fromstring(jsp))

    txt = ET.tostring(model, encoding='unicode')
    base = open(BASE_WORLD).read()
    # 世界里的 <gravity> 用于 float 变体（整世界零重力更彻底）
    if a.variant == 'float':
        base = base.replace('<gravity>0 0 -9.8</gravity>', '<gravity>0 0 0</gravity>')
    out = base.replace('</world>', txt + '\n  </world>')
    with open(a.out, 'w') as f:
        f.write(out)
    print('[mkworld] %s（variant=%s, spawn=%s）→ %s' % (a.model, a.variant, a.spawn, a.out))


if __name__ == '__main__':
    main()
