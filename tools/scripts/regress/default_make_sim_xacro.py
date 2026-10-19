#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""default_make_sim_xacro.py —— 生成**默认模型**（`robot:=''`）的单刚体底盘变体。

为什么需要它（实测归因，见 docs/frame_contract_v3_measurements.md §20）
------------------------------------------------------------------------
默认模型 `sentry_robot_sim.xacro` 是 **base_link + 4 个 continuous 轮子关节** 的多刚体链。
底盘驱动是 `libgazebo_ros_planar_move.so`，它的实现（`gazebo/physics/Model.cc:746-771`）是
**`SetLinearVel` / `SetAngularVel` 对模型里的每一个 link 各设一遍同一个 (v, ω)**：

  · 对**单刚体**自洽（绕质心转时本来就是每个 link 同一个 ω）；
  · 对**多刚体 + 关节树**不自洽（第 i 个 link 的质心速度本应是 ω × r_i，插件写成同一个 v）
    ⇒ 关节约束每步都要把它掰回来，**反作用把底盘的角速度吃掉**。

台架实测（同一世界/出生点、`/cmd_vel_chassis` 纯 `wz=0.8 rad/s` × 2 仿真秒 = 请求 91.7°）：

    默认模型**原样**（5 刚体 / 4 关节）        +3.93°  = **4.3%**
    本工具产物（sdformat 合并成 1 刚体）        +12.93° = **14.1%**
    同上 + 轮子对地摩擦 0.02（本工具同时写的）   +87.0°  = **94.9%**

⇒ 光"锁自由度/塌成单刚体"只到 14%：第二限制项是**轮子对地摩擦**（planar_move 是速度驱动，
轮子被焊死不能滚动 ⇒ 原地转 = 4 个轮子同时横向刮地）。真值剂量-响应（单刚体、同一命令）：
`mu=1 → 14.1% / 0.5 → 34.4% / 0.2 → 72.2% / 0.1 → 84.6% / 0.05 → 90.5% / 0.02 → 94.9% / 0.001 → 97.9%`。
本工具默认取 `--wheel-mu 0.02`（≥90% 且留余量）。**这是物理改动，不是几何改动**：
轮半径/轮心/车体/雷达/IMU 一个字节都不动（见自检）。

它做了什么（**只动"关节刚性"与"轮子对地摩擦"，不动任何几何**）
----------------------------------------------------------------
  1. `base_to_wheel1..4`：`continuous` → `fixed`（去掉 `<axis>/<limit>/<anchor>`）。
     sdformat 的 URDF→SDF 会把 fixed 子 link 的**碰撞/视觉几何与惯量按平行轴定理合并**进
     `base_link` ⇒ Gazebo 里是 **1 个刚体**（与 `robot11` 的 `robot11_weld_chassis.py` 同一效果，
     只是这里把"合并"这件事交给 sdformat 做，所以不需要搬几何、也不可能搬错）。
  2. 给 4 个轮子的 `<gazebo reference="wheel_N">` 补 `<mu1>/<mu2>`（原模型的
     `<mu1>/<mu2>` 写在 `<link>` 里面，Gazebo 的 URDF 解析器**只认 `<gazebo reference>`** ⇒ 一直是坏的）。
  3. 自证（不通过就**报错不写文件**）：
     · `gz sdf -p` 后**模型里只有 1 个 link**（= 真的单刚体），质量/惯量与平行轴定理算出的值相等；
     · 逐条碰撞几何（类型/尺寸/在 base_link 系里的 AABB）与**源模型**（5 link，FK 到 base_link）
       **逐条相等**（<1e-6 m）；整车最低点 = −0.06 m 且落在轮子上；
     · `livox_frame_joint` / `imu_joint` 的 origin 与源模型**逐字相等**（安装几何不变）。

用法：
  python3 tools/scripts/regress/default_make_sim_xacro.py                 # 重新生成 rigid 变体
  python3 tools/scripts/regress/default_make_sim_xacro.py --report-only   # 只自检，不写文件
  python3 tools/scripts/regress/default_make_sim_xacro.py --chassis articulated --out /tmp/x.xacro
      # 回退档：原样复制源文件（逐字节相同）
  python3 tools/scripts/regress/default_make_sim_xacro.py --wheel-mu 1.0  # 复现"只塌刚体"的 14.1%
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot11_weld_chassis as _weld          # noqa: E402  复用同一套 FK / AABB / 惯量合成代码

DEFAULT_IN = os.path.join(REPO, 'src/rm_nav_bringup/urdf/sentry_robot_sim.xacro')
DEFAULT_OUT = os.path.join(REPO, 'src/rm_nav_bringup/urdf/sentry_robot_sim_rigid.xacro')

WHEEL_JOINTS = ('base_to_wheel1', 'base_to_wheel2', 'base_to_wheel3', 'base_to_wheel4')
WHEEL_LINKS = ('wheel_1', 'wheel_2', 'wheel_3', 'wheel_4')

#: 默认轮子对地摩擦：剂量-响应里 0.02 → 94.9%，0.05 → 90.5%，0.1 → 84.6%（台架，wz=0.8×2 s）
DEFAULT_WHEEL_MU = 0.02

#: 生成物文件头（写进 `<robot>` 里第一条注释；**XML 注释里不能有 `--`**）
HEADER = """
  ==================================================================================
  【自动生成，不要手改】sentry_robot_sim_rigid.xacro —— 默认模型（robot:=''）的
  **单刚体底盘**变体。生成器：tools/scripts/regress/default_make_sim_xacro.py
  ==================================================================================
  与源文件 urdf/sentry_robot_sim.xacro 的差别**只有两类**（几何一个字节都没动）：
    ① base_to_wheel1..4：continuous → fixed（去掉 axis/limit/anchor）。
       sdformat 的 URDF→SDF 把 fixed 子 link 的几何/惯量**合并**进 base_link
       ⇒ Gazebo 里是 1 个刚体；而 planar_move 的 Set{Linear,Angular}Vel 只对单刚体自洽。
       台架实测纯 wz=0.8 rad/s × 2 仿真秒：5 刚体 4.3% → 1 刚体 14.1%。
    ② 4 个轮子的 <gazebo reference="wheel_N"> 补 mu1=mu2=__MU__（原模型把 mu1/mu2 写在
       <link> 里，Gazebo 的 URDF 解析器只认 <gazebo reference> ⇒ 一直是无效的）。
       平面速度驱动 + 焊死的轮子 ⇒ 原地转就是 4 个轮子横向刮地；mu=1 时摩擦吃掉 86%。
       单刚体真值剂量-响应：mu 1 → 14.1% / 0.5 → 34.4% / 0.2 → 72.2% / 0.1 → 84.6%
       / 0.05 → 90.5% / 0.02 → 94.9% / 0.001 → 97.9%。
    自检（本文件由生成器逐条断言，不通过就不写文件）：模型里 1 个 link；碰撞几何 AABB
    逐条与源模型相等；最低点 −0.06 m 在轮子上；质量 __MASS__ kg / Izz __IZZ__ kg·m²；
    livox_frame_joint / imu_joint 的 origin 与源模型逐字相同。
    回退：`default_make_sim_xacro.py --chassis articulated`（= 源文件逐字节副本），
    或把 bringup_sim.launch.py 的 _ROBOT_SLOTS[''] 指回 sentry_robot_sim.xacro。
    实测与影响面：docs/frame_contract_v3_measurements.md §20。robot:=robot11 不受影响。
"""
#: `__MU__` 由 `make_rigid(--wheel-mu)` 填；`__MASS__` / `__IZZ__` 由 `main()` 用**自检算出来的**
#: 值填（不写死：将来改了几何，文件头不会说谎）。
HEADER_TPL = HEADER


# ---------------------------------------------------------------- 变换（只改刚性/摩擦）
def make_rigid(xacro_text, wheel_mu=DEFAULT_WHEEL_MU):
    """返回 (新 xacro 文本, report)。只改：4 个轮关节类型 + 4 个 <gazebo reference> 摩擦。"""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(xacro_text, parser=parser)
    _weld._register_xacro_prefix(root)

    rep = {'wheel_joints_fixed': [], 'gazebo_mu_added': [], 'wheel_mu': wheel_mu}
    for j in root.findall('joint'):
        nm = j.get('name')
        if nm not in WHEEL_JOINTS:
            continue
        if j.get('type') != 'continuous':
            raise SystemExit('[rigid] %s 的 type=%r 不是 continuous ⇒ 源文件结构变了，'
                             '请先复核本工具（拒绝盲改）' % (nm, j.get('type')))
        j.set('type', 'fixed')
        for tag in ('axis', 'limit', 'anchor'):
            for e in list(j.findall(tag)):
                j.remove(e)
        rep['wheel_joints_fixed'].append(nm)

    gz = {g.get('reference'): g for g in root.findall('gazebo')}
    for nm in WHEEL_LINKS:
        g = gz.get(nm)
        if g is None:
            raise SystemExit('[rigid] 找不到 <gazebo reference=%r>（摩擦没处可写）' % nm)
        for tag in ('mu1', 'mu2'):
            for e in list(g.findall(tag)):
                g.remove(e)
                rep.setdefault('replaced_mu', []).append('%s/%s' % (nm, tag))
        # 顺序：material 之后追加（Gazebo 不关心顺序）
        e = ET.SubElement(g, 'mu1')
        e.text = '%.9g' % wheel_mu
        e = ET.SubElement(g, 'mu2')
        e.text = '%.9g' % wheel_mu
        rep['gazebo_mu_added'].append(nm)

    # 文件头（插到 <robot> 的最前面 ⇒ 读者第一眼就看见"这是生成的"）
    hdr = HEADER_TPL.replace('__MU__', '%.9g' % wheel_mu).replace('--', '\u2014')
    root.insert(0, ET.Comment(hdr))
    out = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'
    return out, rep


# ---------------------------------------------------------------- 自检
def _xacro(urdf_or_xacro, out_path, extra=()):
    """跑 xacro CLI（与 launch 真正 spawn 时用的是同一个库/同一条命令形态）。"""
    cmd = ['xacro', urdf_or_xacro] + list(extra)
    with open(out_path, 'w', encoding='utf-8') as f:
        p = subprocess.run(cmd, stdout=f, stderr=subprocess.PIPE, text=True, timeout=180)
    if p.returncode != 0:
        raise SystemExit('[rigid] xacro 失败（%s）：\n%s' % (' '.join(cmd), p.stderr[-2000:]))


def _urdf_fk(urdf_path):
    """URDF → ({link: (t, R)} 从 base_link 出发的 FK, {joint: (parent,child,type,xyz,rpy)})。"""
    root = ET.parse(urdf_path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    jinfo = []
    for j in root.findall('joint'):
        xyz, rpy = _weld._pose_of(j, 'joint ' + str(j.get('name')))
        jinfo.append(dict(name=j.get('name'), type=j.get('type'),
                          parent=j.find('parent').get('link'),
                          child=j.find('child').get('link'), xyz=xyz, rpy=rpy))
    eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T = {}
    # 图可能有多个根（dummy 等）：从 base_link 出发即可（默认模型只有一棵树）
    T['base_link'] = ([0.0, 0.0, 0.0], eye)
    changed = True
    while changed:
        changed = False
        for ji in jinfo:
            if ji['parent'] in T and ji['child'] not in T:
                p, R = T[ji['parent']]
                T[ji['child']] = (_weld._add(_weld._mv(R, ji['xyz']), p),
                                  _weld._mm(R, _weld._rpy2R(*ji['rpy'])))
                changed = True
    missing = [nm for nm in links if nm not in T]
    return T, jinfo, missing, links


def _sdf_pose(el, parent=None):
    """`<pose>x y z r p y</pose>` → (t, R)，可选与父系 (lt, lR) 复合（SDF 的 pose 是相对的）。"""
    po = el.find('pose')
    vals = [float(v) for v in (po.text.split() if po is not None and po.text else ['0'] * 6)]
    t, R = vals[0:3], _weld._rpy2R(*vals[3:6])
    if parent is None:
        return t, R
    lt, lR = parent
    return _weld._add(_weld._mv(lR, t), lt), _weld._mm(lR, R)


def _sdf_collision_aabbs(sdf_path):
    """SDF → 每条 collision 的 (minz, AABB, geom 类型, 参数, 归属 link)。只支持 box/cylinder/sphere。"""
    root = ET.parse(sdf_path).getroot()
    model = root.find('model')
    out = []
    for link in model.findall('link'):
        # ⚠️ SDF 里 collision 的 <pose> 是**相对它自己 link** 的，而 link 又有相对 model 的
        #    <pose> ⇒ 必须复合，否则多 link（源模型）与单 link（rigid）没法比。
        lt, lR = _sdf_pose(link)
        for coll in link.findall('collision'):
            t, R = _sdf_pose(coll, parent=(lt, lR))
            geom = coll.find('geometry')
            kind = list(geom)[0]
            if kind.tag == 'box':
                params = tuple(float(v) for v in (kind.findtext('size') or '').split())
            elif kind.tag == 'cylinder':
                params = (float(kind.findtext('radius')), float(kind.findtext('length')))
            elif kind.tag == 'sphere':
                params = (float(kind.findtext('radius')),)
            else:
                out.append((None, None, kind.tag, (), link.get('name'),
                            coll.get('name')))
                continue
            # ⚠️ SDF 的几何尺寸是**子元素**（`<box><size>`），与 URDF 的属性写法不同 ⇒ 不能直接
            #    复用 `_weld._geom_points`（它读属性）；这里按同一套采样口径另算一遍。
            pts = _sdf_geom_points(kind, R, t)
            mn, mx = _weld._iaabb(pts)
            out.append((round(mn[2], 6), tuple(round(v, 6) for v in mn),
                        tuple(round(v, 6) for v in mx), kind.tag, params,
                        link.get('name'), coll.get('name')))
    return out


def _sdf_geom_points(kind, R, t):
    """SDF 几何的特征点（box/cylinder 与 URDF 同一套采样口径：圆柱 2×24 环）。"""
    pts = []
    if kind.tag == 'box':
        s = [float(v) / 2.0 for v in kind.findtext('size').split()]
        for a in (-s[0], s[0]):
            for b in (-s[1], s[1]):
                for c in (-s[2], s[2]):
                    pts.append([a, b, c])
    elif kind.tag == 'cylinder':
        r = float(kind.findtext('radius'))
        h = float(kind.findtext('length')) / 2.0
        for i in range(24):
            th = 2.0 * math.pi * i / 24.0
            for z in (-h, h):
                pts.append([r * math.cos(th), r * math.sin(th), z])
    elif kind.tag == 'sphere':
        r = float(kind.findtext('radius'))
        for dx in (-r, 0.0, r):
            for dy in (-r, 0.0, r):
                for dz in (-r, 0.0, r):
                    pts.append([dx, dy, dz])
    else:
        raise SystemExit('[rigid] SDF 里有不认识的碰撞几何 %r' % kind.tag)
    return [_weld._add(_weld._mv(R, p), t) for p in pts]


def _sdf_to_urdf_check(urdf_path, workdir, tag):
    """`gz sdf -p`（HOME 指到可写临时目录：本机 $HOME 可能是只读的）。"""
    gz = shutil.which('gz')
    if gz is None:
        return None
    home = os.path.join(workdir, 'home-' + tag)
    os.makedirs(home, exist_ok=True)
    out = os.path.join(workdir, tag + '.sdf')
    env = dict(os.environ, HOME=home)
    with open(out, 'w', encoding='utf-8') as f:
        p = subprocess.run([gz, 'sdf', '-p', urdf_path], stdout=f, stderr=subprocess.PIPE,
                           text=True, timeout=180, env=env)
    if p.returncode != 0:
        raise SystemExit('[rigid] gz sdf -p 失败（%s）：\n%s' % (urdf_path, p.stderr[-2000:]))
    return out


def self_check(in_xacro, rigid_xacro, wheel_mu, do_sdf=True):
    """几何/惯量/安装几何不变性自证。不通过 ⇒ SystemExit（生成器**不写文件**）。"""
    rep = {}
    with tempfile.TemporaryDirectory(prefix='dmk-') as td:
        a_urdf = os.path.join(td, 'source.urdf')
        b_urdf = os.path.join(td, 'rigid.urdf')
        _xacro(in_xacro, a_urdf)
        b_x = os.path.join(td, 'rigid.xacro')
        with open(b_x, 'w', encoding='utf-8') as f:
            f.write(rigid_xacro)
        _xacro(b_x, b_urdf)

        # ---- ① URDF 层：FK 到 base_link 的逐条碰撞 AABB + 惯量合成 ----
        Ta, ja, miss_a, links_a = _urdf_fk(a_urdf)
        Tb, jb, miss_b, links_b = _urdf_fk(b_urdf)
        if miss_a or miss_b:
            raise SystemExit('[rigid] 有 link 从 base_link 走不到：%s / %s'
                             % (miss_a, miss_b))
        aabb_a = _weld._collision_aabbs(links_a, Ta)
        aabb_b = _weld._collision_aabbs(links_b, Tb)
        if len(aabb_a) != len(aabb_b):
            raise SystemExit('[rigid] ❌ 碰撞条数变了：%d → %d' % (len(aabb_a), len(aabb_b)))
        for x, y in zip(aabb_a, aabb_b):
            if (abs(x[0] - y[0]) > 1e-9 or any(abs(p - q) > 1e-9 for p, q in zip(x[1], y[1]))
                    or any(abs(p - q) > 1e-9 for p, q in zip(x[2], y[2]))):
                raise SystemExit('[rigid] ❌ 碰撞几何 AABB 变了（几何搬错了）：\n  %s\n  %s'
                                 % (x, y))
        weldable = [nm for nm in links_a]
        Ma, coma, Ia, _ = _weld._merge_inertials(links_a, Ta, weldable, 'base_link')
        Mb, comb, Ib, _ = _weld._merge_inertials(links_b, Tb, weldable, 'base_link')
        if abs(Ma - Mb) > 1e-9:
            raise SystemExit('[rigid] ❌ 总质量变了：%r → %r' % (Ma, Mb))
        for (p, q) in ((Ma, Mb), (Ia[2][2], Ib[2][2]), (Ia[0][0], Ib[0][0]), (Ia[1][1], Ib[1][1])):
            if abs(p - q) > 1e-9:
                raise SystemExit('[rigid] ❌ 惯量变了：%r → %r' % (p, q))
        # 安装几何（fixed 关节链）逐字相等
        joint_a = {j['name']: j for j in ja}
        joint_b = {j['name']: j for j in jb}
        mounts = {}
        for nm in ('livox_frame_joint', 'imu_joint'):
            if nm not in joint_a or nm not in joint_b:
                raise SystemExit('[rigid] 找不到安装关节 %r' % nm)
            if (joint_a[nm]['xyz'] != joint_b[nm]['xyz']
                    or joint_a[nm]['rpy'] != joint_b[nm]['rpy']
                    or joint_a[nm]['type'] != joint_b[nm]['type']):
                raise SystemExit('[rigid] ❌ 安装关节 %r 变了：%r → %r'
                                 % (nm, joint_a[nm], joint_b[nm]))
            mounts[nm] = {'xyz': joint_a[nm]['xyz'], 'rpy': joint_a[nm]['rpy'],
                          'type': joint_a[nm]['type']}
        rep.update(mass_kg=round(Ma, 6), izz=round(Ia[2][2], 6),
                   ixx=round(Ia[0][0], 6), iyy=round(Ia[1][1], 6),
                   com=[round(v, 6) for v in coma],
                   n_collisions=len(aabb_a), min_z=round(min(a[0] for a in aabb_a), 6),
                   min_z_geom=aabb_a[0][3], mounts=mounts,
                   aabb_identical=True, inertia_identical=True)
        if rep['min_z_geom'] != 'cylinder' or abs(rep['min_z'] + 0.06) > 1e-6:
            raise SystemExit('[rigid] ❌ 最低点不在轮子上或不是 -0.06 m：%s' % (aabb_a[0],))
        rep['wheels_are_lowest'] = True

        # ---- ② SDF 层：Gazebo 实际拿到的是不是"1 个刚体" + 几何是否逐条相等 ----
        #   ⚠️ 源模型的 SDF 用 `relative_to=<joint>` 帧语义（link 的 pose 相对关节帧），
        #      直接读 `<pose>` 会得到"所有轮子叠在原点"的假读数 ⇒ **源侧的几何基准一律用
        #      URDF 的 FK**（上面 ① 已算好 aabb_a），SDF 侧只验"rigid 是 1 个 link，且它的
        #      逐条碰撞几何与 aabb_a 相等"。
        if do_sdf:
            a_sdf = _sdf_to_urdf_check(a_urdf, td, 'source')
            b_sdf = _sdf_to_urdf_check(b_urdf, td, 'rigid')
            if a_sdf and b_sdf:
                ma = ET.parse(a_sdf).getroot().find('model')
                mb_sdf = ET.parse(b_sdf).getroot().find('model')
                links_a_sdf = ma.findall('link')
                links_b_sdf = mb_sdf.findall('link')
                rep.update(sdf_links_source=len(links_a_sdf),
                           sdf_links_rigid=len(links_b_sdf))
                # 源模型 URDF 7 link（含 livox/imu）→ SDF 5 link（fixed 的 livox/imu 已被
                # sdformat 合并）⇒ 这里只要求"源侧是多刚体"（>1），刚性档必须是 1。
                if len(links_a_sdf) <= 1:
                    raise SystemExit('[rigid] 源模型 SDF 竟然只有 1 个 link ⇒ 基准不是多刚体')
                if len(links_b_sdf) != 1:
                    raise SystemExit('[rigid] ❌ Gazebo 里不是单刚体：rigid 有 %d 个 link'
                                     % len(links_b_sdf))
                sb = _sdf_collision_aabbs(b_sdf)
                key = lambda r: (r[3], r[1], r[2])          # (geom 类型, AABB min, AABB max)
                sa_k = sorted([key(r) for r in aabb_a], key=str)
                sb_k = sorted([key(r) for r in sb], key=str)
                if len(sa_k) != len(sb_k) or any(
                        a[0] != b[0] or any(abs(p - q) > 5e-5 for p, q in zip(a[1], b[1]))
                        or any(abs(p - q) > 5e-5 for p, q in zip(a[2], b[2]))
                        for a, b in zip(sa_k, sb_k)):
                    raise SystemExit('[rigid] ❌ SDF 单刚体的碰撞几何与源模型 URDF 不等：\n'
                                     '  source=%d 条 %s\n  rigid=%d 条 %s'
                                     % (len(sa_k), sa_k, len(sb_k), sb_k))
                mlink = links_b_sdf[0]
                sdf_mass = float(mlink.find('inertial/mass').text)
                sdf_izz = float(mlink.find('inertial/inertia/izz').text)
                mus = [c.findtext('surface/friction/ode/mu')
                       for c in mlink.findall('collision')
                       if c.find('geometry/cylinder') is not None]
                if abs(sdf_mass - Ma) > 1e-6:
                    raise SystemExit('[rigid] ❌ sdformat 合并后质量 %r ≠ 源 %r'
                                     % (sdf_mass, Ma))
                if abs(sdf_izz - Ia[2][2]) > 1e-6:
                    raise SystemExit('[rigid] ❌ sdformat 合并后 Izz %r ≠ 平行轴定理 %r'
                                     % (sdf_izz, Ia[2][2]))
                if len(mus) != 4 or any(m is None or abs(float(m) - wheel_mu) > 1e-12
                                        for m in mus):
                    raise SystemExit('[rigid] ❌ 轮子摩擦没进 SDF（读到 %r，期望 %r）'
                                     % (mus, wheel_mu))
                rep.update(sdf_mass=round(sdf_mass, 6), sdf_izz=round(sdf_izz, 6),
                           sdf_wheel_mu=mus,
                           sdf_collisions_rigid=len(sb), sdf_collisions_source=len(aabb_a))
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', default=DEFAULT_IN)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--chassis', default='rigid', choices=['rigid', 'articulated'])
    ap.add_argument('--wheel-mu', type=float, default=DEFAULT_WHEEL_MU)
    ap.add_argument('--report-only', action='store_true')
    ap.add_argument('--skip-sdf', action='store_true')
    a = ap.parse_args()

    src = open(a.inp, encoding='utf-8').read()
    if a.chassis == 'articulated':
        rep = {'chassis': 'articulated', 'note': '源文件逐字节副本（回退档）',
               'bytes': len(src)}
        if not a.report_only:
            with open(a.out, 'w', encoding='utf-8') as f:
                f.write(src)
            print('[rigid] 回退档写出 %s（%d B，与源文件逐字节相同）'
                  % (a.out, os.path.getsize(a.out)), file=sys.stderr)
        print(json.dumps(rep, ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    out, rep = make_rigid(src, wheel_mu=a.wheel_mu)
    rep['chassis'] = 'rigid'
    rep.update(self_check(a.inp, out, a.wheel_mu, do_sdf=not a.skip_sdf))
    print(json.dumps(rep, ensure_ascii=False, indent=2, sort_keys=True))
    if not a.report_only:
        out = (out.replace('__MASS__', '%.4f' % rep['mass_kg'])
                  .replace('__IZZ__', '%.6f' % rep['izz']))
        if '__MASS__' in out or '__IZZ__' in out:      # 防御：占位符没填完就别写
            raise SystemExit('[rigid] ❌ 文件头占位符没填完（__MASS__/__IZZ__）')
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write(out)
        sys.stderr.write('[rigid] %s -> %s (%d B)\n' % (a.inp, a.out, os.path.getsize(a.out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
