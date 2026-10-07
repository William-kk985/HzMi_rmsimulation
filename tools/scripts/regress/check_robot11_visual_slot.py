#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_robot11_visual_slot.py —— `robot11_visual:=decimated|full` 的**契约检查**（Phase 5 / §13）

问：换视觉档位，会不会顺手改掉**会改变仿真行为**的任何东西？
（雷达位姿 / 轮心轮径 / 足印 / 云台链 / 质量惯量 / `<collision>` —— 一个都不许动。）

做法：把 `urdf/sentry_robot_robot11_sim.xacro` 用 `xacro` **真的渲染两遍**（decimated / full），
然后逐条断言（不是"读代码觉得没问题"，是"渲染出来的 URDF 逐元素比"）：

  A. 两档之间**只有** `<visual>/<geometry>/<mesh>/@filename` 不同：
     把两份 URDF 的 XML 树规范序列化，把所有 `<visual>` 里的 mesh filename 换成占位符后，
     两份必须**逐字节相同**（`<collision>`/`<inertial>`/`<joint>`/`<gazebo>` 全部在内）。
  B. 每个 link 的 `<visual><origin>` 在两档之间逐字相同（几何位置/单位不变）。
  C. 所有 `<visual>`/`<collision>` 引用的 mesh 文件**都存在**（`package://robot11/` → 源码目录）。
  D. 没有任何 `<mesh scale=...>`（STL 单位 = 米，不加缩放）。
  E. 三角形预算：decimated 档每 link ≤ `--per-link-budget`（默认 50000）、
     整车 ≤ `--total-budget`（默认 200000）；full 档 = 上游原始值（有个已知总数）。
  F. `inventory/visual_decimation.json` 与磁盘上的抽稀件**一致**（三角形数 + sha256），
     且每条容差都过（bbox / 单向表面误差 / 剪影 IoU / 边界边）。
  G. 物理不变量（与上游 `urdf/upstream/robot11.urdf` 对齐）：
     质量合计 9.5521 kg、`body_to_livox` 的 xyz、四个转向关节 origin ±0.1821345596729、
     轮 collision 是 `<cylinder r=0.058 l=0.0452>`。
  H. `visual_decimated`（Phase 4 的布尔别名）与 `robot11_visual` 的**优先级**：
     显式传的那个赢；都不传 ⇒ decimated（默认）。

用法：
  python3 tools/scripts/regress/check_robot11_visual_slot.py          # 打印表格 + 退出码
  python3 tools/scripts/regress/check_robot11_visual_slot.py --json .tmp_robot11/work/visual_slot.json
"""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
PKG = os.path.join(REPO, 'src/rm_simulation/robot11_description')
MESHES = os.path.join(PKG, 'meshes')
UPSTREAM = os.path.join(REPO, 'src/rm_nav_bringup/urdf/upstream/robot11.urdf')
XACRO = os.path.join(REPO, 'src/rm_nav_bringup/urdf/sentry_robot_robot11_sim.xacro')

#: `full` 档的整车三角形数（上游 12 个 STL 的实测值，见 inventory/mesh_inventory.json）
FULL_TOTAL_FACES = 2821320
#: 质量合计（上游 12 条 <inertial> 的 <mass> 求和，§9.4）
TOTAL_MASS = 9.5521
#: 雷达安装点（上游 `body_to_livox` 的 xyz，逐字）
LIVOX_XYZ = (0.000561701465058485, 0.130915824456595, 0.15702816968305)
#: 轴距/轮距的一半（`j2..j5` 的 origin）
WHEEL_OFFSET = 0.1821345596729


def render(extra_args, timeout=180):
    """跑 xacro（真的渲染），返回 (stdout, stderr, rc)。

    ⚠️ 必须带上工作空间的 `install/setup.bash`：xacro 里有 `$(find ros2_livox_simulation)`
    （雷达插件那个包），只 source `/opt/ros/humble` 会报 `PackageNotFoundError`（实测）。
    这里显式 source，免得"忘了 source"被误读成"模型坏了"。
    """
    setup = os.path.join(REPO, 'install', 'setup.bash')
    inner = ('source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
             + ('source %s >/dev/null 2>&1; ' % setup if os.path.isfile(setup) else '')
             + "exec xacro '%s' 'xyz:=0 0 0' 'rpy:=0 0 0' %s"
             % (XACRO, ' '.join("'%s'" % a for a in extra_args)))
    p = subprocess.run(['bash', '-c', inner], capture_output=True, text=True, timeout=timeout)
    return p.stdout, p.stderr, p.returncode


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for c in iter(lambda: f.read(1 << 20), b''):
            h.update(c)
    return h.hexdigest()


def stl_faces(path):
    n = os.path.getsize(path)
    with open(path, 'rb') as f:
        head = f.read(84)
    if len(head) == 84:
        cnt = int.from_bytes(head[80:84], 'little')
        if 84 + 50 * cnt == n:
            return cnt
    raise ValueError('%s 不是 binary STL（ascii STL 未支持：本包 12+12 个全是 binary）' % path)


def normalise(xml_text):
    """把所有 <visual> 里的 mesh filename 换成占位符，返回规范序列化的树（用于逐字节比 A）。"""
    root = ET.fromstring(xml_text)
    for vis in root.iter('visual'):
        m = vis.find('geometry/mesh')
        if m is not None and m.get('filename'):
            m.set('filename', '__VISUAL_MESH__')
    return ET.tostring(root, encoding='unicode')


def mesh_files(xml_text):
    root = ET.fromstring(xml_text)
    out = {'visual': [], 'collision': []}
    for kind in ('visual', 'collision'):
        for el in root.iter(kind):
            m = el.find('geometry/mesh')
            if m is not None:
                out[kind].append(m.get('filename'))
    return out


def resolve_pkg_uri(uri):
    """`package://robot11/<rel>` → 源码目录下的真实文件（**不依赖 install/**，免得先 build 才能查）。"""
    m = re.match(r'^package://([^/]+)/(.*)$', uri)
    if not m:
        return None
    pkg, rel = m.group(1), m.group(2)
    if pkg != 'robot11':
        return None
    return os.path.join(PKG, rel)


def visual_origins(xml_text):
    root = ET.fromstring(xml_text)
    out = {}
    for link in root.iter('link'):
        o = link.find('visual/origin')
        out[link.get('name')] = (o.get('xyz'), o.get('rpy')) if o is not None else None
    return out


def per_link_faces(xml_text):
    """按 <visual> 引用的 URL 统计每个 link 的视觉三角形数。"""
    root = ET.fromstring(xml_text)
    out = {}
    for link in root.iter('link'):
        tot = 0
        for vis in link.findall('visual'):
            m = vis.find('geometry/mesh')
            if m is None:
                continue
            p = resolve_pkg_uri(m.get('filename'))
            if p and os.path.isfile(p):
                tot += stl_faces(p)
        out[link.get('name')] = tot
    return out


def collision_refs(xml_text):
    """`<collision>` 的规范序列化（用于"两档之间逐字节相同"与"轮是 cylinder"两条断言）。"""
    root = ET.fromstring(xml_text)
    return {link.get('name'): ET.tostring(link, encoding='unicode')
            .split('<collision>')[1:] for link in root.iter('link')}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--per-link-budget', type=int, default=50000)
    ap.add_argument('--total-budget', type=int, default=200000)
    ap.add_argument('--json', default=None)
    a = ap.parse_args()

    rep = {'checks': [], 'ok': True}

    def chk(name, ok, detail=''):
        rep['checks'].append({'check': name, 'ok': bool(ok), 'detail': detail})
        if not ok:
            rep['ok'] = False
        print('  %s %-58s %s' % ('OK ' if ok else '**FAIL**', name, detail))
        return bool(ok)

    print('[visual-slot] 渲染两档 xacro（真的跑 xacro，不是读代码）…')
    dec_xml, dec_err, dec_rc = render(['robot11_visual:=decimated'])
    full_xml, full_err, full_rc = render(['robot11_visual:=full'])
    if dec_rc != 0 or full_rc != 0:
        print(dec_err[-2000:]); print(full_err[-2000:])
        raise SystemExit('[visual-slot] xacro 渲染失败（rc=%d/%d）' % (dec_rc, full_rc))
    chk('A0 两档 xacro 都渲染成功', True, 'dec %d B / full %d B' % (len(dec_xml), len(full_xml)))

    # ---- 默认（不传参）= decimated ----
    def_xml, _, def_rc = render([])
    chk('H1 不传参时默认 = decimated（与显式 decimated 逐字节相同）',
        def_rc == 0 and normalise(def_xml) == normalise(dec_xml))
    legacy_true, _, _ = render(['visual_decimated:=true'])
    legacy_false, _, _ = render(['visual_decimated:=false'])
    chk('H2 别名 visual_decimated:=true ⇒ decimated',
        normalise(legacy_true) == normalise(dec_xml))
    chk('H3 别名 visual_decimated:=false ⇒ full',
        normalise(legacy_false) == normalise(full_xml))
    both, _, _ = render(['robot11_visual:=full', 'visual_decimated:=true'])
    chk('H4 两个都给且冲突时 ⇒ 显式别名(非空)赢（= decimated）',
        normalise(both) == normalise(dec_xml))

    # ---- A：两档只差 visual mesh 文件名 ----
    chk('A1 两档 URDF「除 visual mesh 文件名外」逐字节相同',
        normalise(dec_xml) == normalise(full_xml))

    # ---- B：visual origin 不变 ----
    o_dec, o_full = visual_origins(dec_xml), visual_origins(full_xml)
    chk('B1 每个 link 的 <visual><origin> 两档逐字相同', o_dec == o_full,
        '%d 个 link' % len(o_dec))

    # ---- C：文件都存在 ----
    miss = []
    for xml in (dec_xml, full_xml):
        mf = mesh_files(xml)
        for kind in ('visual', 'collision'):
            for uri in mf[kind]:
                p = resolve_pkg_uri(uri)
                if p is None or not os.path.isfile(p):
                    miss.append(uri)
    chk('C1 两档引用的 mesh 文件全部存在', not miss, ('缺：%s' % sorted(set(miss))[:4]
                                                     if miss else ''))

    # ---- D：无 scale ----
    scale = re.findall(r'<mesh[^>]*\bscale\s*=', dec_xml + full_xml)
    chk('D1 没有 <mesh scale=...>（STL 单位 = 米，不加缩放）', not scale)

    # ---- E：三角形预算 ----
    d_faces = per_link_faces(dec_xml)
    f_faces = per_link_faces(full_xml)
    over = {k: v for k, v in d_faces.items() if v > a.per_link_budget}
    chk('E1 decimated 每 link ≤ %d 面' % a.per_link_budget, not over, str(over))
    tot_d, tot_f = sum(d_faces.values()), sum(f_faces.values())
    chk('E2 decimated 整车 ≤ %d 面' % a.total_budget, tot_d <= a.total_budget,
        '实测 %d 面（%.1f%% 于 full）' % (tot_d, 100.0 * tot_d / max(1, tot_f)))
    chk('E3 full 档整车 = 上游原始面数 %d' % FULL_TOTAL_FACES, tot_f == FULL_TOTAL_FACES,
        '实测 %d' % tot_f)

    # ---- F：抽稀清单 ↔ 磁盘一致 + 容差 ----
    vd = json.load(open(os.path.join(PKG, 'inventory/visual_decimation.json')))
    bad = []
    for m in vd['meshes']:
        p = os.path.join(REPO, m['out'])
        if not os.path.isfile(p):
            bad.append('%s 缺' % m['link']); continue
        if stl_faces(p) != m['out_faces']:
            bad.append('%s 面数不符' % m['link'])
        if sha256(p) != m.get('out_sha256'):
            bad.append('%s sha256 不符' % m['link'])
    chk('F1 清单里的抽稀件与磁盘逐字节一致（sha256 + 面数）', not bad, str(bad[:4]))
    fails = [m['link'] for m in vd['meshes'] if not m['ok']]
    chk('F2 每条抽稀件都过容差（bbox/误差/剪影/边界边）', not fails and vd['all_ok'],
        'all_ok=%s' % vd['all_ok'])
    worst = max(vd['meshes'], key=lambda m: m['err_out2src_mm']['p99'])
    chk('F3 最差单向表面误差 p99 ≤ %.1f mm' % vd['tolerances']['err_p99_mm'],
        worst['err_out2src_mm']['p99'] <= vd['tolerances']['err_p99_mm'],
        '%s p99=%.2f mm / max=%.2f mm' % (worst['link'], worst['err_out2src_mm']['p99'],
                                          worst['err_out2src_mm']['max']))
    worstb = max(vd['meshes'], key=lambda m: m['bbox_err_max_m'])
    chk('F4 最差逐轴包围盒差 ≤ %.1f mm' % (vd['tolerances']['bbox_max_m'] * 1000),
        worstb['bbox_err_max_m'] <= vd['tolerances']['bbox_max_m'],
        '%s %.3f mm' % (worstb['link'], worstb['bbox_err_max_m'] * 1000))
    worsts = min(min(m['silhouette_iou'].values()) for m in vd['meshes'])
    chk('F5 最差三视剪影 IoU ≥ %.3f' % vd['tolerances']['silhouette_iou_min'],
        worsts >= vd['tolerances']['silhouette_iou_min'], '%.4f' % worsts)

    # ---- G：物理不变量 ----
    up = ET.parse(UPSTREAM).getroot()
    mass = sum(float(l.find('inertial/mass').get('value')) for l in up.findall('link')
               if l.find('inertial/mass') is not None)
    chk('G1 上游质量合计 = %.4f kg' % TOTAL_MASS, abs(mass - TOTAL_MASS) < 5e-4,
        '实测 %.4f' % mass)
    dec_root = ET.fromstring(dec_xml)
    j = [x for x in dec_root.iter('joint') if x.get('name') == 'body_to_livox'][0]
    xyz = [float(v) for v in j.find('origin').get('xyz').split()]
    chk('G2 body_to_livox 的 xyz 逐字不变（雷达位姿）',
        all(abs(xyz[i] - LIVOX_XYZ[i]) < 1e-12 for i in range(3)),
        ' '.join('%.9f' % v for v in xyz))
    wj = {}
    for x in dec_root.iter('joint'):
        o = x.find('origin')
        if o is None:
            continue
        v = [float(t) for t in o.get('xyz').split()]
        if abs(abs(v[0]) - WHEEL_OFFSET) < 1e-9 and abs(abs(v[1]) - WHEEL_OFFSET) < 1e-9:
            wj[x.get('name')] = tuple(round(t, 9) for t in v)
    chk('G3 四个转向关节 origin = ±%.13f（轴距=轮距）' % WHEEL_OFFSET, len(wj) == 4, str(sorted(wj)))
    cyl = 0
    for link in dec_root.iter('link'):
        if link.get('name') in ('l6', 'l7', 'l8', 'l9'):
            for c in link.findall('collision'):
                g = c.find('geometry/cylinder')
                # 轮宽逐轮取自各自 mesh 的包围盒：l6=0.0452、l7/l8/l9=0.0453（见生成器 WHEELS）
                if g is not None and abs(float(g.get('radius')) - 0.058) < 1e-9 \
                        and abs(float(g.get('length')) - 0.0452) < 1.5e-4:
                    cyl += 1
    chk('G4 四个轮的 <collision> 仍是 cylinder r=0.058 l≈0.0452', cyl == 4, '命中 %d 个' % cyl)
    nc_dec = sum(len(v) for v in collision_refs(dec_xml).values())
    nc_full = sum(len(v) for v in collision_refs(full_xml).values())
    chk('G5 两档 collision 元素个数相同（只换 visual）', nc_dec == nc_full,
        '%d 个' % nc_dec)

    rep.update({'tier_faces': d_faces, 'total_faces': {'decimated': tot_d, 'full': tot_f},
                'per_link_budget': a.per_link_budget, 'total_budget': a.total_budget})
    print('\n[visual-slot] 小结：decimated %d 面 / full %d 面（预算 %d / link, %d 总）'
          % (tot_d, tot_f, a.per_link_budget, a.total_budget))
    print('[visual-slot] 全部检查通过 = %s' % rep['ok'])
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, 'w') as f:
            json.dump(rep, f, indent=2, ensure_ascii=False)
            f.write('\n')
    return 0 if rep['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
