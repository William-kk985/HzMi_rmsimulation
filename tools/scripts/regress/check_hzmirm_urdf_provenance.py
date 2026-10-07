#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_hzmirm_urdf_provenance.py —— 断言 robot:=hzmirm 的模型**逐字保留了用户那份 URDF 的运动学**。

背景（docs/robot_models.md §1/§2）：用户给的 URDF
  configs/sentry_robot.urdf @ William-kk985/hzmirmvision-master
  sha256 9463265182aa320322996a08dc1cbf4c964a7e1528835f495290f9ea477da98f
**不能**直接 spawn 进 Gazebo Classic —— 它所有 link 都没有 <inertial>，sdformat 的 URDF→SDF
会把"无惯性"的 link 整条丢掉（实测 `gz sdf -p` 只剩 `<model name='sentry_robot'/>`）。
所以本仓的做法是：**逐字抄下它的 link/joint，每个 link 再补一条我们写的 inertial**，并补
IMU/雷达/底盘插件 —— 那些"补的"东西必须与"抄的"分得清清楚楚。

本脚本就是把这条纪律变成可执行的断言（不跑 Gazebo、不依赖 ROS）：
  ① 上游那份 URDF 的逐字节副本（urdf/upstream/hzmirm_sentry_robot.urdf）与源文件 sha256 相同；
  ② 上游**确实一个 <inertial> 都没有**（如果哪天上游补了惯性，这条结论就该重写）；
  ③ 我们那份 xacro 展开后（xacro，默认参数 = 云台角 0）：
       · 上游的每个 link 都在，且 visual/collision 的 geometry 与 origin(rpy/xyz)**数值相等**、材质颜色相同；
       · 上游的每个 joint 都在，且 type/parent/child/origin **数值相等**（关节名一个不改）；
       · 上游没有的 link（livox_frame/imu_link）与插件/传感器/gazebo 标签**只可能来自我们**，
         脚本把它们列出来（供人工核对 docs/robot_models.md 的 provenance 表）。
  ④ 数值比较用容差（xacro 会把 `0` 写成 `0.0`），但**不放宽到"差不多"**：1e-9。

用法：
  python3 tools/scripts/regress/check_hzmirm_urdf_provenance.py            # 展开 xacro 并断言
  python3 tools/scripts/regress/check_hzmirm_urdf_provenance.py --print-added
退出码 0 = 通过；非 0 = 有不一致（消息里给出具体 link/joint/字段）。
"""
import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
UPSTREAM = os.path.join(REPO, 'src', 'rm_nav_bringup', 'urdf', 'upstream', 'hzmirm_sentry_robot.urdf')
XACRO = os.path.join(REPO, 'src', 'rm_nav_bringup', 'urdf', 'sentry_robot_hzmirm_sim.xacro')
TOL = 1e-9

#: 上游那份文件按用户给的 URL 取回时的 sha256（docs/robot_models.md §1.1 记录同一个值）
EXPECTED_SHA256 = '9463265182aa320322996a08dc1cbf4c964a7e1528835f495290f9ea477da98f'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for blk in iter(lambda: f.read(65536), b''):
            h.update(blk)
    return h.hexdigest()


def nums(text, n=None):
    """把 xyz/rpy/size 这类字符串切成 float 列表。"""
    if text is None:
        return None
    vals = [float(v) for v in re.split(r'[\s,]+', text.strip()) if v]
    if n is not None and len(vals) != n:
        raise ValueError('期望 %d 个数，实际 %r' % (n, text))
    return vals


def close(a, b):
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    return len(a) == len(b) and all(abs(x - y) <= TOL for x, y in zip(a, b))


def origin_of(elem, tag):
    o = elem.find(tag)
    if o is None:
        return (None, None)
    return (nums(o.get('xyz'), 3), nums(o.get('rpy'), 3))


def geometry_of(elem):
    g = elem.find('geometry')
    if g is None:
        return None
    for kind in ('box', 'cylinder', 'sphere', 'mesh'):
        e = g.find(kind)
        if e is None:
            continue
        if kind == 'box':
            return ('box', nums(e.get('size'), 3))
        if kind == 'cylinder':
            return ('cylinder', [float(e.get('radius')), float(e.get('length'))])
        if kind == 'sphere':
            return ('sphere', [float(e.get('radius'))])
        return ('mesh', [e.get('filename')])
    return None


def color_of(elem):
    m = elem.find('material')
    if m is None:
        return None
    c = m.find('color')
    if c is None:
        return m.get('name')
    return nums(c.get('rgba'), 4)


def fmt(v):
    return 'None' if v is None else str(v)


def compare(kind, name, field, up, mine, problems):
    if not close(up, mine):
        problems.append('%s [%s] 字段 %s 不一致：上游 %s ≠ 我们的 %s'
                        % (kind, name, field, fmt(up), fmt(mine)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--print-added', action='store_true', help='列出"我们加的"元素')
    args = ap.parse_args()

    problems = []
    if not os.path.isfile(UPSTREAM):
        print('❌ 找不到上游逐字节副本：%s' % UPSTREAM)
        return 2
    got = sha256(UPSTREAM)
    if got != EXPECTED_SHA256:
        problems.append('上游副本 sha256 与记录不符：%s ≠ %s（上游改版了？请同步 docs/robot_models.md）'
                        % (got, EXPECTED_SHA256))

    up = ET.parse(UPSTREAM).getroot()
    # ② 上游一个 inertial 都没有
    n_inertial = len(up.findall('.//inertial'))
    if n_inertial != 0:
        problems.append('上游 URDF 里出现了 %d 条 <inertial> —— "必须由我们补惯性"这条结论要重写'
                        % n_inertial)

    # ③ 展开我们的 xacro（默认参数：云台角 0 ⇒ 与上游数值等价）
    xacro_bin = shutil.which('xacro')
    if xacro_bin is None:
        print('⚠️ 找不到 xacro（先 source /opt/ros/humble/setup.bash）；只做了 sha256/惯性检查')
        return 1 if problems else 0
    env = dict(os.environ)
    env.setdefault('ROS_LOG_DIR', os.path.join(REPO, '.tmp_roslog'))
    out = subprocess.run([xacro_bin, XACRO, 'turret_yaw_deg:=0', 'turret_pitch_deg:=0'],
                         capture_output=True, text=True, env=env)
    if out.returncode != 0:
        print('❌ xacro 展开失败：\n%s' % out.stderr)
        return 2
    mine = ET.fromstring(out.stdout)

    up_links = {l.get('name'): l for l in up.findall('link')}
    my_links = {l.get('name'): l for l in mine.findall('link')}
    up_joints = {j.get('name'): j for j in up.findall('joint')}
    my_joints = {j.get('name'): j for j in mine.findall('joint')}

    for name, l in up_links.items():
        if name not in my_links:
            problems.append('link [%s] 在上游有、在我们的模型里**没有**' % name)
            continue
        m = my_links[name]
        for tag in ('visual', 'collision'):
            ue, me = l.find(tag), m.find(tag)
            if (ue is None) != (me is None):
                problems.append('link [%s] 的 <%s> 存在性不一致（上游 %s / 我们 %s）'
                                % (name, tag, ue is not None, me is not None))
                continue
            if ue is None:
                continue
            uo, mo = origin_of(ue, 'origin'), origin_of(me, 'origin')
            compare('link', name, '%s.origin.xyz' % tag, uo[0], mo[0], problems)
            compare('link', name, '%s.origin.rpy' % tag, uo[1], mo[1], problems)
            ug, mg = geometry_of(ue), geometry_of(me)
            if ug != mg:
                if ug is None or mg is None or ug[0] != mg[0] or not close(ug[1], mg[1]):
                    problems.append('link [%s] 的 %s 几何不一致：上游 %s ≠ 我们 %s'
                                    % (name, tag, fmt(ug), fmt(mg)))
            uc, mc = color_of(ue), color_of(me)
            if isinstance(uc, list) and isinstance(mc, list):
                compare('link', name, '%s.material.rgba' % tag, uc, mc, problems)
            elif uc != mc:
                problems.append('link [%s] 的 %s 材质名不一致：%s ≠ %s' % (name, tag, fmt(uc), fmt(mc)))
    for name, j in up_joints.items():
        if name not in my_joints:
            problems.append('joint [%s] 在上游有、在我们的模型里**没有**' % name)
            continue
        m = my_joints[name]
        if j.get('type') != m.get('type'):
            problems.append('joint [%s] 类型不一致：%s ≠ %s' % (name, j.get('type'), m.get('type')))
        for tag in ('parent', 'child'):
            ue, me = j.find(tag), m.find(tag)
            uv = ue.get('link') if ue is not None else None
            mv = me.get('link') if me is not None else None
            if uv != mv:
                problems.append('joint [%s] 的 %s 不一致：%s ≠ %s' % (name, tag, uv, mv))
        uo, mo = origin_of(j, 'origin'), origin_of(m, 'origin')
        compare('joint', name, 'origin.xyz', uo[0], mo[0], problems)
        compare('joint', name, 'origin.rpy', uo[1], mo[1], problems)

    added_links = sorted(set(my_links) - set(up_links))
    added_joints = sorted(set(my_joints) - set(up_joints))
    n_gazebo = len(mine.findall('gazebo'))
    n_sensors = len(mine.findall('.//sensor'))
    n_plugins = len(mine.findall('.//plugin'))
    n_inertial_mine = len(mine.findall('.//inertial'))

    print('上游副本 sha256 = %s（期望 %s）%s'
          % (got, EXPECTED_SHA256, '✔' if got == EXPECTED_SHA256 else '✘'))
    print('上游 link %d / joint %d，<inertial> %d 条（结论：必须由我们补）'
          % (len(up_links), len(up_joints), n_inertial))
    print('我们的模型（xacro 默认参数展开后）：link %d / joint %d / <inertial> %d / '
          '<gazebo> %d / <sensor> %d / <plugin> %d'
          % (len(my_links), len(my_joints), n_inertial_mine, n_gazebo, n_sensors, n_plugins))
    print('我们加的 link  = %s' % (', '.join(added_links) or '（无）'))
    print('我们加的 joint = %s' % (', '.join(added_joints) or '（无）'))
    if args.print_added:
        for name in added_links:
            print('  · link %s: %s' % (name, ET.tostring(my_links[name], encoding='unicode')[:200]))
        for name in added_joints:
            print('  · joint %s: %s' % (name, ET.tostring(my_joints[name], encoding='unicode')[:200]))

    if problems:
        print('\n❌ 不一致 %d 处：' % len(problems))
        for p in problems:
            print('   - ' + p)
        return 1
    print('\n✅ 通过：上游 %d 个 link / %d 个 joint 的名字、类型、父子、xyz、rpy、几何、颜色'
          '在我们的模型里**数值相等**；差异只可能来自上面列出的"我们加的"元素。'
          % (len(up_links), len(up_joints)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
