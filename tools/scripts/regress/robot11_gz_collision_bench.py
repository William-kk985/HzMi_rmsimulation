#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot11 / base_link 碰撞表示代价实测（Phase 1）

问：`base_link.STL`（2,078,226 三角面）能不能当 Gazebo Classic 的 `<collision>`？代价多少？

做法（极简世界 + 单个 spawn 的模型，无头、隔离）：
  1. 起 gzserver（世界 = 只有 ground_plane + sun，**不含机器人**）；用 `gz stats` 轮询到世界就绪 ⇒ `t_world_ready_s`
  2. 采样 5 s 基线 RTF/RSS
  3. `gz model -f <variant>.sdf -m robot11` spawn ⇒ `t_spawn_s`（factory 服务往返墙钟）
  4. spawn 后再采样 10 s ⇒ `rtf_after` / `sim_advance_10s` / RSS 峰值
  5. 收工（**只按 PID 杀**，不用 pkill -f，避免误杀自己的 shell）

隔离：`HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、`unset DISPLAY`、
`GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models`（**不设它 gzserver 会去连 models.gazebosim.org 并挂住** —— 实测）。

⚠️ **不要用 `gzserver --iters N`**：本机实测它会一直不退出（世界在跑但进程不结束）；
   计时改用固定墙钟窗口 + `gz stats`。

用法：
  python3 tools/scripts/regress/robot11_gz_collision_bench.py --out .tmp_robot11/bench/bench.json
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../..'))
MESHES = os.path.join(REPO, 'src/rm_simulation/robot11_description/meshes')
GEN = os.path.join(MESHES, 'generated')

# 轮心位置：**必须**用 FK 实测值（= 轮 mesh 包围盒中心在 body 系下的位置），
# 不能用 j2..j5 的 origin z（0.05735 是**转向关节**高度，轮心在其下方 0.102 m，
# 用错会让底盘"趴"在地上、量出来的 RTF 是盒底蹭地的代价而不是悬空行驶的代价）。
GEO = json.load(open(os.path.join(REPO, '.tmp_robot11/derived/robot11_geometry.json')))
WHEEL_CENTERS = []
for _nm in ('l6', 'l7', 'l8', 'l9'):
    _b = GEO['links'][_nm]['bbox_body_m']
    WHEEL_CENTERS.append([(_b['min'][i] + _b['max'][i]) / 2.0 for i in range(3)])
WHEEL_R = 0.058
WHEEL_W = 0.0452

WORLD = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="bench">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode><solver><type>quick</type><iters>50</iters></solver></ode>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><shadows>false</shadows></scene>
  </world>
</sdf>
"""


def body_link(visual_mesh, collision_xml):
    """底盘 link：惯性逐字取用户 URDF 的 body。"""
    if visual_mesh is None:
        vis = ''
    else:
        vis = ('<visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
               '<mesh filename="file://%s"/></geometry></visual>' % visual_mesh)
    return """
  <link name="body">
    <inertial><origin xyz="0.00436933539925447 0.0121729882941287 0.0624624952683773" rpy="0 0 0"/>
      <mass value="6.98390841611154"/>
      <inertia ixx="0.012450840516849" ixy="8.34803982990868E-06" ixz="2.47736216819592E-06"
               iyy="0.0180415546421503" iyz="6.50018190294825E-06" izz="0.0214898895578463"/></inertial>
    %s
    %s
  </link>""" % (vis, collision_xml)


def wheel_links(mode='true_center'):
    """四个轮：cylinder 碰撞（半径/宽度由轮 mesh 的包围盒得到）。"""
    out = []
    if mode == 'joint_z':   # 故意让底盘"趴"在地上：轮心抬到转向关节高度 ⇒ 底盘底面包络贴地
        centers = [[sx * 0.182134559672904, sy * 0.182134559672906, 0.05735]
                   for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    else:
        centers = WHEEL_CENTERS
    for i, c in enumerate(centers):
        out.append("""
  <link name="wheel_%d">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.15"/>
      <inertia ixx="0.00028" ixy="0" ixz="0" iyy="0.000145" iyz="0" izz="0.000145"/></inertial>
    <collision><origin xyz="0 0 0" rpy="1.5707963267949 0 0"/><geometry>
      <cylinder radius="%f" length="%f"/></geometry></collision>
    <visual><origin xyz="0 0 0" rpy="1.5707963267949 0 0"/><geometry>
      <cylinder radius="%f" length="%f"/></geometry></visual>
  </link>
  <joint name="jw%d" type="continuous">
    <origin xyz="%f %f %f" rpy="0 0 0"/><parent link="body"/><child link="wheel_%d"/>
    <axis xyz="0 1 0"/>
  </joint>""" % (i, WHEEL_R, WHEEL_W, WHEEL_R, WHEEL_W, i, c[0], c[1], c[2], i))
    return ''.join(out)


def make_model(variant):
    """生成 URDF（不是 SDF）：① 关节语义无歧义 ② 与 Phase 2 的 spawn 路径一致
    （SDF 里 `type="continuous"` 会 LoadJoint Failed，实测）。"""
    full = os.path.join(MESHES, 'base_link.STL')
    dec = os.path.join(GEN, 'base_link_collision.stl')
    vis = full if variant['visual'] == 'full' else (None if variant['visual'] == 'none' else dec)
    c = variant['collision']
    if c['kind'] == 'mesh':
        cm = full if c['mesh'] == 'full' else dec
        col = ('<collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
               '<mesh filename="file://%s"/></geometry></collision>' % cm)
    elif c['kind'] == 'boxes':
        parts = []
        for i, b in enumerate(c['boxes']):
            parts.append('<collision><origin xyz="%f %f %f" rpy="0 0 0"/><geometry>'
                         '<box size="%f %f %f"/></geometry></collision>'
                         % (b['center'][0], b['center'][1], b['center'][2],
                            b['size'][0], b['size'][1], b['size'][2]))
        col = ''.join(parts)
    else:
        col = ''
    return ('<?xml version="1.0" ?><robot name="robot11">'
            + body_link(vis, col) + wheel_links(variant.get('wheels', 'true_center')) + '</robot>')


# --------------------------------------------------------------------------- #
def gz_stats(env, duration=2):
    try:
        r = subprocess.run(['gz', 'stats', '-d', str(duration)], env=env,
                           capture_output=True, text=True, timeout=duration + 8)
    except subprocess.TimeoutExpired:
        return None
    lines = [l for l in r.stdout.splitlines() if 'Factor[' in l]
    if not lines:
        return None
    m = re.search(r'Factor\[([\d.]+)\]\s*SimTime\[([\d.]+)\]\s*RealTime\[([\d.]+)\]\s*Paused\[(\w)\]',
                  lines[-1])
    if not m:
        return None
    return {'factor': float(m.group(1)), 'sim': float(m.group(2)),
            'real': float(m.group(3)), 'paused': m.group(4)}


def rss_kb(pid):
    try:
        with open('/proc/%d/status' % pid) as f:
            for l in f:
                if l.startswith('VmRSS:'):
                    return int(l.split()[1])
    except Exception:
        pass
    return None


def run_variant(name, sdf_xml, workdir, env, tag, spawn_wait=90, sample=10):
    os.makedirs(workdir, exist_ok=True)
    sdf = os.path.join(workdir, name + '.urdf')
    world = os.path.join(workdir, name + '.world')
    with open(sdf, 'w') as f:
        f.write(sdf_xml)
    with open(world, 'w') as f:
        f.write(WORLD)
    log = os.path.join(workdir, name + '.gzserver.log')

    res = {'variant': name, 'model_file': sdf, 'model_bytes': os.path.getsize(sdf)}
    env = dict(env)
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:%d' % (11500 + abs(hash(tag)) % 400)

    t0 = time.time()
    with open(log, 'w') as lf:
        srv = subprocess.Popen(['gzserver', '--verbose', world], env=env,
                               stdout=lf, stderr=subprocess.STDOUT)
    res['pid'] = srv.pid
    # 等世界就绪
    ready = None
    while time.time() - t0 < 90:
        if srv.poll() is not None:
            res['error'] = 'gzserver 提前退出 code=%s' % srv.returncode
            return res
        st = gz_stats(env, 1)
        if st:
            ready = time.time() - t0
            break
    res['t_world_ready_s'] = round(ready, 2) if ready else None
    if ready is None:
        res['error'] = '世界未就绪'
        srv.kill()
        return res

    res['rss_baseline_kb'] = rss_kb(srv.pid)
    b = gz_stats(env, 3)
    res['rtf_before'] = b['factor'] if b else None
    res['sim_before'] = b['sim'] if b else None

    # spawn
    ts = time.time()
    try:
        sp = subprocess.run(['gz', 'model', '-f', sdf, '-m', 'robot11', '-x', '0', '-y', '0', '-z', '0.30'],
                            env=env, capture_output=True, text=True, timeout=spawn_wait)
        res['spawn_rc'] = sp.returncode
        res['spawn_stderr_tail'] = (sp.stderr or '').strip().splitlines()[-3:]
        res['spawn_stdout_tail'] = (sp.stdout or '').strip().splitlines()[-3:]
    except subprocess.TimeoutExpired:
        res['spawn_rc'] = 'timeout(%ds)' % spawn_wait
    res['t_spawn_s'] = round(time.time() - ts, 2)
    res['rss_after_spawn_kb'] = rss_kb(srv.pid)

    # spawn 后采样
    sims, factors, peak = [], [], res['rss_after_spawn_kb'] or 0
    t_end = time.time() + sample
    a = None
    while time.time() < t_end:
        st = gz_stats(env, 2)
        if st:
            if a is None:
                a = st
            factors.append(st['factor'])
            sims.append(st['sim'])
        r = rss_kb(srv.pid)
        if r:
            peak = max(peak, r)
        if srv.poll() is not None:
            res['error'] = 'gzserver 在采样期间退出 code=%s' % srv.returncode
            break
    res['rss_peak_kb'] = peak
    res['rtf_after'] = round(sum(factors) / len(factors), 3) if factors else None
    res['rtf_after_min'] = round(min(factors), 3) if factors else None
    res['sim_advance_in_window_s'] = round(sims[-1] - a['sim'], 3) if (sims and a) else None
    res['window_wall_s'] = round(time.time() - (t_end - sample), 2)

    # 静置后的位姿（证明模型"站在地上"而不是穿地/爆掉）
    try:
        pp = subprocess.run(['gz', 'model', '-m', 'robot11', '-p'], env=env,
                            capture_output=True, text=True, timeout=20)
        res['final_pose_xyzrpy'] = pp.stdout.strip().splitlines()[-1:] or None
    except subprocess.TimeoutExpired:
        res['final_pose_xyzrpy'] = 'timeout'

    # 收工：只按 PID 杀
    srv.terminate()
    try:
        srv.wait(timeout=10)
    except subprocess.TimeoutExpired:
        srv.kill()
    res['rss_end_kb'] = None
    res['gzserver_log_tail'] = open(log).read().strip().splitlines()[-4:]
    res['ok'] = 'error' not in res and res.get('spawn_rc') == 0
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='src/rm_simulation/robot11_description/inventory/gz_collision_bench.json')
    ap.add_argument('--only', default='')
    ap.add_argument('--sample', type=int, default=10)
    ap.add_argument('--variants-from', default='src/rm_simulation/robot11_description/inventory/collision_assets.json')
    a = ap.parse_args()

    with open(os.path.join(REPO, a.variants_from)) as f:
        assets = json.load(f)
    boxes = assets['boxes']['base_link']['boxes']

    variants = {
        # A：原网格当碰撞（问题现场）
        'A_fullmesh': {'visual': 'full', 'collision': {'kind': 'mesh', 'mesh': 'full'}},
        # B：原网格视觉 + 4 个 box 碰撞（推荐）
        'B_boxes': {'visual': 'full', 'collision': {'kind': 'boxes', 'boxes': boxes}},
        # C：原网格视觉 + 抽稀网格碰撞（3000 面）
        'C_decimated': {'visual': 'full', 'collision': {'kind': 'mesh', 'mesh': 'dec'}},
        # D：连视觉也抽稀（隔离"视觉网格"的代价）
        'D_decvis_boxes': {'visual': 'dec', 'collision': {'kind': 'boxes', 'boxes': boxes}},
        # E：基线（只有四个轮子，无底盘网格）—— 参考下限
        'E_nowheelchassis': {'visual': 'none', 'collision': {'kind': 'none'}},
        # A2/B2：「底盘真的蹭到东西」的工况（轮心抬到转向关节高度 ⇒ 底盘贴地）
        #   稳态悬空时 A 与 B 一样快（没有接触对被求值）；**一有接触**才是原网格的真正代价
        'A2_contact_fullmesh': {'visual': 'full', 'wheels': 'joint_z',
                                'collision': {'kind': 'mesh', 'mesh': 'full'}},
        'B2_contact_boxes': {'visual': 'full', 'wheels': 'joint_z',
                             'collision': {'kind': 'boxes', 'boxes': boxes}},
        'C2_contact_decimated': {'visual': 'full', 'wheels': 'joint_z',
                                 'collision': {'kind': 'mesh', 'mesh': 'dec'}},
    }
    if a.only:
        keep = set(a.only.split(','))
        variants = {k: v for k, v in variants.items() if k in keep}

    env = dict(os.environ)
    env.pop('DISPLAY', None)
    env['GAZEBO_MODEL_PATH'] = '/usr/share/gazebo-11/models'
    env['ROS_DOMAIN_ID'] = '77'
    tag = 'robot11bench'
    env['HOME'] = '/tmp/gzhome-' + tag
    os.makedirs(env['HOME'], exist_ok=True)
    workdir = os.path.join(REPO, '.tmp_robot11/bench')

    out = {'tool': __file__, 'when': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
           'isolation': {'HOME': env['HOME'], 'ROS_DOMAIN_ID': env['ROS_DOMAIN_ID'],
                         'GAZEBO_MODEL_PATH': env['GAZEBO_MODEL_PATH'], 'DISPLAY': 'unset'},
           'world': 'ground_plane + sun, ODE quick/50, max_step_size 1 ms, RT update rate 1000',
           'note': 'gzserver --iters 在本机会挂住 ⇒ 用固定墙钟窗口 + gz stats 计时；不用 pkill -f。',
           'variants': {}}
    for nm, v in variants.items():
        sys.stderr.write('[bench] %s ...\n' % nm)
        sys.stderr.flush()
        sdf = make_model(v)
        r = run_variant(nm, sdf, workdir, env, tag, spawn_wait=180, sample=a.sample)
        out['variants'][nm] = r
        sys.stderr.write('[bench]   ready=%s spawn=%ss rtf_before=%s rtf_after=%s rss=%sMB peak=%sMB ok=%s %s\n'
                         % (r.get('t_world_ready_s'), r.get('t_spawn_s'), r.get('rtf_before'),
                            r.get('rtf_after'),
                            (r.get('rss_after_spawn_kb') or 0) // 1024,
                            (r.get('rss_peak_kb') or 0) // 1024, r.get('ok'), r.get('error', '')))
    os.makedirs(os.path.dirname(os.path.abspath(os.path.join(REPO, a.out))), exist_ok=True)
    with open(os.path.join(REPO, a.out), 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    sys.stderr.write('[bench] -> %s\n' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
