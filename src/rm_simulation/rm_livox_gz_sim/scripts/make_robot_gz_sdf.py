#!/usr/bin/env python3
# =============================================================================
# make_robot_gz_sdf.py —— 把本仓的 URDF（robot11 等）变成 Gazebo Fortress 能跑的 SDF 模型
# =============================================================================
#
# 输入：一份 URDF 文本（由 launch 用**和 Classic 完全相同的 xacro 参数**生成）。
# 输出：<out-dir>/<model-name>/model.sdf + model.config（可直接被
#       `model://<model-name>` 解析，launch 会把这个 out-dir 放到
#       GZ_SIM_RESOURCE_PATH 的最前面）。
#
# 为什么不是"另写一份 robot11 的 SDF"：robot11 有 113 个真碰撞体 + 12 个 mesh，
# 手抄一定会和 Classic 漂移。这里让它成为**同一份 URDF 的派生**。
#
# 做四件事（每一步都对应一个实测过的 Fortress 坑）：
#  1. 注入 <gazebo reference="<joint>"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
#     —— sdformat 默认会把 fixed joint 折进父 link，`livox_frame` 会**消失**，
#        而我们的插件必须 `LinkByName("livox_frame")`。保留它，才有独立的传感器 link。
#  2. `ign sdf -p` 转 SDF（sdformat12）。它输出的 `<link><pose relative_to='<joint>'>`
#     再把 joint pose 折平进 link pose ⇒ `<link name="livox_frame"><pose>… -0.523599 0 0
#     </pose></link>`，也就是 B① 决策要的「30° 写在 SDF <link><pose> 里」。
#  3. 剥掉所有 Classic 插件/传感器：libros2_livox.so（Classic 的 ray 插件）、
#     libgazebo_ros_imu_sensor.so、libgazebo_ros_planar_move.so。Fortress 加载不了它们
#     （会报 "Failed to load system plugin" 并让模型插入失败）。
#  4. 加回我们的东西：两个 mid360 插件实例（CustomMsg + PointCloud2）、一个 gz `<sensor
#     type="imu">`（<topic> 显式给，插件订阅它转成 ROS 的 /livox/imu）。
#
# 用法：
#   make_robot_gz_sdf.py --urdf <in.urdf> --out-dir <dir> [--model-name robot11]
#                        [--csv <mid360.csv>] [--qos best_effort|reliable]
#                        [--tilt-rpy "-0.523598775598293 0 0"]
#        （--urdf - 表示从 stdin 读）
#   或： make_robot_gz_sdf.py --xacro <file.xacro> [xacro args as k:=v ...] --out-dir <dir>
#
# 打印：生成出来的 model.sdf 路径（最后一行），日志在 stderr。
# =============================================================================
import argparse
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

FIXED_JOINTS_TO_PRESERVE = ('body_to_livox', 'imu_joint')
CLASSIC_PLUGIN_PREFIXES = ('libros2_livox', 'libgazebo_ros_')
# 插件的固定部分（csv / qos 由参数填）
PLUGIN_TMPL = """
    <plugin filename="libmid360_system.so"
            name="mid360_simulation_plugin_ros2::Mid360System">
      <link_name>{link}</link_name>
      <ray_backend>embree</ray_backend>
      <map_key>default</map_key>
      <csv_file>{csv}</csv_file>
      <samples>30000</samples>
      <downsample>1</downsample>
      <update_rate>10</update_rate>
      <point_rate>300000</point_rate>
      <min_range>0.1</min_range>
      <max_range>40.0</max_range>
      <topic>{topic}</topic>
      <publish_pointcloud_type>{kind}</publish_pointcloud_type>
      <frame_id>{frame}</frame_id>
      <output_yaw>0.0</output_yaw>
      <node_name>{node}</node_name>
      <qos_reliability>{qos}</qos_reliability>
      <ignore_self>{ignore_self}</ignore_self>{imu}
    </plugin>"""


def log(msg):
    print('[make_robot_gz_sdf] ' + msg, file=sys.stderr)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def rpy_from_quat(q):
    x, y, z, w = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def fmt(vals):
    return ' '.join('%.12g' % v for v in vals)


def parse_pose(text):
    parts = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    while len(parts) < 6:
        parts.append(0.0)
    return parts[:3], parts[3:6]


def rel_to_model_pose(link, parents):
    """把 <pose relative_to='joint'> 折成相对 model 的绝对 pose。

    parents[link] = (parent_link, joint_pose_xyz, joint_pose_rpy)；只有一层（robot11
    是 base_link -> livox_frame / imu_link）。用矩阵/四元数做，不假设 rpy 分量能相加。
    """
    xyz, rpy = parse_pose(link.findtext('pose'))
    q = quat_from_rpy(*rpy)
    p = list(xyz)
    cur = link.get('name')
    guard = 0
    while cur in parents and guard < 64:
        parent, jxyz, jrpy = parents[cur]
        jq = quat_from_rpy(*jrpy)
        # 绝对 = 关节位姿 ∘ 子 link 相对关节的位姿（只加 R_j*p 会漏掉关节平移）
        rp = quat_rotate(jq, p)
        p = [jxyz[i] + rp[i] for i in range(3)]
        q = quat_mul(jq, q)
        cur = parent
        guard += 1
    return p, quat_from_rpy(*rpy_from_quat(q))


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--urdf', help="URDF 文件；'-' = stdin")
    src.add_argument('--xacro', help='xacro 文件（本脚本自己跑 xacro）')
    ap.add_argument('--xacro-arg', action='append', default=[],
                    help='透传给 xacro 的 k:=v（可重复）')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--model-name', default='rm_robot11_gz')
    ap.add_argument('--csv', default='', help='mid360.csv 绝对路径')
    ap.add_argument('--qos', default='best_effort', choices=['best_effort', 'reliable'])
    ap.add_argument('--tilt-rpy', default='',
                    help='启用「强制把传感器 link 拧到该 rpy」；默认空 = 用 URDF 关节里已有的姿态')
    ap.add_argument('--imu-frame', default='imu_link')
    ap.add_argument('--static', default='true')
    ap.add_argument('--ignore-self', default='true', choices=['true', 'false'],
                    help='true（默认）= 把自己所属 model 注册进 MapRegistry，'
                         'BVH 里剔除它 ⇒ 看不到自己的车体；false 只用于对照实验')
    args = ap.parse_args()

    if args.xacro:
        cmd = ['xacro', args.xacro] + list(args.xacro_arg)
        log('run: ' + ' '.join(cmd))
        urdf = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    elif args.urdf == '-':
        urdf = sys.stdin.read()
    else:
        urdf = open(args.urdf).read()

    # ---- 1. 注入 preserveFixedJoint（否则 livox_frame 会被折进 base_link 而消失）
    inject = ''.join(
        '\n  <gazebo reference="%s"><preserveFixedJoint>true</preserveFixedJoint></gazebo>'
        % j for j in FIXED_JOINTS_TO_PRESERVE)
    if '</robot>' not in urdf:
        sys.exit('URDF 里没有 </robot>，不像一份 URDF')
    urdf2 = urdf.replace('</robot>', inject + '\n</robot>')

    # ---- 2. ign sdf -p（用临时文件：`ign sdf -p -` 在 Fortress 上不接受 stdin）
    import tempfile
    tmpd = tempfile.mkdtemp(prefix='mkgzsdf_')
    tmp_urdf = os.path.join(tmpd, 'in.urdf')
    with open(tmp_urdf, 'w') as f:
        f.write(urdf2)
    log('run: ign sdf -p %s' % tmp_urdf)
    conv = subprocess.run(['ign', 'sdf', '-p', tmp_urdf],
                          capture_output=True, text=True)
    if conv.returncode != 0:
        log(conv.stdout[-2000:])
        sys.exit('ign sdf -p 失败：\n' + conv.stderr[-4000:])
    sdf_text = conv.stdout

    # ---- 3. 后处理
    root = ET.fromstring(sdf_text)
    model = root.find('model')
    if model is None:
        sys.exit('转换结果里没有 <model>')

    # 2a. 先收集 joint 的 pose（折平用），再把 fixed joint 全部删掉
    parents = {}
    links = {l.get('name'): l for l in model.findall('link')}
    for j in list(model.findall('joint')):
        jxyz, jrpy = parse_pose(j.findtext('pose'))
        child = (j.find('child') is not None and j.find('child').text or '').strip()
        parent = (j.find('parent') is not None and j.find('parent').text or '').strip()
        if child and parent:
            parents[child] = (parent, jxyz, jrpy)
    for j in list(model.findall('joint')):
        model.remove(j)

    # 2b. link pose 折成绝对（去掉 relative_to）
    for name, link in links.items():
        pose = link.find('pose')
        abs_p, abs_q = rel_to_model_pose(link, parents)
        rpy = rpy_from_quat(abs_q)
        if pose is None:
            pose = ET.SubElement(link, 'pose')
        if pose.get('relative_to') is not None:
            del pose.attrib['relative_to']
        pose.text = fmt(list(abs_p) + list(rpy))

    # 2c. 剥掉 Classic 插件块 + ray 传感器；IMU 传感器保留但换成 gz 的写法
    dropped = []
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            stype = sensor.get('type')
            if stype == 'ray' or stype == 'gpu_ray':
                for pl in sensor.findall('plugin'):
                    dropped.append(pl.get('filename'))
                link.remove(sensor)
                continue
            # imu：清掉 Classic 插件，加 <topic>
            for pl in list(sensor.findall('plugin')):
                dropped.append(pl.get('filename'))
                sensor.remove(pl)
            if sensor.find('topic') is None:
                t = ET.SubElement(sensor, 'topic')
                t.text = '/mid360/imu_gz'
            if sensor.find('always_on') is None:
                ao = ET.SubElement(sensor, 'always_on')
                ao.text = 'true'
    for pl in list(model.findall('plugin')):
        fn = pl.get('filename') or ''
        if fn.startswith(CLASSIC_PLUGIN_PREFIXES):
            dropped.append(fn)
            model.remove(pl)
    if dropped:
        log('剥掉的 Classic 插件/传感器插件：' + ', '.join(dropped))

    # 2d. 找传感器 link（livox_frame / 其关节的 child）
    link_name = None
    for cand in ('livox_frame', 'livox_link', 'lidar_link'):
        if cand in links:
            link_name = cand
            break
    if link_name is None:
        sys.exit('找不到传感器 link（livox_frame / livox_link / lidar_link）：'
                 + ', '.join(links))
    log('sensor link = %s' % link_name)

    # 2e. --tilt-rpy：强制把该 link 的 roll 写成指定值（B① 的字面实现）
    if args.tilt_rpy:
        link = links[link_name]
        p, _ = parse_pose(link.findtext('pose'))
        tilt = [float(v) for v in args.tilt_rpy.split()]
        while len(tilt) < 3:
            tilt.append(0.0)
        link.find('pose').text = fmt(p + tilt[:3])
        log('强制 %s 的 rpy = %s' % (link_name, fmt(tilt[:3])))

    # 2f. 静态化（无轮子关节 ⇒ 不该让物理引擎去"掉落"它）
    if model.find('static') is None:
        st = ET.SubElement(model, 'static')
        st.text = args.static
    else:
        model.find('static').text = args.static

    # 2g/2h. 序列化后按字符串插入两个实例（ET 对注释/tail 的处理很容易出错，
    #        字符串插入更直白），顺便把版本钉到 1.9（sdformat12 的上限；
    #        1.10 会让整个世界加载失败，实测见 livox_sensor_prior_art.md §10.4.6）。
    root.set('version', '1.9')
    ET.indent(root, space='  ')
    xml = ET.tostring(root, encoding='utf-8').decode('utf-8')
    # 顶层是 <sdf><model>…</model></sdf>：插件必须插进 <model>，不是 <sdf>。
    imu_block = ('\n      <imu_topic>/livox/imu</imu_topic>'
                 '\n      <imu_frame_id>%s</imu_frame_id>'
                 '\n      <gz_imu_topic>/mid360/imu_gz</gz_imu_topic>'
                 '\n      <publish_imu>true</publish_imu>' % args.imu_frame)
    plugins = (
        '\n    <!-- HzMi: 双发布实例 #1 —— 只发 CustomMsg（LIO 用 /livox/lidar） -->'
        + PLUGIN_TMPL.format(link=link_name, csv=args.csv, topic='/livox/lidar',
                             kind='3', frame=link_name, node='mid360_gz_custom',
                             qos=args.qos, ignore_self=args.ignore_self,
                             imu=imu_block)
        + '\n    <!-- HzMi: 双发布实例 #2 —— 只发 PointCloud2（linefit 用 /livox/lidar/pointcloud） -->'
        + PLUGIN_TMPL.format(link=link_name, csv=args.csv,
                             topic='/livox/lidar/pointcloud', kind='2',
                             frame=link_name, node='mid360_gz_pc2', qos=args.qos,
                             ignore_self=args.ignore_self,
                             imu='\n      <publish_imu>false</publish_imu>')
    )
    close = xml.rindex('</model>')
    xml = xml[:close] + plugins + '\n  </model>' + xml[close + len('</model>'):]

    out_model_dir = os.path.join(args.out_dir, args.model_name)
    os.makedirs(out_model_dir, exist_ok=True)
    out_sdf = os.path.join(out_model_dir, 'model.sdf')
    with open(out_sdf, 'w') as f:
        f.write("<?xml version='1.0'?>\n" + xml + '\n')
    with open(os.path.join(out_model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version="1.0" ?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>HzMi</name><email>hzmi@example.com</email></author>
  <description>在运行期由 scripts/make_robot_gz_sdf.py 从本仓 URDF 生成（Gazebo Fortress）。
真源是 URDF；这里只做 Fortress 化 + 插入我们的 MID-360 传感器实例。生成时间相关的
内容一律不写死，免得多份副本漂移。</description>
</model>
""" % args.model_name)
    print(out_sdf)
    log('wrote %s (+ model.config)' % out_sdf)


if __name__ == '__main__':
    main()
