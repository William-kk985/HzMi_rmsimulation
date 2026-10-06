#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图存档的「场地隔离」守卫 + 元数据 sidecar（**纯 Python，不依赖 ROS**）。

谁在用这个文件（三处共用同一份判定，避免三套逻辑各自漂移）：
  1. `launch/bringup_sim.launch.py`  —— `mode:=mapping/slam_nav` 起 slam_toolbox 前，
     在 launch 期（OpaqueFunction）判定"续建 / 从零 / 拒绝"，拒绝时直接抛错终止 launch；
  2. `src/rm_perception/cloud_accumulator`（3D 点云累加器节点）—— 启动时 `load`、
     服务 `save` 前都过同一个 `verify()`；
  3. `tools/scripts/mapping/map_archive.sh` —— `save` 落盘前 / `info` / `list` / `adopt`。

要解决的问题（用户原话）：
  「能不能就是我在上次基础上继续建，手动指定一个地图名字去覆盖之类的」
  「以后换地图换场地会不会有干扰」

隔离的物理依据（不是洁癖，是坐标系的硬事实）：
  本工程的 `map` 系 = **出生点相对系**（原点 = 机器人出生点，见 docs/worlds.md §3~§4）。
  换 world ⇒ 出生点变了 ⇒ 同一份 posegraph 在新场地里天然错位；
  两个场地的位姿图叠在一起 = 假墙 + 回环错配 + 之后所有定位/导航全部报废。
  所以：**存档必须自带"我属于哪个 world / 哪个出生点"的记录（sidecar），
  续建前必须核对，核对不上默认拒绝。**

sidecar 文件：`<存档基名>.meta.yaml`（与 `<基名>.posegraph` / `<基名>.data` / `<基名>.pcd` 同目录）

判定分级（`verify()` 的 code）：
  · fresh                 —— 存档文件不存在 ⇒ 从零建（不算错）
  · resume                —— manifest 与当前 world/spawn 一致 ⇒ 允许续建
  · override              —— 不一致但显式给了 `allow_mismatch` ⇒ 允许，但每条日志都带"已忽略隔离检查"
  · refused_no_manifest   —— 存档在、没有 sidecar（旧版存档 / 手工拷贝）⇒ 拒绝
  · refused_kind          —— sidecar 说它是另一种类型（.posegraph vs .pcd）⇒ 拒绝
  · refused_world         —— world 不同 ⇒ 拒绝
  · refused_spawn         —— 同一 world 但出生点记录不同（地图原点变了）⇒ 拒绝
  · refused_missing_data  —— posegraph 在但 .data 缺（slam_toolbox 读不了）⇒ 拒绝

**软警告**（不拒绝，但会打印）：`resolution` 与记录不同、`map_start_pose` 与记录不同。
  这两项改了不一定错（用户可能在纠正），但一定要让用户看见。
"""
from __future__ import annotations

import argparse
import ast
import datetime
import json
import os
import sys
import xml.etree.ElementTree as ET

try:                                    # PyYAML 在所有 ROS 2 环境里都有；没有也能跑（退化到极简解析）
    import yaml
except Exception:                       # pragma: no cover
    yaml = None

# ---------------------------------------------------------------------------
# 场地出生点表 —— **单一事实来源在本文件**
# 原始定义在 hzmi_rm_simulation/launch/rm_simulation.launch.py::get_world_config()
# （那边是"真正 spawn 用的值"）。本表是给"存档隔离"用的镜像；
# 两边一旦漂移，"出生点比对"就会永远报警/永远静默，所以提供
#   python3 map_asset_guard.py spawn-table --check
# 直接解析那份 launch 文件核对（--check 失败 ⇒ 必须同步，见 docs/continue_mapping.md §4）。
# ---------------------------------------------------------------------------
WORLD_SPAWN = {
    'RMUC':     {'x': 6.35,   'y': 7.6,   'z': 0.2, 'yaw': 0.0},
    'RMUL':     {'x': 4.3,    'y': 3.35,  'z': 0.2, 'yaw': 0.0},
    'RMUL2026': {'x': 4.3,    'y': 3.35,  'z': 0.2, 'yaw': 0.0},
    'RMUC2026': {'x': 10.925, 'y': 2.525, 'z': 0.2, 'yaw': 0.0},
}

SPAWN_EPS = 1e-6
KIND_POSEGRAPH = 'posegraph'
KIND_PCD = 'pcd'

FRAME_CONVENTION = ('map 系 = 出生点相对系（原点 = 机器人出生点，x 前 y 左，单位 m/rad）；'
                    'odom = LIO 里程计系；base = base_link。'
                    '因此同一 world 只要出生点不变，map(0,0,0) 恒等于出生点。')

# 记录进 manifest 的包版本（写存档那一刻）
VERSION_PACKAGES = ('rm_nav_bringup', 'slam_toolbox', 'rm_perception', 'cloud_accumulator')


# ============================================================ 目录解析
def _looks_like_share_dir(path, sub):
    return os.path.isdir(os.path.join(path, sub))


def canonical_asset_dir(share_dir, sub):
    """返回"实际该读写哪个目录"：优先**仓库源码树**，其次 install/share。

    为什么需要它（本仓库的实测坑）：
      `colcon build --symlink-install` 的 `share/rm_nav_bringup/map/` 是一个**真目录**，
      里面每个文件是**指向 src 的单独符号链接**（不是整目录软链）。
      ⇒ 运行期**新建**的文件（`map/<name>.posegraph`、`map/<name>.meta.yaml`）在 install 里
        **根本不存在**，直到你重新 `colcon build --packages-select rm_nav_bringup` 才会补链接。
      如果 launch 从 `share/.../map/` 读、脚本往 `src/.../map/` 写，就会出现
      「明明存了、下次却当没存」这种最难查的静默失败。
      所以两边都走本函数：**只要发现 share 目录里有指向别处的符号链接，就以那份真实目录为准**。
      非 symlink-install（整份拷贝）时挑不出链接 ⇒ 退回 share 目录本身（此时写 share 也能被读到）。
    """
    share_dir = os.path.abspath(share_dir)
    sub_dir = os.path.join(share_dir, sub)
    try:
        for entry in sorted(os.listdir(sub_dir)):
            p = os.path.join(sub_dir, entry)
            if os.path.islink(p):
                real = os.path.dirname(os.path.realpath(p))
                if os.path.isdir(real) and os.path.realpath(real) != os.path.realpath(sub_dir):
                    return real
    except OSError:
        pass
    return sub_dir


def find_share_dir(pkg='rm_nav_bringup'):
    """不依赖 ament_index_python 地找包 share 目录（shell 脚本里也能用）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        pass
    for prefix in (os.environ.get('AMENT_PREFIX_PATH') or '').split(os.pathsep):
        cand = os.path.join(prefix, 'share', pkg)
        if os.path.isdir(cand):
            return cand
    return None


def map_dir(share_dir=None):
    share_dir = share_dir or find_share_dir('rm_nav_bringup')
    if not share_dir:
        raise RuntimeError('找不到 rm_nav_bringup 的 share 目录（AMENT_PREFIX_PATH 里没有？先 source install/setup.bash）')
    return canonical_asset_dir(share_dir, 'map')


def pcd_dir(share_dir=None):
    share_dir = share_dir or find_share_dir('rm_nav_bringup')
    if not share_dir:
        raise RuntimeError('找不到 rm_nav_bringup 的 share 目录（AMENT_PREFIX_PATH 里没有？先 source install/setup.bash）')
    return canonical_asset_dir(share_dir, 'PCD')


def archive_file(base, kind):
    return base + ('.posegraph' if kind == KIND_POSEGRAPH else '.pcd')


def companion_files(base, kind):
    if kind == KIND_POSEGRAPH:
        return [base + '.posegraph', base + '.data', base + '.meta.yaml']
    return [base + '.pcd', base + '.meta.yaml']


def manifest_path(base):
    return base + '.meta.yaml'


# ============================================================ 版本信息
def pkg_version(pkg):
    share = find_share_dir(pkg)
    if not share:
        return None
    pxml = os.path.join(share, 'package.xml')
    try:
        root = ET.parse(pxml).getroot()
        v = root.find('version')
        return v.text.strip() if v is not None and v.text else None
    except Exception:
        return None


def ros_versions(extra_packages=()):
    pkgs = {}
    for p in list(VERSION_PACKAGES) + list(extra_packages):
        v = pkg_version(p)
        if v:
            pkgs[p] = v
    return {
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'packages': pkgs,
    }


def now_iso():
    return datetime.datetime.now().astimezone().replace(microsecond=0).isoformat()


# ============================================================ 出生点
def world_spawn(world):
    """当前 world 的出生点（spawn）。表里没有 ⇒ None（未知 world 只比 world 名，不比出生点）。"""
    return WORLD_SPAWN.get(world)


def spawn_tuple(sp):
    if not sp:
        return None
    return (float(sp.get('x', 0.0)), float(sp.get('y', 0.0)),
            float(sp.get('z', 0.0)), float(sp.get('yaw', 0.0)))


def spawn_str(sp):
    t = spawn_tuple(sp)
    if t is None:
        return '(未记录)'
    return 'x=%.3f y=%.3f z=%.3f yaw=%.3f' % t


def spawn_diff(a, b):
    ta, tb = spawn_tuple(a), spawn_tuple(b)
    if ta is None or tb is None:
        return None
    return max(abs(x - y) for x, y in zip(ta, tb))


# ============================================================ manifest 读写
def read_manifest(base):
    p = manifest_path(base)
    if not os.path.isfile(p):
        return None
    try:
        with open(p, 'r') as f:
            data = yaml.safe_load(f) if yaml else _mini_yaml(f.read())
        return data if isinstance(data, dict) else None
    except Exception as e:                                     # sidecar 坏掉 = 不可信 = 当没有
        return {'__parse_error__': str(e)}


def _mini_yaml(text):                                          # pragma: no cover
    """没有 PyYAML 时的极简兜底：只认 `key: value` 与一层缩进字典，够读我们自己写的 sidecar。"""
    out = {}
    for line in text.splitlines():
        line = line.split('#')[0].rstrip()
        if not line.strip() or ':' not in line:
            continue
        k, _, v = line.partition(':')
        out[k.strip()] = v.strip()
    return out


def write_manifest(base, name, kind, world, files, map_start_pose=None,
                   resolution=None, spawn=None, extra=None):
    """写/更新 sidecar。已存在则保留 created_at，只刷新 updated_at。"""
    old = read_manifest(base) or {}
    created = old.get('created_at') or now_iso()
    man = {
        'name': name,
        'kind': kind,
        'world': world,
        'created_at': created,
        'updated_at': now_iso(),
        'spawn_pose': spawn if spawn is not None else world_spawn(world),
        'map_start_pose': [float(v) for v in (map_start_pose or [0.0, 0.0, 0.0])],
        'resolution': resolution,
        'frames': {'map': 'map', 'odom': 'odom', 'base': 'base_link',
                   'convention': FRAME_CONVENTION},
        'files': [os.path.basename(f) for f in files],
        'file_sizes': {os.path.basename(f): (os.path.getsize(f) if os.path.isfile(f) else None)
                       for f in files},
        'versions': ros_versions(),
        'written_by': 'rm_nav_bringup/scripts/map_asset_guard.py',
    }
    if extra:
        man.update(extra)
    if yaml:
        with open(manifest_path(base), 'w') as f:
            yaml.safe_dump(man, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    else:                                                       # pragma: no cover
        with open(manifest_path(base), 'w') as f:
            for k, v in man.items():
                f.write('%s: %s\n' % (k, v))
    return man


# ============================================================ 拒绝消息（唯一出处）
def _refuse_header():
    return '[map_archive] ❌ 拒绝加载存档（world 隔离保护）'


def refusal_message(code, base, name, kind, world, man, cur_spawn):
    """返回**可操作**的中文拒绝消息（launch / 累加器 / shell 用同一份文本）。"""
    art = archive_file(base, kind)
    lines = [_refuse_header(),
             '  · 存档名        : %s（%s）' % (name, '位姿图 posegraph' if kind == KIND_POSEGRAPH else '3D 点云 PCD'),
             '  · 存档文件      : %s%s' % (art, '（+ .data）' if kind == KIND_POSEGRAPH else ''),
             '  · sidecar       : %s' % manifest_path(base)]
    if man and man.get('world'):
        lines.append('  · 存档记录 world: %s（%s 写）' % (man.get('world'), man.get('updated_at') or man.get('created_at') or '时间未知'))
    else:
        lines.append('  · 存档记录 world: （无记录）')
    lines.append('  · 本次 world    : %s' % world)
    lines.append('  · 存档记录 spawn: %s' % spawn_str((man or {}).get('spawn_pose')))
    lines.append('  · 本次 spawn    : %s' % spawn_str(cur_spawn))

    reason = {
        'refused_no_manifest': '这份存档**没有** sidecar（%s）—— 无法确认它属于哪个 world（旧版存档 / 手工拷贝）。'
                               % os.path.basename(manifest_path(base)),
        'refused_kind': 'sidecar 说这份存档的类型是 `%s`，与本次要加载的 `%s` 不符。'
                        % ((man or {}).get('kind'), kind),
        'refused_world': '存档记录的 world 与本次 `world:=` 不是同一个场地。',
        'refused_spawn': '同一个 world，但存档记录的**出生点**与本次不同 —— 出生点变了 ⇒ `map` 系原点变了，'
                         '旧图在新原点下必然整体错位。',
        'refused_missing_data': '位姿图 `%s.posegraph` 在，但配套的 `%s.data` 不在（slam_toolbox 读不了半份存档）。'
                                % (name, name),
    }.get(code, '存档与本次启动的场地/出生点不一致。')
    lines.append('  · 为什么必须拒绝: %s' % reason)
    lines += [
        '  为什么这事很严重：本工程的 `map` 系是**出生点相对系**（原点=出生点）；',
        '  把 A 场地的位姿图/点云先验加载到 B 场地 = 把两场比赛的地图叠在一起 ⇒',
        '  假墙、回环错配、之后所有定位/导航全部报废（而且要等到跑歪了才发现）。',
        '  四选一（都在**启动命令**上，不用改代码）：',
        '    ① 换个名字（最推荐，各场地各一份存档）：map_name:=%s_%s' % (world, name),
        '    ② 删掉或改名这份存档（连同 .data / .meta.yaml 一起）：',
        '         mv %s %s.bak   # .posegraph；对 .data / .meta.yaml 同样处理' % (art, art),
        '    ③ 本次不要续建、从零开始建图：加 map_autocontinue:=False',
        '    ④ 你确认这份存档确实属于当前 world（例如只是改了出生点、或刚手工搬过来）：',
        '         显式承担风险 ⇒ 加 map_allow_world_mismatch:=True（默认 False）',
    ]
    if code == 'refused_no_manifest' and kind == KIND_POSEGRAPH:
        lines.append('       · 或者先给它补一份 sidecar（会再问一次 world，确认后才写）：'
                     ' tools/scripts/mapping/map_archive.sh adopt --name %s --world %s' % (name, world))
    lines.append('  存档约定与完整流程见 docs/continue_mapping.md')
    return '\n'.join(lines)


# ============================================================ 核心判定
def verify(base, name, world, kind=KIND_POSEGRAPH, allow_mismatch=False,
           map_start_pose=None, resolution=None):
    """判定这次能不能加载 `base` 这份存档。返回 dict（不要抛异常，调用方自己决定怎么处理）。

    返回：{'code', 'may_load', 'message', 'manifest', 'archive'}
    """
    art = archive_file(base, kind)
    res = {'code': 'fresh', 'may_load': False, 'message': '', 'manifest': None, 'archive': art}

    if not os.path.isfile(art):
        res['message'] = ('[map_archive] 没有找到存档 %s ⇒ 本次**从零建图**（不会加载任何旧状态）。'
                          % art)
        return res

    man = read_manifest(base)
    res['manifest'] = man
    cur_spawn = world_spawn(world)

    def refuse(code):
        res['code'] = code
        res['may_load'] = False
        res['message'] = refusal_message(code, base, name, kind, world, man, cur_spawn)
        if allow_mismatch:
            res['message'] += ('\n  ⚠️ 注意：本次给了 map_allow_world_mismatch:=True，'
                               '但上面这条不是"隔离不一致"，而是**存档本身不完整/类型不对**，覆盖参数救不了它。')
        return res

    if man is None or man.get('__parse_error__'):
        return refuse('refused_no_manifest')
    if man.get('kind') and man.get('kind') != kind:
        return refuse('refused_kind')
    if kind == KIND_POSEGRAPH and not os.path.isfile(base + '.data'):
        return refuse('refused_missing_data')

    mismatch = []
    if man.get('world') != world:
        mismatch.append('refused_world')
    d = spawn_diff(man.get('spawn_pose'), cur_spawn)
    if d is not None and d > SPAWN_EPS:
        mismatch.append('refused_spawn')

    if mismatch and not allow_mismatch:
        return refuse(mismatch[0])

    # ---- 到这里：可以加载
    warns = []
    if mismatch:
        res['code'] = 'override'
        warns.append('⚠️ map_allow_world_mismatch:=True 已生效：**跳过了 world/spawn 隔离检查**'
                     '（存档 world=%s，本次 world=%s）——出问题不要怪链路。'
                     % (man.get('world'), world))
    else:
        res['code'] = 'resume'
    if resolution is not None and man.get('resolution') is not None:
        try:
            if abs(float(man['resolution']) - float(resolution)) > 1e-9:
                warns.append('⚠️ resolution 与存档记录不同：存档 %s / 本次 %s（栅格尺寸变了，'
                             '旧图与新帧不在同一栅格上，建图质量自负）'
                             % (man['resolution'], resolution))
        except Exception:
            pass
    if map_start_pose is not None and man.get('map_start_pose') is not None:
        try:
            if max(abs(float(a) - float(b)) for a, b in zip(man['map_start_pose'], map_start_pose)) > 1e-6:
                warns.append('⚠️ map_start_pose 与存档记录不同：存档 %s / 本次 %s'
                             % (man['map_start_pose'], list(map_start_pose)))
        except Exception:
            pass

    res['may_load'] = True
    head = ('[map_archive] ✅ 续建（slam_toolbox 反序列化）：加载 %s' % art) if kind == KIND_POSEGRAPH \
        else ('[map_archive] ✅ 续建（3D 先验点云）：加载 %s' % art)
    res['message'] = '\n'.join([head] +
                               ['  · 存档 world=%s / 本次 world=%s' % (man.get('world'), world),
                                '  · 存档 spawn: %s' % spawn_str(man.get('spawn_pose')),
                                '  · 存档 map_start_pose=%s / 本次=%s'
                                % (man.get('map_start_pose'), list(map_start_pose) if map_start_pose is not None else '(未指定)'),
                                '  · sidecar: %s（updated_at=%s）' % (manifest_path(base), man.get('updated_at'))] +
                               warns)
    return res


# ============================================================ 目录清单（list / info）
def _file_info(p):
    if not os.path.isfile(p):
        return None
    st = os.stat(p)
    return {'path': p, 'size': st.st_size,
            'mtime': datetime.datetime.fromtimestamp(st.st_mtime).replace(microsecond=0).isoformat()}


def collect_archives(directory, kind):
    """扫目录里所有 `<name>.<ext>` 存档（不跟软链重复计数）。"""
    out = []
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return out
    for fn in names:
        if kind == KIND_POSEGRAPH and not fn.endswith('.posegraph'):
            continue
        if kind == KIND_PCD and not fn.endswith('.pcd'):
            continue
        base = os.path.join(directory, fn.rsplit('.', 1)[0])
        item = {'name': os.path.basename(base), 'base': base, 'kind': kind,
                'manifest': read_manifest(base)}
        for f in companion_files(base, kind):
            item[os.path.basename(f)] = _file_info(f)
        out.append(item)
    return out


def human_archive_line(item):
    man = item['manifest'] or {}
    art = archive_file(item['base'], item['kind'])
    fi = _file_info(art)
    size = ('%.2f MB' % (fi['size'] / 1048576.0)) if fi else '缺失'
    return ('  %-28s world=%-9s %-9s %-9s  (%s)' % (
        item['name'],
        man.get('world') or '—(无 sidecar)',
        size,
        (fi['mtime'] if fi else '—'),
        'spawn ' + spawn_str(man.get('spawn_pose')) if man else '**旧版存档：无 sidecar**'))


# ============================================================ spawn 表同步核对
def check_spawn_table_sync(launch_py=None):
    """解析 hzmi_rm_simulation 的 launch，核对本文件的 WORLD_SPAWN 与真正 spawn 用的值一致。

    做法：ast 解析那份 launch 的源码，找 `world_configs = {...}` 字面量，取出每个 world 的 x/y/z/yaw。
    返回 (ok: bool, diffs: list[str], table: dict)。
    """
    if launch_py is None:
        share = find_share_dir('hzmi_rm_simulation')
        launch_py = os.path.join(share, 'launch', 'rm_simulation.launch.py') if share else None
    if not launch_py or not os.path.isfile(launch_py):
        return False, ['找不到 hzmi_rm_simulation 的 launch（先 source install/setup.bash）'], {}
    src = open(launch_py, 'r').read()
    tree = ast.parse(src)
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == 'world_configs' and isinstance(node.value, ast.Dict):
                    for k, v in zip(node.value.keys, node.value.values):
                        if not isinstance(k, ast.Attribute) or not isinstance(v, ast.Dict):
                            continue
                        entry = {}
                        for kk, vv in zip(v.keys, v.values):
                            if isinstance(kk, ast.Constant) and kk.value in ('x', 'y', 'z', 'yaw'):
                                try:
                                    entry[kk.value] = float(ast.literal_eval(vv))
                                except Exception:
                                    pass
                        found[k.attr] = entry
    if not found:
        return False, ['没能从 %s 里解析出 world_configs（launch 结构改了？）' % launch_py], {}
    diffs = []
    for world, sp in WORLD_SPAWN.items():
        got = found.get(world)
        if got is None:
            diffs.append('%s: launch 里没有这个 world（本表有）' % world)
            continue
        for k in ('x', 'y', 'z', 'yaw'):
            if k in got and abs(got[k] - sp[k]) > 1e-9:
                diffs.append('%s.%s: launch=%s 本表=%s' % (world, k, got[k], sp[k]))
    for world in found:
        if world not in WORLD_SPAWN:
            diffs.append('%s: launch 里有、本表没有' % world)
    return (not diffs), diffs, found


# ============================================================ CLI
def _cmd_check(a):
    if a.dir:
        base = os.path.join(a.dir, a.name)
        directory = a.dir
    else:
        directory = map_dir() if a.kind == KIND_POSEGRAPH else pcd_dir()
        base = os.path.join(directory, a.name)
    res = verify(base, a.name, a.world, kind=a.kind, allow_mismatch=a.allow_world_mismatch,
                 map_start_pose=a.map_start_pose, resolution=a.resolution)
    if a.json:
        print(json.dumps({k: v for k, v in res.items() if k != 'manifest'}, ensure_ascii=False, indent=2))
    else:
        print(res['message'])
        print('  · 判定: %s  may_load=%s  base=%s' % (res['code'], res['may_load'], base))
    return 0 if (res['may_load'] or res['code'] == 'fresh') else 3


def _cmd_write(a):
    directory = a.dir or (map_dir() if a.kind == KIND_POSEGRAPH else pcd_dir())
    base = os.path.join(directory, a.name)
    art = archive_file(base, a.kind)
    if not os.path.isfile(art):
        print('[map_archive] ❌ 存档文件还不存在，拒绝写 sidecar：%s' % art, file=sys.stderr)
        return 2
    files = [f for f in companion_files(base, a.kind) if os.path.isfile(f)]
    if a.kind == KIND_POSEGRAPH and not os.path.isfile(base + '.data'):
        print('[map_archive] ❌ 缺 %s.data —— 半份存档，不写 sidecar' % base, file=sys.stderr)
        return 2
    man = write_manifest(base, a.name, a.kind, a.world, files,
                         map_start_pose=a.map_start_pose, resolution=a.resolution,
                         spawn=world_spawn(a.world), extra=getattr(a, 'extra', None) or None)
    print('[map_archive] ✅ sidecar 已写：%s（world=%s）' % (manifest_path(base), man['world']))
    return 0


def _cmd_adopt(a):
    """给旧版（无 sidecar）存档补一份 —— **必须显式给 --world**，等于人工担保。"""
    directory = a.dir or (map_dir() if a.kind == KIND_POSEGRAPH else pcd_dir())
    base = os.path.join(directory, a.name)
    art = archive_file(base, a.kind)
    if not os.path.isfile(art):
        print('[map_archive] ❌ 没有这份存档：%s' % art, file=sys.stderr)
        return 2
    old = read_manifest(base)
    if old and not old.get('__parse_error__'):
        print('[map_archive] ❌ 已有 sidecar（world=%s），adopt 只用于补旧存档；'
              '要改就自己编辑 %s' % (old.get('world'), manifest_path(base)), file=sys.stderr)
        return 2
    sp = world_spawn(a.world)
    print('[map_archive] ⚠️ 你正在给 %s 打上「属于 world=%s / spawn=%s」的记号。'
          % (art, a.world, spawn_str(sp)))
    print('            这个断言只有你能做（脚本无法验证）：如果它其实是别的场地的图，'
          '以后续建就会歪。')
    man = write_manifest(base, a.name, a.kind, a.world,
                         [f for f in companion_files(base, a.kind) if os.path.isfile(f)],
                         map_start_pose=a.map_start_pose, resolution=a.resolution, spawn=sp)
    print('[map_archive] ✅ 已补 sidecar：%s（created_at=%s）' % (manifest_path(base), man['created_at']))
    return 0


def _cmd_show(a):
    directory = a.dir or (map_dir() if a.kind == KIND_POSEGRAPH else pcd_dir())
    base = os.path.join(directory, a.name)
    man = read_manifest(base)
    print('[map_archive] 存档 %s' % base)
    print('  sidecar: %s' % (manifest_path(base) if os.path.isfile(manifest_path(base)) else '（无 —— 旧版存档）'))
    if man:
        print((yaml.safe_dump(man, allow_unicode=True, sort_keys=False) if yaml else str(man)).rstrip())
    print('  文件:')
    for f in companion_files(base, a.kind):
        fi = _file_info(f)
        print('    %-40s %s' % (os.path.basename(f),
                                ('%d B  %s' % (fi['size'], fi['mtime'])) if fi else '缺失'))
    return 0


def _cmd_list(a):
    md, pd_ = map_dir(), pcd_dir()
    print('[map_archive] 2D 位姿图存档目录: %s' % md)
    items = collect_archives(md, KIND_POSEGRAPH)
    if not items:
        print('  （空）')
    for it in items:
        print(human_archive_line(it))
    print('[map_archive] 3D 点云存档目录: %s' % pd_)
    items3 = collect_archives(pd_, KIND_PCD)
    if not items3:
        print('  （空）')
    for it in items3:
        print(human_archive_line(it))
    if a.json:
        print(json.dumps({'map_dir': md, 'pcd_dir': pd_,
                          'posegraph': [{'name': i['name'], 'manifest': i['manifest']} for i in items],
                          'pcd': [{'name': i['name'], 'manifest': i['manifest']} for i in items3]},
                         ensure_ascii=False, indent=2))
    return 0


def _cmd_dirs(a):
    d = {'map_dir': map_dir(), 'pcd_dir': pcd_dir()}
    if a.json:
        print(json.dumps(d, ensure_ascii=False))
    else:
        print('map_dir=%s' % d['map_dir'])
        print('pcd_dir=%s' % d['pcd_dir'])
    return 0


def _cmd_spawn_table(a):
    if a.check:
        ok, diffs, found = check_spawn_table_sync()
        if ok:
            print('[map_archive] ✅ 出生点表与 hzmi_rm_simulation/launch/rm_simulation.launch.py 一致（%d 个 world）'
                  % len(found))
            return 0
        print('[map_archive] ❌ 出生点表与真正 spawn 用的值**不一致**（隔离比对会误判）：', file=sys.stderr)
        for d in diffs:
            print('    · %s' % d, file=sys.stderr)
        return 4
    for w, sp in WORLD_SPAWN.items():
        print('%-10s %s' % (w, spawn_str(sp)))
    return 0


def _parse_pose3(text):
    if text is None:
        return None
    if isinstance(text, (list, tuple)):
        vals = [float(v) for v in text]
    else:
        vals = [float(v) for v in ast.literal_eval(str(text).strip())]
    if len(vals) != 3:
        raise ValueError('需要 3 个数 [x, y, theta]，收到 %r' % (text,))
    return vals


def build_parser():
    ap = argparse.ArgumentParser(description='地图存档的场地隔离守卫（纯 Python，可被 launch / shell / 节点共用）')
    sub = ap.add_subparsers(dest='cmd', required=True)

    def add_common(p):
        p.add_argument('--name', required=True, help='存档基名（不含扩展名）')
        p.add_argument('--kind', choices=[KIND_POSEGRAPH, KIND_PCD], default=KIND_POSEGRAPH)
        p.add_argument('--dir', help='存档目录（默认：map/ 或 PCD/ 的规范目录）')

    p = sub.add_parser('check', help='判定"这次能不能加载这份存档"；退出码 0=可以(或存档不存在)，3=拒绝')
    add_common(p)
    p.add_argument('--world', required=True)
    p.add_argument('--allow-world-mismatch', action='store_true')
    p.add_argument('--map-start-pose', help='本次用的 map_start_pose，如 "[0.0, 0.0, 0.0]"')
    p.add_argument('--resolution', type=float, help='本次用的 resolution（用于软警告）')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_check)

    p = sub.add_parser('write-manifest', help='写/刷新 sidecar（存档必须已存在）')
    add_common(p)
    p.add_argument('--world', required=True)
    p.add_argument('--map-start-pose', help='如 "[0.0, 0.0, 0.0]"')
    p.add_argument('--resolution', type=float)
    p.set_defaults(func=_cmd_write)

    p = sub.add_parser('adopt', help='给旧版（无 sidecar）存档补一份 —— 必须显式 --world，等于人工担保')
    add_common(p)
    p.add_argument('--world', required=True)
    p.add_argument('--map-start-pose', help='如 "[0.0, 0.0, 0.0]"')
    p.add_argument('--resolution', type=float)
    p.set_defaults(func=_cmd_adopt)

    p = sub.add_parser('show', help='打印某份存档的 sidecar + 文件信息')
    add_common(p)
    p.set_defaults(func=_cmd_show)

    p = sub.add_parser('list', help='列出 map/ 与 PCD/ 下的全部存档及其 world/manifest 摘要')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_list)

    p = sub.add_parser('dirs', help='打印规范存档目录（launch 与脚本实际读写的那两个）')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_dirs)

    p = sub.add_parser('spawn-table', help='打印/核对场地出生点表（本文件 = 单一事实来源）')
    p.add_argument('--check', action='store_true', help='与 hzmi_rm_simulation 的 spawn 值逐项核对')
    p.set_defaults(func=_cmd_spawn_table)
    return ap


def main(argv=None):
    ap = build_parser()
    a = ap.parse_args(argv)
    if getattr(a, 'map_start_pose', None) is not None:
        try:
            a.map_start_pose = _parse_pose3(a.map_start_pose)
        except Exception as e:
            print('[map_archive] ❌ --map-start-pose 解析失败：%s' % e, file=sys.stderr)
            return 2
    try:
        return a.func(a)
    except RuntimeError as e:      # 环境问题（没 source / 找不到包）⇒ 干净报错，不要甩 traceback
        print('[map_archive] ❌ %s' % e, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
