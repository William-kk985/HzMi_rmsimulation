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
import re
import subprocess
import sys
import time
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


# ============================================================ 覆盖前备份 / 恢复
# 2026-10-06 事故后新增：`save` 一直是**同名覆盖**，而 3D 先验被 LIO 退化段污染后
# 一次 save 就把好存档换成了坏存档（见 docs/continue_mapping.md §9 事故复盘）。
# 现在**写之前**先把既有产物复制一份带时间戳的备份，写坏了能一条命令回到上一代。
#
# 命名：`<存档基名>.<ext>.prev-<YYYYmmdd-HHMMSS>`（例 `RMUC2026.posegraph.prev-20261006-161530`）
#   · 为什么带时间戳而不是固定 `.bak`：`save` 会被反复调用；固定 `.bak` 只有一代，
#     连续两次坏 save 就把好存档挤掉了。时间戳可以留**多代**，代价只是磁盘（见 keep）。
#   · 为什么 keep 默认 3：一份 ~64 s 的存档 = 5.4 MB posegraph + 0.6 MB data,
#     3 代 ≈ 18 MB；PCD 侧一份 3~18 MB，3 代 ≈ 9~54 MB。既可回溯又不会无界增长。
#     （时间戳排序取**最新 keep 代**，更老的自动删；删除只针对本函数自己生成的名字。）
# 备份的是「一整套」：posegraph 侧 = .posegraph + .data + .meta.yaml，
#   PCD 侧 = .pcd + .meta.yaml —— 单独恢复 `*.data` 而不恢复 `*.posegraph` 没有意义。
BACKUP_MARK = '.prev-'
BACKUP_TS_FMT = '%Y%m%d-%H%M%S'
BACKUP_KEEP = 3
_BACKUP_RE = None


def _backup_re():
    global _BACKUP_RE
    if _BACKUP_RE is None:
        import re
        _BACKUP_RE = re.compile(r'^(.+)\.prev-(\d{8}-\d{6})$')
    return _BACKUP_RE


def backup_stamp(when=None):
    return (when or datetime.datetime.now()).strftime(BACKUP_TS_FMT)


def backup_path(path, ts):
    return '%s%s%s' % (path, BACKUP_MARK, ts)


def backup_ts_of(path):
    """从一个备份文件名里取回时间戳；不是备份文件 ⇒ None。"""
    m = _backup_re().match(os.path.basename(path))
    return m.group(2) if m else None


def artifact_paths(base, kind):
    """一份存档"一整套"包含哪些文件（备份/恢复/展示共用）。"""
    if kind == KIND_POSEGRAPH:
        return [base + '.posegraph', base + '.data', manifest_path(base)]
    if kind == KIND_PCD:
        return [base + '.pcd', manifest_path(base)]
    if kind == 'all':
        return artifact_paths(base, KIND_POSEGRAPH) + [base + '.pcd']
    raise ValueError('kind 只能是 %s/%s/all，收到 %r' % (KIND_POSEGRAPH, KIND_PCD, kind))


def _artifact_sets(name, kind, directory=None):
    """返回 [(path, 人类可读标签)]：把 `--kind all` 展开成 map/ 与 PCD/ 两侧。"""
    out = []
    for k in ([KIND_POSEGRAPH, KIND_PCD] if kind == 'all' else [kind]):
        if directory:
            d = directory
        else:
            d = map_dir() if k == KIND_POSEGRAPH else pcd_dir()
        for p in artifact_paths(os.path.join(d, name), k):
            out.append((p, k))
    return out


def list_backups(path):
    """某文件的所有备份：[(ts, 备份路径)]，**新的在前**。"""
    d = os.path.dirname(path) or '.'
    pre = os.path.basename(path) + BACKUP_MARK
    out = []
    try:
        for fn in os.listdir(d):
            if not fn.startswith(pre):
                continue
            ts = fn[len(pre):]
            if not _backup_re().match(fn):
                continue
            out.append((ts, os.path.join(d, fn)))
    except OSError:
        return []
    return sorted(out, key=lambda t: t[0], reverse=True)


def sha256_12(path, limit=None):
    """取前 12 位十六进制 sha256（人读的指纹；limit = 只算前 N 字节，用于大文件快速比对）。"""
    import hashlib
    h = hashlib.sha256()
    try:
        with open(path, 'rb') as f:
            while True:
                b = f.read(1 << 20)
                if not b:
                    break
                h.update(b)
                if limit and f.tell() >= limit:
                    break
    except OSError:
        return None
    return h.hexdigest()[:12]


def _unique_backup_ts(path, ts):
    """同一秒内重复备份（save 后马上 restore）不能互相覆盖 ⇒ 秒数往后挪到空位。"""
    t = datetime.datetime.strptime(ts, BACKUP_TS_FMT)
    while os.path.exists(backup_path(path, ts)):
        t += datetime.timedelta(seconds=1)
        ts = t.strftime(BACKUP_TS_FMT)
    return ts


def backup_files(paths, keep=BACKUP_KEEP, ts=None, dry_run=False, log=print):
    """把 `paths` 里**存在**的文件各复制一份 `<path>.prev-<ts>`，并按 keep 轮转。返回记录列表。

    每个文件用**同一个时间戳**（同一代），这样 restore 能按代整体回滚。
    返回 [{'src','dst','ts','size','sha256','skipped'}]。
    """
    ts = ts or backup_stamp()
    recs = []
    for p in paths:
        if not os.path.isfile(p):
            recs.append({'src': p, 'dst': None, 'ts': None, 'size': None,
                         'sha256': None, 'skipped': '不存在'})
            continue
        my_ts = _unique_backup_ts(p, ts)
        dst = backup_path(p, my_ts)
        if dry_run:
            recs.append({'src': p, 'dst': dst, 'ts': my_ts,
                         'size': os.path.getsize(p), 'sha256': None, 'skipped': 'dry-run'})
            continue
        tmp = dst + '.part'
        import shutil
        shutil.copy2(p, tmp)
        os.replace(tmp, dst)
        recs.append({'src': p, 'dst': dst, 'ts': my_ts, 'size': os.path.getsize(dst),
                     'sha256': sha256_12(dst), 'skipped': None})
        if log:
            log('  [备份] %s → %s（%.2f MB sha256:%s）'
                % (os.path.basename(p), os.path.basename(dst),
                   os.path.getsize(dst) / 1048576.0, recs[-1]['sha256']))
        _prune_backups(p, keep, log=log)
    return recs


def _prune_backups(path, keep, log=print):
    """只保留最新 keep 代（删除的目标**必须**匹配 `<path>.prev-<ts>` 这个模式）。"""
    if keep is None or keep <= 0:
        return []
    gens = list_backups(path)
    removed = []
    for ts, p in gens[int(keep):]:
        if backup_path(path, ts) != p:        # 双保险：删之前再核一次"这个名字是我生成的"
            continue
        try:
            os.unlink(p)
            removed.append(p)
            if log:
                log('  [轮转] 删掉过老备份 %s（keep=%d）' % (os.path.basename(p), keep))
        except OSError:
            pass
    return removed


def generations(name, kind='all', directory=None):
    """列出这份存档现存的备份代：{ts: [(live_path, backup_path, size)]}，ts 新的在前（dict 保序）。"""
    out = {}
    for p, _k in _artifact_sets(name, kind, directory):
        for ts, b in list_backups(p):
            if not os.path.isfile(b):
                continue
            out.setdefault(ts, []).append((p, b, os.path.getsize(b)))
    return {ts: out[ts] for ts in sorted(out, key=lambda t: t, reverse=True)}


def _group_paths(name, k, directory=None):
    """一套存档里"同一时刻一起备份"的那组文件：位姿图侧 3 个 / PCD 侧 2 个。"""
    d = directory or (map_dir() if k == KIND_POSEGRAPH else pcd_dir())
    return artifact_paths(os.path.join(d, name), k)


def restore_files(name, kind='all', from_ts=None, keep=BACKUP_KEEP, dry_run=False,
                  directory=None, log=print):
    """把一份存档回滚到备份。`from_ts` 可以是时间戳，也可以是某个备份文件路径。

    语义（有意如此）：
      · 不给 `--from` ⇒ **按组各取最新一代**：位姿图侧（posegraph+data+sidecar）取该组最新，
        PCD 侧（pcd+sidecar）取该组最新。为什么按组而不是"全局最新一个时间戳"：
        两侧是**两个不同的写者**在不同时刻备份的（位姿图由 map_archive.sh 写、PCD 由累加器写）
        ⇒ 时间戳天然不同；2026-10-06 实测按"全局最新"回滚只会恢复 PCD 侧、把位姿图漏掉
        （见 .tmp_hygiene/out/real1/run.txt 的 ❌）。组内文件永远同刻同代，所以按组对齐是对的。
      · 给了 `--from T` ⇒ 只回滚**在 T 这一代真的备份过**的文件（其余不动）。
      · 回滚前先把**当前**（要坏的）文件也备份一代 ⇒ restore 本身可逆；
      · 只回滚备份过的文件，不会因为 restore 删掉任何东西。
    返回 (ok, ts, recs, message)；ts 给 `--from` 时是那个时间戳，否则是 None（多代）。
    """
    groups = [KIND_POSEGRAPH, KIND_PCD] if kind == 'all' else [kind]
    ts_arg = None
    if from_ts:
        ts_arg = backup_ts_of(from_ts) or str(from_ts)
        if not ts_arg:
            return False, None, [], '看不懂 --from=%r（要时间戳 20261006-161530 或某个 .prev-* 文件路径）' % (from_ts,)
    plans = []            # [(ts, live, backup)]
    for k in groups:
        paths = _group_paths(name, k, directory)
        if ts_arg:
            for live in paths:
                bak = backup_path(live, ts_arg)
                if os.path.isfile(bak):
                    plans.append((ts_arg, live, bak))
        else:
            by_ts = {}
            for live in paths:
                for ts, bak in list_backups(live):
                    by_ts.setdefault(ts, []).append((live, bak))
            if not by_ts:
                continue
            newest = max(by_ts)
            for live, bak in by_ts[newest]:
                plans.append((newest, live, bak))
    if not plans:
        avail = []
        for k in groups:
            for live in _group_paths(name, k, directory):
                avail += [t for t, _b in list_backups(live)]
        return False, ts_arg, [], ('没有找到可用的备份代（%s%s）；现存代：%s'
                                   % (name, '' if kind == 'all' else '/%s' % kind,
                                      ', '.join(sorted(set(avail), reverse=True)) or '（一个都没有）'))
    recs = []
    for ts, live, bak in plans:
        cur = _file_info(live)
        rec = {'live': live, 'from': bak, 'ts': ts, 'bak_size': os.path.getsize(bak),
               'bak_sha256': sha256_12(bak), 'before': cur,
               'before_sha256': sha256_12(live) if cur else None}
        if not dry_run:
            if cur:                                  # 当前文件先留一代（restore 可逆）
                backup_files([live], keep=keep, log=log)
            import shutil
            tmp = live + '.restore-part'
            shutil.copy2(bak, tmp)
            os.replace(tmp, live)
            rec['after'] = _file_info(live)
            rec['after_sha256'] = sha256_12(live)
        recs.append(rec)
        if log:
            log('  [恢复] %-34s ← %-40s %s B sha256:%s%s'
                % (os.path.basename(live), os.path.basename(bak), rec['bak_size'], rec['bak_sha256'],
                   '  [dry-run]' if dry_run else ' → 现 %s B sha256:%s'
                   % (rec.get('after', {}).get('size'), rec.get('after_sha256'))))
    used = sorted({r['ts'] for r in recs})
    return True, (used[0] if len(used) == 1 else None), recs, \
        ('已恢复 %d 个文件（代：%s；位姿图侧与 PCD 侧各取自己那一组的最新一代）'
         % (len(recs), ', '.join(used)))


# ============================================================ 会话身份：活栈发现（2026-10-06 事故后新增）
# 事故：用户给自己那套栈（map_name:=RMUC2026_v2，18:08 起）跑 save，但 `.session.yaml`
#   已被**并发的自动化测试栈**（19:09:40 起，map_name:=RMUC2026_dropab_ab_g_long）覆盖。
#   当时唯一的"活性检查"是 `kill -0 <session_pid>`，它只打了一句警告就放行 ⇒
#   31.89 MB 的位姿图被写到**别人那套会话的名字**上。
#
# 为什么 pid 不能当活性依据（两头都错）：
#   · 假阳性：pid 会被回收 —— 一个早已结束的会话，其 pid 可能正好被别的进程占用 ⇒ "看起来还活着"；
#   · 假阴性：记录的可能只是**包装进程**（`setsid ros2 launch … &` 的 `$!`、`timeout`、
#     外层脚本），包装一退出 pid 就没了；反过来 launch 主进程被 SIGKILL 时子节点也可能还活着
#     ⇒ 栈活着但 pid 不在（本仓库已有前科：docs/continue_mapping.md §5.2 的 setsid/$! 陷阱）。
#   ⇒ 活性/身份一律以 **ROS 图 + 进程命令行 + 存档参数** 为准，pid 只作为**辅助证据**，
#     且必须同时满足"进程在 + cmdline 与记录一致 + 起始时刻与 started_at 一致"才算证据。
#
# 名字解析顺序（`resolve_session()`，从强到弱，任何一步都不"猜"）：
#   ① `--name X`（人担保）
#   ② ROS 图上的**活会话播报器**：launch 起的 `map_session` 节点在常驻（latched）话题
#      `/map_session/info` 与服务 `/map_session/query` 上广播本次会话（名字/world/存档基名/
#      出生点/started_at/session_id），并**自检**"图上有且只有一个 /slam_toolbox、
#      且它提供 /slam_toolbox/serialize_map"才把 verified 置真；
#   ③ 活着的 `bringup_sim.launch.py` 进程的命令行（`map_name:=` / `world:=`）；
#      看不到 launch 进程时（组合 launch / 进程在别的 PID namespace）退一步用
#      **活映射器自己报的存档基名**（`slam_toolbox` 的 `map_file_name`；**续建**时才有值）；
#   ④ `.session.yaml` + 把它钉在**活栈**上的证明（活 launch cmdline 对得上 / slam_toolbox 的
#      `map_file_name` 基名对得上 / 记录的 pid 活着且 cmdline 与起始时刻都对得上）；
#   ⑤ 都不成立 ⇒ **拒绝**（列出查了什么、怎么继续），绝不退化成"读文件就用"。
#
# 多会话（两套栈同时在跑）⇒ 直接拒绝并列出：两个 `/slam_toolbox` 同名节点、两个 `map_session`
#   节点、两个 mapping 形态的活 launch、或两份 session_id 不同的播报。
SESSION_STATE_NAME = '.session.yaml'
SESSION_NODE = 'map_session'
SESSION_TOPIC = '/map_session/info'
SESSION_SERVICE = '/map_session/query'
SESSION_SCHEMA = 'rm_nav_bringup/map_session@1'
MAPPER_NODE = 'slam_toolbox'
SERIALIZE_SERVICE = '/slam_toolbox/serialize_map'
MAPPING_LAUNCH_FILE = 'bringup_sim.launch.py'
MAPPING_MODES = ('mapping', 'slam_nav')
SESSION_PID_TOL_S = 300.0            # started_at 与进程起始时刻的允许差（launch 写文件在 t≈几秒）


def session_state_path(directory=None):
    return os.path.join(directory or map_dir(), SESSION_STATE_NAME)


def read_session_state(path=None):
    """读 launch 写的会话状态（`.session.yaml`）；不存在 ⇒ None，坏文件 ⇒ {'__parse_error__': …}。"""
    p = path or session_state_path()
    if not os.path.isfile(p):
        return None
    try:
        with open(p, 'r') as f:
            d = yaml.safe_load(f) if yaml else _mini_yaml(f.read())
        if not isinstance(d, dict):
            return {'__parse_error__': '内容不是 mapping', '__path__': p}
        d = dict(d)
        d['__path__'] = p
        return d
    except Exception as e:                                        # pragma: no cover
        return {'__parse_error__': str(e), '__path__': p}


# ------------------------------------------------------------ 进程表（pid 证据的正确用法）
def _proc_stat(pid, btime, clk):
    """从 /proc/<pid>/stat 取 (state, 起始 epoch)。字段 22 = starttime（时钟滴答）。"""
    try:
        with open('/proc/%d/stat' % pid, 'r') as f:
            txt = f.read()
    except Exception:
        return None, None
    rp = txt.rfind(')')                 # comm 里可能带空格/括号 ⇒ 从最后一个 ')' 之后切
    if rp < 0:
        return None, None
    rest = txt[rp + 2:].split()
    try:
        state = rest[0]
        starttime = int(rest[19])       # 3=state ⇒ 22 = rest[19]
    except Exception:
        return state if rest else None, None
    return state, (btime + starttime / float(clk) if btime else None)


def proc_table():
    """当前 PID namespace 看得到的进程表：pid → {state, argv, started_epoch, cmdline}。

    ⚠️ 只看得到同一个 PID namespace 里的进程（本仓库的沙箱 bash 每条命令一个 namespace，
    用户自己的终端则共享一个）。started_epoch 与 `ps -o lstart=` 同源（/proc/stat 的 btime
    + /proc/<pid>/stat 的 starttime），不依赖外部命令。
    """
    btime, clk = 0.0, 100.0
    try:
        with open('/proc/stat', 'r') as f:
            for line in f:
                if line.startswith('btime '):
                    btime = float(line.split()[1])
                    break
    except Exception:                                             # pragma: no cover
        pass
    try:
        clk = float(os.sysconf('SC_CLK_TCK'))
    except Exception:                                             # pragma: no cover
        clk = 100.0
    out = {}
    try:
        names = os.listdir('/proc')
    except Exception:                                             # pragma: no cover
        return out
    for n in names:
        if not n.isdigit():
            continue
        pid = int(n)
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                argv = [a.decode('utf-8', 'replace') for a in f.read().split(b'\0') if a]
        except Exception:
            continue
        if not argv:                    # 内核线程 / 刚退出
            continue
        state, started = _proc_stat(pid, btime, clk)
        out[pid] = {'pid': pid, 'state': state, 'argv': argv, 'started_epoch': started,
                    'cmdline': ' '.join(argv)}
    return out


def parse_launch_args(argv):
    """从 launch 进程 argv 里取 `key:=value`（ros2 launch 的启动参数一律是这个形式）。"""
    out = {}
    for a in argv:
        if ':=' not in a or a.startswith('-'):
            continue
        k, v = a.split(':=', 1)
        if k and all(c.isalnum() or c == '_' for c in k):
            out[k] = v
    return out


def find_mapping_launches(procs):
    """活着的 `bringup_sim.launch.py` 进程（要求该文件名是**独立的一个 argv 项**，
    这样 `bash -c "ros2 launch … bringup_sim.launch.py …"` 这种包装不会被重复计入）。"""
    res = []
    for pid in sorted(procs):
        p = procs[pid]
        if p.get('state') == 'Z':
            continue
        if not any(os.path.basename(a) == MAPPING_LAUNCH_FILE for a in p['argv']):
            continue
        args = parse_launch_args(p['argv'])
        world = (args.get('world') or '').strip()
        name = (args.get('map_name') or '').strip() or world
        res.append({'pid': pid, 'state': p.get('state'), 'argv': p['argv'],
                    'started_epoch': p.get('started_epoch'), 'cmdline': p.get('cmdline'),
                    'args': args, 'world': world, 'map_name': name,
                    'mode': (args.get('mode') or '').strip()})
    return res


def describe_launch(l):
    ts = (datetime.datetime.fromtimestamp(l['started_epoch']).astimezone().strftime('%Y-%m-%d %H:%M:%S')
          if l.get('started_epoch') else '?')
    return ('pid=%d map_name=%s world=%s mode=%s（起于 %s）'
            % (l['pid'], l['map_name'] or '（空）', l['world'] or '（空）',
               l['mode'] or '（默认）', ts))


def _cmdline_matches(recorded, live_argv):
    """记录的 launch argv 与活进程 argv 是否同一次启动（python 会在 argv[0] 前插解释器）。"""
    if not recorded:
        return False
    rec = [str(a) for a in recorded]
    live = [str(a) for a in live_argv]
    return rec == live or rec == live[1:] or live == rec[1:] or \
        (len(rec) >= 3 and rec[-3:] == live[-3:])


# ------------------------------------------------------------ ROS 图 + 会话播报
def ros_param_string(node_name, param, timeout=8):
    """`ros2 param get` 取字符串参数；拿不到/不是字符串 ⇒ None。"""
    try:
        out = subprocess.run(['ros2', 'param', 'get', node_name, param],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout).stdout.decode('utf-8', 'replace')
    except Exception:
        return None
    for line in out.splitlines():
        m = re.match(r'^\s*String value is:\s*(.*)$', line)
        if m:
            v = m.group(1).strip()
            if len(v) >= 2 and v[0] == v[-1] and v[0] in ('"', "'"):
                v = v[1:-1]
            return v
    return None


def ros_graph_probe(timeout=6.0, topic_timeout=5.0, node_name='map_archive_probe'):
    """用 rclpy 看**活图**，并收 `/map_session/info`（latched）里的会话播报。

    返回：ok / error / nodes / node_endpoints / services / helpers / mappers /
          serialize_present / session_service_present / payloads（按 session_id 去重）。
    `payloads` 收到 ≥2 份 ⇒ 图上有两套栈（各自的 session_id 不同）。
    话题收不到时退化为调一次 `/map_session/query` 服务。
    """
    info = {'ok': False, 'error': '', 'nodes': [], 'node_endpoints': [], 'services': [],
            'helpers': [], 'mappers': [], 'serialize_present': False,
            'session_service_present': False, 'payloads': []}
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                               QoSHistoryPolicy)
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except Exception as e:
        info['error'] = 'rclpy/std_msgs/std_srvs 不可用（先 source install/setup.bash？）：%s' % e
        return info
    node = None
    inited = False
    try:
        # ~/.ros/log 不可写时（沙箱/容器/只读 HOME）rclpy 会直接起不来 ⇒ 退到临时日志目录。
        # 只影响本进程的环境变量，不写任何用户文件。
        if not os.environ.get('ROS_LOG_DIR') and not os.access(os.path.expanduser('~'), os.W_OK):
            import tempfile
            os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='map_archive_roslog_')
        rclpy.init(args=None)
        inited = True
        node = Node(node_name)
        msgs = []
        qos = QoSProfile(depth=8, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, SESSION_TOPIC, lambda m: msgs.append(m.data), qos)
        # 等 discovery + latched 回放：收到第一份后再多留 1.0 s（抓可能的**第二套**播报），
        # 最长不超过 topic_timeout（没有播报器时最多白等这么久，$MAP_ARCHIVE_TOPIC_TIMEOUT 可调）。
        deadline = time.time() + max(0.5, float(topic_timeout))
        first_at = None
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if msgs and first_at is None:
                first_at = time.time()
            if first_at is not None and time.time() - first_at > 1.0:
                break

        endpoints = []
        for name, ns in node.get_node_names_and_namespaces():
            endpoints.append(('/' + name) if ns in ('', '/') else (ns.rstrip('/') + '/' + name))
        services = sorted(n for n, _t in node.get_service_names_and_types())
        info['node_endpoints'] = sorted(endpoints)
        info['nodes'] = sorted(set(endpoints))
        info['services'] = services
        info['helpers'] = [n for n in endpoints if os.path.basename(n) == SESSION_NODE]
        info['mappers'] = [n for n in endpoints if os.path.basename(n) == MAPPER_NODE]
        info['serialize_present'] = SERIALIZE_SERVICE in services
        info['session_service_present'] = SESSION_SERVICE in services

        if not msgs and info['session_service_present']:
            # 话题被改名/QoS 不匹配时的兜底：直接问服务（返回 message = 同一份 JSON）
            try:
                cli = node.create_client(Trigger, SESSION_SERVICE)
                if cli.wait_for_service(timeout_sec=min(3.0, max(1.0, timeout / 2.0))):
                    fut = cli.call_async(Trigger.Request())
                    t_end = time.time() + 3.0
                    while not fut.done() and time.time() < t_end:
                        rclpy.spin_once(node, timeout_sec=0.1)
                    res = fut.result() if fut.done() else None
                    if res is not None and res.message:
                        msgs.append(res.message)
            except Exception as e:                                # pragma: no cover
                info['error'] = '（服务兜底也失败：%s）' % e

        seen = {}
        for raw in msgs:
            try:
                d = json.loads(raw)
            except Exception:
                continue
            if not isinstance(d, dict):
                continue
            seen[str(d.get('session_id') or raw)] = d
        info['payloads'] = list(seen.values())
        info['ok'] = True
    except Exception as e:                                        # pragma: no cover
        info['error'] = 'ROS 图探测失败：%s' % e
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:                                         # pragma: no cover
            pass
        if inited:
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:                                     # pragma: no cover
                pass
    return info


# ------------------------------------------------------------ 会话名解析（唯一出处）
def _fmt_payload(d):
    return ('map_name=%s world=%s session_id=%s launch_pid=%s resumed=%s started_at=%s'
            % (d.get('map_name'), d.get('world'), d.get('session_id'), d.get('launch_pid'),
               d.get('resumed'), d.get('started_at')))


def resolve_session(explicit_name=None, explicit_world=None, allow_cross_session=False,
                    probe=None, procs=None, state=None, no_ros=False, topic_timeout=5.0,
                    session_file=None, mapper_param=None):
    """确定「这次 save 该写哪个名字」；返回 dict（见模块头部的顺序说明）。

    返回键：ok / code / name / source / world / map_start_pose / session_id / started_at /
            resumed / archive_base / live_name / live_source / mapper_param / evidence /
            message / summary / exit_code
    · `evidence` 是给人看的证据链（⓵~⓹ 逐条），`message` 是拒绝时的完整文案（多行）。
    · probe / procs / state / mapper_param 可注入 ⇒ 判定逻辑能离线单测（--no-ros 也走这条）。
    """
    ev = ['[map_archive] 会话名解析（①--name ＞ ②图上活会话 ＞ ③活 launch 进程/活映射器自报基名 '
          '＞ ④.session.yaml+活性证明 ＞ ⑤拒绝）']
    st = read_session_state(session_file) if state is None else state
    procs = proc_table() if procs is None else procs
    launches = find_mapping_launches(procs)
    mapping_launches = [l for l in launches if l['mode'] in MAPPING_MODES]
    other_launches = [l for l in launches if l['mode'] not in MAPPING_MODES]
    if probe is None:
        probe = ({'ok': False, 'error': '--no-ros：跳过了 ROS 图探测', 'nodes': [], 'services': [],
                  'helpers': [], 'mappers': [], 'serialize_present': False,
                  'session_service_present': False, 'payloads': []} if no_ros
                 else ros_graph_probe(topic_timeout=topic_timeout))
    mappers = list(probe.get('mappers') or [])
    helpers = list(probe.get('helpers') or [])
    payloads = list(probe.get('payloads') or [])
    serialize_present = bool(probe.get('serialize_present'))
    if mapper_param is None and not no_ros and probe.get('ok') and len(mappers) == 1:
        mapper_param = ros_param_string(mappers[0], 'map_file_name')
    mapper_base = (os.path.basename(mapper_param.rstrip('/')) if mapper_param else '')

    # 图上必须有且只有一个"能落盘的映射器"（唯一 /slam_toolbox + serialize 服务）
    mapper_ok = (len(mappers) == 1 and serialize_present)
    res = {'ok': False, 'code': '', 'name': '', 'source': '', 'world': '',
           'map_start_pose': None, 'session_id': '', 'started_at': '', 'resumed': False,
           'archive_base': '', 'live_name': '', 'live_source': '', 'mapper_param': mapper_param or '',
           'evidence': ev, 'message': '', 'summary': '', 'exit_code': 3}

    def refuse(code, summary, lines):
        res['code'] = code
        res['summary'] = summary
        res['message'] = '\n'.join(lines)
        return res

    def finish(name, source, world='', spawn=None, sid='', started='', resumed=False, base=''):
        res.update(ok=True, code=source, name=name, source=source, world=world or '',
                   map_start_pose=spawn, session_id=sid, started_at=started,
                   resumed=bool(resumed), archive_base=base or '')
        return res

    # ---------------- ⓵ 证据收集（先把"看到了什么"全打出来，再决定） ----------------
    ev.append('[map_archive]   ① --name               : %s'
              % ('%s（显式，最高优先级）' % explicit_name if explicit_name else '（未给）'))
    if payloads:
        if len(payloads) == 1:
            p = payloads[0]
            ev.append('[map_archive]   ② 活会话播报（ROS 图） : %s；自检 verified=%s（mapper=%s ×%d、%s%s）'
                      % (_fmt_payload(p), p.get('verified'), MAPPER_NODE,
                         int(p.get('mapper_count') or 0), SERIALIZE_SERVICE,
                         '✓' if p.get('serialize_present') else '✗'))
        else:
            ev.append('[map_archive]   ② 活会话播报（ROS 图） : ⚠️ 收到 %d 份不同的播报（= 同时有几套栈）' % len(payloads))
            for p in payloads:
                ev.append('[map_archive]        · %s' % _fmt_payload(p))
    elif no_ros:
        ev.append('[map_archive]   ② 活会话播报（ROS 图） : （跳过：--no-ros）')
    else:
        ev.append('[map_archive]   ② 活会话播报（ROS 图） : 没有 %s（%s / 服务 %s）；图探测 ok=%s%s'
                  % (SESSION_NODE, SESSION_TOPIC, SESSION_SERVICE, probe.get('ok'),
                     ('，error=%s' % probe.get('error')) if probe.get('error') else ''))
    if launches:
        ev.append('[map_archive]   ③ 活 launch 进程       : %d 个' % len(launches))
        for l in launches:
            ev.append('[map_archive]        · %s' % describe_launch(l))
    else:
        ev.append('[map_archive]   ③ 活 launch 进程       : 0 个（本 PID namespace 内没看到 %s）'
                  % MAPPING_LAUNCH_FILE)
    if st is None:
        ev.append('[map_archive]   ④ .session.yaml        : 不存在（%s）'
                  % (session_file or '(默认 map_dir/.session.yaml)'))
    elif st.get('__parse_error__'):
        ev.append('[map_archive]   ④ .session.yaml        : ⚠️ 解析失败：%s' % st['__parse_error__'])
    else:
        ev.append('[map_archive]   ④ .session.yaml        : map_name=%s world=%s session_pid=%s '
                  'started_at=%s resumed=%s'
                  % (st.get('map_name'), st.get('world'), st.get('session_pid'),
                     st.get('started_at'), st.get('resumed')))
    if mapper_param:
        ev.append('[map_archive]   ⑤ slam_toolbox 参数    : map_file_name=%s ⇒ 基名 %s'
                  % (mapper_param, mapper_base))
    else:
        ev.append('[map_archive]   ⑤ slam_toolbox 参数    : map_file_name=（未设置/取不到 ⇒ 本次是"从零建图"）')

    # ---------------- ⓶ 多会话：先拒绝，不给任何"猜"的机会 ----------------
    amb = []
    if len(mappers) > 1:
        amb.append('同名映射器 /%s 有 %d 个（ROS 图里同名节点，服务 %s 也无法指定目标；'
                   'param get 同样二义）' % (MAPPER_NODE, len(mappers), SERIALIZE_SERVICE))
    if len(helpers) > 1:
        amb.append('活会话播报器 /%s 有 %d 个（= 同时起了两套会写 .session.yaml 的栈）'
                   % (SESSION_NODE, len(helpers)))
    if len(payloads) > 1:
        amb.append('收到 %d 份不同的会话播报（session_id 不同）' % len(payloads))
    if len(mapping_launches) > 1:
        amb.append('mapping/slam_nav 形态的 %s 进程有 %d 个' % (MAPPING_LAUNCH_FILE, len(mapping_launches)))
    if amb:
        lines = ['[map_archive] ❌ 拒绝存档：检测到**多于一套**在跑的建图栈 —— 现在写盘可能写到你没在看的那一套上。']
        lines += ['[map_archive]    · %s' % a for a in amb]
        for l in launches:
            lines.append('[map_archive]    · 活 launch：%s' % describe_launch(l))
        for p in payloads:
            lines.append('[map_archive]    · 活会话：%s' % _fmt_payload(p))
        lines += ['[map_archive]    · 图上的 /%s 节点：%s' % (MAPPER_NODE, ', '.join(mappers) or '（无）'),
                  '[map_archive]   ⇒ 绝不替你猜。三条出路：',
                  '[map_archive]      ① 只留一套栈（关掉多余的那套）后重跑 save；',
                  '[map_archive]      ② 两套栈各用各的 ROS_DOMAIN_ID（各自的 save 只看得到自己那一套）：',
                  '[map_archive]           ROS_DOMAIN_ID=<n> tools/scripts/mapping/map_archive.sh save',
                  '[map_archive]      ③ 明确知道在写谁：先关掉一套，再 save --name X。',
                  '[map_archive]      参考：ros2 node list | grep -n %s ；pgrep -af %s'
                  % (MAPPER_NODE, MAPPING_LAUNCH_FILE)]
        ev.append('[map_archive]   ❌ 多会话 ⇒ 拒绝（见下）')
        return refuse('refused_multi_session', '同时有多套建图栈在跑', lines)

    # ---------------- ⓷ 活栈身份（名字必须**来自活证据**） ----------------
    live_name, live_source, live_payload = '', '', None
    if len(payloads) == 1:
        p = payloads[0]
        self_ok = bool(p.get('verified'))
        ours_ok = (len(mappers) == 1 and serialize_present) if probe.get('ok') else None
        if self_ok and ours_ok is not False:
            live_name, live_source, live_payload = str(p.get('map_name') or ''), 'live-helper', p
        else:
            why = []
            if not self_ok:
                why.append('播报器自检 verified=False（mapper_count=%s serialize_present=%s）'
                           % (p.get('mapper_count'), p.get('serialize_present')))
            if ours_ok is False:
                why.append('本进程独立核对：/%s 节点 %d 个、%s %s'
                           % (MAPPER_NODE, len(mappers), SERIALIZE_SERVICE,
                              '在' if serialize_present else '不在'))
            ev.append('[map_archive]   ⚠️ 播报器在，但"活栈"没被证实：%s' % '；'.join(why))
            res['_helper_unverified'] = '；'.join(why)
    elif len(mapping_launches) == 1 and not other_launches:
        live_name, live_source = mapping_launches[0]['map_name'], 'live-launch'
    elif len(mapping_launches) == 1 and other_launches:
        ev.append('[map_archive]   ⚠️ 除了一套 mapping 栈，还有 %d 个**非 mapping** 的 %s 进程；'
                  '不带 --name 时不敢替你认哪一套是"本次会话"' % (len(other_launches), MAPPING_LAUNCH_FILE))
        res['_extra_launches'] = [describe_launch(l) for l in other_launches]

    # ③b 看不到 launch 进程时（组合 launch / 进程在别的 PID namespace）退一步：
    #     用**活映射器自己加载的存档基名**（续建时 map_file_name 才有值）——它同样钉在活栈上，
    #     与 `.session.yaml`（会被后人覆盖）无关。从零建图时它是空的 ⇒ 这条自然失效。
    if not live_name and mapper_ok and mapper_base and not mapping_launches:
        if st is not None and not st.get('__parse_error__') and (st.get('map_name') or '').strip() \
                and str(st['map_name']).strip() != mapper_base:
            ev.append('[map_archive]   ⚠️ .session.yaml 说 map_name=%s，而活映射器加载的是 %s '
                      '⇒ 以**映射器自己**为准（文件可能被后启动的 launch 覆盖过）'
                      % (st.get('map_name'), mapper_base))
        live_name, live_source = mapper_base, 'live-mapper-param'
        ev.append('[map_archive]   ✅ 名字取自活映射器自己加载的存档基名（续建）：%s' % mapper_base)

    live_world = ''
    if live_source == 'live-helper' and live_payload:
        live_world = str(live_payload.get('world') or '')
    elif live_source == 'live-launch':
        live_world = mapping_launches[0]['world']

    # 图上的映射器本身必须"活着且是 mapping 模式"，否则 save 无从落盘
    if probe.get('ok') and not mapper_ok:
        ev.append('[map_archive]   ⚠️ 活映射器检查：/%s ×%d、%s %s ⇒ %s'
                  % (MAPPER_NODE, len(mappers), SERIALIZE_SERVICE,
                     '在' if serialize_present else '不在',
                     '没有可写的映射器' if not mappers else
                     ('映射器不在 mapping 模式（没有 %s）' % SERIALIZE_SERVICE)))

    # ---------------- ⓸ ⓵ 显式 --name ----------------
    if explicit_name:
        # 跨会话：解析出来的名字 ≠ 活栈会话名 ⇒ 会凭空造出"半个存档"（除非显式承担）
        if live_name and explicit_name != live_name:
            if not allow_cross_session:
                lines = ['[map_archive] ❌ 拒绝存档：**--name 指定的名字 ≠ 正在跑的那套栈的会话名**'
                         '（跨会话写盘 = 另起一份，而不是"覆盖同一个名字"）。',
                         '[map_archive]    · --name            : %s' % explicit_name,
                         '[map_archive]    · 活栈会话名         : %s（来源=%s%s）'
                         % (live_name, live_source,
                            ('，session_id=%s' % live_payload.get('session_id')) if live_payload else ''),
                         '[map_archive]    · 活栈的 map_name 参数: %s' % (mapper_param or '（未设置）'),
                         '[map_archive]   ⇒ 没有写任何文件。两条出路：',
                         '[map_archive]      ① 用活栈自己的名字：--name %s' % live_name,
                         '[map_archive]      ② 确实要另存一份：加 --allow-cross-session（你自己担保）']
                ev.append('[map_archive]   ❌ --name=%s 与活栈会话名 %s 不一致 ⇒ 拒绝' % (explicit_name, live_name))
                return refuse('refused_cross_session', '--name 与活栈会话不一致', lines)
            ev.append('[map_archive]   ⚠️ --name=%s 与活栈会话名 %s 不一致，但给了 --allow-cross-session ⇒ 放行'
                      % (explicit_name, live_name))
        if live_name:
            ev.append('[map_archive]   ✅ 采用 name=%s（来源=explicit；活栈会话名=%s 已核对）'
                      % (explicit_name, live_name))
        else:
            ev.append('[map_archive]   ✅ 采用 name=%s（来源=explicit；没有可核对的活栈会话名）' % explicit_name)
        w = explicit_world or live_world or (st.get('world') if st and not st.get('__parse_error__') else '')
        spawn = None
        if live_payload and live_payload.get('map_start_pose'):
            spawn = live_payload.get('map_start_pose')
        elif st and not st.get('__parse_error__') and st.get('map_start_pose'):
            spawn = st.get('map_start_pose')
        return finish(explicit_name, 'explicit', w, spawn,
                      (live_payload or {}).get('session_id') or '',
                      (live_payload or {}).get('started_at') or '', False, '')

    # ---------------- ⓹ 没给 --name：按 ②③④ 找活会话 ----------------
    # ② 播报器
    if live_source == 'live-helper':
        warn = _session_file_warning(st, live_name, live_source)
        if warn:
            ev.append(warn)
        # 与 slam_toolbox 自己加载的存档基名交叉核对（续建时才有值）
        if mapper_base and mapper_base != live_name and not allow_cross_session:
            lines = ['[map_archive] ❌ 拒绝存档：**slam_toolbox 自己加载的存档基名 ≠ 本次会话名**。',
                     '[map_archive]    · 会话名（来自活播报器）: %s' % live_name,
                     '[map_archive]    · slam_toolbox map_file_name: %s ⇒ 基名 %s' % (mapper_param, mapper_base),
                     '[map_archive]   ⇒ 这一般意味着"会话状态"与"实际加载的图"不是一次启动的产物；'
                     '写下去会得到一份与它加载的图无关的新存档。',
                     '[map_archive]      确认要这么做：加 --allow-cross-session；否则先查：',
                     '[map_archive]      ros2 param get /%s map_file_name ； ros2 node list | grep %s'
                     % (MAPPER_NODE, SESSION_NODE)]
            ev.append('[map_archive]   ❌ 会话名 %s ≠ slam_toolbox 加载的基名 %s ⇒ 拒绝' % (live_name, mapper_base))
            return refuse('refused_mapper_param_mismatch', 'slam_toolbox 加载的存档与会话名不一致', lines)
        if mapper_base:
            ev.append('[map_archive]   ✅ 与 slam_toolbox 的 map_file_name 基名一致（%s）' % mapper_base)
        ev.append('[map_archive]   ✅ 采用 name=%s（来源=live-helper；world=%s session_id=%s）'
                  % (live_name, live_world or '?', live_payload.get('session_id')))
        return finish(live_name, 'live-helper', explicit_world or live_world,
                      live_payload.get('map_start_pose'),
                      live_payload.get('session_id') or '', live_payload.get('started_at') or '',
                      live_payload.get('resumed'), live_payload.get('archive_base') or '')

    # ② 播报器在但不 verified ⇒ 拒绝（不退化到读文件）
    if payloads and res.get('_helper_unverified'):
        lines = ['[map_archive] ❌ 拒绝存档：**会话播报器在，但"活栈"没被证实** —— 不猜名字。',
                 '[map_archive]    · %s' % _fmt_payload(payloads[0]),
                 '[map_archive]    · 没被证实的原因：%s' % res['_helper_unverified'],
                 '[map_archive]    · 若 slam_toolbox 刚起来（launch 后 ~4 s 才起），等几秒重跑即可；',
                 '[map_archive]      否则说明 /%s 不在 mapping 模式或不是唯一 —— 先看：' % MAPPER_NODE,
                 '[map_archive]      ros2 node list | grep %s ； ros2 service list | grep serialize_map' % MAPPER_NODE,
                 '[map_archive]      也可以自己担保名字：save --name X']
        return refuse('refused_helper_unverified', '播报器自检不通过', lines)

    # ③ 活 launch 进程
    if live_source == 'live-launch':
        l = mapping_launches[0]
        if not mapper_ok:
            lines = ['[map_archive] ❌ 拒绝存档：认出了活栈（%s），但**图上没有唯一可写的映射器**。' % describe_launch(l),
                     '[map_archive]    · /%s ×%d、%s %s' % (MAPPER_NODE, len(mappers), SERIALIZE_SERVICE,
                                                            '在' if serialize_present else '不在'),
                     '[map_archive]    · 可能：slam_toolbox 还没起来（launch 后 ~4 s）/ 不是 mapping 模式 / '
                     'mapper:=cartographer（不支持这条存档路径）',
                     '[map_archive]   ⇒ 没有写任何文件。等栈起来后重跑，或显式 save --name X。']
            return refuse('refused_no_live_mapper', '没有唯一可写的映射器', lines)
        if mapper_base and mapper_base != live_name and not allow_cross_session:
            lines = ['[map_archive] ❌ 拒绝存档：**slam_toolbox 自己加载的存档基名 ≠ 活栈会话名**。',
                     '[map_archive]    · 活 launch：%s' % describe_launch(l),
                     '[map_archive]    · slam_toolbox map_file_name: %s ⇒ 基名 %s' % (mapper_param, mapper_base),
                     '[map_archive]   ⇒ 确认要这么做：加 --allow-cross-session。']
            return refuse('refused_mapper_param_mismatch', 'slam_toolbox 加载的存档与会话名不一致', lines)
        warn = _session_file_warning(st, live_name, live_source)
        if warn:
            ev.append(warn)
        if mapper_base:
            ev.append('[map_archive]   ✅ 与 slam_toolbox 的 map_file_name 基名一致（%s）' % mapper_base)
        ev.append('[map_archive]   ✅ 采用 name=%s（来源=live-launch；world=%s）'
                  % (live_name, live_world or '?'))
        return finish(live_name, 'live-launch', explicit_world or live_world)

    # ③b 活映射器自报的存档基名（续建时才有的 map_file_name）
    if live_source == 'live-mapper-param':
        warn = _session_file_warning(st, live_name, live_source)
        if warn:
            ev.append(warn)
        ev.append('[map_archive]   ✅ 采用 name=%s（来源=live-mapper-param；slam_toolbox map_file_name=%s）'
                  % (live_name, mapper_param))
        return finish(live_name, 'live-mapper-param',
                      explicit_world or (str(st.get('world') or '')
                                         if st and not st.get('__parse_error__') else ''),
                      (st.get('map_start_pose') if st and not st.get('__parse_error__') else None),
                      str((st or {}).get('session_id') or ''),
                      str((st or {}).get('started_at') or ''),
                      bool((st or {}).get('resumed')), mapper_param or '')

    # ⓸ .session.yaml + 把它钉在活栈上的证明
    if st is not None and not st.get('__parse_error__') and (st.get('map_name') or '').strip():
        name = str(st['map_name']).strip()
        proof, notes = _session_file_proof(st, procs, launches, mapper_base, live_name)
        for n in notes:
            ev.append('[map_archive]       · %s' % n)
        if not mapper_ok:
            lines = ['[map_archive] ❌ 拒绝存档：**没有活着的建图栈**（拿不到任何活性证明），'
                     '.session.yaml 里的名字无法证明属于谁 ⇒ 不写任何文件。',
                     '[map_archive]    查了什么（都不是"活"的）：',
                     '[map_archive]      ① /%s 播报器（%s / %s）: 没有'
                     % (SESSION_NODE, SESSION_TOPIC, SESSION_SERVICE),
                     '[map_archive]      ② 活着的 %s 进程        : %d 个' % (MAPPING_LAUNCH_FILE, len(launches)),
                     '[map_archive]      ③ /%s 唯一且提供服务 %s: /%s ×%d，服务 %s'
                     % (MAPPER_NODE, SERIALIZE_SERVICE, MAPPER_NODE, len(mappers),
                        '在' if serialize_present else '不在'),
                     '[map_archive]      ④ .session.yaml         : map_name=%s session_pid=%s started_at=%s（%s）'
                     % (name, st.get('session_pid'), st.get('started_at'), notes[0] if notes else '未证实'),
                     '[map_archive]   ⇒ 这就是 2026-10-06 事故的形态（文件是上一次/别人那次会话留下的）。',
                     '[map_archive]      请：① 起栈后重跑 save（推荐）；或 ② 显式 save --name X（你担保这个名字）']
            ev.append('[map_archive]   ❌ 没有活栈 ⇒ 拒绝使用 .session.yaml 里的名字')
            return refuse('refused_no_live_stack', '没有活着的建图栈', lines)
        if not proof:
            lines = ['[map_archive] ❌ 拒绝存档：**没法把 .session.yaml 钉在活栈上** —— 它可能是被更晚的'
                     'launch 覆盖过的（2026-10-06 事故就是这一条）。',
                     '[map_archive]    · .session.yaml      : map_name=%s session_pid=%s started_at=%s'
                     % (name, st.get('session_pid'), st.get('started_at')),
                     '[map_archive]    · 活着的 launch 进程 : %s'
                     % ('；'.join(describe_launch(l) for l in launches) or '（本 namespace 内 0 个）'),
                     '[map_archive]    · slam_toolbox      : map_file_name=%s' % (mapper_param or '（未设置）'),
                     '[map_archive]    · 证明失败的原因    :']
            lines += ['[map_archive]        - %s' % n for n in notes]
            lines += ['[map_archive]   ⇒ 没有写任何文件。三条出路：',
                      '[map_archive]      ① 用活栈自己的名字：save --name <活栈的 map_name>；',
                      '[map_archive]      ② 让 launch 说清楚（重启栈，或用 2026-10-06 之后带播报器的 launch）；',
                      '[map_archive]      ③ 确认这份文件确实属于当前栈：删掉 %s 后重启 launch'
                      % (st.get('__path__') or session_file or SESSION_STATE_NAME)]
            ev.append('[map_archive]   ❌ .session.yaml 未被活证据钉住 ⇒ 拒绝')
            return refuse('refused_session_file_unproven', '.session.yaml 无法钉在活栈上', lines)
        ev.append('[map_archive]   ✅ 采用 name=%s（来源=session-file+活性证明；%s）' % (name, proof))
        spawn = st.get('map_start_pose')
        return finish(name, 'session-file', explicit_world or str(st.get('world') or ''), spawn,
                      str(st.get('session_id') or ''), str(st.get('started_at') or ''),
                      bool(st.get('resumed')), str(st.get('archive_base') or ''))

    # ⓹ 什么都没有
    lines = ['[map_archive] ❌ 拒绝存档：**无法确定"现在这套栈是谁"** ⇒ 不猜名字、不写任何文件。',
             '[map_archive]    查了什么：',
             '[map_archive]      ① /%s 播报器（%s / %s）: 没有' % (SESSION_NODE, SESSION_TOPIC, SESSION_SERVICE),
             '[map_archive]      ② 活着的 %s 进程        : %d 个' % (MAPPING_LAUNCH_FILE, len(launches)),
             '[map_archive]      ③ /%s（唯一 + %s）    : /%s ×%d，服务 %s'
             % (MAPPER_NODE, SERIALIZE_SERVICE, MAPPER_NODE, len(mappers),
                '在' if serialize_present else '不在'),
             '[map_archive]      ④ .session.yaml         : %s'
             % ('没有' if st is None else
                ('解析失败：%s' % st.get('__parse_error__') if st.get('__parse_error__') else
                 'map_name=%s（未通过活性/身份证明）' % st.get('map_name')))]
    if res.get('_extra_launches'):
        lines.append('[map_archive]      ⚠️ 另有非 mapping 的 launch：%s' % '；'.join(res['_extra_launches']))
    lines += ['[map_archive]   ⇒ 三条出路：',
              '[map_archive]      ① 起栈（launch 里带会话播报器）后重跑 save —— 这是默认推荐路径；',
              '[map_archive]      ② 显式给名字：save --name X（你担保它）；',
              '[map_archive]      ③ 先确认栈真的在 mapping 模式：ros2 service list | grep serialize_map']
    ev.append('[map_archive]   ❌ 既没有活会话，也没有可信的 .session.yaml ⇒ 拒绝')
    return refuse('refused_no_evidence', '无法确定会话名', lines)


def _session_file_warning(st, live_name, live_source):
    """名字来自活证据、而 `.session.yaml` 与它不一致 ⇒ 这正是事故形态，必须大声打出来。"""
    if not st or st.get('__parse_error__'):
        return ''
    fname = str(st.get('map_name') or '').strip()
    if not fname or fname == live_name:
        return ''
    return ('[map_archive]   ⚠️⚠️ .session.yaml 说 map_name=%s（session_pid=%s，started_at=%s），'
            '与活栈（来源=%s）的 %s **不一致** ⇒ 这份文件已被更晚的 launch 覆盖过，'
            '已忽略它（2026-10-06 事故就是这个形态）'
            % (fname, st.get('session_pid'), st.get('started_at'), live_source, live_name))


def _session_file_proof(st, procs, launches, mapper_base, live_name):
    """把 `.session.yaml` 钉在活栈上的三重证明；返回 (证明文本 or '', 每条尝试的说明)。"""
    name = str(st.get('map_name') or '').strip()
    notes = []
    # (a) 活 launch 进程自己就报同一个名字
    for l in launches:
        if l['name'] == name:
            notes.append('活 launch pid=%d 自己的 map_name 就是 %s（%s）'
                         % (l['pid'], name, describe_launch(l)))
            return ('活 launch 进程 pid=%d 报同名 %s' % (l['pid'], name)), notes
    notes.append('活着的 %s 进程没有报出 %s（%d 个：%s）'
                 % (MAPPING_LAUNCH_FILE, name, len(launches),
                    '；'.join(describe_launch(l) for l in launches) or '0 个'))
    # (b) slam_toolbox 自己加载的存档基名
    if mapper_base:
        if mapper_base == name:
            notes.append('slam_toolbox 的 map_file_name 基名 = %s' % mapper_base)
            return ('slam_toolbox 的 map_file_name 基名 = %s' % mapper_base), notes
        notes.append('slam_toolbox 的 map_file_name 基名 = %s ≠ %s' % (mapper_base, name))
    else:
        notes.append('slam_toolbox 的 map_file_name 未设置（从零建图）⇒ 基名对不上任何名字')
    # (c) 记录的 pid 活着 + cmdline/起始时刻对得上（pid 单独不算证据，三条一起才算）
    pid = st.get('session_pid')
    try:
        pid = int(pid)
    except Exception:
        pid = 0
    if pid > 0:
        p = procs.get(pid)
        if p is None:
            notes.append('session_pid=%d 不在 /proc（进程已退出，或它只是包装进程的 pid，'
                         '或它在一个看不到的 PID namespace 里）' % pid)
        elif p.get('state') == 'Z':
            notes.append('session_pid=%d 是僵尸进程 ⇒ 不算活着' % pid)
        else:
            rec = st.get('launch_cmdline')
            if rec and not _cmdline_matches(rec, p['argv']):
                notes.append('session_pid=%d 活着但 cmdline 与记录的 launch 命令行不一致 ⇒ 很可能是'
                             'pid 回收（假阳性）' % pid)
            elif not rec and not any('launch' in a for a in p['argv']):
                notes.append('session_pid=%d 活着但不是 launch 进程（且文件没记 launch_cmdline）' % pid)
            else:
                started = _parse_iso_epoch(st.get('started_at'))
                if started and p.get('started_epoch') and abs(started - p['started_epoch']) > SESSION_PID_TOL_S:
                    notes.append('session_pid=%d 活着，但起始时刻 %s 与 started_at %s 差 %.0f s ⇒ 不是同一次会话'
                                 % (pid,
                                    datetime.datetime.fromtimestamp(p['started_epoch']).astimezone()
                                    .strftime('%H:%M:%S'), st.get('started_at'),
                                    abs(started - p['started_epoch'])))
                else:
                    notes.append('session_pid=%d 活着 + cmdline 与记录一致 + 起始时刻与 started_at 一致' % pid)
                    return ('记录的 session_pid=%d 活着且 cmdline/起始时刻都对得上' % pid), notes
    else:
        notes.append('session_pid 缺失/非法（%r）⇒ pid 一侧无证据' % st.get('session_pid'))
    return '', notes


def _parse_iso_epoch(text):
    if not text:
        return None
    try:
        return datetime.datetime.fromisoformat(str(text)).timestamp()
    except Exception:
        return None


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


def _kind_label(kind):
    return {'all': '2D 位姿图 + 3D 点云', KIND_POSEGRAPH: '2D 位姿图', KIND_PCD: '3D 点云'}[kind]


def _cmd_session_resolve(a):
    """打印「这次 save 该写哪个名字」的证据链；退出码 0=定了，3=拒绝（不写任何文件），2=环境问题。"""
    res = resolve_session(explicit_name=a.name, explicit_world=a.world,
                          allow_cross_session=a.allow_cross_session, no_ros=a.no_ros,
                          topic_timeout=a.topic_timeout,
                          state=(read_session_state(a.session_file) if a.session_file else None))
    if a.json:
        print(json.dumps(res, ensure_ascii=False, indent=2))
        return 0 if res['ok'] else res['exit_code']
    for line in res['evidence']:
        print(line)
    if res['ok']:
        print('session_source=%s' % res['source'])
        print('session_name=%s' % res['name'])
        print('session_world=%s' % (res['world'] or ''))
        if res.get('map_start_pose'):
            print('session_spawn=%s' % json.dumps(res['map_start_pose']))
        print('session_id=%s' % (res.get('session_id') or ''))
        print('session_started_at=%s' % (res.get('started_at') or ''))
        print('session_resumed=%s' % ('true' if res.get('resumed') else 'false'))
        print('session_archive_base=%s' % (res.get('archive_base') or ''))
        return 0
    print(res['message'])
    print('[map_archive] ❌ 拒绝存档（%s）：上面是完整证据链与出路；**没有写任何文件**（退出码 %d）'
          % (res['summary'], res['exit_code']))
    sys.stdout.flush()      # 调用方（map_archive.sh）会捕获 stdout 再原样打印 ⇒ 顺序不乱
    return res['exit_code']


def _cmd_backup(a):
    """把现有存档整套备份一代（写之前调；也可手工调）。"""
    files = [p for p, _k in _artifact_sets(a.name, a.kind, a.dir)]
    exist = [p for p in files if os.path.isfile(p)]
    if not exist:
        print('[map_archive] 没有可备份的文件（%s 这套存档一个都不在）：' % _kind_label(a.kind))
        for p in files:
            print('    %s（不存在）' % p)
        return 0
    print('[map_archive] 备份 %s（%d 个文件，keep=%d）' % (_kind_label(a.kind), len(exist), a.keep))
    recs = backup_files(files, keep=a.keep, dry_run=a.dry_run)
    if a.json:
        print(json.dumps(recs, ensure_ascii=False, indent=2))
    n = sum(1 for r in recs if r['dst'])
    print('[map_archive] ✅ 备份完成：%d 个文件 → `*.prev-<ts>`（keep=%d）' % (n, a.keep))
    return 0


def _cmd_backups(a):
    gens = generations(a.name, a.kind, a.dir)
    print('[map_archive] 备份代（%s，%s）：%s' % (a.name, _kind_label(a.kind),
                                                 '无' if not gens else '%d 代' % len(gens)))
    for ts, items in gens.items():
        print('  · %s（%d 个文件，共 %.2f MB）' % (ts, len(items),
                                                 sum(i[2] for i in items) / 1048576.0))
        for live, bak, size in items:
            print('      %-38s %8.2f MB  %s' % (os.path.basename(bak), size / 1048576.0,
                                                '（当前文件在）' if os.path.isfile(live) else '（当前文件缺失）'))
    if a.json:
        print(json.dumps({ts: [{'live': l, 'backup': b, 'size': s} for l, b, s in items]
                          for ts, items in gens.items()}, ensure_ascii=False, indent=2))
    return 0 if gens else 5


def _cmd_restore(a):
    ok, ts, recs, msg = restore_files(a.name, a.kind, from_ts=a.from_,
                                      keep=a.keep, dry_run=a.dry_run, directory=a.dir)
    if not ok:
        print('[map_archive] ❌ %s' % msg, file=sys.stderr)
        return 5
    print('[map_archive] %s %s' % ('（dry-run）' if a.dry_run else '✅', msg))
    for r in recs:
        print('  · %-34s ← %-40s %s' % (os.path.basename(r['live']), os.path.basename(r['from']),
                                        '%d B sha256:%s → %s B sha256:%s'
                                        % ((r['before']['size'] if r['before'] else -1),
                                           r['before_sha256'] or '—',
                                           (r.get('after') or {}).get('size', -1),
                                           r.get('after_sha256') or '—')))
    print('  提示：restore 只回滚**备份过的**文件；当前文件也已被留了一代 ⇒ restore 本身可逆。')
    print('  续建用的 sidecar（.meta.yaml）与位姿图/点云同代恢复 ⇒ 守卫看到的仍是自洽的一套。')
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

    def add_backup_common(p):
        p.add_argument('--name', required=True, help='存档基名（不含扩展名）')
        p.add_argument('--kind', choices=[KIND_POSEGRAPH, KIND_PCD, 'all'], default='all',
                       help='all = 2D 位姿图 + 3D 点云两侧都算（默认）')
        p.add_argument('--dir', help='覆盖目录（只给单侧 kind 时有意义；测试用）')

    p = sub.add_parser('backup', help='把现有存档整套备份一代（`<文件>.prev-<ts>`，默认留 3 代）')
    add_backup_common(p)
    p.add_argument('--keep', type=int, default=BACKUP_KEEP, help='每侧保留几代（默认 %d）' % BACKUP_KEEP)
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_backup)

    p = sub.add_parser('backups', help='列出现存的备份代（新的在前）；没有备份 ⇒ 退出码 5')
    add_backup_common(p)
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_backups)

    p = sub.add_parser('restore', help='把存档回滚到某一代备份（默认最新一代；退出码 5 = 没有这一代）')
    add_backup_common(p)
    p.add_argument('--from', dest='from_', help='时间戳（20261006-161530）或某个 `*.prev-*` 文件路径')
    p.add_argument('--keep', type=int, default=BACKUP_KEEP)
    p.add_argument('--dry-run', action='store_true')
    p.set_defaults(func=_cmd_restore)

    p = sub.add_parser('spawn-table', help='打印/核对场地出生点表（本文件 = 单一事实来源）')
    p.add_argument('--check', action='store_true', help='与 hzmi_rm_simulation 的 spawn 值逐项核对')
    p.set_defaults(func=_cmd_spawn_table)

    p = sub.add_parser('session-resolve',
                       help='确定「这次 save 写哪个名字」（2026-10-06 事故后新增）：'
                            '①--name ＞ ②图上活会话播报器 ＞ ③活 launch 进程/活映射器自报基名 '
                            '＞ ④.session.yaml+活性证明 ＞ ⑤拒绝')
    p.add_argument('--name', help='显式指定的存档名（最高优先级；仍会与活栈会话做交叉核对）')
    p.add_argument('--world', help='显式指定的 world（只用于跨核对与汇报）')
    p.add_argument('--allow-cross-session', action='store_true',
                   help='放行"解析出的名字 ≠ 活栈会话名"（明确要另存一份时用）')
    p.add_argument('--session-file', help='覆盖 .session.yaml 路径（测试用；默认 <map_dir>/.session.yaml）')
    p.add_argument('--no-ros', action='store_true', help='跳过 ROS 图探测（测试用：只看进程/文件证据）')
    p.add_argument('--topic-timeout', type=float,
                   default=float(os.environ.get('MAP_ARCHIVE_TOPIC_TIMEOUT') or 5.0),
                   help='收 /map_session/info 的等待秒数（默认 5.0，可用 $MAP_ARCHIVE_TOPIC_TIMEOUT 覆盖）')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=_cmd_session_resolve)
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
