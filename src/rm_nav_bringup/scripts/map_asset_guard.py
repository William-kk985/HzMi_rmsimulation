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
