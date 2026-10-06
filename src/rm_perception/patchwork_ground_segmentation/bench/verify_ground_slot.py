#!/usr/bin/env python3
"""静态真值表：`ground` 槽位在**真实 LaunchDescription** 下到底启动了谁。

不是 grep —— 而是把 `bringup_sim.launch.py` 真正 import 进来、生成 LaunchDescription、
递归遍历所有实体，对每个节点用给定的 `ground` 值**求值它的 condition**。
这样"两个分割器互斥"是被 launch 自己的条件逻辑证明的，不是靠读代码猜的。

用法（需要先 source install/setup.bash）：
    python3 verify_ground_slot.py
输出：每个 ground 值下**启用**的分割器节点清单 + 断言（恰好 1 个）。
"""

import importlib.util
import os
import sys

from launch import LaunchContext
from launch_ros.actions import Node
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription

CANDIDATES = [
    'install/rm_nav_bringup/share/rm_nav_bringup/launch/bringup_sim.launch.py',
]


def load_ld():
    path = next((p for p in CANDIDATES if os.path.isfile(p)), None)
    if path is None:
        raise SystemExit('找不到 bringup_sim.launch.py（先 source install/setup.bash）')
    spec = importlib.util.spec_from_file_location('bsl_verify', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description(), path


def walk(entity):
    yield entity
    for attr in ('entities', 'actions'):
        for child in getattr(entity, attr, None) or []:
            yield from walk(child)


def seg_nodes(ld, ground):
    ctx = LaunchContext()
    ctx.launch_configurations['ground'] = ground
    out = []
    for e in walk(ld):
        if not isinstance(e, Node):
            continue
        if 'ground_segmentation' not in e.node_package:
            continue
        cond = e.condition
        enabled = True if cond is None else bool(cond.evaluate(ctx))
        out.append({
            'package': e.node_package,
            'executable': e.node_executable,
            'name': repr(getattr(e, '_Node__node_name', None)),
            'condition': type(cond).__name__ if cond is not None else None,
            'enabled': enabled,
        })
    return out


def main():
    ld, path = load_ld()
    print('LaunchDescription 来源: %s' % path)
    ok = True
    for ground in ('linefit', 'patchwork'):
        rows = seg_nodes(ld, ground)
        on = [r for r in rows if r['enabled']]
        print('\n== ground:=%s ==' % ground)
        for r in rows:
            print('  [%s] %s/%s  name=%s  condition=%s' % (
                'ON ' if r['enabled'] else 'off', r['package'], r['executable'],
                r['name'], r['condition']))
        if len(on) != 1:
            ok = False
            print('  ❌ 期望恰好 1 个分割器节点被启用，实际 %d 个' % len(on))
        else:
            print('  ✅ 恰好 1 个分割器被启用：%s/%s' % (on[0]['package'], on[0]['executable']))
    print('\n互斥断言: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
