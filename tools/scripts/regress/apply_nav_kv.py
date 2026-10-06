#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 nav2 槽位 YAML 打补丁（A/B 单变量用），**拒绝静默失效**。

用法（由 run_nav_clearance_ab.sh 调用，也可单独用）：
    python3 tools/scripts/regress/apply_nav_kv.py <params_dir> "FILE:path.to.key=VALUE" ...

约定
  · FILE          —— params_dir 下的文件名（或绝对路径）。
  · path.to.key   —— 该文件 `ros__parameters` 层下的路径。节点段名（`planner_server` /
                     `controller_server` / `global_costmap` / `local_costmap`）可以写、也可以不写
                     （两种写法都收；base 文件的 `global_costmap: global_costmap:` 双层同名也兼容）。
  · VALUE         —— YAML 标量。
  · **叶子键必须已存在**，否则报错退出（在 8 份 full copy 时代踩过"键名写错 = 静默没生效、A/B 白跑"）；
     确实要新增键（插件有默认值、本仓库没写）时写 `=+VALUE`，会打印 `[新增键]`。

打完补丁后文件被**原地覆盖**（调用方负责备份/还原）。返回值：0 = 全部成功。
"""
from __future__ import annotations

import os
import sys

import yaml


def nodes_of(d):
    """列出该文件里所有 (段名, ros__parameters 字典)。

    兼容两种写法：`planner_server: ros__parameters:`（一层）与
    base 文件里的 `global_costmap: global_costmap: ros__parameters:`（两层同名）。
    """
    out = []
    for sec, v in d.items():
        if not isinstance(v, dict):
            continue
        cur, guard = v, 0
        while 'ros__parameters' not in cur and guard < 4:
            nxt = cur.get(sec)
            if not isinstance(nxt, dict):
                break
            cur, guard = nxt, guard + 1
        if 'ros__parameters' in cur:
            out.append((sec, cur['ros__parameters']))
    return out


def pick_node(d, fname, first_key):
    cands = nodes_of(d)
    named = [c for c in cands if c[0] == first_key]
    if named:                       # 路径第一段就是节点段名 ⇒ 优先它（base 文件有 6 个段）
        return named[0]
    if len(cands) == 1:
        return cands[0]
    sys.exit('[kv] ❌ %s 里有 %d 个 ros__parameters 段（%s），请在路径里写明节点段名'
             % (fname, len(cands), ', '.join(c[0] for c in cands)))


def main(argv):
    pdir, kvs = argv[1], argv[2:]
    cache = {}

    def load(path):
        if path not in cache:
            with open(path) as f:
                cache[path] = yaml.safe_load(f)
        return cache[path]

    for kv in kvs:
        lhs, val = kv.split('=', 1)
        fname, dotted = lhs.split(':', 1)
        path = fname if os.path.isabs(fname) else os.path.join(pdir, fname)
        if not os.path.isfile(path):
            sys.exit('[kv] ❌ 文件不存在：%s' % path)
        d = load(path)
        keys = [k for k in dotted.split('.') if k not in ('', 'ros__parameters')]
        if not keys:
            sys.exit('[kv] ❌ 路径为空：%s' % kv)
        sec, node = pick_node(d, fname, keys[0])
        if keys and keys[0] == sec:
            keys = keys[1:]
        if not keys:
            sys.exit('[kv] ❌ 路径只有节点段名：%s' % kv)
        cur = node
        for k in keys[:-1]:
            if k not in cur or not isinstance(cur[k], dict):
                sys.exit('[kv] ❌ 键路径不存在：%s（在 %s 的 %s 下；断在 %s）'
                         % (dotted, fname, sec, k))
            cur = cur[k]
        add = val.startswith('+')
        if add:
            val = val[1:]
        if keys[-1] not in cur and not add:
            sys.exit('[kv] ❌ 叶子键不存在：%s（在 %s 的 %s 下）—— 拒绝静默失效；'
                     '确实要新增键就写 =+VALUE' % (dotted, fname, sec))
        try:
            v = yaml.safe_load(val)
        except Exception:      # noqa: BLE001
            v = val
        old = cur.get(keys[-1], '<不存在>')
        if isinstance(old, bool) and isinstance(v, str):
            v = v.lower() in ('1', 'true', 'yes', 'on')
        cur[keys[-1]] = v
        print('  %-36s %-50s %r -> %r%s' % (fname, '.'.join(keys), old, v,
                                            '   [新增键]' if old == '<不存在>' else ''))
    for path, d in cache.items():
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False, width=1000)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
