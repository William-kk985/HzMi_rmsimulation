#!/usr/bin/env python3
# =============================================================================
# launch_increment_diff.py —— 证明「不传 sim_backend 时默认路径逐字节没变」
# =============================================================================
# 做法：用**同一组 launch 参数**把两个版本的 `bringup_sim.launch.py` 各构建一次
# LaunchDescription，把整棵 Action 树（每个 Action 的所有字段 + condition + 嵌套子
# Action）规范化成 JSON，然后：
#
#   ① **多重集比较**（与顺序无关）：NEW − OLD = 新增的 Action，OLD − NEW = 消失的 Action。
#      每一项都是"整棵子树的逐字段内容"，所以"改了某个 Node 的一个参数"也会显形。
#   ② **顺序比较**：两边都有的 Action，在列表里的相对顺序必须完全一致（时序证据）。
#
# 为什么不用 `diff 文件`：launch 文件是 Python，注释/换行/变量改名不等于行为变化；
# 这里比的是真正会被 launch 执行的东西。
#
# 用法：
#   launch_increment_diff.py --old <old/launch.py> --new <new/launch.py> [--arg k=v ...]
# 退出码：0 = 纯增量（只有新增、没有删除/改动、顺序不变）；2 = 有非预期差异。
# =============================================================================
import argparse
import importlib.util
import json
import os
import sys
import tempfile
from collections import Counter
from importlib.machinery import SourceFileLoader

# launch 的 logging 会在**导入期**去建 $HOME/.ros/log；本机 $HOME 只读 ⇒ 先把它指到
# 可写目录，否则工具在 `import launch` 阶段就 OSError（与 gz 的 HOME 坑同源）。
os.environ.setdefault('ROS_LOG_DIR',
                      os.path.join(tempfile.gettempdir(), 'rm_lvddiff_roslog'))
os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)

# 这些是 launch 运行期才填的缓存字段（execute() 之后才有），不参与比较
NOISE_SUFFIX = (
    '__sigkill_timeout', '__sigkill_timer', '__sigterm_timeout', '__sigterm_timer',
    '__stderr_buffer', '__stdout_buffer', '__expanded_node_name',
    '__expanded_node_namespace', '__expanded_parameter_arguments',
    '__expanded_remappings', '__extensions', '__final_node_name',
    '__substitutions_performed', '__subprocess_protocol', '__subprocess_transport',
    '__node_executable',
)


def load_module(path):
    name = 'lvd_' + str(abs(hash(path)) % 100000)
    loader = SourceFileLoader(name, path)
    spec = importlib.util.spec_from_loader(name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


def _ld_entities(obj):
    for k, v in vars(obj).items():
        if (k.endswith('__entities') or k.endswith('__actions')) and isinstance(v, list):
            return v
    return None


def ser(obj, depth=0):
    if depth > 16:
        return '<depth-limit>'
    t = type(obj).__name__
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    if isinstance(obj, (bytes, bytearray)):
        return '<bytes:%d>' % len(obj)
    if isinstance(obj, (list, tuple)):
        return [ser(v, depth + 1) for v in obj]
    if isinstance(obj, dict):
        # 键可能是**替换对象的元组**（launch 的 parameters 就长这样）；直接用 str(k)
        # 会把 `object at 0x7f...` 的地址写进 JSON ⇒ 两次构建永远不同（假差异）。
        out = {}
        for k, v in obj.items():
            if isinstance(k, str):
                out[k] = ser(v, depth + 1)
            else:
                out['<key>:' + canon(ser(k, depth + 1))] = ser(v, depth + 1)
        return out
    if t == 'LaunchDescription':
        return {'__type__': t, 'actions': ser(_ld_entities(obj) or [], depth + 1)}
    d = {'__type__': t}
    for k, v in sorted(vars(obj).items()):
        if any(k.endswith(sfx) for sfx in NOISE_SUFFIX) or 'logger' in k.lower():
            continue
        try:
            d[k] = ser(v, depth + 1)
        except Exception as e:                              # pragma: no cover
            d[k] = '<unserializable:%s>' % e
    return d


def strip_conditions(x):
    """去掉所有 __condition 字段：用来区分「Action 内容变了」与「只加了一扇门」。"""
    if isinstance(x, dict):
        return {k: strip_conditions(v) for k, v in x.items() if not k.endswith('__condition')}
    if isinstance(x, list):
        return [strip_conditions(v) for v in x]
    return x


def canon(x):
    return json.dumps(x, sort_keys=True, ensure_ascii=False, default=str)


def brief(x):
    """一行摘要：类型 + condition + 最有辨识度的字段。"""
    t = x.get('__type__', '?')
    bits = [t]
    for k, v in sorted(x.items()):
        if k.endswith('__condition') and isinstance(v, dict):
            bits.append('cond=' + brief(v))
        elif k.endswith('__launch_file_path'):
            s = canon(v)
            bits.append('src=' + (s if len(s) < 160 else s[:157] + '...'))
        elif k.endswith('__name') and isinstance(v, str):
            bits.append('name=' + v)
        elif k.endswith('__cmd') and isinstance(v, list):
            s = ' '.join(str(i) for i in v)
            bits.append('cmd=' + (s if len(s) < 160 else s[:157] + '...'))
        elif k.endswith('__launch_arguments') and isinstance(v, list):
            bits.append('args=' + ','.join(str(i[0][-1]) if isinstance(i, list) else str(i)
                                           for i in v)[:160])
    return ' '.join(bits)


def build(path, args, out_path):
    mod = load_module(path)
    from launch.substitutions import LaunchConfiguration
    for k, v in args.items():
        # 等价于命令行传参：把该 LaunchConfiguration 的求值结果钉成 v
        LaunchConfiguration(k).perform = lambda context, _v=v: _v   # noqa: E731
    ld = mod.generate_launch_description()
    tree = ser(ld)
    with open(out_path, 'w') as f:
        json.dump(tree, f, indent=1, sort_keys=True, ensure_ascii=False)
    return tree['actions']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--old', required=True)
    ap.add_argument('--new', required=True)
    ap.add_argument('--arg', action='append', default=[])
    ap.add_argument('--out-dir', default='')
    ap.add_argument('--strip-conditions', action='store_true',
                    help='比较前去掉所有 condition（证明"Action 内容一字未改，只多了门"）')
    a = ap.parse_args()
    args = {}
    for kv in a.arg:
        k, _, v = kv.partition('=')
        args[k] = v
    out_dir = a.out_dir or tempfile.mkdtemp(prefix='lvddiff_')
    os.makedirs(out_dir, exist_ok=True)
    old = [canon(x) for x in build(a.old, args, os.path.join(out_dir, 'old.json'))]
    new = [canon(x) for x in build(a.new, args, os.path.join(out_dir, 'new.json'))]
    if a.strip_conditions:
        old = [canon(strip_conditions(json.loads(x))) for x in old]
        new = [canon(strip_conditions(json.loads(x))) for x in new]

    co, cn = Counter(old), Counter(new)
    removed = list((co - cn).elements())
    added = list((cn - co).elements())
    # 保序检查：两边共有的 Action（按多重集扣掉新增/消失后）的相对顺序
    common = co & cn
    def common_seq(seq):
        left = Counter(common)
        out = []
        for x in seq:
            if left[x] > 0:
                left[x] -= 1
                out.append(x)
        return out
    order_ok = common_seq(old) == common_seq(new)

    print('args   = %s' % json.dumps(args, ensure_ascii=False))
    print('dumps  = %s' % out_dir)
    print('actions: OLD %d / NEW %d；共有 %d' % (len(old), len(new), sum(common.values())))
    print('')
    print('新增（NEW − OLD，整棵子树逐字段）：%d' % len(added))
    for x in added:
        print('  + ' + brief(json.loads(x)))
    print('消失/变成另一份（OLD − NEW）：%d' % len(removed))
    for x in removed:
        print('  - ' + brief(json.loads(x)))
    print('')
    print('共有 Action 的相对顺序：%s' % ('一致 ✅' if order_ok else '不一致 ❌'))
    if not removed and order_ok:
        print('==> 纯增量 ✅：没有任何 Action 被删/改，只有上面 %d 个新增，原有顺序不变'
              % len(added))
        return 0
    print('==> 非纯增量 ❌（有 Action 消失/被改，或顺序变了）')
    return 2


if __name__ == '__main__':
    sys.exit(main())
