#!/usr/bin/env python3
"""`local_obstacle` / `global_obstacle` 槽位 → 代价地图图层开关的**静态真值表**校验。

它回答一个问题：**每个槽位值下，local_costmap / global_costmap 节点最终收到的
`plugins` 列表 + 每个图层的 `enabled` 到底是什么**，以及"scan/cloud/both 是否与改造前一致"。

做法（不是再抄一份表 —— 抄表迟早会和代码分叉）：
  1. **直接 import** `src/rm_navigation/rm_navigation/launch/navigation_launch.py`，拿到
     唯一的真值表 `LOCAL_OBSTACLE_LAYER_TABLE` / `GLOBAL_OBSTACLE_LAYER_TABLE` 与
     `build_param_substitutions()`；
  2. 对每个槽位值，用**真的 `nav2_common.launch.RewrittenYaml`**（跟 launch 里同一份代码）
     把 `params/nav2_params_sim_base.yaml` 重写一遍（`use_sim_time` / 五个 `*.enabled` 开关），
     再 yaml.safe_load 读回节点真正会收到的参数；
  3. 断言：
     · 每个被开关的图层都必须在 `plugins` 里（否则开关是死的）；
     · `plugins` 里每个图层都要有 `plugin:` 段（否则 nav2 起不来）；
     · `inflation_layer` 恒在且恒开（它不是障碍源，任何槽位都不该动它）；
     · `scan` / `cloud` / `both` 三个值的**生效图层集合**必须与改造前（2026-10-05 之前）逐位一致
       —— 即 stvl_layer 在这三个值下必须 enabled=false；
     · `stvl` → 只有 stvl_layer；`stvl_both` → stvl + scan + cloud；
     · global 三个值（stvl/scan/none）的生效集合不变。
  4. 打印 Markdown 真值表（可直接贴进 docs/stvl_local_costmap.md）。

用法（要先 source ROS 与工作区，脚本自己要 import launch/nav2_common）：
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 tools/scripts/regress/local_obstacle_truth_table.py            # 表格 + 自检
  python3 tools/scripts/regress/local_obstacle_truth_table.py --markdown # 只打 Markdown

退出码：0=全部断言通过；1=有任何一条不成立（并逐条打印原因）。
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import tempfile

# launch 的日志目录默认为 ~/.ros/log；沙箱/CI 里 HOME 可能只读 ⇒ 先指到可写目录再 import launch
os.environ.setdefault("ROS_LOG_DIR", os.path.join(tempfile.gettempdir(), "local_obstacle_tt_log"))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
LAUNCH_PY = os.path.join(ROOT, "src", "rm_navigation", "rm_navigation", "launch",
                         "navigation_launch.py")
BASE_YAML = os.path.join(ROOT, "src", "rm_navigation", "rm_navigation", "params",
                         "nav2_params_sim_base.yaml")

# 改造前（2026-10-05 之前）三个既有值的**生效图层集合** —— 这就是"行为不变"的判据。
LEGACY_LOCAL = {
    "scan": {"obstacle_layer"},
    "cloud": {"obstacle_cloud_layer"},
    "both": {"obstacle_layer", "obstacle_cloud_layer"},
}
LEGACY_GLOBAL = {
    "stvl": {"stvl_layer"},
    "scan": {"obstacle_layer"},
    "none": set(),
}


def _load_launch_module():
    import yaml  # noqa: F401  (提前 import，报错信息更清楚)
    spec = importlib.util.spec_from_file_location("rm_navigation_navigation_launch", LAUNCH_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _materialize(mod, local_obstacle, global_obstacle):
    """跑一遍真 RewrittenYaml，返回 (local_params, global_params) —— 节点真正收到的参数。"""
    import yaml
    from launch import LaunchContext
    from nav2_common.launch import RewrittenYaml

    subs = mod.build_param_substitutions("True", "true")
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        "namespace": "",
        "local_obstacle": local_obstacle,
        "global_obstacle": global_obstacle,
    })
    out = RewrittenYaml(source_file=BASE_YAML, root_key="",
                        param_rewrites=subs, convert_types=True).perform(ctx)
    with open(out) as f:
        data = yaml.safe_load(f)
    os.unlink(out)
    lc = data["local_costmap"]["local_costmap"]["ros__parameters"]
    gc = data["global_costmap"]["global_costmap"]["ros__parameters"]
    return lc, gc


def effective_layers(params):
    """生效图层 = plugins 里 enabled 不是 false 的那些（缺 enabled 键 = nav2 默认 true）。"""
    out = []
    for name in params["plugins"]:
        if name not in params:
            continue
        if params[name].get("enabled", True) is False:
            continue
        out.append(name)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--markdown", action="store_true", help="只输出 Markdown 真值表")
    a = ap.parse_args()

    mod = _load_launch_module()
    local_table = mod.LOCAL_OBSTACLE_LAYER_TABLE
    global_table = mod.GLOBAL_OBSTACLE_LAYER_TABLE

    problems = []
    rows = []
    for value in local_table:
        lc, gc = _materialize(mod, value, "stvl")
        plugins = list(lc["plugins"])
        eff = effective_layers(lc)
        flags = {n: (lc[n].get("enabled", True) is not False) for n in plugins if n in lc}

        # --- 断言 1：开关不能是死的（被表的某行打开 ⇒ 必须在 plugins 里） ---
        for layer, on in local_table[value].items():
            if layer not in plugins:
                problems.append(f"local_obstacle:={value} 打开 {layer}，但它不在 plugins={plugins}")
        # --- 断言 2：plugins 里每个图层都要有 plugin: 段 ---
        for name in plugins:
            if name not in lc:
                problems.append(f"plugins 里的 {name} 没有参数段（nav2 会起不来）")
            elif "plugin" not in lc[name]:
                problems.append(f"{name} 段里没有 plugin: 键")
        # --- 断言 3：inflation_layer 恒在恒开 ---
        if "inflation_layer" not in eff:
            problems.append(f"local_obstacle:={value} 下 inflation_layer 不生效（不该动它）")
        # --- 断言 4：scan/cloud/both 与改造前逐位一致 ---
        expect = {k for k, v in local_table[value].items() if v} | {"inflation_layer"}
        if value in LEGACY_LOCAL:
            legacy = LEGACY_LOCAL[value] | {"inflation_layer"}
            if set(eff) != legacy:
                problems.append(f"local_obstacle:={value} 生效图层 {sorted(eff)} "
                                f"≠ 改造前 {sorted(legacy)}")
            if flags.get("stvl_layer") is not False:
                problems.append(f"local_obstacle:={value} 下 stvl_layer 必须 enabled=false")
        if set(eff) != expect:
            problems.append(f"local_obstacle:={value} 生效图层 {sorted(eff)} ≠ 真值表 {sorted(expect)}")
        rows.append((value, plugins, flags, eff))

    grows = []
    for value in global_table:
        lc, gc = _materialize(mod, "scan", value)
        plugins = list(gc["plugins"])
        eff = effective_layers(gc)
        expect = {k for k, v in global_table[value].items() if v}
        if "static_layer" in plugins:
            expect |= {"static_layer"}
        expect |= {"inflation_layer"}
        if set(eff) != expect:
            problems.append(f"global_obstacle:={value} 生效图层 {sorted(eff)} ≠ 真值表 {sorted(expect)}")
        if set(eff) != (LEGACY_GLOBAL[value] | {"static_layer", "inflation_layer"}):
            problems.append(f"global_obstacle:={value} 生效图层与改造前不一致")
        grows.append((value, plugins, eff))

    if a.markdown:
        pass
    else:
        print("=" * 100)
        print("静态真值表（真的 RewrittenYaml + 真的 params 文件；唯一真值表来自 "
              "navigation_launch.py）")
        print("=" * 100)
        print("槽位值        生效图层（local_costmap）                        "
              "obstacle / cloud / stvl 三个 enabled 开关")
        for value, plugins, flags, eff in rows:
            sw = "/".join("T" if flags.get(k) else "F" for k in
                          ("obstacle_layer", "obstacle_cloud_layer", "stvl_layer"))
            print(f"  {value:<11} {', '.join(eff):<52} {sw:<10} "
                  f"plugins={plugins}")
        print()
        print("global_costmap（对照，未改机制）：")
        for value, plugins, eff in grows:
            print(f"  {value:<11} {', '.join(eff):<40} plugins={plugins}")
        print()

    if a.markdown:
        print("| `local_obstacle` | 生效图层（= enabled 为真的层） | obstacle / cloud / stvl |")
        print("|---|---|---|")
        for value, plugins, flags, eff in rows:
            sw = " / ".join("**开**" if flags.get(k) else "关" for k in
                            ("obstacle_layer", "obstacle_cloud_layer", "stvl_layer"))
            print(f"| `{value}` | {', '.join('`%s`' % e for e in eff)} | {sw} |")

    if problems:
        print("❌ 静态校验失败：")
        for p in problems:
            print("   ·", p)
        return 1
    if not a.markdown:
        print("✅ 全部断言通过：")
        print("   · scan/cloud/both 的生效图层与改造前**逐位一致**（stvl_layer 在这三个值下恒关）")
        print("   · stvl → 只开 stvl_layer；stvl_both → stvl + scan + cloud")
        print("   · global 的 stvl/scan/none 三个值不变")
        print("   · inflation_layer 在所有取值下恒在恒开；plugins 里每层都有 plugin: 段")
    return 0


if __name__ == "__main__":
    sys.exit(main())
