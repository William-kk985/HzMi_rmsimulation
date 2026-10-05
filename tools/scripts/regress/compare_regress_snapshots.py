#!/usr/bin/env python3
"""把 P0 回归快照聚合成一张对比表（重定位 amcl/icp/gicp 三方对比用一条命令搞定）。

它只读 `.tmp_bags/regress_*.json`（`nav_smoke_regression.py` 写的快照），**不启动任何东西、
不改任何配置**：跑完三次回归后执行本工具，就能把三行结果并排看清。

一行 = 一个快照，列：时间戳/文件 · 方法(`localization`) · goal · pass · result/exit_code ·
status · 用时 · d_min · recoveries · 命令链 nav/smooth/chassis 的 max|v|,|w| · pose_delta/真值 ·
settle(settled/漂移 xy/yaw) · map_check.state · yaw_source（`--full` 再加 rtf/fp_distinct/tf_age/hz）。

**重要前提**：快照本身**不记录** `localization`（见 `nav_smoke_regression.py` 的 snap 字典）。
所以"方法"列默认是 `?`，并会打印一条提示——**要得到能读的表，跑之前请自己记住哪次是哪次**，
或者用 `--label` 事后标注：

  # 三次跑完后（顺序=快照时间顺序）
  python3 tools/scripts/regress/compare_regress_snapshots.py \\
      --label regress_1791167204=gicp --label regress_1791167300=amcl --label regress_1791167400=icp

  # 或者子串匹配 + 只看最近 3 个
  python3 tools/scripts/regress/compare_regress_snapshots.py --last 3 \\
      --label 1791167204=gicp --label 1791167300=amcl

`--label NAME=METHOD` 的 NAME 可以是文件名、文件名里的时间戳数字、或文件名的一段唯一子串。
将来若快照里加了 `localization` / `loc` / `relocalization` 字段（或 notes 里写了
`localization=xxx`），本工具会**自动**读出来，无需 `--label`。

用法：
  python3 tools/scripts/regress/compare_regress_snapshots.py                      # 终端对齐表
  python3 tools/scripts/regress/compare_regress_snapshots.py --markdown           # Markdown 表（打印）
  python3 tools/scripts/regress/compare_regress_snapshots.py --markdown out.md    # Markdown 表（打印并写文件）
  python3 tools/scripts/regress/compare_regress_snapshots.py --last 3 --full      # 最近 3 条 + 附加列
  python3 tools/scripts/regress/compare_regress_snapshots.py --glob '.tmp_bags/regress_179*.json'

兼容性：旧快照缺的字段（`result`/`exit_code`/`settle`/`map_check`/`yaw_source` …）一律打印 `-`；
`status`/`用时`/`recoveries` 若没有顶层字段，会退化到解析 notes 里的
`结果 status=... 用时=...s d_min=...m recoveries=...` 那一行（旧快照只有这一处有）。

退出码：0=正常（哪怕有 FAIL 快照）；1=一个快照都没读到（glob 打空 / 全部 JSON 坏）。
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os
import re
import sys
from datetime import datetime

# tools/scripts/regress/compare_regress_snapshots.py -> 仓库根
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
DEFAULT_GLOB = os.path.join(REPO_ROOT, ".tmp_bags", "regress_*.json")

TS_RE = re.compile(r"regress_(\d+)\.json$")
NOTES_RESULT_RE = re.compile(
    r"结果\s*status=(\S+)\s*用时=([0-9.]+)s\s*d_min=(\S+?)m\s*recoveries=(\d+)")
NOTES_SETTLE_RE = re.compile(r"定位在\s*([0-9.]+)s\s*内未稳定（漂移\s*xy=([0-9.]+)\s*yaw=([0-9.]+)）")
NOTES_YAW_RE = re.compile(r"目标朝向\s*yaw=(\S+)\s*rad\s*\((\S+)°\)\s*source=(\S+)")
NOTES_CHAIN_RE = re.compile(r"命令链\s*max\|v\|,\|w\|:\s*nav=\(([-\d.]+),([-\d.]+)\)\s*"
                            r"smooth=\(([-\d.]+),([-\d.]+)\)\s*chassis=\(([-\d.]+),([-\d.]+)\)")
METHOD_RE = re.compile(r"(?:localization|relocalization)\s*[=:]\s*([A-Za-z_][\w.+-]*)")
METHOD_KEYS = ("localization", "loc", "relocalization", "localization_mode", "localization_slot")
MISSING = "-"


# ---------------------------------------------------------------- 取值小工具
def get(doc, *keys, default=None):
    """按顺序取第一个存在且非 None 的键（快照字段名有演进，这里做兼容）。"""
    for k in keys:
        v = doc.get(k)
        if v is not None:
            return v
    return default


def fmt(v, nd=2):
    if v is None:
        return MISSING
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return ("%%.%df" % nd) % v
    return str(v)


def hhmm(ts):
    try:
        return datetime.fromtimestamp(float(ts)).strftime("%m-%d %H:%M")
    except (ValueError, OSError, OverflowError):
        return "??-?? ??:??"


def notes_of(doc):
    n = doc.get("notes")
    return [str(x) for x in n] if isinstance(n, list) else []


def parse_notes(doc):
    """旧快照没有顶层 status/用时/recoveries，只能从 notes 那一行抠。"""
    out = {"status": None, "elapsed": None, "recoveries": None, "d_min_note": None,
           "settle_note": None, "yaw_note": None}
    for line in notes_of(doc):
        m = NOTES_RESULT_RE.search(line)
        if m:
            out["status"] = m.group(1)
            out["elapsed"] = float(m.group(2))
            if m.group(3) not in ("None", "-", "?"):
                try:
                    out["d_min_note"] = float(m.group(3))
                except ValueError:
                    pass
            out["recoveries"] = int(m.group(4))
            continue
        m = NOTES_SETTLE_RE.search(line)
        if m:
            out["settle_note"] = "未稳定 %.1fs/xy=%s/yaw=%s" % (float(m.group(1)), m.group(2), m.group(3))
            continue
        m = NOTES_YAW_RE.search(line)
        if m:
            out["yaw_note"] = "yaw=%s(%s°) source=%s" % (m.group(1), m.group(2), m.group(3))
    return out


def infer_method(doc, text_blob):
    for k in METHOD_KEYS:
        v = doc.get(k)
        if isinstance(v, str) and v.strip():
            return v.strip(), "字段 %s" % k
        if isinstance(v, dict):
            for kk in ("value", "method", "name", "slot"):
                vv = v.get(kk)
                if isinstance(vv, str) and vv.strip():
                    return vv.strip(), "字段 %s.%s" % (k, kk)
    m = METHOD_RE.search(text_blob)
    if m:
        return m.group(1), "notes 文本"
    return None, None


def label_for(path, ts, labels):
    base = os.path.basename(path)
    for key, val in labels:
        if key == base or key == str(ts) or key == os.path.abspath(path) or (key and key in base):
            return val
    return None


# ---------------------------------------------------------------- 读快照
def build_row(path, labels):
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)
    if not isinstance(doc, dict):
        raise ValueError("顶层不是 JSON 对象")

    base = os.path.basename(path)
    m = TS_RE.search(base)
    ts = int(m.group(1)) if m else int(os.path.getmtime(path))
    notes = parse_notes(doc)

    blob = " ".join(notes_of(doc)) + " " + json.dumps(doc, ensure_ascii=False)
    method, method_src = infer_method(doc, blob)
    manual = label_for(path, ts, labels)
    if manual:
        method, method_src = manual, "--label"

    settle = get(doc, "settle", default={}) or {}
    mapck = get(doc, "map_check", default={}) or {}
    cmd = get(doc, "cmd", default={}) or {}

    def pair(name):
        v = cmd.get(name)
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            return "(%s,%s)" % (fmt(v[0]), fmt(v[1]))
        return MISSING

    goal = get(doc, "goal")
    goal_s = "(%s,%s)" % (fmt(goal[0]), fmt(goal[1])) if isinstance(goal, (list, tuple)) and len(goal) >= 2 else MISSING

    d_min = get(doc, "d_min", default=notes["d_min_note"])
    rec = get(doc, "recoveries", default=notes["recoveries"])

    if settle.get("enabled") is False:
        settle_s = "off"
    elif not settle:
        settle_s = MISSING
    else:
        settled = settle.get("settled")
        mark = "✅" if settled is True else ("✗" if settled is False else MISSING)
        settle_s = "%s %s/%s" % (mark, fmt(settle.get("drift_xy"), 4), fmt(settle.get("drift_yaw"), 4))

    if mapck.get("enabled") is False:
        map_s = "off"
    else:
        map_s = mapck.get("state") or (MISSING if mapck else MISSING)

    return {
        "file": base,
        "path": path,
        "ts": ts,
        "when": hhmm(ts),
        "method": method or "?",
        "method_src": method_src,
        "goal": goal_s,
        "pass": doc.get("pass"),
        "pass_s": "✅" if doc.get("pass") is True else ("❌" if doc.get("pass") is False else MISSING),
        "result": get(doc, "result", default=MISSING),
        "exit_code": get(doc, "exit_code", default=MISSING),
        "status": get(doc, "status", default=notes["status"]),
        "elapsed": get(doc, "elapsed", "elapsed_sec", default=notes["elapsed"]),
        "d_min": d_min,
        "recoveries": rec,
        "nav": pair("nav"),
        "smooth": pair("smooth"),
        "chassis": pair("chassis"),
        "gt_pose_delta": get(doc, "gt_pose_delta"),
        "gt_twist_max": get(doc, "gt_twist_max"),
        "settle": settle_s,
        "map_state": map_s,
        "yaw_source": get(doc, "yaw_source", default="auto-note" if notes["yaw_note"] else None),
        "spin_speed": get(doc, "spin_speed"),
        "rtf": get(doc, "rtf"),
        "fp_distinct": get(doc, "fp_distinct"),
        "tf_age": get(doc, "tf_age"),
        "hz": get(doc, "hz", default={}),
        "fails": doc.get("fails") or [],
    }


# ---------------------------------------------------------------- 渲染
def columns(full):
    cols = [
        ("时间戳/文件", lambda r: "%s %s" % (r["when"], r["file"])),
        ("方法", lambda r: r["method"]),
        ("goal", lambda r: r["goal"]),
        ("pass", lambda r: r["pass_s"]),
        ("result/exit", lambda r: "%s/%s" % (fmt(r["result"], 0), fmt(r["exit_code"], 0))),
        ("status", lambda r: fmt(r["status"], 0)),
        ("用时(s)", lambda r: fmt(r["elapsed"], 1)),
        ("d_min(m)", lambda r: fmt(r["d_min"], 3)),
        ("rec", lambda r: fmt(r["recoveries"], 0)),
        ("nav|v|,|w|", lambda r: r["nav"]),
        ("smooth", lambda r: r["smooth"]),
        ("chassis", lambda r: r["chassis"]),
        ("poseΔ(m)", lambda r: fmt(r["gt_pose_delta"], 3)),
        ("真值twist", lambda r: fmt(r["gt_twist_max"], 3)),
        ("settle(settled/xy/yaw)", lambda r: r["settle"]),
        ("map", lambda r: r["map_state"]),
        ("yaw源", lambda r: fmt(r["yaw_source"], 0)),
    ]
    if full:
        cols += [
            ("spin", lambda r: fmt(r["spin_speed"])),
            ("rtf", lambda r: fmt(r["rtf"], 3)),
            ("fp戳数", lambda r: fmt(r["fp_distinct"], 0)),
            ("tf_age", lambda r: fmt(r["tf_age"], 3)),
            ("hz(pcloud/imu/scan/odom)",
             lambda r: "/".join(fmt((r["hz"] or {}).get(k), 1) for k in ("pcloud", "imu", "scan", "odom"))),
        ]
    return cols


def render_plain(rows, cols):
    widths = []
    for title, fn in cols:
        w = max([len(title)] + [len(fn(r)) for r in rows])
        widths.append(min(w, 46))
    out = []
    out.append("  ".join(t.ljust(w) for (t, _), w in zip(cols, widths)).rstrip())
    out.append("  ".join("-" * w for w in widths))
    for r in rows:
        out.append("  ".join(fn(r)[:w].ljust(w) for (_, fn), w in zip(cols, widths)).rstrip())
    return "\n".join(out)


def render_markdown(rows, cols):
    out = ["| " + " | ".join(t.replace("|", "\\|") for t, _ in cols) + " |",
           "|" + "|".join("---" for _ in cols) + "|"]
    for r in rows:
        out.append("| " + " | ".join(fn(r).replace("|", "\\|") for _, fn in cols) + " |")
    return "\n".join(out)


def footer(rows, cols):
    lines = []
    n_pass = sum(1 for r in rows if r["pass"] is True)
    n_fail = sum(1 for r in rows if r["pass"] is False)
    lines.append("共 %d 个快照：PASS %d / FAIL %d（`-` = 该快照没有这个字段；`?` = 快照没记录）"
                 % (len(rows), n_pass, n_fail))

    unknown = [r for r in rows if r["method"] == "?"]
    if unknown:
        lines.append("⚠ 方法列有 %d 行为 `?`：**回归快照不记录 `localization`**"
                     "（`nav_smoke_regression.py` 的 snap 字典里没有这个字段）。"
                     % len(unknown))
        lines.append("   ⇒ 跑三方对比时请自己记住哪次是哪次（顺序=时间顺序），"
                     "或用 `--label <文件名或时间戳数字>=<方法>` 事后标注，例如：")
        lines.append("     python3 tools/scripts/regress/compare_regress_snapshots.py "
                     "--label %s=gicp %s=amcl %s=icp"
                     % (rows[0]["file"], rows[min(1, len(rows) - 1)]["file"],
                        rows[min(2, len(rows) - 1)]["file"]))
    else:
        srcs = sorted({r["method_src"] for r in rows if r["method_src"]})
        lines.append("方法来源：%s" % ("、".join(srcs) if srcs else "—"))

    for r in rows:
        if r["settle"].startswith("✗"):
            lines.append("· %s：settle 未达标（%s）——静止漂移略超工具默认阈值（0.02 m/0.01 rad）时"
                         "应视为**定位噪声底参考**，不代表定位坏了。" % (r["file"], r["settle"]))
    for r in rows:
        if r["fails"]:
            lines.append("· %s 首要断点：%s" % (r["file"], r["fails"][0]))
    return lines


# ---------------------------------------------------------------- main
def parse_label(s):
    if "=" not in s:
        raise argparse.ArgumentTypeError("--label 需要 NAME=METHOD 形式，得到 %r" % s)
    k, v = s.split("=", 1)
    if not k.strip() or not v.strip():
        raise argparse.ArgumentTypeError("--label 的 NAME/METHOD 都不能为空：%r" % s)
    return k.strip(), v.strip()


def main():
    ap = argparse.ArgumentParser(
        description="把 .tmp_bags/regress_*.json 回归快照聚合成一张对比表"
                    "（amcl/icp/gicp 三方对比用；只读文件，不启动任何节点）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例：
  # 1) 同一个 world/同一目标，分别用 localization:=gicp / amcl / icp 各跑一次回归
  # 2) 然后（快照按时间顺序排，先跑的在上）：
  python3 tools/scripts/regress/compare_regress_snapshots.py --last 3 --markdown

  # 快照不记 localization ⇒ 想看到方法列就自己标（顺序=时间顺序）：
  python3 tools/scripts/regress/compare_regress_snapshots.py --last 3 \\
      --label regress_1791167204=gicp --label regress_1791167300=amcl --label regress_1791167400=icp

  # 只挑某些快照 / 加附加列（rtf、fp 戳数、tf_age、hz）
  python3 tools/scripts/regress/compare_regress_snapshots.py --glob '.tmp_bags/regress_179116*.json' --full

退出码：0=正常；1=没读到任何快照。
""")
    ap.add_argument("--glob", default=DEFAULT_GLOB,
                    help="快照 glob（默认 %s；相对路径按当前目录解析）" % DEFAULT_GLOB)
    ap.add_argument("--last", type=int, default=None, metavar="N",
                    help="只看最近 N 个快照（按快照时间戳升序取尾部）")
    ap.add_argument("--label", action="append", default=[], type=parse_label, metavar="NAME=METHOD",
                    help="手动标注某个快照的定位方法（NAME=文件名/时间戳数字/文件名子串）；可重复")
    ap.add_argument("--markdown", nargs="?", const="-", default=None, metavar="[FILE]",
                    help="输出 Markdown 表；给 FILE 则同时写入该文件（不给 = 只打印）")
    ap.add_argument("--full", action="store_true", help="加附加列：spin/rtf/fp 戳数/tf_age/hz")
    ap.add_argument("--no-footer", action="store_true", help="不打印脚注（提示/断点）")
    a = ap.parse_args()

    pattern = a.glob if os.path.isabs(a.glob) else os.path.join(os.getcwd(), a.glob)
    files = sorted(globmod.glob(pattern))
    rows, bad = [], []
    for p in files:
        try:
            rows.append(build_row(p, a.label))
        except (ValueError, KeyError, TypeError, OSError, json.JSONDecodeError) as e:
            bad.append("%s（%s）" % (os.path.basename(p), e))
    rows.sort(key=lambda r: (r["ts"], r["file"]))
    if a.last and a.last > 0:
        rows = rows[-a.last:]
    for b in bad:
        print("[跳过] 读不了：%s" % b, file=sys.stderr)

    if not rows:
        print("[错] 没读到任何快照。glob=%s" % pattern, file=sys.stderr)
        print("     （先跑 tools/scripts/regress/nav_smoke_regression.py 产生 .tmp_bags/regress_*.json）",
              file=sys.stderr)
        return 1

    cols = columns(a.full)
    print("回归快照对比（%d 行；时间顺序）" % len(rows))
    print(render_plain(rows, cols))
    if not a.no_footer:
        print("")
        for line in footer(rows, cols):
            print(line)

    if a.markdown is not None:
        md = render_markdown(rows, cols)
        print("")
        print("---- Markdown ----")
        print(md)
        if a.markdown not in ("-", ""):
            with open(a.markdown, "w", encoding="utf-8") as f:
                f.write(md + "\n")
            print("\n[写出] %s" % os.path.abspath(a.markdown))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[中断]")
        raise SystemExit(130)
