#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_param_sweep.py —— **不开 Gazebo / 不开 nav2** 的 LIO 离线重放 A/B 扫参驱动。

它把「起一个 LIO 节点 + 用 tools/lio_node_alone_check.py 重放同一个 bag + 收判读结果」
这一套动作自动化，并且**并行**跑多组参数（每组一个独立 ROS_DOMAIN_ID，互不串台），
最后打一张「config → 轨迹长度 / 末位置 / /odom 速率 / 与参照的比值」的表。

为什么需要它：
  · 全栈验证要起几十个节点，A/B 一个参数就是几分钟；离线重放一轮只要 `--duration` 秒；
  · 单轮结果有**随机性**（节点是单线程回调，DDS 投递/调度有抖动）⇒ 需要 `--repeat` 看重复性，
    并且必须把**所有 config 放在同一批并行里**跑，条件才可比。

用法（先 source 环境）：

    source /opt/ros/humble/setup.bash && source install/setup.bash

    # ① 单个配置（命令行直接给参数覆盖）
    python3 tools/lio_param_sweep.py --label trial1 \
        --param imu_meas_acc_cov:=0.6 --param point_filter_num:=2 --duration 40

    # ② 一组配置（JSON 列表；每组可指定 node / params / 覆盖）
    python3 tools/lio_param_sweep.py --configs sweep.json --jobs 4 --duration 40

    # ③ 每个配置重复 3 次，检验重复性
    python3 tools/lio_param_sweep.py --configs sweep.json --jobs 6 --repeat 3

sweep.json 的形状：

    [
      {"label": "baseline",  "params": {}},
      {"label": "accov0.6",  "params": {"imu_meas_acc_cov": 0.6}},
      {"label": "fastlio",   "node": "fast_lio", "params": {"common.imu_topic": "/livox/imu"}}
    ]

`params` 里的键值对会以 `-p key:=value` 追加在 `--params-file` 之后（ROS 2 参数覆盖的
正常语义：后者覆盖前者）。布尔/浮点/整型/列表都能写（列表按 `[a,b,c]` 传字符串）。

产物（默认在 `--outdir`，默认 `.tmp_sweep/<时间戳>/`）：
  · `<label>/node.log`       节点 stdout/stderr（含 RCLCPP 日志；失败时先看它）
  · `<label>/check.log`      判读脚本的完整输出
  · `<label>/check.json`     判读结果的机器可读版本（由 lio_node_alone_check.py --json-out 写）
  · `<label>/traj.json`      /odom 与 bag 参照的逐帧轨迹（诊断用）
  · `<label>/result.json`    驱动层汇总（含耗时、退出码）
  · `summary.json` / 终端表格
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import queue
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CHECKER = os.path.join(HERE, "lio_node_alone_check.py")

# 每个节点的默认启动口径（与 docs/lio_slots.md §5 的复现命令一致）
NODES = {
    "small_point_lio": {
        "package": "small_point_lio",
        "executable": "small_point_lio_node",
        "params": "install/small_point_lio/share/small_point_lio/config/mid360_sim.yaml",
        "extra_params": ["use_sim_time:=false", "save_pcd:=false"],
        # 节点发的是硬编码 `/Odometry`（launch 里 remap 成 /odom），离线要按原名订 ⇒ 用默认话题表
        "checker": ["--imu-qos", "sensor"],
    },
    "fast_lio": {
        "package": "fast_lio",
        "executable": "fastlio_mapping",
        # ⚠️ 三处必须对齐，否则"控制组"会**假失败**（2026-10-05 实测踩过）：
        #    ① 话题名：FAST-LIO 硬编码发 `/Odometry`（laserMapping.cpp:939），**不是** /odom
        #       —— launch 里那条 `('/Odometry','/odom')` 重映射只在全栈里生效；离线单跑节点要按原名订。
        #       （判读脚本的默认 --odom-topics 已含 /Odometry，所以这里**不要**覆盖它。）
        #    ② QoS：FAST-LIO 订 IMU 用 rclcpp 默认（RELIABLE）⇒ 判读端必须 --imu-qos reliable。
        #    ③ imu_topic：包内默认 /imu/data（互补滤波后），重放里只有 /livox/imu，必须覆盖。
        #    另外 pcd_save_en 关掉：ROOT_DIR 指向 install/share，开着会往安装树写 PCD。
        "params": "install/fast_lio/share/fast_lio/config/fastlio_mid360_sim.yaml",
        "extra_params": ["use_sim_time:=false", "common.imu_topic:=/livox/imu",
                         "pcd_save.pcd_save_en:=false"],
        "checker": ["--imu-qos", "reliable"],
    },
    "point_lio": {
        "package": "point_lio",
        "executable": "pointlio_mapping",
        "params": "install/point_lio/share/point_lio/config/pointlio_mid360_sim.yaml",
        "extra_params": ["use_sim_time:=false", "pcd_save.pcd_save_en:=false"],
        "checker": ["--imu-qos", "sensor", "--odom-topics", "/odom,/aft_mapped_to_init"],
    },
}


def fmt_value(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(repr(float(x)) for x in v) + "]"
    return str(v)


def _traj_arrays(traj_path):
    """读 --traj-out 的轨迹 JSON，返回 (node 轨迹, {参照名: 轨迹})，时间都**归零**（相对各自首帧）。

    为什么要归零：节点侧的时间戳是**重放的墙钟理想时刻**（几十亿秒），而 bag 里参照轨迹的时间戳是
    **仿真钟**（几百秒）—— 两套时间轴只在"相对时长"上有意义（重放是 1:1 实时）。
    """
    import numpy as np
    with open(traj_path) as fh:
        t = json.load(fh)
    def prep(rows):
        a = np.asarray(rows, dtype=float)
        if a.ndim != 2 or len(a) < 3:
            return None
        a = a[np.argsort(a[:, 0])]
        a = a.copy()
        a[:, 0] -= a[0, 0]
        return a
    nodes = {k: prep(v) for k, v in t.get("node", {}).items()}
    nodes = {k: v for k, v in nodes.items() if v is not None}
    refs = {k: prep(v) for k, v in t.get("ref", {}).items()}
    refs = {k: v for k, v in refs.items() if v is not None}
    return nodes, refs


def shape_metrics(traj_path):
    """与参照轨迹的**形状**一致度（判"是不是真的跟住了"，而不只是"路径变短了"）。

    做法：把节点轨迹与参照轨迹按**相对时间**重采样到同一串时刻，再做 **SE(2) 最小二乘对齐**
    （只允许旋转+平移，**不允许缩放** —— 缩放会把"轨迹长度错了"这件事洗掉），
    报 ATE RMSE / 末位置误差 / 对齐角。

    ⚠️ 这是**诊断指标**，不是判据：它与 `路径长度 / 末位置` 一起看，才能区分
       「真的跟住了」与「运气好凑出一个短路径」。
    """
    import numpy as np
    try:
        nodes, refs = _traj_arrays(traj_path)
    except Exception as exc:  # noqa: BLE001
        return {"error": f"{type(exc).__name__}: {exc}"}
    if not nodes:
        return {"error": "没有节点轨迹"}
    nk = max(nodes, key=lambda k: len(nodes[k]))
    n = nodes[nk]
    out = {"node_topic": nk, "n": int(len(n))}
    for rk, r in refs.items():
        ts = n[:, 0]
        m = ts <= r[-1, 0]
        if m.sum() < 10:
            continue
        ts = ts[m]
        nn = n[m][:, 1:3]
        rr = np.stack([np.interp(ts, r[:, 0], r[:, i]) for i in (1, 2)], 1)
        ca, cb = nn.mean(0), rr.mean(0)
        A, B = nn - ca, rr - cb
        U, S, Vt = np.linalg.svd(A.T @ B)
        D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        tt = cb - R @ ca
        al = (R @ nn.T).T + tt
        err = np.linalg.norm(al - rr, axis=1)
        key = rk.strip("/").replace("/", "_")
        out[f"ate_{key}"] = float(np.sqrt((err ** 2).mean()))
        out[f"enderr_{key}"] = float(err[-1])
        out[f"ang_{key}"] = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        out[f"n_{key}"] = int(len(ts))
    return out


def kill_tree(proc: subprocess.Popen, grace: float = 4.0):
    """杀整棵进程树（ros2 run 会 fork 出真正的节点）。"""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    t0 = time.time()
    while time.time() - t0 < grace:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
    try:
        proc.wait(timeout=3)
    except Exception:
        pass


def run_one(cfg: dict, slot: int, args, run_dir: str) -> dict:
    label = cfg["label"]
    node_key = cfg.get("node", args.node)
    spec = NODES[node_key]
    domain = str(args.domain_base + slot)
    params_file = cfg.get("params_file") or spec["params"]

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain
    env["ROS_LOG_DIR"] = os.path.join(run_dir, "roslog")
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)

    node_cmd = ["ros2", "run", spec["package"], spec["executable"], "--ros-args",
                "--params-file", params_file]
    for p in spec["extra_params"]:
        node_cmd += ["-p", p]
    for k, v in (cfg.get("params") or {}).items():
        node_cmd += ["-p", f"{k}:={fmt_value(v)}"]

    out = {"label": label, "node": node_key, "domain": domain, "params": cfg.get("params") or {},
           "params_file": params_file, "cmd": " ".join(node_cmd), "ok": False, "error": None}
    t0 = time.time()
    node_log = open(os.path.join(run_dir, "node.log"), "w")
    proc = subprocess.Popen(node_cmd, env=env, stdout=node_log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    check_log_path = os.path.join(run_dir, "check.log")
    json_out = os.path.join(run_dir, "check.json")
    traj_out = os.path.join(run_dir, "traj.json")
    try:
        time.sleep(args.settle)
        if proc.poll() is not None:
            raise RuntimeError(f"节点在 {args.settle}s 内就退出了（rc={proc.returncode}），看 node.log")
        # 每组可以自带 window（cfg.duration / cfg.start_offset 覆盖全局），
        # 因为"同一份 bag 的不同时间段"运动强度完全不同（见 docs/lio_slots.md 的泛化测试）。
        cfg_dur = float(cfg.get("duration", args.duration))
        cfg_off = float(cfg.get("start_offset", args.start_offset))
        out["duration"] = cfg_dur
        out["start_offset"] = cfg_off
        check_cmd = [sys.executable, CHECKER, "--bag", args.bag,
                     "--duration", str(cfg_dur), "--speed", str(args.speed),
                     "--start-offset", str(cfg_off),
                     "--json-out", json_out, "--traj-out", traj_out] + spec["checker"]
        r = subprocess.run(check_cmd, env=env, capture_output=True, text=True,
                           timeout=cfg_dur / max(args.speed, 1e-6) + args.check_timeout_pad)
        with open(check_log_path, "w") as fh:
            fh.write(r.stdout or "")
            if r.stderr:
                fh.write("\n===== stderr =====\n" + r.stderr)
        # ⚠️ 判读脚本**收尾**时会偶发 "terminate called without an active exception"（rc=-6）——
        #    那是 rclpy/tf2 在 shutdown 路径上的问题，**不影响已经写出的判读结果**。
        #    所以只要 check.json 存在就照常采用，只把 rc 记下来（否则会把有效数据误判成失败）。
        if r.returncode != 0:
            out["checker_rc"] = r.returncode
            if not os.path.exists(json_out):
                raise RuntimeError(f"判读脚本 rc={r.returncode} 且没写出 check.json（看 check.log）")
        with open(json_out) as fh:
            res = json.load(fh)
        out["result"] = res
        best = res.get("best") or {}
        out["ok"] = bool(best.get("n"))
        out["n"] = best.get("n")
        out["rate_hz"] = (best.get("n") or 0) / max(cfg_dur, 1e-9)
        out["path_m"] = best.get("path")
        out["end"] = best.get("last")
        out["ref_path_m"] = (res.get("ref") or {}).get("path")
        out["ref_end"] = (res.get("ref") or {}).get("last")
        out["ratio"] = res.get("ratio_vs_ref")
        if os.path.exists(traj_out):
            out["shape"] = shape_metrics(traj_out)
    except Exception as exc:  # noqa: BLE001
        out["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        kill_tree(proc)
        node_log.close()
    # 节点日志里的 ERROR/WARN 计数（判"链路是否有异常"用）
    try:
        with open(os.path.join(run_dir, "node.log"), errors="replace") as fh:
            txt = fh.read()
        out["node_errors"] = txt.count("[ERROR]")
        out["node_warns"] = txt.count("[WARN]")
    except Exception:
        out["node_errors"] = out["node_warns"] = None
    out["wall_s"] = round(time.time() - t0, 1)
    with open(os.path.join(run_dir, "result.json"), "w") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    return out


def main():
    ap = argparse.ArgumentParser(description="LIO 离线重放扫参驱动（并行、独立 ROS_DOMAIN_ID）")
    ap.add_argument("--configs", default=None, help="JSON 配置文件（列表；见文件头说明）")
    ap.add_argument("--label", default=None, help="单配置模式下的标签（与 --param 一起用）")
    ap.add_argument("--param", action="append", default=[], help="参数覆盖 k:=v（可重复）")
    ap.add_argument("--node", default="small_point_lio", choices=sorted(NODES),
                    help="用哪个 LIO 节点（默认 small_point_lio）")
    ap.add_argument("--jobs", type=int, default=3, help="并行度（默认 3；每个 job 一个 ROS_DOMAIN_ID）")
    ap.add_argument("--repeat", type=int, default=1, help="每个配置重复次数（看随机性用）")
    ap.add_argument("--duration", type=float, default=40.0, help="每组重放多少秒 bag 时间（默认 40）")
    ap.add_argument("--speed", type=float, default=1.0, help="重放倍速（默认 1.0；>1 会改变动力学，仅应急）")
    ap.add_argument("--start-offset", type=float, default=0.0, help="从 bag 起点跳过多少秒")
    ap.add_argument("--bag", default=".tmp_bags/ret4", help="bag 目录或 .db3（默认 .tmp_bags/ret4）")
    ap.add_argument("--outdir", default=None, help="产物目录（默认 .tmp_sweep/<时间戳>）")
    ap.add_argument("--settle", type=float, default=4.0, help="节点起来后等多久才开始喂（默认 4 s）")
    ap.add_argument("--check-timeout-pad", type=float, default=45.0,
                    help="判读脚本的超时余量（秒，默认 45）")
    ap.add_argument("--domain-base", type=int, default=140, help="ROS_DOMAIN_ID 基数（默认 140）")
    args = ap.parse_args()

    if args.configs:
        with open(args.configs) as fh:
            cfgs = json.load(fh)
    elif args.label:
        overrides = {}
        for item in args.param:
            if ":=" not in item:
                raise SystemExit(f"[FATAL] --param 需要 k:=v 形式，收到 {item!r}")
            k, v = item.split(":=", 1)
            try:
                overrides[k] = json.loads(v)
            except json.JSONDecodeError:
                overrides[k] = v
        cfgs = [{"label": args.label, "node": args.node, "params": overrides}]
    else:
        raise SystemExit("[FATAL] 需要 --configs FILE 或 --label L（--param 可选）")

    stamp = time.strftime("%m%d_%H%M%S")
    outdir = args.outdir or os.path.join(".tmp_sweep", stamp)
    os.makedirs(outdir, exist_ok=True)
    print(f"[info] 产物目录 {outdir}   并行度 {args.jobs}   每组 {args.duration}s (@{args.speed}x)   repeat {args.repeat}")

    # 域号用**队列**发放，不能按 idx % jobs 算：任务完成时刻不齐，取模会撞域（两个节点串台）。
    free_slots = queue.Queue()
    for i in range(args.jobs):
        free_slots.put(i)

    tasks = []
    for rep in range(args.repeat):
        for cfg in cfgs:
            label = cfg["label"] + (f"#r{rep + 1}" if args.repeat > 1 else "")
            tasks.append((label, cfg, rep))

    results = [None] * len(tasks)

    def worker(idx):
        label, cfg, rep = tasks[idx]
        slot = free_slots.get()
        run_dir = os.path.join(outdir, label.replace("/", "_"))
        if os.path.isdir(run_dir):
            shutil.rmtree(run_dir)
        os.makedirs(run_dir, exist_ok=True)
        cfg2 = dict(cfg)
        cfg2["label"] = label
        try:
            res = run_one(cfg2, slot, args, run_dir)
            results[idx] = res
            print(f"  [{label:28s}] path={res.get('path_m')} ratio={res.get('ratio')} "
                  f"err={res.get('error')}", flush=True)
        finally:
            free_slots.put(slot)

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        list(pool.map(worker, range(len(tasks))))
    wall = time.time() - t0

    # ---------------- 表格 ----------------
    def num(v, fmt="{:.2f}"):
        return "-" if v is None else fmt.format(v)

    print(f"\n================ 扫参结果（{len(tasks)} 组，{wall:.0f}s 墙钟） ================")
    hdr = (f"{'config':30s} {'节点':16s} {'n':>4s} {'Hz':>5s} {'轨迹m':>7s} "
           f"{'末位置(x,y)':>18s} {'参照m':>6s} {'比值':>5s} {'ATE真值':>7s} {'末误差':>7s} "
           f"{'err':>4s} {'s':>4s}")
    print(hdr)
    print("-" * len(hdr))
    for res in results:
        end = res.get("end")
        end_s = "-, -" if not end else f"{end[0]:.2f}, {end[1]:.2f}"
        sh = res.get("shape") or {}
        print(f"{res['label']:30s} {res['node']:16s} {str(res.get('n') or 0):>4s} "
              f"{num(res.get('rate_hz'), '{:.1f}'):>5s} {num(res.get('path_m'), '{:7.2f}'):>7s} "
              f"{end_s:>18s} {num(res.get('ref_path_m'), '{:6.2f}'):>6s} "
              f"{num(res.get('ratio'), '{:5.2f}'):>5s} "
              f"{num(sh.get('ate_odom_ground_truth'), '{:7.2f}'):>7s} "
              f"{num(sh.get('enderr_odom_ground_truth'), '{:7.2f}'):>7s} "
              f"{str(res.get('node_errors')):>4s} {str(res.get('wall_s')):>4s}")
        if res.get("error"):
            print(f"    ⚠️ {res['error']}")

    with open(os.path.join(outdir, "summary.json"), "w") as fh:
        json.dump({"args": vars(args), "results": results, "wall_s": wall}, fh,
                  ensure_ascii=False, indent=1)
    print(f"\n[json] 汇总 {os.path.join(outdir, 'summary.json')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
