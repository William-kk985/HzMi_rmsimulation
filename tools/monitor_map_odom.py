#!/usr/bin/env python3
"""map→odom 曲线监视器：判断"地图跟着车转 / 被矫正 / 高频抖动"到底是哪一种。

为什么需要它：`ros2 run tf2_ros tf2_echo map odom` 的打印间隔约 0.8 s（受 tf2_echo 内部
节流影响），既看不出高频抖动，也会把周期性信号采样成假波形。SLAM 调参时我们真正要
区分的是**三种完全不同的"动"**：

  ① 平滑小漂移（几度以内、随时间连续）        → 正常：这就是 map→odom 的职责（全局矫正量）
  ② 锯齿（单向慢慢涨到十几度，再被拉回）      → 局部 SLAM 在子图内走偏 + 全局事后矫正
                                               → **会留残影**（官方文档：坏掉的子图永久保留）
  ③ 高频抖动（逐样本就跳，几十 ms 一次）      → 局部匹配/时间戳/TF 的问题，不是"矫正"

本工具以固定频率采样并直接给出判读，不需要肉眼盯输出。

用法:
  python3 tools/monitor_map_odom.py                      # 仿真时间，50 Hz 采样，每 2 s 一行
  python3 tools/monitor_map_odom.py --csv /tmp/mo.csv    # 同时落 CSV（t,x,y,yaw_deg）
  python3 tools/monitor_map_odom.py --duration 120       # 120 s 后自动出总结并退出
  python3 tools/monitor_map_odom.py --wall-time          # Gazebo 没起来/没有 /clock 时用

判读（写在每行与结尾总结里）:
  * jumps（相邻样本 |Δyaw| > --jump-deg 的次数）：>0 表示存在"跳变式矫正"；
  * P95 |Δyaw|：大（比如 >0.2°/样本 @50Hz）说明高频抖动，去查局部匹配/时间戳；
  * 单向漂移率：用首尾差值/时长估计，慢速单向漂移是允许的，锯齿看 jumps + 峰峰值。

约定: 这是**只读**诊断工具，不发任何控制/参数（见 docs/sim_real_contract.md 的分类规则）。
"""

import argparse
import csv
import math
import statistics
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    import tf2_ros
except ImportError as exc:  # pragma: no cover
    print(f"需要 ROS 2 环境（source /opt/ros/humble/setup.bash 与 install/setup.bash）：{exc}")
    sys.exit(2)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(deg: float) -> float:
    """把角度差归一化到 (-180, 180]。"""
    return (deg + 180.0) % 360.0 - 180.0


class MapOdomMonitor(Node):
    def __init__(self, args):
        # use_sim_time 必须在构造时就定下来（parameter_overrides），否则 ROSClock 已经建好，
        # --wall-time 会失效 → 定时器永不触发（Gazebo 没跑时表现为"卡住没有任何输出"）。
        super().__init__(
            "monitor_map_odom",
            parameter_overrides=[Parameter("use_sim_time", value=not args.wall_time)],
        )
        self.args = args
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.samples = []          # (t, x, y, yaw_deg)
        self.misses = 0
        self.clock_warned = False
        self.start_wall = time.time()
        self.create_timer(1.0 / args.rate, self.tick)
        print(f"采样 {args.parent}→{args.child} @ {args.rate:g} Hz（"
              f"{'墙钟' if args.wall_time else '仿真钟'}），Ctrl-C 结束并打印总结")
        if not args.wall_time:
            print("  提示：使用仿真时间；若 Gazebo 没在跑、这里一直没输出，加 --wall-time")

    # ---------------------------------------------------------------- sampling
    def tick(self):
        if not self.clock_warned and time.time() - self.start_wall > 3.0 and not self.samples:
            self.clock_warned = True
            print("  ⚠️ 3 s 内没拿到变换：检查 launch 是否起了 cartographer、以及是否有 /clock", flush=True)
        try:
            tf = self.buffer.lookup_transform(self.args.parent, self.args.child, Time())
        except Exception:
            self.misses += 1                     # 拿不到也要继续走下面的 duration/报告逻辑
        else:
            tr = tf.transform.translation
            q = tf.transform.rotation
            t = self.get_clock().now().nanoseconds * 1e-9
            self.samples.append((t, tr.x, tr.y, math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w))))
            if self.args.report > 0 and len(self.samples) % max(1, int(self.args.rate * self.args.report)) == 0:
                self.summary_line(prefix="[运行中] ")
        if self.args.duration > 0 and (time.time() - self.start_wall) >= self.args.duration:
            self.finish()

    # ---------------------------------------------------------------- reporting
    def _stats(self):
        ys = [s[3] for s in self.samples]
        xs = [s[1] for s in self.samples]
        yy = [s[2] for s in self.samples]
        deltas = [abs(wrap_deg(ys[i] - ys[i - 1])) for i in range(1, len(ys))]
        jumps = sum(1 for d in deltas if d > self.args.jump_deg)
        p95 = sorted(deltas)[int(0.95 * (len(deltas) - 1))] if deltas else 0.0
        med = statistics.median(deltas) if deltas else 0.0
        maxd = max(deltas) if deltas else 0.0
        span = (self.samples[-1][0] - self.samples[0][0]) if len(self.samples) > 1 else 0.0
        return {
            "n": len(self.samples), "ys": ys, "xs": xs, "yy": yy, "deltas": deltas,
            "jumps": jumps, "p95": p95, "med": med, "maxd": maxd, "span": span,
        }

    def summary_line(self, prefix=""):
        st = self._stats()
        if st["n"] < 2:
            return
        yaw_pp = max(st["ys"]) - min(st["ys"])
        print(f"{prefix}n={st['n']:<6d} yaw[min/max/pp]="
              f"{min(st['ys']):7.2f}/{max(st['ys']):7.2f}/{yaw_pp:6.2f}° "
              f"std={statistics.pstdev(st['ys']):5.2f}° "
              f"|Δyaw|P95={st['p95']:5.3f}° jumps={st['jumps']:<4d} "
              f"xy=[{min(st['xs']):.2f}..{max(st['xs']):.2f}, {min(st['yy']):.2f}..{max(st['yy']):.2f}]")

    def finish(self):
        if getattr(self, "_finished", False):
            return
        self._finished = True
        st = self._stats()
        if st["n"] < 2:
            print(f"样本不足（{st['n']} 个，丢失 {self.misses} 次）→ 没拿到 map→odom")
            if rclpy.ok():
                rclpy.shutdown()
            return
        ys, span = st["ys"], st["span"]
        yaw_pp = max(ys) - min(ys)
        drift = (wrap_deg(ys[-1] - ys[0]) / span) if span > 0 else float("nan")
        print("\n================= 总结 =================")
        print(f"样本 {st['n']}，时长 {span:.1f} s，丢失 {self.misses} 次")
        print(f"yaw: 首 {ys[0]:.2f}° 末 {ys[-1]:.2f}° 峰峰值 {yaw_pp:.2f}° std {statistics.pstdev(ys):.2f}°")
        print(f"单向漂移率 ≈ {drift:+.3f} °/s（{drift * 60:+.2f} °/min）")
        print(f"跳变（|Δyaw|>{self.args.jump_deg:g}°）{st['jumps']} 次；|Δyaw| 中位 {st['med']:.3f}° / "
              f"P95 {st['p95']:.3f}° / max {st['maxd']:.3f}°")
        print("判读：", end="")
        # 关键区分：**逐样本的 P95 就在大范围动** = 高频抖动；
        # 而"锯齿"的特征是"中位数很小 + 极少数大跳变"。
        if st["p95"] > self.args.jump_deg:
            print(f"**高频抖动**（逐样本 P95 {st['p95']:.2f}° 已超过 {self.args.jump_deg:g}°）→ "
                  "查局部匹配/时间戳/扫描频率，这不是「矫正」问题")
        elif st["jumps"] > 0:
            print(f"**跳变式矫正**（{st['jumps']} 次大跳变，中位仅 {st['med']:.3f}°，max {st['maxd']:.2f}°）"
                  "→ 若同时看到残影，属「锯齿」型；按 runbook §5 先局部后全局继续收敛")
        else:
            print("**平滑**（无大跳变、逐样本变化小）→ 属正常全局矫正；"
                  "地图若仍在动，看的是子图重绘而不是 map→odom")
        print("=======================================")
        if self.args.csv and self.samples:
            with open(self.args.csv, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "x", "y", "yaw_deg"])
                for row in self.samples:
                    w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}"])
            print(f"CSV 已写入 {self.args.csv}")
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="监视 map→odom 并判读：平滑漂移 / 锯齿矫正 / 高频抖动")
    ap.add_argument("--parent", default="map", help="父帧（默认 map）")
    ap.add_argument("--child", default="odom", help="子帧（默认 odom）")
    ap.add_argument("--rate", type=float, default=50.0, help="采样频率 Hz（默认 50）")
    ap.add_argument("--report", type=float, default=2.0, help="每隔多少秒打印一行（0=不打印，默认 2）")
    ap.add_argument("--csv", default="", help="把样本写到该 CSV 文件 t,x,y,yaw_deg")
    ap.add_argument("--duration", type=float, default=0.0, help="多少秒后自动总结退出（0=一直跑）")
    ap.add_argument("--jump-deg", type=float, default=0.5, help="相邻样本 yaw 变化超过多少度算一次跳变")
    ap.add_argument("--wall-time", action="store_true", help="用墙钟（默认用仿真时间；无 /clock 时加这个）")
    args = ap.parse_args()
    try:                       # 管道/重定向时也要能实时看到输出
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    rclpy.init()
    node = MapOdomMonitor(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:  # 关停竞态（SIGTERM / context 已失效）不该打 traceback
        if rclpy.ok():
            raise
        print(f"（ROS 上下文已关闭：{type(exc).__name__}）")
    finally:
        node.finish()          # 幂等：内部有 _finished 保护，并自行判断 rclpy.ok()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
