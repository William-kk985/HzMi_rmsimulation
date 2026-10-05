#!/usr/bin/env python3
"""P0 回归：一条命令回答"这套槽位组合能不能正常走完一次导航，断点在哪"。

它做五件事（都不改算法/参数，只订阅 + 发一个目标点）：
  0) 等链路就绪：/livox/lidar/pointcloud、/livox/imu、/scan、/odom、TF(odom->base_link, map->odom)、
     costmap footprint —— 每项都要出现且频率达标；否则直接给出「第一个断点」。
  0.5) **把"发目标"这件事做成人在 RViz 里的语义**（2026-09 新增，之前三点都不对，会产生误导性 FAIL）：
       · 等定位稳定（--settle，默认 3 s）：map→odom 在最近 --settle 秒内平移漂移 ≤0.02 m、
         yaw 漂移 ≤0.01 rad 才继续。人在 RViz 里也是等定位不飘了才点 2D Goal Pose；
         在重定位收敛瞬态里发目标会被 ABORT / 走歪，那不是导航本身的问题。
       · 目标可用性预检（--no-precheck 跳过）：读先验地图 /map（RELIABLE + TRANSIENT_LOCAL，
         map_server 是 latched）把目标格分类 free/occupied/unknown/out_of_map；非 free 直接
         报"请换一个 free 点"并 **exit 3、不发目标**（这不叫导航失败）。
       · 目标朝向（--yaw，默认 auto）：auto = **车当前朝向**，由三条 TF 平面复合得到
         （map->odom ∘ odom->base_link ∘ base_link->base_link_fake），而不是原来写死的 yaw=0
         （"到点朝地图东"）；人在 RViz 里拖箭头默认就是朝车头方向。
  1) 发目标（默认 -1.0, 2.0，可用 --goal 指定；--skip-goal 只做链路自检）。
  2) 四跳命令链断言：/cmd_vel_nav → /cmd_vel → /cmd_vel_chassis，并用 /odom_ground_truth（仿真真值）
     确认车真的动了。**这条判据就是当初抓到 fake_vel_transform 丢角速度的那个判据。**
  3) 导航结果断言：distance_remaining 递减并到达；**拒绝"假到达"**——若结果 <2 s 就 SUCCEEDED
     而距离残余还 >0.5 m ⇒ FAIL（这正是我们花一天追的那个 bug 的特征）。
  4) 输出 PASS/FAIL + 首个断点 + 关键数值，并写 .tmp_bags/regress_<ts>.json（可直接抄进
     docs/algorithm_matrix.md §四 实测状态表）。

用法（栈要先跑起来；本脚本会等它）：
  python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0
  python3 tools/scripts/regress/nav_smoke_regression.py --goal 1.0 -1.0              # 复跑曾误导的用例
  python3 tools/scripts/regress/nav_smoke_regression.py --goal -1.0 2.0 --yaw 1.57 --settle 5
  python3 tools/scripts/regress/nav_smoke_regression.py --skip-goal          # 只体检链路
  python3 tools/scripts/regress/nav_smoke_regression.py --no-precheck        # 跳过目标可用性预检
  python3 tools/scripts/regress/nav_smoke_regression.py --ready-timeout 240 --goal-timeout 180

退出码：0=PASS；1=FAIL（链路/导航，见 fails）；**3=目标在图上是 occupied/unknown/图外，未发目标**
（"这不是导航失败"）；2 未被本工具使用。

判据阈值集中在下面 TH 字典里，可按需调（例如换了 world / 起始点）。
"""
import argparse
import json
import math
import os
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
RELI = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
# 先验地图：与 map_server / segment_goal_navigator.py 的订阅端一致（latched）
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

# 三段 TF：map 系里的车体位姿 = 依次平面复合这三段（fake_vel_transform 只发纯 yaw 的 base_link→base_link_fake）
FULL_CHAIN = ("map->odom", "odom->base_link", "base_link->base_link_fake")
POS_CHAIN = ("map->odom", "odom->base_link")      # 只用来拿"车在地图里的位置"（算朝目标的方向）

EXIT_PASS, EXIT_FAIL, EXIT_GOAL_UNUSABLE = 0, 1, 3

# 阈值（判据集中在此，便于按 world/起始点调整）
TH = {
    "clock_rtf_min": 0.35,        # /clock 相对墙钟的实时因子下限
    "hz_pcloud_min": 4.0,         # 名义 10 Hz；RTF≈0.7 时约 7 Hz
    "hz_imu_min": 40.0,           # 名义 100 Hz
    "hz_scan_min": 4.0,
    "hz_odom_min": 4.0,
    "tf_age_max": 3.0,            # TF 最新戳落后 /clock 的上限（秒）
    "footprint_min_distinct": 3,  # footprint 戳至少要变过几次（证明位姿在更新）
    "fake_success_secs": 2.0,     # 结果快于此且距离没减 ⇒ 判"假到达"
    "fake_success_dist": 0.5,     # 距离残余大于此值 ⇒ 没真到
    "arrive_tol": 0.35,           # 判定"到了"的距离阈值（goal checker 0.25 + 余量）
    "mv_eps": 1e-3,               # 命令链非零判定
    "settle_dxy": 0.02,           # settle 窗口内 map→odom 平移漂移上限（峰峰值，m）
    "settle_dyaw": 0.01,          # settle 窗口内 map→odom yaw 漂移上限（峰峰值，rad）
}


# ---------------------------------------------------------------- 纯函数（不依赖 ROS，可单测）
# TF 记录统一存成 (stamp, (x, y, z), (qx, qy, qz, qw))；本仓库所有相关 TF 都是平面的
# （roll/pitch ≈ 0），所以只用 yaw 做平面复合 —— 刻意不引入 tf_transformations 依赖。
def wrap_angle(t):
    """把角度归一到 (-pi, pi]。"""
    return math.atan2(math.sin(t), math.cos(t))


def yaw_from_quaternion(q):
    """四元数 (x, y, z, w) → 平面 yaw（绕 z 轴）。"""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def planar_of(T):
    """单条 TF 记录 (stamp, (x,y,z), (qx,qy,qz,qw)) → (x, y, yaw)。"""
    _, t, q = T
    return (t[0], t[1], yaw_from_quaternion(q))


def compose_planar(a, b):
    """平面复合 a∘b：p = p_a + R(yaw_a)·p_b，yaw = yaw_a + yaw_b（a、b 都是 (x, y, yaw)）。"""
    xa, ya, ta = a
    xb, yb, tb = b
    c, s = math.cos(ta), math.sin(ta)
    return (xa + c * xb - s * yb, ya + s * xb + c * yb, wrap_angle(ta + tb))


def compose_chain(tf, keys):
    """按 keys 顺序复合若干段 TF ⇒ 末帧在首帧里的 (x, y, yaw)；缺任一段返回 None。

    tf: {key: (stamp, (x,y,z), (qx,qy,qz,qw))}，key 形如 "map->odom"（顺序即父子方向）。
    """
    pose = (0.0, 0.0, 0.0)
    for k in keys:
        T = tf.get(k)
        if T is None:
            return None
        pose = compose_planar(pose, planar_of(T))
    return pose


def choose_goal_yaw(tf, goal, yaw_arg, full_chain=FULL_CHAIN, pos_chain=POS_CHAIN):
    """决定发目标用的平面 yaw（人类在 RViz 里拖 2D Goal Pose 的语义）。

    返回 (yaw, source, why)，source ∈ {"explicit", "tf", "to_goal", "fallback"}：
      · explicit —— 用户给了 --yaw <rad>（A/B 用，原"写死朝向"的行为可这样复现：--yaw 0）
      · tf       —— auto：三条 TF 平面复合得到车当前朝向（首选，与人类习惯一致）
      · to_goal  —— auto 但 TF 位姿还不全：退回"车→目标"的方向
      · fallback —— 连车的位置都没有：最后兜底 yaw=0
    """
    if yaw_arg != "auto":
        return float(yaw_arg), "explicit", f"--yaw {float(yaw_arg):.3f} 显式指定（非 auto）"
    pose = compose_chain(tf, full_chain)
    if pose is not None:
        return pose[2], "tf", ("车当前朝向：由 TF 链 %s 平面复合得到 yaw=%.3f rad"
                               % (" ∘ ".join(full_chain), pose[2]))
    base = compose_chain(tf, pos_chain)
    if base is not None:
        dx, dy = goal[0] - base[0], goal[1] - base[1]
        if math.hypot(dx, dy) > 1e-6:
            y = wrap_angle(math.atan2(dy, dx))
            return y, "to_goal", ("TF 位姿还不全（缺 %s）⇒ 退回『车→目标』方向 yaw=%.3f rad"
                                  % (",".join(k for k in full_chain if k not in tf) or "?", y))
    return 0.0, "fallback", ("TF 位姿还没建立（缺 %s）⇒ 最后兜底 yaw=0"
                             % (",".join(k for k in full_chain if k not in tf) or "?"))


def goal_cell_state(grid, x, y):
    """把目标点 (x, y) 落到**先验地图**栅格上分类。

    grid = (resolution, width, height, origin_x, origin_y, data)；
    data 取值约定与 tools/scripts/nav/segment_goal_navigator.py 的 /map 分支一致：
    0=free、100=occupied、-1=unknown（负值一律当 unknown）。
    返回 (state, value, (cx, cy))，state ∈ {"free", "occupied", "unknown", "out_of_map"}。
    注：落格用 floor（参考脚本用的 int() 会把"原点外侧一点"截断成第 0 行/列，误判成在图内）。
    """
    res, w, h, ox, oy, data = grid
    cx = int(math.floor((x - ox) / res))
    cy = int(math.floor((y - oy) / res))
    if not (0 <= cx < w and 0 <= cy < h):
        return "out_of_map", None, (cx, cy)
    v = int(data[cy * w + cx])
    if v < 0:
        return "unknown", v, (cx, cy)
    return ("free" if v == 0 else "occupied"), v, (cx, cy)


def check_settled(samples, settle_sec, max_dxy, max_dyaw):
    """判断 map→odom 是否"最近 settle_sec 秒内稳定"。

    samples: 按时间升序的 [(wall_t, x, y, yaw), ...]。
    判定：取最后一个采样往前、跨度 ≥ settle_sec 的窗口（≈ 最近 settle_sec 秒），
    窗口内 x/y 的峰峰值 ≤ max_dxy、yaw 的峰峰值 ≤ max_dyaw ⇒ 稳定。
    返回 (ok, drift_xy, drift_yaw, span)：窗口还不够长时 ok=False，但仍报"到目前为止"的漂移
    （供 JSON 记录 / 打印告警），drift 可能为 None（采样不足 2 个）。
    """
    if len(samples) < 2:
        return False, None, None, 0.0
    t_end = samples[-1][0]
    idx = next((k for k, s in enumerate(samples) if t_end - s[0] >= settle_sec), None)
    if idx is None:
        w, span = samples, t_end - samples[0][0]
    else:
        w, span = samples[idx:], t_end - samples[idx][0]
    dxy = max(max(s[1] for s in w) - min(s[1] for s in w),
              max(s[2] for s in w) - min(s[2] for s in w))
    y0 = w[0][3]
    dyaw = max(wrap_angle(s[3] - y0) for s in w) - min(wrap_angle(s[3] - y0) for s in w)
    return (idx is not None and dxy <= max_dxy and dyaw <= max_dyaw), dxy, dyaw, span


def parse_yaw(s):
    """--yaw 的取值：auto（默认）或弧度值。"""
    if s.strip().lower() == "auto":
        return "auto"
    try:
        return float(s)
    except ValueError:
        raise argparse.ArgumentTypeError("--yaw 只能是 auto 或弧度值（例：--yaw 1.57），得到 %r" % s)


def grid_from_msg(m):
    """nav_msgs/OccupancyGrid → goal_cell_state() 要的 grid 元组。"""
    i = m.info
    return (i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y, m.data)


class Regress(Node):
    def __init__(self, a):
        super().__init__("nav_smoke_regression")
        self.args = a
        self.spin_speed = None
        self.clock = None
        self.clock_wall = None
        self.clock0 = None
        self.wall0 = time.time()
        self.stamp, self.cnt = {}, {}
        self.tf, self.tf_seen = {}, set()          # key -> 最新戳（tf_age 判据沿用）
        self.tf_full = {}                          # key -> (stamp, (x,y,z), (qx,qy,qz,qw))：平面复合成 map 位姿
        self.map_msg = None                        # 先验地图（/map，latched）
        self.fp_stamps = []
        self.cmd = {"nav": (0.0, 0.0), "smooth": (0.0, 0.0), "chassis": (0.0, 0.0)}
        self.gt_twist_max = 0.0
        self.gt_pose0 = None
        self.gt_pose_delta = 0.0
        self.d_min = None
        self.recoveries = 0

        def cb(name, qos, stamp_attr=None):
            def f(m):
                s = getattr(m, stamp_attr) if stamp_attr else m.header.stamp
                v = s.sec + s.nanosec * 1e-9
                self.stamp[name] = v
                self.cnt[name] = self.cnt.get(name, 0) + 1
            return f

        self.create_subscription(Clock, "/clock", self.on_clock, BEST)
        self.create_subscription(PointCloud2, "/livox/lidar/pointcloud", cb("pcloud", BEST), BEST)
        self.create_subscription(Imu, "/livox/imu", cb("imu", BEST), BEST)
        self.create_subscription(LaserScan, "/scan", cb("scan", BEST), BEST)
        self.create_subscription(Odometry, "/odom", cb("odom", RELI), RELI)
        self.create_subscription(PolygonStamped, "/local_costmap/published_footprint",
                                 self.on_fp, RELI)
        self.create_subscription(Odometry, "/odom_ground_truth", self.on_gt, RELI)
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELI)
        self.create_subscription(OccupancyGrid, a.map_topic, self.on_map, MAP_QOS)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, RELI)
        self.create_subscription(Twist, "/cmd_vel", self.on_smooth, RELI)
        self.create_subscription(Twist, "/cmd_vel_chassis", self.on_chassis, RELI)
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---- 回调 ----
    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock_wall = time.time()
        if self.clock0 is None:
            self.clock0, self.wall0 = self.clock, time.time()

    def on_fp(self, m):
        self.stamp["footprint"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.cnt["footprint"] = self.cnt.get("footprint", 0) + 1
        self.fp_stamps.append(self.stamp["footprint"])

    def on_gt(self, m):
        p = m.pose.pose.position
        if self.gt_pose0 is None:
            self.gt_pose0 = (p.x, p.y)
        else:
            self.gt_pose_delta = max(self.gt_pose_delta,
                                     abs(p.x - self.gt_pose0[0]) + abs(p.y - self.gt_pose0[1]))
        t = m.twist.twist
        self.gt_twist_max = max(self.gt_twist_max, abs(t.linear.x), abs(t.linear.y), abs(t.angular.z))

    def on_tf(self, m):
        for t in m.transforms:
            k = f"{t.header.frame_id.lstrip('/')}->{t.child_frame_id.lstrip('/')}"
            if k in ("odom->base_link", "map->odom", "base_link->base_link_fake"):
                s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self.tf[k] = s
                # ★ 存**整条变换**（平移 + 四元数）而不只是时间戳：yaw=auto 要用它做平面复合
                tr, q = t.transform.translation, t.transform.rotation
                self.tf_full[k] = (s, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
                self.tf_seen.add(k)

    def on_map(self, m):
        self.map_msg = m

    def on_nav(self, m):
        self.cmd["nav"] = (max(self.cmd["nav"][0], abs(m.linear.x)), max(self.cmd["nav"][1], abs(m.angular.z)))

    def on_smooth(self, m):
        self.cmd["smooth"] = (max(self.cmd["smooth"][0], abs(m.linear.x)), max(self.cmd["smooth"][1], abs(m.angular.z)))

    def on_chassis(self, m):
        self.cmd["chassis"] = (max(self.cmd["chassis"][0], abs(m.linear.x)), max(self.cmd["chassis"][1], abs(m.angular.z)))

    # ---- 工具 ----
    def drain(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def wait_settle(self, settle_sec, timeout, max_dxy, max_dyaw, poll=0.1):
        """等 map→odom 稳定（~10 Hz 轮询 + 每秒打印进度）。

        为什么要等（人类语义）：人在 RViz 里发目标前会先看定位是不是稳了 —— 粒子云没收敛、
        ICP 还在配准、或 SLAM 刚回环跳变时点目标，nav2 会立刻 ABORT / 规划到错的地方，
        看起来像"导航坏了"，其实是**发目标时机**不对。
        返回 (settled, waited, drift_xy, drift_yaw, n_samples)。
        """
        samples, t0, t_print = [], time.time(), 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=poll)
            T = self.tf_full.get("map->odom")
            if T is not None:
                x, y, yaw = planar_of(T)
                samples.append((time.time(), x, y, yaw))
            ok, dxy, dyaw, span = check_settled(samples, settle_sec, max_dxy, max_dyaw)
            el = time.time() - t0
            if el - t_print >= 1.0:
                t_print = el
                print("     [settle %4.1fs] map→odom 漂移 xy=%s yaw=%s（窗口 %s / 需 %.1fs）"
                      % (el, "--" if dxy is None else "%.4fm" % dxy,
                         "--" if dyaw is None else "%.4frad" % dyaw, "%.1fs" % span, settle_sec),
                      flush=True)
            if ok:
                return True, el, dxy, dyaw, len(samples)
            if el >= timeout:
                return False, el, dxy, dyaw, len(samples)

    def hz(self, name, secs=3.0):
        c0 = self.cnt.get(name, 0)
        self.drain(secs)
        return (self.cnt.get(name, 0) - c0) / secs

    def rtf(self):
        if self.clock is None or self.clock is None:
            return None
        if self.clock0 is None:
            return None
        dt_wall = max(1e-6, time.time() - self.wall0)
        return (self.clock - self.clock0) / dt_wall


def param(node_name, name):
    try:
        out = subprocess.run(["ros2", "param", "get", node_name, name],
                             capture_output=True, text=True, timeout=8).stdout
        return float(out.strip().split()[-1])
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, default=[-1.0, 2.0])
    ap.add_argument("--ready-timeout", type=float, default=240.0)
    ap.add_argument("--goal-timeout", type=float, default=180.0)
    ap.add_argument("--skip-goal", action="store_true")
    ap.add_argument("--yaw", type=parse_yaw, default="auto", metavar="{auto|<rad>}",
                    help="目标朝向：auto（默认）= 车当前朝向（map→odom∘odom→base_link∘"
                         "base_link→base_link_fake 平面复合，与人在 RViz 里拖箭头一致）；"
                         "也可给弧度值做 A/B（--yaw 0 = 旧行为『到点朝地图东』）")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="发目标前要求 map→odom 已稳定多久（秒，默认 3.0；0 = 不等，立即发）")
    ap.add_argument("--settle-timeout", type=float, default=30.0,
                    help="等稳定的上限（秒，默认 30.0）；超时只告警并继续，不跳过发目标")
    ap.add_argument("--no-precheck", action="store_true",
                    help="跳过『目标在图上是 free 吗』预检（默认会读 /map 判 free/occupied/unknown/图外）")
    ap.add_argument("--map-topic", default="/map",
                    help="先验地图话题（默认 /map；map_server 是 latched 的，用 RELIABLE+TRANSIENT_LOCAL 订阅）")
    ap.add_argument("--map-timeout", type=float, default=20.0,
                    help="等先验地图的秒数（默认 20.0）；等不到只告警并跳过预检（仍然会发目标）")
    ap.add_argument("--outdir", default=".tmp_bags")
    a = ap.parse_args()

    rclpy.init()
    n = Regress(a)
    fails, notes = [], []

    def hp():
        print("  [%5.1fs] clock=%s | pcloud=%s | imu=%s | scan=%s | odom=%s | fp=%s | TF=%s"
              % (time.time() - n.wall0,
                 "--" if n.clock is None else f"{n.clock:.2f}",
                 n.cnt.get("pcloud", 0), n.cnt.get("imu", 0), n.cnt.get("scan", 0),
                 n.cnt.get("odom", 0), n.cnt.get("footprint", 0),
                 ",".join(sorted(n.tf_seen)) or "--"), flush=True)

    # ---------------- 阶段 0：链路就绪 ----------------
    print("[0] 等链路就绪（最多 %.0fs）… 请确认栈已启动" % a.ready_timeout, flush=True)
    t0 = time.time()
    while time.time() - t0 < a.ready_timeout:
        n.drain(2.0)
        hp()
        if (n.cnt.get("pcloud", 0) and n.cnt.get("imu", 0) and n.cnt.get("scan", 0)
                and n.cnt.get("odom", 0) and n.cnt.get("footprint", 0)
                and {"odom->base_link", "map->odom"} <= n.tf_seen):
            break
    else:
        missing = [k for k in ("pcloud", "imu", "scan", "odom", "footprint") if not n.cnt.get(k)]
        if not {"odom->base_link", "map->odom"} <= n.tf_seen:
            missing += ["TF:" + x for x in ("odom->base_link", "map->odom") if x not in n.tf_seen]
        fails.append("链路未就绪，缺: " + ", ".join(missing))

    rtf = n.rtf()
    hz = {k: n.hz(k) for k in ("pcloud", "imu", "scan", "odom")}
    fp_distinct = len(set(n.fp_stamps))
    if rtf is not None and rtf < TH["clock_rtf_min"]:
        fails.append(f"/clock RTF={rtf:.2f} < {TH['clock_rtf_min']}（仿真没在正常推进）")
    for k, mn in (("pcloud", TH["hz_pcloud_min"]), ("imu", TH["hz_imu_min"]),
                  ("scan", TH["hz_scan_min"]), ("odom", TH["hz_odom_min"])):
        if hz[k] < mn:
            fails.append(f"{k} 频率 {hz[k]:.1f}Hz < {mn}Hz")
    if fp_distinct < TH["footprint_min_distinct"]:
        fails.append(f"costmap footprint 戳只变过 {fp_distinct} 次（位姿未持续更新）")
    age = None if not n.tf else max((n.clock - v) for v in n.tf.values())
    if age is not None and age > TH["tf_age_max"]:
        fails.append(f"TF 最新戳落后 /clock {age:.2f}s > {TH['tf_age_max']}s")
    print(f"[0] RTF={rtf if rtf is None else round(rtf,2)} hz={ {k: round(v,1) for k,v in hz.items()} } "
          f"fp_distinct={fp_distinct} tf_age={None if age is None else round(age,2)}", flush=True)

    # ---------------- 阶段 1：发目标（先"像人一样"选时机 + 验目标，再发） ----------------
    goal_yaw, yaw_source, yaw_why, abort = None, None, None, None
    settle = {"enabled": False, "settled": None, "waited": 0.0, "drift_xy": None,
              "drift_yaw": None, "samples": 0, "settle_sec": a.settle,
              "timeout_sec": a.settle_timeout}
    map_check = {"enabled": not a.no_precheck, "checked": False, "topic": a.map_topic,
                 "state": None, "value": None, "cell": None, "map": None}

    if not a.skip_goal and not fails:
        # ---- 1a 等定位稳定（人类语义：等 RViz 里 map→odom 不飘了再点目标；见 wait_settle 注释）----
        if a.settle > 0:
            print(f"[1a] 等 map→odom 稳定 {a.settle:.1f}s（漂移阈值 ≤{TH['settle_dxy']}m / "
                  f"≤{TH['settle_dyaw']}rad，最多等 {a.settle_timeout:.0f}s）…", flush=True)
            settled, waited, dxy, dyaw, n_s = n.wait_settle(
                a.settle, a.settle_timeout, TH["settle_dxy"], TH["settle_dyaw"])
            settle.update(enabled=True, settled=settled, waited=waited,
                          drift_xy=dxy, drift_yaw=dyaw, samples=n_s)
            if settled:
                print("[1a] ✅ 定位已稳定：等待 %.1fs，窗口内漂移 xy=%.4fm yaw=%.4frad"
                      % (waited, dxy, dyaw), flush=True)
            else:
                print("[1a] ⚠️ WARNING：%.1fs 内 map→odom 没稳定（漂移 xy=%s yaw=%s > 阈值 "
                      "%sm/%srad）⇒ **仍然继续发目标**，但结果若 ABORT/走歪，先怀疑定位瞬态，"
                      "而不是导航链本身（可加大 --settle-timeout 或先看 RViz 粒子云/ICP）"
                      % (waited, "--" if dxy is None else "%.4fm" % dxy,
                         "--" if dyaw is None else "%.4frad" % dyaw,
                         TH["settle_dxy"], TH["settle_dyaw"]), flush=True)
                notes.append("定位在 %.0fs 内未稳定（漂移 xy=%s yaw=%s）：本次结果需按"
                             "『定位瞬态』打折解读" % (waited,
                                                   "--" if dxy is None else round(dxy, 4),
                                                   "--" if dyaw is None else round(dyaw, 4)))
        else:
            print("[1a] --settle 0 ⇒ 不等稳定，立刻发（保持旧行为）", flush=True)
            settle["note"] = "--settle 0 ⇒ 跳过稳定性等待"

        # ---- 1b 目标朝向（默认 auto = 车当前朝向，与人在 RViz 里拖箭头一致）----
        goal_yaw, yaw_source, yaw_why = choose_goal_yaw(n.tf_full, a.goal, a.yaw)
        print("[1b] 目标朝向 yaw=%.3f rad (%.1f°)  yaw_source=%s —— %s"
              % (goal_yaw, math.degrees(goal_yaw), yaw_source, yaw_why), flush=True)
        notes.append("目标朝向 yaw=%.3f rad (%.1f°) source=%s" % (goal_yaw, math.degrees(goal_yaw), yaw_source))
        if yaw_source in ("to_goal", "fallback"):
            print("[1b] ⚠️ 注意：TF 位姿还算不出来，朝向不是『车当前朝向』（见上）", flush=True)

        # ---- 1c 目标可用性预检（先验地图上这一格是 free 吗）----
        if a.no_precheck:
            print("[1c] --no-precheck ⇒ 跳过目标可用性预检", flush=True)
            map_check["state"] = "skipped"
        else:
            print(f"[1c] 读先验地图 {a.map_topic}（RELIABLE+TRANSIENT_LOCAL，等 {a.map_timeout:.0f}s）"
                  f"判目标 ({a.goal[0]:.2f}, {a.goal[1]:.2f}) 是否 free …", flush=True)
            t_map = time.time()
            while n.map_msg is None and time.time() - t_map < a.map_timeout:
                rclpy.spin_once(n, timeout_sec=0.1)
            if n.map_msg is None:
                map_check.update(state="no_map", note=f"{a.map_timeout:.0f}s 内没收到 {a.map_topic}")
                print(f"[1c] ⚠️ WARNING：{a.map_timeout:.0f}s 内没收到 {a.map_topic} ⇒ 预检跳过（仍发目标）。"
                      "若地图话题不同名用 --map-topic，发布端非 latched(VOLATILE) 时本订阅收不到；"
                      "不想要这一步用 --no-precheck", flush=True)
                notes.append("未收到先验地图 %s：跳过目标可用性预检" % a.map_topic)
            else:
                grid = grid_from_msg(n.map_msg)
                state, val, cell = goal_cell_state(grid, a.goal[0], a.goal[1])
                map_check.update(checked=True, state=state, value=val, cell=[cell[0], cell[1]],
                                 map={"resolution": grid[0], "width": grid[1], "height": grid[2],
                                      "origin": [grid[3], grid[4]]})
                print("[1c] 地图 %.3f m/格 %dx%d origin=(%.2f, %.2f)；目标落格 %s = %s (value=%s)"
                      % (grid[0], grid[1], grid[2], grid[3], grid[4], cell, state, val), flush=True)
                if state != "free":
                    abort = (f"目标 ({a.goal[0]:.2f}, {a.goal[1]:.2f}) 在图中是 {state}"
                             f"（落格 {cell}，value={val}）⇒ 请换一个 free 点；"
                             "**这不算导航失败**（未发目标）")
                    print("[1c] ❌ " + abort, flush=True)
                    print("     判读：occupied = 撞墙/在膨胀带里；unknown = 雷达还没扫到（先让车走一段或换点）；"
                          "out_of_map = 超出地图范围（换点，或确认 map 系/地图资产是否对）", flush=True)
                    print("     选点工具：/usr/bin/python3 tools/check_map_reachable.py --map "
                          "src/rm_nav_bringup/map/<world>.yaml --start <起点> --goal "
                          "%.2f %.2f（见 docs/smoke_test_runbook.md §0.6）"
                          % (a.goal[0], a.goal[1]), flush=True)
                    print("     坚持要发这个点：加 --no-precheck", flush=True)
                else:
                    print("[1c] ✅ 目标是已知自由栅格，可以发", flush=True)

    if not a.skip_goal and not fails and abort is None:
        print(f"[1d] 发目标 ({a.goal[0]:.2f}, {a.goal[1]:.2f}) in map，yaw={goal_yaw:.3f} rad "
              f"(source={yaw_source}) …", flush=True)
        if not n.ac.wait_for_server(timeout_sec=20.0):
            fails.append("/navigate_to_pose 动作服务不可用")
        else:
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"      # stamp 留 0 = 取最新（避免墙钟/仿真钟差异）
            g.pose.pose.position.x, g.pose.pose.position.y = a.goal
            # 纯 yaw 四元数（平面）：与 RViz 里拖出来的目标位姿同一语义
            g.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
            g.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
            t_send = time.time()

            def fb(m):
                n.d_min = m.feedback.distance_remaining if n.d_min is None else min(n.d_min, m.feedback.distance_remaining)
                n.recoveries = max(n.recoveries, m.feedback.number_of_recoveries)

            send = n.ac.send_goal_async(g, feedback_callback=fb)
            n.spin_until(lambda: send.done(), 15.0)
            gh = send.result() if send.done() else None
            if gh is None or not gh.accepted:
                fails.append("目标被拒绝（accepted=False）")
            else:
                res = gh.get_result_async()
                n.spin_until(lambda: res.done(), a.goal_timeout)
                n.drain(1.0)
                dt = time.time() - t_send
                if not res.done():
                    fails.append(f"目标超时 {a.goal_timeout:.0f}s（残余距离 "
                                 f"{'?' if n.d_min is None else round(n.d_min,2)}m）")
                else:
                    st = res.result().status
                    names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
                    notes.append(f"结果 status={names.get(st, st)} 用时={dt:.1f}s "
                                 f"d_min={None if n.d_min is None else round(n.d_min,3)}m "
                                 f"recoveries={n.recoveries}")
                    if st == 4 and dt < TH["fake_success_secs"] and (n.d_min or 0) > TH["fake_success_dist"]:
                        fails.append(f"**假到达**：{dt:.1f}s 就 SUCCEEDED 而残余 {n.d_min:.2f}m"
                                     f"（上游 nav2 丢弃 transformPose 失败的典型特征）")
                    elif st == 4 and (n.d_min is None or n.d_min > TH["arrive_tol"]):
                        fails.append(f"报 SUCCEEDED 但残余距离 {n.d_min}m > 容差 {TH['arrive_tol']}m")
                    elif st != 4:
                        fails.append(f"导航未成功：status={names.get(st, st)}"
                                     f"（若为 ABORTED 且 recoveries>0，多为 progress_checker 判 Failed to make progress）")

            # ---- 阶段 2：四跳命令链 + 真值 ----
            if n.spin_speed is None:
                n.spin_speed = param("/fake_vel_transform", "spin_speed")
            nav_l, nav_a = n.cmd["nav"]
            ch_l, ch_a = n.cmd["chassis"]
            notes.append(f"命令链 max|v|,|w|: nav=({nav_l:.2f},{nav_a:.2f}) "
                         f"smooth=({n.cmd['smooth'][0]:.2f},{n.cmd['smooth'][1]:.2f}) "
                         f"chassis=({ch_l:.2f},{ch_a:.2f}) spin_speed={n.spin_speed}")
            notes.append(f"真值: twist_max={n.gt_twist_max:.3f} pose_delta={n.gt_pose_delta:.3f}m")
            if nav_a > TH["mv_eps"] or nav_l > TH["mv_eps"]:
                if ch_l < TH["mv_eps"] and ch_a < TH["mv_eps"]:
                    fails.append("**命令链断**：nav 有输出但 /cmd_vel_chassis 恒 0"
                                 "（fake_vel_transform 或更下游）")
                elif n.spin_speed == 0.0 and nav_a > TH["mv_eps"] and ch_a < TH["mv_eps"]:
                    fails.append("命令链断：spin_speed=0 时 /cmd_vel_chassis 的角速度应等于 nav 的")
                if n.gt_twist_max < TH["mv_eps"] and n.gt_pose_delta < 0.02:
                    fails.append("真值没动：指令下去了但车没动（底盘/物理侧）")

    # ---------------- 汇总 ----------------
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    ok = (not fails) and abort is None
    if abort is not None:
        result, code = "goal_rejected", EXIT_GOAL_UNUSABLE
    elif a.skip_goal:
        result, code = "chain_only", (EXIT_PASS if ok else EXIT_FAIL)
    else:
        result, code = ("pass" if ok else "fail"), (EXIT_PASS if ok else EXIT_FAIL)
    os.makedirs(a.outdir, exist_ok=True)
    out = os.path.join(a.outdir, f"regress_{int(time.time())}.json")
    snap = {"pass": ok, "fails": fails, "notes": notes, "rtf": rtf, "hz": hz,
            "fp_distinct": fp_distinct, "tf_age": age, "cmd": n.cmd,
            "spin_speed": n.spin_speed, "d_min": n.d_min,
            "gt_twist_max": n.gt_twist_max, "gt_pose_delta": n.gt_pose_delta,
            "goal": a.goal, "skip_goal": a.skip_goal,
            # ↓ 本次新增（旧字段全部保持原名/原义，方便老的对比脚本继续读）
            "result": result, "exit_code": code, "nav_failure": bool(fails), "abort": abort,
            "goal_yaw": goal_yaw,
            "goal_yaw_deg": None if goal_yaw is None else round(math.degrees(goal_yaw), 2),
            "yaw_arg": a.yaw, "yaw_source": yaw_source, "yaw_note": yaw_why,
            "settle": settle, "map_check": map_check}
    with open(out, "w") as f:
        json.dump(snap, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 72)
    if abort is not None:
        print("P0 回归：⛔ 目标不可用（未发目标，不算导航失败；exit=%d）" % code)
    else:
        print("P0 回归：" + ("✅ PASS" if ok else "❌ FAIL"))
    for x in notes:
        print("  · " + x)
    for x in fails:
        print("  ✗ " + x)
    if abort is not None:
        print("  ⛔ " + abort)
    if goal_yaw is not None:
        print("  目标朝向：yaw=%.3f rad (%.1f°) source=%s" % (goal_yaw, math.degrees(goal_yaw), yaw_source))
    if settle["enabled"]:
        print("  稳定性：settled=%s 等待=%.1fs 漂移 xy=%s yaw=%s"
              % (settle["settled"], settle["waited"],
                 "--" if settle["drift_xy"] is None else round(settle["drift_xy"], 4),
                 "--" if settle["drift_yaw"] is None else round(settle["drift_yaw"], 4)))
    if map_check["enabled"] and map_check["state"] is not None:
        print("  目标可用性：%s（value=%s，落格 %s）"
              % (map_check["state"], map_check["value"], map_check["cell"]))
    print(f"  首个断点：{fails[0] if fails else ('无（但目标未发）' if abort else '无')}")
    print(f"  快照 -> {out}（可抄进 docs/algorithm_matrix.md §四）")
    print("=" * 72)
    return code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[中断] 未完成，结论无效")
