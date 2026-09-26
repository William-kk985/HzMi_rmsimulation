#!/usr/bin/env python3
"""segment_goal_navigator.py —— 路线 B：**给一个远目标，自动拆成若干短段**逐段导航。

思路（分层调度，不改 nav2 源码）：
  1) 任务层用 planner_server 的 /compute_path_to_pose 算一条**参考路径**（允许 unknown，只作参考）；
  2) 沿参考路径从起点往前走，找**最后一个"已知自由"栅格**（在 /map 上查：0=free；>0=占据；-1=未知），
     并把本段长度**限制在 --max-seg 之内** ⇒ 得到本段目标（保证：每段都落在已知自由空间里）；
  3) 用 /navigate_to_pose 走这一段；到了再从 ① 重新算（所以地图长大了、段也会跟着变长）；
  4) 若整条参考路径都落在已知自由空间 ⇒ 直接一步到位（正常导航，不折腾）；
  5) 若"前进不了 min-seg" ⇒ 明确报**不可达/被堵**（而不是把未知当自由硬穿）。

用法（⚠️ 先 --dry-run 看它怎么切，确认合理再实跑）：
  python3 tools/scripts/nav/segment_goal_navigator.py --goal 3.0 -4.0 --dry-run
  python3 tools/scripts/nav/segment_goal_navigator.py --goal 3.0 -4.0 --max-seg 4.0 --min-seg 1.0

参数：--max-seg 单段最长(m) | --min-seg 能前进的最小步长(m) | --arrive-tol 终点容差(m)
      --seg-timeout 单段超时(s) | --max-segs 最多几段 | --map-topic | --dry-run
"""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


class SegNav(Node):
    def __init__(self, a):
        super().__init__("segment_goal_navigator")
        self.a = a
        self.map = None
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.create_subscription(OccupancyGrid, a.map_topic, self.on_map, MAP_QOS)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def on_map(self, m):
        self.map = m

    def drain(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, fut, timeout):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done()

    # --- 位姿 / 地图 ---
    def robot_pose(self):
        """取 map→base 的位姿；base_frame 不可用时退用 base_link（并告警）。失败时记录原因。"""
        self.tf_err = None
        for f in (self.a.base_frame, "base_link"):
            try:
                t = self.buf.lookup_transform("map", f, Time())
            except Exception as e:                    # noqa: BLE001
                self.tf_err = "%s: %s" % (f, e)
                continue
            if f != self.a.base_frame:
                print("[seg] ⚠️ %s 查不到，退用 %s" % (self.a.base_frame, f), flush=True)
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.orientation = t.transform.rotation
            return p
        return None

    def cell(self, x, y):
        """返回 'free' / 'occ' / 'unk' / 'out'（-1=未知，0=自由，1..100=占据）"""
        m = self.map
        if m is None:
            return "unk"
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        cx, cy = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= cx < m.info.width and 0 <= cy < m.info.height):
            return "out"
        v = m.data[cy * m.info.width + cx]
        if v < 0:
            return "unk"
        return "free" if v == 0 else "occ"

    # --- 规划 ---
    def compute_path(self, start, goal):
        if not self.planner.wait_for_server(timeout_sec=10.0):
            print("[seg] /compute_path_to_pose 不可用", flush=True); return None
        g = ComputePathToPose.Goal()
        g.start, g.goal, g.use_start, g.planner_id = start, goal, True, "GridBased"
        fut = self.planner.send_goal_async(g)
        if not self.wait(fut, 20.0):
            print("[seg] 规划请求超时", flush=True); return None
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("[seg] 规划被拒绝", flush=True); return None
        rf = gh.get_result_async()
        if not self.wait(rf, self.a.seg_timeout):
            print("[seg] 规划无结果（可能必须穿过未知区/不可达）", flush=True); return None
        return rf.result().result.path

    def pick_segment(self, path):
        """沿路径找本段终点：走到第一个非 free 之前为止，且长度不超过 max-seg。
        返回 (pose, 段长, 是否已可直达终点, 截断原因)"""
        poses = path.poses
        if not poses:
            return None, 0.0, False, "empty"
        total = 0.0
        prev = poses[0].pose.position
        cut = None
        blocked_at = None
        for i, ps in enumerate(poses):
            p = ps.pose.position
            if i > 0:
                total += math.hypot(p.x - prev.x, p.y - prev.y)
                prev = p
            st = self.cell(p.x, p.y)
            if st != "free":
                blocked_at = (i, st); break
            if total >= self.a.max_seg:
                cut = i; break
            cut = i
        if blocked_at is None:
            return poses[-1], total, True, "all-free"
        i, st = blocked_at
        if cut is None:
            return None, 0.0, False, "start-%s" % st
        return poses[cut], total, False, "cut@%s" % st

    # --- 执行 ---
    def go(self, pose, timeout):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            print("[seg] /navigate_to_pose 不可用", flush=True); return False
        g = NavigateToPose.Goal()
        g.pose = pose
        g.pose.header.stamp.sec = 0          # 0 = 取最新，避免墙钟/仿真钟差异
        g.pose.header.stamp.nanosec = 0
        fut = self.nav.send_goal_async(g)
        if not self.wait(fut, 15.0):
            return False
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("[seg] 段目标被拒绝", flush=True); return False
        rf = gh.get_result_async()
        if not self.wait(rf, timeout):
            print("[seg] 段超时，取消", flush=True)
            try: gh.cancel_goal_async()
            except Exception: pass
            return False
        st = rf.result().status
        return st == 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True)
    ap.add_argument("--max-seg", type=float, default=4.0)
    ap.add_argument("--min-seg", type=float, default=1.0)
    ap.add_argument("--arrive-tol", type=float, default=0.4)
    ap.add_argument("--seg-timeout", type=float, default=90.0)
    ap.add_argument("--max-segs", type=int, default=12)
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--base-frame", default="base_link_fake")
    ap.add_argument("--dry-run", action="store_true", help="只算并打印分段，不发目标、不动车")
    a = ap.parse_args()

    rclpy.init()
    n = SegNav(a)
    print("[seg] 等 /map …", flush=True)
    end = time.time() + 30
    while n.map is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.map is None:
        print("[seg] 没收到 %s（地图未起？）" % a.map_topic, flush=True)
        n.destroy_node(); rclpy.shutdown(); return 2

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y = a.goal
    goal.pose.orientation.w = 1.0
    print("[seg] 地图 %.3f m/格  %dx%d  目标 (%.2f, %.2f)  max-seg=%.1f  %s"
          % (n.map.info.resolution, n.map.info.width, n.map.info.height, a.goal[0], a.goal[1],
             a.max_seg, "(dry-run)" if a.dry_run else ""), flush=True)

    cur = n.robot_pose()
    if cur is None:
        print("[seg] ✗ 取不到机器人位姿：TF map → %s 查不到。" % a.base_frame, flush=True)
        print("     底层原因：%s" % getattr(n, "tf_err", "?"), flush=True)
        print("     先自查： ros2 run tf2_ros tf2_echo map %s" % a.base_frame, flush=True)
        print("             ros2 run tf2_ros tf2_echo map odom ; ros2 run tf2_ros tf2_echo odom base_link", flush=True)
        print("     常见原因：map→odom 没发布（slam_nav 下 cartographer 零戳/未发布）或 TF 分成两棵树。", flush=True)
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass
        return 3
    seg_i, ok = 0, False
    while seg_i < a.max_segs:
        cur = n.robot_pose() or cur
        path = n.compute_path(cur, goal)
        if path is None:
            print("[seg] ✗ 规划失败：目标不可达（或必须穿过未知区）", flush=True); break
        seg, length, all_free, why = n.pick_segment(path)
        d_goal = math.hypot(goal.pose.position.x - cur.pose.position.x,
                            goal.pose.position.y - cur.pose.position.y)
        if seg is None:
            print("[seg] ✗ 无法前进（原因 %s；车距目标 %.2f m）—— 明确报不可达，不硬穿未知区" % (why, d_goal), flush=True)
            break
        seg_i += 1
        print("[seg] 第 %d 段 → (%.2f, %.2f)  段长 %.2f m  离最终目标 %.2f m  [%s]"
              % (seg_i, seg.pose.position.x, seg.pose.position.y, length, d_goal, why), flush=True)
        if all_free or d_goal <= a.arrive_tol:
            print("[seg] 参考路径全在已知自由空间 ⇒ 一段直达终点", flush=True)
        if a.dry_run:
            cur = seg                       # 虚拟前进，继续算下一段
            if all_free or d_goal <= a.arrive_tol:
                print("[seg] dry-run 结束（预计 %d 段）" % seg_i); break
            continue
        if not n.go(seg, a.seg_timeout):
            print("[seg] ✗ 第 %d 段失败" % seg_i, flush=True); break
        if all_free or d_goal <= a.arrive_tol:
            ok = True; break
        if length < a.min_seg:
            print("[seg] ✗ 单段进展不足 %.2f m < min-seg，停止（避免死循环）" % length, flush=True); break
    print("[seg] %s：共 %d 段" % ("✅ 完成" if ok else ("(dry-run)" if a.dry_run else "❌ 未完成"), seg_i), flush=True)
    n.destroy_node()
    try: rclpy.shutdown()
    except Exception: pass
    return 0 if (ok or a.dry_run) else 1


if __name__ == "__main__":
    raise SystemExit(main())
