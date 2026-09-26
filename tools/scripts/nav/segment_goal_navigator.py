#!/usr/bin/env python3
"""segment_goal_navigator.py —— 路线 B：**给一个远目标，自动拆成若干短段**逐段导航。

设计要点（v2，**完全不依赖 TF**，避免 /tf_static latched、时钟、QoS 那一整类坑）：
  · 当前位姿不自己查 TF，而是**让 planner 给**：/compute_path_to_pose 用 use_start=False（它自己取起点），
    返回路径的第一个点就是"车当前位姿"（map 系）。
  · 沿路径找**最后一个"已知自由"栅格**（/map：0=free，>0=占据，-1=未知），且本段长度 ≤ --max-seg
    ⇒ 得到本段目标（保证每段都落在已知自由空间里）。
  · 用 /navigate_to_pose 走这一段；到了再算下一段（地图长大了段会自然变长）。
  · 整条参考路径都 free ⇒ 一步直达；前进不足 --min-seg ⇒ 明确报"不可达/被堵"，不把未知当自由硬穿。

用法（先 --dry-run 只看分段、不动车）：
  python3 tools/scripts/nav/segment_goal_navigator.py --goal 3.0 -4.0 --dry-run
  python3 tools/scripts/nav/segment_goal_navigator.py --goal 3.0 -4.0 --max-seg 4.0 --min-seg 1.0
"""
import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid

MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


class SegNav(Node):
    def __init__(self, a):
        super().__init__("segment_goal_navigator")
        self.a = a
        self.map = None
        self.costmap = None
        self.create_subscription(OccupancyGrid, a.map_topic, self.on_map, MAP_QOS)
        # ★ 判定"能不能走"应当用**planner 真正使用的那张图**（global costmap），而不是 /map：
        #   /map 是在线建图的产物，可能与 costmap 的 static 层不一致（旧快照/不同阈值）
        self.create_subscription(Costmap, a.costmap_topic, self.on_costmap, MAP_QOS)
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def on_map(self, m):
        self.map = m

    def on_costmap(self, m):
        self.costmap = m

    def wait(self, fut, timeout):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.done()

    def cell(self, x, y):
        """优先用 global costmap 判定：0=free、1..252=膨胀(可走)、>=253=致命/内切、255=未知；
        没有 costmap 时退回 /map（0=free、>0=occ、-1=unk）。"""
        cm = self.costmap
        if cm is not None:
            md = cm.metadata
            cx = int((x - md.origin.position.x) / md.resolution)
            cy = int((y - md.origin.position.y) / md.resolution)
            if not (0 <= cx < md.size_x and 0 <= cy < md.size_y):
                return "out"
            v = int(cm.data[cy * md.size_x + cx])
            if v == 255:
                return "unk"
            return "occ" if v >= 253 else "free"
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

    def compute_path(self, goal, start=None):
        if not self.planner.wait_for_server(timeout_sec=10.0):
            print("[seg] /compute_path_to_pose 不可用（nav2 没起？）", flush=True); return None, None, None
        g = ComputePathToPose.Goal()
        g.goal, g.planner_id = goal, "GridBased"
        g.use_start = start is not None
        g.start = start if start is not None else goal     # use_start=False 时被忽略
        fut = self.planner.send_goal_async(g)
        if not self.wait(fut, 20.0):
            print("[seg] 规划请求超时", flush=True); return None, None, None
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("[seg] 规划被拒绝", flush=True); return None, None, None
        rf = gh.get_result_async()
        if not self.wait(rf, self.a.seg_timeout):
            print("[seg] 规划无结果（目标不可达，或必须穿过未知/障碍区）", flush=True)
            return None, None, None
        res = rf.result()
        return res.result.path, res.status, getattr(res.result, "error_code", None)

    def pick_segment(self, path):
        poses = path.poses
        if not poses:
            return None, 0.0, False, "empty"
        total, cut, blocked = 0.0, None, None
        prev = poses[0].pose.position
        for i, ps in enumerate(poses):
            p = ps.pose.position
            if i > 0:
                total += math.hypot(p.x - prev.x, p.y - prev.y)
                prev = p
            st = self.cell(p.x, p.y)
            if st != "free":
                blocked = (i, st); break
            if total >= self.a.max_seg:
                cut = i; break
            cut = i
        if blocked is None:
            return poses[-1], total, True, "all-free"
        if cut is None:
            return None, 0.0, False, "start-%s" % blocked[1]
        return poses[cut], total, False, "cut@%s" % blocked[1]

    def go(self, pose, timeout):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            print("[seg] /navigate_to_pose 不可用", flush=True); return False
        g = NavigateToPose.Goal()
        g.pose = pose
        g.pose.header.stamp.sec = 0            # 0 = 取最新
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
        return rf.result().status == 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True)
    ap.add_argument("--max-seg", type=float, default=4.0)
    ap.add_argument("--min-seg", type=float, default=1.0)
    ap.add_argument("--arrive-tol", type=float, default=0.4)
    ap.add_argument("--seg-timeout", type=float, default=90.0)
    ap.add_argument("--max-segs", type=int, default=12)
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--costmap-topic", default="/global_costmap/costmap_raw",
                    help="用来判定可通行性的图（默认 global costmap；它就是 planner 用的那张）")
    ap.add_argument("--dry-run", action="store_true", help="只算并打印分段，不发目标、不动车")
    a = ap.parse_args()

    rclpy.init()
    n = SegNav(a)
    print("[seg] 等 %s …" % a.map_topic, flush=True)
    end = time.time() + 30
    while n.map is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.map is None:
        print("[seg] ✗ 没收到 %s（地图未起，或话题名不同：--map-topic）" % a.map_topic, flush=True)
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass
        return 2

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y = a.goal
    goal.pose.orientation.w = 1.0
    src = "global costmap" if n.costmap is not None else "/map（警告：未收到 costmap，判定可能不准）"
    print("[seg] 可通行性判定来源：%s" % src, flush=True)
    print("[seg] 地图 %.3f m/格 %dx%d  目标 (%.2f, %.2f)  max-seg=%.1f %s"
          % (n.map.info.resolution, n.map.info.width, n.map.info.height,
             a.goal[0], a.goal[1], a.max_seg, "(dry-run)" if a.dry_run else ""), flush=True)

    virtual = None            # dry-run 用：虚拟的"当前位置"（= 上一段终点）
    seg_i, ok = 0, False
    while seg_i < a.max_segs:
        path, st, ec = n.compute_path(goal, start=virtual)
        if path is None:
            print("[seg] ✗ 规划请求失败（见上一行）", flush=True); break
        if not path.poses:
            m = n.map
            res_, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
            print("[seg] ✗ planner 返回**空路径**：status=%s error_code=%s" % (st, ec), flush=True)
            print("     地图范围 x[%.2f, %.2f] y[%.2f, %.2f]；目标(%.2f,%.2f) 落格=%s"
                  % (ox, ox + m.info.width * res_, oy, oy + m.info.height * res_,
                     a.goal[0], a.goal[1], n.cell(a.goal[0], a.goal[1])), flush=True)
            print("     判读：若目标是 out/unk/occ ⇒ 目标在地图外/未知/障碍里；"
                  "若目标 free 却仍空路径 ⇒ 多半是**车当前位姿不在 costmap 覆盖范围内**（地图没覆盖到车）",
                  flush=True)
            break
        cur = path.poses[0].pose            # planner 给的"当前位姿"，不自己查 TF
        d_goal = math.hypot(goal.pose.position.x - cur.position.x,
                            goal.pose.position.y - cur.position.y)
        seg, length, all_free, why = n.pick_segment(path)
        if seg is None:
            print("[seg] ✗ 无法前进（%s；车距目标 %.2f m）—— 明确报不可达，不硬穿未知区" % (why, d_goal), flush=True)
            break
        seg_i += 1
        print("[seg] 第 %d 段 → (%.2f, %.2f)  段长 %.2f m  离最终目标 %.2f m  [%s]"
              % (seg_i, seg.pose.position.x, seg.pose.position.y, length, d_goal, why), flush=True)
        if all_free or d_goal <= a.arrive_tol:
            print("[seg] 参考路径全在已知自由空间 ⇒ 一段直达终点", flush=True)
        if a.dry_run:
            if all_free or d_goal <= a.arrive_tol:
                break
            if length < a.min_seg:
                print("[seg] ✗ dry-run 也推进不了（单段仅 %.2f m < min-seg）；"
                      "多半是车前方立刻被占/未知——看上面的 [cut@…] 原因" % length, flush=True)
                break
            virtual = seg                    # 虚拟前进，继续算下一段
            continue
        if not n.go(seg, a.seg_timeout):
            print("[seg] ✗ 第 %d 段失败" % seg_i, flush=True); break
        if all_free or d_goal <= a.arrive_tol:
            ok = True; break
        if length < a.min_seg:
            print("[seg] ✗ 单段进展 %.2f m < min-seg，停止（避免死循环）" % length, flush=True); break
    print("[seg] %s：共 %d 段"
          % ("✅ 完成" if ok else ("(dry-run)" if a.dry_run else "❌ 未完成"), seg_i), flush=True)
    n.destroy_node()
    try: rclpy.shutdown()
    except Exception: pass
    return 0 if (ok or a.dry_run) else 1


if __name__ == "__main__":
    raise SystemExit(main())
