#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_resume_check —— 续建后的**一次性**一致性检查：存档跟现实对不上就立刻喊（可选直接收栈）。

为什么需要它（2026-10-06 事故）：
  用户从 `map/RMUC2026.posegraph` 续建，那份位姿图里**已经含有一段 LIO 退化**（ATE max 1.15 m）
  的位姿 ⇒ 加载进来的图与真实场地不一致 ⇒ 建出来的图/定位"飘"。场地隔离守卫（world/spawn）
  拦不住这种情况：world 和出生点都对，**是图本身歪了**。
  本节点在**续建之后**、机器人基本静止的静置窗口结束时，比一次
  「当前 `map→base_link`」与「存档记录里的 `map_start_pose`（+ 存档 spawn 作为上下文）」：
  偏差超过阈值 ⇒ 响亮、可操作的 WARNING；`strict:=True` ⇒ 直接退出（launch 会把整栈收掉）。

为什么放在这里（最简可靠的位置，三个候选都被否掉）：
  · 不能放 launch 期（OpaqueFunction）：t=0 时既没有 TF 也没有 slam_toolbox，硬等会**推迟所有节点启动**
    —— 本节点由 launch 在 t=0 一起起，自己的定时器等待，**不挡任何人的启动时序**；
  · 不能塞进 slam_toolbox：那是上游包，且"检查"与"建图"职责不同；
  · 也不做成持续 watchdog / 不发任何修正、不发布 `map→odom`（保持「map→odom 单一发布者」契约逐字不变）：
    它是**一次性启动体检**，拿到结论就退出（不留常驻进程）。

契约：只订阅 /tf（TransformListener）+ 读参数；**不发布任何话题、不发布任何 TF**。

判定口径（两条判据，缺一不可；7 个合成场景标定见 .tmp_hygiene/resume_check_test.sh）：
  刚 resume 时 slam_toolbox 会把位姿设成 `map_start_pose`（自我一致），偏差 0；真正暴露问题的是
  ① **随后扫描匹配把位姿按"（可能是歪的）图"拉走**的那部分量 = 自基线以来 `map→odom` 的修正量。
     它不要求机器人静止（期望值用 `odom→base_link` 的位移推过去）⇒ "用户一起来就开走"**不会误报**。
     ⚠️ 只用"实测 vs 存档记录起点"这一条比会把"正常开走"误报成存档坏了（实测 7.5 m 假偏差）。
  ② **实测 map→base_link 与存档记录的 `map_start_pose` 的绝对偏差**：只在"自基线以来一步没动"时
     参与判定 —— 起点被挪/存档被 doctored 时扫描匹配不产生任何修正（①≈0），只有这条抓得住。

退出码：0 = 检查通过 / 检查不了（TF 一直不来、机器人一直没静止且补偿后也在阈值内）；
        1 = **strict 模式**下发现不一致（launch 会据此收栈）；非 strict 下发现不一致也返回 0（只喊不拦）。
"""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


# ------------------------------------------------------------------ 4x4 位姿小工具
def quat_to_mat(q):
    x, y, z, w = [float(v) for v in q]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)]])


def tf_to_mat(tr):
    t = tr.transform.translation
    q = tr.transform.rotation
    m = np.eye(4)
    m[:3, :3] = quat_to_mat((q.x, q.y, q.z, q.w))
    m[:3, 3] = (float(t.x), float(t.y), float(t.z))
    return m


def pose_from_xy_yaw(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    m = np.eye(4)
    m[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    m[:3, 3] = (float(x), float(y), 0.0)
    return m


def yaw_of(m):
    return float(np.arctan2(m[1, 0], m[0, 0]))


def fmt_pose(m):
    return 'x=%.3f y=%.3f z=%.3f yaw=%.2f°' % (m[0, 3], m[1, 3], m[2, 3], np.degrees(yaw_of(m)))


def fmt_xyyaw(m):
    return '(%.3f, %.3f, %.2f°)' % (m[0, 3], m[1, 3], np.degrees(yaw_of(m)))


class MapResumeCheck(Node):
    def __init__(self):
        super().__init__('map_resume_check')
        g = self.declare_parameter
        self.map_frame = g('map_frame', 'map').value
        self.odom_frame = g('odom_frame', 'odom').value
        self.base_frame = g('base_frame', 'base_link').value
        # 期望值 = 存档 sidecar 里记录的 map_start_pose（= 上一次用它时"机器人在旧图的哪儿"）
        self.expected = [float(v) for v in g('expected_pose', [0.0, 0.0, 0.0]).value]
        self.archive_name = g('archive_name', '').value
        self.archive_base = g('archive_base', '').value
        self.archive_world = g('archive_world', '').value
        self.archive_spawn = [float(v) for v in g('archive_spawn', [0.0, 0.0, 0.0, 0.0]).value]
        self.pcd_path = g('pcd_path', '').value
        # 静置/收敛窗口：从**第一次拿到 map→base_link** 起算（那时 slam_toolbox 已经反序列化完）
        self.check_delay = float(g('check_delay', 8.0).value)
        self.still_window = float(g('still_window', 1.0).value)
        self.still_eps = float(g('still_eps', 0.05).value)
        self.settle_timeout = float(g('settle_timeout', 30.0).value)
        self.tf_wait_timeout = float(g('tf_wait_timeout', 60.0).value)
        self.pos_tol = float(g('pos_tol', 0.5).value)
        self.yaw_tol = float(np.radians(g('yaw_tol_deg', 10.0).value))
        self.strict = bool(g('strict', False).value)
        self.tf_timeout = float(g('tf_timeout', 0.15).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t_start = time.monotonic()
        self.t_base = None            # 第一次拿到 map→base_link 的时刻
        self.odom0 = None             # 基线：odom→base_link @ t_base
        self.M0 = None                # 基线：map→odom @ t_base
        self.pose_base = None         # 基线：map→base_link @ t_base
        self.moved_since_base = 0.0   # 自基线以来 map→base_link 平移的最大变化量
        self.samples = []             # [(t, 平移范数)] 用于"是否静止"
        self.exit_code = 0
        self.done = False
        self.create_timer(0.25, self.on_tick)
        self.get_logger().info(
            'map_resume_check 起（一次性检查，不发布任何东西）：存档=%s world=%s\n'
            '  期望 map_start_pose=%s（存档记录） 存档 spawn=%s\n'
            '  静置窗口=%.1f s（首次拿到 TF 起算）/ 静止判定=%.3f m / 超时 %.0f s 后按里程计补偿照判\n'
            '  阈值：位置 %.2f m / 偏航 %.1f°；strict=%s'
            % (self.archive_name or '(未给)', self.archive_world or '(未给)',
               ['%.3f' % v for v in self.expected],
               ['%.3f' % v for v in self.archive_spawn],
               self.check_delay, self.still_eps, self.settle_timeout,
               self.pos_tol, np.degrees(self.yaw_tol), self.strict))

    # ---------------------------------------------------------------- 主循环
    def on_tick(self):
        if self.done:
            return
        now = time.monotonic()
        try:
            # 三者在**同一次 tick** 里取，map→base_link 由 map→odom ∘ odom→base_link 自己算，
            # 避免两次独立查询取到不同时刻的快照（机器人动起来时那会引入假偏差）。
            m = tf_to_mat(self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rclpy.time.Time(),
                Duration(seconds=self.tf_timeout)))
            o = tf_to_mat(self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(),
                Duration(seconds=self.tf_timeout)))
        except TransformException:
            if self.t_base is None and now - self.t_start > self.tf_wait_timeout:
                self.get_logger().warn(
                    '⚠️ 等了 %.0f s 还没拿到 %s→%s 的 TF ⇒ **跳过**续建一致性检查'
                    '（位姿图还可能在加载 / LIO 没起来；不是"检查通过"）'
                    % (self.tf_wait_timeout, self.map_frame, self.base_frame))
                self.finish(0)
            return
        pose = m @ o
        if self.t_base is None:
            self.t_base, self.M0, self.odom0, self.pose_base = now, m, o, pose
            self.get_logger().info('已拿到 %s→%s：%s ⇒ 开始 %.1f s 静置窗口'
                                   '（基线：map→odom=%s odom→base=%s）'
                                   % (self.map_frame, self.base_frame, fmt_pose(pose), self.check_delay,
                                      fmt_pose(m), fmt_pose(o)))
            self.samples = [(now, float(np.linalg.norm(pose[:3, 3])))]
            return
        # 「有没有动过」= 自基线以来 map→base_link 平移的最大变化量（不是只看看最后一秒）：
        # 只有"从头到尾一步没动"才敢把绝对偏差直接算到图/起点头上（见 verdict 的两条判据）。
        self.samples.append((now, float(np.linalg.norm(pose[:3, 3]))))
        self.samples = [(t, v) for t, v in self.samples if now - t <= max(self.still_window, 1.0)]
        moved = 0.0
        if len(self.samples) > 1:
            vals = [v for _t, v in self.samples]
            moved = max(vals) - min(vals)
        self.moved_since_base = max(self.moved_since_base,
                                    float(np.linalg.norm(pose[:3, 3] - self.pose_base[:3, 3])))
        elapsed = now - self.t_base
        if elapsed < self.check_delay:
            return
        if self.moved_since_base <= self.still_eps:
            self.verdict(pose, o, still=True, moved=moved)
        elif elapsed > self.check_delay + self.settle_timeout:
            self.verdict(pose, o, still=False, moved=moved)
        # 否则：还没静下来，再等（最多 settle_timeout）

    # ---------------------------------------------------------------- 判定
    def verdict(self, pose, o, still, moved):
        """两条判据（2026-10-06 用 7 个合成场景标定过，见 docs/continue_mapping.md §8 与
        .tmp_hygiene/resume_check_test.sh）：

        ① `dev_growth` = **自基线以来 map→odom 修正了多少**（等价于"实际位姿 vs 按里程计推算的位姿"）。
           它不依赖"机器人是否静止"，是判断"图把位姿拉走了"的量 —— 图歪了就会一直涨。
        ② `dev_abs` = 实测 map→base_link 与**存档记录的 map_start_pose** 的差。
           只有在"自基线以来一步没动"时才敢用它（否则机器人自己走出来的位移会被误算成图的错）。

        为什么必须有 ①：用户常常一起来就开走；只用 ② 会把"正常开走"误报成"存档坏了"（实测 7.5 m 假偏差）。
        为什么必须有 ②：存档 sidecar 记的起点与真实不符（起点被挪 / 被 doctored）时，
        扫描匹配不会产生任何修正 ⇒ ①≈0，只有 ② 抓得住。
        """
        # ① 修正量：期望 = 基线位姿 ⊕ 基线以来的里程计位移
        exp_growth = self.pose_base @ (np.linalg.inv(self.odom0) @ o)
        dev = np.linalg.inv(exp_growth) @ pose
        pos_dev = float(np.linalg.norm(dev[:3, 3]))
        yaw_dev = abs(yaw_of(dev))
        # ② 绝对偏差：与存档记录的 map_start_pose 比（静止时才有效）
        raw = np.linalg.inv(pose_from_xy_yaw(*self.expected)) @ pose
        pos_raw = float(np.linalg.norm(raw[:3, 3]))
        yaw_raw = abs(yaw_of(raw))
        bad_growth = (pos_dev > self.pos_tol) or (yaw_dev > self.yaw_tol)
        bad_abs = still and ((pos_raw > self.pos_tol) or (yaw_raw > self.yaw_tol))
        bad = bad_growth or bad_abs
        nums = ('  · 实测 %s→%s : %s\n'
                '  · ① 修正量（期望=基线%s ⊕ 里程计位移）= %s\n'
                '       偏差 %.3f m / %.2f°（阈值 %.2f m / %.1f°）⇒ %s\n'
                '  · ② 与存档记录 map_start_pose%s 的绝对偏差 = %.3f m / %.2f°（阈值同上）⇒ %s\n'
                '  · 机器人状态  : %s（自基线以来最大位移 %.3f m；静止判据 %.3f m）'
                % (self.map_frame, self.base_frame, fmt_pose(pose), fmt_pose(self.pose_base),
                   fmt_pose(exp_growth), pos_dev, np.degrees(yaw_dev), self.pos_tol,
                   np.degrees(self.yaw_tol), '❌ 超阈值' if bad_growth else '✅ 在阈值内',
                   ['%.3f' % v for v in self.expected], pos_raw, np.degrees(yaw_raw),
                   '❌ 超阈值' if bad_abs else ('✅ 在阈值内' if still else '— 机器人动过，这条不参与判定'),
                   '自基线以来一步没动' if still else '动过（按里程计补偿后判定）',
                   self.moved_since_base, self.still_eps))
        ctx = ('  存档：%s（world=%s）\n  存档文件：%s\n  存档记录 spawn：x=%.3f y=%.3f z=%.3f yaw=%.2f°'
               % (self.archive_name or '(未给)', self.archive_world or '(未给)',
                  self.archive_base or '(未给)', self.archive_spawn[0], self.archive_spawn[1],
                  self.archive_spawn[2], np.degrees(self.archive_spawn[3])))
        if bad:
            who = []
            if bad_growth:
                who.append('扫描匹配把位姿从「起点 + 里程计推算」的位置拉开了 %.3f m / %.2f°'
                           '（= 图里的几何与真实场地不一致）' % (pos_dev, np.degrees(yaw_dev)))
            if bad_abs:
                who.append('实测起点与存档记录的起点差 %.3f m / %.2f°'
                           '（= 存档的 map_start_pose 与现实不符）' % (pos_raw, np.degrees(yaw_raw)))
            head = ('[map_resume_check] ❌❌ 续建一致性检查**不通过**：加载进来的位姿图与出生点/现实对不上，'
                    '这份存档很可能是**退化过的**（LIO 曾经漂过，见 docs/continue_mapping.md §9）\n'
                    '  · 判定依据：%s' % '；'.join(who))
            advice = ('  这意味着什么：map→base_link 与存档记录的起点差了这么多 ⇒ 位姿图里的几何与真实场地不一致；'
                      '再往这份存档上 `save` 就是**同名覆盖**，会把可能还好的那份存档一起弄坏。\n'
                      '  建议（按优先级）：\n'
                      '    ① 本次**不要 save 到这个名字**：换一个新名字另存 —— 重启加 map_name:=%s_new，'
                      '或在运行期 ros2 param set /cloud_accumulator map_name %s_new\n'
                      '    ② 这份存档先留着别动，用备份回滚到上一代：'
                      'tools/scripts/mapping/map_archive.sh restore --name %s\n'
                      '    ③ 顺手体检 3D 先验的 z 跨度：python3 tools/scripts/mapping/pcd_stats.py %s'
                      '（健康云 z 跨度 ≈2 m；>3 m 基本就是被错位姿污染了）\n'
                      '    ④ 确认是"起点猜错"而不是"图歪了"：检查本次 map_start_pose:=%s 是否真的等于'
                      '机器人在旧图里的位姿\n'
                      '    ⑤ 只想先跑起来、不要这个检查：加 map_resume_check:=False'
                      % (self.archive_name or 'X', self.archive_name or 'X', self.archive_name or 'X',
                         self.pcd_path or ('PCD/%s.pcd' % (self.archive_name or 'X')),
                         ['%.3f' % v for v in self.expected]))
            self.get_logger().error(head + '\n' + nums + '\n' + ctx + '\n' + advice)
            if self.strict:
                self.get_logger().error(
                    'map_resume_check_strict:=True ⇒ **收栈**（退出码 1）：这次加载的位姿图不可信，'
                    '先解决它再建图。')
                self.finish(1)
            else:
                self.get_logger().warn(
                    '（当前是"只喊不拦"模式；要让这一条直接收栈请加 map_resume_check_strict:=True）')
                self.finish(0)
        else:
            self.get_logger().info(
                '✅ 续建一致性检查通过：加载的位姿图与出生点一致\n' + nums + '\n' + ctx +
                '\n  （这只说明"起点对得上"；长跑中的漂移仍要靠 [status] 的 bbox 与 save 前体检兜住）')
            self.finish(0)

    def finish(self, code):
        """一次性检查结束：**直接退出**。

        ⚠️ 2026-10-06 实测：在定时器回调里调 `rclpy.shutdown()` 之后本节点**不退出**
        （进程卡在 rclpy 收尾里，连 `SIGTERM` 都被 rclpy 自己的 handler 吃掉 ⇒ `timeout` 也杀不掉它；
        `.tmp_hygiene/mini.py` 复现、`.tmp_hygiene/mini2.py` 验证修法）。
        改用 `SystemExit`：从回调里抛出 ⇒ 穿过 `rclpy.spin()`，main 的 finally 正常清理，
        退出码就是这里的 code（实测 rc=7）。strict 模式靠这个退出码让 launch 收栈。
        """
        self.exit_code = int(code)
        self.done = True
        raise SystemExit(self.exit_code)


def main(argv=None):
    rclpy.init(args=argv)
    node = MapResumeCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:                                    # pragma: no cover
        pass
    except ExternalShutdownException:                            # 自己调 rclpy.shutdown() 收尾时的正常路径
        pass
    except Exception as e:                                       # pragma: no cover
        node.get_logger().error('map_resume_check 异常：%s' % e)
        if not node.done:
            node.exit_code = 1
    finally:
        code = node.exit_code
        try:
            node.destroy_node()
        except Exception:                                        # pragma: no cover
            pass
    return code


if __name__ == '__main__':
    sys.exit(main())
