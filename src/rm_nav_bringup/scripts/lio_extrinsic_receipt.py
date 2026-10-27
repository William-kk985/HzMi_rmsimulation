#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lio_extrinsic_receipt —— **一次性**断言「LIO 真正生效的外参 == URDF 解析出来的外参」。

为什么需要它（2026-10-27，契约 §4.12.5）：
  `bringup_sim.launch.py` 从 URDF 解析 T(imu←livox) 并注入 LIO，但**每个 LIO 读的键名不一样**：
    · fast_lio        : `mapping.extrinsic_T` / `mapping.extrinsic_R`（laserMapping.cpp:834–835/870–871）
    · point_lio       : `mapping.extrinsic_T` / `mapping.extrinsic_R`（parameters.cpp:179–183）
    · small_point_lio : 顶层 `extrinsic_T` / `extrinsic_R`（small_point_lio/parameters.cpp:110–116）
  launch 原来给三个槽位**都**传了顶层 `extrinsic_T/R` ⇒ fastlio/pointlio 收不到（死键），
  robot11 的 +30° 安装角**静默丢失**（运行期 `mapping.extrinsic_R = I`，§34.3#4 实测）。
  这类错误**没有任何一条告警**：节点照常启动、照常发 odom，只是多半圈 30°。

本节点做什么（只读，不发布任何话题/TF、不改参数）：
  1. 等目标 LIO 节点的 `get_parameters` 服务就绪；
  2. 读回**该后端自己的**两个键 + **另一个命名空间**的两个键；
  3. 与 launch 从 URDF 解析出的 expected_T/expected_R 逐元素比较（默认 tol = 1e-9）；
  4. 另一个命名空间里**只要设了值**就报「死键」（那是"传到没人读的键"的典型形态）；
  5. 打一行 receipt（OK/ERROR），不一致 ⇒ **ERROR + exit 1**。

为什么是「读回」而不是 launch 期自检：launch 只能检查"我传了什么"，
  检查不了"节点读的是什么"；`ros2 param get` 读的才是**生效值**（rclcpp 的
  declare/get_parameter 看到的那一份）。键名写错时，节点照跑，只有读回能抓住。

退出码：0 = 一致；1 = 不一致 / 键缺失 / 类型错 / 死键 / 超时（`strict:=True` 时）。
  （不自动收栈：宁可让用户看到红色 ERROR + `process has died`，也不要因一次误判把整栈收掉。）

参数（全部由 launch 传，见 `_lio_extrinsic_check_params()`）：
  · `target_node`   ：LIO 节点名（`/laser_mapping` / `/laserMapping` / `/small_point_lio`）
  · `param_prefix`  ：`mapping` 或 `''`（空 = 顶层键）—— **与 launch 的 `_LIO_EXTRINSIC_KEYS` 同源**
  · `expected_T`    ：3 个数（URDF 解析的 t_IL）
  · `expected_R`    ：9 个数（URDF 解析的 R_IL，行主序）
  · `timeout_s` / `tol` / `strict`
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters

#: 与 launch 的 `_MountExtrinsic` 解析失败回退值一致（只在 expected 参数缺失时用）
_DEFAULT_R = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]


def _fmt(v, nd=9):
    """稳定、可 grep 的一行格式（不用 repr，避免 -0.0 与 0.0 难比对）。"""
    return '[%s]' % ', '.join(('%.*f' % (nd, x)) for x in v)


def _rpy_deg(R):
    """行主序 9 个数 → rpy（度）。只用于日志可读（Rx(+30°) 应显示 30）。"""
    if len(R) != 9:
        return None
    sy = max(-1.0, min(1.0, -R[6]))
    pitch = math.asin(sy)
    roll = math.atan2(R[7], R[8])
    yaw = math.atan2(R[3], R[0])
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


class LioExtrinsicReceipt(Node):
    def __init__(self):
        super().__init__('lio_extrinsic_receipt')
        target = str(self.declare_parameter('target_node', '/laserMapping').value).strip()
        target = target or '/laserMapping'
        self.target = target if target.startswith('/') else '/' + target
        prefix = str(self.declare_parameter('param_prefix', 'mapping').value).strip().strip('.')
        self.prefix = prefix
        self.expected_T = [float(v) for v in
                           self.declare_parameter('expected_T', [0.0, 0.0, 0.05]).value]
        self.expected_R = [float(v) for v in
                           self.declare_parameter('expected_R', list(_DEFAULT_R)).value]
        self.timeout_s = float(self.declare_parameter('timeout_s', 90.0).value)
        self.tol = float(self.declare_parameter('tol', 1e-9).value)
        self.strict = bool(self.declare_parameter('strict', True).value)

        def join(p, leaf):
            return ('%s.%s' % (p, leaf)) if p else leaf

        self.key_T = join(self.prefix, 'extrinsic_T')
        self.key_R = join(self.prefix, 'extrinsic_R')
        # 「另一个命名空间」= 死键最常见的落点（launch 传错前缀时就是它被写进去）
        alt = '' if self.prefix == 'mapping' else 'mapping'
        self.alt_T = join(alt, 'extrinsic_T')
        self.alt_R = join(alt, 'extrinsic_R')

        self._cli = self.create_client(GetParameters,
                                       '%s/get_parameters' % self.target)
        # 死键探测走 `list_parameters`（只读名字）而不是 `get_parameters`：
        #   rclcpp 的 parameter service 对**未声明**的名字会打一条
        #   `[WARN] Failed to get parameters: extrinsic_T` —— 语义上"这里没有这个键"是**好事**，
        #   但日志里出现两条 WARN 会让用户误以为 LIO 出问题。名字列表不触发那条 WARN。
        self._list_cli = self.create_client(ListParameters,
                                            '%s/list_parameters' % self.target)

    def ready(self):
        return (self._cli.service_is_ready() or self._cli.wait_for_service(timeout_sec=0.5)) \
            and (self._list_cli.service_is_ready()
                 or self._list_cli.wait_for_service(timeout_sec=0.5))

    @staticmethod
    def _as_list(pv):
        """ParameterValue → (list|None, 说明)。None = 未设置。"""
        if pv.type == ParameterType.PARAMETER_NOT_SET:
            return None, '未设置'
        if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return [float(x) for x in pv.double_array_value], 'double[]'
        return None, '类型错(type=%d)' % pv.type

    def _read(self, names):
        req = GetParameters.Request()
        req.names = list(names)
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            return None
        # GetParameters 对每个请求名返回一个值（不存在 ⇒ PARAMETER_NOT_SET）
        return list(res.values)

    def _list_names(self):
        """节点上**已声明**的参数名集合（失败 → None；`prefixes=['']` + depth 0 = 全部）。"""
        req = ListParameters.Request()
        req.prefixes = ['']
        req.depth = 0
        fut = self._list_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            return None
        return set(res.result.names)

    def check(self):
        """返回 (ok, head, problems)。

        ⚠️ 真实键必须**单独读**：rclpy 的 parameter service 在遇到任何一个**未声明**的名字时
        会把整个 response.values 清空（`parameter_service.py:83-86`），而 rclcpp 的服务对每个
        名字回一个 PARAMETER_NOT_SET。若把"真实键 + 死键"放进同一个 get_parameters 请求，
        对 rclpy 节点会永远读到 0 个值（假失败）⇒ 真实键走 get_parameters、死键走 list_parameters。
        """
        vals = self._read([self.key_T, self.key_R])
        if vals is None or len(vals) != 2:
            return False, 'node=%s' % self.target, [
                '读参数服务失败（%s/get_parameters 无响应 / 返回 %s 个值）'
                % (self.target, '?' if vals is None else len(vals))]
        act_T, why_T = self._as_list(vals[0])
        act_R, why_R = self._as_list(vals[1])
        names = self._list_names()
        dead_T = dead_R = None
        if names is not None:
            if self.alt_T in names:
                dead_T = '（已声明）'
            if self.alt_R in names:
                dead_R = '（已声明）'

        problems = []
        if act_T is None:
            problems.append('`%s` 读不到（%s）' % (self.key_T, why_T))
        elif len(act_T) != 3:
            problems.append('`%s` 长度 %d ≠ 3' % (self.key_T, len(act_T)))
        else:
            dT = max(abs(a - b) for a, b in zip(act_T, self.expected_T))
            if dT > self.tol:
                problems.append('`%s` 不一致：实际 %s vs expected %s（max|Δ|=%.3e）'
                                % (self.key_T, _fmt(act_T), _fmt(self.expected_T), dT))
        if act_R is None:
            problems.append('`%s` 读不到（%s）' % (self.key_R, why_R))
        elif len(act_R) != 9:
            problems.append('`%s` 长度 %d ≠ 9' % (self.key_R, len(act_R)))
        else:
            dR = max(abs(a - b) for a, b in zip(act_R, self.expected_R))
            if dR > self.tol:
                hint = ''
                if all(abs(a - b) <= self.tol for a, b in
                       zip(act_R, _DEFAULT_R)) and not all(
                           abs(a - b) <= self.tol for a, b in zip(self.expected_R, _DEFAULT_R)):
                    hint = ' —— 实际 = **I**：安装角根本没进来（典型死键症状）'
                problems.append('`%s` 不一致：实际 %s vs expected %s（max|Δ|=%.3e）%s'
                                % (self.key_R, _fmt(act_R), _fmt(self.expected_R), dR, hint))
        if dead_T is not None:
            problems.append('**死键** `%s` 存在（该后端不读这个键；删掉它）' % self.alt_T)
        if dead_R is not None:
            problems.append('**死键** `%s` 存在（该后端不读这个键；删掉它）' % self.alt_R)

        rpy = None
        if act_R is not None and len(act_R) == 9:
            rpy = _rpy_deg(act_R)
        head = ('node=%s keys=%s/%s（另一个命名空间 %s/%s）actual T=%s R=%s rpy(R)=%s'
                % (self.target, self.key_T, self.key_R, self.alt_T, self.alt_R,
                   _fmt(act_T) if act_T is not None else '(未设置)',
                   _fmt(act_R) if act_R is not None else '(未设置)',
                   '[%s]°' % ', '.join('%.3f' % x for x in rpy) if rpy else '(n/a)'))
        return (not problems), head, problems

    def report(self):
        ok, head, problems = self.check()
        if ok:
            self.get_logger().info(
                '[lio_extrinsic_receipt] OK — 生效外参 == URDF：%s；expected T=%s R=%s '
                '（tol=%.0e）'
                % (head, _fmt(self.expected_T), _fmt(self.expected_R), self.tol))
        else:
            self.get_logger().error(
                '[lio_extrinsic_receipt] ERROR — 生效外参 != URDF（**不要**继续信这栈的 odom/地图）：'
                '%s\n  expected T=%s R=%s\n  问题：%s\n'
                '  ⇒ 查 launch 的 `_LIO_EXTRINSIC_KEYS`（契约 §4.12.5 有 per-LIO 键表）'
                % (head, _fmt(self.expected_T), _fmt(self.expected_R),
                   '；'.join(problems)))
        return ok


def main(argv=None):
    rclpy.init(args=argv)
    node = LioExtrinsicReceipt()
    ok = False
    t0 = time.time()
    try:
        while time.time() - t0 < node.timeout_s:
            if not node.ready():
                continue
            ok = node.report()
            break
        else:
            node.get_logger().error(
                '[lio_extrinsic_receipt] ERROR — %.1f s 内没等到 %s 的 get_parameters 服务'
                '（节点没起来 / 名字错？）⇒ 无法确认外参是否生效'
                % (node.timeout_s, node.target))
            ok = False
    except Exception as exc:                                   # noqa: BLE001
        node.get_logger().error('[lio_extrinsic_receipt] ERROR — 检查异常：%r' % (exc,))
        ok = False
    finally:
        try:
            node.destroy_node()
        except Exception:                                      # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    if not ok and node.strict:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
