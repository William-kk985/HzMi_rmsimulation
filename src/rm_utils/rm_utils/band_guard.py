#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""band_guard —— nav2 高度带（min/max_obstacle_height）**静默丢波束**的只读守卫。

背景（docs/frame_contract_v3_measurements.md §15 的两个失败，都是**全静默**的）：

  ① 局部代价图（`global_frame: odom`，`gravity_align_mode:=off`）：`/scan` 是二维平盘，
     nav2 的 `obstacle_layer` 先 projectLaser（z=0）再把它搬到代价图帧；odom 一旦不重力
     对齐，**每条波束在 odom 里的 z 随 x/y 变化** ⇒ 带子只留下一个**斜板**：实测
     in-band 333/774、kept y ∈ [0.112, 3.568] m（半个 y 方向整片看不见）。
  ② 全局代价图（`global_frame: map`）：扫描平面在 map 里的 z ≈ −0.1786 ⇒ 默认下界 0.0
     把 **774/774** 全丢 ⇒ 云被清空，全局图恒等于 static+inflation。nav2 对此没有
     任何 WARNING / dropped 计数（`ObservationBuffer::bufferCloud`
     `nav2_costmap_2d/src/observation_buffer.cpp:143-144`）。

本节点做的事（**只读**，绝不改任何数据/控制路径）：

  · 订阅 `/scan`（sensor QoS）。
  · 从**活着的**两张代价图节点的参数服务回读**生效值**（单一真源，见下）：
      `plugins`、`<layer>.plugin/.enabled/.observation_sources`、
      `<layer>.min_obstacle_height/.max_obstacle_height`（nav2 的**第二道**闸）、
      `<layer>.<source>.min/max_obstacle_height`（ObservationBuffer 的**第一道**闸）、
      `global_frame`、`transform_tolerance`。
  · 用 TF2 把每条波束的**扫描系**点 `(r·cos a, r·sin a, 0)` 搬到代价图帧（**优先用
    scan 的 stamp**；查不到才退回最新一帧，并在日志里标 `mode=latest`），统计
    `in-band / below / above`、**kept y 区间**（半平面盲区的直接证据）、z 的 min/p50/max。
  · 按规则**只写日志 + 发一个状态话题**（不发布 TF、不发布 /scan、不碰 costmap）：
      `in-band == 0`（且 /scan 在正常发布）          → ERROR
      `in-band < 10%`                                → WARN
      kept-y 单侧（相对 scan 原点的 y）              → WARN（带两个数字）
    每个 costmap 的日志最多 `rate_limit_s`（默认 5 s）一条。
  · 发布 `std_msgs/String`（JSON）到 `/band_guard/status`（transient_local，方便
    A/B 记录/后加入的读者拿最后一条）。

**单一真源的选择（为什么不是 launch 传参）**：
  本节点直接读两张 costmap 的**运行期参数服务**（`/local_costmap/local_costmap`、
  `/global_costmap/global_costmap` 的 `<node>/list_parameters` + `<node>/get_parameters`），
  而不是由 launch 从同一份 nav2 params 文件里再算一遍。理由：
    1. 不会漂移 —— launch 的深合并（base → robot11 覆盖 → global_obstacle:=scan 覆盖）
       与 RewrittenYaml 的 rewrite 都可能改值；参数服务返回的是 nav2 **真正拿到**的值；
    2. 可独立运行 —— `ros2 run rm_utils band_guard.py` 能挂到**任意**已在跑的栈上，
       不需要用户告诉它参数文件在哪、也不需要 launch 改动；
    3. 覆盖动态改参 —— 参数被 `ros2 param set` 改过（或以后加了新的槽位覆盖）也能看到。
  代价：需要 nav2 节点已起、参数服务可查；本节点用 500 ms 轮询等待它们（不挡任何启动），
  参数缓存默认 30 s 刷新一次（服务流量很小；§15.11 记录过 `ros2 param get` 子进程风暴
  会扰动 MessageFilter，这里用**进程内**异步客户端，且只查一次 ~30 个键）。

用法：
  由 launch 起（launch arg `band_guard:=True`，默认 False）：
      ros2 launch rm_nav_bringup bringup_sim.launch.py mode:=nav ... band_guard:=True
  独立挂到已在跑的栈（推荐先 `source install/setup.bash`）：
      ros2 run rm_utils band_guard.py --ros-args -p use_sim_time:=true
      # 换节点名/话题/周期：
      ros2 run rm_utils band_guard.py --ros-args \
          -p use_sim_time:=true -p period_s:=2.0 \
          -p costmap_nodes:="['/local_costmap/local_costmap','/global_costmap/global_costmap']"
  纯源码树（不装）：
      python3 src/rm_utils/rm_utils/band_guard.py --ros-args -p use_sim_time:=true

参数（全部可在 launch / 命令行覆盖）：
  scan_topic             /scan
  costmap_nodes          ['/local_costmap/local_costmap', '/global_costmap/global_costmap']
  period_s               5.0      每次评估的仿真/墙钟周期（节点时钟）
  params_refresh_s       30.0     参数缓存刷新周期
  min_scan_rate_hz       1.0      低于它视为“/scan 没在正常发布” ⇒ 不报带子 ERROR
  scan_stale_s           1.0      最新 /scan 的最大龄期（秒）
  inband_warn_fraction   0.10     in-band 比例低于它 ⇒ WARN
  y_onesided_margin_m    0.05     kept-y 相对 scan 原点的单侧判据余量
  rate_limit_s           5.0      每个 costmap 的日志最小间隔（墙钟）
  status_topic           /band_guard/status
  publish_status         True

契约：只订阅 `/scan`、`/tf`、`/tf_static`，只调用**只读**参数服务，只发布
`/band_guard/status`。不发布 TF、不订阅/发布任何控制话题、不写文件。
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

# 严重度排序（只用于“升级立即打印 / 降级打一条恢复 INFO”）
_SEV_RANK = {'IDLE': 0, 'NO_DATA': 0, 'PARAMS_PENDING': 0,
             'OK': 1, 'NO_POINTS': 2, 'TF_FAIL': 2, 'WARN': 3, 'ERROR': 4}

# 参数服务里我们关心的后缀（其余键不查，保持服务流量最小）
_WANTED_SUFFIXES = (
    '.plugin', '.enabled', '.observation_sources',
    '.min_obstacle_height', '.max_obstacle_height', '.data_type', '.topic',
)


def _num(x, nd=3):
    """稳定的数字格式（None/NaN ⇒ '-'；避免 -0.000）。"""
    if x is None:
        return '-'
    try:
        v = float(x)
    except (TypeError, ValueError):
        return '-'
    if not math.isfinite(v):
        return '-'
    if abs(v) < 0.5 * 10.0 ** (-nd):
        v = 0.0
    return f'{v:.{nd}f}'


def _percentile(values, pct):
    """线性插值分位数（values 非空）。"""
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    k = (len(s) - 1) * (pct / 100.0)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return float(s[lo])
    return float(s[lo] + (s[hi] - s[lo]) * (k - lo))


def _quat_rows(x, y, z, w):
    """返回旋转矩阵的第 1、2 行（只要 y/z 分量；纯 Python，避免 numpy 依赖）。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    s = 2.0 / n
    # 旋转矩阵的第 1、2 行（xyzw 四元数）：
    #   R = [[1-s(y²+z²), s(xy-zw),   s(xz+yw)],
    #        [s(xy+zw),   1-s(x²+z²), s(yz-xw)],
    #        [s(xz-yw),   s(yz+xw),   1-s(x²+y²)]]
    r1 = (s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w))
    r2 = (s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y))
    return r1, r2


def project_scan(scan, min_height, max_height, transform, inf_is_valid=True):
    """把 LaserScan 按 nav2 的口径搬到代价图帧并统计高度带。

    严格复刻 nav2 的两步：
      · `laser_geometry::projectLaser`：扫描系里 (r·cos a, r·sin a, 0)，只保留
        `range_min <= r < range_max`（`range_cutoff` 默认 = range_max）；
      · 若源 `inf_is_valid` 为真，nav2 先把 +inf 换成 `range_max - 1e-4`
        （`ObstacleLayer::laserScanValidInfCallback`），再投影。
      · `ObservationBuffer::bufferCloud` 在**代价图帧**里按 [min,max] 过滤 z。
        这里直接对每条波束算 z（等价于整朵云搬过去后逐点看 z）。

    @param transform (rotation_rows, translation)：
        rotation_rows = (R1, R2)（旋转矩阵第 1、2 行），translation = (tx, ty, tz)。
    @return dict（含 in_band/below/above、kept_y、z 统计、total、raw_beams 等）。
    """
    (r1, r2), (tx, ty, tz) = transform
    r1x, r1y, _r1z = r1
    r2x, r2y, _r2z = r2

    ranges = scan.ranges
    n_raw = len(ranges)
    amin = float(scan.angle_min)
    ainc = float(scan.angle_increment)
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)

    total = 0
    below = 0
    above = 0
    in_band = 0
    kept_ys = []
    all_ys = []
    zs = []
    # 只统计**有限** range 的同一组量（= 既往探针 `n_finite` 的口径，便于跨文档比较）。
    # nav2 在 inf_is_valid=true 时会把 +inf 换成 range_max-eps **一起投影** ⇒ nav2 的真实
    # 云大小 = total（可能比 finite_total 多几条最大量程点）；两个数都打出来。
    finite_total = 0
    finite_below = 0
    finite_above = 0
    finite_in_band = 0
    finite_kept_ys = []
    finite_all_ys = []
    finite_zs = []
    inf_converted = 0

    for i, raw in enumerate(ranges):
        r = float(raw)
        is_finite = math.isfinite(r)
        converted = False
        if not is_finite:
            # nav2 的 inf_is_valid=true 只把正无穷换成 range_max - eps；NaN/-inf 仍被丢。
            if inf_is_valid and r > 0.0:
                r = rmax - 1e-4
                converted = True
            else:
                continue
        if not (r >= rmin and r < rmax):
            continue
        if converted:
            inf_converted += 1
        a = amin + i * ainc
        x = r * math.cos(a)
        y = r * math.sin(a)
        # 扫描系 z 恒为 0 ⇒ 只剩旋转矩阵第 1/2 行 + 平移
        zc = r2x * x + r2y * y + tz
        yc = r1x * x + r1y * y + ty
        total += 1
        zs.append(zc)
        all_ys.append(yc)
        if zc < min_height:
            below += 1
        elif zc > max_height:
            above += 1
        else:
            in_band += 1
            kept_ys.append(yc)
        if is_finite:
            finite_total += 1
            finite_zs.append(zc)
            finite_all_ys.append(yc)
            if zc < min_height:
                finite_below += 1
            elif zc > max_height:
                finite_above += 1
            else:
                finite_in_band += 1
                finite_kept_ys.append(yc)

    def _yrange(ys):
        return {'min': min(ys) if ys else None,
                'max': max(ys) if ys else None,
                'span': (max(ys) - min(ys)) if ys else None}

    def _zrange(zv):
        return {'min': min(zv) if zv else None,
                'p50': _percentile(zv, 50),
                'max': max(zv) if zv else None}

    return {
        'raw_beams': n_raw,
        'total': total,
        'in_band': in_band,
        'below': below,
        'above': above,
        'fraction': (in_band / total) if total else None,
        'finite_total': finite_total,
        'finite_in_band': finite_in_band,
        'finite_below': finite_below,
        'finite_above': finite_above,
        'finite_fraction': (finite_in_band / finite_total) if finite_total else None,
        'inf_converted': inf_converted,
        'kept_y': _yrange(kept_ys),
        'all_y': _yrange(all_ys),
        'z': _zrange(zs),
        'kept_y_finite': _yrange(finite_kept_ys),
        'all_y_finite': _yrange(finite_all_ys),
        'z_finite': _zrange(finite_zs),
    }


class _ParamFetch:
    """一个 costmap 节点的参数读取状态机（异步；永不阻塞 executor）。"""

    def __init__(self, node_name):
        self.node_name = node_name
        self.list_client = None
        self.get_client = None
        self.state = 'wait'       # wait -> list -> get -> done
        self.future = None
        self.sent_wall = 0.0
        self.names = set()
        self.values = None
        self.error = ''

    def clients(self, node):
        if self.list_client is None:
            self.list_client = node.create_client(
                ListParameters, f'{self.node_name}/list_parameters')
        if self.get_client is None:
            self.get_client = node.create_client(
                GetParameters, f'{self.node_name}/get_parameters')
        return self.list_client, self.get_client


class BandGuard(Node):
    def __init__(self):
        super().__init__('band_guard')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('costmap_nodes', [
            '/local_costmap/local_costmap', '/global_costmap/global_costmap'])
        self.declare_parameter('period_s', 5.0)
        self.declare_parameter('params_refresh_s', 30.0)
        self.declare_parameter('min_scan_rate_hz', 1.0)
        self.declare_parameter('scan_stale_s', 1.0)
        self.declare_parameter('inband_warn_fraction', 0.10)
        self.declare_parameter('y_onesided_margin_m', 0.05)
        self.declare_parameter('rate_limit_s', 5.0)
        self.declare_parameter('status_topic', '/band_guard/status')
        self.declare_parameter('publish_status', True)
        self.declare_parameter('log_healthy', True)

        gp = self.get_parameter
        self.scan_topic = str(gp('scan_topic').value)
        self.costmap_nodes = [str(x) for x in gp('costmap_nodes').value]
        self.period_s = max(0.2, float(gp('period_s').value))
        self.params_refresh_s = max(1.0, float(gp('params_refresh_s').value))
        self.min_scan_rate_hz = max(0.0, float(gp('min_scan_rate_hz').value))
        self.scan_stale_s = max(0.1, float(gp('scan_stale_s').value))
        self.inband_warn_fraction = float(gp('inband_warn_fraction').value)
        self.y_margin = float(gp('y_onesided_margin_m').value)
        self.rate_limit_s = max(0.0, float(gp('rate_limit_s').value))
        self.publish_status = bool(gp('publish_status').value)
        self.log_healthy = bool(gp('log_healthy').value)

        self.scan = None
        self.scan_stamps = deque(maxlen=64)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.status_pub = None
        if self.publish_status:
            self.status_pub = self.create_publisher(
                String, str(gp('status_topic').value), status_qos)

        self.fetches = {name: _ParamFetch(name) for name in self.costmap_nodes}
        self.last_readout_key = {}
        self.last_log = {}
        self.last_sev = {}
        self.idle_logged = set()

        self.create_timer(self.period_s, self._on_period)
        # 参数/服务轮询：0.5 s；只做非阻塞 future 检查
        self.create_timer(0.5, self._on_poll)

        self.get_logger().info(
            '[band_guard] 启动（只读）：scan=%s costmaps=%s period=%.1fs '
            'params=live parameter services（单一真源；不读 launch 传参）'
            % (self.scan_topic, self.costmap_nodes, self.period_s))

    # ------------------------------------------------------------------ 时间/scan
    def _now_s(self):
        t = self.get_clock().now()
        return t.nanoseconds * 1e-9

    def _scan_rate(self):
        now = self._now_s()
        stamps = [s for s in self.scan_stamps if now - s <= 3.0]
        if len(stamps) < 2:
            return None
        span = stamps[-1] - stamps[0]
        if span <= 1e-6:
            return None
        return (len(stamps) - 1) / span

    def _scan_ok(self):
        """返回 (ok, reason, age, rate)。"""
        if self.scan is None:
            return False, 'no_scan', None, None
        now = self._now_s()
        stamp = self.scan.header.stamp.sec + self.scan.header.stamp.nanosec * 1e-9
        age = now - stamp
        rate = self._scan_rate()
        if not math.isfinite(age) or age > self.scan_stale_s:
            return False, 'scan_stale', age, rate
        if rate is None or rate < self.min_scan_rate_hz:
            return False, 'scan_rate_low', age, rate
        if len(self.scan.ranges) == 0:
            return False, 'scan_empty', age, rate
        return True, '', age, rate

    def _on_scan(self, msg):
        self.scan = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.scan_stamps.append(stamp)

    # ------------------------------------------------------------------ 参数状态机
    def _maybe_start_fetch(self, fetch):
        list_client, get_client = fetch.clients(self)
        if fetch.state == 'wait':
            if not (list_client.service_is_ready() and get_client.service_is_ready()):
                return
            req = ListParameters.Request()
            # 空 prefixes + depth=0 = 全部声明的参数名（与 `ros2 param list` 同一口径）
            fetch.future = list_client.call_async(req)
            fetch.sent_wall = time.monotonic()
            fetch.state = 'list'
            fetch.error = ''
            return
        if fetch.state == 'list':
            if not fetch.future.done():
                if time.monotonic() - fetch.sent_wall > 5.0:
                    fetch.state = 'wait'
                    fetch.future = None
                return
            try:
                fetch.names = set(fetch.future.result().result.names)
            except Exception as exc:                               # noqa: BLE001
                fetch.error = 'list_parameters: %r' % exc
                fetch.state = 'wait'
                fetch.future = None
                return
            wanted = {'plugins', 'global_frame', 'transform_tolerance'}
            for n in fetch.names:
                if n in wanted or n.endswith(_WANTED_SUFFIXES):
                    wanted.add(n)
            if 'plugins' not in fetch.names:
                fetch.error = 'no "plugins" parameter (costmap not initialized yet?)'
                fetch.state = 'wait'
                fetch.future = None
                return
            req = GetParameters.Request()
            req.names = sorted(wanted)
            fetch.wanted = list(req.names)
            fetch.future = get_client.call_async(req)
            fetch.sent_wall = time.monotonic()
            fetch.state = 'get'
            return
        if fetch.state == 'get':
            if not fetch.future.done():
                if time.monotonic() - fetch.sent_wall > 5.0:
                    fetch.state = 'wait'
                    fetch.future = None
                return
            try:
                resp = fetch.future.result()
            except Exception as exc:                               # noqa: BLE001
                fetch.error = 'get_parameters: %r' % exc
                fetch.state = 'wait'
                fetch.future = None
                return
            values = {}
            for name, pv in zip(getattr(fetch, 'wanted', []), resp.values):
                values[name] = self._pv_value(pv)
            fetch.values = values
            fetch.state = 'done'
            fetch.error = ''
            fetch.future = None
            return


    @staticmethod
    def _pv_value(pv):
        t = pv.type
        if t == ParameterType.PARAMETER_BOOL:
            return bool(pv.bool_value)
        if t == ParameterType.PARAMETER_INTEGER:
            return int(pv.integer_value)
        if t == ParameterType.PARAMETER_DOUBLE:
            return float(pv.double_value)
        if t == ParameterType.PARAMETER_STRING:
            return pv.string_value
        if t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(pv.bool_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(pv.integer_array_value)
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(pv.string_array_value)
        return None

    def _on_poll(self):
        for name, fetch in self.fetches.items():
            if fetch.state != 'done':
                self._maybe_start_fetch(fetch)
            if fetch.state == 'done' and fetch.values is not None:
                key = self._readout_key(fetch.values)
                if self.last_readout_key.get(name) != key:
                    self.last_readout_key[name] = key
                    self.get_logger().info(
                        '[band_guard] 参数回读（live）%s: %s'
                        % (name, self._readout_text(fetch.values)))

    @staticmethod
    def _readout_key(values):
        plugins = tuple(values.get('plugins') or [])
        frame = values.get('global_frame')
        bands = []
        for layer in plugins:
            if values.get('%s.enabled' % layer) is False:
                continue
            bands.append((layer,
                          values.get('%s.min_obstacle_height' % layer),
                          values.get('%s.max_obstacle_height' % layer)))
        return (frame, plugins, tuple(bands))

    @staticmethod
    def _readout_text(values):
        plugins = values.get('plugins') or []
        frame = values.get('global_frame')
        bits = ['frame=%s' % frame, 'plugins=%s' % list(plugins)]
        for layer in plugins:
            en = values.get('%s.enabled' % layer)
            if en is False:
                bits.append('%s(enabled=false)' % layer)
                continue
            band = (values.get('%s.min_obstacle_height' % layer),
                    values.get('%s.max_obstacle_height' % layer))
            if band[0] is not None or band[1] is not None:
                bits.append('%s.band=[%s,%s]' % (layer, _num(band[0]), _num(band[1])))
                for src in str(values.get('%s.observation_sources' % layer) or '').split():
                    dtype = values.get('%s.%s.data_type' % (layer, src))
                    topic = values.get('%s.%s.topic' % (layer, src))
                    sband = (values.get('%s.%s.min_obstacle_height' % (layer, src)),
                             values.get('%s.%s.max_obstacle_height' % (layer, src)))
                    bits.append('%s.%s[%s topic=%s src.band=[%s,%s]]'
                                % (layer, src, dtype, topic,
                                   _num(sband[0]), _num(sband[1])))
        return ' '.join(bits)

    # ------------------------------------------------------------------ 扫描层解析
    def _active_scan_sources(self, values):
        """从生效参数里找出**启用的** obstacle 层的 LaserScan 源。

        @return (sources, skipped)：
          sources = [dict(layer, source, plugin, topic, data_type,
                          layer_min, layer_max, source_min, source_max,
                          eff_min, eff_max, inf_is_valid)]
          skipped = [str]（给状态/日志看的“为什么没算”）
        """
        out = []
        skipped = []
        plugins = values.get('plugins') or []
        for layer in plugins:
            plugin = str(values.get('%s.plugin' % layer) or '')
            enabled = values.get('%s.enabled' % layer)
            if enabled is False:
                skipped.append('%s:enabled=false' % layer)
                continue
            if 'ObstacleLayer' not in plugin or 'spatio' in plugin.lower():
                skipped.append('%s:plugin=%s' % (layer, plugin or '?'))
                continue
            sources = str(values.get('%s.observation_sources' % layer) or '').split()
            if not sources:
                skipped.append('%s:no_sources' % layer)
                continue
            for src in sources:
                dtype = str(values.get('%s.%s.data_type' % (layer, src))
                            or 'LaserScan')
                topic = str(values.get('%s.%s.topic' % (layer, src)) or src)
                if dtype != 'LaserScan':
                    skipped.append('%s.%s:data_type=%s' % (layer, src, dtype))
                    continue
                if topic.lstrip('/') != self.scan_topic.lstrip('/'):
                    skipped.append('%s.%s:topic=%s(not %s)'
                                   % (layer, src, topic, self.scan_topic))
                    continue
                lmin = values.get('%s.min_obstacle_height' % layer)
                lmax = values.get('%s.max_obstacle_height' % layer)
                smin = values.get('%s.%s.min_obstacle_height' % (layer, src))
                smax = values.get('%s.%s.max_obstacle_height' % (layer, src))
                lmin = 0.0 if lmin is None else float(lmin)
                lmax = 2.0 if lmax is None else float(lmax)
                smin = 0.0 if smin is None else float(smin)
                smax = 2.0 if smax is None else float(smax)
                # nav2 **两道闸**都过才算在带内 ⇒ 有效带 = 交集
                eff_min = max(lmin, smin)
                eff_max = min(lmax, smax)
                out.append({
                    'layer': layer, 'source': src, 'plugin': plugin,
                    'topic': topic, 'data_type': dtype,
                    'layer_min': lmin, 'layer_max': lmax,
                    'source_min': smin, 'source_max': smax,
                    'eff_min': eff_min, 'eff_max': eff_max,
                    'inf_is_valid': bool(values.get(
                        '%s.%s.inf_is_valid' % (layer, src), False)),
                })
        return out, skipped

    def _tf(self, target, source, stamp_msg):
        """优先按 scan stamp 查；查不到退回最新一帧（非阻塞，timeout=0）。

        nav2 本身用 `ObservationBuffer` 的 `transform_tolerance` 做查询超时；这里**不阻塞**
        executor（否则 TF 订阅会被饿死），所以读到的 tolerance 只作为可读字段放进状态/日志。
        """
        errs = []
        for mode, t in (('stamp', Time.from_msg(stamp_msg)), ('latest', Time())):
            if mode == 'latest' and not self.tf_buffer.can_transform(
                    target, source, t):
                errs.append('latest: not in buffer')
                continue
            try:
                tr = self.tf_buffer.lookup_transform(target, source, t)
                return tr, mode, errs
            except Exception as exc:                               # noqa: BLE001
                errs.append('%s: %s' % (mode, str(exc)[:180]))
        return None, None, errs

    # ------------------------------------------------------------------ 评估/状态
    def _evaluate(self):
        now = self.get_clock().now()
        scan_ok, scan_reason, scan_age, rate = self._scan_ok()
        cm_status = []
        alerts = []
        for name, fetch in self.fetches.items():
            if fetch.values is None:
                cm_status.append({
                    'node': name, 'state': 'params_pending',
                    'error': fetch.error,
                })
                continue
            values = fetch.values
            frame = str(values.get('global_frame') or '')
            tol = values.get('transform_tolerance')
            tol = 0.3 if tol is None else float(tol)
            sources, skipped = self._active_scan_sources(values)
            if not sources:
                st = {'node': name, 'state': 'idle', 'frame': frame,
                      'skipped': skipped,
                      'note': 'no enabled nav2_costmap_2d::ObstacleLayer with a '
                              'LaserScan source on %s' % self.scan_topic}
                cm_status.append(st)
                if name not in self.idle_logged:
                    self.idle_logged.add(name)
                    self.get_logger().info(
                        '[band_guard] IDLE %s frame=%s：没有启用的 2D scan 高度带'
                        '（%s）；不评估，也不报警'
                        % (name, frame, ', '.join(skipped) or 'no obstacle layer'))
                continue
            if not scan_ok:
                st = {'node': name, 'state': 'no_scan_data',
                      'frame': frame, 'reason': scan_reason,
                      'scan_age_s': scan_age, 'scan_rate_hz': rate,
                      'sources': sources, 'skipped': skipped}
                cm_status.append(st)
                continue

            stamp = self.scan.header.stamp
            for src in sources:
                key = '%s/%s.%s' % (name, src['layer'], src['source'])
                tf, tf_mode, tf_errs = self._tf(frame, self.scan.header.frame_id,
                                                stamp)
                rec = {
                    'node': name, 'state': 'evaluated', 'frame': frame,
                    'layer': src['layer'], 'source': src['source'],
                    'plugin': src['plugin'], 'scan_topic': src['topic'],
                    'band': {'min': src['eff_min'], 'max': src['eff_max'],
                             'source_min': src['source_min'],
                             'source_max': src['source_max'],
                             'layer_min': src['layer_min'],
                             'layer_max': src['layer_max']},
                    'scan': {'frame_id': self.scan.header.frame_id,
                             'stamp': {'sec': int(stamp.sec),
                                       'nanosec': int(stamp.nanosec)},
                             'age_s': scan_age, 'rate_hz': rate},
                    'tf_tolerance_s': tol,
                    'skipped': skipped,
                }
                if tf is None:
                    rec.update({'state': 'tf_fail', 'tf': {
                        'target': frame, 'source': self.scan.header.frame_id,
                        'errors': tf_errs}})
                    rec['severity'] = 'TF_FAIL'
                    rec['hint'] = ('cannot project %s -> %s at scan stamp; '
                                   'no band verdict this cycle'
                                   % (self.scan.header.frame_id, frame))
                    cm_status.append(rec)
                    self._log_record(key, rec)
                    continue

                tr = tf.transform
                q = tr.rotation
                t = tr.translation
                rot = _quat_rows(q.x, q.y, q.z, q.w)
                stats = project_scan(
                    self.scan, src['eff_min'], src['eff_max'],
                    (rot, (float(t.x), float(t.y), float(t.z))),
                    inf_is_valid=src['inf_is_valid'])
                stats['origin_y'] = float(t.y)
                tf_stamp = tf.header.stamp
                tf_age = (stamp.sec - tf_stamp.sec) + \
                    (stamp.nanosec - tf_stamp.nanosec) * 1e-9
                rec.update({
                    'state': 'evaluated',
                    'counts': {'total': stats['total'],
                               'in_band': stats['in_band'],
                               'below': stats['below'],
                               'above': stats['above'],
                               'raw_beams': stats['raw_beams'],
                               'fraction': stats['fraction'],
                               'finite_total': stats['finite_total'],
                               'finite_in_band': stats['finite_in_band'],
                               'finite_below': stats['finite_below'],
                               'finite_above': stats['finite_above'],
                               'finite_fraction': stats['finite_fraction'],
                               'inf_converted': stats['inf_converted']},
                    'kept_y': stats['kept_y'], 'all_y': stats['all_y'],
                    'kept_y_finite': stats['kept_y_finite'],
                    'all_y_finite': stats['all_y_finite'],
                    'z': stats['z'], 'z_finite': stats['z_finite'],
                    'scan_origin_y': stats['origin_y'],
                    'tf': {'target': frame,
                           'source': self.scan.header.frame_id,
                           'mode': tf_mode, 'stamp': {
                               'sec': int(tf_stamp.sec),
                               'nanosec': int(tf_stamp.nanosec)},
                           'age_s': tf_age,
                           'translation': [float(t.x), float(t.y), float(t.z)],
                           'rotation_xyzw': [float(q.x), float(q.y),
                                             float(q.z), float(q.w)]},
                })
                severity, hint = self._classify(stats, src, tf_mode, frame)
                rec['severity'] = severity
                rec['hint'] = hint
                if severity in ('WARN', 'ERROR'):
                    alerts.append({'node': name, 'severity': severity,
                                   'in_band': stats['in_band'],
                                   'total': stats['total'],
                                   'frame': frame, 'hint': hint})
                cm_status.append(rec)
                self._log_record(key, rec)

        status = {
            'schema': 'rm_nav_bringup/band_guard@1',
            'stamp': {'sec': int(now.nanoseconds // 1_000_000_000),
                      'nanosec': int(now.nanoseconds % 1_000_000_000)},
            'scan': {
                'topic': self.scan_topic, 'ok': bool(scan_ok),
                'reason': scan_reason, 'age_s': scan_age,
                'rate_hz': rate,
                'frame_id': (self.scan.header.frame_id if self.scan else None),
                'stamp': ({'sec': int(self.scan.header.stamp.sec),
                           'nanosec': int(self.scan.header.stamp.nanosec)}
                          if self.scan else None),
            },
            'costmaps': cm_status,
            'alerts': alerts,
            'single_source': 'live parameter services',
        }
        if self.status_pub is not None:
            self.status_pub.publish(String(data=json.dumps(
                status, ensure_ascii=False, sort_keys=True)))
        return status

    def _classify(self, stats, src, tf_mode, frame):
        total = stats['total']
        in_band = stats['in_band']
        if total == 0:
            return 'NO_POINTS', ('no projectable beams (range_min/range_max filter); '
                                 'band verdict skipped')
        frac = stats['fraction']
        if in_band == 0:
            return 'ERROR', (
                'ALL %d beams are outside band [%.3f,%.3f] in frame=%s; nav2 will '
                'silently keep 0/%d. Check BOTH %s.%s.min_obstacle_height=%s and '
                '%s.min_obstacle_height=%s (and max).'
                % (total, src['eff_min'], src['eff_max'], frame, total,
                   src['layer'], src['source'], _num(src['source_min']),
                   src['layer'], _num(src['layer_min'])))
        if frac is not None and frac < self.inband_warn_fraction:
            return 'WARN', (
                'only %d/%d (%.1f%%) beams survive band [%.3f,%.3f] in frame=%s; '
                'nav2 silently drops %d. Check %s.%s.min/max_obstacle_height=%s/%s '
                'and %s.min/max_obstacle_height=%s/%s.'
                % (in_band, total, 100.0 * frac, src['eff_min'], src['eff_max'],
                   frame, total - in_band, src['layer'], src['source'],
                   _num(src['source_min']), _num(src['source_max']),
                   src['layer'], _num(src['layer_min']), _num(src['layer_max'])))
        one_sided, side = self._one_sided(stats, src)
        if one_sided:
            return 'WARN', (
                'kept-y is ONE-SIDED (%s): kept=[%s,%s] m while all beams span '
                '[%s,%s] m; nav2 keeps only a slab/half-plane. Check the TF '
                '%s->%s vs band [%.3f,%.3f].'
                % (side, _num(stats['kept_y']['min']), _num(stats['kept_y']['max']),
                   _num(stats['all_y']['min']), _num(stats['all_y']['max']),
                   self.scan.header.frame_id, frame,
                   src['eff_min'], src['eff_max']))
        if tf_mode != 'stamp':
            return 'OK', ('TF at scan stamp unavailable; used latest transform '
                          '(diagnostic only; nav2 uses stamp).')
        return 'OK', 'none'

    def _one_sided(self, stats, src):
        """kept-y 相对 scan 原点单侧 ⇒ (True,'+y'|'-y')。

        只有“全盘跨过原点、而保留部分完全在原点一侧”才报 —— 否则“波束本来就只在
        一侧”的几何会误报。
        """
        if stats['kept_y']['min'] is None:
            return False, ''
        if stats['all_y']['min'] is None or stats['all_y']['max'] is None:
            return False, ''
        # 扫描原点在代价图帧里的 y（= TF 平移 ty）；由 _evaluate 注入 stats['origin_y']。
        origin_y = float(stats.get('origin_y', 0.0))
        m = self.y_margin
        full_crosses = (stats['all_y']['min'] < origin_y - m and
                        stats['all_y']['max'] > origin_y + m)
        if not full_crosses:
            return False, ''
        if stats['kept_y']['min'] > origin_y + m:
            return True, '+y'
        if stats['kept_y']['max'] < origin_y - m:
            return True, '-y'
        return False, ''

    def _log_record(self, key, rec):
        """每个 costmap/源最多 rate_limit_s 一条；升级立即打印，恢复打一条 INFO。"""
        if rec.get('severity') is None:
            return
        sev = rec['severity']
        now_wall = time.monotonic()
        last = self.last_log.get(key)
        last_sev = self.last_sev.get(key)
        changed = last_sev != sev
        due = last is None or (now_wall - last) >= self.rate_limit_s
        # 健康时默认也每周期打一条（便于 A/B 取证）；log_healthy=false 时只在异常/变化时打
        if sev == 'OK' and not self.log_healthy and not changed:
            return
        if not (due or changed):
            return
        self.last_log[key] = now_wall
        self.last_sev[key] = sev
        line = self._format_record(rec)
        if sev == 'ERROR':
            self.get_logger().error(line)
        elif sev in ('WARN', 'TF_FAIL', 'NO_POINTS'):
            self.get_logger().warning(line)
        else:
            self.get_logger().info(line)

    def _format_record(self, rec):
        frame = rec.get('frame', '?')
        node = rec.get('node', '?')
        if rec.get('state') == 'tf_fail':
            t = rec.get('tf', {})
            return ('[band_guard] TF_FAIL %s frame=%s | scan frame=%s | '
                    'cannot transform %s->%s | errors=%s'
                    % (node, frame, rec.get('scan', {}).get('frame_id'),
                       t.get('source'), t.get('target'),
                       '; '.join(t.get('errors') or [])[:400]))
        c = rec.get('counts', {})
        y = rec.get('kept_y', {})
        ay = rec.get('all_y', {})
        z = rec.get('z', {})
        b = rec.get('band', {})
        tf = rec.get('tf', {})
        scan = rec.get('scan', {})
        rate = scan.get('rate_hz')
        frac = c.get('fraction')
        one, side = self._one_sided(
            {'kept_y': y, 'all_y': ay,
             'origin_y': rec.get('scan_origin_y',
                                 tf.get('translation', [None, None])[1])}, b)
        line = (
            '[band_guard] %s %s frame=%s layer=%s source=%s | '
            'in-band %d/%d (%s%%) below=%d above=%d finite=%d/%d inf_converted=%d | '
            'kept-y=[%s, %s] m span=%s origin_y=%s one-sided=%s all-y=[%s, %s] | '
            'finite kept-y=[%s, %s] finite all-y=[%s, %s] | '
            'z min/p50/max=%s/%s/%s finite=%s/%s/%s | '
            'band=[%s,%s] source=[%s,%s] layer=[%s,%s] | '
            'scan frame=%s rate=%s age=%ss | '
            'tf %s->%s @ %ss mode=%s age=%ss | hint=%s'
            % (rec.get('severity'), node, frame, rec.get('layer'),
               rec.get('source'), c.get('in_band', -1), c.get('total', -1),
               '%.1f' % (100.0 * frac) if frac is not None else '-',
               c.get('below', -1), c.get('above', -1),
               c.get('finite_in_band', -1), c.get('finite_total', -1),
               c.get('inf_converted', 0),
               _num(y.get('min')), _num(y.get('max')), _num(y.get('span')),
               _num(tf.get('translation', [None, None])[1]),
               side if one else 'no',
               _num(ay.get('min')), _num(ay.get('max')),
               _num(rec.get('kept_y_finite', {}).get('min')),
               _num(rec.get('kept_y_finite', {}).get('max')),
               _num(rec.get('all_y_finite', {}).get('min')),
               _num(rec.get('all_y_finite', {}).get('max')),
               _num(z.get('min')), _num(z.get('p50')), _num(z.get('max')),
               _num(rec.get('z_finite', {}).get('min')),
               _num(rec.get('z_finite', {}).get('p50')),
               _num(rec.get('z_finite', {}).get('max')),
               _num(b.get('min')), _num(b.get('max')),
               _num(b.get('source_min')), _num(b.get('source_max')),
               _num(b.get('layer_min')), _num(b.get('layer_max')),
               scan.get('frame_id'),
               '%.2fHz' % rate if rate is not None else '-',
               _num(scan.get('age_s'), 3),
               tf.get('source'), tf.get('target'),
               _num(tf.get('stamp', {}).get('sec'), 3),
               tf.get('mode'), _num(tf.get('age_s'), 3),
               rec.get('hint', ''))
        )
        return line

    def _on_period(self):
        # 参数过期 ⇒ 触发刷新；本周期继续用旧缓存（除非从未读到）
        now_wall = time.monotonic()
        for name, fetch in self.fetches.items():
            if fetch.state == 'done' and fetch.values is not None:
                # 用一个简单的墙钟时间戳判断刷新（fetch 完成时记）
                if not hasattr(fetch, 'done_wall'):
                    fetch.done_wall = now_wall
                elif now_wall - fetch.done_wall > self.params_refresh_s:
                    fetch.state = 'wait'
                    fetch.future = None
                    fetch.done_wall = now_wall
        self._evaluate()


def main(args=None):
    rclpy.init(args=args)
    node = BandGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 收栈（SIGINT/SIGTERM）或 `ros2 run` 被 Ctrl-C：正常退出，不刷栈。
        pass
    except Exception as exc:                                       # noqa: BLE001
        if rclpy.ok():
            node.get_logger().error('[band_guard] 异常退出: %r' % exc)
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                          # noqa: BLE001
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
