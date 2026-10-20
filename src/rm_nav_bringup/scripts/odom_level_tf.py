#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_level_tf.py —— 静态边 `odom → odom_level` 的**唯一发布者**（契约 v3 §4.11，2026-10-21）

背景（一句话）：E1 之前把"重力对齐"做成**估计器初始化补丁**（`small_point_lio` 的
`gravity_align_mode`），代价是它只存在于那一个 LIO 槽位、且把里程计输出改成"另一套世界系"。
现在改成**帧图的事**：LIO 保持原始世界系 `odom`（IMU 斜装就是斜的），由本节点发布一条
**静态、latched、唯一发布者**的边

    odom ──(本节点: R_grav + 地面 z 原点)──► odom_level ──(lio_tf_adapter)──► base_link

⇒ `odom_level` = 重力对齐（z 朝上）、**连续**（只是 `odom` 的一个常量变换）、原点在
初始化时刻的地面（z=0 ≈ 地面；robot11 的 base_link z ≈ +0.1025）。

## 数值怎么来的（与 E1 同一套数学，但**只算一次、只发一次**）

| 量 | 来源 | 公式 |
|---|---|---|
| `R_mount` = R(base_link←livox_frame) | **URDF**（launch 从这次真 spawn 的 xacro 解析，`gravity_align_rpy`） | `Rz(y)Ry(p)Rx(r)` |
| `h` = 传感器(livox)离地高度 | **URDF/几何**（launch 的 `gravity_align_h`） | `base_link 离地 + livox_z` |
| `h_imu` = 状态原点(IMU body)离地高度 | 物理量 | `h − (R_mount·t_IL)_z`（`t_IL` = `extrinsic_T`，默认 (0,0,0.05)） |
| `R_level←odom` | 取决于**输入世界是否已经重力对齐** | `tilted ⇒ R_mount`（把 body 系量到的重力方向转到 −z）；`level ⇒ I`（点云/里程计本来就在重力系，**绝不能再转一次**） |
| `t_level` = odom 原点在 odom_level 里的坐标 | 取决于输入的 z 原点与输出约定 | `input_z=sensor ⇒ (0,0,h_imu)`；`input_z=ground ⇒ (0,0,0)`；`output_z=sensor ⇒ (0,0,0)` |

发布值 = **逆向**的 `T_odom←odom_level`：
`R = R_level←odom^T`、`t = −R_level←odom^T · t_level`。

robot11 + `robot11_mount:=sensor`（`R_mount = Rx(−30°)`、`h = 0.25952717`）：
`h_imu = 0.216226`；`R = Rx(+30°)`；`t = (0, +0.108113, −0.187257)`。

## 诊断（**只读、只在日志里**，绝不改发布值）

1. `gravity_align_h_check`（等价于 E1b 的互检）：从一帧 `cloud_registered`（原始世界系）拟合
   地面 ⇒ `h_meas`（传感器到地面的垂距）与 `h` 比，阈值 `<0.03 INFO / 0.03–0.05 WARN / ≥0.05 ERROR`
   （与 `small_point_lio/src/gravity_align_ground.cpp` 同一口径）。**定基永远用 URDF 值**（确定性），
   `h_meas` 只作诊断 —— 与 E1b 的设计原则逐字一致。
2. `gravity_align_init_hybrid` 风格的 URDF↔IMU 互检：静止窗口的加速度均值 ⇒ 实测重力方向 ⇒
   `R_imu`，与 `R_mount` 的夹角 `Δ`：`<2° INFO / 2–5° WARN / ≥5° ERROR`（同一组阈值）。
   ⚠️ **与 E1 的 hybrid 档有意不同**：这里**不会**因为 Δ≥2° 而改用 IMU 值 —— 静态边的值是
   **标定事实**（URDF 是唯一所有者），实测只负责"响亮地报出来"。需要 IMU 救援时用 E1 实验臂
   （`gravity_align_mode:=hybrid` + `level_frame:=odom`，见契约 §4.11 的边界清单）。

## 不变量（本节点负责的两条）

* **唯一发布者**：全栈只有本节点往 `/tf_static` 发 `parent_frame→child_frame`（`rg` 可查）；
* **静态/latched**：只发一次，QoS = `TRANSIENT_LOCAL`（晚加入的 nav2/rviz 也能拿到）；
  因此这条边**连续**（不会跳变/中断）——除非本节点挂掉（那时 TF 树直接断开 = 响亮失败，
  `lio_tf_adapter` 会因查不到 `odom→odom_level` 而停发 `odom_level→base_link`）。

参数（全部由 launch 从 URDF/几何传进来，**禁止手填**）见 `main()` 里的 `declare_parameter`。
"""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

#: 点云字段 → numpy dtype（与 tools/scripts/tiltmount/*、.tmp_gravtest/grav_probe.py 同一张表）
_DATATYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
             5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}

#: URDF↔IMU 互检的三档阈值（度）—— 与契约 §4.3 / E1 hybrid 档**同一组**阈值
_IMU_DELTA_INFO_DEG = 2.0
_IMU_DELTA_WARN_DEG = 5.0

#: h_meas 互检的三档阈值（米）—— 与契约 §4.7 第 4 条 / gravity_align_ground.cpp 同一组
_H_INFO_M = 0.03
_H_WARN_M = 0.05


def _rot_zry_rx(rpy_rad):
    """URDF 固定关节 rpy（Rz·Ry·Rx 约定）→ 旋转矩阵。"""
    r, p, y = rpy_rad
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _rot_from_vectors(a, b):
    """最小旋转：把方向 a 转到方向 b（= E1 里的 `rot_from_vectors`，同一个公式）。"""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    s = float(np.linalg.norm(axis))
    c = float(np.dot(a, b))
    if s <= 1e-12:
        if c > 0.0:
            return np.eye(3)
        # 180°：任取一个与 a 垂直的轴
        tmp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, tmp)
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    axis = axis / s
    ang = math.atan2(s, c)
    k = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(ang) * k + (1.0 - math.cos(ang)) * (k @ k)


def _rot_angle_deg(r1, r2):
    """两个旋转之间的夹角（度）= E1 里 `rot_angle_deg` 的同一公式。"""
    c = float(np.trace(r1.T @ r2))
    return math.degrees(math.acos(max(-1.0, min(1.0, (c - 1.0) / 2.0))))


def _rpy_deg(R):
    """旋转矩阵 → URDF rpy（度），便于人眼核对。"""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _quat_from_R(R):
    """旋转矩阵 → 四元数 (x, y, z, w)（Shepperd 法，数值稳）。"""
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def _pc2_to_xyz(msg):
    """PointCloud2 → (N,3) float64（只取 x/y/z；去掉非有限值）。"""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for k in ('x', 'y', 'z'):
        f = fields[k]
        dt = _DATATYPE.get(f.datatype)
        if dt is None:
            return np.zeros((0, 3))
        cols.append(buf[:, f.offset:f.offset + np.dtype(dt).itemsize].copy()
                    .view(dt).reshape(-1).astype(np.float64))
    P = np.stack(cols, axis=1)
    return P[np.isfinite(P).all(axis=1)]


def _fit_plane(P, n_pred, d_range=(0.02, 0.60), tol=0.02, iters=200, seed=20261021,
               min_inliers=100, cos_tol_deg=20.0):
    """RANSAC + SVD 精修拟合平面（法向先验 = 重力"上"在**输入世界**里的方向）。

    与 `small_point_lio/src/gravity_align_ground.cpp::fit_ground_plane_sensor_frame` 同一套
    约束（法向先验 20°、|d| 物理带、内点 2 cm、固定种子 ⇒ 确定性），只是输入换成
    LIO 的**原始世界系**点云 + 先验方向由本节点算出来。
    返回 dict（n, d, inliers, resid_p50, resid_p95, n_pts）或 None。
    """
    if P.shape[0] < 500:
        return None
    n_pred = n_pred / np.linalg.norm(n_pred)
    cos_tol = math.cos(math.radians(cos_tol_deg))
    stride = max(1, P.shape[0] // 2000)
    S = P[::stride]
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i0, i1, i2 = rng.integers(0, S.shape[0], 3)
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue
        n = np.cross(S[i1] - S[i0], S[i2] - S[i0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = -float(n @ S[i0])
        if float(n @ n_pred) < 0.0:
            n = -n
            d = -d
        if float(n @ n_pred) < cos_tol or d < d_range[0] or d > d_range[1]:
            continue
        inl = int(np.count_nonzero(np.abs(S @ n + d) < tol))
        if best is None or inl > best[0]:
            best = (inl, n, d)
    if best is None or best[0] < min_inliers:
        return None
    _inl, n0, d0 = best
    Q = P[np.abs(P @ n0 + d0) < tol]
    if Q.shape[0] < 300:
        return None
    c = Q.mean(axis=0)
    cov = (Q - c).T @ (Q - c)
    w, v = np.linalg.eigh(cov)
    n = v[:, 0]
    d = -float(n @ c)
    if float(n @ n_pred) < 0.0:
        n = -n
        d = -d
    if d < d_range[0] or d > d_range[1]:
        return None
    res = np.abs(P @ n + d)
    inlier_mask = res < tol
    inl = int(np.count_nonzero(inlier_mask))
    if inl < 300:
        return None
    r = np.sort(res[inlier_mask])
    p50 = float(r[int(0.50 * (len(r) - 1))])
    p95 = float(r[int(0.95 * (len(r) - 1))])
    return {'n': n, 'd': d, 'inliers': inl, 'resid_p50': p50, 'resid_p95': p95,
            'n_pts': int(P.shape[0])}


class OdomLevelTf(Node):
    def __init__(self):
        super().__init__('odom_level_tf')

        # ---------------- 参数（全部来自 launch：URDF 解析值，禁手填） ----------------
        self.parent_frame = self.declare_parameter('parent_frame', 'odom').value
        self.child_frame = self.declare_parameter('child_frame', 'odom_level').value
        self.gravity_rpy_deg = [float(v) for v in
                                self.declare_parameter('gravity_rpy_deg', [0.0, 0.0, 0.0]).value]
        self.sensor_height = float(self.declare_parameter('sensor_height', 0.0).value)
        self.extrinsic_t = [float(v) for v in
                            self.declare_parameter('extrinsic_t', [0.0, 0.0, 0.05]).value]
        self.input_world = str(self.declare_parameter('input_world', 'tilted').value).strip()
        self.input_z = str(self.declare_parameter('input_z', 'sensor').value).strip()
        self.output_z = str(self.declare_parameter('output_z', 'ground').value).strip()
        # ★ 2026-10-21：z 原点的**来源**。
        #   `measured`（默认）= 从**第一帧点云**拟合地面，把"地面相对 LIO 世界原点的位置"量出来
        #       ⇒ odom_level 的 z=0 真的落在地面上。为什么必须能这么选：`odom` 的原点是
        #       **估计器自己的起始位姿**（`gravity_align_mode:=off` 时它由 LIO 内部决定，
        #       实测比 URDF 预测低 ~0.13 m —— 出生下落/初始地图窗口的暂态），URDF 不可能知道它。
        #   `urdf` = 只用 URDF 的 h（确定性优先，但地面会偏 ~0.13 m；A/B/复现用）。
        self.z_origin_source = str(
            self.declare_parameter('z_origin_source', 'measured').value).strip()
        # 诊断（只读；关掉只影响日志）
        self.check_ground = bool(self.declare_parameter('check_ground', True).value)
        self.cloud_topic = str(self.declare_parameter('cloud_topic', '/cloud_registered').value)
        self.ground_check_delay_s = float(
            self.declare_parameter('ground_check_delay_s', 6.0).value)
        self.ground_check_timeout_s = float(
            self.declare_parameter('ground_check_timeout_s', 40.0).value)
        self.check_imu = bool(self.declare_parameter('check_imu', True).value)
        self.imu_topic = str(self.declare_parameter('imu_topic', '/imu/data').value)
        self.imu_samples = int(self.declare_parameter('imu_samples', 200).value)

        # ---------------- 取值校验（选错不静默回退） ----------------
        for name, val, allowed in (('input_world', self.input_world, ('tilted', 'level')),
                                   ('input_z', self.input_z, ('sensor', 'ground')),
                                   ('output_z', self.output_z, ('ground', 'sensor'))):
            if val not in allowed:
                self.get_logger().error(
                    '参数 %s=%r 不是可用取值（%s）⇒ 本节点**不发布**任何 TF（宁可断开也不发错的边）'
                    % (name, val, ' | '.join(allowed)))
                self._valid = False
                return
        if len(self.gravity_rpy_deg) != 3:
            self.get_logger().error(
                'gravity_rpy_deg 需要 3 个数（度），收到 %d 个 ⇒ **不发布**'
                % len(self.gravity_rpy_deg))
            self._valid = False
            return
        if len(self.extrinsic_t) != 3:
            self.get_logger().error(
                'extrinsic_t 需要 3 个数（米），收到 %d 个 ⇒ **不发布**' % len(self.extrinsic_t))
            self._valid = False
            return
        self._valid = True

        # ---------------- 静态边：算一次、发一次（latched） ----------------
        self.R_mount = _rot_zry_rx([math.radians(v) for v in self.gravity_rpy_deg])
        t_IL = np.array(self.extrinsic_t, dtype=float)
        #: 状态原点（IMU body）离地高度 = h − (R_mount·t_IL)_z（= E1b 的 z_origin_state_z）
        self.h_imu = self.sensor_height - float((self.R_mount @ t_IL)[2])
        h_ok = self.sensor_height > 0.0
        if self.output_z == 'ground' and not h_ok:
            self.get_logger().error(
                'output_z=ground 但 sensor_height=%.6f m 无效（应 >0，由 launch 从 URDF 算）⇒ '
                '按 sensor 处理：odom_level 的 z=0 **不是**地面（响亮失败，不静默定基）'
                % self.sensor_height)
            self.output_z_eff = 'sensor'
        else:
            self.output_z_eff = self.output_z

        want_rot = (self.input_world == 'tilted')
        self.R_level_raw = self.R_mount if want_rot else np.eye(3)
        z_in = self.h_imu if (self.input_z == 'sensor' and h_ok) else 0.0
        self._z_in_nominal = z_in
        self.t_level = np.array([0.0, 0.0, z_in if self.output_z_eff == 'ground' else 0.0])
        self.R_pub = self.R_level_raw.T                      # R_parent←child（= odom←odom_level）
        self.t_pub = -self.R_level_raw.T @ self.t_level      # t_parent←child
        self.p_sensor_raw = t_IL                             # 传感器原点在输入世界里的坐标
        # 需要先测地面再发布（`z_origin_source=measured` 且输出 z 原点在地面）
        self._need_ground = (self.z_origin_source == 'measured'
                             and self.output_z_eff == 'ground' and self.check_ground
                             and self.sensor_height > 0.0)
        self._published = False

        # /tf_static：latched（TRANSIENT_LOCAL）+ reliable，depth=100（tf2 的静态 broadcaster 同口径）
        qos_static = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', qos_static)
        if not self._need_ground:
            self._publish_static_tf()

        # ---------------- 诊断订阅（只读） ----------------
        qos_sensor = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self._cloud_sub = None
        self._imu_sub = None
        self._cloud_done = not self.check_ground
        self._imu_done = not self.check_imu
        self._cloud_buf = None
        self._cloud_stamp = None
        self._acc = []
        # ⚠️ 诊断的"等/timed out"一律用**墙钟**（`time.monotonic()`）：
        #   use_sim_time 下 `get_clock().now()` 在 /clock 到达前是 0，之后跳到仿真时间 ⇒
        #   拿它算 elapsed 会在启动瞬间算出假的"已过 640 s"（实测踩到，互检被立刻跳过）。
        #   地面互检的"等出生落定"改成按**点云自己的 header 戳**（仿真时间、以第一帧为基准）
        #   —— 与 RTF 无关，且天然"等 LIO 初始化完"。
        self._t_wall0 = time.monotonic()
        self._first_cloud_stamp = None
        if self.check_imu:
            self._imu_sub = self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_sensor)
        if self.check_ground:
            self._cloud_sub = self.create_subscription(
                PointCloud2, self.cloud_topic, self._on_cloud, qos_sensor)
        self.timer = self.create_timer(0.5, self._tick)

    # ------------------------------------------------------------------ 发布
    def _publish_static_tf(self):
        qx, qy, qz, qw = _quat_from_R(self.R_pub)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = float(self.t_pub[0])
        msg.transform.translation.y = float(self.t_pub[1])
        msg.transform.translation.z = float(self.t_pub[2])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self._tf_static_pub.publish(TFMessage(transforms=[msg]))

        rpy_pub = _rpy_deg(self.R_pub)
        self.get_logger().info(
            '[odom_level_tf] 静态边 %s → %s（latched，一次；全栈唯一发布者）\n'
            '  输入世界=%s（R_level←odom = %s）、输入 z 原点=%s、输出 z 原点=%s\n'
            '  R_mount(base←livox, URDF rpy)=[%.6f, %.6f, %.6f]°  h(URDF)=%.9f m  '
            't_IL(extrinsic_T)=[%.6f, %.6f, %.6f] m\n'
            '  h_imu = h − (R_mount·t_IL).z = %.9f m  ⇒  t_level(odom 原点在 %s 里)='
            '[%.6f, %.6f, %.6f] m\n'
            '  发布值 T_%s←%s: xyz=[%.9f, %.9f, %.9f] m rpy=[%.6f, %.6f, %.6f]°'
            '（= 上面那个变换的**逆**）\n'
            '  ⇒ 期望：地面 z(%s)≈0、livox z≈h=%.6f m（robot11 再 ⇒ base_link z≈+0.1025 m）'
            % (self.parent_frame, self.child_frame,
               self.input_world,
               ('R_mount（把 body 系量到的重力转到 −z）' if self.input_world == 'tilted'
                else 'I（输入本来就重力对齐，**不再转一次**）'),
               self.input_z, self.output_z_eff,
               self.gravity_rpy_deg[0], self.gravity_rpy_deg[1], self.gravity_rpy_deg[2],
               self.sensor_height,
               self.extrinsic_t[0], self.extrinsic_t[1], self.extrinsic_t[2],
               self.h_imu, self.child_frame,
               self.t_level[0], self.t_level[1], self.t_level[2],
               self.parent_frame, self.child_frame,
               self.t_pub[0], self.t_pub[1], self.t_pub[2],
               rpy_pub[0], rpy_pub[1], rpy_pub[2],
               self.child_frame, self.sensor_height))

    # ------------------------------------------------------------------ IMU 互检
    def _on_imu(self, msg):
        if self._imu_done:
            return
        a = msg.linear_acceleration
        v = np.array([a.x, a.y, a.z], dtype=float)
        if not np.all(np.isfinite(v)) or np.linalg.norm(v) < 1e-3:
            return
        self._acc.append(v)
        if len(self._acc) >= max(1, self.imu_samples):
            self._imu_done = True
            self._report_imu()

    def _report_imu(self):
        mean_acc = np.mean(np.array(self._acc), axis=0)
        down_body = -mean_acc / np.linalg.norm(mean_acc)
        R_imu = _rot_from_vectors(down_body, np.array([0.0, 0.0, -1.0]))
        delta = _rot_angle_deg(self.R_mount, R_imu)
        rpy_imu = _rpy_deg(R_imu)
        head = ('gravity_align_init_hybrid(静态边诊断): R_imu rpy(deg)=[%.3f %.3f %.3f] '
                'R_mount(URDF) rpy(deg)=[%.3f %.3f %.3f] Δ(URDF,IMU)=%.3f°'
                % (rpy_imu[0], rpy_imu[1], rpy_imu[2],
                   self.gravity_rpy_deg[0], self.gravity_rpy_deg[1], self.gravity_rpy_deg[2],
                   delta))
        if delta < _IMU_DELTA_INFO_DEG:
            self.get_logger().info(
                head + ' < 2° ⇒ INFO（URDF 先验与实测一致；静态边继续用 URDF 值）')
        elif delta < _IMU_DELTA_WARN_DEG:
            self.get_logger().warn(
                head + ' ∈ [2,5)° ⇒ WARN（URDF 先验可疑：换过支架/轴写错？'
                '**静态边仍用 URDF 值**（确定性）；需要 IMU 救援 ⇒ '
                'gravity_align_mode:=hybrid + level_frame:=odom）')
        else:
            self.get_logger().error(
                head + ' >= 5° ⇒ ERROR（URDF 的 base_link→livox_frame 与实物不符）'
                '—— **静态边仍用 URDF 值**；这是本设计**有意**的差异（标定事实只归 URDF，'
                '实测只负责响亮报错；要 IMU 救援 ⇒ gravity_align_mode:=hybrid + level_frame:=odom）')
        # 释放订阅
        if self._imu_sub is not None:
            try:
                self.destroy_subscription(self._imu_sub)
            except Exception:                                    # noqa: BLE001
                pass
            self._imu_sub = None
        self._acc = []

    # ------------------------------------------------------------------ 地面互检（h_meas）
    def _on_cloud(self, msg):
        if self._cloud_done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._first_cloud_stamp is None:
            self._first_cloud_stamp = stamp
        # 从**第一帧可用点云**起等 ground_check_delay_s（仿真秒）：第一帧意味着 LIO 已经初始化
        # ⇒ 再等一会儿就避开了出生落定窗口（与 E1b 的 200 帧门同一个理由）。
        if stamp - self._first_cloud_stamp < self.ground_check_delay_s:
            return
        P = _pc2_to_xyz(msg)
        if P.shape[0] < 500:
            return
        self._cloud_buf = P
        self._cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._cloud_done = True
        self._report_ground()
        if self._cloud_sub is not None:
            try:
                self.destroy_subscription(self._cloud_sub)
            except Exception:                                    # noqa: BLE001
                pass
            self._cloud_sub = None

    def _report_ground(self):
        P = self._cloud_buf
        r = np.linalg.norm(P, axis=1)
        sel = P[(r > 0.8) & (r < 8.0)]      # 排除自击（r<0.24）与远处稀疏点
        n_pred = self.R_level_raw.T @ np.array([0.0, 0.0, 1.0])
        # |d| 物理带：地面到**状态原点(IMU)** 的距离就是 h_imu ⇒ 用它的 ±60% 当先验（hzmirm 的
        # 0.75 m 也覆盖得到）；h 无效时退回 0.02–0.60 的通用带（与 gravity_align_ground.cpp 同口径）。
        # ⚠️ 下界要放到 **0.02 m**：`odom` 原点到地面的距离是**估计器决定的**（实测 0.085 m，
        #   远小于 URDF 的 h_imu=0.216 m）—— 用 0.4·h_imu 当上界会把真地面挡在外面（实测踩到）。
        # ⚠️ 上界必须**慷慨**：`odom` 原点到地面的距离由估计器的起始位姿决定 —— 本仓实测
        #   **0.348 m**（出生那一刻的 IMU 高度 = 0.216 + 下落 0.13），而 URDF 的 h_imu 只有 0.216
        #   ⇒ 用 1.6·h_imu 当上界会把真地面挡在外面（实测踩到，互检直接"不可信"）。
        d_lo = 0.01
        d_hi = min(3.00, max(0.60, 4.0 * self.h_imu))
        fit = _fit_plane(sel, n_pred, d_range=(d_lo, d_hi))
        if fit is None:
            self.get_logger().warn(
                'gravity_align_h_check: 从 %s 拟合地面**不可信**（%d 点）⇒ 跳过互检；'
                '静态边仍用 URDF 值 h=%.6f m（确定性）'
                % (self.cloud_topic, sel.shape[0], self.sensor_height))
            return
        n, d = fit['n'], fit['d']
        # h_meas = 传感器原点到地面的垂距 = n·p_sensor + d（n 朝"上"，全部在输入世界里）
        h_meas = float(n @ self.p_sensor_raw + d)
        # 地面相对**输入世界原点**的高度（n 朝上 ⇒ 原点在地面之上为正）—— 这就是 odom_level
        # 的 z 原点该用的数（`z_origin_source=measured`）。
        floor_from_origin = float(d)
        # 把平面搬到输出帧（odom_level）：n_l = R·n、d_l = d − n_l·t_level
        n_l = self.R_level_raw @ n
        d_l = d - float(n_l @ self.t_level)
        floor_z = -d_l / n_l[2] if abs(n_l[2]) > 1e-9 else float('nan')
        tilt = math.degrees(math.acos(max(0.0, min(1.0, abs(float(n_l @ np.array([0.0, 0.0, 1.0])))))))
        delta = abs(h_meas - self.sensor_height)
        tail = ('h_urdf=%.4f m h_meas=%.4f m |Δ|=%.4f m ⇒ %s（inliers=%d tilt=%.2f° '
                'resid_p50=%.4f p95=%.4f；%s z(地面)=%+.4f m、tilt(%s)=%.2f°、点云 %d 点）'
                % (self.sensor_height, h_meas, delta, 'INFO' if delta < _H_INFO_M else
                   ('WARN' if delta < _H_WARN_M else 'ERROR'),
                   fit['inliers'], tilt, fit['resid_p50'], fit['resid_p95'],
                   self.child_frame, floor_z, self.child_frame, tilt, fit['n_pts']))
        if self._need_ground:
            self._z_in_nominal = floor_from_origin
            self.t_level = np.array([0.0, 0.0, floor_from_origin])
            self.t_pub = -self.R_level_raw.T @ self.t_level
            self._publish_static_tf()
            self.get_logger().info(
                '[odom_level_tf] z 原点来源=**measured**：地面在输入世界里的高度 d=%.6f m '
                '（= 静态边的 t_level.z；h_urdf 推出的 h_imu=%.6f m 只作对照：差 %+.4f m 来自 '
                '`odom` 原点由估计器起始位姿决定，不是标定误差）'
                % (floor_from_origin, self.h_imu, floor_from_origin - self.h_imu))
        if delta < _H_INFO_M:
            self.get_logger().info(
                'gravity_align_h_check: ' + tail +
                ' < 0.03 ⇒ INFO（URDF 与实测一致；定基仍用 URDF 值）')
        elif delta < _H_WARN_M:
            self.get_logger().warn(
                'gravity_align_h_check: ' + tail +
                ' ∈ [0.03,0.05) ⇒ WARN（URDF 与实测不符/场地不平）—— 仍用 URDF 值定基')
        else:
            self.get_logger().error(
                'gravity_align_h_check: ' + tail +
                ' >= 0.05 ⇒ ERROR（URDF 写错或出生不在平地上）—— 仍用 URDF 值定基（确定性）')

    # ------------------------------------------------------------------ 看门狗
    def _tick(self):
        if not self._imu_done and self.check_imu:
            elapsed = time.monotonic() - self._t_wall0
            if elapsed > self.ground_check_timeout_s:
                self.get_logger().warn(
                    'gravity_align_init_hybrid(静态边诊断): %.0f s 内只收到 %d/%d 帧 IMU（%s）'
                    '⇒ 跳过 URDF↔IMU 互检（静态边不受影响）'
                    % (elapsed, len(self._acc), self.imu_samples, self.imu_topic))
                self._imu_done = True
                if self._imu_sub is not None:
                    try:
                        self.destroy_subscription(self._imu_sub)
                    except Exception:                            # noqa: BLE001
                        pass
                    self._imu_sub = None
        if not self._cloud_done and self.check_ground:
            elapsed = time.monotonic() - self._t_wall0
            if elapsed > self.ground_check_timeout_s:
                self.get_logger().warn(
                    'gravity_align_h_check: %.0f s 内没拿到 %s 的可用点云 ⇒ 跳过互检；'
                    '静态边**改用 URDF 值**发布（z 原点会偏，loud never silent）'
                    % (elapsed, self.cloud_topic))
                if not self._published:
                    self._need_ground = False
                    self.t_level = np.array([0.0, 0.0, self._z_in_nominal])
                    self.t_pub = -self.R_level_raw.T @ self.t_level
                    self._publish_static_tf()
                self._cloud_done = True
                if self._cloud_sub is not None:
                    try:
                        self.destroy_subscription(self._cloud_sub)
                    except Exception:                            # noqa: BLE001
                        pass
                    self._cloud_sub = None
        if self._cloud_done and self._imu_done:
            self.timer.cancel()


def main(argv=None):
    rclpy.init(args=argv)
    node = OdomLevelTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
