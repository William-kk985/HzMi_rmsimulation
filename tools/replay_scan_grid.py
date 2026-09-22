#!/usr/bin/env python3
"""把 bag 里的 /scan（或 /segmentation/obstacle 重算的高度带 scan）+ 里程计，离线重放成
cartographer 2D 概率栅格。**目的是不再靠"让用户跑一遍仿真"来试参数。**

为什么值得写（读源码得到的、与直觉相反的事实，逐条有出处）：
  1) cartographer 的 2D 插入器 `ProbabilityGridRangeDataInserter2D::Insert` →
     `CastRays`（third_party/cartographer/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc）
     · 先写所有 **命中**（returns 的末端格子），后写所有 **清除**（returns 起点→末端的射线途经格子）；
     · 同一个 Insert 周期里，格子**先写谁就只算谁**（`ProbabilityGrid::ApplyLookupTable`
       里 `if (*cell >= kUpdateMarker) return;`，注释明说 "hits priority"）。
     ⇒ 所以"擦了旧墙"的**唯一**来源 = 起点→命中点这段射线扫过的格子。
  2) `missing_data_ray_length` 只在 `local_trajectory_builder_2d.cc` 的
     "range > max_range" 分支里生效：`hit.position = origin + missing_data_ray_length/range*delta`,
     而 /scan 的 range_max=10.0 且 lua 里 max_range=12.0 ⇒ **这条分支永远不走 ⇒ 该参数在本链路是空转**。
     （十次修正把 0.5 改成 0.05，等于什么都没改。）
  3) `/map` 的取值 = 占据概率×100（msg_conversion.cpp `CreateOccupancyGridMsg`:
     `value = round((1 - color/255)*100)`，color 来自 submap_painter 的 correspondence cost），
     未观测 = -1 ⇒ 本工具的 0~30 自由 / 31~64 中间 / 65~100 占据 分桶与真实 /map 一致。

事件的记录与参数**解耦**：`--mode events` 只记 (cycle, cell) 的 hit / pass-only 事件矩阵，
之后 `--mode sweep` 可以用它秒级重算任意 (hit, miss, 子图窗口, 是否清空, 累积帧数) 下的栅格演化。

用法:
  python3 tools/replay_scan_grid.py --bag .tmp_bags/ret4 --mode events
  python3 tools/replay_scan_grid.py --bag .tmp_bags/ret4 --mode events --band 0.6   # 用高 0.6m 的高度带重算 scan
  python3 tools/replay_scan_grid.py --mode sweep --events .tmp_cache/ret4_band0.10.npz \\
      --set hit=0.68,miss=0.40 --set hit=0.85,miss=0.40 --set hit=0.85,miss=0.30

⚠️ 已知近似（不影响"哪个参数更好"的相对结论，但会改变绝对值）：
  · 射线途经格子用"等步长采样"（步长 0.4 格）代替 cartographer 的超覆盖 Bresenham
    ⇒ 只擦到角尖的格子可能漏记（偏乐观）；
  · 子图用"每 num_range_data 个插入周期清零 + 被写过才刷新发布值"近似
    （真实是 2 个活跃子图同时写、后画的赢；窗口大约是本工具的 1~2 倍）；
  · 位姿用 /odom_ground_truth（底盘真值）而不是 cartographer 内部的局部位姿估计。
"""

import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np

RES = 0.05          # 与 lua 的 resolution 一致
STEP = 0.4 * RES    # 射线采样步长
GRID_ORIGIN = (-8.0, -8.0)      # 世界栅格左下角（米）
GRID_SIZE = (700, 700)          # 世界栅格 35m x 35m
LIM = math.log(0.9 / 0.1)       # 概率被夹在 [0.1, 0.9] ⇒ log-odds 上限（probability_values.h）


# ---------------------------------------------------------------- bag 读取

def open_bag(bag):
    path = bag if bag.endswith('.db3') else sorted(glob.glob(os.path.join(bag, '*.db3')))[0]
    return sqlite3.connect(path)


def topic_ids(cur):
    return {name: tid for tid, name in cur.execute('select id, name from topics')}


def msgs(cur, tid):
    for (raw,) in cur.execute('select data from messages where topic_id=? order by id', (tid,)):
        yield bytes(raw)


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quat(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def load_poses(cur, tids, mode='gt', frame_lidar='livox_frame'):
    """→ (t[], xy[2], yaw[]) 传感器(雷达)位姿；外加 base_link->lidar 的平移。

    mode='gt'：用 /odom_ground_truth 的位姿。
      ⚠️ 实测该话题**虽然是世界坐标**（出生点 x=4.30 y=3.35），而 cartographer 的 map 系
      是**出生点相对系**（出生点 = (0,0)）——两者差一个 (4.3, 3.35) 的平移。
      拿 gt 直接当 map 系会把整张图错开 ~4.4m ⇒ 与真 /map 的格子对不上。
    mode='tf'：用 /tf 的 map→odom 与 odom→base_link 复合，天然与 /map 同系（默认）。
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    TF = get_message('tf2_msgs/msg/TFMessage')

    def series(parent, child):
        ts, xs, ys, ys_ = [], [], [], []
        for raw in msgs(cur, tids['/tf']):
            m = deserialize_message(raw, TF)
            for tr in m.transforms:
                if tr.header.frame_id == parent and tr.child_frame_id == child:
                    q = tr.transform.rotation
                    th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    ts.append(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
                    xs.append(tr.transform.translation.x); ys.append(tr.transform.translation.y)
                    ys_.append(th)
        o = np.argsort(ts)
        return np.array(ts)[o], np.array(xs)[o], np.array(ys)[o], np.array(ys_)[o]

    bl_lidar = None
    for raw in msgs(cur, tids['/tf_static']):
        m = deserialize_message(raw, TF)
        for tr in m.transforms:
            if tr.header.frame_id == 'base_link' and tr.child_frame_id == frame_lidar:
                tr_ = tr.transform.translation
                bl_lidar = np.array([tr_.x, tr_.y, tr_.z])

    if mode == 'tf':
        t_mo, x_mo, y_mo, th_mo = series('map', 'odom')
        t_ob, x_ob, y_ob, th_ob = series('odom', 'base_link')
        t = t_ob
        i = np.searchsorted(t_mo, t).clip(1, len(t_mo) - 1)
        pick = np.where(np.abs(t_mo[i] - t) < np.abs(t_mo[i - 1] - t), i, i - 1)
        dx, dy, dth = x_mo[pick], y_mo[pick], th_mo[pick]
        c, s = np.cos(dth), np.sin(dth)
        xy = np.stack([dx + c * x_ob - s * y_ob, dy + s * x_ob + c * y_ob], axis=1)
        yaw = dth + th_ob
    else:
        T = get_message('nav_msgs/msg/Odometry')
        t, xy, yaw = [], [], []
        for raw in msgs(cur, tids['/odom_ground_truth']):
            m = deserialize_message(raw, T)
            p = m.pose.pose
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            xy.append((p.position.x, p.position.y))
            R = quat_to_R(p.orientation)
            yaw.append(math.atan2(R[1, 0], R[0, 0]))
        t, xy, yaw = np.array(t), np.array(xy), np.array(yaw)
    return t, xy, yaw, (bl_lidar if bl_lidar is not None else np.zeros(3))


# ---------------------------------------------------------------- scan 读取/重建

def scan_points(msg):
    """LaserScan → livox_frame 下的 2D 点 (n,2)（不含 inf/超出 range 的 bin）。"""
    r = np.asarray(msg.ranges, dtype=np.float64)
    ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1), ok, r


def load_scans(cur, tid_scan, stride):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    T = get_message('sensor_msgs/msg/LaserScan')
    out = []
    for i, raw in enumerate(msgs(cur, tid_scan)):
        if i % stride:
            continue
        m = deserialize_message(raw, T)
        pts, ok, r = scan_points(m)
        out.append({
            't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'pts': pts, 'n_bins': len(r), 'n_fin': int(ok.sum()),
            'ang_min': m.angle_min, 'ang_inc': m.angle_increment,
            'rmin': m.range_min, 'rmax': m.range_max,
        })
    return out


def load_cloud_scans(cur, tid_cl, ang_min, ang_inc, n_bins, rmin, rmax,
                     band_min, band_max, stride, zmax_keep=None):
    """离线重算 p2l：/segmentation/obstacle(livox_frame) → 每个方位 bin 取最小 range。
    完全照 pointcloud_to_laserscan_node.cpp：z 落带内、r 在 [rmin,rmax]、每 bin 取最小 r。"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    T = get_message('sensor_msgs/msg/PointCloud2')
    out = []
    for i, raw in enumerate(msgs(cur, tid_cl)):
        if i % stride:
            continue
        m = deserialize_message(raw, T)
        p = np.frombuffer(m.data, dtype=np.float32).reshape(-1, m.point_step // 4)
        x, y, z = p[:, 0].astype(np.float64), p[:, 1].astype(np.float64), p[:, 2].astype(np.float64)
        r = np.hypot(x, y)
        th = np.arctan2(y, x)
        sel = (z >= band_min) & (z <= band_max) & (r >= rmin) & (r <= rmax)
        if zmax_keep is not None:                    # 只用于诊断：看带内点的上限
            sel &= (z <= zmax_keep)
        idx = np.floor((th - ang_min) / ang_inc).astype(np.int64)
        np.clip(idx, 0, n_bins - 1, out=idx)
        best = np.full(n_bins, np.inf)
        np.minimum.at(best, idx[sel], r[sel])
        fin = np.isfinite(best)
        ang = ang_min + np.arange(n_bins) * ang_inc
        pts = np.stack([best[fin] * np.cos(ang[fin]), best[fin] * np.sin(ang[fin])], axis=1)
        out.append({
            't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'pts': pts, 'n_bins': n_bins, 'n_fin': int(fin.sum()),
            'ang_min': ang_min, 'ang_inc': ang_inc, 'rmin': rmin, 'rmax': rmax,
        })
    return out



def load_raw_cloud_points(cur, tid_cl, zmin, zmax, stride, rmin=0.25, rmax=10.0):
    """直接用 /livox/lidar/pointcloud（3D 原始点云，livox_frame）当输入：
    按 z 带裁剪 + 丢掉 (0,0,0) 假点/太近的点，**不做 p2l 的"每方位取最小 range"降维**。
    为什么值得试：p2l 把 30000 点压成 1310 条 range，再过 cartographer 的 5cm 体素滤波
    ⇒ 每帧只剩 ~373 个不同的命中格子；而墙总共有几千格 ⇒ 每个墙格平均 9 帧才被命中一次。
    这里保留原始点数，命中格子数应当高一个量级（离线可验证）。
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    T = get_message('sensor_msgs/msg/PointCloud2')
    out = []
    for i, raw in enumerate(msgs(cur, tid_cl)):
        if i % stride:
            continue
        m = deserialize_message(raw, T)
        p = np.frombuffer(m.data, dtype=np.float32).reshape(-1, m.point_step // 4)
        x, y, z = p[:, 0].astype(np.float64), p[:, 1].astype(np.float64), p[:, 2].astype(np.float64)
        r = np.hypot(x, y)
        sel = (r >= rmin) & (r <= rmax) & (z >= zmin) & (z <= zmax)
        out.append({'t': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    'pts': np.stack([x[sel], y[sel]], 1), 'n_bins': len(p), 'n_fin': int(sel.sum())})
    return out


def voxel_filter(pts, size):
    """复刻 cartographer sensor::VoxelFilter：每个 voxel 保留一个点。"""
    if len(pts) == 0:
        return pts
    key = np.floor(pts / size).astype(np.int64)
    _, keep = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(keep)]


# ---------------------------------------------------------------- 位姿 / 射线

def pose_at(tq, xy, yaw, t):
    i = int(np.argmin(np.abs(tq - t)))
    if abs(tq[i] - t) > 0.3:
        return None
    return xy[i], yaw[i]


def world_pts(pts, pose):
    (x, y), th = pose
    c, s = math.cos(th), math.sin(th)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1],
                     y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def cells_of(xy):
    ix = np.floor((xy[:, 0] - GRID_ORIGIN[0]) / RES).astype(np.int64)
    iy = np.floor((xy[:, 1] - GRID_ORIGIN[1]) / RES).astype(np.int64)
    ok = (ix >= 0) & (ix < GRID_SIZE[0]) & (iy >= 0) & (iy < GRID_SIZE[1])
    return (iy[ok] * GRID_SIZE[0] + ix[ok]), ok


def ray_cells_vec(origin, points, n_samples=800):
    """起点→每个命中点的射线途经的全部格子（等步长采样，步长 0.4 格）。"""
    if len(points) == 0:
        return np.zeros(0, dtype=np.int64)
    span = np.linalg.norm(points - origin, axis=1)
    m = max(2, int(np.ceil(span.max() / STEP)) + 1)
    tt = np.linspace(0.0, 1.0, m)[None, :, None]
    samples = origin[None, None, :] + tt * (points - origin)[:, None, :]
    flat = samples.reshape(-1, 2)
    idx, ok = cells_of(flat)
    return np.unique(idx)


# ---------------------------------------------------------------- events

def build_events(args):
    cur = open_bag(args.bag)
    tids = topic_ids(cur)
    t_odom, xy, yaw, bl_lidar = load_poses(cur, tids, args.pose)
    scans = load_scans(cur, tids['/scan'], args.stride)

    if args.raw_cloud:
        zmin, zmax = (float(v) for v in args.raw_cloud.split(':'))
        scans = load_raw_cloud_points(cur, tids['/livox/lidar/pointcloud'], zmin, zmax, args.stride)
    elif args.band is not None:
        s0 = scans[0]
        scans = load_cloud_scans(cur, tids['/segmentation/obstacle'], s0['ang_min'], s0['ang_inc'],
                                 s0['n_bins'], s0['rmin'], s0['rmax'],
                                 args.band_min, args.band, args.stride)

    print(f'[events] scan 帧 {len(scans)}  帧内返回中位 '
          f'{int(np.median([s["n_fin"] for s in scans]))}/{scans[0]["n_bins"]}'
          f'  ({100 * np.median([s["n_fin"] / s["n_bins"] for s in scans]):.1f}%)')

    # 第一遍：收集候选格子（命中过的格子 + 真 /map 曾经占据过的格子）
    hits_per, passes_per, tkeep = [], [], []
    cand = set()
    for s in scans:
        pose = pose_at(t_odom, xy, yaw, s['t'])
        if pose is None:
            tkeep.append(False); hits_per.append(np.zeros(0, np.int64)); passes_per.append(np.zeros(0, np.int64))
            continue
        tkeep.append(True)
        pts = voxel_filter(s['pts'], RES)
        wp = world_pts(pts + np.array([bl_lidar[0], bl_lidar[1]]), pose) if len(pts) else pts
        h, _ = cells_of(wp)
        h = np.unique(h)
        o = pose[0] + np.array([bl_lidar[0], bl_lidar[1]])
        oc, _ = cells_of(o[None, :])
        p = ray_cells_vec(o, wp)
        p = np.setdiff1d(p, h, assume_unique=False)
        hits_per.append(h); passes_per.append(p)
        cand.update(h.tolist())

    real = real_map_cells(args.bag) if args.bag else set()
    cand.update(real)
    cand = np.array(sorted(cand), dtype=np.int64)
    print(f'[events] 候选格子 {len(cand)}（其中真 /map 曾占据 {len(real)}）')

    lut = np.full(GRID_SIZE[0] * GRID_SIZE[1], -1, dtype=np.int64)
    lut[cand] = np.arange(len(cand))
    n = len(cand)
    H = np.zeros((len(scans), n), dtype=np.bool_)
    P = np.zeros((len(scans), n), dtype=np.bool_)
    for c, (h, p) in enumerate(zip(hits_per, passes_per)):
        if len(h):
            H[c, lut[h]] = True
        if len(p):
            P[c, lut[p]] = True
    P &= ~H

    cy, cx = np.divmod(cand, GRID_SIZE[0])
    wxy = np.stack([cx * RES + GRID_ORIGIN[0] + RES / 2, cy * RES + GRID_ORIGIN[1] + RES / 2], axis=1)

    os.makedirs(args.cache, exist_ok=True)
    tag = ('cloud' + args.raw_cloud.replace(':', '_')) if args.raw_cloud else (
        f'band{args.band:.2f}' if args.band is not None else 'raw')
    out = os.path.join(args.cache, f'{os.path.basename(args.bag.rstrip("/"))}_{tag}.npz')
    np.savez_compressed(out, H=H, P=P, wxy=wxy.astype(np.float32), cand=cand,
                        t=np.array([s['t'] for s in scans]), nfin=np.array([s['n_fin'] for s in scans]),
                        nbin=np.array([scans[0]['n_bins']]))
    print(f'[events] 事件矩阵 H{H.shape} P{P.shape} 已写 {out}')
    print(f'[events] 每周期命中格子中位 {int(np.median(H.sum(1)))}，清除格子中位 {int(np.median(P.sum(1)))}')
    return out


def real_map_cells(bag, occ=65):
    """真 /map 的"曾占据"格子（世界格坐标打包成 int，与 cand 同一坐标系）。"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    cur = open_bag(bag)
    tid = topic_ids(cur)['/map']
    T = get_message('nav_msgs/msg/OccupancyGrid')
    ever = set()
    for raw in msgs(cur, tid):
        m = deserialize_message(raw, T)
        d = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        if not (d >= occ).any():
            continue
        o = m.info.origin.position
        iy, ix = np.nonzero(d >= occ)
        wx = o.x + (ix + 0.5) * m.info.resolution
        wy = o.y + (iy + 0.5) * m.info.resolution
        gx = np.floor((wx - GRID_ORIGIN[0]) / RES).astype(np.int64)
        gy = np.floor((wy - GRID_ORIGIN[1]) / RES).astype(np.int64)
        ok = (gx >= 0) & (gx < GRID_SIZE[0]) & (gy >= 0) & (gy < GRID_SIZE[1])
        ever.update((gy[ok] * GRID_SIZE[0] + gx[ok]).tolist())
    return ever


# ---------------------------------------------------------------- sweep

def simulate(H, P, hit, miss, num_range_data=30, insert_free=True, accumulate=1, snap_every=10):
    """返回 snapshots: (n_snap, n_cells) 的占据概率（nan = 未观测）。"""
    a = math.log(hit / (1 - hit))
    b = math.log(miss / (1 - miss))
    n_cyc, n = H.shape
    lo = np.zeros(n, dtype=np.float32)
    pub = np.full(n, np.nan, dtype=np.float32)
    snaps = []
    for c in range(n_cyc):
        if c % accumulate == 0:                       # 累积窗口的起点
            grp_h = H[c:c + accumulate].any(axis=0)
            grp_p = P[c:c + accumulate].any(axis=0) & ~grp_h
        else:
            continue
        if num_range_data and (c // accumulate) % num_range_data == 0:
            lo[:] = 0.0                                # 新子图：证据清零（旧值仍留在 pub 上）
        if len(grp_h):
            lo[grp_h] = np.clip(lo[grp_h] + a, -LIM, LIM)
            pub[grp_h] = lo[grp_h]
        if insert_free and len(grp_p):
            lo[grp_p] = np.clip(lo[grp_p] + b, -LIM, LIM)
            pub[grp_p] = lo[grp_p]
        if c % snap_every == 0:
            snaps.append(pub.copy())
    return np.array(snaps)


def map_value_arr(lo):
    """log-odds(占据) → 真 /map 的 int8 值（**精确复刻** cartographer 那一条链）。

    出处（三处串起来，缺一处就会把阈值判错）：
      · mapping/submaps.h `ProbabilityToLogOddsInteger`：
          lo_int = 1 + round((Logit(p) - Logit(0.1)) * 254 / (Logit(0.9) - Logit(0.1)))
      · mapping/2d/probability_grid.cc `DrawToSubmapTexture`：
          delta = 128 - lo_int; alpha = delta>0 ? 0 : -delta; value = delta>0 ? delta : 0;
          alpha = (value||alpha) ? alpha : 1
      · io/submap_painter.cc + msg_conversion.cpp `CreateOccupancyGridMsg`：
          画在**暗红底(0.5,0,0,1)**上（预乘 alpha，alpha=0 时红通道相加），
          obs = (value==0 && alpha==0) ? 0 : 255；map = obs==0 ? -1 : round((1-color/255)*100)
    ⇒ 实测表：P=0.1→0、0.24→30、0.5→50、0.68→58、0.80→65、0.9→**75（上限，100 永不出现）**
    ⇒ 所以 tools 里的 `>=65` 等价于 **P>=0.80**；"占据 65~100" 实际只可能是 65~75。
    """
    p = 1.0 / (1.0 + np.exp(-lo))
    lg = np.log(p / (1.0 - p))
    lomin, lomax = math.log(0.1 / 0.9), math.log(0.9 / 0.1)
    lo_int = 1 + np.round((lg - lomin) * 254.0 / (lomax - lomin))
    delta = 128.0 - lo_int
    alpha = np.where(delta > 0, 0.0, -delta)
    colorv = np.where(delta > 0, delta, 0.0)
    alpha = np.where((colorv == 0) & (alpha == 0), 1.0, alpha)   # `(value||alpha)?alpha:1`
    color = np.minimum(255.0, 128.0 * (1.0 - alpha / 255.0) + colorv)
    return np.round((1.0 - color / 255.0) * 100.0)


def metrics(snaps, occ=65, free_thr=30):
    """与 tools/analyze_slam_bag.py 第 ⑤ 段同一套指标（按真 /map 的取值口径）。"""
    v = np.where(np.isnan(snaps), -1.0, map_value_arr(np.nan_to_num(snaps)))
    occ_mask = v >= occ
    ever = occ_mask.any(axis=0)
    if ever.sum() == 0:
        return None
    kept = occ_mask[-1][ever].sum()
    n_ever = int(ever.sum())
    ret = {}
    for k in (2, 5, 10, 20):
        if len(snaps) > k:
            pairs = occ_mask[:-k][:, ever] & occ_mask[k:][:, ever]
            base = occ_mask[:-k][:, ever].sum()
            ret[k] = 100.0 * pairs.sum() / max(base, 1)
    fl = ((occ_mask[:-1] & ~occ_mask[1:])[:, ever]).sum(axis=0)
    # "曾经稳定占据过"（连续 >=5 个 /map 快照 = 5 秒）的格子，最后还在不在——
    # 这才是用户说的"墙本来是实的、后来化掉了"
    cur = np.zeros(occ_mask.shape[1], np.int32)
    best = np.zeros(occ_mask.shape[1], np.int32)
    for i in range(occ_mask.shape[0]):
        cur = np.where(occ_mask[i], cur + 1, 0)
        best = np.maximum(best, cur)
    stable = best >= 5
    return {
        'stable_n': int(stable.sum()),
        'stable_kept_pct': 100.0 * float((stable & occ_mask[-1]).sum()) / max(int(stable.sum()), 1),
        'run_med': float(np.median(best[stable])) if stable.any() else 0.0,
        'n_ever': n_ever, 'kept': int(kept), 'kept_pct': 100.0 * kept / n_ever,
        'erased_pct': 100.0 * (1 - kept / n_ever), 'retention': ret,
        'flicker_med': float(np.median(fl)), 'flicker_p95': float(np.percentile(fl, 95)),
        'flicker_ge1': 100.0 * float((fl >= 1).mean()),
        'final_free': int((v[-1] <= free_thr).sum()), 'final_mid': int(((v[-1] > free_thr) & (v[-1] < occ)).sum()),
        'final_occ': int((v[-1] >= occ).sum()),
    }


def render_pgm(fn, snaps, wxy, occ=65, free_thr=30):
    """把最后一个快照画成 nav2 风格的 pgm/yaml（0=黑=占据，254=白=自由，205=未知）。"""
    v = np.where(np.isnan(snaps[-1]), -1.0, map_value_arr(np.nan_to_num(snaps[-1])))
    x0, y0 = wxy[:, 0].min(), wxy[:, 1].min()
    W = int((wxy[:, 0].max() - x0) / RES) + 2
    Hh = int((wxy[:, 1].max() - y0) / RES) + 2
    img = np.full((Hh, W), 205, np.uint8)          # 未知
    ix = ((wxy[:, 0] - x0) / RES).astype(int)
    iy = ((wxy[:, 1] - y0) / RES).astype(int)
    img[iy, ix] = np.where(v >= occ, 0, np.where(v <= free_thr, 254, 205)).astype(np.uint8)
    with open(fn, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, Hh))
        f.write(np.flipud(img).tobytes())
    with open(fn.replace('.pgm', '.yaml'), 'w') as f:
        f.write(f'image: {os.path.basename(fn)}\nresolution: {RES}\norigin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return fn


def sweep(args):
    d = np.load(args.events)
    H, P, wxy = d['H'], d['P'], d['wxy']
    print(f'[sweep] {args.events}: 周期 {H.shape[0]} 候选格 {H.shape[1]}')
    for spec in args.set:
        kv = dict(x.split('=') for x in spec.split(','))
        hit = float(kv.get('hit', 0.68)); miss = float(kv.get('miss', 0.40))
        nrd = int(kv.get('nrd', 30)); acc = int(kv.get('acc', 1))
        free = kv.get('free', 'true') == 'true'
        snaps = simulate(H, P, hit, miss, nrd, free, acc)
        if args.pgm:
            tag = f'{args.pgm}_h{hit}_m{miss}_n{nrd}.pgm'
            render_pgm(tag, snaps, wxy)
        m = metrics(snaps)
        if m is None:
            print(f'  hit={hit} miss={miss} nrd={nrd} acc={acc} free={free}: 无占据'); continue
        r = m['retention']
        print(f'  hit={hit:<5} miss={miss:<5} nrd={nrd:<3} acc={acc} free={str(free):<5} | '
              f'曾占据 {m["n_ever"]:5d}  留存 {m["kept_pct"]:5.1f}%  擦除 {m["erased_pct"]:5.1f}%  '
              f'闪烁中位 {m["flicker_med"]:.0f} | 留存曲线 +2 {r.get(2, float("nan")):5.1f}% '
              f'+5 {r.get(5, float("nan")):5.1f}% +10 {r.get(10, float("nan")):5.1f}% '
              f'+20 {r.get(20, float("nan")):5.1f}% | 曾稳定>=5帧 {m["stable_n"]:5d} 末态仍在 {m["stable_kept_pct"]:5.1f}% '
              f'| 末态 自由 {m["final_free"]} 中 {m["final_mid"]} 占据 {m["final_occ"]}')


def real_map_label(bag, cand):
    """把真 /map 的(曾占据, 末态占据)标签对齐到 cand 的索引上。"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    cur = open_bag(bag)
    tid = topic_ids(cur)['/map']
    T = get_message('nav_msgs/msg/OccupancyGrid')
    ever, last, seen_any = set(), set(), False
    for raw in msgs(cur, tid):
        m = deserialize_message(raw, T)
        d = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        o = m.info.origin.position
        iy, ix = np.nonzero(d >= 65)
        if len(ix) == 0:
            continue
        seen_any = True
        wx = o.x + (ix + 0.5) * m.info.resolution
        wy = o.y + (iy + 0.5) * m.info.resolution
        gx = np.floor((wx - GRID_ORIGIN[0]) / RES).astype(np.int64)
        gy = np.floor((wy - GRID_ORIGIN[1]) / RES).astype(np.int64)
        ok = (gx >= 0) & (gx < GRID_SIZE[0]) & (gy >= 0) & (gy < GRID_SIZE[1])
        packed = (gy[ok] * GRID_SIZE[0] + gx[ok])
        ever.update(packed.tolist())
        last = set(packed.tolist()) if seen_any else last
    return (np.isin(cand, np.fromiter(ever, dtype=np.int64, count=len(ever))),
            np.isin(cand, np.fromiter(last, dtype=np.int64, count=len(last))))


def diag(args):
    d = np.load(args.events)
    H, P, cand = d['H'], d['P'], d['cand']
    ever, last = real_map_label(args.bag, cand)
    touches = H.sum(0) + P.sum(0)
    keep_view = touches >= args.min_touch
    hit = float(args.hit); miss = float(args.miss)
    a = math.log(hit / (1 - hit)); b = math.log(miss / (1 - miss))
    # 最长"只清不命"连续段
    run = np.zeros(H.shape[1], dtype=np.int32); best = np.zeros(H.shape[1], dtype=np.int32)
    for c in range(H.shape[0]):
        run = np.where(P[c] & ~H[c], run + 1, 0)
        best = np.maximum(best, run)
    net = a * H.sum(0) + b * P.sum(0)
    groups = [('真/map 末态仍占据(墙留下来的)', ever & last & keep_view),
              ('真/map 曾占据但被擦掉(墙化掉的)', ever & ~last & keep_view),
              ('真/map 从未占据(纯空地/噪声)', ~ever & keep_view)]
    print(f'[diag] {args.events}  周期 {H.shape[0]} 候选 {H.shape[1]}  '
          f'（只看被触碰 ≥{args.min_touch} 周期的格子，共 {keep_view.sum()}）')
    print(f'       参数 hit={hit} miss={miss} ⇒ 单票 +{a:+.3f} / {b:+.3f}   '
          f'饱和上限 {LIM:.3f}；从 0.9 掉到 0.65 需要连续 {int(math.ceil((LIM - math.log(0.65 / 0.35)) / abs(b)))} 次"只清不命"')
    print(f'  {"组":34s} {"格子":>7s} {"命中/周期":>9s} {"只清/周期":>9s} {"只清占比":>8s} '
          f'{"最长连清P95":>11s} {"净票/周期":>9s}')
    for name, m in groups:
        if m.sum() == 0:
            print(f'  {name:34s} {0:7d}'); continue
        h = H[:, m].sum(0); p = P[:, m].sum(0); t = h + p
        frac = np.where(t > 0, p / np.maximum(t, 1), np.nan)
        print(f'  {name:34s} {int(m.sum()):7d} {h.mean():9.2f} {p.mean():9.2f} '
              f'{np.nanmedian(frac):8.3f} {np.percentile(best[m], 95):11.0f} '
              f'{net[m].mean() / max(t.mean(), 1e-9):+9.3f}')


def main():
    ap = argparse.ArgumentParser(description='离线重放 /scan → cartographer 2D 概率栅格')
    ap.add_argument('--bag', default='')
    ap.add_argument('--mode', choices=['events', 'sweep', 'diag'], default='events')
    ap.add_argument('--events', default='')
    ap.add_argument('--set', action='append', default=[],
                    help='hit=0.68,miss=0.40,nrd=30,acc=1,free=true（可多次）')
    ap.add_argument('--band', type=float, default=None,
                    help='用 /segmentation/obstacle 重算 scan 的 max_height（米，livox_frame）；不给=用录制的 /scan')
    ap.add_argument('--band-min', type=float, default=-1.0)
    ap.add_argument('--raw-cloud', default='', help='直接用原始 3D 点云，格式 zmin:zmax（如 -0.15:0.2）')
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--pose', choices=['tf', 'gt'], default='tf',
                    help="位姿来源：tf=map→odom∘odom→base_link（与真 /map 同系，默认）；gt=/odom_ground_truth（世界坐标）")
    ap.add_argument('--hit', type=float, default=0.68)
    ap.add_argument('--miss', type=float, default=0.40)
    ap.add_argument('--min-touch', type=int, default=20)
    ap.add_argument('--pgm', default='', help='给每个参数组合把**末态快照**渲染成 nav2 风格 pgm/yaml（前缀）')
    ap.add_argument('--cache', default='.tmp_cache')
    args = ap.parse_args()
    if args.mode == 'events':
        build_events(args)
    elif args.mode == 'diag':
        diag(args)
    else:
        if not args.set:
            args.set = ['hit=0.68,miss=0.40']
        sweep(args)


if __name__ == '__main__':
    sys.exit(main())
