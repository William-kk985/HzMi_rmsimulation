#!/usr/bin/env python3
"""sc.py —— Scan Context（Kim & Kim, IROS 2018 / T-RO 2020）的 numpy 参考实现 + 地图检索库。

为什么自己实现：上游参考实现 `gisbi-kim/scancontext_tro` 在本机**拉不下来**（GitHub SSH 多次
断连，5 次重试全失败，见 docs §39.6）。Scan Context 的算法本体很短（20 ring × 60 sector 的
最大高度矩阵 + ring key 检索 + 列移位求 yaw），这里按论文逐条实现，并用合成用例自检：
  · 描述子对 yaw 的循环移位不变性；
  · ring key 的旋转不变性；
  · 合成位移/旋转能被正确恢复（见 selftest()）。

`ScanContextDB` 负责「地图侧」：给一批候选位姿 (x, y) 生成描述子并建 ring-key 索引。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree


def _rot_z(yaw: float) -> np.ndarray:
    """与 third_party/eval/eval_lib.py 的 rot_z 逐字相同（只为本文件自检，避免依赖 eval 包）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

NUM_RINGS = 20
NUM_SECTORS = 60
MAX_RADIUS = 12.0     # m（场地对角线 ~15 m，取 12 保留近处结构）
HEIGHT_MAX = 3.0


def descriptor(pts: np.ndarray, origin_xy=(0.0, 0.0), height_ref: float = 0.0,
               num_rings=NUM_RINGS, num_sectors=NUM_SECTORS, max_radius=MAX_RADIUS,
               height_max=HEIGHT_MAX) -> np.ndarray:
    """标准 Scan Context：bin 值 = 该 (ring, sector) 内点的**最大高度**（相对 height_ref）。"""
    desc = np.zeros((num_rings, num_sectors), dtype=np.float32)
    if len(pts) == 0:
        return desc
    dx = pts[:, 0] - origin_xy[0]
    dy = pts[:, 1] - origin_xy[1]
    r = np.hypot(dx, dy)
    m = (r < max_radius) & (r > 1e-6)
    h = pts[:, 2] - height_ref
    m &= (h > -0.5) & (h < height_max)
    if not m.any():
        return desc
    r, a, h = r[m], np.arctan2(dy[m], dx[m]), h[m]
    ring = np.clip((r / max_radius * num_rings).astype(np.int32), 0, num_rings - 1)
    sector = np.clip(((a + np.pi) / (2 * np.pi) * num_sectors).astype(np.int32), 0, num_sectors - 1)
    np.maximum.at(desc, (ring, sector), np.maximum(h, 0.0))
    return desc


def ring_key(desc: np.ndarray) -> np.ndarray:
    return desc.mean(axis=1)


def sc_distance(A: np.ndarray, B: np.ndarray):
    """SC 距离：A 固定，B 逐列循环移位找最优；返回 (best_dist, best_shift)。"""
    n = A.shape[1]
    best_d, best_s = np.inf, 0
    for s in range(n):
        Bs = np.roll(B, s, axis=1)
        nz = (A > 0) | (Bs > 0)
        c = int(np.count_nonzero(nz))
        if c == 0:
            continue
        d = float(np.sum((A[nz] - Bs[nz]) ** 2) / c)
        if d < best_d:
            best_d, best_s = d, s
    return best_d, best_s


class ScanContextDB:
    """地图侧描述子库：候选位姿来自「可通行区的规则网格」。"""

    def __init__(self, map_pts: np.ndarray, poses_xy: np.ndarray, ground_ref: float,
                 num_rings=NUM_RINGS, num_sectors=NUM_SECTORS):
        self.poses = poses_xy
        self.desc = np.stack([descriptor(map_pts, (x, y), ground_ref, num_rings, num_sectors)
                              for x, y in poses_xy])
        self.rk = np.stack([ring_key(d) for d in self.desc])
        self.tree = cKDTree(self.rk)
        self.ground_ref = ground_ref

    def query(self, qdesc: np.ndarray, topk=20):
        """返回 (best_idx, best_dist, best_shift, second_dist, margin_m)。"""
        d0, idx0 = self.tree.query(ring_key(qdesc), k=min(topk, len(self.poses)))
        idx0 = np.atleast_1d(idx0)
        best = (None, np.inf, 0)
        scored = []
        for i in idx0:
            d, s = sc_distance(qdesc, self.desc[i])
            scored.append((d, int(i), s))
        scored.sort()
        d_best, i_best, s_best = scored[0]
        # 次优：取与最优位置相距 >1 m 的最优分数（用于置信度 margin）
        second = np.inf
        px = self.poses[i_best]
        for d, i, s in scored[1:]:
            if np.linalg.norm(self.poses[i] - px) > 1.0:
                second = d
                break
        return int(i_best), float(d_best), int(s_best), float(second), float(second - d_best)


def shift_to_yaw(shift: int, num_sectors=NUM_SECTORS) -> float:
    """列移位 → yaw（rad）。标定见 selftest：把 query 逆时针转 +θ，最优 shift = θ/Δ。"""
    return 2.0 * math.pi * shift / num_sectors


def query_yaw_from_shift(shift: int, num_sectors=NUM_SECTORS) -> float:
    """**检索侧**：query 相对 keyframe 的 yaw（rad，map 系，逆时针 +）。

    与 `shift_to_yaw()` **差一个负号**。这不是笔误，是 selftest 的实测标定：
    自检里 query 点云 = 地图点经 Rz(θ) 转到"机器人朝向 θ"的机体系（`(pts-c) @ R`），
    恢复出的最优 shift 满足 `shift_to_yaw(s) = -θ` ⇒ query_yaw = -shift_to_yaw(s)。
    `third_party/eval/run_sc_scandb.py` 的 `yaw = -2π·shift/N` 用的是同一个号；
    §41 的 13 帧实测也是这个号（若写反，yaw 误差会是 2θ，一眼可见）。
    """
    return -shift_to_yaw(shift, num_sectors)


def selftest(verbose=True):
    """合成自检：把地图按已知 (dx,dy,yaw) 变换后当 query，检查描述子/检索能不能恢复。"""
    rng = np.random.default_rng(0)
    # 造一个「房间 + 内部矮墙」的合成地图（与真实资产同量级：12×8 m，墙高 0.4/1.0 m）
    pts = []
    for x in np.arange(-2, 10, 0.05):
        pts.append([x, -3.0, 1.0]); pts.append([x, 5.0, 1.0])
        pts.append([x, -0.5, 0.4])
    for y in np.arange(-3, 5, 0.05):
        pts.append([-2.0, y, 1.0]); pts.append([10.0, y, 1.0])
        pts.append([6.0, y, 0.4])
    pts = np.array(pts) + rng.normal(0, 0.01, (len(pts), 3))
    grid = np.array([[x, y] for x in np.arange(-1.0, 9.5, 0.5) for y in np.arange(-2.4, 4.6, 0.5)])
    db = ScanContextDB(pts, grid, ground_ref=0.0)
    ok = 0; tot = 0; errs = []
    for (cx, cy) in [(2.0, 1.0), (6.5, -1.5), (0.5, 3.0), (8.0, 2.0), (4.0, -2.0)]:
        for yaw_deg in [0, 45, 90, 179, -120]:
            th = math.radians(yaw_deg)
            R = _rot_z(th)
            # query 点云 = 地图点在以 (cx,cy,0) 为原点、旋转 -th 的坐标系下的坐标
            q = (pts - np.array([cx, cy, 0.0])) @ R          # p_q = R^T (p - c)
            qd = descriptor(q, (0, 0), 0.0)
            i, d, s, d2, margin = db.query(qd, topk=20)
            est_xy = db.poses[i]
            pos_err = float(np.linalg.norm(est_xy - np.array([cx, cy])))
            # shift → yaw：标定关系
            yaw_est = math.degrees(shift_to_yaw(s))
            # 归一化到 (-180,180] 后与真值比
            yerr = (yaw_est - (-yaw_deg) + 180) % 360 - 180
            tot += 1
            good = pos_err < 0.8 and abs(yerr) < 12
            ok += good
            errs.append((pos_err, yerr))
            if verbose:
                print(f"  c=({cx},{cy}) yaw={yaw_deg:5.1f} -> est=({est_xy[0]:.1f},{est_xy[1]:.1f}) "
                      f"poserr={pos_err:.2f}m shift={s} yaw_est={yaw_est:6.1f} yerr={yerr:6.1f} "
                      f"d={d:.4f} margin={margin:.4f} {'OK' if good else 'FAIL'}")
    if verbose:
        print(f"selftest: {ok}/{tot} within 0.8 m & 12°")
    return ok, tot


if __name__ == "__main__":
    selftest()
