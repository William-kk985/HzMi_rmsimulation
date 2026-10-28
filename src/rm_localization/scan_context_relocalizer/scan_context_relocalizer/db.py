#!/usr/bin/env python3
"""db.py —— Scan Context keyframe DB 的**文件格式**与检索。

格式（`PCD/<world>_sc_db.npz`，numpy 的 `.npz`，无 pickle）：

| key | shape/dtype | 含义 |
|---|---|---|
| `format_version` | int64 标量 | 1 |
| `num_rings` / `num_sectors` | int64 标量 | 描述子尺寸（本仓库 20 / 60） |
| `max_radius` / `height_max` | float64 标量 | 描述子参数（m / m） |
| `descriptors` | (K, 20, 60) float32 | 每个 keyframe 的 Scan Context（**base_link 原点、重力对齐机体系**） |
| `ring_keys` | (K, 20) float32 | descriptors.mean(axis=1)，检索索引 |
| `poses` | (K, 4, 4) float64 | `T_map_base`，keyframe 时刻的机器人位姿（map 系） |
| `level_yaws` | (K,) float64 | keyframe 时刻**重力对齐帧**（level_frame）在 map 系里的 yaw（rad）。**必要**：描述子表达在 level 帧里（世界朝向），而不是机体系；跨会话时两个 level 帧的 yaw 不同（LIO 初始朝向不同）⇒ 必须存下来才能把 SC 的 shift 换回 map 系 heading。见 `hypothesis()`。 |
| `stamps` | (K,) float64 | 建库 bag 里的点云时间戳（秒，仅追溯用） |
| `meta` | str（JSON） | 来源 bag / world / map_frame / 参数 / 建库命令 |

为什么是独立 `.npz` 而不是塞进 PCD：
  · 描述子不是点云（1 MB 量级），PCD 读点器不认识；独立文件可单独替换/回退；
  · `poses` 必须与**同一份 map 资产**同一坐标系（见 docs §4.16），所以放在 `PCD/` 旁边，
    由 `rm_nav_bringup` 的 `INSTALL_TO_SHARE PCD` 一起安装。

**描述子原点 = base_link**（不是雷达）。建库与查询都走 `q = T_level←cloud·p − t_level←base`，
两边同源 ⇒ 不需要再补 0.13 m 杆臂。这一点是 §41 的 `sensor_frame_cloud()` 的等价写法：
那里的 `A·p_odom − A·T_odom_base` 就是"摆平后相对 base_link 的点"。
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from . import scan_context as SC

FORMAT_VERSION = 1


class ScanContextDbError(RuntimeError):
    pass


def _yaw_of(T):
    return math.atan2(float(T[1, 0]), float(T[0, 0]))


class ScanContextDB:
    """只读 keyframe 库 + 检索（ring-key top-K → 60 次列移位）。"""

    def __init__(self, descriptors, poses, ring_keys, stamps, meta, level_yaws=None):
        self.descriptors = np.asarray(descriptors, dtype=np.float32)
        self.poses = np.asarray(poses, dtype=np.float64)
        self.ring_keys = np.asarray(ring_keys, dtype=np.float32)
        self.stamps = np.asarray(stamps, dtype=np.float64)
        self.meta = dict(meta or {})
        # level_yaws：见文件头格式表。老 DB 没有这一列 ⇒ 退化成 pose yaw（同会话勉强能用，
        # 跨会话会差一个"LIO 初始朝向"；node 会在 health 的 db_legacy 里标出来）。
        if level_yaws is None or len(level_yaws) == 0:
            self.level_yaws = np.array([_yaw_of(T) for T in self.poses], dtype=np.float64)
            self.meta.setdefault('legacy_no_level_yaws', True)
        else:
            self.level_yaws = np.asarray(level_yaws, dtype=np.float64)
        n = len(self.poses)
        if n == 0:
            raise ScanContextDbError('DB 里 0 个 keyframe')
        if self.descriptors.shape[0] != n or self.ring_keys.shape[0] != n:
            raise ScanContextDbError(
                'DB 字段长度不一致：descriptors=%s ring_keys=%s poses=%s'
                % (self.descriptors.shape, self.ring_keys.shape, self.poses.shape))
        self.num_rings = int(self.descriptors.shape[1])
        self.num_sectors = int(self.descriptors.shape[2])
        if self.num_sectors != SC.NUM_SECTORS or self.num_rings != SC.NUM_RINGS:
            raise ScanContextDbError(
                'DB 描述子尺寸 %dx%d 与本包 SC 常量 %dx%d 不符（换过参数就必须重建 DB）'
                % (self.num_rings, self.num_sectors, SC.NUM_RINGS, SC.NUM_SECTORS))

    # ------------------------------------------------------------------ load
    @classmethod
    def load(cls, path):
        if not os.path.isfile(path):
            raise ScanContextDbError(
                "Scan Context DB 不存在：%s\n"
                "  建库（离线，一次）：\n"
                "    ros2 run scan_context_relocalizer build_sc_db --\\\n"
                "      --bag <mapping-run bag/raw> --world <world> --out <PCD/<world>_sc_db.npz>\n"
                "  详见 docs/frame_contract_v3.md §4.16" % path)
        with np.load(path, allow_pickle=False) as z:
            for k in ('descriptors', 'ring_keys', 'poses', 'stamps'):
                if k not in z.files:
                    raise ScanContextDbError('DB %s 缺字段 %s（不是本包格式？）' % (path, k))
            meta = z['meta'].item() if 'meta' in z.files else {}
            if isinstance(meta, bytes):
                meta = meta.decode('utf-8')
            meta = json.loads(meta) if isinstance(meta, str) else dict(meta)
            ly = z['level_yaws'] if 'level_yaws' in z.files else None
            return cls(z['descriptors'], z['poses'], z['ring_keys'], z['stamps'], meta, ly)

    # ------------------------------------------------------------------ save
    def save(self, path):
        meta = dict(self.meta)
        meta.setdefault('format_version', FORMAT_VERSION)
        meta.setdefault('num_rings', self.num_rings)
        meta.setdefault('num_sectors', self.num_sectors)
        np.savez_compressed(
            path,
            format_version=np.int64(FORMAT_VERSION),
            num_rings=np.int64(self.num_rings),
            num_sectors=np.int64(self.num_sectors),
            max_radius=np.float64(meta.get('max_radius', SC.MAX_RADIUS)),
            height_max=np.float64(meta.get('height_max', SC.HEIGHT_MAX)),
            descriptors=self.descriptors.astype(np.float32),
            ring_keys=self.ring_keys.astype(np.float32),
            poses=self.poses.astype(np.float64),
            level_yaws=self.level_yaws.astype(np.float64),
            stamps=self.stamps.astype(np.float64),
            meta=json.dumps(meta, ensure_ascii=False, sort_keys=True),
        )

    # ----------------------------------------------------------------- query
    def query(self, qdesc, topk=20, min_sep=1.0):
        """检索。返回 dict：best_index / best_dist / shift / second_dist / margin /
        candidates（[(dist, index, shift), ...] 按 dist 升序）。"""
        qdesc = np.asarray(qdesc, dtype=np.float32)
        if qdesc.shape != (self.num_rings, self.num_sectors):
            raise ScanContextDbError('query 描述子形状 %s 不符' % (qdesc.shape,))
        rk = SC.ring_key(qdesc)
        n = len(self.poses)
        k = max(1, min(int(topk), n))
        # ring-key 粗筛：K 很小（通常 <400）⇒ 直接全量距离，避免 cKDTree 的建树/半径坑。
        d_rk = np.linalg.norm(self.ring_keys - rk[None, :], axis=1)
        cand = np.argpartition(d_rk, k - 1)[:k] if k < n else np.arange(n)
        cand = cand[np.argsort(d_rk[cand])]
        scored = []
        for i in cand:
            d, s = SC.sc_distance(qdesc, self.descriptors[int(i)])
            scored.append((float(d), int(i), int(s)))
        scored.sort(key=lambda t: t[0])
        best_d, best_i, best_s = scored[0]
        best_xy = self.poses[best_i][:2, 3]
        second_d = float('inf')
        for d, i, _s in scored[1:]:
            if float(np.linalg.norm(self.poses[i][:2, 3] - best_xy)) > float(min_sep):
                second_d = float(d)
                break
        return {
            'best_index': int(best_i),
            'best_dist': float(best_d),
            'shift': int(best_s),
            'second_dist': float(second_d),
            'margin': float(second_d - best_d),
            'candidates': scored,
        }

    # -------------------------------------------------------------- hypothesis
    def hypothesis(self, best_index, shift, base_yaw_in_level=0.0):
        """keyframe 位姿 + SC shift ⇒ `T_map_base` 假设（roll/pitch 取 0）。

        **yaw 的三段换算（跨会话必须这样算，否则会差一个 LIO 初始朝向）**：
          shift 给的是"query 的 level 帧相对 keyframe 的 level 帧的 yaw 差" Δ
          （描述子表达在**重力对齐的 level 帧**里 = 世界朝向，不随机体 heading 转）：
              yaw_level_q = yaw_level_k + Δ                       # DB 里逐 keyframe 存的 level_yaws
              yaw_map_q    = yaw_level_q + yaw(T_level←base_query)  # 机体系相对 level 帧的 heading
        旧实现（`yaw_kf + Δ`）只在 query 的 level 帧与 keyframe 的 level 帧 **yaw 相同**
        时才对 —— 同会话自查询成立，跨会话（LIO 从上电朝向初始化）不成立。
        `third_party/eval/run_sc_scandb.py` 的 `T_map_base_kf @ Rz(yaw)` 就是旧写法，§41 的
        13 帧因为 level 帧相同才勉强成立（误差 = 最近 keyframe 与 query 的 heading 差）。
        """
        T = np.array(self.poses[best_index], dtype=np.float64, copy=True)
        delta = SC.query_yaw_from_shift(int(shift), self.num_sectors)
        yaw_level_q = float(self.level_yaws[best_index]) + float(delta)
        yaw = yaw_level_q + float(base_yaw_in_level)
        c, s = math.cos(yaw), math.sin(yaw)
        T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return T, float(delta)
