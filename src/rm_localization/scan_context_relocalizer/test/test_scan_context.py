import numpy as np

from scan_context_relocalizer import scan_context as SC
from scan_context_relocalizer import db as D


def test_sc_selftest():
    ok, tot = SC.selftest(verbose=False)
    assert tot == 25
    assert ok >= 24


def test_yaw_shift_sign():
    # selftest 的标定：query 的 level 帧相对 keyframe 转 +θ ⇒ shift_to_yaw = -θ
    # （见 scan_context.query_yaw_from_shift 的 docstring）
    assert abs(SC.query_yaw_from_shift(0)) < 1e-12
    assert abs(SC.query_yaw_from_shift(15) + SC.shift_to_yaw(15)) < 1e-12


def test_db_roundtrip(tmp_path):
    n = 3
    desc = np.random.rand(n, SC.NUM_RINGS, SC.NUM_SECTORS).astype(np.float32)
    poses = np.tile(np.eye(4), (n, 1, 1))
    ring = np.stack([SC.ring_key(d) for d in desc])
    stamps = np.arange(n, dtype=np.float64)
    level_yaws = np.array([0.0, 0.1, -0.2])
    db = D.ScanContextDB(desc, poses, ring, stamps, {'world': 'X'}, level_yaws=level_yaws)
    p = tmp_path / 'db.npz'
    db.save(str(p))
    db2 = D.ScanContextDB.load(str(p))
    assert np.allclose(db2.level_yaws, level_yaws)
    r = db2.query(desc[0], topk=2)
    assert r['best_index'] == 0
    T, _ = db2.hypothesis(0, r['shift'])
    assert T.shape == (4, 4)
