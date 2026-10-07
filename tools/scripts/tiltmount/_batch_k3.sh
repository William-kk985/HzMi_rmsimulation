#!/usr/bin/env bash
# =============================================================================
# _batch_k3.sh —— §K 的第 3 批：近地剔除 A/B（带栅格落盘 + 目标）与代价图几何两个候选
# （2026-10-09；本主题内部批次脚本）
#
#   ① 近地剔除 关 / 开：`tools/scripts/regress/run_robot11_mount_probe.sh`（**带 `--grid-dump`**
#      与 `--goal-forward`，这样才能拿到"车半径圆 / 径向剖面 / 车那格 / 目标后位移"）。
#   ② 代价图几何候选：现状 0.3565/0.70 ・ 内切 0.300/0.60 ・ 真足印多边形/0.60
#      （都在近地剔除**开**的前提下跑，才是"修完感知之后再谈几何"）。
# 每跑结束还原两份被 A/B 的配置（sha256 校验）。
# =============================================================================
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
NG=src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml
CM=src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
cp "$NG" .tmp_tiltmount/ng.keep; cp "$CM" .tmp_tiltmount/cm.keep
sha256sum "$NG" "$CM" > .tmp_tiltmount/batch_k3.inputs.sha256
restore() { cp .tmp_tiltmount/ng.keep "$NG"; cp .tmp_tiltmount/cm.keep "$CM"; }
trap restore EXIT

setng() { python3 - "$NG" "$1" <<'PY'
import re, sys
p, v = sys.argv[1], sys.argv[2]
s = open(p).read()
s = re.sub(r'(?m)^(\s*obstacle_near_ground_m:\s*)\S+', r'\g<1>' + v, s)
open(p, 'w').write(s)
PY
grep -E "^ *obstacle_near_ground_m:" "$NG"; }

probe() {  # probe <tag> <goal_forward> <extra probe args...>
  local tag="$1"; local gf="$2"; shift 2
  tools/scripts/regress/run_robot11_mount_probe.sh "$tag" "$@" \
      --settle 25 --duration 35 --frames 4 --goal-forward "$gf" --goal-wait 30 \
      --grid-dump ".tmp_robotslot/$tag/local_grid.npz" -- \
      world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
      2>&1 | tail -22
  restore
}

echo "########## ① 近地剔除 关（0.0）"
setng 0.0
probe k3_ngoff_geo1 1.0
echo "########## ② 近地剔除 开（0.05）+ 现状几何 0.3565/0.70"
setng 0.05
bash tools/scripts/tiltmount/_costmap_variant.sh v1 >/dev/null
probe k3_ngon_geo1 1.0
echo "########## ③ 近地剔除 开 + 内切 0.300/0.60"
bash tools/scripts/tiltmount/_costmap_variant.sh v2
probe k3_ngon_geo2 1.0
echo "########## ④ 近地剔除 开 + 真足印多边形/0.60"
bash tools/scripts/tiltmount/_costmap_variant.sh v3
probe k3_ngon_geo3 1.0
echo "=== 批次 k3 完成"
sha256sum -c .tmp_tiltmount/batch_k3.inputs.sha256
