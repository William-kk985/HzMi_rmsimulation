#!/usr/bin/env bash
# _batch_k5.sh —— **干净静止**下的"半径/几何"三档 A/B（近地剔除保持开 0.05；不发目标）
#   ① v1 = 外接 0.3565 + inflation 0.70/0.75（现状）
#   ② v2 = 内切 0.300 + inflation 0.60/0.65
#   ③ v4 = 0.22 + inflation 0.50/0.55（= 默认模型那组，用来分离"半径"与"感知"）
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
NG=src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml
CM=src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
cp "$NG" .tmp_tiltmount/ng.keep; cp "$CM" .tmp_tiltmount/cm.keep
trap 'cp .tmp_tiltmount/ng.keep "$NG"; cp .tmp_tiltmount/cm.keep "$CM"' EXIT
python3 - "$NG" 0.05 <<'PY'
import re,sys
p,v=sys.argv[1],sys.argv[2]; s=open(p).read()
open(p,'w').write(re.sub(r'(?m)^(\s*obstacle_near_ground_m:\s*)\S+', r'\g<1>'+v, s))
PY
for v in v1 v2 v4; do
  if [ "$v" = v4 ]; then
    python3 - "$CM" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
s=s.replace('robot_radius: 0.3565','robot_radius: 0.22')
s=s.replace('      inflation_radius: 0.70','      inflation_radius: 0.50')
s=s.replace('      inflation_radius: 0.75','      inflation_radius: 0.55')
open(p,'w').write(s)
PY
  else
    bash tools/scripts/tiltmount/_costmap_variant.sh "$v" >/dev/null
  fi
  tag="k5_geo$v"
  echo "########## $tag  $(grep -nE 'robot_radius|inflation_radius' "$CM" | tr '\n' ' ')"
  tools/scripts/regress/run_robot11_mount_probe.sh "$tag" \
      --settle 30 --duration 20 --frames 3 --grid-dump ".tmp_robotslot/$tag/local_grid.npz" -- \
      world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
      2>&1 | tail -5
  cp .tmp_tiltmount/cm.keep "$CM"
done
echo "=== 批次 k5 完成"
