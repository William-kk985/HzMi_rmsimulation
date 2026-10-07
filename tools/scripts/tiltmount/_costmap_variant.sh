#!/usr/bin/env bash
# 在三种候选之间切换 nav2_params_sim_robot11_costmap.yaml（仅本次 A/B 用；跑完必须切回 v1）
set -euo pipefail
REPO=/home/weicheng/HzMi_rmsimulation
F=$REPO/src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
case "${1:?variant}" in
  v1)   # 现状：外接 0.3565 + inflation 0.70/0.75
    cp "$REPO/.tmp_tiltmount/costmap_override.v1.bak" "$F";;
  v2)   # 候选 A：内切 0.300 + inflation 0.60/0.65
    python3 - "$F" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
s=s.replace('robot_radius: 0.3565','robot_radius: 0.300')
s=s.replace('      inflation_radius: 0.70','      inflation_radius: 0.60')
s=s.replace('      inflation_radius: 0.75','      inflation_radius: 0.65')
open(p,'w').write(s)
PY
    ;;
  v3)   # 候选 B：真足印多边形（碰撞 mesh 凸包；内切 0.2846 / 外接 0.3525）+ inflation 0.60/0.65
    python3 - "$F" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
fp = ("footprint: [[-0.2968, -0.1901], [-0.1901, -0.2968], [0.1901, -0.2968],\n"
      "                [0.2968, -0.1901], [0.2968, 0.1901], [0.1901, 0.2968],\n"
      "                [-0.1901, 0.2968], [-0.2968, 0.1901]]")
s=s.replace('    ros__parameters:\n      robot_radius: 0.3565\n      inflation_layer:\n        inflation_radius: 0.70',
            '    ros__parameters:\n      '+fp+'\n      inflation_layer:\n        inflation_radius: 0.60',1)
s=s.replace('    ros__parameters:\n      robot_radius: 0.3565\n      inflation_layer:\n        inflation_radius: 0.75',
            '    ros__parameters:\n      '+fp+'\n      inflation_layer:\n        inflation_radius: 0.65',1)
open(p,'w').write(s)
PY
    ;;
  *) echo "unknown $1" >&2; exit 2;;
esac
grep -n "robot_radius\|footprint\|inflation_radius" "$F"
