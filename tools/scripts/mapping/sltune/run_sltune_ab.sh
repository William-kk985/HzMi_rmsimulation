#!/usr/bin/env bash
# =============================================================================
# slam_toolbox 建图调参 A/B：一次调用 = 起栈 → 记录 → 跑「出去-折返」回环路线 → 存位姿图 → 收尾
#
# 用法：
#   bash tools/scripts/mapping/sltune/run_sltune_ab.sh <tag> \
#        [--set loop_match_minimum_chain_size=4 ...] [--route <route.json>] \
#        [--speed 0.40] [--no-restore]
#
# 参数覆盖的做法：把**当前**的 mapper_params_online_async_sim.yaml 拷到临时文件、按 --set 改，
# 再装到 src/…/config/ 下（launch 读的是 install/…/config/ 的**符号链接** ⇒ 立刻生效、不用重编），
# 退出时（trap）**自动还原**。⇒ 不会在仓库里留下"实验用配置"，但每次实验改了哪几个键一目了然。
#
# 隔离（与既有跑法一致）：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI、unset DISPLAY。
# 为什么全部塞进一次 bash 调用：本环境每条命令一个 PID namespace，调用结束即清进程。
# =============================================================================
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
TAG="${1:?用法: run_sltune_ab.sh <tag> [--set k=v ...] [--route r.json]}"; shift
CFG_DST="$WS/src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml"
CFG_ORIG="$WS/.tmp_cache/sltune/cfg_orig_$TAG.yaml"
OUT="$WS/.tmp_cache/sltune/$TAG"
ROUTE="$WS/tools/scripts/mapping/sltune/loop_route_corridor.json"
DOMAIN="${SLTUNE_DOMAIN:-137}"; PORT="${SLTUNE_PORT:-11711}"
SPEED=0.40; TIMEOUT=420; WARMUP=10; RESTORE=1; LOOKAHEAD=0.8; ROTIN=2.2
MAPNAME="RMUC2026_sltune_$TAG"
SETS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --set) SETS+=("$2"); shift 2;;
    --route) ROUTE="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --lookahead) LOOKAHEAD="$2"; shift 2;;
    --rot-in-place-rad) ROTIN="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --map-name) MAPNAME="$2"; shift 2;;
    --no-restore) RESTORE=0; shift;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done

mkdir -p "$OUT" "/tmp/gzhome-$TAG" "$WS/.tmp_cache/sltune/roslog_$TAG"
cp -f "$CFG_DST" "$CFG_ORIG"
restore() {
  if [ "$RESTORE" = 1 ]; then
    cp -f "$CFG_ORIG" "$CFG_DST"
    echo "[run] 已还原 $CFG_DST"
  fi
}
trap restore EXIT

# ---- 装配置（在副本上按 --set 改，再覆盖过去）
python3 - "$CFG_ORIG" "$CFG_DST" "${SETS[@]:-}" <<'PY'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
sets = [a for a in sys.argv[3:] if a]
txt = open(src).read()
applied = []
for kv in sets:
    k, v = kv.split('=', 1)
    new, n = re.subn(r'(?m)^(\s*%s:\s*).*$' % re.escape(k), r'\g<1>%s' % v, txt)
    if n == 0:
        sys.exit('❌ 配置里没有这个键: %s' % k)
    txt = new
    applied.append('%s=%s' % (k, v))
open(dst, 'w').write(txt)
print('[cfg] 覆盖: %s' % (', '.join(applied) if applied else '（无，= 现状基线）'))
PY
[ $? -eq 0 ] || exit 2
diff -u "$CFG_ORIG" "$CFG_DST" > "$OUT/cfg_diff.txt" || true
cat "$OUT/cfg_diff.txt"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$WS/.tmp_cache/sltune/roslog_$TAG"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR="/tmp/mpl-$TAG"
unset DISPLAY
cd "$WS" || exit 2

LOG="$OUT/launch.log"; : > "$LOG"
ARGS=(world:=RMUC2026 mode:=mapping lio:=small_point_lio mapper:=slam_toolbox
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0
      map_name:=$MAPNAME map_autocontinue:=False)
echo "[run] tag=$TAG route=$ROUTE" | tee -a "$LOG"
echo "[run] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[run] launch pid=$LPID pgid=$PGID loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"

cleanup() {
  echo "[run] ---- cleanup ----"
  kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'slam_toolbox|small_point_lio_node|pointcloud_to_laserscan|ground_segmentation|spawn_entity|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform|ros2 launch' 2>/dev/null
  sleep 2
  pgrep -af 'gzserver|slam_toolbox|ros2 launch' || echo "[run] （无残留）"
}
trap 'cleanup; restore' EXIT

ok=0
for i in $(seq 1 75); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[run] ❌ launch 退出（见 $LOG）"; tail -25 "$LOG"; exit 1; fi
  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock'; then ok=1; break; fi
done
echo "[run] /clock 就绪=$ok（≈$((i*2)) s）"
[ "$ok" = 1 ] || { echo "[run] ❌ 没起来"; tail -40 "$LOG"; exit 1; }
echo "[run] 等链路稳定 28 s ..."; sleep 28

# ---- 运行期参数证据（证明本机 2.6.10 有哪些参数、mode/lifelong 是什么）
timeout 25 ros2 param list /slam_toolbox 2>/dev/null | sort > "$OUT/param_list.txt"
for k in mode do_loop_closing loop_match_minimum_chain_size loop_search_maximum_distance \
         loop_search_space_dimension loop_match_minimum_response_coarse minimum_travel_distance \
         minimum_travel_heading minimum_time_interval max_laser_range scan_queue_size \
         restamp_tf lifelong_iou_match; do
  printf '%s = %s\n' "$k" "$(timeout 10 ros2 param get /slam_toolbox $k 2>&1 | tail -1)"
done | tee "$OUT/params.txt"
echo "[run] lifelong 参数（应为空 ⇒ mode:=lifelong 是死参数）："
grep -i lifelong "$OUT/param_list.txt" | tee "$OUT/lifelong_params.txt" || echo "  （无）"
ros2 pkg executables slam_toolbox 2>/dev/null | tee "$OUT/executables.txt"

# ---- 记录仪
python3 -u "$WS/tools/scripts/mapping/sltune/rec_sltune.py" --out "$OUT/rec.jsonl" --dur 900 \
  >"$OUT/rec.log" 2>&1 &
RECPID=$!
sleep 4

# ---- 跑回环路线
python3 -u tools/scripts/mapping/coverage_drive.py \
  --route "$ROUTE" --out-json "$OUT/drive.json" --speed "$SPEED" --timeout "$TIMEOUT" \
  --warmup "$WARMUP" --pose-source gt --final-spin 0 --lookahead "$LOOKAHEAD" --rot-in-place-rad "$ROTIN" \
  --save-2d "$OUT/map_save" >"$OUT/drive.log" 2>&1
echo "[run] drive rc=$?"
tail -20 "$OUT/drive.log"

# ---- 显式零速 + 静置
timeout 5 ros2 topic pub -r 10 -t 15 /cmd_vel_chassis geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
sleep 5

# ---- 存位姿图（回环证据：图里的约束数）
timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$OUT/posegraph'}" > "$OUT/serialize.log" 2>&1
ls -la "$OUT"/posegraph.* >> "$OUT/serialize.log" 2>&1
timeout 20 ros2 param dump /slam_toolbox > "$OUT/param_dump.yaml" 2>/dev/null
sleep 2
kill -INT "$RECPID" 2>/dev/null
echo "[run] DONE tag=$TAG out=$OUT"
