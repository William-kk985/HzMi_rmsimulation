#!/usr/bin/env bash
# =============================================================================
# 无头跑既有 P0 回归（tools/scripts/regress/nav_smoke_regression.py）：
#   一次 bash 调用 = 起栈（可选槽位）→ 等链路 → 跑回归 → 收尾（发零速 + 杀干净）。
#
# 为什么必须"全部塞进一次 bash 调用"：本环境每条命令跑在自己的 PID namespace 里，
#   调用一结束命名空间里的所有进程（含 gzserver）都被清掉 ⇒ 起栈/测量/收尾必须同一条命令。
# 隔离与 run_gicp_nav_ab.sh 一致：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、
#   独立 GAZEBO_MASTER_URI、unset DISPLAY、nav_rviz:=False。
#
# 用法：
#   bash tools/scripts/localization/run_nav_smoke_regress.sh --tag regress_gicp --domain 151 --port 11821 \
#        --localization gicp --goal -1.0 2.0
# =============================================================================
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# 收尾只杀「本 run 自己的」进程：进程组 + /proc/<pid>/environ 的 GAZEBO_MASTER_URI 核对（见该文件）
. "$WS/tools/scripts/lib/scoped_cleanup.sh"
TAG="regress"; DOMAIN=151; PORT=11821; LOC="gicp"; NAV="rpp"; PLANNER="navfn"
LIO="small_point_lio"; WORLD="RMUC2026"; GOAL="-1.0 2.0"; SPIN="0.0"; YAW="auto"; SETTLE="5"
OUT_ROOT="$WS/.tmp_gicp_ab/regress"
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --localization) LOC="$2"; shift 2;;
    --nav) NAV="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --lio) LIO="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --goal) GOAL="$2 $3"; shift 3;;
    --spin-speed) SPIN="$2"; shift 2;;
    --yaw) YAW="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
OUT="$OUT_ROOT/$TAG"; rm -rf "$OUT"; mkdir -p "$OUT" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export HOME="/tmp/gzhome-$TAG" ROS_DOMAIN_ID="$DOMAIN" ROS_LOG_DIR="$OUT/roslog"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT" RCUTILS_COLORIZED_OUTPUT=0 MPLCONFIGDIR="/tmp/mpl-$TAG"
mkdir -p "$ROS_LOG_DIR"; unset DISPLAY; cd "$WS" || exit 2

# 作用域：只认 `/proc/<pid>/environ` 里 GAZEBO_MASTER_URI == 本 run 的 gzserver/gzclient（别人的不动）
LEFT=$(sc_own_gz_pids); [ -n "$LEFT" ] && { echo "[regress] 残留 gzserver（本 master URI）：$LEFT"; sc_kill_own_gz 9; sleep 2; }
command -v ss >/dev/null && ss -ltn 2>/dev/null | grep -q ":$PORT " && { echo "[regress] ❌ 端口 $PORT 被占"; exit 3; }
echo "[regress] tag=$TAG world=$WORLD lio=$LIO localization=$LOC nav=$NAV planner=$PLANNER goal=($GOAL) spin=$SPIN"

LOG="$OUT/launch.log"; : > "$LOG"
ARGS=(world:=$WORLD mode:=nav lio:=$LIO localization:=$LOC nav:=$NAV planner:=$PLANNER
      nav_rviz:=False lio_rviz:=False spin_speed:=$SPIN)
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
cleanup() {
  echo "[regress] ---- cleanup ----"
  timeout 5 ros2 topic pub -r 10 -t 3 /cmd_vel_chassis geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${PGID:-}" ] && kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  [ -n "${PGID:-}" ] && kill -9 -- "-$PGID" 2>/dev/null
  # 作用域：名字只用来挑候选，必须过归属判定（本进程组 / 本 master URI / 本脚本后代）才杀
  sc_kill_own_gz 9
  sc_kill_own_pattern 9 'gicp_registration_node|small_point_lio|controller_server|planner_server|bt_navigator|map_server'
  sc_kill_own_ros2_launch 9
  sleep 2; echo "[regress] 残留（本 master URI/进程组）：$(sc_own_leftovers_desc 'gzserver|gzclient|ros2 launch')"
}
trap cleanup EXIT
for i in $(seq 1 120); do
  sleep 2
  kill -0 "$LPID" 2>/dev/null || { echo "[regress] ❌ launch 退出"; tail -20 "$LOG"; exit 1; }
  timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock' && break
done
echo "[regress] /clock 就绪（≈$((i*2)) s），跑回归…"
python3 -u "$WS/tools/scripts/regress/nav_smoke_regression.py" --localization "$LOC" \
  --goal $GOAL --yaw "$YAW" --settle "$SETTLE" --outdir "$OUT/snap" 2>&1 | tee "$OUT/regress.log"
RC=${PIPESTATUS[0]}
echo "[regress] 回归退出码=$RC（0=PASS / 1=FAIL / 3=目标不可用）"
exit "$RC"
