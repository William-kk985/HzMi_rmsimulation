#!/usr/bin/env bash
# ============================================================================
# run_nav_goal_forensics.sh —— 「目标导航为什么不动」取证跑（2026-10-09）
#
# 用法：
#   tools/scripts/tiltmount/run_nav_goal_forensics.sh <tag> [--settle S] [--duration D] \
#       [--goal-forward H | --goal X Y] [--goal-wait S] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/tiltmount/run_nav_goal_forensics.sh n1_r11_f20 --settle 30 --duration 30 \
#       --goal-forward 2.0 --goal-wait 45 -- \
#       world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
#
# ★ 2026-10-10 新增的三个开关（探针参数直接透传本脚本，见 docs/tilted_lidar_fidelity.md §M.8）：
#   --preflight-reach VX [--preflight-time T]   发目标**之前**先用绕过 nav2 的盲推量"这个出生点
#       物理上能走多远"（正推 + 反推各 T 秒）⇒ 写进 forensics.json 的 reach
#   --reach-gate <带 reach 的 forensics.json>   目标超出实测可达区（留 --reach-margin）就**不发目标**，
#       记 goal_gate（sent=false, reason=…）⇒ 把"目标在墙后面"与"控制器不动"分开
#   --goal-yaw-only-deg D                       原地转目标（位置 = 当前位姿、朝向 = 当前 + D 度）
#       ⚠️ 本仓 controller 用的是 PositionGoalChecker（不查朝向）⇒ 这类目标会立即 SUCCEEDED
#   --dump-traces                               把真值/里程计完整时间线写进 JSON（无指令漂移只能从它读）
#
# 隔离约定（与 run_tilt_mount_probe.sh / run_robot11_mount_probe.sh 同款）：
#   HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、先探测再占用 GAZEBO_MASTER_URI、unset DISPLAY、
#   跑前/跑后只清**本 master URI 上**的 gzserver/gzclient（按 /proc/<pid>/environ 核对，
#   绝不做全机 pkill）。
# 输出：.tmp_tiltmount/<tag>/{launch.log, forensics.json, grids.npz, forensics.log, roslog/}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: run_nav_goal_forensics.sh <tag> [--settle S] ... -- [LAUNCH_ARGS...]}"
shift
SETTLE=30; DURATION=30; GOAL_WAIT=45
PROBE_ARGS=(); LAUNCH_ARGS=(); mode=probe
while [ $# -gt 0 ]; do
  case "$1" in
    --settle) SETTLE="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --goal-wait) GOAL_WAIT="$2"; shift 2;;
    --) mode=launch; shift;;
    *) if [ "$mode" = probe ]; then PROBE_ARGS+=("$1"); else LAUNCH_ARGS+=("$1"); fi; shift;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_tiltmount/$TAG"
mkdir -p "$OUT"

HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
if [ -z "${GAZEBO_MASTER_URI:-}" ]; then
  PORT=$(python3 -c "
import socket
p = 11950 + $HASH % 300
for _ in range(300):
    s = socket.socket()
    try:
        s.bind(('127.0.0.1', p)); s.close(); print(p); break
    except OSError:
        s.close(); p += 1
else:
    print(p)
")
  export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
fi
export HOME="/tmp/gzhome-$TAG"
mkdir -p "$HOME"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

echo "[run] tag=$TAG domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"
echo "[run] launch args: ${LAUNCH_ARGS[*]:-<默认>}"
echo "[run] probe args : settle=$SETTLE duration=$DURATION goal_wait=$GOAL_WAIT ${PROBE_ARGS[*]:-}"

_own_gz_pids() {
  local p
  for p in /proc/[0-9]*; do
    [ -r "$p/environ" ] || continue
    if tr '\0' '\n' < "$p/environ" 2>/dev/null | grep -qx "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"; then
      case "$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null)" in
        *gzserver*|*gzclient*) echo "${p#/proc/}";;
      esac
    fi
  done
}

cleanup() {
  timeout 10 ros2 topic pub -r 5 -t 5 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 3
  for pid in $(_own_gz_pids); do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in $(_own_gz_pids); do kill -9 "$pid" 2>/dev/null || true; done
  [ -n "${LAUNCH_PID:-}" ] && kill -9 "$LAUNCH_PID" 2>/dev/null || true
  sleep 1
  echo "[run] 残留（本 master URI 上的 gzserver/gzclient）: $( _own_gz_pids | wc -l) 个"
}
trap cleanup EXIT

for pid in $(_own_gz_pids); do kill -9 "$pid" 2>/dev/null || true; done
sleep 1

ros2 launch rm_nav_bringup bringup_sim.launch.py nav_rviz:=False lio_rviz:=False \
    "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[run] launch pid=$LAUNCH_PID → $OUT/launch.log"

sleep 3
ros2 node list 2>/dev/null | sort > "$OUT/nodes.txt"
echo "[run] nodes: $(tr '\n' ' ' < "$OUT/nodes.txt")"

python3 "$REPO/tools/scripts/tiltmount/nav_goal_forensics.py" \
    --out "$OUT/forensics.json" --grids "$OUT/grids.npz" \
    --settle "$SETTLE" --duration "$DURATION" --goal-wait "$GOAL_WAIT" \
    "${PROBE_ARGS[@]}" 2>&1 | tee "$OUT/forensics.log"
RC=${PIPESTATUS[0]}
echo "[run] forensics rc=$RC"

echo "=== [run] 运行期生效值 ===" | tee "$OUT/effective.txt"
for kv in "/local_costmap/local_costmap robot_radius" \
          "/local_costmap/local_costmap inflation_layer.inflation_radius" \
          "/global_costmap/global_costmap robot_radius" \
          "/global_costmap/global_costmap inflation_layer.inflation_radius" \
          "/controller_server FollowPath.regulated_linear_scaling_min_radius" \
          "/controller_server FollowPath.use_rotate_to_heading" \
          "/controller_server progress_checker.required_movement_radius" \
          "/ground_segmentation obstacle_near_ground_m"; do
  set -- $kv
  printf '  %-46s %-52s = ' "$1" "$2" | tee -a "$OUT/effective.txt"
  timeout 15 ros2 param get "$1" "$2" 2>&1 | tail -1 | tee -a "$OUT/effective.txt"
done
echo "[run] 输出目录: $OUT"
exit $RC
