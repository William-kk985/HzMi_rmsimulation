#!/usr/bin/env bash
# ============================================================================
# run_costmap_input_dump.sh —— 起一次隔离无头栈 + 采「近场归因」输入（2026-10-09）
#
# 为什么整个塞进**一次** bash 调用：本环境每条 bash 命令跑在自己的 PID namespace
#   （bwrap --unshare-pid --die-with-parent）⇒ 调用一结束，命名空间里的进程（含 gzserver）
#   全被清掉。所以"起栈 / 采样 / 收尾"必须在同一条命令里。
#
# 用法：
#   tools/scripts/tiltmount/run_costmap_input_dump.sh <tag> [--settle S] [--duration D]
#       [--frames N] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/tiltmount/run_costmap_input_dump.sh p5_dump_plugin --settle 25 --duration 20 \
#       --frames 6 -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
#       spin_speed:=0.0 gui:=False
#
# 隔离约定：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、专用 GAZEBO_MASTER_URI、
#   unset DISPLAY、收尾只 kill 本调用自己的 PID（绝不做全机 pkill）。
# 输出：.tmp_tiltmount/<tag>/{launch.log, dump/meta.json, dump/{raw,ground,obstacle,scan}_<i>.csv}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: run_costmap_input_dump.sh <tag> [--settle S] [--duration D] [--frames N] -- [LAUNCH_ARGS...]}"
shift
SETTLE=25; DURATION=20; FRAMES=6
DUMP_ARGS=(); LAUNCH_ARGS=(); mode=probe
while [ $# -gt 0 ]; do
  case "$1" in
    --settle) SETTLE="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --frames) FRAMES="$2"; shift 2;;
    --) mode=launch; shift;;
    *) if [ "$mode" = probe ]; then DUMP_ARGS+=("$1"); else LAUNCH_ARGS+=("$1"); fi; shift;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_tiltmount/$TAG"
mkdir -p "$OUT"
HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:$(( 11500 + HASH % 400 ))}"
export HOME="/tmp/gzhome-$TAG"; mkdir -p "$HOME"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"; mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
echo "[run] tag=$TAG domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"

LAUNCH_PID=""
cleanup() {
  timeout 8 ros2 topic pub -r 5 -t 3 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 4
  [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

ros2 launch rm_nav_bringup bringup_sim.launch.py nav_rviz:=False lio_rviz:=False \
    "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[run] launch pid=$LAUNCH_PID → $OUT/launch.log"
sleep 3
ros2 node list 2>/dev/null | sort | tr '\n' ' ' > "$OUT/nodes.txt"
echo "[run] nodes: $(cat "$OUT/nodes.txt")"

python3 "$REPO/tools/scripts/tiltmount/costmap_input_dump.py" \
    --out-dir "$OUT/dump" --settle "$SETTLE" --duration "$DURATION" --frames "$FRAMES" \
    --use-sim-time 1 \
    "${DUMP_ARGS[@]}" 2>&1 | tee "$OUT/dump.log"
RC=${PIPESTATUS[0]}
echo "[run] dump rc=$RC"

for kv in "/ground_segmentation obstacle_near_ground_m" \
          "/ground_segmentation self_mask_enable" \
          "/pointcloud_to_laserscan min_height" "/pointcloud_to_laserscan max_height" \
          "/local_costmap/local_costmap robot_radius" \
          "/local_costmap/local_costmap footprint" \
          "/local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height"; do
  set -- $kv
  printf '  %-58s %-46s = ' "$1" "$2"
  timeout 12 ros2 param get "$1" "$2" 2>&1 | tail -1
done | tee "$OUT/params.txt"
echo "[run] 输出目录: $OUT"
exit $RC
