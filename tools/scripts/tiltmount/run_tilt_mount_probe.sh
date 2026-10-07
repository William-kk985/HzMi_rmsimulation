#!/usr/bin/env bash
# ============================================================================
# run_tilt_mount_probe.sh —— 「斜装雷达：帧/射线/外参」取证跑（2026-10-09）
#
# 用法：
#   tools/scripts/tiltmount/run_tilt_mount_probe.sh <tag> [--variant NAME] \
#       [--settle S] [--duration D] [--frames N] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/tiltmount/run_tilt_mount_probe.sh tm_plugin --variant plugin \
#       --settle 25 --duration 45 -- world:=RMUL2026 mode:=slam_nav \
#       lio:=small_point_lio robot:=robot11 spin_speed:=0.0
#   tools/scripts/tiltmount/run_tilt_mount_probe.sh tm_urdf --variant urdf \
#       --settle 25 --duration 45 -- world:=RMUL2026 mode:=slam_nav \
#       lio:=small_point_lio robot:=robot11 robot11_mount:=urdf spin_speed:=0.0
#
# 隔离约定（与 tools/scripts/regress/run_robot11_mount_probe.sh 同款，见 docs/smoke_test_runbook.md）：
#   HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、先探测再占用 GAZEBO_MASTER_URI、unset DISPLAY、
#   跑前/跑后只清**本 master URI 上**的 gzserver/gzclient（按 /proc/<pid>/environ 核对，绝不做全机 pkill）。
# 输出：.tmp_tiltmount/<tag>/{launch.log, probe.json, clouds.npz, mount_evidence.txt}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: run_tilt_mount_probe.sh <tag> [--variant NAME] [--settle S] [--duration D] [--frames N] -- [LAUNCH_ARGS...]}"
shift
VARIANT=""; SETTLE=25; DURATION=45; FRAMES=3
PROBE_ARGS=(); LAUNCH_ARGS=(); mode=probe
while [ $# -gt 0 ]; do
  case "$1" in
    --variant) VARIANT="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --frames) FRAMES="$2"; shift 2;;
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
p = 11450 + $HASH % 400
for _ in range(400):
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

echo "[run] tag=$TAG variant=${VARIANT:-?} domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"
echo "[run] launch args: ${LAUNCH_ARGS[*]:-<默认>}"

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
ros2 node list 2>/dev/null | sort | tr '\n' ' ' > "$OUT/nodes.txt"
echo "[run] nodes: $(cat "$OUT/nodes.txt")"

timeout 20 gz topic -l > "$OUT/gz_topics.txt" 2>&1 || true
: > "$OUT/mount_evidence.txt"
grep -m1 -A3 "body_to_livox" "$OUT/launch.log" >> "$OUT/mount_evidence.txt" 2>/dev/null || true

python3 "$REPO/tools/scripts/tiltmount/tilt_mount_probe.py" \
    --out "$OUT/probe.json" --npz "$OUT/clouds.npz" \
    --settle "$SETTLE" --duration "$DURATION" --frames "$FRAMES" \
    --variant "$VARIANT" "${PROBE_ARGS[@]}" 2>&1 | tee "$OUT/probe.log"
RC=${PIPESTATUS[0]}
echo "[run] probe rc=$RC"

echo "=== [run] 运行期生效值 ===" | tee -a "$OUT/mount_evidence.txt"
for kv in "/ground_segmentation sensor_height" "/ground_segmentation gravity_aligned_frame" \
          "/ground_segmentation self_mask_enable" \
          "/pointcloud_to_laserscan min_height" "/pointcloud_to_laserscan max_height" \
          "/pointcloud_to_laserscan target_frame" "/local_costmap/local_costmap robot_radius"; do
  set -- $kv
  printf '  %-46s %-34s = ' "$1" "$2" | tee -a "$OUT/mount_evidence.txt"
  timeout 15 ros2 param get "$1" "$2" 2>&1 | tail -1 | tee -a "$OUT/mount_evidence.txt"
done
echo "=== [run] TF base_link→{livox_frame, imu_link} / odom→base_link ===" | tee -a "$OUT/mount_evidence.txt"
for pair in "base_link livox_frame" "base_link imu_link" "odom base_link" "odom livox_frame"; do
  set -- $pair
  printf '  %s → %-12s ' "$1" "$2" | tee -a "$OUT/mount_evidence.txt"
  timeout 12 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null \
    | grep -m1 -A4 "Translation" | tr '\n' ' ' | sed 's/  */ /g' | tee -a "$OUT/mount_evidence.txt"
  echo | tee -a "$OUT/mount_evidence.txt"
done
echo "=== [run] Gazebo 侧真身（mesh 姿态 + 传感器 scoped name） ===" | tee -a "$OUT/mount_evidence.txt"
timeout 25 gz model -m robot -i > "$OUT/gz_model_info.txt" 2>&1 || true
grep -nE 'livox|imu|<pose|nested|model name' "$OUT/gz_model_info.txt" 2>/dev/null \
    | head -22 | cut -c1-190 | tee -a "$OUT/mount_evidence.txt"
grep -m2 -E "livox_frame/scan|livox" "$OUT/gz_topics.txt" | tee -a "$OUT/mount_evidence.txt"
echo "[run] 输出目录: $OUT"
exit $RC
