#!/usr/bin/env bash
# ============================================================================
# run_robot_model_probe.sh —— 一次"隔离的无头仿真 + 机器人模型探针"跑（2026-10-07）
#
# 用法（**一次 bash 调用跑完，符合本仓的无头跑隔离约定**）：
#   tools/scripts/regress/run_robot_model_probe.sh <tag> [PROBE_ARGS...] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/regress/run_robot_model_probe.sh hzmirm --duration 20 --drive-seconds 10 -- \
#       world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=hzmirm
#
# 隔离约定（照 docs/smoke_test_runbook.md 的口径）：
#   · HOME=/tmp/gzhome-<tag>（Gazebo 要写 ~/.gazebo）
#   · 非默认 ROS_DOMAIN_ID + 专用 GAZEBO_MASTER_URI（不打扰别人的栈）
#   · unset DISPLAY（无头）
#   · 跑前/跑后都清 gzserver/gzclient（只清本 MASTER_URI 上的，避免误杀）
#   · 结束时一定补零速 /cmd_vel_chassis
# 输出：.tmp_robotslot/<tag>/ 下的 launch.log + probe.json + probe.log
# ============================================================================
set -o pipefail
# ⚠️ 不能开 set -u：/opt/ros/humble/setup.bash 自己不干净（会引用未绑定的 AMENT_TRACE_SETUP_FILES）
set +u

TAG="${1:?用法: run_robot_model_probe.sh <tag> [PROBE_ARGS...] -- [LAUNCH_ARGS...]}"
shift
PROBE_ARGS=()
LAUNCH_ARGS=()
mode=probe
for a in "$@"; do
  if [ "$a" = "--" ] && [ "$mode" = probe ]; then mode=launch; continue; fi
  if [ "$mode" = probe ]; then PROBE_ARGS+=("$a"); else LAUNCH_ARGS+=("$a"); fi
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_robotslot/$TAG"
mkdir -p "$OUT"

# 每个 tag 一套隔离环境（domain id 取 tag 的哈希，落到 30..90）
HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID=$(( 30 + HASH % 61 ))
export GAZEBO_MASTER_URI="http://127.0.0.1:$(( 11350 + HASH % 500 ))"
export HOME="/tmp/gzhome-$TAG"
mkdir -p "$HOME"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

GZ_PORT="${GAZEBO_MASTER_URI##*:}"
echo "[run] tag=$TAG domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"
echo "[run] launch args: ${LAUNCH_ARGS[*]:-<默认>}"
echo "[run] probe args : ${PROBE_ARGS[*]:-<默认>}"

cleanup() {
  # 零速兜底（车不能留在动着的状态）
  timeout 10 ros2 topic pub -r 5 -t 5 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 3
  pkill -f "gzserver.*$GZ_PORT" 2>/dev/null || true
  pkill -f "gzclient.*$GZ_PORT" 2>/dev/null || true
  [ -n "${LAUNCH_PID:-}" ] && kill -9 "$LAUNCH_PID" 2>/dev/null || true
  sleep 1
  echo "[run] 残留 gzserver: $(pgrep -fa "gzserver.*$GZ_PORT" | wc -l) 个"
}
trap cleanup EXIT

# 跑前清一次（同一 master URI / 同 domain 的残留）
pkill -f "gzserver.*$GZ_PORT" 2>/dev/null || true
pkill -f "gzclient.*$GZ_PORT" 2>/dev/null || true
sleep 1

ros2 launch rm_nav_bringup bringup_sim.launch.py nav_rviz:=False lio_rviz:=False \
    "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[run] launch pid=$LAUNCH_PID → $OUT/launch.log"

sleep 3
echo "=== [run] 关键参数回读（证明 launch 真的把槽位的参数注进去了） ==="
for kv in "/ground_segmentation sensor_height" "/ground_segmentation gravity_aligned_frame" \
          "/ground_segmentation max_dist_to_line" "/lio_tf_adapter xyz" "/pointcloud_to_laserscan max_height" \
          "/pointcloud_to_laserscan min_height"; do
  set -- $kv
  printf '  %-24s %-22s = ' "$1" "$2"
  timeout 15 ros2 param get "$1" "$2" 2>&1 | tail -1
done
echo "=== [run] linefit 上游判据统计（ground_in = linefit 自己判出的地面点数） ==="
timeout 25 ros2 topic echo --once /ground_segmentation/traversability_stats 2>/dev/null | head -3
echo "=== [run] 感知插件/节点清单 ==="
timeout 20 ros2 node list 2>/dev/null | sort | tr '\n' ' '; echo
python3 "$REPO/tools/scripts/regress/robot_model_probe.py" \
    --out "$OUT/probe.json" "${PROBE_ARGS[@]}" 2>&1 | tee "$OUT/probe.log"
RC=${PIPESTATUS[0]}
echo "[run] probe rc=$RC"

echo "=== [run] 契约检查（单一发布者/单一订阅者） ==="
for t in /cmd_vel_chassis /segmentation/obstacle /segmentation/ground /map /odom /livox/imu; do
  info=$(timeout 20 ros2 topic info -v "$t" 2>/dev/null)
  pubs=$(printf '%s' "$info" | grep -c "Publisher count: " ; printf '%s' "$info" | sed -n 's/^Publisher count: //p')
  subs=$(printf '%s' "$info" | sed -n 's/^Subscription count: //p')
  echo "  $t : publishers=${pubs:-?} subscribers=${subs:-?}"
  printf '%s' "$info" | grep -E "node name|node namespace" | paste - - | sed 's/^/      /'
done
echo "  /cmd_vel_chassis 的订阅者（应只有 gazebo 的 planar_move 插件）："
timeout 20 ros2 topic info -v /cmd_vel_chassis 2>/dev/null | grep -A2 "Subscription count" | grep -E "node name|node namespace" | paste - - | sed 's/^/      /'
echo "=== [run] launch.log 关键行（模型/插件/错误） ==="
grep -nE "\[robot\]|robot_description|spawn|error|Error|ERROR|Failed|failed|Segmentation node|livox|planar|Spawn|Activating|active|Managed nodes|lifecycle|Creating bond|costmap" \
    "$OUT/launch.log" | head -40
echo "[run] 输出目录: $OUT"
exit $RC
