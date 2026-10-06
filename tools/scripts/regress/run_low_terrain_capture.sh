#!/usr/bin/env bash
# =============================================================================
# run_low_terrain_capture.sh —— 一次 bash 调用跑完「把车开到坡脚 + 录一袋点云 + 只读采样」
# -----------------------------------------------------------------------------
# 为什么整个塞进一次 bash 调用：本环境每条 bash 命令跑在自己的 PID namespace
#   （bwrap --unshare-pid --die-with-parent），调用一结束命名空间里的所有进程（含 gzserver）
#   被内核清掉 ⇒ 起栈 / 开车 / 录包 / 采样必须在同一条命令里。
#
# 隔离（与本仓库既有跑法一致）：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、
#   独立 GAZEBO_MASTER_URI 端口、unset DISPLAY、nav_rviz:=False、收尾发零 Twist。
# 残留守卫：开跑前若发现 gzserver 或端口被占 ⇒ 先杀干净/直接退出，绝不复用别人的栈。
#
# 用法：
#   bash tools/scripts/regress/run_low_terrain_capture.sh --tag lt1 --domain 161 --port 11861
#
# 产物（<out-dir>/，默认 .tmp_lowterrain/capture_<tag>/）：
#   <tag>.launch.log      整栈日志
#   <tag>.bag/            录制（点云 + TF + 感知输出；供 low_terrain_ab.py replay 复用**同一批帧**）
#   <tag>.live.json       live 模式的逐帧指标（low_terrain_ab.py live）
#   <tag>.drive.log       航点驾驶日志
# =============================================================================
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; DOMAIN=161; PORT=11861; WORLD=RMUC2026; GROUND=patchwork; OUT_DIR=""
SECONDS_LIVE=75; SECONDS_LAUNCH_WAIT=180; DRIVE_SECS=110

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --ground) GROUND="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --live-seconds) SECONDS_LIVE="$2"; shift 2;;
    --drive-seconds) DRIVE_SECS="$2"; shift 2;;
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_lowterrain/capture_$TAG"
mkdir -p "$OUT_DIR" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"
OUT="$OUT_DIR"; LOG="$OUT/$TAG.launch.log"; BAG="$OUT/$TAG.bag"

export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_LOG_DIR="$OUT/roslog"; mkdir -p "$ROS_LOG_DIR"
export MPLCONFIGDIR="/tmp/mpl-$TAG"
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{time}] [{severity}] [{name}]: {message}'
unset DISPLAY
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || exit 2

echo "[cap:$TAG] ==== 残留检查 $(date '+%F %T') ===="
LEFT=$(pgrep -a -x gzserver || true)
if [ -n "$LEFT" ]; then
  echo "[cap:$TAG] ⚠️ 发现残留 gzserver（会抢 GAZEBO_MASTER_URI）："; echo "$LEFT"
  pkill -9 -x gzserver 2>/dev/null || true; sleep 2
fi
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  echo "[cap:$TAG] ❌ 端口 $PORT 被占用"; ss -ltnp 2>/dev/null | grep ":$PORT "; exit 3
fi
echo "[cap:$TAG] 残留检查通过 loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"

PROCS=()
cleanup() {
  echo "[cap:$TAG] ---- cleanup ----"
  timeout 10 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  sleep 1
  pkill -INT -f "bag record" 2>/dev/null || true
  [ -n "${PGID:-}" ] && kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do [ -n "${LPID:-}" ] && kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  [ -n "${PGID:-}" ] && kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -f 'ros2 launch rm_nav_bringup' 2>/dev/null || true
  pkill -9 -x gzserver 2>/dev/null || true; pkill -9 -x gzclient 2>/dev/null || true
  pkill -9 -f 'small_point_lio|slam_toolbox|spawn_entity|robot_state_publisher|complementary_filter|fake_vel_transform|pointcloud_to_laserscan|ground_segmentation_node|patchwork_ground_segmentation_node|low_terrain_ab|low_terrain_drive|ros2 bag record' 2>/dev/null || true
  sleep 2
  echo "[cap:$TAG] 残留："; pgrep -a -x gzserver || echo "  （无 gzserver）"
}
trap cleanup EXIT INT TERM

: > "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:="$WORLD" mode:=mapping lio:=small_point_lio mapper:=slam_toolbox \
  ground:="$GROUND" spin_speed:=0.0 nav_rviz:=False lio_rviz:=False \
  >>"$LOG" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[cap:$TAG] launch pid=$LPID pgid=$PGID ground=$GROUND"

for i in $(seq 1 $((SECONDS_LAUNCH_WAIT/2))); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[cap:$TAG] ❌ launch 退出"; tail -30 "$LOG"; exit 1; fi
  if ros2 topic list 2>/dev/null | grep -qx '/segmentation/step_edge'; then break; fi
done
echo "[cap:$TAG] 感知链就绪（等了 ≈$((i*2)) s）"
sleep 6

# ---- 录包：点云 + TF + 感知输出（供离线 replay 与事后核对）----
if [ -e "$BAG" ]; then rm -rf "${BAG:?}"; fi
ros2 bag record -o "$BAG" \
  /livox/lidar/pointcloud /tf /tf_static /scan \
  /segmentation/ground /segmentation/obstacle /segmentation/step_edge \
  /ground_segmentation/segmentation_time_ms /ground_segmentation/traversability_stats \
  > "$OUT/$TAG.record.log" 2>&1 &
echo "[cap:$TAG] 开始录包 → $BAG"

# ---- 只读采样（live：直接量**跑着的**这条链）+ 航点驾驶 ----
python3 -u "$WS/tools/scripts/regress/low_terrain_ab.py" live \
  --world "$WORLD" --domain "$DOMAIN" --seconds "$SECONDS_LIVE" --out "$OUT/$TAG.live.json" \
  > "$OUT/$TAG.live.log" 2>&1 &
LIVE=$!
python3 -u "$WS/tools/scripts/regress/low_terrain_ab.py" drive \
  --world "$WORLD" --domain "$DOMAIN" --speed 0.28 --drive-seconds "$DRIVE_SECS" \
  --spin-seconds 12 --spin-rate 0.6 > "$OUT/$TAG.drive.log" 2>&1 &
DRV=$!
wait $DRV || true
echo "[cap:$TAG] 驾驶结束："; tail -6 "$OUT/$TAG.drive.log" || true
wait $LIVE || true
echo "[cap:$TAG] 采样结束："; tail -6 "$OUT/$TAG.live.log" || true

echo "[cap:$TAG] DONE  bag=$BAG live=$OUT/$TAG.live.json"
