#!/usr/bin/env bash
# =============================================================================
# 地面分割槽位整栈 smoke（A/B 用）：**一次 bash 调用跑完一整轮**
# -----------------------------------------------------------------------------
#   bash ground_slot_smoke.sh --ground patchwork --tag pw --domain 97
#
# 只做只读观测 + 记录一袋点云；**不碰** map/、tools/、别人的参数文件。
# 无人值守（headless）口径：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、
#   DISPLAY 清空、GAZEBO_MASTER_URI 独占端口、nav_rviz:=False。
# 收尾：先发零 Twist 停车，再杀进程组（绝不留车在动）。
#
# 产物（<out-dir>/）：
#   <tag>.launch.log        整栈日志
#   <tag>.metrics.json      /scan 频率与内容、/map 格数、RTF、CPU
#   <tag>.bag/              只录 /livox/lidar/pointcloud（供离线 A/B 用同一批真帧）
#   <tag>.summary.json      上面几项的汇总 + 本次 launch 参数
# =============================================================================
# ⚠ 不能用 `set -u`：ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等）⇒ 直接退出。
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
GROUND=""; TAG=""; DOMAIN=97; OUT_DIR=""; SECS=110; WORLD=RMUC2026
LIO=small_point_lio; MAPPER=slam_toolbox; MODE=mapping; GAZ=11399
DRIVE_SECS=70

while [ $# -gt 0 ]; do
  case "$1" in
    --ground) GROUND="$2"; shift 2;;
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --seconds) SECS="$2"; shift 2;;
    --drive-seconds) DRIVE_SECS="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --lio) LIO="$2"; shift 2;;
    --mapper) MAPPER="$2"; shift 2;;
    --gazebo-master) GAZ="$2"; shift 2;;
    -h|--help) sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$GROUND" ] && [ -n "$TAG" ] || { echo "必须给 --ground 与 --tag"; exit 2; }
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_ground_ab/smoke_$TAG"
mkdir -p "$OUT_DIR" "/tmp/gzhome-gnd-$TAG"

export HOME="/tmp/gzhome-gnd-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export GAZEBO_MASTER_URI="http://127.0.0.1:$GAZ"
unset DISPLAY
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

LAUNCH_LOG="$OUT_DIR/$TAG.launch.log"
METRICS="$OUT_DIR/$TAG.metrics.json"
DRIVE_LOG="$OUT_DIR/$TAG.drive.log"
BAGDIR="$OUT_DIR/$TAG.bag"

echo "[smoke:$TAG] HOME=$HOME DOMAIN=$DOMAIN GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI ground=$GROUND"

setsid ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:="$WORLD" mode:="$MODE" lio:="$LIO" mapper:="$MAPPER" \
  ground:="$GROUND" nav_rviz:=False spin_speed:=0.0 \
  > "$LAUNCH_LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" | tr -d ' ')
echo "[smoke:$TAG] launch pid=$LPID pgid=$PGID"

cleanup() {
  echo "[smoke:$TAG] ---- cleanup ----"
  # ① 先停车：零 Twist（走 /cmd_vel → fake_vel_transform → /cmd_vel_chassis）
  timeout 10 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 &
  sleep 2
  pkill -f "ros2 topic pub.*cmd_vel" 2>/dev/null
  # ② 收尾
  pkill -INT -f "bag record" 2>/dev/null
  sleep 1
  kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 15); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -f 'ros2 launch rm_nav_bringup' 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'small_point_lio|slam_toolbox|robot_state_publisher|spawn_entity.py|complementary_filter|fake_vel_transform|pointcloud_to_laserscan|ground_segmentation_node|patchwork_ground_segmentation_node|ros2 bag record|smoke_metrics' 2>/dev/null
  sleep 2
}
trap cleanup EXIT INT TERM

# ---- 等感知链起来（最多 120 s）----
for i in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '^/segmentation/obstacle$'; then break; fi
  sleep 2
done
echo "[smoke:$TAG] /segmentation/obstacle 出现（等了 ${i} 次探测）"
sleep 8

# ---- 录点云（只录一个话题，供离线同帧 A/B）----
if [ -e "$BAGDIR" ]; then rm -rf "${BAGDIR:?}"; fi
ros2 bag record -o "$BAGDIR" /livox/lidar/pointcloud > "$OUT_DIR/$TAG.record.log" 2>&1 &
echo "[smoke:$TAG] 开始录 /livox/lidar/pointcloud"

# ---- 指标采集（后台）----
python3 "$WS/src/rm_perception/patchwork_ground_segmentation/bench/smoke_metrics.py" \
  --out "$METRICS" --seconds "$SECS" \
  --proc linefit --proc patchwork_ground_segmentation_node \
  --proc pointcloud_to_laserscan --proc slam_toolbox --proc gzserver \
  > "$OUT_DIR/$TAG.metrics.log" 2>&1 &
MPID=$!

# ---- 脚本化行驶：直行 → 停 → 左转 → 直行 → 停（同样的时序给两路，保证可比）----
{
  pub() { timeout "$2" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "$1" >/dev/null 2>&1; }
  echo "[drive] t=0 前进 0.35 m/s"
  pub '{linear: {x: 0.35}, angular: {z: 0.0}}' 14
  echo "[drive] 停 3 s"
  pub '{linear: {x: 0.0}, angular: {z: 0.0}}' 3
  echo "[drive] 原地左转 0.6 rad/s"
  pub '{linear: {x: 0.0}, angular: {z: 0.6}}' 8
  echo "[drive] 前进 0.35 m/s（转弯后）"
  pub '{linear: {x: 0.35}, angular: {z: 0.0}}' "$(( DRIVE_SECS - 25 ))"
  echo "[drive] 停车（零 Twist）"
  pub '{linear: {x: 0.0}, angular: {z: 0.0}}' 6
} > "$DRIVE_LOG" 2>&1

wait "$MPID" 2>/dev/null
sleep 2
echo "[smoke:$TAG] 采集完成 -> $METRICS"
