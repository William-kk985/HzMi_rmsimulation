#!/usr/bin/env bash
# run_chassis_yaw_bench.sh <tag> --model <sdf> [--variant V] [--wz W] [--push-wall S] [--add-jsp]
# 单进程台架：gzserver（世界 + 一个模型 + planar_move 插件） + chassis_yaw_probe.py。
#
# 用途：量"底盘到底转不转"的**变体矩阵**（§M 的归因证据就是它产出的）。
#   --variant baseline|float|nofric|nowheelcol|spherewheel|fixsteer|fixwheel|fixall|weld|baseonly|boxwheel
#   --wheel-mu M / --all-mu M / --ur HZ / --vx V / --wz W / --spawn-z Z / --keep-visual
#   ⚠️ 需要先生成模型 SDF：xacro <模型>.xacro … | gz sdf -p /dev/stdin > model.sdf
# 输出：.tmp_tiltmount/<tag>/bench/{world.world,probe.json,gzserver.log,probe.log}
# 隔离：HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、GAZEBO_MASTER_URI 按 tag 哈希、unset DISPLAY。
# 收尾只 kill 本脚本自己的 PID（绝不全机 pkill）。
set -o pipefail
set +u
TAG="${1:?usage: run_bench.sh <tag> --model <sdf> [...]}"; shift
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_tiltmount/$TAG/bench"
mkdir -p "$OUT"
HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
PORT=$(python3 -c "
import socket
p = 11350 + $HASH % 300
for _ in range(300):
    s = socket.socket()
    try:
        s.bind(('127.0.0.1', p)); s.close(); print(p); break
    except OSError:
        s.close(); p += 1
else: print(p)")
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export HOME="/tmp/gzhome-$TAG"; mkdir -p "$HOME"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"; mkdir -p "$ROS_LOG_DIR"
# 关掉在线模型库（免得 gzserver 卡在抓取）
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1/"

MODEL=""; VARIANT=baseline; WZ=1.0; WALL=12.0; ADDJSP=0; SETTLE=8.0; VX=0.0; KEEPVIS=0; SPAWN="4.3 3.35 0.2 0.0"; WHEELMU=""; ALLMU=""
while [ $# -gt 0 ]; do
  case "$1" in
    --model) MODEL="$2"; shift 2;;
    --variant) VARIANT="$2"; shift 2;;
    --wz) WZ="$2"; shift 2;;
    --vx) VX="$2"; shift 2;;
    --push-wall) WALL="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    --add-jsp) ADDJSP=1; shift;;
    --wheel-mu) WHEELMU="--wheel-mu $2"; shift 2;;
    --all-mu) ALLMU="--all-mu $2"; shift 2;;
    --spawn-z) SPAWN="4.3 3.35 $2 0.0"; shift 2;;
    --ur) UR="--ur $2"; shift 2;;
    --keep-visual) KEEPVIS=1; shift;;
    --spawn) SPAWN="$2 $3 $4 $5"; shift 5;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done
[ -n "$MODEL" ] || { echo "need --model"; exit 2; }

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash" 2>/dev/null

WP="$REPO/src/rm_simulation/hzmi_rm_simulation/world"
export GAZEBO_MODEL_PATH="$WP:$REPO/install/robot11/share:$REPO/install/hzmi_rm_simulation/share${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

EXTRA=""
[ "$ADDJSP" = 1 ] && EXTRA="--add-jsp"
[ "$KEEPVIS" = 1 ] && EXTRA="$EXTRA --keep-visual"
python3 "$REPO/tools/scripts/tiltmount/chassis_bench_mkworld.py" --model "$MODEL" --out "$OUT/world.world" \
    --variant "$VARIANT" --spawn $SPAWN $EXTRA $WHEELMU $ALLMU $UR --strip-sensors > "$OUT/mkworld.log" 2>&1
echo "[bench] $TAG variant=$VARIANT wz=$WZ wall=${WALL}s domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"

gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_state.so \
    "$OUT/world.world" > "$OUT/gzserver.log" 2>&1 &
GZPID=$!
trap 'kill -INT $GZPID 2>/dev/null; sleep 2; kill -9 $GZPID 2>/dev/null; wait $GZPID 2>/dev/null' EXIT

# 等模型出现（最多 90 s）
for i in $(seq 1 180); do
  if grep -q "Advertise odometry on \[/odom_ground_truth\]" "$OUT/gzserver.log" 2>/dev/null; then break; fi
  sleep 0.5
done
if ! grep -q "Advertise odometry on \[/odom_ground_truth\]" "$OUT/gzserver.log" 2>/dev/null; then
  echo "[bench] ❌ 底盘插件没起来（看 $OUT/gzserver.log）"; tail -20 "$OUT/gzserver.log"; exit 4
fi
sleep 2
python3 "$REPO/tools/scripts/tiltmount/chassis_yaw_probe.py" --wz "$WZ" --vx "$VX" --push-wall "$WALL" \
    --settle "$SETTLE" --out "$OUT/probe.json" 2>&1 | tee "$OUT/probe.log"
RC=${PIPESTATUS[0]}
echo "[bench] probe rc=$RC → $OUT/probe.json"
exit $RC
