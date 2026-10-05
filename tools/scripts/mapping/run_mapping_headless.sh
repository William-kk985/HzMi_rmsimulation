#!/usr/bin/env bash
# =============================================================================
# 无头建图一条龙：launch(mode:=mapping) → 脚本化覆盖路线 → 存 2D/3D 图 → 体检
#
# 为什么必须"全部塞在一次 bash 调用里"（本仓库环境的硬约束）：
#   每条 bash 命令跑在自己的 PID namespace（bwrap --unshare-pid --die-with-parent）里，
#   调用一结束，命名空间里的所有进程（含 gzserver）被内核清掉 ⇒ 启动、驱动、落盘
#   必须是同一条命令。见 docs/smoke_test_runbook.md 与 .tmp_cache/rmuc2026/run_mapping.sh。
#
# 隔离（与既有跑法一致）：HOME=/tmp/gzhome-<tag>（沙箱里 $HOME 只读，gzserver 建
#   ~/.gazebo 会 SIGABRT）、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI、unset DISPLAY。
#
# 用法（例：small_point_lio 建 RMUC2026 的先验图）：
#   bash tools/scripts/mapping/run_mapping_headless.sh \
#     --lio small_point_lio --tag spl --domain 93 --port 11503 \
#     --map-yaml src/rm_nav_bringup/map/RMUC2026.yaml \
#     --save-2d src/rm_nav_bringup/map/RMUC2026_spl \
#     --final-pcd src/rm_nav_bringup/PCD/RMUC2026_spl.pcd
#
#   # A/B：同一个 world 换 lio:=fastlio（**不要**给 --save-3d-raw 指到 PCD/<world>.pcd，
#   # 否则会覆盖 STL 合成的那份资产；本脚本默认把 raw 落到 .tmp_cache 下）
#   bash tools/scripts/mapping/run_mapping_headless.sh --lio fastlio --tag fastlio_ab \
#     --domain 93 --port 11503 --map-yaml src/rm_nav_bringup/map/RMUC2026.yaml --no-save-3d
#
# 产物（全部路径都会打印）：
#   <out-dir>/<tag>.log                     launch 全量日志
#   <out-dir>/<tag>.drive.json              coverage_drive.py 的体检 JSON
#   <out-dir>/<tag>.route.json              本次实际用的覆盖路线
#   <out-dir>/<tag>.pcd.stats.json          raw PCD 体检
#   <out-dir>/<tag>.final.stats.json        提交版 PCD 体检（体素下采样后）
#   <save-2d>.pgm/.yaml                     2D 栅格图（slam_toolbox 的 /map）
#   <final-pcd>                             3D 先验点云（按 --pcd-voxel 下采样、8 字段布局）
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LIO=""; TAG=""; WORLD="RMUC2026"; MAPPER="slam_toolbox"
DOMAIN=93; PORT=11503
ROUTE=""; MAP_YAML=""
SPEED=0.30; TIMEOUT=900; CLEARANCE=0.35; LANE=2.5; GOAL_STEP=1.5; SPIN_AT=""
SAVE_2D=""; SAVE_3D_RAW=""; FINAL_PCD=""; PCD_VOXEL=0.10
NO_SAVE_3D=0; DUMP_REG=1; OUT_DIR=""; WARMUP=8
POSE_SOURCE=lio; FINAL_SPIN=0; SPIN_RATE=0.7; LOOKAHEAD=0.8; DRIVER_EXTRA="" 

while [ $# -gt 0 ]; do
  case "$1" in
    --lio) LIO="$2"; shift 2;;
    --tag) TAG="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --mapper) MAPPER="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --route) ROUTE="$2"; shift 2;;
    --map-yaml) MAP_YAML="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --clearance) CLEARANCE="$2"; shift 2;;
    --lane-spacing) LANE="$2"; shift 2;;
    --goal-step) GOAL_STEP="$2"; shift 2;;
    --spin-at) SPIN_AT="$2"; shift 2;;
    --save-2d) SAVE_2D="$2"; shift 2;;
    --save-3d-raw) SAVE_3D_RAW="$2"; shift 2;;
    --final-pcd) FINAL_PCD="$2"; shift 2;;
    --pcd-voxel) PCD_VOXEL="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --warmup) WARMUP="$2"; shift 2;;
    --pose-source) POSE_SOURCE="$2"; shift 2;;
    --final-spin) FINAL_SPIN="$2"; shift 2;;
    --spin-rate) SPIN_RATE="$2"; shift 2;;
    --lookahead) LOOKAHEAD="$2"; shift 2;;
    --driver-args) DRIVER_EXTRA="$2"; shift 2;;
    --no-save-3d) NO_SAVE_3D=1; shift;;
    --no-dump-registered) DUMP_REG=0; shift;;
    -h|--help) sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$LIO" ] || { echo "必须给 --lio <fastlio|pointlio|small_point_lio>"; exit 2; }
[ -n "$TAG" ] || TAG="$LIO"
[ -n "$MAP_YAML" ] || MAP_YAML="$WS/src/rm_nav_bringup/map/$WORLD.yaml"
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_cache/mapping_$TAG"
[ -n "$SAVE_2D" ] || SAVE_2D="$WS/src/rm_nav_bringup/map/${WORLD}_${TAG}"
[ -n "$SAVE_3D_RAW" ] || SAVE_3D_RAW="$OUT_DIR/raw/${WORLD}_${TAG}_raw.pcd"
SPL_PCD_IN_TREE="$WS/src/rm_localization/small_point_lio/pcd/scan.pcd"

mkdir -p "$OUT_DIR" "$(dirname "$SAVE_3D_RAW")" "/tmp/gzhome-$TAG"
LOG="$OUT_DIR/$TAG.log"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$OUT_DIR/roslog_$TAG"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR=/tmp/mpl-$TAG
mkdir -p "$ROS_LOG_DIR"
unset DISPLAY
cd "$WS" || exit 2
: > "$LOG"

# ---------------------------------------------------------------- 路线
if [ -z "$ROUTE" ]; then
  ROUTE="$OUT_DIR/$TAG.route.json"
  echo "[run] 生成覆盖路线（coverage_route.py，先验图 $MAP_YAML）"
  python3 -u tools/scripts/mapping/coverage_route.py --map-yaml "$MAP_YAML" \
      --out "$ROUTE" --png "$OUT_DIR/$TAG.route.png" \
      --clearance "$CLEARANCE" --lane-spacing "$LANE" --goal-step "$GOAL_STEP" \
      --speed "$SPEED" 2>&1 | tee -a "$LOG"
  [ -f "$ROUTE" ] || { echo "[run] ❌ 路线没生成（先验图缺？）"; exit 2; }
else
  cp -f "$ROUTE" "$OUT_DIR/$TAG.route.json" 2>/dev/null || true
fi
echo "[run] 路线: $ROUTE"

# ---------------------------------------------------------------- launch
ARGS=(world:=$WORLD mode:=mapping lio:=$LIO mapper:=$MAPPER
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0)
echo "[run] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[run] launch pid=$LPID pgid=$PGID"

cleanup() {
  echo "[run] ---- cleanup $(date '+%T') ----"
  kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 15); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -f 'ros2 launch' 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'small_point_lio_node|fastlio_mapping|pointlio|lio_tf_adapter|slam_toolbox|map_saver' 2>/dev/null
  pkill -9 -f 'ground_segmentation_node|pointcloud_to_laserscan_node|spawn_entity.py|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform' 2>/dev/null
  sleep 2
  echo "[run] 残留复查:"; pgrep -af 'gzserver|ros2 launch|slam_toolbox|small_point_lio' || echo "  （无残留）"
}
trap cleanup EXIT

echo "[run] loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"
ok=0
for i in $(seq 1 75); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[run] ❌ launch 进程已退出（见 $LOG）"; break; fi
  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock'; then ok=1; break; fi
done
echo "[run] /clock 就绪=$ok（等了 ≈$((i*2)) s）" | tee -a "$LOG"
[ "$ok" = 1 ] || { echo "[run] ❌ Gazebo/launch 没起来"; tail -40 "$LOG"; exit 1; }

# --------------------------------------------------- 静态核对：节点/话题
# ⚠️ 必须在 /clock 就绪后再等一会：slam_toolbox 由 TimerAction(4 s) 起，planar_move 的
#    /odom_ground_truth 也晚几秒才 advertise ⇒ 立刻核对会误报"缺话题"（实测踩过）。
echo "[run] 等链路稳定 25 s ..."
sleep 25
echo "[run] --- 节点清单（核对：不许有 map_server / lio_tf_adapter 抢帧）---"
timeout 20 ros2 node list 2>/dev/null | sort | tee "$OUT_DIR/$TAG.nodes.txt"
echo "[run] --- 话题清单 ---"
timeout 20 ros2 topic list 2>/dev/null | sort | tee "$OUT_DIR/$TAG.topics.txt"
for t in /livox/lidar /livox/lidar/pointcloud /livox/imu /scan /odom /odom_ground_truth /map /cloud_registered; do
  grep -qx "$t" "$OUT_DIR/$TAG.topics.txt" && echo "[run] ✅ $t" || echo "[run] ❌ 缺话题 $t"
done
echo "[run] --- 参数文件横幅（证明 launch 到底读了哪份 yaml）---"
grep -a "使用的参数文件" "$LOG" | tail -2
echo "[run] --- 频率 ---"
for t in /livox/lidar /livox/lidar/pointcloud /livox/imu /scan /odom /map; do
  echo -n "[run] $t : "
  # /livox/imu 的发布者是 SensorDataQoS(BEST_EFFORT)（gazebo_ros_imu_sensor 的默认），
  # 默认 RELIABLE 的 `ros2 topic hz` 根本匹配不上 ⇒ 必须显式 best_effort，否则永远"没测到"。
  if [ "$t" = "/livox/imu" ]; then
    timeout 8 ros2 topic hz "$t" --qos-reliability best_effort 2>&1 | grep -m1 "average rate" \
      || echo "(8s 内没测到)"
  else
    timeout 8 ros2 topic hz "$t" 2>&1 | grep -m1 "average rate" || echo "(8s 内没测到)"
  fi
done | tee "$OUT_DIR/$TAG.hz.txt"
echo "[run] --- /map 拓扑（建图模式应只有 slam_toolbox 一个发布者）---"
timeout 20 ros2 topic info /map -v 2>&1 | grep -E "Publisher count|Node name|Topic type" | head -8 | tee "$OUT_DIR/$TAG.map_topology.txt"
echo "[run] --- save_pcd / map_save 服务是否在（源码依据 small_point_lio_node.cpp:37）---"
timeout 20 ros2 service list 2>/dev/null | grep -x "/map_save" && echo "[run] ✅ /map_save 在" || echo "[run] ❌ 没有 /map_save（save_pcd 未开？）"

# ---------------------------------------------------------------- 驱动
DRV=(python3 -u tools/scripts/mapping/coverage_drive.py
     --route "$ROUTE" --out-json "$OUT_DIR/$TAG.drive.json"
     --speed "$SPEED" --timeout "$TIMEOUT" --warmup "$WARMUP"
     --pose-source "$POSE_SOURCE" --final-spin "$FINAL_SPIN"
     --spin-rate "$SPIN_RATE" --lookahead "$LOOKAHEAD"
     --save-2d "$SAVE_2D")
[ -n "$DRIVER_EXTRA" ] && DRV+=($DRIVER_EXTRA)
[ -n "$SPIN_AT" ] && DRV+=(--spin-at "$SPIN_AT")
if [ "$NO_SAVE_3D" = 0 ]; then
  DRV+=(--save-3d "$SAVE_3D_RAW" --spl-pcd-in-tree "$SPL_PCD_IN_TREE")
fi
if [ "$DUMP_REG" = 1 ]; then
  DRV+=(--dump-registered "$OUT_DIR/$TAG.registered_voxels.npz")
fi
echo "[run] --- 跑图：${DRV[*]} ---" | tee -a "$LOG"
timeout $((TIMEOUT + 400)) "${DRV[@]}" 2>&1 | tee "$OUT_DIR/$TAG.drive.log"

# ---------------------------------------------------------------- 收尾统计
echo "[run] --- LIO/链路 关键字（ERROR/WARN/diverge/nan/transform 失败）---"
# gzclient 在无头环境必然 exit -6（unset DISPLAY 后没有 X）⇒ 已知噪声，不参与统计
grep -a -iE "error|warn|diverge|nan|invalid|failed to lookup" "$LOG" | grep -v "RCUTILS" \
  | grep -v "gzclient" | head -25 | tee "$OUT_DIR/$TAG.warnings.txt"
echo "[run] --- PCD 体检（raw）---"
if [ -f "$SAVE_3D_RAW" ]; then
  python3 -u tools/scripts/mapping/pcd_stats.py "$SAVE_3D_RAW" --json "$OUT_DIR/$TAG.pcd.stats.json" \
    | tee "$OUT_DIR/$TAG.pcd.stats.txt"
  if [ -n "$FINAL_PCD" ]; then
    echo "[run] --- PCD 下采样 → $FINAL_PCD（体素 $PCD_VOXEL m，8 字段布局）---"
    python3 -u tools/scripts/mapping/pcd_stats.py "$SAVE_3D_RAW" --voxel "$PCD_VOXEL" --out "$FINAL_PCD" \
      | tail -1 | tee "$OUT_DIR/$TAG.final.stats.txt"
    python3 -u tools/scripts/mapping/pcd_stats.py "$FINAL_PCD" --json "$OUT_DIR/$TAG.final.stats.json" \
      >> "$OUT_DIR/$TAG.final.stats.txt"
  fi
else
  echo "[run] （没有 raw PCD：--no-save-3d 或 /map_save 没落盘）"
fi
echo "[run] --- 2D 图落盘核对 ---"
ls -la "${SAVE_2D}.pgm" "${SAVE_2D}.yaml" 2>&1 | tee -a "$OUT_DIR/$TAG.artifacts.txt"
cat "${SAVE_2D}.yaml" 2>/dev/null | tee -a "$OUT_DIR/$TAG.artifacts.txt"
echo "[run] DONE $(date '+%F %T')  out-dir=$OUT_DIR"
