#!/usr/bin/env bash
# =============================================================================
# run_remap_lowterrain.sh —— 按**用户 workflow**重跑一次建图（测试专用名字）
# -----------------------------------------------------------------------------
# 目的：验证「修好感知之后重跑建图 ⇒ 新先验图里坡脚/台阶不再隐身」这条路径**真的能跑通**，
#       并把用户要照抄的那条命令（mode:=mapping + cloud_accumulator + map_archive.sh save）
#       原样跑一遍。
#
# ⚠ 只用**测试专用**的 map_name / 输出路径（默认 RMUC2026_lt），跑完由调用方清理；
#   **不碰**用户的 map/RMUC2026*、PCD/RMUC2026*（先验图只读，用来生成覆盖路线）。
#
# 用法（一次 bash 调用跑完）：
#   bash tools/scripts/regress/run_remap_lowterrain.sh --tag ltmap --domain 165 --port 11865 \
#        --map-name RMUC2026_lt --timeout 300
#
# 退出码：0 成功；**非 0 = 真的失败**（包括"存档那一步失败"—— 以前这一行是
#   `... | tee ... || true`，save 的退出码被吞掉 ⇒ 建图跑完却什么都没有，看着像成功；
#   现在显式传 `${PIPESTATUS[0]}`，见文件末尾与 docs/frame_contract_v3_measurements.md §19）。
#   可选 `--map-resolution R`：透传给 `map_archive.sh save --resolution R`。
#
# 产物：
#   map/<map-name>.{posegraph,data,meta.yaml}    ← slam_toolbox 存档（测试名）
#   PCD/<map-name>.pcd + .meta.yaml              ← cloud_accumulator 的 3D 先验（测试名）
#   <out-dir>/<tag>.log / .route.json / .drive.json / .hz.txt
# =============================================================================
set -o pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# 收尾只杀「本 run 自己的」进程：进程组 + /proc/<pid>/environ 的 GAZEBO_MASTER_URI 核对（见该文件）
. "$WS/tools/scripts/lib/scoped_cleanup.sh"
TAG=""; DOMAIN=165; PORT=11865; WORLD=RMUC2026; MAP_NAME="RMUC2026_lt"; OUT_DIR=""
TIMEOUT=300; SPEED=0.35; GROUND=linefit; PRIOR_MAP=""
# ★ 2026-10-20：`--map-resolution R` 可选 —— 给了就透传给 `map_archive.sh save --resolution R`
#   （写进 sidecar 的 resolution:）；不给 ⇒ 由 save 自己 `ros2 param get /slam_toolbox resolution`。
MAP_RES=""

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --map-name) MAP_NAME="$2"; shift 2;;
    --ground) GROUND="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --prior-map) PRIOR_MAP="$2"; shift 2;;
    --map-resolution) MAP_RES="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_lowterrain/remap_$TAG"
[ -n "$PRIOR_MAP" ] || PRIOR_MAP="$WS/src/rm_nav_bringup/map/$WORLD.yaml"
mkdir -p "$OUT_DIR" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"
LOG="$OUT_DIR/$TAG.log"; ROUTE="$OUT_DIR/$TAG.route.json"

export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_LOG_DIR="$OUT_DIR/roslog"; mkdir -p "$ROS_LOG_DIR"
export MPLCONFIGDIR="/tmp/mpl-$TAG"
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{time}] [{severity}] [{name}]: {message}'
unset DISPLAY
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS" || exit 2

echo "[remap:$TAG] ==== 残留检查 $(date '+%F %T') ===="
LEFT=$(sc_own_gz_pids)
if [ -n "$LEFT" ]; then echo "[remap:$TAG] ⚠️ 残留 gzserver（本 master URI）："; echo "$LEFT"; sc_kill_own_gz 9; sleep 2; fi
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  echo "[remap:$TAG] ❌ 端口 $PORT 被占用"; ss -ltnp 2>/dev/null | grep ":$PORT "; exit 3
fi
echo "[remap:$TAG] 残留检查通过 loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"

cleanup() {
  echo "[remap:$TAG] ---- cleanup ----"
  timeout 10 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  sleep 1
  [ -n "${PGID:-}" ] && kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do [ -n "${LPID:-}" ] && kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  [ -n "${PGID:-}" ] && kill -9 -- "-$PGID" 2>/dev/null
  # 作用域：名字只用来挑候选，必须过归属判定（本进程组 / 本 master URI / 本脚本后代）才杀
  sc_kill_own_ros2_launch 9 'ros2 launch rm_nav_bringup'
  sc_kill_own_gz 9
  sc_kill_own_pattern 9 'small_point_lio|slam_toolbox|spawn_entity|robot_state_publisher|complementary_filter|fake_vel_transform|pointcloud_to_laserscan|ground_segmentation_node|coverage_drive'
  sleep 2
  echo "[remap:$TAG] 残留（本 master URI/进程组）：$(sc_own_leftovers_desc 'gzserver|gzclient|ros2 launch')"
}
trap cleanup EXIT INT TERM

# ---- 覆盖路线（按**现有**先验图生成；只读用户资产）----
echo "[remap:$TAG] 生成覆盖路线（先验图 $PRIOR_MAP）"
python3 -u tools/scripts/mapping/coverage_route.py --map-yaml "$PRIOR_MAP" --out "$ROUTE" \
  --png "$OUT_DIR/$TAG.route.png" --clearance 0.35 --lane-spacing 2.5 --goal-step 1.5 \
  --speed "$SPEED" > "$OUT_DIR/$TAG.route.log" 2>&1 || { echo "[remap:$TAG] ❌ 路线生成失败"; tail -20 "$OUT_DIR/$TAG.route.log"; exit 2; }
echo "[remap:$TAG] 路线 OK"

# ---- 起栈：**用户 workflow** 的建图命令 + cloud_accumulator ----
ARGS=(world:=$WORLD mode:=mapping lio:=small_point_lio mapper:=slam_toolbox
      map_name:=$MAP_NAME cloud_accumulator:=True ground:=$GROUND
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0)
: > "$LOG"
echo "[remap:$TAG] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
for i in $(seq 1 90); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[remap:$TAG] ❌ launch 退出"; tail -30 "$LOG"; exit 1; fi
  timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock' && break
done
echo "[remap:$TAG] /clock 就绪（≈$((i*2)) s）"
grep -a "使用的参数文件\|判据（坡度/台阶）" "$LOG" | tail -3
sleep 20
echo "[remap:$TAG] --- 关键话题 ---"; timeout 20 ros2 topic list 2>/dev/null | sort | tee "$OUT_DIR/$TAG.topics.txt" | grep -E '^/(scan|map|segmentation/step_edge|cloud_registered)$' || true
echo "[remap:$TAG] --- 会话（map_archive 要用）---"; timeout 20 ros2 topic echo /map_session/info --once --qos-durability transient_local 2>&1 | head -12

# ---- 跑图（coverage_drive.py 自己会在结束时存 2D）----
python3 -u tools/scripts/mapping/coverage_drive.py --route "$ROUTE" \
  --out-json "$OUT_DIR/$TAG.drive.json" --speed "$SPEED" --timeout "$TIMEOUT" --warmup 8 \
  --pose-source lio --lookahead 0.8 --save-2d "$OUT_DIR/$TAG.slam2d" \
  --dump-registered "$OUT_DIR/$TAG.registered.npz" 2>&1 | tee "$OUT_DIR/$TAG.drive.log"

# ---- 存档：**用户 workflow 的最后一步** ----
# ★ 2026-10-20 修：这一行原来写的是
#       timeout 300 …/map_archive.sh save --name "$MAP_NAME" 2>&1 | tee … || true
#   两个问题叠加 ⇒ **静默失败**：
#     ① `|| true` 把整条管道的退出码吃掉（`set -o pipefail` 也救不了它）；
#     ② 而 `map_archive.sh save` 一旦收到不认识的参数（当时 `--resolution` 就是）会打 usage
#        然后 exit 2 ⇒ 看着"跑完了"，磁盘上什么都没有（2026-10-19 建图轮的真实事故，见
#        docs/frame_contract_v3_measurements.md §19）。
#   现在：显式取 `${PIPESTATUS[0]}`（save 自己的退出码，不是 tee 的），非 0 就**把本脚本也
#   弄成非 0**（`exit $rc`），并把失败写进日志目录。`--map-resolution` 是可选项：给了就透传
#   给 `save --resolution`（不给 ⇒ 由 save 去问活 mapper，行为与以前一致）。
echo "[remap:$TAG] --- map_archive.sh save（--name $MAP_NAME${MAP_RES:+ --resolution $MAP_RES}）---"
SAVE_ARGS=(--name "$MAP_NAME")
[ -n "$MAP_RES" ] && SAVE_ARGS+=(--resolution "$MAP_RES")
set +e
timeout 300 tools/scripts/mapping/map_archive.sh save "${SAVE_ARGS[@]}" 2>&1 | tee "$OUT_DIR/$TAG.save.log"
SAVE_RC=${PIPESTATUS[0]}
if [ "$SAVE_RC" != 0 ]; then
  echo "[remap:$TAG] ❌ map_archive.sh save 失败（退出码 $SAVE_RC）—— 存档**没有写成/不完整**；" >&2
  echo "[remap:$TAG]    上面是它的完整输出（也落盘在 $OUT_DIR/$TAG.save.log）。" >&2
  echo "[remap:$TAG]    退出码含义：2 用法/环境 · 3 守卫拒绝（场地/会话名） · 4 落盘超时。" >&2
  exit "$SAVE_RC"
fi
echo "[remap:$TAG] --- 存档产物 ---"
ls -la --time-style=+%H:%M "src/rm_nav_bringup/map/$MAP_NAME".* "src/rm_nav_bringup/PCD/$MAP_NAME".* 2>&1 | head -12
echo "[remap:$TAG] DONE"
