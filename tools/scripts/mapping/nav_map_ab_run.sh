#!/usr/bin/env bash
# =============================================================================
# 2D 先验图 A/B 单跑：把一张候选 pgm/yaml **临时**放到 launch 硬编码的名字上，
# 起 mode:=nav localization:=gicp，问规划器能不能过，再跑一次 P0 回归，最后还原。
#
# ⚠️ 为什么必须"临时换文件"：`bringup_sim.launch.py:168` 把 nav2 的图名硬编码成
#    `map/<world>.yaml`（`:230` 把 PCD 硬编码成 `PCD/<world>.pcd`），**没有任何 launch
#    参数能改文件名**。所以 A/B 只能占用这两个文件名。
#    ⇒ 本脚本**开跑前要求 `map/<world>.pgm/.yaml` 不存在**（有就拒绝跑，绝不覆盖别人的东西），
#      结束（含 Ctrl-C / 出错）时在 trap 里删掉自己写的那两份，并逐个校验"确实不存在了"。
#      全程不碰 `*.prior` 之类的旁支文件。
#
# 用法：
#   bash tools/scripts/mapping/nav_map_ab_run.sh --tag scanmap \
#     --map-prefix src/rm_nav_bringup/map/RMUC2026_spl \
#     --goal 0.5 3.0 --domain 98 --port 11508 --out-dir .tmp_cache/nav_ab/scanmap
#
# 产物：<out>/<tag>.log / .regress.txt / .plan.json / .summary.json
# =============================================================================
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; MAP_PREFIX=""; OUT_DIR=""; GOAL="0.5 3.0"; WORLD="RMUC2026"
DOMAIN=98; PORT=11508; PLANNER=navfn; NAV=rpp; LOCALIZATION=""
PLAN_GOALS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --map-prefix) MAP_PREFIX="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --goal) GOAL="$2 $3"; shift 3;;
    --world) WORLD="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --nav) NAV="$2"; shift 2;;
    --localization) LOCALIZATION="$2"; shift 2;;
    --plan-goal) PLAN_GOALS+=("$2 $3"); shift 3;;   # 收 X Y 两个值
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] && [ -n "$MAP_PREFIX" ] || { echo "必须给 --tag 与 --map-prefix"; exit 2; }
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_cache/nav_ab/$TAG"
MAPDIR="$WS/src/rm_nav_bringup/map"
PGM="$MAPDIR/$WORLD.pgm"; YML="$MAPDIR/$WORLD.yaml"
mkdir -p "$OUT_DIR" "/tmp/gzhome-nav-$TAG"

if [ -e "$PGM" ] || [ -e "$YML" ]; then
  echo "[navab] ❌ $PGM(.yaml) 已经存在 —— 可能别人正在用，拒绝覆盖。先确认再跑。" >&2
  exit 3
fi
echo "[navab] 快照现有 map/ 目录（用于事后逐项核对）："
ls -la "$MAPDIR" > "$OUT_DIR/$TAG.mapdir.before.txt"
sha256sum "$MAPDIR"/* > "$OUT_DIR/$TAG.mapdir.before.sha256" 2>/dev/null || true

cp -f "$MAP_PREFIX.pgm" "$PGM"
cp -f "$MAP_PREFIX.yaml" "$YML"

cleanup() {
  echo "[navab] ---- cleanup ----"
  kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -f 'ros2 launch' 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'small_point_lio_node|slam_toolbox|map_server|planner_server|controller_server|bt_navigator|amcl|gicp_registration|lifecycle_manager|waypoint_follower|velocity_smoother' 2>/dev/null
  pkill -9 -f 'spawn_entity.py|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform|pointcloud_to_laserscan' 2>/dev/null
  sleep 2
  rm -f "$PGM" "$YML"
  echo "[navab] 还原核对：pgm存在=$([ -e "$PGM" ] && echo YES || echo no) yaml存在=$([ -e "$YML" ] && echo YES || echo no)"
  ls -la "$MAPDIR" > "$OUT_DIR/$TAG.mapdir.after.txt" 2>/dev/null || true
  if diff -q "$OUT_DIR/$TAG.mapdir.before.txt" "$OUT_DIR/$TAG.mapdir.after.txt" >/dev/null 2>&1; then
    echo "[navab] ✅ map/ 目录与跑前逐行一致"
  else
    echo "[navab] ⚠️ map/ 目录与跑前不一致（自己看 $OUT_DIR/$TAG.mapdir.after.txt）"
  fi
}

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-nav-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$OUT_DIR/roslog_$TAG"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR="/tmp/mpl-nav-$TAG"
mkdir -p "$ROS_LOG_DIR"
unset DISPLAY
cd "$WS" || exit 2
LOG="$OUT_DIR/$TAG.log"; : > "$LOG"
trap cleanup EXIT

echo "[navab] map A/B：tag=$TAG 用图=$MAP_PREFIX（临时占用 $WORLD.pgm/.yaml）"
ARGS=(world:=$WORLD mode:=nav lio:=small_point_lio localization:=gicp nav:=$NAV planner:=$PLANNER
      spin_speed:=0.0 nav_rviz:=False)
echo "[navab] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
ok=0
for i in $(seq 1 90); do
  sleep 2
  kill -0 "$LPID" 2>/dev/null || { echo "[navab] ❌ launch 退出"; tail -30 "$LOG"; break; }
  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock'; then ok=1; break; fi
done
echo "[navab] /clock 就绪=$ok（≈$((i*2)) s）"
[ "$ok" = 1 ] || { echo "[navab] ❌ Gazebo 没起来"; exit 1; }
echo "[navab] 等导航栈起来 45 s ..."
sleep 45
echo "[navab] --- /map 拓扑 + 实际加载的图 ---"
timeout 20 ros2 topic info /map -v 2>&1 | grep -E "Publisher count|Node name" | head -4 | tee "$OUT_DIR/$TAG.maptopo.txt"
timeout 25 ros2 topic echo /map --once --field info 2>&1 | head -12 | tee "$OUT_DIR/$TAG.mapinfo.txt"
echo "[navab] --- 记 GICP 状态 ---"
grep -a -E "采纳|fitness|score|align" "$LOG" | tail -3 | tee "$OUT_DIR/$TAG.gicp.txt" || true

echo "[navab] --- P0 回归（goal=$GOAL）---"
timeout 400 python3 -u tools/scripts/regress/nav_smoke_regression.py --goal $GOAL \
  --localization "${LOCALIZATION:-$TAG}" --outdir "$OUT_DIR/regress" 2>&1 | tee "$OUT_DIR/$TAG.regress.txt"
REG_RC=$?

echo "[navab] --- 规划器探测（坡道/走廊目标）---"
PGARGS=()
for g in "${PLAN_GOALS[@]}"; do PGARGS+=(--goal $g); done
if [ "${#PGARGS[@]}" -gt 0 ]; then
  timeout 180 python3 -u tools/scripts/regress/plan_probe.py --start 0.0 0.0 \
    "${PGARGS[@]}" --label "$TAG" --out "$OUT_DIR/$TAG.plan.json" 2>&1 | tee "$OUT_DIR/$TAG.plan.txt"
  PLAN_RC=$?
else
  PLAN_RC=99
fi
python3 - "$OUT_DIR" "$TAG" "$MAP_PREFIX" "$GOAL" "$REG_RC" "$PLAN_RC" <<'PY' | tee "$OUT_DIR/$TAG.summary.json"
import json,sys,os
out,tag,mp,goal,reg,plan=sys.argv[1:7]
def j(p,d=None):
    try: return json.load(open(p))
    except Exception: return d
s={'tag':tag,'map_prefix':mp,'goal':goal,'regress_rc':int(reg),'plan_rc':int(plan),
   'plan':j(os.path.join(out,tag+'.plan.json'),{}),
   'mapinfo':open(os.path.join(out,tag+'.mapinfo.txt')).read() if os.path.exists(os.path.join(out,tag+'.mapinfo.txt')) else '',
   'regress_tail':open(os.path.join(out,tag+'.regress.txt')).read()[-2500:] if os.path.exists(os.path.join(out,tag+'.regress.txt')) else ''}
json.dump(s,open(os.path.join(out,tag+'.summary.json'),'w'),ensure_ascii=False,indent=1)
print(json.dumps({k:v for k,v in s.items() if k not in ('regress_tail','mapinfo')},ensure_ascii=False))
PY
echo "[navab] DONE tag=$TAG out=$OUT_DIR"
