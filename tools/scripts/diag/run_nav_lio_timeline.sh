#!/usr/bin/env bash
# =============================================================================
# 「没撞击却飘里程计」专用跑法：起整栈（nav）→ 50 Hz LIO 时间轴记录仪 + 间隙/接触记录仪
#                              → 发用户那个目标 → 收尾（零速 + 杀干净）
#
# 为什么全部塞进一次 bash 调用：本环境每条 bash 命令跑在自己的 PID namespace 里
#   （bwrap --unshare-pid --die-with-parent），调用一结束命名空间里的所有进程（含 gzserver）
#   被内核清掉 ⇒ 起栈 / 记录 / 收尾必须在同一条命令里。见 docs/smoke_test_runbook.md。
#
# 隔离（与本仓既有 A/B 跑法一致）：HOME=/tmp/gzhome-<tag>（沙箱里 $HOME 只读，gzserver 建
#   ~/.gazebo 会 SIGABRT）、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI 端口、unset DISPLAY、
#   nav_rviz:=False/lio_rviz:=False、收尾发零速（由间隙记录仪负责）。
#
# 两个记录仪（都在同一次跑里，互不干扰）：
#   ① tools/scripts/diag/nav_lio_timeline.py       —— **只读** 50 Hz 状态行 + 逐消息事件行
#        （/odom 逐帧单步与隐含速度、真值、指令 ω、/scan 前后向最近回波、IMU、定位健康、限速）
#   ② tools/scripts/regress/nav_clearance_probe.py —— 发目标 + 10 Hz 间隙/接触/余量 + 汇总
#        （A/B 指标提取器 analyze_slope_speed_ab.py 吃的就是它的 samples.jsonl/summary.json）
#
# 用法（例 = 用户那次跑的同参）：
#   bash tools/scripts/diag/run_nav_lio_timeline.sh --tag t1 --domain 211 --port 11911 \
#        --goal -12.64 -0.31 --max-drive-sec 100 --post-goal-sec 25
#   # 单变量 A/B：临时改真源里的若干键（跑完**自动还原 + diff 核对**）
#   bash tools/scripts/diag/run_nav_lio_timeline.sh --tag f1 --domain 213 --port 11913 \
#        --kv "$PWD/src/rm_nav_bringup/config/traversability_criteria.yaml:speed_limit_floor_mps=0.60"
#   # 不发目标（只看静止/起栈基线）
#   bash tools/scripts/diag/run_nav_lio_timeline.sh --tag idle --domain 215 --port 11915 --no-goal
#
# 参数：
#   --tag / --domain / --port / --goal X Y
#   --map-yaml PATH   先验图（默认 = 用户当前的 map/RMUC2026_v3_spl.yaml）
#   --nav / --planner / --lio / --world / --localization    槽位（默认 = 用户那次跑）
#   --extra "a:=b c:=d"      追加 launch 参数（引号包住）
#   --kv "FILE:path.to.key=VALUE"   YAML 覆盖（可重复；FILE 相对 nav2 params 目录或绝对路径）
#   --max-drive-sec / --post-goal-sec / --settle / --ready-timeout
#   --hz              时间轴采样率（默认 50）
#   --no-goal         不跑间隙记录仪（= 不发目标，只录时间轴）
#   --out-root        产物根目录（默认 .tmp_lio_noimpact/out）
#
# 产物（<out-root>/<tag>/）：
#   launch.log      launch 全量日志（RCUTILS 带**墙钟时间戳**，可与记录仪对齐）
#   rows.jsonl      50 Hz 状态行
#   events.jsonl    逐消息事件行（odom/gt/scan/imu/mo/conv/recov/sl）
#   tl.summary.json 时间轴汇总（计数、age、丢队列）
#   samples.jsonl / summary.json / plans.jsonl   间隙记录仪产物
#   gate.txt        GICP 拒帧/失效/恢复行为/导航结局（带墙钟戳，用于因果排序）
#   used/           本次实际生效的参数文件（.orig / .used）
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; DOMAIN=211; PORT=11911; GOAL="-12.64 -0.31"; LOC="gicp"; NAV="mppi"; PLANNER="smac2d"
LIO="small_point_lio"; WORLD="RMUC2026"; MAXDRIVE=100; POST=25; SETTLE=10; READY=300; HZ=50
MAP_YAML="$WS/src/rm_nav_bringup/map/RMUC2026_v3_spl.yaml"
EXTRA=""; NO_GOAL=0; OUT_ROOT="$WS/.tmp_lio_noimpact/out"; KVS=(); CKVS=()
PARAMS_DIR="$WS/src/rm_navigation/rm_navigation/params"
CRITERIA="$WS/src/rm_nav_bringup/config/traversability_criteria.yaml"

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --goal) GOAL="$2 $3"; shift 3;;
    --map-yaml) MAP_YAML="$2"; shift 2;;
    --localization) LOC="$2"; shift 2;;
    --nav) NAV="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --lio) LIO="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --extra) EXTRA="$2"; shift 2;;
    --max-drive-sec) MAXDRIVE="$2"; shift 2;;
    --post-goal-sec) POST="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    --ready-timeout) READY="$2"; shift 2;;
    --hz) HZ="$2"; shift 2;;
    --out-root) OUT_ROOT="$2"; shift 2;;
    --kv) KVS+=("$2"); shift 2;;
    --ckv) CKVS+=("$2"); shift 2;;
    --criteria) CRITERIA="$2"; shift 2;;
    --no-goal) NO_GOAL=1; shift;;
    -h|--help) sed -n '2,52p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }

OUT="$OUT_ROOT/$TAG"; LOGDIR="$OUT/roslog"; USED="$OUT/used"
rm -rf "$OUT"; mkdir -p "$OUT" "$USED" "$LOGDIR" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"

# ---------------- 参数覆盖（跑完**必须还原**） ----------------
FILES=(nav2_params_sim_base.yaml "nav2_params_sim_planner_${PLANNER}.yaml" \
       "nav2_params_sim_controller_${NAV}.yaml")
KVFILES=()
for kv in "${KVS[@]:-}"; do
  [ -n "$kv" ] || continue
  lhs="${kv%%=*}"; f="${lhs%%:*}"
  case "$f" in /*) KVFILES+=("$f");; *) KVFILES+=("$PARAMS_DIR/$f");; esac
done
BACKUP=()
for f in "${FILES[@]}"; do BACKUP+=("$PARAMS_DIR/$f"); done
for f in "${KVFILES[@]:-}"; do [ -n "$f" ] && BACKUP+=("$f"); done
for f in "${BACKUP[@]}"; do
  [ -f "$f" ] || continue
  cp -p "$f" "$USED/$(echo "$f" | tr '/' '_').orig"
done
restore_cfg() {
  for f in "${BACKUP[@]}"; do
    b="$USED/$(echo "$f" | tr '/' '_').orig"
    [ -f "$b" ] && cp -p "$b" "$f"
  done
  return 0
}
if [ ${#KVS[@]} -gt 0 ]; then
  echo "[run] 应用 YAML 覆盖："
  python3 "$WS/tools/scripts/regress/apply_nav_kv.py" "$PARAMS_DIR" "${KVS[@]}"
  [ $? -eq 0 ] || { echo "[run] ❌ 覆盖失败"; restore_cfg; exit 4; }
fi
# ---- 可通行性真源（traversability_criteria.yaml）的**保注释**就地覆盖 ----
#   为什么不用 apply_nav_kv.py：那份文件的价值一半在注释（表怎么推出来的），
#   yaml round-trip 会把注释全抹掉（跑完虽能还原，但中途崩一次就丢了）。
#   这里只在**指定键所在的那一行**上换值，注释与排版原样保留。
if [ ${#CKVS[@]} -gt 0 ]; then
  BACKUP+=("$CRITERIA")
  cp -p "$CRITERIA" "$USED/$(echo "$CRITERIA" | tr '/' '_').orig"
  echo "[run] 覆盖可通行性真源：${CKVS[*]}"
  python3 - "$CRITERIA" "${CKVS[@]}" <<'PY'
import re, sys
path, kvs = sys.argv[1], sys.argv[2:]
txt = open(path, encoding='utf-8').read()
for kv in kvs:
    k, v = kv.split('=', 1)
    pat = re.compile(r'^(\s*%s:\s*)([^#\n]*)(#.*)?$' % re.escape(k.strip()), re.M)
    if not pat.search(txt):
        sys.exit('[run] ❌ 真源里没有键 %s' % k)
    txt = pat.sub(lambda m: m.group(1) + v + '   # [A/B 临时覆盖]' +
                  (m.group(3) or ''), txt, count=1)
    print('  %s = %s' % (k, v))
open(path, 'w', encoding='utf-8').write(txt)
PY
  [ $? -eq 0 ] || { echo "[run] ❌ 真源覆盖失败"; restore_cfg; exit 4; }
fi
for f in "${BACKUP[@]}"; do cp -p "$f" "$USED/$(echo "$f" | tr '/' '_').used"; done

# ---------------- 环境 ----------------
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$LOGDIR"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{time}] [{severity}] [{name}]: {message}'
export MPLCONFIGDIR="/tmp/mpl-$TAG"
unset DISPLAY
cd "$WS" || exit 2

TLPID=""
cleanup() {
  echo "[run] ---- cleanup ----"
  restore_cfg
  [ -n "$TLPID" ] && kill -TERM "$TLPID" 2>/dev/null
  sleep 1
  [ -n "${PGID:-}" ] && kill -INT -- "-$PGID" 2>/dev/null
  [ -n "${LPID:-}" ] && for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  [ -n "${PGID:-}" ] && kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null || true
  pkill -9 -f 'nav_lio_timeline|nav_clearance_probe|gicp_registration_node|small_point_lio|pointcloud_to_laserscan|ground_segmentation|spawn_entity|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform|controller_server|planner_server|bt_navigator|map_server|amcl|ros2 launch' 2>/dev/null || true
  sleep 2
  echo "[run] 残留："; pgrep -a -x gzserver || echo "  （无 gzserver）"
  echo "[run] 参数还原核对："
  for f in "${BACKUP[@]}"; do
    b="$USED/$(echo "$f" | tr '/' '_').orig"
    [ -f "$b" ] || continue
    if diff -q "$b" "$f" >/dev/null; then echo "  ✓ $(basename "$f") 与跑前一致";
    else echo "  ❌ $(basename "$f") 与跑前不一致！"; diff -u "$b" "$f" | head -20; fi
  done
}
trap cleanup EXIT

# ---------------- 残留守卫（别人/上次的 gzserver 会抢 GAZEBO_MASTER_URI） ----------------
echo "[run] ==== 残留检查（tag=$TAG domain=$DOMAIN port=$PORT）$(date '+%F %T') ===="
LEFT=$(pgrep -a -x gzserver || true)
if [ -n "$LEFT" ]; then
  echo "[run] ⚠️ 发现有残留 gzserver："; echo "$LEFT"
  pkill -9 -x gzserver 2>/dev/null || true
  sleep 2
fi
if command -v ss >/dev/null 2>&1; then
  if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "[run] ❌ 端口 $PORT 已被占用"; ss -ltnp 2>/dev/null | grep ":$PORT "; exit 3
  fi
fi
[ -f "$MAP_YAML" ] || { echo "[run] ❌ 先验图不存在：$MAP_YAML"; exit 2; }
echo "[run] 残留检查通过（无 gzserver、端口 $PORT 空闲）loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"
echo "[run] 先验图：$MAP_YAML"

# ---------------- 起栈 ----------------
ARGS=(world:=$WORLD mode:=nav lio:=$LIO localization:=$LOC nav:=$NAV planner:=$PLANNER
      nav_rviz:=False lio_rviz:=False map_yaml:=$MAP_YAML)
[ -n "$EXTRA" ] && ARGS+=($EXTRA)
LOG="$OUT/launch.log"; : > "$LOG"
echo "[run] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!; PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[run] launch pid=$LPID pgid=$PGID"

for i in $(seq 1 120); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[run] ❌ launch 退出（见 $LOG）"; tail -40 "$LOG"; exit 1; fi
  timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock' && break
done
echo "[run] /clock 就绪（≈$((i*2)) s）"

# ---------------- ① 50 Hz 只读时间轴（先起，覆盖整段：settle/行驶/post） ----------------
TLRUN=$(( SETTLE + MAXDRIVE + POST + 120 ))
python3 -u "$WS/tools/scripts/diag/nav_lio_timeline.py" --tag "$TAG" \
  --out "$OUT/rows.jsonl" --events "$OUT/events.jsonl" --summary "$OUT/tl.summary.json" \
  --hz "$HZ" --run-sec "$TLRUN" --ready-timeout "$READY" >"$OUT/timeline.log" 2>&1 &
TLPID=$!
echo "[run] 时间轴记录仪 pid=$TLPID（最多录 ${TLRUN}s）"

# ---------------- ② 间隙/接触记录仪（发目标 + 10 Hz 汇总） ----------------
if [ "$NO_GOAL" = "1" ]; then
  echo "[run] --no-goal：只录时间轴，等到 ${TLRUN}s"
  wait "$TLPID" 2>/dev/null || true
  WRC=0
else
  PARGS=(--tag "$TAG" --out "$OUT/samples.jsonl" --summary "$OUT/summary.json"
         --settle "$SETTLE" --ready-timeout "$READY" --max-drive-sec "$MAXDRIVE"
         --post-goal-sec "$POST" --goal $GOAL)
  echo "[run] 记录仪：python3 tools/scripts/regress/nav_clearance_probe.py ${PARGS[*]}"
  python3 -u "$WS/tools/scripts/regress/nav_clearance_probe.py" "${PARGS[@]}" >"$OUT/probe.log" 2>&1
  WRC=$?
  echo "[run] 记录仪 rc=$WRC"; tail -12 "$OUT/probe.log"
  sleep 3
  kill -TERM "$TLPID" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$TLPID" 2>/dev/null || break; sleep 1; done
fi
kill -0 "$TLPID" 2>/dev/null && { echo "[run] ⚠️ 时间轴记录仪仍在，强杀"; kill -9 "$TLPID" 2>/dev/null; }
TLPID=""

# ---------------- 摘录：拒帧 / 恢复行为 / 导航结局（带墙钟戳 ⇒ 可与时间轴对齐） ----------------
{
  echo "########## GICP 拒帧 / 失效 / 采纳（带墙钟时间戳） ##########"
  grep -aE "GICP 本帧未被采纳|连续 .* 帧被拒绝|定位已判失效|用参数 initial_pose 初始化" "$LOG" | head -400 || echo "(无)"
  echo; echo "########## 恢复行为（behavior_server 的 Running/Canceling <plugin>） ##########"
  grep -aE "behavior_server\]: (Running|Canceling|Completed) " "$LOG" | head -400 || echo "(无)"
  echo; echo "########## 导航结局 / recovery / 进度 / 断点 ##########"
  grep -aE "Reached the goal|Goal succeeded|Goal failed|Aborting handle|Failed to make progress|Running recovery|clearing costmap|Clearing costmap|extrapolation|Invalid frame ID|Begin navigating" "$LOG" | head -300 || echo "(无)"
  echo; echo "########## [slope_speed] 限速行（头 40 / 尾 20） ##########"
  grep -a "\[slope_speed\]" "$LOG" | head -40 || echo "(无)"
  echo "  ...（共 $(grep -ac '\[slope_speed\]' "$LOG" || echo 0) 行）..."
  grep -a "\[slope_speed\]" "$LOG" | tail -20 || echo "(无)"
} > "$OUT/gate.txt" 2>&1

if [ -f "$OUT/summary.json" ]; then
python3 - "$OUT" <<'PY'
import json, os, sys
out = sys.argv[1]
s = json.load(open(os.path.join(out, 'summary.json')))
with open(os.path.join(out, 'plans.jsonl'), 'w') as f:
    for e in s.get('plans', []):
        f.write(json.dumps(e, ensure_ascii=False) + '\n')
print('[run] goal_status=%s travel_gt=%.2fm contact=%d blocked=%d pushed=%d mo_stale=%d'
      % (s.get('goal_status'), s.get('travel_gt', 0.0), s.get('contact_events', 0),
         len(s.get('blocked', [])), len(s.get('pushed', [])), len(s.get('mo_stale_runs', []))))
PY
fi

echo "[run] DONE tag=$TAG rc_probe=$WRC out=$OUT"
exit $WRC
