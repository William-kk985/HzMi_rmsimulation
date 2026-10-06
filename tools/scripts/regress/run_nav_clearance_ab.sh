#!/usr/bin/env bash
# =============================================================================
# 「贴膨胀边 / 撞进去 / 定位跟着坏」的 A/B 跑法（**无头**、一次 bash 调用跑完）
#
# 为什么全部塞进一次 bash 调用：本环境每条 bash 命令跑在自己的 PID namespace 里
#   （bwrap --unshare-pid --die-with-parent），调用一结束命名空间里的所有进程
#   （含 gzserver）被内核清掉 ⇒ 起栈 / 记录 / 收尾必须在同一条命令里。
#
# 隔离（与本仓库既有 A/B 跑法一致）：HOME=/tmp/gzhome-<tag>（沙箱里 $HOME 只读，
#   gzserver 建 ~/.gazebo 会 SIGABRT）、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI 端口、
#   unset DISPLAY、nav_rviz:=False、收尾发零速（由记录仪负责）。
#
# 用法（例 = 用户那次跑的同参基线）：
#   bash tools/scripts/regress/run_nav_clearance_ab.sh --tag base --domain 151 --port 11821 \
#        --goal -12.64 -0.31 --extra "spin_speed:=0.0" --max-drive-sec 90 --post-goal-sec 20
#
#   # nav2 参数 A/B：只在这一次跑里覆盖若干 nav2 键（跑完**自动还原**）
#   bash tools/scripts/regress/run_nav_clearance_ab.sh --tag p_infl --domain 152 --port 11822 \
#        --goal -12.64 -0.31 --extra "spin_speed:=0.0" \
#        --kv "nav2_params_sim_base.yaml:global_costmap.inflation_layer.inflation_radius=0.85"
#
# 参数：
#   --tag            本次跑的标签（输出目录名）
#   --domain         非默认 ROS_DOMAIN_ID（每个 tag 一个，避免串台）
#   --port           独立 GAZEBO_MASTER_URI 端口
#   --goal X Y       目标点（map 系）
#   --nav            控制器槽（rpp|dwb|teb|mppi，默认 mppi = 用户那次）
#   --planner        规划器槽（navfn|smac2d，默认 smac2d = 用户那次）
#   --lio / --world / --localization   与用户那次一致
#   --extra          追加 launch 参数（引号包住，例："spin_speed:=0.0"）
#   --kv "FILE:path.to.key=VALUE"    nav2 参数覆盖（可重复；FILE 在 rm_navigation/params/
#                    下按名找，也可给绝对路径；path 相对该文件里 `ros__parameters` 那一层；
#                    键必须已存在，否则报错退出 —— 免得"静默没生效"的 A/B 白跑）
#   --max-drive-sec  发目标后最多驱动多久（墙钟，默认 90）
#   --post-goal-sec  终态/超时后继续录多久（墙钟，默认 20）
#   --settle         发目标前静置秒数（墙钟，默认 10）
#   --ready-timeout  等链路上限（墙钟秒，默认 300）
#
# 产物（默认 .tmp_clear/out/<tag>/ 下）：
#   launch.log        launch 全量日志（含**墙钟时间戳**：RCUTILS_CONSOLE_OUTPUT_FORMAT）
#   samples.jsonl     10 Hz 逐样本（位姿/余量/接触残差/定位事件）
#   summary.json      汇总（余量分布、接触事件、定位事件、结局）
#   plans.jsonl       每次 /plan 的长度与最小余量（也在 summary.plans 里）
#   gate.txt          GICP 拒帧/失效/导航结局行（带墙钟时间戳，用于因果排序）
#   used/             本次实际生效的参数文件副本
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; DOMAIN=151; PORT=11821; GOAL="-12.64 -0.31"; LOC="gicp"; NAV="mppi"; PLANNER="smac2d"
LIO="small_point_lio"; WORLD="RMUC2026"; MAXDRIVE=90; POST=20; SETTLE=10; READY=300
EXTRA=""; OUT_ROOT="$WS/.tmp_clear/out"; KVS=()
PARAMS_DIR="$WS/src/rm_navigation/rm_navigation/params"

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --goal) GOAL="$2 $3"; shift 3;;
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
    --out-root) OUT_ROOT="$2"; shift 2;;
    --kv) KVS+=("$2"); shift 2;;
    -h|--help) sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }

OUT="$OUT_ROOT/$TAG"; LOGDIR="$OUT/roslog"; USED="$OUT/used"
rm -rf "$OUT"; mkdir -p "$OUT" "$USED" "$LOGDIR" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"

# ---------------- 参数覆盖（跑完**必须还原**） ----------------
# 备份范围 = nav2 三个槽位文件 + 任何被 --kv 点名的文件（例如 gicp 的 config yaml）
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
  echo "[run] 应用 nav2 覆盖："
  python3 "$WS/tools/scripts/regress/apply_nav_kv.py" "$PARAMS_DIR" "${KVS[@]}"
  [ $? -eq 0 ] || { echo "[run] ❌ 覆盖失败"; restore_cfg; exit 4; }
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

cleanup() {
  echo "[run] ---- cleanup ----"
  restore_cfg
  [ -n "${PGID:-}" ] && kill -INT -- "-$PGID" 2>/dev/null
  [ -n "${LPID:-}" ] && for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  [ -n "${PGID:-}" ] && kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null || true
  pkill -9 -f 'gicp_registration_node|small_point_lio|pointcloud_to_laserscan|ground_segmentation|spawn_entity|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform|controller_server|planner_server|bt_navigator|map_server|amcl|ros2 launch' 2>/dev/null || true
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
echo "[run] 残留检查通过（无 gzserver、端口 $PORT 空闲）loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"

# ---------------- 起栈 ----------------
ARGS=(world:=$WORLD mode:=nav lio:=$LIO localization:=$LOC nav:=$NAV planner:=$PLANNER
      nav_rviz:=False lio_rviz:=False)
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

# ---------------- 记录仪（自己等链路 + 发目标 + 收尾发零速） ----------------
PARGS=(--tag "$TAG" --out "$OUT/samples.jsonl" --summary "$OUT/summary.json"
       --settle "$SETTLE" --ready-timeout "$READY" --max-drive-sec "$MAXDRIVE"
       --post-goal-sec "$POST")
[ -n "$GOAL" ] && PARGS+=(--goal $GOAL)
echo "[run] 记录仪：python3 tools/scripts/regress/nav_clearance_probe.py ${PARGS[*]}"
python3 -u "$WS/tools/scripts/regress/nav_clearance_probe.py" "${PARGS[@]}" >"$OUT/probe.log" 2>&1
WRC=$?
echo "[run] 记录仪 rc=$WRC"; tail -25 "$OUT/probe.log"

# ---------------- 摘录：拒帧/失效/导航结局（带墙钟时间戳 ⇒ 可与样本时间轴对齐） ----------------
{
  echo "########## GICP 拒帧 / 失效 / 采纳（带墙钟时间戳） ##########"
  grep -aE "GICP 本帧未被采纳|连续 .* 帧被拒绝|定位已判失效|用参数 initial_pose 初始化" "$LOG" | head -400 || echo "(无)"
  echo; echo "########## [status] 行（每 ~1 s 一条：采纳/拒绝计数） ##########"
  grep -a "\[status\]" "$LOG" | head -400 || echo "(无)"
  echo; echo "########## 导航结局 / recovery / 断点 ##########"
  grep -aE "Reached the goal|Goal succeeded|Goal failed|Aborting handle|Failed to make progress|Running recovery|clearing costmap|Clearing costmap|extrapolation|Invalid frame ID" "$LOG" | head -200 || echo "(无)"
} > "$OUT/gate.txt" 2>&1
python3 - "$OUT" <<'PY'
import json, os, sys
out = sys.argv[1]
s = json.load(open(os.path.join(out, 'summary.json')))
p = os.path.join(out, 'plans.jsonl')
with open(p, 'w') as f:
    for e in s.get('plans', []):
        f.write(json.dumps(e, ensure_ascii=False) + '\n')
print('[run] plans=%d contact=%d blocked=%d pushed=%d mo_stale=%d goal=%s travel=%.2fm'
      % (len(s.get('plans', [])), s.get('contact_events', 0), len(s.get('blocked', [])),
         len(s.get('pushed', [])), len(s.get('mo_stale_runs', [])), s.get('goal_status'),
         s.get('travel_gt', 0.0)))
PY

echo "[run] DONE tag=$TAG rc_probe=$WRC out=$OUT"
exit $WRC
