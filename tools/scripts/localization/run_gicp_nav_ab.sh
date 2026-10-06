#!/usr/bin/env bash
# =============================================================================
# gicp 定位「发散 + 抖动」A/B 跑法（**无头**、一条命令一次跑完）
#
# 为什么全部塞进一次 bash 调用：本环境每条 bash 命令跑在自己的 PID namespace
#   （bwrap --unshare-pid --die-with-parent）里，调用一结束，命名空间里的所有进程
#   （含 gzserver）被内核清掉 ⇒ 起栈 / 记录 / 收尾必须在同一条命令里。
#
# 隔离（与既有跑法一致）：HOME=/tmp/gzhome-<tag>（沙箱里 $HOME 只读，gzserver 建 ~/.gazebo
#   会 SIGABRT）、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI 端口、unset DISPLAY、nav_rviz:=False。
#
# 用法（例 = 复现用户那次失败跑）：
#   bash tools/scripts/localization/run_gicp_nav_ab.sh --tag a_baseline --domain 141 --port 11811 \
#        --goal -12.64 -0.31 --post-goal-sec 150
#
#   # 参数 A/B：只在这一次跑里覆盖 gicp 的若干参数（跑完**自动还原**配置文件）
#   bash tools/scripts/localization/run_gicp_nav_ab.sh --tag b_gate --domain 142 --port 11812 \
#        --goal -12.64 -0.31 --post-goal-sec 150 \
#        --gicp-kv "gate_enable: true" --gicp-kv "gate_trans_max_step: 0.08"
#
# 参数：
#   --tag           本次跑的标签（输出目录名）
#   --domain        非默认 ROS_DOMAIN_ID（每个 tag 一个，避免串台）
#   --port          独立 GAZEBO_MASTER_URI 端口
#   --goal X Y      目标点（map 系）
#   --localization  gicp | small_gicp（默认 gicp）
#   --nav / --planner / --lio / --world   槽位（默认 = 用户那次跑：mppi / smac2d / small_point_lio）
#   --extra        追加的 launch 参数（引号包住，例：--extra "spin_speed:=0.0"）
#   --gicp-kv K: V  gicp 参数覆盖（可重复；跑完还原；见 gicp_registration_sim.yaml）
#   --post-goal-sec 目标结束后继续录多久（**墙钟**秒，默认 150）
#   --settle        发目标前静置秒数（墙钟，默认 10）
#   --ready-timeout 等链路上限（墙钟秒，默认 300）
#   --no-goal       只起栈 + 记录，不发目标（用于静止基线）
#
# 产物（默认 .tmp_gicp_ab/out/<tag>/ 下）：
#   launch.log         launch 全量日志（gicp 的 [status] 行在里面）
#   watch.jsonl        逐帧记录（map→odom / odom→base_link / 真值 / cmd_vel / fitness）
#   watch.json         汇总（目标结果、事件、RTF、话题计数）
#   config.used.yaml   本次实际生效的 gicp 参数文件（含覆盖）
#   status.txt         [status] 行 + 所有 WARN/ERROR 摘录
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; DOMAIN=141; PORT=11811; GOAL="-12.64 -0.31"; LOC="gicp"; NAV="mppi"; PLANNER="smac2d"
LIO="small_point_lio"; WORLD="RMUC2026"; POST=150; SETTLE=10; READY=300; NO_GOAL=0
EXTRA=""; OUT_ROOT="$WS/.tmp_gicp_ab/out"; KVS=()
GI_CFG="$WS/src/rm_localization/gicp_registration/config/gicp_registration_sim.yaml"

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
    --post-goal-sec) POST="$2"; shift 2;;
    --settle) SETTLE="$2"; shift 2;;
    --ready-timeout) READY="$2"; shift 2;;
    --out-root) OUT_ROOT="$2"; shift 2;;
    --gicp-kv) KVS+=("$2"); shift 2;;
    --no-goal) NO_GOAL=1; shift;;
    -h|--help) sed -n '2,45p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }

OUT="$OUT_ROOT/$TAG"; LOGDIR="$OUT/roslog"
rm -rf "$OUT"; mkdir -p "$OUT" "$LOGDIR" "/tmp/gzhome-$TAG" "/tmp/mpl-$TAG"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$LOGDIR"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR="/tmp/mpl-$TAG"
unset DISPLAY
cd "$WS" || exit 2

# ---------------- 残留守卫（别人/上次的 gzserver 会抢 GAZEBO_MASTER_URI） ----------------
echo "[run] ==== 残留检查（tag=$TAG domain=$DOMAIN port=$PORT）$(date '+%F %T') ===="
LEFT=$(pgrep -a -x gzserver || true)
if [ -n "$LEFT" ]; then
  echo "[run] ⚠️ 发现有残留 gzserver："; echo "$LEFT"
  # 只杀"本命名空间里"的（本环境每条命令一个 PID namespace，跨调用看不见 ⇒ 这里的必然是残留）
  pkill -9 -x gzserver 2>/dev/null || true
  sleep 2
fi
if command -v ss >/dev/null 2>&1; then
  if ss -ltn 2>/dev/null | grep -q ":$PORT "; then echo "[run] ❌ 端口 $PORT 已被占用"; ss -ltnp 2>/dev/null | grep ":$PORT "; exit 3; fi
fi
echo "[run] 残留检查通过（无 gzserver、端口 $PORT 空闲）loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"

# ---------------- gicp 参数覆盖（跑完**必须还原**，否则会污染仓库工作区） ----------------
cp -p "$GI_CFG" "$OUT/config.orig.yaml"
restore_cfg() {
  if [ -f "$OUT/config.orig.yaml" ]; then cp -p "$OUT/config.orig.yaml" "$GI_CFG"; fi
}
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
}
trap cleanup EXIT

if [ ${#KVS[@]} -gt 0 ]; then
  echo "[run] 应用 gicp 覆盖：${KVS[*]}"
  python3 - "$GI_CFG" "${KVS[@]}" <<'PY'
import sys, yaml
path, kvs = sys.argv[1], sys.argv[2:]
d = yaml.safe_load(open(path))
# 根键是 "/gicp_registration"（带前导斜杠，ros__parameters 的限定名写法）⇒ 不硬编码，按内容找，
# 免得又一次"覆盖静默没生效、A/B 白跑"（2026-10-06 踩过：KeyError 被 shell 吞掉，跑出来的是默认值）。
root = next(v for v in d.values() if isinstance(v, dict) and 'ros__parameters' in v)
p = root['ros__parameters']
for kv in kvs:
    k, v = kv.split(':', 1)
    k, v = k.strip(), v.strip()
    try:
        v = yaml.safe_load(v)
    except Exception:
        pass
    p[k] = v
    print('  set %s = %r' % (k, v))
yaml.safe_dump(d, open(path, 'w'), allow_unicode=True, sort_keys=False)
PY
fi
cp -p "$GI_CFG" "$OUT/config.used.yaml"

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
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[run] ❌ launch 退出（见 $LOG）"; tail -30 "$LOG"; exit 1; fi
  timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock' && break
done
echo "[run] /clock 就绪（≈$((i*2)) s）"

# ---------------- 记录仪（自己等链路 + 可选发目标）----------------
WARGS=(--tag "$TAG" --out "$OUT/watch.jsonl" --summary "$OUT/watch.json"
       --settle "$SETTLE" --ready-timeout "$READY" --post-goal-sec "$POST")
# 无目标（静止基线）时：把 --post-goal-sec 当作**录制时长**用（没有目标，"目标后窗口"就是全程）
if [ "$NO_GOAL" = 1 ]; then WARGS+=(--post-goal-sec "$POST"); else WARGS+=(--goal $GOAL); fi
echo "[run] 记录仪：python3 tools/scripts/localization/gicp_nav_watch.py ${WARGS[*]}"
python3 -u "$WS/tools/scripts/localization/gicp_nav_watch.py" "${WARGS[@]}" >"$OUT/watch.log" 2>&1
WRC=$?
echo "[run] 记录仪 rc=$WRC"; tail -20 "$OUT/watch.log"

# ---------------- 摘录：gicp [status] 行 + 所有醒目日志 ----------------
{
  echo "########## gicp [status] 行（每 ~1 s 一条） ##########"
  grep -a "\[status\]" "$LOG" || echo "(无)"
  echo; echo "########## 未采纳 / 拒绝 / 定位失效 相关 WARN ##########"
  grep -aE "未被采纳|定位已失效|不发 TF|尚未有 map→odom|GICP align 抛异常|发散|拒绝" "$LOG" | tail -60 || echo "(无)"
  echo; echo "########## 导航结果 / 断点 ##########"
  grep -aE "Reached the goal|Goal succeeded|Goal failed|Aborting handle|Failed to make progress|extrapolation" "$LOG" | tail -30 || echo "(无)"
  echo; echo "########## 本节点启动 banner（前 40 行）##########"
  grep -a -A40 "gicp_registration 启动" "$LOG" | head -45 || echo "(无)"
} > "$OUT/status.txt" 2>&1

echo "[run] DONE tag=$TAG rc_watch=$WRC out=$OUT"
echo "[run] status 摘要："; sed -n '1,6p' "$OUT/status.txt"
exit $WRC
