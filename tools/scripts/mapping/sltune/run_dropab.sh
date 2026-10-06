#!/usr/bin/env bash
# =============================================================================
# 「slam_toolbox 丢帧 vs 回环 / 里程计」单跑装置（A/B 的一格）
#
# 干什么（一次调用 = 起栈 → 记录仪 → 跑路线 → 收尾 → 落证据）：
#   1) 按 --set 把 slam_toolbox 的 yaml **临时**改成被测变体（trap 里逐字节还原）；跑前若
#      该文件与 git HEAD 不一致就直接拒绝跑（避免"以为在跑基线、其实在跑别人的变体"）；
#   2) 无头起 `world:=RMUC2026 mode:=mapping mapper:=slam_toolbox`，lio 槽位可选：
#        --odom lio   （默认）lio:=small_point_lio —— LIO 自己发 odom→base_link（现状）
#        --odom truth lio:=none + truth_odom_tf_bridge.py —— 真值当里程计（**只有它一个发布者**）
#      这是 §4「里程计上界」那一组；`--odom-delay-s` 再把 TF 晚发一点，用来把
#      "精度"与"TF 到得晚"两个因素拆开；
#   3) 并行两个**只读**记录仪（不发任何话题、不做监控/哨兵）：
#        · tools/scripts/diag/scan_tf_timing_probe.py  —— /scan 到达、TF 何时可查、等待时长、
#          被真正处理的帧(/pose)、/map 增长、RTF、逐纳秒戳差（写 .timing.json/.npz）
#        · tools/scripts/mapping/sltune/rec_sltune.py  —— 位姿三件套、位姿图 V/E(环秩)、
#          各进程 CPU%、RTF（写 .rec.jsonl）
#   4) `coverage_drive.py` 走路线（--pose-source gt：几何形状不受 LIO 抖动影响，四格可比）；
#   5) 收尾：零速、序列化位姿图（回环约束的权威证据）、拷 slam_toolbox 节点日志、
#      数 `queue is full` 丢帧、杀干净（含残留 gzserver 自检）。
#
# 为什么必须"全塞在一次 bash 调用里"：本环境每条命令一个 PID namespace，调用结束即清进程。
#
# 用法（例：四格 A/B 中的一格）
#   bash tools/scripts/mapping/sltune/run_dropab.sh a_dense \
#     --route tools/scripts/mapping/sltune/loop_route_corridor_x2.json --timeout 500
#   ... a_dense --set minimum_time_interval=0.3
#   ... a_dense --set scan_queue_size=2
#   ... b_coarse --set minimum_travel_distance=0.5 --set minimum_travel_heading=0.5 \
#                --set minimum_time_interval=0.5
#   ... e1_truth --odom truth --odom-delay-s 0
#   ... e2_truth_delay --odom truth --odom-delay-s 0.13
# 产物（都在 .tmp_cache/dropab/<tag>/ 下）：
#   launch.log  cfg_diff.txt  params.txt  topic_list.txt  service_list.txt
#   <tag>.timing.json/.npz（时间戳链探针）  <tag>.rec.jsonl（位姿图/CPU）
#   <tag>.drive.json（跑图体检）  <tag>.drops.txt（丢帧扫描戳）  <tag>.ssl.log（节点日志）
# =============================================================================
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
TAG="${1:?用法: run_dropab.sh <tag> [--set k=v ...] [--route r.json] [--odom lio|truth]}"; shift
CFG_DST="$WS/src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml"
OUT="$WS/.tmp_cache/dropab/$TAG"
CFG_ORIG="$OUT/cfg_orig.yaml"
ROUTE="$WS/tools/scripts/mapping/sltune/loop_route_corridor_x2.json"
DOMAIN="${DROPAB_DOMAIN:-142}"; PORT="${DROPAB_PORT:-11942}"
WORLD="RMUC2026"; SPEED=0.30; TIMEOUT=500; WARMUP=10; LOOKAHEAD=0.6; ROTIN=1.2
MAPNAME="RMUC2026_dropab_$TAG"; SETS=(); ODOM="lio"; ODELAY=0.0; RESTORE=1
PROBE_DUR=1500
# 出生点自检的期望值：默认 = RMUC2026 世界的出生点（与 segment_drive.py 的 --expect-spawn 同口径）
EXP_SPAWN="10.925 2.525 1.5"
while [ $# -gt 0 ]; do
  case "$1" in
    --set) SETS+=("$2"); shift 2;;
    --route) ROUTE="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --lookahead) LOOKAHEAD="$2"; shift 2;;
    --rot-in-place-rad) ROTIN="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --warmup) WARMUP="$2"; shift 2;;
    --map-name) MAPNAME="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --odom) ODOM="$2"; shift 2;;
    --odom-delay-s) ODELAY="$2"; shift 2;;
    --probe-dur) PROBE_DUR="$2"; shift 2;;
    --expect-spawn) EXP_SPAWN="$2"; shift 2;;
    --no-restore) RESTORE=0; shift;;
    -h|--help) sed -n '2,47p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
case "$ODOM" in lio|truth) ;; *) echo "--odom 只能是 lio|truth"; exit 2;; esac
mkdir -p "$OUT" "/tmp/gzhome-$TAG" "$OUT/roslog"
cd "$WS" || exit 2
# ---- 跑前基线必须干净（上次被强杀会留下改过的配置）
if ! git diff --quiet -- "$CFG_DST"; then
  echo "[dropab] ❌ $CFG_DST 与 git HEAD 不一致（上次跑没还原？）请先 git checkout -- 它" >&2
  exit 3
fi
cp -f "$CFG_DST" "$CFG_ORIG"
restore() {
  if [ "$RESTORE" = 1 ]; then cp -f "$CFG_ORIG" "$CFG_DST"; echo "[dropab] 已还原 $CFG_DST"; fi
}
trap restore EXIT

# ---- 装配置（副本上改，再覆盖过去；键不存在则插入一行）
python3 - "$CFG_ORIG" "$CFG_DST" "${SETS[@]:-}" <<'PY'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
sets = [a for a in sys.argv[3:] if a]
txt = open(src).read()
applied = []
for kv in sets:
    k, v = kv.split('=', 1)
    new, n = re.subn(r'(?m)^(\s*%s:\s*).*$' % re.escape(k), r'\g<1>%s' % v, txt)
    if n == 0:
        # 插入（scan_queue_size 这类默认不在本仓 yaml 里的键）：放在 minimum_time_interval 后
        new, n = re.subn(r'(?m)^(\s*minimum_time_interval:.*)$',
                         lambda m: m.group(1) + '\n    %s: %s' % (k, v), txt, count=1)
        if n == 0:
            sys.exit('❌ 配置里既没有这个键、也没有插入锚点: %s' % k)
        applied.append('%s=%s(插入)' % (k, v))
    else:
        applied.append('%s=%s' % (k, v))
    txt = new
open(dst, 'w').write(txt)
print('[cfg] 覆盖: %s' % (', '.join(applied) if applied else '（无，= 现状基线）'))
PY
[ $? -eq 0 ] || exit 2
diff -u "$CFG_ORIG" "$CFG_DST" > "$OUT/cfg_diff.txt" || true
cat "$OUT/cfg_diff.txt"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$OUT/roslog"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR="/tmp/mpl-$TAG"
unset DISPLAY
cd "$WS" || exit 2

LOG="$OUT/launch.log"; : > "$LOG"
LIO_SLOT=small_point_lio
[ "$ODOM" = truth ] && LIO_SLOT=none
ARGS=(world:=$WORLD mode:=mapping lio:=$LIO_SLOT mapper:=slam_toolbox
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0
      map_name:=$MAPNAME map_autocontinue:=False)
echo "[dropab] tag=$TAG odom=$ODOM(delay=$ODELAY) route=$ROUTE" | tee -a "$LOG"
echo "[dropab] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[dropab] launch pid=$LPID pgid=$PGID load=$(cut -d' ' -f1-3 /proc/loadavg)"
BRPID=""; RPID=""; PPID_=""
cleanup() {
  echo "[dropab] ---- cleanup ----"
  [ -n "$PPID_" ] && kill -INT "$PPID_" 2>/dev/null
  [ -n "$RPID" ] && kill -INT "$RPID" 2>/dev/null
  [ -n "$BRPID" ] && kill -INT "$BRPID" 2>/dev/null
  kill -INT -- "-$PGID" 2>/dev/null
  for _ in $(seq 1 12); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'slam_toolbox|small_point_lio_node|pointcloud_to_laserscan|ground_segmentation|spawn_entity|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform|truth_odom_tf_bridge|coverage_drive' 2>/dev/null
  sleep 2
  pgrep -af 'gzserver|slam_toolbox|ros2 launch|truth_odom' || echo "[dropab] （无残留）"
}
trap 'cleanup; restore' EXIT

ok=0
for i in $(seq 1 75); do
  sleep 2
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[dropab] ❌ launch 退出（见 $LOG）"; tail -25 "$LOG"; exit 1; fi
  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock'; then ok=1; break; fi
done
echo "[dropab] /clock 就绪=$ok（≈$((i*2)) s）"
[ "$ok" = 1 ] || { echo "[dropab] ❌ 没起来"; tail -40 "$LOG"; exit 1; }

# ---- 真值里程计桥（只有 --odom truth 时；启动前会自检 /tf 上没有 LIO 发布者）
if [ "$ODOM" = truth ]; then
  python3 -u tools/scripts/diag/truth_odom_tf_bridge.py --stats "$OUT/bridge.json" \
    --delay-s "$ODELAY" --duration $((PROBE_DUR)) >"$OUT/bridge.log" 2>&1 &
  BRPID=$!
  echo "[dropab] truth 桥 pid=$BRPID delay=${ODELAY}s"
fi

echo "[dropab] 等链路稳定 28 s ..."; sleep 28

# ---- 出生点自检（挡"上一轮 gzserver 没死干净 ⇒ 量到上一台车"）
timeout 15 ros2 topic echo /odom_ground_truth --once --field pose.pose.position \
  > "$OUT/spawn.txt" 2>&1 || true
python3 - "$OUT/spawn.txt" $EXP_SPAWN > "$OUT/spawn_check.txt" 2>&1 <<'PY'
import re, sys
try:
    t = open(sys.argv[1]).read()
except Exception:
    print('[dropab] ⚠️ 出生点读取失败（继续）'); raise SystemExit(0)
ex, ey, tol = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
m = re.findall(r'([-+0-9.eE]+)', t)
v = [float(x) for x in m[:2]] if len(m) >= 2 else []
if len(v) != 2:
    print('[dropab] ⚠️ 出生点解析不到（继续）'); raise SystemExit(0)
d = ((v[0] - ex) ** 2 + (v[1] - ey) ** 2) ** 0.5
if d > tol:
    print('[dropab] ❌ 出生点 (%.2f, %.2f) 离期望 (%.2f, %.2f) %.2f m > %.2f —— '
          '疑似残留 gzserver，本次跑的数据不可信' % (v[0], v[1], ex, ey, d, tol))
    raise SystemExit(4)
print('[dropab] 出生点 (%.2f, %.2f) ✅（期望 (%.2f, %.2f)，差 %.2f m）' % (v[0], v[1], ex, ey, d))
PY
SPAWN_RC=$?
cat "$OUT/spawn_check.txt" | tee -a "$LOG"
[ "$SPAWN_RC" -eq 4 ] && { echo "[dropab] ❌ 出生点自检失败"; exit 4; }

# ---- 运行期证据：2.6.10 到底暴露了什么
timeout 25 ros2 param list /slam_toolbox 2>/dev/null | sort > "$OUT/param_list.txt"
for k in mode do_loop_closing loop_match_minimum_chain_size loop_search_maximum_distance \
         minimum_travel_distance minimum_travel_heading minimum_time_interval max_laser_range \
         scan_queue_size restamp_tf transform_timeout odom_frame base_frame; do
  printf '%s = %s\n' "$k" "$(timeout 10 ros2 param get /slam_toolbox $k 2>&1 | tail -1)"
done | tee "$OUT/params.txt"
timeout 20 ros2 topic list 2>/dev/null | sort > "$OUT/topic_list.txt"
timeout 20 ros2 service list 2>/dev/null | sort > "$OUT/service_list.txt"
timeout 20 ros2 topic list -t 2>/dev/null | grep -i "slam_toolbox\|/pose\|loop" | tee "$OUT/ssl_topics.txt" || true
grep -a "使用的参数文件" "$LOG" | tail -2 || true

# ---- 两个只读记录仪
python3 -u tools/scripts/diag/scan_tf_timing_probe.py --out "$OUT/$TAG.timing" \
  --duration "$PROBE_DUR" >"$OUT/timing.log" 2>&1 &
PPID_=$!
python3 -u tools/scripts/mapping/sltune/rec_sltune.py --out "$OUT/$TAG.rec.jsonl" \
  --dur "$PROBE_DUR" --hz 10 --sys-hz 5 >"$OUT/rec.log" 2>&1 &
RPID=$!
sleep 5

# ---- 跑路线（pose-source gt：几何形状确定 ⇒ 四格可比）
python3 -u tools/scripts/mapping/coverage_drive.py \
  --route "$ROUTE" --out-json "$OUT/$TAG.drive.json" --speed "$SPEED" --timeout "$TIMEOUT" \
  --warmup "$WARMUP" --pose-source gt --final-spin 0 --lookahead "$LOOKAHEAD" \
  --rot-in-place-rad "$ROTIN" >"$OUT/drive.log" 2>&1
echo "[dropab] drive rc=$? （sim 用时见 drive.json）"
tail -12 "$OUT/drive.log"

# ---- 零速 + 静置
timeout 5 ros2 topic pub -r 10 -t 15 /cmd_vel_chassis geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
sleep 4

# ---- 存位姿图（回环约束的权威证据）+ 参数快照
timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$OUT/posegraph'}" > "$OUT/serialize.log" 2>&1
ls -la "$OUT"/posegraph.* >> "$OUT/serialize.log" 2>&1
timeout 20 ros2 param dump /slam_toolbox > "$OUT/param_dump.yaml" 2>/dev/null

# ---- 收记录仪（SIGINT ⇒ 探针自己落盘 npz/json）
kill -INT "$PPID_" 2>/dev/null; sleep 1
for _ in $(seq 1 20); do kill -0 "$PPID_" 2>/dev/null || break; sleep 2; done
kill -INT "$RPID" 2>/dev/null
[ -n "$BRPID" ] && kill -INT "$BRPID" 2>/dev/null
sleep 2

# ---- 丢帧证据：节点日志（干净，不含 rviz）+ launch 日志
cp -f "$ROS_LOG_DIR"/async_slam_toolbox_node_*.log "$OUT/$TAG.ssl.log" 2>/dev/null || true
grep -a "queue is full" "$OUT/$TAG.ssl.log" 2>/dev/null | sed -n 's/.*at time \([0-9.]*\).*/\1/p' > "$OUT/$TAG.drops.txt" || true
grep -ac "queue is full" "$LOG" > "$OUT/drops_launchlog.txt" 2>/dev/null || true
NDROP=$(wc -l < "$OUT/$TAG.drops.txt" | tr -d ' ')
NCERES=$(grep -ac "preprocessor.cc" "$LOG" || true)
echo "[dropab] DONE tag=$TAG drops=$NDROP ceres_lines=$NCERES out=$OUT"
