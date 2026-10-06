#!/usr/bin/env bash
# =============================================================================
# LIO 漂移诊断「一条龙」：起建图栈（从零 or 续建指定存档）→ 分段脚本化路线 →
#                          50 Hz 同步采样 → 离线分析（分段 ATE / 跳变 / 相关性 / PNG）
#
# 为什么必须**全部塞进一次 bash 调用**：本仓库每条 bash 命令跑在自己的 PID namespace
# （bwrap --unshare-pid --die-with-parent），调用一结束命名空间里的进程全被清掉 ⇒
# 启动、驱动、收尾必须是同一条命令。见 docs/smoke_test_runbook.md。
#
# 隔离（与既有跑法一致）：HOME=/tmp/gzhome-<tag>（沙箱里 $HOME 只读，gzserver 建
#   ~/.gazebo 会 SIGABRT）、非默认 ROS_DOMAIN_ID、独立 GAZEBO_MASTER_URI、unset DISPLAY。
#
# 用法：
#   bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag spl \
#        --domain 94 --port 11540
#   # 对照组（仓库默认 LIO）
#   bash tools/scripts/diag/run_drift_diag.sh --lio fastlio --tag fastlio \
#        --domain 94 --port 11541
#   # 续建（污染存档复现）：把 --resume-from 指的存档**复制**成 map_name 那份再续建
#   bash tools/scripts/diag/run_drift_diag.sh --lio small_point_lio --tag resume \
#        --domain 94 --port 11542 --map-name RMUC2026_diagresume \
#        --resume-from src/rm_nav_bringup/map/RMUC2026_cont
#
# 产物（路径都会打印）：
#   <out-dir>/<tag>.log            launch 全量日志
#   <out-dir>/<tag>.csv            50 Hz 时间轴
#   <out-dir>/<tag>.json           事件/元数据
#   <out-dir>/<tag>.analysis.json  分析结果（分段表 / 跳变 / 相关性）
#   docs/img/<tag>_*.png           三张图
#
# ⚠️ 本脚本**不写任何用户存档**：`--map-name` 默认就是测试专用的 RMUC2026_diag；
#    `--resume-from` 只做**复制**（cp），从不改动源文件。
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LIO="small_point_lio"; TAG=""; WORLD="RMUC2026"; MAPPER="slam_toolbox"
MAP_NAME="RMUC2026_diag"; DOMAIN=94; PORT=11540
OUT_DIR=""; RESUME_FROM=""; IMG_DIR="docs/img"
AUTOCONTINUE_FORCE=""; SKIP_ANALYZE=0; NO_FIGS=0
DRIVER_EXTRA=""; ANALYZE_EXTRA=""

while [ $# -gt 0 ]; do
  case "$1" in
    --lio) LIO="$2"; shift 2;;
    --tag) TAG="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --mapper) MAPPER="$2"; shift 2;;
    --map-name) MAP_NAME="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --img-dir) IMG_DIR="$2"; shift 2;;
    --resume-from) RESUME_FROM="$2"; shift 2;;
    --driver-args) DRIVER_EXTRA="$2"; shift 2;;
    --analyze-args) ANALYZE_EXTRA="$2"; shift 2;;
    --skip-analyze) SKIP_ANALYZE=1; shift;;
    --no-figs) NO_FIGS=1; shift;;
    -h|--help) sed -n '2,42p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || TAG="$LIO"
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_cache/diag/$TAG"
MAP_DIR="$WS/src/rm_nav_bringup/map"
mkdir -p "$OUT_DIR" "$MAP_DIR" "$WS/$IMG_DIR" "/tmp/gzhome-diag$TAG"

LOG="$OUT_DIR/$TAG.log"
CSV="$OUT_DIR/$TAG.csv"
JSON="$OUT_DIR/$TAG.json"
ANA="$OUT_DIR/$TAG.analysis.json"

# ---- 会话状态文件：launch 每次会覆写它（map_archive.sh save 靠它认名字）。
#      本诊断跑完把用户原来那份还原回去，免得把别人的流程带偏。
SESSION="$MAP_DIR/.session.yaml"
SESSION_BAK="/tmp/gzhome-diag$TAG/.session.yaml.bak"
[ -f "$SESSION" ] && cp -f "$SESSION" "$SESSION_BAK" || true

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-diag$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$OUT_DIR/roslog_$TAG"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR=/tmp/mpl-diag$TAG
mkdir -p "$ROS_LOG_DIR"
unset DISPLAY
cd "$WS" || exit 2

# ---------------------------------------------------------------- 续建准备（只复制）
if [ -n "$RESUME_FROM" ]; then
  echo "[run] 续建：$RESUME_FROM.* → $MAP_DIR/$MAP_NAME.*（**只复制，不动源文件**）"
  for ext in posegraph data; do
    if [ ! -f "$RESUME_FROM.$ext" ]; then echo "[run] ❌ 缺 $RESUME_FROM.$ext"; exit 2; fi
    cp -f "$RESUME_FROM.$ext" "$MAP_DIR/$MAP_NAME.$ext"
  done
  # 同名 sidecar：字段照抄源存档，只改 name/files（守卫按 world/spawn/resolution 判合规）
  python3 - "$RESUME_FROM.meta.yaml" "$MAP_DIR/$MAP_NAME.meta.yaml" "$MAP_NAME" <<'PY'
import sys, re
src, dst, name = sys.argv[1], sys.argv[2], sys.argv[3]
txt = open(src).read()
txt = re.sub(r'^name:.*$', 'name: %s' % name, txt, count=1, flags=re.M)
txt = re.sub(r'^files:.*?(?=^\S)', 'files:\n- %s.posegraph\n- %s.data\n- %s.meta.yaml\n'
             % (name, name, name), txt, count=1, flags=re.M | re.S)
txt += '\n# 本文件由 tools/scripts/diag/run_drift_diag.sh 生成（诊断用副本，非用户存档）\n'
open(dst, 'w').write(txt)
print('[run] sidecar → %s' % dst)
PY
fi

# ---------------------------------------------------------------- launch
ARGS=(world:=$WORLD mode:=mapping lio:=$LIO mapper:=$MAPPER
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0 map_name:=$MAP_NAME)
[ -n "$AUTOCONTINUE_FORCE" ] && ARGS+=(map_autocontinue:=$AUTOCONTINUE_FORCE)
echo "[run] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee "$LOG"
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
  pkill -9 -f 'small_point_lio_node|fastlio_mapping|pointlio|lio_tf_adapter|slam_toolbox' 2>/dev/null
  pkill -9 -f 'ground_segmentation_node|pointcloud_to_laserscan_node|spawn_entity.py|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform' 2>/dev/null
  sleep 2
  echo "[run] 残留复查:"; pgrep -af 'gzserver|ros2 launch|slam_toolbox|small_point_lio|fastlio' || echo "  （无残留）"
  if [ -f "$SESSION_BAK" ]; then cp -f "$SESSION_BAK" "$SESSION"; echo "[run] 已还原 $SESSION"; fi
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
if [ "$ok" != 1 ]; then echo "[run] ❌ Gazebo/launch 没起来"; tail -40 "$LOG"; exit 1; fi

if grep -qa "Entity \[robot\] already exists" "$LOG" || grep -qa "gzserver-5\]: process has died" "$LOG"; then
  echo "[run] ❌ 检测到 gzserver 启动失败 / Entity [robot] already exists"
  echo "[run]    ⇒ 上一轮的 gzserver 没死干净，本跑会量到上一台车。已中止（换 --port / --domain 重跑）。"
  grep -a -E "already exists|gzserver-5\]: process has died" "$LOG" | head -5
  exit 1
fi
echo "[run] 等链路稳定 30 s ..."
sleep 30
echo "[run] --- 建图栈横幅（证明读的是哪份存档 / 哪个 world）---"
grep -a "map_archive\|resumed\|存档\|lio:=.*使用的参数文件" "$LOG" | tail -12
echo "[run] --- 关键话题 ---"
timeout 20 ros2 topic list 2>/dev/null | sort > "$OUT_DIR/$TAG.topics.txt"
for t in /livox/lidar /scan /odom /odom_ground_truth /map /cmd_vel_chassis /clock; do
  grep -qx "$t" "$OUT_DIR/$TAG.topics.txt" && echo "[run] ✅ $t" || echo "[run] ❌ 缺话题 $t"
done

# ---------------------------------------------------------------- 驱动
DRV=(python3 -u tools/scripts/diag/segment_drive.py
     --out-csv "$CSV" --out-json "$JSON"
     --pose-source gt --warmup 10 --timeout 420)
[ -n "$DRIVER_EXTRA" ] && DRV+=($DRIVER_EXTRA)
echo "[run] --- 分段跑图：${DRV[*]} ---" | tee -a "$LOG"
python3 -u tools/scripts/diag/segment_drive.py --dry-run | tee -a "$LOG"
timeout 700 "${DRV[@]}" 2>&1 | tee "$OUT_DIR/$TAG.drive.log"
DRC=$?
echo "[run] segment_drive 退出码=$DRC"

echo "[run] --- 日志关键字（ERROR/WARN/diverge/message filter）---"
grep -a -iE "error|warn|diverge|nan|invalid|message filter" "$LOG" | grep -v "RCUTILS" \
  | grep -v "gzclient" | head -30 | tee "$OUT_DIR/$TAG.warnings.txt"

# ---------------------------------------------------------------- 收尾：先收栈（释放 CPU）再分析
cleanup
trap - EXIT

if [ "$SKIP_ANALYZE" = 1 ]; then echo "[run] --skip-analyze，分析跳过"; exit $DRC; fi
ANA_ARGS=(--csv "$CSV" --events "$JSON" --out-json "$ANA" --out-dir "$IMG_DIR" --tag "$TAG")
[ "$NO_FIGS" = 1 ] && ANA_ARGS+=(--no-figs)
[ -n "$ANALYZE_EXTRA" ] && ANA_ARGS+=($ANALYZE_EXTRA)
echo "[run] --- 分析：${ANA_ARGS[*]} ---"
python3 -u tools/scripts/diag/lio_drift_analyze.py "${ANA_ARGS[@]}" 2>&1 | tee "$OUT_DIR/$TAG.analysis.txt"
echo "[run] DONE $(date '+%F %T')  out-dir=$OUT_DIR"
exit $DRC
