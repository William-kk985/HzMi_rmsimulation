#!/usr/bin/env bash
# =============================================================================
# 建图 A/B 单跑：一套隔离的无头跑图 + 同时间轴旁路探针 + 丢帧统计 + 存图
#
# 干什么：
#   1) 按命令行**临时**改 1~3 个上游参数（改完写进 trap，跑完逐字节还原）；
#   2) 无头起 `mode:=mapping lio:=small_point_lio mapper:=slam_toolbox`；
#   3) 并行跑 `ramp_section_probe.py`（跳变/打滑/卡死//scan 内容）和
#      `map_odom_jump_gate.py`（单步跳变哨兵，latched 状态话题）；
#   4) `coverage_drive.py` 走给定路线，可选存 2D 图；
#   5) 从 launch 日志里数 `Message Filter dropping`（tf2 MessageFilter 丢帧），
#      给出 **drops/s** 与 **drop 比例**（分母 = 探针实际收到的 /scan 条数），
#      并把探针 + 哨兵的 JSON 汇总成一份 `<tag>.summary.json`。
#
# 为什么必须"全塞在一次 bash 调用里"：本仓库每条 bash 命令跑在自己的 PID namespace，
#   调用一结束里面的进程（含 gzserver）全被清掉（见 run_mapping_headless.sh 头注）。
#
# 用法（例：丢帧 A/B 的 4 次跑之一）
#   bash tools/scripts/mapping/mapping_ab_run.sh \
#     --tag base --domain 97 --port 11507 --route /tmp/ramp_route.json \
#     --out-dir .tmp_cache/ab_scan/base --speed 0.20 --timeout 300 \
#     --save-2d .tmp_cache/ab_scan/base/map
#   # 其它变体：
#   ... --p2l-target-frame base_link
#   ... --slam-transform-timeout 0.5
#   ... --slam-min-time-interval 0.2
#
# 产物（都在 --out-dir 下）：
#   <tag>.log              launch 全量日志（丢帧行在这里）
#   <tag>.drops.txt        丢帧扫描时间戳 + 速率
#   <tag>.probe.json/.npz  旁路探针
#   <tag>.gate.json        跳变哨兵的增量分布 + 跳变明细
#   <tag>.drive.json       coverage_drive 的体检
#   <tag>.summary.json     汇总（本脚本生成，报告直接引用）
# =============================================================================
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TAG=""; OUT_DIR=""; ROUTE=""; SAVE_2D=""; WORLD="RMUC2026"; LIO="small_point_lio"
DOMAIN=97; PORT=11507; SPEED=0.20; TIMEOUT=300; WARMUP=15; POSE_SOURCE=lio
P2L_TF=""; SLAM_TT=""; SLAM_MTI=""
JUMP_T="0.20"; JUMP_R="8.0"; EXTRA_DRIVER=""
P2L_CFG="$WS/src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml"
SLAM_CFG="$WS/src/rm_localization/slam_toolbox/config/mapper_params_online_async_sim.yaml"

while [ $# -gt 0 ]; do
  case "$1" in
    --tag) TAG="$2"; shift 2;;
    --out-dir) OUT_DIR="$2"; shift 2;;
    --route) ROUTE="$2"; shift 2;;
    --save-2d) SAVE_2D="$2"; shift 2;;
    --world) WORLD="$2"; shift 2;;
    --lio) LIO="$2"; shift 2;;
    --domain) DOMAIN="$2"; shift 2;;
    --port) PORT="$2"; shift 2;;
    --speed) SPEED="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --warmup) WARMUP="$2"; shift 2;;
    --pose-source) POSE_SOURCE="$2"; shift 2;;
    --p2l-target-frame) P2L_TF="$2"; shift 2;;
    --slam-transform-timeout) SLAM_TT="$2"; shift 2;;
    --slam-min-time-interval) SLAM_MTI="$2"; shift 2;;
    --jump-threshold-trans) JUMP_T="$2"; shift 2;;
    --jump-threshold-rot-deg) JUMP_R="$2"; shift 2;;
    --driver-args) EXTRA_DRIVER="$2"; shift 2;;
    -h|--help) sed -n '2,42p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
[ -n "$TAG" ] || { echo "必须给 --tag"; exit 2; }
[ -n "$OUT_DIR" ] || OUT_DIR="$WS/.tmp_cache/ab_scan/$TAG"
[ -n "$ROUTE" ] || { echo "必须给 --route（route.json）"; exit 2; }
mkdir -p "$OUT_DIR" "$(dirname "$ROUTE")" "/tmp/gzhome-$TAG"

LOG="$OUT_DIR/$TAG.log"
SUM="$OUT_DIR/$TAG.summary.json"
BK="$OUT_DIR/cfg_backup"; mkdir -p "$BK"

# ---------------------------------------------------------------- 跑前先确保"基线是干净的"
# 上一次跑如果被强杀（trap 没跑到），改过的参数会留在工作区 ⇒ 下一次跑的"基线"就脏了。
# 这里**只检查不改**：与 git HEAD 不一致就直接拒绝跑（要改请自己改回来），
# 避免出现"以为在跑基线、其实在跑别人的变体"这种不可追溯的结果。
cd "$WS" || exit 2
for f in "$P2L_CFG" "$SLAM_CFG"; do
  if ! git diff --quiet -- "$f"; then
    echo "[ab] ❌ $f 与 git HEAD 不一致（上一次跑被强杀，没还原）。请先：" >&2
    echo "     git checkout -- $f" >&2
    exit 3
  fi
done
cd - >/dev/null

# ---------------------------------------------------------------- 参数改写 + 还原
cp -f "$P2L_CFG" "$BK/laserscan_params.yaml"
cp -f "$SLAM_CFG" "$BK/mapper_params_online_async_sim.yaml"
restore() {
  cp -f "$BK/laserscan_params.yaml" "$P2L_CFG"
  cp -f "$BK/mapper_params_online_async_sim.yaml" "$SLAM_CFG"
  echo "[ab] 配置已还原（与跑前逐字节相同：$(cmp -s "$BK/laserscan_params.yaml" "$P2L_CFG" && echo yes || echo NO) / $(cmp -s "$BK/mapper_params_online_async_sim.yaml" "$SLAM_CFG" && echo yes || echo NO)）"
}
cleanup() {
  echo "[ab] ---- cleanup ----"
  kill -INT -- "-$PGID" 2>/dev/null
  pkill -f 'ramp_section_probe|map_odom_jump_gate' 2>/dev/null
  for _ in $(seq 1 15); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
  kill -9 -- "-$PGID" 2>/dev/null
  pkill -9 -f 'ros2 launch' 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f 'small_point_lio_node|fastlio_mapping|pointlio|slam_toolbox|map_saver' 2>/dev/null
  pkill -9 -f 'ground_segmentation_node|pointcloud_to_laserscan_node|spawn_entity.py|robot_state_publisher|joint_state_publisher|complementary_filter|fake_vel_transform' 2>/dev/null
  sleep 2
  restore
}
VARIANTS="baseline"
if [ -n "$P2L_TF" ]; then
  python3 - "$P2L_CFG" "$P2L_TF" <<'PY'
import re,sys
p,v=sys.argv[1],sys.argv[2]
s=open(p).read()
s2,n=re.subn(r'(?m)^(\s*target_frame:\s*).*$', lambda m: m.group(1)+'"%s"'%v, s, count=1)
assert n==1, 'target_frame 没找到'
open(p,'w').write(s2)
PY
  VARIANTS="$VARIANTS,p2l_target_frame=$P2L_TF"
fi
if [ -n "$SLAM_TT" ]; then
  python3 - "$SLAM_CFG" "$SLAM_TT" <<'PY'
import re,sys
p,v=sys.argv[1],sys.argv[2]
s=open(p).read()
s2,n=re.subn(r'(?m)^(\s*transform_timeout:\s*).*$', lambda m: m.group(1)+v, s, count=1)
assert n==1,'transform_timeout 没找到'
open(p,'w').write(s2)
PY
  VARIANTS="$VARIANTS,slam_transform_timeout=$SLAM_TT"
fi
if [ -n "$SLAM_MTI" ]; then
  python3 - "$SLAM_CFG" "$SLAM_MTI" <<'PY'
import re,sys
p,v=sys.argv[1],sys.argv[2]
s=open(p).read()
s2,n=re.subn(r'(?m)^(\s*minimum_time_interval:\s*).*$', lambda m: m.group(1)+v, s, count=1)
assert n==1,'minimum_time_interval 没找到'
open(p,'w').write(s2)
PY
  VARIANTS="$VARIANTS,slam_minimum_time_interval=$SLAM_MTI"
fi
echo "[ab] tag=$TAG variants=$VARIANTS"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export HOME="/tmp/gzhome-$TAG"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOG_DIR="$OUT_DIR/roslog_$TAG"
export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
export RCUTILS_COLORIZED_OUTPUT=0
export MPLCONFIGDIR="/tmp/mpl-$TAG"
mkdir -p "$ROS_LOG_DIR"
unset DISPLAY
cd "$WS" || exit 2
: > "$LOG"
trap cleanup EXIT

echo "[ab] 生效值：$(grep -m1 target_frame "$P2L_CFG" | tr -d ' ') $(grep -m1 transform_timeout "$SLAM_CFG" | tr -d ' ') $(grep -m1 minimum_time_interval "$SLAM_CFG" | tr -d ' ')"

ARGS=(world:=$WORLD mode:=mapping lio:=$LIO mapper:=slam_toolbox
      nav_rviz:=False lio_rviz:=False spin_speed:=0.0)
echo "[ab] ros2 launch rm_nav_bringup bringup_sim.launch.py ${ARGS[*]}" | tee -a "$LOG"
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py "${ARGS[@]}" >>"$LOG" 2>&1 &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
echo "[ab] launch pid=$LPID pgid=$PGID"

ok=0
for i in $(seq 1 75); do
  sleep 2
  kill -0 "$LPID" 2>/dev/null || { echo "[ab] ❌ launch 退出"; tail -30 "$LOG"; break; }
  if timeout 8 ros2 topic list 2>/dev/null | grep -qx '/clock'; then ok=1; break; fi
done
echo "[ab] /clock 就绪=$ok（≈$((i*2)) s）"
[ "$ok" = 1 ] || { echo "[ab] ❌ Gazebo 没起来"; exit 1; }

echo "[ab] 等链路 25 s ..."
sleep 25
grep -a "使用的参数文件" "$LOG" | tail -1 || true
timeout 20 ros2 topic info /scan 2>&1 | grep -E "Publisher count|frame" | head -3 || true
echo "[ab] /scan header.frame_id = $(python3 - <<'PY'
import subprocess,json
try:
    o=subprocess.run(['ros2','topic','echo','/scan','--once','--field','header.frame_id'],
                     capture_output=True,text=True,timeout=20).stdout.strip()
    print(o or '?')
except Exception as e: print('?')
PY
)"

OUT="$OUT_DIR/$TAG"
# 旁路探针 + 跳变哨兵（与开跑并行）
# ⚠️ 外层 GNU timeout 必须比探针**自己的** --duration 宽松 ≥ 60 s：
#    探针到点后还要把 scan_ranges/scan_meta 压成 npz（几十 MB，要 1~3 s），
#    而它的 duration 是从 rclpy.init() 之后才开始计的。第一版只留 30 s 余量，
#    结果 SIGTERM 正好落在落盘中间 ⇒ JSON/NPZ 全丢、现象像"探针挂了"。
timeout -k 5 $((TIMEOUT + WARMUP + 150)) python3 -u tools/scripts/mapping/ramp_section_probe.py --out "$OUT.probe" \
  --duration $((TIMEOUT + WARMUP + 40)) >>"$OUT.probe.log" 2>&1 &
PPID_=$!
timeout -k 5 $((TIMEOUT + WARMUP + 130)) python3 -u tools/scripts/diag/map_odom_jump_gate.py --report-file "$OUT.gate.json" \
  --threshold-trans "$JUMP_T" --threshold-rot-deg "$JUMP_R" \
  --duration $((TIMEOUT + WARMUP + 30)) >"$OUT.gate.log" 2>&1 &
GPID_=$!
sleep 8

DRV=(python3 -u tools/scripts/mapping/coverage_drive.py
     --route "$ROUTE" --out-json "$OUT.drive.json"
     --speed "$SPEED" --timeout "$TIMEOUT" --warmup "$WARMUP"
     --pose-source "$POSE_SOURCE")
[ -n "$SAVE_2D" ] && DRV+=(--save-2d "$SAVE_2D")
[ -n "$EXTRA_DRIVER" ] && DRV+=($EXTRA_DRIVER)
echo "[ab] 跑图：${DRV[*]}"
timeout $((TIMEOUT + 300)) "${DRV[@]}" 2>&1 | tee "$OUT.drive.log"
DRVRC=$?
echo "[ab] coverage_drive rc=$DRVRC"

wait $PPID_ 2>/dev/null; wait $GPID_ 2>/dev/null
grep -a "PROBEJSON" "$OUT.probe.log" | tail -1 || echo "PROBEJSON {}"
grep -a "PROBESTUCK" "$OUT.probe.log" | tail -1 || echo "PROBESTUCK []"

# ---------------------------------------------------------------- 丢帧统计
grep -a "Message Filter dropping" "$LOG" | sed -n 's/.*at time \([0-9.]*\).*/\1/p' > "$OUT.drops.txt" || true
NDROP=$(wc -l < "$OUT.drops.txt" | tr -d ' ')
echo "[ab] 丢帧条数=$NDROP（明细 $OUT.drops.txt）"
head -3 "$OUT.drops.txt"; echo "  ..."; tail -3 "$OUT.drops.txt"
timeout 20 ros2 topic hz /scan 2>&1 | grep -m1 "average rate" || true

# ---------------------------------------------------------------- 汇总
python3 - "$OUT" "$TAG" "$VARIANTS" "$NDROP" "$DRVRC" "$JUMP_T" "$JUMP_R" <<'PY' | tee "$SUM"
import json,sys,os,re,numpy as np
out,tag,variants,ndrop,drvrc,jt,jr = sys.argv[1:9]
def j(p, d=None):
    try:
        return json.load(open(p))
    except Exception:
        return d
drops=[float(x) for x in open(out+'.drops.txt') if x.strip()] if os.path.exists(out+'.drops.txt') else []
probe=j(out+'.probe.json',{}) or {}
gate=j(out+'.gate.json',{}) or {}
drive=j(out+'.drive.json',{}) or {}
scan=probe.get('scan',{}) or {}
# 丢帧窗口 = 探针收到的 /scan 的仿真时间跨度（分母同一把尺子）
n_scan=scan.get('n',0)
sp=scan.get('sample_period_p99')
sim_span=None
if n_scan and sp: sim_span=round(n_scan*sp,1)
dps = round(len(drops)/sim_span,4) if sim_span else None
scan_hz = round(1.0/sp,2) if sp else None
frac = round(dps/scan_hz*100,2) if (dps and scan_hz) else None
summary={
 'tag':tag,'variants':variants,
 'drops':{'n':int(ndrop),'drops_per_sim_s':dps,'sim_span_s':sim_span,
          'scan_pub_hz':scan_hz,'drop_fraction_pct':frac,
          'scan_stamps_first':drops[:3],'scan_stamps_last':drops[-3:]},
 'scan':scan,'probe':{k:v for k,v in probe.items() if k!='stuck_windows'},
 'stuck_windows':probe.get('stuck_windows',[])[:10],
 'gate':{k:v for k,v in gate.items() if k!='jump_details'},
 'gate_jumps':gate.get('jump_details',[])[:10],
 'drive_rc':int(drvrc),
 'drive':{k:v for k,v in drive.items() if not isinstance(v,(list,dict))} if drive else {},
}
json.dump(summary,open(out+'.summary.json','w'),ensure_ascii=False,indent=1)
print(json.dumps(summary,ensure_ascii=False))
PY
echo "[ab] DONE tag=$TAG out=$OUT_DIR"
