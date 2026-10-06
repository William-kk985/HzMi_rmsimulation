#!/usr/bin/env bash
# =============================================================================
# 「丢帧 vs 回环 / 里程计」A/B 系列：一次调用跑完 4~6 格并出汇总（每格 = run_dropab.sh 一次）
#
# 用法：
#   bash tools/scripts/mapping/sltune/run_dropab_series.sh [tag_prefix] [--only a,b,c] [--timeout 450] [--skip-truth]
# 默认六格（全部同一条路线 loop_route_corridor_x2.json、同一速度 0.30、--pose-source gt）：
#   a_dense       现状（0.2/0.2/0.25）—— 预期回环触发
#   b_coarse      0.5/0.5/0.5 —— 对照组：门粗 ⇒ 基本不回环（用于"回环是不是丢帧的原因"）
#   c_q2          a + scan_queue_size=2（一次只动一个变量）
#   d_mti03       a + minimum_time_interval=0.3（一次只动一个变量）
#   e1_truth      a + 真值里程计（lio:=none + truth_odom_tf_bridge，延迟 0）—— 里程计上界
#   e2_truth_dly  a + 真值里程计 + TF 晚 0.13 s —— 把"精度"与"TF 到得晚"拆开
# 产物：.tmp_cache/dropab/<prefix>_<arm>/ + .tmp_cache/dropab/<prefix>_summary.json/.md/.png
# =============================================================================
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PREFIX="${1:-ab}"; shift || true
ONLY=""; TIMEOUT=450; SKIP_TRUTH=0; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --only) ONLY="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --skip-truth) SKIP_TRUTH=1; shift;;
    --set) EXTRA+=(--set "$2"); shift 2;;
    -h|--help) sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 0;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done
R="$WS/tools/scripts/mapping/sltune/run_dropab.sh"
ROUTE="$WS/tools/scripts/mapping/sltune/loop_route_corridor_x2.json"
ARMS=(a_dense b_coarse c_q2 d_mti03 e1_truth e2_truth_dly)
want() { [ -z "$ONLY" ] && return 0; case ",$ONLY," in *",$1,"*) return 0;; esac; return 1; }

run_arm() {  # name  extra-args...
  local name="$1"; shift
  echo "########## $name  $(date '+%T')  load=$(cut -d' ' -f1-3 /proc/loadavg)"
  bash "$R" "${PREFIX}_$name" --route "$ROUTE" --timeout "$TIMEOUT" "$@" 2>&1 | tail -18
  echo "########## $name done $(date '+%T')"
}

for a in "${ARMS[@]}"; do
  want "$a" || continue
  E=(); [ ${#EXTRA[@]} -gt 0 ] && E=("${EXTRA[@]}")
  case "$a" in
    a_dense)      run_arm "$a" "${E[@]}";;
    b_coarse)     run_arm "$a" --set minimum_travel_distance=0.5 --set minimum_travel_heading=0.5 \
                              --set minimum_time_interval=0.5 "${E[@]}";;
    c_q2)         run_arm "$a" --set scan_queue_size=2 "${E[@]}";;
    d_mti03)      run_arm "$a" --set minimum_time_interval=0.3 "${E[@]}";;
    e1_truth)     [ "$SKIP_TRUTH" = 1 ] && continue
                  run_arm "$a" --odom truth --odom-delay-s 0 "${E[@]}";;
    e2_truth_dly) [ "$SKIP_TRUTH" = 1 ] && continue
                  run_arm "$a" --odom truth --odom-delay-s 0.13 "${E[@]}";;
  esac
done

echo "########## 汇总 $(date '+%T')"
set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u
export MPLCONFIGDIR=/tmp/mpl-dropab; mkdir -p /tmp/mpl-dropab
ARGS=()
for a in "${ARMS[@]}"; do
  want "$a" || continue
  d="$WS/.tmp_cache/dropab/${PREFIX}_$a"
  [ -d "$d" ] && ARGS+=(--arm "${PREFIX}_$a=$d")
done
[ ${#ARGS[@]} -gt 0 ] || { echo "没有可分析的跑次"; exit 1; }
cd "$WS"
python3 tools/scripts/mapping/sltune/analyze_dropab.py "${ARGS[@]}" \
  --json ".tmp_cache/dropab/${PREFIX}_summary.json" \
  --md ".tmp_cache/dropab/${PREFIX}_summary.md" \
  --png "docs/img/dropab_${PREFIX}.png"
echo "########## ALL DONE $(date '+%T')"
