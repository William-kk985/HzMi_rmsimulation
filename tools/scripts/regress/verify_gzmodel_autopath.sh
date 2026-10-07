#!/usr/bin/env bash
# ============================================================================
# verify_gzmodel_autopath.sh —— 「`model://robot11/...` 自动解析」取证跑（2026-10-08）
#
# 目的：证明**裸命令**（不许在命令行前缀 `GAZEBO_MODEL_PATH=…`）就能让 robot11 的 12 个
#       mesh 被 gazebo 解析到，并且能回答"哪条机制（launch 侧 / 包 export 侧）在实际运行里干活"。
#
# 用法：
#   tools/scripts/regress/verify_gzmodel_autopath.sh <tag> [--settle N] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/regress/verify_gzmodel_autopath.sh obj1_plain --settle 50 -- \
#       world:=RMUC2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
#   tools/scripts/regress/verify_gzmodel_autopath.sh obj1_default --settle 40 -- \
#       world:=RMUC2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0
#
# 关键约定（与 run_robot11_mount_probe.sh 同款隔离）：
#   · **显式 `unset GAZEBO_MODEL_PATH`** —— 这就是"裸命令"的定义（不手工前缀环境变量）；
#   · `HOME=/tmp/gzhome-<tag>`、非默认 `ROS_DOMAIN_ID`、专用 `GAZEBO_MASTER_URI`、`unset DISPLAY`；
#   · `gui:=False`（无头，不依赖 Xvfb）；跑完只清**本 master URI** 上的 gzserver/gzclient。
# 输出：.tmp_verify/<tag>/{launch.log, gz_env.txt, nodes.txt, summary.txt}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: verify_gzmodel_autopath.sh <tag> [--settle N] -- [LAUNCH_ARGS...]}"
shift
SETTLE=50
PROBE_ARGS=(); LAUNCH_ARGS=(); mode=probe
for a in "$@"; do
  if [ "$a" = "--" ] && [ "$mode" = probe ]; then mode=launch; continue; fi
  if [ "$mode" = probe ]; then PROBE_ARGS+=("$a"); else LAUNCH_ARGS+=("$a"); fi
done
for ((i=0; i<${#PROBE_ARGS[@]}; i++)); do
  if [ "${PROBE_ARGS[$i]}" = "--settle" ]; then SETTLE="${PROBE_ARGS[$((i+1))]}"; fi
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_verify/$TAG"
mkdir -p "$OUT"

HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
if [ -z "${GAZEBO_MASTER_URI:-}" ]; then
  PORT=$(python3 -c "
import socket
p = 11950 + $HASH % 400
for _ in range(350):
    s = socket.socket()
    try:
        s.bind(('127.0.0.1', p)); s.close(); print(p); break
    except OSError:
        s.close(); p += 1
else:
    print(p)
")
  export GAZEBO_MASTER_URI="http://127.0.0.1:$PORT"
fi
export HOME="/tmp/gzhome-$TAG"
mkdir -p "$HOME"
unset DISPLAY
# ★★ 本脚本的**唯一**特殊之处：不许有手工前缀的解析根（这就是"裸命令"的定义）
unset GAZEBO_MODEL_PATH
unset GAZEBO_PLUGIN_PATH
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

#: robot11 的解析根真值（= `dirname(share/robot11)`）—— 后面核对 gzserver 的 env 用
export R11_ROOT
R11_ROOT=$(python3 -c "
import os
from ament_index_python.packages import get_package_share_directory as g
print(os.path.realpath(os.path.dirname(g('robot11'))))" 2>/dev/null || echo '')

#: 只认「环境变量里带本 master URI」的 gazebo 进程 —— 精确到"自己的 PID"。
#: 为什么不能按端口 pkill：gzserver 的 cmdline 里**只有 world 文件**，URI 在环境变量里
#: ⇒ `pkill -f "gzserver.*$GZ_PORT"` 是**空操作**（本仓已知陷阱：全机 pkill 会误伤别人，
#: 按端口又杀不到 ⇒ 两头都不对）。这里按 `/proc/<pid>/environ` 逐个核对。
_own_gz_pids() {
  local p
  for p in /proc/[0-9]*; do
    [ -r "$p/environ" ] || continue
    if tr '\0' '\n' < "$p/environ" 2>/dev/null | grep -qx "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"; then
      case "$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null)" in
        *gzserver*|*gzclient*) echo "${p#/proc/}";;
      esac
    fi
  done
}

echo "[obj1] tag=$TAG domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"
echo "[obj1] 裸命令前提：GAZEBO_MODEL_PATH 在 launch 前 = '${GAZEBO_MODEL_PATH:-<未设置>}'"
echo "[obj1] robot11 解析根真值 R11_ROOT=$R11_ROOT"
echo "[obj1] launch args: ${LAUNCH_ARGS[*]:-<默认>}"

cleanup() {
  timeout 10 ros2 topic pub -r 5 -t 5 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 3
  for pid in $(_own_gz_pids); do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in $(_own_gz_pids); do kill -9 "$pid" 2>/dev/null || true; done
  [ -n "${LAUNCH_PID:-}" ] && kill -9 "$LAUNCH_PID" 2>/dev/null || true
  sleep 1
  echo "[obj1] 残留（本 master URI 上的 gzserver/gzclient）: $( _own_gz_pids | wc -l) 个"
}
trap cleanup EXIT

ros2 launch rm_nav_bringup bringup_sim.launch.py nav_rviz:=False lio_rviz:=False \
    gui:=False "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[obj1] launch pid=$LAUNCH_PID → $OUT/launch.log"

# ---- gzserver 真身的环境（唯一能回答"哪条机制在干活"的地方）-------------------
GPID=""
for _ in $(seq 1 40); do
  GPID=$(_own_gz_pids | head -1)
  [ -n "$GPID" ] && break
  sleep 1
done
: > "$OUT/gz_env.txt"
if [ -n "$GPID" ]; then
  echo "# gzserver pid=$GPID  命令行: $(tr '\0' ' ' < /proc/$GPID/cmdline)" >> "$OUT/gz_env.txt"
  tr '\0' '\n' < /proc/$GPID/environ \
      | grep -E '^(GAZEBO_MODEL_PATH|GAZEBO_PLUGIN_PATH|GAZEBO_RESOURCE_PATH|HOME|ROS_DOMAIN_ID)=' \
      >> "$OUT/gz_env.txt"
  echo "# GAZEBO_MODEL_PATH 拆开（每行一项；右端是该目录的 realpath）:" >> "$OUT/gz_env.txt"
  python3 - "$GPID" >> "$OUT/gz_env.txt" <<'PY'
import os, sys
env = dict(l.split('=', 1) for l in open('/proc/%s/environ' % sys.argv[1]).read().split('\0') if '=' in l)
items = [x for x in env.get('GAZEBO_MODEL_PATH', '').split(os.pathsep) if x]
root = os.environ.get('R11_ROOT', '')
for i, e in enumerate(items):
    mark = ' <== robot11 解析根' if root and os.path.realpath(e) == root else ''
    print('   [%d] %-70s -> %s%s' % (i, e, os.path.realpath(e),
                                     (' (dir OK)' if os.path.isdir(e) else ' (不存在!)') + mark))
print('# GAZEBO_MODEL_PATH 里 robot11 解析根出现次数 : %d'
      % sum(1 for e in items if root and os.path.realpath(e) == root))
PY
else
  echo "# gzserver 没起来（40 s 内没在**本 master URI** 上找到 gzserver）" >> "$OUT/gz_env.txt"
fi

echo "[obj1] settle ${SETTLE}s（等 mesh 加载 / 等可能的在线模型库回落暴露出来）..."
sleep "$SETTLE"

# ---- 计数口径（重要）--------------------------------------------------------
# `No mesh specified` 必须**排除我们自己那条 [gzmodel] 日志**（它的文案里就含这个词），
# 所以判据用 gazebo 真正的错误行：`[Err] [Visual.cc:NNNN] No mesh specified`。
# ⚠️ `grep -c` 在 0 命中时**自己就会打印 0**（只是退出码为 1）⇒ 绝不能写 `|| echo 0`
#    （那会得到两行的 "0\n0"，后面算术直接炸）。用函数统一。
cnt() { grep -cE "$1" "$2" 2>/dev/null; }
NOERR=$(cnt '\[Err\].*No mesh specified' "$OUT/launch.log")
NOMESH_ALL=$(cnt 'No mesh specified' "$OUT/launch.log")
STALL=$(cnt 'Waiting for model database update' "$OUT/launch.log")
GZMODEL=$(cnt '\[gzmodel\]' "$OUT/launch.log")
GZAPPEND=$(cnt '\[gzmodel\].*GAZEBO_MODEL_PATH 追加' "$OUT/launch.log")
SYSPATH=$(cnt 'SystemPaths.cc:459' "$OUT/launch.log")
FUEL=$(cnt 'FuelModelDatabase' "$OUT/launch.log")
SPAWN=$(cnt 'Successfully spawned entity' "$OUT/launch.log")
ros2 node list 2>/dev/null | sort > "$OUT/nodes.txt"

{
  echo "=== $TAG  ($(date '+%F %T')) ==="
  echo "launch args : ${LAUNCH_ARGS[*]:-<默认>}"
  echo "GAZEBO_MODEL_PATH(启动前) : ${GAZEBO_MODEL_PATH:-<未设置>}"
  echo "gzserver 进程的 GAZEBO_MODEL_PATH : $(grep -m1 '^GAZEBO_MODEL_PATH=' "$OUT/gz_env.txt" || echo '<无>')"
  echo "--- 计数（期望：裸 robot11 跑 noerr=0 stall=0；默认模型跑 gzmodel=0）---"
  printf '[Err] ... No mesh specified          : %s\n' "$NOERR"
  printf '  其中有多少是本脚本自己的 [gzmodel] 说明文字 : %s\n' "$(( NOMESH_ALL - NOERR ))"
  printf 'Waiting for model database update    : %s\n' "$STALL"
  printf '[gzmodel] 日志行总数                 : %s\n' "$GZMODEL"
  printf '  其中"真的追加了"的行数             : %s\n' "$GZAPPEND"
  printf 'SystemPaths.cc:459                   : %s\n' "$SYSPATH"
  printf 'FuelModelDatabase                    : %s\n' "$FUEL"
  printf 'Successfully spawned entity          : %s\n' "$SPAWN"
  printf 'ros2 node list 节点数                : %s\n' "$(wc -l < "$OUT/nodes.txt")"
  echo "--- [gzmodel] 行 ---"
  grep -n '\[gzmodel\]' "$OUT/launch.log" || echo '(无)'
  echo "--- 插件/生成物关键行 ---"
  grep -nE 'tilt_rpy|scan info size|sample:|downsample:' "$OUT/launch.log" | head -8
  echo "--- spawn 行 ---"
  grep -nE 'Successfully spawned entity|SpawnEntity' "$OUT/launch.log" | head -5
  echo "--- 本 master URI 上跑过的 gazebo 进程 ---"
  grep -c 'gzserver' "$OUT/launch.log" >/dev/null && echo "(见 launch.log 的 [gzserver-N] 前缀行)"
} | tee "$OUT/summary.txt"
echo "[obj1] 输出目录: $OUT"
