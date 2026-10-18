#!/usr/bin/env bash
# ============================================================================
# uc_efficiency_bench.sh —— 统一输出契约（选项 U）的**单次隔离取证跑**
#
# （原名 .tmp_uc/run_uc.sh；本文件是提交版。原始数据目录固定为 <repo>/.tmp_uc/<tag>/。）
# 精度探针复用 E1 实验的 .tmp_gravtest/{grav_probe.py,raw_world_fit.py}（gitignored 的归档）；
# 若该目录不在（新克隆），用 GRAV_PROBE_DIR=... 指过去，或忽略精度部分（效率部分照跑）。
#
# 用法：
#   .tmp_uc/run_uc.sh <tag> <variant:before|after> [LAUNCH_ARGS...]
# 例：
#   .tmp_uc/run_uc.sh uc_aligned after \
#       world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
#       robot11_mount:=sensor spin_speed:=0.0 gui:=False gravity_align_mode:=hybrid
#
# 隔离（与本仓既有 bench 同款，只清本 run 的进程）：
#   HOME=<repo>/.tmp_uc/home-<tag>（⚠️ 本机 /tmp 是**每条命令一份的私有 tmpfs**，跨命令会消失 ⇒
#     HOME/日志/结果都落在工作区内，tag 目录本身就是取证现场）、按 tag 散列的 ROS_DOMAIN_ID、
#   先探测再占用 GAZEBO_MASTER_URI、unset DISPLAY、setsid 起栈（进程组 = 本 run）、
#   收尾用 tools/scripts/lib/scoped_cleanup.sh 只清**本进程组/本 master URI**（**绝不全机 pkill**）。
#
# 二进制档位：build/small_point_lio/ 下的 `libsmall_point_lio.so` + `small_point_lio_node` 会被
#   换成 .tmp_uc/bins/<variant>/ 的那一对（swap 前后都打 md5，写进 meta.txt）⇒ 同一个 launch、
#   同一份参数，唯一变量 = 二进制。⚠️ 两者必须成对换（节点类布局变了，可执行文件也要匹配）。
#
# 输出：.tmp_uc/<tag>/{launch.log,proc.json,probe.json,clouds.npz,params.txt,publishers.txt,
#        meta.txt,launch_args.txt,ps_*.txt}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: run_uc.sh <tag> <before|after> [LAUNCH_ARGS...]}"
VARIANT="${2:?用法: run_uc.sh <tag> <before|after> [LAUNCH_ARGS...]}"
shift 2
REPO="/home/weicheng/HzMi_rmsimulation"
OUT="$REPO/.tmp_uc/$TAG"
SETTLE="${SETTLE:-20}"
WINDOW="${WINDOW:-20}"
PROBE_TIMEOUT="${PROBE_TIMEOUT:-300}"
WATCH="${WATCH:-small_point_lio,gzserver,lio_tf_adapter}"
GRAV_PROBE_DIR="${GRAV_PROBE_DIR:-$REPO/.tmp_gravtest}"
mkdir -p "$OUT"

# ---------- 0) 二进制换档（成对；md5 记账）----------
BINDIR="$REPO/.tmp_uc/bins/$VARIANT"
[ -f "$BINDIR/libsmall_point_lio.so" ] && [ -f "$BINDIR/small_point_lio_node" ] || {
  echo "[fail] 缺 $BINDIR/{libsmall_point_lio.so,small_point_lio_node}" >&2; exit 1; }
cp -f "$BINDIR/libsmall_point_lio.so" "$REPO/build/small_point_lio/libsmall_point_lio.so"
cp -f "$BINDIR/small_point_lio_node" "$REPO/build/small_point_lio/small_point_lio_node"
chmod +x "$REPO/build/small_point_lio/small_point_lio_node"
MD5_LIO=$(md5sum "$REPO/build/small_point_lio/libsmall_point_lio.so" | cut -d' ' -f1)
MD5_EXE=$(md5sum "$REPO/build/small_point_lio/small_point_lio_node" | cut -d' ' -f1)
MD5_WANT=$(md5sum "$BINDIR/libsmall_point_lio.so" | cut -d' ' -f1)
[ "$MD5_LIO" = "$MD5_WANT" ] || { echo "[fail] swap 校验失败 $MD5_LIO != $MD5_WANT" >&2; exit 1; }

# ---------- 1) 隔离环境 ----------
HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 100 + HASH % 60 ))}"
if [ -z "${GAZEBO_MASTER_URI:-}" ]; then
  PORT=$(python3 -c "
import socket
p = 11700 + $HASH % 200
for _ in range(200):
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
export HOME="$REPO/.tmp_uc/home-$TAG"
mkdir -p "$HOME"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO/install/setup.bash"
# shellcheck disable=SC1091
. "$REPO/tools/scripts/lib/scoped_cleanup.sh"

echo "[run] tag=$TAG variant=$VARIANT domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI settle=${SETTLE}s window=${WINDOW}s"
printf '%s\n' "$*" > "$OUT/launch_args.txt"
T_LAUNCH=$(date +%s.%N)
{
  echo "tag=$TAG"
  echo "variant=$VARIANT"
  echo "date=$(date -Is)"
  echo "git_head=$(git -C "$REPO" rev-parse HEAD)"
  echo "git_status_files=$(git -C "$REPO" status --porcelain | wc -l)"
  echo "bin_lib_small_point_lio_md5=$MD5_LIO"
  echo "bin_small_point_lio_node_md5=$MD5_EXE"
  echo "ros_domain_id=$ROS_DOMAIN_ID"
  echo "gazebo_master_uri=$GAZEBO_MASTER_URI"
  echo "settle=$SETTLE window=$WINDOW watch=$WATCH"
  echo "launch_args=$*"
  echo "nproc=$(nproc)"
  grep -E '^model name' /proc/cpuinfo | head -1 | sed 's/^/cpu_/'
} > "$OUT/meta.txt"

# ---------- 2) 起栈（setsid ⇒ 进程组 = 本 run）----------
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py \
    nav_rviz:=False lio_rviz:=False gui:=False "$@" > "$OUT/launch.log" 2>&1 &
LPID=$!
export LPID
echo "[run] launch pgid=$LPID → $OUT/launch.log"

cleanup() {
  echo "[cleanup] 收尾（只清本 run：进程组 $LPID / master $GAZEBO_MASTER_URI）"
  kill -INT -- "-$LPID" 2>/dev/null
  sleep 2
  sc_kill_own_gz 9
  sc_kill_own_ros2_launch 9
  sc_kill_own_pattern 9 'small_point_lio_node|laserMapping|lio_tf_adapter|rviz2|complementary_filter'
  sleep 1
  echo "[cleanup] 残留: $(sc_own_leftovers_desc) / gz=$(sc_own_gz_count)"
}
trap cleanup EXIT

sleep 2
ps -o pid,pgid,pcpu,rss,etimes,comm -g "$LPID" > "$OUT/ps_early.txt" 2>/dev/null || true

# ---------- 3) 两个探针并行：精度（grav_probe）+ 开销（uc_procstats）----------
# grav_probe 的窗口口径与本脚本一致（settle 后取 frames 帧）；uc_procstats 取 settle 后 window 秒。
if [ -f "$GRAV_PROBE_DIR/grav_probe.py" ]; then
  python3 "$GRAV_PROBE_DIR/grav_probe.py" \
      --out "$OUT/probe.json" --npz "$OUT/clouds.npz" \
      --settle "$SETTLE" --frames 5 --wall-timeout "$PROBE_TIMEOUT" \
      > "$OUT/probe.log" 2>&1 &
  GP=$!
else
  echo "[run] 跳过精度探针（找不到 $GRAV_PROBE_DIR/grav_probe.py；设 GRAV_PROBE_DIR=... 指过去）"
  GP=""
fi
python3 "$REPO/tools/scripts/tiltmount/uc_procstats.py" \
    --out "$OUT/proc.json" --settle "$SETTLE" --window "$WINDOW" \
    --pgid "$LPID" --watch "$WATCH" --launch-epoch "$T_LAUNCH" \
    --wall-timeout "$PROBE_TIMEOUT" > "$OUT/proc.log" 2>&1
RC2=$?
RC1=99
if [ -n "$GP" ]; then wait $GP; RC1=$?; fi
echo "[run] grav_probe rc=$RC1 (99=跳过) / uc_procstats rc=$RC2"

ps -o pid,pgid,pcpu,rss,etimes,comm -g "$LPID" > "$OUT/ps_after.txt" 2>/dev/null || true

# ---------- 4) 生效参数 + 发布者数（契约：每条边恰好一个发布者）----------
{
  for kv in "/small_point_lio publish_tf" "/small_point_lio odom_convention" \
            "/small_point_lio gravity_align_mode" "/small_point_lio gravity_align_rpy" \
            "/small_point_lio fix_gravity_direction" "/small_point_lio gravity" \
            "/lio_tf_adapter odom_topic" "/lio_tf_adapter xyz" "/lio_tf_adapter rpy" \
            "/laserMapping mapping.gravity" "/ground_segmentation gravity_aligned_frame" \
            "/pointcloud_to_laserscan target_frame"; do
    set -- $kv
    printf '%-52s = ' "$1 $2"
    timeout 12 ros2 param get "$1" "$2" 2>&1 | tail -1
  done
} > "$OUT/params.txt" 2>&1

{
  for t in /Odometry /odom /cloud_registered /livox/lidar /scan /tf /tf_static; do
    printf '%-24s publishers=' "$t"
    timeout 15 ros2 topic info "$t" 2>&1 | grep -i "publisher count" | head -1
  done
  echo "--- tf2_echo odom base_link（静止、窗口末端；验收看 roll/pitch ≈0）---"
  timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -A4 "Translation" | head -12
  echo "--- tf2_monitor odom base_link（**广播者清单**：契约 I2 要求只有 1 个）---"
  timeout 14 ros2 run tf2_ros tf2_monitor odom base_link --no-parameter-updates 2>&1 | grep -E "Broadcaster|average rate|Buffer length|Failed|WARNING" | head -14
} > "$OUT/publishers.txt" 2>&1

echo "[run] 完成 → $OUT"
