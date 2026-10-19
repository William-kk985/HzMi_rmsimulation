#!/usr/bin/env bash
# ============================================================================
# spl_ab_bench.sh —— 同一个 `small_point_lio` 的「改前 / 改后」单次隔离取证跑
#
# 与 `uc_efficiency_bench.sh`（2026-10-20 那一轮）的差别：
#   ① 探针可组合：PROBES="grav points navband"（默认 grav），每个探针各自超时、失败不拖垮其它；
#   ② 收尾**把 build/ 的二进制还原成 after（= 当前 HEAD）**，避免批次末尾留下 before 二进制；
#   ③ meta 里额外记 `git log -1 --format=%H` 与两个 md5、以及 `PROBES`/`LIO_ROLE`；
#   ④ params.txt 增加 nav 侧的高度带键（`local/global_costmap.obstacle_layer.scan.*`）。
#
# 用法：
#   tools/scripts/tiltmount/spl_ab_bench.sh <tag> <before|after> [LAUNCH_ARGS...]
# 例（本报告的三档）：
#   PROBES="grav points" tools/scripts/tiltmount/spl_ab_bench.sh v0_r1 before \
#       world:=RMUL2026 mode:=mapping lio:=small_point_lio robot:=robot11 \
#       robot11_mount:=sensor spin_speed:=0.0 gravity_align_mode:=off
#   PROBES="grav points" tools/scripts/tiltmount/spl_ab_bench.sh v2_r1 after \
#       ... gravity_align_mode:=hybrid
#   PROBES="grav navband" tools/scripts/tiltmount/spl_ab_bench.sh n2_r1 after \
#       world:=RMUL2026 mode:=nav lio:=small_point_lio localization:=gicp ... \
#       gravity_align_mode:=hybrid
#
# 环境变量：
#   SETTLE=20 WINDOW=20          窗口口径（仿真秒，与上一轮逐字相同）
#   PROBES="grav points navband" 要跑的探针（grav=精度/姿态；points=每帧进 EKF 点数；navband=代价图）
#   GRAV_PROBE_DIR=<dir>         grav_probe.py 所在目录（默认 .tmp_gravtest；没有就跳过）
#   WATCH=small_point_lio,gzserver,lio_tf_adapter   /proc 记账的进程前缀（pointlio 时改 pointlio）
#   PROBE_TIMEOUT=400            探针墙钟上限
#   KEEP_LOG=1                   保留 roslog
#
# 隔离（与上一轮逐字相同）：HOME=<repo>/.tmp_ab/home-<tag>、按 tag 散列的 ROS_DOMAIN_ID、
#   先探测端口再占用 GAZEBO_MASTER_URI、unset DISPLAY、setsid 起栈（进程组 = 本 run）、
#   收尾只用 tools/scripts/lib/scoped_cleanup.sh 清**本进程组/本 master URI**（**不做全机 pkill**）。
#
# 输出：.tmp_ab/<tag>/{launch.log, proc.json, probe.json, clouds.npz, points.json, navband.json,
#        params.txt, publishers.txt, meta.txt, launch_args.txt, ps_*.txt}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: spl_ab_bench.sh <tag> <before|after> [LAUNCH_ARGS...]}"
VARIANT="${2:?用法: spl_ab_bench.sh <tag> <before|after> [LAUNCH_ARGS...]}"
shift 2
REPO="/home/weicheng/HzMi_rmsimulation"
OUT="$REPO/.tmp_ab/$TAG"
SETTLE="${SETTLE:-20}"
WINDOW="${WINDOW:-20}"
PROBE_TIMEOUT="${PROBE_TIMEOUT:-400}"
PROBES="${PROBES:-grav}"
WATCH="${WATCH:-small_point_lio,gzserver,lio_tf_adapter}"
GRAV_PROBE_DIR="${GRAV_PROBE_DIR:-$REPO/.tmp_gravtest}"
mkdir -p "$OUT"

# ---------- 0) 二进制换档（成对；md5 记账；收尾还原 after）----------
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
export HOME="$REPO/.tmp_ab/home-$TAG"
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

echo "[run] tag=$TAG variant=$VARIANT domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI settle=${SETTLE}s window=${WINDOW}s probes='$PROBES'"
printf '%s\n' "$*" > "$OUT/launch_args.txt"
T_LAUNCH=$(date +%s.%N)
{
  echo "tag=$TAG"
  echo "variant=$VARIANT"
  echo "date=$(date -Is)"
  echo "git_head=$(git -C "$REPO" rev-parse HEAD)"
  echo "git_head_subject=$(git -C "$REPO" log -1 --format=%s)"
  echo "git_status_files=$(git -C "$REPO" status --porcelain | wc -l)"
  echo "bin_lib_small_point_lio_md5=$MD5_LIO"
  echo "bin_small_point_lio_node_md5=$MD5_EXE"
  echo "ros_domain_id=$ROS_DOMAIN_ID"
  echo "gazebo_master_uri=$GAZEBO_MASTER_URI"
  echo "settle=$SETTLE window=$WINDOW watch=$WATCH probes=$PROBES"
  echo "launch_args=$*"
  echo "nproc=$(nproc)"
  grep -E '^model name' /proc/cpuinfo | head -1 | sed 's/^/cpu_/'
} > "$OUT/meta.txt"

# ---------- 2) 起栈（setsid ⇒ 进程组 = 本 run）----------
setsid ros2 launch rm_nav_bringup bringup_sim.launch.py \
    nav_rviz:=False lio_rviz:=False gui:=False "$@" > "$OUT/launch.log" 2>&1 &
LPID=$!
export LPID
export PGID="$LPID"
echo "[run] launch pgid=$LPID → $OUT/launch.log"

cleanup() {
  echo "[cleanup] 收尾（只清本 run：进程组 $LPID / master $GAZEBO_MASTER_URI）"
  kill -INT -- "-$LPID" 2>/dev/null
  sleep 2
  sc_kill_own_gz 9
  sc_kill_own_ros2_launch 9
  sc_kill_own_pattern 9 'small_point_lio_node|laserMapping|lio_tf_adapter|rviz2|complementary_filter|slam_toolbox|linefit|pointcloud_to_laserscan|map_session_announcer|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|smoother_server|lifecycle_manager|costmap_2d|map_server|amcl|gicp_registration|fake_vel_transform|robot_state_publisher' 2>/dev/null
  sleep 1
  echo "[cleanup] 残留: $(sc_own_leftovers_desc) / gz=$(sc_own_gz_count)"
  # 二进制还原成 after（= 当前 HEAD 的构建），避免批次末尾留下 before
  if [ -f "$REPO/.tmp_uc/bins/after/libsmall_point_lio.so" ]; then
    cp -f "$REPO/.tmp_uc/bins/after/libsmall_point_lio.so" "$REPO/build/small_point_lio/libsmall_point_lio.so"
    cp -f "$REPO/.tmp_uc/bins/after/small_point_lio_node" "$REPO/build/small_point_lio/small_point_lio_node"
    chmod +x "$REPO/build/small_point_lio/small_point_lio_node"
    echo "[cleanup] build/ 二进制已还原 after md5=$(md5sum "$REPO/build/small_point_lio/libsmall_point_lio.so" | cut -d' ' -f1)"
  fi
}
trap cleanup EXIT

sleep 2
ps -o pid,pgid,pcpu,rss,etimes,comm -g "$LPID" > "$OUT/ps_early.txt" 2>/dev/null || true

# ---------- 3) 探针（并行；各自超时）----------
PIDS=()
if [[ " $PROBES " == *" grav "* ]] && [ -f "$GRAV_PROBE_DIR/grav_probe.py" ]; then
  timeout $((PROBE_TIMEOUT + 20)) python3 "$GRAV_PROBE_DIR/grav_probe.py" \
      --out "$OUT/probe.json" --npz "$OUT/clouds.npz" \
      --settle "$SETTLE" --frames 5 --wall-timeout "$PROBE_TIMEOUT" \
      > "$OUT/probe.log" 2>&1 &
  PIDS+=("$!")
fi
if [[ " $PROBES " == *" points "* ]]; then
  timeout $((PROBE_TIMEOUT + 20)) python3 "$REPO/tools/scripts/tiltmount/spl_points_probe.py" \
      --out "$OUT/points.json" --settle "$SETTLE" --window "$WINDOW" \
      --wall-timeout "$PROBE_TIMEOUT" > "$OUT/points.log" 2>&1 &
  PIDS+=("$!")
fi
if [[ " $PROBES " == *" navband "* ]]; then
  timeout $((PROBE_TIMEOUT + 20)) python3 "$REPO/tools/scripts/tiltmount/nav_band_probe.py" \
      --out "$OUT/navband.json" --settle "$SETTLE" --window "$WINDOW" \
      --wall-timeout "$PROBE_TIMEOUT" > "$OUT/navband.log" 2>&1 &
  PIDS+=("$!")
fi
timeout $((PROBE_TIMEOUT + 60)) python3 "$REPO/tools/scripts/tiltmount/uc_procstats.py" \
    --out "$OUT/proc.json" --settle "$SETTLE" --window "$WINDOW" \
    --pgid "$LPID" --watch "$WATCH" --launch-epoch "$T_LAUNCH" \
    --wall-timeout "$PROBE_TIMEOUT" > "$OUT/proc.log" 2>&1
RC_PROC=$?
RC_ALL=0
for p in "${PIDS[@]:-}"; do [ -n "$p" ] || continue; wait "$p" || RC_ALL=$((RC_ALL + 1)); done
echo "[run] uc_procstats rc=$RC_PROC / 其它探针失败数=$RC_ALL"

ps -o pid,pgid,pcpu,rss,etimes,comm -g "$LPID" > "$OUT/ps_after.txt" 2>/dev/null || true

# ---------- 4) 生效参数 + 发布者数（契约：每条边恰好一个发布者）----------
{
  for kv in "/small_point_lio publish_tf" "/small_point_lio odom_convention" \
            "/small_point_lio gravity_align_mode" "/small_point_lio gravity_align_rpy" \
            "/small_point_lio fix_gravity_direction" "/small_point_lio gravity" \
            "/small_point_lio point_filter_num" "/small_point_lio space_downsample" \
            "/small_point_lio min_distance" \
            "/lio_tf_adapter odom_topic" "/lio_tf_adapter xyz" "/lio_tf_adapter rpy" \
            "/laserMapping mapping.gravity" "/ground_segmentation gravity_aligned_frame" \
            "/pointcloud_to_laserscan target_frame" \
            "/local_costmap/local_costmap obstacle_layer.scan.min_obstacle_height" \
            "/local_costmap/local_costmap obstacle_layer.scan.max_obstacle_height" \
            "/local_costmap/local_costmap robot_radius" \
            "/global_costmap/global_costmap obstacle_layer.scan.min_obstacle_height"; do
    set -- $kv
    printf '%-64s = ' "$1 $2"
    timeout 12 ros2 param get "$1" "$2" 2>&1 | tail -1
  done
} > "$OUT/params.txt" 2>&1

{
  for t in /Odometry /odom /cloud_registered /livox/lidar /scan /tf /tf_static /local_costmap/costmap_raw; do
    printf '%-32s publishers=' "$t"
    timeout 15 ros2 topic info "$t" 2>&1 | grep -i "publisher count" | head -1
  done
  echo "--- tf2_echo odom base_link（静止、窗口末端）---"
  timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -A4 "Translation" | head -12
  echo "--- tf2_echo odom base_link_fake ---"
  timeout 5 ros2 run tf2_ros tf2_echo odom base_link_fake 2>&1 | grep -A4 "Translation" | head -12
  echo "--- tf2_monitor odom base_link（**广播者清单**：契约 I2 要求只有 1 个）---"
  timeout 14 ros2 run tf2_ros tf2_monitor odom base_link --no-parameter-updates 2>&1 | grep -E "Broadcaster|average rate|Buffer length|Failed|WARNING" | head -14
} > "$OUT/publishers.txt" 2>&1

# nav 侧日志（"掉出高度带"类线索 + 定位/代价图告警）——只 grep，不改任何东西
grep -aE "dropped|out of band|min_obstacle|max_obstacle|Failed to transform|Could not transform|extrapolat|costmap.*(fail|warn)|Killed|not current|isCurrent" \
    "$OUT/launch.log" | head -120 > "$OUT/log_grep.txt" 2>/dev/null || true

echo "[run] 完成 → $OUT"
