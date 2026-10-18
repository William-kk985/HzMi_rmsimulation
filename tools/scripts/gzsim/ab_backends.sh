#!/usr/bin/env bash
# =============================================================================
# ab_backends.sh —— **同一轮、同一世界、同一起始位姿、同一时长**的 Classic vs gz 并排测量
# =============================================================================
# 为什么需要它：`docs/fortress_sim_backend.md` §2.8 的 Classic 一列一直是"别的轮次的
# 历史基线"，所以"变化"只能看量级。本脚本在同一次会话里**先后**把两个后端按同一组
# 参数拉起来（不是同时跑：同时跑会互相抢 CPU，RTF/频率全部失真），用**同一个探针**
# （tools/scripts/gzsim/ab_probe.py）量同一批指标，最后打印并排表。
#
# 协议（两个后端逐字相同，除 `sim_backend` 本身）：
#   world:=RMUC2026  mode:=mapping  lio:=small_point_lio  robot:=robot11
#   robot11_mount:=sensor  gui:=False  spin_speed:=0.0  nav_rviz:=False  lio_rviz:=False
#   采样窗口 = 收到首帧之后 `--duration` 墙钟秒
# 每个后端独占：ROS_DOMAIN_ID / GAZEBO_MASTER_URI（Classic）/ GZ_PARTITION（gz）/ HOME，
# 收尾只杀自己的进程（tools/scripts/lib/scoped_cleanup.sh）。
#
# 用法：
#   tools/scripts/gzsim/ab_backends.sh <tag> [duration_s] [backend...]
#   tools/scripts/gzsim/ab_backends.sh ab1 20                # classic + gz
#   tools/scripts/gzsim/ab_backends.sh ab1 20 gz             # 只跑 gz
#   AB_OUT=/path tools/scripts/gzsim/ab_backends.sh ab1 20
# =============================================================================
set -u
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
TAG=${1:?用法: ab_backends.sh <tag> [duration_s] [backend...]}
DUR=${2:-20}
shift 2 2>/dev/null || shift 1
BACKENDS=("$@")
[ ${#BACKENDS[@]} -eq 0 ] && BACKENDS=(classic gz)

OUT="${AB_OUT:-$REPO/.tmp_gz_ab/$TAG}"
mkdir -p "$OUT"

# ---- 只杀自己的进程 ---------------------------------------------------------
# shellcheck disable=SC1091
. "$REPO/tools/scripts/lib/scoped_cleanup.sh"

# ---- 公共环境（每个后端会各自覆盖 domain/URI/partition） --------------------
export HOME="${AB_HOME:-$REPO/.tmp_gz_ab/home}"
mkdir -p "$HOME/.ignition" "$HOME/.cache" "$HOME/.config" "$HOME/.local/share" "$HOME/.runtime"
chmod 700 "$HOME/.runtime" 2>/dev/null || true
export XDG_RUNTIME_DIR="$HOME/.runtime" XDG_CACHE_HOME="$HOME/.cache"
export XDG_CONFIG_HOME="$HOME/.config" XDG_DATA_HOME="$HOME/.local/share"
export LIBGL_ALWAYS_SOFTWARE=1
unset DISPLAY
set +u
. /opt/ros/humble/setup.bash
. "$REPO/install/setup.bash"
GZSIM_PREFIX="${GZSIM_PREFIX:-$REPO/.tmp_gzlivox2/gzsim_ws/install}"
if [ -f "$GZSIM_PREFIX/setup.bash" ]; then
  . "$GZSIM_PREFIX/setup.bash"
fi
set -u
export GZSIM_PREFIX

echo "TAG=$TAG  DUR=$DUR  OUT=$OUT  backends=${BACKENDS[*]}"
for BACKEND in "${BACKENDS[@]}"; do
  echo
  echo "================= $BACKEND ================="
  RUN="$OUT/$BACKEND"
  rm -rf "$RUN"; mkdir -p "$RUN"
  export ROS_DOMAIN_ID=$(( 120 + RANDOM % 60 ))
  export GAZEBO_MASTER_URI="http://127.0.0.1:$(( 11380 + RANDOM % 100 ))"
  export GZ_PARTITION="rmab_${TAG}_${BACKEND}_$$"
  export IGN_PARTITION="$GZ_PARTITION"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI GZ_PARTITION=$GZ_PARTITION" \
    | tee "$RUN/env.txt"

  T_LAUNCH=$(date +%s.%N)
  setsid ros2 launch rm_nav_bringup bringup_sim.launch.py \
    sim_backend:="$BACKEND" world:=RMUC2026 mode:=mapping lio:=small_point_lio \
    robot:=robot11 robot11_mount:=sensor gui:=False spin_speed:=0.0 \
    nav_rviz:=False lio_rviz:=False gz_partition:="$GZ_PARTITION" \
    > "$RUN/launch.log" 2>&1 &
  LPID=$!
  echo "launch_pid=$LPID t_launch=$T_LAUNCH" | tee -a "$RUN/env.txt"

  # 探针**从 launch 那一刻就开始等首帧**（这样 startup→首帧 才量得准）
  timeout $(( DUR + 400 )) python3 "$REPO/tools/scripts/gzsim/ab_probe.py" \
    --backend "$BACKEND" --duration "$DUR" --launch-epoch "$T_LAUNCH" --pid "$LPID" \
    --log "$RUN/launch.log" --out "$RUN/metrics.json" > "$RUN/probe.log" 2>&1
  echo "probe_exit=$?"

  # 契约证据（发布者数 / TF 边 / IMU 单调性 / gz 通道独占性）
  GZSIDE=""
  [ "$BACKEND" = classic ] && GZSIDE="--skip-gz-side"
  timeout 90 python3 "$REPO/tools/scripts/gzsim/gz_contract_check.py" \
    --duration 10 --out "$RUN/contract.json" $GZSIDE \
    --expect odom:base_link --expect map:odom > "$RUN/contract.log" 2>&1 || true

  # 收尾：只杀本 run（进程组 + GAZEBO_MASTER_URI 判定）
  kill -INT -- "-$LPID" 2>/dev/null || true
  sleep 3
  sc_kill_own_gz 9
  sc_kill_own_ros2_launch 9
  sc_kill_own_pattern 9 'gzserver|gzclient|robot_state_publisher|slam_toolbox|linefit|pointcloud_to_laserscan|small_point_lio'
  sleep 1
  echo "leftovers: $(sc_own_leftovers_desc 'gzserver|gzclient|ros2 launch|small_point_lio')" \
    | tee -a "$RUN/env.txt"
done

# ---- 并排表 ---------------------------------------------------------------
python3 - "$OUT" "${BACKENDS[@]}" <<'PY'
import json, os, sys
out = sys.argv[1]
backs = sys.argv[2:]
rows = {}
for b in backs:
    p = os.path.join(out, b, 'metrics.json')
    if os.path.isfile(p):
        rows[b] = json.load(open(p))

FIELDS = [
    ('rtf_clock_sim_over_wall', 'RTF（/clock 仿真/墙钟）', '%.3f'),
    ('lidar_sim_hz', '`/livox/lidar` 仿真 Hz', '%.3f'),
    ('lidar_wall_hz', '`/livox/lidar` 墙钟 Hz', '%.3f'),
    ('points_median', '每帧点数（中位）', '%.1f'),
    ('raycast_ms_median', '逐帧求交 ms（插件插桩，中位）', '%.3f'),
    ('gzserver_cpu_ms_per_frame', 'gzserver CPU ms/帧', '%.3f'),
    ('rss_server_median_mb', 'gzserver RSS 中位 (MB)', '%.1f'),
    ('rss_total_median_mb', '整棵进程树 RSS 中位 (MB)', '%.1f'),
    ('cpu_server_pct', 'gzserver CPU%（单核）', '%.1f'),
    ('cpu_tree_pct', '整棵树 CPU%（单核）', '%.1f'),
    ('startup_to_first_frame_s', '启动 → 首帧 (s)', '%.2f'),
    ('scan_beams_median', '`/scan` 有限波束数（中位）', '%.0f'),
    ('scan_range_min', '`/scan` range min (m)', '%.2f'),
    ('scan_range_max', '`/scan` range max (m)', '%.2f'),
    ('cloud_registered_n', '`/cloud_registered` 条数（窗口内）', '%.0f'),
    ('cloud_registered_width_median', '`/cloud_registered` width 中位', '%.0f'),
]
hdr = '| 指标 | ' + ' | '.join(backs) + ' |'
print('\n' + hdr)
print('|' + '---|' * (len(backs) + 1))
for key, label, fmt in FIELDS:
    cells = []
    for b in backs:
        v = rows.get(b, {}).get(key)
        cells.append('—' if v is None else (fmt % v))
    print('| %s | %s |' % (label, ' | '.join(cells)))
PY
echo
echo "产物：$OUT/<backend>/{metrics.json,contract.log,launch.log,env.txt}"
