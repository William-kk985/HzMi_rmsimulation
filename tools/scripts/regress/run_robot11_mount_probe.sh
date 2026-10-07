#!/usr/bin/env bash
# ============================================================================
# run_robot11_mount_probe.sh —— 「雷达安装方式」取证跑（2026-10-08）
#
# 用法：
#   tools/scripts/regress/run_robot11_mount_probe.sh <tag> [PROBE_ARGS...] -- [LAUNCH_ARGS...]
# 例：
#   tools/scripts/regress/run_robot11_mount_probe.sh r11m_plugin --duration 60 -- \
#       world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0
#   tools/scripts/regress/run_robot11_mount_probe.sh r11m_urdf --duration 60 -- \
#       world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 \
#       robot11_mount:=urdf spin_speed:=0.0
#
# 隔离约定（与 run_robot_model_probe.sh 同款，见 docs/smoke_test_runbook.md）：
#   HOME=/tmp/gzhome-<tag>、非默认 ROS_DOMAIN_ID、先探测再占用 GAZEBO_MASTER_URI、unset DISPLAY、
#   跑前/跑后只清**本 master URI 上**的 gzserver/gzclient（绝不做全机 pkill）。
# 输出：.tmp_robotslot/<tag>/{launch.log, probe.json, mount_evidence.txt}
# ============================================================================
set -o pipefail
set +u

TAG="${1:?用法: run_robot11_mount_probe.sh <tag> [PROBE_ARGS...] -- [LAUNCH_ARGS...]}"
shift
PROBE_ARGS=(); LAUNCH_ARGS=(); mode=probe
for a in "$@"; do
  if [ "$a" = "--" ] && [ "$mode" = probe ]; then mode=launch; continue; fi
  if [ "$mode" = probe ]; then PROBE_ARGS+=("$a"); else LAUNCH_ARGS+=("$a"); fi
done

# ★ 2026-10-08：`--drive` = 探针窗口内加一段**固定动作**（直行 10 s / 原地转 10 s 交替），
#   用来把"LIO 漂移 vs /odom_ground_truth"从"静止窗噪声"变成可 A/B 的量
#   （docs/tilted_lidar_fidelity.md §C.3 / §G 第 6 项）。它**不是**探针的参数 ⇒ 要摘出来。
DRIVE=0; _rest=()
for a in "${PROBE_ARGS[@]}"; do
  if [ "$a" = "--drive" ]; then DRIVE=1; else _rest+=("$a"); fi
done
PROBE_ARGS=("${_rest[@]}")

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="$REPO/.tmp_robotslot/$TAG"
mkdir -p "$OUT"

HASH=$(printf '%s' "$TAG" | cksum | cut -d' ' -f1)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( 30 + HASH % 61 ))}"
if [ -z "${GAZEBO_MASTER_URI:-}" ]; then
  PORT=$(python3 -c "
import socket
p = 11350 + $HASH % 500
for _ in range(400):
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
export ROS_LOG_DIR="$OUT/roslog"
mkdir -p "$ROS_LOG_DIR"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

GZ_PORT="${GAZEBO_MASTER_URI##*:}"
echo "[run] tag=$TAG domain=$ROS_DOMAIN_ID master=$GAZEBO_MASTER_URI"
echo "[run] launch args: ${LAUNCH_ARGS[*]:-<默认>}"
echo "[run] probe args : ${PROBE_ARGS[*]:-<默认>}"

#: 只认「环境变量里带本 master URI」的 gazebo 进程（精确到自己的 PID）。
#: ⚠️ 不能按端口 pkill：gzserver 的 cmdline 里**没有** URI 端口（URI 在环境变量里）
#: ⇒ `pkill -f "gzserver.*$GZ_PORT"` 是**空操作**；而全机 `pkill -x gzserver` 又会误伤别人
#: （本仓已知陷阱）。这里按 `/proc/<pid>/environ` 逐个核对 master URI。
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

cleanup() {
  timeout 10 ros2 topic pub -r 5 -t 5 /cmd_vel_chassis geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  [ -n "${DRIVE_PID:-}" ] && kill "$DRIVE_PID" 2>/dev/null || true
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  sleep 3
  for pid in $(_own_gz_pids); do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in $(_own_gz_pids); do kill -9 "$pid" 2>/dev/null || true; done
  [ -n "${LAUNCH_PID:-}" ] && kill -9 "$LAUNCH_PID" 2>/dev/null || true
  sleep 1
  echo "[run] 残留（本 master URI 上的 gzserver/gzclient）: $( _own_gz_pids | wc -l) 个"
}
trap cleanup EXIT

for pid in $(_own_gz_pids); do kill -9 "$pid" 2>/dev/null || true; done
sleep 1

ros2 launch rm_nav_bringup bringup_sim.launch.py nav_rviz:=False lio_rviz:=False \
    "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[run] launch pid=$LAUNCH_PID → $OUT/launch.log"

sleep 3
echo "=== [run] xacro 侧证据（生成物真的按开关渲染了吗） ===" | tee "$OUT/mount_evidence.txt"
grep -m1 -A3 "body_to_livox" "$OUT/launch.log" >> "$OUT/mount_evidence.txt" 2>/dev/null || true

echo "=== [run] 传感/分割发布者契约 ===" 
ros2 node list 2>/dev/null | sort | tr '\n' ' '; echo

if [ "$DRIVE" = 1 ]; then
  # 固定动作（两种安装档**逐字相同**）：直行 10 s（vx=0.30）↔ 原地转 10 s（wz=0.60），循环。
  # 口径说明：漂移量是"odom 位移 − 真值位移"（探针里的本仓惯例），有运动才有意义。
  # ⚠️ 注入点 = `/cmd_vel_chassis`（mecanum 插件订阅的那个话题；`fake_vel_transform` 是
  #    **事件驱动**地把 `/cmd_vel` 转发到这里，不会持续发布零速来覆盖我们 —— 源码
  #    `src/rm_navigation/fake_vel_transform/src/fake_vel_transform.cpp:31,33,82`）。
  #    publisher 的 stderr 落盘（drive.log），否则"没动"时分不清是"没发出去"还是"发出去没被执行"。
  echo "=== [run] --drive 注入点 = /cmd_vel_chassis 的发布者/订阅者 ===" | tee -a "$OUT/mount_evidence.txt"
  timeout 15 ros2 topic info /cmd_vel_chassis -v 2>&1 | head -14 | tee -a "$OUT/mount_evidence.txt"
  (
    # ★ 关键：**等到探针的 settle 结束再开始发**（= 与记录窗口对齐）。
    #   实测教训（2026-10-08 接手复核）：publisher 早于"机器人 spawn + 插件加载"出现时，
    #   这一跑里 gzserver 在 `mecanum_controller` 插件加载处 **segfault（exit -11）**
    #   （另一次是 gzclient abort + spawn 服务超时）⇒ 注入点必须在机器人稳定之后。
    sleep "${SETTLE:-25}"
    while :; do
      timeout 13 ros2 topic pub -r 20 --times 200 /cmd_vel_chassis geometry_msgs/msg/Twist \
          '{linear: {x: 0.30}, angular: {z: 0.0}}' 2>&1
      timeout 13 ros2 topic pub -r 20 --times 200 /cmd_vel_chassis geometry_msgs/msg/Twist \
          '{linear: {x: 0.0}, angular: {z: 0.60}}' 2>&1
    done
  ) > "$OUT/drive.log" 2>&1 &
  DRIVE_PID=$!
  echo "[run] --drive 已开：直行 10 s / 原地转 10 s 交替（pid=$DRIVE_PID，日志 $OUT/drive.log）"
fi

python3 "$REPO/tools/scripts/regress/robot11_mount_probe.py" \
    --out "$OUT/probe.json" "${PROBE_ARGS[@]}" 2>&1 | tee "$OUT/probe.log"
RC=${PIPESTATUS[0]}
echo "[run] probe rc=$RC"
[ -n "${DRIVE_PID:-}" ] && kill "$DRIVE_PID" 2>/dev/null || true
if [ "$DRIVE" = 1 ]; then
  echo "=== [run] --drive publisher 日志（尾部 12 行：失败会在这里） ===" | tee -a "$OUT/mount_evidence.txt"
  tail -12 "$OUT/drive.log" 2>/dev/null | cut -c1-160 | tee -a "$OUT/mount_evidence.txt"
  echo "=== [run] --drive 期间的真实运动（/odom_ground_truth 位移，来自探针） ===" | tee -a "$OUT/mount_evidence.txt"
  python3 - "$OUT/probe.json" <<'PY' 2>&1 | tee -a "$OUT/mount_evidence.txt"
import json, sys
d = json.load(open(sys.argv[1]))
for t, v in (d.get('odom') or {}).items():
    print('  %-24s n=%-5s displacement=%-9s path=%s'
          % (t, v.get('n'), v.get('displacement_m'), v.get('path_m')))
print('  drift_vs_truth_m=%s  yaw_deg=%s' % (d.get('drift_vs_truth_m'), d.get('drift_vs_truth_yaw_deg')))
PY
fi

# ★ 2026-10-08：参数/TF 回读放在**探针跑完之后**（nav2 的节点要 20~30 s 才激活；
#   放在 launch 后 3 s 读会全部 "Node not found"，那是读数时机的问题、不是配置的问题）。
echo "=== [run] 运行期生效值（节点侧 ros2 param get，探针之后读） ===" | tee -a "$OUT/mount_evidence.txt"
for kv in "/ground_segmentation sensor_height" "/ground_segmentation gravity_aligned_frame" \
          "/ground_segmentation self_mask_enable" \
          "/pointcloud_to_laserscan min_height" "/pointcloud_to_laserscan max_height" \
          "/pointcloud_to_laserscan range_min" "/pointcloud_to_laserscan range_max" \
          "/local_costmap/local_costmap robot_radius" \
          "/local_costmap/local_costmap inflation_layer.inflation_radius" \
          "/global_costmap/global_costmap robot_radius" \
          "/global_costmap/global_costmap inflation_layer.inflation_radius"; do
  set -- $kv
  printf '  %-46s %-34s = ' "$1" "$2" | tee -a "$OUT/mount_evidence.txt"
  timeout 15 ros2 param get "$1" "$2" 2>&1 | tail -1 | tee -a "$OUT/mount_evidence.txt"
done
echo "=== [run] TF：base_link→{livox_frame, imu_link}（帧到底斜不斜，看这里） ===" | tee -a "$OUT/mount_evidence.txt"
for child in livox_frame imu_link; do
  printf '  base_link → %-12s ' "$child" | tee -a "$OUT/mount_evidence.txt"
  timeout 12 ros2 run tf2_ros tf2_echo base_link "$child" 2>/dev/null \
    | grep -m1 -A4 "Translation" | tr '\n' ' ' | sed 's/  */ /g' | tee -a "$OUT/mount_evidence.txt"
  echo | tee -a "$OUT/mount_evidence.txt"
done
# ★ 2026-10-08：**Gazebo 自己**眼里的 link 姿态（独立于 TF/URDF）——回答"画出来的 mesh 到底斜没斜"。
#   两条路都试，因为第一条路**在本仓的 launch 里根本不存在**：
#   ① `/get_entity_state`（gazebo_msgs/srv/GetEntityState）需要 `libgazebo_ros_api_plugin.so`，
#      而本仓 gzserver 只加载 `libgazebo_ros_{init,factory,force_system}.so`
#      （见 gzserver 的 cmdline）⇒ **服务不存在**，调用只会"waiting for service"。
#      （上一轮把这件事误记成"grep 过滤掉了"，本次复核纠正：不是过滤问题，是没有这个服务。）
#   ② **gazebo transport**（不经 ROS）：`gz topic -e /gazebo/default/pose/info` 里有每个 link 的
#      世界位姿（protobuf 文本），这才是"Gazebo 眼里的 link"的直接读数。
echo "=== [run] Gazebo 侧的 link 世界姿态（独立于 URDF/TF） ===" | tee -a "$OUT/mount_evidence.txt"
ros2 service list 2>/dev/null | sort > "$OUT/gz_services.txt"
echo "  /get_entity_state 服务存在吗: $(grep -c '^/get_entity_state$' "$OUT/gz_services.txt") 个" \
    | tee -a "$OUT/mount_evidence.txt"
: > "$OUT/gz_link_pose.txt"
for ent in "robot::livox_frame" "robot::base_link" "robot::imu_link"; do
  {
    echo "=== /get_entity_state  name='$ent'  reference_frame='world'（预期：服务不存在）"
    timeout 12 ros2 service call /get_entity_state gazebo_msgs/srv/GetEntityState \
        "{name: '$ent', reference_frame: 'world'}" 2>&1 | head -4
  } >> "$OUT/gz_link_pose.txt"
done
# ② gazebo transport 的 pose/info（`-e` 后面必须**紧跟话题名**，`-d` 放最后；
#    写成 `-e -d 5 <topic>` 会被 gz 判成 "Invalid arguments"，实测踩过）
if command -v gz >/dev/null 2>&1; then
  timeout 15 gz topic -l > "$OUT/gz_topics.txt" 2>&1 || true
  POSE_TOPIC=$(grep -m1 'pose/info' "$OUT/gz_topics.txt" || echo '/gazebo/default/pose/info')
  echo "  pose 话题: $POSE_TOPIC（gz topic -l 的第 1 个 pose/info）" | tee -a "$OUT/mount_evidence.txt"
  timeout 25 gz topic -e "$POSE_TOPIC" -d 5 > "$OUT/gz_pose_info.txt" 2>&1 || true
  python3 - "$OUT/gz_pose_info.txt" <<'PY' 2>&1 | tee -a "$OUT/mount_evidence.txt"
import math, re, sys
txt = open(sys.argv[1], errors='replace').read()
blocks = re.findall(r'name:\s*"([^"]+)"(.*?)(?=name:\s*"|\Z)', txt, re.S)
want = ('robot::livox_frame', 'robot::base_link', 'robot::imu_link')
found = {}
for name, body in blocks:
    if name in want and name not in found:
        m = re.search(r'orientation:\s*\{([^}]*)\}', body, re.S)
        p = re.search(r'position:\s*\{([^}]*)\}', body, re.S)
        def f(blk, key):
            mm = re.search(r'%s:\s*(-?[\d.eE+-]+)' % key, blk or '')
            return float(mm.group(1)) if mm else float('nan')
        if m:
            x, y, z, w = (f(m.group(1), k) for k in ('x', 'y', 'z', 'w'))
            n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
            x, y, z, w = x / n, y / n, z / n, w / n
            rpy = (math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
                   math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
                   math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
            xyz = (f(p.group(1), 'x'), f(p.group(1), 'y'), f(p.group(1), 'z')) if p else (float('nan'),) * 3
            found[name] = (xyz, rpy)
for name in want:
    if name in found:
        xyz, rpy = found[name]
        print('  %-20s xyz=[%.4f, %.4f, %.4f]  rpy=[%.3f, %.3f, %.3f] deg'
              % (name, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
    else:
        print('  %-20s <pose/info 里没抓到>' % name)
print('  （口径：gz topic -e /gazebo/default/pose/info，Gazebo 自己的世界位姿，不经 ROS/TF）')
PY
  # ③ Gazebo **自己**建出来的模型 SDF（`gz model -i`）——这是"画出来的 mesh 到底斜没斜"的
  #    直接读数：URDF 的**固定关节**在 spawn 时被 gazebo "lump" 成嵌套模型
  #    （证据：传感器话题名是 `/gazebo/default/robot/base_link/livox_frame/scan`
  #     ⇒ livox_frame 是挂在 base_link 下的嵌套体，所以它**不在** pose/info 的 link 列表里，
  #     这也解释了为什么 TF/URDF 看得到它、`gz topic pose/info` 看不到）。
  timeout 25 gz model -m robot -i > "$OUT/gz_model_info.txt" 2>&1 || true
  echo "  --- gz model -m robot -i 里与 livox/imu 有关的行（Gazebo 侧真身） ---" \
      | tee -a "$OUT/mount_evidence.txt"
  grep -nE 'livox|imu|base_link|<pose|nested|model name' "$OUT/gz_model_info.txt" 2>/dev/null \
      | head -25 | cut -c1-190 | tee -a "$OUT/mount_evidence.txt"
fi
echo "=== [run] launch.log 里的 mount/插件关键行 ==="
grep -nE "tilt_rpy|livox_mount|robot11_mount|scan info size|sample:|downsample:|Spawn|spawn" \
    "$OUT/launch.log" | head -20
echo "[run] 输出目录: $OUT"
exit $RC
