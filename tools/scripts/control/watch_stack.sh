#!/usr/bin/env bash
# 一次性黑匣子：**在旁边盯着导航栈**，自己记录"什么时候谁掉了、掉的那一刻机器状态"。
# 目的：不再一条命令一次地问，而是跑一轮就拿到一个完整证据文件。
#
# 用法（仿真已经在跑的时候，另开一个终端）：
#   tools/scripts/control/watch_stack.sh                 # 默认写 .tmp_bags/watch_<时间>.log
#   tools/scripts/control/watch_stack.sh /tmp/nav1.log   # 指定输出
#   收工：Ctrl+C（脚本会把最后的快照也写进去）
#
# 它记什么：
#   · 每 5 s：/livox/imu、/livox/lidar/pointcloud、/segmentation/obstacle、/scan、/odom 的"最新戳是否在推进"
#     （用 echo --once 拿 header.stamp，比 topic hz 可靠：大点云 hz 会假低）
#   · 每 5 s：amcl/map_server 的 lifecycle 状态、rviz2/gzserver/各节点进程的 RSS+CPU top10、free -h
#   · 一旦检测到 **/scan 从"在发"变"停"** 或 **rviz2 进程消失**：立刻抓 dmesg/journalctl/ps/free 全量快照，
#     并在日志里打一个大大的 === 事件 === 标记（这样"谁先死"一目了然）
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${PROJECT_ROOT}"

OUT="${1:-.tmp_bags/watch_$(date +%m%d_%H%M%S).log}"
mkdir -p "$(dirname "$OUT")"
TOPICS=(/livox/imu /livox/lidar/pointcloud /segmentation/obstacle /scan /odom)

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT"; }

stamp_of() {  # 取一条消息的 header.stamp（超时 2s ⇒ 打印 DEAD）
  local t="$1" out
  # ★ 2026-09-24 修正：**必须显式指定 best_effort** —— Humble 的 `ros2 topic echo/hz` 默认用
  #   RELIABLE 预设，而本链路的 /scan、/segmentation/obstacle、点云都是 BEST_EFFORT ⇒ 默认订阅
  #   根本收不到（QoS 不兼容，静默 0 条），于是把"健康的话题"误判成 DEAD（这就是前几天反复
  #   出现"/scan 挂了"的假警报来源）。判活一律带 --qos-reliability best_effort。
  out=$(timeout 2 ros2 topic echo "$t" --qos-reliability best_effort --field header.stamp --once 2>/dev/null | tr '\n' ' ')
  if [ -z "$out" ]; then echo "DEAD"; else echo "$out" | sed 's/  */ /g'; fi
}

snapshot() {  # 事件快照：此时刻的全部证据
  local tag="$1"
  {
    echo; echo "================= 事件快照: ${tag} @ $(date) ================="
    echo "--- free -h ---";                 free -h
    echo "--- 内存 top12 (RSS KB) ---";      ps -eo pid,rss,pcpu,etime,comm --sort=-rss | head -13
    echo "--- 相关进程是否活着 ---";          pgrep -a -f "gzserver|gzclient|rviz2|component_container|laser_mapping|ground_segmentation|pointcloud_to_laserscan|amcl|map_server|lio_tf" | head -20
    echo "--- 关键话题最新戳 ---"
    for t in "${TOPICS[@]}"; do echo "  $t : $(stamp_of "$t")"; done
    echo "--- lifecycle ---"
    for n in /map_server /amcl /controller_server /planner_server; do echo "  $n : $(timeout 2 ros2 lifecycle get "$n" 2>&1 | head -1)"; done
    echo "--- dmesg 尾部（找 Out of memory / killed process / GPU） ---"
    (dmesg -T 2>/dev/null || journalctl -k --no-pager 2>/dev/null) | tail -30
    echo "--- journalctl --user 尾部 ---"
    journalctl --user --no-pager -n 30 2>/dev/null
    echo "--- /tf 里 map/odom/base_link_fake 的边 ---"
    timeout 3 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id|child_frame_id" | head -20
    echo "================= 快照结束 ================="; echo
  } >>"$OUT" 2>&1
}

log "黑匣子启动 → $OUT"
log "主机: $(uname -sr) | $(nproc) 核 | $(free -h | awk '/内存|Mem/{print $2" RAM"}')"
log "GPU: $(command -v glxinfo >/dev/null && glxinfo -B 2>/dev/null | grep -iE 'OpenGL renderer|device' | head -2 || echo 'glxinfo 未安装（装 mesa-utils 可看是否软件渲染 llvmpipe）')"
log "节点: $(ros2 node list 2>/dev/null | tr '\n' ' ')"
log "开始监视（Ctrl+C 结束）"

prev_scan=""; prev_obstacle=""; prev_rviz=""; n=0
while true; do
  n=$((n+1))
  scan=$(stamp_of /scan)
  obstacle=$(stamp_of /segmentation/obstacle)
  cloud=$(stamp_of /livox/lidar/pointcloud)
  odom=$(stamp_of /odom)
  clk=$(stamp_of /clock)
  rviz=$(pgrep -c -x rviz2 2>/dev/null || echo 0)
  rss=$(ps -eo rss,comm --sort=-rss | awk 'NR<=6{printf "%s(%dMB) ",$2,$1/1024}')
  if [ "$scan" = "$prev_scan" ] && [ "$scan" != "DEAD" ]; then scan="STALE(戳没推进)"; fi
  if [ "$obstacle" = "$prev_obstacle" ] && [ "$obstacle" != "DEAD" ]; then obstacle="STALE(戳没推进)"; fi
  log "#$n clock=${clk:0:34} | scan=${scan:0:34} | obste=${obstacle:0:34} | cloud=${cloud:0:24} | odom=${odom:0:32} | rviz2=${rviz}"
  log "    RSS top: $rss"
  # 事件判定：/scan 从在发 → DEAD，或 rviz2 从有 → 无
  if [ -n "$prev_scan" ] && [ "$prev_scan" != "DEAD" ] && [ "$scan" = "DEAD" ]; then
    log "★★★ 事件：/scan 停止！立即抓快照"; snapshot "/scan 变 DEAD"
  fi
  if [ "$prev_rviz" != "0" ] && [ "$rviz" = "0" ]; then
    log "★★★ 事件：rviz2 进程消失！立即抓快照"; snapshot "rviz2 消失"
  fi
  prev_scan="$scan"; prev_rviz="$rviz"; prev_obstacle="$obstacle"
  sleep 5
done
