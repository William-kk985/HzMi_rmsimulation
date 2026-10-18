#!/usr/bin/env bash
# ============================================================================
# replay_identity.sh —— **同一份输入流**分别喂 before / after 两个二进制，逐位比较发布值。
# ============================================================================
# 为什么这是最强的回归证据：Gazebo 每次跑的出生位姿/落定/初始化窗口都不同（实测地面残差
# 30.110° vs 30.173°、TF rpy 差 0.07°），两次**独立**仿真跑永远不可能逐位相同。把输入流
# （/livox/lidar CustomMsg + /livox/imu + /tf_static）录下来重放，输入就完全一致；LIO 是
# 单线程、按时间戳处理 ⇒ 输出应当**逐位相同**（口径与 docs/tilted_lidar_fidelity.md §O.2 相同）。
#
# 用法： tools/scripts/tiltmount/uc_replay_identity.sh [bag_dir]
# 输出： .tmp_uc/replay/{before.txt,after.txt,diff.txt}
# ============================================================================
set +u
REPO=/home/weicheng/HzMi_rmsimulation
BAG="${1:-$REPO/.tmp_uc/bagrec/bag_in}"
OUT="$REPO/.tmp_uc/replay"; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
. "$REPO/tools/scripts/lib/scoped_cleanup.sh"
PARAMS="$REPO/install/small_point_lio/share/small_point_lio/config/mid360_sim_tuned.yaml"
export HOME="$REPO/.tmp_uc/home-replay"; mkdir -p "$HOME/.ros/log"
unset DISPLAY
export ROS_LOG_DIR="$OUT/roslog"; mkdir -p "$ROS_LOG_DIR"

one() {   # one <variant>
  local V="$1"
  local BIN="$REPO/.tmp_uc/bins/$V"
  cp -f "$BIN/libsmall_point_lio.so" "$REPO/build/small_point_lio/libsmall_point_lio.so"
  cp -f "$BIN/small_point_lio_node" "$REPO/build/small_point_lio/small_point_lio_node"
  chmod +x "$REPO/build/small_point_lio/small_point_lio_node"
  export ROS_DOMAIN_ID=$(( 170 + $(printf '%s' "replay$V" | cksum | cut -d' ' -f1) % 20 ))
  setsid ros2 run small_point_lio small_point_lio_node --ros-args \
      --params-file "$PARAMS" -p use_sim_time:=false > "$OUT/node_$V.log" 2>&1 &
  NPID=$!
  sleep 4
  python3 "$REPO/tools/scripts/tiltmount/uc_odom_compare_recorder.py" --out "$OUT/$V.txt" --seconds 60 \
      > "$OUT/rec_$V.log" 2>&1 &
  RPID=$!
  sleep 2
  timeout 90 ros2 bag play "$BAG" --topics /livox/lidar /livox/imu /tf_static -r 1.0 \
      > "$OUT/play_$V.log" 2>&1
  # 等重放结束 + 余量，再收
  sleep 6
  kill -INT $RPID 2>/dev/null; sleep 1; kill -9 $RPID 2>/dev/null
  kill -INT -- "-$NPID" 2>/dev/null; sleep 2; kill -9 -- "-$NPID" 2>/dev/null
  echo "[replay:$V] $(cat "$OUT/rec_$V.log" 2>/dev/null | tail -1)"
  echo "[replay:$V] md5(lib)=$(md5sum "$REPO/build/small_point_lio/libsmall_point_lio.so" | cut -d' ' -f1)"
}

one before
one after
sort "$OUT/before.txt" > "$OUT/before.sorted.txt"
sort "$OUT/after.txt" > "$OUT/after.sorted.txt"
if diff -q "$OUT/before.sorted.txt" "$OUT/after.sorted.txt" > /dev/null 2>&1; then
  echo "==> 值级逐位相同 ✅  ($(wc -l < "$OUT/before.txt") 行；排序后 diff 空)"
  echo "    （未排序 diff $(diff "$OUT/before.txt" "$OUT/after.txt" | grep -c '^[<>]') 行 = 仅到达顺序不同）"
  : > "$OUT/diff.txt"
else
  echo "==> 有差异 ❌（写到 $OUT/diff.txt）"
  diff "$OUT/before.txt" "$OUT/after.txt" > "$OUT/diff.txt" 2>&1 || true
  head -20 "$OUT/diff.txt"
  echo "... 差异行数: $(wc -l < "$OUT/diff.txt")"
fi
