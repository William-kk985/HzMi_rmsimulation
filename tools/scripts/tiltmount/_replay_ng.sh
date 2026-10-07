#!/usr/bin/env bash
# 一次 bash 调用跑完「同一帧点云 × 近地剔除 0.0 / 0.05」的回放 A/B（隔离：HOME/DOMAIN/DISPLAY）
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
F="${1:-.tmp_tiltmount/k7_geo/dump/raw_0.csv}"
SH="${2:-0.2595}"
export HOME=/tmp/gzhome-replayng; mkdir -p "$HOME"
export ROS_DOMAIN_ID=191
unset DISPLAY
source /opt/ros/humble/setup.bash
source install/setup.bash
for ng in 0.0 0.05 0.10; do
  ros2 run linefit_ground_segmentation_ros ground_segmentation_node --ros-args \
      -p use_sim_time:=false -p sensor_height:=$SH \
      -p input_topic:=/livox/lidar/pointcloud \
      -p ground_output_topic:=segmentation/ground \
      -p obstacle_output_topic:=segmentation/obstacle \
      -p r_min:=0.2 -p max_dist_to_line:=0.05 -p obstacle_near_ground_m:=$ng \
      -p self_mask_enable:=false -p traversability_enable:=true \
      -p step_height_threshold:=0.15 -p drivable_slope_deg:=25.0 -p slope_min_height:=0.05 \
      -p speed_limit_enable:=false \
      > .tmp_tiltmount/replay_$ng.log 2>&1 &
  NODE=$!
  sleep 4
  echo "### obstacle_near_ground_m = $ng"
  timeout 40 python3 tools/scripts/tiltmount/near_ground_replay_probe.py "$F" "0 0 0" "$SH" "$ng" --seconds 8
  kill -INT $NODE 2>/dev/null; sleep 1; kill -9 $NODE 2>/dev/null
  sleep 1
done
echo "=== 回放 A/B 完成"
