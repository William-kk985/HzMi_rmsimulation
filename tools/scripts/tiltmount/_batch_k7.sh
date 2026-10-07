#!/usr/bin/env bash
# _batch_k7.sh —— 量"传感器离地高度"（= linefit `sensor_height` 的物理真值）
# 只做一件事：跑一次静止栈 + dump（带 use_sim_time 的 TF 回读），供离线核对。
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
tools/scripts/tiltmount/run_costmap_input_dump.sh k7_geo --settle 30 --duration 8 --frames 3 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
    2>&1 | tail -8
echo "=== 批次 k7 完成"
