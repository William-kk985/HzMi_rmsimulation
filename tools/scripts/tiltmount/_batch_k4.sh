#!/usr/bin/env bash
# _batch_k4.sh —— 短目标验收 + 反向验证（2026-10-09）
#   ① 近地剔除开（0.05）+ 现状几何：`--goal-forward 0.5`（1.0 m 目标在上一批没走）
#   ② 同上 + `--drive`（固定动作）⇒ 反向验证：限速 / 真障碍仍在 / 漂移
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
NG=src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml
CM=src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
cp "$NG" .tmp_tiltmount/ng.keep; cp "$CM" .tmp_tiltmount/cm.keep
trap 'cp .tmp_tiltmount/ng.keep "$NG"; cp .tmp_tiltmount/cm.keep "$CM"' EXIT

tools/scripts/tiltmount/run_tilt_mount_probe.sh k4_goal05 --variant plugin --settle 25 \
    --duration 30 --frames 3 --goal-forward 0.5 --goal-wait 40 -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
    2>&1 | tail -12
tools/scripts/tiltmount/run_tilt_mount_probe.sh k4_drive --variant plugin --settle 25 \
    --duration 45 --frames 3 --drive -- \
    world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False \
    2>&1 | tail -12
echo "=== 批次 k4 完成"
