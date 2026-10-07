#!/usr/bin/env bash
# =============================================================================
# _batch_k1.sh —— §K 的第 1 批 A/B（2026-10-09，本主题内部批次脚本，非通用工具）
#
# 跑什么（全部隔离无头、标签各不同 ⇒ ROS_DOMAIN_ID / GAZEBO_MASTER_URI 各自独立）：
#   A) 近地剔除 **关**（0.0）：robot11 plugin 档，静止 + `--goal-forward 2.0`
#   B) 近地剔除 **开**（0.05）：同上（逐字相同协议）
#   C) 默认模型（无近地剔除）：同上（对照，证明默认路径不变）
# 每跑结束把 `nav2_params_sim_robot11_costmap.yaml` 与 `traversability_near_ground_robot11.yaml`
# 还原成入库值（sha256 校验），避免"为了 A/B 改配置"留在工作区。
# =============================================================================
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
NG=src/rm_nav_bringup/config/traversability_near_ground_robot11.yaml
CM=src/rm_nav_bringup/config/nav2_params_sim_robot11_costmap.yaml
cp "$NG" .tmp_tiltmount/ng.keep; cp "$CM" .tmp_tiltmount/cm.keep
sha256sum "$NG" "$CM" > .tmp_tiltmount/batch_k1.inputs.sha256
restore() {
  cp .tmp_tiltmount/ng.keep "$NG"; cp .tmp_tiltmount/cm.keep "$CM"
  sha256sum -c .tmp_tiltmount/batch_k1.inputs.sha256 || echo "!!! 还原失败"
}
trap restore EXIT

run() {  # run <tag> <ng_value>
  local tag="$1" ng="$2"
  sed -i "s/^\( *obstacle_near_ground_m:\) .*$/\1 $ng/" "$NG"
  echo "=== [$tag] obstacle_near_ground_m=$(grep -oP 'obstacle_near_ground_m:\s*\K.*' "$NG")"
  tools/scripts/tiltmount/run_tilt_mount_probe.sh "$tag" --variant plugin --settle 25 \
      --duration 35 --frames 4 --goal-forward 2.0 --goal-wait 25 \
      -- world:=RMUL2026 mode:=slam_nav lio:=small_point_lio \
      robot:=robot11 spin_speed:=0.0 gui:=False 2>&1 | tail -32
  restore
}

run k1_ngoff 0.0
run k1_ngon 0.05
echo "=== 批次 A/B 完成"
