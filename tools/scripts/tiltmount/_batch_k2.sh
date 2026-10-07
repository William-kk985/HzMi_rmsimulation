#!/usr/bin/env bash
# =============================================================================
# _batch_k2.sh —— §K 第 2 批：**插件偏移修复**的跨模型 A/B（2026-10-09）
#
# 背景：`livox_points_plugin.cpp` 的 `point = range·axis` 漏了射线起点
#   （`minDist·axis + offset.Pos()`，`minDist` = SDF `<range><min>` = **0.1 m**）
#   ⇒ 每个点沿自己的射线朝传感器内移 0.1 m。这是**共享代码** ⇒ 必须逐模型 A/B。
#
# 口径：同世界（RMUL2026）/同出生点/同命令；每档跑两次（旧行为 / 新行为），
# 两次编译只差插件里那一行 ⇒ 点云/分割/scan/代价图/契约的差就是"0.1 m 内移"造成的。
# 每次跑两件事：`regress/run_robot11_mount_probe.sh`（代价图 + 契约）
#             + `tiltmount/run_costmap_input_dump.sh`（raw 云落盘 ⇒ 离线量"传感器系地面高度"）
# 每档跑完立刻把 `probe.json` 复制到 `<tag>.done.json`（批次被中断也知道哪几跑是好的）。
# =============================================================================
set +u
REPO=/home/weicheng/HzMi_rmsimulation
cd "$REPO"
PLUG=src/rm_simulation/livox_laser_simulation_RO2/src/livox_points_plugin.cpp
cp "$PLUG" .tmp_tiltmount/plugin.keep
trap 'cp .tmp_tiltmount/plugin.keep "$PLUG"; echo "[batch] 已还原插件源码"' EXIT

build_and_run() {
  local mode="$1"; local tag="$2"; shift 2
  cp .tmp_tiltmount/plugin.keep "$PLUG"
  if [ "$mode" = legacy ]; then
    sed -i 's/^        range_from_origin_ = true;$/        range_from_origin_ = false;   \/\/ A\/B: 旧行为/' "$PLUG"
  fi
  echo "########## $tag  插件 range_from_origin_ = $(grep -oP 'range_from_origin_ = \K\w+' "$PLUG" | head -1)"
  ( source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select ros2_livox_simulation 2>&1 | tail -2 )
  tools/scripts/regress/run_robot11_mount_probe.sh "$tag" \
      --settle 25 --duration 20 --frames 3 --grid-dump ".tmp_robotslot/$tag/local_grid.npz" -- \
      "$@" 2>&1 | tail -3
  if [ -f ".tmp_robotslot/$tag/probe.json" ]; then
    python3 - "$tag" <<'PY'
import json, sys
t = sys.argv[1]
d = json.load(open('.tmp_robotslot/%s/probe.json' % t))
ok = bool(d.get('frames'))
print('[batch] %s frames=%s ⇒ %s' % (t, len(d.get('frames') or {}), 'OK' if ok else '**空跑，需要重跑**'))
json.dump(d, open('.tmp_robotslot/%s.done.json' % t, 'w'), ensure_ascii=False, default=str)
PY
  fi
  tools/scripts/tiltmount/run_costmap_input_dump.sh "${tag}_dump" --settle 25 --duration 8 \
      --frames 3 -- "$@" 2>&1 | tail -3
  cp .tmp_tiltmount/plugin.keep "$PLUG"
}

build_and_run legacy k2_def_legacy world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0 gui:=False
build_and_run fixed  k2_def_fixed  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio spin_speed:=0.0 gui:=False
build_and_run legacy k2_r11_legacy world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
build_and_run fixed  k2_r11_fixed  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 spin_speed:=0.0 gui:=False
build_and_run legacy k2_sen_legacy world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gui:=False
build_and_run fixed  k2_sen_fixed  world:=RMUL2026 mode:=slam_nav lio:=small_point_lio robot:=robot11 robot11_mount:=sensor spin_speed:=0.0 gui:=False
echo "=== 批次 k2 完成"
