cmake_minimum_required(VERSION 3.8)
project(rm_nav_bringup)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# scripts/ 现在只剩**功能**，不再有「胶水/诊断」：
#   odom_level_tf.py —— 静态边 `odom → odom_level` 的**唯一发布者**（契约 v3 §4.11）。
#     它是帧图契约的一部分（不是 patch/守卫）⇒ 留在本包，也留在本包的 CMake 安装规则里。
# 运行期诊断/守卫（map_asset_guard / map_resume_check / map_session_announcer /
# lio_extrinsic_receipt / band_guard）已移到独立包 **rm_utils**（`src/rm_utils/`），
# 安装与接线规则见该包的 setup.py；分类与删除条件见 `src/rm_utils/README.md`。
ament_auto_package(
  INSTALL_TO_SHARE
  config        # ★ 2026-10-07：traversability_criteria.yaml（坡度/台阶判据的单一真源）
  launch
  map
  PCD
  rviz
  urdf
)

install(PROGRAMS
  # ★ 2026-10-21（契约 v3 §4.11）：静态边 odom→odom_level 的**唯一发布者**（latched）+ 两条
  #   只读互检（h_meas / URDF↔IMU Δ）。gravity:=tf（默认）时由 launch 起，
  #   把"重力对齐"从估计器补丁（E1）搬到帧图上。
  scripts/odom_level_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
