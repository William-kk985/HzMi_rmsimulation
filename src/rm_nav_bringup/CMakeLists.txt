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

# scripts/ = 与「地图存档」相关的共享代码：
#   map_asset_guard.py      —— 存档 sidecar（<name>.meta.yaml）的读写 + 场地隔离判定 +
#                              覆盖前备份/恢复（`*.prev-<ts>`）。
#     三处共用**同一份判定**（避免三套逻辑漂移）：launch 惰性 import、
#     cloud_accumulator 节点启动/保存时调用、tools/scripts/mapping/map_archive.sh。
#     ⇒ 必须装到 share/rm_nav_bringup/scripts/ 才能被前两者按 ROS 的方式找到。
#   map_resume_check.py     —— 续建后的**一次性**一致性检查节点（2026-10-06 事故后新增）。
#     它不是共享库而是可执行节点 ⇒ 另装到 lib/rm_nav_bringup/，
#     launch 才能用 Node(package='rm_nav_bringup', executable='map_resume_check.py') 起它。
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  PCD
  rviz
  scripts
  urdf
)

install(PROGRAMS
  scripts/map_resume_check.py
  DESTINATION lib/${PROJECT_NAME}
)

