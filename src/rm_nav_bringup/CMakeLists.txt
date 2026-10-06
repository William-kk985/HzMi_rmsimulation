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

# scripts/ = 与「地图存档」相关的共享代码（不是可执行节点）：
#   map_asset_guard.py —— 存档 sidecar（<name>.meta.yaml）的读写 + 场地隔离判定。
#   三处共用**同一份判定**（避免三套逻辑漂移）：launch 惰性 import、
#   cloud_accumulator 节点启动/保存时调用、tools/scripts/mapping/map_archive.sh。
#   ⇒ 必须装到 share/rm_nav_bringup/scripts/ 才能被前两者按 ROS 的方式找到。
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  PCD
  rviz
  scripts
  urdf
)

