cmake_minimum_required(VERSION 3.16)
project(rm_gz_planar_move VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ── 可选依赖：Gazebo Fortress（ignition-gazebo6）──────────────────────────────
# 本包只在 `sim_backend:=gz` 下用。**没有 Fortress 的机器必须能照常 colcon build**
# （Classic 路径零影响）⇒ 这里 QUIET 探测，找不到就"空过"（只装 package.xml）。
find_package(ignition-gazebo6 QUIET)
find_package(ignition-plugin1 QUIET COMPONENTS register)
find_package(ignition-transport11 QUIET)
find_package(ignition-msgs8 QUIET)

if(NOT ignition-gazebo6_FOUND OR NOT ignition-plugin1_FOUND)
  message(STATUS "rm_gz_planar_move: 没找到 ignition-gazebo6（Gazebo Fortress）"
                 "⇒ 跳过编译（sim_backend:=gz 才需要它；Classic 路径不受影响）")
  ament_package()
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(rm_gz_planar_move SHARED src/planar_move_system.cpp)
target_compile_features(rm_gz_planar_move PUBLIC cxx_std_17)
target_link_libraries(rm_gz_planar_move
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-plugin1::register)
if(ignition-transport11_FOUND)
  target_link_libraries(rm_gz_planar_move ignition-transport11::ignition-transport11)
endif()
if(ignition-msgs8_FOUND)
  target_link_libraries(rm_gz_planar_move ignition-msgs8::ignition-msgs8)
endif()
ament_target_dependencies(rm_gz_planar_move rclcpp nav_msgs)

install(TARGETS rm_gz_planar_move
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
