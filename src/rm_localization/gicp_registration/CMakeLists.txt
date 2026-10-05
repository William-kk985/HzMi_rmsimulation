cmake_minimum_required(VERSION 3.8)
project(gicp_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17（与本工程其它 rm_localization 包一致；也用到 std::filesystem）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io filters registration)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# 可执行名固定为 gicp_registration_node（bringup_sim.launch.py 的 localization:=gicp 分支按此名启动）
#
# ★ EXECUTOR MultiThreadedExecutor（2026-10-05）：**本节点必须跑多线程 executor**。
#   原因：单帧 GICP align 实测 327~389 ms（两级 leaf 0.10/0.05、target 12450 / source 3701 点；
#   首个 align 2460 ms，含目标协方差预计算），
#   而 50 Hz 的 TF 定时器原来与点云订阅共用单线程 executor 的默认回调组 ⇒ align 期间定时器排不上队，
#   `ros2 topic hz /tf` 实测 map→odom 只有 2.3 Hz（中位数 1.94）⇒ nav2 在 now+transform_tolerance 处查到的最新
#   条目越来越旧 ⇒ "Lookup would require extrapolation into the future" ⇒ follow_path 每周期 abort。
#   本宏会把 rclcpp_components 的 node_main.cpp.in 里的 @executor@ 展开成 MultiThreadedExecutor
#   （生成物：build/gicp_registration/rclcpp_components/node_main_gicp_registration_node.cpp），
#   main() 里就是 exec.add_node(node) + exec.spin()；节点内部再把 TF 定时器与点云订阅分到两个
#   回调组（tf_cb_group_ / align_cb_group_，见 src/gicp_registration.cpp 的"ROS 接口"段）。
#   二者缺一不可：只分组不换 executor（单线程）依然是串行的。
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN gicp_registration::GicpNode
  EXECUTABLE gicp_registration_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
