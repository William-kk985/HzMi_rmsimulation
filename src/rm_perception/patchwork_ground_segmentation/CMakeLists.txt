cmake_minimum_required(VERSION 3.10)
project(patchwork_ground_segmentation)

# =============================================================================
# 地面分割槽位 `ground:=patchwork` 的包（2026-10-06 新增）
# -----------------------------------------------------------------------------
# 组成：
#   ① thirdparty/patchwork-plusplus/** —— vendored 上游库（BSD-2-Clause，pin 见 VENDORING.md），
#      由 thirdparty/patchwork-plusplus/CMakeLists.txt 编成静态库 patchworkpp_ground_seg；
#      **不在 third_party/**（那里有 COLCON_IGNORE；本包把源码放在包内，colcon 随包一起编）。
#   ② src/patchwork_ground_segmentation_node.cpp —— 与 linefit 同契约的 ROS 2 节点。
#   ③ bench/ground_seg_ab.cpp —— 离线 A/B 台架：在**同一批帧**上同时跑 patchwork 与 linefit，
#      输出逐点一致率、按"离地高度带"分桶的保留率、单帧耗时。证据见
#      docs/ground_segmentation_slots.md §4。
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
# 只需要 pcl_common：pcl::fromROSMsg/toROSMsg（pcl_conversions，头文件）、pcl::transformPointCloud、
# pcl::PointCloud。**故意不要 PCL 的 io 组件** —— 它会在配置期打两行
# "** WARNING ** io features related to pcap/png will be disabled"，破坏"0 warning"验收。
# 也**故意不链 ${PCL_LIBRARIES}**：PCL 1.12 的 PCL_LIBRARIES 里混进了 FLANN::FLANN / QHULL::QHULL
# 这些 IMPORTED 目标，在没 find_package(FLANN/QHULL) 的上下文里会直接 CMake 配置失败。
# ⚠ 必须带上 visualization 组件：linefit 的 `ground_segmentation.h` **无条件** include `viewer.h`，
#   而 `viewer.h` include `pcl/visualization/pcl_visualizer.h` ⇒ 需要 VTK 头文件目录
#   （VTK 由 PCL 的 visualization 组件带进来；我们**不链接**任何 VTK/PCL 库，只是让头文件能解析）。
#   这也是 linefit 包自己能编过的原因（它的 ament_auto 从 pcl_ros 继承到同一批 include dirs）。
find_package(PCL QUIET REQUIRED COMPONENTS common visualization)
find_package(pcl_conversions REQUIRED)

# ---- ① vendored Patchwork++（静态库，目标名 patchworkpp_ground_seg） ----
add_subdirectory(thirdparty/patchwork-plusplus ${CMAKE_CURRENT_BINARY_DIR}/patchwork-plusplus)

# ---- ② 节点 ----
add_executable(patchwork_ground_segmentation_node
  src/patchwork_ground_segmentation_node.cpp)
target_include_directories(patchwork_ground_segmentation_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(patchwork_ground_segmentation_node
  patchworkpp_ground_seg
  ${PCL_COMMON_LIBRARIES})
ament_target_dependencies(patchwork_ground_segmentation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions)

# 本包自己的 TU 吃全量告警（vendored 库的告警在它自己的 CMakeLists 里 PRIVATE 关掉）
target_compile_options(patchwork_ground_segmentation_node PRIVATE
  -Wall -Wextra -Wpedantic)

install(TARGETS patchwork_ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ---- ③ 离线 A/B 台架（可选：找不到 linefit 库就跳过，不让它挡住节点构建） ----
# 台架把 patchwork 与 linefit **同一批帧**各跑一遍 ⇒ 一致率/分桶保留率/耗时都是同一次运行里量的，
# 不存在"两次回放不同帧"的问题。用法见 docs/ground_segmentation_slots.md §4.2。
find_package(linefit_ground_segmentation QUIET)
if(linefit_ground_segmentation_FOUND)
  # ⚠ 为什么不用 ${linefit_ground_segmentation_LIBRARIES} / ament_target_dependencies：
  #   linefit_ground_segmentation 用 ament_cmake_auto，`ament_auto_package()` 会把**整条传递依赖**
  #   导出进 <pkg>_LIBRARIES —— 实测里面有 VTK::RenderingOpenGL2 / FLANN::FLANN / QHULL::QHULL 等
  #   几十个 IMPORTED 目标，而本包并没有 find_package(VTK/FLANN/QHULL) ⇒ CMake 生成期直接失败
  #   （"links to target ... but the target was not found"）。台架只需要**一个**共享库，按路径找它。
  #   路径推导：<prefix>/share/<pkg>/cmake ⇒ 上溯 3 级 = <prefix>；symlink-install 下同样成立。
  get_filename_component(_lf_prefix "${linefit_ground_segmentation_DIR}/../../.." ABSOLUTE)
  find_library(LINEFIT_GROUND_SEGMENTATION_LIB
    NAMES linefit_ground_segmentation
    PATHS "${_lf_prefix}/lib"
    NO_DEFAULT_PATH)
  if(NOT LINEFIT_GROUND_SEGMENTATION_LIB)
    message(WARNING "找得到 linefit_ground_segmentation 的 CMake 配置（${_lf_prefix}）"
      "但 ${_lf_prefix}/lib 下没有 liblinefit_ground_segmentation.so ⇒ 跳过 A/B 台架。")
  endif()
endif()

if(linefit_ground_segmentation_FOUND AND LINEFIT_GROUND_SEGMENTATION_LIB)
  # linefit 的 `ground_segmentation.h` **无条件** include `viewer.h`，
  # 而 `viewer.h` include `pcl/visualization/pcl_visualizer.h` ⇒ 台架的 TU 必须能看到 VTK 头
  # （`vtkSmartPointer.h`）。linefit 包自己能编，是因为它的 ament_auto 从 `pcl_ros` 继承了那批 include
  # dirs（/usr/include/vtk-9.1 等）；本包只 find_package(PCL COMPONENTS common visualization)，
  # 而 PCLConfig 并不会把 VTK 的 include dir 加进 PCL_INCLUDE_DIRS ⇒ 这里**直接从 VTK 目标取**（版本无关）。
  # 只用于让头文件可解析：台架**不链接**任何 VTK 库，也不实例化 PCLVisualizer。
  # 组件清单直接复用 PCLConfig 自己用的那份（PCL_VTK_COMPONENTS）——保证足够解析 pcl_visualization
  # 的全部头（缺一个就会在某个 vtkXxx.h 上编译失败）。找不到就退回最小集。
  if(NOT PCL_VTK_COMPONENTS)
    set(PCL_VTK_COMPONENTS CommonCore CommonDataModel RenderingCore)
  endif()
  find_package(VTK QUIET COMPONENTS ${PCL_VTK_COMPONENTS})
  set(_bench_vtk_incs "")
  if(VTK_FOUND)
    get_target_property(_bench_vtk_incs VTK::CommonCore INTERFACE_INCLUDE_DIRECTORIES)
  endif()
  if(NOT _bench_vtk_incs OR NOT VTK_LIBRARIES)
    message(WARNING "VTK 头/库不全 ⇒ 跳过 A/B 台架 ground_seg_ab："
      "linefit 的 ground_segmentation.h 无条件 include pcl/visualization/pcl_visualizer.h，"
      "既需要 VTK 头也需要 VTK 库（vtkDebugLeaksManager 等符号）。装 libvtk9-dev 即可。节点不受影响。")
  else()
    add_executable(ground_seg_ab bench/ground_seg_ab.cpp)
    # SYSTEM：VTK 与 linefit 的头都不是我们的代码，别让它们的 -Wpedantic/-Wreorder 污染"0 warning"验收。
    # （linefit 的 ground_segmentation.h:11 自己有 -Wreorder；VTK 的 vtkTypeMacro 有 extra ';'）
    target_include_directories(ground_seg_ab SYSTEM PRIVATE
      ${_bench_vtk_incs}
      ${linefit_ground_segmentation_INCLUDE_DIRS})
    target_link_libraries(ground_seg_ab
      patchworkpp_ground_seg
      ${LINEFIT_GROUND_SEGMENTATION_LIB}
      ${VTK_LIBRARIES})
    target_compile_options(ground_seg_ab PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS ground_seg_ab DESTINATION lib/${PROJECT_NAME})
  endif()
else()
  message(STATUS
    "linefit_ground_segmentation 未找到 ⇒ 跳过 A/B 台架 ground_seg_ab（本包节点不受影响）。"
    "要构建台架：colcon build --symlink-install --packages-up-to patchwork_ground_segmentation")
endif()

ament_package()
