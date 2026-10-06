cmake_minimum_required(VERSION 3.10)
project(rm_ground_traversability)

# =============================================================================
# rm_ground_traversability —— 地面分割之后的「坡度 / 台阶」判定（header-only）
# -----------------------------------------------------------------------------
# 只装头文件 + 导出一个 INTERFACE 目标。没有节点、没有话题：谁 include 谁用。
# 消费者：linefit_ground_segmentation_ros/ground_segmentation_node、
#         patchwork_ground_segmentation/patchwork_ground_segmentation_node。
# 阈值真源：src/rm_nav_bringup/config/traversability_criteria.yaml（launch 作为参数文件喂进来）
#          + tools/scripts/mapping/pcd_to_nav2_map.py 读同一份文件。
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# 只需要 pcl_common（pcl::PointCloud / pcl::PointXYZ）；不链任何 PCL 库（纯头文件模板）。
find_package(PCL QUIET REQUIRED COMPONENTS common)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${PCL_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp sensor_msgs std_msgs pcl_conversions)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp sensor_msgs std_msgs pcl_conversions)

ament_package()
