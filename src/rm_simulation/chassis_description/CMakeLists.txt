cmake_minimum_required(VERSION 3.8)
project(chassis_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(chassis_mecanum_drive SHARED src/mecanum_drive_plugin.cpp)
target_include_directories(chassis_mecanum_drive PRIVATE
  ${gazebo_dev_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(chassis_mecanum_drive
  ${gazebo_dev_LIBRARIES} ${gazebo_ros_LIBRARIES}
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})
ament_target_dependencies(chassis_mecanum_drive rclcpp gazebo_ros geometry_msgs nav_msgs sensor_msgs)

install(TARGETS chassis_mecanum_drive
  LIBRARY DESTINATION lib)
install(DIRECTORY meshes urdf config launch textures DESTINATION share/${PROJECT_NAME})

ament_package()
