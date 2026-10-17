cmake_minimum_required(VERSION 3.16)
project(rm_livox_gz_sim NONE)

find_package(ament_cmake REQUIRED)

# 纯资产 + launch 包：没有编译产物，也**不 find_package 任何 gz / rmagine 东西**
# （理由见 package.xml：默认路径 colcon build 必须零影响）。
install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
install(FILES doc/README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
