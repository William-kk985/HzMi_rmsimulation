cmake_minimum_required(VERSION 3.10)
project(robot11)

# 纯资产包（mesh / URDF / 生成式碰撞件）：不编译代码，只把 share/robot11/ 铺出来。
# `package://robot11/meshes/<name>.STL` 由 ament 的 share 目录解析，因此 meshes/ 必须安装。

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# 量测产物（mesh 清单 / 碰撞件清单 / 几何量测 / Gazebo 代价实测）：跟着包一起装，
# 方便"拿到 install/ 就能自己核数"（约 100 KB，不是大文件）。
install(
  DIRECTORY inventory
  DESTINATION share/${PROJECT_NAME}
)

# urdf/ 与 config/ 目前还不存在（Phase 2 会加）；存在才安装，避免"目录不存在"报错。
foreach(d urdf config launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${d}")
    install(DIRECTORY ${d} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
