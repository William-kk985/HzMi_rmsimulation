cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

## use c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

## export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# enable compile warning
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif(MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if(MSVC)
    add_compile_options("/utf-8")
endif()

# optimize performance
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    add_compile_options("-ffast-math")
    add_compile_options("-fno-math-errno")
endif()

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(livox_ros_driver2 QUIET)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
if(livox_ros_driver2_FOUND)
    message(STATUS "Found livox_ros_driver2, enabling Livox lidar support.")
    add_definitions(-DHAVE_LIVOX_DRIVER)
else()
    message(WARNING "livox_ros_driver2 not found. Building without Livox lidar support.")
endif()

###########
## Build ##
###########

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB_RECURSE SOURCE src/*.cpp 3rdparty/*.c 3rdparty/*.cpp)
# ★ 本仓新增（docs/tilted_lidar_fidelity.md §N.5/§O）：外参守卫的实现在独立 TU 里。
#   显式列一遍是**必需**的：GLOB 在 configure 时求值，已有 build/ 的工作区拉到本提交后
#   不会自动重新 glob ⇒ 否则会 undefined symbol。这里显式加入并去重（新工作区两者相同）。
list(APPEND SOURCE src/extrinsic_guard.cpp)
# ★ 本仓新增（docs/frame_contract_v3.md §13，选项 U）：`odom_convention:=sensor` 的发布实现，
#   同样必须是**独立 TU**（理由见该文件头：本包 `-ffast-math`，把新浮点代码放进 odom 回调所在
#   TU 会让默认路径出现 1 ulp 差，口径见 docs/tilted_lidar_fidelity.md §O.2）。
list(APPEND SOURCE src/odometry_convention.cpp)
# ★ 本仓新增（docs/frame_contract_v3.md §4.3，E1b：`gravity_align_z_origin:=ground` 的 z 定基 +
#   初始帧地面拟合互检）。同样是**独立 TU**：默认档（gravity_align_mode:=off）不调用它，
#   避免在 hot path 所在 TU 里新增浮点代码改变 IPA/内联 ⇒ 默认路径逐位不变。
list(APPEND SOURCE src/gravity_align_ground.cpp)
list(REMOVE_DUPLICATES SOURCE)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})
target_precompile_headers(${PROJECT_NAME} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h>")
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    3rdparty
    src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    rclpy
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    std_srvs
)
if(livox_ros_driver2_FOUND)
    ament_target_dependencies(${PROJECT_NAME}
        livox_ros_driver2
    )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN small_point_lio::SmallPointLioNode
    EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)
