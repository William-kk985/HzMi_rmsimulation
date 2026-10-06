# =============================================================================
# 本文件**不是上游文件**，是本仓库为 vendored Patchwork++ 写的构建入口。
# -----------------------------------------------------------------------------
# 上游：url-kaist/patchwork-plusplus @ 3e6903a1d5537a4cc2ace897b0bbb98a92d6014c（= tag v1.4.1）
#   许可证：BSD-2-Clause（见同目录 LICENSE，Copyright (c) 2024, Urban Robotics Lab. @ KAIST）
#   上游原文件：cpp/CMakeLists.txt + cpp/{common,patchworkpp}/CMakeLists.txt
#
# 为什么没有直接用上游的 cpp/CMakeLists.txt（两个理由，都是硬理由）：
#   ① **构建期联网**：上游 `cmake/eigen.cmake` 在系统找不到 Eigen 时走
#      `FetchContent_Declare(eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/...)`。
#      而它用 `option(USE_SYSTEM_EIGEN3 ... OFF)` 且 `cmake_minimum_required(VERSION 3.11)`
#      ⇒ CMP0077 为 OLD ⇒ **普通变量设不进去**，只能靠 `-DUSE_SYSTEM_EIGEN3=ON`；
#      一旦漏传就会在配置阶段去 gitlab 下载 3MB 源码。本仓库要求"克隆即可离线构建"，
#      这个默认值不能接受。
#   ② 上游根 CMakeLists 还 add_subdirectory(patchwork)（经典的 Patchwork，需要 TBB），
#      我们只用 Patchwork++ ⇒ 白编一个库。
#   ⇒ 本文件只做上游 cpp/common + cpp/patchworkpp 那两个 CMakeLists 做的事：
#     把 plane_fit.cpp + patchworkpp.cpp 编成一个 static 库，头文件目录透传，链 Eigen。
#     **没有改任何上游源码**（.h/.cpp 逐字节来自 pinned commit）。
#
# 目标名：patchworkpp_ground_seg（避免和别的包里的 target 重名）
# 依赖：Eigen3（系统包 libeigen3-dev 3.4.0；上游的 Eigen 下载逻辑已被本条替代）
# C++ 标准：上游根 CMakeLists 写 `set(CMAKE_CXX_STANDARD 20)`。我们核对过这两个 .cpp/.h
#   里**没有任何 C++17 之后才有的构造**（无 concepts/ranges/span/<=>/<format>），
#   所以这里按本仓库主口径 **C++17** 编译（ROS 2 Humble 默认），结果与 C++20 一致。
# =============================================================================

cmake_minimum_required(VERSION 3.10)
project(patchworkpp_vendored LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(patchworkpp_ground_seg STATIC
  cpp/common/src/plane_fit.cpp
  cpp/patchworkpp/src/patchworkpp.cpp
)
set_target_properties(patchworkpp_ground_seg PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(patchworkpp_ground_seg SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/common/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/patchworkpp/include>
)
target_link_libraries(patchworkpp_ground_seg PUBLIC Eigen3::Eigen)

# 上游源码的告警（与本仓库 -Wall -Wextra -Wpedantic 口径冲突的地方）**在此显式关掉**，
# 而不是在调用方关 —— 见 VENDORING.md「告警」一节列出的具体条目。
# 关掉的理由：这些是**上游代码**的问题，我们不改上游源码，也不让它们污染"0 warning"验收。
target_compile_options(patchworkpp_ground_seg PRIVATE
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-unused-variable
)
