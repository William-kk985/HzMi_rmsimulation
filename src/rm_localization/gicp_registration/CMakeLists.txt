cmake_minimum_required(VERSION 3.8)
project(gicp_registration)

# ============================ vendored small_gicp（第三种配准后端，2026-10-05） ============================
# 依赖：koide3/small_gicp，**MIT**，pinned commit 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e（= tag v1.0.1，2026-06-13）。
#   上游：https://github.com/koide3/small_gicp  许可证文件：third_party/small_gicp/LICENSE（MIT，Copyright (c) 2024 Kenji Koide）
#   复现 clone（与本仓库当前内容逐字节一致）：
#     git clone https://github.com/koide3/small_gicp.git third_party/small_gicp
#     cd third_party/small_gicp && git checkout 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e
#     rm -rf .git            # 随仓库一起提交（vendored），不再作为 git 仓库
#   ⚠ third_party/COLCON_IGNORE 只让 **colcon 不去发现** third_party/** 里的 package.xml
#     （small_gicp 自带 package.xml，里面是 <build_type>cmake</build_type>）；它**不影响 CMake** ——
#     下面的 add_subdirectory 照常工作。两件事互不相干，别把 COLCON_IGNORE 当"CMake 也要忽略"。
# 为什么用 add_subdirectory 而不是 find_package：
#   · small_gicp 是 header-only（README: "You can just download and drop it in your project directory"），
#     上游 CMake 的目标是 `small_gicp`（BUILD_HELPER=ON 时链接 Eigen + OpenMP 的 helper 库）；
#   · find_package(small_gicp) 需要系统级安装（sudo make install）⇒ 破坏本仓库"克隆即可构建"的可复现性；
#   · 上游默认 BUILD_TESTS/EXAMPLES/BENCHMARKS/PYTHON_BINDINGS=OFF ⇒ 只多编一个 helper 库。
# 上游构建选项（本次核实过的真实名字，不是 -DSMALL_GICP_USE_*）：
#   BUILD_HELPER=ON（默认）· BUILD_WITH_OPENMP=auto（默认；找到就用）· BUILD_WITH_TBB=OFF（默认）
#   BUILD_WITH_MARCH_NATIVE=OFF（默认；**保持 OFF**：开了会把二进制绑死在编译机上）
#   其余 BUILD_TESTS/BUILD_EXAMPLES/BUILD_BENCHMARKS/BUILD_PYTHON_BINDINGS 默认 OFF，本包不打开。
# ★ 位置很关键：add_subdirectory **必须在下面的 add_compile_options(-Wall -Wextra -Wpedantic) 之前**。
#   add_compile_options 只对"调用之后创建"的目标生效（下级目录在 add_subdirectory 时快照继承目录属性）
#   ⇒ 第三方代码不套本包的严格告警契约（否则上游代码的 -Wextra 告警会污染"本包 0 warning"这条要求）。
set(SMALL_GICP_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/small_gicp"
  CACHE PATH "vendored koide3/small_gicp 源码目录（pinned commit 见上）")
if(NOT EXISTS "${SMALL_GICP_VENDOR_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "找不到 vendored small_gicp：${SMALL_GICP_VENDOR_DIR}\n"
    "  复现：git clone https://github.com/koide3/small_gicp.git third_party/small_gicp && "
    "cd third_party/small_gicp && git checkout 57c1106daf83c2c79ee0c58a9c7ed0032298ff4e\n"
    "  或：-DSMALL_GICP_VENDOR_DIR=/path/to/small_gicp")
endif()
# ★★ 构建配置"防污染"（本次实测踩到的坑，必须保留这段）：
#   small_gicp 的 CMakeLists.txt 开头有
#     if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#       set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
#     endif()
#   而 colcon/ament 默认**不设置** CMAKE_BUILD_TYPE（本包改动前的基线 = 该变量为空 ⇒ 编译命令里
#   没有任何 -O 标志）⇒ add_subdirectory 会把这个 Release **FORCE 进 cache**，
#   把**本包自己的所有目标**静默变成 `-O3 -DNDEBUG`。实测后果（同机、同合成扫描、
#   target 12450 / source 3701 点，单节点）：
#     PCL GICP 单帧 align = 395 ms（无 -O，**= 本仓库已验证/已标定的口径**）→ 13 ms（-O3）≈ **30 倍**
#   这等于顺手改了"已验证路径"的构建配置（还附带 -DNDEBUG 关掉 assert）⇒ 绝不能静默发生。
#   所以这里把 CMAKE_BUILD_TYPE **原样还原**（基线为空就还原成空），保证本包的编译标志与改动前逐字一致
#   ⇒ A/B 才是在同一个构建口径下做的。
#   注：还原是**全局**的 ⇒ small_gicp 的 helper 库也按同样的（无 -O）标志编译。对本包无影响：
#   我们只用它的 header-only 模板 API，实测 `--as-needed` 甚至没把 libsmall_gicp.so 写进
#   libgicp_registration.so 的 DT_NEEDED（readelf 可复现）⇒ 运行时不存在"少一个 so"的风险。
set(_gicp_registration_build_type_backup "${CMAKE_BUILD_TYPE}")
add_subdirectory("${SMALL_GICP_VENDOR_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/small_gicp")
set(CMAKE_BUILD_TYPE "${_gicp_registration_build_type_backup}"
  CACHE STRING "Build type（本包已还原成进入 add_subdirectory 之前的值；见上方注释）" FORCE)
unset(_gicp_registration_build_type_backup)
# ⚠ 上游头文件里在 -Wextra 下有 2 处告警（本次实测）：
#     small_gicp/registration/rejector.hpp:23  unused parameter ×5 [-Wunused-parameter]
#     small_gicp/ann/kdtree_omp.hpp:62         signed/unsigned 比较    [-Wsign-compare]
#   而本包的契约是"0 warning"。CMake 侧把依赖的 include 变成 -isystem **做不到**（试过两种写法）：
#   · 只设 INTERFACE_SYSTEM_INCLUDE_DIRECTORIES：CMake 对**非 IMPORTED** 目标只在
#     INTERFACE_INCLUDE_DIRECTORIES 里取路径（实测仍是 -I，告警照旧）；
#   · 清掉 INTERFACE_INCLUDE_DIRECTORIES 只留 SYSTEM：连头文件都找不到了（直接用不了）。
#   ⇒ 改为在**唯一 include small_gicp 的翻译单元**（src/registration_backend.cpp）里用
#     GCC/Clang 的 diagnostic pragma 把这两类告警按"第三方头文件"屏蔽（写法见该文件顶部）。
#   注意：不要用整文件 -w（那会把本包自己的代码也一起静音，等于放弃告警契约）。

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
# OpenMP：我们自己 include small_gicp 的 header（registration_backend.cpp）⇒ **本包的 TU 也必须带 -fopenmp**，
# 否则 small_gicp/registration/reduction_omp.hpp:16 的 `#warning "OpenMP is not available..."`
# 会直接破掉"0 warning"，而且 ParallelReductionOMP 会退化成单线程（性能 A/B 就没意义了）。
find_package(OpenMP REQUIRED)

ament_auto_find_build_dependencies()

# 注意：DIRECTORY src 是 **GLOB_RECURSE**（ament_cmake_auto）⇒ src/ 下任何 .cpp 都会被编进本库。
# 台架（带 main()）因此放在 src/ 之外的 bench/（见文件末尾），不改动这一行。
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  small_gicp          # vendored add_subdirectory 出来的目标（PUBLIC 带上 include/ + Eigen + OpenMP）
  OpenMP::OpenMP_CXX)
# vendored 的 libsmall_gicp.so 装在同一个 install/<pkg>/lib 下：
#   · 本库在 lib/ 里 ⇒ $ORIGIN 够；节点可执行在 lib/gicp_registration/ ⇒ 需要 $ORIGIN/..
#   （不依赖用户先 source setup.bash 也能跑；colcon 的 library_path hook 只是兜底）
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN;$ORIGIN/..")

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

# ============================ A/B 台架（离线，不启 ROS 图） ============================
# bench/gicp_backend_bench：在**同一批帧**上对 pcl / small_gicp 两个后端量 align ms（median/p95/max）、
# fitness（PCL 等价 m²）、以及"故意给坏初值（+0.5 m / +10°）"时的收敛行为。
# 走的是与节点完全相同的代码（gicp_registration::RegistrationBackend + voxelDownsample）
# ⇒ 台架数值应与节点 [status] 行一致。用法见 docs/gicp_backend_small_gicp.md。
# 装在 lib/${PROJECT_NAME}/（与 gicp_registration_node 同目录），因此同样需要 $ORIGIN/.. 的 RPATH。
ament_auto_add_executable(gicp_backend_bench bench/gicp_backend_bench.cpp)
set_target_properties(gicp_backend_bench PROPERTIES INSTALL_RPATH "$ORIGIN;$ORIGIN/..")

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
