#!/bin/bash
#===============================================================================
# 脚本名称：quick_start_cartographer.sh
# 功能描述：一键打开 3 个终端窗口，分别启动 Gazebo、Cartographer 和键盘控制
# 使用场景：配合 generate_cartographer_pbstream.sh 使用
#===============================================================================

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

WORKSPACE="/home/weicheng/HzMi_rmsimulation"
CONFIG_DIR="${WORKSPACE}/src/rm_nav_bringup/config/lua"

echo "╔══════════════════════════════════════════════════════╗"
echo "║   Cartographer 建图 - 快速启动工具                  ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# 检测系统环境
if ! command -v gnome-terminal &> /dev/null; then
    echo "❌ 错误：此脚本需要 gnome-terminal"
    echo "💡 安装方法：sudo apt install gnome-terminal"
    exit 1
fi

echo "📍 工作空间：${WORKSPACE}"
echo "📋 配置目录：${CONFIG_DIR}"
echo ""

#---------------------------------------------------------------------------
# 终端 1: 启动 Gazebo
#---------------------------------------------------------------------------
echo "🚀 [终端 1] 启动 Gazebo 仿真环境..."
gnome-terminal --tab --title="Gazebo-Simulation" -- bash -c "
cd ${WORKSPACE}
source install/setup.bash
echo '======================================'
echo 'Gazebo 仿真环境'
echo '======================================'
echo ''
echo '地图：RMUL2026'
echo '模式：Mapping'
echo ''
${WORKSPACE}/scripts/control/start_sentinel.sh -w RMUL2026 -m mapping
exec bash
"

# 等待 2 秒，确保 Gazebo 开始加载
sleep 2

#---------------------------------------------------------------------------
# 终端 2: 启动 Cartographer
#---------------------------------------------------------------------------
echo "🤖 [终端 2] 启动 Cartographer SLAM 节点..."
gnome-terminal --tab --title="Cartographer-SLAM" -- bash -c "
cd ${WORKSPACE}
source install/setup.bash
echo '======================================'
echo 'Cartographer SLAM'
echo '======================================'
echo ''
echo '配置：cartographer.lua'
echo '轨迹 ID: 0'
echo ''
ros2 launch cartographer_ros cartographer.launch.py \\
  configuration_directory:=\"${CONFIG_DIR}\" \\
  configuration_basename:=\"cartographer.lua\"
exec bash
"

# 等待 3 秒，确保 Cartographer 初始化
sleep 3

#---------------------------------------------------------------------------
# 终端 3: 键盘控制
#---------------------------------------------------------------------------
echo "⌨️  [终端 3] 启动键盘控制节点..."
gnome-terminal --tab --title="Teleop-Keyboard" -- bash -c "
cd ${WORKSPACE}
source install/setup.bash
echo '======================================'
echo '键盘控制 (Teleop)'
echo '======================================'
echo ''
echo '控制键位:'
echo '  I - 前进'
echo '  , - 后退'
echo '  J - 左转'
echo '  L - 右转'
echo '  K - 停止'
echo ''
echo '提示：按 Q 退出'
echo ''
ros2 run teleop_twist_keyboard teleop_twist_keyboard
exec bash
"

#---------------------------------------------------------------------------
# 显示使用说明
#---------------------------------------------------------------------------
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║              ✅ 所有服务已启动                       ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  终端 1: Gazebo 仿真环境                             ║"
echo "║  终端 2: Cartographer SLAM                           ║"
echo "║  终端 3: 键盘控制                                    ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  💡 操作指南：                                       ║"
echo "║  1. 在终端 3 中使用键盘控制机器人移动                ║"
echo "║  2. 观察终端 2 的 Cartographer 状态输出             ║"
echo "║  3. 建图完成后，参考 generate_cartographer_pbstream.sh ║"
echo "║     中的步骤保存.pbstream 文件                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "📖 详细文档请查看：CARTOGRA_PBSTREAM_GUIDE.md"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 可选：自动打开 Rviz（如果需要可视化）
read -p "是否需要打开 Rviz 进行可视化？(y/n): " open_rviz
if [ "$open_rviz" = "y" ]; then
    echo "🎨 启动 Rviz..."
    gnome-terminal --tab --title="RViz-Visualization" -- bash -c "
    cd ${WORKSPACE}
    source install/setup.bash
    rviz2 -d src/rm_nav_bringup/rviz/cartographer.rviz
    exec bash
    "
fi

echo ""
print_info "提示："
echo "   • 可以随时关闭不需要的终端窗口"
echo "   • 所有日志会保存到 /tmp/ 目录"
echo "   • 遇到问题请查看 CARTOGRA_PBSTREAM_GUIDE.md"
echo ""
