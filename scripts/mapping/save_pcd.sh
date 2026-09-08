#!/bin/bash

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

# 检测当前运行的ROS2节点和服务
echo "检测当前系统状态..."

# 检查是否有LIO相关节点（优先检查，因为nav模式下LIO和SLAM Toolbox都运行）
if ros2 node list 2>/dev/null | grep -E "(laser_mapping|fastlio_mapping)"; then
    echo "✅ 检测到LIO建图系统正在运行..."
    echo "💾 保存点云地图..."
    ros2 service call /map_save std_srvs/srv/Trigger

    echo ""
    echo "📍 点云地图将保存到:"
    echo "   src/rm_nav_bringup/PCD/RMUL2026.pcd"
    echo ""
    echo "🔍 保存完成后，你可以使用以下命令验证:"
    echo "   ls -la src/rm_nav_bringup/PCD/"
    exit 0
fi

# 如果没有LIO节点，检查是否有SLAM Toolbox
if ros2 service list 2>/dev/null | grep -q "/slam_toolbox/save_map"; then
    echo "⚠️  检测到SLAM Toolbox正在运行 (nav模式)"
    echo "💡 在nav模式下无法保存点云地图"
    echo ""
    echo "🔄 如果你需要点云地图，请按以下步骤操作:"
    echo "   1. 停止当前系统 (Ctrl+C)"
    echo "   2. 启动mapping模式:"
    echo "      scripts/control/start_sentinel.sh -w RMUL2026 -m mapping --lio fastlio --lio-rviz True"
    echo "   3. 探索完整环境"
    echo "   4. 在新终端中执行: scripts/mapping/save_pcd.sh"
    exit 1
fi

# 如果都没有，提示错误
echo "❌ 未检测到有效的建图系统！"
echo "🔧 请确保在mapping或nav模式下运行系统以使用LIO建图"
echo ""
echo "🚀 启动mapping模式的命令:"
echo "   scripts/control/start_sentinel.sh -w RMUL2026 -m mapping --lio fastlio --lio-rviz True"
echo ""
echo "🚀 启动nav模式的命令:"
echo "   scripts/control/start_sentinel.sh -w RMUL2026 -m nav --lio fastlio --nav-rviz True"
exit 1
