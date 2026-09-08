#!/bin/bash
#===============================================================================
# 脚本名称：generate_cartographer_pbstream.sh
# 功能描述：在 Gazebo 仿真环境中使用 Cartographer 重建 RMUL2026 地图的.pbstream 文件
# 作者：HzMi_rmsimulation 团队
# 版本：1.0.0
# 创建日期：2026-03-21
#===============================================================================

set -e

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

# ========== 配置变量 ==========
MAP_NAME="RMUL2026"
WORKSPACE="/home/weicheng/HzMi_rmsimulation"
MAP_DIR="${WORKSPACE}/src/rm_nav_bringup/map"
CONFIG_DIR="${WORKSPACE}/src/rm_nav_bringup/config/lua"
OUTPUT_FILE="${MAP_DIR}/${MAP_NAME}.pbstream"
LOG_FILE="/tmp/cartographer_pbstream_$(date +%Y%m%d_%H%M%S).log"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ========== 辅助函数 ==========
print_header() {
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║   Cartographer .pbstream 生成工具                   ║"
    echo "║   基于 Gazebo 仿真环境                               ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_step() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}步骤 $1: $2${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${CYAN}💡 $1${NC}"
}

log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "${LOG_FILE}"
}

# ========== 主流程 ==========
main() {
    print_header
    log_message "========== 开始 pbstream 生成流程 =========="
    
    #---------------------------------------------------------------------------
    # 步骤 1: 检查环境和配置文件
    #---------------------------------------------------------------------------
    print_step "1/5" "检查环境和配置文件"
    
    if [ ! -f "${CONFIG_DIR}/cartographer.lua" ]; then
        print_error "找不到 cartographer.lua: ${CONFIG_DIR}/cartographer.lua"
        exit 1
    fi
    
    if [ ! -f "${CONFIG_DIR}/cartographer_localization.lua" ]; then
        print_error "找不到 cartographer_localization.lua: ${CONFIG_DIR}/cartographer_localization.lua"
        exit 1
    fi
    
    if [ ! -f "${MAP_DIR}/${MAP_NAME}.yaml" ]; then
        print_error "找不到 ${MAP_NAME}.yaml: ${MAP_DIR}/${MAP_NAME}.yaml"
        exit 1
    fi
    
    if [ ! -f "${MAP_DIR}/${MAP_NAME}.pgm" ] && [ ! -f "${MAP_DIR}/${MAP_NAME}.png" ]; then
        print_error "找不到 ${MAP_NAME}.pgm 或 .png 文件"
        exit 1
    fi
    
    print_success "配置文件检查通过"
    echo "   - cartographer.lua ✓"
    echo "   - cartographer_localization.lua ✓"
    echo "   - ${MAP_NAME}.yaml ✓"
    log_message "配置文件检查完成"
    
    #---------------------------------------------------------------------------
    # 步骤 2: 显示关键配置参数
    #---------------------------------------------------------------------------
    print_step "2/5" "显示目标项目配置参数"
    
    echo ""
    echo "📊 从 cartographer.lua 提取的关键参数："
    echo "----------------------------------------"
    
    # 提取重要参数
    local tracking_frame=$(grep "tracking_frame" "${CONFIG_DIR}/cartographer.lua" | head -1)
    local published_frame=$(grep "published_frame" "${CONFIG_DIR}/cartographer.lua" | head -1)
    local resolution=$(grep "resolution" "${CONFIG_DIR}/cartographer.lua" | grep "submaps.grid_options_2d" | head -1)
    local num_point_clouds=$(grep "num_point_clouds" "${CONFIG_DIR}/cartographer.lua" | head -1)
    local use_imu=$(grep "use_imu_data" "${CONFIG_DIR}/cartographer.lua" | head -1)
    local submap_num_data=$(grep "submaps.num_range_data" "${CONFIG_DIR}/cartographer.lua" | head -1)
    
    echo "   ${tracking_frame}"
    echo "   ${published_frame}"
    echo "   ${resolution}"
    echo "   ${num_point_clouds}"
    echo "   ${use_imu}"
    echo "   ${submap_num_data}"
    echo "----------------------------------------"
    echo ""
    
    print_info "这些参数将用于生成.pbstream，确保与目标项目一致"
    log_message "配置参数已显示"
    
    read -p "$(echo -e ${YELLOW}⚠️  确认以上参数与目标项目一致吗？(y/n): ${NC})" confirm_config
    if [ "$confirm_config" != "y" ]; then
        print_error "请先确认配置文件正确性或修改配置"
        exit 1
    fi
    log_message "用户确认配置参数"
    
    #---------------------------------------------------------------------------
    # 步骤 3: 启动 Gazebo 和 Cartographer
    #---------------------------------------------------------------------------
    print_step "3/5" "启动 Gazebo 和 Cartographer"
    
    echo ""
    print_info "本步骤需要打开 3 个终端窗口"
    echo ""
    
    # 创建启动脚本
    cat > /tmp/start_cartographer_gazebo.sh << 'INNER_EOF'
#!/bin/bash
# 这个脚本由用户在三个终端中分别执行

WORKSPACE="/home/weicheng/HzMi_rmsimulation"
CONFIG_DIR="${WORKSPACE}/src/rm_nav_bringup/config/lua"

echo "======================================"
echo "请按以下步骤操作："
echo "======================================"
echo ""
echo "【终端 1】启动 Gazebo 仿真环境："
echo "--------------------------------------"
echo "cd ${WORKSPACE}"
echo "source install/setup.bash"
echo "tools/scripts/control/start_sentinel.sh -w RMUL2026 -m mapping"
echo ""
echo "【终端 2】启动 Cartographer 节点："
echo "--------------------------------------"
echo "cd ${WORKSPACE}"
echo "source install/setup.bash"
echo "ros2 launch cartographer_ros cartographer.launch.py \\"
echo "  configuration_directory:=\"${CONFIG_DIR}\" \\"
echo "  configuration_basename:=\"cartographer.lua\""
echo ""
echo "【终端 3】控制机器人运动："
echo "--------------------------------------"
echo "cd ${WORKSPACE}"
echo "source install/setup.bash"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "======================================"
INNER_EOF

    chmod +x /tmp/start_cartographer_gazebo.sh
    
    print_info "已创建启动指南脚本：/tmp/start_cartographer_gazebo.sh"
    echo ""
    print_warning "请手动执行以下操作（需要打开 3 个终端）："
    echo ""
    cat /tmp/start_cartographer_gazebo.sh
    echo ""
    
    read -p "$(echo -e ${YELLOW}按回车键表示已启动所有服务...${NC})"
    log_message "用户已启动 Gazebo 和 Cartographer"
    
    #---------------------------------------------------------------------------
    # 步骤 4: 建图操作指导
    #---------------------------------------------------------------------------
    print_step "4/5" "建图操作指导"
    
    echo ""
    echo "🗺️  请按以下步骤进行建图："
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "1️⃣  控制机器人在 RMUL2026 场地中移动"
    echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   • 覆盖所有区域（墙壁、通道、角落、障碍物）"
    echo "   • 推荐速度：0.3-0.5 m/s（慢速、稳定）"
    echo "   • 多旋转，帮助 Cartographer 识别环境特征"
    echo "   • 尽量走闭合回路，触发回环检测"
    echo ""
    echo "2️⃣  观察 Rviz 中的建图效果（如果已启动）"
    echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   • 绿色/蓝色：已探索的自由空间"
    echo "   • 黑色：障碍物"
    echo "   • 灰色：未知区域"
    echo "   • 目标：地图完整度 > 90%"
    echo ""
    echo "3️⃣  查看 Cartographer 状态"
    echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   ros2 topic echo /trajectory_node_list"
    echo "   ros2 topic echo /submap_list"
    echo ""
    echo "4️⃣  当满意建图质量后，保存.pbstream 文件"
    echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   在新终端执行以下命令："
    echo ""
    echo "   # 4.1 结束当前轨迹"
    echo "   ros2 service call /finish_trajectory \\"
    echo "     cartographer_ros_msgs/srv/FinishTrajectory \\"
    echo "     \"{trajectory_id: 0}\""
    echo ""
    echo "   # 4.2 写入.pbstream 文件"
    echo "   ros2 service call /write_state \\"
    echo "     cartographer_ros_msgs/srv/WriteState \\"
    echo "     \"{filename: '${OUTPUT_FILE}', include_unfinished_submaps: false}\""
    echo ""
    echo "   # 4.3 （可选）保存栅格地图用于验证"
    echo "   ros2 service call /write_assets \\"
    echo "     cartographer_ros_msgs/srv/WriteAssets \\"
    echo "     \"{stem: '${MAP_DIR}/${MAP_NAME}_carto', image_format: 'png'}\""
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    read -p "$(echo -e ${YELLOW}✅ 建图完成后，按回车键继续验证...${NC})"
    log_message "用户已完成建图操作"
    
    #---------------------------------------------------------------------------
    # 步骤 5: 验证结果
    #---------------------------------------------------------------------------
    print_step "5/5" "验证生成的.pbstream 文件"
    
    echo ""
    if [ -f "${OUTPUT_FILE}" ]; then
        FILE_SIZE=$(ls -lh "${OUTPUT_FILE}" | awk '{print $5}')
        
        print_success ".pbstream 文件已成功生成！"
        echo ""
        echo "╔══════════════════════════════════════════════════════╗"
        echo "║              文件信息                                ║"
        echo "╠══════════════════════════════════════════════════════╣"
        echo "║  文件名：$(basename ${OUTPUT_FILE})"
        echo "║  路径：${OUTPUT_FILE}"
        echo "║  大小：${FILE_SIZE}"
        echo "╚══════════════════════════════════════════════════════╝"
        echo ""
        
        print_info "下一步操作："
        echo ""
        echo "1️⃣  复制.pbstream 到目标项目："
        echo "   cp ${OUTPUT_FILE} /path/to/target_project/maps/"
        echo ""
        echo "2️⃣  在目标项目中启动纯定位："
        echo "   ros2 launch cartographer_ros cartographer.launch.py \\"
        echo "     configuration_directory:=\"config/lua\" \\"
        echo "     configuration_basename:=\"cartographer_localization.lua\" \\"
        echo "     load_state_filename:=\"maps/${MAP_NAME}.pbstream\""
        echo ""
        echo "3️⃣  验证定位效果："
        echo "   • 观察 Rviz 中的激光匹配情况"
        echo "   • 检查 /pose 话题的输出"
        echo "   • 测试重定位功能"
        echo ""
        
        print_success "🎉 任务完成！"
        log_message "pbstream 文件生成成功：${OUTPUT_FILE}"
        
    else
        print_error "未找到.pbstream 文件"
        echo ""
        print_info "可能的原因："
        echo "   1. 还未执行 /write_state 服务调用"
        echo "   2. 文件路径有误"
        echo "   3. Cartographer 节点异常退出"
        echo ""
        print_info "解决方法："
        echo "   1. 确认 Cartographer 正常运行"
        echo "   2. 重新执行 /write_state 服务调用"
        echo "   3. 检查日志：ros2 launch cartographer_ros --show-logs"
        echo ""
        log_message "错误：未找到 pbstream 文件"
        exit 1
    fi
    
    #---------------------------------------------------------------------------
    # 清理和总结
    #---------------------------------------------------------------------------
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    print_info "日志文件已保存到：${LOG_FILE}"
    echo ""
    echo "💾 提示："
    echo "   • 建议保留此脚本，下次可直接复用"
    echo "   • 如需调整参数，修改 config/lua/cartographer.lua"
    echo "   • 纯定位模式使用 cartographer_localization.lua"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    log_message "========== pbstream 生成流程结束 =========="
}

# 显示帮助信息
show_help() {
    echo "用法：$0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help     显示此帮助信息"
    echo "  -v, --verbose  显示详细信息"
    echo ""
    echo "示例:"
    echo "  $0             # 运行交互式生成流程"
    echo "  $0 --help      # 显示帮助"
    echo ""
}

# 解析命令行参数
case "$1" in
    -h|--help)
        show_help
        exit 0
        ;;
    -v|--verbose)
        set -x
        shift
        ;;
esac

# 运行主流程
main
