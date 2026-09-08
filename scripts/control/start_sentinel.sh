#!/bin/bash

# RoboMaster哨兵机器人仿真启动脚本
# 作者: Lingma
# 功能: 一键启动RMUL场地的建图模式仿真系统

set -e  # 遇到错误立即退出

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PROJECT_ROOT}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
DEFAULT_WORLD="RMUL2026"
DEFAULT_MODE="mapping"
DEFAULT_LIO="fastlio"
DEFAULT_LIO_RVIZ="False"
DEFAULT_NAV_RVIZ="True"

# 显示帮助信息
show_help() {
    echo -e "${BLUE}=== RoboMaster哨兵机器人仿真启动脚本 ===${NC}"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -w, --world WORLD     场地名称 (默认: $DEFAULT_WORLD)"
    echo "                        可选值: RMUC, RMUL, RMUL2026"
    echo "  -m, --mode MODE       运行模式 (默认: $DEFAULT_MODE)"
    echo "                        可选值: mapping, nav"
    echo "  -l, --lio LIO         LIO算法 (默认: $DEFAULT_LIO)"
    echo "                        可选值: fastlio, pointlio"
    echo "  --lio-rviz BOOL       是否启动LIO可视化 (默认: $DEFAULT_LIO_RVIZ)"
    echo "  --nav-rviz BOOL       是否启动导航可视化 (默认: $DEFAULT_NAV_RVIZ)"
    echo "  -h, --help            显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                                    # 使用默认参数启动"
    echo "  $0 -w RMUC -m nav                     # RMUC场地导航模式"
    echo "  $0 --mode mapping --lio pointlio      # 建图模式使用Point-LIO"
    echo ""
    echo "默认启动命令:"
    echo "  source install/setup.bash && \\"
    echo "  ros2 launch rm_nav_bringup bringup_sim.launch.py \\"
    echo "    world:=$DEFAULT_WORLD \\"
    echo "    mode:=$DEFAULT_MODE \\"
    echo "    lio:=$DEFAULT_LIO \\"
    echo "    lio_rviz:=$DEFAULT_LIO_RVIZ \\"
    echo "    nav_rviz:=$DEFAULT_NAV_RVIZ"
}

# 解析命令行参数
WORLD="$DEFAULT_WORLD"
MODE="$DEFAULT_MODE"
LIO="$DEFAULT_LIO"
LIO_RVIZ="$DEFAULT_LIO_RVIZ"
NAV_RVIZ="$DEFAULT_NAV_RVIZ"

while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--world)
            WORLD="$2"
            shift 2
            ;;
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        -l|--lio)
            LIO="$2"
            shift 2
            ;;
        --lio-rviz)
            LIO_RVIZ="$2"
            shift 2
            ;;
        --nav-rviz)
            NAV_RVIZ="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            echo "使用 $0 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 验证参数
validate_parameters() {
    local valid_worlds=("RMUC" "RMUL" "RMUL2026")
    local valid_modes=("mapping" "nav")
    local valid_lios=("fastlio" "pointlio")
    local valid_bools=("True" "False")

    # 验证场地
    if [[ ! " ${valid_worlds[*]} " =~ " ${WORLD} " ]]; then
        echo -e "${RED}错误: 无效的场地 '$WORLD'${NC}"
        echo "有效值: ${valid_worlds[*]}"
        exit 1
    fi

    # 验证模式
    if [[ ! " ${valid_modes[*]} " =~ " ${MODE} " ]]; then
        echo -e "${RED}错误: 无效的模式 '$MODE'${NC}"
        echo "有效值: ${valid_modes[*]}"
        exit 1
    fi

    # 验证LIO
    if [[ ! " ${valid_lios[*]} " =~ " ${LIO} " ]]; then
        echo -e "${RED}错误: 无效的LIO算法 '$LIO'${NC}"
        echo "有效值: ${valid_lios[*]}"
        exit 1
    fi

    # 验证布尔值
    if [[ ! " ${valid_bools[*]} " =~ " ${LIO_RVIZ} " ]] || [[ ! " ${valid_bools[*]} " =~ " ${NAV_RVIZ} " ]]; then
        echo -e "${RED}错误: 布尔值必须是 True 或 False${NC}"
        exit 1
    fi
}

# 检查环境
check_environment() {
    echo -e "${BLUE}[检查] 验证ROS 2环境...${NC}"
    
    # 检查ROS 2是否已安装
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到ROS 2命令，请先安装ROS 2${NC}"
        exit 1
    fi
    
    # 检查工作空间是否已编译
    if [ ! -d "install" ]; then
        echo -e "${YELLOW}警告: 未找到install目录，可能需要先编译工作空间${NC}"
        echo "建议运行: colcon build --symlink-install"
        read -p "是否继续启动? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    echo -e "${GREEN}[完成] 环境检查通过${NC}"
}

# 清理残留进程
cleanup_processes() {
    echo -e "${BLUE}[清理] 关闭可能的残留进程...${NC}"
    
    # 杀死Gazebo相关进程
    pkill -f gzserver 2>/dev/null || true
    pkill -f gzclient 2>/dev/null || true
    
    # 杀死ROS相关进程
    pkill -f ros 2>/dev/null || true
    
    # 杀死RViz进程
    pkill -f rviz2 2>/dev/null || true
    
    sleep 2
    echo -e "${GREEN}[完成] 进程清理完成${NC}"
}

# 主启动函数
main() {
    echo -e "${BLUE}=== RoboMaster哨兵机器人仿真系统启动 ===${NC}"
    echo ""
    
    # 验证参数
    validate_parameters
    
    # 显示启动配置
    echo -e "${YELLOW}启动配置:${NC}"
    echo "  场地: $WORLD"
    echo "  模式: $MODE"
    echo "  LIO算法: $LIO"
    echo "  LIO可视化: $LIO_RVIZ"
    echo "  导航可视化: $NAV_RVIZ"
    echo ""
    
    # 环境检查
    check_environment
    
    # 清理残留进程
    cleanup_processes
    
    # Source环境
    echo -e "${BLUE}[准备] 初始化ROS 2环境...${NC}"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo -e "${GREEN}[完成] 已source工作空间环境${NC}"
    else
        echo -e "${YELLOW}[警告] 未找到install/setup.bash，使用系统环境${NC}"
        source /opt/ros/$ROS_DISTRO/setup.bash
    fi
    
    # 构建启动命令
    LAUNCH_CMD="ros2 launch rm_nav_bringup bringup_sim.launch.py"
    LAUNCH_ARGS=(
        "world:=$WORLD"
        "mode:=$MODE"
        "lio:=$LIO"
        "lio_rviz:=$LIO_RVIZ"
        "nav_rviz:=$NAV_RVIZ"
    )
    
    echo ""
    echo -e "${BLUE}[启动] 执行启动命令:${NC}"
    echo "  $LAUNCH_CMD \\"
    for arg in "${LAUNCH_ARGS[@]}"; do
        echo "    $arg \\"
    done
    echo ""
    
    # 执行启动
    echo -e "${GREEN}[运行] 启动仿真系统...${NC}"
    echo -e "${YELLOW}按Ctrl+C停止系统${NC}"
    echo ""
    
    # 执行命令
    $LAUNCH_CMD "${LAUNCH_ARGS[@]}"
    
    # 处理退出
    echo ""
    echo -e "${BLUE}[结束] 仿真系统已停止${NC}"
    echo -e "${YELLOW}清理残留进程...${NC}"
    cleanup_processes
    echo -e "${GREEN}[完成] 系统关闭完成${NC}"
}

# 执行主函数
main "$@"
