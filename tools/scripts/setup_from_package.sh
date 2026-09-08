#!/bin/bash

################################################################################
# HzMi_rmsimulation 一键配置脚本
# 用于快速配置和编译项目，无需 Git/Gitee
################################################################################

set -e

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否在正确的工作目录
check_directory() {
    if [ ! -d "src" ]; then
        print_error "请在 HzMi_rmsimulation 目录下运行此脚本"
        exit 1
    fi
}

# 检查 ROS2 环境
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "未检测到 ROS2 环境，尝试自动加载..."
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            print_success "已加载 ROS2 Humble 环境"
        else
            print_error "未找到 ROS2 Humble，请先安装 ROS2"
            echo "参考：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
            exit 1
        fi
    else
        print_info "当前 ROS2 版本：$ROS_DISTRO"
    fi
}

# 检查依赖
check_dependencies() {
    print_info "检查系统依赖..."
    
    local missing_deps=()
    
    # 检查必要的 ROS2 包
    local ros_packages=(
        "ros-humble-slam-toolbox"
        "ros-humble-navigation2"
        "ros-humble-nav2-bringup"
        "ros-humble-gazebo-ros-pkgs"
    )
    
    for pkg in "${ros_packages[@]}"; do
        if ! dpkg -l | grep -q "$pkg"; then
            missing_deps+=("$pkg")
        fi
    done
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_warning "缺少以下依赖："
        printf '  - %s\n' "${missing_deps[@]}"
        echo ""
        read -p "是否自动安装这些依赖？(y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "安装依赖中..."
            sudo apt update
            sudo apt install -y "${missing_deps[@]}"
            print_success "依赖安装完成"
        else
            print_warning "请手动安装缺失的依赖后重新运行此脚本"
            exit 1
        fi
    else
        print_success "系统依赖检查通过"
    fi
}

# 检查 Livox SDK2
check_livox_sdk() {
    print_info "检查 Livox SDK2..."
    
    if [ ! -f "/usr/local/lib/liblivox_sdk_shared.so" ]; then
        print_warning "Livox SDK2 未安装"
        echo ""
        read -p "是否现在安装 Livox SDK2? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "安装 Livox SDK2..."
            cd ~
            
            if [ ! -d "Livox-SDK2" ]; then
                git clone https://github.com/Livox-SDK/Livox-SDK2.git
            fi
            
            cd ./Livox-SDK2/
            mkdir -p build && cd build
            cmake .. && make -j
            sudo make install
            
            # 更新库缓存
            sudo ldconfig
            
            cd ~/HzMi_rmsimulation
            print_success "Livox SDK2 安装完成"
        else
            print_warning "请手动安装 Livox SDK2 后重新运行此脚本"
            echo "参考：https://github.com/Livox-SDK/Livox-SDK2"
            exit 1
        fi
    else
        print_success "Livox SDK2 已安装"
    fi
}

# 安装 ROS 依赖
install_ros_deps() {
    print_info "安装 ROS 依赖..."
    
    if command -v rosdep &> /dev/null; then
        rosdep update 2>/dev/null || true
        rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y || {
            print_warning "rosdep 安装部分依赖失败，继续执行..."
        }
        print_success "ROS 依赖安装完成"
    else
        print_warning "rosdep 未安装，跳过 ROS 依赖安装"
    fi
}

# 编译工作空间
build_workspace() {
    print_info "编译工作空间..."
    
    # 清理之前的编译缓存（可选）
    if [ -d "build" ] && [ -d "install" ]; then
        read -p "是否清理之前的编译结果？(y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "清理编译缓存..."
            rm -rf build install log
        fi
    fi
    
    # 编译
    print_info "开始编译（这可能需要几分钟）..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_success "编译成功！"
    else
        print_error "编译失败，请检查错误信息"
        exit 1
    fi
}

# 创建启动脚本快捷方式
create_shortcuts() {
    print_info "创建启动脚本..."
    
    # 确保脚本有执行权限
    chmod +x start_sentinel.sh 2>/dev/null || true
    chmod +x save_pcd.sh 2>/dev/null || true
    chmod +x save_grid_map.sh 2>/dev/null || true
    chmod +x improved_teleop.sh 2>/dev/null || true
    
    print_success "脚本权限设置完成"
}

# 显示使用说明
show_usage() {
    echo ""
    print_success "=========================================="
    print_success "    HzMi_rmsimulation 配置完成！"
    print_success "=========================================="
    echo ""
    print_info "使用方法："
    echo ""
    echo "  1. 加载环境："
    echo "     source install/setup.bash"
    echo ""
    echo "  2. 启动仿真："
    echo "     tools/scripts/control/start_sentinel.sh"
    echo ""
    echo "  3. 查看帮助："
    echo "     tools/scripts/control/start_sentinel.sh --help"
    echo ""
    echo "  4. 键盘控制："
    echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo ""
    print_info "详细文档请查看：HZMI_RMSIMULATION_CONFIG_GUIDE.md"
    echo ""
}

# 主函数
main() {
    echo ""
    echo "========================================"
    echo "  HzMi_rmsimulation 一键配置工具"
    echo "========================================"
    echo ""
    
    check_directory
    check_ros2
    check_dependencies
    check_livox_sdk
    install_ros_deps
    build_workspace
    create_shortcuts
    show_usage
}

# 执行主函数
main
