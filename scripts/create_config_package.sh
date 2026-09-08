#!/bin/bash

################################################################################
# 创建 HzMi_rmsimulation 离线配置包
# 用于生成可分发的压缩包，包含所有必要的配置文件和文档
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
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否在正确的目录
if [ ! -d "src" ]; then
    print_error "请在 HzMi_rmsimulation 目录下运行此脚本"
    exit 1
fi

WORKSPACE_NAME="HzMi_rmsimulation"
PACKAGE_NAME="HzMi_rmsimulation_config_package"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

print_info "开始创建离线配置包..."
print_info "工作空间：$WORKSPACE_NAME"
print_info "时间戳：$TIMESTAMP"

# 创建临时目录
TEMP_DIR="/tmp/${PACKAGE_NAME}_${TIMESTAMP}"
print_info "创建临时目录：$TEMP_DIR"
mkdir -p "$TEMP_DIR"

# 复制必要文件
print_info "复制源文件..."

# 复制 src 目录（排除 build 和 install）
cp -r src "$TEMP_DIR/"

# 复制启动脚本
cp "${PROJECT_ROOT}/scripts/control/start_sentinel.sh" "$TEMP_DIR/" 2>/dev/null || print_info "start_sentinel.sh 不存在，跳过"
cp "${PROJECT_ROOT}/scripts/mapping/save_pcd.sh" "$TEMP_DIR/" 2>/dev/null || print_info "save_pcd.sh 不存在，跳过"
cp "${PROJECT_ROOT}/scripts/mapping/save_grid_map.sh" "$TEMP_DIR/" 2>/dev/null || print_info "save_grid_map.sh 不存在，跳过"
cp "${PROJECT_ROOT}/scripts/control/improved_teleop.sh" "$TEMP_DIR/" 2>/dev/null || print_info "improved_teleop.sh 不存在，跳过"
cp "${PROJECT_ROOT}/scripts/setup_from_package.sh" "$TEMP_DIR/" 2>/dev/null || print_info "setup_from_package.sh 不存在，跳过"

# 复制文档
cp "${PROJECT_ROOT}/README.md" "$TEMP_DIR/" 2>/dev/null || print_info "README.md 不存在，跳过"
cp "${PROJECT_ROOT}/docs/package/HZMI_RMSIMULATION_CONFIG_GUIDE.md" "$TEMP_DIR/"

# 复制 CMakeLists.txt 和 package.xml（如果存在）
if [ -f "CMakeLists.txt" ]; then
    cp CMakeLists.txt "$TEMP_DIR/"
fi

# 创建 .gitignore（防止误用 git）
cat > "$TEMP_DIR/.gitignore" << 'EOF'
# 编译产物
build/
install/
log/

# IDE 配置
.vscode/
.idea/
*.swp
*.swo
*~

# 系统文件
.DS_Store
Thumbs.db

# Python
__pycache__/
*.pyc
EOF

# 创建快速开始指南
cat > "$TEMP_DIR/QUICK_START.md" << 'EOF'
# 🚀 快速开始指南

## 第一步：运行配置脚本

```bash
cd HzMi_rmsimulation
chmod +x setup_from_package.sh
./setup_from_package.sh
```

这个脚本会自动：
- ✅ 检查并安装依赖
- ✅ 编译工作空间
- ✅ 设置执行权限

## 第二步：启动仿真

配置完成后：

```bash
source install/setup.bash
./start_sentinel.sh
```

## 第三步：控制机器人

新打开一个终端：

```bash
source ~/HzMi_rmsimulation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**按键说明：**
- `i` - 前进
- `,` - 后退  
- `j` - 左移
- `l` - 右移
- `u` - 左旋
- `o` - 右旋
- `空格` - 停止

## 常用命令

### 查看启动脚本帮助
```bash
./start_sentinel.sh --help
```

### 使用不同场地
```bash
# RMUL场地（默认）
./start_sentinel.sh

# RMUC 场地
./start_sentinel.sh -w RMUC
```

### 使用不同 LIO 算法
```bash
# Fast-LIO（默认）
./start_sentinel.sh

# Point-LIO
./start_sentinel.sh --lio pointlio
```

### 保存地图
```bash
# 保存点云地图
./save_pcd.sh

# 保存栅格地图
./save_grid_map.sh
```

## 遇到问题？

请查看详细文档：`HZMI_RMSIMULATION_CONFIG_GUIDE.md`

祝你使用愉快！🎉
EOF

# 复制到临时目录
cp "$TEMP_DIR/QUICK_START.md" "$TEMP_DIR/"

# 创建版本信息文件
cat > "$TEMP_DIR/VERSION.txt" << EOF
HzMi_rmsimulation 离线配置包
版本：1.0.0
创建时间：$TIMESTAMP
ROS2 版本：humble
Ubuntu 版本：22.04

包含内容:
- Gazebo 仿真环境 (RMUC/RMUL)
- Fast-LIO / Point-LIO
- Navigation2 导航
- 完整配置文件和示例

详细使用说明请查看:
1. QUICK_START.md (快速开始)
2. HZMI_RMSIMULATION_CONFIG_GUIDE.md (详细文档)
EOF

# 设置脚本执行权限
chmod +x "$TEMP_DIR"/*.sh 2>/dev/null || true

# 创建压缩包
cd /tmp
print_info "创建压缩包..."

# 创建 tar.gz 格式
tar -czf "${PACKAGE_NAME}_${TIMESTAMP}.tar.gz" "$WORKSPACE_NAME"

# 同时创建 zip 格式（如果安装了 zip）
if command -v zip &> /dev/null; then
    zip -rq "${PACKAGE_NAME}_${TIMESTAMP}.zip" "$WORKSPACE_NAME"
    print_success "创建完成："
    echo "  - ${PACKAGE_NAME}_${TIMESTAMP}.tar.gz"
    echo "  - ${PACKAGE_NAME}_${TIMESTAMP}.zip"
else
    print_success "创建完成："
    echo "  - ${PACKAGE_NAME}_${TIMESTAMP}.tar.gz"
    print_info "如需创建 zip 格式，请安装：sudo apt install zip"
fi

# 清理临时目录
print_info "清理临时文件..."
rm -rf "$TEMP_DIR"

# 移动压缩包到原目录
mv "/tmp/${PACKAGE_NAME}_${TIMESTAMP}."* "./${PACKAGE_NAME}_${TIMESTAMP}/" 2>/dev/null || true

print_success "=========================================="
print_success "    离线配置包创建成功！"
print_success "=========================================="
echo ""
print_info "位置：$(pwd)/${PACKAGE_NAME}_${TIMESTAMP}/"
echo ""
print_info "分发说明："
echo "  将压缩包发送给其他同学"
echo "  同学解压后运行 ./setup_from_package.sh 即可完成配置"
echo ""
print_info "包含的文档："
echo "  - QUICK_START.md (快速开始)"
echo "  - HZMI_RMSIMULATION_CONFIG_GUIDE.md (详细文档)"
echo "  - setup_from_package.sh (一键配置脚本)"
echo ""
