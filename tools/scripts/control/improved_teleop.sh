#!/bin/bash

# 改进版机器人键盘控制脚本
# 添加系统状态检查和更好的错误处理
#
# ===== 键位（2026-09-21 改为方向键操作）=====
#   ↑ / ↓         前进 / 后退（线速度 LINEAR_SPEED，默认 0.5 m/s）
#   ← / →         左转 / 右转（角速度 ANGULAR_SPEED，默认 1.0 rad/s）
#   空格          紧急停止
#   > / <         线速度 +0.1 / -0.1（0.1 ~ 2.0 m/s）
#   . / ,         角速度 +0.1 / -0.1（0.2 ~ 3.0 rad/s）
#   0             速度重置为默认
#   s / h         系统状态 / 帮助
#   q             退出（退出前自动发一次零速）
#   兼容旧键位：i/k/j/l = 前进/后退/左转/右转
#
# 说明：按一下发一次 Twist（ros2 topic pub -1）；底盘会【保持】该速度，
#       所以"停车"必须发零速 —— 按空格（或 q 退出时会自动停）。
# 话题：默认 /cmd_vel_chassis（直连底盘，绕过 fake_vel_transform）；
#       需要走 nav2 那条链（经 fake_vel_transform）时用：
#         TELEOP_TOPIC=/cmd_vel tools/scripts/control/improved_teleop.sh

# ===== 自动定位项目根目录（脚本可在任意位置被调用）=====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${PROJECT_ROOT}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

# 默认参数；话题可用环境变量覆盖：TELEOP_TOPIC=/cmd_vel ...（默认直连底盘）
LINEAR_SPEED=0.5
ANGULAR_SPEED=1.0
CMD_TOPIC="${TELEOP_TOPIC:-/cmd_vel_chassis}"
MAX_RETRIES=3

echo -e "${BLUE}===============================================${NC}"
echo -e "${BLUE}  🤖 改进版机器人键盘控制${NC}"
echo -e "${BLUE}===============================================${NC}"
echo ""

# 检查ROS2环境
if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "${RED}❌ 未检测到ROS2环境${NC}"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查ROS2核心
echo -e "${YELLOW}🔍 检查ROS2核心状态...${NC}"
if ! timeout 3 ros2 node list &>/dev/null; then
    echo -e "${YELLOW}⚠️  启动ROS2核心...${NC}"
    ros2 daemon stop 2>/dev/null || true
    ros2 daemon start
    sleep 2
fi

# 检查控制话题是否存在
echo -e "${YELLOW}🔍 检查控制话题...${NC}"
if ! ros2 topic list | grep -q "^$CMD_TOPIC$"; then
    echo -e "${RED}❌ 控制话题 $CMD_TOPIC 不存在${NC}"
    echo "可用的控制话题:"
    ros2 topic list | grep -E "(cmd_vel|chassis|control)" || echo "  未找到相关话题"
    exit 1
fi

# 检查控制器节点状态
echo -e "${YELLOW}🔍 检查控制器状态...${NC}"
if ros2 node list | grep -q "/mecanum_controller"; then
    echo -e "${GREEN}✅ mecanum控制器在线${NC}"
else
    echo -e "${YELLOW}⚠️  mecanum控制器未找到，检查其他控制器...${NC}"
    # 列出可能的控制器节点
    echo "可用的控制器相关节点:"
    ros2 node list | grep -E "(controller|mecanum|chassis)" || echo "  未找到控制器节点"
fi

echo -e "${GREEN}✅ 系统检查完成${NC}"
echo ""

# 显示控制说明
echo -e "${BLUE}控制说明:${NC}"
echo "  i - 前进    k - 后退"
echo "  j - 左转    l - 右转"
echo "  空格 - 停止  q/z - 加速/减速"
echo "  s - 系统状态  h - 帮助"
echo "  Ctrl+C - 退出"
echo ""

# 信号处理
cleanup() {
    echo ""
    echo -e "${YELLOW}⏹  发送停止指令...${NC}"
    ros2 topic pub -1 "$CMD_TOPIC" geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
    echo -e "${GREEN}✅ 程序退出${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 发送速度命令函数（带重试机制）
send_cmd() {
    local linear=$1
    local angular=$2
    local retries=0
    
    while [[ $retries -lt $MAX_RETRIES ]]; do
        if ros2 topic pub -1 "$CMD_TOPIC" geometry_msgs/msg/Twist "{linear: {x: $linear, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $angular}}" >/dev/null 2>&1; then
            return 0  # 成功发送
        else
            ((retries++))
            if [[ $retries -lt $MAX_RETRIES ]]; then
                echo -e "${YELLOW}⚠️  发送失败，重试 ($retries/$MAX_RETRIES)...${NC}"
                sleep 0.1
            fi
        fi
    done
    
    echo -e "${RED}❌ 发送指令失败，已达最大重试次数${NC}"
    return 1
}

# 显示系统状态
show_system_status() {
    echo ""
    echo -e "${BLUE}📊 系统状态:${NC}"
    
    # 检查控制器
    if ros2 node list | grep -q "/mecanum_controller"; then
        echo -e "  ${GREEN}✓ mecanum控制器: 在线${NC}"
    else
        echo -e "  ${RED}✗ mecanum控制器: 离线${NC}"
    fi
    
    # 检查话题状态
    echo -e "  ${BLUE}控制话题: $CMD_TOPIC${NC}"
    if ros2 topic list | grep -q "^$CMD_TOPIC$"; then
        local pub_count=$(ros2 topic info "$CMD_TOPIC" | grep "Publisher count" | awk '{print $3}')
        local sub_count=$(ros2 topic info "$CMD_TOPIC" | grep "Subscription count" | awk '{print $3}')
        echo -e "  ${GREEN}✓ 话题状态: ${pub_count}发布者, ${sub_count}订阅者${NC}"
    else
        echo -e "  ${RED}✗ 话题不存在${NC}"
    fi
    
    # 显示当前速度设置
    echo -e "  ${BLUE}当前速度设置:${NC}"
    echo -e "    线速度: ${LINEAR_SPEED} m/s"
    echo -e "    角速度: ${ANGULAR_SPEED} rad/s"
    
    # 检查是否有数据流动（简单测试）
    echo -e "  ${BLUE}测试控制命令...${NC}"
    if send_cmd 0.0 0.0; then
        echo -e "  ${GREEN}✓ 控制命令发送成功${NC}"
    else
        echo -e "  ${RED}✗ 控制命令发送失败${NC}"
        echo -e "  ${YELLOW}💡 建议检查机器人是否已完全启动${NC}"
    fi
    echo ""
}

echo -e "${GREEN}🎮 键盘控制已启动（方向键）${NC}"
echo -e "  ↑/↓ 前进/后退    ←/→ 左转/右转    空格 停车"
echo -e "  >/< 线速度±0.1   ./, 角速度±0.1    0 重置   s 状态   h 帮助   q 退出"
echo ""

# 读一个按键：方向键是 ESC 序列（ESC [ A/B/C/D），需要单独解析；
# 用全局变量 KEY 返回，避免 $(...) 子壳层读 stdin 的坑。
read_key() {
    local c1 c2
    KEY=""
    IFS= read -rsn1 KEY || return 1
    if [[ "$KEY" == $'\x1b' ]]; then
        IFS= read -rsn1 -t 0.1 c1 && KEY+="$c1"
        IFS= read -rsn1 -t 0.1 c2 && KEY+="$c2"
    fi
    return 0
}

# 主循环
while true; do
    read_key || continue

    case "$KEY" in
        # ===== 方向键（ESC 序列）=====
        $'\x1b[A')
            echo -e "${BLUE}↑ 前进 (线速度: ${LINEAR_SPEED})${NC}"
            send_cmd "$LINEAR_SPEED" 0.0
            ;;
        $'\x1b[B')
            echo -e "${BLUE}↓ 后退 (线速度: ${LINEAR_SPEED})${NC}"
            send_cmd "-$LINEAR_SPEED" 0.0
            ;;
        $'\x1b[D')
            echo -e "${BLUE}← 左转 (角速度: ${ANGULAR_SPEED})${NC}"
            send_cmd 0.0 "$ANGULAR_SPEED"
            ;;
        $'\x1b[C')
            echo -e "${BLUE}→ 右转 (角速度: ${ANGULAR_SPEED})${NC}"
            send_cmd 0.0 "-$ANGULAR_SPEED"
            ;;
        # ===== 兼容旧键位 =====
        'i'|'I')
            echo -e "${BLUE}→ 前进 (线速度: ${LINEAR_SPEED})${NC}"
            send_cmd "$LINEAR_SPEED" 0.0
            ;;
        'k'|'K')
            echo -e "${BLUE}← 后退 (线速度: ${LINEAR_SPEED})${NC}"
            send_cmd "-$LINEAR_SPEED" 0.0
            ;;
        'j'|'J')
            echo -e "${BLUE}↺ 左转 (角速度: ${ANGULAR_SPEED})${NC}"
            send_cmd 0.0 "$ANGULAR_SPEED"
            ;;
        'l'|'L')
            echo -e "${BLUE}↻ 右转 (角速度: ${ANGULAR_SPEED})${NC}"
            send_cmd 0.0 "-$ANGULAR_SPEED"
            ;;
        ' ')
            echo -e "${YELLOW}⏹ 紧急停止${NC}"
            send_cmd 0.0 0.0
            ;;
        '>')
            LINEAR_SPEED=$(echo "$LINEAR_SPEED + 0.1" | bc -l)
            if (( $(echo "$LINEAR_SPEED > 2.0" | bc -l) )); then LINEAR_SPEED=2.0; fi
            echo -e "${GREEN}⚡ 线速度 +0.1 → ${LINEAR_SPEED} m/s${NC}"
            ;;
        '<')
            LINEAR_SPEED=$(echo "$LINEAR_SPEED - 0.1" | bc -l)
            if (( $(echo "$LINEAR_SPEED < 0.1" | bc -l) )); then LINEAR_SPEED=0.1; fi
            echo -e "${GREEN}⚡ 线速度 -0.1 → ${LINEAR_SPEED} m/s${NC}"
            ;;
        '.')
            ANGULAR_SPEED=$(echo "$ANGULAR_SPEED + 0.1" | bc -l)
            if (( $(echo "$ANGULAR_SPEED > 3.0" | bc -l) )); then ANGULAR_SPEED=3.0; fi
            echo -e "${GREEN}⚡ 角速度 +0.1 → ${ANGULAR_SPEED} rad/s${NC}"
            ;;
        ',')
            ANGULAR_SPEED=$(echo "$ANGULAR_SPEED - 0.1" | bc -l)
            if (( $(echo "$ANGULAR_SPEED < 0.2" | bc -l) )); then ANGULAR_SPEED=0.2; fi
            echo -e "${GREEN}⚡ 角速度 -0.1 → ${ANGULAR_SPEED} rad/s${NC}"
            ;;
        'q'|'Q')
            echo -e "${YELLOW}👋 退出（先发零速停车）${NC}"
            send_cmd 0.0 0.0 || true
            exit 0
            ;;
        '[')
            ANGULAR_SPEED=$(echo "$ANGULAR_SPEED + 0.1" | bc -l)
            if (( $(echo "$ANGULAR_SPEED > 3.0" | bc -l) )); then ANGULAR_SPEED=3.0; fi
            echo -e "${GREEN}⚡ 角速度调整为: ${ANGULAR_SPEED} rad/s${NC}"
            ;;
        ']')
            ANGULAR_SPEED=$(echo "$ANGULAR_SPEED - 0.1" | bc -l)
            if (( $(echo "$ANGULAR_SPEED < 0.2" | bc -l) )); then ANGULAR_SPEED=0.2; fi
            echo -e "${GREEN}⚡ 角速度调整为: ${ANGULAR_SPEED} rad/s${NC}"
            ;;
        '0')
            LINEAR_SPEED=0.5
            ANGULAR_SPEED=1.0
            echo -e "${GREEN}⚡ 速度已重置为默认值${NC}"
            ;;
        's'|'S')
            show_system_status
            ;;
        'h'|'H')
            echo ""
            echo -e "${BLUE}📖 帮助信息:${NC}"
            echo "  ↑ / ↓       : 前进 / 后退"
            echo "  ← / →       : 左转 / 右转"
            echo "  空格        : 紧急停止"
            echo "  > / <       : 线速度 +0.1 / -0.1  (0.1 ~ 2.0 m/s)"
            echo "  . / ,       : 角速度 +0.1 / -0.1  (0.2 ~ 3.0 rad/s)"
            echo "  0           : 重置速度"
            echo "  s           : 系统状态"
            echo "  h           : 显示帮助"
            echo "  q           : 退出（自动停）"
            echo "  兼容旧键: i/k/j/l = 前进/后退/左转/右转；[/] = 角速度 +/-"
            echo ""
            echo "  当前设置:"
            echo "    线速度: ${LINEAR_SPEED} m/s"
            echo "    角速度: ${ANGULAR_SPEED} rad/s"
            echo "    控制话题: $CMD_TOPIC"
            echo ""
            ;;
        *)
            # 忽略其他按键
            ;;
    esac
done