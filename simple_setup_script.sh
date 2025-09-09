#!/bin/bash
# 超简单的Docker启动脚本

set -e

echo "🤖 ROS2 机器人仿真 - 简化版"
echo "=========================="

# 检查Docker
if ! command -v docker &> /dev/null; then
    echo "❌ Docker未安装，请先安装Docker"
    exit 1
fi

# 检查Docker是否运行
if ! docker info &> /dev/null; then
    echo "❌ Docker未运行，请先启动Docker"
    exit 1
fi

case "${1:-help}" in
    "build")
        echo "🔨 构建镜像..."
        docker build -t simple-ros2-robot .
        echo "✅ 构建完成"
        ;;
    
    "run")
        echo "🚀 启动仿真..."
        docker run --rm -it \
            --name robot-sim \
            simple-ros2-robot auto
        ;;
    
    "monitor")
        echo "📊 启动监控..."
        docker run --rm -it \
            --name robot-monitor \
            simple-ros2-robot monitor
        ;;
    
    "shell")
        echo "🐚 进入容器..."
        docker run --rm -it \
            simple-ros2-robot bash
        ;;
    
    "stop")
        echo "⏹️ 停止容器..."
        docker stop robot-sim 2>/dev/null || true
        docker stop robot-monitor 2>/dev/null || true
        ;;
    
    "clean")
        echo "🧹 清理..."
        docker system prune -f
        ;;
    
    *)
        echo "用法: $0 [命令]"
        echo ""
        echo "命令:"
        echo "  build   - 构建Docker镜像"
        echo "  run     - 运行机器人仿真"
        echo "  monitor - 只运行状态监控"
        echo "  shell   - 进入容器shell"
        echo "  stop    - 停止所有容器"
        echo "  clean   - 清理Docker缓存"
        echo ""
        echo "示例:"
        echo "  $0 build"
        echo "  $0 run"
        ;;
esac