#!/bin/bash

# 自动创建ROS2工作空间和two_wheel_robot包结构的脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 默认工作空间路径
WORKSPACE_PATH="$HOME/ros2_ws"
PACKAGE_NAME="two_wheel_robot"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--workspace)
            WORKSPACE_PATH="$2"
            shift 2
            ;;
        -h|--help)
            echo "使用方法: $0 [选项]"
            echo "选项:"
            echo "  -w, --workspace PATH  指定工作空间路径 (默认: $HOME/ros2_ws)"
            echo "  -h, --help           显示此帮助信息"
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

echo "================================================"
echo "     ROS2 Two-Wheel Robot 工作空间创建器"  
echo "================================================"
echo

print_info "工作空间路径: $WORKSPACE_PATH"
print_info "包名称: $PACKAGE_NAME"
echo

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS_DISTRO环境变量未设置"
    print_info "尝试source ROS2环境..."
    
    # 尝试常见的ROS2安装路径
    for distro in humble galactic foxy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source /opt/ros/$distro/setup.bash
            print_success "已自动source ROS2 $distro"
            break
        fi
    done
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "无法找到ROS2安装。请先安装ROS2或手动source环境。"
        exit 1
    fi
fi

print_success "ROS2分布: $ROS_DISTRO"

# 创建工作空间结构
print_info "创建工作空间结构..."

# 创建主目录
mkdir -p "$WORKSPACE_PATH/src"
cd "$WORKSPACE_PATH"

# 创建包目录
PACKAGE_PATH="$WORKSPACE_PATH/src/$PACKAGE_NAME"
mkdir -p "$PACKAGE_PATH"

# 创建所有子目录
mkdir -p "$PACKAGE_PATH/src"
mkdir -p "$PACKAGE_PATH/scripts"
mkdir -p "$PACKAGE_PATH/launch"
mkdir -p "$PACKAGE_PATH/urdf"
mkdir -p "$PACKAGE_PATH/config"
mkdir -p "$PACKAGE_PATH/worlds"

print_success "目录结构创建完成"

# 显示目录结构
print_info "创建的目录结构:"
tree "$PACKAGE_PATH" 2>/dev/null || find "$PACKAGE_PATH" -type d | sed 's|[^/]*/|  |g'

# 创建文件放置指南
PLACEMENT_GUIDE="$PACKAGE_PATH/FILE_PLACEMENT_GUIDE.txt"
cat > "$PLACEMENT_GUIDE" << 'EOF'
文件放置指南
============

请将文件按以下结构放置：

根目录文件 (~/ros2_ws/src/two_wheel_robot/):
├── package.xml                    # ROS2包配置文件
├── CMakeLists.txt                 # CMake构建配置
├── README.md                      # 项目说明文档
├── setup_and_run.sh              # 快速启动脚本
├── install_dependencies.sh       # 依赖安装脚本
└── TROUBLESHOOTING.md            # 故障排除指南

src/ (C++源文件):
├── robot_monitor.cpp             # 机器人状态监控器
└── autonomous_navigator.cpp      # 自动导航控制器

scripts/ (Python脚本):
└── robot_dashboard.py           # GUI监控面板

launch/ (启动文件):
├── robot_simulation.launch.py   # 主仿真启动文件
└── visualize_graph.launch.py    # 计算图可视化

urdf/ (机器人模型):
└── two_wheel_robot.urdf.xacro   # 机器人URDF描述

config/ (配置文件):
└── robot_view.rviz              # RViz可视化配置

worlds/ (Gazebo世界):
└── simple_world.world           # 仿真环境描述

完成文件放置后，运行以下命令：
1. chmod +x setup_and_run.sh install_dependencies.sh
2. ./install_dependencies.sh
3. ./setup_and_run.sh --build
EOF

print_success "文件放置指南已创建: $PLACEMENT_GUIDE"

# 创建快速开始脚本
QUICK_START="$PACKAGE_PATH/quick_start.sh"
cat > "$QUICK_START" << EOF
#!/bin/bash
# 快速开始脚本

echo "Two-Wheel Robot Package 快速开始"
echo "================================"

# 检查文件是否存在
if [ ! -f "package.xml" ]; then
    echo "❌ 错误: package.xml 文件未找到"
    echo "请先按照 FILE_PLACEMENT_GUIDE.txt 放置所有文件"
    exit 1
fi

# 进入工作空间根目录
cd "$WORKSPACE_PATH"

# Source ROS2环境
source /opt/ros/$ROS_DISTRO/setup.bash

echo "✅ 正在安装依赖..."
cd src/$PACKAGE_NAME
chmod +x install_dependencies.sh
./install_dependencies.sh

echo "✅ 正在构建包..."
cd "$WORKSPACE_PATH"
colcon build --packages-select $PACKAGE_NAME

echo "✅ 正在source工作空间..."
source install/setup.bash

echo "✅ 启动仿真..."
ros2 launch $PACKAGE_NAME robot_simulation.launch.py
EOF

chmod +x "$QUICK_START"
print_success "快速开始脚本已创建: $QUICK_START"

# 创建环境设置脚本
ENV_SETUP="$WORKSPACE_PATH/setup_env.sh"
cat > "$ENV_SETUP" << EOF
#!/bin/bash
# 环境设置脚本

echo "设置ROS2环境..."

# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash

# Source工作空间 (如果已构建)
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    source "$WORKSPACE_PATH/install/setup.bash"
    echo "✅ 工作空间环境已加载"
else
    echo "⚠️  工作空间尚未构建，请先运行 colcon build"
fi

# 设置一些有用的别名
alias ws_build="cd $WORKSPACE_PATH && colcon build --packages-select $PACKAGE_NAME"
alias ws_source="source $WORKSPACE_PATH/install/setup.bash"
alias ws_launch="ros2 launch $PACKAGE_NAME robot_simulation.launch.py"

echo "可用的别名:"
echo "  ws_build  - 构建包"
echo "  ws_source - source工作空间"
echo "  ws_launch - 启动仿真"
EOF

chmod +x "$ENV_SETUP"
print_success "环境设置脚本已创建: $ENV_SETUP"

echo
print_success "工作空间创建完成！"
echo
print_info "下一步操作:"
echo "1. 📁 将所有项目文件复制到: $PACKAGE_PATH"
echo "2. 📖 参考文件放置指南: $PACKAGE_PATH/FILE_PLACEMENT_GUIDE.txt"  
echo "3. 🚀 运行快速开始: $PACKAGE_PATH/quick_start.sh"
echo
print_info "或者手动步骤:"
echo "cd $PACKAGE_PATH"
echo "./install_dependencies.sh"
echo "cd $WORKSPACE_PATH"
echo "colcon build --packages-select $PACKAGE_NAME"
echo "source install/setup.bash"
echo "ros2 launch $PACKAGE_NAME robot_simulation.launch.py"
echo
print_info "环境设置:"
echo "每次打开新终端时运行: source $ENV_SETUP"
EOF