#!/bin/bash

# Two-Wheel Robot Simulation Setup and Run Script
# This script helps users quickly set up and run the robot simulation

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
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

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check ROS2 installation
check_ros2() {
    print_info "Checking ROS2 installation..."
    if ! command_exists ros2; then
        print_error "ROS2 is not installed or not sourced. Please install ROS2 and source the setup script."
        exit 1
    fi
    print_success "ROS2 found"
}

# Function to check dependencies
check_dependencies() {
    print_info "Checking dependencies..."
    
    local missing_deps=()
    
    # Check for Gazebo
    if ! command_exists gazebo; then
        missing_deps+=("gazebo")
    fi
    
    # Check for colcon
    if ! command_exists colcon; then
        missing_deps+=("python3-colcon-common-extensions")
    fi
    
    # Check for ROS2 packages
    if ! ros2 pkg list | grep -q "tf2_geometry_msgs"; then
        missing_deps+=("ros-$ROS_DISTRO-tf2-geometry-msgs")
    fi
    
    if ! ros2 pkg list | grep -q "gazebo_ros"; then
        missing_deps+=("ros-$ROS_DISTRO-gazebo-ros-pkgs")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_warning "Missing dependencies: ${missing_deps[*]}"
        print_info "Would you like to install missing dependencies? (y/n)"
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            print_info "Installing missing dependencies..."
            chmod +x install_dependencies.sh
            ./install_dependencies.sh
            print_success "Dependencies installed"
        else
            print_warning "Some dependencies might be missing. You can install them later using:"
            echo "./install_dependencies.sh"
        fi
    else
        print_success "All dependencies satisfied"
    fi
}

# Function to build the package
build_package() {
    print_info "Building the package..."
    
    if [ ! -f "package.xml" ]; then
        print_error "package.xml not found. Make sure you're in the package directory."
        exit 1
    fi
    
    # Go to workspace root
    cd ../..
    
    # Build the package
    if colcon build --packages-select two_wheel_robot; then
        print_success "Package built successfully"
    else
        print_error "Package build failed"
        exit 1
    fi
    
    # Source the workspace
    source install/setup.bash
    print_success "Workspace sourced"
}

# Function to show available launch options
show_options() {
    echo
    print_info "Available launch options:"
    echo "1. Full simulation (Default)"
    echo "2. Simulation with autonomous navigation"
    echo "3. Simulation without teleop (autonomous only)"
    echo "4. Visualization tools only"
    echo "5. Robot dashboard (GUI)"
    echo "6. Custom configuration"
    echo
}

# Function to launch simulation
launch_simulation() {
    local option=$1
    
    case $option in
        1|"")
            print_info "Launching full simulation with manual control..."
            ros2 launch two_wheel_robot robot_simulation.launch.py
            ;;
        2)
            print_info "Launching simulation with autonomous navigation..."
            ros2 launch two_wheel_robot robot_simulation.launch.py use_autonomous:=true
            ;;
        3)
            print_info "Launching autonomous-only simulation..."
            ros2 launch two_wheel_robot robot_simulation.launch.py use_autonomous:=true use_teleop:=false
            ;;
        4)
            print_info "Launching visualization tools..."
            ros2 launch two_wheel_robot visualize_graph.launch.py
            ;;
        5)
            print_info "Launching robot dashboard..."
            ros2 run two_wheel_robot robot_dashboard.py
            ;;
        6)
            print_info "Custom configuration mode..."
            echo "Available parameters:"
            echo "  use_sim_time:=true/false"
            echo "  use_rviz:=true/false"
            echo "  use_teleop:=true/false"
            echo "  use_autonomous:=true/false"
            echo
            echo "Example: ros2 launch two_wheel_robot robot_simulation.launch.py use_rviz:=false"
            ;;
        *)
            print_error "Invalid option: $option"
            exit 1
            ;;
    esac
}

# Main execution
main() {
    echo "================================================"
    echo "    Two-Wheel Robot Simulation Setup"
    echo "================================================"
    echo
    
    # Check system requirements
    check_ros2
    check_dependencies
    
    # Build package if needed
    if [ "$1" = "--build" ] || [ "$1" = "-b" ]; then
        build_package
        shift
    fi
    
    # Show options if no argument provided
    if [ $# -eq 0 ]; then
        show_options
        echo -n "Select an option (1-6, default=1): "
        read -r option
        [ -z "$option" ] && option=1
    else
        option=$1
    fi
    
    # Launch based on selection
    launch_simulation "$option"
}

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS] [LAUNCH_OPTION]"
    echo
    echo "OPTIONS:"
    echo "  -h, --help     Show this help message"
    echo "  -b, --build    Build the package before launching"
    echo
    echo "LAUNCH_OPTIONS:"
    echo "  1              Full simulation (default)"
    echo "  2              Simulation with autonomous navigation"
    echo "  3              Autonomous-only simulation"
    echo "  4              Visualization tools only"
    echo "  5              Robot dashboard"
    echo "  6              Custom configuration"
    echo
    echo "Examples:"
    echo "  $0                    # Interactive mode"
    echo "  $0 2                  # Launch with autonomous navigation"
    echo "  $0 --build 1          # Build package and launch full simulation"
    echo
}

# Handle command line arguments
case $1 in
    -h|--help)
        show_help
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac