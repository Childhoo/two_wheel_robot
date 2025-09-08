cmake_minimum_required(VERSION 3.8)
project(two_wheel_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable for autonomous navigation
add_executable(autonomous_navigator src/autonomous_navigator.cpp)
ament_target_dependencies(autonomous_navigator
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Add executable for robot state monitor
add_executable(robot_monitor src/robot_monitor.cpp)
ament_target_dependencies(robot_monitor
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install executables
install(TARGETS
  autonomous_navigator
  robot_monitor
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch
  urdf
  config
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/robot_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()