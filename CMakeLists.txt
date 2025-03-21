cmake_minimum_required(VERSION 3.5)
project(mobile_odom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Create the mobile_odom executable
add_executable(mobile_odom src/mobile_odom_node.cpp)
ament_target_dependencies(mobile_odom
  nav_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install executables and directories
install(TARGETS
  mobile_odom
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(geometry_msgs rclcpp sensor_msgs nav_msgs std_msgs tf2 tf2_ros)

# Generate package
ament_package()
