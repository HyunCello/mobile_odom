cmake_minimum_required(VERSION 3.0.2)
project(mobile_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
 #INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp roslib rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mobile_odom src/mobile_odom_node.cpp)
add_dependencies(mobile_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_odom ${catkin_LIBRARIES})
