cmake_minimum_required(VERSION 2.8.3)
project(gateway)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gencpp
  genpy
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES monitor
#  CATKIN_DEPENDS gencpp genpy geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


