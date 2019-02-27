cmake_minimum_required(VERSION 2.8.3)
project(global_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  path_msgs
  visualization_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_nav_path
#  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(global_plan_node src/global_plan_node.cpp)
target_link_libraries(global_plan_node ${catkin_LIBRARIES})


























