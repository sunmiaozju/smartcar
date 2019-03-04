cmake_minimum_required(VERSION 2.8.3)
project(filter_global_ground)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.7 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES filter_global_ground
 CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(filter_global_ground src/filter_global_ground_node.cpp src/filter_global_ground_core.cpp)
target_link_libraries(filter_global_ground   ${catkin_LIBRARIES} ${PCL_LIBRARIES})