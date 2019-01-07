cmake_minimum_required(VERSION 2.8.3)
project(voxel_ground_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  rospy
)

find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARIES_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy pcl_ros sensor_msgs
)

add_executable(voxel_ground_filter_node src/voxel_ground_filter_node.cpp src/voxel_ground_filter_core.cpp)
target_link_libraries(voxel_ground_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

