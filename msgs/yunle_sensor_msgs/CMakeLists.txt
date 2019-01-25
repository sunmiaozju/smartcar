cmake_minimum_required(VERSION 2.8.3)
project(yunle_sensor_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  LanePoint.msg
  DetectObject.msg
  DetectLane.msg
  DetectLanes.msg
  DetectObjs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
