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
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  path_msgs
  visualization_msgs
  tf
)

# gennerate_dunamic...必须在catkin_package之前
generate_dynamic_reconfigure_options(
  cfg/global_planning_configure.cfg
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
  ../../common/utils/include
)

add_executable(global_plan_node src/global_plan_node.cpp)
target_link_libraries(global_plan_node ${catkin_LIBRARIES})
add_dependencies(global_plan_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_executable(load_static_path src/load_static_path.cpp)
target_link_libraries(load_static_path ${catkin_LIBRARIES})
add_dependencies(load_static_path
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_gencfg
)
























