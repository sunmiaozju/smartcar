cmake_minimum_required(VERSION 2.8.3)
project(ray_ground_filter)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
)

find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ray_ground_filter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(ray_ground_filter_node src/ray_ground_filter_node.cpp src/ray_ground_filter_core.cpp)


target_link_libraries(ray_ground_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(ray_ground_filter_node
  # ${${PROJECT_NAME}_EXPORTED_TARGETS}
  # ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
