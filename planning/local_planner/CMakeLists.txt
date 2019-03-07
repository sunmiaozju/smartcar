cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    pcl_conversions
    pcl_ros
    std_msgs
    smartcar_msgs
    sensor_msgs
    tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
        roscpp
        std_msgs
        smartcar_msgs
        geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(rollout_generator_node 
               nodes/rollout_generator/rollout_generator_node.cpp
               nodes/rollout_generator/rollout_generator.cpp
               utils/utils.cpp)
target_link_libraries(rollout_generator_node ${catkin_LIBRARIES})

add_executable(local_trajectory_generator_node 
               nodes/local_trajectory_generator/local_trajectory_generator_node.cpp
               nodes/local_trajectory_generator/local_trajectory_generator.cpp
               utils/utils.cpp)
target_link_libraries(local_trajectory_generator_node ${catkin_LIBRARIES})

add_dependencies(rollout_generator_node local_trajectory_generator_node ${catkin_EXPORTED_TARGETS})
