cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)

add_compile_options(-std=c++11)

set(SOURCE_FILES pure_persuit.cpp pure_persuit.h)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
        tf
        pcl_ros
        roscpp
        std_msgs
        geometry_msgs
        smartcar_msgs
        smartcar_config_msgs
        )

catkin_package(
        CATKIN_DEPENDS roscpp
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        smartcar_msgs
        smartcar_config_msgs
        pcl_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pure_persuit_lib SHARED
            nodes/pure_persuit/pure_persuit.cpp 
            nodes/pure_persuit/pure_persuit.h)
target_link_libraries(pure_persuit_lib 
                      ${catkin_LIBRARIES})
target_include_directories(pure_persuit_lib PRIVATE       
                          ${catkin_INCLUDE_DIRS})
add_dependencies(pure_persuit_lib
                 ${catkin_EXPORTED_TARGETS})


add_executable(pure_persuit_node 
               nodes/pure_persuit/pure_persuit_node.cpp)
target_link_libraries(pure_persuit_node
                     ${catkin_LIBRARIES})
target_include_directories(pure_persuit_node PRIVATE
                           ${catkin_INCLUDE_DIRS})
add_dependencies(pure_persuit_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pure_persuit_node pure_persuit_lib)

