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

add_executable(pure_persuit nodes/pure_persuit/pure_persuit.cpp)
#add_dependencies(pure_persuit smartcar_msgs smartcar_config_msgs)
target_link_libraries(pure_persuit ${catkin_LIBRARIES})
add_dependencies(pure_persuit ${catkin_EXPORTED_TARGETS})

