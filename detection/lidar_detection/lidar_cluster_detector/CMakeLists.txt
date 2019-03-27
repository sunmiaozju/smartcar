cmake_minimum_required(VERSION 2.8.3)
project(lidar_cluster_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
            pcl_ros
            roscpp
            geometry_msgs
            std_msgs
            sensor_msgs
            smartcar_msgs
            tf)

find_package(OpenMP)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
            INCLUDE_DIRS include
            CATKIN_DEPENDS 
            pcl_ros
            roscpp
            geometry_msgs
            std_msgs
            sensor_msgs
            smartcar_msgs
            tf
)
include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(lidar_euclidean_cluster_node 
               src/lidar_euclidean_cluster_node.cpp
               src/lidar_euclidean_cluster.cpp
               src/cluster.cpp)

target_link_libraries(lidar_euclidean_cluster_node
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})

add_dependencies(lidar_euclidean_cluster_node
                $(catkin_EXPORTED_TARGETS))

if(OPENMP_FOUND)
    set_target_properties(lidar_euclidean_cluster_node PROPERTIES
                         COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
                         LINK_FLAGS ${OpenMP_CXX_FLAGS})      
endif()
