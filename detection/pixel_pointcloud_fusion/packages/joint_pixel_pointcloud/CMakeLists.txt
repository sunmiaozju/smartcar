cmake_minimum_required(VERSION 2.8.12)
project(joint_pixel_pointcloud)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

#fusion Library
add_library(joint_pixel_pointcloud_lib SHARED
        src/joint_pixel_pointcloud.cpp
        include/joint_pixel_pointcloud.h
        )

target_include_directories(joint_pixel_pointcloud_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(joint_pixel_pointcloud_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_dependencies(joint_pixel_pointcloud_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(joint_pixel_pointcloud
        src/joint_pixel_pointcloud_node.cpp
        src/joint_pixel_pointcloud.cpp
        )

target_include_directories(joint_pixel_pointcloud PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(joint_pixel_pointcloud joint_pixel_pointcloud_lib)




