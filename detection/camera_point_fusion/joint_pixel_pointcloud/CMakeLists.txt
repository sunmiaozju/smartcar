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
        yunle_sensor_msgs
        )
 
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        yunle_sensor_msgs
        )

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

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

target_link_libraries(joint_pixel_pointcloud
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_dependencies(joint_pixel_pointcloud
        ${catkin_EXPORTED_TARGETS}
        )

if(OPENMP_FOUND)
    set_target_properties(joint_pixel_pointcloud PROPERTIES
                        COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
                        LINK_FLAGS ${OpenMP_CXX_FLAGS})      
endif()




