cmake_minimum_required(VERSION 2.8.12)
project(joint_pixel_pointcloud)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

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
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(joint_pixel_pointcloud_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_dependencies(joint_pixel_pointcloud_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(joint_pixel_pointcloud
        src/joint_pixel_pointcloud_node.cpp
        )
target_include_directories(joint_pixel_pointcloud PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(joint_pixel_pointcloud joint_pixel_pointcloud_lib)




