cmake_minimum_required(VERSION 2.8.12)
project(points2image)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        smartcar_msgs
        rosinterface
        fastvirtualscan
        cv_bridge
        tf
        pcl_ros
        pcl_conversions
        )

find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(OpenGL REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp
        std_msgs
        sensor_msgs
        smartcar_msgs
        rosinterface
        fastvirtualscan
        cv_bridge
        tf
        pcl_ros
        pcl_conversions
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

# library
add_library(points_image
        lib/points_image/points_image.cpp
        )
add_dependencies(points_image ${catkin_EXPORTED_TARGETS})
target_link_libraries(points_image
        ${OpenCV_LIBS}
        )

# points2image
add_executable(points2image nodes/points2image/points2image.cpp)
target_link_libraries(points2image
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        points_image
        )
add_dependencies(points2image
        ${catkin_EXPORTED_TARGETS}
        )

