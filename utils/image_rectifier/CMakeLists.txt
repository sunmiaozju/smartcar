cmake_minimum_required(VERSION 2.8.3)
project(image_rectifier)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp
             sensor_msgs
             cv_bridge)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
             sensor_msgs
             cv_bridge)




include_directories(
             include
             ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


add_executable(${PROJECT_NAME}_node 
             src/image_rectifier_node.cpp
             src/image_rectifier.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
             ${OpenCV_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}_node
             ${catkin_LIBRARIES}
             ${OpenCV_LIBS}
)
