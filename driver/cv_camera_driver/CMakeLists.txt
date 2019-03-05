cmake_minimum_required(VERSION 2.8.3)
project(cv_camera)

find_package(catkin REQUIRED COMPONENTS
  image_transport 
  roscpp
  cv_bridge 
  sensor_msgs 
  nodelet 
  camera_info_manager 
  roslint)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cv_camera
  CATKIN_DEPENDS image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager
  DEPENDS OpenCV
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(cv_camera src/capture.cpp src/driver.cpp)
add_library(cv_camera_nodelet src/cv_camera_nodelet.cpp)


## Declare a cpp executable
add_executable(cv_camera_node src/cv_camera_node.cpp)
add_dependencies(cv_camera_node cv_camera)
add_dependencies(cv_camera_nodelet cv_camera)

## Specify libraries to link a library or executable target against
target_link_libraries(cv_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

target_link_libraries(cv_camera_nodelet
  ${catkin_LIBRARIES}
  cv_camera
  )

target_link_libraries(cv_camera_node
  ${catkin_LIBRARIES}
  cv_camera
  )

