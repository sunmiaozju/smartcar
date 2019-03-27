cmake_minimum_required(VERSION 2.8.3)
project(bst_detection)

add_compile_options(-std=c++14)

add_compile_options(-Wall -g -D_REENTRANT)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  yunle_sensor_msgs
  cv_bridge
)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs yunle_sensor_msgs cv_bridge
)

include_directories(
  include/bst_detection/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda-9.0/include/
)

link_directories(
lib/
/usr/local/cuda-9.0/lib64/
)


add_executable(${PROJECT_NAME}_node src/detection_node.cpp)
target_link_libraries(${PROJECT_NAME}_node uff_object.a yunleApi.so nvinfer nvparsers nvinfer_plugin cudnn cublas cudart_static nvToolsExt cudart pthread rt dl ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

