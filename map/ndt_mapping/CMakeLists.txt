cmake_minimum_required(VERSION 2.8.3)
project(ndt_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.7 REQUIRED)
find_package(CUDA)
if (CUDA_FOUND)
	add_definitions(-DCUDA_FOUND)
	include_directories(${CUDA_INCLUDE_DIRS})
endif ()
include_directories(include)
# link_directories(lib)
string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})  # 可以将libgpu_ndt.so加到这个路径里，然后就可以链接到了

find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  nav_msgs
  pcl_conversions
  sensor_msgs
	tf
	std_msgs
	geometry_msgs
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fast_ndt_slam
  CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ../../common/utils/include
)


add_executable(fast_ndt_mapping_node src/fast_ndt_mapping_node.cpp src/LidarMapping.cpp)
target_link_libraries(fast_ndt_mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(fast_ndt_mapping_node ndt_cpu pcl_omp_registration)

if (CUDA_FOUND)
	target_link_libraries(fast_ndt_mapping_node ndt_gpu)
endif ()
# add_dependencies(fast_ndt_mapping_node ${catkin_EXPORTED_TARGETS} )

# 正确顺序  --错误顺序将导致编译失败!!!!!
#find_package()
#catkin_package()
#include_directories()
#
#add_executable()
#add_library()
#target_link_libraries()
