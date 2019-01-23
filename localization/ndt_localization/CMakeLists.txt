cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
find_package(CUDA)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosbag
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  # ${PCL_OPENMP_PACKAGES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf ${PCL_OPENMP_PACKAGES}
 DEPENDS system_lib PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})  # 可以将libgpu_ndt.so加到这个路径里，然后就可以链接到了

include_directories(include)
# link_directories(lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ../../common/user_protocol/include
  ../../common/utils/include
)

add_library(ndt_localization
  src/ndt_localization.cpp
)

target_link_libraries(ndt_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  ndt_gpu
  ndt_cpu
  pcl_omp_registration
)

if (CUDA_FOUND)
    target_include_directories(ndt_localization PRIVATE ${CUDA_INCLUDE_DIRS})
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_localization PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

add_executable(ndt_localization_node
  src/ndt_localization_node.cpp
)

add_dependencies(ndt_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ndt_localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_localization
)

add_executable(ndt_test
  src/ndt_test.cpp
)

target_link_libraries(ndt_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(tf_test
  src/tf_test.cpp
)

add_dependencies(tf_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tf_test
  ${catkin_LIBRARIES}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ndt_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ndt_localization_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ndt_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
