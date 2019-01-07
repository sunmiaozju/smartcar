cmake_minimum_required(VERSION 2.8.3)
project(hall_encoder)

find_package(catkin REQUIRED
        geometry_msgs
        roscpp
        rospy
        std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/hall_encoder_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
