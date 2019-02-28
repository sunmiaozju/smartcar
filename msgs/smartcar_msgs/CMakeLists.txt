cmake_minimum_required(VERSION 2.8.3)

project(smartcar_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        sensor_msgs
        message_generation
        )

add_message_files(
        FILES
        Lane.msg
        ControlCommand.msg
        ControlCommandStamped.msg
        Waypoint.msg
        LaneArray.msg
        PointsImage.msg
        ProjectionMatrix.msg
        ImageObj.msg
        ImageObjects.msg
        ImageObjTracked.msg
        ImageObjRanged.msg
        ImageRect.msg
        ImageRectRanged.msg
        DetectedObject.msg
        DetectedObjectArray.msg
        TrafficLightResult.msg
        TrafficLight.msg
        TrafficLightResultArray.msg
        ImageLaneObjects.msg


)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)



