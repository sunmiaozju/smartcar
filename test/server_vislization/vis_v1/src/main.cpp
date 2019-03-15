#include <dirent.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sstream>

#include <path_msgs/Cross.h>
#include <path_msgs/Lane.h>
#include <path_msgs/choose.h>

// #include <Eigen/Core>
// #include <Eigen/Eigen>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "utils.hpp"
#include <dynamic_reconfigure/server.h>
#include <global_planning/GlobalPlanningConfig.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/Waypoint.h>

namespace VISUALIZATION_V1 {
class monitor {
private:
    ros::Subscriber sub_car0_pose;
    ros::Publisher pub_car0_model;

    void car_pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

    visualization_msgs::Marker marker_initial();

public:
    void run();
};
}

namespace VISUALIZATION_V1 {
void monitor::run()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // sub_car0_pose = nh.subscribe("/car0/current_pose", 1, &monitor::car_pose_cb, this);
    sub_car0_pose = nh.subscribe("/ndt/current_pose", 1, &monitor::car_pose_cb, this);
    pub_car0_model = nh.advertise<visualization_msgs::Marker>("/car0/Car_model", 10);

    ros::spin();
}

void monitor::car_pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    visualization_msgs::Marker car_model = marker_initial();
    car_model.header.stamp = ros::Time::now();
    car_model.pose.position = msg->pose.position;
    double current_roll, current_yaw, current_pitch;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);
    car_model.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(90 * (M_PI / 180.0),
        0 * (M_PI / 180.0),
        current_yaw + M_PI / 2.0);
    pub_car0_model.publish(car_model);
    // ros::Duration(0.05).sleep();
}

visualization_msgs::Marker monitor::marker_initial()
{
    visualization_msgs::Marker marker_car;
    marker_car.header.frame_id = "map";
    marker_car.header.stamp = ros::Time();
    marker_car.ns = "car_model";
    marker_car.id = 0;
    marker_car.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_car.action = visualization_msgs::Marker::ADD;
    // set marker_car.pose.position marker.pose.orientation
    marker_car.color.r = 1;
    // marker_car.color.g = 1;
    // marker_car.color.b = 1;
    marker_car.color.a = 1;
    marker_car.scale.x = 1.0;
    marker_car.scale.y = 1.0;
    marker_car.scale.z = 1.0;
    marker_car.mesh_use_embedded_materials = true;
    marker_car.mesh_resource = "package://car_model/ferrari/dae.DAE";
    return marker_car;
}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_v1");
    VISUALIZATION_V1::monitor app;
    app.run();
    return 0;
}