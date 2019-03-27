#include <Eigen/Core>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <path_msgs/Lane.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/Waypoint.h>
#include <sstream>
#include <utils.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace LOAD_STATIC_PATH {
void load_path(std::string path_file, smartcar_msgs::Lane& path)
{
    if (path_file == "None") {
        ROS_WARN_STREAM("Couldn't find path file! ");
        std::cout << "file: " << path_file << " doesn't exist.";
        return;
    }
    std::FILE* fp = std::fopen(path_file.c_str(), "r");
    while (!std::feof(fp)) {
        smartcar_msgs::Waypoint way_point;
        way_point.is_lane = 1;
        way_point.speed_limit = 3.0;
        std::fscanf(fp, "%lf,%lf,%lf", &way_point.pose.pose.position.x, &way_point.pose.pose.position.y, &way_point.pose.pose.position.z);
        path.waypoints.push_back(way_point);
    }
    std::cout << "success load file: " << path_file << std::endl;
}

void filter_path(smartcar_msgs::Lane& path, double dot)
{
    std::vector<smartcar_msgs::Waypoint> temp_vec;
    double sum = 0.0;
    int length = path.waypoints.size();
    for (int i = 0; i < length - 2; i++) {
        if (sum >= dot) {
            temp_vec.push_back(path.waypoints[i]);
            sum = 0.0;
        }
        sum += util::distance2points(path.waypoints[i].pose.pose.position, path.waypoints[i + 1].pose.pose.position);
    }
    // path.waypoints.clear();
    path.waypoints.assign(temp_vec.begin(), temp_vec.end());
}

void adjust_orientation(smartcar_msgs::Lane& path)
{
    for (size_t i = 0; i < path.waypoints.size() - 2; i++) {
        smartcar_msgs::Waypoint p_c = path.waypoints[i];
        smartcar_msgs::Waypoint p_n = path.waypoints[i + 1];
        double yaw = std::atan2(p_n.pose.pose.position.y - p_c.pose.pose.position.y, p_n.pose.pose.position.x - p_c.pose.pose.position.x);
        p_c.yaw = yaw;
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
        path.waypoints[i].pose.pose.orientation.x = q.x();
        path.waypoints[i].pose.pose.orientation.y = q.y();
        path.waypoints[i].pose.pose.orientation.z = q.z();
        path.waypoints[i].pose.pose.orientation.w = q.w();
    }
}

void debug_show_result(smartcar_msgs::Lane& path, ros::Publisher pub)
{
    visualization_msgs::MarkerArray static_path;
    for (int i = 0; i < path.waypoints.size(); i++) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "map";
        // arrow.header.stamp = ros::Time::now();
        arrow.id = i + 1;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.scale.x = 0.4;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.2;
        arrow.color.r = 0;
        arrow.color.g = 0;
        arrow.color.b = 1;
        arrow.color.a = 1;
        arrow.pose = path.waypoints[i].pose.pose;
        static_path.markers.push_back(arrow);
    }
    ros::Duration(3.0).sleep();
    pub.publish(static_path);
}
}

using namespace LOAD_STATIC_PATH;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_path_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher pub_path = nh.advertise<smartcar_msgs::Lane>("global_path", 1);
    ros::Publisher pub_debug_result = nh.advertise<visualization_msgs::MarkerArray>("/global_path/static_path", 1);

    smartcar_msgs::Lane global_path;
    global_path.header.frame_id = "map";

    std::string path_file;
    double dot;
    pnh.param<std::string>("path_file", path_file, "None");
    pnh.param<double>("dot", dot, 0.3);
    load_path(path_file, global_path);
    filter_path(global_path, dot);
    // smooth(global_path, double )
    adjust_orientation(global_path);
    debug_show_result(global_path, pub_debug_result);

    pub_path.publish(global_path);
    ros::spin();
    return 0;
}