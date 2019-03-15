#ifndef __DDL_UTILS__
#define __DDL_UTILS__

#include <Eigen/Core>
#include <map>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#define PI 3.1415926
#define RAD2ANGLE(x) ((x)*57.295780)
#define ANGLE2RAD(x) ((x)*0.0174533)

namespace util {

bool pubMarkerText(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::string text)
{
    visualization_msgs::Marker msg;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.b = 0.0;
    color.r = 1.0;
    color.g = 1.0;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.ns = "~";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg.action = visualization_msgs::Marker::ADD;
    msg.text = text;
    msg.pose = pose;
    msg.color = color; // yellow
    msg.scale.x = 0.;
    msg.scale.y = 5.;
    msg.scale.z = 1.;
    msg.frame_locked = true;
    pub.publish(msg);
    return true;
}

bool pubMarkerCylinder(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale)
{
    visualization_msgs::Marker msg;
    std_msgs::ColorRGBA color;
    color.a = 0.4;
    color.b = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.ns = "~";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose = pose;
    msg.color = color;
    msg.scale = scale;
    msg.frame_locked = true;
    pub.publish(msg);
    return true;
}

bool pubMarkerCube(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale)
{
    visualization_msgs::Marker msg;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.b = 1.0;
    color.r = 1.0;
    color.g = 1.0;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.ns = "~";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::CUBE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose = pose;
    msg.color = color;
    msg.scale = scale;
    msg.frame_locked = true;
    pub.publish(msg);
}

bool pubMarkerLineList(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::vector<geometry_msgs::Point> points)
{
    visualization_msgs::Marker msg;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.b = 0.0;
    color.r = 1.0;
    color.g = 0.0;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.ns = "~";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose = pose;
    msg.color = color;
    msg.scale.x = 0.1;
    msg.scale.y = 0.1;
    msg.scale.z = 0.1;
    msg.frame_locked = true;
    msg.points.insert(msg.points.end(), points.begin(), points.end());
    pub.publish(msg);
}

double distance2points(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    return std::fabs(std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2)));
}

// TODO::验证正确性，增加RPY2Quaternion的变换
std::map<std::string, double> Quaternion2RPY(double x, double y, double z, double w)
{
    double roll, pitch, yaw;
    // roll = std::atan2(2.0 * (w * z + x * y), 1 - 2 * (z * z + x * x));
    // pitch = std::atan2(2.0 * (w * y + z * x), 1 - 2 * (y * y + x * x));
    // yaw = std::asinhf(2.0 * (w * x - y * z));
    // return Eigen::Vector3d(roll, pitch, yaw);

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);
    std::map<std::string, double> res;
    res["roll"] = roll;
    res["pitch"] = pitch;
    res["yaw"] = yaw;
    return res;
}

} // namespace util

#endif