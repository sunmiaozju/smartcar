#ifndef __DDL_UTILS__
#define __DDL_UTILS__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>

#define PI 3.1415926
#define RAD2ANGLE(x) ((x)*57.295780)
#define ANGLE2RAD(x) ((x)*0.0174533)

namespace util
{

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
} // namespace util

#endif