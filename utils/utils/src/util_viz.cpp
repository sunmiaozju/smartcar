#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <sstream>
#include <vector>

#include <utils.hpp>

ros::Publisher pub_viz_vel_;
ros::Publisher pub_viz_vel_info_;
ros::Publisher pub_viz_loc_conf_;
ros::Publisher pub_viz_loc_info_;
ros::Publisher pub_viz_car_;

std::string param_frame_map_;
double param_car_x_;
double param_car_y_;
double param_car_z_;
double param_bl_x_; // bottom left point in x axis
double param_bl_y_; // bottom left point in y axis
double param_max_vel_;
double param_max_ang_;
double param_len_line_;

geometry_msgs::Vector3 g_scale_car_;

void odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::vector<geometry_msgs::Point> points(16);
  // velocity line list
  points[0].x = param_bl_x_;
  points[0].y = param_bl_y_;
  points[1].x = points[0].x;
  points[1].y = points[0].y - 1.0 * param_len_line_;
  points[2].x = points[0].x + 3.0 * param_len_line_;
  points[2].y = points[0].y;
  points[3].x = points[2].x;
  points[3].y = points[2].y - param_len_line_;
  points[4].x = points[0].x;
  points[4].y = points[0].y - 0.5 * param_len_line_;
  points[5].x = points[4].x + 3.0 * param_len_line_;
  points[5].y = points[4].y;
  points[6].x = points[0].x + 1.0 * param_len_line_ + msg->twist.twist.linear.x / param_max_vel_ * 2.0 * param_len_line_;
  points[6].y = points[0].y - 0.1 * param_len_line_;
  points[7].x = points[6].x;
  points[7].y = points[6].y - 0.8 * param_len_line_;
  // steer line list
  points[8].x = param_bl_x_;
  points[8].y = param_bl_y_ - 2.0 * param_len_line_;
  points[9].x = points[8].x + 1.0 * param_len_line_;
  points[9].y = points[8].y;
  points[10].x = points[8].x;
  points[10].y = points[8].y - 2.0 * param_len_line_;
  points[11].x = points[10].x + 1.0 * param_len_line_;
  points[11].y = points[10].y;
  points[12].x = points[8].x + 0.5 * param_len_line_;
  points[12].y = points[8].y;
  points[13].x = points[12].x;
  points[13].y = points[12].y - 2.0 * param_len_line_;
  points[14].x = points[8].x + 0.1 * param_len_line_;
  points[14].y = points[8].y - 1.0 * param_len_line_ - msg->twist.twist.angular.z / param_max_ang_ * 1.0 * param_len_line_;
  points[15].x = points[14].x + 0.8 * param_len_line_;
  points[15].y = points[14].y;
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  util::pubMarkerLineList(pub_viz_vel_, pose, msg->header.stamp, param_frame_map_, points);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(4) << "vel_x: " << msg->twist.twist.linear.x << "m/s; ang_z: " << msg->twist.twist.angular.z << "rad/s";
  pose.position.x = points[0].x - 1.0;
  pose.position.y = points[0].y;
  pose.position.z = 0;
  util::pubMarkerText(pub_viz_vel_info_, pose, msg->header.stamp, param_frame_map_, ss.str());
}

void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  util::pubMarkerCube(pub_viz_car_, msg->pose, msg->header.stamp, msg->header.frame_id, g_scale_car_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "util_viz");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("frame_map", param_frame_map_, "map");
  pnh.param<double>("car_x", param_car_x_, 0.51);
  pnh.param<double>("car_y", param_car_y_, 0.43);
  pnh.param<double>("car_z", param_car_z_, 0.48);
  pnh.param<double>("bl_x", param_bl_x_, 10.);
  pnh.param<double>("bl_y", param_bl_y_, -20.);
  pnh.param<double>("max_vel", param_max_vel_, 4.0);
  pnh.param<double>("max_ang", param_max_ang_, 3.14);
  pnh.param<double>("len_line", param_len_line_, 2.0);

  ros::Subscriber sub_vel, sub_car;
  sub_vel = nh.subscribe<nav_msgs::Odometry>("/odom/imu", 1, &odomCB);
  sub_car = nh.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 1, &poseCB);
  pub_viz_vel_ = nh.advertise<visualization_msgs::Marker>("/viz/cmd_vel", 1);
  pub_viz_vel_info_ = nh.advertise<visualization_msgs::Marker>("/viz/cmd_vel/info", 1);
  pub_viz_loc_conf_ = nh.advertise<visualization_msgs::Marker>("/viz/loc_conf", 1);
  pub_viz_loc_info_ = nh.advertise<visualization_msgs::Marker>("/viz/loc_info", 1);
  pub_viz_car_ = nh.advertise<visualization_msgs::Marker>("/viz/car", 1);

  g_scale_car_.x = param_car_x_;
  g_scale_car_.y = param_car_y_;
  g_scale_car_.z = param_car_z_;

  ros::spin();
  return 0;
}