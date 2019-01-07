#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_map_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher pub_map;
  sensor_msgs::PointCloud2 msg_map;
  ros::Duration duration;

  std::string param_pcd_file;
  std::string param_map_frame;
  double param_duration;

  pnh.param<std::string>("pcd_file", param_pcd_file, "");
  pnh.param<std::string>("map_frame", param_map_frame, "/map");
  pnh.param<double>("duraion", param_duration, 1.0);

  if (param_pcd_file == "" || pcl::io::loadPCDFile(param_pcd_file, msg_map) == -1)
  {
    ROS_ERROR("Failed to load pcd file: %s", param_pcd_file.c_str());
    return (-1);
  }
  msg_map.header.frame_id = param_map_frame;

  pub_map = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 1);
  duration.fromSec(param_duration);

  while (ros::ok())
  {
    msg_map.header.stamp = ros::Time::now();
    pub_map.publish(msg_map);

    duration.sleep();
  }

  return 0;
}