#include <ros/ros.h>
#include <iostream>
#include "odom_imu/odom_imu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_imu_node");
  ROS_INFO("Start odom_imu_node.\nMake sure the car is in stationary on the ground before starting this node, and the imu is calibrated.");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OdomImu odom_imu(nh, pnh);
  if (!odom_imu.init()){
    ROS_ERROR("Cannot init OdomImu!");
    exit(-1);
  }

  ros::spin();
  return 0;
}