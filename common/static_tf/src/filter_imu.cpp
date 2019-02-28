#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub_imu_filtered;
ros::Subscriber sub_imu;

void imuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
  double roll, pitch, yaw;
  tf::Quaternion orient;
  tf::quaternionMsgToTF(msg->orientation, orient);
  tf::Matrix3x3(orient).getRPY(roll, pitch, pitch);
  sensor_msgs::Imu msg_imu_filtered;
  msg_imu_filtered = *msg;
  orient.setRPY(0., 0., yaw);
  tf::quaternionTFToMsg(orient, msg_imu_filtered.orientation);
  pub_imu_filtered.publish(msg_imu_filtered);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_imu");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 45, imuCB);
  pub_imu_filtered = nh.advertise<sensor_msgs::Imu>("/imu/data/filtered", 45);

  ros::spin();
  return 0;
}