/**
 * @brief 
 * 
 * @file tf_test.cpp
 * @author your name
 * @date 2018-09-20
 */
// #define TF_EULER_DEFAULT_ZYX
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/duration.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_test");
  tf::TransformBroadcaster broadcaster;
  ros::Duration d(1);

  while (ros::ok())
  {
    tf::Quaternion q(0.785, 0, 0);
    double yaw, pitch, roll;
    q.setRPY(0, 0, 0.785);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

    ROS_INFO("yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(0, 1, 0)), ros::Time::now(), "/map", "/odom"));
    tf::Transform cur_pose;
    q.setRPY(0., 0., 0.);
    cur_pose.setRotation(q);
    cur_pose.setOrigin(tf::Vector3(1., 1., 1.));
    geometry_msgs::Point p = geometry_msgs::Point();
    p.x = 3.;
    p.y = 3.;
    p.z = 3.;
    tf::Point pose;
    // pose.setRotation(q);
    // pose.setOrigin(tf::Vector3(3., 3., 3.));
    tf::pointMsgToTF(p, pose);

    // broadcaster.sendTransform(tf::StampedTransform(cur_pose, ros::Time::now(), "/map", "/cur"));
    // broadcaster.sendTransform(tf::StampedTransform(pose, ros::Time::now(), "/map", "/pose"));
    // broadcaster.sendTransform(tf::StampedTransform(cur_pose * pose, ros::Time::now(), "/cur", "/pose"));
    tf::Point r = cur_pose.inverse() * pose;
    ROS_INFO("x: %f, y: %f, z: %f", r.getX(), r.getY(), r.getZ());
    d.sleep();
  }

  ros::spin();
  return 0;
}