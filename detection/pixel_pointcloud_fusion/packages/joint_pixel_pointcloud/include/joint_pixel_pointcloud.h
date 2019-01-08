#ifndef NODE_JOINT_PIXEL_POINTCLOUD_H
#define NODE_JOINT_PIXEL_POINTCLOUD_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

namespace NODE_JOINT_PIXEL_POINTCLOUD
{
class PixelCloudFusion
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher pub_fusion_cloud;
  ros::Subscriber sub_intrinsics;
  ros::Subscriber sub_image;
  ros::Subscriber sub_cloud;

  cv::Size image_size;
  cv::Mat camera_instrinsics; //相机内参
  cv::Mat current_frame;
  cv::Mat distortion_coefficients; //畸变系数

  tf::StampedTransform camera_lidar_tf;
  tf::TransformListener* transform_listener;

  std::string image_frame_id;

  bool camera_info_ok_;
  bool processing_;
  bool camera_lidar_tf_ok_;

  float fx, fy, cx, cy;

  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

  void ImageCallback(const sensor_msgs::Image::ConstPtr &image_msg);

  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  void IntrinsicsCallback(const sensor_msgs::CameraInfo &intrinsics_msg);

  void initROS();

  tf::StampedTransform FindTransform(const std::string &target_frame, const std::string source_frame);

  pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

public:
  void run();

  PixelCloudFusion();
};
} // namespace NODE_JOINT_PIXEL_POINTCLOUD

#endif //NODE_JOINT_PIXEL_POINTCLOUD
