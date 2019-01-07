#include "joint_pixel_pointcloud.h"

using namespace NODE_JOINT_PIXEL_POINTCLOUD;

void PixelCloudFusion::ImageCallback(const sensor_msgs::Image::ConstPtr &image_msg)
{
    if (!camera_info_ok_)
    {
        ROS_INFO("joint_pixel_pointcloud waiting for intrinsics to be availiable");
    }

    if (processing_)
    {
        return;
    }

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat image = cv_image->image;

    // 图像去畸变
    cv::undistort(image, current_frame, camera_instrinsics, distortion_coefficients);

    image_frame_id = image_msg->header.frame_id;
    image_size.height = current_frame.rows;
    image_size.width = current_frame.cols;
}

void PixelCloudFusion::IntrinsicsCallback(const sensor_msgs::CameraInfo &intrinsisc_msg)
{
    image_size.height = intrinsisc_msg.height;
    image_size.width = intrinsisc_msg.width;

    camera_instrinsics = cv::Mat(3, 3, CV_64F);

    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            camera_instrinsics.at<double>(row, col) = intrinsisc_msg.K[row * 3 + col];
        }
    }

    fx = static_cast<float>(intrinsisc_msg.P[0]);
    fy = static_cast<float>(intrinsisc_msg.P[5]);
    cx = static_cast<float>(intrinsisc_msg.P[2]);
    cy = static_cast<float>(intrinsisc_msg.P[6]);

    sub_intrinsics.shutdown();
    camera_info_ok_ = true;
    ROS_INFO("joint_pixel_pointcloud : camera intrinsics get");
}

void PixelCloudFusion::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    if (current_frame.empty() || image_frame_id == "")
    {
        ROS_INFO("joint_pixel_pointcloud : waiting for image frame ");
        return;
    }

    if (!camera_lidar_tf_ok_)
    {
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);
    }

    if (!camera_info_ok_ || !camera_lidar_tf_ok_)
    {
        ROS_INFO("joint_pixel_pointcloud : waiting for camera intrinsics and camera lidar tf");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *in_cloud);

    std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;
    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf);
        
    }
}

tf::StampedTransform PixelCloudFusion::FindTransform(const std::string &target_frame, const std::string source_frame)
{
}

pcl::PointXYZ PixelCloudFusion::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
}
