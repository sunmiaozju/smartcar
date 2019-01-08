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
        // 从tf树里面寻找变换关系
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
    // 存储处理后的点云
    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf);

        int u = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        int v = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);

        if ((u >= 0) && (u < image_size.width) &&
            (v >= 0) && (v < image_size.height) &&
            cam_cloud[i].z > 0)
        {
            projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u, v), in_cloud->points[i]));
        }
    }

    out_cloud->points.clear();

    for (int row = 0; row < image_size.height; row++)
    {
        for (int col = 0; col < image_size.width; col++)
        {
            std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
            pcl::PointXYZ corresponding_3d_point;
            pcl::PointXYZRGB colored_3d_point;
            iterator_3d_2d = projection_map.find(cv::Point(col, row));

            if (iterator_3d_2d != projection_map.end())
            {
                corresponding_3d_point = iterator_3d_2d->second;
                cv::Vec3b rgb_pixel = current_frame.at<cv::Vec3b>(row, col);
                colored_3d_point.x = corresponding_3d_point.x;
                colored_3d_point.y = corresponding_3d_point.y;
                colored_3d_point.z = corresponding_3d_point.z;
                colored_3d_point.r = rgb_pixel[2];
                colored_3d_point.g = rgb_pixel[1];
                colored_3d_point.b = rgb_pixel[0];
                out_cloud->points.push_back(colored_3d_point);
            }
        }
    }
    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*out_cloud, out_cloud_msg);
    out_cloud_msg.header = cloud_msg->header;
    pub_fusion_cloud.publish(cloud_msg);
}

tf::StampedTransform PixelCloudFusion::FindTransform(const std::string &target_frame, const std::string source_frame)
{
    tf::StampedTransform transform;

    camera_lidar_tf_ok_ = false;

    try
    {
        // ros::Time(0)指定了时间为0，即获得最新有效的变换。
        // 改变获取当前时间的变换，即改为ros::Time::now(),不过now的话因为监听器有缓存区的原因。一般会出错
        // 参考：https://www.ncnynl.com/archives/201702/1313.html
        transform_listener->lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        camera_lidar_tf_ok_ = true;
        ROS_INFO("joint_pixel_pointcloud : camera-lidar-tf obtained");
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("joint_pixel_pointcloud : %s", ex.what());
    }

    return;
}

pcl::PointXYZ PixelCloudFusion::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_transformed = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_transformed.x(), tf_point_transformed.y(), tf_point_transformed.z());
}

void PixelCloudFusion::initROS()
{
}

void PixelCloudFusion::run()
{
}

PixelCloudFusion::PixelCloudFusion() : nh_private("~")
{
    initROS();
}
