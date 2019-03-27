/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-02-21 21:41:21
 * @LastEditTime: 2019-03-15 15:04:04
 */
#ifndef NODE_JOINT_PIXEL_POINTCLOUD_H
#define NODE_JOINT_PIXEL_POINTCLOUD_H

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <smartcar_msgs/DetectedObject.h>
#include <smartcar_msgs/DetectedObjectArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yunle_sensor_msgs/DetectObject.h>
#include <yunle_sensor_msgs/DetectObjs.h>

#include <pcl/common/common.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

namespace NODE_JOINT_PIXEL_POINTCLOUD {

class PointXYZRT {
public:
    pcl::PointXYZ point;

    float radius;
    float theta;

    size_t radial_div;
    size_t concentric_div;

    size_t original_index;
};

typedef std::vector<PointXYZRT> PointCloudXYZRT;

class Object {
public:
    double score;
    int category;
    int xmin, ymin, xmax, ymax;
    int xmin_3d_pic, ymin_3d_pic, xmax_3d_pic, ymax_3d_pic;
    double center_x, center_y, center_z;
    double obj_deg;
    double obj_dist;
    double xmin_3d_bbox, ymin_3d_bbox, zmin_3d_bbox, xmax_3d_bbox, ymax_3d_bbox, zmax_3d_bbox;
    pcl::PointCloud<pcl::PointXYZ> pc;
};

class PixelCloudFusion {
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Publisher pub_fusion_cloud;
    ros::Subscriber sub_intrinsics;
    ros::Subscriber sub_image;
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_detection;
    ros::Publisher test_pointcloud;
    ros::Publisher objs_pub_rviz;
    ros::Publisher objs_pub;

    std::vector<Object> objs;
    std::vector<Object> removed_lessPoints_objs;

    cv::Size image_size;
    cv::Mat camera_instrinsics; //相机内参
    cv::Mat current_frame;
    cv::Mat distortion_coefficients; //畸变系数

    tf::StampedTransform camera_lidar_tf;
    tf::TransformListener transform_listener;

    bool camera_lidar_tf_ok_;
    bool camera_info_ok_;
    bool usingObjs;

    double radial_divide_angle;
    double concentric_divide_distance;
    double min_local_height_threshold;

    double clip_left_right_dis;
    double clip_far;
    double clip_dis;
    double clip_height;

    double local_slope_threshold;
    double general_slope_threshhold;
    double sensor_height;

    std::string image_frame_id;

    float fx, fy, cx, cy;

    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;

    void ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    void IntrinsicsCallback(const sensor_msgs::CameraInfo& intrinsics_msg);

    void DetectionCallback(const yunle_sensor_msgs::DetectObjs& objs_msg);

    void initROS();

    void publishObjs();

    tf::StampedTransform FindTransform(const std::string& target_frame, const std::string source_frame);

    pcl::PointXYZ TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform);

    void category_deal(visualization_msgs::Marker& objmarker, Object& obj);

    void removeOutlier(Object& in_detect_obj, const double& max_cluster_dis);

    void calObstacleInfo(Object& in_detect_obj);

    void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& height, const double& near_dis, const double& fat_dis, const double& left_right_dis);

    void removeFloorRayFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_only_ground_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_no_ground_cloud,
        const double& sensor_height, const double& local_max_slope, const double& general_max_slope);

    void convertXYZ2XYZRT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        std::vector<PointCloudXYZRT>& out_radial_divided_cloud);

public:
    PixelCloudFusion();
};
} // namespace NODE_JOINT_PIXEL_POINTCLOUD

#endif //NODE_JOINT_PIXEL_POINTCLOUD
