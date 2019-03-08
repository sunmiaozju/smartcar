/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:26:40
 * @LastEditTime: 2019-03-08 14:59:51
 */
#ifndef LIDAR_EUCLIDEAN_CLUSTER_H
#define LIDAR_EUCLIDEAN_CLUSTER_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>

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

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cluster.h"

namespace LidarDetector {

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

class LidarClusterDetector {
public:
    LidarClusterDetector();

    ~LidarClusterDetector();

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber sub_rawPointCloud;
    ros::Publisher pub_testPointCloud;
    ros::Publisher pub2_testPointCloud;
    ros::Publisher pub_clusters;

    std::chrono::system_clock::time_point start_time, end_time;
    std_msgs::Header msg_header;

    bool processing_now;
    double nearDistance;
    double farDistance;
    double leaf_size;
    double height_threshhold;
    double floor_max_height;
    double floor_max_angle;
    double small_scale;
    double large_scale;
    double angle_threshold;
    double radial_divide_angle;
    double concentric_divide_distance;
    double min_local_height_threshold;
    double sensor_height;
    double local_threshold_slope;
    double general_threshold_slope;
    double left_right_dis_threshold;

    double cluster_min_points;
    double cluster_max_points;

    std::vector<double> dis_range;
    std::vector<double> seg_distances;
    std::string str_range;
    std::string str_seg_distances;

    std::vector<cv::Scalar> color_table;

    void differenceOfNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud);

    void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& only_floor_cloud,
        const double& max_height, const double& floor_max_angle);

    void removeFloorRayFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_only_ground_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_no_ground_cloud,
        const double& sensor_height, const double& local_max_slope, const double& general_max_slope);

    void convertXYZ2XYZRT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        std::vector<PointCloudXYZRT>& out_radial_divided_cloud);

    void mergeClusters();

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& leaf_size);

    void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& height, const double& near_dis, const double& fat_dis, const double& left_right_dis);

    void clusterCpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        std::vector<ClusterPtr>& cluster, const double& max_cluster_dis);

    void clusterGpu();

    void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr);

    void getPointCloud_cb(
        const sensor_msgs::PointCloud2ConstPtr& msg_rawPointCloud);

    void initROS();

    void pubPointCloud(
        const ros::Publisher& publisher,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pointcloud);

    void splitString(const std::string& in_string, std::vector<double>& out_array);

    void pubClusters(const std::vector<ClusterPtr>& in_clusters,
        const ros::Publisher& pub);
};

} // namespace LidarDetector

#endif // LIDAR_EUCLIDEAN_CLUSTER_H
