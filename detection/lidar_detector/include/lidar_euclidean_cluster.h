/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:26:40
 * @LastEditTime: 2019-03-01 22:25:58
 */
#ifndef LIDAR_EUCLIDEAN_CLUSTER_H
#define LIDAR_EUCLIDEAN_CLUSTER_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace LidarDetector {

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

    std::chrono::system_clock::time_point start_time, end_time;
    std_msgs::Header msg_header;

    bool processing_now;
    double nearDistance;
    double farDistance;
    double leaf_size;
    double height_threshhold;
    double floor_max_height;
    double floor_max_angle;

    void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& only_floor_cloud,
        const double& max_height, const double& floor_max_angle);

    void mergeClusters();

    void removeNearAndFarPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& near_dis, const double& fat_dis);

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& leaf_size);

    void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
        const double& height);

    void cluster();

    void clusterGpu();

    void getPointCloud_cb(
        const sensor_msgs::PointCloud2ConstPtr& msg_rawPointCloud);

    void initROS();

    void pubPointCloud(
        const ros::Publisher& publisher,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pointcloud);
};

} // namespace LidarDetector

#endif // LIDAR_EUCLIDEAN_CLUSTER_H
