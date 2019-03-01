/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:25:55
 * @LastEditTime: 2019-03-01 22:33:32
 */

#include "lidar_euclidean_cluster.h"

namespace LidarDetector {
LidarClusterDetector::LidarClusterDetector()
    : private_nh("~")
    , processing_now(false)
{
    initROS();
}

LidarClusterDetector::~LidarClusterDetector() {}

/**
 * @description: ROS参数初始化 
 */
void LidarClusterDetector::initROS()
{
    sub_rawPointCloud = nh.subscribe(
        "velodyne_points", 10, &LidarClusterDetector::getPointCloud_cb, this);
    pub_testPointCloud = nh.advertise<sensor_msgs::PointCloud2>("test_pointcloud", 10);
    pub2_testPointCloud = nh.advertise<sensor_msgs::PointCloud2>("test2_pointcloud", 10);

    private_nh.param<double>("nearDistance", nearDistance, 2.0);
    private_nh.param<double>("farDistance", farDistance, 30);
    private_nh.param<double>("downsampleLeafSize", leaf_size, 0.5);
    private_nh.param<double>("height_threshhold", height_threshhold, 3.0);
    private_nh.param<double>("floor_max_height", floor_max_height, 0.3);
    private_nh.param<double>("floor_max_angle", floor_max_angle, 0.2);
}

/**
 * @description: 原始点云回调函数
 */
void LidarClusterDetector::getPointCloud_cb(
    const sensor_msgs::PointCloud2ConstPtr& msg_rawPointCloud)
{
    if (!processing_now) {
        processing_now = true;
        start_time = std::chrono::system_clock::now();

        msg_header = msg_rawPointCloud->header;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_sensor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_near_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr only_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg_rawPointCloud, *raw_sensor_cloud_ptr);

        clipCloud(raw_sensor_cloud_ptr, clipped_cloud_ptr, height_threshhold);

        removeNearAndFarPoints(clipped_cloud_ptr, removed_near_cloud_ptr, nearDistance, farDistance);

        downsampleCloud(removed_near_cloud_ptr, downsample_cloud_ptr, leaf_size);

        removeFloor(downsample_cloud_ptr, removed_floor_cloud_ptr, only_floor_cloud_ptr, floor_max_height, floor_max_angle);

        pubPointCloud(pub_testPointCloud, removed_floor_cloud_ptr);
        pubPointCloud(pub2_testPointCloud, only_floor_cloud_ptr);

        processing_now = false;
    }
}

/**
 * @description: 去除地面 
 */
void LidarClusterDetector::removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& only_floor_cloud,
    const double& max_height, const double& floor_max_angle)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr indexs(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(floor_max_angle);
    seg.setDistanceThreshold(max_height);
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud);
    seg.segment(*indexs, *coefficients);

    if (indexs->indices.size() == 0) {
        printf("%s\n", "[lidar_euclidean_cluster_node]: could't seg floor");
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(indexs);
    extract.setNegative(true); // true removes the indices, false leaves only the indices
    extract.filter(*out_cloud);

    extract.setNegative(false); // true removes the indices, false leaves only the indices
    extract.filter(*only_floor_cloud);
}

/**
 * @description: 截取点云，去除高度过高的点 
 */
void LidarClusterDetector::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    const double& height)
{
    out_cloud->points.clear();
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        if (in_cloud->points[i].z < height) {
            out_cloud->points.push_back(in_cloud->points[i]);
        }
    }
}

/**
 * @description: 点云下采样
 */
void LidarClusterDetector::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    const double& leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(in_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*out_cloud);
}

/**
 * @description: 去除距离激光雷达中心过近的点
 */
void LidarClusterDetector::removeNearAndFarPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud, const double& near_dis, const double& far_dis)
{
    out_cloud->points.clear();
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        double dis = sqrt(pow(in_cloud->points[i].x, 2) + pow(in_cloud->points[i].y, 2));
        if (dis > near_dis && dis < far_dis) {
            out_cloud->points.push_back(in_cloud->points[i]);
        }
    }
}

/**
 * @description: 点云发布函数 
 */
void LidarClusterDetector::pubPointCloud(
    const ros::Publisher& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pointcloud)
{
    sensor_msgs::PointCloud2 msg_pointcloud;
    pcl::toROSMsg(*in_pointcloud, msg_pointcloud);
    msg_pointcloud.header = msg_header;
    publisher.publish(msg_pointcloud);
}
} // namespace LidarDetector