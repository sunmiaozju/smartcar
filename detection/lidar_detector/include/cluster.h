/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-05 20:42:26
 * @LastEditTime: 2019-03-05 22:39:59
 */
#ifndef CLUSTER_H
#define CLUSTER_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>

#include <pcl/common/common.h>

namespace LidarDetector {
class Cluster {
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc;
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    pcl::PointXYZ central_point;

    float length, width, height;

    std::string label;

public:
    Cluster();
    ~Cluster();
    void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
        const std::vector<int>& cluster_indices, const double& cluster_id);
};

typedef boost::shared_ptr<Cluster> ClusterPtr;

void generateColors(std::vector<cv::Scalar>& colors, size_t count, size_t factor);

static void downsamplePoints(const cv::Mat& src, cv::Mat& dst, size_t count);
}

#endif //CLUSTER_H
