/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:26:01
 * @LastEditTime: 2019-03-01 14:45:39
 */

#include "lidar_euclidean_cluster.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_euclidean_cluster_node");
    LidarDetector::LidarClusterDetector app;
    ros::spin();
    return 0;
}
