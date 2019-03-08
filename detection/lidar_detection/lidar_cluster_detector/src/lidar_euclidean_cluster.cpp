/*
 * @Description:
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-01 11:25:55
 * @LastEditTime: 2019-03-08 14:57:10
 */

#include "lidar_euclidean_cluster.h"

namespace LidarDetector {
LidarClusterDetector::LidarClusterDetector()
    : private_nh("~")
    , processing_now(false)
{
    initROS();

    splitString(str_range, dis_range);
    splitString(str_seg_distances, seg_distances);

    generateColors(color_table, 255, 100);
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
    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("test2_pointcloud", 10);

    private_nh.param<double>("nearDistance", nearDistance, 2.0);
    private_nh.param<double>("farDistance", farDistance, 30);
    private_nh.param<double>("downsampleLeafSize", leaf_size, 0.5);
    private_nh.param<double>("height_threshhold", height_threshhold, 4.0);
    private_nh.param<double>("floor_max_height", floor_max_height, 0.3);
    private_nh.param<double>("floor_max_angle", floor_max_angle, 0.2);
    private_nh.param<double>("small_scale", small_scale, 0.5);
    private_nh.param<double>("large_scale", large_scale, 2.0);
    private_nh.param<double>("angle_threshold", angle_threshold, 0.5);
    private_nh.param<double>("radial_divide_angle", radial_divide_angle, 0.5);
    private_nh.param<double>("concentric_divide_distance", concentric_divide_distance, 0.1);
    private_nh.param<double>("min_local_height_threshold", min_local_height_threshold, 0.05);
    private_nh.param<double>("sensor_height", sensor_height, 0.36);
    private_nh.param<double>("local_threshold_slope", local_threshold_slope, 3.0);
    private_nh.param<double>("general_threshold_slope", general_threshold_slope, 2.0);
    private_nh.param<double>("left_right_dis_threshold", left_right_dis_threshold, 6.5);

    private_nh.param<std::string>("str_range", str_range, "15,30,45,60");
    private_nh.param<std::string>("str_seg_distances", str_seg_distances, "0.5,1.1,1.6,2.1,2.6");

    private_nh.param<double>("cluster_min_points", cluster_min_points, 10);
    private_nh.param<double>("cluster_max_points", cluster_max_points, 100000);
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr only_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr don_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ray_no_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ray_only_floor_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg_rawPointCloud, *raw_sensor_cloud_ptr);

        clipCloud(raw_sensor_cloud_ptr, clipped_cloud_ptr, height_threshhold, nearDistance, farDistance, left_right_dis_threshold);

        downsampleCloud(clipped_cloud_ptr, downsample_cloud_ptr, leaf_size);

        removeFloorRayFiltered(downsample_cloud_ptr, ray_only_floor_cloud_ptr, ray_no_floor_cloud_ptr, sensor_height,
            local_threshold_slope, general_threshold_slope);

        pubPointCloud(pub_testPointCloud, ray_only_floor_cloud_ptr);
        pubPointCloud(pub2_testPointCloud, ray_no_floor_cloud_ptr);

        processing_now = false;
    }
}

/**
 * @description: 基于距离的点云聚类 
 */
void LidarClusterDetector::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr)
{
    // 根据距离不同，设置不同的聚类阈值
    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_segments_array[i] = tmp;
    }

    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++) {
        pcl::PointXYZ p;
        p.x = in_cloud_ptr->points[i].x;
        p.y = in_cloud_ptr->points[i].y;
        p.z = in_cloud_ptr->points[i].z;

        float origin_dis = sqrt(pow(p.x, 2) + pow(p.y, 2));

        if (origin_dis < dis_range[0]) {
            cloud_segments_array[0]->points.push_back(p);
        } else if (origin_dis < dis_range[1]) {
            cloud_segments_array[1]->points.push_back(p);
        } else if (origin_dis < dis_range[2]) {
            cloud_segments_array[2]->points.push_back(p);
        } else if (origin_dis < dis_range[3]) {
            cloud_segments_array[3]->points.push_back(p);
        } else {
            cloud_segments_array[4]->points.push_back(p);
        }
    }

    std::vector<ClusterPtr> clusters;
    for (size_t i = 0; i < cloud_segments_array.size(); i++) {
        clusterCpu(cloud_segments_array[i], clusters, seg_distances[i]);
    }
}

/**
 * @description: 对聚类结果进行后处理，生成聚类的相关信息保存到cluster类中
 */
void LidarClusterDetector::clusterCpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    std::vector<ClusterPtr>& clusters, const double& max_cluster_dis)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*in_cloud, *cloud_2d);

    for (size_t i = 0; i < cloud_2d->points.size(); i++) {
        cloud_2d->points[i].z = 0;
    }

    tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc;
    // setClusterTolerance(). If you take a very small value, it can happen that
    // an actual object can be seen as multiple clusters. On the other hand, if
    // you set the value too high, it could happen, that multiple objects are
    // seen as one cluster. So our recommendation is to just test and try out
    // which value suits your dataset.
    euc.setClusterTolerance(max_cluster_dis);
    euc.setMinClusterSize(cluster_min_points);
    euc.setMaxClusterSize(cluster_max_points);
    euc.setSearchMethod(tree);
    euc.setInputCloud(cloud_2d);
    euc.extract(cluster_indices);

    for (size_t j = 0; j < cluster_indices.size(); j++) {
        ClusterPtr one_cluster;
        one_cluster->setCloud(in_cloud, color_table, cluster_indices[j].indices, j);
        clusters.push_back(one_cluster);
    }
}

/**
 * @description: 基于ray_groud_filtered对地面进行分割 
 */
void LidarClusterDetector::removeFloorRayFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_only_ground_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_no_ground_cloud,
    const double& sensor_height, const double& local_max_slope, const double& general_max_slope)
{
    pcl::PointIndices only_ground_indices;
    out_only_ground_cloud->points.clear();
    out_no_ground_cloud->points.clear();

    std::vector<PointCloudXYZRT> radial_divided_cloud;

    convertXYZ2XYZRT(in_cloud, radial_divided_cloud);

#pragma omp for
    for (size_t i = 0; i < radial_divided_cloud.size(); i++) {
        float prev_radius = 0.0;
        float prev_height = -sensor_height;
        bool prev_ground = false;
        bool current_ground = false;

        for (size_t j = 0; j < radial_divided_cloud[i].size(); j++) {
            float local_twoPoints_dis = radial_divided_cloud[i][j].radius - prev_radius;
            float local_height_threshold = tan(local_max_slope * M_PI / 180.) * local_twoPoints_dis;
            float general_height_threshold = tan(general_max_slope * M_PI / 180.) * radial_divided_cloud[i][j].radius;
            float current_height = radial_divided_cloud[i][j].point.z;

            if (radial_divided_cloud[i][j].radius > concentric_divide_distance && local_height_threshold < min_local_height_threshold) {
                local_height_threshold = min_local_height_threshold;
            }

            if (current_height <= (prev_height + local_height_threshold) && current_height >= (prev_height - local_height_threshold)) {
                if (!prev_ground) {
                    if (current_height <= (-sensor_height + general_height_threshold) && current_height >= (-sensor_height - general_height_threshold)) {
                        current_ground = true;
                    } else {
                        current_ground = false;
                    }
                } else {
                    current_ground = true;
                }
            } else {
                current_ground = false;
            }

            if (current_ground) {
                only_ground_indices.indices.push_back(radial_divided_cloud[i][j].original_index);
                prev_ground = true;
            } else {
                prev_ground = false;
            }
            prev_radius = radial_divided_cloud[i][j].radius;
            prev_height = radial_divided_cloud[i][j].point.z;
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(in_cloud);
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(only_ground_indices));

    extractor.setNegative(false); //true removes the indices, false leaves only the indices
    extractor.filter(*out_only_ground_cloud);

    extractor.setNegative(true); //true removes the indices, false leaves only the indices
    extractor.filter(*out_no_ground_cloud);
}

/**
 * @description: 将原始点云转化为 XYZRadialTheta 结构的点云 
 */
void LidarClusterDetector::convertXYZ2XYZRT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    std::vector<PointCloudXYZRT>& out_radial_divided_cloud)
{
    out_radial_divided_cloud.clear();
    double radial_divide_num = ceil(360
        / radial_divide_angle);
    out_radial_divided_cloud.resize(radial_divide_num);

    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        PointXYZRT p;
        float radius = (float)sqrt(in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        float thera = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;

        if (thera < 0)
            thera += 360;

        size_t radial_div = (size_t)floor(thera / radial_divide_angle);
        size_t concentric_div = (size_t)floor(radius / concentric_divide_distance);

        p.point = in_cloud->points[i];
        p.radius = radius;
        p.theta = thera;
        p.radial_div = radial_div;
        p.concentric_div = concentric_div;
        p.original_index = i;

        out_radial_divided_cloud[radial_div].push_back(p);
    }

#pragma omp for
    for (size_t j = 0; j < out_radial_divided_cloud.size(); j++) {
        std::sort(out_radial_divided_cloud[j].begin(), out_radial_divided_cloud[j].end(),
            [](const PointXYZRT& a, const PointXYZRT& b) { return a.radius < b.radius; });
    }
}

/**
 * @description: 非结构化点云分割：根据大尺度范围的法向量和小尺度范围内的法向量的差异，去除了差异变化不明显的点(近似平面的点)
 * 参考链接：http://pointclouds.org/documentation/tutorials/don_segmentation.php
 */
void LidarClusterDetector::differenceOfNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    // 对于深度图这种结构化的数据，使用OrganizedNeighbor作为查找树
    // 对于激光雷达产生的非结构化数据，使用KdTree作为查找树
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    // 为查找树添加点云数据
    tree->setInputCloud(in_cloud);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_eatimation;
    // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;

    normal_eatimation.setInputCloud(in_cloud);
    normal_eatimation.setSearchMethod(tree);

    // setting viewpoint is very important, so that we can ensure that normals
    // estimated at different scales share a consistent orientation.
    normal_eatimation.setViewPoint(std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

    pcl::PointCloud<pcl::PointNormal>::Ptr normal_small_scale(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_large_scale(new pcl::PointCloud<pcl::PointNormal>);
    // calculate normals with the small scale
    normal_eatimation.setRadiusSearch(small_scale);
    normal_eatimation.compute(*normal_small_scale);
    // calculate normals with the large scale
    normal_eatimation.setRadiusSearch(large_scale);
    normal_eatimation.compute(*normal_large_scale);

    // Create and initial the output cloud for DoN (Difference of Normals) results
    pcl::PointCloud<pcl::PointNormal>::Ptr diff_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud, *diff_normal_cloud);

    // Create DoN operator
    // The pcl::DifferenceOfNormalsEstimation class has 3 template parameters,
    // the first corresponds to the input point cloud type, in this case
    // pcl::PointXYZ, the second corresponds to the type of the normals
    // estimated for the point cloud, in this case pcl::PointNormal, and the
    // third corresponds to the vector field output type, in this case also
    // pcl::PointNormal.
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diff_normal_estimator;
    diff_normal_estimator.setInputCloud(in_cloud);
    diff_normal_estimator.setNormalScaleSmall(normal_small_scale);
    diff_normal_estimator.setNormalScaleLarge(normal_large_scale);
    diff_normal_estimator.initCompute();

    diff_normal_estimator.computeFeature(*diff_normal_cloud);

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>);
    // 设置条件：curvature必须满足大于(greater than) 角度阈值 angle_threshold
    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
        new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(diff_normal_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr diff_normal_filtered_cloud(new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    cond_removal.filter(*diff_normal_filtered_cloud);

    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diff_normal_filtered_cloud, *out_cloud);
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
    // 设置分割对象是垂直平面
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    // 设置随机采样方式
    seg.setMethodType(pcl::SAC_RANSAC);
    // 设置最大迭代次数
    seg.setMaxIterations(100);
    // 设置垂直的轴
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    // 设置垂直角度的最大阈值
    seg.setEpsAngle(floor_max_angle);
    // 设置查询点到目标模型的最大距离
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
 * @description: 截取点云，去除高度过高的点,去除距离激光雷达中心过近的点, 去除非车辆前面的点
 */
void LidarClusterDetector::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    const double& height, const double& near_dis, const double& far_dis,
    const double& left_right_dis)
{
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(in_cloud);
    pcl::PointIndices indices;

#pragma omp for
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        double dis;
        // 计算 需要移除的点
        if (in_cloud->points[i].z > height) {
            indices.indices.push_back(i);
        } else if (in_cloud->points[i].x < 0 || in_cloud->points[i].y > left_right_dis || in_cloud->points[i].y < -left_right_dis) { // 激光雷达 x正方向朝前，y正方向朝左，z正方向朝上
            indices.indices.push_back(i);
        } else if ((dis = sqrt(pow(in_cloud->points[i].x, 2) + pow(in_cloud->points[i].y, 2))) < near_dis || dis > far_dis) {
            indices.indices.push_back(i);
        }
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true); //true removes the indices, false leaves only the indices
    extractor.filter(*out_cloud);
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

void LidarClusterDetector::splitString(const std::string& in_string, std::vector<double>& out_array)
{
    std::string tmp;
    std::istringstream in(in_string);
    while (std::getline(in, tmp, ',')) {
        out_array.push_back(stod(tmp));
    }
}

void pubClusters(const std::vector<ClusterPtr>& in_clusters,
    const ros::Publisher& pub)
{
}

} // namespace LidarDetector