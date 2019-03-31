/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-02-21 21:34:40
 * @LastEditTime: 2019-03-25 15:13:13
 */
#include "joint_pixel_pointcloud.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace NODE_JOINT_PIXEL_POINTCLOUD;

void PixelCloudFusion::ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    if (!camera_info_ok_) {
        ROS_INFO("joint_pixel_pointcloud : waiting for intrinsics to be availiable");
        return;
    }

    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat image = cv_image->image;

    // 图像去畸变
    // 使用相机内参和畸变系数可以图像去畸变
    cv::undistort(image, current_frame, camera_instrinsics, distortion_coefficients);

    static image_transport::ImageTransport it(nh);
    static image_transport::Publisher pub_image = it.advertise("identified_image", 1);
    static sensor_msgs::ImagePtr msg; 
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_frame).toImageMsg();
    pub_image.publish(msg);

    image_frame_id = image_msg->header.frame_id;
    image_size.height = current_frame.rows;
    image_size.width = current_frame.cols;
}

void PixelCloudFusion::IntrinsicsCallback(const sensor_msgs::CameraInfo& intrinsisc_msg)
{
    image_size.height = intrinsisc_msg.height;
    image_size.width = intrinsisc_msg.width;

    // 相机内参
    camera_instrinsics = cv::Mat(3, 3, CV_64F);

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            camera_instrinsics.at<double>(row, col) = intrinsisc_msg.K[row * 3 + col];
        }
    }

    // 相机畸变参数
    distortion_coefficients = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++) {
        distortion_coefficients.at<double>(col) = intrinsisc_msg.D[col];
    }
    // 投影系数
    fx = static_cast<float>(intrinsisc_msg.P[0]);
    fy = static_cast<float>(intrinsisc_msg.P[5]);
    cx = static_cast<float>(intrinsisc_msg.P[2]);
    cy = static_cast<float>(intrinsisc_msg.P[6]);

    sub_intrinsics.shutdown();
    camera_info_ok_ = true;
    ROS_INFO("joint_pixel_pointcloud : camera intrinsics get");
}

void PixelCloudFusion::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (current_frame.empty() || image_frame_id == "") {
        ROS_INFO("joint_pixel_pointcloud : waiting for image frame ");
        return;
    }

    if (!camera_lidar_tf_ok_) {
        // 从tf树里面寻找变换关系
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);
    }

    if (!camera_info_ok_ || !camera_lidar_tf_ok_) {
        ROS_INFO("joint_pixel_pointcloud : waiting for camera intrinsics and camera lidar tf");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *in_cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_clipped(new pcl::PointCloud<pcl::PointXYZ>);
    clipCloud(in_cloud_msg, in_cloud_clipped, clip_height, clip_dis, clip_far, clip_left_right_dis);

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_floor(new pcl::PointCloud<pcl::PointXYZ>);
    removeFloorRayFiltered(in_cloud_clipped, only_floor, in_cloud, sensor_height, local_slope_threshold, general_slope_threshhold);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    out_cloud->points.clear();
    pcl::PointXYZRGB colored_3d_point;
    for (size_t k = 0; k < objs.size(); k++) {
        objs[k].pc.points.clear();
    }
    usingObjs = true;
    // 存储处理后的点云
    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf);
        // transformed_cloud->points.push_back(cam_cloud[i]);
        // 使用相机内参将三维空间点投影到像素平面
        int col = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        int row = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);

        if ((col >= 0) && (col < image_size.width) && (row >= 0) && (row < image_size.height) && cam_cloud[i].z > 0) {
            colored_3d_point.x = in_cloud->points[i].x;
            colored_3d_point.y = in_cloud->points[i].y;
            colored_3d_point.z = in_cloud->points[i].z;

            cv::Vec3b rgb_pixel = current_frame.at<cv::Vec3b>(row, col);
            colored_3d_point.r = rgb_pixel[2] * 2;
            colored_3d_point.g = rgb_pixel[1] * 2;
            colored_3d_point.b = rgb_pixel[0] * 2;
            out_cloud->points.push_back(colored_3d_point);
        }

        for (size_t j = 0; j < objs.size(); j++) {
            if ((col < objs[j].xmax) && (col > objs[j].xmin) && (row > objs[j].ymin) && (row < objs[j].ymax) && (cam_cloud[i].z > 0)) {
                objs[j].pc.points.push_back(in_cloud->points[i]);
            }
        }
    }

    double cluster_dis_table[10] = { 5, 8, 10, 2, 2, 2, 2, 3, 10 };
    removed_lessPoints_objs.clear();
    for (size_t k = 0; k < objs.size(); k++) {
        if (double(objs[k].pc.points.size()) > 5) {
            removed_lessPoints_objs.push_back(objs[k]);
        }
    }
    // printf(">>>>>>>>>>>>>>>>>>>>rm objs  %f\n", double(removed_lessPoints_objs.size()));
    for (size_t k = 0; k < removed_lessPoints_objs.size(); k++) {
        // printf("before cluster %f\n", double(removed_lessPoints_objs[k].pc.points.size()));
        removeOutlier(removed_lessPoints_objs[k], cluster_dis_table[removed_lessPoints_objs[k].category - 1]);
    }

    for (size_t m = 0; m < removed_lessPoints_objs.size(); m++) {
        calObstacleInfo(removed_lessPoints_objs[m]);
    }

    publishObjs();

    sensor_msgs::PointCloud2 test_point;
    pcl::toROSMsg(*in_cloud_msg, test_point);
    test_point.header = cloud_msg->header;
    test_pointcloud.publish(test_point);

    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*out_cloud, out_cloud_msg);
    out_cloud_msg.header = cloud_msg->header;
    pub_fusion_cloud.publish(out_cloud_msg);

    usingObjs = false;
}

/**
 * @description: 截取点云，去除高度过高的点,去除距离激光雷达中心过近的点, 去除非车辆前面的点
 */
void PixelCloudFusion::clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
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
 * @description: 基于ray_groud_filtered对地面进行分割 
 */
void PixelCloudFusion::removeFloorRayFiltered(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
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
void PixelCloudFusion::convertXYZ2XYZRT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
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

void PixelCloudFusion::calObstacleInfo(Object& in_detect_obj)
{
    in_detect_obj.xmin_3d_bbox = 9999;
    in_detect_obj.xmax_3d_bbox = -9999;
    in_detect_obj.ymin_3d_bbox = 9999;
    in_detect_obj.ymax_3d_bbox = -9999;
    in_detect_obj.zmin_3d_bbox = 9999;
    in_detect_obj.zmax_3d_bbox = -9999;
    for (size_t j = 0; j < in_detect_obj.pc.points.size(); j++) {
        pcl::PointXYZ p = in_detect_obj.pc.points[j];

        in_detect_obj.xmin_3d_bbox = p.x < in_detect_obj.xmin_3d_bbox ? p.x : in_detect_obj.xmin_3d_bbox;
        in_detect_obj.ymin_3d_bbox = p.y < in_detect_obj.ymin_3d_bbox ? p.y : in_detect_obj.ymin_3d_bbox;
        in_detect_obj.zmin_3d_bbox = p.z < in_detect_obj.zmin_3d_bbox ? p.z : in_detect_obj.zmin_3d_bbox;
        in_detect_obj.xmax_3d_bbox = p.x > in_detect_obj.xmax_3d_bbox ? p.x : in_detect_obj.xmax_3d_bbox;
        in_detect_obj.ymax_3d_bbox = p.y > in_detect_obj.ymax_3d_bbox ? p.y : in_detect_obj.ymax_3d_bbox;
        in_detect_obj.zmax_3d_bbox = p.z > in_detect_obj.zmax_3d_bbox ? p.z : in_detect_obj.zmax_3d_bbox;
    }
}

void PixelCloudFusion::removeOutlier(Object& in_detect_obj, const double& max_cluster_dis)
{
    double cluster_min_points = 4;
    double cluster_max_points = 10000;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(in_detect_obj.pc, *cloud_2d);

    for (size_t i = 0; i < cloud_2d->points.size(); i++) {
        cloud_2d->points[i].z = 0;
    }

    tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc;

    // TODO 设置不同类别不同的聚类距离
    euc.setClusterTolerance(3);
    euc.setMinClusterSize(cluster_min_points);
    euc.setMaxClusterSize(cluster_max_points);
    euc.setSearchMethod(tree);
    euc.setInputCloud(cloud_2d);
    euc.extract(cluster_indices);

    double best_cluster_index = 0;
    double cluster_points_maxnum = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        if (cluster_indices[i].indices.size() > cluster_points_maxnum) {
            cluster_points_maxnum = cluster_indices[i].indices.size();
            best_cluster_index = i;
        }
    }

    pcl::PointCloud<pcl::PointXYZ> tmp_pc;
    tmp_pc.points.clear();

    if (cluster_indices.size() > 0) {
        in_detect_obj.center_x = 0;
        in_detect_obj.center_y = 0;
        in_detect_obj.center_z = 0;
        for (size_t i = 0; i < cluster_indices[best_cluster_index].indices.size(); i++) {
            pcl::PointXYZ p = in_detect_obj.pc.points[cluster_indices[best_cluster_index].indices[i]];
            if (p.z < 5) {
                // in_detect_obj.center_x += p.x;
                // in_detect_obj.center_y += p.y;
                // in_detect_obj.center_z += p.z;
                tmp_pc.points.push_back(p);
            }
        }
    }

    // in_detect_obj.center_x /= double(cluster_indices[best_cluster_index].indices.size());
    // in_detect_obj.center_y /= double(cluster_indices[best_cluster_index].indices.size());
    // in_detect_obj.center_z /= double(cluster_indices[best_cluster_index].indices.size());

    in_detect_obj.pc.points.clear();
    pcl::copyPointCloud(tmp_pc, in_detect_obj.pc);
    // printf("cluster after %f\n", double(in_detect_obj.pc.points.size()));
}

void PixelCloudFusion::publishObjs()
{
    visualization_msgs::MarkerArray objs_marker;
    visualization_msgs::Marker obj_marker;

    smartcar_msgs::DetectedObjectArray output_objs;
    smartcar_msgs::DetectedObject output_obj;

    obj_marker.header.frame_id = "velodyne";
    obj_marker.header.stamp = ros::Time();
    obj_marker.type = visualization_msgs::Marker::CUBE;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.color.a = 0.6;
    obj_marker.lifetime = ros::Duration(0.1);
    objs_marker.markers.clear();
    for (size_t k = 0; k < removed_lessPoints_objs.size(); k++) {

        // Rviz marker
        obj_marker.id = k;
        category_deal(obj_marker, removed_lessPoints_objs[k]);
        obj_marker.pose.position.x = (removed_lessPoints_objs[k].xmin_3d_bbox + removed_lessPoints_objs[k].xmax_3d_bbox) / 2;
        obj_marker.pose.position.y = (removed_lessPoints_objs[k].ymin_3d_bbox + removed_lessPoints_objs[k].ymax_3d_bbox) / 2;
        obj_marker.pose.position.z = (removed_lessPoints_objs[k].zmin_3d_bbox + removed_lessPoints_objs[k].zmax_3d_bbox) / 2;
        obj_marker.scale.x = std::max(double(0.1), removed_lessPoints_objs[k].xmax_3d_bbox - removed_lessPoints_objs[k].xmin_3d_bbox);
        obj_marker.scale.y = std::max(double(0.1), removed_lessPoints_objs[k].ymax_3d_bbox - removed_lessPoints_objs[k].ymin_3d_bbox);
        obj_marker.scale.z = std::max(double(0.1), removed_lessPoints_objs[k].zmax_3d_bbox - removed_lessPoints_objs[k].zmin_3d_bbox);
        objs_marker.markers.push_back(obj_marker);

        // objs msg
        // 障碍物中心点和id
        output_obj.id = k;
        output_obj.pose.position.x = (removed_lessPoints_objs[k].xmin_3d_bbox + removed_lessPoints_objs[k].xmax_3d_bbox) / 2;
        output_obj.pose.position.y = (removed_lessPoints_objs[k].ymin_3d_bbox + removed_lessPoints_objs[k].ymax_3d_bbox) / 2;
        output_obj.pose.position.z = (removed_lessPoints_objs[k].zmin_3d_bbox + removed_lessPoints_objs[k].zmax_3d_bbox) / 2;

        // 障碍物尺度
        output_obj.dimensions.x = obj_marker.scale.x;
        output_obj.dimensions.y = obj_marker.scale.y;
        output_obj.dimensions.z = obj_marker.scale.z;

        // 障碍物轮廓点
        output_obj.convex_hull.polygon.points.clear();
        std::vector<cv::Point2f> points_2d;
        points_2d.clear();
        for (size_t i = 0; i < removed_lessPoints_objs[k].pc.points.size(); i++) {
            cv::Point2f pp;
            pp.x = removed_lessPoints_objs[k].pc.points[i].x;
            pp.y = removed_lessPoints_objs[k].pc.points[i].y;
            points_2d.push_back(pp);
        }

        std::vector<cv::Point2f> hull;
        if (points_2d.size() > 0)
            cv::convexHull(points_2d, hull);
        output_obj.convex_hull.polygon.points.clear();
        for (size_t i = 0; i < hull.size(); i++) {
            geometry_msgs::Point32 pp;
            pp.x = hull[i].x;
            pp.y = hull[i].y;
            pp.z = 0;
            output_obj.convex_hull.polygon.points.push_back(pp);
        }
        output_objs.objects.push_back(output_obj);
    }
    objs_pub.publish(output_objs);
    objs_pub_rviz.publish(objs_marker);
}

void PixelCloudFusion::category_deal(visualization_msgs::Marker& objmarker, Object& obj)
{
    double colors_table[10][4] = {
        { 1, 0, 0, 1 }, // car
        { 0, 1, 0, 2 }, // truck
        { 0, 0, 1, 2 }, // bus
        { 0, 0, 1, 2 }, // bicycle
        { 1, 1, 1, 0.3 }, // pedestrian
        { 1, 0, 1, 0.5 }, // cyclist
        { 0.5, 0.2, 0.4, 0.1 }, // traffic_signs
        { 1, 0.2, 0.7, 0.1 }, // info_signs
        { 0.3, 0.2, 0.9, 0.5 }, // special
        { 0.5, 0.8, 0.3, 0.1 },
    };

    for (size_t i = 0; i < 10; i++) {
        if (obj.category == int(i)) {
            objmarker.color.b = colors_table[i][0];
            objmarker.color.g = colors_table[i][1];
            objmarker.color.r = colors_table[i][2];
            // objmarker.scale.z = colors_table[i][3];
        }
    }
}

void PixelCloudFusion::DetectionCallback(const yunle_sensor_msgs::DetectObjs& objs_msg)
{
    if (!usingObjs) {
        objs.clear();
        for (size_t i = 0; i < objs_msg.objs.size(); i++) {
            Object obj;
            obj.category = objs_msg.objs[i].category;
            obj.obj_deg = objs_msg.objs[i].obj_deg;
            obj.obj_dist = objs_msg.objs[i].obj_dist;
            obj.score = objs_msg.objs[i].score;
            obj.xmax = objs_msg.objs[i].xmax * 640 / 768;
            obj.xmin = objs_msg.objs[i].xmin * 640 / 768;
            obj.ymax = objs_msg.objs[i].ymax * 480 / 384;
            obj.ymin = objs_msg.objs[i].ymin * 480 / 384;
            obj.xmax_3d_pic = objs_msg.objs[i].xmax_3d * 640 / 768;
            obj.xmin_3d_pic = objs_msg.objs[i].xmin_3d * 640 / 768;
            obj.ymax_3d_pic = objs_msg.objs[i].ymax_3d * 480 / 384;
            obj.ymin_3d_pic = objs_msg.objs[i].ymin_3d * 480 / 384;
            objs.push_back(obj);
        }
    }
}

tf::StampedTransform PixelCloudFusion::FindTransform(const std::string& target_frame, const std::string source_frame)
{
    tf::StampedTransform transform;

    camera_lidar_tf_ok_ = false;

    try {
        // ros::Time(0)指定了时间为0，即获得最新有效的变换。
        // 改变获取当前时间的变换，即改为ros::Time::now(),不过now的话因为监听器有缓存区的原因。一般会出错
        // 参考：https://www.ncnynl.com/archives/201702/1313.html
        transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        camera_lidar_tf_ok_ = true;
        ROS_INFO("joint_pixel_pointcloud : camera-lidar-tf obtained");
    } catch (tf::TransformException ex) {
        ROS_INFO("joint_pixel_pointcloud : %s", ex.what());
    }

    return transform;
}

pcl::PointXYZ PixelCloudFusion::TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform)
{
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_transformed = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_transformed.x(), tf_point_transformed.y(), tf_point_transformed.z());
}

void PixelCloudFusion::initROS()
{
    std::string pointscloud_input, image_input, camera_info_input, fusison_output_topic, test_cloud_topic;
    nh_private.param<std::string>("pointcloud_input", pointscloud_input, "/velodyne_points");
    nh_private.param<std::string>("image_input", image_input, "/cv_camera/image_raw");
    nh_private.param<std::string>("camera_info_input", camera_info_input, "/camera_info");
    nh_private.param<std::string>("fusion_output_topic", fusison_output_topic, "/points_output");
    nh_private.param<std::string>("test_cloud_topic", test_cloud_topic, "/test_cloud");
    nh_private.param<double>("min_local_height_threshold", min_local_height_threshold, 0.05);

    nh_private.param<double>("radial_divide_angle", radial_divide_angle, 0.5);
    nh_private.param<double>("concentric_divide_distance", concentric_divide_distance, 0.1);

    nh_private.param<double>("clip_left_right_dis", clip_left_right_dis, 7);
    nh_private.param<double>("clip_far", clip_far, 40);
    nh_private.param<double>("clip_dis", clip_dis, 2);
    nh_private.param<double>("clip_height", clip_height, 4.0);

    nh_private.param<double>("sensor_height", sensor_height, 0.37);
    nh_private.param<double>("local_slope_threshold", local_slope_threshold, 5.0);
    nh_private.param<double>("general_slope_threshhold", general_slope_threshhold, 3.0);

    sub_cloud = nh.subscribe(pointscloud_input, 1, &PixelCloudFusion::CloudCallback, this);
    sub_image = nh.subscribe(image_input, 1, &PixelCloudFusion::ImageCallback, this);
    sub_intrinsics = nh.subscribe(camera_info_input, 1, &PixelCloudFusion::IntrinsicsCallback, this);
    sub_detection = nh.subscribe("detectObjs", 1, &PixelCloudFusion::DetectionCallback, this);

    pub_fusion_cloud = nh.advertise<sensor_msgs::PointCloud2>(fusison_output_topic, 1);
    test_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(test_cloud_topic, 1);
    objs_pub_rviz = nh.advertise<visualization_msgs::MarkerArray>("fusion_objs_rviz", 1);
    objs_pub = nh.advertise<smartcar_msgs::DetectedObjectArray>("fusion_objs", 1);
}

PixelCloudFusion::PixelCloudFusion()
    : nh_private("~")
    , camera_lidar_tf_ok_(false)
    , camera_info_ok_(false)
    , usingObjs(false)
    , image_frame_id("")
{
    initROS();
}
