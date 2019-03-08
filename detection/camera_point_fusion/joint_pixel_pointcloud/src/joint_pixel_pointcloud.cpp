/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-02-21 21:34:40
 * @LastEditTime: 2019-03-08 22:15:08
 */
#include "joint_pixel_pointcloud.h"

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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *in_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    out_cloud->points.clear();
    pcl::PointXYZRGB colored_3d_point;
    for (size_t k = 0; k < objs.size(); k++) {
        objs[k].pc.points.clear();
    }
    
    // 存储处理后的点云
    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf);
        transformed_cloud->points.push_back(cam_cloud[i]);
        // 使用相机内参将三维空间点投影到像素平面
        int col = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        int row = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);
        
        if ((col >= 0) && (col < image_size.width) && (row >= 0) && (row < image_size.height) && cam_cloud[i].z > 0) {
            colored_3d_point.x = cam_cloud[i].x;
            colored_3d_point.y = cam_cloud[i].y;
            colored_3d_point.z = cam_cloud[i].z;

            cv::Vec3b rgb_pixel = current_frame.at<cv::Vec3b>(row, col);
            colored_3d_point.r = rgb_pixel[2] * 2;
            colored_3d_point.g = rgb_pixel[1] * 2;
            colored_3d_point.b = rgb_pixel[0] * 2;
            out_cloud->points.push_back(colored_3d_point);
        }
        
        for (size_t j = 0; j < objs.size(); j++) {
            if ((col < objs[j].xmax) && (col > objs[j].xmin) && (row > objs[j].ymin) && (row < objs[j].ymax) && (cam_cloud[i].z > 0)) {
                objs[j].pc.points.push_back(cam_cloud[i]);
            }
        }
    }
    double cluster_dis_table[10] = { 5, 8, 10, 2, 2, 2, 2, 3, 10 };
    
    for (size_t k = 0; k < objs.size(); k++) {
        if(objs[k].pc.points.size() > 0){
            removeOutlier(objs[k], cluster_dis_table[objs[k].category - 1]);
        }
        else{
            objs.erase(objs.begin() + k);
        }
    }
    for (size_t m = 0; m < objs.size(); m++) {
        calObstacleInfo(objs[m]);
    }

    publishObjs();

    sensor_msgs::PointCloud2 test_point;
    pcl::toROSMsg(*transformed_cloud, test_point);
    test_point.header = cloud_msg->header;
    transformed_pointcloud.publish(test_point);

    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*out_cloud, out_cloud_msg);
    out_cloud_msg.header = cloud_msg->header;
    pub_fusion_cloud.publish(out_cloud_msg);
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
        objs[j].xmin_3d_bbox = p.x < objs[j].xmin_3d_bbox ? p.x : objs[j].xmin_3d_bbox;
        objs[j].ymin_3d_bbox = p.y < objs[j].ymin_3d_bbox ? p.y : objs[j].ymin_3d_bbox;
        objs[j].zmin_3d_bbox = p.z < objs[j].zmin_3d_bbox ? p.z : objs[j].zmin_3d_bbox;
        objs[j].xmax_3d_bbox = p.x > objs[j].xmax_3d_bbox ? p.x : objs[j].xmax_3d_bbox;
        objs[j].ymax_3d_bbox = p.y > objs[j].ymax_3d_bbox ? p.y : objs[j].ymax_3d_bbox;
        objs[j].zmax_3d_bbox = p.z > objs[j].zmax_3d_bbox ? p.z : objs[j].zmax_3d_bbox;
    }

}

void PixelCloudFusion::removeOutlier(Object& in_detect_obj, const double& max_cluster_dis)
{
    double cluster_min_points = 10;
    double cluster_max_points = 100000;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::copyPointCloud(in_detect_obj.pc, *cloud_2d);

    for (size_t i = 0; i < cloud_2d->points.size(); i++) {
        cloud_2d->points[i].z = 0;
    }

    tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc;

    euc.setClusterTolerance(max_cluster_dis);
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
    // pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud;
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(in_detect_obj.pc);
    // extract.setIndices(boost::shared_ptr<::pcl::PointIndices>(&cluster_indices[best_cluster_index]));
    // extract.setNegative(false); // true removes the indices, false leaves only the indices
    // extract.filter(*out_cloud);

    pcl::PointCloud<pcl::PointXYZ> tmp_pc;

    if(cluster_indices.size() > 0){
        //printf("%f\n", cluster_indices[best_cluster_index].indices.size());
        for(size_t i = 0; i < cluster_indices[best_cluster_index].indices.size(); i++){
            tmp_pc.points.push_back(in_detect_obj.pc.points[cluster_indices[best_cluster_index].indices[i]]);
        }
    }
    pcl::copyPointCloud(tmp_pc, in_detect_obj.pc);
    // in_detect_obj.pc = tmp_pc;
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
    obj_marker.color.a = 0.8;
    obj_marker.lifetime = ros::Duration(0.1);
    printf("%s\n", "00");   
    for (size_t k = 0; k < objs.size(); k++) {
        obj_marker.id = k;
        category_deal(obj_marker, objs[k]);
        obj_marker.pose.position.x = (objs[k].xmin_3d_bbox + objs[k].xmax_3d_bbox) / 2;
        obj_marker.pose.position.y = (objs[k].ymin_3d_bbox + objs[k].ymax_3d_bbox) / 2;
        obj_marker.pose.position.z = (objs[k].zmin_3d_bbox + objs[k].zmax_3d_bbox) / 2;
        obj_marker.scale.x = std::max(float(0.1), objs[k].xmax_3d_bbox - objs[k].xmin_3d_bbox);
        obj_marker.scale.y = std::max(float(0.1), objs[k].ymax_3d_bbox - objs[k].ymin_3d_bbox);
        objs_marker.markers.push_back(obj_marker);
        printf("%s\n", "11");  
        //output_obj.id = k;
        //output_obj.pose.position.x = (objs[k].xmin_3d_bbox + objs[k].xmax_3d_bbox) / 2;
        //output_obj.pose.position.y = (objs[k].ymin_3d_bbox + objs[k].ymax_3d_bbox) / 2;
        //output_obj.pose.position.z = (objs[k].zmin_3d_bbox + objs[k].zmax_3d_bbox) / 2;
        //output_obj.convex_hull.polygon.points.clear();
        //geometry_msgs::Point32 pp;
        //pp.x = objs[k].xmin_3d_bbox;
        //pp.y = objs[k].ymin_3d_bbox;
        //pp.z = objs[k].zmin_3d_bbox;
        //output_obj.convex_hull.polygon.points.push_back(pp);
        //pp.x = objs[k].xmin_3d_bbox;
        //pp.y = objs[k].ymax_3d_bbox;
        //pp.z = objs[k].zmin_3d_bbox;
        //output_obj.convex_hull.polygon.points.push_back(pp);
        //pp.x = objs[k].xmax_3d_bbox;
        //pp.y = objs[k].ymin_3d_bbox;
        //pp.z = objs[k].zmin_3d_bbox;
        //output_obj.convex_hull.polygon.points.push_back(pp);
        //pp.x = objs[k].xmax_3d_bbox;
        //pp.y = objs[k].ymax_3d_bbox;
        //pp.z = objs[k].zmin_3d_bbox;
        //output_obj.convex_hull.polygon.points.push_back(pp);
        //output_objs.objects.push_back(output_obj);
        printf("%s\n", "22");  
    }
    printf("%s\n", "33");  
    //objs_pub.publish(output_objs);
    //objs_pub_rviz.publish(objs_marker);
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
        if (obj.category == i) {
            objmarker.color.b = colors_table[i][0];
            objmarker.color.g = colors_table[i][1];
            objmarker.color.r = colors_table[i][2];
            // objmarker.scale.z = colors_table[i][3];
        }
    }
}

void PixelCloudFusion::DetectionCallback(const yunle_sensor_msgs::DetectObjs& objs_msg)
{
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
    std::string pointscloud_input, image_input, camera_info_input, fusison_output_topic, transformed_cloud_topic;
    nh_private.param<std::string>("pointcloud_input", pointscloud_input, "/velodyne_points");
    nh_private.param<std::string>("image_input", image_input, "/cv_camera/image_raw");
    nh_private.param<std::string>("camera_info_input", camera_info_input, "/camera_info");
    nh_private.param<std::string>("fusion_output_topic", fusison_output_topic, "/points_output");
    nh_private.param<std::string>("transformed_cloud_topic", transformed_cloud_topic, "/transformed_cloud");

    sub_cloud = nh.subscribe(pointscloud_input, 1, &PixelCloudFusion::CloudCallback, this);
    sub_image = nh.subscribe(image_input, 1, &PixelCloudFusion::ImageCallback, this);
    sub_intrinsics = nh.subscribe(camera_info_input, 1, &PixelCloudFusion::IntrinsicsCallback, this);
    sub_detection = nh.subscribe("detectObjs", 1, &PixelCloudFusion::DetectionCallback, this);

    pub_fusion_cloud = nh.advertise<sensor_msgs::PointCloud2>(fusison_output_topic, 1);
    transformed_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(transformed_cloud_topic, 1);
    objs_pub_rviz = nh.advertise<visualization_msgs::MarkerArray>("objs", 1);
    objs_pub = nh.advertise<smartcar_msgs::DetectedObjectArray>("one_obj", 1);
}

PixelCloudFusion::PixelCloudFusion()
    : nh_private("~")
    , camera_lidar_tf_ok_(false)
    , camera_info_ok_(false)
    , image_frame_id("")
{
    initROS();
}
