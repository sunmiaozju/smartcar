#include "ray_ground_filter_core.h"

RayGroundFilter::RayGroundFilter(ros::NodeHandle& nh)
    : nh(nh)
    , above_points(new pcl::PointCloud<pcl::PointXYZI>())
{
}
RayGroundFilter::~RayGroundFilter() {}

void RayGroundFilter::run()
{
    if (!init_param()) {
        ROS_ERROR("Can not init params, exit.");
        exit(1);
    }

    sub_point_cloud_ = nh.subscribe(lidar_topic, 10, &RayGroundFilter::point_cb, this);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_debug_map = nh.advertise<sensor_msgs::PointCloud2>(debug_map_topic, 10);
    pub_filtered_points = nh.advertise<sensor_msgs::PointCloud2>(no_ground_full_height_topic, 10);
}

bool RayGroundFilter::init_param()
{
    nh.param<std::string>("lidar_frame", lidar_frame, "/velodyne");
    nh.param<std::string>("lidar_topic", lidar_topic, "/velodyne_points");
    nh.param<std::string>("ground_topic", ground_topic, "/ray_filter/points_ground");
    nh.param<std::string>("no_ground_topic", no_ground_topic, "/ray_filter/points_no_ground");
    nh.param<std::string>("no_ground_topic_full", no_ground_full_height_topic, "/ray_filter/points_no_ground_full_height");
    std::cout << "    lidar_frame: " << lidar_frame << std::endl;
    std::cout << "    lidar_topic: " << lidar_topic << std::endl;
    std::cout << "   ground_topic: " << ground_topic << std::endl;
    std::cout << "no_ground_topic: " << no_ground_topic << std::endl;
    std::cout << "no_ground_topic_full: " << no_ground_full_height_topic << std::endl;
    std::cout << std::endl;

    nh.param<int>("sensor_model", SENSOR_MODEL, 16);
    nh.param<double>("sensor_height", SENSOR_HEIGHT, 0.3);
    nh.param<double>("local_max_slope", local_max_slope_, 8.0);
    nh.param<double>("general_max_slope", general_max_slope_, 5.0);
    nh.param<double>("min_height_threshold", min_height_threshold_, 0.05);
    nh.param<double>("reclass_distance_threshold", reclass_distance_threshold_, 0.2);
    nh.param<double>("radial_divider_angle", RADIAL_DIVIDER_ANGLE, 0.18);
    nh.param<double>("concentric_divider_distance", concentric_divider_distance_, 0.01);
    nh.param<double>("min_distance", MIN_DISTANCE, 2.4);
    std::cout << "               sensor_model: " << SENSOR_MODEL << std::endl;
    std::cout << "              sensor_height: " << SENSOR_HEIGHT << std::endl;
    std::cout << "            local_max_slope: " << local_max_slope_ << std::endl;
    std::cout << "          general_max_slope: " << general_max_slope_ << std::endl;
    std::cout << "       min_height_threshold: " << min_height_threshold_ << std::endl;
    std::cout << " reclass_distance_threshold: " << reclass_distance_threshold_ << std::endl;
    std::cout << "       radial_divider_angle: " << RADIAL_DIVIDER_ANGLE << std::endl;
    std::cout << "concentric_divider_distance: " << concentric_divider_distance_ << std::endl;
    std::cout << "               min_distance: " << MIN_DISTANCE << std::endl;
    std::cout << std::endl;

    nh.param<double>("clip_height", CLIP_HEIGHT, 1.0);
    nh.param<double>("minX", minX, -5.0);
    nh.param<double>("maxX", maxX, 20.0);
    nh.param<double>("minY", minY, -10.0);
    nh.param<double>("maxY", maxY, 10.0);
    std::cout << "clip_height: " << CLIP_HEIGHT << std::endl;
    std::cout << "       minX: " << minX << std::endl;
    std::cout << "       maxX: " << maxX << std::endl;
    std::cout << "       minY: " << minY << std::endl;
    std::cout << "       maxY: " << maxY << std::endl;
    std::cout << std::endl;

    nh.param<double>("remove_min_x", remove_min_x, -2.0);
    nh.param<double>("remove_min_y", remove_min_y, -0.5);
    nh.param<double>("remove_max_x", remove_max_x, 0);
    nh.param<double>("remove_max_y", remove_max_y, 0.5);
    std::cout << "clip_height: " << CLIP_HEIGHT << std::endl;
    std::cout << "       minX: " << minX << std::endl;
    std::cout << "       maxX: " << maxX << std::endl;
    std::cout << "       minY: " << minY << std::endl;
    std::cout << "       maxY: " << maxY << std::endl;
    std::cout << std::endl;

    // nh.param<double>("default_cluster_height",default_cluster_height,0.05);
    // nh.param<double>("default_plane_radius",default_plane_radius,3.0);
    // std::cout << "default_cluster_height: " << default_cluster_height << std::endl;
    // std::cout << "default_plane_radius: " << default_plane_radius << std::endl;
    // std::cout << std::endl;

    // DEBUG param
    nh.param<bool>("debug_pub_all", debug_pub_all, false);
    nh.param<std::string>("debug_map_topic", debug_map_topic, "/debug/ray_filter_debug");
    if (debug_pub_all) {
        std::cout << "publish debug map with topic: " << debug_map_topic << std::endl;
    }

    return true;
}

void RayGroundFilter::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for // #pragma omp for语法OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速的效果。
    for (size_t i = 0; i < in->points.size(); i++) {
        if (in->points[i].z > clip_height) {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);

    above_points->points.clear();
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); // false to save the indices
    cliper.filter(*above_points);
}

void RayGroundFilter::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance) {
            indices.indices.push_back(i);
        }
        // // extract ROI space  -- but ray ground filter can work well at full space
        // double x = in->points[i].x;
        // double y = in->points[i].y;
        // double z = in->points[i].z;
        // if (minX > x || x > maxX || minY > y || y > maxY) {
        //     indices.indices.push_back(i);
        // }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
void RayGroundFilter::remove_close_pt_rectangle(double remove_min_x,
    double remove_min_y,
    double remove_max_x,
    double remove_max_y,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) {

        double x = in->points[i].x;
        double y = in->points[i].y;
        double z = in->points[i].z;
        if (remove_min_x < x && x < remove_max_x && remove_min_y < y && y < remove_max_y) {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundFilter::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    PointCloudXYZIRTColor& out_organized_points,
    std::vector<pcl::PointIndices>& out_radial_divided_indices,
    std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0) {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } //end for

    //将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++) {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
            [](const PointXYZIRTColor& a, const PointXYZIRTColor& b) { return a.radius < b.radius; });
    }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::classify_pc(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
    pcl::PointIndices& out_ground_indices,
    pcl::PointIndices& out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance < concentric_divider_distance_ || height_threshold < min_height_threshold_) {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold)) {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground) {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold)) {
                        current_ground = true;
                    } else {
                        current_ground = false;
                    }
                } else {
                    current_ground = true;
                }
            } else {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ && (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold))) {
                    current_ground = true;
                } else {
                    current_ground = false;
                }
            }

            if (current_ground) {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            } else {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void RayGroundFilter::publish_cloud(const ros::Publisher& in_publisher,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
    const std_msgs::Header& in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void RayGroundFilter::publish_debug_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc, const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pc, const std_msgs::Header& in_header)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_map(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto point : ground_pc->points) {
        pcl::PointXYZRGB p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.r = 0;
        p.g = 0;
        p.b = 255;
        color_map->points.push_back(p);
    }

    for (auto point : no_ground_pc->points) {
        pcl::PointXYZRGB p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        color_map->points.push_back(p);
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*color_map, cloud_msg);
    cloud_msg.header = in_header;
    pub_debug_map.publish(cloud_msg);
}

void RayGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);

    // remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);
    remove_close_pt_rectangle(remove_min_x, remove_min_y, remove_max_x, remove_max_y, cliped_pc_ptr, remove_close);

    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

    XYZI_to_RTZColor(remove_close, organized_points,
        radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);

    *no_ground_cloud_ptr += *above_points; // 重载过的 +=: 如果使用points.insert(),则必须更新点云的width: PointCloud.resize(xxx)
    publish_cloud(pub_filtered_points, no_ground_cloud_ptr, in_cloud_ptr->header);

    if (debug_pub_all) {
        publish_debug_map(ground_cloud_ptr, no_ground_cloud_ptr, in_cloud_ptr->header);
    }
}

void RayGroundFilter::reset_params(const ray_ground_filter::RayFilterConfig& config)
{
    std::cout << "-----------------------------------------------" << std::endl;
    ROS_INFO_STREAM("Reconfigure ray_ground_filter config.");
    SENSOR_HEIGHT = config.sensor_height;
    RADIAL_DIVIDER_ANGLE = config.radial_divider_angle;
    local_max_slope_ = config.local_max_slope;
    general_max_slope_ = config.general_max_slope;
    CLIP_HEIGHT = config.clip_height;
    minX = config.minX;
    minY = config.minY;
    maxX = config.maxX;
    maxY = config.maxY;
    MIN_DISTANCE = config.min_distance;
    concentric_divider_distance_ = config.concentric_divider_distance;
    min_height_threshold_ = config.min_height_threshold;
    reclass_distance_threshold_ = config.reclass_distance_threshold;
    remove_min_x = config.remove_min_x;
    remove_min_y = config.remove_min_y;
    remove_max_x = config.remove_max_x;
    remove_max_y = config.remove_max_y;

    std::cout << "               sensor_model: " << SENSOR_MODEL << std::endl;
    std::cout << "              sensor_height: " << SENSOR_HEIGHT << std::endl;
    std::cout << "            local_max_slope: " << local_max_slope_ << std::endl;
    std::cout << "          general_max_slope: " << general_max_slope_ << std::endl;
    std::cout << "       min_height_threshold: " << min_height_threshold_ << std::endl;
    std::cout << " reclass_distance_threshold: " << reclass_distance_threshold_ << std::endl;
    std::cout << "       radial_divider_angle: " << RADIAL_DIVIDER_ANGLE << std::endl;
    std::cout << "concentric_divider_distance: " << concentric_divider_distance_ << std::endl;
    std::cout << "               min_distance: " << MIN_DISTANCE << std::endl;
    std::cout << "clip_height: " << CLIP_HEIGHT << std::endl;
    std::cout << "       minX: " << minX << std::endl;
    std::cout << "       maxX: " << maxX << std::endl;
    std::cout << "       minY: " << minY << std::endl;
    std::cout << "       maxY: " << maxY << std::endl;
    std::cout << "remove_min_x: " << remove_min_x << std::endl;
    std::cout << "remove_min_y: " << remove_min_y << std::endl;
    std::cout << "remove_max_x: " << remove_max_x << std::endl;
    std::cout << "remove_max_y: " << remove_max_y << std::endl;
    std::cout << std::endl;

    std::cout << "-----------------------------------------------" << std::endl;
}
