#pragma once

#include <ros/ros.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <omp.h>
#include <ray_ground_filter/RayFilterConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class RayGroundFilter {

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_point_cloud_;
    geometry_msgs::PoseStamped current_pose;
    bool pub_global_noground_map;

    ros::Publisher pub_ground_, pub_no_ground_, pub_debug_map, pub_filtered_points;

    std::string lidar_frame;
    std::string lidar_topic;
    std::string ground_topic;
    std::string no_ground_topic;
    std::string no_ground_full_height_topic;

    pcl::PointCloud<pcl::PointXYZI>::Ptr above_points;

    int SENSOR_MODEL;
    double SENSOR_HEIGHT;
    // double default_cluster_height;
    // double default_plane_radius;

    double RADIAL_DIVIDER_ANGLE; // default: 0.18

    double local_max_slope_; // degree default: 8 //max slope of the ground between points, degree
    double general_max_slope_; // degree  default: 5 //max slope of the ground in entire point cloud, degree

    double CLIP_HEIGHT; //截取掉高于雷达自身xx米的点
    double minX;
    double maxX;
    double minY;
    double maxY;

    // 按照矩形形状,截取掉过近的点,这类点大多是车身
    double remove_min_x;
    double remove_min_y;
    double remove_max_x;
    double remove_max_y;

    // 按照圆形形状,截取掉过近的点,这类点大多是车身
    double MIN_DISTANCE; // default: 2.4

    double concentric_divider_distance_; // default: 0.01 //0.1 meters default
    double min_height_threshold_; // default: 0.05

    double reclass_distance_threshold_; // default: 0.2

    // DEBUG
    bool debug_pub_all;
    std::string debug_map_topic;

    struct PointXYZIRTColor {
        pcl::PointXYZI point;

        float radius; //cylindric coords on XY Plane
        float theta; //angle deg on XY plane

        size_t radial_div; //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to

        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;

    bool init_param();

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
    // void pose_cb(const geometry_msgs::PoseStampedConstPtr &current_pose_ptr);

    void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void remove_close_pt_rectangle(double remove_min_x,
        double remove_min_y,
        double remove_max_x,
        double remove_max_y,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
        PointCloudXYZIRTColor& out_organized_points,
        std::vector<pcl::PointIndices>& out_radial_divided_indices,
        std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);

    void classify_pc(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
        pcl::PointIndices& out_ground_indices,
        pcl::PointIndices& out_no_ground_indices);

    void publish_cloud(const ros::Publisher& in_publisher,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
        const std_msgs::Header& in_header);
    void publish_debug_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pc,
        const std_msgs::Header& in_header);

public:
    RayGroundFilter(ros::NodeHandle& nh_);
    ~RayGroundFilter();
    void run();

    void reset_params(const ray_ground_filter::RayFilterConfig& config);
};
