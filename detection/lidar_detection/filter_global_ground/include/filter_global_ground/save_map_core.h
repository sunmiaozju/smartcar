#ifndef _SAVE_MAP_CORE_H
#define _SAVE_MAP_CORE_H
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <omp.h>

namespace SAVE_GRID_MAP{
    typedef pcl::PointXYZI point;

    class save_grid_map{
        private:
            ros::NodeHandle nh;
            ros::NodeHandle pnh;

            ros::Subscriber sub_points;
            ros::Publisher pub_global_grid_pc;

            // pcl::PointCloud<point>::Ptr input_cloud_ptr;
            tf::TransformListener tf_listener;
            Eigen::Matrix4f mat_map2base,mat_base2laser,mat_map2laser;
            double param_time_out;

            std::string out_topic;
            std::string in_topic;
            std::string lidar_frame;

            pcl::PointCloud<point>::Ptr global_gridmap_ptr;

            bool init();

            void points_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

        public:
            save_grid_map();
            ~save_grid_map();

            void run();
    };
    
}




#endif