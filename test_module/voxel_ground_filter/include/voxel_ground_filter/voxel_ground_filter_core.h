#ifndef VOXEL_GROUND_FILTER_CORE_H
#define VOXEL_GROUND_FILTER_CORE_H

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <vector>
#include <std_msgs/Float64.h>

namespace Voxel_Ground{
    typedef pcl::PointXYZI PointI;
    typedef pcl::PointCloud<PointI> PointCloudI;

    class Voxel_Ground_Filter{
        private:
            ros::NodeHandle handle;
            ros::Subscriber cloud_sub;
            ros::Publisher ground_pc_pub;
            ros::Publisher no_ground_pc_pub;
            ros::Publisher thre_pub;

            std::string lidar_frame_;

            std::string lidar_topic_;
            std::string ground_pc_topic_;
            std::string no_ground_pc_topic_;

            PointCloudI::Ptr inputCloud_ptr;
            PointCloudI::Ptr ground_pc_ptr;
            PointCloudI::Ptr no_ground_pc_ptr;

            bool is_downSample;
            double voxel_leaf_dize;

            bool is_clipRadius;
            double minX;
            double maxX;
            double minY;
            double maxY;

            bool is_clipHeight;
            double minClipHeight;

            double width;
            double length;
            
            double grid_size;

            double threshold_desc;  // 极值阈值
            double threshold_var;  // 方差阈值

        public:
            Voxel_Ground_Filter(ros::NodeHandle &nh):handle(nh){};
            ~Voxel_Ground_Filter(){};
            
            bool cmp(PointI a,PointI b);

            bool init_param();

            void run();

            void points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud_ptr);

            bool is_ground(std::vector<PointI> &in);
            
            double calc_variance(const std::vector<double> &in, double average);

		    void downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud);

			void getROICloud(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

    };
}




#endif
