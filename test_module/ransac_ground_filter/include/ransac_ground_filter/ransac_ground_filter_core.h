#ifndef RANSAC_GROUND_FILTER_CORE_H
#define RANSAC_GROUND_FILTER_CORE_H

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

namespace RANSAC_GROUND{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointXYZI PointI;

	class Ransac_Filter{
		private:
			typedef boost::shared_ptr<Ransac_Filter> Ptr;

			ros::Subscriber cloud_sub;
			ros::Publisher ground_pub;
            ros::Publisher no_ground_pub;

			pcl::PointCloud<PointI>::Ptr inputCloud_ptr;
            pcl::PointCloud<PointI>::Ptr ground_pc_ptr;
            pcl::PointCloud<PointI>::Ptr no_ground_pc_ptr;

            std::string lidar_topic_;
            std::string ground_pc_topic_;
            std::string no_ground_pc_topic_;

            bool is_downSample;
            double voxel_leaf_dize;

            bool is_clipRadius;
            double minClipRadius;
            double maxClipRadius;

            bool is_clipHeight;
            double minClipHeight;

			double _maxFloorAngle; 
			double _maxFloorHeight; 
		public:
			Ransac_Filter(){};
            ~Ransac_Filter(){};

			void setup(ros::NodeHandle &handle);

			void points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud_ptr);

		    void downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud);

		    void polyfit(pcl::PointCloud<PointI> inputCloud);

			void removeCloseRearNoise(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void extractRoad(pcl::PointCloud<PointI>::Ptr inputCloud);

	};
}






#endif