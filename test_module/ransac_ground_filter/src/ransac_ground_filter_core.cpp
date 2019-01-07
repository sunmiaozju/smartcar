#include <ransac_ground_filter/ransac_ground_filter_core.h>

namespace RANSAC_GROUND{
	void Ransac_Filter::setup(ros::NodeHandle &handle) {
        handle.param<std::string>("lidar_topic",lidar_topic_,"/velodyne_points");
        handle.param<std::string>("ground_pc_topic",ground_pc_topic_,"/cloud/ground_cloud");
        handle.param<std::string>("no_ground_pc_topic",no_ground_pc_topic_,"/cloud/no_ground_cloud");
        std::cout << "       lidar_topic: " << lidar_topic_ << std::endl;
        std::cout << "   ground_pc_topic: " << ground_pc_topic_ << std::endl;
        std::cout << "no_ground_pc_topic: " << no_ground_pc_topic_ << std::endl;
        std::cout << std::endl;

        handle.param<bool>("is_downSample",is_downSample,true);
        handle.param<double>("voxel_leaf_size",voxel_leaf_dize,1.0);
        std::cout << "if downSample: " << is_downSample << std::endl;
        if (is_downSample){
            std::cout << "vexel_leaf_size: " << voxel_leaf_dize << std::endl;
        }

        handle.param<bool>("is_clipRadius",is_clipRadius,true);
        handle.param<double>("minClipRadius",minClipRadius,2.0);
        handle.param<double>("maxClipRadius",maxClipRadius,70.0);
        std::cout << "if clipRadius: " << is_clipRadius << std::endl;
        if (is_clipRadius){
            std::cout << "minClipRadius: " << minClipRadius << std::endl;
            std::cout << "maxClipRadius: " << maxClipRadius << std::endl;
        }

        handle.param<bool>("is_clipHeight",is_clipHeight,true);
        handle.param<double>("minClipHeight",minClipHeight,1.0);
        std::cout << "if clipHeight: " << is_clipHeight << std::endl;
        if (is_downSample){
            std::cout << "minClipHeight: " << minClipHeight << std::endl;
        }
        std::cout << std::endl;

        handle.param<double>("maxFloorAngle",_maxFloorAngle,1.0);
        handle.param<double>("maxFloowHeight",_maxFloorHeight,0.15);
        std::cout << "maxFloorAngle: " << _maxFloorAngle << std::endl;
        std::cout << "maxFloorHeight: " << _maxFloorHeight << std::endl;

        inputCloud_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);
        ground_pc_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);
        no_ground_pc_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

        cloud_sub = handle.subscribe(lidar_topic_,10,&Ransac_Filter::points_cb,this);
		ground_pub = handle.advertise<sensor_msgs::PointCloud2>(ground_pc_topic_,10);
        no_ground_pub = handle.advertise<sensor_msgs::PointCloud2>(no_ground_pc_topic_,10);
	}

	void Ransac_Filter::points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud_ptr) {
		pcl::fromROSMsg(*input_cloud_ptr,*inputCloud_ptr);

		pcl::PointCloud<PointI>::Ptr downsampledCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr noiseRemovedCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr clipedHeightCloud(new pcl::PointCloud<PointI>);

		if(is_downSample){
			downsampleCloud(inputCloud_ptr,downsampledCloud);
		}else{
			downsampledCloud = inputCloud_ptr;
		}

		if(is_clipRadius){
			removeCloseRearNoise(downsampledCloud,noiseRemovedCloud);
		}else{
			noiseRemovedCloud = downsampledCloud;
		}

		if(is_clipHeight){
			clipHeight(noiseRemovedCloud,clipedHeightCloud);
		}else{
			clipedHeightCloud = noiseRemovedCloud;
		}

		// 提取路面并发布
		extractRoad(clipedHeightCloud);

		sensor_msgs::PointCloud2 ground_cloud;
		pcl::toROSMsg(*ground_pc_ptr,ground_cloud);
		ground_cloud.header.frame_id="velodyne";
		ground_pub.publish(ground_cloud);

		sensor_msgs::PointCloud2 no_ground_cloud;
		pcl::toROSMsg(*no_ground_pc_ptr,no_ground_cloud);
		no_ground_cloud.header.frame_id="velodyne";
		no_ground_pub.publish(no_ground_cloud);
	}

	void Ransac_Filter::polyfit(pcl::PointCloud<PointI> inputCloud) {

	}

	void Ransac_Filter::downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		pcl::VoxelGrid<PointI> voxelGrid;
		voxelGrid.setLeafSize(voxel_leaf_dize,voxel_leaf_dize,voxel_leaf_dize);
		voxelGrid.setInputCloud(inputCloud);
		voxelGrid.filter(*outCloud);
	}

	void Ransac_Filter::removeCloseRearNoise(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
        double close = minClipRadius * minClipRadius;
        double rear = maxClipRadius * maxClipRadius;
		for(auto point:inputCloud->points){
			double distance = std::pow(point.x,2) + std::pow(point.y,2);
			if(close <= distance && distance <= rear)
				outCloud->points.push_back(point);
		}

	}

	void Ransac_Filter::clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			if(point.z <= minClipHeight && point.x >= -5.0){
				outCloud->points.push_back(point);
			}
		}
	}

	void Ransac_Filter::extractRoad(pcl::PointCloud<PointI>::Ptr inputCloud) {
		pcl::SACSegmentation<PointI> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);  // RACSAC方法，其他还有LMedS方法等
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setAxis(Eigen::Vector3f(0,0,1));
		seg.setEpsAngle(_maxFloorAngle); // the maximum allowed difference between the model normal and the given axis in radians.
		seg.setDistanceThreshold(_maxFloorHeight);  // 判断是否为模型内点的距离阈值

		seg.setInputCloud(inputCloud);
		seg.segment(*inliers,*coefficients);
		if(inliers->indices.empty()){
			std::cout<<"Could not extract the road(plane)"<<std::endl;
		}

		// pcl/filters/extract_indices.h
		pcl::ExtractIndices<PointI> extract;
		extract.setInputCloud(inputCloud);
		extract.setIndices(inliers);
		extract.setNegative(false);  // false to save inliers
		extract.filter(*ground_pc_ptr);

        extract.setNegative(true);  // true to remove inliers
        extract.filter(*no_ground_pc_ptr);
	}

}