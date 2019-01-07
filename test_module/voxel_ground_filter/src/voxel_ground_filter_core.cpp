#include <voxel_ground_filter/voxel_ground_filter_core.h>

namespace Voxel_Ground{
    // Voxel_Ground_Filter::Voxel_Ground_Filter(ros::NodeHandle &nh):handle(nh){};
    // Voxel_Ground_Filter::~Voxel_Ground_Filter(){};

    bool Voxel_Ground_Filter::init_param(){
        handle.param<std::string>("lidar_frame",lidar_frame_,"/velodyne");
        handle.param<std::string>("lidar_topic",lidar_topic_,"/velodyne_points");
        handle.param<std::string>("ground_pc_topic",ground_pc_topic_,"/cloud/ground_cloud");
        handle.param<std::string>("no_ground_pc_topic",no_ground_pc_topic_,"/cloud/no_ground_cloud");
        std::cout << "       lidar_frame: " << lidar_frame_<< std::endl;
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
        std::cout << std::endl;

        handle.param<bool>("is_clipRadius",is_clipRadius,true);
        handle.param<double>("minX",minX,-5.0);
        handle.param<double>("maxX",maxX,20.0);
        handle.param<double>("minY",minY,-10.0);
        handle.param<double>("maxY",maxY,10.0);
        width = maxY - minY;
        length = maxX - minX;
        std::cout << "if clipRadius: " << is_clipRadius << std::endl;
        if (is_clipRadius){
            std::cout << "minX: " << minX << std::endl;
            std::cout << "maxX: " << maxX << std::endl;
            std::cout << "minY: " << minY << std::endl;
            std::cout << "maxY: " << maxY << std::endl;
        }
        std::cout << std::endl;


        handle.param<bool>("is_clipHeight",is_clipHeight,true);
        handle.param<double>("minClipHeight",minClipHeight,1.0);
        std::cout << "if clipHeight: " << is_clipHeight << std::endl;
        if (is_clipHeight){
            std::cout << "minClipHeight: " << minClipHeight << std::endl;
        }
        std::cout << std::endl;

        handle.param<double>("grid_size",grid_size,0.1);
        handle.param<double>("threshold_desc",threshold_desc,0.3);
        handle.param<double>("threshold_var",threshold_var,0.3);
        std::cout << "     grid_size: " << threshold_desc << std::endl;
        std::cout << "threshold_desc: " << threshold_desc << std::endl;
        std::cout << " threshold_var: " << threshold_var << std::endl;
        std::cout << std::endl;

        return true;
    }

    void Voxel_Ground_Filter::run(){
        if(!init_param()){
            ROS_ERROR("Can not init param, exit.");
            exit(1);
        }

        inputCloud_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);
        ground_pc_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);
        no_ground_pc_ptr = pcl::PointCloud<PointI>::Ptr(new pcl::PointCloud<PointI>);

        cloud_sub = handle.subscribe(lidar_topic_,10,&Voxel_Ground_Filter::points_cb,this);
		ground_pc_pub = handle.advertise<sensor_msgs::PointCloud2>(ground_pc_topic_,10);
        no_ground_pc_pub = handle.advertise<sensor_msgs::PointCloud2>(no_ground_pc_topic_,10);
        thre_pub = handle.advertise<std_msgs::Float64>("/threshold_var",10);
    }

    void Voxel_Ground_Filter::points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud_ptr){
		pcl::fromROSMsg(*input_cloud_ptr,*inputCloud_ptr);

		pcl::PointCloud<PointI>::Ptr downsampledCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr ROICloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr clipedHeightCloud(new pcl::PointCloud<PointI>);

		if(is_downSample){
			downsampleCloud(inputCloud_ptr,downsampledCloud);
		}else{
			downsampledCloud = inputCloud_ptr;
		}

		if(is_clipRadius){
			getROICloud(downsampledCloud,ROICloud);
		}else{
			ROICloud = downsampledCloud;
		}

		if(is_clipHeight){
			clipHeight(ROICloud,clipedHeightCloud);
		}else{
			clipedHeightCloud = ROICloud;
		}

        int rows = ceil(length/grid_size);
        int cols = ceil(width/grid_size);
        int nums = rows * cols;
        std::vector<PointI> cell[rows+1][cols+1];

        for(auto point:clipedHeightCloud->points){
            int row = floor((point.x - minX)/grid_size);
            int col = floor((point.y - minY)/grid_size);
            cell[row][col].push_back(point);
        }

        ground_pc_ptr->points.clear();
        no_ground_pc_ptr->points.clear();

        for(int i=0;i<rows;i++){
            for(int j=0;j<cols;j++){
                if(cell[i][j].size() > 0){
                    if(is_ground(cell[i][j])){
                        ground_pc_ptr->points.insert(ground_pc_ptr->points.end(),cell[i][j].begin(),cell[i][j].end());
                    }
                    else{
                        no_ground_pc_ptr->points.insert(no_ground_pc_ptr->points.end(),cell[i][j].begin(),cell[i][j].end());
                    }
                }
            }
        }
		sensor_msgs::PointCloud2 ground_cloud;
		pcl::toROSMsg(*ground_pc_ptr,ground_cloud);
		ground_cloud.header.frame_id=lidar_frame_;
		ground_pc_pub.publish(ground_cloud);

		sensor_msgs::PointCloud2 no_ground_cloud;
		pcl::toROSMsg(*no_ground_pc_ptr,no_ground_cloud);
		no_ground_cloud.header.frame_id=lidar_frame_;
		no_ground_pc_pub.publish(no_ground_cloud);
    }

    bool Voxel_Ground_Filter::is_ground(std::vector<PointI> &in){
        if(in.empty()){
            std::cout << "catch an empty vector." << std::endl;
            return false;
        }
        std::vector<double> var_z;
        double sum = 0.0;
        for(auto point:in){
            var_z.push_back(point.z);
            sum += point.z;
        }
        double average = sum / in.size();
        double threshold = calc_variance(var_z,average);
        if(threshold < threshold_var){
            return true;
        }else{
            return false;
        }
    }

    double Voxel_Ground_Filter::calc_variance(const std::vector<double> &in,double average){
        double sum = 0.0;
        for(auto var:in){
            sum += std::pow((var - average),2.0);
        }
        double threshold = sum / in.size();
        std_msgs::Float64 num;
        num.data = threshold;
        thre_pub.publish(num);
        return threshold;
    }

	void Voxel_Ground_Filter::downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		pcl::VoxelGrid<PointI> voxelGrid;
		voxelGrid.setLeafSize(voxel_leaf_dize,voxel_leaf_dize,voxel_leaf_dize);
		voxelGrid.setInputCloud(inputCloud);
		voxelGrid.filter(*outCloud);
	}

	void Voxel_Ground_Filter::getROICloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			if(minX <= point.x && point.x <=maxX && minY <= point.y && point.y <= maxY){
				outCloud->points.push_back(point);
                // if(point.x <= minX){
                //     std::cout << minX << std::endl;
                //     std::cout << point.x << std::endl;
                //     ROS_ERROR("a fly fish");
                //     exit(1);
                // }
            }
		}

	}

	void Voxel_Ground_Filter::clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			if(-2.0 <= point.z && point.z <= minClipHeight){  // remove lower noise as well
				outCloud->points.push_back(point);
			}
		}
	}
}