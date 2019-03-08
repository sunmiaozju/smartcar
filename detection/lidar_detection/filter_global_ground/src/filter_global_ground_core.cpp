#include <filter_global_ground/save_map_core.h>
#include <ros/duration.h>

namespace SAVE_GRID_MAP{
    save_grid_map::save_grid_map():pnh("~"),global_gridmap_ptr(new pcl::PointCloud<point>){
        global_gridmap_ptr->header.frame_id = "/map";
    }
    save_grid_map::~save_grid_map(){}

    bool save_grid_map::init(){
        pnh.param<std::string>("in_topic",in_topic,"/points_no_ground");
        pnh.param<std::string>("out_topic",out_topic,"/global/points_no_ground");
        pnh.param<std::string>("lidar_frame",lidar_frame,"/velodyne");
        pnh.param<double>("tf_timeout",param_time_out,1.0);

		std::cout << "--------------no ground pc global-----------" << std::endl;
		std::cout << "in_topic: " << in_topic << std::endl;
		std::cout << "out_topic: " << out_topic << std::endl;
		std::cout << "lidar_frame: " << lidar_frame << std::endl;
		std::cout << "tf_timeout: " << param_time_out << std::endl;
		std::cout << std::endl;
		
        return true;
    }

    void save_grid_map::points_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
		pcl::PointCloud<point>::Ptr input_cloud_ptr(new pcl::PointCloud<point>());
        pcl::fromROSMsg(*msg,*input_cloud_ptr);

		tf::StampedTransform transform;
		try{
			ros::Time now = ros::Time::now();
			std::cout << ros::Time(0) << std::endl;
			ROS_INFO("now:%f listen from static_tf and set tf_btol",now);
			tf_listener.waitForTransform("base_link",lidar_frame,now,ros::Duration(param_time_out * 10), ros::Duration(param_time_out / 3));
			tf_listener.lookupTransform("base_link",lidar_frame,now,transform);
			// tf_listener.waitForTransform("base_link",lidar_frame,ros::Time(0),ros::Duration(param_time_out * 10), ros::Duration(param_time_out / 3));
			// tf_listener.lookupTransform("base_link",lidar_frame,ros::Time(0),transform);
			ROS_INFO("success listen from tf");
		}
		catch(const tf::TransformException &ex){
			ROS_ERROR("Error waiting for tf in points_cb: %s",ex.what());
			return;
		}
		Eigen::Translation3f tl_m2b(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
		double roll,pitch,yaw;
		Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
		mat_map2base = (tl_m2b * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        mat_map2laser = (mat_map2base*mat_base2laser).inverse();

        pcl::PointCloud<point>::Ptr transformed_pc_ptr(new pcl::PointCloud<point>());
        pcl::transformPointCloud(*input_cloud_ptr,*transformed_pc_ptr,mat_map2laser);

        *global_gridmap_ptr += *transformed_pc_ptr;

        sensor_msgs::PointCloud2::Ptr msg_out_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*global_gridmap_ptr,*msg_out_ptr);
        pub_global_grid_pc.publish(*msg_out_ptr);
    }

    void save_grid_map::run(){
        if(!init()){
            ROS_ERROR("Can not init save_global_gridmap params, exit.");
            exit(1);
        }
		ros::Duration(1.0).sleep();

		tf::StampedTransform transform;
		try{
			ros::Time now = ros::Time::now();
			ROS_INFO("now:%f listen from static_tf and set matrix_base2laser");
			tf_listener.waitForTransform("base_link",lidar_frame,ros::Time(0),ros::Duration(param_time_out * 10), ros::Duration(param_time_out / 3));
			tf_listener.lookupTransform("base_link",lidar_frame,ros::Time(0),transform);
			ROS_INFO("success listen from tf and set matrix_base2laser");
		}
		catch(const tf::TransformException &ex){
			ROS_ERROR("Error waiting for tf in init(what): %s",ex.what());
			// return;
		}
		Eigen::Translation3f tl_btol(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
		double roll,pitch,yaw;
		Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
		mat_base2laser = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

        sub_points = nh.subscribe(in_topic,1000,&save_grid_map::points_cb,this);
        pub_global_grid_pc = nh.advertise<sensor_msgs::PointCloud2>(out_topic,10);

    }
}