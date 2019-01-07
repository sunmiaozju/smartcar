//
// Created by yunle on 18-12-4.
//

#include <fast_ndt_mapping/LidarMapping.h>
#include <iostream>
#include <ros/ros.h>

int main(int argc,char** argv){
	std::cout<<(CUDA_FOUND? "CUDA_FOUND":"CUDA_NOT_FOUND")<<std::endl;
	ros::init(argc,argv,"ndt_mapping_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	FAST_NDT::LidarMapping mapping;
	mapping.run(nh,private_nh);
	ros::Rate(10);
	ros::spin();
	return 0;
}











