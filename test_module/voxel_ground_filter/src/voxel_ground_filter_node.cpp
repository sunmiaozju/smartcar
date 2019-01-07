#include <voxel_ground_filter/voxel_ground_filter_core.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"voxel_ground_filter_node");
    ros::NodeHandle nh("~");

    Voxel_Ground::Voxel_Ground_Filter filter(nh);
    filter.run();
    ros::spin();
    
    return 0;
}