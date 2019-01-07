#include <ransac_ground_filter/ransac_ground_filter_core.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"ransac_ground_filter_node");
    ros::NodeHandle nh("~");

    RANSAC_GROUND::Ransac_Filter app;
    app.setup(nh);
    ros::spin();
    return 0;
}