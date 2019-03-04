#include <filter_global_ground/save_map_core.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"save_no_ground_global_map");
    
    SAVE_GRID_MAP::save_grid_map saver;
    saver.run();

    ros::spin();

    return 0;
}