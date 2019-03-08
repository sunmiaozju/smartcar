//
// Created by adam on 18-9-21.
//

#include "ray_ground_filter_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ray_ground_filter_node");

    ros::NodeHandle nh_("~");

    RayGroundFilter filter(nh_);
    filter.run();

    ros::spin();
    return 0;
}
