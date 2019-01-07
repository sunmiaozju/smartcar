#include "joint_pixel_pointcloud.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pixel_point_node");

    NODE_JOINT_PIXEL_POINTCLOUD::PixelCloudFusion pcf_node;

    pcf_node.run();

    return 0;
}
