#include "image_rectifier.h"

using namespace image_rectifier;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, _NODE_NAME_);

    ImageRectifier rectifier_node;

    ros::spin();

    return 0;
}
