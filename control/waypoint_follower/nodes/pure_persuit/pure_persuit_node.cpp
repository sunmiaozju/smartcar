#include "pure_persuit.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    waypoint_follower::PurePursuitNode ppn;
    ppn.run();
    return 0;
}
