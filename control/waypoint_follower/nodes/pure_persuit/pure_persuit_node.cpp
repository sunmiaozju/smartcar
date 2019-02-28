/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-01-29 12:16:21
 * @LastEditTime: 2019-02-28 21:53:52
 */
#include "pure_persuit.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    waypoint_follower::PurePursuitNode ppn;
    ppn.run();
    return 0;
}
