/*
 * @Description: collision avoid and local trajectory generator implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-15 14:53:47
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-20 15:46:08
 */
#include <ros/ros.h>
#include <local_trajectory_generator/local_trajectory_generator.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_trajectory_generator_node");
    LocalTrajectoryGeneratorNS::LocalTrajectoryGenerator app;
    app.run(); 
    return 0;
}

