/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 12:05:39
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 14:28:09
 */

#include "rollout_generator/rollout_generator.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rollout_generator_node");
    RolloutGeneratorNS::RolloutGenerator app;
    app.run();
    return 0;
}
