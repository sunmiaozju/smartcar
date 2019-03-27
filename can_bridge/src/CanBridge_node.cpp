/*
 * @Description: can module for communication with ecu and radar
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 13:56:12
 * @LastEditTime: 2019-03-27 09:47:49
 */

#include <CanBridge.h>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, CanBridge::node_name.data());
    CanBridge::Can_app app;
    // app.run(app);
    return 0;
}
