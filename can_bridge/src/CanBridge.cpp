/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:20:45
 * @LastEditTime: 2019-03-25 23:09:28
 */

#include <CanBridge.h>

namespace CanBridge {
Can_app::Can_app()
    : nh_private("~")
{
    initROS();
    pCanClient = CanBridge::CANClient::creatInstance(dev_name);
    if (pCanClient = NULL) {
        ROS_ERROR("opencv device error: %s", dev_name.data());
        exit(EXIT_FAILURE);
    }
}

/**
 * @description: init for ros params 
 */
void Can_app::initROS()
{
    sub_ecu = nh.subscribe("ecu", 500, &Can_app::ecu_cb, this);
    pub_feedback = nh.advertise<can_msgs::feedback>("feed_back", 500);
    dev_name = nh_private.param<std::string>("dev_name", dev_name, "can0");
}

void Can_app::run(const Can_app& a)
{
    LoggingListener* listener = new LoggingListener();
    listener->getPublisher(a);
    ros::spin();
}

/**
 * @description: ecu message dealing function 
 */
void Can_app::ecu_cb(const can_msgs::ecu& in_msg)
{
    ;
}
}