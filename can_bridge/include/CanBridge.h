/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:15:47
 * @LastEditTime: 2019-03-25 23:07:42
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "CanClient.h"
#include <can_msgs/ecu.h>
#include <can_msgs/feedback.h>
#include <iostream>
#include <ros/ros.h>

namespace CanBridge {

const std::string node_name = "can_bridge_node";

class Can_app {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_ecu;
    ros::Publisher pub_feedback;

    std::string dev_name;

    CANClient* pCanClient;

    friend class LoggingListener;

public:
    Can_app();

    Can_app(Can_app&) = delete;

    ~Can_app() = default;

    void ecu_cb(const can_msgs::ecu& in_msg);

    void initROS();

    void run(const Can_app& a);
};

class LoggingListener : public CANListener {
private:
    ros::Publisher pub;

public:
    void onReceiveMsg(RevMsg* msg) override
    {
        can_msgs::feedback feedback;
        feedback.cur_speed = msg->Speed();
        feedback.cur_steer = msg->wheelAngle();
        feedback.shift_level = msg->shiftLevel();
        pub.publish(feedback);
    }

    void getPublisher(const Can_app& a)
    {
        this->pub = a.pub_feedback;
    }
};
}

#endif // !CAN_BRIDGE_H
