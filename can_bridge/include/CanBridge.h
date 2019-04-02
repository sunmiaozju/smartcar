/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:15:47
 * @LastEditTime: 2019-04-02 16:40:23
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "CanClient.h"
#include <can_msgs/battery.h>
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <iostream>
#include <ros/ros.h>

namespace CanBridge {

const std::string node_name = "can_bridge_node";

class Can_app {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_ecu;
    ros::Publisher pub_vehicle_status;
    ros::Publisher pub_battery_status;

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
    ros::Publisher pub_vehicle_status;
    ros::Publisher pub_battery_status;

public:
    void onReceiveMsg(RevMsg* msg) override
    {
        can_msgs::vehicle_status feedback;
        feedback.cur_speed = msg->speed();
        feedback.cur_steer = msg->wheelAngle();

        feedback.wheel_direction = msg->wheelDirection();
        feedback.shift_level = msg->shiftLevel();
        feedback.acc_level = msg->accLevel();
        feedback.brake_level = msg->brakeLevel();
        feedback.drive_mode = msg->driveMode();

        feedback.total_odometer = msg->totalOdometer();
        pub_vehicle_status.publish(feedback);
    }

    void onReceiveMsg(BatteryMsg* msg) override
    {
        can_msgs::battery batteryStatus;
        batteryStatus.voltage = msg->getVoltage();
        batteryStatus.ampere = msg->getAmpere();
        batteryStatus.capacity = msg->getBatteryCapacity();
        batteryStatus.BmuSys_status = msg->getBsuSysStatus();
        batteryStatus.Charge_status = msg->getChargeStatus();

        pub_battery_status.publish(batteryStatus);
    }

    void getPublisher(const Can_app& a)
    {
        this->pub_vehicle_status = a.pub_vehicle_status;
        this->pub_battery_status = a.pub_battery_status;
    }
};
}

#endif // !CAN_BRIDGE_H
