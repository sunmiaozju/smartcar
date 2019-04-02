/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 15:20:45
 * @LastEditTime: 2019-04-02 16:36:06
 */

#include <CanBridge.h>

namespace CanBridge {
Can_app::Can_app()
    : nh_private("~")
{
    initROS();
    pCanClient = CANClient::creatInstance(dev_name);
    if (pCanClient == NULL) {
        ROS_ERROR("open can device error: %s ", dev_name.data());
        exit(EXIT_FAILURE);
    }
}

/**
 * @description: init for ros params 
 */
void Can_app::initROS()
{
    sub_ecu = nh.subscribe("ecu", 500, &Can_app::ecu_cb, this);
    pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 500);
    pub_battery_status = nh.advertise<can_msgs::battery>("battery_status", 500);
    nh_private.param<std::string>("dev_name", dev_name, "can0");
}

void Can_app::run(const Can_app& a)
{
    LoggingListener* listener = new LoggingListener();
    listener->getPublisher(a);
    pCanClient->registerListener(listener);
    pCanClient->start();
    ros::Rate loop_rate(10);
    ros::spin();
    pCanClient->shutdown();
    CanBridge::CANClient::disposal(pCanClient);
    delete (listener);
    return;
}

/**
 * @description: ecu message dealing function 
 */
void Can_app::ecu_cb(const can_msgs::ecu& in_msg)
{
    static double pre_steer = 0;
    SendMsg sMsg;
    sMsg.setDriveMode(DriveMode::AUTO_MODE); // 驾驶模式
    sMsg.setSpeed(in_msg.motor); // 速度
    unsigned int shift = in_msg.shift;
    sMsg.setShiftLevel(ShiftLevel(shift)); //档位
    double steer = (in_msg.steer + pre_steer) / 2.0;
    sMsg.setWheelAngle(steer); //角度
    pre_steer = in_msg.steer;
    sMsg.setEBrake(false); //刹车
    if (pCanClient == NULL) {
        ROS_ERROR("can module error");
        return;
    }
    // sMsg.print();
    pCanClient->writeSendMsg(&sMsg);
}
}
