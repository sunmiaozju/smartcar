#include "hall_encoder_node.h"
#include <ros/ros.h>

namespace hall_encoder
{
HallEncoderNode::HallEncoderNode() : private_nh_("~"),
                                     pi(3.1415926),
                                     init_flag(false),
                                     pre_flag(false),
                                     pre_pre_flag(false),
                                     cur_flag(false)
{
    initROS();
}

HallEncoderNode::~HallEncoderNode()
{
}

void HallEncoderNode::initROS()
{
    // ROS sub
    sub = nh_.subscribe(sub_topic, 10, &HallEncoderNode::callbackFromCurrentVelocity, this);

    // ROS pub
    pub = nh_.advertise<geometry_msgs::TwistStamped>(pub_topic, 10);

    // ROS param
    private_nh_.param("magnet_num", magnet_num, int(1));
    private_nh_.param("pub_rate", pub_rate, int(10));                   // 10hz
    private_nh_.param("wheel_diameter", wheel_diameter, double(0.075)); // m
    private_nh_.param("sub_topic", sub_topic, std::string("/hall_sensor"));
    private_nh_.param("pub_topic", pub_topic, std::string("/hall_speed"));
    private_nh_.param("Q_Covariance", Q_Covariance, double(1.5));
    private_nh_.param("R_Covariance", R_Covariance, double(1.0));

    //Q_Covariance = std::pow(magnet_num * Q_Covariance / 3.0, 2);
    //R_Covariance = std::pow(magnet_num * R_Covariance / 3.0, 2);

    min_interval.fromSec(pub_rate);
}

void HallEncoderNode::callbackFromCurrentVelocity(const std_msgs::BoolConstPtr &msg)
{
    now = ros::Time::now();
    if (!init_flag)
    {
        pre_speed = 0;
        pre_P_Covariance = Q_Covariance / 10.;
        start = now;
        init_flag = true;
        return;
    }

    cur_flag = msg->data;
    if (pre_flag != cur_flag)
    {
        count++;
    }

    pre_pre_flag = pre_flag;
    pre_flag = cur_flag;
    if ((now - start) > min_interval)
    {
        // 因为遇到一次磁铁，霍尔传感器数值变化两次
        count /= 2;

        double raw_speed = count * pi * wheel_diameter / (magnet_num * (now - start).toSec());

        kalman_filter(raw_speed, pre_speed, pre_P_Covariance, cur_speed, cur_P_Covariance);

        if (std::fabs(pre_pre_speed + pre_speed + cur_speed) < 0.0001)
        {
            cur_speed = 0.0;
        }
        else
        {
            // Smooth filter, in the premise that velocity won't change drastically.
            cur_speed = (std::pow(pre_pre_speed, 2) + std::pow(pre_speed, 2) + std::pow(cur_speed, 2)) / (pre_pre_speed + pre_speed + cur_speed);
        }
        speed_msg.header.stamp = now;
        speed_msg.twist.linear.x = cur_speed;

        pub.publish(speed_msg);
        pre_pre_speed = pre_speed;
        pre_speed = cur_speed;
        pre_speed = cur_speed;
        start = ros::Time::now();
        count = 0;
    }
}

void HallEncoderNode::kalman_filter(const double &hall_measure_speed, const double &pre_speed, const double &pre_P_Covariance,
                                    double &cur_speed, double &cur_P_Covariance)
{
    /**
     * pre_speed是上一时刻的预测值  
     * pre_P_Covariance是上一时刻的预测值误差的协方差矩阵 
     * tmp_P_Covariance是这一时刻测量值误差的协方差矩阵的中间值 P_t|t-1
     * cur_P_Covariance是这一时刻测量值误差的协方差矩阵
     * Q_Covariance 是预测值的高斯噪声的协方差矩阵
     * K=卡尔曼增益
     * 本实例H=1 F=1
     * 符号定义参考：https://zhuanlan.zhihu.com/p/36745755
    */

    double tmp_P_Covariance = pre_P_Covariance + Q_Covariance;
    K = tmp_P_Covariance / (tmp_P_Covariance + R_Covariance); 
    cur_speed = pre_speed + K * (hall_measure_speed - pre_speed);
    cur_P_Covariance = tmp_P_Covariance - K * tmp_P_Covariance; 
}

} // namespace hall_encoder

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hall_encoder_node");
    hall_encoder::HallEncoderNode hall_node;

    ros::spin();
    return 0;
}
