#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>

namespace hall_encoder
{
class HallEncoderNode
{
  public:
    HallEncoderNode();

    ~HallEncoderNode();

  private:
    const double pi;
    bool init_flag;

    // hall sensor 
    bool pre_pre_flag;
    bool pre_flag;
    bool cur_flag;
    int count;
    int magnet_num;
    double wheel_diameter;

    // kalman param
    double Q_Covariance;
    double R_Covariance;
    double pre_P_Covariance, cur_P_Covariance; 
    double cur_speed, pre_speed, pre_pre_speed;
    double K;

    // ros param
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::TwistStamped speed_msg;
    ros::Duration min_interval;
    ros::Time start;
    ros::Time now;
    int pub_rate;

    void kalman_filter(const double &hall_measure_speed, const double &pre_speed, const double &pre_P_Covariance,
                                    double &cur_speed, double &cur_P_Covariance);

    void callbackFromCurrentVelocity(const std_msgs::Int32ConstPtr &msg);

    void initROS();
};

} // namespace hall_encoder
