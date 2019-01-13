#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define _NODE_NAME_ "calibration_publisher"

static cv::Mat CameraExtrinsicMat;
static cv::Mat CameraMat;
static cv::Mat DistCoeff;
static cv::Size ImageSize;
static cv::string DistModel;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("register_lidar2camera_tf", );


    return 0;
}
