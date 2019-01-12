#ifndef IMAGE_RECTIFIER_NODE_H
#define IMAGE_RECTIFIER_NODE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#define _NODE_NAME_ "image_rectifier"

namespace image_rectifier
{
class ImageRectifier
{
    ros::NodeHandle nh_private;
    ros::NodeHandle nh;

    ros::Subscriber sub_image_raw;
    ros::Subscriber sub_intrinsics;
    ros::Publisher pub_image_rectifier;

    cv::Size image_size;
    cv::Mat camera_instrinsics;
    cv::Mat distortion_coefficients;

    void ImageCallback(const sensor_msgs::Image &in_image_sensor);

    void IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message);

    void initROS();

  public:

    ImageRectifier();

    ~ImageRectifier();
};

} // namespace image_rectifier

#endif //IMAGE_RECTIFIER_NODE_H
