#include "image_rectifier.h"

namespace image_rectifier
{
ImageRectifier::ImageRectifier() : nh_private("~")
{
    initROS();
}

void ImageRectifier::initROS()
{
    std::string image_raw_topic, camera_info_topic, image_retifier_topic;

    nh_private.param<std::string>("image_src", image_raw_topic, "/image_raw");
    nh_private.param<std::string>("camera_info_src", camera_info_topic, "/camera_info");
    nh_private.param<std::string>("image_rectifier_out", image_retifier_topic, "/image_rectified");

    sub_image_raw = nh.subscribe(image_raw_topic, 1, &ImageRectifier::ImageCallback, this);
    sub_intrinsics = nh.subscribe(camera_info_topic, 1, &ImageRectifier::IntrinsicsCallback, this);

    pub_image_rectifier = nh.advertise<sensor_msgs::Image>(image_retifier_topic, 1);
}

ImageRectifier::~ImageRectifier()
{
}

void ImageRectifier::ImageCallback(const sensor_msgs::Image &in_image_sensor)
{
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
    cv::Mat tmp_image = cv_image->image;
    cv::Mat image;

    if (camera_instrinsics.empty())
    {
        ROS_INFO("[%S] make sure camera_info is published", _NODE_NAME_);
        image = tmp_image;
    }
    else
    {
        cv::undistort(tmp_image, image, camera_instrinsics, distortion_coefficients);
    }

    cv_bridge::CvImage out_msg;
    out_msg.header = in_image_sensor.header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;

    pub_image_rectifier.publish(out_msg.toImageMsg());
}

void ImageRectifier::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
    image_size.height = in_message.height;
    image_size.width = in_message.width;

    camera_instrinsics = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {

        for (int col = 0; col < 3; col++)
        {
            camera_instrinsics.at<double>(row, col) = in_message.K[row * 3 + col];
        }
    }
    distortion_coefficients = cv::Mat(1, 5, CV_64F);
    
    for(int col = 0; col < 5; col++)
    {
        distortion_coefficients.at<double>(col) = in_message.D[col];
    }
    
}
} // namespace image_rectifier