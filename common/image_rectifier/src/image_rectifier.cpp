#include "image_rectifier.h"

namespace image_rectifier
{
ImageRectifier::ImageRectifier() : nh_private("~")
{
    initROS();
    Loadintrinsics(calibration_file);
}

void ImageRectifier::initROS()
{
    nh_private.param<std::string>("image_src", image_raw_topic, "/image_raw");
    nh_private.param<std::string>("camera_info_src", camera_info_topic, "/camera_info");
    nh_private.param<std::string>("image_rectifier_out", image_retifier_topic, "/image_rectified");
    nh_private.param<std::string>("calibration_file", calibration_file, "");

    sub_image_raw = nh.subscribe(image_raw_topic, 1, &ImageRectifier::ImageCallback, this);
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
        ROS_INFO("[%s] make sure camera_info is published", _NODE_NAME_);
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

void ImageRectifier::Loadintrinsics(const std::string &calibration_file_path)
{

    if (calibration_file_path.empty())
    {
        ROS_ERROR("[%s] missing calibration file path", _NODE_NAME_);
        ros::shutdown();
    }

    cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        ROS_ERROR("[%s] cannot open calibration file %s", _NODE_NAME_, calibration_file_path.c_str());
        ros::shutdown();
    }

    camera_instrinsics = cv::Mat(3, 3, CV_64F);
    distortion_coefficients = cv::Mat(1, 5, CV_64F);

    cv::Mat dis_tmp;
    fs["CameraMat"] >> camera_instrinsics;
    fs["DistCoeff"] >> dis_tmp;
    fs["ImageSize"] >> image_size;

    for (int col = 0; col < 5; col++)
    {
        distortion_coefficients.at<double>(col) = dis_tmp.at<double>(col);
    }
}
} // namespace image_rectifier