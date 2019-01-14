#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "smartcar_msgs/ProjectionMatrix.h"

#define _NODE_NAME_ "calibration_publisher"

static cv::Mat CameraExtrinsicMat;
static cv::Mat CameraMat;
static cv::Mat DistCoeff;
static cv::Size ImageSize;
static std::string DistModel;

static ros::Publisher pub_camera_info;
static ros::Publisher pub_projection_matrix;

static bool isRegister_tf;
static bool isPublish_extrinsic;
static bool isPublish_cameraInfo;

static std::string camera_id_str;
static std::string camera_frame;
static std::string target_frame;

static bool instrinsics_paresed;
static bool extrinsics_parsed;

static sensor_msgs::CameraInfo camera_info_msg;
static smartcar_msgs::ProjectionMatrix extrinsic_matrix_msg;

void tfRegistration(const cv::Mat &camExMat, const ros::Time &timeStamp)
{
    tf::Matrix3x3 rotation_mat;
    double roll = 0, pitch = 0, yaw = 0;
    tf::Quaternion quaternion;
    tf::Transform transform;
    static tf::TransformBroadcaster broadcaster;

    rotation_mat.setValue(camExMat.at<double>(0, 0), camExMat.at<double>(0, 1), camExMat.at<double>(0, 2),
                          camExMat.at<double>(1, 0), camExMat.at<double>(1, 1), camExMat.at<double>(1, 2),
                          camExMat.at<double>(2, 0), camExMat.at<double>(2, 1), camExMat.at<double>(2, 2));

    rotation_mat.getRPY(roll, pitch, yaw, 1);

    quaternion.setRPY(roll, pitch, yaw);

    transform.setOrigin(tf::Vector3(camExMat.at<double>(0, 3),
                                    camExMat.at<double>(1, 3),
                                    camExMat.at<double>(2, 3)));

    transform.setRotation(quaternion);

    broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp, target_frame, camera_frame));
}

void projectionMatrix_sender(const cv::Mat &projMat, const ros::Time &timeStamp)
{

    if (!extrinsics_parsed)
    {
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                extrinsic_matrix_msg.projection_matirx[row * 4 + col] = projMat.at<double>(row, col);
            }
        }
        extrinsics_parsed = true;
    }
    extrinsic_matrix_msg.header.stamp = timeStamp;
    extrinsic_matrix_msg.header.frame_id = camera_frame;
    pub_projection_matrix.publish(extrinsic_matrix_msg);
}

void cameraInfo_sender(const cv::Mat &camMat,
                       const cv::Mat &DistCoeff,
                       const cv::Size &imgSize,
                       const std::string &distModel,
                       const ros::Time &timeStamp)
{

    if (!instrinsics_paresed)
    {
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 3; col++)
            {
                camera_info_msg.K[row * 3 + col] = camMat.at<double>(row, col);
            }
        }

        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 4; col++)
            {

                if (col == 3)
                {
                    camera_info_msg.P[row * 4 + col] = 0.0f;
                }
                else
                {
                    camera_info_msg.P[row * 4 + col] = camMat.at<double>(row, col);
                }
            }
        }

        for (int row = 0; row < DistCoeff.rows; row++)
        {
            for (int col = 0; col < DistCoeff.cols; col++)
            {
                camera_info_msg.D.push_back(DistCoeff.at<double>(row, col));
            }
        }
        camera_info_msg.distortion_model = distModel;
        camera_info_msg.height = imgSize.height;
        camera_info_msg.width = imgSize.width;

        instrinsics_paresed = true;
    }

    camera_info_msg.header.stamp = timeStamp;
    camera_info_msg.header.frame_id = camera_frame;

    pub_camera_info.publish(camera_info_msg);
}

static void image_raw_callback(const sensor_msgs::Image &image_msg)
{
    ros::Time timeStampOfImage;
    timeStampOfImage.sec = image_msg.header.stamp.sec;
    timeStampOfImage.nsec = image_msg.header.stamp.nsec;

    if (isRegister_tf)
    {
        tfRegistration(CameraExtrinsicMat, timeStampOfImage);
    }

    if (isPublish_cameraInfo)
    {
        cameraInfo_sender(CameraMat, DistCoeff, ImageSize, DistModel, timeStampOfImage);
    }

    if (isPublish_extrinsic)
    {
        projectionMatrix_sender(CameraExtrinsicMat, timeStampOfImage);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string image_topic;
    std::string camera_info_topic;
    std::string projection_matrix_topic;

    nh_private.param<std::string>("image_topic", image_topic, "/cv_camera/image_raw");
    nh_private.param<std::string>("camera_info_topic", camera_info_topic, "/cv_camera/camera_info");
    nh_private.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");

    nh_private.param<bool>("register_lidar2camera_tf", isRegister_tf, true);
    nh_private.param<bool>("publish_extrinsics_mat", isPublish_extrinsic, true);
    nh_private.param<bool>("publish_camera_info", isPublish_cameraInfo, true);

    nh_private.param<std::string>("camera_frame", camera_frame, "camera");
    nh_private.param<std::string>("target_frame", target_frame, "velodyne");

    std::string calibration_file;
    nh_private.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty())
    {
        ROS_ERROR("[%s] missing calibration file path '%S'. ", _NODE_NAME_, calibration_file.c_str());
        ros::shutdown();
        return -1;
    }

    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_ERROR("[%s] connot open file calibration file %s", _NODE_NAME_, calibration_file.c_str());
        ros::shutdown();
        return -1;
    }

    fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;
    fs["CameraMat"] >> CameraMat;
    fs["DistCoeff"] >> DistCoeff;
    fs["ImageSize"] >> ImageSize;
    fs["DistModel"] >> DistModel;

    instrinsics_paresed = false;
    extrinsics_parsed = false;

    ros::Subscriber sub_image;

    sub_image = nh.subscribe(image_topic, 10, &image_raw_callback);

    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 10, true);

    pub_projection_matrix = nh.advertise<smartcar_msgs::ProjectionMatrix>(projection_matrix_topic, 10, true);

    ros::spin();

    return 0;
}
