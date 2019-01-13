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

static void image_raw_callback(const sensor_msgs::Image &image_msg)
{
    std::cout << "hello" << std::endl;
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

    // projection_matrix_topic = nh.advertise<smartcar_msgs::ProjectionMatrix>(projection_matrix_topic, 10, true);

    return 0;
}
