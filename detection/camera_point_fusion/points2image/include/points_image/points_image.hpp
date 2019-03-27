#ifndef _POINTS_IMAGE_H_
#define _POINTS_IMAGE_H_


#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "smartcar_msgs/PointsImage.h"

void resetMatrix();
smartcar_msgs::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2ConstPtr& pointclound2,
                                                const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
                                                const cv::Mat& distCoeff, const cv::Size& imageSize);

/*points2image::CameraExtrinsic
pointcloud2_to_3d_calibration(const sensor_msgs::PointCloud2ConstPtr& pointclound2,
            const cv::Mat& cameraExtrinsicMat);
*/
#endif /* _POINTS_IMAGE_H_ */
