//
// Created by sunm on 19-01-17.
//

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <iostream>
#include <set>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "monitor_msgs/ModuleHealth.h"
#include "monitor_msgs/ModuleCheck.h"
#include "yunle_sensor_msgs/Picture.h"
#include "yunle_sensor_msgs/LanePoint.h"
#include "yunle_sensor_msgs/DetectObject.h"
#include "yunle_sensor_msgs/DetectObjs.h"
#include "yunle_sensor_msgs/DetectLane.h"
#include "yunle_sensor_msgs/DetectLanes.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs.hpp>

#include "api.h"
#include "yunle_sensor_msg.h"

using namespace std;
using namespace cv;

class BstDetectionApp {
public:

	BstDetectionApp() {}

	int setup(ros::NodeHandle &nh, ros::NodeHandle &private_nh) {
		
		// calibration file path 
		std::string instric_path, exstric_path;
		if (!nh.getParam("instric_path", instric_path) || access(instric_path.c_str(), F_OK) == -1) {
			ROS_ERROR("Instric_path value %s is invalid", instric_path.c_str());
			return -1;
		}
		if (!nh.getParam("exstric_path", exstric_path) || access(exstric_path.c_str(), F_OK) == -1) {
			ROS_ERROR("exstric_path value %s is invalid", exstric_path.c_str());
			return -1;
		}
        ROS_INFO("instric_path is %s and exstric_path is %s", instric_path.c_str(), exstric_path.c_str());
        init(instric_path, exstric_path);
        
        // imgae size
		int crop_x, crop_y;
		nh.param<int>("crop_w", crop_x, 768);
		nh.param<int>("crop_h", crop_y, 384);
		ROS_INFO("crop_x is %d, crop_y is %d", crop_x, crop_y);
		croped_size.width = crop_x;
		croped_size.height = crop_y;

		// sub
		nodeCheckSub = nh.subscribe("nodeCheck", 10, &BstDetectionApp::nodecheck_callback, this);
		
		std::string image_topic;
		nh.param<std::string>("image_topic", image_topic, "front");
		pictureSub = nh.subscribe(image_topic, 1, &BstDetectionApp::image_callback, this);
		
		// pub
        nodeHealthPub = nh.advertise<monitor_msgs::ModuleHealth>("nodeHealth", 10);
		postPub = nh.advertise<sensor_msgs::Image>("postPub", 10);
        lanePub = nh.advertise<yunle_sensor_msgs::DetectLanes>("detectLane", 10);
        objsPub = nh.advertise<yunle_sensor_msgs::DetectObjs>("detectObjs", 10);
		
		// module health monitor
		status.moduleName = "Bst_detection_node";
		status.moduleType = monitor_msgs::ModuleHealth::MODULE_TYPE_HARDWARE;
		status.state = monitor_msgs::ModuleHealth::MODULE_OK;

		return 0;
	}

	void nodecheck_callback(const monitor_msgs::ModuleCheck &msg) {
		monitor_msgs::ModuleHealth report;
		report = status;
		report.requestId = msg.requestId;
		nodeHealthPub.publish(report);
	}
	
	void draw_myObjs(auto &imgObjs, cv::Mat &pic){
		
		for (size_t i = 0; i < imgObjs.size(); i++){
		
			std::cout << " i: " <<   i
					  << " xmax: " <<  imgObjs[i].xmin 
					  << " ymin: "<< imgObjs[i].ymin 
					  << " xmax:"<<  imgObjs[i].xmax
					  << " ymax: " << imgObjs[i].ymax
					  << " category: " <<  imgObjs[i].cls
					  << std::endl;
					  
			cv::circle(pic, 
					   cv::Point(int(imgObjs[i].xmin), int(imgObjs[i].ymin)), 
					   5, 
					   cv::Scalar(0, 0, 255), 
					   cv::FILLED, 
					   cv::LINE_8);
					   
			cv::circle(pic, 
					   cv::Point(int(imgObjs[i].xmax), int(imgObjs[i].ymax)),
					   5, 
					   cv::Scalar(0, 0, 255), 
					   cv::FILLED, 
					   cv::LINE_8);
							  
			cv::rectangle(pic, 
						  cv::Point(int(imgObjs[i].xmin), int(imgObjs[i].ymin)), 
						  cv::Point(int(imgObjs[i].xmax), int(imgObjs[i].ymax)), 
						  cv::Scalar(0, 0, 255), 
						  2,
						  cv::LINE_8);
		}
	}
	
    void draw_mylane(auto &lanes, cv::Mat &pic){
		
	    for (size_t i = 0; i < lanes.lane_pts.size(); i++)
		    for(size_t j = 0; j < lanes.lane_pts[i].size(); j++){
				cv::circle(pic, 
						   cv::Point(lanes.lane_pts[i][j].x, lanes.lane_pts[i][j].y), 
						   1, 
						   cv::Scalar(0, 0, 255), 
						   cv::FILLED, 
						   cv::LINE_8);
			} 
    }

	void process(cv::Mat &crop, double stamp) {
		detection_input_push(crop);
		if(true == detection_output_pop(&result)){
			
			ROS_INFO(" bbox num: %d", result.output_bboxes.size());
			ROS_INFO(" lane num: %d" , result.output_lanes.lane_pts.size());
			
            // pub detect lane
			if (result.output_lanes.lane_pts.size() > 0 ){
			    yunle_sensor_msgs::DetectLanes msg_lanes;
			    for (size_t i = 0; i < result.output_lanes.lane_pts.size() && i < 2; i++){
                    yunle_sensor_msgs::DetectLane msg_lane;
                    for (size_t j = 0; j < result.output_lanes.lane_pts[i].size(); j++){
                        yunle_sensor_msgs::LanePoint lane_point;
                        lane_point.x = result.output_lanes.lane_pts[i][j].x;
                        lane_point.y = result.output_lanes.lane_pts[i][j].y;
                        msg_lane.lane.push_back(lane_point);
                    }
				    msg_lanes.lanes.push_back(msg_lane);
			    }
			    lanePub.publish(msg_lanes);
				// draw_mylane(result.output_lanes, result.prs_img);
            }
			
			// pub detect objects
			yunle_sensor_msgs::DetectObjs msg_objs;
			for (size_t i = 0; i < result.output_bboxes.size(); i++){
			    yunle_sensor_msgs::DetectObject msg_obj;
			    msg_obj.score = result.output_bboxes[i].score;
			    msg_obj.category = result.output_bboxes[i].cls;
			    msg_obj.xmin = result.output_bboxes[i].xmin;
			    msg_obj.ymin = result.output_bboxes[i].ymin;
			    msg_obj.xmax = result.output_bboxes[i].xmax;
			    msg_obj.ymax = result.output_bboxes[i].ymax;
			    msg_obj.xmin_3d = result.output_bboxes[i].xmin_3d;
			    msg_obj.ymin_3d = result.output_bboxes[i].ymin_3d;
			    msg_obj.xmax_3d = result.output_bboxes[i].xmax_3d;
			    msg_obj.xmax_3d = result.output_bboxes[i].xmax_3d;
			    msg_obj.obj_deg = result.output_bboxes[i].obj_deg;
			    msg_obj.obj_dist = result.output_bboxes[i].obj_dist;
			    msg_objs.objs.push_back(msg_obj);	
			}
			objsPub.publish(msg_objs);
            // draw_myObjs(result.output_bboxes, result.prs_img);
            
            // pub processed img
			sensor_msgs::ImagePtr imagePtr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result.prs_img).toImageMsg();
			postPub.publish(imagePtr);
            
            
		}else {
			ROS_WARN("Not detected!");
		}	
	}

	void image_callback(const sensor_msgs::ImageConstPtr & msg) {
		cv_bridge::CvImageConstPtr cv_ptr;
		try {
			cv_ptr=cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		} catch(cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		origin = cv_ptr->image;   
	    // origin = cv::imread("/home/nvidia/catkin_ws/src/bst_control/pics/lane2.jpg");
		origin_size.width = origin.cols;
		origin_size.height = origin.rows;
		cv::resize(origin, crop, croped_size);
		double stamp = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
		process(crop, stamp);
	}


private:
	cv::Mat origin, crop, post;
	cv::Size croped_size, origin_size;

	ros::Subscriber nodeCheckSub;
	ros::Publisher nodeHealthPub;

	ros::Subscriber pictureSub;
	
	ros::Publisher postPub;
	ros::Publisher lanePub;
	ros::Publisher objsPub;

	monitor_msgs::ModuleHealth status;
	
	s_out_img_data result; 
};


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "detection_node");
	BstDetectionApp app;
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	app.setup(nh, private_nh);
	ros::spin();
	return 0;
}
