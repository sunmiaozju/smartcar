#pragma once
#ifndef _H_YUNLE_SENSOR_MSG_H_
#define _H_YUNLE_SENSOR_MSG_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include "api.h"

struct ImageCurrentLane
{
	std::vector<cv::Point> left;
	std::vector<cv::Point> right;
};

struct CurrentLane
{
	bool valid = false;		
	
	float curve = 0.0;		
	float width = 0.0;		
	float diff = 0.0;		

	float start = 0.0;		
	float end = 0.0;		

	bool is_l_virtual = false;	
	bool is_r_virtual = false;	

	float model_l[3];	
	float model_r[3];
};

struct ImageObect
{
	float score;
	int x;
	int y;
	int width;
	int height;
	//class type
    //1:'car', 2:'truck', 3:'bus', 4:'bicycle', 5:'pedestrian',
    //6:'cyclist', 7:'traffic_signs', 8:'info_signs', 9:'special', 10:'dontcare' 
	int category;
};

struct DetectedObject
{
	int id;
	float score;
	float x;
	float y;
	float angle;
	float v;
	int category;
};

//初始化
bool init(std::string intric_path, std::string extric_path);

//周期调用
bool Detect(cv::Mat src, 
			const double timestamp, 
			const float speed,
			cv::Mat & dst, 
			std::vector<ImageObect> & imgObjs,
			ImageCurrentLane & imgLane,
			std::vector<DetectedObject> & objs,
			CurrentLane & lane);

#endif
