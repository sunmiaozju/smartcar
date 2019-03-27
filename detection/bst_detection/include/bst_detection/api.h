#ifndef _API_H_
#define _API_H_

#define IMAGE_WIDTH 768
#define IMAGE_HEIGHT 384
//#define IMAGE_WIDTH 1024
//#define IMAGE_HEIGHT 512
#define IMAGE_CHANNEL 3

#include <vector>
#include <opencv2/core/core.hpp>

struct infer_meta {
    //turn on/off lane
    int lane_on;
    //distance ratio adjustment parameter
    float dis_ratio;

    //accuracy threshold
    //current threshold is 0.2
    //increase this number will increase accuracy, but may also decrease true positive.
    //the range of accu_thres is 0.00 < accu_thres < 1.00.
    float accu_thres;
    //turn on/off debug mode
    int debug_mode;
/*
wait/drop frames
two modes are supported here
1. wait mode (when drop_on is 0)
For push in data with API bool detection_input_push(cv::Mat ori_frame);
If you are pushing data in,
while input buffer is full, inside API, it will wait until one frame space in input buffer is freed, then process data and return true.
if not full, then process data and return true.

For pop out data with API bool detection_output_pop(s_out_img_data *result);
If you are popping data out,
while input buffer is empty, inside API, it will wait until one frame in output buffer is filled, then return true.
if not empty, then process data and return true.
(Multi-thread is implemented, before detection_output_pop processes data for the first time, system will check if pre-requisite for detection_output_pop is ready,
 if not ready, detection_output_pop will also return false.)

2. drop frame mode (when drop_on is 1)
(currently the detection is not accurate when drop frame, will discuss later)
For push in data with API bool detection_input_push(cv::Mat ori_frame);
If you are pushing data in, while input buffer is full, API will return false immediately,
user has their own choice to push this copy of data again later or drop this frame,
if not full, process data and return true.

For pop out data with API bool detection_output_pop(s_out_img_data *result);
If you are popping data out, while input buffer is empty, API will return false immediately.
if not full, process data and return true.


When you use API detection_input_push and detection_input_push, please always check if the return value is true to see
if current push/pop is successful.
*/
    int drop_on;
};

struct apiLane 
{
  //point map of lane
  //The coordinate of top left of image is (0, 0),
  //the coordinate of bottom right of image is (image_width - 1, image_height - 1),
  //the coordinates of points are based on the coordinates of image.

  //std::vector<cv::Point>: vector of points for one lane
  //vector<std::vector... : vector of multiple lane

  //user can get the coordinates of point by doing this.
  /*
    for(auto lane: apiLane.lane_pts){
        for(auto point: lane){
            cout<< "(" << point.x << ", " <<point.y << ")" << " ";
        }
        cout << endl;
    }
  */
  std::vector<std::vector<cv::Point>> lane_pts;
};


struct apiBoundingBox
{
  //confidence score  
  float score;


  //The coordinate of top left of image is (0, 0),
  //the coordinate of bottom right of image is (image_width - 1, image_height - 1),
  //the coordinates of bounding box are based on the coordinates of image.

  //(xmin, ymin) is top left coordinate of the bounding box,
  //xmax = xmin + bbox_width
  //ymax = ymin + bbox_height
  //be noted here the largest x point on bbox is xmin + bbox_width -1
  //the largest y point on bbox is ymin + bbox_height -1
  //only applicable to class type 1:'car', 2:'truck', 3:'bus
  float ymin, xmin, ymax, xmax;


  //head or tail part of car that looks like "3d" coordinates
  //only applicable to class type 1:'car', 2:'truck', 3:'bus
  float ymin_3d, xmin_3d, ymax_3d, xmax_3d;

  
  //class type
  //1:'car', 2:'truck', 3:'bus', 4:'bicycle', 5:'pedestrian',
  //6:'cyclist', 7:'traffic_signs', 8:'info_signs', 9:'special', 10:'dontcare'  
  int cls;



  //object itself pose angle, range: 0-2PI (equivalent to 0-360 degree)
    /*
  From the quadrant coordinate below,
  0: left side is the rear of car, right is the front of car
     what camera see the side of car
  PI/2 (90): what camera see is the rear of car
  PI (180): left side is the front of car, right is the rear of car
     what camera see the side of car
  3PI/2 (270): what camera see is the front of car


  The rotate direction is 0 -> PI/2 ->PI -> 3PI/2 -> 0
  User can calculate the object degree by finding the position in between (0 -> PI/2 ->PI -> 3PI/2 -> 0)

                                                               rear
                                                                *,**,//,**.,
                                                            .#&%%%%@&%%%%&/
                                                            /&&&&@%&&&%,.
                                                    ,     .(((%/#(####(((%/(%       * &.
                                            *%            /,.  .........,..///              #
                                       &                  ****. ......   ,****                  #
                                    @                     ***,,,,,,,,,,,******                     #
                                  #                       @@&&&&&&&&&&&&&&&@@&                       @
                               .@                         @@@@&@@@@@@@@@&&@@@@                         #
                              #&                                                                         #
                                                                PI/2 (90)

                     ,,#&&%%&&&&&&&&%&&%/,,*,,                                             ./.,#&%&&&%%%%%%%%%#%%&&,,
                 *,&(//////%%%########%##,.,,%                                             %....#%%%#(######%%///////(&,/
           .,,..%&@((////,**,,,,,,,,,,,,,,,,((((%                                       %//#/ ..      ...,,,,***,,,//((@%%...,,.
   ,,./,,,**,,...   .... . .        ..,,.,,.  .**   PI(180)                      0     **,  .,......                    ,.,,..,,,./(.*
   , ..*@@@@@%,,,,,,**************,,/&@@@@&/,.***                                    * ****,#@@@@@@%*,*****************,*,%@@@@@&,. ,.
 &.*,,&@&%&(@%@#*******************&@&(&%@&***,/                                      /,***&@&%%@@@&@*****************.//@%&@@@&%@,,*..
 ,%&%%@&(%(##%@%%%%%##%%%%%&&&&&&&@@&(%&(&@%&@(                                        *(@&@%%####&@@&@@@@@@@@@&&&&&%%&%@&&%%%&&@%%&&*
       &&@@//&%                     @&&&(%&                                                   &%(@(%@                      @%(@(%&

                                                                                                            @
                               @                                3PI/2 (270)                                @
                                 @                            .                                           @
                                   &                         *#%%%%%%%%%%#,.                           #
                                     #                   %(.&%&&&&&&&&%##&%& (%                      @
                                                           /,*,,...........,*(.
                                           @              .,*.,,,,.(%,,,,,.*,*                 &
                                                #* &      ,*,,,,,*,,,,*,,,.***         .@.
                                                          /,*@@@@@@@@@@@@@@*.%
                                                          @@@#%%&&&&&&&&%%%@@@
                                                         .@@@%@@@@@@@@@@@@&@@@.
                                                                  front
  */
  float obj_deg;
  
  //object distance
  float obj_dist;
};


struct s_out_img_data
{
  //processed image with bbox and lane on it
  cv::Mat prs_img;
  //bounding boxes in current images
  std::vector<apiBoundingBox> output_bboxes;
  //coordinates of point map of lanes
  apiLane output_lanes;
};




//resource intialization
int obj_lane_detection_init(infer_meta input_if_meta);

/*
wait/drop frames
two modes are supported here
1. wait mode (when drop_on is 0)
For push in data with API bool detection_input_push(cv::Mat ori_frame);
If you are pushing data in,
while input buffer is full, inside API, it will wait until one frame space in input buffer is freed, then process data and return true.
if not full, then process data and return true.

For pop out data with API bool detection_output_pop(s_out_img_data *result);
If you are popping data out,
while input buffer is empty, inside API, it will wait until one frame in output buffer is filled, then return true.
if not empty, then process data and return true.
(Multi-thread is implemented, before detection_output_pop processes data for the first time, system will check if pre-requisite for detection_output_pop is ready,
 if not ready, detection_output_pop will also return false.)

2. drop frame mode (when drop_on is 1)
(currently the detection is not accurate when drop frame, will discuss later)
For push in data with API bool detection_input_push(cv::Mat ori_frame);
If you are pushing data in, while input buffer is full, API will return false immediately,
user has their own choice to push this copy of data again later or drop this frame,
if not full, process data and return true.

For pop out data with API bool detection_output_pop(s_out_img_data *result);
If you are popping data out, while input buffer is empty, API will return false immediately.
if not empty, process data and return true.


When you use API detection_input_push and detection_input_push, please always check if the return value is true to see
if current push/pop is successful.
*/

//push in data
//cv::Mat ori_frame is acquired by cap>>ori_frame
bool detection_input_push(cv::Mat ori_frame);

//pop out data
//if successful, data will be save to result
bool detection_output_pop(s_out_img_data *result);

#endif

