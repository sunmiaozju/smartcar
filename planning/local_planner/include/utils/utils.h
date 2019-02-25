/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-18 17:07:52
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 15:37:14
 */
#ifndef LOCAL_PLANNER_UTILS_H
#define LOCAL_PLANNER_UTILS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <utils/common.h>

namespace UtilityNS
{

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI

int getNextClosePointIndex(const std::vector<UtilityNS::WayPoint> &trajectory,
                           const UtilityNS::WayPoint &curr_pos,
                           const int &prevIndex = 0);

double calDiffBetweenTwoAngle(const double &a1, const double &a2);

double cast_from_0_to_2PI_Angle(const double &ang);

double cast_from_PI_to_PI_Angle(const double &ang);

double diffBetweenTwoAngle(const double &a1, const double &a2);

void visualLaneInRviz(const std::vector<UtilityNS::WayPoint> &lane, ros::Publisher pub_testLane);

bool getRelativeInfo(const std::vector<UtilityNS::WayPoint> &trajectory,
                     const UtilityNS::WayPoint &p,
                     UtilityNS::RelativeInfo &info);

} // namespace UtilityNS

#endif //LOCAL_PLANNER_UTILS_H
