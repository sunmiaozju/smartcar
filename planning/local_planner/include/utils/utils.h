/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-18 17:07:52
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-21 10:09:22
 */
#ifndef LOCAL_PLANNER_UTILS_H
#define LOCAL_PLANNER_UTILS_H

#include <op_planner/RoadNetwork.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

namespace UtilityNS
{

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI

int getNextClosePointIndex(const std::vector<PlannerHNS::WayPoint> &trajectory,
                           const PlannerHNS::WayPoint &curr_pos,
                           const int &prevIndex = 0);
double calDiffBetweenTwoAngle(const double &a1, const double &a2);
double cast_from_0_to_2PI_Angle(const double &ang);
double cast_from_PI_to_PI_Angle(const double &ang);
double diffBetweenTwoAngle(const double &a1, const double &a2);
void visualLaneInRviz(const std::vector<PlannerHNS::WayPoint> &lane, ros::Publisher pub_testLane);
} // namespace UtilityNS

#endif //LOCAL_PLANNER_UTILS_H
