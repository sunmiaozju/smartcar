/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-18 17:07:29
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-21 11:01:12
 */

#include <utils/utils.h>
#include <float.h>

namespace UtilityNS
{
/**
 * @description: 获取轨迹上距离当前位置最近的轨迹点（前方） 
 * @param {type} 
 * @return: 
 */
int getNextClosePointIndex(const std::vector<PlannerHNS::WayPoint> &trajectory,
                           const PlannerHNS::WayPoint &curr_pos,
                           const int &prevIndex)
{
    if (trajectory.size() < 2 || prevIndex < 0)
        return 0;
    double dis = 0, min_dis = DBL_MAX;
    int min_index = prevIndex;

    for (int i = prevIndex; i < trajectory.size(); i++)
    {
        dis = distance2points_pow(trajectory[i].pos, curr_pos.pos);

        if (dis < min_dis)
        {
            min_index = i;
            min_dis = dis;
        }
    }
    // printf("index %d min_dis %f\n", min_index, min_dis);

    if (min_index < (int)trajectory.size() - 2)
    {
        PlannerHNS::GPSPoint closest, next;
        closest = trajectory[min_index].pos;
        next = trajectory[min_index + 1].pos;
        PlannerHNS::GPSPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
        double length1 = calLength(v_1);
        PlannerHNS::GPSPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
        double length2 = calLength(v_2);
        double angle = cast_from_0_to_2PI_Angle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
        if (angle <= M_PI_2)
            min_index = min_index + 1;
    }
    return min_index;
}
/**
 * @description: 显示一条车道线（测试） 
 * @param {type} 
 * @return: 
 */
void visualLaneInRviz(const std::vector<PlannerHNS::WayPoint> &lane, ros::Publisher pub_testLane)
{
    visualization_msgs::Marker lane_marker;

    lane_marker.header.frame_id = "map";
    lane_marker.header.stamp = ros::Time();
    lane_marker.ns = "test_lane";
    lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_marker.action = visualization_msgs::Marker::ADD;
    lane_marker.frame_locked = false;

    lane_marker.scale.x = 0.1;
    lane_marker.frame_locked = false;

    lane_marker.points.clear();
    for (size_t k = 0; k < lane.size(); k++)
    {
        geometry_msgs::Point wp;
        wp.x = lane[k].pos.x;
        wp.y = lane[k].pos.y;
        wp.z = lane[k].pos.z;
        lane_marker.points.push_back(wp);
    }

    lane_marker.color.b = 0;
    lane_marker.color.g = 0;
    lane_marker.color.r = 1;
    lane_marker.color.a = 1;

    pub_testLane.publish(lane_marker);
}

double calDiffBetweenTwoAngle(const double &a1, const double &a2)
{
    double diff = a1 - a2;
    if (diff < 0)
        diff = a2 - a1;
    if (diff > M_PI)
        diff = 2.0 * M_PI - diff;
    return diff;
}
/**
 * @description:将当前角度转换到0～2pi 
 * @param {type} 
 * @return: 
 */
double cast_from_0_to_2PI_Angle(const double &ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI)
    {
        angle = fmod(ang, 2.0 * M_PI);
    }
    else
        angle = ang;

    if (angle < 0)
    {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}
/**
 * @description:将当前角度转换到-pi~pi 
 * @param {type} 
 * @return: 
 */
double cast_from_PI_to_PI_Angle(const double &ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI)
    {
        angle = fmod(ang, 2.0 * M_PI);
    }
    else
        angle = ang;

    if (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

double diffBetweenTwoAngle(const double &a1, const double &a2)
{
    double diff = a1 - a2;
    if (diff < 0)
        diff = -diff;
    if (diff > M_PI)
        diff = 2 * M_PI - diff;
    return diff;
}
} // namespace UtilityNS