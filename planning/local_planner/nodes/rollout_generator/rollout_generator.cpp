/*
 * @Description: local planner implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:46:57
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-05 21:09:24
 */

#include <rollout_generator/rollout_generator.h>

namespace RolloutGeneratorNS
{
RolloutGenerator::RolloutGenerator()
{
    initROS();
}

RolloutGenerator::~RolloutGenerator()
{
}

void RolloutGenerator::initROS()
{
    nh.param<double>("/rollout_generator_node/samplingTipMargin", PlanningParams.carTipMargin, 4);
    nh.param<double>("/rollout_generator_node/samplingOutMargin", PlanningParams.rollInMargin, 16);
    nh.param<double>("/rollout_generator_node/samplingSpeedFactor", PlanningParams.rollInSpeedFactor, 0.25);
    nh.param<bool>("/rollout_generator_node/enableHeadingSmoothing", PlanningParams.enableHeadingSmoothing, false);

    pub_localTrajectories = nh.advertise<smartcar_msgs::LaneArray>("local_trajectories", 1);
    pub_localTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_rviz", 1);

    sub_initialPose = nh.subscribe("/initial_pose", 1, &RolloutGenerator::getInitPose_cb, this);
    sub_currentPose = nh.subscribe("/current_pose", 10, &RolloutGenerator::getCurrentPose_cb, this);
    sub_globalPlannerPath = nh.subscribe("/lane_waypoints_array", 1, &RolloutGenerator::getGlobalPlannerPath_cb, this);
}

void RolloutGenerator::getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    init_pose = current_pose;
    currentPose_flag = true;
    initPose_flag = true;
}

void RolloutGenerator::getGlobalPlannerPath_cb(const smartcar_msgs::LaneArrayConstPtr &msg)
{
    if (msg->lanes.size() > 0)
    {
        bool OldGlobalPath = GlobalPaths.size() == msg->lanes.size();
    }
}

void RolloutGenerator::msgLane2LocalLane(const smartcar_msgs::Lane &msg_path, std::vector<PlannerHNS::WayPoint> &path)
{
    path.clear();
    for (size_t i = 0; i < msg_path.waypoints.size(); i++)
    {
        PlannerHNS::WayPoint wp;
        wp.pos.x = msg_path.waypoints.at(i).pose.pose.position.x;
        wp.pos.y = msg_path.waypoints.at(i).pose.pose.position.y;
        wp.pos.z = msg_path.waypoints.at(i).pose.pose.position.z;
        wp.pos.a = tf::getYaw(msg_path.waypoints.at(i).pose.pose.orientation);
        wp.v = msg_path.waypoints.at(i).twist.twist.linear.x;
        wp.laneId = msg_path.waypoints.at(i).lane_id;
        path.push_back(wp);
    }
}

double RolloutGenerator::calcAngleAndCost(std::vector<PlannerHNS::WayPoint> &path)
{
    if (path.size() < 2)
        return 0;
    if (path.size() == 2)
    {
        path[0].pos.a = dealNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;
        path[1].pos.a = path[0].pos.a;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.a = dealNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;

    for (int j = 1; j < path.size() - 1; j++)
    {
        path[j].pos.a = dealNegativeAngle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }

    int j = (int)path.size() - 1;
    path[j].pos.a = path[j - 1].pos.a;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

    return path[j].cost;
}

double RolloutGenerator::dealNegativeAngle(const double &ang)
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
} // namespace RolloutGeneratorNS
