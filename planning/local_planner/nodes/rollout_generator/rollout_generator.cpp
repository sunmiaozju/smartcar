/*
 * @Description: local planner implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:46:57
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-21 15:46:49
 */

#include <rollout_generator/rollout_generator.h>

using namespace UtilityNS;

namespace RolloutGeneratorNS
{
RolloutGenerator::RolloutGenerator()
{
    initROS();
    currentPose_flag = false;
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

    nh.param<double>("/rollout_generator_node/horizonDistance", PlanningParams.horizonDistance, 200);
    nh.param<double>("/rollout_generaror_node/pathDensity", PlanningParams.pathDensity, 0.5);
    nh.param<bool>("/rollout_generaror_node/enableLaneChange", PlanningParams.enableLaneChange, false);
    nh.param<double>("/rollout_generaror_node/maxLocalPlanDistance", PlanningParams.microPlanDistance, 50);

    nh.param<double>("/rollout_generaror_node/maxVelocity", PlanningParams.maxSpeed, 6.0);
    nh.param<double>("/rollout_generaror_node/minVelocity", PlanningParams.minSpeed, 0.1);

    nh.param<double>("/rollout_generaror_node/rollOutDensity", PlanningParams.rollOutDensity, 0.5);
    nh.param<int>("/rollout_generaror_node/rollOutNumber", PlanningParams.rollOutNumber, 6);

    nh.param<double>("/rollout_generaror_node/smoothingDataWeight", PlanningParams.smoothingDataWeight, 0.45);
    nh.param<double>("/rollout_generaror_node/smoothingSmoothWeight", PlanningParams.smoothingSmoothWeight, 0.4);
    nh.param<double>("/rollout_generaror_node/smoothingToleranceError", PlanningParams.smoothingToleranceError, 0.05);

    nh.param<double>("/rollout_generaror_node/speedProfileFactor", PlanningParams.speedProfileFactor, 1.2);

    pub_localTrajectories = nh.advertise<smartcar_msgs::LaneArray>("local_rollouts", 1);
    pub_localTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_rollouts_rviz", 1);
    pub_centralPathSection = nh.advertise<smartcar_msgs::Lane>("centralPathSection", 1);
    pub_testLane = nh.advertise<visualization_msgs::Marker>("test_lane", 1);

    sub_currentPose = nh.subscribe("/current_pose", 10, &RolloutGenerator::getCurrentPose_cb, this);
    sub_globalPlannerPath = nh.subscribe("/lane_array", 1, &RolloutGenerator::getGlobalPlannerPath_cb, this);
    VehicleStatus.speed = 1;
    current_pose.v = 1;
}

void RolloutGenerator::run()
{
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        if (currentPose_flag && globalPaths.size() > 0)
        {
            globalPathSections.clear();
            for (size_t i = 0; i < globalPaths.size(); i++)
            {
                centralTrajectorySmoothed.clear();
                extractPartFromTrajectory(globalPaths[i], current_pose, 50,
                                          PlanningParams.pathDensity, centralTrajectorySmoothed);
                globalPathSections.push_back(centralTrajectorySmoothed);
            }
            std::vector<PlannerHNS::WayPoint> sampled_points;
            Planner.GenerateRunoffTrajectory(globalPathSections,
                                             current_pose,
                                             PlanningParams.enableLaneChange,
                                             VehicleStatus.speed,
                                             PlanningParams.microPlanDistance,
                                             PlanningParams.maxSpeed,
                                             PlanningParams.minSpeed,
                                             PlanningParams.carTipMargin,
                                             PlanningParams.rollInMargin,
                                             PlanningParams.rollInSpeedFactor,
                                             PlanningParams.pathDensity,
                                             PlanningParams.rollOutDensity,
                                             PlanningParams.rollOutNumber,
                                             PlanningParams.smoothingDataWeight,
                                             PlanningParams.smoothingSmoothWeight,
                                             PlanningParams.smoothingToleranceError,
                                             PlanningParams.speedProfileFactor,
                                             PlanningParams.enableHeadingSmoothing,
                                             -1, -1,
                                             rollOuts,
                                             sampled_points);

            smartcar_msgs::LaneArray local_lanes;
            for (size_t k = 0; k < rollOuts.size(); k++)
            {
                for (size_t m = 0; m < rollOuts[k].size(); m++)
                {
                    smartcar_msgs::Lane local_lane;
                    predictTimeCostForTrajectory(rollOuts[k][m], current_pose, PlanningParams.minSpeed);
                    local_lane.waypoints.clear();
                    for (int h = 0; h < rollOuts[k][m].size(); h++)
                    {
                        smartcar_msgs::Waypoint wp;
                        wp.pose.pose.position.x = rollOuts[k][m][h].pos.x;
                        wp.pose.pose.position.y = rollOuts[k][m][h].pos.y;
                        wp.pose.pose.position.z = rollOuts[k][m][h].pos.z;
                        wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityNS::cast_from_PI_to_PI_Angle(rollOuts[k][m][h].pos.a));
                        wp.lane_id = rollOuts[k][m][h].laneId;
                        local_lane.waypoints.push_back(wp);
                    }
                    local_lane.lane_id = k;
                    local_lanes.lanes.push_back(local_lane);
                }
            }
            pub_localTrajectories.publish(local_lanes);

            visualization_msgs::MarkerArray marker_rollouts;
            trajectoryToMarkers(rollOuts, marker_rollouts);
            pub_localTrajectoriesRviz.publish(marker_rollouts);

            smartcar_msgs::Lane central_path_section;
            for (size_t m = 0; m < globalPathSections.size(); m++)
            {
                for (size_t im = 0; im < globalPathSections[m].size(); im++)
                {
                    smartcar_msgs::Waypoint wp;
                    wp.pose.pose.position.x = globalPathSections[m][im].pos.x;
                    wp.pose.pose.position.y = globalPathSections[m][im].pos.y;
                    wp.pose.pose.position.z = globalPathSections[m][im].pos.z;
                    wp.a = globalPathSections[m][im].pos.a;
                    central_path_section.waypoints.push_back(wp);
                }
            }
            pub_centralPathSection.publish(central_path_section);
        }

        loop_rate.sleep();
    }
}

void RolloutGenerator::trajectoryToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "rollouts";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.05;
    lane_waypoint_marker.frame_locked = false;

    for (size_t i = 0; i < paths.size(); i++)
    {
        for (size_t k = 0; k < paths[i].size(); k++)
        {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = i * 10 + k;

            for (size_t m = 0; m < paths[i][k].size(); m++)
            {
                geometry_msgs::Point wp;
                wp.x = paths[i][k][m].pos.x;
                wp.y = paths[i][k][m].pos.y;
                wp.z = paths[i][k][m].pos.z;
                lane_waypoint_marker.points.push_back(wp);
            }
            lane_waypoint_marker.color.a = 0.7;
            lane_waypoint_marker.color.r = 0.6;
            lane_waypoint_marker.color.g = 0.5;
            lane_waypoint_marker.color.b = 0.0;
            markerArray.markers.push_back(lane_waypoint_marker);
        }
    }
}

void RolloutGenerator::predictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint> &path, const PlannerHNS::WayPoint &p, const double &minSpeed)
{
    if (path.size() == 0)
        return;
    if (p.v == 0 || p.v < minSpeed)
        return;

    for (size_t i = 0; i < path.size(); i++)
        path[i].timeCost = -1;

    int closeFrontIndex = UtilityNS::getNextClosePointIndex(path, p);
    double total_distance = 0;

    path[closeFrontIndex].timeCost = 0;
    if (closeFrontIndex == 0)
        closeFrontIndex++;

    for (size_t k = closeFrontIndex; k < path.size(); k++)
    {
        total_distance += hypot(path[k].pos.x - path[k - 1].pos.x,
                                path[k].pos.y - path[k - 1].pos.y);
        path[k].timeCost = total_distance / p.v;
    }
}

void RolloutGenerator::extractPartFromTrajectory(const std::vector<PlannerHNS::WayPoint> &originalPath,
                                                 const PlannerHNS::WayPoint &currentPos,
                                                 const double &minDistance,
                                                 const double &waypointDensity,
                                                 std::vector<PlannerHNS::WayPoint> &extractedPath)
{
    if (originalPath.size() < 2)
        return;
    extractedPath.clear();
    int close_index = UtilityNS::getNextClosePointIndex(originalPath, currentPos);
    // printf("%d\n", close_index);

    double dis = 0;
    if (close_index >= originalPath.size() - 1)
        close_index = originalPath.size() - 2;

    for (int i = close_index; i >= 0; i--)
    {
        extractedPath.insert(extractedPath.begin(), originalPath[i]);
        if (i < originalPath.size())
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        // printf("%f\n", dis);
        if (dis > 2)
            break;
    }
    dis = 0;
    for (int i = close_index + 1; i < (int)originalPath.size(); i++)
    {
        extractedPath.push_back(originalPath[i]);
        if (i > 0)
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        if (dis > minDistance)
            break;
    }
    if (extractedPath.size() < 2)
    {
        std::cout << std::endl
                  << "[loacal_planner_node] Extracted Rollout Path is too Small, Size = " << extractedPath.size() << std::endl;
        return;
    }
    // UtilityNS::visualLaneInRviz(extractedPath, pub_testLane);
    fixPathDensity(extractedPath, waypointDensity);
    calcAngleAndCost(extractedPath);
}

void RolloutGenerator::fixPathDensity(std::vector<PlannerHNS::WayPoint> &path, const double &pathDensity)
{
    if (path.size() == 0 || pathDensity == 0)
        return;
    double dis = 0, ang = 0;
    double margin = pathDensity * 0.01;
    double remaining = 0;
    int nPoints = 0;
    std::vector<PlannerHNS::WayPoint> fixedPath;
    fixedPath.push_back(path[0]);
    size_t start = 0, next = 1;
    while (next < path.size())
    {
        dis += hypot(path[next].pos.x - path[next - 1].pos.x, path[next].pos.y - path[next - 1].pos.y) + remaining;
        ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);

        if (dis < pathDensity - margin)
        {
            next++;
            remaining = 0;
        }
        else if (dis > (pathDensity + margin))
        {
            PlannerHNS::WayPoint point_start = path[start];
            nPoints = dis / pathDensity;
            for (int j = 0; j < nPoints; j++)
            {
                point_start.pos.x = point_start.pos.x + pathDensity * cos(ang);
                point_start.pos.y = point_start.pos.y + pathDensity * sin(ang);
                fixedPath.push_back(point_start);
            }
            remaining = dis - nPoints * pathDensity;
            start++;
            path[start].pos = point_start.pos;
            dis = 0;
            next++;
        }
        else
        {
            dis = 0;
            remaining = 0;
            fixedPath.push_back(path[next]);
            next++;
            start = next - 1;
        }
    }
    path = fixedPath;
}

void RolloutGenerator::getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    currentPose_flag = true;
}

void RolloutGenerator::getGlobalPlannerPath_cb(const smartcar_msgs::LaneArrayConstPtr &msg)
{
    if (msg->lanes.size() > 0)
    {
        globalPaths.clear();
        std::vector<PlannerHNS::WayPoint> single_path;
        for (size_t i = 0; i < msg->lanes.size(); i++)
        {
            msgLane2LocalLane(msg->lanes[i], single_path);
            calcAngleAndCost(single_path);
            globalPaths.push_back(single_path);
        }
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
        path[0].pos.a = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;
        path[1].pos.a = path[0].pos.a;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.a = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;

    for (int j = 1; j < path.size() - 1; j++)
    {
        path[j].pos.a = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }

    int j = (int)path.size() - 1;
    path[j].pos.a = path[j - 1].pos.a;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

    return path[j].cost;
}
} // namespace RolloutGeneratorNS
