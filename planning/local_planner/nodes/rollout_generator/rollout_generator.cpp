/*
 * @Description: local planner implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:46:57
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 14:32:37
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

/**
 * @description: 主循环函数
 * @param {type} 
 * @return: 
 */
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
            generateRunoffTrajectory(globalPathSections,
                                     current_pose,
                                     VehicleStatus.speed,
                                     PlanningParams.microPlanDistance,
                                     PlanningParams.carTipMargin,
                                     PlanningParams.rollInMargin,
                                     PlanningParams.speedProfileFactor,
                                     PlanningParams.pathDensity,
                                     PlanningParams.rollOutDensity,
                                     PlanningParams.rollOutNumber,
                                     PlanningParams.smoothingDataWeight,
                                     PlanningParams.smoothingSmoothWeight,
                                     PlanningParams.smoothingToleranceError,
                                     rollOuts,
                                     sampled_points);

            // visualInRviz(test_points);
            // printf("%d, %d\n", int(rollOuts.size()), int(rollOuts[0].size()));
            // pub local rollouts
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

            // pub lcoal rollouts in rviz
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
/**
 * @description:生成候选局部规划路径rollouts 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::generateRunoffTrajectory(const std::vector<std::vector<PlannerHNS::WayPoint>> &referencePaths,
                                                const PlannerHNS::WayPoint &carPos,
                                                const double &speed,
                                                const double &microPlanDistance,
                                                const double &carTipMargin,
                                                const double &rollInMargin,
                                                const double &rollInSpeedFactor,
                                                const double &pathDensity,
                                                const double &rollOutDensity,
                                                const int &rollOutNumber,
                                                const double &SmoothDataWeight,
                                                const double &SmoothWeight,
                                                const double &SmoothTolerance,
                                                std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> &rollOutsPaths,
                                                std::vector<PlannerHNS::WayPoint> &sampledPoints_debug)
{

    if (referencePaths.size() == 0)
        return;
    if (microPlanDistance <= 0)
        return;
    rollOutsPaths.clear();
    sampledPoints_debug.clear(); //for visualization only

    for (unsigned int i = 0; i < referencePaths.size(); i++)
    {
        std::vector<std::vector<PlannerHNS::WayPoint>> local_rollOutPaths;
        int s_index = 0, e_index = 0;
        std::vector<double> e_distances;

        if (referencePaths.at(i).size() > 0)
        {
            calculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
                                        local_rollOutPaths, microPlanDistance, carTipMargin, rollInMargin,
                                        rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
                                        SmoothDataWeight, SmoothWeight, SmoothTolerance, sampledPoints_debug);
        }
        rollOutsPaths.push_back(local_rollOutPaths);
    }
}
/**
 * @description: 由中心轨迹的采样点计算候选rollouts的采样点并平滑
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::calculateRollInTrajectories(const PlannerHNS::WayPoint &carPos,
                                                   const double &speed,
                                                   const std::vector<PlannerHNS::WayPoint> &originalCenter,
                                                   int &start_index,
                                                   int &end_index,
                                                   std::vector<double> &end_laterals,
                                                   std::vector<std::vector<PlannerHNS::WayPoint>> &rollInPaths,
                                                   const double &max_roll_distance,
                                                   const double &carTipMargin,
                                                   const double &rollInMargin,
                                                   const double &rollInSpeedFactor,
                                                   const double &pathDensity,
                                                   const double &rollOutDensity,
                                                   const int &rollOutNumber,
                                                   const double &SmoothDataWeight,
                                                   const double &SmoothWeight,
                                                   const double &SmoothTolerance,
                                                   std::vector<PlannerHNS::WayPoint> &sampledPoints)
{
    PlannerHNS::WayPoint p;

    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
    if (iLimitIndex >= originalCenter.size())
        iLimitIndex = originalCenter.size() - 1;

    //Get Closest Index
    PlannerHNS::RelativeInfo info;
    UtilityNS::getRelativeInfo(originalCenter, carPos, info);

    double remaining_distance = 0;
    int close_index = info.iBack;
    for (unsigned int i = close_index; i < originalCenter.size() - 1; i++)
    {
        if (i > 0)
            remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    double initial_roll_in_distance = info.perp_distance; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

    std::vector<PlannerHNS::WayPoint> RollOutStratPath;

    //calculate the starting index
    double d_limit = 0;
    unsigned int far_index = close_index;

    //calculate end index
    double start_distance = rollInSpeedFactor * speed + rollInMargin;
    if (start_distance > remaining_distance)
        start_distance = remaining_distance;

    d_limit = 0;
    for (unsigned int i = close_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);

        if (d_limit >= start_distance)
        {
            far_index = i;
            break;
        }
    }

    int centralTrajectoryIndex = rollOutNumber / 2;
    std::vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++)
    {
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
        end_distance_list.push_back(end_roll_in_distance);
    }

    start_index = close_index;
    end_index = far_index;
    end_laterals = end_distance_list;

    //calculate the actual calculation starting index
    d_limit = 0;
    unsigned int smoothing_start_index = start_index;
    unsigned int smoothing_end_index = end_index;

    for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_start_index++;
    }

    d_limit = 0;
    for (unsigned int i = end_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_end_index++;
    }

    int nSteps = end_index - smoothing_start_index;

    std::vector<double> inc_list;
    rollInPaths.clear();
    std::vector<double> inc_list_inc;
    for (int i = 0; i < rollOutNumber + 1; i++)
    {
        double diff = end_laterals.at(i) - initial_roll_in_distance;
        inc_list.push_back(diff / (double)nSteps);
        rollInPaths.push_back(std::vector<PlannerHNS::WayPoint>());
        inc_list_inc.push_back(0);
    }

    std::vector<std::vector<PlannerHNS::WayPoint>> execluded_from_smoothing;
    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        execluded_from_smoothing.push_back(std::vector<PlannerHNS::WayPoint>());

    //Insert First strait points within the tip of the car range
    for (unsigned int j = start_index; j < smoothing_start_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            if (j < iLimitIndex)
                execluded_from_smoothing.at(i).push_back(p);
            else
                rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }


    for (unsigned int j = smoothing_start_index; j < end_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.a + M_PI_2) - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.a + M_PI_2) - d * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }
    //Insert last strait points to make better smoothing
    for (unsigned int j = end_index; j < smoothing_end_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.a + M_PI_2);
            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;
            rollInPaths.at(i).push_back(p);
            sampledPoints.push_back(p);
        }
    }

    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

    d_limit = 0;
    for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
    {
        if (j > 0)
            d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

        if (d_limit > max_roll_distance)
            break;

        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollInPaths.size(); i++)
        {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }

    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
    {
        smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }
}

/**
 * @description: 平滑生成的曲线 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::smoothPath(std::vector<PlannerHNS::WayPoint> &path, double weight_data,
                                  double weight_smooth, double tolerance)
{
    if (path.size() <= 2)
        return;

    const std::vector<PlannerHNS::WayPoint> &path_in = path;
    std::vector<PlannerHNS::WayPoint> smoothPath_out = path_in;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path_in.size();

    while (change >= tolerance)
    {
        change = 0.0;
        for (int i = 1; i < size - 1; i++)
        {
            xtemp = smoothPath_out[i].pos.x;
            ytemp = smoothPath_out[i].pos.y;

            smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
            smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

            smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
            smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

            change += fabs(xtemp - smoothPath_out[i].pos.x);
            change += fabs(ytemp - smoothPath_out[i].pos.y);
        }
        nIterations++;
    }
    path = smoothPath_out;
}

/**
 * @description: 测试函数：在RVIZ可视化一些测试点 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::visualInRviz(std::vector<PlannerHNS::WayPoint> test_points)
{
    pub_test = nh.advertise<visualization_msgs::MarkerArray>("test_points", 1);
    visualization_msgs::MarkerArray test_markers;
    test_markers.markers.clear();

    visualization_msgs::Marker test_marker;
    test_marker.header.frame_id = "map";
    test_marker.header.stamp = ros::Time();
    test_marker.ns = "test_points";
    test_marker.type = visualization_msgs::Marker::SPHERE;
    test_marker.action = visualization_msgs::Marker::ADD;
    test_marker.frame_locked = false;

    test_marker.scale.x = 0.1;
    test_marker.scale.y = 0.1;
    test_marker.scale.z = 0.1;

    test_marker.color.a = 1.0;
    test_marker.color.r = 1.0;
    test_marker.color.g = 0.0;
    test_marker.color.b = 0.0;

    for (int i = 0; i < test_points.size(); i++)
    {
        test_marker.pose.position.x = test_points[i].pos.x;
        test_marker.pose.position.y = test_points[i].pos.y;
        test_marker.pose.position.z = 0;
        test_marker.id = i;
        test_markers.markers.push_back(test_marker);
    }

    pub_test.publish(test_markers);
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
