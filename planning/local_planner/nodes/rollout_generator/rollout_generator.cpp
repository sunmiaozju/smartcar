/*
 * @Description: local planner implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:46:57
 * @LastEditors: sunm
 * @LastEditTime: 2019-03-18 10:18:58
 */

#include <rollout_generator/rollout_generator.h>

using namespace UtilityNS;

namespace RolloutGeneratorNS {
RolloutGenerator::RolloutGenerator()
    : nh_private("~")
{
    initROS();
    currentPose_flag = false;
}

RolloutGenerator::~RolloutGenerator()
{
}

void RolloutGenerator::initROS()
{
    nh_private.param<double>("samplingTipMargin", PlanningParams.carTipMargin, 4);
    nh_private.param<double>("samplingOutMargin", PlanningParams.rollInMargin, 16);
    nh_private.param<double>("samplingSpeedFactor", PlanningParams.rollInSpeedFactor, 0.25);
    nh_private.param<bool>("enableHeadingSmoothing", PlanningParams.enableHeadingSmoothing, false);

    nh_private.param<double>("horizonDistance", PlanningParams.horizonDistance, 200);
    nh_private.param<double>("pathDensity", PlanningParams.pathDensity, 0.5);
    nh_private.param<bool>("enableLaneChange", PlanningParams.enableLaneChange, false);
    nh_private.param<double>("maxLocalPlanDistance", PlanningParams.microPlanDistance, 30);

    nh_private.param<double>("maxVelocity", PlanningParams.maxSpeed, 6.0);
    nh_private.param<double>("minVelocity", PlanningParams.minSpeed, 0.1);

    nh_private.param<double>("rollOutDensity", PlanningParams.rollOutDensity, 0.5);
    nh_private.param<int>("rollOutsNumber", PlanningParams.rollOutNumber, 6);

    nh_private.param<double>("smoothingDataWeight", PlanningParams.smoothingDataWeight, 0.45);
    nh_private.param<double>("smoothingSmoothWeight", PlanningParams.smoothingSmoothWeight, 0.4);
    nh_private.param<double>("smoothingToleranceError", PlanningParams.smoothingToleranceError, 0.05);

    nh_private.param<double>("speedProfileFactor", PlanningParams.speedProfileFactor, 1.2);

    pub_localTrajectories = nh.advertise<smartcar_msgs::LaneArray>("local_rollouts", 1);
    pub_localTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_rollouts_rviz", 1);
    pub_centralPathSection = nh.advertise<smartcar_msgs::Lane>("centralPathSection", 1);
    pub_testLane = nh.advertise<visualization_msgs::Marker>("test_lane", 1);

    sub_currentPose = nh.subscribe("/ndt/current_pose", 10, &RolloutGenerator::getCurrentPose_cb, this);
    sub_globalPlannerPath = nh.subscribe("/global_path", 1, &RolloutGenerator::getGlobalPlannerPath_cb, this);
    speed = 1;
}

/**
 * @description: 主循环函数
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (currentPose_flag && globalPaths.size() > 0) {
            globalPathSections.clear();
            for (size_t i = 0; i < globalPaths.size(); i++) {
                centralTrajectorySmoothed.clear();
                extractPartFromTrajectory(globalPaths[i], current_pose, PlanningParams.microPlanDistance,
                    PlanningParams.pathDensity, centralTrajectorySmoothed);
                globalPathSections.push_back(centralTrajectorySmoothed);
            }
            std::vector<UtilityNS::WayPoint> sampled_points;
            generateRunoffTrajectory(globalPathSections,
                current_pose,
                speed,
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
            for (size_t k = 0; k < rollOuts.size(); k++) {
                for (size_t m = 0; m < rollOuts[k].size(); m++) {
                    smartcar_msgs::Lane local_lane;
                    local_lane.waypoints.clear();
                    for (int h = 0; h < rollOuts[k][m].size(); h++) {
                        smartcar_msgs::Waypoint wp;
                        wp.pose.pose.position.x = rollOuts[k][m][h].pos.x;
                        wp.pose.pose.position.y = rollOuts[k][m][h].pos.y;
                        wp.pose.pose.position.z = rollOuts[k][m][h].pos.z;
                        wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityNS::cast_from_PI_to_PI_Angle(rollOuts[k][m][h].pos.yaw));
                        // wp.lane_id = rollOuts[k][m][h].laneId;
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
            for (size_t m = 0; m < globalPathSections.size(); m++) {
                for (size_t im = 0; im < globalPathSections[m].size(); im++) {
                    smartcar_msgs::Waypoint wp;
                    wp.pose.pose.position.x = globalPathSections[m][im].pos.x;
                    wp.pose.pose.position.y = globalPathSections[m][im].pos.y;
                    wp.pose.pose.position.z = globalPathSections[m][im].pos.z;
                    wp.yaw = globalPathSections[m][im].pos.yaw;
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
void RolloutGenerator::generateRunoffTrajectory(const std::vector<std::vector<UtilityNS::WayPoint>>& referencePaths,
    const UtilityNS::WayPoint& carPos,
    const double& speed,
    const double& microPlanDistance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& SmoothWeight,
    const double& SmoothTolerance,
    std::vector<std::vector<std::vector<UtilityNS::WayPoint>>>& rollOutsPaths,
    std::vector<UtilityNS::WayPoint>& sampledPoints_debug)
{

    if (referencePaths.size() == 0)
        return;
    if (microPlanDistance <= 0)
        return;
    rollOutsPaths.clear();
    sampledPoints_debug.clear(); //for visualization only

    for (unsigned int i = 0; i < referencePaths.size(); i++) {
        std::vector<std::vector<UtilityNS::WayPoint>> local_rollOutPaths;
        int s_index = 0, e_index = 0;
        std::vector<double> e_distances;

        if (referencePaths.at(i).size() > 0) {
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
void RolloutGenerator::calculateRollInTrajectories(const UtilityNS::WayPoint& carPos,
    const double& speed,
    const std::vector<UtilityNS::WayPoint>& originalCenter,
    int& start_index,
    int& end_index,
    std::vector<double>& end_laterals,
    std::vector<std::vector<UtilityNS::WayPoint>>& rollInPaths,
    const double& max_roll_distance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& SmoothWeight,
    const double& SmoothTolerance,
    std::vector<UtilityNS::WayPoint>& sampledPoints)
{
    UtilityNS::WayPoint p;

    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
    if (iLimitIndex >= originalCenter.size())
        iLimitIndex = originalCenter.size() - 1;

    //Get Closest Index
    UtilityNS::RelativeInfo info;
    UtilityNS::getRelativeInfo(originalCenter, carPos, info);

    double remaining_distance = 0;
    int close_index = info.iBack;
    for (unsigned int i = close_index; i < originalCenter.size() - 1; i++) {
        if (i > 0)
            remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    double initial_roll_in_distance = info.perp_distance; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

    std::vector<UtilityNS::WayPoint> RollOutStratPath;

    //calculate the starting index
    double d_limit = 0;
    unsigned int far_index = close_index;

    //calculate end index
    double start_distance = rollInSpeedFactor * speed + rollInMargin;
    if (start_distance > remaining_distance)
        start_distance = remaining_distance;

    d_limit = 0;
    for (unsigned int i = close_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);

        if (d_limit >= start_distance) {
            far_index = i;
            break;
        }
    }

    int centralTrajectoryIndex = rollOutNumber / 2;
    std::vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++) {
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
        end_distance_list.push_back(end_roll_in_distance);
    }

    start_index = close_index;
    end_index = far_index; // end_index是第二个阶段结尾的点坐标
    end_laterals = end_distance_list;

    //calculate the actual calculation starting index
    d_limit = 0;
    unsigned int smoothing_start_index = start_index;
    unsigned int smoothing_end_index = end_index;

    for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_start_index++; // 这个是一个阶段结尾的点下标
    }

    d_limit = 0;
    for (unsigned int i = end_index; i < originalCenter.size(); i++) {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_end_index++;
    }
    // printf("%s %d %d %d \n", "----------------", int(smoothing_start_index), int(end_index), int(originalCenter.size()));

    int nSteps = end_index - smoothing_start_index;

    std::vector<double> inc_list;
    rollInPaths.clear();
    std::vector<double> inc_list_inc;
    for (int i = 0; i < rollOutNumber + 1; i++) {
        double diff = end_laterals.at(i) - initial_roll_in_distance;
        inc_list.push_back(diff / (double)nSteps);
        rollInPaths.push_back(std::vector<UtilityNS::WayPoint>());
        inc_list_inc.push_back(0);
    }

    std::vector<std::vector<UtilityNS::WayPoint>> execluded_from_smoothing;
    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        execluded_from_smoothing.push_back(std::vector<UtilityNS::WayPoint>());

    //Insert First strait points within the tip of the car range
    for (unsigned int j = start_index; j < smoothing_start_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2);

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

    for (unsigned int j = smoothing_start_index; j < end_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2) - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2) - d * sin(p.pos.yaw + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }
    //Insert last strait points to make better smoothing
    for (unsigned int j = end_index; j < smoothing_end_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);
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
    for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++) {
        if (j > 0)
            d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

        if (d_limit > max_roll_distance)
            break;
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollInPaths.size(); i++) {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }

    for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
        smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }
}

/**
 * @description: 平滑生成的曲线 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::smoothPath(std::vector<UtilityNS::WayPoint>& path, double weight_data,
    double weight_smooth, double tolerance)
{
    if (path.size() <= 2)
        return;

    const std::vector<UtilityNS::WayPoint>& path_in = path;
    std::vector<UtilityNS::WayPoint> smoothPath_out = path_in;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path_in.size();

    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < size - 1; i++) {
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
void RolloutGenerator::visualInRviz(std::vector<UtilityNS::WayPoint> test_points)
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

    for (int i = 0; i < test_points.size(); i++) {
        test_marker.pose.position.x = test_points[i].pos.x;
        test_marker.pose.position.y = test_points[i].pos.y;
        test_marker.pose.position.z = 0;
        test_marker.id = i;
        test_markers.markers.push_back(test_marker);
    }

    pub_test.publish(test_markers);
}

void RolloutGenerator::trajectoryToMarkers(const std::vector<std::vector<std::vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "rollouts";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.05;
    lane_waypoint_marker.frame_locked = false;

    for (size_t i = 0; i < paths.size(); i++) {
        for (size_t k = 0; k < paths[i].size(); k++) {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = i * 10 + k;

            for (size_t m = 0; m < paths[i][k].size(); m++) {
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
/**
 * @description: 从全局路径截取最大局部规划距离 
 * @param {type} 
 * @return: 
 */
void RolloutGenerator::extractPartFromTrajectory(const std::vector<UtilityNS::WayPoint>& originalPath,
    const UtilityNS::WayPoint& currentPos,
    const double& minDistance,
    const double& waypointDensity,
    std::vector<UtilityNS::WayPoint>& extractedPath)
{
    if (originalPath.size() < 2)
        return;
    extractedPath.clear();
    int close_index = UtilityNS::getNextClosePointIndex(originalPath, currentPos);
    // printf("%d\n", close_index);

    double dis = 0;
    if (close_index >= originalPath.size() - 1)
        close_index = originalPath.size() - 2;

    for (int i = close_index; i >= 0; i--) {
        extractedPath.insert(extractedPath.begin(), originalPath[i]);
        if (i < originalPath.size())
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        // printf("%f\n", dis);
        if (dis > 2)
            break;
    }
    dis = 0;
    for (int i = close_index + 1; i < (int)originalPath.size(); i++) {
        extractedPath.push_back(originalPath[i]);
        if (i > 0)
            dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
        if (dis > minDistance)
            break;
    }
    if (extractedPath.size() < 2) {
        std::cout << std::endl
                  << "[loacal_planner_node] Extracted Rollout Path is too Small, Size = " << extractedPath.size() << std::endl;
        return;
    }
    // UtilityNS::visualLaneInRviz(extractedPath, pub_testLane);
    fixPathDensity(extractedPath, waypointDensity);
    calcAngleAndCost(extractedPath);
}

void RolloutGenerator::fixPathDensity(std::vector<UtilityNS::WayPoint>& path, const double& pathDensity)
{
    if (path.size() == 0 || pathDensity == 0)
        return;
    double dis = 0, ang = 0;
    double margin = pathDensity * 0.01;
    double remaining = 0;
    int nPoints = 0;
    std::vector<UtilityNS::WayPoint> fixedPath;
    fixedPath.push_back(path[0]);
    size_t start = 0, next = 1;
    while (next < path.size()) {
        dis += hypot(path[next].pos.x - path[next - 1].pos.x, path[next].pos.y - path[next - 1].pos.y) + remaining;
        ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);

        if (dis < pathDensity - margin) {
            next++;
            remaining = 0;
        } else if (dis > (pathDensity + margin)) {
            UtilityNS::WayPoint point_start = path[start];
            nPoints = dis / pathDensity;
            for (int j = 0; j < nPoints; j++) {
                point_start.pos.x = point_start.pos.x + pathDensity * cos(ang);
                point_start.pos.y = point_start.pos.y + pathDensity * sin(ang);
                fixedPath.push_back(point_start);
            }
            remaining = dis - nPoints * pathDensity;
            start++;
            path[start].pos = point_start.pos;
            dis = 0;
            next++;
        } else {
            dis = 0;
            remaining = 0;
            fixedPath.push_back(path[next]);
            next++;
            start = next - 1;
        }
    }
    path = fixedPath;
}

void RolloutGenerator::getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose = UtilityNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    currentPose_flag = true;
}

void RolloutGenerator::getGlobalPlannerPath_cb(const smartcar_msgs::LaneConstPtr& msg)
{
    if (msg->waypoints.size() > 0) {
        globalPaths.clear();
        std::vector<UtilityNS::WayPoint> single_path;
        msgLane2LocalLane(*msg, single_path);
        calcAngleAndCost(single_path);
        globalPaths.push_back(single_path);
    }
}

void RolloutGenerator::getNavGlobalPlannerPath_cb(const nav_msgs::PathConstPtr& msg)
{
    if (msg->poses.size() > 0) {
        globalPaths.clear();
        std::vector<UtilityNS::WayPoint> single_path;
        single_path.clear();
        for (size_t i = 0; i < msg->poses.size(); i++) {
            UtilityNS::WayPoint pp;
            pp.pos.x = msg->poses[i].pose.position.x;
            pp.pos.y = msg->poses[i].pose.position.y;
            pp.pos.z = msg->poses[i].pose.position.z;
            pp.pos.yaw = tf::getYaw(msg->poses[i].pose.orientation);
            single_path.push_back(pp);
        }
        globalPaths.push_back(single_path);
    }
}

void RolloutGenerator::msgLane2LocalLane(const smartcar_msgs::Lane& msg_path, std::vector<UtilityNS::WayPoint>& path)
{
    path.clear();
    for (size_t i = 0; i < msg_path.waypoints.size(); i++) {
        UtilityNS::WayPoint wp;
        wp.pos.x = msg_path.waypoints.at(i).pose.pose.position.x;
        wp.pos.y = msg_path.waypoints.at(i).pose.pose.position.y;
        wp.pos.z = msg_path.waypoints.at(i).pose.pose.position.z;
        wp.pos.yaw = tf::getYaw(msg_path.waypoints.at(i).pose.pose.orientation);
        wp.v = msg_path.waypoints.at(i).speed_limit;
        // wp.laneId = msg_path.waypoints.at(i).lane_id;
        path.push_back(wp);
    }
}

double RolloutGenerator::calcAngleAndCost(std::vector<UtilityNS::WayPoint>& path)
{
    if (path.size() < 2)
        return 0;
    if (path.size() == 2) {
        path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;
        path[1].pos.yaw = path[0].pos.yaw;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;

    for (int j = 1; j < path.size() - 1; j++) {
        path[j].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }

    int j = (int)path.size() - 1;
    path[j].pos.yaw = path[j - 1].pos.yaw;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

    return path[j].cost;
}
} // namespace RolloutGeneratorNS
