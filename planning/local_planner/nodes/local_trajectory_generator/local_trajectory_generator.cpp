/*
 * @Description: collision avoid and local trajectory generator implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-15 14:54:09
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 13:44:37
 */
#include <local_trajectory_generator/local_trajectory_generator.h>

namespace LocalTrajectoryGeneratorNS
{
LocalTrajectoryGenerator::LocalTrajectoryGenerator()
{
    initROS();
    currentPose_flag = false;
    pre_best_index = -1;

    // for test
    PlannerHNS::DetectedObject obj;
    obj.l = 1;
    obj.w = 1;
    obj.h = 1;

    obj.center.pos.x = 20;
    obj.center.pos.y = 27;
    obj.center.pos.z = 0;
    obj.center.v = 0;

    PlannerHNS::GPSPoint p;
    obj.contour.clear();

    p.x = obj.center.pos.x - 0.5;
    p.y = obj.center.pos.y + 0.5;
    p.z = 0;
    obj.contour.push_back(p);
    p.x = obj.center.pos.x + 0.5;
    p.y = obj.center.pos.y + 0.5;
    p.z = 0;
    obj.contour.push_back(p);
    p.x = obj.center.pos.x - 0.5;
    p.y = obj.center.pos.y - 0.5;
    p.z = 0;
    obj.contour.push_back(p);
    p.x = obj.center.pos.x + 0.5;
    p.y = obj.center.pos.y - 0.5;
    p.z = 0;
    obj.contour.push_back(p);
    detect_objs.push_back(obj);
}
void LocalTrajectoryGenerator::initROS()
{
    sub_currentPose = nh.subscribe("current_pose", 1, &LocalTrajectoryGenerator::getCurrentPose_cb, this);
    sub_detectedObjects = nh.subscribe("detected_objects", 1, &LocalTrajectoryGenerator::getDetectedObjects_cb, this);
    sub_localRollouts = nh.subscribe("local_rollouts", 1, &LocalTrajectoryGenerator::getRolloutPaths_cb, this);
    sub_centralPath = nh.subscribe("centralPathSection", 1, &LocalTrajectoryGenerator::getCentralPathSection_cb, this);

    pub_collisionObjsRviz = nh.advertise<visualization_msgs::MarkerArray>("collision_objs_rviz", 1);
    pub_LocalWeightedTrajectoryRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_rviz", 1);
    pub_LocalWeightedTrajectory = nh.advertise<smartcar_msgs::LaneArray>("local_trajectories", 1);
    pub_TrajectoryCost = nh.advertise<smartcar_msgs::Lane>("local_trajectory_cost", 1);
    pub_testLane = nh.advertise<visualization_msgs::Marker>("test_lane", 1);

    nh.param<double>("/local_trajectory_generator/horizonDistance", plan_params.horizonDistance, 200);
    nh.param<int>("/local_trajectory_generator/rollOutNumber", plan_params.rollOutNumber, 6);
    nh.param<double>("/local_trajectory_generator/rollOutDensity", plan_params.rollOutDensity, 0.5);
    nh.param<double>("/local_trajectory_generator/horizontalSafetyDistancel", plan_params.horizontalSafetyDistancel, 1);
    nh.param<double>("/local_trajectory_generator/verticalSafetyDistance", plan_params.verticalSafetyDistance, 1);
    nh.param<double>("/local_trajectory_generator/minFollowingDistance", plan_params.minFollowingDistance, 50);
    nh.param<double>("/local_trajectory_generator/lateralSkipDistance", plan_params.lateralSkipDistance, 6.0);

    nh.param<double>("/local_trajectory_generator/width", car_info.width, 0.5);
    nh.param<double>("/local_trajectory_generator/length", car_info.length, 0.68);
    nh.param<double>("/local_trajectory_generator/wheel_base", car_info.wheel_base, 0.48);
}
LocalTrajectoryGenerator::~LocalTrajectoryGenerator()
{
}
/**
 * @description: 获取障碍物数组
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::getDetectedObjects_cb(const smartcar_msgs::DetectedObjectArrayConstPtr &msg)
{
    detect_objs.clear();
    for (int i = 0; i < msg->objects.size(); i++)
    {
        PlannerHNS::DetectedObject obj;
        obj.id = msg->objects[i].id;
        obj.label = msg->objects[i].label;
        obj.l = msg->objects[i].dimensions.x;
        obj.w = msg->objects[i].dimensions.y;
        obj.h = msg->objects[i].dimensions.z;

        obj.center.pos.x = msg->objects[i].pose.position.x;
        obj.center.pos.y = msg->objects[i].pose.position.y;
        obj.center.pos.z = msg->objects[i].pose.position.z;
        obj.center.pos.a = tf::getYaw(msg->objects[i].pose.orientation);
        obj.center.v = msg->objects[i].velocity.linear.x;

        PlannerHNS::GPSPoint p;
        obj.contour.clear();
        for (int k = 0; k < msg->objects[i].convex_hull.polygon.points.size(); k++)
        {
            p.x = msg->objects[i].convex_hull.polygon.points[k].x;
            p.y = msg->objects[i].convex_hull.polygon.points[k].y;
            p.z = msg->objects[i].convex_hull.polygon.points[k].z;
            obj.contour.push_back(p);
        }
        detect_objs.push_back(obj);
    }
}
/**
 * @description: 获取备选跟踪轨迹
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::getRolloutPaths_cb(const smartcar_msgs::LaneArrayConstPtr &msg)
{
    if (msg->lanes.size() > 0)
    {
        generated_rollouts.clear();
        for (size_t i = 0; i < msg->lanes.size(); i++)
        {
            std::vector<PlannerHNS::WayPoint> rollout_singleLane;
            for (size_t k = 0; k < msg->lanes[i].waypoints.size(); k++)
            {
                PlannerHNS::WayPoint wp;
                wp.pos.x = msg->lanes[i].waypoints[k].pose.pose.position.x;
                wp.pos.y = msg->lanes[i].waypoints[k].pose.pose.position.y;
                wp.pos.z = msg->lanes[i].waypoints[k].pose.pose.position.z;
                wp.pos.a = tf::getYaw(msg->lanes[i].waypoints[k].pose.pose.orientation);
                rollout_singleLane.push_back(wp);
            }
            generated_rollouts.push_back(rollout_singleLane);
        }
    }
}
/**
 * @description: 获取当前跟踪的全局轨迹的片段
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::getCentralPathSection_cb(const smartcar_msgs::LaneConstPtr &msg)
{
    centralPathSection.clear();
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
        PlannerHNS::WayPoint p;
        p.pos.x = msg->waypoints[i].pose.pose.position.x;
        p.pos.y = msg->waypoints[i].pose.pose.position.y;
        p.pos.z = msg->waypoints[i].pose.pose.position.z;
        p.pos.a = msg->waypoints[i].a;
        centralPathSection.push_back(p);
    }
}
/**
 * @description: 获取当前位置
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    currentPose_flag = true;
}
/**
 * @description: 主循环函数
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        PlannerHNS::TrajectoryCost best_trajectory;
        if (currentPose_flag && centralPathSection.size() > 0)
        {
            best_trajectory = trajectory_evaluator_static(generated_rollouts, centralPathSection, current_pose, plan_params, car_info, detect_objs);
            best_index = best_trajectory.index;
            smartcar_msgs::Lane best_local_lane;
            best_local_lane.closest_obj_dis = best_trajectory.closest_obj_distance;
            best_local_lane.is_blocked = best_trajectory.bBlocked;
            best_local_lane.cost = best_trajectory.cost;
            best_local_lane.best_rollout_index = best_trajectory.index;
            pub_TrajectoryCost.publish(best_local_lane);

            smartcar_msgs::LaneArray local_trajectories_msg;
            for (int m = 0; m < generated_rollouts.size(); m++)
            {
                smartcar_msgs::Lane lane_msg;
                lane_msg.transition_cost = trajectoryCosts[m].transition_cost;
                lane_msg.center_cost = trajectoryCosts[m].priority_cost;
                lane_msg.lateral_cost = trajectoryCosts[m].lateral_cost;
                lane_msg.long_cost = trajectoryCosts[m].longitudinal_cost;
                local_trajectories_msg.lanes.push_back(lane_msg);
            }
            pub_LocalWeightedTrajectory.publish(local_trajectories_msg);

            visualInRviz();
        }
    }
}
/**
 * @description: 输出结果可视化 
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::visualInRviz()
{
    if (trajectoryCosts.size() > 0)
    {
        visualization_msgs::MarkerArray allRollouts_marker;
        visualization_msgs::Marker trajectory_marker;
        trajectory_marker.header.frame_id = "map";
        trajectory_marker.header.stamp = ros::Time();
        trajectory_marker.ns = "trajectories";
        trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_marker.action = visualization_msgs::Marker::ADD;
        trajectory_marker.scale.x = 0.05;
        trajectory_marker.frame_locked = false;
        for (size_t i = 0; i < generated_rollouts.size(); i++)
        {
            trajectory_marker.points.clear();
            trajectory_marker.id = i;
            for (size_t k = 0; k < generated_rollouts[i].size(); k++)
            {
                geometry_msgs::Point wp;
                wp.x = generated_rollouts[i][k].pos.x;
                wp.y = generated_rollouts[i][k].pos.y;
                wp.z = generated_rollouts[i][k].pos.z;
                trajectory_marker.points.push_back(wp);
            }

            if (best_index == i)
            {
                trajectory_marker.color.b = 1;
                trajectory_marker.color.g = 1;
                trajectory_marker.color.r = 1;
                trajectory_marker.color.a = 1;
                trajectory_marker.scale.x = 0.1;
            }
            else
            {
                trajectory_marker.color.b = 0.5;
                trajectory_marker.color.g = 0.8;
                trajectory_marker.color.r = 0.4;
                trajectory_marker.color.a = 0.7;
            }
            allRollouts_marker.markers.push_back(trajectory_marker);
        }
        pub_LocalWeightedTrajectoryRviz.publish(allRollouts_marker);
    }

    if (detect_objs.size() > 0)
    {
        visualization_msgs::MarkerArray objs_marker;
        visualization_msgs::Marker obj_marker;

        obj_marker.header.frame_id = "map";
        obj_marker.header.stamp = ros::Time();
        obj_marker.ns = "detected_objs";
        obj_marker.type = visualization_msgs::Marker::CUBE;
        obj_marker.action = visualization_msgs::Marker::ADD;
        obj_marker.frame_locked = false;
        for (int i = 0; i < detect_objs.size(); i++)
        {
            obj_marker.id = i;
            obj_marker.pose.position.x = detect_objs[i].center.pos.x;
            obj_marker.pose.position.y = detect_objs[i].center.pos.y;
            obj_marker.pose.position.z = detect_objs[i].center.pos.z;

            obj_marker.scale.x = 1;
            obj_marker.scale.y = 1;
            obj_marker.scale.z = 1;

            obj_marker.color.a = 1.0;
            obj_marker.color.b = 0.3;
            obj_marker.color.g = 0.5;
            obj_marker.color.r = 0.7;
            objs_marker.markers.push_back(obj_marker);
        }
        pub_collisionObjsRviz.publish(objs_marker);
    }
}

/**
 * @description: 计算轨迹代价函数（静态）
 * @param {type} 
 * @return: 
 */
PlannerHNS::TrajectoryCost LocalTrajectoryGenerator::trajectory_evaluator_static(const std::vector<std::vector<PlannerHNS::WayPoint>> &rollouts,
                                                                                 const std::vector<PlannerHNS::WayPoint> &centralPath,
                                                                                 const PlannerHNS::WayPoint &currPose,
                                                                                 const PlannerHNS::PlanningParams &params,
                                                                                 const PlannerHNS::CAR_BASIC_INFO &car_info,
                                                                                 const std::vector<PlannerHNS::DetectedObject> &obj_list)
{
    PlannerHNS::TrajectoryCost best_trajectory;
    best_trajectory.bBlocked = true;
    best_trajectory.closest_obj_distance = params.horizonDistance;
    best_trajectory.closest_obj_velocity = 0;
    best_trajectory.index = -1;

    // cal current trajectory where running
    PlannerHNS::RelativeInfo car_relativeInfo;
    UtilityNS::getRelativeInfo(centralPath, currPose, car_relativeInfo);

    int currIndex = params.rollOutNumber / 2 + floor(car_relativeInfo.perp_distance / params.rollOutDensity);
    if (currIndex > params.rollOutNumber)
        currIndex = params.rollOutNumber;
    else if (currIndex < 0)
        currIndex = 0;

    // cal center cost
    trajectoryCosts.clear();
    if (rollouts.size() > 0)
    {
        PlannerHNS::TrajectoryCost tc;
        for (int i = 0; i < rollouts.size(); i++)
        {
            tc.index = i;
            tc.relative_index = i - params.rollOutNumber / 2;
            tc.distance_from_center = params.rollOutDensity * tc.relative_index;
            tc.priority_cost = fabs(tc.distance_from_center);
            tc.closest_obj_distance = params.horizonDistance;
            tc.longitudinal_cost = 0;
            tc.lateral_cost = 0;
            trajectoryCosts.push_back(tc);
        }
    }
    // cal transition cost
    for (int ki = 0; ki < trajectoryCosts.size(); ki++)
        trajectoryCosts[ki].transition_cost = fabs(params.rollOutDensity * (currIndex - ki));

    if (obj_list.size() > 0)
    {
        PlannerHNS::WayPoint obj_p;
        allContourPoints.clear();
        for (int kk = 0; kk < obj_list.size(); kk++)
        {
            for (int m = 0; m < obj_list[kk].contour.size(); m++)
            {
                obj_p.pos = obj_list[kk].contour[m];
                obj_p.v = obj_list[kk].center.v;
                obj_p.id = kk;
                allContourPoints.push_back(obj_p);
            }
        }
        calLateralAndLongitudinalCostsStatic(trajectoryCosts, rollouts, centralPath, currPose, allContourPoints, params, car_info);
    }

    normalizeCosts(trajectoryCosts);

    int bestIndex = -1;
    double smallestCost = DBL_MAX;
    double smallestDis = DBL_MAX;

    for (size_t i = 0; i < trajectoryCosts.size(); i++)
    {
        // printf("%s\n", "--------------------------------");
        // printf("index: %d\n", trajectoryCosts[i].index);
        // printf("cost: %f\n", trajectoryCosts[i].cost);
        // printf("bBlock: %d\n", trajectoryCosts[i].bBlocked);
        // printf("lateral_cost: %f\n", trajectoryCosts[i].lateral_cost);
        // printf("longitudinal_cost: %f\n", trajectoryCosts[i].longitudinal_cost);
        // printf("priority_cost: %f\n", trajectoryCosts[i].priority_cost);
        // printf("transition_cost: %f\n", trajectoryCosts[i].transition_cost);

        if (!trajectoryCosts[i].bBlocked && trajectoryCosts[i].cost < smallestCost)
        {
            smallestCost = trajectoryCosts[i].cost;
            bestIndex = i;
        }

        if (trajectoryCosts[i].closest_obj_distance < smallestDis)
        {
            smallestDis = trajectoryCosts[i].closest_obj_distance;
        }
    }

    if (bestIndex == -1)
    {
        best_trajectory.bBlocked = true;
        best_trajectory.index = pre_best_index;
        best_trajectory.closest_obj_distance = smallestDis;
    }
    else if (bestIndex >= 0)
    {
        best_trajectory = trajectoryCosts[bestIndex];
    }
    pre_best_index = bestIndex;

    return best_trajectory;
}
/**
 * @description:正则化代价损失 
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::normalizeCosts(std::vector<PlannerHNS::TrajectoryCost> &trajectory_cost)
{
    double totalPriorities = 0;
    double totalLateralCosts = 0;
    double totalLongitudinalCosts = 0;
    double totalTransitionCosts = 0;

    double weightPriority = 0.2;
    double weightTransition = 0.0;
    double weightLateral = 1.8;
    double weightLong = 0.8;

    for (int i = 0; i < trajectory_cost.size(); i++)
    {
        totalPriorities += trajectory_cost[i].priority_cost;
        totalLateralCosts += trajectory_cost[i].lateral_cost;
        totalLongitudinalCosts += trajectory_cost[i].longitudinal_cost;
        totalTransitionCosts += trajectory_cost[i].transition_cost;
    }

    for (int k = 0; k < trajectory_cost.size(); k++)
    {
        if (totalPriorities != 0 && !std::isnan(trajectory_cost[k].priority_cost))
            trajectory_cost[k].priority_cost = trajectory_cost[k].priority_cost / totalPriorities;
        else
            trajectory_cost[k].priority_cost = 0;

        if (totalLateralCosts != 0 && !std::isnan(trajectory_cost[k].lateral_cost))
            trajectory_cost[k].lateral_cost = trajectory_cost[k].lateral_cost / totalLateralCosts;
        else
            trajectory_cost[k].lateral_cost = 0;

        if (totalLongitudinalCosts != 0 && !std::isnan(trajectory_cost[k].longitudinal_cost))
            trajectory_cost[k].longitudinal_cost = trajectory_cost[k].longitudinal_cost / totalLongitudinalCosts;
        else
            trajectory_cost[k].longitudinal_cost = 0;

        if (totalTransitionCosts != 0 && !std::isnan(trajectory_cost[k].transition_cost))
            trajectory_cost[k].transition_cost = trajectory_cost[k].transition_cost / totalTransitionCosts;
        else
            trajectory_cost[k].transition_cost = 0;

        trajectory_cost[k].cost = (weightPriority * trajectory_cost[k].priority_cost +
                                   weightLateral * trajectory_cost[k].lateral_cost +
                                   weightLong * trajectory_cost[k].longitudinal_cost +
                                   weightTransition * trajectory_cost[k].transition_cost) /
                                  4.0;

        // std::cout << "Index: " << k
        //                 << ", Priority: " << trajectory_cost.at(k).priority_cost
        //                 << ", Transition: " << trajectory_cost.at(k).transition_cost
        //                 << ", Lat: " << trajectory_cost.at(k).lateral_cost
        //                 << ", Long: " << trajectory_cost.at(k).longitudinal_cost
        //                 << ", Avg: " << trajectory_cost.at(k).cost
        //                 << std::endl;
    }
}

/**
 * @description: 计算轨迹到障碍物的水平代价函数和垂直代价函数
 * @param {type} 
 * @return: 
 */
void LocalTrajectoryGenerator::calLateralAndLongitudinalCostsStatic(std::vector<PlannerHNS::TrajectoryCost> &trajectoryCosts,
                                                                    const std::vector<std::vector<PlannerHNS::WayPoint>> &rollOuts,
                                                                    const std::vector<PlannerHNS::WayPoint> &centerPath,
                                                                    const PlannerHNS::WayPoint &currPose,
                                                                    const std::vector<PlannerHNS::WayPoint> &contourPoints,
                                                                    const PlannerHNS::PlanningParams &params,
                                                                    const PlannerHNS::CAR_BASIC_INFO &carInfo)
{
    double critical_lateral_distance = carInfo.width / 2.0 + params.horizontalSafetyDistancel;
    double critical_long_front_distance = carInfo.wheel_base / 2.0 + carInfo.length / 2.0 + params.verticalSafetyDistance;
    double critical_long_back_distance = carInfo.length / 2.0 - carInfo.wheel_base / 2.0 + params.verticalSafetyDistance;

    PlannerHNS::GPSPoint bottom_left(-critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
    PlannerHNS::GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance, currPose.pos.z, 0);
    PlannerHNS::GPSPoint top_left(-critical_lateral_distance, critical_long_front_distance, currPose.pos.z, 0);
    PlannerHNS::GPSPoint top_right(critical_lateral_distance, critical_long_front_distance, currPose.pos.z, 0);

    PlannerHNS::Mat3 invRoatationMat(currPose.pos.a);
    PlannerHNS::Mat3 invTranslationMat(currPose.pos.x, currPose.pos.y);

    bottom_left = invRoatationMat * bottom_left;
    bottom_left = invTranslationMat * bottom_left;

    bottom_right = invRoatationMat * bottom_right;
    bottom_right = invTranslationMat * bottom_right;

    top_left = invRoatationMat * top_left;
    top_left = invTranslationMat * top_left;

    top_right = invRoatationMat * top_right;
    top_right = invTranslationMat * top_right;
    double lateralDist;
    if (rollOuts.size() > 0 && rollOuts[0].size() > 0)
    {
        PlannerHNS::RelativeInfo car_rela_info;
        UtilityNS::getRelativeInfo(centerPath, currPose, car_rela_info);
        UtilityNS::visualLaneInRviz(centerPath, pub_testLane);
        for (int i = 0; i < rollOuts.size(); i++)
        {
            for (int k = 0; k < contourPoints.size(); k++)
            {
                PlannerHNS::RelativeInfo contour_rela_info;

                UtilityNS::getRelativeInfo(centerPath, contourPoints[k], contour_rela_info);

                if (contour_rela_info.iFront == 0 && contour_rela_info.iBack == 0 && contour_rela_info.direct_distance > 3)
                    continue;

                // 计算当前障碍物点到车辆位置的沿着中心轨迹的距离
                double longitudinalDist = getTwoPointsDistanceAlongTrajectory(centerPath, car_rela_info, contour_rela_info);
                if (contour_rela_info.iFront == 0 && longitudinalDist > 0)
                    longitudinalDist = -longitudinalDist;
                double distance_from_center = trajectoryCosts[i].distance_from_center;
                lateralDist = fabs(contour_rela_info.perp_distance - distance_from_center) * 2;

                if (lateralDist < 2 && longitudinalDist < params.minFollowingDistance && longitudinalDist >= -critical_long_back_distance)
                    trajectoryCosts[i].bBlocked = true;

                if (lateralDist != 0)
                    trajectoryCosts[i].lateral_cost += 1.0 / lateralDist;

                if (longitudinalDist != 0)
                    trajectoryCosts[i].longitudinal_cost += 1.0 / longitudinalDist;

                if (longitudinalDist >= -critical_long_back_distance && longitudinalDist < trajectoryCosts[i].closest_obj_distance)
                    trajectoryCosts[i].closest_obj_distance = longitudinalDist;
            }
        }
    }
}
/**
 * @description: 计算两个点之间的沿着轨迹线的距离 
 * @param {type} 
 * @return: 
 */
double LocalTrajectoryGenerator::getTwoPointsDistanceAlongTrajectory(const std::vector<PlannerHNS::WayPoint> &trajectory,
                                                                     const PlannerHNS::RelativeInfo &p1,
                                                                     const PlannerHNS::RelativeInfo &p2)
{
    if (trajectory.size() == 0)
        return 0;
    if (p1.iFront == p2.iFront && p1.iBack == p2.iBack)
    {
        return p2.to_front_distance - p1.to_front_distance;
    }
    else if (p1.iFront <= p2.iBack)
    {
        double dis_along_tra = p1.to_front_distance + p2.from_back_distance;
        for (int i = p1.iFront; i < p2.iBack; i++)
        {
            dis_along_tra += hypot(trajectory[i + 1].pos.y - trajectory[i].pos.y, trajectory[i + 1].pos.x - trajectory[i].pos.x);
        }
        return dis_along_tra;
    }
    else if (p2.iFront <= p1.iBack)
    {
        double dis_along_tra = p1.to_front_distance + p2.from_back_distance;
        for (int i = p1.iFront; i < p2.iBack; i++)
        {
            dis_along_tra += hypot(trajectory[i + 1].pos.y - trajectory[i].pos.y, trajectory[i + 1].pos.x - trajectory[i].pos.x);
        }
        return -dis_along_tra;
    }
    else
    {
        return 0;
    }
}
} // namespace LocalTrajectoryGeneratorNS