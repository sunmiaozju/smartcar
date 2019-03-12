/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-03-12 15:16:45
 */

#ifndef ROLLOUT_GENERATOR_H
#define ROLLOUT_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <smartcar_msgs/LaneArray.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include "utils/utils.h"

namespace RolloutGeneratorNS {

#define LANE_CHANGE_SPEED_FACTOR 0.5

class RolloutGenerator {
private:
    UtilityNS::WayPoint current_pose;
    UtilityNS::WayPoint init_pose;
    std::vector<std::vector<UtilityNS::WayPoint>> globalPaths;
    std::vector<UtilityNS::WayPoint> centralTrajectorySmoothed;
    std::vector<std::vector<UtilityNS::WayPoint>> globalPathSections;
    std::vector<std::vector<std::vector<UtilityNS::WayPoint>>> rollOuts;

    bool currentPose_flag;

    ros::NodeHandle nh;
    ros::Publisher pub_localTrajectories;
    ros::Publisher pub_localTrajectoriesRviz;
    ros::Publisher pub_centralPathSection;
    ros::Publisher pub_testLane;
    ros::Subscriber sub_currentPose;
    ros::Subscriber sub_currentVelocity;
    ros::Subscriber sub_globalPlannerPath;

    UtilityNS::PlanningParams PlanningParams;
    UtilityNS::CAR_BASIC_INFO CarInfo;
    double speed;

    ros::Publisher pub_test;

    void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

    void visualInRviz(std::vector<UtilityNS::WayPoint> test_points);

    void getVehicleStatus_cb(const geometry_msgs::TwistStampedConstPtr& msg);

    void getRobotOdom_cb(const nav_msgs::OdometryConstPtr& msg);

    void getGlobalPlannerPath_cb(const smartcar_msgs::LaneArrayConstPtr& msg);

    void getNavGlobalPlannerPath_cb(const nav_msgs::PathConstPtr& msg);

    void msgLane2LocalLane(const smartcar_msgs::Lane& trajectory, std::vector<UtilityNS::WayPoint>& path);

    double calcAngleAndCost(std::vector<UtilityNS::WayPoint>& path);

    void extractPartFromTrajectory(const std::vector<UtilityNS::WayPoint>& originalPath,
        const UtilityNS::WayPoint& currnetPos,
        const double& minDistance,
        const double& waypointDensity,
        std::vector<UtilityNS::WayPoint>& extractedPath);

    void fixPathDensity(std::vector<UtilityNS::WayPoint>& path, const double& pathDensity);

    void trajectoryToMarkers(const std::vector<std::vector<std::vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray);

    void generateRunoffTrajectory(const std::vector<std::vector<UtilityNS::WayPoint>>& referencePaths,
        const UtilityNS::WayPoint& carPos, const double& speed, const double& microPlanDistance,
        const double& carTipMargin, const double& rollInMargin, const double& rollInSpeedFactor,
        const double& pathDensity, const double& rollOutDensity, const int& rollOutNumber,
        const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance,
        std::vector<std::vector<std::vector<UtilityNS::WayPoint>>>& rollOutsPaths,
        std::vector<UtilityNS::WayPoint>& sampledPoints_debug);

    void calculateRollInTrajectories(const UtilityNS::WayPoint& carPos, const double& speed, const std::vector<UtilityNS::WayPoint>& originalCenter,
        int& start_index, int& end_index, std::vector<double>& end_laterals,
        std::vector<std::vector<UtilityNS::WayPoint>>& rollInPaths, const double& max_roll_distance,
        const double& carTipMargin, const double& rollInMargin, const double& rollInSpeedFactor,
        const double& pathDensity, const double& rollOutDensity, const int& rollOutNumber,
        const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance,
        std::vector<UtilityNS::WayPoint>& sampledPoints);

    void smoothPath(std::vector<UtilityNS::WayPoint>& path, double weight_data, double weight_smooth, double tolerance);

public:
    RolloutGenerator();

    ~RolloutGenerator();

    void run();

    void initROS();
};
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H
