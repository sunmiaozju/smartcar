/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 14:15:49
 */

#ifndef ROLLOUT_GENERATOR_H
#define ROLLOUT_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include "op_planner/PlannerH.h"
#include "op_planner/PlannerCommonDef.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <smartcar_msgs/LaneArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "utils/utils.h"

namespace RolloutGeneratorNS
{

#define LANE_CHANGE_SPEED_FACTOR 0.5

class RolloutGenerator
{
private:
  PlannerHNS::WayPoint current_pose;
  PlannerHNS::WayPoint init_pose;
  std::vector<std::vector<PlannerHNS::WayPoint>> globalPaths;
  std::vector<PlannerHNS::WayPoint> centralTrajectorySmoothed;
  std::vector<std::vector<PlannerHNS::WayPoint>> globalPathSections;
  std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> rollOuts;

  bool currentPose_flag;

  ros::NodeHandle nh;
  ros::Publisher pub_localTrajectories;
  ros::Publisher pub_localTrajectoriesRviz;
  ros::Publisher pub_centralPathSection;
  ros::Publisher pub_testLane;
  ros::Subscriber sub_currentPose;
  ros::Subscriber sub_currentVelocity;
  ros::Subscriber sub_globalPlannerPath;

  PlannerHNS::PlanningParams PlanningParams;
  PlannerHNS::PlannerH Planner;
  PlannerHNS::CAR_BASIC_INFO CarInfo;
  PlannerHNS::VehicleState VehicleStatus;

  ros::Publisher pub_test;

  void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg);

  void visualInRviz(std::vector<PlannerHNS::WayPoint> test_points);

  void getVehicleStatus_cb(const geometry_msgs::TwistStampedConstPtr &msg);

  void getRobotOdom_cb(const nav_msgs::OdometryConstPtr &msg);

  void getGlobalPlannerPath_cb(const smartcar_msgs::LaneArrayConstPtr &msg);

  void msgLane2LocalLane(const smartcar_msgs::Lane &trajectory, std::vector<PlannerHNS::WayPoint> &path);

  double calcAngleAndCost(std::vector<PlannerHNS::WayPoint> &path);

  void extractPartFromTrajectory(const std::vector<PlannerHNS::WayPoint> &originalPath,
                                 const PlannerHNS::WayPoint &currnetPos,
                                 const double &minDistance,
                                 const double &waypointDensity,
                                 std::vector<PlannerHNS::WayPoint> &extractedPath);

  void fixPathDensity(std::vector<PlannerHNS::WayPoint> &path, const double &pathDensity);

  void predictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint> &path, const PlannerHNS::WayPoint &currPose, const double &minSpeed);

  void trajectoryToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray);

  void generateRunoffTrajectory(const std::vector<std::vector<PlannerHNS::WayPoint>> &referencePaths,
                                const PlannerHNS::WayPoint &carPos, const double &speed, const double &microPlanDistance,
                                const double &carTipMargin, const double &rollInMargin, const double &rollInSpeedFactor,
                                const double &pathDensity, const double &rollOutDensity, const int &rollOutNumber,
                                const double &SmoothDataWeight, const double &SmoothWeight, const double &SmoothTolerance,
                                std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> &rollOutsPaths,
                                std::vector<PlannerHNS::WayPoint> &sampledPoints_debug);

  void calculateRollInTrajectories(const PlannerHNS::WayPoint &carPos, const double &speed, const std::vector<PlannerHNS::WayPoint> &originalCenter,
                                   int &start_index, int &end_index, std::vector<double> &end_laterals,
                                   std::vector<std::vector<PlannerHNS::WayPoint>> &rollInPaths, const double &max_roll_distance,
                                   const double &carTipMargin, const double &rollInMargin, const double &rollInSpeedFactor,
                                   const double &pathDensity, const double &rollOutDensity, const int &rollOutNumber,
                                   const double &SmoothDataWeight, const double &SmoothWeight, const double &SmoothTolerance,
                                   std::vector<PlannerHNS::WayPoint> &sampledPoints);

  void smoothPath(std::vector<PlannerHNS::WayPoint> &path, double weight_data, double weight_smooth, double tolerance);

public:
  RolloutGenerator();

  ~RolloutGenerator();
  
  void run();

  void initROS();
};
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H
