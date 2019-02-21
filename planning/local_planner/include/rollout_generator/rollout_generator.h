/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-21 10:41:49
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

  void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg);
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



public:
  RolloutGenerator();
  ~RolloutGenerator();
  void run();
  void initROS();
};
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H
