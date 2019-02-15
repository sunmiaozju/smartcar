/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-15 10:29:23
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

namespace RolloutGeneratorNS
{

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI

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
  int getNextClosePointIndex(const std::vector<PlannerHNS::WayPoint> &trajectory,
                             const PlannerHNS::WayPoint &current_pos,
                             const int &prevIndex = 0);
  double calDiffBetweenTwoAngle(const double &a1, const double &a2);
  void fixPathDensity(std::vector<PlannerHNS::WayPoint> &path, const double &pathDensity);
  void predictTimeCostForTrajectory(std::vector<PlannerHNS::WayPoint> &path, const PlannerHNS::WayPoint &currPose, const double &minSpeed);
  double cast_from_PI_to_PI_Angle(const double &ang);
  double cast_from_0_to_2PI_Angle(const double &ang);
  void trajectoryToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray);



public:
  RolloutGenerator();
  ~RolloutGenerator();
  void run();
  void initROS();
};
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H
