/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-05 21:14:38
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

class RolloutGenerator
{
private:
  // PlannerHNS::PlannerH m_Planner;
  // geometry_msgs::Pose m_OriginPos;
  PlannerHNS::WayPoint current_pose;
  PlannerHNS::WayPoint init_pose;
  std::vector<std::vector<PlannerHNS::WayPoint>> GlobalPaths;

  bool currentPose_flag;
  bool initPose_flag;

  ros::NodeHandle nh;
  ros::Publisher pub_localTrajectories;
  ros::Publisher pub_localTrajectoriesRviz;
  ros::Subscriber sub_initialPose;
  ros::Subscriber sub_currentPose;
  ros::Subscriber sub_currentVelocity;
  ros::Subscriber sub_robotOdom;
  ros::Subscriber sub_globalPlannerPath;

  PlannerHNS::PlanningParams PlanningParams;
  PlannerHNS::CAR_BASIC_INFO CarInfo;

  void getInitPose_cb(const geometry_msgs::PoseWithCovarianceConstPtr &msg);
  void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg);
  void getVehicleStatus_cb(const geometry_msgs::TwistStampedConstPtr &msg);
  void getRobotOdom_cb(const nav_msgs::OdometryConstPtr &msg);
  void getGlobalPlannerPath_cb(const smartcar_msgs::LaneArrayConstPtr &msg);
  void msgLane2LocalLane(const smartcar_msgs::Lane &trajectory, std::vector<PlannerHNS::WayPoint> &path);
  double calcAngleAndCost(std::vector<PlannerHNS::WayPoint> &path);
  double dealNegativeAngle(const double &ang);

public:
  RolloutGenerator();
  ~RolloutGenerator();
  void run();
  void initROS();
};
} // namespace RolloutGeneratorNS

#endif //ROLLOUT_GENERATOR_H
