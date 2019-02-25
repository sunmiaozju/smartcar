/*
 * @Description: collision avoid and local trajectory generator implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-15 14:55:06
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 14:08:22
 */

#ifndef LOCAL_TRAJECTORY_GENERATOR_H
#define LOCAL_TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>

#include <smartcar_msgs/LaneArray.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/DetectedObjectArray.h>

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "utils/utils.h"

namespace LocalTrajectoryGeneratorNS
{
class LocalTrajectoryGenerator
{
  ros::NodeHandle nh;
  ros::Subscriber sub_currentPose;
  ros::Subscriber sub_localRollouts;
  ros::Subscriber sub_detectedObjects;
  ros::Subscriber sub_centralPath;
  ros::Publisher pub_collisionObjsRviz;
  ros::Publisher pub_LocalWeightedTrajectoryRviz;
  ros::Publisher pub_LocalWeightedTrajectory;
  ros::Publisher pub_TrajectoryCost;
  ros::Publisher pub_testLane;

  std::vector<PlannerHNS::WayPoint> centralPathSection;
  PlannerHNS::WayPoint current_pose;
  std::vector<std::vector<PlannerHNS::WayPoint>> generated_rollouts;
  bool currentPose_flag;
  PlannerHNS::PlanningParams plan_params;
  std::vector<PlannerHNS::TrajectoryCost> trajectoryCosts;
  std::vector<PlannerHNS::WayPoint> allContourPoints;
  PlannerHNS::CAR_BASIC_INFO car_info;
  double pre_best_index;
  double best_index;
  std::vector<PlannerHNS::DetectedObject> detect_objs;

  void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg);
  void getRolloutPaths_cb(const smartcar_msgs::LaneArrayConstPtr &msg);
  void getDetectedObjects_cb(const smartcar_msgs::DetectedObjectArrayConstPtr &msg);
  void getCentralPathSection_cb(const smartcar_msgs::LaneConstPtr &msg);
  void initROS();
  PlannerHNS::TrajectoryCost trajectory_evaluator_static(const std::vector<std::vector<PlannerHNS::WayPoint>> &rollouts,
                                                         const std::vector<PlannerHNS::WayPoint> &centralPath,
                                                         const PlannerHNS::WayPoint &currPose,
                                                         const PlannerHNS::PlanningParams &params,
                                                         const PlannerHNS::CAR_BASIC_INFO &car_info,
                                                         const std::vector<PlannerHNS::DetectedObject> &obj_list);

  void calLateralAndLongitudinalCostsStatic(std::vector<PlannerHNS::TrajectoryCost> &trajectoryCosts,
                                            const std::vector<std::vector<PlannerHNS::WayPoint>> &rollOuts,
                                            const std::vector<PlannerHNS::WayPoint> &centerPath,
                                            const PlannerHNS::WayPoint &currPose,
                                            const std::vector<PlannerHNS::WayPoint> &contourPoints,
                                            const PlannerHNS::PlanningParams &params,
                                            const PlannerHNS::CAR_BASIC_INFO &carInfo);
  double getTwoPointsDistanceAlongTrajectory(const std::vector<PlannerHNS::WayPoint> &trajectory,
                                             const PlannerHNS::RelativeInfo &p1,
                                             const PlannerHNS::RelativeInfo &p2);
  void normalizeCosts(std::vector<PlannerHNS::TrajectoryCost> &trajectory_cost);
  void visualInRviz();

public:
  LocalTrajectoryGenerator();
  ~LocalTrajectoryGenerator();
  void run();
};
} // namespace LocalTrajectoryGeneratorNS

#endif //LOCAL_TRAJECTORY_GENERATOR_H
