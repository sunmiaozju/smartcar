/*
 * @Description: collision avoid and local trajectory generator implement
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-15 14:55:06
 * @LastEditors: sunm
 * @LastEditTime: 2019-02-25 15:36:15
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

  std::vector<UtilityNS::WayPoint> centralPathSection;
  UtilityNS::WayPoint current_pose;
  std::vector<std::vector<UtilityNS::WayPoint>> generated_rollouts;
  bool currentPose_flag;
  UtilityNS::PlanningParams plan_params;
  std::vector<UtilityNS::TrajectoryCost> trajectoryCosts;
  std::vector<UtilityNS::WayPoint> allContourPoints;
  UtilityNS::CAR_BASIC_INFO car_info;
  double pre_best_index;
  double best_index;
  std::vector<UtilityNS::DetectedObject> detect_objs;

  void getCurrentPose_cb(const geometry_msgs::PoseStampedConstPtr &msg);
  void getRolloutPaths_cb(const smartcar_msgs::LaneArrayConstPtr &msg);
  void getDetectedObjects_cb(const smartcar_msgs::DetectedObjectArray &msg);
  void getCentralPathSection_cb(const smartcar_msgs::LaneConstPtr &msg);
  void initROS();
  UtilityNS::TrajectoryCost trajectory_evaluator_static(const std::vector<std::vector<UtilityNS::WayPoint>> &rollouts,
                                                         const std::vector<UtilityNS::WayPoint> &centralPath,
                                                         const UtilityNS::WayPoint &currPose,
                                                         const UtilityNS::PlanningParams &params,
                                                         const UtilityNS::CAR_BASIC_INFO &car_info,
                                                         const std::vector<UtilityNS::DetectedObject> &obj_list);

  void calLateralAndLongitudinalCostsStatic(std::vector<UtilityNS::TrajectoryCost> &trajectoryCosts,
                                            const std::vector<std::vector<UtilityNS::WayPoint>> &rollOuts,
                                            const std::vector<UtilityNS::WayPoint> &centerPath,
                                            const UtilityNS::WayPoint &currPose,
                                            const std::vector<UtilityNS::WayPoint> &contourPoints,
                                            const UtilityNS::PlanningParams &params,
                                            const UtilityNS::CAR_BASIC_INFO &carInfo);
  double getTwoPointsDistanceAlongTrajectory(const std::vector<UtilityNS::WayPoint> &trajectory,
                                             const UtilityNS::RelativeInfo &p1,
                                             const UtilityNS::RelativeInfo &p2);
  void normalizeCosts(std::vector<UtilityNS::TrajectoryCost> &trajectory_cost);
  void visualInRviz();

public:
  LocalTrajectoryGenerator();
  ~LocalTrajectoryGenerator();
  void run();
};
} // namespace LocalTrajectoryGeneratorNS

#endif //LOCAL_TRAJECTORY_GENERATOR_H
