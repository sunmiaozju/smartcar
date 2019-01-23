#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// User defined includes
#include "smartcar_msgs/Lane.h"
#include "smartcar_msgs/LaneArray.h"
#include "smartcar_config_msgs/ConfigWaypointFollower.h"
#include "smartcar_msgs/ControlCommandStamped.h"
//#include "pure_pursuit_viz.h"

namespace waypoint_follower {

    tf::Vector3 point2vector(geometry_msgs::Point point) {
        tf::Vector3 vector(point.x, point.y, point.z);
        return vector;
    }

    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c) {
        //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
        double sub_x = fabs(start.x - end.x);
        double sub_y = fabs(start.y - end.y);
        double error = pow(10, -5);  // 0.00001

        if (sub_x < error && sub_y < error) {
            ROS_INFO("two points are the same point!!");
            return false;
        }

        *a = end.y - start.y;
        *b = (-1) * (end.x - start.x);
        *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

        return true;
    }

    inline double deg2rad(double deg) {
        return deg * M_PI / 180;
    }

//    enum class Mode : int32_t {
//        waypoint,
//        dialog,
//
//        unknown = -1,
//    };

//    template<class T>
//    typename std::underlying_type<T>::type enumToInteger(T t) {
//        return static_cast<typename std::underlying_type<T>::type>(t);
//    }

    class PurePursuitNode {
    public:
        PurePursuitNode();

        ~PurePursuitNode();

        void run();

        // for setting data
        void setLookaheadDistance(const double &ld) {
            lookahead_distance_ = ld;
        }

        void setMinimumLookaheadDistance(const double &minld) {
            minimum_lookahead_distance_ = minld;
        }

        void setCurrentWaypoints(const std::vector <smartcar_msgs::Waypoint> &wps) {
            current_waypoints_ = wps;
        }

        void setCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg) {
            current_pose_ = msg->pose;
        }

        void setLinearInterpolationParameter(const bool &param) {
            is_linear_interpolation_ = param;
        }

        // for debug on ROS
        geometry_msgs::Point getPoseOfNextWaypoint() const {
            return current_waypoints_.at(next_waypoint_number_).pose.pose.position;
        }

        geometry_msgs::Point getPoseOfNextTarget() const {
            return next_target_position_;
        }

        geometry_msgs::Pose getCurrentPose() const {
            return current_pose_;
        }

        std::vector <smartcar_msgs::Waypoint> getCurrentWaypoints() const {
            return current_waypoints_;
        }

        double getLookaheadDistance() const {
            return lookahead_distance_;
        }

        double getMinimumLookaheadDistance() const {
            return minimum_lookahead_distance_;
        }

        // processing
        bool canGetCurvature(double *output_curvature);

    private:
        // handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // publisher
        ros::Publisher pub, pub2_, pub11_, pub12_, pub13_, pub14_, pub15_, pub16_, pub17_;

        // subscriber
        ros::Subscriber sub1_, sub2_, sub3_, sub4_;

        // constant
        const double RADIUS_MAX_;
        const double curvature_MIN_;
        const int LOOP_RATE_;  // processing frequency

        // variables
        bool CONST_VEL_DIS;
        bool is_linear_interpolation_;
        int next_waypoint_number_;
        geometry_msgs::Point next_target_position_;
        double lookahead_distance_;
        geometry_msgs::Pose current_pose_;
        double current_linear_velocity_;
        std::vector <smartcar_msgs::Waypoint> current_waypoints_;
        bool is_velocity_set_;
        bool is_waypoint_set_, is_pose_set_;
        double command_linear_velocity_;
        double wheel_base_;
        bool is_const_vel_dis_;               // false = dynamic_vel_and_ahead_distance, true = const_vel_and_ahead_distance
        double const_lookahead_distance_;  // meter
        double const_velocity_;            // km/h
        double lookahead_distance_ratio_;  // 这个是前视距离与当前速度的比例系数，将前视距离与速度关联起来，速度越快，前视觉距离越远
        double minimum_lookahead_distance_;  // 最小前视距离，通过速度乘系数得到的前视距离不小于这个值，不大于当前速度×10

        // callbacks

        void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

        void callbackFromCurrentVelocity(const nav_msgs::Odometry &msg);

        void callbackFromWayPoints(const smartcar_msgs::LaneArrayConstPtr &msg);

        // initializer
        void initForROS();

        // functions
        double calcCurvature(geometry_msgs::Point target) const;

        bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;

        void getNextWaypoint();

        void publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) const;

        void publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                             const std::vector <smartcar_msgs::Waypoint> &waypoints) const;

        double computeLookaheadDistance() const;

        double computeCommandVelocity() const;

        double computeCommandAccel() const;

        double computeAngularGravity(double velocity, double curvature) const;
    };

    double convertCurvatureToSteeringAngle(const double &wheel_base, const double &curvature);

    // calculation relative coordinate of point from current_pose frame
    geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
        tf::Transform inverse;
        tf::poseMsgToTF(current_pose, inverse);
        tf::Transform transform = inverse.inverse();

        tf::Point p;
        pointMsgToTF(point_msg, p);
        tf::Point tf_p = transform * p;
        geometry_msgs::Point tf_point_msg;
        pointTFToMsg(tf_p, tf_point_msg);

        return tf_point_msg;
    }

    // distance between target 1 and target2 in 2-D
    double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2) {
        tf::Vector3 v1 = point2vector(target1);
        v1.setZ(0);
        tf::Vector3 v2 = point2vector(target2);
        v2.setZ(0);
        return tf::tfDistance(v1, v2);
    }

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c) {
        double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

        return d;
    }

    tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree) {
        tf::Vector3
        w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
           sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
        tf::Vector3 unit_w1 = w1.normalize();
        return unit_w1;
    }

}  // waypoint_follower

#endif  // PURE_PURSUIT_H
