/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-02-25 14:41:15
 * @LastEditTime: 2019-03-18 20:57:02
 */
#ifndef COMMON_H
#define COMMON_H

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

namespace UtilityNS {

class GPSPoint {
public:
    double x;
    double y;
    double z;
    double yaw;

    GPSPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        yaw = 0;
    }

    GPSPoint(const double& x, const double& y, const double& z, const double& yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
    }

    std::string ToString()
    {
        std::stringstream str;
        str.precision(12);
        str << "X:" << x << ", Y:" << y << ", Z:" << z << ", Yaw:" << yaw << std::endl;
        return str.str();
    }
};

class Rotation {
public:
    double x;
    double y;
    double z;
    double w;

    Rotation()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

class WayPoint {
public:
    GPSPoint pos;
    Rotation rot;
    double v;
    double cost;
    int laneId;

    WayPoint()
    {
        v = 0;
        cost = 0;
        laneId = -1;
    }

    WayPoint(const double& x, const double& y, const double& z, const double& a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.yaw = a;

        v = 0;
        cost = 0;
        laneId = -1;
    }
};

class Mat3 {
    double m[3][3];

public:
    Mat3()
    {
        //initialize Identity by default
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = 0;

        m[0][0] = m[1][1] = m[2][2] = 1;
    }

    Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
    {
        m[0][0] = (mirrorX == true) ? -1 : 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = (mirrorY == true) ? -1 : 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double transX, double transY)
    {
        m[0][0] = 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double rotation_angle)
    {
        double c = cos(rotation_angle);
        double s = sin(rotation_angle);
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = 0;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = 0;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(GPSPoint rotationCenter)
    {
        double c = cos(rotationCenter.yaw);
        double s = sin(rotationCenter.yaw);
        double u = rotationCenter.x;
        double v = rotationCenter.y;
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = -u * c + v * s + u;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = -u * s - v * c + v;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    GPSPoint operator*(GPSPoint v)
    {
        GPSPoint _v = v;
        v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
        v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
        return v;
    }
};

class RelativeInfo {
public:
    double perp_distance;
    double to_front_distance; //negative
    double from_back_distance;
    int iFront;
    int iBack;
    WayPoint perp_point;
    double angle_diff; // degrees
    double direct_distance;

    RelativeInfo()
    {
        perp_distance = 0;
        to_front_distance = 0;
        iFront = 0;
        iBack = 0;
        angle_diff = 0;
        direct_distance = 0;
    }
};

class DetectedObject {
public:
    int id;
    std::string label;
    WayPoint center;
    std::vector<GPSPoint> contour;
    double w;
    double l;
    double h;

    ros::Time start_time;

    DetectedObject()
    {
        id = 0;
        w = 0;
        l = 0;
        h = 0;
    }
};

class TrajectoryCost {
public:
    int index;
    int relative_index;

    double cost;
    double priority_cost; //0 to 1
    double transition_cost; // 0 to 1
    double lateral_cost;
    double longitudinal_cost;

    double closest_obj_distance;
    double closest_obj_velocity;

    bool bBlocked;
    double distance_from_center;

    TrajectoryCost()
    {
        index = -1;
        distance_from_center = 0;
        relative_index = -100;
        closest_obj_velocity = 0;
        priority_cost = 0;
        transition_cost = 0;
        cost = 0;
        closest_obj_distance = -1;
        lateral_cost = 0;
        longitudinal_cost = 0;
        bBlocked = false;
    }

    std::string ToString()
    {
        std::ostringstream str;
        str.precision(4);
        str << ", In : " << relative_index;
        str << ", Co : " << cost;
        str << ", Pr : " << priority_cost;
        str << ", Tr : " << transition_cost;
        str << ", La : " << lateral_cost;
        str << ", Lo : " << longitudinal_cost;
        str << ", Bl : " << bBlocked;
        str << "\n";

        return str.str();
    }
};

class PlanningParams {
public:
    double maxSpeed;
    double minSpeed;
    double planningDistance;
    double microPlanDistance;
    double carTipMargin;
    double rollInMargin;
    double rollInSpeedFactor;
    double pathDensity;
    double rollOutDensity;
    int rollOutNumber;
    double horizonDistance;
    double minFollowingDistance; //should be bigger than Distance to follow
    double minDistanceToAvoid; // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
    double maxDistanceToAvoid; // should be smaller than minDistanceToAvoid
    double lateralSkipDistance;
    double speedProfileFactor;
    double smoothingDataWeight;
    double smoothingSmoothWeight;
    double smoothingToleranceError;

    double stopSignStopTime;

    double additionalBrakingDistance;
    double verticalSafetyDistance;
    double horizontalSafetyDistancel;

    double giveUpDistance;

    int nReliableCount;

    bool enableLaneChange;
    bool enableSwerving;
    bool enableFollowing;
    bool enableHeadingSmoothing;
    bool enableTrafficLightBehavior;
    bool enableStopSignBehavior;

    bool enabTrajectoryVelocities;
    double minIndicationDistance;

    PlanningParams()
    {
        maxSpeed = 3;
        lateralSkipDistance = 6.0;
        minSpeed = 0;
        planningDistance = 10000;
        microPlanDistance = 30;
        carTipMargin = 4.0;
        rollInMargin = 12.0;
        rollInSpeedFactor = 0.25;
        pathDensity = 0.25;
        rollOutDensity = 0.5;
        rollOutNumber = 4;
        horizonDistance = 120;
        minFollowingDistance = 35;
        minDistanceToAvoid = 15;
        maxDistanceToAvoid = 5;
        speedProfileFactor = 1.0;
        smoothingDataWeight = 0.47;
        smoothingSmoothWeight = 0.2;
        smoothingToleranceError = 0.05;

        stopSignStopTime = 2.0;

        additionalBrakingDistance = 10.0;
        verticalSafetyDistance = 0.0;
        horizontalSafetyDistancel = 0.0;

        giveUpDistance = -4;
        nReliableCount = 2;

        enableHeadingSmoothing = false;
        enableSwerving = false;
        enableFollowing = false;
        enableTrafficLightBehavior = false;
        enableLaneChange = false;
        enableStopSignBehavior = false;
        enabTrajectoryVelocities = false;
        minIndicationDistance = 15;
    }
};

class CAR_BASIC_INFO {
public:
    double wheel_base;
    double length;
    double width;

    CAR_BASIC_INFO()
    {
        wheel_base = 2.7;
        length = 4.3;
        width = 1.82;
    }
};

} // namespace UtilityNS

#endif //COMMON_H
