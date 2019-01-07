#ifndef WAYPOINT_LOADER_CORE_H
#define WAYPOINT_LOADER_CORE_H

// ROS includes
#include <ros/ros.h>
#include "smartcar_msgs/LaneArray.h"
#include "smartcar_msgs/Lane.h"
#include "smartcar_msgs/Waypoint.h"

// C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <unordered_map>

namespace waypoint_maker
{
const std::string MULTI_LANE_CSV = "/tmp/driving_lane.csv";

enum class FileFormat : int32_t
{
    ver1, // x,y,z,(velocity)  这个是比较基本的csv文件，只有xyz信息和速度信息
    ver2, // 这个版本的csv文件是我们自定义的信息文件，需要把我们自定义的信息写在第一行作为注释标签栏

    unknown = -1,
};

typedef std::underlying_type<FileFormat>::type FileFormatInteger;

inline double kmph2mps(double velocity_kmph)
{
    return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
    return (velocity_mps * 60 * 60) / 1000;
}

class WaypointLoaderNode
{
  public:
    WaypointLoaderNode();

    ~WaypointLoaderNode();

    void run();

  private:
    // handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher & subscriber
    ros::Publisher lane_pub_;
    ros::Subscriber config_sub_;
    ros::Subscriber output_cmd_sub_;

    // variables
    std::string multi_lane_csv_;
    std::vector<std::string> multi_file_path_;
    int speed;
    
    // initializer
    void initROS();

    // functions
    void createLaneWaypoint(const std::string &file_path, smartcar_msgs::Lane *lane);

    void createLaneArray(const std::vector<std::string> &paths, smartcar_msgs::LaneArray *lane_array);

    FileFormat checkFileFormat(const char *filename);

    bool verifyFileConsistency(const char *filename);

    void loadWaypointsForVer1(const char *filename, std::vector<smartcar_msgs::Waypoint> *wps);

    void parseWaypointForVer1(const std::string &line, smartcar_msgs::Waypoint *wp);

    void loadWaypointsForVer2(const char *filename, std::vector<smartcar_msgs::Waypoint> *wps);

    void parseWaypointForVer2(const std::string &line, const std::vector<std::string> &contents,
                                              smartcar_msgs::Waypoint *wp);
};

void parseColumns(const std::string &line, std::vector<std::string> *columns);

size_t countColumns(const std::string &line);
} // namespace waypoint_maker
#endif // WAYPOINT_LOADER_CORE_H