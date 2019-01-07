#include <ros/ros.h>

#include "waypoint_loader.h"

namespace waypoint_maker
{
// Constructor
WaypointLoaderNode::WaypointLoaderNode() : private_nh_("~")
{
    initROS();
}

// Destructor
WaypointLoaderNode::~WaypointLoaderNode()
{
}

void WaypointLoaderNode::initROS()
{
    // setup publisher
    lane_pub_ = nh_.advertise<smartcar_msgs::LaneArray>("lane_array", 10, true);

    // parameter settings
    private_nh_.param("lane_array_filename", multi_lane_csv_, std::string("filename not config in launch"));
    private_nh_.param("speed", speed, int(1));

}

void WaypointLoaderNode::run()
{
    ros::Rate loop_rate(0.5);
    while(ros::ok()){    
        multi_file_path_.clear();
        // multi_lane_csv_是一个以逗号分隔开得很多车道路径名(一个字符串，不同车道线以字符串分割)
        // multi_file_path_是一个元素为string的vector，里面保存了解析出来的多车道路径
        parseColumns(multi_lane_csv_, &multi_file_path_);
        smartcar_msgs::LaneArray lane_array;
        createLaneArray(multi_file_path_, &lane_array);
        lane_pub_.publish(lane_array);
        loop_rate.sleep();
    }
}

void WaypointLoaderNode::createLaneArray(const std::vector<std::string> &paths, smartcar_msgs::LaneArray *lane_array)
{
    for (const auto &el : paths)
    {
        smartcar_msgs::Lane lane;
        createLaneWaypoint(el, &lane);
        
        lane_array->lanes.push_back(lane);
    }
}

void WaypointLoaderNode::createLaneWaypoint(const std::string &file_path, smartcar_msgs::Lane *lane)
{
    if (!verifyFileConsistency(file_path.c_str()))
    {
        ROS_ERROR("lane data is something wrong...");
        return;
    }

    ROS_INFO("lane data is valid. publishing...");
    FileFormat format = checkFileFormat(file_path.c_str());
    std::vector<smartcar_msgs::Waypoint> wps;
    if (format == FileFormat::ver1)
    {
        loadWaypointsForVer1(file_path.c_str(), &wps);
    }
    else if (format == FileFormat::ver2)
    {
        loadWaypointsForVer2(file_path.c_str(), &wps);
    }
    
    lane->header.frame_id = "/map";
    lane->header.stamp = ros::Time(0);
    lane->waypoints = wps;
}

void WaypointLoaderNode::loadWaypointsForVer1(const char *filename, std::vector<smartcar_msgs::Waypoint> *wps)
{
    std::ifstream ifs(filename);

    if (!ifs)
    {
        return;
    }

    std::string line;
    std::getline(ifs, line); // Remove first line

    while (std::getline(ifs, line))
    {
        smartcar_msgs::Waypoint wp;
        parseWaypointForVer1(line, &wp);
        wps->push_back(wp);
    }

    // 因为 ver1 里面只有xyz和vel信息，因此需要自己计算yaw信息
    size_t last = wps->size() - 1;
    for (size_t i = 0; i < wps->size(); ++i)
    {
        if (i != last)
        {
            double yaw = atan2(wps->at(i + 1).pose.pose.position.y - wps->at(i).pose.pose.position.y,
                               wps->at(i + 1).pose.pose.position.x - wps->at(i).pose.pose.position.x);
            wps->at(i).pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            wps->at(i).pose.pose.orientation = wps->at(i - 1).pose.pose.orientation;
        }
    }
}

void WaypointLoaderNode::parseWaypointForVer1(const std::string &line, smartcar_msgs::Waypoint *wp)
{
    std::vector<std::string> columns;
    parseColumns(line, &columns);

    wp->pose.pose.position.x = std::stod(columns[0]);
    wp->pose.pose.position.y = std::stod(columns[1]);
    wp->pose.pose.position.z = std::stod(columns[2]);
    // wp->twist.twist.linear.x = kmph2mps(std::stod(columns[3]));
    wp->twist.twist.linear.x = speed;
}

void WaypointLoaderNode::loadWaypointsForVer2(const char *filename, std::vector<smartcar_msgs::Waypoint> *wps)
{
    std::ifstream ifs(filename);

    if (!ifs)
    {
        return;
    }

    std::string line;
    std::getline(ifs, line); // get first line
    std::vector<std::string> contents;
    parseColumns(line, &contents);

    // std::getline(ifs, line);  // remove second line
    while (std::getline(ifs, line))
    {
        smartcar_msgs::Waypoint wp;
        parseWaypointForVer2(line, contents, &wp);
        wps->push_back(wp);
    }
}

void WaypointLoaderNode::parseWaypointForVer2(const std::string &line, const std::vector<std::string> &contents,
                                              smartcar_msgs::Waypoint *wp)
{
    std::vector<std::string> columns;
    parseColumns(line, &columns);
    std::unordered_map<std::string, std::string> map;
    for (size_t i = 0; i < contents.size(); i++)
    {
        map[contents.at(i)] = columns.at(i);
    }

    wp->pose.pose.position.x = std::stod(map["x"]);
    wp->pose.pose.position.y = std::stod(map["y"]);
    wp->pose.pose.position.z = std::stod(map["z"]);
    wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(map["yaw"]));
    wp->twist.twist.linear.x = kmph2mps(std::stod(map["velocity"]));
    //wp->change_flag = std::stoi(map["change_flag"]);
    //wp->wpstate.steering_state = (map.find("steering_flag") != map.end()) ? std::stoi(map["steering_flag"]) : 0;
    //wp->wpstate.accel_state = (map.find("accel_flag") != map.end()) ? std::stoi(map["accel_flag"]) : 0;
    //wp->wpstate.stopline_state = (map.find("stop_flag") != map.end()) ? std::stoi(map["stop_flag"]) : 0;
    //wp->wpstate.event_state = (map.find("event_flag") != map.end()) ? std::stoi(map["event_flag"]) : 0;
}

FileFormat WaypointLoaderNode::checkFileFormat(const char *filename)
{
    // 定义一个输入文件对象
    std::ifstream ifs(filename);

    if (!ifs)
    {
        return FileFormat::unknown;
    }

    // get first
    // 读取文件的第一行，第一行有很多列，用逗号间隔开
    std::string line;
    std::getline(ifs, line);

    // parse first line
    // 解析第一行数据，以逗号间隔开，最终保存在 vector<string> 中
    std::vector<std::string> parsed_columns;
    parseColumns(line, &parsed_columns);

    // check if first element in the first column does not include digit
    // 检查第一行第一个string字符串里面有没有数字字符，如果一个都没有，则返回FileFormat::ver3，代表这个是第三种文件(第一行是文字说明，肯定没有数字)
    if (!std::any_of(parsed_columns.at(0).cbegin(), parsed_columns.at(0).cend(), isdigit))
    {
        return FileFormat::ver2;
    }
    else
    {
        return FileFormat::ver1;
    }
}

bool WaypointLoaderNode::verifyFileConsistency(const char *filename)
{
    ROS_INFO("verify...");
    std::ifstream ifs(filename);

    if (!ifs)
    {
        return false;
    }

    FileFormat format = checkFileFormat(filename);
    ROS_INFO("file format: %d", static_cast<int>(format)+1);
    if (format == FileFormat::unknown)
    {
        ROS_ERROR("unknown file format");
        return false;
    }

    std::string line;
    std::getline(ifs, line); // remove first line

    // 确定当前的csvfile文件是什么，不同的文件可能第一行不一样，还有每一行的列不一样
    size_t ncol = format == FileFormat::ver1 ? countColumns(line)                                               // x,y,z,velocity
                                             : format == FileFormat::ver2 ? countColumns(line) // x,y,z + 自定义
                                                                          : static_cast<int>(FileFormat::unknown);

    if (ncol == static_cast<int>(FileFormat::unknown))
    {
        return false;
    }

    // 因为文件是以流来操作的，之前已经输入了第一行，因此接下来就开始从第二行输入
    // 检查所有的文件每一行，是否都满足一致的列大小，如果有一列不满足，就返回false，说明文件格式有问题
    while (std::getline(ifs, line)) // search from second line
    {
        if (countColumns(line) != ncol)
        {
            return false;
        }
    }
    return true;
}

void parseColumns(const std::string &line, std::vector<std::string> *columns)
{
    // 拆分字符
    std::istringstream ss(line);
    std::string column;
    while (std::getline(ss, column, ','))
    {
        while (1)
        {
            // 去除（string）column的空格
            auto res = std::find(column.begin(), column.end(), ' ');
            if (res == column.end())
            {
                break;
            }
            column.erase(res);
        }
        if (!column.empty())
        {
            columns->push_back(column);
        }
    }
}

size_t countColumns(const std::string &line)
{
    std::istringstream ss(line);
    size_t ncol = 0;

    std::string column;
    while (std::getline(ss, column, ','))
    {
        ++ncol;
    }

    return ncol;
}

} // namespace waypoint_maker

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader");
    waypoint_maker::WaypointLoaderNode wln;
    wln.run();
    ros::spin();

    return 0;
}