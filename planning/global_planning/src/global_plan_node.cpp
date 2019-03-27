#include <dirent.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sstream>

#include <path_msgs/Cross.h>
#include <path_msgs/Lane.h>
#include <path_msgs/choose.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "utils.hpp"
#include <dynamic_reconfigure/server.h>
#include <global_planning/GlobalPlanningConfig.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/Waypoint.h>

namespace GLOBAL_PLANNER {
bool cmp_c(const path_msgs::Cross a, const path_msgs::Cross b)
{
    return a.id < b.id;
}
bool cmp_l(const path_msgs::Lane a, const path_msgs::Lane b)
{
    return a.id < b.id;
}
class global_plan {
private:
    ros::Subscriber sub_startPose, sub_endPose;
    ros::Publisher pub_path, pub_vis_path, pub_debug_path, pub_arrow_array;
    ros::Publisher pub_marker_start, pub_marker_end;
    bool is_startPose_set, is_endPose_set;

    ros::Publisher pub_car_model;
    bool is_simulate_car;

    std::string path;
    bool if_debug;

    double weight_data;
    double weight_smooth;
    double tolerance;

    double start_length;
    double end_length;

    double lane_speed_limit_;
    double cross_speed_limit_;

    std::vector<path_msgs::Lane> Lane_vec, temp_Lane_vec;
    std::vector<path_msgs::Cross> Cross_vec, temp_Cross_vec;
    visualization_msgs::Marker marker_start, marker_end;
    visualization_msgs::Marker marker_car;
    visualization_msgs::MarkerArray marker_arrow_array, marker_arrow;

    std::vector<int> result;
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path vis_path;
    smartcar_msgs::Lane result_path;

    void constract_Lane_Cross_vec(const std::string path);
    void getAllFiles(const std::string path, std::vector<std::string>& files);
    void readAllFiles(std::vector<std::string> files);
    std::vector<int> get_line_nums(std::string line);

    geometry_msgs::PoseStamped start_pose, end_pose;
    void startPose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void endPose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

    double find_nearest_pose(int& lane_id, int& point_index, const geometry_msgs::PoseStamped pose, const std::vector<path_msgs::Lane> path_list);
    void Quaternion2Euler(const geometry_msgs::Pose pose, double* roll, double* pitch, double* yaw);
    void getPathYaw(int lane_id, int point_index, double* yaw);

    void Dij(std::vector<int>* result);

    void convert_result_to_path(smartcar_msgs::Lane* result_path);

    smartcar_msgs::Lane smooth_path(smartcar_msgs::Lane path);

    void smooth(smartcar_msgs::Lane& path, int start_index, int length, int step, double weight_data, double weight_smooth, double tolerance);

    void marker_initial();
    void cfgCB(const global_planning::GlobalPlanningConfig& config, uint32_t level);
    bool validate(double dist, geometry_msgs::PoseStamped pose, std::vector<path_msgs::Cross> path_list);

public:
    global_plan() {}
    ~global_plan() {}

    void init();
};
} // namespace GLOBAL_PLANNER

namespace GLOBAL_PLANNER {
void global_plan::init()
{
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    is_startPose_set = false;
    is_endPose_set = false;

    vis_path.header.frame_id = "map";

    pnh.param<std::string>("file_path", path, "None");
    pnh.param<bool>("debug", if_debug, true);
    pnh.param<double>("weight_data", weight_data, 0.47);
    pnh.param<double>("weight_smooth", weight_smooth, 0.2);
    pnh.param<double>("tolerance", tolerance, 0.2);
    pnh.param<bool>("Visualize_Simulation", is_simulate_car, false);
    pnh.param<double>("lane_speed_limit", lane_speed_limit_, 3.0);
    pnh.param<double>("cross_speed_limit", cross_speed_limit_, 1.0);
    pnh.param<double>("start_length", start_length, 5.0);
    pnh.param<double>("end_length", end_length, 5.0);

    marker_initial();

    constract_Lane_Cross_vec(path);

    sub_startPose = nh.subscribe("/initialpose", 1, &GLOBAL_PLANNER::global_plan::startPose_cb, this);
    sub_endPose = nh.subscribe("/move_base_simple/goal", 1, &GLOBAL_PLANNER::global_plan::endPose_cb, this);
    pub_path = nh.advertise<smartcar_msgs::Lane>("global_path", 1);
    pub_vis_path = nh.advertise<nav_msgs::Path>("/vis_global_path", 1);
    pub_debug_path = nh.advertise<nav_msgs::Path>("debug_path", 1);
    pub_marker_start = nh.advertise<visualization_msgs::Marker>("/marker/start_pose", 1);
    pub_marker_end = nh.advertise<visualization_msgs::Marker>("/marker/end_pose", 1);
    pub_car_model = nh.advertise<visualization_msgs::Marker>("/marker/Car_model", 10);
    pub_arrow_array = nh.advertise<visualization_msgs::MarkerArray>("/arrows", 10);

    dynamic_reconfigure::Server<global_planning::GlobalPlanningConfig> cfg_server;
    dynamic_reconfigure::Server<global_planning::GlobalPlanningConfig>::CallbackType cfg_callback = boost::bind(&global_plan::cfgCB, this, _1, _2);
    cfg_server.setCallback(cfg_callback);

    ros::spin();
}

void global_plan::constract_Lane_Cross_vec(const std::string path)
{
    std::vector<std::string> files;
    getAllFiles(path, files);
    readAllFiles(files);
}

// refer: https://blog.csdn.net/u012005313/article/details/50687297  C++读取文件夹下的文件
void global_plan::getAllFiles(const std::string path, std::vector<std::string>& files)
{
    DIR* dir;
    struct dirent* ptr;
    char base[1000];
    if ((dir = opendir(path.c_str())) == NULL) {
        perror("Open dir error...");
        std::cout << "Check: " << path << std::endl;
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL) {
        if (ptr->d_type == 8)
            files.push_back(ptr->d_name);
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
}

void global_plan::readAllFiles(std::vector<std::string> files)
{
    for (auto file : files) {
        std::string a_file = path + file;
        // std::fstream f(a_file.c_str(), std::ios::in);
        std::FILE* fp = std::fopen(a_file.c_str(), "r");
        if (if_debug)
            std::cout << "read in: " << file << std::endl;
        if (file.find("lane") == 0) {
            // std::cout << "read in: " << file << std::endl;
            path_msgs::Lane new_lane;
            // f >> new_lane.id;
            // f >> new_lane.length;
            std::fscanf(fp, "%d", &new_lane.id);
            std::fscanf(fp, "%f", &new_lane.length);
            int reverse;
            std::fscanf(fp, "%d", &reverse);
            if (reverse == 1) {
                new_lane.reverse = true;
            } else {
                new_lane.reverse = false;
            }
            std::string pre_id_list, next_id_list;
            // f >> pre_id_list >> next_id_list;
            pre_id_list.resize(10);
            next_id_list.resize(10);
            std::fscanf(fp, "%s", &pre_id_list[0]);
            std::fscanf(fp, "%s", &next_id_list[0]);
            // std::cout << pre_id_list << std::endl;
            new_lane.pre_id = get_line_nums(pre_id_list);
            new_lane.next_id = get_line_nums(next_id_list);
            // while(!f.eof()){
            while (!std::feof(fp)) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                std::fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z,
                    &pose.pose.orientation.x, &pose.pose.orientation.y, &pose.pose.orientation.z, &pose.pose.orientation.w);
                new_lane.path.poses.push_back(pose);
            }
            // for(auto i : new_lane.pre_id) std::cout << i << " ";
            // std::cout << std::endl;
            // for(auto i : new_lane.next_id) std::cout << i << " ";
            // std::cout << std::endl;
            // std::cout << "path size = " << new_lane.path.poses.size() << std::endl;
            new_lane.path.poses.pop_back();
            Lane_vec.push_back(new_lane);
        } else if (file.find("cross") == 0) {
            // std::cout << "read in: " << file << std::endl;
            path_msgs::Cross new_lane;
            // f >> new_lane.id;
            // f >> new_lane.length;
            std::fscanf(fp, "%d", &new_lane.id);
            std::fscanf(fp, "%f", &new_lane.length);
            int reverse;
            std::fscanf(fp, "%d", &reverse);
            if (reverse) {
                new_lane.reverse = true;
            } else {
                new_lane.reverse = false;
            }
            std::string pre_id_list, next_id_list;
            // f >> pre_id_list >> next_id_list;
            pre_id_list.resize(10);
            next_id_list.resize(10);
            std::fscanf(fp, "%s", &pre_id_list[0]);
            std::fscanf(fp, "%s", &next_id_list[0]);
            // std::cout << pre_id_list << std::endl;
            new_lane.pre_id = get_line_nums(pre_id_list);
            new_lane.next_id = get_line_nums(next_id_list);
            // while(!f.eof()){
            while (!std::feof(fp)) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                std::fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z,
                    &pose.pose.orientation.x, &pose.pose.orientation.y, &pose.pose.orientation.z, &pose.pose.orientation.w);
                new_lane.path.poses.push_back(pose);
                // std::cout << pose.pose.position.x << " "
                //           << pose.pose.position.y << std::endl;
            }
            // for(auto i : new_lane.pre_id) std::cout << i << " ";
            // std::cout << std::endl;
            // for(auto i : new_lane.next_id) std::cout << i << " ";
            // std::cout << std::endl;
            // std::cout << "path size = " << new_lane.path.poses.size() << std::endl;
            new_lane.path.poses.pop_back();
            Cross_vec.push_back(new_lane);
        }

        // f.close();
        std::fclose(fp);
    }
    std::sort(Cross_vec.begin(), Cross_vec.end(), cmp_c);
    std::sort(Lane_vec.begin(), Lane_vec.end(), cmp_l);
    if (if_debug) {
        std::cout << "read in Lane.vec  with " << Lane_vec.size() << " lanes" << std::endl;
        std::cout << "read in Cross.vec with " << Cross_vec.size() << " crosses" << std::endl;
    }
    return;
}

std::vector<int> global_plan::get_line_nums(std::string line)
{
    std::vector<int> res;
    int size = line.size();
    int num = 0;
    for (int i = 0; line.c_str()[i] != '\0'; i++) {
        char a = line.c_str()[i];
        // std::cout << "a = " << a << std::endl;
        if (a != ',') {
            num = num * 10 + int(a) - int('0');
        } else {
            res.push_back(num);
            num = 0;
        }
    }
    res.push_back(num);
    return res;
}

void global_plan::startPose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    ROS_INFO_STREAM("Start Pose Handler");
    marker_start.pose = msg->pose.pose;
    pub_marker_start.publish(marker_start);

    start_pose.pose = msg->pose.pose;
    temp_Lane_vec.clear();
    temp_Lane_vec.assign(Lane_vec.begin(), Lane_vec.end());
    temp_Cross_vec.clear();
    temp_Cross_vec.assign(Cross_vec.begin(), Cross_vec.end());

    int lane_id, point_index;
    double min_dist = find_nearest_pose(lane_id, point_index, start_pose, temp_Lane_vec);
    if (validate(min_dist, start_pose, temp_Cross_vec) == false) {
        ROS_WARN_STREAM("Target position cannot be put at cornor, please confirm again.");
        return;
    }

    double start_roll, start_pitch, start_yaw;
    Quaternion2Euler(start_pose.pose, &start_roll, &start_pitch, &start_yaw);
    double path_yaw;
    getPathYaw(lane_id, point_index, &path_yaw);
    path_msgs::Cross c_node, c_node_final;
    path_msgs::Lane l_node, l_node_final;
    c_node.path.header.frame_id = l_node.path.header.frame_id = "map";
    c_node_final.path.header.frame_id = l_node_final.path.header.frame_id = "map";
    geometry_msgs::PoseStamped temp_start_pose;
    temp_start_pose.pose.position = temp_Lane_vec[lane_id].path.poses[point_index].pose.position;
    c_node.path.poses.push_back(temp_start_pose);
    c_node_final.path.poses.push_back(temp_start_pose);

    path_msgs::Lane t_lane = temp_Lane_vec[lane_id];
    if (std::abs(start_yaw - path_yaw) < M_PI / 2) {
        std::cout << "same orientation" << std::endl;
        int traj_size = t_lane.path.poses.size();

        c_node_final.id = temp_Cross_vec.size();
        c_node_final.length = 0;
        c_node_final.reverse = false;
        c_node_final.pre_id.push_back(temp_Lane_vec.size());
        temp_Cross_vec.push_back(c_node_final);

        l_node_final.id = temp_Lane_vec.size();
        l_node_final.length = std::sqrt(std::pow(t_lane.path.poses[0].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[0].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
        l_node_final.reverse = false;
        l_node_final.pre_id.assign(t_lane.pre_id.begin(), t_lane.pre_id.end());
        l_node_final.next_id.push_back(c_node_final.id);
        for (int i = 0; i < point_index; i++) {
            l_node_final.path.poses.push_back(t_lane.path.poses[i]);
        }
        temp_Lane_vec.push_back(l_node_final);

        l_node.id = temp_Lane_vec.size();
        l_node.length = std::sqrt(std::pow(t_lane.path.poses[traj_size - 1].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[traj_size - 1].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
        l_node.reverse = false;
        l_node.pre_id.push_back(temp_Cross_vec.size());
        l_node.next_id.assign(t_lane.next_id.begin(), t_lane.next_id.end());
        for (int i = point_index; i < traj_size; i++) {
            l_node.path.poses.push_back(t_lane.path.poses[i]);
        }
        for (size_t cross_id : t_lane.pre_id) {
            // std::cout << "Handle pre cross_id " << cross_id << std::endl;
            for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del pre_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].next_id.erase(it);
                    temp_Cross_vec[cross_id].next_id.push_back(l_node_final.id);
                    break;
                }
            }
            for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del pre_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.erase(it);
                    temp_Cross_vec[cross_id].pre_id.push_back(l_node_final.id);
                    break;
                }
            }
        }
        for (size_t cross_id : t_lane.next_id) {
            // std::cout << "Handle next cross_id " << cross_id << std::endl;
            for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del next_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].next_id.erase(it);
                    std::cout << "Add next_cross " << cross_id << ".next_id with " << l_node.id << std::endl;
                    temp_Cross_vec[cross_id].next_id.push_back(l_node.id);
                    break;
                }
            }
            for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del next_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.erase(it);
                    std::cout << "Add next_cross " << cross_id << ".pre_id with " << l_node.id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.push_back(l_node.id);
                    break;
                }
            }
        }
        // pub_path.publish(l_node.path);

        c_node.id = temp_Cross_vec.size();
        c_node.length = 0;
        c_node.reverse = false;
        c_node.next_id.push_back(l_node.id);

        temp_Cross_vec.push_back(c_node);
        temp_Lane_vec.push_back(l_node);
    } else {
        std::cout << "different orientation" << std::endl;
        int traj_size = t_lane.path.poses.size();

        c_node_final.id = temp_Cross_vec.size();
        c_node_final.length = 0;
        c_node_final.reverse = false;
        c_node_final.pre_id.push_back(temp_Lane_vec.size());
        temp_Cross_vec.push_back(c_node_final);

        l_node_final.id = temp_Lane_vec.size();
        l_node_final.length = std::sqrt(std::pow(t_lane.path.poses[traj_size - 1].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[traj_size - 1].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
        l_node_final.reverse = false;
        l_node_final.pre_id.assign(t_lane.next_id.begin(), t_lane.next_id.end());
        l_node_final.next_id.push_back(c_node_final.id);
        for (int i = traj_size - 1; i > point_index; i--) {
            l_node_final.path.poses.push_back(t_lane.path.poses[i]);
        }
        temp_Lane_vec.push_back(l_node_final);

        l_node.id = temp_Lane_vec.size();
        l_node.length = std::sqrt(std::pow(t_lane.path.poses[0].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[0].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
        l_node.reverse = false;
        l_node.pre_id.push_back(temp_Cross_vec.size());
        l_node.next_id.assign(t_lane.pre_id.begin(), t_lane.pre_id.end());
        for (int i = point_index; i >= 0; i--) {
            l_node.path.poses.push_back(t_lane.path.poses[i]);
        }

        for (size_t cross_id : t_lane.pre_id) {
            // std::cout << "Handle pre cross_id " << cross_id << std::endl;
            for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del pre_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].next_id.erase(it);
                    std::cout << "Add pre_cross " << cross_id << ".next_id with " << l_node.id << std::endl;
                    temp_Cross_vec[cross_id].next_id.push_back(l_node.id);
                    break;
                }
            }
            for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del pre_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.erase(it);
                    std::cout << "Add pre_cross " << cross_id << ".pre_id with " << l_node.id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.push_back(l_node.id);
                    break;
                }
            }
        }

        for (size_t cross_id : t_lane.next_id) {
            // std::cout << "Handle next cross_id " << cross_id << std::endl;
            for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del next_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].next_id.erase(it);
                    temp_Cross_vec[cross_id].next_id.push_back(l_node_final.id);
                    break;
                }
            }
            for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
                if (*it == lane_id) {
                    std::cout << "Del next_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                    temp_Cross_vec[cross_id].pre_id.erase(it);
                    temp_Cross_vec[cross_id].pre_id.push_back(l_node_final.id);
                    break;
                }
            }
        }

        c_node.id = temp_Cross_vec.size();
        c_node.length = 0;
        c_node.reverse = false;
        c_node.next_id.push_back(l_node.id);

        temp_Cross_vec.push_back(c_node);
        temp_Lane_vec.push_back(l_node);
    }

    is_startPose_set = true;
    temp_Lane_vec[lane_id].next_id.clear();
    temp_Lane_vec[lane_id].pre_id.clear();
    temp_Lane_vec[lane_id].path.poses.clear();
    std::cout << "Start Pose Seted " << std::endl;
    std::cout << "------------------------" << std::endl;
}

void global_plan::endPose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO_STREAM("Target Pose Handler");
    if (util::distance2points(start_pose.pose.position, msg->pose.position) < 20) {
        ROS_WARN("Too close, please walk there");
        return;
    }
    marker_end.pose = msg->pose;
    pub_marker_end.publish(marker_end);

    if (!is_startPose_set) {
        ROS_WARN_STREAM("Please set start pose first.");
        return;
    }
    end_pose.pose = msg->pose;
    int lane_id, point_index;
    double min_dist = find_nearest_pose(lane_id, point_index, end_pose, temp_Lane_vec);
    if (validate(min_dist, end_pose, temp_Cross_vec) == false) {
        ROS_WARN_STREAM("Target position cannot be put at cornor, please confirm again.");
        return;
    }

    path_msgs::Cross c_node;
    path_msgs::Lane l_node1, l_node2;
    c_node.path.header.frame_id = l_node1.path.header.frame_id = l_node2.path.header.frame_id = "map";
    path_msgs::Lane t_lane = temp_Lane_vec[lane_id];

    // for the left(backward) direction
    l_node1.id = temp_Lane_vec.size();
    l_node1.length = std::sqrt(std::pow(t_lane.path.poses[0].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[0].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
    l_node1.reverse = false;
    l_node1.pre_id.assign(t_lane.pre_id.begin(), t_lane.pre_id.end());
    // std::cout << "l_node1.id = " << l_node1.id << std::endl;
    // std::cout << "pre " << t_lane.pre_id[0] << std::endl;
    // std::cout << "t_lane.id = " << lane_id << std::endl;
    l_node1.next_id.push_back(temp_Cross_vec.size());
    for (int i = 0; i < point_index; i++) {
        l_node1.path.poses.push_back(t_lane.path.poses[i]);
    }
    for (size_t cross_id : t_lane.pre_id) {
        // std::cout << "Handle pre cross_id " << cross_id << std::endl;
        for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
            if (*it == lane_id) {
                std::cout << "Del pre_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                temp_Cross_vec[cross_id].next_id.erase(it);
                std::cout << "Add pre_cross " << cross_id << ".next_id with " << l_node1.id << std::endl;
                temp_Cross_vec[cross_id].pre_id.push_back(l_node1.id);
                break;
            }
        }
        for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
            if (*it == lane_id) {
                std::cout << "Del pre_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                temp_Cross_vec[cross_id].pre_id.erase(it);
                std::cout << "Add pre_cross " << cross_id << ".pre_id with " << l_node1.id << std::endl;
                temp_Cross_vec[cross_id].pre_id.push_back(l_node1.id);
                break;
            }
        }
    }
    temp_Lane_vec.push_back(l_node1);
    // pub_path.publish(l_node1.path);

    // for the right(forward) direction
    int traj_size = t_lane.path.poses.size();
    l_node2.id = temp_Lane_vec.size();
    l_node2.length = std::sqrt(std::pow(t_lane.path.poses[traj_size - 1].pose.position.y - t_lane.path.poses[point_index].pose.position.y, 2) + std::pow(t_lane.path.poses[traj_size - 1].pose.position.x - t_lane.path.poses[point_index].pose.position.x, 2));
    l_node2.reverse = false;
    l_node2.pre_id.assign(t_lane.next_id.begin(), t_lane.next_id.end());
    l_node2.next_id.push_back(temp_Cross_vec.size());
    for (int i = t_lane.path.poses.size() - 1; i >= point_index; i--) {
        l_node2.path.poses.push_back(t_lane.path.poses[i]);
    }
    for (size_t cross_id : t_lane.next_id) {
        // std::cout << "Handle pre cross_id " << cross_id << std::endl;
        for (auto it = temp_Cross_vec[cross_id].next_id.begin(); it != temp_Cross_vec[cross_id].next_id.end(); it++) {
            if (*it == lane_id) {
                std::cout << "Del next_cross " << cross_id << ".next_id with " << lane_id << std::endl;
                temp_Cross_vec[cross_id].next_id.erase(it);
                std::cout << "Add next_cross " << cross_id << ".next_id with " << l_node2.id << std::endl;
                temp_Cross_vec[cross_id].pre_id.push_back(l_node2.id);
                break;
            }
        }
        for (auto it = temp_Cross_vec[cross_id].pre_id.begin(); it != temp_Cross_vec[cross_id].pre_id.end(); it++) {
            if (*it == lane_id) {
                std::cout << "Del next_cross " << cross_id << ".pre_id with " << lane_id << std::endl;
                temp_Cross_vec[cross_id].pre_id.erase(it);
                std::cout << "Add next_cross " << cross_id << ".pre_id with " << l_node2.id << std::endl;
                temp_Cross_vec[cross_id].pre_id.push_back(l_node2.id);
                break;
            }
        }
    }

    temp_Lane_vec.push_back(l_node2);
    // pub_path.publish(l_node2.path);

    c_node.id = temp_Cross_vec.size();
    c_node.length = 0;
    c_node.reverse = false;
    c_node.pre_id.push_back(l_node1.id);
    c_node.pre_id.push_back(l_node2.id);
    // std::cout << "c_node.pre_id = " << std::endl;
    // for(auto p: c_node.pre_id) std::cout << "pre " << p << std::endl;
    // std::cout << l_node1.id << std::endl;
    // std::cout << l_node2.id << std::endl;
    temp_Cross_vec.push_back(c_node);
    temp_Lane_vec[lane_id].pre_id.clear();
    temp_Lane_vec[lane_id].next_id.clear();
    temp_Lane_vec[lane_id].path.poses.clear();

    std::cout << "Target Pose Seted " << std::endl;
    std::cout << "------------------------" << std::endl;
    // if(if_debug){
    //     pub_path.publish(l_node1.path);
    //     ros::Duration(1.0).sleep();
    //     pub_path.publish(l_node2.path);
    //     for(int i = 0; i < temp_Lane_vec.size(); i++){
    //         std::cout << "lane " << i << " .length = " << temp_Lane_vec[i].length << std::endl;
    //     }
    // }

    result.clear();
    Dij(&result);

    // nav_msgs::Path result_path;
    // result_path.header.frame_id = "map";
    // convert_result_to_path(&result_path);

    // smooth_path(result_path);
    // // show_result(result_path);
    // pub_path.publish(result_path);

    result_path.waypoints.clear();
    result_path.header.frame_id = "map";
    convert_result_to_path(&result_path);
    smartcar_msgs::Lane res_path = smooth_path(result_path);
    res_path.header.frame_id = "map";
    pub_path.publish(res_path);

    if (is_simulate_car) {
        int cnt = 1;
        for (auto p : result_path.waypoints) {
            tf::Quaternion tmp_q;
            tf::quaternionMsgToTF(p.pose.pose.orientation, tmp_q);
            tf::Transform transform2(tmp_q, tf::Vector3(p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z));
            tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "simu_car"));

            marker_car.header.stamp = ros::Time::now();
            marker_car.pose.position = p.pose.pose.position;
            double current_roll, current_yaw, current_pitch;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(p.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);
            marker_car.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(90 * (M_PI / 180.0),
                0 * (M_PI / 180.0),
                current_yaw + M_PI / 2.0);
            pub_car_model.publish(marker_car);
            ros::Duration(0.05).sleep();
        }
    }

    is_startPose_set = false;
}

double global_plan::find_nearest_pose(int& lane_id, int& point_index, const geometry_msgs::PoseStamped pose, const std::vector<path_msgs::Lane> path_list)
{
    int t_lane_id, t_point_index;
    double min_length = 9999;
    for (size_t i = 0; i < path_list.size(); i++) {
        for (size_t j = 0; j < path_list[i].path.poses.size(); j++) {
            double t = util::distance2points(pose.pose.position, path_list[i].path.poses[j].pose.position);
            if (t < min_length) {
                t_lane_id = path_list[i].id;
                t_point_index = j;
                min_length = t;
            }
        }
    }
    lane_id = t_lane_id;
    point_index = t_point_index;
    if (if_debug) {
        std::cout << "Nearest lane_id     is " << lane_id << std::endl;
        std::cout << "Nearest point_index is " << point_index << std::endl;
    }
    return min_length;
}

// 四元数转欧拉角
void global_plan::Quaternion2Euler(const geometry_msgs::Pose pose, double* roll, double* pitch, double* yaw)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);
}

void global_plan::getPathYaw(int lane_id, int point_index, double* yaw)
{
    double x1 = Lane_vec[lane_id].path.poses[point_index].pose.position.x;
    double y1 = Lane_vec[lane_id].path.poses[point_index].pose.position.y;
    double x2 = Lane_vec[lane_id].path.poses[point_index + 1].pose.position.x;
    double y2 = Lane_vec[lane_id].path.poses[point_index + 1].pose.position.y;
    *yaw = std::atan2(y2 - y1, x2 - x1);
}

void global_plan::Dij(std::vector<int>* result)
{
    result->clear();

    int c_size = temp_Cross_vec.size();
    int l_size = temp_Lane_vec.size();
    int graph[c_size][c_size];
    for (int i = 0; i < c_size; i++)
        std::fill(graph[i], graph[i] + c_size, -1);
    for (int i = 0; i < l_size; i++) {
        for (auto a : temp_Lane_vec[i].pre_id) {
            for (auto b : temp_Lane_vec[i].next_id) {
                graph[a][b] = temp_Lane_vec[i].id;
                if (temp_Lane_vec[i].reverse) {
                    graph[b][a] = temp_Lane_vec[i].id;
                }
                // std::cout << a << " - " << b << " : " << temp_Lane_vec[i].id << std::endl;
            }
        }
    }

    if (if_debug) {
        std::cout << "------------------------" << std::endl;
        std::cout << ">> Search Graph " << std::endl;
        std::printf("%02d ", c_size);
        for (int i = 0; i < c_size; i++) {
            std::printf("%02d ", i);
        }
        std::cout << std::endl;
        for (int i = 0; i < c_size; i++) {
            std::printf("%02d ", i);
            for (int j = 0; j < c_size; j++) {
                // std::cout << graph[i][j] << " ";
                if (graph[i][j] == -1)
                    std::printf("   ");
                else
                    std::printf("%2d ", graph[i][j]);
            }
            std::cout << std::endl;
        }
        std::cout << "------------------------" << std::endl;
    }

    // TODO :: Dijkstra
    int s = c_size - 2;
    int t = c_size - 1;

    double dist[c_size];
    std::fill(dist, dist + c_size, DBL_MAX);
    dist[s] = 0;
    bool visit[c_size];
    std::fill(visit, visit + c_size, false);
    int pre[c_size];
    for (size_t i = 0; i < c_size; i++)
        pre[i] = i;

    for (size_t i = 0; i < c_size; i++) {
        // std::cout << "Handle " << i << std::endl;
        int u = -1;
        double MIN = DBL_MAX;
        for (int j = 0; j < c_size; j++) {
            if (!visit[j] && dist[j] < MIN) {
                u = j;
                MIN = dist[j];
            }
        }
        if (u == -1 || u == t)
            break;
        visit[u] = true;
        for (int v = 0; v < c_size; v++) {
            if (!visit[v] && graph[u][v] != -1) {
                double length = temp_Lane_vec[graph[u][v]].length;
                if (dist[u] + length < dist[v]) {
                    dist[v] = dist[u] + length;
                    pre[v] = u;
                }
            }
        }
    }
    std::cout << "total length: " << dist[t] << std::endl;
    std::vector<int> cross_list;
    // for(int i=0;i<c_size;i++){
    //     std::cout << "pre " << i
    //               << " is " << pre[i]
    //               << std::endl;
    // }
    int n = t;
    while (pre[n] != n) {
        // std::cout << n << std::endl;
        cross_list.push_back(n);
        n = pre[n];
    }
    cross_list.push_back(s);
    std::reverse(cross_list.begin(), cross_list.end());
    if (if_debug) {
        std::cout << ">> The Cross Path" << std::endl;
        for (int i = 0; i < cross_list.size(); i++) {
            std::cout << cross_list[i];
            if (i < cross_list.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
        std::cout << "------------------------" << std::endl;
    }

    for (size_t i = 0; i < cross_list.size(); i++) {
        int u = cross_list[i];
        result->push_back(u);
        if (i < cross_list.size() - 1) {
            int v = cross_list[i + 1];
            result->push_back(graph[u][v]);
        }
    }
    // for(auto p : *result) std::cout << p << " = ";
    // std::cout << std::endl;
}

void global_plan::convert_result_to_path(smartcar_msgs::Lane* result_path)
{
    int size = result.size();
    smartcar_msgs::Waypoint waypoint_start;
    waypoint_start.pose.pose.position = temp_Cross_vec[result[0]].path.poses[0].pose.position;
    waypoint_start.is_lane = 1; // waypoint_start实际上是起始点，存在于temp_Cross_vec，但是里面只有一个点，所以实际使用时归到lane中
    waypoint_start.speed_limit = lane_speed_limit_;
    result_path->waypoints.push_back(waypoint_start);
    double speed_limit;
    int waypoint_type;

    for (int i = 1; i < size - 1; i++) {
        smartcar_msgs::Waypoint pre = result_path->waypoints[result_path->waypoints.size() - 1];
        int id = result[i];
        nav_msgs::Path next;
        next.header.frame_id = "map";
        if (i % 2 == 1) { // lane
            next = temp_Lane_vec[id].path;
            speed_limit = lane_speed_limit_;
            waypoint_type = 1;
        } else { // cross
            next = temp_Cross_vec[id].path;
            speed_limit = cross_speed_limit_;
            waypoint_type = 0;
        }
        geometry_msgs::PoseStamped p_start = next.poses[0];
        geometry_msgs::PoseStamped p_last = next.poses[next.poses.size() - 1];

        if (util::distance2points(pre.pose.pose.position, p_start.pose.position) > util::distance2points(pre.pose.pose.position, p_last.pose.position)) {
            std::reverse(next.poses.begin(), next.poses.end());
        }
        for (auto point : next.poses) {
            smartcar_msgs::Waypoint in_waypoint;
            in_waypoint.is_lane = waypoint_type;
            in_waypoint.speed_limit = speed_limit;
            in_waypoint.pose.pose = point.pose;
            result_path->waypoints.push_back(in_waypoint);
        }
    }
}

smartcar_msgs::Lane global_plan::smooth_path(smartcar_msgs::Lane path)
{
    smartcar_msgs::Lane path_in;
    path_in.waypoints.clear();
    path_in.waypoints.assign(path.waypoints.begin(), path.waypoints.end());
    int size = path_in.waypoints.size();
    if (path_in.waypoints.size() <= 10) {
        //cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
        return path;
    }

    std::vector<smartcar_msgs::Waypoint>::iterator it_front, it_end;
    double th_length_front = 8.0, th_length_end = 8.0;
    double sum_length_front = 0, sum_length_end = 0;
    std::vector<smartcar_msgs::Waypoint> in_front, in_end; // insert waypoints

    // handle front end
    for (it_front = path_in.waypoints.begin(); it_front != path_in.waypoints.end() - 2; it_front++) {
        sum_length_front += util::distance2points((*it_front).pose.pose.position, (*(it_front + 1)).pose.pose.position);
        if (sum_length_front >= th_length_front)
            break;
    }
    if (sum_length_front >= th_length_front) {
        std::vector<smartcar_msgs::Waypoint> lane_front;
        lane_front.assign(path_in.waypoints.begin(), it_front);
        path_in.waypoints.erase(path_in.waypoints.begin(), it_front);

        // vis_path.poses.clear();
        // for(auto p:lane_front){
        //     geometry_msgs::PoseStamped pose;
        //     pose.pose = p.pose.pose;
        //     vis_path.poses.push_back(pose);
        // }
        // pub_vis_path.publish(vis_path);

        double diff_x = start_pose.pose.position.x - lane_front[0].pose.pose.position.x;
        double diff_y = start_pose.pose.position.y - lane_front[0].pose.pose.position.y;
        double sum_front = 0;
        int front_index = 0;
        for (int i = 0; i < lane_front.size() - 2; i++) {
            sum_front += util::distance2points(lane_front[i].pose.pose.position, lane_front[i + 1].pose.pose.position);
            if (sum_front >= start_length) {
                front_index = i;
                break;
            }
        }

        for (int i = 0; i <= front_index; i++) {
            smartcar_msgs::Waypoint temp;
            temp.is_lane = 1;
            temp.speed_limit = 0.5;
            temp.pose.pose.position.x = lane_front[i].pose.pose.position.x + diff_x;
            temp.pose.pose.position.y = lane_front[i].pose.pose.position.y + diff_y;
            in_front.push_back(temp);
        }

        smartcar_msgs::Waypoint s = in_front[front_index];
        double lenx = (*path_in.waypoints.begin()).pose.pose.position.x - s.pose.pose.position.x;
        double leny = (*path_in.waypoints.begin()).pose.pose.position.y - s.pose.pose.position.y;
        int cnt = int(util::distance2points(s.pose.pose.position, (*path_in.waypoints.begin()).pose.pose.position) / 0.5);
        double dx = lenx / cnt;
        double dy = leny / cnt;
        for (int i = 1; i < cnt; i++) {
            smartcar_msgs::Waypoint temp;
            temp.is_lane = 1;
            temp.speed_limit = 1.0; // 缓慢起步
            temp.pose.pose.position.x = s.pose.pose.position.x + i * dx;
            temp.pose.pose.position.y = s.pose.pose.position.y + i * dy;
            in_front.push_back(temp);
        }
        path_in.waypoints.insert(path_in.waypoints.begin(), in_front.begin(), in_front.end());
    }

    // handle target end
    for (it_end = path_in.waypoints.end() - 1; it_end != path_in.waypoints.begin() + 1; it_end--) {
        sum_length_end += util::distance2points((*(it_end - 1)).pose.pose.position, (*it_end).pose.pose.position);
        if (sum_length_end >= th_length_end)
            break;
    }
    if (sum_length_end >= th_length_end) {
        std::vector<smartcar_msgs::Waypoint> lane_end;
        lane_end.assign(it_end, path_in.waypoints.end());
        path_in.waypoints.erase(it_end, path_in.waypoints.end());

        double diff_x = end_pose.pose.position.x - lane_end[lane_end.size() - 1].pose.pose.position.x;
        double diff_y = end_pose.pose.position.y - lane_end[lane_end.size() - 1].pose.pose.position.y;
        double sum_end = 0;
        int end_index = 0;
        for (int i = lane_end.size() - 1; i > 0; i--) {
            sum_end += util::distance2points(lane_end[i].pose.pose.position, lane_end[i - 1].pose.pose.position);
            if (sum_end >= end_length) {
                end_index = i;
                break;
            }
        }

        for (int i = lane_end.size() - 1; i > end_index; i--) {
            smartcar_msgs::Waypoint temp;
            temp.is_lane = 1;
            temp.speed_limit = 0.5;
            temp.pose.pose.position.x = lane_end[i].pose.pose.position.x + diff_x;
            temp.pose.pose.position.y = lane_end[i].pose.pose.position.y + diff_y;
            in_end.push_back(temp);
        }

        smartcar_msgs::Waypoint e = in_end[in_end.size() - 1];
        double lenx = e.pose.pose.position.x - (*(path_in.waypoints.end() - 1)).pose.pose.position.x;
        double leny = e.pose.pose.position.y - (*(path_in.waypoints.end() - 1)).pose.pose.position.y;
        int cnt = int(util::distance2points(e.pose.pose.position, (*(path_in.waypoints.end() - 1)).pose.pose.position) / 0.5);
        double dx = lenx / cnt;
        double dy = leny / cnt;
        for (int i = cnt; i > 0; i--) {
            smartcar_msgs::Waypoint temp;
            temp.is_lane = 1;
            temp.speed_limit = 1.0;
            temp.pose.pose.position.x = e.pose.pose.position.x - (cnt - i) * dx;
            temp.pose.pose.position.y = e.pose.pose.position.y - (cnt - i) * dy;
            in_end.push_back(temp);
        }
        std::reverse(in_end.begin(), in_end.end());
        path_in.waypoints.insert(path_in.waypoints.end(), in_end.begin(), in_end.end());
    }

    size = path_in.waypoints.size();
    // std::vector<int> count;
    // count.resize(size, 0);

    smartcar_msgs::Lane smoothPath_out;
    smoothPath_out.waypoints.clear();
    smoothPath_out.waypoints.assign(path_in.waypoints.begin(), path_in.waypoints.end());

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    // Smooth 函数
    if (size > 30) {
        smooth(smoothPath_out, 0, 40, 2, weight_data, weight_smooth, tolerance);
        smooth(smoothPath_out, size - 40, 40, 2, weight_data, weight_smooth, tolerance);
    } else {
        smooth(smoothPath_out, 0, size, 2, weight_data, weight_smooth, tolerance);
    }

    // 调整每个轨迹点的角度
    for (size_t i = 0; i < smoothPath_out.waypoints.size() - 1; i++) {
        smartcar_msgs::Waypoint p_c = smoothPath_out.waypoints[i];
        smartcar_msgs::Waypoint p_n = smoothPath_out.waypoints[i + 1];
        double yaw = std::atan2(p_n.pose.pose.position.y - p_c.pose.pose.position.y, p_n.pose.pose.position.x - p_c.pose.pose.position.x);
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
        smoothPath_out.waypoints[i].pose.pose.orientation.x = q.x();
        smoothPath_out.waypoints[i].pose.pose.orientation.y = q.y();
        smoothPath_out.waypoints[i].pose.pose.orientation.z = q.z();
        smoothPath_out.waypoints[i].pose.pose.orientation.w = q.w();
    }

    vis_path.poses.clear();
    for (auto p : smoothPath_out.waypoints) {
        geometry_msgs::PoseStamped vis_p;
        vis_p.pose = p.pose.pose;
        vis_path.poses.push_back(vis_p);
    }
    // pub_vis_path.publish(vis_path);
    marker_arrow_array.markers.clear();
    for (int i = 0; i < smoothPath_out.waypoints.size(); i++) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "map";
        // arrow.header.stamp = ros::Time::now();
        arrow.id = i + 1;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.scale.x = 0.4;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.2;
        arrow.color.r = 0;
        arrow.color.g = 0;
        arrow.color.b = 1;
        arrow.color.a = 1;
        arrow.pose = smoothPath_out.waypoints[i].pose.pose;
        marker_arrow_array.markers.push_back(arrow);
    }
    pub_arrow_array.publish(marker_arrow_array);
    return smoothPath_out;
}

void global_plan::marker_initial()
{
    marker_start.header.frame_id = "map";
    marker_start.header.stamp = ros::Time::now();
    marker_start.id = 0;
    marker_start.type = visualization_msgs::Marker::ARROW;
    marker_start.action = visualization_msgs::Marker::ADD;
    marker_start.scale.x = 2.0;
    marker_start.scale.y = 0.5;
    marker_start.scale.z = 0.5;
    marker_start.color.r = 1;
    marker_start.color.g = 0;
    marker_start.color.b = 0;
    marker_start.color.a = 1;
    // marker_start.lifetime = ros::Duration();

    marker_end.header.frame_id = "map";
    marker_end.header.stamp = ros::Time::now();
    marker_end.id = 0;
    marker_end.type = visualization_msgs::Marker::CUBE;
    marker_end.action = visualization_msgs::Marker::ADD;
    marker_end.scale.x = 1.0;
    marker_end.scale.y = 1.0;
    marker_end.scale.z = 1.0;
    marker_end.color.r = 1;
    marker_end.color.g = 0;
    marker_end.color.b = 0;
    marker_end.color.a = 1;
    // marker_end.lifetime = ros::Duration();

    marker_car.header.frame_id = "map";
    marker_car.header.stamp = ros::Time();
    marker_car.ns = "car";
    marker_car.id = 0;
    marker_car.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_car.action = visualization_msgs::Marker::ADD;
    // set marker_car.pose.position marker.pose.orientation
    marker_car.color.r = 1;
    // marker_car.color.g = 1;
    // marker_car.color.b = 1;
    marker_car.color.a = 1;
    marker_car.scale.x = 1.0;
    marker_car.scale.y = 1.0;
    marker_car.scale.z = 1.0;
    marker_car.mesh_use_embedded_materials = true;
    marker_car.mesh_resource = "package://car_model/ferrari/dae.DAE";
}

void global_plan::cfgCB(const global_planning::GlobalPlanningConfig& config, uint32_t level)
{
    ROS_INFO_STREAM("Reconfigure global planning config.");
    weight_data = config.weight_data;
    weight_smooth = config.weight_smooth;
    tolerance = config.tolerance;
    start_length = config.start_length;
    end_length = config.end_length;
    smooth_path(result_path);
}

void global_plan::smooth(smartcar_msgs::Lane& path, int start_index, int length, int step, double weight_data, double weight_smooth, double tolerance)
{
    double change = tolerance;
    int Iterations = 0;
    while (change >= tolerance) {
        change = 0.0;
        for (int i = start_index + step; i < start_index + length - 1 - step; i++) {
            double xtemp = path.waypoints[i].pose.pose.position.x;
            double ytemp = path.waypoints[i].pose.pose.position.y;

            path.waypoints[i].pose.pose.position.x += weight_data * (path.waypoints[i].pose.pose.position.x - path.waypoints[i].pose.pose.position.x);
            path.waypoints[i].pose.pose.position.y += weight_data * (path.waypoints[i].pose.pose.position.y - path.waypoints[i].pose.pose.position.y);

            path.waypoints[i].pose.pose.position.x += weight_smooth * (path.waypoints[i - 1].pose.pose.position.x + path.waypoints[i + 1].pose.pose.position.x - (2.0 * path.waypoints[i].pose.pose.position.x));
            path.waypoints[i].pose.pose.position.y += weight_smooth * (path.waypoints[i - 1].pose.pose.position.y + path.waypoints[i + 1].pose.pose.position.y - (2.0 * path.waypoints[i].pose.pose.position.y));

            change += fabs(xtemp - path.waypoints[i].pose.pose.position.x);
            change += fabs(ytemp - path.waypoints[i].pose.pose.position.y);
        }
        Iterations++;
    }
}

bool global_plan::validate(double dist, geometry_msgs::PoseStamped pose, std::vector<path_msgs::Cross> path_list)
{
    double dist_to_cornor;
    int t_lane_id, t_point_index;
    double min_length = 9999;
    for (size_t i = 0; i < path_list.size(); i++) {
        for (size_t j = 0; j < path_list[i].path.poses.size(); j++) {
            double t = util::distance2points(pose.pose.position, path_list[i].path.poses[j].pose.position);
            if (t < min_length) {
                t_lane_id = path_list[i].id;
                t_point_index = j;
                min_length = t;
            }
        }
    }
    int cross_id = t_lane_id;
    int point_index = t_point_index;
    if (min_length < dist)
        return false;
    return true;
}

} // namespace GLOBAL_PLANNER

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_global_plan");
    GLOBAL_PLANNER::global_plan app;
    app.init();
    return 0;
}
