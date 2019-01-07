// ROS Includes
#include <ros/ros.h>

// User defined includes
#include "pure_persuit.h"

namespace waypoint_follower
{
static float count;
// Constructor
PurePursuitNode::PurePursuitNode() : private_nh_("~"),
                                     LOOP_RATE_(1),
                                     is_waypoint_set_(false),
                                     is_pose_set_(false),
                                     is_velocity_set_(false),
                                     current_linear_velocity_(0),
                                     command_linear_velocity_(0),
                                     CONST_VEL_DIS(true),
                                     const_lookahead_distance_(4.0),
                                     const_velocity_(5.0),
                                     lookahead_distance_ratio_(2.0),
                                     RADIUS_MAX_(9e10),
                                     curvature_MIN_(1 / RADIUS_MAX_),
                                     is_linear_interpolation_(false),
                                     next_waypoint_number_(-1),
                                     lookahead_distance_(0),
                                     minimum_lookahead_distance_(6.0)
{
    // 设置了订阅，发布的话题消息， 读取了launch文件中的配置变量
    initForROS();

    // initialize for PurePursuit
    setLinearInterpolationParameter(is_linear_interpolation_); //true
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
    // ros parameter settings
    private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
    private_nh_.param("wheel_base", wheel_base_, double(0.5));
    private_nh_.param("lookahead_distance_ratio", lookahead_distance_ratio_,double(4.0)); //假设速度的单位是m/s, 1M/S的预瞄距离是4m
    private_nh_.param("minimum_lookahead_distance_", minimum_lookahead_distance_, double(2.5));
    private_nh_.param("const_lookahead_distance_", const_lookahead_distance_, double(4));
    private_nh_.param("const_velocity_", const_velocity_, double(1)); //1m/s
    private_nh_.param("is_const_vel_dis_", is_const_vel_dis_, true);

    // setup subscriber
    sub1_ = nh_.subscribe("lane_array", 10, &PurePursuitNode::callbackFromWayPoints, this);
    sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
    sub4_ = nh_.subscribe("hall_speed", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

    // setup publisher
    pub = nh_.advertise<geometry_msgs::TwistStamped>("ctrl_cmd", 10);
    //pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
    //pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
    //pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
    //pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0); // debug tool
    //pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
    //pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
    //pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
    // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuitNode::run()
{
    ROS_INFO_STREAM("pure pursuit start");
    ros::Rate loop_rate(LOOP_RATE_);
    while (ros::ok())
    {
        ros::spinOnce();
        // 如果信息没有准备好，不执行pure persuit算法，
        // 信息准备好之后，相应的flag会被置1, 执行一遍pure persuit算法之后，pose，路点，速度的标志位要重置
        if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_)
        {
            // ROS_WARN("Necessary topics are not subscribed yet ... ");
            loop_rate.sleep();
        }
        // 将前视距离与速度关联起来，速度越快，前视觉距离越远
        // 通过速度乘系数得到的前视距离不小于最小前视距离，不大于当前速度×10
        setLookaheadDistance(computeLookaheadDistance());
        // 最小前视距离通过参数配置回调函数给出，前视距离与当前速度的比例也跟据配置回调函数给出
        setMinimumLookaheadDistance(minimum_lookahead_distance_);

        double curvature = 0;
        // 传进去的参数是地址，带着计算的曲率回来
        bool can_get_curvature = canGetCurvature(&curvature);
        publishControlCommandStamped(can_get_curvature, curvature);

        // for visualization with Rviz
        // pub11_.publish(displayNextWaypoint(getPoseOfNextWaypoint()));
        // pub13_.publish(displaySearchRadius(getCurrentPose().position, getLookaheadDistance()));
        // pub12_.publish(displayNextTarget(getPoseOfNextTarget()));
        // pub15_.publish(displayTrajectoryCircle(
        //         waypoint_follower::generateTrajectoryCircle(getPoseOfNextTarget(), getCurrentPose())));
        // std_msgs::Float32 angular_gravity_msg;
        // angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), curvature);
        // pub16_.publish(angular_gravity_msg);

        // publishDeviationCurrentPosition(getCurrentPose().position, getCurrentWaypoints());

        is_pose_set_ = false;
        is_velocity_set_ = false;
        is_waypoint_set_ = false;

        loop_rate.sleep();
    }
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) const
{
    geometry_msgs::TwistStamped control_msg;
    control_msg.header.stamp = ros::Time::now();
    control_msg.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
    control_msg.twist.angular.z = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, curvature) : 0;
    pub.publish(control_msg);
}

double PurePursuitNode::computeLookaheadDistance() const
{
    if (is_const_vel_dis_ == CONST_VEL_DIS)
        return const_lookahead_distance_;

    double maximum_lookahead_distance = current_linear_velocity_ * 10;
    double ld = current_linear_velocity_ * lookahead_distance_ratio_;

    return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

double PurePursuitNode::computeCommandVelocity() const
{
    if (is_const_vel_dis_ == CONST_VEL_DIS)
        return const_velocity_;

    return command_linear_velocity_;
}

double PurePursuitNode::computeAngularGravity(double velocity, double curvature) const
{
    const double gravity = 9.80665;
    return (velocity * velocity) / (1.0 / curvature * gravity);
}

void PurePursuitNode::publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                                      const std::vector<smartcar_msgs::Waypoint> &waypoints) const
{
    // Calculate the deviation of current position from the waypoint approximate line

    if (waypoints.size() < 3)
    {
        return;
    }

    double a, b, c;
    double linear_flag_in =
        getLinearEquation(waypoints.at(2).pose.pose.position, waypoints.at(1).pose.pose.position, &a, &b, &c);

    std_msgs::Float32 msg;
    msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);

    pub17_.publish(msg);
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // 读取了当前的车辆pose消息，里面有xyz和四元数
    setCurrentPose(msg);
    is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
    current_linear_velocity_ = msg->twist.linear.x;
    is_velocity_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const smartcar_msgs::LaneArrayConstPtr &msg)
{
    // 从way_ponits里面读取目标速度
    if (msg->lanes.at(0).waypoints.at(0).twist.twist.linear.x != 0)
        command_linear_velocity_ = msg->lanes.at(0).waypoints.at(0).twist.twist.linear.x;
    else
        command_linear_velocity_ = 1; //  1m/s

    // 从消息中保存下载way_points
    setCurrentWaypoints(msg->lanes[0].waypoints);
    is_waypoint_set_ = true;
}

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &curvature)
{
    return atan(wheel_base * curvature);
}

// 计算曲率
// 曲率 = 2 * el / ld^2  el是当前位置和目标预瞄点的横向误差
double PurePursuitNode::calcCurvature(geometry_msgs::Point target) const
{
    double curvature;
    double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
    double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

    if (denominator != 0)
        curvature = numerator / denominator;
    else
    {
        if (numerator > 0)
            curvature = curvature_MIN_;
        else
            curvature = -curvature_MIN_;
    }
    // ROS_INFO("curvature : %lf", curvature);
    return curvature;
}

// linear interpolation of next target
bool PurePursuitNode::interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const
{
    const double ERROR = pow(10, -5); // 0.00001

    int path_size = static_cast<int>(current_waypoints_.size());
    if (next_waypoint == path_size - 1)
    {
        *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
        return true;
    }
    double search_radius = lookahead_distance_;
    geometry_msgs::Point zero_p;
    geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
    geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(x1 - x2" ,c = "(y1-y2)*x1 + (x2-x1)*y1"
    // 这一步就是根据当前计算出来的预瞄点和前一个点，得到一个直线表达式
    double a = 0;
    double b = 0;
    double c = 0;
    double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    if (!get_linear_flag)
        return false;

    // let the center of circle be "(x0,y0)", in my code , the center of circle is vehicle position
    // the distance  "d" between the foot of a perpendicular line and the center of circle is ...
    //    | a * x0 + b * y0 + c |
    // d = -------------------------------
    //          √( a~2 + b~2)
    // 外国人数学说的真不明白
    // 这一步就是计算当前位置到前面拟合出来的直线的“点到直线的距离”
    double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

    // ROS_INFO("a : %lf ", a);
    // ROS_INFO("b : %lf ", b);
    // ROS_INFO("c : %lf ", c);
    // ROS_INFO("distance : %lf ", d);

    if (d > search_radius)
        return false;

    // unit vector of point 'start' to point 'end'
    // 求取这两个点的向量
    tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
    // Normalize this vector x^2 + y^2 + z^2 = 1.
    // 就是三维矢量的方向不变，但是模长变为1,变成单位向量了
    tf::Vector3 unit_v = v.normalize();

    // normal unit vectors of v
    // 将上面的单位向量分别顺时针和逆时针旋转90度
    tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);  // rotate to counter clockwise 90 degree
    tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90); // rotate to counter clockwise -90 degree

    // the foot of a perpendicular line
    // 现在的h1的坐标就是 当前车辆位置到前面拟合直线的垂足的坐标
    // 他的做法是将车辆当前坐标朝着垂足的方向移动了垂线的距离（当前位置到拟合直线的距离）
    geometry_msgs::Point h1;
    h1.x = current_pose_.position.x + d * unit_w1.getX();
    h1.y = current_pose_.position.y + d * unit_w1.getY();
    h1.z = current_pose_.position.z;

    geometry_msgs::Point h2;
    h2.x = current_pose_.position.x + d * unit_w2.getX();
    h2.y = current_pose_.position.y + d * unit_w2.getY();
    h2.z = current_pose_.position.z;

    // 为什么要写两个垂足，这是因为拟合直线的斜率不一样，情况是不同的，
    // 如果拟合直线的斜率是负，那么h1垂足是在直线上，保留，h2舍弃
    // 如果直线的的斜率是正的，那么h2垂足是在直线上，保留，h1舍弃
    // check which of two foot of a perpendicular line is on the line equation
    geometry_msgs::Point h;
    if (fabs(a * h1.x + b * h1.y + c) < ERROR)
    {
        h = h1;
        //   ROS_INFO("use h1");
    }
    else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
    {
        //   ROS_INFO("use h2");
        h = h2;
    }
    else
    {
        return false;
    }

    // get intersection[s]
    // 这里是以车辆当前位置画一个圈，圈的半径就是前视距离
    // 如果计算出来的 当前位置到拟合直线的距离 大于这个圈，即拟合直线和圈没有交点，那么情况错误，函数直接返回false
    // 如果有一个交点，那么这个交点就是我们的预瞄点，如果有两个交点，那么就选取教前方那个点。
    // if there is a intersection
    if (d == search_radius)
    {
        *next_target = h;
        return true;
    }
    else
    {
        // if there are two intersection
        // get intersection in front of vehicle
        double s = sqrt(pow(search_radius, 2) - pow(d, 2));
        geometry_msgs::Point target1;
        target1.x = h.x + s * unit_v.getX();
        target1.y = h.y + s * unit_v.getY();
        target1.z = current_pose_.position.z;

        geometry_msgs::Point target2;
        target2.x = h.x - s * unit_v.getX();
        target2.y = h.y - s * unit_v.getY();
        target2.z = current_pose_.position.z;

        // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
        // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
        // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

        // check intersection is between end and start
        double interval = getPlaneDistance(end, start);
        if (getPlaneDistance(target1, end) < interval)
        {
            // ROS_INFO("result : target1");
            *next_target = target1;
            return true;
        }
        else if (getPlaneDistance(target2, end) < interval)
        {
            // ROS_INFO("result : target2");
            *next_target = target2;
            return true;
        }
        else
        {
            // ROS_INFO("result : false ");
            return false;
        }
    }
}

void PurePursuitNode::getNextWaypoint()
{
    int path_size = static_cast<int>(current_waypoints_.size());

    // if waypoints are not given, do nothing.
    if (path_size == 0)
    {
        next_waypoint_number_ = -1;
        return;
    }

    // look for the next waypoint.
    for (int i = 0; i < path_size; i++)
    {
        // if search waypoint is the last
        if (i == (path_size - 1))
        {
            ROS_INFO("search waypoint is the last");
            next_waypoint_number_ = i;
            return;
        }

        // if there exists an effective waypoint
        // getPlaneDistances()函数是获得水平距离，不计算z轴的数据
        // 如果计算的距离小于前视距离，那么就计算下一个点
        // 如果找完所有的点都没有大于前视距离的点了，那么就选取最后一个点
        // TODO
        // 不过这个查找每次都要遍历所有的way_point，之后看看能不能修改修改
        if (getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position) >
            lookahead_distance_)
        {
            next_waypoint_number_ = i;
            return;
        }
    }

    // if this program reaches here , it means we lost the waypoint!
    next_waypoint_number_ = -1;
    return;
}

bool PurePursuitNode::canGetCurvature(double *output_curvature)
{
    // search next waypoint
    // 获取了大于前视距离的最邻近点，保存在了next_waypoint_number_
    // 如果没有可以用于跟踪的目标路径点,那么会返回-1
    getNextWaypoint();
    if (next_waypoint_number_ == -1)
    {
        ROS_INFO("lost next waypoint");
        return false;
    }
    // check whether curvature is valid or not
    // 下面处理的作用是，确定当前的路点集是不是有效的
    // 如果当前的路点集中所有的点都小于前视距离的最小值，那么就说明这个路点集是无效的
    // 如果路点集中只要有一个点大于前视距离的最小值，那么这个路点集就是有效的
    bool is_valid_curve = false;
    // C++11 的for新写法，类似于python的for循环
    // ：前面是取出的变量，：后面是迭代器
    for (const smartcar_msgs::Waypoint &el : current_waypoints_)
    {
        if (getPlaneDistance(el.pose.pose.position, current_pose_.position) > minimum_lookahead_distance_)
        {
            is_valid_curve = true;
            break;
        }
    }

    if (!is_valid_curve)
    {
        return false;
    }

    // if is_linear_interpolation_ is false or next waypoint is first or last
    if (!is_linear_interpolation_ || next_waypoint_number_ == 0 ||
        next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1)))
    {
        next_target_position_ = current_waypoints_.at(next_waypoint_number_).pose.pose.position;
        *output_curvature = calcCurvature(next_target_position_);
        return true;
    }

    // linear interpolation and calculate angular velocity
    bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);

    if (!interpolation)
    {
        ROS_INFO_STREAM("lost target! ");
        return false;
    }

    // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);

    *output_curvature = calcCurvature(next_target_position_);
    return true;
}

} // namespace waypoint_follower
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    waypoint_follower::PurePursuitNode ppn;
    ppn.run();
    return 0;
}
