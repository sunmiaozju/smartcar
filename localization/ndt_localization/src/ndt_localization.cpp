/**
 * @brief 
 * 
 * @file ndt_localization.cpp
 * @author jyakaranda
 * @date 2018-09-20
 */

#include "ndt_localization/ndt_localization.h"

NDTLocalization::~NDTLocalization()
{
}

bool NDTLocalization::init()
{
  ROS_INFO("Start init NDTLocalization");
  ros::Duration(1.0).sleep();

  pose_init_ = false;
  odom_init_ = false;
  pub_target_map = nh_.advertise<sensor_msgs::PointCloud2>("local_target_map",10);

  pthread_mutex_init(&mutex, NULL);
  pnh_.param<bool>("is_filter_ground", is_filter_ground, true);
  pnh_.param<double>("min_scan_range",param_min_scan_range,1.0);
  pnh_.param<double>("max_scan_range",param_max_scan_range,100.0);
  std::cout << "min_scan_range: " << param_min_scan_range << std::endl;
  std::cout << "max_scan_range: " << param_max_scan_range << std::endl;
  std::cout << std::endl;

  pnh_.param<double>("voxel_leaf_size",voxel_leaf_size,2.0);

  pnh_.param<std::string>("map_frame", param_map_frame_, std::string("/map"));
  pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
  pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
  pnh_.param<std::string>("laser_frame", param_laser_frame_, std::string("/laser"));
  pnh_.param<std::string>("map_topic", param_map_topic_, std::string("/map"));
  pnh_.param<std::string>("odom_topic", param_odom_topic_, std::string("/odom"));
  // pnh_.param<std::string>("imu_topic", param_imu_topic_, std::string("/imu_base_link"));
  pnh_.param<std::string>("lidar_topic", param_lidar_topic_, std::string("/velodyne_points"));

  pnh_.param<double>("tf_timeout", param_tf_timeout_, 0.05);
  pnh_.param<bool>("use_odom", param_use_odom_, true);
  pnh_.param<double>("odom_timeout", param_odom_timeout_, 1);
  if(param_use_odom_){ROS_WARN_STREAM("Use odom.");}else{ROS_WARN_STREAM("Forbid odom");}

  pnh_.param<double>("predict_error_thresh", param_predict_error_thresh_, 0.5);
  pnh_.param<double>("ndt_resolution", param_ndt_resolution_, 1.0);
  pnh_.param<int>("ndt_max_iterations", param_ndt_max_iterations_, 25);
  pnh_.param<double>("ndt_step_size", param_ndt_step_size_, 0.1);
  pnh_.param<double>("ndt_epsilon", param_ndt_epsilon_, 0.01);
  pnh_.param<int>("method_type", param_method_type_, 0);
  if(param_method_type_ == 0){
    std::cout << "Use ndt gpu" << std::endl;
  }else if(param_method_type_ == 3){
    std::cout << "Use ndt cpu" << std::endl;
  }
  pnh_.param<bool>("debug", param_debug_, false);
  pnh_.param<bool>("if_init_pose_with_param",param_init_pose_with_param,true);

  // 更新局部target地图相关参数
  pnh_.param<bool>("use_local_target",use_local_target,false);
  if(use_local_target){ROS_WARN_STREAM("Use local target map");}else{ROS_WARN_STREAM("Use global target map");}
  pnh_.param<double>("target_map_radius",target_map_radius,0.0);
  pnh_.param<double>("length_update_target_map",lengh_update_target_map,1.0);
  pnh_.param<std::string>("global_map_file",map_file,"Confirm Location of Global Map.");

  pnh_.param<double>("length_update_path",length_update_path,0.2);
  debug_path.header.frame_id = "map";

  if (param_method_type_ == METHOD_CUDA)  // 定义GPU使用shared_ptr方式，因此需要初始化;
  {
#ifdef CUDA_FOUND
    ROS_INFO_STREAM("init gpu ndt.");
    anh_gpu_ndt_ptr =
        std::make_shared<gpu::GNormalDistributionsTransform>();
#else
    ROS_ERROR("param method_type set to cuda, but cuda_found not defined!");
#endif
  }

  // set tf_btol and tf_btol.inverse   (base_link -> laser_link)
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time::now();
    ROS_INFO("now: %f", now.toSec());
    tf_listener_.waitForTransform(param_base_frame_, param_laser_frame_, ros::Time(0), ros::Duration(param_tf_timeout_ * 10), ros::Duration(param_tf_timeout_ / 3));
    tf_listener_.lookupTransform(param_base_frame_, param_laser_frame_, ros::Time(0), transform);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Error waiting for tf in init: %s", ex.what());
    return false;
  }
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
          // current_map2odom

  // current_map2odom_ = tf::Transform(tf::Quaternion(0., 0., 0.), tf::Vector3(0., 0., 0.));
  // end set tf_btol

  if(param_init_pose_with_param){
    init_pose_with_param();
  }else{
    sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  }
  while(!load_map(map_file)){
    if(param_init_pose_with_param){
      init_pose_with_param();
    }
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  // sub_map_ = nh_.subscribe<sensor_msgs::PointCloud2>(param_map_topic_, 1, boost::bind(&NDTLocalization::mapCB, this, _1));
  sub_point_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(param_lidar_topic_, 20, boost::bind(&NDTLocalization::pointCloudCB, this, _1));

  if(param_use_odom_){
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(param_odom_topic_, 500, boost::bind(&NDTLocalization::odomCB, this, _1));
  }
  pub_current_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt/current_pose", 10);
  pub_path = nh_.advertise<nav_msgs::Path>("/debug/history_path",10);
  pub_localPC_handled = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_no_ground", 10);

  if(param_debug_){
    pub_ndt_time = nh_.advertise<std_msgs::Float32>("/ndt_align_time",10);
    pub_ndt_iterations = nh_.advertise<std_msgs::Int16>("/ndt_iterations",10);
  }

    pub_marker_loc_conf_ = nh_.advertise<visualization_msgs::Marker>("/ndt/loc_conf", 1);
    pub_marker_trans_prob_ = nh_.advertise<visualization_msgs::Marker>("/ndt/trans_prob", 1);
    
  if (param_debug_)
  {
    pub_rawodom_ = nh_.advertise<nav_msgs::Odometry>("/map/odom", 10);
    msg_rawodom_.header.frame_id = param_map_frame_;  // nav_msgs::Odometry
  }
  ROS_INFO("End init NDTLocalization");
  return true;

} // init params

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (msg->header.frame_id != param_map_frame_)
  {
    ROS_WARN("Please initialize pose under %s frame.", param_map_frame_.c_str());
    pose_init_ = false;
    return;
  }
  geometryPose2Pose(msg->pose.pose, initial_pose_);

  pre_pose_ = pre_pose_odom_ = current_pose_odom_ = current_pose_ = initial_pose_;
  pose_init_ = true;

  offset_odom_.reset();
  offset_imu_.reset();
  if (param_debug_)
  {
    rawodom_init_ = false;
  }
  std::cout << "Initial pose with:" << std::endl;
  std::cout << "    init_x: " << initial_pose_.x << std::endl;
  std::cout << "    init_y: " << initial_pose_.y << std::endl;
  std::cout << "    init_z: " << initial_pose_.z << std::endl;
  std::cout << " init_roll: " << initial_pose_.roll << std::endl;
  std::cout << "init_pitch: " << initial_pose_.pitch << std::endl;
  std::cout << "  init_yaw: " << initial_pose_.yaw << std::endl;  
  ROS_INFO("-------------Current pose initialized.----------");
}

void NDTLocalization::init_pose_with_param(){
  ROS_INFO("Init pose with param");
  pnh_.param<double>("init_x",initial_pose_.x,0.0);
  pnh_.param<double>("init_y",initial_pose_.y,0.0);
  pnh_.param<double>("init_z",initial_pose_.z,0.0);
  pnh_.param<double>("init_roll",initial_pose_.roll,0.0);
  pnh_.param<double>("init_pitch",initial_pose_.pitch,0.0);
  pnh_.param<double>("init_yaw",initial_pose_.yaw,0.0);

  pre_pose_ = pre_pose_odom_ = current_pose_odom_ = current_pose_ = initial_pose_;
  added_pose = initial_pose_;
  pose_init_ = true;

  offset_odom_.reset();
  offset_imu_.reset();
  if (param_debug_)
  {
    rawodom_init_ = false;
  }
  std::cout << "Initial pose with:" << std::endl;
  std::cout << "    init_x: " << initial_pose_.x << std::endl;
  std::cout << "    init_y: " << initial_pose_.y << std::endl;
  std::cout << "    init_z: " << initial_pose_.z << std::endl;
  std::cout << " init_roll: " << initial_pose_.roll << std::endl;
  std::cout << "init_pitch: " << initial_pose_.pitch << std::endl;
  std::cout << "  init_yaw: " << initial_pose_.yaw << std::endl;  
  ROS_INFO("Current pose initialized.");
}

/**
 * @brief 1. caculate pdf(mean, covariance) for each voxel grid in model
 * 
 * @param msg better to be filtered map data.
 */
bool NDTLocalization::load_map(std::string map_file){
  if(!pose_init_){
    ROS_WARN("initial pose not set, cannot update target_map");
    return false;
  }
  sensor_msgs::PointCloud2::Ptr msg_globalmap(new sensor_msgs::PointCloud2);
  pcl::io::loadPCDFile(map_file,*msg_globalmap);
  pcl::fromROSMsg(*msg_globalmap,model_pc_);
  model_pc_num_ = msg_globalmap->width;

  if(use_local_target){
    update_target_map();   // >>>>>>>>>>更新target地图
    ROS_WARN("(local)target map size: %d",target_map_ptr->points.size());
  }else{
    *target_map_ptr = model_pc_;
    ROS_WARN("(global)target map size: %d",target_map_ptr->points.size());
  }

// set NDT target
  pthread_mutex_lock(&mutex);

  if (param_method_type_ == METHOD_CUDA)
  {
#ifdef CUDA_FOUND
    anh_gpu_ndt_ptr->setResolution(param_ndt_resolution_);
    anh_gpu_ndt_ptr->setInputTarget(target_map_ptr);
    anh_gpu_ndt_ptr->setMaximumIterations(param_ndt_max_iterations_);
    anh_gpu_ndt_ptr->setStepSize(param_ndt_step_size_);
    anh_gpu_ndt_ptr->setTransformationEpsilon(param_ndt_epsilon_);

    PointCloudT::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    PointT dummy_point;
    dummy_scan_ptr->push_back(dummy_point);  // ????
    anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);

    anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

#else
    ROS_ERROR("param method_type set to cuda, but cuda_found not defined!");
#endif
  }
  else if (param_method_type_ == METHOD_OMP)
  {
#ifdef USE_OMP
    PointCloudT::Ptr output_cloud(new PointCloudT());
    omp_ndt_.setResolution(param_ndt_resolution_);
    omp_ndt_.setInputTarget(target_map_ptr);
    omp_ndt_.setMaximumIterations(param_ndt_max_iterations_);
    omp_ndt_.setStepSize(param_ndt_step_size_);
    omp_ndt_.setTransformationEpsilon(param_ndt_epsilon_);
    omp_ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
#else
    ROS_ERROR("param method_type set to omp, but use_omp not defined!");
#endif
  }
  else if(param_method_type_ == METHOD_CPU){
    cpu_ndt_.setResolution(param_ndt_resolution_);
    cpu_ndt_.setInputTarget(target_map_ptr);
    cpu_ndt_.setMaximumIterations(param_ndt_max_iterations_);
    cpu_ndt_.setStepSize(param_ndt_step_size_);
    cpu_ndt_.setTransformationEpsilon(param_ndt_epsilon_);

    PointCloudT::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    PointT dummy_point;
    dummy_scan_ptr->push_back(dummy_point);  // ????
    cpu_ndt_.setInputSource(dummy_scan_ptr);

    cpu_ndt_.align(Eigen::Matrix4f::Identity());
  }
  else
  {
    PointCloudT::Ptr output_cloud(new PointCloudT());
    ndt_.setResolution(param_ndt_resolution_);
    ndt_.setInputTarget(target_map_ptr);
    ndt_.setMaximumIterations(param_ndt_max_iterations_);
    ndt_.setStepSize(param_ndt_step_size_);
    ndt_.setTransformationEpsilon(param_ndt_epsilon_);
    ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
  }

  map_init_ = true;
  pthread_mutex_unlock(&mutex);
  ROS_INFO("Update model pc with %d points.", model_pc_num_);
  return true;
}

void NDTLocalization::mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg){
  if (model_pc_num_ == msg->width)
  {
    // suppose it is same map.
    return;
  }
  if(!pose_init_){
    ROS_WARN("initial pose not set, cannot update target_map");
    return;
  }
  model_pc_num_ = msg->width;
  pcl::fromROSMsg(*msg, model_pc_);
  // PointCloudT::Ptr target_map_ptr(new PointCloudT(model_pc_));
  if(use_local_target){
    update_target_map();   // >>>>>>>>>>更新target地图
    ROS_WARN("(local)target map size: %d",target_map_ptr->points.size());
  }else{
    *target_map_ptr = model_pc_;
    ROS_WARN("(global)target map size: %d",target_map_ptr->points.size());
  }

  // set NDT target
  pthread_mutex_lock(&mutex);

  if (param_method_type_ == METHOD_CUDA)
  {
#ifdef CUDA_FOUND
    anh_gpu_ndt_ptr->setResolution(param_ndt_resolution_);
    anh_gpu_ndt_ptr->setInputTarget(target_map_ptr);
    anh_gpu_ndt_ptr->setMaximumIterations(param_ndt_max_iterations_);
    anh_gpu_ndt_ptr->setStepSize(param_ndt_step_size_);
    anh_gpu_ndt_ptr->setTransformationEpsilon(param_ndt_epsilon_);

    PointCloudT::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    PointT dummy_point;
    dummy_scan_ptr->push_back(dummy_point);  // ????
    anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);

    anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

#else
    ROS_ERROR("param method_type set to cuda, but cuda_found not defined!");
#endif
  }
  else if (param_method_type_ == METHOD_OMP)
  {
#ifdef USE_OMP
    PointCloudT::Ptr output_cloud(new PointCloudT());
    omp_ndt_.setResolution(param_ndt_resolution_);
    omp_ndt_.setInputTarget(target_map_ptr);
    omp_ndt_.setMaximumIterations(param_ndt_max_iterations_);
    omp_ndt_.setStepSize(param_ndt_step_size_);
    omp_ndt_.setTransformationEpsilon(param_ndt_epsilon_);
    omp_ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
#else
    ROS_ERROR("param method_type set to omp, but use_omp not defined!");
#endif
  }
  else if(param_method_type_ == METHOD_CPU){
    cpu_ndt_.setResolution(param_ndt_resolution_);
    cpu_ndt_.setInputTarget(target_map_ptr);
    cpu_ndt_.setMaximumIterations(param_ndt_max_iterations_);
    cpu_ndt_.setStepSize(param_ndt_step_size_);
    cpu_ndt_.setTransformationEpsilon(param_ndt_epsilon_);

    PointCloudT::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    PointT dummy_point;
    dummy_scan_ptr->push_back(dummy_point);  // ????
    cpu_ndt_.setInputSource(dummy_scan_ptr);

    cpu_ndt_.align(Eigen::Matrix4f::Identity());
  }
  else
  {
    PointCloudT::Ptr output_cloud(new PointCloudT());
    ndt_.setResolution(param_ndt_resolution_);
    ndt_.setInputTarget(target_map_ptr);
    ndt_.setMaximumIterations(param_ndt_max_iterations_);
    ndt_.setStepSize(param_ndt_step_size_);
    ndt_.setTransformationEpsilon(param_ndt_epsilon_);
    ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
  }

  map_init_ = true;
  pthread_mutex_unlock(&mutex);
  ROS_INFO("Update model pc with %d points.", model_pc_num_);
}

void NDTLocalization::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!odom_init_){
        odom_init_ = true;
        pre_odom_time_ = msg->header.stamp;
        ROS_INFO("Init odom.");
        return;
    }
    double diff_time = (msg->header.stamp - pre_odom_time_).toSec();
    if (diff_time > param_odom_timeout_)
    {
        ROS_WARN("Long time(%f s) waiting for odom msg, ignore this msg.", diff_time);
        pre_odom_time_ = msg->header.stamp;
        return;
    }

    msg_odom_ = msg;
    // offset_odom_.roll += msg->twist.twist.angular.x * diff_time;
    // offset_odom_.pitch += msg->twist.twist.angular.y * diff_time;
    offset_odom_.yaw += msg->twist.twist.angular.z * diff_time;
    double diff_x = msg->twist.twist.linear.x * diff_time;
    offset_odom_.x += std::cos(-predict_pose_odom_.pitch) * std::cos(predict_pose_odom_.yaw) * diff_x;
    offset_odom_.y += std::cos(-predict_pose_odom_.pitch) * std::sin(predict_pose_odom_.yaw) * diff_x;
    offset_odom_.z += std::sin(-predict_pose_odom_.pitch) * diff_x;

    predict_pose_odom_ = pre_pose_ + offset_odom_;
    // pre_pose_odom_ = current_pose_odom_;
    // ROS_INFO("offset_odom.y: %.2f, %f", offset_odom_.y, ros::Time::now().toSec());
    // ROS_INFO("Current odom pose: (%.2f, %.2f, %.2f; %.2f, %.2f, %.2f)", current_pose_odom_.x, current_pose_odom_.y, current_pose_odom_.z, current_pose_odom_.roll, current_pose_odom_.pitch, current_pose_odom_.yaw);
    pre_odom_time_ = msg->header.stamp;

    tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, param_map_frame_, param_odom_frame_));

    if (param_debug_ && rawodom_init_)
    {
        msg_rawodom_.header.stamp = msg->header.stamp;
        tf::Quaternion tmp_q;
        tf::quaternionMsgToTF(msg_rawodom_.pose.pose.orientation, tmp_q);
        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(tmp_q)).getEulerYPR(yaw, pitch, roll);
        msg_rawodom_.pose.pose.position.x += std::cos(-pitch) * std::cos(yaw) * diff_x;
        msg_rawodom_.pose.pose.position.y += std::cos(-pitch) * std::sin(yaw) * diff_x;
        msg_rawodom_.pose.pose.position.z += std::sin(-pitch) * diff_x;
        yaw += msg->twist.twist.angular.z * diff_time;
        tmp_q.setRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(tmp_q, msg_rawodom_.pose.pose.orientation);
        pub_rawodom_.publish(msg_rawodom_);
    }
}

/** 
 * 1. get data points
 * 2. match data points to model points(map)
 * 2.1 caculate score function: put the point to corresponding pdf, and sum it up
 * 2.2 optimize transformation matrix(position) using Newton method until score function is converged
 */
void NDTLocalization::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // TODO main function
  if (!map_init_ || !pose_init_)
  {
    ROS_WARN_STREAM("Cannot localize without given map and initial pose.");
    return;
  }
  PointCloudT scan;
  PointCloudT::Ptr tmp(new PointCloudT());
  pcl::fromROSMsg(*msg, *tmp);
  pcl::PointCloud<PointT>::Ptr local_pc(new PointCloudT());
  if(is_filter_ground){
    filter.setIfClipHeight(false);
    filter.convert(tmp,local_pc);

    sensor_msgs::PointCloud2::Ptr localPC_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*local_pc,*localPC_ptr);
    localPC_ptr->header.frame_id = "velodyne";
    localPC_ptr->header.stamp = ros::Time::now();
    pub_localPC_handled.publish(*localPC_ptr);

  }else{
    local_pc = tmp;
  }
  PointCloudT::Ptr scan_ptr(new PointCloudT());
  // if(use_local_target){
  //   for(auto point:tmp.points){
  //     double dist = std::sqrt(std::pow(point.x,2)+std::pow(point.y,2));
  //     if(dist <= param_max_scan_range){
  //       scan_ptr->points.push_back(point);
  //     }
  //   }
  // }else{
  //   *scan_ptr = tmp;
  // }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(local_pc);
  voxel_grid_filter.filter(*scan_ptr);
  // ROS_WARN("filtered size: %d",scan_ptr->points.size());

  PointCloudT::Ptr output_cloud(new PointCloudT());
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f final_tf;
  Eigen::Matrix4f base_tf;
  pose predict_ndt_pose;
  pose ndt_pose;

  // TODO predict_ndt_pose
  if (param_use_odom_)
  {
    predict_ndt_pose = predict_pose_odom_;
  }else{
    predict_ndt_pose = pre_pose_;
  }

  Eigen::Translation3f init_translation(predict_ndt_pose.x, predict_ndt_pose.y, predict_ndt_pose.z);
  Eigen::AngleAxisf init_rotation_x(predict_ndt_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(predict_ndt_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(predict_ndt_pose.yaw, Eigen::Vector3f::UnitZ());
  // predict_pose_ndt*odom is map_link->base_link; but init_guess matrix should be under the transform of map_link->laser_link   ===that's why <*tf_btol>
  init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol_;

  ros::Time align_start, align_end, getFitnessScore_start, getFitnessScore_end;

  pthread_mutex_lock(&mutex);
  if (param_method_type_ == METHOD_CUDA)
  {
#ifdef CUDA_FOUND
    anh_gpu_ndt_ptr->setInputSource(scan_ptr);
    // if (param_debug_)
    // {
    //   ROS_INFO("Start align cuda");
    // }
    align_start = ros::Time::now();
    anh_gpu_ndt_ptr->align(init_guess);
    align_end = ros::Time::now();

    final_tf = anh_gpu_ndt_ptr->getFinalTransformation();
    has_converged_ = anh_gpu_ndt_ptr->hasConverged();
    iteration_ = anh_gpu_ndt_ptr->getFinalNumIteration();
    trans_probability_ = anh_gpu_ndt_ptr->getTransformationProbability();

    getFitnessScore_start = ros::Time::now();
    fitness_score_ = anh_gpu_ndt_ptr->getFitnessScore();
    getFitnessScore_end = ros::Time::now();
#else
    ROS_ERROR("param method_type set true, but cuda not found!");
#endif
  }
  else if (param_method_type_ == METHOD_OMP)
  {
#ifdef USE_OMP
    if (param_debug_)
    {
      ROS_INFO("Start align omp");
    }
    omp_ndt_.setInputSource(scan_ptr);

    align_start = ros::Time::now();
    omp_ndt_.align(*output_cloud, init_guess);
    align_end = ros::Time::now();

    final_tf = omp_ndt_.getFinalTransformation();
    has_converged_ = omp_ndt_.hasConverged();
    iteration_ = omp_ndt_.getFinalNumIteration();
    trans_probability_ = omp_ndt_.getTransformationProbability();

    getFitnessScore_start = ros::Time::now();
    fitness_score_ = omp_ndt_.getFitnessScore();
    getFitnessScore_end = ros::Time::now();
#else
    ROS_ERROR("param method_type set to omp, but use_omp not defined!");
#endif
  }
  else if(param_method_type_ == METHOD_CPU){
    cpu_ndt_.setInputSource(scan_ptr);
    // if (param_debug_)
    // {
    //  ROS_INFO("Start align cpu");
    // }
    align_start = ros::Time::now();
    cpu_ndt_.align(init_guess);
    align_end = ros::Time::now();

    final_tf = cpu_ndt_.getFinalTransformation();
    has_converged_ = cpu_ndt_.hasConverged();
    iteration_ = cpu_ndt_.getFinalNumIteration();
    trans_probability_ = cpu_ndt_.getTransformationProbability();

    getFitnessScore_start = ros::Time::now();
    fitness_score_ = cpu_ndt_.getFitnessScore();
    getFitnessScore_end = ros::Time::now();
  }
  else
  {
    ndt_.setInputSource(scan_ptr);

    if (param_debug_)
    {
      ROS_INFO("Start align pcl");
    }
    align_start = ros::Time::now();
    ndt_.align(*output_cloud, init_guess);
    align_end = ros::Time::now();

    final_tf = ndt_.getFinalTransformation();
    has_converged_ = ndt_.hasConverged();
    iteration_ = ndt_.getFinalNumIteration();
    trans_probability_ = ndt_.getTransformationProbability();

    getFitnessScore_start = ros::Time::now();
    fitness_score_ = ndt_.getFitnessScore();
    getFitnessScore_end = ros::Time::now();
  }

  // if (param_debug_)  
  // {
  //   ROS_INFO("NDT has converged(time: %.2f): %d, iterations: %d, fitness_score(time: %.2f): %f, trans_probability: %f", (align_end - align_start).toSec(), has_converged_, iteration_, (getFitnessScore_end - getFitnessScore_start).toSec(), fitness_score_, trans_probability_);
  // }

  pthread_mutex_unlock(&mutex);

  base_tf = final_tf * tf_btol_.inverse();
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(base_tf(0, 0)), static_cast<double>(base_tf(0, 1)), static_cast<double>(base_tf(0, 2)),
                 static_cast<double>(base_tf(1, 0)), static_cast<double>(base_tf(1, 1)), static_cast<double>(base_tf(1, 2)),
                 static_cast<double>(base_tf(2, 0)), static_cast<double>(base_tf(2, 1)), static_cast<double>(base_tf(2, 2)));

  ndt_pose.x = base_tf(0, 3);
  ndt_pose.y = base_tf(1, 3);
  ndt_pose.z = base_tf(2, 3);
  mat_b.getEulerYPR(ndt_pose.yaw, ndt_pose.pitch, ndt_pose.roll);

  predict_pose_error_ = std::sqrt((ndt_pose.x - predict_ndt_pose.x) * (ndt_pose.x - predict_ndt_pose.x) +
                                  (ndt_pose.y - predict_ndt_pose.y) * (ndt_pose.y - predict_ndt_pose.y) +
                                  (ndt_pose.z - predict_ndt_pose.z) * (ndt_pose.z - predict_ndt_pose.z));
  // choose which pose to use as currnt pose  == between predict_ndt_pose  && ndt_pose(matching calculated)
  // if predict(guess) error is smaller than threshold , use it as current pose
  // else use ndt_pose as current pose

  bool use_predict_pose;
  if (predict_pose_error_ <= param_predict_error_thresh_)
  {
    use_predict_pose = false;
  }
  else
  {
    use_predict_pose = true;
  }

  use_predict_pose = false;

  if (!use_predict_pose)
  {
    current_pose_ = ndt_pose;
    // if (param_debug_)
    // {
    //   ROS_INFO("Use ndt predict pose: (%.2f, %.2f, %.2f; %.2f, %.2f, %.2f)", current_pose_.x, current_pose_.y, current_pose_.z, current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    // }
  }
  else
  {
    current_pose_ = predict_ndt_pose;
    ROS_WARN("Use odom predict pose");
    // ROS_WARN("Use odom predict pose: (%.2f, %.2f, %.2f; %.2f, %.2f, %.2f)", current_pose_.x, current_pose_.y, current_pose_.z, current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  }

  // 上面以获得确定的全局位置current_pose, 本处确定是否更新target地图
  double length;
  if(use_local_target){
    length = std::sqrt(std::pow(current_pose_.x - added_pose.x,2) + std::pow(current_pose_.y - added_pose.y,2));
    if(length >= lengh_update_target_map){
      update_target_map();

      pthread_mutex_lock(&mutex);
      if(param_method_type_ == METHOD_CUDA){
        anh_gpu_ndt_ptr->setInputTarget(target_map_ptr);
      }
      else if(param_method_type_ == METHOD_CPU){
        cpu_ndt_.setInputTarget(target_map_ptr);
      }
      else if(param_method_type_ == METHOD_OMP){
        omp_ndt_.setInputTarget(target_map_ptr);
      }
      else{
        ndt_.setInputTarget(target_map_ptr);
      }
      pthread_mutex_unlock(&mutex);

      added_pose = current_pose_;
    }
  }
  if (param_debug_){
    std::cout << "    has converged: " << has_converged_ << std::endl;
    std::cout << "       align time: " << (align_end - align_start).toSec() << std::endl;
    std::cout << "       iterations: " << iteration_ << std::endl;
    std::cout << "    fitness score: " << fitness_score_ << std::endl;
    std::cout << "time to get score: " << (getFitnessScore_end - getFitnessScore_start).toSec() << std::endl;
    std::cout << "trans_probability: " << trans_probability_ << std::endl;
    std::cout << "current pose (x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
								<< ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
    if (use_local_target){
      std::cout << "shift to update localmap: " << length << std::endl;
    }
    std_msgs::Float32 ndt_time;
    std_msgs::Int16 ndt_iterations;
    ndt_time.data = (align_end - align_start).toSec();
    ndt_iterations.data = iteration_;
    pub_ndt_time.publish(ndt_time);
    pub_ndt_iterations.publish(ndt_iterations);

    std::cout << "------------------------------------------------" << std::endl;
    
  }
  pose2GeometryPose(msg_current_pose_.pose, current_pose_);
  msg_current_pose_.header.stamp = msg->header.stamp;    // current pose is under "map_frame"
  msg_current_pose_.header.frame_id = param_map_frame_;
  pub_current_pose_.publish(msg_current_pose_);
  pub_debug_path();

  tf::Quaternion tmp_q;
  tmp_q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  tf::Transform transform2(tmp_q, tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform2,ros::Time::now(),param_map_frame_,param_base_frame_));
  // [] transform2 : map -> base_link

  // publish map->odom using map->base and odom->base
  if(param_use_odom_){
    tf::StampedTransform transform1; // odom_link -> base_link
    try
    {
      tf_listener_.waitForTransform(param_odom_frame_, param_base_frame_, ros::Time(0), ros::Duration(param_tf_timeout_), ros::Duration(param_tf_timeout_ / 3));
      tf_listener_.lookupTransform(param_odom_frame_, param_base_frame_, ros::Time(0), transform1);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Error waiting for tf in pointCloudCB: %s", ex.what());
      // TODO do some stuff
      return;
    }
    current_map2odom_ = transform2 * transform1.inverse();
    // tf_broadcaster_.sendTransform(tf::StampedTransform(current_map2odom_, msg->header.stamp, param_map_frame_, param_odom_frame_));
    // [] transform1 : odom -> base_link
  }

  if (param_debug_ && !rawodom_init_ && !use_predict_pose)
  {
    rawodom_init_ = true;
    tf::poseTFToMsg(transform2, msg_rawodom_.pose.pose);
  }
  geometry_msgs::Vector3 scale;
  scale.x = 6.0 / (trans_probability_ + 0.1);
  scale.y = 3.0 / (fitness_score_ + 0.1);
  scale.z = 0.1;
  util::pubMarkerCylinder(pub_marker_loc_conf_, msg_current_pose_.pose, msg->header.stamp, param_map_frame_, scale);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(4) << "ndt_pose: (" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << "; " << RAD2ANGLE(current_pose_.roll) << ", " << RAD2ANGLE(current_pose_.pitch) << ", " << RAD2ANGLE(current_pose_.yaw) << ")" << std::endl
     << "transform prob: " << trans_probability_ << std::endl
     << "ndt score: " << fitness_score_ << std::endl
     << "match time: " << (align_end - align_start).toSec() << "s" << std::endl
     << "iters: " << iteration_ << std::endl;
  geometry_msgs::Pose pose;
  pose.position.x = 0.;
  pose.position.z = 1.;
  pose.position.y = -20.;
  util::pubMarkerText(pub_marker_trans_prob_, pose, msg->header.stamp, param_map_frame_, ss.str());

  offset_odom_.reset();
  // current_pose_odom_ = current_pose_;
  // pre_pose_odom_ = current_pose_;
  if((current_pose_.x + current_pose_.y - pre_pose_.x -pre_pose_.y) >= length_update_path){
    pub_debug_path();
  }
  pre_pose_ = current_pose_;
}


void NDTLocalization::update_target_map(){
  target_map_ptr->points.clear();
  for(auto point:model_pc_.points){
    double dist = std::sqrt(std::pow(point.x-current_pose_.x,2)+std::pow(point.y-current_pose_.y,2));
    if(dist <= target_map_radius){
      target_map_ptr->points.push_back(point);
    }
  }

  // pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new PointCloudT(*target_map_ptr));
  // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(1.0, 1.0, 1.0);
  // voxel_grid_filter.setInputCloud(scan_ptr);
  // target_map_ptr->points.clear();
  // voxel_grid_filter.filter(*target_map_ptr);

  // publish target_map
  sensor_msgs::PointCloud2::Ptr msg_target_map_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*target_map_ptr,*msg_target_map_ptr);
  msg_target_map_ptr->header.frame_id = "map";
  pub_target_map.publish(*msg_target_map_ptr);

  // sensor_msgs::PointCloud2 msg_target_map;
  // pcl::toROSMsg(*target_map_ptr,msg_target_map);
  // msg_target_map.header.frame_id = "map";
  // pub_target_map.publish(msg_target_map);

  ROS_WARN("update local map with %d points",target_map_ptr->points.size());
}

void NDTLocalization::pub_debug_path(){
  geometry_msgs::PoseStamped p;
  p.pose.position.x = current_pose_.x;
  p.pose.position.y = current_pose_.y;
  p.pose.position.z = current_pose_.z;
  
  Eigen::AngleAxisd roll_angle(current_pose_.roll,Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(current_pose_.pitch,Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(current_pose_.yaw,Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = roll_angle*pitch_angle*yaw_angle;
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();

  debug_path.poses.push_back(p);
  pub_path.publish(debug_path);
}
