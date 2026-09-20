/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/odom.h"
#include "dlio/utils.h"

#include <queue>
#include <cstring>
#include <stdexcept>

#include "rclcpp/qos.hpp"

dlio::OdomNode::OdomNode() : Node("dlio_odom_node") {

  this->getParams();

  this->gicp.setNumThreads(this->num_threads_);
  this->gicp_temp.setNumThreads(this->num_threads_);

  this->dlio_initialized = false;
  this->first_valid_scan = false;
  this->first_imu_received = false;
  if (this->imu_calibrate_) {this->imu_calibrated = false;}
  else {this->imu_calibrated = true;}
  this->deskew_status = false;
  this->deskew_size = 0;

  this->lidar_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = this->lidar_cb_group;
  this->lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&dlio::OdomNode::callbackPointCloud, this, std::placeholders::_1), lidar_sub_opt);

  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;
  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS().keep_last(1000),
      std::bind(&dlio::OdomNode::callbackImu, this, std::placeholders::_1), imu_sub_opt);

  this->path_pub     = this->create_publisher<nav_msgs::msg::Path>("path", 1);
  this->deskewed_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("deskewed", 1);

  if (this->publish_pose_odom_) {
    this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
  }
  if (this->publish_keyframes_) {
    this->kf_pose_pub =
        this->create_publisher<geometry_msgs::msg::PoseArray>("kf_pose", 1);
    this->kf_cloud_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("kf_cloud", 1);
    this->keyframe_bundle_pub =
        this->create_publisher<loop_closure::msg::Keyframe>("keyframe", 10);
  }

  this->br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  if (this->publish_pose_odom_) {
    this->publish_timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / this->publish_rate_hz_),
        std::bind(&dlio::OdomNode::publishPose, this));
  }

  this->T = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->T_corr = Eigen::Matrix4f::Identity();

  this->origin = Eigen::Vector3f(0., 0., 0.);
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);

  this->lidarPose.p = Eigen::Vector3f(0., 0., 0.);
  this->lidarPose.q = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->prev_imu_stamp = 0.;

  this->original_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->deskewed_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<const pcl::PointCloud<PointType>>();
  this->submap_cloud = std::make_shared<const pcl::PointCloud<PointType>>();

  this->num_processed_keyframes = 0;

  this->submap_hasChanged = true;
  this->submap_kf_idx_prev.clear();

  this->first_scan_stamp = 0.;
  this->elapsed_time = 0.;
  this->length_traversed = 0.0;

  this->convex_hull.setDimension(3);
  this->concave_hull.setDimension(3);
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
  this->concave_hull.setKeepInformation(true);

  this->gicp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

  this->gicp_temp.setCorrespondenceRandomness(this->gicp_k_correspondences_);
  this->gicp_temp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp_temp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp_temp.setTransformationEpsilon(this->gicp_transformation_ep_);
  this->gicp_temp.setRotationEpsilon(this->gicp_rotation_ep_);
  this->gicp_temp.setInitialLambdaFactor(this->gicp_init_lambda_factor_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);
  this->gicp_temp.setSearchMethodSource(temp, true);
  this->gicp_temp.setSearchMethodTarget(temp, true);

  this->geo.first_opt_done = false;
  this->geo.prev_vel = Eigen::Vector3f(0., 0., 0.);

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

  this->voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);

  this->metrics.spaciousness.push_back(0.);
  this->metrics.density.push_back(this->gicp_max_corr_dist_);

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));

  this->cpu_type = "";

  #ifdef HAS_CPUID
  unsigned int CPUInfo[4] = {0,0,0,0};
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }
  this->cpu_type = CPUBrandString;
  boost::trim(this->cpu_type);
  #endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  this->lastCPU = times(&timeSample);
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;

  file = fopen("/proc/cpuinfo", "r");
  this->numProcessors = 0;
  while(fgets(line, 128, file) != nullptr) {
      if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
  }
  fclose(file);

}

dlio::OdomNode::~OdomNode() {
  stopping_ = true;
  { std::lock_guard<std::mutex> lock(main_loop_running_mutex); main_loop_running = false; }
  cv_imu_stamp.notify_all();
  submap_build_cv.notify_all();
  if (submap_future.valid()) submap_future.wait();
}

void dlio::OdomNode::getParams() {

  // Version
  dlio::declare_param(this, "version", this->version_, "0.0.0");

  // Frames
  dlio::declare_param(this, "frames/odom", this->odom_frame, "odom");
  dlio::declare_param(this, "frames/baselink", this->baselink_frame, "base_link");
  dlio::declare_param(this, "frames/lidar", this->lidar_frame, "lidar");
  dlio::declare_param(this, "frames/imu", this->imu_frame, "imu");

  // The path/cloud/TF outputs are produced with each lidar update. The
  // high-rate pose and odometry topics can be disabled by lightweight wrappers.
  dlio::declare_param(this, "publish/pose_odom", this->publish_pose_odom_, true);
  dlio::declare_param(this, "publish/keyframes", this->publish_keyframes_, true);
  dlio::declare_param(this, "publish/state_rate_hz", this->publish_rate_hz_, 100.0);
  dlio::declare_param(this, "publish/path_capacity", this->path_capacity_, 10000);
  dlio::declare_param(this, "terminal/enabled", this->terminal_enabled_, true);
  dlio::declare_param(this, "odom/preprocessing/maxPoints", this->maximum_tracking_points_, 4000);
  dlio::declare_param(this, "odom/threads", this->num_threads_, 4);
  dlio::declare_param(this, "imu/lever_smoothing_seconds", this->lever_smoothing_seconds_, 0.02);
  if (!(publish_rate_hz_ > 0.0) || maximum_tracking_points_ < 64 ||
      path_capacity_ < 1 || num_threads_ < 1 || lever_smoothing_seconds_ < 0.0) {
    throw std::invalid_argument("invalid DLIO rate, point cap, path size or thread count");
  }

  // Deskew Flag
  dlio::declare_param(this, "pointcloud/deskew", this->deskew_, true);

  // Gravity
  dlio::declare_param(this, "odom/gravity", this->gravity_, 9.80665);

  // Compute time offset between lidar and imu
  dlio::declare_param(this, "odom/computeTimeOffset", this->time_offset_, false);

  // Keyframe Threshold
  dlio::declare_param(this, "odom/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
  dlio::declare_param(this, "odom/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);

  // Submap
  dlio::declare_param(this, "odom/submap/keyframe/knn", this->submap_knn_, 10);
  dlio::declare_param(this, "odom/submap/keyframe/kcv", this->submap_kcv_, 10);
  dlio::declare_param(this, "odom/submap/keyframe/kcc", this->submap_kcc_, 10);

  // Dense map resolution
  dlio::declare_param(this, "map/dense/filtered", this->densemap_filtered_, true);

  // Wait until movement to publish map
  dlio::declare_param(this, "map/waitUntilMove", this->wait_until_move_, false);

  // Crop Box Filter
  dlio::declare_param(this, "odom/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

  // Voxel Grid Filter
  dlio::declare_param(this, "pointcloud/voxelize", this->vf_use_, true);
  dlio::declare_param(this, "odom/preprocessing/voxelFilter/res", this->vf_res_, 0.05);

  // Adaptive Parameters
  dlio::declare_param(this, "adaptive", this->adaptive_params_, true);

  // Extrinsics
  std::vector<double> t_default{0., 0., 0.};
  std::vector<double> R_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::vector<double> fused_rotation, fused_lever;
  dlio::declare_param(this, "imu/fused_correction_R", fused_rotation, R_default);
  dlio::declare_param(this, "imu/fused_lever_arm", fused_lever, t_default);
  if (fused_rotation.size() != 9 || fused_lever.size() != 3) throw std::invalid_argument("invalid fused IMU calibration dimensions");
  for (int i = 0; i < 3; ++i) {
    fused_imu_lever_[i] = fused_lever[i];
    for (int j = 0; j < 3; ++j) fused_imu_rotation_(i,j) = fused_rotation[3*i+j];
  }
  if (!fused_imu_lever_.allFinite() || !fused_imu_rotation_.allFinite() ||
      (fused_imu_rotation_.transpose()*fused_imu_rotation_-Eigen::Matrix3f::Identity()).norm() > 1e-5 ||
      std::abs(fused_imu_rotation_.determinant()-1.f)>1e-5) throw std::invalid_argument("invalid fused IMU rotation");

  // center of gravity to imu
  std::vector<double> baselink2imu_t, baselink2imu_R;
  dlio::declare_param(this, "extrinsics/baselink2imu/t", baselink2imu_t, t_default);
  dlio::declare_param(this, "extrinsics/baselink2imu/R", baselink2imu_R, R_default);
  this->extrinsics.baselink2imu.t =
    Eigen::Vector3f(baselink2imu_t[0], baselink2imu_t[1], baselink2imu_t[2]);
  this->extrinsics.baselink2imu.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(baselink2imu_R.begin(), baselink2imu_R.end()).data(), 3, 3);
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) = this->extrinsics.baselink2imu.t;
  this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) = this->extrinsics.baselink2imu.R;

  // center of gravity to lidar
  std::vector<double> baselink2lidar_t, baselink2lidar_R;
  dlio::declare_param(this, "extrinsics/baselink2lidar/t", baselink2lidar_t, t_default);
  dlio::declare_param(this, "extrinsics/baselink2lidar/R", baselink2lidar_R, R_default);

  this->extrinsics.baselink2lidar.t =
    Eigen::Vector3f(baselink2lidar_t[0], baselink2lidar_t[1], baselink2lidar_t[2]);
  this->extrinsics.baselink2lidar.R =
    Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(baselink2lidar_R.begin(), baselink2lidar_R.end()).data(), 3, 3);

  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T.block(0, 3, 3, 1) = this->extrinsics.baselink2lidar.t;
  this->extrinsics.baselink2lidar_T.block(0, 0, 3, 3) = this->extrinsics.baselink2lidar.R;

  // IMU
  dlio::declare_param(this, "odom/imu/calibration/accel", this->calibrate_accel_, true);
  dlio::declare_param(this, "odom/imu/calibration/gyro", this->calibrate_gyro_, true);
  dlio::declare_param(this, "odom/imu/calibration/time", this->imu_calib_time_, 3.0);
  dlio::declare_param(this, "odom/imu/bufferSize", this->imu_buffer_size_, 2000);

  std::vector<double> accel_default{0., 0., 0.}; std::vector<double> prior_accel_bias;
  std::vector<double> gyro_default{0., 0., 0.}; std::vector<double> prior_gyro_bias;

  dlio::declare_param(this, "odom/imu/approximateGravity", this->gravity_align_, true);
  dlio::declare_param(this, "imu/calibration", this->imu_calibrate_, true);
  dlio::declare_param(this, "imu/intrinsics/accel/bias", prior_accel_bias, accel_default);
  dlio::declare_param(this, "imu/intrinsics/gyro/bias", prior_gyro_bias, gyro_default);

  // scale-misalignment matrix
  std::vector<double> imu_sm_default{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  std::vector<double> imu_sm;

  dlio::declare_param(this, "imu/intrinsics/accel/sm", imu_sm, imu_sm_default);

  if (!this->imu_calibrate_) {
    this->state.b.accel[0] = prior_accel_bias[0];
    this->state.b.accel[1] = prior_accel_bias[1];
    this->state.b.accel[2] = prior_accel_bias[2];
    this->state.b.gyro[0] = prior_gyro_bias[0];
    this->state.b.gyro[1] = prior_gyro_bias[1];
    this->state.b.gyro[2] = prior_gyro_bias[2];
    this->imu_accel_sm_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(std::vector<float>(imu_sm.begin(), imu_sm.end()).data(), 3, 3);
  } else {
    this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
    this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
    this->imu_accel_sm_ = Eigen::Matrix3f::Identity();
  }

  // GICP
  dlio::declare_param(this, "odom/gicp/minNumPoints", this->gicp_min_num_points_, 100);
  dlio::declare_param(this, "odom/gicp/kCorrespondences", this->gicp_k_correspondences_, 20);
  dlio::declare_param(this, "odom/gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_,
      std::sqrt(std::numeric_limits<double>::max()));
  dlio::declare_param(this, "odom/gicp/maxIterations", this->gicp_max_iter_, 64);
  dlio::declare_param(this, "odom/gicp/transformationEpsilon", this->gicp_transformation_ep_, 0.0005);
  dlio::declare_param(this, "odom/gicp/rotationEpsilon", this->gicp_rotation_ep_, 0.0005);
  dlio::declare_param(this, "odom/gicp/initLambdaFactor", this->gicp_init_lambda_factor_, 1e-9);

  // Geometric Observer
  dlio::declare_param(this, "odom/geo/Kp", this->geo_Kp_, 1.0);
  dlio::declare_param(this, "odom/geo/Kv", this->geo_Kv_, 1.0);
  dlio::declare_param(this, "odom/geo/Kq", this->geo_Kq_, 1.0);
  dlio::declare_param(this, "odom/geo/Kab", this->geo_Kab_, 1.0);
  dlio::declare_param(this, "odom/geo/Kgb", this->geo_Kgb_, 1.0);
  dlio::declare_param(this, "odom/geo/abias_max", this->geo_abias_max_, 1.0);
  dlio::declare_param(this, "odom/geo/gbias_max", this->geo_gbias_max_, 1.0);
}

void dlio::OdomNode::start() {

  printf("\033[2J\033[1;1H");
  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|               Direct LiDAR-Inertial Odometry v" << this->version_  << "               |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}

void dlio::OdomNode::publishPose() {
  State snapshot;
  double stamp;
  {
    std::lock_guard<std::mutex> lock(geo.mtx);
    if (!geo.first_opt_done || state_stamp_ - correction_stamp_ > 0.5 ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_imu_arrival_).count() > 0.05) {
      output_rate_hz_ = 0.; published_stamps_.clear(); return;
    }
    if (state_stamp_ <= last_publish_stamp_) return;
    snapshot = state;
    stamp = state_stamp_;
  }
  published_stamps_.push_back(stamp);
  if (published_stamps_.size() > 100) published_stamps_.pop_front();
  if (published_stamps_.size() > 1) output_rate_hz_ =
      (published_stamps_.size()-1)/(published_stamps_.back()-published_stamps_.front());
  last_publish_stamp_ = stamp;
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(stamp * 1e9)));
  odometry.header.frame_id = odom_frame;
  odometry.child_frame_id = baselink_frame;
  odometry.pose.pose.position.x = snapshot.p.x();
  odometry.pose.pose.position.y = snapshot.p.y();
  odometry.pose.pose.position.z = snapshot.p.z();
  odometry.pose.pose.orientation.w = snapshot.q.w();
  odometry.pose.pose.orientation.x = snapshot.q.x();
  odometry.pose.pose.orientation.y = snapshot.q.y();
  odometry.pose.pose.orientation.z = snapshot.q.z();
  const Eigen::Vector3f body_velocity = snapshot.q.conjugate() * snapshot.v.lin.w;
  odometry.twist.twist.linear.x = body_velocity.x();
  odometry.twist.twist.linear.y = body_velocity.y();
  odometry.twist.twist.linear.z = body_velocity.z();
  odometry.twist.twist.angular.x = snapshot.v.ang.b.x();
  odometry.twist.twist.angular.y = snapshot.v.ang.b.y();
  odometry.twist.twist.angular.z = snapshot.v.ang.b.z();
  odom_pub->publish(odometry);
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  pose_pub->publish(pose);
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = baselink_frame;
  transform.transform.translation.x = snapshot.p.x();
  transform.transform.translation.y = snapshot.p.y();
  transform.transform.translation.z = snapshot.p.z();
  transform.transform.rotation = pose.pose.orientation;
  br->sendTransform(transform);
}

void dlio::OdomNode::publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {
  this->publishCloud(published_cloud, T_cloud);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(scan_stamp * 1e9)));
  pose.header.frame_id = odom_frame;
  pose.pose.position.x = lidarPose.p.x();
  pose.pose.position.y = lidarPose.p.y();
  pose.pose.position.z = lidarPose.p.z();
  pose.pose.orientation.w = lidarPose.q.w();
  pose.pose.orientation.x = lidarPose.q.x();
  pose.pose.orientation.y = lidarPose.q.y();
  pose.pose.orientation.z = lidarPose.q.z();
  path_ros.header = pose.header;
  path_ros.poses.push_back(pose);
  if (path_ros.poses.size() > static_cast<std::size_t>(path_capacity_)) path_ros.poses.erase(path_ros.poses.begin());
  path_pub->publish(path_ros);
  if (!publish_pose_odom_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header; transform.child_frame_id = baselink_frame;
    transform.transform.translation.x = lidarPose.p.x();
    transform.transform.translation.y = lidarPose.p.y();
    transform.transform.translation.z = lidarPose.p.z();
    transform.transform.rotation = pose.pose.orientation;
    br->sendTransform(transform);
  }
  for (int i = 0; i < 2; ++i) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.header.frame_id = baselink_frame;
    transform.child_frame_id = i == 0 ? imu_frame : lidar_frame;
    const auto &extrinsic = i == 0 ? extrinsics.baselink2imu : extrinsics.baselink2lidar;
    transform.transform.translation.x = extrinsic.t.x();
    transform.transform.translation.y = extrinsic.t.y();
    transform.transform.translation.z = extrinsic.t.z();
    const Eigen::Quaternionf q(extrinsic.R);
    transform.transform.rotation.w = q.w(); transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y(); transform.transform.rotation.z = q.z();
    br->sendTransform(transform);
  }
}

void dlio::OdomNode::publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud) {

  if (this->wait_until_move_) {
    if (this->length_traversed < 0.1) { return; }
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_t_ = std::make_shared<pcl::PointCloud<PointType>>();

  pcl::transformPointCloud (*published_cloud, *deskewed_scan_t_, T_cloud);

  // published deskewed cloud
  sensor_msgs::msg::PointCloud2 deskewed_ros;
  pcl::toROSMsg(*deskewed_scan_t_, deskewed_ros);
  deskewed_ros.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(scan_stamp * 1e9)));
  deskewed_ros.header.frame_id = this->odom_frame;
  this->deskewed_pub->publish(deskewed_ros);

}

void dlio::OdomNode::publishKeyframe(std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::ConstPtr> kf, rclcpp::Time timestamp) {

  // Push back
  geometry_msgs::msg::Pose p;
  p.position.x = kf.first.first[0];
  p.position.y = kf.first.first[1];
  p.position.z = kf.first.first[2];
  p.orientation.w = kf.first.second.w();
  p.orientation.x = kf.first.second.x();
  p.orientation.y = kf.first.second.y();
  p.orientation.z = kf.first.second.z();
  this->kf_pose_ros.poses.push_back(p);

  // Publish
  this->kf_pose_ros.header.stamp = timestamp;
  this->kf_pose_ros.header.frame_id = this->odom_frame;
  this->kf_pose_pub->publish(this->kf_pose_ros);

  // publish keyframe scan for map
  if (this->vf_use_) {
    if (kf.second->points.size() == kf.second->width * kf.second->height) {
      sensor_msgs::msg::PointCloud2 keyframe_cloud_ros;
      pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
      keyframe_cloud_ros.header.stamp = timestamp;
      keyframe_cloud_ros.header.frame_id = this->odom_frame;
      this->kf_cloud_pub->publish(keyframe_cloud_ros);
    }
  } else {
    sensor_msgs::msg::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*kf.second, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = timestamp;
    keyframe_cloud_ros.header.frame_id = this->odom_frame;
    this->kf_cloud_pub->publish(keyframe_cloud_ros);
  }

}

void dlio::OdomNode::publishKeyframeBundle(
    int id,
    std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::ConstPtr> kf,
    rclcpp::Time timestamp) {
  this->publishKeyframe(kf, timestamp);
  loop_closure::msg::Keyframe msg;
  msg.id = id;
  msg.pose.position.x = kf.first.first[0];
  msg.pose.position.y = kf.first.first[1];
  msg.pose.position.z = kf.first.first[2];
  msg.pose.orientation.w = kf.first.second.w();
  msg.pose.orientation.x = kf.first.second.x();
  msg.pose.orientation.y = kf.first.second.y();
  msg.pose.orientation.z = kf.first.second.z();
  pcl::toROSMsg(*kf.second, msg.cloud);
  msg.cloud.header.stamp = timestamp;
  msg.cloud.header.frame_id = this->odom_frame;
  this->keyframe_bundle_pub->publish(msg);
}

void dlio::OdomNode::getScanFromROS(const sensor_msgs::msg::PointCloud2::SharedPtr& pc) {

  this->scan_valid_ = false;
  this->original_scan.reset(new pcl::PointCloud<PointType>);
  if (!pc->width || !pc->height || pc->is_bigendian || !pc->point_step ||
      pc->row_step < static_cast<std::size_t>(pc->width) * pc->point_step ||
      pc->data.size() < static_cast<std::size_t>(pc->row_step) * pc->height) return;
  const auto field = [&pc](const char *name, int type) -> const sensor_msgs::msg::PointField * {
    for (const auto &f : pc->fields) if (f.name == name && f.datatype == type && f.count == 1 &&
        static_cast<std::size_t>(f.offset) + (type == 8 ? 8 : 4) <= pc->point_step) return &f;
    return nullptr;
  };
  const auto *x = field("x", 7), *y = field("y", 7), *z = field("z", 7);
  const auto *intensity = field("intensity", 7);
  const auto *timing = field("time", 7);
  this->sensor = dlio::SensorType::VELODYNE;
  if (!timing) { timing = field("t", 6); this->sensor = dlio::SensorType::OUSTER; }
  if (!timing) { timing = field("timestamp", 8); this->sensor = dlio::SensorType::HESAI; }
  if (!x || !y || !z || (deskew_ && !timing)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "DLIO requires XYZ and per-point time for deskew");
    return;
  }
  pcl::PointCloud<PointType>::Ptr original_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
  original_scan_->reserve(static_cast<std::size_t>(pc->width) * pc->height);
  double min_time = std::numeric_limits<double>::infinity(), max_time = -min_time;
  for (std::size_t row = 0; row < pc->height; ++row) {
    for (std::size_t col = 0; col < pc->width; ++col) {
      const auto *data = pc->data.data() + row * pc->row_step + col * pc->point_step;
      PointType p;
      p.timestamp = 0.; p.intensity = 0.;
      std::memcpy(&p.x, data + x->offset, 4);
      std::memcpy(&p.y, data + y->offset, 4);
      std::memcpy(&p.z, data + z->offset, 4);
      if (intensity) std::memcpy(&p.intensity, data + intensity->offset, 4);
      if (timing) {
        std::memcpy(&p.timestamp, data + timing->offset, timing->datatype == 8 ? 8 : 4);
        const double t = sensor == dlio::SensorType::VELODYNE ? p.time :
            (sensor == dlio::SensorType::OUSTER ? p.t * 1e-9 : p.timestamp);
        if (!std::isfinite(t) || (sensor == dlio::SensorType::VELODYNE && (t < -.005 || t > .25))) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid point time: fused time must be seconds relative to the scan header");
          return;
        }
        min_time = std::min(min_time, t); max_time = std::max(max_time, t);
      }
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) original_scan_->push_back(p);
    }
  }
  if (sensor == dlio::SensorType::HESAI && min_time > 1e14) sensor = dlio::SensorType::LIVOX;
  if (deskew_ && !(max_time > min_time)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "DLIO rejects zero point times; rebuild fusion_ws to preserve Livox offsets");
    return;
  }

  // Remove NaNs
  std::vector<int> idx;
  original_scan_->is_dense = false;
  pcl::removeNaNFromPointCloud(*original_scan_, *original_scan_, idx);

  // Crop Box Filter
  this->crop.setInputCloud(original_scan_);
  this->crop.filter(*original_scan_);

  if (original_scan_->points.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dropping empty LiDAR scan after filtering");
    this->original_scan = original_scan_;
    return;
  }

  this->scan_header_stamp = pc->header.stamp;
  this->original_scan = original_scan_;
  this->scan_valid_ = true;

}

void dlio::OdomNode::preprocessPoints() {
  if (!scan_valid_) return;

  // Deskew the original dlio-type scan
  if (this->deskew_) {

    this->deskewPointcloud();

    if (!this->first_valid_scan || !this->scan_valid_) {
      return;
    }

  } else {

    this->scan_stamp = rclcpp::Time(this->scan_header_stamp).seconds();

    // don't process scans until IMU data is present
    if (!this->first_valid_scan) {
      std::lock_guard<std::mutex> imu_lock(mtx_imu);
      if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
        return;
      }

      this->first_valid_scan = true;
      this->T_prior = this->T; // assume no motion for the first scan

    } else {

      // IMU prior for second scan onwards
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
      frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                                this->geo.prev_vel.cast<float>(), {this->scan_stamp});

    if (frames.size() > 0) {
      this->T_prior = frames.back();
    } else {
      this->T_prior = this->T;
    }

    }

    pcl::PointCloud<PointType>::Ptr deskewed_scan_ = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*this->original_scan, *deskewed_scan_,
                              this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = false;
  }

  // Voxel Grid Filter
  if (this->vf_use_) {
    pcl::PointCloud<PointType>::Ptr current_scan_ = std::make_shared<pcl::PointCloud<PointType>>(*this->deskewed_scan);
    this->voxel.setInputCloud(current_scan_);
    this->voxel.filter(*current_scan_);
    this->current_scan = current_scan_;
  } else {
    this->current_scan = this->deskewed_scan;
  }
  // Matching/submaps use a bounded sparse cloud. Dense deskewed keyframes
  // remain available to mapping and loop closure.
  if (current_scan->size() > static_cast<std::size_t>(maximum_tracking_points_)) {
    pcl::PointCloud<PointType>::Ptr limited(new pcl::PointCloud<PointType>);
    limited->reserve(maximum_tracking_points_);
    for (int i = 0; i < maximum_tracking_points_; ++i) {
      limited->push_back(current_scan->points[static_cast<std::size_t>(i) * current_scan->size() / maximum_tracking_points_]);
    }
    current_scan = limited;
  }

}

void dlio::OdomNode::deskewPointcloud() {

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ = std::make_shared<pcl::PointCloud<PointType>>(1, this->original_scan->points.size());
  // deskewed_scan_->points.resize(this->original_scan->points.size());
  // individual point timestamps should be relative to this time
  double sweep_ref_time = rclcpp::Time(this->scan_header_stamp).seconds();

  // sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<bool(boost::range::index_value<PointType&, long>,
                     boost::range::index_value<PointType&, long>)> point_time_neq;
  std::function<double(boost::range::index_value<PointType&, long>)> extract_point_time;

  if (this->sensor == dlio::SensorType::OUSTER) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.t < p2.t; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().t != p2.value().t; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().t * 1e-9f; };

  } else if (this->sensor == dlio::SensorType::VELODYNE) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.time < p2.time; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().time != p2.value().time; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return sweep_ref_time + pt.value().time; };

  } else if (this->sensor == dlio::SensorType::HESAI) {

    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().timestamp != p2.value().timestamp; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return pt.value().timestamp; };
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    point_time_cmp = [](const PointType& p1, const PointType& p2)
      { return p1.timestamp < p2.timestamp; };
    point_time_neq = [](boost::range::index_value<PointType&, long> p1,
                        boost::range::index_value<PointType&, long> p2)
      { return p1.value().timestamp != p2.value().timestamp; };
    extract_point_time = [&sweep_ref_time](boost::range::index_value<PointType&, long> pt)
      { return pt.value().timestamp * 1e-9; };
  }

  // copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // filter unique timestamps
  auto points_unique_timestamps = deskewed_scan_->points
                                  | boost::adaptors::indexed()
                                  | boost::adaptors::adjacent_filtered(point_time_neq);

  // extract timestamps from points and put them in their own list
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;

  // compute offset between sweep reference time and first point timestamp
  double offset = 0.0;
  if (this->time_offset_) {
    offset = sweep_ref_time - extract_point_time(*points_unique_timestamps.begin());
  }

  // build list of unique timestamps and indices of first point with each timestamp
  for (auto it = points_unique_timestamps.begin(); it != points_unique_timestamps.end(); it++) {
    timestamps.push_back(extract_point_time(*it) + offset);
    unique_time_indices.push_back(it->index());
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  int median_pt_index = timestamps.size() / 2;
  this->scan_stamp = timestamps[median_pt_index]; // set this->scan_stamp to the timestamp of the median point

  // don't process scans until IMU data is present
  if (!this->first_valid_scan) {
    std::lock_guard<std::mutex> imu_lock(mtx_imu);
    if (this->imu_buffer.empty() || this->scan_stamp <= this->imu_buffer.back().stamp) {
      return;
    }

    this->first_valid_scan = true;
    this->T_prior = this->T; // assume no motion for the first scan
    pcl::transformPointCloud (*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->deskewed_scan = deskewed_scan_;
    this->deskew_status = true;
    return;
  }

  // IMU prior & deskewing for second scan onwards
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                              this->geo.prev_vel.cast<float>(), timestamps);
  this->deskew_size = frames.size(); // if integration successful, equal to timestamps.size()

  // if there are no frames between the start and end of the sweep
  // that probably means that there's a sync issue
  if (frames.size() != timestamps.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping scan: IMU does not cover all point acquisition times");
    this->scan_valid_ = false;
    this->deskew_status = false;
    return;
  }

  // update prior to be the estimated pose at the median time of the scan (corresponds to this->scan_stamp)
  this->T_prior = frames[median_pt_index];

#pragma omp parallel for num_threads(this->num_threads_)
  for (int i = 0; i < timestamps.size(); i++) {

    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->deskewed_scan = deskewed_scan_;
  this->deskew_status = true;

}

void dlio::OdomNode::initializeInputTarget() {

  this->prev_scan_stamp = this->scan_stamp;

  // keep history of keyframes
  this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
  this->dense_keyframe_clouds.push_back(this->deskewed_scan);
  this->keyframe_timestamps.push_back(rclcpp::Time(static_cast<int64_t>(std::llround(scan_stamp * 1e9))));
  this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
  this->keyframe_transformations.push_back(this->T_corr);

}

void dlio::OdomNode::setInputSource() {
  this->gicp.setInputSource(this->current_scan);
  this->gicp.calculateSourceCovariances();
}

void dlio::OdomNode::initializeDLIO() {

  // Wait for IMU
  if (!this->first_imu_received || !this->imu_calibrated) {
    return;
  }

  this->dlio_initialized = true;
  std::cout << std::endl << " DLIO initialized!" << std::endl;

}

void dlio::OdomNode::callbackPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {

  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(main_loop_running_mutex);
  this->main_loop_running = true;
  lock.unlock();
  const auto release_loop = [this](void *) {
    { std::lock_guard<std::mutex> guard(main_loop_running_mutex); main_loop_running = false; }
    submap_build_cv.notify_all();
  };
  std::unique_ptr<void, decltype(release_loop)> running_guard(this, release_loop);

  const auto then = std::chrono::steady_clock::now();

  if (this->first_scan_stamp == 0.) {
    this->first_scan_stamp = rclcpp::Time(pc->header.stamp).seconds();
  }

  // DLIO Initialization procedures (IMU calib, gravity align)
  if (!this->dlio_initialized) {
    this->initializeDLIO();
  }
  if (!this->dlio_initialized) return;

  // Convert incoming scan into DLIO format
  this->getScanFromROS(pc);

  // Preprocess points
  this->preprocessPoints();

  if (!this->first_valid_scan || !this->scan_valid_) {
    return;
  }

  if (this->current_scan->points.size() <= this->gicp_min_num_points_) {
    RCLCPP_FATAL(this->get_logger(), "Low number of points in the cloud!");
    return;
  }

  // Compute Metrics
  this->computeMetrics();

  // Set Adaptive Parameters
  if (this->adaptive_params_) {
    this->setAdaptiveParams();
  }

  // Set new frame as input source
  this->setInputSource();

  // Set initial frame as first keyframe
  if (this->keyframes.size() == 0) {
    this->initializeInputTarget();
    { std::lock_guard<std::mutex> guard(main_loop_running_mutex); this->main_loop_running = false; }
    this->updateState();
    this->submap_future =
      std::async( std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, [this] {
        std::lock_guard<std::mutex> guard(geo.mtx); return state; }());
    this->submap_future.wait(); // wait until completion
    return;
  }

  // Get the next pose via IMU + S2M + GEO
  this->getNextPose();
  if (!scan_valid_) return;

  // Update current keyframe poses and map
  this->updateKeyframes();

  // Build keyframe normals and submap if needed (and if we're not already waiting)
  if (this->new_submap_is_ready) {
    { std::lock_guard<std::mutex> guard(main_loop_running_mutex); this->main_loop_running = false; }
    this->submap_future =
      std::async( std::launch::async, &dlio::OdomNode::buildKeyframesAndSubmap, this, [this] {
        std::lock_guard<std::mutex> guard(geo.mtx); return state; }());
  } else {
    lock.lock();
    this->main_loop_running = false;
    lock.unlock();
    this->submap_build_cv.notify_one();
  }

  // Update trajectory
  this->trajectory.push_back( std::make_pair(this->lidarPose.p, this->lidarPose.q) );
  if (trajectory.size() > static_cast<std::size_t>(path_capacity_)) trajectory.erase(trajectory.begin());

  // Update time stamps
  this->lidar_rates.push_back( 1. / (this->scan_stamp - this->prev_scan_stamp) );
  if (lidar_rates.size() > 200) lidar_rates.erase(lidar_rates.begin());
  this->prev_scan_stamp = this->scan_stamp;
  this->elapsed_time = this->scan_stamp - this->first_scan_stamp;

  // Publish stuff to ROS
  pcl::PointCloud<PointType>::ConstPtr published_cloud;
  if (this->densemap_filtered_) {
    published_cloud = this->current_scan;
  } else {
    published_cloud = this->deskewed_scan;
  }
  this->publishToROS(published_cloud, this->T_corr);

  // Update some statistics
  this->comp_times.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - then).count());
  if (comp_times.size() > 200) comp_times.erase(comp_times.begin());
  this->gicp_hasConverged = this->gicp.hasConverged();

  // Debug statements and publish custom DLIO message
  if (terminal_enabled_) this->debug();

}

void dlio::OdomNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {

  this->first_imu_received = true;

  sensor_msgs::msg::Imu::SharedPtr imu = this->transformImu( imu_raw );
  if (!imu) {
    return;
  }
  std::lock_guard<std::mutex> state_lock(geo.mtx);
  this->imu_stamp = imu->header.stamp;
  last_imu_arrival_ = std::chrono::steady_clock::now();
  double imu_stamp_secs = rclcpp::Time(imu->header.stamp).seconds();

  Eigen::Vector3f lin_accel;
  Eigen::Vector3f ang_vel;

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  lin_accel[0] = imu->linear_acceleration.x;
  lin_accel[1] = imu->linear_acceleration.y;
  lin_accel[2] = imu->linear_acceleration.z;

  if (this->first_imu_stamp == 0.) {
    this->first_imu_stamp = imu_stamp_secs;
  }

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {

    int &num_samples = calibration_samples_;
    Eigen::Vector3f &gyro_avg = calibration_gyro_sum_;
    Eigen::Vector3f &accel_avg = calibration_accel_sum_;
    static bool print = true;

    if ((imu_stamp_secs - this->first_imu_stamp) < this->imu_calib_time_) {

      num_samples++;

      gyro_avg[0] += ang_vel[0];
      gyro_avg[1] += ang_vel[1];
      gyro_avg[2] += ang_vel[2];

      accel_avg[0] += lin_accel[0];
      accel_avg[1] += lin_accel[1];
      accel_avg[2] += lin_accel[2];
      calibration_gyro_squared_ += ang_vel.cwiseProduct(ang_vel);
      calibration_accel_squared_ += lin_accel.cwiseProduct(lin_accel);

      if(print) {
        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... ";
        std::cout.flush();
        print = false;
      }

    } else {

      if (num_samples < 50) { first_imu_stamp = imu_stamp_secs; return; }
      const Eigen::Vector3f amean = accel_avg / num_samples, gmean = gyro_avg / num_samples;
      const float astd = (calibration_accel_squared_ / num_samples - amean.cwiseProduct(amean)).cwiseMax(0.f).cwiseSqrt().maxCoeff();
      const float gstd = (calibration_gyro_squared_ / num_samples - gmean.cwiseProduct(gmean)).cwiseMax(0.f).cwiseSqrt().maxCoeff();
      if (gmean.norm() > .05 || gstd > .02 || astd > .30 || std::abs(amean.norm() - gravity_) > .75) {
        RCLCPP_WARN(get_logger(), "DLIO initialization requires stationary IMU in m/s^2: norm %.3f, std %.3f, gyro %.3f",
            amean.norm(), astd, gmean.norm());
        num_samples = 0; gyro_avg.setZero(); accel_avg.setZero();
        calibration_gyro_squared_.setZero(); calibration_accel_squared_.setZero();
        first_imu_stamp = imu_stamp_secs;
        return;
      }
      std::cout << "done" << std::endl << std::endl;

      gyro_avg /= num_samples;
      accel_avg /= num_samples;

      Eigen::Vector3f grav_vec (0., 0., this->gravity_);

      if (this->gravity_align_) {

        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));

        // set gravity aligned orientation
        this->state.q = grav_q;
        this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
        this->lidarPose.q = this->state.q;

        // rpy
        auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw = euler[0] * (180.0/M_PI);
        double pitch = euler[1] * (180.0/M_PI);
        double roll = euler[2] * (180.0/M_PI);

        // use alternate representation if the yaw is smaller
        if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
          yaw   = remainder(yaw + 180.0,   360.0);
          pitch = remainder(180.0 - pitch, 360.0);
          roll  = remainder(roll + 180.0,  360.0);
        }
        std::cout << " Estimated initial attitude:" << std::endl;
        std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
        std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
        std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
        std::cout << std::endl;
      }

      if (this->calibrate_accel_) {

        // subtract gravity from avg accel to get bias
        this->state.b.accel = accel_avg - grav_vec;

        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
      }

      if (this->calibrate_gyro_) {

        this->state.b.gyro = gyro_avg;

        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                             << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
      }

      this->imu_calibrated = true;

    }

  } else {

    // The first post-calibration sample establishes the IMU time base.  There
    // is no preceding measurement from which a physical dt can be computed.
    const bool first_imu_sample = (this->prev_imu_stamp == 0.0);
    double dt = first_imu_sample ? (1.0 / 200.0) :
                                   (imu_stamp_secs - this->prev_imu_stamp);
    if (!first_imu_sample && dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping IMU sample with invalid dt: %.6f", dt);
      return;
    }
    if (dt > 0.05) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "IMU timestamp gap %.6f s; suspending propagation until next scan correction", dt);
      // Do not integrate across a scheduling/transport gap.  The timestamp
      // is still accepted and becomes the new baseline for the next sample.
      dt = 1.0 / 200.0;
      geo.first_opt_done = false;
      state_history_.clear();
    }
    this->imu_rates.push_back( 1./dt );
    if (imu_rates.size() > 200) imu_rates.erase(imu_rates.begin());

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu_stamp_secs;
    this->imu_meas.dt = dt;
    this->prev_imu_stamp = this->imu_meas.stamp;

    Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;
    Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;
    observer_imu_.push_back({imu_stamp_secs, dt, ang_vel, imu_accel_sm_ * lin_accel});
    while (!observer_imu_.empty() && observer_imu_.front().stamp < imu_stamp_secs - 3.) observer_imu_.pop_front();

    this->imu_meas.lin_accel = lin_accel_corrected;
    this->imu_meas.ang_vel = ang_vel_corrected;

    // Store calibrated IMU measurements into imu buffer for manual integration later.
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->mtx_imu.unlock();

    // Notify the callbackPointCloud thread that IMU data exists for this time
    this->cv_imu_stamp.notify_one();

    if (this->geo.first_opt_done) {
      // Geometric Observer: Propagate State
      this->propagateState();
    }

  }

}

void dlio::OdomNode::getNextPose() {

  // Check if the new submap is ready to be used
  this->new_submap_is_ready = (this->submap_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready);

  if (this->new_submap_is_ready && this->submap_hasChanged) {

    // Set the current global submap as the target cloud
    this->gicp.registerInputTarget(this->submap_cloud);

    // Set submap kdtree
    this->gicp.target_kdtree_ = this->submap_kdtree;

    // Set target cloud's normals as submap normals
    this->gicp.setTargetCovariances(this->submap_normals);

    this->submap_hasChanged = false;
  }

  // Align with current submap with global IMU transformation as initial guess
  pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();
  this->gicp.align(*aligned);

  if (!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite()) {
    scan_valid_ = false;
    gicp_hasConverged = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting failed GICP registration");
    return;
  }

  // Get final transformation in global frame
  this->T_corr = this->gicp.getFinalTransformation(); // "correction" transformation
  this->T = this->T_corr * this->T_prior;

  // Update next global pose
  // Both source and target clouds are in the global frame now, so tranformation is global
  this->propagateGICP();

  // Geometric observer update
  this->updateState();

}

bool dlio::OdomNode::imuMeasFromTimeRange(double start_time, double end_time,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {

  {
    std::unique_lock<std::mutex> lock(this->mtx_imu);
    const bool available = cv_imu_stamp.wait_for(lock, std::chrono::milliseconds(50), [this, end_time] {
      return stopping_ || !rclcpp::ok() || (!imu_buffer.empty() && imu_buffer.front().stamp >= end_time);
    });
    if (!available || stopping_ || !rclcpp::ok() || imu_buffer.size() < 2) return false;
    // Iterators must not point into a circular buffer concurrently overwritten
    // by the IMU callback. Integration uses a short-lock private snapshot.
    integration_buffer = imu_buffer;
  }

  auto imu_it = this->integration_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->integration_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->integration_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->integration_buffer.end()) {
    // not enough IMU measurements, return false
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
dlio::OdomNode::integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                             Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {
  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) return {};
  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it, end_imu_it;
  if (!imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it)) return {};
  const std::vector<dlio::MotionImu> samples(begin_imu_it, end_imu_it);
  return dlio::integrateMotion(start_time, q_init, p_init, v_init, sorted_timestamps, samples, gravity_);
}

void dlio::OdomNode::propagateGICP() {

  this->lidarPose.p << this->T(0,3), this->T(1,3), this->T(2,3);

  Eigen::Matrix3f rotSO3;
  rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
            this->T(1,0), this->T(1,1), this->T(1,2),
            this->T(2,0), this->T(2,1), this->T(2,2);

  Eigen::Quaternionf q(rotSO3);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->lidarPose.q = q;

}

void dlio::OdomNode::propagateState() {
  // Caller holds geo.mtx. During correction this also replays buffered IMUs.
  const double dt = imu_meas.stamp - state_stamp_;
  if (dt <= 0.0) return;
  if (dt > 0.05) { geo.first_opt_done = false; return; }
  const Eigen::Quaternionf before = state.q;
  const float speed = imu_meas.ang_vel.norm();
  if (speed > 1e-9f) state.q = state.q * Eigen::Quaternionf(
      Eigen::AngleAxisf(speed * dt, imu_meas.ang_vel / speed));
  state.q.normalize();
  Eigen::Vector3f acceleration = .5f * (before * imu_meas.lin_accel + state.q * imu_meas.lin_accel);
  acceleration.z() -= gravity_;
  state.p += state.v.lin.w * dt + .5 * acceleration * dt * dt;
  state.v.lin.w += acceleration * dt;
  state.v.lin.b = state.q.conjugate() * state.v.lin.w;
  state.v.ang.b = imu_meas.ang_vel;
  state.v.ang.w = state.q * state.v.ang.b;
  state_stamp_ = imu_meas.stamp;
  state_history_.push_back({state_stamp_, state});
  while (state_history_.size() > 2 && state_history_.front().stamp < state_stamp_ - 3.) state_history_.pop_front();
}

void dlio::OdomNode::updateState() {

  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock( this->geo.mtx );

  if (!geo.first_opt_done || state_history_.empty()) {
    state.p = lidarPose.p;
    state.q = lidarPose.q;
    state.v.lin.w.setZero();
  } else {
    const auto after = std::lower_bound(state_history_.begin(), state_history_.end(), scan_stamp,
        [](const TimedState &s, double t) { return s.stamp < t; });
    if (after == state_history_.end() || scan_stamp < state_history_.front().stamp) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Scan correction outside retained IMU history");
      return;
    }
    if (after == state_history_.begin()) state = after->value;
    else {
      const auto before = std::prev(after);
      const float f = (scan_stamp - before->stamp) / (after->stamp - before->stamp);
      state.p = (1.f-f)*before->value.p + f*after->value.p;
      state.q = before->value.q.slerp(f, after->value.q);
      state.v.lin.w = (1.f-f)*before->value.v.lin.w + f*after->value.v.lin.w;
    }
  }

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = this->scan_stamp - this->prev_scan_stamp;

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion
  qe = qhat.conjugate()*qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn*qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // Update state
  this->state.p += dt * this->geo_Kp_ * err;
  this->state.v.lin.w += dt * this->geo_Kv_ * err;

  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
  this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
  this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
  this->state.q.normalize();

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

  state_stamp_ = scan_stamp;
  correction_stamp_ = scan_stamp;
  state.v.lin.b = state.q.conjugate() * state.v.lin.w;
  state_history_.clear();
  state_history_.push_back({state_stamp_, state});
  geo.first_opt_done = true;
  const ImuMeas latest_measurement = imu_meas;
  for (const auto &raw : observer_imu_) {
    if (raw.stamp <= state_stamp_) continue;
    imu_meas = raw;
    imu_meas.ang_vel -= state.b.gyro;
    imu_meas.lin_accel -= state.b.accel;
    propagateState();
  }
  imu_meas = latest_measurement;

}

sensor_msgs::msg::Imu::SharedPtr dlio::OdomNode::transformImu(const sensor_msgs::msg::Imu::SharedPtr& imu_raw) {
  dlio::MotionImu raw, corrected;
  raw.stamp = rclcpp::Time(imu_raw->header.stamp).seconds();
  raw.ang_vel = {imu_raw->angular_velocity.x, imu_raw->angular_velocity.y, imu_raw->angular_velocity.z};
  raw.lin_accel = {imu_raw->linear_acceleration.x, imu_raw->linear_acceleration.y, imu_raw->linear_acceleration.z};
  imu_conditioner_.rotation = extrinsics.baselink2imu.R * fused_imu_rotation_;
  imu_conditioner_.lever = extrinsics.baselink2imu.t + fused_imu_lever_;
  imu_conditioner_.smoothing = lever_smoothing_seconds_;
  if (!imu_conditioner_.apply(raw, corrected)) return nullptr;
  auto output = std::make_shared<sensor_msgs::msg::Imu>(*imu_raw);
  output->angular_velocity.x = corrected.ang_vel.x(); output->angular_velocity.y = corrected.ang_vel.y(); output->angular_velocity.z = corrected.ang_vel.z();
  output->linear_acceleration.x = corrected.lin_accel.x(); output->linear_acceleration.y = corrected.lin_accel.y(); output->linear_acceleration.z = corrected.lin_accel.z();
  return output;
}

void dlio::OdomNode::computeMetrics() {
  this->computeSpaciousness();
  this->computeDensity();
}

void dlio::OdomNode::computeSpaciousness() {

  // compute range of points
  std::vector<float> ds;

  for (size_t i = 0; i < this->original_scan->points.size(); ++i) {
    float d = std::sqrt(pow(this->original_scan->points[i].x, 2) +
                        pow(this->original_scan->points[i].y, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
  float median_curr = ds[ds.size()/2];
  static float median_prev = median_curr;
  float median_lpf = 0.95*median_prev + 0.05*median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back( median_lpf );
  if (metrics.spaciousness.size() > 200) metrics.spaciousness.erase(metrics.spaciousness.begin());

}

void dlio::OdomNode::computeDensity() {

  float density;

  if (!this->geo.first_opt_done) {
    density = 0.;
  } else {
    density = this->gicp.source_density_;
  }

  static float density_prev = density;
  float density_lpf = 0.95*density_prev + 0.05*density;
  density_prev = density_lpf;

  this->metrics.density.push_back( density_lpf );
  if (metrics.density.size() > 200) metrics.density.erase(metrics.density.begin());

}

void dlio::OdomNode::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_processed_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = std::make_shared<pcl::PointCloud<PointType>>();

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points = std::make_shared<pcl::PointCloud<PointType>>();
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = std::make_shared<pcl::PointIndices>();
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}

void dlio::OdomNode::computeConcaveHull() {

  // at least 5 keyframes for concave hull
  if (this->num_processed_keyframes < 5) {
    return;
  }

  // create a pointcloud with points at keyframes
  auto cloud = std::make_shared<pcl::PointCloud<PointType>>();

  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    PointType pt;
    pt.x = this->keyframes[i].first.first[0];
    pt.y = this->keyframes[i].first.first[1];
    pt.z = this->keyframes[i].first.first[2];
    cloud->push_back(pt);
  }
  lock.unlock();

  // calculate the concave hull of the point cloud
  this->concave_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points = std::make_shared<pcl::PointCloud<PointType>>();
  this->concave_hull.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = std::make_shared<pcl::PointIndices>();
  this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

  this->keyframe_concave.clear();
  for (int i=0; i<concave_hull_point_idx->indices.size(); ++i) {
    this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
  }

}

void dlio::OdomNode::updateKeyframes() {

  // calculate difference in pose and rotation to all poses in trajectory
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = 0;
  int keyframes_idx = 0;

  int num_nearby = 0;

  for (const auto& k : this->keyframes) {

    // calculate distance between current pose and pose in keyframes
    float delta_d = sqrt( pow(this->lidarPose.p[0] - k.first.first[0], 2) +
                          pow(this->lidarPose.p[1] - k.first.first[1], 2) +
                          pow(this->lidarPose.p[2] - k.first.first[2], 2) );

    // count the number nearby current pose
    if (delta_d <= this->keyframe_thresh_dist_ * 1.5){
      ++num_nearby;
    }

    // store into variable
    if (delta_d < closest_d) {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }

    keyframes_idx++;

  }

  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
  Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt( pow(this->lidarPose.p[0] - closest_pose[0], 2) +
                   pow(this->lidarPose.p[1] - closest_pose[1], 2) +
                   pow(this->lidarPose.p[2] - closest_pose[2], 2) );

  // calculate difference in orientation using SLERP
  Eigen::Quaternionf dq;

  if (this->lidarPose.q.dot(closest_pose_r) < 0.) {
    Eigen::Quaternionf lq = closest_pose_r;
    lq.w() *= -1.; lq.x() *= -1.; lq.y() *= -1.; lq.z() *= -1.;
    dq = this->lidarPose.q * lq.inverse();
  } else {
    dq = this->lidarPose.q * closest_pose_r.inverse();
  }

  double theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
  double theta_deg = theta_rad * (180.0/M_PI);

  // update keyframes
  bool newKeyframe = false;

  if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
    newKeyframe = true;
  }

  if (abs(dd) <= this->keyframe_thresh_dist_) {
    newKeyframe = false;
  }

  if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_ && num_nearby <= 1) {
    newKeyframe = true;
  }

  if (newKeyframe) {

    // update keyframe vector
    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
    this->keyframes.push_back(std::make_pair(std::make_pair(this->lidarPose.p, this->lidarPose.q), this->current_scan));
    this->dense_keyframe_clouds.push_back(this->deskewed_scan);
    this->keyframe_timestamps.push_back(rclcpp::Time(static_cast<int64_t>(std::llround(scan_stamp * 1e9))));
    this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
    this->keyframe_transformations.push_back(this->T_corr);
    lock.unlock();

  }

}

void dlio::OdomNode::setAdaptiveParams() {

  // Spaciousness
  float sp = this->metrics.spaciousness.back();

  if (sp < 0.5) { sp = 0.5; }
  if (sp > 5.0) { sp = 5.0; }

  this->keyframe_thresh_dist_ = sp;

  // Density
  float den = this->metrics.density.back();

  if (den < 0.5*this->gicp_max_corr_dist_) { den = 0.5*this->gicp_max_corr_dist_; }
  if (den > 2.0*this->gicp_max_corr_dist_) { den = 2.0*this->gicp_max_corr_dist_; }

  if (sp < 5.0) { den = 0.5*this->gicp_max_corr_dist_; };
  if (sp > 5.0) { den = 2.0*this->gicp_max_corr_dist_; };

  this->gicp.setMaxCorrespondenceDistance(den);

  // Hull alpha stays at its configured value: the submap worker owns the
  // hull object, so changing it from the scan callback would race reconstruct.

}

void dlio::OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

  // make sure dists is not empty
  if (!dists.size()) { return; }

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists) {
    if (pq.size() >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (pq.size() < k) {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element)
      this->submap_kf_idx_curr.push_back(frames[i]);
  }

}

void dlio::OdomNode::buildSubmap(State vehicle_state) {

  // clear vector of keyframe indices to use for submap
  this->submap_kf_idx_curr.clear();

  // calculate distance between current pose and poses in keyframe set
  std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);
  std::vector<float> ds;
  std::vector<int> keyframe_nn;
  for (int i = 0; i < this->num_processed_keyframes; i++) {
    float d = sqrt( pow(vehicle_state.p[0] - this->keyframes[i].first.first[0], 2) +
                    pow(vehicle_state.p[1] - this->keyframes[i].first.first[1], 2) +
                    pow(vehicle_state.p[2] - this->keyframes[i].first.first[2], 2) );
    ds.push_back(d);
    keyframe_nn.push_back(i);
  }
  lock.unlock();

  // get indices for top K nearest neighbor keyframe poses
  this->pushSubmapIndices(ds, this->submap_knn_, keyframe_nn);

  // get convex hull indices
  this->computeConvexHull();

  // get distances for each keyframe on convex hull
  std::vector<float> convex_ds;
  for (const auto& c : this->keyframe_convex) {
    convex_ds.push_back(ds[c]);
  }

  // get indices for top kNN for convex hull
  this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);

  // get concave hull indices
  this->computeConcaveHull();

  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : this->keyframe_concave) {
    concave_ds.push_back(ds[c]);
  }

  // get indices for top kNN for concave hull
  this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

  // sort current and previous submap kf list of indices
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

  // remove duplicate indices
  auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

  // check if submap has changed from previous iteration
  if (this->submap_kf_idx_curr != this->submap_kf_idx_prev){

    this->submap_hasChanged = true;

    // Pause to prevent stealing resources from the main loop if it is running.
    this->pauseSubmapBuildIfNeeded();

    // reinitialize submap cloud and normals
    pcl::PointCloud<PointType>::Ptr submap_cloud_ = std::make_shared<pcl::PointCloud<PointType>>();
    std::shared_ptr<nano_gicp::CovarianceList> submap_normals_ (std::make_shared<nano_gicp::CovarianceList>());

    for (auto k : this->submap_kf_idx_curr) {

      // create current submap cloud
      lock.lock();
      *submap_cloud_ += *this->keyframes[k].second;
      lock.unlock();

      // grab corresponding submap cloud's normals
      submap_normals_->insert( std::end(*submap_normals_),
          std::begin(*(this->keyframe_normals[k])), std::end(*(this->keyframe_normals[k])) );
    }

    this->submap_cloud = submap_cloud_;
    this->submap_normals = submap_normals_;

    // Pause to prevent stealing resources from the main loop if it is running.
    this->pauseSubmapBuildIfNeeded();

    this->gicp_temp.setInputTarget(this->submap_cloud);
    this->submap_kdtree = this->gicp_temp.target_kdtree_;

    this->submap_kf_idx_prev = this->submap_kf_idx_curr;
  }
}

void dlio::OdomNode::buildKeyframesAndSubmap(State vehicle_state) {

  // transform the new keyframe(s) and associated covariance list(s)
    std::unique_lock<decltype(this->keyframes_mutex)> lock(this->keyframes_mutex);

  for (int i = this->num_processed_keyframes; i < this->keyframes.size(); i++) {
    pcl::PointCloud<PointType>::ConstPtr raw_keyframe = this->keyframes[i].second;
    pcl::PointCloud<PointType>::ConstPtr dense_keyframe = this->dense_keyframe_clouds[i];
    std::shared_ptr<const nano_gicp::CovarianceList> raw_covariances = this->keyframe_normals[i];
    Eigen::Matrix4f T = this->keyframe_transformations[i];
    lock.unlock();

    Eigen::Matrix4d Td = T.cast<double>();

    pcl::PointCloud<PointType>::Ptr transformed_keyframe = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud (*raw_keyframe, *transformed_keyframe, T);
    pcl::PointCloud<PointType>::Ptr transformed_dense_keyframe =
      std::make_shared<pcl::PointCloud<PointType>>();
    pcl::transformPointCloud(
      *dense_keyframe, *transformed_dense_keyframe, T);

    std::shared_ptr<nano_gicp::CovarianceList> transformed_covariances (std::make_shared<nano_gicp::CovarianceList>(raw_covariances->size()));
    std::transform(raw_covariances->begin(), raw_covariances->end(), transformed_covariances->begin(),
                   [&Td](Eigen::Matrix4d cov) { return Td * cov * Td.transpose(); });

    ++this->num_processed_keyframes;

    lock.lock();
    this->keyframes[i].second = transformed_keyframe;
    this->keyframe_normals[i] = transformed_covariances;
    auto published_keyframe = this->keyframes[i];
    published_keyframe.second = transformed_dense_keyframe;
    this->dense_keyframe_clouds[i].reset();

    if (this->publish_keyframes_) {
      this->publishKeyframeBundle(i, published_keyframe, this->keyframe_timestamps[i]);
    }
  }

  lock.unlock();

  // Pause to prevent stealing resources from the main loop if it is running.
  this->pauseSubmapBuildIfNeeded();

  this->buildSubmap(vehicle_state);
}

void dlio::OdomNode::pauseSubmapBuildIfNeeded() {
  std::unique_lock<decltype(this->main_loop_running_mutex)> lock(this->main_loop_running_mutex);
  this->submap_build_cv.wait(lock, [this]{ return stopping_ || !this->main_loop_running; });
}

void dlio::OdomNode::debug() {
  State debug_state;
  std::vector<double> debug_imu_rates;
  {
    std::lock_guard<std::mutex> lock(geo.mtx);
    debug_state = state;
    debug_imu_rates = imu_rates;
  }

  // Total length traversed
  double length_traversed = 0.;
  Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
  Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
  for (const auto& t : this->trajectory) {
    if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
      p_prev = t.first;
      continue;
    }
    p_curr = t.first;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.1) {
      length_traversed += l;
      p_prev = p_curr;
    }
  }
  this->length_traversed = length_traversed;

  // Average computation time
  double avg_comp_time =
    std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

  // Average sensor rates
  int win_size = 100;
  double avg_imu_rate;
  double avg_lidar_rate;
  if (debug_imu_rates.size() < win_size) {
    avg_imu_rate =
      std::accumulate(debug_imu_rates.begin(), debug_imu_rates.end(), 0.0) / debug_imu_rates.size();
  } else {
    avg_imu_rate =
      std::accumulate(debug_imu_rates.end()-win_size, debug_imu_rates.end(), 0.0) / win_size;
  }
  if (this->lidar_rates.size() < win_size) {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size();
  } else {
    avg_lidar_rate =
      std::accumulate(this->lidar_rates.end()-win_size, this->lidar_rates.end(), 0.0) / win_size;
  }

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
      timeSample.tms_utime < this->lastUserCPU) {
      cpu_percent = -1.0;
  } else {
      cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
      cpu_percent /= (now - this->lastCPU);
      cpu_percent /= this->numProcessors;
      cpu_percent *= 100.;
  }
  this->lastCPU = now;
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->cpu_percents.push_back(cpu_percent);
  if (cpu_percents.size() > 200) cpu_percents.erase(cpu_percents.begin());
  double avg_cpu_usage =
    std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

  // Print to terminal
  printf("\033[2J\033[1;1H");

  std::cout << std::endl
            << "+-------------------------------------------------------------------+" << std::endl;
  std::cout << "|               Direct LiDAR-Inertial Odometry v" << this->version_  << "               |"
            << std::endl;
  std::cout << "+-------------------------------------------------------------------+" << std::endl;

  std::time_t curr_time = this->scan_stamp;
  std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
  std::cout << "| " << std::left << asc_time;
  std::cout << std::right << std::setfill(' ') << std::setw(42)
    << "Elapsed Time: " + to_string_with_precision(this->elapsed_time, 2) + " seconds "
    << "|" << std::endl;

  if ( !this->cpu_type.empty() ) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << this->cpu_type + " x " + std::to_string(this->numProcessors)
      << "|" << std::endl;
  }

  if (this->sensor == dlio::SensorType::OUSTER) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
                                   + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                     + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::HESAI) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
                                  + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  } else {
    std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
      << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                          + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
      << "|" << std::endl;
  }

  std::cout << "|===================================================================|" << std::endl;
  std::cout << "| Health :: " << (deskew_status && gicp_hasConverged ? "GOOD" : "CHECK - deskew/registration")
            << " ; Odom/TF " << to_string_with_precision(output_rate_hz_.load(), 1) << " Hz\n";
  std::cout << "| Points :: " << original_scan->size() << " -> " << current_scan->size()
            << " / cap " << maximum_tracking_points_ << "; threads " << num_threads_ << "\n";

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Position     {W}  [xyz] :: " + to_string_with_precision(debug_state.p[0], 4) + " "
                                + to_string_with_precision(debug_state.p[1], 4) + " "
                                + to_string_with_precision(debug_state.p[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Orientation  {W} [wxyz] :: " + to_string_with_precision(debug_state.q.w(), 4) + " "
                                + to_string_with_precision(debug_state.q.x(), 4) + " "
                                + to_string_with_precision(debug_state.q.y(), 4) + " "
                                + to_string_with_precision(debug_state.q.z(), 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(debug_state.v.lin.b[0], 4) + " "
                                + to_string_with_precision(debug_state.v.lin.b[1], 4) + " "
                                + to_string_with_precision(debug_state.v.lin.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(debug_state.v.ang.b[0], 4) + " "
                                + to_string_with_precision(debug_state.v.ang.b[1], 4) + " "
                                + to_string_with_precision(debug_state.v.ang.b[2], 4)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Accel Bias        [xyz] :: " + to_string_with_precision(debug_state.b.accel[0], 8) + " "
                                + to_string_with_precision(debug_state.b.accel[1], 8) + " "
                                + to_string_with_precision(debug_state.b.accel[2], 8)
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Gyro Bias         [xyz] :: " + to_string_with_precision(debug_state.b.gyro[0], 8) + " "
                                + to_string_with_precision(debug_state.b.gyro[1], 8) + " "
                                + to_string_with_precision(debug_state.b.gyro[2], 8)
    << "|" << std::endl;

  std::cout << "|                                                                   |" << std::endl;

  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance Traveled  :: " + to_string_with_precision(length_traversed, 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Distance to Origin :: "
      + to_string_with_precision( sqrt(pow(debug_state.p[0]-this->origin[0],2) +
                                       pow(debug_state.p[1]-this->origin[1],2) +
                                       pow(debug_state.p[2]-this->origin[2],2)), 4) + " meters"
    << "|" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "Registration       :: keyframes: " + std::to_string(this->keyframes.size()) + ", "
                               + "deskewed points: " + std::to_string(this->deskew_size)
    << "|" << std::endl;
  std::cout << "|                                                                   |" << std::endl;

  std::cout << std::right << std::setprecision(2) << std::fixed;
  std::cout << "| Computation Time :: "
    << std::setfill(' ') << std::setw(6) << this->comp_times.back()*1000. << " ms    // Avg: "
    << std::setw(6) << avg_comp_time*1000. << " / Max: "
    << std::setw(6) << *std::max_element(this->comp_times.begin(), this->comp_times.end())*1000.
    << "     |" << std::endl;
  std::cout << "| Cores Utilized   :: "
    << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
    << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
    << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
                       * this->numProcessors
    << "     |" << std::endl;
  std::cout << "| CPU Load         :: "
    << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
    << std::setw(6) << avg_cpu_usage << " / Max: "
    << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
    << "     |" << std::endl;
  std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
    << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
    << "|" << std::endl;

  std::cout << "+-------------------------------------------------------------------+" << std::endl;

}
