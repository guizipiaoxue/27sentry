// 两台 MID360 与云台刚性固定，云台带着雷达绕云台坐标系中的固定轴做圆周运动。
// 先静止采集 startup_seconds（默认 1 s），每台雷达分别用自己的静态点云建立
// 局部地图；初始化阶段不做两雷达之间的 GICP。运动阶段每台雷达独立进行
// IMU 三轴角速度积分（作为 GICP 旋转初值）和“当前帧-自己的局部地图”GICP。
// 由各自轨迹拟合旋转轴和圆心，得到固定的 T_gimbal_lidar1/2，再计算
// T_lidar1_lidar2。实时扫描会被
// 变换到 gimbal 坐标系并发布 /gimbal/cloud_lidar{1,2} 与 /gimbal/cloud_fused，
// 可直接在 RViz 中查看。云台绝对 yaw 零位不可观，程序把启动时云台坐标系作为
// gimbal_calibration_frame；输出的相对外参与云台转角无关。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class RotateCali final : public rclcpp::Node {
 public:
  using Msg = livox_ros_driver2::msg::CustomMsg;
  using Imu = sensor_msgs::msg::Imu;
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using Iso = Eigen::Isometry3d;

  struct LidarState {
    Cloud::Ptr static_cloud{new Cloud};
    rclcpp::Time static_start_stamp{0, 0, RCL_ROS_TIME};
    bool static_started = false;
    bool static_ready = false;
    Cloud::Ptr map{new Cloud};
    Iso pose_local = Iso::Identity();     // ref_i_T_lidar_i at current frame.
    bool initialized = false;
    Eigen::Quaterniond imu_rotation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond used_imu_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    std::size_t gyro_count = 0;
    rclcpp::Time last_imu_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time first_imu_stamp{0, 0, RCL_ROS_TIME};
    bool imu_started = false;
    std::deque<std::pair<Iso, double>> extrinsics;
    std::deque<std::pair<Iso, double>> trajectory;
    std::deque<std::pair<rclcpp::Time, Cloud::Ptr>> pending_clouds;
    Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
    double circle_residual = std::numeric_limits<double>::infinity();
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -1.0);
    Cloud::Ptr latest_gimbal_cloud{new Cloud};
    rclcpp::Time latest_stamp{0, 0, RCL_ROS_TIME};
    std::size_t map_updates = 0;
    double min_angle = 0.0;
    double max_angle = 0.0;
  };

  RotateCali() : Node("rotate_cali") {
    cloud_topic_[0] = declare_parameter<std::string>("lidar1_topic", "livox/lidar_192_168_1_5");
    cloud_topic_[1] = declare_parameter<std::string>("lidar2_topic", "livox/lidar_192_168_1_3");
    imu_topic_[0] = declare_parameter<std::string>("imu1_topic", "livox/imu_192_168_1_5");
    imu_topic_[1] = declare_parameter<std::string>("imu2_topic", "livox/imu_192_168_1_3");
    output_file_ = declare_parameter<std::string>("output_file", "lidar_calibration.yaml");
    startup_seconds_ = declare_parameter<double>("startup_seconds", 1.0);
    sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.03);
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.08);
    max_correspondence_ = declare_parameter<double>("max_correspondence", 1.0);
    max_fitness_ = declare_parameter<double>("max_fitness", 0.25);
    min_points_ = declare_parameter<int>("min_points", 150);
    window_size_ = declare_parameter<int>("window_size", 40);
    max_map_points_ = declare_parameter<int>("max_map_points", 200000);
    min_yaw_excitation_deg_ = declare_parameter<double>("min_yaw_excitation_deg", 60.0);
    convergence_translation_std_ = declare_parameter<double>("convergence_translation_std", 0.01);
    convergence_rotation_std_deg_ = declare_parameter<double>("convergence_rotation_std_deg", 0.2);
    convergence_stable_reports_ = declare_parameter<int>("convergence_stable_reports", 5);
    auto qos = rclcpp::SensorDataQoS();
    cloud_sub_[0] = create_subscription<Msg>(cloud_topic_[0], qos, [this](Msg::ConstSharedPtr m) { cloudCallback(0, m); });
    cloud_sub_[1] = create_subscription<Msg>(cloud_topic_[1], qos, [this](Msg::ConstSharedPtr m) { cloudCallback(1, m); });
    imu_sub_[0] = create_subscription<Imu>(imu_topic_[0], qos, [this](Imu::ConstSharedPtr m) { imuCallback(0, m); });
    imu_sub_[1] = create_subscription<Imu>(imu_topic_[1], qos, [this](Imu::ConstSharedPtr m) { imuCallback(1, m); });
    pub_cloud_[0] = create_publisher<sensor_msgs::msg::PointCloud2>("/gimbal/cloud_lidar1", 10);
    pub_cloud_[1] = create_publisher<sensor_msgs::msg::PointCloud2>("/gimbal/cloud_lidar2", 10);
    pub_fused_ = create_publisher<sensor_msgs::msg::PointCloud2>("/gimbal/cloud_fused", 10);
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/gimbal/map", 1);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { reportAndPublish(); });
    RCLCPP_INFO(get_logger(), "topics: %s, %s; IMU: %s, %s", cloud_topic_[0].c_str(), cloud_topic_[1].c_str(), imu_topic_[0].c_str(), imu_topic_[1].c_str());
  }

 private:
  static Cloud::Ptr toCloud(const Msg &msg) {
    auto cloud = std::make_shared<Cloud>();
    cloud->reserve(msg.points.size());
    for (const auto &p : msg.points) {
      const bool valid = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      const double range_sq = p.x * p.x + p.y * p.y + p.z * p.z;
      if (valid && range_sq > 1e-6) {
        cloud->push_back({p.x, p.y, p.z});
      }
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  Cloud::Ptr downsample(const Cloud::Ptr &in) const {
    auto out = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(static_cast<float>(voxel_leaf_),
                   static_cast<float>(voxel_leaf_),
                   static_cast<float>(voxel_leaf_));
    vg.filter(*out);
    return out;
  }

  static Eigen::Matrix4f rotationMatrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = q.normalized().toRotationMatrix().cast<float>();
    return m;
  }

  void imuCallback(int id, Imu::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &s = lidar_[id];
    const rclcpp::Time stamp(msg->header.stamp);
    const double gz = msg->angular_velocity.z;
    const bool valid_gyro = std::isfinite(msg->angular_velocity.x) &&
                            std::isfinite(msg->angular_velocity.y) &&
                            std::isfinite(gz);
    if (!valid_gyro) {
      return;
    }
    if (!s.imu_started) {
      s.imu_started = true;
      s.first_imu_stamp = stamp;
      s.last_imu_stamp = stamp;
    }
    const double dt = (stamp - s.last_imu_stamp).seconds();
    s.last_imu_stamp = stamp;
    const Eigen::Vector3d gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
    if (!calibration_started_ || !initialized_) {
      const bool in_startup = (stamp - s.first_imu_stamp).seconds() <= startup_seconds_;
      if (in_startup && dt >= 0.0 && dt < 0.2) {
        s.gyro_sum += gyro;
        ++s.gyro_count;
        s.gyro_bias = s.gyro_sum / static_cast<double>(s.gyro_count);
      }
      const bool valid_accel = std::isfinite(msg->linear_acceleration.x) &&
                               std::isfinite(msg->linear_acceleration.y) &&
                               std::isfinite(msg->linear_acceleration.z);
      if (in_startup && dt >= 0.0 && dt < 0.2 && valid_accel) {
        s.accel_sum += Eigen::Vector3d(msg->linear_acceleration.x,
                                       msg->linear_acceleration.y,
                                       msg->linear_acceleration.z);
      }
      return;
    }
    if (dt > 0.0 && dt < 0.2) {
      const Eigen::Vector3d w = gyro - s.gyro_bias;
      const double a = w.norm() * dt;
      if (a > 1e-12) {
        s.imu_rotation = (s.imu_rotation *
                          Eigen::Quaterniond(Eigen::AngleAxisd(a, w.normalized())))
                             .normalized();
      }
    }
  }

  void cloudCallback(int id, Msg::ConstSharedPtr msg) {
    auto cloud = toCloud(*msg);
    if (cloud->size() < static_cast<std::size_t>(min_points_)) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!calibration_started_) {
      calibration_started_ = true;
      start_stamp_ = stamp;
    }
    if (!initialized_) {
      auto &state = lidar_[id];
      if (!state.static_started) {
        state.static_started = true;
        state.static_start_stamp = stamp;
        RCLCPP_INFO(get_logger(), "lidar%d static initialization started at %.3f", id + 1, stamp.seconds());
      }
      if ((stamp - state.static_start_stamp).seconds() <= startup_seconds_) {
        *state.static_cloud += *cloud;
      }
      if ((stamp - state.static_start_stamp).seconds() >= startup_seconds_ &&
          state.static_cloud->size() >= static_cast<std::size_t>(min_points_)) {
        state.static_ready = true;
      }
      if (lidar_[0].static_ready && lidar_[1].static_ready) {
        initializeLocked();
      }
      return;
    }

    lidar_[id].pending_clouds.push_back({stamp, downsample(cloud)});
    synchronizeCloudsLocked();
  }

  void synchronizeCloudsLocked() {
    auto &first_queue = lidar_[0].pending_clouds;
    auto &second_queue = lidar_[1].pending_clouds;
    while (!first_queue.empty() && !second_queue.empty()) {
      const double dt =
          (first_queue.front().first - second_queue.front().first).seconds();
      if (std::abs(dt) <= sync_tolerance_) {
        const auto first = first_queue.front();
        const auto second = second_queue.front();
        first_queue.pop_front();
        second_queue.pop_front();
        processCloudLocked(0, first.first, first.second);
        processCloudLocked(1, second.first, second.second);
      } else if (dt < 0.0) {
        first_queue.pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "dropping lidar1 cloud: timestamp difference %.4f s exceeds tolerance %.4f s",
            std::abs(dt), sync_tolerance_);
      } else {
        second_queue.pop_front();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "dropping lidar2 cloud: timestamp difference %.4f s exceeds tolerance %.4f s",
            std::abs(dt), sync_tolerance_);
      }
    }
  }

  bool registerCloud(const Cloud::Ptr &target, const Cloud::Ptr &source, const Eigen::Matrix4f &guess, Iso &pose, double &fitness) const {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> g;
    g.setInputSource(source);
    g.setInputTarget(target);
    g.setMaxCorrespondenceDistance(max_correspondence_);
    g.setMaximumIterations(80);
    g.setTransformationEpsilon(1e-6);
    g.setEuclideanFitnessEpsilon(1e-5);

    Cloud aligned;
    g.align(aligned, guess);
    fitness = g.getFitnessScore();
    if (!g.hasConverged() || !std::isfinite(fitness) || fitness > max_fitness_) {
      return false;
    }

    const Eigen::Matrix4f f = g.getFinalTransformation();
    pose = Iso::Identity();
    pose.linear() = f.block<3, 3>(0, 0).cast<double>();
    pose.translation() = f.block<3, 1>(0, 3).cast<double>();
    return true;
  }

  void initializeLocked() {
    // Each lidar is initialized against its own static map.  There is deliberately
    // no lidar1/lidar2 GICP at startup: their relative pose is solved from the
    // two independently estimated trajectories below.
    for (auto &s:lidar_) {
      s.map = downsample(s.static_cloud);
      s.static_cloud.reset(new Cloud);
      if (s.map->size() < static_cast<std::size_t>(min_points_)) {
        return;
      }
      s.pose_local = Iso::Identity();
      s.imu_rotation = Eigen::Quaterniond::Identity();
      s.used_imu_rotation = Eigen::Quaterniond::Identity();
      if (s.gyro_count > 0 && s.accel_sum.norm() > 1e-9) {
        s.gravity = s.accel_sum.normalized();
      }
      s.extrinsics.clear();
      s.trajectory.clear();
      s.pending_clouds.clear();
      s.initialized = true;
      s.min_angle = 0.0;
      s.max_angle = 0.0;
    }
    initialized_ = true;
    RCLCPP_INFO(get_logger(),
                "static %.1f s ready; independent maps=%zu/%zu (no cross-lidar GICP)",
                startup_seconds_, lidar_[0].map->size(), lidar_[1].map->size());
    RCLCPP_INFO(get_logger(),
                "motion processing uses synchronized cloud pairs (tolerance %.3f s)",
                sync_tolerance_);
  }

  void processCloudLocked(int id, const rclcpp::Time &stamp, const Cloud::Ptr &cloud) {
    auto &s = lidar_[id];
    const Eigen::Quaterniond delta = s.used_imu_rotation.inverse() * s.imu_rotation;
    const Eigen::Matrix4f guess = s.pose_local.matrix().cast<float>() * rotationMatrix(delta);
    Iso local_pose;
    double fitness = 0.0;
    if (!registerCloud(s.map, cloud, guess, local_pose, fitness)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "lidar%d GICP rejected (fitness %.4f)", id + 1, fitness);
      return;
    }
    s.pose_local = local_pose;
    s.used_imu_rotation = s.imu_rotation;
    s.trajectory.push_back({local_pose, fitness});
    while (s.trajectory.size() > static_cast<std::size_t>(window_size_)) {
      s.trajectory.pop_front();
    }
    const double angle = Eigen::AngleAxisd(local_pose.rotation()).angle();
    s.min_angle = std::min(s.min_angle, angle);
    s.max_angle = std::max(s.max_angle, angle);
    ++s.map_updates;

    Cloud::Ptr transformed(new Cloud);
    pcl::transformPointCloud(*cloud, *transformed, local_pose.matrix().cast<float>());
    *s.map += *transformed;
    if (s.map->size() > static_cast<std::size_t>(max_map_points_) || s.map_updates % 10 == 0) {
      s.map = downsample(s.map);
    }

    const Iso ext = estimateExtrinsic(s);
    addExtrinsicLocked(id, ext, fitness);
    const Iso current = ext * s.pose_local;
    Cloud::Ptr gimbal_cloud(new Cloud);
    pcl::transformPointCloud(*cloud, *gimbal_cloud, current.matrix().cast<float>());
    s.latest_gimbal_cloud = gimbal_cloud;
    s.latest_stamp = stamp;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "lidar%d: own-map GICP fitness=%.4f circle_residual=%.4f m",
                         id + 1, fitness, s.circle_residual);
  }

  // Fit the fixed screw motion of a lidar mounted on a rotating gimbal.  In the
  // lidar startup frame p_k = R_k p_0 + (c - R_k c), so the translations lie on
  // a circle about the fixed axis through c.  This does not assume that the axis
  // is Z; the axis is obtained from the GICP rotations themselves.
  Iso estimateExtrinsic(LidarState &s) const {
    if (s.trajectory.empty()) {
      return Iso::Identity();
    }
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    for (const auto &sample : s.trajectory) {
      const Eigen::AngleAxisd aa(sample.first.rotation());
      if (aa.angle() > 1e-4) {
        Eigen::Vector3d a = aa.axis();
        if (axis.dot(a) < 0) {
          a = -a;
        }
        axis += a * aa.angle();
      }
    }
    if (axis.norm() < 1e-9) {
      axis = Eigen::Vector3d::UnitZ();
    } else {
      axis.normalize();
    }
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const auto &sample : s.trajectory) {
      const Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - sample.first.rotation();
      H += A.transpose() * A;
      b += A.transpose() * sample.first.translation();
    }
    Eigen::Vector3d center = H.ldlt().solve(b);
    if (!center.allFinite()) {
      center = Eigen::Vector3d::Zero();
    }
    s.rotation_axis = axis;
    s.circle_center = center;
    double err = 0.0;
    for (const auto &sample : s.trajectory) {
      const Eigen::Vector3d d = sample.first.translation() -
                                (center - sample.first.rotation() * center);
      err += d.squaredNorm();
    }
    s.circle_residual = std::sqrt(err / static_cast<double>(s.trajectory.size()));
    Eigen::Vector3d x = s.gravity - axis * axis.dot(s.gravity);
    if (x.norm() < 1e-6) {
      x = Eigen::Vector3d::UnitY() - axis * axis.dot(Eigen::Vector3d::UnitY());
    }
    x.normalize();
    const Eigen::Vector3d y = axis.cross(x).normalized();
    Eigen::Matrix3d R;
    R.row(0) = x.transpose();
    R.row(1) = y.transpose();
    R.row(2) = axis.transpose();
    Iso ext = Iso::Identity();
    ext.linear() = R;
    ext.translation() = -R * center;
    return ext;
  }

  static Iso relativeFromCircularModels(
      const LidarState &a,
      const LidarState &b) {
    return averageExtrinsic(a.extrinsics).inverse()*averageExtrinsic(b.extrinsics);
  }

  void addExtrinsicLocked(int id, const Iso &ext, double fit) {
    auto &q = lidar_[id].extrinsics;
    q.push_back({ext, fit});
    while (q.size() > static_cast<std::size_t>(window_size_)) {
      q.pop_front();
    }
  }

  static Iso averageExtrinsic(const std::deque<std::pair<Iso,double>> &q) {
    if (q.empty()) {
      return Iso::Identity();
    }
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Quaterniond sum(0, 0, 0, 0);
    const Eigen::Quaterniond ref(q.front().first.rotation());
    for (const auto &m : q) {
      t += m.first.translation();
      Eigen::Quaterniond x(m.first.rotation());
      if (x.dot(ref) < 0) {
        x.coeffs() *= -1;
      }
      sum.coeffs() += x.coeffs();
    }
    t /= static_cast<double>(q.size());
    sum.normalize();
    Iso out = Iso::Identity();
    out.linear() = sum.toRotationMatrix();
    out.translation() = t;
    return out;
  }

  struct ConvergenceStats {
    double translation_std = std::numeric_limits<double>::infinity();
    double rotation_std_deg = std::numeric_limits<double>::infinity();
    double fitness_mean = std::numeric_limits<double>::infinity();
  };

  static ConvergenceStats convergenceStats(
      const std::deque<std::pair<Iso, double>> &q,
      const Iso &mean) {
    ConvergenceStats out;
    if (q.empty()) {
      return out;
    }
    double t2 = 0.0;
    double r2 = 0.0;
    double fit = 0.0;
    const Eigen::Quaterniond qm(mean.rotation());
    for (const auto &m : q) {
      t2 += (m.first.translation() - mean.translation()).squaredNorm();
      const Eigen::Quaterniond qi(m.first.rotation());
      const double dot = std::clamp(std::abs(qi.dot(qm)), 0.0, 1.0);
      const double angle = 2.0 * std::acos(dot);
      r2 += angle * angle;
      fit += m.second;
    }
    out.translation_std = std::sqrt(t2 / static_cast<double>(q.size()));
    out.rotation_std_deg = std::sqrt(r2 / static_cast<double>(q.size())) * 180.0 / M_PI;
    out.fitness_mean = fit / static_cast<double>(q.size());
    return out;
  }

  void publishCloud(
      const Cloud::Ptr &cloud,
      const rclcpp::Time &stamp,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub) const {
    if (!cloud || cloud->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = "gimbal";
    pub->publish(msg);
  }

  void reportAndPublish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "[WAITING] collecting the initial static data");
      return;
    }

    const Iso e1 = averageExtrinsic(lidar_[0].extrinsics);
    const Iso e2 = averageExtrinsic(lidar_[1].extrinsics);
    const Iso rel = relativeFromCircularModels(lidar_[0], lidar_[1]);
    const auto st1 = convergenceStats(lidar_[0].extrinsics, e1);
    const auto st2 = convergenceStats(lidar_[1].extrinsics, e2);

    const double yaw1 =
        (lidar_[0].max_angle - lidar_[0].min_angle) * 180.0 / M_PI;
    const double yaw2 =
        (lidar_[1].max_angle - lidar_[1].min_angle) * 180.0 / M_PI;
    const bool full =
        lidar_[0].extrinsics.size() >= static_cast<std::size_t>(window_size_) &&
        lidar_[1].extrinsics.size() >= static_cast<std::size_t>(window_size_);
    const bool yaw_ok = std::min(yaw1, yaw2) >= min_yaw_excitation_deg_;
    const bool stable =
        full && yaw_ok &&
        st1.translation_std <= convergence_translation_std_ &&
        st2.translation_std <= convergence_translation_std_ &&
        st1.rotation_std_deg <= convergence_rotation_std_deg_ &&
        st2.rotation_std_deg <= convergence_rotation_std_deg_;

    if (stable) {
      ++stable_reports_;
    } else {
      stable_reports_ = 0;
    }
    converged_ = stable_reports_ >= convergence_stable_reports_;
    writeYaml(e1, e2, rel, st1, st2, yaw1, yaw2);

    publishCloud(
        lidar_[0].latest_gimbal_cloud,
        lidar_[0].latest_stamp,
        pub_cloud_[0]);
    publishCloud(
        lidar_[1].latest_gimbal_cloud,
        lidar_[1].latest_stamp,
        pub_cloud_[1]);

    if (!lidar_[0].latest_gimbal_cloud->empty() &&
        !lidar_[1].latest_gimbal_cloud->empty()) {
      auto fused = std::make_shared<Cloud>();
      *fused = *lidar_[0].latest_gimbal_cloud;
      *fused += *lidar_[1].latest_gimbal_cloud;
      publishCloud(fused, get_clock()->now(), pub_fused_);
    }

    auto map = std::make_shared<Cloud>();
    pcl::transformPointCloud(
        *lidar_[0].map, *map, e1.matrix().cast<float>());
    Cloud lidar2_map;
    pcl::transformPointCloud(
        *lidar_[1].map, lidar2_map, e2.matrix().cast<float>());
    *map += lidar2_map;
    publishCloud(map, get_clock()->now(), pub_map_);

    RCLCPP_INFO(
        get_logger(),
        "[%s] samples=%zu/%zu yaw=%.1f/%.1f deg "
        "t_std=%.4f/%.4f m r_std=%.3f/%.3f deg stable=%d/%d",
        converged_ ? "CONVERGED" : "CALIBRATING",
        lidar_[0].extrinsics.size(),
        lidar_[1].extrinsics.size(),
        yaw1,
        yaw2,
        st1.translation_std,
        st2.translation_std,
        st1.rotation_std_deg,
        st2.rotation_std_deg,
        stable_reports_,
        convergence_stable_reports_);
  }

  static void writeMatrix(
      std::ofstream &output,
      const char *name,
      const Iso &transform) {
    output << name << ":\n";
    for (int row = 0; row < 4; ++row) {
      output << "  [";
      for (int col = 0; col < 4; ++col) {
        output << transform.matrix()(row, col)
               << (col == 3 ? "]\n" : ", ");
      }
    }
  }

  void writeYaml(
      const Iso &e1,
      const Iso &e2,
      const Iso &rel,
      const ConvergenceStats &s1,
      const ConvergenceStats &s2,
      double yaw1,
      double yaw2) const {
    std::ofstream output(output_file_);
    if (!output) {
      return;
    }
    output << std::setprecision(12)
           << "# gimbal frame = frame at startup; common yaw motion is removed\n"
           << "converged: " << (converged_ ? "true" : "false") << "\n"
           << "stable_reports: " << stable_reports_ << "\n"
           << "yaw_excitation_deg: [" << yaw1 << ", " << yaw2 << "]\n"
           << "translation_std_m: [" << s1.translation_std << ", "
           << s2.translation_std << "]\n"
           << "rotation_std_deg: [" << s1.rotation_std_deg << ", "
           << s2.rotation_std_deg << "]\n"
           << "fitness_mean: [" << s1.fitness_mean << ", "
           << s2.fitness_mean << "]\n";
    writeMatrix(output, "T_gimbal_lidar1", e1);
    writeMatrix(output, "T_gimbal_lidar2", e2);
    writeMatrix(output, "T_lidar1_lidar2", rel);
    output << "# p_lidar1 = T_lidar1_lidar2 * p_lidar2\n";
  }

  std::string cloud_topic_[2];
  std::string imu_topic_[2];
  std::string output_file_;

  double startup_seconds_;
  double sync_tolerance_;
  double voxel_leaf_;
  double max_correspondence_;
  double max_fitness_;
  double min_yaw_excitation_deg_;
  double convergence_translation_std_;
  double convergence_rotation_std_deg_;

  int min_points_;
  int window_size_;
  int max_map_points_;
  int convergence_stable_reports_;
  int stable_reports_ = 0;

  bool calibration_started_ = false;
  bool initialized_ = false;
  bool converged_ = false;

  rclcpp::Time start_stamp_{0, 0, RCL_ROS_TIME};
  LidarState lidar_[2];
  mutable std::mutex mutex_;

  rclcpp::Subscription<Msg>::SharedPtr cloud_sub_[2];
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_fused_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotateCali>());
  rclcpp::shutdown();
  return 0;
}
