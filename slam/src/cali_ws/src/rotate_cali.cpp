// 两台 MID360 与云台刚性固定，云台只绕 Z 轴 yaw 旋转。
// 按 rotate_cali_node.cpp 的流程：先静止采集 startup_seconds（默认 1 s），
// 用静态点云建立两张局部地图并用 GICP 求两雷达初始相对位姿；随后对每台
// 雷达分别进行 IMU z 轴角速度积分（作为 GICP 旋转初值）和“当前帧-局部地图”
// GICP 迭代建图。由 GICP 位姿的共同 yaw 运动消除云台转角，得到固定的
// T_gimbal_lidar1、T_gimbal_lidar2，再计算 T_lidar1_lidar2。实时扫描会被
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
    Cloud::Ptr map{new Cloud};
    Iso initial_world = Iso::Identity();  // W_T_ref_i, W is lidar1 at startup.
    Iso pose_local = Iso::Identity();     // ref_i_T_lidar_i at current frame.
    bool initialized = false;
    double imu_yaw = 0.0;
    double used_imu_yaw = 0.0;
    double gyro_bias_z = 0.0;
    double gyro_sum_z = 0.0;
    std::size_t gyro_count = 0;
    rclcpp::Time last_imu_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time first_imu_stamp{0, 0, RCL_ROS_TIME};
    bool imu_started = false;
    std::deque<std::pair<Iso, double>> extrinsics;
    Cloud::Ptr latest_gimbal_cloud{new Cloud};
    rclcpp::Time latest_stamp{0, 0, RCL_ROS_TIME};
    std::size_t map_updates = 0;
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
    auto cloud = std::make_shared<Cloud>(); cloud->reserve(msg.points.size());
    for (const auto &p : msg.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.x*p.x+p.y*p.y+p.z*p.z > 1e-6)
        cloud->push_back({p.x, p.y, p.z});
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size()); cloud->height = 1; cloud->is_dense = true; return cloud;
  }
  Cloud::Ptr downsample(const Cloud::Ptr &in) const {
    auto out = std::make_shared<Cloud>(); pcl::VoxelGrid<pcl::PointXYZ> vg; vg.setInputCloud(in);
    vg.setLeafSize(static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_)); vg.filter(*out); return out;
  }
  static Eigen::Matrix4f yawMatrix(double yaw) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity(); const float c=std::cos(static_cast<float>(yaw)), s=std::sin(static_cast<float>(yaw)); m(0,0)=c; m(0,1)=-s; m(1,0)=s; m(1,1)=c; return m;
  }
  void imuCallback(int id, Imu::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_); auto &s=lidar_[id]; const rclcpp::Time stamp(msg->header.stamp); const double gz=msg->angular_velocity.z;
    if (!std::isfinite(gz)) return;
    if (!s.imu_started) { s.imu_started=true; s.first_imu_stamp=stamp; s.last_imu_stamp=stamp; }
    const double dt=(stamp-s.last_imu_stamp).seconds(); s.last_imu_stamp=stamp;
    if (!calibration_started_ || !initialized_) {
      if ((stamp-s.first_imu_stamp).seconds() <= startup_seconds_ && dt >= 0.0 && dt < 0.2) { s.gyro_sum_z += gz; ++s.gyro_count; s.gyro_bias_z=s.gyro_sum_z/static_cast<double>(s.gyro_count); }
      return;
    }
    if (dt > 0.0 && dt < 0.2) s.imu_yaw += (gz-s.gyro_bias_z)*dt;
  }
  void cloudCallback(int id, Msg::ConstSharedPtr msg) {
    auto cloud=toCloud(*msg); if (cloud->size() < static_cast<std::size_t>(min_points_)) return; const rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!calibration_started_) { calibration_started_=true; start_stamp_=stamp; }
    if (!initialized_) {
      *lidar_[id].static_cloud += *cloud;
      if ((stamp-start_stamp_).seconds() >= startup_seconds_ && lidar_[0].static_cloud->size() >= static_cast<std::size_t>(min_points_) && lidar_[1].static_cloud->size() >= static_cast<std::size_t>(min_points_)) initializeLocked();
      return;
    }
    processCloudLocked(id, stamp, downsample(cloud));
  }
  bool registerCloud(const Cloud::Ptr &target, const Cloud::Ptr &source, const Eigen::Matrix4f &guess, Iso &pose, double &fitness) const {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> g; g.setInputSource(source); g.setInputTarget(target); g.setMaxCorrespondenceDistance(max_correspondence_); g.setMaximumIterations(80); g.setTransformationEpsilon(1e-6); g.setEuclideanFitnessEpsilon(1e-5); Cloud aligned; g.align(aligned, guess); fitness=g.getFitnessScore();
    if (!g.hasConverged() || !std::isfinite(fitness) || fitness>max_fitness_) return false; const Eigen::Matrix4f f=g.getFinalTransformation(); pose=Iso::Identity(); pose.linear()=f.block<3,3>(0,0).cast<double>(); pose.translation()=f.block<3,1>(0,3).cast<double>(); return true;
  }
  void initializeLocked() {
    lidar_[0].map=downsample(lidar_[0].static_cloud); lidar_[1].map=downsample(lidar_[1].static_cloud); lidar_[0].static_cloud.reset(new Cloud); lidar_[1].static_cloud.reset(new Cloud);
    if (lidar_[0].map->size()<static_cast<std::size_t>(min_points_) || lidar_[1].map->size()<static_cast<std::size_t>(min_points_)) return;
    Iso ref12; double fit=0.0; if (!registerCloud(lidar_[0].map,lidar_[1].map,Eigen::Matrix4f::Identity(),ref12,fit)) { RCLCPP_WARN(get_logger(), "initial lidar registration failed (fitness %.4f)",fit); return; }
    lidar_[0].initial_world=Iso::Identity(); lidar_[1].initial_world=ref12; lidar_[0].pose_local=Iso::Identity(); lidar_[1].pose_local=Iso::Identity();
    for (auto &s:lidar_) { s.initialized=true; s.imu_yaw=0.0; s.used_imu_yaw=0.0; s.extrinsics.clear(); }
    addExtrinsicLocked(0,Iso::Identity(),fit); addExtrinsicLocked(1,ref12,fit); initialized_=true;
    RCLCPP_INFO(get_logger(), "static %.1f s ready; maps=%zu/%zu, initial fitness=%.4f",startup_seconds_,lidar_[0].map->size(),lidar_[1].map->size(),fit);
  }
  void processCloudLocked(int id, const rclcpp::Time &stamp, const Cloud::Ptr &cloud) {
    auto &s=lidar_[id]; const double d_yaw=s.imu_yaw-s.used_imu_yaw; const Eigen::Matrix4f guess=s.pose_local.matrix().cast<float>()*yawMatrix(d_yaw); Iso local_pose; double fitness=0.0;
    if (!registerCloud(s.map,cloud,guess,local_pose,fitness)) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"lidar%d GICP rejected (fitness %.4f)",id+1,fitness); return; }
    s.pose_local=local_pose; s.used_imu_yaw=s.imu_yaw; ++s.map_updates;
    Cloud::Ptr transformed(new Cloud); pcl::transformPointCloud(*cloud,*transformed,local_pose.matrix().cast<float>()); *s.map += *transformed;
    if (s.map->size()>static_cast<std::size_t>(max_map_points_) || s.map_updates%10==0) s.map=downsample(s.map);
    const Iso world_pose=s.initial_world*s.pose_local; const Eigen::Matrix3d motion=world_pose.rotation()*s.initial_world.rotation().transpose(); const double yaw=std::atan2(motion(1,0),motion(0,0)); const Iso world_gimbal(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())); const Iso ext=world_gimbal.inverse()*world_pose;
    addExtrinsicLocked(id,ext,fitness); const Iso avg=averageExtrinsic(lidar_[id].extrinsics); Cloud::Ptr gimbal_cloud(new Cloud); pcl::transformPointCloud(*cloud,*gimbal_cloud,avg.matrix().cast<float>()); s.latest_gimbal_cloud=gimbal_cloud; s.latest_stamp=stamp;
  }
  void addExtrinsicLocked(int id,const Iso &ext,double fit) { auto &q=lidar_[id].extrinsics; q.push_back({ext,fit}); while(q.size()>static_cast<std::size_t>(window_size_))q.pop_front(); }
  static Iso averageExtrinsic(const std::deque<std::pair<Iso,double>> &q) {
    if (q.empty()) return Iso::Identity(); Eigen::Vector3d t=Eigen::Vector3d::Zero(); Eigen::Quaterniond sum(0,0,0,0),ref(q.front().first.rotation());
    for (const auto &m:q) { t+=m.first.translation(); Eigen::Quaterniond x(m.first.rotation()); if(x.dot(ref)<0)x.coeffs()*=-1; sum.coeffs()+=x.coeffs(); } t/=static_cast<double>(q.size()); sum.normalize(); Iso out=Iso::Identity(); out.linear()=sum.toRotationMatrix(); out.translation()=t; return out;
  }
  void publishCloud(const Cloud::Ptr &cloud,const rclcpp::Time &stamp,const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub) const { if(!cloud||cloud->empty())return; sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(*cloud,msg); msg.header.stamp=stamp; msg.header.frame_id="gimbal"; pub->publish(msg); }
  void reportAndPublish() {
    std::lock_guard<std::mutex> lock(mutex_); if(!initialized_)return; const Iso e1=averageExtrinsic(lidar_[0].extrinsics),e2=averageExtrinsic(lidar_[1].extrinsics),rel=e1.inverse()*e2; writeYaml(e1,e2,rel);
    publishCloud(lidar_[0].latest_gimbal_cloud,lidar_[0].latest_stamp,pub_cloud_[0]); publishCloud(lidar_[1].latest_gimbal_cloud,lidar_[1].latest_stamp,pub_cloud_[1]); if(!lidar_[0].latest_gimbal_cloud->empty()&&!lidar_[1].latest_gimbal_cloud->empty()){auto fused=std::make_shared<Cloud>();*fused=*lidar_[0].latest_gimbal_cloud;*fused+=*lidar_[1].latest_gimbal_cloud;publishCloud(fused,get_clock()->now(),pub_fused_);}
    auto map=std::make_shared<Cloud>(); *map=*lidar_[0].map; Cloud m2; pcl::transformPointCloud(*lidar_[1].map,m2,lidar_[1].initial_world.matrix().cast<float>()); *map+=m2; publishCloud(map,get_clock()->now(),pub_map_);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"samples=%zu/%zu T_lidar1_lidar2=[%.3f %.3f %.3f] fitness=see %s",lidar_[0].extrinsics.size(),lidar_[1].extrinsics.size(),rel.translation().x(),rel.translation().y(),rel.translation().z(),output_file_.c_str());
  }
  static void writeMatrix(std::ofstream &o,const char *name,const Iso &t) { o<<name<<":\n"; for(int r=0;r<4;++r){o<<"  [";for(int c=0;c<4;++c)o<<t.matrix()(r,c)<<(c==3?"]\n":", ");} }
  void writeYaml(const Iso &e1,const Iso &e2,const Iso &rel) const { std::ofstream o(output_file_); if(!o)return; o<<std::setprecision(12)<<"# gimbal frame = frame at startup; common yaw motion is removed\n"; writeMatrix(o,"T_gimbal_lidar1",e1); writeMatrix(o,"T_gimbal_lidar2",e2); writeMatrix(o,"T_lidar1_lidar2",rel); o<<"# p_lidar1 = T_lidar1_lidar2 * p_lidar2\n"; }

  std::string cloud_topic_[2],imu_topic_[2],output_file_; double startup_seconds_,sync_tolerance_,voxel_leaf_,max_correspondence_,max_fitness_; int min_points_,window_size_,max_map_points_; bool calibration_started_=false,initialized_=false; rclcpp::Time start_stamp_{0,0,RCL_ROS_TIME}; LidarState lidar_[2]; mutable std::mutex mutex_; rclcpp::Subscription<Msg>::SharedPtr cloud_sub_[2]; rclcpp::Subscription<Imu>::SharedPtr imu_sub_[2]; rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_[2],pub_fused_,pub_map_; rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RotateCali>());rclcpp::shutdown();return 0;}
