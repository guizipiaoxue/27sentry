#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "loop_closure/msg/loop_constraint.hpp"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

class LoopDetector final : public rclcpp::Node {
 public:
  LoopDetector() : Node("loop_detector") {
    declare_parameter<int>("rings", 20); declare_parameter<int>("sectors", 60);
    declare_parameter<double>("max_radius", 80.0); declare_parameter<double>("voxel_size", 0.25);
    declare_parameter<double>("candidate_distance", 0.20); declare_parameter<int>("num_candidates", 5);
    declare_parameter<int>("exclusion_recent", 30); declare_parameter<double>("min_travel_distance", 10.0);
    declare_parameter<double>("icp_max_correspondence", 2.0); declare_parameter<int>("icp_iterations", 40);
    declare_parameter<double>("max_fitness", 0.35);
    rings_ = get_parameter("rings").as_int(); sectors_ = get_parameter("sectors").as_int();
    max_radius_ = get_parameter("max_radius").as_double(); voxel_ = get_parameter("voxel_size").as_double();
    candidate_distance_ = get_parameter("candidate_distance").as_double(); candidates_ = get_parameter("num_candidates").as_int();
    exclusion_ = get_parameter("exclusion_recent").as_int(); min_travel_ = get_parameter("min_travel_distance").as_double();
    max_corr_ = get_parameter("icp_max_correspondence").as_double(); icp_iter_ = get_parameter("icp_iterations").as_int(); max_fitness_ = get_parameter("max_fitness").as_double();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("keyframes", 10, std::bind(&LoopDetector::poseCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("keyframe_cloud", 10, std::bind(&LoopDetector::cloudCallback, this, std::placeholders::_1));
    loop_pub_ = create_publisher<loop_closure::msg::LoopConstraint>("loop_constraint", 10);
  }

 private:
  struct Frame { geometry_msgs::msg::Pose pose; Cloud::Ptr cloud; Eigen::MatrixXf desc; Eigen::VectorXf ring; };
  int rings_, sectors_, candidates_, exclusion_; double max_radius_, voxel_, candidate_distance_, min_travel_, max_corr_, max_fitness_; int icp_iter_;
  geometry_msgs::msg::PoseArray::SharedPtr poses_; std::vector<Frame> frames_; std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_; rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_; rclcpp::Publisher<loop_closure::msg::LoopConstraint>::SharedPtr loop_pub_;

  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) { std::lock_guard<std::mutex> lock(mutex_); poses_ = msg; }
  Eigen::MatrixXf descriptor(const Cloud& cloud) const {
    Eigen::MatrixXf d = Eigen::MatrixXf::Constant(rings_, sectors_, -std::numeric_limits<float>::infinity());
    for (const auto& p : cloud.points) {
      const float r = std::hypot(p.x, p.y); if (r >= max_radius_ || r < 0.1f) continue;
      int ri = std::min(rings_ - 1, static_cast<int>(r / max_radius_ * rings_));
      float a = std::atan2(p.y, p.x); if (a < 0) a += 2.0f * static_cast<float>(M_PI);
      int si = std::min(sectors_ - 1, static_cast<int>(a / (2.0f * M_PI) * sectors_)); d(ri, si) = std::max(d(ri, si), p.z);
    }
    for (int r = 0; r < rings_; ++r) for (int s = 0; s < sectors_; ++s) if (!std::isfinite(d(r,s))) d(r,s) = 0.0f;
    return d;
  }
  Eigen::VectorXf ringKey(const Eigen::MatrixXf& d) const { return d.rowwise().mean(); }
  double distance(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b, int* best_shift) const {
    double best = std::numeric_limits<double>::max(); int shift_best = 0;
    for (int sh = 0; sh < sectors_; ++sh) { double sum = 0; int n = 0; for (int r=0;r<rings_;++r) for(int s=0;s<sectors_;++s) { double x=a(r,s), y=b(r,(s+sh)%sectors_); double den=std::abs(x)+std::abs(y); if(den>1e-3){sum += std::abs(x-y)/den; ++n;} } double v=n?sum/n:1e9; if(v<best){best=v;shift_best=sh;} }
    if (best_shift) *best_shift = shift_best; return best;
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_); if (!poses_ || poses_->poses.size() <= frames_.size()) return;
    Cloud::Ptr raw(new Cloud); pcl::fromROSMsg(*msg, *raw); Cloud::Ptr filtered(new Cloud); pcl::VoxelGrid<Point> vg; vg.setLeafSize(voxel_, voxel_, voxel_); vg.setInputCloud(raw); vg.filter(*filtered); if (filtered->empty()) return;
    Frame f; f.pose = poses_->poses[frames_.size()]; f.cloud = filtered; f.desc = descriptor(*filtered); f.ring = ringKey(f.desc);
    const int idx = static_cast<int>(frames_.size()); frames_.push_back(f); if (idx <= exclusion_) return;
    double dist = 0; const auto& p = f.pose.position; for (int i=0;i<idx;i++) { double dx=p.x-frames_[i].pose.position.x,dy=p.y-frames_[i].pose.position.y; dist=std::max(dist,std::hypot(dx,dy)); } if (dist < min_travel_) return;
    std::vector<std::pair<double,int>> ranked; for(int i=0;i<idx-exclusion_;++i) ranked.emplace_back((f.ring-frames_[i].ring).norm(),i); std::sort(ranked.begin(),ranked.end());
    int tested=0; for (auto [rk, candidate] : ranked) { if (tested++ >= candidates_ || rk > candidate_distance_*10.0) break; int shift=0; double sc=distance(f.desc,frames_[candidate].desc,&shift); if(sc>candidate_distance_) continue; Eigen::Matrix4f guess=Eigen::Matrix4f::Identity(); float yaw=static_cast<float>(shift)*2.0f*static_cast<float>(M_PI)/sectors_; guess.block<3,3>(0,0)=Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ()).toRotationMatrix(); pcl::GeneralizedIterativeClosestPoint<Point,Point> gicp; gicp.setMaxCorrespondenceDistance(max_corr_); gicp.setMaximumIterations(icp_iter_); gicp.setInputSource(f.cloud); gicp.setInputTarget(frames_[candidate].cloud); Cloud aligned; gicp.align(aligned,guess); if(!gicp.hasConverged() || gicp.getFitnessScore()>max_fitness_) continue; loop_closure::msg::LoopConstraint out; out.current_index=idx; out.matched_index=candidate; Eigen::Matrix4f t=gicp.getFinalTransformation(); Eigen::Quaternionf q(t.block<3,3>(0,0)); out.relative_pose.position.x=t(0,3); out.relative_pose.position.y=t(1,3); out.relative_pose.position.z=t(2,3); out.relative_pose.orientation.w=q.w(); out.relative_pose.orientation.x=q.x(); out.relative_pose.orientation.y=q.y(); out.relative_pose.orientation.z=q.z(); out.fitness=gicp.getFitnessScore(); out.yaw_difference=yaw; loop_pub_->publish(out); RCLCPP_INFO(get_logger(),"loop closure %d <-> %d, score %.3f",idx,candidate,out.fitness); break; }
  }
};

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<LoopDetector>()); rclcpp::shutdown(); return 0; }
