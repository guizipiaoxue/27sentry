#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class KdTreeMapNode final : public rclcpp::Node
{
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  KdTreeMapNode()
  : Node("kdtree_map")
  {
    map_frame_ = declare_parameter<std::string>("map.frame", "odom");
    strict_frame_ = declare_parameter<bool>("map.strict_frame", true);
    input_voxel_size_ =
      declare_parameter<double>("map.input_voxel_size", 0.15);
    min_point_spacing_ =
      declare_parameter<double>("map.min_point_spacing", 0.10);
    publish_voxel_size_ =
      declare_parameter<double>("map.publish_voxel_size", 0.20);
    publish_every_n_keyframes_ =
      declare_parameter<int>("map.publish_every_n_keyframes", 5);
    max_points_ = declare_parameter<std::int64_t>("map.max_points", 5000000);
    min_z_ = declare_parameter<double>("map.min_z", -1000.0);
    max_z_ = declare_parameter<double>("map.max_z", 1000.0);
    save_path_ = declare_parameter<std::string>(
      "map.save_path", "maps/dlio_kdtree_map.pcd");
    save_voxel_size_ =
      declare_parameter<double>("map.save_voxel_size", 0.10);

    validateParameters();

    map_cloud_ = std::make_shared<Cloud>();
    kdtree_ = std::make_shared<pcl::KdTreeFLANN<Point>>();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(1).reliable().transient_local());
    keyframe_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "keyframe", rclcpp::QoS(20).reliable(),
      std::bind(
        &KdTreeMapNode::keyframeCallback, this,
        std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "save_map", std::bind(
        &KdTreeMapNode::saveMap, this,
        std::placeholders::_1, std::placeholders::_2));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      "clear_map", std::bind(
        &KdTreeMapNode::clearMap, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "KD-tree mapper ready: frame=%s, input voxel=%.3f m, spacing=%.3f m",
      map_frame_.c_str(), input_voxel_size_, min_point_spacing_);
  }

private:
  using GridCell = std::array<std::int64_t, 3>;

  struct GridCellHash
  {
    std::size_t operator()(const GridCell & cell) const noexcept
    {
      std::size_t seed = 0;
      for (const auto coordinate : cell) {
        const auto value = std::hash<std::int64_t>{}(coordinate);
        seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      }
      return seed;
    }
  };

  GridCell gridCell(const Point & point) const
  {
    return {static_cast<std::int64_t>(std::floor(point.x / min_point_spacing_)),
      static_cast<std::int64_t>(std::floor(point.y / min_point_spacing_)),
      static_cast<std::int64_t>(std::floor(point.z / min_point_spacing_))};
  }

  bool isCloseToAccepted(
    const Point & point, const Cloud & accepted,
    const std::unordered_map<GridCell, std::vector<std::size_t>,
    GridCellHash> & accepted_grid,
    float spacing_squared) const
  {
    const GridCell center = gridCell(point);
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          const GridCell neighbor{center[0] + dx, center[1] + dy,
            center[2] + dz};
          const auto cell = accepted_grid.find(neighbor);
          if (cell == accepted_grid.end()) {
            continue;
          }
          for (const auto index : cell->second) {
            const Point & other = accepted[index];
            const float x = point.x - other.x;
            const float y = point.y - other.y;
            const float z = point.z - other.z;
            if (x * x + y * y + z * z < spacing_squared) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  void validateParameters() const
  {
    const bool finite_sizes = std::isfinite(input_voxel_size_) &&
      std::isfinite(min_point_spacing_) &&
      std::isfinite(publish_voxel_size_) &&
      std::isfinite(save_voxel_size_);
    const bool finite_height_range = std::isfinite(min_z_) &&
      std::isfinite(max_z_);
    if (map_frame_.empty() || !finite_sizes || !finite_height_range ||
      input_voxel_size_ <= 0.0 ||
      min_point_spacing_ <= 0.0 || publish_voxel_size_ < 0.0 ||
      publish_every_n_keyframes_ <= 0 || max_points_ <= 0 ||
      max_points_ > std::numeric_limits<std::uint32_t>::max() ||
      min_z_ >= max_z_ || save_path_.empty() || save_voxel_size_ < 0.0)
    {
      throw std::invalid_argument("invalid KD-tree map parameters");
    }
  }

  Cloud::Ptr preprocess(const sensor_msgs::msg::PointCloud2 & message) const
  {
    Cloud::Ptr input(new Cloud);
    pcl::fromROSMsg(message, *input);

    Cloud::Ptr finite(new Cloud);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*input, *finite, valid_indices);

    Cloud::Ptr height_filtered(new Cloud);
    height_filtered->reserve(finite->size());
    for (const auto & point : finite->points) {
      if (std::isfinite(point.intensity) && point.z >= min_z_ &&
        point.z <= max_z_)
      {
        height_filtered->push_back(point);
      }
    }
    height_filtered->width =
      static_cast<std::uint32_t>(height_filtered->size());
    height_filtered->height = 1;
    height_filtered->is_dense = true;

    Cloud::Ptr downsampled(new Cloud);
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(height_filtered);
    voxel_filter.setLeafSize(
      input_voxel_size_, input_voxel_size_,
      input_voxel_size_);
    voxel_filter.filter(*downsampled);
    return downsampled;
  }

  void keyframeCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "keyframe frame '%s' differs from configured map frame '%s'%s",
        message->header.frame_id.c_str(), map_frame_.c_str(),
        strict_frame_ ? "; dropping keyframe" : "; accepting without TF");
      if (strict_frame_) {
        return;
      }
    }

    const Cloud::Ptr candidates = preprocess(*message);
    if (candidates->empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const bool has_index = !map_cloud_->empty();
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    const float spacing_squared = static_cast<float>(
      min_point_spacing_ * min_point_spacing_);

    Cloud accepted;
    accepted.reserve(candidates->size());
    std::unordered_map<GridCell, std::vector<std::size_t>, GridCellHash>
    accepted_grid;
    accepted_grid.reserve(candidates->size());
    std::size_t rejected_as_duplicate = 0;
    for (const auto & point : candidates->points) {
      if (map_cloud_->size() + accepted.size() >=
        static_cast<std::size_t>(max_points_))
      {
        break;
      }

      bool has_close_point = false;
      if (has_index &&
        kdtree_->nearestKSearch(
          point, 1, nearest_index,
          nearest_squared_distance) > 0)
      {
        has_close_point = nearest_squared_distance.front() < spacing_squared;
      }
      if (!has_close_point &&
        isCloseToAccepted(
          point, accepted, accepted_grid,
          spacing_squared))
      {
        has_close_point = true;
      }
      if (has_close_point) {
        ++rejected_as_duplicate;
      } else {
        accepted.push_back(point);
        accepted_grid[gridCell(point)].push_back(accepted.size() - 1U);
      }
    }

    if (!accepted.empty()) {
      *map_cloud_ += accepted;
      map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->size());
      map_cloud_->height = 1;
      map_cloud_->is_dense = true;
      kdtree_->setInputCloud(map_cloud_);
    }

    ++keyframe_count_;
    if (map_cloud_->size() >= static_cast<std::size_t>(max_points_) &&
      !map_limit_reported_)
    {
      map_limit_reported_ = true;
      RCLCPP_WARN(
        get_logger(),
        "map reached max_points=%ld; new points will be rejected",
        static_cast<long>(max_points_));
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "KD map: keyframes=%zu, points=%zu, inserted=%zu, duplicates=%zu",
      keyframe_count_, map_cloud_->size(), accepted.size(),
      rejected_as_duplicate);

    if (keyframe_count_ == 1U ||
      keyframe_count_ %
      static_cast<std::size_t>(publish_every_n_keyframes_) ==
      0U)
    {
      publishMap(message->header.stamp);
    }
  }

  Cloud::Ptr filteredMap(double leaf_size) const
  {
    if (map_cloud_->empty() || leaf_size <= 0.0) {
      return std::make_shared<Cloud>(*map_cloud_);
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(map_cloud_);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*filtered);
    return filtered;
  }

  void publishMap(const builtin_interfaces::msg::Time & stamp)
  {
    const Cloud::Ptr output = filteredMap(publish_voxel_size_);
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*output, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    map_pub_->publish(message);
  }

  void saveMap(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_cloud_->empty()) {
      response->success = false;
      response->message = "map is empty";
      return;
    }

    const std::filesystem::path output_path(save_path_);
    std::error_code error;
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path(), error);
    }
    if (error) {
      response->success = false;
      response->message = "cannot create output directory: " + error.message();
      return;
    }

    const Cloud::Ptr output = filteredMap(save_voxel_size_);
    response->success = pcl::io::savePCDFileBinary(save_path_, *output) == 0;
    response->message = response->success ?
      "saved " + std::to_string(output->size()) +
      " points to " + save_path_ :
      "failed to save " + save_path_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void clearMap(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_cloud_->clear();
    map_cloud_->width = 0;
    map_cloud_->height = 1;
    keyframe_count_ = 0;
    map_limit_reported_ = false;
    kdtree_ = std::make_shared<pcl::KdTreeFLANN<Point>>();
    publishMap(now());
    response->success = true;
    response->message = "KD-tree map cleared";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::string map_frame_;
  bool strict_frame_;
  double input_voxel_size_;
  double min_point_spacing_;
  double publish_voxel_size_;
  int publish_every_n_keyframes_;
  std::int64_t max_points_;
  double min_z_;
  double max_z_;
  std::string save_path_;
  double save_voxel_size_;

  std::mutex mutex_;
  Cloud::Ptr map_cloud_;
  pcl::KdTreeFLANN<Point>::Ptr kdtree_;
  std::size_t keyframe_count_ = 0;
  bool map_limit_reported_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KdTreeMapNode>());
  rclcpp::shutdown();
  return 0;
}
