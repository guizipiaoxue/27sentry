#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <receiver/msg/chassis_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

using SteadyClock = std::chrono::steady_clock;
using Vec2 = Eigen::Vector2d;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Vec2 rotate(const Vec2& value, double yaw) {
  return {std::cos(yaw) * value.x() - std::sin(yaw) * value.y(),
          std::sin(yaw) * value.x() + std::cos(yaw) * value.y()};
}

struct Reference {
  Vec2 position;
  Vec2 velocity;
};

class MpcControl : public rclcpp::Node {
 public:
  MpcControl() : Node("mpc_control") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    command_frame_ = declare_parameter<std::string>("command_frame", "base_link");
    period_ms_ = declare_parameter<int>("control_period_ms", 50);
    horizon_ = declare_parameter<int>("horizon_steps", 20);
    target_speed_ = declare_parameter<double>("target_speed_mps", 1.0);
    max_speed_ = declare_parameter<double>("max_speed_mps", 1.0);
    max_accel_ = declare_parameter<double>("max_accel_mps2", 1.5);
    response_tau_ = declare_parameter<double>("velocity_time_constant_sec", 0.15);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance_m", 0.15);
    path_timeout_ = declare_parameter<double>("path_timeout_sec", 15.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout_sec", 0.3);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout_sec", 0.3);
    filter_gain_ = declare_parameter<double>("velocity_filter_gain", 0.35);
    feedback_y_sign_ = declare_parameter<double>("chassis_feedback_y_sign", -1.0);
    w_position_ = declare_parameter<double>("position_weight", 12.0);
    w_velocity_ = declare_parameter<double>("velocity_weight", 2.0);
    w_control_ = declare_parameter<double>("control_weight", 0.05);
    w_smooth_ = declare_parameter<double>("smoothness_weight", 1.5);
    w_terminal_ = declare_parameter<double>("terminal_weight", 20.0);
    solver_iterations_ = declare_parameter<int>("solver_max_iterations", 150);
    solver_tolerance_ = declare_parameter<double>("solver_tolerance", 0.0001);
    if (period_ms_ < 1 || horizon_ < 2 || horizon_ > 100 || target_speed_ <= 0 ||
        max_speed_ <= 0 || max_speed_ > 2.4 || max_accel_ <= 0 || response_tau_ <= 0 ||
        goal_tolerance_ <= 0 || path_timeout_ <= 0 || tf_timeout_ <= 0 ||
        feedback_timeout_ <= 0 || filter_gain_ <= 0 || filter_gain_ > 1 ||
        std::abs(feedback_y_sign_) != 1.0 || w_position_ < 0 || w_velocity_ < 0 ||
        w_control_ <= 0 || w_smooth_ < 0 || w_terminal_ < 0 ||
        solver_iterations_ < 1 || solver_tolerance_ <= 0) {
      throw std::invalid_argument("invalid MPC parameter in config/mpc.yaml");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/sPath", 5, [this](nav_msgs::msg::Path::SharedPtr msg) { update_path(*msg); });
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/setFollowSpeed", 5, [this](std_msgs::msg::Float64::SharedPtr msg) {
          if (std::isfinite(msg->data)) speed_limit_ = std::max(0.0, msg->data);
        });
    chassis_sub_ = create_subscription<receiver::msg::ChassisState>(
        "/base/chassis/state", 10, [this](receiver::msg::ChassisState::SharedPtr msg) {
          if (std::isfinite(msg->velocity_x_mps) && std::isfinite(msg->velocity_y_mps)) {
            feedback_velocity_ = {msg->velocity_x_mps,
                                  feedback_y_sign_ * msg->velocity_y_mps};
            feedback_frame_ = msg->header.frame_id;
            feedback_received_ = SteadyClock::now();
          }
        });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    predicted_pub_ = create_publisher<nav_msgs::msg::Path>("/mpc/predicted_path", 5);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("/ly/navi/reached", 5);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), [this] { step(); });
  }

 private:
  // Keep a distance parameterization so horizon points can be sampled by speed.
  void update_path(const nav_msgs::msg::Path& msg) {
    if (msg.header.frame_id != map_frame_) {
      RCLCPP_WARN(get_logger(), "Path frame must be %s", map_frame_.c_str());
      return;
    }
    path_.clear();
    arc_.clear();
    for (const auto& pose : msg.poses) {
      const Vec2 point(pose.pose.position.x, pose.pose.position.y);
      if (!point.allFinite() || (!path_.empty() &&
          (point - path_.back()).norm() < 1e-5)) continue;
      arc_.push_back(path_.empty() ? 0.0 : arc_.back() + (point - path_.back()).norm());
      path_.push_back(point);
    }
    if (path_.size() < 2) {
      path_.clear();
      arc_.clear();
      RCLCPP_WARN(get_logger(), "Path has fewer than two distinct points");
      return;
    }
    path_received_ = SteadyClock::now();
    warm_x_.resize(0);
    warm_y_.resize(0);
  }

  // Project the current pose onto the polyline before sampling the horizon.
  double nearest_arc(const Vec2& position) const {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {
      const Vec2 segment = path_[i] - path_[i - 1];
      const double fraction = std::clamp(
          (position - path_[i - 1]).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
      const Vec2 projection = path_[i - 1] + fraction * segment;
      const double distance = (position - projection).squaredNorm();
      if (distance < best_distance) {
        best_distance = distance;
        best_arc = arc_[i - 1] + fraction * (arc_[i] - arc_[i - 1]);
      }
    }
    return best_arc;
  }

  Reference sample_reference(double arc_position, double speed) const {
    if (arc_position >= arc_.back()) return {path_.back(), Vec2::Zero()};
    const auto upper = std::upper_bound(arc_.begin(), arc_.end(), arc_position);
    const size_t i = std::max<size_t>(1, upper - arc_.begin());
    const Vec2 delta = path_[i] - path_[i - 1];
    const double fraction = (arc_position - arc_[i - 1]) / (arc_[i] - arc_[i - 1]);
    const double remaining = arc_.back() - arc_position;
    const double braking_speed = std::min(speed, std::sqrt(2.0 * max_accel_ * remaining));
    return {path_[i - 1] + fraction * delta, braking_speed * delta.normalized()};
  }

  // Stale TF is unsafe for feedback control, even when lookup itself succeeds.
  bool lookup_pose(const std::string& frame, Vec2& position, double& yaw,
                   rclcpp::Time& stamp) {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, frame, tf2::TimePointZero);
      stamp = rclcpp::Time(tf.header.stamp);
      if ((now() - stamp).seconds() > tf_timeout_ || (now() - stamp).seconds() < -0.1) {
        return false;
      }
      position = {tf.transform.translation.x, tf.transform.translation.y};
      yaw = yaw_from_quaternion(tf.transform.rotation);
      return position.allFinite() && std::isfinite(yaw);
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  bool estimate_velocity(const Vec2& position, const rclcpp::Time& stamp,
                         double command_yaw, Vec2& velocity) {
    if (!last_pose_stamp_.has_value() || stamp > *last_pose_stamp_) {
      if (last_pose_stamp_.has_value()) {
        const double dt = (stamp - *last_pose_stamp_).seconds();
        if (dt > 1e-3 && dt < 1.0) {
          const Vec2 measured = (position - last_pose_) / dt;
          pose_velocity_ = has_pose_velocity_
                               ? (1.0 - filter_gain_) * pose_velocity_ + filter_gain_ * measured
                               : measured;
          has_pose_velocity_ = true;
        }
      }
      last_pose_ = position;
      last_pose_stamp_ = stamp;
    }

    // Chassis telemetry uses the gimbal axes; use it only when its TF frame matches.
    if (SteadyClock::now() - feedback_received_ <
            std::chrono::duration<double>(feedback_timeout_) &&
        feedback_frame_ == command_frame_) {
      velocity = rotate(feedback_velocity_, command_yaw);
      return true;
    }
    if (has_pose_velocity_) {
      velocity = pose_velocity_;
      return true;
    }
    return false;
  }

  // Solve the condensed finite-horizon QP with projected accelerated gradient.
  bool solve(const Vec2& position, const Vec2& velocity, const Vec2& previous,
             double arc_position, double speed, Vec2& command,
             nav_msgs::msg::Path& prediction) {
    const int n = horizon_;
    const double dt = period_ms_ / 1000.0;
    const double alpha = std::min(1.0, dt / response_tau_);
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd gx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd gy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd cp = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd cv = Eigen::VectorXd::Zero(n);
    Vec2 base_p = position;
    Vec2 base_v = velocity;

    for (int k = 0; k < n; ++k) {
      cv *= 1.0 - alpha;
      cv[k] += alpha;
      base_v *= 1.0 - alpha;
      cp += dt * cv;
      base_p += dt * base_v;
      const auto ref = sample_reference(arc_position + (k + 1) * dt * speed, speed);
      h.noalias() += w_position_ * cp * cp.transpose() + w_velocity_ * cv * cv.transpose();
      gx.noalias() += w_position_ * cp * (base_p.x() - ref.position.x()) +
                      w_velocity_ * cv * (base_v.x() - ref.velocity.x());
      gy.noalias() += w_position_ * cp * (base_p.y() - ref.position.y()) +
                      w_velocity_ * cv * (base_v.y() - ref.velocity.y());
    }
    const auto terminal = sample_reference(arc_position + n * dt * speed, speed);
    h.noalias() += w_terminal_ * cp * cp.transpose();
    gx.noalias() += w_terminal_ * cp * (base_p.x() - terminal.position.x());
    gy.noalias() += w_terminal_ * cp * (base_p.y() - terminal.position.y());
    h.diagonal().array() += w_control_;
    h(0, 0) += w_smooth_;
    gx[0] -= w_smooth_ * previous.x();
    gy[0] -= w_smooth_ * previous.y();
    for (int k = 1; k < n; ++k) {
      h(k, k) += w_smooth_;
      h(k - 1, k - 1) += w_smooth_;
      h(k, k - 1) -= w_smooth_;
      h(k - 1, k) -= w_smooth_;
    }

    const double lipschitz = h.cwiseAbs().rowwise().sum().maxCoeff();
    if (!std::isfinite(lipschitz) || lipschitz <= 0) return false;
    Eigen::VectorXd ux = warm_x_.size() == n ? warm_x_ : Eigen::VectorXd::Zero(n);
    Eigen::VectorXd uy = warm_y_.size() == n ? warm_y_ : Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yx = ux;
    Eigen::VectorXd yy = uy;
    double momentum = 1.0;
    for (int iteration = 0; iteration < solver_iterations_; ++iteration) {
      Eigen::VectorXd next_x = yx - (h * yx + gx) / lipschitz;
      Eigen::VectorXd next_y = yy - (h * yy + gy) / lipschitz;
      for (int k = 0; k < n; ++k) {
        const double norm = std::hypot(next_x[k], next_y[k]);
        if (norm > speed) {
          next_x[k] *= speed / norm;
          next_y[k] *= speed / norm;
        }
      }
      const double delta = std::hypot((next_x - ux).norm(), (next_y - uy).norm());
      const double next_momentum = (1.0 + std::sqrt(1.0 + 4.0 * momentum * momentum)) / 2.0;
      yx = next_x + ((momentum - 1.0) / next_momentum) * (next_x - ux);
      yy = next_y + ((momentum - 1.0) / next_momentum) * (next_y - uy);
      ux.swap(next_x);
      uy.swap(next_y);
      momentum = next_momentum;
      if (delta < solver_tolerance_) break;
    }
    if (!ux.allFinite() || !uy.allFinite()) return false;

    // The first command is rate limited before both publishing and prediction.
    command = {ux[0], uy[0]};
    Vec2 change = command - previous;
    const double max_change = max_accel_ * dt;
    if (change.norm() > max_change) command = previous + max_change * change.normalized();
    warm_x_ = ux;
    warm_y_ = uy;
    for (int k = 0; k < n - 1; ++k) {
      warm_x_[k] = ux[k + 1];
      warm_y_[k] = uy[k + 1];
    }

    // Publish the actual model rollout for tuning and RViz inspection.
    Vec2 predicted_p = position;
    Vec2 predicted_v = velocity;
    prediction.header.stamp = now();
    prediction.header.frame_id = map_frame_;
    for (int k = 0; k < n; ++k) {
      const Vec2 input = k == 0 ? command : Vec2(ux[k], uy[k]);
      predicted_v = (1.0 - alpha) * predicted_v + alpha * input;
      predicted_p += dt * predicted_v;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = prediction.header;
      pose.pose.position.x = predicted_p.x();
      pose.pose.position.y = predicted_p.y();
      pose.pose.orientation.w = 1.0;
      prediction.poses.push_back(pose);
    }
    return true;
  }

  void publish_command(const Vec2& command, bool reached) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.x();
    twist.linear.y = command.y();
    cmd_pub_->publish(twist);
    std_msgs::msg::Bool flag;
    flag.data = reached;
    reached_pub_->publish(flag);
  }

  // Invalid inputs always produce an immediate stop command.
  void step() {
    const auto current = SteadyClock::now();
    if (path_.empty() || current - path_received_ >
            std::chrono::duration<double>(path_timeout_) || speed_limit_ == 0.0) {
      last_command_map_.setZero();
      publish_command(Vec2::Zero(), false);
      return;
    }
    Vec2 position;
    double robot_yaw = 0.0;
    rclcpp::Time robot_stamp(0, 0, get_clock()->get_clock_type());
    if (!lookup_pose(robot_frame_, position, robot_yaw, robot_stamp)) {
      last_command_map_.setZero();
      publish_command(Vec2::Zero(), false);
      return;
    }
    Vec2 command_frame_position = position;
    double command_yaw = robot_yaw;
    rclcpp::Time command_stamp(0, 0, get_clock()->get_clock_type());
    if (command_frame_ != robot_frame_ &&
        !lookup_pose(command_frame_, command_frame_position, command_yaw, command_stamp)) {
      last_command_map_.setZero();
      publish_command(Vec2::Zero(), false);
      return;
    }

    Vec2 velocity;
    if (!estimate_velocity(position, robot_stamp, command_yaw, velocity)) {
      publish_command(Vec2::Zero(), false);
      return;
    }
    const double distance_to_goal = (position - path_.back()).norm();
    if (distance_to_goal <= goal_tolerance_) {
      last_command_map_.setZero();
      publish_command(Vec2::Zero(), true);
      return;
    }

    const double speed = std::min({target_speed_, max_speed_, speed_limit_});
    if (speed <= 0.0) {
      publish_command(Vec2::Zero(), false);
      return;
    }
    Vec2 command_map;
    nav_msgs::msg::Path prediction;
    if (!solve(position, velocity, last_command_map_, nearest_arc(position),
               speed, command_map, prediction)) {
      last_command_map_.setZero();
      publish_command(Vec2::Zero(), false);
      return;
    }

    // Express the rate-limited command in the frame expected by the serial protocol.
    last_command_map_ = command_map;
    publish_command(rotate(command_map, -command_yaw), false);
    predicted_pub_->publish(prediction);
  }

  std::string map_frame_;
  std::string robot_frame_;
  std::string command_frame_;
  std::string feedback_frame_;
  int period_ms_{};
  int horizon_{};
  int solver_iterations_{};
  double target_speed_{};
  double max_speed_{};
  double max_accel_{};
  double response_tau_{};
  double goal_tolerance_{};
  double path_timeout_{};
  double tf_timeout_{};
  double feedback_timeout_{};
  double filter_gain_{};
  double feedback_y_sign_{};
  double w_position_{};
  double w_velocity_{};
  double w_control_{};
  double w_smooth_{};
  double w_terminal_{};
  double solver_tolerance_{};
  double speed_limit_ = std::numeric_limits<double>::infinity();
  bool has_pose_velocity_ = false;
  std::vector<Vec2> path_;
  std::vector<double> arc_;
  Vec2 last_pose_ = Vec2::Zero();
  Vec2 pose_velocity_ = Vec2::Zero();
  Vec2 feedback_velocity_ = Vec2::Zero();
  Vec2 last_command_map_ = Vec2::Zero();
  Eigen::VectorXd warm_x_;
  Eigen::VectorXd warm_y_;
  SteadyClock::time_point path_received_{};
  SteadyClock::time_point feedback_received_{};
  std::optional<rclcpp::Time> last_pose_stamp_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<receiver::msg::ChassisState>::SharedPtr chassis_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MpcControl>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("mpc_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
