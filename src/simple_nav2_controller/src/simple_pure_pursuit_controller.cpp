#include "simple_nav2_controller/simple_pure_pursuit_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

#include <cmath>
#include <limits>

namespace simple_nav2_controller
{

void SimplePurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  auto node = parent.lock();

  node->declare_parameter(name + ".k_angular", 1.5);
  node->declare_parameter(name + ".linear_vel", 0.2);

  node->get_parameter(name + ".k_angular", k_angular_);
  node->get_parameter(name + ".linear_vel", linear_vel_);

  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Simple Controller configured");
}

void SimplePurePursuitController::activate()
{
  RCLCPP_INFO(logger_, "Activated");
}

void SimplePurePursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated");
}

void SimplePurePursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaned up");
}

void SimplePurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped
SimplePurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;

  cmd.header = pose.header;

  if (global_plan_.poses.empty()) {
    return cmd;
  }

  // Step 1: Find nearest waypoint
  double min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::PoseStamped target;

  for (const auto & p : global_plan_.poses) {
    double dx = p.pose.position.x - pose.pose.position.x;
    double dy = p.pose.position.y - pose.pose.position.y;
    double d = std::hypot(dx, dy);

    if (d < min_dist) {
      min_dist = d;
      target = p;
    }
  }

  // Step 2: Compute heading error
  double dx = target.pose.position.x - pose.pose.position.x;
  double dy = target.pose.position.y - pose.pose.position.y;

  double target_yaw = std::atan2(dy, dx);
  double current_yaw = tf2::getYaw(pose.pose.orientation);

  double error = target_yaw - current_yaw;

  // Normalize
  while (error > M_PI) error -= 2 * M_PI;
  while (error < -M_PI) error += 2 * M_PI;

  // Step 3: Control law
  cmd.twist.linear.x = linear_vel_;
  cmd.twist.angular.z = k_angular_ * error;

  return cmd;
}

void SimplePurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (percentage) {
    linear_vel_ *= speed_limit / 100.0;
  } else {
    linear_vel_ = speed_limit;
  }
}

}  // namespace simple_controller

PLUGINLIB_EXPORT_CLASS(
  simple_nav2_controller::SimplePurePursuitController,
  nav2_core::Controller)
