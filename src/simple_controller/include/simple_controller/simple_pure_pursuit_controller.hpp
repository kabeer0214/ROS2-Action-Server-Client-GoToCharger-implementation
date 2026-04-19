#ifndef SIMPLE_PURE_PURSUIT_CONTROLLER_HPP_
#define SIMPLE_PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace simple_controller
{

class SimplePurePursuitController : public nav2_core::Controller
{
public:
  SimplePurePursuitController() = default;
  ~SimplePurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleController")};

  nav_msgs::msg::Path global_plan_;

  double k_angular_{1.5};
  double linear_vel_{0.2};
};

}  // namespace simple_controller

#endif
