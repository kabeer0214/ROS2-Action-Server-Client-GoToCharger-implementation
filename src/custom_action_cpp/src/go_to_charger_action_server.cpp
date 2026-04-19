#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/go_to_charger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace custom_action_cpp
{

class GoToChargerActionServer : public rclcpp::Node
{
public:
  using GoToCharger = custom_action_interfaces::action::GoToCharger;
  using GoalHandleGoToCharger = rclcpp_action::ServerGoalHandle<GoToCharger>;

  explicit GoToChargerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("go_to_charger_action_server", options)
  {
    using namespace std::placeholders;

    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const GoToCharger::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal: distance = %.2f", goal->distance);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received cancel request");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
    {
      std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<GoToCharger>(
      this,
      "go_to_charger",   
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

private:
  rclcpp_action::Server<GoToCharger>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing charging task...");

    const auto goal = goal_handle->get_goal();
    float distance = goal->distance;

    auto feedback = std::make_shared<GoToCharger::Feedback>();
    auto result = std::make_shared<GoToCharger::Result>();

    rclcpp::Rate loop_rate(1);  // 1 Hz

    for (int i = 0; i < 30 && rclcpp::ok(); ++i)
    {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      distance -= 1.0;
      feedback->distance_remaining = distance;

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(),
        "Distance remaining: %.2f", distance);

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->success = true;
      result->total_time = 30.0;

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Reached charging station!");
    }
  }

};  

}  

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<custom_action_cpp::GoToChargerActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


