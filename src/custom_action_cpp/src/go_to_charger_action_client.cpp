#include <functional>
#include <future>
#include <memory>

#include "custom_action_interfaces/action/go_to_charger.hpp"  

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace custom_action_cpp
{
class GoToChargerActionClient : public rclcpp::Node  
{
public:
  using GoToCharger = custom_action_interfaces::action::GoToCharger;
  using GoalHandleGoToCharger = rclcpp_action::ClientGoalHandle<GoToCharger>;

  explicit GoToChargerActionClient(const rclcpp::NodeOptions & options)
  : Node("go_to_charger_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<GoToCharger>(
      this,
      "go_to_charger");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { this->send_goal(); });
  }

  void send_goal()
  {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      rclcpp::shutdown();
    }

    auto goal_msg = GoToCharger::Goal();
    goal_msg.distance = 30.0;   

    RCLCPP_INFO(this->get_logger(),
      "[client] Goal accepted — receiving feedback…");

    auto options = rclcpp_action::Client<GoToCharger>::SendGoalOptions();

    options.feedback_callback =
      [this](GoalHandleGoToCharger::SharedPtr,
      const std::shared_ptr<const GoToCharger::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(),
        "[feedback] Distance remaining: %.2f m",
        feedback->distance_remaining);
    };

    options.result_callback =
      [this](const GoalHandleGoToCharger::WrappedResult & result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(),
          "[client] ✔ Result: Arrived! Travel time: %.2f s",
          result.result->total_time);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Goal failed");
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<GoToCharger>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<custom_action_cpp::GoToChargerActionClient>(
    rclcpp::NodeOptions());

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
