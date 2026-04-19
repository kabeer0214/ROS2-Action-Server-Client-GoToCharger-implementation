#include "rclcpp/rclcpp.hpp"
#include "custom_service_interfaces/srv/go_to_charger.hpp"

using namespace std::chrono_literals;

class ChargerClient : public rclcpp::Node
{
public:
  ChargerClient() : Node("charger_service_client")
  {
    client_ = this->create_client<custom_service_interfaces::srv::GoToCharger>("go_to_charger");

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&ChargerClient::send_request, this));
  }

private:
  void send_request()
  {
    timer_->cancel();

    auto request = std::make_shared<custom_service_interfaces::srv::GoToCharger::Request>();
    request->distance = 30.0;

    RCLCPP_INFO(this->get_logger(), "[client] Requesting SERVICE...");

    if (!client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }

    auto future = client_->async_send_request(request);

    auto status = future.wait_for(5s);  // 🔥 TIMEOUT HERE

    if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(),
        "[client] ✘ TIMED OUT — service did not respond within 5 seconds!");
    } else {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(),
        "Success: %d, Time: %.2f",
        response->success,
        response->total_time);
    }
  }

  rclcpp::Client<custom_service_interfaces::srv::GoToCharger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerClient>());
  rclcpp::shutdown();
  return 0;
}
