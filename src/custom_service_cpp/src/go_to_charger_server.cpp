#include "rclcpp/rclcpp.hpp"
#include "custom_service_interfaces/srv/go_to_charger.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class ChargerServer : public rclcpp::Node
{
public:
  ChargerServer() : Node("charger_service_server")
  {
    service_ = this->create_service<custom_service_interfaces::srv::GoToCharger>(
      "go_to_charger",
      std::bind(&ChargerServer::handle, this,
      std::placeholders::_1,
      std::placeholders::_2));
  }

private:
  void handle(
    const std::shared_ptr<custom_service_interfaces::srv::GoToCharger::Request> request,
    std::shared_ptr<custom_service_interfaces::srv::GoToCharger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service started...");

    std::this_thread::sleep_for(30s);  // simulate travel

    response->success = true;
    response->total_time = 30.0;

    RCLCPP_INFO(this->get_logger(), "Service completed");
  }

  rclcpp::Service<custom_service_interfaces::srv::GoToCharger>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerServer>());
  rclcpp::shutdown();
  return 0;
}
