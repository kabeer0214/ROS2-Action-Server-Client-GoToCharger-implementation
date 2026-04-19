#include <functional>
#include <memory>
#include <chrono>

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.hpp"


class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("tf2_frame_publisher")
  {

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    initTransforms();

    auto handle_goal_pose = [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg)
    {

        map_to_odom_. transform.translation.x = msg->pose.position.x;
        map_to_odom_. transform.translation.y = msg->pose.position.y;
        map_to_odom_. transform.translation.z = msg->pose.position.z;
        map_to_odom_. transform.rotation = msg->pose.orientation;
        
    };
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, handle_goal_pose);
      
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FramePublisher::broadcast_tf, this));
  }

private:
  void initTransforms()
  
  {
  
  //map -> odom 
  
  map_to_odom_.header.frame_id = "map";
  map_to_odom_.child_frame_id = "odom";  
  map_to_odom_.transform.translation.x = 0.0;
  map_to_odom_.transform.translation.y = 0.0;
  map_to_odom_.transform.translation.z = 0.0;
  map_to_odom_.transform.rotation.x = 0.0;
  map_to_odom_.transform.rotation.y = 0.0;
  map_to_odom_.transform.rotation.z = 0.0;
  map_to_odom_.transform.rotation.w = 1.0;
  
  //odom -> base
  odom_to_base_.header.frame_id = "odom";
  odom_to_base_.child_frame_id = "base_link";  
  odom_to_base_.transform.translation.x = 0.0;
  odom_to_base_.transform.translation.y = 0.0;
  odom_to_base_.transform.translation.z = 0.0;
  odom_to_base_.transform.rotation.x = 0.0;
  odom_to_base_.transform.rotation.y = 0.0;
  odom_to_base_.transform.rotation.z = 0.0;
  odom_to_base_.transform.rotation.w = 1.0;
  }
  
  void broadcast_tf()
  
  {
  auto now =this->get_clock()->now();
  map_to_odom_.header.stamp = now;
  odom_to_base_.header.stamp = now;
  
  tf_broadcaster_->sendTransform(map_to_odom_);
  tf_broadcaster_->sendTransform(odom_to_base_);
    
  }
  
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  geometry_msgs::msg::TransformStamped odom_to_base_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
