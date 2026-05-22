#ifndef AMR_SWEEPER_WHEEL_CONTROLLER__AMR_SWEEPER_WHEEL_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_WHEEL_CONTROLLER__AMR_SWEEPER_WHEEL_CONTROLLER_NODE_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_sweeper_wheel_controller
{

class WheelControllerNode : public rclcpp::Node
{
public:
  explicit WheelControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::string input_topic_{"cmd_vel_wheels"};
  std::string output_topic_{"diff_cont/cmd_vel"};
  std::string frame_id_{"base_footprint"};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace amr_sweeper_wheel_controller

#endif  // AMR_SWEEPER_WHEEL_CONTROLLER__AMR_SWEEPER_WHEEL_CONTROLLER_NODE_HPP_
