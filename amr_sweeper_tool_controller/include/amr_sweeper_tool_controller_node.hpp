#ifndef AMR_SWEEPER_TOOL_CONTROLLER__AMR_SWEEPER_TOOL_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_TOOL_CONTROLLER__AMR_SWEEPER_TOOL_CONTROLLER_NODE_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amr_sweeper_tool_controller
{

class ToolControllerNode : public rclcpp::Node
{
public:
  explicit ToolControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg);

  double max_tool_speed_rad_s_{35.0};
  std::string input_topic_{"cmd_vel_sweep_tools"};
  std::string output_topic_{"controller_steadydrive/commands"};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

}  // namespace amr_sweeper_tool_controller

#endif  // AMR_SWEEPER_TOOL_CONTROLLER__AMR_SWEEPER_TOOL_CONTROLLER_NODE_HPP_
