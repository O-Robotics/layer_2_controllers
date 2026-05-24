#include "amr_sweeper_tool_controller_node.hpp"

#include <functional>
#include <memory>

namespace amr_sweeper_tool_controller
{

ToolControllerNode::ToolControllerNode(const rclcpp::NodeOptions & options)
: Node("tool_controller_node", options)
{
  loadParameters();

  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ToolControllerNode::onCommand, this, std::placeholders::_1));
  publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    output_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_logger(),
    "Tool controller listening on '%s' and publishing to '%s'",
    input_topic_.c_str(),
    output_topic_.c_str());
}

void ToolControllerNode::loadParameters()
{
  max_tool_speed_rad_s_ = declare_parameter("max_tool_speed_rad_s", 35.0);
  input_topic_ = declare_parameter("input_topic", std::string{"cmd_vel_sweep_tools"});
  output_topic_ = declare_parameter(
    "output_topic", std::string{"controller_steadydrive/commands"});
}

void ToolControllerNode::onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std_msgs::msg::Float64MultiArray motor_msg;
  motor_msg.data = {
    (msg->linear.x - msg->angular.z) * max_tool_speed_rad_s_,
    (msg->linear.x + msg->angular.z) * max_tool_speed_rad_s_,
  };
  publisher_->publish(motor_msg);
}

}  // namespace amr_sweeper_tool_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_tool_controller::ToolControllerNode>());
  rclcpp::shutdown();
  return 0;
}
