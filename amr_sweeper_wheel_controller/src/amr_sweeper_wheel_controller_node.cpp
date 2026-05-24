#include "amr_sweeper_wheel_controller_node.hpp"

#include <functional>
#include <memory>

namespace amr_sweeper_wheel_controller
{

WheelControllerNode::WheelControllerNode(const rclcpp::NodeOptions & options)
: Node("wheel_command_stamper", options)
{
  loadParameters();

  publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_topic_, rclcpp::SystemDefaultsQoS());
  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WheelControllerNode::onCommand, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Stamping Twist commands from '%s' to '%s'",
    input_topic_.c_str(),
    output_topic_.c_str());
}

void WheelControllerNode::loadParameters()
{
  input_topic_ = declare_parameter("input_topic", std::string{"cmd_vel_sweep_wheels"});
  output_topic_ = declare_parameter("output_topic", std::string{"diff_cont/cmd_vel"});
  frame_id_ = declare_parameter("frame_id", std::string{"base_footprint"});
}

void WheelControllerNode::onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped stamped_msg;
  const auto stamp = now().nanoseconds();
  stamped_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
  stamped_msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
  stamped_msg.header.frame_id = frame_id_;
  stamped_msg.twist = *msg;
  publisher_->publish(stamped_msg);
}

}  // namespace amr_sweeper_wheel_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_wheel_controller::WheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}
