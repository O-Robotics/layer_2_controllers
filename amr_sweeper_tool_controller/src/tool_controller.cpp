#include "amr_sweeper_tool_controller/tool_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_sweeper_tool_controller
{

namespace
{

double clampUnit(double value)
{
  return std::clamp(value, -1.0, 1.0);
}

}  // namespace

controller_interface::CallbackReturn ToolController::on_init()
{
  auto node = get_node();

  node->declare_parameter("left_joint", left_joint_name_);
  node->declare_parameter("right_joint", right_joint_name_);
  node->declare_parameter("input_topic", input_topic_);
  node->declare_parameter("direct_command_topic", direct_command_topic_);
  node->declare_parameter("max_tool_speed_rad_s", max_tool_speed_rad_s_);
  node->declare_parameter("command_timeout_sec", command_timeout_sec_);
  node->declare_parameter("direct_command_timeout_sec", direct_command_timeout_sec_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ToolController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    left_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
    right_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY,
  };
  return config;
}

controller_interface::InterfaceConfiguration ToolController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn ToolController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  left_joint_name_ = node->get_parameter("left_joint").as_string();
  right_joint_name_ = node->get_parameter("right_joint").as_string();
  input_topic_ = node->get_parameter("input_topic").as_string();
  direct_command_topic_ = node->get_parameter("direct_command_topic").as_string();
  max_tool_speed_rad_s_ = node->get_parameter("max_tool_speed_rad_s").as_double();
  command_timeout_sec_ = node->get_parameter("command_timeout_sec").as_double();
  direct_command_timeout_sec_ = node->get_parameter("direct_command_timeout_sec").as_double();

  if (left_joint_name_.empty() || right_joint_name_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Both left_joint and right_joint must be configured.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (max_tool_speed_rad_s_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "max_tool_speed_rad_s must be non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_timeout_sec_ < 0.0 || direct_command_timeout_sec_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Command timeouts must be non-negative.");
    return controller_interface::CallbackReturn::ERROR;
  }

  resetCommandState();

  twist_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ToolController::onToolCommand, this, std::placeholders::_1));
  direct_command_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    direct_command_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ToolController::onDirectCommand, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "Tool controller configured: twist='%s', direct='%s', joints=['%s','%s']",
    input_topic_.c_str(),
    direct_command_topic_.c_str(),
    left_joint_name_.c_str(),
    right_joint_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolController::on_activate(
  const rclcpp_lifecycle::State &)
{
  resetCommandState();
  setHardwareCommand(0.0, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  setHardwareCommand(0.0, 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ToolController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  double left_velocity = 0.0;
  double right_velocity = 0.0;

  {
    std::scoped_lock lock(command_mutex_);

    if (
      latest_direct_command_.valid &&
      isFresh(time, latest_direct_command_.received_at, direct_command_timeout_sec_))
    {
      left_velocity = latest_direct_command_.left_velocity;
      right_velocity = latest_direct_command_.right_velocity;
    } else if (
      latest_twist_command_.valid &&
      isFresh(time, latest_twist_command_.received_at, command_timeout_sec_))
    {
      const double linear = clampUnit(latest_twist_command_.message.linear.x);
      const double angular = clampUnit(latest_twist_command_.message.angular.z);
      left_velocity = (linear - angular) * max_tool_speed_rad_s_;
      right_velocity = (linear + angular) * max_tool_speed_rad_s_;
    }
  }

  setHardwareCommand(left_velocity, right_velocity);
  return controller_interface::return_type::OK;
}

void ToolController::resetCommandState()
{
  std::scoped_lock lock(command_mutex_);
  latest_twist_command_.message = geometry_msgs::msg::Twist{};
  latest_twist_command_.received_at = rclcpp::Time{0, 0, RCL_ROS_TIME};
  latest_twist_command_.valid = false;
  latest_direct_command_.left_velocity = 0.0;
  latest_direct_command_.right_velocity = 0.0;
  latest_direct_command_.received_at = rclcpp::Time{0, 0, RCL_ROS_TIME};
  latest_direct_command_.valid = false;
}

void ToolController::setHardwareCommand(double left_velocity, double right_velocity)
{
  if (command_interfaces_.size() != 2) {
    return;
  }

  const bool left_ok = command_interfaces_[0].set_value(left_velocity);
  const bool right_ok = command_interfaces_[1].set_value(right_velocity);
  if (!left_ok || !right_ok) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to write one or more tool velocity commands to hardware interfaces.");
  }
}

void ToolController::onToolCommand(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::scoped_lock lock(command_mutex_);
  latest_twist_command_.message = *message;
  latest_twist_command_.received_at = get_node()->now();
  latest_twist_command_.valid = true;
}

void ToolController::onDirectCommand(const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  if (!message || message->data.size() != 2) {
    return;
  }

  std::scoped_lock lock(command_mutex_);
  latest_direct_command_.left_velocity = message->data[0];
  latest_direct_command_.right_velocity = message->data[1];
  latest_direct_command_.received_at = get_node()->now();
  latest_direct_command_.valid = true;
}

bool ToolController::isFresh(
  const rclcpp::Time & now,
  const rclcpp::Time & received_at,
  double timeout_sec) const
{
  if (timeout_sec <= 0.0) {
    return true;
  }
  return (now - received_at).seconds() <= timeout_sec;
}

}  // namespace amr_sweeper_tool_controller

PLUGINLIB_EXPORT_CLASS(
  amr_sweeper_tool_controller::ToolController,
  controller_interface::ControllerInterface)
