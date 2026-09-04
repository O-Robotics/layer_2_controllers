#ifndef AMR_SWEEPER_TOOL_CONTROLLER__TOOL_CONTROLLER_HPP_
#define AMR_SWEEPER_TOOL_CONTROLLER__TOOL_CONTROLLER_HPP_

#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amr_sweeper_tool_controller
{

class ToolController : public controller_interface::ControllerInterface
{
public:
  ToolController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct TwistCommand
  {
    geometry_msgs::msg::Twist message;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct DirectCommand
  {
    double left_velocity{0.0};
    double right_velocity{0.0};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void resetCommandState();
  void setHardwareCommand(double left_velocity, double right_velocity);
  void onToolCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void onDirectCommand(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  bool isFresh(
    const rclcpp::Time & now,
    const rclcpp::Time & received_at,
    double timeout_sec) const;

  std::string left_joint_name_{"LeftBrush_joint"};
  std::string right_joint_name_{"RightBrush_joint"};
  std::string input_topic_{"sweeping_controller/cmd_vel_tools"};
  std::string direct_command_topic_{"tool_controller/commands"};
  double max_tool_speed_rad_s_{35.0};
  double command_timeout_sec_{0.5};
  double direct_command_timeout_sec_{0.5};

  mutable std::mutex command_mutex_;
  TwistCommand latest_twist_command_;
  DirectCommand latest_direct_command_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr direct_command_subscription_;
};

}  // namespace amr_sweeper_tool_controller

#endif  // AMR_SWEEPER_TOOL_CONTROLLER__TOOL_CONTROLLER_HPP_
