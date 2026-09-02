#ifndef AMR_SWEEPER_SWEEPING_CONTROLLER__AMR_SWEEPER_SWEEPING_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_SWEEPING_CONTROLLER__AMR_SWEEPER_SWEEPING_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr_sweeper_sweeping_controller
{

struct CommandSourceConfig
{
  bool enabled{true};
  std::string topic;
  double timeout_seconds{0.5};
  int priority{0};
};

struct CommandSourceState
{
  CommandSourceConfig config;
  geometry_msgs::msg::Twist latest_command;
  rclcpp::Time last_received;
  bool has_message{false};
};

class SweepingControllerNode : public rclcpp::Node
{
public:
  explicit SweepingControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void createSubscriptions();
  void publishSelectedCommands();
  void publishLatestStatus();
  void publishStatus(const std::string & wheel_source, const std::string & tool_source) const;
  void handleWheelJoystickCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void handleWheelNavigationCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void handleToolJoystickCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void handleSafetyStopCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void handleWebTeleopControlMode(const std_msgs::msg::String::SharedPtr message);
  void handleWebTeleopToolScale(const std_msgs::msg::Float32::SharedPtr message);
  void storeCommand(CommandSourceState & state, const geometry_msgs::msg::Twist & command);
  [[nodiscard]] geometry_msgs::msg::Twist buildToolNavigationCommand(
    const geometry_msgs::msg::Twist & wheel_navigation_command) const;
  [[nodiscard]] geometry_msgs::msg::Twist buildConstantToolNavigationCommand() const;
  [[nodiscard]] geometry_msgs::msg::Twist scaledTwist(
    const geometry_msgs::msg::Twist & command,
    double scale) const;
  [[nodiscard]] bool webTeleopOneStickActive(const rclcpp::Time & now) const;
  [[nodiscard]] bool isSourceActive(const CommandSourceState & state, const rclcpp::Time & now) const;
  [[nodiscard]] std::pair<std::string, geometry_msgs::msg::Twist> selectWheelCommand(
    const rclcpp::Time & now) const;
  [[nodiscard]] std::pair<std::string, geometry_msgs::msg::Twist> selectToolCommand(
    const rclcpp::Time & now) const;
  [[nodiscard]] bool hasActiveWheelSource(const rclcpp::Time & now) const;
  [[nodiscard]] bool hasActiveToolSource(const rclcpp::Time & now) const;

  double publish_rate_hz_{20.0};
  double status_publish_rate_hz_{2.0};
  std::string wheel_output_topic_{"sweeping_controller/cmd_vel_drive"};
  std::string tool_output_topic_{"sweeping_controller/cmd_vel_tools"};
  std::string status_topic_{"sweeping_controller/status"};
  bool publish_idle_commands_{false};
  std::string last_wheel_source_{"idle"};
  std::string last_tool_source_{"idle"};

  CommandSourceState wheel_safety_source_;
  CommandSourceState wheel_joystick_source_;
  CommandSourceState wheel_navigation_source_;
  CommandSourceState tool_safety_source_;
  CommandSourceState tool_joystick_source_;
  CommandSourceState tool_navigation_source_;

  bool tool_navigation_mapping_enabled_{true};
  std::string tool_navigation_mode_{"mapping"};
  double tool_navigation_linear_x_offset_{0.0};
  double tool_navigation_linear_x_from_linear_x_gain_{0.0};
  double tool_navigation_linear_x_from_angular_z_gain_{0.0};
  double tool_navigation_angular_z_offset_{0.0};
  double tool_navigation_angular_z_from_linear_x_gain_{0.0};
  double tool_navigation_angular_z_from_angular_z_gain_{0.0};
  double tool_navigation_constant_linear_x_{0.0};
  double tool_navigation_constant_angular_z_{0.0};
  std::string web_teleop_control_mode_topic_{"teleop/control_mode"};
  std::string web_teleop_tool_scale_topic_{"teleop/tool_scale"};
  double web_teleop_signal_timeout_seconds_{1.0};
  std::string web_teleop_control_mode_{"two_stick"};
  rclcpp::Time web_teleop_control_mode_last_received_;
  bool web_teleop_control_mode_has_message_{false};
  double web_teleop_tool_scale_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_joystick_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_navigation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tool_joystick_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safety_stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr web_teleop_control_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr web_teleop_tool_scale_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tool_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace amr_sweeper_sweeping_controller

#endif  // AMR_SWEEPER_SWEEPING_CONTROLLER__AMR_SWEEPER_SWEEPING_CONTROLLER_NODE_HPP_
