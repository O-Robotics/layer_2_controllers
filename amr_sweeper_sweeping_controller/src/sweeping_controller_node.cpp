#include "sweeping_controller_node.hpp"

#include <functional>
#include <sstream>
#include <utility>

namespace amr_sweeper_sweeping_controller
{

namespace
{

geometry_msgs::msg::Twist zeroTwist()
{
  return geometry_msgs::msg::Twist{};
}

}  // namespace

SweepingControllerNode::SweepingControllerNode(const rclcpp::NodeOptions & options)
: Node("sweeping_controller_node", options)
{
  loadParameters();
  if (tool_navigation_mode_ != "mapping" && tool_navigation_mode_ != "constant_speed") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown tool navigation mode '%s'; falling back to 'mapping'",
      tool_navigation_mode_.c_str());
    tool_navigation_mode_ = "mapping";
  }
  createSubscriptions();

  wheel_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    wheel_output_topic_,
    rclcpp::SystemDefaultsQoS());
  tool_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    tool_output_topic_,
    rclcpp::SystemDefaultsQoS());
  status_publisher_ = create_publisher<std_msgs::msg::String>(
    status_topic_,
    rclcpp::SystemDefaultsQoS());
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&SweepingControllerNode::publishSelectedCommands, this));

  RCLCPP_INFO(
    get_logger(),
    "Sweeping controller publishing wheels to '%s' and tools to '%s'",
    wheel_output_topic_.c_str(),
    tool_output_topic_.c_str());
}

void SweepingControllerNode::loadParameters()
{
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  wheel_output_topic_ = declare_parameter(
    "wheel_output_topic", std::string{"sweeping_controller/cmd_vel_drive"});
  tool_output_topic_ = declare_parameter(
    "tool_output_topic", std::string{"sweeping_controller/cmd_vel_tools"});
  status_topic_ = declare_parameter("status_topic", std::string{"sweeping_controller/status"});
  publish_idle_commands_ = declare_parameter("publish_idle_commands", false);

  wheel_safety_source_.config.enabled = declare_parameter("wheel_sources.safety_stop.enabled", true);
  wheel_safety_source_.config.topic = declare_parameter(
    "wheel_sources.safety_stop.topic",
    std::string{"safety_controller/cmd_vel_safety_stop"});
  wheel_safety_source_.config.timeout_seconds = declare_parameter(
    "wheel_sources.safety_stop.timeout_seconds",
    0.5);
  wheel_safety_source_.config.priority = declare_parameter("wheel_sources.safety_stop.priority", 255);

  wheel_joystick_source_.config.enabled = declare_parameter("wheel_sources.joystick.enabled", true);
  wheel_joystick_source_.config.topic = declare_parameter(
    "wheel_sources.joystick.topic",
    std::string{"teleop/cmd_vel_drive"});
  wheel_joystick_source_.config.timeout_seconds = declare_parameter(
    "wheel_sources.joystick.timeout_seconds",
    1.0);
  wheel_joystick_source_.config.priority = declare_parameter("wheel_sources.joystick.priority", 100);

  wheel_navigation_source_.config.enabled = declare_parameter("wheel_sources.navigation.enabled", true);
  wheel_navigation_source_.config.topic = declare_parameter(
    "wheel_sources.navigation.topic",
    std::string{"navigation/cmd_vel"});
  wheel_navigation_source_.config.timeout_seconds = declare_parameter(
    "wheel_sources.navigation.timeout_seconds",
    0.5);
  wheel_navigation_source_.config.priority = declare_parameter("wheel_sources.navigation.priority", 5);

  tool_safety_source_.config.enabled = declare_parameter("tool_sources.safety_stop.enabled", true);
  tool_safety_source_.config.topic = declare_parameter(
    "tool_sources.safety_stop.topic",
    std::string{"safety_controller/cmd_vel_safety_stop"});
  tool_safety_source_.config.timeout_seconds = declare_parameter(
    "tool_sources.safety_stop.timeout_seconds",
    0.5);
  tool_safety_source_.config.priority = declare_parameter("tool_sources.safety_stop.priority", 255);

  tool_joystick_source_.config.enabled = declare_parameter("tool_sources.joystick.enabled", true);
  tool_joystick_source_.config.topic = declare_parameter(
    "tool_sources.joystick.topic",
    std::string{"teleop/cmd_vel_tools"});
  tool_joystick_source_.config.timeout_seconds = declare_parameter(
    "tool_sources.joystick.timeout_seconds",
    1.0);
  tool_joystick_source_.config.priority = declare_parameter("tool_sources.joystick.priority", 100);

  tool_navigation_source_.config.enabled = declare_parameter("tool_sources.navigation.enabled", true);
  tool_navigation_source_.config.topic = declare_parameter(
    "tool_sources.navigation.topic",
    std::string{"navigation/cmd_vel"});
  tool_navigation_source_.config.timeout_seconds = declare_parameter(
    "tool_sources.navigation.timeout_seconds",
    0.5);
  tool_navigation_source_.config.priority = declare_parameter("tool_sources.navigation.priority", 5);

  tool_navigation_mapping_enabled_ = declare_parameter("tool_sources.navigation.mapping_enabled", true);
  tool_navigation_mode_ = declare_parameter(
    "tool_sources.navigation.mode",
    std::string{"mapping"});
  tool_navigation_linear_x_offset_ = declare_parameter(
    "tool_sources.navigation.linear_x_offset",
    0.0);
  tool_navigation_linear_x_from_linear_x_gain_ = declare_parameter(
    "tool_sources.navigation.linear_x_from_linear_x_gain",
    1.0);
  tool_navigation_linear_x_from_angular_z_gain_ = declare_parameter(
    "tool_sources.navigation.linear_x_from_angular_z_gain",
    0.0);
  tool_navigation_angular_z_offset_ = declare_parameter(
    "tool_sources.navigation.angular_z_offset",
    0.0);
  tool_navigation_angular_z_from_linear_x_gain_ = declare_parameter(
    "tool_sources.navigation.angular_z_from_linear_x_gain",
    0.0);
  tool_navigation_angular_z_from_angular_z_gain_ = declare_parameter(
    "tool_sources.navigation.angular_z_from_angular_z_gain",
    1.0);
  tool_navigation_constant_linear_x_ = declare_parameter(
    "tool_sources.navigation.constant_linear_x",
    0.0);
  tool_navigation_constant_angular_z_ = declare_parameter(
    "tool_sources.navigation.constant_angular_z",
    0.0);
}

void SweepingControllerNode::createSubscriptions()
{
  if (wheel_joystick_source_.config.enabled) {
    wheel_joystick_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      wheel_joystick_source_.config.topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SweepingControllerNode::handleWheelJoystickCommand, this, std::placeholders::_1));
  }
  if (wheel_navigation_source_.config.enabled) {
    wheel_navigation_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      wheel_navigation_source_.config.topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SweepingControllerNode::handleWheelNavigationCommand, this, std::placeholders::_1));
  }
  if (tool_joystick_source_.config.enabled) {
    tool_joystick_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      tool_joystick_source_.config.topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SweepingControllerNode::handleToolJoystickCommand, this, std::placeholders::_1));
  }
  if (wheel_safety_source_.config.enabled || tool_safety_source_.config.enabled) {
    safety_stop_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      wheel_safety_source_.config.topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SweepingControllerNode::handleSafetyStopCommand, this, std::placeholders::_1));
  }
}

void SweepingControllerNode::publishSelectedCommands()
{
  const auto now = this->now();
  const auto [wheel_source, wheel_command] = selectWheelCommand(now);
  const auto [tool_source, tool_command] = selectToolCommand(now);

  if (publish_idle_commands_ || hasActiveWheelSource(now)) {
    wheel_command_publisher_->publish(wheel_command);
  }
  if (publish_idle_commands_ || hasActiveToolSource(now)) {
    tool_command_publisher_->publish(tool_command);
  }
  publishStatus(wheel_source, tool_source);
}

void SweepingControllerNode::publishStatus(
  const std::string & wheel_source,
  const std::string & tool_source) const
{
  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "wheel_source=" << wheel_source
         << "; tool_source=" << tool_source
         << "; wheel_output_topic=" << wheel_output_topic_
         << "; tool_output_topic=" << tool_output_topic_;
  message.data = stream.str();
  status_publisher_->publish(message);
}

void SweepingControllerNode::handleWheelJoystickCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }
  storeCommand(wheel_joystick_source_, *message);
}

void SweepingControllerNode::handleWheelNavigationCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }
  storeCommand(wheel_navigation_source_, *message);
  if (tool_navigation_source_.config.enabled && tool_navigation_mapping_enabled_) {
    storeCommand(tool_navigation_source_, buildToolNavigationCommand(*message));
  }
}

void SweepingControllerNode::handleToolJoystickCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }
  storeCommand(tool_joystick_source_, *message);
}

void SweepingControllerNode::handleSafetyStopCommand(
  const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }
  storeCommand(wheel_safety_source_, *message);
  storeCommand(tool_safety_source_, *message);
}

void SweepingControllerNode::storeCommand(
  CommandSourceState & state,
  const geometry_msgs::msg::Twist & command)
{
  state.latest_command = command;
  state.last_received = now();
  state.has_message = true;
}

geometry_msgs::msg::Twist SweepingControllerNode::buildToolNavigationCommand(
  const geometry_msgs::msg::Twist & wheel_navigation_command) const
{
  if (tool_navigation_mode_ == "constant_speed") {
    return buildConstantToolNavigationCommand();
  }

  geometry_msgs::msg::Twist tool_command;
  tool_command.linear.x =
    tool_navigation_linear_x_offset_ +
    (wheel_navigation_command.linear.x * tool_navigation_linear_x_from_linear_x_gain_) +
    (wheel_navigation_command.angular.z * tool_navigation_linear_x_from_angular_z_gain_);
  tool_command.angular.z =
    tool_navigation_angular_z_offset_ +
    (wheel_navigation_command.linear.x * tool_navigation_angular_z_from_linear_x_gain_) +
    (wheel_navigation_command.angular.z * tool_navigation_angular_z_from_angular_z_gain_);
  return tool_command;
}

geometry_msgs::msg::Twist SweepingControllerNode::buildConstantToolNavigationCommand() const
{
  geometry_msgs::msg::Twist tool_command;
  tool_command.linear.x = tool_navigation_constant_linear_x_;
  tool_command.angular.z = tool_navigation_constant_angular_z_;
  return tool_command;
}

bool SweepingControllerNode::isSourceActive(
  const CommandSourceState & state,
  const rclcpp::Time & now) const
{
  if (!state.config.enabled || !state.has_message) {
    return false;
  }
  return (now - state.last_received).seconds() <= state.config.timeout_seconds;
}

std::pair<std::string, geometry_msgs::msg::Twist> SweepingControllerNode::selectWheelCommand(
  const rclcpp::Time & now) const
{
  const std::unordered_map<std::string, const CommandSourceState *> sources{
    {"safety_stop", &wheel_safety_source_},
    {"joystick", &wheel_joystick_source_},
    {"navigation", &wheel_navigation_source_},
  };

  const CommandSourceState * selected = nullptr;
  std::string selected_name{"idle"};
  for (const auto & [name, state] : sources) {
    if (!isSourceActive(*state, now)) {
      continue;
    }
    if (!selected || state->config.priority > selected->config.priority) {
      selected = state;
      selected_name = name;
    }
  }

  if (!selected) {
    return {selected_name, zeroTwist()};
  }
  return {selected_name, selected->latest_command};
}

bool SweepingControllerNode::hasActiveWheelSource(const rclcpp::Time & now) const
{
  return isSourceActive(wheel_safety_source_, now) ||
         isSourceActive(wheel_joystick_source_, now) ||
         isSourceActive(wheel_navigation_source_, now);
}

std::pair<std::string, geometry_msgs::msg::Twist> SweepingControllerNode::selectToolCommand(
  const rclcpp::Time & now) const
{
  const std::unordered_map<std::string, const CommandSourceState *> sources{
    {"safety_stop", &tool_safety_source_},
    {"joystick", &tool_joystick_source_},
    {"navigation", &tool_navigation_source_},
  };

  const CommandSourceState * selected = nullptr;
  std::string selected_name{"idle"};
  for (const auto & [name, state] : sources) {
    if (!isSourceActive(*state, now)) {
      continue;
    }
    if (!selected || state->config.priority > selected->config.priority) {
      selected = state;
      selected_name = name;
    }
  }

  if (!selected) {
    return {selected_name, zeroTwist()};
  }
  return {selected_name, selected->latest_command};
}

bool SweepingControllerNode::hasActiveToolSource(const rclcpp::Time & now) const
{
  return isSourceActive(tool_safety_source_, now) ||
         isSourceActive(tool_joystick_source_, now) ||
         isSourceActive(tool_navigation_source_, now);
}

}  // namespace amr_sweeper_sweeping_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_sweeping_controller::SweepingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
