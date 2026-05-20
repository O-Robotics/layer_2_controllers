#include "safety_controller_node.hpp"

#include <chrono>
#include <sstream>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace amr_sweeper_safety_controller
{

namespace
{

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
  const std::string & name,
  const unsigned char level,
  const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "amr_sweeper_safety_controller";
  status.level = level;
  status.message = message;
  return status;
}

builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

SafetyControllerNode::SafetyControllerNode(const rclcpp::NodeOptions & options)
: Node("safety_controller", options)
{
  loadParameters();

  stop_subscription_ = create_subscription<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&SafetyControllerNode::onStopMessage, this, std::placeholders::_1));

  wheel_stop_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    wheel_stop_topic_, rclcpp::SystemDefaultsQoS());
  tool_stop_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    tool_stop_topic_, rclcpp::SystemDefaultsQoS());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "safety_controller/status", rclcpp::SystemDefaultsQoS());

  reset_latched_stop_service_ = create_service<std_srvs::srv::Trigger>(
    "amr_sweeper_safety_controller/reset_latched_stop",
    std::bind(
      &SafetyControllerNode::resetLatchedStopService, this, std::placeholders::_1,
      std::placeholders::_2));
  enable_controller_service_ = create_service<std_srvs::srv::SetBool>(
    "amr_sweeper_safety_controller/enable_controller",
    std::bind(
      &SafetyControllerNode::enableControllerService, this, std::placeholders::_1,
      std::placeholders::_2));

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&SafetyControllerNode::onPublishTimer, this));

  publishStatus();
}

void SafetyControllerNode::loadParameters()
{
  enabled_ = declare_parameter("enabled", true);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive; using 20.0 Hz");
    publish_rate_hz_ = 20.0;
  }

  stop_topic_name_ = declare_parameter("stop_topic_name", "safety_msgs/stop");
  wheel_stop_topic_ = declare_parameter("wheel_stop_topic", "cmd_vel_safety_stop");
  tool_stop_topic_ = declare_parameter("tool_stop_topic", "cmd_vel_joy_tools");
  publish_zero_tool_command_ = declare_parameter("publish_zero_tool_command", true);

  navigation_stop_placeholder_enabled_ =
    declare_parameter("navigation_stop_placeholder_enabled", true);
  motor_stop_placeholder_enabled_ = declare_parameter("motor_stop_placeholder_enabled", true);
  future_motor_stop_interfaces_ = declare_parameter<std::vector<std::string>>(
    "future_motor_stop_interfaces", std::vector<std::string>{});
}

void SafetyControllerNode::onStopMessage(
  const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  latchStop(*msg);
}

void SafetyControllerNode::latchStop(
  const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event)
{
  amr_sweeper_safety_msgs::msg::SafetyStop normalized_event = stop_event;
  if (normalized_event.sender.empty()) {
    normalized_event.sender = "unknown_sender";
  }
  if (normalized_event.reason.empty()) {
    normalized_event.reason = "stop requested";
  }
  if (normalized_event.status.empty()) {
    normalized_event.status = "STOP";
  }
  if (normalized_event.stamp.sec == 0 && normalized_event.stamp.nanosec == 0) {
    normalized_event.stamp = toBuiltinTime(now());
  }

  const bool same_as_active =
    latched_stop_active_ &&
    active_stop_event_.sender == normalized_event.sender &&
    active_stop_event_.reason == normalized_event.reason &&
    active_stop_event_.status == normalized_event.status &&
    active_stop_event_.value == normalized_event.value;

  active_stop_event_ = normalized_event;
  latched_stop_active_ = true;
  if (!same_as_active) {
    navigation_placeholder_logged_ = false;
    motor_stop_placeholder_logged_ = false;

    RCLCPP_ERROR(
      get_logger(),
      "Latched safety stop from '%s': %s (status=%s, value=%.3f)",
      active_stop_event_.sender.c_str(),
      active_stop_event_.reason.c_str(),
      active_stop_event_.status.c_str(),
      active_stop_event_.value);
  }

  publishZeroCommands();
  requestNavigationStopPlaceholder();
  requestMotorStopPlaceholder();
  publishStatus();
}

void SafetyControllerNode::clearLatchedStop()
{
  latched_stop_active_ = false;
  navigation_placeholder_logged_ = false;
  motor_stop_placeholder_logged_ = false;
  active_stop_event_ = amr_sweeper_safety_msgs::msg::SafetyStop();
  publishStatus();
}

void SafetyControllerNode::onPublishTimer()
{
  if (enabled_ && latched_stop_active_) {
    publishZeroCommands();
    requestNavigationStopPlaceholder();
    requestMotorStopPlaceholder();
  }

  publishStatus();
}

void SafetyControllerNode::publishZeroCommands()
{
  geometry_msgs::msg::Twist zero_twist;
  wheel_stop_publisher_->publish(zero_twist);

  if (publish_zero_tool_command_) {
    tool_stop_publisher_->publish(zero_twist);
  }
}

void SafetyControllerNode::publishStatus()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "idle";
  if (!enabled_) {
    message = "disabled";
  } else if (latched_stop_active_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = active_stop_event_.sender + ": " + active_stop_event_.reason;
  }

  auto status = makeStatus("safety_controller", level, message);
  status.values.push_back(keyValue("enabled", enabled_ ? "true" : "false"));
  status.values.push_back(keyValue("latched_stop_active", latched_stop_active_ ? "true" : "false"));
  status.values.push_back(keyValue("stop_topic_name", stop_topic_name_));
  status.values.push_back(keyValue("wheel_stop_topic", wheel_stop_topic_));
  status.values.push_back(keyValue("tool_stop_topic", tool_stop_topic_));
  status.values.push_back(
    keyValue("publish_zero_tool_command", publish_zero_tool_command_ ? "true" : "false"));
  status.values.push_back(keyValue("sender", active_stop_event_.sender));
  status.values.push_back(keyValue("reason", active_stop_event_.reason));
  status.values.push_back(keyValue("status", active_stop_event_.status));
  status.values.push_back(keyValue("value", std::to_string(active_stop_event_.value)));
  status.values.push_back(
    keyValue(
      "stamp",
      std::to_string(active_stop_event_.stamp.sec) + "." +
      std::to_string(active_stop_event_.stamp.nanosec)));
  array.status.push_back(status);

  diagnostics_publisher_->publish(array);
}

void SafetyControllerNode::requestNavigationStopPlaceholder()
{
  if (!navigation_stop_placeholder_enabled_ || navigation_placeholder_logged_) {
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "Navigation stop placeholder active. TODO: cancel active Nav2 goals such as "
    "navigate_to_pose, navigate_through_poses, or follow_waypoints when this interface is wired.");
  navigation_placeholder_logged_ = true;
}

void SafetyControllerNode::requestMotorStopPlaceholder()
{
  if (!motor_stop_placeholder_enabled_ || motor_stop_placeholder_logged_) {
    return;
  }

  std::ostringstream stream;
  stream << "Direct hardware stop placeholder active.";
  if (!future_motor_stop_interfaces_.empty()) {
    stream << " Reserved interfaces:";
    for (const auto & interface_name : future_motor_stop_interfaces_) {
      stream << " " << interface_name;
    }
  }

  RCLCPP_WARN(get_logger(), "%s", stream.str().c_str());
  motor_stop_placeholder_logged_ = true;
}

void SafetyControllerNode::resetLatchedStopService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  clearLatchedStop();
  response->success = true;
  response->message = "latched stop cleared";
}

void SafetyControllerNode::enableControllerService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  if (!enabled_) {
    clearLatchedStop();
  }

  response->success = true;
  response->message = enabled_ ? "safety controller enabled" : "safety controller disabled";
}

}  // namespace amr_sweeper_safety_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_safety_controller::SafetyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
