#include "amr_sweeper_safety_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <iomanip>
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

std::string jsonEscape(const std::string & value)
{
  std::ostringstream stream;
  for (const char character : value) {
    switch (character) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {
          stream << "\\u"
                 << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(static_cast<unsigned char>(character))
                 << std::dec << std::setfill(' ');
        } else {
          stream << character;
        }
        break;
    }
  }
  return stream.str();
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
  wheel_hardware_stop_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    wheel_hardware_stop_topic_, rclcpp::SystemDefaultsQoS());
  tool_hardware_stop_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    tool_hardware_stop_topic_, rclcpp::SystemDefaultsQoS());
  end_mission_client_ =
    create_client<amr_sweeper_mission_executor::srv::EndMission>(end_mission_service_name_);
  clear_safety_stop_clients_.clear();
  for (const auto & service_name : clear_safety_stop_service_names_) {
    clear_safety_stop_clients_.push_back(create_client<std_srvs::srv::Trigger>(service_name));
  }
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "safety_controller/status", rclcpp::SystemDefaultsQoS());
  web_status_publisher_ = create_publisher<std_msgs::msg::String>(
    web_status_topic_, rclcpp::SystemDefaultsQoS());

  reset_latched_stop_service_ = create_service<std_srvs::srv::Trigger>(
    "amr_sweeper_safety_controller/reset_latched_stop",
    std::bind(
      &SafetyControllerNode::resetLatchedStopService, this, std::placeholders::_1,
      std::placeholders::_2));
  clear_safety_stop_service_ = create_service<std_srvs::srv::Trigger>(
    "amr_sweeper_safety_controller/clear_safety_stop",
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
  publish_direct_hardware_stop_ = declare_parameter("publish_direct_hardware_stop", true);
  wheel_hardware_stop_topic_ = declare_parameter(
    "wheel_hardware_stop_topic", std::string("diff_cont/cmd_vel"));
  tool_hardware_stop_topic_ = declare_parameter(
    "tool_hardware_stop_topic", std::string("controller_steadydrive/commands"));

  mission_stop_enabled_ = declare_parameter("mission_stop_enabled", true);
  motor_stop_placeholder_enabled_ = declare_parameter("motor_stop_placeholder_enabled", true);
  future_motor_stop_interfaces_ = declare_parameter<std::vector<std::string>>(
    "future_motor_stop_interfaces", std::vector<std::string>{});
  end_mission_service_name_ = declare_parameter("end_mission_service", "end_mission");
  clear_safety_stop_service_names_ = declare_parameter<std::vector<std::string>>(
    "clear_safety_stop_services",
    std::vector<std::string>{
      "/odrive_ros2_control/clear_safety_stop",
      "/steadydrive_ros2_control/clear_safety_stop"});
  web_status_topic_ = declare_parameter(
    "web_status_topic", std::string("safety_controller/web_status"));
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
  if (normalized_event.stamp.sec == 0 && normalized_event.stamp.nanosec == 0) {
    normalized_event.stamp = toBuiltinTime(now());
  }

  const bool same_as_active =
    latched_stop_active_ &&
    active_stop_event_.sender == normalized_event.sender &&
    active_stop_event_.reason == normalized_event.reason;

  const bool already_recorded = std::any_of(
    latched_stop_events_.begin(), latched_stop_events_.end(),
    [&normalized_event](const auto & existing_event) {
      return existing_event.sender == normalized_event.sender &&
             existing_event.reason == normalized_event.reason;
    });
  if (!already_recorded) {
    latched_stop_events_.push_back(normalized_event);
  }

  active_stop_event_ = normalized_event;
  latched_stop_active_ = true;
  if (!same_as_active) {
    mission_stop_requested_ = false;
    mission_stop_placeholder_logged_ = false;
    motor_stop_placeholder_logged_ = false;

    RCLCPP_ERROR(
      get_logger(),
      "Latched safety stop from '%s': %s",
      active_stop_event_.sender.c_str(),
      active_stop_event_.reason.c_str());
  }

  publishZeroCommands();
  publishDirectHardwareStopCommands();
  requestMissionStop();
  requestMotorStopPlaceholder();
  publishStatus();
}

void SafetyControllerNode::clearLatchedStop()
{
  latched_stop_active_ = false;
  mission_stop_requested_ = false;
  mission_stop_placeholder_logged_ = false;
  motor_stop_placeholder_logged_ = false;
  active_stop_event_ = amr_sweeper_safety_msgs::msg::SafetyStop();
  latched_stop_events_.clear();
  publishStatus();
}

void SafetyControllerNode::onPublishTimer()
{
  if (enabled_ && latched_stop_active_) {
    publishZeroCommands();
    publishDirectHardwareStopCommands();
    requestMissionStop();
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

void SafetyControllerNode::publishDirectHardwareStopCommands()
{
  if (!publish_direct_hardware_stop_) {
    return;
  }

  geometry_msgs::msg::TwistStamped zero_wheel_command;
  zero_wheel_command.header.stamp = now();
  wheel_hardware_stop_publisher_->publish(zero_wheel_command);

  std_msgs::msg::Float64MultiArray zero_tool_command;
  zero_tool_command.data = {0.0, 0.0};
  tool_hardware_stop_publisher_->publish(zero_tool_command);
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
  status.values.push_back(keyValue("stop_active", latched_stop_active_ ? "true" : "false"));
  status.values.push_back(keyValue("sender", active_stop_event_.sender));
  status.values.push_back(keyValue("reason", active_stop_event_.reason));
  array.status.push_back(status);

  diagnostics_publisher_->publish(array);
  publishWebStatus();
}

void SafetyControllerNode::publishWebStatus()
{
  std_msgs::msg::String message;
  message.data = buildWebStatusJson();
  web_status_publisher_->publish(message);
}

std::string SafetyControllerNode::buildWebStatusJson() const
{
  std::ostringstream stream;
  stream << '{';
  stream << "\"latched\":" << (latched_stop_active_ ? "true" : "false") << ',';
  stream << "\"enabled\":" << (enabled_ ? "true" : "false") << ',';
  stream << "\"active_sender\":\"" << jsonEscape(active_stop_event_.sender) << "\",";
  stream << "\"active_reason\":\"" << jsonEscape(active_stop_event_.reason) << "\",";
  stream << "\"causes\":[";
  for (std::size_t index = 0; index < latched_stop_events_.size(); ++index) {
    const auto & event = latched_stop_events_[index];
    if (index > 0) {
      stream << ',';
    }
    stream << '{'
           << "\"sender\":\"" << jsonEscape(event.sender) << "\","
           << "\"reason\":\"" << jsonEscape(event.reason) << "\","
           << "\"stamp\":{"
           << "\"sec\":" << event.stamp.sec << ','
           << "\"nanosec\":" << event.stamp.nanosec
           << "}}";
  }
  stream << "]}";
  return stream.str();
}

void SafetyControllerNode::requestMissionStop()
{
  if (!mission_stop_enabled_ || mission_stop_requested_) {
    return;
  }

  if (!end_mission_client_->service_is_ready()) {
    if (!mission_stop_placeholder_logged_) {
      RCLCPP_WARN(
        get_logger(),
        "Mission stop requested by safety controller, but end_mission service '%s' is not ready. "
        "TODO: keep this wired to mission stop and optionally add direct Nav2 goal cancellation.",
        end_mission_service_name_.c_str());
      mission_stop_placeholder_logged_ = true;
    }
    return;
  }

  auto request = std::make_shared<amr_sweeper_mission_executor::srv::EndMission::Request>();
  request->mission_id = "";
  request->reason = "latched safety stop";
  request->outcome = "ABORTED";
  request->requester = "safety_controller";
  request->priority = 255;
  request->force = true;
  request->request_idling = true;

  mission_stop_requested_ = true;
  mission_stop_placeholder_logged_ = false;
  end_mission_client_->async_send_request(request);
  RCLCPP_WARN(
    get_logger(),
    "Safety controller requested mission stop via '%s'. TODO: optionally add direct Nav2 goal cancellation too.",
    end_mission_service_name_.c_str());
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
  std::string failure_message;
  if (!clearHardwareSafetyStops(failure_message)) {
    response->success = false;
    response->message = failure_message;
    return;
  }
  clearLatchedStop();
  response->success = true;
  response->message = "safety stop cleared and motors re-enabled";
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

bool SafetyControllerNode::clearHardwareSafetyStops(std::string & failure_message)
{
  std::vector<std::string> failures;

  for (std::size_t index = 0; index < clear_safety_stop_clients_.size(); ++index) {
    const auto & client = clear_safety_stop_clients_[index];
    const auto & service_name = clear_safety_stop_service_names_[index];
    if (!client) {
      failures.push_back(service_name + ": client unavailable");
      continue;
    }

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      failures.push_back(service_name + ": service not ready");
      continue;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      failures.push_back(service_name + ": no response");
      continue;
    }

    const auto response = future.get();
    if (!response->success) {
      failures.push_back(service_name + ": " + response->message);
    }
  }

  if (!failures.empty()) {
    std::ostringstream stream;
    stream << "Failed to clear all hardware safety stops:";
    for (const auto & failure : failures) {
      stream << ' ' << '[' << failure << ']';
    }
    failure_message = stream.str();
    RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
    return false;
  }

  return true;
}

}  // namespace amr_sweeper_safety_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_safety_controller::SafetyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
