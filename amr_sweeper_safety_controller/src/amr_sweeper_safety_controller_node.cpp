#include "amr_sweeper_safety_controller_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <future>
#include <iomanip>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <utility>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace amr_sweeper_safety_controller
{

namespace
{

constexpr uint8_t kOdriveEstopCommandId = 0x02;
constexpr uint8_t kSteadydriveMotorOffCommand = 0x80;
constexpr uint8_t kSteadydriveMotorStopCommand = 0x81;
constexpr uint8_t kButtonPressedEvent = 0x01;
constexpr uint8_t kButtonPressedBitMask = 0x11;
constexpr uint8_t kButtonHeartbeatStatusOffsetMsb = 6;
constexpr uint8_t kButtonHeartbeatStatusOffsetLsb = 7;

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

uint32_t parseCanId(const std::string & value, const std::string & parameter_name)
{
  try {
    return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
  } catch (const std::exception & error) {
    throw std::runtime_error(
            "Invalid CAN identifier for parameter '" + parameter_name +
            "': '" + value + "' (" + error.what() + ")");
  }
}

std::string formatCanIdHex(uint32_t can_id)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << can_id;
  return stream.str();
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

  const auto safety_stop_qos = rclcpp::QoS(10).reliable().transient_local();

  stop_subscription_ = create_subscription<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_,
    safety_stop_qos,
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
  fsm_request_client_ =
    create_client<amr_sweeper_fsm::srv::RequestState>(fsm_request_service_name_);
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

SafetyControllerNode::~SafetyControllerNode()
{
  closeCanInterface(odrive_can_state_);
  closeCanInterface(steadydrive_can_state_);
  closeCanInterface(button_can_state_);
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
    "wheel_hardware_stop_topic", std::string("drive_controller/cmd_vel"));
  tool_hardware_stop_topic_ = declare_parameter(
    "tool_hardware_stop_topic", std::string("tool_controller/commands"));

  mission_stop_enabled_ = declare_parameter("mission_stop_enabled", true);
  direct_can_motor_stop_enabled_ = declare_parameter("direct_can_motor_stop_enabled", true);
  odrive_direct_can_stop_enabled_ = declare_parameter("odrive_direct_can_stop_enabled", true);
  odrive_can_interface_ = declare_parameter("odrive_can_interface", std::string("can0"));
  const auto odrive_node_id_values = declare_parameter<std::vector<int64_t>>(
    "odrive_node_ids", std::vector<int64_t>{0, 2});
  odrive_node_ids_.clear();
  odrive_node_ids_.reserve(odrive_node_id_values.size());
  for (const auto value : odrive_node_id_values) {
    if (value < 0 || value > 63) {
      throw std::runtime_error("odrive_node_ids entries must stay within the 6-bit ODrive node ID range");
    }
    odrive_node_ids_.push_back(static_cast<uint32_t>(value));
  }

  steadydrive_direct_can_stop_enabled_ = declare_parameter("steadydrive_direct_can_stop_enabled", true);
  steadydrive_can_interface_ = declare_parameter("steadydrive_can_interface", std::string("can0"));
  const auto steadydrive_motor_id_values = declare_parameter<std::vector<std::string>>(
    "steadydrive_motor_can_ids", std::vector<std::string>{"0x141", "0x142"});
  steadydrive_motor_can_ids_.clear();
  steadydrive_motor_can_ids_.reserve(steadydrive_motor_id_values.size());
  for (std::size_t index = 0; index < steadydrive_motor_id_values.size(); ++index) {
    steadydrive_motor_can_ids_.push_back(
      parseCanId(
        steadydrive_motor_id_values[index],
        "steadydrive_motor_can_ids[" + std::to_string(index) + "]"));
  }
  button_can_monitor_enabled_ = declare_parameter("button_can_monitor_enabled", true);
  button_can_interface_ = declare_parameter("button_can_interface", std::string("can0"));
  button_can_base_id_ = static_cast<uint32_t>(declare_parameter("button_can_base_id", 0x200));
  button_can_base_id_ &= 0x7FFU;
  button_status_period_ms_ = declare_parameter("button_status_period_ms", 5000);
  button_heartbeat_timeout_ms_ = declare_parameter("button_heartbeat_timeout_ms", 12000);
  fsm_request_service_name_ = declare_parameter("fsm_request_service", std::string("request_state"));
  fsm_fault_profile_id_ = static_cast<uint16_t>(declare_parameter("fsm_fault_profile_id", 400));
  fsm_fault_request_priority_ = static_cast<uint8_t>(declare_parameter("fsm_fault_request_priority", 255));

  end_mission_service_name_ = declare_parameter("end_mission_service", "end_mission");
  clear_safety_stop_service_names_ = declare_parameter<std::vector<std::string>>(
    "clear_safety_stop_services",
    std::vector<std::string>{
      "/odrive_ros2_control/clear_safety_stop",
      "/steadydrive_ros2_control/clear_safety_stop"});
  web_status_topic_ = declare_parameter(
    "web_status_topic", std::string("safety_controller/web_status"));

  odrive_can_state_.interface_name = odrive_can_interface_;
  steadydrive_can_state_.interface_name = steadydrive_can_interface_;
  button_can_state_.interface_name = button_can_interface_;
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
    fsm_fault_requested_ = false;
    mission_stop_placeholder_logged_ = false;
    direct_motor_stop_failure_logged_ = false;

    RCLCPP_ERROR(
      get_logger(),
      "Latched safety stop from '%s': %s",
      active_stop_event_.sender.c_str(),
      active_stop_event_.reason.c_str());
  }

  publishZeroCommands();
  publishDirectHardwareStopCommands();
  publishDirectMotorStopCommands();
  requestMissionStop();
  requestFsmFaultState();
  publishStatus();
}

void SafetyControllerNode::clearLatchedStop()
{
  latched_stop_active_ = false;
  mission_stop_requested_ = false;
  fsm_fault_requested_ = false;
  mission_stop_placeholder_logged_ = false;
  direct_motor_stop_failure_logged_ = false;
  direct_motor_stop_healthy_ = true;
  direct_motor_stop_last_failure_message_.clear();
  active_stop_event_ = amr_sweeper_safety_msgs::msg::SafetyStop();
  latched_stop_events_.clear();
  publishStatus();
}

void SafetyControllerNode::onPublishTimer()
{
  if (enabled_) {
    pollButtonCanFrames();
    checkButtonHeartbeatWatchdog();
  }

  if (enabled_ && latched_stop_active_) {
    publishZeroCommands();
    publishDirectHardwareStopCommands();
    publishDirectMotorStopCommands();
    requestMissionStop();
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

void SafetyControllerNode::publishDirectMotorStopCommands()
{
  std::string failure_message;
  direct_motor_stop_healthy_ = sendDirectMotorStopCommands(failure_message);
  if (direct_motor_stop_healthy_) {
    direct_motor_stop_failure_logged_ = false;
    direct_motor_stop_last_failure_message_.clear();
    return;
  }

  direct_motor_stop_last_failure_message_ = failure_message;
  if (!direct_motor_stop_failure_logged_) {
    RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
    direct_motor_stop_failure_logged_ = true;
  }
}

void SafetyControllerNode::pollButtonCanFrames()
{
  if (!button_can_monitor_enabled_) {
    return;
  }

  if (!ensureCanInterface(button_can_state_, "Button CAN monitor")) {
    return;
  }

  while (true) {
    struct can_frame frame {};
    const ssize_t bytes_read = ::read(button_can_state_.socket_fd, &frame, sizeof(frame));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Button CAN monitor read failed on '%s': %s",
        button_can_state_.interface_name.c_str(),
        std::strerror(errno));
      closeCanInterface(button_can_state_);
      return;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    std::vector<uint8_t> payload(frame.data, frame.data + frame.can_dlc);
    (void)parseButtonCanFrame(frame.can_id & CAN_SFF_MASK, payload);
  }
}

void SafetyControllerNode::checkButtonHeartbeatWatchdog()
{
  if (!button_can_monitor_enabled_ || latched_stop_active_ || !button_heartbeat_seen_) {
    return;
  }

  const auto elapsed = now() - button_last_status_frame_time_;
  if (elapsed.nanoseconds() < 0) {
    return;
  }

  if (elapsed > rclcpp::Duration::from_seconds(
      static_cast<double>(button_heartbeat_timeout_ms_) / 1000.0))
  {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = toBuiltinTime(now());
    stop_msg.sender = "button_module_watchdog";
    std::ostringstream reason;
    reason
      << "button module heartbeat missing on CAN status ID "
      << formatCanIdHex((button_can_base_id_ + 1U) & 0x7FFU)
      << " for more than " << button_heartbeat_timeout_ms_ << " ms";
    stop_msg.reason = reason.str();
    latchStop(stop_msg);
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
  if (latched_stop_active_ && !direct_motor_stop_healthy_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  auto status = makeStatus("safety_controller", level, message);
  status.values.push_back(keyValue("stop_active", latched_stop_active_ ? "true" : "false"));
  status.values.push_back(keyValue("sender", active_stop_event_.sender));
  status.values.push_back(keyValue("reason", active_stop_event_.reason));
  status.values.push_back(keyValue("direct_motor_stop_healthy", direct_motor_stop_healthy_ ? "true" : "false"));
  status.values.push_back(keyValue("direct_motor_stop_error", direct_motor_stop_last_failure_message_));
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
  stream << "\"direct_motor_stop_healthy\":"
         << (direct_motor_stop_healthy_ ? "true" : "false") << ',';
  stream << "\"direct_motor_stop_error\":\""
         << jsonEscape(direct_motor_stop_last_failure_message_) << "\",";
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
        "Zero wheel/tool commands, direct hardware stop commands, and direct CAN motor stop "
        "commands remain active while waiting for the mission stack to recover.",
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
    "Safety controller requested mission stop via '%s' while continuing to publish zero wheel/tool "
    "commands, direct hardware stop commands, and direct CAN motor stop commands.",
    end_mission_service_name_.c_str());
}

void SafetyControllerNode::requestFsmFaultState()
{
  if (fsm_fault_requested_) {
    return;
  }
  if (!fsm_request_client_) {
    RCLCPP_ERROR(get_logger(), "FSM FAULT requested by safety controller, but request_state client is unavailable");
    return;
  }
  if (!fsm_request_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(),
      "FSM FAULT requested by safety controller, but service '%s' is not ready",
      fsm_request_service_name_.c_str());
    return;
  }

  auto request = std::make_shared<amr_sweeper_fsm::srv::RequestState::Request>();
  request->target_state = "FAULT";
  request->target_lifecycle = "Active";
  request->target_profile_id = fsm_fault_profile_id_;
  request->requester = "safety_controller";
  request->priority = fsm_fault_request_priority_;
  request->force = true;
  request->reason = "latched safety stop";
  request->mission_execution_directory = "";

  fsm_fault_requested_ = true;
  fsm_request_client_->async_send_request(request);
  RCLCPP_ERROR(
    get_logger(),
    "Safety controller requested FSM FAULT via '%s' using profile %u",
    fsm_request_service_name_.c_str(),
    static_cast<unsigned int>(fsm_fault_profile_id_));
}

bool SafetyControllerNode::ensureCanInterface(
  CanInterfaceState & can_interface,
  const std::string & description)
{
  if (can_interface.socket_fd >= 0) {
    return true;
  }

  const int socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0) {
    direct_motor_stop_last_failure_message_ =
      description + ": failed to create SocketCAN socket on '" + can_interface.interface_name +
      "': " + std::strerror(errno);
    return false;
  }

  const int can_loopback = 0;
  (void)setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &can_loopback, sizeof(can_loopback));
  const int current_flags = ::fcntl(socket_fd, F_GETFL, 0);
  if (current_flags >= 0) {
    (void)::fcntl(socket_fd, F_SETFL, current_flags | O_NONBLOCK);
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface.interface_name.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (::ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    direct_motor_stop_last_failure_message_ =
      description + ": ioctl(SIOCGIFINDEX) failed on '" + can_interface.interface_name +
      "': " + std::strerror(errno);
    ::close(socket_fd);
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    direct_motor_stop_last_failure_message_ =
      description + ": bind failed on '" + can_interface.interface_name + "': " +
      std::strerror(errno);
    ::close(socket_fd);
    return false;
  }

  can_interface.socket_fd = socket_fd;
  return true;
}

void SafetyControllerNode::closeCanInterface(CanInterfaceState & can_interface)
{
  if (can_interface.socket_fd >= 0) {
    ::close(can_interface.socket_fd);
    can_interface.socket_fd = -1;
  }
}

bool SafetyControllerNode::sendCanFrame(
  CanInterfaceState & can_interface,
  uint32_t can_id,
  const std::vector<uint8_t> & payload,
  const std::string & description)
{
  if (!ensureCanInterface(can_interface, description)) {
    return false;
  }

  if (payload.size() > CAN_MAX_DLEN) {
    direct_motor_stop_last_failure_message_ =
      description + ": payload too large for standard CAN frame";
    return false;
  }

  struct can_frame frame {};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(payload.size());
  std::copy(payload.begin(), payload.end(), frame.data);

  if (::write(can_interface.socket_fd, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    direct_motor_stop_last_failure_message_ =
      description + ": write failed on '" + can_interface.interface_name + "': " +
      std::strerror(errno);
    closeCanInterface(can_interface);
    return false;
  }

  return true;
}

bool SafetyControllerNode::sendOdriveEstopCommands()
{
  if (!odrive_direct_can_stop_enabled_ || odrive_node_ids_.empty()) {
    return true;
  }

  for (const auto node_id : odrive_node_ids_) {
    const uint32_t can_id = (node_id << 5) | kOdriveEstopCommandId;
    if (!sendCanFrame(
        odrive_can_state_,
        can_id,
        {},
        "ODrive direct e-stop for node " + std::to_string(node_id)))
    {
      return false;
    }
  }

  return true;
}

bool SafetyControllerNode::sendSteadydriveStopCommands()
{
  if (!steadydrive_direct_can_stop_enabled_ || steadydrive_motor_can_ids_.empty()) {
    return true;
  }

  const std::vector<uint8_t> stop_payload{
    kSteadydriveMotorStopCommand, 0, 0, 0, 0, 0, 0, 0};
  const std::vector<uint8_t> off_payload{
    kSteadydriveMotorOffCommand, 0, 0, 0, 0, 0, 0, 0};

  for (const auto motor_can_id : steadydrive_motor_can_ids_) {
    const auto can_id_hex = formatCanIdHex(motor_can_id);
    if (!sendCanFrame(
        steadydrive_can_state_,
        motor_can_id,
        stop_payload,
        "Steadydrive direct stop for CAN ID " + can_id_hex))
    {
      return false;
    }
    if (!sendCanFrame(
        steadydrive_can_state_,
        motor_can_id,
        off_payload,
        "Steadydrive direct motor-off for CAN ID " + can_id_hex))
    {
      return false;
    }
  }

  return true;
}

bool SafetyControllerNode::sendDirectMotorStopCommands(std::string & failure_message)
{
  if (!direct_can_motor_stop_enabled_) {
    failure_message.clear();
    return true;
  }

  if (!sendOdriveEstopCommands() || !sendSteadydriveStopCommands()) {
    failure_message = direct_motor_stop_last_failure_message_.empty() ?
      "Failed to send direct CAN motor stop command" : direct_motor_stop_last_failure_message_;
    return false;
  }

  failure_message.clear();
  return true;
}

bool SafetyControllerNode::parseButtonCanFrame(uint32_t can_id, const std::vector<uint8_t> & payload)
{
  const uint32_t event_can_id = button_can_base_id_;
  const uint32_t status_can_id = (button_can_base_id_ + 1U) & 0x7FFU;

  if (can_id == event_can_id && !payload.empty() && payload[0] == kButtonPressedEvent) {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = toBuiltinTime(now());
    stop_msg.sender = "button_module_can";
    stop_msg.reason = "physical safety stop button pressed (event frame)";
    latchStop(stop_msg);
    return true;
  }

  if (can_id == status_can_id && payload.size() >= 2 && (payload[1] & kButtonPressedBitMask) != 0U) {
    if (payload.size() >= 8) {
      button_last_status_frame_time_ = now();
      button_last_heartbeat_counter_ = static_cast<uint16_t>(
        (static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetMsb]) << 8) |
        static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetLsb]));
      button_heartbeat_seen_ = true;
    }
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = toBuiltinTime(now());
    stop_msg.sender = "button_module_can";
    stop_msg.reason = "physical safety stop button pressed (status frame)";
    latchStop(stop_msg);
    return true;
  }

  if (can_id == status_can_id && payload.size() >= 8) {
    const uint16_t heartbeat_counter = static_cast<uint16_t>(
      (static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetMsb]) << 8) |
      static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetLsb]));
    if (!button_heartbeat_seen_ || heartbeat_counter != button_last_heartbeat_counter_) {
      button_last_status_frame_time_ = now();
      button_last_heartbeat_counter_ = heartbeat_counter;
      button_heartbeat_seen_ = true;
    }
  }

  return false;
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
