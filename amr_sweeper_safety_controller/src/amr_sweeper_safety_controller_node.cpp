#include "amr_sweeper_safety_controller_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
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
constexpr uint8_t kOdriveHeartbeatCommandId = 0x01;
constexpr uint8_t kOdriveGetEncoderEstimatesCommandId = 0x09;
constexpr uint8_t kSteadydriveMotorOffCommand = 0x80;
constexpr uint8_t kSteadydriveMotorStopCommand = 0x81;
constexpr uint8_t kSteadydriveReadState2Command = 0x9C;
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

uint32_t decodeUint32LE(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int16_t decodeInt16LE(const uint8_t low_byte, const uint8_t high_byte)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(low_byte) |
    (static_cast<uint16_t>(high_byte) << 8));
}

float decodeFloat32LE(const uint8_t * data)
{
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

bool allZeroPayloadExceptCommand(const std::vector<uint8_t> & payload)
{
  if (payload.empty()) {
    return true;
  }
  return std::all_of(
    payload.begin() + 1,
    payload.end(),
    [](const uint8_t value) {
      return value == 0U;
    });
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
  stop_event_publisher_ = create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_,
    safety_stop_qos);

  wheel_stop_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    wheel_stop_topic_, rclcpp::SystemDefaultsQoS());
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

  noteProgress();
  watchdog_running_.store(true);
  watchdog_thread_ = std::thread(&SafetyControllerNode::watchdogThreadMain, this);
  publishStatus();
}

SafetyControllerNode::~SafetyControllerNode()
{
  watchdog_running_.store(false);
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  closeCanInterface(odrive_can_state_);
  closeCanInterface(steadydrive_can_state_);
  closeCanInterface(button_can_state_);
  closeCanInterface(odrive_feedback_can_state_);
  closeCanInterface(steadydrive_feedback_can_state_);
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
  stop_feedback_timeout_ms_ = declare_parameter("stop_feedback_timeout_ms", 1500);
  stop_feedback_stale_ms_ = declare_parameter("stop_feedback_stale_ms", 500);
  odrive_stop_velocity_threshold_rev_s_ =
    declare_parameter("odrive_stop_velocity_threshold_rev_s", 0.05);
  steadydrive_stop_velocity_threshold_deg_s_ =
    declare_parameter("steadydrive_stop_velocity_threshold_deg_s", 5.0);
  internal_watchdog_enabled_ = declare_parameter("internal_watchdog_enabled", true);
  internal_watchdog_timeout_ms_ = declare_parameter("internal_watchdog_timeout_ms", 1500);
  internal_watchdog_check_period_ms_ =
    declare_parameter("internal_watchdog_check_period_ms", 200);
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
  odrive_feedback_can_state_.interface_name = odrive_can_interface_;
  steadydrive_feedback_can_state_.interface_name = steadydrive_can_interface_;

  odrive_feedback_states_.clear();
  odrive_feedback_states_.reserve(odrive_node_ids_.size());
  for (const auto node_id : odrive_node_ids_) {
    OdriveFeedbackState state;
    state.node_id = node_id;
    odrive_feedback_states_.push_back(state);
  }
  steadydrive_feedback_states_.clear();
  steadydrive_feedback_states_.reserve(steadydrive_motor_can_ids_.size());
  for (const auto can_id : steadydrive_motor_can_ids_) {
    SteadydriveFeedbackState state;
    state.can_id = can_id;
    steadydrive_feedback_states_.push_back(state);
  }
}

void SafetyControllerNode::onStopMessage(
  const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr msg)
{
  noteProgress();
  if (!enabled_) {
    return;
  }

  latchStop(*msg);
}

void SafetyControllerNode::noteProgress()
{
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  last_progress_time_ns_.store(now_ns);
}

void SafetyControllerNode::watchdogThreadMain()
{
  while (watchdog_running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(internal_watchdog_check_period_ms_));
    if (!internal_watchdog_enabled_ || !enabled_) {
      continue;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    const auto last_progress_ns = last_progress_time_ns_.load();
    if (last_progress_ns <= 0LL) {
      continue;
    }

    const auto last_progress_tp = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(last_progress_ns));
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_tp - last_progress_tp).count();
    if (elapsed_ms <= internal_watchdog_timeout_ms_) {
      continue;
    }
    if (internal_watchdog_stop_published_.exchange(true)) {
      continue;
    }

    geometry_msgs::msg::Twist zero_twist;
    wheel_stop_publisher_->publish(zero_twist);

    std::string failure_message;
    {
      std::lock_guard<std::mutex> lock(can_tx_mutex_);
      (void)sendDirectMotorStopCommands(failure_message);
    }

    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = toBuiltinTime(now());
    stop_msg.sender = "safety_controller_watchdog";
    std::ostringstream reason;
    reason << "internal watchdog detected stalled safety controller progress for "
           << elapsed_ms << " ms";
    if (!failure_message.empty()) {
      reason << "; direct_stop_result=" << failure_message;
    }
    stop_msg.reason = reason.str();
    stop_event_publisher_->publish(stop_msg);
  }
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
  latched_stop_since_ = now();
  direct_motor_stop_confirmed_ = false;
  direct_motor_stop_confirmation_timeout_reported_ = false;
  direct_motor_stop_confirmation_status_ = "awaiting_feedback";
  internal_watchdog_stop_published_.store(false);
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
  publishDirectMotorStopCommands();
  requestMissionStop();
  requestFsmFaultState();
  publishStatus();
}

void SafetyControllerNode::publishInternalStopEvent(
  const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event)
{
  amr_sweeper_safety_msgs::msg::SafetyStop normalized_event = stop_event;
  if (normalized_event.sender.empty()) {
    normalized_event.sender = "safety_controller";
  }
  if (normalized_event.reason.empty()) {
    normalized_event.reason = "internal safety stop";
  }
  if (normalized_event.stamp.sec == 0 && normalized_event.stamp.nanosec == 0) {
    normalized_event.stamp = toBuiltinTime(now());
  }

  stop_event_publisher_->publish(normalized_event);
  latchStop(normalized_event);
}

void SafetyControllerNode::clearLatchedStop()
{
  latched_stop_active_ = false;
  mission_stop_requested_ = false;
  fsm_fault_requested_ = false;
  mission_stop_placeholder_logged_ = false;
  direct_motor_stop_failure_logged_ = false;
  direct_motor_stop_healthy_ = true;
  direct_motor_stop_confirmed_ = false;
  direct_motor_stop_confirmation_timeout_reported_ = false;
  direct_motor_stop_last_failure_message_.clear();
  direct_motor_stop_confirmation_status_ = "idle";
  internal_watchdog_stop_published_.store(false);
  active_stop_event_ = amr_sweeper_safety_msgs::msg::SafetyStop();
  latched_stop_events_.clear();
  publishStatus();
}

void SafetyControllerNode::onPublishTimer()
{
  noteProgress();
  if (enabled_) {
    pollButtonCanFrames();
    pollOdriveCanFrames();
    pollSteadydriveCanFrames();
    checkButtonHeartbeatWatchdog();
  }

  if (enabled_ && latched_stop_active_) {
    publishZeroCommands();
    publishDirectMotorStopCommands();
    checkDirectMotorStopFeedback();
    requestMissionStop();
    requestFsmFaultState();
  }

  publishStatus();
}

void SafetyControllerNode::publishZeroCommands()
{
  geometry_msgs::msg::Twist zero_twist;
  wheel_stop_publisher_->publish(zero_twist);
}

void SafetyControllerNode::publishDirectMotorStopCommands()
{
  std::string failure_message;
  {
    std::lock_guard<std::mutex> lock(can_tx_mutex_);
    direct_motor_stop_healthy_ = sendDirectMotorStopCommands(failure_message);
  }
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

void SafetyControllerNode::pollOdriveCanFrames()
{
  if (!direct_can_motor_stop_enabled_ || !odrive_direct_can_stop_enabled_) {
    return;
  }

  if (!ensureCanInterface(odrive_feedback_can_state_, "ODrive feedback monitor")) {
    return;
  }

  while (true) {
    struct can_frame frame {};
    const ssize_t bytes_read = ::read(odrive_feedback_can_state_.socket_fd, &frame, sizeof(frame));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "ODrive feedback monitor read failed on '%s': %s",
        odrive_feedback_can_state_.interface_name.c_str(),
        std::strerror(errno));
      closeCanInterface(odrive_feedback_can_state_);
      return;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    std::vector<uint8_t> payload(frame.data, frame.data + frame.can_dlc);
    (void)parseOdriveCanFrame(frame.can_id & CAN_SFF_MASK, payload);
  }
}

void SafetyControllerNode::pollSteadydriveCanFrames()
{
  if (!direct_can_motor_stop_enabled_ || !steadydrive_direct_can_stop_enabled_) {
    return;
  }

  if (!ensureCanInterface(steadydrive_feedback_can_state_, "Steadydrive feedback monitor")) {
    return;
  }

  while (true) {
    struct can_frame frame {};
    const ssize_t bytes_read = ::read(steadydrive_feedback_can_state_.socket_fd, &frame, sizeof(frame));
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Steadydrive feedback monitor read failed on '%s': %s",
        steadydrive_feedback_can_state_.interface_name.c_str(),
        std::strerror(errno));
      closeCanInterface(steadydrive_feedback_can_state_);
      return;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    std::vector<uint8_t> payload(frame.data, frame.data + frame.can_dlc);
    (void)parseSteadydriveCanFrame(frame.can_id & CAN_SFF_MASK, payload);
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
    publishInternalStopEvent(stop_msg);
  }
}

void SafetyControllerNode::checkDirectMotorStopFeedback()
{
  if (!direct_can_motor_stop_enabled_) {
    direct_motor_stop_confirmed_ = true;
    direct_motor_stop_confirmation_status_ = "disabled";
    return;
  }

  const auto now_time = now();
  const auto verification_timeout = rclcpp::Duration::from_seconds(
    static_cast<double>(stop_feedback_timeout_ms_) / 1000.0);
  const auto stale_timeout = rclcpp::Duration::from_seconds(
    static_cast<double>(stop_feedback_stale_ms_) / 1000.0);

  std::vector<std::string> pending_reasons;
  std::vector<std::string> failed_reasons;

  for (const auto & feedback_state : odrive_feedback_states_) {
    const auto node_label = "odrive:" + std::to_string(feedback_state.node_id);
    if (!feedback_state.encoder_seen) {
      pending_reasons.push_back(node_label + "=no_encoder_feedback");
      continue;
    }
    if ((now_time - feedback_state.last_encoder_time) > stale_timeout) {
      pending_reasons.push_back(node_label + "=stale_encoder_feedback");
      continue;
    }
    if (std::fabs(feedback_state.velocity_rev_s) > odrive_stop_velocity_threshold_rev_s_) {
      pending_reasons.push_back(
        node_label + "=velocity_rev_s:" + std::to_string(feedback_state.velocity_rev_s));
      continue;
    }
    if (feedback_state.heartbeat_seen &&
      (now_time - feedback_state.last_heartbeat_time) <= stale_timeout &&
      feedback_state.axis_error != 0U)
    {
      failed_reasons.push_back(
        node_label + "=axis_error:" + std::to_string(feedback_state.axis_error));
    }
  }

  for (const auto & feedback_state : steadydrive_feedback_states_) {
    const auto motor_label = "steadydrive:" + formatCanIdHex(feedback_state.can_id);
    if (!feedback_state.state_2_seen) {
      pending_reasons.push_back(motor_label + "=no_state2_feedback");
      continue;
    }
    if ((now_time - feedback_state.last_state_2_time) > stale_timeout) {
      pending_reasons.push_back(motor_label + "=stale_state2_feedback");
      continue;
    }
    if (std::fabs(feedback_state.velocity_deg_s) > steadydrive_stop_velocity_threshold_deg_s_) {
      pending_reasons.push_back(
        motor_label + "=velocity_deg_s:" + std::to_string(feedback_state.velocity_deg_s));
    }
  }

  if (pending_reasons.empty() && failed_reasons.empty()) {
    direct_motor_stop_confirmed_ = true;
    direct_motor_stop_confirmation_timeout_reported_ = false;
    direct_motor_stop_confirmation_status_ = "confirmed";
    return;
  }

  direct_motor_stop_confirmed_ = false;
  std::ostringstream status_stream;
  status_stream << "pending";
  for (const auto & reason : pending_reasons) {
    status_stream << " [" << reason << "]";
  }
  for (const auto & reason : failed_reasons) {
    status_stream << " [" << reason << "]";
  }
  direct_motor_stop_confirmation_status_ = status_stream.str();

  if ((now_time - latched_stop_since_) > verification_timeout &&
    !direct_motor_stop_confirmation_timeout_reported_)
  {
    direct_motor_stop_confirmation_timeout_reported_ = true;
    RCLCPP_ERROR(
      get_logger(),
      "Direct motor stop confirmation timeout after %d ms: %s",
      stop_feedback_timeout_ms_,
      direct_motor_stop_confirmation_status_.c_str());
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
  status.values.push_back(keyValue("direct_motor_stop_confirmed", direct_motor_stop_confirmed_ ? "true" : "false"));
  status.values.push_back(keyValue("direct_motor_stop_confirmation_status", direct_motor_stop_confirmation_status_));
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
  stream << "\"direct_motor_stop_confirmed\":"
         << (direct_motor_stop_confirmed_ ? "true" : "false") << ',';
  stream << "\"direct_motor_stop_confirmation_status\":\""
         << jsonEscape(direct_motor_stop_confirmation_status_) << "\",";
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
        "Zero sweeping-controller stop commands and direct CAN motor stop commands remain active "
        "while waiting for the mission stack to recover.",
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
    "Safety controller requested mission stop via '%s' while continuing to publish zero "
    "sweeping-controller stop commands and direct CAN motor stop commands.",
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
  request->priority = 0;
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

bool SafetyControllerNode::sendCanRemoteFrame(
  CanInterfaceState & can_interface,
  uint32_t can_id,
  const uint8_t payload_length,
  const std::string & description)
{
  if (!ensureCanInterface(can_interface, description)) {
    return false;
  }

  struct can_frame frame {};
  frame.can_id = can_id | CAN_RTR_FLAG;
  frame.can_dlc = payload_length;

  if (::write(can_interface.socket_fd, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    direct_motor_stop_last_failure_message_ =
      description + ": RTR write failed on '" + can_interface.interface_name + "': " +
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

bool SafetyControllerNode::requestOdriveEncoderEstimates()
{
  if (!odrive_direct_can_stop_enabled_ || odrive_node_ids_.empty()) {
    return true;
  }

  for (const auto node_id : odrive_node_ids_) {
    const uint32_t can_id = (node_id << 5) | kOdriveGetEncoderEstimatesCommandId;
    if (!sendCanRemoteFrame(
        odrive_can_state_,
        can_id,
        8U,
        "ODrive encoder estimate request for node " + std::to_string(node_id)))
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

bool SafetyControllerNode::requestSteadydriveState2()
{
  if (!steadydrive_direct_can_stop_enabled_ || steadydrive_motor_can_ids_.empty()) {
    return true;
  }

  const std::vector<uint8_t> request_payload{
    kSteadydriveReadState2Command, 0, 0, 0, 0, 0, 0, 0};
  for (const auto motor_can_id : steadydrive_motor_can_ids_) {
    if (!sendCanFrame(
        steadydrive_can_state_,
        motor_can_id,
        request_payload,
        "Steadydrive state2 request for CAN ID " + formatCanIdHex(motor_can_id)))
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

  if (!sendOdriveEstopCommands() || !sendSteadydriveStopCommands() ||
    !requestOdriveEncoderEstimates() || !requestSteadydriveState2())
  {
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
    publishInternalStopEvent(stop_msg);
    return true;
  }

  if (can_id == status_can_id && payload.size() >= 8) {
    button_last_status_frame_time_ = now();
    button_last_heartbeat_counter_ = static_cast<uint16_t>(
      (static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetMsb]) << 8) |
      static_cast<uint16_t>(payload[kButtonHeartbeatStatusOffsetLsb]));
    button_heartbeat_seen_ = true;
  }

  if (can_id == status_can_id && payload.size() >= 2 && (payload[1] & kButtonPressedBitMask) != 0U) {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = toBuiltinTime(now());
    stop_msg.sender = "button_module_can";
    stop_msg.reason = "physical safety stop button pressed (status frame)";
    publishInternalStopEvent(stop_msg);
    return true;
  }

  return false;
}

bool SafetyControllerNode::parseOdriveCanFrame(
  const uint32_t can_id,
  const std::vector<uint8_t> & payload)
{
  const uint32_t node_id = (can_id >> 5U) & 0x3FU;
  const uint8_t command_id = static_cast<uint8_t>(can_id & 0x1FU);
  const auto state_it = std::find_if(
    odrive_feedback_states_.begin(),
    odrive_feedback_states_.end(),
    [node_id](const OdriveFeedbackState & state) {
      return state.node_id == node_id;
    });
  if (state_it == odrive_feedback_states_.end()) {
    return false;
  }

  if (command_id == kOdriveHeartbeatCommandId && payload.size() >= 5U) {
    state_it->axis_error = decodeUint32LE(payload.data());
    state_it->axis_state = payload[4];
    state_it->heartbeat_seen = true;
    state_it->last_heartbeat_time = now();
    return true;
  }

  if (command_id == kOdriveGetEncoderEstimatesCommandId && payload.size() >= 8U) {
    state_it->velocity_rev_s = static_cast<double>(decodeFloat32LE(payload.data() + 4U));
    state_it->encoder_seen = true;
    state_it->last_encoder_time = now();
    return true;
  }

  return false;
}

bool SafetyControllerNode::parseSteadydriveCanFrame(
  const uint32_t can_id,
  const std::vector<uint8_t> & payload)
{
  const auto state_it = std::find_if(
    steadydrive_feedback_states_.begin(),
    steadydrive_feedback_states_.end(),
    [can_id](const SteadydriveFeedbackState & state) {
      return state.can_id == can_id;
    });
  if (state_it == steadydrive_feedback_states_.end() || payload.size() < 8U) {
    return false;
  }

  if (payload[0] != kSteadydriveReadState2Command || allZeroPayloadExceptCommand(payload)) {
    return false;
  }

  state_it->velocity_deg_s = static_cast<double>(decodeInt16LE(payload[4], payload[5]));
  state_it->state_2_seen = true;
  state_it->last_state_2_time = now();
  return true;
}

void SafetyControllerNode::resetLatchedStopService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  noteProgress();
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
  noteProgress();
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
