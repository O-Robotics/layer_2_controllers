#include "collision_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace amr_sweeper_collision_detector
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

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const double value)
{
  std::ostringstream stream;
  stream << value;
  return keyValue(key, stream.str());
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
  const std::string & name,
  unsigned char level,
  const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "amr_sweeper_collision_detector";
  status.level = level;
  status.message = message;
  return status;
}

std::string boolString(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

bool isFiniteVector3(const geometry_msgs::msg::Vector3 & vector)
{
  return std::isfinite(vector.x) &&
         std::isfinite(vector.y) &&
         std::isfinite(vector.z);
}

double magnitude(const geometry_msgs::msg::Vector3 & vector)
{
  return std::sqrt(
    (vector.x * vector.x) +
    (vector.y * vector.y) +
    (vector.z * vector.z));
}

bool isZeroTime(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() == 0;
}

SourceHealth evaluateImuHealth(
  const ImuInput & input,
  const rclcpp::Time & now,
  double timeout_warning_sec,
  double timeout_error_sec)
{
  if (!input.has_message) {
    return {false, true, false, "no messages received yet"};
  }

  if (!isFiniteVector3(input.last_msg.linear_acceleration)) {
    return {false, false, true, "non-finite acceleration values"};
  }

  const double receive_age = (now - input.last_received_time).seconds();
  if (receive_age > timeout_error_sec) {
    return {false, false, true, "message timeout error"};
  }
  if (receive_age > timeout_warning_sec) {
    return {true, true, false, "message timeout warning"};
  }

  if (!isZeroTime(input.last_stamp)) {
    const double stamp_age = (now - input.last_stamp).seconds();
    if (stamp_age > timeout_error_sec) {
      return {false, false, true, "stale timestamp error"};
    }
    if (stamp_age > timeout_warning_sec) {
      return {true, true, false, "stale timestamp warning"};
    }
  }

  return {true, false, false, "ok"};
}

SourceHealth evaluateMotorForceHealth(
  const MotorForceInput & input,
  const rclcpp::Time & now,
  double timeout_warning_sec,
  double timeout_error_sec)
{
  if (!input.has_message) {
    return {false, true, false, "no messages received yet"};
  }

  if (!std::isfinite(input.last_effort)) {
    return {false, false, true, "non-finite effort value"};
  }

  const double receive_age = (now - input.last_received_time).seconds();
  if (receive_age > timeout_error_sec) {
    return {false, false, true, "message timeout error"};
  }
  if (receive_age > timeout_warning_sec) {
    return {true, true, false, "message timeout warning"};
  }

  return {true, false, false, "ok"};
}

CollisionDetectorNode::CollisionDetectorNode(const rclcpp::NodeOptions & options)
: Node("collision_detector_node", options)
{
  loadParameters();
  configureImuInputs();
  configureMotorForceInputs();
  last_imu_health_.resize(imu_inputs_.size());
  last_motor_health_.resize(motor_force_inputs_.size());

  impact_state_publisher_ = create_publisher<std_msgs::msg::Bool>(
    impact_state_topic_, rclcpp::SystemDefaultsQoS());
  stop_request_publisher_ = create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_, rclcpp::QoS(10).reliable().transient_local());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "collision_detector/status", rclcpp::SystemDefaultsQoS());

  reset_impact_latch_service_ = create_service<std_srvs::srv::Trigger>(
    "amr_sweeper_collision_detector/reset_impact_latch",
    std::bind(
      &CollisionDetectorNode::resetImpactLatchService, this, std::placeholders::_1,
      std::placeholders::_2));
  enable_detector_service_ = create_service<std_srvs::srv::SetBool>(
    "amr_sweeper_collision_detector/enable_detector",
    std::bind(
      &CollisionDetectorNode::enableDetectorService, this, std::placeholders::_1,
      std::placeholders::_2));

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&CollisionDetectorNode::onTimer, this));

  const auto diagnostics_period = std::chrono::duration<double>(1.0 / status_publish_rate_hz_);
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(diagnostics_period),
    std::bind(&CollisionDetectorNode::publishLatestDiagnostics, this));

  RCLCPP_INFO(
    get_logger(),
    "Collision detector configured with %zu enabled IMU inputs and %zu enabled motor-force inputs",
    imu_inputs_.size(),
    motor_force_inputs_.size());
}

void CollisionDetectorNode::loadParameters()
{
  enabled_ = declare_parameter("enabled", true);
  latch_collision_ = declare_parameter("latch_collision", true);
  publish_stop_request_ = declare_parameter("publish_stop_request", true);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive; using 20.0 Hz");
    publish_rate_hz_ = 20.0;
  }
  status_publish_rate_hz_ = declare_parameter("status_publish_rate_hz", 2.0);
  if (status_publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "status_publish_rate_hz must be positive; using 2.0 Hz");
    status_publish_rate_hz_ = 2.0;
  }

  impact_state_topic_ = declare_parameter(
    "impact_state_topic", std::string("collision_detector/impact_detected"));
  stop_topic_name_ = declare_parameter("stop_topic_name", std::string("safety_msgs/stop"));
  stop_sender_name_ = declare_parameter("stop_sender_name", std::string("collision_detector"));

  imu_timeout_warning_sec_ = declare_parameter("imu_timeout_warning_sec", 0.3);
  imu_timeout_error_sec_ = declare_parameter("imu_timeout_error_sec", 1.0);
  motor_timeout_warning_sec_ = declare_parameter("motor_timeout_warning_sec", 0.5);
  motor_timeout_error_sec_ = declare_parameter("motor_timeout_error_sec", 1.5);

  configured_imu_names_ = declare_parameter<std::vector<std::string>>(
    "imu_inputs.configured", std::vector<std::string>{"main_imu"});

  configured_motor_force_names_ = declare_parameter<std::vector<std::string>>(
    "motor_force_inputs.configured", std::vector<std::string>{});
}

void CollisionDetectorNode::configureImuInputs()
{
  for (const auto & imu_name : configured_imu_names_) {
    const bool enabled = declare_parameter("imu_inputs." + imu_name + ".enabled", false);
    if (!enabled) {
      continue;
    }

    ImuInput input;
    input.name = imu_name;
    input.topic = declare_parameter(
      "imu_inputs." + imu_name + ".topic", std::string("imu/data_raw"));
    input.acceleration_threshold_mps2 = declare_parameter(
      "imu_inputs." + imu_name + ".acceleration_threshold_mps2", 14.0);
    imu_inputs_.push_back(input);

    imu_subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::Imu>(
        input.topic,
        rclcpp::SensorDataQoS(),
        [this, index = imu_inputs_.size() - 1](sensor_msgs::msg::Imu::SharedPtr msg) {
          onImuMessage(std::move(msg), index);
        }));
  }
}

void CollisionDetectorNode::configureMotorForceInputs()
{
  for (const auto & motor_name : configured_motor_force_names_) {
    const bool enabled = declare_parameter("motor_force_inputs." + motor_name + ".enabled", false);
    if (!enabled) {
      continue;
    }

    MotorForceInput input;
    input.name = motor_name;
    input.topic = declare_parameter(
      "motor_force_inputs." + motor_name + ".topic", std::string("attitude_controller/joint_states"));
    input.joint_name = declare_parameter(
      "motor_force_inputs." + motor_name + ".joint_name", std::string(""));
    input.effort_threshold = declare_parameter(
      "motor_force_inputs." + motor_name + ".effort_threshold", 5.0);
    motor_force_inputs_.push_back(input);

    motor_force_subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::JointState>(
        input.topic,
        rclcpp::SystemDefaultsQoS(),
        [this, index = motor_force_inputs_.size() - 1](sensor_msgs::msg::JointState::SharedPtr msg) {
          onMotorForceMessage(std::move(msg), index);
        }));
  }
}

void CollisionDetectorNode::onImuMessage(sensor_msgs::msg::Imu::SharedPtr msg, std::size_t index)
{
  if (!msg || index >= imu_inputs_.size()) {
    return;
  }

  auto & input = imu_inputs_[index];
  input.last_msg = *msg;
  input.last_stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp);
  input.last_received_time = now();
  input.has_message = true;
}

void CollisionDetectorNode::onMotorForceMessage(
  sensor_msgs::msg::JointState::SharedPtr msg,
  std::size_t index)
{
  if (!msg || index >= motor_force_inputs_.size()) {
    return;
  }

  auto & input = motor_force_inputs_[index];
  const auto it = std::find(msg->name.begin(), msg->name.end(), input.joint_name);
  if (it == msg->name.end()) {
    return;
  }

  const auto effort_index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
  if (effort_index >= msg->effort.size()) {
    return;
  }

  input.last_effort = std::fabs(msg->effort[effort_index]);
  input.last_stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    now() : rclcpp::Time(msg->header.stamp);
  input.last_received_time = now();
  input.has_message = true;
}

void CollisionDetectorNode::onTimer()
{
  const auto stamp = now();
  std::vector<SourceHealth> imu_health;
  std::vector<SourceHealth> motor_health;
  imu_health.reserve(imu_inputs_.size());
  motor_health.reserve(motor_force_inputs_.size());

  std::size_t healthy_imu_count = 0;
  bool impact_detected = false;
  std::string impact_reason = "no impact detected";

  for (const auto & input : imu_inputs_) {
    const auto health = evaluateImuHealth(
      input, stamp, imu_timeout_warning_sec_, imu_timeout_error_sec_);
    imu_health.push_back(health);

    if (health.healthy) {
      ++healthy_imu_count;
      const double acceleration_magnitude = magnitude(input.last_msg.linear_acceleration);
      if (acceleration_magnitude >= input.acceleration_threshold_mps2 && !impact_detected) {
        impact_detected = true;
        std::ostringstream stream;
        stream << input.name
               << " acceleration magnitude "
               << acceleration_magnitude
               << " m/s^2 exceeded threshold "
               << input.acceleration_threshold_mps2
               << " m/s^2";
        impact_reason = stream.str();
      }
    }
  }

  for (const auto & input : motor_force_inputs_) {
    const auto health = evaluateMotorForceHealth(
      input, stamp, motor_timeout_warning_sec_, motor_timeout_error_sec_);
    motor_health.push_back(health);

    if (health.healthy && input.last_effort >= input.effort_threshold && !impact_detected) {
      impact_detected = true;
      std::ostringstream stream;
      stream << input.name
             << " effort "
             << input.last_effort
             << " exceeded threshold "
             << input.effort_threshold;
      impact_reason = stream.str();
    }
  }

  if (!enabled_) {
    impact_detected = false;
    impact_reason = "detector disabled";
  } else if (healthy_imu_count == 0 && !imu_inputs_.empty() && !impact_detected) {
    impact_reason = "waiting for at least one healthy IMU";
  }

  if (enabled_ && impact_detected) {
    impact_latched_ = latch_collision_ ? true : impact_detected;
    latched_impact_reason_ = impact_reason;
  } else if (!latch_collision_) {
    impact_latched_ = false;
    latched_impact_reason_ = impact_reason;
  }

  const bool reported_impact = impact_latched_ || (!latch_collision_ && impact_detected);
  const std::string reported_reason = reported_impact ? latched_impact_reason_ : impact_reason;
  publishImpactState(reported_impact, reported_reason);

  // Cache for the independent, lower-rate diagnostics timer; impact_state stays on this
  // full-rate detection loop since it is the real-time safety signal.
  last_imu_health_ = imu_health;
  last_motor_health_ = motor_health;
  last_healthy_imu_count_ = healthy_imu_count;
  last_reported_impact_ = reported_impact;
  last_reported_reason_ = reported_reason;
}

void CollisionDetectorNode::publishLatestDiagnostics()
{
  publishDiagnostics(
    now(),
    last_imu_health_,
    last_motor_health_,
    last_healthy_imu_count_,
    last_reported_impact_,
    last_reported_reason_);
}

void CollisionDetectorNode::publishDiagnostics(
  const rclcpp::Time & stamp,
  const std::vector<SourceHealth> & imu_health,
  const std::vector<SourceHealth> & motor_health,
  std::size_t healthy_imu_count,
  bool impact_detected,
  const std::string & impact_reason)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  unsigned char detector_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string detector_message = "ready";

  if (!enabled_) {
    detector_message = "disabled";
  } else if (impact_detected) {
    detector_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    detector_message = impact_reason;
  } else if (!imu_inputs_.empty() && healthy_imu_count == 0) {
    detector_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    detector_message = "no healthy IMU inputs available";
  } else if (healthy_imu_count < imu_inputs_.size()) {
    detector_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    detector_message = "degraded; running with partial IMU availability";
  } else if (healthy_imu_count > 0 && healthy_imu_count == imu_inputs_.size()) {
    detector_message = "ready";
  }

  auto detector_status = makeStatus("collision_detector", detector_level, detector_message);
  detector_status.values.push_back(keyValue("enabled", boolString(enabled_)));
  detector_status.values.push_back(keyValue("impact_detected", boolString(impact_detected)));
  detector_status.values.push_back(keyValue("impact_reason", impact_reason));
  detector_status.values.push_back(keyValue("healthy_imu_count", static_cast<double>(healthy_imu_count)));
  detector_status.values.push_back(keyValue("configured_imu_count", static_cast<double>(imu_inputs_.size())));
  detector_status.values.push_back(
    keyValue("configured_motor_force_count", static_cast<double>(motor_force_inputs_.size())));
  array.status.push_back(detector_status);

  for (std::size_t index = 0; index < imu_inputs_.size(); ++index) {
    const auto & input = imu_inputs_[index];
    const auto & health = imu_health[index];
    unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (health.error) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (health.warning || !health.healthy) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    auto status = makeStatus("collision_detector/imu/" + input.name, level, health.reason);
    const double acceleration_magnitude = input.has_message ?
      magnitude(input.last_msg.linear_acceleration) : 0.0;
    status.values.push_back(keyValue("topic", input.topic));
    status.values.push_back(keyValue("acceleration_threshold_mps2", input.acceleration_threshold_mps2));
    status.values.push_back(keyValue("acceleration_magnitude_mps2", acceleration_magnitude));
    status.values.push_back(keyValue("healthy", boolString(health.healthy)));
    array.status.push_back(status);
  }

  for (std::size_t index = 0; index < motor_force_inputs_.size(); ++index) {
    const auto & input = motor_force_inputs_[index];
    const auto & health = motor_health[index];
    unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (health.error) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (health.warning || !health.healthy) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    auto status = makeStatus("collision_detector/motor_force/" + input.name, level, health.reason);
    status.values.push_back(keyValue("topic", input.topic));
    status.values.push_back(keyValue("joint_name", input.joint_name));
    status.values.push_back(keyValue("effort_threshold", input.effort_threshold));
    status.values.push_back(keyValue("latest_effort", input.last_effort));
    status.values.push_back(keyValue("healthy", boolString(health.healthy)));
    array.status.push_back(status);
  }

  diagnostics_publisher_->publish(array);
}

void CollisionDetectorNode::publishImpactState(bool impact_detected, const std::string & impact_reason)
{
  std_msgs::msg::Bool state_msg;
  state_msg.data = impact_detected;
  impact_state_publisher_->publish(state_msg);

  if (impact_detected && publish_stop_request_ && !last_reported_impact_state_) {
    amr_sweeper_safety_msgs::msg::SafetyStop stop_msg;
    stop_msg.stamp = now();
    stop_msg.sender = stop_sender_name_;
    stop_msg.reason = impact_reason;
    stop_request_publisher_->publish(stop_msg);

    RCLCPP_ERROR(get_logger(), "Collision detected: %s", impact_reason.c_str());
  }

  if (!impact_detected && last_reported_impact_state_) {
    RCLCPP_INFO(get_logger(), "Collision detector impact state cleared");
  }

  last_reported_impact_state_ = impact_detected;
}

void CollisionDetectorNode::resetImpactLatchService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  impact_latched_ = false;
  latched_impact_reason_ = "impact latch reset";
  response->success = true;
  response->message = "Impact latch reset";
}

void CollisionDetectorNode::enableDetectorService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  if (!enabled_) {
    impact_latched_ = false;
    latched_impact_reason_ = "detector disabled";
  }

  response->success = true;
  response->message = enabled_ ? "Collision detector enabled" : "Collision detector disabled";
}

}  // namespace amr_sweeper_collision_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_collision_detector::CollisionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
