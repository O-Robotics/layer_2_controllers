#include "attitude_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_sweeper_attitude_controller
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
  const unsigned char level,
  const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "amr_sweeper_attitude_controller";
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

bool hasValidOrientation(const sensor_msgs::msg::Imu & msg)
{
  if (msg.orientation_covariance[0] < 0.0) {
    return false;
  }

  const double norm =
    (msg.orientation.x * msg.orientation.x) +
    (msg.orientation.y * msg.orientation.y) +
    (msg.orientation.z * msg.orientation.z) +
    (msg.orientation.w * msg.orientation.w);

  return std::isfinite(msg.orientation.x) &&
         std::isfinite(msg.orientation.y) &&
         std::isfinite(msg.orientation.z) &&
         std::isfinite(msg.orientation.w) &&
         norm > 1e-9;
}

void quaternionToRollPitch(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  double * roll_rad,
  double * pitch_rad)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  double yaw_rad = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(*roll_rad, *pitch_rad, yaw_rad);
}

double quantizeAngleRad(const double angle_rad, const double resolution_deg)
{
  const double resolution_rad = degreesToRadians(resolution_deg);
  if (resolution_rad <= 0.0) {
    return angle_rad;
  }

  return std::round(angle_rad / resolution_rad) * resolution_rad;
}

bool weightedAverageOrientation(
  const std::vector<ImuMeasurement> & measurements,
  double * roll_rad,
  double * pitch_rad)
{
  double roll_sin_sum = 0.0;
  double roll_cos_sum = 0.0;
  double pitch_sin_sum = 0.0;
  double pitch_cos_sum = 0.0;
  double total_weight = 0.0;

  for (const auto & measurement : measurements) {
    const double weight = std::max(0.0, measurement.weight);
    if (weight <= 0.0) {
      continue;
    }

    roll_sin_sum += std::sin(measurement.roll_rad) * weight;
    roll_cos_sum += std::cos(measurement.roll_rad) * weight;
    pitch_sin_sum += std::sin(measurement.pitch_rad) * weight;
    pitch_cos_sum += std::cos(measurement.pitch_rad) * weight;
    total_weight += weight;
  }

  if (total_weight <= 0.0) {
    return false;
  }

  *roll_rad = std::atan2(roll_sin_sum, roll_cos_sum);
  *pitch_rad = std::atan2(pitch_sin_sum, pitch_cos_sum);
  return true;
}

void appendReason(
  std::vector<std::string> * reasons, const bool condition,
  const std::string & reason)
{
  if (condition) {
    reasons->push_back(reason);
  }
}

std::string joinReasons(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "ok";
  }

  std::ostringstream stream;
  for (std::size_t index = 0; index < reasons.size(); ++index) {
    if (index > 0) {
      stream << ", ";
    }
    stream << reasons[index];
  }
  return stream.str();
}

}  // namespace

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool isFiniteVector(const geometry_msgs::msg::Vector3 & vector)
{
  return isFinite(vector.x) && isFinite(vector.y) && isFinite(vector.z);
}

bool isZeroTime(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() == 0;
}

ImuHealth checkImuHealth(
  const ImuInput & input,
  const rclcpp::Time & now,
  const ImuHealthConfig & config)
{
  if (!input.has_message) {
    if (!isZeroTime(input.last_received_time) &&
      (now - input.last_received_time).seconds() < config.startup_grace_sec)
    {
      return {false, true, false, "waiting for first message during startup grace"};
    }
    return {false, false, true, "no messages received"};
  }

  if (isZeroTime(input.last_stamp)) {
    return {false, false, true, "invalid timestamp"};
  }

  if (!isFiniteVector(input.last_msg.linear_acceleration)) {
    return {false, false, true, "linear acceleration contains non-finite values"};
  }

  if (!isFiniteVector(input.last_msg.angular_velocity)) {
    return {false, false, true, "angular velocity contains non-finite values"};
  }

  if (input.last_msg.orientation_covariance[0] < 0.0) {
    return {false, false, true, "orientation unavailable"};
  }

  if (!std::isfinite(input.last_msg.orientation.x) ||
    !std::isfinite(input.last_msg.orientation.y) ||
    !std::isfinite(input.last_msg.orientation.z) ||
    !std::isfinite(input.last_msg.orientation.w))
  {
    return {false, false, true, "orientation contains non-finite values"};
  }

  if (!isZeroTime(input.last_received_time)) {
    const double receive_age = (now - input.last_received_time).seconds();
    if (receive_age > config.timeout_error_sec) {
      return {false, false, true, "message timeout error"};
    }
    if (receive_age > config.timeout_warning_sec) {
      return {true, true, false, "message timeout warning"};
    }
  }

  const double stamp_age = (now - input.last_stamp).seconds();
  if (stamp_age > config.timeout_error_sec) {
    return {false, false, true, "stale timestamp error"};
  }

  if (stamp_age > config.timeout_warning_sec) {
    return {true, true, false, "stale timestamp warning"};
  }

  if (stamp_age < -config.timeout_error_sec) {
    return {false, false, true, "timestamp is in the future"};
  }

  return {true, false, false, "ok"};
}

double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians)
{
  return radians * 180.0 / M_PI;
}

StopSupervisor::StopSupervisor(const StopSupervisorOptions & options)
: options_(options)
{
}

void StopSupervisor::setOptions(const StopSupervisorOptions & options)
{
  options_ = options;
}

StopSupervisorState StopSupervisor::update(const AttitudeEstimate & attitude)
{
  StopSupervisorState next;

  const double roll_error =
    attitude.roll_rad - degreesToRadians(options_.nominal_roll_deg);
  const double pitch_error =
    attitude.pitch_rad - degreesToRadians(options_.nominal_pitch_deg);
  const double abs_roll_error = std::abs(roll_error);
  const double abs_pitch_error = std::abs(pitch_error);

  next.roll_warning = abs_roll_error >= degreesToRadians(options_.roll_warning_deg);
  next.pitch_warning = abs_pitch_error >= degreesToRadians(options_.pitch_warning_deg);
  next.roll_stop = abs_roll_error >= degreesToRadians(options_.roll_stop_deg);
  next.pitch_stop = abs_pitch_error >= degreesToRadians(options_.pitch_stop_deg);

  next.warning = next.roll_warning || next.pitch_warning;
  const bool stop_now = next.roll_stop || next.pitch_stop;

  if (options_.require_manual_reset) {
    next.latched = state_.latched || stop_now;
    next.stopped = next.latched;
  } else {
    next.latched = false;
    next.stopped = stop_now;
  }

  std::vector<std::string> reasons;
  if (!next.roll_stop && !next.pitch_stop) {
    if (next.roll_warning && next.pitch_warning) {
      reasons.push_back("roll and pitch warning");
    } else {
      appendReason(&reasons, next.roll_warning, "roll warning");
      appendReason(&reasons, next.pitch_warning, "pitch warning");
    }
  }

  if (next.roll_stop && next.pitch_stop) {
    reasons.push_back("roll and pitch stop");
  } else {
    appendReason(&reasons, next.roll_stop, "roll stop");
    appendReason(&reasons, next.pitch_stop, "pitch stop");
  }

  appendReason(&reasons, next.latched && !stop_now, "manual reset required");
  next.reason = joinReasons(reasons);

  state_ = next;
  return state_;
}

bool StopSupervisor::resetFault()
{
  state_.latched = false;
  state_.stopped = false;
  state_.reason = "reset";
  return true;
}

StopSupervisorState StopSupervisor::state() const
{
  return state_;
}

AttitudeControllerNode::AttitudeControllerNode(const rclcpp::NodeOptions & options)
: Node("attitude_controller_node", options),
  stop_supervisor_(stop_options_),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  startup_time_ = now();
  loadParameters();
  stop_supervisor_.setOptions(stop_options_);

  attitude_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "attitude_controller/roll_pitch", rclcpp::SystemDefaultsQoS());
  attitude_diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "attitude_controller/status", rclcpp::SystemDefaultsQoS());
  base_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    "attitude_controller/joint_states", rclcpp::SystemDefaultsQoS());
  stop_request_publisher_ = create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_, rclcpp::QoS(10).reliable().transient_local());

  reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
    "amr_sweeper_attitude_controller/reset_fault",
    std::bind(
      &AttitudeControllerNode::resetFaultService, this, std::placeholders::_1,
      std::placeholders::_2));
  enable_attitude_service_ = create_service<std_srvs::srv::SetBool>(
    "amr_sweeper_attitude_controller/enable_attitude_estimation",
    std::bind(
      &AttitudeControllerNode::enableAttitudeEstimationService, this,
      std::placeholders::_1, std::placeholders::_2));
  enable_safety_service_ = create_service<std_srvs::srv::SetBool>(
    "amr_sweeper_attitude_controller/enable_safety_stop",
    std::bind(
      &AttitudeControllerNode::enableSafetyStopService, this,
      std::placeholders::_1, std::placeholders::_2));

  configureImuInputs();

  const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&AttitudeControllerNode::onTimer, this));
}

void AttitudeControllerNode::loadParameters()
{
  attitude_estimation_enabled_ = declare_parameter("attitude_estimation_enabled", true);
  safety_stop_enabled_ = declare_parameter("safety_stop_enabled", true);

  base_footprint_frame_ = declare_parameter("base_footprint_frame", "base_footprint");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  tool_link_frame_ = declare_parameter("tool_link_frame", "tool_link");
  base_roll_joint_name_ = declare_parameter("base_roll_joint_name", "base_roll_joint");
  base_pitch_joint_name_ = declare_parameter("base_pitch_joint_name", "base_pitch_joint");
  stop_topic_name_ = declare_parameter("stop_topic_name", "safety_msgs/stop");

  publish_base_link_joint_states_ = declare_parameter("publish_base_link_joint_states", true);
  publish_tool_link_tf_ = declare_parameter("publish_tool_link_tf", false);

  imu_topics_ = declare_parameter<std::vector<std::string>>(
    "imu_topics", std::vector<std::string>{"imu/data_raw"});
  imu_weights_ = declare_parameter<std::vector<double>>(
    "imu_weights", std::vector<double>{1.0});

  imu_timeout_warning_sec_ = declare_parameter("imu_timeout_warning_sec", 0.3);
  imu_timeout_error_sec_ = declare_parameter("imu_timeout_error_sec", 1.0);
  imu_startup_grace_sec_ = declare_parameter("imu_startup_grace_sec", 3.0);
  imu_timeout_stop_enabled_ = declare_parameter("imu_timeout_stop_enabled", true);
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive; using 50.0 Hz");
    publish_rate_hz_ = 50.0;
  }
  if (imu_timeout_warning_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "imu_timeout_warning_sec must be positive; using 0.3 s");
    imu_timeout_warning_sec_ = 0.3;
  }
  if (imu_timeout_error_sec_ < imu_timeout_warning_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "imu_timeout_error_sec must be >= imu_timeout_warning_sec; using %.3f s",
      imu_timeout_warning_sec_);
    imu_timeout_error_sec_ = imu_timeout_warning_sec_;
  }
  if (imu_startup_grace_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "imu_startup_grace_sec must be non-negative; using 0.0 s");
    imu_startup_grace_sec_ = 0.0;
  }

  stop_options_.roll_warning_deg = declare_parameter("stop.roll_warning_deg", 15.0);
  stop_options_.pitch_warning_deg = declare_parameter("stop.pitch_warning_deg", 15.0);
  stop_options_.roll_stop_deg = declare_parameter("stop.roll_stop_deg", 30.0);
  stop_options_.pitch_stop_deg = declare_parameter("stop.pitch_stop_deg", 30.0);
  stop_options_.nominal_roll_deg = declare_parameter("stop.nominal_roll_deg", 0.0);
  stop_options_.nominal_pitch_deg = declare_parameter("stop.nominal_pitch_deg", 5.0);
  stop_options_.require_manual_reset = declare_parameter("stop.require_manual_reset", true);
}

void AttitudeControllerNode::configureImuInputs()
{
  if (imu_topics_.empty()) {
    RCLCPP_WARN(get_logger(), "imu_topics is empty; using imu/data_raw");
    imu_topics_.push_back("imu/data_raw");
  }

  if (imu_weights_.size() != imu_topics_.size()) {
    RCLCPP_WARN(
      get_logger(),
      "imu_weights length (%zu) does not match imu_topics length (%zu); missing weights use 1.0",
      imu_weights_.size(), imu_topics_.size());
  }

  imu_inputs_.clear();
  imu_subscriptions_.clear();
  imu_inputs_.reserve(imu_topics_.size());
  imu_subscriptions_.reserve(imu_topics_.size());

  for (std::size_t index = 0; index < imu_topics_.size(); ++index) {
    ImuInput input;
    input.topic = imu_topics_[index];
    input.weight = index < imu_weights_.size() ? imu_weights_[index] : 1.0;
    input.last_received_time = startup_time_;
    if (input.weight <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "IMU weight for '%s' must be positive; using 1.0", input.topic.c_str());
      input.weight = 1.0;
    }
    imu_inputs_.push_back(input);

    imu_subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::Imu>(
        imu_topics_[index],
        rclcpp::SensorDataQoS(),
        [this, index](sensor_msgs::msg::Imu::SharedPtr msg) {
          onImuMessage(std::move(msg), index);
        }));
  }
}

void AttitudeControllerNode::onImuMessage(sensor_msgs::msg::Imu::SharedPtr msg, std::size_t index)
{
  if (index >= imu_inputs_.size()) {
    return;
  }

  auto & input = imu_inputs_[index];
  input.last_msg = *msg;
  input.last_stamp = rclcpp::Time(msg->header.stamp);
  input.last_received_time = now();
  input.has_message = true;
}

bool AttitudeControllerNode::transformMeasurementToBaseLink(
  const sensor_msgs::msg::Imu & msg,
  ImuMeasurement * measurement,
  std::string * error_message)
{
  if (!hasValidOrientation(msg)) {
    if (error_message != nullptr) {
      *error_message = "orientation unavailable";
    }
    return false;
  }

  quaternionToRollPitch(msg.orientation, &measurement->roll_rad, &measurement->pitch_rad);

  if (msg.header.frame_id.empty() || msg.header.frame_id == base_link_frame_) {
    return true;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
      base_link_frame_, msg.header.frame_id, tf2::TimePointZero);

    tf2::Quaternion q_world_imu;
    tf2::fromMsg(msg.orientation, q_world_imu);

    tf2::Quaternion q_base_imu;
    tf2::fromMsg(transform.transform.rotation, q_base_imu);

    const tf2::Quaternion q_imu_base = q_base_imu.inverse();
    tf2::Quaternion q_world_base = q_world_imu * q_imu_base;
    q_world_base.normalize();

    quaternionToRollPitch(
      tf2::toMsg(q_world_base), &measurement->roll_rad, &measurement->pitch_rad);
    return true;
  } catch (const tf2::TransformException & exception) {
    if (error_message != nullptr) {
      *error_message = exception.what();
    }
    return false;
  }
}

void AttitudeControllerNode::onTimer()
{
  const auto stamp = now();
  const ImuHealthConfig health_config{
    imu_timeout_warning_sec_,
    imu_timeout_error_sec_,
    imu_startup_grace_sec_};
  std::vector<ImuMeasurement> measurements;
  std::size_t healthy_imu_count = 0;
  bool imu_timeout_error_active = false;
  std::string imu_timeout_error_reason;

  for (auto & input : imu_inputs_) {
    const auto health = checkImuHealth(input, stamp, health_config);
    input.healthy = health.healthy;
    input.timeout_warning = health.timeout_warning;
    input.timeout_error = health.timeout_error;
    input.health_reason = health.reason;
    if (health.timeout_error && imu_timeout_error_reason.empty()) {
      imu_timeout_error_active = true;
      imu_timeout_error_reason = input.topic + ": " + health.reason;
    }

    if (!input.healthy) {
      continue;
    }

    ImuMeasurement measurement;
    measurement.weight = input.weight;
    std::string transform_error;
    if (!transformMeasurementToBaseLink(input.last_msg, &measurement, &transform_error)) {
      input.healthy = false;
      input.health_reason = "TF transform failed: " + transform_error;
      continue;
    }

    measurements.push_back(measurement);
    ++healthy_imu_count;
  }

  if (attitude_estimation_enabled_) {
    last_estimate_ = AttitudeEstimate();
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    if (weightedAverageOrientation(measurements, &roll_rad, &pitch_rad)) {
      last_estimate_.roll_rad = quantizeAngleRad(roll_rad, 0.1);
      last_estimate_.pitch_rad = quantizeAngleRad(pitch_rad, 0.1);
      last_estimate_.healthy = true;
    }
    publishAttitude(stamp, last_estimate_);
    publishAttitudeDiagnostics(stamp, last_estimate_, healthy_imu_count);

    if (publish_base_link_joint_states_) {
      publishBaseLinkJointStates(stamp, last_estimate_);
    }

    if (publish_tool_link_tf_ && !tool_link_warning_logged_) {
      RCLCPP_WARN(
        get_logger(),
        "publish_tool_link_tf is configured, but tool_link roll TF is reserved for a future version");
      tool_link_warning_logged_ = true;
    }
  } else {
    AttitudeEstimate disabled_estimate;
    disabled_estimate.healthy = false;
    publishAttitudeDiagnostics(stamp, disabled_estimate, healthy_imu_count);
  }

  StopSupervisorState safety_state;
  if (safety_stop_enabled_ && imu_timeout_stop_enabled_ && imu_timeout_error_active) {
    safety_state.stopped = true;
    safety_state.latched = true;
    safety_state.reason = "imu timeout error, " + imu_timeout_error_reason;
  } else if (safety_stop_enabled_ && last_estimate_.healthy) {
    safety_state = stop_supervisor_.update(last_estimate_);
  } else {
    safety_state.reason = safety_stop_enabled_ ? "waiting for healthy attitude" : "disabled";
  }

  publishSafety(stamp, safety_state);
}

void AttitudeControllerNode::publishAttitude(
  const rclcpp::Time & stamp,
  const AttitudeEstimate & estimate)
{
  if (!estimate.healthy) {
    return;
  }

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = base_link_frame_;
  msg.vector.x = estimate.roll_rad;
  msg.vector.y = estimate.pitch_rad;
  msg.vector.z = 0.0;
  attitude_publisher_->publish(msg);
}

void AttitudeControllerNode::publishAttitudeDiagnostics(
  const rclcpp::Time & stamp,
  const AttitudeEstimate & estimate,
  std::size_t healthy_imu_count)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  bool timeout_warning_active = false;
  bool timeout_error_active = false;
  std::string timeout_reason;
  for (const auto & input : imu_inputs_) {
    if (input.timeout_error && timeout_reason.empty()) {
      timeout_error_active = true;
      timeout_reason = input.topic + ": " + input.health_reason;
    } else if (input.timeout_warning && timeout_reason.empty()) {
      timeout_warning_active = true;
      timeout_reason = input.topic + ": " + input.health_reason;
    }
  }

  unsigned char level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string attitude_message = "ok";
  if (!attitude_estimation_enabled_) {
    attitude_message = "disabled";
  } else if (timeout_error_active) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    attitude_message = "imu timeout error, " + timeout_reason;
  } else if (timeout_warning_active) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    attitude_message = "imu timeout warning, " + timeout_reason;
  } else if (!estimate.healthy) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    attitude_message = "no healthy orientation";
  }

  auto attitude_status = makeStatus(
    "attitude_estimator",
    level,
    attitude_message);
  attitude_status.values.push_back(
    keyValue(
      "enabled",
      attitude_estimation_enabled_ ? "true" : "false"));
  attitude_status.values.push_back(
    keyValue(
      "healthy_imu_count",
      static_cast<double>(healthy_imu_count)));
  attitude_status.values.push_back(keyValue("roll_rad", estimate.roll_rad));
  attitude_status.values.push_back(keyValue("pitch_rad", estimate.pitch_rad));
  attitude_status.values.push_back(keyValue("roll_deg", radiansToDegrees(estimate.roll_rad)));
  attitude_status.values.push_back(keyValue("pitch_deg", radiansToDegrees(estimate.pitch_rad)));
  array.status.push_back(attitude_status);

  for (const auto & input : imu_inputs_) {
    unsigned char imu_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (!input.timeout_warning && !input.timeout_error && !input.healthy) {
      imu_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    auto imu_status = makeStatus(
      "imu_input: " + input.topic,
      imu_level,
      input.health_reason);
    imu_status.values.push_back(keyValue("weight", input.weight));
    imu_status.values.push_back(keyValue("topic", input.topic));
    array.status.push_back(imu_status);
  }

  attitude_diagnostics_publisher_->publish(array);
}

void AttitudeControllerNode::publishBaseLinkJointStates(
  const rclcpp::Time & stamp,
  const AttitudeEstimate & estimate)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = {base_roll_joint_name_, base_pitch_joint_name_};
  msg.position = {estimate.roll_rad, estimate.pitch_rad};
  base_joint_state_publisher_->publish(msg);
}

void AttitudeControllerNode::publishSafety(
  const rclcpp::Time & stamp,
  const StopSupervisorState & state)
{
  const bool stop_active = safety_stop_enabled_ && state.stopped;
  const bool should_publish_stop_request =
    stop_active && (!last_stop_request_active_ || last_stop_reason_ != state.reason);

  if (should_publish_stop_request) {
    std::ostringstream reason_stream;
    reason_stream << std::fixed << std::setprecision(1)
                  << state.reason
                  << ", roll=" << radiansToDegrees(last_estimate_.roll_rad) << " deg"
                  << ", pitch=" << radiansToDegrees(last_estimate_.pitch_rad) << " deg";

    amr_sweeper_safety_msgs::msg::SafetyStop stop_request_msg;
    stop_request_msg.stamp = toBuiltinTime(stamp);
    stop_request_msg.sender = "amr_sweeper_attitude_controller";
    stop_request_msg.reason = reason_stream.str();
    stop_request_publisher_->publish(stop_request_msg);
  }

  last_stop_request_active_ = stop_active;
  last_stop_reason_ = stop_active ? state.reason : "";
}

void AttitudeControllerNode::resetFaultService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = stop_supervisor_.resetFault();
  response->message = response->success ? "fault reset" : "fault reset failed";
}

void AttitudeControllerNode::enableAttitudeEstimationService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  attitude_estimation_enabled_ = request->data;
  if (!attitude_estimation_enabled_) {
    last_estimate_ = AttitudeEstimate();
  }

  response->success = true;
  response->message = attitude_estimation_enabled_ ?
    "attitude estimation enabled" : "attitude estimation disabled";
}

void AttitudeControllerNode::enableSafetyStopService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  safety_stop_enabled_ = request->data;
  if (!safety_stop_enabled_) {
    stop_supervisor_.resetFault();
  }

  response->success = true;
  response->message = safety_stop_enabled_ ? "safety stop enabled" : "safety stop disabled";
}

}  // namespace amr_sweeper_attitude_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_sweeper_attitude_controller::AttitudeControllerNode>());
  rclcpp::shutdown();
  return 0;
}
