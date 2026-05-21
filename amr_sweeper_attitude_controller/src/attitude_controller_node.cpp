#include "attitude_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
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

}  // namespace

AttitudeControllerNode::AttitudeControllerNode(const rclcpp::NodeOptions & options)
: Node("amr_sweeper_attitude_controller_node", options),
  estimator_(estimator_options_),
  stop_supervisor_(stop_options_),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  loadParameters();
  estimator_.setOptions(estimator_options_);
  stop_supervisor_.setOptions(stop_options_);

  attitude_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "attitude/roll_pitch", rclcpp::SystemDefaultsQoS());
  attitude_diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "attitude/status", rclcpp::SystemDefaultsQoS());
  base_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS());
  stop_request_publisher_ = create_publisher<amr_sweeper_safety_msgs::msg::SafetyStop>(
    stop_topic_name_, rclcpp::SystemDefaultsQoS());

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

  estimator_options_.filter_type = declare_parameter("filter.type", "complementary");
  estimator_options_.accel_alpha = declare_parameter("filter.accel_alpha", 0.02);
  estimator_options_.gyro_alpha = declare_parameter("filter.gyro_alpha", 0.98);
  estimator_options_.max_accel_norm_error =
    declare_parameter("filter.max_accel_norm_error", 2.0);

  stop_options_.roll_warning_deg = declare_parameter("stop.roll_warning_deg", 15.0);
  stop_options_.pitch_warning_deg = declare_parameter("stop.pitch_warning_deg", 15.0);
  stop_options_.roll_stop_deg = declare_parameter("stop.roll_stop_deg", 30.0);
  stop_options_.pitch_stop_deg = declare_parameter("stop.pitch_stop_deg", 30.0);
  stop_options_.nominal_roll_deg = declare_parameter("stop.nominal_roll_deg", 0.0);
  stop_options_.nominal_pitch_deg = declare_parameter("stop.nominal_pitch_deg", 5.0);
  stop_options_.require_manual_reset = declare_parameter("stop.require_manual_reset", true);

  if (estimator_options_.filter_type != "complementary") {
    RCLCPP_WARN(
      get_logger(),
      "Only complementary filter is implemented; requested filter.type='%s'",
      estimator_options_.filter_type.c_str());
  }
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
  measurement->linear_acceleration = msg.linear_acceleration;
  measurement->angular_velocity = msg.angular_velocity;

  if (msg.header.frame_id.empty() || msg.header.frame_id == base_link_frame_) {
    return true;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
      base_link_frame_, msg.header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped accel_in;
    accel_in.header = msg.header;
    accel_in.vector = msg.linear_acceleration;
    geometry_msgs::msg::Vector3Stamped accel_out;
    tf2::doTransform(accel_in, accel_out, transform);
    measurement->linear_acceleration = accel_out.vector;

    geometry_msgs::msg::Vector3Stamped gyro_in;
    gyro_in.header = msg.header;
    gyro_in.vector = msg.angular_velocity;
    geometry_msgs::msg::Vector3Stamped gyro_out;
    tf2::doTransform(gyro_in, gyro_out, transform);
    measurement->angular_velocity = gyro_out.vector;
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
    imu_timeout_warning_sec_, imu_timeout_error_sec_, estimator_options_.max_accel_norm_error};
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
    last_estimate_ = estimator_.update(measurements, stamp);
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
  std::string estimator_message = "ok";
  if (!attitude_estimation_enabled_) {
    estimator_message = "disabled";
  } else if (timeout_error_active) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    estimator_message = "imu timeout error, " + timeout_reason;
  } else if (timeout_warning_active) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    estimator_message = "imu timeout warning, " + timeout_reason;
  } else if (!estimate.healthy) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    estimator_message = "no healthy estimate";
  }

  auto estimator_status = makeStatus(
    "attitude_estimator",
    level,
    estimator_message);
  estimator_status.values.push_back(
    keyValue(
      "enabled",
      attitude_estimation_enabled_ ? "true" : "false"));
  estimator_status.values.push_back(
    keyValue(
      "healthy_imu_count",
      static_cast<double>(healthy_imu_count)));
  estimator_status.values.push_back(keyValue("roll_rad", estimate.roll_rad));
  estimator_status.values.push_back(keyValue("pitch_rad", estimate.pitch_rad));
  estimator_status.values.push_back(keyValue("roll_deg", radiansToDegrees(estimate.roll_rad)));
  estimator_status.values.push_back(keyValue("pitch_deg", radiansToDegrees(estimate.pitch_rad)));
  array.status.push_back(estimator_status);

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
    estimator_.reset();
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
