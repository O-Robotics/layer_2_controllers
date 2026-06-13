#include "drive_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_sweeper_drive_controller
{

namespace
{

double yawToQuaternionZ(double yaw)
{
  return std::sin(yaw * 0.5);
}

double yawToQuaternionW(double yaw)
{
  return std::cos(yaw * 0.5);
}

}  // namespace

controller_interface::CallbackReturn DriveController::on_init()
{
  auto node = get_node();

  node->declare_parameter("left_wheel_names", left_wheel_names_);
  node->declare_parameter("right_wheel_names", right_wheel_names_);
  node->declare_parameter("input_topic", input_topic_);
  node->declare_parameter("direct_command_topic", direct_command_topic_);
  node->declare_parameter("odom_topic", odom_topic_);
  node->declare_parameter("odom_frame_id", odom_frame_id_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("wheel_separation", wheel_separation_);
  node->declare_parameter("wheel_radius", wheel_radius_);
  node->declare_parameter("left_wheel_radius_multiplier", left_wheel_radius_multiplier_);
  node->declare_parameter("right_wheel_radius_multiplier", right_wheel_radius_multiplier_);
  node->declare_parameter("wheel_separation_multiplier", wheel_separation_multiplier_);
  node->declare_parameter("command_timeout_sec", command_timeout_sec_);
  node->declare_parameter("direct_command_timeout_sec", direct_command_timeout_sec_);
  node->declare_parameter("publish_rate", publish_rate_);
  node->declare_parameter("position_feedback", position_feedback_);
  node->declare_parameter("enable_odom_tf", enable_odom_tf_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    left_wheel_names_.front() + "/" + hardware_interface::HW_IF_VELOCITY,
    right_wheel_names_.front() + "/" + hardware_interface::HW_IF_VELOCITY,
  };
  return config;
}

controller_interface::InterfaceConfiguration DriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    left_wheel_names_.front() + "/" + hardware_interface::HW_IF_POSITION,
    left_wheel_names_.front() + "/" + hardware_interface::HW_IF_VELOCITY,
    right_wheel_names_.front() + "/" + hardware_interface::HW_IF_POSITION,
    right_wheel_names_.front() + "/" + hardware_interface::HW_IF_VELOCITY,
  };
  return config;
}

controller_interface::CallbackReturn DriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  left_wheel_names_ = node->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = node->get_parameter("right_wheel_names").as_string_array();
  input_topic_ = node->get_parameter("input_topic").as_string();
  direct_command_topic_ = node->get_parameter("direct_command_topic").as_string();
  odom_topic_ = node->get_parameter("odom_topic").as_string();
  odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = node->get_parameter("base_frame_id").as_string();
  wheel_separation_ = node->get_parameter("wheel_separation").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  left_wheel_radius_multiplier_ =
    node->get_parameter("left_wheel_radius_multiplier").as_double();
  right_wheel_radius_multiplier_ =
    node->get_parameter("right_wheel_radius_multiplier").as_double();
  wheel_separation_multiplier_ =
    node->get_parameter("wheel_separation_multiplier").as_double();
  command_timeout_sec_ = node->get_parameter("command_timeout_sec").as_double();
  direct_command_timeout_sec_ =
    node->get_parameter("direct_command_timeout_sec").as_double();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  position_feedback_ = node->get_parameter("position_feedback").as_bool();
  enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();

  if (left_wheel_names_.size() != 1 || right_wheel_names_.size() != 1) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Drive controller currently requires exactly one left and one right wheel joint.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (
    wheel_separation_ <= 0.0 || wheel_radius_ <= 0.0 ||
    left_wheel_radius_multiplier_ <= 0.0 || right_wheel_radius_multiplier_ <= 0.0 ||
    wheel_separation_multiplier_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Drive-controller wheel geometry parameters must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (command_timeout_sec_ < 0.0 || direct_command_timeout_sec_ < 0.0 || publish_rate_ <= 0.0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Drive-controller timeouts must be non-negative and publish_rate must be positive.");
    return controller_interface::CallbackReturn::ERROR;
  }

  resetCommandState();
  resetOdometryState();

  command_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
    input_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DriveController::onWheelCommand, this, std::placeholders::_1));
  direct_command_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    direct_command_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DriveController::onDirectCommand, this, std::placeholders::_1));
  odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10));
  if (enable_odom_tf_) {
    tf_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10));
  } else {
    tf_publisher_.reset();
  }
  odom_publish_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&DriveController::publishLatestOdometry, this));
  odom_publish_timer_->cancel();

  RCLCPP_INFO(
    node->get_logger(),
    "Drive controller configured: input='%s', direct='%s', odom='%s', controller='drive_controller'",
    input_topic_.c_str(),
    direct_command_topic_.c_str(),
    odom_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  resetCommandState();
  resetOdometryState();
  if (odom_publish_timer_) {
    odom_publish_timer_->reset();
  }

  for (auto & command_interface : command_interfaces_) {
    const bool success = command_interface.set_value(0.0);
    if (!success) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to clear one drive-controller command interface during activation.");
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (odom_publish_timer_) {
    odom_publish_timer_->cancel();
  }

  for (auto & command_interface : command_interfaces_) {
    const bool success = command_interface.set_value(0.0);
    if (!success) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to clear one drive-controller command interface during deactivation.");
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DriveController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  double linear_command = 0.0;
  double angular_command = 0.0;

  {
    std::scoped_lock lock(command_mutex_);
    if (
      latest_direct_command_.valid &&
      isFresh(time, latest_direct_command_.received_at, direct_command_timeout_sec_))
    {
      linear_command = latest_direct_command_.message.twist.linear.x;
      angular_command = latest_direct_command_.message.twist.angular.z;
    } else if (
      latest_twist_command_.valid &&
      isFresh(time, latest_twist_command_.received_at, command_timeout_sec_))
    {
      linear_command = latest_twist_command_.message.linear.x;
      angular_command = latest_twist_command_.message.angular.z;
    }
  }

  const double effective_wheel_separation =
    wheel_separation_ * wheel_separation_multiplier_;
  const double left_wheel_radius =
    wheel_radius_ * left_wheel_radius_multiplier_;
  const double right_wheel_radius =
    wheel_radius_ * right_wheel_radius_multiplier_;

  const double left_linear_velocity =
    linear_command - (angular_command * effective_wheel_separation * 0.5);
  const double right_linear_velocity =
    linear_command + (angular_command * effective_wheel_separation * 0.5);
  const double left_wheel_velocity = left_linear_velocity / left_wheel_radius;
  const double right_wheel_velocity = right_linear_velocity / right_wheel_radius;

  const bool left_ok = command_interfaces_[0].set_value(left_wheel_velocity);
  const bool right_ok = command_interfaces_[1].set_value(right_wheel_velocity);
  if (!left_ok || !right_ok) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to write one or more drive velocity commands to hardware interfaces.");
  }

  const double left_position_rad = state_interfaces_[0].get_optional().value_or(0.0);
  const double left_velocity_rad_s = state_interfaces_[1].get_optional().value_or(0.0);
  const double right_position_rad = state_interfaces_[2].get_optional().value_or(0.0);
  const double right_velocity_rad_s = state_interfaces_[3].get_optional().value_or(0.0);

  const double dt_seconds = period.seconds();
  if (!odometry_initialized_) {
    previous_left_position_rad_ = left_position_rad;
    previous_right_position_rad_ = right_position_rad;
    odometry_initialized_ = true;
  } else if (position_feedback_) {
    integrateWheelPositions(left_position_rad, right_position_rad, dt_seconds);
  } else {
    integrateWheelVelocities(left_velocity_rad_s, right_velocity_rad_s, dt_seconds);
  }

  {
    std::scoped_lock lock(odometry_mutex_);
    latest_odometry_snapshot_.stamp = time;
    latest_odometry_snapshot_.x = x_;
    latest_odometry_snapshot_.y = y_;
    latest_odometry_snapshot_.heading = heading_;
    latest_odometry_snapshot_.linear_velocity = linear_velocity_;
    latest_odometry_snapshot_.angular_velocity = angular_velocity_;
    latest_odometry_snapshot_.valid = true;
  }

  return controller_interface::return_type::OK;
}

void DriveController::resetCommandState()
{
  std::scoped_lock lock(command_mutex_);
  latest_twist_command_.message = geometry_msgs::msg::Twist{};
  latest_twist_command_.received_at = rclcpp::Time{0, 0, RCL_ROS_TIME};
  latest_twist_command_.valid = false;
  latest_direct_command_.message = geometry_msgs::msg::TwistStamped{};
  latest_direct_command_.received_at = rclcpp::Time{0, 0, RCL_ROS_TIME};
  latest_direct_command_.valid = false;
}

void DriveController::resetOdometryState()
{
  odometry_initialized_ = false;
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  linear_velocity_ = 0.0;
  angular_velocity_ = 0.0;
  previous_left_position_rad_ = 0.0;
  previous_right_position_rad_ = 0.0;

  std::scoped_lock lock(odometry_mutex_);
  latest_odometry_snapshot_ = OdometrySnapshot{};
}

void DriveController::onWheelCommand(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::scoped_lock lock(command_mutex_);
  latest_twist_command_.message = *message;
  latest_twist_command_.received_at = get_node()->now();
  latest_twist_command_.valid = true;
}

void DriveController::onDirectCommand(const geometry_msgs::msg::TwistStamped::SharedPtr message)
{
  if (!message) {
    return;
  }

  std::scoped_lock lock(command_mutex_);
  latest_direct_command_.message = *message;
  latest_direct_command_.received_at = get_node()->now();
  latest_direct_command_.valid = true;
}

bool DriveController::isFresh(
  const rclcpp::Time & now,
  const rclcpp::Time & received_at,
  double timeout_sec) const
{
  if (timeout_sec <= 0.0) {
    return true;
  }
  return (now - received_at).seconds() <= timeout_sec;
}

void DriveController::publishLatestOdometry()
{
  if (!odom_publisher_) {
    return;
  }

  OdometrySnapshot snapshot;
  {
    std::scoped_lock lock(odometry_mutex_);
    snapshot = latest_odometry_snapshot_;
  }

  if (!snapshot.valid) {
    return;
  }

  publishOdometry(snapshot);
  if (enable_odom_tf_) {
    publishOdometryTransform(snapshot);
  }
}

void DriveController::publishOdometry(const OdometrySnapshot & snapshot)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = snapshot.stamp;
  message.header.frame_id = odom_frame_id_;
  message.child_frame_id = base_frame_id_;
  message.pose.pose.position.x = snapshot.x;
  message.pose.pose.position.y = snapshot.y;
  message.pose.pose.orientation.z = yawToQuaternionZ(snapshot.heading);
  message.pose.pose.orientation.w = yawToQuaternionW(snapshot.heading);
  message.twist.twist.linear.x = snapshot.linear_velocity;
  message.twist.twist.angular.z = snapshot.angular_velocity;
  odom_publisher_->publish(message);
}

void DriveController::publishOdometryTransform(const OdometrySnapshot & snapshot)
{
  if (!tf_publisher_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = snapshot.stamp;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = base_frame_id_;
  transform.transform.translation.x = snapshot.x;
  transform.transform.translation.y = snapshot.y;
  transform.transform.rotation.z = yawToQuaternionZ(snapshot.heading);
  transform.transform.rotation.w = yawToQuaternionW(snapshot.heading);

  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(std::move(transform));
  tf_publisher_->publish(message);
}

void DriveController::integrateWheelPositions(
  double left_position_rad,
  double right_position_rad,
  double dt_seconds)
{
  const double left_wheel_radius =
    wheel_radius_ * left_wheel_radius_multiplier_;
  const double right_wheel_radius =
    wheel_radius_ * right_wheel_radius_multiplier_;
  const double effective_wheel_separation =
    wheel_separation_ * wheel_separation_multiplier_;

  const double left_delta =
    (left_position_rad - previous_left_position_rad_) * left_wheel_radius;
  const double right_delta =
    (right_position_rad - previous_right_position_rad_) * right_wheel_radius;

  previous_left_position_rad_ = left_position_rad;
  previous_right_position_rad_ = right_position_rad;

  const double delta_linear = 0.5 * (left_delta + right_delta);
  const double delta_heading =
    (right_delta - left_delta) / effective_wheel_separation;
  const double heading_midpoint = heading_ + (delta_heading * 0.5);

  x_ += delta_linear * std::cos(heading_midpoint);
  y_ += delta_linear * std::sin(heading_midpoint);
  heading_ += delta_heading;

  if (dt_seconds > 0.0) {
    linear_velocity_ = delta_linear / dt_seconds;
    angular_velocity_ = delta_heading / dt_seconds;
  }
}

void DriveController::integrateWheelVelocities(
  double left_velocity_rad_s,
  double right_velocity_rad_s,
  double dt_seconds)
{
  const double left_wheel_radius =
    wheel_radius_ * left_wheel_radius_multiplier_;
  const double right_wheel_radius =
    wheel_radius_ * right_wheel_radius_multiplier_;
  const double effective_wheel_separation =
    wheel_separation_ * wheel_separation_multiplier_;

  const double left_linear_velocity = left_velocity_rad_s * left_wheel_radius;
  const double right_linear_velocity = right_velocity_rad_s * right_wheel_radius;
  linear_velocity_ = 0.5 * (left_linear_velocity + right_linear_velocity);
  angular_velocity_ =
    (right_linear_velocity - left_linear_velocity) / effective_wheel_separation;

  const double delta_heading = angular_velocity_ * dt_seconds;
  const double heading_midpoint = heading_ + (delta_heading * 0.5);
  x_ += linear_velocity_ * dt_seconds * std::cos(heading_midpoint);
  y_ += linear_velocity_ * dt_seconds * std::sin(heading_midpoint);
  heading_ += delta_heading;
}

}  // namespace amr_sweeper_drive_controller

PLUGINLIB_EXPORT_CLASS(
  amr_sweeper_drive_controller::DriveController,
  controller_interface::ControllerInterface)
