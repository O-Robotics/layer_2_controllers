#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__IMU_INPUT_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__IMU_INPUT_HPP_

#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace amr_sweeper_attitude_controller
{

constexpr double kGravityMetersPerSecondSquared = 9.80665;

struct ImuInput
{
  std::string topic;
  double weight{1.0};
  sensor_msgs::msg::Imu last_msg;
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_time{0, 0, RCL_ROS_TIME};
  bool has_message{false};
  bool healthy{false};
  std::string health_reason{"no messages received"};
};

struct ImuHealthConfig
{
  double timeout_sec{0.25};
  double max_accel_norm_error{2.0};
};

struct ImuHealth
{
  bool healthy{false};
  std::string reason;
};

inline bool isFinite(double value)
{
  return std::isfinite(value);
}

inline bool isFiniteVector(const geometry_msgs::msg::Vector3 & vector)
{
  return isFinite(vector.x) && isFinite(vector.y) && isFinite(vector.z);
}

inline bool isZeroTime(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() == 0;
}

inline double vectorNorm(const geometry_msgs::msg::Vector3 & vector)
{
  return std::sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z));
}

inline ImuHealth checkImuHealth(
  const ImuInput & input,
  const rclcpp::Time & now,
  const ImuHealthConfig & config)
{
  if (!input.has_message) {
    return {false, "no messages received"};
  }

  if (isZeroTime(input.last_stamp)) {
    return {false, "invalid timestamp"};
  }

  if (!isFiniteVector(input.last_msg.linear_acceleration)) {
    return {false, "linear acceleration contains non-finite values"};
  }

  if (!isFiniteVector(input.last_msg.angular_velocity)) {
    return {false, "angular velocity contains non-finite values"};
  }

  const double accel_norm = vectorNorm(input.last_msg.linear_acceleration);
  if (!std::isfinite(accel_norm)) {
    return {false, "acceleration norm is non-finite"};
  }

  if (std::abs(accel_norm - kGravityMetersPerSecondSquared) > config.max_accel_norm_error) {
    return {false, "acceleration norm outside gravity range"};
  }

  if (!isZeroTime(input.last_received_time)) {
    const double receive_age = (now - input.last_received_time).seconds();
    if (receive_age > config.timeout_sec) {
      return {false, "message timeout"};
    }
  }

  const double stamp_age = (now - input.last_stamp).seconds();
  if (stamp_age > config.timeout_sec) {
    return {false, "stale timestamp"};
  }

  if (stamp_age < -config.timeout_sec) {
    return {false, "timestamp is in the future"};
  }

  return {true, "ok"};
}

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__IMU_INPUT_HPP_
