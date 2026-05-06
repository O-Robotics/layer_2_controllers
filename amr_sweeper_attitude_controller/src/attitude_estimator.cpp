#include "attitude_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "imu_input.hpp"

namespace amr_sweeper_attitude_controller
{

namespace
{

geometry_msgs::msg::Vector3 weightedAverageVector(
  const std::vector<ImuMeasurement> & measurements,
  const bool use_acceleration)
{
  geometry_msgs::msg::Vector3 average;
  double total_weight = 0.0;

  for (const auto & measurement : measurements) {
    const double weight = std::max(0.0, measurement.weight);
    if (weight <= 0.0) {
      continue;
    }

    const auto & vector = use_acceleration ?
      measurement.linear_acceleration : measurement.angular_velocity;
    average.x += vector.x * weight;
    average.y += vector.y * weight;
    average.z += vector.z * weight;
    total_weight += weight;
  }

  if (total_weight > 0.0) {
    average.x /= total_weight;
    average.y /= total_weight;
    average.z /= total_weight;
  }

  return average;
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(const AttitudeEstimatorOptions & options)
: options_(options)
{
}

void AttitudeEstimator::setOptions(const AttitudeEstimatorOptions & options)
{
  options_ = options;
}

void AttitudeEstimator::reset()
{
  estimate_ = AttitudeEstimate();
  last_stamp_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  has_estimate_ = false;
}

AttitudeEstimate AttitudeEstimator::update(
  const std::vector<ImuMeasurement> & measurements,
  const rclcpp::Time & stamp)
{
  if (measurements.empty()) {
    estimate_.healthy = false;
    return estimate_;
  }

  const auto fused_accel = weightedAverageVector(measurements, true);
  const auto fused_gyro = weightedAverageVector(measurements, false);
  const double accel_norm = vectorNorm(fused_accel);
  const bool accel_can_correct =
    std::isfinite(accel_norm) &&
    std::abs(accel_norm - kGravityMetersPerSecondSquared) <= options_.max_accel_norm_error;

  double accel_roll = estimate_.roll_rad;
  double accel_pitch = estimate_.pitch_rad;
  if (accel_can_correct) {
    accel_roll = std::atan2(fused_accel.y, fused_accel.z);
    accel_pitch = std::atan2(
      -fused_accel.x,
      std::sqrt((fused_accel.y * fused_accel.y) + (fused_accel.z * fused_accel.z)));
  }

  if (!has_estimate_) {
    estimate_.roll_rad = accel_can_correct ? accel_roll : 0.0;
    estimate_.pitch_rad = accel_can_correct ? accel_pitch : 0.0;
    estimate_.roll_rate_radps = fused_gyro.x;
    estimate_.pitch_rate_radps = fused_gyro.y;
    estimate_.fused_linear_acceleration = fused_accel;
    estimate_.healthy = true;
    last_stamp_ = stamp;
    has_estimate_ = true;
    return estimate_;
  }

  double dt = (stamp - last_stamp_).seconds();
  if (!std::isfinite(dt) || dt < 0.0) {
    dt = 0.0;
  }

  double integrated_roll = estimate_.roll_rad + (fused_gyro.x * dt);
  double integrated_pitch = estimate_.pitch_rad + (fused_gyro.y * dt);

  if (accel_can_correct) {
    const double total_alpha = options_.gyro_alpha + options_.accel_alpha;
    if (total_alpha > 0.0) {
      integrated_roll =
        ((options_.gyro_alpha * integrated_roll) + (options_.accel_alpha * accel_roll)) /
        total_alpha;
      integrated_pitch =
        ((options_.gyro_alpha * integrated_pitch) + (options_.accel_alpha * accel_pitch)) /
        total_alpha;
    }
  }

  estimate_.roll_rad = integrated_roll;
  estimate_.pitch_rad = integrated_pitch;
  estimate_.roll_rate_radps = fused_gyro.x;
  estimate_.pitch_rate_radps = fused_gyro.y;
  estimate_.fused_linear_acceleration = fused_accel;
  estimate_.healthy = true;
  last_stamp_ = stamp;
  return estimate_;
}

AttitudeEstimate AttitudeEstimator::currentEstimate() const
{
  return estimate_;
}

bool AttitudeEstimator::hasEstimate() const
{
  return has_estimate_;
}

double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians)
{
  return radians * 180.0 / M_PI;
}

}  // namespace amr_sweeper_attitude_controller
