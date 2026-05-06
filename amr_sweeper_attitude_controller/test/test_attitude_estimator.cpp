#include <cmath>
#include <vector>

#include "attitude_estimator.hpp"
#include "gtest/gtest.h"
#include "imu_input.hpp"

namespace
{

using amr_sweeper_attitude_controller::AttitudeEstimator;
using amr_sweeper_attitude_controller::AttitudeEstimatorOptions;
using amr_sweeper_attitude_controller::ImuMeasurement;
using amr_sweeper_attitude_controller::degreesToRadians;

ImuMeasurement measurementForRollPitch(double roll_rad, double pitch_rad)
{
  constexpr double gravity = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;
  ImuMeasurement measurement;
  measurement.linear_acceleration.x = -std::sin(pitch_rad) * gravity;
  measurement.linear_acceleration.y = std::sin(roll_rad) * std::cos(pitch_rad) * gravity;
  measurement.linear_acceleration.z = std::cos(roll_rad) * std::cos(pitch_rad) * gravity;
  measurement.weight = 1.0;
  return measurement;
}

TEST(AttitudeEstimatorTest, InitializesFromAccelerometerTilt)
{
  AttitudeEstimator estimator;
  const auto estimate = estimator.update(
    std::vector<ImuMeasurement>{measurementForRollPitch(degreesToRadians(10.0), 0.0)},
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_TRUE(estimate.healthy);
  EXPECT_NEAR(estimate.roll_rad, degreesToRadians(10.0), 1e-6);
  EXPECT_NEAR(estimate.pitch_rad, 0.0, 1e-6);
}

TEST(AttitudeEstimatorTest, IntegratesGyroWhenAccelerometerCorrectionIsDisabled)
{
  AttitudeEstimatorOptions options;
  options.accel_alpha = 0.0;
  options.gyro_alpha = 1.0;
  AttitudeEstimator estimator(options);

  auto measurement = measurementForRollPitch(0.0, 0.0);
  estimator.update(std::vector<ImuMeasurement>{measurement}, rclcpp::Time(1, 0, RCL_ROS_TIME));

  measurement.angular_velocity.x = 0.1;
  const auto estimate = estimator.update(
    std::vector<ImuMeasurement>{measurement},
    rclcpp::Time(2, 0, RCL_ROS_TIME));

  EXPECT_TRUE(estimate.healthy);
  EXPECT_NEAR(estimate.roll_rad, 0.1, 1e-6);
}

TEST(AttitudeEstimatorTest, ReportsPhysicalPitchWithoutNominalOffset)
{
  AttitudeEstimator estimator;

  const auto estimate = estimator.update(
    std::vector<ImuMeasurement>{measurementForRollPitch(0.0, degreesToRadians(5.0))},
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_TRUE(estimate.healthy);
  EXPECT_NEAR(estimate.roll_rad, 0.0, 1e-6);
  EXPECT_NEAR(estimate.pitch_rad, degreesToRadians(5.0), 1e-6);
}

TEST(AttitudeEstimatorTest, FusesMultipleImusWithWeights)
{
  AttitudeEstimator estimator;
  auto left = measurementForRollPitch(degreesToRadians(10.0), 0.0);
  auto right = measurementForRollPitch(degreesToRadians(20.0), 0.0);
  left.weight = 1.0;
  right.weight = 3.0;

  const auto estimate = estimator.update(
    std::vector<ImuMeasurement>{left, right},
    rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_TRUE(estimate.healthy);
  EXPECT_GT(estimate.roll_rad, degreesToRadians(15.0));
  EXPECT_LT(estimate.roll_rad, degreesToRadians(20.1));
}

}  // namespace
