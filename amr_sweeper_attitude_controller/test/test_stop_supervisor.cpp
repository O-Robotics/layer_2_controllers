#include "gtest/gtest.h"
#include "imu_input.hpp"
#include "stop_supervisor.hpp"

namespace
{

using amr_sweeper_attitude_controller::AttitudeEstimate;
using amr_sweeper_attitude_controller::StopSupervisor;
using amr_sweeper_attitude_controller::StopSupervisorOptions;
using amr_sweeper_attitude_controller::degreesToRadians;

TEST(StopSupervisorTest, WarnsBeforeStopping)
{
  StopSupervisor supervisor;
  AttitudeEstimate attitude;
  attitude.roll_rad = degreesToRadians(9.0);
  geometry_msgs::msg::Vector3 accel;
  accel.z = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;

  const auto state = supervisor.update(attitude, accel, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_TRUE(state.warning);
  EXPECT_TRUE(state.roll_warning);
  EXPECT_FALSE(state.stopped);
}

TEST(StopSupervisorTest, LatchesStopWhenManualResetRequired)
{
  StopSupervisor supervisor;
  AttitudeEstimate attitude;
  attitude.pitch_rad = degreesToRadians(21.0);
  geometry_msgs::msg::Vector3 accel;
  accel.z = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;

  auto state = supervisor.update(attitude, accel, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_TRUE(state.stopped);
  EXPECT_TRUE(state.latched);

  attitude.pitch_rad = 0.0;
  state = supervisor.update(attitude, accel, rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_TRUE(state.stopped);
  EXPECT_TRUE(state.latched);

  EXPECT_TRUE(supervisor.resetFault());
  state = supervisor.update(attitude, accel, rclcpp::Time(3, 0, RCL_ROS_TIME));
  EXPECT_FALSE(state.stopped);
  EXPECT_FALSE(state.latched);
}

TEST(StopSupervisorTest, TreatsNominalPitchAsNormal)
{
  StopSupervisor supervisor;
  AttitudeEstimate attitude;
  attitude.pitch_rad = degreesToRadians(5.0);
  geometry_msgs::msg::Vector3 accel;
  accel.z = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;

  const auto state = supervisor.update(attitude, accel, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_FALSE(state.warning);
  EXPECT_FALSE(state.pitch_warning);
  EXPECT_FALSE(state.stopped);
}

TEST(StopSupervisorTest, TreatsConfiguredNominalRollAsNormal)
{
  StopSupervisorOptions options;
  options.nominal_roll_deg = 3.0;
  StopSupervisor supervisor(options);
  AttitudeEstimate attitude;
  attitude.roll_rad = degreesToRadians(3.0);
  attitude.pitch_rad = degreesToRadians(5.0);
  geometry_msgs::msg::Vector3 accel;
  accel.z = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;

  const auto state = supervisor.update(attitude, accel, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_FALSE(state.warning);
  EXPECT_FALSE(state.roll_warning);
  EXPECT_FALSE(state.stopped);
}

TEST(StopSupervisorTest, DetectsSustainedHardDeceleration)
{
  StopSupervisorOptions options;
  options.require_manual_reset = false;
  options.min_event_duration_ms = 80;
  StopSupervisor supervisor(options);
  AttitudeEstimate attitude;
  geometry_msgs::msg::Vector3 accel;
  accel.x = -5.0;
  accel.z = amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;

  auto state = supervisor.update(attitude, accel, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_FALSE(state.hard_decel);
  EXPECT_FALSE(state.stopped);

  state = supervisor.update(attitude, accel, rclcpp::Time(1, 90000000, RCL_ROS_TIME));
  EXPECT_TRUE(state.hard_decel);
  EXPECT_TRUE(state.stopped);
}

}  // namespace
