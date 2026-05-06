#include <limits>

#include "gtest/gtest.h"
#include "imu_input.hpp"

namespace
{

using amr_sweeper_attitude_controller::ImuHealthConfig;
using amr_sweeper_attitude_controller::ImuInput;
using amr_sweeper_attitude_controller::checkImuHealth;

ImuInput validInput()
{
  ImuInput input;
  input.topic = "/imu";
  input.has_message = true;
  input.last_stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  input.last_received_time = rclcpp::Time(1, 0, RCL_ROS_TIME);
  input.last_msg.header.stamp = input.last_stamp;
  input.last_msg.linear_acceleration.z =
    amr_sweeper_attitude_controller::kGravityMetersPerSecondSquared;
  return input;
}

TEST(ImuHealthTest, ReportsNoMessage)
{
  const auto health = checkImuHealth(
    ImuInput(),
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    ImuHealthConfig{});

  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "no messages received");
}

TEST(ImuHealthTest, AcceptsValidMessage)
{
  const auto health = checkImuHealth(
    validInput(),
    rclcpp::Time(1, 100000000, RCL_ROS_TIME),
    ImuHealthConfig{});

  EXPECT_TRUE(health.healthy);
}

TEST(ImuHealthTest, RejectsNanGyro)
{
  auto input = validInput();
  input.last_msg.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();

  const auto health = checkImuHealth(
    input,
    rclcpp::Time(1, 100000000, RCL_ROS_TIME),
    ImuHealthConfig{});

  EXPECT_FALSE(health.healthy);
}

TEST(ImuHealthTest, RejectsAccelerationOutsideGravityRange)
{
  auto input = validInput();
  input.last_msg.linear_acceleration.z = 20.0;

  const auto health = checkImuHealth(
    input,
    rclcpp::Time(1, 100000000, RCL_ROS_TIME),
    ImuHealthConfig{});

  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "acceleration norm outside gravity range");
}

TEST(ImuHealthTest, RejectsStaleTimestamp)
{
  auto input = validInput();

  const auto health = checkImuHealth(
    input,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    ImuHealthConfig{});

  EXPECT_FALSE(health.healthy);
}

}  // namespace
