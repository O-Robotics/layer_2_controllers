#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_ESTIMATOR_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_ESTIMATOR_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_sweeper_attitude_controller
{

struct AttitudeEstimatorOptions
{
  std::string filter_type{"complementary"};
  double accel_alpha{0.02};
  double gyro_alpha{0.98};
  double max_accel_norm_error{2.0};
};

struct ImuMeasurement
{
  geometry_msgs::msg::Vector3 linear_acceleration;
  geometry_msgs::msg::Vector3 angular_velocity;
  double weight{1.0};
};

struct AttitudeEstimate
{
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double roll_rate_radps{0.0};
  double pitch_rate_radps{0.0};
  geometry_msgs::msg::Vector3 fused_linear_acceleration;
  bool healthy{false};
};

class AttitudeEstimator
{
public:
  explicit AttitudeEstimator(const AttitudeEstimatorOptions & options = AttitudeEstimatorOptions{});

  void setOptions(const AttitudeEstimatorOptions & options);
  void reset();

  AttitudeEstimate update(
    const std::vector<ImuMeasurement> & measurements,
    const rclcpp::Time & stamp);

  AttitudeEstimate currentEstimate() const;
  bool hasEstimate() const;

private:
  AttitudeEstimatorOptions options_;
  AttitudeEstimate estimate_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_estimate_{false};
};

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_ESTIMATOR_HPP_
