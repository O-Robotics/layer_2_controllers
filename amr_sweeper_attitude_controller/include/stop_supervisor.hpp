#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_

#include <string>

#include "attitude_estimator.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_sweeper_attitude_controller
{

struct StopSupervisorOptions
{
  double roll_warning_deg{8.0};
  double pitch_warning_deg{8.0};
  double roll_stop_deg{15.0};
  double pitch_stop_deg{15.0};
  double roll_latch_deg{25.0};
  double pitch_latch_deg{25.0};
  double nominal_roll_deg{0.0};
  double nominal_pitch_deg{5.0};
  double hard_decel_threshold_mps2{4.0};
  double shock_threshold_mps2{12.0};
  int min_event_duration_ms{80};
  bool require_manual_reset{true};
};

struct StopSupervisorState
{
  bool warning{false};
  bool stopped{false};
  bool latched{false};
  bool roll_warning{false};
  bool pitch_warning{false};
  bool roll_stop{false};
  bool pitch_stop{false};
  bool roll_latch{false};
  bool pitch_latch{false};
  bool shock{false};
  bool hard_decel{false};
  std::string reason{"ok"};
};

class StopSupervisor
{
public:
  explicit StopSupervisor(const StopSupervisorOptions & options = StopSupervisorOptions{});

  void setOptions(const StopSupervisorOptions & options);
  StopSupervisorState update(
    const AttitudeEstimate & attitude,
    const geometry_msgs::msg::Vector3 & linear_acceleration,
    const rclcpp::Time & stamp);
  bool resetFault();
  StopSupervisorState state() const;

private:
  StopSupervisorOptions options_;
  StopSupervisorState state_;
  rclcpp::Time hard_decel_start_{0, 0, RCL_ROS_TIME};
  bool hard_decel_active_{false};
};

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_
