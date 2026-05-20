#include "stop_supervisor.hpp"

#include <cmath>
#include <sstream>
#include <vector>

#include "imu_input.hpp"

namespace amr_sweeper_attitude_controller
{

namespace
{

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

StopSupervisor::StopSupervisor(const StopSupervisorOptions & options)
: options_(options)
{
}

void StopSupervisor::setOptions(const StopSupervisorOptions & options)
{
  options_ = options;
}

StopSupervisorState StopSupervisor::update(
  const AttitudeEstimate & attitude,
  const geometry_msgs::msg::Vector3 & linear_acceleration,
  const rclcpp::Time & stamp)
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
  next.roll_latch = abs_roll_error >= degreesToRadians(options_.roll_latch_deg);
  next.pitch_latch = abs_pitch_error >= degreesToRadians(options_.pitch_latch_deg);

  const double accel_norm = vectorNorm(linear_acceleration);
  next.shock =
    std::isfinite(accel_norm) && accel_norm >= options_.shock_threshold_mps2;

  const bool hard_decel_now =
    std::isfinite(linear_acceleration.x) &&
    linear_acceleration.x <= -std::abs(options_.hard_decel_threshold_mps2);
  if (hard_decel_now && !hard_decel_active_) {
    hard_decel_start_ = stamp;
    hard_decel_active_ = true;
  } else if (!hard_decel_now) {
    hard_decel_active_ = false;
    hard_decel_start_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  }

  if (hard_decel_active_ && hard_decel_start_.nanoseconds() != 0) {
    const double duration_ms = (stamp - hard_decel_start_).seconds() * 1000.0;
    next.hard_decel = duration_ms >= static_cast<double>(options_.min_event_duration_ms);
  }

  next.warning = next.roll_warning || next.pitch_warning;
  const bool stop_now =
    next.roll_stop || next.pitch_stop || next.shock || next.hard_decel;
  const bool latch_now = next.roll_latch || next.pitch_latch;

  if (options_.require_manual_reset) {
    next.latched = state_.latched || stop_now || latch_now;
    next.stopped = next.latched;
  } else {
    next.latched = latch_now;
    next.stopped = stop_now || latch_now;
  }

  std::vector<std::string> reasons;
  if (!next.roll_stop && !next.pitch_stop && !next.roll_latch && !next.pitch_latch) {
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

  if (next.roll_latch && next.pitch_latch) {
    reasons.push_back("roll and pitch latch");
  } else {
    appendReason(&reasons, next.roll_latch, "roll latch");
    appendReason(&reasons, next.pitch_latch, "pitch latch");
  }

  appendReason(&reasons, next.shock, "shock acceleration");
  appendReason(&reasons, next.hard_decel, "hard deceleration");
  appendReason(&reasons, next.latched && !stop_now && !latch_now, "manual reset required");
  next.reason = joinReasons(reasons);

  state_ = next;
  return state_;
}

bool StopSupervisor::resetFault()
{
  state_.latched = false;
  state_.stopped = false;
  state_.reason = "reset";
  hard_decel_active_ = false;
  hard_decel_start_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
  return true;
}

StopSupervisorState StopSupervisor::state() const
{
  return state_;
}

}  // namespace amr_sweeper_attitude_controller
