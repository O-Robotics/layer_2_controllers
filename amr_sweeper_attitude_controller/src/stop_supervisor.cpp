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

StopSupervisorState StopSupervisor::update(const AttitudeEstimate & attitude)
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

  next.warning = next.roll_warning || next.pitch_warning;
  const bool stop_now = next.roll_stop || next.pitch_stop;

  if (options_.require_manual_reset) {
    next.latched = state_.latched || stop_now;
    next.stopped = next.latched;
  } else {
    next.latched = false;
    next.stopped = stop_now;
  }

  std::vector<std::string> reasons;
  if (!next.roll_stop && !next.pitch_stop) {
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

  appendReason(&reasons, next.latched && !stop_now, "manual reset required");
  next.reason = joinReasons(reasons);

  state_ = next;
  return state_;
}

bool StopSupervisor::resetFault()
{
  state_.latched = false;
  state_.stopped = false;
  state_.reason = "reset";
  return true;
}

StopSupervisorState StopSupervisor::state() const
{
  return state_;
}

}  // namespace amr_sweeper_attitude_controller
