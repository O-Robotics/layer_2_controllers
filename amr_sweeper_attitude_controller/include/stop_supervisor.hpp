#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_

#include <string>

#include "attitude_estimator.hpp"

namespace amr_sweeper_attitude_controller
{

struct StopSupervisorOptions
{
  double roll_warning_deg{15.0};
  double pitch_warning_deg{15.0};
  double roll_stop_deg{30.0};
  double pitch_stop_deg{30.0};
  double nominal_roll_deg{0.0};
  double nominal_pitch_deg{5.0};
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
  std::string reason{"ok"};
};

class StopSupervisor
{
public:
  explicit StopSupervisor(const StopSupervisorOptions & options = StopSupervisorOptions{});

  void setOptions(const StopSupervisorOptions & options);
  StopSupervisorState update(const AttitudeEstimate & attitude);
  bool resetFault();
  StopSupervisorState state() const;

private:
  StopSupervisorOptions options_;
  StopSupervisorState state_;
};

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__STOP_SUPERVISOR_HPP_
