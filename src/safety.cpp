#include "drivecontrol/safety.hpp"

#include <cmath>

namespace dc {

const char* to_string(DriveState s) {
  switch (s) {
    case DriveState::Disabled: return "disabled";
    case DriveState::Active: return "active";
    case DriveState::ControlledStop: return "controlled_stop";
    case DriveState::EmergencyStop: return "emergency_stop";
    case DriveState::Stopped: return "stopped";
  }
  return "?";
}

void SafetyManager::go(DriveState s, double t, double v) {
  state_ = s;
  t_enter_ = t;
  v_enter_ = v;
  still_since_ = -1;
}

DriveState SafetyManager::update(const Inputs& in, double t) {
  const bool enable_edge = in.enable && !prev_enable_;
  const bool reset_edge = in.reset_request && !prev_reset_;
  prev_enable_ = in.enable;
  prev_reset_ = in.reset_request;

  uint8_t emergency = 0;
  if (in.estop) emergency |= stop_reason::kEStopRequest;
  if (in.estop_link_lost) emergency |= stop_reason::kEStopHeartbeat;
  if (!in.state_valid) emergency |= stop_reason::kInvalidState;

  switch (state_) {
    case DriveState::Disabled:
      if (enable_edge && !emergency && !in.command_link_lost) {
        reasons_ = 0;
        go(DriveState::Active, t, in.v);
      }
      break;
    case DriveState::Active: {
      uint8_t controlled = 0;
      if (in.command_link_lost) controlled |= stop_reason::kCommandTimeout;
      if (std::fabs(in.lateral_error) > cfg_.max_lateral_error) controlled |= stop_reason::kTrackingError;
      if (!in.enable) controlled |= stop_reason::kDisableRequest;
      if (emergency) {
        reasons_ |= emergency;
        go(DriveState::EmergencyStop, t, in.v);
      } else if (controlled) {
        reasons_ |= controlled;
        go(DriveState::ControlledStop, t, in.v);
      }
      break;
    }
    case DriveState::ControlledStop:
      if (emergency) {
        reasons_ |= emergency;
        go(DriveState::EmergencyStop, t, in.v);
        break;
      }
      [[fallthrough]];
    case DriveState::EmergencyStop:
      if (std::fabs(in.v) < cfg_.stop_speed) {
        if (still_since_ < 0) still_since_ = t;
        if (t - still_since_ >= cfg_.stop_hold_s) {
          const uint8_t keep = reasons_;
          go(DriveState::Stopped, t, in.v);
          reasons_ = keep;
        }
      } else {
        still_since_ = -1;
      }
      break;
    case DriveState::Stopped:
      if (reset_edge && !emergency) {
        reasons_ = 0;
        go(DriveState::Disabled, t, in.v);
      }
      break;
  }
  return state_;
}

}  // namespace dc
