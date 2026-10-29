#pragma once
#include <cstdint>

namespace dc {

enum class DriveState : uint8_t { Disabled = 0, Active = 1, ControlledStop = 2, EmergencyStop = 3, Stopped = 4 };
const char* to_string(DriveState s);

/// Reasons latched when leaving Active (bit field, reported in DC_Status).
namespace stop_reason {
constexpr uint8_t kEStopRequest = 0x01;
constexpr uint8_t kEStopHeartbeat = 0x02;  ///< safety channel lost
constexpr uint8_t kCommandTimeout = 0x04;  ///< planner lost
constexpr uint8_t kTrackingError = 0x08;
constexpr uint8_t kInvalidState = 0x10;
constexpr uint8_t kDisableRequest = 0x20;
}  // namespace stop_reason

struct SafetyConfig {
  double max_lateral_error = 3.0;  ///< m, beyond -> controlled stop
  double controlled_decel = 3.0;   ///< m/s^2 for controlled stops
  double stop_speed = 0.1;         ///< m/s considered standstill
  double stop_hold_s = 0.5;
};

/// Emergency-stop / degradation state machine.
///
///   Disabled --enable--> Active
///   Active --planner timeout, tracking error, disable--> ControlledStop
///   Active/ControlledStop --e-stop bit, e-stop heartbeat loss, invalid state--> EmergencyStop
///   ControlledStop/EmergencyStop --standstill for stop_hold_s--> Stopped
///   Stopped --reset request (no e-stop active)--> Disabled
class SafetyManager {
 public:
  struct Inputs {
    bool enable = false;
    bool reset_request = false;
    bool estop = false;
    bool estop_link_lost = false;
    bool command_link_lost = false;
    bool state_valid = true;
    double lateral_error = 0.0;
    double v = 0.0;
  };
  explicit SafetyManager(const SafetyConfig& c) : cfg_(c) {}
  DriveState update(const Inputs& in, double t);
  DriveState state() const { return state_; }
  uint8_t reasons() const { return reasons_; }
  double entered_at() const { return t_enter_; }
  double speed_at_entry() const { return v_enter_; }
  void force_active() { state_ = DriveState::Active; }

 private:
  void go(DriveState s, double t, double v);
  SafetyConfig cfg_;
  DriveState state_ = DriveState::Disabled;
  uint8_t reasons_ = 0;
  double t_enter_ = 0;
  double v_enter_ = 0;
  double still_since_ = -1;
  bool prev_enable_ = false;
  bool prev_reset_ = false;
};

}  // namespace dc
