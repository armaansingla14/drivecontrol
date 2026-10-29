#pragma once
#include "drivecontrol/pid.hpp"
#include "drivecontrol/types.hpp"

namespace dc {

/// Speed controller: PID on speed producing a desired acceleration
/// (feedforward = reference acceleration), converted to throttle/brake with
/// an inverse longitudinal model (mass, drag, rolling resistance). A small
/// hysteresis band around zero force avoids throttle/brake chattering, and
/// saturation of the pedals is fed back to the PID (anti-windup).
struct SpeedControlConfig {
  PidConfig pid{0.8, 0.25, 0.0, 1.0, -8.0, 4.0, 0.05, AntiWindup::BackCalculation, 2.0, true};
  double coast_band_n = 150.0;  ///< |force| below this -> coast
};

class SpeedController {
 public:
  SpeedController(const SpeedControlConfig& c, const VehicleParams& vp) : cfg_(c), vp_(vp), pid_(c.pid) {}
  /// Returns throttle/brake (steer untouched). a_ref is the profile acceleration.
  ActuatorCommand step(double v_ref, double a_ref, double v, double dt);
  void reset() {
    pid_.reset();
    mode_ = 0;
  }
  double desired_accel() const { return a_des_; }
  const Pid& pid() const { return pid_; }

 private:
  SpeedControlConfig cfg_;
  VehicleParams vp_;
  Pid pid_;
  int mode_ = 0;  // +1 drive, -1 brake, 0 coast
  double a_des_ = 0.0;
};

}  // namespace dc
