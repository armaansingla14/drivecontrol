#include "drivecontrol/longitudinal.hpp"

namespace dc {

ActuatorCommand SpeedController::step(double v_ref, double a_ref, double v, double dt) {
  a_des_ = pid_.update(v_ref, v, a_ref, dt);
  const double force = vp_.mass * a_des_ + vp_.resistance(v);
  // hysteresis between drive and brake
  if (force > cfg_.coast_band_n) {
    mode_ = 1;
  } else if (force < -cfg_.coast_band_n) {
    mode_ = -1;
  } else if ((mode_ == 1 && force < 0) || (mode_ == -1 && force > 0)) {
    mode_ = 0;
  }
  ActuatorCommand c;
  double realised_force = force;
  if (mode_ == 1) {
    c.throttle = clamp(force / vp_.drive_force_max, 0.0, 1.0);
    realised_force = c.throttle * vp_.drive_force_max;
  } else if (mode_ == -1) {
    c.brake = clamp(-force / vp_.brake_force_max, 0.0, 1.0);
    realised_force = -c.brake * vp_.brake_force_max;
  } else {
    realised_force = 0.0;
  }
  // pedal saturation -> tell the PID what acceleration was actually requested
  if (mode_ != 0) pid_.track((realised_force - vp_.resistance(v)) / vp_.mass, dt);
  return c;
}

}  // namespace dc
