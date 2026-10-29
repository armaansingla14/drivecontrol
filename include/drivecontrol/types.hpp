#pragma once
#include <cstdint>

namespace dc {

/// Vehicle state as provided by localisation / odometry. Position is the
/// rear-axle centre in a local ENU frame.
struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;       ///< rad
  double v = 0.0;         ///< longitudinal speed, m/s
  double steer = 0.0;     ///< measured road-wheel angle, rad
  double yaw_rate = 0.0;  ///< rad/s
  bool valid = true;
};

/// Output to the actuators.
struct ActuatorCommand {
  double steer = 0.0;     ///< road-wheel angle, rad
  double throttle = 0.0;  ///< 0..1
  double brake = 0.0;     ///< 0..1
};

/// Physical parameters used by feedforward and limits.
struct VehicleParams {
  double wheelbase = 2.7;     ///< m
  double mass = 1500.0;       ///< kg
  double drive_force_max = 6000.0;   ///< N at throttle = 1
  double brake_force_max = 12000.0;  ///< N at brake = 1
  double cd_a = 0.7;          ///< drag coefficient x frontal area, m^2
  double rho = 1.2;           ///< air density
  double c_rr = 0.012;        ///< rolling resistance coefficient
  double resistance(double v) const;  ///< drag + rolling resistance force at speed v (N)
};

/// Actuator constraints enforced on every command.
struct ActuatorLimits {
  double steer_max = 0.52;       ///< rad (about 30 deg road-wheel)
  double steer_rate_max = 0.6;   ///< rad/s
  double throttle_rate_up = 1.5;   ///< 1/s
  double throttle_rate_down = 4.0; ///< 1/s
  double brake_rate_up = 4.0;      ///< 1/s (normal operation)
  double brake_rate_down = 4.0;    ///< 1/s
  double estop_brake_rate = 10.0;  ///< 1/s jerk-limited emergency ramp
};

inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
double wrap_angle(double a);

}  // namespace dc
