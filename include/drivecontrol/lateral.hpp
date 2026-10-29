#pragma once
#include <cstddef>
#include <cstdint>

#include "drivecontrol/path.hpp"
#include "drivecontrol/types.hpp"

namespace dc {

struct LateralDebug {
  double lateral_error = 0;  ///< at the controller's reference point
  double heading_error = 0;
  double lookahead = 0;
  double s = 0;
};

/// Pure pursuit on the rear axle: steer = atan(2 L sin(alpha) / ld), with a
/// speed-scheduled look-ahead distance ld = clamp(ld0 + k_v * v).
struct PurePursuitConfig {
  double ld0 = 3.0;
  double k_v = 0.5;  ///< s
  double ld_min = 3.0;
  double ld_max = 25.0;
};

class PurePursuit {
 public:
  PurePursuit(const PurePursuitConfig& c, double wheelbase) : cfg_(c), L_(wheelbase) {}
  double steer(const Path& path, const VehicleState& s, LateralDebug* dbg = nullptr);
  void reset() { hint_ = SIZE_MAX; }

 private:
  PurePursuitConfig cfg_;
  double L_;
  size_t hint_ = SIZE_MAX;
};

/// Stanley (front axle): steer = psi_e + atan(k e / (k_soft + v)), with
/// yaw-rate damping and an understeer feedforward k_us * v^2 * kappa.
/// The heading error is taken against the path tangent at the front-axle
/// projection, which already contains the kinematic steering angle for a
/// curve; adding atan(L kappa) on top would double-count it.
struct StanleyConfig {
  double k = 1.5;
  double k_soft = 1.0;
  double k_yaw_rate = 0.05;    ///< damping on (yaw_rate - v*kappa)
  double k_understeer = 0.002;  ///< rad per m/s^2 of lateral acceleration
};

class Stanley {
 public:
  Stanley(const StanleyConfig& c, double wheelbase) : cfg_(c), L_(wheelbase) {}
  double steer(const Path& path, const VehicleState& s, LateralDebug* dbg = nullptr);
  void reset() { hint_ = SIZE_MAX; }

 private:
  StanleyConfig cfg_;
  double L_;
  size_t hint_ = SIZE_MAX;
};

}  // namespace dc
