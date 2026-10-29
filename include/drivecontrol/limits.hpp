#pragma once
#include "drivecontrol/types.hpp"

namespace dc {

/// Symmetric or asymmetric slew-rate limiter.
class RateLimiter {
 public:
  RateLimiter() = default;
  RateLimiter(double rate_up, double rate_down) : up_(rate_up), down_(rate_down) {}
  void set_rates(double up, double down) {
    up_ = up;
    down_ = down;
  }
  double step(double target, double dt);
  void reset(double v) { y_ = v; }
  double value() const { return y_; }

 private:
  double up_ = 1e9;
  double down_ = 1e9;
  double y_ = 0.0;
};

/// Saturation followed by rate limiting for all three actuator channels.
class ActuatorShaper {
 public:
  explicit ActuatorShaper(const ActuatorLimits& lim);
  ActuatorCommand apply(const ActuatorCommand& raw, double dt, bool emergency);
  void reset(const ActuatorCommand& c);
  const ActuatorLimits& limits() const { return lim_; }

 private:
  ActuatorLimits lim_;
  RateLimiter steer_, throttle_, brake_;
};

}  // namespace dc
