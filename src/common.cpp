#include <cmath>

#include "drivecontrol/limits.hpp"
#include "drivecontrol/pid.hpp"
#include "drivecontrol/types.hpp"

namespace dc {

double VehicleParams::resistance(double v) const {
  const double drag = 0.5 * rho * cd_a * v * std::fabs(v);
  const double roll = v > 0.05 ? c_rr * mass * 9.81 : 0.0;
  return drag + roll;
}

double wrap_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double RateLimiter::step(double target, double dt) {
  const double d = target - y_;
  const double up = up_ * dt, down = down_ * dt;
  if (d > up) {
    y_ += up;
  } else if (d < -down) {
    y_ -= down;
  } else {
    y_ = target;
  }
  return y_;
}

ActuatorShaper::ActuatorShaper(const ActuatorLimits& lim)
    : lim_(lim),
      steer_(lim.steer_rate_max, lim.steer_rate_max),
      throttle_(lim.throttle_rate_up, lim.throttle_rate_down),
      brake_(lim.brake_rate_up, lim.brake_rate_down) {}

ActuatorCommand ActuatorShaper::apply(const ActuatorCommand& raw, double dt, bool emergency) {
  brake_.set_rates(emergency ? lim_.estop_brake_rate : lim_.brake_rate_up, lim_.brake_rate_down);
  // in an emergency the throttle is cut immediately
  throttle_.set_rates(lim_.throttle_rate_up, emergency ? 1e9 : lim_.throttle_rate_down);
  ActuatorCommand out;
  out.steer = steer_.step(clamp(raw.steer, -lim_.steer_max, lim_.steer_max), dt);
  out.throttle = throttle_.step(clamp(raw.throttle, 0.0, 1.0), dt);
  out.brake = brake_.step(clamp(raw.brake, 0.0, 1.0), dt);
  // never request throttle and brake together
  if (out.brake > 0.0 && out.throttle > 0.0) {
    throttle_.reset(0.0);
    out.throttle = 0.0;
  }
  return out;
}

void ActuatorShaper::reset(const ActuatorCommand& c) {
  steer_.reset(c.steer);
  throttle_.reset(c.throttle);
  brake_.reset(c.brake);
}

double Pid::update(double ref, double meas, double ff, double dt) {
  const double e = ref - meas;
  if (first_) {
    prev_meas_ = meas;
    prev_err_ = e;
    first_ = false;
  }
  const double d_raw = cfg_.d_on_measurement ? -(meas - prev_meas_) / dt : (e - prev_err_) / dt;
  d_ += dt / (cfg_.d_tau + dt) * (d_raw - d_);
  prev_meas_ = meas;
  prev_err_ = e;

  u_unsat_ = cfg_.kp * e + i_ + cfg_.kd * d_ + cfg_.kff * ff;
  u_ = clamp(u_unsat_, cfg_.out_min, cfg_.out_max);

  switch (cfg_.anti_windup) {
    case AntiWindup::None:
      i_ += cfg_.ki * e * dt;
      break;
    case AntiWindup::Clamp: {
      const bool saturated = u_ != u_unsat_;
      const bool helps = (u_unsat_ > u_ && e < 0) || (u_unsat_ < u_ && e > 0);
      if (!saturated || helps) i_ += cfg_.ki * e * dt;
      break;
    }
    case AntiWindup::BackCalculation:
      i_ += (cfg_.ki * e + cfg_.kt * (u_ - u_unsat_)) * dt;
      break;
  }
  return u_;
}

void Pid::track(double u_realised, double dt) {
  if (cfg_.anti_windup != AntiWindup::BackCalculation) return;
  i_ += cfg_.kt * (u_realised - u_) * dt;
}

void Pid::reset() {
  i_ = d_ = 0.0;
  first_ = true;
  u_unsat_ = u_ = 0.0;
}

}  // namespace dc
