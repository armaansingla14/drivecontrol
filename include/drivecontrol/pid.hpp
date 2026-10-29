#pragma once
#include <limits>

namespace dc {

enum class AntiWindup { None, Clamp, BackCalculation };

struct PidConfig {
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double kff = 1.0;  ///< feedforward gain
  double out_min = -std::numeric_limits<double>::infinity();
  double out_max = std::numeric_limits<double>::infinity();
  double d_tau = 0.05;  ///< derivative low-pass time constant, s
  AntiWindup anti_windup = AntiWindup::BackCalculation;
  double kt = 2.0;  ///< back-calculation tracking gain, 1/s
  bool d_on_measurement = true;
};

/// PID with feedforward, filtered derivative (on measurement by default),
/// output saturation and selectable anti-windup.
///
/// u = kp*e + I + kd*D_f + kff*ff, saturated to [out_min, out_max].
/// The integrator is updated after the output is computed:
///   None:            I += ki*e*dt
///   Clamp:           integrate only if not saturated, or if the error drives
///                    the output back out of saturation (conditional integration)
///   BackCalculation: I += (ki*e + kt*(u_sat - u_unsat))*dt
/// track() feeds back a downstream saturation (e.g. throttle at 100 %) so
/// back-calculation also covers limits the PID does not know about.
class Pid {
 public:
  Pid() = default;
  explicit Pid(const PidConfig& c) : cfg_(c) {}
  double update(double ref, double meas, double ff, double dt);
  void track(double u_realised, double dt);
  void reset();
  void set_limits(double lo, double hi) {
    cfg_.out_min = lo;
    cfg_.out_max = hi;
  }
  double integral() const { return i_; }
  double last_unsat() const { return u_unsat_; }
  double last_output() const { return u_; }
  const PidConfig& config() const { return cfg_; }

 private:
  PidConfig cfg_{};
  double i_ = 0.0;
  double d_ = 0.0;
  double prev_meas_ = 0.0;
  double prev_err_ = 0.0;
  bool first_ = true;
  double u_unsat_ = 0.0;
  double u_ = 0.0;
};

}  // namespace dc
