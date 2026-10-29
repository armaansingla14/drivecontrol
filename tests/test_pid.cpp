#include <cmath>

#include "drivecontrol/limits.hpp"
#include "drivecontrol/pid.hpp"
#include "testfw.hpp"

using namespace dc;

TEST(Pid, ProportionalOnly) {
  PidConfig c;
  c.kp = 2.0;
  c.ki = 0;
  c.kff = 0;
  Pid p(c);
  CHECK_NEAR(p.update(3.0, 1.0, 0.0, 0.01), 4.0, 1e-12);
}

TEST(Pid, FeedforwardAdds) {
  PidConfig c;
  c.kp = 1.0;
  c.kff = 0.5;
  Pid p(c);
  CHECK_NEAR(p.update(1.0, 1.0, 4.0, 0.01), 2.0, 1e-12);
}

TEST(Pid, IntegralRemovesSteadyStateError) {
  PidConfig c;
  c.kp = 0.5;
  c.ki = 2.0;
  c.kff = 0;
  Pid p(c);
  // first-order plant y' = -y + u, setpoint 1
  double y = 0;
  for (int i = 0; i < 2000; ++i) {
    const double u = p.update(1.0, y, 0, 0.01);
    y += (-y + u) * 0.01;
  }
  CHECK_NEAR(y, 1.0, 1e-3);
}

TEST(Pid, OutputSaturates) {
  PidConfig c;
  c.kp = 10;
  c.out_min = -1;
  c.out_max = 1;
  Pid p(c);
  CHECK_NEAR(p.update(5, 0, 0, 0.01), 1.0, 1e-12);
  CHECK_NEAR(p.update(-5, 0, 0, 0.01), -1.0, 1e-12);
}

TEST(Pid, DerivativeOnMeasurementNoKickOnSetpointStep) {
  PidConfig c;
  c.kp = 0;
  c.kd = 1.0;
  c.kff = 0;
  c.d_tau = 0.0;
  Pid p(c);
  p.update(0, 0, 0, 0.01);
  CHECK_NEAR(p.update(10, 0, 0, 0.01), 0.0, 1e-12);  // setpoint jump, measurement constant
  CHECK_NEAR(p.update(10, 0.1, 0, 0.01), -10.0, 1e-9);
}

TEST(Pid, DerivativeFilter) {
  PidConfig c;
  c.kp = 0;
  c.kd = 1.0;
  c.kff = 0;
  c.d_tau = 0.09;  // alpha = 0.1
  Pid p(c);
  p.update(0, 0, 0, 0.01);
  CHECK_NEAR(p.update(0, -0.1, 0, 0.01), 1.0, 1e-9);  // 10 * 0.1
}

namespace {
// saturated plant run: returns integrator state after a long saturation
double windup_integral(AntiWindup aw) {
  PidConfig c;
  c.kp = 0.5;
  c.ki = 1.0;
  c.kff = 0;
  c.out_min = -1;
  c.out_max = 1;
  c.anti_windup = aw;
  Pid p(c);
  for (int i = 0; i < 1000; ++i) p.update(10.0, 0.0, 0, 0.01);  // error 10 for 10 s
  return p.integral();
}
}  // namespace

TEST(Pid, NoAntiWindupIntegratorGrows) { CHECK_NEAR(windup_integral(AntiWindup::None), 100.0, 1e-6); }

TEST(Pid, ClampStopsIntegration) { CHECK(windup_integral(AntiWindup::Clamp) < 0.05); }

TEST(Pid, BackCalculationBoundsIntegrator) {
  const double i = windup_integral(AntiWindup::BackCalculation);
  // equilibrium: ki*e + kt*(u_sat - u_unsat) = 0 -> I = 1 - kp*e + ki*e/kt = 1 - 5 + 5 = 1
  CHECK_NEAR(i, 1.0, 1e-3);
}

TEST(Pid, ClampResumesWhenErrorReverses) {
  PidConfig c;
  c.kp = 0.0;
  c.ki = 1.0;
  c.kff = 0;
  c.out_min = -1;
  c.out_max = 1;
  c.anti_windup = AntiWindup::Clamp;
  Pid p(c);
  for (int i = 0; i < 300; ++i) p.update(1.0, 0.0, 0, 0.01);
  const double i_sat = p.integral();
  p.update(-1.0, 0.0, 0, 0.01);
  CHECK(p.integral() < i_sat);
}

TEST(Pid, TrackFeedsBackDownstreamSaturation) {
  PidConfig c;
  c.kp = 1.0;
  c.ki = 1.0;
  c.kff = 0;
  c.anti_windup = AntiWindup::BackCalculation;
  Pid p(c);
  for (int i = 0; i < 500; ++i) {
    p.update(5.0, 0.0, 0, 0.01);
    p.track(1.0, 0.01);  // actuator only delivers 1.0
  }
  CHECK(p.integral() < 3.0);
}

TEST(Pid, ResetClearsState) {
  PidConfig c;
  c.ki = 1.0;
  Pid p(c);
  for (int i = 0; i < 100; ++i) p.update(1, 0, 0, 0.01);
  p.reset();
  CHECK_NEAR(p.integral(), 0.0, 0);
}

TEST(RateLimiter, LimitsSlope) {
  RateLimiter r(1.0, 2.0);
  CHECK_NEAR(r.step(10.0, 0.1), 0.1, 1e-12);
  CHECK_NEAR(r.step(-10.0, 0.1), -0.1, 1e-12);
  r.reset(0.5);
  CHECK_NEAR(r.step(0.55, 0.1), 0.55, 1e-12);
}

TEST(ActuatorShaper, SaturationAndRates) {
  ActuatorLimits lim;
  ActuatorShaper sh(lim);
  ActuatorCommand raw{1.0, 1.0, 0.0};
  const auto out = sh.apply(raw, 0.01, false);
  CHECK_NEAR(out.steer, lim.steer_rate_max * 0.01, 1e-12);
  CHECK_NEAR(out.throttle, lim.throttle_rate_up * 0.01, 1e-12);
  ActuatorCommand last{};
  for (int i = 0; i < 300; ++i) last = sh.apply(raw, 0.01, false);
  CHECK_NEAR(last.steer, lim.steer_max, 1e-12);
  CHECK_NEAR(last.throttle, 1.0, 1e-12);
}

TEST(ActuatorShaper, NoThrottleAndBrakeTogether) {
  ActuatorShaper sh{ActuatorLimits{}};
  for (int i = 0; i < 100; ++i) sh.apply({0, 1.0, 0.0}, 0.01, false);
  const auto out = sh.apply({0, 0.0, 1.0}, 0.01, false);
  CHECK(out.brake > 0);
  CHECK_NEAR(out.throttle, 0.0, 0);
}

TEST(ActuatorShaper, EmergencyBrakeRampFaster) {
  ActuatorLimits lim;
  ActuatorShaper a(lim), b(lim);
  const auto n = a.apply({0, 0, 1}, 0.01, false);
  const auto e = b.apply({0, 0, 1}, 0.01, true);
  CHECK_NEAR(n.brake, lim.brake_rate_up * 0.01, 1e-12);
  CHECK_NEAR(e.brake, lim.estop_brake_rate * 0.01, 1e-12);
}

TEST_MAIN()
