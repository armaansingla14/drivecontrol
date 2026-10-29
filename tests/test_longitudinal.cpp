#include <cmath>

#include "drivecontrol/longitudinal.hpp"
#include "testfw.hpp"

using namespace dc;

namespace {
// simple longitudinal plant matching VehicleParams
double simulate(SpeedController& sc, double v_ref, double v0, double seconds, double grade = 0.0,
                double* max_v = nullptr) {
  const VehicleParams vp;
  double v = v0;
  if (max_v) *max_v = v0;
  for (int i = 0; i < static_cast<int>(seconds / 0.01); ++i) {
    const auto c = sc.step(v_ref, 0.0, v, 0.01);
    const double f = c.throttle * vp.drive_force_max - c.brake * vp.brake_force_max - vp.resistance(v) -
                     vp.mass * 9.81 * grade;
    v = std::max(0.0, v + f / vp.mass * 0.01);
    if (max_v) *max_v = std::max(*max_v, v);
  }
  return v;
}
}  // namespace

TEST(Speed, ReachesSetpoint) {
  SpeedController sc(SpeedControlConfig{}, VehicleParams{});
  CHECK_NEAR(simulate(sc, 20.0, 0.0, 40.0), 20.0, 0.05);
}

TEST(Speed, HoldsSpeedOnGrade) {
  SpeedController sc(SpeedControlConfig{}, VehicleParams{});
  CHECK_NEAR(simulate(sc, 15.0, 15.0, 60.0, 0.05), 15.0, 0.1);
}

TEST(Speed, BrakesToSlowDown) {
  SpeedController sc(SpeedControlConfig{}, VehicleParams{});
  const auto c = sc.step(10.0, 0.0, 25.0, 0.01);
  CHECK(c.brake > 0.0);
  CHECK_NEAR(c.throttle, 0.0, 0);
}

TEST(Speed, ThrottleAndBrakeExclusive) {
  SpeedController sc(SpeedControlConfig{}, VehicleParams{});
  for (double v = 0; v < 40; v += 0.37) {
    const auto c = sc.step(20.0, 0.0, v, 0.01);
    CHECK(!(c.throttle > 0 && c.brake > 0));
  }
}

TEST(Speed, AntiWindupLimitsOvershoot) {
  SpeedControlConfig with;
  SpeedControlConfig without;
  without.pid.anti_windup = AntiWindup::None;
  SpeedController a(with, VehicleParams{}), b(without, VehicleParams{});
  double max_a, max_b;
  simulate(a, 30.0, 0.0, 40.0, 0.0, &max_a);
  simulate(b, 30.0, 0.0, 40.0, 0.0, &max_b);
  CHECK(max_a - 30.0 < 1.0);
  CHECK(max_b - 30.0 > 3.0);
}

TEST(Speed, FeedforwardUsesReferenceAcceleration) {
  SpeedControlConfig cfg;
  cfg.pid.ki = 0;
  SpeedController a(cfg, VehicleParams{});
  SpeedController b(cfg, VehicleParams{});
  const auto c0 = a.step(10.0, 0.0, 10.0, 0.01);
  const auto c1 = b.step(10.0, 1.5, 10.0, 0.01);
  CHECK(c1.throttle > c0.throttle);
  CHECK_NEAR(b.desired_accel(), 1.5, 1e-9);
}

TEST(Speed, CoastBandAvoidsChatter) {
  SpeedController sc(SpeedControlConfig{}, VehicleParams{});
  int switches = 0;
  int prev = 0;
  for (int i = 0; i < 1000; ++i) {
    const double v = 20.0 + 0.02 * std::sin(i * 0.3);  // noise around the setpoint
    const auto c = sc.step(20.0, 0.0, v, 0.01);
    const int mode = c.throttle > 0 ? 1 : (c.brake > 0 ? -1 : 0);
    if (mode != 0 && prev != 0 && mode != prev) ++switches;
    if (mode != 0) prev = mode;
  }
  CHECK_EQ(switches, 0);
}

TEST(Resistance, Positive) {
  VehicleParams vp;
  CHECK(vp.resistance(30) > vp.resistance(10));
  CHECK_NEAR(vp.resistance(0), 0.0, 0);
}

TEST_MAIN()
