// Closed-loop checks of the full controller on a minimal kinematic plant.
#include <cmath>
#include <vector>

#include "drivecontrol/controller.hpp"
#include "testfw.hpp"

using namespace dc;

namespace {
Path straight_path(double v) {
  std::vector<double> x, y, vv;
  for (double s = 0; s <= 2000; s += 0.5) {
    x.push_back(s);
    y.push_back(0);
    vv.push_back(v);
  }
  return Path(x, y, vv);
}

struct Plant {
  VehicleState s;
  double steer = 0;
  void step(const ActuatorCommand& c, double dt) {
    const VehicleParams vp;
    steer += (c.steer - steer) * dt / 0.08;
    const double f = c.throttle * vp.drive_force_max - c.brake * vp.brake_force_max * (s.v > 0.01) - vp.resistance(s.v);
    s.v = std::max(0.0, s.v + f / vp.mass * dt);
    s.yaw += s.v / vp.wheelbase * std::tan(steer) * dt;
    s.x += s.v * std::cos(s.yaw) * dt;
    s.y += s.v * std::sin(s.yaw) * dt;
    s.steer = steer;
  }
};

struct Bus {
  uint8_t ce = 0, cc = 0;
  bool estop = false, hb = true, cmd = true;
  double speed = 20;
  void tick(IController& c, double t, int k) {
    if (hb && k % 2 == 0) c.on_can(encode(EStopMsg{estop}, ce++), t);
    if (cmd && k % 5 == 0) {
      CommandMsg m;
      m.target_speed = speed;
      m.enable = true;
      c.on_can(encode(m, cc++), t);
    }
  }
};
}  // namespace

TEST(Controller, ConvergesOntoPathFromOffset) {
  DriveConfig cfg;
  DriveController dc(cfg);
  dc.set_path(straight_path(10));
  Plant p;
  p.s.y = 2.0;
  p.s.v = 10;
  for (int k = 0; k < 1500; ++k) p.step(dc.step(p.s, k * 0.01, 0.01), 0.01);
  CHECK(std::fabs(p.s.y) < 0.05);
  CHECK_NEAR(p.s.v, 10.0, 0.2);
}

TEST(Controller, PurePursuitConvergesToo) {
  DriveConfig cfg;
  cfg.lateral = LateralMode::PurePursuit;
  DriveController dc(cfg);
  dc.set_path(straight_path(10));
  Plant p;
  p.s.y = -1.5;
  p.s.v = 10;
  for (int k = 0; k < 1500; ++k) p.step(dc.step(p.s, k * 0.01, 0.01), 0.01);
  CHECK(std::fabs(p.s.y) < 0.05);
}

TEST(Controller, OutputsRespectLimits) {
  DriveConfig cfg;
  DriveController dc(cfg);
  dc.set_path(straight_path(25));
  Plant p;
  p.s.y = 5.0;  // large error demands saturated steering
  ActuatorCommand prev{};
  bool ok = true;
  for (int k = 0; k < 500; ++k) {
    const auto c = dc.step(p.s, k * 0.01, 0.01);
    ok &= std::fabs(c.steer) <= cfg.limits.steer_max + 1e-12;
    ok &= std::fabs(c.steer - prev.steer) <= cfg.limits.steer_rate_max * 0.01 + 1e-12;
    ok &= c.throttle - prev.throttle <= cfg.limits.throttle_rate_up * 0.01 + 1e-12;
    ok &= !(c.throttle > 0 && c.brake > 0);
    prev = c;
    p.step(c, 0.01);
  }
  CHECK(ok);
}

TEST(Controller, CanEnableRequired) {
  DriveConfig cfg;
  cfg.use_can = true;
  DriveController dc(cfg);
  dc.set_path(straight_path(10));
  Plant p;
  const auto c = dc.step(p.s, 0.0, 0.01);
  CHECK(dc.state() == DriveState::Disabled);
  CHECK_NEAR(c.throttle, 0.0, 0);
}

TEST(Controller, EStopViaCan) {
  DriveConfig cfg;
  cfg.use_can = true;
  DriveController dc(cfg);
  dc.set_path(straight_path(20));
  Plant p;
  p.s.v = 20;
  Bus bus;
  double t = 0;
  int k = 0;
  for (; k < 300; ++k, t += 0.01) {
    bus.tick(dc, t, k);
    p.step(dc.step(p.s, t, 0.01), 0.01);
  }
  REQUIRE(dc.state() == DriveState::Active);
  bus.estop = true;
  const double x0 = p.s.x;
  for (; k < 1500 && p.s.v > 0.01; ++k, t += 0.01) {
    bus.tick(dc, t, k);
    p.step(dc.step(p.s, t, 0.01), 0.01);
  }
  CHECK(p.s.v < 0.05);
  CHECK(p.s.x - x0 < 35.0);  // 20 m/s: ideal 25 m at 8 m/s^2 plus ramp/lag
  CHECK(dc.state() == DriveState::EmergencyStop || dc.state() == DriveState::Stopped);
}

TEST(Controller, HeartbeatLossStopsCar) {
  DriveConfig cfg;
  cfg.use_can = true;
  DriveController dc(cfg);
  dc.set_path(straight_path(15));
  Plant p;
  p.s.v = 15;
  Bus bus;
  double t = 0;
  for (int k = 0; k < 1500; ++k, t += 0.01) {
    if (k == 300) bus.hb = false;
    bus.tick(dc, t, k);
    p.step(dc.step(p.s, t, 0.01), 0.01);
  }
  CHECK(p.s.v < 0.05);
  CHECK(dc.debug().reasons & stop_reason::kEStopHeartbeat);
}

TEST(Controller, PlannerLossControlledStop) {
  DriveConfig cfg;
  cfg.use_can = true;
  DriveController dc(cfg);
  dc.set_path(straight_path(15));
  Plant p;
  p.s.v = 15;
  Bus bus;
  double t = 0;
  double min_acc = 0, v_prev = 15;
  for (int k = 0; k < 2000; ++k, t += 0.01) {
    if (k == 300) bus.cmd = false;
    bus.tick(dc, t, k);
    p.step(dc.step(p.s, t, 0.01), 0.01);
    if (k > 300) min_acc = std::min(min_acc, (p.s.v - v_prev) / 0.01);
    v_prev = p.s.v;
  }
  CHECK(p.s.v < 0.05);
  CHECK(min_acc > -4.5);  // controlled (about 3 m/s^2), not an emergency stop
  CHECK(dc.debug().reasons & stop_reason::kCommandTimeout);
}

TEST(Controller, TransmitsActuatorAndStatusFrames) {
  DriveConfig cfg;
  DriveController dc(cfg);
  dc.set_path(straight_path(10));
  Plant p;
  int act = 0, status = 0;
  for (int k = 0; k < 100; ++k) {
    dc.step(p.s, k * 0.01, 0.01);
    CanFrame f;
    while (dc.pop_tx(f)) {
      if (f.id == canid::kActuatorCmd) ++act;
      if (f.id == canid::kStatus) ++status;
      CHECK(check(f));
    }
  }
  CHECK_EQ(act, 100);
  CHECK_EQ(status, 50);
}

TEST(Controller, NaiveIgnoresHeartbeatLoss) {
  DriveConfig cfg;
  cfg.use_can = true;
  NaiveController nc(cfg);
  nc.set_path(straight_path(15));
  Plant p;
  p.s.v = 15;
  Bus bus;
  double t = 0;
  for (int k = 0; k < 1000; ++k, t += 0.01) {
    if (k == 300) bus.hb = false;
    bus.tick(nc, t, k);
    p.step(nc.step(p.s, t, 0.01), 0.01);
  }
  CHECK(p.s.v > 10.0);  // the baseline keeps driving
}

TEST(Controller, InvalidStateTriggersEmergency) {
  DriveConfig cfg;
  DriveController dc(cfg);
  dc.set_path(straight_path(10));
  VehicleState s;
  s.v = 10;
  dc.step(s, 0.0, 0.01);
  s.v = std::nan("");
  dc.step(s, 0.01, 0.01);
  CHECK(dc.state() == DriveState::EmergencyStop);
}

TEST_MAIN()
