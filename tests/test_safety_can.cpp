#include "drivecontrol/can_interface.hpp"
#include "drivecontrol/safety.hpp"
#include "testfw.hpp"

using namespace dc;

namespace {
SafetyManager::Inputs ok_inputs(double v = 10.0) {
  SafetyManager::Inputs in;
  in.enable = true;
  in.v = v;
  return in;
}
}  // namespace

TEST(Safety, StartsDisabledAndEnablesOnEdge) {
  SafetyManager sm(SafetyConfig{});
  CHECK(sm.state() == DriveState::Disabled);
  sm.update(ok_inputs(), 0.0);
  CHECK(sm.state() == DriveState::Active);
}

TEST(Safety, NoEnableWhileEStopActive) {
  SafetyManager sm(SafetyConfig{});
  auto in = ok_inputs();
  in.estop = true;
  sm.update(in, 0.0);
  CHECK(sm.state() == DriveState::Disabled);
}

TEST(Safety, EStopBitToEmergency) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.estop = true;
  sm.update(in, 0.1);
  CHECK(sm.state() == DriveState::EmergencyStop);
  CHECK(sm.reasons() & stop_reason::kEStopRequest);
}

TEST(Safety, HeartbeatLossToEmergency) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.estop_link_lost = true;
  sm.update(in, 0.1);
  CHECK(sm.state() == DriveState::EmergencyStop);
  CHECK(sm.reasons() & stop_reason::kEStopHeartbeat);
}

TEST(Safety, PlannerLossToControlledStop) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.command_link_lost = true;
  sm.update(in, 0.1);
  CHECK(sm.state() == DriveState::ControlledStop);
}

TEST(Safety, TrackingErrorToControlledStop) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.lateral_error = 3.5;
  sm.update(in, 0.1);
  CHECK(sm.state() == DriveState::ControlledStop);
  CHECK(sm.reasons() & stop_reason::kTrackingError);
}

TEST(Safety, ControlledStopEscalates) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.command_link_lost = true;
  sm.update(in, 0.1);
  in.estop = true;
  sm.update(in, 0.2);
  CHECK(sm.state() == DriveState::EmergencyStop);
}

TEST(Safety, StoppedAfterStandstillHold) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.estop = true;
  sm.update(in, 0.1);
  in.v = 0.0;
  sm.update(in, 1.0);
  CHECK(sm.state() == DriveState::EmergencyStop);
  sm.update(in, 1.6);
  CHECK(sm.state() == DriveState::Stopped);
}

TEST(Safety, ResetOnlyWhenEStopReleased) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs(0.0);
  in.estop = true;
  sm.update(in, 0.1);
  sm.update(in, 0.2);
  sm.update(in, 0.8);
  REQUIRE(sm.state() == DriveState::Stopped);
  in.reset_request = true;
  sm.update(in, 0.9);
  CHECK(sm.state() == DriveState::Stopped);  // e-stop still pressed
  in.estop = false;
  in.reset_request = false;
  sm.update(in, 1.0);
  in.reset_request = true;
  sm.update(in, 1.1);
  CHECK(sm.state() == DriveState::Disabled);
  in.enable = false;
  sm.update(in, 1.2);
  in.enable = true;
  sm.update(in, 1.3);
  CHECK(sm.state() == DriveState::Active);
}

TEST(Safety, InvalidStateIsEmergency) {
  SafetyManager sm(SafetyConfig{});
  sm.update(ok_inputs(), 0.0);
  auto in = ok_inputs();
  in.state_valid = false;
  sm.update(in, 0.1);
  CHECK(sm.state() == DriveState::EmergencyStop);
}

TEST(Can, CommandRoundTrip) {
  CommandMsg m;
  m.target_speed = 13.37;
  m.enable = true;
  const CanFrame f = encode(m, 5);
  CHECK(check(f));
  CommandMsg o;
  REQUIRE(decode(f, o));
  CHECK_NEAR(o.target_speed, 13.37, 1e-9);
  CHECK(o.enable);
  CHECK(!o.reset_request);
}

TEST(Can, ActuatorRoundTrip) {
  ActuatorCommand c{-0.1234, 0.5, 0.0};
  const CanFrame f = encode(c, 1);
  ActuatorCommand o;
  REQUIRE(decode(f, o));
  CHECK_NEAR(o.steer, -0.1234, 1e-4);
  CHECK_NEAR(o.throttle, 0.5, 0.005);
}

TEST(Can, StatusRoundTrip) {
  StatusMsg s;
  s.state = 3;
  s.speed = 21.5;
  s.steer = 0.02;
  s.faults = 0x05;
  StatusMsg o;
  REQUIRE(decode(encode(s, 2), o));
  CHECK_EQ(o.state, 3);
  CHECK_NEAR(o.speed, 21.5, 0.01);
  CHECK_EQ(o.faults, 0x05);
}

TEST(Can, CrcRejectsCorruption) {
  CanFrame f = encode(EStopMsg{true}, 3);
  for (int byte = 0; byte < 7; ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      CanFrame g = f;
      g.data[static_cast<size_t>(byte)] ^= static_cast<uint8_t>(1 << bit);
      CHECK(!check(g));
    }
  }
}

TEST(Can, WrongIdRejected) {
  CanFrame f = encode(EStopMsg{false}, 3);
  CommandMsg m;
  CHECK(!decode(f, m));
}

TEST(RxSupervisor, TimeoutAndCounter) {
  RxSupervisor rx(0.1);
  CHECK(rx.timed_out(0.0));  // never received
  CHECK(rx.accept(encode(EStopMsg{}, 0), 0.0));
  CHECK(!rx.timed_out(0.05));
  CHECK(rx.timed_out(0.11));
  CHECK(!rx.accept(encode(EStopMsg{}, 0), 0.12));  // repeated counter
  CHECK(rx.accept(encode(EStopMsg{}, 1), 0.14));
  CHECK(!rx.accept(encode(EStopMsg{}, 9), 0.16));  // jump
  CHECK_EQ(rx.rejected(), 2u);
}

TEST(RxSupervisor, CorruptFrameDoesNotRefresh) {
  RxSupervisor rx(0.1);
  rx.accept(encode(EStopMsg{}, 0), 0.0);
  CanFrame bad = encode(EStopMsg{}, 1);
  bad.data[7] ^= 0xFF;
  CHECK(!rx.accept(bad, 0.09));
  CHECK(rx.timed_out(0.15));
}

TEST_MAIN()
