#pragma once
#include <deque>
#include <memory>

#include "drivecontrol/can_interface.hpp"
#include "drivecontrol/lateral.hpp"
#include "drivecontrol/limits.hpp"
#include "drivecontrol/longitudinal.hpp"
#include "drivecontrol/path.hpp"
#include "drivecontrol/safety.hpp"
#include "drivecontrol/types.hpp"

namespace dc {

enum class LateralMode { PurePursuit = 0, Stanley = 1 };

struct DriveConfig {
  LateralMode lateral = LateralMode::Stanley;
  VehicleParams vehicle{};
  ActuatorLimits limits{};
  PurePursuitConfig pure_pursuit{};
  StanleyConfig stanley{};
  SpeedControlConfig speed{};
  SafetyConfig safety{};
  bool use_can = false;          ///< false: always enabled, speed from path profile only
  double estop_timeout_s = 0.1;  ///< DC_EStop heartbeat (20 ms) supervision
  double command_timeout_s = 0.2;  ///< DC_Command (50 ms) supervision
  double status_period_s = 0.02;
};

struct DriveDebug {
  DriveState state = DriveState::Disabled;
  uint8_t reasons = 0;
  LateralDebug lat;
  double v_ref = 0;
  double a_des = 0;
  ActuatorCommand raw;  ///< before actuator shaping
};

/// Common interface for the controller under test and the naive baseline.
class IController {
 public:
  virtual ~IController() = default;
  virtual void set_path(const Path& p) = 0;
  virtual void on_can(const CanFrame& f, double t) = 0;
  virtual ActuatorCommand step(const VehicleState& s, double t, double dt) = 0;
  virtual bool pop_tx(CanFrame& f) = 0;
  virtual DriveDebug debug() const = 0;
};

/// Full drive controller: lateral (pure pursuit or Stanley), PID speed
/// control with feedforward and anti-windup, actuator saturation and rate
/// limits, CAN command interface with E2E/timeout supervision and the
/// emergency-stop state machine.
class DriveController : public IController {
 public:
  explicit DriveController(const DriveConfig& cfg);
  void set_path(const Path& p) override;
  void on_can(const CanFrame& f, double t) override;
  ActuatorCommand step(const VehicleState& s, double t, double dt) override;
  bool pop_tx(CanFrame& f) override;
  DriveDebug debug() const override { return dbg_; }
  DriveState state() const { return safety_.state(); }

 private:
  DriveConfig cfg_;
  Path path_;
  PurePursuit pp_;
  Stanley stanley_;
  SpeedController speed_;
  ActuatorShaper shaper_;
  SafetyManager safety_;
  RxSupervisor rx_estop_, rx_cmd_;
  CommandMsg cmd_{};
  bool estop_bit_ = false;
  std::deque<CanFrame> tx_;
  double last_status_t_ = -1e9;
  uint8_t ctr_status_ = 0, ctr_act_ = 0;
  DriveDebug dbg_;
  ActuatorCommand last_{};
};

/// Baseline: what a quick first implementation looks like. PI speed loop
/// mapped directly to pedals (no feedforward, no anti-windup), proportional
/// lateral law on cross-track and heading error (no look-ahead, no
/// curvature feedforward), steering clamped but not rate limited, e-stop
/// bit -> full brake step, no heartbeat or command-timeout supervision.
class NaiveController : public IController {
 public:
  struct Gains {
    double kp_v = 0.5, ki_v = 0.1;
    double k_e = 0.1, k_psi = 1.0;  // best of a 4x4 grid on the evaluation tracks
  };
  NaiveController(const DriveConfig& cfg, const Gains& g);
  explicit NaiveController(const DriveConfig& cfg) : NaiveController(cfg, Gains{}) {}
  void set_path(const Path& p) override { path_ = p; }
  void on_can(const CanFrame& f, double t) override;
  ActuatorCommand step(const VehicleState& s, double t, double dt) override;
  bool pop_tx(CanFrame& f) override;
  DriveDebug debug() const override { return dbg_; }

 private:
  DriveConfig cfg_;
  Gains g_;
  Path path_;
  double integ_ = 0;
  bool estop_ = false;
  bool enabled_ = false;
  double target_speed_ = 1e9;
  size_t hint_ = static_cast<size_t>(-1);
  std::deque<CanFrame> tx_;
  uint8_t ctr_ = 0;
  DriveDebug dbg_;
};

std::unique_ptr<IController> make_controller(const DriveConfig& cfg, bool naive);

}  // namespace dc
