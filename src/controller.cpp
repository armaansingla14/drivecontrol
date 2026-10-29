#include "drivecontrol/controller.hpp"

#include <cmath>
#include <cstdint>

namespace dc {

DriveController::DriveController(const DriveConfig& cfg)
    : cfg_(cfg),
      pp_(cfg.pure_pursuit, cfg.vehicle.wheelbase),
      stanley_(cfg.stanley, cfg.vehicle.wheelbase),
      speed_(cfg.speed, cfg.vehicle),
      shaper_(cfg.limits),
      safety_(cfg.safety),
      rx_estop_(cfg.estop_timeout_s),
      rx_cmd_(cfg.command_timeout_s) {
  if (!cfg_.use_can) safety_.force_active();
}

void DriveController::set_path(const Path& p) {
  path_ = p;
  pp_.reset();
  stanley_.reset();
}

void DriveController::on_can(const CanFrame& f, double t) {
  if (f.id == canid::kEStop) {
    EStopMsg m;
    if (rx_estop_.accept(f, t) && decode(f, m)) estop_bit_ = m.estop;
  } else if (f.id == canid::kCommand) {
    CommandMsg m;
    if (rx_cmd_.accept(f, t) && decode(f, m)) cmd_ = m;
  }
}

ActuatorCommand DriveController::step(const VehicleState& s, double t, double dt) {
  const bool valid = s.valid && std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.v) && std::isfinite(s.yaw);

  // lateral control runs in every state except Disabled/Stopped so the car
  // keeps following the path while braking
  ActuatorCommand raw{};
  LateralDebug lat{};
  if (!path_.empty() && valid) {
    raw.steer = cfg_.lateral == LateralMode::PurePursuit ? pp_.steer(path_, s, &lat) : stanley_.steer(path_, s, &lat);
  }

  SafetyManager::Inputs si;
  si.v = valid ? s.v : 0.0;
  si.state_valid = valid;
  si.lateral_error = lat.lateral_error;
  if (cfg_.use_can) {
    si.enable = cmd_.enable;
    si.reset_request = cmd_.reset_request;
    si.estop = estop_bit_;
    si.estop_link_lost = rx_estop_.timed_out(t);
    si.command_link_lost = rx_cmd_.timed_out(t);
  } else {
    si.enable = true;
  }
  const DriveState st = safety_.update(si, t);

  double v_ref = 0.0, a_ref = 0.0;
  switch (st) {
    case DriveState::Active: {
      const PathPoint p = path_.empty() ? PathPoint{} : path_.at(lat.s);
      v_ref = p.v_ref;
      a_ref = p.a_ref;
      if (cfg_.use_can) v_ref = std::min(v_ref, cmd_.target_speed);
      const ActuatorCommand lon = speed_.step(v_ref, a_ref, s.v, dt);
      raw.throttle = lon.throttle;
      raw.brake = lon.brake;
      break;
    }
    case DriveState::ControlledStop: {
      const double el = t - safety_.entered_at();
      v_ref = std::max(0.0, safety_.speed_at_entry() - cfg_.safety.controlled_decel * el);
      a_ref = v_ref > 0 ? -cfg_.safety.controlled_decel : 0.0;
      const ActuatorCommand lon = speed_.step(v_ref, a_ref, s.v, dt);
      raw.throttle = 0.0;
      raw.brake = std::max(lon.brake, v_ref <= 0.0 ? 0.3 : 0.0);
      break;
    }
    case DriveState::EmergencyStop:
      raw.throttle = 0.0;
      raw.brake = 1.0;
      break;
    case DriveState::Stopped:
      raw.steer = last_.steer;
      raw.brake = 0.5;  // hold
      speed_.reset();
      break;
    case DriveState::Disabled:
      raw = ActuatorCommand{};
      raw.steer = valid ? s.steer : 0.0;
      speed_.reset();
      break;
  }

  const ActuatorCommand out = shaper_.apply(raw, dt, st == DriveState::EmergencyStop);
  last_ = out;
  dbg_.state = st;
  dbg_.reasons = safety_.reasons();
  dbg_.lat = lat;
  dbg_.v_ref = v_ref;
  dbg_.a_des = speed_.desired_accel();
  dbg_.raw = raw;

  // outgoing CAN: actuator command every step, status every 20 ms
  tx_.push_back(encode(out, ctr_act_++));
  if (t - last_status_t_ >= cfg_.status_period_s - 1e-9) {
    last_status_t_ = t;
    StatusMsg sm;
    sm.state = static_cast<uint8_t>(st);
    sm.speed = std::max(0.0, s.v);
    sm.steer = out.steer;
    sm.faults = safety_.reasons();
    tx_.push_back(encode(sm, ctr_status_++));
  }
  while (tx_.size() > 64) tx_.pop_front();
  return out;
}

bool DriveController::pop_tx(CanFrame& f) {
  if (tx_.empty()) return false;
  f = tx_.front();
  tx_.pop_front();
  return true;
}

NaiveController::NaiveController(const DriveConfig& cfg, const Gains& g) : cfg_(cfg), g_(g) {
  enabled_ = !cfg.use_can;
}

void NaiveController::on_can(const CanFrame& f, double) {
  // decodes whatever arrives; the last value is used forever
  if (f.id == canid::kEStop) {
    EStopMsg m;
    if (decode(f, m)) estop_ = m.estop;
  } else if (f.id == canid::kCommand) {
    CommandMsg m;
    if (decode(f, m)) {
      enabled_ = m.enable;
      target_speed_ = m.target_speed;
    }
  }
}

ActuatorCommand NaiveController::step(const VehicleState& s, double, double dt) {
  ActuatorCommand c;
  const auto pr = path_.project(s.x, s.y, hint_);
  hint_ = pr.index;
  const double psi_e = wrap_angle(pr.yaw - s.yaw);
  c.steer = clamp(-g_.k_e * pr.lateral_error + g_.k_psi * psi_e, -cfg_.limits.steer_max, cfg_.limits.steer_max);
  double v_ref = std::min(pr.v_ref, target_speed_);
  if (!enabled_) v_ref = 0.0;
  if (estop_) {
    c.throttle = 0.0;
    c.brake = 1.0;
  } else {
    const double e = v_ref - s.v;
    integ_ += g_.ki_v * e * dt;  // no anti-windup
    const double u = g_.kp_v * e + integ_;
    c.throttle = clamp(u, 0.0, 1.0);
    c.brake = clamp(-u, 0.0, 1.0);
  }
  dbg_.state = estop_ ? DriveState::EmergencyStop : DriveState::Active;
  dbg_.lat.lateral_error = pr.lateral_error;
  dbg_.lat.heading_error = psi_e;
  dbg_.lat.s = pr.s;
  dbg_.v_ref = v_ref;
  dbg_.raw = c;
  tx_.push_back(encode(c, ctr_++));
  while (tx_.size() > 64) tx_.pop_front();
  return c;
}

bool NaiveController::pop_tx(CanFrame& f) {
  if (tx_.empty()) return false;
  f = tx_.front();
  tx_.pop_front();
  return true;
}

std::unique_ptr<IController> make_controller(const DriveConfig& cfg, bool naive) {
  if (naive) return std::make_unique<NaiveController>(cfg);
  return std::make_unique<DriveController>(cfg);
}

}  // namespace dc
