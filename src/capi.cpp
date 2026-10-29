#include "drivecontrol/capi.h"

#include <cstring>
#include <vector>

#include "drivecontrol/controller.hpp"

using namespace dc;

namespace {
DriveConfig to_cpp(const dc_config_t& c) {
  DriveConfig d;
  d.lateral = c.lateral == 0 ? LateralMode::PurePursuit : LateralMode::Stanley;
  d.speed.pid.anti_windup = c.anti_windup == 0 ? AntiWindup::None
                                               : (c.anti_windup == 1 ? AntiWindup::Clamp : AntiWindup::BackCalculation);
  d.use_can = c.use_can != 0;
  d.pure_pursuit.ld0 = c.pp_ld0;
  d.pure_pursuit.k_v = c.pp_kv;
  d.pure_pursuit.ld_min = c.pp_ld_min;
  d.pure_pursuit.ld_max = c.pp_ld_max;
  d.stanley.k = c.st_k;
  d.stanley.k_soft = c.st_k_soft;
  d.stanley.k_yaw_rate = c.st_k_yaw_rate;
  d.stanley.curvature_ff = c.st_curvature_ff != 0;
  d.speed.pid.kp = c.v_kp;
  d.speed.pid.ki = c.v_ki;
  d.speed.pid.kd = c.v_kd;
  d.limits.steer_max = c.steer_max;
  d.limits.steer_rate_max = c.steer_rate_max;
  d.limits.estop_brake_rate = c.estop_brake_rate;
  d.safety.max_lateral_error = c.max_lateral_error;
  d.safety.controlled_decel = c.controlled_decel;
  return d;
}

struct Handle {
  std::unique_ptr<IController> ctrl;
};

Path make_path(const double* x, const double* y, const double* v, int n) {
  return Path(std::vector<double>(x, x + n), std::vector<double>(y, y + n), std::vector<double>(v, v + n));
}

VehicleState to_state(const dc_state_t* s) {
  VehicleState v;
  v.x = s->x;
  v.y = s->y;
  v.yaw = s->yaw;
  v.v = s->v;
  v.steer = s->steer;
  v.yaw_rate = s->yaw_rate;
  v.valid = s->valid != 0;
  return v;
}
}  // namespace

extern "C" {

void dc_default_config(dc_config_t* c) {
  const DriveConfig d;
  std::memset(c, 0, sizeof(*c));
  c->controller = 0;
  c->lateral = 1;
  c->anti_windup = 2;
  c->use_can = 0;
  c->pp_ld0 = d.pure_pursuit.ld0;
  c->pp_kv = d.pure_pursuit.k_v;
  c->pp_ld_min = d.pure_pursuit.ld_min;
  c->pp_ld_max = d.pure_pursuit.ld_max;
  c->st_k = d.stanley.k;
  c->st_k_soft = d.stanley.k_soft;
  c->st_k_yaw_rate = d.stanley.k_yaw_rate;
  c->st_curvature_ff = d.stanley.curvature_ff ? 1 : 0;
  c->v_kp = d.speed.pid.kp;
  c->v_ki = d.speed.pid.ki;
  c->v_kd = d.speed.pid.kd;
  c->steer_max = d.limits.steer_max;
  c->steer_rate_max = d.limits.steer_rate_max;
  c->estop_brake_rate = d.limits.estop_brake_rate;
  c->max_lateral_error = d.safety.max_lateral_error;
  c->controlled_decel = d.safety.controlled_decel;
}

void* dc_create(const dc_config_t* cfg) {
  auto* h = new Handle;
  h->ctrl = make_controller(to_cpp(*cfg), cfg->controller == 1);
  return h;
}

void dc_destroy(void* h) { delete static_cast<Handle*>(h); }

int dc_set_path(void* h, const double* x, const double* y, const double* v, int n) {
  if (n < 2) return -1;
  static_cast<Handle*>(h)->ctrl->set_path(make_path(x, y, v, n));
  return 0;
}

void dc_can_rx(void* h, uint32_t id, const uint8_t* data, double t) {
  CanFrame f;
  f.id = id;
  std::memcpy(f.data.data(), data, 8);
  static_cast<Handle*>(h)->ctrl->on_can(f, t);
}

void dc_step(void* h, const dc_state_t* s, double t, double dt, dc_cmd_t* out) {
  const ActuatorCommand c = static_cast<Handle*>(h)->ctrl->step(to_state(s), t, dt);
  out->steer = c.steer;
  out->throttle = c.throttle;
  out->brake = c.brake;
}

int dc_can_tx(void* h, uint32_t* id, uint8_t* data) {
  CanFrame f;
  if (!static_cast<Handle*>(h)->ctrl->pop_tx(f)) return 0;
  *id = f.id;
  std::memcpy(data, f.data.data(), 8);
  return 1;
}

void dc_get_debug(void* h, dc_debug_t* d) {
  const DriveDebug g = static_cast<Handle*>(h)->ctrl->debug();
  d->state = static_cast<int32_t>(g.state);
  d->reasons = g.reasons;
  d->lateral_error = g.lat.lateral_error;
  d->heading_error = g.lat.heading_error;
  d->lookahead = g.lat.lookahead;
  d->s = g.lat.s;
  d->v_ref = g.v_ref;
  d->a_des = g.a_des;
  d->raw_steer = g.raw.steer;
  d->raw_throttle = g.raw.throttle;
  d->raw_brake = g.raw.brake;
}

void dc_encode_command(double speed, int enable, int reset, uint8_t counter, uint8_t* out8) {
  CommandMsg m;
  m.target_speed = speed;
  m.enable = enable != 0;
  m.reset_request = reset != 0;
  const CanFrame f = encode(m, counter);
  std::memcpy(out8, f.data.data(), 8);
}

void dc_encode_estop(int estop, uint8_t counter, uint8_t* out8) {
  EStopMsg m;
  m.estop = estop != 0;
  const CanFrame f = encode(m, counter);
  std::memcpy(out8, f.data.data(), 8);
}

int dc_decode_actuator(const uint8_t* data8, dc_cmd_t* out) {
  CanFrame f;
  f.id = canid::kActuatorCmd;
  std::memcpy(f.data.data(), data8, 8);
  ActuatorCommand c;
  if (!decode(f, c)) return 0;
  out->steer = c.steer;
  out->throttle = c.throttle;
  out->brake = c.brake;
  return 1;
}

int dc_decode_status(const uint8_t* data8, int32_t* state, double* speed, int32_t* faults) {
  CanFrame f;
  f.id = canid::kStatus;
  std::memcpy(f.data.data(), data8, 8);
  StatusMsg m;
  if (!decode(f, m)) return 0;
  *state = m.state;
  *speed = m.speed;
  *faults = m.faults;
  return 1;
}

void dc_pid_run(const double* g, const double* ref, const double* meas, const double* ff, int n, double dt,
                double* out) {
  PidConfig c;
  c.kp = g[0];
  c.ki = g[1];
  c.kd = g[2];
  c.kff = g[3];
  c.out_min = g[4];
  c.out_max = g[5];
  c.d_tau = g[6];
  c.anti_windup = g[7] < 0.5 ? AntiWindup::None : (g[7] < 1.5 ? AntiWindup::Clamp : AntiWindup::BackCalculation);
  c.kt = g[8];
  Pid pid(c);
  for (int i = 0; i < n; ++i) out[i] = pid.update(ref[i], meas[i], ff[i], dt);
}

double dc_lateral_once(int mode, const double* cfg4, const double* px, const double* py, const double* pv, int n,
                       const dc_state_t* s, double wheelbase) {
  const Path path = make_path(px, py, pv, n);
  const VehicleState vs = to_state(s);
  if (mode == 0) {
    PurePursuitConfig c{cfg4[0], cfg4[1], cfg4[2], cfg4[3]};
    PurePursuit pp(c, wheelbase);
    return pp.steer(path, vs);
  }
  StanleyConfig c{cfg4[0], cfg4[1], cfg4[2], cfg4[3] > 0.5};
  Stanley st(c, wheelbase);
  return st.steer(path, vs);
}

}  // extern "C"
