// HIL-style replay harness: feeds a recorded CAN log (candump -L format,
// "(t) can0 ID#DATA") into the drive controller at the recorded
// timestamps, closes the loop with a simple kinematic vehicle with
// first-order actuators, and writes every frame the controller transmits
// to an output log in the same format.
//
//   dc_replay --log in.log --out out.log [--naive] [--path straight|circle] [--v0 m/s] [--t-end s]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "drivecontrol/controller.hpp"

using namespace dc;

namespace {

struct LogFrame {
  double t;
  CanFrame f;
};

std::vector<LogFrame> read_log(const std::string& path) {
  std::vector<LogFrame> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] != '(') continue;
    LogFrame lf{};
    char iface[32];
    char body[64];
    if (std::sscanf(line.c_str(), "(%lf) %31s %63s", &lf.t, iface, body) != 3) continue;
    const char* hash = std::strchr(body, '#');
    if (!hash) continue;
    lf.f.id = static_cast<uint32_t>(std::strtoul(std::string(body, static_cast<size_t>(hash - body)).c_str(), nullptr, 16));
    const std::string hex(hash + 1);
    for (size_t i = 0; i + 1 < hex.size() && i / 2 < 8; i += 2) {
      lf.f.data[i / 2] = static_cast<uint8_t>(std::strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
    }
    out.push_back(lf);
  }
  return out;
}

void write_frame(std::ofstream& o, double t, const CanFrame& f) {
  char buf[96];
  int n = std::snprintf(buf, sizeof buf, "(%.6f) vcan0 %03X#", t, f.id);
  for (int i = 0; i < 8; ++i) n += std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), "%02X", f.data[static_cast<size_t>(i)]);
  o << buf << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::string log_in, log_out = "replay_out.log", path_kind = "straight";
  bool naive = false;
  double v0 = 0.0, t_end = -1.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--log" && i + 1 < argc) log_in = argv[++i];
    else if (a == "--out" && i + 1 < argc) log_out = argv[++i];
    else if (a == "--naive") naive = true;
    else if (a == "--path" && i + 1 < argc) path_kind = argv[++i];
    else if (a == "--v0" && i + 1 < argc) v0 = std::atof(argv[++i]);
    else if (a == "--t-end" && i + 1 < argc) t_end = std::atof(argv[++i]);
  }
  if (log_in.empty()) {
    std::fprintf(stderr, "usage: dc_replay --log in.log [--out out.log] [--naive] [--path straight|circle] [--v0 v]\n");
    return 2;
  }
  const auto frames = read_log(log_in);
  if (t_end < 0) t_end = frames.empty() ? 1.0 : frames.back().t + 3.0;

  // reference path
  std::vector<double> px, py, pv;
  for (int i = 0; i <= 4000; ++i) {
    const double s = 0.5 * i;
    if (path_kind == "circle") {
      const double r = 60.0, th = s / r;
      px.push_back(r * std::sin(th));
      py.push_back(r * (1.0 - std::cos(th)));
    } else {
      px.push_back(s);
      py.push_back(0.0);
    }
    pv.push_back(20.0);
  }
  DriveConfig cfg;
  cfg.use_can = true;
  auto ctrl = make_controller(cfg, naive);
  ctrl->set_path(Path(px, py, pv));

  // kinematic plant with actuator lags
  VehicleState s;
  s.v = v0;
  double steer = 0, drive = 0, brake = 0;
  const double dt = 0.01;
  const VehicleParams vp;
  std::ofstream out(log_out);
  size_t next = 0;
  double dist = 0, t_brake = -1, t_stop = -1, d_brake = 0, t_state = -1;
  for (double t = 0.0; t <= t_end + 1e-9; t += dt) {
    while (next < frames.size() && frames[next].t <= t + 1e-9) {
      ctrl->on_can(frames[next].f, frames[next].t);
      ++next;
    }
    const ActuatorCommand c = ctrl->step(s, t, dt);
    CanFrame f;
    while (ctrl->pop_tx(f)) {
      write_frame(out, t, f);
      StatusMsg m;
      if (f.id == canid::kStatus && decode(f, m) && (m.state == 2 || m.state == 3) && t_state < 0) t_state = t;
      ActuatorCommand a;
      if (f.id == canid::kActuatorCmd && decode(f, a) && a.brake > 0.05 && t_brake < 0) {
        t_brake = t;  // first braking request seen on the bus
        d_brake = dist;
      }
    }
    // plant: 1 ms sub-steps
    for (int k = 0; k < 10; ++k) {
      const double h = dt / 10;
      steer += (c.steer - steer) * h / 0.08;
      drive += (c.throttle - drive) * h / 0.15;
      brake += (c.brake - brake) * h / 0.08;
      double f_long = drive * vp.drive_force_max - brake * vp.brake_force_max * (s.v > 0.01 ? 1.0 : 0.0) - vp.resistance(s.v);
      s.v = std::max(0.0, s.v + f_long / vp.mass * h);
      s.yaw += s.v / vp.wheelbase * std::tan(steer) * h;
      s.x += s.v * std::cos(s.yaw) * h;
      s.y += s.v * std::sin(s.yaw) * h;
      dist += s.v * h;
    }
    s.steer = steer;
    s.yaw_rate = s.v / vp.wheelbase * std::tan(steer);
    if (t_brake >= 0 && t_stop < 0 && s.v < 0.05) t_stop = t;
  }
  std::printf("frames_in=%zu stop_state_at=%.3f first_brake_at=%.3f stopped_at=%.3f stop_distance=%.2f final_speed=%.3f\n",
              frames.size(), t_state, t_brake, t_stop, t_brake >= 0 && t_stop >= 0 ? dist - d_brake : -1.0, s.v);
  return 0;
}
