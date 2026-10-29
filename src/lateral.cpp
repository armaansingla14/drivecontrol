#include "drivecontrol/lateral.hpp"

#include <cmath>

namespace dc {

double PurePursuit::steer(const Path& path, const VehicleState& s, LateralDebug* dbg) {
  const auto pr = path.project(s.x, s.y, hint_);
  hint_ = pr.index;
  const double ld = clamp(cfg_.ld0 + cfg_.k_v * s.v, cfg_.ld_min, cfg_.ld_max);
  PathPoint target = path.at(pr.s + ld);
  // near the end of the path, extend along the final tangent
  const double remaining = path.length() - pr.s;
  if (remaining < ld) {
    const double extra = ld - remaining;
    target.x += extra * std::cos(target.yaw);
    target.y += extra * std::sin(target.yaw);
  }
  const double alpha = wrap_angle(std::atan2(target.y - s.y, target.x - s.x) - s.yaw);
  const double dist = std::hypot(target.x - s.x, target.y - s.y);
  const double delta = std::atan2(2.0 * L_ * std::sin(alpha), std::max(dist, 1e-3));
  if (dbg) {
    dbg->lateral_error = pr.lateral_error;
    dbg->heading_error = wrap_angle(pr.yaw - s.yaw);
    dbg->lookahead = ld;
    dbg->s = pr.s;
  }
  return delta;
}

double Stanley::steer(const Path& path, const VehicleState& s, LateralDebug* dbg) {
  const double fx = s.x + L_ * std::cos(s.yaw);
  const double fy = s.y + L_ * std::sin(s.yaw);
  const auto pr = path.project(fx, fy, hint_);
  hint_ = pr.index;
  const double psi_e = wrap_angle(pr.yaw - s.yaw);
  // lateral_error > 0 means the axle is left of the path -> steer right
  double delta = psi_e + std::atan2(-cfg_.k * pr.lateral_error, cfg_.k_soft + std::fabs(s.v));
  delta += cfg_.k_understeer * s.v * s.v * pr.kappa;
  delta -= cfg_.k_yaw_rate * (s.yaw_rate - s.v * pr.kappa);
  if (dbg) {
    dbg->lateral_error = pr.lateral_error;
    dbg->heading_error = psi_e;
    dbg->lookahead = 0.0;
    dbg->s = pr.s;
  }
  return delta;
}

}  // namespace dc
