#include "drivecontrol/path.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "drivecontrol/types.hpp"

namespace dc {

Path::Path(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& v) {
  const size_t n = x.size();
  pts_.resize(n);
  double s = 0;
  for (size_t i = 0; i < n; ++i) {
    pts_[i].x = x[i];
    pts_[i].y = y[i];
    pts_[i].v_ref = i < v.size() ? v[i] : 0.0;
    if (i > 0) s += std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
    pts_[i].s = s;
  }
  for (size_t i = 0; i < n; ++i) {
    const size_t a = i == 0 ? 0 : i - 1;
    const size_t b = i + 1 < n ? i + 1 : n - 1;
    pts_[i].yaw = std::atan2(pts_[b].y - pts_[a].y, pts_[b].x - pts_[a].x);
  }
  for (size_t i = 0; i < n; ++i) {
    const size_t a = i == 0 ? 0 : i - 1;
    const size_t b = i + 1 < n ? i + 1 : n - 1;
    const double ds = pts_[b].s - pts_[a].s;
    pts_[i].kappa = ds > 1e-9 ? wrap_angle(pts_[b].yaw - pts_[a].yaw) / ds : 0.0;
    const double dv2 = pts_[b].v_ref * pts_[b].v_ref - pts_[a].v_ref * pts_[a].v_ref;
    pts_[i].a_ref = ds > 1e-9 ? 0.5 * dv2 / ds : 0.0;
  }
}

Path::Projection Path::project(double px, double py, size_t hint, size_t window) const {
  Projection best;
  if (pts_.size() < 2) return best;
  size_t lo = 0, hi = pts_.size() - 1;
  if (hint != SIZE_MAX) {
    lo = hint > window ? hint - window : 0;
    hi = hint + window < pts_.size() - 1 ? hint + window : pts_.size() - 1;
  }
  // segment distances; on a global search, near-ties (overlapping laps,
  // crossings) resolve to the earliest segment within 0.5 m of the best
  auto seg_dist2 = [&](size_t i, double& t_out, double& qx, double& qy) {
    const PathPoint& a = pts_[i];
    const PathPoint& b = pts_[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((px - a.x) * dx + (py - a.y) * dy) / l2 : 0.0;
    t = clamp(t, 0.0, 1.0);
    qx = a.x + t * dx;
    qy = a.y + t * dy;
    t_out = t;
    return (px - qx) * (px - qx) + (py - qy) * (py - qy);
  };
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = lo; i < hi; ++i) {
    double t, qx, qy;
    const double d2 = seg_dist2(i, t, qx, qy);
    if (d2 < best_d2) {
      best_d2 = d2;
      best.index = i;
      best.t = t;
      best.x = qx;
      best.y = qy;
    }
  }
  if (hint == SIZE_MAX) {
    // earliest local minimum within 0.5 m of the global best
    std::vector<double> d(hi - lo);
    for (size_t i = lo; i < hi; ++i) {
      double t, qx, qy;
      d[i - lo] = seg_dist2(i, t, qx, qy);
    }
    const double lim = std::sqrt(best_d2) + 0.5;
    for (size_t k = 0; k < d.size(); ++k) {
      const bool local_min = (k == 0 || d[k] <= d[k - 1]) && (k + 1 == d.size() || d[k] <= d[k + 1]);
      if (local_min && d[k] <= lim * lim) {
        double t, qx, qy;
        best_d2 = seg_dist2(lo + k, t, qx, qy);
        best.index = lo + k;
        best.t = t;
        best.x = qx;
        best.y = qy;
        break;
      }
    }
  }
  const PathPoint& a = pts_[best.index];
  const PathPoint& b = pts_[best.index + 1];
  best.s = a.s + best.t * (b.s - a.s);
  const double seg_yaw = std::atan2(b.y - a.y, b.x - a.x);
  best.yaw = seg_yaw;
  best.kappa = a.kappa + best.t * (b.kappa - a.kappa);
  best.v_ref = a.v_ref + best.t * (b.v_ref - a.v_ref);
  best.a_ref = a.a_ref + best.t * (b.a_ref - a.a_ref);
  // sign: cross product of path tangent and (p - q)
  const double cx = std::cos(seg_yaw), cy = std::sin(seg_yaw);
  const double ex = px - best.x, ey = py - best.y;
  const double cross = cx * ey - cy * ex;
  best.lateral_error = (cross >= 0 ? 1.0 : -1.0) * std::sqrt(best_d2);
  return best;
}

PathPoint Path::at(double s) const {
  if (pts_.empty()) return {};
  if (s <= 0) return pts_.front();
  if (s >= pts_.back().s) return pts_.back();
  // binary search on s
  size_t lo = 0, hi = pts_.size() - 1;
  while (hi - lo > 1) {
    const size_t mid = (lo + hi) / 2;
    if (pts_[mid].s <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const PathPoint& a = pts_[lo];
  const PathPoint& b = pts_[hi];
  const double t = (s - a.s) / (b.s - a.s);
  PathPoint p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.yaw = std::atan2(b.y - a.y, b.x - a.x);
  p.kappa = a.kappa + t * (b.kappa - a.kappa);
  p.s = s;
  p.v_ref = a.v_ref + t * (b.v_ref - a.v_ref);
  p.a_ref = a.a_ref + t * (b.a_ref - a.a_ref);
  return p;
}

}  // namespace dc
