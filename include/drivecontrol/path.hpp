#pragma once
#include <cstddef>
#include <vector>

namespace dc {

struct PathPoint {
  double x = 0, y = 0;
  double yaw = 0;    ///< tangent heading
  double kappa = 0;  ///< curvature, 1/m
  double s = 0;      ///< arc length
  double v_ref = 0;  ///< reference speed, m/s
  double a_ref = 0;  ///< reference acceleration along the path, m/s^2
};

/// Polyline reference path with arc length, heading, curvature and a speed
/// profile. Nearest-point search is windowed around the previous match so
/// it stays on the right branch of self-intersecting tracks.
class Path {
 public:
  Path() = default;
  Path(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& v);
  bool empty() const { return pts_.size() < 2; }
  size_t size() const { return pts_.size(); }
  const PathPoint& operator[](size_t i) const { return pts_[i]; }
  double length() const { return pts_.empty() ? 0.0 : pts_.back().s; }

  struct Projection {
    size_t index = 0;   ///< segment start index
    double t = 0;       ///< position along the segment [0,1]
    double s = 0;       ///< arc length of the projection
    double x = 0, y = 0;
    double yaw = 0;
    double kappa = 0;
    double v_ref = 0;
    double a_ref = 0;
    double lateral_error = 0;  ///< signed, positive = point is left of the path
  };
  /// Project (px, py) onto the path; `hint` is the previous segment index
  /// (use SIZE_MAX for a global search).
  Projection project(double px, double py, size_t hint, size_t window = 50) const;
  /// Point at arc length s (clamped to the ends).
  PathPoint at(double s) const;

 private:
  std::vector<PathPoint> pts_;
};

}  // namespace dc
