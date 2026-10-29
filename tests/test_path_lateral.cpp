#include <cmath>
#include <cstdint>
#include <vector>

#include "drivecontrol/lateral.hpp"
#include "drivecontrol/path.hpp"
#include "testfw.hpp"

using namespace dc;

namespace {
Path straight(double len = 200, double v = 10) {
  std::vector<double> x, y, vv;
  for (double s = 0; s <= len; s += 0.5) {
    x.push_back(s);
    y.push_back(0);
    vv.push_back(v);
  }
  return Path(x, y, vv);
}
Path circle(double r, double laps = 1.0) {
  std::vector<double> x, y, v;
  for (double th = 0; th <= 2 * M_PI * laps; th += 0.5 / r) {
    x.push_back(r * std::sin(th));
    y.push_back(r * (1 - std::cos(th)));
    v.push_back(10);
  }
  return Path(x, y, v);
}
}  // namespace

TEST(Path, ArcLengthAndHeading) {
  const Path p = straight(100);
  CHECK_NEAR(p.length(), 100.0, 1e-9);
  CHECK_NEAR(p[10].yaw, 0.0, 1e-12);
  CHECK_NEAR(p[10].kappa, 0.0, 1e-12);
}

TEST(Path, CircleCurvature) {
  const Path p = circle(40);
  CHECK_NEAR(p[100].kappa, 1.0 / 40.0, 1e-3);
}

TEST(Path, ProjectionSignedError) {
  const Path p = straight();
  const auto left = p.project(50.2, 1.5, SIZE_MAX);
  CHECK_NEAR(left.lateral_error, 1.5, 1e-9);
  CHECK_NEAR(left.s, 50.2, 1e-9);
  const auto right = p.project(20.0, -0.7, SIZE_MAX);
  CHECK_NEAR(right.lateral_error, -0.7, 1e-9);
}

TEST(Path, AtInterpolates) {
  const Path p = straight();
  const auto q = p.at(33.3);
  CHECK_NEAR(q.x, 33.3, 1e-9);
  CHECK_NEAR(p.at(-5).x, 0.0, 1e-12);
  CHECK_NEAR(p.at(1e6).x, 200.0, 1e-9);
}

TEST(Path, OverlappingLapsPreferEarliest) {
  const Path p = circle(30, 1.25);
  const auto pr = p.project(1.0, 0.05, SIZE_MAX);
  CHECK(pr.s < 10.0);
}

TEST(Path, WindowedSearchFollowsHint) {
  const Path p = circle(30, 1.25);
  size_t hint = SIZE_MAX;
  double s_prev = 0;
  bool monotonic = true;
  for (double th = 0.01; th < 2 * M_PI * 1.2; th += 0.01) {
    const auto pr = p.project(30 * std::sin(th), 30 * (1 - std::cos(th)), hint);
    hint = pr.index;
    if (pr.s + 1e-6 < s_prev) monotonic = false;
    s_prev = pr.s;
  }
  CHECK(monotonic);
  CHECK(s_prev > 2 * M_PI * 30);
}

TEST(Path, ReferenceAcceleration) {
  std::vector<double> x, y, v;
  for (int i = 0; i <= 100; ++i) {
    x.push_back(i);
    y.push_back(0);
    v.push_back(std::sqrt(2.0 * 2.0 * i));  // constant 2 m/s^2 profile
  }
  const Path p(x, y, v);
  CHECK_NEAR(p[50].a_ref, 2.0, 1e-6);
}

TEST(PurePursuit, ZeroOnPath) {
  PurePursuit pp(PurePursuitConfig{}, 2.7);
  VehicleState s;
  s.x = 10;
  s.v = 10;
  CHECK_NEAR(pp.steer(straight(), s), 0.0, 1e-9);
}

TEST(PurePursuit, SteersTowardsPath) {
  PurePursuit pp(PurePursuitConfig{}, 2.7);
  VehicleState s;
  s.x = 10;
  s.y = 1.0;  // left of the path -> steer right (negative)
  s.v = 10;
  CHECK(pp.steer(straight(), s) < 0);
}

TEST(PurePursuit, CircleSteadyStateAngle) {
  PurePursuit pp(PurePursuitConfig{}, 2.7);
  VehicleState s;  // on the circle at the start, tangent heading
  s.v = 5;
  const double d = pp.steer(circle(40), s);
  CHECK_NEAR(d, std::atan(2.7 / 40.0), 0.01);
}

TEST(PurePursuit, LookaheadScheduledWithSpeed) {
  PurePursuit pp(PurePursuitConfig{}, 2.7);
  LateralDebug dbg;
  VehicleState s;
  s.x = 5;
  s.v = 20;
  pp.steer(straight(), s, &dbg);
  CHECK_NEAR(dbg.lookahead, 3.0 + 0.5 * 20, 1e-12);
  s.v = 100;
  pp.steer(straight(), s, &dbg);
  CHECK_NEAR(dbg.lookahead, 25.0, 1e-12);
}

TEST(Stanley, HeadingErrorTerm) {
  StanleyConfig c;
  c.k_understeer = 0.0;
  c.k_yaw_rate = 0;
  Stanley st(c, 2.7);
  VehicleState s;
  s.x = 10;
  s.yaw = -0.1;  // heading right of the path
  s.y = 2.7 * std::sin(0.1);  // front axle exactly on the path
  s.v = 10;
  CHECK_NEAR(st.steer(straight(), s), 0.1, 1e-6);
}

TEST(Stanley, CrossTrackTerm) {
  StanleyConfig c;
  c.k_understeer = 0.0;
  c.k_yaw_rate = 0;
  Stanley st(c, 2.7);
  VehicleState s;
  s.x = 10;
  s.y = 0.5;
  s.v = 9;
  CHECK_NEAR(st.steer(straight(), s), std::atan2(-1.5 * 0.5, 1.0 + 9.0), 1e-9);
}

TEST(Stanley, SteersIntoCurve) {
  StanleyConfig c;
  c.k_yaw_rate = 0;
  Stanley st(c, 2.7);
  VehicleState s;
  s.v = 10;
  const double d = st.steer(circle(40), s);
  // heading error to the tangent at the front-axle projection ~ atan(L/R)
  CHECK(d > 0.06 && d < 0.12);
}

TEST(Stanley, KinematicSteadyStateOnCircle) {
  // with the front axle on a circle and the heading tangent to the rear-axle
  // circle, the law returns the kinematic angle and no correction
  StanleyConfig c;
  c.k_yaw_rate = 0;
  c.k_understeer = 0;
  Stanley st(c, 2.7);
  const double R = 40.0, L = 2.7;
  const double rr = std::sqrt(R * R - L * L);  // rear-axle radius
  VehicleState s;
  s.x = 0;
  s.y = R - rr;  // rear axle on the inner circle, heading +x
  s.v = 10;
  CHECK_NEAR(st.steer(circle(R), s), std::atan(L / rr), 2e-3);
}

TEST_MAIN()
