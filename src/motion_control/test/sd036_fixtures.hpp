#ifndef MOTION_CONTROL_TEST_SD036_FIXTURES_HPP_
#define MOTION_CONTROL_TEST_SD036_FIXTURES_HPP_

// Shared fixtures of the SD036 tests: starting parameter values and scene helpers. The values
// mirror SD036 I4.1, I4.2, I4.3 and I5; the functions under test never carry defaults.

#include <algorithm>
#include <cmath>
#include <vector>

#include "motion_control/avoidance_planner.hpp"
#include "motion_control/follow_control.hpp"
#include "motion_control/guard_decision.hpp"
#include "motion_control/obstacle_points.hpp"
#include "motion_control/virtual_bumper.hpp"

namespace sd036_test {

using motion_control::AvoidanceParams;
using motion_control::FollowControlParams;
using motion_control::GuardParams;
using motion_control::Point2d;
using motion_control::Twist2d;

constexpr double kDt = 1.0 / 20.0;  // motion_control rate_hz = 20
constexpr double kSelfFilterPad = 0.02;
constexpr double kPersonExclusionRadius = 0.25;

// SD025 values as moved to motion_control.yaml (I4.1).
inline FollowControlParams follow_params() {
  FollowControlParams p{};
  p.standoff = 0.5;
  p.max_linear = 0.37;
  p.max_linear_follow = 0.25;
  p.max_angular = 2.0;
  p.kp_lin = 0.8;
  p.kp_ang = 1.5;
  p.dist_deadband = 0.05;
  p.min_breakaway_linear = 0.10;
  p.accel_linear = 0.30;
  p.decel_linear = 0.60;
  p.ang_deadband = 0.05;
  p.track_half_sum = 0.1407;
  return p;
}

// I4.2 and the outline of I4.3.
inline AvoidanceParams avoid_params() {
  AvoidanceParams a{};
  a.footprint = {0.175, 0.166, 0.123};
  a.margin_min = 0.05;
  a.margin_max = 0.12;
  a.margin_speed_ref = 0.25;
  a.influence_range = 0.25;
  a.horizon_s = 1.5;
  a.rollout_dt = 0.1;
  a.max_angular_avoid = 1.0;
  a.rotate_angular = 0.8;
  a.reverse_linear = 0.10;
  a.reverse_angular = 0.5;
  a.reverse_min_s = 0.8;
  a.free_hysteresis = 0.03;
  a.camera_half_fov = 0.645;
  a.fov_margin = 0.10;
  a.w_goal = 1.0;
  a.w_heading = 0.3;
  a.w_fov = 5.0;
  a.kp_repulse = 0.01;
  a.w_smooth = 0.2;
  a.arc_linear_samples = 4;
  a.arc_angular_samples = 9;
  return a;
}

// I5 and the outline of I4.3.
inline GuardParams guard_params() {
  GuardParams g{};
  g.footprint = {0.175, 0.166, 0.123};
  g.stop_margin = 0.02;
  g.stop_horizon_s = 0.5;
  g.rollout_dt = 0.05;
  g.hold_s = 0.3;
  return g;
}

inline std::vector<Point2d> circle(double cx, double cy, double r, int n = 24) {
  std::vector<Point2d> pts;
  for (int i = 0; i < n; ++i) {
    const double angle = 2.0 * M_PI * i / n;
    pts.push_back({cx + r * std::cos(angle), cy + r * std::sin(angle)});
  }
  return pts;
}

// Points every 1 cm from (x0, y0) to (x1, y1).
inline void segment(std::vector<Point2d>& pts, double x0, double y0, double x1, double y1) {
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::ceil(len / 0.01)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    pts.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t});
  }
}

inline bool is_zero(const Twist2d& t) { return t.linear_x == 0.0 && t.angular_z == 0.0; }

inline double margin_of(double v, const AvoidanceParams& a) {
  return motion_control::bumper_margin(std::abs(v), a.margin_min, a.margin_max, a.margin_speed_ref);
}

inline bool cmd_admissible(const Twist2d& cmd, double margin, const AvoidanceParams& a,
                           const std::vector<Point2d>& pts) {
  return motion_control::rollout_admissible(
      motion_control::rollout_twist(a.footprint, cmd.linear_x, cmd.angular_z, a.horizon_s,
                                    a.rollout_dt, pts),
      margin);
}

}  // namespace sd036_test

#endif  // MOTION_CONTROL_TEST_SD036_FIXTURES_HPP_
