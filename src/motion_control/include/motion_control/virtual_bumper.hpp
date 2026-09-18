#ifndef MOTION_CONTROL_VIRTUAL_BUMPER_HPP_
#define MOTION_CONTROL_VIRTUAL_BUMPER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "motion_control/obstacle_points.hpp"

namespace motion_control {

// D4.2 exit rule tolerance: a rollout that dips less than this below its start still counts as
// moving away.
constexpr double kExitRuleEpsilon = 1e-3;  // m

// D3.2. Euclidean distance from p to the outline placed at robot, 0 inside or on the edge.
inline double footprint_clearance(const Footprint& fp, const Pose2d& robot, const Point2d& p) {
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  const double dx = p.x - robot.x;
  const double dy = p.y - robot.y;
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  const double ex = std::max({-fp.back - lx, 0.0, lx - fp.front});
  const double ey = std::max(std::abs(ly) - fp.half_width, 0.0);
  return std::hypot(ex, ey);
}

// Nearest point to the outline; +inf when there are no points.
inline double min_clearance(const Footprint& fp, const Pose2d& robot,
                            const std::vector<Point2d>& pts) {
  double best = std::numeric_limits<double>::infinity();
  for (const Point2d& p : pts) {
    best = std::min(best, footprint_clearance(fp, robot, p));
  }
  return best;
}

// Exact differential-drive arc: straight line for w = 0, turn in place for v = 0.
inline Pose2d integrate_twist(const Pose2d& start, double v, double w, double t) {
  if (std::abs(w) < 1e-9) {
    return Pose2d{start.x + v * t * std::cos(start.yaw), start.y + v * t * std::sin(start.yaw),
                  start.yaw};
  }
  const double yaw = start.yaw + w * t;
  const double r = v / w;
  return Pose2d{start.x + r * (std::sin(yaw) - std::sin(start.yaw)),
                start.y - r * (std::cos(yaw) - std::cos(start.yaw)), yaw};
}

struct Rollout {
  double min_clearance;    // over every sample, t = 0 and the end included
  double start_clearance;  // at t = 0
  // Exit rule (D4.2): no sample drops below start - kExitRuleEpsilon and the end is farther than
  // the start.
  bool non_decreasing;
  Pose2d end;
};

// D3.2. Holds (v, w) for horizon_s from the current pose (origin of base_footprint) and samples
// the outline every step_s; the last sample lands exactly on horizon_s.
inline Rollout rollout_twist(const Footprint& fp, double v, double w, double horizon_s,
                             double step_s, const std::vector<Point2d>& pts) {
  const Pose2d origin{0.0, 0.0, 0.0};
  const double start = min_clearance(fp, origin, pts);
  Rollout out{start, start, true, origin};

  const int steps =
      (horizon_s > 0.0 && step_s > 0.0) ? static_cast<int>(std::ceil(horizon_s / step_s)) : 0;
  double last = start;
  for (int k = 1; k <= steps; ++k) {
    const double t = std::min(static_cast<double>(k) * step_s, horizon_s);
    const Pose2d pose = integrate_twist(origin, v, w, t);
    last = min_clearance(fp, pose, pts);
    out.min_clearance = std::min(out.min_clearance, last);
    if (last < start - kExitRuleEpsilon) {
      out.non_decreasing = false;
    }
    out.end = pose;
  }
  if (!(last > start)) {
    out.non_decreasing = false;
  }
  return out;
}

// D3.3. dynamic_inflation from the reference local_planner: the faster, the wider the bumper.
inline double bumper_margin(double speed_abs, double margin_min, double margin_max,
                            double speed_ref) {
  const double ratio =
      speed_ref > 0.0 ? std::clamp(std::abs(speed_abs) / speed_ref, 0.0, 1.0) : 1.0;
  return margin_min + (margin_max - margin_min) * ratio;
}

// D4.2. Admissible when the whole rollout keeps the margin, or, if the robot already sits inside
// the margin, when it moves away without touching anything.
inline bool rollout_admissible(const Rollout& r, double margin) {
  if (r.min_clearance >= margin) {
    return true;
  }
  return r.start_clearance < margin && r.non_decreasing && r.min_clearance > 0.0;
}

}  // namespace motion_control

#endif  // MOTION_CONTROL_VIRTUAL_BUMPER_HPP_
