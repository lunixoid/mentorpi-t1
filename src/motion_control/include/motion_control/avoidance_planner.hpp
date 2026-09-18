#ifndef MOTION_CONTROL_AVOIDANCE_PLANNER_HPP_
#define MOTION_CONTROL_AVOIDANCE_PLANNER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "motion_control/follow_control.hpp"
#include "motion_control/obstacle_points.hpp"
#include "motion_control/virtual_bumper.hpp"

namespace motion_control {

// Wire values match mentorpi_msgs/msg/ObstacleAvoidanceStatus (SD036 I1.1).
enum class AvoidState : uint8_t { kFree = 1, kAvoiding = 2, kBlocked = 3 };
enum class Maneuver : uint8_t { kNone = 0, kArc = 1, kRotate = 2, kReverse = 3 };

// SD036 I4.2. No defaults here: values live in the node YAML, tests pass their own fixture.
struct AvoidanceParams {
  Footprint footprint;
  double margin_min;         // m, bumper when standing (D3.3)
  double margin_max;         // m, bumper at margin_speed_ref and above
  double margin_speed_ref;   // m/s
  double influence_range;    // m, R of the repulsive term (D4.3)
  double horizon_s;          // s, rollout length
  double rollout_dt;         // s, rollout sampling step
  double max_angular_avoid;  // rad/s, arc yaw sampling range
  double rotate_angular;     // rad/s, turn in place
  double reverse_linear;     // m/s, magnitude of the reverse command
  double reverse_angular;    // rad/s, yaw options while reversing
  double reverse_min_s;      // s, reverse latch (D4.4)
  double free_hysteresis;    // m, extra clearance to leave avoidance (D4.1)
  double camera_half_fov;    // rad
  double fov_margin;         // rad
  double w_goal;             // 1/m
  double w_heading;          // 1/rad
  double w_fov;              // 1/rad
  double kp_repulse;         // m^2, KP of 0.5*KP*(1/d - 1/R)^2
  double w_smooth;
  int arc_linear_samples;
  int arc_angular_samples;  // odd, so that w = 0 is sampled
};

struct AvoidanceMemory {
  AvoidState state{AvoidState::kFree};
  Maneuver maneuver{Maneuver::kNone};
  int rotate_sign{0};
  double reverse_elapsed_s{0.0};
  Twist2d prev_cmd{0.0, 0.0};
  double prev_linear{0.0};  // ramp state, never negative: reverse and turns reset it to zero
};

struct AvoidanceResult {
  Twist2d cmd;
  AvoidState state;
  Maneuver maneuver;
  double min_clearance;  // m, outline to the nearest point at the current pose; +inf without points
  double bumper_margin;  // m, margin for the emitted |linear_x|
};

inline void reset_avoidance(AvoidanceMemory& memory) { memory = AvoidanceMemory{}; }

namespace detail {

constexpr double kRepulseFloor = 1e-3;  // m, keeps 1/d finite on contact
constexpr double kScoreTie = 1e-12;

inline double wrap_angle(double a) { return std::remainder(a, 2.0 * M_PI); }

// D4.3. Lower is better. The target is taken as standing still over the horizon.
inline double score_candidate(const Twist2d& cmd, const Rollout& r, double target_x,
                              double target_y, double ceiling, const FollowControlParams& follow,
                              const AvoidanceParams& avoid, const Twist2d& prev) {
  const double dx = target_x - r.end.x;
  const double dy = target_y - r.end.y;
  const double bearing = std::abs(wrap_angle(std::atan2(dy, dx) - r.end.yaw));

  double j = avoid.w_goal * std::max(0.0, std::hypot(dx, dy) - follow.standoff);
  j += avoid.w_heading * bearing;
  j += avoid.w_fov * std::max(0.0, bearing - (avoid.camera_half_fov - avoid.fov_margin));
  if (r.min_clearance < avoid.influence_range) {
    // Repulsion of the reference local_planner: prefers the arc that passes farther away.
    const double inv = 1.0 / std::max(r.min_clearance, kRepulseFloor) - 1.0 / avoid.influence_range;
    j += 0.5 * avoid.kp_repulse * inv * inv;
  }
  j += avoid.w_smooth * (std::abs(cmd.linear_x - prev.linear_x) / ceiling +
                         std::abs(cmd.angular_z - prev.angular_z) / follow.max_angular);
  return j;
}

struct Best {
  bool found{false};
  Twist2d cmd{0.0, 0.0};
  double score{std::numeric_limits<double>::infinity()};
};

// Admissibility first (D4.2), then the lowest score; a tie goes to the smaller |w|.
inline void consider(Best& best, const Twist2d& cmd, double margin, double target_x,
                     double target_y, double ceiling, const FollowControlParams& follow,
                     const AvoidanceParams& avoid, const Twist2d& prev,
                     const std::vector<Point2d>& points) {
  const Rollout r = rollout_twist(avoid.footprint, cmd.linear_x, cmd.angular_z, avoid.horizon_s,
                                  avoid.rollout_dt, points);
  if (!rollout_admissible(r, margin)) {
    return;
  }
  const double score = score_candidate(cmd, r, target_x, target_y, ceiling, follow, avoid, prev);
  const bool better = !best.found || score < best.score - kScoreTie ||
                      (std::abs(score - best.score) <= kScoreTie &&
                       std::abs(cmd.angular_z) < std::abs(best.cmd.angular_z));
  if (better) {
    best.found = true;
    best.cmd = cmd;
    best.score = score;
  }
}

// D4.5. Arc linear: speeding up follows the SD025 ramp and breakaway-from-standstill rule,
// slowing down to the candidate is immediate. With the decel ramp an arc at follow speed could
// neither slow down nor turn enough to clear an object, and the ladder fell to a turn in place.
// The candidate is scored after shaping, so the rollout checks exactly the command that goes out.
inline double shape_arc_linear(double prev_linear, double target, double dt, double ceiling,
                               const FollowControlParams& follow) {
  if (target <= prev_linear) {
    return target;
  }
  double v = apply_rate_limit(prev_linear, target, dt, follow.accel_linear, follow.accel_linear);
  if (prev_linear == 0.0) {
    v = std::max(v, std::min(follow.min_breakaway_linear, ceiling));
  }
  return v;
}

inline bool admissible(const Twist2d& cmd, double margin, const AvoidanceParams& avoid,
                       const std::vector<Point2d>& points) {
  return rollout_admissible(rollout_twist(avoid.footprint, cmd.linear_x, cmd.angular_z,
                                          avoid.horizon_s, avoid.rollout_dt, points),
                            margin);
}

}  // namespace detail

// SD036 D4. Pure planner with explicit memory. Called only while the follow gate is open; the
// caller resets the memory when the gate closes or stop_request is set. points are obstacle
// points in base_footprint after the self filter and person exclusion.
inline AvoidanceResult plan_avoidance(double target_x, double target_y,
                                      const std::vector<Point2d>& points, double dt,
                                      const FollowControlParams& follow,
                                      const AvoidanceParams& avoid, AvoidanceMemory& memory) {
  const double ceiling = std::min(follow.max_linear_follow, follow.max_linear);
  const double clearance_now = min_clearance(avoid.footprint, Pose2d{0.0, 0.0, 0.0}, points);
  const auto margin_for = [&avoid](double v) {
    return bumper_margin(std::abs(v), avoid.margin_min, avoid.margin_max, avoid.margin_speed_ref);
  };
  const auto finish = [&](Twist2d cmd, AvoidState state, Maneuver maneuver) {
    memory.state = state;
    memory.maneuver = maneuver;
    memory.prev_cmd = cmd;
    return AvoidanceResult{cmd, state, maneuver, clearance_now, margin_for(cmd.linear_x)};
  };
  const Twist2d prev_cmd = memory.prev_cmd;

  // D4.4. Reverse latch: back off for at least reverse_min_s while it stays admissible.
  if (memory.maneuver == Maneuver::kReverse && memory.reverse_elapsed_s < avoid.reverse_min_s &&
      detail::admissible(prev_cmd, margin_for(prev_cmd.linear_x), avoid, points)) {
    memory.reverse_elapsed_s += dt;
    memory.rotate_sign = 0;
    memory.prev_linear = 0.0;
    return finish(prev_cmd, AvoidState::kAvoiding, Maneuver::kReverse);
  }

  // D4.1. The SD025 command as is when it passes the bumper; standing is always allowed. Leaving
  // avoidance needs free_hysteresis on top of the margin.
  const Twist2d nominal = compute_follow_twist(target_x, target_y, memory.prev_linear, dt, follow);
  const bool standing = nominal.linear_x == 0.0 && nominal.angular_z == 0.0;
  const double nominal_margin = margin_for(nominal.linear_x) +
                                (memory.state == AvoidState::kFree ? 0.0 : avoid.free_hysteresis);
  if (standing || detail::admissible(nominal, nominal_margin, avoid, points)) {
    memory.rotate_sign = 0;
    memory.reverse_elapsed_s = 0.0;
    memory.prev_linear = nominal.linear_x;
    return finish(nominal, AvoidState::kFree, Maneuver::kNone);
  }

  // D4.2 step 1. Forward arcs, fast track under the follow ceiling.
  detail::Best best;
  const int nv = std::max(1, avoid.arc_linear_samples);
  const int nw = std::max(1, avoid.arc_angular_samples);
  const double v_low = std::min(follow.min_breakaway_linear, ceiling);
  for (int i = 0; i < nv; ++i) {
    const double v_target = nv == 1 ? ceiling : v_low + (ceiling - v_low) * i / (nv - 1);
    const double v_out =
        detail::shape_arc_linear(memory.prev_linear, v_target, dt, ceiling, follow);
    if (!(v_target > 0.0) || !(v_out > 0.0)) {
      continue;
    }
    const double w_lim =
        std::min(follow.max_angular, std::max(0.0, (ceiling - v_out) / follow.track_half_sum));
    for (int j = 0; j < nw; ++j) {
      const double w_target =
          nw == 1 ? 0.0 : -avoid.max_angular_avoid + 2.0 * avoid.max_angular_avoid * j / (nw - 1);
      // Keep the curvature of the sample at the shaped speed.
      const double w_out = std::clamp(w_target / v_target * v_out, -w_lim, w_lim);
      detail::consider(best, Twist2d{v_out, w_out}, margin_for(v_out), target_x, target_y, ceiling,
                       follow, avoid, prev_cmd, points);
    }
  }
  if (best.found) {
    memory.rotate_sign = 0;
    memory.reverse_elapsed_s = 0.0;
    memory.prev_linear = best.cmd.linear_x;
    return finish(best.cmd, AvoidState::kAvoiding, Maneuver::kArc);
  }

  // D4.2 step 2. Turn in place; the sign chosen on entry holds while it stays admissible (D4.4).
  const double w_rotate = std::min(avoid.rotate_angular, follow.max_angular);
  const Twist2d latched{0.0, memory.rotate_sign * w_rotate};
  if (memory.maneuver == Maneuver::kRotate && memory.rotate_sign != 0 &&
      detail::admissible(latched, margin_for(0.0), avoid, points)) {
    best.found = true;
    best.cmd = latched;
  } else {
    for (const double w : {w_rotate, -w_rotate}) {
      detail::consider(best, Twist2d{0.0, w}, margin_for(0.0), target_x, target_y, ceiling, follow,
                       avoid, prev_cmd, points);
    }
  }
  if (best.found) {
    memory.rotate_sign = best.cmd.angular_z > 0.0 ? 1 : -1;
    memory.reverse_elapsed_s = 0.0;
    memory.prev_linear = 0.0;
    return finish(best.cmd, AvoidState::kAvoiding, Maneuver::kRotate);
  }

  // D4.2 step 3. Back off.
  for (const double w : {0.0, avoid.reverse_angular, -avoid.reverse_angular}) {
    detail::consider(best, Twist2d{-avoid.reverse_linear, w}, margin_for(avoid.reverse_linear),
                     target_x, target_y, ceiling, follow, avoid, prev_cmd, points);
  }
  if (best.found) {
    memory.rotate_sign = 0;
    memory.reverse_elapsed_s = dt;
    memory.prev_linear = 0.0;
    return finish(best.cmd, AvoidState::kAvoiding, Maneuver::kReverse);
  }

  // D4.2 step 4. Every maneuver hits the bumper.
  memory.rotate_sign = 0;
  memory.reverse_elapsed_s = 0.0;
  memory.prev_linear = 0.0;
  return finish(Twist2d{0.0, 0.0}, AvoidState::kBlocked, Maneuver::kNone);
}

}  // namespace motion_control

#endif  // MOTION_CONTROL_AVOIDANCE_PLANNER_HPP_
