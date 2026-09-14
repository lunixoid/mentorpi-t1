#ifndef MOTION_CONTROL_FOLLOW_CONTROL_HPP_
#define MOTION_CONTROL_FOLLOW_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_control {

// Wire values match mentorpi_msgs/msg/ControlState.
constexpr uint8_t kControlForbidden = 0;
constexpr uint8_t kControlManual = 1;
constexpr uint8_t kControlAutoFollow = 2;

struct FollowControlParams {
  double standoff;              // m, target distance
  double max_linear;            // m/s, platform path speed ceiling
  double max_linear_follow;     // m/s, follow-channel ceiling
  double max_angular;           // rad/s
  double kp_lin;                // 1/s, profile slope (D2.1)
  double kp_ang;                // 1/s
  double dist_deadband;         // m, half-corridor around standoff
  double min_breakaway_linear;  // m/s, breakaway command, from standstill only (D2.3)
  double accel_linear;          // m/s^2, rate limit going up (D2.4)
  double decel_linear;          // m/s^2, rate limit going down (D2.4)
  double ang_deadband;          // rad
  double track_half_sum;        // m
};

struct Twist2d {
  double linear_x;
  double angular_z;
};

// Rate limit toward target: up by accel*dt, down by decel*dt, landing exactly on target
// when the residual is smaller than the step. Exact landing is what lets the standstill
// predicate in compute_follow_twist see a true zero. dt <= 0 leaves the command alone.
inline double apply_rate_limit(double prev, double target, double dt, double accel, double decel) {
  if (!(dt > 0.0)) {
    return prev;
  }
  const double delta = target - prev;
  const double step = (delta > 0.0 ? accel : decel) * dt;
  if (std::abs(delta) <= step) {
    return target;
  }
  return prev + std::copysign(step, delta);
}

// Pure follow law: range = hypot(x,y), bearing = atan2(y,x). No ROS, no defaults (I4 lives in
// the node), no state — prev_linear is the last emitted linear command, dt the tick period.
inline Twist2d compute_follow_twist(double x, double y, double prev_linear, double dt,
                                    const FollowControlParams& params) {
  const double range = std::hypot(x, y);
  const double bearing = std::atan2(y, x);
  const double err = range - params.standoff;
  const double ceiling = std::min(params.max_linear_follow, params.max_linear);

  // D2.1. Profile: approach only. Inside the corridor and nearer, the target is zero —
  // the robot waits instead of backing off.
  double target = 0.0;
  if (err > params.dist_deadband) {
    target = std::clamp(params.kp_lin * err, 0.0, ceiling);
  }

  // D2.2. Yaw takes its share of the ceiling first, speed gets the remainder. Reversed against
  // SD020 on purpose: budgeting yaw from the leftover speed would zero the turn exactly when
  // the robot reaches the follow ceiling.
  double angular_des = 0.0;
  if (std::abs(bearing) >= params.ang_deadband) {
    angular_des = std::clamp(params.kp_ang * bearing, -params.max_angular, params.max_angular);
  }
  const double linear_cap =
      std::clamp(ceiling - std::abs(angular_des) * params.track_half_sum, 0.0, ceiling);
  target = std::min(target, linear_cap);

  // D2.3. Breakaway only from standstill: once rolling, the robot is free to decay below it
  // down to zero, which is what makes the approach to the corridor smooth.
  if (target > 0.0 && prev_linear == 0.0) {
    target = std::min(std::max(target, params.min_breakaway_linear), ceiling);
  }

  // D2.4. Rate limit: acceleration and normal deceleration are spread over time.
  double linear_x =
      apply_rate_limit(prev_linear, target, dt, params.accel_linear, params.decel_linear);

  // D2.3, second half: the ramp must not swallow the breakaway. One accel step out of standstill
  // is far below the command the tracks actually move at, so the robot would sit buzzing for
  // several ticks. Starting from a standstill the first command therefore steps straight to the
  // breakaway value; the ramp shapes everything above it.
  if (target > 0.0 && prev_linear == 0.0) {
    linear_x = std::max(linear_x, std::min(params.min_breakaway_linear, ceiling));
  }

  // D2.5. Yaw only while moving — no turning in place. The final clamp keeps the fast track
  // inside the ceiling while the ramp is still catching up with a freshly lowered cap; in
  // steady state linear_x equals linear_cap, so omega_lim equals |angular_des| and nothing
  // is lost.
  double angular_z = 0.0;
  if (linear_x > 0.0) {
    const double omega_lim =
        std::clamp((ceiling - linear_x) / params.track_half_sum, 0.0, params.max_angular);
    angular_z = std::clamp(angular_des, -omega_lim, omega_lim);
  }

  return Twist2d{linear_x, angular_z};
}

}  // namespace motion_control

#endif  // MOTION_CONTROL_FOLLOW_CONTROL_HPP_
