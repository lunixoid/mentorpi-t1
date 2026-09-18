#ifndef MOTION_CONTROL_GUARD_DECISION_HPP_
#define MOTION_CONTROL_GUARD_DECISION_HPP_

#include <cstdint>
#include <vector>

#include "motion_control/follow_control.hpp"
#include "motion_control/obstacle_points.hpp"
#include "motion_control/virtual_bumper.hpp"

namespace motion_control {

// SD036 I2.2: MotionRestriction.reason values.
inline constexpr const char* kReasonNone = "";
inline constexpr const char* kReasonObstacle = "obstacle";
inline constexpr const char* kReasonNoTarget = "no_target";
inline constexpr const char* kReasonNoCommand = "no_command";
inline constexpr const char* kReasonNoScan = "no_scan";

struct GuardInputs {
  bool have_state;
  uint8_t state;       // mentorpi_msgs/ControlState wire value
  bool target_usable;  // valid, not coasting, fresh — the motion_control gate
  bool command_fresh;  // /pnc/desired_twist within command_timeout_ms
  Twist2d command;
  bool scan_fresh;                     // fresh scan and a lidar TF
  const std::vector<Point2d>* points;  // after self filter and person exclusion
  double now_s;
};

// SD036 I5. stop_margin < margin_min and stop_horizon_s <= horizon_s of the planner (D6.2).
struct GuardParams {
  Footprint footprint;
  double stop_margin;     // m
  double stop_horizon_s;  // s
  double rollout_dt;      // s
  double hold_s;          // s, obstacle stop hold
};

struct GuardMemory {
  bool have_obstacle{false};
  double last_obstacle_s{0.0};
};

struct GuardDecision {
  bool stop_request;
  const char* reason;
};

// SD036 D6.1. F13/F14 decision, checked in this order. Only an obstacle stop is held for hold_s;
// the other reasons clear as soon as their cause does, and leaving AUTO_FOLLOW clears everything.
inline GuardDecision decide_guard(const GuardInputs& in, const GuardParams& p, GuardMemory& m) {
  if (!in.have_state || in.state != kControlAutoFollow) {
    m.have_obstacle = false;
    return {false, kReasonNone};
  }
  if (!in.target_usable) {
    return {true, kReasonNoTarget};
  }
  if (!in.command_fresh) {
    return {true, kReasonNoCommand};
  }
  if (!in.scan_fresh || in.points == nullptr) {
    return {false, kReasonNoScan};
  }

  // Standing is safe, a zero command is not rolled out. A hit is a rollout that gets closer than
  // stop_margin while closing in: backing away from an object already that close stays allowed,
  // the same exit rule the planner uses.
  const bool moving = in.command.linear_x != 0.0 || in.command.angular_z != 0.0;
  if (moving) {
    const Rollout r = rollout_twist(p.footprint, in.command.linear_x, in.command.angular_z,
                                    p.stop_horizon_s, p.rollout_dt, *in.points);
    if (r.min_clearance < p.stop_margin && r.min_clearance < r.start_clearance - kExitRuleEpsilon) {
      m.have_obstacle = true;
      m.last_obstacle_s = in.now_s;
      return {true, kReasonObstacle};
    }
  }
  if (m.have_obstacle && in.now_s - m.last_obstacle_s < p.hold_s) {
    return {true, kReasonObstacle};
  }
  m.have_obstacle = false;
  return {false, kReasonNone};
}

}  // namespace motion_control

#endif  // MOTION_CONTROL_GUARD_DECISION_HPP_
