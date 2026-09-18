#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "motion_control/avoidance_planner.hpp"
#include "sd036_fixtures.hpp"

using motion_control::AvoidanceMemory;
using motion_control::AvoidanceParams;
using motion_control::AvoidanceResult;
using motion_control::AvoidState;
using motion_control::compute_follow_twist;
using motion_control::drop_self_points;
using motion_control::FollowControlParams;
using motion_control::Maneuver;
using motion_control::plan_avoidance;
using motion_control::Point2d;
using motion_control::Twist2d;
using sd036_test::circle;
using sd036_test::cmd_admissible;
using sd036_test::is_zero;
using sd036_test::kDt;
using sd036_test::margin_of;
using sd036_test::segment;

namespace {

int g_fails = 0;

void expect(bool cond, const char* what) {
  if (!cond) {
    std::cerr << "FAIL: " << what << '\n';
    ++g_fails;
  }
}

// AC1. No obstacle points: bit-for-bit the SD025 law, tick after tick; standing next to a wall
// is FREE and zero.
void test_free_is_sd025() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  bool identical = true;
  bool all_free = true;
  for (const double x : {-1.0, 0.3, 0.5, 0.55, 1.0, 2.0, 4.0}) {
    for (const double y : {-1.0, -0.3, 0.0, 0.3, 1.0}) {
      AvoidanceMemory m;
      double prev = 0.0;
      for (int tick = 0; tick < 60; ++tick) {
        const Twist2d law = compute_follow_twist(x, y, prev, kDt, f);
        prev = law.linear_x;
        const AvoidanceResult r = plan_avoidance(x, y, {}, kDt, f, a, m);
        identical = identical && r.cmd.linear_x == law.linear_x && r.cmd.angular_z == law.angular_z;
        all_free = all_free && r.state == AvoidState::kFree && r.maneuver == Maneuver::kNone;
      }
    }
  }
  expect(identical, "T2 AC1 without points the command equals compute_follow_twist bit for bit");
  expect(all_free, "T2 AC1 without points the state is FREE");

  std::vector<Point2d> wall;
  segment(wall, -0.3, 0.15, 0.3, 0.15);  // 0.027 m from the side, inside margin_min
  AvoidanceMemory m;
  const AvoidanceResult r = plan_avoidance(f.standoff, 0.0, wall, kDt, f, a, m);
  expect(is_zero(r.cmd) && r.state == AvoidState::kFree,
         "T2 AC1 target in the corridor next to a wall: zero and FREE");
}

// AC2 and AC6 (second half). Obstacle ahead, sides free: an arc that keeps the bumper, away from
// the side the obstacle leans to. The obstacle sits where the SD025 command from standstill no
// longer passes the bumper but an arc still does (0.40-0.42 m with the stand-tuned margins).
void test_arc() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();

  AvoidanceMemory m;
  const auto ahead = circle(0.41, 0.0, 0.05);
  const AvoidanceResult r = plan_avoidance(2.0, 0.0, ahead, kDt, f, a, m);
  expect(r.state == AvoidState::kAvoiding && r.maneuver == Maneuver::kArc,
         "T2 AC2 obstacle ahead: AVOIDING / ARC");
  expect(r.cmd.linear_x > 0.0 && r.cmd.angular_z != 0.0, "T2 AC2 arc drives forward and turns");
  expect(cmd_admissible(r.cmd, margin_of(r.cmd.linear_x, a), a, ahead),
         "T2 AC2 the chosen arc keeps margin(v)");

  // D4.5 as changed on T2: at follow speed the arc slows down at once instead of falling back to a
  // turn in place.
  AvoidanceMemory moving;
  moving.prev_linear = 0.25;
  moving.prev_cmd = {0.25, 0.0};
  const AvoidanceResult fast = plan_avoidance(2.0, 0.0, ahead, kDt, f, a, moving);
  expect(fast.maneuver == Maneuver::kArc && fast.cmd.linear_x < 0.25 - f.decel_linear * kDt,
         "T2 AC2 at follow speed the arc slows down below one decel step and still turns");
  const AvoidanceResult ramp = plan_avoidance(2.0, 0.0, {}, kDt, f, a, moving);
  expect(ramp.cmd.linear_x <= fast.cmd.linear_x + f.accel_linear * kDt + 1e-12,
         "T2 AC2 speeding up after the arc follows the accel ramp");

  AvoidanceMemory right_m;
  const auto right = circle(0.41, -0.08, 0.05);
  const AvoidanceResult rr = plan_avoidance(2.0, 0.0, right, kDt, f, a, right_m);
  expect(rr.maneuver == Maneuver::kArc && rr.cmd.angular_z > 0.0,
         "T2 AC6 obstacle leaning right: the arc goes left, farther from it");

  AvoidanceMemory left_m;
  const auto left = circle(0.41, 0.08, 0.05);
  const AvoidanceResult rl = plan_avoidance(2.0, 0.0, left, kDt, f, a, left_m);
  expect(rl.maneuver == Maneuver::kArc && rl.cmd.angular_z < 0.0,
         "T2 AC6 obstacle leaning left: the arc goes right");
}

// AC6 (first half). Only the field-of-view term differs: the arc that keeps the target in frame
// wins.
void test_fov_preference() {
  const FollowControlParams f = sd036_test::follow_params();
  AvoidanceParams a = sd036_test::avoid_params();
  a.w_goal = 0.0;
  a.w_heading = 0.0;
  a.kp_repulse = 0.0;
  a.w_smooth = 0.0;

  // Target 0.45 rad to the left, obstacle dead ahead.
  const double tx = 1.5 * std::cos(0.45);
  const double ty = 1.5 * std::sin(0.45);
  const auto ahead = circle(0.41, 0.0, 0.05);
  AvoidanceMemory m;
  const AvoidanceResult r = plan_avoidance(tx, ty, ahead, kDt, f, a, m);
  expect(r.maneuver == Maneuver::kArc && r.cmd.angular_z > 0.0,
         "T2 AC6 with only the FOV term the arc turns toward the target side");
}

// AC3. Wall ahead, arcs hit it, turning in place clears it: ROTATE, and the sign holds.
void test_rotate_latch() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  std::vector<Point2d> wall;
  segment(wall, a.footprint.front + 0.10, -0.6, a.footprint.front + 0.10, 0.6);

  AvoidanceMemory m;
  // Target to the right: turning clockwise brings it closer to the heading.
  const AvoidanceResult first = plan_avoidance(2.0, -0.5, wall, kDt, f, a, m);
  expect(first.state == AvoidState::kAvoiding && first.maneuver == Maneuver::kRotate,
         "T2 AC3 arcs blocked, turn possible: ROTATE");
  expect(first.cmd.linear_x == 0.0 && std::abs(first.cmd.angular_z) == a.rotate_angular,
         "T2 AC3 turn in place at rotate_angular");
  expect(first.cmd.angular_z < 0.0, "T2 AC3 entry sign follows the target side");

  // Target moves to the left: without the latch the turn would flip.
  bool held = true;
  for (int tick = 0; tick < 5; ++tick) {
    const AvoidanceResult next = plan_avoidance(2.0, 0.5, wall, kDt, f, a, m);
    held = held && next.maneuver == Maneuver::kRotate && next.cmd.angular_z < 0.0;
  }
  expect(held, "T2 AC3 the rotation sign holds while no arc is admissible");

  const AvoidanceResult open = plan_avoidance(2.0, 0.5, {}, kDt, f, a, m);
  expect(open.maneuver != Maneuver::kRotate, "T2 AC3 the latch releases once the way is open");
}

// AC4. Ahead and turning blocked, back free: REVERSE, held for reverse_min_s, dropped at once
// when it stops being admissible.
void test_reverse_latch() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  const auto& fp = a.footprint;
  std::vector<Point2d> pocket;
  segment(pocket, fp.front + 0.06, -0.5, fp.front + 0.06, 0.5);  // ahead, 0.06 m
  segment(pocket, -0.8, fp.half_width + 0.12, fp.front + 0.06, fp.half_width + 0.12);
  segment(pocket, -0.8, -(fp.half_width + 0.12), fp.front + 0.06, -(fp.half_width + 0.12));

  AvoidanceMemory m;
  m.prev_linear = 0.2;
  const AvoidanceResult first = plan_avoidance(2.0, 0.0, pocket, kDt, f, a, m);
  expect(first.state == AvoidState::kAvoiding && first.maneuver == Maneuver::kReverse,
         "T2 AC4 ahead and turning blocked: REVERSE");
  expect(first.cmd.linear_x == -a.reverse_linear, "T2 AC4 reverse at -reverse_linear");
  expect(m.prev_linear == 0.0, "T2 AC4 reversing resets the ramp");

  // Way ahead opens: the latch still backs off for reverse_min_s.
  int reverse_ticks = 1;
  AvoidanceResult r = first;
  for (int tick = 0; tick < 40 && r.maneuver == Maneuver::kReverse; ++tick) {
    r = plan_avoidance(2.0, 0.0, {}, kDt, f, a, m);
    if (r.maneuver == Maneuver::kReverse) {
      ++reverse_ticks;
    }
  }
  const double held_s = reverse_ticks * kDt;
  expect(held_s >= a.reverse_min_s - 1e-9 && held_s <= a.reverse_min_s + 2.0 * kDt,
         "T2 AC4 reverse holds for reverse_min_s, not longer");
  expect(r.state == AvoidState::kFree, "T2 AC4 after the latch the free path is followed");

  // A wall right behind: the latched reverse is no longer admissible and is dropped at once.
  AvoidanceMemory m2;
  plan_avoidance(2.0, 0.0, pocket, kDt, f, a, m2);
  std::vector<Point2d> closed = pocket;
  segment(closed, -(fp.back + 0.03), -0.5, -(fp.back + 0.03), 0.5);
  const AvoidanceResult dropped = plan_avoidance(2.0, 0.0, closed, kDt, f, a, m2);
  expect(dropped.maneuver != Maneuver::kReverse,
         "T2 AC4 an inadmissible reverse is not held by the latch");
}

// AC5. Boxed in on all sides: BLOCKED, exact zero, ramp reset.
void test_blocked() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  const auto& fp = a.footprint;
  const double gap = 0.03;
  const double xf = fp.front + gap;
  const double xb = -(fp.back + gap);
  const double ys = fp.half_width + gap;
  std::vector<Point2d> box;
  segment(box, xf, -ys, xf, ys);
  segment(box, xb, -ys, xb, ys);
  segment(box, xb, ys, xf, ys);
  segment(box, xb, -ys, xf, -ys);

  AvoidanceMemory m;
  m.prev_linear = 0.2;
  m.prev_cmd = {0.2, 0.0};
  const AvoidanceResult r = plan_avoidance(2.0, 0.0, box, kDt, f, a, m);
  expect(r.state == AvoidState::kBlocked && r.maneuver == Maneuver::kNone,
         "T2 AC5 boxed in: BLOCKED");
  expect(is_zero(r.cmd), "T2 AC5 command is exactly zero");
  expect(m.prev_linear == 0.0, "T2 AC5 ramp is reset");
}

// AC7. Invariants on random scenes, several ticks per scene so the memory evolves.
void test_invariants() {
  const FollowControlParams f = sd036_test::follow_params();
  const AvoidanceParams a = sd036_test::avoid_params();
  const double ceiling = std::min(f.max_linear_follow, f.max_linear);
  const double eps = 1e-9;
  std::mt19937 rng(36);
  std::uniform_real_distribution<double> coord(-1.5, 1.5);
  std::uniform_real_distribution<double> target_x(0.2, 3.0);
  std::uniform_int_distribution<int> count(0, 60);

  bool angular_ok = true;
  bool arc_ok = true;
  bool reverse_only = true;
  bool rotate_only = true;
  bool blocked_zero = true;
  bool free_is_law = true;
  bool hysteresis_ok = true;
  bool maneuver_admissible = true;
  int avoiding_seen = 0;

  for (int scene = 0; scene < 1000; ++scene) {
    std::vector<Point2d> pts;
    const int n = count(rng);
    for (int i = 0; i < n; ++i) {
      pts.push_back({coord(rng), coord(rng)});
    }
    drop_self_points(pts, a.footprint, 0.02);
    const double tx = target_x(rng);
    const double ty = coord(rng);

    AvoidanceMemory m;
    for (int tick = 0; tick < 5; ++tick) {
      const AvoidState prev_state = m.state;
      const Twist2d law = compute_follow_twist(tx, ty, m.prev_linear, kDt, f);
      const AvoidanceResult r = plan_avoidance(tx, ty, pts, kDt, f, a, m);
      const Twist2d& c = r.cmd;

      angular_ok = angular_ok && std::abs(c.angular_z) <= f.max_angular + eps;
      if (r.maneuver == Maneuver::kArc) {
        arc_ok = arc_ok && c.linear_x > 0.0 &&
                 c.linear_x + std::abs(c.angular_z) * f.track_half_sum <= ceiling + eps;
      }
      if (c.linear_x < 0.0) {
        reverse_only = reverse_only && r.maneuver == Maneuver::kReverse;
      }
      if (c.linear_x == 0.0 && c.angular_z != 0.0) {
        rotate_only = rotate_only && r.maneuver == Maneuver::kRotate;
      }
      if (r.state == AvoidState::kBlocked) {
        blocked_zero = blocked_zero && is_zero(c);
      }
      if (r.state == AvoidState::kFree) {
        free_is_law = free_is_law && c.linear_x == law.linear_x && c.angular_z == law.angular_z;
        if (prev_state != AvoidState::kFree && !is_zero(c)) {
          hysteresis_ok = hysteresis_ok &&
                          cmd_admissible(c, margin_of(c.linear_x, a) + a.free_hysteresis, a, pts);
        }
      }
      if (r.state == AvoidState::kAvoiding) {
        ++avoiding_seen;
        maneuver_admissible =
            maneuver_admissible && cmd_admissible(c, margin_of(c.linear_x, a), a, pts);
      }
    }
  }
  expect(angular_ok, "T2 AC7 |w| <= max_angular");
  expect(arc_ok, "T2 AC7 arcs drive forward with the fast track under the ceiling");
  expect(reverse_only, "T2 AC7 negative linear only in REVERSE");
  expect(rotate_only, "T2 AC7 turn in place only in ROTATE");
  expect(blocked_zero, "T2 AC7 BLOCKED is exactly zero");
  expect(free_is_law, "T2 AC7 FREE is the SD025 law");
  expect(hysteresis_ok, "T2 AC7 back to FREE only with free_hysteresis");
  expect(maneuver_admissible, "T2 AC7 every emitted maneuver keeps its bumper");
  expect(avoiding_seen > 100, "T2 AC7 random scenes actually exercise avoidance");
}

}  // namespace

int main() {
  test_free_is_sd025();
  test_arc();
  test_fov_preference();
  test_rotate_latch();
  test_reverse_latch();
  test_blocked();
  test_invariants();
  if (g_fails != 0) {
    std::cerr << g_fails << " check(s) failed\n";
    return 1;
  }
  std::cout << "test_avoidance_planner: ok\n";
  return 0;
}
